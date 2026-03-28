//! BLE Central state machine and CYW43 initialization
//!
//! This module orchestrates all BLE functionality running on Core 0:
//! - CYW43 WiFi/BLE chip initialization (firmware download, PIO SPI setup)
//! - BLE stack construction with bond loading
//! - Command-driven state machine that handles scan, connect, and management commands
//!
//! Sub-modules handle specific phases of the BLE connection lifecycle:
//! - [`connection`]: BLE connection establishment and pairing
//! - [`gatt`]: GATT service discovery and HID notification loop
//! - [`commands`]: Shared command handlers (used by both idle and connected states)

pub mod commands;
pub mod connection;
pub mod gatt;
pub mod slots;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_futures::select::{select, Either};
use embassy_rp::dma;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use trouble_host::prelude::*;
use trouble_host::scan::Scanner;

use crate::ble_state::{
    BleCommand, BleEvent, RpcScannerHandler, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL,
};
use crate::bonding;
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::protocol::ConnectionState;
use crate::rpc_log;
use crate::{Irqs, FLASH_SIZE};

use self::slots::{
    ConnectRequest, SlotCommand, CONNECT_SIGNALS, MAX_CONNECTIONS, SLOT_CMD_CHANNELS,
};

/// Max BLE connections
const CONNECTIONS_MAX: usize = 3;
/// Max L2CAP channels (3 per connection: signal + att + coc)
const L2CAP_CHANNELS_MAX: usize = 9;

/// Type alias for the flash mutex shared between connection manager and slot tasks.
pub type FlashMutex = Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, FLASH_SIZE>>;

// ============ CYW43 Background Tasks ============

/// CYW43 task — required to run the WiFi/BLE chip (spawned on Core 0).
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

/// LED blink task using CYW43 (spawned on Core 0).
#[embassy_executor::task]
async fn led_task(control: &'static mut cyw43::Control<'static>) {
    loop {
        control.gpio_set(0, true).await;
        Timer::after_millis(1000).await;
        control.gpio_set(0, false).await;
        Timer::after_millis(1000).await;
    }
}

// ============ Core 0 Main Task ============

/// Core 0 main task: initializes CYW43, loads bonds, and runs the BLE central state machine.
///
/// Runs on Core 0 where `embassy_rp::init()` has already set up DMA and timer interrupts.
/// Flash bond operations are done inline (no cross-core channel needed since flash and
/// BLE are on the same core).
#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
pub async fn core0_ble_main(
    spawner: Spawner,
    pio0: Peri<'static, PIO0>,
    pin_23: Peri<'static, PIN_23>,
    pin_24: Peri<'static, PIN_24>,
    pin_25: Peri<'static, PIN_25>,
    pin_29: Peri<'static, PIN_29>,
    dma_ch0: Peri<'static, DMA_CH0>,
    mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>,
) -> ! {
    // --- CYW43 Initialization ---
    info!("[core0] Initializing CYW43...");

    let fw = cyw43::aligned_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");
    let btfw = cyw43::aligned_bytes!("../../cyw43-firmware/43439A0_btfw.bin");
    let nvram = cyw43::aligned_bytes!("../../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let dma = dma::Channel::new(dma_ch0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER * 4, // Work around marginal SPI signal integrity
        pio.irq0,
        cs,
        pin_24,
        pin_29,
        dma,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;

    // Spawn CYW43 background task on Core 0
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    info!("[core0] CYW43 initialized with Bluetooth");

    // Spawn LED blink task
    static CONTROL: StaticCell<cyw43::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(control);
    unwrap!(spawner.spawn(led_task(control)));

    // --- Load bonds and preferences from flash ---
    // Safe now that CYW43 firmware is fully downloaded.
    // Flash erase/write pauses Core 1 (USB) via FIFO, which is tolerable.
    let mut loaded_bonds = bonding::load_bonds(&mut flash).await;
    info!(
        "[core0] Loaded {} stored bond(s) from flash",
        loaded_bonds.len()
    );
    for lb in &loaded_bonds {
        info!(
            "  - Device: {:?} (profile: {:?})",
            lb.bond.identity.bd_addr,
            DeviceProfile::from_id(lb.profile_id)
        );
    }

    let active_device_pref = preferences::load_active_device(&mut flash).await;

    // Load forced OS override from flash and cache to scratch register.
    // Phase 0 checks this on the next soft reset to skip the probe.
    {
        let forced_os =
            preferences::load_u32_preference(&mut flash, preferences::PREF_KEY_FORCED_OS, 0).await;
        crate::scratch::write_forced_os(forced_os);
    }

    // Load axis multiplier preferences
    {
        use core::sync::atomic::Ordering::Relaxed;
        let scroll =
            preferences::load_multiplier(&mut flash, preferences::PREF_KEY_SCROLL_MULTIPLIER).await;
        let pan =
            preferences::load_multiplier(&mut flash, preferences::PREF_KEY_PAN_MULTIPLIER).await;
        let x = preferences::load_multiplier(&mut flash, preferences::PREF_KEY_X_MULTIPLIER).await;
        let y = preferences::load_multiplier(&mut flash, preferences::PREF_KEY_Y_MULTIPLIER).await;
        crate::usb_hid::MULTIPLIER_SCROLL.store(scroll, Relaxed);
        crate::usb_hid::MULTIPLIER_PAN.store(pan, Relaxed);
        crate::usb_hid::MULTIPLIER_X.store(x, Relaxed);
        crate::usb_hid::MULTIPLIER_Y.store(y, Relaxed);
        let threshold = preferences::load_u32_preference(
            &mut flash,
            preferences::PREF_KEY_SCROLL_THRESHOLD,
            120,
        )
        .await;
        let max_detents =
            preferences::load_u32_preference(&mut flash, preferences::PREF_KEY_MAX_DETENTS, 3)
                .await;
        crate::usb_hid::SCROLL_THRESHOLD.store(threshold, Relaxed);
        crate::usb_hid::MAX_DETENTS_PER_EMIT.store(max_detents, Relaxed);
        info!(
            "[core0] Axis multipliers: scroll={}% pan={}% x={}% y={}%",
            scroll, pan, x, y
        );
        info!(
            "[core0] Scroll params: threshold={} max_detents={}",
            threshold, max_detents
        );
    }

    if let Some(ref dev) = active_device_pref {
        info!("[core0] Active device preference: {:?}", dev.address);
    } else {
        info!("[core0] No active device preference set");
    }
    // --- Build BLE stack ---
    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    // Static random address: MSB must have bits 11 (>= 0xC0)
    let address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xca]);
    info!("[core0] Our BLE address: {:?}", address);

    static RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let resources = RESOURCES.init(HostResources::new());

    // Create RNG from RP2040's Ring Oscillator (ROSC) for hardware entropy
    use embassy_rp::clocks::RoscRng;
    let mut rosc_rng = RoscRng;
    let mut rng_seed = [0u8; 32];
    rosc_rng.fill_bytes(&mut rng_seed);
    use rand_chacha::ChaCha8Rng;
    use rand_core::SeedableRng;
    let mut rng = ChaCha8Rng::from_seed(rng_seed);

    let stack = trouble_host::new(controller, resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut rng);

    // Add all loaded bonds to the BLE stack BEFORE building
    for lb in &loaded_bonds {
        match stack.add_bond_information(lb.bond.clone()) {
            Ok(()) => {
                info!(
                    "[core0] Added bond for {:?} to BLE stack",
                    lb.bond.identity.bd_addr
                );
            }
            Err(e) => {
                error!(
                    "[core0] Failed to add bond for {:?}: {:?}",
                    lb.bond.identity.bd_addr, e
                );
            }
        }
    }

    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    let scanner_handler = RpcScannerHandler;
    let mut scanner = Scanner::new(central);

    info!(
        "[core0] BLE Central initialized ({} slots)",
        MAX_CONNECTIONS
    );

    // Wrap flash in a mutex for shared access between manager and slot tasks
    static FLASH_MUTEX: StaticCell<FlashMutex> = StaticCell::new();
    let flash_mutex = FLASH_MUTEX.init(Mutex::new(flash));

    // Auto-connect on startup: connect to bonds with auto_connect flag,
    // or fall back to legacy ActiveDevice preference for backward compatibility.
    let has_auto_connect_bonds = loaded_bonds.iter().any(|lb| lb.auto_connect);
    if has_auto_connect_bonds {
        // Queue AutoConnect for each bond with auto_connect=true
        for lb in &loaded_bonds {
            if lb.auto_connect {
                let mut addr = [0u8; 6];
                addr.copy_from_slice(lb.bond.identity.bd_addr.raw());
                let addr_kind = if (addr[5] & 0xC0) == 0xC0 { 1u8 } else { 0u8 };
                info!("[core0] Queuing auto-connect for {:?}", addr);
                let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Connect {
                    address: addr,
                    addr_kind,
                    ignore_bond: false,
                });
            }
        }
    } else if active_device_pref.is_some() {
        // Legacy: single active device preference
        let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
    }

    // --- Run BLE host, connection manager, and slot tasks concurrently ---
    let _ = join(
        runner.run_with_handler(&scanner_handler),
        join(
            connection_manager_loop(
                &mut scanner,
                &stack,
                flash_mutex,
                &mut loaded_bonds,
                &active_device_pref,
            ),
            join3(
                connection_slot_task(0, &stack, flash_mutex, &active_device_pref),
                connection_slot_task(1, &stack, flash_mutex, &active_device_pref),
                connection_slot_task(2, &stack, flash_mutex, &active_device_pref),
            ),
        ),
    )
    .await;

    // Should never reach here
    loop {
        Timer::after_millis(1000).await;
    }
}

// ============ Connection Manager ============

/// Connection manager loop — handles global commands and dispatches connect/disconnect
/// to slot tasks. Runs concurrently with slot tasks and the BLE runner.
async fn connection_manager_loop<
    'a,
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
    _stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
    loaded_bonds: &mut heapless::Vec<bonding::LoadedBond, { bonding::MAX_BONDS }>,
    active_device_pref: &Option<preferences::ActiveDevice>,
) {
    let mut manager_profile = determine_initial_profile(active_device_pref, loaded_bonds);

    info!("[manager] Connection manager started");

    loop {
        let cmd = BLE_CMD_CHANNEL.receive().await;

        match cmd {
            BleCommand::StartScan => {
                run_scan_session(scanner).await;
            }

            BleCommand::StopScan => {}

            BleCommand::Connect {
                address,
                addr_kind,
                ignore_bond: _,
            } => {
                // Check if already connected to this address
                if slots::find_slot_by_address(&address).is_some() {
                    info!("[manager] Already connected to {:?}", address);
                    rpc_log::warn("Already connected to this device");
                    continue;
                }

                // Find an idle slot
                match slots::find_idle_slot() {
                    Some(slot) => {
                        let has_bond = loaded_bonds
                            .iter()
                            .any(|lb| lb.bond.identity.bd_addr.raw() == address);

                        let profile = loaded_bonds
                            .iter()
                            .find(|lb| lb.bond.identity.bd_addr.raw() == address)
                            .map(|lb| DeviceProfile::from_id(lb.profile_id))
                            .unwrap_or(DeviceProfile::Generic);

                        info!(
                            "[manager] Assigning slot {} for {:?} (profile: {:?})",
                            slot, address, profile
                        );
                        CONNECT_SIGNALS[slot].signal(ConnectRequest {
                            address,
                            addr_kind,
                            profile,
                            has_stored_bond: has_bond,
                        });
                    }
                    None => {
                        warn!(
                            "[manager] No idle slots available ({}/{})",
                            MAX_CONNECTIONS, MAX_CONNECTIONS
                        );
                        rpc_log::error("All connection slots are in use");
                    }
                }
            }

            BleCommand::AutoConnect => {
                // Resolve target from preferences or first bond
                let target = resolve_auto_connect_target(loaded_bonds, active_device_pref);
                if let Some((addr, addr_kind, profile)) = target {
                    if slots::find_slot_by_address(&addr).is_some() {
                        info!("[manager] Auto-connect: already connected to {:?}", addr);
                        continue;
                    }
                    if let Some(slot) = slots::find_idle_slot() {
                        info!(
                            "[manager] Auto-connecting slot {} to {:?} (profile: {:?})",
                            slot, addr, profile
                        );
                        rpc_log::info("Auto-connecting to bonded device");
                        CONNECT_SIGNALS[slot].signal(ConnectRequest {
                            address: addr,
                            addr_kind,
                            profile,
                            has_stored_bond: true,
                        });
                    } else {
                        warn!("[manager] No idle slots for auto-connect");
                    }
                }
            }

            BleCommand::Disconnect { address } => {
                match address {
                    Some(addr) => {
                        // Disconnect a specific device
                        if let Some(slot) = slots::find_slot_by_address(&addr) {
                            let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                        } else {
                            info!("[manager] Device {:?} not connected", addr);
                        }
                    }
                    None => {
                        // Disconnect all
                        for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                            if !slots::is_slot_idle(i) {
                                let _ = ch.try_send(SlotCommand::Disconnect);
                            }
                        }
                    }
                }
            }

            BleCommand::SetActiveDevice { address, addr_kind } => {
                let mut f = flash.lock().await;
                commands::handle_set_active_device(&mut f, address, addr_kind).await;
            }

            BleCommand::ClearActiveDevice => {
                let mut f = flash.lock().await;
                commands::handle_clear_active_device(&mut f).await;
            }

            BleCommand::UpdateBondProfile {
                address,
                profile_id,
            } => {
                {
                    let mut f = flash.lock().await;
                    if let Some(new_bonds) =
                        commands::handle_update_bond_profile(&mut f, &address, profile_id).await
                    {
                        *loaded_bonds = new_bonds;
                    }
                }
                // Also notify the slot if connected
                if let Some(slot) = slots::find_slot_by_address(&address) {
                    let _ =
                        SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::UpdateProfile(profile_id));
                }
            }

            BleCommand::GetStatus => {
                debug!("Getting status info");
                commands::handle_get_status(loaded_bonds, manager_profile, active_device_pref);
            }

            BleCommand::GetBonds => {
                debug!("Getting bonds list");
                commands::handle_get_bonds(loaded_bonds);
            }

            BleCommand::ClearBonds => {
                // Disconnect all first
                for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                    if !slots::is_slot_idle(i) {
                        let _ = ch.try_send(SlotCommand::Disconnect);
                    }
                }
                // Brief delay for disconnects to process
                Timer::after_millis(200).await;

                let mut f = flash.lock().await;
                commands::handle_clear_bonds(&mut f).await;
                // handle_clear_bonds resets on success; if we get here it failed
                loaded_bonds.clear();
            }

            BleCommand::ClearBond { address } => {
                // Disconnect the device if connected
                if let Some(slot) = slots::find_slot_by_address(&address) {
                    let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                    Timer::after_millis(200).await;
                }
                let mut f = flash.lock().await;
                commands::handle_clear_bond(&mut f, &address, loaded_bonds).await;
            }

            BleCommand::SetConfig { key, value } => {
                let mut f = flash.lock().await;
                commands::handle_set_config(&mut f, key, value).await;
            }

            BleCommand::SetForcedOs { os } => {
                let mut f = flash.lock().await;
                commands::handle_set_forced_os(&mut f, os).await;
            }

            BleCommand::SetAutoConnect { address, enabled } => {
                let mut f = flash.lock().await;
                commands::handle_set_auto_connect(&mut f, &address, enabled, loaded_bonds).await;
            }

            BleCommand::Restart => {
                commands::handle_restart().await;
            }
        }

        // Update manager's profile from the first connected slot (for status reporting)
        for i in 0..MAX_CONNECTIONS {
            if !slots::is_slot_idle(i) {
                manager_profile = DeviceProfile::from_id(
                    slots::SLOT_PROFILES[i].load(core::sync::atomic::Ordering::Relaxed),
                );
                break;
            }
        }
    }
}

// ============ Connection Slot Task ============

/// Connection slot task — waits for connect signals, runs the connection lifecycle.
///
/// Each slot runs independently. When signaled by the connection manager, it:
/// 1. Creates its own Central via `stack.build()`
/// 2. Connects to the target device
/// 3. Runs the GATT session (HID report forwarding)
/// 4. On disconnect, marks itself idle and loops back
async fn connection_slot_task<'a, C: Controller>(
    slot: usize,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
    active_device_pref: &Option<preferences::ActiveDevice>,
) {
    info!("[slot{}] Connection slot task started", slot);

    loop {
        // Wait for the connection manager to signal us
        let request = CONNECT_SIGNALS[slot].wait().await;
        info!(
            "[slot{}] Connect request: {:?} (profile: {:?})",
            slot, request.address, request.profile
        );

        slots::set_slot_connecting(slot, request.address, request.profile);
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));

        let kind = if request.addr_kind == 1 {
            AddrKind::RANDOM
        } else {
            AddrKind::PUBLIC
        };
        let target = Address {
            kind,
            addr: BdAddr::new(request.address),
        };

        let mut active_profile = request.profile;

        // Run the connection lifecycle (connect, pair, GATT).
        // For bonded devices, auto-reconnect on unexpected disconnection.
        loop {
            connection::ble_connect_and_run(
                stack,
                flash,
                target,
                &mut active_profile,
                request.has_stored_bond,
                slot,
                active_device_pref,
            )
            .await;

            if !request.has_stored_bond {
                break;
            }

            // Bonded device lost — auto-reconnect after a brief delay.
            // Abort if a slot command arrives (e.g. user-initiated disconnect).
            info!("[slot{}] Bonded device lost, will auto-reconnect", slot);
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));
            rpc_log::info("Connection lost, reconnecting...");
            match embassy_futures::select::select(
                embassy_time::Timer::after(embassy_time::Duration::from_secs(2)),
                SLOT_CMD_CHANNELS[slot].receive(),
            )
            .await
            {
                embassy_futures::select::Either::First(_) => {
                    info!("[slot{}] Auto-reconnect attempt", slot);
                }
                embassy_futures::select::Either::Second(cmd) => {
                    info!("[slot{}] Command during reconnect wait: {:?}", slot, cmd);
                    break;
                }
            }
        }

        info!("[slot{}] Connection ended, marking idle", slot);
        slots::set_slot_idle(slot);
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Disconnected));
    }
}

// ============ Helpers ============

/// Determine the initial device profile from preferences or bond data.
fn determine_initial_profile(
    active_device_pref: &Option<preferences::ActiveDevice>,
    loaded_bonds: &[bonding::LoadedBond],
) -> DeviceProfile {
    if let Some(ref pref) = active_device_pref {
        loaded_bonds
            .iter()
            .find(|lb| lb.bond.identity.bd_addr.raw() == pref.address)
            .map(|lb| DeviceProfile::from_id(lb.profile_id))
            .unwrap_or(DeviceProfile::Generic)
    } else if !loaded_bonds.is_empty() {
        DeviceProfile::from_id(loaded_bonds[0].profile_id)
    } else {
        DeviceProfile::Generic
    }
}

/// Resolve the auto-connect target from preferences or bonds.
/// Returns (address, addr_kind, profile) or None.
fn resolve_auto_connect_target(
    loaded_bonds: &[bonding::LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> Option<([u8; 6], u8, DeviceProfile)> {
    let (target_addr, target_kind) = if let Some(ref pref) = active_device_pref {
        (pref.address, pref.addr_kind)
    } else if !loaded_bonds.is_empty() {
        let lb = &loaded_bonds[0];
        let addr_bytes = lb.bond.identity.bd_addr.raw();
        let kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
            1u8
        } else {
            0u8
        };
        let mut addr = [0u8; 6];
        addr.copy_from_slice(addr_bytes);
        (addr, kind)
    } else {
        warn!("AutoConnect: no active device or bonds");
        rpc_log::warn("AutoConnect: no active device or bonds");
        return None;
    };

    let bond_info = loaded_bonds
        .iter()
        .find(|lb| lb.bond.identity.bd_addr.raw() == target_addr);

    match bond_info {
        Some(lb) => {
            let profile = DeviceProfile::from_id(lb.profile_id);
            Some((target_addr, target_kind, profile))
        }
        None => {
            warn!(
                "AutoConnect: no bond found for active device {:?}",
                target_addr
            );
            rpc_log::warn("AutoConnect: device not bonded");
            None
        }
    }
}

/// Run a BLE scan session with a 30-second timeout, interruptible by commands.
///
/// Scan results are emitted via the scanner_handler's `on_adv_reports` callback.
/// The scan ends on: StopScan command, Connect command (re-queued), or 30s timeout.
async fn run_scan_session<
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
) {
    info!("Starting BLE scan...");
    rpc_log::info("Scanning for BLE HID devices");
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Scanning));

    let scan_config = trouble_host::connection::ScanConfig {
        active: true,
        interval: embassy_time::Duration::from_millis(100),
        window: embassy_time::Duration::from_millis(100),
        ..Default::default()
    };

    match scanner.scan(&scan_config).await {
        Ok(_session) => loop {
            match select(
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(30)),
            )
            .await
            {
                Either::First(BleCommand::StopScan) => {
                    info!("Scan stopped by command");
                    rpc_log::info("Scan stopped");
                    break;
                }
                Either::First(BleCommand::Connect {
                    address,
                    addr_kind,
                    ignore_bond,
                }) => {
                    info!("Connect command during scan");
                    let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Connect {
                        address,
                        addr_kind,
                        ignore_bond,
                    });
                    break;
                }
                Either::First(_) => {}
                Either::Second(_) => {
                    info!("Scan timeout (30s)");
                    rpc_log::info("Scan timeout (30s)");
                    break;
                }
            }
        },
        Err(e) => {
            error!("Failed to start scan: {:?}", defmt::Debug2Format(&e));
        }
    }
}
