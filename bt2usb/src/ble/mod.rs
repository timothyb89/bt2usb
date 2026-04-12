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

mod classic;
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
    CLASSIC_CMD_CHANNEL,
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
///
/// Liveness for the watchdog is tracked separately by `crate::watchdog`,
/// not by this task — the LED loop blocks on CYW43 HCI commands which
/// share a controller with the BLE stack and can stall during connect
/// attempts.
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

    // Enable the persistent watchdog now that CYW43 firmware download is done.
    // Per-task liveness is tracked in `crate::watchdog`; the feeder task only
    // pets the HW watchdog when every watched slot is fresh.
    crate::configure_watchdog();
    crate::watchdog::init();
    unwrap!(spawner.spawn(crate::watchdog::feeder_task()));
    unwrap!(spawner.spawn(crate::watchdog::core0_tick_task()));

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

    // Load Classic BT bonds from flash
    let loaded_classic_bonds = bonding::load_classic_bonds(&mut flash).await;
    if !loaded_classic_bonds.is_empty() {
        for cb in &loaded_classic_bonds {
            info!(
                "  - Classic: {:02x} (profile: {:?})",
                cb.addr,
                DeviceProfile::from_id(cb.profile_id)
            );
        }
    }

    let mut active_device_pref = preferences::load_active_device(&mut flash).await;

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
        let smoothing =
            preferences::load_u32_preference(&mut flash, preferences::PREF_KEY_SCROLL_SMOOTHING, 0)
                .await;
        crate::usb_hid::SCROLL_THRESHOLD.store(threshold, Relaxed);
        crate::usb_hid::MAX_DETENTS_PER_EMIT.store(max_detents, Relaxed);
        crate::usb_hid::SCROLL_SMOOTHING.store(smoothing, Relaxed);
        info!(
            "[core0] Axis multipliers: scroll={}% pan={}% x={}% y={}%",
            scroll, pan, x, y
        );
        info!(
            "[core0] Scroll params: threshold={} max_detents={} smoothing={}",
            threshold, max_detents, smoothing
        );
    }

    // Validate ActiveDevice preference against loaded bonds.
    // A stale preference (e.g. bonds cleared but preference persists) would
    // cause a futile auto-connect attempt on boot.
    if let Some(ref dev) = active_device_pref {
        let has_matching_bond = loaded_bonds
            .iter()
            .any(|lb| lb.bond.identity.bd_addr.raw() == dev.address);
        if has_matching_bond {
            info!("[core0] Active device preference: {:?}", dev.address);
        } else {
            warn!(
                "[core0] Ignoring stale active device {:?} (no matching bond)",
                dev.address
            );
            active_device_pref = None;
        }
    }
    // --- Build HCI multiplexer for dual-mode BT (Classic + BLE) ---
    static MUX_RES: StaticCell<hci_mux::MuxResources<10, 4>> = StaticCell::new();
    let mux_res = MUX_RES.init(hci_mux::MuxResources::new());
    let mux = hci_mux::HciMux::new(bt_device, mux_res);
    let (le_controller, classic_controller, mux_runner) = mux.split();

    // The LE controller is passed to trouble-host (replaces ExternalController).
    let controller = le_controller;

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

    // Auto-connect is handled by the background scan in the connection manager.
    // It detects bonded auto_connect devices as they advertise and connects them
    // one at a time, avoiding HCI command collisions.
    // Legacy: single active device preference (queues one connect at most).
    if !loaded_bonds.iter().any(|lb| lb.auto_connect) && active_device_pref.is_some() {
        let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
    }

    // --- Run HCI mux, BLE host, Classic host, and slot tasks concurrently ---
    let _ = join(
        // HCI mux runner: routes packets between BLE and Classic stacks
        mux_runner.run(),
        join(
            runner.run_with_handler(&scanner_handler),
            join(
                connection_manager_loop(
                    &mut scanner,
                    &stack,
                    flash_mutex,
                    &mut loaded_bonds,
                    &loaded_classic_bonds,
                    &mut active_device_pref,
                ),
                join(
                    join3(
                        connection_slot_task(0, &stack, flash_mutex),
                        connection_slot_task(1, &stack, flash_mutex),
                        connection_slot_task(2, &stack, flash_mutex),
                    ),
                    // Classic BT task — connects to MT2 etc.
                    classic::classic_bt_task(
                        &classic_controller,
                        flash_mutex,
                        &loaded_classic_bonds,
                    ),
                ),
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
    loaded_classic_bonds: &[bonding::StoredClassicBond],
    active_device_pref: &mut Option<preferences::ActiveDevice>,
) {
    let mut manager_profile = determine_initial_profile(active_device_pref, loaded_bonds);

    info!("[manager] Connection manager started");

    // Gate: wait for classic_bt_task to reissue the merged HCI event mask
    // before we touch the controller. Without this, the classic task's
    // SetEventMask can land mid-Create-Connection and wedge trouble-host
    // (see log-5.txt for the failing timeline). Pet the watchdog slot while
    // waiting so the gate itself doesn't look like a stall.
    {
        crate::watchdog::pet(crate::watchdog::SLOT_BLE_MANAGER);
        classic::CLASSIC_INIT_DONE.wait().await;
        info!("[manager] Classic init gate released");
    }

    loop {
        crate::watchdog::pet(crate::watchdog::SLOT_BLE_MANAGER);
        // Check if we should run a background scan for bonded auto-connect devices
        let unconnected = get_unconnected_auto_connect_addresses(loaded_bonds);
        let has_idle_slot = slots::find_idle_slot().is_some();

        // Don't start a background scan while any slot is actively connecting —
        // the scan would take the HCI scan state and cause "Command Disallowed"
        // when the slot retries its LE Create Connection.
        let any_connecting = slots::any_slot_connecting();

        let cmd = if !unconnected.is_empty() && has_idle_slot && !any_connecting {
            match run_background_scan(scanner, &unconnected).await {
                BgScanOutcome::BondedDeviceSeen(addr) => {
                    info!("[manager] Background scan: bonded device seen {:?}", addr);
                    rpc_log::info("Bonded device detected, connecting...");
                    // Small delay for HCI state to settle after scan stop
                    Timer::after_millis(50).await;
                    // Dispatch a single connect attempt — no per-slot retry loop.
                    // The bg-scan loop itself is the retry mechanism, which naturally
                    // round-robins across all unconnected auto-connect bonds.
                    dispatch_bg_auto_connect(loaded_bonds, addr);
                    continue;
                }
                BgScanOutcome::Command(cmd) => {
                    // Small delay for HCI state to settle after scan stop
                    Timer::after_millis(50).await;
                    cmd
                }
                BgScanOutcome::Timeout => continue, // Refresh filter list and restart scan
            }
        } else {
            // No background scanning needed. Periodically re-check in case
            // a slot goes idle (device disconnects while we're waiting).
            match embassy_futures::select::select(
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(5)),
            )
            .await
            {
                embassy_futures::select::Either::First(cmd) => cmd,
                embassy_futures::select::Either::Second(_) => continue,
            }
        };

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
                            max_attempts: 5,
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
                            max_attempts: 5,
                        });
                    } else {
                        warn!("[manager] No idle slots for auto-connect");
                    }
                }
            }

            BleCommand::Disconnect { address } => {
                match address {
                    Some(addr) => {
                        // Disconnect a specific device — check BLE slots then Classic
                        if let Some(slot) = slots::find_slot_by_address(&addr) {
                            let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                        } else if slots::is_classic_connected()
                            && slots::get_classic_address() == addr
                        {
                            let _ = CLASSIC_CMD_CHANNEL
                                .try_send(crate::ble_state::ClassicCommand::Disconnect);
                        } else {
                            info!("[manager] Device {:?} not connected", addr);
                        }
                    }
                    None => {
                        // Disconnect all BLE slots
                        for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                            if !slots::is_slot_idle(i) {
                                let _ = ch.try_send(SlotCommand::Disconnect);
                            }
                        }
                        // Disconnect Classic if connected
                        if slots::is_classic_connected() {
                            let _ = CLASSIC_CMD_CHANNEL
                                .try_send(crate::ble_state::ClassicCommand::Disconnect);
                        }
                    }
                }
            }

            BleCommand::SetActiveDevice { address, addr_kind } => {
                let mut f = flash.lock().await;
                commands::handle_set_active_device(&mut f, address, addr_kind).await;
                *active_device_pref = Some(preferences::ActiveDevice { address, addr_kind });
            }

            BleCommand::ClearActiveDevice => {
                let mut f = flash.lock().await;
                commands::handle_clear_active_device(&mut f).await;
                *active_device_pref = None;
            }

            BleCommand::UpdateBondProfile {
                address,
                profile_id,
                transport_type,
            } => {
                {
                    let mut f = flash.lock().await;
                    if let Some(new_bonds) = commands::handle_update_bond_profile(
                        &mut f,
                        &address,
                        profile_id,
                        transport_type,
                    )
                    .await
                    {
                        *loaded_bonds = new_bonds;
                    }
                }
                // Also notify the BLE slot if connected (Classic doesn't use slot commands)
                if transport_type == crate::ble_state::TransportType::Ble {
                    if let Some(slot) = slots::find_slot_by_address(&address) {
                        let _ = SLOT_CMD_CHANNELS[slot]
                            .try_send(SlotCommand::UpdateProfile(profile_id));
                    }
                }
            }

            BleCommand::GetStatus => {
                debug!("Getting status info");
                commands::handle_get_status(loaded_bonds, manager_profile, active_device_pref);
            }

            BleCommand::GetBonds => {
                debug!("Getting bonds list");
                commands::handle_get_bonds(loaded_bonds, loaded_classic_bonds);
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

            BleCommand::ClearBond {
                address,
                transport_type,
            } => {
                // Disconnect the device if connected (BLE slots only)
                if transport_type == crate::ble_state::TransportType::Ble {
                    if let Some(slot) = slots::find_slot_by_address(&address) {
                        let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                        Timer::after_millis(200).await;
                    }
                }
                let mut f = flash.lock().await;
                commands::handle_clear_bond(&mut f, &address, transport_type, loaded_bonds).await;
            }

            BleCommand::SetConfig { key, value } => {
                let mut f = flash.lock().await;
                commands::handle_set_config(&mut f, key, value).await;
            }

            BleCommand::SetForcedOs { os } => {
                let mut f = flash.lock().await;
                commands::handle_set_forced_os(&mut f, os).await;
            }

            BleCommand::SetAutoConnect {
                address,
                enabled,
                transport_type,
            } => {
                let mut f = flash.lock().await;
                commands::handle_set_auto_connect(
                    &mut f,
                    &address,
                    enabled,
                    transport_type,
                    loaded_bonds,
                )
                .await;
            }

            BleCommand::Restart => {
                commands::handle_restart().await;
            }

            BleCommand::FactoryReset => {
                // Disconnect all first
                for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                    if !slots::is_slot_idle(i) {
                        let _ = ch.try_send(SlotCommand::Disconnect);
                    }
                }
                Timer::after_millis(200).await;

                let mut f = flash.lock().await;
                commands::handle_factory_reset(&mut f).await;
                // handle_factory_reset resets on success; if we get here it failed
                loaded_bonds.clear();
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
async fn connection_slot_task<'a, C>(
    slot: usize,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
) where
    C: Controller
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeReadLocalSupportedFeatures>,
{
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
        // When connection ends, mark slot idle. The manager's background scan
        // will detect the device if it re-advertises and signal this slot again.
        connection::ble_connect_and_run(
            stack,
            flash,
            target,
            &mut active_profile,
            request.has_stored_bond,
            slot,
            request.max_attempts,
        )
        .await;

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

// ============ Background Scan for Auto-Reconnect ============

/// Dispatch a single-attempt connect to an idle slot on behalf of the background
/// auto-connect loop. Uses `max_attempts: 1` so the slot releases quickly after
/// one failure — the bg scan is the retry mechanism and naturally round-robins
/// across all unconnected auto-connect bonds.
fn dispatch_bg_auto_connect(loaded_bonds: &[bonding::LoadedBond], address: [u8; 6]) {
    if slots::find_slot_by_address(&address).is_some() {
        return;
    }
    let Some(slot) = slots::find_idle_slot() else {
        warn!("[manager] No idle slot for bg auto-connect {:?}", address);
        return;
    };
    let addr_kind = if (address[5] & 0xC0) == 0xC0 {
        1u8
    } else {
        0u8
    };
    let bond = loaded_bonds
        .iter()
        .find(|lb| lb.bond.identity.bd_addr.raw() == address);
    let profile = bond
        .map(|lb| DeviceProfile::from_id(lb.profile_id))
        .unwrap_or(DeviceProfile::Generic);
    let has_stored_bond = bond.is_some();
    info!(
        "[manager] Assigning slot {} for {:?} (profile: {:?}, bg auto-connect)",
        slot, address, profile
    );
    // Pre-reserve the slot synchronously so the manager's next loop iteration
    // sees `any_slot_connecting() == true` and won't race a new bg scan against
    // this slot's `central.connect()`. Both `scan()` and `connect()` in
    // trouble-host mutate the shared Filter Accept List via `set_accept_filter`
    // without internal serialization — if they interleave, the connect future
    // wedges. See notes/trouble-host-fal-race.md.
    slots::set_slot_connecting(slot, address, profile);
    CONNECT_SIGNALS[slot].signal(ConnectRequest {
        address,
        addr_kind,
        profile,
        has_stored_bond,
        max_attempts: 1,
    });
}

/// Outcome of the background scan.
enum BgScanOutcome {
    /// A bonded device was seen advertising.
    BondedDeviceSeen([u8; 6]),
    /// A BLE command arrived while scanning.
    Command(BleCommand),
    /// Scan timed out (time to refresh the filter list).
    Timeout,
}

/// Returns addresses of bonded devices with `auto_connect=true` that aren't
/// currently connected in any slot.
fn get_unconnected_auto_connect_addresses(
    loaded_bonds: &[bonding::LoadedBond],
) -> heapless::Vec<([u8; 6], u8), { bonding::MAX_BONDS }> {
    let mut addrs = heapless::Vec::new();
    for lb in loaded_bonds {
        if !lb.auto_connect {
            continue;
        }
        let mut addr = [0u8; 6];
        addr.copy_from_slice(lb.bond.identity.bd_addr.raw());
        if slots::find_slot_by_address(&addr).is_some() {
            continue;
        }
        let addr_kind = if (addr[5] & 0xC0) == 0xC0 { 1u8 } else { 0u8 };
        let _ = addrs.push((addr, addr_kind));
    }
    addrs
}

/// Run a passive background scan for bonded-but-not-connected devices.
///
/// Starts a passive scan with a filter accept list of bonded addresses.
/// Returns when a bonded device is seen, a command arrives, or the scan times out.
async fn run_background_scan<
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
    bonded_addrs: &[([u8; 6], u8)],
) -> BgScanOutcome {
    use bt_hci::param::AddrKind;
    use trouble_host::prelude::BdAddr;

    // Build filter accept list from bonded addresses. trouble-host will install
    // these into the controller's FAL and use ScanningFilterPolicy::BasicFiltered,
    // so non-bonded adverts are filtered in hardware and never reach the host.
    // This keeps logs quiet while unconnected auto-connect devices are being
    // waited for.
    //
    // Safety against the FAL race with slot `central.connect()`: we rely on the
    // manager's `any_slot_connecting()` gate plus the synchronous pre-reservation
    // in `dispatch_bg_auto_connect` to ensure bg scan and slot connect never run
    // concurrently. See notes/trouble-host-fal-race.md for the underlying bug.
    let mut bd_addrs: heapless::Vec<BdAddr, { bonding::MAX_BONDS }> = heapless::Vec::new();
    for (addr, _) in bonded_addrs {
        let _ = bd_addrs.push(BdAddr::new(*addr));
    }
    let mut filter_list: heapless::Vec<(AddrKind, &BdAddr), { bonding::MAX_BONDS }> =
        heapless::Vec::new();
    for (i, (_, kind)) in bonded_addrs.iter().enumerate() {
        let addr_kind = if *kind == 1 {
            AddrKind::RANDOM
        } else {
            AddrKind::PUBLIC
        };
        let _ = filter_list.push((addr_kind, &bd_addrs[i]));
    }

    let scan_config = trouble_host::connection::ScanConfig {
        active: false, // Passive scan — lower power
        filter_accept_list: &filter_list,
        interval: embassy_time::Duration::from_millis(200),
        window: embassy_time::Duration::from_millis(100),
        timeout: embassy_time::Duration::from_secs(0), // No HCI timeout
        ..Default::default()
    };

    // Tell the scanner handler we're in bg scan mode (so it signals
    // BONDED_DEVICE_SEEN instead of emitting ScanResult events).
    crate::ble_state::BONDED_DEVICE_SEEN.reset();
    crate::ble_state::BG_SCAN_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);

    let result = match scanner.scan(&scan_config).await {
        Ok(_session) => {
            // _session keeps scan alive; dropping it stops the scan
            // 20s timeout — well under the 60s ble_manager watchdog deadline so
            // that even if the outer loop does nothing but re-run bg scan, the
            // feeder keeps getting fresh pets.
            match embassy_futures::select::select3(
                crate::ble_state::BONDED_DEVICE_SEEN.wait(),
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(20)),
            )
            .await
            {
                embassy_futures::select::Either3::First(addr) => {
                    BgScanOutcome::BondedDeviceSeen(addr)
                }
                embassy_futures::select::Either3::Second(cmd) => BgScanOutcome::Command(cmd),
                embassy_futures::select::Either3::Third(_) => BgScanOutcome::Timeout,
            }
        }
        Err(_) => {
            // Scan failed to start (e.g. HCI busy) — just wait for a command
            BgScanOutcome::Command(BLE_CMD_CHANNEL.receive().await)
        }
    };

    crate::ble_state::BG_SCAN_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
    result
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
    crate::ble_state::hid_cache_clear();
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
