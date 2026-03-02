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

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::Peri;
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

/// Max BLE connections
const CONNECTIONS_MAX: usize = 1;
/// Max L2CAP channels (signal + att + coc)
const L2CAP_CHANNELS_MAX: usize = 3;

// ============ CYW43 Background Tasks ============

/// CYW43 task — required to run the WiFi/BLE chip (spawned on Core 0).
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
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

    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");
    let btfw = include_bytes!("../../cyw43-firmware/43439A0_btfw.bin");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER * 4, // Work around marginal SPI signal integrity
        pio.irq0,
        cs,
        pin_24,
        pin_29,
        dma_ch0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;

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
        info!(
            "[core0] Axis multipliers: scroll={}% pan={}% x={}% y={}%",
            scroll, pan, x, y
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
    info!("[core0] RNG Seed: {=[u8]:x}", rng_seed);

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

    info!("[core0] BLE Central initialized");

    // Auto-connect on startup if an active device preference is set
    if active_device_pref.is_some() {
        let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
    }

    // --- Run BLE host and command state machine concurrently ---
    let _ = join(runner.run_with_handler(&scanner_handler), async {
        let mut active_profile = determine_initial_profile(&active_device_pref, &loaded_bonds);
        let mut pending_cmd: Option<BleCommand> = None;

        loop {
            let _ =
                BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Disconnected));
            info!("[core0] BLE state: Idle. Waiting for command...");

            let cmd = if let Some(c) = pending_cmd.take() {
                c
            } else {
                BLE_CMD_CHANNEL.receive().await
            };

            // --- Idle command dispatch ---
            // Simple commands delegate to commands:: helpers.
            // Connect/scan commands handle scanner ownership inline.
            pending_cmd = match cmd {
                BleCommand::StartScan => {
                    run_scan_session(&mut scanner).await;
                    None
                }

                BleCommand::StopScan => None,

                BleCommand::Connect {
                    address,
                    addr_kind,
                    ignore_bond: _,
                } => {
                    let kind = if addr_kind == 1 {
                        AddrKind::RANDOM
                    } else {
                        AddrKind::PUBLIC
                    };
                    let target = Address {
                        kind,
                        addr: BdAddr::new(address),
                    };
                    active_profile = DeviceProfile::Generic;

                    let has_stored_bond = loaded_bonds
                        .iter()
                        .any(|lb| lb.bond.identity.bd_addr.raw() == address);

                    let mut central = scanner.into_inner();
                    let result = connection::ble_connect_and_run(
                        &mut central,
                        &stack,
                        &mut flash,
                        target,
                        &mut active_profile,
                        has_stored_bond,
                        &loaded_bonds,
                        &active_device_pref,
                    )
                    .await;
                    scanner = Scanner::new(central);
                    result
                }

                BleCommand::AutoConnect => {
                    let mut central = scanner.into_inner();
                    let result = resolve_auto_connect_and_run(
                        &mut central,
                        &stack,
                        &mut flash,
                        &mut active_profile,
                        &mut loaded_bonds,
                        &active_device_pref,
                    )
                    .await;
                    scanner = Scanner::new(central);
                    result
                }

                BleCommand::Disconnect => {
                    info!("Not connected, nothing to disconnect");
                    None
                }

                BleCommand::SetActiveDevice { address, addr_kind } => {
                    commands::handle_set_active_device(&mut flash, address, addr_kind).await;
                    None
                }

                BleCommand::ClearActiveDevice => {
                    commands::handle_clear_active_device(&mut flash).await;
                    None
                }

                BleCommand::UpdateBondProfile {
                    address,
                    profile_id,
                } => {
                    if let Some(new_bonds) =
                        commands::handle_update_bond_profile(&mut flash, &address, profile_id).await
                    {
                        loaded_bonds = new_bonds;
                    }
                    None
                }

                BleCommand::GetStatus => {
                    info!("Getting status info");
                    commands::handle_get_status(&loaded_bonds, active_profile, &active_device_pref);
                    None
                }

                BleCommand::GetBonds => {
                    info!("Getting bonds list");
                    commands::handle_get_bonds(&loaded_bonds);
                    None
                }

                BleCommand::ClearBonds => {
                    commands::handle_clear_bonds(&mut flash).await;
                    // handle_clear_bonds resets on success; if we get here it failed
                    loaded_bonds.clear();
                    None
                }

                BleCommand::SetConfig { key, value } => {
                    commands::handle_set_config(&mut flash, key, value).await;
                    None
                }

                BleCommand::Restart => {
                    commands::handle_restart().await;
                }
            };
        }
    })
    .await;

    // Should never reach here
    loop {
        Timer::after_millis(1000).await;
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

/// Resolve the auto-connect target from preferences/bonds, then connect.
///
/// Uses the active device preference if set, otherwise falls back to the first bond.
/// Returns `Some(BleCommand)` if interrupted by a command during connection.
async fn resolve_auto_connect_and_run<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: &mut DeviceProfile,
    loaded_bonds: &mut heapless::Vec<bonding::LoadedBond, { bonding::MAX_BONDS }>,
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> Option<BleCommand> {
    // Resolve target address from preferences or first bond
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

    // Find the bond for this device
    let bond_info = loaded_bonds
        .iter()
        .find(|lb| lb.bond.identity.bd_addr.raw() == target_addr);

    if bond_info.is_none() {
        warn!(
            "AutoConnect: no bond found for active device {:?}",
            target_addr
        );
        rpc_log::warn("AutoConnect: device not bonded");
        return None;
    }

    *active_profile = DeviceProfile::from_id(bond_info.unwrap().profile_id);
    info!(
        "Auto-connecting to {:?} (profile: {:?})",
        target_addr, active_profile
    );
    rpc_log::info("Auto-connecting to bonded device");

    let kind = if target_kind == 1 {
        AddrKind::RANDOM
    } else {
        AddrKind::PUBLIC
    };
    let target = Address {
        kind,
        addr: BdAddr::new(target_addr),
    };

    connection::ble_connect_and_run(
        central,
        stack,
        flash,
        target,
        active_profile,
        true,
        loaded_bonds,
        active_device_pref,
    )
    .await
}
