//! bt2usb - BLE HID to USB HID Bridge (Dual-Core)
//!
//! This firmware runs on a Raspberry Pi Pico W and:
//! 1. Acts as a BLE Central, connecting to HID peripherals (keyboards, mice)
//! 2. Translates BLE HID reports to USB HID reports
//! 3. Presents itself as a USB HID device to the host computer
//!
//! Dual-core architecture:
//! - Core 0: CYW43 WiFi/BLE chip + BLE central state machine + flash bond storage
//! - Core 1: USB device tasks (keyboard, mouse, RPC)
//!
//! CYW43 runs on Core 0 (where embassy_rp::init() sets up DMA and timer interrupts).
//! USB runs on Core 1 (USB peripheral handles NAKs gracefully during brief flash pauses).
//! Flash erase/write pauses Core 1 via FIFO, which is tolerable for USB but would
//! corrupt BLE timing-sensitive operations if BLE were on the paused core.

#![no_std]
#![no_main]

mod ble_hid;
mod ble_state;
mod bonding;
mod device_profile;
mod framing;
mod protocol;
mod rpc;
mod rpc_log;
mod usb_hid;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::interrupt::InterruptExt;
use embassy_rp::multicore::{spawn_core1, Stack as Core1Stack};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIN_23, PIN_24, PIN_25, PIN_29, PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_usb::class::hid::{HidReaderWriter, HidWriter};
use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use trouble_host::connection::{ConnectConfig, ScanConfig};
use trouble_host::scan::Scanner;

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use ble_hid::{parse_hid_report, HidReportType, HID_REPORT_CHANNEL};
use ble_state::{BleCommand, BleEvent, RpcScannerHandler, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL};
use device_profile::DeviceProfile;
use embassy_time::Timer;
use protocol::ConnectionState;
use usb_hid::{
    serialize_keyboard_report, serialize_mouse_report, serialize_mouse_report_16bit,
    KeyboardHidReport, MOUSE_HIRES_16BIT_REPORT_DESC, MOUSE_HIRES_REPORT_DESC,
    VENDOR_RPC_REPORT_DESC,
};

// ============ Target HID Device Configuration ============
/// Default device profile (used for fresh pairing when no stored profile exists)
const DEFAULT_PROFILE: DeviceProfile = DeviceProfile::FullScrollDial16Bit;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// Flash size for RP2040 (2MB)
const FLASH_SIZE: usize = 2 * 1024 * 1024;

// ============ Dual-Core Infrastructure ============

/// Core 1 stack (16KB - needs room for P-256 crypto, GATT discovery, CYW43 SPI)
static mut CORE1_STACK: Core1Stack<16384> = Core1Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// Signal from Core 1 -> Core 0: Core 1 executor is running.
/// Core 0 must not start its executor until Core 1's executor is up,
/// otherwise both cores starting simultaneously causes a stall.
static CORE1_EXECUTOR_READY: AtomicBool = AtomicBool::new(false);

// ============ Core 0 Tasks (CYW43 + BLE + Flash) ============

/// CYW43 task - required to run the WiFi/BLE chip (spawned on Core 0)
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

/// LED blink task using CYW43 (spawned on Core 0)
#[embassy_executor::task]
async fn led_task(control: &'static mut cyw43::Control<'static>) {
    loop {
        control.gpio_set(0, true).await;
        Timer::after_millis(1000).await;
        control.gpio_set(0, false).await;
        Timer::after_millis(1000).await;
    }
}

/// Max BLE connections
const CONNECTIONS_MAX: usize = 1;
/// Max L2CAP channels (signal + att + coc)
const L2CAP_CHANNELS_MAX: usize = 3;

// ============ Core 0 Task (CYW43 + BLE + Flash) ============

/// Core 0 main task: initializes CYW43, loads bonds, and runs the BLE central state machine.
/// Runs on Core 0 where embassy_rp::init() has already set up DMA and timer interrupts.
/// Flash bond operations are done inline (no cross-core channel needed).
#[embassy_executor::task]
async fn core0_ble_main(
    spawner: Spawner,
    pio0: Peri<'static, PIO0>,
    pin_23: Peri<'static, PIN_23>,
    pin_24: Peri<'static, PIN_24>,
    pin_25: Peri<'static, PIN_25>,
    pin_29: Peri<'static, PIN_29>,
    dma_ch0: Peri<'static, DMA_CH0>,
    mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>,
) -> ! {
    info!("[core0] Initializing CYW43...");

    // CYW43 firmware blobs
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let btfw = include_bytes!("../cyw43-firmware/43439A0_btfw.bin");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER * 2, // Slower clock (half speed) for better signal integrity
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

    // Spawn LED blink task on Core 0
    static CONTROL: StaticCell<cyw43::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(control);
    unwrap!(spawner.spawn(led_task(control)));

    // Flash operations - safe now that CYW43 firmware is fully downloaded.
    // Flash erase/write pauses Core 1 (USB) via FIFO, which is tolerable.
    // CYW43 on this core just has interrupts briefly disabled during flash ops.
    info!("[core0] Clearing all bonds...");
    bonding::clear_all_bonds(&mut flash).await.ok();

    let loaded_bonds = bonding::load_bonds(&mut flash).await;
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

    // ============ BLE Central Logic ============
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

    // If we have stored bonds, auto-connect on startup
    if !loaded_bonds.is_empty() {
        let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
    }

    // Run BLE host (with scanner handler) and command-driven state machine concurrently
    let _ = join(runner.run_with_handler(&scanner_handler), async {
        // ============ Command-driven BLE state machine ============
        let mut active_profile = if !loaded_bonds.is_empty() {
            DeviceProfile::from_id(loaded_bonds[0].profile_id)
        } else {
            DEFAULT_PROFILE
        };

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
            match cmd {
                BleCommand::StartScan => {
                    info!("Starting BLE scan...");
                    rpc_log::info("Scanning for BLE HID devices");
                    let _ = BLE_EVENT_CHANNEL
                        .try_send(BleEvent::StateChanged(ConnectionState::Scanning));

                    let mut scan_config = ScanConfig::default();
                    scan_config.active = true;
                    scan_config.interval = embassy_time::Duration::from_millis(100);
                    scan_config.window = embassy_time::Duration::from_millis(100);

                    match scanner.scan(&scan_config).await {
                        Ok(_session) => loop {
                            match select(
                                BLE_CMD_CHANNEL.receive(),
                                embassy_time::Timer::after(embassy_time::Duration::from_secs(30)),
                            )
                            .await
                            {
                                Either::First(BleCommand::StopScan) => {
                                    info!("Scan stopped by command");
                                    rpc_log::info("Scan stopped");
                                    break;
                                }
                                Either::First(BleCommand::Connect { address, addr_kind }) => {
                                    info!("Connect command during scan");
                                    let _ = BLE_CMD_CHANNEL
                                        .try_send(BleCommand::Connect { address, addr_kind });
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
                            error!("Failed to start scan: {:?}", e);
                        }
                    }
                }

                BleCommand::StopScan => {}

                BleCommand::Connect { address, addr_kind } => {
                    let kind = if addr_kind == 1 {
                        AddrKind::RANDOM
                    } else {
                        AddrKind::PUBLIC
                    };
                    let target = Address {
                        kind,
                        addr: BdAddr::new(address),
                    };
                    active_profile = DEFAULT_PROFILE;

                    let mut central = scanner.into_inner();
                    pending_cmd = ble_connect_and_run(
                        &mut central,
                        &stack,
                        &mut flash,
                        target,
                        active_profile,
                        false,
                    )
                    .await;
                    scanner = Scanner::new(central);
                }

                BleCommand::AutoConnect => {
                    if loaded_bonds.is_empty() {
                        warn!("AutoConnect: no bonds stored");
                        rpc_log::warn("AutoConnect: no bonds stored");
                        continue;
                    }
                    let lb = &loaded_bonds[0];
                    let addr = lb.bond.identity.bd_addr;
                    active_profile = DeviceProfile::from_id(lb.profile_id);
                    info!(
                        "Auto-connecting to bonded device {:?} (profile: {:?})",
                        addr, active_profile
                    );
                    rpc_log::info("Auto-connecting to bonded device");

                    let addr_bytes = addr.raw();
                    let kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
                        AddrKind::RANDOM
                    } else {
                        AddrKind::PUBLIC
                    };
                    let target = Address { kind, addr };

                    let mut central = scanner.into_inner();
                    pending_cmd = ble_connect_and_run(
                        &mut central,
                        &stack,
                        &mut flash,
                        target,
                        active_profile,
                        true,
                    )
                    .await;
                    scanner = Scanner::new(central);
                }

                BleCommand::Disconnect => {
                    info!("Not connected, nothing to disconnect");
                }
            }
        }
    })
    .await;

    // Should never reach here
    loop {
        Timer::after_millis(1000).await;
    }
}

// ============ Core 1 Tasks (USB) ============

/// USB device task - handles USB enumeration and events (spawned on Core 1)
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// USB HID handler task - receives HID reports from BLE (Core 0) and sends to USB
#[embassy_executor::task]
async fn usb_hid_handler_task(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mouse_writer: HidWriter<'static, Driver<'static, USB>, 7>,
) {
    info!("USB HID handler task started, waiting for BLE reports...");

    loop {
        let event = HID_REPORT_CHANNEL.receive().await;

        debug!(
            "Received HID report: type={:?}, len={}",
            event.report_type, event.len
        );

        match event.report_type {
            HidReportType::Keyboard => {
                if event.len >= 8 {
                    let report = KeyboardHidReport {
                        modifier: event.data[0],
                        reserved: event.data[1],
                        leds: 0,
                        keycodes: [
                            event.data[2],
                            event.data[3],
                            event.data[4],
                            event.data[5],
                            event.data[6],
                            event.data[7],
                        ],
                    };
                    let buf = serialize_keyboard_report(&report);
                    if let Err(e) = keyboard_writer.write(&buf).await {
                        warn!("Keyboard write error: {:?}", e);
                    }
                }
            }
            HidReportType::Mouse => {
                if event.len >= 1 {
                    if event.profile.uses_16bit_reports() {
                        let report = event
                            .profile
                            .translate_mouse_report_16bit(&event.data, event.len);
                        let buf = serialize_mouse_report_16bit(&report);
                        if let Err(e) = mouse_writer.write(&buf).await {
                            warn!("Mouse write error (16-bit): {:?}", e);
                        }
                    } else {
                        let report = event.profile.translate_mouse_report(&event.data, event.len);
                        let buf = serialize_mouse_report(&report);
                        if let Err(e) = mouse_writer.write(&buf).await {
                            warn!("Mouse write error (8-bit): {:?}", e);
                        }
                    }
                }
            }
            _ => {
                debug!("Unhandled HID report type");
            }
        }
    }
}

// ============ Entry Point (Dual-Core) ============

/// Main entry point - runs on Core 0.
/// Spawns USB on Core 1, then runs CYW43+BLE+Flash on Core 0.
#[cortex_m_rt::entry]
fn main() -> ! {
    info!("bt2usb starting (dual-core mode)...");

    let p = embassy_rp::init(Default::default());

    // ============ Flash Storage (Core 0) ============
    let flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH1);

    // ============ Spawn Core 1 (USB) ============
    let usb = p.USB;

    spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            // Enable TIMER_IRQ_0 on Core 1's NVIC so embassy_time works
            // (e.g. RPC timeouts). embassy_rp::init() only enables it on Core 0.
            // The timer driver uses hardware spinlocks, safe for dual-core.
            unsafe {
                embassy_rp::interrupt::TIMER_IRQ_0
                    .set_priority(embassy_rp::interrupt::Priority::P3);
                embassy_rp::interrupt::TIMER_IRQ_0.enable();
            }

            // Build USB device on Core 1 - this ensures USBCTRL_IRQ is enabled
            // on Core 1's NVIC (where we want USB interrupts to fire).
            let driver = Driver::new(usb, Irqs);
            let mut config = embassy_usb::Config::new(0x1209, 0x0001);
            config.manufacturer = Some("momentary");
            config.product = Some("BT2USB Bridge");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;

            // Boot Keyboard
            let kb_config = embassy_usb::class::hid::Config {
                report_descriptor: KeyboardReport::desc(),
                request_handler: None,
                poll_ms: 1,
                max_packet_size: 64,
            };
            // Mouse (select descriptor based on profile)
            static MOUSE_HANDLER: StaticCell<usb_hid::HiresMouseRequestHandler> = StaticCell::new();
            let mouse_descriptor = if DEFAULT_PROFILE.uses_16bit_reports() {
                MOUSE_HIRES_16BIT_REPORT_DESC
            } else {
                MOUSE_HIRES_REPORT_DESC
            };
            let mouse_config = embassy_usb::class::hid::Config {
                report_descriptor: mouse_descriptor,
                request_handler: Some(MOUSE_HANDLER.init(usb_hid::HiresMouseRequestHandler)),
                poll_ms: 1,
                max_packet_size: 64,
            };

            // State buffers
            static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
            static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

            static STATE_KB: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
            static STATE_MOUSE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
            static STATE_RPC: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

            let mut builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESC.init([0; 256]),
                BOS_DESC.init([0; 256]),
                MS_DESC.init([0; 256]),
                CONTROL_BUF.init([0; 64]),
            );

            let kb_writer = HidWriter::<_, 8>::new(
                &mut builder,
                STATE_KB.init(embassy_usb::class::hid::State::new()),
                kb_config,
            );

            let mouse_writer = HidWriter::<_, 7>::new(
                &mut builder,
                STATE_MOUSE.init(embassy_usb::class::hid::State::new()),
                mouse_config,
            );

            // Vendor HID interface for RPC communication
            let rpc_config = embassy_usb::class::hid::Config {
                report_descriptor: VENDOR_RPC_REPORT_DESC,
                request_handler: None,
                poll_ms: 10,
                max_packet_size: 64,
            };
            let rpc_hid = HidReaderWriter::<_, 64, 64>::new(
                &mut builder,
                STATE_RPC.init(embassy_usb::class::hid::State::new()),
                rpc_config,
            );
            let (rpc_reader, rpc_writer) = rpc_hid.split();

            let usb_dev = builder.build();
            info!("[core1] USB HID device initialized");

            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(usb_task(usb_dev)).unwrap();
                spawner
                    .spawn(usb_hid_handler_task(kb_writer, mouse_writer))
                    .unwrap();
                spawner
                    .spawn(rpc::rpc_task(rpc_writer, rpc_reader))
                    .unwrap();
                // Signal Core 0: our executor is set up and tasks are spawned
                CORE1_EXECUTOR_READY.store(true, Ordering::Release);
                cortex_m::asm::sev(); // Wake Core 0 from WFE
            });
        },
    );
    info!("[core0] Core 1 spawned, waiting for executor ready...");

    // Wait for Core 1's executor to be fully set up before starting Core 0's.
    // Without this, both executors starting simultaneously causes a stall.
    while !CORE1_EXECUTOR_READY.load(Ordering::Acquire) {
        cortex_m::asm::wfe();
    }
    info!("[core0] Core 1 executor ready");

    // ============ Core 0 Executor (CYW43 + BLE + Flash) ============
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner
            .spawn(core0_ble_main(
                spawner, p.PIO0, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.DMA_CH0, flash,
            ))
            .unwrap();
    });
}

// ============ BLE Connection Logic ============

enum StartError {
    PairingFailed(trouble_host::Error),
    Disconnected(bt_hci::param::Status),
}

/// Connect to a BLE HID device, pair, discover GATT, and run the HID report loop.
///
/// Bond storage is done directly via flash (all on Core 0, no cross-core channel needed).
/// Returns `Option<BleCommand>` if interrupted by a command, or `None` if connection ended naturally.
async fn ble_connect_and_run<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    target: Address,
    active_profile: DeviceProfile,
    has_stored_bond: bool,
) -> Option<BleCommand> {
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));
    rpc_log::info("Connecting to BLE device");

    let mut config = ConnectConfig {
        connect_params: Default::default(),
        scan_config: ScanConfig {
            active: true,
            filter_accept_list: &[(target.kind, &target.addr)],
            ..Default::default()
        },
    };
    config.connect_params.min_connection_interval = embassy_time::Duration::from_micros(7500);
    config.connect_params.max_connection_interval = embassy_time::Duration::from_millis(15);
    config.connect_params.supervision_timeout = embassy_time::Duration::from_secs(2);

    const MAX_PAIRING_RETRIES: u8 = 5;
    let mut pairing_attempts: u8 = 0;

    'connect: loop {
        pairing_attempts += 1;
        info!(
            "Connection attempt {} of {}",
            pairing_attempts, MAX_PAIRING_RETRIES
        );

        // 1. Connect with command interruption
        let conn: Connection<'_, DefaultPacketPool> =
            match select(central.connect(&config), BLE_CMD_CHANNEL.receive()).await {
                Either::First(Ok(conn)) => conn,
                Either::First(Err(e)) => {
                    error!("Connection failed: {:?}", defmt::Debug2Format(&e));
                    rpc_log::error("Connection attempt failed");
                    if pairing_attempts >= MAX_PAIRING_RETRIES {
                        error!("Max connection retries exceeded");
                        rpc_log::error("Max connection retries exceeded");
                        return None;
                    }
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
                    continue 'connect;
                }
                Either::Second(cmd) => {
                    info!("Command received during connection attempt: {:?}", cmd);
                    match cmd {
                        BleCommand::Connect { .. } | BleCommand::AutoConnect => return Some(cmd),
                        BleCommand::Disconnect | BleCommand::StopScan => return None,
                        _ => return None,
                    }
                }
            };

        info!("Connected to HID device!");
        rpc_log::info("Connected to BLE device");
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connected));

        // Set bondable based on whether we have an existing bond
        if let Err(e) = conn.set_bondable(!has_stored_bond) {
            error!("Failed to set bondable: {:?}", e);
        }

        // Request security (triggers pairing or re-encryption)
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Pairing));
        rpc_log::info("Pairing / re-encryption in progress");
        info!("Requesting security (bondable: {})...", !has_stored_bond);
        match conn.request_security() {
            Ok(_) => info!("Security request sent"),
            Err(e) => {
                error!("Failed to request security: {:?}", e);
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    return None;
                }
                embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
                continue 'connect;
            }
        }

        // Wait for pairing outcome with timeout and command interruption
        let pairing_timeout = embassy_time::Timer::after(embassy_time::Duration::from_secs(15));

        let pairing_result = select(
            select(
                async {
                    loop {
                        match conn.next().await {
                            ConnectionEvent::PairingComplete {
                                security_level,
                                bond,
                            } => {
                                return Ok((security_level, bond));
                            }
                            ConnectionEvent::PairingFailed(e) => {
                                return Err(StartError::PairingFailed(e));
                            }
                            ConnectionEvent::Disconnected { reason } => {
                                return Err(StartError::Disconnected(reason));
                            }
                            _ => {}
                        }
                    }
                },
                BLE_CMD_CHANNEL.receive(),
            ),
            pairing_timeout,
        )
        .await;

        let pairing_ok = match pairing_result {
            Either::First(Either::First(Ok((security_level, bond)))) => {
                info!("Pairing complete! Level: {:?}", security_level);
                rpc_log::info("Pairing complete");
                let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingComplete);
                if let Some(bond_info) = bond {
                    // Store bond directly to flash (all on Core 0)
                    match bonding::store_bond(flash, &bond_info, active_profile.to_id()).await {
                        Ok(slot) => {
                            info!("Bond stored in slot {}", slot);
                            let mut addr_buf = [0u8; 6];
                            addr_buf.copy_from_slice(bond_info.identity.bd_addr.raw());
                            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BondStored {
                                address: addr_buf,
                                profile_id: active_profile.to_id(),
                            });
                        }
                        Err(_) => error!("Failed to store bond"),
                    }
                }
                true
            }
            Either::First(Either::First(Err(e))) => {
                match e {
                    StartError::PairingFailed(pe) => {
                        error!("Pairing failed: {:?}", pe);
                        rpc_log::error("Pairing failed");
                        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                    }
                    StartError::Disconnected(reason) => {
                        error!("Disconnected during pairing: {:?}", reason);
                        rpc_log::error("Disconnected during pairing");
                        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                    }
                }
                false
            }
            Either::First(Either::Second(cmd)) => {
                info!("Command received during pairing: {:?}", cmd);
                conn.disconnect();
                let _ = select(
                    async {
                        loop {
                            if let ConnectionEvent::Disconnected { .. } = conn.next().await {
                                break;
                            }
                        }
                    },
                    embassy_time::Timer::after(embassy_time::Duration::from_secs(1)),
                )
                .await;

                match cmd {
                    BleCommand::Connect { .. } | BleCommand::AutoConnect => return Some(cmd),
                    _ => return None,
                }
            }
            Either::Second(_) => {
                error!("Pairing timed out!");
                rpc_log::error("Pairing timed out");
                let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                false
            }
        };

        if !pairing_ok {
            if pairing_attempts >= MAX_PAIRING_RETRIES {
                error!("Max pairing retries exceeded");
                rpc_log::error("Max pairing retries exceeded");
                return None;
            }
            embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
            continue 'connect;
        }

        if pairing_ok {
            info!("Creating GATT client...");

            let client = match GattClient::<C, DefaultPacketPool, 10>::new(stack, &conn).await {
                Ok(c) => c,
                Err(e) => {
                    error!(
                        "Failed to create GATT client: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    return None;
                }
            };

            // Run GATT client and HID loop concurrently

            let run_result = join(client.task(), async {
                // Discover HID service
                let hid_uuid = Uuid::new_short(0x1812);
                let services = match embassy_time::with_timeout(
                    embassy_time::Duration::from_secs(10),
                    client.services_by_uuid(&hid_uuid),
                )
                .await
                {
                    Ok(Ok(s)) if !s.is_empty() => s,
                    Ok(Ok(_)) => {
                        error!("No HID service found!");
                        return None;
                    }
                    Ok(Err(e)) => {
                        error!(
                            "HID service discovery failed: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        return None;
                    }
                    Err(_) => {
                        error!("HID service discovery timeout");
                        return None;
                    }
                };

                let hid_service = &services[0];
                info!("Found HID service");
                rpc_log::info("HID service discovered");

                // Discover HID Report characteristic
                let report_uuid = Uuid::new_short(0x2A4D);
                let report_char: Characteristic<[u8; 64]> = match client
                    .characteristic_by_uuid(hid_service, &report_uuid)
                    .await
                {
                    Ok(c) => {
                        info!("Found Report characteristic (UUID 0x2A4D)");
                        info!("CRITICAL: Characteristic handle = {:?}", c.handle);
                        info!("Linux uses handle 0x0020 for symmetric scroll values");
                        info!("If we're using a different handle, that explains the asymmetry!");
                        c
                    }
                    Err(e) => {
                        error!(
                            "Report characteristic not found: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        return None;
                    }
                };

                // Subscribe to notifications
                match client.subscribe(&report_char, false).await {
                    Ok(mut listener) => {
                        info!("=== HID CONNECTION ESTABLISHED ===");
                        info!(
                            "Subscribed to notifications on handle {:?}",
                            report_char.handle
                        );
                        rpc_log::info("HID connection ready - receiving reports");
                        let _ = BLE_EVENT_CHANNEL
                            .try_send(BleEvent::StateChanged(ConnectionState::Ready));

                        // HID report loop
                        loop {
                            match select(listener.next(), BLE_CMD_CHANNEL.receive()).await {
                                Either::First(notification) => {
                                    let data = notification.as_ref();
                                    if !data.is_empty() {
                                        let preview_len = data.len().min(8);
                                        debug!(
                                            "Raw notification: len={}, first {} bytes: {:?}",
                                            data.len(),
                                            preview_len,
                                            &data[..preview_len]
                                        );
                                        let report = parse_hid_report(data, 0, active_profile);
                                        HID_REPORT_CHANNEL.send(report).await;
                                    }
                                }
                                Either::Second(cmd) => {
                                    info!("Command received: {:?}", cmd);
                                    match cmd {
                                        BleCommand::Disconnect => {
                                            rpc_log::info("Disconnecting by request");
                                            conn.disconnect();
                                            let _ = select(
                                                async {
                                                    loop {
                                                        if let ConnectionEvent::Disconnected {
                                                            ..
                                                        } = conn.next().await
                                                        {
                                                            break;
                                                        }
                                                    }
                                                },
                                                embassy_time::Timer::after(
                                                    embassy_time::Duration::from_secs(1),
                                                ),
                                            )
                                            .await;
                                            return None;
                                        }
                                        BleCommand::Connect { .. } | BleCommand::AutoConnect => {
                                            conn.disconnect();
                                            let _ = select(
                                                async {
                                                    loop {
                                                        if let ConnectionEvent::Disconnected {
                                                            ..
                                                        } = conn.next().await
                                                        {
                                                            break;
                                                        }
                                                    }
                                                },
                                                embassy_time::Timer::after(
                                                    embassy_time::Duration::from_secs(1),
                                                ),
                                            )
                                            .await;
                                            return Some(cmd);
                                        }
                                        _ => {}
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        error!(
                            "Failed to subscribe to notifications: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        return None;
                    }
                }
            })
            .await;

            if let Some(cmd) = run_result.1 {
                return Some(cmd);
            }

            // Connection ended naturally
            break;
        }

        // Pairing failed - retry
        if pairing_attempts >= MAX_PAIRING_RETRIES {
            error!("Max pairing retries exceeded");
            return None;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
    }

    None
}
