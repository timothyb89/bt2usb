//! bt2usb - BLE HID to USB HID Bridge
//!
//! This firmware runs on a Raspberry Pi Pico W and:
//! 1. Acts as a BLE Central, connecting to HID peripherals (keyboards, mice)
//! 2. Translates BLE HID reports to USB HID reports
//! 3. Presents itself as a USB HID device to the host computer
//!
//! Phase 3: BLE Central + USB HID composite device

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

use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
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

/// CYW43 task - required to run the WiFi/BLE chip
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

/// USB device task - handles USB enumeration and events
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// USB HID handler task - receives HID reports from BLE and sends to USB
#[embassy_executor::task]
async fn usb_hid_handler_task(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mouse_writer: HidWriter<'static, Driver<'static, USB>, 7>, // Changed to 7 to support 16-bit mode
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
                    // Check if profile uses 16-bit reports
                    if event.profile.uses_16bit_reports() {
                        // 16-bit mode: full precision, no scaling
                        let report = event
                            .profile
                            .translate_mouse_report_16bit(&event.data, event.len);
                        let buf = serialize_mouse_report_16bit(&report);
                        if let Err(e) = mouse_writer.write(&buf).await {
                            warn!("Mouse write error (16-bit): {:?}", e);
                        }
                    } else {
                        // 8-bit mode: standard with scaling
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

/// LED blink task using CYW43
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

/// Main entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("bt2usb starting...");

    let p = embassy_rp::init(Default::default());

    // ============ CYW43 Initialization with Bluetooth ============
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let btfw = include_bytes!("../cyw43-firmware/43439A0_btfw.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;

    // Spawn CYW43 background task
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    info!("CYW43 initialized with Bluetooth");

    // ============ Flash Storage Initialization ============
    // Initialize flash driver for bond storage (using DMA_CH1 since CH0 is used for PIO)
    info!("Initializing flash storage for bonding...");
    let mut flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH1);

    // DEVELOPMENT: Uncomment to clear bonds on every boot
    info!("Clearing all bonds...");
    bonding::clear_all_bonds(&mut flash).await.ok();

    // ============ USB HID Initialization ============
    let driver = Driver::new(p.USB, Irqs);
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("momentary");
    config.product = Some("BT2USB Bridge");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create HID descriptors
    // Boot Keyboard
    let kb_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };
    // Mouse (select descriptor based on TARGET_PROFILE)
    static MOUSE_HANDLER: StaticCell<usb_hid::HiresMouseRequestHandler> = StaticCell::new();
    let mouse_descriptor = if DEFAULT_PROFILE.uses_16bit_reports() {
        MOUSE_HIRES_16BIT_REPORT_DESC // Experimental 16-bit mode
    } else {
        MOUSE_HIRES_REPORT_DESC // Standard 8-bit mode
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

    // Create HID interfaces
    let kb_writer = HidWriter::<_, 8>::new(
        &mut builder,
        STATE_KB.init(embassy_usb::class::hid::State::new()),
        kb_config,
    );

    let mouse_writer = HidWriter::<_, 7>::new(
        // 7 bytes to support 16-bit mode
        &mut builder,
        STATE_MOUSE.init(embassy_usb::class::hid::State::new()),
        mouse_config,
    );

    // Vendor HID interface for RPC communication (replaces CDC ACM to avoid BLE interference)
    let rpc_config = embassy_usb::class::hid::Config {
        report_descriptor: VENDOR_RPC_REPORT_DESC,
        request_handler: None,
        poll_ms: 10, // 10ms poll — fast enough for streaming events without excessive USB load
        max_packet_size: 64,
    };
    let rpc_hid = HidReaderWriter::<_, 64, 64>::new(
        &mut builder,
        STATE_RPC.init(embassy_usb::class::hid::State::new()),
        rpc_config,
    );
    let (rpc_reader, rpc_writer) = rpc_hid.split();

    let usb_dev = builder.build();

    // Spawn USB tasks
    unwrap!(spawner.spawn(usb_task(usb_dev)));
    unwrap!(spawner.spawn(usb_hid_handler_task(kb_writer, mouse_writer)));
    unwrap!(spawner.spawn(rpc::rpc_task(rpc_writer, rpc_reader)));

    info!("USB HID device initialized");

    // ============ BLE Central Logic ============
    info!("Initializing BLE Central...");

    // Load stored bonds from flash
    let loaded_bonds = bonding::load_bonds(&mut flash).await;
    info!("Loaded {} stored bond(s) from flash", loaded_bonds.len());
    for lb in &loaded_bonds {
        info!(
            "  - Device: {:?} (profile: {:?})",
            lb.bond.identity.bd_addr,
            DeviceProfile::from_id(lb.profile_id)
        );
    }

    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    // Static random address: MSB must have bits 11 (>= 0xC0)
    let address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xca]);
    info!("Our BLE address: {:?}", address);

    static RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let resources = RESOURCES.init(HostResources::new());

    // Create RNG from RP2040's Ring Oscillator (ROSC) for hardware entropy
    use embassy_rp::clocks::RoscRng;
    // use rand_core::RngCore; // Unused
    let mut rosc_rng = RoscRng;
    let mut rng_seed = [0u8; 32];
    rosc_rng.fill_bytes(&mut rng_seed);
    info!("RNG Seed: {:02x}", rng_seed);

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
                info!("Added bond for {:?} to BLE stack", lb.bond.identity.bd_addr);
            }
            Err(e) => {
                error!(
                    "Failed to add bond for {:?}: {:?}",
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

    info!("BLE Central initialized");

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

        loop {
            let _ =
                BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Disconnected));
            info!("BLE state: Idle. Waiting for command...");

            let cmd = BLE_CMD_CHANNEL.receive().await;
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
                        Ok(_session) => {
                            // Scan is active. ScanResult events are emitted by RpcScannerHandler.
                            // Wait for StopScan or Connect command, or timeout.
                            loop {
                                match select(
                                    BLE_CMD_CHANNEL.receive(),
                                    embassy_time::Timer::after(embassy_time::Duration::from_secs(
                                        30,
                                    )),
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
                                        // Session dropped here stops scanning.
                                        // Push Connect command back for the outer loop to handle.
                                        let _ = BLE_CMD_CHANNEL
                                            .try_send(BleCommand::Connect { address, addr_kind });
                                        break;
                                    }
                                    Either::First(_) => {
                                        // Ignore other commands during scan
                                    }
                                    Either::Second(_) => {
                                        info!("Scan timeout (30s)");
                                        rpc_log::info("Scan timeout (30s)");
                                        break;
                                    }
                                }
                            }
                            // Scan session dropped here, scanning stops
                        }
                        Err(e) => {
                            error!("Failed to start scan: {:?}", e);
                        }
                    }
                }

                BleCommand::StopScan => {
                    // Not scanning, ignore
                }

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
                    active_profile = DEFAULT_PROFILE; // Will be updated if bond has a profile

                    // Get central back from scanner for connection
                    let mut central = scanner.into_inner();
                    ble_connect_and_run(
                        &mut central,
                        &stack,
                        &mut flash,
                        target,
                        active_profile,
                        false,
                    )
                    .await;
                    // Recover scanner for next scan
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
                    ble_connect_and_run(
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
}

/// Connect to a BLE HID device, pair, discover GATT, and run the HID report loop.
///
/// Returns when the connection is lost, pairing fails, or an error occurs.
async fn ble_connect_and_run<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &mut Flash<'_, FLASH, Async, FLASH_SIZE>,
    target: Address,
    active_profile: DeviceProfile,
    has_stored_bond: bool,
) {
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

        let conn: Connection<'_, DefaultPacketPool> = match central.connect(&config).await {
            Ok(conn) => conn,
            Err(e) => {
                error!("Connection failed: {:?}", defmt::Debug2Format(&e));
                rpc_log::error("Connection attempt failed");
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    error!("Max connection retries exceeded");
                    rpc_log::error("Max connection retries exceeded");
                    return;
                }
                embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
                continue 'connect;
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
                    return;
                }
                embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
                continue 'connect;
            }
        }

        // Wait for pairing outcome
        let pairing_ok = loop {
            match conn.next().await {
                ConnectionEvent::PairingComplete {
                    security_level,
                    bond,
                } => {
                    info!("Pairing complete! Level: {:?}", security_level);
                    rpc_log::info("Pairing complete");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingComplete);
                    if let Some(bond_info) = bond {
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
                    break true;
                }
                ConnectionEvent::PairingFailed(e) => {
                    error!("Pairing failed: {:?}", e);
                    rpc_log::error("Pairing failed");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                    break false;
                }
                ConnectionEvent::Disconnected { reason } => {
                    error!("Disconnected during pairing: {:?}", reason);
                    rpc_log::error("Disconnected during pairing");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                    break false;
                }
                _ => {}
            }
        };

        if pairing_ok {
            info!("Creating GATT client...");

            let client = match GattClient::<C, DefaultPacketPool, 10>::new(stack, &conn).await {
                Ok(c) => c,
                Err(e) => {
                    error!(
                        "Failed to create GATT client: {:?}",
                        defmt::Debug2Format(&e)
                    );
                    return;
                }
            };

            // Run GATT client and HID loop concurrently
            let _ = join(client.task(), async {
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
                        return;
                    }
                    Ok(Err(e)) => {
                        error!(
                            "HID service discovery failed: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        return;
                    }
                    Err(_) => {
                        error!("HID service discovery timeout");
                        return;
                    }
                };

                let hid_service = &services[0];
                info!("Found HID service");
                rpc_log::info("HID service discovered");

                // Discover HID Report characteristic
                // NOTE: The Full Scroll Dial has multiple Report characteristics.
                // characteristic_by_uuid() returns the FIRST one, which may not be
                // the correct one for scroll wheel deltas!
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
                        return;
                    }
                };

                // TODO: This subscribes to the FIRST Report characteristic, but the device
                // likely has multiple. We may need to enumerate all Report characteristics
                // and check their Report Reference descriptors to find the scroll wheel report.

                // Subscribe to notifications
                match client.subscribe(&report_char, false).await {
                    Ok(mut listener) => {
                        info!("=== HID CONNECTION ESTABLISHED ===");
                        info!("Subscribed to notifications on handle {:?}", report_char.handle);
                        rpc_log::info("HID connection ready - receiving reports");
                        let _ = BLE_EVENT_CHANNEL
                            .try_send(BleEvent::StateChanged(ConnectionState::Ready));

                        // HID report loop - also check for Disconnect commands
                        loop {
                            match select(listener.next(), BLE_CMD_CHANNEL.receive()).await {
                                Either::First(notification) => {
                                    let data = notification.as_ref();
                                    if !data.is_empty() {
                                        let preview_len = data.len().min(8);
                                        debug!("Raw notification: len={}, first {} bytes: {:?}",
                                            data.len(), preview_len, &data[..preview_len]);
                                        let report = parse_hid_report(data, 0, active_profile);
                                        HID_REPORT_CHANNEL.send(report).await;
                                    }
                                }
                                Either::Second(BleCommand::Disconnect) => {
                                    info!("Disconnect command received");
                                    rpc_log::info("Disconnecting by request");
                                    break;
                                }
                                Either::Second(_) => {
                                    // Ignore other commands while connected
                                }
                            }
                        }
                    }
                    Err(e) => {
                        error!(
                            "Failed to subscribe to notifications: {:?}",
                            defmt::Debug2Format(&e)
                        );
                    }
                }
            })
            .await;

            // Connection ended
            break;
        }

        // Pairing failed — retry
        if pairing_attempts >= MAX_PAIRING_RETRIES {
            error!("Max pairing retries exceeded");
            return;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
    }
}
