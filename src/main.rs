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
mod bonding;
mod device_profile;
mod usb_hid;

use core::cell::RefCell;
use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::usb::Driver;
use embassy_time::{Duration, Timer};
use embassy_usb::class::hid::{HidWriter, State};
use embassy_usb::Builder;
use heapless::Deque;
use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use trouble_host::connection::{ConnectConfig, ScanConfig};
use trouble_host::scan::LeAdvReportsIter;
use trouble_host::scan::Scanner;

use usbd_hid::descriptor::{KeyboardReport, MouseReport, SerializedDescriptor};

use ble_hid::{parse_hid_report, HidReportType, HID_REPORT_CHANNEL};
use device_profile::DeviceProfile;
use usb_hid::{
    keyboard_report_descriptor, mouse_report_descriptor, serialize_keyboard_report,
    serialize_mouse_report, serialize_mouse_report_16bit, KeyboardHidReport, MouseHidReport,
    MouseReport16, UsbDeviceHandler, MOUSE_HIRES_16BIT_REPORT_DESC, MOUSE_HIRES_REPORT_DESC,
};

// ============ Target HID Device Configuration ============
/// Change this to switch which BLE device to scan for
const TARGET_PROFILE: DeviceProfile = DeviceProfile::FullScrollDial16Bit;

// HID Service and Characteristic UUIDs
const HID_REPORT_CHAR_UUID: Uuid = Uuid::new_short(0x2A4D); // HID Report
const HID_REPORT_MAP_CHAR_UUID: Uuid = Uuid::new_short(0x2A4B); // HID Report Map

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// Flash size for RP2040 (2MB)
const FLASH_SIZE: usize = 2 * 1024 * 1024;

/// Channel used to signal when the target device is found during scanning.
/// Carries the address of the found device.
static FOUND_DEVICE_CHANNEL: Channel<CriticalSectionRawMutex, Address, 1> = Channel::new();

struct ScannerHandler;

impl EventHandler for ScannerHandler {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = it.next() {
            let data = report.data;
            let mut i = 0;

            // Parse AD structures
            while i < data.len() {
                let len = data[i] as usize;
                if len == 0 {
                    break;
                } // Padding
                if i + 1 + len > data.len() {
                    break;
                } // Malformed

                let type_code = data[i + 1];
                let value = &data[i + 2..i + 1 + len];

                // AD Type 0x08 (Shortened Local Name) or 0x09 (Complete Local Name)
                if type_code == 0x08 || type_code == 0x09 {
                    if let Ok(name) = core::str::from_utf8(value) {
                        // Check if name matches our target device profile
                        if name == TARGET_PROFILE.name() {
                            info!("FOUND DEVICE: {} ({:?})", name, report.addr);
                            let _ = FOUND_DEVICE_CHANNEL.try_send(Address {
                                kind: report.addr_kind,
                                addr: report.addr,
                            });
                            return;
                        }
                    }
                }
                i += 1 + len;
            }
        }
    }
}

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

/// BLE advertisement event handler for scanning
/// Only logs HID devices
struct BleAdvHandler {
    /// Track seen device addresses to avoid duplicate logging
    seen: RefCell<Deque<BdAddr, 32>>,
}

impl BleAdvHandler {
    fn new() -> Self {
        Self {
            seen: RefCell::new(Deque::new()),
        }
    }
}

impl EventHandler for BleAdvHandler {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let mut seen = self.seen.borrow_mut();

        while let Some(Ok(report)) = it.next() {
            // Skip if already seen this device
            if seen.iter().any(|b| b.raw() == report.addr.raw()) {
                continue;
            }

            // Track this device
            if seen.is_full() {
                seen.pop_front();
            }
            let _ = seen.push_back(report.addr);

            // Get the raw address bytes for hex display
            let addr_bytes = report.addr.raw();

            // Parse advertisement data for device name and HID service
            let mut name: Option<&[u8]> = None;
            let mut is_hid = false;

            let data = report.data;
            let mut i = 0;
            while i < data.len() {
                let len = data[i] as usize;
                if len == 0 || i + len >= data.len() {
                    break;
                }
                let ad_type = data[i + 1];
                let ad_data = &data[i + 2..i + 1 + len];

                match ad_type {
                    0x09 | 0x08 => {
                        // Complete or Shortened Local Name
                        name = Some(ad_data);
                    }
                    0x02 | 0x03 => {
                        // Incomplete or Complete List of 16-bit Service UUIDs
                        for chunk in ad_data.chunks(2) {
                            if chunk.len() == 2 && chunk[0] == 0x12 && chunk[1] == 0x18 {
                                is_hid = true;
                            }
                        }
                    }
                    _ => {}
                }
                i += 1 + len;
            }

            // Only log HID devices
            if !is_hid {
                continue;
            }

            // Log the discovery with hex address
            if let Some(name_bytes) = name {
                if let Ok(name_str) = core::str::from_utf8(name_bytes) {
                    info!(
                        "Found HID device: {} [{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}]",
                        name_str,
                        addr_bytes[5],
                        addr_bytes[4],
                        addr_bytes[3],
                        addr_bytes[2],
                        addr_bytes[1],
                        addr_bytes[0]
                    );
                } else {
                    info!(
                        "Found HID device: [{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}]",
                        addr_bytes[5],
                        addr_bytes[4],
                        addr_bytes[3],
                        addr_bytes[2],
                        addr_bytes[1],
                        addr_bytes[0]
                    );
                }
            } else {
                info!(
                    "Found HID device: [{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}]",
                    addr_bytes[5],
                    addr_bytes[4],
                    addr_bytes[3],
                    addr_bytes[2],
                    addr_bytes[1],
                    addr_bytes[0]
                );
            }
        }
    }
}

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
    let mouse_descriptor = if TARGET_PROFILE.uses_16bit_reports() {
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
    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    static STATE_KB: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_MOUSE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

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

    let usb_dev = builder.build();

    // Spawn USB tasks
    unwrap!(spawner.spawn(usb_task(usb_dev)));
    unwrap!(spawner.spawn(usb_hid_handler_task(kb_writer, mouse_writer)));

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

    let controller = ExternalController::new(bt_device);

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

    let scanner_handler = ScannerHandler;
    let mut scanner = Scanner::new(central);

    info!("BLE Central initialized. Starting discovery...");

    // Run BLE host (with scanner handler) and connection logic concurrently
    let _ = join(runner.run_with_handler(&scanner_handler), async {
        // Track whether we have a valid stored bond
        let has_stored_bond = !loaded_bonds.is_empty();

        // Determine the active device profile:
        // - From stored bond if reconnecting
        // - From TARGET_PROFILE if scanning fresh
        let active_profile = if has_stored_bond {
            DeviceProfile::from_id(loaded_bonds[0].profile_id)
        } else {
            TARGET_PROFILE
        };
        info!("Active device profile: {:?}", active_profile);

        // --- DISCOVERY PHASE ---
        let target_address = if has_stored_bond {
            // If we have a bonded device, connect directly to it (skip scanning)
            let lb = &loaded_bonds[0];
            let addr = lb.bond.identity.bd_addr;
            info!("Bonded device found, connecting directly to {:?}", addr);

            // Convert BdAddr to Address
            let addr_bytes = addr.raw();
            let kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
                AddrKind::RANDOM
            } else {
                AddrKind::PUBLIC
            };

            Address { kind, addr }
        } else {
            // No bonds, scan for device by name
            loop {
                info!("Scanning for '{}'...", TARGET_PROFILE.name());

                // Start active scanning (needed to get names usually in scan response)
                let mut scan_config = ScanConfig::default();
                scan_config.active = true;
                scan_config.interval = embassy_time::Duration::from_millis(100);
                scan_config.window = embassy_time::Duration::from_millis(100);
                // No filter list = BasicUnfiltered

                match scanner.scan(&scan_config).await {
                    Ok(_scan_session) => {
                        // Scan started successfully. Wait for FOUND_DEVICE_CHANNEL
                        match embassy_time::with_timeout(
                            embassy_time::Duration::from_secs(10),
                            FOUND_DEVICE_CHANNEL.receive(),
                        )
                        .await
                        {
                            Ok(address) => {
                                info!("Target found! Stopping scan.");
                                break address; // Exit loop with address
                            }
                            Err(_) => {
                                info!("Scan timeout. Restarting scan...");
                                continue;
                            }
                        }
                        // session dropped here, stopping scan
                    }
                    Err(e) => {
                        error!("Failed to start scan: {:?}", e);
                        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
                    }
                }
            }
        };

        // --- CONNECTION PHASE ---
        // Get central back from scanner
        let mut central = scanner.into_inner();

        // Connect to the found target
        let mut config = ConnectConfig {
            connect_params: Default::default(),
            scan_config: ScanConfig {
                active: true,
                filter_accept_list: &[(target_address.kind, &target_address.addr)],
                ..Default::default()
            },
        };
        // Request low-latency connection parameters for high-performance mice
        // BLE intervals work in 1.25ms units, 7.5ms = 6 units (minimum allowed)
        config.connect_params.min_connection_interval = embassy_time::Duration::from_micros(7500); // 7.5ms
        config.connect_params.max_connection_interval = embassy_time::Duration::from_millis(15); // 15ms
        config.connect_params.supervision_timeout = embassy_time::Duration::from_secs(2);

        const MAX_PAIRING_RETRIES: u8 = 5;
        let mut pairing_attempts: u8 = 0;

        'connect: loop {
            pairing_attempts += 1;
            info!(
                "Connection attempt {} of {}",
                pairing_attempts, MAX_PAIRING_RETRIES
            );

            let conn = match central.connect(&config).await {
                Ok(conn) => conn,
                Err(e) => {
                    error!("Connection failed: {:?}", e);
                    if pairing_attempts >= MAX_PAIRING_RETRIES {
                        error!("Max connection retries exceeded, giving up");
                        return;
                    }
                    warn!("Retrying in 500ms...");
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
                    continue 'connect;
                }
            };

            info!("Connected to HID device!");

            // Set bondable based on whether we have an existing bond
            if let Err(e) = conn.set_bondable(!has_stored_bond) {
                error!("Failed to set bondable: {:?}", e);
            }
            info!(
                "Bondable set to: {} (has_stored_bond: {})",
                !has_stored_bond, has_stored_bond
            );

            // Request security (triggers pairing or re-encryption)
            info!(
                "Requesting security (has_stored_bond: {}, bondable: {})...",
                has_stored_bond, !has_stored_bond
            );
            match conn.request_security() {
                Ok(_) => info!("Security request sent"),
                Err(e) => {
                    error!("Failed to request security: {:?}", e);
                    if pairing_attempts >= MAX_PAIRING_RETRIES {
                        error!("Max pairing retries exceeded, giving up");
                        return;
                    }
                    warn!("Retrying in 500ms...");
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
                        if let Some(bond_info) = bond {
                            info!("Bond info received: {:?}", bond_info);
                            info!("Storing bond to flash...");
                            match bonding::store_bond(
                                &mut flash,
                                &bond_info,
                                active_profile.to_id(),
                            )
                            .await
                            {
                                Ok(slot) => {
                                    info!("Bond stored successfully in slot {}", slot);
                                }
                                Err(_) => {
                                    error!("Failed to store bond to flash");
                                }
                            }
                        }
                        break true;
                    }
                    ConnectionEvent::PairingFailed(e) => {
                        error!("Pairing failed: {:?}", e);
                        break false;
                    }
                    ConnectionEvent::Disconnected { reason } => {
                        error!("Disconnected during pairing: {:?}", reason);
                        break false;
                    }
                    other => {
                        debug!("Connection event during security setup: {:?}", other);
                    }
                }
            };

            if pairing_ok {
                info!("Connected, creating GATT client...");

                // Create GATT client to interact with services
                let client =
                    match GattClient::<ExternalController<_, 10>, DefaultPacketPool, 10>::new(
                        &stack, &conn,
                    )
                    .await
                    {
                        Ok(c) => c,
                        Err(e) => {
                            error!("Failed to create GATT client: {:?}", e);
                            return;
                        }
                    };

                // Run GATT client task and HID input handling concurrently
                let _ = join(client.task(), async {
                    // First, probe for common services to see what the device exposes
                    info!("Probing for common BLE services...");

                    let common_services: [(u16, &str); 5] = [
                        (0x1800, "Generic Access"),
                        (0x1801, "Generic Attribute"),
                        (0x180A, "Device Information"),
                        (0x180F, "Battery Service"),
                        (0x1812, "HID Service"),
                    ];

                    for (uuid, name) in common_services.iter() {
                        let service_uuid = Uuid::new_short(*uuid);
                        info!("  Checking for {} (0x{:04X})...", name, uuid);

                        match embassy_time::with_timeout(
                            embassy_time::Duration::from_secs(5),
                            client.services_by_uuid(&service_uuid),
                        )
                        .await
                        {
                            Ok(Ok(services)) if !services.is_empty() => {
                                info!("    FOUND: {} instance(s)", services.len());
                            }
                            Ok(Ok(_)) => {
                                info!("    Not found");
                            }
                            Ok(Err(e)) => {
                                warn!("    Error: {:?}", e);
                            }
                            Err(_) => {
                                error!("    TIMEOUT after 5s");
                            }
                        }
                    }

                    // Now discover HID service specifically
                    info!("Discovering HID service...");
                    let hid_service_uuid = Uuid::new_short(0x1812);

                    let services = match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(10),
                        client.services_by_uuid(&hid_service_uuid),
                    )
                    .await
                    {
                        Ok(Ok(services)) => services,
                        Ok(Err(e)) => {
                            error!("Failed to discover HID service: {:?}", e);
                            return;
                        }
                        Err(_) => {
                            error!("HID service discovery timed out after 10s");
                            return;
                        }
                    };

                    if services.is_empty() {
                        error!("No HID service found!");
                        return;
                    }

                    let hid_service = &services[0];
                    info!("Found HID service");

                    // Find HID Report characteristic (UUID 0x2A4D)
                    // Note: There may be multiple report characteristics, but we'll use the first one
                    // In a full implementation, we'd parse the Report Map to identify each one
                    info!("Discovering HID Report characteristic...");
                    let report_uuid = Uuid::new_short(0x2A4D);

                    let report_char: Characteristic<[u8; 64]> = match client
                        .characteristic_by_uuid(hid_service, &report_uuid)
                        .await
                    {
                        Ok(char) => char,
                        Err(e) => {
                            error!("Failed to discover report characteristic: {:?}", e);
                            return;
                        }
                    };

                    info!("Found HID Report characteristic");
                    info!("Subscribing to HID notifications...");

                    match client.subscribe(&report_char, false).await {
                        Ok(mut listener) => {
                            info!("Subscribed successfully!");
                            info!("=== HID CONNECTION ESTABLISHED ===");
                            info!("Waiting for mouse HID reports...");

                            loop {
                                let notification = listener.next().await;
                                let data = notification.as_ref();

                                if data.is_empty() {
                                    continue;
                                }
                                // debug!("HID Report: {:02x}", data);

                                let report = parse_hid_report(data, 0, active_profile);
                                HID_REPORT_CHANNEL.send(report).await;
                            }
                        }
                        Err(e) => {
                            error!("Failed to subscribe to notifications: {:?}", e);
                        }
                    }
                })
                .await;

                // GATT/HID loop ended (disconnect or error) — exit retry loop
                break;
            }

            // Pairing failed — retry
            if pairing_attempts >= MAX_PAIRING_RETRIES {
                error!("Max pairing retries exceeded, giving up");
                return;
            }
            warn!("Pairing failed, retrying in 500ms...");
            embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
            // conn is dropped here, releasing the connection
        }
    })
    .await;
}
