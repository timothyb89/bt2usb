//! Core 1 USB device setup and tasks — Magic Trackpad 2 emulation
//!
//! This module handles all USB functionality running on Core 1:
//! - Building the USB device as an Apple Magic Trackpad 2 with 4 HID interfaces
//! - Spawning USB tasks (device driver, HID report handler)
//! - Receiving BLE HID reports and forwarding as MT2 touch gestures
//!
//! Core 1 runs independently from Core 0 (CYW43 + BLE). Flash erase/write on
//! Core 0 briefly pauses Core 1 via FIFO, but USB handles NAKs gracefully.

use core::sync::atomic::Ordering;
use defmt::*;
use embassy_executor::Executor;
use embassy_futures::select::{select3, Either3};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_usb::class::hid::{HidReaderWriter, HidWriter};
use embassy_usb::control::InResponse;
use embassy_usb::types::StringIndex;
use static_cell::StaticCell;

use crate::ble_hid::{HidReportType, BATTERY_USB_SIGNAL, HID_REPORT_CHANNEL};
use crate::mt2;
use crate::{Irqs, CORE1_EXECUTOR_READY, EXECUTOR1};

// ============ USB Tasks ============

/// USB device task — handles USB enumeration and events (spawned on Core 1).
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// USB HID handler task — receives HID reports from BLE (Core 0) and converts
/// scroll deltas into MT2 touch gesture reports on the trackpad interface.
///
/// Handles three event sources concurrently via `select3`:
/// - HID_REPORT_CHANNEL: mouse reports from BLE (scroll deltas → touch gestures)
/// - BATTERY_USB_SIGNAL: battery level updates (not used yet in MT2 mode)
/// - Timer: idle timeout check to lift synthetic fingers
#[embassy_executor::task]
async fn usb_hid_handler_task(
    mut trackpad_writer: HidWriter<'static, Driver<'static, USB>, 64>,
) {
    info!("MT2 USB HID handler task started, waiting for BLE reports...");

    let mut touch_synth = mt2::TouchSynthesizer::new();

    loop {
        match select3(
            HID_REPORT_CHANNEL.receive(),
            BATTERY_USB_SIGNAL.wait(),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(50)),
        )
        .await
        {
            Either3::First(event) => {
                match event.report_type {
                    HidReportType::Mouse => {
                        if !mt2::MT_ENABLED.load(Ordering::Relaxed) {
                            // Multitouch not yet activated by host — skip
                            debug!("MT2: scroll event received but MT not yet enabled");
                            continue;
                        }

                        // Extract RAW scroll delta from BLE mouse report.
                        // In MT2 mode we need every delta immediately — the threshold
                        // accumulator in device_profile.rs is designed for standard mouse
                        // reports and would swallow small deltas. We bypass it entirely.
                        //
                        // Full Scroll Dial (16-bit): BLE report = 2-byte LE scroll delta.
                        // Other profiles (8-bit): scroll at byte 3 (after buttons, X, Y).
                        let scroll_delta: i16 = if event.profile.uses_16bit_reports() {
                            if event.len >= 2 {
                                i16::from_le_bytes([event.data[0], event.data[1]])
                            } else {
                                0
                            }
                        } else if event.len >= 4 {
                            event.data[3] as i8 as i16
                        } else {
                            0
                        };

                        if scroll_delta != 0 {
                            // Apply scroll multiplier
                            let scroll_m =
                                crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
                            let scaled = crate::usb_hid::apply_multiplier_i16(scroll_delta, scroll_m);

                            let report = touch_synth.process_scroll(scaled);
                            if let Err(e) = trackpad_writer.write(&report).await {
                                warn!("MT2 trackpad write error: {:?}", e);
                            }
                        }
                    }
                    _ => {
                        // Keyboard and other report types are ignored in MT2 mode
                        debug!("MT2: ignoring non-mouse report type {:?}", event.report_type);
                    }
                }
            }
            Either3::Second(_level) => {
                // Battery update — could send on Interface 0 battery report later
                debug!("MT2: battery update received (not forwarded yet)");
            }
            Either3::Third(_) => {
                // Idle timer fired — check if we need to lift fingers
                if let Some(lift_report) = touch_synth.check_idle() {
                    if let Err(e) = trackpad_writer.write(&lift_report).await {
                        warn!("MT2 trackpad lift write error: {:?}", e);
                    }
                }
            }
        }
    }
}

// ============ Core 1 USB Initialization ============

/// Build and start all USB tasks on Core 1.
///
/// This runs inside the `spawn_core1` closure and must not return. It:
/// 1. Enables TIMER_IRQ_0 on Core 1 (needed for embassy_time)
/// 2. Builds the USB device as an Apple Magic Trackpad 2 with 4 HID interfaces
/// 3. Starts the Core 1 executor with USB tasks
/// 4. Signals Core 0 that the executor is ready via `CORE1_EXECUTOR_READY`
pub fn start_core1_usb(usb: Peri<'static, USB>) -> ! {
    // Enable TIMER_IRQ_0 on Core 1's NVIC so embassy_time works
    unsafe {
        embassy_rp::interrupt::InterruptExt::set_priority(
            embassy_rp::interrupt::TIMER_IRQ_0,
            embassy_rp::interrupt::Priority::P3,
        );
        embassy_rp::interrupt::InterruptExt::enable(embassy_rp::interrupt::TIMER_IRQ_0);
    }

    // Build USB device as Apple Magic Trackpad 2
    let driver = Driver::new(usb, Irqs);
    let mut config = embassy_usb::Config::new(mt2::APPLE_VID, mt2::MT2_PID);
    config.manufacturer = Some(mt2::MT2_MANUFACTURER);
    config.product = Some(mt2::MT2_PRODUCT);
    config.serial_number = Some("12345678");
    config.device_release = mt2::MT2_DEVICE_RELEASE;
    config.max_power = 500; // MT2 reports 500mA
    config.max_packet_size_0 = 64;
    // Real MT2 is USB 2.0 (bcdUSB=0x0200). embassy-usb defaults to USB 2.1 (0x0210),
    // which triggers BOS descriptor requests that a real MT2 wouldn't support.
    // TopCase may use bcdUSB for device identification.
    config.bcd_usb = embassy_usb::UsbVersion::Two;
    // Real MT2 has bmAttributes=0xA0 (Bus Powered + Remote Wakeup)
    config.supports_remote_wakeup = true;
    // Real MT2 has device class 0/0/0 — disable IAD mode which forces 0xEF/0x02/0x01
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    // Interface 0: Device Management (FF00/0B) — request handler for activation
    // Real: EP IN wMaxPacketSize=16, bInterval=8
    let if0_config = embassy_usb::class::hid::Config {
        report_descriptor: mt2::DEVICE_MGMT_REPORT_DESC,
        request_handler: None, // set below via StaticCell
        poll_ms: 8,
        max_packet_size: 16,
    };

    // Interface 1: Mouse/Trackpad (01/02) — main touch data interface
    // Real: EP IN wMaxPacketSize=64, bInterval=1
    let if1_config = embassy_usb::class::hid::Config {
        report_descriptor: mt2::TRACKPAD_REPORT_DESC,
        request_handler: None, // set below via StaticCell
        poll_ms: 1,
        max_packet_size: 64,
    };

    // Interface 2: Vendor (FF00/0D) — has IN + OUT in real device
    // Real: EP IN wMaxPacketSize=16 bInterval=8, EP OUT wMaxPacketSize=64 bInterval=2
    // embassy-usb uses one max_packet_size for both; we set 64 (for OUT) and patch IN below
    let if2_config = embassy_usb::class::hid::Config {
        report_descriptor: mt2::VENDOR2_REPORT_DESC,
        request_handler: None,
        poll_ms: 8,
        max_packet_size: 64,
    };

    // Interface 3: Vendor (FF00/03) — stub
    // Real: EP IN wMaxPacketSize=64, bInterval=2
    let if3_config = embassy_usb::class::hid::Config {
        report_descriptor: mt2::VENDOR3_REPORT_DESC,
        request_handler: None,
        poll_ms: 2,
        max_packet_size: 64,
    };

    // State buffers — increased for 4 interfaces
    static CONFIG_DESC: StaticCell<[u8; 512]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new(); // largest feature = 76 bytes

    static STATE_IF0: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_IF1: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_IF2: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_IF3: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

    static HANDLER_IF0: StaticCell<mt2::Mt2DeviceMgmtRequestHandler> = StaticCell::new();
    static HANDLER_IF1: StaticCell<mt2::Mt2TrackpadRequestHandler> = StaticCell::new();
    static HANDLER_IF2: StaticCell<mt2::Mt2VendorActuatorRequestHandler> = StaticCell::new();
    static DEVICE_HANDLER: StaticCell<Mt2UsbDeviceHandler> = StaticCell::new();

    let config_desc_buf = CONFIG_DESC.init([0; 512]);
    // Save raw pointer for post-build patching of IF1 subclass/protocol
    let config_desc_ptr = config_desc_buf.as_mut_ptr();
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        config_desc_buf,
        BOS_DESC.init([0; 256]),
        MS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 128]),
    );
    builder.handler(DEVICE_HANDLER.init(Mt2UsbDeviceHandler));

    // Interface 0: Device Management — HidReaderWriter (has input reports E0/9A/90)
    // We use HidReaderWriter in case macOS needs the output endpoint, but primarily
    // use the request handler for feature reports.
    let if0_config_with_handler = embassy_usb::class::hid::Config {
        request_handler: Some(HANDLER_IF0.init(mt2::Mt2DeviceMgmtRequestHandler)),
        ..if0_config
    };
    let _if0_hid = HidWriter::<_, 8>::new(
        &mut builder,
        STATE_IF0.init(embassy_usb::class::hid::State::new()),
        if0_config_with_handler,
    );

    // Interface 1: Mouse/Trackpad — HidWriter for touch reports
    let if1_config_with_handler = embassy_usb::class::hid::Config {
        request_handler: Some(HANDLER_IF1.init(mt2::Mt2TrackpadRequestHandler::new())),
        ..if1_config
    };
    let trackpad_writer = HidWriter::<_, 64>::new(
        &mut builder,
        STATE_IF1.init(embassy_usb::class::hid::State::new()),
        if1_config_with_handler,
    );

    // Interface 2: Vendor stub (FF00/0D) — has IN + OUT in real device
    let if2_config_with_handler = embassy_usb::class::hid::Config {
        request_handler: Some(HANDLER_IF2.init(mt2::Mt2VendorActuatorRequestHandler)),
        ..if2_config
    };
    let if2_hid = HidReaderWriter::<_, 64, 64>::new(
        &mut builder,
        STATE_IF2.init(embassy_usb::class::hid::State::new()),
        if2_config_with_handler,
    );
    let (_if2_reader, _if2_writer) = if2_hid.split();

    // Interface 3: Vendor stub (FF00/03) — input only
    let _if3_writer = HidWriter::<_, 64>::new(
        &mut builder,
        STATE_IF3.init(embassy_usb::class::hid::State::new()),
        if3_config,
    );

    let usb_dev = builder.build();

    // Patch config descriptor to match real MT2.
    // embassy-usb HID class hardcodes subclass=0, protocol=0, iInterface=0 for all interfaces.
    // Also patches endpoint wMaxPacketSize/bInterval where embassy-usb can't set per-endpoint values.
    // Real MT2 has:
    //   IF0: iInterface="Device Management"
    //   IF1: subclass=0x01 (Boot), protocol=0x02 (Mouse), iInterface="Trackpad / Boot"
    //   IF2: iInterface="Actuator", EP IN wMaxPacketSize=16, EP OUT bInterval=2
    //   IF3: iInterface="Accelerometer"
    // String indices 4-7 are served by Mt2UsbDeviceHandler::get_string().
    unsafe {
        let desc = core::slice::from_raw_parts_mut(config_desc_ptr, 512);

        // Patch iConfiguration in config descriptor header (byte 6).
        // Real MT2 has iConfiguration=4 with string "Trackpad".
        if desc[0] == 9 && desc[1] == 2 {
            desc[6] = MT2_CONFIG_STRING_IDX;
        }

        let mut current_iface: u8 = 0xFF;
        let mut i = 0;
        while i < desc.len().saturating_sub(7) {
            let b_length = desc[i] as usize;
            let b_desc_type = desc[i + 1];
            if b_length < 2 || i + b_length > desc.len() {
                break;
            }

            // Interface descriptor: bLength=9, bDescriptorType=4
            if b_desc_type == 4 && b_length >= 9 {
                current_iface = desc[i + 2];
                let alt_setting = desc[i + 3];
                if alt_setting == 0 {
                    match current_iface {
                        0 => {
                            desc[i + 8] = MT2_IFACE_STRING_BASE; // iInterface
                        }
                        1 => {
                            desc[i + 6] = 0x01; // bInterfaceSubClass = Boot Interface
                            desc[i + 7] = 0x02; // bInterfaceProtocol = Mouse
                            desc[i + 8] = MT2_IFACE_STRING_BASE + 1; // iInterface
                        }
                        2 => {
                            desc[i + 8] = MT2_IFACE_STRING_BASE + 2; // iInterface
                        }
                        3 => {
                            desc[i + 8] = MT2_IFACE_STRING_BASE + 3; // iInterface
                        }
                        _ => {}
                    }
                }
            }

            // Endpoint descriptor: bLength=7, bDescriptorType=5
            if b_desc_type == 5 && b_length >= 7 && current_iface == 2 {
                let ep_addr = desc[i + 2];
                if ep_addr & 0x80 != 0 {
                    // IF2 IN endpoint: wMaxPacketSize 64→16 (real device)
                    desc[i + 4] = 16; // wMaxPacketSize low byte
                    desc[i + 5] = 0;  // wMaxPacketSize high byte
                } else {
                    // IF2 OUT endpoint: bInterval 8→2 (real device)
                    desc[i + 6] = 2; // bInterval
                }
            }

            i += b_length;
        }
        info!("Patched config descriptor: iInterface strings, IF1 boot, IF2 endpoints");
    }

    info!("[core1] MT2 USB device initialized (4 interfaces)");

    let executor1 = EXECUTOR1.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(usb_task(usb_dev)).unwrap();
        spawner
            .spawn(usb_hid_handler_task(trackpad_writer))
            .unwrap();
        // RPC task dropped for MT2 mode
        CORE1_EXECUTOR_READY.store(true, Ordering::Release);
        cortex_m::asm::sev();
    });
}

// ============ USB Device Handler ============

/// String index for iConfiguration (matches real MT2 index 4).
const MT2_CONFIG_STRING_IDX: u8 = 4;

/// Base string index for interface strings (matches real MT2 indices 5-8).
/// IF0=5 "Device Management", IF1=6 "Trackpad / Boot", IF2=7 "Actuator", IF3=8 "Accelerometer".
const MT2_IFACE_STRING_BASE: u8 = 5;

/// Interface string names matching the real MT2 (from ioreg kUSBString).
const MT2_IFACE_STRINGS: [&str; 4] = [
    "Device Management",
    "Trackpad / Boot",
    "Actuator",
    "Accelerometer",
];

/// MT2 USB device handler — logs state changes, serves interface strings, resets activation on bus reset.
struct Mt2UsbDeviceHandler;

impl embassy_usb::Handler for Mt2UsbDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if enabled {
            info!("MT2 USB device enabled");
        } else {
            info!("MT2 USB device disabled");
        }
    }

    fn reset(&mut self) {
        mt2::MT_ENABLED.store(false, Ordering::Relaxed);
        info!("MT2 USB bus reset, multitouch disabled");
    }

    fn addressed(&mut self, addr: u8) {
        debug!("MT2 USB address set to {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        mt2::MT_ENABLED.store(false, Ordering::Relaxed);
        if configured {
            info!("MT2 USB device configured, multitouch reset");
        } else {
            info!("MT2 USB device unconfigured");
        }
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            debug!("MT2 USB suspended");
        } else {
            debug!("MT2 USB resumed");
        }
    }

    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        let idx: u8 = index.into();
        if idx == MT2_CONFIG_STRING_IDX {
            Some("Trackpad")
        } else if idx >= MT2_IFACE_STRING_BASE && idx < MT2_IFACE_STRING_BASE + 4 {
            Some(MT2_IFACE_STRINGS[(idx - MT2_IFACE_STRING_BASE) as usize])
        } else {
            None
        }
    }

    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        // Only handle Vendor requests — Standard/Class must pass through to HID class handlers
        match req.request_type {
            embassy_usb::control::RequestType::Vendor => {
                info!(
                    "MT2 USB CTRL OUT vendor: req=0x{:02X} val=0x{:04X} idx=0x{:04X} len={}",
                    req.request, req.value, req.index, data.len()
                );
                Some(embassy_usb::control::OutResponse::Accepted)
            }
            _ => None,
        }
    }

    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        _buf: &'a mut [u8],
    ) -> Option<InResponse<'a>> {
        match req.request_type {
            embassy_usb::control::RequestType::Vendor => {
                info!(
                    "MT2 USB CTRL IN vendor: req=0x{:02X} val=0x{:04X} idx=0x{:04X} len={}",
                    req.request, req.value, req.index, req.length
                );
                Some(InResponse::Accepted(&[]))
            }
            embassy_usb::control::RequestType::Class => {
                // Log Class IN requests (GET_REPORT etc.) for diagnosis — don't handle them,
                // let embassy-usb's HID class handler process them.
                // GET_REPORT: bRequest=0x01, wValue = (ReportType<<8 | ReportID)
                if req.request == 0x01 {
                    let report_type = (req.value >> 8) as u8;
                    let report_id = (req.value & 0xFF) as u8;
                    debug!(
                        "MT2 USB GET_REPORT: type={} id=0x{:02X} iface={} wLength={}",
                        report_type, report_id, req.index, req.length
                    );
                }
                None // pass through to HID class handler
            }
            _ => None,
        }
    }
}
