//! Core 1 USB device setup and tasks
//!
//! This module handles all USB functionality running on Core 1:
//! - Building the USB device with keyboard, mouse, and vendor HID interfaces
//! - Conditionally adding MT2 trackpad interface (macOS only)
//! - Spawning USB tasks (device driver, HID report handler, RPC)
//! - Receiving BLE HID reports and forwarding them as USB HID reports
//!
//! Core 1 runs independently from Core 0 (CYW43 + BLE). Flash erase/write on
//! Core 0 briefly pauses Core 1 via FIFO, but USB handles NAKs gracefully.

use core::sync::atomic::Ordering;
use defmt::*;
use embassy_executor::Executor;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_usb::class::hid::{HidReaderWriter, HidWriter};
use static_cell::StaticCell;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use crate::ble_hid::{HidReportEvent, HidReportType, BATTERY_USB_SIGNAL, HID_REPORT_CHANNEL};
use crate::scratch::DetectedOs;
use crate::usb_hid::{
    serialize_keyboard_report, serialize_mouse_report_16bit, KeyboardHidReport,
    MOUSE_HIRES_16BIT_REPORT_DESC, VENDOR_RPC_REPORT_DESC,
};
use crate::{rpc, Irqs, CORE1_EXECUTOR_READY, EXECUTOR1};

// ============ USB Device Identity ============

const BT2USB_VID: u16 = 0x1209; // pid.codes open VID
const BT2USB_PID: u16 = 0x0001; // TODO: register a proper PID
const BT2USB_MANUFACTURER: &str = "bt2usb";
const BT2USB_PRODUCT: &str = "BLE HID Bridge";

// ============ USB Tasks ============

/// USB device task — handles USB enumeration and events (spawned on Core 1).
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

// ============ Shared Report Handlers ============

/// Handle a keyboard HID report event.
async fn handle_keyboard_report(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 8>,
    event: &HidReportEvent,
) {
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
        if let Err(e) = writer.write(&buf).await {
            warn!("Keyboard write error: {:?}", e);
        }
    }
}

/// Handle a mouse HID report event (standard path, no MT2 routing).
async fn handle_mouse_report_standard(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 8>,
    event: &HidReportEvent,
    scroll_accum: &mut crate::device_profile::ScrollAccumState,
) {
    if event.len < 1 {
        return;
    }

    // Layout-aware path: when Generic profile has a parsed Report Map, use it
    // for accurate field extraction instead of heuristics.
    if event.profile == crate::device_profile::DeviceProfile::Generic {
        if let Some(layout) = crate::ble_hid::find_slot_layout(event.slot_index as usize, event.len)
        {
            // BLE HID-over-GATT (HOGP) never includes the report ID in notification
            // data — it's implied by the characteristic handle. Use data as-is.
            let report_data = &event.data[..event.len];

            let mut report16 = crate::usb_hid::MouseReport16 {
                buttons: layout
                    .buttons
                    .map(|loc| crate::hid_report_map::extract_field(report_data, &loc) as u8)
                    .unwrap_or(0)
                    & 0x1F,
                x: layout
                    .x
                    .map(|loc| {
                        crate::hid_report_map::extract_field(report_data, &loc).clamp(-127, 127)
                            as i8
                    })
                    .unwrap_or(0),
                y: layout
                    .y
                    .map(|loc| {
                        crate::hid_report_map::extract_field(report_data, &loc).clamp(-127, 127)
                            as i8
                    })
                    .unwrap_or(0),
                wheel: layout
                    .wheel
                    .map(|loc| {
                        crate::hid_report_map::extract_field(report_data, &loc)
                            .clamp(i16::MIN as i32, i16::MAX as i32) as i16
                    })
                    .unwrap_or(0),
                pan: layout
                    .pan
                    .map(|loc| {
                        crate::hid_report_map::extract_field(report_data, &loc)
                            .clamp(i16::MIN as i32, i16::MAX as i32) as i16
                    })
                    .unwrap_or(0),
            };

            let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
            let pan_m = crate::usb_hid::MULTIPLIER_PAN.load(Ordering::Relaxed);
            let x_m = crate::usb_hid::MULTIPLIER_X.load(Ordering::Relaxed);
            let y_m = crate::usb_hid::MULTIPLIER_Y.load(Ordering::Relaxed);
            report16.x = crate::usb_hid::apply_multiplier_i8(report16.x, x_m);
            report16.y = crate::usb_hid::apply_multiplier_i8(report16.y, y_m);
            report16.wheel = crate::usb_hid::apply_multiplier_i16(report16.wheel, scroll_m);
            report16.pan = crate::usb_hid::apply_multiplier_i16(report16.pan, pan_m);

            // Apply hires scroll scaling only for standard mice (≤8-bit wheel).
            // Devices with 16-bit wheel fields already send fine-grained values.
            let wheel_is_hires_native = layout.wheel.is_some_and(|l| l.bit_size > 8);
            let pan_is_hires_native = layout.pan.is_some_and(|l| l.bit_size > 8);
            if crate::usb_hid::HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
                if !wheel_is_hires_native {
                    report16.wheel = report16.wheel.saturating_mul(120);
                }
                if !pan_is_hires_native {
                    report16.pan = report16.pan.saturating_mul(120);
                }
            }

            let data = serialize_mouse_report_16bit(&report16);
            let mut buf = [0u8; 8];
            buf[0] = 0x01;
            buf[1..].copy_from_slice(&data);
            if let Err(e) = writer.write(&buf).await {
                warn!("Mouse write error (layout): {:?}", e);
            }
            return;
        }
    }

    if event.profile.uses_16bit_reports() {
        let mut report =
            event
                .profile
                .translate_mouse_report_16bit(&event.data, event.len, scroll_accum);
        let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
        let pan_m = crate::usb_hid::MULTIPLIER_PAN.load(Ordering::Relaxed);
        let x_m = crate::usb_hid::MULTIPLIER_X.load(Ordering::Relaxed);
        let y_m = crate::usb_hid::MULTIPLIER_Y.load(Ordering::Relaxed);
        report.x = crate::usb_hid::apply_multiplier_i8(report.x, x_m);
        report.y = crate::usb_hid::apply_multiplier_i8(report.y, y_m);
        report.wheel = crate::usb_hid::apply_multiplier_i16(report.wheel, scroll_m);
        report.pan = crate::usb_hid::apply_multiplier_i16(report.pan, pan_m);
        let data = serialize_mouse_report_16bit(&report);
        let mut buf = [0u8; 8];
        buf[0] = 0x01;
        buf[1..].copy_from_slice(&data);
        if let Err(e) = writer.write(&buf).await {
            warn!("Mouse write error (16-bit): {:?}", e);
        }
    } else {
        let mut report = event
            .profile
            .translate_mouse_report(&event.data, event.len, scroll_accum);
        let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
        let pan_m = crate::usb_hid::MULTIPLIER_PAN.load(Ordering::Relaxed);
        let x_m = crate::usb_hid::MULTIPLIER_X.load(Ordering::Relaxed);
        let y_m = crate::usb_hid::MULTIPLIER_Y.load(Ordering::Relaxed);
        report.x = crate::usb_hid::apply_multiplier_i8(report.x, x_m);
        report.y = crate::usb_hid::apply_multiplier_i8(report.y, y_m);
        report.wheel = crate::usb_hid::apply_multiplier_i8(report.wheel, scroll_m);
        report.pan = crate::usb_hid::apply_multiplier_i8(report.pan, pan_m);
        // USB descriptor always uses 16-bit wheel/pan — promote to MouseReport16.
        // When hires scroll is active, the OS expects scroll in 1/120th-notch units
        // (per the Resolution Multiplier in the descriptor). Standard mice send ±1
        // per detent, so scale by 120 to get one full notch per click.
        let hires_scale: i16 = if crate::usb_hid::HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
            120
        } else {
            1
        };
        let report16 = crate::usb_hid::MouseReport16 {
            buttons: report.buttons,
            x: report.x,
            y: report.y,
            wheel: (report.wheel as i16) * hires_scale,
            pan: (report.pan as i16) * hires_scale,
        };
        let data = serialize_mouse_report_16bit(&report16);
        let mut buf = [0u8; 8];
        buf[0] = 0x01;
        buf[1..].copy_from_slice(&data);
        if let Err(e) = writer.write(&buf).await {
            warn!("Mouse write error (8-bit): {:?}", e);
        }
    }
}

/// Send battery level on the mouse interface (Report ID 2).
async fn send_battery_level(writer: &mut HidWriter<'static, Driver<'static, USB>, 8>, level: u8) {
    let _ = embassy_time::with_timeout(
        embassy_time::Duration::from_secs(5),
        writer.write(&[0x02, level]),
    )
    .await;
}

/// Check if a reprobe has been requested (USB switch detected) and reset if so.
fn check_reprobe() {
    if crate::usb_hid::REPROBE_REQUESTED.load(Ordering::Relaxed) {
        info!("Reprobe requested, resetting for Phase 0...");
        crate::scratch::clear_for_reprobe();
        crate::system_reset();
    }
}

// ============ Standard HID Handler Task (Windows/Linux/Unknown) ============

/// USB HID handler task for standard mode (no MT2).
///
/// Handles keyboard and mouse reports from BLE + battery level updates.
/// Uses `select` with 2 branches (no MT2 tick timer).
#[embassy_executor::task]
async fn usb_hid_handler_task_standard(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mouse_writer: HidWriter<'static, Driver<'static, USB>, 8>,
) {
    info!("USB HID handler task started (standard mode), waiting for BLE reports...");

    // Per-slot scroll accumulators for multi-device support
    let mut scroll_accums = [
        crate::device_profile::ScrollAccumState::new(),
        crate::device_profile::ScrollAccumState::new(),
        crate::device_profile::ScrollAccumState::new(),
    ];

    // Send initial battery level if already known
    let initial_level = crate::ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed);
    if initial_level != 0xFF {
        send_battery_level(&mut mouse_writer, initial_level).await;
    }

    loop {
        check_reprobe();

        // Check if USB device handler requested a scroll accumulator reset
        if crate::device_profile::SCROLL_ACCUM_RESET.load(Ordering::Relaxed) {
            crate::device_profile::SCROLL_ACCUM_RESET.store(false, Ordering::Relaxed);
            for accum in &mut scroll_accums {
                accum.reset();
            }
        }

        match select(HID_REPORT_CHANNEL.receive(), BATTERY_USB_SIGNAL.wait()).await {
            Either::First(event) => match event.report_type {
                HidReportType::Keyboard => {
                    handle_keyboard_report(&mut keyboard_writer, &event).await;
                }
                HidReportType::Mouse
                    if event.profile == crate::device_profile::DeviceProfile::MagicTrackpad =>
                {
                    // MagicTrackpad in standard (non-macOS) mode — PTP translation not yet implemented
                    debug!("MagicTrackpad report in standard mode (ignored, needs PTP)");
                }
                HidReportType::Mouse => {
                    let slot = event.slot_index as usize;
                    let accum = &mut scroll_accums[slot.min(scroll_accums.len() - 1)];
                    handle_mouse_report_standard(&mut mouse_writer, &event, accum).await;
                }
                _ => {
                    debug!("Unhandled HID report type");
                }
            },
            Either::Second(level) => {
                send_battery_level(&mut mouse_writer, level).await;
            }
        }
    }
}

// ============ MT2 HID Handler Task (macOS) ============

/// Tick interval for MT2 continuous touch report streaming (~250 Hz).
/// The real MT2 uses ~91 Hz (11ms) over USB, but we can go faster to
/// improve smoothness when the input source (BLE) has lower/variable rate.
const MT2_TICK_MS: u64 = 4;

/// USB HID handler task for macOS mode (with MT2 trackpad).
///
/// Handles keyboard and mouse reports from BLE, battery updates,
/// and MT2 touch synthesis tick timer.
#[embassy_executor::task]
async fn usb_hid_handler_task_mt2(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mt2_writer: HidWriter<'static, Driver<'static, USB>, 64>,
) {
    info!("USB HID handler task started (macOS/MT2 mode), waiting for BLE reports...");

    let mut touch_synth = crate::mt2::TouchSynthesizer::new();

    loop {
        check_reprobe();

        match select3(
            HID_REPORT_CHANNEL.receive(),
            BATTERY_USB_SIGNAL.wait(),
            embassy_time::Timer::after(embassy_time::Duration::from_millis(MT2_TICK_MS)),
        )
        .await
        {
            Either3::First(event) => match event.report_type {
                HidReportType::Keyboard => {
                    handle_keyboard_report(&mut keyboard_writer, &event).await;
                }
                HidReportType::Mouse
                    if event.profile == crate::device_profile::DeviceProfile::MagicTrackpad =>
                {
                    // Magic Trackpad passthrough: write pre-translated USB report directly
                    if event.len > 0 {
                        debug!("MT2 passthrough: writing {} bytes", event.len);
                        if let Err(e) = mt2_writer.write(&event.data[..event.len]).await {
                            warn!("MT2 passthrough write error: {:?}", e);
                        }
                    }
                }
                HidReportType::Mouse => {
                    if !crate::mt2::MT_ENABLED.load(Ordering::Relaxed) {
                        debug!("MT2: scroll event received but MT not yet enabled");
                        continue;
                    }

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
                        let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
                        let scaled = crate::usb_hid::apply_multiplier_i16(scroll_delta, scroll_m);
                        let reports = touch_synth.process_scroll(scaled);
                        for report in reports {
                            if let Err(e) = mt2_writer.write(&report).await {
                                warn!("MT2 trackpad write error: {:?}", e);
                            }
                        }
                    }
                }
                _ => {
                    debug!("Unhandled HID report type");
                }
            },
            Either3::Second(_level) => {
                debug!("MT2: battery update received (not forwarded)");
            }
            Either3::Third(_) => {
                // MT2 tick: send continuous touch reports while gesture is active
                if let Some(report) = touch_synth.tick() {
                    if let Err(e) = mt2_writer.write(&report).await {
                        warn!("MT2 trackpad tick write error: {:?}", e);
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
/// 1. Enables TIMER_IRQ_0 on Core 1 (needed for embassy_time, e.g. RPC timeouts)
/// 2. Builds the USB device with OS-appropriate interfaces and identity
/// 3. Starts the Core 1 executor with all USB tasks
/// 4. Signals Core 0 that the executor is ready via `CORE1_EXECUTOR_READY`
pub fn start_core1_usb(usb: Peri<'static, USB>, detected_os: DetectedOs) -> ! {
    // Set the detected OS atomic so the rest of the USB subsystem knows
    crate::usb_hid::DETECTED_OS.store(detected_os as u8, Ordering::Relaxed);

    // Enable TIMER_IRQ_0 on Core 1's NVIC so embassy_time works
    // (e.g. RPC timeouts). embassy_rp::init() only enables it on Core 0.
    // The timer driver uses hardware spinlocks, safe for dual-core.
    unsafe {
        embassy_rp::interrupt::InterruptExt::set_priority(
            embassy_rp::interrupt::TIMER_IRQ_0,
            embassy_rp::interrupt::Priority::P3,
        );
        embassy_rp::interrupt::InterruptExt::enable(embassy_rp::interrupt::TIMER_IRQ_0);
    }

    // Build USB device on Core 1 — this ensures USBCTRL_IRQ is enabled
    // on Core 1's NVIC (where we want USB interrupts to fire).
    let driver = Driver::new(usb, Irqs);

    // Select USB identity based on detected OS
    let is_macos = detected_os == DetectedOs::MacOs;
    let (vid, pid, manufacturer, product, device_release) = if is_macos {
        (
            crate::mt2::APPLE_VID,
            crate::mt2::MT2_PID,
            crate::mt2::MT2_MANUFACTURER,
            crate::mt2::MT2_PRODUCT,
            crate::mt2::MT2_DEVICE_RELEASE,
        )
    } else {
        (
            BT2USB_VID,
            BT2USB_PID,
            BT2USB_MANUFACTURER,
            BT2USB_PRODUCT,
            0x0100u16,
        )
    };

    info!(
        "[core1] USB identity: VID={:04X} PID={:04X} {}",
        vid, pid, product
    );

    let mut config = embassy_usb::Config::new(vid, pid);
    config.manufacturer = Some(manufacturer);
    config.product = Some(product);
    config.serial_number = Some("BT2USB0000000001");
    config.device_release = device_release;
    config.max_power = 500;
    config.max_packet_size_0 = 64;
    config.bcd_usb = embassy_usb::UsbVersion::Two;
    config.supports_remote_wakeup = true;
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    // Boot Keyboard
    let kb_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };

    // State buffers — 512 for config desc to fit up to 4 interfaces, 128 for control
    // to accommodate Feature 0xDB (76 bytes)
    static CONFIG_DESC: StaticCell<[u8; 512]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();

    static STATE_KB: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_MOUSE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_RPC: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

    static DEVICE_HANDLER: StaticCell<crate::usb_hid::UsbDeviceHandler> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 512]),
        BOS_DESC.init([0; 256]),
        MS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 128]),
    );
    builder.handler(DEVICE_HANDLER.init(crate::usb_hid::UsbDeviceHandler));

    let kb_writer = HidWriter::<_, 8>::new(
        &mut builder,
        STATE_KB.init(embassy_usb::class::hid::State::new()),
        kb_config,
    );

    // Mouse interface — only for non-macOS (macOS uses MT2 trackpad for scroll)
    let mouse_writer = if !is_macos {
        static MOUSE_HANDLER: StaticCell<crate::usb_hid::HiresMouseRequestHandler> =
            StaticCell::new();
        let mouse_config = embassy_usb::class::hid::Config {
            report_descriptor: MOUSE_HIRES_16BIT_REPORT_DESC,
            request_handler: Some(MOUSE_HANDLER.init(crate::usb_hid::HiresMouseRequestHandler)),
            poll_ms: 1,
            max_packet_size: 64,
        };
        Some(HidWriter::<_, 8>::new(
            &mut builder,
            STATE_MOUSE.init(embassy_usb::class::hid::State::new()),
            mouse_config,
        ))
    } else {
        None
    };

    // MT2 Trackpad — only for macOS (minimizes interface count for hub compatibility)
    static STATE_MT2: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static MT2_HANDLER: StaticCell<crate::mt2::Mt2TrackpadRequestHandler> = StaticCell::new();
    let mt2_writer = if is_macos {
        let mt2_config = embassy_usb::class::hid::Config {
            report_descriptor: crate::mt2::TRACKPAD_REPORT_DESC,
            request_handler: Some(MT2_HANDLER.init(crate::mt2::Mt2TrackpadRequestHandler::new())),
            poll_ms: 1,
            max_packet_size: 64,
        };
        Some(HidWriter::<_, 64>::new(
            &mut builder,
            STATE_MT2.init(embassy_usb::class::hid::State::new()),
            mt2_config,
        ))
    } else {
        None
    };

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
    info!(
        "[core1] USB HID device initialized ({})",
        if is_macos { "macOS/MT2" } else { "standard" }
    );

    let executor1 = EXECUTOR1.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(usb_task(usb_dev)).unwrap();

        // Spawn the appropriate HID handler task based on OS
        match (mt2_writer, mouse_writer) {
            (Some(mt2_w), _) => {
                // macOS: MT2 trackpad for scroll, no mouse interface
                spawner
                    .spawn(usb_hid_handler_task_mt2(kb_writer, mt2_w))
                    .unwrap();
            }
            (None, Some(mouse_w)) => {
                // Windows/Linux: standard mouse interface
                spawner
                    .spawn(usb_hid_handler_task_standard(kb_writer, mouse_w))
                    .unwrap();
            }
            (None, None) => {
                core::unreachable!("must have either MT2 or mouse interface");
            }
        }

        spawner
            .spawn(rpc::rpc_task(rpc_writer, rpc_reader))
            .unwrap();
        // Signal Core 0: our executor is set up and tasks are spawned
        CORE1_EXECUTOR_READY.store(true, Ordering::Release);
        cortex_m::asm::sev(); // Wake Core 0 from WFE
    });
}
