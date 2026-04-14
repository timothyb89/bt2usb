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
use embassy_futures::select::{select3, Either3};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_usb::class::hid::{HidReader, HidReaderWriter, HidWriter};
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
        crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_KEYBOARD, &buf, false);
        if let Err(e) = writer.write(&buf).await {
            warn!("Keyboard write error: {:?}", e);
        }
    }
}

/// Apply user-configured axis multipliers to a 16-bit mouse report.
fn apply_multipliers(report: &mut crate::usb_hid::MouseReport16) {
    let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
    let pan_m = crate::usb_hid::MULTIPLIER_PAN.load(Ordering::Relaxed);
    let x_m = crate::usb_hid::MULTIPLIER_X.load(Ordering::Relaxed);
    let y_m = crate::usb_hid::MULTIPLIER_Y.load(Ordering::Relaxed);
    report.x = crate::usb_hid::apply_multiplier_i8(report.x, x_m);
    report.y = crate::usb_hid::apply_multiplier_i8(report.y, y_m);
    report.wheel = crate::usb_hid::apply_multiplier_i16(report.wheel, scroll_m);
    report.pan = crate::usb_hid::apply_multiplier_i16(report.pan, pan_m);
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

            apply_multipliers(&mut report16);

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
            crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MOUSE, &buf, true);
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
        apply_multipliers(&mut report);
        let data = serialize_mouse_report_16bit(&report);
        let mut buf = [0u8; 8];
        buf[0] = 0x01;
        buf[1..].copy_from_slice(&data);
        crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MOUSE, &buf, true);
        if let Err(e) = writer.write(&buf).await {
            warn!("Mouse write error (16-bit): {:?}", e);
        }
    } else {
        let report = event
            .profile
            .translate_mouse_report(&event.data, event.len, scroll_accum);
        // USB descriptor always uses 16-bit wheel/pan — promote to MouseReport16.
        // When hires scroll is active, the OS expects scroll in 1/120th-notch units
        // (per the Resolution Multiplier in the descriptor). Standard mice send ±1
        // per detent, so scale by 120 to get one full notch per click.
        let hires_scale: i16 = if crate::usb_hid::HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
            120
        } else {
            1
        };
        let mut report16 = crate::usb_hid::MouseReport16 {
            buttons: report.buttons,
            x: report.x,
            y: report.y,
            wheel: (report.wheel as i16) * hires_scale,
            pan: (report.pan as i16) * hires_scale,
        };
        apply_multipliers(&mut report16);
        let data = serialize_mouse_report_16bit(&report16);
        let mut buf = [0u8; 8];
        buf[0] = 0x01;
        buf[1..].copy_from_slice(&data);
        crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MOUSE, &buf, true);
        if let Err(e) = writer.write(&buf).await {
            warn!("Mouse write error (8-bit): {:?}", e);
        }
    }
}

/// Send battery level Input report on the mouse interface (Report ID 2).
///
/// The report lives in a top-level Background Controls Application collection
/// that is a sibling of the Mouse collection on the same interface, so Input
/// reports here do not count as mouse activity for host idle-sleep purposes.
async fn send_battery_level(writer: &mut HidWriter<'static, Driver<'static, USB>, 8>, level: u8) {
    let buf = [0x02, level];
    crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MOUSE, &buf, true);
    let _ =
        embassy_time::with_timeout(embassy_time::Duration::from_secs(5), writer.write(&buf)).await;
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

/// USB HID handler task for standard mode (Windows/Linux).
///
/// Handles keyboard and mouse reports from BLE, battery level updates,
/// and PTP touchpad output for MagicTrackpad via BT Classic.
#[embassy_executor::task]
async fn usb_hid_handler_task_standard(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mouse_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut ptp_writer: HidWriter<'static, Driver<'static, USB>, 64>,
) {
    info!("USB HID handler task started (standard+PTP mode), waiting for BLE reports...");

    let mut ptp_passthrough = crate::ptp_translate::PtpPassthrough::new();

    // Per-slot scroll accumulators for multi-device support
    let mut scroll_accums = [
        crate::device_profile::ScrollAccumState::new(),
        crate::device_profile::ScrollAccumState::new(),
        crate::device_profile::ScrollAccumState::new(),
    ];

    // Send initial battery level if already known. Report ID 2 lives in its
    // own Background Controls top-level collection so this Input push should
    // not count as mouse activity for host idle-sleep tracking.
    let initial_level = crate::ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed);
    if initial_level != 0xFF {
        send_battery_level(&mut mouse_writer, initial_level).await;
    }

    // Periodic ticker for PTP passthrough reclocking. See mt2 task for
    // why this must be a Ticker rather than a Timer::after.
    let mut ptp_ticker =
        embassy_time::Ticker::every(embassy_time::Duration::from_millis(MT2_TICK_MS));

    loop {
        check_reprobe();

        // Check if USB device handler requested a scroll accumulator reset
        if crate::device_profile::SCROLL_ACCUM_RESET.load(Ordering::Relaxed) {
            crate::device_profile::SCROLL_ACCUM_RESET.store(false, Ordering::Relaxed);
            for accum in &mut scroll_accums {
                accum.reset();
            }
        }

        match select3(
            HID_REPORT_CHANNEL.receive(),
            BATTERY_USB_SIGNAL.wait(),
            ptp_ticker.next(),
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
                    // MagicTrackpad BT Classic → PTP touchpad
                    if !crate::ptp::PTP_ENABLED.load(Ordering::Relaxed) {
                        continue;
                    }
                    ptp_passthrough.receive_bt_report(&event.data[..event.len]);
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
            Either3::Second(level) => {
                send_battery_level(&mut mouse_writer, level).await;
            }
            Either3::Third(_) => {
                // PTP passthrough tick: reclocked output at steady USB rate
                if let Some(report) = ptp_passthrough.tick() {
                    if crate::ptp::PTP_ENABLED.load(Ordering::Relaxed) {
                        crate::usb_hid::record_hid_write(
                            crate::usb_hid::HID_IFACE_MT2,
                            &report,
                            true,
                        );
                        if let Err(e) = ptp_writer.write(&report).await {
                            warn!("PTP tick write error: {:?}", e);
                        }
                    }
                }
            }
        }
    }
}

/// Extract mouse fields from a BLE report using the standard byte-offset heuristic.
/// Returns (buttons, x, y, scroll_delta) for reports that follow the common layout:
///   [buttons(1), X(i8), Y(i8), wheel(i8), pan(i8)]
fn extract_mouse_heuristic(event: &HidReportEvent) -> (u8, i8, i8, i16) {
    if event.len < 3 {
        return (0, 0, 0, 0);
    }
    let buttons = event.data[0] & 0x1F;
    let x = event.data[1] as i8;
    let y = event.data[2] as i8;
    let wheel = if event.len >= 4 {
        event.data[3] as i8 as i16
    } else {
        0
    };
    (buttons, x, y, wheel)
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
    let mut mt2_passthrough = crate::mt2_translate::Mt2Passthrough::new();

    // Periodic ticker for MT2 synthesis. Unlike Timer::after (which resets
    // on every loop iteration), Ticker fires at fixed intervals regardless
    // of how frequently BLE events arrive. This is critical: under streaming
    // mouse movement, a reset-on-iteration timer is starved out by events,
    // which stops tick() from draining the scroll buffer and sending
    // stationary touch reports — causing macOS to fling at gesture end.
    let mut mt2_ticker =
        embassy_time::Ticker::every(embassy_time::Duration::from_millis(MT2_TICK_MS));

    loop {
        check_reprobe();

        match select3(
            HID_REPORT_CHANNEL.receive(),
            BATTERY_USB_SIGNAL.wait(),
            mt2_ticker.next(),
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
                    // MT2 reclocked passthrough: store raw BT report for steady-rate output
                    if !crate::mt2::MT_ENABLED.load(Ordering::Relaxed) {
                        debug!("MT2: report received but MT not yet enabled");
                        continue;
                    }
                    mt2_passthrough.receive_bt_report(&event.data[..event.len]);
                }
                HidReportType::Mouse => {
                    // Non-MT2 BLE devices: split into cursor + scroll paths.
                    // Buttons/X/Y → Report ID 0x02 (MT2 mouse collection)
                    // Wheel → TouchSynthesizer → Report ID 0x44 (MT2 touch)

                    // Extract mouse fields via layout-aware path or heuristic.
                    // For scroll, also determine whether the wheel field is low-res
                    // (≤8 bits, ±1 per detent) or high-res (>8 bits, native velocity).
                    // Low-res values must be scaled up for the TouchSynthesizer, which
                    // was tuned for high-res 16-bit deltas from devices like the Dial.
                    let (buttons, x, y, scroll_delta, wheel_is_lowres) = if event.profile
                        == crate::device_profile::DeviceProfile::Generic
                    {
                        if let Some(layout) =
                            crate::ble_hid::find_slot_layout(event.slot_index as usize, event.len)
                        {
                            let d = &event.data[..event.len];
                            let lowres = layout.wheel.is_some_and(|l| l.bit_size <= 8);
                            (
                                layout
                                    .buttons
                                    .map(|loc| crate::hid_report_map::extract_field(d, &loc) as u8)
                                    .unwrap_or(0)
                                    & 0x1F,
                                layout
                                    .x
                                    .map(|loc| {
                                        crate::hid_report_map::extract_field(d, &loc)
                                            .clamp(-127, 127)
                                            as i8
                                    })
                                    .unwrap_or(0),
                                layout
                                    .y
                                    .map(|loc| {
                                        crate::hid_report_map::extract_field(d, &loc)
                                            .clamp(-127, 127)
                                            as i8
                                    })
                                    .unwrap_or(0),
                                layout
                                    .wheel
                                    .map(|loc| {
                                        crate::hid_report_map::extract_field(d, &loc)
                                            .clamp(i16::MIN as i32, i16::MAX as i32)
                                            as i16
                                    })
                                    .unwrap_or(0),
                                lowres,
                            )
                        } else {
                            // Heuristic fallback: standard mouse layout (always low-res)
                            let (b, x, y, w) = extract_mouse_heuristic(&event);
                            (b, x, y, w, true)
                        }
                    } else if event.profile.uses_16bit_reports() {
                        // Named 16-bit profiles (FullScrollDial16Bit): scroll in bytes 0-1
                        let scroll = if event.len >= 2 {
                            i16::from_le_bytes([event.data[0], event.data[1]])
                        } else {
                            0
                        };
                        (0, 0, 0, scroll, false)
                    } else {
                        // Named 8-bit profiles: use standard heuristic
                        let (b, x, y, w) = extract_mouse_heuristic(&event);
                        (b, x, y, w, true)
                    };

                    // Forward buttons/cursor via MT2 mouse collection (Report ID 0x02)
                    // This works regardless of MT_ENABLED — the mouse collection is
                    // always present in the MT2 descriptor.
                    if buttons != 0 || x != 0 || y != 0 {
                        let mut buf = [0u8; 8];
                        buf[0] = 0x02; // Report ID
                        buf[1] = buttons;
                        buf[2] = x as u8;
                        buf[3] = y as u8;
                        // bytes 4-7: padding (matches MT2 descriptor)
                        crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MT2, &buf, true);
                        if let Err(e) = mt2_writer.write(&buf).await {
                            warn!("MT2 mouse write error: {:?}", e);
                        }
                    }

                    // Route scroll through TouchSynthesizer (requires MT activation)
                    if scroll_delta != 0 && crate::mt2::MT_ENABLED.load(Ordering::Relaxed) {
                        // Low-res wheel (≤8-bit, standard mice): scale ±1 detent up
                        // to high-res units and use the lowres synthesizer path.
                        // High-res devices (Dial) send native 16-bit velocity and
                        // use the EMA-based drain rate in process_scroll.
                        let scroll_hires = if wheel_is_lowres {
                            scroll_delta.saturating_mul(120)
                        } else {
                            scroll_delta
                        };
                        let scroll_m = crate::usb_hid::MULTIPLIER_SCROLL.load(Ordering::Relaxed);
                        let scaled = crate::usb_hid::apply_multiplier_i16(scroll_hires, scroll_m);
                        let reports = if wheel_is_lowres {
                            touch_synth.process_scroll_lowres(scaled)
                        } else {
                            touch_synth.process_scroll(scaled)
                        };
                        for report in reports {
                            crate::usb_hid::record_hid_write(
                                crate::usb_hid::HID_IFACE_MT2,
                                &report,
                                true,
                            );
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
                // MT2 descriptor doesn't currently have a battery collection;
                // swallow the signal so it doesn't stall the standard task.
                debug!("MT2: battery update received (not forwarded)");
            }
            Either3::Third(_) => {
                // MT2 passthrough tick: reclocked output at steady USB rate
                if let Some((report, len)) = mt2_passthrough.tick() {
                    crate::usb_hid::record_hid_write(
                        crate::usb_hid::HID_IFACE_MT2,
                        &report[..len],
                        true,
                    );
                    if let Err(e) = mt2_writer.write(&report[..len]).await {
                        warn!("MT2 passthrough tick write error: {:?}", e);
                    }
                }
                // TouchSynthesizer tick: Dial scroll synthesis
                if let Some(report) = touch_synth.tick() {
                    crate::usb_hid::record_hid_write(crate::usb_hid::HID_IFACE_MT2, &report, true);
                    if let Err(e) = mt2_writer.write(&report).await {
                        warn!("MT2 synth tick write error: {:?}", e);
                    }
                }
            }
        }
    }
}

// ============ MT2 Actuator Task ============

/// Reads actuator OUTPUT reports (Report ID 0x53) from macOS and auto-ACKs them.
///
/// macOS sends actuator commands when it detects sufficient force pressure in
/// touch data. The MT2 over BT Classic handles haptics autonomously, so we just
/// need to accept the commands — macOS then reads the button bit from subsequent
/// touch reports to confirm the click.
#[embassy_executor::task]
async fn mt2_actuator_task(mut reader: HidReader<'static, Driver<'static, USB>, 64>) {
    info!("[actuator] MT2 actuator reader task started");
    let mut buf = [0u8; 64];
    loop {
        match reader.read(&mut buf).await {
            Ok(n) if n > 0 => {
                info!(
                    "[actuator] Received {} bytes: report_id=0x{:02X}",
                    n, buf[0]
                );
            }
            Ok(_) => {}
            Err(e) => {
                warn!("[actuator] Read error: {:?}", e);
                // USB disconnected/reset — wait briefly and retry
                embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
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
    // Remote wakeup is advertised only if the driver can actually issue resume
    // signaling. embassy-rp 0.9 returns Unsupported from Bus::remote_wakeup(),
    // so we'd be claiming a capability we can't deliver — and on Windows that
    // keeps some hosts from entering selective suspend / system sleep. Leave
    // this off until the RP2040 USB driver gains real remote-wakeup support.
    config.supports_remote_wakeup = false;
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

    // State buffers — 768 for config desc (PTP descriptor ~565 bytes + interface overhead),
    // 320 for control to accommodate PTP certification blob (257 bytes)
    static CONFIG_DESC: StaticCell<[u8; 768]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 320]> = StaticCell::new();

    static STATE_KB: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_MOUSE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_RPC: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

    static DEVICE_HANDLER: StaticCell<crate::usb_hid::UsbDeviceHandler> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 768]),
        BOS_DESC.init([0; 256]),
        MS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 320]),
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

    // PTP Touchpad — only for non-macOS (Windows/Linux PTP gestures from MT2)
    static STATE_PTP: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static PTP_HANDLER: StaticCell<crate::ptp::PtpRequestHandler> = StaticCell::new();
    let ptp_writer = if !is_macos {
        let ptp_config = embassy_usb::class::hid::Config {
            report_descriptor: crate::ptp::PTP_REPORT_DESC,
            request_handler: Some(PTP_HANDLER.init(crate::ptp::PtpRequestHandler::new())),
            poll_ms: 1,
            max_packet_size: 64,
        };
        Some(HidWriter::<_, 64>::new(
            &mut builder,
            STATE_PTP.init(embassy_usb::class::hid::State::new()),
            ptp_config,
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

    // MT2 Actuator interface — only for macOS.
    // Real MT2 Interface 2: accepts actuator OUTPUT reports (Report ID 0x53)
    // from macOS for haptic feedback. Without this, macOS ignores the click bit.
    static STATE_ACTUATOR: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static ACTUATOR_HANDLER: StaticCell<crate::mt2::Mt2ActuatorRequestHandler> = StaticCell::new();
    let actuator_reader = if is_macos {
        let actuator_config = embassy_usb::class::hid::Config {
            report_descriptor: crate::mt2::ACTUATOR_REPORT_DESC,
            request_handler: Some(
                ACTUATOR_HANDLER.init(crate::mt2::Mt2ActuatorRequestHandler::new()),
            ),
            poll_ms: 1,
            max_packet_size: 64,
        };
        let actuator_hid = HidReaderWriter::<_, 64, 64>::new(
            &mut builder,
            STATE_ACTUATOR.init(embassy_usb::class::hid::State::new()),
            actuator_config,
        );
        let (actuator_r, _actuator_w) = actuator_hid.split();
        Some(actuator_r)
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
        spawner.spawn(crate::watchdog::core1_tick_task()).unwrap();

        // Spawn the appropriate HID handler task based on OS
        match (mt2_writer, mouse_writer, ptp_writer) {
            (Some(mt2_w), _, _) => {
                // macOS: MT2 trackpad for scroll, no mouse interface
                spawner
                    .spawn(usb_hid_handler_task_mt2(kb_writer, mt2_w))
                    .unwrap();
            }
            (None, Some(mouse_w), Some(ptp_w)) => {
                // Windows/Linux: standard mouse + PTP touchpad
                spawner
                    .spawn(usb_hid_handler_task_standard(kb_writer, mouse_w, ptp_w))
                    .unwrap();
            }
            _ => {
                core::unreachable!("must have either MT2 or mouse+PTP interfaces");
            }
        }

        // Spawn actuator reader task (macOS only)
        if let Some(act_reader) = actuator_reader {
            spawner.spawn(mt2_actuator_task(act_reader)).unwrap();
        }

        spawner
            .spawn(rpc::rpc_task(rpc_writer, rpc_reader))
            .unwrap();
        // Signal Core 0: our executor is set up and tasks are spawned
        CORE1_EXECUTOR_READY.store(true, Ordering::Release);
        cortex_m::asm::sev(); // Wake Core 0 from WFE
    });
}
