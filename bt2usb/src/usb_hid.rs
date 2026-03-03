//! USB HID device implementation for bt2usb
//!
//! This module provides USB HID functionality:
//! - Keyboard HID device with standard 6-key rollover report
//! - Mouse HID device with buttons, X/Y movement, and scroll wheel
//! - Composite device support for simultaneous keyboard + mouse

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use portable_atomic::AtomicU64;
use defmt::*;
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::types::StringIndex;
use embassy_usb::Handler;
use usbd_hid::descriptor::MouseReport;

/// Whether the host has enabled high-resolution scroll mode via the
/// Resolution Multiplier Feature report.
pub static HIRES_SCROLL_ENABLED: AtomicBool = AtomicBool::new(false);

// --- OS Detection ---
// Detected via USB enumeration behavior:
// - Windows: requests String descriptor 0xEE (MS OS String Descriptor)
// - Linux: enables hires scroll via SET_REPORT without requesting 0xEE
// - macOS: neither (timeout-based detection)
pub const OS_UNKNOWN: u8 = 0;
pub const OS_WINDOWS: u8 = 1;
pub const OS_LINUX: u8 = 2;
pub const OS_MACOS: u8 = 3;

/// Detected host operating system.
pub static DETECTED_OS: AtomicU8 = AtomicU8::new(OS_UNKNOWN);

/// Tick count when USB was last configured (for OS detection timeout).
pub static CONFIGURED_AT_TICKS: AtomicU64 = AtomicU64::new(0);

/// Axis multipliers as percentages (100 = 1.0x, 200 = 2.0x, 50 = 0.5x).
/// Applied after profile-specific translation, before USB serialization.
pub static MULTIPLIER_SCROLL: AtomicU32 = AtomicU32::new(100);
pub static MULTIPLIER_PAN: AtomicU32 = AtomicU32::new(100);
pub static MULTIPLIER_X: AtomicU32 = AtomicU32::new(100);
pub static MULTIPLIER_Y: AtomicU32 = AtomicU32::new(100);

/// Config keys (shared with protocol/RPC layer)
pub const CONFIG_KEY_SCROLL_MULT: u8 = 0;
pub const CONFIG_KEY_PAN_MULT: u8 = 1;
pub const CONFIG_KEY_X_MULT: u8 = 2;
pub const CONFIG_KEY_Y_MULT: u8 = 3;
pub const CONFIG_KEY_SCROLL_THRESHOLD: u8 = 4;
pub const CONFIG_KEY_MAX_DETENTS: u8 = 5;

/// Scroll accumulator threshold (raw units before emitting).
/// Default 120 = one standard detent in HID Resolution Multiplier spec.
pub static SCROLL_THRESHOLD: AtomicU32 = AtomicU32::new(120);

/// Maximum detents emitted per scroll event in standard mode.
/// Caps fast-scroll bursts to stay in macOS's linear acceleration region.
pub static MAX_DETENTS_PER_EMIT: AtomicU32 = AtomicU32::new(3);

/// Apply a percentage multiplier to an i8 value, clamping to i8 range.
pub fn apply_multiplier_i8(value: i8, multiplier_pct: u32) -> i8 {
    if multiplier_pct == 100 {
        return value;
    }
    let scaled = (value as i32) * (multiplier_pct as i32) / 100;
    scaled.clamp(-127, 127) as i8
}

/// Apply a percentage multiplier to an i16 value, clamping to i16 range.
pub fn apply_multiplier_i16(value: i16, multiplier_pct: u32) -> i16 {
    if multiplier_pct == 100 {
        return value;
    }
    let scaled = (value as i32) * (multiplier_pct as i32) / 100;
    scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Mouse report with 16-bit wheel and pan for experimental mode
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct MouseReport16 {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
    pub wheel: i16,
    pub pan: i16,
}

/// Keyboard HID report - re-export from usbd_hid for compatibility
pub use usbd_hid::descriptor::KeyboardReport as KeyboardHidReport;

/// Mouse HID report descriptor with Resolution Multiplier for high-res scrolling.
///
/// Input report: 5 bytes [buttons, x, y, wheel, pan] — same layout as standard MouseReport.
/// Feature report: 1 byte [vert_mult:2, horiz_mult:2, padding:4].
///
/// When the OS enables the multiplier, each wheel/pan unit = 1/120th of a detent,
/// enabling smooth per-pixel scrolling.
///
/// Kept for reference; the 16-bit variant (`MOUSE_HIRES_16BIT_REPORT_DESC`) is
/// always used in practice.
#[allow(dead_code)]
pub const MOUSE_HIRES_REPORT_DESC: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    // -- Buttons (5 bits + 3 padding = 1 byte) --
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (1)
    0x29, 0x05, //     Usage Maximum (5)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x05, //     Report Count (5)
    0x81, 0x02, //     Input (Data, Variable, Absolute)
    0x75, 0x03, //     Report Size (3)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x01, //     Input (Constant) - padding
    // -- X, Y movement (2 bytes) --
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    // -- Vertical wheel with Resolution Multiplier --
    0xA1, 0x02, //     Collection (Logical)
    0x09, 0x48, //       Usage (Resolution Multiplier)
    0x15, 0x00, //       Logical Minimum (0)
    0x25, 0x01, //       Logical Maximum (1)
    0x35, 0x01, //       Physical Minimum (1)
    0x45, 0x78, //       Physical Maximum (120)
    0x75, 0x02, //       Report Size (2)
    0x95, 0x01, //       Report Count (1)
    0xB1, 0x02, //       Feature (Data, Variable, Absolute)
    0x09, 0x38, //       Usage (Wheel)
    0x15, 0x81, //       Logical Minimum (-127)
    0x25, 0x7F, //       Logical Maximum (127)
    0x35, 0x00, //       Physical Minimum (0) - reset
    0x45, 0x00, //       Physical Maximum (0) - reset
    0x75, 0x08, //       Report Size (8)
    0x95, 0x01, //       Report Count (1)
    0x81, 0x06, //       Input (Data, Variable, Relative)
    0xC0, //     End Collection
    // -- Horizontal scroll (pan) with Resolution Multiplier --
    0x05, 0x0C, //     Usage Page (Consumer)
    0xA1, 0x02, //     Collection (Logical)
    0x05, 0x01, //       Usage Page (Generic Desktop)
    0x09, 0x48, //       Usage (Resolution Multiplier)
    0x15, 0x00, //       Logical Minimum (0)
    0x25, 0x01, //       Logical Maximum (1)
    0x35, 0x01, //       Physical Minimum (1)
    0x45, 0x78, //       Physical Maximum (120)
    0x75, 0x02, //       Report Size (2)
    0x95, 0x01, //       Report Count (1)
    0xB1, 0x02, //       Feature (Data, Variable, Absolute)
    0x05, 0x0C, //       Usage Page (Consumer)
    0x0A, 0x38, 0x02, // Usage (AC Pan)
    0x15, 0x81, //       Logical Minimum (-127)
    0x25, 0x7F, //       Logical Maximum (127)
    0x35, 0x00, //       Physical Minimum (0)
    0x45, 0x00, //       Physical Maximum (0)
    0x75, 0x08, //       Report Size (8)
    0x95, 0x01, //       Report Count (1)
    0x81, 0x06, //       Input (Data, Variable, Relative)
    0xC0, //     End Collection
    // -- Feature report padding (4 bits to byte-align) --
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0xB1, 0x01, //     Feature (Constant)
    0xC0, //   End Collection (Physical)
    0xC0, // End Collection (Application)
];

/// Mouse HID report descriptor with 16-bit wheel/pan and embedded battery level.
///
/// Report ID 1 — Mouse:
///   Input:   7 bytes [buttons:1, x:1, y:1, wheel:2, pan:2]
///   Feature: 1 byte  [vert_mult:2, horiz_mult:2, padding:4]
///
/// Report ID 2 — Battery Level (Battery System page 0x85, usage 0x65 AbsoluteStateOfCharge):
///   Input:   1 byte  [level:8]  — sent proactively when BLE battery changes;
///                                 Linux hid-battery reads this and updates
///                                 /sys/class/power_supply/hid-*/capacity
///   Feature: 1 byte  [level:8]  — returned on GET_REPORT(Feature, 2);
///                                 readable on Windows via HidD_GetFeature
///
/// Usage 0x65 (AbsoluteStateOfCharge) is required — kernel only calls
/// hidinput_setup_battery() for this usage. Usage 0x44 (Charging) is also
/// recognized but only sets EV_PWR without creating a power_supply.
/// Embedding battery in the mouse interface is required for Linux power_supply
/// integration: hidinput_setup_battery() is only called for input interfaces.
pub const MOUSE_HIRES_16BIT_REPORT_DESC: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    // ---- Report ID 1: Mouse movement + Resolution Multiplier ----
    0x85, 0x01, //   Report ID (1)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    // -- Buttons (5 bits + 3 padding = 1 byte) --
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (1)
    0x29, 0x05, //     Usage Maximum (5)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x05, //     Report Count (5)
    0x81, 0x02, //     Input (Data, Variable, Absolute)
    0x75, 0x03, //     Report Size (3)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x01, //     Input (Constant) - padding
    // -- X, Y movement (2 bytes, 8-bit) --
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    // -- Vertical wheel (16-bit) with Resolution Multiplier --
    0xA1, 0x02, //     Collection (Logical)
    0x09, 0x48, //       Usage (Resolution Multiplier)
    0x15, 0x00, //       Logical Minimum (0)
    0x25, 0x01, //       Logical Maximum (1)
    0x35, 0x01, //       Physical Minimum (1)
    0x45, 0x78, //       Physical Maximum (120)
    0x75, 0x02, //       Report Size (2)
    0x95, 0x01, //       Report Count (1)
    0xB1, 0x02, //       Feature (Data, Variable, Absolute) [ID=1]
    0x09, 0x38, //       Usage (Wheel)
    0x16, 0x00, 0x80, // Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, // Logical Maximum (32767)
    0x35, 0x00, //       Physical Minimum (0) - reset
    0x45, 0x00, //       Physical Maximum (0) - reset
    0x75, 0x10, //       Report Size (16) ← 16-bit field
    0x95, 0x01, //       Report Count (1)
    0x81, 0x06, //       Input (Data, Variable, Relative) [ID=1]
    0xC0, //     End Collection
    // -- Horizontal scroll/pan (16-bit) with Resolution Multiplier --
    0x05, 0x0C, //     Usage Page (Consumer)
    0xA1, 0x02, //     Collection (Logical)
    0x05, 0x01, //       Usage Page (Generic Desktop)
    0x09, 0x48, //       Usage (Resolution Multiplier)
    0x15, 0x00, //       Logical Minimum (0)
    0x25, 0x01, //       Logical Maximum (1)
    0x35, 0x01, //       Physical Minimum (1)
    0x45, 0x78, //       Physical Maximum (120)
    0x75, 0x02, //       Report Size (2)
    0x95, 0x01, //       Report Count (1)
    0xB1, 0x02, //       Feature (Data, Variable, Absolute) [ID=1]
    0x05, 0x0C, //       Usage Page (Consumer)
    0x0A, 0x38, 0x02, // Usage (AC Pan)
    0x16, 0x00, 0x80, // Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, // Logical Maximum (32767)
    0x35, 0x00, //       Physical Minimum (0)
    0x45, 0x00, //       Physical Maximum (0)
    0x75, 0x10, //       Report Size (16) ← 16-bit field
    0x95, 0x01, //       Report Count (1)
    0x81, 0x06, //       Input (Data, Variable, Relative) [ID=1]
    0xC0, //     End Collection
    // -- Feature padding (4 bits to byte-align the 4-bit mult pair) [ID=1] --
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0xB1, 0x01, //     Feature (Constant)
    0xC0, //   End Collection (Physical)
    // ---- Report ID 2: Battery Level (Battery System page) ----
    0x85, 0x02, //   Report ID (2)
    0x05, 0x85, //   Usage Page (Battery System)
    0x09, 0x65, //   Usage (Absolute State of Charge) — triggers hidinput_setup_battery()
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x64, //   Logical Maximum (100)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x02, //   Input (Data, Variable, Absolute) — proactive push
    0x09, 0x65, //   Usage (Absolute State of Charge)
    0xB1, 0x02, //   Feature (Data, Variable, Absolute) — GET_REPORT query
    0xC0, // End Collection (Application)
];

/// Mouse request handler with Resolution Multiplier Feature report support.
///
/// When the OS sends SET_REPORT(Feature) to enable high-res scroll,
/// this handler sets HIRES_SCROLL_ENABLED so the translation layer
/// can scale wheel values appropriately.
pub struct HiresMouseRequestHandler;

impl RequestHandler for HiresMouseRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        if buf.is_empty() {
            return None;
        }
        match id {
            ReportId::Feature(1) => {
                // Resolution Multiplier state: bits 0-1 vert, bits 2-3 horiz
                let enabled = HIRES_SCROLL_ENABLED.load(Ordering::Relaxed);
                buf[0] = if enabled { 0x05 } else { 0x00 };
                Some(1)
            }
            ReportId::Feature(2) => {
                // Battery level (0-100), or 0 if unknown
                let level = crate::ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed);
                buf[0] = if level == 0xFF { 0 } else { level };
                Some(1)
            }
            _ => {
                debug!("Mouse get_report: {:?}", id);
                None
            }
        }
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        // Only Report ID 1 (scroll multiplier) accepts SET_REPORT
        if let ReportId::Feature(1) = id {
            if !data.is_empty() {
                let vert_mult = data[0] & 0x03;
                let enabled = vert_mult > 0;
                HIRES_SCROLL_ENABLED.store(enabled, Ordering::Relaxed);
                info!(
                    "High-res scroll: {}",
                    if enabled { "enabled" } else { "disabled" }
                );
                // OS detection: hires enabled without prior Windows detection = Linux
                if enabled && DETECTED_OS.load(Ordering::Relaxed) != OS_WINDOWS {
                    info!("OS detected: Linux (hires enabled, no String 0xEE)");
                    DETECTED_OS.store(OS_LINUX, Ordering::Relaxed);
                }
            }
        }
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}

    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }
}

/// Vendor-specific HID report descriptor for the RPC interface.
///
/// Uses Vendor Defined usage page (0xFF00) with 64-byte input and output reports.
/// No report IDs — single report per interface, so the full 64 bytes are payload.
/// This replaces CDC ACM for RPC communication, eliminating the extra endpoints
/// and host-initiated control transfers that interfere with BLE pairing.
pub const VENDOR_RPC_REPORT_DESC: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01, // Usage (Vendor Usage 1)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Vendor Usage 1)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x40, //   Report Count (64)
    0x81, 0x02, //   Input (Data, Variable, Absolute)
    0x09, 0x01, //   Usage (Vendor Usage 1)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x40, //   Report Count (64)
    0x91, 0x02, //   Output (Data, Variable, Absolute)
    0xC0, // End Collection
];

/// Device handler for USB state changes
pub struct UsbDeviceHandler;

impl Handler for UsbDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        if enabled {
            info!("USB device enabled");
        } else {
            info!("USB device disabled");
        }
    }

    fn reset(&mut self) {
        // Reset high-res scroll state so the OS must re-enable it after
        // re-enumeration. Without this, a sleep/wake or USB switch causes a
        // mismatch: firmware thinks hires is on (passthrough) but the OS
        // treats units as standard (1 unit = 1 detent) → 120× too fast.
        HIRES_SCROLL_ENABLED.store(false, Ordering::Relaxed);
        DETECTED_OS.store(OS_UNKNOWN, Ordering::Relaxed);
        crate::device_profile::reset_scroll_accumulator();
        info!("USB bus reset, high-res scroll and OS detection reset");
    }

    fn addressed(&mut self, addr: u8) {
        debug!("USB address set to {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        // Reset high-res scroll on any re-configuration. USB switches may
        // re-enumerate without a full bus reset, so reset() alone isn't enough.
        HIRES_SCROLL_ENABLED.store(false, Ordering::Relaxed);
        DETECTED_OS.store(OS_UNKNOWN, Ordering::Relaxed);
        CONFIGURED_AT_TICKS.store(
            embassy_time::Instant::now().as_ticks(),
            Ordering::Relaxed,
        );
        crate::device_profile::reset_scroll_accumulator();
        if configured {
            info!("USB device configured, high-res scroll and OS detection reset");
        } else {
            info!("USB device unconfigured, high-res scroll and OS detection reset");
        }
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            debug!("USB suspended");
        } else {
            debug!("USB resumed");
        }
    }

    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        // Windows requests String descriptor 0xEE (MS OS String Descriptor)
        // during enumeration. macOS and Linux never do.
        if index == StringIndex(0xEE) {
            info!("OS detected: Windows (String 0xEE requested)");
            DETECTED_OS.store(OS_WINDOWS, Ordering::Relaxed);
        }
        None
    }
}

/// Serialize a KeyboardReport to bytes for USB transmission
pub fn serialize_keyboard_report(report: &KeyboardHidReport) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = report.modifier;
    buf[1] = report.reserved;
    buf[2..8].copy_from_slice(&report.keycodes);
    buf
}

/// Serialize a MouseReport to bytes for USB transmission
/// MouseReport layout: buttons (1), x (1), y (1), wheel (1), pan (1) = 5 bytes
pub fn serialize_mouse_report(report: &MouseReport) -> [u8; 5] {
    let mut buf = [0u8; 5];
    buf[0] = report.buttons;
    buf[1] = report.x as u8;
    buf[2] = report.y as u8;
    buf[3] = report.wheel as u8;
    buf[4] = report.pan as u8;
    buf
}

/// Serialize a MouseReport16 to bytes for USB transmission
/// MouseReport16 layout: buttons (1), x (1), y (1), wheel (2), pan (2) = 7 bytes
pub fn serialize_mouse_report_16bit(report: &MouseReport16) -> [u8; 7] {
    let mut buf = [0u8; 7];
    buf[0] = report.buttons;
    buf[1] = report.x as u8;
    buf[2] = report.y as u8;
    buf[3..5].copy_from_slice(&report.wheel.to_le_bytes());
    buf[5..7].copy_from_slice(&report.pan.to_le_bytes());

    debug!("USB HID 16-bit: wheel={} -> bytes=[{:02x},{:02x}], full=[{:02x},{:02x},{:02x},{:02x},{:02x},{:02x},{:02x}]",
        report.wheel, buf[3], buf[4], buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

    buf
}
