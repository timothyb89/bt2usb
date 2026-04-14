//! USB HID device implementation for bt2usb
//!
//! This module provides USB HID functionality:
//! - Keyboard HID device with standard 6-key rollover report
//! - Mouse HID device with buttons, X/Y movement, and scroll wheel
//! - Composite device support for simultaneous keyboard + mouse

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use defmt::*;
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::types::StringIndex;
use embassy_usb::Handler;
use portable_atomic::AtomicU64;
use usbd_hid::descriptor::MouseReport;

/// Whether the host has enabled high-resolution scroll mode via the
/// Resolution Multiplier Feature report.
pub static HIRES_SCROLL_ENABLED: AtomicBool = AtomicBool::new(false);

// --- OS Detection ---
// Detected via USB enumeration behavior:
// - Windows: requests String descriptor 0xEE (MS OS String Descriptor)
// - Linux: enables hires scroll via SET_REPORT without requesting 0xEE
// - macOS: neither (timeout-based detection)
#[allow(dead_code)]
pub const OS_UNKNOWN: u8 = 0;
#[allow(dead_code)]
pub const OS_WINDOWS: u8 = 1;
#[allow(dead_code)]
pub const OS_LINUX: u8 = 2;
#[allow(dead_code)]
pub const OS_MACOS: u8 = 3;

/// Detected host operating system.
pub static DETECTED_OS: AtomicU8 = AtomicU8::new(OS_UNKNOWN);

/// Tick count when USB was last configured (for OS detection timeout).
pub static CONFIGURED_AT_TICKS: AtomicU64 = AtomicU64::new(0);

/// Set true when a USB bus reset occurs >2s after Phase 1 configuration.
/// Checked by the HID handler task to trigger a reprobe (watchdog reset → Phase 0).
pub static REPROBE_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Minimum uptime (ticks) before a bus reset triggers reprobe.
/// 2 seconds at 1MHz tick rate.
const REPROBE_THRESHOLD_TICKS: u64 = 2_000_000;

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
pub const CONFIG_KEY_SCROLL_SMOOTHING: u8 = 6;

/// Scroll smoothing mode for low-res mice on macOS (0=linear, 1=smooth).
/// Linear: fixed-rate drain, bypasses macOS acceleration.
/// Smooth: ease-in/out velocity envelope, enables natural macOS scrolling.
pub static SCROLL_SMOOTHING: AtomicU32 = AtomicU32::new(0);

/// Scroll accumulator threshold (raw units before emitting).
/// Default 120 = one standard detent in HID Resolution Multiplier spec.
pub static SCROLL_THRESHOLD: AtomicU32 = AtomicU32::new(120);

/// Maximum detents emitted per scroll event in standard mode.
/// Caps fast-scroll bursts to stay in macOS's linear acceleration region.
pub static MAX_DETENTS_PER_EMIT: AtomicU32 = AtomicU32::new(3);

// ============ HID activity tracking ============
//
// Per-interface counters for debugging "device prevents host sleep" reports.
// Windows resets its idle-sleep timer on any HID input report on a mouse or
// keyboard interface, so stray writes (battery updates routed to the mouse
// interface, idle/release notifications from the BLE peripheral, etc.) can
// silently keep the host awake. These counters let us see at a glance which
// interface is writing and how recently.
//
// `last_write_ticks` stores the embassy `Instant::now().as_ticks()` value at
// the most recent write (1 tick = 1 µs on RP2040). 0 means "never written."
// `writes` is a monotonic write count. Report ID of the most recent write is
// also tracked so battery reports (ID 0x02 on the mouse interface) can be
// distinguished from standard mouse reports (ID 0x01).

pub const HID_IFACE_KEYBOARD: usize = 0;
pub const HID_IFACE_MOUSE: usize = 1;
pub const HID_IFACE_MT2: usize = 2;
pub const HID_IFACE_RPC: usize = 3;
pub const HID_IFACE_COUNT: usize = 4;

pub static HID_ACTIVITY_LAST_TICKS: [AtomicU64; HID_IFACE_COUNT] = [
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
];

// Use portable-atomic for AtomicU32 because fetch_add isn't available on
// core::sync::atomic::AtomicU32 for ARMv6-M (RP2040 lacks LDREX/STREX).
pub static HID_ACTIVITY_WRITES: [portable_atomic::AtomicU32; HID_IFACE_COUNT] = [
    portable_atomic::AtomicU32::new(0),
    portable_atomic::AtomicU32::new(0),
    portable_atomic::AtomicU32::new(0),
    portable_atomic::AtomicU32::new(0),
];

pub static HID_ACTIVITY_LAST_REPORT_ID: [AtomicU8; HID_IFACE_COUNT] = [
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
];

fn iface_name(iface: usize) -> &'static str {
    match iface {
        HID_IFACE_KEYBOARD => "kbd",
        HID_IFACE_MOUSE => "mouse",
        HID_IFACE_MT2 => "mt2",
        HID_IFACE_RPC => "rpc",
        _ => "?",
    }
}

/// Record a HID write for activity tracking and debug logging.
///
/// Call before `HidWriter::write` at every write site. Payload should be the
/// bytes passed to the writer (Report ID in byte 0 for most of our paths;
/// pass `report_id_in_byte0 = false` for keyboard/RPC interfaces that don't
/// prepend an explicit report ID).
pub fn record_hid_write(iface: usize, payload: &[u8], report_id_in_byte0: bool) {
    if iface >= HID_IFACE_COUNT || payload.is_empty() {
        return;
    }
    let now = embassy_time::Instant::now().as_ticks();
    HID_ACTIVITY_LAST_TICKS[iface].store(now, Ordering::Relaxed);
    HID_ACTIVITY_WRITES[iface].fetch_add(1, Ordering::Relaxed);

    let (report_id, body) = if report_id_in_byte0 {
        (payload[0], &payload[1..])
    } else {
        (0u8, payload)
    };
    HID_ACTIVITY_LAST_REPORT_ID[iface].store(report_id, Ordering::Relaxed);

    // An "idle" write is one whose body (after Report ID) is entirely zero.
    // These still reset the Windows idle-sleep timer on mouse/keyboard
    // interfaces and are the most common cause of unwanted wake behavior.
    let is_idle = body.iter().all(|b| *b == 0);
    debug!(
        "hid-tx {}: id=0x{:02X} len={} idle={}",
        iface_name(iface),
        report_id,
        payload.len(),
        is_idle,
    );
}

pub use bt2usb_core::mouse::{apply_multiplier_i16, apply_multiplier_i8, MouseReport16};

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
        crate::device_profile::SCROLL_ACCUM_RESET.store(true, Ordering::Relaxed);

        // Switch detection: if the device has been configured for >2s,
        // this bus reset is likely from a USB switch changeover or physical
        // reconnection to a different host. Request reprobe to re-fingerprint.
        // Also reset protocol state (hires scroll, MT) since the new host
        // needs to re-negotiate. For quick re-enumerations (<2s, e.g. VM
        // hotplug), keep protocol state — the host expects the device to
        // remain in the same state and may not re-send Feature reports.
        let configured_at = CONFIGURED_AT_TICKS.load(Ordering::Relaxed);
        if configured_at > 0 {
            let elapsed = embassy_time::Instant::now().as_ticks() - configured_at;
            if elapsed > REPROBE_THRESHOLD_TICKS {
                info!("USB bus reset >2s after config -> reprobe requested");
                REPROBE_REQUESTED.store(true, Ordering::Relaxed);
                HIRES_SCROLL_ENABLED.store(false, Ordering::Relaxed);
                crate::mt2::MT_ENABLED.store(false, Ordering::Relaxed);
                crate::ptp::reset();
            }
        }

        info!("USB bus reset");
    }

    fn addressed(&mut self, addr: u8) {
        debug!("USB address set to {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        CONFIGURED_AT_TICKS.store(embassy_time::Instant::now().as_ticks(), Ordering::Relaxed);
        crate::device_profile::SCROLL_ACCUM_RESET.store(true, Ordering::Relaxed);
        if configured {
            info!("USB device configured");
        } else {
            info!("USB device unconfigured");
        }
    }

    fn suspended(&mut self, suspended: bool) {
        if suspended {
            debug!("USB suspended");
        } else {
            // Reset high-res scroll on resume. Sleep/wake on Windows often
            // resumes without a full bus reset or reconfiguration, but the OS
            // may not re-send SET_REPORT to re-enable hires. Clear the flag
            // so the host must re-negotiate — same rationale as reset()/configured().
            HIRES_SCROLL_ENABLED.store(false, Ordering::Relaxed);
            crate::ptp::reset();
            crate::device_profile::SCROLL_ACCUM_RESET.store(true, Ordering::Relaxed);
            debug!("USB resumed, high-res scroll + PTP reset");
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
#[allow(dead_code)]
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

    buf
}
