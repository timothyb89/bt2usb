//! Magic Trackpad 2 (MT2) emulation module
//!
//! Provides everything needed to present as an Apple Magic Trackpad 2 over USB:
//! - HID report descriptors for all 4 interfaces (matching the real device)
//! - Feature report data captured from a real MT2
//! - Request handlers that serve feature reports to macOS
//! - Touch report synthesis for converting scroll deltas to 2-finger gestures
//!
//! The real MT2 has VID 0x05AC, PID 0x0265, and 4 USB HID interfaces:
//!   Interface 0: Device Management (vendor FF00/0B)
//!   Interface 1: Mouse + Trackpad (Generic Desktop 01/02, Digitizer 0D/05, Vendor FF00/0C)
//!   Interface 2: Vendor (FF00/0D)
//!   Interface 3: Vendor (FF00/03)

use core::sync::atomic::{AtomicBool, Ordering};
use defmt::*;
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;

// Real MT2 touch reports captured from USB (2-finger scroll gesture).
// Used for replay testing to verify the USB data path works.
pub const REAL_MT2_REPORTS: &[[u8; 30]] = &[
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x5c, 0x29, 0xe6, 0x9b, 0x5b, 0xb6, 0x23, 0x53, 0x7e, 0x18, 0x0d, 0x25, 0xbe, 0xff, 0xc7, 0x23, 0x6c, 0x87, 0x12, 0x0d, 0x28],
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0xb4, 0x29, 0xe6, 0xa5, 0x5b, 0xbb, 0x6b, 0x5a, 0x83, 0x1a, 0x10, 0x85, 0xc2, 0x3f, 0xcf, 0x6f, 0x6e, 0x87, 0x15, 0x0e, 0x88],
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x0c, 0x2a, 0xe6, 0xaf, 0xdb, 0xc0, 0x8b, 0x62, 0x89, 0x1b, 0x15, 0x85, 0xda, 0xff, 0xd6, 0x8f, 0x75, 0x80, 0x16, 0x12, 0x08],
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x64, 0x2a, 0xe6, 0xb8, 0x9b, 0xc4, 0x8b, 0x62, 0x87, 0x1b, 0x15, 0x85, 0xe4, 0xff, 0xda, 0x8f, 0x7f, 0x7a, 0x17, 0x13, 0x08],
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0xc4, 0x2a, 0xe6, 0xc2, 0xdb, 0xc7, 0x8b, 0x68, 0x8a, 0x1c, 0x16, 0x85, 0xf1, 0x5f, 0xdf, 0x8f, 0x82, 0x76, 0x18, 0x14, 0x08],
    [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x1c, 0x2b, 0xe6, 0xd3, 0x3b, 0xcb, 0x8b, 0x76, 0x88, 0x1d, 0x16, 0x85, 0x04, 0x80, 0xe3, 0x8f, 0x77, 0x7f, 0x18, 0x14, 0x88],
    [0x02, 0x00, 0x00, 0xfd, 0x00, 0x00, 0x00, 0x03, 0x31, 0x74, 0x2b, 0xe6, 0xe3, 0x9b, 0xce, 0x8b, 0x81, 0x84, 0x1d, 0x16, 0x05, 0x16, 0x80, 0xe7, 0x8f, 0x6c, 0x7e, 0x18, 0x14, 0x88],
    [0x02, 0x00, 0x00, 0xfd, 0x00, 0x00, 0x00, 0x03, 0x31, 0xcc, 0x2b, 0xe6, 0xf2, 0x1b, 0xd2, 0x8b, 0x82, 0x83, 0x1d, 0x16, 0x85, 0x26, 0xa0, 0xeb, 0x8f, 0x68, 0x7f, 0x18, 0x14, 0x88],
    [0x02, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x03, 0x31, 0x2c, 0x2c, 0xe6, 0x01, 0x9c, 0xd5, 0x8b, 0x83, 0x85, 0x1d, 0x15, 0x85, 0x35, 0xc0, 0xef, 0x8f, 0x67, 0x7d, 0x19, 0x14, 0x88],
    [0x02, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x03, 0x31, 0x84, 0x2c, 0xe6, 0x0f, 0x5c, 0xd9, 0x8b, 0x7f, 0x86, 0x1d, 0x14, 0x85, 0x43, 0x20, 0xf4, 0x8f, 0x67, 0x7f, 0x19, 0x13, 0x88],
    [0x02, 0x00, 0x00, 0xfb, 0x00, 0x00, 0x00, 0x03, 0x31, 0xdc, 0x2c, 0xe6, 0x1d, 0x3c, 0xdd, 0x8b, 0x7d, 0x86, 0x1d, 0x14, 0x85, 0x51, 0xc0, 0xf8, 0x8f, 0x69, 0x80, 0x19, 0x13, 0x88],
    [0x02, 0x00, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x03, 0x31, 0x34, 0x2d, 0xe6, 0x34, 0xfc, 0xe2, 0x8b, 0x77, 0x85, 0x1d, 0x13, 0x85, 0x66, 0xc0, 0xff, 0x8f, 0x6d, 0x83, 0x1a, 0x13, 0x88],
    [0x02, 0x00, 0x01, 0xf5, 0x00, 0x00, 0x00, 0x03, 0x31, 0x94, 0x2d, 0xe6, 0x49, 0xfc, 0xe8, 0x8b, 0x76, 0x85, 0x1e, 0x13, 0x85, 0x7a, 0xc0, 0x06, 0x8c, 0x72, 0x82, 0x1a, 0x13, 0x88],
    [0x02, 0x00, 0x02, 0xf3, 0x00, 0x00, 0x00, 0x03, 0x31, 0xec, 0x2d, 0xe6, 0x5e, 0xfc, 0xee, 0x8b, 0x71, 0x86, 0x1e, 0x13, 0x85, 0x8e, 0xe0, 0x0d, 0x8c, 0x76, 0x82, 0x1b, 0x13, 0x88],
    [0x02, 0x00, 0x03, 0xf4, 0x00, 0x00, 0x00, 0x03, 0x31, 0x44, 0x2e, 0xe6, 0x6b, 0xfc, 0xf2, 0x8b, 0x6f, 0x89, 0x1e, 0x13, 0x85, 0xa1, 0xe0, 0x14, 0x8c, 0x7a, 0x86, 0x1b, 0x13, 0x88],
    [0x02, 0x00, 0x01, 0xf6, 0x00, 0x00, 0x00, 0x03, 0x31, 0x9c, 0x2e, 0xe6, 0x7e, 0x9c, 0xf8, 0x8b, 0x6f, 0x8b, 0x1e, 0x14, 0x85, 0xad, 0x40, 0x19, 0x8c, 0x7c, 0x89, 0x1c, 0x14, 0x88],
    [0x02, 0x00, 0x01, 0xf7, 0x00, 0x00, 0x00, 0x03, 0x31, 0xfc, 0x2e, 0xe6, 0x8f, 0xdc, 0xfd, 0x8b, 0x6f, 0x8d, 0x1f, 0x15, 0x85, 0xbe, 0x40, 0x1f, 0x8c, 0x7f, 0x87, 0x1c, 0x15, 0x88],
    [0x02, 0x00, 0x02, 0xf5, 0x00, 0x00, 0x00, 0x03, 0x31, 0x54, 0x2f, 0xe6, 0x98, 0x7c, 0x00, 0x88, 0x6d, 0x88, 0x1e, 0x15, 0x85, 0xc8, 0x40, 0x22, 0x8c, 0x7f, 0x85, 0x1c, 0x15, 0x88],
    [0x02, 0x00, 0x00, 0xfb, 0x00, 0x00, 0x00, 0x03, 0x31, 0xac, 0x2f, 0xe6, 0xa5, 0x3c, 0x04, 0x88, 0x6d, 0x87, 0x1e, 0x14, 0x85, 0xd6, 0xc0, 0x26, 0x8c, 0x80, 0x83, 0x1c, 0x14, 0x88],
    [0x02, 0x00, 0x02, 0xf9, 0x00, 0x00, 0x00, 0x03, 0x31, 0x04, 0x30, 0xe6, 0xaf, 0x3c, 0x07, 0x88, 0x6e, 0x86, 0x1e, 0x12, 0x85, 0xe3, 0x20, 0x2a, 0x8c, 0x80, 0x82, 0x1b, 0x12, 0x88],
];

// ============ USB Device Identity ============

pub const APPLE_VID: u16 = 0x05AC;
pub const MT2_PID: u16 = 0x0265;
pub const MT2_DEVICE_RELEASE: u16 = 0x0871;
pub const MT2_MANUFACTURER: &str = "Apple Inc.";
pub const MT2_PRODUCT: &str = "Magic Trackpad";

/// Whether multitouch mode has been activated by the host.
/// Set when macOS sends SET_REPORT(Feature(0x02), [0x01]) or Feature(0xC3, [0x01]).
pub static MT_ENABLED: AtomicBool = AtomicBool::new(false);

// ============ Interface 0: Device Management (FF00/0B) ============

/// HID report descriptor for Interface 0 — Device Management.
///
/// Byte-for-byte match of the real MT2's Interface 0 descriptor (83 bytes).
/// No Feature reports are declared — the real device relies on the driver's
/// hardcoded knowledge of Apple devices to issue GET_REPORT requests, and
/// our RequestHandler serves them without descriptor declarations.
pub const DEVICE_MGMT_REPORT_DESC: &[u8] = &[
    // === Collection 1: Vendor Device Management (FF00/0B) ===
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x0B,       // Usage (Vendor 0x0B)
    0xA1, 0x01,       // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0B,       //   Usage (Vendor 0x0B)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    // Input: Report ID 0xE0 (4 bytes)
    0x96, 0x04, 0x00, //   Report Count (4)
    0x85, 0xE0,       //   Report ID (0xE0)
    0x81, 0x22,       //   Input (Data, Variable, Absolute, No Preferred)
    // Input: Report ID 0x9A (1 byte)
    0x09, 0x0B,       //   Usage (Vendor 0x0B)
    0x96, 0x01, 0x00, //   Report Count (1)
    0x85, 0x9A,       //   Report ID (0x9A)
    0x81, 0x22,       //   Input (Data, Variable, Absolute, No Preferred)
    0xC0,             // End Collection

    // === Collection 2: Power/Battery (FF00/0x14) ===
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x14,       // Usage (Vendor 0x14)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x90,       //   Report ID (0x90)
    0x05, 0x84,       //   Usage Page (Power Device)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x03,       //   Report Count (3)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x09, 0x61,       //   Usage (Good)
    0x05, 0x85,       //   Usage Page (Battery System)
    0x09, 0x44,       //   Usage (Charging)
    0x09, 0x46,       //   Usage (?)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0x95, 0x05,       //   Report Count (5)
    0x81, 0x01,       //   Input (Constant) - padding
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x01,       //   Report Count (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x09, 0x65,       //   Usage (Absolute State of Charge)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0xC0,             // End Collection
];

// ============ Interface 1: Mouse + Trackpad (01/02) ============

/// HID report descriptor for Interface 1 — byte-for-byte match of the real MT2.
///
/// 110 bytes, identical to the real Magic Trackpad 2. Contains:
///   Collection 1: Mouse (Generic Desktop) — Report ID 0x02, 7 bytes
///   Collection 2: Digitizer Touch Pad — Report ID 0x3F, 16 bytes vendor
///   Collection 3: Vendor Multitouch — Report ID 0x44, 1387 bytes
///
/// No Finger collections or Feature declarations — the real MT2 doesn't
/// have them either. macOS reports `digitizer: 0` and "Invalid digitizer
/// transducer" for both the real MT2 and our device — this is normal and
/// does not prevent scrolling (confirmed by testing the real MT2 on the VM).
pub const TRACKPAD_REPORT_DESC: &[u8] = &[
    // === Collection 1: Mouse (Generic Desktop / Mouse) ===
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Button)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x03,       //     Usage Maximum (3)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x85, 0x02,       //     Report ID (2)
    0x95, 0x03,       //     Report Count (3)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data, Variable, Absolute)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x05,       //     Report Size (5)
    0x81, 0x01,       //     Input (Constant) - padding
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x06,       //     Input (Data, Variable, Relative)
    0x95, 0x04,       //     Report Count (4)
    0x75, 0x08,       //     Report Size (8)
    0x81, 0x01,       //     Input (Constant) - padding
    0xC0,             //   End Collection (Physical)
    0xC0,             // End Collection (Application)

    // === Collection 2: Digitizer Touch Pad (vendor data only) ===
    0x05, 0x0D,       // Usage Page (Digitizer)
    0x09, 0x05,       // Usage (Touch Pad)
    0xA1, 0x01,       // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0C,       //   Usage (Vendor 0x0C)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x10,       //   Report Count (16)
    0x85, 0x3F,       //   Report ID (0x3F)
    0x81, 0x22,       //   Input (Data, Variable, Absolute, No Preferred)
    0xC0,             // End Collection

    // === Collection 3: Vendor Multitouch ===
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x0C,       // Usage (Vendor 0x0C)
    0xA1, 0x01,       // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0C,       //   Usage (Vendor 0x0C)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x85, 0x44,       //   Report ID (0x44)
    0x75, 0x08,       //   Report Size (8)
    0x96, 0x6B, 0x05, //   Report Count (1387)
    0x81, 0x00,       //   Input (Data, Array, Absolute)
    0xC0,             // End Collection
];

// ============ Interface 2: Vendor (FF00/0D) ============

/// HID report descriptor for Interface 2 — matches real MT2.
pub const VENDOR2_REPORT_DESC: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x0D,       // Usage (Vendor 0x0D)
    0xA1, 0x01,       // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0D,       //   Usage (Vendor 0x0D)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    // Input: Report ID 0x3F (15 bytes)
    0x85, 0x3F,       //   Report ID (0x3F)
    0x96, 0x0F, 0x00, //   Report Count (15)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    // Output: Report ID 0x53 (63 bytes)
    0x09, 0x0D,       //   Usage (Vendor 0x0D)
    0x85, 0x53,       //   Report ID (0x53)
    0x96, 0x3F, 0x00, //   Report Count (63)
    0x91, 0x02,       //   Output (Data, Variable, Absolute)
    0xC0,             // End Collection
];

// ============ Interface 3: Vendor (FF00/03) ============

/// HID report descriptor for Interface 3 — matches real MT2.
pub const VENDOR3_REPORT_DESC: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x03,       // Usage (Vendor 0x03)
    0xA1, 0x01,       // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x03,       //   Usage (Vendor 0x03)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x85, 0xC0,       //   Report ID (0xC0)
    0x96, 0x6B, 0x00, //   Report Count (107)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0xC0,             // End Collection
];

// ============ Feature Report Data (captured from real MT2) ============
// Source: notes/magic-mouse/capture_trackpad_features_result.txt
// All arrays contain DATA ONLY (no Report ID prefix — embassy-usb adds it).

/// Feature 0xDB — Compound device properties (Interface 1).
/// Contains embedded sub-reports for D1, D3, D0, A1, D9, 7F.
/// 75 data bytes. Last 12 bytes approximated (capture truncated at 64 display bytes).
const FEATURE_DB_IF1: &[u8] = &[
    0x01, 0x02, 0x00, 0xd1, 0x81, 0x0f, 0x00, 0xd3,
    0x01, 0x16, 0x1e, 0x05, 0x15, 0x00, 0x14, 0x1e,
    0x62, 0x05, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00,
    0xd0, 0x02, 0x01, 0x00, 0x14, 0x01, 0x00, 0x1e,
    0x00, 0x02, 0x14, 0x02, 0x01, 0x0e, 0x02, 0x00,
    0x07, 0x00, 0xa1, 0x00, 0x00, 0x05, 0x00, 0xfa,
    0x01, 0x11, 0x00, 0xd9, 0xf0, 0x3c, 0x00, 0x00,
    0x20, 0x2b, 0x00, 0x00, 0x44, 0xe3, 0x52,
    // Remaining bytes reconstructed from individual feature reports
    0xff, 0xbd, 0x1e, 0xe4, 0x26, // last 5 bytes of D9 sub-report
    0x05, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, // 7F sub-report entry
];

/// Feature 0xD1 — Device version (Interface 1). 1 data byte.
const FEATURE_D1_IF1: &[u8] = &[0x81];

/// Feature 0xD3 — Surface dimensions (Interface 1). 14 data bytes.
const FEATURE_D3_IF1: &[u8] = &[
    0x01, 0x16, 0x1e, 0x05, 0x15, 0x00, 0x14,
    0x1e, 0x62, 0x05, 0x00, 0x00, 0x01, 0x00,
];

/// Feature 0xD0 — Calibration data (Interface 1). 15 data bytes.
const FEATURE_D0_IF1: &[u8] = &[
    0x02, 0x01, 0x00, 0x14, 0x01, 0x00, 0x1e,
    0x00, 0x02, 0x14, 0x02, 0x01, 0x0e, 0x02, 0x00,
];

/// Feature 0xA1 — Device state (Interface 1). 6 data bytes.
const FEATURE_A1_IF1: &[u8] = &[0x00, 0x00, 0x05, 0x00, 0xfa, 0x01];

/// Feature 0xD9 — Calibration 2 (Interface 1). 16 data bytes.
const FEATURE_D9_IF1: &[u8] = &[
    0xf0, 0x3c, 0x00, 0x00, 0x20, 0x2b, 0x00, 0x00,
    0x44, 0xe3, 0x52, 0xff, 0xbd, 0x1e, 0xe4, 0x26,
];

/// Feature 0x7F — Config (Interface 1). 4 data bytes.
const FEATURE_7F_IF1: &[u8] = &[0x00, 0x00, 0x00, 0x00];

/// Feature 0xDB — Interface 0 variant. 3 data bytes.
const FEATURE_DB_IF0: &[u8] = &[0xfc, 0xff, 0xff];

/// Feature 0xD1 — Interface 0 variant. 2 data bytes.
const FEATURE_D1_IF0: &[u8] = &[0x53, 0x07];

/// Feature 0x01 — Multitouch report info (Interface 1). 4 data bytes.
/// Byte 3 (0x02) tells AppleMultitouchDevice which Report ID carries MT data.
const FEATURE_01_IF1: &[u8] = &[0x99, 0x00, 0x02, 0x00];

/// Feature 0x34 — Bluetooth info (Interface 0). 76 data bytes.
const FEATURE_34_IF0: &[u8] = &[
    0x03, 0x18, 0x03, 0xe0, 0xeb, 0x40, 0xf1, 0x10,
    0x19, 0x00, 0x25, 0x94, 0x4d, 0x61, 0x67, 0x69,
    0x63, 0x20, 0x54, 0x72, 0x61, 0x63, 0x6b, 0x70,
    0x61, 0x64, 0x20, 0x32, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
];

/// Feature 0xB4 — ST firmware version (Interface 0). 2 data bytes.
const FEATURE_B4_IF0: &[u8] = &[0x08, 0x71];

/// Feature 0xC5 — Wake reason (Interface 0). 1 data byte.
const FEATURE_C5_IF0: &[u8] = &[0x01];

/// Feature 0xE0 — Critical error (Interface 0). 4 data bytes.
const FEATURE_E0_IF0: &[u8] = &[0x00, 0x00, 0x00, 0x00];

// ============ Request Handlers ============

/// Write a Feature report response into `buf`, prepending the report ID.
///
/// USB HID spec requires GET_REPORT responses to include the report ID as
/// the first byte when report IDs are in use. embassy-usb does NOT add this
/// automatically — the handler must include it.
fn write_feature(buf: &mut [u8], report_id: u8, data: &[u8]) -> usize {
    buf[0] = report_id;
    let len = data.len().min(buf.len() - 1);
    buf[1..1 + len].copy_from_slice(&data[..len]);
    1 + len
}

/// Feature report handler for Interface 1 (Mouse/Trackpad).
///
/// Serves Apple proprietary feature reports and handles multitouch activation.
/// Feature 0x01 is stateful: the driver SETs a selector, then GETs the response.
pub struct Mt2TrackpadRequestHandler {
    /// Last data written via SET Feature(0x01). The driver uses this to select
    /// which sub-report to read. GET Feature(0x01) echoes it back.
    feature_01_data: [u8; 4],
}

impl Mt2TrackpadRequestHandler {
    pub const fn new() -> Self {
        Self {
            // Default from real device capture: [sub_id, 0x00, len_lo, len_hi]
            // Real device returns [0x01, 0x99, 0x00, 0x02, 0x00] for GET Feature(0x01)
            // where 0x99 = report count(?), 0x00 = padding, 0x02 = MT report ID, 0x00 = high
            feature_01_data: [0x99, 0x00, 0x02, 0x00],
        }
    }
}

impl RequestHandler for Mt2TrackpadRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let len = match id {
            ReportId::Feature(0x00) => {
                // Default/device info. Real device returns [0x00, 0x01].
                write_feature(buf, 0x00, &[0x01])
            }
            ReportId::Feature(0x01) => write_feature(buf, 0x01, &self.feature_01_data),
            ReportId::Feature(0x02) => {
                // Multitouch activation state
                let enabled = MT_ENABLED.load(Ordering::Relaxed);
                let val = if enabled { 0x01 } else { 0x00 };
                write_feature(buf, 0x02, &[val])
            }
            ReportId::Feature(0xDB) => write_feature(buf, 0xDB, FEATURE_DB_IF1),
            ReportId::Feature(0xD1) => write_feature(buf, 0xD1, FEATURE_D1_IF1),
            ReportId::Feature(0xD3) => write_feature(buf, 0xD3, FEATURE_D3_IF1),
            ReportId::Feature(0xD0) => write_feature(buf, 0xD0, FEATURE_D0_IF1),
            ReportId::Feature(0xA1) => write_feature(buf, 0xA1, FEATURE_A1_IF1),
            ReportId::Feature(0xD9) => write_feature(buf, 0xD9, FEATURE_D9_IF1),
            ReportId::Feature(0x7F) => write_feature(buf, 0x7F, FEATURE_7F_IF1),
            _ => {
                info!("MT2 trackpad: GET {:?} -> not handled", id);
                return None;
            }
        };
        info!("MT2 trackpad: GET Feature(0x{:02X}) -> {} bytes", buf[0], len);
        Some(len)
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        match id {
            ReportId::Feature(0x01) => {
                // Sub-report selector protocol used by decodeDeviceProperty:
                //   1. Host SETs Feature(0x01, [selector=0x01, sub_report_id])
                //   2. Host GETs Feature(0x01) → [selector, sub_id, 0x00, len_lo, len_hi]
                //   3. Host GETs Feature(sub_id) with wLength = len + 1
                //
                // From USB capture of real MT2: GET Feature(0x01) returns
                // [0x01, sub_id, 0x00, data_len_lo, data_len_hi] where data_len
                // is the sub-report DATA size (excluding report ID byte).
                // TopCase reads bytes 3-4 as LE uint16 and adds 1 for wLength.
                if data.len() >= 2 && data[0] == 0x01 {
                    let sub_id = data[1];
                    let data_len: u16 = match sub_id {
                        0xDB => FEATURE_DB_IF1.len() as u16,
                        0xD1 => FEATURE_D1_IF1.len() as u16,
                        0xD3 => FEATURE_D3_IF1.len() as u16,
                        0xD0 => FEATURE_D0_IF1.len() as u16,
                        0xA1 => FEATURE_A1_IF1.len() as u16,
                        0xD9 => FEATURE_D9_IF1.len() as u16,
                        0x7F => FEATURE_7F_IF1.len() as u16,
                        _ => 0,
                    };
                    self.feature_01_data = [
                        sub_id,
                        0x00, // padding byte (confirmed from real MT2 USB capture)
                        (data_len & 0xFF) as u8,
                        ((data_len >> 8) & 0xFF) as u8,
                    ];
                    info!(
                        "MT2 trackpad: SET Feature(0x01) -> sub=0x{:02X} data_len={}",
                        sub_id, data_len
                    );
                } else {
                    let copy_len = data.len().min(self.feature_01_data.len());
                    self.feature_01_data[..copy_len].copy_from_slice(&data[..copy_len]);
                    info!("MT2 trackpad: SET Feature(0x01) ({} bytes)", data.len());
                }
                OutResponse::Accepted
            }
            ReportId::Feature(0x02) => {
                // Multitouch activation: {0x02, 0x01} enables, {0x02, 0x00} disables
                if !data.is_empty() {
                    let enabled = data[0] != 0;
                    MT_ENABLED.store(enabled, Ordering::Relaxed);
                    info!(
                        "MT2 trackpad: SET Feature(0x02) -> multitouch {}",
                        if enabled { "ENABLED" } else { "disabled" }
                    );
                }
                OutResponse::Accepted
            }
            _ => {
                info!("MT2 trackpad: SET {:?} ({} bytes) -> accepted", id, data.len());
                OutResponse::Accepted
            }
        }
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> { None }
}

/// Feature report handler for Interface 0 (Device Management).
///
/// Stores data for echo-back reports (e.g. Feature 0x35) that macOS SETs
/// and then immediately GETs back during initialization.
pub struct Mt2DeviceMgmtRequestHandler {
    /// Buffer for Feature 0x35 echo-back. macOS SETs this during init and
    /// expects to GET it back. STALLing the GET causes "pipe stalled" errors.
    feature_35_data: [u8; 25],
    feature_35_len: usize,
}

impl Mt2DeviceMgmtRequestHandler {
    pub const fn new() -> Self {
        Self {
            feature_35_data: [0u8; 25],
            feature_35_len: 0,
        }
    }
}

impl RequestHandler for Mt2DeviceMgmtRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let len = match id {
            ReportId::Feature(0xC3) => {
                let enabled = MT_ENABLED.load(Ordering::Relaxed);
                let val = if enabled { 0x01 } else { 0x00 };
                write_feature(buf, 0xC3, &[val])
            }
            ReportId::Feature(0x35) => {
                write_feature(buf, 0x35, &self.feature_35_data[..self.feature_35_len])
            }
            ReportId::Feature(0xDB) => write_feature(buf, 0xDB, FEATURE_DB_IF0),
            ReportId::Feature(0xD1) => write_feature(buf, 0xD1, FEATURE_D1_IF0),
            ReportId::Feature(0x34) => write_feature(buf, 0x34, FEATURE_34_IF0),
            ReportId::Feature(0xB4) => write_feature(buf, 0xB4, FEATURE_B4_IF0),
            ReportId::Feature(0xC5) => write_feature(buf, 0xC5, FEATURE_C5_IF0),
            ReportId::Feature(0xE0) => write_feature(buf, 0xE0, FEATURE_E0_IF0),
            // Reports queried by macOS during init — real device data from capture.
            ReportId::Feature(0xBB) => write_feature(buf, 0xBB, &[
                0x01, 0x00, 0x71, 0x08, 0x20, 0x00,
                0x15, 0x05, 0x30, 0x00, 0x18, 0x03,
            ]),
            ReportId::Feature(0x14) => write_feature(buf, 0x14, &[0x00]),
            ReportId::Feature(0xB8) => write_feature(buf, 0xB8, &[0x48, 0x00]),
            ReportId::Feature(0xBC) => write_feature(buf, 0xBC, &[0x00, 0x00]),
            // Battery status (Input report, queried via GET_REPORT)
            // Format: 3 flag bits (Good|Charging|Unknown) + 5 pad + charge%
            ReportId::In(0x90) => write_feature(buf, 0x90, &[0x01, 0x64]),
            _ => {
                info!("MT2 devmgmt: GET {:?} -> not handled", id);
                return None;
            }
        };
        info!("MT2 devmgmt: GET 0x{:02X} -> {} bytes", buf[0], len);
        Some(len)
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        match id {
            ReportId::Feature(0xC3) => {
                if !data.is_empty() {
                    let enabled = data[0] != 0;
                    MT_ENABLED.store(enabled, Ordering::Relaxed);
                    info!(
                        "MT2 devmgmt: SET Feature(0xC3) -> multitouch {}",
                        if enabled { "ENABLED" } else { "disabled" }
                    );
                }
            }
            ReportId::Feature(0x35) => {
                let copy_len = data.len().min(self.feature_35_data.len());
                self.feature_35_data[..copy_len].copy_from_slice(&data[..copy_len]);
                self.feature_35_len = copy_len;
                info!("MT2 devmgmt: SET Feature(0x35) ({} bytes) -> stored", copy_len);
            }
            _ => {
                info!("MT2 devmgmt: SET {:?} ({} bytes) -> accepted", id, data.len());
            }
        }
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> { None }
}

/// Feature report handler for Interface 2 (Vendor/Actuator).
///
/// Handles Feature 0x4A (actuator report) to avoid STALLing the
/// AppleActuatorHIDEventDriver. Returns zeroed data.
pub struct Mt2VendorActuatorRequestHandler;

impl RequestHandler for Mt2VendorActuatorRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        match id {
            ReportId::Feature(0x4A) => {
                // Actuator device properties — return zeroed data to avoid STALL
                let len = write_feature(buf, 0x4A, &[0x00; 16]);
                info!("MT2 actuator: GET Feature(0x4A) -> {} bytes", len);
                Some(len)
            }
            _ => {
                info!("MT2 actuator: GET {:?} -> not handled", id);
                None
            }
        }
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("MT2 actuator: SET {:?} ({} bytes) -> accepted", id, data.len());
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> { None }
}

// ============ Touch Report Synthesis ============

/// MT2 trackpad coordinate space (from Linux kernel hid-magicmouse.c).
#[allow(dead_code)]
const TRACKPAD2_MIN_X: i16 = -3678;
#[allow(dead_code)]
const TRACKPAD2_MAX_X: i16 = 3934;
const TRACKPAD2_MIN_Y: i16 = -2478;
const TRACKPAD2_MAX_Y: i16 = 2587;

/// Idle timeout in ticks before lifting fingers (150ms at 1MHz tick rate).
const IDLE_TIMEOUT_TICKS: u64 = 150_000;

/// Maximum Y displacement before resetting finger positions.
const Y_BOUNDARY: i16 = 2000;

// Template touch reports captured from a real MT2 scroll gesture.
// Index 0 = initial NONE state, 1 = APPROACHING, 2+ = TOUCHING.
// We use these as byte-exact templates and only modify the Y coordinates.
const TEMPLATE_NONE: [u8; 30] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x5c, 0x29, 0xe6, 0x9b, 0x5b, 0xb6, 0x23, 0x53, 0x7e, 0x18, 0x0d, 0x25, 0xbe, 0xff, 0xc7, 0x23, 0x6c, 0x87, 0x12, 0x0d, 0x28];
const TEMPLATE_APPROACH: [u8; 30] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0xb4, 0x29, 0xe6, 0xa5, 0x5b, 0xbb, 0x6b, 0x5a, 0x83, 0x1a, 0x10, 0x85, 0xc2, 0x3f, 0xcf, 0x6f, 0x6e, 0x87, 0x15, 0x0e, 0x88];
const TEMPLATE_TOUCH: [u8; 30] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x0c, 0x2a, 0xe6, 0xaf, 0xdb, 0xc0, 0x8b, 0x62, 0x89, 0x1b, 0x15, 0x85, 0xda, 0xff, 0xd6, 0x8f, 0x75, 0x80, 0x16, 0x12, 0x08];
// Release/lift template (from end of real capture)
const TEMPLATE_RELEASE: [u8; 30] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x31, 0xec, 0x47, 0xe6, 0x18, 0x7d, 0xbe, 0x03, 0x00, 0x00, 0x00, 0x00, 0x85, 0xc2, 0x00, 0xdd, 0xef, 0x00, 0x00, 0x00, 0x00, 0x28];

/// Synthesizes 2-finger scroll gestures from scroll deltas.
///
/// Uses real MT2 captured reports as byte-exact templates, modifying only
/// the Y coordinates via the packed 13-bit encoding. This guarantees the
/// touch attributes, state bytes, and header format match the real device.
pub struct TouchSynthesizer {
    /// Accumulated Y offset from center for both fingers.
    y_offset: i16,
    /// Whether fingers are currently active.
    active: bool,
    /// Reports sent since gesture start (for lifecycle phases).
    report_count: u16,
    /// Tick count of the last scroll event (for idle detection).
    last_event_ticks: u64,
    /// Internal timestamp for report headers.
    timestamp: u16,
}

impl TouchSynthesizer {
    pub const fn new() -> Self {
        Self {
            y_offset: 0,
            active: false,
            report_count: 0,
            last_event_ticks: 0,
            timestamp: 10000, // Start at non-zero like real device
        }
    }

    /// Process a scroll delta. Returns 1-3 reports to send (gesture start
    /// includes NONE + APPROACH before the TOUCH report).
    pub fn process_scroll(&mut self, delta: i16) -> heapless::Vec<[u8; 30], 3> {
        let now = embassy_time::Instant::now().as_ticks();
        self.last_event_ticks = now;
        let mut out = heapless::Vec::new();

        if !self.active {
            // Start new gesture: send NONE then APPROACH then first TOUCH
            self.y_offset = 0;
            self.active = true;
            self.report_count = 0;
            debug!("MT2 touch: gesture START");

            let _ = out.push(self.make_report(&TEMPLATE_NONE));
            let _ = out.push(self.make_report(&TEMPLATE_APPROACH));
        }

        // Apply delta
        self.y_offset = (self.y_offset as i32 + delta as i32)
            .clamp(TRACKPAD2_MIN_Y as i32, TRACKPAD2_MAX_Y as i32) as i16;

        if self.y_offset.abs() > Y_BOUNDARY {
            debug!("MT2 touch: Y boundary hit ({})", self.y_offset);
        }

        let _ = out.push(self.make_report(&TEMPLATE_TOUCH));
        self.report_count = self.report_count.saturating_add(1);
        out
    }

    /// Called on every timer tick (~11ms).
    pub fn tick(&mut self) -> Option<[u8; 30]> {
        if !self.active {
            return None;
        }

        let now = embassy_time::Instant::now().as_ticks();
        if now - self.last_event_ticks > IDLE_TIMEOUT_TICKS {
            self.active = false;
            debug!("MT2 touch: gesture END");
            Some(self.make_report(&TEMPLATE_RELEASE))
        } else {
            // Send continuous TOUCH report at current position
            Some(self.make_report(&TEMPLATE_TOUCH))
        }
    }

    /// Create a report from a template, patching the timestamp and Y coordinates.
    fn make_report(&mut self, template: &[u8; 30]) -> [u8; 30] {
        let mut report = *template;

        // Patch timestamp (bytes 9-10)
        report[9] = (self.timestamp & 0xFF) as u8;
        report[10] = (self.timestamp >> 8) as u8;
        self.timestamp = self.timestamp.wrapping_add(88);

        // Patch Y in both touch points using the packed encoding.
        // We keep the template's X coordinates and attributes intact,
        // and only modify the Y bits (spread across bytes 1-3 of each touch).
        self.patch_y(&mut report, 12, self.y_offset); // finger 0
        self.patch_y(&mut report, 21, self.y_offset); // finger 1

        report
    }

    /// Patch the Y coordinate in a 9-byte packed touch point at the given offset.
    /// Y is encoded as negated 13-bit value across touch bytes 1-3.
    fn patch_y(&self, report: &mut [u8; 30], off: usize, y: i16) {
        let neg_y_raw = ((-y) as u16) & 0x1FFF;
        let t = &mut report[off..off + 9];
        // Byte 1 bits [5:7] = neg_y[2:0]
        t[1] = (t[1] & 0x1F) | (((neg_y_raw & 0x07) << 5) as u8);
        // Byte 2 = neg_y[10:3]
        t[2] = ((neg_y_raw >> 3) & 0xFF) as u8;
        // Byte 3 bits [0:1] = neg_y[12:11], keep state bits [6:7]
        t[3] = (t[3] & 0xFC) | (((neg_y_raw >> 11) & 0x03) as u8);
    }

}

/// Encode a single MT2 touch point into 9 bytes (Apple packed format).
///
/// Same format as the real MT2 sends over USB and as decoded by
/// Linux kernel `magicmouse_emit_touch()` in hid-magicmouse.c.
fn encode_touch_point(id: u8, x: i16, y: i16, state: u8) -> [u8; 9] {
    let x_raw = (x as u16) & 0x1FFF;
    let x_lo = (x_raw & 0xFF) as u8;
    let x_hi = ((x_raw >> 8) & 0x1F) as u8;

    let neg_y_raw = ((-y) as u16) & 0x1FFF;
    let y_b1 = ((neg_y_raw & 0x07) << 5) as u8;
    let y_b2 = ((neg_y_raw >> 3) & 0xFF) as u8;
    let y_b3 = ((neg_y_raw >> 11) & 0x03) as u8;

    let byte1 = x_hi | y_b1;
    let byte3 = y_b3 | (state & 0xC0);

    // Touch attributes vary by state (values from real MT2 captures):
    //   approaching: smaller contact, light pressure
    //   touching: full contact, moderate pressure
    //   releasing: shrinking contact, fading pressure
    //   none: all zeros
    let (touch_major, touch_minor, size, pressure) = match state & 0xC0 {
        0x40 => (85, 130, 22, 10),   // approaching
        0x80 => (100, 135, 26, 15),  // touching
        0xC0 => (70, 120, 18, 5),    // releasing
        _ => (0, 0, 0, 0),           // none
    };
    let byte8 = (id & 0x0F) | (4u8 << 5);

    [x_lo, byte1, y_b2, byte3, touch_major, touch_minor, size, pressure, byte8]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_touch(tdata: &[u8; 9]) -> (u8, i16, i16, u8) {
        let id = tdata[8] & 0x0F;
        let x = (((tdata[1] as i32) << 27 | (tdata[0] as i32) << 19) >> 19) as i16;
        let neg_y = (((tdata[3] as i32) << 30
            | (tdata[2] as i32) << 22
            | (tdata[1] as i32) << 14)
            >> 19) as i16;
        let y = -neg_y;
        let state = tdata[3] & 0xC0;
        (id, x, y, state)
    }

    #[test]
    fn test_encode_decode_origin() {
        let encoded = encode_touch_point(0, 0, 0, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (0, 0, 0, TOUCH_STATE_DOWN));
    }

    #[test]
    fn test_encode_decode_positive() {
        let encoded = encode_touch_point(1, 500, 1000, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (1, 500, 1000, TOUCH_STATE_DOWN));
    }

    #[test]
    fn test_encode_decode_negative() {
        let encoded = encode_touch_point(2, -500, -1000, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (2, -500, -1000, TOUCH_STATE_DOWN));
    }

    #[test]
    fn test_encode_decode_up() {
        let encoded = encode_touch_point(0, 100, -200, TOUCH_STATE_UP);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (0, 100, -200, TOUCH_STATE_UP));
    }

    #[test]
    fn test_encode_decode_extremes() {
        let encoded = encode_touch_point(0, -3678, -2478, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (0, -3678, -2478, TOUCH_STATE_DOWN));

        let encoded = encode_touch_point(1, 3934, 2587, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!((id, x, y, state), (1, 3934, 2587, TOUCH_STATE_DOWN));
    }
}
