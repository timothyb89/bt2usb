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

/// HID report descriptor for Interface 1 — Mouse + Trackpad.
///
/// This is the EXACT raw descriptor from the real Magic Trackpad 2 (110 bytes).
/// It has NO Finger collections, NO Feature declarations, and NO Contact Count.
///
/// TopCase (AppleUSBTopCaseHIDDriver) is supposed to internally augment this
/// descriptor and populate its report length tables. With the real device this
/// works, enabling correct wLength for GET_REPORT on proprietary feature reports.
/// With our emulated device, TopCase's augmentation doesn't activate — the root
/// cause is under investigation (USB config descriptor differences suspected).
///
/// Report IDs:
///   0x02: Mouse (8 bytes: 3 buttons + 5-bit pad + X/Y relative + 4-byte pad)
///         Overloaded for 30-byte multitouch after SET_REPORT activation
///   0x3F: Vendor digitizer input (16 bytes)
///   0x44: Vendor multitouch input (1387 bytes)
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
            // Default from real device capture
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
                //   2. Host GETs Feature(0x01) → [selector, sub_id, len_lo, len_hi]
                //   3. Host GETs Feature(sub_id) with wLength derived from len
                //
                // The device must fill bytes 2-3 with the TOTAL report length
                // (including report ID byte) as LE uint16. If this is wrong,
                // decodeDeviceProperty sends wLength=1 and gets no data.
                if data.len() >= 2 && data[0] == 0x01 {
                    let sub_id = data[1];
                    let total_len: u16 = match sub_id {
                        0xDB => 1 + FEATURE_DB_IF1.len() as u16,
                        0xD1 => 1 + FEATURE_D1_IF1.len() as u16,
                        0xD3 => 1 + FEATURE_D3_IF1.len() as u16,
                        0xD0 => 1 + FEATURE_D0_IF1.len() as u16,
                        0xA1 => 1 + FEATURE_A1_IF1.len() as u16,
                        0xD9 => 1 + FEATURE_D9_IF1.len() as u16,
                        0x7F => 1 + FEATURE_7F_IF1.len() as u16,
                        _ => 1, // unknown sub-report → just report ID
                    };
                    self.feature_01_data = [
                        data[0],
                        data[1],
                        (total_len & 0xFF) as u8,
                        ((total_len >> 8) & 0xFF) as u8,
                    ];
                    info!(
                        "MT2 trackpad: SET Feature(0x01) -> [{:02X} {:02X}] len={}",
                        data[0], data[1], total_len
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
pub struct Mt2DeviceMgmtRequestHandler;

impl RequestHandler for Mt2DeviceMgmtRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let len = match id {
            ReportId::Feature(0xC3) => {
                let enabled = MT_ENABLED.load(Ordering::Relaxed);
                let val = if enabled { 0x01 } else { 0x00 };
                write_feature(buf, 0xC3, &[val])
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
            ReportId::Feature(0xBC) => write_feature(buf, 0xBC, &[0x00]),
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
        if let ReportId::Feature(0xC3) = id {
            if !data.is_empty() {
                let enabled = data[0] != 0;
                MT_ENABLED.store(enabled, Ordering::Relaxed);
                info!(
                    "MT2 devmgmt: SET Feature(0xC3) -> multitouch {}",
                    if enabled { "ENABLED" } else { "disabled" }
                );
            }
        } else {
            info!("MT2 devmgmt: SET {:?} ({} bytes) -> accepted", id, data.len());
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

/// Fixed X positions for the two synthetic fingers (left and right of center).
const FINGER0_X: i16 = -500;
const FINGER1_X: i16 = 500;

/// Idle timeout in ticks before lifting fingers (150ms at 1MHz tick rate).
const IDLE_TIMEOUT_TICKS: u64 = 150_000;

/// Maximum Y displacement before resetting finger positions.
const Y_BOUNDARY: i16 = 2000;

/// Touch state values (from kernel driver).
const TOUCH_STATE_DOWN: u8 = 0x80;
const TOUCH_STATE_UP: u8 = 0x00;

/// Synthesizes 2-finger scroll gestures from scroll deltas.
///
/// Converts incoming scroll delta values (from the Full Scroll Dial) into
/// MT2-format touch reports with two fingers moving in parallel, which
/// macOS interprets as a scroll gesture.
pub struct TouchSynthesizer {
    /// Current Y positions of the two fingers.
    finger_y: [i16; 2],
    /// Whether fingers are currently "touching" the trackpad.
    fingers_down: bool,
    /// Tick count of the last scroll event (for idle detection).
    last_event_ticks: u64,
    /// Internal 8kHz-equivalent timestamp for report headers.
    /// Real MT2 increments by ~88 per report (~11ms * 8kHz).
    timestamp: u16,
}

impl TouchSynthesizer {
    pub const fn new() -> Self {
        Self {
            finger_y: [0, 0],
            fingers_down: false,
            last_event_ticks: 0,
            timestamp: 0,
        }
    }

    /// Process a scroll delta and return a touch report to send.
    ///
    /// Returns a 30-byte report: [Report ID 0x02, 11-byte header, 2x 9-byte touches].
    pub fn process_scroll(&mut self, delta: i16) -> [u8; 30] {
        let now = embassy_time::Instant::now().as_ticks();
        self.last_event_ticks = now;

        if !self.fingers_down {
            // Start new gesture: place fingers at center
            self.finger_y = [0, 0];
            self.fingers_down = true;
            debug!("MT2 touch: gesture START at Y=0");
        }

        // Move both fingers by the scroll delta
        // Positive delta = scroll down = fingers move down (negative Y)
        // The sign convention matches trackpad natural scrolling
        self.finger_y[0] = (self.finger_y[0] as i32 + delta as i32)
            .clamp(TRACKPAD2_MIN_Y as i32, TRACKPAD2_MAX_Y as i32) as i16;
        self.finger_y[1] = (self.finger_y[1] as i32 + delta as i32)
            .clamp(TRACKPAD2_MIN_Y as i32, TRACKPAD2_MAX_Y as i32) as i16;

        // Check if we've hit the boundary and need to reset
        if self.finger_y[0].abs() > Y_BOUNDARY || self.finger_y[1].abs() > Y_BOUNDARY {
            debug!("MT2 touch: Y boundary hit ({}), will reset on next idle", self.finger_y[0]);
        }

        debug!(
            "MT2 touch: delta={} -> Y=[{}, {}]",
            delta, self.finger_y[0], self.finger_y[1]
        );

        self.build_report(TOUCH_STATE_DOWN)
    }

    /// Check if fingers should be lifted due to idle timeout.
    /// Returns Some(report) if a lift report should be sent.
    pub fn check_idle(&mut self) -> Option<[u8; 30]> {
        if !self.fingers_down {
            return None;
        }

        let now = embassy_time::Instant::now().as_ticks();
        if now - self.last_event_ticks > IDLE_TIMEOUT_TICKS {
            self.fingers_down = false;
            debug!("MT2 touch: gesture END (idle timeout)");
            Some(self.build_report(TOUCH_STATE_UP))
        } else {
            None
        }
    }

    /// Build a 30-byte MT2 USB touch report.
    ///
    /// Format: Report ID 0x02 (overloaded for multitouch after activation).
    /// [0x02, clicks, 10 header bytes, touch0[9], touch1[9]] = 30 bytes.
    ///
    /// Header format (from real MT2 USB captures):
    ///   Byte 1: clicks (0 for scroll)
    ///   Bytes 2-6: counters/state (zeroed is fine)
    ///   Byte 7: 0x01 if fingers down, 0x00 if up
    ///   Byte 8: 0x31 (format marker, always present)
    ///   Bytes 9-10: 16-bit LE timestamp (8kHz clock)
    ///   Byte 11: 0x88 (constant marker)
    fn build_report(&mut self, state: u8) -> [u8; 30] {
        let mut report = [0u8; 30];
        report[0] = 0x02; // Report ID
        report[1] = 0x00; // clicks = 0 (no button press for scroll)
        // Bytes 2-6: zero (real device has minor counters here, not required)
        report[7] = if state == TOUCH_STATE_DOWN { 0x01 } else { 0x00 };
        report[8] = 0x31; // Format marker (constant in all real MT2 reports)
        // Timestamp: 16-bit LE, increments by ~88 per report (~8kHz)
        report[9] = (self.timestamp & 0xFF) as u8;
        report[10] = (self.timestamp >> 8) as u8;
        report[11] = 0x88; // Constant marker

        // Advance timestamp (~88 ticks per report at 8kHz)
        self.timestamp = self.timestamp.wrapping_add(88);

        // Touch point 0 (bytes 12-20)
        let t0 = encode_touch_point(0, FINGER0_X, self.finger_y[0], state);
        report[12..21].copy_from_slice(&t0);

        // Touch point 1 (bytes 21-29)
        let t1 = encode_touch_point(1, FINGER1_X, self.finger_y[1], state);
        report[21..30].copy_from_slice(&t1);

        report
    }
}

/// Encode a single MT2 touch point into 9 bytes.
///
/// This is the inverse of the Linux kernel's decode in `magicmouse_emit_touch()`:
///   id = tdata[8] & 0x0F
///   x = (tdata[1] << 27 | tdata[0] << 19) >> 19   // 13-bit signed
///   y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19)  // 13-bit signed, negated
///   state = tdata[3] & 0xC0
///   touch_major = tdata[4]
///   touch_minor = tdata[5]
///   size = tdata[6]
///   pressure = tdata[7]
///   orientation = (tdata[8] >> 5) - 4
fn encode_touch_point(id: u8, x: i16, y: i16, state: u8) -> [u8; 9] {
    // X is 13-bit signed, stored in bits [0:12] across bytes 0-1
    // The kernel decodes: x = (tdata[1] << 27 | tdata[0] << 19) >> 19
    // This is equivalent to: x = ((tdata[1] & 0x1F) << 8 | tdata[0]) sign-extended from 13 bits
    //
    // Encoding: x_raw = x & 0x1FFF (13-bit two's complement)
    //   byte0 = x_raw & 0xFF         (low 8 bits)
    //   byte1 bits[0:4] = (x_raw >> 8) & 0x1F  (high 5 bits)
    let x_raw = (x as u16) & 0x1FFF; // 13-bit two's complement
    let x_lo = (x_raw & 0xFF) as u8;
    let x_hi = ((x_raw >> 8) & 0x1F) as u8;

    // Y is 13-bit signed, NEGATED, stored across bytes 1-3
    // The kernel decodes: neg_y = (tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19
    // then y = -neg_y
    //
    // So we encode: neg_y = -y, as 13-bit two's complement
    //   neg_y_raw = (-y) & 0x1FFF
    //   byte1 bits[5:7] = neg_y_raw & 0x07  (low 3 bits of neg_y in high 3 bits of byte1)
    //   byte2 = (neg_y_raw >> 3) & 0xFF     (middle 8 bits)
    //   byte3 bits[0:1] = (neg_y_raw >> 11) & 0x03  (high 2 bits)
    let neg_y_raw = ((-y) as u16) & 0x1FFF; // 13-bit two's complement of -y
    let y_b1 = ((neg_y_raw & 0x07) << 5) as u8; // 3 low bits into byte1 bits 5-7
    let y_b2 = ((neg_y_raw >> 3) & 0xFF) as u8; // middle 8 bits
    let y_b3 = ((neg_y_raw >> 11) & 0x03) as u8; // high 2 bits

    // Combine byte1: x high bits (0-4) | y low bits (5-7)
    let byte1 = x_hi | y_b1;

    // byte3: y high bits (0-1) | state (6-7)
    let byte3 = y_b3 | (state & 0xC0);

    // Touch attributes — values based on real MT2 capture data:
    // Real device during scroll: major=80-130, minor=110-150, size=20-28, pressure=5-18
    let touch_major: u8 = if state == TOUCH_STATE_DOWN { 100 } else { 0 };
    let touch_minor: u8 = if state == TOUCH_STATE_DOWN { 130 } else { 0 };
    let size: u8 = if state == TOUCH_STATE_DOWN { 24 } else { 0 };
    let pressure: u8 = if state == TOUCH_STATE_DOWN { 12 } else { 0 };

    // Orientation (0 = neutral) and tracking ID
    // orientation = (tdata[8] >> 5) - 4, so for orient=0: (tdata[8] >> 5) = 4
    // byte8 = (id & 0x0F) | ((orient + 4) << 5)
    let orient_enc = 4u8; // orientation=0 -> encoded as 4
    let byte8 = (id & 0x0F) | (orient_enc << 5);

    [x_lo, byte1, y_b2, byte3, touch_major, touch_minor, size, pressure, byte8]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify encode/decode roundtrip matches the Linux kernel's decode logic.
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
        assert_eq!(id, 0);
        assert_eq!(x, 0);
        assert_eq!(y, 0);
        assert_eq!(state, TOUCH_STATE_DOWN);
    }

    #[test]
    fn test_encode_decode_positive() {
        let encoded = encode_touch_point(1, 500, 1000, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!(id, 1);
        assert_eq!(x, 500);
        assert_eq!(y, 1000);
        assert_eq!(state, TOUCH_STATE_DOWN);
    }

    #[test]
    fn test_encode_decode_negative() {
        let encoded = encode_touch_point(2, -500, -1000, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!(id, 2);
        assert_eq!(x, -500);
        assert_eq!(y, -1000);
        assert_eq!(state, TOUCH_STATE_DOWN);
    }

    #[test]
    fn test_encode_decode_up() {
        let encoded = encode_touch_point(0, 100, -200, TOUCH_STATE_UP);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!(id, 0);
        assert_eq!(x, 100);
        assert_eq!(y, -200);
        assert_eq!(state, TOUCH_STATE_UP);
    }

    #[test]
    fn test_encode_decode_extremes() {
        // Test near MT2 coordinate boundaries
        let encoded = encode_touch_point(0, -3678, -2478, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!(id, 0);
        assert_eq!(x, -3678);
        assert_eq!(y, -2478);
        assert_eq!(state, TOUCH_STATE_DOWN);

        let encoded = encode_touch_point(1, 3934, 2587, TOUCH_STATE_DOWN);
        let (id, x, y, state) = decode_touch(&encoded);
        assert_eq!(id, 1);
        assert_eq!(x, 3934);
        assert_eq!(y, 2587);
        assert_eq!(state, TOUCH_STATE_DOWN);
    }
}
