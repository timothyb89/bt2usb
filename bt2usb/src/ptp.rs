//! Windows Precision Touchpad (PTP) output module.
//!
//! Presents as a PTP-compliant touchpad on Windows and Linux, providing
//! native gesture support (scroll, pinch, 3-finger swipe, etc.) without
//! any custom driver. Modeled on the Ploopy Pavonis Trackpad's proven
//! descriptor structure, adapted for the Magic Trackpad 2's physical
//! dimensions (160mm x 114.9mm).
//!
//! The PTP protocol requires two HID top-level collections:
//! - Touchpad TLC (0x0D/0x05): touch input + capability/certification features
//! - Configuration TLC (0x0D/0x0E): Input Mode + Selective Reporting features
//!
//! The host boots in mouse mode (Input Mode=0) and sends SET_FEATURE to
//! switch to touchpad mode (Input Mode=3).

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use defmt::*;
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;

/// Whether PTP touchpad mode is active (Input Mode = 3).
/// Set when the host sends SET_FEATURE(Input Mode, 3).
pub static PTP_ENABLED: AtomicBool = AtomicBool::new(false);

/// Current Input Mode value (for GET_FEATURE responses).
static PTP_INPUT_MODE: AtomicU8 = AtomicU8::new(0);

// ============ Coordinate Constants ============

/// MT2 X coordinate range: [-3678, +3934], total span 7612.
pub const MT2_X_MIN: i16 = -3678;
pub const MT2_X_SPAN: u16 = 7612;

/// MT2 Y coordinate range: [-2478, +2587], total span 5065.
pub const MT2_Y_MIN: i16 = -2478;
pub const MT2_Y_SPAN: u16 = 5065;

/// Maximum fingers in PTP reports.
pub const MAX_FINGERS: usize = 5;

/// PTP input report size: 1 byte report ID + 5 * 6 bytes fingers + 3 bytes global = 34 bytes.
pub const PTP_REPORT_SIZE: usize = 34;

/// PTP Report ID for touch input.
pub const PTP_INPUT_REPORT_ID: u8 = 0x08;

// ============ PTP HID Report Descriptor ============

/// PTP HID report descriptor — based on Ploopy Pavonis Trackpad (known working
/// on Windows and Linux), with coordinate ranges adapted for MT2 surface.
///
/// Structure: Touchpad TLC + Configuration TLC (565 bytes).
/// Report IDs: 0x08 (input), 0x0A (input mode), 0x0B (capabilities),
///             0x0C (selective reporting), 0x0D (certification).
///
/// Per-finger (6 bytes): Confidence(1b) + TipSwitch(1b) + 6b pad +
///   ContactID(3b) + 5b pad + X(16b) + Y(16b)
///
/// Changes from Ploopy:
///   X: Logical Max 7612, Physical Max 1600 (160.0mm)
///   Y: Logical Max 5065, Physical Max 1149 (114.9mm)
///   System Control + Consumer TLCs removed (not needed)
pub const PTP_REPORT_DESC: &[u8] = &[
    // === Touchpad TLC (Usage Page 0x0D, Usage 0x05) ===
    0x05, 0x0D, // Usage Page (Digitizer)
    0x09, 0x05, // Usage (Touch Pad)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x08, //   Report ID (0x08)
    // --- Finger 0 ---
    0x09, 0x22, //   Usage (Finger)
    0xA1, 0x00, //   Collection (Physical)
    0xA4, //     Push
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x09, 0x47, //     Usage (Confidence)
    0x09, 0x42, //     Usage (Tip Switch)
    0x95, 0x02, //     Report Count (2)
    0x75, 0x01, //     Report Size (1)
    0x81, 0x02, //     Input (Data, Var, Abs) — 2 bits
    0x75, 0x01, //     Report Size (1)
    0x95, 0x06, //     Report Count (6)
    0x81, 0x01, //     Input (Const) — 6 bits padding
    0x95, 0x01, //     Report Count (1)
    0x75, 0x03, //     Report Size (3)
    0x25, 0x05, //     Logical Maximum (5)
    0x09, 0x51, //     Usage (Contact ID)
    0x81, 0x02, //     Input (Data, Var, Abs) — 3 bits
    0x75, 0x01, //     Report Size (1)
    0x95, 0x05, //     Report Count (5)
    0x81, 0x01, //     Input (Const) — 5 bits padding
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x15, 0x00, //     Logical Minimum (0)
    0x26, 0xBC, 0x1D, //     Logical Maximum (7612) — MT2 X span
    0x75, 0x10, //     Report Size (16)
    0x55, 0x0E, //     Unit Exponent (-2)
    0x65, 0x11, //     Unit (cm)
    0x09, 0x30, //     Usage (X)
    0x35, 0x00, //     Physical Minimum (0)
    0x46, 0x40, 0x06, //     Physical Maximum (1600) — 160.0mm
    0x95, 0x01, //     Report Count (1)
    0x81, 0x02, //     Input (Data, Var, Abs) — 16 bits X
    0x26, 0xC9, 0x13, //     Logical Maximum (5065) — MT2 Y span
    0x46, 0x7D, 0x04, //     Physical Maximum (1149) — 114.9mm
    0x09, 0x31, //     Usage (Y)
    0x81, 0x02, //     Input (Data, Var, Abs) — 16 bits Y
    0xB4, //     Pop
    0xC0, //   End Collection (Finger 0)
    // --- Finger 1 ---
    0x05, 0x0D, 0x09, 0x22, 0xA1, 0x00, 0xA4, 0x15, 0x00, 0x25, 0x01, 0x09, 0x47, 0x09, 0x42, 0x95,
    0x02, 0x75, 0x01, 0x81, 0x02, 0x75, 0x01, 0x95, 0x06, 0x81, 0x01, 0x95, 0x01, 0x75, 0x03, 0x25,
    0x05, 0x09, 0x51, 0x81, 0x02, 0x75, 0x01, 0x95, 0x05, 0x81, 0x01, 0x05, 0x01, 0x15, 0x00, 0x26,
    0xBC, 0x1D, 0x75, 0x10, 0x55, 0x0E, 0x65, 0x11, 0x09, 0x30, 0x35, 0x00, 0x46, 0x40, 0x06, 0x95,
    0x01, 0x81, 0x02, 0x26, 0xC9, 0x13, 0x46, 0x7D, 0x04, 0x09, 0x31, 0x81, 0x02, 0xB4, 0xC0,
    // --- Finger 2 ---
    0x05, 0x0D, 0x09, 0x22, 0xA1, 0x00, 0xA4, 0x15, 0x00, 0x25, 0x01, 0x09, 0x47, 0x09, 0x42, 0x95,
    0x02, 0x75, 0x01, 0x81, 0x02, 0x75, 0x01, 0x95, 0x06, 0x81, 0x01, 0x95, 0x01, 0x75, 0x03, 0x25,
    0x05, 0x09, 0x51, 0x81, 0x02, 0x75, 0x01, 0x95, 0x05, 0x81, 0x01, 0x05, 0x01, 0x15, 0x00, 0x26,
    0xBC, 0x1D, 0x75, 0x10, 0x55, 0x0E, 0x65, 0x11, 0x09, 0x30, 0x35, 0x00, 0x46, 0x40, 0x06, 0x95,
    0x01, 0x81, 0x02, 0x26, 0xC9, 0x13, 0x46, 0x7D, 0x04, 0x09, 0x31, 0x81, 0x02, 0xB4, 0xC0,
    // --- Finger 3 ---
    0x05, 0x0D, 0x09, 0x22, 0xA1, 0x00, 0xA4, 0x15, 0x00, 0x25, 0x01, 0x09, 0x47, 0x09, 0x42, 0x95,
    0x02, 0x75, 0x01, 0x81, 0x02, 0x75, 0x01, 0x95, 0x06, 0x81, 0x01, 0x95, 0x01, 0x75, 0x03, 0x25,
    0x05, 0x09, 0x51, 0x81, 0x02, 0x75, 0x01, 0x95, 0x05, 0x81, 0x01, 0x05, 0x01, 0x15, 0x00, 0x26,
    0xBC, 0x1D, 0x75, 0x10, 0x55, 0x0E, 0x65, 0x11, 0x09, 0x30, 0x35, 0x00, 0x46, 0x40, 0x06, 0x95,
    0x01, 0x81, 0x02, 0x26, 0xC9, 0x13, 0x46, 0x7D, 0x04, 0x09, 0x31, 0x81, 0x02, 0xB4, 0xC0,
    // --- Finger 4 ---
    0x05, 0x0D, 0x09, 0x22, 0xA1, 0x00, 0xA4, 0x15, 0x00, 0x25, 0x01, 0x09, 0x47, 0x09, 0x42, 0x95,
    0x02, 0x75, 0x01, 0x81, 0x02, 0x75, 0x01, 0x95, 0x06, 0x81, 0x01, 0x95, 0x01, 0x75, 0x03, 0x25,
    0x05, 0x09, 0x51, 0x81, 0x02, 0x75, 0x01, 0x95, 0x05, 0x81, 0x01, 0x05, 0x01, 0x15, 0x00, 0x26,
    0xBC, 0x1D, 0x75, 0x10, 0x55, 0x0E, 0x65, 0x11, 0x09, 0x30, 0x35, 0x00, 0x46, 0x40, 0x06, 0x95,
    0x01, 0x81, 0x02, 0x26, 0xC9, 0x13, 0x46, 0x7D, 0x04, 0x09, 0x31, 0x81, 0x02, 0xB4, 0xC0,
    // --- Scan Time (after all fingers) ---
    0xA4, //     Push
    0x55, 0x0C, //     Unit Exponent (-4) → 100μs units
    0x66, 0x01, 0x10, //     Unit (seconds, SI Linear)
    0x05, 0x0D, //     Usage Page (Digitizer)
    0x09, 0x56, //     Usage (Scan Time)
    0x34, //     Physical Minimum (0) — short form
    0x14, //     Logical Minimum (0) — short form
    0x47, 0xFF, 0xFF, 0x00, 0x00, // Physical Maximum (65535)
    0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65535)
    0x75, 0x10, //     Report Size (16)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x02, //     Input (Data, Var, Abs) — 16 bits scan time
    // --- Contact Count ---
    0x09, 0x54, //     Usage (Contact Count)
    0x25, 0x05, //     Logical Maximum (5)
    0x95, 0x01, //     Report Count (1)
    0x75, 0x04, //     Report Size (4)
    0x81, 0x02, //     Input (Data, Var, Abs) — 4 bits
    // --- Buttons ---
    0x05, 0x09, //     Usage Page (Button)
    0x09, 0x01, //     Usage (Button 1)
    0x09, 0x02, //     Usage (Button 2)
    0x09, 0x03, //     Usage (Button 3)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x03, //     Report Count (3)
    0x81, 0x02, //     Input (Data, Var, Abs) — 3 bits buttons
    0x75, 0x01, //     Report Size (1)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x01, //     Input (Const) — 1 bit padding
    // --- Feature: Contact Count Maximum + Pad Type + Surface Switch (Report ID 0x0B) ---
    0x05, 0x0D, //     Usage Page (Digitizer)
    0x85, 0x0B, //     Report ID (0x0B)
    0x09, 0x55, //     Usage (Contact Count Maximum)
    0x75, 0x04, //     Report Size (4)
    0x95, 0x01, //     Report Count (1)
    0x25, 0x0F, //     Logical Maximum (15)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 4 bits
    0x09, 0x59, //     Usage (Pad Type)
    0x75, 0x03, //     Report Size (3)
    0x95, 0x01, //     Report Count (1)
    0x25, 0x07, //     Logical Maximum (7)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 3 bits
    0x09, 0x57, //     Usage (Surface Switch)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x01, //     Report Count (1)
    0x25, 0x01, //     Logical Maximum (1)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 1 bit
    // --- Feature: Device Certification Status (Report ID 0x0D) ---
    0x06, 0xFF, 0x00, //     Usage Page (Vendor 0xFF00)
    0x85, 0x0D, //     Report ID (0x0D)
    0x09, 0xC5, //     Usage (0xC5)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    0x75, 0x08, //     Report Size (8)
    0x96, 0x00, 0x01, //     Report Count (256)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 256 bytes
    0xB4, //     Pop
    0xC0, //   End Collection (Touchpad TLC)
    // === Configuration TLC (Usage Page 0x0D, Usage 0x0E) ===
    0x05, 0x0D, // Usage Page (Digitizer)
    0x09, 0x0E, // Usage (Device Configuration)
    0xA1, 0x01, // Collection (Application)
    0xA4, //   Push
    // --- Feature: Input Mode (Report ID 0x0A) ---
    0x85, 0x0A, //   Report ID (0x0A)
    0x09, 0x22, //   Usage (Finger)
    0xA1, 0x02, //   Collection (Logical)
    0x09, 0x52, //     Usage (Input Mode)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x0A, //     Logical Maximum (10)
    0x95, 0x01, //     Report Count (1)
    0x75, 0x08, //     Report Size (8)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 1 byte
    0xC0, //   End Collection (Logical)
    // --- Feature: Selective Reporting (Report ID 0x0C) ---
    0x09, 0x22, //   Usage (Finger)
    0xA1, 0x00, //   Collection (Physical)
    0x85, 0x0C, //     Report ID (0x0C)
    0x09, 0x57, //     Usage (Surface Switch)
    0x09, 0x58, //     Usage (Button Switch)
    0x25, 0x01, //     Logical Maximum (1)
    0x95, 0x02, //     Report Count (2)
    0x75, 0x01, //     Report Size (1)
    0xB1, 0x02, //     Feature (Data, Var, Abs) — 2 bits
    0x95, 0x06, //     Report Count (6)
    0xB1, 0x03, //     Feature (Const) — 6 bits padding
    0xB4, //   Pop
    0xC0, //   End Collection (Physical)
    0xC0, // End Collection (Configuration TLC)
];

// ============ Feature Report Constants ============

/// Feature 0x0B: Contact Count Maximum (5) + Pad Type (0 = clickpad) + Surface Switch (0).
/// Bit layout: [ContactCountMax:4][PadType:3][SurfaceSwitch:1]
/// Value: 0b_0_000_0101 = 0x05
const FEATURE_0B_DEFAULT: u8 = 0x05;

// ============ Request Handler ============

/// Feature report handler for the PTP interface.
///
/// Manages Input Mode switching (mouse ↔ touchpad) and serves
/// capability/certification feature reports.
pub struct PtpRequestHandler {
    surface_switch: bool,
    button_switch: bool,
}

impl PtpRequestHandler {
    pub const fn new() -> Self {
        Self {
            surface_switch: true,
            button_switch: true,
        }
    }
}

impl RequestHandler for PtpRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        match id {
            ReportId::Feature(0x0A) => {
                // Input Mode
                let mode = PTP_INPUT_MODE.load(Ordering::Relaxed);
                buf[0] = 0x0A;
                buf[1] = mode;
                info!("PTP: GET Feature(0x0A) Input Mode = {}", mode);
                Some(2)
            }
            ReportId::Feature(0x0B) => {
                // Contact Count Max + Pad Type + Surface Switch
                buf[0] = 0x0B;
                buf[1] = FEATURE_0B_DEFAULT | if self.surface_switch { 0x80 } else { 0 };
                info!("PTP: GET Feature(0x0B) = 0x{:02X}", buf[1]);
                Some(2)
            }
            ReportId::Feature(0x0C) => {
                // Selective Reporting
                buf[0] = 0x0C;
                buf[1] = (self.surface_switch as u8) | ((self.button_switch as u8) << 1);
                info!("PTP: GET Feature(0x0C) Selective Reporting");
                Some(2)
            }
            ReportId::Feature(0x0D) => {
                // Device Certification Status — 256 bytes of zeros (valid for Win10+)
                buf[0] = 0x0D;
                for b in buf[1..257].iter_mut() {
                    *b = 0;
                }
                info!("PTP: GET Feature(0x0D) Certification (257 bytes)");
                Some(257)
            }
            _ => {
                debug!("PTP: GET {:?} -> not handled", id);
                None
            }
        }
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        match id {
            ReportId::Feature(0x0A) => {
                // Input Mode: 0=mouse, 3=touchpad
                // data[0] is the report ID (0x0A), data[1] is the actual mode value
                if data.len() >= 2 {
                    let mode = data[1];
                    PTP_INPUT_MODE.store(mode, Ordering::Relaxed);
                    let enabled = mode == 3;
                    PTP_ENABLED.store(enabled, Ordering::Relaxed);
                    info!(
                        "PTP: SET Feature(0x0A) Input Mode = {} ({})",
                        mode,
                        if enabled { "TOUCHPAD" } else { "mouse" }
                    );
                }
                OutResponse::Accepted
            }
            ReportId::Feature(0x0C) => {
                // Selective Reporting
                // data[0] is the report ID (0x0C), data[1] is the actual data
                if data.len() >= 2 {
                    self.surface_switch = data[1] & 0x01 != 0;
                    self.button_switch = data[1] & 0x02 != 0;
                    info!(
                        "PTP: SET Feature(0x0C) surface={} button={}",
                        self.surface_switch, self.button_switch
                    );
                }
                OutResponse::Accepted
            }
            _ => {
                info!("PTP: SET {:?} ({} bytes) -> accepted", id, data.len());
                OutResponse::Accepted
            }
        }
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }
}

// ============ Report Serialization ============

/// Serialize a PTP touch report into a 35-byte buffer.
///
/// Layout: [ReportID(1)] [Finger0(6)] [Finger1(6)] [Finger2(6)] [Finger3(6)]
///         [Finger4(6)] [ScanTime(2)] [ContactCount+Buttons(1)] [Pad(0)]
///
/// Per finger (6 bytes): [Confidence:1|TipSwitch:1|pad:6] [ContactID:3|pad:5] [X_lo] [X_hi] [Y_lo] [Y_hi]
pub fn serialize_ptp_report(
    buf: &mut [u8; PTP_REPORT_SIZE],
    fingers: &[(bool, bool, u8, u16, u16); MAX_FINGERS], // (confidence, tip_switch, id, x, y)
    finger_count: u8,
    scan_time: u16,
    button: bool,
) {
    buf[0] = PTP_INPUT_REPORT_ID;

    // Serialize each finger (6 bytes each)
    for (i, &(confidence, tip_switch, id, x, y)) in fingers.iter().enumerate().take(MAX_FINGERS) {
        let off = 1 + i * 6;
        if (i as u8) < finger_count {
            buf[off] = (confidence as u8) | ((tip_switch as u8) << 1);
            buf[off + 1] = id & 0x07; // 3 bits
            buf[off + 2] = (x & 0xFF) as u8;
            buf[off + 3] = (x >> 8) as u8;
            buf[off + 4] = (y & 0xFF) as u8;
            buf[off + 5] = (y >> 8) as u8;
        } else {
            // Inactive finger — zero all fields
            buf[off..off + 6].fill(0);
        }
    }

    // Scan Time (2 bytes LE) at offset 31
    buf[31] = (scan_time & 0xFF) as u8;
    buf[32] = (scan_time >> 8) as u8;

    // Contact Count (4 bits) + Button 1-3 (3 bits) + 1 bit padding at offset 33
    buf[33] = (finger_count & 0x0F) | ((button as u8) << 4);
}

/// Reset PTP state (called on USB bus reset / reprobe).
pub fn reset() {
    PTP_ENABLED.store(false, Ordering::Relaxed);
    PTP_INPUT_MODE.store(0, Ordering::Relaxed);
}
