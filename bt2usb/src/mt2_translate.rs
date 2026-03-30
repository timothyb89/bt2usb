//! Magic Trackpad 2: BT Classic → USB report translation.
//!
//! The MT2 uses identical 9-byte touch point encoding over both BT and USB.
//! Only the report headers differ:
//!
//! BT (Report ID 0x31):  [0x31, clicks_ts, ts_lo, ts_hi, <N*9 touch bytes>]
//! USB (Report ID 0x02): [0x02, clicks, 0,0,0,0,0, count, 0x31, ts_lo, ts_hi, 0xE6, <N*9 touch bytes>]
//!
//! This module converts between the two formats, using the exact header layout
//! that macOS's AppleMultitouchDriver expects (verified via the working scroll
//! emulation templates in mt2.rs).

/// Maximum output buffer size: 12-byte USB header + 15*9 = 147 bytes max.
pub const MAX_USB_REPORT: usize = 147;

/// Translate a BT Classic MT2 report (Report ID 0x31) to USB format (Report ID 0x02).
///
/// `bt_data` is the HIDP report payload starting with Report ID 0x31.
/// Returns the number of bytes written to `usb_out`, or None if the input is invalid.
pub fn bt_to_usb(bt_data: &[u8], usb_out: &mut [u8]) -> Option<usize> {
    // Minimum BT report: 4-byte header (report_id + clicks_ts + ts_lo + ts_hi)
    if bt_data.len() < 4 || bt_data[0] != 0x31 {
        return None;
    }

    let n_touch_bytes = bt_data.len() - 4;
    if !n_touch_bytes.is_multiple_of(9) {
        return None;
    }

    let n_fingers = n_touch_bytes / 9;
    let usb_len = 12 + n_touch_bytes;

    if usb_out.len() < usb_len {
        return None;
    }

    // Build USB 12-byte header.
    // This matches the exact format from real MT2 USB captures and the
    // working scroll emulation templates (mt2.rs TEMPLATE_TOUCH).
    usb_out[0] = 0x02; // USB Report ID
    usb_out[1] = bt_data[1] & 0x01; // Click button (bit 0 only; BT has timestamp in upper bits)
    usb_out[2] = 0x00;
    usb_out[3] = 0x00;
    usb_out[4] = 0x00;
    usb_out[5] = 0x00;
    usb_out[6] = 0x00;
    // Byte 7: finger count/presence indicator.
    // From working templates: 0x03 when fingers present, 0x00 when none.
    // Real MT2 USB uses the number of active touch points here.
    usb_out[7] = n_fingers as u8;
    usb_out[8] = 0x31; // Constant marker
    usb_out[9] = bt_data[2]; // Timestamp low
    usb_out[10] = bt_data[3]; // Timestamp high
    usb_out[11] = 0xE6; // Constant marker

    // Copy touch data verbatim — the 9-byte per-finger encoding
    // (13-bit X/Y, pressure, state, touch_major/minor, orientation, finger_id)
    // is identical between BT and USB.
    usb_out[12..usb_len].copy_from_slice(&bt_data[4..]);

    Some(usb_len)
}
