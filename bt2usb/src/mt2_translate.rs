//! Magic Trackpad 2: BT Classic → USB report translation.
//!
//! The MT2 uses identical 9-byte touch point encoding over both BT and USB.
//! Only the report headers differ:
//!
//! BT (Report ID 0x31):  [0x31, clicks_ts, ts_lo, ts_hi, <N*9 touch bytes>]
//! USB (Report ID 0x02): [0x02, clicks, 0,0,0,0,0, bitmap, 0x31, ts_lo, ts_hi, 0xE6, <N*9 touch bytes>]
//!
//! This module converts between the two formats.

/// Maximum output buffer size: 12-byte USB header + 15*9 = 147 bytes max.
pub const MAX_USB_REPORT: usize = 147;

/// Translate a BT Classic MT2 report (Report ID 0x31) to USB format (Report ID 0x02).
///
/// `bt_data` is the HIDP report payload starting with Report ID 0x31.
/// Returns the number of bytes written to `usb_out`, or None if the input is invalid.
pub fn bt_to_usb(bt_data: &[u8], usb_out: &mut [u8]) -> Option<usize> {
    // Minimum BT report: 4-byte header (report_id + clicks + ts_lo + ts_hi)
    if bt_data.len() < 4 || bt_data[0] != 0x31 {
        return None;
    }

    let n_touch_bytes = bt_data.len() - 4;
    // Touch data must be a multiple of 9 bytes
    if !n_touch_bytes.is_multiple_of(9) {
        return None;
    }

    let n_fingers = n_touch_bytes / 9;
    // macOS may expect at least 2 finger slots (30 bytes total).
    // Pad with zero-state finger data if only 1 finger.
    let min_touch_bytes = 18; // 2 * 9
    let padded_touch_bytes = n_touch_bytes.max(min_touch_bytes);
    let usb_len = 12 + padded_touch_bytes;

    if usb_out.len() < usb_len {
        return None;
    }

    // Zero the output buffer (ensures padding fingers have state=NONE)
    usb_out[..usb_len].fill(0);

    // Build USB 12-byte header.
    // BT byte 1 has: bits 0-5 = click/flags, bits 6-7 = timestamp low.
    // USB byte 1 should only have the click bit (bit 0). The real MT2 USB
    // format has a clean clicks byte without timestamp contamination.
    usb_out[0] = 0x02; // USB Report ID
    usb_out[1] = bt_data[1] & 0x01; // Extract only the click button bit
    usb_out[2] = 0x00;
    usb_out[3] = 0x00;
    usb_out[4] = 0x00;
    usb_out[5] = 0x00;
    usb_out[6] = 0x00;

    // Byte 7: finger presence bitmap/count
    // From real MT2 USB captures, this byte indicates which fingers are present.
    // For simplicity, set a bitmask of active fingers.
    let mut bitmap: u8 = 0;
    for i in 0..n_fingers.min(8) {
        // Each touch point has a finger ID at tdata[8] & 0x0F
        let tdata_offset = 4 + i * 9;
        if tdata_offset + 8 < bt_data.len() {
            let finger_id = bt_data[tdata_offset + 8] & 0x0F;
            bitmap |= 1 << (finger_id & 0x07);
        }
    }
    usb_out[7] = bitmap;

    usb_out[8] = 0x31; // Constant marker (same as BT report ID)
    usb_out[9] = bt_data[2]; // Timestamp low
    usb_out[10] = bt_data[3]; // Timestamp high
    usb_out[11] = 0xE6; // Constant marker (from real MT2 USB captures)

    // Copy touch data verbatim — encoding is identical between BT and USB
    usb_out[12..usb_len].copy_from_slice(&bt_data[4..]);

    Some(usb_len)
}
