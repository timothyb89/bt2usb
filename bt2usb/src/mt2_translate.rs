//! Magic Trackpad 2: BT Classic → USB reclocked passthrough with interpolation.
//!
//! Buffers incoming BT touch reports and retransmits at a steady USB rate
//! (250 Hz / 4 ms tick). Touch point encoding is byte-identical between BT
//! and USB — only the report headers differ.
//!
//! Interpolation, click latching, and idle tracking are handled by
//! [`crate::interp::InterpolationCore`]. This module is responsible for the
//! MT2-specific USB header format and re-encoding interpolated X/Y back into
//! the 9-byte MT2 touch point wire format.

use crate::interp::{self, InterpolationCore};

/// Maximum USB report size: 12-byte header + 5 fingers × 9 bytes.
const MAX_USB_REPORT: usize = 57;

/// USB timestamp increment per tick (~8 kHz / 250 Hz ≈ 32).
const TIMESTAMP_INCREMENT: u16 = 32;

// --- MT2 touch-point encode helpers ---

/// Encode a 13-bit signed X into a 9-byte touch point.
/// Preserves byte[1] bits 7-5 (Y's low bits).
fn encode_x(t: &mut [u8], x: i16) {
    let raw = (x as u16) & 0x1FFF;
    t[0] = (raw & 0xFF) as u8;
    t[1] = (t[1] & 0xE0) | ((raw >> 8) & 0x1F) as u8;
}

/// Encode a 13-bit signed Y into a 9-byte touch point.
/// Preserves byte[1] bits 4-0 (X's high bits) and byte[3] bits 7-2 (state etc).
fn encode_y(t: &mut [u8], y: i16) {
    let neg_y_raw = ((-y) as u16) & 0x1FFF;
    t[1] = (t[1] & 0x1F) | (((neg_y_raw & 0x07) << 5) as u8);
    t[2] = ((neg_y_raw >> 3) & 0xFF) as u8;
    t[3] = (t[3] & 0xFC) | ((neg_y_raw >> 11) & 0x03) as u8;
}

// --- Main passthrough struct ---

/// Reclocked passthrough with linear interpolation between BT reports.
pub struct Mt2Passthrough {
    core: InterpolationCore,
    /// USB report buffer (header + touch data). Touch bytes are patched in
    /// `tick()` with interpolated X/Y before sending.
    report: [u8; MAX_USB_REPORT],
    /// Actual length of the current report.
    report_len: usize,
    /// Monotonic 16-bit USB timestamp, incremented by 32 per tick.
    timestamp: u16,
}

impl Mt2Passthrough {
    pub const fn new() -> Self {
        Self {
            core: InterpolationCore::new(),
            report: [0u8; MAX_USB_REPORT],
            report_len: 0,
            timestamp: 10000,
        }
    }

    /// Translate a BT Classic MT2 report into USB format and set up interpolation.
    ///
    /// `bt_data` is the full HIDP report: `[0x31][hdr 3 bytes][touch×N]`.
    pub fn receive_bt_report(&mut self, bt_data: &[u8]) {
        let Some(info) = self.core.receive_bt_report(bt_data) else {
            return;
        };
        let n_points = info.n_points;
        let touch_bytes = n_points * 9;

        // Build USB header (12 bytes).
        // Values match real MT2 USB capture (capture_mt2_raw.txt):
        //   r[7] = 0x01 when touch present (real MT2), not 0x03
        //   r[11] = 0x88 (real MT2 constant), not 0xe6
        let r = &mut self.report;
        r[0] = 0x02;
        r[1] = 0x00; // Click patched in tick()
        r[2] = 0x00;
        r[3] = 0x00;
        r[4] = 0x00;
        r[5] = 0x00;
        r[6] = 0x00;
        r[7] = if n_points > 0 { 0x01 } else { 0x00 };
        r[8] = 0x31;
        r[9] = 0x00; // Timestamp patched in tick()
        r[10] = 0x00;
        r[11] = 0x88;

        // Copy touch data (encoding is identical between BT and USB).
        r[12..12 + touch_bytes].copy_from_slice(&bt_data[4..4 + touch_bytes]);
        self.report_len = 12 + touch_bytes;

        // Diagnostic logging (first 50 reports, then every 100th).
        if info.count <= 50 || info.count.is_multiple_of(100) {
            let x0 = if n_points > 0 {
                interp::decode_x(&bt_data[4..13])
            } else {
                0
            };
            let y0 = if n_points > 0 {
                interp::decode_y(&bt_data[4..13])
            } else {
                0
            };
            defmt::debug!(
                "[mt2] BT #{} dt={} est={} n={} x={} y={} interp={}",
                info.count,
                info.dt,
                info.interval,
                n_points,
                x0,
                y0,
                info.can_interp,
            );
        }
    }

    /// Called every 4 ms. Returns a USB report to send, or `None` if idle.
    pub fn tick(&mut self) -> Option<([u8; MAX_USB_REPORT], usize)> {
        let out = self.core.tick()?;

        // Patch interpolated X/Y back into the touch data in the USB buffer.
        let n = out.n_fingers as usize;
        for i in 0..n.min(interp::MAX_FINGERS) {
            let off = 12 + i * 9;
            if off + 9 <= self.report_len {
                let t = &mut self.report[off..off + 9];
                encode_x(t, interp::from_fixed(out.x[i]));
                encode_y(t, interp::from_fixed(out.y[i]));
            }
        }

        // Patch click bit.
        self.report[1] = out.button as u8;

        // Patch timestamp.
        self.report[9] = (self.timestamp & 0xFF) as u8;
        self.report[10] = (self.timestamp >> 8) as u8;
        self.timestamp = self.timestamp.wrapping_add(TIMESTAMP_INCREMENT);

        Some((self.report, self.report_len))
    }
}
