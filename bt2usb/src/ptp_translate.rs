//! Magic Trackpad 2: BT Classic → PTP (Precision Touchpad) reclocked passthrough.
//!
//! Same interpolation strategy as `mt2_translate` — BT interval EMA, 20.12
//! fixed-point per-finger XY, freeze-after-interpolation — but outputs
//! PTP-format reports for Windows and Linux.
//!
//! Coordinate mapping:
//!   MT2 X [-3678, +3934] → PTP X [0, 7612]  (Physical 160.0 mm ≈ 1209 DPI)
//!   MT2 Y [-2478, +2587] → PTP Y [0, 5065]  (Physical 114.9 mm ≈ 1120 DPI)
//!   MT2 −Y = top of pad  → PTP 0 = top

use crate::interp::{self, InterpolationCore};
use crate::ptp::{self, PTP_REPORT_SIZE};

/// Scan time increment per 4 ms tick (4000 µs / 100 µs units = 40).
const SCAN_TIME_INCREMENT: u16 = 40;

// --- Coordinate conversion ---

/// Map MT2 X coordinate to PTP unsigned X.
fn mt2_x_to_ptp(x: i16) -> u16 {
    ((x as i32) - (ptp::MT2_X_MIN as i32)).clamp(0, ptp::MT2_X_SPAN as i32) as u16
}

/// Map MT2 Y coordinate to PTP unsigned Y.
/// MT2 uses negative Y = top; PTP uses 0 = top — no sign flip needed, just offset.
fn mt2_y_to_ptp(y: i16) -> u16 {
    ((y as i32) - (ptp::MT2_Y_MIN as i32)).clamp(0, ptp::MT2_Y_SPAN as i32) as u16
}

// --- Main passthrough struct ---

/// Reclocked passthrough with linear interpolation, outputting PTP format.
pub struct PtpPassthrough {
    core: InterpolationCore,
    /// PTP scan time counter (100 µs units, wrapping 16-bit).
    scan_time: u16,
}

impl PtpPassthrough {
    pub const fn new() -> Self {
        Self {
            core: InterpolationCore::new(),
            scan_time: 0,
        }
    }

    /// Translate a BT Classic MT2 report and set up interpolation.
    ///
    /// `bt_data` is the full HIDP report: `[0x31][hdr 3 bytes][touch×N]`.
    pub fn receive_bt_report(&mut self, bt_data: &[u8]) {
        let Some(info) = self.core.receive_bt_report(bt_data) else {
            return;
        };

        // Diagnostic logging (first 20 reports, then every 200th).
        if info.count <= 20 || info.count.is_multiple_of(200) {
            defmt::debug!(
                "[ptp] BT #{} dt={} est={} n={}",
                info.count,
                info.dt,
                info.interval,
                info.n_points,
            );
        }
    }

    /// Called every 4 ms. Returns a PTP report to send, or `None` if idle.
    pub fn tick(&mut self) -> Option<[u8; PTP_REPORT_SIZE]> {
        let out = self.core.tick()?;

        let n = out.n_fingers as usize;
        let mut fingers = [(false, false, 0u8, 0u16, 0u16); interp::MAX_FINGERS];
        let mut active_count: u8 = 0;

        for i in 0..n.min(interp::MAX_FINGERS) {
            let state = out.states[i];
            // 0x80 (contact) and 0x40 (near): tip down.
            // 0xC0 (release): send one frame with tip_switch=0, confidence=1.
            // 0x00 (none): omit entirely.
            let tip_switch = state == 0x80 || state == 0x40;
            let confidence = state != 0x00;

            if confidence {
                let mt2_x = interp::from_fixed(out.x[i]);
                let mt2_y = interp::from_fixed(out.y[i]);
                fingers[active_count as usize] = (
                    confidence,
                    tip_switch,
                    out.ids[i],
                    mt2_x_to_ptp(mt2_x),
                    mt2_y_to_ptp(mt2_y),
                );
                active_count += 1;
            }
        }

        self.scan_time = self.scan_time.wrapping_add(SCAN_TIME_INCREMENT);

        let mut report = [0u8; PTP_REPORT_SIZE];
        ptp::serialize_ptp_report(
            &mut report,
            &fingers,
            active_count,
            self.scan_time,
            out.button,
        );
        Some(report)
    }
}
