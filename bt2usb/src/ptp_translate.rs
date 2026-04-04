//! Magic Trackpad 2: BT Classic → PTP (Precision Touchpad) reclocked passthrough.
//!
//! Same interpolation strategy as `mt2_translate.rs` (EMA interval estimate,
//! 20.12 fixed-point per-finger XY, freeze-after-interpolation) but outputs
//! PTP-format reports instead of Apple MT2 format.
//!
//! Coordinate mapping:
//!   MT2 X [-3678, +3934] → PTP X [0, 7612]
//!   MT2 Y [-2478, +2587] → PTP Y [0, 5065]  (MT2 -Y = top, PTP 0 = top)

use crate::ptp::{self, PTP_REPORT_SIZE};

/// Maximum fingers we track for interpolation.
const MAX_FINGERS: usize = 5;

/// Ticks of no BT input before deactivating (300ms at 4ms tick).
const IDLE_TIMEOUT_TICKS: u16 = 75;

/// Fixed-point fractional bits for interpolation (20.12 format).
const FRAC_BITS: i32 = 12;

/// Default estimated BT interval in ticks (12ms / 4ms = 3 ticks ≈ 83Hz).
const DEFAULT_BT_INTERVAL: u16 = 3;
/// Minimum BT interval (4ms = 1 tick).
const MIN_BT_INTERVAL: u16 = 1;
/// Maximum BT interval (100ms = 25 ticks).
const MAX_BT_INTERVAL: u16 = 25;

/// Scan time increment per 4ms tick (4000μs / 100μs = 40).
const SCAN_TIME_INCREMENT: u16 = 40;

// --- Touch point decode helpers (shared with mt2_translate) ---

/// Decode 13-bit signed X from a 9-byte touch point.
fn decode_x(t: &[u8]) -> i16 {
    let raw = (t[0] as u16) | (((t[1] & 0x1F) as u16) << 8);
    ((raw << 3) as i16) >> 3
}

/// Decode 13-bit signed Y from a 9-byte touch point.
fn decode_y(t: &[u8]) -> i16 {
    let neg_y_raw = ((t[1] >> 5) as u16) | ((t[2] as u16) << 3) | (((t[3] & 0x03) as u16) << 11);
    -(((neg_y_raw << 3) as i16) >> 3)
}

/// Extract finger ID from a 9-byte touch point (byte[8] bits 3-0).
fn finger_id(t: &[u8]) -> u8 {
    t[8] & 0x0F
}

/// Extract touch state from byte[3] bits 6-7.
fn touch_state(t: &[u8]) -> u8 {
    t[3] & 0xC0
}

/// Convert i16 coordinate to 20.12 fixed-point.
fn to_fixed(v: i16) -> i32 {
    (v as i32) << FRAC_BITS
}

/// Convert 20.12 fixed-point back to i16 coordinate.
fn from_fixed(v: i32) -> i16 {
    (v >> FRAC_BITS) as i16
}

/// Convert MT2 X coordinate to PTP X.
fn mt2_x_to_ptp(x: i16) -> u16 {
    ((x as i32) - (ptp::MT2_X_MIN as i32)).clamp(0, ptp::MT2_X_SPAN as i32) as u16
}

/// Convert MT2 Y coordinate to PTP Y.
/// MT2: negative Y = top of pad. PTP: 0 = top.
fn mt2_y_to_ptp(y: i16) -> u16 {
    ((y as i32) - (ptp::MT2_Y_MIN as i32)).clamp(0, ptp::MT2_Y_SPAN as i32) as u16
}

// --- Main passthrough struct ---

/// Reclocked passthrough with linear interpolation, outputting PTP format.
pub struct PtpPassthrough {
    /// PTP scan time counter (100μs units, wrapping 16-bit).
    scan_time: u16,
    /// Whether we're actively sending reports.
    active: bool,
    /// Ticks since last BT report received.
    idle_ticks: u16,
    /// Latched click state (sticky set, cleared after tick sends it).
    click_latched: bool,
    /// Last known BT click state.
    last_bt_click: bool,

    // --- Interpolation state ---
    /// Number of fingers in the current report.
    n_fingers: u8,
    /// Finger IDs from the latest BT report.
    finger_ids: [u8; MAX_FINGERS],
    /// Touch states from the latest BT report.
    finger_states: [u8; MAX_FINGERS],
    /// Current interpolated X positions (20.12 fixed-point, MT2 coordinates).
    cur_x: [i32; MAX_FINGERS],
    /// Current interpolated Y positions (20.12 fixed-point, MT2 coordinates).
    cur_y: [i32; MAX_FINGERS],
    /// X delta per tick (20.12 fixed-point).
    dx: [i32; MAX_FINGERS],
    /// Y delta per tick (20.12 fixed-point).
    dy: [i32; MAX_FINGERS],

    // --- BT interval tracking ---
    /// Ticks elapsed since last BT report.
    ticks_since_bt: u16,
    /// Smoothed estimate of BT report interval in ticks (EMA).
    bt_interval: u16,
    /// Ticks remaining in current interpolation.
    interp_remaining: u16,
    /// Whether we have a previous report's positions to interpolate from.
    has_prev: bool,
    /// Diagnostic: BT report counter.
    diag_bt_count: u32,
}

impl PtpPassthrough {
    pub const fn new() -> Self {
        Self {
            scan_time: 0,
            active: false,
            idle_ticks: 0,
            click_latched: false,
            last_bt_click: false,
            n_fingers: 0,
            finger_ids: [0; MAX_FINGERS],
            finger_states: [0; MAX_FINGERS],
            cur_x: [0; MAX_FINGERS],
            cur_y: [0; MAX_FINGERS],
            dx: [0; MAX_FINGERS],
            dy: [0; MAX_FINGERS],
            ticks_since_bt: 0,
            bt_interval: DEFAULT_BT_INTERVAL,
            interp_remaining: 0,
            has_prev: false,
            diag_bt_count: 0,
        }
    }

    /// Translate a BT Classic MT2 report and set up interpolation.
    ///
    /// `bt_data` is the full HIDP report: `[0x31][hdr 3 bytes][touch×N]`.
    pub fn receive_bt_report(&mut self, bt_data: &[u8]) {
        if bt_data.len() < 4 || bt_data[0] != 0x31 {
            return;
        }
        let n_touch_bytes = bt_data.len() - 4;
        if !n_touch_bytes.is_multiple_of(9) {
            return;
        }
        let n_points = (n_touch_bytes / 9).min(MAX_FINGERS);

        // Track BT click state
        let bt_click = bt_data[1] & 0x01 != 0;
        self.last_bt_click = bt_click;
        if bt_click {
            self.click_latched = true;
        }

        // Update BT interval estimate (EMA: 75% old + 25% new)
        if self.ticks_since_bt > 0 && self.has_prev {
            let measured = self.ticks_since_bt.clamp(MIN_BT_INTERVAL, MAX_BT_INTERVAL);
            self.bt_interval = ((3 * self.bt_interval as u32 + measured as u32) / 4).max(1) as u16;
        }

        // Check if we can interpolate (same finger count + matching IDs)
        let n = n_points as u8;
        let can_interp = self.has_prev && n == self.n_fingers && n > 0 && {
            let mut ids_match = true;
            for i in 0..n as usize {
                let t = &bt_data[4 + i * 9..4 + (i + 1) * 9];
                if finger_id(t) != self.finger_ids[i] {
                    ids_match = false;
                    break;
                }
            }
            ids_match
        };

        let interval = self.bt_interval.max(1) as i32;

        for i in 0..n_points {
            let t = &bt_data[4 + i * 9..4 + (i + 1) * 9];
            let target_x = to_fixed(decode_x(t));
            let target_y = to_fixed(decode_y(t));
            self.finger_ids[i] = finger_id(t);
            self.finger_states[i] = touch_state(t);

            if can_interp {
                self.dx[i] = (target_x - self.cur_x[i]) / interval;
                self.dy[i] = (target_y - self.cur_y[i]) / interval;
            } else {
                self.cur_x[i] = target_x;
                self.cur_y[i] = target_y;
                self.dx[i] = 0;
                self.dy[i] = 0;
            }
        }

        // Diagnostic logging
        self.diag_bt_count += 1;
        if self.diag_bt_count <= 20 || self.diag_bt_count.is_multiple_of(200) {
            defmt::debug!(
                "[ptp] BT #{} dt={} est={} n={}",
                self.diag_bt_count,
                self.ticks_since_bt,
                self.bt_interval,
                n_points,
            );
        }

        self.n_fingers = n;
        self.has_prev = true;
        self.interp_remaining = self.bt_interval;
        self.ticks_since_bt = 0;
        self.idle_ticks = 0;
        self.active = true;
    }

    /// Called every 4ms. Returns a PTP report to send, or None if idle.
    pub fn tick(&mut self) -> Option<[u8; PTP_REPORT_SIZE]> {
        if !self.active {
            return None;
        }

        self.idle_ticks = self.idle_ticks.saturating_add(1);
        self.ticks_since_bt = self.ticks_since_bt.saturating_add(1);

        // Deactivate after idle timeout with no active fingers
        if self.idle_ticks > IDLE_TIMEOUT_TICKS && !self.has_active_fingers() {
            self.active = false;
            self.has_prev = false;
            self.diag_bt_count = 0;
            return None;
        }

        // Advance positions during interpolation
        let advancing = self.interp_remaining > 0;
        if advancing {
            self.interp_remaining -= 1;
        }

        // Build finger data and advance interpolation
        let n = self.n_fingers as usize;
        let mut fingers = [(false, false, 0u8, 0u16, 0u16); MAX_FINGERS];
        let mut active_count: u8 = 0;

        for i in 0..n.min(MAX_FINGERS) {
            if advancing {
                self.cur_x[i] += self.dx[i];
                self.cur_y[i] += self.dy[i];
            }

            let state = self.finger_states[i];
            // State 0x80 (contact) and 0x40 (near/approach): finger active
            // State 0xC0 (release): send one frame with tip_switch=0
            // State 0x00 (none): omit finger
            let tip_switch = state == 0x80 || state == 0x40;
            let confidence = state != 0x00;

            if confidence {
                let mt2_x = from_fixed(self.cur_x[i]);
                let mt2_y = from_fixed(self.cur_y[i]);
                fingers[active_count as usize] = (
                    true, // confidence
                    tip_switch,
                    self.finger_ids[i],
                    mt2_x_to_ptp(mt2_x),
                    mt2_y_to_ptp(mt2_y),
                );
                active_count += 1;
            }
        }

        // Click state
        let button = self.click_latched || self.last_bt_click;
        self.click_latched = false;

        // Advance scan time
        self.scan_time = self.scan_time.wrapping_add(SCAN_TIME_INCREMENT);

        // Serialize PTP report
        let mut report = [0u8; PTP_REPORT_SIZE];
        ptp::serialize_ptp_report(&mut report, &fingers, active_count, self.scan_time, button);
        Some(report)
    }

    /// Check if any finger has an active touch state.
    fn has_active_fingers(&self) -> bool {
        for i in 0..self.n_fingers as usize {
            if self.finger_states[i] == 0x80 || self.finger_states[i] == 0x40 {
                return true;
            }
        }
        false
    }
}
