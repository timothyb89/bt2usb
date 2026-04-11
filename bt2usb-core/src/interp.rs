//! Shared interpolation core for BT Classic → USB reclocked passthrough.
//!
//! Both `mt2_translate` (MT2/macOS format) and `ptp_translate` (PTP/Windows+Linux
//! format) use identical BT interval estimation, per-finger position interpolation,
//! and click latching. This module provides that shared logic.
//!
//! **Interpolation strategy**: BT Classic delivers reports in bursts (~50-90 Hz
//! with jitter). Between bursts the same position would repeat, then jump — causing
//! the host to compute velocity spikes. We linearly interpolate finger X/Y positions
//! between BT reports so each USB tick (4 ms) sees a proportional position change.
//!
//! **Interval estimate**: An EMA (75 % old, 25 % new) over the measured ticks
//! between successive BT reports smooths burst noise. After one estimated interval
//! of interpolation, positions freeze until the next BT report arrives.
//!
//! **Click latching**: A click that spans only a few BT reports can arrive and
//! release within a single 4 ms tick. The click bit is latched (sticky set,
//! cleared after `tick()` reads it) to guarantee at least one delivery.

/// Maximum fingers tracked for interpolation.
pub const MAX_FINGERS: usize = 5;

/// Fixed-point fractional bits (20.12 format).
pub const FRAC_BITS: i32 = 12;

/// Default estimated BT interval in ticks (12 ms / 4 ms = 3 ticks ≈ 83 Hz).
const DEFAULT_BT_INTERVAL: u16 = 3;
/// Minimum BT interval (4 ms = 1 tick).
const MIN_BT_INTERVAL: u16 = 1;
/// Maximum BT interval (100 ms = 25 ticks).
const MAX_BT_INTERVAL: u16 = 25;

/// Ticks of no BT input before deactivating (300 ms at 4 ms/tick).
const IDLE_TIMEOUT_TICKS: u16 = 75;

// --- Touch-point decode helpers ---

/// Decode 13-bit signed X from a 9-byte MT2 touch point.
/// X spans byte[0] (low 8 bits) and byte[1] bits 4-0 (high 5 bits).
pub fn decode_x(t: &[u8]) -> i16 {
    let raw = (t[0] as u16) | (((t[1] & 0x1F) as u16) << 8);
    ((raw << 3) as i16) >> 3
}

/// Decode 13-bit signed Y from a 9-byte MT2 touch point.
/// Y is stored negated across byte[1] bits 7-5, byte[2], and byte[3] bits 1-0.
pub fn decode_y(t: &[u8]) -> i16 {
    let neg_y_raw = ((t[1] >> 5) as u16) | ((t[2] as u16) << 3) | (((t[3] & 0x03) as u16) << 11);
    -(((neg_y_raw << 3) as i16) >> 3)
}

/// Extract finger ID from a 9-byte touch point (byte[8] bits 3-0).
pub fn finger_id(t: &[u8]) -> u8 {
    t[8] & 0x0F
}

/// Extract touch state from byte[3] bits 6-7.
/// 0x00 = none/approach, 0x40 = near, 0x80 = contact, 0xC0 = release.
pub fn touch_state(t: &[u8]) -> u8 {
    t[3] & 0xC0
}

/// Convert an i16 MT2 coordinate to 20.12 fixed-point.
pub fn to_fixed(v: i16) -> i32 {
    (v as i32) << FRAC_BITS
}

/// Convert a 20.12 fixed-point value back to an i16 MT2 coordinate.
pub fn from_fixed(v: i32) -> i16 {
    (v >> FRAC_BITS) as i16
}

// --- Output types ---

/// Diagnostic information returned by `InterpolationCore::receive_bt_report`.
/// Callers use this to emit format-specific log messages.
pub struct ReceiveInfo {
    /// Number of touch points in this report.
    pub n_points: usize,
    /// Ticks elapsed since the previous BT report (before this call).
    pub dt: u16,
    /// Smoothed BT interval estimate after this update.
    pub interval: u16,
    /// Whether linear interpolation was set up (false = position snapped).
    pub can_interp: bool,
    /// Running count of received BT reports (for log-rate gating).
    pub count: u32,
}

/// Per-finger data returned by `InterpolationCore::tick`.
/// Coordinates are in 20.12 fixed-point MT2 native units.
/// Callers call `from_fixed()` and convert to their target format.
pub struct TickOutput {
    pub n_fingers: u8,
    pub ids: [u8; MAX_FINGERS],
    /// MT2 touch states (0x80 contact, 0x40 near, 0xC0 release, 0x00 none).
    pub states: [u8; MAX_FINGERS],
    /// Interpolated X positions (20.12 fixed-point, MT2 coordinates).
    pub x: [i32; MAX_FINGERS],
    /// Interpolated Y positions (20.12 fixed-point, MT2 coordinates).
    pub y: [i32; MAX_FINGERS],
    /// Resolved click/button state for this tick.
    pub button: bool,
}

// --- Core struct ---

/// Shared interpolation state for BT Classic → USB reclocked passthrough.
///
/// Manages BT interval estimation, per-finger linear interpolation, click
/// latching, and idle deactivation. Output coordinates are in MT2 native
/// units (20.12 fixed-point); callers convert to their target format.
pub struct InterpolationCore {
    n_fingers: u8,
    finger_ids: [u8; MAX_FINGERS],
    finger_states: [u8; MAX_FINGERS],
    cur_x: [i32; MAX_FINGERS],
    cur_y: [i32; MAX_FINGERS],
    dx: [i32; MAX_FINGERS],
    dy: [i32; MAX_FINGERS],

    ticks_since_bt: u16,
    bt_interval: u16,
    interp_remaining: u16,
    has_prev: bool,

    active: bool,
    idle_ticks: u16,
    click_latched: bool,
    last_bt_click: bool,
    diag_bt_count: u32,
}

impl Default for InterpolationCore {
    fn default() -> Self {
        Self::new()
    }
}

impl InterpolationCore {
    pub const fn new() -> Self {
        Self {
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
            active: false,
            idle_ticks: 0,
            click_latched: false,
            last_bt_click: false,
            diag_bt_count: 0,
        }
    }

    /// Process an incoming BT Classic MT2 report and set up interpolation.
    ///
    /// `bt_data` must be a BT HIDP report: `[0x31][hdr 3 bytes][touch×N]`.
    /// Returns `None` if the report is invalid (wrong ID or malformed length),
    /// or `Some(ReceiveInfo)` with diagnostic data the caller may log.
    pub fn receive_bt_report(&mut self, bt_data: &[u8]) -> Option<ReceiveInfo> {
        if bt_data.len() < 4 || bt_data[0] != 0x31 {
            return None;
        }
        let n_touch_bytes = bt_data.len() - 4;
        if !n_touch_bytes.is_multiple_of(9) {
            return None;
        }
        let n_points = (n_touch_bytes / 9).min(MAX_FINGERS);

        // Track BT click state: last_bt_click sustains held clicks; click_latched
        // catches brief clicks that arrive and release before tick() runs.
        let bt_click = bt_data[1] & 0x01 != 0;
        self.last_bt_click = bt_click;
        if bt_click {
            self.click_latched = true;
        }

        // Update BT interval estimate (EMA: 75 % old + 25 % new).
        let dt = self.ticks_since_bt;
        if dt > 0 && self.has_prev {
            let measured = dt.clamp(MIN_BT_INTERVAL, MAX_BT_INTERVAL);
            self.bt_interval = ((3 * self.bt_interval as u32 + measured as u32) / 4).max(1) as u16;
        }

        // Check if we can interpolate (same finger count + matching IDs).
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
                // Interpolate from wherever ticks left cur_x/y toward new target.
                self.dx[i] = (target_x - self.cur_x[i]) / interval;
                self.dy[i] = (target_y - self.cur_y[i]) / interval;
            } else {
                // Snap: no previous data or finger layout changed.
                self.cur_x[i] = target_x;
                self.cur_y[i] = target_y;
                self.dx[i] = 0;
                self.dy[i] = 0;
            }
        }

        self.diag_bt_count += 1;
        self.n_fingers = n;
        self.has_prev = true;
        self.interp_remaining = self.bt_interval;
        self.ticks_since_bt = 0;
        self.idle_ticks = 0;
        self.active = true;

        Some(ReceiveInfo {
            n_points,
            dt,
            interval: self.bt_interval,
            can_interp,
            count: self.diag_bt_count,
        })
    }

    /// Advance one 4 ms tick.
    ///
    /// Returns per-finger positions (20.12 fixed-point MT2 coordinates) and
    /// click state if the touchpad is active, or `None` once idle times out.
    pub fn tick(&mut self) -> Option<TickOutput> {
        if !self.active {
            return None;
        }

        self.idle_ticks = self.idle_ticks.saturating_add(1);
        self.ticks_since_bt = self.ticks_since_bt.saturating_add(1);

        // Deactivate after idle timeout with no active fingers.
        if self.idle_ticks > IDLE_TIMEOUT_TICKS && !self.has_active_fingers() {
            self.active = false;
            self.has_prev = false;
            self.diag_bt_count = 0;
            return None;
        }

        // Advance positions during the interpolation window; hold after completion.
        let advancing = self.interp_remaining > 0;
        if advancing {
            self.interp_remaining -= 1;
        }

        let mut out_x = [0i32; MAX_FINGERS];
        let mut out_y = [0i32; MAX_FINGERS];
        let n = self.n_fingers as usize;
        for i in 0..n.min(MAX_FINGERS) {
            if advancing {
                self.cur_x[i] += self.dx[i];
                self.cur_y[i] += self.dy[i];
            }
            out_x[i] = self.cur_x[i];
            out_y[i] = self.cur_y[i];
        }

        // Resolve click: latch catches brief clicks; last_bt_click sustains held ones.
        let button = self.click_latched || self.last_bt_click;
        self.click_latched = false;

        Some(TickOutput {
            n_fingers: self.n_fingers,
            ids: self.finger_ids,
            states: self.finger_states,
            x: out_x,
            y: out_y,
            button,
        })
    }

    fn has_active_fingers(&self) -> bool {
        for i in 0..self.n_fingers as usize {
            let s = self.finger_states[i];
            if s == 0x80 || s == 0x40 {
                return true;
            }
        }
        false
    }
}
