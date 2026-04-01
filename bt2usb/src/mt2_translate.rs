//! Magic Trackpad 2: BT Classic → USB reclocked passthrough with interpolation.
//!
//! Buffers incoming BT touch reports and retransmits at a steady USB rate
//! (250 Hz / 4ms tick). Touch point encoding is byte-identical between BT
//! and USB — only the report headers differ.
//!
//! **Interpolation**: BT Classic delivers reports in bursts (~50-90Hz with
//! jitter). Without interpolation, the same position repeats at 250Hz between
//! bursts, then jumps — causing macOS to compute velocity spikes. This module
//! linearly interpolates finger X/Y positions between BT reports so each USB
//! tick shows a proportional position change, giving macOS correct velocity.
//!
//! **Click latching**: A click that spans only a few BT reports can arrive
//! and release within a single tick interval. The click bit is latched (sticky)
//! to guarantee delivery.

/// Maximum USB report size: 12-byte header + 5 fingers × 9 bytes.
const MAX_USB_REPORT: usize = 57;

/// Maximum fingers we track for interpolation.
const MAX_FINGERS: usize = 5;

/// USB timestamp increment per tick (~8kHz / 250Hz ≈ 32).
const TIMESTAMP_INCREMENT: u16 = 32;

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

// --- Touch point decode/encode helpers ---

/// Decode 13-bit signed X from a 9-byte touch point.
/// X is stored in byte[0] (low 8 bits) and byte[1] bits 4-0 (high 5 bits).
fn decode_x(t: &[u8]) -> i16 {
    let raw = (t[0] as u16) | (((t[1] & 0x1F) as u16) << 8);
    // Sign-extend from 13 bits
    ((raw << 3) as i16) >> 3
}

/// Decode 13-bit signed Y from a 9-byte touch point.
/// Y is stored (negated) across byte[1] bits 7-5, byte[2], byte[3] bits 1-0.
/// (Byte[3] bits 7-6 are the state field — must not be confused with Y.)
fn decode_y(t: &[u8]) -> i16 {
    let neg_y_raw = ((t[1] >> 5) as u16)
        | ((t[2] as u16) << 3)
        | (((t[3] & 0x03) as u16) << 11); // bits 0-1 of byte 3 → Y bits 11-12
    // Sign-extend from 13 bits and negate
    -(((neg_y_raw << 3) as i16) >> 3)
}

/// Encode 13-bit signed X into a 9-byte touch point.
/// Preserves byte[1] bits 7-5 (Y's low bits).
fn encode_x(t: &mut [u8], x: i16) {
    let raw = (x as u16) & 0x1FFF;
    t[0] = (raw & 0xFF) as u8;
    t[1] = (t[1] & 0xE0) | ((raw >> 8) & 0x1F) as u8;
}

/// Encode 13-bit signed Y into a 9-byte touch point.
/// Preserves byte[1] bits 4-0 (X's high bits) and byte[3] bits 7-2 (state etc).
fn encode_y(t: &mut [u8], y: i16) {
    let neg_y_raw = ((-y) as u16) & 0x1FFF;
    t[1] = (t[1] & 0x1F) | (((neg_y_raw & 0x07) << 5) as u8);
    t[2] = ((neg_y_raw >> 3) & 0xFF) as u8;
    t[3] = (t[3] & 0xFC) | ((neg_y_raw >> 11) & 0x03) as u8;
}

/// Extract finger ID from a 9-byte touch point (byte[8] bits 3-0).
fn finger_id(t: &[u8]) -> u8 {
    t[8] & 0x0F
}

/// Convert i16 coordinate to 20.12 fixed-point.
fn to_fixed(v: i16) -> i32 {
    (v as i32) << FRAC_BITS
}

/// Convert 20.12 fixed-point back to i16 coordinate.
fn from_fixed(v: i32) -> i16 {
    (v >> FRAC_BITS) as i16
}

// --- Main passthrough struct ---

/// Reclocked passthrough with linear interpolation between BT reports.
pub struct Mt2Passthrough {
    /// USB report (header + touch data). Touch bytes are patched in tick()
    /// with interpolated X/Y before sending.
    report: [u8; MAX_USB_REPORT],
    /// Actual length of the report.
    report_len: usize,

    /// Monotonic 16-bit USB timestamp, incremented by 32 per tick.
    timestamp: u16,
    /// Whether we're actively sending reports.
    active: bool,
    /// Ticks since last BT report received.
    idle_ticks: u16,
    /// Latched click state (sticky set, cleared after tick sends it).
    click_latched: bool,

    // --- Interpolation state ---
    /// Number of fingers in the current report.
    n_fingers: u8,
    /// Finger IDs from the latest BT report.
    finger_ids: [u8; MAX_FINGERS],
    /// Current interpolated X positions (20.12 fixed-point).
    cur_x: [i32; MAX_FINGERS],
    /// Current interpolated Y positions (20.12 fixed-point).
    cur_y: [i32; MAX_FINGERS],
    /// X delta per tick (20.12 fixed-point).
    dx: [i32; MAX_FINGERS],
    /// Y delta per tick (20.12 fixed-point).
    dy: [i32; MAX_FINGERS],

    // --- BT interval tracking ---
    /// Ticks elapsed since last BT report (incremented in tick()).
    ticks_since_bt: u16,
    /// Smoothed estimate of BT report interval in ticks (EMA).
    bt_interval: u16,
    /// Ticks remaining in current interpolation. When zero, positions hold.
    interp_remaining: u16,
    /// Whether we have a previous report's positions to interpolate from.
    has_prev: bool,
    /// Diagnostic: BT report counter (for gated logging).
    diag_bt_count: u32,
}

impl Mt2Passthrough {
    pub const fn new() -> Self {
        Self {
            report: [0u8; MAX_USB_REPORT],
            report_len: 0,
            timestamp: 10000,
            active: false,
            idle_ticks: 0,
            click_latched: false,
            n_fingers: 0,
            finger_ids: [0; MAX_FINGERS],
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

    /// Translate a BT Classic MT2 report into USB format and set up interpolation.
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
        let usb_touch_bytes = n_points * 9;
        let usb_len = 12 + usb_touch_bytes;

        // Latch click bit (sticky — cleared only after tick sends it)
        if bt_data[1] & 0x01 != 0 {
            self.click_latched = true;
        }

        // Build USB header (12 bytes)
        let r = &mut self.report;
        r[0] = 0x02;
        r[1] = 0x00; // Click patched in tick()
        r[2] = 0x00;
        r[3] = 0x00;
        r[4] = 0x00;
        r[5] = 0x00;
        r[6] = 0x00;
        r[7] = if n_points > 0 { 0x03 } else { 0x00 };
        r[8] = 0x31;
        r[9] = 0x00; // Timestamp patched in tick()
        r[10] = 0x00;
        r[11] = 0xe6;

        // Copy touch data (encoding identical BT↔USB)
        r[12..12 + usb_touch_bytes].copy_from_slice(&bt_data[4..4 + usb_touch_bytes]);
        self.report_len = usb_len;

        // --- Set up interpolation ---

        // Update BT interval estimate (EMA: 75% old + 25% new).
        // Smoothing prevents wild swings from variable dt values.
        if self.ticks_since_bt > 0 && self.has_prev {
            let measured = self.ticks_since_bt.clamp(MIN_BT_INTERVAL, MAX_BT_INTERVAL);
            self.bt_interval =
                ((3 * self.bt_interval as u32 + measured as u32) / 4).max(1) as u16;
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

            if can_interp {
                // Interpolate from current position (as advanced by ticks) to new target.
                // No fast-forward: cur_x/y is wherever the ticks left it.
                self.dx[i] = (target_x - self.cur_x[i]) / interval;
                self.dy[i] = (target_y - self.cur_y[i]) / interval;
            } else {
                // Snap: no interpolation (first report or finger change)
                self.cur_x[i] = target_x;
                self.cur_y[i] = target_y;
                self.dx[i] = 0;
                self.dy[i] = 0;
            }
        }

        // Diagnostic: log BT arrival timing (first 50 per session, then every 100th)
        self.diag_bt_count += 1;
        if self.diag_bt_count <= 50 || self.diag_bt_count.is_multiple_of(100) {
            let x0 = if n_points > 0 { decode_x(&bt_data[4..13]) } else { 0 };
            let y0 = if n_points > 0 { decode_y(&bt_data[4..13]) } else { 0 };
            defmt::debug!(
                "[mt2] BT #{} dt={} est={} n={} x={} y={} interp={}",
                self.diag_bt_count,
                self.ticks_since_bt,
                self.bt_interval,
                n_points,
                x0,
                y0,
                can_interp,
            );
        }

        self.n_fingers = n;
        self.has_prev = true;
        self.interp_remaining = self.bt_interval;
        self.ticks_since_bt = 0;
        self.idle_ticks = 0;
        self.active = true;
    }

    /// Called every 4ms. Returns a USB report to send, or None if idle.
    pub fn tick(&mut self) -> Option<([u8; MAX_USB_REPORT], usize)> {
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

        // Advance positions during interpolation, hold after completion.
        // interp_remaining prevents overshoot: after the estimated interval,
        // positions hold until the next BT report provides a new target.
        let advancing = self.interp_remaining > 0;
        if advancing {
            self.interp_remaining -= 1;
        }

        let n = self.n_fingers as usize;
        for i in 0..n.min(MAX_FINGERS) {
            if advancing {
                self.cur_x[i] += self.dx[i];
                self.cur_y[i] += self.dy[i];
            }

            // Always encode current position into the report
            let off = 12 + i * 9;
            if off + 9 <= self.report_len {
                let t = &mut self.report[off..off + 9];
                encode_x(t, from_fixed(self.cur_x[i]));
                encode_y(t, from_fixed(self.cur_y[i]));
            }
        }

        // Patch click bit from latch
        self.report[1] = if self.click_latched { 0x01 } else { 0x00 };
        self.click_latched = false;

        // Patch timestamp
        self.report[9] = (self.timestamp & 0xFF) as u8;
        self.report[10] = (self.timestamp >> 8) as u8;
        self.timestamp = self.timestamp.wrapping_add(TIMESTAMP_INCREMENT);

        Some((self.report, self.report_len))
    }

    /// Check if any finger in the stored report has a non-NONE state.
    fn has_active_fingers(&self) -> bool {
        let n = self.n_fingers as usize;
        for i in 0..n.min(MAX_FINGERS) {
            let off = 12 + i * 9;
            if off + 9 <= self.report_len {
                let state = self.report[off + 3] & 0xC0;
                if state != 0x00 {
                    return true;
                }
            }
        }
        false
    }
}
