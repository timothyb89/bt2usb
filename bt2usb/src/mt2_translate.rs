//! Magic Trackpad 2: BT Classic → USB reclocked passthrough.
//!
//! Buffers incoming BT touch reports and retransmits at a steady USB rate
//! (250 Hz / 4ms tick). The BT→USB touch point encoding is byte-identical;
//! only the report headers differ.
//!
//! BT format:  [0x31][clicks|ts][ts][ts] + N×9-byte touch points
//! USB format: [0x02][clicks][0×5][mode][0x31][ts_lo][ts_hi][0xe6] + N×9-byte touch points
//!
//! Click latching: BT Classic delivers reports in bursts. A click that spans
//! only a few reports can arrive and release within a single tick interval,
//! making it invisible to USB. The click bit is latched (sticky): once set
//! by any BT report, it stays set until a tick sends it to USB. This
//! guarantees every click is delivered.

/// USB timestamp increment per tick (~8kHz / 250Hz ≈ 32).
const TIMESTAMP_INCREMENT: u16 = 32;

/// Ticks of no BT input before deactivating (300ms at 4ms tick).
const IDLE_TIMEOUT_TICKS: u16 = 75;

/// Maximum USB report size: 12-byte header + 5 fingers × 9 bytes.
const MAX_USB_REPORT: usize = 57;

/// Reclocked passthrough: stores latest BT touch state and outputs it
/// at a steady USB rate with proper header translation and timestamps.
pub struct Mt2Passthrough {
    /// Pre-translated USB report (header + touch data).
    latest_usb_report: [u8; MAX_USB_REPORT],
    /// Actual length of the stored report.
    latest_len: usize,
    /// Monotonic 16-bit USB timestamp, incremented by 32 per tick.
    timestamp: u16,
    /// Whether we're actively sending reports.
    active: bool,
    /// Ticks since last BT report received.
    idle_ticks: u16,
    /// Latched click state. Set by any BT report with click=1, cleared after
    /// tick() sends it. Prevents clicks from being lost in BT bursts.
    click_latched: bool,
}

impl Mt2Passthrough {
    pub const fn new() -> Self {
        Self {
            latest_usb_report: [0u8; MAX_USB_REPORT],
            latest_len: 0,
            timestamp: 10000,
            active: false,
            idle_ticks: 0,
            click_latched: false,
        }
    }

    /// Translate a BT Classic MT2 report into USB format and store it.
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

        let n_points = (n_touch_bytes / 9).min(5);
        let usb_touch_bytes = n_points * 9;
        let usb_len = 12 + usb_touch_bytes;

        // Latch click bit: once set, stays set until tick() sends it.
        // This prevents clicks from being lost when the entire press+release
        // sequence arrives in a single BT burst (before any tick fires).
        if bt_data[1] & 0x01 != 0 {
            self.click_latched = true;
        }

        // Build USB header (12 bytes)
        // Note: byte[1] (click) is patched in tick() from the latch state,
        // not set here — avoids the latch being overwritten by a later
        // non-click report in the same burst.
        let r = &mut self.latest_usb_report;
        r[0] = 0x02; // USB report ID
        r[1] = 0x00; // Click bit patched in tick()
        r[2] = 0x00;
        r[3] = 0x00;
        r[4] = 0x00;
        r[5] = 0x00;
        r[6] = 0x00;
        r[7] = if n_points > 0 { 0x03 } else { 0x00 }; // Contact tracking flag
        r[8] = 0x31; // Fixed marker
        // Bytes 9-10: timestamp (patched on each tick)
        r[9] = 0x00;
        r[10] = 0x00;
        r[11] = 0xe6; // Fixed constant

        // Copy touch data unchanged (encoding is identical BT↔USB)
        r[12..12 + usb_touch_bytes].copy_from_slice(&bt_data[4..4 + usb_touch_bytes]);

        self.latest_len = usb_len;
        self.active = true;
        self.idle_ticks = 0;
    }

    /// Called every 4ms. Returns a USB report to send, or None if idle.
    pub fn tick(&mut self) -> Option<([u8; MAX_USB_REPORT], usize)> {
        if !self.active {
            return None;
        }

        self.idle_ticks = self.idle_ticks.saturating_add(1);

        // Deactivate after idle timeout with no active fingers
        if self.idle_ticks > IDLE_TIMEOUT_TICKS && !self.has_active_fingers() {
            self.active = false;
            return None;
        }

        // Patch click bit from latch
        self.latest_usb_report[1] = if self.click_latched { 0x01 } else { 0x00 };
        // Clear latch after sending — will be re-set by next BT report if click continues
        self.click_latched = false;

        // Patch timestamp
        self.latest_usb_report[9] = (self.timestamp & 0xFF) as u8;
        self.latest_usb_report[10] = (self.timestamp >> 8) as u8;
        self.timestamp = self.timestamp.wrapping_add(TIMESTAMP_INCREMENT);

        Some((self.latest_usb_report, self.latest_len))
    }

    /// Check if any finger in the stored report has a non-NONE state.
    /// States: 0x00=none, 0x40=near, 0x80=contact, 0xC0=release.
    fn has_active_fingers(&self) -> bool {
        let touch_bytes = self.latest_len.saturating_sub(12);
        let n_fingers = touch_bytes / 9;
        for i in 0..n_fingers {
            let state = self.latest_usb_report[12 + i * 9 + 3] & 0xC0;
            if state != 0x00 {
                return true;
            }
        }
        false
    }
}
