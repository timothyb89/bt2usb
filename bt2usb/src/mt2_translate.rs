//! Magic Trackpad 2: BT Classic touch data interpretation.
//!
//! Phase 1 (interpreted mode): Extracts scroll deltas from multi-finger
//! touch data and feeds them to the existing TouchSynthesizer pipeline.
//!
//! Phase 2 (future, reclocked passthrough): Will buffer BT touch reports
//! and retransmit at steady USB rate for full gesture passthrough.

/// Stateful interpreter that tracks finger positions across reports
/// to compute scroll deltas from MT2 multi-finger touch data.
pub struct Mt2Translator {
    /// Previous average Y for scroll delta computation.
    scroll_prev_y: Option<i16>,
}

impl Mt2Translator {
    pub const fn new() -> Self {
        Self {
            scroll_prev_y: None,
        }
    }

    /// Extract X/Y from a 9-byte touch point using the kernel formula.
    fn decode_xy(tdata: &[u8]) -> (i16, i16) {
        // x = (tdata[1] << 27 | tdata[0] << 19) >> 19  (13-bit signed)
        let raw_x = ((tdata[1] as u32) << 27) | ((tdata[0] as u32) << 19);
        let x = (raw_x as i32 >> 19) as i16;

        // y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19)
        let raw_y =
            ((tdata[3] as u32) << 30) | ((tdata[2] as u32) << 22) | ((tdata[1] as u32) << 14);
        let y = (-(raw_y as i32 >> 19)) as i16;

        (x, y)
    }

    /// Extract a scroll delta from a BT Classic MT2 report.
    ///
    /// For 2+ finger touch in contact state, computes the average Y delta
    /// across all fingers. Returns the delta as an i16 suitable for feeding
    /// to the existing TouchSynthesizer scroll pipeline.
    ///
    /// Returns `Some(dy)` when scrolling is detected, `None` otherwise.
    pub fn extract_scroll(&mut self, bt_data: &[u8]) -> Option<i16> {
        if bt_data.len() < 4 || bt_data[0] != 0x31 {
            return None;
        }

        let n_touch_bytes = bt_data.len() - 4;
        if !n_touch_bytes.is_multiple_of(9) {
            return None;
        }

        let n_fingers = n_touch_bytes / 9;

        // Need 2+ fingers for scroll
        if n_fingers < 2 {
            self.scroll_prev_y = None;
            return None;
        }

        // Compute average Y of all fingers in contact state (0x80)
        let touch_data = &bt_data[4..];
        let mut y_sum: i32 = 0;
        let mut contact_count: u8 = 0;

        for i in 0..n_fingers {
            let tdata = &touch_data[i * 9..(i + 1) * 9];
            let state = tdata[3] & 0xC0;
            if state == 0x80 {
                let (_, y) = Self::decode_xy(tdata);
                y_sum += y as i32;
                contact_count += 1;
            }
        }

        if contact_count < 2 {
            // Not enough fingers in contact yet (still approaching)
            return None;
        }

        let avg_y = (y_sum / contact_count as i32) as i16;

        let delta = if let Some(prev) = self.scroll_prev_y {
            (avg_y as i32 - prev as i32) as i16
        } else {
            0
        };

        self.scroll_prev_y = Some(avg_y);

        if delta != 0 { Some(delta) } else { None }
    }
}
