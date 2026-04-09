//! ACL fragment reassembly into L2CAP SDUs.
//!
//! Classic Bluetooth ACL packets can be fragmented across multiple HCI ACL data
//! packets. The first fragment has PB flag = 0b10 (first non-flushable) or 0b00
//! (first auto-flushable). Continuation fragments have PB flag = 0b01.
//!
//! L2CAP frame format (in the first fragment):
//!   [length(2), channel_id(2), payload...]
//!
//! We reassemble fragments into complete L2CAP frames in a fixed-size buffer.

/// ACL Packet Boundary flags (bits 12-13 of the handle field).
pub mod pb_flag {
    /// First automatically-flushable packet of a higher-layer message.
    pub const FIRST_FLUSHABLE: u8 = 0b00;
    /// Continuing fragment.
    pub const CONTINUATION: u8 = 0b01;
    /// First non-automatically-flushable packet.
    pub const FIRST_NON_FLUSHABLE: u8 = 0b10;
}

/// ACL packet reassembly buffer.
pub struct ReassemblyBuffer<const N: usize> {
    buf: [u8; N],
    /// Number of bytes currently in the buffer.
    len: usize,
    /// Expected total L2CAP frame length (from the L2CAP header).
    expected_len: usize,
    /// Whether we're currently reassembling a fragmented packet.
    in_progress: bool,
}

impl<const N: usize> Default for ReassemblyBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ReassemblyBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; N],
            len: 0,
            expected_len: 0,
            in_progress: false,
        }
    }

    /// Process an ACL data fragment.
    ///
    /// `handle_and_flags` is the raw 16-bit field from the ACL header (handle + PB + BC flags).
    /// `data` is the ACL payload (after the 4-byte ACL header).
    ///
    /// Returns `Some((channel_id, payload))` when a complete L2CAP frame is assembled.
    /// Returns `None` if more fragments are needed.
    pub fn process_fragment<'a>(
        &'a mut self,
        handle_and_flags: u16,
        data: &[u8],
    ) -> Option<(u16, &'a [u8])> {
        let pb = ((handle_and_flags >> 12) & 0x03) as u8;

        match pb {
            pb_flag::FIRST_FLUSHABLE | pb_flag::FIRST_NON_FLUSHABLE => {
                // Start of a new L2CAP frame.
                // First 4 bytes are L2CAP header: [length(2), channel_id(2)]
                if data.len() < 4 {
                    self.reset();
                    return None;
                }

                let l2cap_len = u16::from_le_bytes([data[0], data[1]]) as usize;
                let _channel_id = u16::from_le_bytes([data[2], data[3]]);
                let total_with_header = l2cap_len + 4; // L2CAP payload + 4-byte L2CAP header

                // Copy into buffer
                let copy_len = data.len().min(N);
                self.buf[..copy_len].copy_from_slice(&data[..copy_len]);
                self.len = copy_len;
                self.expected_len = total_with_header;

                if self.len >= self.expected_len {
                    // Complete in a single fragment
                    self.in_progress = false;
                    let channel_id = u16::from_le_bytes([self.buf[2], self.buf[3]]);
                    Some((channel_id, &self.buf[4..self.expected_len]))
                } else {
                    // Need more fragments
                    self.in_progress = true;
                    None
                }
            }

            pb_flag::CONTINUATION => {
                if !self.in_progress {
                    // Stale continuation fragment — drop
                    return None;
                }

                // Append to buffer
                let space = N - self.len;
                let copy_len = data.len().min(space);
                self.buf[self.len..self.len + copy_len].copy_from_slice(&data[..copy_len]);
                self.len += copy_len;

                if self.len >= self.expected_len {
                    // Complete
                    self.in_progress = false;
                    let channel_id = u16::from_le_bytes([self.buf[2], self.buf[3]]);
                    Some((channel_id, &self.buf[4..self.expected_len]))
                } else {
                    None
                }
            }

            _ => {
                // Unknown PB flag — reset
                self.reset();
                None
            }
        }
    }

    /// Reset the reassembly buffer.
    pub fn reset(&mut self) {
        self.len = 0;
        self.expected_len = 0;
        self.in_progress = false;
    }
}
