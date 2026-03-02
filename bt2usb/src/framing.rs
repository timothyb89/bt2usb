//! COBS framing for RPC messages
//!
//! Provides frame encoding and a streaming frame accumulator for decoding
//! COBS-framed messages over a byte stream (USB CDC ACM).
//!
//! Wire format: [COBS-encoded payload] [0x00 delimiter]

use crate::protocol::{self, HEADER_SIZE};

/// Maximum CBOR message body size.
pub const MAX_MSG_SIZE: usize = 256;

/// Maximum payload size (header + CBOR body).
pub const MAX_PAYLOAD_SIZE: usize = HEADER_SIZE + MAX_MSG_SIZE;

/// Maximum COBS-encoded frame size (payload + COBS overhead + delimiter).
pub const MAX_FRAME_SIZE: usize = cobs::max_encoding_length(MAX_PAYLOAD_SIZE) + 1;

/// Encode a complete RPC message into a COBS frame.
///
/// Takes the message type, sequence ID, and CBOR body, produces a
/// COBS-encoded frame with a 0x00 delimiter at the end.
///
/// Returns the total frame length written to `frame_buf`.
pub fn encode_frame(msg_type: u8, seq_id: u16, cbor_body: &[u8], frame_buf: &mut [u8]) -> usize {
    // Build the raw payload: [header | cbor_body]
    let payload_len = HEADER_SIZE + cbor_body.len();
    let mut payload = [0u8; MAX_PAYLOAD_SIZE];
    protocol::encode_header(&mut payload, msg_type, seq_id);
    payload[HEADER_SIZE..payload_len].copy_from_slice(cbor_body);

    // COBS encode
    let encoded_len = cobs::encode(&payload[..payload_len], frame_buf);

    // Add 0x00 delimiter
    frame_buf[encoded_len] = 0x00;
    encoded_len + 1
}

/// Streaming COBS frame accumulator.
///
/// Feeds raw bytes from a serial stream and extracts complete decoded frames.
/// Handles partial frames across multiple feed() calls.
pub struct FrameAccumulator {
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
}

impl FrameAccumulator {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME_SIZE],
            pos: 0,
        }
    }

    /// Feed received bytes and try to extract a complete decoded frame.
    ///
    /// When a complete COBS frame is found (delimited by 0x00), it is decoded
    /// and written to `out`. Returns `(bytes_consumed, Some(decoded_len))` on
    /// success. Call repeatedly until `None` is returned to process all frames
    /// in the input.
    ///
    /// Returns `(bytes_consumed, None)` when no more complete frames are
    /// available (partial data is buffered for the next call).
    pub fn feed(&mut self, data: &[u8], out: &mut [u8]) -> (usize, Option<usize>) {
        for (i, &byte) in data.iter().enumerate() {
            if byte == 0x00 {
                // Frame delimiter found
                if self.pos > 0 {
                    if let Ok(decoded_len) = cobs::decode(&self.buf[..self.pos], out) {
                        self.pos = 0;
                        return (i + 1, Some(decoded_len));
                    }
                    // Decode failed - discard and continue
                    self.pos = 0;
                }
            } else if self.pos < self.buf.len() {
                self.buf[self.pos] = byte;
                self.pos += 1;
            } else {
                // Buffer overflow - discard accumulated data
                self.pos = 0;
            }
        }
        (data.len(), None)
    }

    /// Reset the accumulator state, discarding any partial frame.
    pub fn reset(&mut self) {
        self.pos = 0;
    }
}
