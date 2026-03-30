//! HIDP (HID Profile) protocol implementation.
//!
//! HIDP runs over two L2CAP channels:
//! - Control (PSM 0x0011): Commands, GET/SET_REPORT, handshakes
//! - Interrupt (PSM 0x0013): Unsolicited input reports (HID data)
//!
//! Message format: header byte (upper nibble = type, lower = param) + payload.

pub mod client;
pub mod types;

// TODO: Phase 4 implementation
