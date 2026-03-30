//! Classic L2CAP (Logical Link Control and Adaptation Protocol).
//!
//! Implements connection-oriented channels for Classic Bluetooth.
//! Key differences from LE L2CAP:
//! - Full signaling handshake (Connection Request/Response + Configuration)
//! - MTU negotiation in a separate Configuration phase
//! - No credit-based flow control (basic mode)
//! - Fixed PSMs for HID: 0x0011 (Control) and 0x0013 (Interrupt)

pub mod channel;
pub mod reassembly;
pub mod signal;

// TODO: Phase 3 implementation
