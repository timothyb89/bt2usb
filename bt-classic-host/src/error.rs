//! Error types for the Classic Bluetooth host stack.

/// Errors that can occur in the Classic Bluetooth host stack.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// HCI controller error.
    Controller(E),
    /// Connection failed or was rejected.
    ConnectionFailed,
    /// Authentication/pairing failed.
    AuthenticationFailed,
    /// Encryption setup failed.
    EncryptionFailed,
    /// L2CAP channel open was rejected.
    L2capRejected,
    /// L2CAP configuration failed.
    L2capConfigFailed,
    /// HIDP handshake indicated an error.
    HidpError(u8),
    /// No free connection slots available.
    NoFreeSlots,
    /// No free L2CAP channel IDs available.
    NoFreeCids,
    /// Operation timed out.
    Timeout,
    /// Connection was disconnected.
    Disconnected,
    /// Invalid state for this operation.
    InvalidState,
}
