//! HIDP message types and constants.

/// HIDP message type (upper nibble of header byte).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum MessageType {
    /// Handshake response (device → host on control channel).
    Handshake = 0x00,
    /// HID Control command (suspend, reset, unplug).
    HidControl = 0x10,
    /// Get Report request.
    GetReport = 0x40,
    /// Set Report request.
    SetReport = 0x50,
    /// Get Protocol request.
    GetProtocol = 0x60,
    /// Set Protocol request.
    SetProtocol = 0x70,
    /// Get Idle request.
    GetIdle = 0x80,
    /// Set Idle request.
    SetIdle = 0x90,
    /// Data message (input/output reports on interrupt channel).
    Data = 0xA0,
    /// Data continuation (large report spanning multiple messages).
    DataContinuation = 0xB0,
}

/// HIDP report type (lower nibble parameter for Get/Set Report and Data).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ReportType {
    Other = 0x00,
    Input = 0x01,
    Output = 0x02,
    Feature = 0x03,
}

/// HIDP handshake result codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum HandshakeResult {
    Successful = 0x00,
    NotReady = 0x01,
    InvalidReportId = 0x02,
    UnsupportedRequest = 0x03,
    InvalidParameter = 0x04,
    UnknownError = 0x0E,
    FatalError = 0x0F,
}

/// L2CAP PSM for HID Control channel.
pub const PSM_HID_CONTROL: u16 = 0x0011;
/// L2CAP PSM for HID Interrupt channel.
pub const PSM_HID_INTERRUPT: u16 = 0x0013;

/// Parse the HIDP header byte into message type and parameter.
pub fn parse_header(header: u8) -> (u8, u8) {
    let msg_type = header & 0xF0;
    let param = header & 0x0F;
    (msg_type, param)
}

/// Build a HIDP header byte from message type and parameter.
pub fn build_header(msg_type: u8, param: u8) -> u8 {
    (msg_type & 0xF0) | (param & 0x0F)
}
