//! L2CAP signaling commands (CID 0x0001).
//!
//! Classic L2CAP uses a signaling channel for connection management:
//!   Connection Request (0x02) / Response (0x03)
//!   Configuration Request (0x04) / Response (0x05)
//!   Disconnection Request (0x06) / Response (0x07)
//!
//! Each command has a 4-byte header: [code, identifier, length_lo, length_hi]

/// L2CAP signaling CID (fixed channel).
pub const SIGNALING_CID: u16 = 0x0001;

/// Signaling command codes.
pub mod code {
    pub const CONNECTION_REQUEST: u8 = 0x02;
    pub const CONNECTION_RESPONSE: u8 = 0x03;
    pub const CONFIGURATION_REQUEST: u8 = 0x04;
    pub const CONFIGURATION_RESPONSE: u8 = 0x05;
    pub const DISCONNECTION_REQUEST: u8 = 0x06;
    pub const DISCONNECTION_RESPONSE: u8 = 0x07;
    pub const INFORMATION_REQUEST: u8 = 0x0A;
    pub const INFORMATION_RESPONSE: u8 = 0x0B;
}

/// Connection Response result codes.
pub mod conn_result {
    pub const SUCCESS: u16 = 0x0000;
    pub const PENDING: u16 = 0x0001;
    pub const REFUSED_PSM: u16 = 0x0002;
    pub const REFUSED_SECURITY: u16 = 0x0003;
    pub const REFUSED_RESOURCES: u16 = 0x0004;
}

/// Configuration Response result codes.
pub mod config_result {
    pub const SUCCESS: u16 = 0x0000;
    pub const UNACCEPTABLE: u16 = 0x0001;
    pub const REJECTED: u16 = 0x0002;
    pub const UNKNOWN_OPTIONS: u16 = 0x0003;
}

/// Configuration option types (TLV).
pub mod config_option {
    pub const MTU: u8 = 0x01;
    pub const FLUSH_TIMEOUT: u8 = 0x02;
}

/// Build a Connection Request signaling command.
///
/// Format: [code=0x02, id, len_lo, len_hi, psm_lo, psm_hi, scid_lo, scid_hi]
pub fn build_connection_request(id: u8, psm: u16, source_cid: u16) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = code::CONNECTION_REQUEST;
    buf[1] = id;
    buf[2] = 4; // payload length
    buf[3] = 0;
    buf[4..6].copy_from_slice(&psm.to_le_bytes());
    buf[6..8].copy_from_slice(&source_cid.to_le_bytes());
    buf
}

/// Build a Configuration Request signaling command with MTU option.
///
/// Format: [code=0x04, id, len, 0, dcid, 0, flags, 0, mtu_type, mtu_len, mtu_lo, mtu_hi]
pub fn build_configuration_request(id: u8, dest_cid: u16, mtu: u16) -> [u8; 12] {
    let mut buf = [0u8; 12];
    buf[0] = code::CONFIGURATION_REQUEST;
    buf[1] = id;
    buf[2] = 8; // payload length (4 base + 4 MTU option)
    buf[3] = 0;
    buf[4..6].copy_from_slice(&dest_cid.to_le_bytes());
    buf[6..8].copy_from_slice(&0u16.to_le_bytes()); // flags = 0 (no continuation)
    // MTU option TLV
    buf[8] = config_option::MTU;
    buf[9] = 2; // length
    buf[10..12].copy_from_slice(&mtu.to_le_bytes());
    buf
}

/// Build a Configuration Response signaling command.
///
/// Format: [code=0x05, id, len, 0, scid, 0, flags, 0, result, 0]
pub fn build_configuration_response(id: u8, source_cid: u16, result: u16) -> [u8; 10] {
    let mut buf = [0u8; 10];
    buf[0] = code::CONFIGURATION_RESPONSE;
    buf[1] = id;
    buf[2] = 6; // payload length
    buf[3] = 0;
    buf[4..6].copy_from_slice(&source_cid.to_le_bytes());
    buf[6..8].copy_from_slice(&0u16.to_le_bytes()); // flags
    buf[8..10].copy_from_slice(&result.to_le_bytes());
    buf
}

/// Build a Disconnection Request signaling command.
///
/// Format: [code=0x06, id, len=4, 0, dcid, scid]
pub fn build_disconnection_request(id: u8, dest_cid: u16, source_cid: u16) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = code::DISCONNECTION_REQUEST;
    buf[1] = id;
    buf[2] = 4;
    buf[3] = 0;
    buf[4..6].copy_from_slice(&dest_cid.to_le_bytes());
    buf[6..8].copy_from_slice(&source_cid.to_le_bytes());
    buf
}

/// Build a Disconnection Response signaling command.
///
/// Format: [code=0x07, id, len=4, 0, dcid, scid]
pub fn build_disconnection_response(id: u8, dest_cid: u16, source_cid: u16) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = code::DISCONNECTION_RESPONSE;
    buf[1] = id;
    buf[2] = 4;
    buf[3] = 0;
    buf[4..6].copy_from_slice(&dest_cid.to_le_bytes());
    buf[6..8].copy_from_slice(&source_cid.to_le_bytes());
    buf
}

/// Build an Information Response (Extended Features) signaling command.
///
/// This is sent in response to Information Request for Extended Features Mask.
/// We report no extended features (basic L2CAP mode only).
pub fn build_info_response_extended_features(id: u8) -> [u8; 12] {
    let mut buf = [0u8; 12];
    buf[0] = code::INFORMATION_RESPONSE;
    buf[1] = id;
    buf[2] = 8; // length
    buf[3] = 0;
    buf[4..6].copy_from_slice(&0x0002u16.to_le_bytes()); // info_type = Extended Features
    buf[6..8].copy_from_slice(&0x0000u16.to_le_bytes()); // result = success
    buf[8..12].copy_from_slice(&0x00000000u32.to_le_bytes()); // no extended features
    buf
}

/// Parse a signaling command header.
///
/// Returns (code, identifier, payload_data) or None if too short.
pub fn parse_signal_header(data: &[u8]) -> Option<(u8, u8, &[u8])> {
    if data.len() < 4 {
        return None;
    }
    let code = data[0];
    let id = data[1];
    let length = u16::from_le_bytes([data[2], data[3]]) as usize;
    let payload_end = (4 + length).min(data.len());
    Some((code, id, &data[4..payload_end]))
}

/// Parse a Connection Response payload.
///
/// Returns (dest_cid, source_cid, result, status).
pub fn parse_connection_response(payload: &[u8]) -> Option<(u16, u16, u16, u16)> {
    if payload.len() < 8 {
        return None;
    }
    let dcid = u16::from_le_bytes([payload[0], payload[1]]);
    let scid = u16::from_le_bytes([payload[2], payload[3]]);
    let result = u16::from_le_bytes([payload[4], payload[5]]);
    let status = u16::from_le_bytes([payload[6], payload[7]]);
    Some((dcid, scid, result, status))
}

/// Parse a Configuration Response payload.
///
/// Returns (source_cid, flags, result).
pub fn parse_configuration_response(payload: &[u8]) -> Option<(u16, u16, u16)> {
    if payload.len() < 6 {
        return None;
    }
    let scid = u16::from_le_bytes([payload[0], payload[1]]);
    let flags = u16::from_le_bytes([payload[2], payload[3]]);
    let result = u16::from_le_bytes([payload[4], payload[5]]);
    Some((scid, flags, result))
}

/// Parse a Configuration Request payload.
///
/// Returns (dest_cid, flags, options_slice).
pub fn parse_configuration_request(payload: &[u8]) -> Option<(u16, u16, &[u8])> {
    if payload.len() < 4 {
        return None;
    }
    let dcid = u16::from_le_bytes([payload[0], payload[1]]);
    let flags = u16::from_le_bytes([payload[2], payload[3]]);
    Some((dcid, flags, &payload[4..]))
}

/// Extract MTU from configuration options TLV data.
pub fn extract_mtu_from_options(options: &[u8]) -> Option<u16> {
    let mut offset = 0;
    while offset + 2 <= options.len() {
        let opt_type = options[offset];
        let opt_len = options[offset + 1] as usize;
        if offset + 2 + opt_len > options.len() {
            break;
        }
        if opt_type == config_option::MTU && opt_len == 2 {
            return Some(u16::from_le_bytes([
                options[offset + 2],
                options[offset + 3],
            ]));
        }
        offset += 2 + opt_len;
    }
    None
}
