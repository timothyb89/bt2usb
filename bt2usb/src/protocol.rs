//! RPC protocol message definitions
//!
//! Wire format: COBS-framed messages with a 3-byte header + CBOR body.
//!
//! Header:
//!   byte 0:    msg_type (0x01=Request, 0x02=Response, 0x03=Event)
//!   byte 1-2:  seq_id (u16 LE) - correlates responses to requests; 0 for events
//!
//! CBOR body encoding:
//!   Request:  [command_id, ...args]
//!   Response: [command_id, ...fields]
//!   Event:    [event_id, ...fields]

use minicbor::encode::write::Cursor;
use minicbor::{Decoder, Encoder};

// ============ Message type tags ============

pub const MSG_REQUEST: u8 = 0x01;
pub const MSG_RESPONSE: u8 = 0x02;
pub const MSG_EVENT: u8 = 0x03;

// ============ Header ============

pub const HEADER_SIZE: usize = 3;

pub fn encode_header(buf: &mut [u8], msg_type: u8, seq_id: u16) {
    buf[0] = msg_type;
    buf[1] = seq_id as u8;
    buf[2] = (seq_id >> 8) as u8;
}

pub fn decode_header(buf: &[u8]) -> (u8, u16) {
    let msg_type = buf[0];
    let seq_id = u16::from_le_bytes([buf[1], buf[2]]);
    (msg_type, seq_id)
}

// ============ Request commands (host -> device) ============

pub const CMD_GET_STATUS: u8 = 0;
pub const CMD_START_SCAN: u8 = 1;
pub const CMD_STOP_SCAN: u8 = 2;
pub const CMD_CONNECT: u8 = 3;
pub const CMD_DISCONNECT: u8 = 4;
pub const CMD_GET_BONDS: u8 = 5;
pub const CMD_CLEAR_BONDS: u8 = 6;
pub const CMD_SET_PROFILE: u8 = 7;
pub const CMD_GET_CONFIG: u8 = 8;
pub const CMD_SET_CONFIG: u8 = 9;
pub const CMD_SUBSCRIBE_LOGS: u8 = 10;
pub const CMD_UNSUBSCRIBE_LOGS: u8 = 11;
pub const CMD_GET_VERSION: u8 = 12;
pub const CMD_SET_ACTIVE_DEVICE: u8 = 13;
pub const CMD_CLEAR_ACTIVE_DEVICE: u8 = 14;
pub const CMD_UPDATE_BOND_PROFILE: u8 = 15;
pub const CMD_AUTO_CONNECT: u8 = 16;
pub const CMD_RESTART: u8 = 17;

#[derive(Clone, Debug, defmt::Format)]
pub enum Request {
    GetStatus,
    StartScan,
    StopScan,
    Connect {
        address: [u8; 6],
        addr_kind: u8,
        ignore_bond: bool,
    },
    Disconnect,
    GetBonds,
    ClearBonds,
    SetProfile {
        profile_id: u8,
    },
    GetConfig,
    SetConfig {
        key: u8,
        value: u32,
    },
    SubscribeLogs {
        level: u8,
    },
    UnsubscribeLogs,
    GetVersion,
    SetActiveDevice {
        address: [u8; 6],
        addr_kind: u8,
    },
    ClearActiveDevice,
    UpdateBondProfile {
        address: [u8; 6],
        profile_id: u8,
    },
    AutoConnect,
    Restart,
}

// ============ Response (device -> host) ============

pub const RESP_OK: u8 = 0;
pub const RESP_ERROR: u8 = 1;
pub const RESP_STATUS: u8 = 2;
pub const RESP_BONDS: u8 = 3;
pub const RESP_CONFIG: u8 = 4;
pub const RESP_VERSION: u8 = 5;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum ConnectionState {
    Disconnected = 0,
    Scanning = 1,
    Connecting = 2,
    Connected = 3,
    Pairing = 4,
    Ready = 5,
}

// ============ Events (device -> host, unsolicited) ============

pub const EVT_SCAN_RESULT: u8 = 0;
pub const EVT_CONNECTION_STATE: u8 = 1;
pub const EVT_PAIRING_STATUS: u8 = 2;
pub const EVT_LOG: u8 = 3;
pub const EVT_BOND_STORED: u8 = 4;
pub const EVT_BATTERY_LEVEL: u8 = 5;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum PairingState {
    // Started and KeysExchanged are protocol-defined states decoded by the CLI,
    // but the firmware only ever emits Complete and Failed.
    #[allow(dead_code)]
    Started = 0,
    #[allow(dead_code)]
    KeysExchanged = 1,
    Complete = 2,
    Failed = 3,
}

// ============ Error type ============

#[derive(Debug)]
#[allow(dead_code)]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidCbor,
    UnknownCommand(u8),
    MissingField,
}

type EncResult = Result<usize, ProtocolError>;

// ============ Decoding (host -> device) ============

/// Decode a request from a CBOR payload (after header has been stripped).
pub fn decode_request(cbor: &[u8]) -> Result<Request, ProtocolError> {
    let mut d = Decoder::new(cbor);
    let _arr_len = d.array().map_err(|_| ProtocolError::InvalidCbor)?;
    let cmd_id = d.u8().map_err(|_| ProtocolError::InvalidCbor)?;

    match cmd_id {
        CMD_GET_STATUS => Ok(Request::GetStatus),
        CMD_START_SCAN => Ok(Request::StartScan),
        CMD_STOP_SCAN => Ok(Request::StopScan),
        CMD_CONNECT => {
            let addr_bytes = d.bytes().map_err(|_| ProtocolError::MissingField)?;
            if addr_bytes.len() < 6 {
                return Err(ProtocolError::MissingField);
            }
            let mut address = [0u8; 6];
            address.copy_from_slice(&addr_bytes[..6]);
            let addr_kind = d.u8().map_err(|_| ProtocolError::MissingField)?;
            // ignore_bond field is optional for backwards compatibility
            let ignore_bond = d.bool().unwrap_or(false);
            Ok(Request::Connect {
                address,
                addr_kind,
                ignore_bond,
            })
        }
        CMD_DISCONNECT => Ok(Request::Disconnect),
        CMD_GET_BONDS => Ok(Request::GetBonds),
        CMD_CLEAR_BONDS => Ok(Request::ClearBonds),
        CMD_SET_PROFILE => {
            let profile_id = d.u8().map_err(|_| ProtocolError::MissingField)?;
            Ok(Request::SetProfile { profile_id })
        }
        CMD_GET_CONFIG => Ok(Request::GetConfig),
        CMD_SET_CONFIG => {
            let key = d.u8().map_err(|_| ProtocolError::MissingField)?;
            let value = d.u32().map_err(|_| ProtocolError::MissingField)?;
            Ok(Request::SetConfig { key, value })
        }
        CMD_SUBSCRIBE_LOGS => {
            let level = d.u8().map_err(|_| ProtocolError::MissingField)?;
            Ok(Request::SubscribeLogs { level })
        }
        CMD_UNSUBSCRIBE_LOGS => Ok(Request::UnsubscribeLogs),
        CMD_GET_VERSION => Ok(Request::GetVersion),
        CMD_SET_ACTIVE_DEVICE => {
            let addr_bytes = d.bytes().map_err(|_| ProtocolError::MissingField)?;
            if addr_bytes.len() < 6 {
                return Err(ProtocolError::MissingField);
            }
            let mut address = [0u8; 6];
            address.copy_from_slice(&addr_bytes[..6]);
            let addr_kind = d.u8().map_err(|_| ProtocolError::MissingField)?;
            Ok(Request::SetActiveDevice { address, addr_kind })
        }
        CMD_CLEAR_ACTIVE_DEVICE => Ok(Request::ClearActiveDevice),
        CMD_UPDATE_BOND_PROFILE => {
            let addr_bytes = d.bytes().map_err(|_| ProtocolError::MissingField)?;
            if addr_bytes.len() < 6 {
                return Err(ProtocolError::MissingField);
            }
            let mut address = [0u8; 6];
            address.copy_from_slice(&addr_bytes[..6]);
            let profile_id = d.u8().map_err(|_| ProtocolError::MissingField)?;
            Ok(Request::UpdateBondProfile {
                address,
                profile_id,
            })
        }
        CMD_AUTO_CONNECT => Ok(Request::AutoConnect),
        CMD_RESTART => Ok(Request::Restart),
        _ => Err(ProtocolError::UnknownCommand(cmd_id)),
    }
}

// ============ Encoding helpers ============

fn cbor_encode(buf: &mut [u8], f: impl FnOnce(&mut Encoder<&mut Cursor<&mut [u8]>>)) -> EncResult {
    let mut cursor = Cursor::new(buf);
    {
        let mut e = Encoder::new(&mut cursor);
        f(&mut e);
    }
    Ok(cursor.position())
}

// ============ Response encoding ============

/// Encode a simple Ok response.
pub fn encode_response_ok(buf: &mut [u8]) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(1).unwrap().u8(RESP_OK).unwrap();
    })
}

/// Encode an error response with a message.
pub fn encode_response_error(buf: &mut [u8], code: u8, message: &str) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(RESP_ERROR)
            .unwrap()
            .u8(code)
            .unwrap()
            .str(message)
            .unwrap();
    })
}

/// Encode a status response.
/// Extended format includes active device info and battery level
/// (backward compatible - clients can ignore extra fields).
pub fn encode_response_status(
    buf: &mut [u8],
    state: ConnectionState,
    bonded_count: u8,
    active_profile: u8,
    active_device_set: bool,
    active_device_address: &[u8; 6],
    battery_level: u8,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(7)
            .unwrap()
            .u8(RESP_STATUS)
            .unwrap()
            .u8(state as u8)
            .unwrap()
            .u8(bonded_count)
            .unwrap()
            .u8(active_profile)
            .unwrap()
            .bool(active_device_set)
            .unwrap()
            .bytes(active_device_address)
            .unwrap()
            .u8(battery_level)
            .unwrap();
    })
}

/// Encode a bonds response.
/// Each bond: [address(6 bytes), addr_kind, profile_id, name_str]
pub fn encode_response_bonds(buf: &mut [u8], bonds: &[([u8; 6], u8, u8, &str)]) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2).unwrap().u8(RESP_BONDS).unwrap();
        e.array(bonds.len() as u64).unwrap();
        for (addr, kind, profile, name) in bonds {
            e.array(4)
                .unwrap()
                .bytes(addr)
                .unwrap()
                .u8(*kind)
                .unwrap()
                .u8(*profile)
                .unwrap()
                .str(name)
                .unwrap();
        }
    })
}

/// Encode a version response.
pub fn encode_response_version(buf: &mut [u8], version: &str) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2)
            .unwrap()
            .u8(RESP_VERSION)
            .unwrap()
            .str(version)
            .unwrap();
    })
}

/// Encode a config response with all configurable values.
/// Format: [RESP_CONFIG, scroll_pct, pan_pct, x_pct, y_pct, scroll_threshold, max_detents]
pub fn encode_response_config(
    buf: &mut [u8],
    scroll_mult: u32,
    pan_mult: u32,
    x_mult: u32,
    y_mult: u32,
    scroll_threshold: u32,
    max_detents: u32,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(7)
            .unwrap()
            .u8(RESP_CONFIG)
            .unwrap()
            .u32(scroll_mult)
            .unwrap()
            .u32(pan_mult)
            .unwrap()
            .u32(x_mult)
            .unwrap()
            .u32(y_mult)
            .unwrap()
            .u32(scroll_threshold)
            .unwrap()
            .u32(max_detents)
            .unwrap();
    })
}

// ============ Event encoding ============

/// Encode a scan result event.
pub fn encode_event_scan_result(
    buf: &mut [u8],
    address: &[u8; 6],
    addr_kind: u8,
    name: &str,
    rssi: i8,
    is_hid: bool,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(6)
            .unwrap()
            .u8(EVT_SCAN_RESULT)
            .unwrap()
            .bytes(address)
            .unwrap()
            .u8(addr_kind)
            .unwrap()
            .str(name)
            .unwrap()
            .i8(rssi)
            .unwrap()
            .bool(is_hid)
            .unwrap();
    })
}

/// Encode a connection state change event.
pub fn encode_event_connection_state(
    buf: &mut [u8],
    state: ConnectionState,
    address: &[u8; 6],
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(EVT_CONNECTION_STATE)
            .unwrap()
            .u8(state as u8)
            .unwrap()
            .bytes(address)
            .unwrap();
    })
}

/// Encode a pairing status event.
pub fn encode_event_pairing_status(buf: &mut [u8], status: PairingState) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2)
            .unwrap()
            .u8(EVT_PAIRING_STATUS)
            .unwrap()
            .u8(status as u8)
            .unwrap();
    })
}

/// Encode a log event.
pub fn encode_event_log(buf: &mut [u8], level: u8, message: &str) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(EVT_LOG)
            .unwrap()
            .u8(level)
            .unwrap()
            .str(message)
            .unwrap();
    })
}

/// Encode a battery level event.
pub fn encode_event_battery_level(buf: &mut [u8], level: u8) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2)
            .unwrap()
            .u8(EVT_BATTERY_LEVEL)
            .unwrap()
            .u8(level)
            .unwrap();
    })
}

/// Encode a bond stored event.
pub fn encode_event_bond_stored(buf: &mut [u8], address: &[u8; 6], profile_id: u8) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(EVT_BOND_STORED)
            .unwrap()
            .bytes(address)
            .unwrap()
            .u8(profile_id)
            .unwrap();
    })
}
