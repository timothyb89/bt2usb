//! RPC protocol definitions (host-side mirror of firmware protocol.rs)
//!
//! Wire format: COBS-framed messages with a 3-byte header + CBOR body.

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

// ============ Command IDs ============

pub const CMD_GET_STATUS: u8 = 0;
pub const CMD_START_SCAN: u8 = 1;
pub const CMD_STOP_SCAN: u8 = 2;
pub const CMD_CONNECT: u8 = 3;
pub const CMD_DISCONNECT: u8 = 4;
pub const CMD_GET_BONDS: u8 = 5;
pub const CMD_CLEAR_BONDS: u8 = 6;
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

// ============ Response IDs ============

pub const RESP_OK: u8 = 0;
pub const RESP_ERROR: u8 = 1;
pub const RESP_STATUS: u8 = 2;
pub const RESP_BONDS: u8 = 3;
pub const RESP_CONFIG: u8 = 4;
pub const RESP_VERSION: u8 = 5;
pub const RESP_ACTIVE_DEVICE: u8 = 6;

// ============ Event IDs ============

pub const EVT_SCAN_RESULT: u8 = 0;
pub const EVT_CONNECTION_STATE: u8 = 1;
pub const EVT_PAIRING_STATUS: u8 = 2;
pub const EVT_LOG: u8 = 3;
pub const EVT_BOND_STORED: u8 = 4;
pub const EVT_BATTERY_LEVEL: u8 = 5;

// ============ Connection state ============

#[derive(Clone, Copy, Debug)]
pub enum ConnectionState {
    Disconnected = 0,
    Scanning = 1,
    Connecting = 2,
    Connected = 3,
    Pairing = 4,
    Ready = 5,
}

impl ConnectionState {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Disconnected,
            1 => Self::Scanning,
            2 => Self::Connecting,
            3 => Self::Connected,
            4 => Self::Pairing,
            5 => Self::Ready,
            _ => Self::Disconnected,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Disconnected => "Disconnected",
            Self::Scanning => "Scanning",
            Self::Connecting => "Connecting",
            Self::Connected => "Connected",
            Self::Pairing => "Pairing",
            Self::Ready => "Ready",
        }
    }
}

// ============ Pairing state ============

#[derive(Clone, Copy, Debug)]
pub enum PairingState {
    Started = 0,
    KeysExchanged = 1,
    Complete = 2,
    Failed = 3,
}

impl PairingState {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Started,
            1 => Self::KeysExchanged,
            2 => Self::Complete,
            3 => Self::Failed,
            _ => Self::Failed,
        }
    }
}

// ============ Config keys ============

pub fn config_key_from_name(name: &str) -> Option<u8> {
    match name {
        "scroll" => Some(0),
        "pan" => Some(1),
        "x" => Some(2),
        "y" => Some(3),
        "threshold" => Some(4),
        "max_detents" => Some(5),
        _ => None,
    }
}

// ============ Log levels ============

pub fn log_level_name(level: u8) -> &'static str {
    match level {
        0 => "DEBUG",
        1 => "INFO",
        2 => "WARN",
        3 => "ERROR",
        _ => "?",
    }
}

// ============ CBOR encoding helpers ============

type EncResult = Result<usize, EncodeError>;

#[derive(Debug)]
pub struct EncodeError;

fn cbor_encode(buf: &mut [u8], f: impl FnOnce(&mut Encoder<&mut Cursor<&mut [u8]>>)) -> EncResult {
    let mut cursor = Cursor::new(buf);
    {
        let mut e = Encoder::new(&mut cursor);
        f(&mut e);
    }
    Ok(cursor.position())
}

// ============ Request encoding (host -> device) ============

pub fn encode_request_simple(buf: &mut [u8], cmd_id: u8) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(1).unwrap().u8(cmd_id).unwrap();
    })
}

pub fn encode_request_connect(buf: &mut [u8], address: &[u8; 6], addr_kind: u8) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(4)
            .unwrap()
            .u8(CMD_CONNECT)
            .unwrap()
            .bytes(address)
            .unwrap()
            .u8(addr_kind)
            .unwrap()
            .bool(false) // ignore_bond = false
            .unwrap();
    })
}

pub fn encode_request_subscribe_logs(buf: &mut [u8], level: u8) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2)
            .unwrap()
            .u8(CMD_SUBSCRIBE_LOGS)
            .unwrap()
            .u8(level)
            .unwrap();
    })
}

pub fn encode_request_set_active_device(
    buf: &mut [u8],
    address: &[u8; 6],
    addr_kind: u8,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(CMD_SET_ACTIVE_DEVICE)
            .unwrap()
            .bytes(address)
            .unwrap()
            .u8(addr_kind)
            .unwrap();
    })
}

pub fn encode_request_update_bond_profile(
    buf: &mut [u8],
    address: &[u8; 6],
    profile_id: u8,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(CMD_UPDATE_BOND_PROFILE)
            .unwrap()
            .bytes(address)
            .unwrap()
            .u8(profile_id)
            .unwrap();
    })
}

pub fn encode_request_set_config(buf: &mut [u8], key: u8, value: u32) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(CMD_SET_CONFIG)
            .unwrap()
            .u8(key)
            .unwrap()
            .u32(value)
            .unwrap();
    })
}

// ============ Response decoding (device -> host) ============

#[derive(Debug)]
pub enum Response {
    Ok,
    Error {
        code: u8,
        message: String,
    },
    Status {
        state: ConnectionState,
        bonded_count: u8,
        active_profile: u8,
        active_device_set: bool,
        active_device_address: [u8; 6],
        battery_level: u8,
    },
    Bonds {
        bonds: Vec<BondEntry>,
    },
    Config {
        scroll_mult: u32,
        pan_mult: u32,
        x_mult: u32,
        y_mult: u32,
        scroll_threshold: u32,
        max_detents: u32,
    },
    Version {
        version: String,
    },
    ActiveDevice {
        #[allow(dead_code)]
        address: [u8; 6],
        #[allow(dead_code)]
        addr_kind: u8,
    },
}

#[derive(Debug)]
pub struct BondEntry {
    pub address: [u8; 6],
    pub addr_kind: u8,
    pub profile_id: u8,
    pub name: String,
}

pub fn decode_response(cbor: &[u8]) -> Result<Response, String> {
    let mut d = Decoder::new(cbor);
    let _arr_len = d.array().map_err(|e| format!("array: {e}"))?;
    let resp_id = d.u8().map_err(|e| format!("resp_id: {e}"))?;

    match resp_id {
        RESP_OK => Ok(Response::Ok),
        RESP_ERROR => {
            let code = d.u8().map_err(|e| format!("error code: {e}"))?;
            let message = d.str().map_err(|e| format!("error msg: {e}"))?.to_string();
            Ok(Response::Error { code, message })
        }
        RESP_STATUS => {
            let state = ConnectionState::from_u8(d.u8().map_err(|e| format!("state: {e}"))?);
            let bonded_count = d.u8().map_err(|e| format!("bonded: {e}"))?;
            let active_profile = d.u8().map_err(|e| format!("profile: {e}"))?;

            // New fields (backward compatible - optional for older firmware)
            let active_device_set = d.bool().unwrap_or(false);
            let addr_bytes = d.bytes().ok();
            let mut active_device_address = [0u8; 6];
            if let Some(bytes) = addr_bytes {
                if bytes.len() >= 6 {
                    active_device_address.copy_from_slice(&bytes[..6]);
                }
            }

            let battery_level = d.u8().unwrap_or(0xFF);

            Ok(Response::Status {
                state,
                bonded_count,
                active_profile,
                active_device_set,
                active_device_address,
                battery_level,
            })
        }
        RESP_BONDS => {
            let _bond_arr = d.array().map_err(|e| format!("bonds array: {e}"))?;
            let mut bonds = Vec::new();
            // Try to decode bond entries until we run out
            while d.position() < cbor.len() {
                if d.array().is_err() {
                    break;
                }
                let addr_bytes = d.bytes().map_err(|e| format!("addr: {e}"))?;
                let mut address = [0u8; 6];
                if addr_bytes.len() >= 6 {
                    address.copy_from_slice(&addr_bytes[..6]);
                }
                let addr_kind = d.u8().map_err(|e| format!("kind: {e}"))?;
                let profile_id = d.u8().map_err(|e| format!("profile: {e}"))?;
                let name = d.str().map_err(|e| format!("name: {e}"))?.to_string();
                bonds.push(BondEntry {
                    address,
                    addr_kind,
                    profile_id,
                    name,
                });
            }
            Ok(Response::Bonds { bonds })
        }
        RESP_CONFIG => {
            let scroll_mult = d.u32().map_err(|e| format!("scroll: {e}"))?;
            let pan_mult = d.u32().map_err(|e| format!("pan: {e}"))?;
            let x_mult = d.u32().map_err(|e| format!("x: {e}"))?;
            let y_mult = d.u32().map_err(|e| format!("y: {e}"))?;
            // New fields (backward compatible — default if missing)
            let scroll_threshold = d.u32().unwrap_or(120);
            let max_detents = d.u32().unwrap_or(3);
            Ok(Response::Config {
                scroll_mult,
                pan_mult,
                x_mult,
                y_mult,
                scroll_threshold,
                max_detents,
            })
        }
        RESP_VERSION => {
            let version = d.str().map_err(|e| format!("version: {e}"))?.to_string();
            Ok(Response::Version { version })
        }
        RESP_ACTIVE_DEVICE => {
            let addr_bytes = d.bytes().map_err(|e| format!("addr: {e}"))?;
            let mut address = [0u8; 6];
            if addr_bytes.len() >= 6 {
                address.copy_from_slice(&addr_bytes[..6]);
            }
            let addr_kind = d.u8().map_err(|e| format!("kind: {e}"))?;
            Ok(Response::ActiveDevice { address, addr_kind })
        }
        _ => Err(format!("unknown response id: {resp_id}")),
    }
}

// ============ Event decoding (device -> host) ============

#[derive(Debug)]
pub enum Event {
    ScanResult {
        address: [u8; 6],
        #[allow(dead_code)]
        addr_kind: u8,
        name: String,
        rssi: i8,
        is_hid: bool,
    },
    ConnectionState {
        state: self::ConnectionState,
        #[allow(dead_code)]
        address: [u8; 6],
    },
    PairingStatus {
        status: PairingState,
    },
    Log {
        level: u8,
        message: String,
    },
    BondStored {
        address: [u8; 6],
        profile_id: u8,
    },
    BatteryLevel {
        level: u8,
    },
}

pub fn decode_event(cbor: &[u8]) -> Result<Event, String> {
    let mut d = Decoder::new(cbor);
    let _arr_len = d.array().map_err(|e| format!("array: {e}"))?;
    let evt_id = d.u8().map_err(|e| format!("evt_id: {e}"))?;

    match evt_id {
        EVT_SCAN_RESULT => {
            let addr_bytes = d.bytes().map_err(|e| format!("addr: {e}"))?;
            let mut address = [0u8; 6];
            if addr_bytes.len() >= 6 {
                address.copy_from_slice(&addr_bytes[..6]);
            }
            let addr_kind = d.u8().map_err(|e| format!("kind: {e}"))?;
            let name = d.str().map_err(|e| format!("name: {e}"))?.to_string();
            let rssi = d.i8().map_err(|e| format!("rssi: {e}"))?;
            let is_hid = d.bool().map_err(|e| format!("is_hid: {e}"))?;
            Ok(Event::ScanResult {
                address,
                addr_kind,
                name,
                rssi,
                is_hid,
            })
        }
        EVT_CONNECTION_STATE => {
            let state = ConnectionState::from_u8(d.u8().map_err(|e| format!("state: {e}"))?);
            let addr_bytes = d.bytes().map_err(|e| format!("addr: {e}"))?;
            let mut address = [0u8; 6];
            if addr_bytes.len() >= 6 {
                address.copy_from_slice(&addr_bytes[..6]);
            }
            Ok(Event::ConnectionState { state, address })
        }
        EVT_PAIRING_STATUS => {
            let status = PairingState::from_u8(d.u8().map_err(|e| format!("status: {e}"))?);
            Ok(Event::PairingStatus { status })
        }
        EVT_LOG => {
            let level = d.u8().map_err(|e| format!("level: {e}"))?;
            let message = d.str().map_err(|e| format!("msg: {e}"))?.to_string();
            Ok(Event::Log { level, message })
        }
        EVT_BOND_STORED => {
            let addr_bytes = d.bytes().map_err(|e| format!("addr: {e}"))?;
            let mut address = [0u8; 6];
            if addr_bytes.len() >= 6 {
                address.copy_from_slice(&addr_bytes[..6]);
            }
            let profile_id = d.u8().map_err(|e| format!("profile: {e}"))?;
            Ok(Event::BondStored {
                address,
                profile_id,
            })
        }
        EVT_BATTERY_LEVEL => {
            let level = d.u8().map_err(|e| format!("level: {e}"))?;
            Ok(Event::BatteryLevel { level })
        }
        _ => Err(format!("unknown event id: {evt_id}")),
    }
}
