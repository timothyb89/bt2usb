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
pub const CMD_FORCE_REPROBE: u8 = 18;
pub const CMD_SET_FORCED_OS: u8 = 19;
pub const CMD_SUBSCRIBE_DEFMT: u8 = 20;
pub const CMD_UNSUBSCRIBE_DEFMT: u8 = 21;
pub const CMD_SET_AUTO_CONNECT: u8 = 22;
pub const CMD_CLEAR_BOND: u8 = 23;
pub const CMD_FACTORY_RESET: u8 = 24;
pub const CMD_CLASSIC_SCAN: u8 = 25;
pub const CMD_CLASSIC_SCAN_STOP: u8 = 26;
pub const CMD_GET_HID_ACTIVITY: u8 = 27;

#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    GetStatus,
    StartScan,
    StopScan,
    Connect {
        address: [u8; 6],
        addr_kind: u8,
        ignore_bond: bool,
        /// 0=BLE (default), 1=Classic BT
        transport_type: u8,
    },
    ClassicScan,
    ClassicScanStop,
    Disconnect {
        address: Option<[u8; 6]>,
    },
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
        transport_type: u8, // 0=BLE, 1=Classic
    },
    AutoConnect,
    Restart,
    ForceReprobe,
    /// Set forced OS override (0=Auto, 1=Windows, 2=Linux, 3=macOS).
    SetForcedOs {
        os: u8,
    },
    SubscribeDefmt,
    UnsubscribeDefmt,
    /// Set auto-connect flag for a bonded device.
    SetAutoConnect {
        address: [u8; 6],
        enabled: bool,
        transport_type: u8, // 0=BLE, 1=Classic
    },
    /// Clear a single bond by address.
    ClearBond {
        address: [u8; 6],
        transport_type: u8, // 0=BLE, 1=Classic
    },
    /// Factory reset: clear all bonds and preferences, then restart.
    FactoryReset,
    /// Get per-interface HID write activity counters (debug instrumentation).
    GetHidActivity,
}

// ============ Response (device -> host) ============

pub const RESP_OK: u8 = 0;
pub const RESP_ERROR: u8 = 1;
pub const RESP_STATUS: u8 = 2;
pub const RESP_BONDS: u8 = 3;
pub const RESP_CONFIG: u8 = 4;
pub const RESP_VERSION: u8 = 5;
// 6 is reserved by a legacy CLI `RESP_ACTIVE_DEVICE` path that was never
// emitted by firmware. Use 7 to avoid any wire-format ambiguity.
pub const RESP_HID_ACTIVITY: u8 = 7;

/// Number of HID interfaces tracked by HidActivity reports.
/// Must match `HID_IFACE_COUNT` in the firmware.
pub const HID_ACTIVITY_IFACE_COUNT: usize = 4;

/// One interface's activity snapshot: (last_write_ticks_us, writes, last_report_id).
/// `last_write_ticks_us == 0` means "never written since boot".
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HidActivityEntry {
    pub last_write_ticks_us: u64,
    pub writes: u32,
    pub last_report_id: u8,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
pub const EVT_DEFMT: u8 = 6;

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

// ============ Connected device info for status encoding ============

/// Wire-level connected device info used by `encode_response_status`.
/// Transport type is a raw u8 for wire compatibility.
pub struct ConnectedDevice {
    pub address: [u8; 6],
    pub profile_id: u8,
    pub battery_level: u8,
    pub transport_type: u8,
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
            // transport_type is optional (default 0=BLE for backward compat)
            let transport_type = d.u8().unwrap_or(0);
            Ok(Request::Connect {
                address,
                addr_kind,
                ignore_bond,
                transport_type,
            })
        }
        CMD_DISCONNECT => {
            // Optional address field for targeted disconnect (backward compatible)
            let address = if let Ok(addr_bytes) = d.bytes() {
                if addr_bytes.len() >= 6 {
                    let mut address = [0u8; 6];
                    address.copy_from_slice(&addr_bytes[..6]);
                    Some(address)
                } else {
                    None
                }
            } else {
                None
            };
            Ok(Request::Disconnect { address })
        }
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
            let transport_type = d.u8().unwrap_or(0); // backward-compatible
            Ok(Request::UpdateBondProfile {
                address,
                profile_id,
                transport_type,
            })
        }
        CMD_AUTO_CONNECT => Ok(Request::AutoConnect),
        CMD_RESTART => Ok(Request::Restart),
        CMD_FORCE_REPROBE => Ok(Request::ForceReprobe),
        CMD_SET_FORCED_OS => {
            let os = d.u8().map_err(|_| ProtocolError::InvalidCbor)?;
            Ok(Request::SetForcedOs { os })
        }
        CMD_SET_AUTO_CONNECT => {
            let addr_bytes = d.bytes().map_err(|_| ProtocolError::MissingField)?;
            if addr_bytes.len() < 6 {
                return Err(ProtocolError::MissingField);
            }
            let mut address = [0u8; 6];
            address.copy_from_slice(&addr_bytes[..6]);
            let enabled = d.bool().map_err(|_| ProtocolError::MissingField)?;
            let transport_type = d.u8().unwrap_or(0); // backward-compatible
            Ok(Request::SetAutoConnect {
                address,
                enabled,
                transport_type,
            })
        }
        CMD_CLEAR_BOND => {
            let addr_bytes = d.bytes().map_err(|_| ProtocolError::MissingField)?;
            if addr_bytes.len() < 6 {
                return Err(ProtocolError::MissingField);
            }
            let mut address = [0u8; 6];
            address.copy_from_slice(&addr_bytes[..6]);
            let transport_type = d.u8().unwrap_or(0); // backward-compatible
            Ok(Request::ClearBond {
                address,
                transport_type,
            })
        }
        CMD_SUBSCRIBE_DEFMT => Ok(Request::SubscribeDefmt),
        CMD_UNSUBSCRIBE_DEFMT => Ok(Request::UnsubscribeDefmt),
        CMD_FACTORY_RESET => Ok(Request::FactoryReset),
        CMD_CLASSIC_SCAN => Ok(Request::ClassicScan),
        CMD_CLASSIC_SCAN_STOP => Ok(Request::ClassicScanStop),
        CMD_GET_HID_ACTIVITY => Ok(Request::GetHidActivity),
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
/// Extended format includes active device info, battery level, and connected devices.
/// (backward compatible - older clients ignore extra fields).
pub fn encode_response_status(
    buf: &mut [u8],
    state: ConnectionState,
    bonded_count: u8,
    active_profile: u8,
    active_device_set: bool,
    active_device_address: &[u8; 6],
    battery_level: u8,
    detected_os: u8,
    connected_count: u8,
    connected_devices: &[Option<ConnectedDevice>; 4],
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(10)
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
            .unwrap()
            .u8(detected_os)
            .unwrap()
            .u8(connected_count)
            .unwrap();
        // Connected devices array (each: [addr, profile, battery, transport])
        e.array(connected_count as u64).unwrap();
        for dev in connected_devices.iter().flatten() {
            e.array(4)
                .unwrap()
                .bytes(&dev.address)
                .unwrap()
                .u8(dev.profile_id)
                .unwrap()
                .u8(dev.battery_level)
                .unwrap()
                .u8(dev.transport_type)
                .unwrap();
        }
    })
}

/// Encode a bonds response.
/// Each bond: [address, addr_kind, profile_id, name, auto_connect, transport_type]
/// transport_type: 0=BLE, 1=Classic BT
#[allow(clippy::type_complexity)]
pub fn encode_response_bonds(
    buf: &mut [u8],
    bonds: &[([u8; 6], u8, u8, &str, bool, u8)],
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2).unwrap().u8(RESP_BONDS).unwrap();
        e.array(bonds.len() as u64).unwrap();
        for (addr, kind, profile, name, auto_connect, transport) in bonds {
            e.array(6)
                .unwrap()
                .bytes(addr)
                .unwrap()
                .u8(*kind)
                .unwrap()
                .u8(*profile)
                .unwrap()
                .str(name)
                .unwrap()
                .bool(*auto_connect)
                .unwrap()
                .u8(*transport)
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
/// Format: [RESP_CONFIG, scroll_pct, pan_pct, x_pct, y_pct, scroll_threshold, max_detents, scroll_smoothing]
pub fn encode_response_config(
    buf: &mut [u8],
    scroll_mult: u32,
    pan_mult: u32,
    x_mult: u32,
    y_mult: u32,
    scroll_threshold: u32,
    max_detents: u32,
    scroll_smoothing: u32,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(8)
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
            .unwrap()
            .u32(scroll_smoothing)
            .unwrap();
    })
}

/// Encode a HID activity response.
///
/// Wire format: `[RESP_HID_ACTIVITY, now_ticks_us, [[last_us, writes, report_id], ...]]`
/// Timestamps are raw embassy tick counts (1 µs per tick on RP2040). The host
/// computes "N ms ago" as `(now - last) / 1000`.
pub fn encode_response_hid_activity(
    buf: &mut [u8],
    now_ticks_us: u64,
    entries: &[HidActivityEntry; HID_ACTIVITY_IFACE_COUNT],
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(3)
            .unwrap()
            .u8(RESP_HID_ACTIVITY)
            .unwrap()
            .u64(now_ticks_us)
            .unwrap();
        e.array(HID_ACTIVITY_IFACE_COUNT as u64).unwrap();
        for entry in entries {
            e.array(3)
                .unwrap()
                .u64(entry.last_write_ticks_us)
                .unwrap()
                .u32(entry.writes)
                .unwrap()
                .u8(entry.last_report_id)
                .unwrap();
        }
    })
}

/// Decode a HID activity response body (CBOR, after header).
/// Returns `(now_ticks_us, entries)`.
pub fn decode_response_hid_activity(
    cbor: &[u8],
) -> Result<(u64, [HidActivityEntry; HID_ACTIVITY_IFACE_COUNT]), ProtocolError> {
    let mut d = Decoder::new(cbor);
    let _arr = d.array().map_err(|_| ProtocolError::InvalidCbor)?;
    let tag = d.u8().map_err(|_| ProtocolError::InvalidCbor)?;
    if tag != RESP_HID_ACTIVITY {
        return Err(ProtocolError::InvalidCbor);
    }
    let now = d.u64().map_err(|_| ProtocolError::MissingField)?;
    let count = d
        .array()
        .map_err(|_| ProtocolError::InvalidCbor)?
        .ok_or(ProtocolError::MissingField)?;
    let mut entries: [HidActivityEntry; HID_ACTIVITY_IFACE_COUNT] = Default::default();
    for entry in entries.iter_mut().take(count as usize) {
        let _inner = d.array().map_err(|_| ProtocolError::InvalidCbor)?;
        entry.last_write_ticks_us = d.u64().map_err(|_| ProtocolError::MissingField)?;
        entry.writes = d.u32().map_err(|_| ProtocolError::MissingField)?;
        entry.last_report_id = d.u8().map_err(|_| ProtocolError::MissingField)?;
    }
    Ok((now, entries))
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
    transport_type: u8,
) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(7)
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
            .unwrap()
            .u8(transport_type)
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

/// Encode a raw defmt frame event.
pub fn encode_event_defmt(buf: &mut [u8], frame_data: &[u8]) -> EncResult {
    cbor_encode(buf, |e| {
        e.array(2)
            .unwrap()
            .u8(EVT_DEFMT)
            .unwrap()
            .bytes(frame_data)
            .unwrap();
    })
}
