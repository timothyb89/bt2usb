//! BLE state machine types and inter-task channels
//!
//! Defines the command/event protocol between the RPC handler and the
//! BLE central logic. The BLE task receives commands and emits events.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use trouble_host::prelude::*;
use trouble_host::scan::LeAdvReportsIter;

use crate::protocol::ConnectionState;

// ============ Commands (RPC -> BLE) ============

#[derive(Clone, Debug, defmt::Format)]
pub enum BleCommand {
    /// Start BLE scanning for HID devices. Results emitted as ScanResult events.
    StartScan,
    /// Stop an active scan.
    StopScan,
    /// Connect to a specific BLE device by address.
    Connect {
        address: [u8; 6],
        addr_kind: u8,
        ignore_bond: bool,
    },
    /// Disconnect a specific device, or all devices if address is None.
    Disconnect { address: Option<[u8; 6]> },
    /// Get current status (bond count, active profile, active device).
    GetStatus,
    /// Get list of bonded devices.
    GetBonds,
    /// Clear all bonded devices.
    ClearBonds,
    /// Auto-connect to the active device from preferences.
    AutoConnect,
    /// Set the active device for auto-connect.
    SetActiveDevice { address: [u8; 6], addr_kind: u8 },
    /// Clear the active device preference (disable auto-connect).
    ClearActiveDevice,
    /// Update the profile for an existing bond.
    UpdateBondProfile {
        address: [u8; 6],
        profile_id: u8,
        transport_type: TransportType,
    },
    /// Restart the system.
    Restart,
    /// Set a configuration value and persist to flash.
    SetConfig { key: u8, value: u32 },
    /// Set forced OS override (0=Auto, 1-3=forced) and persist to flash.
    SetForcedOs { os: u8 },
    /// Set auto-connect flag for a bonded device.
    SetAutoConnect {
        address: [u8; 6],
        enabled: bool,
        transport_type: TransportType,
    },
    /// Clear a single bond by address.
    ClearBond {
        address: [u8; 6],
        transport_type: TransportType,
    },
    /// Factory reset: clear all bonds and preferences, then restart.
    FactoryReset,
}

// ============ Events (BLE -> RPC) ============

/// A discovered device from scanning (BLE or Classic).
#[derive(Clone, Debug, defmt::Format)]
pub struct ScanResultData {
    pub address: [u8; 6],
    pub addr_kind: u8,
    pub name: [u8; 32],
    pub name_len: u8,
    pub rssi: i8,
    pub is_hid: bool,
    pub transport_type: TransportType,
}

#[derive(Clone, Debug, defmt::Format)]
pub enum BleEvent {
    /// A BLE device was discovered during scanning.
    ScanResult(ScanResultData),
    /// The connection state changed.
    StateChanged(ConnectionState),
    /// Pairing status update.
    PairingComplete,
    PairingFailed,
    /// A new bond was stored to flash.
    BondStored {
        address: [u8; 6],
        profile_id: u8,
    },
    /// Battery level changed (0–100%).
    BatteryLevel(u8),
}

// ============ Channels ============

/// Commands from RPC handler to BLE state machine (capacity 4).
pub static BLE_CMD_CHANNEL: Channel<CriticalSectionRawMutex, BleCommand, 4> = Channel::new();

/// Events from BLE state machine to RPC handler (capacity 8).
pub static BLE_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, BleEvent, 8> = Channel::new();

// ============ Transport Type ============

/// BLE vs Classic BT transport type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum TransportType {
    Ble = 0,
    Classic = 1,
}

impl TransportType {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Classic,
            _ => Self::Ble,
        }
    }

    pub fn as_u8(self) -> u8 {
        self as u8
    }
}

// ============ Classic BT Commands ============

/// Commands targeted at the Classic BT task.
#[derive(Clone, Debug, defmt::Format)]
pub enum ClassicCommand {
    /// Connect to a specific Classic BT device by address.
    Connect { address: [u8; 6] },
    /// Disconnect the active Classic BT connection.
    Disconnect,
    /// Start Classic Inquiry scan.
    Scan,
    /// Stop active Inquiry scan.
    ScanStop,
    /// A bond was cleared from flash — drop the in-memory link key for this
    /// address and disconnect the device if it's currently connected, so it
    /// can't be silently re-stored on the next reconnect.
    ClearBond { address: [u8; 6] },
}

/// Command channel for Classic BT task (capacity 2).
pub static CLASSIC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, ClassicCommand, 2> =
    Channel::new();

/// Bond list response for GetBonds command
/// Bond list entry: (address, addr_kind, profile_id, name, auto_connect, transport_type)
pub type BondList = heapless::Vec<([u8; 6], u8, u8, heapless::String<32>, bool, TransportType), 20>;

/// Response channel for GetBonds (capacity 1, only one request at a time).
pub static BONDS_RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, BondList, 1> = Channel::new();

/// Per-slot connection info for status reporting.
#[derive(Clone, Debug, defmt::Format)]
pub struct ConnectedDeviceInfo {
    pub address: [u8; 6],
    pub profile_id: u8,
    pub battery_level: u8,
    pub transport_type: TransportType,
}

/// Status information response
#[derive(Clone, Debug, defmt::Format)]
pub struct StatusInfo {
    pub bonded_count: u8,
    pub active_profile: u8,
    pub active_device_set: bool,
    pub active_device_address: [u8; 6],
    /// Last known battery level (0–100), or 0xFF if unknown/disconnected.
    pub battery_level: u8,
    /// Number of currently connected devices.
    pub connected_count: u8,
    /// Per-device connection info (up to 3 BLE + 1 Classic).
    pub connected_devices: [Option<ConnectedDeviceInfo>; 4],
}

/// Response channel for GetStatus (capacity 1, only one request at a time).
pub static STATUS_RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, StatusInfo, 1> =
    Channel::new();

/// Response channel for ClearBond — true on success, false if the bond was
/// not found (e.g. wrong transport or stale address).
pub static CLEAR_BOND_RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, bool, 1> = Channel::new();

// ============ Background Scan State ============

/// Signal from scanner handler to manager when a bonded device is seen advertising.
/// During background scan, the controller's Filter Accept List ensures that only
/// adverts from bonded devices reach us, so the handler can signal unconditionally.
pub static BONDED_DEVICE_SEEN: Signal<CriticalSectionRawMutex, [u8; 6]> = Signal::new();

/// Whether a background scan is active. During bg scan, the handler signals
/// `BONDED_DEVICE_SEEN` instead of emitting ScanResult events. The HW filter
/// guarantees the advert is from a bonded device.
pub static BG_SCAN_ACTIVE: AtomicBool = AtomicBool::new(false);

// ============ Scanner Event Handler ============

/// Max number of recently-seen HID devices to cache for merging scan responses.
const SCAN_HID_CACHE_SIZE: usize = 16;

/// Per-entry size: 6 (addr) + 32 (name) + 1 (name_len) = 39 bytes
const CACHE_ENTRY_SIZE: usize = 6 + 32 + 1;

/// Number of valid entries in the cache.
static SCAN_HID_CACHE_COUNT: AtomicU8 = AtomicU8::new(0);

/// Cache of recently-seen HID device addresses and names so that scan
/// responses (which carry the name but not the HID UUID) can be matched back,
/// and so the name is available at bond-store time.
#[allow(clippy::declare_interior_mutable_const)]
static SCAN_HID_CACHE: [AtomicU8; CACHE_ENTRY_SIZE * SCAN_HID_CACHE_SIZE] = {
    const ZERO: AtomicU8 = AtomicU8::new(0);
    [ZERO; CACHE_ENTRY_SIZE * SCAN_HID_CACHE_SIZE]
};

/// Find cache index for an address, or None.
fn hid_cache_find(addr: &[u8; 6]) -> Option<usize> {
    let count = SCAN_HID_CACHE_COUNT.load(Ordering::Relaxed) as usize;
    for i in 0..count {
        let base = i * CACHE_ENTRY_SIZE;
        let mut found = true;
        for j in 0..6 {
            if SCAN_HID_CACHE[base + j].load(Ordering::Relaxed) != addr[j] {
                found = false;
                break;
            }
        }
        if found {
            return Some(i);
        }
    }
    None
}

/// Record a HID device address (and name if present) so future scan responses
/// can be matched and the name is available at bond-store time.
fn hid_cache_insert(addr: &[u8; 6], name: &[u8; 32], name_len: u8) {
    if let Some(idx) = hid_cache_find(addr) {
        // Already present — update name if this report has a better one
        if name_len > 0 {
            let base = idx * CACHE_ENTRY_SIZE;
            for j in 0..32 {
                SCAN_HID_CACHE[base + 6 + j].store(name[j], Ordering::Relaxed);
            }
            SCAN_HID_CACHE[base + 38].store(name_len, Ordering::Relaxed);
        }
        return;
    }
    // Insert new entry if space available
    let count = SCAN_HID_CACHE_COUNT.load(Ordering::Relaxed) as usize;
    if count < SCAN_HID_CACHE_SIZE {
        let base = count * CACHE_ENTRY_SIZE;
        for j in 0..6 {
            SCAN_HID_CACHE[base + j].store(addr[j], Ordering::Relaxed);
        }
        for j in 0..32 {
            SCAN_HID_CACHE[base + 6 + j].store(name[j], Ordering::Relaxed);
        }
        SCAN_HID_CACHE[base + 38].store(name_len, Ordering::Relaxed);
        SCAN_HID_CACHE_COUNT.store((count + 1) as u8, Ordering::Relaxed);
    }
}

/// Check if an address was previously seen as a HID device.
fn hid_cache_contains(addr: &[u8; 6]) -> bool {
    hid_cache_find(addr).is_some()
}

/// Look up the cached device name for an address.
/// Returns (name, name_len). name_len == 0 means no name was cached.
pub fn hid_cache_get_name(addr: &[u8; 6]) -> ([u8; 32], u8) {
    if let Some(idx) = hid_cache_find(addr) {
        let base = idx * CACHE_ENTRY_SIZE;
        let mut name = [0u8; 32];
        for j in 0..32 {
            name[j] = SCAN_HID_CACHE[base + 6 + j].load(Ordering::Relaxed);
        }
        let name_len = SCAN_HID_CACHE[base + 38].load(Ordering::Relaxed);
        (name, name_len)
    } else {
        ([0u8; 32], 0)
    }
}

/// Clear the HID cache (called when a new scan starts).
pub fn hid_cache_clear() {
    SCAN_HID_CACHE_COUNT.store(0, Ordering::Relaxed);
}

/// BLE advertisement event handler that emits ScanResult events
/// for all discovered HID devices, and signals when bonded devices
/// are seen during background scanning.
pub struct RpcScannerHandler;

impl EventHandler for RpcScannerHandler {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let bg_active = BG_SCAN_ACTIVE.load(Ordering::Relaxed);

        while let Some(Ok(report)) = it.next() {
            // Background scan: HW Filter Accept List guarantees this advert is
            // from a bonded device, so signal directly without re-checking.
            if bg_active {
                let mut addr_bytes = [0u8; 6];
                addr_bytes.copy_from_slice(report.addr.raw());
                BONDED_DEVICE_SEEN.signal(addr_bytes);
                return;
            }

            // User-initiated scan: parse AD structures for HID devices
            let data = report.data;
            let mut i = 0;
            let mut name = [0u8; 32];
            let mut name_len: u8 = 0;
            let mut is_hid = false;

            while i < data.len() {
                let len = data[i] as usize;
                if len == 0 || i + 1 + len > data.len() {
                    break;
                }

                let ad_type = data[i + 1];
                let ad_data = &data[i + 2..i + 1 + len];

                match ad_type {
                    // Complete or Shortened Local Name
                    0x08 | 0x09 => {
                        let copy_len = ad_data.len().min(32);
                        name[..copy_len].copy_from_slice(&ad_data[..copy_len]);
                        name_len = copy_len as u8;
                    }
                    // 16-bit Service UUIDs (incomplete or complete)
                    0x02 | 0x03 => {
                        for chunk in ad_data.chunks(2) {
                            if chunk.len() == 2 && chunk[0] == 0x12 && chunk[1] == 0x18 {
                                is_hid = true;
                            }
                        }
                    }
                    _ => {}
                }
                i += 1 + len;
            }

            let mut addr_bytes = [0u8; 6];
            addr_bytes.copy_from_slice(report.addr.raw());

            if is_hid {
                // Cache this address (and name) so scan responses can be matched
                // and the name is available at bond-store time
                hid_cache_insert(&addr_bytes, &name, name_len);
                let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::ScanResult(ScanResultData {
                    address: addr_bytes,
                    addr_kind: if report.addr_kind == AddrKind::RANDOM {
                        1
                    } else {
                        0
                    },
                    name,
                    name_len,
                    rssi: report.rssi,
                    is_hid: true,
                    transport_type: TransportType::Ble,
                }));
            } else if name_len > 0 && hid_cache_contains(&addr_bytes) {
                // Scan response with a name for a previously-seen HID device —
                // update the cached name and emit a scan result
                hid_cache_insert(&addr_bytes, &name, name_len);
                let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::ScanResult(ScanResultData {
                    address: addr_bytes,
                    addr_kind: if report.addr_kind == AddrKind::RANDOM {
                        1
                    } else {
                        0
                    },
                    name,
                    name_len,
                    rssi: report.rssi,
                    is_hid: true,
                    transport_type: TransportType::Ble,
                }));
            }
        }
    }
}
