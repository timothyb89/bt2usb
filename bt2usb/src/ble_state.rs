//! BLE state machine types and inter-task channels
//!
//! Defines the command/event protocol between the RPC handler and the
//! BLE central logic. The BLE task receives commands and emits events.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
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
    /// Disconnect the current device.
    Disconnect,
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
    UpdateBondProfile { address: [u8; 6], profile_id: u8 },
    /// Restart the system.
    Restart,
}

// ============ Events (BLE -> RPC) ============

/// A discovered BLE device from scanning.
#[derive(Clone, Debug, defmt::Format)]
pub struct ScanResultData {
    pub address: [u8; 6],
    pub addr_kind: u8,
    pub name: [u8; 32],
    pub name_len: u8,
    pub rssi: i8,
    pub is_hid: bool,
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
}

// ============ Channels ============

/// Commands from RPC handler to BLE state machine (capacity 4).
pub static BLE_CMD_CHANNEL: Channel<CriticalSectionRawMutex, BleCommand, 4> = Channel::new();

/// Events from BLE state machine to RPC handler (capacity 8).
pub static BLE_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, BleEvent, 8> = Channel::new();

/// Bond list response for GetBonds command
pub type BondList = heapless::Vec<([u8; 6], u8, u8, heapless::String<32>), 10>;

/// Response channel for GetBonds (capacity 1, only one request at a time).
pub static BONDS_RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, BondList, 1> = Channel::new();

/// Status information response
#[derive(Clone, Debug, defmt::Format)]
pub struct StatusInfo {
    pub bonded_count: u8,
    pub active_profile: u8,
    pub active_device_set: bool,
    pub active_device_address: [u8; 6],
}

/// Response channel for GetStatus (capacity 1, only one request at a time).
pub static STATUS_RESPONSE_CHANNEL: Channel<CriticalSectionRawMutex, StatusInfo, 1> = Channel::new();

// ============ Scanner Event Handler ============

/// BLE advertisement event handler that emits ScanResult events
/// for all discovered HID devices.
pub struct RpcScannerHandler;

impl EventHandler for RpcScannerHandler {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = it.next() {
            let data = report.data;
            let mut i = 0;
            let mut name = [0u8; 32];
            let mut name_len: u8 = 0;
            let mut is_hid = false;

            // Parse AD structures
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

            // Emit event for HID devices
            if is_hid {
                let mut addr_bytes = [0u8; 6];
                addr_bytes.copy_from_slice(report.addr.raw());
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
                }));
            }
        }
    }
}
