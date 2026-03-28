//! Connection slot management for multi-device support
//!
//! Provides per-slot state, command channels, and connect signals for up to
//! MAX_CONNECTIONS concurrent BLE connections. The connection manager dispatches
//! commands to slots, and each slot runs its own connection/GATT lifecycle.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

use crate::device_profile::DeviceProfile;

/// Maximum concurrent BLE connections.
pub const MAX_CONNECTIONS: usize = 3;

// ============ Slot State ============

/// Slot states (stored as AtomicU8 for cross-task visibility).
const SLOT_IDLE: u8 = 0;
const SLOT_CONNECTING: u8 = 1;
const SLOT_CONNECTED: u8 = 2;

/// Per-slot connection state (atomics for lock-free cross-task reads).
pub static SLOT_STATES: [AtomicU8; MAX_CONNECTIONS] =
    [AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0)];

/// Per-slot device address bytes (flat array of 6 * MAX_CONNECTIONS AtomicU8s).
/// Only the owning slot task writes; any task can read.
static SLOT_ADDR_BYTES: [AtomicU8; 6 * MAX_CONNECTIONS] = [
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
    AtomicU8::new(0),
];

fn set_slot_address(slot: usize, addr: [u8; 6]) {
    let base = slot * 6;
    for i in 0..6 {
        SLOT_ADDR_BYTES[base + i].store(addr[i], Ordering::Relaxed);
    }
}

pub fn get_slot_address(slot: usize) -> [u8; 6] {
    let base = slot * 6;
    let mut addr = [0u8; 6];
    for i in 0..6 {
        addr[i] = SLOT_ADDR_BYTES[base + i].load(Ordering::Relaxed);
    }
    addr
}

/// Per-slot device profile (set when connecting).
pub static SLOT_PROFILES: [AtomicU8; MAX_CONNECTIONS] =
    [AtomicU8::new(0), AtomicU8::new(0), AtomicU8::new(0)];

pub fn set_slot_state(slot: usize, state: u8) {
    SLOT_STATES[slot].store(state, Ordering::Relaxed);
}

pub fn get_slot_state(slot: usize) -> u8 {
    SLOT_STATES[slot].load(Ordering::Relaxed)
}

pub fn is_slot_idle(slot: usize) -> bool {
    get_slot_state(slot) == SLOT_IDLE
}

pub fn set_slot_idle(slot: usize) {
    set_slot_state(slot, SLOT_IDLE);
    set_slot_address(slot, [0; 6]);
    SLOT_PROFILES[slot].store(0, Ordering::Relaxed);
}

pub fn set_slot_connecting(slot: usize, address: [u8; 6], profile: DeviceProfile) {
    set_slot_address(slot, address);
    SLOT_PROFILES[slot].store(profile.to_id(), Ordering::Relaxed);
    set_slot_state(slot, SLOT_CONNECTING);
}

pub fn set_slot_connected(slot: usize) {
    set_slot_state(slot, SLOT_CONNECTED);
}

/// Find the first idle slot. Returns None if all slots are occupied.
pub fn find_idle_slot() -> Option<usize> {
    (0..MAX_CONNECTIONS).find(|&i| is_slot_idle(i))
}

/// Find the slot connected to a specific address. Returns None if not found.
pub fn find_slot_by_address(address: &[u8; 6]) -> Option<usize> {
    (0..MAX_CONNECTIONS)
        .find(|&i| get_slot_state(i) != SLOT_IDLE && get_slot_address(i) == *address)
}

/// Count of currently non-idle slots.
#[allow(dead_code)]
pub fn connected_count() -> u8 {
    (0..MAX_CONNECTIONS).filter(|&i| !is_slot_idle(i)).count() as u8
}

// ============ Connect Signals ============

/// Request sent from the connection manager to a slot task to initiate a connection.
#[derive(Clone, Debug, defmt::Format)]
pub struct ConnectRequest {
    pub address: [u8; 6],
    pub addr_kind: u8,
    pub profile: DeviceProfile,
    pub has_stored_bond: bool,
}

/// Per-slot connect signals. The manager signals a slot to connect.
pub static CONNECT_SIGNALS: [Signal<CriticalSectionRawMutex, ConnectRequest>; MAX_CONNECTIONS] =
    [Signal::new(), Signal::new(), Signal::new()];

// ============ Slot Commands ============

/// Commands targeted at a specific active connection slot.
#[derive(Clone, Debug, defmt::Format)]
pub enum SlotCommand {
    /// Disconnect this slot's connection.
    Disconnect,
    /// Update the device profile for this slot's connection.
    UpdateProfile(u8),
}

/// Per-slot command channels for targeted commands (disconnect, profile update).
pub static SLOT_CMD_CHANNELS: [Channel<CriticalSectionRawMutex, SlotCommand, 2>; MAX_CONNECTIONS] =
    [Channel::new(), Channel::new(), Channel::new()];
