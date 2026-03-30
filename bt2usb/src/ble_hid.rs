//! BLE HID Central implementation for bt2usb
//!
//! This module handles:
//! - BLE HID service UUIDs and constants
//! - HID report event types for communication between tasks
//! - BLE HID report parsing

use core::sync::atomic::AtomicU8;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use trouble_host::prelude::*;

use crate::device_profile::DeviceProfile;

/// HID Service UUID (0x1812)
pub const HID_SERVICE_UUID: Uuid = Uuid::new_short(0x1812);

/// HID Report Characteristic UUID (0x2A4D)
pub const HID_REPORT_CHAR_UUID: Uuid = Uuid::new_short(0x2A4D);

/// HID Report Map Characteristic UUID (0x2A4B)
pub const HID_REPORT_MAP_UUID: Uuid = Uuid::new_short(0x2A4B);

/// Battery Service UUID (0x180F)
pub const BATTERY_SERVICE_UUID: Uuid = Uuid::new_short(0x180F);

/// Battery Level Characteristic UUID (0x2A19)
pub const BATTERY_LEVEL_CHAR_UUID: Uuid = Uuid::new_short(0x2A19);

use crate::ble::slots::MAX_CONNECTIONS;

/// Per-slot battery levels. 0xFF = unknown / not connected.
/// Updated by slot tasks on Core 0, read by USB task on Core 1.
pub static BATTERY_LEVELS: [AtomicU8; MAX_CONNECTIONS] = [
    AtomicU8::new(0xFF),
    AtomicU8::new(0xFF),
    AtomicU8::new(0xFF),
];

/// Legacy single battery level — returns the minimum across connected slots.
/// Used for backward compatibility with single-device status reporting.
pub static BATTERY_LEVEL: AtomicU8 = AtomicU8::new(0xFF);

/// Signal from BLE (Core 0) to USB battery task (Core 1).
/// Signaled with the new battery level (0-100) whenever it changes.
/// The USB task wakes up and sends a USB HID battery input report.
pub static BATTERY_USB_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Update battery level for a specific slot and refresh the aggregate.
pub fn update_battery_level(slot: usize, level: u8) {
    use core::sync::atomic::Ordering::Relaxed;
    BATTERY_LEVELS[slot].store(level, Relaxed);

    // Aggregate: minimum non-0xFF level across all slots
    let mut min = 0xFFu8;
    for bl in &BATTERY_LEVELS {
        let l = bl.load(Relaxed);
        if l != 0xFF && l < min {
            min = l;
        }
    }
    BATTERY_LEVEL.store(min, Relaxed);
    BATTERY_USB_SIGNAL.signal(min);
}

/// Clear battery level for a specific slot and refresh the aggregate.
pub fn clear_battery_level(slot: usize) {
    update_battery_level(slot, 0xFF);
}

// ============ Per-slot parsed report layouts ============

use crate::hid_report_map::{MouseReportLayout, MAX_LAYOUTS};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

/// Per-slot parsed mouse report layouts (multiple per slot for multi-report-ID devices).
/// Written by BLE (Core 0) during GATT discovery, read by USB (Core 1) during translation.
type SlotLayouts = heapless::Vec<MouseReportLayout, MAX_LAYOUTS>;

static SLOT_LAYOUTS: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<[SlotLayouts; MAX_CONNECTIONS]>,
> = BlockingMutex::new(RefCell::new([
    heapless::Vec::new(),
    heapless::Vec::new(),
    heapless::Vec::new(),
]));

/// Store parsed report layouts for a connection slot.
pub fn set_slot_layouts(
    slot: usize,
    layouts: Option<heapless::Vec<MouseReportLayout, MAX_LAYOUTS>>,
) {
    SLOT_LAYOUTS.lock(|inner| {
        let mut slots = inner.borrow_mut();
        slots[slot].clear();
        if let Some(ls) = layouts {
            for l in ls {
                let _ = slots[slot].push(l);
            }
        }
    });
}

/// Find the matching report layout for a notification of the given byte length.
/// Returns the layout whose `total_bytes` matches, or None.
pub fn find_slot_layout(slot: usize, data_len: usize) -> Option<MouseReportLayout> {
    SLOT_LAYOUTS.lock(|inner| {
        let slots = inner.borrow();
        slots[slot]
            .iter()
            .find(|l| l.total_bytes as usize == data_len)
            .copied()
    })
}

/// Maximum HID report size we'll handle.
/// Keep at 64 to avoid stack overflow in embassy tasks — HidReportEvent is passed by value.
/// MT2 translated reports (up to 57 bytes for 5 fingers) fit within this limit.
pub const MAX_HID_REPORT_SIZE: usize = 64;

/// Channel for passing HID reports from BLE to USB task
pub static HID_REPORT_CHANNEL: Channel<CriticalSectionRawMutex, HidReportEvent, 8> = Channel::new();

/// Types of HID reports
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum HidReportType {
    Keyboard,
    Mouse,
    Unknown(u8),
}

/// HID report event received from BLE device
#[derive(Clone, Debug, defmt::Format)]
pub struct HidReportEvent {
    /// Type of HID report
    pub report_type: HidReportType,
    /// Report ID (if present)
    pub report_id: u8,
    /// Device profile that produced this report
    pub profile: DeviceProfile,
    /// Connection slot index that produced this report
    pub slot_index: u8,
    /// Report data
    pub data: [u8; MAX_HID_REPORT_SIZE],
    /// Actual length of data
    pub len: usize,
}

impl HidReportEvent {
    pub fn new() -> Self {
        Self {
            report_type: HidReportType::Unknown(0),
            report_id: 0,
            profile: DeviceProfile::Generic,
            slot_index: 0,
            data: [0; MAX_HID_REPORT_SIZE],
            len: 0,
        }
    }
}

/// Parse a raw HID report and determine its type.
///
/// Classification is profile-aware: the Full Scroll Dial sends short reports
/// that would be misclassified by size heuristics alone.
pub fn parse_hid_report(data: &[u8], report_id: u8, profile: DeviceProfile) -> HidReportEvent {
    let mut event = HidReportEvent::new();
    event.report_id = report_id;
    event.profile = profile;
    event.len = data.len().min(MAX_HID_REPORT_SIZE);
    event.data[..event.len].copy_from_slice(&data[..event.len]);

    event.report_type = match profile {
        // Scroll dial variants are always mouse-type reports (scroll events)
        DeviceProfile::FullScrollDial | DeviceProfile::FullScrollDial16Bit => HidReportType::Mouse,
        // For other devices, use size-based heuristic
        _ => match data.len() {
            8 => HidReportType::Keyboard,
            3..=7 => HidReportType::Mouse,
            _ => HidReportType::Unknown(report_id),
        },
    };

    event
}
