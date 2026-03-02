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

/// Battery Service UUID (0x180F)
pub const BATTERY_SERVICE_UUID: Uuid = Uuid::new_short(0x180F);

/// Battery Level Characteristic UUID (0x2A19)
pub const BATTERY_LEVEL_CHAR_UUID: Uuid = Uuid::new_short(0x2A19);

/// Last known battery level from the connected device.
/// 0xFF = unknown / not connected. Updated by the BLE task, read by any core.
pub static BATTERY_LEVEL: AtomicU8 = AtomicU8::new(0xFF);

/// Signal from BLE (Core 0) to USB battery task (Core 1).
/// Signaled with the new battery level (0-100) whenever it changes.
/// The USB task wakes up and sends a USB HID battery input report.
pub static BATTERY_USB_SIGNAL: Signal<CriticalSectionRawMutex, u8> = Signal::new();

/// Maximum HID report size we'll handle
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
