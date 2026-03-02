//! Device profile system for bt2usb
//!
//! Each supported BLE HID device has a profile that defines:
//! - Its advertised BLE name (for scanning)
//! - How to parse its raw BLE HID reports into USB HID reports
//! - A storage ID for persisting the profile alongside bond data

use core::sync::atomic::{AtomicI32, Ordering};
use defmt::*;
use usbd_hid::descriptor::MouseReport;

use crate::usb_hid::{MouseReport16, HIRES_SCROLL_ENABLED};

static PREV_SCROLL_RAW: AtomicI32 = AtomicI32::new(0);
static SCROLL_ACCUMULATOR: AtomicI32 = AtomicI32::new(0);

/// Threshold in raw scroll units before emitting an event in standard mode.
/// 120 = one standard detent in the HID Resolution Multiplier spec.
const SCROLL_ACCUMULATOR_THRESHOLD: i32 = 120;

/// Reset scroll accumulator state (call on USB reconfiguration).
pub fn reset_scroll_accumulator() {
    SCROLL_ACCUMULATOR.store(0, Ordering::Relaxed);
    PREV_SCROLL_RAW.store(0, Ordering::Relaxed);
}

/// Known device profiles
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum DeviceProfile {
    /// Logitech MX Master 3S — full mouse with 16-bit X/Y, wheel, pan
    MxMaster3S,
    /// Full Scroll Dial — scroll-only device with 8-bit wheel (scaled, compatible)
    FullScrollDial,
    /// Full Scroll Dial — experimental 16-bit wheel mode (unscaled, may have OS compatibility issues)
    FullScrollDial16Bit,
    /// Fallback for unknown devices — standard 3-byte mouse
    Generic,
}

impl DeviceProfile {
    /// BLE advertised name used for scanning
    pub fn name(&self) -> &'static str {
        match self {
            Self::MxMaster3S => "MX Master 3S",
            Self::FullScrollDial => "Full Scroll Dial",
            Self::FullScrollDial16Bit => "Full Scroll Dial", // Same BLE device
            Self::Generic => "",
        }
    }

    /// Look up a profile by BLE device name
    #[allow(dead_code)]
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "MX Master 3S" => Some(Self::MxMaster3S),
            "Full Scroll Dial" => Some(Self::FullScrollDial),
            _ => None,
        }
    }

    /// Numeric ID for flash storage
    pub fn to_id(self) -> u8 {
        match self {
            Self::Generic => 0,
            Self::MxMaster3S => 1,
            Self::FullScrollDial => 2,
            Self::FullScrollDial16Bit => 3,
        }
    }

    /// Restore profile from stored numeric ID
    pub fn from_id(id: u8) -> Self {
        match id {
            1 => Self::MxMaster3S,
            2 => Self::FullScrollDial,
            3 => Self::FullScrollDial16Bit,
            _ => Self::Generic,
        }
    }

    /// Translate a raw BLE HID report into a USB MouseReport.
    ///
    /// Each device has its own byte layout; this dispatches to the
    /// appropriate parser. Returns a zeroed report if data is too short.
    pub fn translate_mouse_report(&self, data: &[u8], len: usize) -> MouseReport {
        match self {
            Self::MxMaster3S => translate_mx_master(data, len),
            Self::FullScrollDial => translate_scroll_dial(data, len),
            Self::FullScrollDial16Bit => translate_scroll_dial(data, len), // Fallback for 8-bit mode
            Self::Generic => translate_generic(data, len),
        }
    }

    /// Check if this profile uses 16-bit mouse reports
    pub fn uses_16bit_reports(&self) -> bool {
        matches!(self, Self::FullScrollDial16Bit)
    }

    /// Translate a raw BLE HID report into a 16-bit USB MouseReport16.
    /// Only valid for profiles that use 16-bit reports.
    pub fn translate_mouse_report_16bit(&self, data: &[u8], len: usize) -> MouseReport16 {
        match self {
            Self::FullScrollDial16Bit => translate_scroll_dial_16bit(data, len),
            // Other profiles don't support 16-bit mode, return zeros
            _ => MouseReport16 {
                buttons: 0,
                x: 0,
                y: 0,
                wheel: 0,
                pan: 0,
            },
        }
    }
}

/// MX Master 3S: 7-byte report
///   Byte 0: buttons
///   Byte 1-2: X delta (16-bit LE signed)
///   Byte 3-4: Y delta (16-bit LE signed)
///   Byte 5: wheel
///   Byte 6: pan (horizontal scroll)
fn translate_mx_master(data: &[u8], len: usize) -> MouseReport {
    if len < 3 {
        return MouseReport {
            buttons: 0,
            x: 0,
            y: 0,
            wheel: 0,
            pan: 0,
        };
    }

    let (x, y) = if len >= 5 {
        let x16 = i16::from_le_bytes([data[1], data[2]]);
        let y16 = i16::from_le_bytes([data[3], data[4]]);
        debug!("MX Master 16-bit: X={}, Y={}", x16, y16);
        // MX Master 3S has asymmetric native resolution
        let x_scaled = (x16 / 64).clamp(-127, 127) as i8;
        let y_scaled = (y16 / 8).clamp(-127, 127) as i8;
        (x_scaled, y_scaled)
    } else {
        (data[1] as i8, data[2] as i8)
    };

    let wheel_raw = data.get(5).copied().unwrap_or(0) as i8;
    let pan_raw = data.get(6).copied().unwrap_or(0) as i8;

    // In high-res mode (120 units/detent), MX Master wheel values are
    // already in detent units (typically ±1), so scale up to match.
    let (wheel, pan) = if HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
        (
            (wheel_raw as i16 * 120).clamp(-127, 127) as i8,
            (pan_raw as i16 * 120).clamp(-127, 127) as i8,
        )
    } else {
        (wheel_raw, pan_raw)
    };

    MouseReport {
        buttons: data[0],
        x,
        y,
        wheel,
        pan,
    }
}

/// Full Scroll Dial: scroll-only device with button.
///
/// Confirmed report format (4 bytes):
///   Bytes 0-1: scroll delta (signed i16 LE, positive = scroll down)
///   Bytes 2-3: always 0x00 (reserved / unknown)
///
/// No button byte — the device only reports scroll rotation.
///
/// The device sends velocity-dependent 16-bit values:
///   Slow scroll: ~5-20 raw units
///   Fast scroll: ~100-500+ raw units
///
/// Scaling:
///   High-res (120 units/detent): divide by 3 to fit in i8 USB range (-127 to 127).
///     ~360 raw → 120 USB units = 1 detent
///   Standard (1 unit/detent): divide by 360 for detent-level scrolling.
fn translate_scroll_dial(data: &[u8], len: usize) -> MouseReport {
    if len < 2 {
        return MouseReport {
            buttons: 0,
            x: 0,
            y: 0,
            wheel: 0,
            pan: 0,
        };
    }

    // Device sends 16-bit scroll delta in little-endian format
    let raw = i16::from_le_bytes([data[0], data[1]]);

    let wheel = if HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
        // High-res: 120 units per detent. Device values range from ~5 (slow)
        // to 500+ (fast). Divide by 3 to map into USB i8 range while avoiding
        // frequent clamping. ~360 raw → 120 USB = 1 detent. Values >381 clamp.
        // This prevents fluctuation during coasting by giving more headroom.
        (raw / 3).clamp(-127, 127) as i8
    } else {
        // Standard mode with accumulator: track remainder across samples so
        // sub-detent contributions aren't lost. This is especially important
        // on macOS which doesn't enable hires mode and filters small values.
        let prev_acc = SCROLL_ACCUMULATOR.load(Ordering::Relaxed);
        let new_acc = prev_acc + raw as i32;

        if new_acc.abs() >= SCROLL_ACCUMULATOR_THRESHOLD {
            let detents = new_acc / SCROLL_ACCUMULATOR_THRESHOLD;
            let remainder = new_acc % SCROLL_ACCUMULATOR_THRESHOLD;
            SCROLL_ACCUMULATOR.store(remainder, Ordering::Relaxed);
            debug!(
                "8-bit mode (standard/accum): acc={} -> emit {} detents, remainder={}",
                new_acc, detents, remainder
            );
            detents.clamp(-127, 127) as i8
        } else {
            SCROLL_ACCUMULATOR.store(new_acc, Ordering::Relaxed);
            debug!("8-bit mode (standard/accum): acc={}, below threshold", new_acc);
            0i8
        }
    };

    MouseReport {
        buttons: 0,
        x: 0,
        y: 0,
        wheel,
        pan: 0,
    }
}

/// Full Scroll Dial: 16-bit mode (EXPERIMENTAL).
///
/// Same report format as 8-bit mode, but returns the full 16-bit scroll values
/// without scaling. This preserves complete velocity information from the BLE device.
///
/// Scaling:
///   High-res mode: No scaling, direct passthrough of 16-bit values
///   Standard mode: Divide by 120 to convert to detent units
fn translate_scroll_dial_16bit(data: &[u8], len: usize) -> MouseReport16 {
    if len < 2 {
        return MouseReport16 {
            buttons: 0,
            x: 0,
            y: 0,
            wheel: 0,
            pan: 0,
        };
    }

    // Device sends 16-bit scroll delta in little-endian format
    let raw = i16::from_le_bytes([data[0], data[1]]);
    debug!(
        "Scroll dial 16-bit: raw bytes=[{:02x},{:02x}] -> i16={}",
        data[0], data[1], raw
    );

    // SPI Corruption Workaround:
    // The specific corruption observed matches Bit 0 of Byte 1 (the high byte) being cleared (0xFE instead of 0xFF).
    // This turns small negative numbers (e.g. -5 [0xFFFB]) into large negative spikes (-261 [0xFEFB]).
    // We detect this by checking if the high byte is 0xFE, and if restoring it to 0xFF makes the value
    // significantly closer to the previous value.
    // NOTE: This has been largely resolved by lowering the SPI clock speed to the cyw43.
    let mut clean_raw = raw;

    // Check if high byte is 0xFE (values within [-512, -257])
    if (raw & 0xFF00u16 as i16) == 0xFE00u16 as i16 {
        // Proposed fix: set bit 8 to restore 0xFF high byte
        let proposed = raw | 0x0100;
        let prev = PREV_SCROLL_RAW.load(Ordering::Relaxed) as i16;

        let diff_raw = (raw as i32 - prev as i32).abs();
        let diff_proposed = (proposed as i32 - prev as i32).abs();

        // If the proposed adjustment is closer to previous value, assume it's the correct one
        if diff_proposed < diff_raw {
            warn!(
                "SPI corruption detected: {} -> {} (delta {} vs {})",
                raw, proposed, diff_proposed, diff_raw
            );
            clean_raw = proposed;
        }
    }

    PREV_SCROLL_RAW.store(clean_raw as i32, Ordering::Relaxed);

    let wheel = if HIRES_SCROLL_ENABLED.load(Ordering::Relaxed) {
        // High-res: 120 units per detent.
        // When working correctly (handle 0x0020), device reports symmetric values
        // (±14 to ±38 for moderate scroll). Passthrough these values directly.
        //
        // NOTE: If asymmetric values return (~8 upward, ~-262 downward), check
        // the handle ID in logs. This indicates USB/BLE interference is either:
        // 1. Corrupting GATT discovery (wrong handle selected), or
        // 2. Corrupting notification data (if handle is still 0x0020)
        if clean_raw != raw {
            debug!(
                "16-bit mode (hires): {} passthrough (corrected to {})",
                raw, clean_raw
            );
        } else {
            debug!("16-bit mode (hires): {} passthrough", clean_raw);
        }
        clean_raw
    } else {
        // Standard mode with accumulator: sum raw values and emit the full
        // accumulated magnitude when the threshold is reached. Each emission
        // is >= 120 in magnitude, crossing macOS's scroll acceleration deadzone.
        // Without this, /120 produces mostly 0 with occasional +/-1 that macOS
        // filters out.
        let prev_acc = SCROLL_ACCUMULATOR.load(Ordering::Relaxed);
        let new_acc = prev_acc + clean_raw as i32;

        if new_acc.abs() >= SCROLL_ACCUMULATOR_THRESHOLD {
            // Emit the full accumulated value (preserves velocity info)
            let emit = new_acc.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            SCROLL_ACCUMULATOR.store(0, Ordering::Relaxed);
            debug!(
                "16-bit mode (standard/accum): acc={} -> emit {}",
                new_acc, emit
            );
            emit
        } else {
            SCROLL_ACCUMULATOR.store(new_acc, Ordering::Relaxed);
            debug!(
                "16-bit mode (standard/accum): acc={}, below threshold",
                new_acc
            );
            0
        }
    };

    MouseReport16 {
        buttons: 0,
        x: 0,
        y: 0,
        wheel,
        pan: 0,
    }
}

/// Generic fallback: standard 3-byte mouse (buttons, X, Y)
fn translate_generic(data: &[u8], len: usize) -> MouseReport {
    if len < 3 {
        return MouseReport {
            buttons: 0,
            x: 0,
            y: 0,
            wheel: 0,
            pan: 0,
        };
    }

    MouseReport {
        buttons: data[0],
        x: data[1] as i8,
        y: data[2] as i8,
        wheel: data.get(3).copied().unwrap_or(0) as i8,
        pan: data.get(4).copied().unwrap_or(0) as i8,
    }
}
