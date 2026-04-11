//! Mouse report types and multiplier helpers.
//!
//! Provides the 16-bit mouse report struct and axis multiplier functions
//! used by both the USB HID handler and the device profile translators.

/// Mouse report with 16-bit wheel and pan for high-resolution scrolling.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MouseReport16 {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
    pub wheel: i16,
    pub pan: i16,
}

/// Apply a percentage multiplier to an i8 value, clamping to i8 range.
pub fn apply_multiplier_i8(value: i8, multiplier_pct: u32) -> i8 {
    if multiplier_pct == 100 {
        return value;
    }
    let scaled = (value as i32) * (multiplier_pct as i32) / 100;
    scaled.clamp(-127, 127) as i8
}

/// Apply a percentage multiplier to an i16 value, clamping to i16 range.
pub fn apply_multiplier_i16(value: i16, multiplier_pct: u32) -> i16 {
    if multiplier_pct == 100 {
        return value;
    }
    let scaled = (value as i32) * (multiplier_pct as i32) / 100;
    scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}
