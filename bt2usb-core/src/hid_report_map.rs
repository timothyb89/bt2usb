//! Minimal HID Report Descriptor (Report Map) parser for BLE HID devices.
//!
//! Parses the HID Report Map characteristic (UUID 0x2A4B) to extract the
//! field layout of mouse Input reports. This enables the Generic device profile
//! to correctly interpret reports from any standard HID mouse without needing
//! device-specific profiles.
//!
//! The parser handles the subset of HID descriptor items needed for mice:
//! - Usage Page, Usage, Usage Min/Max (identify field purpose)
//! - Report Size, Report Count (field dimensions)
//! - Logical Min/Max (value range and signedness)
//! - Report ID (multi-report devices)
//! - Input items (mark data fields)
//! - Collection/End Collection (nesting)

// ============ Public Types ============

/// What a field represents from the perspective of mouse input.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FieldUsage {
    Buttons,
    X,
    Y,
    Wheel,
    Pan,
    #[allow(dead_code)]
    Padding,
    Other,
}

/// Location and format of a single field within a report.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FieldLocation {
    pub bit_offset: u16,
    pub bit_size: u16,
    pub is_signed: bool,
}

/// Compact layout for a single mouse report, extracted from the Report Map.
/// This is what the event loop and USB handler use to interpret incoming notifications.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MouseReportLayout {
    /// Report ID for the mouse report (0 if device doesn't use report IDs).
    pub report_id: u8,
    /// Whether the device uses report IDs (byte 0 of notification = report ID).
    pub has_report_ids: bool,
    /// Expected report data size in bytes (excluding report ID).
    pub total_bytes: u8,
    /// Button field (Usage Page 0x09).
    pub buttons: Option<FieldLocation>,
    /// X axis (Usage Page 0x01, Usage 0x30).
    pub x: Option<FieldLocation>,
    /// Y axis (Usage Page 0x01, Usage 0x31).
    pub y: Option<FieldLocation>,
    /// Scroll wheel (Usage Page 0x01, Usage 0x38).
    pub wheel: Option<FieldLocation>,
    /// Horizontal pan (Usage Page 0x0C, Usage 0x0238).
    pub pan: Option<FieldLocation>,
}

/// Maximum number of parsed report layouts stored per slot.
pub const MAX_LAYOUTS: usize = 4;

// ============ Bit-field Extraction ============

/// Extract a field value from a report byte buffer.
///
/// Handles arbitrary bit offsets and sizes up to 32 bits.
/// Sign-extends the result if `is_signed` is true.
pub fn extract_field(data: &[u8], loc: &FieldLocation) -> i32 {
    let bit_offset = loc.bit_offset as usize;
    let bit_size = loc.bit_size as usize;

    if bit_size == 0 || bit_size > 32 {
        return 0;
    }

    // Check if the field fits within the data
    let end_bit = bit_offset + bit_size;
    if end_bit.div_ceil(8) > data.len() {
        return 0;
    }

    // Extract bits across byte boundaries
    let mut value: u32 = 0;
    for i in 0..bit_size {
        let bit_pos = bit_offset + i;
        let byte_idx = bit_pos / 8;
        let bit_idx = bit_pos % 8;
        if data[byte_idx] & (1 << bit_idx) != 0 {
            value |= 1 << i;
        }
    }

    // Sign extension
    if loc.is_signed && bit_size < 32 {
        let sign_bit = 1u32 << (bit_size - 1);
        if value & sign_bit != 0 {
            // Extend sign to 32 bits
            value |= !((1u32 << bit_size) - 1);
        }
    }

    value as i32
}

// ============ Parser ============

/// Maximum number of usages we track per Input item.
const MAX_USAGES: usize = 16;

/// HID Global state (persists across Main items).
struct GlobalState {
    usage_page: u16,
    logical_min: i32,
    logical_max: i32,
    report_size: u16,
    report_count: u16,
    report_id: u8,
}

/// HID Local state (resets after each Main item).
struct LocalState {
    usages: [u16; MAX_USAGES],
    usage_count: usize,
    usage_min: u16,
    usage_max: u16,
    has_usage_range: bool,
}

impl LocalState {
    fn reset(&mut self) {
        self.usage_count = 0;
        self.usage_min = 0;
        self.usage_max = 0;
        self.has_usage_range = false;
    }
}

/// Candidate mouse report found during parsing.
struct CandidateReport {
    report_id: u8,
    buttons: Option<FieldLocation>,
    x: Option<FieldLocation>,
    y: Option<FieldLocation>,
    wheel: Option<FieldLocation>,
    pan: Option<FieldLocation>,
    mouse_field_count: u8,
}

impl CandidateReport {
    fn new(report_id: u8) -> Self {
        Self {
            report_id,
            buttons: None,
            x: None,
            y: None,
            wheel: None,
            pan: None,
            mouse_field_count: 0,
        }
    }
}

/// Maximum number of distinct report IDs we track.
const MAX_REPORTS: usize = 4;

/// Parse an HID Report Map descriptor and extract all mouse-like report layouts.
///
/// Returns an array of layouts (one per report ID that contains mouse fields).
/// The caller should match incoming notification data size against `total_bytes`
/// to select the correct layout at runtime.
///
/// Returns `None` if the descriptor is malformed or contains no mouse-like reports.
pub fn parse_report_map(data: &[u8]) -> Option<heapless::Vec<MouseReportLayout, MAX_LAYOUTS>> {
    let mut global = GlobalState {
        usage_page: 0,
        logical_min: 0,
        logical_max: 0,
        report_size: 0,
        report_count: 0,
        report_id: 0,
    };

    let mut local = LocalState {
        usages: [0; MAX_USAGES],
        usage_count: 0,
        usage_min: 0,
        usage_max: 0,
        has_usage_range: false,
    };

    let mut candidates: [Option<CandidateReport>; MAX_REPORTS] = [None, None, None, None];
    let mut has_report_ids = false;

    // Per-report-ID bit offset tracking
    let mut bit_offsets: [u16; MAX_REPORTS] = [0; MAX_REPORTS];
    let mut report_id_map: [u8; MAX_REPORTS] = [0; MAX_REPORTS]; // report_id → index
    let mut report_id_count: usize = 0;

    let mut pos = 0;
    while pos < data.len() {
        let prefix = data[pos];

        // Handle End Collection (special: 0 data bytes, encoded as 0xC0)
        if prefix == 0xC0 {
            pos += 1;
            continue;
        }

        // Parse item prefix
        let byte_count = match prefix & 0x03 {
            0 => 0,
            1 => 1,
            2 => 2,
            3 => 4, // "3" means 4 bytes per HID spec
            _ => unreachable!(),
        };
        let item_type = (prefix >> 2) & 0x03;
        let tag = (prefix >> 4) & 0x0F;

        // Check we have enough data
        if pos + 1 + byte_count > data.len() {
            #[cfg(feature = "defmt")]
            defmt::warn!("Report Map truncated at offset {}", pos);
            break;
        }

        let item_data = &data[pos + 1..pos + 1 + byte_count];

        // Read item value (little-endian, signed or unsigned depending on context)
        let value_u = match byte_count {
            0 => 0u32,
            1 => item_data[0] as u32,
            2 => u16::from_le_bytes([item_data[0], item_data[1]]) as u32,
            4 => u32::from_le_bytes([item_data[0], item_data[1], item_data[2], item_data[3]]),
            _ => 0,
        };
        let value_i = match byte_count {
            0 => 0i32,
            1 => item_data[0] as i8 as i32,
            2 => i16::from_le_bytes([item_data[0], item_data[1]]) as i32,
            4 => i32::from_le_bytes([item_data[0], item_data[1], item_data[2], item_data[3]]),
            _ => 0,
        };

        match item_type {
            // Global items (type = 1)
            1 => match tag {
                0x0 => global.usage_page = value_u as u16,  // Usage Page
                0x1 => global.logical_min = value_i,        // Logical Minimum
                0x2 => global.logical_max = value_i,        // Logical Maximum
                0x7 => global.report_size = value_u as u16, // Report Size
                0x8 => {
                    // Report ID
                    global.report_id = value_u as u8;
                    has_report_ids = true;
                }
                0x9 => global.report_count = value_u as u16, // Report Count
                _ => {} // Physical Min/Max, Unit, etc. — not needed
            },
            // Local items (type = 2)
            2 => match tag {
                0x0 => {
                    // Usage
                    if local.usage_count < MAX_USAGES {
                        local.usages[local.usage_count] = value_u as u16;
                        local.usage_count += 1;
                    }
                }
                0x1 => {
                    // Usage Minimum
                    local.usage_min = value_u as u16;
                    local.has_usage_range = true;
                }
                0x2 => {
                    // Usage Maximum
                    local.usage_max = value_u as u16;
                    local.has_usage_range = true;
                }
                _ => {} // Designator, String, etc. — not needed
            },
            // Main items (type = 0)
            0 => {
                match tag {
                    0x8 => {
                        // Input item
                        let is_constant = (value_u & 0x01) != 0;
                        let is_relative = (value_u & 0x04) != 0;

                        // Find or create candidate for this report ID
                        let rid_idx = find_or_create_report_id(
                            global.report_id,
                            &mut report_id_map,
                            &mut report_id_count,
                        );

                        if let Some(idx) = rid_idx {
                            let bit_offset = bit_offsets[idx];
                            let total_bits = global.report_size * global.report_count;

                            if !is_constant {
                                // Ensure candidate exists
                                if candidates[idx].is_none() {
                                    candidates[idx] = Some(CandidateReport::new(global.report_id));
                                }

                                if let Some(ref mut cand) = candidates[idx] {
                                    process_input_fields(
                                        cand,
                                        &global,
                                        &local,
                                        bit_offset,
                                        is_relative,
                                    );
                                }
                            }

                            bit_offsets[idx] += total_bits;
                        }
                    }
                    0xA => {} // Collection — track nesting if needed
                    _ => {}   // Output (0x9), Feature (0xB)
                }
                // Reset local state after Main item
                local.reset();
            }
            _ => {} // Reserved
        }

        pos += 1 + byte_count;
    }

    // Collect all candidates with at least one mouse-like field
    let mut result: heapless::Vec<MouseReportLayout, MAX_LAYOUTS> = heapless::Vec::new();

    for (i, cand) in candidates.iter().enumerate() {
        let cand = match cand {
            Some(c) if c.mouse_field_count > 0 => c,
            _ => continue,
        };

        let total_bits = bit_offsets[i];
        let total_bytes = total_bits.div_ceil(8) as u8;

        let layout = MouseReportLayout {
            report_id: cand.report_id,
            has_report_ids,
            total_bytes,
            buttons: cand.buttons,
            x: cand.x,
            y: cand.y,
            wheel: cand.wheel,
            pan: cand.pan,
        };

        #[cfg(feature = "defmt")]
        {
            defmt::debug!(
                "Report Map: report_id={}, {}B, fields={}",
                cand.report_id,
                total_bytes,
                cand.mouse_field_count
            );
            if let Some(ref f) = cand.buttons {
                defmt::debug!("  buttons: offset={} size={}", f.bit_offset, f.bit_size);
            }
            if let Some(ref f) = cand.x {
                defmt::debug!(
                    "  X: offset={} size={} signed={}",
                    f.bit_offset,
                    f.bit_size,
                    f.is_signed
                );
            }
            if let Some(ref f) = cand.y {
                defmt::debug!(
                    "  Y: offset={} size={} signed={}",
                    f.bit_offset,
                    f.bit_size,
                    f.is_signed
                );
            }
            if let Some(ref f) = cand.wheel {
                defmt::debug!(
                    "  wheel: offset={} size={} signed={}",
                    f.bit_offset,
                    f.bit_size,
                    f.is_signed
                );
            }
            if let Some(ref f) = cand.pan {
                defmt::debug!(
                    "  pan: offset={} size={} signed={}",
                    f.bit_offset,
                    f.bit_size,
                    f.is_signed
                );
            }
        }

        let _ = result.push(layout);
    }

    if result.is_empty() {
        #[cfg(feature = "defmt")]
        defmt::debug!("Report Map: no mouse-like fields found");
        return None;
    }

    Some(result)
}

/// Find the index for a report ID, or create a new slot.
fn find_or_create_report_id(
    report_id: u8,
    map: &mut [u8; MAX_REPORTS],
    count: &mut usize,
) -> Option<usize> {
    for (i, entry) in map.iter().enumerate().take(*count) {
        if *entry == report_id {
            return Some(i);
        }
    }
    if *count < MAX_REPORTS {
        map[*count] = report_id;
        *count += 1;
        Some(*count - 1)
    } else {
        None
    }
}

/// Process the fields described by a single Input item.
fn process_input_fields(
    cand: &mut CandidateReport,
    global: &GlobalState,
    local: &LocalState,
    base_bit_offset: u16,
    is_relative: bool,
) {
    let is_signed = global.logical_min < 0 || is_relative;

    if local.has_usage_range && global.usage_page == 0x09 {
        // Button range (Usage Page: Button, Usage Min..Max)
        let total_bits = global.report_size * global.report_count;
        let loc = FieldLocation {
            bit_offset: base_bit_offset,
            bit_size: total_bits,
            is_signed: false,
        };
        if cand.buttons.is_none() {
            cand.buttons = Some(loc);
            cand.mouse_field_count += 1;
        }
        return;
    }

    // Process individual usages
    let mut bit_offset = base_bit_offset;
    for i in 0..global.report_count as usize {
        let usage = if i < local.usage_count {
            local.usages[i]
        } else if local.usage_count > 0 {
            // Per HID spec: if fewer usages than report_count, last usage repeats
            local.usages[local.usage_count - 1]
        } else {
            0
        };

        let loc = FieldLocation {
            bit_offset,
            bit_size: global.report_size,
            is_signed,
        };

        let field_usage = classify_usage(global.usage_page, usage);
        match field_usage {
            FieldUsage::X if cand.x.is_none() => {
                cand.x = Some(loc);
                cand.mouse_field_count += 1;
            }
            FieldUsage::Y if cand.y.is_none() => {
                cand.y = Some(loc);
                cand.mouse_field_count += 1;
            }
            FieldUsage::Wheel if cand.wheel.is_none() => {
                cand.wheel = Some(loc);
                cand.mouse_field_count += 1;
            }
            FieldUsage::Pan if cand.pan.is_none() => {
                cand.pan = Some(loc);
                cand.mouse_field_count += 1;
            }
            FieldUsage::Buttons if cand.buttons.is_none() => {
                cand.buttons = Some(loc);
                cand.mouse_field_count += 1;
            }
            _ => {}
        }

        bit_offset += global.report_size;
    }
}

/// Map (usage_page, usage) to a FieldUsage.
fn classify_usage(usage_page: u16, usage: u16) -> FieldUsage {
    match usage_page {
        0x01 => match usage {
            // Generic Desktop
            0x30 => FieldUsage::X,
            0x31 => FieldUsage::Y,
            0x38 => FieldUsage::Wheel,
            _ => FieldUsage::Other,
        },
        0x09 => FieldUsage::Buttons, // Button page
        0x0C => match usage {
            // Consumer
            0x0238 => FieldUsage::Pan, // AC Pan
            _ => FieldUsage::Other,
        },
        _ => FieldUsage::Other,
    }
}
