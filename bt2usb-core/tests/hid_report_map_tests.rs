use bt2usb_core::hid_report_map::{extract_field, parse_report_map, FieldLocation};

// ============ extract_field tests ============

#[test]
fn extract_byte_aligned_unsigned() {
    let data = [0xAB, 0xCD];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 8,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 0xAB);
}

#[test]
fn extract_byte_aligned_signed_negative() {
    let data = [0x80]; // -128 as i8
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 8,
        is_signed: true,
    };
    assert_eq!(extract_field(&data, &loc), -128);
}

#[test]
fn extract_byte_aligned_signed_positive() {
    let data = [0x7F]; // 127 as i8
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 8,
        is_signed: true,
    };
    assert_eq!(extract_field(&data, &loc), 127);
}

#[test]
fn extract_cross_byte_boundary() {
    // 12-bit value spanning bytes 0-1
    // Bits: byte0=0xF0 (11110000), byte1=0x0A (00001010)
    // 12 bits from offset 0: bits 0-7 from byte0 + bits 0-3 from byte1
    // = 11110000 | 1010 = 0xAF0
    let data = [0xF0, 0x0A];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 12,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 0xAF0);
}

#[test]
fn extract_cross_byte_signed_negative() {
    // 12-bit signed value = -1 (0xFFF)
    let data = [0xFF, 0x0F];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 12,
        is_signed: true,
    };
    assert_eq!(extract_field(&data, &loc), -1);
}

#[test]
fn extract_non_byte_aligned_offset() {
    // 4-bit field starting at bit offset 4
    // byte0=0xA5 (bits: 10100101), extract bits 4-7 = 1010 = 0x0A
    let data = [0xA5];
    let loc = FieldLocation {
        bit_offset: 4,
        bit_size: 4,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 0x0A);
}

#[test]
fn extract_single_bit() {
    let data = [0x04]; // bit 2 set
    let loc = FieldLocation {
        bit_offset: 2,
        bit_size: 1,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 1);

    let loc_off = FieldLocation {
        bit_offset: 3,
        bit_size: 1,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc_off), 0);
}

#[test]
fn extract_zero_size_returns_zero() {
    let data = [0xFF];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 0,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 0);
}

#[test]
fn extract_oversized_returns_zero() {
    let data = [0xFF];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 33,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 0);
}

#[test]
fn extract_out_of_bounds_returns_zero() {
    let data = [0xFF];
    let loc = FieldLocation {
        bit_offset: 4,
        bit_size: 8,
        is_signed: false,
    };
    // Would need bits 4-11 but only 8 bits available
    assert_eq!(extract_field(&data, &loc), 0);
}

#[test]
fn extract_16bit_signed() {
    // -256 as 16-bit signed = 0xFF00
    let data = [0x00, 0xFF];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 16,
        is_signed: true,
    };
    assert_eq!(extract_field(&data, &loc), -256);
}

#[test]
fn extract_buttons_3bit() {
    // 3 buttons: bits 0-2, value = 0b101 = buttons 1 and 3 pressed
    let data = [0x05, 0x00, 0x00, 0x00];
    let loc = FieldLocation {
        bit_offset: 0,
        bit_size: 3,
        is_signed: false,
    };
    assert_eq!(extract_field(&data, &loc), 5);
}

// ============ parse_report_map tests ============

/// Logitech MX Master 3S report descriptor (mouse portion).
/// Report ID 0x02: 16-bit buttons, 16-bit X, 16-bit Y, 8-bit wheel, 8-bit pan
const MX_MASTER_3S_REPORT_MAP: &[u8] = &[
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Mouse)
    0x09, 0x02,
    // Collection (Application)
    0xA1, 0x01,
    // Report ID (2)
    0x85, 0x02,
    // Usage (Pointer)
    0x09, 0x01,
    // Collection (Physical)
    0xA1, 0x00,
    // Usage Page (Button)
    0x05, 0x09,
    // Usage Minimum (1)
    0x19, 0x01,
    // Usage Maximum (16)
    0x29, 0x10,
    // Logical Minimum (0)
    0x15, 0x00,
    // Logical Maximum (1)
    0x25, 0x01,
    // Report Count (16)
    0x95, 0x10,
    // Report Size (1)
    0x75, 0x01,
    // Input (Data, Variable, Absolute)
    0x81, 0x02,
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (X)
    0x09, 0x30,
    // Usage (Y)
    0x09, 0x31,
    // Logical Minimum (-32767)
    0x16, 0x01, 0x80,
    // Logical Maximum (32767)
    0x26, 0xFF, 0x7F,
    // Report Count (2)
    0x95, 0x02,
    // Report Size (16)
    0x75, 0x10,
    // Input (Data, Variable, Relative)
    0x81, 0x06,
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Wheel)
    0x09, 0x38,
    // Logical Minimum (-127)
    0x15, 0x81,
    // Logical Maximum (127)
    0x25, 0x7F,
    // Report Count (1)
    0x95, 0x01,
    // Report Size (8)
    0x75, 0x08,
    // Input (Data, Variable, Relative)
    0x81, 0x06,
    // Usage Page (Consumer)
    0x05, 0x0C,
    // Usage (AC Pan)
    0x0A, 0x38, 0x02,
    // Report Count (1)
    0x95, 0x01,
    // Report Size (8)
    0x75, 0x08,
    // Input (Data, Variable, Relative)
    0x81, 0x06,
    // End Collection
    0xC0,
    // End Collection
    0xC0,
];

#[test]
fn parse_mx_master_3s() {
    let layouts = parse_report_map(MX_MASTER_3S_REPORT_MAP).expect("should parse");
    assert_eq!(layouts.len(), 1);

    let layout = &layouts[0];
    assert_eq!(layout.report_id, 2);
    assert!(layout.has_report_ids);

    // 16 button bits + 2×16 X/Y + 8 wheel + 8 pan = 16+32+8+8 = 64 bits = 8 bytes
    assert_eq!(layout.total_bytes, 8);

    let buttons = layout.buttons.expect("should have buttons");
    assert_eq!(buttons.bit_offset, 0);
    assert_eq!(buttons.bit_size, 16);
    assert!(!buttons.is_signed);

    let x = layout.x.expect("should have X");
    assert_eq!(x.bit_offset, 16);
    assert_eq!(x.bit_size, 16);
    assert!(x.is_signed);

    let y = layout.y.expect("should have Y");
    assert_eq!(y.bit_offset, 32);
    assert_eq!(y.bit_size, 16);
    assert!(y.is_signed);

    let wheel = layout.wheel.expect("should have wheel");
    assert_eq!(wheel.bit_offset, 48);
    assert_eq!(wheel.bit_size, 8);
    assert!(wheel.is_signed);

    let pan = layout.pan.expect("should have pan");
    assert_eq!(pan.bit_offset, 56);
    assert_eq!(pan.bit_size, 8);
    assert!(pan.is_signed);
}

/// Simple 3-button mouse: 3 buttons + 5 padding + 8-bit X + 8-bit Y + 8-bit wheel
/// No report IDs, single report.
const SIMPLE_MOUSE_REPORT_MAP: &[u8] = &[
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Mouse)
    0x09, 0x02,
    // Collection (Application)
    0xA1, 0x01,
    // Usage (Pointer)
    0x09, 0x01,
    // Collection (Physical)
    0xA1, 0x00,
    // Usage Page (Button)
    0x05, 0x09,
    // Usage Minimum (1)
    0x19, 0x01,
    // Usage Maximum (3)
    0x29, 0x03,
    // Logical Minimum (0)
    0x15, 0x00,
    // Logical Maximum (1)
    0x25, 0x01,
    // Report Count (3)
    0x95, 0x03,
    // Report Size (1)
    0x75, 0x01,
    // Input (Data, Variable, Absolute)
    0x81, 0x02,
    // Report Count (1)
    0x95, 0x01,
    // Report Size (5)
    0x75, 0x05,
    // Input (Constant) - padding
    0x81, 0x01,
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (X)
    0x09, 0x30,
    // Usage (Y)
    0x09, 0x31,
    // Usage (Wheel)
    0x09, 0x38,
    // Logical Minimum (-127)
    0x15, 0x81,
    // Logical Maximum (127)
    0x25, 0x7F,
    // Report Count (3)
    0x95, 0x03,
    // Report Size (8)
    0x75, 0x08,
    // Input (Data, Variable, Relative)
    0x81, 0x06,
    // End Collection
    0xC0,
    // End Collection
    0xC0,
];

#[test]
fn parse_simple_3button_mouse() {
    let layouts = parse_report_map(SIMPLE_MOUSE_REPORT_MAP).expect("should parse");
    assert_eq!(layouts.len(), 1);

    let layout = &layouts[0];
    assert_eq!(layout.report_id, 0);
    assert!(!layout.has_report_ids);
    // 3 buttons + 5 padding + 3×8 axes = 32 bits = 4 bytes
    assert_eq!(layout.total_bytes, 4);

    let buttons = layout.buttons.expect("should have buttons");
    assert_eq!(buttons.bit_offset, 0);
    assert_eq!(buttons.bit_size, 3);

    let x = layout.x.expect("should have X");
    assert_eq!(x.bit_offset, 8);
    assert_eq!(x.bit_size, 8);
    assert!(x.is_signed);

    let y = layout.y.expect("should have Y");
    assert_eq!(y.bit_offset, 16);
    assert_eq!(y.bit_size, 8);

    let wheel = layout.wheel.expect("should have wheel");
    assert_eq!(wheel.bit_offset, 24);
    assert_eq!(wheel.bit_size, 8);

    assert!(layout.pan.is_none());
}

#[test]
fn parse_extract_roundtrip() {
    // Parse the MX Master 3S descriptor, then extract from a sample report
    let layouts = parse_report_map(MX_MASTER_3S_REPORT_MAP).unwrap();
    let layout = &layouts[0];

    // Construct a report: buttons=0x0005 (btn 1+3), X=-10, Y=20, wheel=-1, pan=3
    let mut report = [0u8; 8];
    // Buttons: 16-bit LE at offset 0
    report[0] = 0x05;
    report[1] = 0x00;
    // X: 16-bit LE signed = -10 = 0xFFF6
    report[2] = 0xF6;
    report[3] = 0xFF;
    // Y: 16-bit LE signed = 20
    report[4] = 0x14;
    report[5] = 0x00;
    // Wheel: 8-bit signed = -1 = 0xFF
    report[6] = 0xFF;
    // Pan: 8-bit signed = 3
    report[7] = 0x03;

    assert_eq!(
        extract_field(&report, layout.buttons.as_ref().unwrap()),
        0x0005
    );
    assert_eq!(extract_field(&report, layout.x.as_ref().unwrap()), -10);
    assert_eq!(extract_field(&report, layout.y.as_ref().unwrap()), 20);
    assert_eq!(extract_field(&report, layout.wheel.as_ref().unwrap()), -1);
    assert_eq!(extract_field(&report, layout.pan.as_ref().unwrap()), 3);
}

#[test]
fn parse_empty_returns_none() {
    assert!(parse_report_map(&[]).is_none());
}

#[test]
fn parse_keyboard_only_returns_none() {
    // A keyboard-only descriptor (Usage Page 0x07, no mouse fields)
    let desc = &[
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x06, // Usage (Keyboard)
        0xA1, 0x01, // Collection (Application)
        0x05, 0x07, // Usage Page (Keyboard/Keypad)
        0x19, 0xE0, // Usage Min (Left Control)
        0x29, 0xE7, // Usage Max (Right GUI)
        0x15, 0x00, // Logical Min (0)
        0x25, 0x01, // Logical Max (1)
        0x75, 0x01, // Report Size (1)
        0x95, 0x08, // Report Count (8)
        0x81, 0x02, // Input (Data, Variable, Absolute)
        0xC0, // End Collection
    ];
    assert!(parse_report_map(desc).is_none());
}
