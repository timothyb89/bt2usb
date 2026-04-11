use proptest::prelude::*;

use bt2usb_core::framing::{
    encode_frame, FrameAccumulator, MAX_FRAME_SIZE, MAX_MSG_SIZE, MAX_PAYLOAD_SIZE,
};
use bt2usb_core::hid_report_map::{extract_field, FieldLocation};
use bt2usb_core::interp::{decode_x, decode_y, InterpolationCore};
use bt2usb_core::mouse::{apply_multiplier_i16, apply_multiplier_i8};
use bt2usb_core::protocol::{decode_request, HEADER_SIZE, MSG_REQUEST};

// ============ extract_field properties ============

proptest! {
    #[test]
    fn extract_field_never_panics(
        data in prop::collection::vec(any::<u8>(), 0..32),
        bit_offset in 0u16..256,
        bit_size in 0u16..40,
        is_signed in any::<bool>(),
    ) {
        let loc = FieldLocation { bit_offset, bit_size, is_signed };
        // Should never panic, regardless of inputs
        let _ = extract_field(&data, &loc);
    }

    #[test]
    fn extract_unsigned_is_non_negative(
        data in prop::collection::vec(any::<u8>(), 1..16),
        bit_offset in 0u16..64,
        bit_size in 1u16..32,
    ) {
        let loc = FieldLocation { bit_offset, bit_size, is_signed: false };
        let val = extract_field(&data, &loc);
        // Unsigned extraction should never return a negative value
        // (when bit_size < 32 and is_signed is false)
        if bit_size < 32 {
            prop_assert!(val >= 0, "unsigned field returned {}", val);
        }
    }

    #[test]
    fn extract_single_byte_matches(byte_val in any::<u8>()) {
        // Extracting 8 unsigned bits from byte 0 should match the byte value
        let data = [byte_val];
        let loc = FieldLocation { bit_offset: 0, bit_size: 8, is_signed: false };
        let val = extract_field(&data, &loc);
        prop_assert_eq!(val, byte_val as i32);
    }

    #[test]
    fn extract_signed_byte_matches(byte_val in any::<u8>()) {
        // Extracting 8 signed bits from byte 0 should match i8 interpretation
        let data = [byte_val];
        let loc = FieldLocation { bit_offset: 0, bit_size: 8, is_signed: true };
        let val = extract_field(&data, &loc);
        prop_assert_eq!(val, byte_val as i8 as i32);
    }
}

// ============ framing properties ============

proptest! {
    #[test]
    fn framing_roundtrip(
        msg_type in 1u8..4,
        seq_id in any::<u16>(),
        body in prop::collection::vec(any::<u8>(), 0..MAX_MSG_SIZE),
    ) {
        let mut frame_buf = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(msg_type, seq_id, &body, &mut frame_buf);

        let mut acc = FrameAccumulator::new();
        let mut out = [0u8; MAX_PAYLOAD_SIZE];
        let (consumed, decoded) = acc.feed(&frame_buf[..frame_len], &mut out);

        prop_assert_eq!(consumed, frame_len);
        let decoded_len = decoded.unwrap();
        prop_assert_eq!(decoded_len, HEADER_SIZE + body.len());

        // Verify header
        let (dec_type, dec_seq) = bt2usb_core::protocol::decode_header(&out);
        prop_assert_eq!(dec_type, msg_type);
        prop_assert_eq!(dec_seq, seq_id);

        // Verify body
        prop_assert_eq!(&out[HEADER_SIZE..decoded_len], body.as_slice());
    }

    #[test]
    fn framing_split_at_any_point(
        body in prop::collection::vec(any::<u8>(), 1..64),
        split_frac in 0.0f64..1.0,
    ) {
        let mut frame_buf = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(MSG_REQUEST, 1, &body, &mut frame_buf);

        let split = (split_frac * frame_len as f64) as usize;
        let split = split.clamp(0, frame_len);

        let mut acc = FrameAccumulator::new();
        let mut out = [0u8; MAX_PAYLOAD_SIZE];

        // Feed first part
        let (_, r1) = acc.feed(&frame_buf[..split], &mut out);
        if r1.is_some() {
            // Frame completed in first part (split was after delimiter)
            return Ok(());
        }

        // Feed second part
        let (_, r2) = acc.feed(&frame_buf[split..frame_len], &mut out);
        let decoded_len = r2.unwrap();
        prop_assert_eq!(decoded_len, HEADER_SIZE + body.len());
    }

    #[test]
    fn accumulator_handles_junk_then_valid(
        junk in prop::collection::vec(1u8..=255, 0..200),
        body in prop::collection::vec(any::<u8>(), 1..32),
    ) {
        let mut acc = FrameAccumulator::new();
        let mut out = [0u8; MAX_PAYLOAD_SIZE];

        // Feed junk (no zeros = no delimiters)
        let _ = acc.feed(&junk, &mut out);
        // Delimiter to flush junk
        let _ = acc.feed(&[0x00], &mut out);

        // Now feed a valid frame
        let mut frame_buf = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(MSG_REQUEST, 42, &body, &mut frame_buf);
        let (_, result) = acc.feed(&frame_buf[..frame_len], &mut out);

        let decoded_len = result.unwrap();
        prop_assert_eq!(decoded_len, HEADER_SIZE + body.len());
    }
}

// ============ multiplier properties ============

proptest! {
    #[test]
    fn multiplier_i8_identity(value in any::<i8>()) {
        prop_assert_eq!(apply_multiplier_i8(value, 100), value);
    }

    #[test]
    fn multiplier_i8_zero_pct(value in any::<i8>()) {
        prop_assert_eq!(apply_multiplier_i8(value, 0), 0);
    }

    #[test]
    fn multiplier_i8_never_exceeds_range(
        value in any::<i8>(),
        pct in 0u32..1000,
    ) {
        let result = apply_multiplier_i8(value, pct);
        prop_assert!((-127..=127).contains(&result));
    }

    #[test]
    fn multiplier_i8_zero_input(pct in any::<u32>()) {
        prop_assert_eq!(apply_multiplier_i8(0, pct), 0);
    }

    #[test]
    fn multiplier_i16_identity(value in any::<i16>()) {
        prop_assert_eq!(apply_multiplier_i16(value, 100), value);
    }

    #[test]
    fn multiplier_i16_zero_pct(value in any::<i16>()) {
        prop_assert_eq!(apply_multiplier_i16(value, 0), 0);
    }

    #[test]
    fn multiplier_i16_never_exceeds_range(
        value in any::<i16>(),
        pct in 0u32..10000,
    ) {
        let result = apply_multiplier_i16(value, pct);
        prop_assert!(result >= i16::MIN && result <= i16::MAX);
    }

    #[test]
    fn multiplier_i16_zero_input(pct in any::<u32>()) {
        prop_assert_eq!(apply_multiplier_i16(0, pct), 0);
    }
}

// ============ touch decode properties ============

proptest! {
    #[test]
    fn decode_x_in_13bit_range(b0 in any::<u8>(), b1 in any::<u8>()) {
        let t = [b0, b1, 0, 0, 0, 0, 0, 0, 0];
        let x = decode_x(&t);
        prop_assert!(x >= -4096 && x <= 4095, "decode_x={} out of 13-bit range", x);
    }

    #[test]
    fn decode_y_in_13bit_range(b1 in any::<u8>(), b2 in any::<u8>(), b3 in any::<u8>()) {
        let t = [0, b1, b2, b3, 0, 0, 0, 0, 0];
        let y = decode_y(&t);
        prop_assert!(y >= -4096 && y <= 4095, "decode_y={} out of 13-bit range", y);
    }
}

// ============ InterpolationCore properties ============

fn arb_bt_report() -> impl Strategy<Value = Vec<u8>> {
    // 0-5 touch points, each 9 bytes, plus 4-byte header
    (any::<bool>(), 0usize..=5).prop_flat_map(|(click, n_points)| {
        prop::collection::vec(any::<u8>(), n_points * 9).prop_map(move |touch_data| {
            let mut report = vec![0x31, if click { 0x01 } else { 0x00 }, 0x00, 0x00];
            report.extend_from_slice(&touch_data);
            report
        })
    })
}

proptest! {
    #[test]
    fn interp_never_panics_on_valid_reports(reports in prop::collection::vec(arb_bt_report(), 1..20)) {
        let mut core = InterpolationCore::new();
        for report in &reports {
            let _ = core.receive_bt_report(report);
            // Tick a few times between reports
            for _ in 0..5 {
                let _ = core.tick();
            }
        }
    }

    #[test]
    fn interp_never_panics_on_random_data(data in prop::collection::vec(any::<u8>(), 0..64)) {
        let mut core = InterpolationCore::new();
        // Should gracefully reject invalid data without panicking
        let _ = core.receive_bt_report(&data);
        let _ = core.tick();
    }
}

// ============ protocol decode properties ============

proptest! {
    #[test]
    fn decode_request_never_panics(data in prop::collection::vec(any::<u8>(), 0..128)) {
        // Should never panic, even on random bytes
        let _ = decode_request(&data);
    }
}
