use bt2usb_core::framing::{encode_frame, FrameAccumulator, MAX_FRAME_SIZE, MAX_PAYLOAD_SIZE};
use bt2usb_core::protocol::{decode_header, HEADER_SIZE, MSG_REQUEST};

#[test]
fn encode_decode_roundtrip() {
    let cbor_body = [0x01, 0x02, 0x03];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let frame_len = encode_frame(MSG_REQUEST, 42, &cbor_body, &mut frame_buf);

    assert!(frame_len > 0);
    // Last byte is the 0x00 delimiter
    assert_eq!(frame_buf[frame_len - 1], 0x00);

    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];
    let (consumed, decoded) = acc.feed(&frame_buf[..frame_len], &mut out);
    assert_eq!(consumed, frame_len);
    let decoded_len = decoded.expect("should decode a complete frame");

    // Decoded payload = header + cbor_body
    assert_eq!(decoded_len, HEADER_SIZE + cbor_body.len());

    let (msg_type, seq_id) = decode_header(&out);
    assert_eq!(msg_type, MSG_REQUEST);
    assert_eq!(seq_id, 42);
    assert_eq!(&out[HEADER_SIZE..decoded_len], &cbor_body);
}

#[test]
fn partial_frame_across_feeds() {
    let cbor_body = [0xAA, 0xBB];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let frame_len = encode_frame(MSG_REQUEST, 100, &cbor_body, &mut frame_buf);

    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];

    // Feed first half — no complete frame yet
    let mid = frame_len / 2;
    let (consumed1, result1) = acc.feed(&frame_buf[..mid], &mut out);
    assert_eq!(consumed1, mid);
    assert!(result1.is_none());

    // Feed second half — should complete the frame
    let (consumed2, result2) = acc.feed(&frame_buf[mid..frame_len], &mut out);
    assert_eq!(consumed2, frame_len - mid);
    let decoded_len = result2.expect("should decode after second feed");
    assert_eq!(decoded_len, HEADER_SIZE + cbor_body.len());

    let (msg_type, seq_id) = decode_header(&out);
    assert_eq!(msg_type, MSG_REQUEST);
    assert_eq!(seq_id, 100);
}

#[test]
fn multiple_frames_in_one_buffer() {
    let mut frame_buf = [0u8; MAX_FRAME_SIZE * 3];
    let body1 = [0x01];
    let body2 = [0x02];

    let len1 = encode_frame(MSG_REQUEST, 1, &body1, &mut frame_buf);
    let len2 = encode_frame(MSG_REQUEST, 2, &body2, &mut frame_buf[len1..]);
    let total = len1 + len2;

    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];

    // First feed should yield first frame
    let (consumed1, result1) = acc.feed(&frame_buf[..total], &mut out);
    let decoded_len1 = result1.expect("should decode first frame");
    let (_, seq1) = decode_header(&out);
    assert_eq!(seq1, 1);
    assert_eq!(decoded_len1, HEADER_SIZE + 1);

    // Feed remaining — should yield second frame
    let (consumed2, result2) = acc.feed(&frame_buf[consumed1..total], &mut out);
    let _ = consumed2;
    let decoded_len2 = result2.expect("should decode second frame");
    let (_, seq2) = decode_header(&out);
    assert_eq!(seq2, 2);
    assert_eq!(decoded_len2, HEADER_SIZE + 1);
}

#[test]
fn leading_zeros_are_ignored() {
    // Leading 0x00 delimiters with no preceding data should be skipped
    let body = [0x42];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let frame_len = encode_frame(MSG_REQUEST, 7, &body, &mut frame_buf);

    // Prepend some zero bytes
    let mut with_leading = [0u8; MAX_FRAME_SIZE + 4];
    with_leading[0] = 0x00;
    with_leading[1] = 0x00;
    with_leading[2] = 0x00;
    with_leading[3..3 + frame_len].copy_from_slice(&frame_buf[..frame_len]);
    let total = 3 + frame_len;

    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];

    // Feed all at once — the leading zeros should be consumed, then the frame decoded
    let mut offset = 0;
    let mut decoded = None;
    while offset < total && decoded.is_none() {
        let (consumed, result) = acc.feed(&with_leading[offset..total], &mut out);
        offset += consumed;
        decoded = result;
    }
    assert!(decoded.is_some());
    let (_, seq) = decode_header(&out);
    assert_eq!(seq, 7);
}

#[test]
fn buffer_overflow_discards_and_recovers() {
    // Feed more bytes than MAX_FRAME_SIZE without a delimiter,
    // then a valid frame — the accumulator should discard the overflow
    // and recover to decode the valid frame.
    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];

    // Feed a bunch of non-zero bytes (no delimiter) exceeding buffer
    let junk = [0xAA; MAX_FRAME_SIZE + 100];
    let (consumed, result) = acc.feed(&junk, &mut out);
    assert_eq!(consumed, junk.len());
    assert!(result.is_none());

    // Now send a delimiter to flush the junk
    let (_, result) = acc.feed(&[0x00], &mut out);
    // Decode will fail because the junk is invalid COBS, but the
    // accumulator should reset
    assert!(result.is_none());

    // Now send a valid frame
    let body = [0x55];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let frame_len = encode_frame(MSG_REQUEST, 99, &body, &mut frame_buf);
    let (_, result) = acc.feed(&frame_buf[..frame_len], &mut out);
    let decoded_len = result.expect("should recover and decode");
    assert_eq!(decoded_len, HEADER_SIZE + 1);
    let (_, seq) = decode_header(&out);
    assert_eq!(seq, 99);
}

#[test]
fn empty_feed_returns_none() {
    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];
    let (consumed, result) = acc.feed(&[], &mut out);
    assert_eq!(consumed, 0);
    assert!(result.is_none());
}

#[test]
fn byte_by_byte_feed() {
    let body = [0x10, 0x20, 0x30];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let frame_len = encode_frame(MSG_REQUEST, 55, &body, &mut frame_buf);

    let mut acc = FrameAccumulator::new();
    let mut out = [0u8; MAX_PAYLOAD_SIZE];

    let mut decoded = None;
    for i in 0..frame_len {
        let (_, result) = acc.feed(&frame_buf[i..i + 1], &mut out);
        if result.is_some() {
            decoded = result;
            break;
        }
    }

    let decoded_len = decoded.expect("should decode frame fed byte by byte");
    assert_eq!(decoded_len, HEADER_SIZE + body.len());
    let (_, seq) = decode_header(&out);
    assert_eq!(seq, 55);
}
