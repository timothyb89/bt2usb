use bt2usb_core::protocol::*;

// ============ Header encode/decode ============

#[test]
fn header_roundtrip() {
    let mut buf = [0u8; 3];
    encode_header(&mut buf, MSG_REQUEST, 0x1234);
    let (msg_type, seq_id) = decode_header(&buf);
    assert_eq!(msg_type, MSG_REQUEST);
    assert_eq!(seq_id, 0x1234);
}

#[test]
fn header_zero_seq() {
    let mut buf = [0u8; 3];
    encode_header(&mut buf, MSG_EVENT, 0);
    let (msg_type, seq_id) = decode_header(&buf);
    assert_eq!(msg_type, MSG_EVENT);
    assert_eq!(seq_id, 0);
}

#[test]
fn header_max_seq() {
    let mut buf = [0u8; 3];
    encode_header(&mut buf, MSG_RESPONSE, 0xFFFF);
    let (msg_type, seq_id) = decode_header(&buf);
    assert_eq!(msg_type, MSG_RESPONSE);
    assert_eq!(seq_id, 0xFFFF);
}

// ============ Request decode ============

fn encode_cbor(
    f: impl FnOnce(&mut minicbor::Encoder<&mut minicbor::encode::write::Cursor<&mut [u8]>>),
) -> Vec<u8> {
    let mut buf = [0u8; 256];
    let mut cursor = minicbor::encode::write::Cursor::new(&mut buf[..]);
    {
        let mut e = minicbor::Encoder::new(&mut cursor);
        f(&mut e);
    }
    let len = cursor.position();
    buf[..len].to_vec()
}

#[test]
fn decode_get_status() {
    let cbor = encode_cbor(|e| {
        e.array(1).unwrap().u8(CMD_GET_STATUS).unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::GetStatus => {}
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_start_scan() {
    let cbor = encode_cbor(|e| {
        e.array(1).unwrap().u8(CMD_START_SCAN).unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::StartScan => {}
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_connect_full() {
    let addr = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    let cbor = encode_cbor(|e| {
        e.array(5)
            .unwrap()
            .u8(CMD_CONNECT)
            .unwrap()
            .bytes(&addr)
            .unwrap()
            .u8(1)
            .unwrap()
            .bool(true)
            .unwrap()
            .u8(1)
            .unwrap(); // Classic
    });
    match decode_request(&cbor).unwrap() {
        Request::Connect {
            address,
            addr_kind,
            ignore_bond,
            transport_type,
        } => {
            assert_eq!(address, addr);
            assert_eq!(addr_kind, 1);
            assert!(ignore_bond);
            assert_eq!(transport_type, 1);
        }
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_connect_backward_compat() {
    // Older clients don't send ignore_bond or transport_type
    let addr = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    let cbor = encode_cbor(|e| {
        e.array(3)
            .unwrap()
            .u8(CMD_CONNECT)
            .unwrap()
            .bytes(&addr)
            .unwrap()
            .u8(0)
            .unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::Connect {
            address,
            addr_kind,
            ignore_bond,
            transport_type,
        } => {
            assert_eq!(address, addr);
            assert_eq!(addr_kind, 0);
            assert!(!ignore_bond);
            assert_eq!(transport_type, 0);
        }
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_disconnect_with_address() {
    let addr = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
    let cbor = encode_cbor(|e| {
        e.array(2)
            .unwrap()
            .u8(CMD_DISCONNECT)
            .unwrap()
            .bytes(&addr)
            .unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::Disconnect { address } => {
            assert_eq!(address, Some(addr));
        }
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_disconnect_no_address() {
    let cbor = encode_cbor(|e| {
        e.array(1).unwrap().u8(CMD_DISCONNECT).unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::Disconnect { address } => {
            assert_eq!(address, None);
        }
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_set_config() {
    let cbor = encode_cbor(|e| {
        e.array(3)
            .unwrap()
            .u8(CMD_SET_CONFIG)
            .unwrap()
            .u8(5)
            .unwrap()
            .u32(12345)
            .unwrap();
    });
    match decode_request(&cbor).unwrap() {
        Request::SetConfig { key, value } => {
            assert_eq!(key, 5);
            assert_eq!(value, 12345);
        }
        other => panic!("unexpected: {:?}", other),
    }
}

#[test]
fn decode_unknown_command() {
    let cbor = encode_cbor(|e| {
        e.array(1).unwrap().u8(0xFF).unwrap();
    });
    match decode_request(&cbor) {
        Err(ProtocolError::UnknownCommand(0xFF)) => {}
        other => panic!("expected UnknownCommand(0xFF), got: {:?}", other),
    }
}

#[test]
fn decode_invalid_cbor() {
    let bad = [0xFF, 0xFF, 0xFF];
    match decode_request(&bad) {
        Err(ProtocolError::InvalidCbor) => {}
        other => panic!("expected InvalidCbor, got: {:?}", other),
    }
}

// ============ Response encoding ============

#[test]
fn encode_ok_response() {
    let mut buf = [0u8; 64];
    let len = encode_response_ok(&mut buf).unwrap();
    assert!(len > 0);

    // Decode and verify
    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 1);
    assert_eq!(d.u8().unwrap(), RESP_OK);
}

#[test]
fn encode_error_response() {
    let mut buf = [0u8; 128];
    let len = encode_response_error(&mut buf, 42, "something failed").unwrap();
    assert!(len > 0);

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 3);
    assert_eq!(d.u8().unwrap(), RESP_ERROR);
    assert_eq!(d.u8().unwrap(), 42);
    assert_eq!(d.str().unwrap(), "something failed");
}

#[test]
fn encode_version_response() {
    let mut buf = [0u8; 64];
    let len = encode_response_version(&mut buf, "1.2.3").unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 2);
    assert_eq!(d.u8().unwrap(), RESP_VERSION);
    assert_eq!(d.str().unwrap(), "1.2.3");
}

#[test]
fn encode_bonds_response() {
    let bonds: &[([u8; 6], u8, u8, &str, bool, u8)] = &[
        ([1, 2, 3, 4, 5, 6], 0, 2, "Mouse A", true, 0),
        ([7, 8, 9, 10, 11, 12], 1, 3, "Mouse B", false, 1),
    ];
    let mut buf = [0u8; 256];
    let len = encode_response_bonds(&mut buf, bonds).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 2);
    assert_eq!(d.u8().unwrap(), RESP_BONDS);
    let bonds_arr = d.array().unwrap().unwrap();
    assert_eq!(bonds_arr, 2);

    // First bond
    let _ = d.array().unwrap();
    let addr = d.bytes().unwrap();
    assert_eq!(addr, &[1, 2, 3, 4, 5, 6]);
    assert_eq!(d.u8().unwrap(), 0); // addr_kind
    assert_eq!(d.u8().unwrap(), 2); // profile_id
    assert_eq!(d.str().unwrap(), "Mouse A");
    assert!(d.bool().unwrap()); // auto_connect
    assert_eq!(d.u8().unwrap(), 0); // transport (BLE)
}

#[test]
fn encode_config_response() {
    let mut buf = [0u8; 64];
    let len = encode_response_config(&mut buf, 100, 200, 300, 400, 10, 5).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 7);
    assert_eq!(d.u8().unwrap(), RESP_CONFIG);
    assert_eq!(d.u32().unwrap(), 100);
    assert_eq!(d.u32().unwrap(), 200);
    assert_eq!(d.u32().unwrap(), 300);
    assert_eq!(d.u32().unwrap(), 400);
    assert_eq!(d.u32().unwrap(), 10);
    assert_eq!(d.u32().unwrap(), 5);
}

#[test]
fn encode_status_response_no_devices() {
    let empty = [None, None, None, None];
    let addr = [0u8; 6];
    let mut buf = [0u8; 128];
    let len = encode_response_status(
        &mut buf,
        ConnectionState::Disconnected,
        0,
        0,
        false,
        &addr,
        0xFF,
        0,
        0,
        &empty,
    )
    .unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let arr_len = d.array().unwrap().unwrap();
    assert_eq!(arr_len, 10);
    assert_eq!(d.u8().unwrap(), RESP_STATUS);
    assert_eq!(d.u8().unwrap(), ConnectionState::Disconnected as u8);
    assert_eq!(d.u8().unwrap(), 0); // bonded_count
    assert_eq!(d.u8().unwrap(), 0); // active_profile
    assert!(!d.bool().unwrap()); // active_device_set
    let _ = d.bytes().unwrap(); // active_device_address
    assert_eq!(d.u8().unwrap(), 0xFF); // battery_level
    assert_eq!(d.u8().unwrap(), 0); // detected_os
    assert_eq!(d.u8().unwrap(), 0); // connected_count
    let dev_arr = d.array().unwrap().unwrap();
    assert_eq!(dev_arr, 0);
}

#[test]
fn encode_status_response_with_device() {
    let dev = ConnectedDevice {
        address: [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
        profile_id: 2,
        battery_level: 85,
        transport_type: 0,
    };
    let devs = [Some(dev), None, None, None];
    let addr = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    let mut buf = [0u8; 256];
    let len = encode_response_status(
        &mut buf,
        ConnectionState::Ready,
        1,
        2,
        true,
        &addr,
        85,
        3,
        1,
        &devs,
    )
    .unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), RESP_STATUS);
    assert_eq!(d.u8().unwrap(), ConnectionState::Ready as u8);
    assert_eq!(d.u8().unwrap(), 1); // bonded_count
    assert_eq!(d.u8().unwrap(), 2); // active_profile
    assert!(d.bool().unwrap()); // active_device_set
    assert_eq!(d.bytes().unwrap(), &addr);
    assert_eq!(d.u8().unwrap(), 85); // battery
    assert_eq!(d.u8().unwrap(), 3); // detected_os
    assert_eq!(d.u8().unwrap(), 1); // connected_count
    let dev_arr = d.array().unwrap().unwrap();
    assert_eq!(dev_arr, 1);
    let _ = d.array().unwrap();
    assert_eq!(d.bytes().unwrap(), &addr);
    assert_eq!(d.u8().unwrap(), 2); // profile
    assert_eq!(d.u8().unwrap(), 85); // battery
    assert_eq!(d.u8().unwrap(), 0); // transport
}

// ============ Event encoding ============

#[test]
fn encode_scan_result_event() {
    let addr = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    let mut buf = [0u8; 128];
    let len = encode_event_scan_result(&mut buf, &addr, 1, "My Mouse", -50, true, 0).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_SCAN_RESULT);
    assert_eq!(d.bytes().unwrap(), &addr);
    assert_eq!(d.u8().unwrap(), 1); // addr_kind
    assert_eq!(d.str().unwrap(), "My Mouse");
    assert_eq!(d.i8().unwrap(), -50); // rssi
    assert!(d.bool().unwrap()); // is_hid
    assert_eq!(d.u8().unwrap(), 0); // transport
}

#[test]
fn encode_connection_state_event() {
    let addr = [0x01; 6];
    let mut buf = [0u8; 64];
    let len = encode_event_connection_state(&mut buf, ConnectionState::Connected, &addr).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_CONNECTION_STATE);
    assert_eq!(d.u8().unwrap(), ConnectionState::Connected as u8);
    assert_eq!(d.bytes().unwrap(), &addr);
}

#[test]
fn encode_pairing_status_event() {
    let mut buf = [0u8; 32];
    let len = encode_event_pairing_status(&mut buf, PairingState::Complete).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_PAIRING_STATUS);
    assert_eq!(d.u8().unwrap(), PairingState::Complete as u8);
}

#[test]
fn encode_battery_level_event() {
    let mut buf = [0u8; 16];
    let len = encode_event_battery_level(&mut buf, 75).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_BATTERY_LEVEL);
    assert_eq!(d.u8().unwrap(), 75);
}

#[test]
fn encode_log_event() {
    let mut buf = [0u8; 128];
    let len = encode_event_log(&mut buf, 2, "hello world").unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_LOG);
    assert_eq!(d.u8().unwrap(), 2);
    assert_eq!(d.str().unwrap(), "hello world");
}

#[test]
fn encode_bond_stored_event() {
    let addr = [0xAA; 6];
    let mut buf = [0u8; 32];
    let len = encode_event_bond_stored(&mut buf, &addr, 3).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_BOND_STORED);
    assert_eq!(d.bytes().unwrap(), &addr);
    assert_eq!(d.u8().unwrap(), 3);
}

#[test]
fn encode_defmt_event() {
    let frame = [0x01, 0x02, 0x03, 0x04];
    let mut buf = [0u8; 32];
    let len = encode_event_defmt(&mut buf, &frame).unwrap();

    let mut d = minicbor::Decoder::new(&buf[..len]);
    let _ = d.array().unwrap();
    assert_eq!(d.u8().unwrap(), EVT_DEFMT);
    assert_eq!(d.bytes().unwrap(), &frame);
}
