//! RPC handler over USB HID vendor interface
//!
//! Decodes COBS-framed CBOR requests from the host, dispatches commands
//! to the BLE state machine, and encodes responses + events back.
//! Also forwards log events when a client subscribes.

use defmt::*;
use embassy_futures::select::{select3, Either3};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_usb::class::hid::{HidReader, HidWriter};

use crate::ble_state::{
    BleCommand, BleEvent, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL, BONDS_RESPONSE_CHANNEL,
    STATUS_RESPONSE_CHANNEL,
};
use crate::framing::{self, FrameAccumulator, MAX_FRAME_SIZE, MAX_PAYLOAD_SIZE};
use crate::protocol::{self, ConnectionState, HEADER_SIZE, MSG_EVENT, MSG_REQUEST, MSG_RESPONSE};
use crate::rpc_log::LOG_CHANNEL;

const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Send a complete COBS frame over HID, splitting into 64-byte reports as needed.
/// Each HID report is zero-padded to exactly 64 bytes. Trailing zeros are harmless
/// because the COBS decoder treats 0x00 as a frame delimiter (empty frames are skipped).
async fn send_frame(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 64>,
    frame: &[u8],
) -> Result<(), embassy_usb::driver::EndpointError> {
    for chunk in frame.chunks(64) {
        let mut buf = [0u8; 64];
        buf[..chunk.len()].copy_from_slice(chunk);
        writer.write(&buf).await?;
    }
    Ok(())
}

/// Encode a response and send it as a COBS frame.
async fn send_response(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 64>,
    seq_id: u16,
    cbor_buf: &[u8],
    frame_buf: &mut [u8],
    cbor_len: usize,
) {
    let frame_len = framing::encode_frame(MSG_RESPONSE, seq_id, &cbor_buf[..cbor_len], frame_buf);
    if let Err(e) = send_frame(writer, &frame_buf[..frame_len]).await {
        warn!("HID TX error: {:?}", e);
    }
}

/// Encode an event and send it as a COBS frame (seq_id = 0).
async fn send_event(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 64>,
    cbor_buf: &[u8],
    frame_buf: &mut [u8],
    cbor_len: usize,
) {
    let frame_len = framing::encode_frame(MSG_EVENT, 0, &cbor_buf[..cbor_len], frame_buf);
    if let Err(e) = send_frame(writer, &frame_buf[..frame_len]).await {
        warn!("HID TX error: {:?}", e);
    }
}

/// RPC handler task - decodes requests, dispatches to BLE, encodes responses + events
#[embassy_executor::task]
pub async fn rpc_task(
    mut writer: HidWriter<'static, Driver<'static, USB>, 64>,
    mut reader: HidReader<'static, Driver<'static, USB>, 64>,
) {
    info!("RPC task started (HID vendor interface)");

    let mut accumulator = FrameAccumulator::new();
    let mut rx_buf = [0u8; 64];
    let mut decoded_buf = [0u8; MAX_PAYLOAD_SIZE];
    let mut cbor_buf = [0u8; 256];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
    let mut last_state = ConnectionState::Disconnected;
    let mut log_subscribed = false;
    let mut log_min_level: u8 = 0;

    loop {
        match select3(
            reader.read(&mut rx_buf),
            BLE_EVENT_CHANNEL.receive(),
            LOG_CHANNEL.receive(),
        )
        .await
        {
            // ── HID RX: decode COBS frames and dispatch requests ──
            Either3::First(Ok(n)) if n > 0 => {
                let mut offset = 0;
                while offset < n {
                    let (consumed, frame) = accumulator.feed(&rx_buf[offset..n], &mut decoded_buf);
                    offset += consumed;

                    if let Some(decoded_len) = frame {
                        if decoded_len < HEADER_SIZE {
                            debug!("Frame too short: {} bytes", decoded_len);
                            continue;
                        }

                        let (msg_type, seq_id) = protocol::decode_header(&decoded_buf);
                        let cbor_body = &decoded_buf[HEADER_SIZE..decoded_len];

                        if msg_type == MSG_REQUEST {
                            match protocol::decode_request(cbor_body) {
                                Ok(request) => {
                                    debug!("RPC request: {:?}", request);

                                    // Handle subscription state changes
                                    if let protocol::Request::SubscribeLogs { level } = &request {
                                        log_subscribed = true;
                                        log_min_level = *level;
                                        info!("Log subscription: level >= {}", log_min_level);
                                    }
                                    if matches!(request, protocol::Request::UnsubscribeLogs) {
                                        log_subscribed = false;
                                        info!("Log subscription: off");
                                    }

                                    dispatch_request(
                                        &mut writer,
                                        seq_id,
                                        &request,
                                        last_state,
                                        &mut cbor_buf,
                                        &mut frame_buf,
                                    )
                                    .await;
                                }
                                Err(_) => {
                                    debug!("Failed to decode CBOR request");
                                    if let Ok(len) = protocol::encode_response_error(
                                        &mut cbor_buf,
                                        1,
                                        "invalid request",
                                    ) {
                                        send_response(
                                            &mut writer,
                                            seq_id,
                                            &cbor_buf,
                                            &mut frame_buf,
                                            len,
                                        )
                                        .await;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            Either3::First(Ok(_)) => {} // 0-byte read, ignore
            Either3::First(Err(e)) => {
                warn!("HID read error: {:?}", e);
                // USB not configured or endpoint error — wait briefly before retrying
                embassy_time::Timer::after_millis(100).await;
            }

            // ── BLE events: encode and forward to host ──
            Either3::Second(event) => {
                let cbor_len = encode_ble_event(&event, &mut cbor_buf, &mut last_state);
                if cbor_len > 0 {
                    send_event(&mut writer, &cbor_buf, &mut frame_buf, cbor_len).await;
                }
            }

            // ── Log events: forward if subscribed ──
            Either3::Third(log_event) => {
                if log_subscribed && log_event.level >= log_min_level {
                    let msg = core::str::from_utf8(&log_event.message[..log_event.len as usize])
                        .unwrap_or("?");
                    if let Ok(len) = protocol::encode_event_log(&mut cbor_buf, log_event.level, msg)
                    {
                        send_event(&mut writer, &cbor_buf, &mut frame_buf, len).await;
                    }
                }
                // If not subscribed, the log event is silently discarded
            }
        }
    }
}

/// Encode a BLE event into CBOR. Updates `last_state` for StateChanged events.
/// Returns the CBOR body length, or 0 on encoding failure.
fn encode_ble_event(
    event: &BleEvent,
    cbor_buf: &mut [u8],
    last_state: &mut ConnectionState,
) -> usize {
    match event {
        BleEvent::ScanResult(data) => {
            let name = core::str::from_utf8(&data.name[..data.name_len as usize]).unwrap_or("?");
            protocol::encode_event_scan_result(
                cbor_buf,
                &data.address,
                data.addr_kind,
                name,
                data.rssi,
                data.is_hid,
            )
            .unwrap_or(0)
        }
        BleEvent::StateChanged(state) => {
            *last_state = *state;
            protocol::encode_event_connection_state(cbor_buf, *state, &[0; 6]).unwrap_or(0)
        }
        BleEvent::PairingComplete => {
            protocol::encode_event_pairing_status(cbor_buf, protocol::PairingState::Complete)
                .unwrap_or(0)
        }
        BleEvent::PairingFailed => {
            protocol::encode_event_pairing_status(cbor_buf, protocol::PairingState::Failed)
                .unwrap_or(0)
        }
        BleEvent::BondStored {
            address,
            profile_id,
        } => protocol::encode_event_bond_stored(cbor_buf, address, *profile_id).unwrap_or(0),
        BleEvent::BatteryLevel(level) => {
            protocol::encode_event_battery_level(cbor_buf, *level).unwrap_or(0)
        }
    }
}

/// Dispatch a decoded request: forward to BLE state machine and encode response.
async fn dispatch_request(
    writer: &mut HidWriter<'static, Driver<'static, USB>, 64>,
    seq_id: u16,
    request: &protocol::Request,
    last_state: ConnectionState,
    cbor_buf: &mut [u8],
    frame_buf: &mut [u8],
) {
    let cbor_len = match request {
        protocol::Request::GetStatus => {
            // Request status from BLE task (which has access to bonds and profile)
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::GetStatus);

            // Wait for response (with timeout)
            match embassy_time::with_timeout(
                embassy_time::Duration::from_millis(500),
                STATUS_RESPONSE_CHANNEL.receive(),
            )
            .await
            {
                Ok(status) => protocol::encode_response_status(
                    cbor_buf,
                    last_state,
                    status.bonded_count,
                    status.active_profile,
                    status.active_device_set,
                    &status.active_device_address,
                    status.battery_level,
                )
                .unwrap_or(0),
                Err(_) => {
                    // Timeout - use defaults
                    protocol::encode_response_status(
                        cbor_buf, last_state, 0, 0, false, &[0u8; 6], 0xFF,
                    )
                    .unwrap_or(0)
                }
            }
        }

        protocol::Request::StartScan => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::StartScan);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::StopScan => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::StopScan);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::Connect {
            address,
            addr_kind,
            ignore_bond,
        } => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Connect {
                address: *address,
                addr_kind: *addr_kind,
                ignore_bond: *ignore_bond,
            });
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::Disconnect => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Disconnect);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::GetBonds => {
            // Request bonds from BLE task (which has flash access)
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::GetBonds);

            // Wait for response (with timeout)
            match embassy_time::with_timeout(
                embassy_time::Duration::from_millis(500),
                BONDS_RESPONSE_CHANNEL.receive(),
            )
            .await
            {
                Ok(bonds) => {
                    // Convert heapless types to slices for encoding
                    let mut bond_refs: heapless::Vec<([u8; 6], u8, u8, &str), 10> =
                        heapless::Vec::new();
                    for (addr, kind, profile, name) in &bonds {
                        let _ = bond_refs.push((*addr, *kind, *profile, name.as_str()));
                    }
                    protocol::encode_response_bonds(cbor_buf, bond_refs.as_slice()).unwrap_or(0)
                }
                Err(_) => protocol::encode_response_error(cbor_buf, 3, "timeout").unwrap_or(0),
            }
        }

        protocol::Request::ClearBonds => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::ClearBonds);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::SetProfile { .. } => {
            protocol::encode_response_error(cbor_buf, 2, "not implemented").unwrap_or(0)
        }

        protocol::Request::GetConfig => {
            let scroll =
                crate::usb_hid::MULTIPLIER_SCROLL.load(core::sync::atomic::Ordering::Relaxed);
            let pan = crate::usb_hid::MULTIPLIER_PAN.load(core::sync::atomic::Ordering::Relaxed);
            let x = crate::usb_hid::MULTIPLIER_X.load(core::sync::atomic::Ordering::Relaxed);
            let y = crate::usb_hid::MULTIPLIER_Y.load(core::sync::atomic::Ordering::Relaxed);
            protocol::encode_response_config(cbor_buf, scroll, pan, x, y).unwrap_or(0)
        }

        protocol::Request::SetConfig { key, value } => {
            use crate::usb_hid::*;
            use core::sync::atomic::Ordering::Relaxed;
            let valid = match *key {
                CONFIG_KEY_SCROLL_MULT => {
                    MULTIPLIER_SCROLL.store(*value, Relaxed);
                    true
                }
                CONFIG_KEY_PAN_MULT => {
                    MULTIPLIER_PAN.store(*value, Relaxed);
                    true
                }
                CONFIG_KEY_X_MULT => {
                    MULTIPLIER_X.store(*value, Relaxed);
                    true
                }
                CONFIG_KEY_Y_MULT => {
                    MULTIPLIER_Y.store(*value, Relaxed);
                    true
                }
                _ => false,
            };
            if valid {
                // Persist to flash via Core 0
                let _ = BLE_CMD_CHANNEL.try_send(BleCommand::SetConfig {
                    key: *key,
                    value: *value,
                });
                protocol::encode_response_ok(cbor_buf).unwrap_or(0)
            } else {
                protocol::encode_response_error(cbor_buf, 2, "unknown config key").unwrap_or(0)
            }
        }

        protocol::Request::SubscribeLogs { .. } => {
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::UnsubscribeLogs => protocol::encode_response_ok(cbor_buf).unwrap_or(0),

        protocol::Request::GetVersion => {
            protocol::encode_response_version(cbor_buf, VERSION).unwrap_or(0)
        }

        protocol::Request::SetActiveDevice { address, addr_kind } => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::SetActiveDevice {
                address: *address,
                addr_kind: *addr_kind,
            });
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::ClearActiveDevice => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::ClearActiveDevice);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::UpdateBondProfile {
            address,
            profile_id,
        } => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::UpdateBondProfile {
                address: *address,
                profile_id: *profile_id,
            });
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::AutoConnect => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }

        protocol::Request::Restart => {
            let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Restart);
            protocol::encode_response_ok(cbor_buf).unwrap_or(0)
        }
    };

    if cbor_len > 0 {
        send_response(writer, seq_id, cbor_buf, frame_buf, cbor_len).await;
    }
}
