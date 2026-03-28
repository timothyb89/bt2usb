//! GATT service discovery and HID notification loop
//!
//! After a BLE connection is established and paired, this module handles:
//! 1. Discovering the HID service (UUID 0x1812) and Report characteristic (0x2A4D)
//! 2. Discovering the Battery service (UUID 0x180F) for battery level monitoring
//! 3. Running the main HID event loop that forwards BLE reports to USB
//!
//! The event loop uses `select4` to concurrently handle:
//! - HID report notifications from the BLE device
//! - Slot commands from the connection manager (disconnect, profile changes)
//! - Battery level notifications (if supported by device)
//! - Battery level polling fallback (if notifications not supported)

use defmt::*;
use embassy_futures::select::{select, select3, select4, Either3, Either4};
use embassy_time::Ticker;
use trouble_host::prelude::*;

use crate::ble_hid::{
    self, parse_hid_report, BATTERY_LEVEL_CHAR_UUID, BATTERY_SERVICE_UUID, HID_REPORT_CHANNEL,
    HID_REPORT_CHAR_UUID, HID_SERVICE_UUID,
};
use crate::ble_state::{BleEvent, BLE_EVENT_CHANNEL};
use crate::device_profile::DeviceProfile;
use crate::protocol::ConnectionState;
use crate::rpc_log;

use super::slots::{self, SlotCommand, SLOT_CMD_CHANNELS};

/// Result of a GATT session.
pub enum GattSessionResult {
    /// Connection ended naturally (GATT client dropped, device disconnected).
    ConnectionLost,
    /// Session ended (by command or naturally).
    Ended,
    /// Service discovery failed (timeout or not found) — connection still alive, can retry.
    DiscoveryFailed,
}

/// Run the full GATT session: discover services, subscribe to HID, and process events.
///
/// This creates a GATT client and runs it concurrently with the HID notification loop.
/// Returns a `GattSessionResult` indicating how the session ended.
pub async fn run_gatt_session<'a, C: Controller>(
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    conn: &Connection<'a, DefaultPacketPool>,
    active_profile: &mut DeviceProfile,
    slot: usize,
) -> GattSessionResult {
    info!("[slot{}] Creating GATT client...", slot);

    let client = match GattClient::<C, DefaultPacketPool, 10>::new(stack, conn).await {
        Ok(c) => c,
        Err(e) => {
            error!(
                "[slot{}] Failed to create GATT client: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            return GattSessionResult::Ended;
        }
    };

    // Run GATT client driver, HID session, and connection event handler concurrently.
    // client.task() must run for GATT operations to proceed.
    // Connection events must be polled to handle parameter update requests from the peripheral.
    let run_result = select3(
        client.task(),
        run_hid_session(&client, conn, active_profile, slot),
        handle_connection_events(conn, stack, slot),
    )
    .await;

    // Clear battery level for this slot on disconnect
    ble_hid::clear_battery_level(slot);

    match run_result {
        Either3::First(_) => {
            info!("[slot{}] GATT client task ended - connection lost", slot);
            GattSessionResult::ConnectionLost
        }
        Either3::Second(result) => result,
        Either3::Third(_) => {
            info!(
                "[slot{}] Connection event handler ended - connection lost",
                slot
            );
            GattSessionResult::ConnectionLost
        }
    }
}

/// Inner HID session: discover services, subscribe, and run the event loop.
async fn run_hid_session<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    conn: &Connection<'a, DefaultPacketPool>,
    active_profile: &mut DeviceProfile,
    slot: usize,
) -> GattSessionResult {
    // --- Discover HID service ---
    let report_char = match discover_hid_service(client, slot).await {
        Some(c) => c,
        None => return GattSessionResult::DiscoveryFailed,
    };

    // --- Subscribe to HID notifications ---
    let mut listener = match client.subscribe(&report_char, false).await {
        Ok(l) => l,
        Err(e) => {
            error!(
                "[slot{}] Failed to subscribe to notifications: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            return GattSessionResult::Ended;
        }
    };

    info!("[slot{}] === HID CONNECTION ESTABLISHED ===", slot);
    info!(
        "[slot{}] Subscribed to notifications on handle {:?}",
        slot, report_char.handle
    );
    rpc_log::info("HID connection ready - receiving reports");
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Ready));

    // --- Battery service (best-effort — HID continues regardless) ---
    let battery_char = discover_battery_service(client, slot).await;

    // Read initial battery level immediately
    if let Some(ref c) = battery_char {
        read_battery_level(client, c, slot).await;
    }

    // Subscribe for ongoing battery notifications
    let mut battery_listener = if let Some(ref c) = battery_char {
        match client.subscribe(c, false).await {
            Ok(l) => {
                info!("[slot{}] Subscribed to battery notifications", slot);
                Some(l)
            }
            Err(_) => {
                info!(
                    "[slot{}] Battery notifications not supported by device",
                    slot
                );
                None
            }
        }
    } else {
        None
    };

    // Polling fallback: if no notifications, poll every 5 minutes
    let mut battery_poll_ticker = if battery_listener.is_none() && battery_char.is_some() {
        info!(
            "[slot{}] Battery poll fallback active (5 min interval)",
            slot
        );
        Some(Ticker::every(embassy_time::Duration::from_secs(5 * 60)))
    } else {
        None
    };

    // --- Main HID event loop ---
    run_hid_event_loop(
        &mut listener,
        &mut battery_listener,
        &mut battery_poll_ticker,
        &battery_char,
        client,
        conn,
        active_profile,
        slot,
    )
    .await
}

/// Discover the HID service and its Report characteristic.
async fn discover_hid_service<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    slot: usize,
) -> Option<Characteristic<[u8; 64]>> {
    let services = match embassy_time::with_timeout(
        embassy_time::Duration::from_secs(10),
        client.services_by_uuid(&HID_SERVICE_UUID),
    )
    .await
    {
        Ok(Ok(s)) if !s.is_empty() => s,
        Ok(Ok(_)) => {
            error!("[slot{}] No HID service found!", slot);
            return None;
        }
        Ok(Err(e)) => {
            error!(
                "[slot{}] HID service discovery failed: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            return None;
        }
        Err(_) => {
            error!("[slot{}] HID service discovery timeout", slot);
            return None;
        }
    };

    let hid_service = &services[0];
    info!("[slot{}] Found HID service", slot);
    rpc_log::info("HID service discovered");

    match client
        .characteristic_by_uuid(hid_service, &HID_REPORT_CHAR_UUID)
        .await
    {
        Ok(c) => {
            info!("[slot{}] Found Report characteristic (UUID 0x2A4D)", slot);
            Some(c)
        }
        Err(e) => {
            error!(
                "[slot{}] Report characteristic not found: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            None
        }
    }
}

/// Discover the Battery service and its Battery Level characteristic.
async fn discover_battery_service<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    slot: usize,
) -> Option<Characteristic<u8>> {
    match embassy_time::with_timeout(
        embassy_time::Duration::from_secs(5),
        client.services_by_uuid(&BATTERY_SERVICE_UUID),
    )
    .await
    {
        Ok(Ok(svcs)) if !svcs.is_empty() => {
            match client
                .characteristic_by_uuid(&svcs[0], &BATTERY_LEVEL_CHAR_UUID)
                .await
            {
                Ok(c) => {
                    info!("[slot{}] Found battery level characteristic", slot);
                    Some(c)
                }
                Err(_) => {
                    info!("[slot{}] Battery level characteristic not found", slot);
                    None
                }
            }
        }
        _ => {
            info!("[slot{}] Battery service not found", slot);
            None
        }
    }
}

/// Read the current battery level from the device and update the global state.
async fn read_battery_level<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    battery_char: &Characteristic<u8>,
    slot: usize,
) {
    let mut data = [0u8; 1];
    match embassy_time::with_timeout(
        embassy_time::Duration::from_secs(3),
        client.read_characteristic(battery_char, &mut data),
    )
    .await
    {
        Ok(Ok(_)) => {
            let level = data[0];
            ble_hid::update_battery_level(slot, level);
            info!("[slot{}] Battery level: {}%", slot, level);
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BatteryLevel(level));
        }
        _ => info!("[slot{}] Could not read battery level", slot),
    }
}

/// The main HID event loop — processes notifications, slot commands, and battery updates.
///
/// Uses `select4` to concurrently wait on:
/// 1. HID report notifications from the BLE device
/// 2. Slot commands from the connection manager (disconnect, profile update)
/// 3. Battery level notifications (if supported)
/// 4. Battery level poll timer (fallback if notifications not supported)
#[allow(clippy::too_many_arguments)]
async fn run_hid_event_loop<'a, 'c, C: Controller>(
    listener: &mut NotificationListener<'c, 512>,
    battery_listener: &mut Option<NotificationListener<'c, 512>>,
    battery_poll_ticker: &mut Option<Ticker>,
    battery_char: &Option<Characteristic<u8>>,
    client: &'c GattClient<'a, C, DefaultPacketPool, 10>,
    conn: &Connection<'a, DefaultPacketPool>,
    active_profile: &mut DeviceProfile,
    slot: usize,
) -> GattSessionResult {
    loop {
        match select4(
            listener.next(),
            SLOT_CMD_CHANNELS[slot].receive(),
            async {
                if let Some(ref mut bl) = battery_listener {
                    bl.next().await
                } else {
                    core::future::pending().await
                }
            },
            async {
                if let Some(ref mut t) = battery_poll_ticker {
                    t.next().await
                } else {
                    core::future::pending().await
                }
            },
        )
        .await
        {
            // HID report notification
            Either4::First(notification) => {
                let data = notification.as_ref();
                if !data.is_empty() {
                    let mut report = parse_hid_report(data, 0, *active_profile);
                    report.slot_index = slot as u8;
                    HID_REPORT_CHANNEL.send(report).await;
                }
            }

            // Slot command from connection manager
            Either4::Second(cmd) => match cmd {
                SlotCommand::Disconnect => {
                    info!("[slot{}] Disconnect command received", slot);
                    rpc_log::info("Disconnecting by request");
                    conn.disconnect();
                    wait_for_disconnect(conn).await;
                    return GattSessionResult::Ended;
                }
                SlotCommand::UpdateProfile(profile_id) => {
                    info!("[slot{}] Updating profile to {}", slot, profile_id);
                    *active_profile = DeviceProfile::from_id(profile_id);
                    slots::SLOT_PROFILES[slot]
                        .store(profile_id, core::sync::atomic::Ordering::Relaxed);
                    rpc_log::info("Profile updated");
                }
            },

            // Battery notification
            Either4::Third(battery_notif) => {
                let data = battery_notif.as_ref();
                if !data.is_empty() {
                    let level = data[0];
                    ble_hid::update_battery_level(slot, level);
                    info!("[slot{}] Battery level update: {}%", slot, level);
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BatteryLevel(level));
                }
            }

            // Battery poll timer
            Either4::Fourth(()) => {
                if let Some(ref c) = battery_char {
                    read_battery_level(client, c, slot).await;
                }
            }
        }
    }
}

/// Handle connection events during the GATT session.
///
/// Polls `conn.next()` to process events like connection parameter update requests.
/// Without this, the peripheral's L2CAP parameter update request goes unanswered
/// and it disconnects after the 30-second signaling timeout.
async fn handle_connection_events<'a, C: Controller>(
    conn: &Connection<'a, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    slot: usize,
) {
    loop {
        match conn.next().await {
            ConnectionEvent::RequestConnectionParams(req) => {
                info!(
                    "[slot{}] Accepting connection parameter update: {:?}",
                    slot,
                    req.params()
                );
                if let Err(e) = req.accept(None, stack).await {
                    warn!(
                        "[slot{}] Failed to accept conn params: {:?}",
                        slot,
                        defmt::Debug2Format(&e)
                    );
                }
            }
            ConnectionEvent::Disconnected { reason } => {
                info!(
                    "[slot{}] Connection event: disconnected ({:?})",
                    slot, reason
                );
                break;
            }
            _ => {}
        }
    }
}

/// Wait for the connection to fully disconnect (with timeout).
async fn wait_for_disconnect<'a>(conn: &Connection<'a, DefaultPacketPool>) {
    let _ = select(
        async {
            loop {
                if let ConnectionEvent::Disconnected { .. } = conn.next().await {
                    break;
                }
            }
        },
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)),
    )
    .await;
}
