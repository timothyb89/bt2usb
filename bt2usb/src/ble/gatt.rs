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

use core::sync::atomic::Ordering;

use portable_atomic::AtomicU64;

use defmt::*;
use embassy_futures::select::{select, select3, select4, Either3, Either4};
use embassy_time::{Instant, Ticker};
use trouble_host::prelude::*;

use crate::ble_hid::{
    self, parse_hid_report, BATTERY_LEVEL_CHAR_UUID, BATTERY_SERVICE_UUID, HID_REPORT_CHANNEL,
    HID_REPORT_CHAR_UUID, HID_SERVICE_UUID,
};
use crate::ble_state::{BleEvent, BLE_EVENT_CHANNEL};
use crate::device_profile::DeviceProfile;
use crate::protocol::ConnectionState;
use crate::rpc_log;

use super::slots::{self, SlotCommand, MAX_CONNECTIONS, SLOT_CMD_CHANNELS};

/// Latency ladder: as the device sits idle, walk up the peripheral latency
/// (in connection events) to save peer-side battery. Each entry is
/// `(idle threshold, latency)`; the first entry whose threshold is greater
/// than the current idle time wins. After the last threshold the target
/// becomes the peer's own preferred latency (whatever it most recently
/// requested) so we honor each device's chosen sleep behavior. Every step
/// is also clamped against the peer's preferred value so we never push it
/// past what it asked for.
const LATENCY_LADDER: &[(u64, u16)] = &[
    (60, 0),   // <60s idle: stay fully responsive
    (120, 5),  // <120s: tolerate ~75ms wake-up
    (300, 15), // <300s: tolerate ~225ms wake-up
];

/// How often the connection-event handler re-evaluates the latency ladder.
const LADDER_TICK: embassy_time::Duration = embassy_time::Duration::from_secs(5);

/// Per-slot timestamp (Embassy ticks) of the last HID notification received.
/// Written by the notification loop, read by the connection-event handler.
static SLOT_LAST_ACTIVITY_TICKS: [AtomicU64; MAX_CONNECTIONS] =
    [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];

fn mark_slot_activity(slot: usize) {
    if slot < MAX_CONNECTIONS {
        SLOT_LAST_ACTIVITY_TICKS[slot].store(Instant::now().as_ticks(), Ordering::Relaxed);
    }
}

fn slot_idle_secs(slot: usize) -> u64 {
    if slot >= MAX_CONNECTIONS {
        return 0;
    }
    let last = SLOT_LAST_ACTIVITY_TICKS[slot].load(Ordering::Relaxed);
    if last == 0 {
        return 0;
    }
    Instant::now()
        .checked_duration_since(Instant::from_ticks(last))
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn target_latency_for_idle(idle_secs: u64, peer_preferred: u16) -> u16 {
    for &(threshold, latency) in LATENCY_LADDER {
        if idle_secs < threshold {
            return latency.min(peer_preferred);
        }
    }
    peer_preferred
}

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
pub async fn run_gatt_session<'a, C>(
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    conn: &Connection<'a, DefaultPacketPool>,
    active_profile: &mut DeviceProfile,
    slot: usize,
) -> GattSessionResult
where
    C: Controller
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeReadLocalSupportedFeatures>,
{
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

    // Clear per-slot state on disconnect
    ble_hid::clear_battery_level(slot);
    ble_hid::set_slot_layouts(slot, None);

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
    // --- Discover HID service and parse Report Map ---
    let (report_char, mouse_layout) = match discover_hid_service(client, slot).await {
        Some(result) => result,
        None => return GattSessionResult::DiscoveryFailed,
    };

    // Store parsed layouts for this slot (used by USB handler on Core 1)
    ble_hid::set_slot_layouts(slot, mouse_layout);

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

/// Discover the HID service, its Report characteristic, and parse the Report Map.
///
/// Returns the Report characteristic for notification subscription, plus an optional
/// parsed mouse layout from the Report Map descriptor (used for Generic profile translation).
async fn discover_hid_service<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    slot: usize,
) -> Option<(
    Characteristic<[u8; 64]>,
    Option<
        heapless::Vec<
            crate::hid_report_map::MouseReportLayout,
            { crate::hid_report_map::MAX_LAYOUTS },
        >,
    >,
)> {
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

    let report_char = match client
        .characteristic_by_uuid(hid_service, &HID_REPORT_CHAR_UUID)
        .await
    {
        Ok(c) => {
            info!("[slot{}] Found Report characteristic (UUID 0x2A4D)", slot);
            c
        }
        Err(e) => {
            error!(
                "[slot{}] Report characteristic not found: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            return None;
        }
    };

    // --- Read and parse the Report Map (best-effort) ---
    let mouse_layout = read_report_map(client, hid_service, slot).await;

    Some((report_char, mouse_layout))
}

/// Read and parse the HID Report Map characteristic.
///
/// Returns a MouseReportLayout if the Report Map was successfully read and contains
/// mouse-like fields. Returns None on any failure (characteristic not found, read error,
/// parse failure) — the caller falls back to heuristic-based parsing.
async fn read_report_map<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    hid_service: &ServiceHandle,
    slot: usize,
) -> Option<
    heapless::Vec<crate::hid_report_map::MouseReportLayout, { crate::hid_report_map::MAX_LAYOUTS }>,
> {
    // Discover the Report Map characteristic handle
    let report_map_char: Characteristic<[u8; 512]> = match client
        .characteristic_by_uuid(hid_service, &ble_hid::HID_REPORT_MAP_UUID)
        .await
    {
        Ok(c) => c,
        Err(_) => {
            debug!("[slot{}] Report Map characteristic not found", slot);
            return None;
        }
    };

    // Read the full descriptor (auto long-read if > MTU)
    let mut buf = [0u8; 512];
    let len = match embassy_time::with_timeout(
        embassy_time::Duration::from_secs(5),
        client.read_characteristic(&report_map_char, &mut buf),
    )
    .await
    {
        Ok(Ok(n)) => n,
        Ok(Err(e)) => {
            warn!(
                "[slot{}] Failed to read Report Map: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            return None;
        }
        Err(_) => {
            warn!("[slot{}] Report Map read timeout", slot);
            return None;
        }
    };

    info!("[slot{}] Read Report Map ({} bytes)", slot, len);

    match crate::hid_report_map::parse_report_map(&buf[..len]) {
        Some(layouts) => {
            info!(
                "[slot{}] Parsed {} report layout(s) from Report Map",
                slot,
                layouts.len()
            );
            Some(layouts)
        }
        None => {
            info!("[slot{}] Report Map did not contain a mouse layout", slot);
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
                    mark_slot_activity(slot);
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
/// Polls `conn.next()` to process events like connection parameter update requests
/// (without this, the peripheral's L2CAP parameter update request goes unanswered
/// and it disconnects after the 30-second signaling timeout) and runs the latency
/// ladder timer that walks peripheral latency up as the slot stays idle.
async fn handle_connection_events<'a, C>(
    conn: &Connection<'a, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    slot: usize,
) where
    C: Controller
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeReadLocalSupportedFeatures>,
{
    // Reset activity to "now" so a freshly-connected slot is treated as active
    // and the ladder doesn't immediately push it up.
    mark_slot_activity(slot);

    // Track the most recent peer-requested params so we can dedup retries
    // (avoids LL Procedure Collision loops while an update is in flight).
    let mut last_seen_peer_interval: u16 = 0;
    let mut last_seen_peer_latency: u16 = 0;
    // The current latency we're targeting on the link. Starts at 0 and walks
    // up via the ladder; resets to 0 when the peer signals wake.
    let mut current_target_latency: u16 = 0;
    // The highest latency the peer has asked for during this session. We use
    // this as the ladder ceiling so each device gets its own preferred sleep
    // behavior instead of a hard-coded constant.
    let mut peer_preferred_latency: u16 = 0;
    // Last params the peer requested — used as the base for self-initiated
    // updates so we keep its preferred interval/timeout.
    let mut base_params: Option<RequestedConnParams> = None;
    // After a self-initiated `update_connection_params`, the LL procedure
    // takes ~6+ connection intervals to apply. During that window, accepting
    // an incoming peer request would issue a second LL update and collide
    // (HCI status 0x23). Drop peer requests until this deadline passes.
    let mut self_update_until: Option<Instant> = None;

    let mut ladder_ticker = Ticker::every(LADDER_TICK);

    loop {
        match select(conn.next(), ladder_ticker.next()).await {
            embassy_futures::select::Either::First(event) => match event {
                ConnectionEvent::RequestConnectionParams(req) => {
                    let requested = req.params().clone();
                    let interval = requested.max_connection_interval.as_micros() as u16;
                    let latency = requested.max_latency;

                    // If a self-initiated update is still propagating, drop
                    // peer requests rather than colliding with it.
                    if let Some(until) = self_update_until {
                        if Instant::now() < until {
                            core::mem::forget(req);
                            continue;
                        }
                        self_update_until = None;
                    }

                    // A peer request asking for *lower* latency than our
                    // current step is the wake-from-sleep signal — snap our
                    // target down to 0 and mark the slot active so the ladder
                    // restarts its climb from the bottom.
                    if latency < current_target_latency {
                        debug!(
                            "[slot{}] Peer wake signal (latency {} < target {}) — snapping to 0",
                            slot, latency, current_target_latency
                        );
                        current_target_latency = 0;
                        mark_slot_activity(slot);
                        last_seen_peer_interval = 0;
                        last_seen_peer_latency = 0;
                    }

                    if interval == last_seen_peer_interval && latency == last_seen_peer_latency {
                        // Same params we already processed — skip silently.
                        core::mem::forget(req);
                        continue;
                    }

                    // Clamp the peer's requested latency to our current ladder
                    // step (peer can only ask for *more* latency than us; if
                    // it asks for less we already snapped above).
                    let clamped = RequestedConnParams {
                        max_latency: current_target_latency,
                        ..requested
                    };
                    base_params = Some(clamped.clone());

                    debug!(
                        "[slot{}] Peer param update: interval={} latency={} → clamped latency={}",
                        slot, interval, latency, current_target_latency
                    );
                    let _ = req.accept(Some(&clamped), stack).await;
                    last_seen_peer_interval = interval;
                    last_seen_peer_latency = latency;
                    if latency > peer_preferred_latency {
                        peer_preferred_latency = latency;
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
            },

            embassy_futures::select::Either::Second(()) => {
                if peer_preferred_latency == 0 {
                    // Peer hasn't asked for any latency yet — nothing to walk
                    // toward. Wait for a request to set the ceiling.
                    continue;
                }
                let idle = slot_idle_secs(slot);
                let target = target_latency_for_idle(idle, peer_preferred_latency);
                if target == current_target_latency {
                    continue;
                }
                let Some(ref base) = base_params else {
                    current_target_latency = target;
                    continue;
                };
                let params = RequestedConnParams {
                    max_latency: target,
                    ..base.clone()
                };
                if !params.is_valid() {
                    continue;
                }
                debug!(
                    "[slot{}] Ladder step: idle={}s, latency {} → {}",
                    slot, idle, current_target_latency, target
                );
                match conn.update_connection_params(stack, &params).await {
                    Ok(()) => {
                        current_target_latency = target;
                        base_params = Some(params);
                        // Block peer-request processing until the LL update
                        // has had time to apply (~6 conn intervals + slack).
                        self_update_until =
                            Some(Instant::now() + embassy_time::Duration::from_millis(500));
                    }
                    Err(_) => {
                        // Likely LL Procedure Collision — try again next tick.
                    }
                }
            }
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
