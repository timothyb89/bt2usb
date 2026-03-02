//! GATT service discovery and HID notification loop
//!
//! After a BLE connection is established and paired, this module handles:
//! 1. Discovering the HID service (UUID 0x1812) and Report characteristic (0x2A4D)
//! 2. Discovering the Battery service (UUID 0x180F) for battery level monitoring
//! 3. Running the main HID event loop that forwards BLE reports to USB
//!
//! The event loop uses `select4` to concurrently handle:
//! - HID report notifications from the BLE device
//! - Commands from the RPC handler (disconnect, profile changes, etc.)
//! - Battery level notifications (if supported by device)
//! - Battery level polling fallback (if notifications not supported)

use core::sync::atomic::Ordering;

use defmt::*;
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_time::{Ticker, Timer};
use trouble_host::prelude::*;

use crate::ble_hid::{
    self, parse_hid_report, HID_REPORT_CHANNEL, BATTERY_LEVEL_CHAR_UUID, BATTERY_SERVICE_UUID,
    HID_REPORT_CHAR_UUID, HID_SERVICE_UUID,
};
use crate::ble_state::{BleCommand, BleEvent, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL};
use crate::bonding::{self, LoadedBond};
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::protocol::ConnectionState;
use crate::rpc_log;
use crate::FLASH_SIZE;

use super::commands;

/// Result of a GATT session — either ended naturally or was interrupted by a command.
pub enum GattSessionResult {
    /// Connection ended naturally (GATT client dropped, device disconnected).
    ConnectionLost,
    /// A command was received that should be forwarded to the outer state machine.
    InterruptedByCommand(BleCommand),
    /// Session ended without a pending command.
    Ended,
}

/// Run the full GATT session: discover services, subscribe to HID, and process events.
///
/// This creates a GATT client and runs it concurrently with the HID notification loop.
/// Returns a `GattSessionResult` indicating how the session ended.
pub async fn run_gatt_session<'a, C: Controller>(
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: &mut DeviceProfile,
    loaded_bonds: &[LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> GattSessionResult {
    info!("Creating GATT client...");

    let client = match GattClient::<C, DefaultPacketPool, 10>::new(stack, conn).await {
        Ok(c) => c,
        Err(e) => {
            error!(
                "Failed to create GATT client: {:?}",
                defmt::Debug2Format(&e)
            );
            return GattSessionResult::Ended;
        }
    };

    // Run GATT client driver and HID session concurrently.
    // client.task() must run for GATT operations to proceed.
    let run_result = select(
        client.task(),
        run_hid_session(
            &client,
            conn,
            flash,
            active_profile,
            loaded_bonds,
            active_device_pref,
        ),
    )
    .await;

    // Clear battery level on disconnect
    ble_hid::BATTERY_LEVEL.store(0xFF, Ordering::Relaxed);

    match run_result {
        Either::First(_) => {
            // GATT client task ended (connection dropped)
            info!("GATT client task ended - connection lost");
            GattSessionResult::ConnectionLost
        }
        Either::Second(result) => result,
    }
}

/// Inner HID session: discover services, subscribe, and run the event loop.
///
/// This runs inside `select(client.task(), ...)` so the GATT client driver
/// is active for all operations.
async fn run_hid_session<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: &mut DeviceProfile,
    loaded_bonds: &[LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> GattSessionResult {
    // --- Discover HID service ---
    let report_char = match discover_hid_service(client).await {
        Some(c) => c,
        None => return GattSessionResult::Ended,
    };

    // --- Subscribe to HID notifications ---
    let mut listener = match client.subscribe(&report_char, false).await {
        Ok(l) => l,
        Err(e) => {
            error!(
                "Failed to subscribe to notifications: {:?}",
                defmt::Debug2Format(&e)
            );
            return GattSessionResult::Ended;
        }
    };

    info!("=== HID CONNECTION ESTABLISHED ===");
    info!(
        "Subscribed to notifications on handle {:?}",
        report_char.handle
    );
    rpc_log::info("HID connection ready - receiving reports");
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Ready));

    // --- Battery service (best-effort — HID continues regardless) ---
    let battery_char = discover_battery_service(client).await;

    // Read initial battery level immediately so GetStatus is populated right away
    if let Some(ref c) = battery_char {
        read_battery_level(client, c).await;
    }

    // Subscribe for ongoing battery notifications (best-effort)
    let mut battery_listener = if let Some(ref c) = battery_char {
        match client.subscribe(c, false).await {
            Ok(l) => {
                info!("Subscribed to battery notifications");
                Some(l)
            }
            Err(_) => {
                info!("Battery notifications not supported by device");
                None
            }
        }
    } else {
        None
    };

    // Polling fallback: if no notifications, poll every 5 minutes
    let mut battery_poll_ticker = if battery_listener.is_none() && battery_char.is_some() {
        info!("Battery poll fallback active (5 min interval)");
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
        flash,
        active_profile,
        loaded_bonds,
        active_device_pref,
    )
    .await
}

/// Discover the HID service and its Report characteristic.
///
/// Returns the Report characteristic on success, or None if discovery fails.
async fn discover_hid_service<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
) -> Option<Characteristic<[u8; 64]>> {
    let services = match embassy_time::with_timeout(
        embassy_time::Duration::from_secs(10),
        client.services_by_uuid(&HID_SERVICE_UUID),
    )
    .await
    {
        Ok(Ok(s)) if !s.is_empty() => s,
        Ok(Ok(_)) => {
            error!("No HID service found!");
            return None;
        }
        Ok(Err(e)) => {
            error!(
                "HID service discovery failed: {:?}",
                defmt::Debug2Format(&e)
            );
            return None;
        }
        Err(_) => {
            error!("HID service discovery timeout");
            return None;
        }
    };

    let hid_service = &services[0];
    info!("Found HID service");
    rpc_log::info("HID service discovered");

    // Discover HID Report characteristic (UUID 0x2A4D)
    match client
        .characteristic_by_uuid(hid_service, &HID_REPORT_CHAR_UUID)
        .await
    {
        Ok(c) => {
            info!("Found Report characteristic (UUID 0x2A4D)");
            info!("CRITICAL: Characteristic handle = {:?}", c.handle);
            info!("Linux uses handle 0x0020 for symmetric scroll values");
            info!("If we're using a different handle, that explains the asymmetry!");
            Some(c)
        }
        Err(e) => {
            error!(
                "Report characteristic not found: {:?}",
                defmt::Debug2Format(&e)
            );
            None
        }
    }
}

/// Discover the Battery service and its Battery Level characteristic.
///
/// Returns the Battery Level characteristic on success, or None if not found.
/// This is best-effort — the HID session continues regardless.
async fn discover_battery_service<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
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
                    info!("Found battery level characteristic");
                    Some(c)
                }
                Err(_) => {
                    info!("Battery level characteristic not found");
                    None
                }
            }
        }
        _ => {
            info!("Battery service not found");
            None
        }
    }
}

/// Read the current battery level from the device and update the global state.
async fn read_battery_level<'a, C: Controller>(
    client: &GattClient<'a, C, DefaultPacketPool, 10>,
    battery_char: &Characteristic<u8>,
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
            ble_hid::BATTERY_LEVEL.store(level, Ordering::Relaxed);
            ble_hid::BATTERY_USB_SIGNAL.signal(level);
            info!("Battery level: {}%", level);
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BatteryLevel(level));
        }
        _ => info!("Could not read battery level"),
    }
}

/// The main HID event loop — processes notifications, commands, and battery updates.
///
/// Uses `select4` to concurrently wait on:
/// 1. HID report notifications from the BLE device
/// 2. Commands from the RPC handler
/// 3. Battery level notifications (if supported)
/// 4. Battery level poll timer (fallback if notifications not supported)
///
/// Returns a `GattSessionResult` indicating how the loop ended.
#[allow(clippy::too_many_arguments)]
async fn run_hid_event_loop<'a, 'c, C: Controller>(
    listener: &mut NotificationListener<'c, 512>,
    battery_listener: &mut Option<NotificationListener<'c, 512>>,
    battery_poll_ticker: &mut Option<Ticker>,
    battery_char: &Option<Characteristic<u8>>,
    client: &'c GattClient<'a, C, DefaultPacketPool, 10>,
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: &mut DeviceProfile,
    loaded_bonds: &[LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> GattSessionResult {
    loop {
        match select4(
            listener.next(),
            BLE_CMD_CHANNEL.receive(),
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
                    let preview_len = data.len().min(8);
                    debug!(
                        "Raw notification: len={}, first {} bytes: {:?}",
                        data.len(),
                        preview_len,
                        &data[..preview_len]
                    );
                    let report = parse_hid_report(data, 0, *active_profile);
                    HID_REPORT_CHANNEL.send(report).await;
                }
            }

            // Command from RPC
            Either4::Second(cmd) => {
                match handle_command_during_hid_loop(
                    cmd,
                    conn,
                    flash,
                    active_profile,
                    loaded_bonds,
                    active_device_pref,
                )
                .await
                {
                    CommandResult::Continue => {}
                    CommandResult::Disconnect => return GattSessionResult::Ended,
                    CommandResult::Forward(cmd) => {
                        return GattSessionResult::InterruptedByCommand(cmd)
                    }
                }
            }

            // Battery notification
            Either4::Third(battery_notif) => {
                let data = battery_notif.as_ref();
                if !data.is_empty() {
                    let level = data[0];
                    ble_hid::BATTERY_LEVEL.store(level, Ordering::Relaxed);
                    ble_hid::BATTERY_USB_SIGNAL.signal(level);
                    info!("Battery level update: {}%", level);
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BatteryLevel(level));
                }
            }

            // Battery poll timer
            Either4::Fourth(()) => {
                if let Some(ref c) = battery_char {
                    read_battery_level(client, c).await;
                }
            }
        }
    }
}

/// Result of handling a command during the HID event loop.
enum CommandResult {
    /// Command handled, continue the loop.
    Continue,
    /// Disconnect requested, end the session.
    Disconnect,
    /// Command should be forwarded to the outer state machine.
    Forward(BleCommand),
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
        Timer::after(embassy_time::Duration::from_secs(1)),
    )
    .await;
}

/// Handle a BleCommand received during the active HID event loop.
///
/// Some commands (GetStatus, GetBonds, etc.) are handled inline and the loop
/// continues. Others (Disconnect, Connect, Restart) end the session.
async fn handle_command_during_hid_loop<'a>(
    cmd: BleCommand,
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: &mut DeviceProfile,
    loaded_bonds: &[LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> CommandResult {
    info!("Command received: {:?}", cmd);

    match cmd {
        BleCommand::Disconnect => {
            rpc_log::info("Disconnecting by request");
            conn.disconnect();
            wait_for_disconnect(conn).await;
            CommandResult::Disconnect
        }

        BleCommand::Connect { .. } | BleCommand::AutoConnect => {
            conn.disconnect();
            wait_for_disconnect(conn).await;
            CommandResult::Forward(cmd)
        }

        BleCommand::UpdateBondProfile {
            address,
            profile_id,
        } => {
            info!("Updating active profile to {}", profile_id);
            *active_profile = DeviceProfile::from_id(profile_id);

            // Persist to flash
            match bonding::update_bond_profile(flash, &address, profile_id).await {
                Ok(slot) => {
                    info!("Bond profile persisted to slot {}", slot);
                    rpc_log::info("Profile updated and saved");
                }
                Err(_) => {
                    error!("Failed to persist bond profile");
                    rpc_log::error("Failed to save profile");
                }
            }
            CommandResult::Continue
        }

        BleCommand::Restart => {
            commands::handle_restart().await;
        }

        BleCommand::ClearBonds => {
            // Disconnect, clear bonds, and restart
            rpc_log::info("Disconnecting to clear bonds...");
            conn.disconnect();
            Timer::after_millis(200).await;

            rpc_log::info("Clearing bonds...");
            match bonding::clear_all_bonds(flash).await {
                Ok(_) => {
                    rpc_log::info("Restarting device...");
                    Timer::after_millis(100).await;
                    crate::system_reset();
                }
                Err(_) => {
                    rpc_log::error("Failed to clear bonds");
                    CommandResult::Disconnect
                }
            }
        }

        BleCommand::GetStatus => {
            info!("Getting status info (connected)");
            commands::handle_get_status(loaded_bonds, *active_profile, active_device_pref);
            CommandResult::Continue
        }

        BleCommand::GetBonds => {
            info!("Getting bonds list");
            commands::handle_get_bonds(loaded_bonds);
            CommandResult::Continue
        }

        BleCommand::SetActiveDevice { address, addr_kind } => {
            commands::handle_set_active_device(flash, address, addr_kind).await;
            CommandResult::Continue
        }

        BleCommand::ClearActiveDevice => {
            commands::handle_clear_active_device(flash).await;
            CommandResult::Continue
        }

        BleCommand::SetConfig { key, value } => {
            commands::handle_set_config(flash, key, value).await;
            CommandResult::Continue
        }

        _ => CommandResult::Continue,
    }
}
