//! BLE connection and pairing logic
//!
//! Handles the full lifecycle of connecting to a BLE HID device:
//! 1. Establish BLE connection (with retry and command interruption)
//! 2. Pair or re-encrypt (depending on whether a bond exists)
//! 3. Store new bond to flash on successful pairing
//! 4. Hand off to GATT session for HID report streaming
//!
//! The outer retry loop handles transient connection/pairing failures,
//! retrying up to MAX_PAIRING_RETRIES times before giving up.

use defmt::*;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Timer};
use trouble_host::connection::{ConnectConfig, ScanConfig};
use trouble_host::prelude::*;
use trouble_host::Host;

use crate::ble_state::{BleEvent, BLE_EVENT_CHANNEL};
use crate::bonding;
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::protocol::ConnectionState;
use crate::rpc_log;

use super::gatt::{self, GattSessionResult};
use super::slots::{self, SLOT_CMD_CHANNELS};
use super::FlashMutex;

const MAX_PAIRING_RETRIES: u8 = 5;

/// Error during the connection/pairing phase.
enum StartError {
    PairingFailed(trouble_host::Error),
    Disconnected(bt_hci::param::Status),
}

/// Connect to a BLE HID device, pair, discover GATT, and run the HID report loop.
///
/// Creates its own `Central` via `stack.build()`, so the caller's Scanner/Central
/// is not consumed. This enables concurrent connections across multiple slots.
///
/// Bond storage uses the shared flash mutex (all on Core 0).
/// Returns `None` when the connection ends naturally or after max retries.
#[allow(clippy::too_many_arguments)]
pub async fn ble_connect_and_run<'a, C: Controller>(
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
    target: Address,
    active_profile: &mut DeviceProfile,
    has_stored_bond: bool,
    slot: usize,
    _active_device_pref: &Option<preferences::ActiveDevice>,
) -> Option<()> {
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));
    rpc_log::info("Connecting to BLE device");

    // Get our own Central from the stack — does not consume the caller's Scanner/Central
    let Host { mut central, .. } = stack.build();

    let mut config = ConnectConfig {
        connect_params: Default::default(),
        scan_config: ScanConfig {
            active: true,
            filter_accept_list: &[(target.kind, &target.addr)],
            ..Default::default()
        },
    };
    config.connect_params.min_connection_interval = Duration::from_micros(7500);
    config.connect_params.max_connection_interval = Duration::from_millis(15);
    config.connect_params.supervision_timeout = Duration::from_secs(2);

    let mut pairing_attempts: u8 = 0;

    'connect: loop {
        pairing_attempts += 1;
        info!(
            "[slot{}] Connection attempt {} of {}",
            slot, pairing_attempts, MAX_PAIRING_RETRIES
        );

        // Phase 1: Establish BLE connection
        let conn = match connect_to_device(&mut central, &config, slot).await {
            ConnectOutcome::Connected(conn) => {
                slots::set_slot_connected(slot);
                conn
            }
            ConnectOutcome::RetryableError => {
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    error!("[slot{}] Max connection retries exceeded", slot);
                    rpc_log::error("Max connection retries exceeded");
                    return None;
                }
                Timer::after(Duration::from_millis(500)).await;
                continue 'connect;
            }
            ConnectOutcome::Cancelled => return None,
        };

        // Phase 2: Pair or re-encrypt
        match initiate_security(&conn, has_stored_bond).await {
            SecurityOutcome::Ready => {}
            SecurityOutcome::RetryableError => {
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    return None;
                }
                Timer::after(Duration::from_millis(500)).await;
                continue 'connect;
            }
        }

        // Phase 3: Wait for pairing to complete
        match await_pairing(&conn, flash, *active_profile, slot).await {
            PairingOutcome::Success => {}
            PairingOutcome::Failed => {
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    error!("[slot{}] Max pairing retries exceeded", slot);
                    rpc_log::error("Max pairing retries exceeded");
                    return None;
                }
                Timer::after(Duration::from_millis(500)).await;
                continue 'connect;
            }
            PairingOutcome::Cancelled => return None,
        }

        // Phase 4: GATT discovery and HID report loop (with retry).
        //
        // Some devices (e.g. MX Master 3S) initiate their own GATT discovery of the
        // central immediately after encryption. Since trouble-host has no GATT server,
        // these requests go unanswered. The device may stall our service discovery
        // until its own ATT transaction times out (~30s). We retry GATT discovery
        // to handle this: the first attempt may timeout, but the second succeeds
        // after the device gives up on its own request.
        const MAX_GATT_RETRIES: u8 = 3;
        let mut gatt_attempts: u8 = 0;
        loop {
            gatt_attempts += 1;
            match gatt::run_gatt_session(stack, &conn, active_profile, slot).await {
                GattSessionResult::ConnectionLost => break 'connect,
                GattSessionResult::Ended => break 'connect,
                GattSessionResult::DiscoveryFailed => {
                    if gatt_attempts >= MAX_GATT_RETRIES {
                        warn!(
                            "[slot{}] GATT discovery failed after {} attempts",
                            slot, gatt_attempts
                        );
                        break 'connect;
                    }
                    info!(
                        "[slot{}] GATT discovery failed, retrying ({}/{})",
                        slot, gatt_attempts, MAX_GATT_RETRIES
                    );
                    Timer::after(Duration::from_millis(1000)).await;
                }
            }
        }
    }

    None
}

// ============ Phase 1: Connection ============

enum ConnectOutcome<'a> {
    Connected(Connection<'a, DefaultPacketPool>),
    RetryableError,
    Cancelled,
}

/// Attempt to establish a BLE connection, with timeout and slot command interruption.
///
/// Times out after 30 seconds if the target device is not found (not advertising).
async fn connect_to_device<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    config: &ConnectConfig<'_>,
    slot: usize,
) -> ConnectOutcome<'a> {
    match select3(
        central.connect(config),
        SLOT_CMD_CHANNELS[slot].receive(),
        Timer::after(Duration::from_secs(30)),
    )
    .await
    {
        Either3::First(Ok(conn)) => {
            info!("[slot{}] Connected to HID device!", slot);
            rpc_log::info("Connected to BLE device");
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connected));
            ConnectOutcome::Connected(conn)
        }
        Either3::First(Err(e)) => {
            error!(
                "[slot{}] Connection failed: {:?}",
                slot,
                defmt::Debug2Format(&e)
            );
            rpc_log::error("Connection attempt failed");
            ConnectOutcome::RetryableError
        }
        Either3::Second(cmd) => {
            info!("[slot{}] Command during connection: {:?}", slot, cmd);
            ConnectOutcome::Cancelled
        }
        Either3::Third(_) => {
            warn!("[slot{}] Connection attempt timed out (30s)", slot);
            rpc_log::warn("Connection timed out - device not found");
            ConnectOutcome::RetryableError
        }
    }
}

// ============ Phase 2: Security Setup ============

enum SecurityOutcome {
    Ready,
    RetryableError,
}

/// Initiate the security process based on whether a bond exists.
///
/// - With an existing bond: just wait for automatic re-encryption
/// - Without a bond: request security (pairing)
async fn initiate_security<'a>(
    conn: &Connection<'a, DefaultPacketPool>,
    has_stored_bond: bool,
) -> SecurityOutcome {
    if has_stored_bond {
        info!("Existing bond detected - waiting for automatic re-encryption...");
        rpc_log::info("Re-encryption in progress");
        SecurityOutcome::Ready
    } else {
        if let Err(e) = conn.set_bondable(true) {
            error!("Failed to set bondable: {:?}", e);
        }

        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Pairing));
        rpc_log::info("Initiating pairing");
        info!("Requesting security (bondable: true)...");

        match conn.request_security() {
            Ok(_) => {
                info!("Security request sent");
                SecurityOutcome::Ready
            }
            Err(e) => {
                error!("Failed to request security: {:?}", e);
                SecurityOutcome::RetryableError
            }
        }
    }
}

// ============ Phase 3: Pairing Completion ============

enum PairingOutcome {
    Success,
    Failed,
    Cancelled,
}

/// Wait for pairing/re-encryption to complete, with timeout and slot command interruption.
///
/// On success with a new bond, stores the bond info to flash via the shared mutex.
async fn await_pairing<'a>(
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &FlashMutex,
    active_profile: DeviceProfile,
    slot: usize,
) -> PairingOutcome {
    let pairing_timeout = Timer::after(Duration::from_secs(15));

    let pairing_result = select(
        select(
            wait_for_pairing_event(conn),
            SLOT_CMD_CHANNELS[slot].receive(),
        ),
        pairing_timeout,
    )
    .await;

    match pairing_result {
        // Pairing event received
        Either::First(Either::First(Ok((security_level, bond)))) => {
            info!(
                "[slot{}] Pairing complete! Level: {:?}",
                slot, security_level
            );
            rpc_log::info("Pairing complete");
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingComplete);

            if let Some(bond_info) = bond {
                store_new_bond(flash, &bond_info, active_profile).await;
            }
            PairingOutcome::Success
        }

        // Pairing or connection error
        Either::First(Either::First(Err(e))) => {
            match e {
                StartError::PairingFailed(pe) => {
                    error!("[slot{}] Pairing failed: {:?}", slot, pe);
                    rpc_log::error("Pairing failed");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                }
                StartError::Disconnected(reason) => {
                    error!("[slot{}] Disconnected during pairing: {:?}", slot, reason);
                    rpc_log::error("Disconnected during pairing");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                }
            }
            PairingOutcome::Failed
        }

        // Slot command received during pairing
        Either::First(Either::Second(_cmd)) => {
            info!("[slot{}] Command during pairing, disconnecting", slot);
            conn.disconnect();
            let _ = select(
                async {
                    loop {
                        if let ConnectionEvent::Disconnected { .. } = conn.next().await {
                            break;
                        }
                    }
                },
                Timer::after(Duration::from_secs(1)),
            )
            .await;
            PairingOutcome::Cancelled
        }

        // Timeout
        Either::Second(_) => {
            error!("[slot{}] Pairing timed out!", slot);
            rpc_log::error("Pairing timed out");
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
            PairingOutcome::Failed
        }
    }
}

/// Wait for a pairing-related connection event (PairingComplete, PairingFailed, or Disconnected).
async fn wait_for_pairing_event<'a>(
    conn: &Connection<'a, DefaultPacketPool>,
) -> Result<(SecurityLevel, Option<BondInformation>), StartError> {
    loop {
        match conn.next().await {
            ConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                return Ok((security_level, bond));
            }
            ConnectionEvent::PairingFailed(e) => {
                return Err(StartError::PairingFailed(e));
            }
            ConnectionEvent::Disconnected { reason } => {
                return Err(StartError::Disconnected(reason));
            }
            _ => {}
        }
    }
}

/// Store a new bond to flash (via shared mutex) and emit the BondStored event.
async fn store_new_bond(
    flash: &FlashMutex,
    bond_info: &BondInformation,
    active_profile: DeviceProfile,
) {
    let mut f = flash.lock().await;
    match bonding::store_bond(&mut f, bond_info, active_profile.to_id()).await {
        Ok(slot) => {
            info!("Bond stored in slot {}", slot);
            let mut addr_buf = [0u8; 6];
            addr_buf.copy_from_slice(bond_info.identity.bd_addr.raw());
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::BondStored {
                address: addr_buf,
                profile_id: active_profile.to_id(),
            });
        }
        Err(_) => error!("Failed to store bond"),
    }
}
