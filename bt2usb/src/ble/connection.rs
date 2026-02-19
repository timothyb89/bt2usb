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
use embassy_futures::select::{select, Either};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_time::{Duration, Timer};
use trouble_host::connection::{ConnectConfig, ScanConfig};
use trouble_host::prelude::*;

use crate::ble_state::{BleCommand, BleEvent, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL};
use crate::bonding::{self, LoadedBond};
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::protocol::ConnectionState;
use crate::rpc_log;
use crate::FLASH_SIZE;

use super::gatt::{self, GattSessionResult};

const MAX_PAIRING_RETRIES: u8 = 5;

/// Error during the connection/pairing phase.
enum StartError {
    PairingFailed(trouble_host::Error),
    Disconnected(bt_hci::param::Status),
}

/// Connect to a BLE HID device, pair, discover GATT, and run the HID report loop.
///
/// Bond storage is done directly via flash (all on Core 0, no cross-core channel needed).
/// Returns `Some(BleCommand)` if interrupted by a command that should be re-dispatched,
/// or `None` if the connection ended naturally or after max retries.
pub async fn ble_connect_and_run<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    target: Address,
    active_profile: &mut DeviceProfile,
    has_stored_bond: bool,
    loaded_bonds: &[LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> Option<BleCommand> {
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));
    rpc_log::info("Connecting to BLE device");

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
            "Connection attempt {} of {}",
            pairing_attempts, MAX_PAIRING_RETRIES
        );

        // Phase 1: Establish BLE connection
        let conn = match connect_to_device(central, &config).await {
            ConnectOutcome::Connected(conn) => conn,
            ConnectOutcome::RetryableError => {
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    error!("Max connection retries exceeded");
                    rpc_log::error("Max connection retries exceeded");
                    return None;
                }
                Timer::after(Duration::from_millis(500)).await;
                continue 'connect;
            }
            ConnectOutcome::InterruptedByCommand(cmd) => return Some(cmd),
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
        match await_pairing(&conn, flash, *active_profile).await {
            PairingOutcome::Success => {}
            PairingOutcome::Failed => {
                if pairing_attempts >= MAX_PAIRING_RETRIES {
                    error!("Max pairing retries exceeded");
                    rpc_log::error("Max pairing retries exceeded");
                    return None;
                }
                Timer::after(Duration::from_millis(500)).await;
                continue 'connect;
            }
            PairingOutcome::InterruptedByCommand(cmd) => return Some(cmd),
            PairingOutcome::Cancelled => return None,
        }

        // Phase 4: GATT discovery and HID report loop
        match gatt::run_gatt_session(
            stack,
            &conn,
            flash,
            active_profile,
            loaded_bonds,
            active_device_pref,
        )
        .await
        {
            GattSessionResult::ConnectionLost => {
                // Connection dropped, break to outer state machine
                break;
            }
            GattSessionResult::InterruptedByCommand(cmd) => {
                return Some(cmd);
            }
            GattSessionResult::Ended => {
                break;
            }
        }
    }

    None
}

// ============ Phase 1: Connection ============

enum ConnectOutcome<'a> {
    Connected(Connection<'a, DefaultPacketPool>),
    RetryableError,
    InterruptedByCommand(BleCommand),
    Cancelled,
}

/// Attempt to establish a BLE connection, with command interruption support.
async fn connect_to_device<'a, C: Controller>(
    central: &mut Central<'a, C, DefaultPacketPool>,
    config: &ConnectConfig<'_>,
) -> ConnectOutcome<'a> {
    match select(central.connect(config), BLE_CMD_CHANNEL.receive()).await {
        Either::First(Ok(conn)) => {
            info!("Connected to HID device!");
            rpc_log::info("Connected to BLE device");
            let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connected));
            ConnectOutcome::Connected(conn)
        }
        Either::First(Err(e)) => {
            error!("Connection failed: {:?}", defmt::Debug2Format(&e));
            rpc_log::error("Connection attempt failed");
            ConnectOutcome::RetryableError
        }
        Either::Second(cmd) => {
            info!("Command received during connection attempt: {:?}", cmd);
            match cmd {
                BleCommand::Connect { .. } | BleCommand::AutoConnect => {
                    ConnectOutcome::InterruptedByCommand(cmd)
                }
                BleCommand::Disconnect | BleCommand::StopScan => ConnectOutcome::Cancelled,
                _ => ConnectOutcome::Cancelled,
            }
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
        // With existing bond, peripheral will automatically request encryption.
        // We just wait for PairingComplete event.
        info!("Existing bond detected - waiting for automatic re-encryption...");
        rpc_log::info("Re-encryption in progress");
        SecurityOutcome::Ready
    } else {
        // No bond exists — initiate pairing
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
    InterruptedByCommand(BleCommand),
    Cancelled,
}

/// Wait for pairing/re-encryption to complete, with timeout and command interruption.
///
/// On success with a new bond, stores the bond info to flash.
async fn await_pairing<'a>(
    conn: &Connection<'a, DefaultPacketPool>,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    active_profile: DeviceProfile,
) -> PairingOutcome {
    let pairing_timeout = Timer::after(Duration::from_secs(15));

    let pairing_result = select(
        select(
            wait_for_pairing_event(conn),
            BLE_CMD_CHANNEL.receive(),
        ),
        pairing_timeout,
    )
    .await;

    match pairing_result {
        // Pairing event received
        Either::First(Either::First(Ok((security_level, bond)))) => {
            info!("Pairing complete! Level: {:?}", security_level);
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
                    error!("Pairing failed: {:?}", pe);
                    rpc_log::error("Pairing failed");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                }
                StartError::Disconnected(reason) => {
                    error!("Disconnected during pairing: {:?}", reason);
                    rpc_log::error("Disconnected during pairing");
                    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::PairingFailed);
                }
            }
            PairingOutcome::Failed
        }

        // Command received during pairing
        Either::First(Either::Second(cmd)) => {
            info!("Command received during pairing: {:?}", cmd);
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

            match cmd {
                BleCommand::Connect { .. } | BleCommand::AutoConnect => {
                    PairingOutcome::InterruptedByCommand(cmd)
                }
                _ => PairingOutcome::Cancelled,
            }
        }

        // Timeout
        Either::Second(_) => {
            error!("Pairing timed out!");
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

/// Store a new bond to flash and emit the BondStored event.
async fn store_new_bond(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    bond_info: &BondInformation,
    active_profile: DeviceProfile,
) {
    match bonding::store_bond(flash, bond_info, active_profile.to_id()).await {
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
