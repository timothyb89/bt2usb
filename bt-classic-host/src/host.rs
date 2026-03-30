//! Classic Bluetooth host runner and event dispatch.
//!
//! The runner reads HCI events from the controller and dispatches them to
//! the connection state machine, pairing handler, and L2CAP layer.

use core::cell::RefCell;

use bt_hci::cmd::link_control::*;
use bt_hci::controller::{Controller, ControllerCmdSync};
use bt_hci::event::EventKind;
use bt_hci::param::*;
use bt_hci::ControllerToHostPacket;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::connection::{ClassicConnection, ConnectionStorage};
use crate::error::Error;
use crate::link_key::{LinkKeyInfo, LinkKeyStore};
use crate::pairing::{self, PairingContext, PairingState};

/// Signal used to notify user code of connection state changes.
pub type ConnSignal = Signal<CriticalSectionRawMutex, ConnEvent>;

/// Connection events signaled to user code.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConnEvent {
    /// ACL connection established.
    Connected(ConnHandle),
    /// Connection is authenticated and encrypted, ready for L2CAP.
    Encrypted(ConnHandle),
    /// Connection failed.
    ConnectionFailed,
    /// Disconnected.
    Disconnected,
}

/// Resources for the Classic Bluetooth host. Allocate statically.
pub struct HostResources<const CONNS: usize> {
    pub(crate) connections: [ConnectionStorage; CONNS],
    pub(crate) pairing: [PairingContext; CONNS],
    #[allow(dead_code)] // Will be used for async connection event notification
    pub(crate) conn_signals: [ConnSignal; CONNS],
}

impl<const CONNS: usize> Default for HostResources<CONNS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CONNS: usize> HostResources<CONNS> {
    pub fn new() -> Self {
        Self {
            connections: [const { ConnectionStorage::new() }; CONNS],
            pairing: [const {
                PairingContext {
                    state: PairingState::Idle,
                    remote_io_cap: None,
                    peer_addr: // Safety: BdAddr is repr(transparent) over [u8; 6]
                    unsafe { core::mem::zeroed() },
                    success: false,
                    new_link_key: None,
                    link_key_type: 0,
                }
            }; CONNS],
            conn_signals: [const { Signal::new() }; CONNS],
        }
    }
}

/// The Classic Bluetooth host runner.
///
/// Reads HCI events and drives the connection/pairing state machines.
pub struct ClassicRunner<'a, C, L, const CONNS: usize> {
    controller: &'a C,
    resources: &'a mut HostResources<CONNS>,
    link_keys: &'a RefCell<L>,
}

impl<'a, C, L, const CONNS: usize> ClassicRunner<'a, C, L, CONNS>
where
    C: Controller
        + ControllerCmdSync<CreateConnection>
        + ControllerCmdSync<AuthenticationRequested>
        + ControllerCmdSync<SetConnectionEncryption>
        + ControllerCmdSync<LinkKeyRequestReply>
        + ControllerCmdSync<LinkKeyRequestNegativeReply>
        + ControllerCmdSync<IoCapabilityRequestReply>
        + ControllerCmdSync<UserConfirmationRequestReply>,
    L: LinkKeyStore,
{
    /// Create a new runner.
    pub fn new(
        controller: &'a C,
        resources: &'a mut HostResources<CONNS>,
        link_keys: &'a RefCell<L>,
    ) -> Self {
        Self {
            controller,
            resources,
            link_keys,
        }
    }

    /// Connect to a Classic Bluetooth device.
    ///
    /// Initiates ACL connection, then drives through authentication and encryption.
    /// Returns when the connection is fully encrypted and ready for L2CAP.
    pub async fn connect(&mut self, addr: &BdAddr) -> Result<ConnHandle, Error<C::Error>> {
        // Find a free slot
        let slot = self
            .resources
            .connections
            .iter_mut()
            .enumerate()
            .find(|(_, s)| !s.active)
            .map(|(i, _)| i)
            .ok_or(Error::NoFreeSlots)?;

        let conn = &mut self.resources.connections[slot];
        conn.active = true;
        conn.conn = ClassicConnection::new(*addr);
        conn.conn.has_link_key = self.link_keys.borrow().load(addr).is_some();

        self.resources.pairing[slot] = PairingContext::new(*addr);

        // Send CreateConnection
        // Enable all standard ACL packet types for best throughput.
        let mut pkt_type = PacketType::default();
        pkt_type = pkt_type
            .set_dh1_may_be_used(true)
            .set_dm3_may_be_used(true)
            .set_dh3_may_be_used(true)
            .set_dm5_may_be_used(true)
            .set_dh5_may_be_used(true);

        conn.conn.set_connecting();
        self.controller
            .exec(&CreateConnection::new(
                *addr,
                pkt_type,
                PageScanRepetitionMode::R1,
                0, // reserved
                ClockOffset::default(),
                AllowRoleSwitch::NotAllowed,
            ))
            .await
            .map_err(Error::Command)?;

        // Now run the event loop until we reach Encrypted state or fail
        self.run_until_encrypted(slot).await
    }

    /// Run the HCI event loop until the given slot reaches Encrypted state.
    async fn run_until_encrypted(
        &mut self,
        target_slot: usize,
    ) -> Result<ConnHandle, Error<C::Error>> {
        const MAX_HCI_PACKET_LEN: usize = 259;
        let mut rx = [0u8; MAX_HCI_PACKET_LEN];

        loop {
            let rx_ref = unsafe { core::slice::from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };
            let packet = self.controller.read(rx_ref).await.map_err(Error::Hci)?;

            match packet {
                ControllerToHostPacket::Event(event) => {
                    match event.kind {
                        EventKind::ConnectionComplete => {
                            // [status(1), handle(2), bdaddr(6), link_type(1), encryption(1)]
                            if event.data.len() >= 11 {
                                let status = Status::new(event.data[0]);
                                let handle_raw =
                                    u16::from_le_bytes([event.data[1], event.data[2]]) & 0x0FFF;
                                let handle = ConnHandle::new(handle_raw);
                                let link_type = event.data[9];
                                let encryption = event.data[10];

                                let conn = &mut self.resources.connections[target_slot];
                                match conn
                                    .conn
                                    .on_connection_complete(status, handle, link_type, encryption)
                                {
                                    Ok(_handle) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::info!("[classic] Connected, handle={}", handle_raw);

                                        // Start authentication
                                        conn.conn.set_authenticating();
                                        self.controller
                                            .exec(&AuthenticationRequested::new(handle))
                                            .await
                                            .map_err(Error::Command)?;
                                    }
                                    Err(_status) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::warn!("[classic] Connection failed");
                                        conn.active = false;
                                        return Err(Error::ConnectionFailed);
                                    }
                                }
                            }
                        }

                        EventKind::LinkKeyRequest => {
                            // [bdaddr(6)]
                            if event.data.len() >= 6 {
                                let addr = BdAddr::new([
                                    event.data[0],
                                    event.data[1],
                                    event.data[2],
                                    event.data[3],
                                    event.data[4],
                                    event.data[5],
                                ]);
                                let stored_key = self.link_keys.borrow().load(&addr).map(|k| k.key);

                                if let Some(key) = stored_key {
                                    #[cfg(feature = "defmt")]
                                    defmt::info!("[classic] Replying with stored link key");
                                    self.controller
                                        .exec(&LinkKeyRequestReply::new(addr, key))
                                        .await
                                        .map_err(Error::Command)?;
                                } else {
                                    #[cfg(feature = "defmt")]
                                    defmt::info!("[classic] No stored link key, negative reply (will trigger pairing)");
                                    self.controller
                                        .exec(&LinkKeyRequestNegativeReply::new(addr))
                                        .await
                                        .map_err(Error::Command)?;
                                    // SSP pairing will follow
                                    self.resources.pairing[target_slot].start();
                                }
                            }
                        }

                        EventKind::IoCapabilityRequest => {
                            // [bdaddr(6)]
                            if event.data.len() >= 6 {
                                let addr = BdAddr::new([
                                    event.data[0],
                                    event.data[1],
                                    event.data[2],
                                    event.data[3],
                                    event.data[4],
                                    event.data[5],
                                ]);
                                #[cfg(feature = "defmt")]
                                defmt::info!(
                                    "[classic] IoCapabilityRequest, replying NoInputNoOutput"
                                );
                                self.controller
                                    .exec(&IoCapabilityRequestReply::new(
                                        addr,
                                        pairing::OUR_IO_CAPABILITY,
                                        OobDataPresent::NotPresent,
                                        AuthenticationRequirements::MitmNotRequiredDedicatedBonding,
                                    ))
                                    .await
                                    .map_err(Error::Command)?;
                            }
                        }

                        EventKind::IoCapabilityResponse => {
                            // [bdaddr(6), io_cap(1), oob(1), auth_req(1)]
                            if event.data.len() >= 9 {
                                // Parse remote IO capability from raw byte
                                let io_cap = match event.data[6] {
                                    0x00 => IoCapability::DisplayOnly,
                                    0x01 => IoCapability::DisplayYesNo,
                                    0x02 => IoCapability::KeyboardOnly,
                                    _ => IoCapability::NoInputNoOutput,
                                };
                                self.resources.pairing[target_slot]
                                    .on_io_capability_response(io_cap);
                                #[cfg(feature = "defmt")]
                                defmt::info!("[classic] Remote IO capability: {:?}", io_cap);
                            }
                        }

                        EventKind::UserConfirmationRequest => {
                            // [bdaddr(6), numeric_value(4)]
                            if event.data.len() >= 6 {
                                let addr = BdAddr::new([
                                    event.data[0],
                                    event.data[1],
                                    event.data[2],
                                    event.data[3],
                                    event.data[4],
                                    event.data[5],
                                ]);
                                #[cfg(feature = "defmt")]
                                defmt::info!(
                                    "[classic] Auto-accepting user confirmation (Just Works)"
                                );
                                self.controller
                                    .exec(&UserConfirmationRequestReply::new(addr))
                                    .await
                                    .map_err(Error::Command)?;
                            }
                        }

                        EventKind::SimplePairingComplete => {
                            // [status(1), bdaddr(6)]
                            if !event.data.is_empty() {
                                let success = event.data[0] == 0x00;
                                self.resources.pairing[target_slot]
                                    .on_simple_pairing_complete(success);
                                #[cfg(feature = "defmt")]
                                defmt::info!(
                                    "[classic] SimplePairingComplete, success={}",
                                    success
                                );
                            }
                        }

                        EventKind::LinkKeyNotification => {
                            // [bdaddr(6), link_key(16), key_type(1)]
                            if event.data.len() >= 23 {
                                let addr = BdAddr::new([
                                    event.data[0],
                                    event.data[1],
                                    event.data[2],
                                    event.data[3],
                                    event.data[4],
                                    event.data[5],
                                ]);
                                let mut key = [0u8; 16];
                                key.copy_from_slice(&event.data[6..22]);
                                let key_type = event.data[22];

                                self.resources.pairing[target_slot]
                                    .on_link_key_notification(key, key_type);

                                // Store the new link key
                                self.link_keys
                                    .borrow_mut()
                                    .store(&addr, LinkKeyInfo { key, key_type });
                                #[cfg(feature = "defmt")]
                                defmt::info!("[classic] New link key stored, type={}", key_type);
                            }
                        }

                        EventKind::AuthenticationComplete => {
                            // [status(1), handle(2)]
                            if event.data.len() >= 3 {
                                let status = Status::new(event.data[0]);
                                let conn = &mut self.resources.connections[target_slot];

                                match conn.conn.on_authentication_complete(status) {
                                    Ok(()) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::info!("[classic] Authentication complete, enabling encryption");

                                        // Enable encryption
                                        if let Some(handle) = conn.conn.handle {
                                            conn.conn.set_encrypting();
                                            self.controller
                                                .exec(&SetConnectionEncryption::new(handle, true))
                                                .await
                                                .map_err(Error::Command)?;
                                        }
                                    }
                                    Err(_status) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::warn!("[classic] Authentication failed");
                                        conn.active = false;
                                        return Err(Error::AuthenticationFailed);
                                    }
                                }
                            }
                        }

                        EventKind::EncryptionChangeV1 | EventKind::EncryptionChangeV2 => {
                            // [status(1), handle(2), encryption_enabled(1)]
                            if event.data.len() >= 4 {
                                let status = Status::new(event.data[0]);
                                let encryption_enabled = event.data[3];

                                let conn = &mut self.resources.connections[target_slot];
                                match conn.conn.on_encryption_change(status, encryption_enabled) {
                                    Ok(()) if conn.conn.is_ready() => {
                                        #[cfg(feature = "defmt")]
                                        defmt::info!(
                                            "[classic] Encryption enabled! Connection ready."
                                        );
                                        return Ok(conn.conn.handle.unwrap());
                                    }
                                    Ok(()) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::warn!(
                                            "[classic] EncryptionChange but not encrypted"
                                        );
                                        return Err(Error::EncryptionFailed);
                                    }
                                    Err(_status) => {
                                        #[cfg(feature = "defmt")]
                                        defmt::warn!("[classic] Encryption failed");
                                        conn.active = false;
                                        return Err(Error::EncryptionFailed);
                                    }
                                }
                            }
                        }

                        EventKind::DisconnectionComplete => {
                            // Connection lost during setup
                            let conn = &mut self.resources.connections[target_slot];
                            conn.conn.on_disconnected();
                            conn.active = false;
                            return Err(Error::Disconnected);
                        }

                        // Ignore other events during connection setup
                        _ => {}
                    }
                }

                // ACL data during connection setup — buffer for later L2CAP processing
                ControllerToHostPacket::Acl(_acl) => {
                    // During connection setup, we may receive L2CAP signaling from the remote.
                    // For now, drop it — L2CAP setup happens after connect() returns.
                    #[cfg(feature = "defmt")]
                    defmt::debug!("[classic] ACL data during connection setup (ignored)");
                }

                _ => {}
            }
        }
    }
}
