//! Classic Bluetooth ACL connection lifecycle.
//!
//! State machine:
//!   Idle → Connecting → Connected → Authenticating → Encrypted → Ready
//!
//! This module manages the HCI-level connection to a Classic Bluetooth device,
//! handling ACL link creation, SSP/legacy pairing, and encryption setup.

use bt_hci::param::{BdAddr, ConnHandle, Status};

/// Connection state machine states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConnState {
    /// No active connection.
    Idle,
    /// HCI CreateConnection sent, waiting for ConnectionComplete.
    Connecting,
    /// ACL link established (unencrypted).
    Connected,
    /// AuthenticationRequested sent, waiting for AuthenticationComplete.
    /// Pairing events (LinkKeyRequest, IoCapabilityRequest, etc.) arrive during this phase.
    Authenticating,
    /// SetConnectionEncryption sent, waiting for EncryptionChange.
    Encrypting,
    /// Connection is authenticated and encrypted. Ready for L2CAP.
    Encrypted,
}

/// Information about an active Classic Bluetooth connection.
pub struct ClassicConnection {
    /// Current state of the connection.
    pub state: ConnState,
    /// HCI connection handle (assigned by controller on ConnectionComplete).
    pub handle: Option<ConnHandle>,
    /// Remote device address.
    pub peer_addr: BdAddr,
    /// Whether we have a stored link key for this device.
    pub has_link_key: bool,
    /// Link type from ConnectionComplete (0x01 = ACL).
    pub link_type: u8,
    /// Encryption enabled flag.
    pub encryption_enabled: bool,
}

impl ClassicConnection {
    /// Create a new connection in the Idle state.
    pub fn new(peer_addr: BdAddr) -> Self {
        Self {
            state: ConnState::Idle,
            handle: None,
            peer_addr,
            has_link_key: false,
            link_type: 0,
            encryption_enabled: false,
        }
    }

    /// Transition to Connecting state (after sending CreateConnection).
    pub fn set_connecting(&mut self) {
        self.state = ConnState::Connecting;
    }

    /// Handle ConnectionComplete event.
    ///
    /// Returns `Ok(handle)` if successful, `Err(status)` if the connection failed.
    pub fn on_connection_complete(
        &mut self,
        status: Status,
        handle: ConnHandle,
        link_type: u8,
        encryption_enabled: u8,
    ) -> Result<ConnHandle, Status> {
        if status != Status::SUCCESS {
            self.state = ConnState::Idle;
            self.handle = None;
            return Err(status);
        }

        self.state = ConnState::Connected;
        self.handle = Some(handle);
        self.link_type = link_type;
        self.encryption_enabled = encryption_enabled != 0;
        Ok(handle)
    }

    /// Transition to Authenticating state (after sending AuthenticationRequested).
    pub fn set_authenticating(&mut self) {
        self.state = ConnState::Authenticating;
    }

    /// Handle AuthenticationComplete event.
    pub fn on_authentication_complete(&mut self, status: Status) -> Result<(), Status> {
        if status != Status::SUCCESS {
            // Authentication failed — connection may still be up but not trusted
            return Err(status);
        }
        // Stay in Authenticating state until we transition to Encrypting
        Ok(())
    }

    /// Transition to Encrypting state (after sending SetConnectionEncryption).
    pub fn set_encrypting(&mut self) {
        self.state = ConnState::Encrypting;
    }

    /// Handle EncryptionChange event.
    pub fn on_encryption_change(
        &mut self,
        status: Status,
        encryption_enabled: u8,
    ) -> Result<(), Status> {
        if status != Status::SUCCESS {
            return Err(status);
        }
        self.encryption_enabled = encryption_enabled != 0;
        if self.encryption_enabled {
            self.state = ConnState::Encrypted;
        }
        Ok(())
    }

    /// Handle disconnection.
    pub fn on_disconnected(&mut self) {
        self.state = ConnState::Idle;
        self.handle = None;
        self.encryption_enabled = false;
    }

    /// Check if the connection is in a state where L2CAP can be used.
    pub fn is_ready(&self) -> bool {
        self.state == ConnState::Encrypted
    }
}

/// Storage for a connection slot, including L2CAP channel state.
pub struct ConnectionStorage {
    pub conn: ClassicConnection,
    /// Whether this slot is in use.
    pub active: bool,
}

impl Default for ConnectionStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl ConnectionStorage {
    pub const fn new() -> Self {
        Self {
            conn: ClassicConnection {
                state: ConnState::Idle,
                handle: None,
                peer_addr: // Safety: BdAddr is repr(transparent) over [u8; 6], zeroed is valid
                unsafe { core::mem::zeroed() },
                has_link_key: false,
                link_type: 0,
                encryption_enabled: false,
            },
            active: false,
        }
    }
}
