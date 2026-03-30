//! Classic Bluetooth pairing: Secure Simple Pairing (SSP) and legacy PIN.
//!
//! SSP flow:
//!   1. AuthenticationRequested → controller initiates pairing
//!   2. IoCapabilityRequest → we reply with our IO capabilities
//!   3. IoCapabilityResponse → we learn the remote's capabilities
//!   4. UserConfirmationRequest → we auto-accept (NoInputNoOutput mode)
//!   5. SimplePairingComplete → pairing done (success or failure)
//!   6. LinkKeyNotification → new link key generated, store it
//!   7. AuthenticationComplete → authentication finished
//!
//! For bonded devices with a stored link key:
//!   1. AuthenticationRequested → controller checks link key
//!   2. LinkKeyRequest → we reply with stored key
//!   3. AuthenticationComplete → success (no SSP needed)
//!
//! We use NoInputNoOutput IO capability since the Pico has no display or keyboard.
//! This results in "Just Works" pairing (no numeric comparison or passkey).

use bt_hci::param::{BdAddr, IoCapability};

/// Our IO capability for SSP pairing.
///
/// NoInputNoOutput results in "Just Works" association model.
/// This is appropriate for a headless bridge device.
pub const OUR_IO_CAPABILITY: IoCapability = IoCapability::NoInputNoOutput;

/// OOB (Out of Band) data presence flag.
/// We don't support OOB pairing.
pub const OUR_OOB_DATA: u8 = 0x00; // Not present

/// Authentication requirements.
/// MITM not required (since we use NoInputNoOutput anyway).
pub const OUR_AUTH_REQUIREMENTS: u8 = 0x03; // MITM not required, dedicated bonding

/// Pairing state tracker for a connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingState {
    /// No pairing in progress.
    Idle,
    /// Waiting for IoCapabilityRequest from controller.
    WaitingIoCap,
    /// Sent IoCapabilityRequestReply, waiting for remote's response.
    IoCapsExchanged,
    /// Waiting for UserConfirmationRequest.
    WaitingConfirmation,
    /// Sent UserConfirmationRequestReply, waiting for SimplePairingComplete.
    WaitingComplete,
    /// Pairing complete, waiting for LinkKeyNotification.
    WaitingLinkKey,
    /// Pairing finished (success or failure).
    Done,
}

/// Tracks the pairing state for a single connection.
pub struct PairingContext {
    pub state: PairingState,
    /// Remote device's IO capability (learned from IoCapabilityResponse).
    pub remote_io_cap: Option<IoCapability>,
    /// The remote BD_ADDR being paired with.
    pub peer_addr: BdAddr,
    /// Whether pairing succeeded.
    pub success: bool,
    /// Newly generated link key (from LinkKeyNotification).
    pub new_link_key: Option<[u8; 16]>,
    /// Link key type from LinkKeyNotification.
    pub link_key_type: u8,
}

impl PairingContext {
    pub fn new(peer_addr: BdAddr) -> Self {
        Self {
            state: PairingState::Idle,
            remote_io_cap: None,
            peer_addr,
            success: false,
            new_link_key: None,
            link_key_type: 0,
        }
    }

    /// Start a new pairing attempt.
    pub fn start(&mut self) {
        self.state = PairingState::WaitingIoCap;
        self.remote_io_cap = None;
        self.success = false;
        self.new_link_key = None;
    }

    /// Handle IoCapabilityResponse event (remote's capabilities).
    pub fn on_io_capability_response(&mut self, io_cap: IoCapability) {
        self.remote_io_cap = Some(io_cap);
        self.state = PairingState::WaitingConfirmation;
    }

    /// Handle SimplePairingComplete event.
    pub fn on_simple_pairing_complete(&mut self, success: bool) {
        self.success = success;
        self.state = if success {
            PairingState::WaitingLinkKey
        } else {
            PairingState::Done
        };
    }

    /// Handle LinkKeyNotification event.
    pub fn on_link_key_notification(&mut self, key: [u8; 16], key_type: u8) {
        self.new_link_key = Some(key);
        self.link_key_type = key_type;
        self.state = PairingState::Done;
    }
}
