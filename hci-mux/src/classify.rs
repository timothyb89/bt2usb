//! HCI event classification for routing between LE and Classic stacks.
//!
//! The HCI specification defines events that are:
//! - LE-only (subevent of LE Meta Event 0x3E)
//! - Classic-only (ConnectionComplete, LinkKeyRequest, etc.)
//! - Shared (DisconnectionComplete, EncryptionChange) — routed by connection handle
//! - Command responses (CommandComplete, CommandStatus) — routed by opcode

use bt_hci::event::EventKind;
use bt_hci::param::ConnHandle;

/// Where an HCI packet should be routed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketTarget {
    /// Route to the LE (BLE) stack only.
    Le,
    /// Route to the Classic (BR/EDR) stack only.
    Classic,
    /// Route based on the connection handle (could be either stack).
    ByHandle(ConnHandle),
    /// Route to both stacks.
    Both,
}

/// Classify an HCI event by its EventKind to determine which stack should receive it.
///
/// This follows the Bluetooth Core Spec v5.4 event definitions:
/// - LE Meta Events (0x3E) → always LE
/// - Classic-specific events (ConnectionComplete, pairing events) → always Classic
/// - Shared events (DisconnectionComplete, etc.) → route by connection handle
/// - Command responses → route by opcode (handled separately in the runner)
pub fn classify_event(kind: EventKind) -> PacketTarget {
    match kind {
        // LE-only events
        EventKind::Le => PacketTarget::Le,

        // Classic-only events — these never occur for LE connections
        EventKind::ConnectionComplete => PacketTarget::Classic,
        EventKind::ConnectionRequest => PacketTarget::Classic,
        EventKind::AuthenticationComplete => PacketTarget::Classic,
        EventKind::RemoteNameRequestComplete => PacketTarget::Classic,
        EventKind::ReadRemoteSupportedFeaturesComplete => PacketTarget::Classic,
        EventKind::ReadRemoteVersionInformationComplete => PacketTarget::Classic,
        EventKind::LinkKeyRequest => PacketTarget::Classic,
        EventKind::LinkKeyNotification => PacketTarget::Classic,
        EventKind::PinCodeRequest => PacketTarget::Classic,
        EventKind::RoleChange => PacketTarget::Classic,
        EventKind::ModeChange => PacketTarget::Classic,
        EventKind::IoCapabilityRequest => PacketTarget::Classic,
        EventKind::IoCapabilityResponse => PacketTarget::Classic,
        EventKind::UserConfirmationRequest => PacketTarget::Classic,
        EventKind::UserPasskeyRequest => PacketTarget::Classic,
        EventKind::UserPasskeyNotification => PacketTarget::Classic,
        EventKind::SimplePairingComplete => PacketTarget::Classic,
        EventKind::LinkSupervisionTimeoutChanged => PacketTarget::Classic,
        EventKind::SynchronousConnectionComplete => PacketTarget::Classic,
        EventKind::SynchronousConnectionChanged => PacketTarget::Classic,
        EventKind::InquiryComplete => PacketTarget::Classic,
        EventKind::InquiryResult => PacketTarget::Classic,
        EventKind::InquiryResultWithRssi => PacketTarget::Classic,
        EventKind::ExtendedInquiryResult => PacketTarget::Classic,

        // Command responses — handled separately by opcode in the runner.
        // If they reach this function, route to both as a fallback.
        EventKind::CommandComplete | EventKind::CommandStatus => PacketTarget::Both,

        // Shared events that need handle-based routing are handled by the runner
        // before calling this function. If they arrive here, route to both.
        EventKind::DisconnectionComplete
        | EventKind::EncryptionChangeV1
        | EventKind::EncryptionChangeV2
        | EventKind::EncryptionKeyRefreshComplete
        | EventKind::NumberOfCompletedPackets => PacketTarget::Both,

        // Vendor-specific and hardware errors go to both
        EventKind::Vendor | EventKind::HardwareError => PacketTarget::Both,

        // Unknown events — send to both to avoid dropping anything important
        _ => PacketTarget::Both,
    }
}

/// Determine if an HCI command opcode belongs to the LE command group.
///
/// LE commands have OGF = 0x08 (LE Controller). All other OGFs are either
/// shared or Classic-specific.
pub fn is_le_opcode(opcode: u16) -> bool {
    let ogf = (opcode >> 10) & 0x3F;
    ogf == 0x08 // LE Controller commands
}
