//! Link key storage trait for Classic Bluetooth bonding.
//!
//! Classic Bluetooth uses link keys (128-bit symmetric keys) for authentication
//! and encryption. These are exchanged during pairing and must be stored persistently
//! for bonded devices to reconnect without re-pairing.

use bt_hci::param::BdAddr;

/// Information about a stored link key.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinkKeyInfo {
    /// The 128-bit link key.
    pub key: [u8; 16],
    /// Link key type (from HCI LinkKeyNotification event).
    /// 0x00 = Combination, 0x01 = LocalUnit, 0x03 = AuthenticatedCombination,
    /// 0x04 = UnauthenticatedP256, 0x05 = AuthenticatedP256, etc.
    pub key_type: u8,
}

/// Trait for persistent link key storage.
///
/// Implementations should store link keys in non-volatile memory (flash, EEPROM, etc.)
/// so that bonded devices can reconnect without re-pairing after power cycles.
pub trait LinkKeyStore {
    /// Load the link key for a given Bluetooth address.
    ///
    /// Returns `None` if no key is stored for this address.
    fn load(&self, addr: &BdAddr) -> Option<LinkKeyInfo>;

    /// Store a link key for a given Bluetooth address.
    ///
    /// If a key already exists for this address, it should be overwritten.
    fn store(&mut self, addr: &BdAddr, key: LinkKeyInfo);

    /// Remove the stored link key for a given Bluetooth address.
    fn remove(&mut self, addr: &BdAddr);
}
