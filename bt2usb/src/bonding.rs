//! Bond Storage (BLE + Classic BT)
//!
//! Stores bonding information to flash memory so devices stay paired
//! across power cycles. Uses sequential-storage for wear-leveled writes.
//!
//! BLE bonds use keys 0-9, Classic BT bonds use keys 128-137.
//! Both share the same 16KB flash region (sequential-storage handles it).

use core::ops::Range;
use defmt::{debug, error, info, warn};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use heapless::Vec;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{
    fetch_item, remove_all_items, remove_item, store_item, SerializationError, Value,
};
use trouble_host::prelude::{BdAddr, BondInformation, Identity, LongTermKey, SecurityLevel};

/// Maximum number of bonded devices we support
pub const MAX_BONDS: usize = 10;

/// Flash storage range for bonding data
/// Uses 16KB (4 x 4KB erase sectors) for wear leveling with up to 10 bonds
/// RP2040 has 2MB flash, allocated at offset 0x1F0000
const BONDING_FLASH_OFFSET: u32 = 2 * 1024 * 1024 - 64 * 1024; // 0x1F0000
const BONDING_FLASH_SIZE: u32 = 16 * 1024; // 16KB

fn flash_range() -> Range<u32> {
    BONDING_FLASH_OFFSET..(BONDING_FLASH_OFFSET + BONDING_FLASH_SIZE)
}

/// Value for storing bond info: all the data needed to restore a bond
#[derive(Clone)]
pub struct StoredBondInfo {
    pub addr: [u8; 6],          // BdAddr raw bytes
    pub ltk: [u8; 16],          // LongTermKey raw bytes
    pub security_level: u8,     // 0=None, 1=Encrypted, 2=EncryptedAuthenticated
    pub profile_id: u8,         // DeviceProfile discriminant
    pub auto_connect: bool,     // Whether to automatically connect on startup
    pub ediv: u16,              // Encrypted diversifier (0 for LESC, non-zero for legacy)
    pub rand: [u8; 8],          // Random number (zeros for LESC, non-zero for legacy)
    pub encryption_key_len: u8, // Encryption key length in bytes (16 for LESC)
    pub name: [u8; 32],         // BLE device name (UTF-8, from advertisement/scan response)
    pub name_len: u8,           // Actual name length (0 = unknown)
}

impl<'a> Value<'a> for StoredBondInfo {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // 6 + 16 + 1 + 1 + 1 + 2 + 8 + 1 + 32 + 1 = 69 bytes
        if buffer.len() < 69 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(&self.addr);
        buffer[6..22].copy_from_slice(&self.ltk);
        buffer[22] = self.security_level;
        buffer[23] = self.profile_id;
        buffer[24] = if self.auto_connect { 1 } else { 0 };
        buffer[25..27].copy_from_slice(&self.ediv.to_le_bytes());
        buffer[27..35].copy_from_slice(&self.rand);
        buffer[35] = self.encryption_key_len;
        buffer[36..68].copy_from_slice(&self.name);
        buffer[68] = self.name_len;
        Ok(69)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 23 {
            return Err(SerializationError::BufferTooSmall);
        }
        let mut addr = [0u8; 6];
        let mut ltk = [0u8; 16];
        addr.copy_from_slice(&buffer[0..6]);
        ltk.copy_from_slice(&buffer[6..22]);
        let security_level = buffer[22];
        // Handle old 23-byte records that lack profile_id
        let profile_id = if buffer.len() >= 24 { buffer[23] } else { 0 };
        // Handle old 24-byte records that lack auto_connect
        let auto_connect = if buffer.len() >= 25 {
            buffer[24] != 0
        } else {
            false
        };
        // Handle old records that lack legacy pairing fields
        let ediv = if buffer.len() >= 27 {
            u16::from_le_bytes([buffer[25], buffer[26]])
        } else {
            0
        };
        let mut rand = [0u8; 8];
        if buffer.len() >= 35 {
            rand.copy_from_slice(&buffer[27..35]);
        }
        let encryption_key_len = if buffer.len() >= 36 { buffer[35] } else { 16 };
        // Handle old records that lack device name (pre-v69)
        let mut name = [0u8; 32];
        let name_len = if buffer.len() >= 69 {
            name.copy_from_slice(&buffer[36..68]);
            buffer[68]
        } else {
            0
        };
        Ok(StoredBondInfo {
            addr,
            ltk,
            security_level,
            profile_id,
            auto_connect,
            ediv,
            rand,
            encryption_key_len,
            name,
            name_len,
        })
    }
}

/// A loaded bond paired with its device profile ID, auto-connect flag, and device name.
pub struct LoadedBond {
    pub bond: BondInformation,
    pub profile_id: u8,
    pub auto_connect: bool,
    pub name: [u8; 32],
    pub name_len: u8,
}

/// Load all stored bonds from flash
pub async fn load_bonds(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Vec<LoadedBond, MAX_BONDS> {
    let mut bonds = Vec::new();
    let mut buffer = [0u8; 128];

    // Try to fetch bond from each slot
    for slot in 0..MAX_BONDS as u8 {
        match fetch_item::<u8, StoredBondInfo, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &slot,
        )
        .await
        {
            Ok(Some(stored)) => {
                // Convert back to BondInformation
                let security_level = match stored.security_level {
                    0 => SecurityLevel::NoEncryption,
                    1 => SecurityLevel::Encrypted,
                    _ => SecurityLevel::EncryptedAuthenticated,
                };

                let bond = BondInformation {
                    identity: Identity {
                        bd_addr: BdAddr::new(stored.addr),
                        irk: None,
                    },
                    security_level,
                    is_bonded: true,
                    ltk: LongTermKey::from_le_bytes(stored.ltk),
                    ediv: stored.ediv,
                    rand: stored.rand,
                    encryption_key_len: stored.encryption_key_len,
                };

                debug!(
                    "Loaded bond from slot {} (profile {}): {:?}",
                    slot, stored.profile_id, stored.addr
                );
                let _ = bonds.push(LoadedBond {
                    bond,
                    profile_id: stored.profile_id,
                    auto_connect: stored.auto_connect,
                    name: stored.name,
                    name_len: stored.name_len,
                });
            }
            Ok(None) => {
                // Slot empty, continue
            }
            Err(e) => {
                warn!(
                    "Error reading bond slot {}: {:?}",
                    slot,
                    defmt::Debug2Format(&e)
                );
            }
        }
    }

    info!("Loaded {} bonds from flash", bonds.len());
    bonds
}

/// Store a bond to flash after successful pairing.
/// `device_name` is the BLE device name from advertisement/scan response (may be empty).
/// Returns the slot it was stored in, or error if no slots available.
pub async fn store_bond(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    bond: &BondInformation,
    profile_id: u8,
    device_name: &[u8; 32],
    device_name_len: u8,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    // Find an empty slot or reuse slot with same address
    let mut target_slot: Option<u8> = None;

    for slot in 0..MAX_BONDS as u8 {
        match fetch_item::<u8, StoredBondInfo, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &slot,
        )
        .await
        {
            Ok(Some(existing)) => {
                // Check if this is the same device
                if existing.addr == *bond.identity.bd_addr.raw() {
                    target_slot = Some(slot);
                    break;
                }
            }
            Ok(None) => {
                // Found empty slot
                if target_slot.is_none() {
                    target_slot = Some(slot);
                }
            }
            Err(_) => {
                // Error reading, try this slot
                if target_slot.is_none() {
                    target_slot = Some(slot);
                }
            }
        }
    }

    let slot = target_slot.ok_or(())?;

    // Convert to storable format
    let mut addr = [0u8; 6];
    addr.copy_from_slice(bond.identity.bd_addr.raw());

    let stored = StoredBondInfo {
        addr,
        ltk: bond.ltk.to_le_bytes(),
        security_level: match bond.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        },
        profile_id,
        auto_connect: false,
        ediv: bond.ediv,
        rand: bond.rand,
        encryption_key_len: bond.encryption_key_len,
        name: *device_name,
        name_len: device_name_len,
    };

    match store_item(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &slot,
        &stored,
    )
    .await
    {
        Ok(_) => {
            info!("Stored bond in slot {}: {:?}", slot, stored.addr);
            Ok(slot)
        }
        Err(e) => {
            error!("Failed to store bond: {:?}", defmt::Debug2Format(&e));
            Err(())
        }
    }
}

/// Clear all stored bonds from flash
pub async fn clear_all_bonds(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Result<(), ()> {
    info!("Clearing all bonds from flash...");
    let mut buffer = [0u8; 128];

    match remove_all_items::<u8, _>(flash, flash_range(), &mut NoCache::new(), &mut buffer).await {
        Ok(_) => {
            info!("All bonds cleared!");
            Ok(())
        }
        Err(e) => {
            error!("Failed to clear bonds: {:?}", defmt::Debug2Format(&e));
            Err(())
        }
    }
}

/// Update the profile ID for an existing bond by address
/// Returns Ok(slot) if bond was found and updated, Err(()) if not found
pub async fn update_bond_profile(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
    new_profile_id: u8,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    // Find bond with matching address
    for slot in 0..MAX_BONDS as u8 {
        match fetch_item::<u8, StoredBondInfo, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &slot,
        )
        .await
        {
            Ok(Some(mut stored)) => {
                if stored.addr == *address {
                    // Found the bond, update profile
                    stored.profile_id = new_profile_id;

                    match store_item(
                        flash,
                        flash_range(),
                        &mut NoCache::new(),
                        &mut buffer,
                        &slot,
                        &stored,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!(
                                "Updated bond profile in slot {} to {}",
                                slot, new_profile_id
                            );
                            return Ok(slot);
                        }
                        Err(e) => {
                            error!(
                                "Failed to update bond profile: {:?}",
                                defmt::Debug2Format(&e)
                            );
                            return Err(());
                        }
                    }
                }
            }
            _ => continue,
        }
    }

    warn!("Bond not found for address {:?}", address);
    Err(())
}

/// Update the auto_connect flag for an existing bond by address.
/// Returns Ok(slot) if found and updated, Err(()) if not found.
pub async fn update_auto_connect(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
    enabled: bool,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    for slot in 0..MAX_BONDS as u8 {
        match fetch_item::<u8, StoredBondInfo, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &slot,
        )
        .await
        {
            Ok(Some(mut stored)) => {
                if stored.addr == *address {
                    stored.auto_connect = enabled;
                    match store_item(
                        flash,
                        flash_range(),
                        &mut NoCache::new(),
                        &mut buffer,
                        &slot,
                        &stored,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!("Updated auto_connect for slot {} to {}", slot, enabled);
                            return Ok(slot);
                        }
                        Err(e) => {
                            error!(
                                "Failed to update auto_connect: {:?}",
                                defmt::Debug2Format(&e)
                            );
                            return Err(());
                        }
                    }
                }
            }
            _ => continue,
        }
    }

    warn!("Bond not found for address {:?}", address);
    Err(())
}

/// Remove a single bond by address.
/// Returns Ok(slot) if found and removed, Err(()) if not found.
pub async fn clear_bond_by_address(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    for slot in 0..MAX_BONDS as u8 {
        match fetch_item::<u8, StoredBondInfo, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &slot,
        )
        .await
        {
            Ok(Some(stored)) => {
                if stored.addr == *address {
                    match remove_item::<u8, _>(
                        flash,
                        flash_range(),
                        &mut NoCache::new(),
                        &mut buffer,
                        &slot,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!("Removed bond in slot {} for {:?}", slot, address);
                            return Ok(slot);
                        }
                        Err(e) => {
                            error!("Failed to remove bond: {:?}", defmt::Debug2Format(&e));
                            return Err(());
                        }
                    }
                }
            }
            _ => continue,
        }
    }

    warn!("Bond not found for address {:?}", address);
    Err(())
}

// ============ Classic BT Bond Storage ============

/// Key offset for Classic BT bonds (128-137), separate from BLE bonds (0-9).
const CLASSIC_BOND_KEY_BASE: u8 = 128;

/// Maximum number of Classic BT bonded devices.
pub const MAX_CLASSIC_BONDS: usize = 10;

/// Stored Classic BT bond: link key + device metadata.
#[derive(Clone)]
pub struct StoredClassicBond {
    pub addr: [u8; 6],
    pub link_key: [u8; 16],
    pub key_type: u8,
    pub profile_id: u8,
    pub name: [u8; 32],
    pub name_len: u8,
}

impl<'a> Value<'a> for StoredClassicBond {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // 6 + 16 + 1 + 1 + 32 + 1 = 57 bytes
        if buffer.len() < 57 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(&self.addr);
        buffer[6..22].copy_from_slice(&self.link_key);
        buffer[22] = self.key_type;
        buffer[23] = self.profile_id;
        buffer[24..56].copy_from_slice(&self.name);
        buffer[56] = self.name_len;
        Ok(57)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 24 {
            return Err(SerializationError::BufferTooSmall);
        }
        let mut addr = [0u8; 6];
        let mut link_key = [0u8; 16];
        addr.copy_from_slice(&buffer[0..6]);
        link_key.copy_from_slice(&buffer[6..22]);
        let key_type = buffer[22];
        let profile_id = buffer[23];
        let mut name = [0u8; 32];
        let name_len = if buffer.len() >= 57 {
            name.copy_from_slice(&buffer[24..56]);
            buffer[56]
        } else {
            0
        };
        Ok(StoredClassicBond {
            addr,
            link_key,
            key_type,
            profile_id,
            name,
            name_len,
        })
    }
}

/// Load all stored Classic BT bonds from flash.
pub async fn load_classic_bonds(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Vec<StoredClassicBond, MAX_CLASSIC_BONDS> {
    let mut bonds = Vec::new();
    let mut buffer = [0u8; 128];

    for i in 0..MAX_CLASSIC_BONDS as u8 {
        let key = CLASSIC_BOND_KEY_BASE + i;
        match fetch_item::<u8, StoredClassicBond, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &key,
        )
        .await
        {
            Ok(Some(stored)) => {
                debug!(
                    "Loaded Classic bond from slot {} (profile {}): {:02x}",
                    i, stored.profile_id, stored.addr
                );
                let _ = bonds.push(stored);
            }
            Ok(None) => {}
            Err(e) => {
                warn!(
                    "Error reading Classic bond slot {}: {:?}",
                    i,
                    defmt::Debug2Format(&e)
                );
            }
        }
    }

    info!("Loaded {} Classic bond(s) from flash", bonds.len());
    bonds
}

/// Store a Classic BT bond to flash after successful pairing.
/// Returns the slot number on success.
pub async fn store_classic_bond(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    addr: &[u8; 6],
    link_key: &[u8; 16],
    key_type: u8,
    profile_id: u8,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];
    let mut target_slot: Option<u8> = None;

    for i in 0..MAX_CLASSIC_BONDS as u8 {
        let key = CLASSIC_BOND_KEY_BASE + i;
        match fetch_item::<u8, StoredClassicBond, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &key,
        )
        .await
        {
            Ok(Some(existing)) => {
                if existing.addr == *addr {
                    target_slot = Some(i);
                    break;
                }
            }
            Ok(None) => {
                if target_slot.is_none() {
                    target_slot = Some(i);
                }
            }
            Err(_) => {
                if target_slot.is_none() {
                    target_slot = Some(i);
                }
            }
        }
    }

    let slot = target_slot.ok_or(())?;
    let key = CLASSIC_BOND_KEY_BASE + slot;

    let stored = StoredClassicBond {
        addr: *addr,
        link_key: *link_key,
        key_type,
        profile_id,
        name: [0u8; 32],
        name_len: 0,
    };

    match store_item(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &key,
        &stored,
    )
    .await
    {
        Ok(_) => {
            info!("Stored Classic bond in slot {}: {:02x}", slot, addr);
            Ok(slot)
        }
        Err(e) => {
            error!(
                "Failed to store Classic bond: {:?}",
                defmt::Debug2Format(&e)
            );
            Err(())
        }
    }
}

/// Clear all Classic bonds from flash.
pub async fn clear_all_classic_bonds(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Result<(), ()> {
    info!("Clearing all Classic bonds from flash...");
    let mut buffer = [0u8; 128];
    let mut cleared = 0u8;

    for i in 0..MAX_CLASSIC_BONDS as u8 {
        let key = CLASSIC_BOND_KEY_BASE + i;
        if remove_item::<u8, _>(flash, flash_range(), &mut NoCache::new(), &mut buffer, &key)
            .await
            .is_ok()
        {
            cleared += 1;
        }
    }

    info!("Cleared {} Classic bond(s)", cleared);
    Ok(())
}

/// Update the profile ID for an existing Classic bond by address.
#[allow(dead_code)]
pub async fn update_classic_bond_profile(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
    new_profile_id: u8,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    for i in 0..MAX_CLASSIC_BONDS as u8 {
        let key = CLASSIC_BOND_KEY_BASE + i;
        match fetch_item::<u8, StoredClassicBond, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &key,
        )
        .await
        {
            Ok(Some(mut stored)) => {
                if stored.addr == *address {
                    stored.profile_id = new_profile_id;
                    match store_item(
                        flash,
                        flash_range(),
                        &mut NoCache::new(),
                        &mut buffer,
                        &key,
                        &stored,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!(
                                "Updated Classic bond profile in slot {} to {}",
                                i, new_profile_id
                            );
                            return Ok(i);
                        }
                        Err(e) => {
                            error!(
                                "Failed to update Classic bond profile: {:?}",
                                defmt::Debug2Format(&e)
                            );
                            return Err(());
                        }
                    }
                }
            }
            _ => continue,
        }
    }

    warn!("Classic bond not found for address {:02x}", address);
    Err(())
}

/// Remove a single Classic bond by address.
#[allow(dead_code)]
pub async fn clear_classic_bond_by_address(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
) -> Result<u8, ()> {
    let mut buffer = [0u8; 128];

    for i in 0..MAX_CLASSIC_BONDS as u8 {
        let key = CLASSIC_BOND_KEY_BASE + i;
        match fetch_item::<u8, StoredClassicBond, _>(
            flash,
            flash_range(),
            &mut NoCache::new(),
            &mut buffer,
            &key,
        )
        .await
        {
            Ok(Some(stored)) => {
                if stored.addr == *address {
                    match remove_item::<u8, _>(
                        flash,
                        flash_range(),
                        &mut NoCache::new(),
                        &mut buffer,
                        &key,
                    )
                    .await
                    {
                        Ok(_) => {
                            info!("Removed Classic bond in slot {} for {:02x}", i, address);
                            return Ok(i);
                        }
                        Err(e) => {
                            error!(
                                "Failed to remove Classic bond: {:?}",
                                defmt::Debug2Format(&e)
                            );
                            return Err(());
                        }
                    }
                }
            }
            _ => continue,
        }
    }

    warn!("Classic bond not found for address {:02x}", address);
    Err(())
}
