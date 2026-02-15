//! BLE Bonding Storage
//!
//! Stores bonding information to flash memory so devices stay paired
//! across power cycles. Uses sequential-storage for wear-leveled writes.

use core::ops::Range;
use defmt::{debug, error, info, warn};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use heapless::Vec;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{
    fetch_item, remove_all_items, store_item, SerializationError, Value,
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

/// Key type - a simple slot index (0-9) for storing up to MAX_BONDS devices
/// We use simple u8 keys instead of device addresses for simplicity
pub type BondSlot = u8;

/// Value for storing bond info: all the data needed to restore a bond
#[derive(Clone)]
pub struct StoredBondInfo {
    pub addr: [u8; 6],      // BdAddr raw bytes
    pub ltk: [u8; 16],      // LongTermKey raw bytes
    pub security_level: u8, // 0=None, 1=Encrypted, 2=EncryptedAuthenticated
    pub profile_id: u8, // DeviceProfile discriminant (0=Generic, 1=MxMaster3S, 2=FullScrollDial)
}

impl<'a> Value<'a> for StoredBondInfo {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // 6 + 16 + 1 + 1 = 24 bytes
        if buffer.len() < 24 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(&self.addr);
        buffer[6..22].copy_from_slice(&self.ltk);
        buffer[22] = self.security_level;
        buffer[23] = self.profile_id;
        Ok(24)
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
        Ok(StoredBondInfo {
            addr,
            ltk,
            security_level,
            profile_id,
        })
    }
}

/// A loaded bond paired with its device profile ID
pub struct LoadedBond {
    pub bond: BondInformation,
    pub profile_id: u8,
}

/// Load all stored bonds from flash
pub async fn load_bonds(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Vec<LoadedBond, MAX_BONDS> {
    let mut bonds = Vec::new();
    let mut buffer = [0u8; 64];

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
                };

                debug!(
                    "Loaded bond from slot {} (profile {}): {:?}",
                    slot, stored.profile_id, stored.addr
                );
                let _ = bonds.push(LoadedBond {
                    bond,
                    profile_id: stored.profile_id,
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

/// Store a bond to flash after successful pairing
/// Returns the slot it was stored in, or error if no slots available
pub async fn store_bond(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    bond: &BondInformation,
    profile_id: u8,
) -> Result<u8, ()> {
    let mut buffer = [0u8; 64];

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
    let mut buffer = [0u8; 64];

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
    let mut buffer = [0u8; 64];

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

/// Find a bond by address and return its slot and profile
/// Returns Ok((slot, profile_id)) if found, Err(()) if not found
pub async fn find_bond(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    address: &[u8; 6],
) -> Result<(u8, u8), ()> {
    let mut buffer = [0u8; 64];

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
                    return Ok((slot, stored.profile_id));
                }
            }
            _ => continue,
        }
    }

    Err(())
}
