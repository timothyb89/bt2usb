//! User Preferences Storage
//!
//! Stores user preferences like active device selection to flash memory.
//! Uses sequential-storage for wear-leveled writes.

use core::ops::Range;
use defmt::{debug, error, info, warn};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_item, store_item, SerializationError, Value};

/// Flash storage range for preferences
/// Uses 16KB (4 x 4KB erase sectors) starting at 0x1F4000
/// This is right after the bond storage (0x1F0000-0x1F3FFF)
const PREFERENCES_FLASH_OFFSET: u32 = 2 * 1024 * 1024 - 48 * 1024; // 0x1F4000
const PREFERENCES_FLASH_SIZE: u32 = 16 * 1024; // 16KB

fn flash_range() -> Range<u32> {
    PREFERENCES_FLASH_OFFSET..(PREFERENCES_FLASH_OFFSET + PREFERENCES_FLASH_SIZE)
}

/// Preference keys - each key maps to a different preference
const PREF_KEY_ACTIVE_DEVICE: u8 = 0;
pub const PREF_KEY_SCROLL_MULTIPLIER: u8 = 2;
pub const PREF_KEY_PAN_MULTIPLIER: u8 = 3;
pub const PREF_KEY_X_MULTIPLIER: u8 = 4;
pub const PREF_KEY_Y_MULTIPLIER: u8 = 5;

/// Active device preference - which device to auto-connect to
#[derive(Clone, Debug)]
pub struct ActiveDevice {
    pub address: [u8; 6],
    pub addr_kind: u8, // 0 = PUBLIC, 1 = RANDOM
}

impl<'a> Value<'a> for ActiveDevice {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // 6 bytes (address) + 1 byte (kind) = 7 bytes
        if buffer.len() < 7 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(&self.address);
        buffer[6] = self.addr_kind;
        Ok(7)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 7 {
            return Err(SerializationError::BufferTooSmall);
        }
        let mut address = [0u8; 6];
        address.copy_from_slice(&buffer[0..6]);
        let addr_kind = buffer[6];
        Ok(ActiveDevice { address, addr_kind })
    }
}

/// Load the active device preference from flash
/// Returns None if no active device is set
pub async fn load_active_device(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Option<ActiveDevice> {
    let mut buffer = [0u8; 64];

    match fetch_item::<u8, ActiveDevice, _>(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &PREF_KEY_ACTIVE_DEVICE,
    )
    .await
    {
        Ok(Some(device)) => {
            debug!("Loaded active device: {:?}", device.address);
            Some(device)
        }
        Ok(None) => {
            debug!("No active device preference set");
            None
        }
        Err(e) => {
            warn!(
                "Error loading active device preference: {:?}",
                defmt::Debug2Format(&e)
            );
            None
        }
    }
}

/// Set the active device preference
pub async fn set_active_device(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    device: &ActiveDevice,
) -> Result<(), ()> {
    let mut buffer = [0u8; 64];

    match store_item(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &PREF_KEY_ACTIVE_DEVICE,
        device,
    )
    .await
    {
        Ok(_) => {
            info!("Set active device: {:?}", device.address);
            Ok(())
        }
        Err(e) => {
            error!("Failed to set active device: {:?}", defmt::Debug2Format(&e));
            Err(())
        }
    }
}

/// Clear the active device preference
pub async fn clear_active_device(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
) -> Result<(), ()> {
    // Store a dummy value with all zeros to effectively clear
    let dummy = ActiveDevice {
        address: [0u8; 6],
        addr_kind: 0,
    };
    set_active_device(flash, &dummy).await
}

/// Axis multiplier value (percentage, e.g. 100 = 1.0x)
#[derive(Clone, Debug)]
pub struct MultiplierValue {
    pub percent: u32,
}

impl<'a> Value<'a> for MultiplierValue {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..4].copy_from_slice(&self.percent.to_le_bytes());
        Ok(4)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        let percent = u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        Ok(MultiplierValue { percent })
    }
}

/// Load a multiplier preference from flash. Returns 100 (1.0x) if not set.
pub async fn load_multiplier(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    key: u8,
) -> u32 {
    let mut buffer = [0u8; 64];
    match fetch_item::<u8, MultiplierValue, _>(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &key,
    )
    .await
    {
        Ok(Some(val)) => {
            debug!("Loaded multiplier key={}: {}%", key, val.percent);
            val.percent
        }
        Ok(None) => {
            debug!("No multiplier for key={}, defaulting to 100%", key);
            100
        }
        Err(e) => {
            warn!(
                "Error loading multiplier key={}: {:?}",
                key,
                defmt::Debug2Format(&e)
            );
            100
        }
    }
}

/// Store a multiplier preference to flash.
pub async fn store_multiplier(
    flash: &mut Flash<'_, FLASH, Async, { 2 * 1024 * 1024 }>,
    key: u8,
    percent: u32,
) -> Result<(), ()> {
    let mut buffer = [0u8; 64];
    let val = MultiplierValue { percent };
    match store_item(
        flash,
        flash_range(),
        &mut NoCache::new(),
        &mut buffer,
        &key,
        &val,
    )
    .await
    {
        Ok(_) => {
            info!("Stored multiplier key={}: {}%", key, percent);
            Ok(())
        }
        Err(e) => {
            error!(
                "Failed to store multiplier key={}: {:?}",
                key,
                defmt::Debug2Format(&e)
            );
            Err(())
        }
    }
}
