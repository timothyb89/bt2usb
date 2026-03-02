//! Shared BLE command handlers
//!
//! These handlers are used by both the idle command loop (in `mod.rs`) and
//! the connected HID loop (in `gatt.rs`) to avoid duplicating logic for
//! commands like GetStatus, GetBonds, SetActiveDevice, etc.

use core::sync::atomic::Ordering;

use defmt::*;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_time::Timer;

use crate::ble_hid;
use crate::ble_state;
use crate::bonding::{self, LoadedBond};
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::rpc_log;
use crate::FLASH_SIZE;

/// Build and send a StatusInfo response over the STATUS_RESPONSE_CHANNEL.
pub fn handle_get_status(
    loaded_bonds: &[LoadedBond],
    active_profile: DeviceProfile,
    active_device_pref: &Option<preferences::ActiveDevice>,
) {
    let (has_active, active_addr) = if let Some(ref pref) = active_device_pref {
        (pref.address != [0u8; 6], pref.address)
    } else {
        (false, [0u8; 6])
    };

    let status = ble_state::StatusInfo {
        bonded_count: loaded_bonds.len() as u8,
        active_profile: active_profile.to_id(),
        active_device_set: has_active,
        active_device_address: active_addr,
        battery_level: ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed),
    };

    let _ = ble_state::STATUS_RESPONSE_CHANNEL.try_send(status);
}

/// Build and send a BondList response over the BONDS_RESPONSE_CHANNEL.
pub fn handle_get_bonds(loaded_bonds: &[LoadedBond]) {
    let mut bond_list: ble_state::BondList = heapless::Vec::new();
    for lb in loaded_bonds {
        let addr_bytes = lb.bond.identity.bd_addr.raw();
        let mut addr = [0u8; 6];
        addr.copy_from_slice(addr_bytes);

        let addr_kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
            1u8
        } else {
            0u8
        };

        let profile = DeviceProfile::from_id(lb.profile_id);
        let name_str = profile.name();
        let mut name: heapless::String<32> = heapless::String::new();
        let _ = name.push_str(if name_str.is_empty() {
            "Unknown Device"
        } else {
            name_str
        });

        let _ = bond_list.push((addr, addr_kind, lb.profile_id, name));
    }

    let _ = ble_state::BONDS_RESPONSE_CHANNEL.try_send(bond_list);
}

/// Save the active device preference to flash.
pub async fn handle_set_active_device(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    address: [u8; 6],
    addr_kind: u8,
) {
    info!("Setting active device: {:?}", address);
    let device = preferences::ActiveDevice { address, addr_kind };
    match preferences::set_active_device(flash, &device).await {
        Ok(()) => {
            info!("Active device preference saved");
            rpc_log::info("Active device set");
        }
        Err(()) => {
            error!("Failed to save active device preference");
            rpc_log::error("Failed to set active device");
        }
    }
}

/// Clear the active device preference from flash.
pub async fn handle_clear_active_device(flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>) {
    info!("Clearing active device preference");
    match preferences::clear_active_device(flash).await {
        Ok(()) => {
            info!("Active device preference cleared");
            rpc_log::info("Active device cleared");
        }
        Err(()) => {
            error!("Failed to clear active device preference");
            rpc_log::error("Failed to clear active device");
        }
    }
}

/// Update the device profile for an existing bond, then reload bonds.
///
/// Returns the updated bond list on success, or None if the update failed.
pub async fn handle_update_bond_profile(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    address: &[u8; 6],
    profile_id: u8,
) -> Option<heapless::Vec<LoadedBond, { bonding::MAX_BONDS }>> {
    info!("Updating bond profile for {:?} to {}", address, profile_id);
    match bonding::update_bond_profile(flash, address, profile_id).await {
        Ok(slot) => {
            info!("Bond profile updated in slot {}", slot);
            rpc_log::info("Bond profile updated");
            // Reload bonds to update in-memory list
            Some(bonding::load_bonds(flash).await)
        }
        Err(()) => {
            error!("Failed to update bond profile");
            rpc_log::error("Bond not found");
            None
        }
    }
}

/// Clear all bonds from flash, log the action, and trigger a system reset.
///
/// This function does not return on success (system resets).
pub async fn handle_clear_bonds(flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>) {
    info!("Clearing all bonds");
    rpc_log::info("Clearing all bonds...");
    match bonding::clear_all_bonds(flash).await {
        Ok(()) => {
            info!("All bonds cleared successfully");
            rpc_log::info("All bonds cleared - restarting...");
            // Give time for log message to be sent
            Timer::after_millis(100).await;
            crate::system_reset();
        }
        Err(()) => {
            error!("Failed to clear bonds");
            rpc_log::error("Failed to clear bonds");
        }
    }
}

/// Persist a config value (axis multiplier) to flash.
pub async fn handle_set_config(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    key: u8,
    value: u32,
) {
    let pref_key = match key {
        crate::usb_hid::CONFIG_KEY_SCROLL_MULT => preferences::PREF_KEY_SCROLL_MULTIPLIER,
        crate::usb_hid::CONFIG_KEY_PAN_MULT => preferences::PREF_KEY_PAN_MULTIPLIER,
        crate::usb_hid::CONFIG_KEY_X_MULT => preferences::PREF_KEY_X_MULTIPLIER,
        crate::usb_hid::CONFIG_KEY_Y_MULT => preferences::PREF_KEY_Y_MULTIPLIER,
        _ => {
            warn!("Unknown config key: {}", key);
            return;
        }
    };
    let _ = preferences::store_multiplier(flash, pref_key, value).await;
}

/// Log a restart message and trigger a system reset.
///
/// This function does not return.
pub async fn handle_restart() -> ! {
    info!("Manual restart requested");
    rpc_log::info("Restarting device...");
    // Give time for log message and RPC response to be sent
    Timer::after_millis(100).await;
    crate::system_reset();
}
