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
    use crate::ble::slots;

    let (has_active, active_addr) = if let Some(ref pref) = active_device_pref {
        (pref.address != [0u8; 6], pref.address)
    } else {
        (false, [0u8; 6])
    };

    // Gather per-slot connection info (BLE slots 0-2 + Classic slot 3)
    let mut connected_devices: [Option<ble_state::ConnectedDeviceInfo>; 4] =
        [None, None, None, None];
    let mut connected_count = 0u8;
    for (i, dev) in connected_devices
        .iter_mut()
        .enumerate()
        .take(slots::MAX_CONNECTIONS)
    {
        if !slots::is_slot_idle(i) {
            *dev = Some(ble_state::ConnectedDeviceInfo {
                address: slots::get_slot_address(i),
                profile_id: slots::SLOT_PROFILES[i].load(Ordering::Relaxed),
                battery_level: ble_hid::BATTERY_LEVELS[i].load(Ordering::Relaxed),
                transport_type: ble_state::TransportType::Ble,
            });
            connected_count += 1;
        }
    }
    // Classic BT slot
    if slots::is_classic_connected() {
        connected_devices[3] = Some(ble_state::ConnectedDeviceInfo {
            address: slots::get_classic_address(),
            profile_id: slots::CLASSIC_PROFILE.load(Ordering::Relaxed),
            battery_level: 0xFF, // Classic doesn't report battery via HID
            transport_type: ble_state::TransportType::Classic,
        });
        connected_count += 1;
    }

    let status = ble_state::StatusInfo {
        bonded_count: loaded_bonds.len() as u8,
        active_profile: active_profile.to_id(),
        active_device_set: has_active,
        active_device_address: active_addr,
        battery_level: ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed),
        connected_count,
        connected_devices,
    };

    let _ = ble_state::STATUS_RESPONSE_CHANNEL.try_send(status);
}

/// Build and send a BondList response over the BONDS_RESPONSE_CHANNEL.
/// Includes both BLE and Classic BT bonds.
pub fn handle_get_bonds(
    loaded_bonds: &[LoadedBond],
    loaded_classic_bonds: &[bonding::StoredClassicBond],
) {
    let mut bond_list: ble_state::BondList = heapless::Vec::new();

    // BLE bonds
    for lb in loaded_bonds {
        let addr_bytes = lb.bond.identity.bd_addr.raw();
        let mut addr = [0u8; 6];
        addr.copy_from_slice(addr_bytes);

        let addr_kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
            1u8
        } else {
            0u8
        };

        let mut name: heapless::String<32> = heapless::String::new();
        if lb.name_len > 0 {
            if let Ok(s) = core::str::from_utf8(&lb.name[..lb.name_len as usize]) {
                let _ = name.push_str(s);
            }
        }
        if name.is_empty() {
            let profile = DeviceProfile::from_id(lb.profile_id);
            let profile_name = profile.name();
            let _ = name.push_str(if profile_name.is_empty() {
                "Unknown Device"
            } else {
                profile_name
            });
        }

        let _ = bond_list.push((
            addr,
            addr_kind,
            lb.profile_id,
            name,
            lb.auto_connect,
            ble_state::TransportType::Ble,
        ));
    }

    // Classic BT bonds
    for cb in loaded_classic_bonds {
        let mut name: heapless::String<32> = heapless::String::new();
        if cb.name_len > 0 {
            if let Ok(s) = core::str::from_utf8(&cb.name[..cb.name_len as usize]) {
                let _ = name.push_str(s);
            }
        }
        if name.is_empty() {
            let profile = DeviceProfile::from_id(cb.profile_id);
            let profile_name = profile.name();
            let _ = name.push_str(if profile_name.is_empty() {
                "Unknown Device"
            } else {
                profile_name
            });
        }

        let _ = bond_list.push((
            cb.addr,
            0,
            cb.profile_id,
            name,
            cb.auto_connect,
            ble_state::TransportType::Classic,
        ));
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
    transport_type: ble_state::TransportType,
) -> Option<heapless::Vec<LoadedBond, { bonding::MAX_BONDS }>> {
    info!(
        "Updating bond profile for {:?} (transport={}) to {}",
        address, transport_type, profile_id
    );
    let result = if transport_type == ble_state::TransportType::Classic {
        bonding::update_classic_bond_profile(flash, address, profile_id).await
    } else {
        bonding::update_bond_profile(flash, address, profile_id).await
    };
    match result {
        Ok(slot) => {
            info!("Bond profile updated in slot {}", slot);
            rpc_log::info("Bond profile updated");
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
    info!("Clearing all bonds (BLE + Classic)");
    rpc_log::info("Clearing all bonds...");
    let ble_ok = bonding::clear_all_bonds(flash).await.is_ok();
    let classic_ok = bonding::clear_all_classic_bonds(flash).await.is_ok();
    if ble_ok && classic_ok {
        info!("All bonds cleared successfully");
        rpc_log::info("All bonds cleared - restarting...");
        Timer::after_millis(100).await;
        crate::system_reset();
    } else {
        error!(
            "Failed to clear bonds (ble={}, classic={})",
            ble_ok, classic_ok
        );
        rpc_log::error("Failed to clear bonds");
    }
}

/// Factory reset: clear all bonds and preferences, then restart.
///
/// This function does not return on success (system resets).
pub async fn handle_factory_reset(flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>) {
    info!("Factory reset: clearing all bonds and preferences");
    rpc_log::info("Factory reset...");

    let bonds_ok = bonding::clear_all_bonds(flash).await.is_ok();
    let prefs_ok = crate::preferences::clear_all_preferences(flash)
        .await
        .is_ok();

    if bonds_ok && prefs_ok {
        info!("Factory reset complete");
        rpc_log::info("Factory reset complete - restarting...");
        Timer::after_millis(100).await;
        crate::system_reset();
    } else {
        error!(
            "Factory reset partially failed (bonds={}, prefs={})",
            bonds_ok, prefs_ok
        );
        rpc_log::error("Factory reset failed");
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
        crate::usb_hid::CONFIG_KEY_SCROLL_THRESHOLD => preferences::PREF_KEY_SCROLL_THRESHOLD,
        crate::usb_hid::CONFIG_KEY_MAX_DETENTS => preferences::PREF_KEY_MAX_DETENTS,
        crate::usb_hid::CONFIG_KEY_SCROLL_SMOOTHING => preferences::PREF_KEY_SCROLL_SMOOTHING,
        _ => {
            warn!("Unknown config key: {}", key);
            return;
        }
    };
    let _ = preferences::store_multiplier(flash, pref_key, value).await;
}

/// Persist a forced OS override preference to flash.
///
/// 0 = Auto (probe normally), 1 = Windows, 2 = Linux, 3 = macOS.
/// The scratch register cache is updated by the RPC handler on Core 1.
pub async fn handle_set_forced_os(flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>, os: u8) {
    let _ = preferences::store_multiplier(flash, preferences::PREF_KEY_FORCED_OS, os as u32).await;
    info!("Forced OS preference stored: {}", os);
}

/// Update the auto-connect flag for a bonded device and refresh the in-memory bond list.
pub async fn handle_set_auto_connect(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    address: &[u8; 6],
    enabled: bool,
    transport_type: ble_state::TransportType,
    loaded_bonds: &mut heapless::Vec<LoadedBond, { bonding::MAX_BONDS }>,
) {
    info!(
        "Setting auto_connect for {:?} (transport={}) to {}",
        address, transport_type, enabled
    );
    let result = if transport_type == ble_state::TransportType::Classic {
        bonding::update_classic_auto_connect(flash, address, enabled).await
    } else {
        bonding::update_auto_connect(flash, address, enabled).await
    };
    match result {
        Ok(slot) => {
            info!("Auto-connect updated in slot {}", slot);
            rpc_log::info(if enabled {
                "Auto-connect enabled"
            } else {
                "Auto-connect disabled"
            });
            *loaded_bonds = bonding::load_bonds(flash).await;
        }
        Err(()) => {
            error!("Failed to update auto-connect: bond not found");
            rpc_log::error("Bond not found");
        }
    }
}

/// Remove a single bond by address and refresh the in-memory bond list.
pub async fn handle_clear_bond(
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    address: &[u8; 6],
    transport_type: ble_state::TransportType,
    loaded_bonds: &mut heapless::Vec<LoadedBond, { bonding::MAX_BONDS }>,
) {
    info!(
        "Clearing bond for {:?} (transport={})",
        address, transport_type
    );
    let result = if transport_type == ble_state::TransportType::Classic {
        bonding::clear_classic_bond_by_address(flash, address).await
    } else {
        bonding::clear_bond_by_address(flash, address).await
    };
    match result {
        Ok(slot) => {
            info!("Bond removed from slot {}", slot);
            rpc_log::info("Bond removed");
            *loaded_bonds = bonding::load_bonds(flash).await;
        }
        Err(()) => {
            error!("Failed to clear bond: not found");
            rpc_log::error("Bond not found");
        }
    }
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
