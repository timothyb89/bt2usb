//! Classic Bluetooth task — connects to Classic HID devices (Magic Trackpad 2).
//!
//! This module handles the full Classic BT connection lifecycle:
//! ACL → Auth → Encrypt → L2CAP → HIDP, then forwards received HID reports
//! to `HID_REPORT_CHANNEL` for USB translation.
//!
//! Runs concurrently with the BLE slot tasks and connection manager on Core 0.

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use bt_classic_host::hidp::ReportType;
use bt_classic_host::{ClassicRunner, HidClient, HostResources, L2capState};
use defmt::*;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;

use super::slots;
use super::FlashMutex;
use crate::ble_hid::{HidReportEvent, HidReportType, HID_REPORT_CHANNEL};
use crate::ble_state::{ClassicCommand, CLASSIC_CMD_CHANNEL};
use crate::bonding;
use crate::device_profile::DeviceProfile;

// ============ Link Key Storage ============

/// In-memory link key store for Classic BT, backed by flash persistence.
/// Populated from flash at startup; new keys persisted after connect().
struct MemoryLinkKeyStore {
    keys: heapless::Vec<(bt_hci::param::BdAddr, bt_classic_host::LinkKeyInfo), 10>,
}

impl MemoryLinkKeyStore {
    fn new() -> Self {
        Self {
            keys: heapless::Vec::new(),
        }
    }

    /// Populate from loaded flash bonds.
    fn load_from_flash(&mut self, bonds: &[bonding::StoredClassicBond]) {
        for bond in bonds {
            let addr = bt_hci::param::BdAddr::new(bond.addr);
            let key_info = bt_classic_host::LinkKeyInfo {
                key: bond.link_key,
                key_type: bond.key_type,
            };
            let _ = self.keys.push((addr, key_info));
        }
    }
}

impl bt_classic_host::LinkKeyStore for MemoryLinkKeyStore {
    fn load(&self, addr: &bt_hci::param::BdAddr) -> Option<bt_classic_host::LinkKeyInfo> {
        self.keys
            .iter()
            .find(|(a, _)| a.raw() == addr.raw())
            .map(|(_, k)| k.clone())
    }

    fn store(&mut self, addr: &bt_hci::param::BdAddr, key: bt_classic_host::LinkKeyInfo) {
        // Update existing or insert new
        for (a, k) in self.keys.iter_mut() {
            if a.raw() == addr.raw() {
                *k = key;
                return;
            }
        }
        let _ = self.keys.push((*addr, key));
    }

    fn remove(&mut self, addr: &bt_hci::param::BdAddr) {
        if let Some(pos) = self.keys.iter().position(|(a, _)| a.raw() == addr.raw()) {
            self.keys.swap_remove(pos);
        }
    }
}

// ============ Helpers ============

/// Check if a Bluetooth Class of Device (CoD) indicates a Peripheral (HID) device.
/// CoD is 3 bytes (24 bits). Major Device Class is bits 12:8 (upper 5 bits of cod[1]).
/// Major class 0x05 = Peripheral (keyboards, mice, trackpads, gamepads).
fn cod_is_peripheral(cod: &[u8]) -> bool {
    if cod.len() < 3 {
        return false;
    }
    // CoD byte layout (little-endian from HCI):
    //   cod[0] = bits 7:0 (format type + minor service)
    //   cod[1] = bits 15:8 (major service class bits + major device class)
    //   cod[2] = bits 23:16 (major service class continued)
    // Major Device Class = bits 12:8 = (cod[1] >> 0) & 0x1F
    let major_device_class = cod[1] & 0x1F;
    major_device_class == 0x05
}

// ============ Classic BT Task ============

/// Classic Bluetooth task — connects to Classic HID devices (Magic Trackpad 2).
///
/// Runs the full connection lifecycle: ACL → Auth → Encrypt → L2CAP → HIDP.
/// Forwards received HID reports to HID_REPORT_CHANNEL for USB translation.
pub async fn classic_bt_task<C>(
    controller: &C,
    flash_mutex: &FlashMutex,
    loaded_classic_bonds: &[bonding::StoredClassicBond],
) where
    C: bt_hci::controller::Controller
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::controller_baseband::SetEventMask>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::CreateConnection>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::AuthenticationRequested>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::SetConnectionEncryption>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::LinkKeyRequestReply>
        + bt_hci::controller::ControllerCmdSync<
            bt_hci::cmd::link_control::LinkKeyRequestNegativeReply,
        > + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::PinCodeRequestReply>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::IoCapabilityRequestReply>
        + bt_hci::controller::ControllerCmdSync<
            bt_hci::cmd::link_control::UserConfirmationRequestReply,
        > + bt_hci::controller::ControllerCmdSync<bt_classic_host::link_policy::WriteLinkPolicySettings>
        + bt_hci::controller::ControllerCmdSync<bt_classic_host::link_policy::SniffMode>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::Inquiry>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::InquiryCancel>,
{
    info!("[classic] Classic BT task started");

    // Wait for BLE stack to initialize first (it sends SetEventMask during init)
    Timer::after_millis(2000).await;

    // --- Fix event mask: trouble-host's SetEventMask disables Classic auth events
    // (LinkKeyRequest, PinCodeRequest, AuthenticationComplete). Re-send with
    // Classic events enabled so the controller generates them during pairing.
    {
        use bt_hci::cmd::controller_baseband::SetEventMask;
        use bt_hci::param::EventMask;

        let mask = EventMask::new()
            // trouble-host's events
            .enable_le_meta(true)
            .enable_conn_request(true)
            .enable_conn_complete(true)
            .enable_hardware_error(true)
            .enable_disconnection_complete(true)
            .enable_encryption_change_v1(true)
            .enable_encryption_key_refresh_complete(true)
            // Classic auth events
            .enable_authentication_complete(true)
            .enable_link_key_request(true)
            .enable_link_key_notification(true)
            .enable_pin_code_request(true)
            .enable_io_capability_request(true)
            .enable_io_capability_response(true)
            .enable_user_confirmation_request(true)
            .enable_simple_pairing_complete(true)
            .enable_mode_change(true)
            .enable_inquiry_complete(true)
            .enable_inquiry_result(true);

        match controller.exec(&SetEventMask::new(mask)).await {
            Ok(_) => info!("[classic] Event mask updated with Classic events"),
            Err(e) => warn!(
                "[classic] Failed to set event mask: {:?}",
                defmt::Debug2Format(&e)
            ),
        }
    }

    // --- Create resources with flash-backed link key store ---
    let mut link_key_store = MemoryLinkKeyStore::new();
    link_key_store.load_from_flash(loaded_classic_bonds);
    info!(
        "[classic] Loaded {} link key(s) from flash",
        link_key_store.keys.len()
    );
    let link_keys = RefCell::new(link_key_store);
    let mut resources = HostResources::<1>::new();

    // --- Main Classic connection loop (reconnects on disconnect) ---
    'connect: loop {
        resources.reset();
        let mut runner = ClassicRunner::new(controller, &mut resources, &link_keys);

        // --- Resolve target address: auto-connect bond, or wait for command ---
        let target_addr = {
            // Check for auto-connect Classic bond
            let auto_bond = loaded_classic_bonds.iter().find(|b| b.auto_connect);

            if let Some(bond) = auto_bond {
                let addr = bt_hci::param::BdAddr::new(bond.addr);
                info!(
                    "[classic] Auto-connecting to bonded device: {:?} (profile: {:?})",
                    addr,
                    DeviceProfile::from_id(bond.profile_id)
                );
                addr
            } else {
                // No auto-connect bond — wait for a Connect command from RPC
                info!("[classic] No auto-connect Classic bond. Waiting for connect command...");
                loop {
                    match CLASSIC_CMD_CHANNEL.receive().await {
                        ClassicCommand::Connect { address } => {
                            let addr = bt_hci::param::BdAddr::new(address);
                            info!("[classic] Connect command received for {:?}", addr);
                            break addr;
                        }
                        ClassicCommand::Scan => {
                            info!("[classic] Starting Inquiry scan...");
                            crate::rpc_log::info("Classic Inquiry scan starting (~10s)...");

                            // CYW43439 only supports inquiry mode 0 (standard results,
                            // no RSSI/EIR). Modes 1 and 2 return zero results.

                            // GIAC LAP = 0x9E8B33, duration 23 (~29.4s), unlimited responses.
                            // Matches CLI's default 30s scan timeout.
                            match controller
                                .exec(&bt_hci::cmd::link_control::Inquiry::new(
                                    [0x33, 0x8B, 0x9E],
                                    23,
                                    0,
                                ))
                                .await
                            {
                                Ok(_) => {
                                    // Read Inquiry events until InquiryComplete or ScanStop.
                                    // Note: CYW43439 only supports inquiry mode 0 (no RSSI/EIR),
                                    // and RemoteNameRequest events are not delivered by the mux.
                                    // Names are unavailable during Classic scan.
                                    let mut rx = [0u8; 259];
                                    let rx_ref = &mut rx[..];
                                    let mut inquiry_done = false;
                                    while !inquiry_done {
                                        match select(
                                            controller.read(rx_ref),
                                            CLASSIC_CMD_CHANNEL.receive(),
                                        )
                                        .await
                                        {
                                            Either::Second(ClassicCommand::ScanStop) => {
                                                info!("[classic] Inquiry cancelled by user");
                                                let _ = controller
                                                .exec(
                                                    &bt_hci::cmd::link_control::InquiryCancel::new(),
                                                )
                                                .await;
                                                crate::rpc_log::info("Classic Inquiry cancelled");
                                                inquiry_done = true;
                                            }
                                            Either::Second(cmd) => {
                                                // Non-stop command during inquiry — will be
                                                // handled after inquiry completes. Re-send it.
                                                let _ = CLASSIC_CMD_CHANNEL.try_send(cmd);
                                            }
                                            Either::First(read_result) => {
                                                match read_result {
                                                    Ok(bt_hci::ControllerToHostPacket::Event(
                                                        evt,
                                                    )) => {
                                                        match evt.kind {
                                                bt_hci::event::EventKind::InquiryComplete => {
                                                    info!("[classic] Inquiry complete");
                                                    crate::rpc_log::info(
                                                        "Classic Inquiry scan complete",
                                                    );
                                                    inquiry_done = true;
                                                }
                                                bt_hci::event::EventKind::InquiryResult => {
                                                    // Standard: per entry = 14 bytes (addr6+psrm1+reserved2+cod3+clock2)
                                                    let data = evt.data;
                                                    if !data.is_empty() {
                                                        let n = data[0] as usize;
                                                        for i in 0..n {
                                                            let off = 1 + i * 14;
                                                            if off + 14 <= data.len() {
                                                                let mut addr = [0u8; 6];
                                                                addr.copy_from_slice(&data[off..off + 6]);
                                                                // CoD at off+9 (after addr6+psrm1+reserved2)
                                                                let cod = &data[off + 9..off + 12];
                                                                let is_hid = cod_is_peripheral(cod);
                                                                info!("[classic] Found: {:02x} CoD={:02x} hid={}", addr, cod, is_hid);
                                                                let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                                                                    crate::ble_state::BleEvent::ScanResult(
                                                                        crate::ble_state::ScanResultData {
                                                                            address: addr,
                                                                            addr_kind: 0,
                                                                            name: [0u8; 32],
                                                                            name_len: 0,
                                                                            rssi: -127,
                                                                            is_hid,
                                                                            transport_type: crate::ble_state::TransportType::Classic,
                                                                        },
                                                                    ),
                                                                );
                                                            }
                                                        }
                                                    }
                                                }
                                                bt_hci::event::EventKind::InquiryResultWithRssi => {
                                                    // With RSSI: per entry = 14 bytes (addr6+psrm1+reserved1+cod3+clock2+rssi1)
                                                    let data = evt.data;
                                                    if !data.is_empty() {
                                                        let n = data[0] as usize;
                                                        for i in 0..n {
                                                            let off = 1 + i * 14;
                                                            if off + 14 <= data.len() {
                                                                let mut addr = [0u8; 6];
                                                                addr.copy_from_slice(&data[off..off + 6]);
                                                                // CoD at off+8 (after addr6+psrm1+reserved1)
                                                                let cod = &data[off + 8..off + 11];
                                                                let is_hid = cod_is_peripheral(cod);
                                                                let rssi = data[off + 13] as i8;
                                                                info!("[classic] Found: {:02x} RSSI={} CoD={:02x} hid={}", addr, rssi, cod, is_hid);
                                                                let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                                                                    crate::ble_state::BleEvent::ScanResult(
                                                                        crate::ble_state::ScanResultData {
                                                                            address: addr,
                                                                            addr_kind: 0,
                                                                            name: [0u8; 32],
                                                                            name_len: 0,
                                                                            rssi,
                                                                            is_hid,
                                                                            transport_type: crate::ble_state::TransportType::Classic,
                                                                        },
                                                                    ),
                                                                );
                                                            }
                                                        }
                                                    }
                                                }
                                                bt_hci::event::EventKind::ExtendedInquiryResult => {
                                                    // EIR: data[0]=num, data[1..7]=addr, data[7]=psrm,
                                                    // data[8]=reserved, data[9..12]=cod, data[12..14]=clock,
                                                    // data[14]=rssi, data[15..]=eir_data
                                                    let data = evt.data;
                                                    if data.len() >= 15 {
                                                        let mut addr = [0u8; 6];
                                                        addr.copy_from_slice(&data[1..7]);
                                                        // CoD at data[9..12]
                                                        let cod = &data[9..12];
                                                        let is_hid = cod_is_peripheral(cod);
                                                        let rssi = data[14] as i8;
                                                        // Parse EIR for device name
                                                        let mut name = [0u8; 32];
                                                        let mut name_len = 0u8;
                                                        if data.len() > 15 {
                                                            let eir = &data[15..];
                                                            let mut pos = 0;
                                                            while pos + 1 < eir.len() {
                                                                let len = eir[pos] as usize;
                                                                if len == 0 { break; }
                                                                let ad_type = eir[pos + 1];
                                                                if (ad_type == 0x08 || ad_type == 0x09) && len > 1 {
                                                                    let n = (len - 1).min(name.len());
                                                                    let end = (pos + 2 + n).min(eir.len());
                                                                    let actual = end - (pos + 2);
                                                                    name[..actual].copy_from_slice(&eir[pos + 2..end]);
                                                                    name_len = actual as u8;
                                                                }
                                                                pos += 1 + len;
                                                            }
                                                        }
                                                        info!(
                                                            "[classic] Found: {:02x} rssi={} name_len={} CoD={:02x} hid={}",
                                                            addr, rssi, name_len, cod, is_hid
                                                        );
                                                        let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                                                            crate::ble_state::BleEvent::ScanResult(
                                                                crate::ble_state::ScanResultData {
                                                                    address: addr,
                                                                    addr_kind: 0,
                                                                    name,
                                                                    name_len,
                                                                    rssi,
                                                                    is_hid,
                                                                    transport_type: crate::ble_state::TransportType::Classic,
                                                                },
                                                            ),
                                                        );
                                                    }
                                                }
                                                _ => {
                                                    debug!(
                                                        "[classic] Inquiry: unhandled event {:?}",
                                                        evt.kind
                                                    );
                                                }
                                            }
                                                    }
                                                    Ok(_) => {}
                                                    Err(_) => {
                                                        Timer::after_millis(10).await;
                                                    }
                                                } // match read_result
                                            } // Either::First
                                        } // match select
                                    } // while !inquiry_done
                                }
                                Err(e) => {
                                    warn!(
                                        "[classic] Inquiry failed: {:?}",
                                        defmt::Debug2Format(&e)
                                    );
                                    crate::rpc_log::error("Classic Inquiry failed");
                                }
                            }
                        }
                        ClassicCommand::ScanStop => {}
                        ClassicCommand::Disconnect => {}
                    }
                }
            }
        };

        // --- Connect with retry ---
        // The remote device may still think it's connected from a previous session
        // (link supervision timeout not yet expired). Retry a few times with delays.
        let handle = {
            const MAX_RETRIES: u8 = 5;
            let mut attempt = 0u8;
            loop {
                attempt += 1;
                info!("[classic] Connecting (attempt {}/{})", attempt, MAX_RETRIES);
                let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                    crate::ble_state::BleEvent::StateChanged(
                        crate::protocol::ConnectionState::Connecting,
                    ),
                );
                match runner.connect(&target_addr).await {
                    Ok(h) => {
                        info!("[classic] Connected and encrypted! Handle={}", h.raw());
                        let _ = crate::ble_state::BLE_EVENT_CHANNEL
                            .try_send(crate::ble_state::BleEvent::PairingComplete);
                        break h;
                    }
                    Err(e) => {
                        warn!(
                            "[classic] Connection attempt {} failed: {:?}",
                            attempt,
                            defmt::Debug2Format(&e)
                        );
                        if attempt >= MAX_RETRIES {
                            warn!(
                                "[classic] All {} attempts failed, will retry in 30s",
                                MAX_RETRIES
                            );
                            Timer::after_secs(30).await;
                            continue 'connect;
                        }
                        Timer::after_secs(5).await;
                    }
                }
            }
        };

        // --- Mark Classic device as connected for status reporting ---
        let addr_bytes: &[u8; 6] = target_addr.raw().try_into().unwrap_or(&[0u8; 6]);
        let classic_profile_id = loaded_classic_bonds
            .iter()
            .find(|b| &b.addr == addr_bytes)
            .map(|b| b.profile_id)
            .unwrap_or(DeviceProfile::MagicTrackpad.to_id());
        slots::set_classic_connected(addr_bytes, classic_profile_id);

        // --- Persist link key to flash after successful connect ---
        {
            let key_info = {
                use bt_classic_host::LinkKeyStore;
                link_keys.borrow().load(&target_addr)
            };
            if let Some(ki) = key_info {
                let mut f = flash_mutex.lock().await;
                match bonding::store_classic_bond(
                    &mut f,
                    addr_bytes,
                    &ki.key,
                    ki.key_type,
                    classic_profile_id,
                    true, // auto_connect on first pairing
                )
                .await
                {
                    Ok(slot) => {
                        info!("[classic] Bond persisted to flash slot {}", slot);
                        let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                            crate::ble_state::BleEvent::BondStored {
                                address: *addr_bytes,
                                profile_id: classic_profile_id,
                            },
                        );
                    }
                    Err(_) => warn!("[classic] Failed to persist bond to flash"),
                }
            }
        }

        // --- Configure sniff mode for steady report delivery ---
        // Without sniff, BT Classic delivers HID reports in bursts (3-5 reports
        // per burst, 32-44ms gaps). Sniff mode forces regular polling at ~91Hz
        // (11.25ms interval), matching the real MT2's native USB report rate.
        {
            use bt_classic_host::link_policy::{
                SniffMode, WriteLinkPolicySettings, LINK_POLICY_ENABLE_SNIFF,
            };

            // Enable sniff mode in link policy
            match controller
                .exec(&WriteLinkPolicySettings::new(
                    handle,
                    LINK_POLICY_ENABLE_SNIFF,
                ))
                .await
            {
                Ok(_) => info!("[classic] Link policy: sniff enabled"),
                Err(e) => warn!(
                    "[classic] Failed to set link policy: {:?}",
                    defmt::Debug2Format(&e)
                ),
            }

            // Request sniff mode: ~91Hz (11.25ms = 18 slots of 0.625ms)
            // sniff_attempt=4: listen for 4 slots per sniff interval
            // sniff_timeout=1: minimal extra window after receiving
            match controller.exec(&SniffMode::new(handle, 18, 18, 4, 1)).await {
                Ok(_) => info!("[classic] Sniff mode requested (interval=18 slots / 11.25ms)"),
                Err(e) => warn!(
                    "[classic] Failed to request sniff mode: {:?}",
                    defmt::Debug2Format(&e)
                ),
            }
        }

        // --- Open L2CAP HID channels ---
        let mut l2cap = L2capState::<4>::new(handle);
        let mut hid = HidClient::new();

        info!("[classic] Opening HID L2CAP channels...");
        if let Err(e) = hid.open_channels(&mut l2cap, controller).await {
            error!(
                "[classic] Failed to open HID channels: {:?}",
                defmt::Debug2Format(&e)
            );
            Timer::after_secs(5).await;
            continue 'connect;
        }

        // --- Run L2CAP event loop until HID channels are open ---
        info!("[classic] Waiting for HID channels to open...");
        let mut acl_buf = [0u8; 512];
        let mut attempts = 0u16;
        while !hid.is_ready(&l2cap) {
            let mut rx = [0u8; 259];
            let rx_ref = &mut rx[..];
            match controller.read(rx_ref).await {
                Ok(bt_hci::ControllerToHostPacket::Acl(_acl)) => {
                    // rx[0] = HCI type indicator (0x02); ACL header starts at rx[1]
                    let handle_and_flags = u16::from_le_bytes([rx[1], rx[2]]);
                    let data_len = u16::from_le_bytes([rx[3], rx[4]]) as usize;
                    let acl_data = &rx[5..5 + data_len.min(rx.len() - 5)];

                    if let Err(e) = l2cap
                        .process_acl(controller, handle_and_flags, acl_data, &mut acl_buf)
                        .await
                    {
                        warn!(
                            "[classic] L2CAP error during setup: {:?}",
                            defmt::Debug2Format(&e)
                        );
                    }
                }
                Ok(_) => {} // Ignore non-ACL during channel setup
                Err(_) => {
                    Timer::after_millis(10).await;
                }
            }
            attempts += 1;
            if attempts > 5000 {
                error!("[classic] Timeout waiting for HID channels");
                Timer::after_secs(5).await;
                continue 'connect;
            }
        }

        info!("[classic] HID channels open! Activating multitouch...");

        // --- Activate Magic Trackpad 2 multitouch ---
        // BT Classic MT enable: SET_REPORT(Feature, report_id=0xF1, data={0x02, 0x01})
        // Matches the Linux kernel's magicmouse_bt_enable() exactly.
        if let Err(e) = hid
            .set_report(&l2cap, controller, ReportType::Feature, 0xF1, &[0x02, 0x01])
            .await
        {
            warn!(
                "[classic] Failed to enable multitouch: {:?}",
                defmt::Debug2Format(&e)
            );
        } else {
            info!("[classic] Multitouch enabled!");
        }

        // --- Main HID report loop ---
        info!("[classic] === CLASSIC HID SESSION ACTIVE ===");
        info!("[classic] Listening for touch reports...");
        let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
            crate::ble_state::BleEvent::StateChanged(crate::protocol::ConnectionState::Connected),
        );

        let mut report = bt_classic_host::HidReport::new();
        loop {
            let mut rx = [0u8; 259];
            let rx_ref = &mut rx[..];
            match select(controller.read(rx_ref), CLASSIC_CMD_CHANNEL.receive()).await {
                Either::Second(ClassicCommand::Disconnect) => {
                    info!("[classic] Disconnect command received");
                    break;
                }
                Either::Second(_) => {
                    // Ignore other commands (Scan/Connect) while connected
                    continue;
                }
                Either::First(Ok(bt_hci::ControllerToHostPacket::Acl(_acl))) => {
                    // rx[0] = HCI type indicator (0x02); ACL header starts at rx[1]
                    let handle_and_flags = u16::from_le_bytes([rx[1], rx[2]]);
                    let data_len = u16::from_le_bytes([rx[3], rx[4]]) as usize;
                    let acl_data = &rx[5..5 + data_len.min(rx.len() - 5)];

                    match l2cap
                        .process_acl(controller, handle_and_flags, acl_data, &mut acl_buf)
                        .await
                    {
                        Ok(Some((channel_idx, data_len))) => {
                            // L2CAP data arrived on a HID channel
                            if hid.process_data(channel_idx, &acl_buf[..data_len], &mut report) {
                                // Got a HID report from the MT2.
                                // Report ID 0x31 = BT multitouch data (4-byte header + N*9 touch points)
                                // Other report IDs are status/control — log but don't forward yet.
                                let report_id = if report.len > 0 { report.data[0] } else { 0 };

                                if report_id == 0x31 && report.len >= 4 {
                                    // Touch report — forward full BT report for reclocked passthrough
                                    let n_fingers = (report.len - 4) / 9;
                                    static MT2_LOG_COUNT: portable_atomic::AtomicU8 =
                                        portable_atomic::AtomicU8::new(0);
                                    let log_n = MT2_LOG_COUNT.load(Ordering::Relaxed);
                                    if log_n < 10 {
                                        MT2_LOG_COUNT.store(log_n + 1, Ordering::Relaxed);
                                        info!(
                                            "[classic] MT2 report #{}: fingers={} len={}",
                                            log_n, n_fingers, report.len
                                        );
                                    }

                                    let mut event = HidReportEvent::new();
                                    event.report_type = HidReportType::Mouse;
                                    event.profile = DeviceProfile::MagicTrackpad;
                                    event.slot_index = 3;
                                    event.report_id = 0x31;
                                    let copy_len = report.len.min(event.data.len());
                                    event.data[..copy_len]
                                        .copy_from_slice(&report.data[..copy_len]);
                                    event.len = copy_len;
                                    match HID_REPORT_CHANNEL.try_send(event) {
                                        Ok(()) => {}
                                        Err(_) => {
                                            debug!(
                                                "[classic] HID channel full, dropping MT2 report"
                                            );
                                        }
                                    }
                                } else {
                                    // Log non-touch reports for debugging
                                    debug!(
                                        "[classic] HID report id=0x{:02x} len={} data={:02x}",
                                        report_id,
                                        report.len,
                                        &report.data[..report.len.min(16)]
                                    );
                                }
                            }
                        }
                        Ok(None) => {} // Signaling or fragment, handled internally
                        Err(e) => {
                            warn!("[classic] L2CAP error: {:?}", defmt::Debug2Format(&e));
                        }
                    }
                }
                Either::First(Ok(bt_hci::ControllerToHostPacket::Event(event))) => {
                    // Handle disconnection during active session
                    if event.kind == bt_hci::event::EventKind::DisconnectionComplete {
                        error!("[classic] Disconnected!");
                        break;
                    }
                }
                Either::First(Ok(_)) => {}
                Either::First(Err(_)) => {
                    Timer::after_millis(1).await;
                }
            }
        }

        // Disconnected — clean up and reconnect
        warn!("[classic] Classic HID session ended, will reconnect...");
        slots::set_classic_disconnected();
        let _ =
            crate::ble_state::BLE_EVENT_CHANNEL.try_send(crate::ble_state::BleEvent::StateChanged(
                crate::protocol::ConnectionState::Disconnected,
            ));
        // Brief delay before reconnect attempt
        Timer::after_secs(2).await;
    } // end of main Classic connection loop
}
