//! BLE Central state machine and CYW43 initialization
//!
//! This module orchestrates all BLE functionality running on Core 0:
//! - CYW43 WiFi/BLE chip initialization (firmware download, PIO SPI setup)
//! - BLE stack construction with bond loading
//! - Command-driven state machine that handles scan, connect, and management commands
//!
//! Sub-modules handle specific phases of the BLE connection lifecycle:
//! - [`connection`]: BLE connection establishment and pairing
//! - [`gatt`]: GATT service discovery and HID notification loop
//! - [`commands`]: Shared command handlers (used by both idle and connected states)

pub mod commands;
pub mod connection;
pub mod gatt;
pub mod slots;

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3};
use embassy_futures::select::{select, Either};
use embassy_rp::dma;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, FLASH, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use trouble_host::prelude::*;
use trouble_host::scan::Scanner;

use crate::ble_state::{
    BleCommand, BleEvent, RpcScannerHandler, BLE_CMD_CHANNEL, BLE_EVENT_CHANNEL,
};
use crate::bonding;
use crate::device_profile::DeviceProfile;
use crate::preferences;
use crate::protocol::ConnectionState;
use crate::rpc_log;
use crate::{Irqs, FLASH_SIZE};

use self::slots::{
    ConnectRequest, SlotCommand, CONNECT_SIGNALS, MAX_CONNECTIONS, SLOT_CMD_CHANNELS,
};

/// Max BLE connections
const CONNECTIONS_MAX: usize = 3;
/// Max L2CAP channels (3 per connection: signal + att + coc)
const L2CAP_CHANNELS_MAX: usize = 9;

/// Type alias for the flash mutex shared between connection manager and slot tasks.
pub type FlashMutex = Mutex<CriticalSectionRawMutex, Flash<'static, FLASH, Async, FLASH_SIZE>>;

// ============ CYW43 Background Tasks ============

/// CYW43 task — required to run the WiFi/BLE chip (spawned on Core 0).
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

/// LED blink task using CYW43 (spawned on Core 0).
#[embassy_executor::task]
async fn led_task(control: &'static mut cyw43::Control<'static>) {
    loop {
        control.gpio_set(0, true).await;
        Timer::after_millis(1000).await;
        control.gpio_set(0, false).await;
        Timer::after_millis(1000).await;
    }
}

// ============ Core 0 Main Task ============

/// Core 0 main task: initializes CYW43, loads bonds, and runs the BLE central state machine.
///
/// Runs on Core 0 where `embassy_rp::init()` has already set up DMA and timer interrupts.
/// Flash bond operations are done inline (no cross-core channel needed since flash and
/// BLE are on the same core).
#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
pub async fn core0_ble_main(
    spawner: Spawner,
    pio0: Peri<'static, PIO0>,
    pin_23: Peri<'static, PIN_23>,
    pin_24: Peri<'static, PIN_24>,
    pin_25: Peri<'static, PIN_25>,
    pin_29: Peri<'static, PIN_29>,
    dma_ch0: Peri<'static, DMA_CH0>,
    mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>,
) -> ! {
    // --- CYW43 Initialization ---
    info!("[core0] Initializing CYW43...");

    let fw = cyw43::aligned_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");
    let btfw = cyw43::aligned_bytes!("../../cyw43-firmware/43439A0_btfw.bin");
    let nvram = cyw43::aligned_bytes!("../../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let dma = dma::Channel::new(dma_ch0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER * 4, // Work around marginal SPI signal integrity
        pio.irq0,
        cs,
        pin_24,
        pin_29,
        dma,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    let (_net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;

    // Spawn CYW43 background task on Core 0
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    info!("[core0] CYW43 initialized with Bluetooth");

    // Spawn LED blink task
    static CONTROL: StaticCell<cyw43::Control<'static>> = StaticCell::new();
    let control = CONTROL.init(control);
    unwrap!(spawner.spawn(led_task(control)));

    // --- Load bonds and preferences from flash ---
    // Safe now that CYW43 firmware is fully downloaded.
    // Flash erase/write pauses Core 1 (USB) via FIFO, which is tolerable.
    let mut loaded_bonds = bonding::load_bonds(&mut flash).await;
    info!(
        "[core0] Loaded {} stored bond(s) from flash",
        loaded_bonds.len()
    );
    for lb in &loaded_bonds {
        info!(
            "  - Device: {:?} (profile: {:?})",
            lb.bond.identity.bd_addr,
            DeviceProfile::from_id(lb.profile_id)
        );
    }

    // Load Classic BT bonds from flash
    let loaded_classic_bonds = bonding::load_classic_bonds(&mut flash).await;
    if !loaded_classic_bonds.is_empty() {
        for cb in &loaded_classic_bonds {
            info!(
                "  - Classic: {:02x} (profile: {:?})",
                cb.addr,
                DeviceProfile::from_id(cb.profile_id)
            );
        }
    }

    let mut active_device_pref = preferences::load_active_device(&mut flash).await;

    // Load forced OS override from flash and cache to scratch register.
    // Phase 0 checks this on the next soft reset to skip the probe.
    {
        let forced_os =
            preferences::load_u32_preference(&mut flash, preferences::PREF_KEY_FORCED_OS, 0).await;
        crate::scratch::write_forced_os(forced_os);
    }

    // Load axis multiplier preferences
    {
        use core::sync::atomic::Ordering::Relaxed;
        let scroll =
            preferences::load_multiplier(&mut flash, preferences::PREF_KEY_SCROLL_MULTIPLIER).await;
        let pan =
            preferences::load_multiplier(&mut flash, preferences::PREF_KEY_PAN_MULTIPLIER).await;
        let x = preferences::load_multiplier(&mut flash, preferences::PREF_KEY_X_MULTIPLIER).await;
        let y = preferences::load_multiplier(&mut flash, preferences::PREF_KEY_Y_MULTIPLIER).await;
        crate::usb_hid::MULTIPLIER_SCROLL.store(scroll, Relaxed);
        crate::usb_hid::MULTIPLIER_PAN.store(pan, Relaxed);
        crate::usb_hid::MULTIPLIER_X.store(x, Relaxed);
        crate::usb_hid::MULTIPLIER_Y.store(y, Relaxed);
        let threshold = preferences::load_u32_preference(
            &mut flash,
            preferences::PREF_KEY_SCROLL_THRESHOLD,
            120,
        )
        .await;
        let max_detents =
            preferences::load_u32_preference(&mut flash, preferences::PREF_KEY_MAX_DETENTS, 3)
                .await;
        crate::usb_hid::SCROLL_THRESHOLD.store(threshold, Relaxed);
        crate::usb_hid::MAX_DETENTS_PER_EMIT.store(max_detents, Relaxed);
        info!(
            "[core0] Axis multipliers: scroll={}% pan={}% x={}% y={}%",
            scroll, pan, x, y
        );
        info!(
            "[core0] Scroll params: threshold={} max_detents={}",
            threshold, max_detents
        );
    }

    // Validate ActiveDevice preference against loaded bonds.
    // A stale preference (e.g. bonds cleared but preference persists) would
    // cause a futile auto-connect attempt on boot.
    if let Some(ref dev) = active_device_pref {
        let has_matching_bond = loaded_bonds
            .iter()
            .any(|lb| lb.bond.identity.bd_addr.raw() == dev.address);
        if has_matching_bond {
            info!("[core0] Active device preference: {:?}", dev.address);
        } else {
            warn!(
                "[core0] Ignoring stale active device {:?} (no matching bond)",
                dev.address
            );
            active_device_pref = None;
        }
    }
    // --- Build HCI multiplexer for dual-mode BT (Classic + BLE) ---
    static MUX_RES: StaticCell<hci_mux::MuxResources<10, 4>> = StaticCell::new();
    let mux_res = MUX_RES.init(hci_mux::MuxResources::new());
    let mux = hci_mux::HciMux::new(bt_device, mux_res);
    let (le_controller, classic_controller, mux_runner) = mux.split();

    // The LE controller is passed to trouble-host (replaces ExternalController).
    let controller = le_controller;

    // Static random address: MSB must have bits 11 (>= 0xC0)
    let address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xca]);
    info!("[core0] Our BLE address: {:?}", address);

    static RESOURCES: StaticCell<
        HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>,
    > = StaticCell::new();
    let resources = RESOURCES.init(HostResources::new());

    // Create RNG from RP2040's Ring Oscillator (ROSC) for hardware entropy
    use embassy_rp::clocks::RoscRng;
    let mut rosc_rng = RoscRng;
    let mut rng_seed = [0u8; 32];
    rosc_rng.fill_bytes(&mut rng_seed);
    use rand_chacha::ChaCha8Rng;
    use rand_core::SeedableRng;
    let mut rng = ChaCha8Rng::from_seed(rng_seed);

    let stack = trouble_host::new(controller, resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut rng);

    // Add all loaded bonds to the BLE stack BEFORE building
    for lb in &loaded_bonds {
        match stack.add_bond_information(lb.bond.clone()) {
            Ok(()) => {
                info!(
                    "[core0] Added bond for {:?} to BLE stack",
                    lb.bond.identity.bd_addr
                );
            }
            Err(e) => {
                error!(
                    "[core0] Failed to add bond for {:?}: {:?}",
                    lb.bond.identity.bd_addr, e
                );
            }
        }
    }

    let Host {
        central,
        mut runner,
        ..
    } = stack.build();

    let scanner_handler = RpcScannerHandler;
    let mut scanner = Scanner::new(central);

    info!(
        "[core0] BLE Central initialized ({} slots)",
        MAX_CONNECTIONS
    );

    // Wrap flash in a mutex for shared access between manager and slot tasks
    static FLASH_MUTEX: StaticCell<FlashMutex> = StaticCell::new();
    let flash_mutex = FLASH_MUTEX.init(Mutex::new(flash));

    // Auto-connect is handled by the background scan in the connection manager.
    // It detects bonded auto_connect devices as they advertise and connects them
    // one at a time, avoiding HCI command collisions.
    // Legacy: single active device preference (queues one connect at most).
    if !loaded_bonds.iter().any(|lb| lb.auto_connect) && active_device_pref.is_some() {
        let _ = BLE_CMD_CHANNEL.try_send(BleCommand::AutoConnect);
    }

    // --- Run HCI mux, BLE host, Classic host, and slot tasks concurrently ---
    let _ = join(
        // HCI mux runner: routes packets between BLE and Classic stacks
        mux_runner.run(),
        join(
            runner.run_with_handler(&scanner_handler),
            join(
                connection_manager_loop(
                    &mut scanner,
                    &stack,
                    flash_mutex,
                    &mut loaded_bonds,
                    &loaded_classic_bonds,
                    &active_device_pref,
                ),
                join(
                    join3(
                        connection_slot_task(0, &stack, flash_mutex, &active_device_pref),
                        connection_slot_task(1, &stack, flash_mutex, &active_device_pref),
                        connection_slot_task(2, &stack, flash_mutex, &active_device_pref),
                    ),
                    // Classic BT task — connects to MT2 etc.
                    classic_bt_task(
                        &classic_controller,
                        flash_mutex,
                        &loaded_classic_bonds,
                    ),
                ),
            ),
        ),
    )
    .await;

    // Should never reach here
    loop {
        Timer::after_millis(1000).await;
    }
}

// ============ Classic BT Task ============

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

/// Classic Bluetooth task — connects to Classic HID devices (Magic Trackpad 2).
///
/// Runs the full connection lifecycle: ACL → Auth → Encrypt → L2CAP → HIDP.
/// Forwards received HID reports to HID_REPORT_CHANNEL for USB translation.
async fn classic_bt_task<C>(
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
        > + bt_hci::controller::ControllerCmdSync<
            bt_classic_host::link_policy::WriteLinkPolicySettings,
        > + bt_hci::controller::ControllerCmdSync<bt_classic_host::link_policy::SniffMode>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::Inquiry>
        + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::link_control::InquiryCancel>,
{
    use core::cell::RefCell;
    use core::sync::atomic::Ordering;

    use bt_classic_host::hidp::ReportType;
    use bt_classic_host::{ClassicRunner, HidClient, HostResources, L2capState};

    use crate::ble_hid::{HidReportEvent, HidReportType, HID_REPORT_CHANNEL};
    use crate::device_profile::DeviceProfile;

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
    let mut runner = ClassicRunner::new(controller, &mut resources, &link_keys);

    // --- Resolve target address: auto-connect bond, or wait for command ---
    let target_addr = {
        use crate::ble_state::{ClassicCommand, CLASSIC_CMD_CHANNEL};

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

                        // GIAC LAP = 0x9E8B33, duration 8 (10.24s), unlimited responses
                        match controller
                            .exec(&bt_hci::cmd::link_control::Inquiry::new(
                                [0x33, 0x8B, 0x9E],
                                8,
                                0,
                            ))
                            .await
                        {
                            Ok(_) => {
                                // Read Inquiry events until InquiryComplete
                                let mut rx = [0u8; 259];
                                let rx_ref = unsafe {
                                    core::slice::from_raw_parts_mut(rx.as_mut_ptr(), rx.len())
                                };
                                loop {
                                    match controller.read(rx_ref).await {
                                        Ok(bt_hci::ControllerToHostPacket::Event(evt)) => {
                                            match evt.kind {
                                                bt_hci::event::EventKind::InquiryComplete => {
                                                    info!("[classic] Inquiry complete");
                                                    crate::rpc_log::info(
                                                        "Classic Inquiry scan complete",
                                                    );
                                                    break;
                                                }
                                                bt_hci::event::EventKind::InquiryResult
                                                | bt_hci::event::EventKind::InquiryResultWithRssi => {
                                                    // Parse addresses from raw event data
                                                    // data[0] = num_responses, then BD_ADDRs (6 bytes each)
                                                    let data = evt.data;
                                                    if !data.is_empty() {
                                                        let n = data[0] as usize;
                                                        for i in 0..n {
                                                            let off = 1 + i * 6;
                                                            if off + 6 <= data.len() {
                                                                let mut addr = [0u8; 6];
                                                                addr.copy_from_slice(&data[off..off + 6]);
                                                                info!("[classic] Found: {:02x}", addr);
                                                                let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                                                                    crate::ble_state::BleEvent::ScanResult(
                                                                        crate::ble_state::ScanResultData {
                                                                            address: addr,
                                                                            addr_kind: 0,
                                                                            name: [0u8; 32],
                                                                            name_len: 0,
                                                                            rssi: -127,
                                                                            is_hid: true,
                                                                            transport_type: 1,
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
                                                            "[classic] Found: {:02x} rssi={} name_len={}",
                                                            addr, rssi, name_len
                                                        );
                                                        let _ = crate::ble_state::BLE_EVENT_CHANNEL.try_send(
                                                            crate::ble_state::BleEvent::ScanResult(
                                                                crate::ble_state::ScanResultData {
                                                                    address: addr,
                                                                    addr_kind: 0,
                                                                    name,
                                                                    name_len,
                                                                    rssi,
                                                                    is_hid: true,
                                                                    transport_type: 1,
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
                                    }
                                }
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
            info!(
                "[classic] Connecting (attempt {}/{})",
                attempt, MAX_RETRIES
            );
            match runner.connect(&target_addr).await {
                Ok(h) => {
                    info!("[classic] Connected and encrypted! Handle={}", h.raw());
                    break h;
                }
                Err(e) => {
                    warn!(
                        "[classic] Connection attempt {} failed: {:?}",
                        attempt,
                        defmt::Debug2Format(&e)
                    );
                    if attempt >= MAX_RETRIES {
                        error!("[classic] All {} attempts failed, idling", MAX_RETRIES);
                        loop {
                            Timer::after_secs(3600).await;
                        }
                    }
                    Timer::after_secs(5).await;
                }
            }
        }
    };

    // --- Mark Classic device as connected for status reporting ---
    let classic_profile_id = DeviceProfile::MagicTrackpad.to_id();
    slots::set_classic_connected(
        target_addr.raw().try_into().unwrap_or(&[0u8; 6]),
        classic_profile_id,
    );

    // --- Persist link key to flash after successful connect ---
    {
        let key_info = {
            use bt_classic_host::LinkKeyStore;
            link_keys.borrow().load(&target_addr)
        };
        if let Some(ki) = key_info {
            let mut f = flash_mutex.lock().await;
            let addr_bytes: &[u8; 6] = target_addr.raw().try_into().unwrap_or(&[0u8; 6]);
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
                Ok(slot) => info!("[classic] Bond persisted to flash slot {}", slot),
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
            .exec(&WriteLinkPolicySettings::new(handle, LINK_POLICY_ENABLE_SNIFF))
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
        match controller
            .exec(&SniffMode::new(handle, 18, 18, 4, 1))
            .await
        {
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
        loop {
            Timer::after_secs(3600).await;
        }
    }

    // --- Run L2CAP event loop until HID channels are open ---
    info!("[classic] Waiting for HID channels to open...");
    let mut acl_buf = [0u8; 512];
    let mut attempts = 0u16;
    while !hid.is_ready(&l2cap) {
        let mut rx = [0u8; 259];
        let rx_ref = unsafe { core::slice::from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };
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
            loop {
                Timer::after_secs(3600).await;
            }
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

    let mut report = bt_classic_host::HidReport::new();
    loop {
        let mut rx = [0u8; 259];
        let rx_ref = unsafe { core::slice::from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };
        match controller.read(rx_ref).await {
            Ok(bt_hci::ControllerToHostPacket::Acl(_acl)) => {
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
                                event.data[..copy_len].copy_from_slice(&report.data[..copy_len]);
                                event.len = copy_len;
                                match HID_REPORT_CHANNEL.try_send(event) {
                                    Ok(()) => {}
                                    Err(_) => {
                                        debug!("[classic] HID channel full, dropping MT2 report");
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
            Ok(bt_hci::ControllerToHostPacket::Event(event)) => {
                // Handle disconnection during active session
                if event.kind == bt_hci::event::EventKind::DisconnectionComplete {
                    error!("[classic] Disconnected!");
                    break;
                }
            }
            Ok(_) => {}
            Err(_) => {
                Timer::after_millis(1).await;
            }
        }
    }

    // Disconnected — idle until reset
    warn!("[classic] Classic HID session ended");
    loop {
        Timer::after_secs(3600).await;
    }
}

// ============ Connection Manager ============

/// Connection manager loop — handles global commands and dispatches connect/disconnect
/// to slot tasks. Runs concurrently with slot tasks and the BLE runner.
async fn connection_manager_loop<
    'a,
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
    _stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
    loaded_bonds: &mut heapless::Vec<bonding::LoadedBond, { bonding::MAX_BONDS }>,
    loaded_classic_bonds: &[bonding::StoredClassicBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) {
    let mut manager_profile = determine_initial_profile(active_device_pref, loaded_bonds);

    info!("[manager] Connection manager started");

    loop {
        // Check if we should run a background scan for bonded auto-connect devices
        let unconnected = get_unconnected_auto_connect_addresses(loaded_bonds);
        let has_idle_slot = slots::find_idle_slot().is_some();

        // Don't start a background scan while any slot is actively connecting —
        // the scan would take the HCI scan state and cause "Command Disallowed"
        // when the slot retries its LE Create Connection.
        let any_connecting = slots::any_slot_connecting();

        let cmd = if !unconnected.is_empty() && has_idle_slot && !any_connecting {
            match run_background_scan(scanner, &unconnected).await {
                BgScanOutcome::BondedDeviceSeen(addr) => {
                    info!("[manager] Background scan: bonded device seen {:?}", addr);
                    rpc_log::info("Bonded device detected, connecting...");
                    // Small delay for HCI state to settle after scan stop
                    Timer::after_millis(50).await;
                    let addr_kind = if (addr[5] & 0xC0) == 0xC0 { 1u8 } else { 0u8 };
                    BleCommand::Connect {
                        address: addr,
                        addr_kind,
                        ignore_bond: false,
                    }
                }
                BgScanOutcome::Command(cmd) => {
                    // Small delay for HCI state to settle after scan stop
                    Timer::after_millis(50).await;
                    cmd
                }
                BgScanOutcome::Timeout => continue, // Refresh filter list and restart scan
            }
        } else {
            // No background scanning needed. Periodically re-check in case
            // a slot goes idle (device disconnects while we're waiting).
            match embassy_futures::select::select(
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(5)),
            )
            .await
            {
                embassy_futures::select::Either::First(cmd) => cmd,
                embassy_futures::select::Either::Second(_) => continue,
            }
        };

        match cmd {
            BleCommand::StartScan => {
                run_scan_session(scanner).await;
            }

            BleCommand::StopScan => {}

            BleCommand::Connect {
                address,
                addr_kind,
                ignore_bond: _,
            } => {
                // Check if already connected to this address
                if slots::find_slot_by_address(&address).is_some() {
                    info!("[manager] Already connected to {:?}", address);
                    rpc_log::warn("Already connected to this device");
                    continue;
                }

                // Find an idle slot
                match slots::find_idle_slot() {
                    Some(slot) => {
                        let has_bond = loaded_bonds
                            .iter()
                            .any(|lb| lb.bond.identity.bd_addr.raw() == address);

                        let profile = loaded_bonds
                            .iter()
                            .find(|lb| lb.bond.identity.bd_addr.raw() == address)
                            .map(|lb| DeviceProfile::from_id(lb.profile_id))
                            .unwrap_or(DeviceProfile::Generic);

                        info!(
                            "[manager] Assigning slot {} for {:?} (profile: {:?})",
                            slot, address, profile
                        );
                        CONNECT_SIGNALS[slot].signal(ConnectRequest {
                            address,
                            addr_kind,
                            profile,
                            has_stored_bond: has_bond,
                        });
                    }
                    None => {
                        warn!(
                            "[manager] No idle slots available ({}/{})",
                            MAX_CONNECTIONS, MAX_CONNECTIONS
                        );
                        rpc_log::error("All connection slots are in use");
                    }
                }
            }

            BleCommand::AutoConnect => {
                // Resolve target from preferences or first bond
                let target = resolve_auto_connect_target(loaded_bonds, active_device_pref);
                if let Some((addr, addr_kind, profile)) = target {
                    if slots::find_slot_by_address(&addr).is_some() {
                        info!("[manager] Auto-connect: already connected to {:?}", addr);
                        continue;
                    }
                    if let Some(slot) = slots::find_idle_slot() {
                        info!(
                            "[manager] Auto-connecting slot {} to {:?} (profile: {:?})",
                            slot, addr, profile
                        );
                        rpc_log::info("Auto-connecting to bonded device");
                        CONNECT_SIGNALS[slot].signal(ConnectRequest {
                            address: addr,
                            addr_kind,
                            profile,
                            has_stored_bond: true,
                        });
                    } else {
                        warn!("[manager] No idle slots for auto-connect");
                    }
                }
            }

            BleCommand::Disconnect { address } => {
                match address {
                    Some(addr) => {
                        // Disconnect a specific device
                        if let Some(slot) = slots::find_slot_by_address(&addr) {
                            let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                        } else {
                            info!("[manager] Device {:?} not connected", addr);
                        }
                    }
                    None => {
                        // Disconnect all
                        for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                            if !slots::is_slot_idle(i) {
                                let _ = ch.try_send(SlotCommand::Disconnect);
                            }
                        }
                    }
                }
            }

            BleCommand::SetActiveDevice { address, addr_kind } => {
                let mut f = flash.lock().await;
                commands::handle_set_active_device(&mut f, address, addr_kind).await;
            }

            BleCommand::ClearActiveDevice => {
                let mut f = flash.lock().await;
                commands::handle_clear_active_device(&mut f).await;
            }

            BleCommand::UpdateBondProfile {
                address,
                profile_id,
            } => {
                {
                    let mut f = flash.lock().await;
                    if let Some(new_bonds) =
                        commands::handle_update_bond_profile(&mut f, &address, profile_id).await
                    {
                        *loaded_bonds = new_bonds;
                    }
                }
                // Also notify the slot if connected
                if let Some(slot) = slots::find_slot_by_address(&address) {
                    let _ =
                        SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::UpdateProfile(profile_id));
                }
            }

            BleCommand::GetStatus => {
                debug!("Getting status info");
                commands::handle_get_status(loaded_bonds, manager_profile, active_device_pref);
            }

            BleCommand::GetBonds => {
                debug!("Getting bonds list");
                commands::handle_get_bonds(loaded_bonds, loaded_classic_bonds);
            }

            BleCommand::ClearBonds => {
                // Disconnect all first
                for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                    if !slots::is_slot_idle(i) {
                        let _ = ch.try_send(SlotCommand::Disconnect);
                    }
                }
                // Brief delay for disconnects to process
                Timer::after_millis(200).await;

                let mut f = flash.lock().await;
                commands::handle_clear_bonds(&mut f).await;
                // handle_clear_bonds resets on success; if we get here it failed
                loaded_bonds.clear();
            }

            BleCommand::ClearBond { address } => {
                // Disconnect the device if connected
                if let Some(slot) = slots::find_slot_by_address(&address) {
                    let _ = SLOT_CMD_CHANNELS[slot].try_send(SlotCommand::Disconnect);
                    Timer::after_millis(200).await;
                }
                let mut f = flash.lock().await;
                commands::handle_clear_bond(&mut f, &address, loaded_bonds).await;
            }

            BleCommand::SetConfig { key, value } => {
                let mut f = flash.lock().await;
                commands::handle_set_config(&mut f, key, value).await;
            }

            BleCommand::SetForcedOs { os } => {
                let mut f = flash.lock().await;
                commands::handle_set_forced_os(&mut f, os).await;
            }

            BleCommand::SetAutoConnect { address, enabled } => {
                let mut f = flash.lock().await;
                commands::handle_set_auto_connect(&mut f, &address, enabled, loaded_bonds).await;
            }

            BleCommand::Restart => {
                commands::handle_restart().await;
            }

            BleCommand::FactoryReset => {
                // Disconnect all first
                for (i, ch) in SLOT_CMD_CHANNELS.iter().enumerate() {
                    if !slots::is_slot_idle(i) {
                        let _ = ch.try_send(SlotCommand::Disconnect);
                    }
                }
                Timer::after_millis(200).await;

                let mut f = flash.lock().await;
                commands::handle_factory_reset(&mut f).await;
                // handle_factory_reset resets on success; if we get here it failed
                loaded_bonds.clear();
            }
        }

        // Update manager's profile from the first connected slot (for status reporting)
        for i in 0..MAX_CONNECTIONS {
            if !slots::is_slot_idle(i) {
                manager_profile = DeviceProfile::from_id(
                    slots::SLOT_PROFILES[i].load(core::sync::atomic::Ordering::Relaxed),
                );
                break;
            }
        }
    }
}

// ============ Connection Slot Task ============

/// Connection slot task — waits for connect signals, runs the connection lifecycle.
///
/// Each slot runs independently. When signaled by the connection manager, it:
/// 1. Creates its own Central via `stack.build()`
/// 2. Connects to the target device
/// 3. Runs the GATT session (HID report forwarding)
/// 4. On disconnect, marks itself idle and loops back
async fn connection_slot_task<'a, C: Controller>(
    slot: usize,
    stack: &'a Stack<'a, C, DefaultPacketPool>,
    flash: &'a FlashMutex,
    active_device_pref: &Option<preferences::ActiveDevice>,
) {
    info!("[slot{}] Connection slot task started", slot);

    loop {
        // Wait for the connection manager to signal us
        let request = CONNECT_SIGNALS[slot].wait().await;
        info!(
            "[slot{}] Connect request: {:?} (profile: {:?})",
            slot, request.address, request.profile
        );

        slots::set_slot_connecting(slot, request.address, request.profile);
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Connecting));

        let kind = if request.addr_kind == 1 {
            AddrKind::RANDOM
        } else {
            AddrKind::PUBLIC
        };
        let target = Address {
            kind,
            addr: BdAddr::new(request.address),
        };

        let mut active_profile = request.profile;

        // Run the connection lifecycle (connect, pair, GATT).
        // When connection ends, mark slot idle. The manager's background scan
        // will detect the device if it re-advertises and signal this slot again.
        connection::ble_connect_and_run(
            stack,
            flash,
            target,
            &mut active_profile,
            request.has_stored_bond,
            slot,
            active_device_pref,
        )
        .await;

        info!("[slot{}] Connection ended, marking idle", slot);
        slots::set_slot_idle(slot);
        let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Disconnected));
    }
}

// ============ Helpers ============

/// Determine the initial device profile from preferences or bond data.
fn determine_initial_profile(
    active_device_pref: &Option<preferences::ActiveDevice>,
    loaded_bonds: &[bonding::LoadedBond],
) -> DeviceProfile {
    if let Some(ref pref) = active_device_pref {
        loaded_bonds
            .iter()
            .find(|lb| lb.bond.identity.bd_addr.raw() == pref.address)
            .map(|lb| DeviceProfile::from_id(lb.profile_id))
            .unwrap_or(DeviceProfile::Generic)
    } else if !loaded_bonds.is_empty() {
        DeviceProfile::from_id(loaded_bonds[0].profile_id)
    } else {
        DeviceProfile::Generic
    }
}

/// Resolve the auto-connect target from preferences or bonds.
/// Returns (address, addr_kind, profile) or None.
fn resolve_auto_connect_target(
    loaded_bonds: &[bonding::LoadedBond],
    active_device_pref: &Option<preferences::ActiveDevice>,
) -> Option<([u8; 6], u8, DeviceProfile)> {
    let (target_addr, target_kind) = if let Some(ref pref) = active_device_pref {
        (pref.address, pref.addr_kind)
    } else if !loaded_bonds.is_empty() {
        let lb = &loaded_bonds[0];
        let addr_bytes = lb.bond.identity.bd_addr.raw();
        let kind = if (addr_bytes[5] & 0xC0) == 0xC0 {
            1u8
        } else {
            0u8
        };
        let mut addr = [0u8; 6];
        addr.copy_from_slice(addr_bytes);
        (addr, kind)
    } else {
        warn!("AutoConnect: no active device or bonds");
        rpc_log::warn("AutoConnect: no active device or bonds");
        return None;
    };

    let bond_info = loaded_bonds
        .iter()
        .find(|lb| lb.bond.identity.bd_addr.raw() == target_addr);

    match bond_info {
        Some(lb) => {
            let profile = DeviceProfile::from_id(lb.profile_id);
            Some((target_addr, target_kind, profile))
        }
        None => {
            warn!(
                "AutoConnect: no bond found for active device {:?}",
                target_addr
            );
            rpc_log::warn("AutoConnect: device not bonded");
            None
        }
    }
}

// ============ Background Scan for Auto-Reconnect ============

/// Outcome of the background scan.
enum BgScanOutcome {
    /// A bonded device was seen advertising.
    BondedDeviceSeen([u8; 6]),
    /// A BLE command arrived while scanning.
    Command(BleCommand),
    /// Scan timed out (time to refresh the filter list).
    Timeout,
}

/// Returns addresses of bonded devices with `auto_connect=true` that aren't
/// currently connected in any slot.
fn get_unconnected_auto_connect_addresses(
    loaded_bonds: &[bonding::LoadedBond],
) -> heapless::Vec<([u8; 6], u8), { bonding::MAX_BONDS }> {
    let mut addrs = heapless::Vec::new();
    for lb in loaded_bonds {
        if !lb.auto_connect {
            continue;
        }
        let mut addr = [0u8; 6];
        addr.copy_from_slice(lb.bond.identity.bd_addr.raw());
        if slots::find_slot_by_address(&addr).is_some() {
            continue;
        }
        let addr_kind = if (addr[5] & 0xC0) == 0xC0 { 1u8 } else { 0u8 };
        let _ = addrs.push((addr, addr_kind));
    }
    addrs
}

/// Run a passive background scan for bonded-but-not-connected devices.
///
/// Starts a passive scan with a filter accept list of bonded addresses.
/// Returns when a bonded device is seen, a command arrives, or the scan times out.
async fn run_background_scan<
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
    bonded_addrs: &[([u8; 6], u8)],
) -> BgScanOutcome {
    use bt_hci::param::AddrKind;
    use trouble_host::prelude::BdAddr;

    // Build filter accept list from bonded addresses
    let mut bd_addrs: heapless::Vec<BdAddr, { bonding::MAX_BONDS }> = heapless::Vec::new();
    for (addr, _) in bonded_addrs {
        let _ = bd_addrs.push(BdAddr::new(*addr));
    }
    let mut filter_list: heapless::Vec<(AddrKind, &BdAddr), { bonding::MAX_BONDS }> =
        heapless::Vec::new();
    for (i, (_, kind)) in bonded_addrs.iter().enumerate() {
        let addr_kind = if *kind == 1 {
            AddrKind::RANDOM
        } else {
            AddrKind::PUBLIC
        };
        let _ = filter_list.push((addr_kind, &bd_addrs[i]));
    }

    let scan_config = trouble_host::connection::ScanConfig {
        active: false, // Passive scan — lower power
        filter_accept_list: &filter_list,
        interval: embassy_time::Duration::from_millis(200),
        window: embassy_time::Duration::from_millis(100),
        timeout: embassy_time::Duration::from_secs(0), // No HCI timeout
        ..Default::default()
    };

    // Set the filter state for the scanner handler
    crate::ble_state::set_bg_scan_filter(bonded_addrs);
    crate::ble_state::BONDED_DEVICE_SEEN.reset();
    crate::ble_state::BG_SCAN_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);

    let result = match scanner.scan(&scan_config).await {
        Ok(_session) => {
            // _session keeps scan alive; dropping it stops the scan
            match embassy_futures::select::select3(
                crate::ble_state::BONDED_DEVICE_SEEN.wait(),
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(60)),
            )
            .await
            {
                embassy_futures::select::Either3::First(addr) => {
                    BgScanOutcome::BondedDeviceSeen(addr)
                }
                embassy_futures::select::Either3::Second(cmd) => BgScanOutcome::Command(cmd),
                embassy_futures::select::Either3::Third(_) => BgScanOutcome::Timeout,
            }
        }
        Err(_) => {
            // Scan failed to start (e.g. HCI busy) — just wait for a command
            BgScanOutcome::Command(BLE_CMD_CHANNEL.receive().await)
        }
    };

    crate::ble_state::BG_SCAN_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
    result
}

/// Run a BLE scan session with a 30-second timeout, interruptible by commands.
///
/// Scan results are emitted via the scanner_handler's `on_adv_reports` callback.
/// The scan ends on: StopScan command, Connect command (re-queued), or 30s timeout.
async fn run_scan_session<
    C: Controller + bt_hci::controller::ControllerCmdSync<bt_hci::cmd::le::LeSetScanParams>,
>(
    scanner: &mut Scanner<'_, C, DefaultPacketPool>,
) {
    info!("Starting BLE scan...");
    rpc_log::info("Scanning for BLE HID devices");
    crate::ble_state::hid_cache_clear();
    let _ = BLE_EVENT_CHANNEL.try_send(BleEvent::StateChanged(ConnectionState::Scanning));

    let scan_config = trouble_host::connection::ScanConfig {
        active: true,
        interval: embassy_time::Duration::from_millis(100),
        window: embassy_time::Duration::from_millis(100),
        ..Default::default()
    };

    match scanner.scan(&scan_config).await {
        Ok(_session) => loop {
            match select(
                BLE_CMD_CHANNEL.receive(),
                Timer::after(embassy_time::Duration::from_secs(30)),
            )
            .await
            {
                Either::First(BleCommand::StopScan) => {
                    info!("Scan stopped by command");
                    rpc_log::info("Scan stopped");
                    break;
                }
                Either::First(BleCommand::Connect {
                    address,
                    addr_kind,
                    ignore_bond,
                }) => {
                    info!("Connect command during scan");
                    let _ = BLE_CMD_CHANNEL.try_send(BleCommand::Connect {
                        address,
                        addr_kind,
                        ignore_bond,
                    });
                    break;
                }
                Either::First(_) => {}
                Either::Second(_) => {
                    info!("Scan timeout (30s)");
                    rpc_log::info("Scan timeout (30s)");
                    break;
                }
            }
        },
        Err(e) => {
            error!("Failed to start scan: {:?}", defmt::Debug2Format(&e));
        }
    }
}
