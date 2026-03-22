//! Phase 0 probe USB device for OS fingerprinting.
//!
//! Presents a minimal vendor-HID device during Phase 0 to fingerprint the host
//! OS via USB enumeration behavior. After collecting signals for up to 1 second,
//! writes the detected OS to scratch registers and triggers a watchdog reset
//! to enter Phase 1 with the appropriate USB configuration.

use core::sync::atomic::{AtomicBool, Ordering};
use defmt::*;
use embassy_executor::Executor;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_usb::class::hid::{HidWriter, ReportId};
use embassy_usb::control::OutResponse;
use embassy_usb::types::StringIndex;
use static_cell::StaticCell;

use crate::scratch::{self, DetectedOs};
use crate::{Irqs, CORE1_EXECUTOR_READY, EXECUTOR1};

// ============ Fingerprinting Signals ============

/// Windows requests String descriptor 0xEE (MS OS String Descriptor).
static PROBE_STRING_0X_EE: AtomicBool = AtomicBool::new(false);

/// macOS skips String 0 (LangID list); Windows and Linux request it.
static PROBE_LANG_ID_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Windows and Linux send SET_IDLE for HID interfaces; macOS typically doesn't.
static PROBE_SET_IDLE_RECEIVED: AtomicBool = AtomicBool::new(false);

/// Set when the host sends SET_CONFIGURATION (device is fully enumerated).
static PROBE_CONFIGURED: AtomicBool = AtomicBool::new(false);

/// Bus reset count (Windows typically does 2 before SET_ADDRESS).
/// Using AtomicBool as a "seen at least one reset" flag since AtomicU8
/// doesn't support fetch_add on Cortex-M0+.
static PROBE_RESET_SEEN: AtomicBool = AtomicBool::new(false);

// ============ Probe HID Descriptor ============

/// Minimal vendor-usage-page HID report descriptor.
///
/// Single 1-byte input report on Vendor page 0xFF00. No OS will bind a
/// keyboard/mouse driver to this — it triggers standard HID enumeration
/// (SET_IDLE, GET_DESCRIPTOR(HID Report)) without creating phantom devices.
const PROBE_REPORT_DESC: &[u8] = &[
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01, // Usage (Vendor Usage 1)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Vendor Usage 1)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x02, //   Input (Data, Variable, Absolute)
    0xC0, // End Collection
];

// ============ Probe USB Handlers ============

/// Device-level handler that tracks USB enumeration signals for fingerprinting.
struct ProbeDeviceHandler;

impl embassy_usb::Handler for ProbeDeviceHandler {
    fn reset(&mut self) {
        PROBE_RESET_SEEN.store(true, Ordering::Relaxed);
        debug!("[probe] bus reset");
    }

    fn configured(&mut self, configured: bool) {
        if configured {
            PROBE_CONFIGURED.store(true, Ordering::Relaxed);
            info!("[probe] USB configured");
        }
    }

    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == StringIndex(0xEE) {
            PROBE_STRING_0X_EE.store(true, Ordering::Relaxed);
            info!("[probe] String 0xEE requested -> Windows signal");
        }
        if index == StringIndex(0) {
            PROBE_LANG_ID_REQUESTED.store(true, Ordering::Relaxed);
            debug!("[probe] LangID (String 0) requested");
        }
        None
    }
}

/// HID request handler that tracks SET_IDLE for fingerprinting.
struct ProbeRequestHandler;

impl embassy_usb::class::hid::RequestHandler for ProbeRequestHandler {
    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {
        PROBE_SET_IDLE_RECEIVED.store(true, Ordering::Relaxed);
        info!("[probe] SET_IDLE received");
    }

    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }

    fn get_report(&mut self, _id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        None
    }

    fn set_report(&mut self, _id: ReportId, _data: &[u8]) -> OutResponse {
        OutResponse::Accepted
    }
}

// ============ Detection Logic ============

/// Evaluate the collected fingerprinting signals and determine the host OS.
fn evaluate_os() -> DetectedOs {
    // Definite signal: Windows requests String 0xEE (MS OS String Descriptor)
    if PROBE_STRING_0X_EE.load(Ordering::Relaxed) {
        return DetectedOs::Windows;
    }

    let lang_id = PROBE_LANG_ID_REQUESTED.load(Ordering::Relaxed);
    let set_idle = PROBE_SET_IDLE_RECEIVED.load(Ordering::Relaxed);

    // macOS skips LangID (String 0) and doesn't send SET_IDLE for vendor HID
    if !lang_id && !set_idle {
        return DetectedOs::MacOs;
    }

    // Linux sends SET_IDLE and requests LangID, but never String 0xEE
    if set_idle {
        return DetectedOs::Linux;
    }

    DetectedOs::Unknown
}

// ============ Probe Tasks ============

/// USB device task for the probe phase.
#[embassy_executor::task]
async fn probe_usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// Fingerprint evaluation task.
///
/// Waits for USB configuration, collects signals, evaluates the OS,
/// writes the result to scratch registers, and triggers a watchdog reset.
#[embassy_executor::task]
async fn probe_eval_task(_writer: HidWriter<'static, Driver<'static, USB>, 8>) {
    use embassy_time::{Duration, Timer};

    // Signal Core 0 that our executor is running
    CORE1_EXECUTOR_READY.store(true, Ordering::Release);
    cortex_m::asm::sev();

    // Wait for USB to be configured (host completed enumeration).
    // Timeout after 2s — if USB never configures (no host), commit Unknown.
    let configured = {
        let deadline = embassy_time::Instant::now() + Duration::from_secs(2);
        loop {
            if PROBE_CONFIGURED.load(Ordering::Relaxed) {
                break true;
            }
            if embassy_time::Instant::now() >= deadline {
                break false;
            }
            Timer::after(Duration::from_millis(10)).await;
        }
    };

    if !configured {
        warn!("[probe] USB not configured within 2s, committing Unknown");
        scratch::write_probe_result(DetectedOs::Unknown);
        crate::system_reset();
    }

    // Check for definite signals every 50ms for 500ms
    for _ in 0..10 {
        Timer::after(Duration::from_millis(50)).await;

        // Windows: String 0xEE is a definite signal — commit immediately
        if PROBE_STRING_0X_EE.load(Ordering::Relaxed) {
            let os = evaluate_os();
            info!("[probe] Definite signal -> {}", os);
            scratch::write_probe_result(os);
            crate::system_reset();
        }
    }

    // At T+500ms: evaluate heuristic signals
    let os = evaluate_os();
    if os != DetectedOs::Unknown {
        info!("[probe] Heuristic at 500ms -> {}", os);
        scratch::write_probe_result(os);
        crate::system_reset();
    }

    // Wait until T+1000ms hard timeout
    Timer::after(Duration::from_millis(500)).await;
    let os = evaluate_os();
    info!("[probe] Timeout at 1000ms -> {}", os);
    scratch::write_probe_result(os);
    crate::system_reset();
}

// ============ Phase 0 Entry Point ============

/// Build and run the Phase 0 probe USB device on Core 1.
///
/// This is the probe-phase equivalent of `usb::start_core1_usb()`. It builds
/// a minimal single-interface vendor-HID device, starts the fingerprinting
/// evaluation task, and never returns (the eval task triggers a watchdog reset).
///
/// Uses its own set of StaticCells — safe because Phase 0 and Phase 1 are
/// separate boot cycles (watchdog reset zeroes RAM between them).
pub fn start_core1_usb_probe(usb: Peri<'static, USB>, attempt_count: u8) -> ! {
    // Enable TIMER_IRQ_0 on Core 1 (needed for embassy_time)
    unsafe {
        embassy_rp::interrupt::InterruptExt::set_priority(
            embassy_rp::interrupt::TIMER_IRQ_0,
            embassy_rp::interrupt::Priority::P3,
        );
        embassy_rp::interrupt::InterruptExt::enable(embassy_rp::interrupt::TIMER_IRQ_0);
    }

    let driver = Driver::new(usb, Irqs);

    // Vary bcdDevice by attempt count to bust Windows 0xEE registry cache.
    // Windows caches the MS OS String Descriptor result per VID/PID/bcdDevice.
    let device_release = 0x0100 | (attempt_count as u16 & 0x0F);

    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("bt2usb");
    config.product = Some("BLE HID Bridge");
    config.serial_number = Some("PROBE");
    config.device_release = device_release;
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.bcd_usb = embassy_usb::UsbVersion::Two;
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    // Probe-phase StaticCells (smaller than Phase 1)
    static PROBE_CONFIG_DESC: StaticCell<[u8; 128]> = StaticCell::new();
    static PROBE_BOS_DESC: StaticCell<[u8; 64]> = StaticCell::new();
    static PROBE_MS_DESC: StaticCell<[u8; 64]> = StaticCell::new();
    static PROBE_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static PROBE_STATE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static PROBE_DEV_HANDLER: StaticCell<ProbeDeviceHandler> = StaticCell::new();
    static PROBE_REQ_HANDLER: StaticCell<ProbeRequestHandler> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        PROBE_CONFIG_DESC.init([0; 128]),
        PROBE_BOS_DESC.init([0; 64]),
        PROBE_MS_DESC.init([0; 64]),
        PROBE_CONTROL_BUF.init([0; 64]),
    );
    builder.handler(PROBE_DEV_HANDLER.init(ProbeDeviceHandler));

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: PROBE_REPORT_DESC,
        request_handler: Some(PROBE_REQ_HANDLER.init(ProbeRequestHandler)),
        poll_ms: 255, // Slow poll — we never send reports
        max_packet_size: 8,
    };
    let writer = HidWriter::<_, 8>::new(
        &mut builder,
        PROBE_STATE.init(embassy_usb::class::hid::State::new()),
        hid_config,
    );

    let usb_dev = builder.build();
    info!(
        "[probe] Phase 0 probe USB device initialized (attempt={})",
        attempt_count
    );

    // Reuse the shared EXECUTOR1 — safe because only one phase runs per boot
    // cycle. Watchdog reset zeroes RAM, making the StaticCell fresh again.
    let executor1 = EXECUTOR1.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(probe_usb_task(usb_dev)).unwrap();
        spawner.spawn(probe_eval_task(writer)).unwrap();
    });
}
