//! Core 1 USB device setup and tasks
//!
//! This module handles all USB functionality running on Core 1:
//! - Building the USB device with keyboard, mouse, and vendor HID interfaces
//! - Spawning USB tasks (device driver, HID report handler, RPC)
//! - Receiving BLE HID reports and forwarding them as USB HID reports
//!
//! Core 1 runs independently from Core 0 (CYW43 + BLE). Flash erase/write on
//! Core 0 briefly pauses Core 1 via FIFO, but USB handles NAKs gracefully.

use core::sync::atomic::Ordering;
use defmt::*;
use embassy_executor::Executor;
use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_time;
use embassy_usb::class::hid::{HidReaderWriter, HidWriter};
use static_cell::StaticCell;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use crate::ble_hid::{HidReportType, HID_REPORT_CHANNEL, BATTERY_USB_SIGNAL};
use crate::usb_hid::{
    serialize_keyboard_report, serialize_mouse_report, serialize_mouse_report_16bit,
    KeyboardHidReport, MOUSE_HIRES_16BIT_REPORT_DESC, VENDOR_RPC_REPORT_DESC,
};
use crate::{rpc, Irqs, CORE1_EXECUTOR_READY, EXECUTOR1};

// ============ USB Tasks ============

/// USB device task — handles USB enumeration and events (spawned on Core 1).
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}

/// USB HID handler task — receives HID reports from BLE (Core 0) and sends to USB.
///
/// Handles two event sources concurrently via `select`:
/// - HID_REPORT_CHANNEL: keyboard and mouse reports from BLE
/// - BATTERY_USB_SIGNAL: battery level updates from the BLE battery poller
///
/// Mouse reports use Report ID 1 (prepend 0x01). Battery updates use Report ID 2
/// ([0x02, level]) on the same mouse interface, triggering Linux's hid-battery
/// module to update /sys/class/power_supply/hid-*/capacity.
#[embassy_executor::task]
async fn usb_hid_handler_task(
    mut keyboard_writer: HidWriter<'static, Driver<'static, USB>, 8>,
    mut mouse_writer: HidWriter<'static, Driver<'static, USB>, 8>,
) {
    info!("USB HID handler task started, waiting for BLE reports...");

    // Send initial battery level if already known (e.g. reconnection scenario)
    let initial_level = crate::ble_hid::BATTERY_LEVEL.load(Ordering::Relaxed);
    if initial_level != 0xFF {
        let _ = embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            mouse_writer.write(&[0x02, initial_level]),
        )
        .await;
    }

    loop {
        match select(HID_REPORT_CHANNEL.receive(), BATTERY_USB_SIGNAL.wait()).await {
            Either::First(event) => {
                debug!(
                    "Received HID report: type={:?}, len={}",
                    event.report_type, event.len
                );
                match event.report_type {
                    HidReportType::Keyboard => {
                        if event.len >= 8 {
                            let report = KeyboardHidReport {
                                modifier: event.data[0],
                                reserved: event.data[1],
                                leds: 0,
                                keycodes: [
                                    event.data[2],
                                    event.data[3],
                                    event.data[4],
                                    event.data[5],
                                    event.data[6],
                                    event.data[7],
                                ],
                            };
                            let buf = serialize_keyboard_report(&report);
                            if let Err(e) = keyboard_writer.write(&buf).await {
                                warn!("Keyboard write error: {:?}", e);
                            }
                        }
                    }
                    HidReportType::Mouse => {
                        if event.len >= 1 {
                            if event.profile.uses_16bit_reports() {
                                let report = event
                                    .profile
                                    .translate_mouse_report_16bit(&event.data, event.len);
                                let data = serialize_mouse_report_16bit(&report);
                                // Report ID 1 prefix for mouse reports
                                let mut buf = [0u8; 8];
                                buf[0] = 0x01;
                                buf[1..].copy_from_slice(&data);
                                if let Err(e) = mouse_writer.write(&buf).await {
                                    warn!("Mouse write error (16-bit): {:?}", e);
                                }
                            } else {
                                let report =
                                    event.profile.translate_mouse_report(&event.data, event.len);
                                let data = serialize_mouse_report(&report);
                                // Report ID 1 prefix for mouse reports
                                let mut buf = [0u8; 6];
                                buf[0] = 0x01;
                                buf[1..].copy_from_slice(&data);
                                if let Err(e) = mouse_writer.write(&buf).await {
                                    warn!("Mouse write error (8-bit): {:?}", e);
                                }
                            }
                        }
                    }
                    _ => {
                        debug!("Unhandled HID report type");
                    }
                }
            }
            Either::Second(level) => {
                // Battery level changed: send Report ID 2 on the mouse interface.
                // Linux's hid-battery picks this up and updates power_supply capacity.
                let _ = embassy_time::with_timeout(
                    embassy_time::Duration::from_secs(5),
                    mouse_writer.write(&[0x02, level]),
                )
                .await;
            }
        }
    }
}

// ============ Core 1 USB Initialization ============

/// Build and start all USB tasks on Core 1.
///
/// This runs inside the `spawn_core1` closure and must not return. It:
/// 1. Enables TIMER_IRQ_0 on Core 1 (needed for embassy_time, e.g. RPC timeouts)
/// 2. Builds the USB device with keyboard, mouse, and vendor HID interfaces
/// 3. Starts the Core 1 executor with all USB tasks
/// 4. Signals Core 0 that the executor is ready via `CORE1_EXECUTOR_READY`
pub fn start_core1_usb(usb: Peri<'static, USB>) -> ! {
    // Enable TIMER_IRQ_0 on Core 1's NVIC so embassy_time works
    // (e.g. RPC timeouts). embassy_rp::init() only enables it on Core 0.
    // The timer driver uses hardware spinlocks, safe for dual-core.
    unsafe {
        embassy_rp::interrupt::InterruptExt::set_priority(
            embassy_rp::interrupt::TIMER_IRQ_0,
            embassy_rp::interrupt::Priority::P3,
        );
        embassy_rp::interrupt::InterruptExt::enable(embassy_rp::interrupt::TIMER_IRQ_0);
    }

    // Build USB device on Core 1 — this ensures USBCTRL_IRQ is enabled
    // on Core 1's NVIC (where we want USB interrupts to fire).
    let driver = Driver::new(usb, Irqs);
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("momentary");
    config.product = Some("BT2USB Bridge");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Boot Keyboard
    let kb_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };

    // Mouse (always use 16-bit descriptor for maximum capability)
    // Profile settings control how reports are scaled/interpreted
    static MOUSE_HANDLER: StaticCell<crate::usb_hid::HiresMouseRequestHandler> = StaticCell::new();
    let mouse_config = embassy_usb::class::hid::Config {
        report_descriptor: MOUSE_HIRES_16BIT_REPORT_DESC,
        request_handler: Some(MOUSE_HANDLER.init(crate::usb_hid::HiresMouseRequestHandler)),
        poll_ms: 1,
        max_packet_size: 64,
    };

    // State buffers
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    static STATE_KB: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_MOUSE: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();
    static STATE_RPC: StaticCell<embassy_usb::class::hid::State> = StaticCell::new();

    static DEVICE_HANDLER: StaticCell<crate::usb_hid::UsbDeviceHandler> = StaticCell::new();

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );
    builder.handler(DEVICE_HANDLER.init(crate::usb_hid::UsbDeviceHandler));

    let kb_writer = HidWriter::<_, 8>::new(
        &mut builder,
        STATE_KB.init(embassy_usb::class::hid::State::new()),
        kb_config,
    );

    // Mouse uses Report ID 1 for movement, Report ID 2 for battery level.
    // Battery Level (Battery System page 0x85) embedded here so Linux's
    // hid-battery module picks it up from the mouse input device.
    let mouse_writer = HidWriter::<_, 8>::new(
        &mut builder,
        STATE_MOUSE.init(embassy_usb::class::hid::State::new()),
        mouse_config,
    );

    // Vendor HID interface for RPC communication
    let rpc_config = embassy_usb::class::hid::Config {
        report_descriptor: VENDOR_RPC_REPORT_DESC,
        request_handler: None,
        poll_ms: 10,
        max_packet_size: 64,
    };
    let rpc_hid = HidReaderWriter::<_, 64, 64>::new(
        &mut builder,
        STATE_RPC.init(embassy_usb::class::hid::State::new()),
        rpc_config,
    );
    let (rpc_reader, rpc_writer) = rpc_hid.split();

    let usb_dev = builder.build();
    info!("[core1] USB HID device initialized");

    let executor1 = EXECUTOR1.init(Executor::new());
    executor1.run(|spawner| {
        spawner.spawn(usb_task(usb_dev)).unwrap();
        spawner
            .spawn(usb_hid_handler_task(kb_writer, mouse_writer))
            .unwrap();
        spawner
            .spawn(rpc::rpc_task(rpc_writer, rpc_reader))
            .unwrap();
        // Signal Core 0: our executor is set up and tasks are spawned
        CORE1_EXECUTOR_READY.store(true, Ordering::Release);
        cortex_m::asm::sev(); // Wake Core 0 from WFE
    });
}
