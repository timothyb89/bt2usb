//! bt2usb — BLE HID to USB HID Bridge (Dual-Core)
//!
//! This firmware runs on a Raspberry Pi Pico W and:
//! 1. Acts as a BLE Central, connecting to HID peripherals (keyboards, mice)
//! 2. Translates BLE HID reports to USB HID reports
//! 3. Presents itself as a USB HID device to the host computer
//!
//! ## Dual-core architecture
//!
//! - **Core 0**: CYW43 WiFi/BLE chip + BLE central state machine + flash bond storage
//! - **Core 1**: USB device tasks (keyboard, mouse, RPC)
//!
//! CYW43 runs on Core 0 (where `embassy_rp::init()` sets up DMA and timer interrupts).
//! USB runs on Core 1 (USB peripheral handles NAKs gracefully during brief flash pauses).
//! Flash erase/write pauses Core 1 via FIFO, which is tolerable for USB but would
//! corrupt BLE timing-sensitive operations if BLE were on the paused core.
//!
//! ## Module structure
//!
//! - [`ble`] — BLE central logic: CYW43 init, connection, GATT discovery, HID loop
//! - [`usb`] — Core 1 USB device setup and HID report forwarding
//! - [`bonding`] — Flash-based BLE bond storage
//! - [`preferences`] — Flash-based user preferences (active device, axis multipliers)
//! - [`device_profile`] — Device-specific HID report translation
//! - [`protocol`] — RPC wire format (COBS-framed CBOR)
//! - [`rpc`] — RPC handler over USB HID vendor interface
//! - [`usb_hid`] — USB HID descriptors and report serialization
//! - [`ble_hid`] — BLE HID constants and report parsing
//! - [`ble_state`] — BLE command/event types and inter-task channels
//! - [`framing`] — COBS frame encoding/decoding
//! - [`rpc_log`] — Log forwarding over RPC

#![no_std]
#![no_main]

mod ble;
mod ble_hid;
mod ble_state;
mod bonding;
mod device_profile;
mod framing;
mod preferences;
mod protocol;
mod rpc;
mod rpc_log;
mod usb;
mod usb_hid;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Executor;
use embassy_rp::bind_interrupts;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::multicore::{spawn_core1, Stack as Core1Stack};
use embassy_rp::peripherals::{PIO0, USB};
use panic_probe as _;
use static_cell::StaticCell;

// ============ Interrupts ============

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// Flash size for RP2040 (2MB).
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

// ============ System Reset ============

/// Trigger a full system reset via the RP2040 watchdog.
///
/// Called from Core 0 (BLE task) to ensure proper reset of both cores.
/// This function does not return.
pub fn system_reset() -> ! {
    unsafe {
        const WATCHDOG_BASE: u32 = 0x40058000;
        const PSM_BASE: u32 = 0x40010000;

        // Disable all interrupts globally on this core
        cortex_m::interrupt::disable();

        // Configure PSM to reset everything on watchdog timeout
        core::ptr::write_volatile((PSM_BASE + 0x08) as *mut u32, 0xFFFFFFFF);

        // Trigger watchdog with minimal timeout
        core::ptr::write_volatile((WATCHDOG_BASE + 0x04) as *mut u32, 1);

        // Enable watchdog (bit 30)
        core::ptr::write_volatile(WATCHDOG_BASE as *mut u32, 1 << 30);

        // Spin until watchdog fires
        loop {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
            cortex_m::asm::nop();
        }
    }
}

// ============ Dual-Core Infrastructure ============

/// Core 1 stack (16KB — needs room for P-256 crypto, GATT discovery, CYW43 SPI).
static mut CORE1_STACK: Core1Stack<16384> = Core1Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// Signal from Core 1 -> Core 0: Core 1 executor is running.
/// Core 0 must not start its executor until Core 1's is up,
/// otherwise both cores starting simultaneously causes a stall.
static CORE1_EXECUTOR_READY: AtomicBool = AtomicBool::new(false);

// ============ Entry Point ============

/// Main entry point — runs on Core 0.
///
/// Spawns USB on Core 1, then runs CYW43 + BLE + Flash on Core 0.
#[cortex_m_rt::entry]
fn main() -> ! {
    info!("bt2usb starting (dual-core mode)...");

    let p = embassy_rp::init(Default::default());

    // Flash storage (used on Core 0 for bond and preference storage)
    let flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH1);

    // Spawn Core 1 (USB)
    let usb = p.USB;

    spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            usb::start_core1_usb(usb);
        },
    );
    info!("[core0] Core 1 spawned, waiting for executor ready...");

    // Wait for Core 1's executor to be fully set up before starting Core 0's.
    // Without this, both executors starting simultaneously causes a stall.
    while !CORE1_EXECUTOR_READY.load(Ordering::Acquire) {
        cortex_m::asm::wfe();
    }
    info!("[core0] Core 1 executor ready");

    // Start Core 0 executor (CYW43 + BLE + Flash)
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner
            .spawn(ble::core0_ble_main(
                spawner, p.PIO0, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.DMA_CH0, flash,
            ))
            .unwrap();
    });
}
