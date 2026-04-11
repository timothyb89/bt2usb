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
mod defmt_usb;
mod device_profile;
mod framing;
mod hid_report_map;
mod interp;
mod mt2;
mod mt2_translate;
mod preferences;
mod probe;
mod protocol;
mod ptp;
mod ptp_translate;
mod rpc;
mod rpc_log;
pub mod scratch;
mod usb;
mod usb_hid;

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::*;
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
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
                  embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

/// Flash size for RP2040 (2MB).
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

// ============ System Reset & Watchdog ============

const WATCHDOG_BASE: u32 = 0x4005_8000;
const PSM_BASE: u32 = 0x4001_0000;

/// Watchdog timeout for the persistent watchdog: 8 seconds (in µs).
/// Must exceed the longest gap between `feed_watchdog()` calls.
/// The LED task blinks every ~2 s, giving comfortable 4× headroom.
const WATCHDOG_TIMEOUT_US: u32 = 8_000_000;

/// Trigger a full system reset via the RP2040 watchdog.
///
/// Safe to call from either core — the watchdog reset is chip-wide and
/// resets both cores, all peripherals, and all RAM (except scratch registers).
/// This function does not return.
pub fn system_reset() -> ! {
    unsafe {
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

/// Configure the persistent watchdog with `WATCHDOG_TIMEOUT_US` timeout.
///
/// Call once after CYW43 init is complete (to avoid triggering during firmware
/// download). The LED task on Core 0 keeps the watchdog alive by calling
/// `feed_watchdog()` every blink cycle (~2 s).
///
/// If neither core feeds the watchdog within the timeout (e.g. deadlock or a
/// panic that fires while the defmt spinlock is held), the chip resets to Phase 1
/// via the preserved scratch registers.
pub fn configure_watchdog() {
    unsafe {
        // Reset all subsystems on watchdog expiry (same scope as system_reset)
        core::ptr::write_volatile((PSM_BASE + 0x08) as *mut u32, 0xFFFFFFFF);

        // Load initial countdown
        core::ptr::write_volatile((WATCHDOG_BASE + 0x04) as *mut u32, WATCHDOG_TIMEOUT_US);

        // Enable watchdog; pause while either core is being debugged so a
        // breakpoint session doesn't trigger a spurious reset.
        const CTRL_ENABLE: u32 = 1 << 30;
        const CTRL_PAUSE_DBG0: u32 = 1 << 27;
        const CTRL_PAUSE_DBG1: u32 = 1 << 28;
        core::ptr::write_volatile(
            WATCHDOG_BASE as *mut u32,
            CTRL_ENABLE | CTRL_PAUSE_DBG0 | CTRL_PAUSE_DBG1,
        );
    }
    info!(
        "Watchdog configured ({}s timeout)",
        WATCHDOG_TIMEOUT_US / 1_000_000
    );
}

/// Feed the persistent watchdog to reset the countdown.
///
/// Must be called at least once every `WATCHDOG_TIMEOUT_US` microseconds.
/// Currently called from `led_task` on Core 0 each blink cycle (~2 s).
#[inline]
pub fn feed_watchdog() {
    unsafe {
        core::ptr::write_volatile((WATCHDOG_BASE + 0x04) as *mut u32, WATCHDOG_TIMEOUT_US);
    }
}

/// HardFault exception handler — resets the device instead of hanging.
///
/// On Cortex-M0+ without a debug probe, `panic_probe` ends with a loop of
/// `bkpt` instructions. Each `bkpt` triggers a HardFault, and the default
/// cortex-m-rt handler is itself an infinite loop, so any panic causes a
/// permanent hang. Overriding HardFault with `system_reset()` ensures the
/// device recovers from both panics and genuine CPU faults (invalid instructions,
/// unaligned accesses). The watchdog-based reset preserves scratch registers so
/// Phase 1 resumes immediately without going through the OS probe again.
///
/// This vector is in the shared table so it covers both cores.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    system_reset()
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

// ============ Build-time OS override ============

/// Returns a compile-time forced OS if a `force-*` feature is enabled.
/// Used to bypass Phase 0 probing (e.g. for VM testing).
fn build_time_forced_os() -> Option<scratch::DetectedOs> {
    #[cfg(feature = "force-macos")]
    return Some(scratch::DetectedOs::MacOs);
    #[cfg(feature = "force-windows")]
    return Some(scratch::DetectedOs::Windows);
    #[cfg(feature = "force-linux")]
    return Some(scratch::DetectedOs::Linux);
    #[cfg(not(any(
        feature = "force-macos",
        feature = "force-windows",
        feature = "force-linux"
    )))]
    None
}

// ============ Entry Point ============

/// Main entry point — runs on Core 0.
///
/// Two-phase boot:
/// - Phase 0 (Probe): Spawn minimal USB device on Core 1 to fingerprint host OS.
///   Core 0 idles. Probe writes result to scratch registers and resets.
/// - Phase 1 (Configured): Spawn OS-appropriate USB on Core 1, then run BLE on Core 0.
///
/// Build-time override: `--features force-macos` (or force-windows/force-linux)
/// skips Phase 0 entirely and goes straight to Phase 1 with the specified OS.
#[cortex_m_rt::entry]
fn main() -> ! {
    info!("bt2usb starting (dual-core mode)...");

    let p = embassy_rp::init(Default::default());

    // Read boot state from watchdog scratch registers.
    // Cold boot (power-on) zeroes scratch → magic invalid → Phase 0.
    // Watchdog reset from Phase 0 preserves scratch → Phase 1.
    let boot_state = scratch::read_boot_state();
    info!(
        "Boot state: phase={}, os={}, attempts={}",
        boot_state.phase, boot_state.detected_os, boot_state.attempt_count
    );

    // Determine the detected OS: build-time override > runtime probe/scratch.
    // Build-time override skips Phase 0 entirely (useful for VM testing
    // where USB hotplugging breaks the probe).
    let detected_os = if let Some(os) = build_time_forced_os() {
        // On cold boot (Phase 0), write the result and do a system reset
        // just like the normal probe flow. This gives the USB host (especially
        // VMs) a clean "device appeared" event that it can properly enumerate.
        if boot_state.phase == scratch::BootPhase::Probe {
            info!(
                "[core0] Build-time forced OS={}, writing scratch + reset",
                os
            );
            scratch::write_probe_result(os);
            system_reset();
        }
        info!("[core0] Build-time forced OS={}", os);
        os
    } else {
        match boot_state.phase {
            scratch::BootPhase::Probe => {
                // Check for a forced OS override cached in scratch[4] by Phase 1.
                // If set, skip the probe and go straight to Phase 1 with that OS.
                if let Some(forced_os) = scratch::read_forced_os() {
                    info!(
                        "[core0] Phase 0: forced OS override -> {}, skipping probe",
                        forced_os
                    );
                    scratch::write_probe_result(forced_os);
                    system_reset();
                }

                // Phase 0: fingerprint the host OS via a minimal probe USB device.
                let attempts = scratch::increment_attempts();
                if scratch::max_attempts_exceeded(attempts) {
                    info!("Max probe attempts exceeded, skipping probe -> safe default");
                    scratch::write_probe_result(scratch::DetectedOs::Unknown);
                    system_reset();
                }

                info!("[core0] Phase 0: USB probe for OS fingerprinting");
                let usb = p.USB;
                spawn_core1(
                    p.CORE1,
                    unsafe { &mut *addr_of_mut!(CORE1_STACK) },
                    move || {
                        probe::start_core1_usb_probe(usb, attempts);
                    },
                );

                // Wait for Core 1 executor to start, then idle.
                // The probe task will write scratch and trigger watchdog reset.
                while !CORE1_EXECUTOR_READY.load(Ordering::Acquire) {
                    cortex_m::asm::wfe();
                }
                info!("[core0] Phase 0: Core 1 probe running, waiting for reset...");
                loop {
                    cortex_m::asm::wfe();
                }
            }
            scratch::BootPhase::Configured => {
                // Phase 1: forced OS override (from `set-os` command) takes precedence.
                let os = scratch::read_forced_os().unwrap_or(boot_state.detected_os);
                info!("[core0] Phase 1: configured mode, OS={}", os);
                os
            }
        }
    };

    // Phase 1: present the OS-appropriate USB configuration and run BLE.
    let flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH1, Irqs);

    let usb = p.USB;
    spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            usb::start_core1_usb(usb, detected_os);
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
