//! Watchdog scratch register abstraction for two-phase boot.
//!
//! RP2040 watchdog has 8 × 32-bit scratch registers that survive soft/watchdog
//! reset but clear on cold boot (power-on). We use four of them to pass state
//! between Phase 0 (OS probe) and Phase 1 (configured operation).
//!
//! Layout:
//!   scratch[0] = detected OS (0=Unknown, 1=Windows, 2=Linux, 3=macOS)
//!   scratch[1] = boot phase  (0=Probe, 1=Configured)
//!   scratch[2] = probe attempt counter
//!   scratch[3] = magic sentinel (validates that values are ours)

use defmt::*;

const SCRATCH_OS: usize = 0;
const SCRATCH_PHASE: usize = 1;
const SCRATCH_ATTEMPTS: usize = 2;
const SCRATCH_MAGIC: usize = 3;

const MAGIC_VALUE: u32 = 0xB72B_0001;
const MAX_PROBE_ATTEMPTS: u8 = 3;

/// RP2040 WATCHDOG base address.
const WATCHDOG_BASE: u32 = 0x4005_8000;
/// Offset of SCRATCH0 register (SCRATCH0..SCRATCH7 are consecutive u32s).
const SCRATCH0_OFFSET: u32 = 0x0C;

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum DetectedOs {
    Unknown = 0,
    Windows = 1,
    Linux = 2,
    MacOs = 3,
}

impl DetectedOs {
    fn from_u32(val: u32) -> Self {
        match val {
            1 => Self::Windows,
            2 => Self::Linux,
            3 => Self::MacOs,
            _ => Self::Unknown,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum BootPhase {
    Probe = 0,
    Configured = 1,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct BootState {
    pub phase: BootPhase,
    pub detected_os: DetectedOs,
    pub attempt_count: u8,
}

fn scratch_read(index: usize) -> u32 {
    let addr = (WATCHDOG_BASE + SCRATCH0_OFFSET + (index as u32) * 4) as *const u32;
    unsafe { core::ptr::read_volatile(addr) }
}

fn scratch_write(index: usize, value: u32) {
    let addr = (WATCHDOG_BASE + SCRATCH0_OFFSET + (index as u32) * 4) as *mut u32;
    unsafe { core::ptr::write_volatile(addr, value) };
}

fn scratch_valid() -> bool {
    scratch_read(SCRATCH_MAGIC) == MAGIC_VALUE
}

/// Read the boot state from scratch registers.
///
/// If the magic sentinel is invalid (cold boot or corrupted), returns
/// defaults: Phase=Probe, OS=Unknown, attempts=0.
pub fn read_boot_state() -> BootState {
    if !scratch_valid() {
        return BootState {
            phase: BootPhase::Probe,
            detected_os: DetectedOs::Unknown,
            attempt_count: 0,
        };
    }

    let phase = if scratch_read(SCRATCH_PHASE) == 1 {
        BootPhase::Configured
    } else {
        BootPhase::Probe
    };

    BootState {
        phase,
        detected_os: DetectedOs::from_u32(scratch_read(SCRATCH_OS)),
        attempt_count: scratch_read(SCRATCH_ATTEMPTS) as u8,
    }
}

/// Write the probe result and transition to Phase 1 (Configured).
///
/// Writes OS, phase, and magic sentinel. The magic is written last to
/// ensure atomicity — a partial write won't pass validation.
pub fn write_probe_result(os: DetectedOs) {
    scratch_write(SCRATCH_OS, os as u32);
    scratch_write(SCRATCH_PHASE, BootPhase::Configured as u32);
    // Magic last — validates all other fields
    scratch_write(SCRATCH_MAGIC, MAGIC_VALUE);
    info!("Scratch: wrote probe result os={}, phase=Configured", os);
}

/// Increment the probe attempt counter. Returns the new count.
///
/// Also writes the magic sentinel if not already valid (first Phase 0 entry
/// after cold boot needs the sentinel for the counter to persist).
pub fn increment_attempts() -> u8 {
    let current = if scratch_valid() {
        scratch_read(SCRATCH_ATTEMPTS) as u8
    } else {
        0
    };
    let next = current.saturating_add(1);
    scratch_write(SCRATCH_ATTEMPTS, next as u32);
    // Ensure magic is set so the counter survives the reset
    if !scratch_valid() {
        scratch_write(SCRATCH_MAGIC, MAGIC_VALUE);
    }
    info!("Scratch: probe attempt {} -> {}", current, next);
    next
}

/// Check if the probe attempt counter has exceeded the maximum.
pub fn max_attempts_exceeded(count: u8) -> bool {
    count > MAX_PROBE_ATTEMPTS
}

/// Invalidate the scratch sentinel so the next boot enters Phase 0 (probe).
pub fn clear_for_reprobe() {
    scratch_write(SCRATCH_MAGIC, 0);
    info!("Scratch: cleared for reprobe");
}
