//! Liveness-tracked watchdog feeder.
//!
//! Several "slots" represent task health. Each watched task pets its slot
//! at a natural point in its loop. A dedicated feeder task wakes once per
//! second, checks every slot's age against its deadline, and only feeds the
//! hardware watchdog if all slots are fresh. If any slot is stale the feeder
//! stops feeding, and the hardware watchdog (configured in `main`) fires
//! a few seconds later — picking up the stall as a real reset condition.
//!
//! Slots cover:
//! - `SLOT_CORE0_TICK` / `SLOT_CORE1_TICK` — per-core ticker tasks. Catch
//!   executor wedges or starvation on either core.
//! - `SLOT_BLE_MANAGER` — the BLE connection manager loop iteration. Catches
//!   the manager getting stuck (e.g. blocked on a never-completing future).
//!   Deadline is generous to accommodate long connect attempts.

use core::sync::atomic::Ordering;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::AtomicU64;

pub const SLOT_CORE0_TICK: usize = 0;
pub const SLOT_CORE1_TICK: usize = 1;
pub const SLOT_BLE_MANAGER: usize = 2;
const NUM_SLOTS: usize = 3;

struct Slot {
    last_ms: AtomicU64,
    deadline_ms: u64,
    name: &'static str,
}

static SLOTS: [Slot; NUM_SLOTS] = [
    Slot {
        last_ms: AtomicU64::new(0),
        deadline_ms: 4_000,
        name: "core0",
    },
    Slot {
        last_ms: AtomicU64::new(0),
        deadline_ms: 4_000,
        name: "core1",
    },
    Slot {
        last_ms: AtomicU64::new(0),
        deadline_ms: 60_000,
        name: "ble_manager",
    },
];

/// Mark a slot as alive at the current time.
pub fn pet(slot: usize) {
    SLOTS[slot]
        .last_ms
        .store(Instant::now().as_millis(), Ordering::Relaxed);
}

/// Initialize all slot timestamps to "now". Call once after the time driver
/// is up and before spawning the feeder, so newly-spawned tasks have grace
/// time to reach their first pet.
pub fn init() {
    let now = Instant::now().as_millis();
    for s in &SLOTS {
        s.last_ms.store(now, Ordering::Relaxed);
    }
}

/// Feeder task — runs on Core 0. Pets the hardware watchdog only if every
/// slot is within its deadline. A stale slot causes the HW watchdog to fire.
#[embassy_executor::task]
pub async fn feeder_task() -> ! {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        let now = Instant::now().as_millis();
        let mut all_alive = true;
        for s in &SLOTS {
            let last = s.last_ms.load(Ordering::Relaxed);
            let age = now.saturating_sub(last);
            if age > s.deadline_ms {
                defmt::error!("[wdt] slot '{}' stalled ({} ms)", s.name, age);
                all_alive = false;
            }
        }
        if all_alive {
            crate::feed_watchdog();
        }
    }
}

/// Core 0 liveness ticker.
#[embassy_executor::task]
pub async fn core0_tick_task() -> ! {
    loop {
        pet(SLOT_CORE0_TICK);
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Core 1 liveness ticker.
#[embassy_executor::task]
pub async fn core1_tick_task() -> ! {
    loop {
        pet(SLOT_CORE1_TICK);
        Timer::after(Duration::from_millis(500)).await;
    }
}
