//! Command slot management for HCI command/response matching.
//!
//! Replicates the `ControllerState` pattern from `bt_hci::controller::ExternalController`,
//! but as a standalone public type that can be shared between the mux runner and virtual controllers.

use core::cell::RefCell;
use core::future::poll_fn;
use core::task::Poll;

use bt_hci::cmd::{self, controller_baseband::Reset, Cmd};
use bt_hci::param::Status;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use futures_intrusive::sync::LocalSemaphore;

/// Response from a completed HCI command.
#[derive(Clone, Copy)]
pub struct CommandResponse {
    pub status: Status,
    pub len: usize,
}

/// State of a single command slot.
enum Slot {
    Empty,
    Pending {
        opcode: u16,
        /// Pointer to the caller's return-value buffer.
        event: *mut [u8],
    },
}

// Safety: We only access slots from a single executor (embedded, single-core for each stack).
unsafe impl Send for Slot {}

/// Command slot manager. Tracks pending HCI commands and signals their completion.
///
/// `N` is the maximum number of concurrent pending commands.
pub struct CommandSlots<const N: usize> {
    permits: LocalSemaphore,
    slots: RefCell<[Slot; N]>,
    signals: [Signal<NoopRawMutex, CommandResponse>; N],
    waker: AtomicWaker,
}

// Safety: On embedded (single-threaded executor), this is safe.
// The RefCell and LocalSemaphore are only accessed from one executor.
unsafe impl<const N: usize> Sync for CommandSlots<N> {}

impl<const N: usize> CommandSlots<N> {
    const EMPTY_SLOT: Slot = Slot::Empty;
    #[allow(clippy::declare_interior_mutable_const)]
    const EMPTY_SIGNAL: Signal<NoopRawMutex, CommandResponse> = Signal::new();

    /// Create a new command slot manager.
    pub fn new() -> Self {
        Self {
            permits: LocalSemaphore::new(true, 1),
            slots: RefCell::new([const { Self::EMPTY_SLOT }; N]),
            signals: [const { Self::EMPTY_SIGNAL }; N],
            waker: AtomicWaker::new(),
        }
    }

    /// Complete a pending command with the given response.
    ///
    /// Called by the mux runner when a CommandComplete or CommandStatus event arrives.
    pub fn complete(&self, opcode: cmd::Opcode, status: Status, num_hci_cmd_pkts: usize, data: &[u8]) {
        let mut slots = self.slots.borrow_mut();
        for (idx, slot) in slots.iter_mut().enumerate() {
            match slot {
                Slot::Pending {
                    opcode: op,
                    event,
                } if *op == opcode.to_raw() => {
                    if !data.is_empty() {
                        assert!(!event.is_null());
                        // Safety: the slot is pending, so the caller's stack frame is still valid.
                        unsafe { (&mut (**event))[..data.len()].copy_from_slice(data) };
                    }
                    self.signals[idx].signal(CommandResponse {
                        status,
                        len: data.len(),
                    });
                    if opcode != Reset::OPCODE {
                        break;
                    }
                }
                Slot::Pending { .. } if opcode == Reset::OPCODE => {
                    // Reset cancels all pending commands
                    self.signals[idx].signal(CommandResponse {
                        status: Status::CONTROLLER_BUSY,
                        len: 0,
                    });
                }
                _ => {}
            }
        }

        self.permits
            .release(num_hci_cmd_pkts.saturating_sub(self.permits.permits()));
    }

    /// Acquire a command slot for sending a command.
    ///
    /// Returns a signal to wait on and the slot index (for release).
    pub async fn acquire(
        &self,
        opcode: cmd::Opcode,
        event: *mut [u8],
    ) -> (&Signal<NoopRawMutex, CommandResponse>, usize) {
        let to_acquire = if opcode == Reset::OPCODE {
            self.permits.permits()
        } else {
            1
        };
        let mut permit: futures_intrusive::sync::LocalSemaphoreReleaser<'_> =
            self.permits.acquire(to_acquire).await;
        permit.disarm();
        poll_fn(|cx| match self.try_acquire(opcode, event) {
            Some(ret) => Poll::Ready(ret),
            None => {
                self.waker.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }

    /// Release a command slot.
    pub fn release(&self, idx: usize) {
        let mut slots = self.slots.borrow_mut();
        slots[idx] = Slot::Empty;
    }

    fn try_acquire(
        &self,
        opcode: cmd::Opcode,
        event: *mut [u8],
    ) -> Option<(&Signal<NoopRawMutex, CommandResponse>, usize)> {
        let mut slots = self.slots.borrow_mut();
        // Ensure no duplicate opcode
        for slot in slots.iter() {
            if let Slot::Pending { opcode: op, .. } = slot {
                if *op == opcode.to_raw() {
                    return None;
                }
            }
        }
        // Find a free slot
        for (idx, slot) in slots.iter_mut().enumerate() {
            if matches!(slot, Slot::Empty) {
                *slot = Slot::Pending {
                    opcode: opcode.to_raw(),
                    event,
                };
                self.signals[idx].reset();
                return Some((&self.signals[idx], idx));
            }
        }
        None
    }
}
