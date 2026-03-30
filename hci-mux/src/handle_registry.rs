//! Connection handle registry for tracking which stack owns each ACL connection.
//!
//! When an ACL connection is established, it is registered as either LE or Classic.
//! Shared events (DisconnectionComplete, EncryptionChange, etc.) and ACL data
//! are routed to the owning stack based on this registry.

use bt_hci::param::ConnHandle;

/// Identifies which stack a connection belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StackId {
    Le,
    Classic,
}

/// Entry in the handle registry.
#[derive(Debug, Clone, Copy)]
enum Entry {
    Free,
    Used { handle: ConnHandle, stack: StackId },
}

/// Fixed-size registry mapping connection handles to their owning stack.
///
/// The `N` parameter sets the maximum number of concurrent connections tracked.
/// Typically 4-8 is sufficient (3 LE + 1 Classic for bt2usb).
pub struct HandleRegistry<const N: usize> {
    entries: [Entry; N],
}

impl<const N: usize> HandleRegistry<N> {
    /// Create a new empty registry.
    pub const fn new() -> Self {
        Self {
            entries: [Entry::Free; N],
        }
    }

    /// Register a connection handle for a stack.
    ///
    /// Returns `true` if successfully registered, `false` if the registry is full.
    pub fn register(&mut self, handle: ConnHandle, stack: StackId) -> bool {
        // Check if already registered (update stack if so)
        for entry in self.entries.iter_mut() {
            if let Entry::Used { handle: h, stack: s } = entry {
                if h.raw() == handle.raw() {
                    *s = stack;
                    return true;
                }
            }
        }
        // Find a free slot
        for entry in self.entries.iter_mut() {
            if matches!(entry, Entry::Free) {
                *entry = Entry::Used { handle, stack };
                return true;
            }
        }
        false
    }

    /// Unregister a connection handle.
    pub fn unregister(&mut self, handle: ConnHandle) {
        for entry in self.entries.iter_mut() {
            if let Entry::Used { handle: h, .. } = entry {
                if h.raw() == handle.raw() {
                    *entry = Entry::Free;
                    return;
                }
            }
        }
    }

    /// Look up which stack owns a connection handle.
    pub fn lookup(&self, handle: ConnHandle) -> Option<StackId> {
        for entry in &self.entries {
            if let Entry::Used { handle: h, stack } = entry {
                if h.raw() == handle.raw() {
                    return Some(*stack);
                }
            }
        }
        None
    }
}
