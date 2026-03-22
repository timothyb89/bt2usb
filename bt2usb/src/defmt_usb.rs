//! Custom defmt global logger: dual output to RTT + USB channel.
//!
//! Replaces `defmt-rtt` with a logger that writes encoded defmt frames to
//! both the RTT ring buffer (for probe-rs) and a channel (for USB forwarding).
//!
//! The RTT implementation is vendored from `defmt-rtt` 1.x to avoid conflicts
//! with the `#[defmt::global_logger]` attribute (only one crate can define it).
//!
//! ## Usage
//!
//! Import this module in `main.rs` with `mod defmt_usb;`. The
//! `#[defmt::global_logger]` attribute links it automatically.

use core::{
    cell::UnsafeCell,
    ptr,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// ── Defmt frame forwarding ──────────────────────────────────────────────────

/// Maximum captured frame size. Typical defmt frames are 20–100 bytes;
/// 256 bytes is generous headroom.
const MAX_FRAME_SIZE: usize = 256;

/// A captured defmt frame ready for USB forwarding.
#[derive(Clone)]
pub struct DefmtFrame {
    pub data: [u8; MAX_FRAME_SIZE],
    pub len: u16,
}

/// Channel drained by `rpc_task` on Core 1. Depth 16 ≈ 4 KB static RAM.
/// Frames are silently dropped if the channel is full (non-blocking send).
pub static DEFMT_CHANNEL: Channel<CriticalSectionRawMutex, DefmtFrame, 16> = Channel::new();

// ── RTT plumbing (vendored from defmt-rtt 1.x) ─────────────────────────────

const RTT_BUF_SIZE: usize = 1024;

/// RTT mode: don't block if buffer is full, trim output.
const MODE_NON_BLOCKING_TRIM: u32 = 1;
/// RTT mode: block until host reads (set by probe-rs).
const MODE_BLOCK_IF_FULL: u32 = 2;
/// Mask for the mode bits in the flags field.
const MODE_MASK: u32 = 0b11;

#[repr(C)]
struct RttChannel {
    name: *const u8,
    buffer: *mut u8,
    size: u32,
    write: AtomicU32,
    read: AtomicU32,
    flags: AtomicU32,
}

impl RttChannel {
    fn write_all(&self, mut bytes: &[u8]) {
        while !bytes.is_empty() {
            let consumed = if self.host_is_connected() {
                self.blocking_write(bytes)
            } else {
                self.nonblocking_write(bytes)
            };
            if consumed != 0 {
                bytes = &bytes[consumed..];
            }
        }
    }

    fn blocking_write(&self, bytes: &[u8]) -> usize {
        if bytes.is_empty() {
            return 0;
        }
        let read = self.read.load(Ordering::Relaxed) as usize;
        let write = self.write.load(Ordering::Acquire) as usize;
        let available = available_buffer_size(read, write);
        if available == 0 {
            return 0;
        }
        self.write_impl(bytes, write, available)
    }

    fn nonblocking_write(&self, bytes: &[u8]) -> usize {
        let write = self.write.load(Ordering::Acquire) as usize;
        self.write_impl(bytes, write, RTT_BUF_SIZE)
    }

    fn write_impl(&self, bytes: &[u8], cursor: usize, available: usize) -> usize {
        let len = bytes.len().min(available);
        unsafe {
            if cursor + len > RTT_BUF_SIZE {
                let pivot = RTT_BUF_SIZE - cursor;
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), pivot);
                ptr::copy_nonoverlapping(bytes.as_ptr().add(pivot), self.buffer, len - pivot);
            } else {
                ptr::copy_nonoverlapping(bytes.as_ptr(), self.buffer.add(cursor), len);
            }
        }
        self.write.store(
            (cursor.wrapping_add(len) % RTT_BUF_SIZE) as u32,
            Ordering::Release,
        );
        len
    }

    fn flush(&self) {
        if !self.host_is_connected() {
            return;
        }
        let read = || self.read.load(Ordering::Relaxed);
        let write = || self.write.load(Ordering::Relaxed);
        while read() != write() {}
    }

    fn host_is_connected(&self) -> bool {
        self.flags.load(Ordering::Relaxed) & MODE_MASK == MODE_BLOCK_IF_FULL
    }
}

fn available_buffer_size(read_cursor: usize, write_cursor: usize) -> usize {
    if read_cursor > write_cursor {
        read_cursor - write_cursor - 1
    } else {
        RTT_BUF_SIZE - write_cursor - 1 + read_cursor
    }
}

#[repr(C)]
struct RttHeader {
    id: [u8; 16],
    max_up_channels: u32,
    max_down_channels: u32,
    up_channel: RttChannel,
}

unsafe impl Sync for RttHeader {}

struct RttBuffer {
    inner: UnsafeCell<[u8; RTT_BUF_SIZE]>,
}

impl RttBuffer {
    const fn new() -> Self {
        Self {
            inner: UnsafeCell::new([0; RTT_BUF_SIZE]),
        }
    }

    const fn get(&self) -> *mut u8 {
        self.inner.get() as _
    }
}

unsafe impl Sync for RttBuffer {}

// RTT control block — discovered by probe-rs via the magic "SEGGER RTT" ID.
#[no_mangle]
static _SEGGER_RTT: RttHeader = RttHeader {
    id: *b"SEGGER RTT\0\0\0\0\0\0",
    max_up_channels: 1,
    max_down_channels: 0,
    up_channel: RttChannel {
        name: RTT_NAME.as_ptr(),
        buffer: RTT_BUFFER.get(),
        size: RTT_BUF_SIZE as u32,
        write: AtomicU32::new(0),
        read: AtomicU32::new(0),
        flags: AtomicU32::new(MODE_NON_BLOCKING_TRIM),
    },
};

#[cfg_attr(not(target_os = "macos"), link_section = ".uninit.defmt-rtt.BUFFER")]
#[cfg_attr(target_os = "macos", link_section = ".uninit,defmt-rtt.BUFFER")]
static RTT_BUFFER: RttBuffer = RttBuffer::new();

#[cfg_attr(not(target_os = "macos"), link_section = ".data.defmt-rtt.NAME")]
#[cfg_attr(target_os = "macos", link_section = ".data,defmt-rtt.NAME")]
static RTT_NAME: [u8; 6] = *b"defmt\0";

// ── Global logger implementation ────────────────────────────────────────────

#[defmt::global_logger]
struct Logger;

/// Encoder state: critical section guard, defmt encoder, and USB capture buffer.
struct EncoderState {
    taken: AtomicBool,
    cs_restore: UnsafeCell<critical_section::RestoreState>,
    encoder: UnsafeCell<defmt::Encoder>,
    /// Capture buffer for the current frame (accumulated during write calls).
    capture_buf: UnsafeCell<[u8; MAX_FRAME_SIZE]>,
    /// Current position in the capture buffer.
    capture_pos: UnsafeCell<usize>,
}

impl EncoderState {
    const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
            encoder: UnsafeCell::new(defmt::Encoder::new()),
            capture_buf: UnsafeCell::new([0u8; MAX_FRAME_SIZE]),
            capture_pos: UnsafeCell::new(0),
        }
    }

    /// Write encoded bytes to both RTT and the capture buffer.
    ///
    /// # Safety
    /// Must only be called between acquire() and release() (inside critical section).
    unsafe fn dual_write(bytes: &[u8]) {
        // Write to RTT for probe-rs
        _SEGGER_RTT.up_channel.write_all(bytes);

        // Write to capture buffer for USB forwarding
        let pos = &mut *ENCODER_STATE.capture_pos.get();
        let buf = &mut *ENCODER_STATE.capture_buf.get();
        let remaining = MAX_FRAME_SIZE - *pos;
        let len = bytes.len().min(remaining);
        if len > 0 {
            buf[*pos..*pos + len].copy_from_slice(&bytes[..len]);
            *pos += len;
        }
        // If frame exceeds MAX_FRAME_SIZE, excess bytes are silently truncated
        // in the capture buffer (RTT still gets everything).
    }
}

unsafe impl Sync for EncoderState {}

static ENCODER_STATE: EncoderState = EncoderState::new();

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // Enter critical section (disables interrupts on Cortex-M)
        let restore = unsafe { critical_section::acquire() };

        if ENCODER_STATE.taken.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }
        ENCODER_STATE.taken.store(true, Ordering::Relaxed);

        unsafe {
            ENCODER_STATE.cs_restore.get().write(restore);

            // Reset capture buffer for the new frame
            *ENCODER_STATE.capture_pos.get() = 0;

            let encoder: &mut defmt::Encoder = &mut *ENCODER_STATE.encoder.get();
            encoder.start_frame(|b| EncoderState::dual_write(b));
        }
    }

    unsafe fn write(bytes: &[u8]) {
        let encoder: &mut defmt::Encoder = &mut *ENCODER_STATE.encoder.get();
        encoder.write(bytes, |b| EncoderState::dual_write(b));
    }

    unsafe fn flush() {
        _SEGGER_RTT.up_channel.flush();
    }

    unsafe fn release() {
        if !ENCODER_STATE.taken.load(Ordering::Relaxed) {
            panic!("defmt release out of context")
        }

        let encoder: &mut defmt::Encoder = &mut *ENCODER_STATE.encoder.get();
        encoder.end_frame(|b| EncoderState::dual_write(b));

        // Send the captured frame to the USB channel (non-blocking).
        let pos = *ENCODER_STATE.capture_pos.get();
        if pos > 0 {
            let buf = &*ENCODER_STATE.capture_buf.get();
            let mut frame = DefmtFrame {
                data: [0u8; MAX_FRAME_SIZE],
                len: pos as u16,
            };
            frame.data[..pos].copy_from_slice(&buf[..pos]);
            let _ = DEFMT_CHANNEL.try_send(frame);
        }

        let restore = ENCODER_STATE.cs_restore.get().read();
        ENCODER_STATE.taken.store(false, Ordering::Relaxed);
        critical_section::release(restore);
    }
}
