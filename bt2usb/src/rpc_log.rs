//! Log forwarding over RPC
//!
//! Provides simple logging functions that forward messages to the RPC
//! handler for delivery to connected clients via CDC ACM.
//!
//! These functions are non-blocking: if the log channel is full, messages
//! are silently dropped. This coexists with defmt RTT logging - both
//! work simultaneously.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Log levels
pub const LEVEL_DEBUG: u8 = 0;
pub const LEVEL_INFO: u8 = 1;
pub const LEVEL_WARN: u8 = 2;
pub const LEVEL_ERROR: u8 = 3;

/// Maximum log message length
pub const MAX_MSG_LEN: usize = 80;

/// A log event to be forwarded over RPC.
#[derive(Clone)]
pub struct LogEvent {
    pub level: u8,
    pub message: [u8; MAX_MSG_LEN],
    pub len: u8,
}

/// Log channel - holds up to 8 pending log events.
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogEvent, 8> = Channel::new();

fn send_log(level: u8, msg: &str) {
    let bytes = msg.as_bytes();
    let len = bytes.len().min(MAX_MSG_LEN);
    let mut message = [0u8; MAX_MSG_LEN];
    message[..len].copy_from_slice(&bytes[..len]);
    let _ = LOG_CHANNEL.try_send(LogEvent {
        level,
        message,
        len: len as u8,
    });
}

pub fn info(msg: &str) {
    send_log(LEVEL_INFO, msg);
}

pub fn warn(msg: &str) {
    send_log(LEVEL_WARN, msg);
}

pub fn error(msg: &str) {
    send_log(LEVEL_ERROR, msg);
}
