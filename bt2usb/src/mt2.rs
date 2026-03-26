//! Magic Trackpad 2 (MT2) emulation module
//!
//! Provides everything needed to present as an Apple Magic Trackpad 2 over USB:
//! - HID report descriptor for Interface 1 (Mouse + Trackpad)
//! - Feature report data captured from a real MT2
//! - Request handler that serves feature reports to macOS
//! - Touch report synthesis for converting scroll deltas to 2-finger gestures

use core::sync::atomic::{AtomicBool, Ordering};
use defmt::*;
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;

// ============ USB Device Identity ============
// Real MT2 USB identity, used in macOS mode (via OS fingerprinting).

pub const APPLE_VID: u16 = 0x05AC;
pub const MT2_PID: u16 = 0x0265;
pub const MT2_DEVICE_RELEASE: u16 = 0x0871;
pub const MT2_MANUFACTURER: &str = "Apple Inc.";
pub const MT2_PRODUCT: &str = "Magic Trackpad";

/// Whether multitouch mode has been activated by the host.
/// Set when macOS sends SET_REPORT(Feature(0x02), [0x01]).
pub static MT_ENABLED: AtomicBool = AtomicBool::new(false);

// ============ Interface 1: Mouse + Trackpad (01/02) ============

/// HID report descriptor for Interface 1 — byte-for-byte match of the real MT2.
///
/// 110 bytes, identical to the real Magic Trackpad 2. Contains:
///   Collection 1: Mouse (Generic Desktop) — Report ID 0x02, 7 bytes
///   Collection 2: Digitizer Touch Pad — Report ID 0x3F, 16 bytes vendor
///   Collection 3: Vendor Multitouch — Report ID 0x44, 1387 bytes
///
/// No Finger collections or Feature declarations — the real MT2 doesn't
/// have them either. macOS reports `digitizer: 0` and "Invalid digitizer
/// transducer" for both the real MT2 and our device — this is normal and
/// does not prevent scrolling (confirmed by testing the real MT2 on the VM).
pub const TRACKPAD_REPORT_DESC: &[u8] = &[
    // === Collection 1: Mouse (Generic Desktop / Mouse) ===
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (1)
    0x29, 0x03, //     Usage Maximum (3)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x85, 0x02, //     Report ID (2)
    0x95, 0x03, //     Report Count (3)
    0x75, 0x01, //     Report Size (1)
    0x81, 0x02, //     Input (Data, Variable, Absolute)
    0x95, 0x01, //     Report Count (1)
    0x75, 0x05, //     Report Size (5)
    0x81, 0x01, //     Input (Constant) - padding
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    0x95, 0x04, //     Report Count (4)
    0x75, 0x08, //     Report Size (8)
    0x81, 0x01, //     Input (Constant) - padding
    0xC0, //   End Collection (Physical)
    0xC0, // End Collection (Application)
    // === Collection 2: Digitizer Touch Pad (vendor data only) ===
    0x05, 0x0D, // Usage Page (Digitizer)
    0x09, 0x05, // Usage (Touch Pad)
    0xA1, 0x01, // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0C, //   Usage (Vendor 0x0C)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x10, //   Report Count (16)
    0x85, 0x3F, //   Report ID (0x3F)
    0x81, 0x22, //   Input (Data, Variable, Absolute, No Preferred)
    0xC0, // End Collection
    // === Collection 3: Vendor Multitouch ===
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x0C, // Usage (Vendor 0x0C)
    0xA1, 0x01, // Collection (Application)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor 0xFF00)
    0x09, 0x0C, //   Usage (Vendor 0x0C)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x85, 0x44, //   Report ID (0x44)
    0x75, 0x08, //   Report Size (8)
    0x96, 0x6B, 0x05, //   Report Count (1387)
    0x81, 0x00, //   Input (Data, Array, Absolute)
    0xC0, // End Collection
];

// ============ Feature Report Data (captured from real MT2) ============
// Source: notes/magic-mouse/capture_trackpad_features_result.txt
// All arrays contain DATA ONLY (no Report ID prefix — write_feature() adds it).

/// Feature 0xDB — Compound device properties (Interface 1).
/// Contains embedded sub-reports for D1, D3, D0, A1, D9, 7F.
/// 75 data bytes. Last 12 bytes approximated (capture truncated at 64 display bytes).
const FEATURE_DB_IF1: &[u8] = &[
    0x01, 0x02, 0x00, 0xd1, 0x81, 0x0f, 0x00, 0xd3, 0x01, 0x16, 0x1e, 0x05, 0x15, 0x00, 0x14, 0x1e,
    0x62, 0x05, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0xd0, 0x02, 0x01, 0x00, 0x14, 0x01, 0x00, 0x1e,
    0x00, 0x02, 0x14, 0x02, 0x01, 0x0e, 0x02, 0x00, 0x07, 0x00, 0xa1, 0x00, 0x00, 0x05, 0x00, 0xfa,
    0x01, 0x11, 0x00, 0xd9, 0xf0, 0x3c, 0x00, 0x00, 0x20, 0x2b, 0x00, 0x00, 0x44, 0xe3, 0x52,
    // Remaining bytes reconstructed from individual feature reports
    0xff, 0xbd, 0x1e, 0xe4, 0x26, // last 5 bytes of D9 sub-report
    0x05, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, // 7F sub-report entry
];

// Individual feature constants removed — only the compound 0xDB is needed.
// The sub-report data (D1, D3, D0, A1, D9, 7F) is embedded within FEATURE_DB_IF1.
// IF0 feature data (34, B4, C5, E0, DB_IF0, D1_IF0) removed — IF0 not required.

// ============ Request Handlers ============

/// Write a Feature report response into `buf`, prepending the report ID.
///
/// USB HID spec requires GET_REPORT responses to include the report ID as
/// the first byte when report IDs are in use. embassy-usb does NOT add this
/// automatically — the handler must include it.
fn write_feature(buf: &mut [u8], report_id: u8, data: &[u8]) -> usize {
    buf[0] = report_id;
    let len = data.len().min(buf.len() - 1);
    buf[1..1 + len].copy_from_slice(&data[..len]);
    1 + len
}

/// Feature report handler for Interface 1 (Mouse/Trackpad).
///
/// Serves Apple proprietary feature reports and handles multitouch activation.
/// Feature 0x01 is stateful: the driver SETs a selector, then GETs the response.
pub struct Mt2TrackpadRequestHandler {
    /// Last data written via SET Feature(0x01). The driver uses this to select
    /// which sub-report to read. GET Feature(0x01) echoes it back.
    feature_01_data: [u8; 4],
}

impl Mt2TrackpadRequestHandler {
    pub const fn new() -> Self {
        Self {
            // Default from real device capture: [sub_id, 0x00, len_lo, len_hi]
            // Real device returns [0x01, 0x99, 0x00, 0x02, 0x00] for GET Feature(0x01)
            // where 0x99 = report count(?), 0x00 = padding, 0x02 = MT report ID, 0x00 = high
            feature_01_data: [0x99, 0x00, 0x02, 0x00],
        }
    }
}

impl RequestHandler for Mt2TrackpadRequestHandler {
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let len = match id {
            ReportId::Feature(0x00) => write_feature(buf, 0x00, &[0x01]),
            ReportId::Feature(0x01) => write_feature(buf, 0x01, &self.feature_01_data),
            ReportId::Feature(0x02) => {
                let enabled = MT_ENABLED.load(Ordering::Relaxed);
                write_feature(buf, 0x02, &[if enabled { 0x01 } else { 0x00 }])
            }
            ReportId::Feature(0xDB) => write_feature(buf, 0xDB, FEATURE_DB_IF1),
            _ => {
                // Accept any unknown GET with a STALL (matching real MT2 behavior)
                debug!("MT2 trackpad: GET {:?} -> not handled", id);
                return None;
            }
        };
        info!(
            "MT2 trackpad: GET Feature(0x{:02X}) -> {} bytes",
            buf[0], len
        );
        Some(len)
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        match id {
            ReportId::Feature(0x01) => {
                // Sub-report selector protocol used by decodeDeviceProperty:
                //   1. Host SETs Feature(0x01, [selector=0x01, sub_report_id])
                //   2. Host GETs Feature(0x01) → [selector, sub_id, 0x00, len_lo, len_hi]
                //   3. Host GETs Feature(sub_id) with wLength = len + 1
                //
                // From USB capture of real MT2: GET Feature(0x01) returns
                // [0x01, sub_id, 0x00, data_len_lo, data_len_hi] where data_len
                // is the sub-report DATA size (excluding report ID byte).
                // TopCase reads bytes 3-4 as LE uint16 and adds 1 for wLength.
                if data.len() >= 2 && data[0] == 0x01 {
                    let sub_id = data[1];
                    // Only 0xDB (compound properties) is ever fetched in practice
                    let data_len: u16 = match sub_id {
                        0xDB => FEATURE_DB_IF1.len() as u16,
                        _ => 0,
                    };
                    self.feature_01_data = [
                        sub_id,
                        0x00, // padding byte (confirmed from real MT2 USB capture)
                        (data_len & 0xFF) as u8,
                        ((data_len >> 8) & 0xFF) as u8,
                    ];
                    info!(
                        "MT2 trackpad: SET Feature(0x01) -> sub=0x{:02X} data_len={}",
                        sub_id, data_len
                    );
                } else {
                    let copy_len = data.len().min(self.feature_01_data.len());
                    self.feature_01_data[..copy_len].copy_from_slice(&data[..copy_len]);
                    info!("MT2 trackpad: SET Feature(0x01) ({} bytes)", data.len());
                }
                OutResponse::Accepted
            }
            ReportId::Feature(0x02) => {
                // Multitouch activation: {0x02, 0x01} enables, {0x02, 0x00} disables
                if !data.is_empty() {
                    let enabled = data[0] != 0;
                    MT_ENABLED.store(enabled, Ordering::Relaxed);
                    info!(
                        "MT2 trackpad: SET Feature(0x02) -> multitouch {}",
                        if enabled { "ENABLED" } else { "disabled" }
                    );
                }
                OutResponse::Accepted
            }
            _ => {
                info!(
                    "MT2 trackpad: SET {:?} ({} bytes) -> accepted",
                    id,
                    data.len()
                );
                OutResponse::Accepted
            }
        }
    }

    fn set_idle_ms(&mut self, _id: Option<ReportId>, _dur: u32) {}
    fn get_idle_ms(&mut self, _id: Option<ReportId>) -> Option<u32> {
        None
    }
}

// ============ Touch Report Synthesis ============

/// MT2 trackpad Y coordinate limits.
/// Extended beyond the real MT2 (-2478/+2587) to allow more finger travel
/// before replanting. The 13-bit packed format supports ±4095.
const TRACKPAD2_MIN_Y: i16 = -4000;
const TRACKPAD2_MAX_Y: i16 = 4000;

/// Idle timeout in ticks before lifting fingers (150ms at 1MHz tick rate).
const IDLE_TIMEOUT_TICKS: u64 = 150_000;

/// Maximum per-event Y delta. Caps the raw BLE delta before adding to
/// the velocity buffer.
const MAX_DELTA_PER_EVENT: i16 = 500;

/// MT2-specific scroll scale as percentage. Applied to deltas inside the
/// synthesizer so it only affects the MT2 trackpad path, not the standard
/// USB mouse wheel path. Tuned so that a given MULTIPLIER_SCROLL setting
/// produces roughly the same scroll speed on macOS (via MT2) as on
/// Windows/Linux (via standard HID wheel).
const MT2_SCROLL_SCALE_PCT: i32 = 200;

/// Maximum drain rate (Y units per tick). Caps the tracked velocity to
/// prevent extreme speeds in macOS's nonlinear acceleration region.
/// At 4ms ticks: 250 units/tick = 62,500 units/sec.
const MAX_DRAIN_RATE: i32 = 250;

/// Maximum velocity buffer size. Prevents unbounded accumulation when
/// BLE input rate exceeds drain rate momentarily.
const MAX_BUFFER: i32 = 4000;

/// Buffers at or below this size are applied in one tick instead of
/// being smoothed. Prevents tiny deltas from being spread across many
/// ticks at sub-threshold velocity that macOS ignores.
const IMMEDIATE_THRESHOLD: i32 = 30;

/// Tick interval in microseconds (4ms). Used to convert elapsed time
/// between BLE events into tick counts for drain rate calculation.
const TICK_INTERVAL_US: u64 = 4000;

/// Timestamp increment per report (~8kHz / ~250Hz ≈ 32 ticks).
const TIMESTAMP_INCREMENT: u16 = 32;

/// Byte offset of finger 0 touch data within the 30-byte report.
const FINGER0_OFFSET: usize = 12;
/// Byte offset of finger 1 touch data within the 30-byte report.
const FINGER1_OFFSET: usize = 21;

/// Y range for finger travel before resetting. Uses most of the extended
/// ±4000 coordinate space, leaving headroom so a max-size delta doesn't
/// overshoot the 13-bit encoding limit (±4095).
const Y_RESET_THRESHOLD: i16 = 3500;

/// Number of reports for the lift-and-replace sequence.
/// RELEASE(2) + NONE(1) + APPROACH(1) = 4 reports at ~4ms = ~16ms.
/// (The real MT2 reports at ~91 Hz / ~11ms; we use 250 Hz / 4ms for smoother output.)
const REPLANT_REPORTS: u8 = 4;

// Template touch reports captured from a real MT2 scroll gesture.
// Used as byte-exact templates — only Y coordinates and timestamps are patched.
const TEMPLATE_NONE: [u8; 30] = [
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x5c, 0x29, 0xe6, 0x9b, 0x5b, 0xb6, 0x23,
    0x53, 0x7e, 0x18, 0x0d, 0x25, 0xbe, 0xff, 0xc7, 0x23, 0x6c, 0x87, 0x12, 0x0d, 0x28,
];
const TEMPLATE_APPROACH: [u8; 30] = [
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0xb4, 0x29, 0xe6, 0xa5, 0x5b, 0xbb, 0x6b,
    0x5a, 0x83, 0x1a, 0x10, 0x85, 0xc2, 0x3f, 0xcf, 0x6f, 0x6e, 0x87, 0x15, 0x0e, 0x88,
];
const TEMPLATE_TOUCH: [u8; 30] = [
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x31, 0x0c, 0x2a, 0xe6, 0xaf, 0xdb, 0xc0, 0x8b,
    0x62, 0x89, 0x1b, 0x15, 0x85, 0xda, 0xff, 0xd6, 0x8f, 0x75, 0x80, 0x16, 0x12, 0x08,
];
// Release template: both fingers in RELEASE state (0xC0) with fading attributes,
// then byte7=0x03 (both still present). Uses same X positions as TEMPLATE_TOUCH
// to avoid position jumps. Touch attributes are small (fading contact).
const TEMPLATE_RELEASE: [u8; 30] = [
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // header: byte7=0x03 (both fingers)
    0x31, 0x00, 0x00, 0xe6, // marker, timestamp (patched), constant
    // Finger 0: same X as TEMPLATE_TOUCH, Y=0, state=0xC0 (releasing), small attrs
    0xaf, 0xdb, 0x00, 0xc0, 0x40, 0x50, 0x10, 0x03, 0x85,
    // Finger 1: same X as TEMPLATE_TOUCH, Y=0, state=0xC0 (releasing), small attrs
    0xda, 0xff, 0x00, 0xc0, 0x40, 0x50, 0x10, 0x03, 0x08,
];
// Final "gone" template: both fingers NONE state, zero attributes.
const TEMPLATE_GONE: [u8; 30] = [
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // header: byte7=0x00 (no fingers)
    0x31, 0x00, 0x00, 0xe6, // Finger 0: state=0x00, zero attrs
    0xaf, 0xdb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85,
    // Finger 1: state=0x00, zero attrs
    0xda, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
];

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    /// No fingers on surface.
    Idle,
    /// Fingers actively scrolling (TOUCH state).
    Scrolling,
    /// Briefly lifting fingers to reset Y position (replanting).
    Replanting(u8),
    /// Ending gesture: clean release sequence to avoid tap detection.
    Ending(u8),
}

/// Synthesizes 2-finger scroll gestures from scroll deltas.
///
/// Uses real MT2 captured reports as byte-exact templates, modifying only
/// the Y coordinates. When fingers reach the edge of the virtual surface,
/// performs a quick lift-and-replace cycle (like a real user would) to
/// reset the Y position and continue scrolling.
///
/// Constant-rate drain: BLE deltas accumulate in a velocity buffer.
/// Each BLE event updates a smoothed drain rate (delta / elapsed_ticks).
/// tick() applies that rate uniformly, producing constant Y velocity
/// between events. This eliminates the sawtooth velocity pattern from
/// exponential decay that caused macOS's nonlinear acceleration to
/// amplify peaks disproportionally (the "runaway acceleration" effect).
pub struct TouchSynthesizer {
    /// Current Y position of both fingers on the virtual surface.
    y_pos: i16,
    /// Current gesture phase.
    phase: Phase,
    /// Accumulated Y delta to be applied across tick() calls.
    velocity_buffer: i32,
    /// Smoothed drain rate: Y units per tick. Updated on each BLE event
    /// as `|delta| / elapsed_ticks`, then EMA-smoothed. Produces constant
    /// velocity between events — no sawtooth, no nonlinear amplification.
    drain_rate: i32,
    /// Tick count of the last scroll event (for idle detection).
    last_event_ticks: u64,
    /// Internal timestamp for report headers.
    timestamp: u16,
}

impl TouchSynthesizer {
    pub const fn new() -> Self {
        Self {
            y_pos: 0,
            phase: Phase::Idle,
            velocity_buffer: 0,
            drain_rate: 0,
            last_event_ticks: 0,
            timestamp: 10000,
        }
    }

    /// Process a scroll delta. Returns 0-2 reports (gesture start only).
    ///
    /// The delta is added to the velocity buffer. The drain rate is updated
    /// based on `|delta| / ticks_since_last_event`, smoothed via EMA.
    /// tick() drains the buffer at that constant rate.
    pub fn process_scroll(&mut self, delta: i16) -> heapless::Vec<[u8; 30], 3> {
        let now = embassy_time::Instant::now().as_ticks();
        let elapsed_us = now.saturating_sub(self.last_event_ticks);
        self.last_event_ticks = now;
        let mut out = heapless::Vec::new();

        // Invert delta: positive dial rotation = scroll down = fingers move
        // in negative Y direction on the trackpad surface.
        // Apply MT2-specific scale to match macOS scroll speed to Windows/Linux.
        let delta = (-delta).clamp(-MAX_DELTA_PER_EVENT, MAX_DELTA_PER_EVENT);
        let delta = ((delta as i32 * MT2_SCROLL_SCALE_PCT) / 100) as i16;

        // Update drain rate from input velocity: |delta| / elapsed_ticks.
        // EMA smoothing (75% old + 25% new) prevents jitter from
        // irregular BLE event timing.
        let elapsed_ticks = (elapsed_us / TICK_INTERVAL_US).max(1) as i32;
        let new_rate = ((delta.abs() as i32) / elapsed_ticks).clamp(1, MAX_DRAIN_RATE);
        let prev_rate = self.drain_rate;
        self.drain_rate = if prev_rate == 0 {
            new_rate
        } else {
            (prev_rate * 3 + new_rate) / 4
        };

        // If input velocity has dropped dramatically (dial nearly stopped),
        // discard the buffered backlog. The user's intent is "stop" — not
        // "finish draining the 4000-unit buffer at 10 units/tick."
        if prev_rate > 0 && new_rate <= prev_rate / 4 {
            self.velocity_buffer = 0;
        }

        match self.phase {
            Phase::Idle | Phase::Ending(_) => {
                // Start new gesture
                self.y_pos = 0;
                self.velocity_buffer = delta as i32;
                self.drain_rate = new_rate;
                debug!("MT2 touch: gesture START");
                let _ = out.push(self.make_report(&TEMPLATE_NONE));
                let _ = out.push(self.make_report(&TEMPLATE_APPROACH));
                self.phase = Phase::Scrolling;
            }
            Phase::Replanting(_) | Phase::Scrolling => {
                self.velocity_buffer =
                    (self.velocity_buffer + delta as i32).clamp(-MAX_BUFFER, MAX_BUFFER);
            }
        }

        out
    }

    /// Called on every timer tick (~4ms). Drains the velocity buffer at
    /// the tracked constant rate.
    pub fn tick(&mut self) -> Option<[u8; 30]> {
        match self.phase {
            Phase::Idle => None,
            Phase::Replanting(remaining) => {
                if remaining == 0 {
                    self.phase = Phase::Scrolling;
                    debug!("MT2 touch: replant done, resuming");
                    Some(self.make_report(&TEMPLATE_TOUCH))
                } else {
                    self.phase = Phase::Replanting(remaining - 1);
                    match remaining {
                        4 | 3 => Some(self.make_report(&TEMPLATE_RELEASE)),
                        2 => Some(self.make_report(&TEMPLATE_NONE)),
                        1 => Some(self.make_report(&TEMPLATE_APPROACH)),
                        _ => Some(self.make_report(&TEMPLATE_RELEASE)),
                    }
                }
            }
            Phase::Ending(remaining) => {
                if remaining == 0 {
                    self.phase = Phase::Idle;
                    Some(self.make_report(&TEMPLATE_GONE))
                } else {
                    self.phase = Phase::Ending(remaining - 1);
                    Some(self.make_report(&TEMPLATE_RELEASE))
                }
            }
            Phase::Scrolling => {
                // If no BLE events for a while, the dial has stopped (or
                // nearly so). Halve the drain rate each tick so the buffer
                // drains rapidly instead of coasting at the old speed.
                // 50ms ≈ 12 ticks → rate halved 12 times → effectively 0.
                let now_ticks = embassy_time::Instant::now().as_ticks();
                let since_last = now_ticks.saturating_sub(self.last_event_ticks);
                if since_last > 50_000 && self.drain_rate > 0 {
                    self.drain_rate /= 2;
                    if self.drain_rate == 0 {
                        self.velocity_buffer = 0;
                    }
                }

                // Drain velocity buffer at the tracked constant rate.
                if self.velocity_buffer != 0 {
                    let step = if self.velocity_buffer.abs() <= IMMEDIATE_THRESHOLD {
                        // Small buffer: apply all at once so macOS registers it
                        self.velocity_buffer
                    } else {
                        // Constant-rate drain: use tracked rate, capped to
                        // remaining buffer. Produces uniform velocity between
                        // BLE events — no sawtooth from exponential decay.
                        let rate = self.drain_rate.max(1);
                        let magnitude = rate.min(self.velocity_buffer.abs());
                        magnitude * self.velocity_buffer.signum()
                    };
                    self.velocity_buffer -= step;

                    let new_y = (self.y_pos as i32 + step)
                        .clamp(TRACKPAD2_MIN_Y as i32, TRACKPAD2_MAX_Y as i32)
                        as i16;

                    if new_y.abs() > Y_RESET_THRESHOLD {
                        debug!("MT2 touch: replanting (Y={})", self.y_pos);
                        let report = self.make_report(&TEMPLATE_RELEASE);
                        self.y_pos = 0;
                        self.phase = Phase::Replanting(REPLANT_REPORTS);
                        return Some(report);
                    }

                    self.y_pos = new_y;
                }

                // End gesture when no BLE events for IDLE_TIMEOUT and
                // buffer is fully drained (so RELEASE fires at zero velocity).
                let now = embassy_time::Instant::now().as_ticks();
                if now - self.last_event_ticks > IDLE_TIMEOUT_TICKS && self.velocity_buffer == 0 {
                    self.phase = Phase::Ending(1);
                    self.drain_rate = 0;
                    debug!("MT2 touch: gesture END");
                    Some(self.make_report(&TEMPLATE_RELEASE))
                } else {
                    Some(self.make_report(&TEMPLATE_TOUCH))
                }
            }
        }
    }

    /// Create a report from a template, patching the timestamp and Y coordinates.
    fn make_report(&mut self, template: &[u8; 30]) -> [u8; 30] {
        let mut report = *template;

        // Patch timestamp (bytes 9-10)
        report[9] = (self.timestamp & 0xFF) as u8;
        report[10] = (self.timestamp >> 8) as u8;
        self.timestamp = self.timestamp.wrapping_add(TIMESTAMP_INCREMENT);

        // Patch Y in both touch points
        self.patch_y(&mut report, FINGER0_OFFSET, self.y_pos);
        self.patch_y(&mut report, FINGER1_OFFSET, self.y_pos);

        report
    }

    /// Patch the Y coordinate in a packed touch point at the given offset.
    fn patch_y(&self, report: &mut [u8; 30], off: usize, y: i16) {
        let neg_y_raw = ((-y) as u16) & 0x1FFF;
        let t = &mut report[off..off + 9];
        t[1] = (t[1] & 0x1F) | (((neg_y_raw & 0x07) << 5) as u8);
        t[2] = ((neg_y_raw >> 3) & 0xFF) as u8;
        t[3] = (t[3] & 0xFC) | (((neg_y_raw >> 11) & 0x03) as u8);
    }
}
