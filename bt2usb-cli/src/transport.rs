//! HID transport with COBS framing
//!
//! Handles auto-detection of the bt2usb vendor HID interface, COBS frame
//! encode/decode, and request/response correlation via sequence IDs.

use anyhow::{bail, Context, Result};
use hidapi::{HidApi, HidDevice};
use std::time::{Duration, Instant};

use crate::protocol::{self, HEADER_SIZE, MSG_EVENT, MSG_REQUEST, MSG_RESPONSE};

/// USB VID:PID for bt2usb device
const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;

/// Vendor HID usage page for the RPC interface
const VENDOR_USAGE_PAGE: u16 = 0xFF00;

/// Maximum sizes (must match firmware framing.rs)
const MAX_MSG_SIZE: usize = 256;
const MAX_PAYLOAD_SIZE: usize = HEADER_SIZE + MAX_MSG_SIZE;
const MAX_FRAME_SIZE: usize = cobs::max_encoding_length(MAX_PAYLOAD_SIZE) + 1;

/// A message received from the device (either a Response or an Event).
#[derive(Debug)]
pub enum Message {
    Response { seq_id: u16, cbor: Vec<u8> },
    Event { cbor: Vec<u8> },
}

/// Transport layer for communicating with the bt2usb device over HID.
pub struct Transport {
    device: HidDevice,
    seq_counter: u16,
    /// Buffered bytes from partial reads
    read_buf: Vec<u8>,
}

impl Transport {
    /// Auto-detect and connect to the bt2usb vendor HID interface.
    pub fn connect() -> Result<Self> {
        let api = HidApi::new().context("Failed to initialize HID API")?;
        let device = find_device_hid(&api)?;

        Ok(Self {
            device,
            seq_counter: 1,
            read_buf: Vec::with_capacity(512),
        })
    }

    /// Connect to a specific HID device by path.
    pub fn connect_to(device_path: &str) -> Result<Self> {
        let api = HidApi::new().context("Failed to initialize HID API")?;
        let device = api
            .open_path(std::ffi::CString::new(device_path)?.as_c_str())
            .with_context(|| format!("Failed to open HID device: {device_path}"))?;

        Ok(Self {
            device,
            seq_counter: 1,
            read_buf: Vec::with_capacity(512),
        })
    }

    /// Send a request and wait for the matching response.
    /// Returns the response, and any events received while waiting.
    pub fn request(
        &mut self,
        cbor_body: &[u8],
        timeout: Duration,
    ) -> Result<(protocol::Response, Vec<protocol::Event>)> {
        let seq_id = self.seq_counter;
        self.seq_counter = self.seq_counter.wrapping_add(1);
        if self.seq_counter == 0 {
            self.seq_counter = 1; // 0 is reserved for events
        }

        // Encode and send
        let frame = encode_frame(MSG_REQUEST, seq_id, cbor_body);
        self.write_frame(&frame)?;

        // Wait for matching response
        let deadline = Instant::now() + timeout;
        let mut events = Vec::new();

        loop {
            if Instant::now() > deadline {
                bail!("Request timeout (seq_id={seq_id})");
            }

            match self.read_message() {
                Ok(Some(Message::Response { seq_id: rid, cbor })) if rid == seq_id => {
                    let resp = protocol::decode_response(&cbor)
                        .map_err(|e| anyhow::anyhow!("decode response: {e}"))?;
                    return Ok((resp, events));
                }
                Ok(Some(Message::Response { seq_id: rid, .. })) => {
                    // Response for a different seq_id, discard
                    eprintln!("Warning: unexpected response seq_id={rid}, expected {seq_id}");
                }
                Ok(Some(Message::Event { cbor })) => {
                    if let Ok(evt) = protocol::decode_event(&cbor) {
                        events.push(evt);
                    }
                }
                Ok(None) => {
                    // No complete message yet, continue reading
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }
    }

    /// Send a request with no payload (simple command).
    pub fn request_simple(
        &mut self,
        cmd_id: u8,
        timeout: Duration,
    ) -> Result<(protocol::Response, Vec<protocol::Event>)> {
        let mut cbor_buf = [0u8; 16];
        let len = protocol::encode_request_simple(&mut cbor_buf, cmd_id)
            .map_err(|_| anyhow::anyhow!("encode failed"))?;
        self.request(&cbor_buf[..len], timeout)
    }

    /// Read messages (events) in a streaming fashion. Calls the callback for each
    /// event/response received. Returns on timeout or when callback returns false.
    pub fn stream_messages(
        &mut self,
        timeout: Duration,
        mut callback: impl FnMut(Message) -> bool,
    ) -> Result<()> {
        let deadline = Instant::now() + timeout;

        loop {
            if Instant::now() > deadline {
                return Ok(());
            }

            match self.read_message() {
                Ok(Some(msg)) => {
                    if !callback(msg) {
                        return Ok(());
                    }
                }
                Ok(None) => {}
                Err(e) => {
                    return Err(e);
                }
            }
        }
    }

    /// Write a COBS frame over HID, splitting into 64-byte reports as needed.
    /// Each report is zero-padded to exactly 64 bytes. The first byte is the
    /// report ID (0x00 for devices without numbered reports).
    fn write_frame(&self, frame: &[u8]) -> Result<()> {
        for chunk in frame.chunks(64) {
            let mut buf = [0u8; 65]; // 1 byte report ID + 64 bytes data
            buf[0] = 0x00; // report ID (none)
            buf[1..1 + chunk.len()].copy_from_slice(chunk);
            // Remaining bytes are already zero (padding)
            self.device
                .write(&buf)
                .context("HID write failed")?;
        }
        Ok(())
    }

    /// Try to read one complete message from the HID device.
    fn read_message(&mut self) -> Result<Option<Message>> {
        // Read available HID report (100ms timeout)
        let mut tmp = [0u8; 64];
        match self.device.read_timeout(&mut tmp, 100) {
            Ok(n) if n > 0 => {
                // Trim trailing zero padding from HID report.
                // COBS data never contains 0x00, so trailing zeros are always padding.
                // Keep one trailing 0x00 as the COBS frame delimiter.
                let end = match tmp[..n].iter().rposition(|&b| b != 0) {
                    Some(pos) => (pos + 2).min(n),
                    None => 0, // all zeros = pure padding, discard
                };
                if end > 0 {
                    self.read_buf.extend_from_slice(&tmp[..end]);
                }
            }
            Ok(_) => {} // timeout, no data
            Err(e) => bail!("HID read error: {e}"),
        }

        // Look for a COBS frame delimiter (0x00)
        if let Some(delim_pos) = self.read_buf.iter().position(|&b| b == 0x00) {
            let frame_data: Vec<u8> = self.read_buf.drain(..=delim_pos).collect();
            // frame_data includes the delimiter; COBS data is everything before it
            let cobs_data = &frame_data[..frame_data.len() - 1];

            if cobs_data.is_empty() {
                return Ok(None);
            }

            let mut decoded = vec![0u8; cobs_data.len()];
            let decoded_len = cobs::decode(cobs_data, &mut decoded)
                .map_err(|_| anyhow::anyhow!("COBS decode failed"))?;

            if decoded_len < HEADER_SIZE {
                return Ok(None);
            }

            let (msg_type, seq_id) = protocol::decode_header(&decoded);
            let cbor = decoded[HEADER_SIZE..decoded_len].to_vec();

            match msg_type {
                MSG_RESPONSE => Ok(Some(Message::Response { seq_id, cbor })),
                MSG_EVENT => Ok(Some(Message::Event { cbor })),
                _ => Ok(None),
            }
        } else {
            Ok(None)
        }
    }
}

/// Encode a message into a COBS frame with 0x00 delimiter.
fn encode_frame(msg_type: u8, seq_id: u16, cbor_body: &[u8]) -> Vec<u8> {
    let payload_len = HEADER_SIZE + cbor_body.len();
    let mut payload = vec![0u8; payload_len];
    protocol::encode_header(&mut payload, msg_type, seq_id);
    payload[HEADER_SIZE..].copy_from_slice(cbor_body);

    let mut frame = vec![0u8; MAX_FRAME_SIZE];
    let encoded_len = cobs::encode(&payload, &mut frame);
    frame.truncate(encoded_len + 1);
    frame[encoded_len] = 0x00; // delimiter
    frame
}

/// Auto-detect the bt2usb vendor HID interface by VID:PID and usage page.
fn find_device_hid(api: &HidApi) -> Result<HidDevice> {
    for info in api.device_list() {
        if info.vendor_id() == USB_VID
            && info.product_id() == USB_PID
            && info.usage_page() == VENDOR_USAGE_PAGE
        {
            return info
                .open_device(api)
                .context("Failed to open bt2usb HID device");
        }
    }

    // List what we did find for debugging
    let found: Vec<String> = api
        .device_list()
        .filter(|d| d.vendor_id() == USB_VID && d.product_id() == USB_PID)
        .map(|d| {
            format!(
                "  usage_page=0x{:04X} usage=0x{:04X} iface={}",
                d.usage_page(),
                d.usage(),
                d.interface_number()
            )
        })
        .collect();

    if found.is_empty() {
        bail!(
            "bt2usb device not found (VID:{USB_VID:04X} PID:{USB_PID:04X}).\n\
             No matching USB HID devices detected.\n\
             Use --device to specify a device path manually."
        );
    } else {
        bail!(
            "bt2usb device found but vendor RPC interface (usage page 0xFF00) not detected.\n\
             Found interfaces:\n{}\n\
             The device may be running older firmware without HID RPC support.",
            found.join("\n")
        );
    }
}

/// Format a BLE address as a human-readable string.
pub fn format_address(addr: &[u8; 6]) -> String {
    format!(
        "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]
    )
}

/// Parse a BLE address from "AA:BB:CC:DD:EE:FF" format.
pub fn parse_address(s: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = s.split(':').collect();
    if parts.len() != 6 {
        bail!("Invalid BLE address format (expected AA:BB:CC:DD:EE:FF)");
    }
    let mut addr = [0u8; 6];
    for (i, part) in parts.iter().enumerate() {
        addr[5 - i] = u8::from_str_radix(part, 16)
            .with_context(|| format!("Invalid hex byte: {part}"))?;
    }
    Ok(addr)
}
