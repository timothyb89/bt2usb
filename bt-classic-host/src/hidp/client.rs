//! HIDP HID Client: opens HID channels, receives reports, sends SET_REPORT.
//!
//! The HidClient manages two L2CAP channels:
//! - Control (PSM 0x0011): Commands, GET/SET_REPORT, handshakes
//! - Interrupt (PSM 0x0013): Unsolicited input reports
//!
//! Usage:
//! ```rust,ignore
//! let mut hid = HidClient::new();
//! hid.open_channels(&mut l2cap, &controller).await?;
//! // Then in the event loop, feed ACL data to process() and check for reports.
//! ```

use bt_hci::controller::Controller;

use super::types::{self, HandshakeResult, MessageType, ReportType, PSM_HID_CONTROL, PSM_HID_INTERRUPT};
use crate::error::Error;
use crate::l2cap::L2capState;

/// Maximum HID report size we support.
pub const MAX_REPORT_SIZE: usize = 256;

/// A received HID report.
pub struct HidReport {
    /// Report type (Input, Output, Feature).
    pub report_type: ReportType,
    /// Report data (starting with Report ID if present).
    pub data: [u8; MAX_REPORT_SIZE],
    /// Length of valid data in the buffer.
    pub len: usize,
}

impl HidReport {
    pub const fn new() -> Self {
        Self {
            report_type: ReportType::Input,
            data: [0u8; MAX_REPORT_SIZE],
            len: 0,
        }
    }
}

/// HIDP client managing HID channels and report reception.
pub struct HidClient {
    /// L2CAP channel index for HID Control (PSM 0x0011).
    pub control_channel: Option<usize>,
    /// L2CAP channel index for HID Interrupt (PSM 0x0013).
    pub interrupt_channel: Option<usize>,
}

impl HidClient {
    pub const fn new() -> Self {
        Self {
            control_channel: None,
            interrupt_channel: None,
        }
    }

    /// Open both HID L2CAP channels (Control + Interrupt).
    ///
    /// Returns after sending Connection Requests. The channels will
    /// transition to Open as signaling responses arrive via process_acl().
    pub async fn open_channels<C: Controller, const CH: usize>(
        &mut self,
        l2cap: &mut L2capState<CH>,
        controller: &C,
    ) -> Result<(), Error<C::Error>> {
        // Open Control channel (PSM 0x0011)
        let ctrl_idx = l2cap.open_channel(controller, PSM_HID_CONTROL).await?;
        self.control_channel = Some(ctrl_idx);

        // Open Interrupt channel (PSM 0x0013)
        let intr_idx = l2cap.open_channel(controller, PSM_HID_INTERRUPT).await?;
        self.interrupt_channel = Some(intr_idx);

        #[cfg(feature = "defmt")]
        defmt::info!(
            "[hidp] Opening HID channels: control={}, interrupt={}",
            ctrl_idx,
            intr_idx
        );

        Ok(())
    }

    /// Check if both HID channels are open.
    pub fn is_ready<const CH: usize>(&self, l2cap: &L2capState<CH>) -> bool {
        let ctrl_open = self
            .control_channel
            .map_or(false, |idx| l2cap.is_channel_open(idx));
        let intr_open = self
            .interrupt_channel
            .map_or(false, |idx| l2cap.is_channel_open(idx));
        ctrl_open && intr_open
    }

    /// Process L2CAP data that arrived on one of our HID channels.
    ///
    /// `channel_idx` and `data` come from L2capState::process_acl().
    /// Returns `Some(report)` if this was an input report on the interrupt channel.
    pub fn process_data(
        &self,
        channel_idx: usize,
        data: &[u8],
        report: &mut HidReport,
    ) -> bool {
        if data.is_empty() {
            return false;
        }

        let (msg_type, param) = types::parse_header(data[0]);
        let payload = &data[1..];

        if Some(channel_idx) == self.interrupt_channel {
            // Interrupt channel: DATA messages (input reports)
            if msg_type == MessageType::Data as u8 {
                let report_type = match param {
                    0x01 => ReportType::Input,
                    0x02 => ReportType::Output,
                    _ => ReportType::Input, // Default to input
                };
                let copy_len = payload.len().min(MAX_REPORT_SIZE);
                report.data[..copy_len].copy_from_slice(&payload[..copy_len]);
                report.len = copy_len;
                report.report_type = report_type;
                return true;
            }
        } else if Some(channel_idx) == self.control_channel {
            // Control channel: handshakes and responses
            if msg_type == MessageType::Handshake as u8 {
                let result = param;
                if result != HandshakeResult::Successful as u8 {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("[hidp] Handshake error: 0x{:02x}", result);
                }
            }
            // DATA on control channel = response to GET_REPORT
            if msg_type == MessageType::Data as u8 {
                let report_type = match param {
                    0x01 => ReportType::Input,
                    0x02 => ReportType::Output,
                    0x03 => ReportType::Feature,
                    _ => ReportType::Other,
                };
                let copy_len = payload.len().min(MAX_REPORT_SIZE);
                report.data[..copy_len].copy_from_slice(&payload[..copy_len]);
                report.len = copy_len;
                report.report_type = report_type;
                return true;
            }
        }

        false
    }

    /// Send a SET_REPORT command on the control channel.
    ///
    /// `report_type`: Feature (0x03), Output (0x02), or Input (0x01).
    /// `report_id`: The HID report ID.
    /// `data`: Report data (after the report ID).
    pub async fn set_report<C: Controller, const CH: usize>(
        &self,
        l2cap: &L2capState<CH>,
        controller: &C,
        report_type: ReportType,
        report_id: u8,
        data: &[u8],
    ) -> Result<(), Error<C::Error>> {
        let ctrl_idx = self.control_channel.ok_or(Error::InvalidState)?;

        // Build HIDP SET_REPORT message: [header, report_id, data...]
        let header = types::build_header(MessageType::SetReport as u8, report_type as u8);
        let mut buf = [0u8; 258]; // header(1) + report_id(1) + max_data(256)
        buf[0] = header;
        buf[1] = report_id;
        let copy_len = data.len().min(256);
        buf[2..2 + copy_len].copy_from_slice(&data[..copy_len]);
        let total = 2 + copy_len;

        l2cap.send_data(controller, ctrl_idx, &buf[..total]).await?;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "[hidp] SET_REPORT type={} id=0x{:02x} len={}",
            report_type as u8,
            report_id,
            copy_len
        );

        Ok(())
    }

    /// Send SET_PROTOCOL (Report Protocol) on the control channel.
    pub async fn set_protocol_report<C: Controller, const CH: usize>(
        &self,
        l2cap: &L2capState<CH>,
        controller: &C,
    ) -> Result<(), Error<C::Error>> {
        let ctrl_idx = self.control_channel.ok_or(Error::InvalidState)?;

        // HIDP SET_PROTOCOL: header byte only, param = 0x01 (Report Protocol)
        let header = types::build_header(MessageType::SetProtocol as u8, 0x01);
        l2cap.send_data(controller, ctrl_idx, &[header]).await?;

        #[cfg(feature = "defmt")]
        defmt::info!("[hidp] SET_PROTOCOL Report Mode");

        Ok(())
    }
}
