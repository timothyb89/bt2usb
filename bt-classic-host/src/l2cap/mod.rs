//! Classic L2CAP (Logical Link Control and Adaptation Protocol).
//!
//! Implements connection-oriented channels for Classic Bluetooth.
//! Key differences from LE L2CAP:
//! - Full signaling handshake (Connection Request/Response + Configuration)
//! - MTU negotiation in a separate Configuration phase
//! - No credit-based flow control (basic mode)
//! - Fixed PSMs for HID: 0x0011 (Control) and 0x0013 (Interrupt)

pub mod channel;
pub mod reassembly;
pub mod signal;

use bt_hci::controller::Controller;
use bt_hci::data::{AclBroadcastFlag, AclPacket, AclPacketBoundary};
use bt_hci::param::ConnHandle;

use channel::{ChannelManager, ChannelState};
use reassembly::ReassemblyBuffer;
use signal::SIGNALING_CID;

use crate::error::Error;

/// Default MTU for Classic L2CAP channels.
pub const DEFAULT_MTU: u16 = 672;

/// Maximum L2CAP frame size we support (MTU + 4-byte header).
const MAX_L2CAP_FRAME: usize = 1024;

/// L2CAP layer managing channels and signaling for one ACL connection.
pub struct L2capState<const CHANNELS: usize> {
    /// Channel manager with CID allocation.
    pub channels: ChannelManager<CHANNELS>,
    /// ACL fragment reassembly buffer.
    reassembly: ReassemblyBuffer<MAX_L2CAP_FRAME>,
    /// The ACL connection handle.
    handle: ConnHandle,
    /// Our desired MTU.
    our_mtu: u16,
}

impl<const CHANNELS: usize> L2capState<CHANNELS> {
    /// Create a new L2CAP state for an ACL connection.
    pub fn new(handle: ConnHandle) -> Self {
        Self {
            channels: ChannelManager::new(),
            reassembly: ReassemblyBuffer::new(),
            handle,
            our_mtu: DEFAULT_MTU,
        }
    }

    /// Open an L2CAP channel to the given PSM.
    ///
    /// Sends a Connection Request and returns the channel index.
    /// The channel will be in WaitConnectRsp state — call process_acl()
    /// repeatedly until it transitions to Open.
    pub async fn open_channel<C: Controller>(
        &mut self,
        controller: &C,
        psm: u16,
    ) -> Result<usize, Error<C::Error>> {
        let (idx, local_cid) = self.channels.allocate(psm).ok_or(Error::NoFreeCids)?;
        let sig_id = self.channels.channels[idx].pending_signal_id;

        // Send L2CAP Connection Request via signaling channel
        let cmd = signal::build_connection_request(sig_id, psm, local_cid);
        self.send_signaling(controller, &cmd).await?;

        #[cfg(feature = "defmt")]
        defmt::info!("[l2cap] Sent ConnReq PSM=0x{:04x} SCID=0x{:04x}", psm, local_cid);

        Ok(idx)
    }

    /// Process an incoming ACL data packet.
    ///
    /// Handles L2CAP fragment reassembly and signaling commands.
    /// For non-signaling data, copies it into `out_buf` and returns
    /// `Some((channel_index, length))`.
    pub async fn process_acl<C: Controller>(
        &mut self,
        controller: &C,
        handle_and_flags: u16,
        acl_data: &[u8],
        out_buf: &mut [u8],
    ) -> Result<Option<(usize, usize)>, Error<C::Error>> {
        // Reassemble fragments
        let frame = self.reassembly.process_fragment(handle_and_flags, acl_data);

        let (channel_id, payload) = match frame {
            Some(f) => f,
            None => return Ok(None), // Need more fragments
        };

        if channel_id == SIGNALING_CID {
            // Copy payload before calling handle_signaling (releases reassembly borrow)
            let sig_len = payload.len().min(256);
            let mut sig_buf = [0u8; 256];
            sig_buf[..sig_len].copy_from_slice(&payload[..sig_len]);
            self.handle_signaling(controller, &sig_buf[..sig_len]).await?;
            return Ok(None);
        }

        // Find the channel for this CID
        if let Some((idx, _ch)) = self.channels.find_by_local_cid(channel_id) {
            // Copy data to output buffer for the caller (HIDP layer)
            let copy_len = payload.len().min(out_buf.len());
            out_buf[..copy_len].copy_from_slice(&payload[..copy_len]);
            Ok(Some((idx, copy_len)))
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("[l2cap] Data for unknown CID 0x{:04x}", channel_id);
            Ok(None)
        }
    }

    /// Handle an L2CAP signaling command.
    async fn handle_signaling<C: Controller>(
        &mut self,
        controller: &C,
        data: &[u8],
    ) -> Result<(), Error<C::Error>> {
        // There may be multiple signaling commands in one L2CAP frame
        let mut offset = 0;
        while offset < data.len() {
            let remaining = &data[offset..];
            let (code, id, payload) = match signal::parse_signal_header(remaining) {
                Some(parsed) => parsed,
                None => break,
            };

            match code {
                signal::code::CONNECTION_RESPONSE => {
                    if let Some((dcid, scid, result, _status)) =
                        signal::parse_connection_response(payload)
                    {
                        self.handle_connection_response(controller, dcid, scid, result)
                            .await?;
                    }
                }

                signal::code::CONFIGURATION_REQUEST => {
                    if let Some((dcid, _flags, options)) =
                        signal::parse_configuration_request(payload)
                    {
                        let remote_mtu = signal::extract_mtu_from_options(options).unwrap_or(DEFAULT_MTU);
                        self.handle_configuration_request(controller, id, dcid, remote_mtu)
                            .await?;
                    }
                }

                signal::code::CONFIGURATION_RESPONSE => {
                    if let Some((scid, _flags, result)) =
                        signal::parse_configuration_response(payload)
                    {
                        self.handle_configuration_response(scid, result);
                    }
                }

                signal::code::DISCONNECTION_REQUEST => {
                    if payload.len() >= 4 {
                        let dcid = u16::from_le_bytes([payload[0], payload[1]]);
                        let scid = u16::from_le_bytes([payload[2], payload[3]]);
                        self.handle_disconnection_request(controller, id, dcid, scid)
                            .await?;
                    }
                }

                signal::code::INFORMATION_REQUEST => {
                    // Remote wants to know our extended features
                    if payload.len() >= 2 {
                        let info_type = u16::from_le_bytes([payload[0], payload[1]]);
                        if info_type == 0x0002 {
                            // Extended Features Mask
                            let resp = signal::build_info_response_extended_features(id);
                            self.send_signaling(controller, &resp).await?;
                        }
                    }
                }

                _ => {
                    #[cfg(feature = "defmt")]
                    defmt::debug!("[l2cap] Unknown signaling code 0x{:02x}", code);
                }
            }

            // Advance past this command (4-byte header + payload length)
            let cmd_len = 4 + u16::from_le_bytes([remaining[2], remaining[3]]) as usize;
            offset += cmd_len;
        }
        Ok(())
    }

    /// Handle Connection Response from remote.
    async fn handle_connection_response<C: Controller>(
        &mut self,
        controller: &C,
        dest_cid: u16,
        source_cid: u16,
        result: u16,
    ) -> Result<(), Error<C::Error>> {
        match result {
            signal::conn_result::SUCCESS => {
                if let Some((_idx, ch)) = self.channels.find_by_local_cid(source_cid) {
                    ch.on_connect_response(dest_cid);
                    #[cfg(feature = "defmt")]
                    defmt::info!(
                        "[l2cap] ConnRsp OK: SCID=0x{:04x} DCID=0x{:04x}",
                        source_cid,
                        dest_cid
                    );

                    // Send our Configuration Request
                    let sig_id = self.channels.next_signal_id();
                    let cmd = signal::build_configuration_request(sig_id, dest_cid, self.our_mtu);
                    self.send_signaling(controller, &cmd).await?;
                }
            }
            signal::conn_result::PENDING => {
                #[cfg(feature = "defmt")]
                defmt::debug!("[l2cap] ConnRsp PENDING for SCID=0x{:04x}", source_cid);
                // Stay in WaitConnectRsp, another response will come
            }
            _ => {
                #[cfg(feature = "defmt")]
                defmt::warn!("[l2cap] ConnRsp REJECTED result={}", result);
                if let Some((_idx, ch)) = self.channels.find_by_local_cid(source_cid) {
                    ch.close();
                }
                return Err(Error::L2capRejected);
            }
        }
        Ok(())
    }

    /// Handle Configuration Request from remote.
    async fn handle_configuration_request<C: Controller>(
        &mut self,
        controller: &C,
        signal_id: u8,
        dest_cid: u16,
        remote_mtu: u16,
    ) -> Result<(), Error<C::Error>> {
        // dest_cid is our local CID (the remote is configuring our end)
        if let Some((_idx, ch)) = self.channels.find_by_local_cid(dest_cid) {
            ch.on_remote_config_complete(remote_mtu);

            // Send Configuration Response (accept)
            let resp = signal::build_configuration_response(
                signal_id,
                ch.remote_cid,
                signal::config_result::SUCCESS,
            );
            self.send_signaling(controller, &resp).await?;

            #[cfg(feature = "defmt")]
            defmt::info!(
                "[l2cap] ConfigReq from remote, MTU={}, sent ConfigRsp OK",
                remote_mtu
            );
        }
        Ok(())
    }

    /// Handle Configuration Response from remote.
    fn handle_configuration_response(&mut self, source_cid: u16, result: u16) {
        // source_cid is our local CID
        if result == signal::config_result::SUCCESS {
            if let Some((_idx, ch)) = self.channels.find_by_local_cid(source_cid) {
                ch.on_local_config_complete();
                #[cfg(feature = "defmt")]
                defmt::info!(
                    "[l2cap] ConfigRsp OK for SCID=0x{:04x}, state={:?}",
                    source_cid,
                    ch.state
                );
            }
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("[l2cap] ConfigRsp FAILED result={}", result);
        }
    }

    /// Handle Disconnection Request from remote.
    async fn handle_disconnection_request<C: Controller>(
        &mut self,
        controller: &C,
        signal_id: u8,
        dest_cid: u16,
        source_cid: u16,
    ) -> Result<(), Error<C::Error>> {
        // dest_cid is our local CID, source_cid is remote's CID
        let resp = signal::build_disconnection_response(signal_id, dest_cid, source_cid);
        self.send_signaling(controller, &resp).await?;

        if let Some((_idx, ch)) = self.channels.find_by_local_cid(dest_cid) {
            ch.close();
        }
        Ok(())
    }

    /// Send an L2CAP signaling command over the ACL connection.
    async fn send_signaling<C: Controller>(
        &self,
        controller: &C,
        signaling_data: &[u8],
    ) -> Result<(), Error<C::Error>> {
        // Build L2CAP frame: [length(2), channel_id(2), signaling_data...]
        let l2cap_len = signaling_data.len() as u16;
        let mut frame = [0u8; 256];
        frame[0..2].copy_from_slice(&l2cap_len.to_le_bytes());
        frame[2..4].copy_from_slice(&SIGNALING_CID.to_le_bytes());
        let frame_end = 4 + signaling_data.len();
        frame[4..frame_end].copy_from_slice(signaling_data);

        self.send_acl(controller, &frame[..frame_end]).await
    }

    /// Send data on an open L2CAP channel.
    pub async fn send_data<C: Controller>(
        &self,
        controller: &C,
        channel_idx: usize,
        data: &[u8],
    ) -> Result<(), Error<C::Error>> {
        let ch = &self.channels.channels[channel_idx];
        if ch.state != ChannelState::Open {
            return Err(Error::InvalidState);
        }

        // Build L2CAP frame: [length(2), remote_cid(2), data...]
        let l2cap_len = data.len() as u16;
        let mut frame = [0u8; 1024];
        frame[0..2].copy_from_slice(&l2cap_len.to_le_bytes());
        frame[2..4].copy_from_slice(&ch.remote_cid.to_le_bytes());
        let frame_end = 4 + data.len();
        frame[4..frame_end].copy_from_slice(data);

        self.send_acl(controller, &frame[..frame_end]).await
    }

    /// Send a raw L2CAP frame as an ACL packet.
    async fn send_acl<C: Controller>(
        &self,
        controller: &C,
        l2cap_frame: &[u8],
    ) -> Result<(), Error<C::Error>> {
        let packet = AclPacket::new(
            self.handle,
            AclPacketBoundary::FirstNonFlushable,
            AclBroadcastFlag::PointToPoint,
            l2cap_frame,
        );
        controller.write_acl_data(&packet).await.map_err(Error::Hci)
    }

    /// Check if a channel is open.
    pub fn is_channel_open(&self, idx: usize) -> bool {
        self.channels.channels.get(idx).map_or(false, |ch| ch.is_open())
    }

    /// Check if all specified channel indices are open.
    pub fn all_channels_open(&self, indices: &[usize]) -> bool {
        indices.iter().all(|&idx| self.is_channel_open(idx))
    }
}
