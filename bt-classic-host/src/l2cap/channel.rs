//! L2CAP channel state machine and CID allocation.
//!
//! States: Closed → WaitConnectRsp → WaitConfig → Open → WaitDisconnect → Closed
//!
//! Configuration is bidirectional: both sides send ConfigReq and must receive ConfigRsp.
//! We track this with two flags: local_configured and remote_configured.

/// L2CAP channel state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChannelState {
    /// Channel is not in use.
    Closed,
    /// Sent Connection Request, waiting for Connection Response.
    WaitConnectRsp,
    /// Connection accepted, exchanging Configuration Requests/Responses.
    Config,
    /// Channel is open and ready for data transfer.
    Open,
    /// Sent Disconnection Request, waiting for Disconnection Response.
    WaitDisconnect,
}

/// An L2CAP channel.
#[derive(Debug)]
pub struct L2capChannel {
    /// Current channel state.
    pub state: ChannelState,
    /// Our local CID (Source CID in signaling).
    pub local_cid: u16,
    /// Remote CID (Destination CID from Connection Response).
    pub remote_cid: u16,
    /// PSM this channel is connected to.
    pub psm: u16,
    /// Negotiated MTU (minimum of local and remote).
    pub mtu: u16,
    /// The remote's MTU (from their ConfigReq).
    pub remote_mtu: u16,
    /// Whether we have sent our ConfigReq and received ConfigRsp (success).
    pub local_configured: bool,
    /// Whether we have received the remote's ConfigReq and sent ConfigRsp.
    pub remote_configured: bool,
    /// Signaling identifier used for the current pending request.
    pub pending_signal_id: u8,
}

impl L2capChannel {
    pub const fn new() -> Self {
        Self {
            state: ChannelState::Closed,
            local_cid: 0,
            remote_cid: 0,
            psm: 0,
            mtu: 672, // default Classic L2CAP MTU
            remote_mtu: 672,
            local_configured: false,
            remote_configured: false,
            pending_signal_id: 0,
        }
    }

    /// Start opening a channel: set to WaitConnectRsp.
    pub fn start_connect(&mut self, local_cid: u16, psm: u16, signal_id: u8) {
        self.state = ChannelState::WaitConnectRsp;
        self.local_cid = local_cid;
        self.psm = psm;
        self.pending_signal_id = signal_id;
        self.local_configured = false;
        self.remote_configured = false;
    }

    /// Handle Connection Response (success).
    pub fn on_connect_response(&mut self, remote_cid: u16) {
        self.remote_cid = remote_cid;
        self.state = ChannelState::Config;
    }

    /// Handle our Configuration Response received (local side configured).
    pub fn on_local_config_complete(&mut self) {
        self.local_configured = true;
        self.check_open();
    }

    /// Handle remote's Configuration Request received (remote side configured).
    pub fn on_remote_config_complete(&mut self, remote_mtu: u16) {
        self.remote_mtu = remote_mtu;
        self.mtu = self.mtu.min(remote_mtu);
        self.remote_configured = true;
        self.check_open();
    }

    /// Transition to Open if both sides are configured.
    fn check_open(&mut self) {
        if self.local_configured && self.remote_configured && self.state == ChannelState::Config {
            self.state = ChannelState::Open;
        }
    }

    /// Check if channel is open and ready for data.
    pub fn is_open(&self) -> bool {
        self.state == ChannelState::Open
    }

    /// Handle disconnection.
    pub fn close(&mut self) {
        self.state = ChannelState::Closed;
        self.local_cid = 0;
        self.remote_cid = 0;
    }
}

/// L2CAP channel storage with CID allocator.
pub struct ChannelManager<const N: usize> {
    pub channels: [L2capChannel; N],
    /// Next CID to allocate (dynamic range starts at 0x0040).
    next_cid: u16,
    /// Next signaling identifier.
    next_signal_id: u8,
}

impl<const N: usize> ChannelManager<N> {
    /// First dynamic CID per Bluetooth spec.
    const FIRST_DYNAMIC_CID: u16 = 0x0040;

    pub const fn new() -> Self {
        Self {
            channels: [const { L2capChannel::new() }; N],
            next_cid: Self::FIRST_DYNAMIC_CID,
            next_signal_id: 1,
        }
    }

    /// Allocate a new channel for the given PSM.
    ///
    /// Returns the channel index and allocated local CID, or None if full.
    pub fn allocate(&mut self, psm: u16) -> Option<(usize, u16)> {
        // Pre-compute CID and signal ID to avoid borrow conflicts
        let cid = self.next_cid;
        let sig_id = self.next_signal_id();

        for (idx, ch) in self.channels.iter_mut().enumerate() {
            if ch.state == ChannelState::Closed {
                self.next_cid = cid.wrapping_add(1);
                if self.next_cid < Self::FIRST_DYNAMIC_CID {
                    self.next_cid = Self::FIRST_DYNAMIC_CID;
                }
                ch.start_connect(cid, psm, sig_id);
                return Some((idx, cid));
            }
        }
        None
    }

    /// Get the next signaling identifier (wraps 1-255, never 0).
    pub fn next_signal_id(&mut self) -> u8 {
        let id = self.next_signal_id;
        self.next_signal_id = if self.next_signal_id >= 255 {
            1
        } else {
            self.next_signal_id + 1
        };
        id
    }

    /// Find a channel by local CID.
    pub fn find_by_local_cid(&mut self, cid: u16) -> Option<(usize, &mut L2capChannel)> {
        self.channels
            .iter_mut()
            .enumerate()
            .find(|(_, ch)| ch.local_cid == cid && ch.state != ChannelState::Closed)
    }

    /// Find a channel by remote CID.
    pub fn find_by_remote_cid(&mut self, cid: u16) -> Option<(usize, &mut L2capChannel)> {
        self.channels
            .iter_mut()
            .enumerate()
            .find(|(_, ch)| ch.remote_cid == cid && ch.state != ChannelState::Closed)
    }

    /// Find a channel by PSM.
    pub fn find_by_psm(&mut self, psm: u16) -> Option<(usize, &mut L2capChannel)> {
        self.channels
            .iter_mut()
            .enumerate()
            .find(|(_, ch)| ch.psm == psm && ch.state != ChannelState::Closed)
    }
}
