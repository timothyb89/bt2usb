//! HCI Link Policy commands for Classic Bluetooth.
//!
//! These commands are not defined in bt-hci 0.8's cmd module, so we define
//! them here using the same `cmd!` macro pattern.

use bt_hci::cmd;
use bt_hci::param::ConnHandle;

// OGF 0x02 = LINK_POLICY

cmd! {
    /// Write Link Policy Settings (OGF=0x02, OCF=0x000D)
    ///
    /// Configures which link modes the local controller is allowed to use.
    /// link_policy_settings bitmask: bit 0=Role Switch, bit 1=Hold, bit 2=Sniff, bit 3=Park.
    WriteLinkPolicySettings(LINK_POLICY, 0x000d) {
        WriteLinkPolicySettingsParams {
            handle: ConnHandle,
            link_policy_settings: u16,
        }
        WriteLinkPolicySettingsReturn {
            handle: ConnHandle,
        }
    }
}

cmd! {
    /// Sniff Mode (OGF=0x02, OCF=0x0003)
    ///
    /// Requests Sniff mode for the given connection. The controller polls the
    /// remote device at the specified interval (units of 0.625ms).
    /// Returns CommandStatus; ModeChange event follows when sniff is entered.
    SniffMode(LINK_POLICY, 0x0003) {
        SniffModeParams {
            handle: ConnHandle,
            sniff_max_interval: u16,
            sniff_min_interval: u16,
            sniff_attempt: u16,
            sniff_timeout: u16,
        }
        Return = ();
    }
}

cmd! {
    /// Sniff Subrating (OGF=0x02, OCF=0x0011)
    ///
    /// Layered on top of Sniff mode. Lets the link manager extend the effective
    /// sniff interval when no data is flowing, and collapse back to the base
    /// sniff interval as soon as traffic resumes — without a full mode change.
    /// The peer peripheral can sleep its radio for up to `max_latency` slots
    /// (units of 0.625ms) during idle, which meaningfully improves HID device
    /// battery life.
    ///
    /// `min_remote_timeout` / `min_local_timeout` are the minimum number of
    /// slots the remote/local device must listen after receiving a packet
    /// before re-entering subrated sniff. Zero is a valid "no extra wait"
    /// value.
    SniffSubrating(LINK_POLICY, 0x0011) {
        SniffSubratingParams {
            handle: ConnHandle,
            max_latency: u16,
            min_remote_timeout: u16,
            min_local_timeout: u16,
        }
        SniffSubratingReturn {
            handle: ConnHandle,
        }
    }
}

// OGF 0x03 = HOST_CONTROLLER_BASEBAND

cmd! {
    /// Write Inquiry Mode (OGF=0x03, OCF=0x0045)
    ///
    /// Sets the Inquiry Result format: 0=standard (no RSSI/name),
    /// 1=with RSSI, 2=Extended Inquiry Result (RSSI + EIR data with name).
    /// Mode 2 is required to get device names during Classic Inquiry.
    WriteInquiryMode(CONTROL_BASEBAND, 0x0045) {
        WriteInquiryModeParams {
            inquiry_mode: u8,
        }
        Return = ();
    }
}

cmd! {
    /// Write Simple Pairing Mode (OGF=0x03, OCF=0x0056)
    ///
    /// Enables Secure Simple Pairing on the controller. Must be set to 1 before
    /// the controller will emit Extended Inquiry Result events, even if the
    /// inquiry mode has been set to 2 via WriteInquiryMode.
    WriteSimplePairingMode(CONTROL_BASEBAND, 0x0056) {
        WriteSimplePairingModeParams {
            simple_pairing_mode: u8,
        }
        Return = ();
    }
}

/// Link policy setting bits.
pub const LINK_POLICY_ENABLE_SNIFF: u16 = 0x0004;
