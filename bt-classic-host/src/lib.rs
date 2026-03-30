//! Bluetooth Classic HID Host stack for embedded systems.
//!
//! This crate provides a `no_std` async Bluetooth Classic host implementation
//! focused on the HID Profile (HIDP). It connects to Classic Bluetooth HID
//! devices (keyboards, mice, trackpads) and receives HID reports over L2CAP.
//!
//! # Architecture
//!
//! The stack implements three protocol layers:
//!
//! - **HCI**: Connection management, pairing (SSP + legacy), encryption
//! - **L2CAP**: Classic connection-oriented channels (PSM 0x0011 control, PSM 0x0013 interrupt)
//! - **HIDP**: HID Profile message framing, report reception, SET/GET_REPORT
//!
//! # Usage
//!
//! ```rust,ignore
//! static RESOURCES: StaticCell<HostResources<1>> = StaticCell::new();
//!
//! let res = RESOURCES.init(HostResources::new());
//! let link_keys = RefCell::new(MyLinkKeyStore::new());
//! let mut runner = ClassicRunner::new(&controller, res, &link_keys);
//!
//! runner.init().await?;
//! let handle = runner.connect(&target_addr).await?;
//! // Connection is now encrypted, ready for L2CAP/HIDP
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

pub mod connection;
pub mod error;
pub mod hidp;
pub mod host;
pub mod l2cap;
pub mod link_key;
pub mod pairing;

pub use connection::{ClassicConnection, ConnState};
pub use error::Error;
pub use hidp::{HidClient, HidReport};
pub use host::{ClassicRunner, ConnEvent, HostResources};
pub use l2cap::L2capState;
pub use link_key::{LinkKeyInfo, LinkKeyStore};
pub use pairing::PairingContext;
