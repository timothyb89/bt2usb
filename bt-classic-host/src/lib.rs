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
//! // Allocate resources statically
//! static RESOURCES: StaticCell<HostResources<1, 3>> = StaticCell::new();
//!
//! // Build the stack
//! let stack = bt_classic_host::new(controller, resources);
//! let Host { runner, central } = stack.build();
//!
//! // Run the event loop concurrently with application code
//! join(runner.run(), async {
//!     let conn = central.connect(&addr).await?;
//!     conn.wait_encrypted().await?;
//!     let hid = HidClient::new(&conn).await?;
//!     loop {
//!         let report = hid.receive_report().await?;
//!         // process report...
//!     }
//! }).await;
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

pub use error::Error;
pub use link_key::{LinkKeyInfo, LinkKeyStore};

// TODO: Public API types (HostResources, new(), ClassicStack, Host, Central, HidClient)
// will be added as the implementation progresses through Phases 2-4.
