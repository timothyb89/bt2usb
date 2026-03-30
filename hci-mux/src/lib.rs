//! HCI Multiplexer for dual-mode Bluetooth (Classic + BLE).
//!
//! This crate provides a multiplexing layer that sits between a raw HCI transport
//! (e.g., CYW43439) and two Bluetooth stacks (e.g., trouble-host for BLE and
//! bt-classic-host for Classic). It routes HCI events, ACL data, and command
//! responses to the appropriate stack based on packet type and connection handle.
//!
//! # Architecture
//!
//! ```text
//! CYW43 bt_device (raw HCI transport)
//!         │
//!    ┌────┴────┐
//!    │ HciMux  │  ← routes events/ACL by type, serializes commands
//!    └──┬───┬──┘
//!       │   │
//!       │   └──── ClassicController ──→ bt-classic-host
//!       │
//!       └──────── LeController ──────→ trouble-host
//! ```
//!
//! # Usage
//!
//! ```rust,ignore
//! // Create mux from raw transport (e.g., cyw43 bt_device)
//! static MUX_RES: StaticCell<MuxResources> = StaticCell::new();
//! let res = MUX_RES.init(MuxResources::new());
//! let mux = HciMux::new(bt_device, res);
//! let (le_ctrl, classic_ctrl, runner) = mux.split();
//!
//! // le_ctrl: implements bt_hci::controller::Controller — pass to trouble_host::new()
//! // classic_ctrl: implements bt_hci::controller::Controller — pass to bt_classic_host::new()
//! // runner.run(): must be polled concurrently with both stacks
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

mod classify;
mod cmd_slots;
mod handle_registry;

pub use classify::PacketTarget;
pub use handle_registry::HandleRegistry;

use core::cell::RefCell;

use bt_hci::cmd::{self, CmdReturnBuf};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data;
use bt_hci::event::{CommandComplete, CommandCompleteWithStatus, CommandStatus, EventKind};
use bt_hci::param::ConnHandle;
use bt_hci::transport::Transport;
use bt_hci::{ControllerToHostPacket, FixedSizeValue, FromHciBytes, FromHciBytesError};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_io::ErrorType;

use classify::{classify_event, is_le_opcode};
use cmd_slots::CommandSlots;
use handle_registry::StackId;

/// Maximum HCI packet size.
const MAX_HCI_PACKET_LEN: usize = 259;

/// Raw packet buffer for dispatch channels.
pub struct RawPacket {
    pub buf: [u8; MAX_HCI_PACKET_LEN],
    pub len: usize,
}

impl RawPacket {
    const fn new() -> Self {
        Self {
            buf: [0u8; MAX_HCI_PACKET_LEN],
            len: 0,
        }
    }
}

impl Clone for RawPacket {
    fn clone(&self) -> Self {
        let mut buf = [0u8; MAX_HCI_PACKET_LEN];
        buf[..self.len].copy_from_slice(&self.buf[..self.len]);
        Self { buf, len: self.len }
    }
}

/// Resources for the HCI multiplexer. Allocate statically.
pub struct MuxResources<const LE_SLOTS: usize = 10, const CLASSIC_SLOTS: usize = 4> {
    le_slots: CommandSlots<LE_SLOTS>,
    classic_slots: CommandSlots<CLASSIC_SLOTS>,
    le_channel: Channel<CriticalSectionRawMutex, RawPacket, 4>,
    classic_channel: Channel<CriticalSectionRawMutex, RawPacket, 4>,
    handles: RefCell<HandleRegistry<8>>,
}

impl<const LE_SLOTS: usize, const CLASSIC_SLOTS: usize> Default
    for MuxResources<LE_SLOTS, CLASSIC_SLOTS>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const LE_SLOTS: usize, const CLASSIC_SLOTS: usize> MuxResources<LE_SLOTS, CLASSIC_SLOTS> {
    /// Create new mux resources. Call once, store in a StaticCell.
    pub fn new() -> Self {
        Self {
            le_slots: CommandSlots::new(),
            classic_slots: CommandSlots::new(),
            le_channel: Channel::new(),
            classic_channel: Channel::new(),
            handles: RefCell::new(HandleRegistry::new()),
        }
    }
}

/// The HCI multiplexer. Call [`split()`](HciMux::split) to obtain virtual controllers.
///
/// Must live for the duration of the program (on the stack of a `-> !` task).
/// All components returned by `split()` borrow from this struct.
pub struct HciMux<'a, T, const LE_SLOTS: usize = 10, const CLASSIC_SLOTS: usize = 4> {
    /// The raw HCI transport. Transport trait takes &self, so concurrent
    /// read/write is handled internally by the transport implementation.
    /// Only the MuxRunner reads; virtual controllers only write.
    transport: T,
    res: &'a MuxResources<LE_SLOTS, CLASSIC_SLOTS>,
}

impl<'a, T: Transport, const LS: usize, const CS: usize> HciMux<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    /// Create a new mux wrapping the given transport.
    pub fn new(transport: T, resources: &'a MuxResources<LS, CS>) -> Self {
        Self {
            transport,
            res: resources,
        }
    }

    /// Split into the two virtual controllers and a runner.
    ///
    /// All three components borrow from `self`, so the `HciMux` must outlive them
    /// (lives on the stack of the `-> !` main task).
    pub fn split(
        &'a self,
    ) -> (
        LeController<'a, T, LS, CS>,
        ClassicController<'a, T, LS, CS>,
        MuxRunner<'a, T, LS, CS>,
    ) {
        let le = LeController {
            mux: self,
            res: self.res,
        };
        let classic = ClassicController {
            mux: self,
            res: self.res,
        };
        let runner = MuxRunner {
            mux: self,
            res: self.res,
        };
        (le, classic, runner)
    }
}

/// Internal dispatch decision made by the runner.
enum DispatchAction {
    /// Send to LE stack only.
    Le(usize),
    /// Send to Classic stack only.
    Classic(usize),
    /// Send to both stacks.
    Both(usize),
    /// Command response already handled via slots.complete().
    CmdComplete { is_le: bool },
    /// Drop the packet.
    Drop,
}

/// The mux runner. Reads from the real transport and dispatches to virtual controllers.
///
/// Must be polled concurrently with both BT stacks.
pub struct MuxRunner<'a, T, const LS: usize = 10, const CS: usize = 4> {
    mux: &'a HciMux<'a, T, LS, CS>,
    res: &'a MuxResources<LS, CS>,
}

impl<'a, T: Transport, const LS: usize, const CS: usize> MuxRunner<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    /// Run the mux event loop. This never returns under normal operation.
    pub async fn run(&self) -> Result<(), T::Error> {
        let mut rx = [0u8; MAX_HCI_PACKET_LEN];
        loop {
            // Read the next packet from the real transport.
            // We use the same unsafe reborrow pattern as ExternalController to avoid
            // holding references to rx across dispatch calls.
            let rx_ref = unsafe { core::slice::from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };
            let transport = &self.mux.transport;
            let result = transport.read(rx_ref).await;

            // Determine the dispatch action without holding a borrow on rx.
            let action = match result {
                Ok(ControllerToHostPacket::Event(ref event)) => self.classify_event_action(event),
                Ok(ControllerToHostPacket::Acl(ref acl)) => {
                    let handle = acl.handle();
                    // ACL: 1 (type indicator 0x02) + 2 (handle) + 2 (data_len) + data
                    let total_len = 5 + acl.data().len();
                    let handles = self.res.handles.borrow();
                    let target = handles.lookup(handle);
                    drop(handles);
                    match target {
                        Some(StackId::Le) => DispatchAction::Le(total_len),
                        Some(StackId::Classic) => DispatchAction::Classic(total_len),
                        None => DispatchAction::Drop,
                    }
                }
                Ok(_) => DispatchAction::Drop,
                Err(e) => return Err(e),
            };

            // Now dispatch (copies from rx, so no borrow conflict)
            match action {
                DispatchAction::Le(len) => self.dispatch_le(&rx, len).await,
                DispatchAction::Classic(len) => self.dispatch_classic(&rx, len).await,
                DispatchAction::Both(len) => self.dispatch_to_both(&rx, len).await,
                DispatchAction::CmdComplete { is_le } => {
                    // Already handled by classify_event_action via slots.complete()
                    let _ = is_le;
                }
                DispatchAction::Drop => {}
            }
        }
    }

    /// Classify an event and determine the dispatch action.
    /// For command responses, this also calls complete() on the appropriate slots.
    fn classify_event_action(&self, event: &bt_hci::event::EventPacket<'_>) -> DispatchAction {
        // Event: 1 (type indicator 0x04) + 1 (event code) + 1 (param_len) + data
        let total_len = event.data.len() + 3;

        match event.kind {
            // Command responses: route by opcode, complete the slot
            EventKind::CommandComplete => {
                if let Ok(e) = CommandComplete::from_hci_bytes_complete(event.data) {
                    if e.has_status() {
                        if let Ok(e) = TryInto::<CommandCompleteWithStatus>::try_into(e) {
                            let is_le = is_le_opcode(e.cmd_opcode.to_raw());
                            if is_le {
                                self.res.le_slots.complete(
                                    e.cmd_opcode,
                                    e.status,
                                    e.num_hci_cmd_pkts as usize,
                                    e.return_param_bytes.as_ref(),
                                );
                            } else {
                                self.res.classic_slots.complete(
                                    e.cmd_opcode,
                                    e.status,
                                    e.num_hci_cmd_pkts as usize,
                                    e.return_param_bytes.as_ref(),
                                );
                            }
                            return DispatchAction::CmdComplete { is_le };
                        }
                    }
                }
                DispatchAction::Both(total_len)
            }
            EventKind::CommandStatus => {
                if let Ok(e) = CommandStatus::from_hci_bytes_complete(event.data) {
                    let is_le = is_le_opcode(e.cmd_opcode.to_raw());
                    if is_le {
                        self.res.le_slots.complete(
                            e.cmd_opcode,
                            e.status,
                            e.num_hci_cmd_pkts as usize,
                            &[],
                        );
                    } else {
                        self.res.classic_slots.complete(
                            e.cmd_opcode,
                            e.status,
                            e.num_hci_cmd_pkts as usize,
                            &[],
                        );
                    }
                    return DispatchAction::CmdComplete { is_le };
                }
                DispatchAction::Both(total_len)
            }

            // Handle-based events
            EventKind::DisconnectionComplete
            | EventKind::EncryptionChangeV1
            | EventKind::EncryptionChangeV2
            | EventKind::EncryptionKeyRefreshComplete => {
                if event.data.len() >= 3 {
                    let handle_raw = u16::from_le_bytes([event.data[1], event.data[2]]) & 0x0FFF;
                    let handle = ConnHandle::new(handle_raw);
                    if event.kind == EventKind::DisconnectionComplete {
                        self.res.handles.borrow_mut().unregister(handle);
                    }
                    let handles = self.res.handles.borrow();
                    match handles.lookup(handle) {
                        Some(StackId::Le) => DispatchAction::Le(total_len),
                        Some(StackId::Classic) => DispatchAction::Classic(total_len),
                        None => DispatchAction::Both(total_len),
                    }
                } else {
                    DispatchAction::Both(total_len)
                }
            }

            EventKind::NumberOfCompletedPackets => DispatchAction::Both(total_len),

            // LE connection events — register handle
            EventKind::Le => {
                if event.data.len() >= 5 {
                    let sub_event = event.data[0];
                    if sub_event == 0x01 || sub_event == 0x0A {
                        let status = event.data[1];
                        if status == 0x00 {
                            let handle_raw =
                                u16::from_le_bytes([event.data[2], event.data[3]]) & 0x0FFF;
                            self.res
                                .handles
                                .borrow_mut()
                                .register(ConnHandle::new(handle_raw), StackId::Le);
                        }
                    }
                }
                DispatchAction::Le(total_len)
            }

            // Classic connection events — register handle
            EventKind::ConnectionComplete => {
                if event.data.len() >= 3 {
                    let status = event.data[0];
                    if status == 0x00 {
                        let handle_raw =
                            u16::from_le_bytes([event.data[1], event.data[2]]) & 0x0FFF;
                        self.res
                            .handles
                            .borrow_mut()
                            .register(ConnHandle::new(handle_raw), StackId::Classic);
                    }
                }
                DispatchAction::Classic(total_len)
            }

            // All other events: classify by type
            other => {
                let target = classify_event(other);
                match target {
                    PacketTarget::Le => DispatchAction::Le(total_len),
                    PacketTarget::Classic => DispatchAction::Classic(total_len),
                    PacketTarget::Both | PacketTarget::ByHandle(_) => {
                        DispatchAction::Both(total_len)
                    }
                }
            }
        }
    }

    async fn dispatch_le(&self, buf: &[u8], len: usize) {
        let mut pkt = RawPacket::new();
        let copy_len = len.min(MAX_HCI_PACKET_LEN);
        pkt.buf[..copy_len].copy_from_slice(&buf[..copy_len]);
        pkt.len = copy_len;
        self.res.le_channel.send(pkt).await;
    }

    async fn dispatch_classic(&self, buf: &[u8], len: usize) {
        let mut pkt = RawPacket::new();
        let copy_len = len.min(MAX_HCI_PACKET_LEN);
        pkt.buf[..copy_len].copy_from_slice(&buf[..copy_len]);
        pkt.len = copy_len;
        self.res.classic_channel.send(pkt).await;
    }

    async fn dispatch_to_both(&self, buf: &[u8], len: usize) {
        let mut pkt = RawPacket::new();
        let copy_len = len.min(MAX_HCI_PACKET_LEN);
        pkt.buf[..copy_len].copy_from_slice(&buf[..copy_len]);
        pkt.len = copy_len;
        // Send to LE first, then Classic
        self.res.le_channel.send(pkt.clone()).await;
        self.res.classic_channel.send(pkt).await;
    }
}

/// Virtual controller for the LE (BLE) stack.
///
/// Implements `bt_hci::controller::Controller` — pass to `trouble_host::new()`.
pub struct LeController<'a, T, const LS: usize = 10, const CS: usize = 4> {
    mux: &'a HciMux<'a, T, LS, CS>,
    res: &'a MuxResources<LS, CS>,
}

impl<'a, T, const LS: usize, const CS: usize> ErrorType for LeController<'a, T, LS, CS>
where
    T: ErrorType,
{
    type Error = T::Error;
}

impl<'a, T: Transport, const LS: usize, const CS: usize> bt_hci::controller::Controller
    for LeController<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    async fn write_acl_data(&self, packet: &data::AclPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn write_sync_data(&self, packet: &data::SyncPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn write_iso_data(&self, packet: &data::IsoPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn read<'b>(&self, buf: &'b mut [u8]) -> Result<ControllerToHostPacket<'b>, Self::Error> {
        let pkt = self.res.le_channel.receive().await;
        let len = pkt.len.min(buf.len());
        buf[..len].copy_from_slice(&pkt.buf[..len]);
        // The channel only receives well-formed packets from the mux runner.
        // If parsing fails, return a synthetic error rather than looping.
        ControllerToHostPacket::from_hci_bytes_complete(&buf[..len]).map_err(T::Error::from)
    }
}

impl<'a, T: Transport, C: cmd::SyncCmd, const LS: usize, const CS: usize> ControllerCmdSync<C>
    for LeController<'a, T, LS, CS>
where
    C::Return: FixedSizeValue,
    T::Error: From<FromHciBytesError>,
{
    async fn exec(&self, cmd: &C) -> Result<C::Return, cmd::Error<Self::Error>> {
        let mut retval: C::ReturnBuf = CmdReturnBuf::new();
        let (signal, idx) = self.res.le_slots.acquire(C::OPCODE, retval.as_mut()).await;

        let transport = &self.mux.transport;
        let write_result = transport.write(cmd).await;

        if let Err(e) = write_result {
            self.res.le_slots.release(idx);
            return Err(cmd::Error::Io(e));
        }

        let resp = signal.wait().await;
        self.res.le_slots.release(idx);

        let return_param_bytes =
            bt_hci::param::RemainingBytes::from_hci_bytes_complete(&retval.as_ref()[..resp.len])
                .unwrap();
        let e = CommandCompleteWithStatus {
            num_hci_cmd_pkts: 0,
            status: resp.status,
            cmd_opcode: C::OPCODE,
            return_param_bytes,
        };
        let r = e.to_result::<C>().map_err(cmd::Error::Hci)?;
        Ok(r)
    }
}

impl<'a, T: Transport, C: cmd::AsyncCmd, const LS: usize, const CS: usize> ControllerCmdAsync<C>
    for LeController<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    async fn exec(&self, cmd: &C) -> Result<(), cmd::Error<Self::Error>> {
        let (signal, idx) = self.res.le_slots.acquire(C::OPCODE, &mut []).await;

        let transport = &self.mux.transport;
        let write_result = transport.write(cmd).await;

        if let Err(e) = write_result {
            self.res.le_slots.release(idx);
            return Err(cmd::Error::Io(e));
        }

        let resp = signal.wait().await;
        self.res.le_slots.release(idx);

        resp.status.to_result()?;
        Ok(())
    }
}

/// Virtual controller for the Classic (BR/EDR) stack.
///
/// Implements `bt_hci::controller::Controller` — pass to `bt_classic_host::new()`.
pub struct ClassicController<'a, T, const LS: usize = 10, const CS: usize = 4> {
    mux: &'a HciMux<'a, T, LS, CS>,
    res: &'a MuxResources<LS, CS>,
}

impl<'a, T, const LS: usize, const CS: usize> ErrorType for ClassicController<'a, T, LS, CS>
where
    T: ErrorType,
{
    type Error = T::Error;
}

impl<'a, T: Transport, const LS: usize, const CS: usize> bt_hci::controller::Controller
    for ClassicController<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    async fn write_acl_data(&self, packet: &data::AclPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn write_sync_data(&self, packet: &data::SyncPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn write_iso_data(&self, packet: &data::IsoPacket<'_>) -> Result<(), Self::Error> {
        let transport = &self.mux.transport;
        transport.write(packet).await?;
        Ok(())
    }

    async fn read<'b>(&self, buf: &'b mut [u8]) -> Result<ControllerToHostPacket<'b>, Self::Error> {
        let pkt = self.res.classic_channel.receive().await;
        let len = pkt.len.min(buf.len());
        buf[..len].copy_from_slice(&pkt.buf[..len]);
        ControllerToHostPacket::from_hci_bytes_complete(&buf[..len]).map_err(T::Error::from)
    }
}

impl<'a, T: Transport, C: cmd::SyncCmd, const LS: usize, const CS: usize> ControllerCmdSync<C>
    for ClassicController<'a, T, LS, CS>
where
    C::Return: FixedSizeValue,
    T::Error: From<FromHciBytesError>,
{
    async fn exec(&self, cmd: &C) -> Result<C::Return, cmd::Error<Self::Error>> {
        let mut retval: C::ReturnBuf = CmdReturnBuf::new();
        let (signal, idx) = self
            .res
            .classic_slots
            .acquire(C::OPCODE, retval.as_mut())
            .await;

        let transport = &self.mux.transport;
        let write_result = transport.write(cmd).await;

        if let Err(e) = write_result {
            self.res.classic_slots.release(idx);
            return Err(cmd::Error::Io(e));
        }

        let resp = signal.wait().await;
        self.res.classic_slots.release(idx);

        let return_param_bytes =
            bt_hci::param::RemainingBytes::from_hci_bytes_complete(&retval.as_ref()[..resp.len])
                .unwrap();
        let e = CommandCompleteWithStatus {
            num_hci_cmd_pkts: 0,
            status: resp.status,
            cmd_opcode: C::OPCODE,
            return_param_bytes,
        };
        let r = e.to_result::<C>().map_err(cmd::Error::Hci)?;
        Ok(r)
    }
}

impl<'a, T: Transport, C: cmd::AsyncCmd, const LS: usize, const CS: usize> ControllerCmdAsync<C>
    for ClassicController<'a, T, LS, CS>
where
    T::Error: From<FromHciBytesError>,
{
    async fn exec(&self, cmd: &C) -> Result<(), cmd::Error<Self::Error>> {
        let (signal, idx) = self.res.classic_slots.acquire(C::OPCODE, &mut []).await;

        let transport = &self.mux.transport;
        let write_result = transport.write(cmd).await;

        if let Err(e) = write_result {
            self.res.classic_slots.release(idx);
            return Err(cmd::Error::Io(e));
        }

        let resp = signal.wait().await;
        self.res.classic_slots.release(idx);

        resp.status.to_result()?;
        Ok(())
    }
}
