# hci-mux

An HCI multiplexer that allows a Bluetooth Classic host stack and a BLE host
stack to share a single physical HCI controller concurrently.

## Overview

Most embedded Bluetooth controllers (e.g. the CYW43439 on the Raspberry Pi
Pico W) expose a single HCI transport that can carry both Classic (BR/EDR) and
LE traffic. This crate wraps that transport and presents two virtual
controllers — one for each stack — with correct routing of commands, events,
and ACL data between them.

```
Physical HCI transport (e.g. CYW43)
           │
      ┌────┴────┐
      │ HciMux  │  serialises commands, routes events/ACL by type and handle
      └──┬───┬──┘
         │   │
LeController  ClassicController
    │              │
trouble-host   bt-classic-host
 (or any BLE)  (or any Classic)
```

## Usage

### 1. Allocate resources (once, in a `StaticCell`)

```rust
use hci_mux::{HciMux, MuxResources};
use static_cell::StaticCell;

// LE_SLOTS = max concurrent pending LE commands (default 10)
// CLASSIC_SLOTS = max concurrent pending Classic commands (default 4)
static MUX_RESOURCES: StaticCell<MuxResources> = StaticCell::new();
let resources = MUX_RESOURCES.init(MuxResources::new());
```

### 2. Split the transport

```rust
let mux = HciMux::new(transport, resources);
let (le_controller, classic_controller, mut runner) = mux.split();
```

`le_controller` implements `bt_hci::controller::Controller` and can be passed
directly to `trouble_host::new()`. `classic_controller` implements the same
trait and can be passed to `bt_classic_host::ClassicRunner::new()`.

### 3. Run the mux

The `MuxRunner` must be polled continuously alongside both stacks. With
Embassy, spawn it as a dedicated task:

```rust
#[embassy_executor::task]
async fn mux_task(mut runner: MuxRunner<'static, impl Transport>) {
    runner.run().await;
}

spawner.spawn(mux_task(runner)).unwrap();
```

`runner.run()` never returns under normal operation.

### Feature flags

| Feature | Description |
|---------|-------------|
| `defmt` | Enable `defmt` structured logging |

## Architecture

### Command routing

Each virtual controller serialises its callers behind a semaphore-guarded slot
pool (`CommandSlots`). When a command is sent, the caller's opcode and response
buffer pointer are stored in a free slot. On `CommandComplete`/`CommandStatus`,
the runner signals both slot pools; only the one holding that opcode reacts.
This prevents responses from being delivered to the wrong stack even when both
issue commands simultaneously.

Reset commands are handled specially: all pending slots for both stacks are
cancelled with `CONTROLLER_BUSY`.

### Event routing

Events are classified into four categories (`PacketTarget`):

| Category | Examples | Action |
|----------|----------|--------|
| `Le` | LE Meta Events | Sent to LE stack only |
| `Classic` | ConnectionComplete, LinkKeyRequest | Sent to Classic stack only |
| `ByHandle` | DisconnectionComplete, EncryptionChange | Looked up in handle registry, sent to owning stack |
| `Both` | CommandComplete, NumberOfCompletedPackets | Broadcast to both stacks |

### ACL routing

ACL packets carry a 12-bit connection handle in their header. The handle
registry (populated on every `ConnectionComplete` event) maps each handle to
its owning stack. Incoming ACL data is forwarded to that stack's channel.

## Sizing

`MuxResources` has two const generic parameters:

```rust
pub struct MuxResources<const LE_SLOTS: usize = 10, const CLASSIC_SLOTS: usize = 4>
```

Increase `LE_SLOTS` if your BLE stack issues many simultaneous commands.
Increase `CLASSIC_SLOTS` if your Classic stack does the same. If all slots are
occupied a caller will block in `acquire()` until one is released.

The handle registry is fixed at 8 concurrent ACL connections. `register()`
returns `false` (and the connection is not tracked) if this limit is exceeded.

## Limitations

- **Max HCI packet size**: Hard-coded at 259 bytes. Larger packets are not
  supported.
- **Dispatch channel depth**: 8 packets per stack. If a stack is slow to drain
  its channel, `try_send()` will silently drop the overflow for broadcast
  events.
- **No credit-based ACL flow control**: The mux forwards ACL data without
  back-pressure beyond what the Embassy channel provides.
- **Handle registry limit**: At most 8 concurrent ACL connections are tracked.
  Connections beyond this limit receive events from the wrong stack or not at
  all.
- **Unsafe reborrow pattern**: The runner borrows the transport for each I/O
  call using an unsafe reborrow to avoid holding a reference across dispatch
  points. This is modelled after the `ExternalController` pattern from `bt-hci`
  and is sound in practice, but is not expressed in safe Rust.
- **No support for multiple physical controllers**: Designed for a single
  controller chip exposing one HCI transport.
