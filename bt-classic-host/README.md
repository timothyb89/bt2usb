# bt-classic-host

A `no_std` async Bluetooth Classic HID host stack for embedded systems.

Implements the protocol layers needed to connect to, pair with, and receive HID
input from Bluetooth Classic (BR/EDR) HID devices such as keyboards and mice.

## Protocol layers

```
HIDP (HID Profile over L2CAP)
  PSM 0x0011 — Control channel
  PSM 0x0013 — Interrupt channel
          │
L2CAP (Logical Link Control and Adaptation Protocol)
  Connection-oriented channels
  Fragment reassembly
  Signaling: Connection / Configuration / Disconnection
          │
HCI (Host Controller Interface)
  Connection management
  Pairing: Secure Simple Pairing (SSP) + legacy PIN
  Encryption
```

## Usage

### 1. Implement `LinkKeyStore`

Provide a persistent backing store for bonding keys so paired devices can
reconnect without re-pairing:

```rust
use bt_classic_host::{LinkKeyInfo, LinkKeyStore};
use bt_hci::param::BdAddr;

struct MyKeyStore { /* flash, EEPROM, etc. */ }

impl LinkKeyStore for MyKeyStore {
    fn load(&self, addr: &BdAddr) -> Option<LinkKeyInfo> { ... }
    fn store(&mut self, addr: &BdAddr, key: LinkKeyInfo) { ... }
    fn remove(&mut self, addr: &BdAddr) { ... }
}
```

Keys are 128-bit symmetric values exchanged during pairing. Storing them
persistently enables bonded reconnection.

### 2. Allocate resources

```rust
use bt_classic_host::HostResources;
use static_cell::StaticCell;

// CONNS = max concurrent Classic ACL connections
static RESOURCES: StaticCell<HostResources<1>> = StaticCell::new();
let resources = RESOURCES.init(HostResources::new());
```

### 3. Create the runner and connect

```rust
use bt_classic_host::ClassicRunner;

let mut runner = ClassicRunner::new(classic_controller, &mut key_store, resources);

// Initiate a connection; blocks until the link is encrypted and ready.
// Returns a handle for the active ACL connection.
let conn = runner.connect(peer_addr).await?;
```

The `connect()` call handles the full setup sequence internally:
1. Sends `CreateConnection`
2. Replies to `LinkKeyRequest` (uses stored key or triggers pairing)
3. Runs SSP handshake (NoInputNoOutput "Just Works") or legacy PIN pairing
4. Requests authentication (`AuthenticationRequested`)
5. Enables encryption (`SetConnectionEncryption`)
6. Returns once `EncryptionChange` confirms the link is encrypted

### 4. Open HIDP channels and receive reports

```rust
use bt_classic_host::{HidClient, L2capState};

let mut l2cap = L2capState::new(conn);
let mut hid   = HidClient::new();

// Open the HID control and interrupt L2CAP channels.
hid.open_channels(&conn, &mut l2cap).await?;

// Wait for both channels to complete L2CAP configuration.
while !hid.is_ready(&l2cap) {
    runner.process_acl(&conn, &mut l2cap).await?;
}

// Set Report Protocol mode (required by most HID devices).
hid.set_protocol_report(&conn, &mut l2cap).await?;

// Main input loop.
loop {
    let acl = runner.read_acl(&conn).await?;
    if let Some(report) = l2cap.process_acl(&acl, &mut hid)? {
        // report.data[..report.len] contains the raw HID input report
    }
}
```

### 5. Send output reports (e.g. LEDs)

```rust
// SET_REPORT on the control channel
hid.set_report(&conn, &mut l2cap, ReportType::Output, &[0x01]).await?;
```

### Feature flags

| Feature | Description |
|---------|-------------|
| `defmt` | Enable `defmt` structured logging |

## Connection lifecycle

```
Idle
  → Connecting      (CreateConnection sent)
  → Connected       (ConnectionComplete received, handle assigned)
  → Authenticating  (AuthenticationRequested sent)
  → Encrypting      (SetConnectionEncryption sent)
  → Encrypted       (EncryptionChange confirmed — link is ready)
```

`ClassicConnection::is_ready()` returns `true` only in the `Encrypted` state.

## Pairing model

The stack always advertises `NoInputNoOutput` IO capability, which selects
"Just Works" SSP for all peers that support SSP. User confirmation requests
are accepted automatically without numeric comparison.

For peers that fall back to legacy pairing, the stack responds with the PIN
`"0000"`. This will only succeed if the device accepts that PIN.

Neither passkey entry, numeric comparison, nor OOB pairing is supported.

## Limitations

- **"Just Works" only**: The `NoInputNoOutput` IO capability means no MITM
  protection. Passkey entry, numeric comparison, and OOB methods are not
  implemented.
- **Legacy PIN is always `"0000"`**: Devices requiring a different PIN will
  fail to pair.
- **Single connection per `ClassicRunner`**: `connect()` manages one slot at a
  time. Running multiple concurrent connections requires multiple runner
  instances and manual coordination.
- **Basic L2CAP mode only**: Extended flow control and segmentation (ERTM/
  streaming mode) are not implemented. Devices that require these modes are not
  supported.
- **Reassembly buffer capped at 1024 bytes**: L2CAP frames larger than 1024
  bytes are dropped during reassembly.
- **Fixed-size channel table**: The number of L2CAP channels per connection is
  fixed at compile time. The allocator wraps around if exceeded.
- **No outbound L2CAP connection requests from the peer**: The stack only
  initiates L2CAP connections; it cannot accept inbound connection requests
  from the remote device.
- **No `GET_REPORT` response handling**: Responses to `GET_REPORT` on the
  control channel are logged but not returned to the caller.
- **ACL data arriving before encryption is dropped**: Any ACL packets that
  arrive during the HCI connection setup phase are silently discarded rather
  than buffered for later delivery to L2CAP.
- **No sniff-mode or role-switch negotiation**: Link policy commands are
  defined in `link_policy.rs` but are not automatically issued. Power
  management is left to the application.
