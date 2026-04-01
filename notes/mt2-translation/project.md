# magic trackpad translation

## Context

The bt2usb project is designed to emulate a USB device on behalf of a Bluetooth
device. This is useful for compatibility purposes or for quickly switching
devices between machines with e.g. a USB switch.

Currently, this project primarily targets BLE mouse-like devices. We'd like to
extend compatibility for connecting a Magic Trackpad 2 or 3 via Bluetooth and
having it appear as a compatible device to the host system.

## Goal

Regardless of host system, the connected trackpad should function as if it were
a native device.

- On Windows: it should act as close to a "Precision" trackpad as possible.
  Multitouch and native Windows gestures should all work.

  Force feedback should not be passed through unless there is a driverless
  way of doing so. The expectation is that we will need to emulate force
  feedback ourselves. Hopefully, multitouch gestures can be otherwise
  interpreted by the host OS.
- On macOS and Linux: we should pass through/translate bidirectional events.
  These hosts have robust drivers for the real Magic Trackpad so no force
  feedback emulation should be needed.

## Reference material

- Our original Magic Trackpad emulation work for the Full Scroll Dial
   - Notes in `notes/magic-mouse/project.md`
   - Custom tooling, logs, data captures are in `notes/magic-mouse/`
   - "Full" build before simplification remains in `mt2-emulation` git branch
- Linux kernel `hid-magicmouse.c` is in notes/magic-mouse/
- Ploopy trackpad acts as a driverless, generic Windows trackpad with full
  gesture support. I have a physical unit and can capture data as needed.

## Key protocol findings

### CRITICAL: Magic Trackpad 2 uses Bluetooth Classic, NOT BLE

Confirmed 2026-03-30 via live testing on Linux:

```
hcitool con → ACL (Classic), not LE
bluetoothctl info → UUID 0x1124 (SDP HID Profile), not 0x1812 (BLE HID)
busctl tree → No GATT service objects under device
Modalias: bluetooth:v004Cp0265d0318
```

The MT2 connects via **Bluetooth Classic HID Profile (HIDP over L2CAP)**, not
BLE HOGP (HID over GATT). This means:
- The existing trouble-host BLE stack **cannot** connect to the MT2
- We need Classic Bluetooth support: L2CAP, SDP, HIDP protocol layers
- The CYW43439 chip supports Classic BT (BR/EDR) at the HCI level
- No existing Rust library provides Classic BT HID host functionality
- bleak (BLE-only) cannot discover MT2 HID services — they don't exist on GATT

The kernel driver hid-magicmouse.c distinguishes BT vs USB by vendor ID
(`BT_VENDOR_ID_APPLE` vs `USB_VENDOR_ID_APPLE`), not by BLE vs Classic.
The "BT" report format (Report ID 0x31, 4-byte header) is used over Classic
BT HIDP, not BLE GATT.

### BT Classic vs USB report format (from hid-magicmouse.c)

Touch point encoding is **identical** between BT and USB — 9 bytes per point
with 13-bit packed X/Y. Only headers differ:

- BT (Classic HIDP): Report ID `0x31`, 4-byte header, touches at offset 4
- USB: Report ID `0x02`, 12-byte header, touches at offset 12
- MT enable: BT `SET_REPORT(0xF1, {0x02, 0x01})`, USB `SET_REPORT(0x02, {0x01})`
- Double-report packing: BT Report ID `0xF7` wraps two reports

### Touch point fields (per 9-byte tdata)

- `id = tdata[8] & 0xF`
- `x = (tdata[1] << 27 | tdata[0] << 19) >> 19` (13-bit signed)
- `y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19)` (13-bit signed)
- `pressure = tdata[7]`, `state = tdata[3] & 0xC0` (0x80 = down)
- `touch_major = tdata[4]`, `touch_minor = tdata[5]`, `size = tdata[6]`
- `orientation = (tdata[8] >> 5) - 4`

### MT2 physical surface dimensions

- X: [-3678, +3934] (160mm)
- Y: [-2478, +2587] (114.9mm)

### MT3 (USB-C variant)

Same protocol, BT version `0x314` vs MT2's `0x110`. Kernel handles identically.

### Haptic feedback (critical)

The MT2 has **zero physical actuation**. All click feedback comes from the
Taptic Engine, driven by host commands. Actuator protocol must be
reverse-engineered — it is essential for Windows UX and must be forwarded
for macOS/Linux passthrough.

## Architecture implications

The Classic BT finding means the BLE pipeline (trouble-host, GATT, HOGP)
cannot be used for the MT2. Instead we need a Classic BT HID host stack:

**Protocol stack required:**
```
Application (HID reports)
        ↓
    HIDP (HID Profile) — HID report framing over L2CAP
        ↓
    L2CAP — Logical link layer (PSM 0x0011 control, 0x0013 interrupt)
        ↓
    HCI — Host Controller Interface (ACL data, commands/events)
        ↓
    CYW43439 firmware (baseband, link manager)
```

**Options under consideration:**
1. Implement Classic BT L2CAP + HIDP on top of CYW43439 HCI (substantial work,
   no existing Rust crate)
2. Check if the MT2 has an undocumented BLE HOGP mode (unlikely)
3. Check if the MT3 (USB-C) uses BLE instead of Classic (needs testing)
4. Use a separate Classic BT module/chip (adds hardware complexity)

**What still applies from original plan:**
- Touch data encoding (9-byte format) is identical regardless of transport
- USB output side (MT2 emulation, PTP, OS fingerprinting) is unchanged
- Actuator/haptic protocol investigation is still needed
- Ploopy PTP capture is still needed for Windows output

## Research notes

### 2026-03-30: MT2 uses Bluetooth Classic, not BLE

Confirmed via hcitool/bluetoothctl on Linux. ACL connection, UUID 0x1124 (SDP HID
Profile). No GATT services. This necessitates a Classic BT stack.

### 2026-03-30: bt-hci 0.8 has all needed Classic HCI commands

CreateConnection, AuthenticationRequested, LinkKeyRequestReply, IoCapabilityRequestReply,
SetConnectionEncryption, UserConfirmationRequestReply, PinCodeRequestReply — all defined
in bt-hci 0.8's cmd::link_control module. No custom HCI definitions needed.

### 2026-03-30: CYW43439 is dual-mode, concurrent Classic + BLE

Single HCI interface multiplexes both. trouble-host takes exclusive ownership of
the controller and drops non-LE events, so we need an HCI multiplexer layer.

### 2026-03-30: Touch data confirmed correct

From Build 13 (logs/log-2.txt), touch data decodes correctly:
- X range observed: ~-30 to ~2127 (within expected -3678..+3934)
- Y range observed: ~-1464 to ~1173 (within expected -2478..+2587)
- Pressure: 5-25 for light touch, 195-200 for hard press (click)
- Touch state: 0x00=approach, 0x40=near, 0x80=contact (matches kernel)
- Click: `click=1` when force-pressed (with pressure ~200)
- Multi-finger: n=1,2,3 all work, finger IDs tracked correctly
- The "clicks" field we logged is actually the lower byte of the timestamp
  (it wraps rapidly), NOT the click button. The actual click bit is bit 0 of
  the byte at offset 1 in the BT report header (per kernel: `clicks = data[1]`).
  Our decode is correct: click=1 with high pressure = force click.

### 2026-03-30: MT2 provides autonomous haptics?

The user reported basic haptic feedback working without us sending any
actuator commands. The MT2 may have built-in haptic responses for clicks
when it detects sufficient force, or the pairing handshake enabled some
default haptic mode. This needs further investigation — if the MT2 handles
click haptics internally over BT, we may not need to implement actuator
commands for basic click feedback.

### 2026-03-30: New crate architecture

- `hci-mux`: HCI multiplexer between trouble-host (LE) and bt-classic-host (Classic)
- `bt-classic-host`: Standalone no_std Classic BT HID host (L2CAP + HIDP + SSP pairing)
- Both added to workspace and compiling (scaffold phase)

## Iterations

1. `a34d763` HCI mux scaffold + BLE passthrough verified. Existing BLE devices
   (Generic mouse) connect, pair, and stream HID reports through the mux
   without issues. Windows OS detected, USB HID standard mode. Logs: logs/log-1.txt
2. `bae39c1` Classic BT connection state machine + SSP pairing (Just Works).
   Full HCI event loop: CreateConnection → LinkKeyRequest → SSP exchange →
   AuthenticationComplete → SetConnectionEncryption → EncryptionChange.
3. `9acf8a0` Classic L2CAP: signaling (ConnReq/Rsp, ConfigReq/Rsp), channel
   state machine, CID allocation, ACL fragment reassembly, MTU negotiation.
4. `9c6f992` HIDP client: opens Control+Interrupt channels, receives input
   reports (DATA 0xA1), sends SET_REPORT/SET_PROTOCOL commands. Stack complete:
   HCI → L2CAP → HIDP.
5. `75b95b4` Firmware integration: HCI mux wired into bt2usb. ExternalController
   replaced with HciMux.split(). Classic controller available in task tree.
   Firmware builds + clippy clean. Classic task is placeholder — ready for MT2.
6. `9caee49` Full classic_bt_task: connects to MT2 (hardcoded addr), SSP pairing,
   L2CAP HID channels, multitouch enable, HIDP report loop → HID_REPORT_CHANNEL.
   Ready for first hardware test.
7. `17378e5` Fix UB: MuxShared dangling ref → store transport in HciMux.
8. `2ca943a` Fix deadlock: remove Mutex around transport (Transport takes &self).
9. `e63cf62` Fix dispatch: include HCI type indicator byte in packet lengths.
10. `90fd02f` Handle PinCodeRequest for legacy pairing (MT2 doesn't use SSP).
11. `c2e5903` Fix ACL parsing: skip HCI type indicator byte in rx buffer.
12. `8f3868d` Filter reports by ID, log non-touch data.
13. `70248e7` **TOUCH DATA RECEIVED!** Full Classic BT stack working end-to-end:
    Connection → PIN pairing → Encryption → L2CAP → HIDP → multitouch reports.
    Decoded 1/2/3-finger touch (X/Y/pressure/state), clicks (click=1 with p=200),
    and the MT2 provides some haptic feedback autonomously. Logs: logs/log-2.txt
14-21. USB translation iterations. BT→USB header translation implemented.
    macOS loads AppleMultitouchDriver and enables MT. Data reaches macOS
    (writes succeed) but produces random clicks instead of proper trackpad
    input. Multiple fixes: clicks byte masking, byte 7 mode flag (0x03),
    report padding. Critical fixes: command routing to both slot sets,
    event mask with Classic events, mux try_send to prevent deadlock.
    RESOLVED: USB output path IS working. The apparent failure was caused
    by the Dial being in Generic profile, which has a pre-existing MT2
    emulation bug (also broken on master). With profile=3 (FullScrollDial16Bit),
    Dial scroll works correctly on macOS via mt2-translation branch.
    Remaining issue: MT2 BT→USB passthrough format produces random clicks
    on macOS instead of proper trackpad input. Translation format needs work.
22. USB header format confirmed correct via pcap analysis (real MT2 USB capture
    matches our translated output byte-for-byte). Mouse delta bytes (2-3) added
    for cursor movement. Stateful Mt2Translator computes deltas from touch coords.
23. 1-finger reports were correct (user was touching with 1 finger). Multi-finger
    confirmed working (3-finger reports with len=31). BT data pipeline is fully
    functional.
24. **Root cause of "spurious clicks" identified**: macOS's AppleMultitouchDriver
    requires steady-rate report delivery with proper lifecycle timing. Raw BT→USB
    passthrough delivers reports in bursts with jitter, which the driver interprets
    as taps. Confirmed by diagnostic: hardcoded TEMPLATE_TOUCH (known-working bytes)
    also produces no useful input when triggered by MT2 events — the issue is
    delivery timing, not data format.
25. **Mux dispatch strategy**: `dispatch_le`/`dispatch_classic` use blocking
    `send().await` (ACL data must not be dropped). `dispatch_to_both` uses
    `try_send` (shared events like NumberOfCompletedPackets are safe to drop,
    and blocking here causes deadlock when BLE and Classic exec() overlap).
    SetEventMask re-send is REQUIRED — trouble-host's mask disables Classic
    auth events (LinkKeyRequest, PinCodeRequest, AuthenticationComplete).

26. **End-to-end scroll working!** Phase 1 interpreted mode: `Mt2Translator`
    extracts 2-finger scroll deltas → feeds to existing `TouchSynthesizer` →
    macOS scrolls. Known issues: low-speed rounding (small deltas lost),
    no momentum (macOS inertial scroll needs raw touch positions), jitter
    from Dial replanting logic (not appropriate for real trackpad), added
    latency from Dial velocity buffering. These stem from reusing the Dial's
    `TouchSynthesizer` which was designed for discrete scroll detents, not
    continuous touch input. A separate implementation is needed for Phase 2.

27. **Phase 2: Reclocked passthrough implemented.** `Mt2Passthrough` replaces
    `Mt2Translator`. Classic BT task sends full raw reports through channel.
    USB handler stores latest, retransmits at 250Hz with BT→USB header
    translation. Click bit latched to survive burst delivery. Results:
    all gestures working (cursor, scroll with momentum, pinch, rotate,
    3-finger drag). Logs: logs/log-33-pico.txt, logs/log-34-pico.txt.
    Known issues: (1) some stutter from BT Classic burst delivery, (2) force
    clicks don't register — macOS expects host-controlled actuator loop for
    USB MT2, ignores the autonomous click bit from BT mode.

28. **Sniff mode + interpolation tuning.** Sniff at 18 slots (11.25ms ≈ 89Hz)
    regularizes BT delivery, eliminating burst-induced stutter. Linear XY
    interpolation with EMA interval estimate and freeze-after-completion.
    Diagnostic tooling (cursor-diag.html + firmware timing logs) used to
    identify BT delivery as root cause. Logs: logs/log-36 through log-42.

29. **Classic bond storage.** `StoredClassicBond` with auto_connect flag,
    flash persistence (keys 128-137 in shared bonding flash range). Load
    at startup, persist after pairing, clear via RPC.

30. **Full RPC/CLI/status integration.** Transport type (BLE/Classic) added
    to status, bonds, scan results, and connect commands. CLI shows [BLE]
    and [Classic] labels. Classic connection state tracked via slots atomics.

31. **Classic Inquiry scanning.** HCI Inquiry with interactive CLI UI
    (same TUI as BLE scan). ExtendedInquiryResult name parsing. ScanStop
    via select-based cancellation. Command-driven classic_bt_task: no
    hardcoded address, auto-connects from stored bond or waits for RPC.

32. **Reconnect on disconnect.** Classic task wraps connect→session in a
    loop. On disconnect, resets HostResources, emits StateChanged(Disconnected),
    waits 2s, then reconnects from stored bond or waits for command.

### Passthrough architecture

The BT→USB report format is correct (confirmed via pcap comparison). The
blocker was that macOS's AppleMultitouchDriver expects USB-like report timing
(steady ~91Hz) but BT Classic delivers in bursts with jitter.

**Phase 1 (complete): Interpreted mode** — Extract scroll deltas from MT2
2-finger touch data, feed to existing TouchSynthesizer. Proves end-to-end
pipeline. Limited: no momentum, no cursor, no gestures beyond scroll.

**Phase 2 (implemented): Reclocked passthrough with interpolation** —
`Mt2Passthrough` in `mt2_translate.rs`. BT→USB header translation
(0x31→0x02, 4-byte→12-byte), retransmits at 250Hz (4ms tick). Click bit
latched to survive BT bursts. Linear interpolation of finger X/Y positions
between BT reports (20.12 fixed-point, EMA interval estimate). Sniff mode
configured at 18 slots (11.25ms ≈ 89Hz) for steady BT Classic delivery.
Results: all gestures working with smooth input — cursor, scroll with
momentum, pinch, rotate, 3-finger. Force clicks not working (see below).

### BT Classic delivery analysis (2026-04-01)

Diagnostic tooling: `cursor-diag.html` (browser-side pointer event capture
with interval histogram) + firmware `[mt2] BT` timing logs. CSV captures
in `logs/log-{36..42}-cursor-diag.csv`.

**Delivery pattern**: BT Classic delivers HID reports in bursts of 3-5
(dt=0-2 ticks / 0-8ms apart), with gaps of 8-11 ticks (32-44ms) between
bursts. Effective rate ~90 reports/sec but NOT steady.

**Interpolation exploration** (builds 37-42):
- EMA + freeze after interpolation: best for slow movements (7 jumps >50px
  vs 15 baseline). Velocity oscillation ("elasticity") at higher speeds.
- Dead reckoning: massive overshoot (69 jumps >50px).
- Raw interval (no smoothing): noisier, more jumps (32).
- Snap-to-target: position discontinuities (35 jumps).
- Current: EMA + freeze (log-37 approach). Works well for normal use.

**Root cause + fix**: CYW43439 BT Classic connection used default link
policy → unpredictable burst delivery. **FIXED**: Custom HCI link_policy
commands (WriteLinkPolicySettings + SniffMode) defined in
`bt-classic-host/src/link_policy.rs`. Sniff configured at 18 slots
(11.25ms ≈ 89Hz) after connection reaches Encrypted state. Result:
perfectly smooth input delivery, eliminating all interpolation artifacts.

### Force click analysis (2026-04-01)

The MT2 has **two click modes**:
- **BT mode (autonomous)**: Internal pressure threshold, fires Taptic Engine
  independently, sets click bit (byte[1] & 0x01) in report. BT host just
  reads the button bit. This is how the MT2 works over BT Classic.
- **USB mode (host-controlled)**: macOS's AppleMultitouchDriver reads pressure
  from touch data, decides when to click, sends actuator OUTPUT report to
  trackpad, Taptic Engine fires, trackpad confirms with button bit.

Our bridge: MT2 is in BT mode (autonomous clicks, haptics fire, click bit
set). But macOS sees a USB MT2 and expects host-controlled mode. It ignores
the button bit because it never sent an actuator command.

Evidence:
- Click bit IS present in BT data (confirmed: 14+ consecutive reports with
  click=1, pressure ~105). Click IS latched and forwarded in USB reports.
- Force clicks **never** register on macOS (not intermittent — deterministic).
- "Tap to click" works (macOS detects quick touch→release gesture from touch
  data alone, independent of button bit).
- MT2 fires haptic feedback autonomously over BT (user feels the click).

**Fix options** (in order of likely effort):
1. Auto-ACK actuator: intercept macOS OUTPUT reports for actuator, respond
   immediately. MT2 is already clicking autonomously, macOS just needs to
   think its command succeeded.
2. Feature report tweak: 0xDB compound properties may contain a force-touch
   mode flag. If we indicate "trackpad-controlled clicks," macOS might fall
   back to reading the button bit.
3. Full actuator forwarding: USB OUTPUT → BT SET_REPORT. Most correct but
   most complex (bidirectional HIDP, new L2CAP flow).

### Remaining TODOs

**Force click / actuator** (high priority for UX):
1. Auto-ACK actuator: intercept macOS OUTPUT reports for actuator, respond
   immediately. MT2 is already clicking autonomously, macOS just needs to
   think its command succeeded.
2. Feature report tweak: 0xDB compound properties may contain a force-touch
   mode flag. If we indicate "trackpad-controlled clicks," macOS might fall
   back to reading the button bit.
3. Full actuator forwarding: USB OUTPUT → BT SET_REPORT. Most correct but
   most complex (bidirectional HIDP, new L2CAP flow).

**Windows PTP output**: Present as Precision Touchpad on Windows hosts.
Separate USB descriptor + report format. Ploopy trackpad captures in
`notes/magic-mouse/` for reference.

**RPC integration gaps**:
- Wire `UpdateBondProfile` to also check Classic bonds (currently BLE-only)
- Wire `SetAutoConnect` to also check Classic bonds
- Classic Inquiry: add RemoteNameRequest for discovered devices (names only
  show for ExtendedInquiryResult currently, not standard InquiryResult)

### Reference captures

- `notes/magic-mouse/mt2-capture.pcap` — Real MT2 over USB (macOS, Darwin format)
- `notes/magic-mouse/mt2-capture-emulated-28.pcap` — Emulated MT2 scroll (macOS)
- `notes/mt2-translation/capture.btsnoop` — MT2 over BT Classic on Linux (btmon).
  Shows full HIDP command sequence including GET_REPORT(0x90) polling that triggers
  multitouch activation. Includes 1/2/3-finger gesture data.
- `notes/mt2-translation/cursor-diag.html` — Browser diagnostic for pointer
  event timing analysis.

