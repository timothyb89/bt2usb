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

