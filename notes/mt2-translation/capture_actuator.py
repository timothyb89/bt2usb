#!/usr/bin/env python3
"""
Magic Trackpad 2 Taptic Engine (Actuator) protocol investigation tool.

The MT2 is a solid glass slab with ZERO physical click mechanism. All haptic
feedback comes from a linear actuator (Taptic Engine) driven by commands sent
from the host OS over USB. On macOS, MultitouchSupport.framework /
AppleUSBTopCaseHIDDriver sends Output reports to Interface 2 of the MT2 to
trigger haptic pulses. On Linux, hid-magicmouse.c does NOT handle the actuator
at all, so the trackpad feels "dead" when clicking on Linux.

This script investigates the actuator protocol to enable:
  1. Forwarding actuator commands from macOS/Linux through our USB-to-BLE bridge
  2. Generating our own haptic commands on Windows (no native Apple driver)

=== What we know about Interface 2 (Actuator) ===

From real MT2 USB descriptor captures (notes/magic-mouse/):

  Interface 2: "Actuator"
    bInterfaceClass  = 3 (HID)
    bInterfaceSubCl  = 0
    bInterfaceProto  = 0
    2 endpoints:
      EP 0x84 (IN  INT): maxPacket=16, interval=8ms
      EP 0x04 (OUT INT): maxPacket=64, interval=2ms

  HID Report Descriptor (36 bytes):
    Usage Page (Vendor 0xFF00)
    Usage (0x0D)
    Collection (Application)
      Usage Page (Vendor 0xFF00)
      Usage (0x0D)
      Logical Minimum (0)
      Logical Maximum (255)
      Report Size (8)

      Report ID (0x3F)
      Report Count (15)
      Input (Data, Variable, Absolute)    <-- 15 bytes IN from device

      Usage (0x0D)
      Report ID (0x53)
      Report Count (63)
      Output (Data, Variable, Absolute)   <-- 63 bytes OUT to device (actuator cmd!)

    End Collection

  So the actuator interface has:
    - Report ID 0x3F: 15-byte INPUT report (actuator -> host, status/feedback?)
    - Report ID 0x53: 63-byte OUTPUT report (host -> actuator, haptic commands!)

  The OUT endpoint (EP 0x04) is for sending Report 0x53 to trigger haptics.
  The IN endpoint (EP 0x84) likely returns actuator status or acknowledgments.

=== Modes ===

  --monitor   : Capture actuator traffic (IN + OUT) while user interacts
  --replay    : Send a known command to the actuator to verify understanding
  --probe     : Systematically try payloads to discover command space
  --descriptor: Dump and parse HID descriptors for all interfaces

Usage:
    pip3 install pyusb
    sudo python3 capture_actuator.py --monitor
    sudo python3 capture_actuator.py --replay --command "53 01 00 ..."
    sudo python3 capture_actuator.py --probe
    sudo python3 capture_actuator.py --descriptor

Requirements:
    - pyusb (pip3 install pyusb)
    - libusb backend (apt install libusb-1.0-0-dev / brew install libusb)
    - Root / sudo on Linux (for USB device access)
    - On macOS, may need to disable SIP or use a codeless kext to detach driver
"""

import argparse
import json
import os
import platform
import signal
import struct
import sys
import time
from datetime import datetime

try:
    import usb.core
    import usb.util
except ImportError:
    print("ERROR: pyusb not installed. Run: pip3 install pyusb")
    print("Also ensure libusb is installed:")
    print("  Linux:  sudo apt install libusb-1.0-0-dev")
    print("  macOS:  brew install libusb")
    sys.exit(1)


# ============================================================================
# Constants
# ============================================================================

# Apple Magic Trackpad 2 USB identity
DEFAULT_VID = 0x05AC
DEFAULT_PID = 0x0265

# MT2 interface assignments (from real device capture)
IF_DEVICE_MGMT = 0   # "Device Management" — vendor reports (0xE0, 0x9A, 0x90)
IF_TRACKPAD = 1       # "Trackpad / Boot" — mouse + multitouch (0x02, 0x3F, 0x44)
IF_ACTUATOR = 2       # "Actuator" — Taptic Engine (IN: 0x3F, OUT: 0x53)
IF_ACCELEROMETER = 3  # "Accelerometer" — motion data (0xC0)

# Known report IDs on the actuator interface (IF2)
ACTUATOR_IN_REPORT_ID = 0x3F   # 15 bytes, device -> host
ACTUATOR_OUT_REPORT_ID = 0x53  # 63 bytes, host -> device (haptic command)

# Actuator endpoints (from descriptor)
ACTUATOR_EP_IN = 0x84   # IN interrupt, 16 bytes max, 8ms interval
ACTUATOR_EP_OUT = 0x04  # OUT interrupt, 64 bytes max, 2ms interval

# USB HID class request constants
HID_GET_REPORT = 0x01
HID_SET_REPORT = 0x09
HID_REPORT_TYPE_INPUT = 0x01
HID_REPORT_TYPE_OUTPUT = 0x02
HID_REPORT_TYPE_FEATURE = 0x03

# HID descriptor type
DT_HID_REPORT = 0x22
USB_GET_DESCRIPTOR = 0x06

# Interface names for display
INTERFACE_NAMES = {
    0: "Device Management",
    1: "Trackpad / Boot",
    2: "Actuator",
    3: "Accelerometer",
}

# Feature report IDs observed during macOS driver initialization.
# These are probed on various interfaces during enumeration.
KNOWN_FEATURE_REPORT_IDS = [0x00, 0xDB, 0xD1, 0xD3, 0xD0, 0xA1, 0xD9, 0x7F, 0xC3]


# ============================================================================
# Helpers
# ============================================================================

def hex_dump(data, prefix="  "):
    """Pretty-print a byte sequence as hex + ASCII."""
    data = bytes(data)
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{i:04x}: {hex_part:<48s}  {ascii_part}")
    return "\n".join(lines)


def hex_string(data):
    """Compact hex string."""
    return " ".join(f"{b:02x}" for b in bytes(data))


def timestamp():
    """High-resolution timestamp string."""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def find_device(vid, pid):
    """Find and return the USB device, or exit with error."""
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        print(f"ERROR: No device found with VID=0x{vid:04X} PID=0x{pid:04X}")
        print("Make sure the Magic Trackpad 2 is connected via USB.")
        sys.exit(1)

    try:
        mfr = dev.manufacturer or "(unknown)"
        prod = dev.product or "(unknown)"
        serial = dev.serial_number or "(unknown)"
    except Exception:
        mfr = prod = serial = "(could not read)"

    print(f"Found device: {mfr} {prod}")
    print(f"  Serial: {serial}")
    print(f"  Bus {dev.bus}, Address {dev.address}")
    print(f"  bcdDevice: 0x{dev.bcdDevice:04X}")
    return dev


def detach_kernel_driver(dev, interface):
    """Detach kernel driver from interface if attached (Linux only)."""
    try:
        if dev.is_kernel_driver_active(interface):
            dev.detach_kernel_driver(interface)
            print(f"  Detached kernel driver from interface {interface} "
                  f"({INTERFACE_NAMES.get(interface, '?')})")
            return True
    except NotImplementedError:
        # macOS libusb doesn't support this
        pass
    except usb.core.USBError as e:
        print(f"  Warning: could not detach kernel driver from IF{interface}: {e}")
    return False


def reattach_kernel_driver(dev, interface):
    """Try to reattach kernel driver."""
    try:
        dev.attach_kernel_driver(interface)
        print(f"  Reattached kernel driver to interface {interface}")
    except Exception:
        pass


def claim_interface(dev, interface):
    """Claim a USB interface, detaching kernel driver if needed."""
    detach_kernel_driver(dev, interface)
    try:
        usb.util.claim_interface(dev, interface)
        print(f"  Claimed interface {interface} ({INTERFACE_NAMES.get(interface, '?')})")
    except usb.core.USBError as e:
        print(f"  ERROR: Could not claim interface {interface}: {e}")
        print("  On macOS, you may need to disable SIP or use a codeless kext.")
        print("  On Linux, ensure you're running as root (sudo).")
        sys.exit(1)


def release_interface(dev, interface):
    """Release interface and try to reattach kernel driver."""
    try:
        usb.util.release_interface(dev, interface)
    except Exception:
        pass
    reattach_kernel_driver(dev, interface)


def find_endpoints(dev, interface):
    """Find IN and OUT endpoints for the given interface."""
    cfg = dev.get_active_configuration()
    intf = cfg[(interface, 0)]

    ep_in = None
    ep_out = None
    for ep in intf:
        direction = usb.util.endpoint_direction(ep.bEndpointAddress)
        if direction == usb.util.ENDPOINT_IN:
            ep_in = ep
        elif direction == usb.util.ENDPOINT_OUT:
            ep_out = ep

    return ep_in, ep_out


def open_output_file(path):
    """Open a JSON lines output file."""
    if path is None:
        return None
    f = open(path, "a")
    # Write session header
    session = {
        "type": "session_start",
        "timestamp": datetime.now().isoformat(),
        "platform": platform.system(),
        "python": platform.python_version(),
    }
    f.write(json.dumps(session) + "\n")
    return f


def write_record(f, record):
    """Write a record to the JSON lines file."""
    if f is not None:
        f.write(json.dumps(record) + "\n")
        f.flush()


def parse_hid_descriptor(data):
    """
    Parse a HID report descriptor and return a list of human-readable lines.
    Also returns structured info about report IDs and their types/sizes.
    """
    lines = []
    reports = {}  # report_id -> {"input_bits": N, "output_bits": N, "feature_bits": N}
    indent = 0
    i = 0
    current_report_id = 0
    current_report_size = 0
    current_report_count = 0

    while i < len(data):
        b = data[i]

        # Short item format
        bSize = b & 0x03
        if bSize == 3:
            bSize = 4
        bType = (b >> 2) & 0x03
        bTag = (b >> 4) & 0x0F

        if b == 0xFE:
            # Long item
            if i + 2 >= len(data):
                break
            data_size = data[i + 1]
            lines.append(f"{'  ' * indent}[{i:3d}] LONG ITEM ({data_size} bytes)")
            i += 3 + data_size
            continue

        # Read value
        val = 0
        raw_bytes = [b]
        for j in range(bSize):
            if i + 1 + j < len(data):
                val |= data[i + 1 + j] << (8 * j)
                raw_bytes.append(data[i + 1 + j])

        hex_str = " ".join(f"{x:02x}" for x in raw_bytes)

        # Decode specific items
        desc = ""
        if bType == 0:  # Main
            if bTag == 0x08:
                desc = f"Input (0x{val:02X})"
                bits = current_report_size * current_report_count
                if current_report_id not in reports:
                    reports[current_report_id] = {"input_bits": 0, "output_bits": 0, "feature_bits": 0}
                reports[current_report_id]["input_bits"] += bits
            elif bTag == 0x09:
                desc = f"Output (0x{val:02X})"
                bits = current_report_size * current_report_count
                if current_report_id not in reports:
                    reports[current_report_id] = {"input_bits": 0, "output_bits": 0, "feature_bits": 0}
                reports[current_report_id]["output_bits"] += bits
            elif bTag == 0x0B:
                desc = f"Feature (0x{val:02X})"
                bits = current_report_size * current_report_count
                if current_report_id not in reports:
                    reports[current_report_id] = {"input_bits": 0, "output_bits": 0, "feature_bits": 0}
                reports[current_report_id]["feature_bits"] += bits
            elif bTag == 0x0A:
                ctype = {0: "Physical", 1: "Application", 2: "Logical"}.get(val, f"0x{val:02X}")
                desc = f"Collection ({ctype})"
                indent += 1
            elif bTag == 0x0C:
                indent = max(0, indent - 1)
                desc = "End Collection"
        elif bType == 1:  # Global
            names = {
                0: "Usage Page", 1: "Logical Minimum", 2: "Logical Maximum",
                3: "Physical Minimum", 4: "Physical Maximum", 5: "Unit Exponent",
                6: "Unit", 7: "Report Size", 8: "Report ID", 9: "Report Count",
            }
            name = names.get(bTag, f"Global(0x{bTag:X})")
            if bTag == 0 and val >= 0xFF00:
                desc = f"{name} (Vendor 0x{val:04X})"
            elif bTag == 0:
                page_names = {
                    0x01: "Generic Desktop", 0x05: "Gaming", 0x09: "Button",
                    0x0D: "Digitizer", 0x84: "Power Device", 0x85: "Battery System",
                }
                desc = f"{name} ({page_names.get(val, f'0x{val:02X}')})"
            elif bTag == 7:
                desc = f"{name} ({val})"
                current_report_size = val
            elif bTag == 8:
                desc = f"{name} (0x{val:02X})"
                current_report_id = val
            elif bTag == 9:
                desc = f"{name} ({val})"
                current_report_count = val
            else:
                desc = f"{name} ({val})"
        elif bType == 2:  # Local
            names = {0: "Usage", 1: "Usage Minimum", 2: "Usage Maximum"}
            name = names.get(bTag, f"Local(0x{bTag:X})")
            desc = f"{name} (0x{val:02X})"

        pad = "  " * (indent if bTag != 0x0C or bType != 0 else indent)
        lines.append(f"{pad}[{i:3d}] {hex_str:20s} {desc}")

        i += 1 + bSize

    return lines, reports


# ============================================================================
# Mode: Monitor
# ============================================================================

def mode_monitor(args):
    """
    Monitor actuator traffic on the MT2.

    This is the primary investigation mode. We claim both the actuator interface
    (IF2) and optionally the trackpad interface (IF1), then poll for:

      - IN reports on IF2 (actuator status/feedback, Report ID 0x3F)
      - OUT reports we could intercept if we were the host driver (on macOS)
      - IN reports on IF1 (touch data) to correlate haptics with touches

    IMPORTANT LIMITATION: pyusb can only see traffic on interfaces we claim.
    If the macOS kernel driver is sending actuator commands, we need to detach
    it first -- but then the driver stops sending commands! This is the
    fundamental chicken-and-egg problem of USB monitoring from userspace.

    Strategies to work around this:
      1. Use a USB hardware analyzer (Beagle, Total Phase) for passive capture
      2. On macOS, use `sudo dtrace` or IOKit tracing
      3. On Linux with usbmon: `sudo modprobe usbmon && sudo cat /sys/kernel/debug/usb/usbmon/*u`
      4. Use a USB-over-IP setup to MITM
      5. Capture with Wireshark + usbmon on Linux

    What this script CAN do:
      - Read actuator IN reports (status/ack from the Taptic Engine)
      - After we send OUT commands in replay/probe mode, monitor the response
      - Correlate with touch data if we also monitor IF1
    """
    dev = find_device(args.vid, args.pid)
    output_file = open_output_file(args.output)

    interfaces_to_claim = [args.interface]
    if args.also_trackpad:
        interfaces_to_claim.append(IF_TRACKPAD)

    # Claim interfaces
    for iface in interfaces_to_claim:
        claim_interface(dev, iface)

    # Find endpoints
    actuator_ep_in, actuator_ep_out = find_endpoints(dev, args.interface)
    trackpad_ep_in = None
    if args.also_trackpad:
        trackpad_ep_in, _ = find_endpoints(dev, IF_TRACKPAD)
        # Activate multitouch mode on IF1 so we get full touch reports
        try:
            dev.ctrl_transfer(0x21, HID_SET_REPORT,
                              (HID_REPORT_TYPE_FEATURE << 8) | 0x02,
                              IF_TRACKPAD, bytes([0x01]), timeout=1000)
            print("  Activated multitouch mode on IF1")
        except usb.core.USBError as e:
            print(f"  Warning: could not activate multitouch: {e}")

    print(f"\nEndpoints on IF{args.interface} (Actuator):")
    if actuator_ep_in:
        print(f"  IN:  0x{actuator_ep_in.bEndpointAddress:02X} "
              f"(maxPacket={actuator_ep_in.wMaxPacketSize}, interval={actuator_ep_in.bInterval}ms)")
    else:
        print("  IN:  (none)")
    if actuator_ep_out:
        print(f"  OUT: 0x{actuator_ep_out.bEndpointAddress:02X} "
              f"(maxPacket={actuator_ep_out.wMaxPacketSize}, interval={actuator_ep_out.bInterval}ms)")
    else:
        print("  OUT: (none)")

    if trackpad_ep_in:
        print(f"\nAlso monitoring IF{IF_TRACKPAD} (Trackpad):")
        print(f"  IN:  0x{trackpad_ep_in.bEndpointAddress:02X} "
              f"(maxPacket={trackpad_ep_in.wMaxPacketSize})")

    # Also try reading actuator feature reports for baseline state
    print("\n--- Probing actuator Feature reports ---")
    for rid in [0x3F, 0x53] + KNOWN_FEATURE_REPORT_IDS:
        try:
            feat = dev.ctrl_transfer(
                0xA1, HID_GET_REPORT,
                (HID_REPORT_TYPE_FEATURE << 8) | rid,
                args.interface, 256, timeout=500)
            print(f"  GET_REPORT(Feature, 0x{rid:02X}) on IF{args.interface}: "
                  f"[{len(feat)}B] {hex_string(feat[:32])}"
                  + (f" ... ({len(feat)}B total)" if len(feat) > 32 else ""))
            write_record(output_file, {
                "type": "feature_report",
                "timestamp": timestamp(),
                "interface": args.interface,
                "report_id": f"0x{rid:02X}",
                "direction": "in",
                "length": len(feat),
                "data": hex_string(feat),
            })
        except usb.core.USBError:
            pass  # Not all report IDs will be valid

    # Also probe Output report via GET_REPORT to see if we can read back state
    print("\n--- Probing actuator Output reports (GET_REPORT) ---")
    for rid in [0x53]:
        try:
            feat = dev.ctrl_transfer(
                0xA1, HID_GET_REPORT,
                (HID_REPORT_TYPE_OUTPUT << 8) | rid,
                args.interface, 256, timeout=500)
            print(f"  GET_REPORT(Output, 0x{rid:02X}) on IF{args.interface}: "
                  f"[{len(feat)}B] {hex_string(feat[:32])}"
                  + (f" ... ({len(feat)}B total)" if len(feat) > 32 else ""))
        except usb.core.USBError as e:
            print(f"  GET_REPORT(Output, 0x{rid:02X}) on IF{args.interface}: {e}")

    print("\n" + "=" * 70)
    print("MONITORING — press Ctrl-C to stop")
    print("=" * 70)
    print()
    print("NOTE: If the macOS kernel driver was detached, actuator commands from")
    print("the OS will NOT be captured (the driver has been disconnected).")
    print("Use usbmon/Wireshark/hardware analyzer for passive monitoring.")
    print()
    print("This monitor will capture:")
    print("  - Actuator IN reports (status from Taptic Engine)")
    if args.also_trackpad:
        print("  - Trackpad IN reports (touch data for correlation)")
    print()

    count = 0
    t0 = time.monotonic()

    # Set up graceful exit
    running = [True]

    def signal_handler(sig, frame):
        running[0] = False

    signal.signal(signal.SIGINT, signal_handler)

    while running[0]:
        # Poll actuator IN endpoint
        if actuator_ep_in:
            try:
                data = actuator_ep_in.read(actuator_ep_in.wMaxPacketSize, timeout=50)
                data = bytes(data)
                elapsed = time.monotonic() - t0
                rid = data[0] if len(data) > 0 else 0xFF

                record = {
                    "type": "actuator_in",
                    "seq": count,
                    "timestamp": timestamp(),
                    "elapsed_ms": round(elapsed * 1000, 1),
                    "interface": args.interface,
                    "endpoint": f"0x{actuator_ep_in.bEndpointAddress:02X}",
                    "report_id": f"0x{rid:02X}",
                    "length": len(data),
                    "data": hex_string(data),
                }
                write_record(output_file, record)

                print(f"[{count:5d}] {timestamp()} ACTUATOR IN  "
                      f"RID=0x{rid:02X} [{len(data):3d}B] {hex_string(data)}")
                count += 1
            except usb.core.USBTimeoutError:
                pass
            except usb.core.USBError as e:
                if running[0]:
                    print(f"  Actuator IN error: {e}")
                break

        # Poll trackpad IN endpoint
        if trackpad_ep_in:
            try:
                data = trackpad_ep_in.read(trackpad_ep_in.wMaxPacketSize, timeout=10)
                data = bytes(data)
                elapsed = time.monotonic() - t0
                rid = data[0] if len(data) > 0 else 0xFF

                record = {
                    "type": "trackpad_in",
                    "seq": count,
                    "timestamp": timestamp(),
                    "elapsed_ms": round(elapsed * 1000, 1),
                    "interface": IF_TRACKPAD,
                    "endpoint": f"0x{trackpad_ep_in.bEndpointAddress:02X}",
                    "report_id": f"0x{rid:02X}",
                    "length": len(data),
                    "data": hex_string(data),
                }
                write_record(output_file, record)

                # Compact display for trackpad data
                if rid == 0x02 and len(data) > 12:
                    clicks = data[1]
                    n_touches = (len(data) - 12) // 9
                    print(f"[{count:5d}] {timestamp()} TRACKPAD IN  "
                          f"RID=0x{rid:02X} [{len(data):3d}B] "
                          f"clicks={clicks} touches={n_touches}")
                else:
                    print(f"[{count:5d}] {timestamp()} TRACKPAD IN  "
                          f"RID=0x{rid:02X} [{len(data):3d}B] "
                          f"{hex_string(data[:32])}"
                          + ("..." if len(data) > 32 else ""))
                count += 1
            except usb.core.USBTimeoutError:
                pass
            except usb.core.USBError:
                pass  # Trackpad errors are non-fatal

    # Cleanup
    print(f"\n--- Captured {count} reports ---")
    for iface in interfaces_to_claim:
        release_interface(dev, iface)
    if output_file:
        output_file.close()
        print(f"Output saved to: {args.output}")


# ============================================================================
# Mode: Replay
# ============================================================================

def mode_replay(args):
    """
    Send a known actuator command to the MT2 to verify it triggers the Taptic Engine.

    The actuator accepts 63-byte Output reports with Report ID 0x53 on Interface 2.
    Commands can be sent either via:
      a) The OUT interrupt endpoint (EP 0x04) — include Report ID as first byte
      b) SET_REPORT class request on the control endpoint

    We try both methods and the user confirms whether they felt a haptic pulse.
    """
    if not args.command and not args.file:
        print("ERROR: Provide --command (hex string) or --file (path to hex file)")
        sys.exit(1)

    # Parse the command payload
    if args.command:
        # Accept hex strings like "53 01 00 ff" or "53010000ff"
        hex_str = args.command.replace(" ", "").replace("0x", "").replace(",", "")
        try:
            payload = bytes.fromhex(hex_str)
        except ValueError:
            print(f"ERROR: Invalid hex string: {args.command}")
            sys.exit(1)
    elif args.file:
        with open(args.file, "r") as f:
            hex_str = f.read().strip().replace(" ", "").replace("\n", "")
            payload = bytes.fromhex(hex_str)

    print(f"Payload ({len(payload)} bytes):")
    print(hex_dump(payload))

    # The report ID should be the first byte for endpoint transfers,
    # or specified separately for SET_REPORT control transfers.
    if len(payload) > 0 and payload[0] == ACTUATOR_OUT_REPORT_ID:
        report_id = payload[0]
        report_data = payload[1:]
        print(f"\nFirst byte is Report ID 0x{report_id:02X} (will be used as report ID)")
    else:
        report_id = ACTUATOR_OUT_REPORT_ID
        report_data = payload
        print(f"\nUsing default Report ID 0x{report_id:02X}")
        print(f"Full payload will be sent as report data (no ID prefix)")

    dev = find_device(args.vid, args.pid)
    output_file = open_output_file(args.output)
    claim_interface(dev, args.interface)

    _, ep_out = find_endpoints(dev, args.interface)

    # Method 1: Send via OUT interrupt endpoint
    if ep_out and not args.control_only:
        print(f"\n--- Method 1: OUT interrupt endpoint (EP 0x{ep_out.bEndpointAddress:02X}) ---")
        # For interrupt OUT, the report ID is the first byte of the transfer
        out_data = bytes([report_id]) + report_data
        # Pad to expected size if needed (Report 0x53 = 63 bytes + 1 ID = 64 bytes)
        expected_size = 64  # Report ID (1) + Report Count (63)
        if len(out_data) < expected_size:
            out_data = out_data + bytes(expected_size - len(out_data))
            print(f"  Padded to {len(out_data)} bytes (descriptor says 63-byte report + 1 ID)")

        print(f"  Sending {len(out_data)} bytes to EP 0x{ep_out.bEndpointAddress:02X}:")
        print(hex_dump(out_data, prefix="    "))

        try:
            written = ep_out.write(out_data, timeout=1000)
            print(f"  OK: wrote {written} bytes")
            write_record(output_file, {
                "type": "replay_out_endpoint",
                "timestamp": timestamp(),
                "interface": args.interface,
                "endpoint": f"0x{ep_out.bEndpointAddress:02X}",
                "report_id": f"0x{report_id:02X}",
                "length": written,
                "data": hex_string(out_data),
                "method": "interrupt_out",
            })
            print("\n  >>> Did you feel a haptic click? (check the trackpad) <<<")
        except usb.core.USBError as e:
            print(f"  ERROR: {e}")

        time.sleep(0.5)

    # Method 2: Send via SET_REPORT control transfer
    if not args.endpoint_only:
        print(f"\n--- Method 2: SET_REPORT control transfer ---")
        # SET_REPORT: bmRequestType=0x21, bRequest=0x09
        # wValue: (report_type << 8) | report_id
        # wIndex: interface number
        # For Output reports, report_type = 0x02
        wValue = (HID_REPORT_TYPE_OUTPUT << 8) | report_id

        # The data does NOT include the report ID for SET_REPORT
        set_data = report_data
        # Pad to expected report size
        expected_data_size = 63  # Report Count from descriptor
        if len(set_data) < expected_data_size:
            set_data = set_data + bytes(expected_data_size - len(set_data))
            print(f"  Padded data to {len(set_data)} bytes (descriptor says 63)")

        print(f"  SET_REPORT(Output, ID=0x{report_id:02X}) on IF{args.interface}:")
        print(f"  wValue=0x{wValue:04X}, wIndex={args.interface}, {len(set_data)} bytes data")
        print(hex_dump(set_data, prefix="    "))

        try:
            dev.ctrl_transfer(0x21, HID_SET_REPORT, wValue,
                              args.interface, set_data, timeout=1000)
            print(f"  OK: SET_REPORT accepted")
            write_record(output_file, {
                "type": "replay_set_report",
                "timestamp": timestamp(),
                "interface": args.interface,
                "report_id": f"0x{report_id:02X}",
                "report_type": "output",
                "length": len(set_data),
                "data": hex_string(set_data),
                "method": "set_report",
            })
            print("\n  >>> Did you feel a haptic click? (check the trackpad) <<<")
        except usb.core.USBError as e:
            print(f"  ERROR: {e}")

    # Read back any response on the IN endpoint
    ep_in, _ = find_endpoints(dev, args.interface)
    if ep_in:
        print(f"\n--- Checking for actuator response on EP 0x{ep_in.bEndpointAddress:02X} ---")
        for attempt in range(5):
            try:
                data = ep_in.read(ep_in.wMaxPacketSize, timeout=200)
                data = bytes(data)
                print(f"  Response [{len(data)}B]: {hex_string(data)}")
                write_record(output_file, {
                    "type": "replay_response",
                    "timestamp": timestamp(),
                    "interface": args.interface,
                    "endpoint": f"0x{ep_in.bEndpointAddress:02X}",
                    "length": len(data),
                    "data": hex_string(data),
                })
            except usb.core.USBTimeoutError:
                if attempt == 0:
                    print("  (no response within 200ms)")
                break
            except usb.core.USBError as e:
                print(f"  Error: {e}")
                break

    release_interface(dev, args.interface)
    if output_file:
        output_file.close()
        print(f"\nOutput saved to: {args.output}")


# ============================================================================
# Mode: Probe
# ============================================================================

def mode_probe(args):
    """
    Systematically probe the actuator command space.

    Strategy:
      1. Try sending Report ID 0x53 with various first-byte values (potential command IDs)
      2. Try other report IDs that might work on IF2
      3. Try SET_REPORT with Feature type (in case there are undocumented Feature reports)
      4. Try small payloads to large payloads
      5. Log everything — the user confirms haptic responses tactilely

    CAUTION: This sends arbitrary data to the device. The MT2 firmware should
    be robust against invalid commands, but proceed carefully. We start with
    small, conservative payloads.
    """
    dev = find_device(args.vid, args.pid)
    output_file = open_output_file(args.output)
    claim_interface(dev, args.interface)

    _, ep_out = find_endpoints(dev, args.interface)
    ep_in, _ = find_endpoints(dev, args.interface)

    print("\n" + "=" * 70)
    print("ACTUATOR PROBE MODE")
    print("=" * 70)
    print()
    print("This will send various commands to the actuator interface.")
    print("Keep your finger on the trackpad surface and note which commands")
    print("produce a haptic sensation. Press Enter after each command to continue,")
    print("or type 'y' if you felt something, 'n' if not, 'q' to quit.")
    print()

    probe_count = 0

    def send_and_check(desc, method, report_id, data, wait_s=0.3):
        """Send a command and ask the user if they felt it."""
        nonlocal probe_count
        probe_count += 1

        print(f"\n[{probe_count:3d}] {desc}")
        print(f"      Report ID: 0x{report_id:02X}, Data: {hex_string(data[:32])}"
              + ("..." if len(data) > 32 else "")
              + f" ({len(data)} bytes)")

        success = False
        error_msg = None

        if method == "endpoint" and ep_out:
            out_data = bytes([report_id]) + data
            # Pad to 64 bytes for endpoint transfer
            if len(out_data) < 64:
                out_data = out_data + bytes(64 - len(out_data))
            try:
                ep_out.write(out_data, timeout=1000)
                success = True
            except usb.core.USBError as e:
                error_msg = str(e)

        elif method == "set_report":
            wValue = (HID_REPORT_TYPE_OUTPUT << 8) | report_id
            pad_data = data
            if len(pad_data) < 63:
                pad_data = pad_data + bytes(63 - len(pad_data))
            try:
                dev.ctrl_transfer(0x21, HID_SET_REPORT, wValue,
                                  args.interface, pad_data, timeout=1000)
                success = True
            except usb.core.USBError as e:
                error_msg = str(e)

        if not success:
            print(f"      REJECTED: {error_msg}")
            write_record(output_file, {
                "type": "probe",
                "seq": probe_count,
                "timestamp": timestamp(),
                "description": desc,
                "method": method,
                "report_id": f"0x{report_id:02X}",
                "data": hex_string(data),
                "result": "rejected",
                "error": error_msg,
            })
            return

        print(f"      ACCEPTED by device")

        # Check for IN response
        if ep_in:
            try:
                resp = ep_in.read(ep_in.wMaxPacketSize, timeout=100)
                resp = bytes(resp)
                print(f"      Response: {hex_string(resp)}")
            except usb.core.USBTimeoutError:
                pass

        time.sleep(wait_s)

        # Ask user
        try:
            response = input("      Did you feel a haptic? (y/n/q, Enter=no): ").strip().lower()
        except EOFError:
            response = ""

        felt = response == "y"
        if response == "q":
            raise KeyboardInterrupt

        write_record(output_file, {
            "type": "probe",
            "seq": probe_count,
            "timestamp": timestamp(),
            "description": desc,
            "method": method,
            "report_id": f"0x{report_id:02X}",
            "data": hex_string(data),
            "result": "felt" if felt else "accepted_no_feel",
        })

        if felt:
            print("      *** HAPTIC CONFIRMED! ***")

    try:
        # Phase 1: Try known Report ID 0x53 with simple payloads
        # The descriptor says Report 0x53 is a 63-byte Output report.
        # We don't know the command format yet, so try systematic patterns.
        print("\n=== Phase 1: Report ID 0x53 basic patterns ===")
        print("(Using SET_REPORT on control endpoint)")

        # Try all-zeros (null command)
        send_and_check("0x53: all zeros", "set_report", 0x53, bytes(63))

        # Try first byte as a command selector, rest zeros
        for cmd in [0x01, 0x02, 0x03, 0x04, 0x05, 0x10, 0x20, 0x40, 0x80, 0xFF]:
            send_and_check(f"0x53: cmd=0x{cmd:02X}", "set_report", 0x53,
                           bytes([cmd]) + bytes(62))

        # Phase 2: Try with intensity-like values in byte 2
        print("\n=== Phase 2: Report ID 0x53 with intensity variations ===")
        for cmd in [0x01, 0x02, 0x10]:
            for intensity in [0x20, 0x40, 0x80, 0xFF]:
                send_and_check(
                    f"0x53: cmd=0x{cmd:02X} intensity=0x{intensity:02X}",
                    "set_report", 0x53,
                    bytes([cmd, intensity]) + bytes(61))

        # Phase 3: Try Apple-style patterns
        # Apple often uses structured payloads: [type, subtype, param, ...]
        print("\n=== Phase 3: Apple-style structured payloads ===")

        # Pattern from AppleHaptics: [type=0x11, waveform, amplitude_lo, amplitude_hi, ...]
        for wave in range(0, 16):
            send_and_check(
                f"0x53: haptic? type=0x11 wave={wave}",
                "set_report", 0x53,
                bytes([0x11, wave, 0xFF, 0x00]) + bytes(59))

        # Phase 4: Try the OUT interrupt endpoint instead
        if ep_out:
            print("\n=== Phase 4: OUT interrupt endpoint ===")
            for cmd in [0x01, 0x02, 0x10, 0x11, 0xFF]:
                send_and_check(
                    f"EP OUT: cmd=0x{cmd:02X}",
                    "endpoint", 0x53,
                    bytes([cmd, 0x80]) + bytes(61))

        # Phase 5: Try other report IDs on IF2
        print("\n=== Phase 5: Other report IDs via SET_REPORT ===")
        for rid in [0x01, 0x02, 0x3F, 0x44, 0x52, 0x54, 0x55, 0xE0, 0xF0, 0xFF]:
            if rid == 0x53:
                continue
            send_and_check(
                f"RID 0x{rid:02X}: cmd=0x01",
                "set_report", rid,
                bytes([0x01, 0x80]) + bytes(61))

        # Phase 6: Try Feature report SET on IF2
        print("\n=== Phase 6: Feature report SET on IF2 ===")
        for rid in [0x3F, 0x53, 0xD0, 0xD1]:
            try:
                wValue = (HID_REPORT_TYPE_FEATURE << 8) | rid
                dev.ctrl_transfer(0x21, HID_SET_REPORT, wValue,
                                  args.interface, bytes([0x01]) + bytes(62),
                                  timeout=1000)
                print(f"  SET_REPORT(Feature, 0x{rid:02X}): accepted")
                write_record(output_file, {
                    "type": "probe_feature",
                    "report_id": f"0x{rid:02X}",
                    "result": "accepted",
                })
            except usb.core.USBError as e:
                print(f"  SET_REPORT(Feature, 0x{rid:02X}): {e}")

    except KeyboardInterrupt:
        print("\n\nProbe stopped by user.")

    print(f"\n--- Probed {probe_count} commands ---")
    release_interface(dev, args.interface)
    if output_file:
        output_file.close()
        print(f"Output saved to: {args.output}")


# ============================================================================
# Mode: Descriptor
# ============================================================================

def mode_descriptor(args):
    """
    Read and display HID descriptors for all interfaces of the MT2.

    The MT2 has 4 HID interfaces. Interface 2 (Actuator) is the most interesting
    for this investigation, but we dump all of them for completeness and
    cross-reference with our existing captures in notes/magic-mouse/.

    Interface layout:
      IF0 "Device Management"  — 83 bytes, vendor reports (battery, power state)
      IF1 "Trackpad / Boot"    — 110 bytes, mouse + multitouch
      IF2 "Actuator"           — 36 bytes, Taptic Engine control
      IF3 "Accelerometer"      — 27 bytes, motion data
    """
    dev = find_device(args.vid, args.pid)
    output_file = open_output_file(args.output)

    cfg = dev.get_active_configuration()
    print(f"\nConfiguration {cfg.bConfigurationValue}: {cfg.bNumInterfaces} interfaces")
    print(f"  wTotalLength: {cfg.wTotalLength}")
    print(f"  bmAttributes: 0x{cfg.bmAttributes:02X}")
    print(f"  bMaxPower: {cfg.bMaxPower} ({cfg.bMaxPower * 2}mA)")

    all_descriptors = {}

    for intf in cfg:
        iface_num = intf.bInterfaceNumber
        iface_name = INTERFACE_NAMES.get(iface_num, "Unknown")

        # Try to read interface string
        iface_string = ""
        if intf.iInterface > 0:
            try:
                iface_string = usb.util.get_string(dev, intf.iInterface)
            except Exception:
                iface_string = "(could not read)"

        print(f"\n{'=' * 70}")
        print(f"Interface {iface_num}: \"{iface_string or iface_name}\"")
        print(f"  bInterfaceClass   = 0x{intf.bInterfaceClass:02X} (HID)"
              if intf.bInterfaceClass == 3
              else f"  bInterfaceClass   = 0x{intf.bInterfaceClass:02X}")
        print(f"  bInterfaceSubClass= 0x{intf.bInterfaceSubClass:02X}"
              + (" (Boot Interface)" if intf.bInterfaceSubClass == 1 else ""))
        print(f"  bInterfaceProtocol= 0x{intf.bInterfaceProtocol:02X}"
              + (" (Mouse)" if intf.bInterfaceProtocol == 2
                 else " (Keyboard)" if intf.bInterfaceProtocol == 1 else ""))
        print(f"  bNumEndpoints     = {intf.bNumEndpoints}")

        # Parse embedded HID descriptor from extra data
        extra = None
        if hasattr(intf, 'extra_descriptors') and intf.extra_descriptors:
            extra = bytes(intf.extra_descriptors)
        elif hasattr(intf, 'extra') and intf.extra:
            extra = bytes(intf.extra)

        if extra and len(extra) >= 9 and extra[1] == 0x21:
            bcdHID = struct.unpack_from('<H', extra, 2)[0]
            country = extra[4]
            numDesc = extra[5]
            wDescLen = struct.unpack_from('<H', extra, 7)[0]
            print(f"  HID Descriptor:")
            print(f"    bcdHID         = 0x{bcdHID:04X}")
            print(f"    bCountryCode   = {country}")
            print(f"    bNumDescriptors= {numDesc}")
            print(f"    wDescriptorLen = {wDescLen}")

        # Endpoints
        for ep in intf:
            direction = "IN" if ep.bEndpointAddress & 0x80 else "OUT"
            xfer_type = {0: "CTRL", 1: "ISO", 2: "BULK", 3: "INT"}.get(
                ep.bmAttributes & 3, "?")
            print(f"  EP 0x{ep.bEndpointAddress:02X} ({direction} {xfer_type}): "
                  f"maxPacket={ep.wMaxPacketSize}, interval={ep.bInterval}ms")

            # Highlight the actuator OUT endpoint
            if iface_num == IF_ACTUATOR and direction == "OUT":
                print(f"    ^^^ This is the actuator command endpoint (Taptic Engine) ^^^")

        # Detach kernel driver and read HID report descriptor
        detach_kernel_driver(dev, iface_num)
        try:
            desc = dev.ctrl_transfer(
                0x81, USB_GET_DESCRIPTOR,
                (DT_HID_REPORT << 8),
                iface_num, 4096, timeout=2000)
            desc = bytes(desc)

            print(f"\n  HID Report Descriptor ({len(desc)} bytes):")
            print(f"  Raw hex: {hex_string(desc)}")
            print()

            # Parse and display
            lines, reports = parse_hid_descriptor(desc)
            for line in lines:
                print(f"    {line}")
            print()

            # Summary of report IDs
            if reports:
                print(f"  Report summary:")
                for rid, info in sorted(reports.items()):
                    parts = []
                    if info["input_bits"] > 0:
                        parts.append(f"Input={info['input_bits'] // 8}B")
                    if info["output_bits"] > 0:
                        parts.append(f"Output={info['output_bits'] // 8}B")
                    if info["feature_bits"] > 0:
                        parts.append(f"Feature={info['feature_bits'] // 8}B")
                    print(f"    Report ID 0x{rid:02X}: {', '.join(parts)}")

            # Extra annotation for IF2
            if iface_num == IF_ACTUATOR:
                print()
                print("  *** ACTUATOR INTERFACE ANALYSIS ***")
                print("  Report ID 0x3F: 15 bytes INPUT (device -> host)")
                print("    - Likely actuator status / acknowledgment")
                print("    - Same Report ID as IF1's digitizer report (different interface)")
                print("  Report ID 0x53: 63 bytes OUTPUT (host -> device)")
                print("    - This is the haptic command channel")
                print("    - macOS kernel sends commands here to trigger Taptic Engine")
                print("    - EP 0x04 (OUT INT) is the interrupt endpoint for this")
                print("    - Can also send via SET_REPORT(Output) on control endpoint")

            all_descriptors[iface_num] = {
                "name": iface_string or iface_name,
                "descriptor_length": len(desc),
                "descriptor_hex": hex_string(desc),
                "reports": {f"0x{k:02X}": v for k, v in reports.items()},
            }

            write_record(output_file, {
                "type": "descriptor",
                "interface": iface_num,
                "name": iface_string or iface_name,
                "descriptor_length": len(desc),
                "descriptor_hex": hex_string(desc),
                "reports": {f"0x{k:02X}": v for k, v in reports.items()},
            })

        except usb.core.USBError as e:
            print(f"  Failed to read descriptor: {e}")

    # Reattach drivers
    for intf in cfg:
        reattach_kernel_driver(dev, intf.bInterfaceNumber)

    # Cross-reference summary
    print(f"\n{'=' * 70}")
    print("CROSS-REFERENCE SUMMARY")
    print(f"{'=' * 70}")
    print()
    print("Expected layout from previous captures (notes/magic-mouse/):")
    print("  IF0 Device Management:  83B desc, Reports: 0xE0(IN 4B), 0x9A(IN 1B), 0x90(IN 2B)")
    print("  IF1 Trackpad / Boot:   110B desc, Reports: 0x02(IN 7B), 0x3F(IN 16B), 0x44(IN 1387B)")
    print("  IF2 Actuator:           36B desc, Reports: 0x3F(IN 15B), 0x53(OUT 63B)")
    print("  IF3 Accelerometer:      27B desc, Reports: 0xC0(IN 107B)")
    print()
    print("Actual from this capture:")
    for iface_num, info in sorted(all_descriptors.items()):
        reports_str = ", ".join(
            f"{rid}({'/'.join(f'{k}={v}B' for k, v in rinfo.items() if v > 0)})"
            for rid, rinfo in sorted(info["reports"].items())
        )
        print(f"  IF{iface_num} {info['name']:20s}: {info['descriptor_length']:3d}B desc, "
              f"Reports: {reports_str}")

    if output_file:
        output_file.close()
        print(f"\nOutput saved to: {args.output}")

    print("\nDone.")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Magic Trackpad 2 Taptic Engine (Actuator) protocol investigation tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Dump all HID descriptors
  sudo python3 capture_actuator.py --descriptor

  # Monitor actuator traffic (also capture trackpad for correlation)
  sudo python3 capture_actuator.py --monitor --also-trackpad -o actuator_capture.jsonl

  # Send a test command to the actuator
  sudo python3 capture_actuator.py --replay --command "53 01 80 00 00"

  # Probe the actuator command space interactively
  sudo python3 capture_actuator.py --probe -o probe_results.jsonl

  # Monitor a different interface
  sudo python3 capture_actuator.py --monitor --interface 0

Known MT2 USB interfaces:
  IF0: Device Management (vendor reports, battery)
  IF1: Trackpad / Boot (mouse + multitouch data)
  IF2: Actuator (Taptic Engine control) <-- primary target
  IF3: Accelerometer (motion data)

Actuator protocol (IF2):
  Report 0x3F (IN):  15 bytes — status/feedback from Taptic Engine
  Report 0x53 (OUT): 63 bytes — haptic commands TO Taptic Engine
  EP 0x84 (IN INT):  status reads, 16-byte max, 8ms interval
  EP 0x04 (OUT INT): command writes, 64-byte max, 2ms interval
""")

    # Mode selection (mutually exclusive)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--monitor", action="store_true",
                      help="Monitor actuator IN reports and optionally trackpad data")
    mode.add_argument("--replay", action="store_true",
                      help="Send a known command to the actuator")
    mode.add_argument("--probe", action="store_true",
                      help="Systematically probe the actuator command space")
    mode.add_argument("--descriptor", action="store_true",
                      help="Dump HID descriptors for all interfaces")

    # Device selection
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=DEFAULT_VID,
                        help=f"USB Vendor ID (default: 0x{DEFAULT_VID:04X})")
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=DEFAULT_PID,
                        help=f"USB Product ID (default: 0x{DEFAULT_PID:04X})")
    parser.add_argument("--interface", type=int, default=IF_ACTUATOR,
                        help=f"USB interface number (default: {IF_ACTUATOR} for Actuator)")

    # Monitor options
    parser.add_argument("--also-trackpad", action="store_true",
                        help="Also monitor trackpad interface (IF1) for correlation")

    # Replay options
    parser.add_argument("--command", type=str,
                        help="Hex string of the command to send (e.g., '53 01 80 00')")
    parser.add_argument("--file", type=str,
                        help="File containing hex command to send")
    parser.add_argument("--control-only", action="store_true",
                        help="Only use SET_REPORT control transfer (skip endpoint)")
    parser.add_argument("--endpoint-only", action="store_true",
                        help="Only use OUT interrupt endpoint (skip SET_REPORT)")

    # Output
    parser.add_argument("-o", "--output", type=str,
                        help="Output file path (JSON lines format)")

    args = parser.parse_args()

    # Dispatch to mode
    if args.monitor:
        mode_monitor(args)
    elif args.replay:
        mode_replay(args)
    elif args.probe:
        mode_probe(args)
    elif args.descriptor:
        mode_descriptor(args)


if __name__ == "__main__":
    main()
