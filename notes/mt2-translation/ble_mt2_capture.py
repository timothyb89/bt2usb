#!/usr/bin/env python3
"""
BLE capture and analysis tool for Apple Magic Trackpad 2 / 3.

Uses the `bleak` BLE library to discover, connect, enumerate GATT services,
read HID Report Maps, capture live multitouch data, and probe feature reports
over Bluetooth Low Energy.

Protocol references:
  - Linux kernel drivers/hid/hid-magicmouse.c (touch decoding, MT enable)
  - bt2usb firmware mt2.rs (USB-side emulation, touch synthesis)

Installation:
    pip3 install bleak

Usage:
    python3 ble_mt2_capture.py scan
    python3 ble_mt2_capture.py gatt --address AA:BB:CC:DD:EE:FF
    python3 ble_mt2_capture.py report-map --address AA:BB:CC:DD:EE:FF
    python3 ble_mt2_capture.py capture --address AA:BB:CC:DD:EE:FF
    python3 ble_mt2_capture.py features --address AA:BB:CC:DD:EE:FF
"""

import argparse
import asyncio
import json
import os
import signal
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
    from bleak.backends.descriptor import BleakGATTDescriptor
except ImportError:
    print("Install bleak first: pip3 install bleak")
    sys.exit(1)


async def ensure_paired(client: "BleakClient") -> None:
    """Pair with the device if not already bonded.

    BLE HID Service (0x1812) requires an encrypted connection. Without pairing,
    GATT discovery will not reveal HID characteristics. On Linux/BlueZ, bleak's
    pair() triggers SMP bonding. If already bonded, this is a no-op.
    """
    try:
        paired = await client.pair()
        if paired:
            print("  Paired successfully (bonded)")
        else:
            print("  Already paired/bonded")
    except NotImplementedError:
        # Some bleak backends don't support pair() — assume already paired
        print("  (pair() not supported on this backend, assuming already bonded)")
    except Exception as e:
        print(f"  Pairing attempt: {e}")
        print("  (continuing anyway — device may already be bonded)")


# ============================================================================
# Standard BLE UUIDs
# ============================================================================

# Standard BLE service UUIDs (16-bit shorthand -> full 128-bit)
KNOWN_SERVICES = {
    "00001800-0000-1000-8000-00805f9b34fb": "Generic Access (0x1800)",
    "00001801-0000-1000-8000-00805f9b34fb": "Generic Attribute (0x1801)",
    "0000180a-0000-1000-8000-00805f9b34fb": "Device Information (0x180A)",
    "0000180f-0000-1000-8000-00805f9b34fb": "Battery Service (0x180F)",
    "00001812-0000-1000-8000-00805f9b34fb": "HID Service (0x1812)",
}

# Standard BLE characteristic UUIDs
KNOWN_CHARACTERISTICS = {
    # Generic Access
    "00002a00-0000-1000-8000-00805f9b34fb": "Device Name (0x2A00)",
    "00002a01-0000-1000-8000-00805f9b34fb": "Appearance (0x2A01)",
    "00002a04-0000-1000-8000-00805f9b34fb": "Peripheral Preferred Connection Parameters (0x2A04)",
    # Device Information
    "00002a24-0000-1000-8000-00805f9b34fb": "Model Number String (0x2A24)",
    "00002a25-0000-1000-8000-00805f9b34fb": "Serial Number String (0x2A25)",
    "00002a26-0000-1000-8000-00805f9b34fb": "Firmware Revision String (0x2A26)",
    "00002a27-0000-1000-8000-00805f9b34fb": "Hardware Revision String (0x2A27)",
    "00002a28-0000-1000-8000-00805f9b34fb": "Software Revision String (0x2A28)",
    "00002a29-0000-1000-8000-00805f9b34fb": "Manufacturer Name String (0x2A29)",
    "00002a50-0000-1000-8000-00805f9b34fb": "PnP ID (0x2A50)",
    # Battery Service
    "00002a19-0000-1000-8000-00805f9b34fb": "Battery Level (0x2A19)",
    # HID Service
    "00002a4a-0000-1000-8000-00805f9b34fb": "HID Information (0x2A4A)",
    "00002a4b-0000-1000-8000-00805f9b34fb": "Report Map (0x2A4B)",
    "00002a4c-0000-1000-8000-00805f9b34fb": "HID Control Point (0x2A4C)",
    "00002a4d-0000-1000-8000-00805f9b34fb": "Report (0x2A4D)",
    "00002a4e-0000-1000-8000-00805f9b34fb": "Protocol Mode (0x2A4E)",
}

# Standard BLE descriptor UUIDs
KNOWN_DESCRIPTORS = {
    "00002900-0000-1000-8000-00805f9b34fb": "Characteristic Extended Properties (0x2900)",
    "00002901-0000-1000-8000-00805f9b34fb": "Characteristic User Description (0x2901)",
    "00002902-0000-1000-8000-00805f9b34fb": "Client Characteristic Configuration (0x2902)",
    "00002908-0000-1000-8000-00805f9b34fb": "Report Reference (0x2908)",
}

# UUID constants for comparison
UUID_HID_SERVICE = "00001812-0000-1000-8000-00805f9b34fb"
UUID_BATTERY_SERVICE = "0000180f-0000-1000-8000-00805f9b34fb"
UUID_DEVICE_INFO_SERVICE = "0000180a-0000-1000-8000-00805f9b34fb"
UUID_REPORT = "00002a4d-0000-1000-8000-00805f9b34fb"
UUID_REPORT_MAP = "00002a4b-0000-1000-8000-00805f9b34fb"
UUID_REPORT_REFERENCE = "00002908-0000-1000-8000-00805f9b34fb"
UUID_HID_INFORMATION = "00002a4a-0000-1000-8000-00805f9b34fb"
UUID_HID_CONTROL_POINT = "00002a4c-0000-1000-8000-00805f9b34fb"
UUID_PROTOCOL_MODE = "00002a4e-0000-1000-8000-00805f9b34fb"
UUID_BATTERY_LEVEL = "00002a19-0000-1000-8000-00805f9b34fb"
UUID_CCCD = "00002902-0000-1000-8000-00805f9b34fb"


# ============================================================================
# MT2-specific constants
# ============================================================================
# From Linux kernel hid-magicmouse.c and bt2usb firmware mt2.rs

# BLE report IDs
MT2_BT_REPORT_ID = 0x31        # Multitouch report over BLE (4-byte header + N*9 touch)
DOUBLE_REPORT_ID = 0xF7         # Two touch reports packed in one BLE notification
TRACKPAD_REPORT_ID = 0x28       # Original Magic Trackpad (not MT2)
MOUSE_REPORT_ID = 0x29          # Magic Mouse 1
MOUSE2_REPORT_ID = 0x12         # Magic Mouse 2

# Multitouch enable: SET_REPORT(Feature, report_id=0xF1, data={0x02, 0x01})
# Over BLE, the first byte is the report ID (0xF1), followed by the payload.
# This is equivalent to USB SET_REPORT(Feature, ID=0x02, data={0x01}) but
# wrapped in the BLE Feature Report characteristic for report ID 0xF1.
MT_ENABLE_REPORT_ID = 0xF1
MT_ENABLE_PAYLOAD = bytes([0xF1, 0x02, 0x01])

# USB-side multitouch enable for reference (not used in BLE mode):
# SET_REPORT(Feature, ID=0x02, data={0x01})
MT_ENABLE_USB = bytes([0x02, 0x01])

# MT2 physical surface coordinate ranges (from kernel driver)
MT2_X_MIN = -3678
MT2_X_MAX = 3934
MT2_Y_MIN = -2478
MT2_Y_MAX = 2587

# Touch state flags (byte 3 of touch data, upper 2 bits)
TOUCH_STATE_NONE = 0x00
TOUCH_STATE_START = 0x40       # Finger approaching / just placed
TOUCH_STATE_DOWN = 0x80        # Finger in contact
TOUCH_STATE_RELEASE = 0xC0     # Finger lifting

# BLE header for report ID 0x31 / 0x28:
#   byte 0: report ID
#   byte 1: clicks (button state) + timestamp bits [7:6]
#   byte 2: timestamp bits [9:2]
#   byte 3: timestamp bits [17:10]
# Touch data starts at offset 4, each finger is 9 bytes.
BLE_MT2_HEADER_SIZE = 4

# BT version identifiers
TRACKPAD2_2021_BT_VERSION = 0x110
TRACKPAD_2024_BT_VERSION = 0x314  # MT3 (USB-C)

# Report Reference descriptor type values
REPORT_TYPE_INPUT = 0x01
REPORT_TYPE_OUTPUT = 0x02
REPORT_TYPE_FEATURE = 0x03

REPORT_TYPE_NAMES = {
    REPORT_TYPE_INPUT: "Input",
    REPORT_TYPE_OUTPUT: "Output",
    REPORT_TYPE_FEATURE: "Feature",
}


# ============================================================================
# HID Report Descriptor parser
# ============================================================================

# Usage page names
USAGE_PAGE_NAMES = {
    0x01: "Generic Desktop",
    0x02: "Simulation Controls",
    0x05: "Game Controls",
    0x06: "Generic Device Controls",
    0x07: "Keyboard/Keypad",
    0x08: "LED",
    0x09: "Button",
    0x0C: "Consumer",
    0x0D: "Digitizer",
    0x84: "Power Device",
    0x85: "Battery System",
}

# Generic Desktop usages
USAGE_NAMES_01 = {
    0x00: "Undefined",
    0x01: "Pointer",
    0x02: "Mouse",
    0x04: "Joystick",
    0x05: "Game Pad",
    0x06: "Keyboard",
    0x07: "Keypad",
    0x08: "Multi-axis Controller",
    0x30: "X",
    0x31: "Y",
    0x32: "Z",
    0x33: "Rx",
    0x34: "Ry",
    0x35: "Rz",
    0x36: "Slider",
    0x37: "Dial",
    0x38: "Wheel",
    0x39: "Hat Switch",
    0x48: "Resolution Multiplier",
}

# Digitizer usages
USAGE_NAMES_0D = {
    0x00: "Undefined",
    0x01: "Digitizer",
    0x02: "Pen",
    0x04: "Touch Screen",
    0x05: "Touch Pad",
    0x0E: "Device Configuration",
    0x22: "Finger",
    0x30: "Tip Pressure",
    0x32: "In Range",
    0x33: "Touch",
    0x42: "Tip Switch",
    0x47: "Confidence",
    0x48: "Width",
    0x49: "Height",
    0x51: "Contact Identifier",
    0x54: "Contact Count",
    0x55: "Contact Count Maximum",
    0x56: "Scan Time",
}

# Consumer page usages (partial)
USAGE_NAMES_0C = {
    0x0238: "AC Pan",
}


def usage_name(page: int, usage: int) -> str:
    """Look up a human-readable name for a HID usage."""
    if page >= 0xFF00:
        return f"Vendor 0x{usage:04X}"
    names = {
        0x01: USAGE_NAMES_01,
        0x0D: USAGE_NAMES_0D,
        0x0C: USAGE_NAMES_0C,
    }.get(page, {})
    name = names.get(usage)
    if name:
        return name
    return f"0x{usage:04X}"


class HidReportField:
    """A single field (Input/Output/Feature) within a HID report."""

    def __init__(self, report_type: str, report_id: int, usage_page: int,
                 usages: list, logical_min: int, logical_max: int,
                 physical_min: int, physical_max: int,
                 report_size: int, report_count: int, flags: int):
        self.report_type = report_type
        self.report_id = report_id
        self.usage_page = usage_page
        self.usages = usages
        self.logical_min = logical_min
        self.logical_max = logical_max
        self.physical_min = physical_min
        self.physical_max = physical_max
        self.report_size = report_size
        self.report_count = report_count
        self.flags = flags

    @property
    def total_bits(self) -> int:
        return self.report_size * self.report_count

    def flag_str(self) -> str:
        parts = []
        parts.append("Const" if self.flags & 0x01 else "Data")
        parts.append("Variable" if self.flags & 0x02 else "Array")
        parts.append("Relative" if self.flags & 0x04 else "Absolute")
        if self.flags & 0x08:
            parts.append("Wrap")
        if self.flags & 0x10:
            parts.append("NonLinear")
        if self.flags & 0x20:
            parts.append("NoPreferred")
        if self.flags & 0x40:
            parts.append("NullState")
        if self.flags & 0x80:
            parts.append("Volatile")
        if self.flags & 0x100:
            parts.append("BufferedBytes")
        return ", ".join(parts)


def parse_hid_report_descriptor(data: bytes) -> list[HidReportField]:
    """
    Parse a HID Report Descriptor into a list of HidReportField objects.

    This implements the HID spec's item parser, tracking global and local
    state as items are encountered. Main items (Input/Output/Feature) emit
    fields; collection items adjust nesting.
    """
    fields: list[HidReportField] = []

    # Global state
    usage_page = 0
    logical_min = 0
    logical_max = 0
    physical_min = 0
    physical_max = 0
    report_size = 0
    report_count = 0
    report_id = 0

    # Local state (reset after each Main item)
    usages: list[int] = []
    usage_min = 0
    usage_max = 0

    i = 0
    while i < len(data):
        b = data[i]

        # Long item
        if b == 0xFE:
            if i + 2 >= len(data):
                break
            data_size = data[i + 1]
            i += 3 + data_size
            continue

        # Short item
        bSize = b & 0x03
        if bSize == 3:
            bSize = 4
        bType = (b >> 2) & 0x03
        bTag = (b >> 4) & 0x0F

        # Read value (signed for certain items)
        val = 0
        for j in range(bSize):
            if i + 1 + j < len(data):
                val |= data[i + 1 + j] << (8 * j)

        # Sign-extend for items that use signed values
        def signed_val(v: int, size: int) -> int:
            if size > 0 and v >= (1 << (size * 8 - 1)):
                v -= 1 << (size * 8)
            return v

        if bType == 0:  # Main
            if bTag in (0x08, 0x09, 0x0B):  # Input, Output, Feature
                type_name = {0x08: "Input", 0x09: "Output", 0x0B: "Feature"}[bTag]

                # Build usage list
                actual_usages = list(usages)
                if not actual_usages and usage_max >= usage_min:
                    actual_usages = list(range(usage_min, usage_max + 1))

                fields.append(HidReportField(
                    report_type=type_name,
                    report_id=report_id,
                    usage_page=usage_page,
                    usages=actual_usages,
                    logical_min=logical_min,
                    logical_max=logical_max,
                    physical_min=physical_min,
                    physical_max=physical_max,
                    report_size=report_size,
                    report_count=report_count,
                    flags=val,
                ))

                # Reset local state
                usages = []
                usage_min = 0
                usage_max = 0

            elif bTag == 0x0A:  # Collection
                usages = []
                usage_min = 0
                usage_max = 0

            elif bTag == 0x0C:  # End Collection
                pass

        elif bType == 1:  # Global
            if bTag == 0x00:
                usage_page = val
            elif bTag == 0x01:
                logical_min = signed_val(val, bSize)
            elif bTag == 0x02:
                logical_max = signed_val(val, bSize)
            elif bTag == 0x03:
                physical_min = signed_val(val, bSize)
            elif bTag == 0x04:
                physical_max = signed_val(val, bSize)
            elif bTag == 0x07:
                report_size = val
            elif bTag == 0x08:
                report_id = val
            elif bTag == 0x09:
                report_count = val

        elif bType == 2:  # Local
            if bTag == 0x00:
                usages.append(val)
            elif bTag == 0x01:
                usage_min = val
            elif bTag == 0x02:
                usage_max = val

        i += 1 + bSize

    return fields


def print_report_descriptor_parsed(data: bytes) -> None:
    """Print a human-readable parse of a HID Report Descriptor."""
    indent = 0
    i = 0
    while i < len(data):
        b = data[i]

        # Long item
        if b == 0xFE:
            if i + 2 >= len(data):
                break
            data_size = data[i + 1]
            print(f"    {'  ' * indent}[{i:3d}] LONG ITEM ({data_size} bytes)")
            i += 3 + data_size
            continue

        # Short item
        bSize = b & 0x03
        if bSize == 3:
            bSize = 4
        bType = (b >> 2) & 0x03
        bTag = (b >> 4) & 0x0F

        # Read value
        val = 0
        raw_bytes = [b]
        for j in range(bSize):
            if i + 1 + j < len(data):
                val |= data[i + 1 + j] << (8 * j)
                raw_bytes.append(data[i + 1 + j])

        # Sign extend for logical min/max
        def signed_val(v: int, size: int) -> int:
            if size > 0 and v >= (1 << (size * 8 - 1)):
                v -= 1 << (size * 8)
            return v

        hex_str = " ".join(f"{x:02x}" for x in raw_bytes)

        desc = ""
        if bType == 0:  # Main
            if bTag == 0x08:
                flags = []
                flags.append("Const" if val & 0x01 else "Data")
                flags.append("Variable" if val & 0x02 else "Array")
                flags.append("Relative" if val & 0x04 else "Absolute")
                if val & 0x20:
                    flags.append("NoPreferred")
                desc = f"Input ({', '.join(flags)})"
            elif bTag == 0x09:
                flags = []
                flags.append("Const" if val & 0x01 else "Data")
                flags.append("Variable" if val & 0x02 else "Array")
                desc = f"Output ({', '.join(flags)})"
            elif bTag == 0x0B:
                flags = []
                flags.append("Const" if val & 0x01 else "Data")
                flags.append("Variable" if val & 0x02 else "Array")
                flags.append("Relative" if val & 0x04 else "Absolute")
                if val & 0x80:
                    flags.append("Volatile")
                desc = f"Feature ({', '.join(flags)})"
            elif bTag == 0x0A:
                ctype = {
                    0: "Physical", 1: "Application", 2: "Logical",
                    3: "Report", 4: "Named Array", 5: "Usage Switch",
                    6: "Usage Modifier",
                }.get(val, f"0x{val:02X}")
                desc = f"Collection ({ctype})"
            elif bTag == 0x0C:
                desc = "End Collection"

        elif bType == 1:  # Global
            names = {
                0x00: "Usage Page",
                0x01: "Logical Minimum",
                0x02: "Logical Maximum",
                0x03: "Physical Minimum",
                0x04: "Physical Maximum",
                0x05: "Unit Exponent",
                0x06: "Unit",
                0x07: "Report Size",
                0x08: "Report ID",
                0x09: "Report Count",
                0x0A: "Push",
                0x0B: "Pop",
            }
            name = names.get(bTag, f"Global(0x{bTag:X})")
            if bTag == 0x00:  # Usage Page
                if val >= 0xFF00:
                    desc = f"{name} (Vendor 0x{val:04X})"
                else:
                    page_name = USAGE_PAGE_NAMES.get(val, f"0x{val:04X}")
                    desc = f"{name} ({page_name})"
            elif bTag == 0x08:  # Report ID
                desc = f"{name} (0x{val:02X})"
            elif bTag in (0x01, 0x02, 0x03, 0x04):  # Signed min/max
                sval = signed_val(val, bSize)
                desc = f"{name} ({sval})"
            else:
                desc = f"{name} ({val})"

        elif bType == 2:  # Local
            names = {
                0x00: "Usage",
                0x01: "Usage Minimum",
                0x02: "Usage Maximum",
                0x03: "Designator Index",
                0x04: "Designator Minimum",
                0x05: "Designator Maximum",
                0x07: "String Index",
                0x08: "String Minimum",
                0x09: "String Maximum",
                0x0A: "Delimiter",
            }
            name = names.get(bTag, f"Local(0x{bTag:X})")
            desc = f"{name} (0x{val:04X})"

        # Adjust indent for collections
        if bType == 0 and bTag == 0x0C:
            indent = max(0, indent - 1)

        pad = "  " * indent
        print(f"    {pad}[{i:3d}] {hex_str:24s} {desc}")

        if bType == 0 and bTag == 0x0A:
            indent += 1

        i += 1 + bSize


# ============================================================================
# MT2 touch point decoder
# ============================================================================

def decode_mt2_touch(tdata: bytes) -> Optional[dict]:
    """
    Decode a 9-byte MT2/MT3 touch point.

    Field layout (from Linux kernel hid-magicmouse.c, magicmouse_emit_touch):

      tdata[0..1]: X coordinate, 13-bit signed
        x = (tdata[1] << 27 | tdata[0] << 19) >> 19

      tdata[1..3]: Y coordinate, 13-bit signed, negated
        y = -((tdata[3] << 30 | tdata[2] << 22 | tdata[1] << 14) >> 19)

        Note: X uses bits from tdata[0] and lower bits of tdata[1].
              Y uses upper bits of tdata[1], tdata[2], and lower bits of tdata[3].
              The bit fields overlap in tdata[1], which is shared between X and Y.

      tdata[3] & 0xC0: Touch state (upper 2 bits)
        0x00 = none/not touching
        0x40 = approaching / start
        0x80 = down / in contact
        0xC0 = releasing / lifting

      tdata[4]: touch_major (contact ellipse major axis)
      tdata[5]: touch_minor (contact ellipse minor axis)
      tdata[6]: size (contact area, used for palm rejection)
      tdata[7]: pressure (0-255, used for force touch on MT2)

      tdata[8] & 0x0F: tracking ID (0-15, assigned by firmware)
      tdata[8] >> 5 - 4: orientation (-4 to +3, contact ellipse angle)

    All coordinates are in the MT2 native coordinate space:
      X: [-3678, +3934] (160mm physical)
      Y: [-2478, +2587] (114.9mm physical)
    """
    if len(tdata) < 9:
        return None

    # X: 13-bit signed. Must simulate C 32-bit signed arithmetic.
    # Python has arbitrary-precision ints, so we mask to 32 bits then sign-extend.
    x_raw = ((tdata[1] << 27) | (tdata[0] << 19)) & 0xFFFFFFFF
    if x_raw >= 0x80000000:
        x_raw -= 0x100000000
    x = x_raw >> 19  # arithmetic right shift (Python preserves sign)

    # Y: 13-bit signed, negated.
    y_raw = ((tdata[3] << 30) | (tdata[2] << 22) | (tdata[1] << 14)) & 0xFFFFFFFF
    if y_raw >= 0x80000000:
        y_raw -= 0x100000000
    y = -(y_raw >> 19)

    state = tdata[3] & 0xC0
    touch_major = tdata[4]
    touch_minor = tdata[5]
    size = tdata[6]
    pressure = tdata[7]
    tracking_id = tdata[8] & 0x0F
    orientation = (tdata[8] >> 5) - 4

    state_names = {
        0x00: "NONE",
        0x40: "START",
        0x80: "DOWN",
        0xC0: "RELEASE",
    }

    return {
        "id": tracking_id,
        "x": x,
        "y": y,
        "state": state,
        "state_name": state_names.get(state, f"0x{state:02X}"),
        "down": state == 0x80,
        "major": touch_major,
        "minor": touch_minor,
        "size": size,
        "pressure": pressure,
        "orientation": orientation,
    }


def decode_ble_timestamp(data: bytes) -> int:
    """
    Extract the 18-bit timestamp from a BLE MT2 report header.

    From hid-magicmouse.c:
      ts = data[1] >> 6 | data[2] << 2 | data[3] << 10

    The timestamp is in device-internal ticks (not wall clock).
    """
    if len(data) < 4:
        return 0
    return (data[1] >> 6) | (data[2] << 2) | (data[3] << 10)


# ============================================================================
# Scan mode
# ============================================================================

async def cmd_scan(args: argparse.Namespace) -> None:
    """Discover nearby BLE devices, highlighting Apple / Magic Trackpad devices."""
    duration = getattr(args, "duration", None) or 10.0
    print(f"Scanning for BLE devices ({duration}s)...")
    print()

    devices = await BleakScanner.discover(timeout=duration, return_adv=True)

    # Sort by RSSI (strongest first)
    sorted_devices = sorted(
        devices.values(),
        key=lambda x: x[1].rssi if x[1].rssi is not None else -999,
        reverse=True,
    )

    apple_found = []

    for device, adv_data in sorted_devices:
        name = adv_data.local_name or device.name or "(unknown)"
        rssi = adv_data.rssi if adv_data.rssi is not None else "?"
        addr = device.address
        addr_type = getattr(device.details, "address_type", "?") if hasattr(device, "details") else "?"

        # Check for Apple / Magic Trackpad
        is_apple = False
        name_lower = name.lower() if name else ""
        if any(kw in name_lower for kw in ["magic", "trackpad", "apple", "magic mouse"]):
            is_apple = True

        # Check for Apple manufacturer data (company ID 0x004C)
        mfr_data = adv_data.manufacturer_data
        if 0x004C in mfr_data:
            is_apple = True

        # Service UUIDs
        svc_uuids = adv_data.service_uuids or []
        svc_str = ", ".join(svc_uuids) if svc_uuids else "(none)"

        marker = " <<< APPLE" if is_apple else ""
        if is_apple:
            apple_found.append((addr, name, rssi))

        print(f"  {addr}  RSSI={rssi:>4}  Name=\"{name}\"{marker}")
        if svc_uuids:
            for uuid in svc_uuids:
                svc_name = KNOWN_SERVICES.get(uuid, "")
                label = f" = {svc_name}" if svc_name else ""
                print(f"    Service: {uuid}{label}")
        if mfr_data:
            for company_id, payload in mfr_data.items():
                hex_str = " ".join(f"{b:02x}" for b in payload[:32])
                company = "Apple" if company_id == 0x004C else f"0x{company_id:04X}"
                if len(payload) > 32:
                    hex_str += f" ... ({len(payload)}B total)"
                print(f"    Manufacturer ({company}): {hex_str}")

    print()
    print(f"Total: {len(sorted_devices)} device(s) found")
    if apple_found:
        print()
        print("Apple devices found:")
        for addr, name, rssi in apple_found:
            print(f"  {addr}  \"{name}\"  RSSI={rssi}")
        print()
        print("To connect, use:")
        print(f"  python3 {sys.argv[0]} gatt --address {apple_found[0][0]}")
    else:
        print("No Apple / Magic Trackpad devices found.")
        print("Make sure the trackpad is in pairing mode (hold power button).")


# ============================================================================
# GATT dump mode
# ============================================================================

async def cmd_gatt(args: argparse.Namespace) -> None:
    """Connect and enumerate all GATT services, characteristics, and descriptors."""
    address = args.address
    if not address:
        print("Error: --address is required for gatt mode")
        sys.exit(1)

    print(f"Connecting to {address}...")
    print("  Hint: disconnect the device from bluetoothctl first ('disconnect' cmd)")
    print("  Hint: unload kernel HID driver: sudo rmmod hid_magicmouse hid_generic")
    print()
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect!")
            return

        print(f"Connected to {address}")
        await ensure_paired(client)

        services = client.services
        svc_list = list(services)
        print(f"  Services discovered: {len(svc_list)}")
        if not svc_list:
            print()
            print("  WARNING: No services found!")
            print("  This usually means BlueZ's input plugin or a kernel HID driver")
            print("  has claimed the device's GATT connection.")
            print()
            print("  Try this sequence:")
            print("    1. sudo rmmod hid_magicmouse hid_generic  (unload HID drivers)")
            print("    2. bluetoothctl -> disconnect <address>    (release BLE connection)")
            print("    3. bluetoothctl -> remove <address>        (clear cached services)")
            print("    4. Re-run this script (it will re-pair)")
            print()
            print("  If that doesn't work, try disabling BlueZ input plugin:")
            print("    Edit /etc/bluetooth/input.conf and set ClassicBondedOnly=true")
            print("    Then restart bluetoothd: sudo systemctl restart bluetooth")
            return

        print()
        for service in svc_list:
            svc_uuid = service.uuid
            svc_name = KNOWN_SERVICES.get(svc_uuid, "")

            # Flag Apple-proprietary services
            is_apple = False
            uuid_upper = svc_uuid.upper()
            # Apple uses various proprietary base UUIDs
            if any(base in uuid_upper for base in [
                "D0611E78-BBB4",  # Apple Notification Center
                "7905F431-B5CE",  # Apple Media Service
                "89D3502B-0F36",  # Apple continuity
                "9FA480E0-4967",  # Apple current time
            ]):
                is_apple = True
            # Non-standard 128-bit UUIDs that are not in the BT SIG range
            if not svc_uuid.startswith("0000") or not svc_uuid.endswith("0000-1000-8000-00805f9b34fb"):
                if not is_apple:
                    svc_name = svc_name or "Non-standard (possibly Apple proprietary)"

            label = f" -- {svc_name}" if svc_name else ""
            is_hid = svc_uuid == UUID_HID_SERVICE
            is_battery = svc_uuid == UUID_BATTERY_SERVICE

            marker = ""
            if is_hid:
                marker = " [HID]"
            elif is_battery:
                marker = " [BATTERY]"
            elif is_apple:
                marker = " [APPLE PROPRIETARY]"

            print(f"{'=' * 70}")
            print(f"Service: {svc_uuid}{label}{marker}")
            print(f"  Handle range: {service.handle}")
            print()

            for char in service.characteristics:
                char_uuid = char.uuid
                char_name = KNOWN_CHARACTERISTICS.get(char_uuid, "")
                props = ", ".join(char.properties)

                label = f" -- {char_name}" if char_name else ""
                print(f"  Characteristic: {char_uuid}{label}")
                print(f"    Handle: {char.handle}")
                print(f"    Properties: {props}")

                # Try to read the value if readable
                if "read" in char.properties:
                    try:
                        value = await client.read_gatt_char(char)
                        hex_str = " ".join(f"{b:02x}" for b in value)
                        # Also try to decode as UTF-8 string
                        try:
                            text = value.decode("utf-8")
                            print(f"    Value: [{len(value)}B] {hex_str}")
                            print(f"    As string: \"{text}\"")
                        except (UnicodeDecodeError, ValueError):
                            print(f"    Value: [{len(value)}B] {hex_str}")
                    except Exception as e:
                        print(f"    Value: (read failed: {e})")

                # Read descriptors
                for desc in char.descriptors:
                    desc_uuid = desc.uuid
                    desc_name = KNOWN_DESCRIPTORS.get(desc_uuid, "")
                    label = f" -- {desc_name}" if desc_name else ""

                    try:
                        desc_value = await client.read_gatt_descriptor(desc.handle)
                        hex_str = " ".join(f"{b:02x}" for b in desc_value)

                        # Decode Report Reference (0x2908)
                        if desc_uuid == UUID_REPORT_REFERENCE and len(desc_value) >= 2:
                            rid = desc_value[0]
                            rtype = desc_value[1]
                            rtype_name = REPORT_TYPE_NAMES.get(rtype, f"Unknown(0x{rtype:02X})")
                            print(f"    Descriptor: {desc_uuid}{label}")
                            print(f"      Handle: {desc.handle}")
                            print(f"      Value: [{len(desc_value)}B] {hex_str}")
                            print(f"      -> Report ID: 0x{rid:02X}, Type: {rtype_name}")
                        # Decode CCCD (0x2902)
                        elif desc_uuid == UUID_CCCD and len(desc_value) >= 2:
                            cccd_val = desc_value[0] | (desc_value[1] << 8)
                            notif = "Notifications ON" if cccd_val & 0x01 else "Notifications OFF"
                            indic = ", Indications ON" if cccd_val & 0x02 else ""
                            print(f"    Descriptor: {desc_uuid}{label}")
                            print(f"      Handle: {desc.handle}")
                            print(f"      Value: [{len(desc_value)}B] {hex_str} ({notif}{indic})")
                        else:
                            print(f"    Descriptor: {desc_uuid}{label}")
                            print(f"      Handle: {desc.handle}")
                            print(f"      Value: [{len(desc_value)}B] {hex_str}")
                    except Exception as e:
                        print(f"    Descriptor: {desc_uuid}{label}")
                        print(f"      Handle: {desc.handle}")
                        print(f"      Value: (read failed: {e})")

                print()


# ============================================================================
# Report Map mode
# ============================================================================

async def cmd_report_map(args: argparse.Namespace) -> None:
    """Connect and read+parse the HID Report Map characteristic."""
    address = args.address
    if not address:
        print("Error: --address is required for report-map mode")
        sys.exit(1)

    print(f"Connecting to {address}...")
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect!")
            return

        print(f"Connected to {address}")
        await ensure_paired(client)
        print()

        # Find Report Map characteristic(s)
        found = False
        for service in client.services:
            if service.uuid != UUID_HID_SERVICE:
                continue

            for char in service.characteristics:
                if char.uuid != UUID_REPORT_MAP:
                    continue

                found = True
                print(f"Reading Report Map from handle {char.handle}...")
                try:
                    data = await client.read_gatt_char(char)
                except Exception as e:
                    print(f"  Failed to read: {e}")
                    continue

                print(f"Report Map: {len(data)} bytes")
                print()

                # Raw hex dump
                print("Raw hex:")
                for offset in range(0, len(data), 16):
                    chunk = data[offset:offset + 16]
                    hex_str = " ".join(f"{b:02x}" for b in chunk)
                    print(f"  [{offset:4d}] {hex_str}")
                print()

                # Parsed items
                print("Parsed items:")
                print_report_descriptor_parsed(data)
                print()

                # Structured summary
                fields = parse_hid_report_descriptor(data)
                report_ids: dict[int, dict[str, int]] = defaultdict(lambda: defaultdict(int))

                for field in fields:
                    rid = field.report_id
                    rtype = field.report_type
                    report_ids[rid][rtype] += field.total_bits

                print("Report summary:")
                print(f"  {'ID':>4s}  {'Type':>8s}  {'Bits':>6s}  {'Bytes':>6s}  Notes")
                print(f"  {'----':>4s}  {'--------':>8s}  {'------':>6s}  {'------':>6s}  -----")
                for rid in sorted(report_ids.keys()):
                    for rtype in sorted(report_ids[rid].keys()):
                        bits = report_ids[rid][rtype]
                        byte_count = (bits + 7) // 8
                        notes = ""
                        if rid == MT2_BT_REPORT_ID:
                            notes = "MT2 BLE multitouch"
                        elif rid == DOUBLE_REPORT_ID:
                            notes = "Double-report wrapper"
                        elif rid == MT_ENABLE_REPORT_ID:
                            notes = "MT enable / feature control"
                        print(f"  0x{rid:02X}  {rtype:>8s}  {bits:6d}  {byte_count:6d}  {notes}")

                print()

                # Per-field detail
                print("Field details:")
                for field in fields:
                    page_name = USAGE_PAGE_NAMES.get(field.usage_page, f"0x{field.usage_page:04X}")
                    if field.usage_page >= 0xFF00:
                        page_name = f"Vendor 0x{field.usage_page:04X}"

                    usage_strs = []
                    for u in field.usages[:8]:
                        usage_strs.append(usage_name(field.usage_page, u))
                    if len(field.usages) > 8:
                        usage_strs.append(f"... +{len(field.usages) - 8} more")
                    usage_str = ", ".join(usage_strs) if usage_strs else "(none)"

                    print(f"  {field.report_type} (ID=0x{field.report_id:02X}): "
                          f"{field.report_count}x {field.report_size}bit = {field.total_bits}bit")
                    print(f"    Page: {page_name}, Usages: [{usage_str}]")
                    print(f"    Logical: [{field.logical_min}, {field.logical_max}]"
                          f"  Physical: [{field.physical_min}, {field.physical_max}]")
                    print(f"    Flags: {field.flag_str()}")

        if not found:
            print("No HID Service or Report Map characteristic found!")
            print("Is this device paired? BLE HID requires an encrypted connection.")


# ============================================================================
# Capture mode
# ============================================================================

async def cmd_capture(args: argparse.Namespace) -> None:
    """
    Connect, activate multitouch, and capture live touch data.

    This subscribes to all Input Report notifications on the HID Service,
    then sends the MT enable command to activate multitouch reporting.

    Report flow over BLE:
      1. Each Report characteristic (UUID 0x2A4D) has a Report Reference
         descriptor (UUID 0x2908) that identifies its report ID and type.
      2. Input reports arrive as GATT notifications on the corresponding
         Report characteristic.
      3. The report data includes the report ID as the first byte (unlike
         USB HID where the report ID is implicit from the endpoint).
      4. For MT2 BLE, touch reports use report ID 0x31 with a 4-byte header
         followed by N*9 bytes of touch point data.
      5. Double-reports (ID 0xF7) pack two reports: byte 1 = length of
         first sub-report, bytes 2..2+len = first report, remainder = second.
    """
    address = args.address
    if not address:
        print("Error: --address is required for capture mode")
        sys.exit(1)

    duration = getattr(args, "duration", None)
    output_file = getattr(args, "output", None)

    print(f"Connecting to {address}...")

    # Statistics
    stats = {
        "total_notifications": 0,
        "report_ids": defaultdict(int),
        "touch_reports": 0,
        "double_reports": 0,
        "max_fingers": 0,
        "start_time": None,
        "last_time": None,
    }

    output_fh = None
    if output_file:
        output_fh = open(output_file, "w")
        print(f"Saving capture data to: {output_file}")

    stop_event = asyncio.Event()

    def signal_handler(sig, frame):
        print("\nCtrl+C received, stopping capture...")
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect!")
            return

        print(f"Connected to {address}")
        await ensure_paired(client)
        print()

        # ---- Step 1: Discover Report characteristics and their report IDs ----
        report_chars: list[tuple[BleakGATTCharacteristic, int, int]] = []  # (char, report_id, report_type)
        feature_chars: list[tuple[BleakGATTCharacteristic, int]] = []  # (char, report_id)
        mt_enable_char: Optional[BleakGATTCharacteristic] = None

        for service in client.services:
            if service.uuid != UUID_HID_SERVICE:
                continue

            print(f"HID Service found (handle {service.handle})")

            for char in service.characteristics:
                if char.uuid != UUID_REPORT:
                    continue

                # Read Report Reference descriptor
                report_id = 0
                report_type = 0
                for desc in char.descriptors:
                    if desc.uuid == UUID_REPORT_REFERENCE:
                        try:
                            ref = await client.read_gatt_descriptor(desc.handle)
                            if len(ref) >= 2:
                                report_id = ref[0]
                                report_type = ref[1]
                        except Exception as e:
                            print(f"  Warning: could not read Report Reference: {e}")

                type_name = REPORT_TYPE_NAMES.get(report_type, f"Unknown(0x{report_type:02X})")
                props = ", ".join(char.properties)
                print(f"  Report char handle={char.handle}: "
                      f"ID=0x{report_id:02X} Type={type_name} Props=[{props}]")

                report_chars.append((char, report_id, report_type))

                if report_type == REPORT_TYPE_FEATURE:
                    feature_chars.append((char, report_id))
                    # Check if this is the MT enable characteristic (report ID 0xF1)
                    if report_id == MT_ENABLE_REPORT_ID:
                        mt_enable_char = char

        print()

        # ---- Step 2: Define notification handler ----

        def notification_handler(char: BleakGATTCharacteristic, data: bytearray) -> None:
            """Handle incoming BLE HID notifications."""
            now = time.time()
            if stats["start_time"] is None:
                stats["start_time"] = now
            stats["last_time"] = now
            stats["total_notifications"] += 1

            data = bytes(data)
            if len(data) == 0:
                return

            report_id = data[0]
            stats["report_ids"][report_id] += 1
            size = len(data)

            hex_str = " ".join(f"{b:02x}" for b in data[:80])
            if size > 80:
                hex_str += f" ... ({size}B total)"

            timestamp_str = f"{now:.6f}"
            count = stats["total_notifications"]

            print(f"[{count:5d}] t={timestamp_str} handle={char.handle} "
                  f"RID=0x{report_id:02X} [{size:3d}B] {hex_str}")

            # Save to output file
            if output_fh:
                record = {
                    "seq": count,
                    "time": now,
                    "handle": char.handle,
                    "report_id": report_id,
                    "size": size,
                    "hex": data.hex(),
                }
                output_fh.write(json.dumps(record) + "\n")

            # ---- Decode MT2 BLE touch reports (report ID 0x31) ----
            #
            # BLE report format (from hid-magicmouse.c):
            #   byte 0:   report ID (0x31)
            #   byte 1:   clicks (button) | timestamp[7:6]
            #   byte 2:   timestamp[9:2]
            #   byte 3:   timestamp[17:10]
            #   byte 4..: N * 9 bytes of touch point data
            #
            # The header is 4 bytes. Touch data size must be a multiple of 9.
            # Maximum 15 simultaneous touch points.
            if report_id == MT2_BT_REPORT_ID:
                _decode_and_print_mt2_report(data, stats)

            # ---- Decode double-reports (report ID 0xF7) ----
            #
            # From hid-magicmouse.c:
            #   byte 0: 0xF7 (report ID)
            #   byte 1: length of first sub-report
            #   byte 2..2+len1: first sub-report (starts with its own report ID)
            #   byte 2+len1..: second sub-report (starts with its own report ID)
            #
            # This packing occurs when the device has two reports ready in the
            # same connection interval. Both sub-reports are decoded recursively.
            elif report_id == DOUBLE_REPORT_ID:
                stats["double_reports"] += 1
                if size >= 3:
                    len1 = data[1]
                    if 2 + len1 <= size:
                        sub1 = data[2:2 + len1]
                        sub2 = data[2 + len1:]
                        print(f"        -> Double report: sub1={len1}B, sub2={len(sub2)}B")
                        if len(sub1) > 0 and sub1[0] == MT2_BT_REPORT_ID:
                            print(f"        Sub-report 1:")
                            _decode_and_print_mt2_report(bytes(sub1), stats, indent=12)
                        if len(sub2) > 0 and sub2[0] == MT2_BT_REPORT_ID:
                            print(f"        Sub-report 2:")
                            _decode_and_print_mt2_report(bytes(sub2), stats, indent=12)

        # ---- Step 3: Subscribe to all Input Report notifications ----
        print("Subscribing to Input Report notifications...")
        subscribed_count = 0
        for char, report_id, report_type in report_chars:
            if report_type == REPORT_TYPE_INPUT and "notify" in char.properties:
                try:
                    await client.start_notify(char, notification_handler)
                    print(f"  Subscribed: handle={char.handle} ID=0x{report_id:02X}")
                    subscribed_count += 1
                except Exception as e:
                    print(f"  Failed to subscribe handle={char.handle}: {e}")

        if subscribed_count == 0:
            print("  WARNING: No Input Report characteristics subscribed!")
            print("  Is the device paired? BLE HID requires encryption.")
        print()

        # ---- Step 4: Activate multitouch ----
        #
        # The Magic Trackpad 2/3 requires an explicit Feature Report write to
        # enable multitouch data. Over BLE, this is done by writing to the
        # Feature Report characteristic for report ID 0xF1.
        #
        # From hid-magicmouse.c (magicmouse_enable_multitouch):
        #   BLE trackpad2: feature_mt_trackpad2_bt[] = { 0xF1, 0x02, 0x01 }
        #   - 0xF1 = report ID (identifies the Feature Report characteristic)
        #   - 0x02 = sub-command (multitouch mode selector)
        #   - 0x01 = enable
        #
        # This is analogous to USB SET_REPORT(Feature, ID=0x02, data={0x01})
        # but the BLE HID profile wraps it differently: the report ID 0xF1
        # routes to a specific GATT characteristic, and the payload contains
        # the original USB report ID (0x02) and enable byte (0x01).
        print("Activating multitouch mode...")
        mt_activated = False

        if mt_enable_char is not None:
            try:
                # Write the MT enable payload to the Feature Report characteristic.
                # bleak's write_gatt_char sends a GATT Write Request.
                await client.write_gatt_char(mt_enable_char, MT_ENABLE_PAYLOAD)
                print(f"  Sent MT enable via Feature Report 0xF1 (handle={mt_enable_char.handle})")
                print(f"  Payload: {' '.join(f'{b:02x}' for b in MT_ENABLE_PAYLOAD)}")
                mt_activated = True
            except Exception as e:
                print(f"  Failed to write Feature 0xF1: {e}")

        # Fallback: try writing to any Feature Report characteristic
        if not mt_activated:
            print("  Feature Report 0xF1 not found, trying alternatives...")
            for char, report_id, report_type in report_chars:
                if report_type != REPORT_TYPE_FEATURE:
                    continue
                if "write" not in char.properties and "write-without-response" not in char.properties:
                    continue

                # Try the BLE MT enable payload on every writable Feature characteristic
                try:
                    await client.write_gatt_char(char, MT_ENABLE_PAYLOAD)
                    print(f"  Wrote MT enable to Feature ID=0x{report_id:02X} "
                          f"(handle={char.handle})")
                    mt_activated = True
                    break
                except Exception as e:
                    print(f"  Feature ID=0x{report_id:02X} write failed: {e}")

            if not mt_activated:
                # Last resort: try the USB-style enable {0x02, 0x01}
                for char, report_id, report_type in report_chars:
                    if report_type != REPORT_TYPE_FEATURE:
                        continue
                    if "write" not in char.properties and "write-without-response" not in char.properties:
                        continue
                    try:
                        await client.write_gatt_char(char, MT_ENABLE_USB)
                        print(f"  Wrote USB-style MT enable to Feature ID=0x{report_id:02X} "
                              f"(handle={char.handle})")
                        mt_activated = True
                        break
                    except Exception as e:
                        pass

        if not mt_activated:
            print("  WARNING: Could not activate multitouch!")
            print("  Touch reports may not be generated.")
        print()

        # ---- Step 5: Capture until Ctrl+C or duration ----
        print("Capturing... Press Ctrl+C to stop.")
        if duration:
            print(f"  (Auto-stop after {duration}s)")
        print()

        try:
            if duration:
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=float(duration))
                except asyncio.TimeoutError:
                    print(f"\nDuration ({duration}s) reached.")
            else:
                await stop_event.wait()
        except asyncio.CancelledError:
            pass

        # ---- Step 6: Cleanup and summary ----
        print()
        for char, report_id, report_type in report_chars:
            if report_type == REPORT_TYPE_INPUT and "notify" in char.properties:
                try:
                    await client.stop_notify(char)
                except Exception:
                    pass

    if output_fh:
        output_fh.close()
        print(f"Capture saved to: {output_file}")

    # Print summary
    print()
    print("=" * 60)
    print("Capture Summary")
    print("=" * 60)
    print(f"  Total notifications: {stats['total_notifications']}")
    if stats["start_time"] and stats["last_time"]:
        elapsed = stats["last_time"] - stats["start_time"]
        rate = stats["total_notifications"] / elapsed if elapsed > 0 else 0
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Average rate: {rate:.1f} notifications/sec")
    print(f"  Touch reports (0x31): {stats['touch_reports']}")
    print(f"  Double reports (0xF7): {stats['double_reports']}")
    print(f"  Max simultaneous fingers: {stats['max_fingers']}")
    print()
    print("  Report ID counts:")
    for rid in sorted(stats["report_ids"].keys()):
        count = stats["report_ids"][rid]
        print(f"    0x{rid:02X}: {count}")


def _decode_and_print_mt2_report(data: bytes, stats: dict, indent: int = 8) -> None:
    """Decode and print a single MT2 BLE touch report."""
    pad = " " * indent
    size = len(data)

    if size < BLE_MT2_HEADER_SIZE:
        print(f"{pad}(too short for MT2 report: {size} bytes)")
        return

    # Validate: touch data must be a multiple of 9 bytes
    touch_data_len = size - BLE_MT2_HEADER_SIZE
    if touch_data_len % 9 != 0:
        print(f"{pad}(invalid touch data size: {touch_data_len} bytes, not multiple of 9)")
        return

    npoints = touch_data_len // 9
    if npoints > 15:
        print(f"{pad}(too many touch points: {npoints})")
        return

    stats["touch_reports"] += 1
    if npoints > stats["max_fingers"]:
        stats["max_fingers"] = npoints

    # Decode header
    clicks = data[1] & 0x3F  # Lower 6 bits are button state
    dev_timestamp = decode_ble_timestamp(data)

    print(f"{pad}-> MT2 BLE: clicks={clicks}, {npoints} touch(es), "
          f"dev_ts={dev_timestamp}")

    # Decode each touch point
    for i in range(min(npoints, 15)):
        offset = BLE_MT2_HEADER_SIZE + i * 9
        tdata = data[offset:offset + 9]
        touch = decode_mt2_touch(tdata)
        if touch:
            raw_hex = " ".join(f"{b:02x}" for b in tdata)
            print(f"{pad}  finger[{i}]: id={touch['id']:2d} "
                  f"x={touch['x']:5d} y={touch['y']:5d} "
                  f"state={touch['state_name']:7s} "
                  f"major={touch['major']:3d} minor={touch['minor']:3d} "
                  f"size={touch['size']:3d} pressure={touch['pressure']:3d} "
                  f"orient={touch['orientation']:+d}")
            print(f"{pad}    raw: [{raw_hex}]")


# ============================================================================
# Feature probe mode
# ============================================================================

async def cmd_features(args: argparse.Namespace) -> None:
    """Connect and read all Feature reports."""
    address = args.address
    if not address:
        print("Error: --address is required for features mode")
        sys.exit(1)

    print(f"Connecting to {address}...")
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect!")
            return

        print(f"Connected to {address}")
        await ensure_paired(client)
        print()

        for service in client.services:
            if service.uuid != UUID_HID_SERVICE:
                continue

            print(f"HID Service (handle {service.handle})")
            print()

            for char in service.characteristics:
                if char.uuid != UUID_REPORT:
                    continue

                # Read Report Reference
                report_id = 0
                report_type = 0
                for desc in char.descriptors:
                    if desc.uuid == UUID_REPORT_REFERENCE:
                        try:
                            ref = await client.read_gatt_descriptor(desc.handle)
                            if len(ref) >= 2:
                                report_id = ref[0]
                                report_type = ref[1]
                        except Exception:
                            pass

                if report_type != REPORT_TYPE_FEATURE:
                    continue

                print(f"Feature Report ID=0x{report_id:02X} (handle={char.handle})")

                if "read" not in char.properties:
                    print(f"  (not readable)")
                    print()
                    continue

                try:
                    data = await client.read_gatt_char(char)
                    hex_str = " ".join(f"{b:02x}" for b in data)
                    print(f"  [{len(data):3d}B] {hex_str}")

                    # Annotate known feature reports
                    if report_id == MT_ENABLE_REPORT_ID:
                        print(f"  -> MT control feature report")
                        if len(data) >= 3:
                            print(f"     Sub-report ID: 0x{data[1]:02X}, "
                                  f"Value: 0x{data[2]:02X}")
                    elif report_id == 0xD0:
                        print(f"  -> Device property (sub-report D0)")
                    elif report_id == 0xD1:
                        print(f"  -> Device surface dimensions (sub-report D1)")
                    elif report_id == 0xD3:
                        print(f"  -> Device resolution (sub-report D3)")
                    elif report_id == 0xDB:
                        print(f"  -> Compound device properties")
                    elif report_id == 0xA1:
                        print(f"  -> Device configuration (sub-report A1)")

                except Exception as e:
                    print(f"  (read failed: {e})")

                print()

        # Also read HID Information and Protocol Mode
        print("-" * 40)
        print("Other HID characteristics:")
        print()

        for service in client.services:
            if service.uuid != UUID_HID_SERVICE:
                continue

            for char in service.characteristics:
                if char.uuid == UUID_HID_INFORMATION:
                    try:
                        data = await client.read_gatt_char(char)
                        hex_str = " ".join(f"{b:02x}" for b in data)
                        print(f"HID Information (handle={char.handle}):")
                        print(f"  [{len(data)}B] {hex_str}")
                        if len(data) >= 4:
                            bcd_hid = data[0] | (data[1] << 8)
                            country = data[2]
                            flags = data[3]
                            print(f"  HID Version: {bcd_hid >> 8}.{(bcd_hid >> 4) & 0xF}.{bcd_hid & 0xF}")
                            print(f"  Country Code: {country}")
                            print(f"  Flags: 0x{flags:02X}"
                                  f" ({'RemoteWake' if flags & 0x01 else ''}"
                                  f"{'NormallyConnectable' if flags & 0x02 else ''})")
                        print()
                    except Exception as e:
                        print(f"HID Information: (read failed: {e})")
                        print()

                elif char.uuid == UUID_PROTOCOL_MODE:
                    try:
                        data = await client.read_gatt_char(char)
                        hex_str = " ".join(f"{b:02x}" for b in data)
                        mode = "Boot" if data[0] == 0 else "Report" if data[0] == 1 else f"Unknown(0x{data[0]:02X})"
                        print(f"Protocol Mode (handle={char.handle}):")
                        print(f"  [{len(data)}B] {hex_str} -> {mode} Protocol")
                        print()
                    except Exception as e:
                        print(f"Protocol Mode: (read failed: {e})")
                        print()

        # Battery level
        for service in client.services:
            if service.uuid != UUID_BATTERY_SERVICE:
                continue
            for char in service.characteristics:
                if char.uuid == UUID_BATTERY_LEVEL:
                    try:
                        data = await client.read_gatt_char(char)
                        if data:
                            print(f"Battery Level: {data[0]}%")
                        print()
                    except Exception as e:
                        print(f"Battery Level: (read failed: {e})")
                        print()


# ============================================================================
# Main / argument parsing
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="BLE capture and analysis tool for Apple Magic Trackpad 2 / 3",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s scan                                    Discover nearby BLE devices
  %(prog)s gatt -a AA:BB:CC:DD:EE:FF              Dump all GATT services
  %(prog)s report-map -a AA:BB:CC:DD:EE:FF        Read and parse HID Report Map
  %(prog)s capture -a AA:BB:CC:DD:EE:FF            Capture live touch data
  %(prog)s capture -a AA:BB:CC:DD:EE:FF -o cap.jsonl   Save to file
  %(prog)s features -a AA:BB:CC:DD:EE:FF           Read all Feature reports

Notes:
  - The device must be paired before most operations work (BLE HID requires
    encryption). Use your OS Bluetooth settings to pair first, then use this
    tool to connect.
  - On Linux, you may need to run with sudo or configure BlueZ permissions.
  - The --address parameter accepts the BLE MAC address format shown by scan.
""",
    )

    subparsers = parser.add_subparsers(dest="command", help="Operation mode")

    # scan
    p_scan = subparsers.add_parser("scan", help="Discover nearby BLE devices")
    p_scan.add_argument("-d", "--duration", type=float, default=10.0,
                        help="Scan duration in seconds (default: 10)")

    # gatt
    p_gatt = subparsers.add_parser("gatt", help="Dump all GATT services and characteristics")
    p_gatt.add_argument("-a", "--address", required=True,
                        help="BLE device address (e.g., AA:BB:CC:DD:EE:FF)")

    # report-map
    p_rmap = subparsers.add_parser("report-map",
                                   help="Read and parse HID Report Map descriptor")
    p_rmap.add_argument("-a", "--address", required=True,
                        help="BLE device address")

    # capture
    p_cap = subparsers.add_parser("capture",
                                  help="Capture live multitouch data")
    p_cap.add_argument("-a", "--address", required=True,
                       help="BLE device address")
    p_cap.add_argument("-d", "--duration", type=float, default=None,
                       help="Capture duration in seconds (default: unlimited, Ctrl+C to stop)")
    p_cap.add_argument("-o", "--output", type=str, default=None,
                       help="Output file for capture data (JSON lines format)")

    # features
    p_feat = subparsers.add_parser("features",
                                   help="Read all HID Feature reports")
    p_feat.add_argument("-a", "--address", required=True,
                        help="BLE device address")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    # Dispatch to the appropriate async handler
    handlers = {
        "scan": cmd_scan,
        "gatt": cmd_gatt,
        "report-map": cmd_report_map,
        "capture": cmd_capture,
        "features": cmd_features,
    }

    handler = handlers[args.command]
    asyncio.run(handler(args))


if __name__ == "__main__":
    main()
