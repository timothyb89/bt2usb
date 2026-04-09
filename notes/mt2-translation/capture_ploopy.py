#!/usr/bin/env python3
"""
Capture HID Report Descriptors, Feature Reports, and Input Reports from a
Ploopy trackpad (or any USB HID device).

The Ploopy trackpad implements Windows Precision Touchpad (PTP) natively,
without requiring drivers. This script captures its HID Report Descriptor
as a reference for building our own PTP descriptor in bt2usb.

PTP (Windows Precision Touchpad) HID specification overview:
  - Usage Page 0x0D (Digitizer), Usage 0x05 (Touch Pad) for the main collection
  - Finger collections: Usage 0x22 (Finger) with Contact ID, Tip Switch, X, Y,
    confidence, width/height
  - Contact Count (Usage 0x54) in the top-level input report
  - Scan Time (Usage 0x56) for timing information
  - Feature reports: Input Mode (0x52), Max Contact Count (0x55),
    Device Certification Status (0xC5), Latency Mode (0x60)
  - Button (Usage Page 0x09) for physical button state
  - PTP devices must support at least 5 simultaneous contacts

Modes:
  --descriptor (default): Read and parse HID Report Descriptors
  --capture:              Read live Input reports
  --features:             Read Feature reports

Usage:
    pip3 install pyusb hidapi
    sudo python3 capture_ploopy.py                       # Descriptor mode, pick device
    sudo python3 capture_ploopy.py --vid 0x2E8A --pid 0xFEDD  # Specific Ploopy
    sudo python3 capture_ploopy.py --capture
    sudo python3 capture_ploopy.py --features
    sudo python3 capture_ploopy.py --descriptor --output ploopy_descriptor.txt

Note: On Linux, root/sudo is typically required to detach kernel drivers and
read raw HID descriptors via pyusb. For --capture and --features modes,
hidapi is used which may work with udev rules (no root) or may need root.
"""

import argparse
import sys
import time
import struct
from typing import Optional

# ---------------------------------------------------------------------------
# HID Usage Page and Usage name tables
# ---------------------------------------------------------------------------

USAGE_PAGE_NAMES = {
    0x01: "Generic Desktop",
    0x02: "Simulation Controls",
    0x03: "VR Controls",
    0x04: "Sport Controls",
    0x05: "Game Controls",
    0x06: "Generic Device Controls",
    0x07: "Keyboard/Keypad",
    0x08: "LED",
    0x09: "Button",
    0x0A: "Ordinal",
    0x0B: "Telephony Device",
    0x0C: "Consumer",
    0x0D: "Digitizer",
    0x0E: "Haptics",
    0x0F: "Physical Input Device",
    0x10: "Unicode",
    0x12: "Eye and Head Trackers",
    0x14: "Auxiliary Display",
    0x20: "Sensor",
    0x40: "Medical Instrument",
    0x41: "Braille Display",
    0x59: "Lighting and Illumination",
    0x80: "Monitor",
    0x81: "Monitor Enumerated",
    0x82: "VESA Virtual Controls",
    0x84: "Power Device",
    0x85: "Battery System",
    0xF1D0: "FIDO Alliance",
}

USAGE_NAMES_GENERIC_DESKTOP = {
    0x00: "Undefined",
    0x01: "Pointer",
    0x02: "Mouse",
    0x04: "Joystick",
    0x05: "Game Pad",
    0x06: "Keyboard",
    0x07: "Keypad",
    0x08: "Multi-axis Controller",
    0x09: "Tablet PC System Controls",
    0x0A: "Water Cooling Device",
    0x0B: "Computer Chassis Device",
    0x0C: "Wireless Radio Controls",
    0x0D: "Portable Device Control",
    0x0E: "System Multi-Axis Controller",
    0x0F: "Spatial Controller",
    0x10: "Assistive Control",
    0x11: "Device Dock",
    0x12: "Dockable Device",
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
    0x3A: "Counted Buffer",
    0x3B: "Byte Count",
    0x3C: "Motion Wakeup",
    0x3D: "Start",
    0x3E: "Select",
    0x40: "Vx",
    0x41: "Vy",
    0x42: "Vz",
    0x43: "Vbrx",
    0x44: "Vbry",
    0x45: "Vbrz",
    0x46: "Vno",
    0x47: "Feature Notification",
    0x48: "Resolution Multiplier",
    0x49: "Qx",
    0x4A: "Qy",
    0x4B: "Qz",
    0x4C: "Qw",
    0x80: "System Control",
    0x81: "System Power Down",
    0x82: "System Sleep",
    0x83: "System Wake Up",
}

USAGE_NAMES_DIGITIZER = {
    0x00: "Undefined",
    0x01: "Digitizer",
    0x02: "Pen",
    0x03: "Light Pen",
    0x04: "Touch Screen",
    0x05: "Touch Pad",                   # <-- PTP top-level usage
    0x06: "White Board",
    0x07: "Coordinate Measuring Machine",
    0x08: "3D Digitizer",
    0x09: "Stereo Plotter",
    0x0A: "Articulated Arm",
    0x0B: "Armature",
    0x0C: "Multiple Point Digitizer",
    0x0D: "Free Space Wand",
    0x0E: "Device Configuration",
    0x0F: "Capacitive Heat Map Digitizer",
    0x20: "Stylus",
    0x22: "Finger",                       # <-- PTP finger collection
    0x23: "Device Settings",              # <-- PTP device configuration
    0x24: "Character Gesture",
    0x30: "Tip Pressure",
    0x31: "Barrel Pressure",
    0x32: "In Range",
    0x33: "Touch",
    0x34: "Untouch",
    0x35: "Tap",
    0x36: "Quality",
    0x37: "Data Valid",
    0x38: "Transducer Index",
    0x39: "Tablet Function Keys",
    0x3A: "Program Change Keys",
    0x3B: "Battery Strength",
    0x3C: "Invert",
    0x3D: "X Tilt",
    0x3E: "Y Tilt",
    0x3F: "Azimuth",
    0x40: "Altitude",
    0x41: "Twist",
    0x42: "Tip Switch",                   # <-- PTP: finger down
    0x43: "Secondary Tip Switch",
    0x44: "Barrel Switch",
    0x45: "Eraser",
    0x46: "Tablet Pick",
    0x47: "Touch Valid (Confidence)",      # <-- PTP: confidence
    0x48: "Width",                         # <-- PTP: contact width
    0x49: "Height",                        # <-- PTP: contact height
    0x51: "Contact Identifier",            # <-- PTP: contact ID
    0x52: "Device Mode (Input Mode)",      # <-- PTP feature report
    0x53: "Device Identifier",
    0x54: "Contact Count",                 # <-- PTP: number of active contacts
    0x55: "Contact Count Maximum",         # <-- PTP feature report
    0x56: "Scan Time",                     # <-- PTP: report timestamp
    0x57: "Surface Switch",               # <-- PTP: enable/disable surface
    0x58: "Button Switch",                # <-- PTP: physical button enable
    0x59: "Pad Type",
    0x5A: "Secondary Barrel Switch",
    0x5B: "Transducer Serial Number",
    0x5C: "Preferred Color",
    0x5D: "Preferred Color is Locked",
    0x5E: "Preferred Line Width",
    0x5F: "Preferred Line Width is Locked",
    0x60: "Latency Mode",                 # <-- PTP feature report
    0x61: "Input Mode 2",
    0x62: "No Preferred Color",
    0x6E: "Transducer Connected",
    0xC5: "Device Certification Status",  # <-- PTP feature report (blob)
}

USAGE_NAMES_BUTTON = {}  # Buttons are just numbered: Button 1, Button 2, ...

# Per-page lookup
USAGE_NAMES_BY_PAGE = {
    0x01: USAGE_NAMES_GENERIC_DESKTOP,
    0x09: USAGE_NAMES_BUTTON,
    0x0D: USAGE_NAMES_DIGITIZER,
}

# PTP-specific usages we want to highlight in output
PTP_HIGHLIGHT_USAGES = {
    (0x0D, 0x05),  # Touch Pad
    (0x0D, 0x22),  # Finger
    (0x0D, 0x23),  # Device Settings
    (0x0D, 0x42),  # Tip Switch
    (0x0D, 0x47),  # Confidence
    (0x0D, 0x48),  # Width
    (0x0D, 0x49),  # Height
    (0x0D, 0x51),  # Contact ID
    (0x0D, 0x52),  # Input Mode
    (0x0D, 0x54),  # Contact Count
    (0x0D, 0x55),  # Contact Count Maximum
    (0x0D, 0x56),  # Scan Time
    (0x0D, 0x57),  # Surface Switch
    (0x0D, 0x58),  # Button Switch
    (0x0D, 0x60),  # Latency Mode
    (0x0D, 0xC5),  # Device Certification Status
}


def usage_name(page: int, usage: int) -> str:
    """Return human-readable name for a usage, or hex if unknown."""
    if page == 0x09:
        return f"Button {usage}"
    table = USAGE_NAMES_BY_PAGE.get(page, {})
    name = table.get(usage)
    if name:
        return name
    return f"0x{usage:04X}"


def usage_page_name(page: int) -> str:
    """Return human-readable name for a usage page."""
    if page >= 0xFF00:
        return f"Vendor Defined (0x{page:04X})"
    return USAGE_PAGE_NAMES.get(page, f"0x{page:04X}")


# ---------------------------------------------------------------------------
# Input/Output/Feature field flag decoding
# ---------------------------------------------------------------------------

def decode_io_flags(val: int) -> str:
    """Decode Input/Output/Feature item flags to a human-readable string."""
    parts = []
    parts.append("Const" if val & 0x01 else "Data")
    parts.append("Var" if val & 0x02 else "Array")
    parts.append("Rel" if val & 0x04 else "Abs")
    if val & 0x08:
        parts.append("Wrap")
    if val & 0x10:
        parts.append("NonLinear")
    if val & 0x20:
        parts.append("NoPref")
    if val & 0x40:
        parts.append("Null")
    if val & 0x80:
        parts.append("Volatile")
    if val & 0x100:
        parts.append("Buffered")
    return ", ".join(parts)


COLLECTION_TYPES = {
    0x00: "Physical",
    0x01: "Application",
    0x02: "Logical",
    0x03: "Report",
    0x04: "Named Array",
    0x05: "Usage Switch",
    0x06: "Usage Modifier",
}


# ---------------------------------------------------------------------------
# HID Report Descriptor parser
# ---------------------------------------------------------------------------

class HidField:
    """Represents a single parsed field from a HID report descriptor."""
    def __init__(self, field_type, report_id, usage_page, usages, logical_min,
                 logical_max, physical_min, physical_max, report_size,
                 report_count, flags, unit, unit_exponent):
        self.field_type = field_type  # "Input", "Output", "Feature"
        self.report_id = report_id
        self.usage_page = usage_page
        self.usages = usages  # list of (page, usage)
        self.logical_min = logical_min
        self.logical_max = logical_max
        self.physical_min = physical_min
        self.physical_max = physical_max
        self.report_size = report_size
        self.report_count = report_count
        self.flags = flags
        self.unit = unit
        self.unit_exponent = unit_exponent


def signed_value(val: int, size: int) -> int:
    """Convert an unsigned HID item value to signed based on byte size."""
    if size == 0:
        return 0
    bits = size * 8
    if val >= (1 << (bits - 1)):
        val -= (1 << bits)
    return val


def parse_hid_descriptor(data: bytes, output_lines: Optional[list] = None):
    """
    Parse a HID Report Descriptor and print a detailed human-readable
    representation. If output_lines is provided, lines are also appended there.

    Returns a list of HidField objects for later use in report decoding.
    """
    def emit(line: str):
        print(line)
        if output_lines is not None:
            output_lines.append(line)

    # Global state
    g_usage_page = 0
    g_logical_min = 0
    g_logical_max = 0
    g_physical_min = 0
    g_physical_max = 0
    g_report_size = 0
    g_report_count = 0
    g_report_id = 0
    g_unit = 0
    g_unit_exponent = 0

    # Local state (reset after each Main item)
    l_usages = []
    l_usage_min = None
    l_usage_max = None
    l_designator_index = None
    l_designator_min = None
    l_designator_max = None
    l_string_index = None
    l_string_min = None
    l_string_max = None

    # Global state stack for Push/Pop
    g_stack = []

    indent = 0
    fields = []
    i = 0

    while i < len(data):
        b = data[i]

        # Long item
        if b == 0xFE:
            if i + 2 >= len(data):
                emit(f"  {'  ' * indent}[{i:4d}] TRUNCATED LONG ITEM")
                break
            data_size = data[i + 1]
            long_tag = data[i + 2]
            raw = data[i:i + 3 + data_size]
            hex_str = " ".join(f"{x:02x}" for x in raw)
            emit(f"  {'  ' * indent}[{i:4d}] {hex_str:<28s}  Long Item (tag=0x{long_tag:02X}, {data_size} data bytes)")
            i += 3 + data_size
            continue

        # Short item
        bSize = b & 0x03
        if bSize == 3:
            bSize = 4
        bType = (b >> 2) & 0x03
        bTag = (b >> 4) & 0x0F

        # Read value (unsigned)
        val = 0
        raw_bytes = data[i:i + 1 + bSize]
        for j in range(bSize):
            if i + 1 + j < len(data):
                val |= data[i + 1 + j] << (8 * j)

        # Signed value for min/max items
        sval = signed_value(val, bSize)

        hex_str = " ".join(f"{x:02x}" for x in raw_bytes)
        pad = "  " * indent

        # ---- MAIN items (bType == 0) ----
        if bType == 0:
            if bTag == 0x08:  # Input
                field_usages = list(l_usages)
                if l_usage_min is not None and l_usage_max is not None:
                    for u in range(l_usage_min, l_usage_max + 1):
                        field_usages.append((g_usage_page, u))
                if not field_usages and g_report_count > 0:
                    # Padding or constant field
                    field_usages = [(g_usage_page, 0)]

                f = HidField("Input", g_report_id, g_usage_page, field_usages,
                             g_logical_min, g_logical_max, g_physical_min,
                             g_physical_max, g_report_size, g_report_count,
                             val, g_unit, g_unit_exponent)
                fields.append(f)

                flag_desc = decode_io_flags(val)
                bits = g_report_size * g_report_count
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Input (0x{val:02X}): {flag_desc}  [{g_report_count}x {g_report_size}bit = {bits}bits]")

            elif bTag == 0x09:  # Output
                flag_desc = decode_io_flags(val)
                bits = g_report_size * g_report_count
                f = HidField("Output", g_report_id, g_usage_page, list(l_usages),
                             g_logical_min, g_logical_max, g_physical_min,
                             g_physical_max, g_report_size, g_report_count,
                             val, g_unit, g_unit_exponent)
                fields.append(f)
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Output (0x{val:02X}): {flag_desc}  [{g_report_count}x {g_report_size}bit = {bits}bits]")

            elif bTag == 0x0B:  # Feature
                field_usages = list(l_usages)
                if l_usage_min is not None and l_usage_max is not None:
                    for u in range(l_usage_min, l_usage_max + 1):
                        field_usages.append((g_usage_page, u))
                if not field_usages and g_report_count > 0:
                    field_usages = [(g_usage_page, 0)]

                f = HidField("Feature", g_report_id, g_usage_page, field_usages,
                             g_logical_min, g_logical_max, g_physical_min,
                             g_physical_max, g_report_size, g_report_count,
                             val, g_unit, g_unit_exponent)
                fields.append(f)

                flag_desc = decode_io_flags(val)
                bits = g_report_size * g_report_count
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Feature (0x{val:02X}): {flag_desc}  [{g_report_count}x {g_report_size}bit = {bits}bits]")

            elif bTag == 0x0A:  # Collection
                ctype = COLLECTION_TYPES.get(val, f"Vendor (0x{val:02X})")

                # Show which usage this collection is for (from local state)
                usage_desc = ""
                if l_usages:
                    page, usg = l_usages[-1]
                    uname = usage_name(page, usg)
                    usage_desc = f"  -- {usage_page_name(page)}: {uname}"
                    # PTP highlight
                    if (page, usg) in PTP_HIGHLIGHT_USAGES:
                        usage_desc += "  *** PTP ***"

                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Collection ({ctype}){usage_desc}")
                indent += 1

            elif bTag == 0x0C:  # End Collection
                indent = max(0, indent - 1)
                pad = "  " * indent
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  End Collection")

            else:
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Main(tag=0x{bTag:X}, val=0x{val:X})")

            # Reset local state after Main item
            l_usages = []
            l_usage_min = None
            l_usage_max = None
            l_designator_index = None
            l_designator_min = None
            l_designator_max = None
            l_string_index = None
            l_string_min = None
            l_string_max = None

        # ---- GLOBAL items (bType == 1) ----
        elif bType == 1:
            if bTag == 0x00:  # Usage Page
                g_usage_page = val
                pname = usage_page_name(val)
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Usage Page ({pname})")

            elif bTag == 0x01:  # Logical Minimum
                g_logical_min = sval
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Logical Minimum ({sval})")

            elif bTag == 0x02:  # Logical Maximum
                g_logical_max = sval
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Logical Maximum ({sval})")

            elif bTag == 0x03:  # Physical Minimum
                g_physical_min = sval
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Physical Minimum ({sval})")

            elif bTag == 0x04:  # Physical Maximum
                g_physical_max = sval
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Physical Maximum ({sval})")

            elif bTag == 0x05:  # Unit Exponent
                g_unit_exponent = sval
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Unit Exponent ({sval})")

            elif bTag == 0x06:  # Unit
                g_unit = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Unit (0x{val:X})")

            elif bTag == 0x07:  # Report Size
                g_report_size = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Report Size ({val})")

            elif bTag == 0x08:  # Report ID
                g_report_id = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Report ID (0x{val:02X})")

            elif bTag == 0x09:  # Report Count
                g_report_count = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Report Count ({val})")

            elif bTag == 0x0A:  # Push
                g_stack.append((g_usage_page, g_logical_min, g_logical_max,
                                g_physical_min, g_physical_max, g_report_size,
                                g_report_count, g_report_id, g_unit, g_unit_exponent))
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Push")

            elif bTag == 0x0B:  # Pop
                if g_stack:
                    (g_usage_page, g_logical_min, g_logical_max,
                     g_physical_min, g_physical_max, g_report_size,
                     g_report_count, g_report_id, g_unit, g_unit_exponent) = g_stack.pop()
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Pop")

            else:
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Global(tag=0x{bTag:X}, val={val})")

        # ---- LOCAL items (bType == 2) ----
        elif bType == 2:
            if bTag == 0x00:  # Usage
                # Usage can be 16-bit (extended) or 8-bit
                if bSize <= 2:
                    upage = g_usage_page
                    uval = val
                else:
                    # 32-bit extended usage: high 16 = page, low 16 = usage
                    upage = (val >> 16) & 0xFFFF
                    uval = val & 0xFFFF

                uname = usage_name(upage, uval)
                ptp_mark = ""
                if (upage, uval) in PTP_HIGHLIGHT_USAGES:
                    ptp_mark = "  *** PTP ***"
                l_usages.append((upage, uval))
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Usage ({uname} [0x{uval:04X}]){ptp_mark}")

            elif bTag == 0x01:  # Usage Minimum
                if bSize <= 2:
                    l_usage_min = val
                else:
                    l_usage_min = val & 0xFFFF
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Usage Minimum (0x{val:04X})")

            elif bTag == 0x02:  # Usage Maximum
                if bSize <= 2:
                    l_usage_max = val
                else:
                    l_usage_max = val & 0xFFFF
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Usage Maximum (0x{val:04X})")

            elif bTag == 0x03:  # Designator Index
                l_designator_index = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Designator Index ({val})")

            elif bTag == 0x04:  # Designator Minimum
                l_designator_min = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Designator Minimum ({val})")

            elif bTag == 0x05:  # Designator Maximum
                l_designator_max = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Designator Maximum ({val})")

            elif bTag == 0x07:  # String Index
                l_string_index = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  String Index ({val})")

            elif bTag == 0x08:  # String Minimum
                l_string_min = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  String Minimum ({val})")

            elif bTag == 0x09:  # String Maximum
                l_string_max = val
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  String Maximum ({val})")

            elif bTag == 0x0A:  # Delimiter
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Delimiter ({'Open' if val == 1 else 'Close'})")

            else:
                emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Local(tag=0x{bTag:X}, val=0x{val:X})")

        # ---- RESERVED (bType == 3) ----
        else:
            emit(f"  {pad}[{i:4d}] {hex_str:<28s}  Reserved(type=3, tag=0x{bTag:X})")

        i += 1 + bSize

    return fields


def build_report_layout(fields: list) -> dict:
    """
    Build a layout map from parsed fields for decoding live reports.

    Returns: {report_id: [(field_name, bit_offset, bit_size, logical_min, logical_max, field_type), ...]}
    where field_type is "Input", "Output", or "Feature".
    """
    layouts = {}

    for f in fields:
        if f.report_id not in layouts:
            layouts[f.report_id] = {"Input": [], "Output": [], "Feature": []}

        total_bits = f.report_size * f.report_count
        usages = f.usages

        if f.report_count == 1 or (f.flags & 0x02):
            # Variable: one field per report count, each with its own usage
            for idx in range(f.report_count):
                if idx < len(usages):
                    page, usg = usages[idx]
                    name = usage_name(page, usg)
                else:
                    if usages:
                        page, usg = usages[-1]
                        name = usage_name(page, usg)
                    else:
                        name = f"Padding"

                layouts[f.report_id][f.field_type].append({
                    "name": name,
                    "page": usages[min(idx, len(usages) - 1)][0] if usages else 0,
                    "usage": usages[min(idx, len(usages) - 1)][1] if usages else 0,
                    "size": f.report_size,
                    "count": 1,
                    "logical_min": f.logical_min,
                    "logical_max": f.logical_max,
                    "flags": f.flags,
                })
        else:
            # Array: the whole block is one logical field
            name = "Array"
            if usages:
                page, usg = usages[0]
                name = f"{usage_name(page, usg)} Array"

            layouts[f.report_id][f.field_type].append({
                "name": name,
                "page": usages[0][0] if usages else 0,
                "usage": usages[0][1] if usages else 0,
                "size": f.report_size,
                "count": f.report_count,
                "logical_min": f.logical_min,
                "logical_max": f.logical_max,
                "flags": f.flags,
            })

    return layouts


def extract_field_value(data: bytes, bit_offset: int, bit_size: int,
                        signed: bool = False) -> int:
    """Extract a field value from report data at a given bit offset and size."""
    val = 0
    for bit in range(bit_size):
        byte_idx = (bit_offset + bit) // 8
        bit_idx = (bit_offset + bit) % 8
        if byte_idx < len(data):
            if data[byte_idx] & (1 << bit_idx):
                val |= (1 << bit)
    if signed and val >= (1 << (bit_size - 1)):
        val -= (1 << bit_size)
    return val


def decode_report(data: bytes, layouts: dict):
    """
    Decode a report using the parsed layout.
    data[0] is the report ID.
    Returns a list of (field_name, value) tuples.
    """
    if not data:
        return []

    report_id = data[0]
    if report_id not in layouts:
        return []

    input_fields = layouts[report_id].get("Input", [])
    if not input_fields:
        return []

    results = []
    bit_offset = 0  # Bit offset within the report body (after report ID byte)
    report_body = data[1:]  # Skip report ID

    for field in input_fields:
        total_bits = field["size"] * field["count"]
        is_const = field["flags"] & 0x01

        if is_const:
            bit_offset += total_bits
            continue

        is_signed = field["logical_min"] < 0

        if field["count"] == 1:
            val = extract_field_value(report_body, bit_offset, field["size"], is_signed)
            results.append((field["name"], val, field))
        else:
            for c in range(field["count"]):
                val = extract_field_value(report_body, bit_offset + c * field["size"],
                                          field["size"], is_signed)
                results.append((f"{field['name']}[{c}]", val, field))

        bit_offset += total_bits

    return results


def print_report_summary(fields: list, output_lines: Optional[list] = None):
    """Print a summary of all report IDs and their sizes."""
    def emit(line: str):
        print(line)
        if output_lines is not None:
            output_lines.append(line)

    # Group by report ID and type
    reports = {}
    for f in fields:
        key = (f.report_id, f.field_type)
        if key not in reports:
            reports[key] = 0
        reports[key] += f.report_size * f.report_count

    emit("\n--- Report Summary ---")
    for (rid, ftype), bits in sorted(reports.items()):
        byte_size = (bits + 7) // 8
        emit(f"  Report ID 0x{rid:02X} {ftype:8s}: {bits} bits ({byte_size} bytes) + 1 byte report ID = {byte_size + 1} bytes total")


# ---------------------------------------------------------------------------
# Descriptor mode
# ---------------------------------------------------------------------------

def do_descriptor(vid: Optional[int], pid: Optional[int], output_path: Optional[str]):
    """Read and display HID Report Descriptors using pyusb."""
    try:
        import usb.core
        import usb.util
    except ImportError:
        print("ERROR: pyusb is required for descriptor mode.")
        print("  Install: pip3 install pyusb")
        print("  On Linux you also need libusb: sudo apt install libusb-1.0-0-dev")
        sys.exit(1)

    DT_HID_REPORT = 0x22
    GET_DESCRIPTOR = 0x06

    dev = None
    if vid is not None and pid is not None:
        dev = usb.core.find(idVendor=vid, idProduct=pid)
        if dev is None:
            print(f"No device found with VID=0x{vid:04X} PID=0x{pid:04X}")
            sys.exit(1)
    else:
        # List all USB devices and let user choose
        all_devs = list(usb.core.find(find_all=True))
        hid_devs = []
        for d in all_devs:
            try:
                cfg = d.get_active_configuration()
                for intf in cfg:
                    if intf.bInterfaceClass == 0x03:  # HID
                        hid_devs.append(d)
                        break
            except Exception:
                pass

        if not hid_devs:
            print("No USB HID devices found.")
            sys.exit(1)

        print("Available USB HID devices:")
        for idx, d in enumerate(hid_devs):
            try:
                mfg = d.manufacturer or "?"
            except Exception:
                mfg = "?"
            try:
                prod = d.product or "?"
            except Exception:
                prod = "?"
            print(f"  [{idx}] VID=0x{d.idVendor:04X} PID=0x{d.idProduct:04X}  {mfg} - {prod}")

        try:
            choice = int(input("\nSelect device number: "))
            dev = hid_devs[choice]
        except (ValueError, IndexError, KeyboardInterrupt):
            print("Invalid selection.")
            sys.exit(1)

    try:
        mfg = dev.manufacturer or "Unknown"
    except Exception:
        mfg = "Unknown"
    try:
        prod = dev.product or "Unknown"
    except Exception:
        prod = "Unknown"

    output_lines = [] if output_path else None

    def emit(line: str):
        print(line)
        if output_lines is not None:
            output_lines.append(line)

    emit(f"Device: {mfg} {prod}")
    emit(f"VID=0x{dev.idVendor:04X} PID=0x{dev.idProduct:04X}")

    cfg = dev.get_active_configuration()
    emit(f"Configuration {cfg.bConfigurationValue}: {cfg.bNumInterfaces} interface(s)\n")

    all_fields = []

    for intf in cfg:
        iface_num = intf.bInterfaceNumber
        emit("=" * 72)
        class_desc = ""
        if intf.bInterfaceClass == 0x03:
            subclass_map = {0: "None", 1: "Boot"}
            proto_map = {0: "None", 1: "Keyboard", 2: "Mouse"}
            sc = subclass_map.get(intf.bInterfaceSubClass, f"0x{intf.bInterfaceSubClass:02X}")
            pr = proto_map.get(intf.bInterfaceProtocol, f"0x{intf.bInterfaceProtocol:02X}")
            class_desc = f" (HID, subclass={sc}, protocol={pr})"

        emit(f"Interface {iface_num}: class=0x{intf.bInterfaceClass:02X} "
             f"subclass=0x{intf.bInterfaceSubClass:02X} "
             f"protocol=0x{intf.bInterfaceProtocol:02X}{class_desc}")

        # Endpoints
        for ep in intf:
            direction = "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT"
            emit(f"  Endpoint 0x{ep.bEndpointAddress:02X} ({direction}): "
                 f"maxPacket={ep.wMaxPacketSize}, interval={ep.bInterval}ms")

        if intf.bInterfaceClass != 0x03:
            emit("  (not HID, skipping descriptor read)")
            emit("")
            continue

        # Try to detach kernel driver
        detached = False
        try:
            if dev.is_kernel_driver_active(iface_num):
                dev.detach_kernel_driver(iface_num)
                detached = True
                emit("  (detached kernel driver)")
        except Exception:
            pass

        # Read HID Report Descriptor
        try:
            desc = dev.ctrl_transfer(
                0x81,           # bmRequestType: Device-to-host, Standard, Interface
                GET_DESCRIPTOR,
                (DT_HID_REPORT << 8),  # wValue: HID Report Descriptor
                iface_num,
                4096,
                timeout=2000,
            )
            desc = bytes(desc)
            emit(f"\n  HID Report Descriptor: {len(desc)} bytes")

            # Raw hex dump with 16 bytes per line
            emit("  Raw hex:")
            for row in range(0, len(desc), 16):
                chunk = desc[row:row + 16]
                hex_part = " ".join(f"{b:02x}" for b in chunk)
                ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
                emit(f"    {row:04X}: {hex_part:<48s}  {ascii_part}")

            # C array for easy embedding
            emit(f"\n  C array ({len(desc)} bytes):")
            for row in range(0, len(desc), 16):
                chunk = desc[row:row + 16]
                c_hex = ", ".join(f"0x{b:02X}" for b in chunk)
                trail = "," if row + 16 < len(desc) else ""
                emit(f"    {c_hex}{trail}")

            emit(f"\n  Parsed descriptor:")
            iface_fields = parse_hid_descriptor(desc, output_lines)
            all_fields.extend(iface_fields)

            # Print report summary for this interface
            if iface_fields:
                print_report_summary(iface_fields, output_lines)

        except Exception as e:
            emit(f"  Failed to read descriptor: {e}")

        emit("")

        # Reattach kernel driver
        if detached:
            try:
                dev.attach_kernel_driver(iface_num)
            except Exception:
                pass

    # Overall PTP analysis
    emit("\n" + "=" * 72)
    emit("PTP ANALYSIS")
    emit("=" * 72)

    ptp_found = False
    for f in all_fields:
        for page, usg in f.usages:
            if (page, usg) in PTP_HIGHLIGHT_USAGES:
                ptp_found = True
                uname = usage_name(page, usg)
                pname = usage_page_name(page)
                emit(f"  [{f.field_type:8s} RID=0x{f.report_id:02X}] {pname}: {uname} "
                     f"(size={f.report_size}, count={f.report_count}, "
                     f"range=[{f.logical_min}, {f.logical_max}])")

    if not ptp_found:
        emit("  No PTP-specific usages found in this device.")
        emit("  (Expected: Digitizer page 0x0D usages for Touch Pad, Finger, Contact Count, etc.)")

    emit("")

    # Save to file if requested
    if output_path and output_lines:
        with open(output_path, "w") as fp:
            for line in output_lines:
                fp.write(line + "\n")
        print(f"Output saved to: {output_path}")


# ---------------------------------------------------------------------------
# Capture mode
# ---------------------------------------------------------------------------

def do_capture(vid: Optional[int], pid: Optional[int]):
    """Capture live Input reports using hidapi."""
    try:
        import hid
    except ImportError:
        print("ERROR: hidapi is required for capture mode.")
        print("  Install: pip3 install hidapi")
        sys.exit(1)

    if vid is not None and pid is not None:
        devices = hid.enumerate(vid, pid)
    else:
        devices = hid.enumerate()
        # Filter to HID devices only (hid.enumerate returns all)
        if not devices:
            print("No HID devices found.")
            sys.exit(1)

        print("Available HID devices:")
        for idx, d in enumerate(devices):
            prod = d.get("product_string", "?") or "?"
            mfg = d.get("manufacturer_string", "?") or "?"
            print(f"  [{idx}] VID=0x{d['vendor_id']:04X} PID=0x{d['product_id']:04X}  "
                  f"{mfg} - {prod}  "
                  f"(iface={d.get('interface_number', '?')}, "
                  f"usage=0x{d.get('usage_page', 0):04X}/0x{d.get('usage', 0):04X})")

        try:
            choice = int(input("\nSelect device number (or -1 for all): "))
            if choice == -1:
                pass  # Use all
            else:
                devices = [devices[choice]]
        except (ValueError, IndexError, KeyboardInterrupt):
            print("Invalid selection.")
            sys.exit(1)

    if not devices:
        print(f"No HID device found" +
              (f" with VID=0x{vid:04X} PID=0x{pid:04X}" if vid else "") + ".")
        sys.exit(1)

    print(f"\nFound {len(devices)} interface(s):")
    for idx, d in enumerate(devices):
        prod = d.get("product_string", "?") or "?"
        print(f"  [{idx}] interface={d.get('interface_number', '?')}, "
              f"usage=0x{d.get('usage_page', 0):04X}/0x{d.get('usage', 0):04X}, "
              f"product={prod}")

    for d_info in devices:
        iface = d_info.get("interface_number", "?")
        usage = f"0x{d_info.get('usage_page', 0):04X}/0x{d_info.get('usage', 0):04X}"

        print(f"\n{'='*72}")
        print(f"Reading from interface {iface} (usage {usage})")
        print(f"Touch the trackpad surface. Press Ctrl-C to stop (or move to next interface).")
        print()

        try:
            dev = hid.device()
            dev.open_path(d_info["path"])
            dev.set_nonblocking(False)

            count = 0
            report_ids_seen = {}
            t0 = time.monotonic()

            try:
                while True:
                    data = dev.read(4096, timeout_ms=5000)
                    if not data:
                        if count == 0:
                            print("  No data received (5s timeout). Skipping interface.")
                        break

                    now = time.monotonic() - t0
                    rid = data[0] if data else 0
                    size = len(data)

                    if rid not in report_ids_seen:
                        report_ids_seen[rid] = 0
                    report_ids_seen[rid] += 1

                    # Raw hex (truncate very long reports)
                    max_show = 64
                    hex_str = " ".join(f"{b:02x}" for b in data[:max_show])
                    if size > max_show:
                        hex_str += f" ... ({size} bytes total)"

                    print(f"  [{count:5d}] t={now:8.3f}s RID=0x{rid:02X} [{size:4d}B] {hex_str}")

                    # PTP-specific decoding heuristics
                    # Contact Count is typically in the report, and Finger data
                    # follows a pattern. We try common PTP layouts.
                    _try_decode_ptp_report(data, rid)

                    count += 1

            except KeyboardInterrupt:
                pass

            print(f"\n  Report IDs seen:")
            for rid_val, cnt in sorted(report_ids_seen.items()):
                print(f"    0x{rid_val:02X}: {cnt} reports")
            print(f"  Total reports: {count}")
            dev.close()

        except Exception as e:
            print(f"  Could not open: {e}")

    print("\nDone.")


def _try_decode_ptp_report(data: bytes, rid: int):
    """
    Attempt to decode a PTP touchpad Input report.

    PTP reports typically contain:
    - Report ID (1 byte)
    - Per-finger data (repeated N times):
        - Tip Switch + Confidence (packed bits)
        - Contact ID
        - X (16-bit LE)
        - Y (16-bit LE)
        - Optionally: Width, Height, Pressure
    - Contact Count (1 byte)
    - Scan Time (16-bit LE)
    - Button (1 byte or packed bit)

    Since we do not know the exact layout without parsing the descriptor,
    we use a heuristic approach. The descriptor parse (--descriptor mode)
    gives the definitive layout.
    """
    if len(data) < 5:
        return

    body = data[1:]  # Skip report ID

    # Heuristic: Look for patterns common in PTP reports.
    # Many PTP devices use a structure like:
    #   [tip+conf] [contact_id] [x_lo] [x_hi] [y_lo] [y_hi] ... repeated
    #   [contact_count] [scan_time_lo] [scan_time_hi] [button]
    #
    # We check if the last few bytes look like contact_count + scan_time + button.

    # Try to find contact count: it should be a small number (0-10) near the end
    # This is very heuristic and may not work for all devices.
    # For proper decoding, use --descriptor first to get the layout.

    # Check for a simple PTP pattern: 7-byte finger records
    # [tip_conf] [contact_id] [x_lo] [x_hi] [y_lo] [y_hi] [?]
    # followed by [contact_count] [scan_time_lo] [scan_time_hi] [button]

    # Also try 9-byte finger records (with width + height)

    for finger_size in [6, 7, 8, 9, 10]:
        # Assume last 3-4 bytes are contact_count + scan_time + button
        for trailer_size in [3, 4]:
            finger_area = len(body) - trailer_size
            if finger_area <= 0 or finger_area % finger_size != 0:
                continue

            max_fingers = finger_area // finger_size
            if max_fingers < 1 or max_fingers > 10:
                continue

            # Check if contact_count field is plausible
            cc = body[finger_area]
            if cc > max_fingers:
                continue
            if cc == 0 and any(body[j] != 0 for j in range(finger_area)):
                # Non-zero finger data but 0 contact count -- unlikely
                continue

            # Plausible layout found
            if cc > 0:
                scan_time_offset = finger_area + 1
                if scan_time_offset + 2 <= len(body):
                    scan_time = body[scan_time_offset] | (body[scan_time_offset + 1] << 8)
                else:
                    scan_time = 0

                button_offset = scan_time_offset + 2
                button = body[button_offset] if button_offset < len(body) else 0

                print(f"          -> Possible PTP decode (finger_size={finger_size}, trailer={trailer_size}):")
                print(f"             Contact Count={cc}, Scan Time={scan_time} (0x{scan_time:04X}), Button={button}")

                for f_idx in range(cc):
                    f_data = body[f_idx * finger_size:(f_idx + 1) * finger_size]
                    if len(f_data) < 6:
                        continue

                    # Common layout: byte 0 = tip+conf bits, byte 1 = contact_id,
                    # bytes 2-3 = X (LE), bytes 4-5 = Y (LE)
                    tip_conf = f_data[0]
                    cid = f_data[1]
                    x = f_data[2] | (f_data[3] << 8)
                    y = f_data[4] | (f_data[5] << 8)

                    extras = ""
                    if finger_size >= 8:
                        w = f_data[6]
                        h = f_data[7] if finger_size >= 8 else 0
                        extras = f", W={w}, H={h}"
                    if finger_size >= 9:
                        extras += f", pressure={f_data[8]}"

                    print(f"             Finger[{f_idx}]: CID={cid}, Tip={tip_conf & 1}, "
                          f"Conf={(tip_conf >> 1) & 1}, X={x}, Y={y}{extras}")

                return  # Found a plausible decode, stop trying


# ---------------------------------------------------------------------------
# Feature mode
# ---------------------------------------------------------------------------

# Well-known PTP Feature Report IDs and their meanings
PTP_FEATURE_REPORTS = {
    # Input Mode (Usage 0x0D/0x52): Controls whether device sends mouse or
    # touchpad reports. Value 0 = Mouse, 3 = Touch Pad (PTP).
    "Input Mode": {
        "usage_page": 0x0D,
        "usage": 0x52,
        "decode": lambda data: f"Mode={'Touch Pad (PTP)' if (data[1] if len(data) > 1 else 0) == 3 else 'Mouse' if (data[1] if len(data) > 1 else 0) == 0 else f'Unknown ({data[1] if len(data) > 1 else 0})'}"
    },
    # Max Contact Count (Usage 0x0D/0x55): Number of simultaneous contacts.
    # PTP requires at least 3, typically 5.
    "Max Contact Count": {
        "usage_page": 0x0D,
        "usage": 0x55,
        "decode": lambda data: f"Max Contacts={data[1] if len(data) > 1 else '?'}"
    },
    # Device Certification Status (Usage 0x0D/0xC5): 256-byte blob,
    # Windows checks for this but often accepts all-zero.
    "Device Certification Status": {
        "usage_page": 0x0D,
        "usage": 0xC5,
        "decode": lambda data: f"Cert blob ({len(data)} bytes)"
    },
    # Latency Mode (Usage 0x0D/0x60): 0 = Normal, 1 = High (low latency).
    "Latency Mode": {
        "usage_page": 0x0D,
        "usage": 0x60,
        "decode": lambda data: f"Mode={'High (low latency)' if (data[1] if len(data) > 1 else 0) == 1 else 'Normal'}"
    },
}


def do_features(vid: Optional[int], pid: Optional[int]):
    """Read Feature reports using hidapi."""
    try:
        import hid
    except ImportError:
        print("ERROR: hidapi is required for feature mode.")
        print("  Install: pip3 install hidapi")
        sys.exit(1)

    if vid is not None and pid is not None:
        devices = hid.enumerate(vid, pid)
    else:
        devices = hid.enumerate()
        if not devices:
            print("No HID devices found.")
            sys.exit(1)

        print("Available HID devices:")
        for idx, d in enumerate(devices):
            prod = d.get("product_string", "?") or "?"
            mfg = d.get("manufacturer_string", "?") or "?"
            print(f"  [{idx}] VID=0x{d['vendor_id']:04X} PID=0x{d['product_id']:04X}  "
                  f"{mfg} - {prod}  "
                  f"(iface={d.get('interface_number', '?')}, "
                  f"usage=0x{d.get('usage_page', 0):04X}/0x{d.get('usage', 0):04X})")

        try:
            choice = int(input("\nSelect device number: "))
            devices = [devices[choice]]
        except (ValueError, IndexError, KeyboardInterrupt):
            print("Invalid selection.")
            sys.exit(1)

    if not devices:
        print(f"No HID device found" +
              (f" with VID=0x{vid:04X} PID=0x{pid:04X}" if vid else "") + ".")
        sys.exit(1)

    print(f"\nFound {len(devices)} interface(s):")
    for idx, d in enumerate(devices):
        prod = d.get("product_string", "?") or "?"
        print(f"  [{idx}] interface={d.get('interface_number', '?')}, "
              f"usage=0x{d.get('usage_page', 0):04X}/0x{d.get('usage', 0):04X}, "
              f"product={prod}")

    # Standard PTP feature report IDs to try, plus common IDs
    # The actual IDs depend on the descriptor; we try a broad range.
    feature_ids = list(range(0, 16)) + list(range(0x40, 0x50)) + [0xDB, 0xD1, 0xD3, 0xD0, 0xA1, 0xD9, 0x7F, 0xC3]

    for d_info in devices:
        iface = d_info.get("interface_number", "?")
        usage_page = d_info.get("usage_page", 0)
        usage_val = d_info.get("usage", 0)
        usage = f"0x{usage_page:04X}/0x{usage_val:04X}"

        print(f"\n{'='*72}")
        print(f"Interface {iface} (usage {usage})")

        # Highlight if this looks like a PTP interface
        if usage_page == 0x0D and usage_val == 0x05:
            print("  *** This is a Touch Pad (PTP) interface ***")
        elif usage_page == 0x0D and usage_val == 0x0E:
            print("  *** This is a Device Configuration interface ***")

        try:
            dev = hid.device()
            dev.open_path(d_info["path"])

            found_any = False
            for rid in feature_ids:
                try:
                    data = dev.get_feature_report(rid, 512)
                    if data is None or len(data) == 0:
                        continue

                    found_any = True

                    # Raw hex
                    max_show = 64
                    hex_str = " ".join(f"{b:02x}" for b in data[:max_show])
                    if len(data) > max_show:
                        hex_str += f" ... ({len(data)} bytes total)"

                    # Try to identify well-known PTP feature reports
                    label = ""
                    for name, info in PTP_FEATURE_REPORTS.items():
                        # We cannot perfectly match without descriptor parse,
                        # but we annotate based on report ID heuristics
                        pass

                    print(f"  Feature 0x{rid:02X}: [{len(data):3d}B] {hex_str}")

                    # If it is small enough, show full hex on next line
                    if len(data) > max_show:
                        print(f"    Full hex:")
                        for row in range(0, len(data), 16):
                            chunk = data[row:row + 16]
                            h = " ".join(f"{b:02x}" for b in chunk)
                            print(f"      {row:04X}: {h}")

                except Exception:
                    # Most report IDs will fail -- that is expected
                    pass

            if not found_any:
                print("  No feature reports found on this interface.")

            dev.close()

        except Exception as e:
            print(f"  Could not open: {e}")

    print("\nDone.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_vid_pid(s: str) -> int:
    """Parse a VID/PID string, accepting 0x prefix or plain hex."""
    s = s.strip()
    if s.startswith("0x") or s.startswith("0X"):
        return int(s, 16)
    # Try hex first if it looks hex-ish, otherwise decimal
    try:
        return int(s, 16)
    except ValueError:
        return int(s)


def main():
    parser = argparse.ArgumentParser(
        description="Capture HID Report Descriptors, Feature Reports, and Input Reports "
                    "from a Ploopy trackpad or any USB HID device. "
                    "Designed for PTP (Windows Precision Touchpad) research.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  # Read descriptors from a specific device
  sudo python3 capture_ploopy.py --vid 0x2E8A --pid 0xFEDD

  # Interactive device selection (descriptor mode is default)
  sudo python3 capture_ploopy.py

  # Capture live input reports
  sudo python3 capture_ploopy.py --capture --vid 0x2E8A --pid 0xFEDD

  # Read feature reports
  sudo python3 capture_ploopy.py --features

  # Save parsed descriptor to file
  sudo python3 capture_ploopy.py --descriptor --output ploopy_desc.txt

PTP Feature Report Reference:
  Input Mode (0x0D/0x52):    0=Mouse, 3=Touchpad. Host writes this.
  Max Contact Count (0x0D/0x55): Typically 5 for PTP devices.
  Certification (0x0D/0xC5): 256-byte blob. Windows checks presence, not content.
  Latency Mode (0x0D/0x60):  0=Normal, 1=High-performance.
""")

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--descriptor", action="store_true", default=True,
                            help="Read and parse HID Report Descriptors (default)")
    mode_group.add_argument("--capture", action="store_true",
                            help="Capture live Input reports")
    mode_group.add_argument("--features", action="store_true",
                            help="Read Feature reports")

    parser.add_argument("--vid", type=str, default=None,
                        help="USB Vendor ID (hex, e.g. 0x2E8A)")
    parser.add_argument("--pid", type=str, default=None,
                        help="USB Product ID (hex, e.g. 0xFEDD)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Save parsed output to file (descriptor mode only)")

    args = parser.parse_args()

    vid = parse_vid_pid(args.vid) if args.vid else None
    pid = parse_vid_pid(args.pid) if args.pid else None

    # Ensure both VID and PID are provided if either is
    if (vid is None) != (pid is None):
        print("ERROR: --vid and --pid must both be specified, or neither.")
        sys.exit(1)

    if args.capture:
        do_capture(vid, pid)
    elif args.features:
        do_features(vid, pid)
    else:
        do_descriptor(vid, pid, args.output)


if __name__ == "__main__":
    main()
