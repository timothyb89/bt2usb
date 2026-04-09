#!/usr/bin/env python3
"""
Read BlueZ's cached GATT database via D-Bus.

This bypasses bleak entirely and reads the GATT services/characteristics
that BlueZ has already discovered, even if the HOG plugin has claimed them.

Usage:
    python3 dbus_gatt_dump.py                          # list all BLE devices
    python3 dbus_gatt_dump.py E0:EB:40:F1:10:19       # dump GATT for specific device
    python3 dbus_gatt_dump.py E0:EB:40:F1:10:19 --read # also read characteristic values

Requires: pydbus or dbus-python (usually pre-installed on Linux)
    pip3 install pydbus   (if not available)

The device must be connected via bluetoothctl for this to work.
"""

import sys
import struct

# Try pydbus first, fall back to raw subprocess + busctl
try:
    import pydbus
    HAS_PYDBUS = True
except ImportError:
    HAS_PYDBUS = False

import subprocess
import re

BLUEZ_SERVICE = "org.bluez"
OBJECT_MANAGER_IFACE = "org.freedesktop.DBus.ObjectManager"
DEVICE_IFACE = "org.bluez.Device1"
SERVICE_IFACE = "org.bluez.GattService1"
CHAR_IFACE = "org.bluez.GattCharacteristic1"
DESC_IFACE = "org.bluez.GattDescriptor1"

KNOWN_SERVICES = {
    "00001800-0000-1000-8000-00805f9b34fb": "Generic Access",
    "00001801-0000-1000-8000-00805f9b34fb": "Generic Attribute",
    "0000180a-0000-1000-8000-00805f9b34fb": "Device Information",
    "0000180f-0000-1000-8000-00805f9b34fb": "Battery Service",
    "00001812-0000-1000-8000-00805f9b34fb": "HID Service",
}

KNOWN_CHARS = {
    "00002a00-0000-1000-8000-00805f9b34fb": "Device Name",
    "00002a01-0000-1000-8000-00805f9b34fb": "Appearance",
    "00002a04-0000-1000-8000-00805f9b34fb": "Preferred Conn Params",
    "00002a19-0000-1000-8000-00805f9b34fb": "Battery Level",
    "00002a24-0000-1000-8000-00805f9b34fb": "Model Number",
    "00002a25-0000-1000-8000-00805f9b34fb": "Serial Number",
    "00002a26-0000-1000-8000-00805f9b34fb": "Firmware Revision",
    "00002a27-0000-1000-8000-00805f9b34fb": "Hardware Revision",
    "00002a28-0000-1000-8000-00805f9b34fb": "Software Revision",
    "00002a29-0000-1000-8000-00805f9b34fb": "Manufacturer Name",
    "00002a4a-0000-1000-8000-00805f9b34fb": "HID Information",
    "00002a4b-0000-1000-8000-00805f9b34fb": "Report Map",
    "00002a4c-0000-1000-8000-00805f9b34fb": "HID Control Point",
    "00002a4d-0000-1000-8000-00805f9b34fb": "Report",
    "00002a4e-0000-1000-8000-00805f9b34fb": "Protocol Mode",
    "00002a50-0000-1000-8000-00805f9b34fb": "PnP ID",
}

KNOWN_DESCS = {
    "00002902-0000-1000-8000-00805f9b34fb": "CCCD",
    "00002908-0000-1000-8000-00805f9b34fb": "Report Reference",
}

REPORT_TYPES = {0: "Other", 1: "Input", 2: "Output", 3: "Feature"}


def addr_to_path(addr: str) -> str:
    """Convert BT address to BlueZ D-Bus path suffix."""
    return "dev_" + addr.replace(":", "_")


def hex_dump(data: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data)


def dump_via_busctl(target_addr: str | None = None, read_values: bool = False):
    """Use busctl to introspect BlueZ objects — no Python D-Bus library needed."""

    # Get all managed objects via busctl
    result = subprocess.run(
        ["busctl", "tree", "org.bluez"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print("Error: busctl failed. Is BlueZ running?")
        print(result.stderr)
        sys.exit(1)

    lines = result.stdout.strip().split("\n")
    paths = [line.strip().lstrip("├─└│ ") for line in lines if "/dev_" in line or "/service" in line or "/char" in line or "/desc" in line]

    if not target_addr:
        # List all devices
        print("Connected BLE devices:")
        print()
        for path in paths:
            if "/dev_" in path and "/service" not in path and "/char" not in path:
                # Get device properties
                try:
                    props = subprocess.run(
                        ["busctl", "introspect", "org.bluez", path, "org.bluez.Device1"],
                        capture_output=True, text=True
                    )
                    name = "?"
                    addr = "?"
                    connected = "?"
                    services_resolved = "?"
                    for pline in props.stdout.split("\n"):
                        if ".Name " in pline:
                            m = re.search(r'"([^"]*)"', pline)
                            if m:
                                name = m.group(1)
                        if ".Address " in pline:
                            m = re.search(r'"([^"]*)"', pline)
                            if m:
                                addr = m.group(1)
                        if ".Connected " in pline:
                            connected = "yes" if "true" in pline.lower() else "no"
                        if ".ServicesResolved " in pline:
                            services_resolved = "yes" if "true" in pline.lower() else "no"
                    print(f"  {addr}  {name}")
                    print(f"    Connected: {connected}, ServicesResolved: {services_resolved}")
                    print(f"    Path: {path}")
                    print()
                except Exception:
                    pass
        return

    # Dump GATT for a specific device
    addr_suffix = addr_to_path(target_addr)

    # Find the device path
    dev_path = None
    for path in paths:
        if addr_suffix in path and "/service" not in path and "/char" not in path and "/desc" not in path:
            dev_path = path
            break

    if not dev_path:
        print(f"Device {target_addr} not found in BlueZ. Is it connected?")
        return

    # Check ServicesResolved
    props = subprocess.run(
        ["busctl", "introspect", "org.bluez", dev_path, "org.bluez.Device1"],
        capture_output=True, text=True
    )
    resolved = False
    for pline in props.stdout.split("\n"):
        if ".ServicesResolved " in pline:
            resolved = "true" in pline.lower()
    print(f"Device: {target_addr}")
    print(f"Path: {dev_path}")
    print(f"ServicesResolved: {resolved}")
    if not resolved:
        print()
        print("WARNING: Services not resolved! The device may not be connected,")
        print("or GATT discovery hasn't completed yet.")
        print("Try: bluetoothctl -> connect <address>")
    print()

    # Find all services, characteristics, descriptors under this device
    dev_prefix = dev_path
    services = {}
    chars = {}
    descs = {}

    for path in paths:
        if not path.startswith(dev_prefix + "/"):
            continue
        rel = path[len(dev_prefix) + 1:]
        parts = rel.split("/")

        if len(parts) == 1 and parts[0].startswith("service"):
            services[path] = {"chars": {}}
        elif len(parts) == 2 and parts[1].startswith("char"):
            parent_svc = dev_prefix + "/" + parts[0]
            if parent_svc in services:
                services[parent_svc]["chars"][path] = {"descs": {}}
            chars[path] = parent_svc
        elif len(parts) == 3 and parts[2].startswith("desc"):
            parent_char = dev_prefix + "/" + parts[0] + "/" + parts[1]
            parent_svc = dev_prefix + "/" + parts[0]
            if parent_svc in services and parent_char in services[parent_svc]["chars"]:
                services[parent_svc]["chars"][parent_char]["descs"][path] = {}
            descs[path] = parent_char

    if not services:
        print("No GATT services found under this device!")
        print("This might mean:")
        print("  - Device is not connected")
        print("  - GATT discovery hasn't completed (ServicesResolved=false)")
        print("  - BlueZ cache was cleared (try reconnecting)")
        return

    # Now read properties of each service/char/desc
    for svc_path in sorted(services.keys()):
        svc_uuid = read_property_busctl(svc_path, SERVICE_IFACE, "UUID")
        svc_name = KNOWN_SERVICES.get(svc_uuid, "")
        is_hid = "1812" in (svc_uuid or "")
        marker = " *** HID ***" if is_hid else ""

        print(f"{'=' * 70}")
        print(f"Service: {svc_uuid}  {svc_name}{marker}")
        print(f"  Path: {svc_path}")
        print()

        for char_path in sorted(services[svc_path]["chars"].keys()):
            char_uuid = read_property_busctl(char_path, CHAR_IFACE, "UUID")
            char_name = KNOWN_CHARS.get(char_uuid, "")
            char_flags = read_property_busctl(char_path, CHAR_IFACE, "Flags")

            print(f"  Characteristic: {char_uuid}  {char_name}")
            print(f"    Path: {char_path}")
            if char_flags:
                print(f"    Flags: {char_flags}")

            # Try to read value if requested and readable
            if read_values and char_flags and "read" in char_flags.lower():
                value = read_char_value_busctl(char_path)
                if value is not None:
                    print(f"    Value: [{len(value)}B] {hex_dump(value)}")
                    # Try UTF-8
                    try:
                        text = bytes(value).decode("utf-8")
                        if text.isprintable():
                            print(f"    As string: \"{text}\"")
                    except (UnicodeDecodeError, ValueError):
                        pass

                    # Decode HID Information
                    if char_uuid == "00002a4a-0000-1000-8000-00805f9b34fb" and len(value) >= 4:
                        bcd_hid = value[0] | (value[1] << 8)
                        country = value[2]
                        flags = value[3]
                        print(f"    HID Version: {bcd_hid >> 8}.{(bcd_hid >> 4) & 0xF}.{bcd_hid & 0xF}")
                        print(f"    Country Code: {country}")
                        print(f"    Flags: 0x{flags:02x} ({'Remote Wake' if flags & 1 else ''} {'Normally Connectable' if flags & 2 else ''})")

                    # Decode PnP ID
                    if char_uuid == "00002a50-0000-1000-8000-00805f9b34fb" and len(value) >= 7:
                        vendor_src = value[0]
                        vid = value[1] | (value[2] << 8)
                        pid = value[3] | (value[4] << 8)
                        ver = value[5] | (value[6] << 8)
                        src_name = {1: "Bluetooth SIG", 2: "USB-IF"}.get(vendor_src, f"Unknown({vendor_src})")
                        print(f"    Vendor ID Source: {src_name}")
                        print(f"    Vendor ID: 0x{vid:04X}")
                        print(f"    Product ID: 0x{pid:04X}")
                        print(f"    Version: 0x{ver:04X}")

            # Descriptors
            for desc_path in sorted(services[svc_path]["chars"][char_path]["descs"].keys()):
                desc_uuid = read_property_busctl(desc_path, DESC_IFACE, "UUID")
                desc_name = KNOWN_DESCS.get(desc_uuid, "")

                desc_value = read_desc_value_busctl(desc_path)
                value_str = ""
                if desc_value is not None:
                    value_str = f" [{len(desc_value)}B] {hex_dump(desc_value)}"

                    # Decode Report Reference
                    if desc_uuid == "00002908-0000-1000-8000-00805f9b34fb" and len(desc_value) >= 2:
                        rid = desc_value[0]
                        rtype = REPORT_TYPES.get(desc_value[1], f"Unknown({desc_value[1]})")
                        value_str += f"  -> Report ID=0x{rid:02X}, Type={rtype}"

                    # Decode CCCD
                    if desc_uuid == "00002902-0000-1000-8000-00805f9b34fb" and len(desc_value) >= 2:
                        cccd = desc_value[0] | (desc_value[1] << 8)
                        n = "Notif ON" if cccd & 1 else "Notif OFF"
                        i = ", Indic ON" if cccd & 2 else ""
                        value_str += f"  ({n}{i})"

                print(f"    Descriptor: {desc_uuid}  {desc_name}{value_str}")

            print()


def read_property_busctl(path: str, iface: str, prop: str) -> str | None:
    """Read a single D-Bus property via busctl."""
    try:
        result = subprocess.run(
            ["busctl", "get-property", "org.bluez", path, iface, prop],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            # Parse busctl output: 's "value"' or 'as N "v1" "v2"' or 'b true'
            out = result.stdout.strip()
            if out.startswith('s "'):
                return out[3:-1]  # strip 's "' and '"'
            elif out.startswith('as '):
                # Array of strings
                return out
            elif out.startswith('b '):
                return out[2:]
            return out
    except Exception:
        pass
    return None


def read_char_value_busctl(path: str) -> list[int] | None:
    """Read a characteristic value via busctl call to ReadValue."""
    try:
        result = subprocess.run(
            ["busctl", "call", "org.bluez", path,
             "org.bluez.GattCharacteristic1", "ReadValue", "a{sv}", "0"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            # Output format: "ay N byte1 byte2 ..."
            out = result.stdout.strip()
            if out.startswith("ay "):
                parts = out.split()
                if len(parts) >= 2:
                    count = int(parts[1])
                    values = [int(p) for p in parts[2:2 + count]]
                    return values
    except Exception:
        pass
    return None


def read_desc_value_busctl(path: str) -> list[int] | None:
    """Read a descriptor value via busctl call to ReadValue."""
    try:
        result = subprocess.run(
            ["busctl", "call", "org.bluez", path,
             "org.bluez.GattDescriptor1", "ReadValue", "a{sv}", "0"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            out = result.stdout.strip()
            if out.startswith("ay "):
                parts = out.split()
                if len(parts) >= 2:
                    count = int(parts[1])
                    values = [int(p) for p in parts[2:2 + count]]
                    return values
    except Exception:
        pass
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Dump BlueZ GATT database via D-Bus (bypasses bleak/HOG issues)"
    )
    parser.add_argument("address", nargs="?", help="Bluetooth address (omit to list devices)")
    parser.add_argument("--read", action="store_true", help="Also read characteristic values")
    args = parser.parse_args()

    dump_via_busctl(args.address, args.read)


if __name__ == "__main__":
    main()
