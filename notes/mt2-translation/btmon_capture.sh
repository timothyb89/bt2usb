#!/bin/bash
# Capture raw BLE HCI traffic while connecting to the Magic Trackpad 2.
# This bypasses BlueZ's HOG plugin and captures the actual GATT discovery
# and HID report data at the HCI level.
#
# Usage:
#   Terminal 1: sudo bash btmon_capture.sh E0:EB:40:F1:10:19
#   Terminal 2: bluetoothctl -> connect E0:EB:40:F1:10:19
#               (perform gestures on the trackpad)
#   Terminal 1: Ctrl+C to stop
#
# Output: btmon_<timestamp>.log (text) and btmon_<timestamp>.btsnoop (binary)
#
# The .btsnoop file can be opened in Wireshark for detailed analysis.
# The .log file contains human-readable ATT/GATT protocol traces.
#
# After capture, search the log for:
#   - "Read By Group Type" responses → service UUIDs
#   - "Read By Type" responses → characteristic UUIDs
#   - "Find Information" responses → descriptor UUIDs
#   - "Handle Value Notification" → HID report data
#   - Report Reference values → report ID mapping

set -e

ADDR="${1:?Usage: $0 <BT_ADDRESS>}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOGFILE="btmon_${TIMESTAMP}.log"
BTSNOOP="btmon_${TIMESTAMP}.btsnoop"

echo "Starting btmon capture..."
echo "  Text log:   $LOGFILE"
echo "  BTSnoop:    $BTSNOOP"
echo ""
echo "Now connect to the device in another terminal:"
echo "  bluetoothctl -> connect $ADDR"
echo ""
echo "Press Ctrl+C to stop capture."
echo ""

sudo btmon -T -w "$BTSNOOP" 2>&1 | tee "$LOGFILE"
