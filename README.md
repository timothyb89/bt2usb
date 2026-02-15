# bt2usb

BLE HID to USB HID Bridge for Raspberry Pi Pico W.

## Disclosure

🚨 AI SLOP WARNING 🚨

This was largely made to experiment with AI tooling, and the code was predominantly written by LLMs. The functionality has been reviewed but should be used with care. This isn't a particularly dangerous project and probably doesn't have the capability to break anything other than the Pico you flash it to, if that, but you've been warned.

## Overview

This firmware connects to Bluetooth LE HID devices (keyboards, mice, custom devices) and translates their input into USB HID, allowing them to be used with a USB switch between computers without re-pairing.

I built this to pair with the fantastic [Full Scroll Dial by Engineer Bo](https://www.youtube.com/watch?v=tzqJ1rJURgs) so I could easily switch it between computers with a USB switch. The idea is inspired by the excellent [`hid-remapper`](https://github.com/jfedor2/hid-remapper), but written in embedded Rust and with specific device profiles.

## Requirements

- A Raspberry Pi Pico W with the CYW43439 wireless chipset. Other RP2040 boards with different wireless chipsets will not work. 

No other hardware is required, aside from the BLE device you want to convert to USB.

## Features

- Rough profiles for the Full Scroll Dial and Logitech MX Master 3S
- High reporting rate for connected devices
- Experimental 16-bit reporting mode for the Full Scroll Dial for native scroll velocity

## TODOs

- Bonding and stored bonds. Currently devices must pair from scratch on each start. The Full Scroll Dial will happily pair whenever no other devices is connected. The MX Master 3S must be paired each time.
- Bond management / deletion / reset
- Various resiliency fixes. The Pico's BLE stack is a bit finnicky.
- Select between devices and profiles at runtime (with a HID/serial interface + WebUSB)
- Some solution for status reporting
- Generic mouse/keyboard device profiles

Longer term desired TODOs:

- Bluetooth Classic support. Supported by the Pico W, but not by `trouble`.

## Building

### Quick Start with Just

This project uses [`just`](https://github.com/casey/just) as a command runner. Install it with:

```bash
cargo install just
```

Then run the setup command to download firmware and check for required tools:

```bash
just setup
```

### Build Commands

```bash
# Build for development with debug probe
just dev

# Build release UF2 for drag-and-drop flashing
just release

# Download CYW43 firmware files (run once, or after clean)
just download-firmware

# Flash with debug probe (probe-rs)
just run release
```

### Manual Build (without just)

1. Install prerequisites:
```bash
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs probe-rs
```

2. Download CYW43 firmware:
```bash
# Windows PowerShell
$fw_dir = "bt2usb/cyw43-firmware"
$base_url = "https://raw.githubusercontent.com/embassy-rs/embassy/main/cyw43-firmware"
New-Item -ItemType Directory -Force -Path $fw_dir
foreach ($file in @("43439A0.bin", "43439A0_btfw.bin", "43439A0_clm.bin")) {
    Invoke-WebRequest -Uri "$base_url/$file" -OutFile "$fw_dir/$file"
}
```

3. Build and generate UF2:
```bash
cargo build --package bt2usb --release
elf2uf2-rs target/thumbv6m-none-eabi/release/bt2usb bt2usb/bt2usb.uf2
```

## Flashing

### Method 1: Drag-and-Drop (No debugger required)

1. Build the UF2 file: `just release`
2. Hold the BOOTSEL button on the Pico W while plugging it into USB
3. The Pico will appear as a USB mass storage device (drive letter like `RPI-RP2`)
4. Drag and drop `bt2usb/bt2usb.uf2` onto the drive
5. The Pico will automatically reboot and start running the firmware

### Method 2: With Debug Probe (for development)

If you have a debug probe connected:

```bash
just dev          # Development build with RTT logging
just run release  # Release build
```

## License

MIT OR Apache-2.0
