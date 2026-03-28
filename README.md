# bt2usb

BLE HID to USB HID Bridge for Raspberry Pi Pico W.

## Overview

This firmware connects to Bluetooth LE HID devices (mainly mice) and translates their input into USB HID, allowing them to be used with a USB switch between computers without re-pairing.

I built this to pair with the fantastic
[Full Scroll Dial by Engineer Bo](https://www.youtube.com/watch?v=tzqJ1rJURgs)
so I could easily switch it between computers with a USB switch. The idea is
inspired by the excellent
[`hid-remapper`](https://github.com/jfedor2/hid-remapper), but without support
for actually remapping inputs and instead focusing on providing native-like
input feel and explicit support for a few tricky devices. It's since evolved to
help bridge OS support gaps by emulating supported devices on OSes that wouldn't
otherwise support them (like the Full Scroll Dial on macOS).

Notably, it fully supports the high resolution scrolling events emitted by the
Full Scroll Dial and translates them to USB HID with no noticeable loss of
scrolling precision versus using the dial natively via Bluetooth. Additionally,
the reporting rate (both for input via Bluetooth and output via USB HID) is
enough to ensure fairly minimal added latency compared to connecting directly
to the target system via Bluetooth.

Moreover, it also supports high-resolution scrolling on macOS without any
software running on the host. macOS [traditionally does not support][ploopy]
high-resolution scrolling for third-party input devices, accepting only
line-based scrolling events to which it applies an aggressive acceleration
curve. `bt2usb` works around this limitation by emulating an Apple Magic
Trackpad 2 and synthesizing touch scroll events. This allows accurate
pixel-level scrolling without any custom driver, and since `bt2usb` can
detect when connected to macOS hosts, will automatically switch between
standard HID and Magic Trackpad emulation, even on a USB switch.

[ploopy]: https://ploopyco.github.io/knob/appendices/macos/

## Features

- High reporting rate for native-like input performance
- Explicit support for high-res scrolling devices, particularly the Full Scroll
  Dial
- macOS support for high-res (smooth, pixel-level) scrolling via Magic Trackpad 2
  emulation
- Supports up to 3 concurrently connected devices

## Device support notes

I've tested a number of BLE devices successfully. The `Generic` (default)
profile should be sufficient for most standard Bluetooth HID mice.

Known to work:
- Full Scroll Dial
- Logitech MX Master 3
- Keychron M3 8K

Not currently supported:
- Keyboards
- Game controllers

We might consider supporting keyboards and game controllers in the future.

## Requirements

- A Raspberry Pi Pico W with the CYW43439 wireless chipset. Other RP2040 boards with different wireless chipsets will not work. 
- A USB cable

No other hardware is required, aside from the BLE device you want to convert to USB.

For development and full debugging support you'll want a debug probe. Raspberry
Pi sells [an official debug probe](rpi-probe), but you can also build your own
using another Pico running the [`debugprobe`
firmware](https://github.com/raspberrypi/debugprobe).

[rpi-probe]: https://www.raspberrypi.com/products/debug-probe/

## TODOs

- Improved bond storage robustness for the MX Master mice
- Generic mouse/keyboard device profiles
- Magic Trackpad 2 input support to emulate a generic touchpad on Windows

Longer term desired TODOs:

- Bluetooth Classic support. Supported by the Pico W, but not by `trouble`.
  Many Bluetooth keyboards only support Bluetooth Classic.

## Flashing

### Method 1: Drag-and-Drop (No debugger required)

1. Download the UF2 file from [the releases 
   page](https://github.com/timothyb89/bt2usb/releases/latest) or build your
   own with: `just release`
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

## Usage

Once flashed, the Pico should expose both a USB HID input, as well as a HID interface for configuration. To configure it, you can use either the web interface or the CLI tool.

### Web UI

You can open the web UI locally in any Chromium-based browser by downloading [`web/index.html`](./web/index.html). Alternatively, you can browse to https://timothyb89.org/bt2usb/ for a publicly-hosted version.

1. Select the "Connect" button at the top of the page to open the "BT2USB Bridge" device.
2. Once connected, use the "Start Scan" button to search for pairable devices.
3. When the desired device is found, the appropriate profile will be auto-selected (the Full Scroll Dial defaults to 16-bit mode). Select "Connect" to begin pairing.

It should connect to the device and begin forwarding input events.

### CLI

Grab `bt2usb-cli` from the releases page and ensure it's in your `$PATH`, then:

```code
# Ensure you can communicate with the device
$ bt2usb-cli status 

# Scan for pairable devices
$ bt2usb-cli scan

# Connect to a device
$ bt2usb-cli connect <address> 

# If needed, change the profile (Full Scroll Dial defaults to 16-bit, profile 3)
$ bt2usb-cli set-profile <address> <profile>

# Mark the device as active (enables auto connect)
$ bt2usb-cli set-active-device <address>

# See `help` for all commands
$ bt2usb-cli help
```

### macOS Support for the Full Scroll Dial

macOS does not support generic trackpads or devices that provide high
resolution scrolling input. We work around that by emulating a Magic Trackpad 2
and synthesizing scroll events on macOS hosts, which is the only input device
that macOS supports that can both provide high-res scroll events and do so over
USB rather than Bluetooth.

To ensure we emulate the correct device for a given host system, on startup,
the firmware first runs an OS probe, then reboots with the proper configuration
for the detected host OS. This means you will notice a quick device reconnect
whenever it's attached.

The OS probing process is designed to work properly with a USB switch, and
will probe again whenever the USB connection is interrupted.

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

## Debugging

The simplest way to view debug logs is to use the CLI:

```
$ bt2usb-cli logs
```

This streams bt2usb's [`defmt` logs][defmt] over the existing USB HID interface
and does not require a debug probe. However, it can only stream logs after the
interface has already been initialized and won't include logs from early in the
boot process.

If you need earlier logs, you'll need a debug probe (either the official
Raspberry Pi debugging probe or one built using another Pico).

Logs will stream as soon as you flash the device. However, `probe-rs` will
disconnect after the device reboots during the init process after it finishes
probing the host OS.

The easiest way around _that_ is to either reattach probe-rs, or use our builtin
helper:

```
$ probe-rs attach --chip RP2040 ../target/thumbv6m-none-eabi/release/bt2usb
# or
$ cargo build --release --features=probe && cargo run -- logs --probe
```

The helper has a more aggressive retry/reconnect loop than probe-rs, and the
firmware is embedded directly in the CLI binary to decode `defmt` messages
without having to specify the binary path. (Note that you will need to rebuild
the CLI if you change logs in the firmware, as it needs an accurate reference
for `defmt`'s interned strings.)

Alternatively, once the device is running, you can force a specific OS to
assume for the next boot, and then trigger a software reset:
```
# Set an OS override
$ bt2usb-cli set-os <win/mac/linux>

# ...in another session using a CLI built with the `probe` feature
$ bt2usb-cli logs --probe

# Then restart the device
$ bt2usb-cli restart
```

[defmt]: https://defmt.ferrous-systems.com/

## Disclosure

🚨 AI SLOP WARNING 🚨

This was largely made to experiment with AI tooling, and the code was predominantly written by LLMs. The functionality has been reviewed but should be used with care. This isn't a particularly dangerous project and probably doesn't have the capability to break anything other than the Pico you flash it to, if that, but you've been warned.

## License

MIT OR Apache-2.0
