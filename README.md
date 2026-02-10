# bt2usb

BLE HID to USB HID Bridge for Raspberry Pi Pico W.

## Disclosure

🚨 AI SLOP WARNING 🚨

This was largely made to experiment with AI tooling, and the code was predominantly written by LLMs. The functionality has been reviewed but should be used with care. This isn't a particularly dangerous project and probably doesn't have the capability to break anything other than the Pico you flash it to, if that, but you've been warned.

## Overview

This firmware connects to Bluetooth LE HID devices (keyboards, mice, custom devices) and translates their input into USB HID, allowing them to be used with a USB switch between computers without re-pairing.

I built this to pair with the fantastic [Full Scroll Dial by Engineer Bo](https://www.youtube.com/watch?v=tzqJ1rJURgs) so I could easily switch it between computers with a USB switch. The idea is inspired by the excellent [`hid-remapper`](https://github.com/jfedor2/hid-remapper), but written in embedded Rust and with specific device profiles.

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

## Building

```bash
cargo build --release
```

## Flashing

With a Picoprobe connected:
```bash
cargo run --release
```

## License

MIT OR Apache-2.0
