# BT2USB Project Justfile
# Commands to automate firmware building and flashing

set shell := ["bash", "-c"]

# Default recipe - show available commands
default:
    @just --list

# Download CYW43 firmware files from Embassy repository
download-firmware:
    @echo "Downloading CYW43 firmware files from Embassy repository..."
    @mkdir -p bt2usb/cyw43-firmware
    @echo "  Downloading 43439A0.bin..."
    @curl -fsSL "https://raw.githubusercontent.com/embassy-rs/embassy/main/cyw43-firmware/43439A0.bin" -o "bt2usb/cyw43-firmware/43439A0.bin"
    @echo "  Downloading 43439A0_btfw.bin..."
    @curl -fsSL "https://raw.githubusercontent.com/embassy-rs/embassy/main/cyw43-firmware/43439A0_btfw.bin" -o "bt2usb/cyw43-firmware/43439A0_btfw.bin"
    @echo "  Downloading 43439A0_clm.bin..."
    @curl -fsSL "https://raw.githubusercontent.com/embassy-rs/embassy/main/cyw43-firmware/43439A0_clm.bin" -o "bt2usb/cyw43-firmware/43439A0_clm.bin"
    @echo "Firmware download complete!"

# Build the firmware binary
build profile='release':
    cargo build --package bt2usb --{{profile}}

# Build and convert to UF2 format for drag-and-drop flashing
build-uf2 profile='release': (build profile)
    @echo "Converting to UF2 format..."
    elf2uf2-rs target/thumbv6m-none-eabi/{{profile}}/bt2usb bt2usb/bt2usb.uf2
    @echo "UF2 file created: bt2usb/bt2usb.uf2"
    @echo "To flash: Copy this file to the Pico in BOOTSEL mode"

# Flash and run with probe-rs (requires debug probe)
run profile='dev':
    cargo run --package bt2usb --{{profile}}

# Flash with probe-rs without running RTT console
flash profile='release':
    probe-rs download --chip RP2040 target/thumbv6m-none-eabi/{{profile}}/bt2usb

# Clean build artifacts
clean:
    cargo clean

# Setup: Download firmware and check for required tools
setup: download-firmware
    @echo ""
    @echo "Checking required tools..."
    @-command -v probe-rs > /dev/null && echo "  ✓ probe-rs installed" || echo "  ✗ probe-rs not found (install: cargo install probe-rs)"
    @-command -v elf2uf2-rs > /dev/null && echo "  ✓ elf2uf2-rs installed" || echo "  ✗ elf2uf2-rs not found (install: cargo install elf2uf2-rs)"
    @echo ""
    @echo "Setup complete! Ready to build."

# Quick build and flash cycle for development
dev:
    just run dev

# Full release build with UF2 generation
release:
    just build-uf2 release
