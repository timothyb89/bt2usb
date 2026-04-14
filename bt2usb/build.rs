use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// Minimum embassy-usb handler slot count required by the firmware.
/// 4 HID interfaces (keyboard, mouse, MT2 trackpad, RPC) + device handler = 5,
/// rounded up with headroom. If this isn't set in `.cargo/config.toml`, embassy-usb
/// falls back to its default of 4 and silently overflows at runtime, bricking the
/// device on boot. Fail the build instead.
const MIN_EMBASSY_USB_HANDLER_COUNT: u32 = 8;

fn main() {
    println!("cargo:rerun-if-env-changed=EMBASSY_USB_MAX_HANDLER_COUNT");
    let count = env::var("EMBASSY_USB_MAX_HANDLER_COUNT")
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(0);
    if count < MIN_EMBASSY_USB_HANDLER_COUNT {
        panic!(
            "EMBASSY_USB_MAX_HANDLER_COUNT must be set to at least {MIN_EMBASSY_USB_HANDLER_COUNT} \
             (currently {}). Set it in the workspace-root .cargo/config.toml under [env]; cargo \
             only discovers config files by walking up from the invocation cwd, so a value placed \
             only in bt2usb/.cargo/config.toml will be missed when building from the repo root.",
            if count == 0 { "unset".to_string() } else { count.to_string() },
        );
    }

    // Put `memory.x` in our output directory and ensure it's on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
