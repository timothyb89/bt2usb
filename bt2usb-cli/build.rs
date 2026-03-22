use std::path::Path;

fn main() {
    let elf_path = "../target/thumbv6m-none-eabi/release/bt2usb";
    println!("cargo:rerun-if-changed={elf_path}");

    let out_dir = std::env::var("OUT_DIR").unwrap();
    let dest = Path::new(&out_dir).join("embedded_elf.rs");

    let elf_src = Path::new(elf_path);
    if elf_src.exists() {
        // Copy the ELF into OUT_DIR, then reference it with concat!(env!("OUT_DIR"), ...).
        // This avoids canonicalize() which produces \\?\ prefixed paths on Windows
        // that break inside string literals.
        let elf_dest = Path::new(&out_dir).join("firmware.elf");
        std::fs::copy(elf_src, &elf_dest).expect("failed to copy firmware ELF to OUT_DIR");

        let size = std::fs::metadata(&elf_dest).unwrap().len();

        // Wrap in a repr(align(4)) struct so the ELF parser can read multi-byte
        // header fields. Plain include_bytes! only guarantees align(1), which
        // causes "Invalid ELF header size or alignment" on some platforms.
        std::fs::write(
            &dest,
            format!(
                concat!(
                    "#[repr(C, align(4))]\n",
                    "struct _FwElf([u8; {size}]);\n",
                    "static _FW_ELF: _FwElf = _FwElf(*include_bytes!(concat!(env!(\"OUT_DIR\"), \"/firmware.elf\")));\n",
                    "pub static FIRMWARE_ELF: &[u8] = &_FW_ELF.0;\n",
                ),
                size = size,
            ),
        )
        .unwrap();
    } else {
        // Firmware not built yet — empty slice, CLI still compiles
        std::fs::write(&dest, "pub static FIRMWARE_ELF: &[u8] = &[];\n").unwrap();
    }
}
