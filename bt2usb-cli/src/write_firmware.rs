//! Flash firmware to a Raspberry Pi Pico in BOOTSEL mode.
//!
//! Converts the embedded ELF to UF2 and writes it to the mounted Pico volume.

use anyhow::Result;
use colored::Colorize;

use crate::FIRMWARE_ELF;

/// Well-known mount point name for a Pico in BOOTSEL mode.
const PICO_VOLUME_NAME: &str = "RPI-RP2";

pub fn cmd_write_firmware(path: Option<&str>) -> Result<()> {
    if FIRMWARE_ELF.is_empty() {
        eprintln!("No firmware ELF embedded in this CLI build.");
        eprintln!("Build with `just build-all` to embed the firmware ELF.");
        std::process::exit(1);
    }

    // Convert ELF → UF2
    println!("{}", "Converting ELF to UF2...".dimmed());
    let uf2_data = crate::uf2::elf_to_uf2(FIRMWARE_ELF)?;
    println!(
        "UF2: {} blocks, {} bytes",
        uf2_data.len() / 512,
        uf2_data.len()
    );

    // Resolve target directory
    let mount_path = match path {
        Some(p) => std::path::PathBuf::from(p),
        None => {
            println!("{}", "Auto-detecting Pico in BOOTSEL mode...".dimmed());
            match find_pico_mount() {
                Some(p) => {
                    println!("Found Pico at {}", p.display().to_string().bold());
                    p
                }
                None => {
                    eprintln!(
                        "{}: Could not find a mounted Pico ({PICO_VOLUME_NAME}).",
                        "Error".red()
                    );
                    eprintln!("Put the Pico into BOOTSEL mode (hold BOOTSEL while plugging in),");
                    eprintln!("then either re-run or specify the mount path manually:");
                    eprintln!("  bt2usb-cli write-firmware /path/to/{PICO_VOLUME_NAME}");
                    std::process::exit(1);
                }
            }
        }
    };

    // Validate mount point: must be a directory containing INFO_UF2.TXT
    if !mount_path.is_dir() {
        anyhow::bail!(
            "'{}' is not a directory. Provide the mount point of the Pico.",
            mount_path.display()
        );
    }
    if !mount_path.join("INFO_UF2.TXT").exists() {
        anyhow::bail!(
            "'{}' does not look like a Pico in BOOTSEL mode (missing INFO_UF2.TXT).",
            mount_path.display()
        );
    }

    let dest = mount_path.join("bt2usb.uf2");
    println!("Writing to {}...", dest.display().to_string().bold());
    std::fs::write(&dest, &uf2_data)?;

    println!(
        "{}",
        "Firmware written. The Pico will reboot automatically.".green()
    );
    Ok(())
}

/// Try to find a mounted Pico in BOOTSEL mode by searching common mount locations.
fn find_pico_mount() -> Option<std::path::PathBuf> {
    let candidates: Vec<std::path::PathBuf> = if cfg!(target_os = "macos") {
        vec![std::path::PathBuf::from(format!(
            "/Volumes/{PICO_VOLUME_NAME}"
        ))]
    } else if cfg!(target_os = "linux") {
        // Check /media/$USER/RPI-RP2 and /run/media/$USER/RPI-RP2
        let mut paths = Vec::new();
        if let Ok(user) = std::env::var("USER") {
            paths.push(std::path::PathBuf::from(format!(
                "/media/{user}/{PICO_VOLUME_NAME}"
            )));
            paths.push(std::path::PathBuf::from(format!(
                "/run/media/{user}/{PICO_VOLUME_NAME}"
            )));
        }
        // Also scan /media/*/RPI-RP2 in case username doesn't match
        if let Ok(entries) = std::fs::read_dir("/media") {
            for entry in entries.flatten() {
                let p = entry.path().join(PICO_VOLUME_NAME);
                if !paths.contains(&p) {
                    paths.push(p);
                }
            }
        }
        paths
    } else if cfg!(target_os = "windows") {
        // On Windows, scan drive letters for INFO_UF2.TXT
        (b'D'..=b'Z')
            .map(|c| std::path::PathBuf::from(format!("{}:\\", c as char)))
            .collect()
    } else {
        vec![]
    };

    candidates
        .into_iter()
        .find(|p| p.is_dir() && p.join("INFO_UF2.TXT").exists())
}
