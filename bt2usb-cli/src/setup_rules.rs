//! Install/uninstall OS-specific device access rules (udev on Linux).

use anyhow::Result;
use colored::Colorize;

use crate::transport;

/// udev rules file path on Linux.
const UDEV_RULES_PATH: &str = "/etc/udev/rules.d/99-bt2usb.rules";

/// Generate udev rules content for both bt2usb USB identities.
fn udev_rules_content() -> String {
    format!(
        r#"# bt2usb — allow non-root access to the BLE-to-USB bridge
# Apple MT2 mode (macOS identity)
SUBSYSTEM=="hidraw", ATTRS{{idVendor}}=="{:04x}", ATTRS{{idProduct}}=="{:04x}", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{{idVendor}}=="{:04x}", ATTRS{{idProduct}}=="{:04x}", MODE="0666", TAG+="uaccess"
# Generic bt2usb mode (Linux/Windows identity)
SUBSYSTEM=="hidraw", ATTRS{{idVendor}}=="{:04x}", ATTRS{{idProduct}}=="{:04x}", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{{idVendor}}=="{:04x}", ATTRS{{idProduct}}=="{:04x}", MODE="0666", TAG+="uaccess"
"#,
        transport::USB_VID_APPLE,
        transport::USB_PID_MT2,
        transport::USB_VID_APPLE,
        transport::USB_PID_MT2,
        transport::USB_VID_BT2USB,
        transport::USB_PID_BT2USB,
        transport::USB_VID_BT2USB,
        transport::USB_PID_BT2USB,
    )
}

pub fn cmd_setup_rules(uninstall: bool) -> Result<()> {
    match std::env::consts::OS {
        "linux" => {
            if uninstall {
                linux_uninstall()
            } else {
                linux_install()
            }
        }
        "macos" => {
            println!("{}", "No special rules are required on macOS.".green());
            println!(
                "{}",
                "HID devices are accessible to all users by default.".dimmed()
            );
            Ok(())
        }
        os => {
            eprintln!("{}: setup-rules is not supported on '{os}'.", "Error".red());
            eprintln!("This command currently supports Linux (udev rules).");
            std::process::exit(1);
        }
    }
}

fn linux_install() -> Result<()> {
    let rules = udev_rules_content();

    // Check for existing rules
    if std::path::Path::new(UDEV_RULES_PATH).exists() {
        if let Ok(existing) = std::fs::read_to_string(UDEV_RULES_PATH) {
            if existing == rules {
                println!("{}", "Rules are already installed and up to date.".green());
                return Ok(());
            }
        }
        println!(
            "{}",
            "Existing rules file found — it will be overwritten.".yellow()
        );
    }

    // Try direct write first; fall back to sudo
    if std::fs::write(UDEV_RULES_PATH, &rules).is_ok() {
        println!("Installed {}", UDEV_RULES_PATH.bold());
    } else {
        println!(
            "{}",
            "Root privileges required. Running with sudo...".dimmed()
        );

        // Write rules to a temp file, then sudo-copy into place
        let tmp = std::env::temp_dir().join("99-bt2usb.rules");
        std::fs::write(&tmp, &rules)
            .map_err(|e| anyhow::anyhow!("Failed to write temp file {}: {e}", tmp.display()))?;

        let status = std::process::Command::new("sudo")
            .args(["cp", &tmp.to_string_lossy(), UDEV_RULES_PATH])
            .status()
            .map_err(|e| anyhow::anyhow!("Failed to run sudo: {e}"))?;
        let _ = std::fs::remove_file(&tmp);

        if !status.success() {
            anyhow::bail!("sudo cp failed (exit code: {:?})", status.code());
        }
        println!("Installed {}", UDEV_RULES_PATH.bold());
    }

    // Reload udev rules
    println!("{}", "Reloading udev rules...".dimmed());
    let reload = std::process::Command::new("sudo")
        .args(["udevadm", "control", "--reload-rules"])
        .status();
    let trigger = std::process::Command::new("sudo")
        .args(["udevadm", "trigger"])
        .status();

    match (reload, trigger) {
        (Ok(r), Ok(t)) if r.success() && t.success() => {
            println!("{}", "udev rules reloaded successfully.".green());
        }
        _ => {
            println!(
                "{}",
                "Could not reload rules automatically. Run manually:".yellow()
            );
            println!("  sudo udevadm control --reload-rules && sudo udevadm trigger");
        }
    }

    println!();
    println!(
        "{}",
        "You may need to re-plug the device for rules to take effect.".dimmed()
    );
    Ok(())
}

fn linux_uninstall() -> Result<()> {
    if !std::path::Path::new(UDEV_RULES_PATH).exists() {
        println!("{}", "No rules file found — nothing to remove.".yellow());
        return Ok(());
    }

    // Try direct remove; fall back to sudo
    if std::fs::remove_file(UDEV_RULES_PATH).is_ok() {
        println!("Removed {}", UDEV_RULES_PATH.bold());
    } else {
        println!(
            "{}",
            "Root privileges required. Running with sudo...".dimmed()
        );
        let status = std::process::Command::new("sudo")
            .args(["rm", UDEV_RULES_PATH])
            .status()
            .map_err(|e| anyhow::anyhow!("Failed to run sudo: {e}"))?;
        if !status.success() {
            anyhow::bail!("sudo rm failed (exit code: {:?})", status.code());
        }
        println!("Removed {}", UDEV_RULES_PATH.bold());
    }

    // Reload udev rules
    let _ = std::process::Command::new("sudo")
        .args(["udevadm", "control", "--reload-rules"])
        .status();
    println!("{}", "udev rules reloaded.".green());
    Ok(())
}
