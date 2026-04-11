//! Device RPC command implementations.
//!
//! Each `cmd_*` function sends a request over the transport, waits for a
//! response, and pretty-prints the result.

use anyhow::Result;
use colored::Colorize;
use std::time::Duration;

use crate::protocol::*;
use crate::transport::{format_address, parse_address, Message, Transport};

/// Default request timeout.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(3);

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// Check if a response is Ok, print error if not.
pub fn check_ok(resp: &Response) -> Result<()> {
    match resp {
        Response::Ok => Ok(()),
        Response::Error { code, message } => {
            anyhow::bail!("Device error (code {code}): {message}");
        }
        other => {
            anyhow::bail!("Unexpected response: {other:?}");
        }
    }
}

/// Pretty-print an event to stdout.
pub fn print_event(event: &Event) {
    match event {
        Event::ScanResult {
            address,
            name,
            rssi,
            is_hid,
            ..
        } => {
            let hid_tag = if *is_hid {
                "[HID]".green().to_string()
            } else {
                "     ".to_string()
            };
            println!(
                "  {}  {:30}  RSSI: {:4}  {}",
                format_address(address).bold(),
                format!("\"{}\"", name),
                rssi,
                hid_tag
            );
        }
        Event::ConnectionState { state, .. } => {
            let label = state.label();
            let colored_label = match state {
                ConnectionState::Ready => label.green().to_string(),
                ConnectionState::Disconnected => label.red().to_string(),
                _ => label.yellow().to_string(),
            };
            println!("  State: {colored_label}");
        }
        Event::PairingStatus { status } => {
            let msg = match status {
                PairingState::Started => "Pairing started...".yellow().to_string(),
                PairingState::KeysExchanged => "Keys exchanged".yellow().to_string(),
                PairingState::Complete => "Pairing complete!".green().to_string(),
                PairingState::Failed => "Pairing FAILED".red().to_string(),
            };
            println!("  {msg}");
        }
        Event::Log { level, message } => {
            let level_str = log_level_name(*level);
            let colored = match level {
                0 => format!("[{level_str}]").dimmed().to_string(),
                1 => format!("[{level_str}]").blue().to_string(),
                2 => format!("[{level_str}]").yellow().to_string(),
                _ => format!("[{level_str}]").red().to_string(),
            };
            println!("  {colored} {message}");
        }
        Event::BondStored {
            address,
            profile_id,
        } => {
            let profile = profile_name(*profile_id);
            println!(
                "  {} Bond stored for {} (profile: {profile})",
                "OK".green(),
                format_address(address).bold()
            );
        }
        Event::BatteryLevel { level } => {
            println!("  Battery: {}%", level.to_string().bold());
        }
        Event::DefmtFrame { data } => {
            println!(
                "  [defmt] {} raw bytes (use defmt mode to decode)",
                data.len()
            );
        }
    }
}

/// Get profile name from ID.
pub fn profile_name(id: u8) -> &'static str {
    match id {
        0 => "Generic",
        1 => "MX Master 3S",
        2 => "Full Scroll Dial",
        3 => "Full Scroll Dial 16-bit",
        _ => "Unknown",
    }
}

/// Query all bonds from the device.
fn get_all_bonds(transport: &mut Transport) -> Vec<BondEntry> {
    let Ok((resp, _)) = transport.request_simple(CMD_GET_BONDS, DEFAULT_TIMEOUT) else {
        return Vec::new();
    };
    match resp {
        Response::Bonds { bonds } => bonds,
        _ => Vec::new(),
    }
}

// ---------------------------------------------------------------------------
// Device commands
// ---------------------------------------------------------------------------

pub fn cmd_status(transport: &mut Transport) -> Result<()> {
    let (resp, _events) = transport.request_simple(CMD_GET_STATUS, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Status {
            state,
            bonded_count,
            active_profile: _,
            active_device_set,
            active_device_address,
            battery_level,
            detected_os,
            connected_devices,
        } => {
            println!("  State:          {}", state.label().bold());
            println!("  Host OS:        {}", detected_os_name(detected_os).bold());
            println!("  Bonded devices: {bonded_count}");

            // Fetch bonds once for name lookups and auto-connect display
            let all_bonds = get_all_bonds(transport);

            if connected_devices.is_empty() {
                println!("  Connected:      {}", "None".dimmed());

                let battery_str = if battery_level == 0xFF {
                    "Unknown".dimmed().to_string()
                } else {
                    format!("{}%", battery_level).bold().to_string()
                };
                println!("  Battery:        {battery_str}");
            } else {
                println!(
                    "  Connected:      {}",
                    connected_devices.len().to_string().bold()
                );
                for (i, dev) in connected_devices.iter().enumerate() {
                    let bat = if dev.battery_level == 0xFF {
                        "?".dimmed().to_string()
                    } else {
                        format!("{}%", dev.battery_level)
                    };
                    let transport = if dev.transport_type == 1 {
                        "Classic"
                    } else {
                        "BLE"
                    };
                    // Look up device name from bonds
                    let name = all_bonds
                        .iter()
                        .find(|b| b.address == dev.address)
                        .map(|b| b.name.as_str())
                        .unwrap_or("");
                    if name.is_empty() {
                        println!(
                            "    {}. {}  {}  [{}]  battery: {}",
                            i + 1,
                            format_address(&dev.address).bold(),
                            profile_name(dev.profile_id),
                            transport,
                            bat,
                        );
                    } else {
                        println!(
                            "    {}. {}  {}  {}  [{}]  battery: {}",
                            i + 1,
                            format_address(&dev.address).bold(),
                            name,
                            profile_name(dev.profile_id).dimmed(),
                            transport,
                            bat,
                        );
                    }
                }
            }

            // Show auto-connect info from bonds (replaces legacy active device)
            let auto_bonds: Vec<&BondEntry> = all_bonds.iter().filter(|b| b.auto_connect).collect();
            if auto_bonds.is_empty() {
                println!(
                    "  Auto-connect:   {}",
                    "disabled (use set-auto-connect to enable)".dimmed()
                );
            } else {
                println!("  Auto-connect:");
                for bond in &auto_bonds {
                    let status = if connected_devices.iter().any(|d| d.address == bond.address) {
                        "connected".green().to_string()
                    } else {
                        "scanning...".yellow().to_string()
                    };
                    let display_name = if bond.name.is_empty() {
                        profile_name(bond.profile_id).to_string()
                    } else {
                        bond.name.clone()
                    };
                    println!(
                        "    {} {} [{}]",
                        format_address(&bond.address).bold(),
                        display_name,
                        status,
                    );
                }
            }

            if active_device_set {
                println!(
                    "  {}  {} {}",
                    "(legacy)".dimmed(),
                    format_address(&active_device_address),
                    "(use factory-reset to clear, then set-auto-connect)".dimmed(),
                );
            }
        }
        Response::Error { code, message } => {
            eprintln!("{} (code {code}): {message}", "Error".red());
        }
        other => {
            eprintln!("Unexpected response: {other:?}");
        }
    }
    Ok(())
}

pub fn cmd_connect(
    transport: &mut Transport,
    address: &str,
    addr_kind: u8,
    profile: Option<u8>,
    classic: bool,
) -> Result<()> {
    let addr = parse_address(address)?;
    let transport_type = if classic { 1u8 } else { 0u8 };

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_connect(&mut cbor_buf, &addr, addr_kind, transport_type)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let transport_label = if classic { "Classic" } else { "BLE" };
    println!(
        "Connecting to {} [{}]...",
        format_address(&addr).bold(),
        transport_label
    );

    if let Some(profile_id) = profile {
        let name = profile_name(profile_id);
        println!("  Profile: {name}");
    }

    let (resp, events) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    for evt in events {
        print_event(&evt);
    }

    let connect_timeout = Duration::from_secs(30);
    let mut pairing_complete = false;
    transport.stream_messages(connect_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
                match &evt {
                    Event::ConnectionState { state, .. } => {
                        matches!(
                            state,
                            ConnectionState::Connecting
                                | ConnectionState::Connected
                                | ConnectionState::Pairing
                                | ConnectionState::Scanning
                        )
                    }
                    Event::PairingStatus { status } => {
                        if matches!(status, PairingState::Complete) {
                            pairing_complete = true;
                        }
                        !matches!(status, PairingState::Complete | PairingState::Failed)
                    }
                    _ => true,
                }
            } else {
                true
            }
        } else {
            true
        }
    })?;

    if let Some(profile_id) = profile {
        if pairing_complete {
            println!("Setting profile...");
            let len = encode_request_update_bond_profile(
                &mut cbor_buf,
                &addr,
                profile_id,
                transport_type,
            )
            .map_err(|_| anyhow::anyhow!("encode failed"))?;
            let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
            check_ok(&resp)?;
            println!("{}", "Profile set successfully.".green());
        }
    }

    Ok(())
}

pub fn cmd_disconnect(transport: &mut Transport, address: Option<&str>) -> Result<()> {
    if let Some(addr_str) = address {
        let addr = parse_address(addr_str)?;
        println!("Disconnecting {}...", format_address(&addr).bold());
        let mut cbor_buf = [0u8; 32];
        let len = encode_request_disconnect(&mut cbor_buf, Some(&addr))
            .map_err(|_| anyhow::anyhow!("encode failed"))?;
        let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
        check_ok(&resp)?;
    } else {
        let (resp, _) = transport.request_simple(CMD_DISCONNECT, DEFAULT_TIMEOUT)?;
        check_ok(&resp)?;
    }
    println!("{}", "Disconnect command sent.".green());
    Ok(())
}

pub fn cmd_bonds(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_GET_BONDS, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Bonds { bonds } => {
            if bonds.is_empty() {
                println!("No bonded devices.");
            } else {
                println!("Bonded devices:");
                for (i, bond) in bonds.iter().enumerate() {
                    let auto = if bond.auto_connect {
                        " auto-connect".green().to_string()
                    } else {
                        String::new()
                    };
                    let name_display = if bond.name.is_empty() {
                        String::new()
                    } else {
                        format!("  {}", bond.name)
                    };
                    let transport = if bond.transport_type == 1 {
                        "Classic"
                    } else {
                        "BLE"
                    };
                    println!(
                        "  {}. {}{}  {}  [{}]{}",
                        i + 1,
                        format_address(&bond.address).bold(),
                        name_display,
                        profile_name(bond.profile_id),
                        transport,
                        auto,
                    );
                }
            }
        }
        Response::Error { code, message } => {
            eprintln!("{} (code {code}): {message}", "Error".red());
        }
        other => {
            eprintln!("Unexpected response: {other:?}");
        }
    }
    Ok(())
}

pub fn cmd_clear_bonds(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_CLEAR_BONDS, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "All bonds cleared.".green());
    println!(
        "{}",
        "Note: reboot the device to remove bonds from the BLE stack.".dimmed()
    );
    Ok(())
}

pub fn cmd_factory_reset(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_FACTORY_RESET, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!(
        "{}",
        "Factory reset complete (bonds and preferences cleared).".green()
    );
    println!("{}", "Device will restart.".dimmed());
    Ok(())
}

pub fn cmd_clear_bond(transport: &mut Transport, address: &str, classic: bool) -> Result<()> {
    let addr = parse_address(address)?;
    let transport_type = if classic { 1u8 } else { 0u8 };
    println!(
        "Clearing {} bond for {}...",
        if classic { "Classic" } else { "BLE" },
        format_address(&addr).bold()
    );

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_clear_bond(&mut cbor_buf, &addr, transport_type)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Bond cleared.".green());
    println!(
        "{}",
        "Note: reboot the device to remove the bond from the BLE stack.".dimmed()
    );
    Ok(())
}

pub fn cmd_set_profile(
    transport: &mut Transport,
    address: &str,
    profile_id: u8,
    classic: bool,
) -> Result<()> {
    let addr = parse_address(address)?;
    let transport_type = if classic { 1u8 } else { 0u8 };
    let profile = profile_name(profile_id);

    println!(
        "Setting {} profile for {} to {}...",
        if classic { "Classic" } else { "BLE" },
        format_address(&addr).bold(),
        profile.bold()
    );

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_update_bond_profile(&mut cbor_buf, &addr, profile_id, transport_type)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    println!("{}", "Profile updated successfully.".green());
    Ok(())
}

pub fn cmd_set_active_device(
    transport: &mut Transport,
    address: &str,
    addr_kind: u8,
) -> Result<()> {
    let addr = parse_address(address)?;

    println!(
        "Setting active device to {}...",
        format_address(&addr).bold()
    );

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_set_active_device(&mut cbor_buf, &addr, addr_kind)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    println!("{}", "Active device set.".green());
    Ok(())
}

pub fn cmd_clear_active_device(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_CLEAR_ACTIVE_DEVICE, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Active device cleared.".green());
    Ok(())
}

pub fn cmd_set_auto_connect(
    transport: &mut Transport,
    address: &str,
    enabled: bool,
    classic: bool,
) -> Result<()> {
    let addr = parse_address(address)?;
    let transport_type = if classic { 1u8 } else { 0u8 };

    println!(
        "{} {} auto-connect for {}...",
        if enabled { "Enabling" } else { "Disabling" },
        if classic { "Classic" } else { "BLE" },
        format_address(&addr).bold()
    );

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_set_auto_connect(&mut cbor_buf, &addr, enabled, transport_type)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    println!(
        "{}",
        if enabled {
            "Auto-connect enabled.".green()
        } else {
            "Auto-connect disabled.".green()
        }
    );
    Ok(())
}

pub fn cmd_auto_connect(transport: &mut Transport) -> Result<()> {
    println!("{}", "Auto-connecting to active device...".cyan());

    let (resp, events) = transport.request_simple(CMD_AUTO_CONNECT, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    for evt in events {
        print_event(&evt);
    }

    let connect_timeout = Duration::from_secs(30);
    transport.stream_messages(connect_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
                match &evt {
                    Event::ConnectionState { state, .. } => {
                        matches!(
                            state,
                            ConnectionState::Connecting
                                | ConnectionState::Connected
                                | ConnectionState::Pairing
                                | ConnectionState::Scanning
                        )
                    }
                    _ => true,
                }
            } else {
                true
            }
        } else {
            true
        }
    })?;

    Ok(())
}

pub fn cmd_get_config(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_GET_CONFIG, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Config {
            scroll_mult,
            pan_mult,
            x_mult,
            y_mult,
            scroll_threshold,
            max_detents,
            scroll_smoothing,
        } => {
            println!("Configuration:");
            println!("  scroll:        {}%", scroll_mult.to_string().bold());
            println!("  pan:           {}%", pan_mult.to_string().bold());
            println!("  x:             {}%", x_mult.to_string().bold());
            println!("  y:             {}%", y_mult.to_string().bold());
            println!("  threshold:     {}", scroll_threshold.to_string().bold());
            println!("  max_detents:   {}", max_detents.to_string().bold());
            let smoothing_label = if scroll_smoothing > 0 {
                "smooth"
            } else {
                "linear"
            };
            println!("  smoothing:     {}", smoothing_label.bold());
        }
        Response::Error { code, message } => {
            eprintln!("{} (code {code}): {message}", "Error".red());
        }
        other => {
            eprintln!("Unexpected response: {other:?}");
        }
    }
    Ok(())
}

pub fn cmd_set_config(transport: &mut Transport, key_name: &str, value: u32) -> Result<()> {
    let key = config_key_from_name(key_name).ok_or_else(|| {
        anyhow::anyhow!(
            "Unknown config key: {key_name}. Valid keys: scroll, pan, x, y, threshold, max_detents"
        )
    })?;

    let mut cbor_buf = [0u8; 16];
    let len = encode_request_set_config(&mut cbor_buf, key, value)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    let suffix = if key >= 4 { "" } else { "%" };
    println!("{} {} = {}{}", "Set".green(), key_name, value, suffix);
    Ok(())
}

pub fn cmd_version(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_GET_VERSION, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Version { version } => {
            println!("bt2usb firmware v{version}");
        }
        Response::Error { code, message } => {
            eprintln!("{} (code {code}): {message}", "Error".red());
        }
        other => {
            eprintln!("Unexpected response: {other:?}");
        }
    }
    Ok(())
}

pub fn cmd_restart(transport: &mut Transport) -> Result<()> {
    println!("{}", "Restarting device...".cyan());
    let (resp, _) = transport.request_simple(CMD_RESTART, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Device restart initiated.".green());
    Ok(())
}

pub fn cmd_reprobe(transport: &mut Transport) -> Result<()> {
    println!(
        "{}",
        "Force re-probing host OS (device will reset)...".cyan()
    );
    let (resp, _) = transport.request_simple(CMD_FORCE_REPROBE, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!(
        "{}",
        "Reprobe initiated. Device will re-detect host OS and present appropriate USB identity."
            .green()
    );
    Ok(())
}

pub fn cmd_set_os(transport: &mut Transport, os_name: &str) -> Result<()> {
    let os_val: u8 = match os_name.to_lowercase().as_str() {
        "auto" | "probe" | "0" => 0,
        "windows" | "win" | "1" => 1,
        "linux" | "2" => 2,
        "macos" | "mac" | "3" => 3,
        _ => {
            eprintln!(
                "{}: unknown OS '{}'. Valid: auto, windows, linux, macos",
                "Error".red(),
                os_name
            );
            std::process::exit(1);
        }
    };

    let os_label = match os_val {
        0 => "Auto (probe on each reset)",
        1 => "Windows",
        2 => "Linux",
        3 => "macOS",
        _ => unreachable!(),
    };

    let mut cbor_buf = [0u8; 16];
    let cbor_len = encode_request_set_forced_os(&mut cbor_buf, os_val)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;
    let (resp, _) = transport.request(&cbor_buf[..cbor_len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("Forced OS set to: {}", os_label.bold());
    if os_val > 0 {
        println!(
            "{}",
            "Subsequent soft resets will skip OS probe and use this identity.".dimmed()
        );
        println!(
            "{}",
            "Use 'set-os auto' to re-enable automatic detection.".dimmed()
        );
    } else {
        println!(
            "{}",
            "OS will be auto-detected on each boot/reset.".dimmed()
        );
    }
    Ok(())
}
