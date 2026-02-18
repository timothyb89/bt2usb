//! bt2usb-cli - Command-line interface for the bt2usb BLE-to-USB bridge
//!
//! Communicates with the bt2usb device over a vendor HID interface
//! using COBS-framed CBOR messages.

mod protocol;
mod transport;

use anyhow::Result;
use clap::{Parser, Subcommand};
use colored::Colorize;
use std::time::Duration;

use protocol::*;
use transport::{format_address, parse_address, Message, Transport};

/// Default request timeout
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(3);

#[derive(Parser)]
#[command(name = "bt2usb-cli", about = "Control the bt2usb BLE-to-USB bridge")]
struct Cli {
    /// HID device path to use (auto-detected if not specified)
    #[arg(short, long)]
    device: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Show device status
    Status,

    /// Scan for BLE HID devices
    Scan {
        /// Scan duration in seconds
        #[arg(short, long, default_value = "15")]
        timeout: u64,
    },

    /// Connect to a BLE device by address
    Connect {
        /// BLE address (AA:BB:CC:DD:EE:FF)
        address: String,

        /// Address type: 0=public, 1=random
        #[arg(short = 'k', long, default_value = "1")]
        addr_kind: u8,

        /// Device profile: 0=generic, 1=MxMaster3S, 2=FullScrollDial, 3=FullScrollDial16Bit
        #[arg(short, long)]
        profile: Option<u8>,
    },

    /// Disconnect the current device
    Disconnect,

    /// List bonded devices
    Bonds,

    /// Clear all bonds
    ClearBonds,

    /// Set the profile for a bonded device
    SetProfile {
        /// BLE address (AA:BB:CC:DD:EE:FF)
        address: String,

        /// Profile ID: 0=generic, 1=MxMaster3S, 2=FullScrollDial, 3=FullScrollDial16Bit
        profile_id: u8,
    },

    /// Set the active device for auto-reconnect
    SetActiveDevice {
        /// BLE address (AA:BB:CC:DD:EE:FF)
        address: String,

        /// Address type: 0=public, 1=random
        #[arg(short = 'k', long, default_value = "1")]
        addr_kind: u8,
    },

    /// Clear the active device (disable auto-reconnect)
    ClearActiveDevice,

    /// Auto-connect to the active device from preferences
    AutoConnect,

    /// Stream live logs from device
    Logs {
        /// Minimum log level: 0=debug, 1=info, 2=warn, 3=error
        #[arg(short, long, default_value = "1")]
        level: u8,

        /// Duration to stream logs (seconds, 0=forever)
        #[arg(short, long, default_value = "0")]
        timeout: u64,
    },

    /// Show firmware version
    Version,

    /// Restart the device
    Restart,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let mut transport = if let Some(device) = &cli.device {
        Transport::connect_to(device)?
    } else {
        Transport::connect()?
    };

    match cli.command {
        Command::Status => cmd_status(&mut transport),
        Command::Scan { timeout } => cmd_scan(&mut transport, timeout),
        Command::Connect { address, addr_kind, profile } => cmd_connect(&mut transport, &address, addr_kind, profile),
        Command::Disconnect => cmd_disconnect(&mut transport),
        Command::Bonds => cmd_bonds(&mut transport),
        Command::ClearBonds => cmd_clear_bonds(&mut transport),
        Command::SetProfile { address, profile_id } => cmd_set_profile(&mut transport, &address, profile_id),
        Command::SetActiveDevice { address, addr_kind } => cmd_set_active_device(&mut transport, &address, addr_kind),
        Command::ClearActiveDevice => cmd_clear_active_device(&mut transport),
        Command::AutoConnect => cmd_auto_connect(&mut transport),
        Command::Logs { level, timeout } => cmd_logs(&mut transport, level, timeout),
        Command::Version => cmd_version(&mut transport),
        Command::Restart => cmd_restart(&mut transport),
    }
}

fn cmd_status(transport: &mut Transport) -> Result<()> {
    let (resp, _events) = transport.request_simple(CMD_GET_STATUS, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Status {
            state,
            bonded_count,
            active_profile,
            active_device_set,
            active_device_address,
            battery_level,
        } => {
            println!("  State:          {}", state.label().bold());
            println!("  Bonded devices: {bonded_count}");
            println!("  Active profile: {} ({})", active_profile, profile_name(active_profile));

            if active_device_set {
                println!(
                    "  Active device:  {} {}",
                    format_address(&active_device_address).bold(),
                    "(auto-connect enabled)".green()
                );
            } else {
                println!("  Active device:  {} (auto-connect disabled)", "None".dimmed());
            }

            let battery_str = if battery_level == 0xFF {
                "Unknown".dimmed().to_string()
            } else {
                format!("{}%", battery_level).bold().to_string()
            };
            println!("  Battery:        {battery_str}");
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

fn cmd_scan(transport: &mut Transport, timeout_secs: u64) -> Result<()> {
    // Start scan
    let (resp, events) = transport.request_simple(CMD_START_SCAN, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    println!("{}", "Scanning for BLE HID devices...".cyan());
    println!();

    // Print any events that came with the response
    for evt in events {
        print_event(&evt);
    }

    // Stream events for the specified duration
    let scan_timeout = Duration::from_secs(timeout_secs);
    let mut device_count = 0u32;

    transport.stream_messages(scan_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                match &evt {
                    Event::ScanResult { .. } => {
                        device_count += 1;
                        print_event(&evt);
                    }
                    Event::ConnectionState { state, .. } => {
                        // Scan may have been stopped by firmware
                        if matches!(state, self::ConnectionState::Disconnected) {
                            return false;
                        }
                        print_event(&evt);
                    }
                    _ => print_event(&evt),
                }
            }
        }
        true
    })?;

    // Stop scan
    let _ = transport.request_simple(CMD_STOP_SCAN, DEFAULT_TIMEOUT);

    println!();
    println!(
        "Scan complete. {} device(s) found.",
        device_count.to_string().bold()
    );

    Ok(())
}

fn cmd_connect(transport: &mut Transport, address: &str, addr_kind: u8, profile: Option<u8>) -> Result<()> {
    let addr = parse_address(address)?;

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_connect(&mut cbor_buf, &addr, addr_kind)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    println!("Connecting to {}...", format_address(&addr).bold());

    if let Some(profile_id) = profile {
        let profile_name = profile_name(profile_id);
        println!("  Profile: {profile_name}");
    }

    let (resp, events) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    for evt in events {
        print_event(&evt);
    }

    // Stream events until we see Ready or an error
    let connect_timeout = Duration::from_secs(30);
    let mut pairing_complete = false;
    transport.stream_messages(connect_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
                match &evt {
                    Event::ConnectionState { state, .. } => {
                        matches!(state, self::ConnectionState::Connecting | self::ConnectionState::Connected | self::ConnectionState::Pairing | self::ConnectionState::Scanning)
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

    // If profile specified and pairing completed, update the bond profile
    if let Some(profile_id) = profile {
        if pairing_complete {
            println!("Setting profile...");
            let len = encode_request_update_bond_profile(&mut cbor_buf, &addr, profile_id)
                .map_err(|_| anyhow::anyhow!("encode failed"))?;
            let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
            check_ok(&resp)?;
            println!("{}", "Profile set successfully.".green());
        }
    }

    Ok(())
}

fn cmd_disconnect(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_DISCONNECT, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Disconnect command sent.".green());
    Ok(())
}

fn cmd_bonds(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_GET_BONDS, DEFAULT_TIMEOUT)?;
    match resp {
        Response::Bonds { bonds } => {
            if bonds.is_empty() {
                println!("No bonded devices.");
            } else {
                println!("Bonded devices:");
                for (i, bond) in bonds.iter().enumerate() {
                    println!(
                        "  {}. {}  \"{}\"  profile={}  kind={}",
                        i + 1,
                        format_address(&bond.address).bold(),
                        bond.name,
                        bond.profile_id,
                        if bond.addr_kind == 1 { "random" } else { "public" }
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

fn cmd_clear_bonds(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_CLEAR_BONDS, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "All bonds cleared.".green());
    Ok(())
}

fn cmd_logs(transport: &mut Transport, level: u8, timeout_secs: u64) -> Result<()> {
    // Subscribe to logs
    let mut cbor_buf = [0u8; 16];
    let len = encode_request_subscribe_logs(&mut cbor_buf, level)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;
    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    let level_name = log_level_name(level);
    println!("Streaming logs (level >= {level_name})... Press Ctrl+C to stop.");
    println!();

    let stream_timeout = if timeout_secs == 0 {
        Duration::from_secs(86400) // ~24 hours as "forever"
    } else {
        Duration::from_secs(timeout_secs)
    };

    let result = transport.stream_messages(stream_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
            }
        }
        true
    });

    // Unsubscribe
    let _ = transport.request_simple(CMD_UNSUBSCRIBE_LOGS, DEFAULT_TIMEOUT);

    result
}

fn cmd_version(transport: &mut Transport) -> Result<()> {
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

fn cmd_restart(transport: &mut Transport) -> Result<()> {
    println!("{}", "Restarting device...".cyan());
    let (resp, _) = transport.request_simple(CMD_RESTART, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Device restart initiated.".green());
    Ok(())
}

/// Check if a response is Ok, print error if not.
fn check_ok(resp: &Response) -> Result<()> {
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
fn print_event(event: &Event) {
    match event {
        Event::ScanResult { address, name, rssi, is_hid, .. } => {
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
                self::ConnectionState::Ready => label.green().to_string(),
                self::ConnectionState::Disconnected => label.red().to_string(),
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
        Event::BondStored { address, profile_id } => {
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
    }
}

fn cmd_set_profile(transport: &mut Transport, address: &str, profile_id: u8) -> Result<()> {
    let addr = parse_address(address)?;
    let profile = profile_name(profile_id);

    println!(
        "Setting profile for {} to {}...",
        format_address(&addr).bold(),
        profile.bold()
    );

    let mut cbor_buf = [0u8; 32];
    let len = encode_request_update_bond_profile(&mut cbor_buf, &addr, profile_id)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;

    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    println!("{}", "Profile updated successfully.".green());
    Ok(())
}

fn cmd_set_active_device(transport: &mut Transport, address: &str, addr_kind: u8) -> Result<()> {
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

fn cmd_clear_active_device(transport: &mut Transport) -> Result<()> {
    let (resp, _) = transport.request_simple(CMD_CLEAR_ACTIVE_DEVICE, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;
    println!("{}", "Active device cleared.".green());
    Ok(())
}

fn cmd_auto_connect(transport: &mut Transport) -> Result<()> {
    println!("{}", "Auto-connecting to active device...".cyan());

    let (resp, events) = transport.request_simple(CMD_AUTO_CONNECT, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    for evt in events {
        print_event(&evt);
    }

    // Stream events until connected
    let connect_timeout = Duration::from_secs(30);
    transport.stream_messages(connect_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
                match &evt {
                    Event::ConnectionState { state, .. } => {
                        matches!(state, self::ConnectionState::Connecting | self::ConnectionState::Connected | self::ConnectionState::Pairing | self::ConnectionState::Scanning)
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

/// Get profile name from ID
fn profile_name(id: u8) -> &'static str {
    match id {
        0 => "Generic",
        1 => "MX Master 3S",
        2 => "Full Scroll Dial",
        3 => "Full Scroll Dial 16-bit",
        _ => "Unknown",
    }
}
