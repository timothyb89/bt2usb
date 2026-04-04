//! Interactive BLE scan with in-place device list, deduplication, and instant
//! device selection.

use std::collections::HashMap;
use std::io::{self, IsTerminal, Write};
use std::time::{Duration, Instant};

use anyhow::Result;
use colored::Colorize;
use crossterm::event::{self, Event as TermEvent, KeyCode, KeyModifiers};
use crossterm::terminal;

use crate::commands::{check_ok, cmd_connect, print_event, DEFAULT_TIMEOUT};
use crate::protocol::*;
use crate::transport::{format_address, Message, Transport};

// ---------------------------------------------------------------------------
// Device tracking
// ---------------------------------------------------------------------------

struct ScannedDevice {
    address: [u8; 6],
    addr_kind: u8,
    name: String,
    rssi: i8,
    is_hid: bool,
    transport_type: u8, // 0=BLE, 1=Classic
}

/// Insert or update a device. Returns `true` if anything changed.
fn upsert_device(
    devices: &mut HashMap<[u8; 6], ScannedDevice>,
    order: &mut Vec<[u8; 6]>,
    address: [u8; 6],
    addr_kind: u8,
    name: String,
    rssi: i8,
    is_hid: bool,
    transport_type: u8,
) -> bool {
    if let Some(dev) = devices.get_mut(&address) {
        let mut changed = false;
        if !name.is_empty() && (dev.name.is_empty() || dev.name != name) {
            dev.name = name;
            changed = true;
        }
        if dev.rssi != rssi {
            dev.rssi = rssi;
            changed = true;
        }
        if is_hid && !dev.is_hid {
            dev.is_hid = true;
            changed = true;
        }
        changed
    } else {
        order.push(address);
        devices.insert(
            address,
            ScannedDevice {
                address,
                addr_kind,
                name,
                rssi,
                is_hid,
                transport_type,
            },
        );
        true
    }
}

/// Process a single RPC message, upserting scan results.
/// Returns `true` if the device list changed.
fn process_message(
    msg: Message,
    devices: &mut HashMap<[u8; 6], ScannedDevice>,
    order: &mut Vec<[u8; 6]>,
) -> bool {
    if let Message::Event { cbor } = msg {
        if let Ok(Event::ScanResult {
            address,
            addr_kind,
            name,
            rssi,
            is_hid,
            transport_type,
        }) = decode_event(&cbor)
        {
            return upsert_device(
                devices,
                order,
                address,
                addr_kind,
                name,
                rssi,
                is_hid,
                transport_type,
            );
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

/// Run a BLE scan. In a terminal, shows an interactive TUI with instant device
/// selection. Otherwise falls back to streaming output with deduplication.
pub fn cmd_scan(transport: &mut Transport, timeout_secs: u64, profile: Option<u8>) -> Result<()> {
    cmd_scan_with(
        transport,
        timeout_secs,
        profile,
        CMD_START_SCAN,
        CMD_STOP_SCAN,
        false,
    )
}

/// Run a Classic BT Inquiry scan using the same interactive UI as BLE.
pub fn cmd_classic_scan(
    transport: &mut Transport,
    timeout_secs: u64,
    profile: Option<u8>,
) -> Result<()> {
    cmd_scan_with(
        transport,
        timeout_secs,
        profile,
        CMD_CLASSIC_SCAN,
        CMD_CLASSIC_SCAN_STOP,
        true,
    )
}

fn cmd_scan_with(
    transport: &mut Transport,
    timeout_secs: u64,
    profile: Option<u8>,
    start_cmd: u8,
    stop_cmd: u8,
    is_classic: bool,
) -> Result<()> {
    // Start scan on firmware
    let (resp, initial_events) = transport.request_simple(start_cmd, DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    let mut devices: HashMap<[u8; 6], ScannedDevice> = HashMap::new();
    let mut order: Vec<[u8; 6]> = Vec::new();

    for evt in initial_events {
        if let Event::ScanResult {
            address,
            addr_kind,
            name,
            rssi,
            is_hid,
            transport_type,
        } = evt
        {
            upsert_device(
                &mut devices,
                &mut order,
                address,
                addr_kind,
                name,
                rssi,
                is_hid,
                transport_type,
            );
        }
    }

    let result = if io::stdin().is_terminal() {
        run_interactive(
            transport,
            &mut devices,
            &mut order,
            timeout_secs,
            is_classic,
        )
    } else {
        run_streaming(transport, &mut devices, &mut order, timeout_secs).map(|()| None)
    };

    // Always stop scan, even on error
    let _ = transport.request_simple(stop_cmd, DEFAULT_TIMEOUT);

    let selected = result?;

    if let Some(idx) = selected {
        let addr = order[idx];
        let dev = &devices[&addr];
        let addr_str = format_address(&dev.address);
        let classic = dev.transport_type == 1;
        cmd_connect(transport, &addr_str, dev.addr_kind, profile, classic)?;
    } else {
        println!(
            "Scan complete. {} device(s) found.",
            order.len().to_string().bold()
        );
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Interactive mode (terminal)
// ---------------------------------------------------------------------------

/// RAII guard that restores the terminal on drop (including panics).
struct RawGuard;

impl Drop for RawGuard {
    fn drop(&mut self) {
        let _ = io::stdout().write_all(b"\x1b[?25h"); // show cursor
        let _ = io::stdout().flush();
        let _ = terminal::disable_raw_mode();
    }
}

fn run_interactive(
    transport: &mut Transport,
    devices: &mut HashMap<[u8; 6], ScannedDevice>,
    order: &mut Vec<[u8; 6]>,
    timeout_secs: u64,
    is_classic: bool,
) -> Result<Option<usize>> {
    terminal::enable_raw_mode()?;
    let _guard = RawGuard;

    let mut stdout = io::stdout();
    write!(stdout, "\x1b[?25l")?; // hide cursor

    let deadline = Instant::now() + Duration::from_secs(timeout_secs);
    let mut displayed_lines: u16 = 0;
    let mut last_secs = timeout_secs + 1; // force initial render
    let mut selected: Option<usize> = None;
    let mut free_form = String::new();
    let mut in_free_form = false;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let remaining_secs = remaining.as_secs();

        if remaining.is_zero() {
            displayed_lines = render(
                &mut stdout,
                displayed_lines,
                0,
                devices,
                order,
                in_free_form,
                is_classic,
                &free_form,
            )?;
            break;
        }

        // Redraw on second tick
        let mut needs_redraw = remaining_secs != last_secs;
        if needs_redraw {
            last_secs = remaining_secs;
        }

        // Drain all pending key events
        while event::poll(Duration::ZERO)? {
            if let TermEvent::Key(key) = event::read()? {
                // Ctrl+C — cancel immediately
                if key.modifiers.contains(KeyModifiers::CONTROL)
                    && matches!(key.code, KeyCode::Char('c'))
                {
                    // Render final state then bail
                    let _ = render(
                        &mut stdout,
                        displayed_lines,
                        0,
                        devices,
                        order,
                        false,
                        is_classic,
                        "",
                    );
                    write!(stdout, "\r\n{}\r\n", "Scan cancelled.".yellow())?;
                    stdout.flush()?;
                    return Ok(None);
                }

                if in_free_form {
                    match key.code {
                        KeyCode::Enter => {
                            if let Ok(n) = free_form.parse::<usize>() {
                                if n >= 1 && n <= order.len() {
                                    selected = Some(n - 1);
                                }
                            }
                            free_form.clear();
                            in_free_form = false;
                            needs_redraw = true;
                        }
                        KeyCode::Char(c) if c.is_ascii_digit() => {
                            free_form.push(c);
                            needs_redraw = true;
                        }
                        KeyCode::Backspace => {
                            free_form.pop();
                            if free_form.is_empty() {
                                in_free_form = false;
                            }
                            needs_redraw = true;
                        }
                        KeyCode::Esc => {
                            free_form.clear();
                            in_free_form = false;
                            needs_redraw = true;
                        }
                        _ => {}
                    }
                } else {
                    match key.code {
                        KeyCode::Char(c @ '1'..='9') => {
                            let n = (c as usize) - ('0' as usize);
                            if n <= order.len() {
                                selected = Some(n - 1);
                            }
                        }
                        KeyCode::Char('0') if order.len() > 9 => {
                            in_free_form = true;
                            free_form.clear();
                            needs_redraw = true;
                        }
                        _ => {}
                    }
                }
            }
        }

        if selected.is_some() {
            break;
        }

        // Poll HID (blocks up to 100 ms inside read_message)
        match transport.read_message() {
            Ok(Some(msg)) => {
                if process_message(msg, devices, order) {
                    needs_redraw = true;
                }
            }
            Ok(None) => {}
            Err(e) => return Err(e),
        }

        if needs_redraw {
            displayed_lines = render(
                &mut stdout,
                displayed_lines,
                remaining_secs,
                devices,
                order,
                in_free_form,
                is_classic,
                &free_form,
            )?;
        }
    }

    // Final clean render (no countdown)
    let _ = render(
        &mut stdout,
        displayed_lines,
        0,
        devices,
        order,
        false,
        is_classic,
        "",
    );
    // blank line to separate from subsequent output
    write!(stdout, "\r\n")?;
    stdout.flush()?;

    // _guard drop restores terminal

    Ok(selected)
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

fn render(
    stdout: &mut impl Write,
    prev_lines: u16,
    remaining_secs: u64,
    devices: &HashMap<[u8; 6], ScannedDevice>,
    order: &[[u8; 6]],
    in_free_form: bool,
    is_classic: bool,
    free_form: &str,
) -> io::Result<u16> {
    // Move cursor up to overwrite previous render
    if prev_lines > 0 {
        write!(stdout, "\x1b[{}A\r", prev_lines)?;
    }

    let mut lines: u16 = 0;

    // \x1b[2K = erase entire line (prevents leftover chars when new text is shorter)
    const CLR: &str = "\x1b[2K";

    // Header
    if remaining_secs > 0 {
        write!(
            stdout,
            "{CLR}{}\r\n",
            format!(
                "Scanning for {} devices...  {}s left{}",
                if is_classic { "Classic BT" } else { "BLE" },
                remaining_secs,
                if is_classic {
                    "  (names/RSSI unavailable)"
                } else {
                    ""
                }
            )
            .cyan()
        )?;
    } else {
        write!(stdout, "{CLR}{}\r\n", "Scan complete.".cyan())?;
    }
    lines += 1;

    // Blank separator
    write!(stdout, "{CLR}\r\n")?;
    lines += 1;

    // Device list
    if order.is_empty() {
        write!(stdout, "{CLR}  {}\r\n", "No devices found yet...".dimmed())?;
        lines += 1;
    } else {
        for (i, addr) in order.iter().enumerate() {
            let dev = &devices[addr];
            let num = i + 1;

            let name_display = if dev.name.is_empty() {
                let s = truncate_name("(unknown)", 28);
                format!("{:28}", s).dimmed().to_string()
            } else {
                let s = truncate_name(&dev.name, 28);
                format!("{:28}", s)
            };

            let rssi_str = format!("{:>4}", dev.rssi);
            let rssi_colored = if dev.rssi >= -50 {
                rssi_str.green()
            } else if dev.rssi >= -70 {
                rssi_str.yellow()
            } else {
                rssi_str.red()
            };

            let hid_tag = if dev.is_hid {
                " [HID]".green().to_string()
            } else {
                String::new()
            };
            let transport_tag = if dev.transport_type == 1 {
                " [Classic]".cyan().to_string()
            } else {
                " [BLE]".dimmed().to_string()
            };

            write!(
                stdout,
                "{CLR}  {:>2}  {}  {}  {} dBm{}{}\r\n",
                num.to_string().bold(),
                format_address(&dev.address).bold(),
                name_display,
                rssi_colored,
                hid_tag,
                transport_tag,
            )?;
            lines += 1;
        }
    }

    // Blank separator
    write!(stdout, "{CLR}\r\n")?;
    lines += 1;

    // Prompt line
    if in_free_form {
        write!(stdout, "{CLR}  Device #: {}{}\r\n", free_form, "▌".dimmed())?;
    } else if remaining_secs > 0 {
        let n = order.len();
        if n == 0 {
            write!(
                stdout,
                "{CLR}  {}\r\n",
                "Waiting for devices... Ctrl+C to cancel".dimmed()
            )?;
        } else if n <= 9 {
            write!(
                stdout,
                "{CLR}  Press {}{}{} to connect, {} to cancel\r\n",
                "1".bold(),
                if n > 1 { "-" } else { "" },
                if n > 1 {
                    n.to_string().bold().to_string()
                } else {
                    String::new()
                },
                "Ctrl+C".bold(),
            )?;
        } else {
            write!(
                stdout,
                "{CLR}  Press {}-{} or {} then number for 10+, {} to cancel\r\n",
                "1".bold(),
                "9".bold(),
                "0".bold(),
                "Ctrl+C".bold(),
            )?;
        }
    } else {
        // Scan finished — no prompt needed, but keep the line for layout stability
        write!(stdout, "{CLR}\r\n")?;
    }
    lines += 1;

    // Clear any leftover lines from a previous (taller) render
    write!(stdout, "\x1b[J")?;
    stdout.flush()?;

    Ok(lines)
}

fn truncate_name(name: &str, max_len: usize) -> String {
    let count = name.chars().count();
    if count > max_len {
        let truncated: String = name.chars().take(max_len - 1).collect();
        format!("{truncated}…")
    } else {
        name.to_string()
    }
}

// ---------------------------------------------------------------------------
// Non-interactive fallback (piped / non-TTY)
// ---------------------------------------------------------------------------

fn run_streaming(
    transport: &mut Transport,
    devices: &mut HashMap<[u8; 6], ScannedDevice>,
    order: &mut Vec<[u8; 6]>,
    timeout_secs: u64,
) -> Result<()> {
    println!("{}", "Scanning for BLE HID devices...".cyan());
    println!();

    // Print already-known devices
    for addr in order.iter() {
        print_device_line(&devices[addr]);
    }

    let scan_timeout = Duration::from_secs(timeout_secs);
    transport.stream_messages(scan_timeout, |msg| {
        if let Message::Event { cbor } = msg {
            if let Ok(Event::ScanResult {
                address,
                addr_kind,
                name,
                rssi,
                is_hid,
                transport_type,
            }) = decode_event(&cbor)
            {
                let is_new = !devices.contains_key(&address);
                upsert_device(
                    devices,
                    order,
                    address,
                    addr_kind,
                    name,
                    rssi,
                    is_hid,
                    transport_type,
                );
                if is_new {
                    print_device_line(&devices[&address]);
                }
            } else if let Ok(evt) = decode_event(&cbor) {
                print_event(&evt);
            }
        }
        true
    })?;

    println!();
    Ok(())
}

fn print_device_line(dev: &ScannedDevice) {
    let hid_tag = if dev.is_hid {
        "[HID]".green().to_string()
    } else {
        "     ".to_string()
    };
    let transport_tag = if dev.transport_type == 1 {
        "[Classic]".cyan().to_string()
    } else {
        "[BLE]".dimmed().to_string()
    };
    let name = if dev.name.is_empty() {
        "(unknown)".to_string()
    } else {
        format!("\"{}\"", dev.name)
    };
    println!(
        "  {}  {:30}  RSSI: {:4}  {} {}",
        format_address(&dev.address).bold(),
        name,
        dev.rssi,
        hid_tag,
        transport_tag,
    );
}
