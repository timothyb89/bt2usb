//! Log streaming commands (defmt, text, and RTT probe modes).
//!
//! These commands manage their own connection lifecycle — they wait for a
//! device to appear, reconnect on disconnect, and stream until timeout or
//! Ctrl+C.

use anyhow::Result;
use colored::Colorize;
use std::time::Duration;

use crate::commands::{check_ok, print_event, DEFAULT_TIMEOUT};
use crate::protocol::*;
use crate::transport::{Message, Transport};
use crate::FIRMWARE_ELF;

/// Polling interval when waiting for the device to appear.
const POLL_INTERVAL: Duration = Duration::from_millis(250);

pub fn cmd_logs(
    transport: Option<Transport>,
    level: u8,
    timeout_secs: u64,
    elf_path: Option<&str>,
    text_mode: bool,
    probe_mode: bool,
) -> Result<()> {
    if text_mode {
        let mut t = wait_for_device(transport);
        return cmd_logs_text(&mut t, level, timeout_secs);
    }

    // Resolve ELF bytes for defmt decoding
    let (decoder, _elf_bytes) = resolve_defmt_decoder(elf_path);
    // _elf_bytes used by probe mode for RTT address extraction
    #[cfg(feature = "probe")]
    let elf_bytes = _elf_bytes;

    if probe_mode {
        #[cfg(feature = "probe")]
        {
            let decoder = match decoder {
                Some(d) => d,
                None => anyhow::bail!(
                    "--probe requires a defmt decoder. Use --elf <path> or build with `just build-all`."
                ),
            };
            return crate::probe_rtt::cmd_logs_probe(decoder, elf_bytes, timeout_secs);
        }
        #[cfg(not(feature = "probe"))]
        anyhow::bail!(
            "--probe requires the probe feature. Build with: cargo build --package bt2usb-cli --features probe"
        );
    }

    if let Some(decoder) = decoder {
        cmd_logs_defmt(transport, decoder, timeout_secs)
    } else {
        let mut t = wait_for_device(transport);
        cmd_logs_text(&mut t, level, timeout_secs)
    }
}

/// Try to create a defmt decoder from the provided ELF path or the embedded ELF.
/// Returns the decoder (if successful) and the raw ELF bytes (for RTT address extraction).
fn resolve_defmt_decoder(
    elf_path: Option<&str>,
) -> (Option<crate::defmt_decode::DefmtDecoder>, &'static [u8]) {
    if let Some(path) = elf_path {
        match crate::defmt_decode::DefmtDecoder::from_elf_path(std::path::Path::new(path)) {
            Ok(d) => {
                return (Some(d), FIRMWARE_ELF);
            }
            Err(e) => {
                eprintln!("Failed to load ELF '{}': {e:#}", path);
                return (None, FIRMWARE_ELF);
            }
        }
    }

    if !FIRMWARE_ELF.is_empty() {
        match crate::defmt_decode::DefmtDecoder::from_elf(FIRMWARE_ELF) {
            Ok(d) => return (Some(d), FIRMWARE_ELF),
            Err(e) => {
                eprintln!(
                    "Embedded ELF ({} bytes) failed to parse: {e:#}",
                    FIRMWARE_ELF.len()
                );
                return (None, FIRMWARE_ELF);
            }
        }
    }

    eprintln!("{}", "No firmware ELF embedded in this CLI build.".yellow());
    eprintln!("Use --elf <path> or --text, or rebuild with `just build-all`.");
    (None, FIRMWARE_ELF)
}

/// Block until a device is connected, returning a live transport.
/// If `existing` is already Some, returns it immediately.
fn wait_for_device(existing: Option<Transport>) -> Transport {
    if let Some(t) = existing {
        return t;
    }
    eprint!("{}", "Waiting for device...".dimmed());
    loop {
        if let Ok(t) = Transport::connect() {
            eprintln!(" {}", "connected".green());
            return t;
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

fn cmd_logs_defmt(
    mut transport: Option<Transport>,
    mut decoder: crate::defmt_decode::DefmtDecoder,
    timeout_secs: u64,
) -> Result<()> {
    let stream_timeout = if timeout_secs == 0 {
        Duration::from_secs(86400)
    } else {
        Duration::from_secs(timeout_secs)
    };

    let mut version_checked = false;

    loop {
        // Ensure we have a connection
        let t = match &mut transport {
            Some(t) => t,
            None => {
                loop {
                    match Transport::connect() {
                        Ok(new_t) => {
                            transport = Some(new_t);
                            eprintln!(" {}", "connected".green());
                            break;
                        }
                        Err(_) => std::thread::sleep(POLL_INTERVAL),
                    }
                }
                transport.as_mut().unwrap()
            }
        };

        // Version mismatch check (once per session)
        if !version_checked {
            version_checked = true;
            if let Ok((Response::Version { version: dev_ver }, _)) =
                t.request_simple(CMD_GET_VERSION, DEFAULT_TIMEOUT)
            {
                let cli_ver = env!("CARGO_PKG_VERSION");
                if dev_ver != cli_ver {
                    eprintln!(
                        "{}: firmware version ({dev_ver}) != CLI version ({cli_ver})",
                        "Warning".yellow()
                    );
                    eprintln!(
                        "{}",
                        "defmt frames may not decode correctly. Rebuild with: just build-all"
                            .dimmed()
                    );
                }
            }
        }

        // Subscribe to defmt stream
        let subscribe_result = (|| -> Result<()> {
            let mut cbor_buf = [0u8; 16];
            let len = encode_request_subscribe_defmt(&mut cbor_buf)
                .map_err(|_| anyhow::anyhow!("encode failed"))?;
            let (resp, _) = t.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
            check_ok(&resp)
        })();

        if subscribe_result.is_err() {
            transport = None;
            eprint!("{}", "Waiting for device...".dimmed());
            std::thread::sleep(POLL_INTERVAL);
            continue;
        }

        eprintln!(
            "{}",
            "Streaming defmt logs... Press Ctrl+C to stop.".dimmed()
        );

        let result = t.stream_messages(stream_timeout, |msg| {
            if let Message::Event { cbor } = msg {
                if let Ok(evt) = decode_event(&cbor) {
                    match evt {
                        Event::DefmtFrame { data } => {
                            for line in decoder.feed(&data) {
                                println!("{line}");
                            }
                        }
                        other => print_event(&other),
                    }
                }
            }
            true
        });

        // Try to unsubscribe (best-effort)
        let _ = t.request_simple(CMD_UNSUBSCRIBE_DEFMT, DEFAULT_TIMEOUT);

        match result {
            Ok(()) => return Ok(()),
            Err(_) => {
                transport = None;
                eprint!("{}", "Waiting for device...".dimmed());
                std::thread::sleep(POLL_INTERVAL);
            }
        }
    }
}

fn cmd_logs_text(transport: &mut Transport, level: u8, timeout_secs: u64) -> Result<()> {
    let mut cbor_buf = [0u8; 16];
    let len = encode_request_subscribe_logs(&mut cbor_buf, level)
        .map_err(|_| anyhow::anyhow!("encode failed"))?;
    let (resp, _) = transport.request(&cbor_buf[..len], DEFAULT_TIMEOUT)?;
    check_ok(&resp)?;

    let level_name = log_level_name(level);
    println!("Streaming text logs (level >= {level_name})... Press Ctrl+C to stop.");
    println!();

    let stream_timeout = if timeout_secs == 0 {
        Duration::from_secs(86400)
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

    let _ = transport.request_simple(CMD_UNSUBSCRIBE_LOGS, DEFAULT_TIMEOUT);

    result
}
