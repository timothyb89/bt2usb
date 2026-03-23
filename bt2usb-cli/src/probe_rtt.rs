//! RTT log reading via a debug probe.
//!
//! Reads raw defmt frames from the firmware's RTT channel using probe-rs,
//! with aggressive retry behavior for probe discovery and RTT attachment.
//! This captures logs from power-on, including before USB is initialized.

use std::thread;
use std::time::Duration;

use anyhow::Result;
use colored::Colorize;
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::Permissions;

use crate::defmt_decode::DefmtDecoder;

/// Polling interval when waiting for a probe or RTT control block.
const POLL_INTERVAL: Duration = Duration::from_millis(250);

/// How long to retry RTT attachment before giving up and re-probing.
const RTT_ATTACH_TIMEOUT: Duration = Duration::from_secs(5);

/// Interval between RTT read attempts when no data is available.
const RTT_READ_INTERVAL: Duration = Duration::from_millis(1);

/// Extract the `_SEGGER_RTT` symbol address from an ELF for fast RTT attachment.
/// Returns `None` if the ELF can't be parsed or the symbol isn't found.
pub fn extract_rtt_address(elf_bytes: &[u8]) -> Option<u64> {
    use object::{Object, ObjectSymbol};
    let file = object::File::parse(elf_bytes).ok()?;
    for symbol in file.symbols() {
        if symbol.name() == Ok("_SEGGER_RTT") {
            return Some(symbol.address());
        }
    }
    None
}

/// Stream defmt logs via RTT through a debug probe.
///
/// Polls for a probe, attaches to the RP2040, finds the RTT control block,
/// and reads raw defmt frames into the decoder. On any error (probe disconnect,
/// flash, reset), retries automatically.
pub fn cmd_logs_probe(
    mut decoder: DefmtDecoder,
    elf_bytes: &[u8],
    timeout_secs: u64,
) -> Result<()> {
    let rtt_addr = extract_rtt_address(elf_bytes);
    if let Some(addr) = rtt_addr {
        eprintln!(
            "{}",
            format!("RTT control block: 0x{addr:08x} (from ELF)").dimmed()
        );
    } else {
        eprintln!("{}", "RTT address not found in ELF; will scan RAM".dimmed());
    }

    let scan_region = match rtt_addr {
        Some(addr) => ScanRegion::Exact(addr),
        None => ScanRegion::Ram,
    };

    let deadline = if timeout_secs == 0 {
        None
    } else {
        Some(std::time::Instant::now() + Duration::from_secs(timeout_secs))
    };

    loop {
        if deadline.is_some_and(|d| std::time::Instant::now() > d) {
            return Ok(());
        }

        // 1. Wait for a debug probe
        let lister = Lister::new();
        let probes = lister.list_all();
        if probes.is_empty() {
            eprint!("{}", "Waiting for debug probe...".dimmed());
            loop {
                thread::sleep(POLL_INTERVAL);
                let probes = lister.list_all();
                if !probes.is_empty() {
                    eprintln!(" {}", "found".green());
                    break;
                }
            }
        }

        // 2. Open probe and attach to RP2040
        let probes = lister.list_all();
        let probe = match probes.into_iter().next() {
            Some(info) => match info.open() {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("{}", format!("Failed to open probe: {e}").yellow());
                    thread::sleep(POLL_INTERVAL);
                    continue;
                }
            },
            None => {
                thread::sleep(POLL_INTERVAL);
                continue;
            }
        };

        let mut session = match probe.attach("rp2040", Permissions::default()) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("{}", format!("Failed to attach to RP2040: {e}").yellow());
                thread::sleep(POLL_INTERVAL);
                continue;
            }
        };

        // 3. Attach RTT (retry until control block appears or timeout)
        let rtt = {
            let start = std::time::Instant::now();
            loop {
                match session.core(0) {
                    Ok(mut core) => match Rtt::attach_region(&mut core, &scan_region) {
                        Ok(rtt) => break Some(rtt),
                        Err(e) => {
                            if start.elapsed() > RTT_ATTACH_TIMEOUT {
                                eprintln!("{}", format!("RTT attach timeout: {e}").yellow());
                                break None;
                            }
                        }
                    },
                    Err(e) => {
                        eprintln!("{}", format!("Core access failed: {e}").yellow());
                        break None;
                    }
                }
                thread::sleep(POLL_INTERVAL);
            }
        };

        let Some(mut rtt) = rtt else {
            thread::sleep(POLL_INTERVAL);
            continue;
        };

        let Some(channel) = rtt.up_channel(0) else {
            eprintln!("{}", "RTT channel 0 not found".yellow());
            thread::sleep(POLL_INTERVAL);
            continue;
        };

        let channel_name = channel.name().unwrap_or("unnamed").to_string();
        eprintln!(
            "{}",
            format!("Streaming RTT logs (channel 0: \"{channel_name}\")... Press Ctrl+C to stop.")
                .dimmed()
        );

        // 4. Read loop
        let mut buf = [0u8; 1024];
        let read_err: anyhow::Error = loop {
            if deadline.is_some_and(|d| std::time::Instant::now() > d) {
                return Ok(());
            }

            let mut core = match session.core(0) {
                Ok(c) => c,
                Err(e) => break anyhow::Error::from(e),
            };

            let channel = match rtt.up_channel(0) {
                Some(ch) => ch,
                None => break anyhow::anyhow!("RTT channel lost"),
            };

            match channel.read(&mut core, &mut buf) {
                Ok(n) if n > 0 => {
                    for line in decoder.feed(&buf[..n]) {
                        println!("{line}");
                    }
                }
                Ok(_) => {
                    // No data available — brief pause to avoid busy-spinning
                    drop(core);
                    thread::sleep(RTT_READ_INTERVAL);
                }
                Err(e) => break anyhow::Error::from(e),
            }
        };

        eprintln!("{}", format!("Probe disconnected: {read_err}").yellow());
        eprint!("{}", "Waiting for debug probe...".dimmed());
        thread::sleep(POLL_INTERVAL);
    }
}
