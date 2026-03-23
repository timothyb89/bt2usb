//! defmt frame decoder using the firmware ELF's string table.
//!
//! Wraps `defmt-decoder`'s streaming decoder to provide a simple
//! feed-and-decode interface for raw defmt frames received over USB.
//!
//! The `Table` is leaked to obtain a `'static` reference — this is
//! appropriate since exactly one decoder is created per CLI invocation
//! and the memory is freed when the process exits.

use anyhow::{Context, Result};
use colored::Colorize;
use defmt_decoder::{DecodeError, Frame, Locations, StreamDecoder, Table};
use std::path::Path;

pub struct DefmtDecoder {
    decoder: Box<dyn StreamDecoder + 'static>,
    locations: Option<Locations>,
}

impl DefmtDecoder {
    /// Create a decoder from ELF bytes (e.g. embedded via `include_bytes!`).
    pub fn from_elf(elf_bytes: &[u8]) -> Result<Self> {
        let table = Table::parse(elf_bytes)
            .context("failed to parse ELF")?
            .context("ELF has no .defmt section (was it built with defmt?)")?;
        // Leak the table to get a 'static reference for the StreamDecoder.
        // One allocation per CLI invocation; freed on process exit.
        let table: &'static Table = Box::leak(Box::new(table));

        // Extract source locations (file:line for each log callsite)
        let locations = table.get_locations(elf_bytes).ok();

        let decoder = table.new_stream_decoder();
        Ok(Self { decoder, locations })
    }

    /// Create a decoder from an ELF file on disk.
    pub fn from_elf_path(path: &Path) -> Result<Self> {
        let elf_bytes = std::fs::read(path)
            .with_context(|| format!("failed to read ELF: {}", path.display()))?;
        Self::from_elf(&elf_bytes)
    }

    /// Feed raw defmt bytes and return any fully decoded frames.
    pub fn feed(&mut self, data: &[u8]) -> Vec<String> {
        self.decoder.received(data);
        let mut output = Vec::new();
        loop {
            match self.decoder.decode() {
                Ok(frame) => {
                    output.push(format_frame(&frame, &self.locations));
                }
                Err(DecodeError::UnexpectedEof) => break,
                Err(DecodeError::Malformed) => {
                    output.push("Warning: malformed defmt frame".to_string());
                    break;
                }
            }
        }
        output
    }
}

/// Format a decoded defmt frame with color, level prefix, and source location.
fn format_frame(frame: &Frame, locations: &Option<Locations>) -> String {
    let msg = format!("{}", frame.display(true));

    // Append source location if available (dimmed, like probe-rs)
    if let Some(locations) = locations {
        if let Some(loc) = locations.get(&frame.index()) {
            let file = loc.file.to_string_lossy();
            let line = loc.line;
            let short_file = shorten_path(&file);
            return format!("{msg} {}", format!("({short_file}:{line})").dimmed());
        }
    }
    msg
}

/// Shorten a source file path for display.
/// Strips cargo registry paths to show `crate_name-version/src/file.rs`.
fn shorten_path(path: &str) -> &str {
    // Cargo registry: .../registry/src/index.crates.io-.../crate-version/src/file.rs
    // → crate-version/src/file.rs
    if let Some(pos) = path.find("/registry/src/") {
        if let Some(crate_start) = path[pos..].find('/').map(|i| pos + i + 1) {
            if let Some(next_slash) = path[crate_start..].find('/').map(|i| crate_start + i + 1) {
                return &path[next_slash..];
            }
        }
    }
    // Local paths: strip everything before src/
    if let Some(pos) = path.rfind("/src/") {
        // Include the parent dir for context: "module/src/file.rs"
        if let Some(parent_start) = path[..pos].rfind('/') {
            return &path[parent_start + 1..];
        }
    }
    path
}
