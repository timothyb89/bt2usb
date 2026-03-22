//! defmt frame decoder using the firmware ELF's string table.
//!
//! Wraps `defmt-decoder`'s streaming decoder to provide a simple
//! feed-and-decode interface for raw defmt frames received over USB.
//!
//! The `Table` is leaked to obtain a `'static` reference — this is
//! appropriate since exactly one decoder is created per CLI invocation
//! and the memory is freed when the process exits.

use anyhow::{Context, Result};
use defmt_decoder::{DecodeError, Frame, StreamDecoder, Table};
use std::path::Path;

pub struct DefmtDecoder {
    decoder: Box<dyn StreamDecoder + 'static>,
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
        let decoder = table.new_stream_decoder();
        Ok(Self { decoder })
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
                    output.push(format_frame(&frame));
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

/// Format a decoded defmt frame with color and level prefix.
fn format_frame(frame: &Frame) -> String {
    // frame.display(true) includes timestamp, level, and colored message
    format!("{}", frame.display(true))
}
