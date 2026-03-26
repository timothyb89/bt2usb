//! ELF to UF2 conversion for RP2040.
//!
//! UF2 is a simple block-based format designed for flashing microcontrollers
//! via USB mass storage (BOOTSEL mode on the Pico). Each 512-byte block carries
//! up to 256 bytes of payload with a target flash address.

use anyhow::{bail, Result};
use object::elf::{self, ProgramHeader32, PT_LOAD};
use object::read::elf::{FileHeader, ProgramHeader};
use object::LittleEndian as LE;

/// RP2040 UF2 family ID.
const RP2040_FAMILY_ID: u32 = 0xE48BFF56;

/// RP2040 flash address range.
const FLASH_START: u32 = 0x1000_0000;
const FLASH_END: u32 = 0x1020_0000; // 2 MB

/// UF2 block constants.
const UF2_MAGIC_START0: u32 = 0x0A324655;
const UF2_MAGIC_START1: u32 = 0x9E5D5157;
const UF2_MAGIC_END: u32 = 0x0AB16F30;
const UF2_FLAG_FAMILY_ID: u32 = 0x0000_2000;
const UF2_PAYLOAD_SIZE: usize = 256;
const UF2_BLOCK_SIZE: usize = 512;

/// A contiguous region of data to be flashed.
struct FlashSegment {
    address: u32,
    data: Vec<u8>,
}

/// Convert an ELF binary to UF2 format for the RP2040.
pub fn elf_to_uf2(elf_data: &[u8]) -> Result<Vec<u8>> {
    let segments = extract_flash_segments(elf_data)?;

    if segments.is_empty() {
        bail!("No loadable segments found in flash range (0x{FLASH_START:08X}..0x{FLASH_END:08X})");
    }

    // Count total blocks needed
    let total_blocks: usize = segments
        .iter()
        .map(|s| s.data.len().div_ceil(UF2_PAYLOAD_SIZE))
        .sum();

    let mut uf2 = Vec::with_capacity(total_blocks * UF2_BLOCK_SIZE);
    let mut block_num: u32 = 0;

    for seg in &segments {
        for (chunk_idx, chunk) in seg.data.chunks(UF2_PAYLOAD_SIZE).enumerate() {
            let target_addr = seg.address + (chunk_idx * UF2_PAYLOAD_SIZE) as u32;
            let mut block = [0u8; UF2_BLOCK_SIZE];

            // Header
            block[0..4].copy_from_slice(&UF2_MAGIC_START0.to_le_bytes());
            block[4..8].copy_from_slice(&UF2_MAGIC_START1.to_le_bytes());
            block[8..12].copy_from_slice(&UF2_FLAG_FAMILY_ID.to_le_bytes());
            block[12..16].copy_from_slice(&target_addr.to_le_bytes());
            block[16..20].copy_from_slice(&(UF2_PAYLOAD_SIZE as u32).to_le_bytes());
            block[20..24].copy_from_slice(&block_num.to_le_bytes());
            block[24..28].copy_from_slice(&(total_blocks as u32).to_le_bytes());
            block[28..32].copy_from_slice(&RP2040_FAMILY_ID.to_le_bytes());

            // Payload (rest of the 476-byte data area stays zeroed)
            block[32..32 + chunk.len()].copy_from_slice(chunk);

            // Footer
            block[508..512].copy_from_slice(&UF2_MAGIC_END.to_le_bytes());

            uf2.extend_from_slice(&block);
            block_num += 1;
        }
    }

    Ok(uf2)
}

/// Extract PT_LOAD segments that fall within the RP2040 flash address range.
fn extract_flash_segments(data: &[u8]) -> Result<Vec<FlashSegment>> {
    // Parse as 32-bit little-endian ELF (RP2040 is Cortex-M0+)
    let header = elf::FileHeader32::<LE>::parse(data)
        .map_err(|e| anyhow::anyhow!("Failed to parse ELF header: {e}"))?;

    let endian = LE;
    let phdrs = header
        .program_headers(endian, data)
        .map_err(|e| anyhow::anyhow!("Failed to read program headers: {e}"))?;

    let mut segments = Vec::new();

    for phdr in phdrs.iter() {
        let phdr: &ProgramHeader32<LE> = phdr;
        if phdr.p_type(endian) != PT_LOAD {
            continue;
        }

        let paddr = phdr.p_paddr(endian);
        let filesz = phdr.p_filesz(endian) as usize;

        // Skip segments outside flash, or with no file data
        if !(FLASH_START..FLASH_END).contains(&paddr) || filesz == 0 {
            continue;
        }

        let offset = phdr.p_offset(endian) as usize;
        if offset + filesz > data.len() {
            bail!(
                "ELF segment at 0x{paddr:08X} extends beyond file (offset {offset}, size {filesz})"
            );
        }

        segments.push(FlashSegment {
            address: paddr,
            data: data[offset..offset + filesz].to_vec(),
        });
    }

    // Sort by address for deterministic output
    segments.sort_by_key(|s| s.address);
    Ok(segments)
}
