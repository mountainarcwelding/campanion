//! The `.pw0Img` layer image codec.
//!
//! Derived from the sample export. A layer image is a sequence of big-endian 16-bit words in
//! row-major pixel order. The top 4 bits of a word are the gray value (0 off, 15 full exposure) and
//! the low 12 bits are the run length (1 to 4095). A longer run is split greedily into 4095-pixel
//! words, and runs continue across row ends. The decoded run lengths always add up to
//! `width * height`.

use crate::error::{ResinError, Result};

/// Longest run one word can carry.
pub const MAX_RUN: u32 = 0x0fff;
/// Highest gray value (4 bits).
pub const MAX_GRAY: u8 = 15;
/// Largest image we will decode or encode, in pixels.
pub const MAX_PIXELS: u64 = 1 << 28;

/// Builds an encoded layer image from runs, merging neighbours with the same value.
#[derive(Debug, Default)]
pub struct RunWriter {
    out: Vec<u8>,
    value: u8,
    pending: u64,
    total: u64,
}

impl RunWriter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append `count` pixels of gray `value` (0 to 15).
    pub fn push(&mut self, value: u8, count: u64) -> Result<()> {
        if value > MAX_GRAY {
            return Err(ResinError::Image(format!("gray value {value} is above {MAX_GRAY}")));
        }
        if count == 0 {
            return Ok(());
        }
        self.total = self.total.checked_add(count).ok_or_else(|| ResinError::Image("image too large".into()))?;
        if self.total > MAX_PIXELS {
            return Err(ResinError::Image("image too large".into()));
        }
        if self.pending > 0 && self.value != value {
            self.flush();
        }
        self.value = value;
        self.pending += count;
        Ok(())
    }

    fn flush(&mut self) {
        let mut left = self.pending;
        while left > 0 {
            let chunk = left.min(u64::from(MAX_RUN)) as u16;
            let word = (u16::from(self.value) << 12) | chunk;
            self.out.extend_from_slice(&word.to_be_bytes());
            left -= u64::from(chunk);
        }
        self.pending = 0;
    }

    /// Pixels pushed so far.
    pub fn total(&self) -> u64 {
        self.total
    }

    pub fn finish(mut self) -> Vec<u8> {
        self.flush();
        self.out
    }
}

/// Encode a dense row-major bitmap of gray values 0 to 15.
pub fn encode(pixels: &[u8]) -> Result<Vec<u8>> {
    let mut writer = RunWriter::new();
    let mut iter = pixels.iter().copied();
    let Some(mut value) = iter.next() else {
        return Ok(Vec::new());
    };
    let mut run = 1u64;
    for next in iter {
        if next == value {
            run += 1;
        } else {
            writer.push(value, run)?;
            value = next;
            run = 1;
        }
    }
    writer.push(value, run)?;
    Ok(writer.finish())
}

/// Decode to `(gray, length)` runs. Fails on an odd length, a zero-length run, or a pixel total
/// other than `expected_pixels`.
pub fn decode_runs(bytes: &[u8], expected_pixels: u64) -> Result<Vec<(u8, u32)>> {
    if expected_pixels > MAX_PIXELS {
        return Err(ResinError::Image("image too large".into()));
    }
    if !bytes.len().is_multiple_of(2) {
        return Err(ResinError::Image("odd byte count".into()));
    }
    let mut runs = Vec::with_capacity(bytes.len() / 2);
    let mut total = 0u64;
    for pair in bytes.chunks_exact(2) {
        let word = u16::from_be_bytes([pair.first().copied().unwrap_or(0), pair.get(1).copied().unwrap_or(0)]);
        let value = (word >> 12) as u8;
        let len = u32::from(word & 0x0fff);
        if len == 0 {
            return Err(ResinError::Image("zero-length run".into()));
        }
        total += u64::from(len);
        if total > expected_pixels {
            return Err(ResinError::Image("runs are longer than the image".into()));
        }
        runs.push((value, len));
    }
    if total != expected_pixels {
        return Err(ResinError::Image(format!("runs cover {total} pixels, expected {expected_pixels}")));
    }
    Ok(runs)
}

/// Decode to a dense row-major bitmap of gray values 0 to 15.
pub fn decode(bytes: &[u8], width: u32, height: u32) -> Result<Vec<u8>> {
    let pixels = u64::from(width) * u64::from(height);
    let runs = decode_runs(bytes, pixels)?;
    let mut out = Vec::with_capacity(pixels as usize);
    for (value, len) in runs {
        out.resize(out.len() + len as usize, value);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn long_runs_split_at_4095() {
        let bytes = encode(&vec![0u8; 5000]).unwrap_or_default();
        assert_eq!(bytes, vec![0x0f, 0xff, 0x03, 0x89]);
        assert_eq!(decode(&bytes, 5000, 1).unwrap_or_default(), vec![0u8; 5000]);
    }

    #[test]
    fn gray_and_run_fields() {
        let mut px = vec![0u8; 3];
        px.extend([15u8; 2]);
        assert_eq!(encode(&px).unwrap_or_default(), vec![0x00, 0x03, 0xf0, 0x02]);
    }

    #[test]
    fn rejects_bad_input() {
        assert!(decode(&[0x00], 1, 1).is_err());
        assert!(decode(&[0x00, 0x00], 1, 1).is_err());
        assert!(decode(&[0x00, 0x02], 1, 1).is_err());
        assert!(decode(&[0x00, 0x01], 2, 1).is_err());
        assert!(encode(&[16]).is_err());
    }

    #[test]
    fn round_trip_patterns() {
        let mut px = Vec::new();
        for i in 0..20_000u32 {
            px.push(if (i / 37) % 3 == 0 { 15 } else { (i % 4) as u8 });
        }
        let bytes = encode(&px).unwrap_or_default();
        assert_eq!(decode(&bytes, 20_000, 1).unwrap_or_default(), px);
    }

    #[test]
    fn run_writer_merges_neighbours() {
        let mut w = RunWriter::new();
        let _ = w.push(0, 3000);
        let _ = w.push(0, 3000);
        let _ = w.push(15, 1);
        assert_eq!(w.finish(), vec![0x0f, 0xff, 0x07, 0x71, 0xf0, 0x01]);
    }
}
