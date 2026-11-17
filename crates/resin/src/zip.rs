//! A minimal zip reader and writer (stored and deflate entries, no zip64, no encryption).
//!
//! Written from the public ZIP file format description. Entry times are fixed so output is
//! deterministic.

use crate::error::{ResinError, Result};

const LOCAL_SIG: u32 = 0x0403_4b50;
const CENTRAL_SIG: u32 = 0x0201_4b50;
const END_SIG: u32 = 0x0605_4b50;
/// 1980-01-01 00:00:00 in DOS date and time.
const DOS_DATE: u16 = 0x0021;
/// Most entries we accept, and the largest single entry once inflated.
pub const MAX_ENTRIES: usize = 65_000;
pub const MAX_ENTRY_BYTES: usize = 256 * 1024 * 1024;

fn zip_err(message: impl Into<String>) -> ResinError {
    ResinError::Zip(message.into())
}

/// CRC-32 (IEEE) of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// A table-driven CRC for the larger layer images.
fn crc32_fast(data: &[u8]) -> u32 {
    let mut table = [0u32; 256];
    for (i, slot) in table.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xedb8_8320 } else { c >> 1 };
        }
        *slot = c;
    }
    let mut crc = !0u32;
    for &byte in data {
        let idx = ((crc ^ u32::from(byte)) & 0xff) as usize;
        crc = (crc >> 8) ^ table.get(idx).copied().unwrap_or(0);
    }
    !crc
}

/// Builds a zip in memory.
#[derive(Debug, Default)]
pub struct ZipWriter {
    out: Vec<u8>,
    central: Vec<u8>,
    entries: usize,
}

impl ZipWriter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one file. Deflate is used when it makes the entry smaller.
    pub fn add(&mut self, name: &str, data: &[u8]) -> Result<()> {
        if self.entries >= MAX_ENTRIES {
            return Err(zip_err("too many entries"));
        }
        if name.is_empty() || name.len() > 1000 || !name.is_ascii() {
            return Err(zip_err(format!("bad entry name `{name}`")));
        }
        let crc = crc32_fast(data);
        let deflated = miniz_oxide::deflate::compress_to_vec(data, 6);
        let (method, body): (u16, &[u8]) = if deflated.len() < data.len() { (8, &deflated) } else { (0, data) };
        let offset = u32::try_from(self.out.len()).map_err(|_| zip_err("archive over 4 GiB"))?;
        let csize = u32::try_from(body.len()).map_err(|_| zip_err("entry over 4 GiB"))?;
        let usize_ = u32::try_from(data.len()).map_err(|_| zip_err("entry over 4 GiB"))?;
        let name_len = name.len() as u16;

        self.out.extend_from_slice(&LOCAL_SIG.to_le_bytes());
        self.out.extend_from_slice(&20u16.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        self.out.extend_from_slice(&method.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        self.out.extend_from_slice(&DOS_DATE.to_le_bytes());
        self.out.extend_from_slice(&crc.to_le_bytes());
        self.out.extend_from_slice(&csize.to_le_bytes());
        self.out.extend_from_slice(&usize_.to_le_bytes());
        self.out.extend_from_slice(&name_len.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        self.out.extend_from_slice(name.as_bytes());
        self.out.extend_from_slice(body);

        let c = &mut self.central;
        c.extend_from_slice(&CENTRAL_SIG.to_le_bytes());
        c.extend_from_slice(&20u16.to_le_bytes());
        c.extend_from_slice(&20u16.to_le_bytes());
        c.extend_from_slice(&0u16.to_le_bytes());
        c.extend_from_slice(&method.to_le_bytes());
        c.extend_from_slice(&0u16.to_le_bytes());
        c.extend_from_slice(&DOS_DATE.to_le_bytes());
        c.extend_from_slice(&crc.to_le_bytes());
        c.extend_from_slice(&csize.to_le_bytes());
        c.extend_from_slice(&usize_.to_le_bytes());
        c.extend_from_slice(&name_len.to_le_bytes());
        c.extend_from_slice(&0u16.to_le_bytes()); // extra
        c.extend_from_slice(&0u16.to_le_bytes()); // comment
        c.extend_from_slice(&0u16.to_le_bytes()); // disk
        c.extend_from_slice(&0u16.to_le_bytes()); // internal attrs
        c.extend_from_slice(&0u32.to_le_bytes()); // external attrs
        c.extend_from_slice(&offset.to_le_bytes());
        c.extend_from_slice(name.as_bytes());
        self.entries += 1;
        Ok(())
    }

    pub fn finish(mut self) -> Result<Vec<u8>> {
        let cd_offset = u32::try_from(self.out.len()).map_err(|_| zip_err("archive over 4 GiB"))?;
        let cd_size = u32::try_from(self.central.len()).map_err(|_| zip_err("archive over 4 GiB"))?;
        let count = self.entries as u16;
        self.out.extend_from_slice(&self.central);
        self.out.extend_from_slice(&END_SIG.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        self.out.extend_from_slice(&count.to_le_bytes());
        self.out.extend_from_slice(&count.to_le_bytes());
        self.out.extend_from_slice(&cd_size.to_le_bytes());
        self.out.extend_from_slice(&cd_offset.to_le_bytes());
        self.out.extend_from_slice(&0u16.to_le_bytes());
        Ok(self.out)
    }
}

fn u16_at(data: &[u8], at: usize) -> Result<u16> {
    let bytes = data.get(at..at + 2).ok_or_else(|| zip_err("truncated"))?;
    Ok(u16::from_le_bytes([bytes.first().copied().unwrap_or(0), bytes.get(1).copied().unwrap_or(0)]))
}

fn u32_at(data: &[u8], at: usize) -> Result<u32> {
    let bytes = data.get(at..at + 4).ok_or_else(|| zip_err("truncated"))?;
    let mut buf = [0u8; 4];
    buf.copy_from_slice(bytes);
    Ok(u32::from_le_bytes(buf))
}

/// One inflated entry.
#[derive(Debug, Clone)]
pub struct ZipEntry {
    pub name: String,
    pub data: Vec<u8>,
}

/// Read every entry of a zip, in central directory order. CRCs are checked.
pub fn read_zip(data: &[u8]) -> Result<Vec<ZipEntry>> {
    let search_from = data.len().saturating_sub(22 + 65_535);
    let mut eocd = None;
    let mut at = data.len().saturating_sub(22);
    loop {
        if u32_at(data, at).is_ok_and(|sig| sig == END_SIG) {
            eocd = Some(at);
            break;
        }
        if at <= search_from {
            break;
        }
        at -= 1;
    }
    let eocd = eocd.ok_or_else(|| zip_err("no end-of-central-directory record"))?;
    let count = usize::from(u16_at(data, eocd + 10)?);
    let cd_offset = u32_at(data, eocd + 16)? as usize;
    if count > MAX_ENTRIES {
        return Err(zip_err("too many entries"));
    }
    let mut entries = Vec::with_capacity(count);
    let mut pos = cd_offset;
    for _ in 0..count {
        if u32_at(data, pos)? != CENTRAL_SIG {
            return Err(zip_err("bad central directory entry"));
        }
        let flags = u16_at(data, pos + 8)?;
        let method = u16_at(data, pos + 10)?;
        let crc = u32_at(data, pos + 16)?;
        let csize = u32_at(data, pos + 20)? as usize;
        let usize_ = u32_at(data, pos + 24)? as usize;
        let name_len = usize::from(u16_at(data, pos + 28)?);
        let extra_len = usize::from(u16_at(data, pos + 30)?);
        let comment_len = usize::from(u16_at(data, pos + 32)?);
        let local = u32_at(data, pos + 42)? as usize;
        let name_bytes = data.get(pos + 46..pos + 46 + name_len).ok_or_else(|| zip_err("truncated name"))?;
        let name = String::from_utf8_lossy(name_bytes).into_owned();
        pos += 46 + name_len + extra_len + comment_len;
        if flags & 1 != 0 {
            return Err(zip_err("encrypted entries are not supported"));
        }
        if usize_ > MAX_ENTRY_BYTES {
            return Err(zip_err(format!("entry `{name}` is too large")));
        }
        if u32_at(data, local)? != LOCAL_SIG {
            return Err(zip_err(format!("bad local header for `{name}`")));
        }
        let l_name = usize::from(u16_at(data, local + 26)?);
        let l_extra = usize::from(u16_at(data, local + 28)?);
        let start = local + 30 + l_name + l_extra;
        let raw = data.get(start..start + csize).ok_or_else(|| zip_err(format!("entry `{name}` is truncated")))?;
        let body = match method {
            0 => raw.to_vec(),
            8 => miniz_oxide::inflate::decompress_to_vec_with_limit(raw, MAX_ENTRY_BYTES)
                .map_err(|err| zip_err(format!("entry `{name}` does not inflate: {err:?}")))?,
            other => return Err(zip_err(format!("entry `{name}` uses unsupported method {other}"))),
        };
        if body.len() != usize_ {
            return Err(zip_err(format!("entry `{name}` has the wrong size")));
        }
        if crc32_fast(&body) != crc {
            return Err(zip_err(format!("entry `{name}` fails its CRC")));
        }
        entries.push(ZipEntry { name, data: body });
    }
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_value() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32_fast(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn round_trip() {
        let mut w = ZipWriter::new();
        w.add("a.txt", b"hello hello hello hello hello hello").unwrap();
        w.add("dir/b.bin", &[1, 2, 3]).unwrap();
        w.add("empty", &[]).unwrap();
        let bytes = w.finish().unwrap();
        let entries = read_zip(&bytes).unwrap();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].name, "a.txt");
        assert_eq!(entries[0].data, b"hello hello hello hello hello hello");
        assert_eq!(entries[1].data, vec![1, 2, 3]);
        assert!(entries[2].data.is_empty());
    }

    #[test]
    fn rejects_garbage_and_bad_crc() {
        assert!(read_zip(b"nope").is_err());
        let mut w = ZipWriter::new();
        w.add("x", b"abc").unwrap();
        let mut bytes = w.finish().unwrap();
        bytes[30 + 1] ^= 0xff; // flip the stored data
        assert!(read_zip(&bytes).is_err());
    }
}
