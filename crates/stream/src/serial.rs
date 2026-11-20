//! Native Unix serial-port transport via the MIT-licensed `nix` crate.
//!
//! Compiled only with the `serial` feature on Unix, never on wasm. This avoids the
//! MPL-2.0 `serialport` crate, which cargo-deny rejects.

#![cfg(all(feature = "serial", unix, not(target_arch = "wasm32")))]

use std::fs::{File, OpenOptions, read_dir};
use std::io::{Read, Write};
use std::os::fd::AsFd;
use std::os::unix::fs::OpenOptionsExt;

use nix::fcntl::OFlag;
use nix::sys::termios::{
    BaudRate, ControlFlags, InputFlags, LocalFlags, OutputFlags, SetArg, SpecialCharacterIndices, cfsetispeed, cfsetospeed, tcgetattr, tcsetattr,
};

use crate::session::{Transport, TransportError};

/// Names of USB and onboard serial devices under `/dev`.
pub fn available_port_names() -> Vec<String> {
    let mut names = Vec::new();
    if let Ok(entries) = read_dir("/dev/serial/by-id") {
        for entry in entries.flatten() {
            names.push(entry.path().to_string_lossy().into_owned());
        }
    }
    if let Ok(entries) = read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("ttyUSB") || name.starts_with("ttyACM") || name.starts_with("ttyAMA") || name.starts_with("cu.") {
                names.push(format!("/dev/{name}"));
            }
        }
    }
    names.sort();
    names.dedup();
    names
}

/// Open `path` at the user-selected `baud`, 8-N-1, as the Grbl commands document describes.
pub fn open_port(path: &str, baud: u32) -> Result<SerialTransport, TransportError> {
    let baud_rate = map_baud(baud)?;
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags((OFlag::O_NOCTTY | OFlag::O_NONBLOCK).bits())
        .open(path)
        .map_err(|err| TransportError::Message(format!("could not open {path}: {err}")))?;
    configure_termios(&file, baud_rate).map_err(|err| TransportError::Message(format!("could not configure {path}: {err}")))?;
    Ok(SerialTransport { file, name: path.to_string() })
}

fn map_baud(baud: u32) -> Result<BaudRate, TransportError> {
    // These are POSIX/termios rates, not machine defaults. Grbl documents 115200; it is
    // offered only when the user selects or types it.
    match baud {
        9600 => Ok(BaudRate::B9600),
        19200 => Ok(BaudRate::B19200),
        38400 => Ok(BaudRate::B38400),
        57600 => Ok(BaudRate::B57600),
        115200 => Ok(BaudRate::B115200),
        230400 => Ok(BaudRate::B230400),
        _ => Err(TransportError::Message(format!(
            "baud {baud} is not a POSIX termios rate this build can set; use 9600, 19200, 38400, 57600, 115200, or 230400"
        ))),
    }
}

fn configure_termios(file: &File, baud: BaudRate) -> Result<(), String> {
    let fd = file.as_fd();
    let mut term = tcgetattr(fd).map_err(|err| err.to_string())?;
    term.input_flags = InputFlags::empty();
    term.output_flags = OutputFlags::empty();
    term.local_flags = LocalFlags::empty();
    term.control_flags = ControlFlags::CS8 | ControlFlags::CREAD | ControlFlags::CLOCAL;
    cfsetispeed(&mut term, baud).map_err(|err| err.to_string())?;
    cfsetospeed(&mut term, baud).map_err(|err| err.to_string())?;
    // Non-canonical, non-blocking reads: return immediately with whatever is available.
    term.control_chars[SpecialCharacterIndices::VMIN as usize] = 0;
    term.control_chars[SpecialCharacterIndices::VTIME as usize] = 0;
    tcsetattr(fd, SetArg::TCSANOW, &term).map_err(|err| err.to_string())?;
    Ok(())
}

pub struct SerialTransport {
    file: File,
    name: String,
}

impl SerialTransport {
    fn map_err(err: std::io::Error) -> TransportError {
        match err.kind() {
            std::io::ErrorKind::BrokenPipe | std::io::ErrorKind::NotConnected => TransportError::Disconnected,
            _ => TransportError::Message(err.to_string()),
        }
    }
}

impl Transport for SerialTransport {
    fn write(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        let mut written = 0;
        while written < bytes.len() {
            match self.file.write(&bytes[written..]) {
                Ok(0) => return Err(TransportError::Disconnected),
                Ok(n) => written += n,
                Err(err) if err.kind() == std::io::ErrorKind::TimedOut || err.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(err) => return Err(Self::map_err(err)),
            }
        }
        let _ = self.file.flush();
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        match self.file.read(buf) {
            Ok(n) => Ok(n),
            Err(err) if err.kind() == std::io::ErrorKind::TimedOut || err.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(err) => Err(Self::map_err(err)),
        }
    }

    fn close(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}
