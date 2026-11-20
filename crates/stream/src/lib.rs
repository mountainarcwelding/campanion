//! Grbl-class streaming for CAMpanion.
//!
//! Clean-room implementation of the public Grbl v1.1 protocols cited in [`docs`].
//! This crate has not been hardware-tested. First run on a real machine: low power,
//! hand on the power switch.

#![cfg_attr(target_arch = "wasm32", allow(unused))]

pub mod docs;
pub mod frame;
pub mod mock;
pub mod protocol;
pub mod session;
pub mod streamer;

#[cfg(all(feature = "serial", unix, not(target_arch = "wasm32")))]
pub mod serial;

pub use docs::{GRBL_DOCUMENTED_BAUD, GRBL_DOCUMENTED_RX_BUFFER, REALTIME_HOLD, REALTIME_RESET, REALTIME_RESUME, REALTIME_STATUS};
pub use frame::{Bounds, frame_program, parse_feed};
pub use protocol::{MachineState, RxMessage, StatusReport, alarm_description, error_description, parse_line};
pub use session::{Session, SessionError, Transport, TransportError, list_ports, open_named, parse_baud};
pub use streamer::{HostPhase, JobKind, StreamError, StreamEvent, StreamMode, Streamer};

/// Parse a user-entered RX buffer size in characters. Empty means unset.
pub fn parse_rx_buffer(text: &str) -> Result<Option<usize>, StreamError> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }
    let value: usize = trimmed.parse().map_err(|_| StreamError::Message(format!("`{trimmed}` is not a serial RX buffer size in characters")))?;
    if value == 0 {
        return Err(StreamError::Message("serial RX buffer must be a positive number of characters".into()));
    }
    Ok(Some(value))
}

/// Build a stream mode from a user-entered buffer and a safe-mode flag.
///
/// Character-counting is used when `safe_mode` is false and a buffer was entered.
/// Otherwise send-response is used. Character-counting without a buffer is an error.
pub fn mode_from_profile(rx_buffer: Option<usize>, safe_mode: bool) -> Result<StreamMode, StreamError> {
    if safe_mode {
        return Ok(StreamMode::SendResponse);
    }
    match rx_buffer {
        Some(size) => StreamMode::character_counting(size),
        None => Err(StreamError::Message(Streamer::missing_buffer_message())),
    }
}
