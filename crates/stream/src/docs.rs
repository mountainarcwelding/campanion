//! Citations for the public Grbl v1.1 protocols this crate follows.
//!
//! Implementation is clean-room from these documents only:
//!
//! - Interface (streaming, responses, push messages, status reports, error and alarm codes):
//!   <https://github.com/gnea/grbl/blob/master/doc/markdown/interface.md>
//! - Commands (`$` system commands, `$X`, `$J=`, realtime `?` `!` `~` `0x18`):
//!   <https://github.com/gnea/grbl/blob/master/doc/markdown/commands.md>
//! - Jogging (`$J=` word rules, Idle/Jog only):
//!   <https://github.com/gnea/grbl/blob/master/doc/markdown/jogging.md>
//!
//! This crate does not invent a serial RX buffer size or a baud rate. Grbl's interface
//! document describes a 128-character receive buffer and the commands document describes
//! 115200 8-N-1 as the rate to use when talking to Grbl, but both values must be entered
//! or selected by the user (or send-response mode used, which needs no buffer size).

/// Real-time status query. Commands document: `?` is a single character, no CR/LF,
/// never placed in the RX buffer.
pub const REALTIME_STATUS: u8 = b'?';

/// Real-time feed hold. Commands document: `!`.
pub const REALTIME_HOLD: u8 = b'!';

/// Real-time cycle start / resume. Commands document: `~`.
pub const REALTIME_RESUME: u8 = b'~';

/// Real-time soft-reset. Commands document: `0x18` (ctrl-x).
pub const REALTIME_RESET: u8 = 0x18;

/// Line terminator this host sends. The interface document counts whatever line-feed
/// and carriage-return bytes are actually transmitted toward the RX buffer. This crate
/// sends a single LF and counts that one byte.
pub const LINE_ENDING: &[u8] = b"\n";

/// Official Grbl welcome-message prefix from the interface document.
pub const WELCOME_PREFIX: &str = "Grbl ";

/// Baud rate documented in the commands "Getting Started" section. Offered as a
/// selectable value only; never applied unless the user selects or types it.
pub const GRBL_DOCUMENTED_BAUD: u32 = 115_200;

/// RX buffer size described in the interface document's character-counting section.
/// Shown as documentation only; never used unless the user enters this (or another)
/// value on the machine profile.
pub const GRBL_DOCUMENTED_RX_BUFFER: usize = 128;
