//! Machine-neutral toolpath intermediate representation.
//!
//! A [`Document`] is JSON tagged [`FORMAT_TAG`]. It holds two levels:
//! - **Tool space** (cutter location): tool tip position and tool axis direction in the part frame.
//! - **Axis space**: joint targets for one machine of up to [`MAX_AXES`] axes, grouped into channels.
//!
//! Units are millimetres, radians and seconds. Feeds are millimetres per minute. Motion keeps
//! lines, arcs and helices. Process timing from `campanion-path` is stored on the process
//! parameters and, when a path is converted, expanded into process events and dwells in that order.
//! [`write_ngc`] prints a program as generic RS274/NGC text for tests. It is not a machine post.
//! The text ends with `M30`, or with `M2` when [`NgcOptions`] says so.
//!
//! # Rotary axis
//!
//! A program may name a 4th-axis setup ([`RotarySetup`]): A about part X or B about part Y, a
//! point on that axis, a direction sign, and optional travel limits in degrees. A [`Pose`] may
//! then carry a machine A/B angle in degrees (the value may exceed 360 for multi-turn motion).
//! Poses without an angle stay 3-axis. Helpers rotate part-frame geometry into the machine frame
//! and recover an index angle that turns a tool axis onto machine +Z. [`wrap_point`] maps a flat
//! `(u, v, w)` coordinate on a [`Cylinder`] into that frame; [`wrap_polyline`] splits the flat
//! path so each chord stays within tolerance. Surface feed is the flat-frame length. The IR does
//! not encode a particular controller.
//!
//! # Turning
//!
//! A program may name a lathe setup ([`TurningSetup`]) instead of a rotary table. The two cannot
//! be combined yet. Turning toolpaths stay ordinary [`ClPath`]s whose poses are in the part frame.
//! Z is the spindle axis, positive away from the chuck toward the tailstock and the tool. X is a
//! radius in millimetres, never a diameter. Y is 0 for XZ work. Diameter mode is a post setting
//! only; the IR never stores diameters in X. The pose axis is not interpreted for XZ turning
//! moves. Turning code passes [`Vec3::Z`] because a pose requires a unit axis. A single-point
//! thread is [`ClMotion::Thread`]: a straight move whose `lead` is millimetres per spindle
//! revolution. It is valid only when the program has a turning setup.
//!
//! Spindles are named (`main` now, a sub-spindle later). Each has its own optional C axis and a
//! work-zero offset along Z. A head, a turret or a B-axis head, carries X and Z and optionally Y
//! and B. Every turning toolpath has a binding that names its spindle, head and channel. An M10
//! lathe uses one spindle `main`, one head `turret1` and channel 0. Live tooling, Y, B and a
//! second spindle fit in these types: spindles and heads are lists, and the extra axes and flags
//! are optional fields. Bar and tube stock runs from the finished front face at Z = 0 back to
//! Z = -length. Facing allowance comes from the operation, not from the stock value.
//! The IR stores the finished section on the turning setup when one is recorded.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod check;
mod convert;
mod geom;
mod machine;
mod motion;
mod ngc;
mod process;
mod program;
mod rotary;
mod schema;
mod turning;
mod verify;
mod wrap;

pub use convert::Import;
pub use geom::{ArcSense, Plane, Pose, Vec3};
pub use machine::{Axis, AxisIndex, AxisKind, AxisLetter, AxisLimits, AxisName, Channel, KinematicNode, Machine};
pub use motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, ProfileTab};
pub use ngc::{NgcOptions, ProgramEnd, write_ngc, write_ngc_with};
pub use process::{CoolantMode, Event, ParamValue, ProcessKind, ProcessParams, Tool};
pub use program::{Document, Program, SyncPoint};
pub use rotary::{
    RotaryAxisKind, RotaryDirection, RotaryLimits, RotarySetup, index_angle_for_axis, machine_to_part, part_to_machine, rotate_point, rotate_vector,
    unwrap_angle,
};
pub use schema::json_schema;
pub use turning::{Hand, Head, HeadAxis, Insert, InsertShape, Spindle, SpindleSpeed, TurnBinding, TurnInsert, TurnSection, TurnStock, TurningSetup};
pub use verify::{Bounds, Issue, RefKind, VerifyOptions, VerifyReport, verify, verify_with};
pub use wrap::{Cylinder, unwrap_pose, wrap_point, wrap_polyline};

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Identifier for the current on-disk / interchange IR format.
pub const FORMAT_TAG: &str = "campanion-ir/1";

/// Major version this build reads and writes.
pub const FORMAT_MAJOR: u32 = 1;

/// Hard cap on joint/axis count.
///
/// The target is 13 axes and 16 is the hard cap matching common controller joint limits.
pub const MAX_AXES: usize = 16;

/// Hard cap on channels in one machine.
pub const MAX_CHANNELS: usize = 16;

/// How many named axis letters exist before extras are indexed.
///
/// The letters are X Y Z A B C U V W, in that order.
pub const AXIS_LETTER_COUNT: usize = 9;

/// Result type for IR operations.
pub type Result<T> = std::result::Result<T, IrError>;

/// Errors from IR encode/decode and validation.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum IrError {
    /// The requested operation is not supported yet.
    #[error("not supported yet: {0}")]
    NotSupported(String),
    /// The caller supplied invalid input.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// `format` names this family but the major version is not [`FORMAT_MAJOR`].
    #[error("unsupported IR major version {found}; this build reads major {supported}")]
    UnsupportedVersion {
        /// Major version parsed from the document tag.
        found: u32,
        /// Major version this build accepts.
        supported: u32,
    },
    /// `format` is missing the `campanion-ir/` prefix or is otherwise not a version tag.
    #[error("unrecognised format tag `{0}`")]
    UnknownFormat(String),
    /// JSON parse or encode failed.
    #[error("json: {0}")]
    Json(String),
    /// A path operation failed while converting into the IR.
    #[error(transparent)]
    Path(#[from] campanion_path::PathError),
}

/// Accept [`FORMAT_TAG`] and later minor tags of the same major (`campanion-ir/1.2`).
///
/// A different major is [`IrError::UnsupportedVersion`]. Anything that is not a
/// `campanion-ir/<major>` tag is [`IrError::UnknownFormat`].
pub fn check_format(tag: &str) -> Result<()> {
    let Some(rest) = tag.strip_prefix("campanion-ir/") else {
        return Err(IrError::UnknownFormat(tag.to_string()));
    };
    let (major_token, minor) = match rest.split_once('.') {
        Some((major, minor)) => (major, Some(minor)),
        None => (rest, None),
    };
    let Some(major) = parse_major(major_token) else {
        return Err(IrError::UnknownFormat(tag.to_string()));
    };
    if major != FORMAT_MAJOR {
        return Err(IrError::UnsupportedVersion { found: major, supported: FORMAT_MAJOR });
    }
    if let Some(minor) = minor
        && !valid_minor(minor)
    {
        return Err(IrError::UnknownFormat(tag.to_string()));
    }
    Ok(())
}

fn parse_major(token: &str) -> Option<u32> {
    if token.is_empty() || !token.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    if token.len() > 1 && token.starts_with('0') {
        return None;
    }
    token.parse().ok()
}

fn valid_minor(minor: &str) -> bool {
    if minor.is_empty() || minor.starts_with('.') || minor.ends_with('.') || minor.contains("..") {
        return false;
    }
    minor.bytes().all(|byte| byte.is_ascii_digit() || byte == b'.')
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod turning_tests;
