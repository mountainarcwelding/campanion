//! Data-defined RS274/NGC posts.
//!
//! A [`PostDef`] is JSON. [`post_program`] writes cutter-location toolpaths as G-code for that
//! dialect. Axis-space channel programs are not written. A lathe post with `channels` writes each
//! turning channel as its own section of the same file. The text has no dates or timestamps, so
//! the same program and post always produce the same bytes.
//!
//! [`crate::read_back`] reads the dialect back into moves. [`crate::compare`] checks those moves
//! against the program that was posted.
//!
//! A post may declare a rotary axis. Poses that carry an angle are written in the machine frame.
//! The feed rule for a move that changes the angle is in the crate README.
//!
//! A post may declare a lathe block. A turning program is written in the part XZ frame. Diameter
//! mode, constant surface speed and the G2/G3 sense are described in the crate README.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod balance;
mod compare;
mod def;
mod error;
mod fmt;
mod geom;
mod interp;
mod read;
mod rotary;
mod write;

pub use balance::{ChannelSegment, channel_balance};
pub use compare::compare;
pub use read::{ReadMove, read_back};
pub use write::{PostOptions, post_program};

pub use def::{ChannelsOutput, PostDef, load_post};
pub use error::{PostError, Result};

const LINUXCNC_JSON: &str = include_str!("../posts/linuxcnc.json");
const GRBL_JSON: &str = include_str!("../posts/grbl.json");
const MACH3_JSON: &str = include_str!("../posts/mach3.json");
const FANUC_JSON: &str = include_str!("../posts/fanuc.json");
const HAAS_JSON: &str = include_str!("../posts/haas.json");
const PLASMA_JSON: &str = include_str!("../posts/plasma.json");
const GRBL_LASER_JSON: &str = include_str!("../posts/grbl_laser.json");
const LINUXCNC_LATHE_JSON: &str = include_str!("../posts/linuxcnc_lathe.json");
const FANUC_LATHE_JSON: &str = include_str!("../posts/fanuc_lathe.json");
const HAAS_LATHE_JSON: &str = include_str!("../posts/haas_lathe.json");
const GRBL_LATHE_JSON: &str = include_str!("../posts/grbl_lathe.json");
const GENERIC_LATHE_2CH_JSON: &str = include_str!("../posts/generic_lathe_2ch.json");

/// Ids of the posts compiled into this crate, in a stable order.
pub fn builtin_ids() -> &'static [&'static str] {
    &[
        "linuxcnc",
        "grbl",
        "mach3",
        "fanuc",
        "haas",
        "plasma",
        "grbl_laser",
        "linuxcnc_lathe",
        "fanuc_lathe",
        "haas_lathe",
        "grbl_lathe",
        "generic_lathe_2ch",
    ]
}

/// A built-in post, or `None` when `id` is unknown or its JSON fails validation.
pub fn builtin(id: &str) -> Option<PostDef> {
    let json = match id {
        "linuxcnc" => LINUXCNC_JSON,
        "grbl" => GRBL_JSON,
        "mach3" => MACH3_JSON,
        "fanuc" => FANUC_JSON,
        "haas" => HAAS_JSON,
        "plasma" => PLASMA_JSON,
        "grbl_laser" => GRBL_LASER_JSON,
        "linuxcnc_lathe" => LINUXCNC_LATHE_JSON,
        "fanuc_lathe" => FANUC_LATHE_JSON,
        "haas_lathe" => HAAS_LATHE_JSON,
        "grbl_lathe" => GRBL_LATHE_JSON,
        "generic_lathe_2ch" => GENERIC_LATHE_2CH_JSON,
        _ => return None,
    };
    load_post(json).ok()
}

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests;
