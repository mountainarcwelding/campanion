//! Data-defined RS274/NGC posts.
//!
//! A [`PostDef`] is JSON. [`post_program`] writes cutter-location toolpaths as G-code for that
//! dialect. Channel programs are not written. The text has no dates or timestamps, so the same
//! program and post always produce the same bytes.
//!
//! [`crate::read_back`] reads the dialect back into moves. [`crate::compare`] checks those moves
//! against the program that was posted.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod compare;
mod def;
mod error;
mod fmt;
mod geom;
mod read;
mod write;

pub use compare::compare;
pub use read::{ReadMove, read_back};
pub use write::{PostOptions, post_program};

pub use def::{PostDef, load_post};
pub use error::{PostError, Result};

const LINUXCNC_JSON: &str = include_str!("../posts/linuxcnc.json");
const GRBL_JSON: &str = include_str!("../posts/grbl.json");
const MACH3_JSON: &str = include_str!("../posts/mach3.json");
const FANUC_JSON: &str = include_str!("../posts/fanuc.json");
const HAAS_JSON: &str = include_str!("../posts/haas.json");
const PLASMA_JSON: &str = include_str!("../posts/plasma.json");
const GRBL_LASER_JSON: &str = include_str!("../posts/grbl_laser.json");

/// Ids of the posts compiled into this crate, in a stable order.
pub fn builtin_ids() -> &'static [&'static str] {
    &["linuxcnc", "grbl", "mach3", "fanuc", "haas", "plasma", "grbl_laser"]
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
        _ => return None,
    };
    load_post(json).ok()
}

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests;
