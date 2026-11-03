//! Data-defined RS274/NGC posts.
//!
//! A [`PostDef`] is JSON. [`post_program`] writes cutter-location toolpaths as G-code for that
//! dialect. Axis-space channel programs are not written. A lathe post with `channels` writes each
//! turning channel as its own section of the same file. The text has no dates or timestamps, so
//! the same program and post always produce the same bytes.
//!
//! [`crate::read_back`] reads the dialect back into moves. [`crate::compare`] checks those moves
//! against the program that was posted. [`audit`] replays a mill program and reports the modal
//! state a control would keep at each motion. [`audit_lathe`] does the same for a lathe or
//! mill-turn program.
//!
//! A post may declare a rotary axis. Poses that carry an angle are written in the machine frame.
//! The feed rule for a move that changes the angle is in the crate README.
//!
//! A post with `five_axis.continuous` writes simultaneous five-axis motion. Without `tcp` the post
//! solves the inverse and emits linear blocks with XYZ and rotary words. With `tcp`, XYZ is the
//! part-frame tool tip and the rotary words still come from that solution.
//!
//! A post may declare a lathe block. A turning program is written in the part XZ frame. Diameter
//! mode, constant surface speed and the G2/G3 sense are described in the crate README.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod audit;
mod audit_lathe;
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

pub use audit::{
    ArcReplay, AuditReport, CoolantState, DistanceMode, FeedMode, LengthOffset, MotionMode, MotionState, SpindleState, UnitMode, WorkOffset, audit,
};
pub use audit_lathe::{ChannelAudit, HousingMove, InterpFrame, LatheAudit, LatheFeedMode, LatheMotion, NoseComp, XMode, audit_lathe};
pub use balance::{ChannelSegment, channel_balance};
pub use compare::compare;
pub use read::{ReadMove, read_back};
pub use write::{PostOptions, post_program};

pub use def::{ChannelsOutput, PostDef, load_post};
pub use error::{PostError, Result};

const LINUXCNC_JSON: &str = include_str!("../posts/linuxcnc.json");
const LINUXCNC_5AX_JSON: &str = include_str!("../posts/linuxcnc_5ax.json");
const LINUXCNC_5AX_TCP_JSON: &str = include_str!("../posts/linuxcnc_5ax_tcp.json");
const GRBL_JSON: &str = include_str!("../posts/grbl.json");
const MACH3_JSON: &str = include_str!("../posts/mach3.json");
const FANUC_JSON: &str = include_str!("../posts/fanuc.json");
const HAAS_JSON: &str = include_str!("../posts/haas.json");
const HAAS_UMC_JSON: &str = include_str!("../posts/haas_umc.json");
const HAAS_UMC_TCPC_JSON: &str = include_str!("../posts/haas_umc_tcpc.json");
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
        "linuxcnc_5ax",
        "linuxcnc_5ax_tcp",
        "grbl",
        "mach3",
        "fanuc",
        "haas",
        "haas_umc",
        "haas_umc_tcpc",
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
        "linuxcnc_5ax" => LINUXCNC_5AX_JSON,
        "linuxcnc_5ax_tcp" => LINUXCNC_5AX_TCP_JSON,
        "grbl" => GRBL_JSON,
        "mach3" => MACH3_JSON,
        "fanuc" => FANUC_JSON,
        "haas" => HAAS_JSON,
        "haas_umc" => HAAS_UMC_JSON,
        "haas_umc_tcpc" => HAAS_UMC_TCPC_JSON,
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
