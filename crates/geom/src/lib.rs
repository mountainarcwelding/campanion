//! 2D geometry primitives for the CAMpanion toolpath engine.
//!
//! Units are millimetres and radians. The frame is right-handed with Z up; this crate is the XY
//! plane. Lines and circular arcs stay exact. Offset and boolean operations go through
//! [`backend`], which is the only module that knows which third-party engine is linked.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod arc;
pub mod backend;
mod bbox;
mod cleanup;
mod contour;
mod line;
mod point;
mod polyline;
mod region;
mod segment;
mod tie;
mod tolerance;
mod util;

pub use arc::Arc;
pub use backend::{BooleanOp, CavalierEngine, RegionEngine, boolean, offset, offset_region, offset_rings};
pub use bbox::BoundingBox;
pub use cleanup::cleanup;
pub use contour::{Containment, Contour, Orientation};
pub use line::Line;
pub use point::{Point, Vec2};
pub use polyline::{Polyline, Vertex};
pub use region::{Region, boolean_regions};
pub use segment::Segment;
pub use tie::{cmp_float_key, cmp_xy_rounded, float_keys_equal, max_beats, min_beats, select_max_by_key, select_min_by_key};
pub use tolerance::Tolerance;

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Upper bound on segments accepted by [`cleanup`] and the region backend.
///
/// Larger inputs are rejected so a hostile file cannot force an unbounded join.
pub const MAX_SEGMENTS: usize = 8192;

/// Result type for geometry operations.
pub type Result<T> = std::result::Result<T, GeomError>;

/// Errors from geometry operations.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum GeomError {
    /// The requested operation is not supported yet.
    #[error("not supported yet: {0}")]
    NotSupported(String),
    /// The caller supplied invalid input.
    #[error("invalid input: {0}")]
    InvalidInput(String),
}

#[cfg(test)]
mod props;

#[cfg(test)]
mod tests {
    use super::GeomError;

    #[test]
    fn geom_error_display() {
        assert_eq!(GeomError::NotSupported("arcs".into()).to_string(), "not supported yet: arcs");
        assert_eq!(GeomError::InvalidInput("nan".into()).to_string(), "invalid input: nan");
    }
}
