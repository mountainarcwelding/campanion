//! Machine-neutral 2D toolpaths.
//!
//! Moves carry geometry and a feed class. They are not G-code and they do not name a controller.
//! Profiles, pockets, facing, drilling, engraving and chamfers build those moves. A profile either offsets the
//! contour here (computed compensation, through [`campanion_geom`]) or leaves the path on the
//! contour and records a left/right flag for a later IR. Ordering links those features.
//! Depth passes repeat a closed contour downward.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod chamfer;
mod check;
mod curve;
mod depth;
mod drill;
mod engrave;
mod face;
mod order;
mod pocket;
mod profile;
mod timing;
mod tool;
mod toolpath;

pub use chamfer::{Chamfer, ChamferOp};
pub use depth::{DepthEntry, DepthPlan, depth_toolpath, pass_depths};
pub use drill::{DrillCycle, DrillHit, DrillOp, Drilling};
pub use engrave::{Engrave, EngraveOp};
pub use face::{Face, FaceOp};
pub use order::{Feature, OrderedCuts, link_features, order_and_link};
pub use pocket::{Pocket, PocketOp, PocketStrategy, RingOrder};
pub use profile::{CutDirection, Lead, Profile, ProfileOp, ProfileSide, StartPoint, Tab, TabLayout, TabSpec};
pub use timing::{ProcessTiming, apply_timing};
pub use tool::{Compensation, ControllerSide, Tool};
pub use toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Result type for path operations.
pub type Result<T> = std::result::Result<T, PathError>;

/// Errors from path operations.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum PathError {
    /// The requested operation is not supported yet.
    #[error("not supported yet: {0}")]
    NotSupported(String),
    /// The caller supplied invalid input.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// A geometry error from [`campanion_geom`].
    #[error(transparent)]
    Geom(#[from] campanion_geom::GeomError),
}

#[cfg(test)]
mod tests {
    use super::PathError;

    #[test]
    fn path_error_display() {
        assert_eq!(PathError::NotSupported("splines".into()).to_string(), "not supported yet: splines");
        assert_eq!(PathError::InvalidInput("empty".into()).to_string(), "invalid input: empty");
        let geom = campanion_geom::GeomError::InvalidInput("nan".into());
        assert_eq!(PathError::from(geom).to_string(), "invalid input: nan");
    }
}
