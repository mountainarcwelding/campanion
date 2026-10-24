//! Lathe profile input for CAMpanion.
//!
//! A lathe half-profile lives in a 2D plane of [`campanion_geom::Point`] values.
//! `point.x()` is **Z**, the coordinate along the spindle axis, with **+Z pointing
//! away from the chuck**. `point.y()` is the **radius** and is non-negative, apart
//! from about 1e-9 mm of float noise (or the tolerance passed to [`TurnProfile::try_new`]).
//! Units are millimetres. **Z = 0 is the finished front face** by convention, and the
//! part extends toward **−Z**.
//!
//! A [`TurnProfile`] is an open chain ordered from the front (the end with the larger
//! Z) toward the chuck. Lines and arcs stay exact. [`TurnProfile::radius_at`] reports
//! the largest radius where the line `x = z` meets that chain.
//!
//! # Drawings
//!
//! [`TurnProfile::from_dxf`] and [`TurnProfile::from_svg`] read a half-profile drawn
//! with the spindle axis along drawing X. Drawing X becomes Z and drawing Y becomes
//! the radius. [`campanion_import::import_svg`] already flips SVG Y into the same
//! plane as DXF, so both importers share that mapping. A drawing that lies mostly
//! below the axis is mirrored before it is validated. A closed contour is opened by
//! dropping the axis segment (the run that lies on the spindle).
//!
//! # Meshes and STEP
//!
//! [`TurnProfile::from_mesh`] samples the **outside diameter** of a mesh that is
//! already centred on the spindle. An internal bore is not recovered.
//! [`TurnProfile::from_section_dxf`] reads a full planar section through the turning
//! axis (DXF X is a signed radius, DXF Y is Z), as `solidcraft-cli` writes it, and
//! keeps one half-profile.
//!
//! Facing, longitudinal roughing, and finishing are in [`ops`]. Those paths are
//! coordinates of the programmed point with the nose radius applied here, so they
//! do not depend on cutter compensation (G41/G42). Threading, grooving, parting and
//! centreline drilling are in [`cycles`]. [`to_cl_path`] writes one path as a
//! [`campanion_ir::ClPath`] in the ZX plane.
//!
//! [`check_turn_gouges`] places the insert outline and an optional holder on every
//! sample of a [`TurnPath`], including rapids, and reports how far the nose, a
//! cutting edge, or the holder enters the finished solid. [`TurnGouge::toolpath`]
//! is left at 0; the caller that checks several paths sets the index.
//!
//! This crate is clean-room. It does not use OCCT, FreeCAD, planegcs, or OpenCAMLib.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod cycles;
mod gouge;
mod ir;
mod mesh;
mod ops;
mod profile;
mod section;

#[cfg(test)]
mod section_tests;

pub use cycles::{DrillParams, GrooveParams, Infeed, PartParams, ThreadParams, centre_drill, groove, part_off, thread, thread_pass_depths};
pub use gouge::{GougeKind, HolderGeom, InsertGeom, TurnGouge, check_turn_gouges};
pub use ir::to_cl_path;
pub use mesh::MeshAxis;
pub use ops::{FaceParams, FinishParams, RoughParams, Side, TipPoint, TurnMove, TurnPath, TurnTool, compensated_profile, face, finish, rough};
pub use profile::TurnProfile;

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Most segments [`TurnProfile::try_new`] accepts.
pub const MAX_PROFILE_SEGMENTS: usize = 100_000;

/// Most axial samples [`TurnProfile::from_mesh`] will take.
pub const MAX_MESH_SAMPLES: usize = 100_000;

/// Result type for lathe profile operations.
pub type Result<T> = std::result::Result<T, TurnError>;

/// Failures while building a lathe profile.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum TurnError {
    /// The caller supplied a number, a chain, or a sample count this crate rejects.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// The drawing or mesh has no usable half-profile.
    #[error("no profile: {0}")]
    NoProfile(String),
    /// A geometry primitive rejected the value.
    #[error(transparent)]
    Geom(#[from] campanion_geom::GeomError),
    /// DXF or SVG import failed.
    #[error(transparent)]
    Import(#[from] campanion_import::ImportError),
    /// Reading the mesh failed. The mesh error is kept as text.
    #[error("mesh: {0}")]
    Mesh(String),
    /// The cutter-location path rejected the converted motion.
    #[error(transparent)]
    Ir(#[from] campanion_ir::IrError),
}

#[cfg(test)]
mod tests;
