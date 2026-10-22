//! ASCII DXF import into [`campanion_geom`] contours.
//!
//! The reader is a clean-room group-code parser for the public DXF specification. It reads
//! `LINE`, `ARC`, `CIRCLE` and `LWPOLYLINE` (including bulges and the closed flag). Circles,
//! arcs and bulge spans stay exact. `POLYLINE`, `SPLINE` and `ELLIPSE` are skipped and counted
//! until a later pass. Any other entity is skipped and counted the same way.
//!
//! `$INSUNITS` is converted to millimetres. A missing code, or code 0 (unitless), is read as
//! millimetres. Coordinates are the XY plane; Z is dropped. An extrusion not parallel to Z is
//! rejected. An extrusion of `(0, 0, -1)` is mirrored on X, which reverses bulge and arc sense.
//!
//! Open segments on the same layer are chained when their ends lie within the gap tolerance.
//! A gap wider than the linear tolerance is filled with a straight bridge so the contour is
//! connected. Pieces that do not close stay open. Each contour keeps the DXF layer name (`"0"`
//! when the entity omits group 8).
//!
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

#[allow(dead_code)]
mod arcfit;
mod chain;
mod dxf;
#[allow(dead_code)]
mod ellipse;
mod svg;
mod units;

pub use dxf::import_dxf;
pub use svg::import_svg;

use std::fmt;

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Largest ASCII DXF accepted, in bytes.
pub const MAX_DXF_BYTES: usize = 32 * 1024 * 1024;

/// Most segments produced from one entity (a polyline, a spline or an ellipse).
pub const MAX_ENTITY_SEGMENTS: usize = 65_536;

/// Most segments returned from one import, across every layer.
pub const MAX_TOTAL_SEGMENTS: usize = 200_000;

/// Largest absolute coordinate kept after conversion to millimetres.
pub const MAX_COORD_MM: f64 = 1.0e8;

/// Result type for import.
pub type Result<T> = std::result::Result<T, ImportError>;

/// A contour and the DXF layer it came from.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportedContour {
    layer: String,
    contour: campanion_geom::Contour,
}

impl ImportedContour {
    pub(crate) fn new(layer: impl Into<String>, contour: campanion_geom::Contour) -> Self {
        Self { layer: layer.into(), contour }
    }

    /// Layer name. Omitted group 8 becomes `"0"`.
    pub fn layer(&self) -> &str {
        &self.layer
    }

    /// Lines and arcs in millimetres.
    pub fn contour(&self) -> &campanion_geom::Contour {
        &self.contour
    }
}

/// Contours read from one DXF, in millimetres.
///
/// Closed entities stay in file order. Open paths are chained per layer and appended after them.
/// [`Self::skipped_entities`] counts entities this importer does not read.
/// [`Self::skipped_types`] names those entities and how many of each were skipped, sorted by name.
#[derive(Clone, Debug, PartialEq)]
pub struct Drawing {
    contours: Vec<ImportedContour>,
    skipped_entities: usize,
    skipped_types: Vec<(String, usize)>,
}

impl Drawing {
    pub(crate) fn new(contours: Vec<ImportedContour>, skipped_entities: usize, mut skipped_types: Vec<(String, usize)>) -> Self {
        skipped_types.sort();
        Self { contours, skipped_entities, skipped_types }
    }

    /// Contours in millimetres, each tagged with its DXF layer.
    pub fn contours(&self) -> &[ImportedContour] {
        &self.contours
    }

    /// Entities in the ENTITIES section that were not imported.
    ///
    /// A `POLYLINE` and the `VERTEX` / `SEQEND` records that belong to it count as one.
    /// That total is the sum of [`Self::skipped_types`].
    pub fn skipped_entities(&self) -> usize {
        self.skipped_entities
    }

    /// Skipped entity type names and counts, sorted by name.
    ///
    /// Each pair is `(name, count)`, for example `("TEXT", 1)`.
    pub fn skipped_types(&self) -> &[(String, usize)] {
        &self.skipped_types
    }
}

/// Tolerances for one import.
///
/// `gap` is the largest opening, in millimetres, that still joins two open ends. It defaults to
/// the linear tolerance. A positive gap larger than that is bridged with a line.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportOptions {
    tolerance: campanion_geom::Tolerance,
    gap: f64,
}

impl ImportOptions {
    /// Gap equals the linear tolerance.
    pub fn new(tolerance: campanion_geom::Tolerance) -> Self {
        Self { gap: tolerance.linear(), tolerance }
    }

    /// `gap` must be finite and `>= 0`.
    pub fn with_gap(tolerance: campanion_geom::Tolerance, gap: f64) -> Result<Self> {
        if !gap.is_finite() || gap < 0.0 {
            return Err(invalid("gap tolerance must be finite and >= 0"));
        }
        Ok(Self { tolerance, gap })
    }

    pub fn tolerance(&self) -> campanion_geom::Tolerance {
        self.tolerance
    }

    pub fn gap(&self) -> f64 {
        self.gap
    }
}

/// Failures while reading a DXF or building contours.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ImportError {
    /// The file uses an entity or a unit this importer does not read yet.
    #[error("not supported yet: {0}")]
    NotSupported(String),
    /// The file or the options are not usable.
    #[error("invalid input: {0}")]
    InvalidInput(String),
}

impl ImportError {
    pub(crate) fn from_geom(err: campanion_geom::GeomError) -> Self {
        match err {
            campanion_geom::GeomError::NotSupported(message) => Self::NotSupported(message),
            campanion_geom::GeomError::InvalidInput(message) => Self::InvalidInput(message),
        }
    }
}

pub(crate) fn invalid(message: impl Into<String>) -> ImportError {
    ImportError::InvalidInput(message.into())
}

pub(crate) fn unsupported(message: impl Into<String>) -> ImportError {
    ImportError::NotSupported(message.into())
}

pub(crate) fn invalid_at(line: usize, message: impl fmt::Display) -> ImportError {
    invalid(format!("line {line}: {message}"))
}
