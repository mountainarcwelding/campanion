//! Resin MSLA output for the Anycubic Photon Mono M7 Max.
//!
//! CAMpanion slices a triangle mesh into one bitmap per layer, run-length encodes each bitmap as a
//! `.pw0Img` layer image, and writes a `.pm7m` file (a zip container) in the layout of a sample
//! export. The format was derived only from that one sample file, so every
//! field the sample does not explain is copied as a constant and listed in `docs/resin.md`.
//!
//! **Not printer-tested.** Nothing here has been printed yet.
//!
//! - [`pw0`] is the layer image codec (4-bit gray value plus 12-bit run length).
//! - [`slice`] turns a mesh into per-layer pixel spans.
//! - [`pm7m`] writes and reads the container.
//! - [`zip`] is the minimal zip reader and writer the container needs.
//! - [`preview`] draws the two thumbnails.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod build;
mod error;
pub mod pm7m;
pub mod preview;
pub mod pw0;
pub mod slice;
pub mod supports;
pub mod zip;

pub use build::{BuildReport, build_pm7m};
pub use error::ResinError;
pub use pm7m::{LayerParams, LayerRecord, M7_MAX, MachineSpec, Pm7mFile, ResinSettings, WriteInput, make_previews, read_pm7m, write_pm7m};
pub use slice::{Placement, PreparedMesh, SliceGrid, Span, encode_spans};
pub use supports::{
    InputPoint, REQUIRED_FLAGS, SupportParams, SupportParamsOverlay, SupportPoint, SupportedMesh, generate_supports, is_watertight,
    parse_points_text, parse_support_params_json, render_supported_png,
};
