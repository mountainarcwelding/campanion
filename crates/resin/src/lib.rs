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
pub use pm7m::{
    LayerParams, LayerRecord, M7_MAX, MachineSpec, Pm7mFile, ResinSettings, WriteInput, load_print_settings, make_previews, read_pm7m,
    settings_from_pm7m_bytes, write_pm7m,
};
pub use slice::{Placement, PreparedMesh, SliceGrid, Span, encode_spans};
pub use supports::{
    CALIBRATION_SCHEMA, CalibratedTip, CalibrationMeasurement, CalibrationProfile, CancelFlag, ClassifiedRegion, DetectedFeatures, FeatureClass,
    ForceBreakdown, GenerateOptions, GenerateProgress, InputPoint, Island, IslandDetectParams, MeshPose, MissingConstant, OptimizeInput,
    OptimizeResult, PointSource, REQUIRED_CONSTANTS, REQUIRED_FLAGS, REQUIRED_TIP_FIELDS, STANDARD_GRAVITY_M_S2, SupportParams, SupportParamsOverlay,
    SupportPoint, SupportedMesh, add_support_point, apply_mesh_pose, center_on_plate, classify_features, constant_label, delete_support_point,
    detect_islands, detect_islands_with, drop_to_plate, generate_supports, generate_supports_with, import_calibration, is_watertight,
    load_calibration, missing_constants_in, move_support_point, optimize_supports, parse_calibration, parse_points_text, parse_support_params_json,
    rebuild_supported, render_supported_png, snap_to_step, write_points_text,
};
