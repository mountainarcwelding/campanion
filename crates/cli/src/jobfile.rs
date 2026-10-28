//! `campanion-job/1`: a source part, a tool table and layer operations.
//!
//! Lengths and feeds in a loaded [`JobFile`] are millimetres and millimetres per minute.
//! The file's `units` field says whether those numbers were written as millimetres or inches.
//! The optional job-level `heights` object carries a clearance (safe) height and a retract
//! height. Omitted heights are 5 mm and 2 mm. Explicit heights scale with `units`. The retract
//! height is at most the clearance height, and both values are finite.
//! Tool, feed and timing objects are parsed by the same functions as the profile job in `job`.
//! Timing defaults match `campanion-path`: pierce delay runs with the process on, post delay
//! runs with the process off, and both delays are zero when the file omits them.
//! Each operation may set `entry`. Omitted, it is a plunge. A ramp stores `angle_deg` from the
//! XY plane as radians on [`DepthEntry::Ramp`]. A helix stores a unit-scaled `radius` and the
//! same angle; one turn drops `2π r tan(angle)` millimetres, which is [`DepthEntry::Helix`]'s
//! pitch. The angle is in `(0, 90]` degrees. The radius is positive and at most 1e6 mm.
//!
//! A STEP source, a path ending in `.step` or `.stp` in any case, must include `face`: a
//! point `[x, y, z]` on the planar face to cut. Those coordinates are millimetres in model
//! space and are never multiplied by `units`. The face outline and its holes are the 2D
//! geometry. Every other source must leave `face` out.
//!
//! The source hash is FNV-1a 64 of the part file's bytes. It is a change-detection checksum,
//! not a cryptographic hash. See `docs/job-file.md`.
//!
//! An STL source, a path ending in `.stl` in any case, is a triangle mesh. Its hash is those
//! raw bytes. A mesh job may only contain `surface`, `rough`, `waterline` and `rotary_finish`
//! operations, and those kinds are rejected on every other source. A surface rasters the mesh. It requires exactly one of
//! `stepover` and `scallop_height`. `scallop_height` is a flat-floor cusp height; the run turns
//! it into an XY stepover and caps that stepover at the tool diameter. On a slope the real
//! cusp is larger for the same XY stepover, because raster spacing is measured in XY.
//! `sample_step` defaults to half of the stepover that is used. `angle_deg` is 0 or 90,
//! `margin` grows the mesh XY bounds, and `floor_z` defaults to the mesh minimum Z.
//! `stock_to_leave` on a surface is one length that raises every cutter-location Z. That is an
//! axial offset, not a 3D rest offset. `layers`, `depth`, `step_down` and `side` are rejected.
//!
//! `rough` is Z-level roughing. `step_down` is required. `stepover` defaults to 0.4 times the
//! tool diameter. `margin` defaults to the tool radius and grows the mesh XY bounds into the
//! stock rectangle. `top_z` defaults to the mesh maximum Z and `floor_z` to the minimum.
//! `grid` defaults to the minimum of `stepover / 4` and 0.5 mm. `stock_to_leave` is one length;
//! omitted, it is 0.5 mm, which is not scaled again when the file is in inches. `entry` is the
//! same object a 2D operation uses. Omitted, it is a plunge. Keys that belong to other kinds
//! are rejected.
//!
//! `waterline` finishes steep walls. `step_down` is required. `margin` defaults to the tool
//! radius. `top_z` and `floor_z` default as they do for a rough, and the depth plan is the same.
//! `grid` defaults to the minimum of `step_down` and 0.25 mm. That 0.25 mm cap is not scaled
//! again when the file is in inches. `min_slope_deg` defaults to 0 and is not scaled. Zero keeps
//! every closed loop. A positive value keeps only the portions at least that steep, and a cut
//! portion is an open polyline. `stock_to_leave` is one axial length, default 0, added to every
//! cutter-location Z. Entry is a plunge.
//!
//! A tool may set `shape`: `"flat"` (the default), `"ball"`, or `{"bull": corner_radius}`.
//! A shape other than flat is only used by a surface, rough, waterline or rotary finish operation.
//! Two-dimensional operations keep the flat diameter offset.
//! A tool may set `speed`, a spindle speed in rpm: positive, finite, and at most 100000. An
//! operation may set `speed` too; the operation value wins. Omitted, there is no speed. Speed is
//! not scaled by `units`. `campanion run` copies the resolved speed onto process-on. When the
//! chosen post's spindle-on template uses `{speed}` and a spindle operation has none, `run` fails
//! and names the operation. Consecutive toolpaths that share a tool get one tool-change event;
//! the first toolpath, and each later toolpath whose tool differs, get a change.
//!
//! A face clears the bounding box of its layers. `angle_deg` is 0 or 90 and is stored as
//! radians. `margin` grows that box on every side. `stepover` defaults to three quarters of
//! the tool diameter. A kerf tool cannot face. `stock_to_leave` and `finish` stay on profiles
//! and pockets.
//!
//! A chamfer follows each closed contour with a V-bit. `side` is `outside` or `inside`
//! (`on` is rejected). `width` and `v_angle_deg` are required. `v_angle_deg` is the included
//! angle in degrees and is stored as radians. `tip_diameter` and `tip_clearance` default to 0.
//! `tip_diameter` is strictly less than the tool diameter. The tip depth is
//! `width / tan(v_angle / 2) + tip_clearance`; `depth()` and `step_down()` return that value.
//! `depth` and `step_down` in the file are rejected on a chamfer. A surface rejects both.
//! A rough requires `step_down` and rejects `depth`. Every other kind requires both.
//! The chamfer keys are rejected on every other kind. A kerf tool cannot chamfer.
//!
//! A wrap lays the selected layer contours onto a cylinder about the job rotary axis.
//! `diameter` is the cylinder surface diameter. `depth` is a positive distance below that
//! surface and `step_down` is the maximum pass thickness. Drawing X is `u` along the axis,
//! plus optional `u_offset`. Drawing Y is `v`, the arc length around the surface, plus
//! optional `v_offset`. Both offsets default to 0 and may be negative. `index_deg` is
//! rejected: the angle follows `v`. A wrap without `rotary` is an error that names the
//! operation. See `docs/job-file.md`.
//!
//! `rotary_finish` is a continuous 4-axis finish around the job rotary axis. It needs that
//! setup and an STL source. `u_min` and `u_max` limit the pass along the axis and default to
//! the mesh extent. `angle_step_deg` is the rotary step in degrees, not scaled by `units`.
//! `stepover` is the same step measured in millimetres at the largest radius; when both are
//! set, `stepover` is used. When neither is set, the stepover is the flat-floor scallop for
//! the job chord tolerance, capped at the tool diameter, which needs a ball or bull nose.
//! `sample_step` is the spacing along the axis. Omitted, the run uses `min(0.25, chord * 10)`.
//! `direction` is `zigzag` (the default) or `one_way`. `index_deg` is rejected. A kerf tool
//! is rejected. See `docs/job-file.md`.
//!
//! An optional top-level `rotary` object is a 4th-axis table: `axis` `"a"` or `"b"`, `center`
//! `[x, y, z]` (scaled by `units`), `direction` `"positive"` or `"negative"`, and optional
//! `limits` `{min_deg, max_deg}` in degrees (not scaled). An operation may set `index_deg`,
//! a finite angle in degrees, also not scaled. `index_deg` without `rotary` is an error that
//! names the operation. With a setup and no `index_deg`, the operation runs at 0°. See
//! `docs/job-file.md`.
//!
//! An optional top-level `kinematics` path, relative to the job file, loads a kinematics JSON
//! file. `solution` is `nearest` (the default), `first_positive` or `first_negative`. An
//! operation may set `plane` (`origin`, `normal`, optional `x_dir`). `origin` scales with
//! `units`; the directions are normalised. A plane needs kinematics, and kinematics cannot be
//! combined with `rotary`. A 2D operation with `plane` is planned in that view and mapped
//! into the part frame. `surface`, `rough` and `waterline` reject `plane`. See `docs/job-file.md`.

use std::fmt;
use std::fs::File;
use std::io::Read;
use std::path::{Component, Path, PathBuf};

use campanion_geom::Tolerance;
use campanion_ir::{
    Feeds, FiveAxisSetup, Kinematics, ProcessParams, RotaryAxisKind, RotaryDirection, RotaryLimits, RotarySetup, SolutionPolicy, Tool as IrTool,
    ToolPose, Vec3,
};
use campanion_path::{DepthEntry, ProfileSide, Tool as PathTool};
use serde_json::{Map, Number, Value};

use crate::error::{CliError, MAX_JSON_BYTES};
use crate::job::{
    as_bool, as_f64, as_str, as_u32, check_keys, object, optional, parse_feeds, parse_params, parse_side, parse_tool, require, required_name,
};

/// On-disk tag this build reads. Minor tags are not accepted: unknown fields already fail the load.
pub const FORMAT_TAG: &str = "campanion-job/1";

/// Major version accepted by [`FORMAT_TAG`].
pub const FORMAT_MAJOR: u32 = 1;

/// Default chord tolerance for curve fitting in job files; the geometry engine's 1e-6 mm
/// default would turn every curve into thousands of tiny arcs.
pub const DEFAULT_JOB_CHORD_MM: f64 = 0.01;

/// FNV-1a 64 offset basis.
const FNV_OFFSET: u64 = 0xcbf29ce484222325;
/// FNV-1a 64 prime.
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

const MM_PER_INCH: f64 = 25.4;
const MAX_MM: f64 = 1.0e6;
/// Default clearance (safe height), in millimetres.
const DEFAULT_CLEARANCE_MM: f64 = 5.0;
/// Default retract height, in millimetres.
const DEFAULT_RETRACT_MM: f64 = 2.0;
/// Face stepover when the file omits it, as a fraction of the tool diameter.
const FACE_STEPOVER_OF_DIAMETER: f64 = 0.75;
/// Roughing stepover when the file omits it, as a fraction of the tool diameter.
const ROUGH_STEPOVER_OF_DIAMETER: f64 = 0.4;
/// Stock left by a rough when the file omits `stock_to_leave`, in millimetres.
const DEFAULT_ROUGH_STOCK_MM: f64 = 0.5;
/// Upper bound of the default roughing sample grid, in millimetres.
const DEFAULT_ROUGH_GRID_MM: f64 = 0.5;
/// Upper bound of the default waterline sample grid, in millimetres. Not scaled for inches.
const DEFAULT_WATERLINE_GRID_MM: f64 = 0.25;
const MAX_FEED_MM_PER_MIN: f64 = 1.0e9;
const MAX_DELAY_S: f64 = 1.0e6;
/// Spindle speed ceiling, in rpm. The same cap a live-tool rpm uses.
const MAX_SPINDLE_RPM: f64 = 100_000.0;
const MAX_TOOLS: usize = 256;
const MAX_OPERATIONS: usize = 1_024;
const MAX_LAYERS: usize = 64;
const MAX_SOURCE_PATH: usize = 4_096;

/// A job file could not be read or did not match `campanion-job/1`.
#[derive(Debug, PartialEq, Eq)]
pub enum JobFileError {
    /// Opening or reading the file failed.
    Io {
        /// Path that was opened.
        path: String,
        /// Filesystem error text.
        message: String,
    },
    /// The file is larger than [`MAX_JSON_BYTES`].
    TooLarge {
        /// Path that was opened.
        path: String,
    },
    /// The bytes are not JSON.
    Json {
        /// Parser message, without a `json:` prefix.
        message: String,
    },
    /// A field is missing, the wrong type, or out of range. The text starts with `invalid input:`.
    Invalid {
        /// Display text.
        message: String,
    },
    /// `format` is not a `campanion-job/<major>` tag.
    UnknownFormat {
        /// Tag from the file.
        tag: String,
    },
    /// `format` is `campanion-job/<major>` for some other major.
    UnsupportedVersion {
        /// Major version in the file.
        found: u32,
        /// Major version this build reads.
        supported: u32,
    },
}

impl JobFileError {
    fn io(path: &Path, err: std::io::Error) -> Self {
        Self::Io { path: path.display().to_string(), message: err.to_string() }
    }

    fn invalid(message: impl fmt::Display) -> Self {
        Self::Invalid { message: format!("invalid input: {message}") }
    }
}

impl fmt::Display for JobFileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, message } => write!(f, "{path}: {message}"),
            Self::TooLarge { path } => write!(f, "{path} is larger than {MAX_JSON_BYTES} bytes"),
            Self::Json { message } => write!(f, "json: {message}"),
            Self::Invalid { message } => f.write_str(message),
            Self::UnknownFormat { tag } => write!(f, "unrecognised format tag `{tag}`"),
            Self::UnsupportedVersion { found, supported } => {
                write!(f, "unsupported job major version {found}; this build reads major {supported}")
            }
        }
    }
}

impl std::error::Error for JobFileError {}

/// Length unit of the on-disk numbers. Loaded values are millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Units {
    /// One file unit is one millimetre.
    Millimetre,
    /// One file unit is one inch (25.4 mm).
    Inch,
}

impl Units {
    /// Millimetres per file unit.
    pub fn mm_per_unit(self) -> f64 {
        match self {
            Self::Millimetre => 1.0,
            Self::Inch => MM_PER_INCH,
        }
    }
}

impl fmt::Display for Units {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Millimetre => "mm",
            Self::Inch => "inch",
        })
    }
}

/// Part file the job was built from. `path` is relative to the job file.
///
/// [`Self::face`] is set for a STEP part: a point on the planar face whose outline and holes
/// become the 2D geometry. The point is millimetres in model coordinates and is never scaled
/// by the job's units.
#[derive(Clone, Debug, PartialEq)]
pub struct Source {
    path: String,
    hash: String,
    face: Option<[f64; 3]>,
}

// `parse_face` only stores finite coordinates, so equality is an equivalence.
impl Eq for Source {}

impl Source {
    /// Relative path as written in the file.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// Lowercase 16-digit hex FNV-1a 64 of the part bytes.
    pub fn hash(&self) -> &str {
        &self.hash
    }

    /// Point on the planar face to cut, in millimetres and model coordinates.
    ///
    /// Present only on a STEP source. The outline and holes of that face become the 2D geometry.
    /// The point is never scaled by the job's `units`.
    pub fn face(&self) -> Option<[f64; 3]> {
        self.face
    }

    /// True when the path extension is `step` or `stp`, ignoring ASCII case.
    pub fn is_step(&self) -> bool {
        path_is_step(&self.path)
    }

    /// True when the path extension is `stl`, ignoring ASCII case.
    pub fn is_stl(&self) -> bool {
        path_is_stl(&self.path)
    }
}

fn path_extension_is(path: &str, extension: &str) -> bool {
    Path::new(path).extension().and_then(|extension| extension.to_str()).is_some_and(|text| text.eq_ignore_ascii_case(extension))
}

fn path_is_step(path: &str) -> bool {
    path_extension_is(path, "step") || path_extension_is(path, "stp")
}

fn path_is_stl(path: &str) -> bool {
    path_extension_is(path, "stl")
}

/// Rotating-cutter profile. Flat is the default and the only shape a 2D operation uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ToolShape {
    /// Flat end mill. The bottom is a disk of the tool radius at the tip.
    Flat,
    /// Ball nose. The sphere radius is the tool radius.
    Ball,
    /// Bull nose. `corner_radius` lies in `(0, tool radius)`.
    Bull {
        /// Toroidal corner radius, in millimetres.
        corner_radius: f64,
    },
}

impl ToolShape {
    /// True for [`Self::Flat`], including a tool that omits `shape`.
    pub fn is_flat(self) -> bool {
        matches!(self, Self::Flat)
    }
}

/// One cutter. `id` matches [`IrTool::id`].
#[derive(Clone, Debug, PartialEq)]
pub struct JobTool {
    id: u32,
    path_tool: PathTool,
    ir: IrTool,
    shape: ToolShape,
    /// Spindle speed in rpm. `None` when the file omits `tools[].speed`.
    speed: Option<f64>,
}

impl JobTool {
    pub fn id(&self) -> u32 {
        self.id
    }

    /// Diameter or kerf, in millimetres, as `campanion-path` stores it.
    pub fn path_tool(&self) -> PathTool {
        self.path_tool
    }

    /// The same cutter as an IR tool.
    pub fn ir(&self) -> &IrTool {
        &self.ir
    }

    /// Rotating-cutter diameter in millimetres. `None` for a kerf tool.
    pub fn diameter(&self) -> Option<f64> {
        self.ir.diameter()
    }

    /// Cutter profile. Omitted `shape` is [`ToolShape::Flat`]. A 2D operation ignores this.
    pub fn shape(&self) -> ToolShape {
        self.shape
    }

    /// Spindle speed in rpm. `None` when the file omits `speed`.
    pub fn speed(&self) -> Option<f64> {
        self.speed
    }
}

/// What an operation asks the path engine to do. Geometry comes from the source layers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationKind {
    /// Profile. `side` is the same enum the profile job uses.
    Profile {
        /// Inside, outside, or on the contour.
        side: ProfileSide,
    },
    /// Clear the interior of the layers.
    Pocket,
    /// Drill circles and points on the layers.
    Drill,
    /// Follow the layers on the line.
    Engrave,
    /// Clear a rectangle around the layers. No side.
    Face,
    /// Chamfer a closed contour with a V-bit. `side` is outside or inside. `On` is rejected.
    Chamfer {
        /// Inside or outside the contour.
        side: ProfileSide,
    },
    /// Parallel finish over an STL mesh. No layers and no side.
    Surface,
    /// Z-level roughing over an STL mesh. No layers and no side. `entry` defaults to a plunge.
    Rough,
    /// Waterline finish of steep walls on an STL mesh. No layers and no side. Entry is a plunge.
    Waterline,
    /// Wrap the layer contours onto a cylinder about the job rotary axis. No side.
    Wrap,
    /// Continuous rotary finish around the job rotary axis. No layers and no side.
    RotaryFinish,
}

/// How consecutive rotary-finish passes are joined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotaryTravel {
    /// Alternate direction. The angle changes at the shared end of the two passes.
    Zigzag,
    /// Every pass runs from low `u` to high `u`. The angle changes on the safe retract.
    OneWay,
}

/// Axis window and angular step for a [`OperationKind::RotaryFinish`].
///
/// Lengths are millimetres. `angle_step_deg` is degrees and is not scaled by the file units.
/// `None` on either field means the run fills it in: the mesh extent for `u`, and the stepover
/// or the scallop rule for the angle.
#[derive(Clone, Copy, Debug, PartialEq)]
struct RotaryFinishGeom {
    u_min: Option<f64>,
    u_max: Option<f64>,
    angle_step_deg: Option<f64>,
    travel: RotaryTravel,
}

/// Cylinder and flat-frame offsets for a [`OperationKind::Wrap`]. Lengths are millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
struct WrapGeom {
    diameter: f64,
    u_offset: f64,
    v_offset: f64,
}

/// Stock left by a roughing pass, in millimetres. Zero means cut to the contour and to `depth`.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct StockToLeave {
    wall: f64,
    floor: f64,
}

impl StockToLeave {
    /// Radial stock left on walls, in millimetres.
    pub fn wall(self) -> f64 {
        self.wall
    }

    /// Axial stock left above the floor, in millimetres.
    pub fn floor(self) -> f64 {
        self.floor
    }
}

/// Finishing pass after roughing. `step_down` is millimetres; `floor` is a pocket-only floor pass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Finish {
    step_down: f64,
    floor: bool,
}

impl Finish {
    /// Maximum thickness of one finishing pass, in millimetres.
    pub fn step_down(self) -> f64 {
        self.step_down
    }

    /// When true, a pocket also finishes the floor.
    pub fn floor(self) -> bool {
        self.floor
    }
}

/// One layer operation. Depths, heights and feeds are millimetres after loading.
///
/// `depth` and `step_down` are a positive distance downward. They must form a legal
/// `campanion-path` depth plan (at most 1024 passes, each within 1e6 mm). A chamfer omits
/// both in the file. The stored values are then the tip depth
/// `width / tan(v_angle / 2) + tip_clearance`, and both accessors return that distance.
/// Heights on [`Self::params`] are absolute Z. [`Self::entry`] is how each pass is entered.
/// An omitted entry is a plunge. [`Self::stock_to_leave`] is millimetres of material
/// left by roughing; an omitted [`Self::finish`] means no finishing pass.
/// [`Self::face_angle`] and [`Self::margin`] describe a face or a mesh operation.
/// A rough or a waterline stores its margin (the file value, or the tool radius when omitted).
/// Other kinds store 0. [`Self::width`], [`Self::v_angle`], [`Self::tip_diameter`] and
/// [`Self::tip_clearance`] describe a chamfer. Other kinds store 0.
/// [`Self::top_z`] and [`Self::grid`] describe a rough or a waterline. Other kinds store `None`.
/// [`Self::min_slope_deg`] describes a waterline. Other kinds store 0.
/// [`Self::dwell`] is the pause at the bottom of a drill, in seconds. Other kinds store `None`.
/// On a drill, [`Self::step_down`] is the peck depth: when it is less than [`Self::depth`] the
/// drill pecks, and otherwise the drill feeds to depth in one move.
/// [`Self::index_deg`] is the machine A/B angle for this operation, in degrees. `None` means the
/// file omitted it. A job with a rotary setup then runs the operation at 0°. A wrap rejects
/// `index_deg`. [`Self::cylinder_diameter`] is set only on a wrap. [`Self::u_offset`] and
/// [`Self::v_offset`] are 0 on every other kind.
/// [`Self::speed`] is the operation spindle speed in rpm. `None` means the file omitted
/// `operations[].speed`; [`Self::resolved_speed`] then uses the tool's speed.
/// [`Self::plane`] is the tilted work plane. `None` means the file omitted `plane`.
#[derive(Clone, Debug, PartialEq)]
pub struct Operation {
    name: String,
    kind: OperationKind,
    layers: Vec<String>,
    tool_id: u32,
    depth: f64,
    step_down: f64,
    /// Pause at the bottom of a drill, in seconds. `None` except on a drill that sets `dwell`.
    dwell: Option<f64>,
    stepover: Option<f64>,
    /// Flat-floor scallop height in millimetres. `None` except on a surface that sets it instead of `stepover`.
    scallop_height: Option<f64>,
    /// Spacing of the regular surface samples, in millimetres. `None` except on a surface.
    sample_step: Option<f64>,
    /// Tip floor for a surface, or the deepest roughing level, in millimetres.
    /// `None` means the mesh minimum Z. Other kinds store `None`.
    floor_z: Option<f64>,
    /// Top of the roughing stock, in millimetres. `None` means the mesh maximum Z.
    /// Other kinds store `None`.
    top_z: Option<f64>,
    /// Roughing or waterline sample grid, in millimetres. `None` except on those kinds.
    /// An omitted rough grid is `min(stepover / 4, 0.5 mm)`. An omitted waterline grid is
    /// `min(step_down, 0.25 mm)`.
    grid: Option<f64>,
    /// Minimum slope kept by a waterline, in degrees from horizontal. Other kinds store `0`.
    min_slope_deg: f64,
    face_angle: f64,
    margin: f64,
    width: f64,
    v_angle: f64,
    tip_diameter: f64,
    tip_clearance: f64,
    stock_to_leave: StockToLeave,
    finish: Option<Finish>,
    entry: DepthEntry,
    feeds: Feeds,
    params: ProcessParams,
    /// Machine A/B index in degrees. `None` when the file omits `index_deg`.
    index_deg: Option<f64>,
    /// Cylinder diameter and flat-frame offsets. `None` except on a wrap.
    wrap_geom: Option<WrapGeom>,
    /// Axis window, angle step and travel. `None` except on a rotary finish.
    rotary_geom: Option<RotaryFinishGeom>,
    /// Tilted work plane. `None` when the file omits `plane`.
    plane: Option<WorkPlane>,
    /// Spindle speed in rpm. `None` when the file omits `operations[].speed`.
    speed: Option<f64>,
}

/// Work plane for one indexed 3+2 operation.
///
/// `origin` is millimetres in the part frame. `normal` and `x_dir` are unit directions.
/// View +X is `x_dir`, view +Z is `normal`, and the view origin is `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorkPlane {
    origin: Vec3,
    normal: Vec3,
    x_dir: Vec3,
}

impl WorkPlane {
    /// Unit `normal` and an in-plane unit `x_dir`.
    pub(crate) fn new(origin: Vec3, normal: Vec3, x_dir: Vec3) -> Self {
        Self { origin, normal, x_dir }
    }

    /// Plane origin, millimetres, part frame.
    pub fn origin(self) -> Vec3 {
        self.origin
    }

    /// Unit plane normal. In the part frame this is the tool axis.
    pub fn normal(self) -> Vec3 {
        self.normal
    }

    /// Unit view +X, lying in the plane.
    pub fn x_dir(self) -> Vec3 {
        self.x_dir
    }
}

impl Operation {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> OperationKind {
        self.kind
    }

    /// Layer names in the source part. Unique within this operation, case-sensitive.
    pub fn layers(&self) -> &[String] {
        &self.layers
    }

    pub fn tool_id(&self) -> u32 {
        self.tool_id
    }

    /// Total depth downward from the stock surface, in millimetres.
    ///
    /// A chamfer has no file `depth`. This is the tip depth
    /// `width / tan(v_angle / 2) + tip_clearance`.
    pub fn depth(&self) -> f64 {
        self.depth
    }

    /// Maximum thickness of one depth pass, in millimetres.
    ///
    /// A chamfer cuts the tip depth in one pass, so this matches [`Self::depth`].
    /// On a drill this is the peck depth. When it is less than [`Self::depth`] the drill pecks,
    /// and otherwise the drill feeds to depth in one move.
    pub fn step_down(&self) -> f64 {
        self.step_down
    }

    /// Pause at the bottom of a drilled hole, in seconds.
    ///
    /// `None` when the file omits `dwell`, and on every kind other than a drill. Not scaled by
    /// the file's `units`.
    pub fn dwell(&self) -> Option<f64> {
        self.dwell
    }

    /// Chamfer width, in millimetres. The horizontal leg of the bevel. Other operations store `0`.
    pub fn width(&self) -> f64 {
        self.width
    }

    /// Included angle of the V-bit, in radians. Other operations store `0`.
    pub fn v_angle(&self) -> f64 {
        self.v_angle
    }

    /// Flat tip diameter of the V-bit, in millimetres. Other operations store `0`.
    pub fn tip_diameter(&self) -> f64 {
        self.tip_diameter
    }

    /// How far the V-bit tip sits below the bevel, in millimetres. Other operations store `0`.
    pub fn tip_clearance(&self) -> f64 {
        self.tip_clearance
    }

    /// Radial stepover in millimetres.
    ///
    /// `None` when a pocket omits it, and when a surface sets [`Self::scallop_height`] instead.
    /// A face always has one: the file value, or three quarters of the tool diameter when the
    /// file omits it. A surface that sets `stepover` stores that value.
    pub fn stepover(&self) -> Option<f64> {
        self.stepover
    }

    /// Flat-floor scallop height in millimetres.
    ///
    /// `Some` only on a surface that sets `scallop_height` instead of `stepover`. The run turns
    /// this into an XY stepover. Raster spacing is measured in XY, so on a slope the real cusp
    /// is larger than this flat-floor value for the same stepover.
    pub fn scallop_height(&self) -> Option<f64> {
        self.scallop_height
    }

    /// Facing direction in radians from +X. `0` or `π/2`. Other operations store `0`.
    pub fn face_angle(&self) -> f64 {
        self.face_angle
    }

    /// Extra material around the geometry bounding box, in millimetres, on every side.
    ///
    /// A face, a surface, a rough, or a waterline sets this. A rough or a waterline that omits
    /// it stores the tool radius. Other operations store `0`.
    pub fn margin(&self) -> f64 {
        self.margin
    }

    /// Maximum gap between regular surface samples, in millimetres.
    ///
    /// `None` except on a surface. An omitted file value is half of the stepover. When the
    /// surface sets [`Self::scallop_height`] and omits `sample_step`, this stays `None` and the
    /// run uses half of the stepover resolved from that height, after the tool-diameter cap.
    pub fn sample_step(&self) -> Option<f64> {
        self.sample_step
    }

    /// Surface tip floor, or the deepest roughing or waterline level, in millimetres.
    /// `None` means the mesh minimum Z when the job runs.
    pub fn floor_z(&self) -> Option<f64> {
        self.floor_z
    }

    /// Top of the roughing or waterline stock, in millimetres. `None` means the mesh maximum Z when the job runs.
    pub fn top_z(&self) -> Option<f64> {
        self.top_z
    }

    /// Roughing or waterline sample grid, in millimetres. `None` except on those kinds.
    pub fn grid(&self) -> Option<f64> {
        self.grid
    }

    /// Minimum slope kept by a waterline, in degrees from horizontal. Other kinds return `0`.
    pub fn min_slope_deg(&self) -> f64 {
        self.min_slope_deg
    }

    /// Stock left by roughing, in millimetres. Omitted fields are zero.
    pub fn stock_to_leave(&self) -> StockToLeave {
        self.stock_to_leave
    }

    /// Finishing pass. `None` when the file omits `finish`.
    pub fn finish(&self) -> Option<Finish> {
        self.finish
    }

    /// How the tool enters each depth pass. An omitted `entry` object is [`DepthEntry::Plunge`].
    pub fn entry(&self) -> DepthEntry {
        self.entry
    }

    /// Cut feed, and an optional plunge feed. `plunge` of `None` means "use the cut feed".
    pub fn feeds(&self) -> Feeds {
        self.feeds
    }

    /// Pierce and post delays, their toggles, and any pierce or cut height.
    pub fn params(&self) -> &ProcessParams {
        &self.params
    }

    /// Machine A/B index in degrees. `None` when the file omits `index_deg`.
    ///
    /// The number is not scaled by `units`. A job with a rotary setup and no index runs at 0°.
    pub fn index_deg(&self) -> Option<f64> {
        self.index_deg
    }

    /// Cylinder surface diameter in millimetres. `Some` only on a wrap.
    pub fn cylinder_diameter(&self) -> Option<f64> {
        self.wrap_geom.map(|geom| geom.diameter)
    }

    /// Added to drawing X to make the flat-frame `u`, in millimetres. `0` except on a wrap.
    pub fn u_offset(&self) -> f64 {
        match self.wrap_geom {
            Some(geom) => geom.u_offset,
            None => 0.0,
        }
    }

    /// Added to drawing Y to make the flat-frame `v`, in millimetres. `0` except on a wrap.
    pub fn v_offset(&self) -> f64 {
        match self.wrap_geom {
            Some(geom) => geom.v_offset,
            None => 0.0,
        }
    }

    /// Low end of a rotary finish along the axis, in millimetres. `None` except when the file sets `u_min`.
    pub fn u_min(&self) -> Option<f64> {
        self.rotary_geom.and_then(|geom| geom.u_min)
    }

    /// High end of a rotary finish along the axis, in millimetres. `None` except when the file sets `u_max`.
    pub fn u_max(&self) -> Option<f64> {
        self.rotary_geom.and_then(|geom| geom.u_max)
    }

    /// Rotary step in degrees. `None` except when a rotary finish sets `angle_step_deg`.
    ///
    /// Not scaled by `units`. When this and [`Self::stepover`] are both set, the run uses the stepover.
    pub fn angle_step_deg(&self) -> Option<f64> {
        self.rotary_geom.and_then(|geom| geom.angle_step_deg)
    }

    /// How a rotary finish joins passes. `None` on every other kind.
    pub fn rotary_travel(&self) -> Option<RotaryTravel> {
        self.rotary_geom.map(|geom| geom.travel)
    }

    /// Tilted work plane. `None` when the file omits `plane`.
    pub fn plane(&self) -> Option<WorkPlane> {
        self.plane
    }

    /// Spindle speed in rpm from the operation. `None` when the file omits `operations[].speed`.
    pub fn speed(&self) -> Option<f64> {
        self.speed
    }

    /// Operation speed if the file sets one, otherwise the tool's speed.
    pub fn resolved_speed(&self, tool: &JobTool) -> Option<f64> {
        self.speed.or(tool.speed())
    }

    /// [`Self::params`] with [`Self::resolved_speed`] copied onto `speed` when it is set.
    pub fn params_with_speed(&self, tool: &JobTool) -> Result<ProcessParams, campanion_ir::IrError> {
        match self.resolved_speed(tool) {
            Some(rpm) => self.params.clone().speed(Some(rpm)),
            None => Ok(self.params.clone()),
        }
    }
}

/// A loaded `campanion-job/1` file.
///
/// [`Self::clearance`] and [`Self::retract`] are millimetres. They come from the optional
/// job-level `heights` object. An omitted clearance is 5 mm and an omitted retract is 2 mm.
/// [`Self::rotary`] is the 4th-axis table, when the file has one.
/// [`Self::five_axis`] is the kinematics file and solution policy, when the file has one.
#[derive(Clone, Debug, PartialEq)]
pub struct JobFile {
    path: PathBuf,
    source: Source,
    units: Units,
    tolerance: Tolerance,
    tools: Vec<JobTool>,
    operations: Vec<Operation>,
    clearance: f64,
    retract: f64,
    rotary: Option<RotarySetup>,
    five_axis: Option<FiveAxisSetup>,
}

impl JobFile {
    /// Path passed to [`load`]. Not canonicalised.
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn format(&self) -> &str {
        FORMAT_TAG
    }

    pub fn source(&self) -> &Source {
        &self.source
    }

    /// Unit the file was written in. Numeric fields on this value are already millimetres.
    pub fn units(&self) -> Units {
        self.units
    }

    pub fn tolerance(&self) -> Tolerance {
        self.tolerance
    }

    /// Safe height in millimetres. The default is 5 mm when the file omits `heights.clearance`.
    /// An explicit value is converted from the file's `units`.
    pub fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Retract height in millimetres. The default is 2 mm when the file omits `heights.retract`.
    /// An explicit value is converted from the file's `units`. The loaded value is at most [`Self::clearance`].
    pub fn retract(&self) -> f64 {
        self.retract
    }

    pub fn tools(&self) -> &[JobTool] {
        &self.tools
    }

    pub fn operations(&self) -> &[Operation] {
        &self.operations
    }

    /// The tool with this id, if the file defines it.
    pub fn tool(&self, id: u32) -> Option<&JobTool> {
        self.tools.iter().find(|tool| tool.id() == id)
    }

    /// Fourth-axis table. `None` when the file omits `rotary`. Centre coordinates are millimetres.
    pub fn rotary(&self) -> Option<RotarySetup> {
        self.rotary
    }

    /// Five-axis kinematics and solution policy. `None` when the file omits `kinematics`.
    pub fn five_axis(&self) -> Option<&FiveAxisSetup> {
        self.five_axis.as_ref()
    }
}

struct Body {
    source: Source,
    units: Units,
    tolerance: Tolerance,
    tools: Vec<JobTool>,
    operations: Vec<Operation>,
    clearance: f64,
    retract: f64,
    rotary: Option<RotarySetup>,
    five_axis: Option<FiveAxisSetup>,
}

/// Read a `campanion-job/1` file. Lengths in the result are millimetres.
pub fn load(path: &Path) -> Result<JobFile, JobFileError> {
    let text = read_text(path)?;
    let body = parse_text(&text, path)?;
    Ok(JobFile {
        path: path.to_path_buf(),
        source: body.source,
        units: body.units,
        tolerance: body.tolerance,
        tools: body.tools,
        operations: body.operations,
        clearance: body.clearance,
        retract: body.retract,
        rotary: body.rotary,
        five_axis: body.five_axis,
    })
}

/// FNV-1a 64 of `bytes`, as 16 lowercase hex digits.
///
/// Change detection only. This is not a cryptographic hash and it is not an integrity check.
#[must_use]
pub fn hash_bytes(bytes: &[u8]) -> String {
    let mut hash = FNV_OFFSET;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    format!("{hash:016x}")
}

/// True when [`hash_bytes`] of `bytes` is not the hash stored on `job`.
#[must_use]
pub fn source_changed(job: &JobFile, bytes: &[u8]) -> bool {
    hash_bytes(bytes) != job.source.hash
}

/// Copy of `job` whose source hash is [`hash_bytes`] of `bytes`. The file on disk is not written.
#[must_use]
pub fn with_updated_hash(job: &JobFile, bytes: &[u8]) -> JobFile {
    let mut updated = job.clone();
    updated.source.hash = hash_bytes(bytes);
    updated
}

/// Job-file directory joined with the relative source path. `.` and `..` are folded lexically.
/// The filesystem is not consulted, so a missing part still yields a path.
#[must_use]
pub fn source_path(job: &JobFile) -> PathBuf {
    let joined = match job.path.parent() {
        Some(dir) => dir.join(&job.source.path),
        None => PathBuf::from(&job.source.path),
    };
    fold_dots(&joined)
}

fn read_text(path: &Path) -> Result<String, JobFileError> {
    let file = File::open(path).map_err(|err| JobFileError::io(path, err))?;
    if let Ok(meta) = file.metadata()
        && meta.len() > MAX_JSON_BYTES as u64
    {
        return Err(JobFileError::TooLarge { path: path.display().to_string() });
    }
    let mut bytes = Vec::new();
    file.take(MAX_JSON_BYTES as u64 + 1).read_to_end(&mut bytes).map_err(|err| JobFileError::io(path, err))?;
    if bytes.len() > MAX_JSON_BYTES {
        return Err(JobFileError::TooLarge { path: path.display().to_string() });
    }
    String::from_utf8(bytes).map_err(|_| JobFileError::invalid(format!("{} is not valid UTF-8", path.display())))
}

fn parse_text(text: &str, job_path: &Path) -> Result<Body, JobFileError> {
    let value: Value = serde_json::from_str(text).map_err(|err| JobFileError::Json { message: err.to_string() })?;
    let map = object(&value, "job").map_err(from_cli)?;
    let tag = match optional(map, "format") {
        Some(value) => as_str(value, "format tag").map_err(from_cli)?,
        None => return Err(JobFileError::invalid("missing format tag")),
    };
    check_format(tag)?;
    check_keys(map, &["format", "source", "units", "tolerance", "heights", "tools", "operations", "rotary", "kinematics", "solution"], "job")
        .map_err(from_cli)?;
    let units = parse_units(require(map, "units").map_err(from_cli)?)?;
    let scale = units.mm_per_unit();
    let source = parse_source(require(map, "source").map_err(from_cli)?)?;
    let tolerance = parse_tolerance(optional(map, "tolerance"), scale)?;
    let (clearance, retract) = parse_heights(optional(map, "heights"), scale)?;
    let rotary = parse_rotary(optional(map, "rotary"), scale)?;
    let kinematics = match optional(map, "kinematics") {
        Some(value) => Some(load_kinematics(job_path, value)?),
        None => None,
    };
    let solution_set = optional(map, "solution").is_some();
    let policy = parse_solution(optional(map, "solution"))?;
    if solution_set && kinematics.is_none() {
        return Err(JobFileError::invalid("solution needs a kinematics file"));
    }
    if kinematics.is_some() && rotary.is_some() {
        return Err(JobFileError::invalid("kinematics and rotary cannot be combined"));
    }
    let tools = parse_tools(require(map, "tools").map_err(from_cli)?, scale)?;
    let operations = parse_operations(require(map, "operations").map_err(from_cli)?, scale, &tools)?;
    if rotary.is_none()
        && let Some(operation) = operations.iter().find(|operation| operation.index_deg().is_some())
    {
        return Err(JobFileError::invalid(format!("operation `{}`: index_deg needs a rotary setup", operation.name())));
    }
    if rotary.is_none()
        && let Some(operation) = operations.iter().find(|operation| matches!(operation.kind(), OperationKind::Wrap))
    {
        return Err(JobFileError::invalid(format!("operation `{}`: wrap needs a rotary setup", operation.name())));
    }
    if rotary.is_none()
        && let Some(operation) = operations.iter().find(|operation| matches!(operation.kind(), OperationKind::RotaryFinish))
    {
        return Err(JobFileError::invalid(format!("operation `{}`: rotary finish needs a rotary setup", operation.name())));
    }
    if let Some(operation) = operations.iter().find(|operation| operation.plane().is_some())
        && kinematics.is_none()
    {
        return Err(JobFileError::invalid(format!("operation `{}`: plane needs a kinematics file", operation.name())));
    }
    if let Some(kinematics) = &kinematics {
        for operation in &operations {
            check_reachable(kinematics, policy, operation)?;
        }
    }
    check_mesh_source(&source, &operations, &tools)?;
    let five_axis = kinematics.map(|kinematics| FiveAxisSetup { kinematics, policy });
    Ok(Body { source, units, tolerance, tools, operations, clearance, retract, rotary, five_axis })
}

/// An STL part only accepts `surface`, `rough`, `waterline` and `rotary_finish`. Each only accepts an STL part.
/// A non-flat tool shape is rejected when a 2D operation uses that tool.
fn check_mesh_source(source: &Source, operations: &[Operation], tools: &[JobTool]) -> Result<(), JobFileError> {
    let stl = source.is_stl();
    for operation in operations {
        let mesh_op = is_mesh_op(operation.kind());
        if stl && !mesh_op {
            return Err(JobFileError::invalid("an STL source only allows surface, rough, waterline or rotary_finish operations"));
        }
        if matches!(operation.kind(), OperationKind::Surface) && !stl {
            return Err(JobFileError::invalid("surface is only valid on an STL source"));
        }
        if matches!(operation.kind(), OperationKind::Rough) && !stl {
            return Err(JobFileError::invalid("rough is only valid on an STL source"));
        }
        if matches!(operation.kind(), OperationKind::Waterline) && !stl {
            return Err(JobFileError::invalid("waterline is only valid on an STL source"));
        }
        if matches!(operation.kind(), OperationKind::RotaryFinish) && !stl {
            return Err(JobFileError::invalid("rotary finish is only valid on an STL source"));
        }
    }
    for tool in tools {
        if tool.shape().is_flat() {
            continue;
        }
        let used_on_flat_work = operations.iter().any(|operation| operation.tool_id() == tool.id() && !is_mesh_op(operation.kind()));
        if used_on_flat_work {
            return Err(JobFileError::invalid(format!(
                "tool {} shape is only valid on a surface, rough or waterline operation, or a rotary finish",
                tool.id()
            )));
        }
    }
    Ok(())
}

fn is_mesh_op(kind: OperationKind) -> bool {
    matches!(kind, OperationKind::Surface | OperationKind::Rough | OperationKind::Waterline | OperationKind::RotaryFinish)
}

fn check_format(tag: &str) -> Result<(), JobFileError> {
    let Some(rest) = tag.strip_prefix("campanion-job/") else {
        return Err(JobFileError::UnknownFormat { tag: tag.to_string() });
    };
    if rest.is_empty() || !rest.bytes().all(|byte| byte.is_ascii_digit()) || (rest.len() > 1 && rest.starts_with('0')) {
        return Err(JobFileError::UnknownFormat { tag: tag.to_string() });
    }
    let Ok(major) = rest.parse::<u32>() else {
        return Err(JobFileError::UnknownFormat { tag: tag.to_string() });
    };
    if major == FORMAT_MAJOR { Ok(()) } else { Err(JobFileError::UnsupportedVersion { found: major, supported: FORMAT_MAJOR }) }
}

fn parse_source(value: &Value) -> Result<Source, JobFileError> {
    let map = object(value, "source").map_err(from_cli)?;
    check_keys(map, &["path", "hash", "face"], "source").map_err(from_cli)?;
    let path_value = require(map, "path").map_err(from_cli)?;
    let Some(path) = path_value.as_str() else {
        return Err(JobFileError::invalid("source path must be a string"));
    };
    check_source_path(path)?;
    let hash_text = as_str(require(map, "hash").map_err(from_cli)?, "source hash").map_err(from_cli)?;
    // `face` is model millimetres, not a file-unit length, so it is not multiplied by the job scale.
    let face = parse_source_face(path, optional(map, "face"))?;
    Ok(Source { path: path.to_string(), hash: normalize_hash(hash_text)?, face })
}

fn parse_source_face(path: &str, face: Option<&Value>) -> Result<Option<[f64; 3]>, JobFileError> {
    if path_is_step(path) {
        let Some(value) = face else {
            return Err(JobFileError::invalid("a STEP source needs `face`: a point [x, y, z] in mm on the planar face to cut"));
        };
        return Ok(Some(parse_face(value)?));
    }
    if face.is_some() {
        return Err(JobFileError::invalid("`face` only applies to STEP sources"));
    }
    Ok(None)
}

fn parse_face(value: &Value) -> Result<[f64; 3], JobFileError> {
    let Some(list) = value.as_array() else {
        return Err(JobFileError::invalid("face must be an array of 3 numbers"));
    };
    if list.len() != 3 {
        return Err(JobFileError::invalid("face must be exactly 3 numbers"));
    }
    let mut point = [0.0; 3];
    for (slot, item) in point.iter_mut().zip(list.iter()) {
        let number = as_f64(item, "face").map_err(from_cli)?;
        if !number.is_finite() {
            return Err(JobFileError::invalid("face must be finite"));
        }
        if number.abs() > MAX_MM {
            return Err(JobFileError::invalid(format!("face is outside ±{MAX_MM} mm")));
        }
        *slot = number;
    }
    Ok(point)
}

fn check_source_path(text: &str) -> Result<(), JobFileError> {
    if text.is_empty() || text.len() > MAX_SOURCE_PATH {
        return Err(JobFileError::invalid(format!("source path must be 1 to {MAX_SOURCE_PATH} bytes")));
    }
    if text.bytes().any(|byte| byte < 0x20 || byte == 0x7f) {
        return Err(JobFileError::invalid("source path has a control character"));
    }
    let path = Path::new(text);
    if path.is_absolute() {
        return Err(JobFileError::invalid("source path must be relative to the job file"));
    }
    let mut named = false;
    for component in path.components() {
        match component {
            Component::Normal(_) => named = true,
            Component::CurDir | Component::ParentDir => {}
            Component::RootDir | Component::Prefix(_) => {
                return Err(JobFileError::invalid("source path must be relative to the job file"));
            }
        }
    }
    if !named {
        return Err(JobFileError::invalid("source path must name a file"));
    }
    Ok(())
}

fn normalize_hash(text: &str) -> Result<String, JobFileError> {
    if text.len() != 16 || !text.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(JobFileError::invalid("source hash must be 16 hex digits"));
    }
    Ok(text.to_ascii_lowercase())
}

fn parse_units(value: &Value) -> Result<Units, JobFileError> {
    match as_str(value, "units").map_err(from_cli)? {
        "mm" => Ok(Units::Millimetre),
        "inch" => Ok(Units::Inch),
        other => Err(JobFileError::invalid(format!("unknown units `{other}`"))),
    }
}

fn parse_tolerance(value: Option<&Value>, scale: f64) -> Result<Tolerance, JobFileError> {
    // Chord defaults are already millimetres. Only an explicit `chord` is multiplied by `scale`.
    let Some(value) = value else {
        return Tolerance::with_chord(Tolerance::DEFAULT_LINEAR_MM, Tolerance::DEFAULT_ANGULAR_RAD, DEFAULT_JOB_CHORD_MM)
            .map_err(|err| JobFileError::Invalid { message: err.to_string() });
    };
    let map = object(value, "tolerance").map_err(from_cli)?;
    check_keys(map, &["linear", "angular", "chord"], "tolerance").map_err(from_cli)?;
    let linear = match optional(map, "linear") {
        Some(value) => cap_under(scale_f64(value, scale, "linear tolerance")?, MAX_MM, "linear tolerance")?,
        None => Tolerance::DEFAULT_LINEAR_MM,
    };
    let angular = match optional(map, "angular") {
        Some(value) => cap_under(as_f64(value, "angular tolerance").map_err(from_cli)?, MAX_MM, "angular tolerance")?,
        None => Tolerance::DEFAULT_ANGULAR_RAD,
    };
    let chord = match optional(map, "chord") {
        Some(value) => cap_under(scale_f64(value, scale, "chord tolerance")?, MAX_MM, "chord tolerance")?,
        None => DEFAULT_JOB_CHORD_MM,
    };
    Tolerance::with_chord(linear, angular, chord).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

/// `heights` is optional. Each omitted field keeps its millimetre default, which is already in
/// internal units, so only an explicit number is multiplied by `scale`.
fn parse_heights(value: Option<&Value>, scale: f64) -> Result<(f64, f64), JobFileError> {
    let (mut clearance, mut retract) = (DEFAULT_CLEARANCE_MM, DEFAULT_RETRACT_MM);
    if let Some(value) = value {
        let map = object(value, "heights").map_err(from_cli)?;
        check_keys(map, &["clearance", "retract"], "heights").map_err(from_cli)?;
        if let Some(value) = optional(map, "clearance") {
            clearance = scale_f64(value, scale, "clearance")?;
        }
        if let Some(value) = optional(map, "retract") {
            retract = scale_f64(value, scale, "retract")?;
        }
    }
    clearance = finite_height(clearance, "clearance")?;
    retract = finite_height(retract, "retract")?;
    if retract > clearance {
        return Err(JobFileError::invalid("retract is above clearance"));
    }
    Ok((clearance, retract))
}

fn parse_tools(value: &Value, scale: f64) -> Result<Vec<JobTool>, JobFileError> {
    let list = array(value, "tools", MAX_TOOLS)?;
    let mut tools: Vec<JobTool> = Vec::with_capacity(list.len());
    for (index, item) in list.iter().enumerate() {
        let tool = parse_tool_value(item, scale).map_err(|err| at_index("tool", index, err))?;
        if tools.iter().any(|existing| existing.id() == tool.id()) {
            return Err(JobFileError::invalid(format!("tool {index}: tool id {} is repeated", tool.id())));
        }
        tools.push(tool);
    }
    Ok(tools)
}

fn parse_tool_value(value: &Value, scale: f64) -> Result<JobTool, JobFileError> {
    let map = object(value, "tool").map_err(from_cli)?;
    check_keys(map, &["id", "diameter", "kerf", "shape", "speed"], "tool").map_err(from_cli)?;
    let mut scaled = map.clone();
    scaled.remove("shape");
    scaled.remove("speed");
    scale_key(&mut scaled, "diameter", scale)?;
    scale_key(&mut scaled, "kerf", scale)?;
    let parsed = parse_tool(&Value::Object(scaled)).map_err(from_cli)?;
    parsed.path_tool().compensation_distance().map_err(|err| JobFileError::Invalid { message: err.to_string() })?;
    let shape = parse_tool_shape(optional(map, "shape"), parsed.ir().diameter(), scale)?;
    let speed = parse_rpm(optional(map, "speed"))?;
    Ok(JobTool { id: parsed.ir().id(), path_tool: parsed.path_tool(), ir: parsed.ir().clone(), shape, speed })
}

/// `"flat"`, `"ball"`, or `{"bull": corner_radius}`. The corner radius is a length in file units.
fn parse_tool_shape(value: Option<&Value>, diameter: Option<f64>, scale: f64) -> Result<ToolShape, JobFileError> {
    let Some(value) = value else {
        return Ok(ToolShape::Flat);
    };
    let Some(diameter) = diameter else {
        return Err(JobFileError::invalid("shape is only valid on a diameter tool"));
    };
    let radius = diameter * 0.5;
    if let Some(text) = value.as_str() {
        return match text {
            "flat" => Ok(ToolShape::Flat),
            "ball" => Ok(ToolShape::Ball),
            other => Err(JobFileError::invalid(format!("unknown tool shape `{other}`"))),
        };
    }
    if !value.is_object() {
        return Err(JobFileError::invalid("tool shape must be \"flat\", \"ball\" or {\"bull\": corner_radius}"));
    }
    let map = object(value, "shape").map_err(from_cli)?;
    check_keys(map, &["bull"], "shape").map_err(from_cli)?;
    let Some(corner_value) = optional(map, "bull") else {
        return Err(JobFileError::invalid("tool shape object needs a bull corner radius"));
    };
    let corner = positive_length(corner_value, scale, "bull corner radius")?;
    if corner >= radius {
        return Err(JobFileError::invalid("bull corner radius must be less than the tool radius"));
    }
    Ok(ToolShape::Bull { corner_radius: corner })
}

fn parse_operations(value: &Value, scale: f64, tools: &[JobTool]) -> Result<Vec<Operation>, JobFileError> {
    let list = array(value, "operations", MAX_OPERATIONS)?;
    let mut operations: Vec<Operation> = Vec::with_capacity(list.len());
    for (index, item) in list.iter().enumerate() {
        let operation = parse_operation(item, scale, tools).map_err(|err| at_index("operation", index, err))?;
        if operations.iter().any(|existing| existing.name() == operation.name()) {
            return Err(JobFileError::invalid(format!("operation {index}: name `{}` is repeated", operation.name())));
        }
        operations.push(operation);
    }
    Ok(operations)
}

fn parse_operation(value: &Value, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    let map = object(value, "operation").map_err(from_cli)?;
    check_keys(
        map,
        &[
            "name",
            "type",
            "layers",
            "side",
            "tool_id",
            "depth",
            "dwell",
            "step_down",
            "stepover",
            "scallop_height",
            "sample_step",
            "angle_deg",
            "margin",
            "floor_z",
            "top_z",
            "grid",
            "min_slope_deg",
            "width",
            "v_angle_deg",
            "tip_diameter",
            "tip_clearance",
            "stock_to_leave",
            "finish",
            "entry",
            "feeds",
            "cut_height",
            "pierce_height",
            "timing",
            "index_deg",
            "diameter",
            "u_offset",
            "v_offset",
            "u_min",
            "u_max",
            "angle_step_deg",
            "direction",
            "plane",
            "speed",
        ],
        "operation",
    )
    .map_err(from_cli)?;
    let name_value = require(map, "name").map_err(from_cli)?;
    let name = required_name(as_str(name_value, "operation name").map_err(from_cli)?, "operation name").map_err(from_cli)?;
    let index_deg = parse_index_deg(optional(map, "index_deg"))?;
    let speed = parse_rpm(optional(map, "speed"))?;
    let kind = parse_kind(map)?;
    let dwell = parse_dwell(optional(map, "dwell"), kind)?;
    let plane = take_plane(&name, map, &kind, index_deg.is_some(), scale)?;
    if matches!(kind, OperationKind::Surface) {
        return Ok(attach_plane(with_speed(with_index(finish_surface(name, map, scale, tools)?, index_deg), speed), plane));
    }
    if matches!(kind, OperationKind::Rough) {
        return Ok(attach_plane(with_speed(with_index(finish_rough(name, map, scale, tools)?, index_deg), speed), plane));
    }
    if matches!(kind, OperationKind::Waterline) {
        return Ok(attach_plane(with_speed(with_index(finish_waterline(name, map, scale, tools)?, index_deg), speed), plane));
    }
    if matches!(kind, OperationKind::Wrap) {
        return Ok(attach_plane(with_speed(finish_wrap(name, map, scale, tools)?, speed), plane));
    }
    if matches!(kind, OperationKind::RotaryFinish) {
        return Ok(attach_plane(with_speed(finish_rotary(name, map, scale, tools)?, speed), plane));
    }
    if optional(map, "sample_step").is_some() {
        return Err(JobFileError::invalid("sample_step is only valid on a surface"));
    }
    if optional(map, "scallop_height").is_some() {
        return Err(JobFileError::invalid("scallop_height is only valid on a surface"));
    }
    if optional(map, "floor_z").is_some() {
        return Err(JobFileError::invalid("floor_z is only valid on a surface, a rough or a waterline"));
    }
    if optional(map, "top_z").is_some() {
        return Err(JobFileError::invalid("top_z is only valid on a rough or a waterline"));
    }
    if optional(map, "grid").is_some() {
        return Err(JobFileError::invalid("grid is only valid on a rough or a waterline"));
    }
    if optional(map, "min_slope_deg").is_some() {
        return Err(JobFileError::invalid("min_slope_deg is only valid on a waterline"));
    }
    if optional(map, "diameter").is_some() {
        return Err(JobFileError::invalid("diameter is only valid on a wrap"));
    }
    if optional(map, "u_offset").is_some() {
        return Err(JobFileError::invalid("u_offset is only valid on a wrap"));
    }
    if optional(map, "v_offset").is_some() {
        return Err(JobFileError::invalid("v_offset is only valid on a wrap"));
    }
    for key in ["u_min", "u_max", "angle_step_deg", "direction"] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is only valid on a rotary finish")));
        }
    }
    reject_chamfer_cut_keys(map, kind)?;
    let layers = parse_layers(require(map, "layers").map_err(from_cli)?)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    let chamfer = parse_chamfer(map, kind, tool, scale)?;
    let (depth, step_down) = parse_depth_and_step(map, kind, scale, chamfer.tip_depth)?;
    let stepover = parse_stepover(optional(map, "stepover"), kind, tool, scale)?;
    let face_angle = parse_face_angle(optional(map, "angle_deg"), kind)?;
    let margin = parse_margin(optional(map, "margin"), kind, scale)?;
    let stock_to_leave = parse_stock_to_leave(optional(map, "stock_to_leave"), kind, depth, scale)?;
    let finish = parse_finish(optional(map, "finish"), kind, depth, scale)?;
    let entry = parse_entry(optional(map, "entry"), scale)?;
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_timing(map, scale)?;
    Ok(Operation {
        name,
        kind,
        layers,
        tool_id,
        depth,
        step_down,
        dwell,
        stepover,
        scallop_height: None,
        sample_step: None,
        floor_z: None,
        top_z: None,
        grid: None,
        min_slope_deg: 0.0,
        face_angle,
        margin,
        width: chamfer.width,
        v_angle: chamfer.v_angle,
        tip_diameter: chamfer.tip_diameter,
        tip_clearance: chamfer.tip_clearance,
        stock_to_leave,
        finish,
        entry,
        feeds,
        params,
        index_deg,
        wrap_geom: None,
        rotary_geom: None,
        plane,
        speed,
    })
}

/// Keys a surface does not accept. `side` is already rejected by [`parse_kind`].
fn reject_surface_foreign_keys(map: &Map<String, Value>) -> Result<(), JobFileError> {
    for key in [
        "layers",
        "depth",
        "step_down",
        "top_z",
        "grid",
        "min_slope_deg",
        "width",
        "v_angle_deg",
        "tip_diameter",
        "tip_clearance",
        "finish",
        "entry",
        "cut_height",
        "pierce_height",
        "timing",
        "diameter",
        "u_offset",
        "v_offset",
        "u_min",
        "u_max",
        "angle_step_deg",
        "direction",
    ] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is not valid on a surface")));
        }
    }
    Ok(())
}

/// Exactly one of `stepover` and `scallop_height`. Lengths are millimetres after scaling.
///
/// An explicit stepover larger than the tool diameter is rejected. A scallop height is stored
/// as given; the run derives the stepover and caps it at the diameter.
fn surface_spacing(
    stepover: Option<&Value>,
    scallop_height: Option<&Value>,
    diameter: f64,
    scale: f64,
) -> Result<(Option<f64>, Option<f64>), JobFileError> {
    match (stepover, scallop_height) {
        (None, None) => Err(JobFileError::invalid("surface needs a stepover or a scallop_height")),
        (Some(_), Some(_)) => Err(JobFileError::invalid("surface takes stepover or scallop_height, not both")),
        (Some(value), None) => {
            let stepover = positive_length(value, scale, "stepover")?;
            if stepover > diameter {
                return Err(JobFileError::invalid("stepover is larger than the tool diameter"));
            }
            Ok((Some(stepover), None))
        }
        (None, Some(value)) => Ok((None, Some(positive_length(value, scale, "scallop_height")?))),
    }
}

/// Parallel finish. Depth stays 0: the mesh and `floor_z` decide the tip height.
fn finish_surface(name: String, map: &Map<String, Value>, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    reject_surface_foreign_keys(map)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("surface is not valid with a kerf tool"));
    };
    let (stepover, scallop_height) = surface_spacing(optional(map, "stepover"), optional(map, "scallop_height"), diameter, scale)?;
    let sample_step = match optional(map, "sample_step") {
        Some(value) => Some(positive_length(value, scale, "sample_step")?),
        // Half of a known stepover. A scallop height is resolved when the job runs, and the
        // sample gap is then half of that capped stepover.
        None => stepover.map(|stepover| stepover * 0.5),
    };
    let face_angle = parse_face_angle(optional(map, "angle_deg"), OperationKind::Surface)?;
    let margin = parse_margin(optional(map, "margin"), OperationKind::Surface, scale)?;
    let floor_z = match optional(map, "floor_z") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "floor_z")?, "floor_z")?),
        None => None,
    };
    let stock_to_leave = parse_stock_to_leave(optional(map, "stock_to_leave"), OperationKind::Surface, 0.0, scale)?;
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_params(None).map_err(from_cli)?;
    Ok(Operation {
        name,
        kind: OperationKind::Surface,
        layers: Vec::new(),
        tool_id,
        depth: 0.0,
        step_down: 0.0,
        dwell: None,
        stepover,
        scallop_height,
        sample_step,
        floor_z,
        top_z: None,
        grid: None,
        min_slope_deg: 0.0,
        face_angle,
        margin,
        width: 0.0,
        v_angle: 0.0,
        tip_diameter: 0.0,
        tip_clearance: 0.0,
        stock_to_leave,
        finish: None,
        entry: DepthEntry::Plunge,
        feeds,
        params,
        index_deg: None,
        wrap_geom: None,
        rotary_geom: None,
        plane: None,
        speed: None,
    })
}

/// Keys a rough does not accept. `side` is already rejected by [`parse_kind`].
fn reject_rough_foreign_keys(map: &Map<String, Value>) -> Result<(), JobFileError> {
    for key in [
        "layers",
        "depth",
        "scallop_height",
        "sample_step",
        "angle_deg",
        "min_slope_deg",
        "width",
        "v_angle_deg",
        "tip_diameter",
        "tip_clearance",
        "finish",
        "cut_height",
        "pierce_height",
        "timing",
        "diameter",
        "u_offset",
        "v_offset",
        "u_min",
        "u_max",
        "angle_step_deg",
        "direction",
    ] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is not valid on a rough")));
        }
    }
    Ok(())
}

/// Z-level rough. `depth` stays 0: `top_z` and `floor_z` are absolute and default to the mesh.
/// `entry` uses the same parser as a 2D operation. Omitted, it is a plunge.
fn finish_rough(name: String, map: &Map<String, Value>, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    reject_rough_foreign_keys(map)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("rough is not valid with a kerf tool"));
    };
    let stepover = match optional(map, "stepover") {
        Some(value) => {
            let stepover = positive_length(value, scale, "stepover")?;
            if stepover > diameter {
                return Err(JobFileError::invalid("stepover is larger than the tool diameter"));
            }
            stepover
        }
        None => diameter * ROUGH_STEPOVER_OF_DIAMETER,
    };
    let Some(step_value) = optional(map, "step_down") else {
        return Err(JobFileError::invalid("rough needs a step_down"));
    };
    let step_down = positive_length(step_value, scale, "step_down")?;
    let margin = match optional(map, "margin") {
        Some(value) => cap_under(non_negative_length(value, scale, "margin")?, MAX_MM, "margin")?,
        None => diameter * 0.5,
    };
    let floor_z = match optional(map, "floor_z") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "floor_z")?, "floor_z")?),
        None => None,
    };
    let top_z = match optional(map, "top_z") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "top_z")?, "top_z")?),
        None => None,
    };
    if let (Some(top), Some(floor)) = (top_z, floor_z)
        && top <= floor
    {
        return Err(JobFileError::invalid("top_z must be above floor_z"));
    }
    let grid = match optional(map, "grid") {
        Some(value) => positive_length(value, scale, "grid")?,
        None => (stepover * 0.25).min(DEFAULT_ROUGH_GRID_MM),
    };
    let stock = match optional(map, "stock_to_leave") {
        Some(value) => {
            if !value.is_number() {
                return Err(JobFileError::invalid("rough stock_to_leave must be a number"));
            }
            cap_under(non_negative_length(value, scale, "stock_to_leave")?, MAX_MM, "stock_to_leave")?
        }
        None => DEFAULT_ROUGH_STOCK_MM,
    };
    let entry = parse_entry(optional(map, "entry"), scale)?;
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_params(None).map_err(from_cli)?;
    Ok(Operation {
        name,
        kind: OperationKind::Rough,
        layers: Vec::new(),
        tool_id,
        depth: 0.0,
        step_down,
        dwell: None,
        stepover: Some(stepover),
        scallop_height: None,
        sample_step: None,
        floor_z,
        top_z,
        grid: Some(grid),
        min_slope_deg: 0.0,
        face_angle: 0.0,
        margin,
        width: 0.0,
        v_angle: 0.0,
        tip_diameter: 0.0,
        tip_clearance: 0.0,
        stock_to_leave: StockToLeave { wall: 0.0, floor: stock },
        finish: None,
        entry,
        feeds,
        params,
        index_deg: None,
        wrap_geom: None,
        rotary_geom: None,
        plane: None,
        speed: None,
    })
}

/// Keys a waterline does not accept. `side` is already rejected by [`parse_kind`].
fn reject_waterline_foreign_keys(map: &Map<String, Value>) -> Result<(), JobFileError> {
    for key in [
        "layers",
        "depth",
        "stepover",
        "scallop_height",
        "sample_step",
        "angle_deg",
        "width",
        "v_angle_deg",
        "tip_diameter",
        "tip_clearance",
        "finish",
        "entry",
        "cut_height",
        "pierce_height",
        "timing",
        "diameter",
        "u_offset",
        "v_offset",
        "u_min",
        "u_max",
        "angle_step_deg",
        "direction",
    ] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is not valid on a waterline")));
        }
    }
    Ok(())
}

/// Steep-wall waterline. `depth` stays 0. Axial stock defaults to 0 and is added to cut Z at run time.
fn finish_waterline(name: String, map: &Map<String, Value>, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    reject_waterline_foreign_keys(map)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("waterline is not valid with a kerf tool"));
    };
    let Some(step_value) = optional(map, "step_down") else {
        return Err(JobFileError::invalid("waterline needs a step_down"));
    };
    let step_down = positive_length(step_value, scale, "step_down")?;
    let margin = match optional(map, "margin") {
        Some(value) => cap_under(non_negative_length(value, scale, "margin")?, MAX_MM, "margin")?,
        None => diameter * 0.5,
    };
    let floor_z = match optional(map, "floor_z") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "floor_z")?, "floor_z")?),
        None => None,
    };
    let top_z = match optional(map, "top_z") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "top_z")?, "top_z")?),
        None => None,
    };
    if let (Some(top), Some(floor)) = (top_z, floor_z)
        && top <= floor
    {
        return Err(JobFileError::invalid("top_z must be above floor_z"));
    }
    let grid = match optional(map, "grid") {
        Some(value) => positive_length(value, scale, "grid")?,
        None => step_down.min(DEFAULT_WATERLINE_GRID_MM),
    };
    let min_slope_deg = match optional(map, "min_slope_deg") {
        Some(value) => {
            let degrees = as_f64(value, "min_slope_deg").map_err(from_cli)?;
            if !(0.0..=90.0).contains(&degrees) {
                return Err(JobFileError::invalid("min_slope_deg must lie in [0, 90]"));
            }
            degrees
        }
        None => 0.0,
    };
    let stock = match optional(map, "stock_to_leave") {
        Some(value) => {
            if !value.is_number() {
                return Err(JobFileError::invalid("waterline stock_to_leave must be a number"));
            }
            cap_under(non_negative_length(value, scale, "stock_to_leave")?, MAX_MM, "stock_to_leave")?
        }
        None => 0.0,
    };
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_params(None).map_err(from_cli)?;
    Ok(Operation {
        name,
        kind: OperationKind::Waterline,
        layers: Vec::new(),
        tool_id,
        depth: 0.0,
        step_down,
        dwell: None,
        stepover: None,
        scallop_height: None,
        sample_step: None,
        floor_z,
        top_z,
        grid: Some(grid),
        min_slope_deg,
        face_angle: 0.0,
        margin,
        width: 0.0,
        v_angle: 0.0,
        tip_diameter: 0.0,
        tip_clearance: 0.0,
        stock_to_leave: StockToLeave { wall: 0.0, floor: stock },
        finish: None,
        entry: DepthEntry::Plunge,
        feeds,
        params,
        index_deg: None,
        wrap_geom: None,
        rotary_geom: None,
        plane: None,
        speed: None,
    })
}

/// Keys a wrap does not accept. `side` is already rejected by [`parse_kind`].
fn reject_wrap_foreign_keys(map: &Map<String, Value>) -> Result<(), JobFileError> {
    for key in [
        "stepover",
        "scallop_height",
        "sample_step",
        "angle_deg",
        "margin",
        "floor_z",
        "top_z",
        "grid",
        "min_slope_deg",
        "width",
        "v_angle_deg",
        "tip_diameter",
        "tip_clearance",
        "stock_to_leave",
        "finish",
        "entry",
        "cut_height",
        "pierce_height",
        "timing",
        "index_deg",
        "u_min",
        "u_max",
        "angle_step_deg",
        "direction",
    ] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is not valid on a wrap")));
        }
    }
    Ok(())
}

/// Wrap onto a cylinder. `depth` is below the surface. Offsets default to 0 and may be negative.
fn finish_wrap(name: String, map: &Map<String, Value>, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    reject_wrap_foreign_keys(map)?;
    let layers = parse_layers(require(map, "layers").map_err(from_cli)?)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    if !tools.iter().any(|tool| tool.id() == tool_id) {
        return Err(JobFileError::invalid(format!("unknown tool {tool_id}")));
    }
    let (depth, step_down) = parse_depth_and_step(map, OperationKind::Wrap, scale, None)?;
    let Some(diameter_value) = optional(map, "diameter") else {
        return Err(JobFileError::invalid("wrap needs a diameter"));
    };
    let diameter = positive_length(diameter_value, scale, "diameter")?;
    let u_offset = match optional(map, "u_offset") {
        Some(value) => finite_height(scale_f64(value, scale, "u_offset")?, "u_offset")?,
        None => 0.0,
    };
    let v_offset = match optional(map, "v_offset") {
        Some(value) => finite_height(scale_f64(value, scale, "v_offset")?, "v_offset")?,
        None => 0.0,
    };
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_params(None).map_err(from_cli)?;
    Ok(Operation {
        name,
        kind: OperationKind::Wrap,
        layers,
        tool_id,
        depth,
        step_down,
        dwell: None,
        stepover: None,
        scallop_height: None,
        sample_step: None,
        floor_z: None,
        top_z: None,
        grid: None,
        min_slope_deg: 0.0,
        face_angle: 0.0,
        margin: 0.0,
        width: 0.0,
        v_angle: 0.0,
        tip_diameter: 0.0,
        tip_clearance: 0.0,
        stock_to_leave: StockToLeave::default(),
        finish: None,
        entry: DepthEntry::Plunge,
        feeds,
        params,
        index_deg: None,
        wrap_geom: Some(WrapGeom { diameter, u_offset, v_offset }),
        rotary_geom: None,
        plane: None,
        speed: None,
    })
}

/// Keys a rotary finish does not accept. `side` is already rejected by [`parse_kind`].
fn reject_rotary_foreign_keys(map: &Map<String, Value>) -> Result<(), JobFileError> {
    for key in [
        "layers",
        "depth",
        "step_down",
        "scallop_height",
        "angle_deg",
        "margin",
        "floor_z",
        "top_z",
        "grid",
        "min_slope_deg",
        "width",
        "v_angle_deg",
        "tip_diameter",
        "tip_clearance",
        "stock_to_leave",
        "finish",
        "entry",
        "cut_height",
        "pierce_height",
        "timing",
        "index_deg",
        "diameter",
        "u_offset",
        "v_offset",
    ] {
        if optional(map, key).is_some() {
            return Err(JobFileError::invalid(format!("{key} is not valid on a rotary finish")));
        }
    }
    Ok(())
}

/// Continuous rotary finish. `u_step` stays `None` when the file omits `sample_step`.
fn finish_rotary(name: String, map: &Map<String, Value>, scale: f64, tools: &[JobTool]) -> Result<Operation, JobFileError> {
    reject_rotary_foreign_keys(map)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    if tool.diameter().is_none() {
        return Err(JobFileError::invalid("rotary finish is not valid with a kerf tool"));
    }
    let stepover = match optional(map, "stepover") {
        Some(value) => Some(positive_length(value, scale, "stepover")?),
        None => None,
    };
    let sample_step = match optional(map, "sample_step") {
        Some(value) => Some(positive_length(value, scale, "sample_step")?),
        None => None,
    };
    let u_min = match optional(map, "u_min") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "u_min")?, "u_min")?),
        None => None,
    };
    let u_max = match optional(map, "u_max") {
        Some(value) => Some(finite_height(scale_f64(value, scale, "u_max")?, "u_max")?),
        None => None,
    };
    if let (Some(low), Some(high)) = (u_min, u_max)
        && low > high
    {
        return Err(JobFileError::invalid("u_min is greater than u_max"));
    }
    let angle_step_deg = match optional(map, "angle_step_deg") {
        Some(value) => {
            let degrees = as_f64(value, "angle_step_deg").map_err(from_cli)?;
            if !(degrees.is_finite() && degrees > 0.0 && degrees <= 360.0) {
                return Err(JobFileError::invalid("angle_step_deg must be greater than 0 and at most 360"));
            }
            Some(degrees)
        }
        None => None,
    };
    let travel = match optional(map, "direction") {
        None => RotaryTravel::Zigzag,
        Some(value) => match as_str(value, "direction").map_err(from_cli)? {
            "zigzag" => RotaryTravel::Zigzag,
            "one_way" => RotaryTravel::OneWay,
            _ => return Err(JobFileError::invalid("direction must be \"zigzag\" or \"one_way\"")),
        },
    };
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_params(None).map_err(from_cli)?;
    Ok(Operation {
        name,
        kind: OperationKind::RotaryFinish,
        layers: Vec::new(),
        tool_id,
        depth: 0.0,
        step_down: 0.0,
        dwell: None,
        stepover,
        scallop_height: None,
        sample_step,
        floor_z: None,
        top_z: None,
        grid: None,
        min_slope_deg: 0.0,
        face_angle: 0.0,
        margin: 0.0,
        width: 0.0,
        v_angle: 0.0,
        tip_diameter: 0.0,
        tip_clearance: 0.0,
        stock_to_leave: StockToLeave::default(),
        finish: None,
        entry: DepthEntry::Plunge,
        feeds,
        params,
        index_deg: None,
        wrap_geom: None,
        rotary_geom: Some(RotaryFinishGeom { u_min, u_max, angle_step_deg, travel }),
        plane: None,
        speed: None,
    })
}

fn with_index(mut operation: Operation, index_deg: Option<f64>) -> Operation {
    operation.index_deg = index_deg;
    operation
}

fn with_speed(mut operation: Operation, speed: Option<f64>) -> Operation {
    operation.speed = speed;
    operation
}

/// Spindle rpm. Omitted is `None`. The number is not scaled by `units`.
fn parse_rpm(value: Option<&Value>) -> Result<Option<f64>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    let rpm = as_f64(value, "speed").map_err(from_cli)?;
    if !(rpm > 0.0 && rpm <= MAX_SPINDLE_RPM) {
        return Err(JobFileError::invalid("speed must be positive and at most 100000"));
    }
    Ok(Some(rpm))
}

/// `index_deg` is a finite number of degrees. It is not scaled by `units`.
fn parse_index_deg(value: Option<&Value>) -> Result<Option<f64>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    Ok(Some(as_f64(value, "index_deg").map_err(from_cli)?))
}

/// Fourth-axis table. Lengths in `center` are scaled. Angles are degrees and are not scaled.
fn parse_rotary(value: Option<&Value>, scale: f64) -> Result<Option<RotarySetup>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    let map = object(value, "rotary").map_err(from_cli)?;
    check_keys(map, &["axis", "center", "direction", "limits"], "rotary").map_err(from_cli)?;
    let axis_text = as_str(require(map, "axis").map_err(from_cli)?, "rotary axis").map_err(from_cli)?;
    let axis = match axis_text {
        "a" => RotaryAxisKind::A,
        "b" => RotaryAxisKind::B,
        _ => return Err(JobFileError::invalid("rotary axis must be \"a\" or \"b\"")),
    };
    let direction_text = as_str(require(map, "direction").map_err(from_cli)?, "rotary direction").map_err(from_cli)?;
    let direction = match direction_text {
        "positive" => RotaryDirection::Positive,
        "negative" => RotaryDirection::Negative,
        _ => return Err(JobFileError::invalid("rotary direction must be \"positive\" or \"negative\"")),
    };
    let center = parse_rotary_center(require(map, "center").map_err(from_cli)?, scale)?;
    let limits = match optional(map, "limits") {
        Some(value) => Some(parse_rotary_limits(value)?),
        None => None,
    };
    RotarySetup::new(axis, center, direction, limits).map(Some).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

fn parse_rotary_center(value: &Value, scale: f64) -> Result<Vec3, JobFileError> {
    let list = array(value, "rotary center", 3)?;
    if list.len() != 3 {
        return Err(JobFileError::invalid("rotary center must be exactly 3 numbers"));
    }
    let mut coords = [0.0; 3];
    for (slot, item) in coords.iter_mut().zip(list) {
        *slot = finite_height(scale_f64(item, scale, "rotary center")?, "rotary center")?;
    }
    Vec3::new(coords[0], coords[1], coords[2]).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

fn parse_rotary_limits(value: &Value) -> Result<RotaryLimits, JobFileError> {
    let map = object(value, "rotary limits").map_err(from_cli)?;
    check_keys(map, &["min_deg", "max_deg"], "rotary limits").map_err(from_cli)?;
    let min_deg = as_f64(require(map, "min_deg").map_err(from_cli)?, "rotary limit minimum").map_err(from_cli)?;
    let max_deg = as_f64(require(map, "max_deg").map_err(from_cli)?, "rotary limit maximum").map_err(from_cli)?;
    RotaryLimits::new(min_deg, max_deg).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

/// Chamfer numbers after scaling. `tip_depth` is `width / tan(v_angle / 2) + tip_clearance`.
/// Other kinds store zeros and a `tip_depth` of `None`.
struct ChamferFields {
    width: f64,
    v_angle: f64,
    tip_diameter: f64,
    tip_clearance: f64,
    tip_depth: Option<f64>,
}

fn parse_chamfer(map: &Map<String, Value>, kind: OperationKind, tool: &JobTool, scale: f64) -> Result<ChamferFields, JobFileError> {
    let width = optional(map, "width");
    let angle = optional(map, "v_angle_deg");
    let tip = optional(map, "tip_diameter");
    let clearance = optional(map, "tip_clearance");
    if !matches!(kind, OperationKind::Chamfer { .. }) {
        if width.is_some() {
            return Err(JobFileError::invalid("width is only valid on a chamfer"));
        }
        if angle.is_some() {
            return Err(JobFileError::invalid("v_angle_deg is only valid on a chamfer"));
        }
        if tip.is_some() {
            return Err(JobFileError::invalid("tip_diameter is only valid on a chamfer"));
        }
        if clearance.is_some() {
            return Err(JobFileError::invalid("tip_clearance is only valid on a chamfer"));
        }
        return Ok(ChamferFields { width: 0.0, v_angle: 0.0, tip_diameter: 0.0, tip_clearance: 0.0, tip_depth: None });
    }
    let Some(width_value) = width else {
        return Err(JobFileError::invalid("chamfer needs a width"));
    };
    let width = positive_length(width_value, scale, "width")?;
    let Some(angle_value) = angle else {
        return Err(JobFileError::invalid("chamfer needs a v_angle_deg"));
    };
    let degrees = as_f64(angle_value, "v_angle_deg").map_err(from_cli)?;
    if !(degrees > 0.0 && degrees < 180.0) {
        return Err(JobFileError::invalid("v_angle_deg must be greater than 0 and less than 180"));
    }
    let v_angle = degrees.to_radians();
    let tip_diameter = match tip {
        Some(value) => non_negative_length(value, scale, "tip_diameter")?,
        None => 0.0,
    };
    let tip_diameter = cap_under(tip_diameter, MAX_MM, "tip_diameter")?;
    let tip_clearance = match clearance {
        Some(value) => non_negative_length(value, scale, "tip_clearance")?,
        None => 0.0,
    };
    let tip_clearance = cap_under(tip_clearance, MAX_MM, "tip_clearance")?;
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("chamfer is not valid with a kerf tool"));
    };
    if tip_diameter >= diameter {
        return Err(JobFileError::invalid("tip_diameter must be less than the tool diameter"));
    }
    let tip_depth = chamfer_tip_depth(width, v_angle, tip_clearance)?;
    Ok(ChamferFields { width, v_angle, tip_diameter, tip_clearance, tip_depth: Some(tip_depth) })
}

/// `width / tan(a / 2) + clearance`, in millimetres. `angle_rad` is the included angle.
fn chamfer_tip_depth(width: f64, angle_rad: f64, clearance: f64) -> Result<f64, JobFileError> {
    let tan_half = (angle_rad * 0.5).tan();
    if !tan_half.is_finite() || tan_half <= 0.0 {
        return Err(JobFileError::invalid("v_angle_deg does not produce a finite tip depth"));
    }
    let depth = width / tan_half + clearance;
    if !depth.is_finite() || !(depth > 0.0) {
        return Err(JobFileError::invalid("chamfer tip depth must be finite and positive"));
    }
    cap_under(depth, MAX_MM, "chamfer tip depth")
}

/// A chamfer takes its depth from the V-bit geometry, so a file `depth` or `step_down` is an error.
fn reject_chamfer_cut_keys(map: &Map<String, Value>, kind: OperationKind) -> Result<(), JobFileError> {
    if !matches!(kind, OperationKind::Chamfer { .. }) {
        return Ok(());
    }
    if optional(map, "depth").is_some() {
        return Err(JobFileError::invalid("depth is not valid on a chamfer; the depth follows from the geometry"));
    }
    if optional(map, "step_down").is_some() {
        return Err(JobFileError::invalid("step_down is not valid on a chamfer; the depth follows from the geometry"));
    }
    Ok(())
}

/// File `depth` and `step_down` for every kind except chamfer. A chamfer stores the derived
/// tip depth as a single pass. [`reject_chamfer_cut_keys`] has already rejected those keys.
fn parse_depth_and_step(map: &Map<String, Value>, kind: OperationKind, scale: f64, tip_depth: Option<f64>) -> Result<(f64, f64), JobFileError> {
    if matches!(kind, OperationKind::Chamfer { .. }) {
        reject_chamfer_cut_keys(map, kind)?;
        let Some(depth) = tip_depth else {
            return Err(JobFileError::invalid("chamfer tip depth is missing"));
        };
        check_depth(depth, depth)?;
        return Ok((depth, depth));
    }
    let depth = scale_f64(require(map, "depth").map_err(from_cli)?, scale, "depth")?;
    let step_down = scale_f64(require(map, "step_down").map_err(from_cli)?, scale, "step_down")?;
    check_depth(depth, step_down)?;
    Ok((depth, step_down))
}

fn positive_length(value: &Value, scale: f64, what: &str) -> Result<f64, JobFileError> {
    let length = scale_f64(value, scale, what)?;
    if !(length > 0.0) {
        return Err(JobFileError::invalid(format!("{what} must be greater than 0")));
    }
    cap_under(length, MAX_MM, what)
}

fn parse_stepover(value: Option<&Value>, kind: OperationKind, tool: &JobTool, scale: f64) -> Result<Option<f64>, JobFileError> {
    if matches!(kind, OperationKind::Face) && tool.diameter().is_none() {
        return Err(JobFileError::invalid("face is not valid with a kerf tool"));
    }
    let Some(value) = value else {
        return default_stepover(kind, tool);
    };
    if !matches!(kind, OperationKind::Pocket | OperationKind::Face) {
        return Err(JobFileError::invalid("stepover is only valid on a pocket or a face"));
    }
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("stepover is not valid with a kerf tool"));
    };
    let stepover = scale_f64(value, scale, "stepover")?;
    if !(stepover > 0.0) {
        return Err(JobFileError::invalid("stepover must be greater than 0"));
    }
    if stepover > diameter {
        return Err(JobFileError::invalid("stepover is larger than the tool diameter"));
    }
    Ok(Some(stepover))
}

/// A face that omits `stepover` uses three quarters of the tool diameter. Any other kind stores none.
fn default_stepover(kind: OperationKind, tool: &JobTool) -> Result<Option<f64>, JobFileError> {
    if !matches!(kind, OperationKind::Face) {
        return Ok(None);
    }
    let Some(diameter) = tool.diameter() else {
        return Err(JobFileError::invalid("face is not valid with a kerf tool"));
    };
    Ok(Some(diameter * FACE_STEPOVER_OF_DIAMETER))
}

/// Facing direction. Only `0` and `90` degrees are accepted, and the value is stored as radians.
fn parse_face_angle(value: Option<&Value>, kind: OperationKind) -> Result<f64, JobFileError> {
    let Some(value) = value else {
        return Ok(0.0);
    };
    if !matches!(kind, OperationKind::Face | OperationKind::Surface) {
        return Err(JobFileError::invalid("angle_deg is only valid on a face or a surface"));
    }
    let degrees = as_f64(value, "angle_deg").map_err(from_cli)?;
    if degrees == 0.0 {
        Ok(0.0)
    } else if degrees == 90.0 {
        Ok(std::f64::consts::FRAC_PI_2)
    } else {
        Err(JobFileError::invalid("angle_deg must be 0 or 90"))
    }
}

/// Seconds at the bottom of a drill. Absent is `None`. Not scaled by `units`.
fn parse_dwell(value: Option<&Value>, kind: OperationKind) -> Result<Option<f64>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    if !matches!(kind, OperationKind::Drill) {
        return Err(JobFileError::invalid("dwell is only valid on a drill"));
    }
    let Some(seconds) = value.as_f64() else {
        return Err(JobFileError::invalid("dwell must be a number"));
    };
    if !(seconds.is_finite() && (0.0..=3600.0).contains(&seconds)) {
        return Err(JobFileError::invalid("dwell must be between 0 and 3600 seconds"));
    }
    Ok(Some(seconds))
}

/// Extra stock around the faced bounding box. A length in file units, at least 0.
fn parse_margin(value: Option<&Value>, kind: OperationKind, scale: f64) -> Result<f64, JobFileError> {
    let Some(value) = value else {
        return Ok(0.0);
    };
    if !matches!(kind, OperationKind::Face | OperationKind::Surface) {
        return Err(JobFileError::invalid("margin is only valid on a face or a surface"));
    }
    let margin = non_negative_length(value, scale, "margin")?;
    cap_under(margin, MAX_MM, "margin")
}

fn parse_stock_to_leave(value: Option<&Value>, kind: OperationKind, depth: f64, scale: f64) -> Result<StockToLeave, JobFileError> {
    let Some(value) = value else {
        return Ok(StockToLeave::default());
    };
    if matches!(kind, OperationKind::Surface) {
        if !value.is_number() {
            return Err(JobFileError::invalid("surface stock_to_leave must be a number"));
        }
        let axial = non_negative_length(value, scale, "stock_to_leave")?;
        let axial = cap_under(axial, MAX_MM, "stock_to_leave")?;
        return Ok(StockToLeave { wall: 0.0, floor: axial });
    }
    if !matches!(kind, OperationKind::Profile { .. } | OperationKind::Pocket) {
        return Err(JobFileError::invalid("stock_to_leave is only valid on a profile or pocket or a surface"));
    }
    let map = object(value, "stock_to_leave").map_err(from_cli)?;
    check_keys(map, &["wall", "floor"], "stock_to_leave").map_err(from_cli)?;
    let wall = match optional(map, "wall") {
        Some(value) => non_negative_length(value, scale, "stock_to_leave wall")?,
        None => 0.0,
    };
    let floor = match optional(map, "floor") {
        Some(value) => non_negative_length(value, scale, "stock_to_leave floor")?,
        None => 0.0,
    };
    if floor >= depth {
        return Err(JobFileError::invalid("stock_to_leave floor must be less than depth"));
    }
    Ok(StockToLeave { wall, floor })
}

fn non_negative_length(value: &Value, scale: f64, what: &str) -> Result<f64, JobFileError> {
    let length = scale_f64(value, scale, what)?;
    if length < 0.0 {
        return Err(JobFileError::invalid(format!("{what} must be at least 0")));
    }
    Ok(length)
}

fn parse_finish(value: Option<&Value>, kind: OperationKind, depth: f64, scale: f64) -> Result<Option<Finish>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    if !matches!(kind, OperationKind::Profile { .. } | OperationKind::Pocket) {
        return Err(JobFileError::invalid("finish is only valid on a profile or pocket"));
    }
    let map = object(value, "finish").map_err(from_cli)?;
    check_keys(map, &["step_down", "floor"], "finish").map_err(from_cli)?;
    if matches!(kind, OperationKind::Profile { .. }) && optional(map, "floor").is_some() {
        return Err(JobFileError::invalid("finish floor is only valid on a pocket"));
    }
    let step_down = match optional(map, "step_down") {
        Some(value) => {
            let step_down = scale_f64(value, scale, "finish step_down")?;
            if !(step_down > 0.0) {
                return Err(JobFileError::invalid("finish step_down must be positive"));
            }
            check_depth(depth, step_down)?;
            step_down
        }
        None => depth,
    };
    let floor = match optional(map, "floor") {
        Some(value) => as_bool(value, "floor").map_err(from_cli)?,
        None => true,
    };
    Ok(Some(Finish { step_down, floor }))
}

/// `entry` is optional. Lengths scale with the file units. Angles stay in degrees until they
/// are stored as radians. A helix pitch is the drop of one turn at that angle.
fn parse_entry(value: Option<&Value>, scale: f64) -> Result<DepthEntry, JobFileError> {
    let Some(value) = value else {
        return Ok(DepthEntry::Plunge);
    };
    let map = object(value, "entry").map_err(from_cli)?;
    let kind = as_str(require(map, "type").map_err(from_cli)?, "entry type").map_err(from_cli)?;
    match kind {
        "plunge" => {
            check_keys(map, &["type"], "entry").map_err(from_cli)?;
            Ok(DepthEntry::Plunge)
        }
        "ramp" => {
            check_keys(map, &["type", "angle_deg"], "entry").map_err(from_cli)?;
            let max_angle = parse_angle_deg(require(map, "angle_deg").map_err(from_cli)?)?;
            Ok(DepthEntry::Ramp { max_angle })
        }
        "helix" => {
            check_keys(map, &["type", "radius", "angle_deg"], "entry").map_err(from_cli)?;
            let radius = parse_helix_radius(require(map, "radius").map_err(from_cli)?, scale)?;
            let angle = parse_angle_deg(require(map, "angle_deg").map_err(from_cli)?)?;
            let pitch = helix_pitch(radius, angle)?;
            Ok(DepthEntry::Helix { radius, pitch })
        }
        other => Err(JobFileError::invalid(format!("unknown entry type `{other}`"))),
    }
}

/// Degrees from the XY plane, in `(0, 90]`, returned as radians.
fn parse_angle_deg(value: &Value) -> Result<f64, JobFileError> {
    let degrees = as_f64(value, "angle_deg").map_err(from_cli)?;
    if !(degrees > 0.0 && degrees <= 90.0) {
        return Err(JobFileError::invalid("angle_deg must be greater than 0 and at most 90"));
    }
    Ok(degrees.to_radians())
}

fn parse_helix_radius(value: &Value, scale: f64) -> Result<f64, JobFileError> {
    let radius = scale_f64(value, scale, "helix radius")?;
    if radius <= 0.0 {
        return Err(JobFileError::invalid("helix radius must be positive"));
    }
    cap_under(radius, MAX_MM, "helix radius")
}

/// Drop of one full turn, in millimetres. `angle_rad` is from the XY plane, so
/// `pitch = 2π r tan(angle)`.
fn helix_pitch(radius: f64, angle_rad: f64) -> Result<f64, JobFileError> {
    let pitch = radius * std::f64::consts::TAU * angle_rad.tan();
    if !pitch.is_finite() || pitch <= 0.0 {
        return Err(JobFileError::invalid("helix pitch must be finite and positive"));
    }
    cap_under(pitch, MAX_MM, "helix pitch")
}

fn parse_kind(map: &Map<String, Value>) -> Result<OperationKind, JobFileError> {
    let kind = as_str(require(map, "type").map_err(from_cli)?, "operation type").map_err(from_cli)?;
    let side = optional(map, "side");
    match kind {
        "profile" => {
            let Some(side) = side else {
                return Err(JobFileError::invalid("profile needs a side"));
            };
            let side = parse_side(as_str(side, "side").map_err(from_cli)?).map_err(from_cli)?;
            Ok(OperationKind::Profile { side })
        }
        "pocket" => {
            reject_side(side)?;
            Ok(OperationKind::Pocket)
        }
        "drill" => {
            reject_side(side)?;
            Ok(OperationKind::Drill)
        }
        "engrave" => {
            reject_side(side)?;
            Ok(OperationKind::Engrave)
        }
        "face" => {
            reject_side(side)?;
            Ok(OperationKind::Face)
        }
        "surface" => {
            reject_side(side)?;
            Ok(OperationKind::Surface)
        }
        "rough" => {
            reject_side(side)?;
            Ok(OperationKind::Rough)
        }
        "waterline" => {
            reject_side(side)?;
            Ok(OperationKind::Waterline)
        }
        "wrap" => {
            reject_side(side)?;
            Ok(OperationKind::Wrap)
        }
        "rotary_finish" => {
            reject_side(side)?;
            Ok(OperationKind::RotaryFinish)
        }
        "chamfer" => {
            let Some(side) = side else {
                return Err(JobFileError::invalid("chamfer needs a side"));
            };
            let side = parse_side(as_str(side, "side").map_err(from_cli)?).map_err(from_cli)?;
            if matches!(side, ProfileSide::On) {
                return Err(JobFileError::invalid("chamfer side must be outside or inside"));
            }
            Ok(OperationKind::Chamfer { side })
        }
        other => Err(JobFileError::invalid(format!("unknown operation type `{other}`"))),
    }
}

fn reject_side(side: Option<&Value>) -> Result<(), JobFileError> {
    if side.is_some() { Err(JobFileError::invalid("side is only valid on a profile or a chamfer")) } else { Ok(()) }
}

fn parse_layers(value: &Value) -> Result<Vec<String>, JobFileError> {
    let list = array(value, "layers", MAX_LAYERS)?;
    if list.is_empty() {
        return Err(JobFileError::invalid("operation needs at least one layer"));
    }
    let mut layers = Vec::with_capacity(list.len());
    for item in list {
        let name = required_name(as_str(item, "layer").map_err(from_cli)?, "layer").map_err(from_cli)?;
        if layers.iter().any(|existing| existing == &name) {
            return Err(JobFileError::invalid(format!("layer `{name}` is repeated")));
        }
        layers.push(name);
    }
    Ok(layers)
}

fn parse_operation_feeds(value: &Value, scale: f64) -> Result<Feeds, JobFileError> {
    let map = object(value, "feeds").map_err(from_cli)?;
    check_keys(map, &["cut", "plunge"], "feeds").map_err(from_cli)?;
    let cut = cap_feed(scale_f64(require(map, "cut").map_err(from_cli)?, scale, "cut feed")?)?;
    let plunge = match optional(map, "plunge") {
        Some(value) => Some(cap_feed(scale_f64(value, scale, "plunge feed")?)?),
        None => None,
    };
    let plunge_only = match plunge {
        Some(plunge) => {
            let mut feeds = Map::new();
            feeds.insert("plunge".to_string(), number_value(plunge, "plunge feed")?);
            Some(Value::Object(feeds))
        }
        None => None,
    };
    parse_feeds(plunge_only.as_ref(), cut).map_err(from_cli)
}

fn parse_timing(op: &Map<String, Value>, scale: f64) -> Result<ProcessParams, JobFileError> {
    let mut params = Map::new();
    if let Some(timing) = optional(op, "timing") {
        let timing = object(timing, "timing").map_err(from_cli)?;
        check_keys(timing, &["pierce_delay", "post_delay", "pierce_process_on_during_delay", "post_process_on_during_delay"], "timing")
            .map_err(from_cli)?;
        copy_delay(&mut params, timing, "pierce_delay", "pierce_delay_s")?;
        copy_delay(&mut params, timing, "post_delay", "post_delay_s")?;
        copy_toggle(&mut params, timing, "pierce_process_on_during_delay")?;
        copy_toggle(&mut params, timing, "post_process_on_during_delay")?;
    }
    copy_height(&mut params, op, "pierce_height", scale)?;
    copy_height(&mut params, op, "cut_height", scale)?;
    let value = if params.is_empty() { None } else { Some(Value::Object(params)) };
    // `parse_params` fills the same defaults as `ProcessTiming::new`: pierce on, post off, delays 0.
    parse_params(value.as_ref()).map_err(from_cli)
}

fn copy_delay(params: &mut Map<String, Value>, timing: &Map<String, Value>, from: &str, to: &str) -> Result<(), JobFileError> {
    let Some(value) = optional(timing, from) else {
        return Ok(());
    };
    let seconds = as_f64(value, from).map_err(from_cli)?;
    if seconds > MAX_DELAY_S {
        return Err(JobFileError::invalid(format!("{from} is larger than {MAX_DELAY_S} s")));
    }
    params.insert(to.to_string(), number_value(seconds, from)?);
    Ok(())
}

fn copy_toggle(params: &mut Map<String, Value>, timing: &Map<String, Value>, key: &str) -> Result<(), JobFileError> {
    let Some(value) = optional(timing, key) else {
        return Ok(());
    };
    let on = as_bool(value, key).map_err(from_cli)?;
    params.insert(key.to_string(), Value::Bool(on));
    Ok(())
}

fn copy_height(params: &mut Map<String, Value>, op: &Map<String, Value>, key: &str, scale: f64) -> Result<(), JobFileError> {
    let Some(value) = optional(op, key) else {
        return Ok(());
    };
    let z = finite_height(scale_f64(value, scale, key)?, key)?;
    params.insert(key.to_string(), number_value(z, key)?);
    Ok(())
}

fn finite_height(z: f64, what: &str) -> Result<f64, JobFileError> {
    if !z.is_finite() {
        return Err(JobFileError::invalid(format!("{what} must be finite")));
    }
    if z.abs() > MAX_MM {
        return Err(JobFileError::invalid(format!("{what} is outside ±{MAX_MM} mm")));
    }
    Ok(z)
}

fn check_depth(depth: f64, step_down: f64) -> Result<(), JobFileError> {
    let depths = campanion_path::pass_depths(depth, step_down).map_err(|err| {
        let message = err.to_string();
        let detail = message.strip_prefix("invalid input: ").unwrap_or(message.as_str());
        let detail = detail.replacen("total depth", "depth", 1);
        JobFileError::invalid(detail.replacen("step-down", "step_down", 1))
    })?;
    if depths.is_empty() {
        return Err(JobFileError::invalid("depth plan is empty"));
    }
    Ok(())
}

fn scale_key(map: &mut Map<String, Value>, key: &str, scale: f64) -> Result<(), JobFileError> {
    let Some(value) = optional(map, key) else {
        return Ok(());
    };
    let scaled = scale_f64(value, scale, key)?;
    map.insert(key.to_string(), number_value(scaled, key)?);
    Ok(())
}

fn scale_f64(value: &Value, scale: f64, what: &str) -> Result<f64, JobFileError> {
    let number = as_f64(value, what).map_err(from_cli)?;
    let scaled = number * scale;
    if scaled.is_finite() { Ok(scaled) } else { Err(JobFileError::invalid(format!("{what} must be finite"))) }
}

fn cap_feed(value: f64) -> Result<f64, JobFileError> {
    if value > MAX_FEED_MM_PER_MIN { Err(JobFileError::invalid(format!("feed is larger than {MAX_FEED_MM_PER_MIN} mm/min"))) } else { Ok(value) }
}

fn cap_under(value: f64, max: f64, what: &str) -> Result<f64, JobFileError> {
    if value > max { Err(JobFileError::invalid(format!("{what} is larger than {max}"))) } else { Ok(value) }
}

fn number_value(number: f64, what: &str) -> Result<Value, JobFileError> {
    let Some(json) = Number::from_f64(number) else {
        return Err(JobFileError::invalid(format!("{what} must be finite")));
    };
    Ok(Value::Number(json))
}

fn array<'a>(value: &'a Value, what: &str, max: usize) -> Result<&'a Vec<Value>, JobFileError> {
    let Some(list) = value.as_array() else {
        return Err(JobFileError::invalid(format!("{what} must be an array")));
    };
    if list.len() > max {
        return Err(JobFileError::invalid(format!("too many {what}: {} > {max}", list.len())));
    }
    Ok(list)
}

/// A component this close to zero, or to ±X, is treated as that direction.
const PLANE_DIR_EPS: f64 = 1e-9;

fn op_invalid(name: &str, detail: impl fmt::Display) -> JobFileError {
    JobFileError::invalid(format!("operation `{name}`: {detail}"))
}

fn named_field(name: &str, err: JobFileError) -> JobFileError {
    match err {
        JobFileError::Invalid { message } => {
            let detail = message.strip_prefix("invalid input: ").unwrap_or(message.as_str());
            if detail.starts_with("operation `") { JobFileError::Invalid { message } } else { op_invalid(name, detail) }
        }
        other => other,
    }
}

fn take_plane(name: &str, map: &Map<String, Value>, kind: &OperationKind, indexed: bool, scale: f64) -> Result<Option<WorkPlane>, JobFileError> {
    let Some(value) = optional(map, "plane") else {
        return Ok(None);
    };
    if let Some(type_name) = unsupported_plane(kind) {
        return Err(op_invalid(name, format!("plane is not supported on {type_name}")));
    }
    if indexed {
        return Err(op_invalid(name, "plane and index_deg cannot be combined"));
    }
    Ok(Some(parse_plane(name, value, scale)?))
}

fn unsupported_plane(kind: &OperationKind) -> Option<&'static str> {
    match kind {
        OperationKind::Wrap => Some("wrap"),
        OperationKind::RotaryFinish => Some("rotary_finish"),
        OperationKind::Surface => Some("surface"),
        OperationKind::Rough => Some("rough"),
        OperationKind::Waterline => Some("waterline"),
        _ => None,
    }
}

fn attach_plane(mut operation: Operation, plane: Option<WorkPlane>) -> Operation {
    operation.plane = plane;
    operation
}

fn parse_plane(name: &str, value: &Value, scale: f64) -> Result<WorkPlane, JobFileError> {
    let map = object(value, "plane").map_err(|err| named_field(name, from_cli(err)))?;
    check_keys(map, &["origin", "normal", "x_dir"], "plane").map_err(|err| named_field(name, from_cli(err)))?;
    let origin = parse_plane_origin(name, require(map, "origin").map_err(|err| named_field(name, from_cli(err)))?, scale)?;
    let normal = plane_normal(name, require(map, "normal").map_err(|err| named_field(name, from_cli(err)))?)?;
    let raw_x = match optional(map, "x_dir") {
        Some(value) => plane_vector(name, value, "plane x_dir")?,
        None => default_plane_x(normal),
    };
    let x_dir = match unit_direction(project_onto_plane(raw_x, normal)) {
        Ok(direction) => direction,
        Err(PlaneDirError::NotFinite) => return Err(op_invalid(name, "plane x_dir must be finite")),
        Err(PlaneDirError::Zero) => return Err(op_invalid(name, "plane x_dir is parallel to the normal")),
    };
    Ok(WorkPlane { origin, normal, x_dir })
}

fn parse_plane_origin(name: &str, value: &Value, scale: f64) -> Result<Vec3, JobFileError> {
    let list = array(value, "plane origin", 3).map_err(|err| named_field(name, err))?;
    if list.len() != 3 {
        return Err(op_invalid(name, "plane origin must be exactly 3 numbers"));
    }
    let mut coords = [0.0; 3];
    for (slot, item) in coords.iter_mut().zip(list.iter()) {
        let scaled = scale_f64(item, scale, "plane origin").map_err(|err| named_field(name, err))?;
        *slot = finite_height(scaled, "plane origin").map_err(|err| named_field(name, err))?;
    }
    Vec3::new(coords[0], coords[1], coords[2]).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

fn plane_normal(name: &str, value: &Value) -> Result<Vec3, JobFileError> {
    let raw = plane_vector(name, value, "plane normal")?;
    match unit_direction(raw) {
        Ok(direction) => Ok(direction),
        Err(PlaneDirError::NotFinite) => Err(op_invalid(name, "plane normal must be finite")),
        Err(PlaneDirError::Zero) => Err(op_invalid(name, "plane normal is zero")),
    }
}

/// Direction components. Not scaled by `units`.
fn plane_vector(name: &str, value: &Value, what: &str) -> Result<Vec3, JobFileError> {
    let list = array(value, what, 3).map_err(|err| named_field(name, err))?;
    if list.len() != 3 {
        return Err(op_invalid(name, format!("{what} must be exactly 3 numbers")));
    }
    let mut coords = [0.0; 3];
    for (slot, item) in coords.iter_mut().zip(list.iter()) {
        let number = as_f64(item, what).map_err(|err| named_field(name, from_cli(err)))?;
        if !number.is_finite() {
            return Err(op_invalid(name, format!("{what} must be finite")));
        }
        *slot = number;
    }
    Vec3::new(coords[0], coords[1], coords[2]).map_err(|err| JobFileError::Invalid { message: err.to_string() })
}

fn default_plane_x(normal: Vec3) -> Vec3 {
    let seed = if near_pm_x(normal) { Vec3::Y } else { Vec3::X };
    project_onto_plane(seed, normal)
}

fn near_pm_x(normal: Vec3) -> bool {
    let to_pos = hypot3(normal.x() - 1.0, normal.y(), normal.z());
    let to_neg = hypot3(normal.x() + 1.0, normal.y(), normal.z());
    to_pos <= PLANE_DIR_EPS || to_neg <= PLANE_DIR_EPS
}

fn project_onto_plane(vector: Vec3, normal: Vec3) -> Vec3 {
    let along = vector.x() * normal.x() + vector.y() * normal.y() + vector.z() * normal.z();
    match Vec3::new(vector.x() - normal.x() * along, vector.y() - normal.y() * along, vector.z() - normal.z() * along) {
        Ok(projected) => projected,
        Err(_) => Vec3::ZERO,
    }
}

enum PlaneDirError {
    Zero,
    NotFinite,
}

fn unit_direction(vector: Vec3) -> Result<Vec3, PlaneDirError> {
    let length_sq = vector.x() * vector.x() + vector.y() * vector.y() + vector.z() * vector.z();
    if !length_sq.is_finite() {
        return Err(PlaneDirError::NotFinite);
    }
    let length = length_sq.sqrt();
    if length <= PLANE_DIR_EPS {
        return Err(PlaneDirError::Zero);
    }
    Vec3::new(vector.x() / length, vector.y() / length, vector.z() / length).map_err(|_| PlaneDirError::NotFinite)
}

fn hypot3(x: f64, y: f64, z: f64) -> f64 {
    (x * x + y * y + z * z).sqrt()
}

fn load_kinematics(job_path: &Path, value: &Value) -> Result<Kinematics, JobFileError> {
    let relative = as_str(value, "kinematics").map_err(from_cli)?;
    if relative.is_empty() {
        return Err(JobFileError::invalid("kinematics: path is empty"));
    }
    let dir = job_path.parent().unwrap_or(Path::new(""));
    let path = dir.join(relative);
    let text = read_kinematics(&path)?;
    Kinematics::from_json(&text).map_err(|err| JobFileError::invalid(format!("kinematics: {}", kinematics_detail(err))))
}

fn read_kinematics(path: &Path) -> Result<String, JobFileError> {
    let file = File::open(path).map_err(|err| JobFileError::invalid(format!("kinematics: {}: {err}", path.display())))?;
    if let Ok(meta) = file.metadata()
        && meta.len() > MAX_JSON_BYTES as u64
    {
        return Err(JobFileError::invalid(format!("kinematics: {} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    let mut bytes = Vec::new();
    file.take(MAX_JSON_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|err| JobFileError::invalid(format!("kinematics: {}: {err}", path.display())))?;
    if bytes.len() > MAX_JSON_BYTES {
        return Err(JobFileError::invalid(format!("kinematics: {} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    String::from_utf8(bytes).map_err(|_| JobFileError::invalid(format!("kinematics: {} is not valid UTF-8", path.display())))
}

fn kinematics_detail(err: campanion_ir::IrError) -> String {
    match err {
        campanion_ir::IrError::InvalidInput(message) => message,
        other => other.to_string(),
    }
}

fn parse_solution(value: Option<&Value>) -> Result<SolutionPolicy, JobFileError> {
    let Some(value) = value else {
        return Ok(SolutionPolicy::Nearest);
    };
    match as_str(value, "solution").map_err(from_cli)? {
        "nearest" => Ok(SolutionPolicy::Nearest),
        "first_positive" => Ok(SolutionPolicy::FirstPositive),
        "first_negative" => Ok(SolutionPolicy::FirstNegative),
        _ => Err(JobFileError::invalid("solution must be \"nearest\", \"first_positive\" or \"first_negative\"")),
    }
}

fn check_reachable(kinematics: &Kinematics, policy: SolutionPolicy, operation: &Operation) -> Result<(), JobFileError> {
    let (tip, axis) = match operation.plane() {
        Some(plane) => (plane.origin(), plane.normal()),
        None => (Vec3::ZERO, Vec3::Z),
    };
    kinematics.choose(&ToolPose { tip, axis }, 0.0, policy.choice([0.0, 0.0])).map_err(|err| op_invalid(operation.name(), kinematics_detail(err)))?;
    Ok(())
}

fn at_index(kind: &str, index: usize, err: JobFileError) -> JobFileError {
    match err {
        JobFileError::Invalid { message } => {
            let detail = message.strip_prefix("invalid input: ").unwrap_or(message.as_str());
            // A plane error already names the operation. Keep that text.
            if detail.starts_with("operation `") {
                return JobFileError::Invalid { message };
            }
            JobFileError::invalid(format!("{kind} {index}: {detail}"))
        }
        other => other,
    }
}

fn from_cli(err: CliError) -> JobFileError {
    JobFileError::Invalid { message: err.to_string() }
}

fn fold_dots(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                let top = out.components().next_back().map(|component| match component {
                    Component::Normal(_) => DotFold::Named,
                    Component::ParentDir => DotFold::Parent,
                    Component::RootDir | Component::Prefix(_) | Component::CurDir => DotFold::Root,
                });
                match top {
                    Some(DotFold::Named) => {
                        out.pop();
                    }
                    Some(DotFold::Root) => {}
                    Some(DotFold::Parent) | None => out.push(Component::ParentDir),
                }
            }
            other => out.push(other),
        }
    }
    out
}

enum DotFold {
    Named,
    Parent,
    Root,
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{SolutionPolicy, Vec3};
    use campanion_path::Tool as PathTool;
    use std::fs;
    use std::path::Path;
    use std::time::{SystemTime, UNIX_EPOCH};

    const EMPTY_HASH: &str = "cbf29ce484222325";

    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-9, "{actual} != {expected}");
    }

    fn write_job(name: &str, text: &str) -> PathBuf {
        let ticks = SystemTime::now().duration_since(UNIX_EPOCH).map(|duration| duration.as_nanos()).unwrap_or(0);
        let dir = std::env::temp_dir().join(format!("campanion-jobfile-{name}-{ticks}"));
        fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("nested").join("job.json");
        fs::create_dir_all(path.parent().expect("parent")).expect("nested");
        fs::write(&path, text).expect("write job");
        path
    }

    fn envelope(source_path: &str, tools: &str, ops: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"{source_path}","hash":"{EMPTY_HASH}"}},"units":"mm","tools":{tools},"operations":{ops}}}"#
        )
    }

    #[test]
    fn fnv1a64_known_vectors() {
        assert_eq!(hash_bytes(b""), "cbf29ce484222325");
        assert_eq!(hash_bytes(b"a"), "af63dc4c8601ec8c");
        assert_eq!(hash_bytes(b"foobar"), "85944171f73967e8");
        assert_ne!(hash_bytes(b"foobar"), hash_bytes(b"foobas"));
    }

    #[test]
    fn loads_operations_and_detects_source_changes() {
        let text = r#"{
          "format": "campanion-job/1",
          "source": { "path": "../parts/plate.dxf", "hash": "CBF29CE484222325" },
          "units": "mm",
          "tools": [ { "id": 1, "diameter": 6.0 }, { "id": 2, "kerf": 1.2 } ],
          "operations": [
            {
              "name": "outer",
              "type": "profile",
              "layers": ["OUTLINE"],
              "side": "outside",
              "tool_id": 1,
              "depth": 3.0,
              "step_down": 1.5,
              "feeds": { "cut": 1000.0, "plunge": 300.0 },
              "timing": { "pierce_delay": 0.25 }
            },
            {
              "name": "holes",
              "type": "drill",
              "layers": ["HOLES"],
              "tool_id": 1,
              "depth": 10.0,
              "step_down": 10.0,
              "feeds": { "cut": 800.0 }
            },
            {
              "name": "mark",
              "type": "engrave",
              "layers": ["MARK", "TEXT"],
              "tool_id": 2,
              "depth": 0.2,
              "step_down": 0.2,
              "feeds": { "cut": 500.0 }
            }
          ]
        }"#;
        let path = write_job("mm", text);
        let job = load(&path).expect("load");
        assert_eq!(job.format(), FORMAT_TAG);
        assert_eq!(job.units(), Units::Millimetre);
        assert_eq!(job.units().to_string(), "mm");
        assert_eq!(job.source().path(), "../parts/plate.dxf");
        assert_eq!(job.source().hash(), EMPTY_HASH);
        assert!(source_path(&job).ends_with("parts/plate.dxf"));
        assert!(!source_changed(&job, b""));
        assert!(source_changed(&job, b"a"));
        let updated = with_updated_hash(&job, b"a");
        assert_eq!(updated.source().hash(), hash_bytes(b"a"));
        assert!(!source_changed(&updated, b"a"));
        assert_eq!(job.source().hash(), EMPTY_HASH);
        assert_eq!(job.tolerance().linear(), Tolerance::DEFAULT_LINEAR_MM);
        assert_eq!(job.clearance(), 5.0);
        assert_eq!(job.retract(), 2.0);

        let diameter = job.tool(1).expect("tool 1");
        assert_eq!(diameter.path_tool(), PathTool::Diameter { diameter: 6.0 });
        assert_eq!(diameter.ir().diameter(), Some(6.0));
        assert_eq!(diameter.ir().kerf(), None);
        assert_eq!(diameter.speed(), None);
        let kerf = job.tool(2).expect("tool 2");
        assert_eq!(kerf.path_tool(), PathTool::Kerf { width: 1.2 });
        assert_eq!(kerf.ir().kerf(), Some(1.2));

        let ops = job.operations();
        assert_eq!(ops.len(), 3);
        assert_eq!(ops[0].kind(), OperationKind::Profile { side: ProfileSide::Outside });
        assert_eq!(ops[0].layers(), &["OUTLINE".to_string()]);
        assert_eq!(ops[0].depth(), 3.0);
        assert_eq!(ops[0].step_down(), 1.5);
        assert_eq!(ops[0].feeds().cut(), 1000.0);
        assert_eq!(ops[0].feeds().plunge(), Some(300.0));
        assert_eq!(ops[0].params().pierce_delay_s(), 0.25);
        assert_eq!(ops[0].params().post_delay_s(), 0.0);
        assert!(ops[0].params().pierce_process_on_during_delay());
        assert!(!ops[0].params().post_process_on_during_delay());
        assert_eq!(ops[0].params().pierce_height_z(), None);
        assert_eq!(ops[0].speed(), None);
        assert_eq!(ops[0].resolved_speed(diameter), None);
        assert_eq!(ops[0].entry(), DepthEntry::Plunge);
        assert_eq!(ops[1].kind(), OperationKind::Drill);
        assert_eq!(ops[1].entry(), DepthEntry::Plunge);
        assert_eq!(ops[1].feeds().plunge(), None);
        assert_eq!(ops[1].params().pierce_delay_s(), 0.0);
        assert!(ops[1].params().pierce_process_on_during_delay());
        assert!(!ops[1].params().post_process_on_during_delay());
        assert_eq!(ops[2].kind(), OperationKind::Engrave);
        assert_eq!(ops[2].layers().len(), 2);
        assert_eq!(ops[2].tool_id(), 2);
        assert_eq!(ops[2].entry(), DepthEntry::Plunge);

        let empty = load(&write_job("empty", &envelope("plate.dxf", "[]", "[]"))).expect("empty");
        assert!(empty.tools().is_empty());
        assert!(empty.operations().is_empty());
    }

    #[test]
    fn parses_tool_and_operation_speed() {
        let text = r#"{
          "format": "campanion-job/1",
          "source": { "path": "plate.dxf", "hash": "cbf29ce484222325" },
          "units": "mm",
          "tools": [
            { "id": 1, "diameter": 6.0, "speed": 12000 },
            { "id": 2, "diameter": 3.0, "speed": 8000 }
          ],
          "operations": [
            {
              "name": "slot",
              "type": "pocket",
              "layers": ["SLOT"],
              "tool_id": 2,
              "depth": 3.0,
              "step_down": 1.5,
              "feeds": { "cut": 1500.0 }
            },
            {
              "name": "outline",
              "type": "profile",
              "layers": ["OUTLINE"],
              "side": "outside",
              "tool_id": 1,
              "depth": 6.0,
              "step_down": 3.0,
              "speed": 9000,
              "feeds": { "cut": 1500.0 }
            }
          ]
        }"#;
        let job = load(&write_job("speed", text)).expect("load");
        let tool1 = job.tool(1).expect("tool 1");
        let tool2 = job.tool(2).expect("tool 2");
        assert_eq!(tool1.speed(), Some(12000.0));
        assert_eq!(tool2.speed(), Some(8000.0));
        let ops = job.operations();
        assert_eq!(ops[0].speed(), None);
        assert_eq!(ops[0].resolved_speed(tool2), Some(8000.0));
        assert_eq!(ops[1].speed(), Some(9000.0));
        assert_eq!(ops[1].resolved_speed(tool1), Some(9000.0));
        let params = ops[1].params_with_speed(tool1).expect("params");
        assert_eq!(params.speed_value(), Some(9000.0));

        let ceiling = envelope("plate.dxf", r#"[{"id":1,"diameter":6,"speed":100000}]"#, "[]");
        load(&write_job("speed-max", &ceiling)).expect("100000 rpm");

        for (label, tools) in [
            ("zero", r#"[{"id":1,"diameter":6,"speed":0}]"#),
            ("negative", r#"[{"id":1,"diameter":6,"speed":-1}]"#),
            ("too-fast", r#"[{"id":1,"diameter":6,"speed":100001}]"#),
        ] {
            let err = load(&write_job(label, &envelope("plate.dxf", tools, "[]"))).expect_err(label);
            assert!(err.to_string().contains("speed must be positive and at most 100000"), "{label}: {err}");
        }

        let op_job = envelope(
            "plate.dxf",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"speed":0,"feeds":{"cut":1}}]"#,
        );
        let err = load(&write_job("op-zero", &op_job)).expect_err("op zero");
        assert!(err.to_string().contains("speed must be positive and at most 100000"), "{err}");
    }

    #[test]
    fn converts_inches_heights_and_timing_toggles() {
        let text = r#"{
          "format": "campanion-job/1",
          "source": { "path": "plate.dxf", "hash": "cbf29ce484222325" },
          "units": "inch",
          "tolerance": { "linear": 0.001, "angular": 0.01, "chord": 0.002 },
          "tools": [ { "id": 3, "diameter": 0.25 } ],
          "operations": [
            {
              "name": "pocket",
              "type": "pocket",
              "layers": ["POCKET"],
              "tool_id": 3,
              "depth": 0.5,
              "step_down": 0.25,
              "feeds": { "cut": 10.0, "plunge": 2.0 },
              "pierce_height": 0.1,
              "cut_height": -0.05,
              "timing": {
                "pierce_delay": 0.4,
                "post_delay": 0.2,
                "pierce_process_on_during_delay": false,
                "post_process_on_during_delay": true
              }
            }
          ]
        }"#;
        let job = load(&write_job("inch", text)).expect("inch");
        assert_eq!(job.units(), Units::Inch);
        assert_eq!(job.clearance(), 5.0);
        assert_eq!(job.retract(), 2.0);
        near(job.tool(3).expect("tool").path_tool().compensation_distance().expect("radius"), 0.25 * 25.4 / 2.0);
        near(job.tolerance().linear(), 0.001 * 25.4);
        assert_eq!(job.tolerance().angular(), 0.01);
        near(job.tolerance().chord(), 0.002 * 25.4);
        let op = job.operations().first().expect("op");
        assert_eq!(op.kind(), OperationKind::Pocket);
        near(op.depth(), 0.5 * 25.4);
        near(op.step_down(), 0.25 * 25.4);
        near(op.feeds().cut(), 10.0 * 25.4);
        near(op.feeds().plunge().expect("plunge"), 2.0 * 25.4);
        near(op.params().pierce_height_z().expect("pierce"), 0.1 * 25.4);
        near(op.params().cut_height_z().expect("cut"), -0.05 * 25.4);
        assert_eq!(op.params().pierce_delay_s(), 0.4);
        assert_eq!(op.params().post_delay_s(), 0.2);
        assert!(!op.params().pierce_process_on_during_delay());
        assert!(op.params().post_process_on_during_delay());
    }

    fn tolerance_job(units: &str, tolerance: &str) -> String {
        let tolerance = if tolerance.is_empty() { String::new() } else { format!(r#","tolerance":{tolerance}"#) };
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},"units":"{units}"{tolerance},"tools":[],"operations":[]}}"#
        )
    }

    #[test]
    fn chord_defaults_to_job_tolerance_and_keeps_an_explicit_value() {
        let omitted = load(&write_job("tol-absent", &tolerance_job("mm", ""))).expect("no tolerance");
        assert_eq!(omitted.tolerance().linear(), Tolerance::DEFAULT_LINEAR_MM);
        assert_eq!(omitted.tolerance().angular(), Tolerance::DEFAULT_ANGULAR_RAD);
        assert_eq!(omitted.tolerance().chord(), 0.01);
        assert_eq!(omitted.tolerance().chord(), DEFAULT_JOB_CHORD_MM);

        let linear_only = load(&write_job("tol-linear", &tolerance_job("mm", r#"{"linear":0.001}"#))).expect("linear only");
        near(linear_only.tolerance().linear(), 0.001);
        assert_eq!(linear_only.tolerance().angular(), Tolerance::DEFAULT_ANGULAR_RAD);
        assert_eq!(linear_only.tolerance().chord(), 0.01);

        // The omitted chord is already millimetres, so inch jobs do not scale it.
        let inch_linear = load(&write_job("tol-inch-linear", &tolerance_job("inch", r#"{"linear":0.001}"#))).expect("inch linear");
        near(inch_linear.tolerance().linear(), 0.001 * 25.4);
        assert_eq!(inch_linear.tolerance().chord(), DEFAULT_JOB_CHORD_MM);

        let explicit = load(&write_job("tol-chord", &tolerance_job("mm", r#"{"linear":0.001,"chord":0.05}"#))).expect("explicit");
        near(explicit.tolerance().linear(), 0.001);
        assert_eq!(explicit.tolerance().chord(), 0.05);
    }

    #[test]
    fn inch_job_without_tolerance_keeps_millimetre_defaults() {
        // 0.01 mm is about 0.0004 in. Stored tolerances are millimetres, so these defaults
        // must not be multiplied by 25.4 when the job is written in inches.
        let job = load(&write_job("tol-inch-absent", &tolerance_job("inch", ""))).expect("inch defaults");
        assert_eq!(job.units(), Units::Inch);
        assert_eq!(job.tolerance().chord(), 0.01);
        assert_eq!(job.tolerance().chord(), DEFAULT_JOB_CHORD_MM);
        assert_eq!(job.tolerance().linear(), 1e-6);
        assert_eq!(job.tolerance().linear(), Tolerance::DEFAULT_LINEAR_MM);
        assert_eq!(job.tolerance().angular(), Tolerance::DEFAULT_ANGULAR_RAD);
    }

    fn source_job(path: &str, face: &str, units: &str) -> String {
        let face = if face.is_empty() { String::new() } else { format!(r#","face":{face}"#) };
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"{path}","hash":"{EMPTY_HASH}"{face}}},"units":"{units}","tools":[],"operations":[]}}"#
        )
    }

    #[test]
    fn step_face_parses_unscaled_and_is_rejected_elsewhere() {
        let step = load(&write_job("step-face", &source_job("part.step", "[12.5, -3.0, 1.0]", "mm"))).expect("step");
        assert!(step.source().is_step());
        assert_eq!(step.source().path(), "part.step");
        assert_eq!(step.source().face(), Some([12.5, -3.0, 1.0]));

        let missing = load(&write_job("step-missing-face", &source_job("part.step", "", "mm")));
        let err = missing.expect_err("step without face");
        assert!(err.to_string().contains("a STEP source needs `face`: a point [x, y, z] in mm on the planar face to cut"), "{err}");
        assert!(matches!(err, JobFileError::Invalid { .. }), "{err:?}");

        let dxf = load(&write_job("dxf-face", &source_job("plate.dxf", "[0.0, 0.0, 0.0]", "mm")));
        let err = dxf.expect_err("dxf with face");
        assert!(err.to_string().contains("`face` only applies to STEP sources"), "{err}");
        assert!(matches!(err, JobFileError::Invalid { .. }), "{err:?}");

        let plain = load(&write_job("dxf-plain", &source_job("plate.dxf", "", "mm"))).expect("dxf");
        assert!(!plain.source().is_step());
        assert_eq!(plain.source().face(), None);

        // `.STP` is STEP. Face stays in model millimetres even when the job units are inches.
        let stp = load(&write_job("stp-face", &source_job("parts/Cover.STP", "[4.0, 5.0, 6.0]", "inch"))).expect("stp");
        assert!(stp.source().is_step());
        assert_eq!(stp.units(), Units::Inch);
        assert_eq!(stp.source().face(), Some([4.0, 5.0, 6.0]));

        let edge = load(&write_job("step-edge-face", &source_job("part.step", "[-1000000.0, 1000000.0, 0.0]", "mm"))).expect("edge");
        assert_eq!(edge.source().face(), Some([-1.0e6, 1.0e6, 0.0]));

        let short = load(&write_job("step-short-face", &source_job("part.step", "[1.0, 2.0]", "mm")));
        let err = short.expect_err("short face");
        assert!(err.to_string().contains("face must be exactly 3 numbers"), "{err}");

        let huge = load(&write_job("step-huge-face", &source_job("part.step", "[0.0, 0.0, 1000001.0]", "mm")));
        let err = huge.expect_err("huge face");
        assert!(err.to_string().contains("face is outside"), "{err}");
    }

    fn height_job(units_and_heights: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},{units_and_heights},"tools":[{{"id":1,"diameter":6}}],"operations":[{{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{{"cut":1}}}}]}}"#
        )
    }

    #[test]
    fn heights_default_explicit_and_reject_retract_above_clearance() {
        let defaults = load(&write_job("heights-default", &height_job(r#""units":"mm""#))).expect("defaults");
        assert_eq!(defaults.clearance(), 5.0);
        assert_eq!(defaults.retract(), 2.0);
        let updated = with_updated_hash(&defaults, b"a");
        assert_eq!(updated.clearance(), 5.0);
        assert_eq!(updated.retract(), 2.0);

        let empty = load(&write_job("heights-empty", &height_job(r#""units":"mm","heights":{}"#))).expect("empty heights");
        assert_eq!(empty.clearance(), 5.0);
        assert_eq!(empty.retract(), 2.0);

        let explicit = load(&write_job("heights-mm", &height_job(r#""units":"mm","heights":{"clearance":8.0,"retract":3.5}"#))).expect("explicit");
        near(explicit.clearance(), 8.0);
        near(explicit.retract(), 3.5);
        let kept = with_updated_hash(&explicit, b"x");
        near(kept.clearance(), 8.0);
        near(kept.retract(), 3.5);

        let equal = load(&write_job("heights-eq", &height_job(r#""units":"mm","heights":{"clearance":4,"retract":4}"#))).expect("equal");
        assert_eq!(equal.clearance(), 4.0);
        assert_eq!(equal.retract(), 4.0);

        let inches =
            load(&write_job("heights-inch", &height_job(r#""units":"inch","heights":{"clearance":0.5,"retract":0.1}"#))).expect("inch heights");
        near(inches.clearance(), 0.5 * 25.4);
        near(inches.retract(), 0.1 * 25.4);

        let clearance_only =
            load(&write_job("heights-clearance", &height_job(r#""units":"inch","heights":{"clearance":1}"#))).expect("clearance only");
        near(clearance_only.clearance(), 25.4);
        assert_eq!(clearance_only.retract(), 2.0);

        let above = load(&write_job("heights-above", &height_job(r#""units":"mm","heights":{"clearance":2.0,"retract":5.0}"#)));
        let err = above.expect_err("retract above clearance");
        assert!(err.to_string().contains("retract is above clearance"), "{err}");
        assert!(matches!(err, JobFileError::Invalid { .. }), "{err:?}");

        // 0.3 inch is 7.62 mm, which is above the unscaled 5 mm default clearance.
        let scaled_above = load(&write_job("heights-scaled-above", &height_job(r#""units":"inch","heights":{"retract":0.3}"#)));
        let err = scaled_above.expect_err("scaled retract above default clearance");
        assert!(err.to_string().contains("retract is above clearance"), "{err}");
    }

    fn entry_job(units: &str, entry: &str) -> String {
        let entry = if entry.is_empty() { String::new() } else { format!(",{entry}") };
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},"units":"{units}","tools":[{{"id":1,"diameter":6}}],"operations":[{{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{{"cut":1}}{entry}}}]}}"#
        )
    }

    #[test]
    fn entry_defaults_to_plunge_and_maps_ramp_and_helix() {
        let omitted = load(&write_job("entry-default", &entry_job("mm", ""))).expect("default");
        assert_eq!(omitted.operations()[0].entry(), DepthEntry::Plunge);
        assert_eq!(with_updated_hash(&omitted, b"a").operations()[0].entry(), DepthEntry::Plunge);

        let plunge = load(&write_job("entry-plunge", &entry_job("mm", r#""entry":{"type":"plunge"}"#))).expect("plunge");
        assert_eq!(plunge.operations()[0].entry(), DepthEntry::Plunge);

        let ramp = load(&write_job("entry-ramp", &entry_job("mm", r#""entry":{"type":"ramp","angle_deg":12.5}"#))).expect("ramp");
        match ramp.operations()[0].entry() {
            DepthEntry::Ramp { max_angle } => near(max_angle, 12.5_f64.to_radians()),
            other => panic!("ramp: {other:?}"),
        }

        // A right angle is allowed. Inches must not scale the angle.
        let vertical = load(&write_job("entry-ramp-90", &entry_job("inch", r#""entry":{"type":"ramp","angle_deg":90}"#))).expect("90");
        match vertical.operations()[0].entry() {
            DepthEntry::Ramp { max_angle } => near(max_angle, 90.0_f64.to_radians()),
            other => panic!("vertical ramp: {other:?}"),
        }

        let helix = load(&write_job("entry-helix", &entry_job("mm", r#""entry":{"type":"helix","radius":10,"angle_deg":30}"#))).expect("helix");
        match helix.operations()[0].entry() {
            DepthEntry::Helix { radius, pitch } => {
                near(radius, 10.0);
                // One turn drops 2π r tan(angle). 10 mm at 30° is about 36.276 mm.
                near(pitch, 10.0 * std::f64::consts::TAU * 30.0_f64.to_radians().tan());
                near(pitch, 36.275987284684355);
            }
            other => panic!("helix: {other:?}"),
        }

        let inch =
            load(&write_job("entry-helix-inch", &entry_job("inch", r#""entry":{"type":"helix","radius":0.5,"angle_deg":10}"#))).expect("inch helix");
        match inch.operations()[0].entry() {
            DepthEntry::Helix { radius, pitch } => {
                let radius_mm = 0.5 * 25.4;
                near(radius, radius_mm);
                near(pitch, radius_mm * std::f64::consts::TAU * 10.0_f64.to_radians().tan());
            }
            other => panic!("inch helix: {other:?}"),
        }
    }

    #[test]
    fn rejects_bad_format_tools_and_operations() {
        let missing = load(Path::new("campanion-jobfile-missing.json"));
        assert!(matches!(missing, Err(JobFileError::Io { .. })), "{missing:?}");

        let path = write_job("syntax", "{");
        assert!(matches!(load(&path), Err(JobFileError::Json { .. })));

        let path = write_job("empty-object", "{}");
        let err = load(&path).expect_err("format");
        assert!(err.to_string().contains("missing format tag"), "{err}");

        let path = write_job("tag", r#"{"format":"nope"}"#);
        assert!(matches!(load(&path), Err(JobFileError::UnknownFormat { .. })));

        let path = write_job("major", r#"{"format":"campanion-job/2"}"#);
        assert!(matches!(load(&path), Err(JobFileError::UnsupportedVersion { found: 2, supported: 1 })));

        let path = write_job("minor", r#"{"format":"campanion-job/1.1"}"#);
        assert!(matches!(load(&path), Err(JobFileError::UnknownFormat { .. })));

        let path = write_job("profile-job", include_str!("sample.job.json"));
        let err = load(&path).expect_err("old job");
        assert!(err.to_string().contains("missing format tag"), "{err}");

        let cases = [
            (envelope("/tmp/plate.dxf", "[]", "[]"), "relative"),
            (envelope("..", "[]", "[]"), "name a file"),
            (envelope("plate.dxf", "[]", "[]").replace(EMPTY_HASH, "abc"), "16 hex"),
            (envelope("plate.dxf", r#"[{"id":1,"diameter":6,"kerf":1}]"#, "[]"), "both diameter and kerf"),
            (envelope("plate.dxf", r#"[{"id":1}]"#, "[]"), "diameter or a kerf"),
            (envelope("plate.dxf", r#"[{"id":1,"diameter":6},{"id":1,"kerf":1}]"#, "[]"), "repeated"),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"profile","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "needs a side",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"pocket","layers":["L"],"side":"inside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "only valid on a profile",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"slot","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "unknown operation type",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"drill","layers":["L"],"tool_id":9,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "unknown tool",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":[],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "at least one layer",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":0,"feeds":{"cut":1}}]"#,
                ),
                "step_down",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":10,"step_down":0.001,"feeds":{"cut":1}}]"#,
                ),
                "too many depth passes",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"ramp","angle_deg":0}}]"#,
                ),
                "angle_deg must be greater than 0 and at most 90",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"ramp","angle_deg":-5}}]"#,
                ),
                "angle_deg must be greater than 0 and at most 90",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","radius":2,"angle_deg":90.1}}]"#,
                ),
                "angle_deg must be greater than 0 and at most 90",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","radius":0,"angle_deg":10}}]"#,
                ),
                "helix radius must be positive",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","radius":-1,"angle_deg":10}}]"#,
                ),
                "helix radius must be positive",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","radius":1000001,"angle_deg":10}}]"#,
                ),
                "helix radius is larger than",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","radius":1000000,"angle_deg":45}}]"#,
                ),
                "helix pitch is larger than",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"spiral","angle_deg":10}}]"#,
                ),
                "unknown entry type",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"ramp"}}]"#,
                ),
                "missing `angle_deg`",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"helix","angle_deg":10}}]"#,
                ),
                "missing `radius`",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":{"type":"plunge","angle_deg":90}}]"#,
                ),
                "unknown entry field",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"entry":[]}]"#,
                ),
                "entry must be an object",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
            assert!(matches!(err, JobFileError::Invalid { .. }), "{needle}: {err:?}");
        }
    }

    fn op_job(units: &str, tools: &str, op: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},"units":"{units}","tools":{tools},"operations":[{op}]}}"#
        )
    }

    #[test]
    fn parses_stepover_stock_to_leave_and_finish_in_mm_and_inches() {
        let mm = load(&write_job(
            "stl-mm",
            &op_job(
                "mm",
                r#"[{"id":1,"diameter":6}]"#,
                r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stepover":3,"stock_to_leave":{"wall":0.4,"floor":0.2},"finish":{"step_down":1.5,"floor":false}}"#,
            ),
        ))
        .expect("mm");
        let op = &mm.operations()[0];
        near(op.stepover().expect("stepover"), 3.0);
        near(op.stock_to_leave().wall(), 0.4);
        near(op.stock_to_leave().floor(), 0.2);
        let finish = op.finish().expect("finish");
        near(finish.step_down(), 1.5);
        assert!(!finish.floor());

        let inch = load(&write_job(
            "stl-inch",
            &op_job(
                "inch",
                r#"[{"id":1,"diameter":0.25}]"#,
                r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":0.5,"step_down":0.25,"feeds":{"cut":1},"stepover":0.1,"stock_to_leave":{"wall":0.01,"floor":0.02},"finish":{"step_down":0.1,"floor":true}}"#,
            ),
        ))
        .expect("inch");
        let op = &inch.operations()[0];
        near(op.stepover().expect("stepover"), 0.1 * 25.4);
        near(op.stock_to_leave().wall(), 0.01 * 25.4);
        near(op.stock_to_leave().floor(), 0.02 * 25.4);
        let finish = op.finish().expect("finish");
        near(finish.step_down(), 0.1 * 25.4);
        assert!(finish.floor());

        let profile = load(&write_job(
            "stl-profile",
            &op_job(
                "mm",
                r#"[{"id":1,"diameter":6}]"#,
                r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stock_to_leave":{"wall":0.5},"finish":{"step_down":2}}"#,
            ),
        ))
        .expect("profile");
        let op = &profile.operations()[0];
        assert_eq!(op.stepover(), None);
        near(op.stock_to_leave().wall(), 0.5);
        near(op.stock_to_leave().floor(), 0.0);
        let finish = op.finish().expect("finish");
        near(finish.step_down(), 2.0);
        assert!(finish.floor());
    }

    #[test]
    fn omitted_stepover_stock_to_leave_and_finish_use_defaults() {
        let omitted = load(&write_job(
            "stl-omitted",
            &op_job(
                "mm",
                r#"[{"id":1,"diameter":6}]"#,
                r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("omitted");
        let op = &omitted.operations()[0];
        assert_eq!(op.stepover(), None);
        assert_eq!(op.stock_to_leave(), StockToLeave::default());
        assert_eq!(op.stock_to_leave().wall(), 0.0);
        assert_eq!(op.stock_to_leave().floor(), 0.0);
        assert_eq!(op.finish(), None);

        let empty = load(&write_job(
            "stl-empty",
            &op_job(
                "mm",
                r#"[{"id":1,"diameter":6}]"#,
                r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stock_to_leave":{},"finish":{}}"#,
            ),
        ))
        .expect("empty objects");
        let op = &empty.operations()[0];
        assert_eq!(op.stepover(), None);
        assert_eq!(op.stock_to_leave().wall(), 0.0);
        assert_eq!(op.stock_to_leave().floor(), 0.0);
        let finish = op.finish().expect("finish");
        near(finish.step_down(), 6.0);
        assert!(finish.floor());
    }

    #[test]
    fn rejects_invalid_stepover_stock_to_leave_and_finish() {
        let diameter = r#"[{"id":1,"diameter":6}]"#;
        let kerf = r#"[{"id":1,"kerf":1.2}]"#;
        let cases = [
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stepover":0}"#,
                ),
                "stepover must be greater than 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stepover":6.1}"#,
                ),
                "stepover is larger than the tool diameter",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stepover":1}"#,
                ),
                "stepover is only valid on a pocket",
            ),
            (
                op_job(
                    "mm",
                    kerf,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stepover":0.5}"#,
                ),
                "stepover is not valid with a kerf tool",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stock_to_leave":{"wall":-0.1}}"#,
                ),
                "stock_to_leave wall must be at least 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stock_to_leave":{"floor":6}}"#,
                ),
                "stock_to_leave floor must be less than depth",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"drill","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"finish":{}}"#,
                ),
                "finish is only valid on a profile or pocket",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"finish":{"floor":true}}"#,
                ),
                "finish floor is only valid on a pocket",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"stock_to_leave":{"bar":1}}"#,
                ),
                "unknown stock_to_leave field",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":6,"step_down":2,"feeds":{"cut":1},"finish":{"step_down":0}}"#,
                ),
                "finish step_down must be positive",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":10,"step_down":10,"feeds":{"cut":1},"finish":{"step_down":0.001}}"#,
                ),
                "too many depth passes",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("stl-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
            assert!(matches!(err, JobFileError::Invalid { .. }), "{needle}: {err:?}");
        }
    }

    #[test]
    fn parses_face_angle_margin_and_stepover_in_mm_and_inches() {
        let diameter = r#"[{"id":1,"diameter":6}]"#;
        let explicit = load(&write_job(
            "face-mm",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":0.5,"step_down":0.25,"feeds":{"cut":1},"stepover":4,"angle_deg":90,"margin":2}"#,
            ),
        ))
        .expect("mm face");
        let op = &explicit.operations()[0];
        assert_eq!(op.kind(), OperationKind::Face);
        near(op.stepover().expect("stepover"), 4.0);
        assert_eq!(op.face_angle(), std::f64::consts::FRAC_PI_2);
        near(op.margin(), 2.0);
        let equal = load(&write_job(
            "face-stepover-eq",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"stepover":6}"#,
            ),
        ))
        .expect("stepover equal to diameter");
        near(equal.operations()[0].stepover().expect("stepover"), 6.0);
        assert_eq!(op.stock_to_leave(), StockToLeave::default());
        assert_eq!(op.finish(), None);

        let along_x = load(&write_job(
            "face-angle-0",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":0.5,"step_down":0.25,"feeds":{"cut":1},"angle_deg":0,"margin":0}"#,
            ),
        ))
        .expect("angle 0");
        let op = &along_x.operations()[0];
        assert_eq!(op.face_angle(), 0.0);
        assert_eq!(op.margin(), 0.0);
        near(op.stepover().expect("default stepover"), 6.0 * FACE_STEPOVER_OF_DIAMETER);

        let omitted = load(&write_job(
            "face-defaults",
            &op_job("mm", diameter, r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":0.5,"step_down":0.5,"feeds":{"cut":1}}"#),
        ))
        .expect("defaults");
        let op = &omitted.operations()[0];
        assert_eq!(op.face_angle(), 0.0);
        assert_eq!(op.margin(), 0.0);
        near(op.stepover().expect("default stepover"), 6.0 * 0.75);

        let inch_tool = r#"[{"id":1,"diameter":0.25}]"#;
        let inch = load(&write_job(
            "face-inch",
            &op_job(
                "inch",
                inch_tool,
                r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":0.1,"step_down":0.05,"feeds":{"cut":1},"stepover":0.2,"angle_deg":90,"margin":0.05}"#,
            ),
        ))
        .expect("inch face");
        let op = &inch.operations()[0];
        assert_eq!(op.kind(), OperationKind::Face);
        assert_eq!(op.face_angle(), std::f64::consts::FRAC_PI_2);
        near(op.stepover().expect("stepover"), 0.2 * 25.4);
        near(op.margin(), 0.05 * 25.4);
        near(op.depth(), 0.1 * 25.4);

        let inch_default = load(&write_job(
            "face-inch-default",
            &op_job("inch", inch_tool, r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":0.1,"step_down":0.1,"feeds":{"cut":1}}"#),
        ))
        .expect("inch defaults");
        let op = &inch_default.operations()[0];
        let diameter_mm = 0.25 * 25.4;
        near(inch_default.tool(1).expect("tool").diameter().expect("diameter"), diameter_mm);
        near(op.stepover().expect("default stepover"), diameter_mm * FACE_STEPOVER_OF_DIAMETER);
        assert_eq!(op.face_angle(), 0.0);
        assert_eq!(op.margin(), 0.0);

        // A pocket that omits stepover still stores none. Facing defaults do not leak.
        let pocket = load(&write_job(
            "face-pocket-untouched",
            &op_job("mm", diameter, r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}"#),
        ))
        .expect("pocket");
        let op = &pocket.operations()[0];
        assert_eq!(op.stepover(), None);
        assert_eq!(op.face_angle(), 0.0);
        assert_eq!(op.margin(), 0.0);
    }

    #[test]
    fn rejects_face_fields_on_the_wrong_kind_and_bad_face_values() {
        let diameter = r#"[{"id":1,"diameter":6}]"#;
        let kerf = r#"[{"id":1,"kerf":1.2}]"#;
        let face = r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}"#;
        let cases = [
            (op_job("mm", diameter, &format!("{face},\"side\":\"outside\"}}")), "side is only valid on a profile"),
            (op_job("mm", kerf, &format!("{face}}}")), "face is not valid with a kerf tool"),
            (op_job("mm", kerf, &format!("{face},\"stepover\":0.5}}")), "face is not valid with a kerf tool"),
            (op_job("mm", diameter, &format!("{face},\"stepover\":0}}")), "stepover must be greater than 0"),
            (op_job("mm", diameter, &format!("{face},\"stepover\":6.1}}")), "stepover is larger than the tool diameter"),
            (op_job("mm", diameter, &format!("{face},\"angle_deg\":45}}")), "angle_deg must be 0 or 90"),
            (op_job("mm", diameter, &format!("{face},\"angle_deg\":180}}")), "angle_deg must be 0 or 90"),
            (op_job("mm", diameter, &format!("{face},\"angle_deg\":-90}}")), "angle_deg must be 0 or 90"),
            (op_job("mm", diameter, &format!("{face},\"angle_deg\":90.1}}")), "angle_deg must be 0 or 90"),
            (op_job("mm", diameter, &format!("{face},\"margin\":-0.1}}")), "margin must be at least 0"),
            (op_job("inch", r#"[{"id":1,"diameter":0.25}]"#, &format!("{face},\"margin\":-0.01}}")), "margin must be at least 0"),
            (op_job("mm", diameter, &format!("{face},\"margin\":1000001}}")), "margin is larger than"),
            (op_job("mm", diameter, &format!("{face},\"stock_to_leave\":{{\"wall\":0.1}}}}")), "stock_to_leave is only valid on a profile or pocket"),
            (op_job("mm", diameter, &format!("{face},\"finish\":{{}}}}")), "finish is only valid on a profile or pocket"),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"angle_deg":0}"#,
                ),
                "angle_deg is only valid on a face",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"angle_deg":90}"#,
                ),
                "angle_deg is only valid on a face",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"margin":1}"#,
                ),
                "margin is only valid on a face",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"drill","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"margin":0}"#,
                ),
                "margin is only valid on a face",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"profile","layers":["L"],"side":"on","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"stepover":1}"#,
                ),
                "stepover is only valid on a pocket or a face",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("face-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
            assert!(matches!(err, JobFileError::Invalid { .. }), "{needle}: {err:?}");
        }
    }

    #[test]
    fn parses_chamfer_in_mm_and_inches() {
        let diameter = r#"[{"id":1,"diameter":6}]"#;
        let explicit = load(&write_job(
            "chamfer-mm",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"tip_diameter":0.4,"tip_clearance":0.1,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("mm chamfer");
        let op = &explicit.operations()[0];
        assert_eq!(op.kind(), OperationKind::Chamfer { side: ProfileSide::Outside });
        near(op.width(), 0.5);
        near(op.v_angle(), 90.0_f64.to_radians());
        near(op.tip_diameter(), 0.4);
        near(op.tip_clearance(), 0.1);
        // 90°: tan(a/2) is 1, so the tip depth is width + clearance.
        near(op.depth(), 0.6);
        near(op.step_down(), 0.6);
        assert_eq!(op.stepover(), None);
        assert_eq!(op.face_angle(), 0.0);
        assert_eq!(op.margin(), 0.0);
        assert_eq!(op.stock_to_leave(), StockToLeave::default());
        assert_eq!(op.finish(), None);
        assert_eq!(op.entry(), DepthEntry::Plunge);

        let sharp = load(&write_job(
            "chamfer-60",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"chamfer","layers":["L"],"side":"inside","tool_id":1,"width":1.2,"v_angle_deg":60,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("60 degree");
        let op = &sharp.operations()[0];
        assert_eq!(op.kind(), OperationKind::Chamfer { side: ProfileSide::Inside });
        near(op.width(), 1.2);
        near(op.v_angle(), 60.0_f64.to_radians());
        assert_eq!(op.tip_diameter(), 0.0);
        assert_eq!(op.tip_clearance(), 0.0);
        let expected = 1.2 / (60.0_f64.to_radians() * 0.5).tan();
        near(op.depth(), expected);
        near(op.step_down(), expected);

        let under = load(&write_job(
            "chamfer-tip-under",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.2,"v_angle_deg":90,"tip_diameter":5.999,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("tip just under the diameter");
        near(under.operations()[0].tip_diameter(), 5.999);

        let inch_tool = r#"[{"id":1,"diameter":0.25}]"#;
        let inch = load(&write_job(
            "chamfer-inch",
            &op_job(
                "inch",
                inch_tool,
                r#"{"name":"a","type":"chamfer","layers":["L"],"side":"inside","tool_id":1,"width":0.02,"v_angle_deg":90,"tip_diameter":0.01,"tip_clearance":0.004,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("inch chamfer");
        let op = &inch.operations()[0];
        assert_eq!(op.kind(), OperationKind::Chamfer { side: ProfileSide::Inside });
        near(op.width(), 0.02 * 25.4);
        near(op.v_angle(), 90.0_f64.to_radians());
        near(op.tip_diameter(), 0.01 * 25.4);
        near(op.tip_clearance(), 0.004 * 25.4);
        near(op.depth(), (0.02 + 0.004) * 25.4);
        near(op.step_down(), op.depth());

        let plain = load(&write_job(
            "chamfer-profile-zeros",
            &op_job(
                "mm",
                diameter,
                r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}"#,
            ),
        ))
        .expect("profile");
        let op = &plain.operations()[0];
        assert_eq!(op.width(), 0.0);
        assert_eq!(op.v_angle(), 0.0);
        assert_eq!(op.tip_diameter(), 0.0);
        assert_eq!(op.tip_clearance(), 0.0);
        near(op.depth(), 1.0);
        near(op.step_down(), 1.0);
    }

    #[test]
    fn rejects_bad_chamfer_fields() {
        let diameter = r#"[{"id":1,"diameter":6}]"#;
        let kerf = r#"[{"id":1,"kerf":1.2}]"#;
        let inch_tool = r#"[{"id":1,"diameter":0.25}]"#;
        let cases = [
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "chamfer needs a width",
            ),
            (
                op_job("mm", diameter, r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"feeds":{"cut":1}}"#),
                "chamfer needs a v_angle_deg",
            ),
            (
                op_job("mm", diameter, r#"{"name":"a","type":"chamfer","layers":["L"],"tool_id":1,"width":0.5,"v_angle_deg":90,"feeds":{"cut":1}}"#),
                "chamfer needs a side",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"on","tool_id":1,"width":0.5,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "chamfer side must be outside or inside",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "width must be greater than 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":-0.2,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "width must be greater than 0",
            ),
            (
                op_job(
                    "inch",
                    inch_tool,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":-0.01,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "width must be greater than 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":0,"feeds":{"cut":1}}"#,
                ),
                "v_angle_deg must be greater than 0 and less than 180",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":180,"feeds":{"cut":1}}"#,
                ),
                "v_angle_deg must be greater than 0 and less than 180",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":-10,"feeds":{"cut":1}}"#,
                ),
                "v_angle_deg must be greater than 0 and less than 180",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":180.1,"feeds":{"cut":1}}"#,
                ),
                "v_angle_deg must be greater than 0 and less than 180",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"tip_diameter":-0.1,"feeds":{"cut":1}}"#,
                ),
                "tip_diameter must be at least 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"tip_diameter":6,"feeds":{"cut":1}}"#,
                ),
                "tip_diameter must be less than the tool diameter",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"tip_diameter":6.1,"feeds":{"cut":1}}"#,
                ),
                "tip_diameter must be less than the tool diameter",
            ),
            (
                op_job(
                    "inch",
                    inch_tool,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.02,"v_angle_deg":90,"tip_diameter":0.25,"feeds":{"cut":1}}"#,
                ),
                "tip_diameter must be less than the tool diameter",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"tip_clearance":-0.2,"feeds":{"cut":1}}"#,
                ),
                "tip_clearance must be at least 0",
            ),
            (
                op_job(
                    "inch",
                    inch_tool,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"inside","tool_id":1,"width":0.02,"v_angle_deg":90,"tip_clearance":-0.001,"feeds":{"cut":1}}"#,
                ),
                "tip_clearance must be at least 0",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"depth":1,"feeds":{"cut":1}}"#,
                ),
                "depth is not valid on a chamfer; the depth follows from the geometry",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"step_down":0.5,"feeds":{"cut":1}}"#,
                ),
                "step_down is not valid on a chamfer; the depth follows from the geometry",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}"#,
                ),
                "depth is not valid on a chamfer; the depth follows from the geometry",
            ),
            (
                op_job(
                    "mm",
                    kerf,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "chamfer is not valid with a kerf tool",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"width":0.5}"#,
                ),
                "width is only valid on a chamfer",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"v_angle_deg":90}"#,
                ),
                "v_angle_deg is only valid on a chamfer",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"tip_diameter":0}"#,
                ),
                "tip_diameter is only valid on a chamfer",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"drill","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"tip_clearance":0}"#,
                ),
                "tip_clearance is only valid on a chamfer",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"width":1}"#,
                ),
                "width is only valid on a chamfer",
            ),
            (
                op_job(
                    "mm",
                    diameter,
                    r#"{"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":1000001,"v_angle_deg":90,"feeds":{"cut":1}}"#,
                ),
                "width is larger than",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("chamfer-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
            assert!(matches!(err, JobFileError::Invalid { .. }), "{needle}: {err:?}");
        }
    }

    #[test]
    fn parses_surface_shapes_and_rejects_the_wrong_source() {
        let text = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6,"shape":"ball"},{"id":2,"diameter":10,"shape":{"bull":1}}]"#,
            r#"[{"name":"dome","type":"surface","tool_id":1,"stepover":1,"margin":0.5,"angle_deg":90,"stock_to_leave":0.2,"feeds":{"cut":1000,"plunge":300}}]"#,
        );
        let job = load(&write_job("surface-mm", &text)).expect("surface");
        assert!(job.source().is_stl());
        assert!(!job.source().is_step());
        let op = &job.operations()[0];
        assert_eq!(op.kind(), OperationKind::Surface);
        assert!(op.layers().is_empty());
        near(op.stepover().expect("stepover"), 1.0);
        near(op.sample_step().expect("sample"), 0.5);
        assert_eq!(op.floor_z(), None);
        near(op.margin(), 0.5);
        near(op.face_angle(), std::f64::consts::FRAC_PI_2);
        near(op.stock_to_leave().wall(), 0.0);
        near(op.stock_to_leave().floor(), 0.2);
        assert_eq!(op.depth(), 0.0);
        assert!(matches!(job.tool(1).expect("ball").shape(), ToolShape::Ball));
        match job.tool(2).expect("bull").shape() {
            ToolShape::Bull { corner_radius } => near(corner_radius, 1.0),
            other => panic!("expected a bull nose, got {other:?}"),
        }

        let inch = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"dome.STL","hash":"{EMPTY_HASH}"}},"units":"inch","tools":[{{"id":1,"diameter":0.25,"shape":"flat"}}],"operations":[{{"name":"cap","type":"surface","tool_id":1,"stepover":0.04,"sample_step":0.02,"floor_z":-0.1,"angle_deg":0,"stock_to_leave":0.01,"feeds":{{"cut":10}}}}]}}"#
        );
        let inch = load(&write_job("surface-inch", &inch)).expect("inch surface");
        assert!(inch.source().is_stl());
        let op = &inch.operations()[0];
        near(op.stepover().expect("stepover"), 0.04 * 25.4);
        near(op.sample_step().expect("sample"), 0.02 * 25.4);
        near(op.floor_z().expect("floor"), -0.1 * 25.4);
        near(op.face_angle(), 0.0);
        near(op.stock_to_leave().floor(), 0.01 * 25.4);
        assert!(inch.tool(1).expect("flat").shape().is_flat());

        let surface = r#"{"name":"a","type":"surface","tool_id":1,"stepover":1,"feeds":{"cut":1}}"#;
        let profile = r#"{"name":"a","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}"#;
        let cases = [
            (envelope("plate.dxf", r#"[{"id":1,"diameter":6,"shape":"ball"}]"#, &format!("[{surface}]")), "surface is only valid on an STL source"),
            (
                envelope("part.stl", r#"[{"id":1,"diameter":6}]"#, &format!("[{profile}]")),
                "an STL source only allows surface, rough, waterline or rotary_finish operations",
            ),
            (
                envelope("plate.dxf", r#"[{"id":1,"diameter":6,"shape":"ball"}]"#, &format!("[{profile}]")),
                "shape is only valid on a surface, rough or waterline operation",
            ),
            (envelope("part.stl", r#"[{"id":1,"kerf":1.2}]"#, &format!("[{surface}]")), "surface is not valid with a kerf tool"),
            (envelope("part.stl", r#"[{"id":1,"kerf":1.2,"shape":"ball"}]"#, "[]"), "shape is only valid on a diameter tool"),
            (envelope("part.stl", r#"[{"id":1,"diameter":6,"shape":{"bull":3}}]"#, "[]"), "bull corner radius must be less than the tool radius"),
            (
                envelope("part.stl", r#"[{"id":1,"diameter":6}]"#, r#"[{"name":"a","type":"surface","tool_id":1,"feeds":{"cut":1}}]"#),
                "surface needs a stepover or a scallop_height",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"layers":["L"],"feeds":{"cut":1}}]"#,
                ),
                "layers is not valid on a surface",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"depth":1,"feeds":{"cut":1}}]"#,
                ),
                "depth is not valid on a surface",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"sample_step":0.5}]"#,
                ),
                "sample_step is only valid on a surface",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"floor_z":0}]"#,
                ),
                "floor_z is only valid on a surface",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"stock_to_leave":{"wall":0.1},"feeds":{"cut":1}}]"#,
                ),
                "surface stock_to_leave must be a number",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("surface-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
        }
    }

    #[test]
    fn parses_scallop_height_in_mm_and_inches_and_rejects_both_or_neither() {
        let mm = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
            r#"[{"name":"dome","type":"surface","tool_id":1,"scallop_height":0.01,"feeds":{"cut":1}}]"#,
        );
        let job = load(&write_job("scallop-mm", &mm)).expect("mm scallop");
        let op = &job.operations()[0];
        assert_eq!(op.stepover(), None);
        assert_eq!(op.sample_step(), None);
        near(op.scallop_height().expect("scallop"), 0.01);

        let inch = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"dome.stl","hash":"{EMPTY_HASH}"}},"units":"inch","tools":[{{"id":1,"diameter":0.25,"shape":{{"bull":0.05}}}}],"operations":[{{"name":"cap","type":"surface","tool_id":1,"scallop_height":0.01,"sample_step":0.02,"feeds":{{"cut":10}}}}]}}"#
        );
        let inch = load(&write_job("scallop-inch", &inch)).expect("inch scallop");
        let op = &inch.operations()[0];
        assert_eq!(op.stepover(), None);
        near(op.scallop_height().expect("scallop"), 0.01 * 25.4);
        near(op.sample_step().expect("sample"), 0.02 * 25.4);

        // A flat tool loads. The run is what rejects a scallop on a flat.
        let flat = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6,"shape":"flat"}]"#,
            r#"[{"name":"dome","type":"surface","tool_id":1,"scallop_height":0.2,"feeds":{"cut":1}}]"#,
        );
        let flat = load(&write_job("scallop-flat", &flat)).expect("flat scallop loads");
        assert!(flat.tool(1).expect("flat").shape().is_flat());
        near(flat.operations()[0].scallop_height().expect("scallop"), 0.2);
        assert_eq!(flat.operations()[0].stepover(), None);

        let cases = [
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"scallop_height":0.01,"feeds":{"cut":1}}]"#,
                ),
                "surface takes stepover or scallop_height, not both",
            ),
            (
                envelope("part.stl", r#"[{"id":1,"diameter":6,"shape":"ball"}]"#, r#"[{"name":"a","type":"surface","tool_id":1,"feeds":{"cut":1}}]"#),
                "surface needs a stepover or a scallop_height",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"scallop_height":0,"feeds":{"cut":1}}]"#,
                ),
                "scallop_height must be greater than 0",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"scallop_height":-0.01,"feeds":{"cut":1}}]"#,
                ),
                "scallop_height must be greater than 0",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1},"scallop_height":0.1}]"#,
                ),
                "scallop_height is only valid on a surface",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("scallop-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
            assert!(matches!(err, JobFileError::Invalid { .. }), "{needle}: {err:?}");
        }
    }

    #[test]
    fn parses_rough_and_rejects_other_kinds_keys() {
        let text = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6,"shape":"flat"},{"id":2,"diameter":8,"shape":"ball"}]"#,
            r#"[{"name":"rough","type":"rough","tool_id":1,"step_down":2.5,"stepover":2.4,"stock_to_leave":0.3,"margin":1,"top_z":12,"floor_z":-1,"grid":0.4,"feeds":{"cut":1000,"plunge":300}}]"#,
        );
        let job = load(&write_job("rough-mm", &text)).expect("rough");
        assert!(job.source().is_stl());
        let op = &job.operations()[0];
        assert_eq!(op.kind(), OperationKind::Rough);
        assert!(op.layers().is_empty());
        near(op.step_down(), 2.5);
        near(op.stepover().expect("stepover"), 2.4);
        near(op.stock_to_leave().wall(), 0.0);
        near(op.stock_to_leave().floor(), 0.3);
        near(op.margin(), 1.0);
        near(op.top_z().expect("top"), 12.0);
        near(op.floor_z().expect("floor"), -1.0);
        near(op.grid().expect("grid"), 0.4);
        assert_eq!(op.depth(), 0.0);
        assert!(op.sample_step().is_none());
        assert!(op.scallop_height().is_none());
        assert!(matches!(op.entry(), DepthEntry::Plunge));
        assert!(job.tool(1).expect("flat").shape().is_flat());

        let ramp = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"rough","type":"rough","tool_id":1,"step_down":1,"entry":{"type":"ramp","angle_deg":10},"feeds":{"cut":1}}]"#,
        );
        let ramp = load(&write_job("rough-ramp", &ramp)).expect("rough ramp");
        match ramp.operations()[0].entry() {
            DepthEntry::Ramp { max_angle } => near(max_angle, 10.0_f64.to_radians()),
            other => panic!("rough ramp: {other:?}"),
        }
        let helix = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"rough","type":"rough","tool_id":1,"step_down":1,"entry":{"type":"helix","radius":2,"angle_deg":3},"feeds":{"cut":1}}]"#,
        );
        let helix = load(&write_job("rough-helix", &helix)).expect("rough helix");
        match helix.operations()[0].entry() {
            DepthEntry::Helix { radius, pitch } => {
                near(radius, 2.0);
                near(pitch, 2.0 * std::f64::consts::TAU * 3.0_f64.to_radians().tan());
            }
            other => panic!("rough helix: {other:?}"),
        }

        let defaults =
            envelope("dome.stl", r#"[{"id":1,"diameter":10}]"#, r#"[{"name":"rough","type":"rough","tool_id":1,"step_down":1,"feeds":{"cut":1}}]"#);
        let defaults = load(&write_job("rough-defaults", &defaults)).expect("defaults");
        let op = &defaults.operations()[0];
        near(op.stepover().expect("stepover"), 4.0);
        near(op.margin(), 5.0);
        near(op.stock_to_leave().floor(), 0.5);
        near(op.grid().expect("grid"), 0.5);
        assert_eq!(op.top_z(), None);
        assert_eq!(op.floor_z(), None);

        let fine = envelope(
            "part.stl",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"rough","type":"rough","tool_id":1,"step_down":1,"stepover":1,"feeds":{"cut":1}}]"#,
        );
        let fine = load(&write_job("rough-grid", &fine)).expect("grid");
        near(fine.operations()[0].grid().expect("grid"), 0.25);

        let inch = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"dome.STL","hash":"{EMPTY_HASH}"}},"units":"inch","tools":[{{"id":1,"diameter":0.25,"shape":"flat"}}],"operations":[{{"name":"cap","type":"rough","tool_id":1,"step_down":0.1,"stepover":0.08,"stock_to_leave":0.01,"margin":0.05,"top_z":0.4,"floor_z":-0.1,"grid":0.02,"feeds":{{"cut":10}}}}]}}"#
        );
        let inch = load(&write_job("rough-inch", &inch)).expect("inch rough");
        let op = &inch.operations()[0];
        near(op.step_down(), 0.1 * 25.4);
        near(op.stepover().expect("stepover"), 0.08 * 25.4);
        near(op.stock_to_leave().floor(), 0.01 * 25.4);
        near(op.margin(), 0.05 * 25.4);
        near(op.top_z().expect("top"), 0.4 * 25.4);
        near(op.floor_z().expect("floor"), -0.1 * 25.4);
        near(op.grid().expect("grid"), 0.02 * 25.4);

        let rough = r#"{"name":"a","type":"rough","tool_id":1,"step_down":1,"feeds":{"cut":1}}"#;
        let cases = [
            (envelope("plate.dxf", r#"[{"id":1,"diameter":6}]"#, &format!("[{rough}]")), "rough is only valid on an STL source"),
            (envelope("part.stl", r#"[{"id":1,"kerf":1.2}]"#, &format!("[{rough}]")), "rough is not valid with a kerf tool"),
            (
                envelope("part.stl", r#"[{"id":1,"diameter":6}]"#, r#"[{"name":"a","type":"rough","tool_id":1,"feeds":{"cut":1}}]"#),
                "rough needs a step_down",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"stepover":7,"feeds":{"cut":1}}]"#,
                ),
                "stepover is larger than the tool diameter",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"layers":["L"],"feeds":{"cut":1}}]"#,
                ),
                "layers is not valid on a rough",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"depth":1,"feeds":{"cut":1}}]"#,
                ),
                "depth is not valid on a rough",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"scallop_height":0.01,"feeds":{"cut":1}}]"#,
                ),
                "scallop_height is not valid on a rough",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"sample_step":0.2,"feeds":{"cut":1}}]"#,
                ),
                "sample_step is not valid on a rough",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"angle_deg":0,"feeds":{"cut":1}}]"#,
                ),
                "angle_deg is not valid on a rough",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"entry":{"type":"spiral","angle_deg":10},"feeds":{"cut":1}}]"#,
                ),
                "unknown entry type",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"stock_to_leave":{"wall":0.1},"feeds":{"cut":1}}]"#,
                ),
                "rough stock_to_leave must be a number",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"rough","tool_id":1,"step_down":1,"top_z":0,"floor_z":1,"feeds":{"cut":1}}]"#,
                ),
                "top_z must be above floor_z",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"top_z":1,"feeds":{"cut":1}}]"#,
                ),
                "top_z is not valid on a surface",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"grid":0.2,"feeds":{"cut":1}}]"#,
                ),
                "grid is not valid on a surface",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"top_z":1,"feeds":{"cut":1}}]"#,
                ),
                "top_z is only valid on a rough",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"pocket","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"grid":0.2,"feeds":{"cut":1}}]"#,
                ),
                "grid is only valid on a rough",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("rough-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
        }

        let ball = envelope(
            "part.stl",
            r#"[{"id":2,"diameter":8,"shape":"ball"}]"#,
            r#"[{"name":"a","type":"rough","tool_id":2,"step_down":1,"feeds":{"cut":1}}]"#,
        );
        let ball = load(&write_job("rough-ball", &ball)).expect("ball on rough");
        assert!(matches!(ball.tool(2).expect("ball").shape(), ToolShape::Ball));
        assert_eq!(ball.operations()[0].kind(), OperationKind::Rough);
    }

    #[test]
    fn parses_waterline_defaults_and_rejects_foreign_keys() {
        let text = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
            r#"[{"name":"walls","type":"waterline","tool_id":1,"step_down":0.5,"min_slope_deg":30,"margin":1,"top_z":12,"floor_z":0,"grid":0.4,"stock_to_leave":0.2,"feeds":{"cut":1000,"plunge":300}}]"#,
        );
        let job = load(&write_job("waterline-mm", &text)).expect("waterline");
        let op = &job.operations()[0];
        assert_eq!(op.kind(), OperationKind::Waterline);
        assert!(op.layers().is_empty());
        near(op.step_down(), 0.5);
        near(op.min_slope_deg(), 30.0);
        near(op.margin(), 1.0);
        near(op.top_z().expect("top"), 12.0);
        near(op.floor_z().expect("floor"), 0.0);
        near(op.grid().expect("grid"), 0.4);
        near(op.stock_to_leave().floor(), 0.2);
        near(op.stock_to_leave().wall(), 0.0);
        assert_eq!(op.stepover(), None);
        assert!(matches!(op.entry(), DepthEntry::Plunge));

        let defaults = envelope(
            "dome.stl",
            r#"[{"id":1,"diameter":10}]"#,
            r#"[{"name":"walls","type":"waterline","tool_id":1,"step_down":1,"feeds":{"cut":1}}]"#,
        );
        let defaults = load(&write_job("waterline-defaults", &defaults)).expect("defaults");
        let op = &defaults.operations()[0];
        near(op.margin(), 5.0);
        near(op.grid().expect("grid"), 0.25);
        near(op.stock_to_leave().floor(), 0.0);
        near(op.min_slope_deg(), 0.0);
        assert_eq!(op.top_z(), None);
        assert_eq!(op.floor_z(), None);

        let fine = envelope(
            "part.stl",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"walls","type":"waterline","tool_id":1,"step_down":0.1,"feeds":{"cut":1}}]"#,
        );
        let fine = load(&write_job("waterline-grid", &fine)).expect("fine grid");
        near(fine.operations()[0].grid().expect("grid"), 0.1);

        let inch = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"dome.STL","hash":"{EMPTY_HASH}"}},"units":"inch","tools":[{{"id":1,"diameter":0.25,"shape":"ball"}}],"operations":[{{"name":"walls","type":"waterline","tool_id":1,"step_down":0.1,"min_slope_deg":45,"margin":0.05,"top_z":0.4,"floor_z":0,"grid":0.02,"stock_to_leave":0.01,"feeds":{{"cut":10}}}}]}}"#
        );
        let inch = load(&write_job("waterline-inch", &inch)).expect("inch waterline");
        let op = &inch.operations()[0];
        near(op.step_down(), 0.1 * 25.4);
        near(op.min_slope_deg(), 45.0);
        near(op.margin(), 0.05 * 25.4);
        near(op.top_z().expect("top"), 0.4 * 25.4);
        near(op.floor_z().expect("floor"), 0.0);
        near(op.grid().expect("grid"), 0.02 * 25.4);
        near(op.stock_to_leave().floor(), 0.01 * 25.4);

        let inch_default = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"dome.stl","hash":"{EMPTY_HASH}"}},"units":"inch","tools":[{{"id":1,"diameter":0.25}}],"operations":[{{"name":"walls","type":"waterline","tool_id":1,"step_down":0.02,"feeds":{{"cut":1}}}}]}}"#
        );
        let inch_default = load(&write_job("waterline-inch-grid", &inch_default)).expect("inch default grid");
        // step_down is 0.508 mm, so the unscaled 0.25 mm cap is the default grid.
        near(inch_default.operations()[0].grid().expect("grid"), 0.25);
        near(inch_default.operations()[0].margin(), 0.125 * 25.4);

        let line = r#"{"name":"a","type":"waterline","tool_id":1,"step_down":1,"feeds":{"cut":1}}"#;
        let cases = [
            (envelope("plate.dxf", r#"[{"id":1,"diameter":6}]"#, &format!("[{line}]")), "waterline is only valid on an STL source"),
            (envelope("part.stl", r#"[{"id":1,"kerf":1.2}]"#, &format!("[{line}]")), "waterline is not valid with a kerf tool"),
            (
                envelope("part.stl", r#"[{"id":1,"diameter":6}]"#, r#"[{"name":"a","type":"waterline","tool_id":1,"feeds":{"cut":1}}]"#),
                "waterline needs a step_down",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"layers":["L"],"feeds":{"cut":1}}]"#,
                ),
                "layers is not valid on a waterline",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"stepover":1,"feeds":{"cut":1}}]"#,
                ),
                "stepover is not valid on a waterline",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"min_slope_deg":91,"feeds":{"cut":1}}]"#,
                ),
                "min_slope_deg must lie in [0, 90]",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"min_slope_deg":-1,"feeds":{"cut":1}}]"#,
                ),
                "min_slope_deg must lie in [0, 90]",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"stock_to_leave":{"wall":0.1},"feeds":{"cut":1}}]"#,
                ),
                "waterline stock_to_leave must be a number",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"waterline","tool_id":1,"step_down":1,"top_z":0,"floor_z":1,"feeds":{"cut":1}}]"#,
                ),
                "top_z must be above floor_z",
            ),
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"face","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"min_slope_deg":30,"feeds":{"cut":1}}]"#,
                ),
                "min_slope_deg is only valid on a waterline",
            ),
            (
                envelope(
                    "part.stl",
                    r#"[{"id":1,"diameter":6}]"#,
                    r#"[{"name":"a","type":"surface","tool_id":1,"stepover":1,"min_slope_deg":30,"feeds":{"cut":1}}]"#,
                ),
                "min_slope_deg is not valid on a surface",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let path = write_job(&format!("waterline-bad-{index}"), text);
            let err = load(&path).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
        }
    }

    #[test]
    fn rotary_setup_and_index_angles_load_and_inches_scale_the_centre_only() {
        let text = r#"{
          "format": "campanion-job/1",
          "source": { "path": "dome.stl", "hash": "cbf29ce484222325" },
          "units": "inch",
          "rotary": {
            "axis": "b",
            "center": [1.0, 0.0, -2.0],
            "direction": "negative",
            "limits": { "min_deg": -30.0, "max_deg": 120.5 }
          },
          "tools": [ { "id": 1, "diameter": 0.25, "shape": "ball" } ],
          "operations": [
            {
              "name": "face",
              "type": "surface",
              "tool_id": 1,
              "stepover": 0.1,
              "index_deg": 90.0,
              "feeds": { "cut": 10.0 }
            },
            {
              "name": "other",
              "type": "rough",
              "tool_id": 1,
              "step_down": 0.05,
              "feeds": { "cut": 10.0 }
            }
          ]
        }"#;
        let job = load(&write_job("rotary-inch", text)).expect("rotary job");
        let setup = job.rotary().expect("rotary");
        assert_eq!(setup.axis(), RotaryAxisKind::B);
        assert_eq!(setup.direction(), RotaryDirection::Negative);
        near(setup.center().x(), 25.4);
        near(setup.center().y(), 0.0);
        near(setup.center().z(), -50.8);
        let limits = setup.limits().expect("limits");
        near(limits.min_deg(), -30.0);
        near(limits.max_deg(), 120.5);
        assert_eq!(job.operations()[0].index_deg(), Some(90.0));
        assert_eq!(job.operations()[1].index_deg(), None);

        let plain = envelope(
            "plate.dxf",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"outer","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
        );
        let plain = load(&write_job("rotary-absent", &plain)).expect("plain");
        assert!(plain.rotary().is_none());
        assert_eq!(plain.operations()[0].index_deg(), None);
    }

    #[test]
    fn index_without_rotary_names_the_operation_and_bad_rotary_fields_fail() {
        let indexed = envelope(
            "plate.dxf",
            r#"[{"id":1,"diameter":6}]"#,
            r#"[{"name":"outer","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"index_deg":45,"feeds":{"cut":1}}]"#,
        );
        let err = load(&write_job("index-without-rotary", &indexed)).expect_err("index");
        assert!(err.to_string().contains("operation `outer`"), "{err}");
        assert!(err.to_string().contains("index_deg needs a rotary setup"), "{err}");

        let base = |rotary: &str, op: &str| {
            format!(
                r#"{{"format":"campanion-job/1","source":{{"path":"dome.stl","hash":"{EMPTY_HASH}"}},"units":"mm","rotary":{rotary},"tools":[{{"id":1,"diameter":6,"shape":"ball"}}],"operations":[{op}]}}"#
            )
        };
        let op = r#"{"name":"face","type":"surface","tool_id":1,"stepover":1,"feeds":{"cut":1}}"#;
        let cases = [
            (base(r#"{"axis":"c","center":[0,0,0],"direction":"positive"}"#, op), "rotary axis must be"),
            (base(r#"{"axis":"a","center":[0,0,0],"direction":"cw"}"#, op), "rotary direction must be"),
            (base(r#"{"axis":"A","center":[0,0,0],"direction":"positive"}"#, op), "rotary axis must be"),
            (base(r#"{"axis":"a","center":[0,0],"direction":"positive"}"#, op), "exactly 3"),
            (base(r#"{"axis":"a","center":[0,0,0],"direction":"positive","limits":{"min_deg":10,"max_deg":10}}"#, op), "rotary limit"),
            (base(r#"{"axis":"a","center":[0,0,0],"direction":"positive","limits":{"min_deg":90,"max_deg":-90}}"#, op), "rotary limit"),
            (
                base(
                    r#"{"axis":"a","center":[0,0,0],"direction":"positive"}"#,
                    r#"{"name":"face","type":"surface","tool_id":1,"stepover":1,"index_deg":"90","feeds":{"cut":1}}"#,
                ),
                "index_deg must be a number",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let err = load(&write_job(&format!("rotary-bad-{index}"), text)).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
        }
    }

    fn rotary_job(units: &str, tools: &str, ops: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},"units":"{units}","rotary":{{"axis":"a","center":[0,0,0],"direction":"positive"}},"tools":{tools},"operations":{ops}}}"#
        )
    }

    #[test]
    fn wrap_loads_offsets_and_rejects_a_missing_rotary_or_a_bad_diameter() {
        let letters = r#"{"name":"letters","type":"wrap","layers":["LETTERS"],"tool_id":1,"depth":0.5,"step_down":0.25,"diameter":40,"feeds":{"cut":800,"plunge":200}}"#;
        let missing = envelope("plate.dxf", r#"[{"id":1,"diameter":3}]"#, &format!("[{letters}]"));
        let err = load(&write_job("wrap-no-rotary", &missing)).expect_err("rotary");
        assert!(err.to_string().contains("operation `letters`"), "{err}");
        assert!(err.to_string().contains("wrap needs a rotary setup"), "{err}");

        let loaded = load(&write_job("wrap-mm", &rotary_job("mm", r#"[{"id":1,"diameter":3}]"#, &format!("[{letters}]")))).expect("wrap");
        let op = &loaded.operations()[0];
        assert_eq!(op.kind(), OperationKind::Wrap);
        assert_eq!(op.cylinder_diameter(), Some(40.0));
        near(op.u_offset(), 0.0);
        near(op.v_offset(), 0.0);
        near(op.depth(), 0.5);
        near(op.step_down(), 0.25);
        assert!(op.index_deg().is_none());

        let inches = r#"{"name":"letters","type":"wrap","layers":["LETTERS"],"tool_id":1,"depth":0.1,"step_down":0.05,"diameter":1,"u_offset":0.5,"v_offset":-0.25,"feeds":{"cut":10,"plunge":4}}"#;
        let scaled = load(&write_job("wrap-inch", &rotary_job("inch", r#"[{"id":1,"diameter":0.125}]"#, &format!("[{inches}]")))).expect("inch wrap");
        let op = &scaled.operations()[0];
        near(op.cylinder_diameter().expect("diameter"), 25.4);
        near(op.u_offset(), 12.7);
        near(op.v_offset(), -6.35);
        near(op.depth(), 2.54);
        near(op.feeds().cut(), 254.0);

        let kerf = r#"{"name":"letters","type":"wrap","layers":["L"],"tool_id":2,"depth":1,"step_down":1,"diameter":40,"feeds":{"cut":1}}"#;
        let kerf_job = load(&write_job("wrap-kerf", &rotary_job("mm", r#"[{"id":2,"kerf":0.2}]"#, &format!("[{kerf}]")))).expect("kerf");
        assert_eq!(kerf_job.operations()[0].kind(), OperationKind::Wrap);

        let cases = [
            (
                envelope(
                    "plate.dxf",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]"#,
                ),
                "wrap needs a diameter",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":0,"feeds":{"cut":1}}]"#,
                ),
                "diameter must be greater than 0",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":-2,"feeds":{"cut":1}}]"#,
                ),
                "diameter must be greater than 0",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":"40","feeds":{"cut":1}}]"#,
                ),
                "diameter must be a number",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":1000001,"feeds":{"cut":1}}]"#,
                ),
                "diameter is larger than",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":40,"index_deg":90,"feeds":{"cut":1}}]"#,
                ),
                "index_deg is not valid on a wrap",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"mark","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":40,"feeds":{"cut":1}}]"#,
                ),
                "diameter is only valid on a wrap",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"mark","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"u_offset":1,"feeds":{"cut":1}}]"#,
                ),
                "u_offset is only valid on a wrap",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3}]"#,
                    r#"[{"name":"mark","type":"engrave","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"v_offset":1,"feeds":{"cut":1}}]"#,
                ),
                "v_offset is only valid on a wrap",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":6,"shape":"ball"}]"#,
                    r#"[{"name":"face","type":"surface","tool_id":1,"stepover":1,"diameter":40,"feeds":{"cut":1}}]"#,
                ),
                "diameter is not valid on a surface",
            ),
            (
                rotary_job(
                    "mm",
                    r#"[{"id":1,"diameter":3,"shape":"ball"}]"#,
                    r#"[{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":40,"feeds":{"cut":1}}]"#,
                ),
                "shape is only valid on a surface",
            ),
            (
                format!(
                    r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"{EMPTY_HASH}"}},"units":"mm","rotary":{{"axis":"a","center":[0,0,0],"direction":"positive"}},"tools":[{{"id":1,"diameter":6}}],"operations":[{{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":40,"feeds":{{"cut":1}}}}]}}"#
                ),
                "an STL source only allows surface, rough, waterline or rotary_finish",
            ),
        ];
        for (index, (text, needle)) in cases.iter().enumerate() {
            let err = load(&write_job(&format!("wrap-bad-{index}"), text)).expect_err(needle);
            assert!(err.to_string().contains(needle), "{needle}: {err}");
        }
    }

    fn invalid_detail(err: &JobFileError) -> String {
        let text = err.to_string();
        text.strip_prefix("invalid input: ").unwrap_or(&text).to_string()
    }

    fn write_beside(job: &Path, name: &str, text: &str) {
        fs::write(job.parent().expect("job dir").join(name), text).expect("beside");
    }

    fn example_kinematics() -> String {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/table_table_bc.kin.json");
        fs::read_to_string(path).expect("example kinematics")
    }

    fn profile_op(name: &str, extra: &str) -> String {
        format!(
            r#"{{"name":"{name}","type":"profile","layers":["L"],"side":"outside","tool_id":1,"depth":1,"step_down":1{extra},"feeds":{{"cut":1}}}}"#
        )
    }

    fn kin_job(units: &str, kinematics: &str, solution: &str, rotary: &str, ops: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{EMPTY_HASH}"}},"units":"{units}"{kinematics}{solution}{rotary},"tools":[{{"id":1,"diameter":6}}],"operations":[{ops}]}}"#
        )
    }

    #[test]
    fn kinematics_and_a_tilted_plane_load() {
        let kin = example_kinematics();
        let flat = profile_op("flat", r#","plane":{"origin":[1,2,3],"normal":[0,0,2]}"#);
        let side = profile_op("side", r#","plane":{"origin":[0,0,0],"normal":[1,0,0],"x_dir":[0,4,0]}"#);
        let plain = profile_op("plain", "");
        let text =
            kin_job("inch", r#","kinematics":"table_table_bc.kin.json""#, r#","solution":"first_positive""#, "", &format!("{flat},{side},{plain}"));
        let path = write_job("kin-plane", &text);
        write_beside(&path, "table_table_bc.kin.json", &kin);
        let job = load(&path).expect("load");
        let setup = job.five_axis().expect("five axis");
        assert_eq!(setup.policy, SolutionPolicy::FirstPositive);
        assert_eq!(setup.kinematics.name, "table_table_bc");
        let flat = job.operations()[0].plane().expect("flat");
        near(flat.origin().x(), 25.4);
        near(flat.origin().y(), 50.8);
        near(flat.origin().z(), 76.2);
        assert_eq!(flat.normal(), Vec3::Z);
        assert_eq!(flat.x_dir(), Vec3::X);
        let side = job.operations()[1].plane().expect("side");
        assert_eq!(side.normal(), Vec3::X);
        assert_eq!(side.x_dir(), Vec3::Y);
        assert!(job.operations()[2].plane().is_none());

        let omitted = kin_job("mm", r#","kinematics":"table_table_bc.kin.json""#, "", "", &profile_op("plain", ""));
        let path = write_job("kin-default-solution", &omitted);
        write_beside(&path, "table_table_bc.kin.json", &kin);
        assert_eq!(load(&path).expect("default").five_axis().expect("setup").policy, SolutionPolicy::Nearest);

        let negative = kin_job("mm", r#","kinematics":"table_table_bc.kin.json""#, r#","solution":"first_negative""#, "", &profile_op("plain", ""));
        let path = write_job("kin-negative", &negative);
        write_beside(&path, "table_table_bc.kin.json", &kin);
        assert_eq!(load(&path).expect("negative").five_axis().expect("setup").policy, SolutionPolicy::FirstNegative);
    }

    #[test]
    fn kinematics_and_plane_errors_name_the_failure() {
        let op = profile_op("outer", "");
        let missing = kin_job("mm", r#","kinematics":"missing.kin.json""#, "", "", &op);
        let err = load(&write_job("kin-missing", &missing)).expect_err("missing");
        assert!(invalid_detail(&err).starts_with("kinematics: "), "{err}");

        let bad = kin_job("mm", r#","kinematics":"bad.kin.json""#, "", "", &op);
        let path = write_job("kin-bad-json", &bad);
        write_beside(&path, "bad.kin.json", "{");
        let err = load(&path).expect_err("json");
        assert!(invalid_detail(&err).starts_with("kinematics: "), "{err}");

        let solution = kin_job("mm", "", r#","solution":"nearest""#, "", &op);
        let err = load(&write_job("solution-without-kin", &solution)).expect_err("solution");
        assert_eq!(invalid_detail(&err), "solution needs a kinematics file");

        let both =
            kin_job("mm", r#","kinematics":"table_table_bc.kin.json""#, "", r#","rotary":{"axis":"a","center":[0,0,0],"direction":"positive"}"#, &op);
        let path = write_job("kin-and-rotary", &both);
        write_beside(&path, "table_table_bc.kin.json", &example_kinematics());
        let err = load(&path).expect_err("combined");
        assert_eq!(invalid_detail(&err), "kinematics and rotary cannot be combined");

        let needs = kin_job("mm", "", "", "", &profile_op("outer", r#","plane":{"origin":[0,0,0],"normal":[0,0,1]}"#));
        let err = load(&write_job("plane-needs-kin", &needs)).expect_err("plane");
        assert_eq!(invalid_detail(&err), "operation `outer`: plane needs a kinematics file");

        let zero = kin_job("mm", "", "", "", &profile_op("outer", r#","plane":{"origin":[0,0,0],"normal":[0,0,0]}"#));
        let err = load(&write_job("plane-zero", &zero)).expect_err("zero");
        assert_eq!(invalid_detail(&err), "operation `outer`: plane normal is zero");

        let parallel = kin_job("mm", "", "", "", &profile_op("outer", r#","plane":{"origin":[0,0,0],"normal":[0,0,1],"x_dir":[0,0,2]}"#));
        let err = load(&write_job("plane-parallel", &parallel)).expect_err("parallel");
        assert_eq!(invalid_detail(&err), "operation `outer`: plane x_dir is parallel to the normal");

        let indexed = kin_job("mm", "", "", "", &profile_op("outer", r#","index_deg":10,"plane":{"origin":[0,0,0],"normal":[0,0,1]}"#));
        let err = load(&write_job("plane-index", &indexed)).expect_err("index");
        assert_eq!(invalid_detail(&err), "operation `outer`: plane and index_deg cannot be combined");

        let wrap = kin_job(
            "mm",
            "",
            "",
            "",
            r#"{"name":"letters","type":"wrap","layers":["L"],"tool_id":1,"depth":1,"step_down":1,"diameter":40,"plane":{"origin":[0,0,0],"normal":[0,0,1]},"feeds":{"cut":1}}"#,
        );
        let err = load(&write_job("plane-wrap", &wrap)).expect_err("wrap");
        assert_eq!(invalid_detail(&err), "operation `letters`: plane is not supported on wrap");

        let finish = kin_job(
            "mm",
            "",
            "",
            "",
            r#"{"name":"spin","type":"rotary_finish","tool_id":1,"plane":{"origin":[0,0,0],"normal":[0,0,1]},"feeds":{"cut":1}}"#,
        );
        let err = load(&write_job("plane-rotary-finish", &finish)).expect_err("rotary finish");
        assert_eq!(invalid_detail(&err), "operation `spin`: plane is not supported on rotary_finish");

        let surface = kin_job(
            "mm",
            "",
            "",
            "",
            r#"{"name":"cap","type":"surface","tool_id":1,"stepover":1,"plane":{"origin":[0,0,0],"normal":[0,0,1]},"feeds":{"cut":1}}"#,
        );
        let err = load(&write_job("plane-surface", &surface)).expect_err("surface");
        assert_eq!(invalid_detail(&err), "operation `cap`: plane is not supported on surface");

        let tilt = kin_job(
            "mm",
            r#","kinematics":"table_table_bc.kin.json""#,
            "",
            "",
            &profile_op("tilt", r#","plane":{"origin":[1,2,3],"normal":[0,0,-1]}"#),
        );
        let path = write_job("plane-past-b", &tilt);
        write_beside(&path, "table_table_bc.kin.json", &example_kinematics());
        let err = load(&path).expect_err("limits");
        assert_eq!(invalid_detail(&err), "operation `tilt`: tool axis (0.000000, 0.000000, -1.000000) is outside the rotary limits");

        let tight = r#"{"name":"tight_b","rotaries":[{"letter":"B","carrier":"table","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0},"limits":{"min_deg":10.0,"max_deg":20.0}},{"letter":"C","carrier":"table","axis":{"x":0.0,"y":0.0,"z":1.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}}]}"#;
        let upright = kin_job("mm", r#","kinematics":"tight.kin.json""#, "", "", &profile_op("inner", ""));
        let path = write_job("plane-plus-z", &upright);
        write_beside(&path, "tight.kin.json", tight);
        let err = load(&path).expect_err("plus z");
        assert_eq!(invalid_detail(&err), "operation `inner`: tool axis (0.000000, 0.000000, 1.000000) is outside the rotary limits");
    }
}
