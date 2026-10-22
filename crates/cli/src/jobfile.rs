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

use std::fmt;
use std::fs::File;
use std::io::Read;
use std::path::{Component, Path, PathBuf};

use campanion_geom::Tolerance;
use campanion_ir::{Feeds, ProcessParams, Tool as IrTool};
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
const MAX_FEED_MM_PER_MIN: f64 = 1.0e9;
const MAX_DELAY_S: f64 = 1.0e6;
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
}

fn path_is_step(path: &str) -> bool {
    Path::new(path)
        .extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| extension.eq_ignore_ascii_case("step") || extension.eq_ignore_ascii_case("stp"))
}

/// One cutter. `id` matches [`IrTool::id`].
#[derive(Clone, Debug, PartialEq)]
pub struct JobTool {
    id: u32,
    path_tool: PathTool,
    ir: IrTool,
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
/// `campanion-path` depth plan (at most 1024 passes, each within 1e6 mm). Heights on
/// [`Self::params`] are absolute Z. [`Self::entry`] is how each pass is entered.
/// An omitted entry is a plunge. [`Self::stock_to_leave`] is millimetres of material
/// left by roughing; an omitted [`Self::finish`] means no finishing pass.
#[derive(Clone, Debug, PartialEq)]
pub struct Operation {
    name: String,
    kind: OperationKind,
    layers: Vec<String>,
    tool_id: u32,
    depth: f64,
    step_down: f64,
    stepover: Option<f64>,
    stock_to_leave: StockToLeave,
    finish: Option<Finish>,
    entry: DepthEntry,
    feeds: Feeds,
    params: ProcessParams,
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
    pub fn depth(&self) -> f64 {
        self.depth
    }

    /// Maximum thickness of one depth pass, in millimetres.
    pub fn step_down(&self) -> f64 {
        self.step_down
    }

    /// Pocket radial stepover in millimetres. `None` when the file omits it.
    pub fn stepover(&self) -> Option<f64> {
        self.stepover
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
}

/// A loaded `campanion-job/1` file.
///
/// [`Self::clearance`] and [`Self::retract`] are millimetres. They come from the optional
/// job-level `heights` object. An omitted clearance is 5 mm and an omitted retract is 2 mm.
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
}

struct Body {
    source: Source,
    units: Units,
    tolerance: Tolerance,
    tools: Vec<JobTool>,
    operations: Vec<Operation>,
    clearance: f64,
    retract: f64,
}

/// Read a `campanion-job/1` file. Lengths in the result are millimetres.
pub fn load(path: &Path) -> Result<JobFile, JobFileError> {
    let text = read_text(path)?;
    let body = parse_text(&text)?;
    Ok(JobFile {
        path: path.to_path_buf(),
        source: body.source,
        units: body.units,
        tolerance: body.tolerance,
        tools: body.tools,
        operations: body.operations,
        clearance: body.clearance,
        retract: body.retract,
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

fn parse_text(text: &str) -> Result<Body, JobFileError> {
    let value: Value = serde_json::from_str(text).map_err(|err| JobFileError::Json { message: err.to_string() })?;
    let map = object(&value, "job").map_err(from_cli)?;
    let tag = match optional(map, "format") {
        Some(value) => as_str(value, "format tag").map_err(from_cli)?,
        None => return Err(JobFileError::invalid("missing format tag")),
    };
    check_format(tag)?;
    check_keys(map, &["format", "source", "units", "tolerance", "heights", "tools", "operations"], "job").map_err(from_cli)?;
    let units = parse_units(require(map, "units").map_err(from_cli)?)?;
    let scale = units.mm_per_unit();
    let source = parse_source(require(map, "source").map_err(from_cli)?)?;
    let tolerance = parse_tolerance(optional(map, "tolerance"), scale)?;
    let (clearance, retract) = parse_heights(optional(map, "heights"), scale)?;
    let tools = parse_tools(require(map, "tools").map_err(from_cli)?, scale)?;
    let operations = parse_operations(require(map, "operations").map_err(from_cli)?, scale, &tools)?;
    Ok(Body { source, units, tolerance, tools, operations, clearance, retract })
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
    check_keys(map, &["id", "diameter", "kerf"], "tool").map_err(from_cli)?;
    let mut scaled = map.clone();
    scale_key(&mut scaled, "diameter", scale)?;
    scale_key(&mut scaled, "kerf", scale)?;
    let parsed = parse_tool(&Value::Object(scaled)).map_err(from_cli)?;
    parsed.path_tool().compensation_distance().map_err(|err| JobFileError::Invalid { message: err.to_string() })?;
    Ok(JobTool { id: parsed.ir().id(), path_tool: parsed.path_tool(), ir: parsed.ir().clone() })
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
            "step_down",
            "stepover",
            "stock_to_leave",
            "finish",
            "entry",
            "feeds",
            "cut_height",
            "pierce_height",
            "timing",
        ],
        "operation",
    )
    .map_err(from_cli)?;
    let name_value = require(map, "name").map_err(from_cli)?;
    let name = required_name(as_str(name_value, "operation name").map_err(from_cli)?, "operation name").map_err(from_cli)?;
    let kind = parse_kind(map)?;
    let layers = parse_layers(require(map, "layers").map_err(from_cli)?)?;
    let tool_id = as_u32(require(map, "tool_id").map_err(from_cli)?, "tool_id").map_err(from_cli)?;
    let tool = tools.iter().find(|tool| tool.id() == tool_id).ok_or_else(|| JobFileError::invalid(format!("unknown tool {tool_id}")))?;
    let depth = scale_f64(require(map, "depth").map_err(from_cli)?, scale, "depth")?;
    let step_down = scale_f64(require(map, "step_down").map_err(from_cli)?, scale, "step_down")?;
    check_depth(depth, step_down)?;
    let stepover = parse_stepover(optional(map, "stepover"), kind, tool, scale)?;
    let stock_to_leave = parse_stock_to_leave(optional(map, "stock_to_leave"), kind, depth, scale)?;
    let finish = parse_finish(optional(map, "finish"), kind, depth, scale)?;
    let entry = parse_entry(optional(map, "entry"), scale)?;
    let feeds = parse_operation_feeds(require(map, "feeds").map_err(from_cli)?, scale)?;
    let params = parse_timing(map, scale)?;
    Ok(Operation { name, kind, layers, tool_id, depth, step_down, stepover, stock_to_leave, finish, entry, feeds, params })
}

fn parse_stepover(value: Option<&Value>, kind: OperationKind, tool: &JobTool, scale: f64) -> Result<Option<f64>, JobFileError> {
    let Some(value) = value else {
        return Ok(None);
    };
    if kind != OperationKind::Pocket {
        return Err(JobFileError::invalid("stepover is only valid on a pocket"));
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

fn parse_stock_to_leave(value: Option<&Value>, kind: OperationKind, depth: f64, scale: f64) -> Result<StockToLeave, JobFileError> {
    let Some(value) = value else {
        return Ok(StockToLeave::default());
    };
    if !matches!(kind, OperationKind::Profile { .. } | OperationKind::Pocket) {
        return Err(JobFileError::invalid("stock_to_leave is only valid on a profile or pocket"));
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
        other => Err(JobFileError::invalid(format!("unknown operation type `{other}`"))),
    }
}

fn reject_side(side: Option<&Value>) -> Result<(), JobFileError> {
    if side.is_some() { Err(JobFileError::invalid("side is only valid on a profile")) } else { Ok(()) }
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

fn at_index(kind: &str, index: usize, err: JobFileError) -> JobFileError {
    match err {
        JobFileError::Invalid { message } => {
            let detail = message.strip_prefix("invalid input: ").unwrap_or(message.as_str());
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
    use campanion_path::Tool as PathTool;
    use std::fs;
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
}
