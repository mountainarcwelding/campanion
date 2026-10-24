//! `lathe`: load a `campanion-lathe/1` file and write turning toolpaths.
//!
//! The source path is relative to the job file. `.dxf` and `.svg` are half-profiles in the
//! drawing plane (drawing X is Z, drawing Y is the radius). `.stl` is an outside-diameter
//! envelope. `.step` and `.stp` are sectioned through the spindle axis by `solidcraft-cli`.
//! Units are millimetres. Feeds are millimetres per minute. Constant surface speed is
//! metres per minute and is capped at the spindle `max_rpm`.
//!
//! Z = 0 is the finished front face. `stock.front_allowance` is how far the blank sticks out in
//! +Z; facing cuts that down to 0. The stock record itself still occupies Z in `[-length, 0]`.
//! The optional `sub` block is a second spindle. Its profile uses that same frame (back face at
//! Z 0, part toward −Z). `back_allowance` is the facing depth on the sub. An operation's
//! `spindle` is `main` or `sub`. The stored section stays the main profile.
//! A turning operation is one toolpath: a tool change when the tool id changes, spindle on, the
//! moves, then spindle off. Speed is stored on the turning binding, which names the operation's
//! spindle. An indexed live drill or slot
//! changes the tool the same way, engages C, starts the live tool, clamps the brake around each
//! feed, then stops the live tool and disengages C. `live_flat` engages C, rapids to the flat,
//! clamps, then starts the live tool, and mills in Y. That binding stores no spindle speed. The IR
//! stores the main finished section on the turning setup, sampled from the main profile at the
//! job chord tolerance.
//!
//! `campanion_ir::verify` runs before anything is written. Issues are printed and the process
//! exits 1. Face, rough and finish paths are checked against that spindle's finished profile for
//! insert and holder gouges. A sub operation uses the sub profile. A gouge is printed and the
//! process exits 1. Nothing is written in either case.
//! `--post` writes that dialect instead of the test-sink NGC and compares the read-back
//! at the job chord tolerance. A mismatch exits 1 and nothing is written. ZX arcs in the test
//! sink are `G18`. The test sink does not write synchronised thread moves, so a job that threads
//! needs `--post`.

use std::collections::BTreeSet;
use std::ffi::{OsStr, OsString};
use std::io::{self, Write};
use std::path::{Component, Path, PathBuf};
use std::process::ExitCode;

use campanion_geom::Tolerance;
use campanion_import::{ImportOptions, import_svg};
use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, Document, Event, Feeds, Hand, Head, HeadAxis, Insert, InsertShape, LiveDirection, MoveRole, Pose,
    ProcessKind, ProcessParams, Program, Spindle, SpindleSpeed, Tool, TurnBinding, TurnInsert, TurnSection, TurnStock, TurningSetup, Vec3,
};
use campanion_turn::{
    DrillParams, FaceParams, FinishParams, FlatPath, FlatStep, GrooveParams, HolderGeom, Infeed, InsertGeom, LiveDrillParams, LiveFace,
    LiveFlatParams, LivePath, LiveSlotParams, LiveStation, LiveStep, MAX_LIVE_POSITIONS, MeshAxis, PartParams, RoughParams, Side, ThreadParams,
    TipPoint, TurnGouge, TurnPath, TurnProfile, TurnTool, centre_drill, check_turn_gouges, face, finish, groove, live_drill, live_flat, live_slot,
    part_off, rough, thread, to_cl_path,
};
use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_bool, as_f64, as_str, as_u32, check_keys, object, optional, read_text, require, required_name};
use crate::jobfile::DEFAULT_JOB_CHORD_MM;

const FORMAT_TAG: &str = "campanion-lathe/1";
const MAX_TOOLS: usize = 256;
const MAX_OPERATIONS: usize = 1_024;
const MAX_SOURCE_PATH: usize = 4_096;
/// [`SpindleSpeed::css`] rejects a cap above this, and the job uses the spindle cap as that limit.
/// A live-tool rpm uses the same ceiling because [`Event::LiveToolOn`] rejects a faster speed.
const MAX_SPINDLE_RPM: f64 = 100_000.0;
/// [`Tool::try_new`] rejects a cutter diameter above this.
const MAX_TOOL_DIAMETER_MM: f64 = 1.0e6;
/// Keys a live tool accepts. An insert field on the same object uses this sentence.
const LIVE_TOOL_FIELDS: &str = "id, station, live, kind, diameter, point_angle_deg, max_rpm";
const HEAD: &str = "turret1";
const SPINDLE: &str = "main";
const SUB_SPINDLE: &str = "sub";
/// [`TurningSetup::single`] builds a 12-station turret.
const STATIONS: u32 = 12;
/// Radial depth of a metric 60° thread, as a fraction of the pitch.
const METRIC_THREAD_DEPTH_FACTOR: f64 = 0.6134;
/// Compound infeed for a 60° metric form.
const DEFAULT_FLANK_DEG: f64 = 29.5;
/// Default first-pass depth is `min(depth, 0.35)`.
const DEFAULT_FIRST_DEPTH_CAP: f64 = 0.35;
/// Thread cycles emit rapids and synchronised moves. The path still needs a positive cut feed.
const THREAD_RECORD_FEED: f64 = 100.0;
/// Flank length, millimetres, used when a face, rough or finish path is checked for gouges.
const TURN_EDGE_MM: f64 = 4.0;

struct LatheArgs {
    job: PathBuf,
    output: PathBuf,
    ir: Option<PathBuf>,
    post: Option<PathBuf>,
    up_axis: crate::step::UpAxis,
}

struct LatheJob {
    source: Source,
    stock: Stock,
    front_allowance: f64,
    max_rpm: f64,
    c_axis: bool,
    /// Second spindle. Its profile is drawn in the sub frame: back face at Z 0, part toward −Z.
    sub: Option<SubBlock>,
    head: LatheHead,
    tools: Vec<LatheTool>,
    operations: Vec<Operation>,
    linear: f64,
    chord: f64,
    clearance: f64,
}

/// Optional sub-spindle. Stock diameter and length come from the main stock.
struct SubBlock {
    max_rpm: f64,
    source: Source,
    back_allowance: f64,
}

/// Which work spindle an operation cuts on. The default in the job file is main.
#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkSpindle {
    Main,
    Sub,
}

impl WorkSpindle {
    fn name(self) -> &'static str {
        match self {
            Self::Main => "main",
            Self::Sub => "sub",
        }
    }
}

/// Optional Y travel on turret `turret1`. Limits are required when `y_axis` is true.
struct LatheHead {
    y_axis: bool,
    y_min: Option<f64>,
    y_max: Option<f64>,
}

struct Source {
    path: String,
    kind: SourceKind,
}

enum SourceKind {
    Dxf { layer: Option<String> },
    Svg { layer: Option<String> },
    Stl { axis: MeshAxis, step: f64 },
    Step { body: Option<String> },
}

enum Stock {
    Bar { diameter: f64, length: f64 },
    Tube { outer: f64, inner: f64, length: f64 },
}

struct LatheTool {
    id: u32,
    station: u16,
    body: ToolBody,
}

/// An insert, or a driven tool that is not mounted as an insert.
enum ToolBody {
    Insert {
        insert: Insert,
        tip: TipPoint,
        nose_radius: f64,
        holder: Option<HolderGeom>,
    },
    /// `max_rpm` limits the live-tool rpm of an operation that uses this tool.
    Live {
        orientation: LiveOrientation,
        kind: LiveKind,
        diameter: f64,
        max_rpm: Option<f64>,
    },
}

enum LiveOrientation {
    Axial,
    Radial,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LiveKind {
    Drill,
    Endmill,
}

impl ToolBody {
    fn is_live(&self) -> bool {
        matches!(self, Self::Live { .. })
    }
}

enum Operation {
    Face {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        step_z: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Rough {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        depth_of_cut: f64,
        leave_x: f64,
        leave_z: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Finish {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        leave_x: f64,
        leave_z: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Thread {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        major_diameter: f64,
        pitch: f64,
        z_start: f64,
        z_end: f64,
        depth: f64,
        first_depth: f64,
        min_depth: f64,
        spring_passes: u8,
        infeed: Infeed,
        clearance: f64,
        speed: SpindleSpeed,
    },
    Groove {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        z_left: f64,
        z_right: f64,
        bottom_diameter: f64,
        surface_diameter: Option<f64>,
        insert_width: Option<f64>,
        peck: Option<f64>,
        stepover_frac: f64,
        clearance: f64,
        dwell_s: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Part {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        z: f64,
        to_diameter: f64,
        chamfer: Option<f64>,
        peck: Option<f64>,
        clearance: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Drill {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        z_start: f64,
        depth: f64,
        diameter: f64,
        peck: Option<f64>,
        retract: f64,
        point_angle_deg: f64,
        through: bool,
        feed: f64,
        speed: SpindleSpeed,
    },
    LiveDrill {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        params: LiveDrillParams,
    },
    LiveSlot {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        params: LiveSlotParams,
    },
    LiveFlat {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        c_deg: f64,
        diameter: f64,
        across: f64,
        z_from: f64,
        z_to: f64,
        width: Option<f64>,
        step_down: Option<f64>,
        stepover: f64,
    },
}

impl Operation {
    fn name(&self) -> &str {
        match self {
            Self::Face { name, .. }
            | Self::Rough { name, .. }
            | Self::Finish { name, .. }
            | Self::Thread { name, .. }
            | Self::Groove { name, .. }
            | Self::Part { name, .. }
            | Self::Drill { name, .. }
            | Self::LiveDrill { name, .. }
            | Self::LiveSlot { name, .. }
            | Self::LiveFlat { name, .. } => name,
        }
    }

    fn spindle(&self) -> WorkSpindle {
        match self {
            Self::Face { spindle, .. }
            | Self::Rough { spindle, .. }
            | Self::Finish { spindle, .. }
            | Self::Thread { spindle, .. }
            | Self::Groove { spindle, .. }
            | Self::Part { spindle, .. }
            | Self::Drill { spindle, .. }
            | Self::LiveDrill { spindle, .. }
            | Self::LiveSlot { spindle, .. }
            | Self::LiveFlat { spindle, .. } => *spindle,
        }
    }

    fn tool_id(&self) -> u32 {
        match self {
            Self::Face { tool_id, .. }
            | Self::Rough { tool_id, .. }
            | Self::Finish { tool_id, .. }
            | Self::Thread { tool_id, .. }
            | Self::Groove { tool_id, .. }
            | Self::Part { tool_id, .. }
            | Self::Drill { tool_id, .. }
            | Self::LiveDrill { tool_id, .. }
            | Self::LiveSlot { tool_id, .. }
            | Self::LiveFlat { tool_id, .. } => *tool_id,
        }
    }

    fn feed(&self) -> f64 {
        match self {
            Self::Face { feed, .. }
            | Self::Rough { feed, .. }
            | Self::Finish { feed, .. }
            | Self::Groove { feed, .. }
            | Self::Part { feed, .. }
            | Self::Drill { feed, .. }
            | Self::LiveDrill { feed, .. }
            | Self::LiveSlot { feed, .. }
            | Self::LiveFlat { feed, .. } => *feed,
            Self::Thread { .. } => THREAD_RECORD_FEED,
        }
    }

    /// Spindle speed for a turning pass. A live cycle returns `None`.
    fn turning_speed(&self) -> Option<SpindleSpeed> {
        match self {
            Self::Face { speed, .. }
            | Self::Rough { speed, .. }
            | Self::Finish { speed, .. }
            | Self::Thread { speed, .. }
            | Self::Groove { speed, .. }
            | Self::Part { speed, .. }
            | Self::Drill { speed, .. } => Some(*speed),
            Self::LiveDrill { .. } | Self::LiveSlot { .. } | Self::LiveFlat { .. } => None,
        }
    }

    fn is_live(&self) -> bool {
        matches!(self, Self::LiveDrill { .. } | Self::LiveSlot { .. })
    }

    fn is_flat(&self) -> bool {
        matches!(self, Self::LiveFlat { .. })
    }

    fn live_rpm(&self) -> Option<f64> {
        match self {
            Self::LiveDrill { rpm, .. } | Self::LiveSlot { rpm, .. } | Self::LiveFlat { rpm, .. } => Some(*rpm),
            _ => None,
        }
    }

    fn live_face(&self) -> Option<LiveFace> {
        match self {
            Self::LiveDrill { params, .. } => Some(params.face),
            Self::LiveSlot { params, .. } => Some(params.face),
            Self::LiveFlat { .. } => Some(LiveFace::Od),
            _ => None,
        }
    }
}

pub fn lathe(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_lathe(args)?;
    let post = match &args.post {
        Some(spec) => Some(crate::post::load_spec(spec)?),
        None => None,
    };
    let text = read_text(&args.job)?;
    let job = parse_job(&text)?;
    let profile = load_profile(&args.job, &job, args.up_axis)?;
    let sub_profile = match &job.sub {
        Some(sub) => Some(load_source(&args.job, &sub.source, job.linear, args.up_axis)?),
        None => None,
    };
    let (doc, gouges) = build_document(&args.job, &job, &profile, sub_profile.as_ref())?;
    if !gouges.is_empty() {
        let mut err_out = io::stderr().lock();
        for line in &gouges {
            writeln!(err_out, "{line}")?;
        }
        return Ok(ExitCode::from(1));
    }
    emit(&args, &job, &doc, post.as_ref())
}

fn parse_lathe(args: &[OsString]) -> Result<LatheArgs, CliError> {
    let mut job: Option<&OsStr> = None;
    let mut output: Option<&OsStr> = None;
    let mut ir: Option<&OsStr> = None;
    let mut post: Option<&OsStr> = None;
    let mut up_axis: Option<crate::step::UpAxis> = None;
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "--up-axis" {
            let Some(value) = args.get(index) else {
                return Err(CliError::usage("--up-axis needs a value"));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage("--up-axis needs a value"));
            }
            if up_axis.is_some() {
                return Err(CliError::usage("--up-axis is already set"));
            }
            let Some(text) = value.to_str() else {
                return Err(CliError::usage("--up-axis must be z, y or auto"));
            };
            up_axis = Some(crate::step::UpAxis::parse(text)?);
        } else if arg.as_os_str() == "-o" || arg.as_os_str() == "--ir" || arg.as_os_str() == "--post" {
            let flag = if arg.as_os_str() == "-o" {
                "-o"
            } else if arg.as_os_str() == "--ir" {
                "--ir"
            } else {
                "--post"
            };
            let Some(value) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            }
            let slot = if flag == "-o" {
                &mut output
            } else if flag == "--ir" {
                &mut ir
            } else {
                &mut post
            };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(value.as_os_str());
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(CliError::usage("unknown argument"));
        } else if job.is_some() || arg.is_empty() {
            return Err(CliError::usage("lathe takes one job file"));
        } else {
            job = Some(arg.as_os_str());
        }
    }
    let Some(job) = job else {
        return Err(CliError::usage("lathe needs a job file"));
    };
    let Some(output) = output else {
        return Err(CliError::usage("lathe needs -o OUT"));
    };
    if ir.is_some_and(|path| path == output) {
        return Err(CliError::usage("-o and --ir must be different files"));
    }
    Ok(LatheArgs {
        job: PathBuf::from(job),
        output: PathBuf::from(output),
        ir: ir.map(PathBuf::from),
        post: post.map(PathBuf::from),
        up_axis: up_axis.unwrap_or(crate::step::UpAxis::Z),
    })
}

fn parse_job(text: &str) -> Result<LatheJob, CliError> {
    let value: Value = serde_json::from_str(text).map_err(|err| CliError::run(format!("json: {err}")))?;
    let map = object(&value, "job")?;
    let tag = as_str(require(map, "format")?, "format tag")?;
    check_format(tag)?;
    check_keys(map, &["format", "source", "units", "stock", "spindle", "sub", "head", "tools", "operations", "tolerance", "clearance"], "job")?;
    let units = as_str(require(map, "units")?, "units")?;
    if units != "mm" {
        return Err(CliError::invalid(format!("units must be mm, not `{units}`")));
    }
    let source = parse_source(require(map, "source")?)?;
    let (stock, front_allowance) = parse_stock(require(map, "stock")?)?;
    let (max_rpm, c_axis) = parse_spindle(require(map, "spindle")?)?;
    let sub = match optional(map, "sub") {
        Some(value) => Some(parse_sub(value)?),
        None => None,
    };
    let head = parse_head(optional(map, "head"))?;
    let tools = parse_tools(require(map, "tools")?)?;
    let (linear, chord) = parse_tolerance(optional(map, "tolerance"))?;
    let clearance = non_negative(require(map, "clearance")?, "clearance")?;
    let sub_rpm = sub.as_ref().map(|block| block.max_rpm);
    let operations = parse_operations(require(map, "operations")?, max_rpm, sub_rpm, clearance)?;
    for operation in &operations {
        let Some(tool) = tools.iter().find(|tool| tool.id == operation.tool_id()) else {
            return Err(CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())));
        };
        check_tool_use(operation, tool, c_axis)?;
        check_flat(operation, tool, &head, clearance)?;
    }
    Ok(LatheJob { source, stock, front_allowance, max_rpm, c_axis, sub, head, tools, operations, linear, chord, clearance })
}

fn parse_sub(value: &Value) -> Result<SubBlock, CliError> {
    let map = object(value, "sub")?;
    check_keys(map, &["max_rpm", "source", "back_allowance"], "sub")?;
    let max_rpm = positive(require(map, "max_rpm")?, "max_rpm")?;
    if max_rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("sub max_rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    let source = parse_source(require(map, "source")?)?;
    let back_allowance = match optional(map, "back_allowance") {
        Some(value) => non_negative(value, "back_allowance")?,
        None => 0.0,
    };
    Ok(SubBlock { max_rpm, source, back_allowance })
}

fn parse_head(value: Option<&Value>) -> Result<LatheHead, CliError> {
    let Some(value) = value else {
        return Ok(LatheHead { y_axis: false, y_min: None, y_max: None });
    };
    let map = object(value, "head")?;
    check_keys(map, &["y_axis", "y_min", "y_max"], "head")?;
    let y_axis = match optional(map, "y_axis") {
        Some(value) => as_bool(value, "y_axis")?,
        None => false,
    };
    if !y_axis {
        if map.contains_key("y_min") || map.contains_key("y_max") {
            return Err(CliError::invalid("y_min and y_max need head.y_axis true"));
        }
        return Ok(LatheHead { y_axis: false, y_min: None, y_max: None });
    }
    let y_min = as_f64(require(map, "y_min")?, "y_min")?;
    let y_max = as_f64(require(map, "y_max")?, "y_max")?;
    if !(y_min < 0.0 && y_max > 0.0) {
        return Err(CliError::invalid("y_min must be < 0 and y_max must be > 0"));
    }
    Ok(LatheHead { y_axis: true, y_min: Some(y_min), y_max: Some(y_max) })
}

fn check_format(tag: &str) -> Result<(), CliError> {
    if tag == FORMAT_TAG {
        return Ok(());
    }
    let Some(rest) = tag.strip_prefix("campanion-lathe/") else {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    };
    if rest.is_empty() || !rest.bytes().all(|byte| byte.is_ascii_digit()) || (rest.len() > 1 && rest.starts_with('0')) {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    }
    let Ok(major) = rest.parse::<u32>() else {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    };
    if major == 1 { Ok(()) } else { Err(CliError::invalid(format!("unsupported lathe job major version {major}; this build reads major 1"))) }
}

fn parse_source(value: &Value) -> Result<Source, CliError> {
    let map = object(value, "source")?;
    let path = source_path_text(require(map, "path")?)?;
    let kind = source_kind(path)?;
    let allowed: &[&str] = match kind {
        SourceKind::Dxf { .. } | SourceKind::Svg { .. } => &["path", "layer"],
        SourceKind::Stl { .. } => &["path", "axis", "step"],
        SourceKind::Step { .. } => &["path", "body"],
    };
    check_keys(map, allowed, "source")?;
    let source_kind = match kind {
        SourceKind::Dxf { .. } => SourceKind::Dxf { layer: optional_layer(map)? },
        SourceKind::Svg { .. } => SourceKind::Svg { layer: optional_layer(map)? },
        SourceKind::Stl { .. } => {
            let axis = parse_axis(as_str(require(map, "axis")?, "axis")?)?;
            let step = positive(require(map, "step")?, "step")?;
            SourceKind::Stl { axis, step }
        }
        SourceKind::Step { .. } => SourceKind::Step { body: optional_body(map)? },
    };
    Ok(Source { path: path.to_string(), kind: source_kind })
}

fn source_kind(path: &str) -> Result<SourceKind, CliError> {
    let ext = Path::new(path).extension().and_then(|ext| ext.to_str()).unwrap_or("");
    match ext.to_ascii_lowercase().as_str() {
        "dxf" => Ok(SourceKind::Dxf { layer: None }),
        "svg" => Ok(SourceKind::Svg { layer: None }),
        "stl" => Ok(SourceKind::Stl { axis: MeshAxis::Z, step: 1.0 }),
        "step" | "stp" => Ok(SourceKind::Step { body: None }),
        _ => Err(CliError::invalid("source path must end in .dxf, .svg, .stl, .step or .stp")),
    }
}

fn optional_body(map: &Map<String, Value>) -> Result<Option<String>, CliError> {
    match optional(map, "body") {
        Some(value) => Ok(Some(required_name(as_str(value, "body")?, "body")?)),
        None => Ok(None),
    }
}

fn optional_layer(map: &Map<String, Value>) -> Result<Option<String>, CliError> {
    match optional(map, "layer") {
        Some(value) => Ok(Some(required_name(as_str(value, "layer")?, "layer")?)),
        None => Ok(None),
    }
}

fn parse_axis(text: &str) -> Result<MeshAxis, CliError> {
    match text {
        "x" => Ok(MeshAxis::X),
        "y" => Ok(MeshAxis::Y),
        "z" => Ok(MeshAxis::Z),
        other => Err(CliError::invalid(format!("unknown axis `{other}`"))),
    }
}

fn source_path_text(value: &Value) -> Result<&str, CliError> {
    let Some(text) = value.as_str() else {
        return Err(CliError::invalid("source path must be a string"));
    };
    if text.is_empty() || text.len() > MAX_SOURCE_PATH {
        return Err(CliError::invalid(format!("source path must be 1 to {MAX_SOURCE_PATH} bytes")));
    }
    if text.bytes().any(|byte| byte < 0x20 || byte == 0x7f) {
        return Err(CliError::invalid("source path has a control character"));
    }
    let path = Path::new(text);
    if path.is_absolute() {
        return Err(CliError::invalid("source path must be relative to the job file"));
    }
    let mut named = false;
    for component in path.components() {
        match component {
            Component::Normal(_) => named = true,
            Component::CurDir | Component::ParentDir => {}
            Component::RootDir | Component::Prefix(_) => {
                return Err(CliError::invalid("source path must be relative to the job file"));
            }
        }
    }
    if !named {
        return Err(CliError::invalid("source path must name a file"));
    }
    Ok(text)
}

fn parse_stock(value: &Value) -> Result<(Stock, f64), CliError> {
    let map = object(value, "stock")?;
    let kind = as_str(require(map, "kind")?, "stock kind")?;
    let allowed: &[&str] = match kind {
        "bar" => &["kind", "diameter", "length", "front_allowance"],
        "tube" => &["kind", "outer_diameter", "inner_diameter", "length", "front_allowance"],
        other => return Err(CliError::invalid(format!("unknown stock kind `{other}`"))),
    };
    check_keys(map, allowed, "stock")?;
    let length = positive(require(map, "length")?, "length")?;
    let front_allowance = non_negative(require(map, "front_allowance")?, "front_allowance")?;
    let stock = match kind {
        "bar" => Stock::Bar { diameter: positive(require(map, "diameter")?, "diameter")?, length },
        "tube" => Stock::Tube {
            outer: positive(require(map, "outer_diameter")?, "outer_diameter")?,
            inner: positive(require(map, "inner_diameter")?, "inner_diameter")?,
            length,
        },
        _ => return Err(CliError::invalid(format!("unknown stock kind `{kind}`"))),
    };
    Ok((stock, front_allowance))
}

fn parse_spindle(value: &Value) -> Result<(f64, bool), CliError> {
    let map = object(value, "spindle")?;
    check_keys(map, &["max_rpm", "c_axis"], "spindle")?;
    let max_rpm = positive(require(map, "max_rpm")?, "max_rpm")?;
    if max_rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("spindle max_rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    let c_axis = match optional(map, "c_axis") {
        Some(value) => as_bool(value, "c_axis")?,
        None => false,
    };
    Ok((max_rpm, c_axis))
}

fn parse_tools(value: &Value) -> Result<Vec<LatheTool>, CliError> {
    let list = array(value, "tools", MAX_TOOLS)?;
    let mut ids = BTreeSet::new();
    let mut stations = BTreeSet::new();
    let mut tools = Vec::with_capacity(list.len());
    for item in list {
        let tool = parse_tool(item)?;
        if !ids.insert(tool.id) {
            return Err(CliError::invalid(format!("duplicate tool id {}", tool.id)));
        }
        if !stations.insert(tool.station) {
            return Err(CliError::invalid(format!("duplicate tool station {}", tool.station)));
        }
        tools.push(tool);
    }
    Ok(tools)
}

fn parse_tool(value: &Value) -> Result<LatheTool, CliError> {
    let map = object(value, "tool")?;
    if map.contains_key("live") {
        return parse_live_tool(map);
    }
    check_keys(map, &["id", "station", "shape", "nose_radius", "orientation_deg", "hand", "tip", "width", "holder"], "tool")?;
    let id = as_u32(require(map, "id")?, "tool id")?;
    let station = parse_station(map)?;
    let shape = parse_shape(as_str(require(map, "shape")?, "shape")?)?;
    let nose_radius = positive(require(map, "nose_radius")?, "nose_radius")?;
    let orientation_deg = as_f64(require(map, "orientation_deg")?, "orientation_deg")?;
    let hand = parse_hand(as_str(require(map, "hand")?, "hand")?)?;
    let tip_name = as_str(require(map, "tip")?, "tip")?;
    let width = match optional(map, "width") {
        Some(value) => Some(positive(value, "width")?),
        None => None,
    };
    let insert = Insert::try_new(shape, nose_radius, width, hand, orientation_deg).map_err(|err| CliError::run(format!("tool {id}: {err}")))?;
    let tip = programmed_tip(&insert, tip_name)?;
    let holder = match optional(map, "holder") {
        Some(value) => Some(parse_holder(value)?),
        None => None,
    };
    Ok(LatheTool { id, station, body: ToolBody::Insert { insert, tip, nose_radius, holder } })
}

fn parse_station(map: &Map<String, Value>) -> Result<u16, CliError> {
    let station_u = as_u32(require(map, "station")?, "station")?;
    if !(1..=STATIONS).contains(&station_u) {
        return Err(CliError::invalid(format!("station must be from 1 to {STATIONS}")));
    }
    u16::try_from(station_u).map_err(|_| CliError::invalid(format!("station must be from 1 to {STATIONS}")))
}

/// A tool that carries `live` is a driven drill or endmill. Insert fields are rejected, and the
/// point angle is checked for a drill then dropped: the hole depth is the tip depth.
fn parse_live_tool(map: &Map<String, Value>) -> Result<LatheTool, CliError> {
    let id = as_u32(require(map, "id")?, "tool id")?;
    const INSERT_KEYS: &[&str] = &["shape", "nose_radius", "orientation_deg", "hand", "tip", "width", "holder"];
    if INSERT_KEYS.iter().any(|key| map.contains_key(*key)) {
        return Err(CliError::invalid(format!("tool {id}: live tools take {LIVE_TOOL_FIELDS}")));
    }
    check_keys(map, &["id", "station", "live", "kind", "diameter", "point_angle_deg", "max_rpm"], "tool")?;
    let station = parse_station(map)?;
    let orientation = match as_str(require(map, "live")?, "live")? {
        "axial" => LiveOrientation::Axial,
        "radial" => LiveOrientation::Radial,
        other => return Err(CliError::invalid(format!("unknown live `{other}`"))),
    };
    let kind = match as_str(require(map, "kind")?, "kind")? {
        "drill" => LiveKind::Drill,
        "endmill" => LiveKind::Endmill,
        other => return Err(CliError::invalid(format!("unknown kind `{other}`"))),
    };
    match (kind, optional(map, "point_angle_deg")) {
        (LiveKind::Endmill, Some(_)) => {
            return Err(CliError::invalid(format!("tool {id}: point_angle_deg is only for a drill")));
        }
        (LiveKind::Drill, Some(value)) => {
            let angle = as_f64(value, "point_angle_deg")?;
            if !(angle > 0.0 && angle < 180.0) {
                return Err(CliError::invalid("point_angle_deg must be between 0 and 180"));
            }
        }
        (LiveKind::Drill, None) | (LiveKind::Endmill, None) => {}
    }
    let diameter = positive(require(map, "diameter")?, "diameter")?;
    if diameter > MAX_TOOL_DIAMETER_MM {
        return Err(CliError::invalid(format!("diameter is larger than {MAX_TOOL_DIAMETER_MM} mm")));
    }
    let max_rpm = match optional(map, "max_rpm") {
        Some(value) => {
            let rpm = positive(value, "max_rpm")?;
            if rpm > MAX_SPINDLE_RPM {
                return Err(CliError::invalid(format!("max_rpm must be at most {MAX_SPINDLE_RPM}")));
            }
            Some(rpm)
        }
        None => None,
    };
    Ok(LatheTool { id, station, body: ToolBody::Live { orientation, kind, diameter, max_rpm } })
}

fn parse_holder(value: &Value) -> Result<HolderGeom, CliError> {
    let map = object(value, "holder")?;
    check_keys(map, &["width", "length"], "holder")?;
    let width = positive(require(map, "width")?, "width")?;
    let length = positive(require(map, "length")?, "length")?;
    Ok(HolderGeom { width, length })
}

fn parse_shape(text: &str) -> Result<InsertShape, CliError> {
    match text {
        "c80" => Ok(InsertShape::C80),
        "d55" => Ok(InsertShape::D55),
        "v35" => Ok(InsertShape::V35),
        "w80" => Ok(InsertShape::W80),
        "t60" => Ok(InsertShape::T60),
        "s90" => Ok(InsertShape::S90),
        "r" => Ok(InsertShape::R),
        "groove" => Ok(InsertShape::Groove),
        "thread" => Ok(InsertShape::Thread),
        other => Err(CliError::invalid(format!("unknown insert shape `{other}`"))),
    }
}

fn parse_hand(text: &str) -> Result<Hand, CliError> {
    match text {
        "left" => Ok(Hand::Left),
        "right" => Ok(Hand::Right),
        "neutral" => Ok(Hand::Neutral),
        other => Err(CliError::invalid(format!("unknown hand `{other}`"))),
    }
}

/// Programmed point for one insert.
///
/// [`Insert::tip_offset`] is `(dx, dz)` in `(X, Z)`. [`TipPoint::ImaginaryTip`] stores that
/// same offset as `{ dz, dx }`: the imaginary corner minus the nose centre. At 135° the holder
/// bisector has positive X and negative Z, so the offset is `dx = -nose` and `dz = +nose`.
fn programmed_tip(insert: &Insert, tip: &str) -> Result<TipPoint, CliError> {
    match tip {
        "centre" => Ok(TipPoint::Centre),
        "imaginary" => {
            let (dx, dz) = insert.tip_offset();
            Ok(TipPoint::ImaginaryTip { dz, dx })
        }
        other => Err(CliError::invalid(format!("unknown tip `{other}`"))),
    }
}

fn parse_operations(value: &Value, main_rpm: f64, sub_rpm: Option<f64>, clearance: f64) -> Result<Vec<Operation>, CliError> {
    let list = array(value, "operations", MAX_OPERATIONS)?;
    let mut names = BTreeSet::new();
    let mut operations = Vec::with_capacity(list.len());
    for item in list {
        operations.push(parse_operation(item, main_rpm, sub_rpm, clearance, &mut names)?);
    }
    Ok(operations)
}

fn parse_work_spindle(map: &Map<String, Value>, name: &str, has_sub: bool, live: bool) -> Result<WorkSpindle, CliError> {
    let text = match optional(map, "spindle") {
        Some(value) => as_str(value, "spindle")?,
        None => "main",
    };
    match text {
        "main" => Ok(WorkSpindle::Main),
        "sub" if live => Err(CliError::invalid(format!("operation {name}: live tooling on the sub-spindle is not supported"))),
        "sub" if !has_sub => Err(CliError::invalid(format!("operation {name}: spindle sub needs a \"sub\" block"))),
        "sub" => Ok(WorkSpindle::Sub),
        other => Err(CliError::invalid(format!("operation {name}: unknown spindle `{other}`"))),
    }
}

fn speed_cap(spindle: WorkSpindle, main_rpm: f64, sub_rpm: Option<f64>) -> Result<f64, CliError> {
    match spindle {
        WorkSpindle::Main => Ok(main_rpm),
        WorkSpindle::Sub => sub_rpm.ok_or_else(|| CliError::run("internal: sub spindle is missing")),
    }
}

fn parse_operation(value: &Value, main_rpm: f64, sub_rpm: Option<f64>, clearance: f64, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    let map = object(value, "operation")?;
    let op_type = as_str(require(map, "type")?, "operation type")?;
    let has_sub = sub_rpm.is_some();
    if op_type == "live_mill" {
        let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
        if !names.insert(name.clone()) {
            return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
        }
        parse_work_spindle(map, &name, has_sub, true)?;
        return Err(CliError::invalid(format!("unknown operation type `{op_type}`")));
    }
    if op_type == "live_drill" || op_type == "live_slot" {
        return parse_live_operation(map, op_type, clearance, names, has_sub);
    }
    if op_type == "live_flat" {
        return parse_live_flat(map, names, has_sub);
    }
    let allowed: &[&str] = match op_type {
        "face" => &["name", "type", "tool_id", "spindle", "step_z", "feed", "speed"],
        "rough" => &["name", "type", "tool_id", "spindle", "side", "depth_of_cut", "leave_x", "leave_z", "feed", "speed"],
        "finish" => &["name", "type", "tool_id", "spindle", "side", "feed", "speed", "leave_x", "leave_z"],
        "thread" => &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "side",
            "major_diameter",
            "pitch",
            "z_start",
            "z_end",
            "depth",
            "first_depth",
            "min_depth",
            "spring_passes",
            "infeed",
            "clearance",
            "rpm",
        ],
        "groove" => &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "side",
            "z_left",
            "z_right",
            "bottom_diameter",
            "surface_diameter",
            "insert_width",
            "peck",
            "stepover_frac",
            "clearance",
            "dwell_s",
            "feed",
            "speed",
        ],
        "part" => &["name", "type", "tool_id", "spindle", "z", "to_diameter", "chamfer", "peck", "clearance", "feed", "speed"],
        "drill" => {
            &["name", "type", "tool_id", "spindle", "z_start", "depth", "diameter", "peck", "retract", "point_angle_deg", "through", "feed", "rpm"]
        }
        other => return Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, false)?;
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
    let cap = speed_cap(spindle, main_rpm, sub_rpm)?;
    match op_type {
        "face" | "rough" | "finish" | "groove" | "part" => {
            let feed = positive(require(map, "feed")?, "feed")?;
            let speed = parse_speed(require(map, "speed")?, cap)?;
            match op_type {
                "face" => Ok(Operation::Face { name, spindle, tool_id, step_z: positive(require(map, "step_z")?, "step_z")?, feed, speed }),
                "rough" => Ok(Operation::Rough {
                    name,
                    spindle,
                    tool_id,
                    side: parse_side(as_str(require(map, "side")?, "side")?)?,
                    depth_of_cut: positive(require(map, "depth_of_cut")?, "depth_of_cut")?,
                    leave_x: non_negative(require(map, "leave_x")?, "leave_x")?,
                    leave_z: non_negative(require(map, "leave_z")?, "leave_z")?,
                    feed,
                    speed,
                }),
                "finish" => Ok(Operation::Finish {
                    name,
                    spindle,
                    tool_id,
                    side: parse_side(as_str(require(map, "side")?, "side")?)?,
                    leave_x: optional_non_negative(map, "leave_x")?.unwrap_or(0.0),
                    leave_z: optional_non_negative(map, "leave_z")?.unwrap_or(0.0),
                    feed,
                    speed,
                }),
                "groove" => parse_groove(map, name, spindle, tool_id, feed, speed),
                "part" => parse_part(map, name, spindle, tool_id, feed, speed),
                _ => Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
            }
        }
        "thread" => parse_thread(map, name, spindle, tool_id),
        "drill" => parse_drill(map, name, spindle, tool_id),
        other => Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    }
}

fn parse_thread(map: &Map<String, Value>, name: String, spindle: WorkSpindle, tool_id: u32) -> Result<Operation, CliError> {
    let pitch = positive(require(map, "pitch")?, "pitch")?;
    let depth = match optional(map, "depth") {
        Some(value) => positive(value, "depth")?,
        None => METRIC_THREAD_DEPTH_FACTOR * pitch,
    };
    let first_depth = match optional(map, "first_depth") {
        Some(value) => positive(value, "first_depth")?,
        None => depth.min(DEFAULT_FIRST_DEPTH_CAP),
    };
    let min_depth = match optional(map, "min_depth") {
        Some(value) => positive(value, "min_depth")?,
        None => 0.05,
    };
    Ok(Operation::Thread {
        name,
        spindle,
        tool_id,
        side: parse_side(as_str(require(map, "side")?, "side")?)?,
        major_diameter: positive(require(map, "major_diameter")?, "major_diameter")?,
        pitch,
        z_start: as_f64(require(map, "z_start")?, "z_start")?,
        z_end: as_f64(require(map, "z_end")?, "z_end")?,
        depth,
        first_depth,
        min_depth,
        spring_passes: optional_u8(map, "spring_passes")?.unwrap_or(1),
        infeed: match optional(map, "infeed") {
            Some(value) => parse_infeed(as_str(value, "infeed")?)?,
            None => Infeed::Flank { angle_deg: DEFAULT_FLANK_DEG },
        },
        clearance: match optional(map, "clearance") {
            Some(value) => non_negative(value, "clearance")?,
            None => 1.0,
        },
        speed: SpindleSpeed::rpm(positive(require(map, "rpm")?, "rpm")?)?,
    })
}

fn parse_groove(
    map: &Map<String, Value>,
    name: String,
    spindle: WorkSpindle,
    tool_id: u32,
    feed: f64,
    speed: SpindleSpeed,
) -> Result<Operation, CliError> {
    let stepover_frac = match optional(map, "stepover_frac") {
        Some(value) => as_f64(value, "stepover_frac")?,
        None => 0.8,
    };
    if !(stepover_frac > 0.0 && stepover_frac <= 1.0) {
        return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
    }
    Ok(Operation::Groove {
        name,
        spindle,
        tool_id,
        side: parse_side(as_str(require(map, "side")?, "side")?)?,
        z_left: as_f64(require(map, "z_left")?, "z_left")?,
        z_right: as_f64(require(map, "z_right")?, "z_right")?,
        bottom_diameter: positive(require(map, "bottom_diameter")?, "bottom_diameter")?,
        surface_diameter: optional_positive(map, "surface_diameter")?,
        insert_width: optional_positive(map, "insert_width")?,
        peck: optional_positive(map, "peck")?,
        stepover_frac,
        clearance: match optional(map, "clearance") {
            Some(value) => non_negative(value, "clearance")?,
            None => 1.0,
        },
        dwell_s: optional_non_negative(map, "dwell_s")?.unwrap_or(0.0),
        feed,
        speed,
    })
}

fn parse_part(
    map: &Map<String, Value>,
    name: String,
    spindle: WorkSpindle,
    tool_id: u32,
    feed: f64,
    speed: SpindleSpeed,
) -> Result<Operation, CliError> {
    Ok(Operation::Part {
        name,
        spindle,
        tool_id,
        z: as_f64(require(map, "z")?, "z")?,
        to_diameter: match optional(map, "to_diameter") {
            Some(value) => as_f64(value, "to_diameter")?,
            None => -0.5,
        },
        chamfer: optional_positive(map, "chamfer")?,
        peck: optional_positive(map, "peck")?,
        clearance: non_negative(require(map, "clearance")?, "clearance")?,
        feed,
        speed,
    })
}

fn parse_drill(map: &Map<String, Value>, name: String, spindle: WorkSpindle, tool_id: u32) -> Result<Operation, CliError> {
    let point_angle_deg = match optional(map, "point_angle_deg") {
        Some(value) => as_f64(value, "point_angle_deg")?,
        None => 118.0,
    };
    Ok(Operation::Drill {
        name,
        spindle,
        tool_id,
        z_start: match optional(map, "z_start") {
            Some(value) => as_f64(value, "z_start")?,
            None => 0.0,
        },
        depth: positive(require(map, "depth")?, "depth")?,
        diameter: positive(require(map, "diameter")?, "diameter")?,
        peck: optional_positive(map, "peck")?,
        retract: match optional(map, "retract") {
            Some(value) => non_negative(value, "retract")?,
            None => 1.0,
        },
        point_angle_deg,
        through: match optional(map, "through") {
            Some(value) => as_bool(value, "through")?,
            None => false,
        },
        feed: positive(require(map, "feed")?, "feed")?,
        speed: SpindleSpeed::rpm(positive(require(map, "rpm")?, "rpm")?)?,
    })
}

fn parse_live_operation(
    map: &Map<String, Value>,
    op_type: &str,
    clearance: f64,
    names: &mut BTreeSet<String>,
    has_sub: bool,
) -> Result<Operation, CliError> {
    let face = parse_live_face(as_str(require(map, "face")?, "face")?)?;
    let allowed: &[&str] = match (op_type, face) {
        ("live_drill", LiveFace::Front) => {
            &["name", "type", "tool_id", "spindle", "face", "positions", "depth", "peck", "retract", "rpm", "feed", "z_start"]
        }
        ("live_drill", LiveFace::Od) => {
            &["name", "type", "tool_id", "spindle", "face", "positions", "depth", "peck", "retract", "rpm", "feed", "diameter"]
        }
        ("live_slot", LiveFace::Front) => {
            &["name", "type", "tool_id", "spindle", "face", "c", "from", "to", "depth", "step_down", "rpm", "feed", "z_start"]
        }
        ("live_slot", LiveFace::Od) => {
            &["name", "type", "tool_id", "spindle", "face", "c", "from", "to", "depth", "step_down", "rpm", "feed", "diameter"]
        }
        _ => return Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, true)?;
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
    let depth = positive(require(map, "depth")?, "depth")?;
    let rpm = parse_live_rpm(require(map, "rpm")?)?;
    let feed = positive(require(map, "feed")?, "feed")?;
    let (z_start, diameter) = match face {
        LiveFace::Front => (as_f64(require(map, "z_start")?, "z_start")?, 0.0),
        LiveFace::Od => (0.0, positive(require(map, "diameter")?, "diameter")?),
    };
    match op_type {
        "live_drill" => {
            let retract = match optional(map, "retract") {
                Some(value) => non_negative(value, "retract")?,
                None => clearance,
            };
            let params = LiveDrillParams {
                face,
                positions: parse_live_positions(require(map, "positions")?, face)?,
                depth,
                peck: optional_positive(map, "peck")?,
                retract,
                z_start,
                diameter,
            };
            live_drill(&params).map_err(|err| geometry_err(&name, err))?;
            Ok(Operation::LiveDrill { name, spindle, tool_id, rpm, feed, params })
        }
        "live_slot" => {
            let step_down = match optional(map, "step_down") {
                Some(value) => positive(value, "step_down")?,
                None => depth,
            };
            let params = LiveSlotParams {
                face,
                c_deg: as_f64(require(map, "c")?, "c")?,
                from: as_f64(require(map, "from")?, "from")?,
                to: as_f64(require(map, "to")?, "to")?,
                depth,
                step_down,
                retract: clearance,
                z_start,
                diameter,
            };
            live_slot(&params).map_err(|err| geometry_err(&name, err))?;
            Ok(Operation::LiveSlot { name, spindle, tool_id, rpm, feed, params })
        }
        _ => Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
    }
}

fn parse_live_flat(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    check_keys(
        map,
        &["name", "type", "tool_id", "spindle", "c", "diameter", "across", "z_from", "z_to", "width", "step_down", "stepover", "rpm", "feed"],
        "operation",
    )?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, true)?;
    let stepover = match optional(map, "stepover") {
        Some(value) => as_f64(value, "stepover")?,
        None => 0.6,
    };
    if !(stepover > 0.0 && stepover <= 1.0) {
        return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
    }
    Ok(Operation::LiveFlat {
        name,
        spindle,
        tool_id: as_u32(require(map, "tool_id")?, "tool_id")?,
        rpm: parse_live_rpm(require(map, "rpm")?)?,
        feed: positive(require(map, "feed")?, "feed")?,
        c_deg: as_f64(require(map, "c")?, "c")?,
        diameter: positive(require(map, "diameter")?, "diameter")?,
        across: positive(require(map, "across")?, "across")?,
        z_from: as_f64(require(map, "z_from")?, "z_from")?,
        z_to: as_f64(require(map, "z_to")?, "z_to")?,
        width: optional_positive(map, "width")?,
        step_down: optional_positive(map, "step_down")?,
        stepover,
    })
}

fn parse_live_face(text: &str) -> Result<LiveFace, CliError> {
    match text {
        "front" => Ok(LiveFace::Front),
        "od" => Ok(LiveFace::Od),
        other => Err(CliError::invalid(format!("unknown face `{other}`"))),
    }
}

fn parse_live_positions(value: &Value, face: LiveFace) -> Result<Vec<LiveStation>, CliError> {
    let list = array(value, "positions", MAX_LIVE_POSITIONS)?;
    let mut positions = Vec::with_capacity(list.len());
    for item in list {
        let map = object(item, "position")?;
        let allowed: &[&str] = match face {
            LiveFace::Front => &["c", "r"],
            LiveFace::Od => &["c", "z"],
        };
        check_keys(map, allowed, "position")?;
        let c_deg = as_f64(require(map, "c")?, "c")?;
        let at = match face {
            LiveFace::Front => non_negative(require(map, "r")?, "r")?,
            LiveFace::Od => as_f64(require(map, "z")?, "z")?,
        };
        positions.push(LiveStation { c_deg, at });
    }
    Ok(positions)
}

fn parse_live_rpm(value: &Value) -> Result<f64, CliError> {
    let rpm = positive(value, "rpm")?;
    if rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    Ok(rpm)
}

/// `c_axis` is checked before the tool orientation, so a live cycle on a spindle with no C axis
/// reports that even when the tool also faces the wrong way.
fn check_tool_use(operation: &Operation, tool: &LatheTool, c_axis: bool) -> Result<(), CliError> {
    let Some(face) = operation.live_face() else {
        return match &tool.body {
            ToolBody::Live { .. } => Err(CliError::run(format!("operation {}: tool {} is a live tool", operation.name(), tool.id))),
            ToolBody::Insert { .. } => Ok(()),
        };
    };
    if !c_axis {
        return Err(CliError::invalid("live operations need spindle.c_axis true"));
    }
    let ToolBody::Live { orientation, kind, max_rpm, .. } = &tool.body else {
        let which = if face.axial() { "an axial" } else { "a radial" };
        return Err(CliError::invalid(format!("operation {}: tool {} is not {which} live tool", operation.name(), tool.id)));
    };
    let axial = matches!(orientation, LiveOrientation::Axial);
    if axial != face.axial() {
        let which = if face.axial() { "an axial" } else { "a radial" };
        return Err(CliError::invalid(format!("operation {}: tool {} is not {which} live tool", operation.name(), tool.id)));
    }
    if operation.is_flat() && *kind != LiveKind::Endmill {
        return Err(CliError::invalid(format!("operation {}: tool {} is not an endmill", operation.name(), tool.id)));
    }
    if let Some(cap) = max_rpm
        && let Some(rpm) = operation.live_rpm()
        && rpm > *cap
    {
        return Err(CliError::invalid(format!("operation {}: rpm exceeds tool {} max_rpm", operation.name(), tool.id)));
    }
    Ok(())
}

/// A flat needs a Y head. Every tool-centre Y on the path must sit in `[y_min, y_max]`.
fn check_flat(operation: &Operation, tool: &LatheTool, head: &LatheHead, clearance: f64) -> Result<(), CliError> {
    let Operation::LiveFlat { name, .. } = operation else {
        return Ok(());
    };
    if !head.y_axis {
        return Err(CliError::invalid(format!("operation {name}: head has no Y axis")));
    }
    let (Some(y_min), Some(y_max)) = (head.y_min, head.y_max) else {
        return Err(CliError::invalid(format!("operation {name}: head has no Y axis")));
    };
    let ToolBody::Live { diameter, .. } = &tool.body else {
        return Ok(());
    };
    let path = live_flat(&flat_params(operation, *diameter, clearance)?).map_err(|err| geometry_err(name, err))?;
    for step in &path.steps {
        let y = step.y();
        if y < y_min - 1e-9 || y > y_max + 1e-9 {
            return Err(CliError::invalid(format!("operation {name}: Y {} outside head travel", format_travel(y))));
        }
    }
    Ok(())
}

fn format_travel(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    let text = format!("{value:.9}");
    let Some((head, frac)) = text.split_once('.') else {
        return text;
    };
    let frac = frac.trim_end_matches('0');
    if frac.is_empty() { head.to_string() } else { format!("{head}.{frac}") }
}

fn geometry_err(name: &str, err: impl std::fmt::Display) -> CliError {
    let text = err.to_string();
    let detail = text.strip_prefix("invalid input: ").unwrap_or(text.as_str());
    CliError::invalid(format!("operation {name}: {detail}"))
}

fn parse_infeed(text: &str) -> Result<Infeed, CliError> {
    match text {
        "radial" => Ok(Infeed::Radial),
        "flank" => Ok(Infeed::Flank { angle_deg: DEFAULT_FLANK_DEG }),
        other => Err(CliError::invalid(format!("unknown infeed `{other}`"))),
    }
}

fn parse_side(text: &str) -> Result<Side, CliError> {
    match text {
        "outside" => Ok(Side::Outside),
        "inside" => Ok(Side::Inside),
        other => Err(CliError::invalid(format!("unknown side `{other}`"))),
    }
}

fn parse_speed(value: &Value, max_rpm: f64) -> Result<SpindleSpeed, CliError> {
    let map = object(value, "speed")?;
    check_keys(map, &["css", "rpm"], "speed")?;
    match (optional(map, "css"), optional(map, "rpm")) {
        (Some(css), None) => Ok(SpindleSpeed::css(positive(css, "css")?, max_rpm)?),
        (None, Some(rpm)) => Ok(SpindleSpeed::rpm(positive(rpm, "rpm")?)?),
        _ => Err(CliError::invalid("speed needs exactly one of `css` or `rpm`")),
    }
}

fn parse_tolerance(value: Option<&Value>) -> Result<(f64, f64), CliError> {
    let Some(value) = value else {
        return Ok((Tolerance::DEFAULT_LINEAR_MM, DEFAULT_JOB_CHORD_MM));
    };
    let map = object(value, "tolerance")?;
    check_keys(map, &["linear", "angular", "chord"], "tolerance")?;
    let linear = match optional(map, "linear") {
        Some(value) => positive(value, "linear")?,
        None => Tolerance::DEFAULT_LINEAR_MM,
    };
    if let Some(angular) = optional(map, "angular") {
        non_negative(angular, "angular")?;
    }
    let chord = match optional(map, "chord") {
        Some(value) => non_negative(value, "chord")?,
        None => DEFAULT_JOB_CHORD_MM,
    };
    Ok((linear, chord))
}

fn load_profile(job_path: &Path, job: &LatheJob, up_axis: crate::step::UpAxis) -> Result<TurnProfile, CliError> {
    load_source(job_path, &job.source, job.linear, up_axis)
}

fn load_source(job_path: &Path, source: &Source, linear: f64, up_axis: crate::step::UpAxis) -> Result<TurnProfile, CliError> {
    let path = resolve_source(job_path, &source.path);
    match &source.kind {
        SourceKind::Dxf { layer } => {
            let text = read_text(&path)?;
            TurnProfile::from_dxf(&text, layer.as_deref(), linear).map_err(show)
        }
        SourceKind::Svg { layer: None } => {
            let text = read_text(&path)?;
            TurnProfile::from_svg(&text, linear).map_err(show)
        }
        SourceKind::Svg { layer: Some(layer) } => {
            let text = read_text(&path)?;
            let tolerance = Tolerance::new(linear, Tolerance::DEFAULT_ANGULAR_RAD).map_err(show)?;
            let drawing = import_svg(&text, &ImportOptions::new(tolerance)).map_err(show)?;
            TurnProfile::from_drawing(&drawing, Some(layer), linear).map_err(show)
        }
        SourceKind::Stl { axis, step } => {
            let bytes = crate::run::read_bytes(&path)?;
            let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
            TurnProfile::from_mesh(&mesh, *axis, *step).map_err(show)
        }
        SourceKind::Step { body } => {
            let cli = crate::step::find_cli().ok_or_else(crate::step::missing_cli_error)?;
            let text = crate::step::section_to_dxf(&cli, &path, body.as_deref(), up_axis)?;
            TurnProfile::from_section_dxf(&text, linear).map_err(show)
        }
    }
}

fn resolve_source(job: &Path, relative: &str) -> PathBuf {
    match job.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(relative),
        _ => PathBuf::from(relative),
    }
}

fn build_document(
    job_path: &Path,
    job: &LatheJob,
    profile: &TurnProfile,
    sub_profile: Option<&TurnProfile>,
) -> Result<(Document, Vec<String>), CliError> {
    let stock = turn_stock(&job.stock)?;
    let points = profile.to_polyline_points(job.chord).into_iter().map(|point| [point.x(), point.y()]).collect();
    let mut setup = turning_setup(job, stock)?.with_section(Some(TurnSection::try_new(points)?))?;
    for tool in &job.tools {
        // Live tools are not mounted as inserts and are not gouge-checked. The cutter diameter
        // is stored on the tool table. A placeholder insert is not used.
        let ToolBody::Insert { insert, .. } = &tool.body else {
            continue;
        };
        let mounted = TurnInsert::try_new(tool.id, HEAD, tool.station, *insert, false)?;
        setup = setup.with_insert(mounted)?;
    }
    let check = job.operations.iter().any(|operation| gouge_side(operation).is_some());
    let sample_step = if check { Some(crate::gouge::sample_step_for(job.chord)?) } else { None };
    let section = sampled_section(profile, job.chord, sample_step.is_some());
    let sub_section = match (sample_step, sub_profile) {
        (Some(_), Some(sub)) => sampled_section(sub, job.chord, true),
        _ => Vec::new(),
    };
    let mut gouge_lines = Vec::new();
    let mut toolpaths = Vec::with_capacity(job.operations.len());
    let mut previous: Option<u32> = None;
    for (index, operation) in job.operations.iter().enumerate() {
        let active_profile = match operation.spindle() {
            WorkSpindle::Main => profile,
            WorkSpindle::Sub => sub_profile.ok_or_else(|| CliError::run("internal: sub profile is missing"))?,
        };
        let gouge_section = match operation.spindle() {
            WorkSpindle::Main => section.as_slice(),
            WorkSpindle::Sub => sub_section.as_slice(),
        };
        let toolpath = if operation.is_flat() {
            flat_toolpath(job, operation, &mut previous)?
        } else if operation.is_live() {
            let motion = live_motion(operation)?;
            let rpm = operation.live_rpm().ok_or_else(|| CliError::run(format!("internal: operation {} has no live rpm", operation.name())))?;
            with_live_events(motion, operation.tool_id(), rpm, &mut previous)?
        } else {
            let turned = turn_operation(job, active_profile, operation)?;
            if let Some(step) = sample_step
                && let Some(side) = gouge_side(operation)
            {
                gouge_lines.extend(turn_gouges(job, operation, index, &turned, side, gouge_section, step)?);
            }
            let motion = to_cl_path(&turned, Some(operation.name().to_string()), operation.tool_id(), operation.feed(), Some(job.clearance))
                .map_err(|err| op_fail(operation.name(), err))?;
            with_events(motion, operation.tool_id(), &mut previous)?
        };
        toolpaths.push(toolpath);
        let binding = TurnBinding::try_new(index as u32, operation.spindle().name(), HEAD, 0)?.with_speed(operation.turning_speed())?;
        setup = setup.with_binding(binding)?;
    }
    let tools = job
        .tools
        .iter()
        .map(|tool| {
            let diameter = match &tool.body {
                ToolBody::Live { diameter, .. } => Some(*diameter),
                ToolBody::Insert { .. } => None,
            };
            Tool::try_new(tool.id, None, diameter, None)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let program = Program::try_new(program_name(job_path), tools, None, toolpaths, Vec::new(), Vec::new(), None)?.with_turning(Some(setup))?;
    Ok((Document::new(program)?, gouge_lines))
}

/// Face is always outside. Rough and finish use the operation side. Thread, groove, part, drill
/// and live cycles are skipped: those tools are meant to enter the profile, or they are not inserts.
fn gouge_side(operation: &Operation) -> Option<Side> {
    match operation {
        Operation::Face { .. } => Some(Side::Outside),
        Operation::Rough { side, .. } | Operation::Finish { side, .. } => Some(*side),
        Operation::Thread { .. }
        | Operation::Groove { .. }
        | Operation::Part { .. }
        | Operation::Drill { .. }
        | Operation::LiveDrill { .. }
        | Operation::LiveSlot { .. }
        | Operation::LiveFlat { .. } => None,
    }
}

fn sampled_section(profile: &TurnProfile, chord: f64, sample: bool) -> Vec<(f64, f64)> {
    if !sample {
        return Vec::new();
    }
    profile.to_polyline_points(chord).into_iter().map(|point| (point.x(), point.y())).collect()
}

/// Spindle `main`, plus spindle `sub` when the job has a sub block. C axis id 0 when the job
/// asks for it. The head is `turret1`, X and Z, plus Y when `head.y_axis` is true, 12 stations.
/// `live_tooling` is set when any tool is live. A plain turning job with no sub keeps
/// [`TurningSetup::single`], which has no C axis, no Y and no live tooling. The sub spindle has
/// no C axis, work zero 0 and is not reversed. Stock geometry is the main blank either way.
fn turning_setup(job: &LatheJob, stock: TurnStock) -> Result<TurningSetup, CliError> {
    let live_tooling = job.tools.iter().any(|tool| tool.body.is_live());
    if !job.c_axis && !live_tooling && !job.head.y_axis && job.sub.is_none() {
        return Ok(TurningSetup::single(Some(stock), job.max_rpm)?);
    }
    let c_axis = if job.c_axis { Some(0) } else { None };
    let mut spindles = vec![Spindle::try_new(SPINDLE, c_axis, 0.0, job.max_rpm)?];
    if let Some(sub) = &job.sub {
        spindles.push(Spindle::try_new(SUB_SPINDLE, None, 0.0, sub.max_rpm)?);
    }
    let stations = u16::try_from(STATIONS).map_err(|_| CliError::run("turret station count is out of range"))?;
    let axes = if job.head.y_axis { vec![HeadAxis::X, HeadAxis::Y, HeadAxis::Z] } else { vec![HeadAxis::X, HeadAxis::Z] };
    let head = Head::try_new(HEAD, axes, stations, live_tooling)?;
    Ok(TurningSetup::try_new(spindles, vec![head], Some(stock), Vec::new(), Vec::new())?)
}

fn turn_gouges(
    job: &LatheJob,
    operation: &Operation,
    index: usize,
    path: &TurnPath,
    side: Side,
    section: &[(f64, f64)],
    sample_step: f64,
) -> Result<Vec<String>, CliError> {
    let (insert, tip, _nose_radius, holder) = require_insert(job, operation)?;
    let outline = InsertGeom::from_insert(insert, TURN_EDGE_MM).map_err(|err| op_fail(operation.name(), err))?;
    let mut found =
        check_turn_gouges(section, side, path, tip, &outline, holder, sample_step, job.chord).map_err(|err| op_fail(operation.name(), err))?;
    for gouge in &mut found {
        gouge.toolpath = index;
    }
    Ok(found.iter().map(|gouge| format_turn_gouge(operation.name(), gouge)).collect())
}

fn format_turn_gouge(name: &str, gouge: &TurnGouge) -> String {
    let mut clean = String::new();
    for ch in name.chars() {
        if ch.is_control() {
            clean.push(' ');
        } else {
            clean.push(ch);
        }
    }
    format!("gouge operation {clean} z {} x {} depth {} {}", num6(gouge.z), num6(gouge.x), num6(gouge.depth), gouge.what.name())
}

fn num6(value: f64) -> String {
    if value.is_finite() { format!("{value:.6}") } else { "non_finite".to_string() }
}

fn turn_stock(stock: &Stock) -> Result<TurnStock, CliError> {
    match *stock {
        Stock::Bar { diameter, length } => Ok(TurnStock::bar(diameter, length)?),
        Stock::Tube { outer, inner, length } => Ok(TurnStock::tube(outer, inner, length)?),
    }
}

fn require_insert<'a>(job: &'a LatheJob, operation: &Operation) -> Result<(&'a Insert, TipPoint, f64, Option<&'a HolderGeom>), CliError> {
    let tool = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?;
    match &tool.body {
        ToolBody::Insert { insert, tip, nose_radius, holder } => Ok((insert, *tip, *nose_radius, holder.as_ref())),
        ToolBody::Live { .. } => Err(CliError::run(format!("operation {}: tool {} is a live tool", operation.name(), tool.id))),
    }
}

/// Facing depth for this spindle. The sub uses `back_allowance` and the same stock diameters.
fn front_allowance(job: &LatheJob, spindle: WorkSpindle) -> Result<f64, CliError> {
    match spindle {
        WorkSpindle::Main => Ok(job.front_allowance),
        WorkSpindle::Sub => job.sub.as_ref().map(|block| block.back_allowance).ok_or_else(|| CliError::run("internal: sub spindle is missing")),
    }
}

fn turn_operation(job: &LatheJob, profile: &TurnProfile, operation: &Operation) -> Result<TurnPath, CliError> {
    if operation.is_live() {
        return Err(CliError::run(format!("internal: operation {} has no turning path", operation.name())));
    }
    let (insert, tip, nose_radius, _) = require_insert(job, operation)?;
    let turn_tool = TurnTool { nose_radius, tip };
    let stock = turn_stock(&job.stock)?;
    match operation {
        Operation::Face { name, spindle, step_z, .. } => {
            let params = FaceParams {
                stock_front_z: front_allowance(job, *spindle)?,
                final_z: 0.0,
                step_z: *step_z,
                from_radius: stock.outer_radius(),
                to_radius: face_to_radius(&turn_tool),
                clearance: job.clearance,
            };
            face(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Rough { name, side, depth_of_cut, leave_x, leave_z, .. } => {
            let params = RoughParams {
                side: *side,
                stock_outer_radius: stock.outer_radius(),
                stock_inner_radius: stock.inner_radius(),
                depth_of_cut: *depth_of_cut,
                leave_x: *leave_x,
                leave_z: *leave_z,
                clearance: job.clearance,
                retract: job.clearance,
            };
            rough(profile, &turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Finish { name, side, leave_x, leave_z, .. } => {
            let params = FinishParams { side: *side, leave_x: *leave_x, leave_z: *leave_z, clearance: job.clearance, tol: job.linear };
            finish(profile, &turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Thread {
            name,
            side,
            major_diameter,
            pitch,
            z_start,
            z_end,
            depth,
            first_depth,
            min_depth,
            spring_passes,
            infeed,
            clearance,
            ..
        } => {
            let params = ThreadParams {
                side: *side,
                major_radius: *major_diameter * 0.5,
                depth: *depth,
                pitch: *pitch,
                z_start: *z_start,
                z_end: *z_end,
                first_depth: *first_depth,
                min_depth: *min_depth,
                spring_passes: *spring_passes,
                infeed: *infeed,
                clearance: *clearance,
                included_angle_deg: 60.0,
            };
            thread(&params).map_err(|err| op_fail(name, err))
        }
        Operation::Groove {
            name,
            side,
            z_left,
            z_right,
            bottom_diameter,
            surface_diameter,
            insert_width,
            peck,
            stepover_frac,
            clearance,
            dwell_s,
            ..
        } => {
            let width = match insert_width {
                Some(width) => *width,
                None => insert.width().ok_or_else(|| op_fail(name, "groove insert needs a width"))?,
            };
            let params = GrooveParams {
                side: *side,
                z_left: *z_left,
                z_right: *z_right,
                bottom_radius: *bottom_diameter * 0.5,
                surface_radius: surface_diameter.map(|diameter| diameter * 0.5).unwrap_or_else(|| stock.outer_radius()),
                insert_width: width,
                peck: *peck,
                stepover_frac: *stepover_frac,
                clearance: *clearance,
                dwell_s: *dwell_s,
            };
            groove(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Part { name, z, to_diameter, chamfer, peck, clearance, .. } => {
            let width = insert.width().ok_or_else(|| op_fail(name, "part-off insert needs a width"))?;
            let params = PartParams {
                z: *z,
                surface_radius: stock.outer_radius(),
                insert_width: width,
                to_radius: *to_diameter * 0.5,
                chamfer: *chamfer,
                peck: *peck,
                clearance: *clearance,
            };
            part_off(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Drill { name, z_start, depth, diameter, peck, retract, point_angle_deg, through, .. } => {
            let params = DrillParams {
                z_start: *z_start,
                depth: *depth,
                peck: *peck,
                retract: *retract,
                clearance: job.clearance,
                point_angle_deg: *point_angle_deg,
                diameter: *diameter,
                through: *through,
            };
            centre_drill(&params).map_err(|err| op_fail(name, err))
        }
        Operation::LiveDrill { .. } | Operation::LiveSlot { .. } | Operation::LiveFlat { .. } => {
            Err(CliError::run(format!("internal: operation {} has no turning path", operation.name())))
        }
    }
}

/// End radius passed to [`face`].
///
/// A centre tip is shifted by `−nose_radius` inside [`face`], so `to_radius` 0 programs the nose
/// centre at `−nose_radius`. An imaginary tip is used as given, so `to_radius` is already
/// `−nose_radius`. Either way the cutting feed ends one nose radius past the spindle axis.
/// [`face`] also adds [`FaceParams::clearance`] to [`FaceParams::from_radius`], so the pass
/// starts at the stock outer radius plus the job clearance.
fn face_to_radius(tool: &TurnTool) -> f64 {
    match tool.tip {
        TipPoint::Centre => 0.0,
        TipPoint::ImaginaryTip { .. } => -tool.nose_radius,
    }
}

fn live_motion(operation: &Operation) -> Result<ClPath, CliError> {
    let (name, tool_id, feed, path) = match operation {
        Operation::LiveDrill { name, tool_id, feed, params, .. } => {
            let path = live_drill(params).map_err(|err| geometry_err(name, err))?;
            (name.as_str(), *tool_id, *feed, path)
        }
        Operation::LiveSlot { name, tool_id, feed, params, .. } => {
            let path = live_slot(params).map_err(|err| geometry_err(name, err))?;
            (name.as_str(), *tool_id, *feed, path)
        }
        _ => return Err(CliError::run(format!("internal: operation {} is not a live cycle", operation.name()))),
    };
    live_cl_path(name, tool_id, feed, &path)
}

/// Every motion carries the current C. The pose axis is +Z for a front (axial) tool and +X for
/// an OD (radial) tool. Rapids have no feed. Feeds are [`MoveRole::Cut`].
fn live_cl_path(name: &str, tool_id: u32, feed: f64, path: &LivePath) -> Result<ClPath, CliError> {
    let axis = if path.axial { Vec3::Z } else { Vec3::X };
    let feeds = Feeds::try_new(feed, None, None, None, None)?;
    let mut blocks = Vec::with_capacity(path.steps.len());
    for step in &path.steps {
        let block = match *step {
            LiveStep::Rapid { z, x, c_deg } => ClBlock::Motion(ClMotion::Rapid { to: live_pose(z, x, c_deg, axis)?, feed: None }),
            LiveStep::Feed { z, x, c_deg } => ClBlock::Motion(ClMotion::Linear { to: live_pose(z, x, c_deg, axis)?, feed, role: MoveRole::Cut }),
            LiveStep::Clamp => ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: true }),
            LiveStep::Release => ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: false }),
        };
        blocks.push(block);
    }
    Ok(ClPath::try_new(Some(name.to_string()), Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)?)
}

fn flat_params(operation: &Operation, tool_diameter: f64, clearance: f64) -> Result<LiveFlatParams, CliError> {
    let Operation::LiveFlat { c_deg, diameter, across, z_from, z_to, width, step_down, stepover, .. } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a flat", operation.name())));
    };
    Ok(LiveFlatParams {
        c_deg: *c_deg,
        stock_diameter: *diameter,
        across: *across,
        z_from: *z_from,
        z_to: *z_to,
        width: *width,
        step_down: *step_down,
        stepover: *stepover,
        tool_diameter,
        clearance,
    })
}

/// Tool change, C engage, the Y=0 rapid, brake clamp, live tool on, the feeds, live tool off,
/// brake release, C disengage.
fn flat_toolpath(job: &LatheJob, operation: &Operation, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let ToolBody::Live { diameter, .. } = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .map(|tool| &tool.body)
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?
    else {
        return Err(CliError::run(format!("operation {}: tool {} is not a live tool", operation.name(), operation.tool_id())));
    };
    let path = live_flat(&flat_params(operation, *diameter, job.clearance)?).map_err(|err| geometry_err(operation.name(), err))?;
    let Some(rpm) = operation.live_rpm() else {
        return Err(CliError::run(format!("internal: operation {} has no live rpm", operation.name())));
    };
    flat_blocks(operation.name(), operation.tool_id(), operation.feed(), rpm, &path, previous)
}

fn flat_blocks(name: &str, tool_id: u32, feed: f64, rpm: f64, path: &FlatPath, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let Some((first, rest)) = path.steps.split_first() else {
        return Err(CliError::run(format!("operation {name}: flat path is empty")));
    };
    let FlatStep::Rapid { .. } = first else {
        return Err(CliError::run(format!("operation {name}: flat path does not start on a rapid")));
    };
    let mut blocks = Vec::with_capacity(path.steps.len().saturating_add(6));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: true }));
    blocks.push(flat_block(first, feed)?);
    blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: true }));
    blocks.push(ClBlock::Event(Event::LiveToolOn { rpm, direction: LiveDirection::Forward }));
    for step in rest {
        blocks.push(flat_block(step, feed)?);
    }
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: false }));
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: false }));
    let feeds = Feeds::try_new(feed, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.to_string()), Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)?)
}

fn flat_block(step: &FlatStep, feed: f64) -> Result<ClBlock, CliError> {
    let pose = Pose::new(Vec3::new(step.x(), step.y(), step.z())?, Vec3::X)?.with_rotary(step.c_deg())?;
    let motion = match step {
        FlatStep::Rapid { .. } => ClMotion::Rapid { to: pose, feed: None },
        FlatStep::Feed { .. } => ClMotion::Linear { to: pose, feed, role: MoveRole::Cut },
    };
    Ok(ClBlock::Motion(motion))
}

fn live_pose(z: f64, x: f64, c_deg: f64, axis: Vec3) -> Result<Pose, CliError> {
    Ok(Pose::new(Vec3::new(x, 0.0, z)?, axis)?.with_rotary(c_deg)?)
}

/// Tool change when the id changes, then C engage, live tool forward, the cycle, live tool off,
/// C disengage. No turning-spindle [`Event::ProcessOn`]: the post rejects that while C is engaged.
fn with_live_events(motion: ClPath, tool_id: u32, rpm: f64, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(motion.blocks().len().saturating_add(5));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: true }));
    blocks.push(ClBlock::Event(Event::LiveToolOn { rpm, direction: LiveDirection::Forward }));
    blocks.extend(motion.blocks().iter().cloned());
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: false }));
    Ok(ClPath::try_new(motion.name().map(str::to_string), motion.tool_id(), motion.compensation(), motion.feeds(), None, Vec::new(), blocks)?)
}

fn with_events(motion: ClPath, tool_id: u32, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(motion.blocks().len().saturating_add(3));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }));
    blocks.extend(motion.blocks().iter().cloned());
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }));
    Ok(ClPath::try_new(motion.name().map(str::to_string), motion.tool_id(), motion.compensation(), motion.feeds(), None, Vec::new(), blocks)?)
}

fn program_name(path: &Path) -> Option<String> {
    let stem = path.file_stem().and_then(|stem| stem.to_str())?;
    if stem.is_empty() || stem.len() > 256 { None } else { Some(stem.to_string()) }
}

fn emit(args: &LatheArgs, job: &LatheJob, doc: &Document, post: Option<&campanion_post::PostDef>) -> Result<ExitCode, CliError> {
    let report = campanion_ir::verify(doc.program());
    if !report.issues().is_empty() {
        let mut err_out = io::stderr().lock();
        err_out.write_all(crate::report::format_report(&report).as_bytes())?;
        return Ok(ExitCode::from(1));
    }
    let text = match post {
        Some(post) => crate::post::render(doc.program(), post, job.chord)?,
        None => campanion_ir::write_ngc(doc.program())?,
    };
    crate::run::write_output(&args.output, &text)?;
    if let Some(ir_path) = &args.ir {
        crate::run::write_output(ir_path, &doc.to_json_pretty()?)?;
    }
    Ok(ExitCode::SUCCESS)
}

fn array<'a>(value: &'a Value, what: &str, max: usize) -> Result<&'a [Value], CliError> {
    let Some(list) = value.as_array() else {
        return Err(CliError::invalid(format!("{what} must be an array")));
    };
    if list.len() > max {
        return Err(CliError::invalid(format!("too many {what}: {} > {max}", list.len())));
    }
    Ok(list)
}

fn positive(value: &Value, what: &str) -> Result<f64, CliError> {
    let number = as_f64(value, what)?;
    if number > 0.0 { Ok(number) } else { Err(CliError::invalid(format!("{what} must be positive"))) }
}

fn non_negative(value: &Value, what: &str) -> Result<f64, CliError> {
    let number = as_f64(value, what)?;
    if number >= 0.0 { Ok(number) } else { Err(CliError::invalid(format!("{what} must be >= 0"))) }
}

fn optional_non_negative(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, CliError> {
    match optional(map, key) {
        Some(value) => Ok(Some(non_negative(value, key)?)),
        None => Ok(None),
    }
}

fn optional_positive(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, CliError> {
    match optional(map, key) {
        Some(value) => Ok(Some(positive(value, key)?)),
        None => Ok(None),
    }
}

fn optional_u8(map: &Map<String, Value>, key: &str) -> Result<Option<u8>, CliError> {
    let Some(value) = optional(map, key) else {
        return Ok(None);
    };
    let number = as_f64(value, key)?;
    if !(0.0..=255.0).contains(&number) {
        return Err(CliError::invalid(format!("{key} is out of range")));
    }
    let rounded = number.round();
    if (number - rounded).abs() > 1e-6 {
        return Err(CliError::invalid(format!("{key} must be an integer")));
    }
    Ok(Some(rounded as u8))
}

fn show(err: impl std::fmt::Display) -> CliError {
    CliError::run(err.to_string())
}

fn op_fail(name: &str, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {name}: {err}"))
}

#[cfg(test)]
mod tests {
    use campanion_geom::{Line, Point, Segment};
    use campanion_ir::{Hand, Insert, InsertShape};
    use campanion_turn::{FaceParams, FinishParams, Side, TipPoint, TurnMove, TurnProfile, TurnTool, face, finish};

    use super::{SourceKind, face_to_radius, parse_source, programmed_tip, source_kind};

    fn ok_source(value: serde_json::Value) -> super::Source {
        parse_source(&value).unwrap_or_else(|err| panic!("{err}"))
    }

    #[test]
    fn step_and_stp_sources_take_an_optional_body() {
        assert!(matches!(source_kind("Part.STEP").unwrap_or_else(|err| panic!("{err}")), SourceKind::Step { body: None }));
        assert!(matches!(source_kind("shaft.Stp").unwrap_or_else(|err| panic!("{err}")), SourceKind::Step { body: None }));
        let rejected = match source_kind("part.igs") {
            Ok(_) => panic!("igs was accepted"),
            Err(err) => err,
        };
        assert!(rejected.to_string().contains("source path must end in .dxf, .svg, .stl, .step or .stp"), "{rejected}");
        let named = ok_source(serde_json::json!({"path": "a.step", "body": "shaft"}));
        assert!(matches!(named.kind, SourceKind::Step { body: Some(name) } if name == "shaft"));
        let omitted = ok_source(serde_json::json!({"path": "a.stp"}));
        assert!(matches!(omitted.kind, SourceKind::Step { body: None }));
        let empty = match parse_source(&serde_json::json!({"path": "a.step", "body": ""})) {
            Ok(_) => panic!("empty body was accepted"),
            Err(err) => err,
        };
        assert!(empty.to_string().contains("body is empty"), "{empty}");
        let layer = match parse_source(&serde_json::json!({"path": "a.step", "layer": "0"})) {
            Ok(_) => panic!("layer was accepted"),
            Err(err) => err,
        };
        assert!(layer.to_string().contains("layer"), "{layer}");
    }

    fn close(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() <= 1e-9
    }

    fn tip_ok(insert: &Insert, name: &str) -> TipPoint {
        match programmed_tip(insert, name) {
            Ok(tip) => tip,
            Err(err) => panic!("{name}: {err}"),
        }
    }

    fn cylinder(radius: f64, length: f64) -> TurnProfile {
        let start = Point::new(0.0, radius).expect("start");
        let end = Point::new(-length, radius).expect("end");
        let line = Line::new(start, end).expect("line");
        TurnProfile::try_new(vec![Segment::Line(line)], 1e-6).expect("profile")
    }

    #[test]
    fn imaginary_tip_uses_insert_offset_signs() {
        let nose = 0.8;
        let at_45 = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 45.0).expect("insert");
        let (dx, dz) = at_45.tip_offset();
        assert!(close(dx, -nose) && close(dz, -nose), "45° offset dx {dx} dz {dz}");
        assert_eq!(tip_ok(&at_45, "imaginary"), TipPoint::ImaginaryTip { dz, dx });
        assert_eq!(tip_ok(&at_45, "centre"), TipPoint::Centre);

        // 135°: sin > 0 and cos < 0, so (dx, dz) = (−nose, +nose). Swapping the pair is visible.
        let at_135 = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 135.0).expect("insert");
        let (dx, dz) = at_135.tip_offset();
        assert!(close(dx, -nose), "135° dx {dx}");
        assert!(close(dz, nose), "135° dz {dz}");
        let tip = tip_ok(&at_135, "imaginary");
        assert_eq!(tip, TipPoint::ImaginaryTip { dz, dx });

        let tool = TurnTool { nose_radius: nose, tip };
        let finished =
            finish(&cylinder(20.0, 30.0), &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 0.5, tol: 1e-6 })
                .expect("finish");
        let mut cursor = finished.start;
        let mut straight = None;
        for motion in &finished.moves {
            if let TurnMove::Feed { z, x } = *motion
                && (x - cursor.1).abs() <= 1e-6
                && (z - cursor.0).abs() > 10.0
                && (x - 20.0).abs() < 1e-4
            {
                straight = Some((cursor.0, z, x));
            }
            cursor = match *motion {
                TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => (z, x),
                TurnMove::Thread { z, x, .. } => (z, x),
                TurnMove::Dwell { .. } => cursor,
            };
        }
        let Some((z0, z1, x)) = straight else {
            panic!("imaginary finish straight is not on the part radius: {:?}", finished.moves);
        };
        assert!(close(x, 20.0) || (x - 20.0).abs() < 1e-4, "straight x {x}");
        assert!(z0.max(z1) > 0.2, "Z should shift by +nose, high end was {z0} {z1}");
    }

    #[test]
    fn facing_ends_one_nose_past_centre() {
        let nose = 0.8;
        let insert = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 135.0).expect("insert");
        let tools = [TurnTool { nose_radius: nose, tip: TipPoint::Centre }, TurnTool { nose_radius: nose, tip: tip_ok(&insert, "imaginary") }];
        for tool in tools {
            let path = face(
                &tool,
                &FaceParams { stock_front_z: 1.0, final_z: 0.0, step_z: 0.5, from_radius: 25.0, to_radius: face_to_radius(&tool), clearance: 2.0 },
            )
            .expect("face");
            assert!((path.start.1 - 27.0).abs() < 1e-9, "start radius {}", path.start.1);
            let mut cursor = path.start;
            let mut saw_end = false;
            for motion in &path.moves {
                if let TurnMove::Feed { z, x } = *motion
                    && (z - cursor.0).abs() <= 1e-9
                    && (x - cursor.1).abs() > 1.0
                    && x < 0.0
                {
                    assert!((x + nose).abs() < 1e-9, "face end radius {x}");
                    saw_end = true;
                }
                cursor = match *motion {
                    TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => (z, x),
                    TurnMove::Thread { z, x, .. } => (z, x),
                    TurnMove::Dwell { .. } => cursor,
                };
            }
            assert!(saw_end, "facing feed did not cross the centre: {:?}", path.moves);
        }
    }
}
