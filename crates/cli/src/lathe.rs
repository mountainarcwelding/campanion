//! `lathe`: load a `campanion-lathe/1` file and write turning toolpaths.
//!
//! The source path is relative to the job file. `.dxf` and `.svg` are half-profiles in the
//! drawing plane (drawing X is Z, drawing Y is the radius). `.stl` is an outside-diameter
//! envelope. Units are millimetres. Feeds are millimetres per minute. Constant surface speed is
//! metres per minute and is capped at the spindle `max_rpm`.
//!
//! Z = 0 is the finished front face. `stock.front_allowance` is how far the blank sticks out in
//! +Z; facing cuts that down to 0. The stock record itself still occupies Z in `[-length, 0]`.
//! Each operation is one toolpath: a tool change when the tool id changes, spindle on, the
//! moves, then spindle off. Speed is stored on the turning binding. The IR stores the finished
//! section on the turning setup, sampled from the profile at the job chord tolerance.
//!
//! `campanion_ir::verify` runs before anything is written. Issues are printed and the process
//! exits 1. Face, rough and finish paths are checked against the finished section for insert and
//! holder gouges. A gouge is printed and the process exits 1. Nothing is written in either case.
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
    ClBlock, ClPath, Document, Event, Hand, Insert, InsertShape, ProcessKind, ProcessParams, Program, SpindleSpeed, Tool, TurnBinding, TurnInsert,
    TurnSection, TurnStock, TurningSetup,
};
use campanion_turn::{
    DrillParams, FaceParams, FinishParams, GrooveParams, HolderGeom, Infeed, InsertGeom, MeshAxis, PartParams, RoughParams, Side, ThreadParams,
    TipPoint, TurnGouge, TurnPath, TurnProfile, TurnTool, centre_drill, check_turn_gouges, face, finish, groove, part_off, rough, thread, to_cl_path,
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
const MAX_SPINDLE_RPM: f64 = 100_000.0;
const HEAD: &str = "turret1";
const SPINDLE: &str = "main";
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
}

struct LatheJob {
    source: Source,
    stock: Stock,
    front_allowance: f64,
    max_rpm: f64,
    tools: Vec<LatheTool>,
    operations: Vec<Operation>,
    linear: f64,
    chord: f64,
    clearance: f64,
}

struct Source {
    path: String,
    kind: SourceKind,
}

enum SourceKind {
    Dxf { layer: Option<String> },
    Svg { layer: Option<String> },
    Stl { axis: MeshAxis, step: f64 },
}

enum Stock {
    Bar { diameter: f64, length: f64 },
    Tube { outer: f64, inner: f64, length: f64 },
}

struct LatheTool {
    id: u32,
    station: u16,
    insert: Insert,
    tip: TipPoint,
    nose_radius: f64,
    holder: Option<HolderGeom>,
}

enum Operation {
    Face {
        name: String,
        tool_id: u32,
        step_z: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Rough {
        name: String,
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
        tool_id: u32,
        side: Side,
        leave_x: f64,
        leave_z: f64,
        feed: f64,
        speed: SpindleSpeed,
    },
    Thread {
        name: String,
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
            | Self::Drill { name, .. } => name,
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
            | Self::Drill { tool_id, .. } => *tool_id,
        }
    }

    fn feed(&self) -> f64 {
        match self {
            Self::Face { feed, .. }
            | Self::Rough { feed, .. }
            | Self::Finish { feed, .. }
            | Self::Groove { feed, .. }
            | Self::Part { feed, .. }
            | Self::Drill { feed, .. } => *feed,
            Self::Thread { .. } => THREAD_RECORD_FEED,
        }
    }

    fn speed(&self) -> SpindleSpeed {
        match self {
            Self::Face { speed, .. }
            | Self::Rough { speed, .. }
            | Self::Finish { speed, .. }
            | Self::Thread { speed, .. }
            | Self::Groove { speed, .. }
            | Self::Part { speed, .. }
            | Self::Drill { speed, .. } => *speed,
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
    let profile = load_profile(&args.job, &job)?;
    let (doc, gouges) = build_document(&args.job, &job, &profile)?;
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
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "-o" || arg.as_os_str() == "--ir" || arg.as_os_str() == "--post" {
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
    Ok(LatheArgs { job: PathBuf::from(job), output: PathBuf::from(output), ir: ir.map(PathBuf::from), post: post.map(PathBuf::from) })
}

fn parse_job(text: &str) -> Result<LatheJob, CliError> {
    let value: Value = serde_json::from_str(text).map_err(|err| CliError::run(format!("json: {err}")))?;
    let map = object(&value, "job")?;
    let tag = as_str(require(map, "format")?, "format tag")?;
    check_format(tag)?;
    check_keys(map, &["format", "source", "units", "stock", "spindle", "tools", "operations", "tolerance", "clearance"], "job")?;
    let units = as_str(require(map, "units")?, "units")?;
    if units != "mm" {
        return Err(CliError::invalid(format!("units must be mm, not `{units}`")));
    }
    let source = parse_source(require(map, "source")?)?;
    let (stock, front_allowance) = parse_stock(require(map, "stock")?)?;
    let max_rpm = parse_spindle(require(map, "spindle")?)?;
    let tools = parse_tools(require(map, "tools")?)?;
    let operations = parse_operations(require(map, "operations")?, max_rpm)?;
    for operation in &operations {
        if !tools.iter().any(|tool| tool.id == operation.tool_id()) {
            return Err(CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())));
        }
    }
    let (linear, chord) = parse_tolerance(optional(map, "tolerance"))?;
    let clearance = non_negative(require(map, "clearance")?, "clearance")?;
    Ok(LatheJob { source, stock, front_allowance, max_rpm, tools, operations, linear, chord, clearance })
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
    };
    Ok(Source { path: path.to_string(), kind: source_kind })
}

fn source_kind(path: &str) -> Result<SourceKind, CliError> {
    let ext = Path::new(path).extension().and_then(|ext| ext.to_str()).unwrap_or("");
    match ext.to_ascii_lowercase().as_str() {
        "dxf" => Ok(SourceKind::Dxf { layer: None }),
        "svg" => Ok(SourceKind::Svg { layer: None }),
        "stl" => Ok(SourceKind::Stl { axis: MeshAxis::Z, step: 1.0 }),
        _ => Err(CliError::invalid("source path must end in .dxf, .svg or .stl")),
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

fn parse_spindle(value: &Value) -> Result<f64, CliError> {
    let map = object(value, "spindle")?;
    check_keys(map, &["max_rpm"], "spindle")?;
    let max_rpm = positive(require(map, "max_rpm")?, "max_rpm")?;
    if max_rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("spindle max_rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    Ok(max_rpm)
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
    check_keys(map, &["id", "station", "shape", "nose_radius", "orientation_deg", "hand", "tip", "width", "holder"], "tool")?;
    let id = as_u32(require(map, "id")?, "tool id")?;
    let station_u = as_u32(require(map, "station")?, "station")?;
    if !(1..=STATIONS).contains(&station_u) {
        return Err(CliError::invalid(format!("station must be from 1 to {STATIONS}")));
    }
    let station = u16::try_from(station_u).map_err(|_| CliError::invalid(format!("station must be from 1 to {STATIONS}")))?;
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
    Ok(LatheTool { id, station, insert, tip, nose_radius, holder })
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

fn parse_operations(value: &Value, max_rpm: f64) -> Result<Vec<Operation>, CliError> {
    let list = array(value, "operations", MAX_OPERATIONS)?;
    let mut names = BTreeSet::new();
    let mut operations = Vec::with_capacity(list.len());
    for item in list {
        operations.push(parse_operation(item, max_rpm, &mut names)?);
    }
    Ok(operations)
}

fn parse_operation(value: &Value, max_rpm: f64, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    let map = object(value, "operation")?;
    let op_type = as_str(require(map, "type")?, "operation type")?;
    let allowed: &[&str] = match op_type {
        "face" => &["name", "type", "tool_id", "step_z", "feed", "speed"],
        "rough" => &["name", "type", "tool_id", "side", "depth_of_cut", "leave_x", "leave_z", "feed", "speed"],
        "finish" => &["name", "type", "tool_id", "side", "feed", "speed", "leave_x", "leave_z"],
        "thread" => &[
            "name",
            "type",
            "tool_id",
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
        "part" => &["name", "type", "tool_id", "z", "to_diameter", "chamfer", "peck", "clearance", "feed", "speed"],
        "drill" => &["name", "type", "tool_id", "z_start", "depth", "diameter", "peck", "retract", "point_angle_deg", "through", "feed", "rpm"],
        other => return Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
    match op_type {
        "face" | "rough" | "finish" | "groove" | "part" => {
            let feed = positive(require(map, "feed")?, "feed")?;
            let speed = parse_speed(require(map, "speed")?, max_rpm)?;
            match op_type {
                "face" => Ok(Operation::Face { name, tool_id, step_z: positive(require(map, "step_z")?, "step_z")?, feed, speed }),
                "rough" => Ok(Operation::Rough {
                    name,
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
                    tool_id,
                    side: parse_side(as_str(require(map, "side")?, "side")?)?,
                    leave_x: optional_non_negative(map, "leave_x")?.unwrap_or(0.0),
                    leave_z: optional_non_negative(map, "leave_z")?.unwrap_or(0.0),
                    feed,
                    speed,
                }),
                "groove" => parse_groove(map, name, tool_id, feed, speed),
                "part" => parse_part(map, name, tool_id, feed, speed),
                _ => Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
            }
        }
        "thread" => parse_thread(map, name, tool_id),
        "drill" => parse_drill(map, name, tool_id),
        other => Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    }
}

fn parse_thread(map: &Map<String, Value>, name: String, tool_id: u32) -> Result<Operation, CliError> {
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

fn parse_groove(map: &Map<String, Value>, name: String, tool_id: u32, feed: f64, speed: SpindleSpeed) -> Result<Operation, CliError> {
    let stepover_frac = match optional(map, "stepover_frac") {
        Some(value) => as_f64(value, "stepover_frac")?,
        None => 0.8,
    };
    if !(stepover_frac > 0.0 && stepover_frac <= 1.0) {
        return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
    }
    Ok(Operation::Groove {
        name,
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

fn parse_part(map: &Map<String, Value>, name: String, tool_id: u32, feed: f64, speed: SpindleSpeed) -> Result<Operation, CliError> {
    Ok(Operation::Part {
        name,
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

fn parse_drill(map: &Map<String, Value>, name: String, tool_id: u32) -> Result<Operation, CliError> {
    let point_angle_deg = match optional(map, "point_angle_deg") {
        Some(value) => as_f64(value, "point_angle_deg")?,
        None => 118.0,
    };
    Ok(Operation::Drill {
        name,
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

fn load_profile(job_path: &Path, job: &LatheJob) -> Result<TurnProfile, CliError> {
    let path = resolve_source(job_path, &job.source.path);
    match &job.source.kind {
        SourceKind::Dxf { layer } => {
            let text = read_text(&path)?;
            TurnProfile::from_dxf(&text, layer.as_deref(), job.linear).map_err(show)
        }
        SourceKind::Svg { layer: None } => {
            let text = read_text(&path)?;
            TurnProfile::from_svg(&text, job.linear).map_err(show)
        }
        SourceKind::Svg { layer: Some(layer) } => {
            let text = read_text(&path)?;
            let tolerance = Tolerance::new(job.linear, Tolerance::DEFAULT_ANGULAR_RAD).map_err(show)?;
            let drawing = import_svg(&text, &ImportOptions::new(tolerance)).map_err(show)?;
            TurnProfile::from_drawing(&drawing, Some(layer), job.linear).map_err(show)
        }
        SourceKind::Stl { axis, step } => {
            let bytes = crate::run::read_bytes(&path)?;
            let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
            TurnProfile::from_mesh(&mesh, *axis, *step).map_err(show)
        }
    }
}

fn resolve_source(job: &Path, relative: &str) -> PathBuf {
    match job.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(relative),
        _ => PathBuf::from(relative),
    }
}

fn build_document(job_path: &Path, job: &LatheJob, profile: &TurnProfile) -> Result<(Document, Vec<String>), CliError> {
    let stock = turn_stock(&job.stock)?;
    let points = profile.to_polyline_points(job.chord).into_iter().map(|point| [point.x(), point.y()]).collect();
    let mut setup = TurningSetup::single(Some(stock), job.max_rpm)?.with_section(Some(TurnSection::try_new(points)?))?;
    for tool in &job.tools {
        let mounted = TurnInsert::try_new(tool.id, HEAD, tool.station, tool.insert, false)?;
        setup = setup.with_insert(mounted)?;
    }
    let check = job.operations.iter().any(|operation| gouge_side(operation).is_some());
    let sample_step = if check { Some(crate::gouge::sample_step_for(job.chord)?) } else { None };
    let section: Vec<(f64, f64)> = if sample_step.is_some() {
        profile.to_polyline_points(job.chord).into_iter().map(|point| (point.x(), point.y())).collect()
    } else {
        Vec::new()
    };
    let mut gouge_lines = Vec::new();
    let mut toolpaths = Vec::with_capacity(job.operations.len());
    let mut previous: Option<u32> = None;
    for (index, operation) in job.operations.iter().enumerate() {
        let turned = turn_operation(job, profile, operation)?;
        if let Some(step) = sample_step
            && let Some(side) = gouge_side(operation)
        {
            gouge_lines.extend(turn_gouges(job, operation, index, &turned, side, &section, step)?);
        }
        let motion = to_cl_path(&turned, Some(operation.name().to_string()), operation.tool_id(), operation.feed(), Some(job.clearance))
            .map_err(|err| op_fail(operation.name(), err))?;
        toolpaths.push(with_events(motion, operation.tool_id(), &mut previous)?);
        let binding = TurnBinding::try_new(index as u32, SPINDLE, HEAD, 0)?.with_speed(Some(operation.speed()))?;
        setup = setup.with_binding(binding)?;
    }
    let tools = job.tools.iter().map(|tool| Tool::try_new(tool.id, None, None, None)).collect::<Result<Vec<_>, _>>()?;
    let program = Program::try_new(program_name(job_path), tools, None, toolpaths, Vec::new(), Vec::new(), None)?.with_turning(Some(setup))?;
    Ok((Document::new(program)?, gouge_lines))
}

/// Face is always outside. Rough and finish use the operation side. Thread, groove, part and
/// drill are skipped: those inserts are meant to enter the profile.
fn gouge_side(operation: &Operation) -> Option<Side> {
    match operation {
        Operation::Face { .. } => Some(Side::Outside),
        Operation::Rough { side, .. } | Operation::Finish { side, .. } => Some(*side),
        Operation::Thread { .. } | Operation::Groove { .. } | Operation::Part { .. } | Operation::Drill { .. } => None,
    }
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
    let tool = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?;
    let insert = InsertGeom::from_insert(&tool.insert, TURN_EDGE_MM).map_err(|err| op_fail(operation.name(), err))?;
    let mut found = check_turn_gouges(section, side, path, tool.tip, &insert, tool.holder.as_ref(), sample_step, job.chord)
        .map_err(|err| op_fail(operation.name(), err))?;
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

fn turn_operation(job: &LatheJob, profile: &TurnProfile, operation: &Operation) -> Result<TurnPath, CliError> {
    let tool = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?;
    let turn_tool = TurnTool { nose_radius: tool.nose_radius, tip: tool.tip };
    let stock = turn_stock(&job.stock)?;
    match operation {
        Operation::Face { name, step_z, .. } => {
            let params = FaceParams {
                stock_front_z: job.front_allowance,
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
                None => tool.insert.width().ok_or_else(|| op_fail(name, "groove insert needs a width"))?,
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
            let width = tool.insert.width().ok_or_else(|| op_fail(name, "part-off insert needs a width"))?;
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

    use super::{face_to_radius, programmed_tip};

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
