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
//! exits 1. `--post` writes that dialect instead of the test-sink NGC and compares the read-back
//! at the job chord tolerance. A mismatch exits 1 and nothing is written. ZX arcs in the test
//! sink are `G18`.

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
    FaceParams, FinishParams, MeshAxis, RoughParams, Side, TipPoint, TurnPath, TurnProfile, TurnTool, face, finish, rough, to_cl_path,
};
use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_f64, as_str, as_u32, check_keys, object, optional, read_text, require, required_name};
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
}

enum Operation {
    Face { name: String, tool_id: u32, step_z: f64, feed: f64, speed: SpindleSpeed },
    Rough { name: String, tool_id: u32, side: Side, depth_of_cut: f64, leave_x: f64, leave_z: f64, feed: f64, speed: SpindleSpeed },
    Finish { name: String, tool_id: u32, side: Side, leave_x: f64, leave_z: f64, feed: f64, speed: SpindleSpeed },
}

impl Operation {
    fn name(&self) -> &str {
        match self {
            Self::Face { name, .. } | Self::Rough { name, .. } | Self::Finish { name, .. } => name,
        }
    }

    fn tool_id(&self) -> u32 {
        match self {
            Self::Face { tool_id, .. } | Self::Rough { tool_id, .. } | Self::Finish { tool_id, .. } => *tool_id,
        }
    }

    fn feed(&self) -> f64 {
        match self {
            Self::Face { feed, .. } | Self::Rough { feed, .. } | Self::Finish { feed, .. } => *feed,
        }
    }

    fn speed(&self) -> SpindleSpeed {
        match self {
            Self::Face { speed, .. } | Self::Rough { speed, .. } | Self::Finish { speed, .. } => *speed,
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
    let doc = build_document(&args.job, &job, &profile)?;
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
    check_keys(map, &["id", "station", "shape", "nose_radius", "orientation_deg", "hand", "tip", "width"], "tool")?;
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
    Ok(LatheTool { id, station, insert, tip, nose_radius })
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
        other => return Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
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
        other => Err(CliError::invalid(format!("unknown operation type `{other}`"))),
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

fn build_document(job_path: &Path, job: &LatheJob, profile: &TurnProfile) -> Result<Document, CliError> {
    let stock = turn_stock(&job.stock)?;
    let points = profile.to_polyline_points(job.chord).into_iter().map(|point| [point.x(), point.y()]).collect();
    let mut setup = TurningSetup::single(Some(stock), job.max_rpm)?.with_section(Some(TurnSection::try_new(points)?))?;
    for tool in &job.tools {
        let mounted = TurnInsert::try_new(tool.id, HEAD, tool.station, tool.insert, false)?;
        setup = setup.with_insert(mounted)?;
    }
    let mut toolpaths = Vec::with_capacity(job.operations.len());
    let mut previous: Option<u32> = None;
    for (index, operation) in job.operations.iter().enumerate() {
        let turned = turn_operation(job, profile, operation)?;
        let motion = to_cl_path(&turned, Some(operation.name().to_string()), operation.tool_id(), operation.feed(), Some(job.clearance))
            .map_err(|err| op_fail(operation.name(), err))?;
        toolpaths.push(with_events(motion, operation.tool_id(), &mut previous)?);
        let binding = TurnBinding::try_new(index as u32, SPINDLE, HEAD, 0)?.with_speed(Some(operation.speed()))?;
        setup = setup.with_binding(binding)?;
    }
    let tools = job.tools.iter().map(|tool| Tool::try_new(tool.id, None, None, None)).collect::<Result<Vec<_>, _>>()?;
    let program = Program::try_new(program_name(job_path), tools, None, toolpaths, Vec::new(), Vec::new(), None)?.with_turning(Some(setup))?;
    Ok(Document::new(program)?)
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
