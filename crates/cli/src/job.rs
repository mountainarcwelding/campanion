//! One 2D profile job, as JSON, turned into a [`Document`] through `campanion-path`.
//!
//! This is the small profile document read by `ir` and `ngc`. The `campanion-job/1` file
//! (source hash, a tool table and several operations) lives in `jobfile`.
//!
//! The built-in sample is `sample.job.json`. Lengths are millimetres. Feeds are millimetres per
//! minute. A job object holds:
//! - `name` (optional)
//! - `process`: `spindle`, `torch`, `laser`, `extruder` or `knife`
//! - `cut_feed`
//! - `tool`: `id`, optional `name`, and either `diameter` or `kerf`
//! - `feeds` (optional): `lead_in`, `lead_out`, `plunge`, `retract`
//! - `params` (optional): `speed`, `power`, pierce and post delays and their toggles,
//!   `pierce_height`, `cut_height`
//! - `heights` (optional): `clearance` and `retract`, the same object a `campanion-job/1` file
//!   uses. Omitted fields default to 5 mm and 2 mm. Both must be finite, `retract` must be above
//!   the stock top, and `clearance` must be at least `retract`.
//! - `profile`: `side` (`inside`, `outside`, `on`), `direction` (`climb`, `conventional`),
//!   `compensation` (`computed`, `controller_left`, `controller_right`), optional `z`,
//!   `allowance`, `overcut`, `lead_in`, `lead_out`, `tabs`, `closed`, and `points` as `[x, y]`
//!
//! A lead is `{"type":"line","length":…}`, `{"type":"arc","radius":…}`,
//! `{"type":"perpendicular","length":…}` or `{"type":"none"}`. Tabs use `width` and either
//! `count` or `spacing`, with an optional `height`. A positive height is crossed at `z + height`
//! as feed along the compensated contour (lines, arcs and corners), never as a rapid. Omitted or
//! zero height is a full-depth tab: feed up to the retract height, rapid across at that height,
//! feed back down. The program rapids to the pierce XY at the clearance height, rapids down to
//! the retract height, feeds to depth, and rapids back to clearance after process-off. An XY
//! rapid below the retract height is rejected. That height is above the stock top.

use std::path::Path;

use campanion_geom::{Contour, Line, Point, Segment, Tolerance};
use campanion_ir::{CompensationMode, Document, Feeds, Import, ProcessKind, ProcessParams, Tool, VerifyOptions, verify_with};
use campanion_path::{Compensation, ControllerSide, CutDirection, Lead, ProfileOp, ProfileSide, TabSpec, Tool as PathTool};
use serde_json::{Map, Value};

use crate::error::{CliError, MAX_JSON_BYTES};
use crate::jobfile::{DEFAULT_CLEARANCE_MM, DEFAULT_RETRACT_MM};

/// Absolute coordinate cap, in millimetres. Larger values are rejected so a hostile file cannot
/// ask the offsetter for an unbounded shape.
const MAX_COORD_MM: f64 = 1.0e6;

/// How many tabs one profile will accept. Matches the path crate's own cap.
const MAX_TABS: usize = 4_096;

const MAX_NAME: usize = 256;

/// Built-in 40 mm by 20 mm outside profile.
const SAMPLE_JOB: &str = include_str!("sample.job.json");

/// A tool parsed once for `campanion-path` and once for the IR. Shared with `jobfile`.
pub(crate) struct ParsedTool {
    path: PathTool,
    ir: Tool,
}

impl ParsedTool {
    pub(crate) fn path_tool(&self) -> PathTool {
        self.path
    }

    pub(crate) fn ir(&self) -> &Tool {
        &self.ir
    }
}

struct ProfileSpec {
    side: ProfileSide,
    direction: CutDirection,
    path_compensation: Compensation,
    ir_compensation: CompensationMode,
    z: f64,
    allowance: Option<f64>,
    overcut: Option<f64>,
    lead_in: Lead,
    lead_out: Lead,
    tabs: Option<TabSpec>,
    closed: bool,
    points: Vec<(f64, f64)>,
}

struct Job {
    name: Option<String>,
    process: ProcessKind,
    cut_feed: f64,
    tool: ParsedTool,
    feeds: Feeds,
    params: ProcessParams,
    clearance: f64,
    retract: f64,
    profile: ProfileSpec,
}

/// Parse `text` and convert its profile into an IR document.
pub fn load(text: &str) -> Result<Document, CliError> {
    if text.len() > MAX_JSON_BYTES {
        return Err(CliError::invalid(format!("json is larger than {MAX_JSON_BYTES} bytes")));
    }
    let value: Value = serde_json::from_str(text).map_err(|err| CliError::run(format!("json: {err}")))?;
    realise(parse_job(&value)?)
}

/// The built-in sample job.
pub fn load_sample() -> Result<Document, CliError> {
    load(SAMPLE_JOB)
}

/// Read a UTF-8 file up to [`MAX_JSON_BYTES`].
pub fn read_text(path: &Path) -> Result<String, CliError> {
    use std::fs::File;
    use std::io::Read;

    let file = File::open(path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if let Ok(meta) = file.metadata()
        && meta.len() > MAX_JSON_BYTES as u64
    {
        return Err(CliError::invalid(format!("{} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    let mut text = String::new();
    let limit = MAX_JSON_BYTES as u64 + 1;
    file.take(limit).read_to_string(&mut text).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if text.len() > MAX_JSON_BYTES {
        return Err(CliError::invalid(format!("{} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    Ok(text)
}

fn realise(job: Job) -> Result<Document, CliError> {
    let op = profile_op(&job.profile, job.tool.path, job.clearance, job.retract);
    let mut import = Import::new(job.process, job.cut_feed)?;
    if let Some(name) = job.name {
        import = import.name(name)?;
    }
    import = import.feeds(job.feeds)?.params(job.params)?.tool(job.tool.ir)?;
    import = import.compensation(job.profile.ir_compensation);
    let contour = contour_from(job.profile.points, job.profile.closed)?;
    let profile = op.generate(&contour).map_err(|err| CliError::run(err.to_string()))?;
    let doc = Document::from_profile(&profile, &import)?;
    reject_xy_rapids_below_retract(&doc, job.clearance, job.retract)?;
    Ok(doc)
}

/// XY rapids on this 2.5D document have to stay at or above the retract height.
///
/// The retract height is above the stock top, so the floor is strictly above Z 0. A traverse
/// between the retract and clearance heights is left as a non-blocking finding, the same rule
/// `campanion run` uses.
fn reject_xy_rapids_below_retract(doc: &Document, clearance: f64, retract: f64) -> Result<(), CliError> {
    let report = verify_with(doc.program(), &VerifyOptions { safe_z: Some(clearance), retract_z: Some(retract) });
    let lines = crate::safety::blocking_rapids(&report, retract);
    match lines.first() {
        Some(line) => Err(CliError::run(line.clone())),
        None => Ok(()),
    }
}

fn profile_op(spec: &ProfileSpec, tool: PathTool, clearance: f64, retract: f64) -> ProfileOp {
    let mut op = ProfileOp::new(tool, spec.path_compensation, spec.side, spec.direction).z(spec.z).heights(clearance, retract);
    if let Some(allowance) = spec.allowance {
        op = op.allowance(allowance);
    }
    if let Some(overcut) = spec.overcut {
        op = op.overcut(overcut);
    }
    op = op.lead_in(spec.lead_in).lead_out(spec.lead_out);
    if let Some(tabs) = spec.tabs {
        op = op.tabs(tabs);
    }
    op
}

fn parse_job(value: &Value) -> Result<Job, CliError> {
    let map = object(value, "job")?;
    check_keys(map, &["name", "process", "cut_feed", "tool", "feeds", "params", "heights", "profile"], "job")?;
    let name = match optional(map, "name") {
        Some(value) => Some(required_name(as_str(value, "name")?, "name")?),
        None => None,
    };
    let process = parse_process(require(map, "process")?)?;
    let cut_feed = as_f64(require(map, "cut_feed")?, "cut_feed")?;
    let tool = parse_tool(require(map, "tool")?)?;
    let feeds = parse_feeds(optional(map, "feeds"), cut_feed)?;
    let params = parse_params(optional(map, "params"))?;
    let (clearance, retract) = parse_profile_heights(optional(map, "heights"))?;
    let profile = parse_profile(require(map, "profile")?)?;
    Ok(Job { name, process, cut_feed, tool, feeds, params, clearance, retract, profile })
}

/// Optional `heights`, the same fields as `campanion-job/1`. Lengths are millimetres.
///
/// An omitted field keeps the run default. `retract` has to be above the stock top, and
/// `clearance` has to be at least `retract`.
fn parse_profile_heights(value: Option<&Value>) -> Result<(f64, f64), CliError> {
    let mut clearance = DEFAULT_CLEARANCE_MM;
    let mut retract = DEFAULT_RETRACT_MM;
    if let Some(value) = value {
        let map = object(value, "heights")?;
        check_keys(map, &["clearance", "retract"], "heights")?;
        if let Some(value) = optional(map, "clearance") {
            clearance = as_f64(value, "clearance")?;
        }
        if let Some(value) = optional(map, "retract") {
            retract = as_f64(value, "retract")?;
        }
    }
    if retract <= 0.0 {
        return Err(CliError::invalid("retract must be above the stock top"));
    }
    if retract > clearance {
        return Err(CliError::invalid("retract is above clearance"));
    }
    Ok((clearance, retract))
}

fn parse_process(value: &Value) -> Result<ProcessKind, CliError> {
    match as_str(value, "process")? {
        "spindle" => Ok(ProcessKind::Spindle),
        "torch" => Ok(ProcessKind::Torch),
        "laser" => Ok(ProcessKind::Laser),
        "extruder" => Ok(ProcessKind::Extruder),
        "knife" => Ok(ProcessKind::Knife),
        other => Err(CliError::invalid(format!("unknown process `{other}`"))),
    }
}

pub(crate) fn parse_tool(value: &Value) -> Result<ParsedTool, CliError> {
    let map = object(value, "tool")?;
    check_keys(map, &["id", "name", "diameter", "kerf"], "tool")?;
    let id = as_u32(require(map, "id")?, "tool id")?;
    let name = match optional(map, "name") {
        Some(value) => Some(required_name(as_str(value, "tool name")?, "tool name")?),
        None => None,
    };
    let diameter = optional_f64(map, "diameter")?;
    let kerf = optional_f64(map, "kerf")?;
    let (path, diameter, kerf) = match (diameter, kerf) {
        (Some(diameter), None) => (PathTool::Diameter { diameter }, Some(diameter), None),
        (None, Some(kerf)) => (PathTool::Kerf { width: kerf }, None, Some(kerf)),
        (Some(_), Some(_)) => return Err(CliError::invalid("tool has both diameter and kerf")),
        (None, None) => return Err(CliError::invalid("tool needs a diameter or a kerf")),
    };
    let ir = Tool::try_new(id, name, diameter, kerf)?;
    Ok(ParsedTool { path, ir })
}

pub(crate) fn parse_feeds(value: Option<&Value>, cut_feed: f64) -> Result<Feeds, CliError> {
    let Some(value) = value else {
        return Ok(Feeds::try_new(cut_feed, None, None, None, None)?);
    };
    let map = object(value, "feeds")?;
    check_keys(map, &["lead_in", "lead_out", "plunge", "retract"], "feeds")?;
    Ok(Feeds::try_new(
        cut_feed,
        optional_f64(map, "lead_in")?,
        optional_f64(map, "lead_out")?,
        optional_f64(map, "plunge")?,
        optional_f64(map, "retract")?,
    )?)
}

pub(crate) fn parse_params(value: Option<&Value>) -> Result<ProcessParams, CliError> {
    let Some(value) = value else {
        return Ok(ProcessParams::new());
    };
    let map = object(value, "params")?;
    check_keys(
        map,
        &[
            "speed",
            "power",
            "pierce_delay_s",
            "pierce_process_on_during_delay",
            "post_delay_s",
            "post_process_on_during_delay",
            "pierce_height",
            "cut_height",
        ],
        "params",
    )?;
    let mut params = ProcessParams::new();
    if let Some(speed) = optional(map, "speed") {
        params = params.speed(Some(as_f64(speed, "speed")?))?;
    }
    if let Some(power) = optional(map, "power") {
        params = params.power(Some(as_f64(power, "power")?))?;
    }
    params = apply_delay(params, map, "pierce_delay_s", "pierce_process_on_during_delay", true)?;
    params = apply_delay(params, map, "post_delay_s", "post_process_on_during_delay", false)?;
    if let Some(z) = optional(map, "pierce_height") {
        params = params.pierce_height(Some(as_f64(z, "pierce_height")?))?;
    }
    if let Some(z) = optional(map, "cut_height") {
        params = params.cut_height(Some(as_f64(z, "cut_height")?))?;
    }
    Ok(params)
}

fn apply_delay(params: ProcessParams, map: &Map<String, Value>, delay_key: &str, toggle_key: &str, pierce: bool) -> Result<ProcessParams, CliError> {
    let delay = optional(map, delay_key);
    let toggle = optional(map, toggle_key);
    if delay.is_none() && toggle.is_none() {
        return Ok(params);
    }
    let seconds = match delay {
        Some(value) => as_f64(value, delay_key)?,
        None => {
            if pierce {
                params.pierce_delay_s()
            } else {
                params.post_delay_s()
            }
        }
    };
    let on = match toggle {
        Some(value) => as_bool(value, toggle_key)?,
        None => {
            if pierce {
                params.pierce_process_on_during_delay()
            } else {
                params.post_process_on_during_delay()
            }
        }
    };
    let updated = if pierce { params.pierce_delay(seconds, on) } else { params.post_delay(seconds, on) };
    updated.map_err(CliError::from)
}

fn parse_profile(value: &Value) -> Result<ProfileSpec, CliError> {
    let map = object(value, "profile")?;
    check_keys(
        map,
        &["side", "direction", "compensation", "z", "allowance", "overcut", "lead_in", "lead_out", "tabs", "closed", "points"],
        "profile",
    )?;
    let (path_compensation, ir_compensation) = parse_compensation(require(map, "compensation")?)?;
    Ok(ProfileSpec {
        side: parse_side(as_str(require(map, "side")?, "side")?)?,
        direction: parse_direction(as_str(require(map, "direction")?, "direction")?)?,
        path_compensation,
        ir_compensation,
        z: match optional(map, "z") {
            Some(value) => as_f64(value, "z")?,
            None => 0.0,
        },
        allowance: optional_f64(map, "allowance")?,
        overcut: optional_f64(map, "overcut")?,
        lead_in: parse_lead(optional(map, "lead_in"))?,
        lead_out: parse_lead(optional(map, "lead_out"))?,
        tabs: parse_tabs(optional(map, "tabs"))?,
        closed: as_bool(require(map, "closed")?, "closed")?,
        points: parse_points(require(map, "points")?)?,
    })
}

pub(crate) fn parse_side(text: &str) -> Result<ProfileSide, CliError> {
    match text {
        "inside" => Ok(ProfileSide::Inside),
        "outside" => Ok(ProfileSide::Outside),
        "on" => Ok(ProfileSide::On),
        _ => Err(CliError::invalid(format!("unknown profile side `{text}`"))),
    }
}

fn parse_direction(text: &str) -> Result<CutDirection, CliError> {
    match text {
        "climb" => Ok(CutDirection::Climb),
        "conventional" => Ok(CutDirection::Conventional),
        _ => Err(CliError::invalid(format!("unknown direction `{text}`"))),
    }
}

fn parse_compensation(value: &Value) -> Result<(Compensation, CompensationMode), CliError> {
    match as_str(value, "compensation")? {
        "computed" => Ok((Compensation::Computed, CompensationMode::Computed)),
        "controller_left" => Ok((Compensation::Controller(ControllerSide::Left), CompensationMode::ControllerLeft)),
        "controller_right" => Ok((Compensation::Controller(ControllerSide::Right), CompensationMode::ControllerRight)),
        other => Err(CliError::invalid(format!("unknown compensation `{other}`"))),
    }
}

fn parse_lead(value: Option<&Value>) -> Result<Lead, CliError> {
    let Some(value) = value else {
        return Ok(Lead::None);
    };
    let map = object(value, "lead")?;
    let kind = as_str(require(map, "type")?, "lead type")?;
    match kind {
        "none" => {
            check_keys(map, &["type"], "lead")?;
            Ok(Lead::None)
        }
        "line" => {
            check_keys(map, &["type", "length"], "lead")?;
            Ok(Lead::Line { length: as_f64(require(map, "length")?, "lead length")? })
        }
        "perpendicular" => {
            check_keys(map, &["type", "length"], "lead")?;
            Ok(Lead::Perpendicular { length: as_f64(require(map, "length")?, "lead length")? })
        }
        "arc" => {
            check_keys(map, &["type", "radius"], "lead")?;
            Ok(Lead::Arc { radius: as_f64(require(map, "radius")?, "lead radius")? })
        }
        _ => Err(CliError::invalid(format!("unknown lead type `{kind}`"))),
    }
}

fn parse_tabs(value: Option<&Value>) -> Result<Option<TabSpec>, CliError> {
    let Some(value) = value else {
        return Ok(None);
    };
    let map = object(value, "tabs")?;
    check_keys(map, &["count", "spacing", "width", "height"], "tabs")?;
    let width = as_f64(require(map, "width")?, "tab width")?;
    let spec = match (optional(map, "count"), optional(map, "spacing")) {
        (Some(count), None) => {
            let count = as_usize(count, "tab count", MAX_TABS)?;
            if count == 0 {
                return Err(CliError::invalid("tab count must be positive"));
            }
            TabSpec::count(count, width)
        }
        (None, Some(spacing)) => TabSpec::spacing(as_f64(spacing, "tab spacing")?, width),
        (None, None) => return Err(CliError::invalid("tabs need count or spacing")),
        (Some(_), Some(_)) => return Err(CliError::invalid("tabs have both count and spacing")),
    };
    let spec = match optional(map, "height") {
        Some(height) => spec.height(as_f64(height, "tab height")?),
        None => spec,
    };
    Ok(Some(spec))
}

fn parse_points(value: &Value) -> Result<Vec<(f64, f64)>, CliError> {
    let Some(list) = value.as_array() else {
        return Err(CliError::invalid("points must be an array"));
    };
    if list.len() > campanion_geom::MAX_SEGMENTS {
        return Err(CliError::invalid(format!("too many points: {} > {}", list.len(), campanion_geom::MAX_SEGMENTS)));
    }
    let mut points = Vec::with_capacity(list.len());
    for (index, item) in list.iter().enumerate() {
        let Some(pair) = item.as_array() else {
            return Err(CliError::invalid(format!("point {index} must be [x, y]")));
        };
        if pair.len() != 2 {
            return Err(CliError::invalid(format!("point {index} must be [x, y]")));
        }
        let (Some(x), Some(y)) = (pair.first(), pair.get(1)) else {
            return Err(CliError::invalid(format!("point {index} must be [x, y]")));
        };
        let x = as_f64(x, &format!("point {index} x"))?;
        let y = as_f64(y, &format!("point {index} y"))?;
        if x.abs() > MAX_COORD_MM || y.abs() > MAX_COORD_MM {
            return Err(CliError::invalid(format!("point {index} is outside ±{MAX_COORD_MM} mm")));
        }
        points.push((x, y));
    }
    Ok(points)
}

fn contour_from(points: Vec<(f64, f64)>, closed: bool) -> Result<Contour, CliError> {
    let points = prepare_points(points, closed)?;
    let mut segments = Vec::with_capacity(points.len());
    for pair in points.windows(2) {
        let (Some(&start), Some(&end)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        segments.push(line_segment(start, end)?);
    }
    if closed {
        let (Some(&start), Some(&end)) = (points.last(), points.first()) else {
            return Err(CliError::invalid("contour is empty"));
        };
        segments.push(line_segment(start, end)?);
    }
    if segments.is_empty() {
        return Err(CliError::invalid("contour is empty"));
    }
    if segments.len() > campanion_geom::MAX_SEGMENTS {
        return Err(CliError::invalid(format!("too many segments: {} > {}", segments.len(), campanion_geom::MAX_SEGMENTS)));
    }
    Contour::try_new(segments, closed, &Tolerance::default()).map_err(|err| CliError::run(err.to_string()))
}

fn prepare_points(mut points: Vec<(f64, f64)>, closed: bool) -> Result<Vec<(f64, f64)>, CliError> {
    if closed
        && let (Some(first), Some(last)) = (points.first().copied(), points.last().copied())
        && near(first, last)
    {
        points.pop();
    }
    let min = if closed { 3 } else { 2 };
    if points.len() < min {
        return Err(CliError::invalid(format!("contour needs at least {min} points")));
    }
    Ok(points)
}

fn near(a: (f64, f64), b: (f64, f64)) -> bool {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let distance = dx.hypot(dy);
    distance.is_finite() && distance <= Tolerance::DEFAULT_LINEAR_MM
}

fn line_segment(start: (f64, f64), end: (f64, f64)) -> Result<Segment, CliError> {
    let start = Point::new(start.0, start.1).map_err(|err| CliError::run(err.to_string()))?;
    let end = Point::new(end.0, end.1).map_err(|err| CliError::run(err.to_string()))?;
    let line = Line::new(start, end).map_err(|err| CliError::run(err.to_string()))?;
    Ok(Segment::Line(line))
}

pub(crate) fn object<'a>(value: &'a Value, what: &str) -> Result<&'a Map<String, Value>, CliError> {
    value.as_object().ok_or_else(|| CliError::invalid(format!("{what} must be an object")))
}

pub(crate) fn check_keys(map: &Map<String, Value>, allowed: &[&str], what: &str) -> Result<(), CliError> {
    for key in map.keys() {
        if !allowed.iter().any(|name| *name == key) {
            return Err(CliError::invalid(format!("unknown {what} field `{key}`")));
        }
    }
    Ok(())
}

pub(crate) fn require<'a>(map: &'a Map<String, Value>, key: &str) -> Result<&'a Value, CliError> {
    match map.get(key) {
        Some(value) if !value.is_null() => Ok(value),
        _ => Err(CliError::invalid(format!("missing `{key}`"))),
    }
}

pub(crate) fn optional<'a>(map: &'a Map<String, Value>, key: &str) -> Option<&'a Value> {
    map.get(key).filter(|value| !value.is_null())
}

pub(crate) fn optional_f64(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, CliError> {
    match optional(map, key) {
        Some(value) => Ok(Some(as_f64(value, key)?)),
        None => Ok(None),
    }
}

pub(crate) fn as_f64(value: &Value, what: &str) -> Result<f64, CliError> {
    let Some(number) = value.as_f64() else {
        return Err(CliError::invalid(format!("{what} must be a number")));
    };
    if number.is_finite() { Ok(number) } else { Err(CliError::invalid(format!("{what} must be finite"))) }
}

pub(crate) fn as_bool(value: &Value, what: &str) -> Result<bool, CliError> {
    value.as_bool().ok_or_else(|| CliError::invalid(format!("{what} must be a boolean")))
}

pub(crate) fn as_str<'a>(value: &'a Value, what: &str) -> Result<&'a str, CliError> {
    let Some(text) = value.as_str() else {
        return Err(CliError::invalid(format!("{what} must be a string")));
    };
    if text.len() > MAX_NAME {
        return Err(CliError::invalid(format!("{what} is longer than {MAX_NAME} bytes")));
    }
    Ok(text)
}

pub(crate) fn required_name(text: &str, what: &str) -> Result<String, CliError> {
    if text.is_empty() {
        return Err(CliError::invalid(format!("{what} is empty")));
    }
    Ok(text.to_string())
}

pub(crate) fn as_u32(value: &Value, what: &str) -> Result<u32, CliError> {
    let number = as_f64(value, what)?;
    if !(0.0..=f64::from(u32::MAX)).contains(&number) {
        return Err(CliError::invalid(format!("{what} is out of range")));
    }
    let rounded = number.round();
    if (number - rounded).abs() > 1e-6 {
        return Err(CliError::invalid(format!("{what} must be an integer")));
    }
    Ok(rounded as u32)
}

fn as_usize(value: &Value, what: &str, max: usize) -> Result<usize, CliError> {
    let number = as_f64(value, what)?;
    let max_f = max as f64;
    if !(0.0..=max_f).contains(&number) {
        return Err(CliError::invalid(format!("{what} is out of range")));
    }
    let rounded = number.round();
    if (number - rounded).abs() > 1e-6 {
        return Err(CliError::invalid(format!("{what} must be an integer")));
    }
    Ok(rounded as usize)
}
