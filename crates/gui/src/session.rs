//! Job session logic used by the GUI: import, edit, plan, post and export paths.
//!
//! This module talks to the existing CLI planning and posting code. It does not invent machine
//! limits, feeds or material defaults. Empty fields stay empty until the user types a value.

use std::path::{Path, PathBuf};

use campanion_cli::{
    FORMAT_TAG, ImportedPart, JobFile, LayerSummary, PlanOptions, SourceKind, builtin_posts, import_part, parse_job, plan_job, post_job,
    suggested_post,
};
use campanion_geom::Segment;
use campanion_import::{Drawing, place_drawing};
use campanion_ir::Document;
use campanion_path::{Compensation, ControllerSide, CutDirection, Lead, Tool as PathTool};
use campanion_view::{Scene, Stroke, StrokeKind, tessellate_arc};
use serde_json::{Map, Value};

use crate::DEFAULT_ARC_TOL_MM;

/// Visible notice that CAMpanion is still primarily command-line driven.
pub const GUI_PREVIEW_NOTICE: &str = "Preview: CAMpanion is primarily command-line driven; the GUI is still in development.";

/// Shorter status-bar form of [`GUI_PREVIEW_NOTICE`].
pub const GUI_PREVIEW_NOTICE_SHORT: &str = "Preview: primarily command-line; GUI still in development";

/// A notice to show in the GUI.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SessionMessage {
    /// Severity.
    pub kind: MessageKind,
    /// Already-formatted text.
    pub text: String,
}

/// Kind of [`SessionMessage`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageKind {
    /// A refusal or verify error. Planning or posting did not produce a usable program.
    Error,
    /// A backend warning the user should see (S0 power, source hash, skipped entities).
    Warning,
    /// Informational notes such as stock peaks.
    Note,
}

/// How a tool is declared in the job file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolKind {
    /// `kerf` width, millimetres.
    Kerf,
    /// Rotating-cutter `diameter`, millimetres.
    Diameter,
    /// `wire` diameter, millimetres.
    Wire,
}

/// An editable tool row. Size stays empty until the user or a loaded job fills it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ToolDraft {
    pub id: u32,
    pub name: String,
    pub kind: ToolKind,
    /// Diameter, kerf or wire width, millimetres, as typed.
    pub size: String,
}

/// An editable profile (or other) operation. Numeric fields start empty for a new operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpDraft {
    pub name: String,
    /// Job `type` value: `profile`, `pocket`, and so on.
    pub kind: String,
    pub layers: Vec<String>,
    pub side: String,
    pub tool_id: u32,
    pub depth: String,
    pub step_down: String,
    pub cut_feed: String,
    pub plunge_feed: String,
    /// Process power as an IR fraction in `0..=1`. Empty until the user enters it.
    pub power: String,
    /// `none`, `line`, `arc`, `perpendicular`, or empty (job default).
    pub lead_in: String,
    pub lead_in_size: String,
    pub lead_out: String,
    pub lead_out_size: String,
    pub direction: String,
    pub compensation: String,
    pub pierce_delay: String,
    pub post_delay: String,
}

impl OpDraft {
    /// A new profile operation with empty numeric fields.
    pub fn new_profile(name: impl Into<String>, tool_id: u32) -> Self {
        Self {
            name: name.into(),
            kind: "profile".to_string(),
            layers: Vec::new(),
            side: "outside".to_string(),
            tool_id,
            depth: String::new(),
            step_down: String::new(),
            cut_feed: String::new(),
            plunge_feed: String::new(),
            power: String::new(),
            lead_in: String::new(),
            lead_in_size: String::new(),
            lead_out: String::new(),
            lead_out_size: String::new(),
            direction: String::new(),
            compensation: String::new(),
            pierce_delay: String::new(),
            post_delay: String::new(),
        }
    }
}

/// What the process tree has selected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TreeSelection {
    /// The imported part (the job source).
    Source,
    /// A tool table row.
    Tool(usize),
    /// An operation.
    Operation(usize),
    /// A source layer name.
    Layer(String),
}

/// In-memory job the GUI can edit, plan and save.
#[derive(Debug, Clone)]
pub struct JobSession {
    /// Path of a saved job file, when one is open.
    pub job_path: Option<PathBuf>,
    /// Original JSON, used to preserve fields the GUI does not edit.
    pub original_json: Option<Value>,
    /// Units tag as written in the file: `mm` or `inch`. Displayed values are millimetres.
    pub units: String,
    /// Part path, as stored or imported.
    pub source_path: Option<PathBuf>,
    /// Source hash, when known.
    pub source_hash: Option<String>,
    /// Kind of the source file, when known.
    pub source_kind: Option<SourceKind>,
    /// Layers from the imported drawing.
    pub layers: Vec<LayerSummary>,
    pub tools: Vec<ToolDraft>,
    pub operations: Vec<OpDraft>,
    pub selected: Option<TreeSelection>,
    /// Built-in post id or a `.json` post path. Empty until the user picks one.
    pub post_id: String,
    /// Untransformed source drawing, when a 2D part is loaded.
    pub source_drawing: Option<Drawing>,
    /// Placement X, millimetres, as typed.
    pub placement_x: String,
    /// Placement Y, millimetres, as typed.
    pub placement_y: String,
    /// Placement rotation, degrees counter-clockwise, as typed.
    pub placement_rotate: String,
    /// User machine profile id (file stem), when the job names one.
    pub machine_profile: String,
    /// Explicit profile path, used when `machine_profile` is empty.
    pub machine_path: String,
    /// Selected head id on that profile.
    pub machine_head: String,
    pub dirty: bool,
    pub messages: Vec<SessionMessage>,
}

impl JobSession {
    /// An empty session.
    pub fn new() -> Self {
        Self {
            job_path: None,
            original_json: None,
            units: "mm".to_string(),
            source_path: None,
            source_hash: None,
            source_kind: None,
            layers: Vec::new(),
            tools: Vec::new(),
            operations: Vec::new(),
            selected: None,
            post_id: String::new(),
            source_drawing: None,
            placement_x: String::new(),
            placement_y: String::new(),
            placement_rotate: String::new(),
            machine_profile: String::new(),
            machine_path: String::new(),
            machine_head: String::new(),
            dirty: false,
            messages: Vec::new(),
        }
    }
}

impl Default for JobSession {
    fn default() -> Self {
        Self::new()
    }
}

/// Load a `campanion-job/1` file into an editable session and, when possible, import its source.
pub fn open_job_file(path: &Path) -> Result<(JobSession, Option<ImportedPart>), String> {
    let text = std::fs::read_to_string(path).map_err(|err| format!("{}: {err}", path.display()))?;
    open_job_text(path, &text)
}

/// Parse job text the same way as [`open_job_file`].
pub fn open_job_text(path: &Path, text: &str) -> Result<(JobSession, Option<ImportedPart>), String> {
    let value: Value = serde_json::from_str(text).map_err(|err| format!("{}: {err}", path.display()))?;
    let job = parse_job(text, path).map_err(|err| err.to_string())?;
    let mut session = session_from_job(path, value, &job);
    let imported = match import_part(&campanion_cli::source_path(&job)) {
        Ok(part) => {
            session.layers = part.layers.clone();
            session.source_kind = Some(part.kind);
            session.source_drawing = part.drawing.clone();
            attach_import_messages(&mut session, &part);
            Some(part)
        }
        Err(err) => {
            session.messages.push(warning(err.to_string()));
            None
        }
    };
    Ok((session, imported))
}

/// Start a new job from an imported part. No tools or operations are invented.
pub fn session_from_import(part: &ImportedPart) -> JobSession {
    let mut session = JobSession::new();
    session.source_path = Some(part.path.clone());
    session.source_hash = Some(part.hash.clone());
    session.source_kind = Some(part.kind);
    session.layers = part.layers.clone();
    session.source_drawing = part.drawing.clone();
    session.dirty = true;
    if part.kind == SourceKind::Step {
        session.messages.push(note(
            "STEP import is recorded. A job must name a face, and planning uses solidcraft-cli, the same way as campanion run.".to_string(),
        ));
    }
    attach_import_messages(&mut session, part);
    session
}

fn attach_import_messages(session: &mut JobSession, part: &ImportedPart) {
    for text in &part.warnings {
        session.messages.push(warning(text.clone()));
    }
    if part.skipped_entities > 0 {
        let kinds = part.skipped_types.iter().map(|(name, count)| format!("{name} x{count}")).collect::<Vec<_>>().join(", ");
        session.messages.push(warning(format!("skipped source entities: {kinds}")));
    }
}

/// Import a part file (DXF, SVG, STEP or STL).
pub fn import_part_file(path: &Path) -> Result<ImportedPart, String> {
    import_part(path).map_err(|err| {
        let text = err.to_string();
        if text.starts_with("Could not import") { text } else { format!("Could not import {}: {text}", path.display()) }
    })
}

fn session_from_job(path: &Path, value: Value, job: &JobFile) -> JobSession {
    let mut session = JobSession::new();
    session.job_path = Some(path.to_path_buf());
    session.original_json = Some(value);
    session.units = job.units().to_string();
    session.source_path = Some(campanion_cli::source_path(job));
    session.source_hash = Some(job.source().hash().to_string());
    session.tools = job.tools().iter().map(tool_draft_from).collect();
    session.operations = job
        .operations()
        .iter()
        .enumerate()
        .map(|(index, op)| {
            let original =
                session.original_json.as_ref().and_then(|value| value.get("operations")).and_then(Value::as_array).and_then(|ops| ops.get(index));
            op_draft_from(op, original)
        })
        .collect();
    if let Some(placement) = job.placement() {
        session.placement_x = format_number(placement.x());
        session.placement_y = format_number(placement.y());
        if placement.rotate_deg() != 0.0 {
            session.placement_rotate = format_number(placement.rotate_deg());
        }
    }
    if let Some(machine) = job.machine() {
        session.machine_profile = machine.profile.clone().unwrap_or_default();
        session.machine_path = machine.path.as_ref().map(|path| path.display().to_string()).unwrap_or_default();
        session.machine_head = machine.head.clone().unwrap_or_default();
    }
    if let Some(first) = session.operations.first() {
        session.selected = Some(TreeSelection::Operation(0));
        let _ = first;
    }
    session
}

fn tool_draft_from(tool: &campanion_cli::JobTool) -> ToolDraft {
    let name = tool.ir().name().unwrap_or("").to_string();
    if let Some(wire) = tool.wire_diameter() {
        return ToolDraft { id: tool.id(), name, kind: ToolKind::Wire, size: format_number(wire) };
    }
    match tool.path_tool() {
        PathTool::Kerf { width } => ToolDraft { id: tool.id(), name, kind: ToolKind::Kerf, size: format_number(width) },
        PathTool::Diameter { diameter, .. } => ToolDraft { id: tool.id(), name, kind: ToolKind::Diameter, size: format_number(diameter) },
    }
}

fn op_draft_from(op: &campanion_cli::Operation, original: Option<&Value>) -> OpDraft {
    let (kind, side) = match op.kind() {
        campanion_cli::OperationKind::Profile { side } => ("profile", profile_side_label(side)),
        campanion_cli::OperationKind::Pocket => ("pocket", ""),
        campanion_cli::OperationKind::Drill => ("drill", ""),
        campanion_cli::OperationKind::Engrave => ("engrave", ""),
        campanion_cli::OperationKind::Face => ("face", ""),
        campanion_cli::OperationKind::Chamfer { .. } => ("chamfer", ""),
        campanion_cli::OperationKind::Surface => ("surface", ""),
        campanion_cli::OperationKind::Rough => ("rough", ""),
        campanion_cli::OperationKind::Waterline => ("waterline", ""),
        campanion_cli::OperationKind::Wrap => ("wrap", ""),
        campanion_cli::OperationKind::RotaryFinish => ("rotary_finish", ""),
        campanion_cli::OperationKind::LaserRaster => ("laser_raster", ""),
    };
    let feeds = op.feeds();
    let lead_in = lead_fields(op.lead_in());
    let lead_out = lead_fields(op.lead_out());
    let power = op.params().power_value().map(format_number).unwrap_or_default();
    let pierce = if op.params().pierce_delay_s() > 0.0 { format_number(op.params().pierce_delay_s()) } else { String::new() };
    let post = if op.params().post_delay_s() > 0.0 { format_number(op.params().post_delay_s()) } else { String::new() };
    OpDraft {
        name: op.name().to_string(),
        kind: kind.to_string(),
        layers: op.layers().to_vec(),
        side: side.to_string(),
        tool_id: op.tool_id(),
        depth: format_number(op.depth()),
        step_down: format_number(op.step_down()),
        cut_feed: format_number(feeds.cut()),
        plunge_feed: feeds.plunge().map(format_number).unwrap_or_default(),
        power,
        lead_in: lead_in.0,
        lead_in_size: lead_in.1,
        lead_out: lead_out.0,
        lead_out_size: lead_out.1,
        direction: direction_draft(op, original).to_string(),
        compensation: compensation_label(op.compensation()).to_string(),
        pierce_delay: pierce,
        post_delay: post,
    }
}

fn profile_side_label(side: campanion_cli::JobProfileSide) -> &'static str {
    match side {
        campanion_cli::JobProfileSide::Inside => "inside",
        campanion_cli::JobProfileSide::Outside => "outside",
        campanion_cli::JobProfileSide::On => "on",
        campanion_cli::JobProfileSide::Auto => "auto",
    }
}

fn direction_label(direction: CutDirection) -> &'static str {
    match direction {
        CutDirection::Climb => "climb",
        CutDirection::Conventional => "conventional",
    }
}

fn direction_draft(op: &campanion_cli::Operation, original: Option<&Value>) -> &'static str {
    if original.and_then(|value| value.get("direction")).and_then(Value::as_str) == Some("auto") {
        return "auto";
    }
    direction_label(op.cut_direction())
}

fn compensation_label(compensation: Compensation) -> &'static str {
    match compensation {
        Compensation::Computed => "computed",
        Compensation::Controller(ControllerSide::Left) => "controller_left",
        Compensation::Controller(ControllerSide::Right) => "controller_right",
    }
}

fn lead_fields(lead: Option<Lead>) -> (String, String) {
    match lead {
        None => (String::new(), String::new()),
        Some(Lead::None) => ("none".to_string(), String::new()),
        Some(Lead::Line { length }) => ("line".to_string(), format_number(length)),
        Some(Lead::Arc { radius }) => ("arc".to_string(), format_number(radius)),
        Some(Lead::Perpendicular { length }) => ("perpendicular".to_string(), format_number(length)),
        Some(Lead::FromPoint { .. }) => (String::new(), String::new()),
    }
}

/// Format a finite number without trailing zeros. Non-finite values become an empty string.
pub fn format_number(value: f64) -> String {
    if !value.is_finite() {
        return String::new();
    }
    let text = format!("{value:.6}");
    let trimmed = text.trim_end_matches('0').trim_end_matches('.');
    if trimmed.is_empty() || trimmed == "-" { "0".to_string() } else { trimmed.to_string() }
}

/// Join an export directory, file name and extension (`.ngc` or `.gcode`) into a path.
pub fn export_destination(dir: &str, name: &str, format: &str) -> Result<PathBuf, String> {
    let format = normalize_extension(format)?;
    let dir = dir.trim();
    let name = name.trim();
    if dir.is_empty() && name.is_empty() {
        return Err("enter a file name and a folder to export to".to_string());
    }
    if !dir.is_empty() {
        let path = Path::new(dir);
        if looks_like_gcode_file(path) {
            return Ok(path.to_path_buf());
        }
        if name.is_empty() {
            return Err("enter a file name to export".to_string());
        }
        return Ok(path.join(format!("{name}{format}")));
    }
    if name.is_empty() {
        return Err("enter a file name to export".to_string());
    }
    let path = Path::new(name);
    if looks_like_gcode_file(path) { Ok(path.to_path_buf()) } else { Ok(PathBuf::from(format!("{name}{format}"))) }
}

fn normalize_extension(format: &str) -> Result<String, String> {
    let trimmed = format.trim();
    if trimmed.is_empty() {
        return Err("enter an export format, such as .ngc or .gcode".to_string());
    }
    let with_dot = if trimmed.starts_with('.') { trimmed.to_string() } else { format!(".{trimmed}") };
    if with_dot.eq_ignore_ascii_case(".ngc") || with_dot.eq_ignore_ascii_case(".gcode") || with_dot.eq_ignore_ascii_case(".nc") {
        Ok(with_dot)
    } else {
        Err(format!("unsupported export format {trimmed}; use .ngc, .gcode or .nc"))
    }
}

fn looks_like_gcode_file(path: &Path) -> bool {
    path.extension()
        .and_then(|ext| ext.to_str())
        .is_some_and(|ext| ext.eq_ignore_ascii_case("ngc") || ext.eq_ignore_ascii_case("gcode") || ext.eq_ignore_ascii_case("nc"))
}

/// Build job-file JSON from the session drafts, preserving unused original fields when present.
pub fn job_json_of(session: &JobSession) -> Result<String, String> {
    let value = job_value_of(session)?;
    serde_json::to_string_pretty(&value).map_err(|err| err.to_string())
}

fn job_value_of(session: &JobSession) -> Result<Value, String> {
    let mut root = match &session.original_json {
        Some(Value::Object(map)) => Value::Object(map.clone()),
        _ => serde_json::json!({ "format": FORMAT_TAG, "units": session.units }),
    };
    let Some(obj) = root.as_object_mut() else {
        return Err("job JSON is not an object".to_string());
    };
    obj.insert("format".into(), Value::String(FORMAT_TAG.to_string()));
    obj.insert("units".into(), Value::String(session.units.clone()));
    obj.insert("source".into(), source_object(session)?);
    obj.insert("tools".into(), Value::Array(session.tools.iter().map(|tool| tool_value(tool, &session.units)).collect::<Result<Vec<_>, _>>()?));
    let mut operations = Vec::new();
    for (index, op) in session.operations.iter().enumerate() {
        let original =
            session.original_json.as_ref().and_then(|value| value.get("operations")).and_then(Value::as_array).and_then(|ops| ops.get(index));
        operations.push(operation_value(op, original, &session.units)?);
    }
    obj.insert("operations".into(), Value::Array(operations));
    match placement_value(session) {
        Ok(Some(value)) => {
            obj.insert("placement".into(), value);
        }
        Ok(None) => {
            obj.remove("placement");
        }
        Err(err) => return Err(err),
    }
    match machine_value(session) {
        Some(value) => {
            obj.insert("machine".into(), value);
        }
        None => {
            obj.remove("machine");
        }
    }
    Ok(root)
}

fn machine_value(session: &JobSession) -> Option<Value> {
    if !session.machine_profile.is_empty() {
        let mut map = serde_json::Map::new();
        map.insert("profile".into(), Value::String(session.machine_profile.clone()));
        if !session.machine_head.is_empty() {
            map.insert("head".into(), Value::String(session.machine_head.clone()));
        }
        return Some(Value::Object(map));
    }
    if !session.machine_path.is_empty() {
        let mut map = serde_json::Map::new();
        map.insert("path".into(), Value::String(session.machine_path.clone()));
        if !session.machine_head.is_empty() {
            map.insert("head".into(), Value::String(session.machine_head.clone()));
        }
        return Some(Value::Object(map));
    }
    None
}

fn placement_value(session: &JobSession) -> Result<Option<Value>, String> {
    let (x, y, rotate) = parse_placement_mm(session)?;
    if x == 0.0 && y == 0.0 && rotate == 0.0 {
        return Ok(None);
    }
    Ok(Some(serde_json::json!({
        "x": to_file_units(x, &session.units),
        "y": to_file_units(y, &session.units),
        "rotate_deg": rotate,
    })))
}

/// Parse the typed placement fields, millimetres and degrees. Empty fields are zero.
pub fn parse_placement_mm(session: &JobSession) -> Result<(f64, f64, f64), String> {
    Ok((
        parse_optional_number(&session.placement_x, "placement X")?,
        parse_optional_number(&session.placement_y, "placement Y")?,
        parse_optional_number(&session.placement_rotate, "placement rotate_deg")?,
    ))
}

fn parse_optional_number(text: &str, what: &str) -> Result<f64, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Ok(0.0);
    }
    let value: f64 = trimmed.parse().map_err(|_| format!("{what} is not a number"))?;
    if !value.is_finite() {
        return Err(format!("{what} must be finite"));
    }
    Ok(value)
}

/// The source drawing after the session placement is applied.
pub fn placed_drawing(session: &JobSession) -> Result<Option<Drawing>, String> {
    let Some(drawing) = session.source_drawing.as_ref() else {
        return Ok(None);
    };
    let (x, y, rotate) = parse_placement_mm(session)?;
    if x == 0.0 && y == 0.0 && rotate == 0.0 {
        return Ok(Some(drawing.clone()));
    }
    let tolerance = campanion_geom::Tolerance::default();
    place_drawing(drawing, x, y, rotate, &tolerance).map(Some).map_err(|err| err.to_string())
}

fn source_object(session: &JobSession) -> Result<Value, String> {
    let Some(source) = &session.source_path else {
        return Err("import a part or open a job before saving".to_string());
    };
    let hash = session.source_hash.clone().ok_or_else(|| "source hash is missing; import the part again".to_string())?;
    let stored = stored_source_path(session.job_path.as_deref(), source);
    let mut map = Map::new();
    map.insert("path".into(), Value::String(stored));
    map.insert("hash".into(), Value::String(hash));
    if let Some(Value::Object(original)) = session.original_json.as_ref().and_then(|value| value.get("source"))
        && let Some(face) = original.get("face")
    {
        map.insert("face".into(), face.clone());
    }
    Ok(Value::Object(map))
}

/// Path written into the job file.
///
/// When the job has been saved, this prefers a path relative to the job directory
/// (including `..` when the part sits in a nearby folder). If no relative path can
/// be formed, or the job has not been saved yet, the source path is stored as given,
/// which for a GUI import is typically absolute.
pub fn stored_source_path(job_path: Option<&Path>, source: &Path) -> String {
    if let Some(job) = job_path
        && let Some(job_dir) = job.parent()
        && let Some(relative) = relative_to(source, job_dir)
    {
        return relative.to_string_lossy().into_owned();
    }
    source.to_string_lossy().into_owned()
}

/// Lexical relative path from `base_dir` to `source`.
///
/// Both paths must be absolute and share a root (or prefix on Windows). The result
/// names a file; a path that would be empty is rejected.
fn relative_to(source: &Path, base_dir: &Path) -> Option<PathBuf> {
    if !source.is_absolute() || !base_dir.is_absolute() {
        return None;
    }
    let source_comps: Vec<_> = source.components().collect();
    let base_comps: Vec<_> = base_dir.components().collect();
    if source_comps.first() != base_comps.first() {
        return None;
    }
    let common = source_comps.iter().zip(&base_comps).take_while(|(a, b)| a == b).count();
    if common == 0 {
        return None;
    }
    let mut relative = PathBuf::new();
    for _ in common..base_comps.len() {
        relative.push("..");
    }
    for component in &source_comps[common..] {
        relative.push(component.as_os_str());
    }
    if relative.as_os_str().is_empty() || source.file_name().is_none() {
        return None;
    }
    Some(relative)
}

fn tool_value(tool: &ToolDraft, _units: &str) -> Result<Value, String> {
    let mut map = Map::new();
    map.insert("id".into(), Value::from(tool.id));
    if !tool.name.is_empty() {
        map.insert("name".into(), Value::String(tool.name.clone()));
    }
    let size = parse_required_number(&tool.size, &format!("tool {} size", tool.id))?;
    let key = match tool.kind {
        ToolKind::Kerf => "kerf",
        ToolKind::Diameter => "diameter",
        ToolKind::Wire => "wire",
    };
    map.insert(key.into(), json_number(to_file_units(size, _units)));
    Ok(Value::Object(map))
}

fn operation_value(op: &OpDraft, original: Option<&Value>, units: &str) -> Result<Value, String> {
    let mut map = match original.and_then(Value::as_object) {
        Some(existing) => existing.clone(),
        None => Map::new(),
    };
    if op.name.trim().is_empty() {
        return Err("each operation needs a name".to_string());
    }
    map.insert("name".into(), Value::String(op.name.clone()));
    map.insert("type".into(), Value::String(op.kind.clone()));
    if op.kind == "profile" {
        if op.layers.is_empty() {
            return Err(format!("operation `{}` needs at least one layer", op.name));
        }
        map.insert("layers".into(), Value::Array(op.layers.iter().map(|layer| Value::String(layer.clone())).collect()));
        if !op.side.is_empty() {
            map.insert("side".into(), Value::String(op.side.clone()));
        }
    }
    map.insert("tool_id".into(), Value::from(op.tool_id));
    let depth = parse_required_number(&op.depth, &format!("operation `{}` depth", op.name))?;
    let step_down = parse_required_number(&op.step_down, &format!("operation `{}` step down", op.name))?;
    map.insert("depth".into(), json_number(to_file_units(depth, units)));
    map.insert("step_down".into(), json_number(to_file_units(step_down, units)));
    let cut = parse_required_number(&op.cut_feed, &format!("operation `{}` cut feed", op.name))?;
    let mut feeds = Map::new();
    feeds.insert("cut".into(), json_number(to_file_units(cut, units)));
    if !op.plunge_feed.trim().is_empty() {
        let plunge = parse_required_number(&op.plunge_feed, &format!("operation `{}` plunge feed", op.name))?;
        feeds.insert("plunge".into(), json_number(to_file_units(plunge, units)));
    }
    map.insert("feeds".into(), Value::Object(feeds));
    set_optional_fraction(&mut map, "power", &op.power, &format!("operation `{}` power", op.name))?;
    set_optional_lead(&mut map, "lead_in", &op.lead_in, &op.lead_in_size, units, &op.name)?;
    set_optional_lead(&mut map, "lead_out", &op.lead_out, &op.lead_out_size, units, &op.name)?;
    if !op.direction.trim().is_empty() {
        map.insert("direction".into(), Value::String(op.direction.clone()));
    }
    if !op.compensation.trim().is_empty() {
        map.insert("compensation".into(), Value::String(op.compensation.clone()));
    }
    let mut timing = Map::new();
    if !op.pierce_delay.trim().is_empty() {
        let value = parse_required_number(&op.pierce_delay, &format!("operation `{}` pierce delay", op.name))?;
        timing.insert("pierce_delay".into(), json_number(value));
    }
    if !op.post_delay.trim().is_empty() {
        let value = parse_required_number(&op.post_delay, &format!("operation `{}` post delay", op.name))?;
        timing.insert("post_delay".into(), json_number(value));
    }
    if timing.is_empty() {
        map.remove("timing");
    } else {
        map.insert("timing".into(), Value::Object(timing));
    }
    Ok(Value::Object(map))
}

fn set_optional_fraction(map: &mut Map<String, Value>, key: &str, text: &str, what: &str) -> Result<(), String> {
    if text.trim().is_empty() {
        map.remove(key);
        return Ok(());
    }
    let value = parse_required_number(text, what)?;
    map.insert(key.into(), json_number(value));
    Ok(())
}

fn set_optional_lead(map: &mut Map<String, Value>, key: &str, kind: &str, size: &str, units: &str, name: &str) -> Result<(), String> {
    let kind = kind.trim();
    if kind.is_empty() {
        map.remove(key);
        return Ok(());
    }
    if kind == "none" {
        map.insert(key.into(), serde_json::json!({ "type": "none" }));
        return Ok(());
    }
    let size = parse_required_number(size, &format!("operation `{name}` {key} size"))?;
    let field = if kind == "arc" { "radius" } else { "length" };
    map.insert(key.into(), serde_json::json!({ "type": kind, field: to_file_units(size, units) }));
    Ok(())
}

fn parse_required_number(text: &str, what: &str) -> Result<f64, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err(format!("{what} is empty; enter a value. CAMpanion does not fill in feeds, power or sizes"));
    }
    let value: f64 = trimmed.parse().map_err(|_| format!("{what} is not a number"))?;
    if !value.is_finite() {
        return Err(format!("{what} is not finite"));
    }
    Ok(value)
}

fn to_file_units(mm: f64, units: &str) -> f64 {
    if units == "inch" { mm / 25.4 } else { mm }
}

fn json_number(value: f64) -> Value {
    serde_json::Number::from_f64(value).map(Value::Number).unwrap_or(Value::Null)
}

/// Parse the session into a [`JobFile`].
pub fn job_file_of(session: &JobSession) -> Result<JobFile, String> {
    let json = job_json_of(session)?;
    let path = session.job_path.clone().unwrap_or_else(|| PathBuf::from("unsaved.job.json"));
    parse_job(&json, &path).map_err(|err| err.to_string())
}

/// Planned program and the messages to show.
#[derive(Debug, Clone)]
pub struct PlannedSession {
    pub document: Document,
    pub scene: Scene,
    pub messages: Vec<SessionMessage>,
}

/// Generate toolpaths for the current drafts.
pub fn generate_toolpaths(session: &JobSession) -> Result<PlannedSession, String> {
    let job = job_file_of(session)?;
    let planned = plan_job(&job, PlanOptions::default()).map_err(|err| err.to_string())?;
    let mut messages = Vec::new();
    for text in planned.warnings {
        messages.push(warning(text));
    }
    for text in planned.notes {
        messages.push(note(text));
    }
    if !planned.blocking.is_empty() {
        for text in planned.blocking {
            messages.push(error_msg(text));
        }
        return Err(messages_as_error(&messages));
    }
    let scene = campanion_view::build_scene(&planned.document, DEFAULT_ARC_TOL_MM);
    Ok(PlannedSession { document: planned.document, scene, messages })
}

fn messages_as_error(messages: &[SessionMessage]) -> String {
    messages.iter().filter(|message| message.kind == MessageKind::Error).map(|message| message.text.as_str()).collect::<Vec<_>>().join("\n")
}

/// Post the planned program. Returns G-code text and any post warnings (including S0).
pub fn post_session(session: &JobSession, document: &Document) -> Result<(String, Vec<SessionMessage>), String> {
    if session.post_id.trim().is_empty() {
        return Err("choose a post before previewing or exporting G-code".to_string());
    }
    let job = job_file_of(session)?;
    let posted = post_job(&job, document, session.post_id.trim()).map_err(|err| err.to_string())?;
    let messages = posted.warnings.into_iter().map(warning).collect();
    Ok((posted.text, messages))
}

/// Write G-code to the export destination.
pub fn write_gcode(path: &Path, text: &str) -> Result<(), String> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
    }
    std::fs::write(path, text).map_err(|err| format!("{}: {err}", path.display()))
}

/// Write a job file.
pub fn write_job(path: &Path, session: &JobSession) -> Result<(), String> {
    let json = job_json_of(session)?;
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
    }
    std::fs::write(path, json).map_err(|err| format!("{}: {err}", path.display()))
}

/// Suggested post for a GUI process label such as `Laser`.
pub fn suggested_post_for_process(label: &str) -> Option<&'static str> {
    let key = label.to_ascii_lowercase().replace(' ', "-");
    suggested_post(&key).or_else(|| suggested_post(label))
}

/// Built-in posts for the Export chooser.
pub fn list_posts() -> Vec<(String, String, String)> {
    builtin_posts().into_iter().map(|post| (post.id, post.name, post.description)).collect()
}

/// Preview scene for imported 2D geometry, before toolpaths exist.
pub fn scene_from_drawing(drawing: &Drawing, arc_tol: f64) -> Scene {
    let tol = if arc_tol.is_finite() && arc_tol > 0.0 { arc_tol } else { DEFAULT_ARC_TOL_MM };
    let mut scene = Scene {
        strokes: Vec::new(),
        pierces: Vec::new(),
        labels: Vec::new(),
        bounds: None,
        z_range: Some((0.0, 0.0)),
        total_length: 0.0,
        names: Vec::new(),
        rotary: None,
        lathe: None,
        timeline: Vec::new(),
    };
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    for (index, imported) in drawing.contours().iter().enumerate() {
        for segment in imported.contour().segments() {
            let points = segment_points(*segment, tol);
            if points.len() < 2 {
                continue;
            }
            let mut length = 0.0;
            for pair in points.windows(2) {
                let dx = pair[1][0] - pair[0][0];
                let dy = pair[1][1] - pair[0][1];
                length += (dx * dx + dy * dy).sqrt();
            }
            for point in &points {
                min[0] = min[0].min(point[0]);
                min[1] = min[1].min(point[1]);
                max[0] = max[0].max(point[0]);
                max[1] = max[1].max(point[1]);
            }
            let zs = vec![0.0; points.len()];
            scene.total_length += length;
            scene.strokes.push(Stroke { kind: StrokeKind::Cut, toolpath: index, points, zs, z_start: 0.0, z_end: 0.0, angles: Vec::new(), length });
        }
        scene.names.push(imported.layer().to_string());
    }
    if min[0].is_finite() {
        scene.bounds = Some((min, max));
    }
    scene
}

fn segment_points(segment: Segment, tol: f64) -> Vec<[f64; 2]> {
    match segment {
        Segment::Line(line) => {
            let start = line.start();
            let end = line.end();
            vec![[start.x(), start.y()], [end.x(), end.y()]]
        }
        Segment::Arc(arc) => {
            let start = arc.start_point();
            let end = arc.end_point();
            let center = arc.center();
            let ccw = arc.sweep() > 0.0;
            tessellate_arc([start.x(), start.y()], [end.x(), end.y()], [center.x(), center.y()], ccw, tol)
        }
    }
}

fn warning(text: impl Into<String>) -> SessionMessage {
    SessionMessage { kind: MessageKind::Warning, text: text.into() }
}

fn error_msg(text: impl Into<String>) -> SessionMessage {
    SessionMessage { kind: MessageKind::Error, text: text.into() }
}

fn note(text: impl Into<String>) -> SessionMessage {
    SessionMessage { kind: MessageKind::Note, text: text.into() }
}

/// Next unused tool id, starting at 1.
pub fn next_tool_id(tools: &[ToolDraft]) -> u32 {
    tools.iter().map(|tool| tool.id).max().unwrap_or(0).saturating_add(1).max(1)
}

/// Unique operation name such as `profile`, `profile-2`.
pub fn next_operation_name(operations: &[OpDraft], stem: &str) -> String {
    if !operations.iter().any(|op| op.name == stem) {
        return stem.to_string();
    }
    for index in 2..=10_000 {
        let name = format!("{stem}-{index}");
        if !operations.iter().any(|op| op.name == name) {
            return name;
        }
    }
    format!("{stem}-new")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir).join(name);
        fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    fn fnv1a64(bytes: &[u8]) -> String {
        campanion_cli::hash_bytes(bytes)
    }

    const PART_SVG: &str = concat!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10">"#,
        r#"<rect x="1" y="1" width="12.8" height="6"/>"#,
        "</svg>",
    );

    #[test]
    fn preview_notice_mentions_cli() {
        assert!(GUI_PREVIEW_NOTICE.contains("command-line driven"));
        assert!(GUI_PREVIEW_NOTICE.contains("GUI is still in development"));
        assert!(GUI_PREVIEW_NOTICE_SHORT.contains("command-line"));
    }

    #[test]
    fn new_profile_leaves_material_values_empty() {
        let op = OpDraft::new_profile("outline", 1);
        assert!(op.depth.is_empty());
        assert!(op.step_down.is_empty());
        assert!(op.cut_feed.is_empty());
        assert!(op.power.is_empty());
        assert!(op.lead_in.is_empty());
    }

    #[test]
    fn export_destination_joins_dir_name_and_extension() {
        let path = export_destination("/tmp/out", "plate", ".ngc").expect("path");
        assert_eq!(path, PathBuf::from("/tmp/out/plate.ngc"));
        let path = export_destination("/tmp/out/part.gcode", "", ".ngc").expect("full");
        assert_eq!(path, PathBuf::from("/tmp/out/part.gcode"));
        let err = export_destination("", "", ".ngc").expect_err("empty");
        assert!(err.contains("file name"));
        let err = export_destination("/tmp", "part", ".pm7m").expect_err("format");
        assert!(err.contains("unsupported export format"));
    }

    #[test]
    fn stored_source_path_uses_file_name_in_same_directory() {
        let job = Path::new("/work/job.json");
        let source = Path::new("/work/part.svg");
        assert_eq!(stored_source_path(Some(job), source), "part.svg");
        assert_eq!(stored_source_path(Some(job), Path::new("/other/part.svg")), "../other/part.svg");
        assert_eq!(stored_source_path(Some(job), Path::new("/work/nested/part.svg")), "nested/part.svg");
        assert_eq!(stored_source_path(None, Path::new("/other/part.svg")), "/other/part.svg");
    }

    fn laser_session(imported: &ImportedPart) -> JobSession {
        let mut session = session_from_import(imported);
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = OpDraft::new_profile("outline", 1);
        op.layers = vec![imported.layers[0].name.clone()];
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations.push(op);
        session.post_id = "grbl_laser".to_string();
        session
    }

    #[test]
    fn generate_toolpaths_from_unsaved_job_with_absolute_source() {
        let downloads = temp_dir("campanion-gui-unsaved-abs-source");
        fs::write(downloads.join("part.svg"), PART_SVG).expect("svg");
        let imported = import_part_file(&downloads.join("part.svg")).expect("import");
        assert!(imported.path.is_absolute(), "{:?}", imported.path);
        let session = laser_session(&imported);
        assert!(session.job_path.is_none());
        let stored = stored_source_path(session.job_path.as_deref(), &imported.path);
        assert!(Path::new(&stored).is_absolute(), "{stored}");
        generate_toolpaths(&session).expect("plan unsaved job from an absolute source");
    }

    #[test]
    fn generate_toolpaths_when_job_and_part_are_in_different_folders() {
        let parts = temp_dir("campanion-gui-parts-dir");
        let jobs = temp_dir("campanion-gui-jobs-dir");
        fs::write(parts.join("part.svg"), PART_SVG).expect("svg");
        let imported = import_part_file(&parts.join("part.svg")).expect("import");
        let mut session = laser_session(&imported);
        session.job_path = Some(jobs.join("job.json"));
        let stored = stored_source_path(session.job_path.as_deref(), &imported.path);
        assert!(!Path::new(&stored).is_absolute(), "{stored}");
        assert!(stored.ends_with("part.svg"), "{stored}");
        generate_toolpaths(&session).expect("plan job saved beside a part in another folder");
        write_job(&jobs.join("job.json"), &session).expect("save");
        let saved = fs::read_to_string(jobs.join("job.json")).expect("read");
        assert!(saved.contains(&stored), "{saved}");
        let (reopened, _) = open_job_file(&jobs.join("job.json")).expect("reopen");
        generate_toolpaths(&reopened).expect("plan reopened job");
    }

    #[test]
    fn suggested_laser_post() {
        assert_eq!(suggested_post_for_process("Laser"), Some("grbl_laser"));
        assert_eq!(suggested_post_for_process("Milling"), None);
    }

    #[test]
    fn import_svg_and_plan_laser_with_power() {
        let dir = temp_dir("campanion-gui-session-laser");
        fs::write(dir.join("part.svg"), PART_SVG).expect("svg");
        let imported = import_part_file(&dir.join("part.svg")).expect("import");
        assert!(!imported.layers.is_empty());
        let mut session = session_from_import(&imported);
        session.job_path = Some(dir.join("job.json"));
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = OpDraft::new_profile("outline", 1);
        let layer = imported.layers[0].name.clone();
        op.layers = vec![layer];
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        op.power = "0.2".to_string();
        session.operations.push(op);
        session.post_id = "grbl_laser".to_string();
        let planned = generate_toolpaths(&session).expect("plan");
        assert!(planned.scene.strokes.iter().any(|stroke| stroke.kind == StrokeKind::Cut));
        assert!(planned.messages.iter().all(|message| !message.text.contains("power was not set")));
        let (gcode, messages) = post_session(&session, &planned.document).expect("post");
        assert!(gcode.contains("M4 S200"), "{gcode}");
        assert!(messages.iter().all(|message| !message.text.contains("power was not set")));
        let json = job_json_of(&session).expect("json");
        assert!(json.contains("\"kerf\""));
        assert!(json.contains("\"power\""));
        write_job(&dir.join("job.json"), &session).expect("save");
        let (reopened, _) = open_job_file(&dir.join("job.json")).expect("reopen");
        assert_eq!(reopened.operations[0].power, "0.2");
        assert_eq!(reopened.operations[0].cut_feed, "300");
    }

    #[test]
    fn omitted_power_surfaces_s0_warning() {
        let dir = temp_dir("campanion-gui-session-s0");
        fs::write(dir.join("part.svg"), PART_SVG).expect("svg");
        let imported = import_part_file(&dir.join("part.svg")).expect("import");
        let mut session = session_from_import(&imported);
        session.job_path = Some(dir.join("job.json"));
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = OpDraft::new_profile("outline", 1);
        op.layers = vec![imported.layers[0].name.clone()];
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations.push(op);
        session.post_id = "grbl_laser".to_string();
        let planned = generate_toolpaths(&session).expect("plan");
        let (gcode, messages) = post_session(&session, &planned.document).expect("post");
        assert!(gcode.contains("M4 S0"), "{gcode}");
        assert!(!gcode.contains("S1000"), "{gcode}");
        assert!(messages.iter().any(|message| message.kind == MessageKind::Warning && message.text.contains("power was not set")));
    }

    #[test]
    fn narrow_slot_refusal_is_an_error() {
        let dir = temp_dir("campanion-gui-session-slot");
        let entities = "0\nLINE\n8\nPLATE\n10\n0\n20\n0\n11\n10\n21\n0\n0\nLINE\n8\nPLATE\n10\n10\n20\n0\n11\n10\n21\n1\n0\nLINE\n8\nPLATE\n10\n10\n20\n1\n11\n0\n21\n1\n0\nLINE\n8\nPLATE\n10\n0\n20\n1\n11\n0\n21\n0\n";
        let dxf = format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n");
        fs::write(dir.join("plate.dxf"), &dxf).expect("dxf");
        let hash = fnv1a64(dxf.as_bytes());
        let job = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"kerf":2.0}}],"operations":[{{"name":"plate","type":"profile","layers":["PLATE"],"side":"inside","tool_id":1,"depth":6.0,"step_down":6.0,"feeds":{{"cut":1000}},"power":1}}]}}"#
        );
        fs::write(dir.join("job.json"), job).expect("job");
        let (session, _) = open_job_file(&dir.join("job.json")).expect("open");
        let err = generate_toolpaths(&session).expect_err("narrow slot");
        assert!(err.contains("slot is narrower than the kerf"), "{err}");
    }

    #[test]
    fn empty_feed_is_refused_not_defaulted() {
        let mut session = JobSession::new();
        session.source_path = Some(PathBuf::from("part.svg"));
        session.source_hash = Some("0".repeat(16));
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = OpDraft::new_profile("outline", 1);
        op.layers = vec!["0".to_string()];
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        session.operations.push(op);
        let err = job_json_of(&session).expect_err("feed");
        assert!(err.contains("cut feed"));
        assert!(err.contains("does not fill in"));
    }

    #[test]
    fn auto_direction_is_preserved_on_open_and_save() {
        let dir = temp_dir("campanion-gui-session-auto-dir");
        fs::write(dir.join("part.svg"), PART_SVG).expect("svg");
        let hash = fnv1a64(PART_SVG.as_bytes());
        let job = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"kerf":0.1}}],"operations":[{{"name":"outline","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{{"cut":300}},"direction":"auto"}}]}}"#
        );
        let path = dir.join("job.json");
        fs::write(&path, &job).expect("job");
        let (session, _) = open_job_file(&path).expect("open");
        assert_eq!(session.operations[0].direction, "auto");
        let saved = job_json_of(&session).expect("save");
        let value: Value = serde_json::from_str(&saved).expect("json");
        assert_eq!(value["operations"][0]["direction"], "auto");
    }

    #[test]
    fn placement_is_written_and_reloaded() {
        let dir = temp_dir("campanion-gui-session-placement");
        fs::write(dir.join("part.svg"), PART_SVG).expect("svg");
        let imported = import_part_file(&dir.join("part.svg")).expect("import");
        let mut session = session_from_import(&imported);
        session.job_path = Some(dir.join("job.json"));
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = OpDraft::new_profile("outline", 1);
        op.layers = vec![imported.layers[0].name.clone()];
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations.push(op);
        session.placement_x = "10".to_string();
        session.placement_y = "5".to_string();
        session.placement_rotate = "90".to_string();
        let saved = job_json_of(&session).expect("save");
        let value: Value = serde_json::from_str(&saved).expect("json");
        assert_eq!(value["placement"]["x"], 10.0);
        assert_eq!(value["placement"]["y"], 5.0);
        assert_eq!(value["placement"]["rotate_deg"], 90.0);
        fs::write(dir.join("job.json"), saved).expect("write");
        let (reopened, _) = open_job_file(&dir.join("job.json")).expect("reopen");
        assert_eq!(reopened.placement_x, "10");
        assert_eq!(reopened.placement_y, "5");
        assert_eq!(reopened.placement_rotate, "90");
    }
}
