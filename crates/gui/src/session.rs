//! Job session logic used by the GUI: import, edit, plan, post and export paths.
//!
//! This module talks to the existing CLI planning and posting code. It does not invent machine
//! limits, feeds or material defaults. Empty fields stay empty until the user types a value.

use std::path::{Path, PathBuf};

use campanion_cli::{
    FORMAT_TAG, FORMAT_TAG_V2, ImportedPart, JobFile, LayerSummary, PlanCancel, PlanOptions, SourceKind, builtin_posts, import_part, parse_job,
    plan_job, plan_job_cancellable, post_job, suggested_post,
};
use campanion_geom::Segment;
use campanion_import::Drawing;
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
    /// `parts`, `layers`, or `contours`. Empty means layers on the operation.
    pub assign: String,
    /// Part ids for a parts/layers selector, comma-separated.
    pub assign_parts: String,
    /// Contour ids for a contours selector, comma-separated.
    pub assign_contours: String,
    /// Tab count, when entered. Mutually exclusive with spacing in the job file.
    pub tabs_count: String,
    /// Tab spacing, millimetres, when entered.
    pub tabs_spacing: String,
    /// Tab width, millimetres, when entered.
    pub tabs_width: String,
    /// Tab height, millimetres, when entered.
    pub tabs_height: String,
    /// Material preset name last applied to this operation, when any.
    pub material_name: String,
    /// Field names the user overrode after applying a preset (shown in orange).
    pub override_fields: Vec<String>,
    /// Plasma small-hole max diameter, millimetres, when the user entered one.
    pub small_hole_max: String,
    /// User-entered small-hole action (`pierce_only`, `thc_off`, or a recorded intent).
    pub small_hole_action: String,
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
            assign: String::new(),
            assign_parts: String::new(),
            assign_contours: String::new(),
            tabs_count: String::new(),
            tabs_spacing: String::new(),
            tabs_width: String::new(),
            tabs_height: String::new(),
            material_name: String::new(),
            override_fields: Vec::new(),
            small_hole_max: String::new(),
            small_hole_action: String::new(),
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
    /// A part in the Setup parts list.
    Part(usize),
    /// A placed instance (`part`, `instance`).
    Instance(usize, usize),
}

/// One imported source and its linked instances.
#[derive(Debug, Clone)]
pub struct PartDraft {
    pub id: String,
    pub source_path: Option<PathBuf>,
    pub source_hash: Option<String>,
    pub source_kind: Option<SourceKind>,
    pub layers: Vec<LayerSummary>,
    pub drawing: Option<Drawing>,
    pub instances: Vec<InstanceDraft>,
}

/// One placed copy of a part.
#[derive(Debug, Clone)]
pub struct InstanceDraft {
    pub id: String,
    pub x: String,
    pub y: String,
    pub rotate: String,
    pub scale: String,
    pub mirror: bool,
}

impl InstanceDraft {
    /// Identity placement with the given instance id.
    #[must_use]
    pub fn identity(id: impl Into<String>) -> Self {
        Self { id: id.into(), x: String::new(), y: String::new(), rotate: String::new(), scale: String::new(), mirror: false }
    }

    /// Draft fields from a parsed placement. Zero translation, zero rotation and scale `1` stay empty.
    #[must_use]
    pub fn from_placement(id: impl Into<String>, placement: campanion_cli::Placement) -> Self {
        Self {
            id: id.into(),
            x: if placement.x() == 0.0 { String::new() } else { format_number(placement.x()) },
            y: if placement.y() == 0.0 { String::new() } else { format_number(placement.y()) },
            rotate: if placement.rotate_deg() == 0.0 { String::new() } else { format_number(placement.rotate_deg()) },
            scale: if placement.scale() == 1.0 { String::new() } else { format_number(placement.scale()) },
            mirror: placement.mirror(),
        }
    }

    /// Parse this draft into a placement.
    pub fn to_placement(&self) -> Result<campanion_cli::Placement, String> {
        let x = parse_optional_number(&self.x, "placement X")?;
        let y = parse_optional_number(&self.y, "placement Y")?;
        let rotate = parse_optional_number(&self.rotate, "placement rotate_deg")?;
        let scale = if self.scale.trim().is_empty() { 1.0 } else { parse_optional_number(&self.scale, "placement scale")? };
        if scale == 0.0 || !scale.is_finite() {
            return Err("placement scale must be a non-zero finite number".to_string());
        }
        Ok(campanion_cli::Placement::new(x, y, rotate, self.mirror, scale))
    }
}

/// In-memory job the GUI can edit, plan and save.
#[derive(Debug, Clone)]
pub struct JobSession {
    /// Path of a saved job file, when one is open.
    pub job_path: Option<PathBuf>,
    /// Original JSON, used to preserve fields the GUI does not edit.
    pub original_json: Option<Value>,
    /// Parsed job as opened from disk. Used to plan and post v2 / multi-part jobs
    /// until the Setup editor can represent them.
    pub loaded_job: Option<JobFile>,
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
    /// Parts and linked instances. Empty until a part is imported or a job is opened.
    pub parts: Vec<PartDraft>,
    /// Placement scale, when typed. Empty means 1.
    pub placement_scale: String,
    /// Mirror about the Y axis through the drawing bounds centre.
    pub placement_mirror: bool,
    /// `z_zero` value: empty, `stock_top`, or `bed`.
    pub z_zero: String,
    /// Stock thickness, millimetres, when Z-zero is the bed.
    pub stock_thickness: String,
    /// Datum anchor: empty, `min_min`, `max_min`, `min_max`, `max_max`, or `center`.
    pub datum_anchor: String,
    /// Optional datum offset X, millimetres.
    pub datum_offset_x: String,
    /// Optional datum offset Y, millimetres.
    pub datum_offset_y: String,
    /// `start_from` value: empty, `absolute`, `current_position`, or `user_origin`.
    pub start_from: String,
    /// Nine-position `job_origin`, when entered.
    pub job_origin: String,
    /// When true, write `order_rules` on save.
    pub order_rules_set: bool,
    pub order_inner_first: bool,
    pub order_cuts_last: bool,
    /// Sheet width, millimetres, when entered.
    pub sheet_width: String,
    /// Sheet height, millimetres, when entered.
    pub sheet_height: String,
    /// Increment applied by the Setup Rotate command, degrees, when typed.
    pub rotate_by: String,
    /// Job-level clearance height, millimetres, when entered.
    pub clearance: String,
    /// Job-level retract height, millimetres, when entered.
    pub retract: String,
    /// True when the user (or a loaded file) set an explicit `cut_order`.
    pub cut_order_explicit: bool,
    pub dirty: bool,
    pub messages: Vec<SessionMessage>,
}

impl JobSession {
    /// An empty session.
    pub fn new() -> Self {
        Self {
            job_path: None,
            original_json: None,
            loaded_job: None,
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
            parts: Vec::new(),
            placement_scale: String::new(),
            placement_mirror: false,
            z_zero: String::new(),
            stock_thickness: String::new(),
            datum_anchor: String::new(),
            datum_offset_x: String::new(),
            datum_offset_y: String::new(),
            start_from: String::new(),
            job_origin: String::new(),
            order_rules_set: false,
            order_inner_first: false,
            order_cuts_last: false,
            sheet_width: String::new(),
            sheet_height: String::new(),
            rotate_by: String::new(),
            clearance: String::new(),
            retract: String::new(),
            cut_order_explicit: false,
            dirty: false,
            messages: Vec::new(),
        }
    }

    /// Copy the selected instance's placement into the typed fields.
    pub fn sync_placement_from_selection(&mut self) {
        if let Some(instance) = self.selected_instance() {
            let x = instance.x.clone();
            let y = instance.y.clone();
            let rotate = instance.rotate.clone();
            let scale = instance.scale.clone();
            let mirror = instance.mirror;
            self.placement_x = x;
            self.placement_y = y;
            self.placement_rotate = rotate;
            self.placement_scale = scale;
            self.placement_mirror = mirror;
        }
        if let Some(part) = self.selected_part() {
            let source_path = part.source_path.clone();
            let source_hash = part.source_hash.clone();
            let source_kind = part.source_kind;
            let layers = part.layers.clone();
            let drawing = part.drawing.clone();
            self.source_path = source_path;
            self.source_hash = source_hash;
            self.source_kind = source_kind;
            self.layers = layers;
            self.source_drawing = drawing;
        }
    }

    /// Write the typed placement fields back onto the selected instance, when there is one.
    pub fn commit_placement(&mut self) {
        let Some(TreeSelection::Instance(part, _) | TreeSelection::Part(part)) = self.selected else {
            if let Some(part) = self.parts.first_mut().and_then(|part| part.instances.first_mut()) {
                part.x = self.placement_x.clone();
                part.y = self.placement_y.clone();
                part.rotate = self.placement_rotate.clone();
                part.scale = self.placement_scale.clone();
                part.mirror = self.placement_mirror;
            }
            return;
        };
        let inst = match self.selected {
            Some(TreeSelection::Instance(_, inst)) => inst,
            _ => 0,
        };
        if let Some(instance) = self.parts.get_mut(part).and_then(|item| item.instances.get_mut(inst)) {
            instance.x = self.placement_x.clone();
            instance.y = self.placement_y.clone();
            instance.rotate = self.placement_rotate.clone();
            instance.scale = self.placement_scale.clone();
            instance.mirror = self.placement_mirror;
        }
    }

    /// Select a part instance and sync the typed fields.
    pub fn select_instance(&mut self, part: usize, inst: usize) {
        self.selected = Some(TreeSelection::Instance(part, inst));
        self.sync_placement_from_selection();
    }

    /// The selected or first part.
    #[must_use]
    pub fn selected_part(&self) -> Option<&PartDraft> {
        match self.selected {
            Some(TreeSelection::Part(part) | TreeSelection::Instance(part, _)) => self.parts.get(part),
            _ => self.parts.first(),
        }
    }

    /// The selected or first instance.
    #[must_use]
    pub fn selected_instance(&self) -> Option<&InstanceDraft> {
        match self.selected {
            Some(TreeSelection::Instance(part, inst)) => self.parts.get(part).and_then(|item| item.instances.get(inst)),
            Some(TreeSelection::Part(part)) => self.parts.get(part).and_then(|item| item.instances.first()),
            _ => self.parts.first().and_then(|item| item.instances.first()),
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
    let sources: Vec<(usize, PathBuf)> =
        session.parts.iter().enumerate().filter_map(|(index, part)| part.source_path.clone().map(|path| (index, path))).collect();
    let mut first = None;
    for (index, source) in sources {
        match import_part(&source) {
            Ok(imported) => {
                if let Some(part) = session.parts.get_mut(index) {
                    part.layers = imported.layers.clone();
                    part.source_kind = Some(imported.kind);
                    part.drawing = imported.drawing.clone();
                }
                attach_import_messages(&mut session, &imported);
                if index == 0 {
                    first = Some(imported);
                }
            }
            Err(err) => session.messages.push(warning(err.to_string())),
        }
    }
    session.sync_placement_from_selection();
    Ok((session, first))
}

/// Start a new job from an imported part. No tools or operations are invented.
pub fn session_from_import(part: &ImportedPart) -> JobSession {
    let mut session = JobSession::new();
    add_imported_part(&mut session, part);
    session.dirty = true;
    if part.kind == SourceKind::Step {
        session.messages.push(note(
            "STEP import is recorded. A job must name a face, and planning uses solidcraft-cli, the same way as campanion run.".to_string(),
        ));
    }
    session
}

/// Append an imported part as a new Setup part with one instance. The first import also
/// fills the legacy single-source fields.
pub fn add_imported_part(session: &mut JobSession, part: &ImportedPart) {
    let suggested = part.path.file_stem().and_then(|stem| stem.to_str()).unwrap_or("part");
    let id = crate::setup::next_part_id(session, suggested);
    let draft = PartDraft {
        id,
        source_path: Some(part.path.clone()),
        source_hash: Some(part.hash.clone()),
        source_kind: Some(part.kind),
        layers: part.layers.clone(),
        drawing: part.drawing.clone(),
        instances: vec![InstanceDraft::identity("1")],
    };
    session.parts.push(draft);
    let part_index = session.parts.len() - 1;
    session.select_instance(part_index, 0);
    session.dirty = true;
    attach_import_messages(session, part);
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

/// Part ids in this session, comma-separated, for a parts selector.
#[must_use]
pub fn all_part_ids(session: &JobSession) -> String {
    session.parts.iter().map(|part| part.id.as_str()).collect::<Vec<_>>().join(", ")
}

/// Assign `op` to every part, with no layer filter.
pub fn assign_to_all_parts(op: &mut OpDraft, session: &JobSession) {
    op.assign = "parts".to_string();
    op.assign_parts = all_part_ids(session);
    op.layers.clear();
}

/// Layers from every imported part, merged by name. Falls back to the selected part list.
#[must_use]
pub fn all_part_layers(session: &JobSession) -> Vec<LayerSummary> {
    let mut out: Vec<LayerSummary> = Vec::new();
    for part in &session.parts {
        for layer in &part.layers {
            if let Some(existing) = out.iter_mut().find(|item| item.name == layer.name) {
                existing.contour_count = existing.contour_count.saturating_add(layer.contour_count);
            } else {
                out.push(layer.clone());
            }
        }
    }
    if out.is_empty() {
        out = session.layers.clone();
    }
    out
}

/// Layer name with the part ids that actually have that layer, e.g. `OUTER (sample, curves)`.
#[must_use]
pub fn layer_parts_label(session: &JobSession, layer: &str) -> String {
    let parts: Vec<&str> =
        session.parts.iter().filter(|part| part.layers.iter().any(|item| item.name == layer)).map(|part| part.id.as_str()).collect();
    if parts.is_empty() { layer.to_string() } else { format!("{layer} ({})", parts.join(", ")) }
}

fn part_has_layer(part: &PartDraft, layer: &str) -> bool {
    part.layers.iter().any(|item| item.name == layer)
        || part.drawing.as_ref().is_some_and(|drawing| drawing.contours().iter().any(|item| item.layer() == layer))
}

fn part_has_any_geometry(part: &PartDraft) -> bool {
    !part.layers.is_empty() || part.drawing.as_ref().is_some_and(|drawing| !drawing.contours().is_empty())
}

fn part_matches_op_layers(part: &PartDraft, op: &OpDraft) -> bool {
    if op.layers.is_empty() { part_has_any_geometry(part) } else { op.layers.iter().any(|layer| part_has_layer(part, layer)) }
}

/// Whether `op`'s current assignment would select any imported contour.
#[must_use]
pub fn operation_has_geometry(session: &JobSession, op: &OpDraft) -> bool {
    if session.parts.is_empty() {
        return false;
    }
    let listed = csv_list(&op.assign_parts);
    match op.assign.trim() {
        "parts" => {
            if listed.is_empty() {
                return false;
            }
            session.parts.iter().filter(|part| listed.iter().any(|id| id == &part.id)).any(|part| part_matches_op_layers(part, op))
        }
        "layers" => {
            if op.layers.is_empty() {
                return false;
            }
            session
                .parts
                .iter()
                .filter(|part| listed.is_empty() || listed.iter().any(|id| id == &part.id))
                .any(|part| part_matches_op_layers(part, op))
        }
        "contours" => !csv_list(&op.assign_contours).is_empty(),
        "" => session.parts.iter().any(|part| part_matches_op_layers(part, op)),
        _ => false,
    }
}

/// Inline warning when an operation's assignment matches no geometry.
#[must_use]
pub fn operation_match_warning(session: &JobSession, op: &OpDraft) -> Option<String> {
    if operation_has_geometry(session, op) {
        return None;
    }
    if session.parts.is_empty() {
        return Some("This operation matches no geometry. Import a part, then assign this operation to it.".to_string());
    }
    Some("This operation matches no geometry. Assign it to a part, or pick a layer that exists on an imported part.".to_string())
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
    session.job_path = Some(absolute_path(path));
    session.original_json = Some(value);
    session.loaded_job = Some(job.clone());
    session.units = job.units().to_string();
    session.source_path = Some(absolute_path(&campanion_cli::source_path(job)));
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
        if placement.scale() != 1.0 {
            session.placement_scale = format_number(placement.scale());
        }
        session.placement_mirror = placement.mirror();
    }
    load_setup_fields(&mut session, job);
    fill_operation_extras(&mut session, job);
    load_height_fields(&mut session);
    apply_loaded_cut_order(&mut session, job);
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
    let mut draft = OpDraft {
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
        assign: String::new(),
        assign_parts: String::new(),
        assign_contours: String::new(),
        tabs_count: String::new(),
        tabs_spacing: String::new(),
        tabs_width: String::new(),
        tabs_height: String::new(),
        material_name: String::new(),
        override_fields: Vec::new(),
        small_hole_max: String::new(),
        small_hole_action: String::new(),
    };
    fill_tabs_from_op(&mut draft, op);
    draft
}

fn fill_tabs_from_op(draft: &mut OpDraft, op: &campanion_cli::Operation) {
    let Some(tabs) = op.tabs() else { return };
    draft.tabs_width = format_number(tabs.width());
    if let Some(height) = tabs.tab_height() {
        draft.tabs_height = format_number(height);
    }
    match tabs.layout() {
        campanion_path::TabLayout::Count(count) => draft.tabs_count = count.to_string(),
        campanion_path::TabLayout::Spacing(spacing) => draft.tabs_spacing = format_number(spacing),
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
    let mut working = session.clone();
    working.commit_placement();
    let session = &working;
    let mut root = match &session.original_json {
        Some(Value::Object(map)) => Value::Object(map.clone()),
        _ => serde_json::json!({ "format": FORMAT_TAG, "units": session.units }),
    };
    let Some(obj) = root.as_object_mut() else {
        return Err("job JSON is not an object".to_string());
    };
    obj.insert("units".into(), Value::String(session.units.clone()));
    obj.insert("tools".into(), Value::Array(session.tools.iter().map(|tool| tool_value(tool, &session.units)).collect::<Result<Vec<_>, _>>()?));
    let mut operations = Vec::new();
    for (index, op) in session.operations.iter().enumerate() {
        let original =
            session.original_json.as_ref().and_then(|value| value.get("operations")).and_then(Value::as_array).and_then(|ops| ops.get(index));
        operations.push(operation_value(op, original, &session.units, save_as_v2(session))?);
    }
    obj.insert("operations".into(), Value::Array(operations));
    match machine_value(session) {
        Some(value) => {
            obj.insert("machine".into(), value);
        }
        None => {
            obj.remove("machine");
        }
    }
    if save_as_v2(session) {
        write_v2_fields(session, obj)?;
    } else {
        obj.insert("format".into(), Value::String(FORMAT_TAG.to_string()));
        obj.insert("source".into(), source_object(session)?);
        obj.remove("parts");
        obj.remove("z_zero");
        obj.remove("stock_thickness");
        obj.remove("datum");
        obj.remove("start_from");
        obj.remove("job_origin");
        obj.remove("order_rules");
        obj.remove("sheet");
        match placement_value(session) {
            Ok(Some(value)) => {
                obj.insert("placement".into(), value);
            }
            Ok(None) => {
                obj.remove("placement");
            }
            Err(err) => return Err(err),
        }
    }
    Ok(root)
}

fn save_as_v2(session: &JobSession) -> bool {
    if session.original_json.as_ref().and_then(|value| value.get("format")).and_then(Value::as_str) == Some(FORMAT_TAG_V2) {
        return true;
    }
    if session.parts.len() > 1 || session.parts.iter().any(|part| part.instances.len() > 1) {
        return true;
    }
    if session.parts.iter().flat_map(|part| &part.instances).any(|instance| instance.mirror || !instance.scale.trim().is_empty()) {
        return true;
    }
    hooks_or_sheet_present(session)
}

fn hooks_or_sheet_present(session: &JobSession) -> bool {
    !session.z_zero.trim().is_empty()
        || !session.stock_thickness.trim().is_empty()
        || !session.datum_anchor.trim().is_empty()
        || !session.datum_offset_x.trim().is_empty()
        || !session.datum_offset_y.trim().is_empty()
        || !session.start_from.trim().is_empty()
        || !session.job_origin.trim().is_empty()
        || session.order_rules_set
        || session.cut_order_explicit
        || !session.clearance.trim().is_empty()
        || !session.retract.trim().is_empty()
        || session.operations.iter().any(|op| {
            !op.assign.trim().is_empty()
                || !op.override_fields.is_empty()
                || !op.small_hole_max.trim().is_empty()
                || !op.material_name.trim().is_empty()
        })
        || !session.sheet_width.trim().is_empty()
        || !session.sheet_height.trim().is_empty()
}

fn write_v2_fields(session: &JobSession, obj: &mut Map<String, Value>) -> Result<(), String> {
    obj.insert("format".into(), Value::String(FORMAT_TAG_V2.to_string()));
    obj.remove("source");
    obj.remove("placement");
    let parts = effective_parts(session)?;
    if parts.is_empty() {
        return Err("import a part or open a job before saving".to_string());
    }
    obj.insert("parts".into(), Value::Array(parts));
    write_setup_hooks(session, obj)?;
    write_heights(session, obj)?;
    write_cut_order(session, obj);
    match crate::setup::sheet_bounds(session)? {
        Some([_, _, width, height]) => {
            obj.insert(
                "sheet".into(),
                serde_json::json!({
                    "width": to_file_units(width, &session.units),
                    "height": to_file_units(height, &session.units),
                }),
            );
        }
        None => {
            obj.remove("sheet");
        }
    }
    Ok(())
}

fn effective_parts(session: &JobSession) -> Result<Vec<Value>, String> {
    let drafts = if session.parts.is_empty() {
        if session.source_path.is_none() {
            return Ok(Vec::new());
        }
        vec![PartDraft {
            id: "part".to_string(),
            source_path: session.source_path.clone(),
            source_hash: session.source_hash.clone(),
            source_kind: session.source_kind,
            layers: session.layers.clone(),
            drawing: session.source_drawing.clone(),
            instances: vec![InstanceDraft {
                id: "1".to_string(),
                x: session.placement_x.clone(),
                y: session.placement_y.clone(),
                rotate: session.placement_rotate.clone(),
                scale: session.placement_scale.clone(),
                mirror: session.placement_mirror,
            }],
        }]
    } else {
        session.parts.clone()
    };
    let mut values = Vec::new();
    for part in &drafts {
        values.push(part_value(session, part)?);
    }
    Ok(values)
}

fn part_value(session: &JobSession, part: &PartDraft) -> Result<Value, String> {
    let Some(source) = &part.source_path else {
        return Err(format!("part `{}` has no source path", part.id));
    };
    let hash = part.source_hash.clone().ok_or_else(|| format!("part `{}` is missing a source hash; import the part again", part.id))?;
    let stored = stored_source_path(session.job_path.as_deref(), source);
    let mut source_map = Map::new();
    source_map.insert("path".into(), Value::String(stored));
    source_map.insert("hash".into(), Value::String(hash));
    let mut instances = Vec::new();
    if part.instances.is_empty() {
        return Err(format!("part `{}` needs at least one instance", part.id));
    }
    for instance in &part.instances {
        let placement = instance.to_placement()?;
        let mut item = Map::new();
        item.insert("id".into(), Value::String(instance.id.clone()));
        if !placement.is_identity() {
            item.insert("placement".into(), placement_json(placement, &session.units));
        }
        instances.push(Value::Object(item));
    }
    Ok(serde_json::json!({
        "id": part.id,
        "source": Value::Object(source_map),
        "instances": instances,
    }))
}

fn placement_json(placement: campanion_cli::Placement, units: &str) -> Value {
    let mut map = Map::new();
    if placement.x() != 0.0 {
        map.insert("x".into(), json_number(to_file_units(placement.x(), units)));
    }
    if placement.y() != 0.0 {
        map.insert("y".into(), json_number(to_file_units(placement.y(), units)));
    }
    if placement.rotate_deg() != 0.0 {
        map.insert("rotate_deg".into(), json_number(placement.rotate_deg()));
    }
    if placement.mirror() {
        map.insert("mirror".into(), Value::Bool(true));
    }
    if placement.scale() != 1.0 {
        map.insert("scale".into(), json_number(placement.scale()));
    }
    Value::Object(map)
}

fn write_setup_hooks(session: &JobSession, obj: &mut Map<String, Value>) -> Result<(), String> {
    for key in ["z_zero", "stock_thickness", "datum", "start_from", "job_origin", "order_rules"] {
        obj.remove(key);
    }
    let z_zero = session.z_zero.trim();
    if !z_zero.is_empty() {
        if z_zero != "stock_top" && z_zero != "bed" {
            return Err("z_zero must be stock_top or bed".to_string());
        }
        obj.insert("z_zero".into(), Value::String(z_zero.to_string()));
    }
    let thickness = session.stock_thickness.trim();
    if !thickness.is_empty() {
        let value = parse_optional_number(thickness, "stock thickness")?;
        if value <= 0.0 {
            return Err("stock thickness must be greater than zero".to_string());
        }
        obj.insert("stock_thickness".into(), json_number(to_file_units(value, &session.units)));
    } else if z_zero == "bed" {
        return Err("z_zero bed needs a stock thickness".to_string());
    }
    let anchor = session.datum_anchor.trim();
    if !anchor.is_empty() {
        match anchor {
            "min_min" | "max_min" | "min_max" | "max_max" | "center" => {}
            other => return Err(format!("datum anchor `{other}` is not valid")),
        }
        let mut datum = Map::new();
        datum.insert("anchor".into(), Value::String(anchor.to_string()));
        let ox = session.datum_offset_x.trim();
        let oy = session.datum_offset_y.trim();
        if !ox.is_empty() || !oy.is_empty() {
            let x = parse_optional_number(&session.datum_offset_x, "datum offset X")?;
            let y = parse_optional_number(&session.datum_offset_y, "datum offset Y")?;
            datum.insert(
                "offset".into(),
                Value::Array(vec![json_number(to_file_units(x, &session.units)), json_number(to_file_units(y, &session.units))]),
            );
        }
        obj.insert("datum".into(), Value::Object(datum));
    }
    let start = session.start_from.trim();
    if !start.is_empty() {
        if !matches!(start, "absolute" | "current_position" | "user_origin") {
            return Err("start_from must be absolute, current_position, or user_origin".to_string());
        }
        obj.insert("start_from".into(), Value::String(start.to_string()));
    }
    let origin = session.job_origin.trim();
    if !origin.is_empty() {
        obj.insert("job_origin".into(), Value::String(origin.to_string()));
    }
    if session.order_rules_set {
        obj.insert("order_rules".into(), serde_json::json!({ "inner_first": session.order_inner_first, "cuts_last": session.order_cuts_last }));
    }
    Ok(())
}

fn fill_operation_extras(session: &mut JobSession, job: &JobFile) {
    for op in &mut session.operations {
        let Some(extra) = job.operation_extras(&op.name) else { continue };
        match &extra.selector {
            Some(campanion_cli::OpSelector::Parts { parts }) => {
                op.assign = "parts".to_string();
                op.assign_parts = parts.join(", ");
            }
            Some(campanion_cli::OpSelector::Layers { parts, layers }) => {
                op.assign = "layers".to_string();
                op.assign_parts = parts.join(", ");
                op.layers = layers.clone();
            }
            Some(campanion_cli::OpSelector::Contours { part, ids }) => {
                op.assign = "contours".to_string();
                op.assign_parts = part.clone();
                op.assign_contours = ids.join(", ");
            }
            Some(campanion_cli::OpSelector::Color { .. }) | None => {}
        }
        op.override_fields = extra.overrides.names.clone();
        if let Some(rule) = &extra.small_hole {
            op.small_hole_max = format_number(rule.max_diameter);
            op.small_hole_action = rule.action.clone();
        }
    }
}

fn load_height_fields(session: &mut JobSession) {
    let Some(heights) = session.original_json.as_ref().and_then(|value| value.get("heights")).and_then(Value::as_object) else {
        return;
    };
    if let Some(value) = heights.get("clearance").and_then(Value::as_f64) {
        session.clearance = format_number(from_file_units(value, &session.units));
    }
    if let Some(value) = heights.get("retract").and_then(Value::as_f64) {
        session.retract = format_number(from_file_units(value, &session.units));
    }
}

fn apply_loaded_cut_order(session: &mut JobSession, job: &JobFile) {
    if job.cut_order().is_empty() {
        return;
    }
    session.cut_order_explicit = true;
    session.operations.sort_by_key(|op| job.cut_order().iter().position(|name| name == &op.name).unwrap_or(usize::MAX));
}

fn from_file_units(value: f64, units: &str) -> f64 {
    if units == "inch" { value * 25.4 } else { value }
}

fn load_setup_fields(session: &mut JobSession, job: &JobFile) {
    for part in job.parts() {
        let source_path = resolved_part_path(job, part.source.path());
        let instances = part.instances.iter().map(|instance| InstanceDraft::from_placement(&instance.id, instance.placement)).collect();
        session.parts.push(PartDraft {
            id: part.id.clone(),
            source_path: Some(source_path),
            source_hash: Some(part.source.hash().to_string()),
            source_kind: None,
            layers: Vec::new(),
            drawing: None,
            instances,
        });
    }
    if !session.parts.is_empty() {
        session.select_instance(0, 0);
    }
    if let Some(sheet) = job.sheet() {
        session.sheet_width = format_number(sheet.width());
        session.sheet_height = format_number(sheet.height());
    }
    let hooks = job.setup_hooks();
    if let Some(z_zero) = hooks.z_zero {
        session.z_zero = z_zero.as_str().to_string();
    }
    if let Some(thickness) = hooks.stock_thickness {
        session.stock_thickness = format_number(thickness);
    }
    if let Some(datum) = hooks.datum {
        session.datum_anchor = datum.anchor.as_str().to_string();
        if let Some([x, y]) = datum.offset {
            session.datum_offset_x = format_number(x);
            session.datum_offset_y = format_number(y);
        }
    }
    if let Some(start) = hooks.start_from {
        session.start_from = start.as_str().to_string();
    }
    if let Some(origin) = hooks.job_origin {
        session.job_origin = origin.as_str().to_string();
    }
    if let Some(rules) = hooks.order_rules {
        session.order_rules_set = true;
        session.order_inner_first = rules.inner_first;
        session.order_cuts_last = rules.cuts_last;
    }
}

fn resolved_part_path(job: &JobFile, stored: &str) -> PathBuf {
    let path = Path::new(stored);
    if path.is_absolute() {
        return path.to_path_buf();
    }
    let joined = match job.path().parent() {
        Some(dir) => dir.join(path),
        None => path.to_path_buf(),
    };
    absolute_path(&joined)
}

fn write_heights(session: &JobSession, obj: &mut Map<String, Value>) -> Result<(), String> {
    let clearance = session.clearance.trim();
    let retract = session.retract.trim();
    if clearance.is_empty() && retract.is_empty() {
        obj.remove("heights");
        return Ok(());
    }
    let mut heights = Map::new();
    if !clearance.is_empty() {
        heights.insert("clearance".into(), json_number(to_file_units(parse_required_number(&session.clearance, "clearance")?, &session.units)));
    }
    if !retract.is_empty() {
        heights.insert("retract".into(), json_number(to_file_units(parse_required_number(&session.retract, "retract")?, &session.units)));
    }
    obj.insert("heights".into(), Value::Object(heights));
    Ok(())
}

fn write_cut_order(session: &JobSession, obj: &mut Map<String, Value>) {
    if session.cut_order_explicit && session.operations.len() > 1 {
        let names: Vec<Value> = session.operations.iter().map(|op| Value::String(op.name.clone())).collect();
        obj.insert("cut_order".into(), Value::Array(names));
    } else {
        obj.remove("cut_order");
    }
}

fn write_selector(map: &mut Map<String, Value>, op: &OpDraft, units: &str) -> Result<(), String> {
    match op.assign.trim() {
        "" => {
            map.remove("selector");
        }
        "parts" => {
            let parts = csv_list(&op.assign_parts);
            if parts.is_empty() {
                return Err(format!("operation `{}` assign-by-part needs at least one part id", op.name));
            }
            map.insert("selector".into(), serde_json::json!({ "type": "parts", "parts": parts }));
        }
        "layers" => {
            if op.layers.is_empty() {
                return Err(format!("operation `{}` assign-by-layer needs at least one layer", op.name));
            }
            let mut sel = serde_json::Map::new();
            sel.insert("type".into(), Value::String("layers".into()));
            sel.insert("layers".into(), Value::Array(op.layers.iter().map(|layer| Value::String(layer.clone())).collect()));
            let parts = csv_list(&op.assign_parts);
            if !parts.is_empty() {
                sel.insert("parts".into(), Value::Array(parts.into_iter().map(Value::String).collect()));
            }
            map.insert("selector".into(), Value::Object(sel));
        }
        "contours" => {
            let ids = csv_list(&op.assign_contours);
            if ids.is_empty() {
                return Err(format!("operation `{}` assign-by-contour needs at least one contour id", op.name));
            }
            let part = csv_list(&op.assign_parts).into_iter().next().unwrap_or_else(|| "part".to_string());
            map.insert("selector".into(), serde_json::json!({ "type": "contours", "part": part, "ids": ids }));
        }
        other => return Err(format!("operation `{}` assign mode `{other}` is not valid", op.name)),
    }
    if op.override_fields.is_empty() {
        map.remove("overrides");
    } else {
        map.insert("overrides".into(), Value::Array(op.override_fields.iter().map(|name| Value::String(name.clone())).collect()));
    }
    if op.small_hole_max.trim().is_empty() && op.small_hole_action.trim().is_empty() {
        map.remove("small_hole");
    } else {
        if op.small_hole_max.trim().is_empty() || op.small_hole_action.trim().is_empty() {
            return Err(format!("operation `{}` small-hole rule needs a max diameter and an action, both user-entered", op.name));
        }
        let max = parse_required_number(&op.small_hole_max, &format!("operation `{}` small-hole max diameter", op.name))?;
        if max <= 0.0 {
            return Err(format!("operation `{}` small-hole max diameter must be greater than zero", op.name));
        }
        map.insert(
            "small_hole".into(),
            serde_json::json!({
                "max_diameter": to_file_units(max, units),
                "action": op.small_hole_action.trim(),
            }),
        );
    }
    Ok(())
}

fn csv_list(text: &str) -> Vec<String> {
    text.split(',').map(str::trim).filter(|item| !item.is_empty()).map(str::to_string).collect()
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

pub fn parse_optional_number(text: &str, what: &str) -> Result<f64, String> {
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
        if let Some(part) = session.selected_part().or(session.parts.first())
            && let (Some(drawing), Some(instance)) = (part.drawing.as_ref(), part.instances.first())
        {
            return crate::setup::placed_drawing_of(drawing, instance).map(Some);
        }
        return Ok(None);
    };
    if let Some(instance) = session.selected_instance() {
        return crate::setup::placed_drawing_of(drawing, instance).map(Some);
    }
    let (x, y, rotate) = parse_placement_mm(session)?;
    let scale = if session.placement_scale.trim().is_empty() { 1.0 } else { parse_optional_number(&session.placement_scale, "placement scale")? };
    if x == 0.0 && y == 0.0 && rotate == 0.0 && !session.placement_mirror && scale == 1.0 {
        return Ok(Some(drawing.clone()));
    }
    let tolerance = campanion_geom::Tolerance::default();
    campanion_import::place_with(drawing, x, y, rotate, session.placement_mirror, scale, &tolerance).map(Some).map_err(|err| err.to_string())
}

/// Combined preview of every placed instance.
pub fn placed_scene_parts(session: &JobSession, arc_tol: f64) -> Result<Option<campanion_view::Scene>, String> {
    if session.parts.is_empty() {
        return match placed_drawing(session)? {
            Some(drawing) => Ok(Some(scene_from_drawing(&drawing, arc_tol))),
            None => Ok(None),
        };
    }
    let mut combined: Option<campanion_view::Scene> = None;
    for part in &session.parts {
        let Some(drawing) = part.drawing.as_ref() else { continue };
        for instance in &part.instances {
            let placed = crate::setup::placed_drawing_of(drawing, instance)?;
            let scene = scene_from_drawing(&placed, arc_tol);
            combined = Some(match combined {
                None => scene,
                Some(mut acc) => {
                    acc.strokes.extend(scene.strokes);
                    acc.names.extend(scene.names);
                    acc.total_length += scene.total_length;
                    acc.bounds = merge_bounds(acc.bounds, scene.bounds);
                    acc
                }
            });
        }
    }
    Ok(combined)
}

fn merge_bounds(a: Option<([f64; 2], [f64; 2])>, b: Option<([f64; 2], [f64; 2])>) -> Option<([f64; 2], [f64; 2])> {
    match (a, b) {
        (None, other) | (other, None) => other,
        (Some((amin, amax)), Some((bmin, bmax))) => {
            Some(([amin[0].min(bmin[0]), amin[1].min(bmin[1])], [amax[0].max(bmax[0]), amax[1].max(bmax[1])]))
        }
    }
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
    stored_source_path_in(job_path, source, &current_dir())
}

fn current_dir() -> PathBuf {
    std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."))
}

/// Make `path` absolute against the process current directory when it is relative.
pub fn absolute_path(path: &Path) -> PathBuf {
    absolute_path_in(path, &current_dir())
}

fn absolute_path_in(path: &Path, cwd: &Path) -> PathBuf {
    if path.is_absolute() { path.to_path_buf() } else { cwd.join(path) }
}

fn stored_source_path_in(job_path: Option<&Path>, source: &Path, cwd: &Path) -> String {
    let source = absolute_path_in(source, cwd);
    if let Some(job) = job_path
        && let Some(job_dir) = absolute_path_in(job, cwd).parent()
        && let Some(relative) = relative_to(&source, job_dir)
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

fn operation_value(op: &OpDraft, original: Option<&Value>, units: &str, v2: bool) -> Result<Value, String> {
    let mut map = match original.and_then(Value::as_object) {
        Some(existing) => existing.clone(),
        None => Map::new(),
    };
    if op.name.trim().is_empty() {
        return Err("each operation needs a name".to_string());
    }
    map.insert("name".into(), Value::String(op.name.clone()));
    map.insert("type".into(), Value::String(op.kind.clone()));
    if op.kind == "profile" && op.layers.is_empty() && op.assign != "contours" && op.assign != "parts" {
        return Err(format!("operation `{}` needs at least one layer, or an assign-by-part or assign-by-contour selector", op.name));
    }
    if !op.layers.is_empty() {
        map.insert("layers".into(), Value::Array(op.layers.iter().map(|layer| Value::String(layer.clone())).collect()));
    } else {
        // Do not keep layers from a previously opened job when this draft has none.
        map.remove("layers");
    }
    if op.kind == "profile" && !op.side.is_empty() {
        map.insert("side".into(), Value::String(op.side.clone()));
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
    if v2 {
        if op.power.trim().is_empty() {
            map.remove("power");
        } else {
            let fraction = parse_required_number(&op.power, &format!("operation `{}` power", op.name))?;
            if !(0.0..=1.0).contains(&fraction) {
                return Err(format!("operation `{}` power must be a fraction in 0..=1", op.name));
            }
            map.insert("power".into(), serde_json::json!({ "unit": "percent", "value": fraction * 100.0 }));
        }
    } else {
        set_optional_fraction(&mut map, "power", &op.power, &format!("operation `{}` power", op.name))?;
    }
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
    write_selector(&mut map, op, units)?;
    write_tabs(&mut map, op, units)?;
    Ok(Value::Object(map))
}

fn write_tabs(map: &mut Map<String, Value>, op: &OpDraft, units: &str) -> Result<(), String> {
    let width = op.tabs_width.trim();
    let count = op.tabs_count.trim();
    let spacing = op.tabs_spacing.trim();
    if width.is_empty() && count.is_empty() && spacing.is_empty() && op.tabs_height.trim().is_empty() {
        map.remove("tabs");
        return Ok(());
    }
    if width.is_empty() {
        return Err(format!("operation `{}` tabs need a width", op.name));
    }
    let mut tabs = Map::new();
    tabs.insert("width".into(), json_number(to_file_units(parse_required_number(width, &format!("operation `{}` tab width", op.name))?, units)));
    if !count.is_empty() {
        let value: u32 = count.parse().map_err(|_| format!("operation `{}` tab count is not an integer", op.name))?;
        if value == 0 {
            return Err(format!("operation `{}` tab count must be positive", op.name));
        }
        tabs.insert("count".into(), Value::from(value));
    } else if !spacing.is_empty() {
        tabs.insert(
            "spacing".into(),
            json_number(to_file_units(parse_required_number(spacing, &format!("operation `{}` tab spacing", op.name))?, units)),
        );
    } else {
        return Err(format!("operation `{}` tabs need a count or a spacing", op.name));
    }
    if !op.tabs_height.trim().is_empty() {
        tabs.insert(
            "height".into(),
            json_number(to_file_units(parse_required_number(&op.tabs_height, &format!("operation `{}` tab height", op.name))?, units)),
        );
    }
    map.insert("tabs".into(), Value::Object(tabs));
    Ok(())
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

/// Job used for planning and posting. A loaded v2 or multi-part file is kept as-is
/// so the current single-part editor cannot drop instances.
fn planning_job(session: &JobSession) -> Result<JobFile, String> {
    job_file_of(session)
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
    let job = planning_job(session)?;
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

/// Generate toolpaths, stopping when `cancel` is signalled.
pub fn generate_toolpaths_cancellable(session: &JobSession, cancel: Option<&PlanCancel>) -> Result<PlannedSession, String> {
    let job = planning_job(session)?;
    let planned = plan_job_cancellable(&job, PlanOptions::default(), cancel).map_err(|err| err.to_string())?;
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
    if let Some(cancel) = cancel
        && cancel.is_cancelled()
    {
        return Err("planning was cancelled".to_string());
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
    let job = planning_job(session)?;
    let posted = post_job(&job, document, session.post_id.trim()).map_err(|err| err.to_string())?;
    let mut messages: Vec<SessionMessage> = posted.warnings.into_iter().map(warning).collect();
    if session.operations.iter().any(|op| !op.small_hole_max.trim().is_empty()) {
        messages.push(warning("small-hole rules are recorded as entered; this post does not emit uncited THC or pierce codes for them".to_string()));
    }
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

pub(crate) fn warning(text: impl Into<String>) -> SessionMessage {
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

    #[test]
    fn stored_source_path_resolves_cwd_relative_job_and_source() {
        let cwd = Path::new("/repo");
        let stored = stored_source_path_in(Some(Path::new("examples/laser_rect.job.json")), Path::new("examples/laser_rect.svg"), cwd);
        assert_eq!(stored, "laser_rect.svg");
        let doubled = stored_source_path_in(Some(Path::new("examples/laser_rect.job.json")), Path::new("examples/examples/laser_rect.svg"), cwd);
        assert_eq!(doubled, "examples/laser_rect.svg");
    }

    #[test]
    fn opening_example_job_with_relative_path_does_not_look_under_examples_examples() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo");
        let prev = std::env::current_dir().expect("cwd");
        std::env::set_current_dir(&root).expect("chdir");
        let result = (|| {
            let (session, _) = open_job_file(Path::new("examples/laser_rect.job.json"))?;
            let source = session.source_path.as_deref().ok_or_else(|| "source".to_string())?;
            assert!(source.is_absolute(), "{source:?}");
            assert!(!source.ends_with("examples/examples/laser_rect.svg"), "{source:?}");
            assert!(source.ends_with("examples/laser_rect.svg"), "{source:?}");
            let stored = stored_source_path(session.job_path.as_deref(), source);
            assert_eq!(stored, "laser_rect.svg", "{stored}");
            generate_toolpaths(&session).map(|_| ())
        })();
        let _ = std::env::set_current_dir(&prev);
        result.expect("open and plan laser_rect from repo root");
    }

    #[test]
    fn operations_workflow_exports_gcode_that_reflects_order_override_and_z_zero() {
        let dir = temp_dir("campanion-gui-g2-workflow");
        fs::write(dir.join("a.svg"), PART_SVG).expect("a");
        fs::write(dir.join("b.svg"), PART_SVG).expect("b");
        let first = import_part_file(&dir.join("a.svg")).expect("import a");
        let second = import_part_file(&dir.join("b.svg")).expect("import b");
        let mut session = session_from_import(&first);
        add_imported_part(&mut session, &second);
        assert_eq!(session.parts.len(), 2);
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let layer = first.layers[0].name.clone();
        let mut cut = OpDraft::new_profile("cut", 1);
        cut.layers = vec![layer.clone()];
        cut.side = "outside".to_string();
        cut.depth = "0.26".to_string();
        cut.step_down = "0.26".to_string();
        cut.cut_feed = "300".to_string();
        cut.power = "0.8".to_string();
        cut.material_name = "example acrylic".to_string();
        cut.override_fields = vec!["power".to_string()];
        let mut mark = crate::operations::new_operation("mark", "engrave", 1);
        mark.layers = vec![layer];
        mark.depth = "0.26".to_string();
        mark.step_down = "0.26".to_string();
        mark.cut_feed = "400".to_string();
        mark.power = "0.2".to_string();
        session.operations = vec![cut, mark];
        session.cut_order_explicit = true;
        session.operations.swap(0, 1); // mark first
        session.z_zero = "bed".to_string();
        session.stock_thickness = "6".to_string();
        session.post_id = "grbl_laser".to_string();
        session.order_rules_set = true;
        session.order_cuts_last = true;
        let planned = generate_toolpaths(&session).expect("plan");
        let names: Vec<_> = planned.document.program().toolpaths().iter().filter_map(|path| path.name().map(str::to_string)).collect();
        let mark_at = names.iter().position(|name| name.contains("mark")).expect("mark path");
        let cut_at = names.iter().position(|name| name.contains("cut")).expect("cut path");
        assert!(mark_at < cut_at, "explicit cut_order should put mark before cut: {names:?}");
        let (gcode, _) = post_session(&session, &planned.document).expect("post");
        let mark_line = gcode.find("(toolpath mark)").expect("mark comment");
        let cut_line = gcode.find("(toolpath cut)").expect("cut comment");
        assert!(mark_line < cut_line, "gcode order {gcode}");
        assert!(gcode.contains("S200"), "mark power {gcode}");
        assert!(gcode.contains("S800"), "cut override power {gcode}");
        let mut stock_top = session.clone();
        stock_top.z_zero = "stock_top".to_string();
        stock_top.stock_thickness.clear();
        let top = generate_toolpaths(&stock_top).expect("stock top");
        let raised = first_cut_z(&planned.document);
        let baseline = first_cut_z(&top.document);
        assert!((raised - (baseline + 6.0)).abs() < 1e-6, "raised {raised} baseline {baseline}");
        // The stock-top G-code must stay different from the bed-zero program in IR. A laser
        // post may omit Z when it never changes; the planned Z still moves.
        let json = job_json_of(&session).expect("json");
        assert!(json.contains("cut_order"), "{json}");
        assert!(json.contains("overrides"), "{json}");
        assert!(json.contains("z_zero"), "{json}");
    }

    fn first_cut_z(document: &Document) -> f64 {
        for path in document.program().toolpaths() {
            for block in path.blocks() {
                let campanion_ir::ClBlock::Motion(motion) = block else { continue };
                match motion {
                    campanion_ir::ClMotion::Linear { to, role, .. } | campanion_ir::ClMotion::Arc { to, role, .. }
                        if *role == campanion_ir::MoveRole::Cut =>
                    {
                        return to.tip().z();
                    }
                    _ => {}
                }
            }
        }
        panic!("no cut");
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

    #[test]
    fn multi_part_save_keeps_instances_and_hooks() {
        let dir = temp_dir("campanion-gui-session-multipart");
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
        crate::setup::linked_copy(&mut session).expect("copy");
        session.z_zero = "bed".to_string();
        session.stock_thickness = "6".to_string();
        session.datum_anchor = "min_min".to_string();
        session.start_from = "absolute".to_string();
        session.sheet_width = "200".to_string();
        session.sheet_height = "100".to_string();
        let saved = job_json_of(&session).expect("save");
        let value: Value = serde_json::from_str(&saved).expect("json");
        assert_eq!(value["format"], "campanion-job/2");
        assert!(value.get("source").is_none());
        assert_eq!(value["parts"][0]["instances"].as_array().map(Vec::len), Some(2));
        assert_eq!(value["z_zero"], "bed");
        assert_eq!(value["stock_thickness"], 6.0);
        assert_eq!(value["sheet"]["width"], 200.0);
        fs::write(dir.join("job.json"), saved).expect("write");
        let (reopened, _) = open_job_file(&dir.join("job.json")).expect("reopen");
        assert_eq!(reopened.parts.len(), 1);
        assert_eq!(reopened.parts[0].instances.len(), 2);
        assert_eq!(reopened.z_zero, "bed");
        assert_eq!(reopened.stock_thickness, "6");
        assert_eq!(reopened.sheet_width, "200");
        assert_eq!(reopened.datum_anchor, "min_min");
    }

    #[test]
    fn new_operation_defaults_to_all_parts_and_plans_mixed_layer_sources() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo");
        let sample = import_part_file(&root.join("examples/sample.svg")).expect("sample");
        let nest = import_part_file(&root.join("examples/nest.dxf")).expect("nest");
        let mut session = session_from_import(&sample);
        add_imported_part(&mut session, &nest);
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = crate::operations::new_operation_on_session(&session, "outline", "profile", 1);
        assert_eq!(op.assign, "parts");
        assert!(op.assign_parts.contains("sample"), "{}", op.assign_parts);
        assert!(op.assign_parts.contains("nest"), "{}", op.assign_parts);
        assert!(op.layers.is_empty());
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations.push(op);
        assert!(operation_match_warning(&session, &session.operations[0]).is_none());
        let planned = generate_toolpaths(&session).expect("plan mixed-layer parts by part assignment");
        assert!(planned.scene.strokes.iter().any(|stroke| stroke.kind == StrokeKind::Cut), "toolpaths should be generated");
        let names: Vec<_> = all_part_layers(&session).into_iter().map(|layer| layer.name).collect();
        for expected in ["OUTER", "INNER", "LEFT", "RIGHT"] {
            assert!(names.iter().any(|name| name == expected), "missing {expected} in {names:?}");
        }
    }

    #[test]
    fn missing_layer_assignment_warns_and_does_not_invent_geometry() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo");
        let sample = import_part_file(&root.join("examples/sample.svg")).expect("sample");
        let nest = import_part_file(&root.join("examples/nest.dxf")).expect("nest");
        let mut session = session_from_import(&sample);
        add_imported_part(&mut session, &nest);
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        let mut op = crate::operations::new_operation_on_session(&session, "outline", "profile", 1);
        op.assign = "layers".to_string();
        op.layers = vec!["NOPE".to_string()];
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations.push(op);
        let warning = operation_match_warning(&session, &session.operations[0]).expect("warning");
        assert!(warning.contains("matches no geometry"), "{warning}");
        let err = generate_toolpaths(&session).expect_err("typo layer on every part must not plan");
        assert!(err.contains("no geometry") || err.contains("NOPE") || err.contains("outline"), "{err}");
    }

    #[test]
    fn empty_layers_on_a_parts_assignment_do_not_keep_layers_from_the_opened_job() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo");
        let (mut session, _) = open_job_file(&root.join("examples/laser_rect.job.json")).expect("open");
        let sample = import_part_file(&root.join("examples/sample.svg")).expect("sample");
        let nest = import_part_file(&root.join("examples/nest.dxf")).expect("nest");
        session.parts.clear();
        add_imported_part(&mut session, &sample);
        add_imported_part(&mut session, &nest);
        let mut op = crate::operations::new_operation_on_session(&session, "outline", "profile", 1);
        op.side = "outside".to_string();
        op.depth = "0.26".to_string();
        op.step_down = "0.26".to_string();
        op.cut_feed = "300".to_string();
        session.operations = vec![op];
        let json = job_json_of(&session).expect("json");
        let value: Value = serde_json::from_str(&json).expect("parse");
        assert!(value["operations"][0].get("layers").is_none(), "leftover layers: {json}");
        let planned = generate_toolpaths(&session).expect("plan after replacing layers with a parts assignment");
        assert!(planned.scene.strokes.iter().any(|stroke| stroke.kind == StrokeKind::Cut));
        assert!(!planned.messages.iter().any(|message| message.text.contains("no geometry")), "messages {:?}", planned.messages);
    }
}
