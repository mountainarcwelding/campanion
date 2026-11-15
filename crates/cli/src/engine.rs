//! Planning, posting and import helpers shared by the CLI and the GUI.

use std::path::{Path, PathBuf};

use campanion_geom::Tolerance;
use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg};
use campanion_ir::Document;

use crate::error::CliError;
use crate::job::read_text;
use crate::jobfile::{self, DEFAULT_JOB_CHORD_MM, JobFile, JobFileError};
use crate::post;
use crate::run::{self, PlannedProgram};
use crate::step;

/// How a part file is read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceKind {
    /// ASCII DXF.
    Dxf,
    /// SVG.
    Svg,
    /// STEP, imported through `solidcraft-cli` when a job names a face.
    Step,
    /// Binary or ASCII STL mesh.
    Stl,
}

/// One layer name and how many contours it carries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LayerSummary {
    /// Layer name from the drawing (DXF group 8, or `"0"` when omitted).
    pub name: String,
    /// Contours on this layer.
    pub contour_count: usize,
}

/// A part imported for a new job or for preview.
#[derive(Debug, Clone)]
pub struct ImportedPart {
    /// Path that was read.
    pub path: PathBuf,
    /// FNV-1a 64 of the file bytes, 16 lowercase hex digits.
    pub hash: String,
    /// Kind inferred from the extension.
    pub kind: SourceKind,
    /// 2D contours. `None` for an STL, and for a STEP file until a job names a face.
    pub drawing: Option<Drawing>,
    /// Layers in the drawing, in first-seen order.
    pub layers: Vec<LayerSummary>,
    /// Skipped entity count from the importer.
    pub skipped_entities: usize,
    /// Skipped entity types and counts.
    pub skipped_types: Vec<(String, usize)>,
}

/// Options for [`plan_job`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PlanOptions {
    /// STEP up-axis. Ignored for every other source. The default is Z.
    pub up_axis: UpAxis,
}

/// STEP up-axis flag, matching `campanion run --up-axis`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UpAxis {
    /// Keep the file frame. This is the `campanion run` default.
    #[default]
    Z,
    /// Treat the file as Y-up and turn it onto Z.
    Y,
    /// Trust the exporter header. Opt-in only.
    Auto,
}

/// Result of planning a job, including notices the GUI should show.
#[derive(Debug, Clone)]
pub struct PlanResult {
    /// Machine-neutral program.
    pub document: Document,
    /// Notices that the CLI prints as `campanion: warning: ...`.
    pub warnings: Vec<String>,
    /// Other stderr notes (stock peaks, nest summary).
    pub notes: Vec<String>,
    /// Verify and retract-height findings that make `campanion run` exit 1.
    pub blocking: Vec<String>,
}

/// Posted G-code and any notices taken from it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Posted {
    /// Dialect text.
    pub text: String,
    /// Notices the GUI should surface, including a missing-power S0 warning.
    pub warnings: Vec<String>,
}

/// A built-in post, for a chooser list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PostInfo {
    /// Built-in id, for example `grbl_laser`.
    pub id: String,
    /// Short display name.
    pub name: String,
    /// One-line description from the post definition.
    pub description: String,
}

/// Parse a `campanion-job/1` document. `path` is stored for relative source resolution.
pub fn parse_job(text: &str, path: &Path) -> Result<JobFile, JobFileError> {
    jobfile::parse(text, path)
}

/// Import a DXF, SVG, STEP or STL part for the GUI.
///
/// STEP is recorded (path and hash) but has no 2D drawing until a job names a face and
/// `solidcraft-cli` is available. Planning a STEP job uses the same path as `campanion run`.
pub fn import_part(path: &Path) -> Result<ImportedPart, CliError> {
    let kind = source_kind(path).ok_or_else(|| CliError::run(format!("{}: expected a .dxf, .svg, .step, .stp or .stl file", path.display())))?;
    match kind {
        SourceKind::Stl => {
            let bytes = run::read_bytes(path)?;
            Ok(ImportedPart {
                path: path.to_path_buf(),
                hash: jobfile::hash_bytes(&bytes),
                kind,
                drawing: None,
                layers: Vec::new(),
                skipped_entities: 0,
                skipped_types: Vec::new(),
            })
        }
        SourceKind::Step => {
            let bytes = run::read_bytes(path)?;
            Ok(ImportedPart {
                path: path.to_path_buf(),
                hash: jobfile::hash_bytes(&bytes),
                kind,
                drawing: None,
                layers: Vec::new(),
                skipped_entities: 0,
                skipped_types: Vec::new(),
            })
        }
        SourceKind::Dxf | SourceKind::Svg => {
            let text = read_text(path)?;
            let bytes = text.as_bytes();
            let options = ImportOptions::new(default_tolerance());
            let drawing = if kind == SourceKind::Svg { import_svg(&text, &options) } else { import_dxf(&text, &options) }
                .map_err(|err| CliError::run(err.to_string()))?;
            let layers = layer_summaries(&drawing);
            let skipped_entities = drawing.skipped_entities();
            let skipped_types = drawing.skipped_types().to_vec();
            Ok(ImportedPart {
                path: path.to_path_buf(),
                hash: jobfile::hash_bytes(bytes),
                kind,
                drawing: Some(drawing),
                layers,
                skipped_entities,
                skipped_types,
            })
        }
    }
}

/// Plan a loaded job the same way `campanion run` does, without writing G-code.
pub fn plan_job(job: &JobFile, options: PlanOptions) -> Result<PlanResult, CliError> {
    let planned: PlannedProgram = run::plan_loaded_job(job, options.up_axis.into())?;
    let blocking = run::blocking_findings(job, &planned.document, &planned.gouges);
    Ok(PlanResult { document: planned.document, warnings: planned.warnings, notes: planned.notes, blocking })
}

/// Post a planned program with a built-in id or a `.json` post file.
///
/// The text is read back and compared at the job chord tolerance, matching `campanion run`.
pub fn post_job(job: &JobFile, document: &Document, post_spec: &str) -> Result<Posted, CliError> {
    if post_spec.is_empty() {
        return Err(CliError::run("choose a post before exporting G-code"));
    }
    let post_def = post::load_spec(Path::new(post_spec))?;
    run::require_spindle_speeds(job, &post_def)?;
    let text = post::render(document.program(), &post_def, job.tolerance().chord())?;
    let mut warnings = Vec::new();
    if text.contains("power was not set") {
        warnings.push("process power was not set; writing S0 (beam off)".to_string());
    }
    Ok(Posted { text, warnings })
}

/// Built-in posts, in the same order as `campanion posts`.
pub fn builtin_posts() -> Vec<PostInfo> {
    let mut posts = Vec::new();
    for id in campanion_post::builtin_ids() {
        if let Some(post) = campanion_post::builtin(id) {
            posts.push(PostInfo { id: post.id().to_string(), name: post.name().to_string(), description: post.description().to_string() });
        }
    }
    posts
}

/// A built-in post that matches a GUI sub-process, when there is a single clear choice.
///
/// Milling and routing have several mill posts, so this returns `None` for those. The value is a
/// suggestion for the chooser, not a machine default.
pub fn suggested_post(process: &str) -> Option<&'static str> {
    match process {
        "laser" => Some("grbl_laser"),
        "plasma" => Some("plasma"),
        "waterjet" => Some("waterjet"),
        "oxyfuel" | "oxy-fuel" => Some("oxyfuel"),
        "wireedm" | "wire-edm" | "wire_edm" => Some("wire_edm"),
        _ => None,
    }
}

fn default_tolerance() -> Tolerance {
    Tolerance::with_chord(Tolerance::DEFAULT_LINEAR_MM, Tolerance::DEFAULT_ANGULAR_RAD, DEFAULT_JOB_CHORD_MM).unwrap_or_else(|_| Tolerance::default())
}

fn source_kind(path: &Path) -> Option<SourceKind> {
    let ext = path.extension()?.to_str()?;
    if ext.eq_ignore_ascii_case("dxf") {
        Some(SourceKind::Dxf)
    } else if ext.eq_ignore_ascii_case("svg") {
        Some(SourceKind::Svg)
    } else if ext.eq_ignore_ascii_case("step") || ext.eq_ignore_ascii_case("stp") {
        Some(SourceKind::Step)
    } else if ext.eq_ignore_ascii_case("stl") {
        Some(SourceKind::Stl)
    } else {
        None
    }
}

fn layer_summaries(drawing: &Drawing) -> Vec<LayerSummary> {
    let mut layers = Vec::new();
    for contour in drawing.contours() {
        match layers.iter_mut().find(|layer: &&mut LayerSummary| layer.name == contour.layer()) {
            Some(layer) => layer.contour_count = layer.contour_count.saturating_add(1),
            None => layers.push(LayerSummary { name: contour.layer().to_string(), contour_count: 1 }),
        }
    }
    layers
}

impl From<UpAxis> for step::UpAxis {
    fn from(value: UpAxis) -> Self {
        match value {
            UpAxis::Z => step::UpAxis::Z,
            UpAxis::Y => step::UpAxis::Y,
            UpAxis::Auto => step::UpAxis::Auto,
        }
    }
}

impl From<step::UpAxis> for UpAxis {
    fn from(value: step::UpAxis) -> Self {
        match value {
            step::UpAxis::Z => UpAxis::Z,
            step::UpAxis::Y => UpAxis::Y,
            step::UpAxis::Auto => UpAxis::Auto,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn suggested_laser_post_is_grbl_laser() {
        assert_eq!(suggested_post("laser"), Some("grbl_laser"));
        assert_eq!(suggested_post("milling"), None);
    }

    #[test]
    fn source_kind_reads_extensions() {
        assert_eq!(source_kind(Path::new("part.DXF")), Some(SourceKind::Dxf));
        assert_eq!(source_kind(Path::new("part.svg")), Some(SourceKind::Svg));
        assert_eq!(source_kind(Path::new("part.stp")), Some(SourceKind::Step));
        assert_eq!(source_kind(Path::new("part.job.json")), None);
    }
}
