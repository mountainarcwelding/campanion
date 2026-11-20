//! Simulate-tab playback index, run-time estimates, and session pre-flight checks.
//!
//! The G-code listing is the posted text from [`crate::session::post_session`], not a second
//! writer. Line mapping uses [`campanion_post::read_back_indexed`] on that text. Run-time
//! estimates use only feeds and delays already present on the IR (user-entered rates). A rapid
//! with no feed is omitted from the total rather than given an invented rate.

use campanion_cli::{PreflightItem, PreflightLevel, ProfileStore, export_preflight_with};
use campanion_ir::{ArcSense, ClBlock, ClMotion, Document, Event, Plane, Vec3};
use campanion_post::{IndexedMove, ReadMove, builtin, load_post, read_back_indexed};
use campanion_view::{Scene, Stroke, StrokeKind};

use crate::session::{JobSession, job_file_of, operation_match_warning};

/// One posted G-code line together with the scene-length interval it covers, when it is motion.
#[derive(Debug, Clone, PartialEq)]
pub struct LineSpan {
    /// Distance along the scene at the start of this motion, millimetres.
    pub start: f64,
    /// Distance along the scene at the end of this motion, millimetres.
    pub end: f64,
    /// 0-based line index in the posted text.
    pub line: usize,
    /// Whether this span is a rapid.
    pub rapid: bool,
}

/// A playhead-to-G-code index built from a scene and the posted text that produced it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlayIndex {
    /// Posted lines, including comments and blanks, in file order.
    pub lines: Vec<String>,
    /// Motion spans in scene order.
    pub spans: Vec<LineSpan>,
}

impl PlayIndex {
    /// The G-code line that covers `position` along the scene, if any.
    ///
    /// A position at a boundary belongs to the following span, except at the exact end, which
    /// stays on the last span.
    #[must_use]
    pub fn line_at(&self, position: f64) -> Option<usize> {
        if self.spans.is_empty() {
            return None;
        }
        let last = self.spans.len() - 1;
        for (index, span) in self.spans.iter().enumerate() {
            if index == last {
                if position >= span.start {
                    return Some(span.line);
                }
            } else if position >= span.start && position < span.end {
                return Some(span.line);
            }
        }
        self.spans.first().map(|span| span.line)
    }

    /// Scene position to jump to when the user clicks `line`.
    ///
    /// A non-motion line maps to the next motion line, or the last motion before it when there
    /// is no later motion.
    #[must_use]
    pub fn position_for_line(&self, line: usize) -> Option<f64> {
        if let Some(span) = self.spans.iter().find(|span| span.line == line) {
            return Some(span.start);
        }
        if let Some(span) = self.spans.iter().find(|span| span.line > line) {
            return Some(span.start);
        }
        self.spans.last().map(|span| span.start)
    }
}

/// Build a play index from a preview scene and the G-code posted for that program.
pub fn play_index(scene: &Scene, gcode: &str, post_spec: &str) -> Result<PlayIndex, String> {
    let post = load_post_def(post_spec)?;
    let moves = read_back_indexed(gcode, &post).map_err(|err| err.to_string())?;
    Ok(play_index_from_moves(scene, gcode, &moves))
}

fn load_post_def(spec: &str) -> Result<campanion_post::PostDef, String> {
    let trimmed = spec.trim();
    if trimmed.is_empty() {
        return Err("choose a post before linking G-code to the playhead".to_string());
    }
    if trimmed.ends_with(".json") {
        let text = std::fs::read_to_string(trimmed).map_err(|err| format!("{trimmed}: {err}"))?;
        return load_post(&text).map_err(|err| err.to_string());
    }
    builtin(trimmed).ok_or_else(|| format!("unknown post `{trimmed}`"))
}

fn play_index_from_moves(scene: &Scene, gcode: &str, moves: &[IndexedMove]) -> PlayIndex {
    let lines: Vec<String> = gcode.lines().map(str::to_string).collect();
    let motions: Vec<(usize, bool)> = moves
        .iter()
        .filter_map(|item| match item.motion {
            ReadMove::Rapid { .. } => Some((item.line, true)),
            ReadMove::Linear { .. } | ReadMove::Arc { .. } | ReadMove::Thread { .. } => Some((item.line, false)),
            ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
        })
        .collect();
    let stroke_indexes = pair_strokes_to_motions(&scene.strokes, motions.len());
    let mut spans = Vec::new();
    let mut distance = 0.0;
    for (stroke_index, stroke) in scene.strokes.iter().enumerate() {
        let start = distance;
        distance += stroke.length;
        let Some(motion_index) = stroke_indexes.get(stroke_index).copied().flatten() else {
            continue;
        };
        let Some(&(line, rapid)) = motions.get(motion_index) else {
            continue;
        };
        spans.push(LineSpan { start, end: distance, line, rapid });
    }
    PlayIndex { lines, spans }
}

/// Pair scene strokes with posted motion blocks.
///
/// Laser posts may drop Z-only rapids. Those strokes are left unmapped when the scene has more
/// strokes than posted motion blocks.
fn pair_strokes_to_motions(strokes: &[Stroke], motion_count: usize) -> Vec<Option<usize>> {
    let mut map = vec![None; strokes.len()];
    if motion_count == 0 || strokes.is_empty() {
        return map;
    }
    if strokes.len() == motion_count {
        for (index, slot) in map.iter_mut().enumerate() {
            *slot = Some(index);
        }
        return map;
    }
    let skippable = strokes.iter().filter(|stroke| is_unposted_rapid(stroke)).count();
    let posted_strokes = strokes.len().saturating_sub(skippable);
    let skip_unposted = posted_strokes == motion_count && skippable > 0;
    let mut motion_index = 0;
    for (stroke_index, stroke) in strokes.iter().enumerate() {
        if motion_index >= motion_count {
            break;
        }
        if skip_unposted && is_unposted_rapid(stroke) {
            continue;
        }
        map[stroke_index] = Some(motion_index);
        motion_index += 1;
    }
    map
}

fn is_unposted_rapid(stroke: &Stroke) -> bool {
    stroke.kind == StrokeKind::Rapid && plan_length(stroke) < 1e-9
}

fn plan_length(stroke: &Stroke) -> f64 {
    let Some(first) = stroke.points.first() else {
        return 0.0;
    };
    let Some(last) = stroke.points.last() else {
        return 0.0;
    };
    let dx = last[0] - first[0];
    let dy = last[1] - first[1];
    (dx * dx + dy * dy).sqrt()
}

/// Run-time estimate taken only from feeds and delays already on the IR.
#[derive(Debug, Clone, PartialEq)]
pub struct TimeEstimate {
    /// Seconds of cutting, plunge, lead and retract motion that carry a feed.
    pub cut_seconds: f64,
    /// Seconds of rapid motion that carry a user-entered feed. `None` when any rapid has no feed.
    pub rapid_seconds: Option<f64>,
    /// Seconds from IR dwells and entered pierce or post delays.
    pub dwell_seconds: f64,
    /// True when at least one rapid had no feed and was omitted from the total.
    pub rapid_omitted: bool,
}

impl TimeEstimate {
    /// A short label that does not invent a rapid rate.
    #[must_use]
    pub fn label(&self) -> String {
        let cut = format_seconds(self.cut_seconds);
        let mut parts = vec![format!("Cut time from entered feeds: {cut}")];
        if self.dwell_seconds > 0.0 {
            parts.push(format!("Entered dwells: {}", format_seconds(self.dwell_seconds)));
        }
        if self.rapid_omitted || self.rapid_seconds.is_none() {
            parts.push("Rapid time omitted — no rapid rate was entered".to_string());
        } else if let Some(rapid) = self.rapid_seconds {
            parts.push(format!("Rapid time from entered rates: {}", format_seconds(rapid)));
        }
        parts.join(". ") + "."
    }
}

/// Format a duration for the Simulate panel.
#[must_use]
pub fn format_seconds(seconds: f64) -> String {
    if !seconds.is_finite() || seconds <= 0.0 {
        return "0 s".to_string();
    }
    if seconds < 60.0 {
        if seconds >= 10.0 && (seconds - seconds.round()).abs() < 0.05 {
            return format!("{} s", seconds.round() as i64);
        }
        return format!("{seconds:.1} s");
    }
    let total = seconds.round() as i64;
    let minutes = total / 60;
    let rest = total % 60;
    format!("{minutes}:{rest:02}")
}

/// Sum cut/plunge time from IR feeds. Rapids without a feed are omitted, not invented.
#[must_use]
pub fn estimate_time(document: &Document) -> TimeEstimate {
    let mut cut_seconds = 0.0;
    let mut rapid_seconds = 0.0;
    let mut rapid_omitted = false;
    let mut dwell_seconds = 0.0;
    for path in document.program().toolpaths() {
        let mut from: Option<Vec3> = None;
        for block in path.blocks() {
            match block {
                ClBlock::Event(Event::Dwell { seconds }) => dwell_seconds += *seconds,
                ClBlock::Event(Event::ProcessOn { params, .. }) => {
                    dwell_seconds += params.pierce_delay_s();
                    dwell_seconds += params.post_delay_s();
                }
                ClBlock::Event(_) => {}
                ClBlock::Motion(motion) => {
                    let end = motion_end(motion);
                    let length = match from {
                        Some(start) => motion_length(start, motion),
                        None => 0.0,
                    };
                    from = Some(end);
                    match motion {
                        ClMotion::Rapid { feed, .. } => match *feed {
                            Some(rate) if rate > 0.0 && rate.is_finite() => rapid_seconds += length_time(length, rate),
                            _ => rapid_omitted = true,
                        },
                        ClMotion::Linear { feed, .. } | ClMotion::Arc { feed, .. } | ClMotion::Helix { feed, .. } => {
                            if *feed > 0.0 && feed.is_finite() {
                                cut_seconds += length_time(length, *feed);
                            }
                        }
                        ClMotion::Thread { .. } => {}
                    }
                }
            }
        }
    }
    TimeEstimate { cut_seconds, rapid_seconds: if rapid_omitted { None } else { Some(rapid_seconds) }, dwell_seconds, rapid_omitted }
}

fn length_time(length_mm: f64, feed_mm_per_min: f64) -> f64 {
    if length_mm <= 0.0 || !length_mm.is_finite() {
        return 0.0;
    }
    (length_mm / feed_mm_per_min) * 60.0
}

fn motion_end(motion: &ClMotion) -> Vec3 {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => to.tip(),
    }
}

fn motion_length(from: Vec3, motion: &ClMotion) -> f64 {
    match motion {
        ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. } | ClMotion::Thread { to, .. } => tip_distance(from, to.tip()),
        ClMotion::Arc { to, center, plane, sense, .. } => arc_length(from, to.tip(), *center, *plane, *sense),
        ClMotion::Helix { to, center, plane, sweep, .. } => helix_length(from, to.tip(), *center, *plane, *sweep),
    }
}

fn tip_distance(a: Vec3, b: Vec3) -> f64 {
    let dx = a.x() - b.x();
    let dy = a.y() - b.y();
    let dz = a.z() - b.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn plane_coords(point: Vec3, plane: Plane) -> (f64, f64, f64) {
    match plane {
        Plane::Xy => (point.x(), point.y(), point.z()),
        Plane::Zx => (point.z(), point.x(), point.y()),
        Plane::Yz => (point.y(), point.z(), point.x()),
    }
}

fn arc_length(from: Vec3, to: Vec3, center: Vec3, plane: Plane, sense: ArcSense) -> f64 {
    let (sx, sy, sz) = plane_coords(from, plane);
    let (ex, ey, ez) = plane_coords(to, plane);
    let (cx, cy, _) = plane_coords(center, plane);
    let start = (sx - cx, sy - cy);
    let end = (ex - cx, ey - cy);
    let radius = (start.0 * start.0 + start.1 * start.1).sqrt();
    if radius < 1e-12 {
        return tip_distance(from, to);
    }
    let a0 = start.1.atan2(start.0);
    let a1 = end.1.atan2(end.0);
    let mut sweep = match sense {
        ArcSense::Ccw => a1 - a0,
        ArcSense::Cw => a0 - a1,
    };
    if sweep <= 1e-12 {
        sweep += std::f64::consts::TAU;
    }
    let along = radius * sweep;
    let dz = ez - sz;
    (along * along + dz * dz).sqrt()
}

fn helix_length(from: Vec3, to: Vec3, center: Vec3, plane: Plane, sweep: f64) -> f64 {
    let (sx, sy, sz) = plane_coords(from, plane);
    let (_, _, ez) = plane_coords(to, plane);
    let (cx, cy, _) = plane_coords(center, plane);
    let radius = ((sx - cx).powi(2) + (sy - cy).powi(2)).sqrt();
    let along = radius * sweep.abs();
    let dz = ez - sz;
    (along * along + dz * dz).sqrt()
}

/// Session-level checks plus the shared job pre-flight list.
///
/// Missing feeds and empty operations are errors. Missing laser power and unused material
/// fields are warnings. Off-bed is an error; off-sheet is a warning.
pub fn session_checks(session: &JobSession, document: Option<&Document>, store: Option<&ProfileStore>, process: Option<&str>) -> Vec<PreflightItem> {
    let mut items = Vec::new();
    if session.operations.is_empty() {
        items.push(PreflightItem { level: PreflightLevel::Error, message: "the job has no operations".to_string() });
    }
    let laser = process.is_some_and(|name| name.eq_ignore_ascii_case("laser")) || session.post_id.contains("laser");
    for op in &session.operations {
        if op.cut_feed.trim().is_empty() {
            items.push(PreflightItem {
                level: PreflightLevel::Error,
                message: format!("operation `{}` is missing a cut feed. Enter a feed or apply a material that has one.", op.name),
            });
        }
        if op.material_name.trim().is_empty() && (op.cut_feed.trim().is_empty() || op.depth.trim().is_empty() || op.power.trim().is_empty()) {
            items.push(PreflightItem { level: PreflightLevel::Warning, message: format!("operation `{}` has no material values", op.name) });
        }
        if laser && op.power.trim().is_empty() {
            items.push(PreflightItem {
                level: PreflightLevel::Warning,
                message: format!("operation `{}` has no power. A laser post writes S0 (beam off).", op.name),
            });
        }
        if let Some(message) = operation_match_warning(session, op) {
            items.push(PreflightItem { level: PreflightLevel::Error, message });
        }
    }
    match job_file_of(session) {
        Ok(job) => match export_preflight_with(&job, document, store) {
            Ok(extra) => {
                for item in extra {
                    if !items.iter().any(|existing| existing.message == item.message) {
                        items.push(item);
                    }
                }
            }
            Err(err) => items.push(PreflightItem { level: PreflightLevel::Error, message: err.to_string() }),
        },
        Err(err) => {
            if !items.iter().any(|item| item.blocks_export()) {
                items.push(PreflightItem { level: PreflightLevel::Error, message: err });
            }
        }
    }
    items
}

/// True when any check must block export.
#[must_use]
pub fn blocks_export(items: &[PreflightItem]) -> bool {
    items.iter().any(PreflightItem::blocks_export)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machines::{apply_material, first_user_laser_profile};
    use crate::operations::new_operation_on_session;
    use crate::session::{
        ToolDraft, ToolKind, TreeSelection, add_imported_part, generate_toolpaths, import_part_file, post_session, write_gcode, write_job,
    };
    use campanion_cli::{PlanOptions, parse_job, plan_job, post_job};
    use campanion_view::build_scene;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples")
    }

    fn laser_rect_svg() -> PathBuf {
        examples_dir().join("laser_rect.svg")
    }

    fn first_user_session() -> (JobSession, campanion_ir::Document, campanion_view::Scene, String) {
        let mut session = JobSession::new();
        let imported = import_part_file(&laser_rect_svg()).unwrap_or_else(|err| panic!("{err}"));
        add_imported_part(&mut session, &imported);
        if session.tools.is_empty() {
            session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: String::new() });
        }
        let tool_id = session.tools[0].id;
        session.operations.push(new_operation_on_session(&session, "outline", "profile", tool_id));
        session.selected = Some(TreeSelection::Operation(0));
        let profile = first_user_laser_profile();
        let material = profile.heads[0].materials[0].clone();
        apply_material(&mut session, &material).unwrap_or_else(|err| panic!("{err}"));
        session.post_id = "grbl_laser".to_string();
        let planned = generate_toolpaths(&session).unwrap_or_else(|err| panic!("{err}"));
        let (gcode, _) = post_session(&session, &planned.document).unwrap_or_else(|err| panic!("{err}"));
        let scene = build_scene(&planned.document, 0.01);
        (session, planned.document, scene, gcode)
    }

    #[test]
    fn time_estimate_uses_entered_feeds_and_omits_rapids_without_a_rate() {
        let (_session, document, _scene, _gcode) = first_user_session();
        let estimate = estimate_time(&document);
        assert!(estimate.cut_seconds > 0.0, "cut time should be positive: {estimate:?}");
        assert!(estimate.rapid_omitted, "laser_rect rapids should have no invented feed: {estimate:?}");
        assert!(estimate.rapid_seconds.is_none());
        let label = estimate.label();
        assert!(label.contains("Cut time from entered feeds"), "{label}");
        assert!(label.contains("no rapid rate was entered"), "{label}");
        assert!(!label.to_lowercase().contains("invent"));
    }

    #[test]
    fn playhead_line_matches_a_known_ir_move_and_gcode_line() {
        let (_session, _document, scene, gcode) = first_user_session();
        let index = play_index(&scene, &gcode, "grbl_laser").unwrap_or_else(|err| panic!("{err}"));
        assert!(!index.spans.is_empty(), "expected mapped motion spans");
        let first_cut = index.spans.iter().find(|span| !span.rapid).unwrap_or_else(|| panic!("no cut span: {:?}", index.spans));
        let line = first_cut.line;
        let text = index.lines.get(line).unwrap_or_else(|| panic!("missing line {line}"));
        assert!(
            text.contains("G1") || text.contains("G01") || text.contains("X") || text.contains("Y"),
            "expected a feed move on line {line}: {text}"
        );
        assert_eq!(index.line_at(first_cut.start + 0.1 * (first_cut.end - first_cut.start).max(0.01)), Some(line));
        assert_eq!(index.position_for_line(line), Some(first_cut.start));
        // Header comments should jump to the first motion.
        if line > 0 {
            let jumped = index.position_for_line(0);
            assert_eq!(jumped, Some(index.spans[0].start));
        }
    }

    #[test]
    fn preflight_blocks_empty_ops_and_allows_warning_only_jobs() {
        let (mut session, document, _scene, _gcode) = first_user_session();
        let items = session_checks(&session, Some(&document), None, Some("laser"));
        assert!(!blocks_export(&items), "a complete first-user job should export: {items:?}");

        session.operations[0].layers = vec!["NOPE".to_string()];
        session.operations[0].assign = "layers".to_string();
        session.operations[0].assign_parts.clear();
        let items = session_checks(&session, Some(&document), None, Some("laser"));
        assert!(blocks_export(&items), "a typo layer must block export: {items:?}");

        let mut missing = JobSession::new();
        missing.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: String::new() });
        missing.operations.push(new_operation_on_session(&missing, "outline", "profile", 1));
        let items = session_checks(&missing, None, None, Some("laser"));
        assert!(items.iter().any(|item| item.blocks_export() && item.message.contains("cut feed")), "{items:?}");
        assert!(items.iter().any(|item| item.level == PreflightLevel::Warning && item.message.contains("material values")), "{items:?}");
    }

    #[test]
    fn first_user_export_matches_cli_post_of_the_saved_job() {
        let (mut session, _document, _scene, _preview) = first_user_session();
        let dir = std::env::temp_dir().join(format!("campanion-g3-flow-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap_or_else(|err| panic!("{err}"));
        let job_path = dir.join("job.json");
        let gui_path = dir.join("gui.nc");
        write_job(&job_path, &session).unwrap_or_else(|err| panic!("{err}"));
        session.job_path = Some(job_path.clone());
        let planned = generate_toolpaths(&session).unwrap_or_else(|err| panic!("{err}"));
        let (gcode, _) = post_session(&session, &planned.document).unwrap_or_else(|err| panic!("{err}"));
        write_gcode(&gui_path, &gcode).unwrap_or_else(|err| panic!("{err}"));
        assert!(gcode.contains("G21"), "expected grbl_laser header: {gcode}");
        let text = std::fs::read_to_string(&job_path).unwrap_or_else(|err| panic!("{err}"));
        let job = parse_job(&text, &job_path).unwrap_or_else(|err| panic!("{err}"));
        let planned = plan_job(&job, PlanOptions { up_axis: Default::default() }).unwrap_or_else(|err| panic!("{err}"));
        let posted = post_job(&job, &planned.document, "grbl_laser").unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(gcode, posted.text, "GUI export must match campanion run --post grbl_laser on the saved job");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn off_bed_placement_still_blocks_export_when_an_operation_is_selected() {
        let (mut session, document, _scene, _gcode) = first_user_session();
        session.parts[0].instances[0].x = "700".to_string();
        session.parts[0].instances[0].y = "20".to_string();
        session.selected = Some(TreeSelection::Operation(0));
        let dir = std::env::temp_dir().join(format!("campanion-g3-offbed-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("machines")).unwrap_or_else(|err| panic!("{err}"));
        let store = campanion_cli::ProfileStore::from_dir(dir.clone());
        store.save("my-laser", &first_user_laser_profile()).unwrap_or_else(|err| panic!("{err}"));
        session.machine_profile = "my-laser".to_string();
        session.machine_head = "laser".to_string();
        let items = session_checks(&session, Some(&document), Some(&store), Some("laser"));
        assert!(
            items.iter().any(|item| item.blocks_export() && item.message.contains("off the bed")),
            "expected an off-bed export error, got {items:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
