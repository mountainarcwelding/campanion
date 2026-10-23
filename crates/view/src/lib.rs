//! Plan-view preview model for CAMpanion toolpaths.
//!
//! The crate is pure data and geometry: no UI toolkit. Distances are millimetres. Arc sweeps are
//! radians; rotary angles are degrees, matching the IR. World coordinates are Y-up and
//! right-handed. Screen coordinates are Y-down pixels.
//!
//! A [`Scene`] is built by walking every toolpath with one cursor. The first motion of the
//! program only places that cursor; every later motion becomes a [`Stroke`]. XY arcs and helices
//! are tessellated so the chord error stays within the requested tolerance. Playback then reveals
//! the scene by arc length, and [`View`] maps world millimetres onto a viewport. A rotary setup
//! adds the axis segment through the centre, across the toolpath extent, and either the distinct
//! index angles or the continuous angle range.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

use campanion_ir::{ArcSense, ClBlock, ClMotion, Document, Event, Plane, RotaryAxisKind, RotarySetup};

/// Motions whose XY travel is at or below this (millimetres) are vertical feeds.
///
/// One nanometre is far below any posted coordinate, so float noise does not turn a plunge into a cut.
const XY_CHANGE_MM: f64 = 1e-6;

/// Hard cap on segments in one tessellated arc. Keeps a hostile tolerance from allocating without bound.
const MAX_ARC_SEGMENTS: usize = 8_192;

/// Chord tolerance used when the caller passes a non-positive or non-finite value.
const DEFAULT_ARC_TOL_MM: f64 = 0.05;

/// Angles smaller than this (radians) are treated as "no sweep" rather than a full turn.
const MIN_SWEEP_RAD: f64 = 1e-12;

/// Length comparisons below this (millimetres) are ties.
const LEN_EPS_MM: f64 = 1e-9;

/// Plan labels closer than this (millimetres) share one callout.
const LABEL_MERGE_MM: f64 = 0.5;

/// Angles closer than this, in degrees, are one index.
///
/// A feed whose ends differ by more than this is continuous rotary motion. Index-change rapids
/// are allowed to jump without making the program continuous.
const INDEX_ANGLE_EPS_DEG: f64 = 1e-6;

/// Flat cut colour, the same blue the preview uses for an unshaded cut stroke: `(90, 200, 250)`.
const FLAT_CUT_RGB: [u8; 3] = [90, 200, 250];

/// Kind of motion drawn in the plan preview.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrokeKind {
    /// A rapid traverse. Vertical rapids stay rapids.
    Rapid,
    /// A feed move whose plan view changes XY, including arcs and helices.
    Cut,
    /// A non-rapid move whose XY does not change (a vertical feed).
    Plunge,
}

/// One continuous polyline in the plan view, produced from a single motion.
///
/// `length` is the 3D length of `points` in millimetres, using [`Stroke::zs`] as the height of
/// each vertex. Playback cuts this same polyline, so the stored length and the drawn path agree.
#[derive(Debug, Clone, PartialEq)]
pub struct Stroke {
    /// Rapid, cut, or vertical plunge.
    pub kind: StrokeKind,
    /// Index of the source toolpath in program order.
    pub toolpath: usize,
    /// Plan-view polyline, including the start and the exact end. Y is up.
    pub points: Vec<[f64; 2]>,
    /// Z of each point in [`Stroke::points`], in millimetres. Always the same length as `points`.
    ///
    /// A line stores the exact endpoint heights. An arc or helix stores heights interpolated by
    /// XY arc length from `z_start` to `z_end`.
    pub zs: Vec<f64>,
    /// Z of the first point, in millimetres.
    pub z_start: f64,
    /// Z of the last point, in millimetres.
    pub z_end: f64,
    /// Rotary angle of each point, in degrees. Empty when the motion has no rotary angle.
    ///
    /// The same length as [`Stroke::points`] when present. A line stores the endpoint angles.
    /// An arc or helix interpolates by XY arc length, the same way as [`Stroke::zs`].
    pub angles: Vec<f64>,
    /// 3D length of the polyline, in millimetres.
    pub length: f64,
}

/// Distinct index angles, or the range of a continuous rotary move.
#[derive(Debug, Clone, PartialEq)]
pub enum RotarySpan {
    /// Constant angles, degrees, ascending and de-duplicated.
    Indexed(Vec<f64>),
    /// Inclusive range of angles that change along a feed, in degrees.
    Continuous { min_deg: f64, max_deg: f64 },
}

/// Rotary angles at the two ends of one motion.
///
/// `rapid` is an index change or a positioning move. A jump on a rapid does not make the program
/// continuous; a feed whose ends differ does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotarySample {
    /// Angle at the start of the motion, degrees.
    pub start_deg: f64,
    /// Angle at the end of the motion, degrees.
    pub end_deg: f64,
    /// `true` when the motion is a rapid.
    pub rapid: bool,
}

/// Colour stops for the rotary-angle legend. Low is blue and high is red, same ramp as height.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AngleLegend {
    /// Label for the low end of the range, such as `"0°"`.
    pub low_label: String,
    /// Label for the high end of the range, such as `"90°"`.
    pub high_label: String,
    /// One colour per step from low to high, inclusive.
    pub colours: Vec<[u8; 3]>,
}

/// Axis line and angle summary for a program that has a rotary setup.
#[derive(Debug, Clone, PartialEq)]
pub struct RotaryPreview {
    /// `"A"` (part +X) or `"B"` (part +Y), drawn beside the axis line.
    pub label: &'static str,
    /// A point on the axis, millimetres, part frame.
    pub center: [f64; 3],
    /// Segment through [`RotaryPreview::center`] spanning the toolpath on that axis.
    ///
    /// `None` when the program drew nothing. The pair is `(start, end)`.
    pub line: Option<([f64; 3], [f64; 3])>,
    /// Index list or continuous range. `None` when no pose carries an angle.
    pub span: Option<RotarySpan>,
}

impl RotaryPreview {
    /// Low and high angles used to colour moves, in degrees.
    ///
    /// Indexed programs use the first and last distinct angle. A single index has a zero-width
    /// range, which [`angle_color`] draws as the flat cut colour.
    pub fn angle_range(&self) -> Option<(f64, f64)> {
        match &self.span {
            Some(RotarySpan::Indexed(angles)) => {
                let min_deg = angles.iter().copied().fold(f64::INFINITY, f64::min);
                let max_deg = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                if min_deg.is_finite() && max_deg.is_finite() { Some((min_deg, max_deg)) } else { None }
            }
            Some(RotarySpan::Continuous { min_deg, max_deg }) => Some((*min_deg, *max_deg)),
            None => None,
        }
    }

    /// Overlay sentence for this setup, such as `"Index: 0°, 90°"`.
    pub fn overlay(&self) -> Option<String> {
        self.span.as_ref().map(rotary_overlay)
    }
}

/// A pierce: the plan position where a process turns on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Marker {
    /// XY of the cursor when `ProcessOn` is seen, in millimetres.
    pub at: [f64; 2],
    /// Index of the toolpath that contains the event.
    pub toolpath: usize,
    /// Index of the next stroke (`strokes.len()` at the event). May equal the stroke count when nothing follows.
    pub stroke_index: usize,
}

/// An operation-order label drawn at the first cutting point of a toolpath.
#[derive(Debug, Clone, PartialEq)]
pub struct Label {
    /// Plan position of the label, in millimetres. Y is up.
    pub at: [f64; 2],
    /// 1-based toolpath index, rendered as `"1"`, `"2"`, and so on.
    pub text: String,
}

/// Everything the plan preview draws and measures for one program.
#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    /// Motions in program order. The first motion of the program is not a stroke; it only sets the cursor.
    pub strokes: Vec<Stroke>,
    /// One marker for each `ProcessOn` that happens after the cursor exists.
    pub pierces: Vec<Marker>,
    /// Callouts for toolpaths that cut or plunge. Starts within 0.5 mm share one label (`"2, 3"`).
    pub labels: Vec<Label>,
    /// Axis-aligned bounds of every stroke point, rapids included. `None` when nothing was drawn.
    /// The pair is `(min_xy, max_xy)`.
    pub bounds: Option<([f64; 2], [f64; 2])>,
    /// Min and max Z of every cut and plunge point, in millimetres. Rapids are left out.
    /// `None` when the program never cuts or plunges.
    pub z_range: Option<(f64, f64)>,
    /// Sum of [`Stroke::length`], in millimetres.
    pub total_length: f64,
    /// Toolpath names in program order. Missing names are empty strings.
    pub names: Vec<String>,
    /// Rotary axis and angles when the program has a setup. `None` for a 3-axis program.
    pub rotary: Option<RotaryPreview>,
}

/// Parse a program and build its preview scene.
///
/// `arc_tol` is the maximum chord deviation for XY arcs and helices, in millimetres. A
/// non-positive or non-finite tolerance falls back to 0.05 mm.
///
/// # Errors
///
/// Returns the IR parser error when `json` is not a document.
pub fn load_scene(json: &str, arc_tol: f64) -> Result<Scene, String> {
    let doc = Document::from_json(json).map_err(|err| err.to_string())?;
    Ok(build_scene(&doc, arc_tol))
}

/// Build a preview scene from an already parsed document.
///
/// Toolpaths are walked in order with a single cursor. The first finite motion only places that
/// cursor (a program has no position before it). Each later motion becomes one stroke from the
/// cursor to the motion's tip. `ProcessOn` drops a pierce at the cursor. `ProcessOff`, `Dwell`,
/// and any other event do not move the cursor.
///
/// XY arcs take their sweep from the start and end angles about the centre, in the given sense. A
/// full circle is used when the start and end plan points coincide. Helices use `sweep` as the
/// total angle in radians (the sign is ignored; [`ArcSense`] picks the direction). Arcs and
/// helices on any other plane are drawn as a straight chord. The end point of every stroke is the
/// motion tip exactly.
///
/// Labels whose points lie within 0.5 mm of an earlier label are then merged into that label.
/// Texts join with `", "` (for example `"2, 3"`) and the earlier label keeps its position.
///
/// When the program has a rotary setup, each stroke also stores the rotary angle along the move
/// (empty for a 3-axis pose). The scene's [`RotaryPreview`] is the axis segment through the
/// setup centre, across the toolpath extent, plus the index list or the continuous range.
pub fn build_scene(doc: &Document, arc_tol: f64) -> Scene {
    let tol = sanitize_tol(arc_tol);
    let mut scene = empty_scene();
    let mut bounds = BoundsAcc::new();
    let mut z_range = ZRangeAcc::new();
    let mut cursor: Option<Placed> = None;
    let mut samples = Vec::new();

    for (toolpath, path) in doc.program().toolpaths().iter().enumerate() {
        scene.names.push(path.name().unwrap_or("").to_string());
        let mut labeled = false;
        for block in path.blocks() {
            match block {
                ClBlock::Motion(motion) => {
                    let Some(end) = motion_tip(motion) else {
                        continue;
                    };
                    let end_angle = motion_angle(motion);
                    let Some(start) = cursor else {
                        if let Some(angle) = end_angle {
                            samples.push(RotarySample { start_deg: angle, end_deg: angle, rapid: true });
                        }
                        cursor = Some(Placed { at: end, angle: end_angle });
                        continue;
                    };
                    if let Some(angle) = end_angle {
                        let start_deg = start.angle.filter(|value| value.is_finite()).unwrap_or(angle);
                        let rapid = matches!(motion, ClMotion::Rapid { .. });
                        samples.push(RotarySample { start_deg, end_deg: angle, rapid });
                    }
                    push_stroke(&mut scene, &mut bounds, &mut z_range, &mut labeled, toolpath, motion, start.at, end, start.angle, end_angle, tol);
                    cursor = Some(Placed { at: end, angle: end_angle });
                }
                ClBlock::Event(event) => record_event(&mut scene, toolpath, event, cursor.as_ref().map(|placed| placed.at)),
            }
        }
    }

    scene.bounds = bounds.finish();
    scene.z_range = z_range.finish();
    scene.labels = merge_labels(std::mem::take(&mut scene.labels), LABEL_MERGE_MM);
    if let Some(setup) = doc.program().rotary().copied() {
        scene.rotary = Some(rotary_preview(&setup, scene.bounds, &samples));
    }
    scene
}

/// Join labels that land on the same plan point.
///
/// Each label is compared with the labels already kept, in order. When its point is within
/// `radius_mm` of an earlier label, its text is appended with `", "` and that earlier label keeps
/// its position. A label farther than `radius_mm` from every kept label stays on its own, so the
/// order of first occurrences is unchanged.
pub fn merge_labels(labels: Vec<Label>, radius_mm: f64) -> Vec<Label> {
    let mut merged: Vec<Label> = Vec::with_capacity(labels.len());
    for label in labels {
        let mut joined = false;
        for earlier in &mut merged {
            let distance = (earlier.at[0] - label.at[0]).hypot(earlier.at[1] - label.at[1]);
            if distance.is_finite() && distance <= radius_mm {
                earlier.text.push_str(", ");
                earlier.text.push_str(&label.text);
                joined = true;
                break;
            }
        }
        if !joined {
            merged.push(label);
        }
    }
    merged
}

/// Tessellate a circular arc in the plane into a polyline whose chord deviation is at most `tol`.
///
/// `ccw` selects the direction. When `start` and `end` are the same point the sweep is a full
/// turn. The returned polyline begins with `start` and ends with `end` exactly. A degenerate arc
/// (zero radius, non-finite input, or no angular sweep) is the straight chord from `start` to `end`.
///
/// `tol` is the maximum distance from the true arc to a chord, in the same units as the points.
/// A non-positive tolerance is replaced with a small fraction of the radius so the function still
/// returns a bounded polyline.
pub fn tessellate_arc(start: [f64; 2], end: [f64; 2], center: [f64; 2], ccw: bool, tol: f64) -> Vec<[f64; 2]> {
    xy_arc(XyArc { start, end, center, ccw, sweep: None, tol })
}

/// Playback head for a scene, measured in millimetres along [`Scene::total_length`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Playback {
    /// Distance from the start of the program, in millimetres. `total_length` shows every stroke.
    pub position: f64,
    /// Whether [`Playback::step`] advances `position`.
    pub playing: bool,
    /// Advance rate while playing, in millimetres per second.
    pub speed: f64,
}

impl Playback {
    /// Show the whole path, paused. Speed is `max(total / 10, 1)` millimetres per second, so a
    /// ten-second preview never runs slower than 1 mm/s.
    ///
    /// A non-finite or negative `total` is treated as zero.
    pub fn new(total: f64) -> Self {
        let total = finite_nonneg(total);
        Self { position: total, playing: false, speed: (total / 10.0).max(1.0) }
    }

    /// Advance by `speed * dt_s` while playing, then clamp to `[0, total]`.
    ///
    /// Reaching `total` stops playback. A non-positive or non-finite `dt_s` does not move the head.
    /// When playback is stopped the position is still clamped into range.
    pub fn step(&mut self, dt_s: f64, total: f64) {
        let total = finite_nonneg(total);
        if !self.position.is_finite() || self.position < 0.0 {
            self.position = 0.0;
        }
        if self.position >= total {
            self.position = total;
            self.playing = false;
            return;
        }
        if !self.playing || !dt_s.is_finite() || dt_s <= 0.0 || !self.speed.is_finite() || self.speed <= 0.0 {
            return;
        }
        self.position += self.speed * dt_s;
        if !self.position.is_finite() || self.position >= total {
            self.position = total;
            self.playing = false;
        }
    }

    /// Jump to the start and begin playing. Speed is left unchanged.
    pub fn restart(&mut self) {
        self.position = 0.0;
        self.playing = true;
    }
}

/// Strokes visible when the head is `position` millimetres along the scene.
///
/// Strokes whose cumulative length ends at or before `position` are copied whole. The next stroke,
/// if the head sits inside it, is cut so its polyline ends at that exact distance. A position past
/// the end shows every stroke. A position at or before zero shows nothing (the tool sits on the
/// first point; see [`tool_position`]).
pub fn visible(scene: &Scene, position: f64) -> Vec<Stroke> {
    let pos = finite_nonneg(position);
    let mut out = Vec::new();
    let mut cum = 0.0;
    for stroke in &scene.strokes {
        let len = positive_len(stroke.length);
        if cum + len <= pos + LEN_EPS_MM {
            out.push(stroke.clone());
            cum += len;
            continue;
        }
        let dist = pos - cum;
        if dist > LEN_EPS_MM {
            out.push(cut_stroke(stroke, dist));
        }
        break;
    }
    out
}

/// Plan position of the tool at `position` millimetres along the scene.
///
/// Returns `None` when the scene has no stroke points. A position at or before zero is the first
/// point. A position at or beyond the end is the last point. In between, the point is interpolated
/// along the stroke polyline in 3D and then dropped to XY.
pub fn tool_position(scene: &Scene, position: f64) -> Option<[f64; 2]> {
    let pos = finite_nonneg(position);
    let mut cum = 0.0;
    let mut last_point: Option<[f64; 2]> = None;
    for stroke in &scene.strokes {
        if pos <= cum + LEN_EPS_MM {
            if let Some(point) = stroke.points.first().copied() {
                return Some(point);
            }
            continue;
        }
        let len = positive_len(stroke.length);
        if pos < cum + len {
            let cut = cut_stroke(stroke, pos - cum);
            if let Some(point) = cut.points.last().copied() {
                return Some(point);
            }
        }
        cum += len;
        if let Some(point) = stroke.points.last().copied() {
            last_point = Some(point);
        }
    }
    last_point
}

/// How a tool point is drawn.
///
/// [`Projection::Top`] is the plan view. [`Projection::Iso`] is a standard isometric view with Y up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Projection {
    /// Plan view. The point is returned unchanged and Z is ignored.
    Top,
    /// Isometric view: `x' = (x − y) · cos(30°)`, `y' = (x + y) · sin(30°) + z`.
    Iso,
}

/// Project one tool point into the preview plane.
///
/// [`Projection::Top`] returns `p`. [`Projection::Iso`] uses
/// `x' = (x − y) · cos(30°)` and `y' = (x + y) · sin(30°) + z`. Y stays up.
pub fn project(p: [f64; 2], z: f64, projection: Projection) -> [f64; 2] {
    match projection {
        Projection::Top => p,
        Projection::Iso => {
            let cos30 = std::f64::consts::FRAC_PI_6.cos();
            let sin30 = std::f64::consts::FRAC_PI_6.sin();
            [(p[0] - p[1]) * cos30, (p[0] + p[1]) * sin30 + z]
        }
    }
}

/// Axis-aligned bounds of every stroke point after [`project`].
///
/// Rapids are included, matching [`Scene::bounds`]. `None` when the scene has no finite projected point.
pub fn projected_bounds(scene: &Scene, projection: Projection) -> Option<([f64; 2], [f64; 2])> {
    let mut bounds = BoundsAcc::new();
    for stroke in &scene.strokes {
        for (index, point) in stroke.points.iter().enumerate() {
            let z = stroke.zs.get(index).copied().filter(|value| value.is_finite()).unwrap_or(stroke.z_start);
            bounds.include(project(*point, z, projection));
        }
    }
    if let Some((start, end)) = scene.rotary.as_ref().and_then(|rotary| rotary.line) {
        bounds.include(project([start[0], start[1]], start[2], projection));
        bounds.include(project([end[0], end[1]], end[2], projection));
    }
    bounds.finish()
}

/// Blue (low) through green and yellow to red (high), clamped to `range`.
///
/// `range` is `(low, high)` in millimetres. A zero-width or non-finite range returns the flat cut
/// colour `(90, 200, 250)`.
pub fn height_color(z: f64, range: (f64, f64)) -> [u8; 3] {
    let (low, high) = if range.0 <= range.1 { (range.0, range.1) } else { (range.1, range.0) };
    if !z.is_finite() || !low.is_finite() || !high.is_finite() {
        return FLAT_CUT_RGB;
    }
    let span = high - low;
    if !(span > 0.0) {
        return FLAT_CUT_RGB;
    }
    let t = ((z - low) / span).clamp(0.0, 1.0);
    if t <= 0.0 {
        return [0, 0, 255];
    }
    if t >= 1.0 {
        return [255, 0, 0];
    }
    let (red, green, blue) = if t < (1.0 / 3.0) {
        let along = t / (1.0 / 3.0);
        (0.0, 255.0 * along, 255.0 * (1.0 - along))
    } else if t < (2.0 / 3.0) {
        let along = (t - 1.0 / 3.0) / (1.0 / 3.0);
        (255.0 * along, 255.0, 0.0)
    } else {
        let along = ((t - 2.0 / 3.0) / (1.0 / 3.0)).clamp(0.0, 1.0);
        (255.0, 255.0 * (1.0 - along), 0.0)
    };
    [colour_byte(red), colour_byte(green), colour_byte(blue)]
}

fn colour_byte(value: f64) -> u8 {
    if !value.is_finite() || value <= 0.0 {
        0
    } else if value >= 255.0 {
        255
    } else {
        value.round() as u8
    }
}

/// Blue (low angle) through green and yellow to red (high angle), clamped to `range`.
///
/// `range` is `(low_deg, high_deg)`. A zero-width or non-finite range returns the flat cut colour
/// `(90, 200, 250)`, the same rule as [`height_color`].
pub fn angle_color(angle_deg: f64, range: (f64, f64)) -> [u8; 3] {
    height_color(angle_deg, range)
}

/// Legend colours and end labels for [`angle_color`].
///
/// `steps` is the number of swatches from the low angle to the high angle, inclusive. A zero-width
/// range, a non-finite range, or `steps == 0` returns `None` (there is nothing to show). `steps`
/// is capped at 32.
pub fn angle_legend(range: (f64, f64), steps: usize) -> Option<AngleLegend> {
    let (low, high) = if range.0 <= range.1 { (range.0, range.1) } else { (range.1, range.0) };
    if !low.is_finite() || !high.is_finite() || !(high > low) || steps == 0 {
        return None;
    }
    let steps = steps.min(32);
    let mut colours = Vec::with_capacity(steps);
    let last = (steps - 1) as f64;
    for index in 0..steps {
        let t = if steps == 1 { 0.0 } else { index as f64 / last };
        colours.push(angle_color(low + (high - low) * t, (low, high)));
    }
    Some(AngleLegend { low_label: format_angle_deg(low), high_label: format_angle_deg(high), colours })
}

/// Format one angle for the overlay and the legend. Whole degrees drop the fraction (`"90°"`).
pub fn format_angle_deg(deg: f64) -> String {
    if !deg.is_finite() {
        return "—".to_string();
    }
    let rounded = deg.round();
    if (deg - rounded).abs() <= 1e-6 && rounded.abs() <= 1.0e9 {
        let whole = if rounded == 0.0 { 0.0 } else { rounded };
        return format!("{whole:.0}°");
    }
    let mut text = format!("{deg:.4}");
    while text.contains('.') && (text.ends_with('0') || text.ends_with('.')) {
        text.pop();
    }
    format!("{text}°")
}

/// Overlay text for index angles or a continuous range.
///
/// Indexed: `"Index: 0°, 90°"`. Continuous: `"Angle: 0°..180°"`.
pub fn rotary_overlay(span: &RotarySpan) -> String {
    match span {
        RotarySpan::Indexed(angles) => {
            let body = angles.iter().copied().map(format_angle_deg).collect::<Vec<_>>().join(", ");
            format!("Index: {body}")
        }
        RotarySpan::Continuous { min_deg, max_deg } => format!("Angle: {}..{}", format_angle_deg(*min_deg), format_angle_deg(*max_deg)),
    }
}

/// Classify rotary samples as an index list or a continuous range.
///
/// A feed whose ends differ by more than [`INDEX_ANGLE_EPS_DEG`] is continuous, and the range is
/// the minimum and maximum of every finite sample. Otherwise the distinct angles, sorted
/// ascending, are the index list. Rapids may jump between indexes. Non-finite samples are left
/// out. `None` when nothing finite remains.
pub fn rotary_span(samples: &[RotarySample]) -> Option<RotarySpan> {
    let mut values = Vec::new();
    let mut continuous = false;
    for sample in samples {
        if !sample.start_deg.is_finite() || !sample.end_deg.is_finite() {
            continue;
        }
        if !sample.rapid && (sample.end_deg - sample.start_deg).abs() > INDEX_ANGLE_EPS_DEG {
            continuous = true;
        }
        values.push(sample.start_deg);
        values.push(sample.end_deg);
    }
    if values.is_empty() {
        return None;
    }
    if continuous {
        let min_deg = values.iter().copied().fold(f64::INFINITY, f64::min);
        let max_deg = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        return Some(RotarySpan::Continuous { min_deg, max_deg });
    }
    values.sort_by(f64::total_cmp);
    let mut distinct = Vec::new();
    for value in values {
        let value = if value == 0.0 { 0.0 } else { value };
        if distinct.last().is_none_or(|prev: &f64| (value - *prev).abs() > INDEX_ANGLE_EPS_DEG) {
            distinct.push(value);
        }
    }
    Some(RotarySpan::Indexed(distinct))
}

/// Endpoints of the rotary axis through `center`, across the toolpath extent.
///
/// [`RotaryAxisKind::A`] runs along part +X at the centre's Y and Z. [`RotaryAxisKind::B`] runs
/// along part +Y at the centre's X and Z. The segment includes the centre, so a centre outside
/// the toolpath still lies on the line, and it covers the toolpath's extent on that axis.
/// `bounds` is `(min_xy, max_xy)` in millimetres. `None` when `center` or `bounds` is not finite.
pub fn rotary_axis_endpoints(kind: RotaryAxisKind, center: [f64; 3], bounds: ([f64; 2], [f64; 2])) -> Option<([f64; 3], [f64; 3])> {
    if !center.iter().all(|value| value.is_finite()) {
        return None;
    }
    let (min_xy, max_xy) = bounds;
    if ![min_xy[0], min_xy[1], max_xy[0], max_xy[1]].into_iter().all(f64::is_finite) {
        return None;
    }
    let (start, end) = match kind {
        RotaryAxisKind::A => {
            let lo = min_xy[0].min(center[0]);
            let hi = max_xy[0].max(center[0]);
            ([lo, center[1], center[2]], [hi, center[1], center[2]])
        }
        RotaryAxisKind::B => {
            let lo = min_xy[1].min(center[1]);
            let hi = max_xy[1].max(center[1]);
            ([center[0], lo, center[2]], [center[0], hi, center[2]])
        }
    };
    Some((start, end))
}

/// World-to-screen transform for the plan view.
///
/// `center` is the world point (millimetres, Y up) mapped to the middle of the viewport.
/// `scale` is pixels per millimetre. Screen Y grows downward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    /// World point kept at the centre of the viewport, in millimetres.
    pub center: [f64; 2],
    /// Pixels per millimetre. Reads and writes through the view methods stay inside `[1e-4, 1e6]`.
    pub scale: f64,
}

impl View {
    /// Frame `bounds` `(min_xy, max_xy)` inside a viewport, leaving `margin_px` on every side.
    ///
    /// The fit uses the tighter of the two axes. A zero-area bounds keeps a scale of 1 and centres
    /// on the point. Non-finite inputs produce a unit view at the origin.
    pub fn fit(bounds: ([f64; 2], [f64; 2]), viewport_w: f64, viewport_h: f64, margin_px: f64) -> Self {
        let min_x = bounds.0[0].min(bounds.1[0]);
        let min_y = bounds.0[1].min(bounds.1[1]);
        let max_x = bounds.0[0].max(bounds.1[0]);
        let max_y = bounds.0[1].max(bounds.1[1]);
        let finite = [min_x, min_y, max_x, max_y, viewport_w, viewport_h, margin_px].into_iter().all(f64::is_finite);
        if !finite || viewport_w <= 0.0 || viewport_h <= 0.0 {
            return Self { center: [0.0, 0.0], scale: 1.0 };
        }
        let center = [(min_x + max_x) * 0.5, (min_y + max_y) * 0.5];
        let width = max_x - min_x;
        let height = max_y - min_y;
        let margin = margin_px.max(0.0);
        let avail_w = (viewport_w - 2.0 * margin).max(1.0);
        let avail_h = (viewport_h - 2.0 * margin).max(1.0);
        let scale = if width <= XY_CHANGE_MM && height <= XY_CHANGE_MM {
            1.0
        } else if width <= XY_CHANGE_MM {
            avail_h / height
        } else if height <= XY_CHANGE_MM {
            avail_w / width
        } else {
            (avail_w / width).min(avail_h / height)
        };
        Self { center, scale: clamp_scale(scale) }
    }

    /// Map a world point to viewport pixels. `(vw, vh)` is the viewport size in pixels.
    pub fn to_screen(&self, p: [f64; 2], vw: f64, vh: f64) -> [f64; 2] {
        if !p[0].is_finite() || !p[1].is_finite() || !vw.is_finite() || !vh.is_finite() {
            return [0.0, 0.0];
        }
        let scale = self.effective_scale();
        let center = finite_center(self.center);
        [vw * 0.5 + (p[0] - center[0]) * scale, vh * 0.5 - (p[1] - center[1]) * scale]
    }

    /// Inverse of [`View::to_screen`].
    pub fn to_world(&self, p: [f64; 2], vw: f64, vh: f64) -> [f64; 2] {
        if !p[0].is_finite() || !p[1].is_finite() || !vw.is_finite() || !vh.is_finite() {
            return finite_center(self.center);
        }
        let scale = self.effective_scale();
        let center = finite_center(self.center);
        [center[0] + (p[0] - vw * 0.5) / scale, center[1] - (p[1] - vh * 0.5) / scale]
    }

    /// Multiply `scale` by `factor`, keeping the world point under `screen` fixed.
    ///
    /// `factor` must be finite and positive; otherwise the view is unchanged. The resulting scale
    /// is clamped to `[1e-4, 1e6]`.
    pub fn zoom_at(&mut self, factor: f64, screen: [f64; 2], vw: f64, vh: f64) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        let before = self.to_world(screen, vw, vh);
        self.scale = clamp_scale(self.effective_scale() * factor);
        let after = self.to_world(screen, vw, vh);
        self.center = [self.center[0] + (before[0] - after[0]), self.center[1] + (before[1] - after[1])];
        self.center = finite_center(self.center);
    }

    /// Slide the scene by a screen delta. Positive `dx_px` moves content right; positive `dy_px`
    /// moves content down. Scale is unchanged.
    pub fn pan(&mut self, dx_px: f64, dy_px: f64) {
        if !dx_px.is_finite() || !dy_px.is_finite() {
            return;
        }
        self.scale = self.effective_scale();
        self.center[0] -= dx_px / self.scale;
        self.center[1] += dy_px / self.scale;
        self.center = finite_center(self.center);
    }

    fn effective_scale(&self) -> f64 {
        clamp_scale(self.scale)
    }
}

/// First existing font file from `CAMPANION_VIEW_FONT` or a fixed list of common sans faces.
///
/// The environment variable wins when it is set and names a file that exists. Otherwise the search
/// is DejaVu Sans, Liberation Sans, and Noto Sans on Linux, Arial on macOS, then Arial on Windows.
/// Targets without a filesystem return `None`.
pub fn find_system_font() -> Option<std::path::PathBuf> {
    if let Ok(value) = std::env::var("CAMPANION_VIEW_FONT") {
        let path = std::path::PathBuf::from(&value);
        if !value.is_empty() && path.is_file() {
            return Some(path);
        }
    }
    const CANDIDATES: &[&str] = &[
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "C:\\Windows\\Fonts\\arial.ttf",
    ];
    for candidate in CANDIDATES {
        let path = std::path::PathBuf::from(candidate);
        if path.is_file() {
            return Some(path);
        }
    }
    None
}

struct XyArc {
    start: [f64; 2],
    end: [f64; 2],
    center: [f64; 2],
    ccw: bool,
    /// `None` derives the sweep from the endpoints. `Some` is the total angle in radians.
    sweep: Option<f64>,
    tol: f64,
}

struct ZRangeAcc {
    min: f64,
    max: f64,
    any: bool,
}

impl ZRangeAcc {
    fn new() -> Self {
        Self { min: 0.0, max: 0.0, any: false }
    }

    fn include(&mut self, z: f64) {
        if !z.is_finite() {
            return;
        }
        if !self.any {
            self.min = z;
            self.max = z;
            self.any = true;
            return;
        }
        self.min = self.min.min(z);
        self.max = self.max.max(z);
    }

    fn finish(self) -> Option<(f64, f64)> {
        if self.any { Some((self.min, self.max)) } else { None }
    }
}

struct BoundsAcc {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    any: bool,
}

impl BoundsAcc {
    fn new() -> Self {
        Self { min_x: 0.0, min_y: 0.0, max_x: 0.0, max_y: 0.0, any: false }
    }

    fn include(&mut self, point: [f64; 2]) {
        if !point[0].is_finite() || !point[1].is_finite() {
            return;
        }
        if !self.any {
            self.min_x = point[0];
            self.max_x = point[0];
            self.min_y = point[1];
            self.max_y = point[1];
            self.any = true;
            return;
        }
        self.min_x = self.min_x.min(point[0]);
        self.max_x = self.max_x.max(point[0]);
        self.min_y = self.min_y.min(point[1]);
        self.max_y = self.max_y.max(point[1]);
    }

    fn finish(self) -> Option<([f64; 2], [f64; 2])> {
        if self.any { Some(([self.min_x, self.min_y], [self.max_x, self.max_y])) } else { None }
    }
}

fn empty_scene() -> Scene {
    Scene {
        strokes: Vec::new(),
        pierces: Vec::new(),
        labels: Vec::new(),
        bounds: None,
        z_range: None,
        total_length: 0.0,
        names: Vec::new(),
        rotary: None,
    }
}

/// Tool tip and rotary angle after one motion. The angle is `None` on a 3-axis pose.
struct Placed {
    at: [f64; 3],
    angle: Option<f64>,
}

fn rotary_preview(setup: &RotarySetup, bounds: Option<([f64; 2], [f64; 2])>, samples: &[RotarySample]) -> RotaryPreview {
    let center_v = setup.center();
    let center = [center_v.x(), center_v.y(), center_v.z()];
    let label = match setup.axis() {
        RotaryAxisKind::A => "A",
        RotaryAxisKind::B => "B",
    };
    let line = bounds.and_then(|bounds| rotary_axis_endpoints(setup.axis(), center, bounds));
    RotaryPreview { label, center, line, span: rotary_span(samples) }
}

fn push_stroke(
    scene: &mut Scene,
    bounds: &mut BoundsAcc,
    z_range: &mut ZRangeAcc,
    labeled: &mut bool,
    toolpath: usize,
    motion: &ClMotion,
    start: [f64; 3],
    end: [f64; 3],
    start_angle: Option<f64>,
    end_angle: Option<f64>,
    tol: f64,
) {
    let stroke = stroke_from_motion(toolpath, motion, start, end, start_angle, end_angle, tol);
    if stroke.points.is_empty() {
        return;
    }
    if stroke.kind != StrokeKind::Rapid
        && !*labeled
        && let Some(at) = stroke.points.first().copied()
    {
        scene.labels.push(Label { at, text: (toolpath + 1).to_string() });
        *labeled = true;
    }
    for point in &stroke.points {
        bounds.include(*point);
    }
    if stroke.kind != StrokeKind::Rapid {
        if stroke.zs.is_empty() {
            z_range.include(stroke.z_start);
            z_range.include(stroke.z_end);
        } else {
            for z in &stroke.zs {
                z_range.include(*z);
            }
        }
    }
    if stroke.length.is_finite() {
        scene.total_length += stroke.length;
    }
    scene.strokes.push(stroke);
}

fn record_event(scene: &mut Scene, toolpath: usize, event: &Event, cursor: Option<[f64; 3]>) {
    match event {
        Event::ProcessOn { .. } => {
            let Some(cursor) = cursor else {
                return;
            };
            let at = [cursor[0], cursor[1]];
            if !at[0].is_finite() || !at[1].is_finite() {
                return;
            }
            scene.pierces.push(Marker { at, toolpath, stroke_index: scene.strokes.len() });
        }
        Event::ProcessOff { .. } | Event::Dwell { .. } => {}
        _ => {}
    }
}

fn motion_pose(motion: &ClMotion) -> &campanion_ir::Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => to,
    }
}

fn motion_tip(motion: &ClMotion) -> Option<[f64; 3]> {
    let tip = motion_pose(motion).tip();
    let point = [tip.x(), tip.y(), tip.z()];
    if point.iter().all(|value| value.is_finite()) { Some(point) } else { None }
}

fn motion_angle(motion: &ClMotion) -> Option<f64> {
    motion_pose(motion).rotary().filter(|deg| deg.is_finite())
}

fn stroke_from_motion(
    toolpath: usize,
    motion: &ClMotion,
    start: [f64; 3],
    end: [f64; 3],
    start_angle: Option<f64>,
    end_angle: Option<f64>,
    tol: f64,
) -> Stroke {
    let start_xy = [start[0], start[1]];
    let end_xy = [end[0], end[1]];
    let (points, rapid) = match motion {
        ClMotion::Rapid { .. } => (vec![start_xy, end_xy], true),
        ClMotion::Linear { .. } | ClMotion::Thread { .. } => (vec![start_xy, end_xy], false),
        ClMotion::Arc { plane, center, sense, .. } => {
            let points = plan_curve(start_xy, end_xy, center, sense, plane, None, tol);
            (points, false)
        }
        ClMotion::Helix { plane, center, sense, sweep, .. } => {
            let points = plan_curve(start_xy, end_xy, center, sense, plane, Some(*sweep), tol);
            (points, false)
        }
    };
    let zs = heights_for(motion, &points, start[2], end[2]);
    let angles = match end_angle.filter(|angle| angle.is_finite()) {
        Some(end_angle) => {
            let start_angle = start_angle.filter(|angle| angle.is_finite()).unwrap_or(end_angle);
            heights_for(motion, &points, start_angle, end_angle)
        }
        None => Vec::new(),
    };
    let length = polyline_length_zs(&points, &zs);
    Stroke { kind: stroke_kind(rapid, &points), toolpath, points, zs, z_start: start[2], z_end: end[2], angles, length }
}

/// Heights for every plan point. Lines keep the real endpoint Z. Arcs and helices follow XY arc length.
fn heights_for(motion: &ClMotion, points: &[[f64; 2]], z_start: f64, z_end: f64) -> Vec<f64> {
    match motion {
        ClMotion::Rapid { .. } | ClMotion::Linear { .. } | ClMotion::Thread { .. } => endpoint_heights(points, z_start, z_end),
        ClMotion::Arc { .. } | ClMotion::Helix { .. } => arc_length_heights(points, z_start, z_end),
    }
}

/// Exact endpoint Z. A two-point line is `[z_start, z_end]`. A lone point keeps the start height.
fn endpoint_heights(points: &[[f64; 2]], z_start: f64, z_end: f64) -> Vec<f64> {
    let z_start = finite_z(z_start, 0.0);
    let z_end = finite_z(z_end, z_start);
    match points.len() {
        0 => Vec::new(),
        1 => vec![z_start],
        2 => vec![z_start, z_end],
        _ => arc_length_heights(points, z_start, z_end),
    }
}

/// Interpolate Z by cumulative XY length, which is arc length for a uniformly sampled circle or helix.
///
/// A path with no XY travel (a degenerate arc) falls back to an even split so Z still reaches `z_end`.
fn arc_length_heights(points: &[[f64; 2]], z_start: f64, z_end: f64) -> Vec<f64> {
    let count = points.len();
    let z_start = finite_z(z_start, 0.0);
    let z_end = finite_z(z_end, z_start);
    if count == 0 {
        return Vec::new();
    }
    if count == 1 {
        return vec![z_start];
    }
    let mut cumulative = Vec::with_capacity(count);
    cumulative.push(0.0);
    let mut total = 0.0;
    for index in 1..count {
        let (Some(prev), Some(point)) = (points.get(index - 1), points.get(index)) else {
            break;
        };
        let step = (point[0] - prev[0]).hypot(point[1] - prev[1]);
        if step.is_finite() {
            total += step;
        }
        cumulative.push(total);
    }
    if cumulative.len() != count || total <= LEN_EPS_MM {
        let steps = (count - 1) as f64;
        return (0..count).map(|index| lerp(z_start, z_end, index as f64 / steps)).collect();
    }
    cumulative.into_iter().map(|distance| lerp(z_start, z_end, distance / total)).collect()
}

fn plan_curve(
    start: [f64; 2],
    end: [f64; 2],
    center: &campanion_ir::Vec3,
    sense: &ArcSense,
    plane: &Plane,
    sweep: Option<f64>,
    tol: f64,
) -> Vec<[f64; 2]> {
    let center_xy = [center.x(), center.y()];
    let xy_plane = matches!(plane, Plane::Xy);
    if !xy_plane || !center_xy[0].is_finite() || !center_xy[1].is_finite() {
        return vec![start, end];
    }
    xy_arc(XyArc { start, end, center: center_xy, ccw: matches!(sense, ArcSense::Ccw), sweep, tol })
}

fn stroke_kind(rapid: bool, points: &[[f64; 2]]) -> StrokeKind {
    if rapid {
        StrokeKind::Rapid
    } else if xy_unchanged(points) {
        StrokeKind::Plunge
    } else {
        StrokeKind::Cut
    }
}

fn xy_unchanged(points: &[[f64; 2]]) -> bool {
    let Some(first) = points.first() else {
        return true;
    };
    points.iter().all(|point| (point[0] - first[0]).hypot(point[1] - first[1]) <= XY_CHANGE_MM)
}

fn xy_arc(arc: XyArc) -> Vec<[f64; 2]> {
    if !is_finite2(arc.start) || !is_finite2(arc.end) || !is_finite2(arc.center) {
        return finite_points(&[arc.start, arc.end]);
    }
    let radial_x = arc.start[0] - arc.center[0];
    let radial_y = arc.start[1] - arc.center[1];
    let radius = radial_x.hypot(radial_y);
    if !radius.is_finite() || radius <= XY_CHANGE_MM {
        return vec![arc.start, arc.end];
    }
    let start_angle = radial_y.atan2(radial_x);
    if let Some(sweep) = arc.sweep {
        let sweep = sweep.abs();
        if !sweep.is_finite() || sweep <= MIN_SWEEP_RAD {
            return vec![arc.start, arc.end];
        }
        return sample_arc(&arc, radius, start_angle, sweep);
    }
    if same_point(arc.start, arc.end) {
        return sample_arc(&arc, radius, start_angle, std::f64::consts::TAU);
    }
    let end_radial_x = arc.end[0] - arc.center[0];
    let end_radial_y = arc.end[1] - arc.center[1];
    let end_angle = end_radial_y.atan2(end_radial_x);
    // Same ray (a radial step, or atan2 noise) is a chord. A real arc has a measurable sweep,
    // and an almost-full turn stays large on the short-angle test below.
    if shortest_angle(start_angle, end_angle) <= MIN_SWEEP_RAD {
        return vec![arc.start, arc.end];
    }
    let mut sweep = if arc.ccw { end_angle - start_angle } else { start_angle - end_angle };
    if sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    if sweep <= MIN_SWEEP_RAD {
        return vec![arc.start, arc.end];
    }
    sample_arc(&arc, radius, start_angle, sweep)
}

fn sample_arc(arc: &XyArc, radius: f64, start_angle: f64, sweep: f64) -> Vec<[f64; 2]> {
    let segments = segment_count(sweep, radius, arc.tol);
    let mut points = Vec::with_capacity(segments + 1);
    points.push(arc.start);
    let sign = if arc.ccw { 1.0 } else { -1.0 };
    let steps = segments as f64;
    for index in 1..segments {
        let angle = start_angle + sign * sweep * (index as f64) / steps;
        let point = [arc.center[0] + radius * angle.cos(), arc.center[1] + radius * angle.sin()];
        if is_finite2(point) {
            points.push(point);
        }
    }
    // The tip is exact even when it is a hair off the sampled circle.
    points.push(arc.end);
    points
}

fn segment_count(sweep: f64, radius: f64, tol: f64) -> usize {
    if !sweep.is_finite() || sweep <= 0.0 || !radius.is_finite() || radius <= 0.0 {
        return 1;
    }
    let max_step = max_segment_angle(radius, tol);
    if !max_step.is_finite() || max_step <= MIN_SWEEP_RAD {
        return MAX_ARC_SEGMENTS;
    }
    let count = (sweep / max_step).ceil();
    if !count.is_finite() || count <= 1.0 {
        1
    } else if count >= MAX_ARC_SEGMENTS as f64 {
        MAX_ARC_SEGMENTS
    } else {
        // `count` is in (1, MAX_ARC_SEGMENTS), so the conversion is exact.
        count as usize
    }
}

/// Largest sweep whose sagitta is within `tol`: `2 * acos(1 - tol / radius)`.
fn max_segment_angle(radius: f64, tol: f64) -> f64 {
    let tol = if tol.is_finite() && tol > 0.0 { tol } else { radius * 1e-4 };
    let ratio = tol / radius;
    if !ratio.is_finite() {
        return std::f64::consts::FRAC_PI_8;
    }
    if ratio >= 2.0 {
        return std::f64::consts::TAU;
    }
    // `acos(1 - x)` loses the small angle in the subtraction. The Taylor step is slightly
    // conservative (`1 - cos` is a bit under `x^2 / 2`), so the chord stays inside `tol`.
    if ratio < 1e-8 {
        let theta = 2.0 * (2.0 * ratio).sqrt();
        return if theta.is_finite() && theta > 0.0 { theta } else { MIN_SWEEP_RAD * 10.0 };
    }
    let theta = 2.0 * (1.0 - ratio).clamp(-1.0, 1.0).acos();
    if theta.is_finite() && theta > 0.0 { theta } else { std::f64::consts::FRAC_PI_8 }
}

#[cfg(test)]
fn polyline_length_3d(points: &[[f64; 2]], z_start: f64, z_end: f64) -> f64 {
    let segments = points.len().saturating_sub(1);
    if segments == 0 || !z_start.is_finite() || !z_end.is_finite() {
        return 0.0;
    }
    let mut length = 0.0;
    let steps = segments as f64;
    for index in 0..segments {
        let (Some(start), Some(end)) = (points.get(index), points.get(index + 1)) else {
            break;
        };
        let z0 = lerp(z_start, z_end, index as f64 / steps);
        let z1 = lerp(z_start, z_end, (index + 1) as f64 / steps);
        let segment = (end[0] - start[0]).hypot(end[1] - start[1]).hypot(z1 - z0);
        if segment.is_finite() {
            length += segment;
        }
    }
    if length.is_finite() { length } else { 0.0 }
}

/// 3D length using the stored height of each vertex.
fn polyline_length_zs(points: &[[f64; 2]], zs: &[f64]) -> f64 {
    let segments = points.len().saturating_sub(1);
    if segments == 0 {
        return 0.0;
    }
    let mut length = 0.0;
    for index in 0..segments {
        let (Some(start), Some(end)) = (points.get(index), points.get(index + 1)) else {
            break;
        };
        let z0 = zs.get(index).copied().filter(|z| z.is_finite()).unwrap_or(0.0);
        let z1 = zs.get(index + 1).copied().filter(|z| z.is_finite()).unwrap_or(z0);
        let segment = (end[0] - start[0]).hypot(end[1] - start[1]).hypot(z1 - z0);
        if segment.is_finite() {
            length += segment;
        }
    }
    if length.is_finite() { length } else { 0.0 }
}

fn cut_stroke(stroke: &Stroke, dist: f64) -> Stroke {
    let dist = finite_nonneg(dist);
    let z_start = finite_z(stroke.z_start, 0.0);
    let z_end = finite_z(stroke.z_end, z_start);
    let segments = stroke.points.len().saturating_sub(1);
    let track_angles = stroke.angles.len() == stroke.points.len();
    let Some(&first) = stroke.points.first() else {
        return cut_piece(stroke, Vec::new(), Vec::new(), Vec::new(), z_start, z_start, 0.0);
    };
    let z_first = stored_z(stroke, 0, segments, z_start, z_end);
    let mut angles = Vec::new();
    if track_angles {
        angles.push(stored_angle(stroke, 0));
    }
    if segments == 0 || dist <= 0.0 {
        return cut_piece(stroke, vec![first], vec![z_first], angles, z_start, z_start, 0.0);
    }
    let mut traveled = 0.0;
    let mut points = vec![first];
    let mut zs = vec![z_first];
    for index in 0..segments {
        let (Some(start), Some(end)) = (stroke.points.get(index), stroke.points.get(index + 1)) else {
            break;
        };
        let z0 = stored_z(stroke, index, segments, z_start, z_end);
        let z1 = stored_z(stroke, index + 1, segments, z_start, z_end);
        let delta_x = end[0] - start[0];
        let delta_y = end[1] - start[1];
        let delta_z = z1 - z0;
        let segment = delta_x.hypot(delta_y).hypot(delta_z);
        if !segment.is_finite() || segment <= 0.0 {
            points.push(*end);
            zs.push(z1);
            if track_angles {
                angles.push(stored_angle(stroke, index + 1));
            }
            continue;
        }
        if traveled + segment + LEN_EPS_MM >= dist {
            let t = ((dist - traveled) / segment).clamp(0.0, 1.0);
            if t > LEN_EPS_MM {
                points.push([start[0] + delta_x * t, start[1] + delta_y * t]);
                zs.push(z0 + delta_z * t);
                if track_angles {
                    angles.push(lerp(stored_angle(stroke, index), stored_angle(stroke, index + 1), t));
                }
            }
            return cut_piece(stroke, points, zs, angles, z_start, z0 + delta_z * t, dist);
        }
        traveled += segment;
        points.push(*end);
        zs.push(z1);
        if track_angles {
            angles.push(stored_angle(stroke, index + 1));
        }
    }
    cut_piece(stroke, points, zs, angles, z_start, z_end, traveled)
}

fn cut_piece(stroke: &Stroke, points: Vec<[f64; 2]>, zs: Vec<f64>, angles: Vec<f64>, z_start: f64, z_end: f64, length: f64) -> Stroke {
    Stroke { kind: stroke.kind, toolpath: stroke.toolpath, points, zs, z_start, z_end, angles, length }
}

fn stored_angle(stroke: &Stroke, index: usize) -> f64 {
    stroke.angles.get(index).copied().filter(|angle| angle.is_finite()).unwrap_or(0.0)
}

/// Stored height at `index`, or the same uniform split `cut_stroke` used before `zs` existed.
fn stored_z(stroke: &Stroke, index: usize, segments: usize, z_start: f64, z_end: f64) -> f64 {
    if let Some(z) = stroke.zs.get(index).copied()
        && z.is_finite()
    {
        return z;
    }
    if segments == 0 { z_start } else { lerp(z_start, z_end, index as f64 / segments as f64) }
}

fn lerp(start: f64, end: f64, t: f64) -> f64 {
    start + (end - start) * t
}

fn sanitize_tol(tol: f64) -> f64 {
    if tol.is_finite() && tol > 0.0 { tol } else { DEFAULT_ARC_TOL_MM }
}

fn finite_nonneg(value: f64) -> f64 {
    if value.is_finite() && value > 0.0 { value } else { 0.0 }
}

fn finite_z(value: f64, fallback: f64) -> f64 {
    if value.is_finite() { value } else { fallback }
}

fn positive_len(length: f64) -> f64 {
    if length.is_finite() && length > 0.0 { length } else { 0.0 }
}

fn clamp_scale(scale: f64) -> f64 {
    if !scale.is_finite() || scale <= 0.0 { 1.0 } else { scale.clamp(1e-4, 1e6) }
}

fn finite_center(center: [f64; 2]) -> [f64; 2] {
    if center[0].is_finite() && center[1].is_finite() { center } else { [0.0, 0.0] }
}

fn is_finite2(point: [f64; 2]) -> bool {
    point[0].is_finite() && point[1].is_finite()
}

fn finite_points(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    points.iter().copied().filter(|point| is_finite2(*point)).collect()
}

fn same_point(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9
}

fn shortest_angle(start: f64, end: f64) -> f64 {
    let mut delta = (end - start).abs();
    if delta > std::f64::consts::PI {
        delta = std::f64::consts::TAU - delta;
    }
    delta
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::float_cmp)]
mod tests {
    use super::{
        Label, Playback, Projection, RotarySample, RotarySpan, Stroke, StrokeKind, View, angle_color, angle_legend, build_scene, find_system_font,
        format_angle_deg, height_color, load_scene, merge_labels, polyline_length_3d, project, projected_bounds, rotary_axis_endpoints,
        rotary_overlay, rotary_span, stroke_kind, tessellate_arc, tool_position, visible,
    };

    fn assert_close(actual: f64, expected: f64) {
        let scale = 1.0 + actual.abs().max(expected.abs());
        assert!((actual - expected).abs() <= 1e-9 * scale, "{actual} != {expected}");
    }

    fn assert_pt(actual: [f64; 2], expected: [f64; 2]) {
        assert_close(actual[0], expected[0]);
        assert_close(actual[1], expected[1]);
    }

    fn dist_point_to_seg(point: [f64; 2], start: [f64; 2], end: [f64; 2]) -> f64 {
        let ab_x = end[0] - start[0];
        let ab_y = end[1] - start[1];
        let ap_x = point[0] - start[0];
        let ap_y = point[1] - start[1];
        let ab_len_sq = ab_x * ab_x + ab_y * ab_y;
        let t = if ab_len_sq <= 1e-30 { 0.0 } else { ((ap_x * ab_x + ap_y * ab_y) / ab_len_sq).clamp(0.0, 1.0) };
        let closest_x = start[0] + ab_x * t;
        let closest_y = start[1] + ab_y * t;
        (point[0] - closest_x).hypot(point[1] - closest_y)
    }

    fn max_arc_deviation(points: &[[f64; 2]], center: [f64; 2], radius: f64, ccw: bool, sweep: f64) -> f64 {
        let start = points[0];
        let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
        let sign = if ccw { 1.0 } else { -1.0 };
        let samples = 64;
        let mut max_dev: f64 = 0.0;
        for step in 0..=samples {
            let angle = start_angle + sign * sweep * (step as f64) / (samples as f64);
            let point = [center[0] + radius * angle.cos(), center[1] + radius * angle.sin()];
            let mut best = f64::MAX;
            for window in points.windows(2) {
                best = best.min(dist_point_to_seg(point, window[0], window[1]));
            }
            max_dev = max_dev.max(best);
        }
        max_dev
    }

    #[test]
    fn view_fit_round_trip_zoom_and_pan() {
        let viewport_w = 200.0;
        let viewport_h = 100.0;
        let view = View::fit(([0.0, 0.0], [100.0, 50.0]), viewport_w, viewport_h, 0.0);
        assert_close(view.scale, 2.0);
        assert_pt(view.center, [50.0, 25.0]);

        let world_points = [[0.0, 0.0], [100.0, 50.0], [10.0, 40.0], [50.0, 25.0], [0.0, 50.0]];
        for point in world_points {
            let screen = view.to_screen(point, viewport_w, viewport_h);
            assert_pt(view.to_world(screen, viewport_w, viewport_h), point);
        }
        let higher = view.to_screen([50.0, 40.0], viewport_w, viewport_h);
        let lower = view.to_screen([50.0, 10.0], viewport_w, viewport_h);
        assert!(higher[1] < lower[1], "world Y up must map to screen Y down");

        let fitted = View::fit(([0.0, 0.0], [100.0, 50.0]), viewport_w, viewport_h, 10.0);
        assert_close(fitted.scale, 1.6);
        let top = fitted.to_screen([50.0, 50.0], viewport_w, viewport_h);
        let bottom = fitted.to_screen([50.0, 0.0], viewport_w, viewport_h);
        assert_close(top[1], 10.0);
        assert_close(bottom[1], 90.0);

        let mut zoomed = view;
        let anchor_screen = [30.0, 70.0];
        let anchor_world = zoomed.to_world(anchor_screen, viewport_w, viewport_h);
        zoomed.zoom_at(2.5, anchor_screen, viewport_w, viewport_h);
        assert_close(zoomed.scale, 5.0);
        assert_pt(zoomed.to_world(anchor_screen, viewport_w, viewport_h), anchor_world);
        zoomed.zoom_at(0.0, anchor_screen, viewport_w, viewport_h);
        assert_close(zoomed.scale, 5.0);

        let before_pan = zoomed.to_screen([12.0, 8.0], viewport_w, viewport_h);
        zoomed.pan(15.0, -4.0);
        let after_pan = zoomed.to_screen([12.0, 8.0], viewport_w, viewport_h);
        assert_close(after_pan[0], before_pan[0] + 15.0);
        assert_close(after_pan[1], before_pan[1] - 4.0);
        assert_pt(zoomed.to_world(zoomed.to_screen([3.0, 4.0], viewport_w, viewport_h), viewport_w, viewport_h), [3.0, 4.0]);
    }

    #[test]
    fn view_scale_is_clamped_and_anchor_stays_put() {
        let viewport_w = 200.0;
        let viewport_h = 150.0;
        let mut view = View { center: [5.0, -3.0], scale: 1.0e5 };
        let screen = [20.0, 80.0];
        let before = view.to_world(screen, viewport_w, viewport_h);
        view.zoom_at(100.0, screen, viewport_w, viewport_h);
        assert_close(view.scale, 1.0e6);
        assert_pt(view.to_world(screen, viewport_w, viewport_h), before);

        view.scale = 1.0e-3;
        let before = view.to_world(screen, viewport_w, viewport_h);
        view.zoom_at(0.01, screen, viewport_w, viewport_h);
        assert_close(view.scale, 1.0e-4);
        assert_pt(view.to_world(screen, viewport_w, viewport_h), before);
    }

    #[test]
    fn playback_steps_clamps_and_restarts() {
        let mut playback = Playback::new(100.0);
        assert!(!playback.playing);
        assert_close(playback.position, 100.0);
        assert_close(playback.speed, 10.0);
        playback.step(1.0, 100.0);
        assert_close(playback.position, 100.0);
        assert!(!playback.playing);

        playback.restart();
        assert!(playback.playing);
        assert_close(playback.position, 0.0);
        assert_close(playback.speed, 10.0);
        playback.step(0.5, 100.0);
        assert!(playback.playing);
        assert_close(playback.position, 5.0);
        playback.step(100.0, 100.0);
        assert!(!playback.playing);
        assert_close(playback.position, 100.0);
        playback.step(1.0, 100.0);
        assert_close(playback.position, 100.0);

        let slow = Playback::new(5.0);
        assert_close(slow.position, 5.0);
        assert_close(slow.speed, 1.0);
        let empty = Playback::new(f64::NAN);
        assert_close(empty.position, 0.0);
        assert_close(empty.speed, 1.0);
    }

    fn line_scene() -> super::Scene {
        super::Scene {
            strokes: vec![
                Stroke {
                    kind: StrokeKind::Cut,
                    toolpath: 0,
                    points: vec![[0.0, 0.0], [10.0, 0.0]],
                    zs: vec![0.0, 0.0],
                    z_start: 0.0,
                    z_end: 0.0,
                    angles: Vec::new(),
                    length: 10.0,
                },
                Stroke {
                    kind: StrokeKind::Cut,
                    toolpath: 0,
                    points: vec![[10.0, 0.0], [10.0, 10.0]],
                    zs: vec![0.0, 0.0],
                    z_start: 0.0,
                    z_end: 0.0,
                    angles: Vec::new(),
                    length: 10.0,
                },
            ],
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: Some(([0.0, 0.0], [10.0, 10.0])),
            z_range: Some((0.0, 0.0)),
            total_length: 20.0,
            names: vec!["path".to_string()],
            rotary: None,
        }
    }

    #[test]
    fn visible_cuts_a_partial_stroke_and_reports_the_tool() {
        let scene = line_scene();
        assert!(visible(&scene, 0.0).is_empty());
        assert_pt(tool_position(&scene, 0.0).unwrap(), [0.0, 0.0]);

        let shown = visible(&scene, 15.0);
        assert_eq!(shown.len(), 2);
        assert_close(shown[0].length, 10.0);
        assert_eq!(shown[0].points, vec![[0.0, 0.0], [10.0, 0.0]]);
        assert_close(shown[1].length, 5.0);
        assert_pt(*shown[1].points.first().unwrap(), [10.0, 0.0]);
        assert_pt(*shown[1].points.last().unwrap(), [10.0, 5.0]);
        assert_close(shown[1].z_start, 0.0);
        assert_close(shown[1].z_end, 0.0);
        assert_pt(tool_position(&scene, 15.0).unwrap(), [10.0, 5.0]);
        assert_pt(*shown.last().unwrap().points.last().unwrap(), tool_position(&scene, 15.0).unwrap());

        let all = visible(&scene, 20.0);
        assert_eq!(all.len(), 2);
        assert_close(all[1].length, 10.0);
        assert_pt(tool_position(&scene, 20.0).unwrap(), [10.0, 10.0]);
        assert_pt(tool_position(&scene, 500.0).unwrap(), [10.0, 10.0]);
        assert!(tool_position(&super::Scene { total_length: 0.0, ..line_scene() }, 0.0).is_some());
    }

    #[test]
    fn visible_interpolates_polyline_vertices_and_z() {
        let bend = super::Scene {
            strokes: vec![Stroke {
                kind: StrokeKind::Cut,
                toolpath: 1,
                points: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]],
                zs: vec![0.0, 0.0, 0.0],
                z_start: 0.0,
                z_end: 0.0,
                angles: Vec::new(),
                length: 20.0,
            }],
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: None,
            z_range: Some((0.0, 0.0)),
            total_length: 20.0,
            names: Vec::new(),
            rotary: None,
        };
        let shown = visible(&bend, 15.0);
        assert_eq!(shown.len(), 1);
        assert_close(shown[0].length, 15.0);
        assert_eq!(shown[0].points.len(), 3);
        assert_pt(shown[0].points[0], [0.0, 0.0]);
        assert_pt(shown[0].points[1], [10.0, 0.0]);
        assert_pt(shown[0].points[2], [10.0, 5.0]);
        assert_pt(tool_position(&bend, 15.0).unwrap(), [10.0, 5.0]);

        let length = 10.0_f64.hypot(10.0);
        let ramp = super::Scene {
            strokes: vec![Stroke {
                kind: StrokeKind::Plunge,
                toolpath: 0,
                points: vec![[0.0, 0.0], [10.0, 0.0]],
                zs: vec![0.0, 10.0],
                z_start: 0.0,
                z_end: 10.0,
                angles: Vec::new(),
                length,
            }],
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: None,
            z_range: Some((0.0, 10.0)),
            total_length: length,
            names: Vec::new(),
            rotary: None,
        };
        let half = visible(&ramp, length * 0.5);
        assert_eq!(half.len(), 1);
        assert_close(half[0].length, length * 0.5);
        assert_pt(*half[0].points.last().unwrap(), [5.0, 0.0]);
        assert_close(half[0].z_end, 5.0);
        assert_eq!(half[0].zs.len(), half[0].points.len());
        assert_close(half[0].zs[0], 0.0);
        assert_close(*half[0].zs.last().unwrap(), 5.0);
        assert_pt(tool_position(&ramp, length * 0.5).unwrap(), [5.0, 0.0]);
    }

    #[test]
    fn quarter_and_full_circle_stay_within_tolerance() {
        let center = [0.0, 0.0];
        let start = [1.0, 0.0];
        let end = [0.0, 1.0];
        let tol = 0.01;
        let quarter = tessellate_arc(start, end, center, true, tol);
        assert_eq!(quarter.first().copied(), Some(start));
        assert_eq!(quarter.last().copied(), Some(end));
        assert!(quarter.len() >= 3);
        for point in &quarter {
            let angle = point[1].atan2(point[0]);
            assert!((-1e-9..=std::f64::consts::FRAC_PI_2 + 1e-9).contains(&angle));
            assert_close(point[0].hypot(point[1]), 1.0);
        }
        let deviation = max_arc_deviation(&quarter, center, 1.0, true, std::f64::consts::FRAC_PI_2);
        assert!(deviation <= tol + 1e-9, "quarter deviation {deviation} exceeds {tol}");

        let coarse = tessellate_arc(start, end, center, true, 0.5);
        assert_eq!(coarse, vec![start, end]);
        let fine = tessellate_arc(start, end, center, true, 0.001);
        assert!(fine.len() > quarter.len());
        let fine_dev = max_arc_deviation(&fine, center, 1.0, true, std::f64::consts::FRAC_PI_2);
        assert!(fine_dev <= 0.001 + 1e-9, "fine deviation {fine_dev}");

        let turned = tessellate_arc([13.0, 20.0], [10.0, 23.0], [10.0, 20.0], true, 0.02);
        assert_eq!(turned.first().copied(), Some([13.0, 20.0]));
        assert_eq!(turned.last().copied(), Some([10.0, 23.0]));
        let turned_dev = max_arc_deviation(&turned, [10.0, 20.0], 3.0, true, std::f64::consts::FRAC_PI_2);
        assert!(turned_dev <= 0.02 + 1e-9, "translated deviation {turned_dev}");

        let clockwise = tessellate_arc(start, [0.0, -1.0], center, false, tol);
        assert_eq!(clockwise.first().copied(), Some(start));
        assert_eq!(clockwise.last().copied(), Some([0.0, -1.0]));
        for point in &clockwise {
            let angle = point[1].atan2(point[0]);
            assert!((-std::f64::consts::FRAC_PI_2 - 1e-9..=1e-9).contains(&angle));
        }

        let full = tessellate_arc(start, start, center, true, 0.05);
        assert_eq!(full.first().copied(), Some(start));
        assert_eq!(full.last().copied(), Some(start));
        assert!(full.len() >= 8);
        assert!(full.iter().any(|point| point[0] < -0.5));
        assert!(full.iter().any(|point| point[1] > 0.5));
        assert!(full.iter().any(|point| point[1] < -0.5));
        let full_dev = max_arc_deviation(&full, center, 1.0, true, std::f64::consts::TAU);
        assert!(full_dev <= 0.05 + 1e-9, "full-circle deviation {full_dev}");

        let radial = tessellate_arc([1.0, 0.0], [2.0, 0.0], center, true, 0.01);
        assert_eq!(radial, vec![[1.0, 0.0], [2.0, 0.0]]);
    }

    #[test]
    fn stroke_length_and_kind_follow_the_polyline() {
        assert_close(polyline_length_3d(&[[0.0, 0.0], [3.0, 4.0]], 0.0, 12.0), 13.0);
        assert_close(polyline_length_3d(&[[1.0, 1.0], [1.0, 1.0]], 2.0, 7.0), 5.0);
        assert_close(polyline_length_3d(&[[0.0, 0.0]], 0.0, 1.0), 0.0);

        assert_eq!(stroke_kind(true, &[[0.0, 0.0], [0.0, 0.0]]), StrokeKind::Rapid);
        assert_eq!(stroke_kind(false, &[[0.0, 0.0], [0.0, 0.0]]), StrokeKind::Plunge);
        assert_eq!(stroke_kind(false, &[[0.0, 0.0], [1.0, 0.0]]), StrokeKind::Cut);
        let circle = tessellate_arc([1.0, 0.0], [1.0, 0.0], [0.0, 0.0], true, 0.1);
        assert_eq!(stroke_kind(false, &circle), StrokeKind::Cut);
    }

    #[test]
    fn merge_labels_joins_nearby_callouts_and_keeps_order() {
        // Scene labels are 1-based toolpath numbers at the first cut. "2" sits on the radius of
        // "1", "4" is closer still, "3" is far, and "5" is just beyond "3".
        let labels = vec![
            Label { at: [0.0, 0.0], text: "1".to_string() },
            Label { at: [0.5, 0.0], text: "2".to_string() },
            Label { at: [20.0, 5.0], text: "3".to_string() },
            Label { at: [0.1, -0.2], text: "4".to_string() },
            Label { at: [20.0, 5.6], text: "5".to_string() },
        ];
        let merged = merge_labels(labels, 0.5);
        assert_eq!(
            merged,
            vec![
                Label { at: [0.0, 0.0], text: "1, 2, 4".to_string() },
                Label { at: [20.0, 5.0], text: "3".to_string() },
                Label { at: [20.0, 5.6], text: "5".to_string() },
            ]
        );
    }

    #[test]
    fn invalid_json_is_an_error_and_a_font_path_is_a_file() {
        let loaded = load_scene("not json", 0.1);
        assert!(loaded.is_err(), "garbage must not parse: {loaded:?}");
        if let Some(path) = find_system_font() {
            assert!(path.is_file(), "{}", path.display());
        }
    }

    fn pose(x: f64, y: f64, z: f64) -> campanion_ir::Pose {
        let tip = campanion_ir::Vec3::new(x, y, z).expect("tip");
        campanion_ir::Pose::new(tip, campanion_ir::Vec3::Z).expect("pose")
    }

    /// Rapid, line, quarter-circle arc, quarter-turn helix, then a plunge.
    fn curve_scene() -> super::Scene {
        use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Document, Feeds, MoveRole, Plane, Program, Vec3};
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let center = Vec3::new(0.0, 0.0, 0.0).expect("centre");
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 20.0), feed: None }),
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(10.0, 0.0, 2.0), feed: 100.0, role: MoveRole::Cut }),
            ClBlock::Motion(ClMotion::Arc {
                to: pose(0.0, 10.0, 4.0),
                plane: Plane::Xy,
                center,
                sense: ArcSense::Ccw,
                feed: 100.0,
                role: MoveRole::Cut,
            }),
            ClBlock::Motion(ClMotion::Helix {
                to: pose(10.0, 0.0, 8.0),
                plane: Plane::Xy,
                center,
                sense: ArcSense::Cw,
                sweep: std::f64::consts::FRAC_PI_2,
                feed: 100.0,
                role: MoveRole::Cut,
            }),
            ClBlock::Motion(ClMotion::Linear { to: pose(10.0, 0.0, 5.0), feed: 100.0, role: MoveRole::Plunge }),
        ];
        let path = ClPath::try_new(Some("curves".to_string()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let doc = Document::new(program).expect("document");
        build_scene(&doc, 0.05)
    }

    fn assert_heights_follow_arc_length(stroke: &Stroke) {
        assert_eq!(stroke.zs.len(), stroke.points.len());
        assert!(stroke.points.len() >= 2, "expected a tessellated arc, got {} points", stroke.points.len());
        let mut cumulative = vec![0.0];
        let mut total = 0.0;
        for window in stroke.points.windows(2) {
            total += (window[1][0] - window[0][0]).hypot(window[1][1] - window[0][1]);
            cumulative.push(total);
        }
        assert!(total > 1.0, "arc length {total}");
        for (index, z) in stroke.zs.iter().enumerate() {
            let fraction = cumulative[index] / total;
            assert_close(*z, stroke.z_start + (stroke.z_end - stroke.z_start) * fraction);
        }
    }

    #[test]
    fn line_arc_and_helix_store_a_z_per_point() {
        let scene = curve_scene();
        assert_eq!(scene.strokes.len(), 5);
        for stroke in &scene.strokes {
            assert_eq!(stroke.zs.len(), stroke.points.len(), "{:?} zs {}", stroke.kind, stroke.zs.len());
        }

        let rapid = &scene.strokes[0];
        assert_eq!(rapid.kind, StrokeKind::Rapid);
        assert_eq!(rapid.points.len(), 2);
        assert_close(rapid.zs[0], 20.0);
        assert_close(rapid.zs[1], 0.0);

        let line = &scene.strokes[1];
        assert_eq!(line.kind, StrokeKind::Cut);
        assert_eq!(line.points, vec![[0.0, 0.0], [10.0, 0.0]]);
        assert_eq!(line.zs.len(), 2);
        assert_close(line.zs[0], 0.0);
        assert_close(line.zs[1], 2.0);
        assert_close(line.z_start, 0.0);
        assert_close(line.z_end, 2.0);

        let arc = &scene.strokes[2];
        assert_eq!(arc.kind, StrokeKind::Cut);
        assert_close(arc.z_start, 2.0);
        assert_close(arc.z_end, 4.0);
        assert_heights_follow_arc_length(arc);

        let helix = &scene.strokes[3];
        assert_eq!(helix.kind, StrokeKind::Cut);
        assert_close(helix.z_start, 4.0);
        assert_close(helix.z_end, 8.0);
        assert_heights_follow_arc_length(helix);

        let plunge = &scene.strokes[4];
        assert_eq!(plunge.kind, StrokeKind::Plunge);
        assert_eq!(plunge.zs.len(), plunge.points.len());
        assert_close(plunge.zs[0], 8.0);
        assert_close(*plunge.zs.last().unwrap(), 5.0);

        let (low, high) = scene.z_range.expect("cut range");
        assert_close(low, 0.0);
        assert_close(high, 8.0);

        let partial = visible(&scene, rapid.length + line.length * 0.5);
        assert_eq!(partial.last().expect("cut").zs.len(), partial.last().expect("cut").points.len());
    }

    #[test]
    fn iso_projection_matches_the_axis_vectors() {
        let cos30 = 3.0_f64.sqrt() / 2.0;
        assert_pt(project([1.0, 0.0], 0.0, Projection::Iso), [cos30, 0.5]);
        assert_pt(project([0.0, 1.0], 0.0, Projection::Iso), [-cos30, 0.5]);
        assert_pt(project([0.0, 0.0], 1.0, Projection::Iso), [0.0, 1.0]);
        assert_pt(project([4.0, -2.0], 9.0, Projection::Top), [4.0, -2.0]);
    }

    #[test]
    fn height_color_runs_blue_to_red_and_clamps() {
        assert_eq!(height_color(0.0, (0.0, 4.0)), [0, 0, 255]);
        assert_eq!(height_color(4.0, (0.0, 4.0)), [255, 0, 0]);
        assert_eq!(height_color(-2.0, (0.0, 4.0)), [0, 0, 255]);
        assert_eq!(height_color(9.0, (0.0, 4.0)), [255, 0, 0]);
        assert_eq!(height_color(-5.0, (-5.0, -1.0)), [0, 0, 255]);
        assert_eq!(height_color(-1.0, (-5.0, -1.0)), [255, 0, 0]);
        assert_eq!(height_color(1.0, (2.0, 2.0)), [90, 200, 250]);
        assert_eq!(height_color(f64::NAN, (0.0, 1.0)), [90, 200, 250]);
        let mid = height_color(2.0, (0.0, 4.0));
        assert!(mid[1] > mid[0] && mid[1] > mid[2], "middle of the ramp should be green, got {mid:?}");
    }

    #[test]
    fn projected_bounds_contain_every_projected_point() {
        let scene = curve_scene();
        assert_bounds_cover(&scene, Projection::Top);
        assert_bounds_cover(&scene, Projection::Iso);
        let empty = super::Scene {
            strokes: Vec::new(),
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: None,
            z_range: None,
            total_length: 0.0,
            names: Vec::new(),
            rotary: None,
        };
        assert!(projected_bounds(&empty, Projection::Top).is_none());
        assert!(projected_bounds(&empty, Projection::Iso).is_none());
    }

    fn assert_bounds_cover(scene: &super::Scene, projection: Projection) {
        let (min, max) = projected_bounds(scene, projection).expect("bounds");
        let mut hit_min_x = false;
        let mut hit_min_y = false;
        let mut hit_max_x = false;
        let mut hit_max_y = false;
        let mut any = false;
        for stroke in &scene.strokes {
            for (index, point) in stroke.points.iter().enumerate() {
                let z = stroke.zs[index];
                let projected = project(*point, z, projection);
                assert!(projected[0] + 1e-9 >= min[0] && projected[0] - 1e-9 <= max[0], "{projected:?} x outside {min:?} {max:?}");
                assert!(projected[1] + 1e-9 >= min[1] && projected[1] - 1e-9 <= max[1], "{projected:?} y outside {min:?} {max:?}");
                hit_min_x |= (projected[0] - min[0]).abs() <= 1e-6;
                hit_min_y |= (projected[1] - min[1]).abs() <= 1e-6;
                hit_max_x |= (projected[0] - max[0]).abs() <= 1e-6;
                hit_max_y |= (projected[1] - max[1]).abs() <= 1e-6;
                any = true;
            }
        }
        assert!(any);
        assert!(hit_min_x && hit_min_y && hit_max_x && hit_max_y, "bounds are loose for {projection:?}: {min:?} {max:?}");
    }

    #[test]
    fn non_rotary_scene_has_no_axis_and_no_angles() {
        let scene = curve_scene();
        assert!(scene.rotary.is_none());
        for stroke in &scene.strokes {
            assert!(stroke.angles.is_empty(), "{:?} stored angles", stroke.kind);
        }
    }

    #[test]
    fn rotary_axis_endpoints_follow_the_extent_through_the_centre() {
        use campanion_ir::RotaryAxisKind;
        let bounds = ([0.0, -5.0], [30.0, 15.0]);
        let center = [4.0, 2.0, 7.0];
        let (start, end) = rotary_axis_endpoints(RotaryAxisKind::A, center, bounds).expect("a axis");
        assert_eq!(start, [0.0, 2.0, 7.0]);
        assert_eq!(end, [30.0, 2.0, 7.0]);

        let outside = [40.0, 2.0, 7.0];
        let (start, end) = rotary_axis_endpoints(RotaryAxisKind::A, outside, bounds).expect("a past the part");
        assert_eq!(start, [0.0, 2.0, 7.0]);
        assert_eq!(end, [40.0, 2.0, 7.0]);

        let (start, end) = rotary_axis_endpoints(RotaryAxisKind::B, center, bounds).expect("b axis");
        assert_eq!(start, [4.0, -5.0, 7.0]);
        assert_eq!(end, [4.0, 15.0, 7.0]);

        let below = [4.0, -9.0, 1.0];
        let (start, end) = rotary_axis_endpoints(RotaryAxisKind::B, below, bounds).expect("b below the part");
        assert_eq!(start, [4.0, -9.0, 1.0]);
        assert_eq!(end, [4.0, 15.0, 1.0]);

        assert!(rotary_axis_endpoints(RotaryAxisKind::A, [f64::NAN, 0.0, 0.0], bounds).is_none());
        assert!(rotary_axis_endpoints(RotaryAxisKind::B, center, ([f64::INFINITY, 0.0], [1.0, 1.0])).is_none());
    }

    #[test]
    fn angle_color_maps_the_range_and_the_legend_labels_the_ends() {
        assert_eq!(angle_color(0.0, (0.0, 90.0)), [0, 0, 255]);
        assert_eq!(angle_color(90.0, (0.0, 90.0)), [255, 0, 0]);
        assert_eq!(angle_color(-5.0, (0.0, 90.0)), [0, 0, 255]);
        assert_eq!(angle_color(180.0, (0.0, 90.0)), [255, 0, 0]);
        assert_eq!(angle_color(10.0, (10.0, 10.0)), [90, 200, 250]);
        assert_eq!(angle_color(f64::NAN, (0.0, 90.0)), [90, 200, 250]);
        assert_eq!(angle_color(45.0, (0.0, 90.0)), height_color(45.0, (0.0, 90.0)));
        let mid = angle_color(45.0, (0.0, 90.0));
        assert!(mid[1] > mid[0] && mid[1] >= mid[2], "middle of the ramp should be green or yellow, got {mid:?}");

        let legend = angle_legend((0.0, 90.0), 2).expect("legend");
        assert_eq!(legend.low_label, "0°");
        assert_eq!(legend.high_label, "90°");
        assert_eq!(legend.colours, vec![[0, 0, 255], [255, 0, 0]]);
        assert!(angle_legend((90.0, 90.0), 4).is_none());
        assert!(angle_legend((0.0, 90.0), 0).is_none());
        assert_eq!(format_angle_deg(-90.0), "-90°");
        assert_eq!(format_angle_deg(12.5), "12.5°");
        assert_eq!(format_angle_deg(-0.0), "0°");
    }

    #[test]
    fn rotary_span_lists_indexes_and_ranges_continuous_feeds() {
        let indexed = rotary_span(&[
            RotarySample { start_deg: 0.0, end_deg: 0.0, rapid: true },
            RotarySample { start_deg: 0.0, end_deg: 0.0, rapid: false },
            RotarySample { start_deg: 0.0, end_deg: 90.0, rapid: true },
            RotarySample { start_deg: 90.0, end_deg: 90.0, rapid: false },
            RotarySample { start_deg: 90.0, end_deg: 90.0 + 1e-9, rapid: false },
        ])
        .expect("span");
        assert_eq!(indexed, RotarySpan::Indexed(vec![0.0, 90.0]));
        assert_eq!(rotary_overlay(&indexed), "Index: 0°, 90°");

        let reversed = rotary_span(&[
            RotarySample { start_deg: 90.0, end_deg: 90.0, rapid: false },
            RotarySample { start_deg: 90.0, end_deg: -90.0, rapid: true },
            RotarySample { start_deg: -90.0, end_deg: -90.0, rapid: false },
        ])
        .expect("span");
        assert_eq!(rotary_overlay(&reversed), "Index: -90°, 90°");

        let continuous = rotary_span(&[
            RotarySample { start_deg: 0.0, end_deg: 0.0, rapid: true },
            RotarySample { start_deg: 0.0, end_deg: 180.0, rapid: false },
            RotarySample { start_deg: 180.0, end_deg: 270.5, rapid: false },
        ])
        .expect("span");
        assert_eq!(continuous, RotarySpan::Continuous { min_deg: 0.0, max_deg: 270.5 });
        assert_eq!(rotary_overlay(&continuous), "Angle: 0°..270.5°");
        assert!(rotary_span(&[RotarySample { start_deg: f64::NAN, end_deg: 1.0, rapid: false }]).is_none());
    }

    fn rotary_tip(setup: &campanion_ir::RotarySetup, x: f64, y: f64, z: f64, angle: f64) -> campanion_ir::Pose {
        use campanion_ir::{Pose, Vec3, rotate_vector};
        let tip = Vec3::new(x, y, z).expect("tip");
        let axis = rotate_vector(setup, Vec3::Z, -angle);
        Pose::new(tip, axis).expect("pose").with_rotary(angle).expect("angle")
    }

    /// Two indexed faces, 0° then 90°, with the A axis offset from the origin.
    fn indexed_example() -> super::Scene {
        use campanion_ir::{
            ClBlock, ClMotion, ClPath, CompensationMode, Document, Feeds, MoveRole, Program, RotaryAxisKind, RotaryDirection, RotarySetup, Vec3,
        };
        let center = Vec3::new(5.0, -2.0, 3.0).expect("centre");
        let setup = RotarySetup::new(RotaryAxisKind::A, center, RotaryDirection::Positive, None).expect("setup");
        let feeds = Feeds::try_new(1000.0, None, None, Some(300.0), None).expect("feeds");
        let face0 = vec![
            ClBlock::Motion(ClMotion::Rapid { to: rotary_tip(&setup, 0.0, 0.0, 5.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: rotary_tip(&setup, 20.0, 0.0, 5.0, 0.0), feed: 1000.0, role: MoveRole::Cut }),
        ];
        let face90 = vec![
            ClBlock::Motion(ClMotion::Rapid { to: rotary_tip(&setup, 20.0, 8.0, 5.0, 90.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: rotary_tip(&setup, 0.0, 8.0, 5.0, 90.0), feed: 1000.0, role: MoveRole::Cut }),
        ];
        let path =
            |name: &str, blocks| ClPath::try_new(Some(name.to_string()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(
            Some("indexed".into()),
            Vec::new(),
            None,
            vec![path("face0", face0), path("face90", face90)],
            Vec::new(),
            Vec::new(),
            Some(setup),
        )
        .expect("program");
        build_scene(&Document::new(program).expect("document"), 0.05)
    }

    #[test]
    fn indexed_example_draws_the_axis_and_lists_the_angles() {
        let scene = indexed_example();
        let rotary = scene.rotary.clone().expect("rotary setup");
        assert_eq!(rotary.label, "A");
        assert_eq!(rotary.center, [5.0, -2.0, 3.0]);
        let (start, end) = rotary.line.expect("axis line");
        assert_eq!(start, [0.0, -2.0, 3.0]);
        assert_eq!(end, [20.0, -2.0, 3.0]);
        assert_eq!(rotary.overlay().as_deref(), Some("Index: 0°, 90°"));
        assert_eq!(rotary.angle_range(), Some((0.0, 90.0)));
        assert_eq!(angle_color(0.0, rotary.angle_range().expect("range")), [0, 0, 255]);
        assert_eq!(angle_color(90.0, rotary.angle_range().expect("range")), [255, 0, 0]);

        assert_eq!(scene.strokes.len(), 3);
        let cut0 = &scene.strokes[0];
        assert_eq!(cut0.kind, StrokeKind::Cut);
        assert_eq!(cut0.angles, vec![0.0, 0.0]);
        let change = &scene.strokes[1];
        assert_eq!(change.kind, StrokeKind::Rapid);
        assert_eq!(change.angles, vec![0.0, 90.0]);
        let cut90 = &scene.strokes[2];
        assert_eq!(cut90.kind, StrokeKind::Cut);
        assert_eq!(cut90.angles, vec![90.0, 90.0]);

        let (min, max) = projected_bounds(&scene, Projection::Top).expect("bounds");
        assert!(min[1] <= -2.0 + 1e-9, "axis y must be inside the fit bounds, min {min:?}");
        assert!(max[0] + 1e-9 >= 20.0 && min[0] <= 1e-9, "axis x extent {min:?} {max:?}");
    }

    #[test]
    fn continuous_rotary_reports_the_range_and_interpolates_the_angle() {
        use campanion_ir::{
            ClBlock, ClMotion, ClPath, CompensationMode, Document, Feeds, MoveRole, Program, RotaryAxisKind, RotaryDirection, RotarySetup, Vec3,
        };
        let setup = RotarySetup::new(RotaryAxisKind::B, Vec3::new(1.0, 2.0, 3.0).expect("centre"), RotaryDirection::Positive, None).expect("setup");
        let feeds = Feeds::try_new(800.0, None, None, None, None).expect("feeds");
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: rotary_tip(&setup, 0.0, 0.0, 0.0, 10.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: rotary_tip(&setup, 0.0, 10.0, 0.0, 190.0), feed: 800.0, role: MoveRole::Cut }),
        ];
        let path = ClPath::try_new(Some("wrap".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), Some(setup)).expect("program");
        let scene = build_scene(&Document::new(program).expect("document"), 0.05);

        let rotary = scene.rotary.clone().expect("rotary");
        assert_eq!(rotary.label, "B");
        let (start, end) = rotary.line.expect("axis");
        assert_eq!(start, [1.0, 0.0, 3.0]);
        assert_eq!(end, [1.0, 10.0, 3.0]);
        assert_eq!(rotary.overlay().as_deref(), Some("Angle: 10°..190°"));

        assert_eq!(scene.strokes.len(), 1);
        let cut = &scene.strokes[0];
        assert_eq!(cut.kind, StrokeKind::Cut);
        assert_eq!(cut.angles.len(), 2);
        assert_close(cut.angles[0], 10.0);
        assert_close(cut.angles[1], 190.0);

        let half = visible(&scene, cut.length * 0.5);
        let shown = half.last().expect("half cut");
        assert_eq!(shown.angles.len(), shown.points.len());
        assert_close(*shown.angles.last().expect("angle"), 100.0);
        assert_pt(*shown.points.last().expect("point"), [0.0, 5.0]);
    }
}
