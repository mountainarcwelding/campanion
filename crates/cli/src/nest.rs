//! Deterministic sheet packer for a router job.
//!
//! Each part is an axis-aligned rectangle: the bounds of named operations, the bounds of a
//! DXF or SVG, or an explicit width and height. The packer does not look at the contour, and
//! it makes no claim of optimal yield. A later contour packer can replace the rectangle.
//!
//! Parts are placed in input order. For each part the candidate translations are the sheet
//! inset (`margin`) and the far side of every part already placed and of every keep-out.
//! That is exact bottom-left. A uniform grid of `max(1 mm, gap)` is not used, because a part
//! whose size is not a multiple of that step can fit and still miss every grid point.
//!
//! The first feasible candidate wins. Order is ascending Y, then ascending X, then ascending
//! rotation index. Index 0 is 0°. Index 1 is 90° when `rotation_step_deg` is 90. A step of
//! 0° or 180° tries 0° only: a rectangle's axis-aligned bounds repeat every 180°.
//!
//! Clearance, measured between the rectangles:
//! - A part lies inside the sheet inset by `margin`. The tool radius is not added to the margin.
//! - Two parts must not overlap once each rectangle is expanded by `gap / 2`. The gap between
//!   them is at least `gap` along X or along Y. The tool radius is not added again; put it in
//!   `gap` when two cuts face each other.
//! - A part clears each keep-out by `tool_radius + gap` along X or along Y. `tool_radius` is
//!   `nest.tool_radius` when the file sets it, otherwise half the largest tool diameter or kerf.
//!   No tools means 0.
//!
//! A fit that is short by at most 1e-6 mm counts as touching. The same input produces the same
//! placements. The IR is unchanged: `campanion run` applies the placement as a transform while
//! it builds the paths.

use std::cmp::Ordering;
use std::path::Path;

use campanion_geom::{Arc, BoundingBox, Contour, Line, Point, Segment, Tolerance, cmp_float_key};
use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg};

use crate::error::CliError;
use crate::jobfile::{self, JobFile, Nest, NestPartGeom, Sheet};

/// A shortfall of this size, in millimetres, still counts as touching.
const FIT_EPS_MM: f64 = 1e-6;
const MAX_PACK_PARTS: usize = 256;
const MAX_PACK_KEEPOUTS: usize = 64;
const MAX_PACK_MM: f64 = 1.0e6;

/// Packer or clearance failure. The text is the run error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct NestError(String);

impl std::fmt::Display for NestError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

/// Axis-aligned rectangle in the sheet frame, millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PackRect {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) max_x: f64,
    pub(crate) max_y: f64,
}

/// One rectangle to place. `rotations_deg` is tried in order; the index is the tie-break.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PackPart {
    pub(crate) name: String,
    pub(crate) width: f64,
    pub(crate) height: f64,
    pub(crate) rotations_deg: Vec<f64>,
}

/// Sheet, spacing and the parts in input order.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PackInput {
    pub(crate) sheet_width: f64,
    pub(crate) sheet_height: f64,
    pub(crate) gap: f64,
    pub(crate) margin: f64,
    pub(crate) tool_radius: f64,
    pub(crate) keep_out: Vec<PackRect>,
    pub(crate) parts: Vec<PackPart>,
}

/// Where a part's rectangle sits on the sheet. `x` and `y` are the minimum corner.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Placement {
    pub(crate) name: String,
    pub(crate) x: f64,
    pub(crate) y: f64,
    pub(crate) rotation_deg: f64,
    pub(crate) width: f64,
    pub(crate) height: f64,
}

/// Rigid move from a part's drawing frame into the sheet frame.
///
/// Rotation is about the part AABB centre, counter-clockwise, then `translate` is added.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PartFrame {
    center_x: f64,
    center_y: f64,
    rotation_deg: f64,
    translate_x: f64,
    translate_y: f64,
}

/// Contours of one operation, already limited to that operation's layers.
pub(crate) struct OpGeometry<'a> {
    pub(crate) name: &'a str,
    pub(crate) contours: &'a [Contour],
}

/// Placements, the stderr lines, and the `campanion-nest/1` JSON.
pub(crate) struct Report {
    frames: Vec<(String, PartFrame)>,
    pub(crate) summary: String,
    pub(crate) json: String,
}

impl Report {
    /// Transform for an operation that belongs to a placed part. `None` when the operation is not nested.
    pub(crate) fn frame(&self, operation: &str) -> Option<&PartFrame> {
        self.frames.iter().find(|(name, _)| name == operation).map(|(_, frame)| frame)
    }
}

/// Angles the packer tries, in rotation-index order.
///
/// 90° tries 0° then 90°. 0° and 180° try 0° only.
pub(crate) fn rotation_angles(step_deg: f64) -> Vec<f64> {
    if (step_deg - 90.0).abs() <= 1e-9 { vec![0.0, 90.0] } else { vec![0.0] }
}

/// `explicit` wins. Otherwise the largest radius, or 0 when there are no tools.
pub(crate) fn choose_tool_radius(explicit: Option<f64>, radii: impl IntoIterator<Item = f64>) -> f64 {
    if let Some(radius) = explicit {
        return radius;
    }
    let mut best = 0.0;
    for radius in radii {
        if radius > best {
            best = radius;
        }
    }
    best
}

/// Place every part. The same input returns the same placements, or the same error.
pub(crate) fn pack(input: &PackInput) -> Result<Vec<Placement>, NestError> {
    validate(input)?;
    let mut placed = Vec::with_capacity(input.parts.len());
    for part in &input.parts {
        let Some(pose) = first_pose(input, &placed, part) else {
            return Err(NestError(format!(
                "nest: part `{}` does not fit on the sheet ({} x {} mm)",
                part.name, input.sheet_width, input.sheet_height
            )));
        };
        placed.push(pose);
    }
    check_placements(input, &placed)?;
    Ok(placed)
}

/// Sheet inset, part gap and keep-out clearance. The first failure is the error.
pub(crate) fn check_placements(input: &PackInput, placements: &[Placement]) -> Result<(), NestError> {
    if placements.len() != input.parts.len() {
        return Err(NestError("nest: placement count does not match the parts".to_string()));
    }
    for (place, part) in placements.iter().zip(input.parts.iter()) {
        if place.name != part.name {
            return Err(NestError(format!("nest: placement `{}` is out of order", place.name)));
        }
        if !part.rotations_deg.iter().any(|angle| (*angle - place.rotation_deg).abs() <= 1e-6) {
            return Err(NestError(format!("nest: part `{}` uses a rotation the job did not allow", part.name)));
        }
        let (width, height) = placed_size(part.width, part.height, place.rotation_deg);
        if (width - place.width).abs() > FIT_EPS_MM || (height - place.height).abs() > FIT_EPS_MM {
            return Err(NestError(format!("nest: part `{}` placement size does not match its rotation", part.name)));
        }
        if !inside_margin(input, place.x, place.y, place.width, place.height) {
            return Err(NestError(format!("nest: part `{}` lies outside the sheet", part.name)));
        }
    }
    for (index, place) in placements.iter().enumerate() {
        for other in placements.iter().skip(index + 1) {
            if !separated(
                place.x,
                place.y,
                place.x + place.width,
                place.y + place.height,
                other.x,
                other.y,
                other.x + other.width,
                other.y + other.height,
                input.gap,
            ) {
                return Err(NestError(format!("nest: parts `{}` and `{}` are closer than the gap ({} mm)", place.name, other.name, input.gap)));
            }
        }
        let need = input.tool_radius + input.gap;
        for rect in &input.keep_out {
            if !separated(place.x, place.y, place.x + place.width, place.y + place.height, rect.min_x, rect.min_y, rect.max_x, rect.max_y, need) {
                return Err(NestError(format!("nest: part `{}` is closer than {need} mm to a keep-out", place.name)));
            }
        }
    }
    Ok(())
}

/// Pack when the job has `sheet` and `nest`. `None` when either is absent.
pub(crate) fn plan(job: &JobFile, drawing: &Drawing, ops: &[OpGeometry<'_>]) -> Result<Option<Report>, CliError> {
    let Some(nest) = job.nest() else {
        return Ok(None);
    };
    let Some(sheet) = job.sheet() else {
        return Err(CliError::run("nest needs a sheet"));
    };
    let tool_radius = choose_tool_radius(nest.tool_radius(), tool_radii(job)?);
    let resolved = resolve_parts(job, drawing, ops, nest)?;
    let input = PackInput {
        sheet_width: sheet.width(),
        sheet_height: sheet.height(),
        gap: nest.gap(),
        margin: nest.margin(),
        tool_radius,
        keep_out: nest
            .keep_outs()
            .iter()
            .map(|rect| PackRect { min_x: rect.min_x(), min_y: rect.min_y(), max_x: rect.max_x(), max_y: rect.max_y() })
            .collect(),
        parts: resolved.iter().map(|part| part.pack_part()).collect(),
    };
    let placements = pack(&input).map_err(|err| CliError::run(err.to_string()))?;
    check_placements(&input, &placements).map_err(|err| CliError::run(err.to_string()))?;
    let mut frames = Vec::new();
    if resolved.len() != placements.len() {
        return Err(CliError::run("nest: placement count does not match the parts"));
    }
    for (part, placement) in resolved.iter().zip(placements.iter()) {
        if part.operations.is_empty() {
            continue;
        }
        let frame = frame_for(part.min_x, part.min_y, part.min_x + part.width, part.min_y + part.height, placement);
        for name in &part.operations {
            frames.push((name.clone(), frame));
        }
    }
    let summary = summary_lines(&placements);
    let json = placements_json(sheet, nest, tool_radius, &placements)?;
    Ok(Some(Report { frames, summary, json }))
}

/// Move contours into the sheet frame. Lines stay exact on a quarter turn. An arc keeps its sweep.
pub(crate) fn transform_contours(contours: &[Contour], frame: &PartFrame, tolerance: Tolerance) -> Result<Vec<Contour>, CliError> {
    let mut out = Vec::with_capacity(contours.len());
    for contour in contours {
        let mut segments = Vec::with_capacity(contour.segments().len());
        for segment in contour.segments() {
            segments.push(transform_segment(*segment, frame)?);
        }
        let built = Contour::try_new(segments, contour.is_closed(), &tolerance).map_err(|err| CliError::run(format!("nest: {err}")))?;
        out.push(built);
    }
    Ok(out)
}

struct ResolvedPart {
    name: String,
    width: f64,
    height: f64,
    min_x: f64,
    min_y: f64,
    rotations_deg: Vec<f64>,
    operations: Vec<String>,
}

impl ResolvedPart {
    fn pack_part(&self) -> PackPart {
        PackPart { name: self.name.clone(), width: self.width, height: self.height, rotations_deg: self.rotations_deg.clone() }
    }
}

fn tool_radii(job: &JobFile) -> Result<Vec<f64>, CliError> {
    let mut radii = Vec::with_capacity(job.tools().len());
    for tool in job.tools() {
        let radius = tool.path_tool().compensation_distance().map_err(|err| CliError::run(err.to_string()))?;
        radii.push(radius);
    }
    Ok(radii)
}

fn resolve_parts(job: &JobFile, drawing: &Drawing, ops: &[OpGeometry<'_>], nest: &Nest) -> Result<Vec<ResolvedPart>, CliError> {
    let mut resolved = Vec::with_capacity(nest.parts().len());
    for part in nest.parts() {
        let (min_x, min_y, width, height, operations) = match part.geom() {
            NestPartGeom::Rectangle { width, height } => (0.0, 0.0, *width, *height, Vec::new()),
            NestPartGeom::Operations { names } => {
                let (min_x, min_y, width, height) = bounds_of_ops(part.name(), names, ops)?;
                (min_x, min_y, width, height, names.clone())
            }
            NestPartGeom::Source { path, hash, layers } => {
                let (min_x, min_y, width, height) = bounds_of_source(job, drawing, part.name(), path, hash, layers.as_deref())?;
                (min_x, min_y, width, height, Vec::new())
            }
        };
        resolved.push(ResolvedPart {
            name: part.name().to_string(),
            width,
            height,
            min_x,
            min_y,
            rotations_deg: rotation_angles(part.rotation_step_deg()),
            operations,
        });
    }
    Ok(resolved)
}

fn bounds_of_ops(part_name: &str, names: &[String], ops: &[OpGeometry<'_>]) -> Result<(f64, f64, f64, f64), CliError> {
    let mut bounds: Option<BoundingBox> = None;
    for name in names {
        let Some(op) = ops.iter().find(|op| op.name == name) else {
            return Err(CliError::run(format!("nest part `{part_name}`: operation `{name}` has no geometry")));
        };
        for contour in op.contours {
            let next = contour.bounding_box().map_err(|err| CliError::run(format!("nest part `{part_name}`: {err}")))?;
            bounds = Some(match bounds {
                Some(acc) => acc.union(next),
                None => next,
            });
        }
    }
    box_size(part_name, bounds)
}

fn bounds_of_source(
    job: &JobFile,
    job_drawing: &Drawing,
    part_name: &str,
    path: &str,
    hash: &str,
    layers: Option<&[String]>,
) -> Result<(f64, f64, f64, f64), CliError> {
    if path == job.source().path() {
        if hash != job.source().hash() {
            crate::run::warn(&format!("nest part `{part_name}` source hash {hash} does not match the job source hash {}", job.source().hash()))?;
        }
        return box_size(part_name, layer_bounds(part_name, job_drawing, layers)?);
    }
    if is_step_or_stl(path) {
        return Err(CliError::run(format!("nest part `{part_name}`: source must be DXF or SVG")));
    }
    let full = jobfile::join_job_path(job, path);
    let text = crate::job::read_text(&full).map_err(|err| CliError::run(format!("nest part `{part_name}`: {err}")))?;
    let actual = jobfile::hash_bytes(text.as_bytes());
    if actual != hash {
        crate::run::warn(&format!("nest part `{part_name}` source {path} changed: stored hash {hash}, file hash {actual}"))?;
    }
    let options = ImportOptions::new(job.tolerance());
    let imported = if is_svg(path) { import_svg(&text, &options) } else { import_dxf(&text, &options) }
        .map_err(|err| CliError::run(format!("nest part `{part_name}`: {err}")))?;
    box_size(part_name, layer_bounds(part_name, &imported, layers)?)
}

fn layer_bounds(part_name: &str, drawing: &Drawing, layers: Option<&[String]>) -> Result<Option<BoundingBox>, CliError> {
    if let Some(layers) = layers {
        for layer in layers {
            if !drawing.contours().iter().any(|item| item.layer() == layer) {
                return Err(CliError::run(format!("nest part `{part_name}`: no geometry on layer `{layer}`")));
            }
        }
    }
    let mut bounds: Option<BoundingBox> = None;
    for item in drawing.contours() {
        let take = match layers {
            None => true,
            Some(layers) => layers.iter().any(|layer| layer == item.layer()),
        };
        if !take {
            continue;
        }
        let next = item.contour().bounding_box().map_err(|err| CliError::run(format!("nest part `{part_name}`: {err}")))?;
        bounds = Some(match bounds {
            Some(acc) => acc.union(next),
            None => next,
        });
    }
    if bounds.is_none() {
        return Err(CliError::run(format!("nest part `{part_name}`: no geometry")));
    }
    Ok(bounds)
}

fn box_size(name: &str, bounds: Option<BoundingBox>) -> Result<(f64, f64, f64, f64), CliError> {
    let Some(bounds) = bounds else {
        return Err(CliError::run(format!("nest part `{name}`: no geometry")));
    };
    let width = bounds.width();
    let height = bounds.height();
    if !(width.is_finite() && height.is_finite() && width > 0.0 && height > 0.0) {
        return Err(CliError::run(format!("nest part `{name}` has no area")));
    }
    if width > MAX_PACK_MM || height > MAX_PACK_MM {
        return Err(CliError::run(format!("nest part `{name}` is larger than {MAX_PACK_MM} mm")));
    }
    Ok((bounds.min().x(), bounds.min().y(), width, height))
}

fn is_svg(path: &str) -> bool {
    Path::new(path).extension().and_then(|extension| extension.to_str()).is_some_and(|text| text.eq_ignore_ascii_case("svg"))
}

fn is_step_or_stl(path: &str) -> bool {
    let Some(text) = Path::new(path).extension().and_then(|extension| extension.to_str()) else {
        return false;
    };
    text.eq_ignore_ascii_case("step") || text.eq_ignore_ascii_case("stp") || text.eq_ignore_ascii_case("stl")
}

fn frame_for(min_x: f64, min_y: f64, max_x: f64, max_y: f64, placement: &Placement) -> PartFrame {
    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    let (rot_min_x, rot_min_y) = rotated_aabb_min(min_x, min_y, max_x, max_y, placement.rotation_deg);
    PartFrame { center_x, center_y, rotation_deg: placement.rotation_deg, translate_x: placement.x - rot_min_x, translate_y: placement.y - rot_min_y }
}

fn rotated_aabb_min(min_x: f64, min_y: f64, max_x: f64, max_y: f64, rotation_deg: f64) -> (f64, f64) {
    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;
    let corners = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)];
    let mut out_x = f64::MAX;
    let mut out_y = f64::MAX;
    for (x, y) in corners {
        let (px, py) = rotate_about(x, y, center_x, center_y, rotation_deg);
        out_x = out_x.min(px);
        out_y = out_y.min(py);
    }
    (out_x, out_y)
}

fn rotate_about(x: f64, y: f64, center_x: f64, center_y: f64, rotation_deg: f64) -> (f64, f64) {
    let (rx, ry) = rotate_xy(x - center_x, y - center_y, rotation_deg);
    (center_x + rx, center_y + ry)
}

fn apply_frame(frame: &PartFrame, point: Point) -> Result<Point, CliError> {
    let (rx, ry) = rotate_xy(point.x() - frame.center_x, point.y() - frame.center_y, frame.rotation_deg);
    Point::new(frame.center_x + rx + frame.translate_x, frame.center_y + ry + frame.translate_y).map_err(|err| CliError::run(format!("nest: {err}")))
}

fn transform_segment(segment: Segment, frame: &PartFrame) -> Result<Segment, CliError> {
    match segment {
        Segment::Line(line) => {
            let start = apply_frame(frame, line.start())?;
            let end = apply_frame(frame, line.end())?;
            Ok(Segment::Line(Line::new(start, end).map_err(|err| CliError::run(format!("nest: {err}")))?))
        }
        Segment::Arc(arc) => {
            let start = apply_frame(frame, arc.start_point())?;
            let end = apply_frame(frame, arc.end_point())?;
            if arc.is_full_turn() {
                let center = apply_frame(frame, arc.center())?;
                let start_angle = (start - center).angle();
                let built = Arc::new(center, arc.radius(), start_angle, arc.sweep()).map_err(|err| CliError::run(format!("nest: {err}")))?;
                return Ok(Segment::Arc(built));
            }
            match arc.bulge() {
                Ok(bulge) => {
                    let built = Arc::from_bulge(start, end, bulge).map_err(|err| CliError::run(format!("nest: {err}")))?;
                    Ok(Segment::Arc(built))
                }
                Err(_) => {
                    let center = apply_frame(frame, arc.center())?;
                    let start_angle = (start - center).angle();
                    let built = Arc::new(center, arc.radius(), start_angle, arc.sweep()).map_err(|err| CliError::run(format!("nest: {err}")))?;
                    Ok(Segment::Arc(built))
                }
            }
        }
    }
}

fn summary_lines(placements: &[Placement]) -> String {
    let mut text = String::new();
    for placement in placements {
        text.push_str(&format!(
            "nest: `{}` at ({}, {}) mm, {} x {} mm, rotation {} deg\n",
            placement.name, placement.x, placement.y, placement.width, placement.height, placement.rotation_deg
        ));
    }
    text
}

fn placements_json(sheet: Sheet, nest: &Nest, tool_radius: f64, placements: &[Placement]) -> Result<String, CliError> {
    let mut keep_out = Vec::with_capacity(nest.keep_outs().len());
    for rect in nest.keep_outs() {
        keep_out.push(serde_json::json!({
            "min": [rect.min_x(), rect.min_y()],
            "max": [rect.max_x(), rect.max_y()],
        }));
    }
    let mut placed = Vec::with_capacity(placements.len());
    for placement in placements {
        placed.push(serde_json::json!({
            "name": placement.name,
            "x": placement.x,
            "y": placement.y,
            "rotation_deg": placement.rotation_deg,
            "width": placement.width,
            "height": placement.height,
        }));
    }
    let value = serde_json::json!({
        "format": "campanion-nest/1",
        "sheet": { "width": sheet.width(), "height": sheet.height() },
        "gap": nest.gap(),
        "margin": nest.margin(),
        "tool_radius": tool_radius,
        "keep_out": keep_out,
        "placements": placed,
    });
    let text = serde_json::to_string_pretty(&value).map_err(|err| CliError::run(format!("nest: {err}")))?;
    Ok(text)
}

fn validate(input: &PackInput) -> Result<(), NestError> {
    if !(input.sheet_width.is_finite() && input.sheet_height.is_finite() && input.sheet_width > 0.0 && input.sheet_height > 0.0) {
        return Err(NestError("nest: sheet width and height must be greater than 0".to_string()));
    }
    if input.sheet_width > MAX_PACK_MM || input.sheet_height > MAX_PACK_MM {
        return Err(NestError(format!("nest: sheet is larger than {MAX_PACK_MM} mm")));
    }
    if !(input.gap.is_finite() && input.gap >= 0.0) {
        return Err(NestError("nest: gap must be at least 0".to_string()));
    }
    if !(input.margin.is_finite() && input.margin >= 0.0) {
        return Err(NestError("nest: margin must be at least 0".to_string()));
    }
    if !(input.tool_radius.is_finite() && input.tool_radius >= 0.0) {
        return Err(NestError("nest: tool_radius must be at least 0".to_string()));
    }
    if input.parts.len() > MAX_PACK_PARTS {
        return Err(NestError(format!("nest: too many parts: {} > {MAX_PACK_PARTS}", input.parts.len())));
    }
    if input.keep_out.len() > MAX_PACK_KEEPOUTS {
        return Err(NestError(format!("nest: too many keep-outs: {} > {MAX_PACK_KEEPOUTS}", input.keep_out.len())));
    }
    for rect in &input.keep_out {
        if !(rect.min_x.is_finite() && rect.min_y.is_finite() && rect.max_x.is_finite() && rect.max_y.is_finite()) {
            return Err(NestError("nest: keep-out must be finite".to_string()));
        }
        if !(rect.max_x > rect.min_x && rect.max_y > rect.min_y) {
            return Err(NestError("nest: keep-out max must be greater than min".to_string()));
        }
    }
    for part in &input.parts {
        if part.name.is_empty() {
            return Err(NestError("nest: part name is empty".to_string()));
        }
        if !(part.width.is_finite() && part.height.is_finite() && part.width > 0.0 && part.height > 0.0) {
            return Err(NestError(format!("nest: part `{}` needs a positive width and height", part.name)));
        }
        if part.rotations_deg.is_empty() {
            return Err(NestError(format!("nest: part `{}` has no rotation", part.name)));
        }
        for angle in &part.rotations_deg {
            if !angle.is_finite() {
                return Err(NestError(format!("nest: part `{}` rotation must be finite", part.name)));
            }
        }
    }
    Ok(())
}

fn first_pose(input: &PackInput, placed: &[Placement], part: &PackPart) -> Option<Placement> {
    let xs = candidate_axis(
        input.margin,
        placed.iter().map(|place| place.x + place.width),
        input.keep_out.iter().map(|rect| rect.max_x),
        input.gap,
        input.tool_radius,
    );
    let ys = candidate_axis(
        input.margin,
        placed.iter().map(|place| place.y + place.height),
        input.keep_out.iter().map(|rect| rect.max_y),
        input.gap,
        input.tool_radius,
    );
    for y in &ys {
        for x in &xs {
            for rotation in &part.rotations_deg {
                let (width, height) = placed_size(part.width, part.height, *rotation);
                if pose_fits(input, placed, *x, *y, width, height) {
                    return Some(Placement { name: part.name.clone(), x: *x, y: *y, rotation_deg: *rotation, width, height });
                }
            }
        }
    }
    None
}

fn candidate_axis(origin: f64, placed_far: impl Iterator<Item = f64>, keep_far: impl Iterator<Item = f64>, gap: f64, tool_radius: f64) -> Vec<f64> {
    let mut values = vec![origin];
    for far in placed_far {
        values.push(far + gap);
    }
    let keep = tool_radius + gap;
    for far in keep_far {
        values.push(far + keep);
    }
    unique_sorted(values)
}

fn pose_fits(input: &PackInput, placed: &[Placement], x: f64, y: f64, width: f64, height: f64) -> bool {
    if !inside_margin(input, x, y, width, height) {
        return false;
    }
    for place in placed {
        if !separated(x, y, x + width, y + height, place.x, place.y, place.x + place.width, place.y + place.height, input.gap) {
            return false;
        }
    }
    let need = input.tool_radius + input.gap;
    for rect in &input.keep_out {
        if !separated(x, y, x + width, y + height, rect.min_x, rect.min_y, rect.max_x, rect.max_y, need) {
            return false;
        }
    }
    true
}

fn inside_margin(input: &PackInput, x: f64, y: f64, width: f64, height: f64) -> bool {
    x >= input.margin - FIT_EPS_MM
        && y >= input.margin - FIT_EPS_MM
        && x + width <= input.sheet_width - input.margin + FIT_EPS_MM
        && y + height <= input.sheet_height - input.margin + FIT_EPS_MM
}

/// True when the two rectangles are at least `need` apart along X or along Y.
fn separated(ax0: f64, ay0: f64, ax1: f64, ay1: f64, bx0: f64, by0: f64, bx1: f64, by1: f64, need: f64) -> bool {
    ax1 + need <= bx0 + FIT_EPS_MM || bx1 + need <= ax0 + FIT_EPS_MM || ay1 + need <= by0 + FIT_EPS_MM || by1 + need <= ay0 + FIT_EPS_MM
}

fn placed_size(width: f64, height: f64, rotation_deg: f64) -> (f64, f64) {
    match quarter_turns(rotation_deg) {
        Some(0 | 2) => (width, height),
        Some(1 | 3) => (height, width),
        _ => {
            let (sin_t, cos_t) = rotation_deg.to_radians().sin_cos();
            let (sin_abs, cos_abs) = (sin_t.abs(), cos_t.abs());
            (width * cos_abs + height * sin_abs, width * sin_abs + height * cos_abs)
        }
    }
}

fn rotate_xy(dx: f64, dy: f64, rotation_deg: f64) -> (f64, f64) {
    match quarter_turns(rotation_deg) {
        Some(0) => (dx, dy),
        Some(1) => (-dy, dx),
        Some(2) => (-dx, -dy),
        Some(3) => (dy, -dx),
        _ => {
            let (sin_t, cos_t) = rotation_deg.to_radians().sin_cos();
            (cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy)
        }
    }
}

/// 0, 1, 2 or 3 when `rotation_deg` is a quarter turn. `None` otherwise.
fn quarter_turns(rotation_deg: f64) -> Option<u8> {
    if !rotation_deg.is_finite() {
        return None;
    }
    let turns = rotation_deg / 90.0;
    let rounded = turns.round();
    if (turns - rounded).abs() > 1e-6 {
        return None;
    }
    let wrapped = rounded.rem_euclid(4.0);
    if (wrapped - 0.0).abs() <= 1e-6 {
        Some(0)
    } else if (wrapped - 1.0).abs() <= 1e-6 {
        Some(1)
    } else if (wrapped - 2.0).abs() <= 1e-6 {
        Some(2)
    } else if (wrapped - 3.0).abs() <= 1e-6 {
        Some(3)
    } else {
        None
    }
}

fn unique_sorted(mut values: Vec<f64>) -> Vec<f64> {
    values.sort_by(|left, right| cmp_float_key(*left, *right));
    let mut out = Vec::with_capacity(values.len());
    for value in values {
        if out.last().is_some_and(|prev| cmp_float_key(*prev, value) == Ordering::Equal) {
            continue;
        }
        out.push(value);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_geom::Tolerance;
    use proptest::prelude::*;

    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
    }

    fn part(name: &str, width: f64, height: f64, rotations: &[f64]) -> PackPart {
        PackPart { name: name.to_string(), width, height, rotations_deg: rotations.to_vec() }
    }

    fn sheet(width: f64, height: f64, gap: f64, margin: f64, tool_radius: f64, keep_out: Vec<PackRect>, parts: Vec<PackPart>) -> PackInput {
        PackInput { sheet_width: width, sheet_height: height, gap, margin, tool_radius, keep_out, parts }
    }

    fn placed(input: &PackInput) -> Vec<Placement> {
        pack(input).unwrap_or_else(|err| panic!("{err}"))
    }

    fn square_contour(x: f64, y: f64, size: f64) -> Contour {
        rect_contour(x, y, size, size)
    }

    fn rect_contour(x: f64, y: f64, width: f64, height: f64) -> Contour {
        let point = |px: f64, py: f64| Point::new(px, py).expect("point");
        let line = |a: Point, b: Point| Segment::Line(Line::new(a, b).expect("line"));
        let a = point(x, y);
        let b = point(x + width, y);
        let c = point(x + width, y + height);
        let d = point(x, y + height);
        Contour::try_new(vec![line(a, b), line(b, c), line(c, d), line(d, a)], true, &Tolerance::new(1e-6, 1e-6).expect("tol")).expect("contour")
    }

    #[test]
    fn two_squares_sit_side_by_side() {
        let input = sheet(40.0, 20.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("a", 20.0, 20.0, &[0.0]), part("b", 20.0, 20.0, &[0.0])]);
        let poses = placed(&input);
        assert_eq!(poses.len(), 2);
        near(poses[0].x, 0.0);
        near(poses[0].y, 0.0);
        near(poses[1].x, 20.0);
        near(poses[1].y, 0.0);
        check_placements(&input, &poses).expect("check");
    }

    #[test]
    fn gap_separates_identical_squares() {
        let input = sheet(50.0, 20.0, 6.0, 0.0, 0.0, Vec::new(), vec![part("a", 20.0, 20.0, &[0.0]), part("b", 20.0, 20.0, &[0.0])]);
        let poses = placed(&input);
        near(poses[1].x, 26.0);
        near(poses[1].y, 0.0);
    }

    #[test]
    fn margin_insets_the_first_corner() {
        let input = sheet(30.0, 20.0, 0.0, 4.0, 0.0, Vec::new(), vec![part("a", 10.0, 8.0, &[0.0])]);
        let poses = placed(&input);
        near(poses[0].x, 4.0);
        near(poses[0].y, 4.0);
    }

    #[test]
    fn keep_out_blocks_the_side_slot() {
        // The second square would sit at x = 20. The clamp occupies that slot, so it goes to x = 40.
        let keep = PackRect { min_x: 20.0, min_y: 0.0, max_x: 40.0, max_y: 20.0 };
        let input = sheet(60.0, 20.0, 0.0, 0.0, 0.0, vec![keep], vec![part("a", 20.0, 20.0, &[0.0]), part("b", 20.0, 20.0, &[0.0])]);
        let poses = placed(&input);
        near(poses[0].x, 0.0);
        near(poses[0].y, 0.0);
        near(poses[1].x, 40.0);
        near(poses[1].y, 0.0);
        check_placements(&input, &poses).expect("check");
    }

    #[test]
    fn tool_radius_pushes_a_part_off_a_clamp() {
        let keep = PackRect { min_x: 0.0, min_y: 0.0, max_x: 10.0, max_y: 20.0 };
        let input = sheet(50.0, 20.0, 0.0, 0.0, 5.0, vec![keep], vec![part("a", 10.0, 10.0, &[0.0])]);
        let poses = placed(&input);
        near(poses[0].x, 15.0);
        near(poses[0].y, 0.0);
    }

    #[test]
    fn lower_row_wins_over_a_higher_row() {
        let input = sheet(40.0, 30.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("a", 20.0, 20.0, &[0.0]), part("b", 20.0, 10.0, &[0.0])]);
        let poses = placed(&input);
        near(poses[1].x, 20.0);
        near(poses[1].y, 0.0);
    }

    #[test]
    fn input_order_not_size_order() {
        let large_first = sheet(40.0, 10.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("large", 30.0, 10.0, &[0.0]), part("small", 10.0, 10.0, &[0.0])]);
        let small_first = sheet(40.0, 10.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("small", 10.0, 10.0, &[0.0]), part("large", 30.0, 10.0, &[0.0])]);
        let large = placed(&large_first);
        let small = placed(&small_first);
        near(large[0].x, 0.0);
        near(large[1].x, 30.0);
        near(small[0].x, 0.0);
        near(small[1].x, 10.0);
    }

    #[test]
    fn quarter_turn_when_the_long_side_does_not_fit() {
        let input = sheet(15.0, 40.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("strip", 30.0, 10.0, &[0.0, 90.0])]);
        let poses = placed(&input);
        near(poses[0].rotation_deg, 90.0);
        near(poses[0].width, 10.0);
        near(poses[0].height, 30.0);
        near(poses[0].x, 0.0);
        near(poses[0].y, 0.0);
    }

    #[test]
    fn zero_rotation_wins_when_both_fit_at_the_same_corner() {
        let input = sheet(40.0, 40.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("strip", 30.0, 10.0, &[0.0, 90.0])]);
        let poses = placed(&input);
        near(poses[0].rotation_deg, 0.0);
        near(poses[0].x, 0.0);
        near(poses[0].y, 0.0);
    }

    #[test]
    fn part_larger_than_the_sheet_names_the_part() {
        let input = sheet(40.0, 40.0, 0.0, 0.0, 0.0, Vec::new(), vec![part("big", 50.0, 10.0, &[0.0, 90.0])]);
        let err = pack(&input).expect_err("too big");
        assert!(err.to_string().contains("part `big`"), "{err}");
        assert!(err.to_string().contains("does not fit"), "{err}");
    }

    #[test]
    fn margin_can_make_a_part_not_fit() {
        let input = sheet(20.0, 20.0, 0.0, 2.0, 0.0, Vec::new(), vec![part("wide", 18.0, 10.0, &[0.0])]);
        let err = pack(&input).expect_err("margin");
        assert!(err.to_string().contains("part `wide`"), "{err}");
    }

    #[test]
    fn check_rejects_an_overlap_and_a_clamp_hit() {
        let input = sheet(
            40.0,
            20.0,
            4.0,
            0.0,
            3.0,
            vec![PackRect { min_x: 0.0, min_y: 0.0, max_x: 5.0, max_y: 5.0 }],
            vec![part("a", 10.0, 10.0, &[0.0]), part("b", 10.0, 10.0, &[0.0])],
        );
        let overlapping = vec![
            Placement { name: "a".to_string(), x: 0.0, y: 0.0, rotation_deg: 0.0, width: 10.0, height: 10.0 },
            Placement { name: "b".to_string(), x: 10.0, y: 0.0, rotation_deg: 0.0, width: 10.0, height: 10.0 },
        ];
        let err = check_placements(&input, &overlapping).expect_err("gap");
        assert!(err.to_string().contains("closer than the gap"), "{err}");
        let clamped = vec![Placement { name: "a".to_string(), x: 6.0, y: 0.0, rotation_deg: 0.0, width: 10.0, height: 10.0 }];
        let one =
            sheet(40.0, 20.0, 0.0, 0.0, 3.0, vec![PackRect { min_x: 0.0, min_y: 0.0, max_x: 5.0, max_y: 5.0 }], vec![part("a", 10.0, 10.0, &[0.0])]);
        let err = check_placements(&one, &clamped).expect_err("clamp");
        assert!(err.to_string().contains("keep-out"), "{err}");
        let outside = vec![Placement { name: "a".to_string(), x: 35.0, y: 0.0, rotation_deg: 0.0, width: 10.0, height: 10.0 }];
        let err = check_placements(&one, &outside).expect_err("sheet");
        assert!(err.to_string().contains("outside the sheet"), "{err}");
    }

    #[test]
    fn translation_moves_a_square_onto_its_placement() {
        let contour = square_contour(0.0, 0.0, 20.0);
        let placement = Placement { name: "a".to_string(), x: 24.0, y: 0.0, rotation_deg: 0.0, width: 20.0, height: 20.0 };
        let frame = frame_for(0.0, 0.0, 20.0, 20.0, &placement);
        let moved = transform_contours(&[contour], &frame, Tolerance::new(1e-6, 1e-6).expect("tol")).unwrap_or_else(|err| panic!("{err}"));
        let bounds = moved.first().expect("contour").bounding_box().expect("bounds");
        near(bounds.min().x(), 24.0);
        near(bounds.min().y(), 0.0);
        near(bounds.max().x(), 44.0);
        near(bounds.max().y(), 20.0);
    }

    #[test]
    fn quarter_turn_maps_the_rectangle_aabb() {
        let contour = rect_contour(0.0, 0.0, 30.0, 10.0);
        let placement = Placement { name: "strip".to_string(), x: 0.0, y: 0.0, rotation_deg: 90.0, width: 10.0, height: 30.0 };
        let frame = frame_for(0.0, 0.0, 30.0, 10.0, &placement);
        let moved = transform_contours(&[contour], &frame, Tolerance::new(1e-6, 1e-6).expect("tol")).unwrap_or_else(|err| panic!("{err}"));
        let bounds = moved.first().expect("contour").bounding_box().expect("bounds");
        near(bounds.min().x(), 0.0);
        near(bounds.min().y(), 0.0);
        near(bounds.max().x(), 10.0);
        near(bounds.max().y(), 30.0);
    }

    #[test]
    fn tool_radius_choice() {
        near(choose_tool_radius(Some(1.0), [5.0, 2.0]), 1.0);
        near(choose_tool_radius(None, [2.0, 5.0, 1.0]), 5.0);
        near(choose_tool_radius(None, Vec::<f64>::new()), 0.0);
    }

    fn pack_config() -> ProptestConfig {
        ProptestConfig { cases: 32, rng_seed: proptest::test_runner::RngSeed::Fixed(0x25C0_5E57), ..ProptestConfig::default() }
    }

    proptest! {
        #![proptest_config(pack_config())]
        #[test]
        fn same_input_same_placements(
            sheet_w in 40.0_f64..160.0,
            sheet_h in 40.0_f64..160.0,
            gap in 0.0_f64..5.0,
            margin in 0.0_f64..4.0,
            tool_radius in 0.0_f64..3.0,
            count in 1_usize..5,
            seed in 0_u64..20_000,
        ) {
            let mut state = seed | 1;
            let mut next = || {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                state
            };
            let mut parts = Vec::new();
            for index in 0..count {
                let width = 1.0 + f64::from((next() % 25) as u32);
                let height = 1.0 + f64::from((next() % 25) as u32);
                let rotations = if next() % 2 == 0 { vec![0.0] } else { vec![0.0, 90.0] };
                parts.push(part(&format!("p{index}"), width, height, &rotations));
            }
            let keep_out = if next() % 2 == 0 {
                Vec::new()
            } else {
                let x = f64::from((next() % 30) as u32);
                let y = f64::from((next() % 30) as u32);
                vec![PackRect { min_x: x, min_y: y, max_x: x + 8.0, max_y: y + 6.0 }]
            };
            let input = sheet(sheet_w, sheet_h, gap, margin, tool_radius, keep_out, parts);
            let first = pack(&input);
            let second = pack(&input);
            match (&first, &second) {
                (Ok(left), Ok(right)) => {
                    prop_assert_eq!(left, right);
                    prop_assert_eq!(left.len(), input.parts.len());
                    for (pose, part) in left.iter().zip(input.parts.iter()) {
                        prop_assert_eq!(&pose.name, &part.name);
                    }
                    prop_assert!(check_placements(&input, left).is_ok());
                }
                (Err(left), Err(right)) => {
                    prop_assert_eq!(left, right);
                    prop_assert!(left.to_string().contains("does not fit"), "{}", left);
                }
                _ => prop_assert!(false, "pack changed between calls"),
            }
        }
    }
}
