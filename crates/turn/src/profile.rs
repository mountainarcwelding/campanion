//! Open lathe half-profile: validation, radius queries, and drawing import.

use campanion_geom::{Arc, Contour, Line, Point, Segment, Tolerance};
use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg};
use campanion_mesh::Mesh;

use crate::mesh::MeshAxis;
use crate::{MAX_PROFILE_SEGMENTS, Result, TurnError};

/// A vertical line is one whose Z span is at most this (millimetres).
const VERTICAL_DZ: f64 = 1e-12;

/// Parameter slack so an endpoint of a line still counts as a hit.
const PARAM_SLACK: f64 = 1e-12;

/// An open half-profile of a turned part.
///
/// `segments` is a contiguous chain of lines and arcs. Every point has radius
/// `>= -tol` from [`Self::try_new`] (about −1e-9 mm for ordinary float noise).
/// The chain runs from the front toward the chuck: [`Self::start`]`.x()` is at
/// least [`Self::end`]`.x()`.
#[derive(Clone, Debug, PartialEq)]
pub struct TurnProfile {
    segments: Vec<Segment>,
    start: Point,
    end: Point,
}

impl TurnProfile {
    /// Validate an open chain and orient it from the front (larger Z) toward the chuck.
    ///
    /// `tol` is millimetres. Joints must lie within `tol`; they are checked, not snapped.
    /// The chain must be non-empty and no longer than [`MAX_PROFILE_SEGMENTS`].
    /// Every point, including arc extremes taken from the segment bounding box, must
    /// have radius `>= -tol`. A chain whose start Z is less than its end Z is reversed.
    pub fn try_new(mut segments: Vec<Segment>, tol: f64) -> Result<Self> {
        check_tol(tol)?;
        if segments.is_empty() {
            return Err(TurnError::InvalidInput("profile is empty".into()));
        }
        if segments.len() > MAX_PROFILE_SEGMENTS {
            return Err(TurnError::InvalidInput(format!("too many segments: {} > {MAX_PROFILE_SEGMENTS}", segments.len())));
        }
        for pair in segments.windows(2) {
            let (Some(prev), Some(next)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            let gap = prev.end().distance(next.start());
            if !gap.is_finite() || gap > tol {
                return Err(TurnError::InvalidInput(format!("segments do not connect within {tol} (gap {gap})")));
            }
        }
        for segment in &segments {
            let min_y = segment.bounding_box().min().y();
            if !min_y.is_finite() || min_y < -tol {
                return Err(TurnError::InvalidInput(format!("radius {min_y} is below -{tol}")));
            }
        }
        let (start_z, end_z) = match (segments.first(), segments.last()) {
            (Some(first), Some(last)) => (first.start().x(), last.end().x()),
            _ => return Err(TurnError::InvalidInput("profile is empty".into())),
        };
        if start_z < end_z {
            segments = reverse_chain(segments);
        }
        let (Some(first), Some(last)) = (segments.first(), segments.last()) else {
            return Err(TurnError::InvalidInput("profile is empty".into()));
        };
        Ok(Self { start: first.start(), end: last.end(), segments })
    }

    /// Lines and arcs from the front toward the chuck.
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    /// Front end of the chain. Its Z is at least [`Self::end`]'s Z.
    pub fn start(&self) -> Point {
        self.start
    }

    /// Chuck end of the chain.
    pub fn end(&self) -> Point {
        self.end
    }

    /// Minimum and maximum Z of the chain, including arc extremes.
    pub fn z_range(&self) -> (f64, f64) {
        let mut zmin = f64::INFINITY;
        let mut zmax = f64::NEG_INFINITY;
        for segment in &self.segments {
            let bounds = segment.bounding_box();
            zmin = zmin.min(bounds.min().x());
            zmax = zmax.max(bounds.max().x());
        }
        if !zmin.is_finite() || !zmax.is_finite() { (0.0, 0.0) } else { (zmin, zmax) }
    }

    /// Largest radius on the chain. Arc extremes come from the bounding box.
    pub fn max_radius(&self) -> f64 {
        let mut best = f64::NEG_INFINITY;
        for segment in &self.segments {
            best = best.max(segment.bounding_box().max().y());
        }
        if best.is_finite() { best } else { 0.0 }
    }

    /// Smallest radius on the chain. Arc extremes come from the bounding box.
    pub fn min_radius(&self) -> f64 {
        let mut best = f64::INFINITY;
        for segment in &self.segments {
            best = best.min(segment.bounding_box().min().y());
        }
        if best.is_finite() { best } else { 0.0 }
    }

    /// Largest radius where the vertical line `x = z` meets the profile.
    ///
    /// Lines are interpolated. A segment of constant Z contributes its maximum radius.
    /// An arc is solved against its circle and only the hits that lie on the sweep are
    /// kept. Returns `None` outside [`Self::z_range`], or when `z` is not finite.
    pub fn radius_at(&self, z: f64) -> Option<f64> {
        if !z.is_finite() {
            return None;
        }
        let (zmin, zmax) = self.z_range();
        if z < zmin || z > zmax {
            return None;
        }
        let mut best: Option<f64> = None;
        for segment in &self.segments {
            if let Some(radius) = segment_radius_at(*segment, z) {
                best = Some(best.map_or(radius, |current| current.max(radius)));
            }
        }
        best
    }

    /// Build a profile from the largest contour in `drawing`.
    ///
    /// When `layer` is set, only that layer is considered. The winner is the contour
    /// with the largest bounding-box area. A closed contour is opened by deleting the
    /// axis run and rotating so the chain starts just after it. An open contour is
    /// used as it stands. If most of the length lies at negative radius, Y is mirrored
    /// before the axis is removed and the chain is validated.
    ///
    /// [`TurnError::NoProfile`] means no contour matched.
    pub fn from_drawing(drawing: &Drawing, layer: Option<&str>, tol: f64) -> Result<Self> {
        check_tol(tol)?;
        let contour = largest_contour(drawing, layer)?;
        let mut segments = contour.segments().to_vec();
        if mostly_negative(&segments, tol) {
            segments = mirror_chain(&segments)?;
        }
        if contour.is_closed() {
            segments = open_along_axis(&segments, tol)?;
        }
        Self::try_new(segments, tol)
    }

    /// Import an ASCII DXF half-profile.
    ///
    /// `tol` is the linear tolerance in millimetres. The angular tolerance is the
    /// geometry default. Drawing X is Z and drawing Y is the radius.
    pub fn from_dxf(text: &str, layer: Option<&str>, tol: f64) -> Result<Self> {
        let tolerance = import_tolerance(tol)?;
        let drawing = import_dxf(text, &ImportOptions::new(tolerance))?;
        Self::from_drawing(&drawing, layer, tol)
    }

    /// Import an SVG half-profile.
    ///
    /// `tol` is the linear tolerance in millimetres. SVG Y is flipped into the DXF
    /// plane by the importer; drawing X is then Z and drawing Y is the radius.
    pub fn from_svg(text: &str, tol: f64) -> Result<Self> {
        let tolerance = import_tolerance(tol)?;
        let drawing = import_svg(text, &ImportOptions::new(tolerance))?;
        Self::from_drawing(&drawing, None, tol)
    }

    /// Outside-diameter envelope of a mesh already centred on `axis`.
    ///
    /// Planes are taken at `zmin + i * step`, plus the exact maximum Z. Each sample is
    /// the maximum radial distance of the triangle edges that cross the plane, together
    /// with vertices that lie on the plane and vertices within `step / 2` of it (so a
    /// sharp shoulder between samples is not missed). The result is an open polyline
    /// from the front (maximum Z) toward the chuck. Consecutive points that are
    /// collinear within 1e-9 mm are collapsed. The ends are left open: no radial leg
    /// is added down to the axis.
    ///
    /// This is the outside envelope only. An internal bore is not recovered. `step`
    /// must be finite and `> 0` and must not produce more than [`MAX_MESH_SAMPLES`]
    /// samples.
    ///
    /// STEP input is not meshed here. Section the solid with `solidcraft-cli` and
    /// read that DXF with [`TurnProfile::from_section_dxf`].
    pub fn from_mesh(mesh: &Mesh, axis: MeshAxis, step: f64) -> Result<Self> {
        let segments = crate::mesh::envelope(mesh, axis, step)?;
        Self::try_new(segments, 1e-9)
    }

    /// Sample the chain as points. Arcs become chords whose sagitta is within `tol`.
    ///
    /// A non-finite or non-positive `tol` is treated as 1e-4 mm so the call still
    /// returns. Each arc is split into at most 10 000 chords.
    pub fn to_polyline_points(&self, tol: f64) -> Vec<Point> {
        let mut points = Vec::new();
        for segment in &self.segments {
            push_if_apart(&mut points, segment.start());
            match *segment {
                Segment::Line(line) => push_if_apart(&mut points, line.end()),
                Segment::Arc(arc) => {
                    let steps = arc_chord_steps(arc, tol);
                    let steps_f = steps as f64;
                    for step in 1..=steps {
                        push_if_apart(&mut points, arc.point_at(step as f64 / steps_f));
                    }
                }
            }
        }
        points
    }
}

fn check_tol(tol: f64) -> Result<()> {
    if tol.is_finite() && tol >= 0.0 { Ok(()) } else { Err(TurnError::InvalidInput("tolerance must be finite and >= 0".into())) }
}

pub(crate) fn import_tolerance(tol: f64) -> Result<Tolerance> {
    check_tol(tol)?;
    Ok(Tolerance::new(tol, Tolerance::DEFAULT_ANGULAR_RAD)?)
}

fn reverse_chain(segments: Vec<Segment>) -> Vec<Segment> {
    segments.into_iter().rev().map(Segment::reversed).collect()
}

fn push_if_apart(points: &mut Vec<Point>, point: Point) {
    if points.last().is_none_or(|prev| prev.distance(point) > 1e-12) {
        points.push(point);
    }
}

/// Chord count so the sagitta `R (1 - cos(δ / 2))` stays within `tol`.
fn arc_chord_steps(arc: Arc, tol: f64) -> usize {
    let sweep = arc.sweep().abs();
    if !(sweep > 1e-15) {
        return 1;
    }
    let tol = if tol.is_finite() && tol > 0.0 { tol } else { 1e-4 };
    let radius = arc.radius();
    let full_sagitta = radius * (1.0 - (sweep * 0.5).cos());
    if !full_sagitta.is_finite() || full_sagitta <= tol {
        return 1;
    }
    let cos_half = (1.0 - tol / radius).clamp(-1.0, 1.0);
    let delta = (2.0 * cos_half.acos()).max(1e-6);
    let n = (sweep / delta).ceil();
    if !n.is_finite() || n < 1.0 {
        return 1;
    }
    if n > 10_000.0 { 10_000 } else { n as usize }
}

fn segment_radius_at(segment: Segment, z: f64) -> Option<f64> {
    match segment {
        Segment::Line(line) => line_radius_at(line, z),
        Segment::Arc(arc) => arc_radius_at(arc, z),
    }
}

fn line_radius_at(line: Line, z: f64) -> Option<f64> {
    let z0 = line.start().x();
    let z1 = line.end().x();
    let r0 = line.start().y();
    let r1 = line.end().y();
    let dz = z1 - z0;
    if dz.abs() <= VERTICAL_DZ {
        let low = z0.min(z1) - PARAM_SLACK;
        let high = z0.max(z1) + PARAM_SLACK;
        if z >= low && z <= high { Some(r0.max(r1)) } else { None }
    } else {
        let t = (z - z0) / dz;
        if !t.is_finite() || !(-PARAM_SLACK..=1.0 + PARAM_SLACK).contains(&t) {
            return None;
        }
        let radius = r0 + t.clamp(0.0, 1.0) * (r1 - r0);
        if radius.is_finite() { Some(radius) } else { None }
    }
}

fn arc_radius_at(arc: Arc, z: f64) -> Option<f64> {
    let bounds = arc.bounding_box();
    if z < bounds.min().x() || z > bounds.max().x() {
        return None;
    }
    let center = arc.center();
    let radius = arc.radius();
    let dx = z - center.x();
    let mut disc = radius * radius - dx * dx;
    // A tangent sits on the bounding-box extreme; float error can make `disc` tiny and negative.
    if disc < 0.0 && disc > -1e-8 * radius.max(1.0) {
        disc = 0.0;
    }
    let mut best: Option<f64> = None;
    if disc >= 0.0 {
        let root = disc.sqrt();
        for y in [center.y() + root, center.y() - root] {
            if !y.is_finite() {
                continue;
            }
            let Ok(point) = Point::new(z, y) else {
                continue;
            };
            let theta = (point - center).angle();
            if angle_on_sweep(arc.start_angle(), arc.sweep(), theta) {
                best = Some(best.map_or(y, |current| current.max(y)));
            }
        }
    }
    for point in [arc.start_point(), arc.end_point()] {
        if (point.x() - z).abs() <= 1e-12 {
            best = Some(best.map_or(point.y(), |current| current.max(point.y())));
        }
    }
    best
}

/// `true` when `theta` lies on the signed sweep that starts at `start`.
///
/// The slack matches [`campanion_geom::Arc`]'s own sweep test so endpoints count.
fn angle_on_sweep(start: f64, sweep: f64, theta: f64) -> bool {
    if !theta.is_finite() || !start.is_finite() || !sweep.is_finite() {
        return false;
    }
    const TAU: f64 = std::f64::consts::TAU;
    let abs_sweep = sweep.abs();
    if abs_sweep < 1e-15 {
        return false;
    }
    if abs_sweep >= TAU - 1e-8 {
        return true;
    }
    let along = if sweep > 0.0 { (theta - start).rem_euclid(TAU) } else { (start - theta).rem_euclid(TAU) };
    let slack = 1e-8;
    let adjusted = if TAU - along <= slack { 0.0 } else { along };
    adjusted <= abs_sweep + slack
}

fn largest_contour<'a>(drawing: &'a Drawing, layer: Option<&str>) -> Result<&'a Contour> {
    let mut best: Option<&Contour> = None;
    let mut best_area = f64::NEG_INFINITY;
    for imported in drawing.contours() {
        if let Some(wanted) = layer
            && imported.layer() != wanted
        {
            continue;
        }
        let contour = imported.contour();
        let bounds = contour.bounding_box()?;
        let area = bounds.width() * bounds.height();
        if !area.is_finite() {
            return Err(TurnError::InvalidInput("contour bounds are not finite".into()));
        }
        if best.is_none() || area > best_area {
            best = Some(contour);
            best_area = area;
        }
    }
    best.ok_or_else(|| match layer {
        Some(name) => TurnError::NoProfile(format!("no contour on layer {name}")),
        None => TurnError::NoProfile("drawing has no contour".into()),
    })
}

/// Length below `-tol` exceeds length above `+tol`. Samples are the ends and the midpoint.
fn mostly_negative(segments: &[Segment], tol: f64) -> bool {
    let mut negative = 0.0;
    let mut positive = 0.0;
    for segment in segments {
        let len = segment.length();
        if !len.is_finite() || !(len > 0.0) {
            continue;
        }
        let samples = match *segment {
            Segment::Line(line) => {
                let y0 = line.start().y();
                let y1 = line.end().y();
                [y0, 0.5 * (y0 + y1), y1]
            }
            Segment::Arc(arc) => [arc.point_at(0.0).y(), arc.point_at(0.5).y(), arc.point_at(1.0).y()],
        };
        let piece = len / 3.0;
        for y in samples {
            if y < -tol {
                negative += piece;
            } else if y > tol {
                positive += piece;
            }
        }
    }
    negative > positive
}

fn mirror_chain(segments: &[Segment]) -> Result<Vec<Segment>> {
    let mut mirrored = Vec::with_capacity(segments.len());
    for segment in segments {
        mirrored.push(mirror_segment(*segment)?);
    }
    Ok(mirrored)
}

fn mirror_point(point: Point) -> Result<Point> {
    Ok(Point::new(point.x(), -point.y())?)
}

pub(crate) fn mirror_segment(segment: Segment) -> Result<Segment> {
    match segment {
        Segment::Line(line) => {
            let start = mirror_point(line.start())?;
            let end = mirror_point(line.end())?;
            Ok(Segment::Line(Line::new(start, end)?))
        }
        Segment::Arc(arc) => {
            let center = mirror_point(arc.center())?;
            // Reflecting Y sends angle θ to −θ and reverses the sense of the sweep.
            let mirrored = Arc::new(center, arc.radius(), -arc.start_angle(), -arc.sweep())?;
            Ok(Segment::Arc(mirrored))
        }
    }
}

/// Drop the spindle-axis run of a closed contour and rotate to the first kept segment.
fn open_along_axis(segments: &[Segment], tol: f64) -> Result<Vec<Segment>> {
    if segments.is_empty() {
        return Err(TurnError::NoProfile("closed contour is empty".into()));
    }
    let min_radius = segments.iter().map(|segment| segment.bounding_box().min().y()).fold(f64::INFINITY, f64::min);
    let drop: Vec<bool> = segments.iter().map(|segment| is_axis_segment(*segment, min_radius, tol)).collect();
    let n = drop.len();
    let kept = drop.iter().filter(|flag| !**flag).count();
    if kept == 0 {
        return Err(TurnError::NoProfile("contour lies on the spindle axis".into()));
    }
    if kept == n {
        return Err(TurnError::InvalidInput("closed contour does not meet the spindle axis".into()));
    }
    if axis_runs(&drop) != 1 {
        return Err(TurnError::InvalidInput("axis segments are not one contiguous run".into()));
    }
    let Some(start) = chain_start(&drop) else {
        return Err(TurnError::InvalidInput("could not rotate the profile to the front of the axis cut".into()));
    };
    let mut chain = Vec::with_capacity(kept);
    for k in 0..kept {
        let index = (start + k) % n;
        let Some(segment) = segments.get(index).copied() else {
            return Err(TurnError::InvalidInput("segment index missing".into()));
        };
        chain.push(segment);
    }
    Ok(chain)
}

fn is_axis_segment(segment: Segment, min_radius: f64, tol: f64) -> bool {
    let y0 = segment.start().y();
    let y1 = segment.end().y();
    if matches!(segment, Segment::Line(_)) && y0.abs() <= tol && y1.abs() <= tol {
        return true;
    }
    // A closed half-profile's axis is the run at the contour's minimum radius, and
    // only when that radius is the spindle (within `tol` of zero).
    if min_radius > tol {
        return false;
    }
    let bounds = segment.bounding_box();
    let low = bounds.min().y();
    let high = bounds.max().y();
    (high - low) <= tol && (low - min_radius).abs() <= tol && (high - min_radius).abs() <= tol
}

fn axis_runs(drop: &[bool]) -> usize {
    let n = drop.len();
    if n == 0 {
        return 0;
    }
    let mut runs = 0usize;
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let prev_drop = drop.get(prev).copied().unwrap_or(false);
        let cur_drop = drop.get(i).copied().unwrap_or(false);
        if cur_drop && !prev_drop {
            runs += 1;
        }
    }
    runs
}

fn chain_start(drop: &[bool]) -> Option<usize> {
    let n = drop.len();
    if n == 0 {
        return None;
    }
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let prev_drop = drop.get(prev).copied()?;
        let cur_drop = drop.get(i).copied()?;
        if !cur_drop && prev_drop {
            return Some(i);
        }
    }
    None
}
