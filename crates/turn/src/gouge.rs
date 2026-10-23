//! Insert, edge and holder gouge checks for a turning toolpath.
//!
//! The section is a polyline in the `(Z, radius)` plane. `+Z` points away from the chuck.
//! Insert angles are measured from `+Z` toward `+radius`. [`InsertGeom::orientation_deg`] is the
//! bisector from the nose centre into the holder.
//!
//! The nose is a circle. It gouges when `nose_radius` exceeds the signed distance from the nose
//! centre to the section by more than the tolerance. Signed distance is positive on the free
//! side. Cutting edges are the two flanks at `±included/2` about that bisector, each
//! [`InsertGeom::edge_length`] long, closed across the back of the insert. The holder is a
//! rectangle of [`HolderGeom`] centred on the same bisector, starting at that back edge and
//! extending `length` into the shank.
//!
//! For [`Side::Outside`] the material is the solid inside the section: `Z` within the section's
//! range and `0 <= radius` below the boundary. In front of the part (and past the chuck end) is
//! free. For [`Side::Inside`] the material is outside the bore, up to a large outer radius.
//! A point's penetration is its distance to the section when it lies in that solid.
//!
//! Feed, arc and thread moves are sampled every `sample_step`, including both endpoints. Rapids
//! are sampled the same way: a rapid into the solid is a gouge of whichever part hits. The
//! deepest sample of each kind on a move is kept. [`TurnGouge::toolpath`] is `0`; a caller that
//! checks several paths sets the index.
//!
//! This module is clean-room. It does not use OCCT, FreeCAD, planegcs, or OpenCAMLib.

use std::cell::Cell;
use std::f64::consts::{PI, TAU};

use campanion_ir::InsertShape;

use crate::ops::{self, Side, TipPoint, TurnMove, TurnPath};
use crate::{Result, TurnError};

/// Samples one gouge check may take. A smaller step on a long path is rejected.
const MAX_GOUGE_SAMPLES: usize = 2_000_000;

/// Inside bore material stops at this radius, in millimetres, unless the section is larger.
const ID_OUTER_RADIUS: f64 = 1.0e6;

/// A point this close to the section is on the boundary, not in the solid.
const BOUNDARY_BAND: f64 = 1.0e-9;

/// Included angle stored for a round insert. The outline is the full nose circle.
const ROUND_ANGLE_DEG: f64 = 360.0;

/// Nose circle, flank angle and shank direction for one turning insert.
///
/// Angles are degrees. `orientation_deg` points from the nose centre into the holder.
/// `width` is the blade width of a grooving insert; pointed inserts leave it unset.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InsertGeom {
    /// Nose radius, millimetres. Zero is a sharp corner.
    pub nose_radius: f64,
    /// Included point angle, degrees. `0` is a grooving blade. `360` is a round insert.
    pub included_angle_deg: f64,
    /// Bisector from the nose centre into the holder, degrees from `+Z` toward `+radius`.
    pub orientation_deg: f64,
    /// Length of each cutting flank, millimetres.
    pub edge_length: f64,
    /// Grooving blade width, millimetres.
    pub width: Option<f64>,
}

impl InsertGeom {
    /// Build geometry from an IR insert.
    ///
    /// `edge_length` is the flank length in millimetres and must be finite and positive.
    /// A round insert uses a full circle. A groove insert uses its width and parallel flanks.
    /// Shapes with an included angle use that angle. The nose radius and orientation are copied.
    pub fn from_insert(insert: &campanion_ir::Insert, edge_length: f64) -> Result<Self> {
        if !edge_length.is_finite() || edge_length <= 0.0 {
            return Err(invalid("edge length must be finite and positive"));
        }
        let included_angle_deg = match insert.shape().included_angle_deg() {
            Some(angle) => angle,
            None if insert.shape() == InsertShape::R => ROUND_ANGLE_DEG,
            None if insert.shape() == InsertShape::Groove => {
                if insert.width().is_none() {
                    return Err(invalid("groove insert needs a width"));
                }
                0.0
            }
            None => return Err(invalid("insert has no included angle")),
        };
        if let Some(width) = insert.width()
            && !(width.is_finite() && width > 0.0)
        {
            return Err(invalid("insert width must be finite and positive"));
        }
        Ok(Self {
            nose_radius: insert.nose_radius(),
            included_angle_deg,
            orientation_deg: insert.orientation_deg(),
            edge_length,
            width: insert.width(),
        })
    }

    /// Closed outline of the insert with the nose centre at `nose_centre`.
    ///
    /// `tol` is the chord height of the nose arc, in millimetres. A non-finite or non-positive
    /// `tol` is treated as `1e-4`. The ring runs around the cutting arc, out along one flank,
    /// across the back, and back along the other flank. An invalid insert returns an empty ring.
    pub fn outline(&self, nose_centre: (f64, f64), tol: f64) -> Vec<(f64, f64)> {
        let Ok(shape) = insert_shape(self, nose_centre) else {
            return Vec::new();
        };
        let tol = if tol.is_finite() && tol > 0.0 { tol } else { 1.0e-4 };
        let mut points = Vec::new();
        if shape.arc_span.abs() > 1e-12 && shape.nose_radius > 0.0 {
            let steps = arc_steps(shape.nose_radius, shape.arc_span, tol).max(1);
            let steps_f = steps as f64;
            for step in 0..=steps {
                let angle = shape.arc_start + shape.arc_span * (step as f64 / steps_f);
                push_apart(&mut points, (nose_centre.0 + shape.nose_radius * angle.cos(), nose_centre.1 + shape.nose_radius * angle.sin()));
            }
        } else if let Some(first) = shape.flanks.first() {
            push_apart(&mut points, first.start);
        }
        for flank in &shape.flanks {
            push_apart(&mut points, flank.end);
        }
        if let Some(first) = points.first().copied()
            && points.last().is_none_or(|last| hypot2(*last, first) > 1e-12)
        {
            points.push(first);
        }
        points
    }
}

/// Shank rectangle behind the insert.
///
/// The rectangle is centred on the insert bisector. It starts at the back of the insert and
/// extends `length` millimetres further into the holder. `width` is the size across the bisector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HolderGeom {
    /// Width across the bisector, millimetres.
    pub width: f64,
    /// Length along the bisector, millimetres.
    pub length: f64,
}

/// Which part of the tool entered the solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum GougeKind {
    /// Nose circle crossed the section.
    Nose,
    /// A cutting flank or the back of the insert entered the solid.
    Edge,
    /// The holder rectangle entered the solid.
    Holder,
}

impl GougeKind {
    /// Short name used in reports: `nose`, `edge` or `holder`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Nose => "nose",
            Self::Edge => "edge",
            Self::Holder => "holder",
        }
    }
}

/// One gouge sample. `depth` is the deepest penetration of `what` on `move_index`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurnGouge {
    /// Index of the path in a program. This check sets it to `0`.
    pub toolpath: usize,
    /// Index into [`TurnPath::moves`].
    pub move_index: usize,
    /// Axial position of the sample, millimetres.
    pub z: f64,
    /// Radius of the sample, millimetres.
    pub x: f64,
    /// Penetration into the solid, millimetres.
    pub depth: f64,
    /// Which part of the tool hit.
    pub what: GougeKind,
}

/// Sample `path` against `section` and return penetrations deeper than `tol`.
///
/// `section` points are `(Z, radius)`, ordered along the half-profile. `tip` converts each
/// programmed point to a nose centre. `sample_step` and `tol` are millimetres. `sample_step`
/// must be finite and positive. `tol` must be finite and `>= 0`. An empty section, a non-finite
/// coordinate, or a non-positive step is [`TurnError::InvalidInput`].
pub fn check_turn_gouges(
    section: &[(f64, f64)],
    side: Side,
    path: &TurnPath,
    tip: TipPoint,
    insert: &InsertGeom,
    holder: Option<&HolderGeom>,
    sample_step: f64,
    tol: f64,
) -> Result<Vec<TurnGouge>> {
    if !sample_step.is_finite() || sample_step <= 0.0 {
        return Err(invalid("sample step must be finite and positive"));
    }
    if !tol.is_finite() || tol < 0.0 {
        return Err(invalid("tolerance must be finite and >= 0"));
    }
    validate_insert(insert)?;
    if let Some(holder) = holder {
        validate_holder(holder)?;
    }
    let shape_at = |centre: (f64, f64)| insert_shape(insert, centre);
    // Reject a shape that cannot be built before walking the path.
    shape_at((0.0, 0.0))?;
    let region = Region::new(section, side)?;
    finite_pair(path.start, "path start")?;
    let (dz, dx) = tip_shift(tip)?;
    let budget = Budget::new(MAX_GOUGE_SAMPLES);
    let mut cursor = path.start;
    let mut gouges = Vec::new();
    for (move_index, motion) in path.moves.iter().copied().enumerate() {
        let end = motion_end(cursor, motion)?;
        match motion {
            TurnMove::Dwell { seconds } => {
                if !seconds.is_finite() || seconds < 0.0 {
                    return Err(invalid("dwell must be finite and >= 0"));
                }
            }
            TurnMove::Rapid { .. } | TurnMove::Feed { .. } | TurnMove::Thread { .. } | TurnMove::Arc { .. } => {
                let mut worst = Worst::default();
                sample_motion(cursor, motion, sample_step, &budget, &mut |point| {
                    let centre = (point.0 - dz, point.1 - dx);
                    finite_pair(centre, "nose centre")?;
                    let shape = shape_at(centre)?;
                    inspect(&region, &shape, holder, sample_step, tol, &budget, &mut worst)
                })?;
                push_worst(&mut gouges, move_index, &worst);
            }
        }
        cursor = end;
    }
    gouges.sort_by(|left, right| right.depth.total_cmp(&left.depth).then(left.move_index.cmp(&right.move_index)).then(left.what.cmp(&right.what)));
    Ok(gouges)
}

#[derive(Clone, Copy)]
struct Flank {
    start: (f64, f64),
    end: (f64, f64),
}

struct InsertShapeGeom {
    nose_radius: f64,
    arc_start: f64,
    arc_span: f64,
    flanks: Vec<Flank>,
    /// Unit bisector, nose centre toward the holder, `(Z, radius)`.
    axis: (f64, f64),
    /// Unit perpendicular, `(Z, radius)`.
    perp: (f64, f64),
    /// Back of the insert, where the holder rectangle starts.
    back: (f64, f64),
}

fn insert_shape(insert: &InsertGeom, centre: (f64, f64)) -> Result<InsertShapeGeom> {
    let theta = insert.orientation_deg.to_radians();
    if !theta.is_finite() {
        return Err(invalid("insert orientation is not finite"));
    }
    let axis = (theta.cos(), theta.sin());
    let perp = (-theta.sin(), theta.cos());
    let radius = insert.nose_radius;
    if insert.included_angle_deg >= 180.0 {
        let back = add(centre, scale(axis, radius));
        return Ok(InsertShapeGeom { nose_radius: radius, arc_start: 0.0, arc_span: TAU, flanks: Vec::new(), axis, perp, back });
    }
    if insert.included_angle_deg.abs() <= 1e-9 {
        return groove_shape(insert, centre, axis, perp);
    }
    let alpha = insert.orientation_ok_angle()?;
    let half = alpha * 0.5;
    let outward = theta + PI;
    let offset = PI * 0.5 - half;
    let a0 = outward - offset;
    let a1 = outward + offset;
    let arc_span = a1 - a0;
    let t0 = if radius > 0.0 { add(centre, scale((a0.cos(), a0.sin()), radius)) } else { centre };
    let t1 = if radius > 0.0 { add(centre, scale((a1.cos(), a1.sin()), radius)) } else { centre };
    let d0 = flank_dir(a0, axis);
    let d1 = flank_dir(a1, axis);
    let e0 = add(t0, scale(d0, insert.edge_length));
    let e1 = add(t1, scale(d1, insert.edge_length));
    let back = ((e0.0 + e1.0) * 0.5, (e0.1 + e1.1) * 0.5);
    Ok(InsertShapeGeom {
        nose_radius: radius,
        arc_start: a0,
        arc_span,
        flanks: vec![Flank { start: t0, end: e0 }, Flank { start: t1, end: e1 }, Flank { start: e0, end: e1 }],
        axis,
        perp,
        back,
    })
}

impl InsertGeom {
    fn orientation_ok_angle(&self) -> Result<f64> {
        let alpha = self.included_angle_deg.to_radians();
        if alpha.is_finite() && alpha > 1e-6 && alpha < PI - 1e-6 {
            Ok(alpha)
        } else {
            Err(invalid("included angle must be between 0 and 180 degrees"))
        }
    }
}

fn groove_shape(insert: &InsertGeom, centre: (f64, f64), axis: (f64, f64), perp: (f64, f64)) -> Result<InsertShapeGeom> {
    let Some(width) = insert.width else {
        return Err(invalid("groove insert needs a width"));
    };
    if !(width.is_finite() && width > 0.0) {
        return Err(invalid("insert width must be finite and positive"));
    }
    let half = width * 0.5;
    let left = add(centre, scale(perp, half));
    let right = add(centre, scale(perp, -half));
    let back_left = add(left, scale(axis, insert.edge_length));
    let back_right = add(right, scale(axis, insert.edge_length));
    let back = add(centre, scale(axis, insert.edge_length));
    Ok(InsertShapeGeom {
        nose_radius: insert.nose_radius,
        arc_start: 0.0,
        arc_span: 0.0,
        flanks: vec![
            Flank { start: left, end: right },
            Flank { start: right, end: back_right },
            Flank { start: back_right, end: back_left },
            Flank { start: back_left, end: left },
        ],
        axis,
        perp,
        back,
    })
}

/// Perpendicular to the radius at `tangent_angle` that points toward `axis` (the holder).
fn flank_dir(tangent_angle: f64, axis: (f64, f64)) -> (f64, f64) {
    let candidates = [tangent_angle + PI * 0.5, tangent_angle - PI * 0.5];
    let mut best = candidates[0];
    let mut best_dot = f64::NEG_INFINITY;
    for angle in candidates {
        let dir = (angle.cos(), angle.sin());
        let dot = dir.0 * axis.0 + dir.1 * axis.1;
        if dot > best_dot {
            best_dot = dot;
            best = angle;
        }
    }
    (best.cos(), best.sin())
}

fn validate_insert(insert: &InsertGeom) -> Result<()> {
    if !insert.nose_radius.is_finite() || insert.nose_radius < 0.0 {
        return Err(invalid("nose radius must be finite and >= 0"));
    }
    if !insert.included_angle_deg.is_finite() {
        return Err(invalid("included angle is not finite"));
    }
    if insert.included_angle_deg < -1e-9 {
        return Err(invalid("included angle must be between 0 and 180 degrees"));
    }
    if !insert.orientation_deg.is_finite() {
        return Err(invalid("insert orientation is not finite"));
    }
    if !insert.edge_length.is_finite() || insert.edge_length <= 0.0 {
        return Err(invalid("edge length must be finite and positive"));
    }
    if let Some(width) = insert.width
        && !(width.is_finite() && width > 0.0)
    {
        return Err(invalid("insert width must be finite and positive"));
    }
    if insert.included_angle_deg.abs() <= 1e-9 && insert.width.is_none() {
        return Err(invalid("groove insert needs a width"));
    }
    if (1e-6..180.0).contains(&insert.included_angle_deg) {
        let _ = insert.orientation_ok_angle()?;
    }
    Ok(())
}

fn validate_holder(holder: &HolderGeom) -> Result<()> {
    if !holder.width.is_finite() || holder.width <= 0.0 {
        return Err(invalid("holder width must be finite and positive"));
    }
    if !holder.length.is_finite() || holder.length <= 0.0 {
        return Err(invalid("holder length must be finite and positive"));
    }
    Ok(())
}

struct Region {
    /// Profile polyline, `(Z, radius)`.
    section: Vec<(f64, f64)>,
    /// Closed material polygon.
    polygon: Vec<(f64, f64)>,
}

impl Region {
    fn new(section: &[(f64, f64)], side: Side) -> Result<Self> {
        if section.is_empty() {
            return Err(invalid("section is empty"));
        }
        let mut clean = Vec::with_capacity(section.len());
        for point in section {
            if !point.0.is_finite() || !point.1.is_finite() {
                return Err(invalid("section point is not finite"));
            }
            if clean.last().is_none_or(|prev| hypot2(*prev, *point) > 1e-12) {
                clean.push(*point);
            }
        }
        if clean.len() < 2 {
            return Err(invalid("section is empty"));
        }
        let mut r_max = 0.0_f64;
        for point in &clean {
            r_max = r_max.max(point.1.abs());
        }
        let outer = ID_OUTER_RADIUS.max(r_max + ID_OUTER_RADIUS);
        let (Some(&first), Some(&last)) = (clean.first(), clean.last()) else {
            return Err(invalid("section is empty"));
        };
        let mut polygon = clean.clone();
        match side {
            Side::Outside => {
                polygon.push((last.0, 0.0));
                polygon.push((first.0, 0.0));
            }
            Side::Inside => {
                polygon.push((last.0, outer));
                polygon.push((first.0, outer));
            }
        }
        Ok(Self { section: clean, polygon })
    }

    fn distance(&self, point: (f64, f64)) -> f64 {
        let mut best = f64::INFINITY;
        for pair in self.section.windows(2) {
            let (Some(start), Some(end)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            best = best.min(dist_seg(point, *start, *end));
        }
        best
    }

    /// Strict interior of the material polygon. The section boundary is outside.
    fn contains(&self, point: (f64, f64)) -> bool {
        let distance = self.distance(point);
        if !distance.is_finite() || distance <= BOUNDARY_BAND {
            return false;
        }
        point_in_polygon(point, &self.polygon)
    }

    /// Positive on the free side, negative in the solid.
    fn signed_distance(&self, point: (f64, f64)) -> f64 {
        let distance = self.distance(point);
        if self.contains(point) { -distance } else { distance }
    }
}

fn point_in_polygon(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut index = 0;
    while index < n {
        let next = (index + 1) % n;
        let (Some(a), Some(b)) = (polygon.get(index), polygon.get(next)) else {
            break;
        };
        let crosses = (a.1 > point.1) != (b.1 > point.1);
        if crosses {
            let span = b.1 - a.1;
            if span.abs() > 1e-15 {
                let t = (point.1 - a.1) / span;
                let z_at = a.0 + t * (b.0 - a.0);
                if z_at > point.0 {
                    inside = !inside;
                }
            }
        }
        index += 1;
    }
    inside
}

#[derive(Default)]
struct Worst {
    nose: Option<Hit>,
    edge: Option<Hit>,
    holder: Option<Hit>,
}

struct Hit {
    z: f64,
    x: f64,
    depth: f64,
}

/// Shared sample allowance. Nested edge and holder samples draw from the same counter.
struct Budget(Cell<usize>);

impl Budget {
    fn new(limit: usize) -> Self {
        Self(Cell::new(limit))
    }

    fn take(&self, count: usize) -> Result<()> {
        let left = self.0.get();
        if count > left {
            return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
        }
        self.0.set(left - count);
        Ok(())
    }
}

fn inspect(
    region: &Region,
    shape: &InsertShapeGeom,
    holder: Option<&HolderGeom>,
    sample_step: f64,
    tol: f64,
    budget: &Budget,
    worst: &mut Worst,
) -> Result<()> {
    let centre = nose_from_shape(shape);
    let signed = region.signed_distance(centre);
    if !signed.is_finite() {
        return Err(invalid("gouge distance is not finite"));
    }
    let nose_depth = shape.nose_radius - signed;
    consider(&mut worst.nose, centre, nose_depth, tol);

    for flank in &shape.flanks {
        sample_segment(flank.start, flank.end, sample_step, budget, &mut |point| record_solid(region, point, tol, &mut worst.edge))?;
    }
    if let Some(holder) = holder {
        sample_holder(shape, holder, sample_step, budget, &mut |point| record_solid(region, point, tol, &mut worst.holder))?;
    }
    Ok(())
}

fn nose_from_shape(shape: &InsertShapeGeom) -> (f64, f64) {
    // `back` is `centre + axis * distance`. Recovering the centre from a flank is unnecessary:
    // the arc is centred on the nose, and flanks start at `centre + radius * direction`.
    if let Some(flank) = shape.flanks.first()
        && shape.nose_radius > 0.0
        && shape.arc_span.abs() > 1e-9
        && shape.arc_span.abs() < TAU - 1e-6
    {
        let dir = (shape.arc_start.cos(), shape.arc_start.sin());
        return (flank.start.0 - shape.nose_radius * dir.0, flank.start.1 - shape.nose_radius * dir.1);
    }
    if shape.arc_span.abs() >= TAU - 1e-6 {
        return (shape.back.0 - shape.axis.0 * shape.nose_radius, shape.back.1 - shape.axis.1 * shape.nose_radius);
    }
    // Groove front is centred on the nose. The first flank runs across that front.
    if let Some(flank) = shape.flanks.first() {
        return ((flank.start.0 + flank.end.0) * 0.5, (flank.start.1 + flank.end.1) * 0.5);
    }
    shape.back
}

fn record_solid(region: &Region, point: (f64, f64), tol: f64, slot: &mut Option<Hit>) -> Result<()> {
    if !point.0.is_finite() || !point.1.is_finite() {
        return Err(invalid("gouge sample is not finite"));
    }
    if region.contains(point) {
        let depth = region.distance(point);
        if !depth.is_finite() {
            return Err(invalid("gouge distance is not finite"));
        }
        consider(slot, point, depth, tol);
    }
    Ok(())
}

fn consider(slot: &mut Option<Hit>, point: (f64, f64), depth: f64, tol: f64) {
    if !depth.is_finite() || depth <= tol {
        return;
    }
    match slot {
        Some(hit) if hit.depth >= depth => {}
        _ => *slot = Some(Hit { z: point.0, x: point.1, depth }),
    }
}

fn push_worst(gouges: &mut Vec<TurnGouge>, move_index: usize, worst: &Worst) {
    for (hit, what) in [(&worst.nose, GougeKind::Nose), (&worst.edge, GougeKind::Edge), (&worst.holder, GougeKind::Holder)] {
        if let Some(hit) = hit {
            gouges.push(TurnGouge { toolpath: 0, move_index, z: hit.z, x: hit.x, depth: hit.depth, what });
        }
    }
}

fn sample_holder(
    shape: &InsertShapeGeom,
    holder: &HolderGeom,
    sample_step: f64,
    budget: &Budget,
    mut visit: impl FnMut((f64, f64)) -> Result<()>,
) -> Result<()> {
    let half = holder.width * 0.5;
    let near_left = add(shape.back, scale(shape.perp, half));
    let near_right = add(shape.back, scale(shape.perp, -half));
    let far_left = add(near_left, scale(shape.axis, holder.length));
    let far_right = add(near_right, scale(shape.axis, holder.length));
    let corners = [near_left, near_right, far_right, far_left];
    for index in 0..4 {
        let Some(start) = corners.get(index).copied() else {
            continue;
        };
        let Some(end) = corners.get((index + 1) % 4).copied() else {
            continue;
        };
        sample_segment(start, end, sample_step, budget, &mut visit)?;
    }
    // Interior grid so a buried shank reports a depth past the rectangle's skin.
    let along = grid_count(holder.length, sample_step)?;
    let across = grid_count(holder.width, sample_step)?;
    let count = along.saturating_mul(across);
    budget.take(count)?;
    for i in 0..along {
        let u = if along == 1 { 0.5 } else { i as f64 / (along - 1) as f64 };
        for j in 0..across {
            let v = if across == 1 { 0.5 } else { j as f64 / (across - 1) as f64 };
            let point = add(add(shape.back, scale(shape.axis, holder.length * u)), scale(shape.perp, holder.width * (v - 0.5)));
            visit(point)?;
        }
    }
    Ok(())
}

fn grid_count(span: f64, step: f64) -> Result<usize> {
    if !(span.is_finite() && step.is_finite() && step > 0.0) {
        return Err(invalid("holder sample step is not usable"));
    }
    let n = (span / step).ceil();
    if !n.is_finite() || n < 1.0 {
        return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
    }
    let count = (n as usize).saturating_add(1).clamp(2, 64);
    Ok(count)
}

fn sample_motion(from: (f64, f64), motion: TurnMove, step: f64, budget: &Budget, visit: &mut impl FnMut((f64, f64)) -> Result<()>) -> Result<()> {
    match motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Thread { z, x, .. } => {
            finite_pair((z, x), "move endpoint")?;
            sample_segment(from, (z, x), step, budget, visit)
        }
        TurnMove::Arc { z, x, cz, cx, ccw } => {
            finite_pair((z, x), "arc endpoint")?;
            finite_pair((cz, cx), "arc centre")?;
            sample_arc(from, (z, x), (cz, cx), ccw, step, budget, visit)
        }
        TurnMove::Dwell { .. } => Ok(()),
    }
}

fn sample_segment(from: (f64, f64), to: (f64, f64), step: f64, budget: &Budget, visit: &mut impl FnMut((f64, f64)) -> Result<()>) -> Result<()> {
    let length = hypot2(from, to);
    let count = spaced_points(length, step)?;
    budget.take(count)?;
    if count <= 1 {
        return visit(from);
    }
    let last = (count - 1) as f64;
    for index in 0..count {
        let t = index as f64 / last;
        let point = (from.0 + (to.0 - from.0) * t, from.1 + (to.1 - from.1) * t);
        if !point.0.is_finite() || !point.1.is_finite() {
            return Err(invalid("gouge sample is not finite"));
        }
        visit(point)?;
    }
    Ok(())
}

fn sample_arc(
    from: (f64, f64),
    to: (f64, f64),
    centre: (f64, f64),
    ccw: bool,
    step: f64,
    budget: &Budget,
    visit: &mut impl FnMut((f64, f64)) -> Result<()>,
) -> Result<()> {
    let radius = hypot2(from, centre);
    let end_radius = hypot2(to, centre);
    if !radius.is_finite() || !end_radius.is_finite() || radius <= 1e-12 {
        return sample_segment(from, to, step, budget, visit);
    }
    if (radius - end_radius).abs() > 1.0e-3_f64.max(radius * 1.0e-4) {
        return Err(invalid("arc endpoints are not on one circle"));
    }
    let a0 = (from.1 - centre.1).atan2(from.0 - centre.0);
    let a1 = (to.1 - centre.1).atan2(to.0 - centre.0);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(invalid("arc angle is not finite"));
    }
    let chord = hypot2(from, to);
    let sweep = if ccw { ccw_sweep(a0, a1, chord)? } else { -ccw_sweep(a1, a0, chord)? };
    let length = (radius * sweep.abs()).abs();
    let count = spaced_points(length, step)?.max(2);
    budget.take(count)?;
    let last = (count - 1) as f64;
    for index in 0..count {
        let point = if index == 0 {
            from
        } else if index + 1 == count {
            to
        } else {
            let t = index as f64 / last;
            let angle = a0 + sweep * t;
            (centre.0 + radius * angle.cos(), centre.1 + radius * angle.sin())
        };
        if !point.0.is_finite() || !point.1.is_finite() {
            return Err(invalid("gouge sample is not finite"));
        }
        visit(point)?;
    }
    Ok(())
}

fn ccw_sweep(a0: f64, a1: f64, chord: f64) -> Result<f64> {
    let delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= 1.0e-12 || delta <= 1e-15 { TAU } else { delta };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(invalid("arc sweep is not usable")) }
}

fn spaced_points(length: f64, step: f64) -> Result<usize> {
    if !length.is_finite() || length < 0.0 {
        return Err(invalid("gouge sample length is not finite"));
    }
    if length == 0.0 {
        return Ok(1);
    }
    if !step.is_finite() || step <= 0.0 {
        return Err(invalid("sample step must be finite and positive"));
    }
    let mut intervals = (length / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
    }
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(invalid("gouge sample spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_GOUGE_SAMPLES as f64 {
        return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
    }
    Ok(count as usize)
}

fn motion_end(from: (f64, f64), motion: TurnMove) -> Result<(f64, f64)> {
    let end = match motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } | TurnMove::Thread { z, x, .. } => (z, x),
        TurnMove::Dwell { .. } => from,
    };
    finite_pair(end, "move endpoint")?;
    if let TurnMove::Thread { lead, .. } = motion
        && !(lead.is_finite() && lead > 0.0)
    {
        return Err(invalid("thread lead must be finite and positive"));
    }
    Ok(end)
}

fn tip_shift(tip: TipPoint) -> Result<(f64, f64)> {
    let shift = match tip {
        TipPoint::Centre => (0.0, 0.0),
        TipPoint::ImaginaryTip { dz, dx } => (dz, dx),
    };
    finite_pair(shift, "tip offset")?;
    Ok(shift)
}

fn finite_pair(point: (f64, f64), what: &str) -> Result<()> {
    if point.0.is_finite() && point.1.is_finite() { Ok(()) } else { Err(invalid(format!("{what} is not finite"))) }
}

fn arc_steps(radius: f64, sweep: f64, tol: f64) -> usize {
    let sweep = sweep.abs();
    if !(sweep > 1e-12) || radius <= 0.0 {
        return 1;
    }
    let sagitta = radius * (1.0 - (sweep * 0.5).cos());
    if !sagitta.is_finite() || sagitta <= tol {
        return 1;
    }
    let cos_half = (1.0 - (tol / radius)).clamp(-1.0, 1.0);
    let delta = (2.0 * cos_half.acos()).max(1e-6);
    let n = (sweep / delta).ceil();
    if !n.is_finite() || n < 1.0 {
        return 1;
    }
    if n > 10_000.0 { 10_000 } else { n as usize }
}

fn push_apart(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
    if points.last().is_none_or(|prev| hypot2(*prev, point) > 1e-12) {
        points.push(point);
    }
}

fn add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn scale(v: (f64, f64), k: f64) -> (f64, f64) {
    (v.0 * k, v.1 * k)
}

fn hypot2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn dist_seg(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> f64 {
    let dz = end.0 - start.0;
    let dx = end.1 - start.1;
    let len2 = dz * dz + dx * dx;
    if len2 <= 1e-24 {
        return hypot2(point, start);
    }
    let t = ((point.0 - start.0) * dz + (point.1 - start.1) * dx) / len2;
    let t = t.clamp(0.0, 1.0);
    hypot2(point, (start.0 + t * dz, start.1 + t * dx))
}

fn invalid(message: impl Into<String>) -> TurnError {
    ops::invalid(message)
}

#[cfg(test)]
#[path = "gouge_tests.rs"]
mod tests;
