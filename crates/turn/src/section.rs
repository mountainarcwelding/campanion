//! Half-profile from a full planar section through the turning axis.

use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use campanion_geom::{Arc, Line, Point, Segment};
use campanion_import::{ImportOptions, import_dxf};

use crate::profile::{import_tolerance, mirror_segment};
use crate::{Result, TurnError, TurnProfile};

/// Endpoints closer than this still join when the caller tolerance is tighter.
const MIN_JOIN_MM: f64 = 1e-4;
/// Samples used to compare the two halves.
const HALF_SAMPLES: usize = 65;
/// Parameter margin so a crossing on an endpoint is not treated as interior.
const SPLIT_MARGIN: f64 = 1e-8;

impl TurnProfile {
    /// Half-profile from a full planar section through the turning axis.
    ///
    /// `text` is a DXF section in the XZ plane: DXF X is a signed radius and DXF Y is Z.
    /// `tol` is the linear tolerance in millimetres, the same one [`TurnProfile::from_dxf`] uses.
    ///
    /// 1. Import with [`campanion_import::import_dxf`] at that tolerance and keep every segment
    ///    of every contour, on every layer.
    /// 2. Swap each segment into profile coordinates `(Z, radius) = (y, x)`. A swap is a
    ///    reflection, so an arc's direction flips. An arc whose sweep is longer than half a turn
    ///    is replaced first by the minor arc between the same endpoints. The section records the
    ///    edge's start and end angles in traversal order, and a counter-clockwise reading of a
    ///    clockwise fillet is the long complement through the part.
    /// 3. Split every line or arc that crosses radius 0, so each piece lies on one side.
    /// 4. `JOIN` is `max(tol, 1e-4)` mm. The upper half is every piece whose points all have
    ///    radius `>= -JOIN`. The lower half is every piece whose points all have radius
    ///    `<= JOIN`, mirrored (`radius -> -radius`, which flips arcs again). A piece that lies
    ///    on the axis along its whole length (`|radius| <= JOIN`) goes in neither.
    /// 5. A half is usable when those pieces form exactly one open chain: endpoints match within
    ///    `JOIN`, joints snap to the shared point, pieces reverse as needed, every piece is used
    ///    once, and nothing branches. Both ends must have `|radius| <= JOIN`. The chain runs
    ///    front (larger Z) first. The end radii are set to 0 and the chain is built with
    ///    [`TurnProfile::try_new`] at `JOIN.max(tol)`. A flat back face (a line at the
    ///    smallest Z down to the axis) is dropped first, so the profile stops at the chuck-end
    ///    diameter like a drawn profile. A line whose Z changes by at most `JOIN` and whose
    ///    radius changes by more than `JOIN` is a radial face: both ends take the Z of the end
    ///    nearer the axis. A seam that pulls the outer end of that face off the plane stays one
    ///    face, so the two halves are not reported as different radii there.
    /// 6. When both halves are usable, compare [`TurnProfile::radius_at`] at 65 evenly spaced Z
    ///    values on the overlapping Z range, skipping a Z where either returns `None`. If the
    ///    largest difference is at most `max(10 * tol, 1e-3)`, return the upper half. A larger
    ///    difference is rejected: the two halves differ by that much at that Z, so the part is
    ///    not turned about Z. One usable half is returned as it is. If neither half is usable,
    ///    the section does not give one half-profile from the axis back to the axis. No segments
    ///    at all means the section is empty.
    pub fn from_section_dxf(text: &str, tol: f64) -> Result<Self> {
        let tolerance = import_tolerance(tol)?;
        let drawing = import_dxf(text, &ImportOptions::new(tolerance))?;
        let mut pieces = Vec::new();
        for imported in drawing.contours() {
            for segment in imported.contour().segments() {
                let swapped = swap_segment(*segment)?;
                pieces.extend(split_at_axis(swapped)?);
            }
        }
        if pieces.is_empty() {
            return Err(TurnError::InvalidInput("the section is empty".into()));
        }
        let join = tol.max(MIN_JOIN_MM);
        let (upper_pieces, lower_pieces) = classify_halves(&pieces, join)?;
        let upper = chain_half(&upper_pieces, join, tol);
        let lower = chain_half(&lower_pieces, join, tol);
        match (upper, lower) {
            (Some(upper), Some(lower)) => {
                if let Some((diff, z)) = largest_radius_gap(&upper, &lower) {
                    let limit = (10.0 * tol).max(1e-3);
                    if diff > limit {
                        return Err(TurnError::InvalidInput(format!(
                            "the two halves of the section differ by {diff} mm at Z {z}; the part is not turned about Z"
                        )));
                    }
                }
                Ok(upper)
            }
            (Some(upper), None) => Ok(upper),
            (None, Some(lower)) => Ok(lower),
            (None, None) => Err(TurnError::InvalidInput(
                "the section does not give one half-profile from the axis back to the axis (a through-bore or a part that is not on the Z axis is not supported)".into(),
            )),
        }
    }
}

/// `(Z, radius) = (y, x)`. An arc longer than half a turn becomes the minor arc first.
fn swap_segment(segment: Segment) -> Result<Segment> {
    match segment {
        Segment::Line(line) => {
            let line = Line::new(swap_point(line.start())?, swap_point(line.end())?)?;
            Ok(Segment::Line(line))
        }
        Segment::Arc(arc) => {
            let arc = minor_arc(arc)?;
            let center = swap_point(arc.center())?;
            let swapped = Arc::new(center, arc.radius(), FRAC_PI_2 - arc.start_angle(), -arc.sweep())?;
            Ok(Segment::Arc(swapped))
        }
    }
}

fn swap_point(point: Point) -> Result<Point> {
    Ok(Point::new(point.y(), point.x())?)
}

/// Minor arc between the same endpoints. A full turn is left alone.
fn minor_arc(arc: Arc) -> Result<Arc> {
    let sweep = arc.sweep();
    if sweep.abs() <= PI || sweep.abs() >= TAU - 1e-6 {
        return Ok(arc);
    }
    let complement = sweep - sweep.signum() * TAU;
    Ok(Arc::new(arc.center(), arc.radius(), arc.start_angle(), complement)?)
}

fn split_at_axis(segment: Segment) -> Result<Vec<Segment>> {
    match segment {
        Segment::Line(line) => split_line(line),
        Segment::Arc(arc) => split_arc(arc),
    }
}

fn split_line(line: Line) -> Result<Vec<Segment>> {
    let y0 = line.start().y();
    let y1 = line.end().y();
    if y0 * y1 >= 0.0 {
        return Ok(vec![Segment::Line(line)]);
    }
    let denom = y0 - y1;
    let t = y0 / denom;
    if !t.is_finite() || t <= 0.0 || t >= 1.0 {
        return Ok(vec![Segment::Line(line)]);
    }
    let x = line.start().x() + t * (line.end().x() - line.start().x());
    let mid = Point::new(x, 0.0)?;
    let head = Line::new(line.start(), mid)?;
    let tail = Line::new(mid, line.end())?;
    Ok(vec![Segment::Line(head), Segment::Line(tail)])
}

fn split_arc(arc: Arc) -> Result<Vec<Segment>> {
    let center_y = arc.center().y();
    let radius = arc.radius();
    let disc = radius * radius - center_y * center_y;
    let flat = 1e-12 * radius.max(1.0);
    if disc <= flat {
        return Ok(vec![Segment::Arc(arc)]);
    }
    let sine = (-center_y / radius).clamp(-1.0, 1.0);
    let principal = sine.asin();
    let mut cuts = Vec::new();
    for theta in [principal, PI - principal] {
        let Some(t) = param_on_sweep(arc.start_angle(), arc.sweep(), theta) else {
            continue;
        };
        if t > SPLIT_MARGIN && t < 1.0 - SPLIT_MARGIN {
            cuts.push(t);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
    if cuts.is_empty() {
        return Ok(vec![Segment::Arc(arc)]);
    }
    let mut pieces = Vec::new();
    let mut from = 0.0;
    for t in cuts {
        pieces.push(sub_arc(arc, from, t)?);
        from = t;
    }
    pieces.push(sub_arc(arc, from, 1.0)?);
    Ok(pieces)
}

fn sub_arc(arc: Arc, t0: f64, t1: f64) -> Result<Segment> {
    let start = arc.start_angle() + arc.sweep() * t0;
    let sweep = arc.sweep() * (t1 - t0);
    Ok(Segment::Arc(Arc::new(arc.center(), arc.radius(), start, sweep)?))
}

/// `t` in `[0, 1]` when `theta` lies on the signed sweep, otherwise `None`.
fn param_on_sweep(start: f64, sweep: f64, theta: f64) -> Option<f64> {
    if !theta.is_finite() || !start.is_finite() || !sweep.is_finite() {
        return None;
    }
    let abs_sweep = sweep.abs();
    if abs_sweep < 1e-15 {
        return None;
    }
    if abs_sweep >= TAU - 1e-8 {
        let ccw = (theta - start).rem_euclid(TAU);
        let t = if sweep >= 0.0 { ccw / TAU } else { (start - theta).rem_euclid(TAU) / TAU };
        return Some(t.clamp(0.0, 1.0));
    }
    let along = if sweep > 0.0 { (theta - start).rem_euclid(TAU) } else { (start - theta).rem_euclid(TAU) };
    let adjusted = if TAU - along <= 1e-8 { 0.0 } else { along };
    if adjusted <= abs_sweep + 1e-8 { Some((adjusted / abs_sweep).clamp(0.0, 1.0)) } else { None }
}

fn classify_halves(pieces: &[Segment], join: f64) -> Result<(Vec<Segment>, Vec<Segment>)> {
    let mut upper = Vec::new();
    let mut lower = Vec::new();
    for piece in pieces {
        let bounds = piece.bounding_box();
        let min_r = bounds.min().y();
        let max_r = bounds.max().y();
        if min_r >= -join && max_r <= join {
            continue;
        }
        if min_r >= -join {
            upper.push(*piece);
        } else if max_r <= join {
            lower.push(mirror_segment(*piece)?);
        }
    }
    Ok((upper, lower))
}

fn chain_half(pieces: &[Segment], join: f64, tol: f64) -> Option<TurnProfile> {
    let ordered = order_chain(pieces, join)?;
    let mut snapped = snap_chain(&ordered, join)?;
    drop_back_face(&mut snapped, join);
    straighten_radial_faces(&mut snapped, join)?;
    TurnProfile::try_new(snapped, join.max(tol)).ok()
}

/// Put a radial face back on one Z.
///
/// Joining a seam can drag the outer end of a front or shoulder face off its plane by less than
/// `join`. [`TurnProfile::radius_at`] then samples the axis end of that sloped line and reports
/// radius 0, while the other half's square face reports the outside radius. Both ends take the Z
/// of the end nearer the axis. A line that is already vertical, or that does not change radius by
/// more than `join`, is left alone.
fn straighten_radial_faces(segments: &mut [Segment], join: f64) -> Option<()> {
    if join <= 0.0 {
        return Some(());
    }
    for segment in segments.iter_mut() {
        let Segment::Line(line) = *segment else {
            continue;
        };
        let z0 = line.start().x();
        let z1 = line.end().x();
        let dz = (z0 - z1).abs();
        if dz <= 1e-15 || dz > join {
            continue;
        }
        let r0 = line.start().y();
        let r1 = line.end().y();
        if (r0 - r1).abs() <= join {
            continue;
        }
        let face_z = if r0.abs() <= r1.abs() { z0 } else { z1 };
        let start = Point::new(face_z, r0).ok()?;
        let end = Point::new(face_z, r1).ok()?;
        *segment = Segment::Line(Line::new(start, end).ok()?);
    }
    Some(())
}

/// Remove the flat back face at the chuck end (a vertical line down to the axis).
///
/// A drawn lathe profile stops at the outside diameter by the chuck; the turning operations
/// never cut that face. The section closes it, so it is dropped here. A chain that is only
/// that face is left alone and fails later.
fn drop_back_face(segments: &mut Vec<Segment>, join: f64) {
    if segments.len() < 2 {
        return;
    }
    let (Some(first), Some(last)) = (segments.first().copied(), segments.last().copied()) else {
        return;
    };
    // Chains run front first after `order_chain`, but check both ends so either order works.
    let flat_to_axis = |segment: Segment, axis_end: Point| {
        matches!(segment, Segment::Line(_)) && (segment.start().x() - segment.end().x()).abs() <= join && axis_end.y().abs() <= join
    };
    if last.end().x() < first.start().x() {
        if flat_to_axis(last, last.end()) {
            segments.pop();
        }
    } else if flat_to_axis(first, first.start()) {
        segments.remove(0);
    }
}

struct EndPoint {
    seg: usize,
    is_start: bool,
    point: Point,
}

struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(count: usize) -> Self {
        Self { parent: (0..count).collect() }
    }

    fn find(&self, mut index: usize) -> usize {
        while let Some(&parent) = self.parent.get(index) {
            if parent == index {
                return index;
            }
            index = parent;
        }
        index
    }

    fn union(&mut self, a: usize, b: usize) {
        let root_a = self.find(a);
        let root_b = self.find(b);
        if root_a != root_b
            && let Some(slot) = self.parent.get_mut(root_a)
        {
            *slot = root_b;
        }
    }
}

fn cell_of(value: f64, join: f64) -> i64 {
    let scaled = (value / join).floor();
    if !scaled.is_finite() {
        return 0;
    }
    if scaled < i64::MIN as f64 {
        i64::MIN
    } else if scaled > i64::MAX as f64 {
        i64::MAX
    } else {
        scaled as i64
    }
}

fn order_chain(pieces: &[Segment], join: f64) -> Option<Vec<Segment>> {
    if pieces.is_empty() || join <= 0.0 {
        return None;
    }
    let mut ends = Vec::with_capacity(pieces.len() * 2);
    for (seg, piece) in pieces.iter().copied().enumerate() {
        ends.push(EndPoint { seg, is_start: true, point: piece.start() });
        ends.push(EndPoint { seg, is_start: false, point: piece.end() });
    }
    let mut buckets: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (index, end) in ends.iter().enumerate() {
        buckets.entry((cell_of(end.point.x(), join), cell_of(end.point.y(), join))).or_default().push(index);
    }
    let mut uf = UnionFind::new(ends.len());
    for (index, end) in ends.iter().enumerate() {
        let cx = cell_of(end.point.x(), join);
        let cy = cell_of(end.point.y(), join);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(bucket) = buckets.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &other in bucket {
                    if other <= index {
                        continue;
                    }
                    let Some(partner) = ends.get(other) else {
                        continue;
                    };
                    if partner.seg == end.seg {
                        continue;
                    }
                    if end.point.distance(partner.point) <= join {
                        uf.union(index, other);
                    }
                }
            }
        }
    }
    let mut groups: HashMap<usize, Vec<usize>> = HashMap::new();
    for index in 0..ends.len() {
        groups.entry(uf.find(index)).or_default().push(index);
    }
    let mut tails = Vec::new();
    for members in groups.values() {
        match members.len() {
            1 => {
                let &index = members.first()?;
                tails.push(index);
            }
            2 => {}
            _ => return None,
        }
    }
    let (Some(&first), Some(&second)) = (tails.first(), tails.get(1)) else {
        return None;
    };
    if tails.len() != 2 {
        return None;
    }
    let start_tail = match (ends.get(first), ends.get(second)) {
        (Some(a), Some(b)) if b.point.x() > a.point.x() => second,
        _ => first,
    };
    let start_point = ends.get(start_tail)?.point;
    if start_point.y().abs() > join {
        return None;
    }
    let mut ordered = Vec::with_capacity(pieces.len());
    let mut seen = vec![false; pieces.len()];
    let mut current = start_tail;
    for _ in 0..pieces.len() {
        let end = ends.get(current)?;
        if *seen.get(end.seg)? {
            return None;
        }
        if let Some(flag) = seen.get_mut(end.seg) {
            *flag = true;
        }
        let piece = *pieces.get(end.seg)?;
        ordered.push(if end.is_start { piece } else { piece.reversed() });
        let exit_index = if end.is_start { end.seg * 2 + 1 } else { end.seg * 2 };
        let members = groups.get(&uf.find(exit_index))?;
        if members.len() == 1 {
            let exit = ends.get(exit_index)?;
            if exit.point.y().abs() > join || seen.iter().any(|flag| !*flag) {
                return None;
            }
            return Some(ordered);
        }
        current = *members.iter().find(|index| **index != exit_index)?;
    }
    None
}

fn snap_chain(segments: &[Segment], join: f64) -> Option<Vec<Segment>> {
    if segments.is_empty() {
        return None;
    }
    let mut built = Vec::with_capacity(segments.len());
    for (index, segment) in segments.iter().copied().enumerate() {
        if index == 0 && segment.start().y().abs() > join {
            return None;
        }
        if index + 1 == segments.len() && segment.end().y().abs() > join {
            return None;
        }
        let start = if index == 0 { on_axis(segment.start())? } else { joint_point(segments, index)? };
        let end = if index + 1 == segments.len() { on_axis(segment.end())? } else { joint_point_after(segments, index)? };
        built.push(retarget(segment, start, end, join)?);
    }
    tighten(&mut built, join)?;
    pin_arc_ends(&mut built, join)?;
    Some(built)
}

fn joint_point(segments: &[Segment], index: usize) -> Option<Point> {
    let prev = segments.get(index - 1)?.end();
    let current = segments.get(index)?.start();
    Some(prev.midpoint(current))
}

fn joint_point_after(segments: &[Segment], index: usize) -> Option<Point> {
    let current = segments.get(index)?.end();
    let next = segments.get(index + 1)?.start();
    Some(current.midpoint(next))
}

fn on_axis(point: Point) -> Option<Point> {
    Point::new(point.x(), 0.0).ok()
}

fn retarget(segment: Segment, start: Point, end: Point, join: f64) -> Option<Segment> {
    match segment {
        Segment::Line(_) => Some(Segment::Line(Line::new(start, end).ok()?)),
        Segment::Arc(arc) => retarget_arc(arc, start, end, join),
    }
}

fn retarget_arc(arc: Arc, start: Point, end: Point, join: f64) -> Option<Segment> {
    let center = arc.center();
    let to_start = start - center;
    let to_end = end - center;
    if to_start.length() <= 1e-15 || to_end.length() <= 1e-15 {
        return None;
    }
    let start_angle = to_start.angle();
    let end_angle = to_end.angle();
    let ccw = (end_angle - start_angle).rem_euclid(TAU);
    let sweep = if ccw <= 1e-12 {
        if arc.sweep().abs() > PI { arc.sweep().signum() * TAU } else { return None }
    } else {
        let cw = ccw - TAU;
        if (ccw - arc.sweep()).abs() <= (cw - arc.sweep()).abs() { ccw } else { cw }
    };
    let built = Arc::new(center, arc.radius(), start_angle, sweep).ok()?;
    if built.start_point().distance(start) > join || built.end_point().distance(end) > join {
        return None;
    }
    Some(Segment::Arc(built))
}

fn tighten(segments: &mut [Segment], join: f64) -> Option<()> {
    let last = segments.len().saturating_sub(1);
    for index in 0..last {
        let left = *segments.get(index)?;
        let right = *segments.get(index + 1)?;
        let gap = left.end().distance(right.start());
        if gap <= 1e-12 {
            continue;
        }
        match (left, right) {
            (Segment::Arc(_), Segment::Line(_)) => set_line_start(segments.get_mut(index + 1)?, left.end())?,
            (Segment::Line(_), Segment::Arc(_)) => set_line_end(segments.get_mut(index)?, right.start())?,
            (Segment::Line(_), Segment::Line(_)) => {
                let mid = left.end().midpoint(right.start());
                set_line_end(segments.get_mut(index)?, mid)?;
                set_line_start(segments.get_mut(index + 1)?, mid)?;
            }
            (Segment::Arc(_), Segment::Arc(_)) => {
                if gap > join {
                    return None;
                }
            }
        }
    }
    Some(())
}

fn set_line_start(segment: &mut Segment, start: Point) -> Option<()> {
    let Segment::Line(line) = segment else {
        return None;
    };
    *segment = Segment::Line(Line::new(start, line.end()).ok()?);
    Some(())
}

fn set_line_end(segment: &mut Segment, end: Point) -> Option<()> {
    let Segment::Line(line) = segment else {
        return None;
    };
    *segment = Segment::Line(Line::new(line.start(), end).ok()?);
    Some(())
}

fn pin_arc_ends(segments: &mut Vec<Segment>, join: f64) -> Option<()> {
    pin_arc_start(segments, join)?;
    pin_arc_end(segments, join)
}

fn pin_arc_start(segments: &mut Vec<Segment>, join: f64) -> Option<()> {
    let first = *segments.first()?;
    let Segment::Arc(_) = first else {
        return Some(());
    };
    let start = first.start();
    if start.y().abs() <= 1e-15 {
        return Some(());
    }
    if start.y().abs() > join {
        return None;
    }
    let pinned = on_axis(start)?;
    segments.insert(0, Segment::Line(Line::new(pinned, start).ok()?));
    Some(())
}

fn pin_arc_end(segments: &mut Vec<Segment>, join: f64) -> Option<()> {
    let last = *segments.last()?;
    let Segment::Arc(_) = last else {
        return Some(());
    };
    let end = last.end();
    if end.y().abs() <= 1e-15 {
        return Some(());
    }
    if end.y().abs() > join {
        return None;
    }
    let pinned = on_axis(end)?;
    segments.push(Segment::Line(Line::new(end, pinned).ok()?));
    Some(())
}

fn largest_radius_gap(upper: &TurnProfile, lower: &TurnProfile) -> Option<(f64, f64)> {
    let (upper_min, upper_max) = upper.z_range();
    let (lower_min, lower_max) = lower.z_range();
    let z0 = upper_min.max(lower_min);
    let z1 = upper_max.min(lower_max);
    if !z0.is_finite() || !z1.is_finite() || z1 < z0 {
        return None;
    }
    let mut worst: Option<(f64, f64)> = None;
    let steps = HALF_SAMPLES.saturating_sub(1).max(1);
    for index in 0..HALF_SAMPLES {
        let z = if index + 1 == HALF_SAMPLES { z1 } else { z0 + (z1 - z0) * (index as f64) / (steps as f64) };
        let (Some(upper_r), Some(lower_r)) = (upper.radius_at(z), lower.radius_at(z)) else {
            continue;
        };
        let diff = (upper_r - lower_r).abs();
        if worst.is_none_or(|(best, _)| diff > best) {
            worst = Some((diff, z));
        }
    }
    worst
}
