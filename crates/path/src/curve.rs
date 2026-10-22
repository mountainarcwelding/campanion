//! Contour queries the geometry crate does not expose: closest point, tangents, splits.
//!
//! Arc parameter math follows the same sweep rules as `campanion-geom` (signed sweep, positive
//! counter-clockwise, angles from +X).

use campanion_geom::{Arc, Contour, Line, Point, Segment, Tolerance, Vec2};

use crate::{PathError, Result};

const TAU: f64 = std::f64::consts::TAU;

/// A point on a contour.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Location {
    pub index: usize,
    /// Parameter along that segment, in `[0, 1]`.
    pub t: f64,
    pub point: Point,
}

pub(crate) fn closest_location(contour: &Contour, point: Point, tolerance: &Tolerance) -> Result<Location> {
    let mut best: Option<(f64, Location)> = None;
    for (index, seg) in contour.segments().iter().copied().enumerate() {
        let (t, at) = closest_on_segment(seg, point)?;
        let (t, at) = snap_ends(seg, t, at, tolerance.linear());
        let distance = at.distance(point);
        let replace = match best {
            None => true,
            Some((so_far, _)) => distance < so_far,
        };
        if replace {
            best = Some((distance, Location { index, t, point: at }));
        }
    }
    best.map(|(_, loc)| loc).ok_or_else(|| PathError::InvalidInput("contour is empty".into()))
}

/// Midpoint of the longest line. `None` when every edge is an arc or shorter than `min_length`.
pub(crate) fn longest_line_midpoint(contour: &Contour, min_length: f64) -> Result<Option<Location>> {
    let mut best_len = min_length;
    let mut best: Option<Location> = None;
    for (index, seg) in contour.segments().iter().copied().enumerate() {
        let Segment::Line(line) = seg else {
            continue;
        };
        let len = line.length();
        if !len.is_finite() {
            return Err(PathError::InvalidInput("segment length is not finite".into()));
        }
        if len > best_len {
            best_len = len;
            best = Some(Location { index, t: 0.5, point: line.start().midpoint(line.end()) });
        }
    }
    Ok(best)
}

/// Vertex with the largest absolute turn. Ties keep the earliest vertex.
pub(crate) fn sharpest_corner(contour: &Contour) -> Result<Option<Location>> {
    let segs = contour.segments();
    let n = segs.len();
    if n == 0 {
        return Ok(None);
    }
    let mut best_turn = -1.0;
    let mut best: Option<Location> = None;
    for index in 0..n {
        if !contour.is_closed() && index == 0 {
            continue;
        }
        let prev_index = if index == 0 { n - 1 } else { index - 1 };
        let (Some(prev), Some(here)) = (segs.get(prev_index).copied(), segs.get(index).copied()) else {
            return Err(PathError::InvalidInput("segment is missing".into()));
        };
        let incoming = end_tangent_of(prev)?;
        let outgoing = start_tangent_of(here)?;
        let turn = incoming.cross(outgoing).atan2(incoming.dot(outgoing)).abs();
        if turn > best_turn {
            best_turn = turn;
            best = Some(Location { index, t: 0.0, point: here.start() });
        }
    }
    if best.is_none()
        && let Some(first) = segs.first()
    {
        return Ok(Some(Location { index: 0, t: 0.0, point: first.start() }));
    }
    Ok(best)
}

pub(crate) fn rotate_closed(contour: &Contour, loc: &Location, tolerance: &Tolerance) -> Result<Contour> {
    if !contour.is_closed() {
        return Err(PathError::InvalidInput("cannot rotate an open contour".into()));
    }
    let segs = contour.segments();
    let n = segs.len();
    if n == 0 {
        return Err(PathError::InvalidInput("contour is empty".into()));
    }
    let Some(seg) = segs.get(loc.index).copied() else {
        return Err(PathError::InvalidInput("start segment is missing".into()));
    };
    let tol = tolerance.linear().max(1e-9);
    if loc.t <= 1e-9 || seg.start().almost_eq(loc.point, tol) {
        return rotate_index(contour, loc.index, tolerance);
    }
    if loc.t >= 1.0 - 1e-9 || seg.end().almost_eq(loc.point, tol) {
        return rotate_index(contour, (loc.index + 1) % n, tolerance);
    }
    let (left, right) = split_segment(seg, loc.t)?;
    let mut out = Vec::with_capacity(n + 1);
    out.push(right);
    for index in (loc.index + 1)..n {
        let Some(segment) = segs.get(index) else {
            return Err(PathError::InvalidInput("segment is missing".into()));
        };
        out.push(*segment);
    }
    for index in 0..loc.index {
        let Some(segment) = segs.get(index) else {
            return Err(PathError::InvalidInput("segment is missing".into()));
        };
        out.push(*segment);
    }
    out.push(left);
    Contour::try_new(out, true, tolerance).map_err(PathError::from)
}

pub(crate) fn point_at_length(contour: &Contour, s: f64) -> Result<Point> {
    let total = finite_length(contour)?;
    if !s.is_finite() {
        return Err(PathError::InvalidInput("arc length must be finite".into()));
    }
    if s < -1e-9 || s > total + 1e-6 {
        return Err(PathError::InvalidInput("arc length is outside the contour".into()));
    }
    if s <= 1e-12 {
        return first_point(contour);
    }
    if (total - s).abs() <= 1e-9 || s >= total {
        return last_point(contour);
    }
    let mut acc = 0.0;
    for seg in contour.segments() {
        let len = seg.length();
        if !len.is_finite() {
            return Err(PathError::InvalidInput("segment length is not finite".into()));
        }
        if len <= 1e-15 {
            acc += len;
            continue;
        }
        if acc + len >= s {
            let t = ((s - acc) / len).clamp(0.0, 1.0);
            return Ok(point_on_segment(*seg, t));
        }
        acc += len;
    }
    last_point(contour)
}

pub(crate) fn tangent_at_length(contour: &Contour, s: f64) -> Result<Vec2> {
    let total = finite_length(contour)?;
    if !s.is_finite() {
        return Err(PathError::InvalidInput("arc length must be finite".into()));
    }
    if s < -1e-9 || s > total + 1e-6 {
        return Err(PathError::InvalidInput("arc length is outside the contour".into()));
    }
    if contour.is_closed() && (s <= 1e-9 || (total - s).abs() <= 1e-8) {
        return start_tangent(contour);
    }
    if !contour.is_closed() && (total - s).abs() <= 1e-8 {
        return end_tangent(contour);
    }
    let segs = contour.segments();
    let mut acc = 0.0;
    for (index, seg) in segs.iter().copied().enumerate() {
        let len = seg.length();
        if !len.is_finite() {
            return Err(PathError::InvalidInput("segment length is not finite".into()));
        }
        if len <= 1e-15 {
            acc += len;
            continue;
        }
        let last = index + 1 == segs.len();
        if acc + len + 1e-12 >= s || last {
            let t = ((s - acc) / len).clamp(0.0, 1.0);
            if t >= 1.0 - 1e-9 {
                if let Some(next) = segs.get(index + 1).copied() {
                    return start_tangent_of(next);
                }
                if contour.is_closed() {
                    return start_tangent(contour);
                }
                return end_tangent_of(seg);
            }
            return tangent_on(seg, t);
        }
        acc += len;
    }
    start_tangent(contour)
}

pub(crate) fn subdivide(contour: &Contour, s0: f64, s1: f64) -> Result<Vec<Segment>> {
    if !s0.is_finite() || !s1.is_finite() || s1 < s0 - 1e-9 {
        return Err(PathError::InvalidInput("bad span".into()));
    }
    let mut out = Vec::new();
    let mut acc = 0.0;
    for seg in contour.segments().iter().copied() {
        let len = seg.length();
        if !len.is_finite() {
            return Err(PathError::InvalidInput("segment length is not finite".into()));
        }
        let seg_s0 = acc;
        acc += len;
        if len <= 1e-15 || acc < s0 - 1e-12 {
            continue;
        }
        if seg_s0 > s1 + 1e-12 {
            break;
        }
        let t0 = ((s0 - seg_s0) / len).clamp(0.0, 1.0);
        let t1 = ((s1 - seg_s0) / len).clamp(0.0, 1.0);
        if (t1 - t0) * len < 1e-12 {
            continue;
        }
        out.push(trim_segment(seg, t0, t1)?);
    }
    Ok(out)
}

pub(crate) fn start_tangent(contour: &Contour) -> Result<Vec2> {
    let Some(seg) = contour.segments().first().copied() else {
        return Err(PathError::InvalidInput("contour is empty".into()));
    };
    start_tangent_of(seg)
}

pub(crate) fn end_tangent(contour: &Contour) -> Result<Vec2> {
    let Some(seg) = contour.segments().last().copied() else {
        return Err(PathError::InvalidInput("contour is empty".into()));
    };
    end_tangent_of(seg)
}

fn first_point(contour: &Contour) -> Result<Point> {
    contour.segments().first().map(|seg| seg.start()).ok_or_else(|| PathError::InvalidInput("contour is empty".into()))
}

fn last_point(contour: &Contour) -> Result<Point> {
    contour.segments().last().map(|seg| seg.end()).ok_or_else(|| PathError::InvalidInput("contour is empty".into()))
}

fn finite_length(contour: &Contour) -> Result<f64> {
    let length = contour.length()?;
    if length.is_finite() { Ok(length) } else { Err(PathError::InvalidInput("contour length is not finite".into())) }
}

fn rotate_index(contour: &Contour, start: usize, tolerance: &Tolerance) -> Result<Contour> {
    let segs = contour.segments();
    let n = segs.len();
    if n == 0 || start >= n {
        return Err(PathError::InvalidInput("start segment is missing".into()));
    }
    let mut out = Vec::with_capacity(n);
    for offset in 0..n {
        let Some(seg) = segs.get((start + offset) % n) else {
            return Err(PathError::InvalidInput("segment is missing".into()));
        };
        out.push(*seg);
    }
    Contour::try_new(out, true, tolerance).map_err(PathError::from)
}

fn split_segment(seg: Segment, t: f64) -> Result<(Segment, Segment)> {
    if !(0.0..=1.0).contains(&t) {
        return Err(PathError::InvalidInput("split parameter is outside [0, 1]".into()));
    }
    match seg {
        Segment::Line(line) => {
            let point = point_on_line(line, t);
            let left = Segment::Line(Line::new(line.start(), point)?);
            let right = Segment::Line(Line::new(point, line.end())?);
            Ok((left, right))
        }
        Segment::Arc(arc) => {
            let left_sweep = arc.sweep() * t;
            let right_sweep = arc.sweep() * (1.0 - t);
            let left = Arc::new(arc.center(), arc.radius(), arc.start_angle(), left_sweep)?;
            let right = Arc::new(arc.center(), arc.radius(), arc.start_angle() + left_sweep, right_sweep)?;
            Ok((Segment::Arc(left), Segment::Arc(right)))
        }
    }
}

fn trim_segment(seg: Segment, t0: f64, t1: f64) -> Result<Segment> {
    match seg {
        Segment::Line(line) => {
            let from = point_on_line(line, t0);
            let to = point_on_line(line, t1);
            Ok(Segment::Line(Line::new(from, to)?))
        }
        Segment::Arc(arc) => {
            let sweep = arc.sweep() * (t1 - t0);
            let start = arc.start_angle() + arc.sweep() * t0;
            Ok(Segment::Arc(Arc::new(arc.center(), arc.radius(), start, sweep)?))
        }
    }
}

fn point_on_line(line: Line, t: f64) -> Point {
    line.start() + line.direction() * t
}

fn point_on_segment(seg: Segment, t: f64) -> Point {
    match seg {
        Segment::Line(line) => point_on_line(line, t),
        Segment::Arc(arc) => arc.point_at(t.clamp(0.0, 1.0)),
    }
}

fn closest_on_segment(seg: Segment, point: Point) -> Result<(f64, Point)> {
    match seg {
        Segment::Line(line) => Ok(closest_on_line(line, point)),
        Segment::Arc(arc) => closest_on_arc(arc, point),
    }
}

fn closest_on_line(line: Line, point: Point) -> (f64, Point) {
    let direction = line.direction();
    let len2 = direction.dot(direction);
    if !len2.is_finite() || len2 <= 1e-24 {
        return (0.0, line.start());
    }
    let t = ((point - line.start()).dot(direction) / len2).clamp(0.0, 1.0);
    (t, point_on_line(line, t))
}

fn closest_on_arc(arc: Arc, point: Point) -> Result<(f64, Point)> {
    let offset = point - arc.center();
    let radial = offset.length();
    if radial > 1e-15
        && offset.angle().is_finite()
        && let Some(t) = param_of_angle(arc.start_angle(), arc.sweep(), offset.angle())
    {
        return Ok((t, arc.point_at(t)));
    }
    let start = arc.start_point();
    let end = arc.end_point();
    if start.distance(point) <= end.distance(point) { Ok((0.0, start)) } else { Ok((1.0, end)) }
}

fn snap_ends(seg: Segment, t: f64, point: Point, tol: f64) -> (f64, Point) {
    if point.almost_eq(seg.start(), tol) {
        (0.0, seg.start())
    } else if point.almost_eq(seg.end(), tol) {
        (1.0, seg.end())
    } else {
        (t, point)
    }
}

/// `t` in `[0, 1]` when `theta` lies on the sweep.
fn param_of_angle(start: f64, sweep: f64, theta: f64) -> Option<f64> {
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
    let slack = 1e-8;
    let adjusted = if TAU - along <= slack { 0.0 } else { along };
    if adjusted <= abs_sweep + slack { Some((adjusted / abs_sweep).clamp(0.0, 1.0)) } else { None }
}

pub(crate) fn start_tangent_of(seg: Segment) -> Result<Vec2> {
    tangent_on(seg, 0.0)
}

pub(crate) fn end_tangent_of(seg: Segment) -> Result<Vec2> {
    tangent_on(seg, 1.0)
}

fn tangent_on(seg: Segment, t: f64) -> Result<Vec2> {
    match seg {
        Segment::Line(line) => unit(line.direction()),
        Segment::Arc(arc) => {
            let theta = arc.start_angle() + arc.sweep() * t;
            let (sin_t, cos_t) = theta.sin_cos();
            if arc.sweep() >= 0.0 { unit(Vec2::new(-sin_t, cos_t)?) } else { unit(Vec2::new(sin_t, -cos_t)?) }
        }
    }
}

pub(crate) fn unit(v: Vec2) -> Result<Vec2> {
    let len = v.length();
    if !len.is_finite() || len <= f64::EPSILON {
        return Err(PathError::InvalidInput("direction is degenerate".into()));
    }
    Vec2::new(v.x() / len, v.y() / len).map_err(PathError::from)
}

pub(crate) fn left_normal(tangent: Vec2) -> Result<Vec2> {
    unit(Vec2::new(-tangent.y(), tangent.x())?)
}
