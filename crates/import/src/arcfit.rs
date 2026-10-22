//! Fit quadratic and cubic Bézier curves with lines and circular arcs.
//!
//! Biarc join points use the incenter of the tangent triangle (equal tangent-length construction).
//! The maths is written from first principles (public-domain geometry).

use campanion_geom::{Arc, Line, Point, Segment};

use crate::{ImportError, Result, invalid};

const TAU: f64 = std::f64::consts::TAU;
const EPS_T: f64 = 1e-10;
const EPS_ZERO: f64 = 1e-12;
const EPS_UNIT: f64 = 1e-15;
const CONNECT_TOL: f64 = 1e-9;
const SAMPLE_STEPS: usize = 64;

/// Options for Bézier-to-arc fitting.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FitOptions {
    pub tolerance: f64,
    pub min_radius: f64,
    pub max_depth: u32,
    pub max_segments: usize,
}

impl FitOptions {
    pub(crate) fn new(tolerance: f64) -> Self {
        Self { tolerance, min_radius: tolerance, max_depth: 16, max_segments: 4096 }
    }
}

/// Fits a cubic Bézier (final output coordinates, mm, Y up) with lines and circular arcs.
pub(crate) fn fit_cubic(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64), opts: &FitOptions) -> Result<Vec<Segment>> {
    let c = cubic_from_tuples(p0, p1, p2, p3)?;
    validate_opts(opts)?;
    fit_cubic_inner(c, opts)
}

/// Elevate a quadratic Bézier to a cubic, then fit.
pub(crate) fn fit_quad(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), opts: &FitOptions) -> Result<Vec<Segment>> {
    let q0 = v2_from_tuple(p0)?;
    let q1 = v2_from_tuple(p1)?;
    let q2 = v2_from_tuple(p2)?;
    validate_opts(opts)?;
    let c1 = q0.add(q1.mul(2.0)).mul(1.0 / 3.0);
    let c2 = q1.mul(2.0).add(q2).mul(1.0 / 3.0);
    if !c1.is_finite() || !c2.is_finite() {
        return Err(invalid("non-finite coordinate"));
    }
    fit_cubic_inner(Cubic { p0: q0, p1: c1, p2: c2, p3: q2 }, opts)
}

fn fit_cubic_inner(c: Cubic, opts: &FitOptions) -> Result<Vec<Segment>> {
    if is_near_linear(&c, opts.tolerance) {
        return degenerate_output(&c, opts.tolerance);
    }
    let params = split_params(&c);
    let pieces = split_at_params(c, &params);
    let mut out = Vec::new();
    for piece in pieces {
        fit_piece(&piece, opts, 0, &mut out)?;
    }
    Ok(out)
}

fn validate_opts(opts: &FitOptions) -> Result<()> {
    if !(opts.tolerance > 0.0 && opts.tolerance.is_finite()) {
        return Err(invalid("tolerance must be positive and finite"));
    }
    Ok(())
}

fn cubic_from_tuples(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)) -> Result<Cubic> {
    Ok(Cubic { p0: v2_from_tuple(p0)?, p1: v2_from_tuple(p1)?, p2: v2_from_tuple(p2)?, p3: v2_from_tuple(p3)? })
}

fn v2_from_tuple(p: (f64, f64)) -> Result<V2> {
    if p.0.is_finite() && p.1.is_finite() { Ok(V2::new(p.0, p.1)) } else { Err(invalid("non-finite coordinate")) }
}

fn degenerate_output(c: &Cubic, tolerance: f64) -> Result<Vec<Segment>> {
    if c.p0.almost_eq(c.p3, EPS_ZERO) {
        let extent = c.p1.sub(c.p0).len().max(c.p2.sub(c.p0).len()).max(c.p3.sub(c.p0).len());
        if extent <= tolerance {
            return Ok(Vec::new());
        }
    }
    match make_line(c.p0, c.p3)? {
        Some(seg) => Ok(vec![seg]),
        None => Ok(Vec::new()),
    }
}

fn fit_piece(c: &Cubic, opts: &FitOptions, depth: u32, out: &mut Vec<Segment>) -> Result<()> {
    if out.len() >= opts.max_segments {
        return Err(invalid("curve needs too many segments"));
    }
    if is_near_linear(c, opts.tolerance) {
        if let Some(seg) = degenerate_output(c, opts.tolerance)?.into_iter().next() {
            push_seg(out, seg, opts.max_segments)?;
        }
        return Ok(());
    }
    if c.p0.almost_eq(c.p3, EPS_ZERO) {
        if depth >= opts.max_depth {
            return fallback_lines(c, opts, out);
        }
        let (left, right) = c.split(0.5);
        fit_piece(&left, opts, depth + 1, out)?;
        fit_piece(&right, opts, depth + 1, out)?;
        return Ok(());
    }
    match try_biarc(c, opts)? {
        Some(segs) => {
            for seg in segs {
                push_seg(out, seg, opts.max_segments)?;
            }
            Ok(())
        }
        None => {
            if depth >= opts.max_depth {
                fallback_lines(c, opts, out)
            } else {
                let (left, right) = c.split(0.5);
                fit_piece(&left, opts, depth + 1, out)?;
                fit_piece(&right, opts, depth + 1, out)
            }
        }
    }
}

fn push_seg(out: &mut Vec<Segment>, seg: Segment, max_segments: usize) -> Result<()> {
    if seg.length() < EPS_ZERO {
        return Ok(());
    }
    if out.len() >= max_segments {
        return Err(invalid("curve needs too many segments"));
    }
    out.push(seg);
    Ok(())
}

fn is_near_linear(c: &Cubic, tolerance: f64) -> bool {
    let d1 = dist_to_segment(c.p1, c.p0, c.p3);
    let d2 = dist_to_segment(c.p2, c.p0, c.p3);
    d1.max(d2) <= tolerance * 0.5
}

fn dist_to_segment(p: V2, a: V2, b: V2) -> f64 {
    let ab = b.sub(a);
    let len2 = ab.len2();
    if len2 < EPS_ZERO * EPS_ZERO {
        return p.sub(a).len();
    }
    let t = (p.sub(a).dot(ab) / len2).clamp(0.0, 1.0);
    p.sub(a.add(ab.mul(t))).len()
}

fn split_params(c: &Cubic) -> Vec<f64> {
    let mut ts = Vec::new();
    inflections(c, &mut ts);
    cusps(c, &mut ts);
    ts.retain(|t| t.is_finite() && *t > EPS_T && *t < 1.0 - EPS_T);
    ts.sort_by(|a, b| a.total_cmp(b));
    let mut unique: Vec<f64> = Vec::new();
    for t in ts {
        match unique.last() {
            Some(prev) if (t - prev).abs() < EPS_T => {}
            _ => unique.push(t),
        }
    }
    unique
}

fn inflections(c: &Cubic, out: &mut Vec<f64>) {
    let c1 = c.p1.sub(c.p0);
    let c2 = c.p0.sub(c.p1.mul(2.0)).add(c.p2);
    let c3 = c.p0.mul(-1.0).add(c.p1.mul(3.0)).sub(c.p2.mul(3.0)).add(c.p3);
    quadratic_roots(c2.cross(c3), c1.cross(c3), c1.cross(c2), out);
}

fn cusps(c: &Cubic, out: &mut Vec<f64>) {
    let c1 = c.p1.sub(c.p0);
    let c2 = c.p0.sub(c.p1.mul(2.0)).add(c.p2);
    let c3 = c.p0.mul(-1.0).add(c.p1.mul(3.0)).sub(c.p2.mul(3.0)).add(c.p3);
    let mut candidates = Vec::new();
    quadratic_roots(c3.x, 2.0 * c2.x, c1.x, &mut candidates);
    quadratic_roots(c3.y, 2.0 * c2.y, c1.y, &mut candidates);
    let scale = 1.0 + c1.len() + c2.len() + c3.len();
    let eps = 1e-10 * scale;
    for t in candidates {
        if t > EPS_T && t < 1.0 - EPS_T {
            let d = c.deriv(t);
            if d.len() <= eps {
                out.push(t);
            }
        }
    }
}

fn quadratic_roots(a: f64, b: f64, c: f64, out: &mut Vec<f64>) {
    if !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return;
    }
    let scale = 1.0 + a.abs() + b.abs() + c.abs();
    let eps = 1e-18 * scale;
    if a.abs() <= eps {
        if b.abs() <= eps {
            return;
        }
        let t = -c / b;
        if t.is_finite() {
            out.push(t);
        }
        return;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < -eps * eps {
        return;
    }
    if disc <= eps * eps {
        let t = -b / (2.0 * a);
        if t.is_finite() {
            out.push(t);
        }
        return;
    }
    let sqrt_d = disc.sqrt();
    let sign = if b >= 0.0 { 1.0 } else { -1.0 };
    let q = -0.5 * (b + sign * sqrt_d);
    if q.abs() > eps {
        let t0 = q / a;
        let t1 = c / q;
        if t0.is_finite() {
            out.push(t0);
        }
        if t1.is_finite() {
            out.push(t1);
        }
    } else {
        let t = -b / (2.0 * a);
        if t.is_finite() {
            out.push(t);
        }
    }
}

fn split_at_params(c: Cubic, ts: &[f64]) -> Vec<Cubic> {
    if ts.is_empty() {
        return vec![c];
    }
    let mut pieces = Vec::new();
    let mut rest = c;
    let mut prev_global = 0.0_f64;
    for &t_global in ts {
        let denom = 1.0 - prev_global;
        if denom.abs() < EPS_T {
            break;
        }
        let local = ((t_global - prev_global) / denom).clamp(0.0, 1.0);
        if local <= EPS_T {
            continue;
        }
        if local >= 1.0 - EPS_T {
            break;
        }
        let (left, right) = rest.split(local);
        pieces.push(left);
        rest = right;
        prev_global = t_global;
    }
    pieces.push(rest);
    pieces
}

fn try_biarc(c: &Cubic, opts: &FitOptions) -> Result<Option<Vec<Segment>>> {
    let Some(u0) = c.start_tangent().unit() else {
        return Ok(match make_line(c.p0, c.p3)? {
            Some(seg) => Some(vec![seg]),
            None => Some(Vec::new()),
        });
    };
    let Some(u1) = c.end_tangent().unit() else {
        return Ok(match make_line(c.p0, c.p3)? {
            Some(seg) => Some(vec![seg]),
            None => Some(Vec::new()),
        });
    };
    let join = biarc_join(c.p0, u0, c.p3, u1);
    if !join.is_finite() {
        return Ok(None);
    }
    let mut segs = Vec::new();
    if let Some(seg) = arc_from_start(c.p0, u0, join, opts)? {
        if arc_radius_too_small(&seg, opts.min_radius) {
            return Ok(None);
        }
        segs.push(seg);
    }
    if let Some(seg) = arc_from_end(join, c.p3, u1, opts)? {
        if arc_radius_too_small(&seg, opts.min_radius) {
            return Ok(None);
        }
        segs.push(seg);
    }
    if segs.is_empty() {
        return Ok(Some(Vec::new()));
    }
    let dev = sample_dev(&segs, c, SAMPLE_STEPS)?;
    if dev > opts.tolerance {
        return Ok(None);
    }
    Ok(Some(segs))
}

fn arc_radius_too_small(seg: &Segment, min_radius: f64) -> bool {
    match seg {
        Segment::Arc(a) => a.radius() < min_radius,
        Segment::Line(_) => false,
    }
}

fn biarc_join(p0: V2, t0: V2, p1: V2, t1: V2) -> V2 {
    let chord = p1.sub(p0);
    let det = t0.cross(t1);
    if det.abs() > 1e-8 {
        let s = chord.cross(t1) / det;
        let v = p0.add(t0.mul(s));
        if v.is_finite() {
            let a = v.sub(p1).len();
            let b = v.sub(p0).len();
            let c = chord.len();
            let sum = a + b + c;
            let far = a.max(b) > 1.0e6 * c.max(1.0);
            if sum > EPS_UNIT && a.is_finite() && b.is_finite() && !far {
                let join = p0.mul(a).add(p1.mul(b)).add(v.mul(c)).mul(1.0 / sum);
                if join.is_finite() {
                    return join;
                }
            }
        }
    }
    let denom = chord.dot(t0) + chord.dot(t1);
    let c2 = chord.len2();
    if denom.abs() > 1e-14 * (1.0 + c2) {
        let d = c2 / denom;
        if d.is_finite() && d.abs() < 1.0e8 {
            let join = p0.add(t0.mul(d)).add(p1.sub(t1.mul(d))).mul(0.5);
            if join.is_finite() {
                return join;
            }
        }
    }
    p0.add(p1).mul(0.5)
}

fn arc_from_start(p: V2, t_unit: V2, q: V2, _opts: &FitOptions) -> Result<Option<Segment>> {
    let h = q.sub(p);
    let h_len = h.len();
    if h_len < EPS_ZERO {
        return Ok(None);
    }
    let cr = t_unit.cross(h);
    let sin_a = cr.abs() / h_len;
    if sin_a <= 1e-10 {
        return make_line(p, q);
    }
    let t_param = (h_len * h_len) / (2.0 * cr);
    if !t_param.is_finite() {
        return make_line(p, q);
    }
    let n = V2::new(-t_unit.y, t_unit.x);
    let center = p.add(n.mul(t_param));
    let radius = t_param.abs();
    if !center.is_finite() || !radius.is_finite() || radius <= 0.0 {
        return make_line(p, q);
    }
    let sweep_half = (h_len * 0.5) / radius;
    let sagitta = if sweep_half.abs() < 1.0 { radius * (1.0 - (1.0 - sweep_half * sweep_half).max(0.0).sqrt()) } else { radius };
    if radius > 1.0e10 || sagitta <= EPS_ZERO {
        return make_line(p, q);
    }
    let start_angle = (p.y - center.y).atan2(p.x - center.x);
    let end_angle = (q.y - center.y).atan2(q.x - center.x);
    let Some(sweep) = signed_sweep(start_angle, end_angle, cr > 0.0) else {
        return make_line(p, q);
    };
    if sweep.abs() > TAU || !sweep.is_finite() || !start_angle.is_finite() {
        return make_line(p, q);
    }
    let center_pt = match to_point(center) {
        Ok(pt) => pt,
        Err(_) => return make_line(p, q),
    };
    match Arc::new(center_pt, radius, start_angle, sweep) {
        Ok(arc) => {
            if arc.length() < EPS_ZERO {
                make_line(p, q)
            } else {
                Ok(Some(Segment::Arc(arc)))
            }
        }
        Err(_) => make_line(p, q),
    }
}

fn arc_from_end(start: V2, end: V2, t_end: V2, opts: &FitOptions) -> Result<Option<Segment>> {
    match arc_from_start(end, t_end.mul(-1.0), start, opts)? {
        None => Ok(None),
        Some(Segment::Line(_)) => make_line(start, end),
        Some(Segment::Arc(arc)) => reverse_arc(arc, start, end),
    }
}

fn reverse_arc(arc: Arc, start: V2, end: V2) -> Result<Option<Segment>> {
    let new_start = arc.start_angle() + arc.sweep();
    let new_sweep = -arc.sweep();
    if !new_start.is_finite() || !new_sweep.is_finite() || new_sweep.abs() < 1e-15 || new_sweep.abs() > TAU {
        return make_line(start, end);
    }
    match Arc::new(arc.center(), arc.radius(), new_start, new_sweep) {
        Ok(rev) => {
            if rev.length() < EPS_ZERO {
                make_line(start, end)
            } else {
                Ok(Some(Segment::Arc(rev)))
            }
        }
        Err(_) => make_line(start, end),
    }
}

fn signed_sweep(start: f64, end: f64, ccw: bool) -> Option<f64> {
    let delta = end - start;
    if !delta.is_finite() {
        return None;
    }
    if ccw {
        let sweep = delta.rem_euclid(TAU);
        if sweep <= 1e-15 || sweep > TAU { None } else { Some(sweep) }
    } else {
        let sweep = delta.rem_euclid(TAU);
        if sweep <= 1e-15 { None } else { Some(sweep - TAU) }
    }
}

fn make_line(a: V2, b: V2) -> Result<Option<Segment>> {
    if a.almost_eq(b, EPS_ZERO) {
        return Ok(None);
    }
    let p = to_point(a)?;
    let q = to_point(b)?;
    match Line::new(p, q) {
        Ok(line) => Ok(Some(Segment::Line(line))),
        Err(_) => Ok(None),
    }
}

fn to_point(v: V2) -> Result<Point> {
    Point::new(v.x, v.y).map_err(ImportError::from_geom)
}

fn sample_dev(segments: &[Segment], c: &Cubic, steps: usize) -> Result<f64> {
    let steps = steps.max(32);
    let mut max_d = 0.0_f64;
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        max_d = max_d.max(point_dev(segments, c.eval(t), c.p0)?);
    }
    for t in [0.25, 0.5, 0.75] {
        max_d = max_d.max(point_dev(segments, c.eval(t), c.p0)?);
    }
    Ok(max_d)
}

fn point_dev(segments: &[Segment], p: V2, origin: V2) -> Result<f64> {
    if segments.is_empty() {
        return Ok(p.sub(origin).len());
    }
    let pt = to_point(p)?;
    let mut min_d = f64::INFINITY;
    for seg in segments {
        let d = seg.distance_to_point(pt);
        if d < min_d {
            min_d = d;
        }
    }
    if min_d.is_finite() { Ok(min_d) } else { Ok(p.sub(origin).len()) }
}

fn fallback_lines(c: &Cubic, opts: &FitOptions, out: &mut Vec<Segment>) -> Result<()> {
    let remaining = opts.max_segments.saturating_sub(out.len());
    if remaining == 0 {
        return Err(invalid("curve needs too many segments"));
    }
    let mut n = 1usize;
    loop {
        let n_use = n.min(remaining);
        let segs = lines_through_samples(c, n_use)?;
        let check_steps = SAMPLE_STEPS.max(n_use.saturating_mul(4)).min(512);
        let dev = sample_dev(&segs, c, check_steps)?;
        if dev <= opts.tolerance {
            for seg in segs {
                push_seg(out, seg, opts.max_segments)?;
            }
            return Ok(());
        }
        if n_use >= remaining {
            return Err(invalid("curve needs too many segments"));
        }
        let next = n.saturating_mul(2);
        n = if next > remaining { remaining } else { next.max(n + 1) };
    }
}

fn lines_through_samples(c: &Cubic, n_segs: usize) -> Result<Vec<Segment>> {
    let n = n_segs.max(1);
    let mut segs = Vec::new();
    let mut prev = c.eval(0.0);
    for i in 1..=n {
        let t = i as f64 / n as f64;
        let curr = c.eval(t);
        if let Some(seg) = make_line(prev, curr)? {
            segs.push(seg);
        }
        prev = curr;
    }
    Ok(segs)
}

#[derive(Clone, Copy, Debug)]
struct V2 {
    x: f64,
    y: f64,
}

impl V2 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    fn add(self, o: Self) -> Self {
        Self { x: self.x + o.x, y: self.y + o.y }
    }

    fn sub(self, o: Self) -> Self {
        Self { x: self.x - o.x, y: self.y - o.y }
    }

    fn mul(self, s: f64) -> Self {
        Self { x: self.x * s, y: self.y * s }
    }

    fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y
    }

    fn cross(self, o: Self) -> f64 {
        self.x * o.y - self.y * o.x
    }

    fn len2(self) -> f64 {
        self.dot(self)
    }

    fn len(self) -> f64 {
        self.len2().sqrt()
    }

    fn unit(self) -> Option<Self> {
        let l = self.len();
        if l < EPS_UNIT || !l.is_finite() { None } else { Some(self.mul(1.0 / l)) }
    }

    fn lerp(self, o: Self, t: f64) -> Self {
        Self { x: self.x + (o.x - self.x) * t, y: self.y + (o.y - self.y) * t }
    }

    fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    fn almost_eq(self, o: Self, tol: f64) -> bool {
        self.sub(o).len() <= tol
    }
}

#[derive(Clone, Copy, Debug)]
struct Cubic {
    p0: V2,
    p1: V2,
    p2: V2,
    p3: V2,
}

impl Cubic {
    fn eval(self, t: f64) -> V2 {
        let u = 1.0 - t;
        let uu = u * u;
        let tt = t * t;
        self.p0.mul(uu * u).add(self.p1.mul(3.0 * uu * t)).add(self.p2.mul(3.0 * u * tt)).add(self.p3.mul(tt * t))
    }

    fn deriv(self, t: f64) -> V2 {
        let u = 1.0 - t;
        let a = self.p1.sub(self.p0);
        let b = self.p2.sub(self.p1);
        let c = self.p3.sub(self.p2);
        a.mul(3.0 * u * u).add(b.mul(6.0 * u * t)).add(c.mul(3.0 * t * t))
    }

    fn split(self, t: f64) -> (Self, Self) {
        let p01 = self.p0.lerp(self.p1, t);
        let p12 = self.p1.lerp(self.p2, t);
        let p23 = self.p2.lerp(self.p3, t);
        let p012 = p01.lerp(p12, t);
        let p123 = p12.lerp(p23, t);
        let p0123 = p012.lerp(p123, t);
        (Self { p0: self.p0, p1: p01, p2: p012, p3: p0123 }, Self { p0: p0123, p1: p123, p2: p23, p3: self.p3 })
    }

    fn start_tangent(self) -> V2 {
        let d = self.deriv(0.0);
        if d.len() > EPS_ZERO {
            return d;
        }
        let d2 = self.p2.sub(self.p0);
        if d2.len() > EPS_ZERO {
            return d2;
        }
        self.p3.sub(self.p0)
    }

    fn end_tangent(self) -> V2 {
        let d = self.deriv(1.0);
        if d.len() > EPS_ZERO {
            return d;
        }
        let d2 = self.p3.sub(self.p1);
        if d2.len() > EPS_ZERO {
            return d2;
        }
        self.p3.sub(self.p0)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn max_dev(segments: &[Segment], cubic: [(f64, f64); 4], samples: usize) -> f64 {
        let c = Cubic {
            p0: V2::new(cubic[0].0, cubic[0].1),
            p1: V2::new(cubic[1].0, cubic[1].1),
            p2: V2::new(cubic[2].0, cubic[2].1),
            p3: V2::new(cubic[3].0, cubic[3].1),
        };
        sample_dev(segments, &c, samples).unwrap()
    }

    fn connected(segments: &[Segment], p0: (f64, f64), p3: (f64, f64)) -> bool {
        if segments.is_empty() {
            return (p0.0 - p3.0).hypot(p0.1 - p3.1) <= CONNECT_TOL;
        }
        let first = segments.first().unwrap();
        let last = segments.last().unwrap();
        if (first.start().x() - p0.0).hypot(first.start().y() - p0.1) > CONNECT_TOL {
            return false;
        }
        if (last.end().x() - p3.0).hypot(last.end().y() - p3.1) > CONNECT_TOL {
            return false;
        }
        for w in segments.windows(2) {
            if (w[0].end().x() - w[1].start().x()).hypot(w[0].end().y() - w[1].start().y()) > CONNECT_TOL {
                return false;
            }
        }
        true
    }

    fn arc_sweeps(segments: &[Segment]) -> Vec<f64> {
        segments
            .iter()
            .filter_map(|s| match s {
                Segment::Arc(a) => Some(a.sweep()),
                Segment::Line(_) => None,
            })
            .collect()
    }

    #[test]
    fn quarter_circle_radius_10() {
        const K: f64 = 0.5522847498;
        let r = 10.0;
        let p0 = (r, 0.0);
        let p1 = (r, r * K);
        let p2 = (r * K, r);
        let p3 = (0.0, r);
        let opts = FitOptions::new(0.01);
        let segs = fit_cubic(p0, p1, p2, p3, &opts).unwrap();
        let n_arcs = segs.iter().filter(|s| matches!(s, Segment::Arc(_))).count();
        assert!((1..=2).contains(&n_arcs), "expected 1 or 2 arcs, got {n_arcs} in {} segments", segs.len());
        for s in &segs {
            if let Segment::Arc(a) = s {
                assert!((a.radius() - 10.0).abs() <= 0.05, "radius {}", a.radius());
            }
        }
        let dev = max_dev(&segs, [p0, p1, p2, p3], 512);
        eprintln!("quarter circle: max_dev={dev} segments={}", segs.len());
        assert!(dev <= opts.tolerance, "max_dev={dev}");
        assert!(connected(&segs, p0, p3));

        let m = |p: (f64, f64)| (-p.0, p.1);
        let segs_m = fit_cubic(m(p0), m(p1), m(p2), m(p3), &opts).unwrap();
        let sw = arc_sweeps(&segs);
        let sw_m = arc_sweeps(&segs_m);
        assert!(!sw.is_empty() && !sw_m.is_empty());
        let sign = |v: &[f64]| v.iter().map(|s| s.signum()).sum::<f64>();
        assert!(sign(&sw) * sign(&sw_m) < 0.0, "mirrored sweeps should flip sign: {sw:?} vs {sw_m:?}");
    }

    #[test]
    fn s_curve_splits_at_inflection() {
        let p0 = (0.0, 0.0);
        let p1 = (10.0, 10.0);
        let p2 = (20.0, -10.0);
        let p3 = (30.0, 0.0);
        let opts = FitOptions::new(0.01);
        let segs = fit_cubic(p0, p1, p2, p3, &opts).unwrap();
        assert!(segs.len() >= 2, "expected split, got {} segments", segs.len());
        let sweeps = arc_sweeps(&segs);
        assert!(sweeps.iter().any(|s| *s > 0.0) && sweeps.iter().any(|s| *s < 0.0), "expected opposite sweep signs, got {sweeps:?}");
        let dev = max_dev(&segs, [p0, p1, p2, p3], 512);
        eprintln!("s-curve: max_dev={dev} segments={}", segs.len());
        assert!(dev <= opts.tolerance, "max_dev={dev}");
        assert!(connected(&segs, p0, p3));
    }

    #[test]
    fn near_straight_is_one_line() {
        let p0 = (0.0, 0.0);
        let p1 = (1.0, 1e-6);
        let p2 = (2.0, 1e-6);
        let p3 = (3.0, 0.0);
        let segs = fit_cubic(p0, p1, p2, p3, &FitOptions::new(0.01)).unwrap();
        assert_eq!(segs.len(), 1);
        assert!(matches!(segs[0], Segment::Line(_)));
        assert!(connected(&segs, p0, p3));
    }

    #[test]
    fn cusp_cubic_fits() {
        let p0 = (0.0, 0.0);
        let p1 = (10.0, 10.0);
        let p2 = (0.0, 10.0);
        let p3 = (10.0, 0.0);
        let opts = FitOptions::new(0.01);
        let segs = fit_cubic(p0, p1, p2, p3, &opts).unwrap();
        let dev = max_dev(&segs, [p0, p1, p2, p3], 512);
        assert!(dev <= opts.tolerance, "max_dev={dev}");
        assert!(connected(&segs, p0, p3));
    }

    #[test]
    fn tight_cubic_respects_min_radius() {
        let p0 = (0.0, 0.0);
        let p1 = (0.01, 1.0);
        let p2 = (-0.01, 1.0);
        let p3 = (0.0, 0.0);
        let opts = FitOptions::new(0.01);
        let segs = fit_cubic(p0, p1, p2, p3, &opts).unwrap();
        for s in &segs {
            if let Segment::Arc(a) = s {
                assert!(a.radius() >= opts.min_radius, "radius {} < min {}", a.radius(), opts.min_radius);
            }
        }
        assert!(connected(&segs, p0, p3));
    }

    #[test]
    fn quad_fits_and_connects() {
        let p0 = (0.0, 0.0);
        let p1 = (5.0, 10.0);
        let p2 = (10.0, 0.0);
        let opts = FitOptions::new(0.01);
        let segs = fit_quad(p0, p1, p2, &opts).unwrap();
        let q1 = ((p0.0 + 2.0 * p1.0) / 3.0, (p0.1 + 2.0 * p1.1) / 3.0);
        let q2 = ((2.0 * p1.0 + p2.0) / 3.0, (2.0 * p1.1 + p2.1) / 3.0);
        let dev = max_dev(&segs, [p0, q1, q2, p2], 512);
        assert!(dev <= opts.tolerance, "max_dev={dev}");
        assert!(connected(&segs, p0, p2));
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(256))]

        #[test]
        fn random_cubics(
            x0 in -100.0f64..100.0,
            y0 in -100.0f64..100.0,
            x1 in -100.0f64..100.0,
            y1 in -100.0f64..100.0,
            x2 in -100.0f64..100.0,
            y2 in -100.0f64..100.0,
            x3 in -100.0f64..100.0,
            y3 in -100.0f64..100.0,
            tol in 0.001f64..1.0,
        ) {
            let p0 = (x0, y0);
            let p1 = (x1, y1);
            let p2 = (x2, y2);
            let p3 = (x3, y3);
            let opts = FitOptions::new(tol);
            match fit_cubic(p0, p1, p2, p3, &opts) {
                Ok(segs) => {
                    prop_assert!(connected(&segs, p0, p3), "not connected");
                    let dev = max_dev(&segs, [p0, p1, p2, p3], 512);
                    prop_assert!(dev <= tol * 1.0001, "max_dev={dev} tol={tol}");
                }
                Err(e) => {
                    let msg = format!("{e}");
                    prop_assert!(msg.contains("too many segments"), "unexpected error: {msg}");
                }
            }
        }

        #[test]
        fn random_quads(
            x0 in -100.0f64..100.0,
            y0 in -100.0f64..100.0,
            x1 in -100.0f64..100.0,
            y1 in -100.0f64..100.0,
            x2 in -100.0f64..100.0,
            y2 in -100.0f64..100.0,
            tol in 0.001f64..1.0,
        ) {
            let p0 = (x0, y0);
            let p1 = (x1, y1);
            let p2 = (x2, y2);
            let opts = FitOptions::new(tol);
            match fit_quad(p0, p1, p2, &opts) {
                Ok(segs) => {
                    prop_assert!(connected(&segs, p0, p2), "not connected");
                    let q1 = ((p0.0 + 2.0 * p1.0) / 3.0, (p0.1 + 2.0 * p1.1) / 3.0);
                    let q2 = ((2.0 * p1.0 + p2.0) / 3.0, (2.0 * p1.1 + p2.1) / 3.0);
                    let dev = max_dev(&segs, [p0, q1, q2, p2], 512);
                    prop_assert!(dev <= tol * 1.0001, "max_dev={dev} tol={tol}");
                }
                Err(e) => {
                    let msg = format!("{e}");
                    prop_assert!(msg.contains("too many segments"), "unexpected error: {msg}");
                }
            }
        }
    }
}
