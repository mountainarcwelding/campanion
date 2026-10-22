//! Depth passes, with a linear ramp, a helix, or a plunge.
//!
//! Depth is a positive distance downward from the stock surface. Z is up, so pass `d` is cut
//! at `stock_z - d`. Passes are spaced by the step-down. The last pass is the total depth and
//! is not deeper: 5 mm at a 2 mm step is 2, 4, 5.
//!
//! A ramp follows the contour. Its angle with the XY plane never exceeds the given maximum.
//! A helix is a counter-clockwise circle about the contour's bounding-box centre; each full
//! turn lowers Z by the pitch. A plunge is a vertical move at the contour start. The contour
//! is then cut once at the pass height. The contour must be closed.

use campanion_geom::{Contour, Point, Segment, Tolerance};

use crate::Result;
use crate::check::{self, finite, positive};
use crate::toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};

const MAX_PASSES: usize = 1024;
const MAX_LAPS: usize = 1024;
const MAX_MM: f64 = 1.0e6;

/// How the tool enters each depth pass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DepthEntry {
    /// Follow the contour while descending. `max_angle` is radians from the XY plane.
    Ramp { max_angle: f64 },
    /// Counter-clockwise helix about the contour's bounding-box centre.
    ///
    /// `radius` and `pitch` are millimetres. A full turn is one arc and lowers Z by `pitch`.
    Helix { radius: f64, pitch: f64 },
    /// Straight down at the contour start.
    Plunge,
}

/// Stock height, total depth, step-down, and the entry move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthPlan {
    stock_z: f64,
    total_depth: f64,
    step_down: f64,
    entry: DepthEntry,
}

impl DepthPlan {
    /// `total_depth` and `step_down` are millimetres downward from `stock_z`.
    ///
    /// Values are checked when the plan is used.
    pub fn new(stock_z: f64, total_depth: f64, step_down: f64, entry: DepthEntry) -> Self {
        Self { stock_z, total_depth, step_down, entry }
    }

    pub fn stock_z(self) -> f64 {
        self.stock_z
    }

    pub fn total_depth(self) -> f64 {
        self.total_depth
    }

    pub fn step_down(self) -> f64 {
        self.step_down
    }

    pub fn entry(self) -> DepthEntry {
        self.entry
    }
}

/// Depth of each pass, in millimetres downward from the stock surface.
///
/// Passes are `step_down` apart until the last, which is `total_depth` and is not deeper.
/// Five millimetres at a two millimetre step is `[2, 4, 5]`.
pub fn pass_depths(total_depth: f64, step_down: f64) -> Result<Vec<f64>> {
    let total_depth = capped_positive(total_depth, "total depth")?;
    let step_down = capped_positive(step_down, "step-down")?;
    let steps = (total_depth / step_down).ceil();
    if !steps.is_finite() || steps > MAX_PASSES as f64 {
        return Err(check::invalid(format!("too many depth passes: limit is {MAX_PASSES}")));
    }
    let mut depths = Vec::new();
    let mut done = 0.0;
    loop {
        if depths.len() >= MAX_PASSES {
            return Err(check::invalid(format!("too many depth passes: limit is {MAX_PASSES}")));
        }
        let next = done + step_down;
        if !next.is_finite() {
            return Err(check::invalid("depth pass is not finite"));
        }
        if next >= total_depth - 1e-9 {
            depths.push(total_depth);
            break;
        }
        done = next;
        depths.push(done);
    }
    Ok(depths)
}

/// Cut `contour` once at each depth, entering with `plan.entry`.
pub fn depth_toolpath(contour: &Contour, plan: &DepthPlan, tolerance: &Tolerance) -> Result<Toolpath> {
    if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
        return Err(check::invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
    }
    if !contour.is_closed() {
        return Err(check::invalid("depth passes need a closed contour"));
    }
    let stock_z = capped_abs(plan.stock_z, "stock z")?;
    let depths = pass_depths(plan.total_depth, plan.step_down)?;
    let length = contour.length()?;
    if !length.is_finite() || length <= tolerance.linear() {
        return Err(check::invalid("contour length is too small"));
    }
    let floor = stock_z - plan.total_depth;
    finite(floor, "final depth")?;
    match plan.entry {
        DepthEntry::Ramp { max_angle } => ramp_passes(contour, length, stock_z, &depths, max_angle, tolerance),
        DepthEntry::Helix { radius, pitch } => helix_passes(contour, stock_z, &depths, radius, pitch),
        DepthEntry::Plunge => plunge_passes(contour, stock_z, &depths),
    }
}

struct Pen {
    moves: Vec<Move>,
    at: Option<Point3>,
}

impl Pen {
    fn new() -> Self {
        Self { moves: Vec::new(), at: None }
    }

    fn z(&self) -> Result<f64> {
        self.at.map(Point3::z).ok_or_else(|| check::invalid("toolpath has no position"))
    }

    fn rapid(&mut self, to: Point3) -> Result<()> {
        if self.at.is_some_and(|at| at.almost_eq(to, 1e-9)) {
            self.at = Some(to);
            return Ok(());
        }
        check::push(&mut self.moves, Move::Rapid { to })?;
        self.at = Some(to);
        Ok(())
    }

    fn linear(&mut self, to: Point3, kind: CutKind) -> Result<()> {
        if self.at.is_some_and(|at| at.almost_eq(to, 1e-9)) {
            self.at = Some(to);
            return Ok(());
        }
        check::push(&mut self.moves, Move::Linear { to, kind })?;
        self.at = Some(to);
        Ok(())
    }

    fn arc(&mut self, to: Point3, centre: Point3, ccw: bool, kind: CutKind) -> Result<()> {
        check::push(&mut self.moves, Move::Arc { to, centre, ccw, kind })?;
        self.at = Some(to);
        Ok(())
    }

    fn event(&mut self, event: ProcessEvent) -> Result<()> {
        check::push(&mut self.moves, Move::Event(event))
    }

    fn finish(self) -> Result<Toolpath> {
        Toolpath::new(self.moves)
    }
}

fn plunge_passes(contour: &Contour, stock_z: f64, depths: &[f64]) -> Result<Toolpath> {
    let mut pen = Pen::new();
    let start = first_point(contour)?;
    pen.rapid(Point3::new(start.x(), start.y(), stock_z)?)?;
    pen.event(ProcessEvent::On)?;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        pen.linear(Point3::new(start.x(), start.y(), target)?, CutKind::Plunge)?;
        cut_contour(&mut pen, contour, target)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn helix_passes(contour: &Contour, stock_z: f64, depths: &[f64], radius: f64, pitch: f64) -> Result<Toolpath> {
    let radius = capped_positive(radius, "helix radius")?;
    let pitch = capped_positive(pitch, "helix pitch")?;
    let bounds = contour.bounding_box()?;
    let cx = (bounds.min().x() + bounds.max().x()) * 0.5;
    let cy = (bounds.min().y() + bounds.max().y()) * 0.5;
    if !cx.is_finite() || !cy.is_finite() {
        return Err(check::invalid("helix centre is not finite"));
    }
    let mut pen = Pen::new();
    let start = helix_point(cx, cy, radius, 0.0, stock_z)?;
    pen.rapid(start)?;
    pen.event(ProcessEvent::On)?;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        let z = pen.z()?;
        pen.linear(helix_point(cx, cy, radius, 0.0, z)?, CutKind::LeadIn)?;
        descend_helix(&mut pen, cx, cy, radius, pitch, target)?;
        cut_contour(&mut pen, contour, target)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn descend_helix(pen: &mut Pen, cx: f64, cy: f64, radius: f64, pitch: f64, z_target: f64) -> Result<()> {
    let mut guard = 0;
    loop {
        let z = pen.z()?;
        let remain = z - z_target;
        if remain <= 1e-9 {
            break;
        }
        guard += 1;
        if guard > MAX_LAPS {
            return Err(check::invalid(format!("too many helix turns: limit is {MAX_LAPS}")));
        }
        if remain >= pitch - 1e-9 {
            let next = z - pitch;
            let to = helix_point(cx, cy, radius, 0.0, next)?;
            let centre = Point3::new(cx, cy, z)?;
            pen.arc(to, centre, true, CutKind::LeadIn)?;
        } else {
            let sweep = std::f64::consts::TAU * (remain / pitch);
            if !sweep.is_finite() || sweep <= 0.0 || sweep > std::f64::consts::TAU + 1e-6 {
                return Err(check::invalid("helix sweep is not usable"));
            }
            let to = helix_point(cx, cy, radius, sweep, z_target)?;
            let centre = Point3::new(cx, cy, z)?;
            pen.arc(to, centre, true, CutKind::LeadIn)?;
            break;
        }
    }
    Ok(())
}

fn helix_point(cx: f64, cy: f64, radius: f64, angle: f64, z: f64) -> Result<Point3> {
    Point3::new(cx + radius * angle.cos(), cy + radius * angle.sin(), z)
}

fn ramp_passes(contour: &Contour, length: f64, stock_z: f64, depths: &[f64], max_angle: f64, tolerance: &Tolerance) -> Result<Toolpath> {
    let max_angle = finite(max_angle, "ramp angle")?;
    if max_angle <= 0.0 || max_angle >= std::f64::consts::FRAC_PI_2 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let slope = max_angle.tan();
    if !slope.is_finite() || slope <= 0.0 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let eps = tolerance.linear().max(1e-9);
    let mut pen = Pen::new();
    let start = first_point(contour)?;
    pen.rapid(Point3::new(start.x(), start.y(), stock_z)?)?;
    pen.event(ProcessEvent::On)?;
    let mut s = 0.0;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        s = descend_along(&mut pen, contour, length, s, target, slope, eps)?;
        s = flat_lap(&mut pen, contour, length, s, target, eps)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn descend_along(pen: &mut Pen, contour: &Contour, length: f64, mut s: f64, z_target: f64, slope: f64, eps: f64) -> Result<f64> {
    let mut laps = 0;
    loop {
        let z = pen.z()?;
        if z <= z_target + eps {
            break;
        }
        laps += 1;
        if laps > MAX_LAPS {
            return Err(check::invalid(format!("too many ramp turns: limit is {MAX_LAPS}")));
        }
        if s >= length - eps {
            s = 0.0;
        }
        let room = length - s;
        if room <= eps {
            return Err(check::invalid("ramp span is empty"));
        }
        let need = (z - z_target) / slope;
        if !need.is_finite() {
            return Err(check::invalid("ramp length is not finite"));
        }
        if need <= 1e-9 {
            break;
        }
        let step = room.min(need);
        if step <= 1e-9 {
            return Err(check::invalid("ramp span is empty"));
        }
        let s1 = (s + step).min(length);
        let before = z;
        emit_sloped(pen, contour, s, s1, z_target, slope, CutKind::LeadIn)?;
        let after = pen.z()?;
        if after > before - 1e-12 {
            return Err(check::invalid("ramp did not descend"));
        }
        s = if s1 >= length - eps { 0.0 } else { s1 };
    }
    Ok(s)
}

fn flat_lap(pen: &mut Pen, contour: &Contour, length: f64, s: f64, z: f64, eps: f64) -> Result<f64> {
    if s <= eps || length - s <= eps {
        emit_at_height(pen, contour, 0.0, length, z, CutKind::Cut)?;
        return Ok(0.0);
    }
    emit_at_height(pen, contour, s, length, z, CutKind::Cut)?;
    emit_at_height(pen, contour, 0.0, s, z, CutKind::Cut)?;
    Ok(s)
}

/// Descend at most `slope` millimetres of Z per millimetre of path, and not below `z_floor`.
fn emit_sloped(pen: &mut Pen, contour: &Contour, s0: f64, s1: f64, z_floor: f64, slope: f64, kind: CutKind) -> Result<()> {
    let z0 = pen.z()?;
    let segs = spans(contour, s0, s1)?;
    let total = span_length(&segs)?;
    let drop_limit = (z0 - z_floor).max(0.0);
    let drop = (slope * total).min(drop_limit);
    let z1 = z0 - drop;
    emit_pieces(pen, &segs, z0, z1, kind)
}

fn emit_at_height(pen: &mut Pen, contour: &Contour, s0: f64, s1: f64, z: f64, kind: CutKind) -> Result<()> {
    let segs = spans(contour, s0, s1)?;
    let z0 = pen.z()?;
    emit_pieces(pen, &segs, z0, z, kind)
}

fn spans(contour: &Contour, s0: f64, s1: f64) -> Result<Vec<Segment>> {
    if !s0.is_finite() || !s1.is_finite() || s1 < s0 - 1e-9 {
        return Err(check::invalid("ramp span is empty"));
    }
    let segs = crate::curve::subdivide(contour, s0, s1)?;
    if segs.is_empty() {
        return Err(check::invalid("ramp span is empty"));
    }
    Ok(segs)
}

fn span_length(segs: &[Segment]) -> Result<f64> {
    let mut total = 0.0;
    for seg in segs {
        let len = seg.length();
        if !len.is_finite() || len < 0.0 {
            return Err(check::invalid("segment length is not finite"));
        }
        total += len;
    }
    if total <= 1e-15 {
        return Err(check::invalid("ramp span is empty"));
    }
    if !total.is_finite() {
        return Err(check::invalid("ramp span is not finite"));
    }
    Ok(total)
}

fn emit_pieces(pen: &mut Pen, segs: &[Segment], z0: f64, z1: f64, kind: CutKind) -> Result<()> {
    let total = span_length(segs)?;
    if !z1.is_finite() {
        return Err(check::invalid("pass height is not finite"));
    }
    let mut acc = 0.0;
    for seg in segs {
        let len = seg.length();
        acc += len;
        let mut z = z0 + (z1 - z0) * (acc / total);
        if (acc - total).abs() <= 1e-9 {
            z = z1;
        }
        if !z.is_finite() {
            return Err(check::invalid("pass height is not finite"));
        }
        emit_segment(pen, *seg, z, kind)?;
    }
    Ok(())
}

fn emit_segment(pen: &mut Pen, seg: Segment, z: f64, kind: CutKind) -> Result<()> {
    match seg {
        Segment::Line(line) => {
            pen.linear(Point3::new(line.end().x(), line.end().y(), z)?, kind)?;
        }
        Segment::Arc(arc) => {
            let to = Point3::new(arc.end_point().x(), arc.end_point().y(), z)?;
            let centre = Point3::new(arc.center().x(), arc.center().y(), z)?;
            pen.arc(to, centre, arc.sweep() > 0.0, kind)?;
        }
    }
    Ok(())
}

fn cut_contour(pen: &mut Pen, contour: &Contour, z: f64) -> Result<()> {
    let start = first_point(contour)?;
    pen.linear(Point3::new(start.x(), start.y(), z)?, CutKind::LeadIn)?;
    for seg in contour.segments() {
        emit_segment(pen, *seg, z, CutKind::Cut)?;
    }
    Ok(())
}

fn pass_height(stock_z: f64, depth: f64) -> Result<f64> {
    let z = stock_z - depth;
    finite(z, "pass height")
}

fn first_point(contour: &Contour) -> Result<Point> {
    contour.segments().first().map(|seg| seg.start()).ok_or_else(|| check::invalid("contour is empty"))
}

fn capped_positive(value: f64, what: &str) -> Result<f64> {
    let value = positive(value, what)?;
    if value > MAX_MM {
        return Err(check::invalid(format!("{what} is larger than {MAX_MM} mm")));
    }
    Ok(value)
}

fn capped_abs(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value.abs() > MAX_MM {
        return Err(check::invalid(format!("{what} is larger than {MAX_MM} mm")));
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_geom::{Line, Point, Segment};

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-6, "{a} != {b}");
    }

    fn rect(w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![edge(p(0.0, 0.0), p(w, 0.0)), edge(p(w, 0.0), p(w, h)), edge(p(w, h), p(0.0, h)), edge(p(0.0, h), p(0.0, 0.0))],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    #[test]
    fn five_millimetres_at_two_millimetre_step_is_two_four_five() {
        let depths = pass_depths(5.0, 2.0).unwrap();
        assert_eq!(depths, vec![2.0, 4.0, 5.0]);
        assert!(depths.iter().all(|depth| *depth <= 5.0));
        assert_eq!(depths.last().copied(), Some(5.0));

        // A step that lands on the total does not add another pass past it.
        assert_eq!(pass_depths(4.0, 2.0).unwrap(), vec![2.0, 4.0]);
        assert_eq!(pass_depths(2.0, 2.0).unwrap(), vec![2.0]);
        assert_eq!(pass_depths(2.0, 5.0).unwrap(), vec![2.0]);
        assert_eq!(pass_depths(6.0, 3.0).unwrap(), vec![3.0, 6.0]);

        let shape = rect(10.0, 10.0);
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Plunge);
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        let mut plunges = Vec::new();
        for mv in path.moves() {
            if let Move::Linear { to, kind: CutKind::Plunge } = *mv {
                plunges.push(to.z());
                assert!(to.z() >= -5.0 - 1e-9);
            }
        }
        assert_eq!(plunges.len(), 3);
        near(plunges[0], -2.0);
        near(plunges[1], -4.0);
        near(plunges[2], -5.0);
    }

    #[test]
    fn ramp_angle_is_never_exceeded() {
        let shape = rect(30.0, 10.0);
        let max_angle = 0.1_f64.atan();
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Ramp { max_angle });
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();

        let mut cursor: Option<Point3> = None;
        let mut steepest = 0.0_f64;
        let mut cut_z = Vec::new();
        for mv in path.moves() {
            let next = match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let (Some(from), Some(to)) = (cursor, next) {
                let horiz = (to.x() - from.x()).hypot(to.y() - from.y());
                let dz = (to.z() - from.z()).abs();
                if horiz > 1e-12 || dz > 1e-12 {
                    let angle = dz.atan2(horiz);
                    assert!(angle <= max_angle + 1e-6, "angle {angle} exceeds {max_angle}");
                    steepest = steepest.max(angle);
                }
            }
            if let Some(to) = next {
                if let Move::Linear { kind: CutKind::Cut, .. } = *mv {
                    cut_z.push(to.z());
                    assert!(to.z() >= -5.0 - 1e-6, "cut below the total depth at {}", to.z());
                }
                cursor = Some(to);
            }
        }
        assert!(steepest + 1e-6 >= max_angle * 0.5, "ramp did not use the allowed angle");
        for target in [-2.0, -4.0, -5.0] {
            assert!(cut_z.iter().any(|z| (*z - target).abs() < 1e-6), "missing cut at {target}: {cut_z:?}");
        }
    }

    #[test]
    fn helix_descends_by_pitch_per_turn() {
        let shape = rect(10.0, 10.0);
        let pitch = 2.0;
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Helix { radius: 1.0, pitch });
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();

        match path.moves().first() {
            Some(Move::Rapid { to }) => {
                near(to.x(), 6.0);
                near(to.y(), 5.0);
                near(to.z(), 0.0);
            }
            other => panic!("expected a rapid onto the helix, got {other:?}"),
        }

        let mut cursor: Option<Point3> = None;
        let mut full_drops = Vec::new();
        let mut arc_drop = 0.0;
        for mv in path.moves() {
            match *mv {
                Move::Arc { to, centre, ccw, kind: CutKind::LeadIn } => {
                    let from = cursor.expect("arc has a start");
                    assert!(ccw);
                    near(centre.x(), 5.0);
                    near(centre.y(), 5.0);
                    let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                    near(radius, 1.0);
                    near((to.x() - centre.x()).hypot(to.y() - centre.y()), 1.0);
                    let drop = from.z() - to.z();
                    assert!(drop >= -1e-9, "helix climbed");
                    let chord = (to.x() - from.x()).hypot(to.y() - from.y());
                    if chord <= 1e-6 {
                        full_drops.push(drop);
                    } else {
                        assert!(drop < pitch + 1e-8, "partial turn dropped {drop}, pitch is {pitch}");
                    }
                    arc_drop += drop;
                    cursor = Some(to);
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        assert_eq!(full_drops.len(), 2, "{full_drops:?}");
        for drop in full_drops {
            near(drop, pitch);
        }
        near(arc_drop, 5.0);
    }

    #[test]
    fn hostile_depth_input_is_an_error() {
        let shape = rect(10.0, 4.0);
        let tol = Tolerance::default();
        for (total, step) in [
            (f64::NAN, 2.0),
            (5.0, f64::NAN),
            (5.0, f64::INFINITY),
            (f64::NEG_INFINITY, 1.0),
            (0.0, 2.0),
            (-5.0, 2.0),
            (5.0, 0.0),
            (5.0, -1.0),
            (1.0e7, 1.0),
            (100.0, 1.0e-4),
        ] {
            assert!(pass_depths(total, step).is_err(), "accepted {total}, {step}");
        }

        let plunge = DepthEntry::Plunge;
        assert!(depth_toolpath(&shape, &DepthPlan::new(f64::NAN, 1.0, 1.0, plunge), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: 0.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: -0.2 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: std::f64::consts::FRAC_PI_2 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: f64::NAN }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: 0.0, pitch: 1.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: -1.0, pitch: 1.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: 1.0, pitch: 0.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: f64::INFINITY, pitch: 1.0 }), &tol).is_err());

        let open =
            Contour::try_new(vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(5.0, 0.0).unwrap()).unwrap())], false, &tol)
                .unwrap();
        assert!(depth_toolpath(&open, &DepthPlan::new(0.0, 1.0, 1.0, plunge), &tol).is_err());

        // A shallow angle on a short contour would spiral for far too many laps.
        let tiny = DepthPlan::new(0.0, 10.0, 10.0, DepthEntry::Ramp { max_angle: 1e-4 });
        let err = depth_toolpath(&shape, &tiny, &tol).unwrap_err();
        assert!(err.to_string().contains("ramp"), "{err}");

        let fine = DepthPlan::new(0.0, 10.0, 10.0, DepthEntry::Helix { radius: 1.0, pitch: 1.0e-4 });
        let err = depth_toolpath(&shape, &fine, &tol).unwrap_err();
        assert!(err.to_string().contains("helix"), "{err}");
    }
}
