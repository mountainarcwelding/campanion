//! Face the stock over a rectangle.
//!
//! Passes run parallel to the angle and step over outside the area, so the link between two
//! lines does not cross the rectangle. Each depth repeats that pattern. [`crate::pass_depths`]
//! chooses the levels: the last one is the total depth and is not deeper.
//!
//! An angle of `0` runs along +X. `π/2` runs along +Y. Any other angle is
//! [`crate::PathError::NotSupported`].

use campanion_geom::BoundingBox;

use crate::check::{self, finite, invalid, non_negative, positive};
use crate::pass_depths;
use crate::toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};
use crate::{PathError, Result};

/// Parallel lines in one depth level. A hostile stepover fails instead of allocating without bound.
const MAX_LINES: usize = 100_000;

const ANGLE_EPS: f64 = 1e-9;

/// A finished facing toolpath.
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    toolpath: Toolpath,
    pass_count: usize,
}

impl Face {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Lines in one depth level.
    ///
    /// Depth repeats the same pattern. It does not change this count.
    pub fn pass_count(&self) -> usize {
        self.pass_count
    }
}

/// Parameters for facing a rectangle.
///
/// Values are checked when [`Self::generate`] runs.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceOp {
    tool_diameter: f64,
    stepover: f64,
    angle: f64,
    overrun: Option<f64>,
    stock_z: f64,
    depth: f64,
    step_down: f64,
    retract_z: f64,
}

impl FaceOp {
    /// `stepover` is the distance between parallel passes, in millimetres.
    ///
    /// It must be positive and no larger than `tool_diameter`.
    pub fn new(tool_diameter: f64, stepover: f64) -> Self {
        Self { tool_diameter, stepover, angle: 0.0, overrun: None, stock_z: 0.0, depth: 0.0, step_down: 0.0, retract_z: 2.0 }
    }

    /// Direction of the passes, in radians from +X.
    ///
    /// `0` runs along +X and steps over in Y. [`std::f64::consts::FRAC_PI_2`] runs along +Y.
    /// Any other angle is [`PathError::NotSupported`].
    pub fn angle(mut self, radians: f64) -> Self {
        self.angle = radians;
        self
    }

    /// How far each pass runs past the rectangle, in millimetres.
    ///
    /// The default is the tool radius, so the cutter clears the edge. Each pass starts and ends
    /// this far beyond the area, and the step to the next line is cut out there.
    pub fn overrun(mut self, mm: f64) -> Self {
        self.overrun = Some(mm);
        self
    }

    /// Z of the stock surface. A pass of depth `d` is cut at `stock_z - d`. Z is up.
    pub fn stock_z(mut self, z: f64) -> Self {
        self.stock_z = z;
        self
    }

    /// Total depth downward from [`Self::stock_z`], in millimetres. Zero is rejected.
    pub fn depth(mut self, total: f64) -> Self {
        self.depth = total;
        self
    }

    /// Maximum depth of one pass, in millimetres. The last pass stops at the total depth.
    pub fn step_down(mut self, step: f64) -> Self {
        self.step_down = step;
        self
    }

    /// Safe height for the rapid into each pass and the retract out. Defaults to 2 mm.
    pub fn retract_z(mut self, z: f64) -> Self {
        self.retract_z = z;
        self
    }

    /// Face `area`.
    ///
    /// Line centres run from one cross-edge of the rectangle to the other, with a gap of at most
    /// `stepover`. The tool diameter is at least that gap, so the discs cover the rectangle.
    pub fn generate(&self, area: &BoundingBox) -> Result<Face> {
        let diameter = positive(self.tool_diameter, "tool diameter")?;
        let stepover = positive(self.stepover, "stepover")?;
        if stepover > diameter {
            return Err(invalid("stepover must be <= tool diameter"));
        }
        let radius = diameter * 0.5;
        if !radius.is_finite() {
            return Err(invalid("tool radius is not finite"));
        }
        let overrun = match self.overrun {
            Some(value) => non_negative(value, "overrun")?,
            None => radius,
        };
        let along_x = along_x(self.angle)?;
        let stock_z = finite(self.stock_z, "stock z")?;
        let retract_z = finite(self.retract_z, "retract z")?;
        let depths = pass_depths(self.depth, self.step_down)?;

        let min_x = finite(area.min().x(), "area")?;
        let min_y = finite(area.min().y(), "area")?;
        let max_x = finite(area.max().x(), "area")?;
        let max_y = finite(area.max().y(), "area")?;
        let (min_along, max_along, min_cross, max_cross) = if along_x { (min_x, max_x, min_y, max_y) } else { (min_y, max_y, min_x, max_x) };
        let start_along = finite(min_along - overrun, "pass start")?;
        let far_along = finite(max_along + overrun, "pass end")?;
        let crosses = cross_positions(min_cross, max_cross, stepover)?;
        let pass_count = crosses.len();

        let mut moves = Vec::new();
        for (index, depth) in depths.iter().enumerate() {
            let z = finite(stock_z - depth, "pass height")?;
            emit_level(&mut moves, along_x, start_along, far_along, &crosses, z, retract_z, index == 0)?;
        }
        check::push(&mut moves, Move::Event(ProcessEvent::Off))?;
        Ok(Face { toolpath: Toolpath::new(moves)?, pass_count })
    }
}

/// `true` when passes run along +X. `false` when they run along +Y.
fn along_x(angle: f64) -> Result<bool> {
    let angle = finite(angle, "angle")?;
    if angle.abs() <= ANGLE_EPS {
        Ok(true)
    } else if (angle - std::f64::consts::FRAC_PI_2).abs() <= ANGLE_EPS {
        Ok(false)
    } else {
        Err(PathError::NotSupported(format!("facing angle {angle}")))
    }
}

/// Centres from `min_cross` to `max_cross` inclusive, with a gap of at most `stepover`.
///
/// A zero width is one line on that coordinate.
fn cross_positions(min_cross: f64, max_cross: f64, stepover: f64) -> Result<Vec<f64>> {
    let width = max_cross - min_cross;
    if !width.is_finite() || width < 0.0 {
        return Err(invalid("area width is not usable"));
    }
    if width == 0.0 {
        return Ok(vec![min_cross]);
    }
    let mut intervals = (width / stepover).ceil();
    if !intervals.is_finite() {
        return Err(invalid(format!("too many facing lines: limit is {MAX_LINES}")));
    }
    if intervals < 1.0 {
        intervals = 1.0;
    }
    // `ceil` can leave the gap a fraction over `stepover`. One extra interval closes it.
    let mut guard = 0;
    while width > stepover * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 4 || !intervals.is_finite() {
            return Err(invalid("facing spacing is not usable"));
        }
    }
    if !(1.0..MAX_LINES as f64).contains(&intervals) {
        return Err(invalid(format!("too many facing lines: limit is {MAX_LINES}")));
    }
    let gaps = intervals as usize;
    let mut lines = Vec::with_capacity(gaps + 1);
    for i in 0..gaps {
        let pos = min_cross + width * (i as f64) / (gaps as f64);
        if !pos.is_finite() {
            return Err(invalid("facing line is not finite"));
        }
        lines.push(pos);
    }
    lines.push(max_cross);
    Ok(lines)
}

fn at(along_x: bool, along: f64, cross: f64, z: f64) -> Result<Point3> {
    if along_x { Point3::new(along, cross, z) } else { Point3::new(cross, along, z) }
}

/// One depth: rapid, optional process-on, plunge outside the area, zigzag, retract.
///
/// The first pass runs toward `far_along`. Each later pass flips direction. The stepover between
/// them is a cut at the overrun end, outside the rectangle.
fn emit_level(
    moves: &mut Vec<Move>,
    along_x: bool,
    start_along: f64,
    far_along: f64,
    crosses: &[f64],
    z: f64,
    retract_z: f64,
    turn_on: bool,
) -> Result<()> {
    let Some(&first_cross) = crosses.first() else {
        return Err(invalid("facing produced no lines"));
    };
    check::push(moves, Move::Rapid { to: at(along_x, start_along, first_cross, retract_z)? })?;
    if turn_on {
        check::push(moves, Move::Event(ProcessEvent::On))?;
    }
    check::push(moves, Move::Linear { to: at(along_x, start_along, first_cross, z)?, kind: CutKind::Plunge })?;

    let mut along = far_along;
    let mut cross = first_cross;
    let mut went_toward_far = true;
    check::push(moves, Move::Linear { to: at(along_x, along, cross, z)?, kind: CutKind::Cut })?;
    for next in crosses.iter().copied().skip(1) {
        check::push(moves, Move::Linear { to: at(along_x, along, next, z)?, kind: CutKind::Cut })?;
        went_toward_far = !went_toward_far;
        along = if went_toward_far { far_along } else { start_along };
        cross = next;
        check::push(moves, Move::Linear { to: at(along_x, along, cross, z)?, kind: CutKind::Cut })?;
    }
    check::push(moves, Move::Linear { to: at(along_x, along, cross, retract_z)?, kind: CutKind::Retract })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PathError;
    use crate::pass_depths;
    use campanion_geom::Point;
    use proptest::prelude::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn rectangle(x: f64, y: f64, w: f64, h: f64) -> BoundingBox {
        BoundingBox::from_points(Point::new(x, y).unwrap(), Point::new(x + w, y + h).unwrap())
    }

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    fn assert_at(point: Point3, x: f64, y: f64, z: f64) {
        near(point.x(), x);
        near(point.y(), y);
        near(point.z(), z);
    }

    fn dist_to_box(x: f64, y: f64, area: BoundingBox) -> f64 {
        let min = area.min();
        let max = area.max();
        let dx = if x < min.x() {
            min.x() - x
        } else if x > max.x() {
            x - max.x()
        } else {
            0.0
        };
        let dy = if y < min.y() {
            min.y() - y
        } else if y > max.y() {
            y - max.y()
        } else {
            0.0
        };
        dx.hypot(dy)
    }

    struct Seg {
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
        z: f64,
    }

    fn cuts(path: &Toolpath) -> Vec<Seg> {
        let mut cursor = None;
        let mut out = Vec::new();
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } => cursor = Some(to),
                Move::Linear { to, kind } => {
                    if kind == CutKind::Cut
                        && let Some(from) = cursor
                    {
                        out.push(Seg { x0: from.x(), y0: from.y(), x1: to.x(), y1: to.y(), z: to.z() });
                    }
                    cursor = Some(to);
                }
                Move::Arc { to, .. } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        out
    }

    fn dist_to_seg(x: f64, y: f64, seg: &Seg) -> f64 {
        let dx = seg.x1 - seg.x0;
        let dy = seg.y1 - seg.y0;
        let len2 = dx * dx + dy * dy;
        if len2 <= 1e-18 {
            return (x - seg.x0).hypot(y - seg.y0);
        }
        let t = (((x - seg.x0) * dx + (y - seg.y0) * dy) / len2).clamp(0.0, 1.0);
        (x - (seg.x0 + t * dx)).hypot(y - (seg.y0 + t * dy))
    }

    fn axis_samples(min: f64, max: f64, step: f64) -> Vec<f64> {
        if !step.is_finite() || step <= 0.0 || max <= min + 1e-12 {
            return vec![min];
        }
        let mut values = Vec::new();
        let mut cursor = min;
        let mut guard = 0;
        while cursor < max - 1e-9 {
            values.push(cursor);
            cursor += step;
            guard += 1;
            if guard > 100_000 {
                break;
            }
        }
        values.push(max);
        values
    }

    fn assert_covered(area: BoundingBox, segs: &[Seg], radius: f64, step: f64) {
        assert!(!segs.is_empty(), "no cutting lines");
        let min = area.min();
        let max = area.max();
        for x in axis_samples(min.x(), max.x(), step) {
            for y in axis_samples(min.y(), max.y(), step) {
                let best = segs.iter().fold(f64::INFINITY, |best, seg| best.min(dist_to_seg(x, y, seg)));
                assert!(best <= radius + 1e-6, "({x}, {y}) is {best} from a cutting line, radius {radius}");
            }
        }
    }

    fn plunges(path: &Toolpath) -> Vec<Point3> {
        path.moves()
            .iter()
            .filter_map(|mv| match *mv {
                Move::Linear { to, kind: CutKind::Plunge } => Some(to),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn thirty_by_twenty_is_covered_and_plunges_outside() {
        // 10 mm tool, 6 mm stepover. Cross width 20 → ceil(20/6)+1 = 5 lines, 5 mm apart,
        // on Y = 0, 5, 10, 15, 20. Overrun is the 5 mm radius, so each pass runs X = -5..35.
        let area = rectangle(0.0, 0.0, 30.0, 20.0);
        let face = FaceOp::new(10.0, 6.0).angle(0.0).depth(1.0).step_down(1.0).generate(&area).expect("face");
        assert_eq!(face.pass_count(), 5);
        let moves = face.toolpath().moves();
        let radius = 5.0;
        let final_z = -1.0;

        let ends =
            [(-5.0, 0.0), (35.0, 0.0), (35.0, 5.0), (-5.0, 5.0), (-5.0, 10.0), (35.0, 10.0), (35.0, 15.0), (-5.0, 15.0), (-5.0, 20.0), (35.0, 20.0)];
        assert!(matches!(moves.first(), Some(Move::Rapid { .. })));
        assert!(matches!(moves.get(1), Some(Move::Event(ProcessEvent::On))));
        assert!(matches!(moves.last(), Some(Move::Event(ProcessEvent::Off))));
        match moves.first() {
            Some(Move::Rapid { to }) => assert_at(*to, ends[0].0, ends[0].1, 2.0),
            other => panic!("expected a rapid, got {other:?}"),
        }
        let plunge_moves: Vec<_> = plunges(face.toolpath());
        assert_eq!(plunge_moves.len(), 1);
        assert_at(plunge_moves[0], ends[0].0, ends[0].1, final_z);
        for plunge in &plunge_moves {
            let dist = dist_to_box(plunge.x(), plunge.y(), area);
            assert!(dist >= radius - 1e-9, "plunge is {dist} from the box");
        }

        let cut_ends: Vec<_> = moves
            .iter()
            .filter_map(|mv| match *mv {
                Move::Linear { to, kind: CutKind::Cut } => Some(to),
                _ => None,
            })
            .collect();
        assert_eq!(cut_ends.len(), ends.len() - 1);
        for (point, expected) in cut_ends.iter().zip(ends.iter().skip(1)) {
            assert_at(*point, expected.0, expected.1, final_z);
        }
        match moves.iter().rev().nth(1) {
            Some(Move::Linear { to, kind: CutKind::Retract }) => assert_at(*to, 35.0, 20.0, 2.0),
            other => panic!("expected a retract before process-off, got {other:?}"),
        }

        let at_floor: Vec<_> = cuts(face.toolpath()).into_iter().filter(|seg| (seg.z - final_z).abs() <= 1e-8).collect();
        assert_covered(area, &at_floor, radius, 0.5);

        let wider = FaceOp::new(10.0, 6.0).overrun(8.0).depth(1.0).step_down(1.0).generate(&area).expect("overrun");
        assert_at(plunges(wider.toolpath())[0], -8.0, 0.0, final_z);
        assert!(FaceOp::new(10.0, 10.0).depth(1.0).step_down(1.0).generate(&area).is_ok());
    }

    #[test]
    fn levels_follow_pass_depths() {
        let area = rectangle(0.0, 0.0, 30.0, 20.0);
        let stock = 1.0;
        let face = FaceOp::new(10.0, 6.0).stock_z(stock).retract_z(3.0).depth(5.0).step_down(2.0).generate(&area).expect("levels");
        let depths = pass_depths(5.0, 2.0).unwrap();
        assert_eq!(depths, vec![2.0, 4.0, 5.0]);
        assert_eq!(face.pass_count(), 5, "lines per level, not times the depth count");

        let plunge_z: Vec<_> = plunges(face.toolpath()).into_iter().map(|point| point.z()).collect();
        assert_eq!(plunge_z.len(), depths.len());
        for (z, depth) in plunge_z.iter().zip(&depths) {
            near(*z, stock - depth);
        }
        let final_z = stock - depths.last().copied().unwrap_or(0.0);
        let segs = cuts(face.toolpath());
        for depth in &depths {
            let z = stock - depth;
            assert!(segs.iter().any(|seg| (seg.z - z).abs() < 1e-8), "missing cuts at {z}");
        }
        for seg in &segs {
            assert!(depths.iter().any(|depth| (seg.z - (stock - depth)).abs() < 1e-8), "cut at {} is not a pass", seg.z);
            assert!(seg.z + 1e-9 >= final_z, "cut below the final level");
        }
        for mv in face.toolpath().moves() {
            if let Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } = *mv {
                assert!(to.z() + 1e-9 >= final_z, "move at {} is below the final level", to.z());
            }
        }
        let retracts: Vec<_> = face
            .toolpath()
            .moves()
            .iter()
            .filter_map(|mv| match *mv {
                Move::Linear { to, kind: CutKind::Retract } => Some(to.z()),
                _ => None,
            })
            .collect();
        assert_eq!(retracts.len(), depths.len());
        assert!(retracts.iter().all(|z| (*z - 3.0).abs() < 1e-12));

        let events: Vec<_> = face
            .toolpath()
            .moves()
            .iter()
            .filter_map(|mv| match mv {
                Move::Event(event) => Some(*event),
                _ => None,
            })
            .collect();
        assert_eq!(events, vec![ProcessEvent::On, ProcessEvent::Off]);
        assert!(matches!(face.toolpath().moves().last(), Some(Move::Event(ProcessEvent::Off))));
        for plunge in plunges(face.toolpath()) {
            let dist = dist_to_box(plunge.x(), plunge.y(), area);
            assert!(dist >= 5.0 - 1e-9, "plunge is {dist} from the box");
            assert_at(plunge, -5.0, 0.0, plunge.z());
        }
    }

    #[test]
    fn a_right_angle_swaps_the_axes() {
        // Cross width is now 30. ceil(30/6)+1 = 6 lines, 6 mm apart, on X = 0, 6, …, 30.
        // Passes run in Y from -5 to 25.
        let area = rectangle(0.0, 0.0, 30.0, 20.0);
        let face = FaceOp::new(10.0, 6.0).angle(FRAC_PI_2).depth(1.0).step_down(1.0).generate(&area).expect("angle");
        assert_eq!(face.pass_count(), 6);
        let along_x = FaceOp::new(10.0, 6.0).depth(1.0).step_down(1.0).generate(&area).expect("along x");
        assert_eq!(along_x.pass_count(), 5);

        let ends = [
            (0.0, -5.0),
            (0.0, 25.0),
            (6.0, 25.0),
            (6.0, -5.0),
            (12.0, -5.0),
            (12.0, 25.0),
            (18.0, 25.0),
            (18.0, -5.0),
            (24.0, -5.0),
            (24.0, 25.0),
            (30.0, 25.0),
            (30.0, -5.0),
        ];
        let plunge_moves = plunges(face.toolpath());
        assert_eq!(plunge_moves.len(), 1);
        assert_at(plunge_moves[0], 0.0, -5.0, -1.0);
        assert!(dist_to_box(plunge_moves[0].x(), plunge_moves[0].y(), area) >= 5.0 - 1e-9);

        let cut_ends: Vec<_> = face
            .toolpath()
            .moves()
            .iter()
            .filter_map(|mv| match *mv {
                Move::Linear { to, kind: CutKind::Cut } => Some(to),
                _ => None,
            })
            .collect();
        assert_eq!(cut_ends.len(), ends.len() - 1);
        for (point, expected) in cut_ends.iter().zip(ends.iter().skip(1)) {
            assert_at(*point, expected.0, expected.1, -1.0);
        }
        let at_floor: Vec<_> = cuts(face.toolpath()).into_iter().filter(|seg| (seg.z + 1.0).abs() <= 1e-8).collect();
        assert_covered(area, &at_floor, 5.0, 0.5);
    }

    #[test]
    fn zero_cross_extent_is_a_single_line() {
        let area = rectangle(0.0, 0.0, 15.0, 0.0);
        let face = FaceOp::new(8.0, 3.0).depth(2.0).step_down(2.0).generate(&area).expect("flat");
        assert_eq!(face.pass_count(), 1);
        let segs = cuts(face.toolpath());
        assert_eq!(segs.len(), 1);
        assert!(segs.iter().all(|seg| seg.y0.abs() < 1e-9 && seg.y1.abs() < 1e-9));
        near(segs[0].x0, -4.0);
        near(segs[0].x1, 19.0);

        let swapped = rectangle(0.0, 0.0, 0.0, 15.0);
        let face = FaceOp::new(8.0, 3.0).angle(FRAC_PI_2).depth(2.0).step_down(2.0).generate(&swapped).expect("line");
        assert_eq!(face.pass_count(), 1);
        let segs = cuts(face.toolpath());
        assert!(segs.iter().all(|seg| seg.x0.abs() < 1e-9 && seg.x1.abs() < 1e-9));
    }

    #[test]
    fn hostile_input_is_rejected() {
        let area = rectangle(0.0, 0.0, 30.0, 20.0);
        let bad = [
            FaceOp::new(10.0, 0.0).depth(1.0).step_down(1.0),
            FaceOp::new(10.0, -1.0).depth(1.0).step_down(1.0),
            FaceOp::new(10.0, 10.0 + 1e-9).depth(1.0).step_down(1.0),
            FaceOp::new(10.0, 6.0).depth(0.0).step_down(1.0),
            FaceOp::new(10.0, 6.0).depth(-2.0).step_down(1.0),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(0.0),
            FaceOp::new(f64::NAN, 6.0).depth(1.0).step_down(1.0),
            FaceOp::new(10.0, f64::NAN).depth(1.0).step_down(1.0),
            FaceOp::new(10.0, 6.0).depth(f64::NAN).step_down(1.0),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(f64::NAN),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(1.0).stock_z(f64::NAN),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(1.0).retract_z(f64::INFINITY),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(1.0).overrun(f64::NAN),
            FaceOp::new(10.0, 6.0).depth(1.0).step_down(1.0).overrun(-0.1),
            FaceOp::new(10.0, 6.0).angle(f64::NAN).depth(1.0).step_down(1.0),
            FaceOp::new(0.0, 1.0).depth(1.0).step_down(1.0),
            FaceOp::new(-5.0, 1.0).depth(1.0).step_down(1.0),
        ];
        for op in bad {
            let err = op.generate(&area).expect_err("hostile input");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        for angle in [0.4, PI, -FRAC_PI_2, FRAC_PI_2 * 3.0] {
            let err = FaceOp::new(10.0, 6.0).angle(angle).depth(1.0).step_down(1.0).generate(&area).expect_err("angle");
            assert!(matches!(err, PathError::NotSupported(_)), "{angle}: {err:?}");
        }

        let tall = rectangle(0.0, 0.0, 10.0, 100_000.0);
        let err = FaceOp::new(2.0, 1.0).depth(1.0).step_down(1.0).generate(&tall).expect_err("line cap");
        assert!(err.to_string().contains("100000"), "{err}");
    }

    fn coarse_step(span: f64) -> f64 {
        if span <= 1e-12 { 1.0 } else { span / 8.0 }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        #[test]
        fn random_rectangles_are_covered_at_the_final_level(
            origin_x in -30.0..30.0,
            origin_y in -30.0..30.0,
            width in 0.0..200.0,
            height in 0.0..200.0,
            diameter in 1.0..25.0,
            fraction in 0.1..=1.0,
            along_y in any::<bool>(),
        ) {
            let stepover = f64::min(diameter * fraction, diameter);
            let mut op = FaceOp::new(diameter, stepover).depth(1.0).step_down(1.0);
            if along_y {
                op = op.angle(FRAC_PI_2);
            }
            let area = rectangle(origin_x, origin_y, width, height);
            let face = op.generate(&area).unwrap_or_else(|err| panic!("generate failed: {err}"));
            let radius = diameter * 0.5;
            let final_z = -1.0;
            prop_assert!(face.pass_count() >= 1);
            prop_assert!(face.pass_count() <= MAX_LINES);

            let segs = cuts(face.toolpath());
            prop_assert!(!segs.is_empty());
            let mut long = 0;
            for seg in &segs {
                prop_assert!((seg.z - final_z).abs() < 1e-8, "cut at {} is not the final level {final_z}", seg.z);
                let dx = (seg.x0 - seg.x1).abs();
                let dy = (seg.y0 - seg.y1).abs();
                prop_assert!(dx <= 1e-7 || dy <= 1e-7, "cut is not axis-aligned");
                let link = if along_y { dy <= 1e-7 && dx > 1e-7 } else { dx <= 1e-7 && dy > 1e-7 };
                if link {
                    prop_assert!(dist_to_box(seg.x0, seg.y0, area) >= radius - 1e-6);
                    prop_assert!(dist_to_box(seg.x1, seg.y1, area) >= radius - 1e-6);
                }
                let along = if along_y { dy } else { dx };
                if along > 1e-7 {
                    long += 1;
                }
            }
            prop_assert_eq!(long, face.pass_count());

            let min = area.min();
            let max = area.max();
            for x in axis_samples(min.x(), max.x(), coarse_step(max.x() - min.x())) {
                for y in axis_samples(min.y(), max.y(), coarse_step(max.y() - min.y())) {
                    let best = segs.iter().fold(f64::INFINITY, |best, seg| best.min(dist_to_seg(x, y, seg)));
                    prop_assert!(best <= radius + 1e-6, "({x}, {y}) is {best} from a cutting line, radius {radius}");
                }
            }
            for mv in face.toolpath().moves() {
                if let Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } = *mv {
                    prop_assert!(to.z() + 1e-9 >= final_z, "move at {} is below the final level", to.z());
                }
                if let Move::Linear { to, kind: CutKind::Cut } = *mv {
                    prop_assert!((to.z() - final_z).abs() < 1e-8);
                }
                if let Move::Linear { to, kind: CutKind::Plunge } = *mv {
                    prop_assert!(dist_to_box(to.x(), to.y(), area) >= radius - 1e-6);
                    prop_assert!((to.z() - final_z).abs() < 1e-8);
                }
            }
        }
    }
}
