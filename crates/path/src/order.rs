//! Order features and link them with a retract, a rapid, and a plunge.
//!
//! A feature whose contour lies inside another feature's contour is cut first. That is the
//! "inside before outside" rule: holes before the profile that contains them. Features at the
//! same nesting depth are ordered by nearest neighbour from the current point, then shortened
//! with 2-opt. 2-opt stays inside one depth, so it cannot move a hole after its container.
//!
//! The tool starts at `(start.x, start.y, clearance_z)`. Each link retracts straight up to the
//! clearance height, rapids at that height, and plunges to the next feature. The feature's own
//! leading rapid is replaced by that link. After the last feature the tool retracts to clearance.
//! [`OrderedCuts::rapid_length`] is the sum of every rapid in the linked toolpath.

use campanion_geom::{BoundingBox, Containment, Contour, Point, Segment, Tolerance};

use crate::Result;
use crate::check::{self, finite};
use crate::toolpath::{CutKind, Move, Point3, Toolpath};

/// Largest set this module will order. Containment is quadratic in the feature count.
const MAX_FEATURES: usize = 256;

/// 2-opt stops after this many full passes even if a pass still finds a shorter tour.
const MAX_TWO_OPT_PASSES: usize = 64;

const MAX_MM: f64 = 1.0e6;

/// One feature: the contour that decides containment, and the toolpath that is cut.
#[derive(Clone, Debug, PartialEq)]
pub struct Feature {
    contour: Contour,
    toolpath: Toolpath,
}

impl Feature {
    pub fn new(contour: Contour, toolpath: Toolpath) -> Self {
        Self { contour, toolpath }
    }

    pub fn contour(&self) -> &Contour {
        &self.contour
    }

    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }
}

/// Features linked in [`Self::sequence`] order.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderedCuts {
    toolpath: Toolpath,
    rapid_length: f64,
    sequence: Vec<usize>,
}

impl OrderedCuts {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Millimetres of rapid motion in [`Self::toolpath`], from the start point onward.
    pub fn rapid_length(&self) -> f64 {
        self.rapid_length
    }

    /// Indices into the input slice. Inner features come before their containers.
    pub fn sequence(&self) -> &[usize] {
        &self.sequence
    }
}

/// Order `features` and link them.
///
/// `start` is the tool's XY before the first rapid. `clearance_z` is the absolute retract
/// height. It must be at least as high as every point of every feature.
pub fn order_and_link(features: &[Feature], start: Point, clearance_z: f64, tolerance: &Tolerance) -> Result<OrderedCuts> {
    let sequence = sequence_of(features, start, tolerance)?;
    link_features(features, &sequence, start, clearance_z, tolerance)
}

/// Link `features` in an explicit order. `sequence` must list each input index once.
///
/// The link is the same retract, rapid, and plunge as [`order_and_link`]. A caller uses this
/// to cut in a chosen order, or to measure that order's rapid length.
pub fn link_features(features: &[Feature], sequence: &[usize], start: Point, clearance_z: f64, tolerance: &Tolerance) -> Result<OrderedCuts> {
    if features.len() > MAX_FEATURES {
        return Err(check::invalid(format!("too many features: {} > {MAX_FEATURES}", features.len())));
    }
    let clearance = capped_abs(clearance_z, "clearance")?;
    let eps = tolerance.linear().max(1e-9);
    check_sequence(features.len(), sequence)?;
    let mut ends = Vec::with_capacity(features.len());
    for feature in features {
        if feature.contour.segments().len() > campanion_geom::MAX_SEGMENTS {
            return Err(check::invalid(format!("too many segments: {} > {}", feature.contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
        }
        if path_above(feature.toolpath(), clearance, eps) {
            return Err(check::invalid("clearance is below the toolpath"));
        }
        ends.push(motion_ends(feature.toolpath())?);
    }

    let mut moves = Vec::new();
    let mut cursor = Point3::new(start.x(), start.y(), clearance)?;
    let mut rapid_length = 0.0;
    for &index in sequence {
        let Some(feature) = features.get(index) else {
            return Err(check::invalid("feature index is out of range"));
        };
        let Some(end) = ends.get(index) else {
            return Err(check::invalid("feature index is out of range"));
        };
        retract_to(&mut moves, &mut cursor, clearance, eps)?;
        let above = Point3::new(end.entry.x(), end.entry.y(), clearance)?;
        emit_rapid(&mut moves, &mut cursor, above, &mut rapid_length, eps)?;
        if !cursor.almost_eq(end.entry, eps) {
            check::push(&mut moves, Move::Linear { to: end.entry, kind: CutKind::Plunge })?;
            cursor = end.entry;
        }
        replay(&mut moves, &mut cursor, &mut rapid_length, feature.toolpath(), eps)?;
    }
    if !sequence.is_empty() {
        retract_to(&mut moves, &mut cursor, clearance, eps)?;
    }
    if !rapid_length.is_finite() {
        return Err(check::invalid("rapid length is not finite"));
    }
    Ok(OrderedCuts { toolpath: Toolpath::new(moves)?, rapid_length, sequence: sequence.to_vec() })
}

struct Ends {
    entry: Point3,
    exit: Point3,
}

struct Shape {
    samples: Vec<Point>,
    bounds: BoundingBox,
}

fn sequence_of(features: &[Feature], start: Point, tolerance: &Tolerance) -> Result<Vec<usize>> {
    if features.len() > MAX_FEATURES {
        return Err(check::invalid(format!("too many features: {} > {MAX_FEATURES}", features.len())));
    }
    let n = features.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    let depth = nesting_depth(features, tolerance)?;
    let mut entry_xy = Vec::with_capacity(n);
    let mut exit_xy = Vec::with_capacity(n);
    for feature in features {
        let ends = motion_ends(feature.toolpath())?;
        entry_xy.push(Point::new(ends.entry.x(), ends.entry.y())?);
        exit_xy.push(Point::new(ends.exit.x(), ends.exit.y())?);
    }
    let max_depth = depth.iter().copied().fold(0usize, usize::max);
    let mut cursor = start;
    let mut sequence = Vec::with_capacity(n);
    for level in (0..=max_depth).rev() {
        let mut group = Vec::new();
        for (index, feature_depth) in depth.iter().copied().enumerate() {
            if feature_depth == level {
                group.push(index);
            }
        }
        if group.is_empty() {
            continue;
        }
        let mut ordered = nearest(&group, cursor, &entry_xy, &exit_xy)?;
        two_opt(&mut ordered, cursor, &entry_xy, &exit_xy);
        if let Some(&last) = ordered.last() {
            let Some(exit) = exit_xy.get(last) else {
                return Err(check::invalid("feature index is out of range"));
            };
            cursor = *exit;
        }
        sequence.extend(ordered);
    }
    Ok(sequence)
}

fn nesting_depth(features: &[Feature], tolerance: &Tolerance) -> Result<Vec<usize>> {
    let mut shapes = Vec::with_capacity(features.len());
    for feature in features {
        if feature.contour.segments().len() > campanion_geom::MAX_SEGMENTS {
            return Err(check::invalid(format!("too many segments: {} > {}", feature.contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
        }
        shapes.push(shape_of(feature.contour())?);
    }
    let mut depth = vec![0usize; features.len()];
    for i in 0..features.len() {
        for j in 0..features.len() {
            if i == j {
                continue;
            }
            let (Some(inner), Some(outer_shape)) = (shapes.get(i), shapes.get(j)) else {
                return Err(check::invalid("feature index is out of range"));
            };
            let Some(outer) = features.get(j) else {
                return Err(check::invalid("feature index is out of range"));
            };
            if contained(inner, outer.contour(), outer_shape.bounds, tolerance)? {
                let Some(slot) = depth.get_mut(i) else {
                    return Err(check::invalid("feature index is out of range"));
                };
                *slot += 1;
            }
        }
    }
    Ok(depth)
}

fn shape_of(contour: &Contour) -> Result<Shape> {
    let mut samples = Vec::new();
    let segs = contour.segments();
    for (index, seg) in segs.iter().copied().enumerate() {
        let last = index + 1 == segs.len();
        match seg {
            Segment::Line(line) => {
                samples.push(line.start());
                samples.push(line.start().midpoint(line.end()));
                if last {
                    samples.push(line.end());
                }
            }
            Segment::Arc(arc) => {
                let steps = 8;
                let end = if last { steps } else { steps - 1 };
                for k in 0..=end {
                    samples.push(arc.point_at(k as f64 / steps as f64));
                }
            }
        }
    }
    if samples.is_empty() {
        return Err(check::invalid("contour has no sample points"));
    }
    Ok(Shape { samples, bounds: contour.bounding_box()? })
}

/// `inner` lies in `outer` when every sample is inside or on the boundary and one is strictly inside.
fn contained(inner: &Shape, outer: &Contour, outer_bounds: BoundingBox, tolerance: &Tolerance) -> Result<bool> {
    if !outer.is_closed() {
        return Ok(false);
    }
    let eps = tolerance.linear();
    if !bounds_inside(inner.bounds, outer_bounds, eps) {
        return Ok(false);
    }
    let mut strictly = false;
    for point in &inner.samples {
        match outer.containment(*point, tolerance)? {
            Containment::Outside => return Ok(false),
            Containment::Inside => strictly = true,
            Containment::Boundary => {}
        }
    }
    Ok(strictly)
}

fn bounds_inside(inner: BoundingBox, outer: BoundingBox, eps: f64) -> bool {
    inner.min().x() >= outer.min().x() - eps
        && inner.min().y() >= outer.min().y() - eps
        && inner.max().x() <= outer.max().x() + eps
        && inner.max().y() <= outer.max().y() + eps
}

fn nearest(group: &[usize], mut from: Point, entries: &[Point], exits: &[Point]) -> Result<Vec<usize>> {
    let mut remaining = group.to_vec();
    let mut ordered = Vec::with_capacity(remaining.len());
    while !remaining.is_empty() {
        let mut best: Option<(usize, usize, f64)> = None;
        for (pos, index) in remaining.iter().copied().enumerate() {
            let Some(entry) = entries.get(index) else {
                return Err(check::invalid("feature index is out of range"));
            };
            let dist = from.distance(*entry);
            if !dist.is_finite() {
                return Err(check::invalid("feature distance is not finite"));
            }
            let replace = match best {
                None => true,
                Some((_, best_index, best_dist)) => dist + 1e-12 < best_dist || ((dist - best_dist).abs() <= 1e-12 && index < best_index),
            };
            if replace {
                best = Some((pos, index, dist));
            }
        }
        let Some((best_pos, index, _)) = best else {
            return Err(check::invalid("feature group is empty"));
        };
        remaining.remove(best_pos);
        ordered.push(index);
        let Some(exit) = exits.get(index) else {
            return Err(check::invalid("feature index is out of range"));
        };
        from = *exit;
    }
    Ok(ordered)
}

/// Open-path 2-opt. The cost is the XY rapid from each feature's exit to the next entry.
fn two_opt(order: &mut [usize], origin: Point, entries: &[Point], exits: &[Point]) {
    if order.len() < 2 {
        return;
    }
    let mut improved = true;
    let mut passes = 0;
    while improved && passes < MAX_TWO_OPT_PASSES {
        improved = false;
        passes += 1;
        let n = order.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let before = tour_length(order, origin, entries, exits);
                order[i..=j].reverse();
                let after = tour_length(order, origin, entries, exits);
                if after + 1e-9 < before {
                    improved = true;
                } else {
                    order[i..=j].reverse();
                }
            }
        }
    }
}

fn tour_length(order: &[usize], origin: Point, entries: &[Point], exits: &[Point]) -> f64 {
    let mut total = 0.0;
    let mut from = origin;
    for &index in order {
        let (Some(entry), Some(exit)) = (entries.get(index), exits.get(index)) else {
            return f64::INFINITY;
        };
        total += from.distance(*entry);
        from = *exit;
    }
    total
}

fn path_above(path: &Toolpath, clearance: f64, eps: f64) -> bool {
    path.moves().iter().filter_map(|mv| motion_point(*mv)).any(|point| point.z() > clearance + eps)
}

fn motion_ends(path: &Toolpath) -> Result<Ends> {
    let mut first: Option<Point3> = None;
    let mut last: Option<Point3> = None;
    let mut first_was_rapid = false;
    for mv in path.moves() {
        let Some(point) = motion_point(*mv) else {
            continue;
        };
        if first.is_none() {
            first_was_rapid = matches!(mv, Move::Rapid { .. });
            first = Some(point);
        }
        last = Some(point);
    }
    if first.is_none() {
        return Err(check::invalid("toolpath has no motion"));
    }
    if !first_was_rapid {
        return Err(check::invalid("toolpath needs a rapid to its start"));
    }
    match (first, last) {
        (Some(entry), Some(exit)) => Ok(Ends { entry, exit }),
        _ => Err(check::invalid("toolpath has no motion")),
    }
}

fn motion_point(mv: Move) -> Option<Point3> {
    match mv {
        Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
        Move::Event(_) | Move::Dwell { .. } => None,
    }
}

fn check_sequence(count: usize, sequence: &[usize]) -> Result<()> {
    if sequence.len() != count {
        return Err(check::invalid("sequence does not cover every feature"));
    }
    let mut seen = vec![false; count];
    for &index in sequence {
        let Some(slot) = seen.get_mut(index) else {
            return Err(check::invalid("feature index is out of range"));
        };
        if *slot {
            return Err(check::invalid("feature repeated in the sequence"));
        }
        *slot = true;
    }
    Ok(())
}

fn retract_to(moves: &mut Vec<Move>, cursor: &mut Point3, clearance: f64, eps: f64) -> Result<()> {
    if cursor.z() + eps < clearance {
        let up = Point3::new(cursor.x(), cursor.y(), clearance)?;
        check::push(moves, Move::Linear { to: up, kind: CutKind::Retract })?;
        *cursor = up;
    }
    Ok(())
}

fn emit_rapid(moves: &mut Vec<Move>, cursor: &mut Point3, to: Point3, rapid_length: &mut f64, eps: f64) -> Result<()> {
    if cursor.almost_eq(to, eps) {
        *cursor = to;
        return Ok(());
    }
    let distance = cursor.distance(to);
    if !distance.is_finite() {
        return Err(check::invalid("rapid length is not finite"));
    }
    *rapid_length += distance;
    check::push(moves, Move::Rapid { to })?;
    *cursor = to;
    Ok(())
}

fn replay(moves: &mut Vec<Move>, cursor: &mut Point3, rapid_length: &mut f64, path: &Toolpath, eps: f64) -> Result<()> {
    let mut skipped = false;
    for mv in path.moves() {
        if !skipped && matches!(mv, Move::Rapid { .. }) {
            skipped = true;
            continue;
        }
        match *mv {
            Move::Rapid { to } => emit_rapid(moves, cursor, to, rapid_length, eps)?,
            Move::Linear { to, kind } => {
                check::push(moves, Move::Linear { to, kind })?;
                *cursor = to;
            }
            Move::Arc { to, centre, ccw, kind } => {
                check::push(moves, Move::Arc { to, centre, ccw, kind })?;
                *cursor = to;
            }
            Move::Event(event) => check::push(moves, Move::Event(event))?,
            Move::Dwell { seconds } => check::push(moves, Move::Dwell { seconds })?,
        }
    }
    Ok(())
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
    use crate::toolpath::{CutKind, ProcessEvent};
    use campanion_geom::{Line, Point, Segment};

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-6, "{a} != {b}");
    }

    fn square(x: f64, y: f64, w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![edge(p(x, y), p(x + w, y)), edge(p(x + w, y), p(x + w, y + h)), edge(p(x + w, y + h), p(x, y + h)), edge(p(x, y + h), p(x, y))],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    fn feature(contour: &Contour, z: f64) -> Feature {
        let start = contour.segments().first().unwrap().start();
        let mut moves = vec![Move::Rapid { to: Point3::new(start.x(), start.y(), z).unwrap() }, Move::Event(ProcessEvent::On)];
        for seg in contour.segments() {
            let end = seg.end();
            moves.push(Move::Linear { to: Point3::new(end.x(), end.y(), z).unwrap(), kind: CutKind::Cut });
        }
        moves.push(Move::Event(ProcessEvent::Off));
        Feature::new(contour.clone(), Toolpath::new(moves).unwrap())
    }

    fn measured_rapids(path: &Toolpath, start: Point3) -> f64 {
        let mut cursor = start;
        let mut sum = 0.0;
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } => {
                    sum += cursor.distance(to);
                    cursor = to;
                }
                Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = to,
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        sum
    }

    #[test]
    fn holes_are_ordered_before_the_outer_profile() {
        // The tool starts on the outer corner, so nearest-neighbour alone would cut the
        // outer first. Containment has to win.
        let outer = square(0.0, 0.0, 100.0, 100.0);
        let hole_a = square(20.0, 20.0, 10.0, 10.0).reversed();
        let hole_b = square(60.0, 60.0, 10.0, 10.0).reversed();
        let features = [feature(&outer, -2.0), feature(&hole_a, -2.0), feature(&hole_b, -2.0)];
        let start = Point::new(0.0, 0.0).unwrap();
        let clearance = 8.0;
        let ordered = order_and_link(&features, start, clearance, &Tolerance::default()).unwrap();

        let seq = ordered.sequence();
        let outer_at = seq.iter().position(|index| *index == 0).unwrap();
        let hole_a_at = seq.iter().position(|index| *index == 1).unwrap();
        let hole_b_at = seq.iter().position(|index| *index == 2).unwrap();
        assert!(hole_a_at < outer_at, "hole A after the outer: {seq:?}");
        assert!(hole_b_at < outer_at, "hole B after the outer: {seq:?}");
        assert!(hole_a_at < hole_b_at, "nearer hole should be first: {seq:?}");
        assert_eq!(seq, &[1, 2, 0]);

        let mut saw_retract = false;
        let mut saw_plunge = false;
        let mut saw_rapid = false;
        for mv in ordered.toolpath().moves() {
            match *mv {
                Move::Rapid { to } => {
                    saw_rapid = true;
                    near(to.z(), clearance);
                }
                Move::Linear { kind: CutKind::Retract, to } => {
                    saw_retract = true;
                    near(to.z(), clearance);
                }
                Move::Linear { kind: CutKind::Plunge, to } => {
                    saw_plunge = true;
                    near(to.z(), -2.0);
                }
                _ => {}
            }
        }
        assert!(saw_retract && saw_plunge && saw_rapid);

        let expected = 120.0 * std::f64::consts::SQRT_2;
        near(ordered.rapid_length(), expected);
        let origin = Point3::new(0.0, 0.0, clearance).unwrap();
        near(measured_rapids(ordered.toolpath(), origin), ordered.rapid_length());
    }

    #[test]
    fn nearest_neighbour_and_two_opt_is_no_longer_than_a_naive_grid() {
        // 4x4 holes, stored in an order that jumps across the grid.
        let scramble = [0usize, 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6, 9, 7, 8];
        let mut features = Vec::new();
        for grid in scramble {
            let x = (grid % 4) as f64 * 10.0;
            let y = (grid / 4) as f64 * 10.0;
            features.push(feature(&square(x, y, 1.0, 1.0), -1.0));
        }
        let start = Point::new(0.0, 0.0).unwrap();
        let clearance = 5.0;
        let tol = Tolerance::default();
        let ordered = order_and_link(&features, start, clearance, &tol).unwrap();
        let naive_seq: Vec<usize> = (0..features.len()).collect();
        let naive = link_features(&features, &naive_seq, start, clearance, &tol).unwrap();

        assert!(ordered.rapid_length() <= naive.rapid_length() + 1e-6, "optimised {} > naive {}", ordered.rapid_length(), naive.rapid_length());
        assert!(
            ordered.rapid_length() < naive.rapid_length() - 1.0,
            "optimised {} is not shorter than naive {}",
            ordered.rapid_length(),
            naive.rapid_length()
        );
        // A Hamiltonian path on the unit grid of spacing 10 is 15 edges.
        assert!(ordered.rapid_length() <= 150.0 + 1e-6, "{}", ordered.rapid_length());

        let mut seen = [false; 16];
        for &index in ordered.sequence() {
            assert!(!seen[index]);
            seen[index] = true;
        }
        assert!(seen.iter().all(|flag| *flag));
        let origin = Point3::new(start.x(), start.y(), clearance).unwrap();
        near(measured_rapids(ordered.toolpath(), origin), ordered.rapid_length());
    }

    #[test]
    fn hostile_order_input_is_an_error() {
        let contour = square(0.0, 0.0, 4.0, 4.0);
        let feat = feature(&contour, 0.0);
        let start = Point::new(0.0, 0.0).unwrap();
        let tol = Tolerance::default();

        assert!(order_and_link(std::slice::from_ref(&feat), start, f64::NAN, &tol).is_err());
        assert!(order_and_link(std::slice::from_ref(&feat), start, f64::INFINITY, &tol).is_err());
        assert!(order_and_link(std::slice::from_ref(&feat), start, f64::NEG_INFINITY, &tol).is_err());
        let below = order_and_link(std::slice::from_ref(&feat), start, -1.0, &tol).unwrap_err();
        assert!(below.to_string().contains("clearance"), "{below}");

        let empty = Feature::new(contour.clone(), Toolpath::new(vec![]).unwrap());
        let err = order_and_link(&[empty], start, 1.0, &tol).unwrap_err();
        assert!(err.to_string().contains("no motion"), "{err}");

        let linear =
            Feature::new(contour.clone(), Toolpath::new(vec![Move::Linear { to: Point3::new(1.0, 0.0, 0.0).unwrap(), kind: CutKind::Cut }]).unwrap());
        let err = order_and_link(&[linear], start, 1.0, &tol).unwrap_err();
        assert!(err.to_string().contains("rapid"), "{err}");

        let err = link_features(&[feat.clone(), feat.clone()], &[0, 0], start, 1.0, &tol).unwrap_err();
        assert!(err.to_string().contains("repeated"), "{err}");
        let err = link_features(std::slice::from_ref(&feat), &[3], start, 1.0, &tol).unwrap_err();
        assert!(err.to_string().contains("out of range") || err.to_string().contains("does not cover"), "{err}");

        let mut many = Vec::new();
        for _ in 0..=MAX_FEATURES {
            many.push(feat.clone());
        }
        let err = order_and_link(&many, start, 1.0, &tol).unwrap_err();
        assert!(err.to_string().contains("too many features"), "{err}");

        let none = order_and_link(&[], start, 1.0, &tol).unwrap();
        assert!(none.toolpath().is_empty());
        near(none.rapid_length(), 0.0);
        assert!(none.sequence().is_empty());
    }
}
