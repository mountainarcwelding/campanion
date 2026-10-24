//! Drill positions collected from circles and explicit points, and the motion that cuts them.
//!
//! A hit is a position. [`DrillCycle`] carries peck depth, retract and dwell.
//! [`Drilling::toolpath`] is hole positions only and must not be posted. [`Drilling::expand`]
//! writes the motion: a rapid at clearance, a rapid at the R plane, a feed into the hole, and a
//! rapid back out. Between pecks the tool retracts all the way to the R plane and feeds back in.
//! [`DrillCycle::retract_distance`] is not used for that pull-back (full retract, G83 style).
//! This module does not emit machine codes.

use campanion_geom::{Contour, Point, Segment, Tolerance};

use crate::Result;
use crate::check::{self, finite, non_negative, positive};
use crate::pass_depths;
use crate::toolpath::{CutKind, Move, Point3, Toolpath};

const TAU: f64 = std::f64::consts::TAU;

/// Cycle parameters. `None` means the field is unset.
///
/// Lengths are millimetres. `dwell` is seconds. Nothing here is a machine code.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DrillCycle {
    peck_depth: Option<f64>,
    retract: Option<f64>,
    dwell: Option<f64>,
}

impl DrillCycle {
    pub fn new() -> Self {
        Self { peck_depth: None, retract: None, dwell: None }
    }

    /// Depth of one peck, in millimetres. Validated when the operation runs.
    pub fn peck_depth(mut self, depth: f64) -> Self {
        self.peck_depth = Some(depth);
        self
    }

    /// Distance to pull back between pecks, in millimetres.
    pub fn retract(mut self, distance: f64) -> Self {
        self.retract = Some(distance);
        self
    }

    /// Time to pause at the bottom of the hole, in seconds.
    pub fn dwell(mut self, seconds: f64) -> Self {
        self.dwell = Some(seconds);
        self
    }

    pub fn peck(&self) -> Option<f64> {
        self.peck_depth
    }

    pub fn retract_distance(&self) -> Option<f64> {
        self.retract
    }

    pub fn dwell_seconds(&self) -> Option<f64> {
        self.dwell
    }

    fn validate(self) -> Result<()> {
        if let Some(depth) = self.peck_depth {
            positive(depth, "peck depth")?;
        }
        if let Some(distance) = self.retract {
            non_negative(distance, "retract")?;
        }
        if let Some(seconds) = self.dwell {
            non_negative(seconds, "dwell")?;
        }
        Ok(())
    }
}

impl Default for DrillCycle {
    fn default() -> Self {
        Self::new()
    }
}

/// One hole.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DrillHit {
    position: Point3,
    /// Diameter of the source circle. `None` for an explicit point.
    diameter: Option<f64>,
}

impl DrillHit {
    pub fn position(self) -> Point3 {
        self.position
    }

    pub fn diameter(self) -> Option<f64> {
        self.diameter
    }
}

/// Where to drill, and the cycle marker that applies to every hit.
#[derive(Clone, Debug, PartialEq)]
pub struct Drilling {
    toolpath: Toolpath,
    hits: Vec<DrillHit>,
    cycle: DrillCycle,
}

impl Drilling {
    /// Hole positions only. One rapid per hit, at the hole bottom.
    ///
    /// This must not be posted. Use [`Self::expand`] for motion.
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Safe motion for every hole, in order.
    ///
    /// `z` on a hit is the hole bottom. For each hole the moves are:
    ///
    /// 1. Rapid to `(x, y, clearance)`, so XY travel stays at clearance.
    /// 2. Rapid to `(x, y, r_plane)`. This is as low as a rapid goes.
    /// 3. Feed to each peck depth. With no peck depth, that is one feed to the bottom.
    ///    Otherwise the depths are [`pass_depths`] of `top_z - bottom`, mapped with
    ///    `top_z - depth`, and the last depth is exactly the bottom. After every peck except
    ///    the last, rapid back to the R plane. The next peck feeds in from there.
    /// 4. Dwell, when [`DrillCycle::dwell_seconds`] is `Some(seconds)` and `seconds > 0`.
    /// 5. Rapid straight out to `(x, y, clearance)` along the hole axis.
    ///
    /// [`DrillCycle::retract_distance`] is not used. The pull-back between pecks is a full
    /// retract to the R plane, G83 style, not a short chip-break distance. Nothing rapids
    /// below the R plane.
    ///
    /// `feed_kind` is the [`CutKind`] of every feed into the hole.
    pub fn expand(&self, top_z: f64, r_plane: f64, clearance: f64, feed_kind: CutKind) -> Result<Toolpath> {
        if !r_plane.is_finite() || !clearance.is_finite() {
            return Err(check::invalid("R plane and clearance must be finite"));
        }
        finite(top_z, "top z")?;
        if clearance < r_plane {
            return Err(check::invalid("clearance must be at or above the R plane"));
        }
        if r_plane < top_z {
            return Err(check::invalid("R plane must be above the hole bottom"));
        }
        let mut moves = Vec::new();
        for hit in &self.hits {
            let x = hit.position.x();
            let y = hit.position.y();
            let bottom = hit.position.z();
            if r_plane <= bottom {
                return Err(check::invalid("R plane must be above the hole bottom"));
            }
            let levels = hole_levels(self.cycle.peck(), top_z, bottom)?;
            let last = levels.len().saturating_sub(1);
            check::push(&mut moves, Move::Rapid { to: Point3::new(x, y, clearance)? })?;
            check::push(&mut moves, Move::Rapid { to: Point3::new(x, y, r_plane)? })?;
            for (index, level) in levels.iter().copied().enumerate() {
                check::push(&mut moves, Move::Linear { to: Point3::new(x, y, level)?, kind: feed_kind })?;
                if index != last {
                    check::push(&mut moves, Move::Rapid { to: Point3::new(x, y, r_plane)? })?;
                }
            }
            if let Some(seconds) = self.cycle.dwell_seconds()
                && seconds > 0.0
            {
                check::push(&mut moves, Move::Dwell { seconds })?;
            }
            check::push(&mut moves, Move::Rapid { to: Point3::new(x, y, clearance)? })?;
        }
        Toolpath::new(moves)
    }

    pub fn hits(&self) -> &[DrillHit] {
        &self.hits
    }

    /// Cycle marker. Peck depth and the other fields stay data.
    pub fn cycle(&self) -> DrillCycle {
        self.cycle
    }
}

/// Collect drill positions.
///
/// Circles contribute their centres. A contour that is not a full circle is an error.
/// Explicit points have no diameter, so a diameter filter does not remove them.
#[derive(Clone, Debug, PartialEq)]
pub struct DrillOp {
    cycle: DrillCycle,
    min_diameter: Option<f64>,
    max_diameter: Option<f64>,
    z: f64,
    tolerance: Tolerance,
}

impl DrillOp {
    pub fn new(cycle: DrillCycle) -> Self {
        Self { cycle, min_diameter: None, max_diameter: None, z: 0.0, tolerance: Tolerance::default() }
    }

    /// Keep circles whose diameter is at least this value, in millimetres.
    pub fn min_diameter(mut self, diameter: f64) -> Self {
        self.min_diameter = Some(diameter);
        self
    }

    /// Keep circles whose diameter is at most this value, in millimetres.
    pub fn max_diameter(mut self, diameter: f64) -> Self {
        self.max_diameter = Some(diameter);
        self
    }

    pub fn z(mut self, z: f64) -> Self {
        self.z = z;
        self
    }

    pub fn tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    pub fn cycle(&self) -> DrillCycle {
        self.cycle
    }

    /// `circles` are closed contours. `points` are extra hole positions.
    pub fn generate(&self, circles: &[Contour], points: &[Point]) -> Result<Drilling> {
        self.cycle.validate()?;
        let z = finite(self.z, "z")?;
        let min_diameter = match self.min_diameter {
            Some(value) => Some(non_negative(value, "minimum diameter")?),
            None => None,
        };
        let max_diameter = match self.max_diameter {
            Some(value) => Some(non_negative(value, "maximum diameter")?),
            None => None,
        };
        if let (Some(min_diameter), Some(max_diameter)) = (min_diameter, max_diameter)
            && min_diameter > max_diameter
        {
            return Err(check::invalid("minimum diameter is greater than maximum diameter"));
        }
        if circles.len().saturating_add(points.len()) > crate::toolpath::MAX_MOVES {
            return Err(check::invalid("too many drill points"));
        }

        let mut hits = Vec::new();
        for contour in circles {
            if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
                return Err(check::invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
            }
            let (center, radius) = circle_of(contour, &self.tolerance)?;
            let diameter = radius * 2.0;
            if !diameter.is_finite() {
                return Err(check::invalid("circle diameter is not finite"));
            }
            if !passes_filter(diameter, min_diameter, max_diameter) {
                continue;
            }
            hits.push(DrillHit { position: Point3::new(center.x(), center.y(), z)?, diameter: Some(diameter) });
        }
        for point in points {
            hits.push(DrillHit { position: Point3::new(point.x(), point.y(), z)?, diameter: None });
        }

        let mut moves = Vec::with_capacity(hits.len());
        for hit in &hits {
            check::push(&mut moves, Move::Rapid { to: hit.position })?;
        }
        Ok(Drilling { toolpath: Toolpath::new(moves)?, hits, cycle: self.cycle })
    }
}

/// Peck Z levels for one hole. The last entry is exactly `bottom`.
fn hole_levels(peck: Option<f64>, top_z: f64, bottom: f64) -> Result<Vec<f64>> {
    let Some(peck) = peck else {
        return Ok(vec![bottom]);
    };
    let mut levels: Vec<f64> = pass_depths(top_z - bottom, peck)?.into_iter().map(|depth| top_z - depth).collect();
    match levels.last_mut() {
        Some(last) => *last = bottom,
        None => levels.push(bottom),
    }
    Ok(levels)
}

fn passes_filter(diameter: f64, min_diameter: Option<f64>, max_diameter: Option<f64>) -> bool {
    if let Some(min_diameter) = min_diameter
        && diameter + 1e-9 < min_diameter
    {
        return false;
    }
    if let Some(max_diameter) = max_diameter
        && diameter - 1e-9 > max_diameter
    {
        return false;
    }
    true
}

/// Centre and radius when every edge is an arc of one circle that completes one turn.
fn circle_of(contour: &Contour, tolerance: &Tolerance) -> Result<(Point, f64)> {
    if !contour.is_closed() {
        return Err(check::invalid("drill contour must be closed"));
    }
    let segments = contour.segments();
    if segments.is_empty() {
        return Err(check::invalid("contour is empty"));
    }
    let mut center: Option<Point> = None;
    let mut radius: Option<f64> = None;
    let mut sweep = 0.0;
    let linear = tolerance.linear().max(1e-6);
    for seg in segments {
        let Segment::Arc(arc) = seg else {
            return Err(check::invalid("contour is not a circle"));
        };
        if let Some(found) = center {
            if !found.almost_eq(arc.center(), linear) {
                return Err(check::invalid("contour is not a circle"));
            }
        } else {
            center = Some(arc.center());
        }
        if let Some(found) = radius {
            let scale = found.max(1.0);
            if (found - arc.radius()).abs() > linear.max(1e-6 * scale) {
                return Err(check::invalid("contour is not a circle"));
            }
        } else {
            radius = Some(arc.radius());
        }
        sweep += arc.sweep();
    }
    let (Some(center), Some(radius)) = (center, radius) else {
        return Err(check::invalid("contour is not a circle"));
    };
    let slack = tolerance.angular().max(1e-4).max(1e-3);
    if (sweep.abs() - TAU).abs() > slack {
        return Err(check::invalid("contour is not a full circle"));
    }
    Ok((center, radius))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PathError;
    use campanion_geom::{Line, Polyline, Vertex};

    fn circle(cx: f64, cy: f64, r: f64) -> Contour {
        let tol = Tolerance::default();
        Polyline::new(
            vec![Vertex::new(Point::new(cx + r, cy).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(cx - r, cy).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap()
        .to_contour(&tol)
        .unwrap()
    }

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    #[test]
    fn three_circles_keep_their_centres_in_order() {
        let circles = [circle(0.0, 0.0, 3.0), circle(10.0, 0.0, 5.0), circle(25.0, 7.0, 2.0)];
        let cycle = DrillCycle::new().peck_depth(1.5).retract(0.4).dwell(0.2);
        let drilling = DrillOp::new(cycle).z(-8.0).generate(&circles, &[]).unwrap();
        assert_eq!(drilling.hits().len(), 3);
        let expected = [(0.0, 0.0, 6.0), (10.0, 0.0, 10.0), (25.0, 7.0, 4.0)];
        for (hit, (x, y, diameter)) in drilling.hits().iter().zip(expected) {
            near(hit.position().x(), x);
            near(hit.position().y(), y);
            near(hit.position().z(), -8.0);
            near(hit.diameter().unwrap(), diameter);
        }
        assert_eq!(drilling.cycle().peck(), Some(1.5));
        assert_eq!(drilling.cycle().retract_distance(), Some(0.4));
        assert_eq!(drilling.cycle().dwell_seconds(), Some(0.2));
        assert_eq!(drilling.toolpath().moves().len(), 3);
        assert!(drilling.toolpath().moves().iter().all(|mv| matches!(mv, Move::Rapid { .. })));
        assert!(drilling.toolpath().cutting_length().unwrap().abs() < 1e-12);
    }

    #[test]
    fn diameter_filter_drops_circles_and_keeps_explicit_points() {
        // Diameters 6, 10 and 4. The window [7, 12] keeps only the middle circle.
        let circles = [circle(0.0, 0.0, 3.0), circle(10.0, 0.0, 5.0), circle(25.0, 7.0, 2.0)];
        let extra = [Point::new(1.0, 2.0).unwrap()];
        let drilling = DrillOp::new(DrillCycle::new()).min_diameter(7.0).max_diameter(12.0).generate(&circles, &extra).unwrap();
        assert_eq!(drilling.hits().len(), 2);
        near(drilling.hits()[0].position().x(), 10.0);
        near(drilling.hits()[0].diameter().unwrap(), 10.0);
        near(drilling.hits()[1].position().x(), 1.0);
        near(drilling.hits()[1].position().y(), 2.0);
        assert!(drilling.hits()[1].diameter().is_none());
    }

    #[test]
    fn hostile_input_is_an_error() {
        let circles = [circle(0.0, 0.0, 3.0)];
        let square = {
            let p = |x, y| Point::new(x, y).unwrap();
            let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
            Contour::try_new(
                vec![edge(p(0.0, 0.0), p(4.0, 0.0)), edge(p(4.0, 0.0), p(4.0, 4.0)), edge(p(4.0, 4.0), p(0.0, 4.0)), edge(p(0.0, 4.0), p(0.0, 0.0))],
                true,
                &Tolerance::default(),
            )
            .unwrap()
        };
        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let arc = Contour::try_new(
            vec![Segment::Arc(campanion_geom::Arc::from_bulge(Point::new(3.0, 0.0).unwrap(), Point::new(-3.0, 0.0).unwrap(), 1.0).unwrap())],
            true,
            &Tolerance::default(),
        );
        // A single semicircle is closed only if its ends meet, which they do not.
        assert!(arc.is_err() || DrillOp::new(DrillCycle::new()).generate(std::slice::from_ref(arc.as_ref().unwrap()), &[]).is_err());

        let cases = [
            DrillOp::new(DrillCycle::new().peck_depth(f64::NAN)),
            DrillOp::new(DrillCycle::new().peck_depth(0.0)),
            DrillOp::new(DrillCycle::new().peck_depth(-1.0)),
            DrillOp::new(DrillCycle::new().peck_depth(f64::INFINITY)),
            DrillOp::new(DrillCycle::new().retract(f64::NAN)),
            DrillOp::new(DrillCycle::new().retract(-0.2)),
            DrillOp::new(DrillCycle::new().dwell(f64::NEG_INFINITY)),
            DrillOp::new(DrillCycle::new()).z(f64::NAN),
            DrillOp::new(DrillCycle::new()).min_diameter(-1.0),
            DrillOp::new(DrillCycle::new()).max_diameter(f64::NAN),
            DrillOp::new(DrillCycle::new()).min_diameter(8.0).max_diameter(2.0),
        ];
        for op in cases {
            let err = op.generate(&circles, &[]).expect_err("hostile drill input");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        let err = DrillOp::new(DrillCycle::new()).generate(std::slice::from_ref(&square), &[]).unwrap_err();
        assert!(err.to_string().contains("circle"), "{err}");
        let err = DrillOp::new(DrillCycle::new()).generate(std::slice::from_ref(&open), &[]).unwrap_err();
        assert!(err.to_string().contains("closed"), "{err}");

        let mut segments = Vec::new();
        for i in 0..campanion_geom::MAX_SEGMENTS + 1 {
            let x0 = i as f64;
            segments.push(Segment::Line(Line::new(Point::new(x0, 0.0).unwrap(), Point::new(x0 + 1.0, 0.0).unwrap()).unwrap()));
        }
        let long = Contour::try_new(segments, false, &Tolerance::default()).unwrap();
        let err = DrillOp::new(DrillCycle::new()).generate(std::slice::from_ref(&long), &[]).unwrap_err();
        assert!(err.to_string().contains("too many segments") || err.to_string().contains("closed"), "{err}");
    }

    fn pt(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z).unwrap()
    }

    fn one_hole(cycle: DrillCycle, z: f64) -> Drilling {
        DrillOp::new(cycle).z(z).generate(&[], &[Point::new(4.0, 7.0).unwrap()]).unwrap()
    }

    #[test]
    fn expand_without_peck_lists_the_moves_for_one_hole() {
        let path = one_hole(DrillCycle::new(), -3.0).expand(0.0, 2.0, 5.0, CutKind::Plunge).unwrap();
        assert_eq!(
            path.moves(),
            &[
                Move::Rapid { to: pt(4.0, 7.0, 5.0) },
                Move::Rapid { to: pt(4.0, 7.0, 2.0) },
                Move::Linear { to: pt(4.0, 7.0, -3.0), kind: CutKind::Plunge },
                Move::Rapid { to: pt(4.0, 7.0, 5.0) },
            ]
        );
    }

    #[test]
    fn expand_peck_feeds_each_level_and_reenters_from_the_r_plane() {
        // Depth 3 mm, peck 1 mm, stock at 0: levels -1, -2, -3. A short retract distance is ignored.
        let cycle = DrillCycle::new().peck_depth(1.0).retract(0.2);
        let path = one_hole(cycle, -3.0).expand(0.0, 2.0, 5.0, CutKind::Plunge).unwrap();
        assert_eq!(
            path.moves(),
            &[
                Move::Rapid { to: pt(4.0, 7.0, 5.0) },
                Move::Rapid { to: pt(4.0, 7.0, 2.0) },
                Move::Linear { to: pt(4.0, 7.0, -1.0), kind: CutKind::Plunge },
                Move::Rapid { to: pt(4.0, 7.0, 2.0) },
                Move::Linear { to: pt(4.0, 7.0, -2.0), kind: CutKind::Plunge },
                Move::Rapid { to: pt(4.0, 7.0, 2.0) },
                Move::Linear { to: pt(4.0, 7.0, -3.0), kind: CutKind::Plunge },
                Move::Rapid { to: pt(4.0, 7.0, 5.0) },
            ]
        );
    }

    #[test]
    fn expand_dwells_after_the_last_level() {
        let cycle = DrillCycle::new().peck_depth(1.0).dwell(0.5);
        let path = one_hole(cycle, -3.0).expand(0.0, 2.0, 5.0, CutKind::Plunge).unwrap();
        let dwell_at = path.moves().iter().position(|mv| matches!(mv, Move::Dwell { seconds } if (*seconds - 0.5).abs() < 1e-12)).expect("dwell");
        let last_feed =
            path.moves().iter().rposition(|mv| matches!(mv, Move::Linear { to, .. } if (to.z() + 3.0).abs() < 1e-12)).expect("final feed");
        assert_eq!(dwell_at, last_feed + 1);
        assert!(matches!(path.moves().get(dwell_at + 1), Some(Move::Rapid { to }) if (to.z() - 5.0).abs() < 1e-12));
        assert_eq!(path.moves().iter().filter(|mv| matches!(mv, Move::Dwell { .. })).count(), 1);

        let silent = one_hole(DrillCycle::new().dwell(0.0), -3.0).expand(0.0, 2.0, 5.0, CutKind::Plunge).unwrap();
        assert!(silent.moves().iter().all(|mv| !matches!(mv, Move::Dwell { .. })));
    }

    #[test]
    fn expand_rapids_stay_at_or_above_the_r_plane() {
        let cycle = DrillCycle::new().peck_depth(1.0).dwell(0.5);
        let path = one_hole(cycle, -3.0).expand(0.0, 2.0, 5.0, CutKind::Plunge).unwrap();
        let mut exceptions = Vec::new();
        let mut fed_below = false;
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } if to.z() < 2.0 => exceptions.push(to.z()),
                Move::Linear { to, .. } if to.z() < 2.0 => fed_below = true,
                _ => {}
            }
        }
        assert!(fed_below, "the plunge has to go below the R plane");
        assert!(exceptions.is_empty(), "exceptions: none, found {exceptions:?}");
    }

    #[test]
    fn expand_rejects_a_low_r_plane_a_low_clearance_and_a_non_finite_height() {
        let drilling = one_hole(DrillCycle::new(), -3.0);
        let cases = [
            (0.0, -3.0, 5.0, "R plane must be above the hole bottom"),
            (0.0, -0.5, 5.0, "R plane must be above the hole bottom"),
            (0.0, 2.0, 1.0, "clearance must be at or above the R plane"),
            (0.0, f64::NAN, 5.0, "R plane and clearance must be finite"),
            (0.0, 2.0, f64::INFINITY, "R plane and clearance must be finite"),
        ];
        for (top_z, r_plane, clearance, needle) in cases {
            let err = drilling.expand(top_z, r_plane, clearance, CutKind::Plunge).unwrap_err();
            assert_eq!(err, PathError::InvalidInput(needle.into()), "{needle}");
        }
    }
}
