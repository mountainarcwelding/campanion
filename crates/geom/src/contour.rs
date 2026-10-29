//! Open and closed contours made of lines and arcs.

use crate::GeomError;
use crate::Result;
use crate::bbox::BoundingBox;
use crate::point::Point;
use crate::segment::Segment;
use crate::tolerance::Tolerance;

/// Which way a closed contour winds. Outers are counter-clockwise; holes are clockwise.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    CounterClockwise,
    Clockwise,
    Degenerate,
}

/// Where a point sits relative to a closed contour. The boundary is a closed set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Containment {
    Inside,
    Outside,
    Boundary,
}

/// A connected chain of lines and arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct Contour {
    segments: Vec<Segment>,
    closed: bool,
}

impl Contour {
    pub fn try_new(segments: Vec<Segment>, closed: bool, tolerance: &Tolerance) -> Result<Self> {
        if segments.is_empty() {
            return Err(GeomError::InvalidInput("contour is empty".into()));
        }
        let tol = tolerance.linear();
        for pair in segments.windows(2) {
            let (Some(prev), Some(next)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            ensure_connected(prev.end(), next.start(), tol)?;
        }
        if closed {
            let (Some(first), Some(last)) = (segments.first(), segments.last()) else {
                return Err(GeomError::InvalidInput("contour is empty".into()));
            };
            ensure_connected(last.end(), first.start(), tol)?;
        }
        Ok(Self { segments, closed })
    }

    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }

    pub fn length(&self) -> Result<f64> {
        let sum: f64 = self.segments.iter().map(|seg| seg.length()).sum();
        if sum.is_finite() { Ok(sum) } else { Err(GeomError::InvalidInput("length is not finite".into())) }
    }

    /// Signed area in mm². Counter-clockwise is positive. Exact for circular arcs.
    ///
    /// Each edge contributes `∫ (x dy - y dx) / 2`. A residual chord closes a contour whose
    /// ends meet only within tolerance.
    pub fn signed_area(&self) -> Result<f64> {
        if !self.closed {
            return Err(GeomError::InvalidInput("open contour has no area".into()));
        }
        let mut area = 0.0;
        for seg in &self.segments {
            area += seg.area_contribution();
        }
        if let (Some(first), Some(last)) = (self.segments.first(), self.segments.last()) {
            let a = last.end();
            let b = first.start();
            area += (a.x() * b.y() - b.x() * a.y()) * 0.5;
        }
        if area.is_finite() { Ok(area) } else { Err(GeomError::InvalidInput("area is not finite".into())) }
    }

    pub fn orientation(&self, tolerance: &Tolerance) -> Result<Orientation> {
        let area = self.signed_area()?;
        let bounds = self.bounding_box()?;
        let scale = bounds.width().max(bounds.height()).max(tolerance.linear());
        let eps = tolerance.linear() * scale;
        if area > eps {
            Ok(Orientation::CounterClockwise)
        } else if area < -eps {
            Ok(Orientation::Clockwise)
        } else {
            Ok(Orientation::Degenerate)
        }
    }

    pub fn bounding_box(&self) -> Result<BoundingBox> {
        let mut iter = self.segments.iter();
        let Some(first) = iter.next() else {
            return Err(GeomError::InvalidInput("contour is empty".into()));
        };
        let mut bounds = first.bounding_box();
        for seg in iter {
            bounds = bounds.union(seg.bounding_box());
        }
        Ok(bounds)
    }

    /// Non-zero winding. Boundary (within the linear tolerance) is reported separately.
    pub fn containment(&self, point: Point, tolerance: &Tolerance) -> Result<Containment> {
        if !self.closed {
            return Err(GeomError::InvalidInput("open contour has no containment".into()));
        }
        for seg in &self.segments {
            let distance = seg.distance_to_point(point);
            if distance.is_finite() && distance <= tolerance.linear() {
                return Ok(Containment::Boundary);
            }
        }
        let winding: i32 = self.segments.iter().map(|seg| seg.winding(point)).sum();
        if winding == 0 { Ok(Containment::Outside) } else { Ok(Containment::Inside) }
    }

    pub fn reversed(&self) -> Self {
        let segments = self.segments.iter().rev().map(|seg| seg.reversed()).collect();
        Self { segments, closed: self.closed }
    }

    /// Rotate a closed contour onto a start vertex that does not follow libm noise.
    ///
    /// The vertex with the least (x, y), after rounding each coordinate to 1e-6 mm, wins.
    /// Equal rounded coordinates keep the earlier vertex. An open contour is unchanged.
    pub fn with_stable_start(&self) -> Self {
        if !self.closed || self.segments.len() < 2 {
            return self.clone();
        }
        let mut best = 0usize;
        for index in 1..self.segments.len() {
            let (Some(here), Some(champ)) = (self.segments.get(index), self.segments.get(best)) else {
                continue;
            };
            let here_xy = (here.start().x(), here.start().y());
            let champ_xy = (champ.start().x(), champ.start().y());
            if crate::cmp_xy_rounded(here_xy, champ_xy).then(index.cmp(&best)) == std::cmp::Ordering::Less {
                best = index;
            }
        }
        if best == 0 {
            return self.clone();
        }
        let mut segments = Vec::with_capacity(self.segments.len());
        if let Some(tail) = self.segments.get(best..) {
            segments.extend_from_slice(tail);
        }
        if let Some(head) = self.segments.get(..best) {
            segments.extend_from_slice(head);
        }
        Self { segments, closed: self.closed }
    }
}

pub(crate) fn ensure_connected(end: Point, start: Point, tol: f64) -> Result<()> {
    if end.almost_eq(start, tol) { Ok(()) } else { Err(GeomError::InvalidInput(format!("segments do not connect at ({}, {})", end.x(), end.y()))) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arc::Arc;
    use crate::line::Line;
    use crate::point::Point;
    use std::f64::consts::PI;

    fn rect(w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).expect("point");
        let line = |a, b| Segment::Line(Line::new(a, b).expect("line"));
        let tol = Tolerance::default();
        Contour::try_new(
            vec![line(p(0.0, 0.0), p(w, 0.0)), line(p(w, 0.0), p(w, h)), line(p(w, h), p(0.0, h)), line(p(0.0, h), p(0.0, 0.0))],
            true,
            &tol,
        )
        .expect("rect")
    }

    #[test]
    fn rectangle_area_length_and_containment() {
        let shape = rect(4.0, 3.0);
        assert!((shape.signed_area().expect("area") - 12.0).abs() < 1e-12);
        assert!((shape.length().expect("len") - 14.0).abs() < 1e-12);
        assert_eq!(shape.orientation(&Tolerance::default()).unwrap(), Orientation::CounterClockwise);
        let box_ = shape.bounding_box().unwrap();
        assert!(box_.min().almost_eq(Point::xy(0.0, 0.0), 1e-12));
        assert!(box_.max().almost_eq(Point::xy(4.0, 3.0), 1e-12));
        let tol = Tolerance::default();
        assert_eq!(shape.containment(Point::new(2.0, 1.5).unwrap(), &tol).unwrap(), Containment::Inside);
        assert_eq!(shape.containment(Point::new(-1.0, -1.0).unwrap(), &tol).unwrap(), Containment::Outside);
        assert_eq!(shape.containment(Point::new(2.0, 0.0).unwrap(), &tol).unwrap(), Containment::Boundary);
        assert_eq!(shape.containment(Point::new(0.0, 0.0).unwrap(), &tol).unwrap(), Containment::Boundary);
        let reversed = shape.reversed();
        assert!((reversed.signed_area().unwrap() + 12.0).abs() < 1e-12);
        assert_eq!(reversed.orientation(&tol).unwrap(), Orientation::Clockwise);
    }

    #[test]
    fn split_vertex_on_the_ray_does_not_count_twice() {
        // The shared corner is stored twice, one ulp above the ray and one ulp below.
        // Counting both edges puts a point to the left of the contour inside it.
        let tol = Tolerance::default();
        let above = 1e-15;
        let below = -1e-15;
        let p = |x, y| Point::new(x, y).unwrap();
        let line = |a, b| Segment::Line(Line::new(a, b).unwrap());
        let shape = Contour::try_new(
            vec![
                line(p(2.0, 5.0), p(2.0, below)),
                line(p(2.0, above), p(10.0, below)),
                line(p(10.0, below), p(10.0, 5.0)),
                line(p(10.0, 5.0), p(2.0, 5.0)),
            ],
            true,
            &tol,
        )
        .unwrap();
        assert_eq!(shape.containment(p(-1.0, 0.0), &tol).unwrap(), Containment::Outside);
        assert_eq!(shape.containment(p(6.0, 2.0), &tol).unwrap(), Containment::Inside);
    }

    #[test]
    fn three_four_five_triangle() {
        let p = |x, y| Point::new(x, y).unwrap();
        let line = |a, b| Segment::Line(Line::new(a, b).unwrap());
        let tri = Contour::try_new(
            vec![line(p(0.0, 0.0), p(3.0, 0.0)), line(p(3.0, 0.0), p(0.0, 4.0)), line(p(0.0, 4.0), p(0.0, 0.0))],
            true,
            &Tolerance::default(),
        )
        .unwrap();
        assert!((tri.signed_area().unwrap() - 6.0).abs() < 1e-12);
        assert!((tri.length().unwrap() - 12.0).abs() < 1e-12);
    }

    #[test]
    fn circle_from_two_bulge_arcs_has_area_pi_r_squared() {
        // Two semicircles, bulge = tan(π/4) = 1. Centre origin, radius 2.5.
        let r = 2.5;
        let east = Point::new(r, 0.0).unwrap();
        let west = Point::new(-r, 0.0).unwrap();
        let upper = Arc::from_bulge(east, west, 1.0).unwrap();
        let lower = Arc::from_bulge(west, east, 1.0).unwrap();
        let circle = Contour::try_new(vec![Segment::Arc(upper), Segment::Arc(lower)], true, &Tolerance::default()).unwrap();
        let area = circle.signed_area().unwrap();
        assert!((area - PI * r * r).abs() < 1e-9, "area {area}");
        assert!((circle.length().unwrap() - 2.0 * PI * r).abs() < 1e-9);
        let tol = Tolerance::default();
        assert_eq!(circle.containment(Point::new(0.0, 0.0).unwrap(), &tol).unwrap(), Containment::Inside);
        assert_eq!(circle.containment(Point::new(r + 0.1, 0.0).unwrap(), &tol).unwrap(), Containment::Outside);
        assert_eq!(circle.containment(Point::new(0.0, r).unwrap(), &tol).unwrap(), Containment::Boundary);
    }

    #[test]
    fn slot_area_is_rectangle_plus_circle() {
        // Stadium: centres 20 mm apart, radius 5. CCW, both caps bulge +1.
        // Area = 20 * 10 + π * 25. Length = 40 + 10π.
        let p = |x, y| Point::new(x, y).unwrap();
        let line = |a, b| Segment::Line(Line::new(a, b).unwrap());
        let cap_left = Arc::from_bulge(p(-10.0, 5.0), p(-10.0, -5.0), 1.0).unwrap();
        let cap_right = Arc::from_bulge(p(10.0, -5.0), p(10.0, 5.0), 1.0).unwrap();
        let slot = Contour::try_new(
            vec![line(p(10.0, 5.0), p(-10.0, 5.0)), Segment::Arc(cap_left), line(p(-10.0, -5.0), p(10.0, -5.0)), Segment::Arc(cap_right)],
            true,
            &Tolerance::default(),
        )
        .unwrap();
        let expected = 200.0 + 25.0 * PI;
        assert!((slot.signed_area().unwrap() - expected).abs() < 1e-8, "area {}", slot.signed_area().unwrap());
        assert!((slot.length().unwrap() - (40.0 + 10.0 * PI)).abs() < 1e-8);
        let tol = Tolerance::default();
        assert_eq!(slot.containment(p(0.0, 0.0), &tol).unwrap(), Containment::Inside);
        assert_eq!(slot.containment(p(0.0, 6.0), &tol).unwrap(), Containment::Outside);
        assert_eq!(slot.containment(p(14.0, 0.0), &tol).unwrap(), Containment::Inside);
        assert_eq!(slot.containment(p(16.0, 0.0), &tol).unwrap(), Containment::Outside);
        assert_eq!(slot.containment(p(10.0, 5.0), &tol).unwrap(), Containment::Boundary);
        assert_eq!(slot.orientation(&tol).unwrap(), Orientation::CounterClockwise);
    }

    #[test]
    fn open_contour_refuses_area_and_containment() {
        let line = Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap());
        let open = Contour::try_new(vec![line], false, &Tolerance::default()).unwrap();
        assert!(open.signed_area().is_err());
        assert!(open.containment(Point::new(0.0, 0.0).unwrap(), &Tolerance::default()).is_err());
        assert!((open.length().unwrap() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn disconnected_segments_are_rejected() {
        let a = Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap());
        let b = Segment::Line(Line::new(Point::new(2.0, 0.0).unwrap(), Point::new(3.0, 0.0).unwrap()).unwrap());
        let err = Contour::try_new(vec![a, b], false, &Tolerance::default()).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("do not connect"), "{text}");
    }

    fn closed_poly(points: &[(f64, f64)]) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let line = |a, b| Segment::Line(Line::new(a, b).unwrap());
        let mut segs = Vec::new();
        for pair in points.windows(2) {
            segs.push(line(p(pair[0].0, pair[0].1), p(pair[1].0, pair[1].1)));
        }
        let (Some(first), Some(last)) = (points.first(), points.last()) else {
            panic!("polygon needs vertices");
        };
        segs.push(line(p(last.0, last.1), p(first.0, first.1)));
        Contour::try_new(segs, true, &Tolerance::default()).unwrap()
    }

    #[test]
    fn stable_start_follows_the_low_vertex_under_mirror_and_ulp_noise() {
        // Diamond. The west vertex is the unique rounded-lex start, whichever vertex is first.
        let diamond = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        for shift in 0..diamond.len() {
            let mut pts = diamond.to_vec();
            pts.rotate_left(shift);
            let start = closed_poly(&pts).with_stable_start().segments().first().unwrap().start();
            assert!((start.x() + 1.0).abs() < 1e-12 && start.y().abs() < 1e-12, "shift {shift}");
        }
        // Five ulps toward zero still round to the same micrometre and still beat the mirrors.
        let mut west_x = -1.0_f64;
        for _ in 0..5 {
            west_x = west_x.next_up();
        }
        let noisy = [(1.0, 0.0), (0.0, 1.0), (west_x, 0.0), (0.0, -1.0)];
        for bump in -5_i32..=5 {
            let mut moved = noisy;
            let mut west = west_x;
            for _ in 0..bump.unsigned_abs() {
                west = if bump > 0 { west.next_up() } else { west.next_down() };
                for pt in &mut moved {
                    pt.0 = if bump > 0 { pt.0.next_up() } else { pt.0.next_down() };
                }
            }
            for shift in 0..moved.len() {
                let mut pts = moved;
                pts.rotate_left(shift);
                let start = closed_poly(&pts).with_stable_start().segments().first().unwrap().start();
                assert!((start.x() - west).abs() < 1e-15, "shift {shift} bump {bump} start {}", start.x());
            }
        }
        // A vertex a tenth of a millimetre lower wins even when it is last.
        let clear = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-0.1, 0.5)];
        let start = closed_poly(&clear).with_stable_start().segments().first().unwrap().start();
        assert!((start.x() + 0.1).abs() < 1e-12);

        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(1.0, 0.0).unwrap(), Point::new(-5.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        assert!((open.with_stable_start().segments()[0].start().x() - 1.0).abs() < 1e-12);
    }
}
