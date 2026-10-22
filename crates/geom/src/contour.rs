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
}
