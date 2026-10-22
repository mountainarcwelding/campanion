//! Straight segments.

use crate::Result;
use crate::bbox::BoundingBox;
use crate::point::{Point, Vec2};

/// A finite straight segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line {
    start: Point,
    end: Point,
}

impl Line {
    pub fn new(start: Point, end: Point) -> Result<Self> {
        Ok(Self { start, end })
    }

    pub fn start(self) -> Point {
        self.start
    }

    pub fn end(self) -> Point {
        self.end
    }

    pub fn length(self) -> f64 {
        self.start.distance(self.end)
    }

    pub fn direction(self) -> Vec2 {
        self.end - self.start
    }

    pub fn reversed(self) -> Self {
        Self { start: self.end, end: self.start }
    }

    pub fn bounding_box(self) -> BoundingBox {
        BoundingBox::from_points(self.start, self.end)
    }

    pub fn distance_to_point(self, p: Point) -> f64 {
        let ab = self.direction();
        let len2 = ab.dot(ab);
        if !len2.is_finite() || len2 <= 1e-24 {
            return self.start.distance(p);
        }
        let ap = p - self.start;
        let t = (ap.dot(ab) / len2).clamp(0.0, 1.0);
        let projected = self.start + ab * t;
        p.distance(projected)
    }

    /// Shoelace term `(x0 y1 - x1 y0) / 2`.
    pub(crate) fn area_contribution(self) -> f64 {
        (self.start.x() * self.end.y() - self.end.x() * self.start.y()) * 0.5
    }

    /// Non-zero winding contribution of this edge against a horizontal ray to +∞.
    ///
    /// Uses the half-open rule: an upward crossing counts when the end is strictly
    /// above the ray, a downward crossing when the end is at or below it.
    pub(crate) fn winding(self, p: Point) -> i32 {
        let y0 = self.start.y();
        let y1 = self.end.y();
        let py = p.y();
        let upward = y0 <= py && y1 > py;
        let downward = y0 > py && y1 <= py;
        if !upward && !downward {
            return 0;
        }
        let dy = y1 - y0;
        if !dy.is_finite() || dy == 0.0 {
            return 0;
        }
        let t = (py - y0) / dy;
        let x = self.start.x() + t * (self.end.x() - self.start.x());
        if x > p.x() { if upward { 1 } else { -1 } } else { 0 }
    }
}
