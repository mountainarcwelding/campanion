//! Axis-aligned bounding boxes.

use crate::point::Point;

/// Inclusive axis-aligned bounds in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox {
    min: Point,
    max: Point,
}

impl BoundingBox {
    pub fn from_point(p: Point) -> Self {
        Self { min: p, max: p }
    }

    pub fn from_points(a: Point, b: Point) -> Self {
        Self { min: Point::xy(a.x().min(b.x()), a.y().min(b.y())), max: Point::xy(a.x().max(b.x()), a.y().max(b.y())) }
    }

    pub fn expand(self, p: Point) -> Self {
        Self { min: Point::xy(self.min.x().min(p.x()), self.min.y().min(p.y())), max: Point::xy(self.max.x().max(p.x()), self.max.y().max(p.y())) }
    }

    pub fn union(self, other: Self) -> Self {
        self.expand(other.min).expand(other.max)
    }

    pub fn min(self) -> Point {
        self.min
    }

    pub fn max(self) -> Point {
        self.max
    }

    pub fn width(self) -> f64 {
        self.max.x() - self.min.x()
    }

    pub fn height(self) -> f64 {
        self.max.y() - self.min.y()
    }

    pub fn contains_point(self, p: Point, tol: f64) -> bool {
        p.x() >= self.min.x() - tol && p.x() <= self.max.x() + tol && p.y() >= self.min.y() - tol && p.y() <= self.max.y() + tol
    }
}
