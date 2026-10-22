//! A contour edge: a line or a circular arc.

use crate::arc::Arc;
use crate::bbox::BoundingBox;
use crate::line::Line;
use crate::point::{Point, Vec2};

/// One edge of a contour.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Segment {
    Line(Line),
    Arc(Arc),
}

impl Segment {
    pub fn start(self) -> Point {
        match self {
            Self::Line(line) => line.start(),
            Self::Arc(arc) => arc.start_point(),
        }
    }

    pub fn end(self) -> Point {
        match self {
            Self::Line(line) => line.end(),
            Self::Arc(arc) => arc.end_point(),
        }
    }

    pub fn length(self) -> f64 {
        match self {
            Self::Line(line) => line.length(),
            Self::Arc(arc) => arc.length(),
        }
    }

    pub fn reversed(self) -> Self {
        match self {
            Self::Line(line) => Self::Line(line.reversed()),
            Self::Arc(arc) => Self::Arc(arc.reversed()),
        }
    }

    pub fn bounding_box(self) -> BoundingBox {
        match self {
            Self::Line(line) => line.bounding_box(),
            Self::Arc(arc) => arc.bounding_box(),
        }
    }

    pub fn distance_to_point(self, p: Point) -> f64 {
        match self {
            Self::Line(line) => line.distance_to_point(p),
            Self::Arc(arc) => arc.distance_to_point(p),
        }
    }

    pub(crate) fn area_contribution(self) -> f64 {
        match self {
            Self::Line(line) => line.area_contribution(),
            Self::Arc(arc) => arc.area_contribution(),
        }
    }

    pub(crate) fn winding(self, p: Point) -> i32 {
        match self {
            Self::Line(line) => line.winding(p),
            Self::Arc(arc) => arc.winding(p),
        }
    }

    pub(crate) fn start_tangent(self) -> Option<Vec2> {
        match self {
            Self::Line(line) => line.direction().normalized(),
            Self::Arc(arc) => arc.start_tangent().normalized(),
        }
    }

    pub(crate) fn end_tangent(self) -> Option<Vec2> {
        match self {
            Self::Line(line) => line.direction().normalized(),
            Self::Arc(arc) => arc.end_tangent().normalized(),
        }
    }
}
