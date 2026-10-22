//! Bulge polylines. Bulge on vertex *i* describes the span to the next vertex.
//!
//! On an open polyline the last bulge is ignored. A bulge of zero is a line. `tan(sweep / 4)` is
//! the DXF convention; values outside [-1, 1] are major arcs and are accepted here.

use crate::GeomError;
use crate::Result;
use crate::arc::Arc;
use crate::contour::Contour;
use crate::line::Line;
use crate::point::Point;
use crate::segment::Segment;
use crate::tolerance::Tolerance;
use crate::util;

/// A vertex and the bulge of the span that leaves it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    point: Point,
    bulge: f64,
}

impl Vertex {
    pub fn new(point: Point, bulge: f64) -> Result<Self> {
        Ok(Self { point, bulge: util::finite(bulge, "bulge")? })
    }

    pub fn point(self) -> Point {
        self.point
    }

    pub fn bulge(self) -> f64 {
        self.bulge
    }
}

/// Open or closed polyline in bulge form.
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline {
    vertices: Vec<Vertex>,
    closed: bool,
}

impl Polyline {
    pub fn new(vertices: Vec<Vertex>, closed: bool) -> Result<Self> {
        if vertices.len() < 2 {
            return Err(GeomError::InvalidInput("polyline needs at least two vertices".into()));
        }
        Ok(Self { vertices, closed })
    }

    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }

    pub fn to_contour(&self, tolerance: &Tolerance) -> Result<Contour> {
        let n = self.vertices.len();
        let seg_count = if self.closed { n } else { n.saturating_sub(1) };
        if seg_count == 0 {
            return Err(GeomError::InvalidInput("polyline has no segments".into()));
        }
        let mut segments = Vec::with_capacity(seg_count);
        for i in 0..seg_count {
            let Some(from) = self.vertices.get(i) else {
                return Err(GeomError::InvalidInput("polyline index missing".into()));
            };
            let next_index = if i + 1 == n { 0 } else { i + 1 };
            let Some(to) = self.vertices.get(next_index) else {
                return Err(GeomError::InvalidInput("polyline index missing".into()));
            };
            if from.bulge.abs() <= util::BULGE_LINE_EPS {
                segments.push(Segment::Line(Line::new(from.point, to.point)?));
            } else {
                segments.push(Segment::Arc(Arc::from_bulge(from.point, to.point, from.bulge)?));
            }
        }
        Contour::try_new(segments, self.closed, tolerance)
    }

    pub fn from_contour(contour: &Contour) -> Result<Self> {
        let mut vertices = Vec::new();
        for seg in contour.segments() {
            match *seg {
                Segment::Line(line) => vertices.push(Vertex::new(line.start(), 0.0)?),
                Segment::Arc(arc) => {
                    for piece in arc.split_for_bulge()? {
                        vertices.push(Vertex::new(piece.start_point(), piece.bulge()?)?);
                    }
                }
            }
        }
        if !contour.is_closed()
            && let Some(last) = contour.segments().last()
        {
            vertices.push(Vertex::new(last.end(), 0.0)?);
        }
        // A full-turn arc becomes two half-turns. Anything still above half a turn is a bug.
        if vertices.iter().any(|v| v.bulge.abs() > 1.0 + 1e-9) {
            return Err(GeomError::InvalidInput("bulge split left a span larger than half a turn".into()));
        }
        Self::new(vertices, contour.is_closed())
    }
}

impl Contour {
    pub fn to_polyline(&self) -> Result<Polyline> {
        Polyline::from_contour(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn circle_polyline_roundtrip_keeps_pi_r_squared() {
        let r = 4.0;
        let poly = Polyline::new(
            vec![Vertex::new(Point::new(r, 0.0).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(-r, 0.0).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap();
        let contour = poly.to_contour(&Tolerance::default()).unwrap();
        assert!((contour.signed_area().unwrap() - PI * r * r).abs() < 1e-8);
        let back = contour.to_polyline().unwrap();
        assert!(back.is_closed());
        assert_eq!(back.vertices().len(), 2);
        let again = back.to_contour(&Tolerance::default()).unwrap();
        assert!((again.signed_area().unwrap() - PI * r * r).abs() < 1e-8);
    }
}
