//! Follow a contour on the line.
//!
//! Engraving does not compensate. Open and closed contours are both followed in the
//! direction they are stored. The cutting length is the contour length.

use campanion_geom::{Contour, Tolerance};

use crate::Result;
use crate::check::{self, finite};
use crate::toolpath::{CutKind, Move, ProcessEvent, Toolpath};

/// A finished engrave path.
#[derive(Clone, Debug, PartialEq)]
pub struct Engrave {
    toolpath: Toolpath,
}

impl Engrave {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }
}

/// On-line follow. There is no tool radius and no side.
#[derive(Clone, Debug, PartialEq)]
pub struct EngraveOp {
    z: f64,
    tolerance: Tolerance,
}

impl EngraveOp {
    pub fn new() -> Self {
        Self { z: 0.0, tolerance: Tolerance::default() }
    }

    pub fn z(mut self, z: f64) -> Self {
        self.z = z;
        self
    }

    pub fn tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    pub fn generate(&self, contour: &Contour) -> Result<Engrave> {
        let z = finite(self.z, "z")?;
        if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
            return Err(check::invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
        }
        let length = contour.length()?;
        if !length.is_finite() || length <= self.tolerance.linear() {
            return Err(check::invalid("contour length is too small"));
        }
        let start = contour.segments().first().map(|seg| seg.start()).ok_or_else(|| check::invalid("contour is empty"))?;
        let mut moves = Vec::new();
        check::push(&mut moves, Move::Rapid { to: check::point3(start, z)? })?;
        check::push(&mut moves, Move::Event(ProcessEvent::On))?;
        for seg in contour.segments() {
            check::push_segment(*seg, z, CutKind::Cut, &mut moves)?;
        }
        check::push(&mut moves, Move::Event(ProcessEvent::Off))?;
        Ok(Engrave { toolpath: Toolpath::new(moves)? })
    }
}

impl Default for EngraveOp {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PathError;
    use campanion_geom::{Line, Point, Polyline, Segment, Vertex};
    use std::f64::consts::PI;

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b}");
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
    fn closed_and_open_lengths_match_the_contour() {
        let shape = rect(20.0, 10.0);
        let engrave = EngraveOp::new().z(-1.25).generate(&shape).unwrap();
        near(engrave.toolpath().cutting_length().unwrap(), shape.length().unwrap(), 1e-9);
        near(engrave.toolpath().cutting_length().unwrap(), 60.0, 1e-9);
        let moves = engrave.toolpath().moves();
        assert!(matches!(moves.first(), Some(Move::Rapid { .. })));
        assert!(matches!(moves.get(1), Some(Move::Event(ProcessEvent::On))));
        assert!(matches!(moves.last(), Some(Move::Event(ProcessEvent::Off))));
        assert!(moves.iter().all(|mv| match mv {
            Move::Rapid { to } | Move::Linear { to, .. } => (to.z() + 1.25).abs() < 1e-12,
            Move::Arc { .. } => false,
            Move::Event(_) | Move::Dwell { .. } => true,
        }));

        let open = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(3.0, 0.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(3.0, 0.0).unwrap(), Point::new(3.0, 4.0).unwrap()).unwrap()),
            ],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let followed = EngraveOp::new().generate(&open).unwrap();
        near(followed.toolpath().cutting_length().unwrap(), 7.0, 1e-12);
        near(followed.toolpath().cutting_length().unwrap(), open.length().unwrap(), 1e-12);
    }

    #[test]
    fn a_circle_keeps_its_arc_length() {
        let r = 5.0;
        let circle = Polyline::new(
            vec![Vertex::new(Point::new(r, 0.0).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(-r, 0.0).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap()
        .to_contour(&Tolerance::default())
        .unwrap();
        let engrave = EngraveOp::new().generate(&circle).unwrap();
        near(engrave.toolpath().cutting_length().unwrap(), circle.length().unwrap(), 1e-9);
        near(engrave.toolpath().cutting_length().unwrap(), TAU_CIRC * r, 1e-8);
        let arcs = engrave.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Arc { kind: CutKind::Cut, .. })).count();
        assert_eq!(arcs, 2);
    }

    const TAU_CIRC: f64 = PI * 2.0;

    #[test]
    fn hostile_input_is_an_error() {
        let shape = rect(4.0, 4.0);
        for op in [EngraveOp::new().z(f64::NAN), EngraveOp::new().z(f64::INFINITY), EngraveOp::new().z(f64::NEG_INFINITY)] {
            let err = op.generate(&shape).expect_err("bad z");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        let point = Point::new(1.0, 1.0).unwrap();
        let degenerate = Contour::try_new(vec![Segment::Line(Line::new(point, point).unwrap())], true, &Tolerance::default()).unwrap();
        assert!(EngraveOp::new().generate(&degenerate).is_err());

        let mut segments = Vec::new();
        for i in 0..campanion_geom::MAX_SEGMENTS + 1 {
            let x0 = i as f64;
            segments.push(Segment::Line(Line::new(Point::new(x0, 0.0).unwrap(), Point::new(x0 + 1.0, 0.0).unwrap()).unwrap()));
        }
        let long = Contour::try_new(segments, false, &Tolerance::default()).unwrap();
        let err = EngraveOp::new().generate(&long).unwrap_err();
        assert!(err.to_string().contains("too many segments"), "{err}");
    }
}
