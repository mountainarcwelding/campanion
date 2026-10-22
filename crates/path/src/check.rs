//! Shared numeric checks and toolpath emission.

use campanion_geom::{Point, Segment};

use crate::toolpath::{CutKind, MAX_MOVES, Move, Point3};
use crate::{PathError, Result};

pub(crate) fn invalid(message: impl Into<String>) -> PathError {
    PathError::InvalidInput(message.into())
}

pub(crate) fn finite(value: f64, what: &str) -> Result<f64> {
    if value.is_finite() { Ok(value) } else { Err(invalid(format!("{what} must be finite"))) }
}

pub(crate) fn non_negative(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value >= 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be >= 0"))) }
}

pub(crate) fn positive(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value > 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be positive"))) }
}

pub(crate) fn point3(point: Point, z: f64) -> Result<Point3> {
    Point3::new(point.x(), point.y(), z)
}

pub(crate) fn push(moves: &mut Vec<Move>, mv: Move) -> Result<()> {
    if moves.len() >= MAX_MOVES {
        return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
    }
    moves.push(mv);
    Ok(())
}

pub(crate) fn push_segment(seg: Segment, z: f64, kind: CutKind, moves: &mut Vec<Move>) -> Result<()> {
    match seg {
        Segment::Line(line) => push(moves, Move::Linear { to: point3(line.end(), z)?, kind })?,
        Segment::Arc(arc) => {
            push(moves, Move::Arc { to: point3(arc.end_point(), z)?, centre: point3(arc.center(), z)?, ccw: arc.sweep() > 0.0, kind })?;
        }
    }
    Ok(())
}
