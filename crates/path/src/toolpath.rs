//! Toolpath geometry: rapids, line and arc feeds, dwells, and generic process events.
//!
//! End points are 3D so a later depth pass can carry Z. Arcs are circular in XY; `ccw` is the
//! sense when viewed from +Z. Nothing here is G-code.

use crate::{PathError, Result};

/// Largest toolpath this crate will hold. A hostile generator must fail instead of growing without bound.
pub const MAX_MOVES: usize = 65_536;

const TAU: f64 = std::f64::consts::TAU;

/// A position in millimetres. Z is up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Point3 {
    /// Finite coordinates only. NaN and infinity are rejected.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(PathError::InvalidInput("point must be finite".into()));
        }
        Ok(Self { x, y, z })
    }

    pub fn x(self) -> f64 {
        self.x
    }

    pub fn y(self) -> f64 {
        self.y
    }

    pub fn z(self) -> f64 {
        self.z
    }

    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn almost_eq(self, other: Self, tol: f64) -> bool {
        self.distance(other) <= tol
    }
}

/// Feed class. Leads, plunges and retracts are not ordinary cutting moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutKind {
    /// Cutting feed along the profile.
    Cut,
    /// Approach onto the profile.
    LeadIn,
    /// Departure off the profile.
    LeadOut,
    /// Vertical entry.
    Plunge,
    /// Vertical exit.
    Retract,
}

/// Generic process marker. Not tied to a spindle, torch, laser or other device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProcessEvent {
    /// Process becomes active (pierce, spindle, beam, jet, …).
    On,
    /// Process becomes inactive.
    Off,
}

/// One step of a toolpath.
///
/// [`Move::Arc`] is the unique arc in the requested sense from the previous point to `to`,
/// shorter than or equal to a full turn. When the previous point and `to` are the same point
/// the arc is a full turn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Move {
    /// Non-cutting positioning move.
    Rapid { to: Point3 },
    /// Straight feed.
    Linear { to: Point3, kind: CutKind },
    /// Circular feed. `centre` is the arc centre. `ccw` is counter-clockwise viewed from +Z.
    Arc { to: Point3, centre: Point3, ccw: bool, kind: CutKind },
    /// Process on/off. The tool does not move.
    Event(ProcessEvent),
    /// Pause, in seconds. The tool does not move.
    Dwell { seconds: f64 },
}

/// An ordered list of moves.
#[derive(Clone, Debug, PartialEq)]
pub struct Toolpath {
    moves: Vec<Move>,
}

impl Toolpath {
    pub fn new(moves: Vec<Move>) -> Result<Self> {
        if moves.len() > MAX_MOVES {
            return Err(PathError::InvalidInput(format!("too many moves: {} > {MAX_MOVES}", moves.len())));
        }
        Ok(Self { moves })
    }

    pub fn moves(&self) -> &[Move] {
        &self.moves
    }

    pub fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    /// Sum of linear and arc lengths whose [`CutKind`] is [`CutKind::Cut`].
    ///
    /// A rapid establishes position and adds nothing. The length is in millimetres and includes
    /// a helical rise when an arc changes Z.
    pub fn cutting_length(&self) -> Result<f64> {
        self.sum_lengths(|kind| kind == CutKind::Cut)
    }

    /// Sum of every linear and arc move, including leads, plunges and retracts.
    pub fn motion_length(&self) -> Result<f64> {
        self.sum_lengths(|_| true)
    }

    fn sum_lengths(&self, include: impl Fn(CutKind) -> bool) -> Result<f64> {
        let mut cursor: Option<Point3> = None;
        let mut sum = 0.0;
        for mv in &self.moves {
            match *mv {
                Move::Rapid { to } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
                Move::Linear { to, kind } => {
                    if include(kind) {
                        let Some(from) = cursor else {
                            return Err(PathError::InvalidInput("length needs a start position".into()));
                        };
                        let distance = from.distance(to);
                        if !distance.is_finite() {
                            return Err(PathError::InvalidInput("move length is not finite".into()));
                        }
                        sum += distance;
                    }
                    cursor = Some(to);
                }
                Move::Arc { to, centre, ccw, kind } => {
                    if include(kind) {
                        let Some(from) = cursor else {
                            return Err(PathError::InvalidInput("length needs a start position".into()));
                        };
                        sum += arc_length(from, to, centre, ccw)?;
                    }
                    cursor = Some(to);
                }
            }
        }
        if sum.is_finite() { Ok(sum) } else { Err(PathError::InvalidInput("toolpath length is not finite".into())) }
    }
}

fn arc_length(start: Point3, end: Point3, centre: Point3, ccw: bool) -> Result<f64> {
    let r0 = hypot2(start.x() - centre.x(), start.y() - centre.y());
    let r1 = hypot2(end.x() - centre.x(), end.y() - centre.y());
    if !r0.is_finite() || !r1.is_finite() || r0 <= 1e-15 {
        return Err(PathError::InvalidInput("arc radius is not positive".into()));
    }
    if (r0 - r1).abs() > 1e-3 * r0.max(1.0) {
        return Err(PathError::InvalidInput("arc start and end radii differ".into()));
    }
    let a0 = (start.y() - centre.y()).atan2(start.x() - centre.x());
    let a1 = (end.y() - centre.y()).atan2(end.x() - centre.x());
    if !a0.is_finite() || !a1.is_finite() {
        return Err(PathError::InvalidInput("arc angle is not finite".into()));
    }
    let chord = hypot2(end.x() - start.x(), end.y() - start.y());
    // Same closure as `campanion_geom::Arc::full_turn_chord`: 1e-9 mm, or 1e-9 radii.
    let closure = 1e-9_f64.max(r0 * 1e-9);
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= closure {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    let planar = r0 * sweep;
    let dz = end.z() - start.z();
    let length = (planar * planar + dz * dz).sqrt();
    if length.is_finite() { Ok(length) } else { Err(PathError::InvalidInput("arc length is not finite".into())) }
}

fn hypot2(x: f64, y: f64) -> f64 {
    x.hypot(y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z).expect("point")
    }

    #[test]
    fn square_and_quarter_circle_lengths() {
        // Rapid to the origin, then a 10 by 10 square. Cutting length is 40.
        // A separate quarter circle of radius 2, from (2, 0) to (0, 2), is π.
        let square = Toolpath::new(vec![
            Move::Rapid { to: p(0.0, 0.0, 0.0) },
            Move::Event(ProcessEvent::On),
            Move::Linear { to: p(10.0, 0.0, 0.0), kind: CutKind::Cut },
            Move::Linear { to: p(10.0, 10.0, 0.0), kind: CutKind::Cut },
            Move::Linear { to: p(0.0, 10.0, 0.0), kind: CutKind::Cut },
            Move::Linear { to: p(0.0, 0.0, 0.0), kind: CutKind::Cut },
            Move::Event(ProcessEvent::Off),
        ])
        .unwrap();
        assert!((square.cutting_length().unwrap() - 40.0).abs() < 1e-12);
        assert!((square.motion_length().unwrap() - 40.0).abs() < 1e-12);

        let arc = Toolpath::new(vec![
            Move::Rapid { to: p(2.0, 0.0, 0.0) },
            Move::Arc { to: p(0.0, 2.0, 0.0), centre: p(0.0, 0.0, 0.0), ccw: true, kind: CutKind::Cut },
            Move::Linear { to: p(0.0, 2.0, 1.0), kind: CutKind::Retract },
        ])
        .unwrap();
        assert!((arc.cutting_length().unwrap() - PI).abs() < 1e-12);
        assert!((arc.motion_length().unwrap() - (PI + 1.0)).abs() < 1e-12);
    }

    #[test]
    fn clockwise_quarter_and_full_turn() {
        // From (1, 0) clockwise to (0, -1) about the origin is a quarter turn.
        let quarter = Toolpath::new(vec![
            Move::Rapid { to: p(1.0, 0.0, 0.0) },
            Move::Arc { to: p(0.0, -1.0, 0.0), centre: p(0.0, 0.0, 0.0), ccw: false, kind: CutKind::LeadIn },
        ])
        .unwrap();
        assert!((quarter.motion_length().unwrap() - PI * 0.5).abs() < 1e-12);
        assert!((quarter.cutting_length().unwrap()).abs() < 1e-12);

        let full = Toolpath::new(vec![
            Move::Rapid { to: p(3.0, 0.0, 1.0) },
            Move::Arc { to: p(3.0, 0.0, 1.0), centre: p(0.0, 0.0, 1.0), ccw: true, kind: CutKind::Cut },
        ])
        .unwrap();
        assert!((full.cutting_length().unwrap() - TAU * 3.0).abs() < 1e-9);
    }

    #[test]
    fn hostile_points_and_move_count() {
        assert!(Point3::new(f64::NAN, 0.0, 0.0).is_err());
        assert!(Point3::new(0.0, f64::INFINITY, 0.0).is_err());
        assert!(Point3::new(0.0, 0.0, f64::NEG_INFINITY).is_err());
        let mut moves = Vec::new();
        for _ in 0..=MAX_MOVES {
            moves.push(Move::Event(ProcessEvent::On));
        }
        let err = Toolpath::new(moves).unwrap_err();
        assert!(err.to_string().contains("too many moves"), "{err}");
        assert!(Toolpath::new(vec![]).unwrap().is_empty());
        let no_start = Toolpath::new(vec![Move::Linear { to: p(1.0, 0.0, 0.0), kind: CutKind::Cut }]).unwrap();
        assert!(no_start.cutting_length().is_err());
    }
}
