//! Circular arcs: centre, radius, start angle, and signed sweep.
//!
//! Sweep is radians, positive counter-clockwise. A bulge is `tan(sweep / 4)`, the DXF convention:
//! positive bulge leaves the arc to the left of the chord.

use crate::GeomError;
use crate::Result;
use crate::bbox::BoundingBox;
use crate::point::{Point, Vec2};
use crate::util::{self, PI, TAU};

/// A circular arc of at most one full turn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc {
    center: Point,
    radius: f64,
    start_angle: f64,
    sweep: f64,
}

impl Arc {
    /// `start_angle` is radians from +X, counter-clockwise. `sweep` is signed and must lie in `[-τ, τ]`.
    pub fn new(center: Point, radius: f64, start_angle: f64, sweep: f64) -> Result<Self> {
        let radius = util::finite(radius, "arc radius")?;
        let start_angle = util::finite(start_angle, "arc start angle")?;
        let mut sweep = util::finite(sweep, "arc sweep")?;
        if radius <= 0.0 {
            return Err(GeomError::InvalidInput("arc radius must be positive".into()));
        }
        if sweep == 0.0 {
            return Err(GeomError::InvalidInput("arc sweep must be non-zero".into()));
        }
        if sweep.abs() > TAU + 1e-6 {
            return Err(GeomError::InvalidInput("arc sweep exceeds one full turn".into()));
        }
        if sweep.abs() > TAU {
            sweep = sweep.signum() * TAU;
        }
        Ok(Self { center, radius, start_angle: util::wrap_tau(start_angle), sweep })
    }

    /// Arc from `start` to `end` with DXF bulge `tan(included_angle / 4)`.
    pub fn from_bulge(start: Point, end: Point, bulge: f64) -> Result<Self> {
        let bulge = util::finite(bulge, "bulge")?;
        if bulge.abs() <= util::BULGE_LINE_EPS {
            return Err(GeomError::InvalidInput("bulge is zero; use a line".into()));
        }
        let chord = end - start;
        let chord_len = chord.length();
        if !chord_len.is_finite() || chord_len <= 0.0 {
            return Err(GeomError::InvalidInput("bulge chord length must be positive".into()));
        }
        // θ = 4 atan(b), |r| = L (1+b²) / (4|b|), centre offset from the chord midpoint
        // along the left normal is L (1-b²) / (4b).
        let sweep = 4.0 * bulge.atan();
        let radius = chord_len * (1.0 + bulge * bulge) / (4.0 * bulge.abs());
        let offset = chord_len * (1.0 - bulge * bulge) / (4.0 * bulge);
        if !radius.is_finite() || !offset.is_finite() || radius <= 0.0 {
            return Err(GeomError::InvalidInput("bulge does not define a finite arc".into()));
        }
        let left = Vec2::xy(-chord.y() / chord_len, chord.x() / chord_len);
        let center = start.midpoint(end) + left * offset;
        if !center.x().is_finite() || !center.y().is_finite() {
            return Err(GeomError::InvalidInput("bulge centre is not finite".into()));
        }
        let start_angle = (start - center).angle();
        let arc = Self::new(center, radius, start_angle, sweep)?;
        // A tiny bulge makes a huge radius whose reconstructed end can drift. Refuse that.
        if arc.end_point().distance(end) > 1e-6 {
            return Err(GeomError::InvalidInput("bulge arc does not reach the end point".into()));
        }
        Ok(arc)
    }

    /// Bulge of this arc. Sweeps larger than half a turn have no single finite bulge; split them first.
    pub fn bulge(self) -> Result<f64> {
        if self.sweep.abs() > PI + 1e-9 {
            return Err(GeomError::InvalidInput("bulge is not defined for a sweep larger than half a turn".into()));
        }
        let bulge = (self.sweep * 0.25).tan();
        if !bulge.is_finite() {
            return Err(GeomError::InvalidInput("bulge is not finite".into()));
        }
        Ok(bulge)
    }

    pub fn center(self) -> Point {
        self.center
    }

    pub fn radius(self) -> f64 {
        self.radius
    }

    pub fn start_angle(self) -> f64 {
        self.start_angle
    }

    pub fn sweep(self) -> f64 {
        self.sweep
    }

    pub fn end_angle(self) -> f64 {
        self.start_angle + self.sweep
    }

    pub fn point_at(self, t: f64) -> Point {
        let angle = self.start_angle + self.sweep * t;
        self.center + Vec2::xy(self.radius * angle.cos(), self.radius * angle.sin())
    }

    pub fn start_point(self) -> Point {
        self.point_at(0.0)
    }

    /// End point.
    ///
    /// A full turn ends on its start. `point_at(1)` of a sweep of τ is short by about one ulp
    /// of the angle (`sin(τ)` is not zero), which is enough to make one post treat the move as
    /// a full circle and another as a near-zero arc. The same snap applies when the sweep is
    /// already the long way and the analytic end lies within [`full_turn_chord`] of the start.
    pub fn end_point(self) -> Point {
        if self.is_full_turn() { self.start_point() } else { self.point_at(1.0) }
    }

    /// Gap, in millimetres, that still meets the start of a full turn.
    ///
    /// The floor is 1e-9 mm. Above that the gap may be `1e-9` radii. A short arc that only
    /// meets its start after rounding to a post's decimals is wider than this.
    pub fn full_turn_chord(radius: f64) -> f64 {
        1e-9_f64.max(radius.abs() * 1e-9)
    }

    /// One full turn: the sweep is within 1e-8 rad of τ, or it is already the long way and the
    /// analytic end lies within [`Self::full_turn_chord`] of the start.
    pub fn is_full_turn(self) -> bool {
        if self.sweep.abs() >= TAU - 1e-8 {
            return true;
        }
        self.sweep.abs() > PI && self.start_point().distance(self.point_at(1.0)) <= Self::full_turn_chord(self.radius)
    }

    pub fn length(self) -> f64 {
        self.radius * self.sweep.abs()
    }

    pub fn reversed(self) -> Self {
        Self { center: self.center, radius: self.radius, start_angle: util::wrap_tau(self.end_angle()), sweep: -self.sweep }
    }

    pub(crate) fn start_tangent(self) -> Vec2 {
        self.tangent_at(self.start_angle)
    }

    pub(crate) fn end_tangent(self) -> Vec2 {
        self.tangent_at(self.end_angle())
    }

    fn tangent_at(self, theta: f64) -> Vec2 {
        let (sin_t, cos_t) = theta.sin_cos();
        // d/dθ of (cos θ, sin θ) is (-sin θ, cos θ). Flip when the sweep is clockwise.
        if self.sweep >= 0.0 { Vec2::xy(-sin_t, cos_t) } else { Vec2::xy(sin_t, -cos_t) }
    }

    pub fn bounding_box(self) -> BoundingBox {
        let mut box_ = BoundingBox::from_points(self.start_point(), self.end_point());
        for k in 0..4 {
            let angle = f64::from(k) * PI * 0.5;
            if self.param_of_angle(angle).is_some() {
                box_ = box_.expand(self.center + Vec2::xy(self.radius * angle.cos(), self.radius * angle.sin()));
            }
        }
        box_
    }

    /// Parameter `t` in `[0, 1]` if `theta` lies on the sweep, otherwise `None`.
    pub(crate) fn param_of_angle(self, theta: f64) -> Option<f64> {
        if !theta.is_finite() {
            return None;
        }
        let sweep = self.sweep;
        let abs_sweep = sweep.abs();
        if abs_sweep < 1e-15 {
            return None;
        }
        if abs_sweep >= TAU - 1e-8 {
            let ccw = (theta - self.start_angle).rem_euclid(TAU);
            let t = if sweep >= 0.0 { ccw / TAU } else { (self.start_angle - theta).rem_euclid(TAU) / TAU };
            return Some(t.clamp(0.0, 1.0));
        }
        let along = if sweep > 0.0 { (theta - self.start_angle).rem_euclid(TAU) } else { (self.start_angle - theta).rem_euclid(TAU) };
        let slack = 1e-8;
        let adjusted = if TAU - along <= slack { 0.0 } else { along };
        if adjusted <= abs_sweep + slack { Some((adjusted / abs_sweep).clamp(0.0, 1.0)) } else { None }
    }

    pub fn distance_to_point(self, p: Point) -> f64 {
        let offset = p - self.center;
        let radial = offset.length();
        if radial > 0.0 && offset.angle().is_finite() && self.param_of_angle(offset.angle()).is_some() {
            (radial - self.radius).abs()
        } else {
            self.start_point().distance(p).min(self.end_point().distance(p))
        }
    }

    /// Green's theorem contribution `∫ (x dy - y dx) / 2` along the arc.
    pub(crate) fn area_contribution(self) -> f64 {
        let t0 = self.start_angle;
        let t1 = self.end_angle();
        let (cx, cy, r) = (self.center.x(), self.center.y(), self.radius);
        let integrated = r * (cx * (t1.sin() - t0.sin()) - cy * (t1.cos() - t0.cos())) + r * r * self.sweep;
        integrated * 0.5
    }

    pub(crate) fn winding(self, p: Point) -> i32 {
        let mut cuts = vec![0.0, 1.0];
        for k in 0..4 {
            let angle = f64::from(k) * PI * 0.5;
            if let Some(t) = self.param_of_angle(angle) {
                cuts.push(t);
            }
        }
        cuts.sort_by(|a, b| a.total_cmp(b));
        let mut uniq: Vec<f64> = Vec::new();
        for t in cuts {
            if uniq.last().is_none_or(|prev| (t - prev).abs() > 1e-12) {
                uniq.push(t);
            }
        }
        let mut winding = 0;
        for pair in uniq.windows(2) {
            let (Some(&t0), Some(&t1)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            if t1 - t0 <= 1e-15 {
                continue;
            }
            let y0 = self.point_at(t0).y();
            let y1 = self.point_at(t1).y();
            let py = p.y();
            let upward = y0 <= py && y1 > py;
            let downward = y0 > py && y1 <= py;
            if !upward && !downward {
                continue;
            }
            if let Some(x) = self.crossing_x(t0, t1, py)
                && x > p.x()
            {
                winding += if upward { 1 } else { -1 };
            }
        }
        winding
    }

    fn crossing_x(self, t0: f64, t1: f64, py: f64) -> Option<f64> {
        let dy = py - self.center.y();
        let disc = self.radius * self.radius - dy * dy;
        if !disc.is_finite() || disc < 0.0 {
            return None;
        }
        let root = disc.sqrt();
        let candidates = [self.center.x() - root, self.center.x() + root];
        for x in candidates {
            let theta = dy.atan2(x - self.center.x());
            if let Some(t) = self.param_of_angle(theta)
                && t >= t0 - 1e-7
                && t <= t1 + 1e-7
            {
                return Some(x);
            }
        }
        None
    }

    /// Split into spans of at most half a turn so each has a bulge in `[-1, 1]`.
    pub(crate) fn split_for_bulge(self) -> Result<Vec<Self>> {
        let mut left = self.sweep;
        let mut start = self.start_angle;
        let mut out = Vec::new();
        let mut guard = 0;
        while left.abs() > 1e-15 {
            guard += 1;
            if guard > 8 {
                return Err(GeomError::InvalidInput("arc split did not finish".into()));
            }
            let chunk = if left.abs() > PI { left.signum() * PI } else { left };
            out.push(Self::new(self.center, self.radius, start, chunk)?);
            start += chunk;
            left -= chunk;
        }
        if out.is_empty() {
            return Err(GeomError::InvalidInput("arc sweep is empty".into()));
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_circle_bulge_roundtrip_is_tan_pi_over_8() {
        // tan(π/8) = √2 - 1. Start (1, 0), end (0, 1), centre (0, 0), radius 1, sweep π/2.
        let bulge = 2.0_f64.sqrt() - 1.0;
        let start = Point::new(1.0, 0.0).expect("start");
        let end = Point::new(0.0, 1.0).expect("end");
        let arc = Arc::from_bulge(start, end, bulge).expect("arc");
        assert!(arc.center().almost_eq(Point::xy(0.0, 0.0), 1e-12));
        assert!((arc.radius() - 1.0).abs() < 1e-12);
        assert!((arc.sweep() - PI * 0.5).abs() < 1e-12);
        assert!(arc.end_point().almost_eq(end, 1e-12));
        assert!((arc.bulge().expect("bulge") - bulge).abs() < 1e-12);
        assert!((arc.length() - PI * 0.5).abs() < 1e-12);
        let box_ = arc.bounding_box();
        assert!(box_.min().almost_eq(Point::xy(0.0, 0.0), 1e-12));
        assert!(box_.max().almost_eq(Point::xy(1.0, 1.0), 1e-12));
    }

    #[test]
    fn clockwise_bulge_places_the_centre() {
        // Sweep -π/2 from (1, 0) to (0, -1). Bulge = -tan(π/8).
        let bulge = -(2.0_f64.sqrt() - 1.0);
        let arc = Arc::from_bulge(Point::new(1.0, 0.0).unwrap(), Point::new(0.0, -1.0).unwrap(), bulge).unwrap();
        assert!(arc.center().almost_eq(Point::xy(0.0, 0.0), 1e-12));
        assert!((arc.sweep() + PI * 0.5).abs() < 1e-12);
    }

    #[test]
    fn rejects_bad_arcs() {
        let c = Point::new(0.0, 0.0).unwrap();
        assert!(Arc::new(c, 0.0, 0.0, 1.0).is_err());
        assert!(Arc::new(c, -2.0, 0.0, 1.0).is_err());
        assert!(Arc::new(c, 1.0, 0.0, 0.0).is_err());
        assert!(Arc::new(c, 1.0, f64::NAN, 1.0).is_err());
        assert!(Arc::new(c, 1.0, 0.0, TAU * 2.0).is_err());
        assert!(Arc::from_bulge(c, c, 1.0).is_err());
        assert!(Arc::from_bulge(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap(), f64::INFINITY).is_err());
    }

    #[test]
    fn semicircle_bulge_is_one() {
        let arc = Arc::new(Point::new(0.0, 0.0).unwrap(), 5.0, 0.0, PI).unwrap();
        assert!((arc.bulge().unwrap() - 1.0).abs() < 1e-12);
        assert!(Arc::new(Point::new(0.0, 0.0).unwrap(), 5.0, 0.0, TAU).unwrap().bulge().is_err());
    }

    #[test]
    fn full_turn_ends_on_its_start() {
        let centre = Point::new(40.0, 40.05843).unwrap();
        let ccw = Arc::new(centre, 27.0, 0.3, TAU).unwrap();
        assert!(ccw.end_point().almost_eq(ccw.start_point(), 0.0));
        let cw = Arc::new(centre, 2.0, 1.1, -TAU).unwrap();
        assert!(cw.end_point().almost_eq(cw.start_point(), 0.0));
        let quarter = Arc::new(centre, 27.0, 0.0, PI * 0.5).unwrap();
        assert!(quarter.end_point().distance(quarter.start_point()) > 1.0);
    }
}
