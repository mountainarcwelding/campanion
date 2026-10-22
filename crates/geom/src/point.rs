//! Points and free vectors in the XY plane. Units are millimetres.

use std::ops::{Add, AddAssign, Mul, Neg, Sub};

use crate::Result;
use crate::util::finite;

/// A position in the XY plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    x: f64,
    y: f64,
}

impl Point {
    /// Finite coordinates only. NaN and infinity are rejected.
    pub fn new(x: f64, y: f64) -> Result<Self> {
        Ok(Self { x: finite(x, "point x")?, y: finite(y, "point y")? })
    }

    pub(crate) fn xy(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn x(self) -> f64 {
        self.x
    }

    pub fn y(self) -> f64 {
        self.y
    }

    pub fn distance(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx.hypot(dy)
    }

    pub fn midpoint(self, other: Self) -> Self {
        Self { x: (self.x + other.x) * 0.5, y: (self.y + other.y) * 0.5 }
    }

    pub fn almost_eq(self, other: Self, tol: f64) -> bool {
        self.distance(other) <= tol
    }
}

impl Add<Vec2> for Point {
    type Output = Self;

    fn add(self, rhs: Vec2) -> Self {
        Self { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl AddAssign<Vec2> for Point {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Point {
    type Output = Vec2;

    fn sub(self, rhs: Self) -> Vec2 {
        Vec2 { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

/// A free vector in the XY plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Result<Self> {
        Ok(Self { x: finite(x, "vector x")?, y: finite(y, "vector y")? })
    }

    pub(crate) fn xy(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn x(self) -> f64 {
        self.x
    }

    pub fn y(self) -> f64 {
        self.y
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Z component of the 3D cross product. Positive when `other` lies to the left of `self`.
    pub fn cross(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }

    pub(crate) fn normalized(self) -> Option<Self> {
        let len = self.length();
        if !len.is_finite() || len <= f64::EPSILON { None } else { Some(Self { x: self.x / len, y: self.y / len }) }
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Self { x: -self.x, y: -self.y }
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self { x: self.x * rhs, y: self.y * rhs }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_non_finite_components() {
        assert!(Point::new(f64::NAN, 0.0).is_err());
        assert!(Point::new(0.0, f64::INFINITY).is_err());
        assert!(Vec2::new(f64::NEG_INFINITY, 1.0).is_err());
    }

    #[test]
    fn distance_and_cross_match_a_3_4_5_triangle() {
        let origin = Point::new(0.0, 0.0).expect("origin");
        let along = Point::new(3.0, 0.0).expect("along");
        let up = Point::new(0.0, 4.0).expect("up");
        assert!((origin.distance(along) - 3.0).abs() < 1e-12);
        assert!((along.distance(up) - 5.0).abs() < 1e-12);
        let v = along - origin;
        let w = up - origin;
        assert!((v.cross(w) - 12.0).abs() < 1e-12);
        assert!((v.dot(w)).abs() < 1e-12);
    }
}
