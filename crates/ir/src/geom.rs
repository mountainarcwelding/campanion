//! Positions and tool axes in the part frame.
//!
//! Z is up. Frames are right-handed. Lengths are millimetres.

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, finite};

/// A vector in millimetres, or a unitless direction.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Vec3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vec3 {
    /// +X.
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    /// +Y.
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    /// +Z, the default tool axis.
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };
    /// Origin.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    /// Finite coordinates only.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self> {
        Ok(Self { x: finite(x, "x")?, y: finite(y, "y")?, z: finite(z, "z")? })
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

    /// Components as given. The caller must pass finite values when they will be checked later.
    pub(crate) fn from_xyz(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub(crate) fn check(self, what: &str) -> Result<()> {
        finite(self.x, what)?;
        finite(self.y, what)?;
        finite(self.z, what)?;
        Ok(())
    }

    pub(crate) fn check_non_zero(self, what: &str) -> Result<()> {
        self.check(what)?;
        if self.x.abs() <= check::MIN_COMPONENT && self.y.abs() <= check::MIN_COMPONENT && self.z.abs() <= check::MIN_COMPONENT {
            Err(check::invalid(format!("{what} must be non-zero")))
        } else {
            Ok(())
        }
    }
}

/// Tool pose in the part frame.
///
/// `tip` is the tool tip. `axis` points from the tip toward the spindle (or along the process
/// direction). It is stored as given; a unit vector is the convention.
///
/// `rotary` is the machine A/B angle in degrees when the program has a [`crate::RotarySetup`].
/// It may exceed 360 for multi-turn motion. In a turning program it is the C angle in degrees:
/// finite, and at most 99999 in absolute value. A pose with no rotary angle does not command
/// that axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Pose {
    tip: Vec3,
    axis: Vec3,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    rotary: Option<f64>,
}

impl Pose {
    pub fn new(tip: Vec3, axis: Vec3) -> Result<Self> {
        let pose = Self { tip, axis, rotary: None };
        pose.check()?;
        Ok(pose)
    }

    /// Tool tip, millimetres.
    pub fn tip(self) -> Vec3 {
        self.tip
    }

    /// Tool axis direction.
    pub fn axis(self) -> Vec3 {
        self.axis
    }

    /// Machine A/B angle in degrees, or the C angle in a turning program.
    pub fn rotary(self) -> Option<f64> {
        self.rotary
    }

    /// Set the machine A/B angle, or the C angle in a turning program, in degrees.
    ///
    /// The value must be finite. A turning program also requires `|c| <= 99999`.
    pub fn with_rotary(self, deg: f64) -> Result<Self> {
        let pose = Self { rotary: Some(deg), ..self };
        pose.check()?;
        Ok(pose)
    }

    pub(crate) fn from_parts(tip: Vec3, axis: Vec3, rotary: Option<f64>) -> Self {
        Self { tip, axis, rotary }
    }

    pub(crate) fn check(self) -> Result<()> {
        self.tip.check("tool tip")?;
        self.axis.check_non_zero("tool axis")?;
        if let Some(deg) = self.rotary {
            finite(deg, "rotary angle")?;
        }
        Ok(())
    }
}

/// Plane of a circular arc or helix, named by its two axes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Plane {
    /// XY, viewed from +Z. The usual flat work plane.
    Xy,
    /// ZX, viewed from +Y.
    Zx,
    /// YZ, viewed from +X.
    Yz,
}

/// Clockwise or counter-clockwise when looking along the plane normal (+Z for XY, +Y for ZX, +X for YZ).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ArcSense {
    /// Clockwise.
    Cw,
    /// Counter-clockwise.
    Ccw,
}
