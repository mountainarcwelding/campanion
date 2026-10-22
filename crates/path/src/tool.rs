//! Cutters and where compensation is applied.
//!
//! A round tool is described by its diameter. A kerf-only process (no rotating cutter) is
//! described by the kerf width. In both cases the computed offset distance is half of that value.

use crate::{PathError, Result};

/// Maximum accepted diameter or kerf width, in millimetres.
///
/// Larger values are rejected so a hostile job cannot ask for an unbounded offset.
const MAX_CUTTER_MM: f64 = 1.0e6;

/// A cutter. Compensation distance is half the diameter or half the kerf.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tool {
    /// Rotating tool. Offset by the radius (`diameter / 2`).
    Diameter { diameter: f64 },
    /// Kerf-only process. Offset by half the kerf (`width / 2`).
    Kerf { width: f64 },
}

impl Tool {
    /// Radius or half-kerf, in millimetres.
    pub fn compensation_distance(self) -> Result<f64> {
        let distance = match self {
            Self::Diameter { diameter } => half_positive(diameter, "tool diameter")?,
            Self::Kerf { width } => half_positive(width, "kerf width")?,
        };
        if distance.is_finite() { Ok(distance) } else { Err(PathError::InvalidInput("compensation distance is not finite".into())) }
    }
}

/// Which side of travel a later controller should keep the tool on.
///
/// This crate does not emit machine codes. `Left` is the flag a later IR maps to compensation
/// left of travel (G41). `Right` maps to compensation right of travel (G42).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControllerSide {
    /// Tool to the left of the programmed travel.
    Left,
    /// Tool to the right of the programmed travel.
    Right,
}

/// Where the tool radius or half-kerf is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Compensation {
    /// Offset the path in this crate by the tool radius or half the kerf.
    Computed,
    /// Leave the path on the contour. `side` is carried for a later IR.
    Controller(ControllerSide),
}

fn half_positive(value: f64, what: &str) -> Result<f64> {
    if !value.is_finite() {
        return Err(PathError::InvalidInput(format!("{what} must be finite")));
    }
    if value <= 0.0 {
        return Err(PathError::InvalidInput(format!("{what} must be positive")));
    }
    if value > MAX_CUTTER_MM {
        return Err(PathError::InvalidInput(format!("{what} is larger than {MAX_CUTTER_MM} mm")));
    }
    Ok(value * 0.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diameter_and_kerf_use_half() {
        let tool = Tool::Diameter { diameter: 2.0 };
        assert!((tool.compensation_distance().unwrap() - 1.0).abs() < 1e-12);
        let kerf = Tool::Kerf { width: 1.5 };
        assert!((kerf.compensation_distance().unwrap() - 0.75).abs() < 1e-12);
    }

    #[test]
    fn hostile_cutter_sizes_are_errors() {
        for tool in [Tool::Diameter { diameter: f64::NAN }, Tool::Diameter { diameter: f64::INFINITY }, Tool::Kerf { width: f64::NEG_INFINITY }] {
            assert!(tool.compensation_distance().is_err());
        }
        assert!(Tool::Diameter { diameter: 0.0 }.compensation_distance().is_err());
        assert!(Tool::Diameter { diameter: -2.0 }.compensation_distance().is_err());
        assert!(Tool::Kerf { width: -1.5 }.compensation_distance().is_err());
        assert!(Tool::Kerf { width: 0.0 }.compensation_distance().is_err());
        assert!(Tool::Diameter { diameter: 1.0e9 }.compensation_distance().is_err());
    }
}
