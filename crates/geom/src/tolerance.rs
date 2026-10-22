//! One tolerance model for every geometry operation.

use crate::Result;
use crate::util::finite;

/// Linear, angular, and chord tolerances.
///
/// Linear and chord values are millimetres. Angular values are radians.
/// Defaults are 1e-6 mm, 1e-6 rad, and 1e-6 mm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerance {
    linear: f64,
    angular: f64,
    chord: f64,
}

impl Tolerance {
    pub const DEFAULT_LINEAR_MM: f64 = 1e-6;
    pub const DEFAULT_ANGULAR_RAD: f64 = 1e-6;
    pub const DEFAULT_CHORD_MM: f64 = 1e-6;

    pub fn new(linear: f64, angular: f64) -> Result<Self> {
        Self::with_chord(linear, angular, linear)
    }

    pub fn with_chord(linear: f64, angular: f64, chord: f64) -> Result<Self> {
        let linear = finite(linear, "linear tolerance")?;
        let angular = finite(angular, "angular tolerance")?;
        let chord = finite(chord, "chord tolerance")?;
        if linear < 0.0 || angular < 0.0 || chord < 0.0 {
            return Err(crate::GeomError::InvalidInput("tolerance must be >= 0".into()));
        }
        Ok(Self { linear, angular, chord })
    }

    pub fn linear(self) -> f64 {
        self.linear
    }

    pub fn angular(self) -> f64 {
        self.angular
    }

    pub fn chord(self) -> f64 {
        self.chord
    }
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { linear: Self::DEFAULT_LINEAR_MM, angular: Self::DEFAULT_ANGULAR_RAD, chord: Self::DEFAULT_CHORD_MM }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_one_micrometer_and_one_microradian() {
        let tol = Tolerance::default();
        assert_eq!(tol.linear(), 1e-6);
        assert_eq!(tol.angular(), 1e-6);
        assert_eq!(tol.chord(), 1e-6);
    }

    #[test]
    fn rejects_negative_and_non_finite() {
        assert!(Tolerance::new(-1.0, 1e-6).is_err());
        assert!(Tolerance::new(1e-6, f64::NAN).is_err());
        assert!(Tolerance::with_chord(1e-6, 1e-6, f64::INFINITY).is_err());
    }
}
