//! Cutters and where compensation is applied.
//!
//! A round tool is described by its diameter. A kerf-only process (no rotating cutter) is
//! described by the kerf width. In both cases the computed offset distance is half of that value.
//! A diameter tool may name a router cutter sense. That value is data for the job and a later
//! verifier; compensation stays half the diameter.

use crate::{PathError, Result};

/// Maximum accepted diameter or kerf width, in millimetres.
///
/// Larger values are rejected so a hostile job cannot ask for an unbounded offset.
const MAX_CUTTER_MM: f64 = 1.0e6;

/// Helix sense of a rotating router cutter.
///
/// `Compression` also needs [`Tool::Diameter`]'s `upcut_length`: millimetres of upcut at the tip.
/// Without stickout, that length may be any positive finite value at most `diameter * 100`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutterSense {
    /// Chips climb the flute toward the shank.
    Upcut,
    /// Chips are pushed toward the tip.
    Downcut,
    /// Upcut at the tip, downcut above that.
    Compression,
}

/// A cutter. Compensation distance is half the diameter or half the kerf.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tool {
    /// Rotating tool. Offset by the radius (`diameter / 2`).
    ///
    /// `cutter_sense` is omitted for the flat diameter offset used when the job does not name a
    /// sense. `upcut_length` is millimetres at the tip and is only set with
    /// [`CutterSense::Compression`].
    Diameter {
        /// Cutting diameter, millimetres.
        diameter: f64,
        /// Router flute helix. `None` keeps today's diameter offset.
        cutter_sense: Option<CutterSense>,
        /// Upcut length at the tip, millimetres. Set only with [`CutterSense::Compression`].
        upcut_length: Option<f64>,
    },
    /// Kerf-only process. Offset by half the kerf (`width / 2`).
    Kerf { width: f64 },
}

impl Tool {
    /// Rotating cutter with no cutter sense. Compensation is half of `diameter`.
    pub fn diameter(diameter: f64) -> Self {
        Self::Diameter { diameter, cutter_sense: None, upcut_length: None }
    }

    /// Kerf-only process. Compensation is half of `width`.
    pub fn kerf(width: f64) -> Self {
        Self::Kerf { width }
    }

    /// Store cutter sense on a diameter tool and re-validate.
    ///
    /// A kerf tool cannot carry a sense. [`CutterSense::Compression`] needs a positive finite
    /// `upcut_length` at most `diameter * 100`. Other senses reject `upcut_length`.
    pub fn with_cutter_sense(self, cutter_sense: Option<CutterSense>, upcut_length: Option<f64>) -> Result<Self> {
        match self {
            Self::Kerf { .. } => {
                if cutter_sense.is_some() || upcut_length.is_some() {
                    return Err(PathError::InvalidInput("cutter_sense is not valid on a kerf tool".into()));
                }
                Ok(self)
            }
            Self::Diameter { diameter, .. } => {
                check_cutter_sense(Some(diameter), None, None, cutter_sense, upcut_length)?;
                Ok(Self::Diameter { diameter, cutter_sense, upcut_length })
            }
        }
    }

    /// Helix sense. `None` on a kerf tool and on a diameter tool that omits it.
    pub fn cutter_sense(self) -> Option<CutterSense> {
        match self {
            Self::Diameter { cutter_sense, .. } => cutter_sense,
            Self::Kerf { .. } => None,
        }
    }

    /// Upcut length at the tip, millimetres. `None` unless the sense is compression.
    pub fn upcut_length(self) -> Option<f64> {
        match self {
            Self::Diameter { upcut_length, .. } => upcut_length,
            Self::Kerf { .. } => None,
        }
    }

    /// Radius or half-kerf, in millimetres.
    ///
    /// Cutter sense does not change this distance.
    pub fn compensation_distance(self) -> Result<f64> {
        let distance = match self {
            Self::Diameter { diameter, .. } => half_positive(diameter, "tool diameter")?,
            Self::Kerf { width } => half_positive(width, "kerf width")?,
        };
        if distance.is_finite() { Ok(distance) } else { Err(PathError::InvalidInput("compensation distance is not finite".into())) }
    }
}

/// Check helix sense and, for compression, the upcut length at the tip.
///
/// `stickout` and `flute_length` are millimetres when the tool names an assembly. The length must
/// be positive, finite, and at most those values when they exist. Without stickout, any positive
/// finite value at most `diameter * 100` is accepted.
pub fn check_cutter_sense(
    diameter: Option<f64>,
    stickout: Option<f64>,
    flute_length: Option<f64>,
    cutter_sense: Option<CutterSense>,
    upcut_length: Option<f64>,
) -> Result<()> {
    if cutter_sense.is_none() && upcut_length.is_none() {
        return Ok(());
    }
    let Some(diameter) = diameter else {
        return Err(PathError::InvalidInput("cutter_sense is not valid on a kerf tool".into()));
    };
    match cutter_sense {
        None | Some(CutterSense::Upcut) | Some(CutterSense::Downcut) => {
            if upcut_length.is_some() {
                return Err(PathError::InvalidInput("upcut_length is only valid with compression".into()));
            }
            Ok(())
        }
        Some(CutterSense::Compression) => {
            let Some(length) = upcut_length else {
                return Err(PathError::InvalidInput("compression needs upcut_length".into()));
            };
            if !length.is_finite() {
                return Err(PathError::InvalidInput("upcut_length must be finite".into()));
            }
            if length <= 0.0 {
                return Err(PathError::InvalidInput("upcut_length must be positive".into()));
            }
            let (cap, what) = upcut_cap(diameter, stickout, flute_length);
            if length > cap {
                return Err(PathError::InvalidInput(format!("upcut_length must be at most {what}")));
            }
            Ok(())
        }
    }
}

fn upcut_cap(diameter: f64, stickout: Option<f64>, flute_length: Option<f64>) -> (f64, &'static str) {
    match (stickout, flute_length) {
        (Some(stickout), Some(flute)) if flute < stickout => (flute, "flute_length"),
        (Some(stickout), _) => (stickout, "stickout"),
        (None, Some(flute)) => (flute, "flute_length"),
        (None, None) => (diameter * 100.0, "diameter * 100"),
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
    use proptest::prelude::*;

    #[test]
    fn diameter_and_kerf_use_half() {
        let tool = Tool::diameter(2.0);
        assert!((tool.compensation_distance().unwrap() - 1.0).abs() < 1e-12);
        let kerf = Tool::kerf(1.5);
        assert!((kerf.compensation_distance().unwrap() - 0.75).abs() < 1e-12);
    }

    #[test]
    fn cutter_sense_does_not_change_compensation() {
        let bare = Tool::diameter(6.0);
        let upcut = bare.with_cutter_sense(Some(CutterSense::Upcut), None).unwrap();
        let downcut = bare.with_cutter_sense(Some(CutterSense::Downcut), None).unwrap();
        let compression = bare.with_cutter_sense(Some(CutterSense::Compression), Some(12.0)).unwrap();
        let expected = bare.compensation_distance().unwrap();
        for tool in [upcut, downcut, compression] {
            assert!((tool.compensation_distance().unwrap() - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn cutter_sense_rejects_illegal_upcut_length() {
        let mill = Tool::diameter(6.0);
        assert!(mill.with_cutter_sense(Some(CutterSense::Upcut), Some(1.0)).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Downcut), Some(1.0)).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Compression), None).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Compression), Some(0.0)).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Compression), Some(-1.0)).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Compression), Some(f64::NAN)).is_err());
        assert!(mill.with_cutter_sense(Some(CutterSense::Compression), Some(601.0)).is_err());
        assert!(mill.with_cutter_sense(None, Some(1.0)).is_err());
        assert!(Tool::kerf(1.2).with_cutter_sense(Some(CutterSense::Upcut), None).is_err());
        assert!(check_cutter_sense(Some(6.0), Some(20.0), Some(10.0), Some(CutterSense::Compression), Some(12.0)).is_err());
        assert!(check_cutter_sense(Some(6.0), Some(20.0), Some(10.0), Some(CutterSense::Compression), Some(10.0)).is_ok());
        assert!(check_cutter_sense(Some(6.0), Some(20.0), None, Some(CutterSense::Compression), Some(20.0)).is_ok());
        assert!(check_cutter_sense(Some(6.0), Some(20.0), None, Some(CutterSense::Compression), Some(20.1)).is_err());
    }

    #[test]
    fn hostile_cutter_sizes_are_errors() {
        for tool in [Tool::diameter(f64::NAN), Tool::diameter(f64::INFINITY), Tool::kerf(f64::NEG_INFINITY)] {
            assert!(tool.compensation_distance().is_err());
        }
        assert!(Tool::diameter(0.0).compensation_distance().is_err());
        assert!(Tool::diameter(-2.0).compensation_distance().is_err());
        assert!(Tool::kerf(-1.5).compensation_distance().is_err());
        assert!(Tool::kerf(0.0).compensation_distance().is_err());
        assert!(Tool::diameter(1.0e9).compensation_distance().is_err());
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 24,
            rng_seed: proptest::test_runner::RngSeed::Fixed(0xC077_E5E1),
            ..ProptestConfig::default()
        })]
        #[test]
        fn cutter_sense_keeps_compensation_distance(diameter in 0.5f64..100.0, length in 0.1f64..50.0) {
            prop_assume!(length <= diameter * 100.0);
            let bare = Tool::diameter(diameter);
            let Ok(expected) = bare.compensation_distance() else {
                return Ok(());
            };
            let upcut = bare.with_cutter_sense(Some(CutterSense::Upcut), None).expect("upcut");
            let compression = bare.with_cutter_sense(Some(CutterSense::Compression), Some(length)).expect("compression");
            prop_assert!((upcut.compensation_distance().expect("upcut radius") - expected).abs() < 1e-12);
            prop_assert!((compression.compensation_distance().expect("compression radius") - expected).abs() < 1e-12);
        }
    }
}
