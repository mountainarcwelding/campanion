//! Flat-floor cusp height to XY stepover.
//!
//! Raster spacing is a plan (XY) distance. On a slope the real cusp left between two passes is
//! larger than the flat-floor value this module returns, because the tool axes stay that same
//! plan distance apart while the surface rises between them.

use crate::{Cutter, Result, invalid};

/// XY stepover that leaves a cusp of `scallop_height` between passes on a flat floor.
///
/// Distances are millimetres. `scallop_height` must be finite, greater than 0, and strictly
/// less than the corner radius. A ball's corner radius is its sphere radius `R`, and the
/// stepover is `2 * sqrt(2*R*h - h²)`. A bull nose of outer radius `R` and corner radius `c`
/// uses `2*(R - c) + 2*sqrt(2*c*h - h²)`. A flat end mill, and a bull nose whose corner radius
/// is 0, returns [`crate::MeshError::InvalidInput`] with the message
/// `scallop height needs a ball or bull-nose tool`.
///
/// This is the flat-floor value. Raster spacing is measured in XY, so on a slope the real
/// scallop is larger for the same stepover.
pub fn scallop_stepover(cutter: &Cutter, scallop_height: f64) -> Result<f64> {
    let Some((outer, corner)) = round_profile(cutter) else {
        return Err(invalid("scallop height needs a ball or bull-nose tool"));
    };
    if !scallop_height.is_finite() || scallop_height <= 0.0 || scallop_height >= corner {
        return Err(invalid("scallop height must be greater than 0 and less than the corner radius"));
    }
    let rise = 2.0 * corner * scallop_height - scallop_height * scallop_height;
    let step = 2.0 * (outer - corner) + 2.0 * rise.sqrt();
    if !step.is_finite() || step <= 0.0 {
        return Err(invalid("scallop stepover is not finite"));
    }
    Ok(step)
}

/// `(outer radius, corner radius)` for a ball or a bull nose with a positive corner.
///
/// A bull corner equal to the outer radius is the ball limit and is accepted. A zero corner is
/// the flat limit and is rejected by the caller.
fn round_profile(cutter: &Cutter) -> Option<(f64, f64)> {
    match *cutter {
        Cutter::Ball { radius } if radius.is_finite() && radius > 0.0 => Some((radius, radius)),
        Cutter::Bull { radius, corner_radius }
            if radius.is_finite() && radius > 0.0 && corner_radius.is_finite() && corner_radius > 0.0 && corner_radius <= radius =>
        {
            Some((radius, corner_radius))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MeshError;
    use proptest::prelude::*;

    fn ball(radius: f64) -> Cutter {
        Cutter::new_ball(radius).expect("ball")
    }

    fn bull(radius: f64, corner: f64) -> Cutter {
        Cutter::new_bull(radius, corner).expect("bull")
    }

    /// `h' = R - sqrt(R² - (s/2)²)`, the flat-floor cusp of a ball at stepover `s`.
    fn ball_cusp(radius: f64, stepover: f64) -> f64 {
        let half = stepover * 0.5;
        radius - (radius * radius - half * half).sqrt()
    }

    #[test]
    fn ball_closed_form_is_six_for_radius_five_and_height_one() {
        let step = scallop_stepover(&ball(5.0), 1.0).expect("step");
        assert!((step - 6.0).abs() < 1e-12, "{step}");
        assert!((ball_cusp(5.0, step) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn six_millimetre_ball_at_a_hundredth_of_a_millimetre() {
        let height = 0.01;
        let step = scallop_stepover(&ball(3.0), height).expect("step");
        let expected = 2.0 * (2.0 * 3.0 * height - height * height).sqrt();
        assert!((step - expected).abs() < 1e-12, "{step} != {expected}");
        assert!((ball_cusp(3.0, step) - height).abs() < 1e-9);
    }

    #[test]
    fn bull_closed_form_adds_the_flat_land_to_the_corner_cusp() {
        // R = 8, c = 5, h = 1: 2*(8-5) + 2*sqrt(2*5*1 - 1) = 6 + 2*3 = 12.
        let step = scallop_stepover(&bull(8.0, 5.0), 1.0).expect("step");
        assert!((step - 12.0).abs() < 1e-12, "{step}");
        let between_corners = step - 2.0 * (8.0 - 5.0);
        let recovered = 5.0 - (25.0 - (between_corners * 0.5).powi(2)).sqrt();
        assert!((recovered - 1.0).abs() < 1e-12, "{recovered}");

        let height = 0.5;
        let corner = 2.0;
        let outer = 6.0;
        let step = scallop_stepover(&bull(outer, corner), height).expect("step");
        let expected = 2.0 * (outer - corner) + 2.0 * (2.0 * corner * height - height * height).sqrt();
        assert!((step - expected).abs() < 1e-12, "{step} != {expected}");
    }

    #[test]
    fn bull_at_the_ball_limit_matches_a_ball() {
        let height = 0.2;
        let ball_step = scallop_stepover(&ball(4.0), height).expect("ball");
        let limit = Cutter::Bull { radius: 4.0, corner_radius: 4.0 };
        let bull_step = scallop_stepover(&limit, height).expect("bull limit");
        assert!((ball_step - bull_step).abs() < 1e-12);
    }

    #[test]
    fn flat_and_out_of_range_heights_are_rejected() {
        let flat = Cutter::new_flat(3.0).expect("flat");
        let err = scallop_stepover(&flat, 0.1).expect_err("flat");
        assert_eq!(err, MeshError::InvalidInput("scallop height needs a ball or bull-nose tool".into()));
        let zero_corner = Cutter::Bull { radius: 4.0, corner_radius: 0.0 };
        let err = scallop_stepover(&zero_corner, 0.1).expect_err("zero corner");
        assert_eq!(err, MeshError::InvalidInput("scallop height needs a ball or bull-nose tool".into()));

        let cutter = ball(3.0);
        for height in [0.0, -0.01, 3.0, 3.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let err = scallop_stepover(&cutter, height).expect_err("height");
            assert_eq!(err, MeshError::InvalidInput("scallop height must be greater than 0 and less than the corner radius".into()), "{height}");
        }
        let bull = bull(5.0, 1.5);
        for height in [1.5, 1.6, 0.0] {
            let err = scallop_stepover(&bull, height).expect_err("bull height");
            assert!(err.to_string().contains("corner radius"), "{err}");
        }
        assert!(scallop_stepover(&bull, 1.5 - 1e-9).is_ok());
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn ball_stepover_inverts_to_the_scallop_height(radius in 1e-3f64..1e3, fraction in 1e-6f64..0.95) {
            let height = radius * fraction;
            let step = scallop_stepover(&ball(radius), height).expect("step");
            let recovered = ball_cusp(radius, step);
            prop_assert!((recovered - height).abs() < 1e-9, "radius {radius} height {height} recovered {recovered} step {step}");
        }
    }
}
