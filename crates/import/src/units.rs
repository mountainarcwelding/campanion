//! `$INSUNITS` to millimetres. Values follow the public DXF header table.

use crate::{ImportError, Result, unsupported};

/// Millimetres per drawing unit.
///
/// Code 0 (unitless) and a missing variable are millimetres: this crate stores geometry in
/// millimetres and has no other default.
pub(crate) fn mm_per_unit(code: i32) -> Result<f64> {
    let scale: f64 = match code {
        0 | 4 => 1.0,                   // unitless, millimetres
        1 => 25.4,                      // inches
        2 => 304.8,                     // feet
        3 => 1_609_344.0,               // miles
        5 => 10.0,                      // centimetres
        6 => 1_000.0,                   // metres
        7 => 1_000_000.0,               // kilometres
        8 => 25.4e-6,                   // microinches
        9 => 0.0254,                    // mils
        10 => 914.4,                    // yards
        11 => 1e-7,                     // angstroms
        12 => 1e-6,                     // nanometres
        13 => 1e-3,                     // microns
        14 => 100.0,                    // decimetres
        15 => 10_000.0,                 // decametres
        16 => 100_000.0,                // hectometres
        17 => 1e12,                     // gigametres
        18 => 1.495978707e14,           // astronomical units
        19 => 9.4607304725808e18,       // light years
        20 => 3.085_677_581_491_367e19, // parsecs
        other => return Err(unsupported(format!("$INSUNITS code {other}"))),
    };
    if scale.is_finite() && scale > 0.0 { Ok(scale) } else { Err(ImportError::InvalidInput("unit scale is not finite".into())) }
}

#[cfg(test)]
mod tests {
    use super::mm_per_unit;

    #[test]
    fn known_codes_are_positive_and_millimetres_stay_one() {
        assert_eq!(mm_per_unit(0).unwrap(), 1.0);
        assert_eq!(mm_per_unit(4).unwrap(), 1.0);
        assert_eq!(mm_per_unit(1).unwrap(), 25.4);
        assert_eq!(mm_per_unit(5).unwrap(), 10.0);
        assert_eq!(mm_per_unit(6).unwrap(), 1_000.0);
        assert!((mm_per_unit(9).unwrap() - 0.0254).abs() < 1e-15);
        assert!(mm_per_unit(21).is_err());
        assert!(mm_per_unit(-1).is_err());
    }
}
