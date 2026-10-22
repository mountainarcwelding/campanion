//! Numeric helpers shared inside the crate.

use crate::{GeomError, Result};

pub(crate) const PI: f64 = std::f64::consts::PI;
pub(crate) const TAU: f64 = std::f64::consts::TAU;

/// Bulges smaller than this are straight segments. `tan(sweep/4)` for a sweep of about 4e-15.
pub(crate) const BULGE_LINE_EPS: f64 = 1e-15;

pub(crate) fn finite(value: f64, what: &str) -> Result<f64> {
    if value.is_finite() { Ok(value) } else { Err(GeomError::InvalidInput(format!("{what} must be finite"))) }
}

/// Wrap to `[0, τ)`.
pub(crate) fn wrap_tau(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(TAU);
    if wrapped >= TAU { 0.0 } else { wrapped }
}

/// Signed smallest difference `to - from`, in `(-π, π]`.
pub(crate) fn angle_diff(from: f64, to: f64) -> f64 {
    let mut delta = (to - from).rem_euclid(TAU);
    if delta > PI {
        delta -= TAU;
    }
    delta
}

pub(crate) fn check_segment_count(count: usize) -> Result<()> {
    if count > crate::MAX_SEGMENTS { Err(GeomError::InvalidInput(format!("too many segments: {count} > {}", crate::MAX_SEGMENTS))) } else { Ok(()) }
}
