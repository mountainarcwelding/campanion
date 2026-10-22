//! Shared numeric and text checks.

use crate::{IrError, Result};

pub(crate) const MAX_NAME: usize = 256;
pub(crate) const MAX_TEXT: usize = 4_096;
pub(crate) const MAX_PARAMS: usize = 64;
pub(crate) const MAX_TOOLS: usize = 1_024;
pub(crate) const MAX_TOOLPATHS: usize = 1_024;
pub(crate) const MAX_BLOCKS: usize = 65_536;
pub(crate) const MAX_SYNCS: usize = 4_096;
pub(crate) const MAX_JSON_BYTES: usize = 32 * 1024 * 1024;
/// Matches the cutter cap in `campanion-path`.
pub(crate) const MAX_CUTTER_MM: f64 = 1.0e6;
/// Below this absolute component, a direction is treated as zero.
pub(crate) const MIN_COMPONENT: f64 = 1e-15;
/// Millimetres. A gap at or below this still counts as the same point.
pub(crate) const CONTINUITY_MM: f64 = 1.0e-4;

/// Absolute distance between two finite numbers. Non-finite inputs yield infinity.
pub(crate) fn abs_gap(left: f64, right: f64) -> f64 {
    let gap = (left - right).abs();
    if gap.is_finite() { gap } else { f64::INFINITY }
}

pub(crate) fn invalid(message: impl Into<String>) -> IrError {
    IrError::InvalidInput(message.into())
}

pub(crate) fn finite(value: f64, what: &str) -> Result<f64> {
    if value.is_finite() { Ok(value) } else { Err(invalid(format!("{what} must be finite"))) }
}

pub(crate) fn non_negative(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value >= 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be >= 0"))) }
}

pub(crate) fn positive(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value > 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be positive"))) }
}

pub(crate) fn text_len(text: &str, max: usize, what: &str) -> Result<()> {
    if text.len() > max { Err(invalid(format!("{what} is longer than {max} bytes"))) } else { Ok(()) }
}

pub(crate) fn required_name(text: &str, what: &str) -> Result<()> {
    if text.is_empty() {
        return Err(invalid(format!("{what} is empty")));
    }
    text_len(text, MAX_NAME, what)
}

pub(crate) fn optional_name(text: &Option<String>, what: &str) -> Result<()> {
    if let Some(text) = text {
        required_name(text, what)?;
    }
    Ok(())
}
