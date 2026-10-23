//! Fixed-point numbers for G-code words.
//!
//! Output never uses scientific notation. A value that rounds to zero is written without a sign,
//! including a negative zero. [`format_dim`] keeps a decimal point on a whole number (`52.`, `0.`)
//! when a control would otherwise read the word as least-input increments.

use crate::error::{Result, invalid};

/// Largest absolute value written as a plain decimal.
const MAX_ABS: f64 = 1.0e9;

pub(crate) fn format_fixed(value: f64, decimals: u8, trailing_zeros: bool) -> Result<String> {
    if !value.is_finite() {
        return Err(invalid("number must be finite"));
    }
    if value.abs() > MAX_ABS {
        return Err(invalid("number is too large"));
    }
    let mut text = format!("{:.*}", usize::from(decimals), value);
    if let Some(rest) = text.strip_prefix('-')
        && rest.chars().all(|ch| ch == '0' || ch == '.')
    {
        text.remove(0);
    }
    if text.contains('e') || text.contains('E') {
        return Err(invalid("number is not a plain decimal"));
    }
    if !trailing_zeros && text.contains('.') {
        let trimmed = text.trim_end_matches('0').trim_end_matches('.');
        text = if trimmed.is_empty() || trimmed == "-" { "0".to_string() } else { trimmed.to_string() };
    }
    Ok(text)
}

/// Dimensional word (`X` `Y` `Z` `I` `J` `K` `R`, rotary `A`/`B`, lathe `X`/`Z`).
///
/// When `decimal_point` is set, a whole number keeps the point (`52.`, `0.`). The sign of a
/// value that rounded to zero was already dropped, so `-0.` is never written. Feed, spindle,
/// dwell, and line numbers stay on [`format_fixed`].
pub(crate) fn format_dim(value: f64, decimals: u8, trailing_zeros: bool, decimal_point: bool) -> Result<String> {
    let mut text = format_fixed(value, decimals, trailing_zeros)?;
    if decimal_point && !text.contains('.') {
        text.push('.');
    }
    Ok(text)
}

/// True when `a` and `b` write the same `X` and `Y` at this post's linear precision.
///
/// Uses the same text as the axis words, including a trailing decimal point when the post
/// requires one.
pub(crate) fn xy_text_eq(a: [f64; 3], b: [f64; 3], decimals: u8, trailing_zeros: bool, decimal_point: bool) -> Result<bool> {
    let same = |left: f64, right: f64| -> Result<bool> {
        Ok(format_dim(left, decimals, trailing_zeros, decimal_point)? == format_dim(right, decimals, trailing_zeros, decimal_point)?)
    };
    Ok(same(a[0], b[0])? && same(a[1], b[1])?)
}

#[cfg(test)]
mod tests {
    use super::{format_dim, format_fixed, xy_text_eq};

    #[test]
    fn rounds_and_strips() {
        assert_eq!(format_fixed(1.23456, 4, true).unwrap(), "1.2346");
        assert_eq!(format_fixed(-1.2, 4, true).unwrap(), "-1.2000");
        assert_eq!(format_fixed(-1.2, 4, false).unwrap(), "-1.2");
        assert_eq!(format_fixed(10.0, 3, false).unwrap(), "10");
        assert_eq!(format_fixed(-0.00004, 4, true).unwrap(), "0.0000");
        assert_eq!(format_fixed(-0.0, 4, false).unwrap(), "0");
        assert_eq!(format_fixed(0.6, 0, true).unwrap(), "1");
    }

    #[test]
    fn dimensional_words_keep_a_decimal_point() {
        assert_eq!(format_dim(52.0, 3, false, true).unwrap(), "52.");
        assert_eq!(format_dim(0.0, 3, false, true).unwrap(), "0.");
        assert_eq!(format_dim(-0.0, 3, false, true).unwrap(), "0.");
        assert_eq!(format_dim(-0.00004, 4, false, true).unwrap(), "0.");
        assert_eq!(format_dim(-52.0, 3, false, true).unwrap(), "-52.");
        assert_eq!(format_dim(52.5, 3, false, true).unwrap(), "52.5");
        assert_eq!(format_dim(52.0, 3, true, true).unwrap(), "52.000");
        assert_eq!(format_dim(0.0, 3, true, true).unwrap(), "0.000");
        assert_eq!(format_dim(52.0, 3, false, false).unwrap(), "52");
        assert_eq!(format_dim(0.6, 0, true, true).unwrap(), "1.");
        assert!(!format_dim(-0.0, 4, false, true).unwrap().starts_with('-'));
    }

    #[test]
    fn text_equality_follows_the_dimensional_formatter() {
        assert!(xy_text_eq([10.0, 0.0, 0.0], [10.0004, 0.0, 0.0], 3, false, true).unwrap());
        assert!(xy_text_eq([10.0, 0.0, 0.0], [10.0004, 0.0, 0.0], 3, false, false).unwrap());
        assert!(!xy_text_eq([10.0, 0.0, 0.0], [10.1, 0.0, 0.0], 3, false, true).unwrap());
        assert!(!xy_text_eq([52.0, 1.0, 0.0], [52.0, 2.0, 0.0], 3, false, true).unwrap());
    }
}
