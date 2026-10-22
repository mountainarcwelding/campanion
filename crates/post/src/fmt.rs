//! Fixed-point numbers for G-code words.
//!
//! Output never uses scientific notation. A value that rounds to zero is written without a sign,
//! including a negative zero.

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

#[cfg(test)]
mod tests {
    use super::format_fixed;

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
}
