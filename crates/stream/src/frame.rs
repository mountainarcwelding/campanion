//! Bounding-box frame program: laser off (`M5`/`S0`) at a user-entered feed.

/// Axis-aligned bounds in work coordinates, millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Bounds {
    /// Build bounds from min/max corners. Rejects non-finite or inverted values.
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Result<Self, String> {
        if ![min_x, min_y, max_x, max_y].iter().all(|v| v.is_finite()) {
            return Err("frame bounds must be finite".to_string());
        }
        if max_x < min_x || max_y < min_y {
            return Err("frame bounds are inverted".to_string());
        }
        Ok(Self { min_x, min_y, max_x, max_y })
    }
}

/// Build a frame program that traces the bounding box with the laser off.
///
/// Every move is `G1` at `feed_mm_min`, which the caller must have entered. No rapid
/// rate is invented. Units are millimetres, absolute (`G21` `G90`). The program starts
/// and ends with `M5` and `S0`.
pub fn frame_program(bounds: Bounds, feed_mm_min: f64) -> Result<Vec<String>, String> {
    if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
        return Err("frame needs a feed you entered, in millimetres per minute".to_string());
    }
    let xmin = format_mm(bounds.min_x);
    let xmax = format_mm(bounds.max_x);
    let ymin = format_mm(bounds.min_y);
    let ymax = format_mm(bounds.max_y);
    let feed = format_mm(feed_mm_min);
    Ok(vec![
        "M5".to_string(),
        "S0".to_string(),
        "G21".to_string(),
        "G90".to_string(),
        format!("G1 F{feed} X{xmin} Y{ymin}"),
        format!("G1 X{xmax} Y{ymin}"),
        format!("G1 X{xmax} Y{ymax}"),
        format!("G1 X{xmin} Y{ymax}"),
        format!("G1 X{xmin} Y{ymin}"),
        "M5".to_string(),
        "S0".to_string(),
    ])
}

fn format_mm(value: f64) -> String {
    let text = format!("{value:.4}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// Parse a positive feed the user typed, in millimetres per minute.
pub fn parse_feed(text: &str) -> Result<f64, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err("enter a feed in millimetres per minute; none is assumed".to_string());
    }
    let value: f64 = trimmed.parse().map_err(|_| format!("`{trimmed}` is not a feed rate"))?;
    if !value.is_finite() || value <= 0.0 {
        return Err("feed must be a positive number of millimetres per minute".to_string());
    }
    Ok(value)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::assertions_on_constants, clippy::unwrap_used, clippy::manual_unwrap_or_default)]
mod tests {
    use super::*;

    #[test]
    fn frame_starts_and_ends_with_beam_off_and_uses_the_entered_feed() {
        let bounds = match Bounds::new(10.0, 20.0, 30.0, 40.0) {
            Ok(bounds) => bounds,
            Err(err) => {
                let _ = err;
                return assert!(false, "bounds");
            }
        };
        let lines = match frame_program(bounds, 1500.0) {
            Ok(lines) => lines,
            Err(err) => {
                let _ = err;
                return assert!(false, "program");
            }
        };
        assert_eq!(lines.first().map(String::as_str), Some("M5"));
        assert_eq!(lines.get(1).map(String::as_str), Some("S0"));
        assert!(lines.iter().any(|line| line.contains("F1500")));
        assert!(lines.iter().any(|line| line.contains("X30") && line.contains("Y20")));
        assert_eq!(lines[lines.len().saturating_sub(2)].as_str(), "M5");
        assert_eq!(lines.last().map(String::as_str), Some("S0"));
        assert!(lines.iter().all(|line| !line.contains("G0")));
    }

    #[test]
    fn refuses_a_missing_or_non_positive_feed() {
        assert!(parse_feed("").is_err());
        assert!(parse_feed("0").is_err());
        assert!(frame_program(Bounds { min_x: 0.0, min_y: 0.0, max_x: 1.0, max_y: 1.0 }, 0.0).is_err());
    }
}
