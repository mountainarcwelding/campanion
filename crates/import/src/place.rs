//! Rigid placement of an imported drawing: rotate about the bounds centre, then translate.

use campanion_geom::{Arc, Contour, Line, Point, Segment, Tolerance};

use crate::{Drawing, ImportedContour, Result, invalid};

/// Rotate `drawing` about its bounding-box centre by `rotate_deg` (counter-clockwise),
/// then translate by `(x, y)` millimetres. A zero placement returns a clone.
pub fn place_drawing(drawing: &Drawing, x: f64, y: f64, rotate_deg: f64, tolerance: &Tolerance) -> Result<Drawing> {
    if !x.is_finite() || !y.is_finite() || !rotate_deg.is_finite() {
        return Err(invalid("placement must be finite"));
    }
    if x == 0.0 && y == 0.0 && rotate_deg == 0.0 {
        return Ok(drawing.clone());
    }
    let theta = rotate_deg.to_radians();
    let (sin, cos) = (theta.sin(), theta.cos());
    let (cx, cy) = bounds_centre(drawing).unwrap_or((0.0, 0.0));
    let mut contours = Vec::with_capacity(drawing.contours().len());
    for item in drawing.contours() {
        let mut segments = Vec::with_capacity(item.contour().segments().len());
        for segment in item.contour().segments() {
            segments.push(map_segment(segment, cx, cy, cos, sin, theta, x, y)?);
        }
        let contour = Contour::try_new(segments, item.contour().is_closed(), tolerance).map_err(crate::ImportError::from_geom)?;
        contours.push(ImportedContour::new(item.layer().to_string(), contour));
    }
    Ok(Drawing::new(contours, drawing.skipped_entities(), drawing.skipped_types().to_vec()).with_warnings(drawing.warnings().to_vec()))
}

fn bounds_centre(drawing: &Drawing) -> Option<(f64, f64)> {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for item in drawing.contours() {
        let Ok(bounds) = item.contour().bounding_box() else {
            continue;
        };
        min_x = min_x.min(bounds.min().x());
        min_y = min_y.min(bounds.min().y());
        max_x = max_x.max(bounds.max().x());
        max_y = max_y.max(bounds.max().y());
    }
    if min_x.is_finite() && max_x.is_finite() { Some(((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)) } else { None }
}

fn map_segment(segment: &Segment, cx: f64, cy: f64, cos: f64, sin: f64, theta: f64, x: f64, y: f64) -> Result<Segment> {
    match segment {
        Segment::Line(line) => {
            let start = map_point(line.start(), cx, cy, cos, sin, x, y)?;
            let end = map_point(line.end(), cx, cy, cos, sin, x, y)?;
            Line::new(start, end).map(Segment::Line).map_err(crate::ImportError::from_geom)
        }
        Segment::Arc(arc) => {
            let center = map_point(arc.center(), cx, cy, cos, sin, x, y)?;
            Arc::new(center, arc.radius(), arc.start_angle() + theta, arc.sweep()).map(Segment::Arc).map_err(crate::ImportError::from_geom)
        }
    }
}

fn map_point(point: Point, cx: f64, cy: f64, cos: f64, sin: f64, x: f64, y: f64) -> Result<Point> {
    let dx = point.x() - cx;
    let dy = point.y() - cy;
    Point::new(cx + dx * cos - dy * sin + x, cy + dx * sin + dy * cos + y).map_err(crate::ImportError::from_geom)
}

#[cfg(test)]
mod tests {
    use campanion_geom::{Contour, Line, Point, Segment, Tolerance};

    use super::place_drawing;
    use crate::{Drawing, ImportedContour};

    fn square() -> Drawing {
        let tol = Tolerance::default();
        let points = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)];
        let mut segments = Vec::new();
        for window in points.windows(2) {
            let start = Point::new(window[0].0, window[0].1).expect("pt");
            let end = Point::new(window[1].0, window[1].1).expect("pt");
            segments.push(Segment::Line(Line::new(start, end).expect("line")));
        }
        let contour = Contour::try_new(segments, true, &tol).expect("contour");
        Drawing::new(vec![ImportedContour::new("0", contour)], 0, Vec::new())
    }

    #[test]
    fn translate_moves_bounds() {
        let placed = place_drawing(&square(), 5.0, -2.0, 0.0, &Tolerance::default()).expect("place");
        let bounds = placed.contours()[0].contour().bounding_box().expect("bounds");
        assert!((bounds.min().x() - 5.0).abs() < 1e-9);
        assert!((bounds.min().y() + 2.0).abs() < 1e-9);
        assert!((bounds.max().x() - 15.0).abs() < 1e-9);
        assert!((bounds.max().y() - 8.0).abs() < 1e-9);
    }

    #[test]
    fn rotate_180_about_centre_then_translate() {
        // Centre is (5, 5). 180° leaves the square in place, then +1,+1.
        let placed = place_drawing(&square(), 1.0, 1.0, 180.0, &Tolerance::default()).expect("place");
        let bounds = placed.contours()[0].contour().bounding_box().expect("bounds");
        assert!((bounds.min().x() - 1.0).abs() < 1e-9, "{}", bounds.min().x());
        assert!((bounds.min().y() - 1.0).abs() < 1e-9, "{}", bounds.min().y());
        assert!((bounds.max().x() - 11.0).abs() < 1e-9);
        assert!((bounds.max().y() - 11.0).abs() < 1e-9);
    }

    #[test]
    fn identity_is_a_clone() {
        let drawing = square();
        let placed = place_drawing(&drawing, 0.0, 0.0, 0.0, &Tolerance::default()).expect("id");
        assert_eq!(placed, drawing);
    }
}
