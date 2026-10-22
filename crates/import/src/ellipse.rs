//! Ellipse to true arcs or to chords within the chord tolerance.
//!
//! Not wired into import yet (`ELLIPSE` entities are skipped). The math stays here for that pass.
//!
//! `P(t) = center + major cos t + minor sin t`. The parameter increases from `start` by `sweep`.
//! `|P''|` is at most the major-axis length, so a uniform step
//! `dt = sqrt(8 * tol / (a * sqrt(2)))` keeps the chord error inside the tolerance.

use campanion_geom::{Arc, Contour, Line, Point, Polyline, Segment, Tolerance, Vec2, Vertex};

use crate::{ImportError, MAX_ENTITY_SEGMENTS, Result, invalid};

const TAU: f64 = std::f64::consts::TAU;
const SQRT_2: f64 = std::f64::consts::SQRT_2;

pub(crate) fn approximate(center: Point, major: Vec2, minor: Vec2, start: f64, sweep: f64, tolerance: &Tolerance) -> Result<(Vec<Segment>, bool)> {
    if !start.is_finite() || !sweep.is_finite() || sweep <= 0.0 {
        return Err(invalid("ellipse parameter span is empty"));
    }
    let a = major.length();
    let b = minor.length();
    if !a.is_finite() || !b.is_finite() || a <= 0.0 || b <= 0.0 {
        return Err(invalid("ellipse axes must be positive"));
    }
    if a <= tolerance.linear() {
        return Err(invalid("ellipse is smaller than the linear tolerance"));
    }
    let det = major.x() * minor.y() - major.y() * minor.x();
    if !det.is_finite() || det.abs() <= 1e-12 * a * a {
        return Err(invalid("ellipse axes are parallel"));
    }
    let closed = sweep >= TAU - 1e-8;
    let radial = (a - b).abs();
    if radial <= tolerance.chord() {
        return circular(center, a, major, det, start, sweep, closed, tolerance);
    }
    chords(center, major, minor, a, start, sweep, closed, tolerance)
}

fn circular(
    center: Point,
    radius: f64,
    major: Vec2,
    det: f64,
    start: f64,
    sweep: f64,
    closed: bool,
    tolerance: &Tolerance,
) -> Result<(Vec<Segment>, bool)> {
    if closed {
        let east = Point::new(center.x() + radius, center.y()).map_err(ImportError::from_geom)?;
        let west = Point::new(center.x() - radius, center.y()).map_err(ImportError::from_geom)?;
        let poly = Polyline::new(
            vec![Vertex::new(east, 1.0).map_err(ImportError::from_geom)?, Vertex::new(west, 1.0).map_err(ImportError::from_geom)?],
            true,
        )
        .map_err(ImportError::from_geom)?;
        let contour = poly.to_contour(tolerance).map_err(ImportError::from_geom)?;
        return Ok((contour.segments().to_vec(), true));
    }
    let start_pt = ellipse_point(center, major, minor_for_circle(major, radius, det)?, start)?;
    let start_angle = (start_pt - center).angle();
    let signed = if det > 0.0 { sweep } else { -sweep };
    let arc = Arc::new(center, radius, start_angle, signed).map_err(ImportError::from_geom)?;
    Ok((vec![Segment::Arc(arc)], false))
}

/// Minor axis of a circle of `radius` whose orientation matches `det`.
fn minor_for_circle(major: Vec2, radius: f64, det: f64) -> Result<Vec2> {
    let len = major.length();
    if len <= 0.0 || !len.is_finite() {
        return Err(invalid("ellipse major axis is zero"));
    }
    let sign = if det > 0.0 { 1.0 } else { -1.0 };
    // +90° rotation of the major axis, scaled to `radius`.
    Vec2::new(sign * -major.y() / len * radius, sign * major.x() / len * radius).map_err(ImportError::from_geom)
}

fn chords(
    center: Point,
    major: Vec2,
    minor: Vec2,
    a: f64,
    start: f64,
    sweep: f64,
    closed: bool,
    tolerance: &Tolerance,
) -> Result<(Vec<Segment>, bool)> {
    let chord = tolerance.chord();
    if !(chord.is_finite()) || chord < 0.0 {
        return Err(invalid("chord tolerance must be finite and >= 0"));
    }
    if chord == 0.0 {
        return Err(invalid("ellipse needs a positive chord tolerance"));
    }
    let dt = (8.0 * chord / (a * SQRT_2)).sqrt();
    if !dt.is_finite() || dt <= 0.0 {
        return Err(invalid("ellipse step is not finite"));
    }
    let mut steps = (sweep / dt).ceil();
    if !steps.is_finite() {
        return Err(invalid("ellipse step count is not finite"));
    }
    if steps < 1.0 {
        steps = 1.0;
    }
    if closed && steps < 4.0 {
        steps = 4.0;
    }
    if steps > MAX_ENTITY_SEGMENTS as f64 {
        return Err(invalid(format!(
            "ellipse approximation needs {steps} segments at the chord tolerance, above the limit of {MAX_ENTITY_SEGMENTS}"
        )));
    }
    let steps = steps as usize;
    let start_pt = ellipse_point(center, major, minor, start)?;
    let mut segments = Vec::with_capacity(steps);
    for i in 0..steps {
        let from = if i == 0 { start_pt } else { ellipse_point(center, major, minor, start + sweep * (i as f64 / steps as f64))? };
        let to =
            if closed && i + 1 == steps { start_pt } else { ellipse_point(center, major, minor, start + sweep * ((i as f64 + 1.0) / steps as f64))? };
        if from.distance(to) <= tolerance.linear() {
            continue;
        }
        let line = Line::new(from, to).map_err(ImportError::from_geom)?;
        segments.push(Segment::Line(line));
    }
    if segments.is_empty() {
        return Err(invalid("ellipse approximation is empty"));
    }
    if closed {
        // The last chord ends on the start point, so the loop meets exactly.
        let contour = Contour::try_new(segments.clone(), true, tolerance).map_err(ImportError::from_geom)?;
        Ok((contour.segments().to_vec(), true))
    } else {
        Ok((segments, false))
    }
}

fn ellipse_point(center: Point, major: Vec2, minor: Vec2, t: f64) -> Result<Point> {
    let (sin_t, cos_t) = t.sin_cos();
    Point::new(center.x() + major.x() * cos_t + minor.x() * sin_t, center.y() + major.y() * cos_t + minor.y() * sin_t).map_err(ImportError::from_geom)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_aligned_ellipse_samples_stay_inside_the_chord_tolerance() {
        let tol = Tolerance::with_chord(1e-6, 1e-6, 0.05).unwrap();
        let (segs, closed) =
            approximate(Point::new(0.0, 0.0).unwrap(), Vec2::new(10.0, 0.0).unwrap(), Vec2::new(0.0, 5.0).unwrap(), 0.0, TAU, &tol).unwrap();
        assert!(closed);
        let contour = Contour::try_new(segs, true, &tol).unwrap();
        for i in 0..64 {
            let t = TAU * (i as f64) / 64.0;
            let p = Point::new(10.0 * t.cos(), 5.0 * t.sin()).unwrap();
            let distance = contour.segments().iter().map(|seg| seg.distance_to_point(p)).fold(f64::MAX, f64::min);
            assert!(distance <= 0.05 + 1e-6, "t={t} distance={distance}");
        }
    }
}
