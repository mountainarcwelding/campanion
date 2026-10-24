//! Face (XY) and cylinder (unrolled Y) frames, and the X/C fallback those frames share.
//!
//! The writer emits either the post's interpolation block or this polar form. Read-back of the
//! fallback is an ordinary lathe X and C. [`to_polar`] is the conversion `compare` applies to the
//! IR pose so both sides meet.

use campanion_ir::{ArcSense, Plane, unwrap_angle};

use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};

/// A face path this close to the spindle axis, in millimetres, is rejected.
pub(crate) const CENTRE_MM: f64 = 1e-3;

pub(crate) const CENTRE_MSG: &str = "face path crosses the spindle centre; split it or use a post with face interpolation";

/// Which interpolation frame a pose is in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum PolarFrame {
    /// `tip.x` and `tip.y` are Cartesian about the spindle.
    Face,
    /// `tip.y` is the arc length along the circumference at `radius`. `tip.x` is the tool-tip radius.
    Cylinder { radius: f64 },
}

/// X/C target. `radius` is the tool-tip radius (the lathe X value before diameter mode).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PolarTarget {
    pub(crate) radius: f64,
    pub(crate) c_deg: f64,
    pub(crate) z: f64,
}

/// Convert one pose into the X/C fallback.
///
/// Face C is `atan2(y, x)` in degrees, unwrapped so it stays within 180° of `prev_c`. Cylinder C
/// is `y / radius` converted from radians to degrees and is not unwrapped: the unrolled distance
/// is already absolute.
pub(crate) fn to_polar(frame: PolarFrame, tip: Xyz, prev_c: f64) -> Result<PolarTarget> {
    if !tip[2].is_finite() {
        return Err(invalid("interpolation point is not finite"));
    }
    match frame {
        PolarFrame::Face => {
            let radius = tip[0].hypot(tip[1]);
            let principal = tip[1].atan2(tip[0]).to_degrees();
            if !radius.is_finite() || !principal.is_finite() || !prev_c.is_finite() {
                return Err(invalid("face angle is not finite"));
            }
            Ok(PolarTarget { radius, c_deg: unwrap_angle(prev_c, principal), z: tip[2] })
        }
        PolarFrame::Cylinder { radius } => {
            if !(radius.is_finite() && radius > 0.0) {
                return Err(invalid("cylinder interpolation radius must be positive"));
            }
            if !tip[0].is_finite() || !tip[1].is_finite() {
                return Err(invalid("cylinder point is not finite"));
            }
            let c_deg = (tip[1] / radius).to_degrees();
            if !c_deg.is_finite() {
                return Err(invalid("cylinder angle is not finite"));
            }
            Ok(PolarTarget { radius: tip[0], c_deg, z: tip[2] })
        }
    }
}

/// Inverse of [`to_polar`]. Face uses `radius` as the polar radius. Cylinder rebuilds Y from C.
pub(crate) fn polar_to_cart(frame: PolarFrame, radius: f64, c_deg: f64, z: f64) -> Xyz {
    match frame {
        PolarFrame::Face => {
            let (sin_c, cos_c) = c_deg.to_radians().sin_cos();
            [radius * cos_c, radius * sin_c, z]
        }
        PolarFrame::Cylinder { radius: mapping } => [radius, c_deg.to_radians() * mapping, z],
    }
}

pub(crate) fn reject_centre(clearance: f64) -> Result<()> {
    if !clearance.is_finite() {
        return Err(invalid("face path is not finite"));
    }
    if clearance <= CENTRE_MM { Err(invalid(CENTRE_MSG)) } else { Ok(()) }
}

/// Closest distance from the XY origin to the segment `a`–`b`. Z is ignored.
pub(crate) fn xy_segment_min(a: Xyz, b: Xyz) -> f64 {
    let (ax, ay) = (a[0], a[1]);
    let (dx, dy) = (b[0] - ax, b[1] - ay);
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-24 {
        return ax.hypot(ay);
    }
    let t = ((-ax * dx - ay * dy) / len2).clamp(0.0, 1.0);
    (ax + t * dx).hypot(ay + t * dy)
}

/// Closest XY distance from the origin to a circular move. The curve itself is measured, not the chord.
pub(crate) fn xy_arc_min(start: Xyz, end: Xyz, center: Xyz, plane: Plane, sense: ArcSense, sweep: f64) -> f64 {
    if plane == Plane::Xy {
        return xy_circle_min(start, end, center, sense, sweep);
    }
    let mut best = start[0].hypot(start[1]).min(end[0].hypot(end[1]));
    let steps = ((sweep / 0.01).ceil() as u32).clamp(4, 4_096);
    for index in 1..steps {
        let t = f64::from(index) / f64::from(steps);
        let point = geom::point_at(start, end, center, plane, sense, sweep, t, false);
        best = best.min(point[0].hypot(point[1]));
    }
    best
}

/// Minimum distance from the origin to an XY circular arc.
///
/// The full circle's nearest point lies toward the origin from the centre. It counts only when
/// that angle sits on the sweep; otherwise an endpoint is closer.
fn xy_circle_min(start: Xyz, end: Xyz, center: Xyz, sense: ArcSense, sweep: f64) -> f64 {
    let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
    let mut best = start[0].hypot(start[1]).min(end[0].hypot(end[1]));
    let centre_radius = center[0].hypot(center[1]);
    if centre_radius <= 1e-15 {
        return best.min(radius);
    }
    let toward = (-center[1]).atan2(-center[0]);
    let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
    let delta = geom::forward_delta(start_angle, toward, matches!(sense, ArcSense::Ccw));
    if delta <= sweep + 1e-8 {
        best = best.min((centre_radius - radius).abs());
    }
    best
}

/// Distance from `point` to the segment `a`–`b`.
pub(crate) fn segment_distance(point: Xyz, a: Xyz, b: Xyz) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ap = [point[0] - a[0], point[1] - a[1], point[2] - a[2]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2];
    if len2 <= 1e-24 {
        return geom::dist3(point, a);
    }
    let t = ((ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2).clamp(0.0, 1.0);
    let closest = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    geom::dist3(point, closest)
}

/// Continuous polar target at the end of `curve`, chaining [`to_polar`] so a face sweep longer
/// than 180° does not take the short unwrap.
pub(crate) fn chained_polar(frame: PolarFrame, curve: &PolarCurve, prev_c: f64) -> Result<PolarTarget> {
    match *curve {
        PolarCurve::Point { end } => to_polar(frame, end, prev_c),
        PolarCurve::Line { start, end } => chain_samples(frame, prev_c, 8, |step| lerp(start, end, step)),
        PolarCurve::Arc { start, end, center, plane, sense, sweep } => {
            let steps = ((sweep / 0.15).ceil() as u32).clamp(4, 20_000);
            chain_samples(frame, prev_c, steps, |step| geom::point_at(start, end, center, plane, sense, sweep, step, true))
        }
    }
}

fn chain_samples<F>(frame: PolarFrame, prev_c: f64, steps: u32, at: F) -> Result<PolarTarget>
where
    F: Fn(f64) -> Xyz,
{
    let mut prev = prev_c;
    let mut last = to_polar(frame, at(0.0), prev)?;
    let steps = steps.max(1);
    for index in 1..=steps {
        let point = at(f64::from(index) / f64::from(steps));
        last = to_polar(frame, point, prev)?;
        prev = last.c_deg;
    }
    Ok(last)
}

fn lerp(start: Xyz, end: Xyz, t: f64) -> Xyz {
    [start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t, start[2] + (end[2] - start[2]) * t]
}

/// Geometry `compare` walks when the post wrote the X/C fallback instead of a native block.
#[derive(Clone, Copy, Debug)]
pub(crate) enum PolarCurve {
    Point { end: Xyz },
    Line { start: Xyz, end: Xyz },
    Arc { start: Xyz, end: Xyz, center: Xyz, plane: Plane, sense: ArcSense, sweep: f64 },
}

impl PolarCurve {
    /// Point at fraction `t` of the whole curve. `t >= 1` is the end point.
    pub(crate) fn at(self, t: f64) -> Xyz {
        match self {
            Self::Point { end } => end,
            Self::Line { start, end } => lerp(start, end, t),
            Self::Arc { start, end, center, plane, sense, sweep } => geom::point_at(start, end, center, plane, sense, sweep, t, true),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{CENTRE_MM, PolarFrame, chained_polar, to_polar, xy_arc_min, xy_segment_min};
    use crate::geom::Xyz;
    use campanion_ir::{ArcSense, Plane};

    #[test]
    fn segment_through_the_origin_is_inside_the_centre_limit() {
        let min = xy_segment_min([5.0, 0.0, -1.0], [-1.0, 0.0, -1.0]);
        assert!(min <= CENTRE_MM, "{min}");
        let clear = xy_segment_min([2.0, -8.0, -1.0], [2.0, 8.0, -1.0]);
        assert!(clear > 1.9, "{clear}");
    }

    #[test]
    fn face_unwrap_stays_with_the_previous_c_and_cylinder_c_is_absolute() {
        let frame = PolarFrame::Face;
        let first = to_polar(frame, [4.0, -10.0, -1.0], 0.0).expect("first");
        let next = to_polar(frame, [-4.0, -10.0, -1.0], first.c_deg).expect("next");
        assert!(next.c_deg < first.c_deg);
        assert!((next.c_deg - first.c_deg).abs() <= 180.0);
        let wound = to_polar(frame, [4.0, -10.0, -1.0], 360.0).expect("wind");
        assert!((wound.c_deg - 360.0).abs() < 90.0);

        let cylinder = PolarFrame::Cylinder { radius: 20.0 };
        let slot = to_polar(cylinder, [20.0, 5.0, -10.0], 0.0).expect("slot");
        let expect = (5.0_f64 / 20.0).to_degrees();
        assert!((slot.c_deg - expect).abs() < 1e-9, "{}", slot.c_deg);
        assert!((slot.radius - 20.0).abs() < 1e-12);
    }

    #[test]
    fn a_long_face_arc_chains_the_long_way() {
        let start: Xyz = [10.0, 0.0, 0.0];
        let end: Xyz = [0.0, 10.0, 0.0];
        let curve =
            super::PolarCurve::Arc { start, end, center: [0.0, 0.0, 0.0], plane: Plane::Xy, sense: ArcSense::Cw, sweep: 270.0_f64.to_radians() };
        let polar = chained_polar(PolarFrame::Face, &curve, 0.0).expect("chain");
        assert!((polar.c_deg + 270.0).abs() < 1e-6, "{}", polar.c_deg);
        let _ = xy_arc_min(start, end, [0.0, 0.0, 0.0], Plane::Xy, ArcSense::Cw, 270.0_f64.to_radians());
    }
}
