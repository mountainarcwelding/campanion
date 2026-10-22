//! Flat paths wrapped onto a cylinder about a rotary axis.
//!
//! Lengths are millimetres. The flat frame is:
//! - `u`, distance along [`RotarySetup::axis_direction`] from [`RotarySetup::center`];
//! - `v`, arc length around the cylinder surface;
//! - `w`, radial offset from that surface (`0` on the surface, negative into the material).
//!
//! A wrapped [`Pose`] stores the machine angle in degrees. The angle is not folded into one turn,
//! so a long `v` becomes a continuous multi-turn move.

use crate::Result;
use crate::check::{self, finite, positive};
use crate::geom::{Pose, Vec3};
use crate::rotary::{RotarySetup, machine_to_part, part_to_machine};

/// Upper bound on input points and on poses [`wrap_polyline`] may return.
const MAX_WRAP_POSES: usize = 1_048_576;

/// Cylinder the flat frame is wrapped onto. `diameter` is the surface diameter, millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cylinder {
    diameter: f64,
}

impl Cylinder {
    /// `diameter` must be finite and greater than zero.
    pub fn new(diameter: f64) -> Result<Self> {
        Ok(Self { diameter: positive(diameter, "cylinder diameter")? })
    }

    /// Surface diameter, millimetres.
    pub fn diameter(self) -> f64 {
        self.diameter
    }
}

/// Machine angle in degrees for arc length `v` on the cylinder surface.
///
/// `a = v / (π · diameter) · 360`. The value is not reduced modulo 360.
fn angle_deg(cyl: Cylinder, v: f64) -> f64 {
    v / (std::f64::consts::PI * cyl.diameter()) * 360.0
}

/// Inverse of [`angle_deg`].
fn arc_length(cyl: Cylinder, angle_deg: f64) -> f64 {
    angle_deg / 360.0 * std::f64::consts::PI * cyl.diameter()
}

/// Tip radius `diameter / 2 + w`. Errors when it is not strictly positive.
fn tip_radius(cyl: Cylinder, w: f64) -> Result<f64> {
    let radius = cyl.diameter() * 0.5 + w;
    if radius.is_finite() && radius > 0.0 { Ok(radius) } else { Err(check::invalid("wrap radius must be positive")) }
}

/// Wrap one flat-frame point onto the cylinder.
///
/// The machine-view tip is `center + u · axis_direction + (diameter / 2 + w) · Z`, with tool axis
/// machine +Z. The returned pose is that view mapped back into the part frame and tagged with the
/// unwrapped angle `v / (π · diameter) · 360` degrees. Errors when `diameter / 2 + w` is not
/// strictly positive, or when a coordinate is not finite.
pub fn wrap_point(setup: &RotarySetup, cyl: Cylinder, u: f64, v: f64, w: f64) -> Result<Pose> {
    let u = finite(u, "wrap u")?;
    let v = finite(v, "wrap v")?;
    let w = finite(w, "wrap w")?;
    let radius = tip_radius(cyl, w)?;
    let angle = angle_deg(cyl, v);
    let center = setup.center();
    let axis = setup.axis_direction();
    let tip = Vec3::new(center.x() + u * axis.x(), center.y() + u * axis.y(), center.z() + u * axis.z() + radius)?;
    let machine = Pose::new(tip, Vec3::Z)?.with_rotary(angle)?;
    let pose = machine_to_part(setup, machine, angle);
    pose.check()?;
    Ok(pose)
}

/// Inverse of [`wrap_point`], using the pose's stored rotary angle.
///
/// Errors when the pose has no rotary angle. `v` comes from that angle, so many turns are
/// recovered exactly. `u` and `w` are read from the machine-view tip after undoing the angle.
pub fn unwrap_pose(setup: &RotarySetup, cyl: Cylinder, pose: Pose) -> Result<(f64, f64, f64)> {
    let Some(angle) = pose.rotary() else {
        return Err(check::invalid("pose has no rotary angle"));
    };
    let v = arc_length(cyl, angle);
    let machine = part_to_machine(setup, pose, angle);
    let center = setup.center();
    let axis = setup.axis_direction();
    let dx = machine.tip().x() - center.x();
    let dy = machine.tip().y() - center.y();
    let dz = machine.tip().z() - center.z();
    let u = dx * axis.x() + dy * axis.y() + dz * axis.z();
    let w = dz - cyl.diameter() * 0.5;
    if u.is_finite() && v.is_finite() && w.is_finite() { Ok((u, v, w)) } else { Err(check::invalid("unwrapped coordinate must be finite")) }
}

/// Largest sweep, in radians, whose sagitta at `radius` is within `tol`.
///
/// Solves `radius * (1 - cos(Δθ / 2)) = tol`. A full turn is the largest sweep this formula
/// describes; a looser tolerance still cannot mean one chord across more than one revolution.
fn max_sweep_rad(radius: f64, tol: f64) -> f64 {
    let ratio = tol / radius;
    if !ratio.is_finite() || ratio <= 0.0 {
        return 0.0;
    }
    if ratio >= 2.0 {
        return std::f64::consts::TAU;
    }
    // For a small ratio, `acos(1 - ratio)` cancels. `1 - cos(φ) <= φ² / 2`, so
    // `φ = sqrt(2 · ratio)` keeps the sagitta inside `tol`.
    if ratio < 1.0e-8 {
        return 2.0 * (2.0 * ratio).sqrt();
    }
    2.0 * (1.0 - ratio).acos()
}

/// How many equal flat-frame steps keep this segment inside `tol`.
///
/// `delta_theta` is the swept axis angle in radians. `r_start` and `r_end` are tip radii.
fn step_count(delta_theta: f64, r_start: f64, r_end: f64, tol: f64) -> Result<usize> {
    let theta = delta_theta.abs();
    let r_max = r_start.max(r_end);
    if !(r_max.is_finite() && r_max > 0.0) {
        return Err(check::invalid("wrap radius must be positive"));
    }
    if !theta.is_finite() {
        return Err(check::invalid("wrap angle must be finite"));
    }
    // No sweep: `u` and `w` move in a straight line, so the part-frame image is already a chord.
    if theta == 0.0 {
        return Ok(1);
    }
    let sweep = max_sweep_rad(r_max, tol);
    // Stay a hair inside the sagitta limit so a rounded angle step still satisfies it.
    let sweep = sweep * (1.0 - 1.0e-4);
    if !(sweep.is_finite() && sweep > 0.0) {
        return Err(check::invalid("wrap tolerance cannot bound the sagitta"));
    }
    let mut steps = (theta / sweep).ceil().max(1.0);
    // Chord error of `p(t) = r(t) · exp(i θ(t))` on `t ∈ [0, 1]` is at most `|p''| / 8`.
    // `|p''| = |Δθ| · sqrt(r² Δθ² + 4 Δr²)`, which is the cylinder sagitta when `Δr = 0` and
    // grows when the flat frame changes `w` along the sweep. Equal steps divide that by `n²`.
    let delta_r = (r_end - r_start).abs();
    let second = (r_max * r_max * theta * theta + 4.0 * delta_r * delta_r).sqrt();
    let deviation = theta * second / 8.0;
    if deviation > tol {
        let flat_steps = ((deviation / tol) * (1.0 + 1.0e-6)).sqrt().ceil();
        if flat_steps > steps {
            steps = flat_steps;
        }
    }
    if !steps.is_finite() || steps < 1.0 || steps > MAX_WRAP_POSES as f64 {
        return Err(check::invalid("wrap polyline has too many points"));
    }
    // `steps` is in `1..=MAX_WRAP_POSES`, so the conversion is exact.
    Ok(steps as usize)
}

/// Wrap a flat polyline onto the cylinder.
///
/// Each input segment is subdivided in the flat frame `(u, v, w)`. Samples stay on that segment,
/// so the flat-frame chord does not leave the input path. The number of steps keeps two cylinder
/// errors within `tol`: the sagitta of the swept angle at the tip radius, `r (1 - cos(Δθ / 2))`
/// with `r = diameter / 2 + w` (the larger end radius on the segment), and the gap between the
/// part-frame chord and the wrapped segment when `w` changes along the sweep. Consecutive rotary
/// angles follow `v` and are not reduced modulo 360.
///
/// Feeds for these poses should stay surface feeds. Use the flat-frame length
/// `sqrt(Δu² + Δv² + Δw²)`, the distance along the developed cylinder. The straight chord between
/// part-frame tips is shorter than that arc, so a feed taken from the chord is not the surface feed.
pub fn wrap_polyline(setup: &RotarySetup, cyl: Cylinder, pts: &[(f64, f64, f64)], tol: f64) -> Result<Vec<Pose>> {
    positive(tol, "wrap tolerance")?;
    if pts.len() > MAX_WRAP_POSES {
        return Err(check::invalid("wrap polyline has too many points"));
    }
    let mut poses = Vec::new();
    let Some((first, rest)) = pts.split_first() else {
        return Ok(poses);
    };
    poses.push(wrap_point(setup, cyl, first.0, first.1, first.2)?);
    let mut prev = *first;
    for next in rest {
        append_segment(setup, cyl, prev, *next, tol, &mut poses)?;
        prev = *next;
    }
    Ok(poses)
}

fn append_segment(setup: &RotarySetup, cyl: Cylinder, start: (f64, f64, f64), end: (f64, f64, f64), tol: f64, poses: &mut Vec<Pose>) -> Result<()> {
    if start.0 == end.0 && start.1 == end.1 && start.2 == end.2 {
        return Ok(());
    }
    let r_start = tip_radius(cyl, finite(start.2, "wrap w")?)?;
    let r_end = tip_radius(cyl, finite(end.2, "wrap w")?)?;
    finite(start.0, "wrap u")?;
    finite(end.0, "wrap u")?;
    let v0 = finite(start.1, "wrap v")?;
    let v1 = finite(end.1, "wrap v")?;
    let a0 = angle_deg(cyl, v0);
    let a1 = angle_deg(cyl, v1);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(check::invalid("wrap angle must be finite"));
    }
    let steps = step_count((a1 - a0).to_radians(), r_start, r_end, tol)?;
    if poses.len().saturating_add(steps) > MAX_WRAP_POSES {
        return Err(check::invalid("wrap polyline has too many points"));
    }
    for step in 1..=steps {
        let point = if step == steps {
            end
        } else {
            let t = step as f64 / steps as f64;
            (start.0 + (end.0 - start.0) * t, start.1 + (end.1 - start.1) * t, start.2 + (end.2 - start.2) * t)
        };
        poses.push(wrap_point(setup, cyl, point.0, point.1, point.2)?);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn close_vec(left: Vec3, right: Vec3, tol: f64) {
        let gap = (left.x() - right.x()).hypot(left.y() - right.y()).hypot(left.z() - right.z());
        assert!(gap <= tol, "gap {gap}");
    }

    #[test]
    fn cylinder_accepts_a_positive_diameter_and_rejects_the_rest() {
        let cyl = Cylinder::new(25.0).expect("diameter");
        assert_eq!(cyl.diameter(), 25.0);
        assert!(Cylinder::new(0.0).is_err());
        assert!(Cylinder::new(-1.0).is_err());
        assert!(Cylinder::new(f64::NAN).is_err());
        assert!(Cylinder::new(f64::INFINITY).is_err());
        assert!(Cylinder::new(f64::NEG_INFINITY).is_err());
    }

    #[test]
    fn wrap_point_matches_a_known_a_axis_quarter_turn() {
        let setup = RotarySetup::new(crate::RotaryAxisKind::A, Vec3::new(1.0, 2.0, 3.0).expect("centre"), crate::RotaryDirection::Positive, None)
            .expect("setup");
        let cyl = Cylinder::new(20.0).expect("cylinder");
        let v = 0.25 * std::f64::consts::PI * cyl.diameter();
        let pose = wrap_point(&setup, cyl, 4.0, v, 1.0).expect("wrap");
        assert!((pose.rotary().expect("angle") - 90.0).abs() < 1e-12);
        close_vec(pose.tip(), Vec3::new(5.0, 13.0, 3.0).expect("tip"), 1e-12);
        close_vec(pose.axis(), Vec3::new(0.0, 1.0, 0.0).expect("axis"), 1e-12);
        let (u, got_v, w) = unwrap_pose(&setup, cyl, pose).expect("unwrap");
        assert!((u - 4.0).abs() < 1e-12);
        assert!((got_v - v).abs() < 1e-12);
        assert!((w - 1.0).abs() < 1e-12);
    }

    fn setup_at(kind: crate::RotaryAxisKind, direction: crate::RotaryDirection, center: Vec3) -> RotarySetup {
        RotarySetup::new(kind, center, direction, None).expect("setup")
    }

    fn origin_setup(kind: crate::RotaryAxisKind, direction: crate::RotaryDirection) -> RotarySetup {
        setup_at(kind, direction, Vec3::ZERO)
    }

    #[test]
    fn wrap_point_matches_known_b_axis_and_negative_a() {
        let b = setup_at(crate::RotaryAxisKind::B, crate::RotaryDirection::Positive, Vec3::new(1.0, 2.0, 3.0).expect("centre"));
        let cyl = Cylinder::new(20.0).expect("cylinder");
        let quarter = 0.25 * std::f64::consts::PI * cyl.diameter();
        let pose = wrap_point(&b, cyl, 5.0, quarter, 0.0).expect("wrap");
        assert!((pose.rotary().expect("angle") - 90.0).abs() < 1e-12);
        close_vec(pose.tip(), Vec3::new(-9.0, 7.0, 3.0).expect("tip"), 1e-12);
        close_vec(pose.axis(), Vec3::new(-1.0, 0.0, 0.0).expect("axis"), 1e-12);

        let a_neg = setup_at(crate::RotaryAxisKind::A, crate::RotaryDirection::Negative, Vec3::new(1.0, 2.0, 3.0).expect("centre"));
        let pose = wrap_point(&a_neg, cyl, 4.0, quarter, 1.0).expect("wrap");
        close_vec(pose.tip(), Vec3::new(5.0, -9.0, 3.0).expect("tip"), 1e-12);
        close_vec(pose.axis(), Vec3::new(0.0, -1.0, 0.0).expect("axis"), 1e-12);
        assert!(pose.axis().x().abs() <= 1e-12);
    }

    #[test]
    fn angles_are_not_folded_into_one_turn() {
        let setup = origin_setup(crate::RotaryAxisKind::A, crate::RotaryDirection::Positive);
        let cyl = Cylinder::new(12.5).expect("cylinder");
        let turns = 100.0;
        let v = turns * std::f64::consts::PI * cyl.diameter();
        let pose = wrap_point(&setup, cyl, -3.0, v, 0.25).expect("wrap");
        assert!((pose.rotary().expect("angle") - turns * 360.0).abs() < 1e-9);
        let (u, got_v, w) = unwrap_pose(&setup, cyl, pose).expect("unwrap");
        assert!((u - -3.0).abs() <= 1e-9);
        assert!((got_v - v).abs() <= 1e-9);
        assert!((w - 0.25).abs() <= 1e-9);

        let back = -0.25 * std::f64::consts::PI * cyl.diameter();
        let pose = wrap_point(&setup, cyl, 0.0, back, 0.0).expect("wrap");
        assert!((pose.rotary().expect("angle") - -90.0).abs() < 1e-12);
    }

    #[test]
    fn wrap_rejects_bad_inputs_and_a_pose_without_a_rotary_angle() {
        let setup = origin_setup(crate::RotaryAxisKind::B, crate::RotaryDirection::Negative);
        let cyl = Cylinder::new(10.0).expect("cylinder");
        assert!(wrap_point(&setup, cyl, 0.0, 0.0, -5.0).is_err());
        assert!(wrap_point(&setup, cyl, 0.0, 0.0, -5.1).is_err());
        assert!(wrap_point(&setup, cyl, f64::NAN, 0.0, 0.0).is_err());
        assert!(wrap_point(&setup, cyl, 0.0, f64::INFINITY, 0.0).is_err());
        assert!(wrap_point(&setup, cyl, 0.0, 0.0, f64::NEG_INFINITY).is_err());
        let bare = Pose::new(Vec3::ZERO, Vec3::Z).expect("pose");
        assert!(unwrap_pose(&setup, cyl, bare).is_err());
        assert!(wrap_polyline(&setup, cyl, &[(0.0, 0.0, 0.0)], 0.0).is_err());
        assert!(wrap_polyline(&setup, cyl, &[(0.0, 0.0, 0.0)], -0.01).is_err());
        assert!(wrap_polyline(&setup, cyl, &[(0.0, 0.0, 0.0)], f64::NAN).is_err());
        assert!(wrap_polyline(&setup, cyl, &[(0.0, 0.0, -5.0)], 0.1).is_err());
        assert!(wrap_polyline(&setup, cyl, &[(0.0, 0.0, 0.0), (1.0, 1.0, -9.0)], 0.1).is_err());
        assert!(wrap_polyline(&setup, cyl, &[], 0.1).expect("empty").is_empty());
        assert_eq!(wrap_polyline(&setup, cyl, &[(1.0, 2.0, 0.0)], 0.1).expect("one").len(), 1);
    }

    fn max_step_rad(radius: f64, tol: f64) -> f64 {
        let ratio = tol / radius;
        if !ratio.is_finite() || ratio <= 0.0 {
            return 0.0;
        }
        if ratio >= 2.0 {
            return std::f64::consts::TAU;
        }
        2.0 * (1.0 - ratio).acos()
    }

    fn sagitta(radius: f64, sweep_rad: f64) -> f64 {
        radius * (1.0 - (sweep_rad.abs() * 0.5).cos())
    }

    fn hypot3(x: f64, y: f64, z: f64) -> f64 {
        x.hypot(y).hypot(z)
    }

    fn flat_length(pts: &[(f64, f64, f64)]) -> f64 {
        pts.windows(2).map(|pair| hypot3(pair[1].0 - pair[0].0, pair[1].1 - pair[0].1, pair[1].2 - pair[0].2)).sum()
    }

    fn dist_to_segment(point: (f64, f64, f64), start: (f64, f64, f64), end: (f64, f64, f64)) -> f64 {
        let ab = (end.0 - start.0, end.1 - start.1, end.2 - start.2);
        let ap = (point.0 - start.0, point.1 - start.1, point.2 - start.2);
        let ab2 = ab.0 * ab.0 + ab.1 * ab.1 + ab.2 * ab.2;
        let t = if ab2 == 0.0 { 0.0 } else { ((ap.0 * ab.0 + ap.1 * ab.1 + ap.2 * ab.2) / ab2).clamp(0.0, 1.0) };
        hypot3(point.0 - (start.0 + ab.0 * t), point.1 - (start.1 + ab.1 * t), point.2 - (start.2 + ab.2 * t))
    }

    fn chord_gap(setup: &RotarySetup, cyl: Cylinder, start: &Pose, end: &Pose, flat_start: (f64, f64, f64), flat_end: (f64, f64, f64)) -> f64 {
        let mut max_gap = 0.0_f64;
        for k in 0..=32 {
            let t = f64::from(k) / 32.0;
            let u = flat_start.0 + (flat_end.0 - flat_start.0) * t;
            let v = flat_start.1 + (flat_end.1 - flat_start.1) * t;
            let w = flat_start.2 + (flat_end.2 - flat_start.2) * t;
            let true_tip = wrap_point(setup, cyl, u, v, w).expect("curve").tip();
            let gap = hypot3(
                true_tip.x() - (start.tip().x() + (end.tip().x() - start.tip().x()) * t),
                true_tip.y() - (start.tip().y() + (end.tip().y() - start.tip().y()) * t),
                true_tip.z() - (start.tip().z() + (end.tip().z() - start.tip().z()) * t),
            );
            max_gap = max_gap.max(gap);
        }
        max_gap
    }

    fn assert_polyline_bounds(setup: &RotarySetup, cyl: Cylinder, pts: &[(f64, f64, f64)], tol: f64, poses: &[Pose]) {
        assert_eq!(poses.is_empty(), pts.is_empty());
        if pts.is_empty() {
            return;
        }
        let mut flats = Vec::new();
        let mut prev_angle: Option<f64> = None;
        let mut prev_flat: Option<(f64, f64, f64)> = None;
        let mut prev_pose: Option<&Pose> = None;
        for pose in poses {
            let flat = unwrap_pose(setup, cyl, *pose).expect("unwrap");
            let angle = pose.rotary().expect("angle");
            let expect_angle = flat.1 / (std::f64::consts::PI * cyl.diameter()) * 360.0;
            assert!((angle - expect_angle).abs() <= 1e-9, "angle {angle} vs {expect_angle}");
            let radial = setup.axis_direction();
            let dot = pose.axis().x() * radial.x() + pose.axis().y() * radial.y() + pose.axis().z() * radial.z();
            assert!(dot.abs() <= 1e-9, "tool axis not perpendicular: {dot}");
            let mut on_path = f64::INFINITY;
            for window in pts.windows(2) {
                on_path = on_path.min(dist_to_segment(flat, window[0], window[1]));
            }
            if pts.len() == 1 {
                on_path = hypot3(flat.0 - pts[0].0, flat.1 - pts[0].1, flat.2 - pts[0].2);
            }
            assert!(on_path <= 1e-8, "flat sample left the input path by {on_path}");
            if let (Some(prev_angle), Some(prev_flat), Some(prev_pose)) = (prev_angle, prev_flat, prev_pose) {
                let delta_v_angle = (flat.1 - prev_flat.1) / (std::f64::consts::PI * cyl.diameter()) * 360.0;
                assert!((angle - prev_angle - delta_v_angle).abs() <= 1e-6, "angle jumped");
                let radius = (cyl.diameter() * 0.5 + prev_flat.2).max(cyl.diameter() * 0.5 + flat.2);
                let sweep = (angle - prev_angle).to_radians().abs();
                let sag = sagitta(radius, sweep);
                assert!(sag <= tol, "sagitta {sag} exceeds {tol}");
                assert!(sweep <= max_step_rad(radius, tol) + 1e-9, "step {sweep} exceeds {}", max_step_rad(radius, tol));
                let gap = chord_gap(setup, cyl, prev_pose, pose, prev_flat, flat);
                assert!(gap <= tol + 1e-8, "chord gap {gap} exceeds {tol}");
            }
            prev_angle = Some(angle);
            prev_flat = Some(flat);
            prev_pose = Some(pose);
            flats.push(flat);
        }
        let length_gap = (flat_length(&flats) - flat_length(pts)).abs();
        assert!(length_gap <= 1e-6, "flat length changed by {length_gap}");
    }

    #[test]
    fn straight_axial_move_is_not_subdivided() {
        let setup = origin_setup(crate::RotaryAxisKind::A, crate::RotaryDirection::Positive);
        let cyl = Cylinder::new(30.0).expect("cylinder");
        let pts = [(0.0, 1.0, 0.0), (25.0, 1.0, 0.0)];
        let poses = wrap_polyline(&setup, cyl, &pts, 0.01).expect("wrap");
        assert_eq!(poses.len(), 2);
        assert_polyline_bounds(&setup, cyl, &pts, 0.01, &poses);
    }

    #[test]
    fn quarter_turn_and_varying_radius_stay_inside_tolerance() {
        let setup = setup_at(crate::RotaryAxisKind::A, crate::RotaryDirection::Negative, Vec3::new(2.0, -4.0, 1.0).expect("centre"));
        let cyl = Cylinder::new(40.0).expect("cylinder");
        let circumference = std::f64::consts::PI * cyl.diameter();
        let pts = [(0.0, 0.0, 0.0), (0.0, 0.25 * circumference, 0.0), (6.0, 0.25 * circumference + 8.0, 15.0)];
        let tol = 0.01;
        let poses = wrap_polyline(&setup, cyl, &pts, tol).expect("wrap");
        assert!(poses.len() > pts.len(), "expected subdivision, got {}", poses.len());
        assert_polyline_bounds(&setup, cyl, &pts, tol, &poses);

        // A short sweep with a large radial change. The sagitta split alone is too coarse.
        let harsh = [(0.0, 0.0, 0.0), (1.0, 0.2 * cyl.diameter(), 30.0)];
        let harsh_tol = 0.01;
        let harsh_poses = wrap_polyline(&setup, cyl, &harsh, harsh_tol).expect("harsh");
        assert_polyline_bounds(&setup, cyl, &harsh, harsh_tol, &harsh_poses);
    }

    #[test]
    fn multi_turn_polyline_keeps_angles_continuous() {
        let setup = setup_at(crate::RotaryAxisKind::B, crate::RotaryDirection::Positive, Vec3::new(0.0, 5.0, -2.0).expect("centre"));
        let cyl = Cylinder::new(50.0).expect("cylinder");
        let circumference = std::f64::consts::PI * cyl.diameter();
        let pts = [(0.0, 0.0, 0.0), (4.0, 3.5 * circumference, -0.5), (-2.0, -0.5 * circumference, 0.25), (-2.0, -0.5 * circumference, 0.25)];
        let tol = 0.05;
        let poses = wrap_polyline(&setup, cyl, &pts, tol).expect("wrap");
        assert_polyline_bounds(&setup, cyl, &pts, tol, &poses);
        let angles: Vec<f64> = poses.iter().map(|pose| pose.rotary().expect("angle")).collect();
        let first = angles[0];
        let peak = angles.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(peak > 3.0 * 360.0, "peak {peak} from {first}");
        let last = angles[angles.len() - 1];
        assert!(last < 0.0, "last angle {last}");
    }

    fn arb_kind() -> impl proptest::strategy::Strategy<Value = crate::RotaryAxisKind> {
        prop_oneof![Just(crate::RotaryAxisKind::A), Just(crate::RotaryAxisKind::B)]
    }

    fn arb_dir() -> impl proptest::strategy::Strategy<Value = crate::RotaryDirection> {
        prop_oneof![Just(crate::RotaryDirection::Positive), Just(crate::RotaryDirection::Negative)]
    }

    fn arb_vec(range: std::ops::Range<f64>) -> impl proptest::strategy::Strategy<Value = Vec3> {
        (range.clone(), range.clone(), range).prop_filter_map("finite vec", |(x, y, z)| Vec3::new(x, y, z).ok())
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(80))]

        #[test]
        fn unwrap_of_wrap_round_trips_and_tool_axis_is_perpendicular(
            kind in arb_kind(),
            direction in arb_dir(),
            center in arb_vec(-80.0..80.0),
            diameter in 0.5f64..400.0,
            u in -2_000.0f64..2_000.0,
            v in -20_000.0f64..20_000.0,
            radius_frac in 0.002f64..1.5,
        ) {
            let setup = RotarySetup::new(kind, center, direction, None).expect("setup");
            let cyl = Cylinder::new(diameter).expect("cylinder");
            let w = diameter * radius_frac - diameter * 0.5;
            let pose = wrap_point(&setup, cyl, u, v, w).expect("wrap");
            let (got_u, got_v, got_w) = unwrap_pose(&setup, cyl, pose).expect("unwrap");
            prop_assert!((got_u - u).abs() <= 1e-9, "u {got_u} vs {u}");
            prop_assert!((got_v - v).abs() <= 1e-9, "v {got_v} vs {v}");
            prop_assert!((got_w - w).abs() <= 1e-9, "w {got_w} vs {w}");
            let radial = setup.axis_direction();
            let dot = pose.axis().x() * radial.x() + pose.axis().y() * radial.y() + pose.axis().z() * radial.z();
            prop_assert!(dot.abs() <= 1e-9, "dot {dot}");
            let axis_len = hypot3(pose.axis().x(), pose.axis().y(), pose.axis().z());
            prop_assert!((axis_len - 1.0).abs() <= 1e-9, "axis length {axis_len}");
            let along = (pose.tip().x() - center.x()) * radial.x()
                + (pose.tip().y() - center.y()) * radial.y()
                + (pose.tip().z() - center.z()) * radial.z();
            let px = pose.tip().x() - center.x() - along * radial.x();
            let py = pose.tip().y() - center.y() - along * radial.y();
            let pz = pose.tip().z() - center.z() - along * radial.z();
            let align = (px * pose.axis().x() + py * pose.axis().y() + pz * pose.axis().z()) / hypot3(px, py, pz);
            prop_assert!((align - 1.0).abs() <= 1e-8, "alignment {align}");
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(40))]

        #[test]
        fn polyline_sagitta_and_angle_steps_stay_inside_tolerance(
            kind in arb_kind(),
            direction in arb_dir(),
            center in arb_vec(-30.0..30.0),
            diameter in 20.0f64..80.0,
            tol in 0.02f64..0.3,
            raw in prop::collection::vec((-40.0f64..40.0, -80.0f64..80.0, 0.05f64..1.2), 1..=4),
        ) {
            let setup = RotarySetup::new(kind, center, direction, None).expect("setup");
            let cyl = Cylinder::new(diameter).expect("cylinder");
            let pts: Vec<(f64, f64, f64)> = raw.iter().map(|(u, v, frac)| (*u, *v, diameter * frac - diameter * 0.5)).collect();
            let poses = wrap_polyline(&setup, cyl, &pts, tol).expect("wrap");
            assert_polyline_bounds(&setup, cyl, &pts, tol, &poses);
        }
    }
}
