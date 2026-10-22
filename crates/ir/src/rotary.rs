//! Fourth-axis setup in the part frame, and the rotations it defines.
//!
//! The IR stays process-neutral. A [`RotarySetup`] names an A or B table axis, a point on that
//! axis, a sign for the machine angle, and optional travel limits. Helpers rotate points, vectors
//! and poses about that axis. Public angles are degrees. Machine joint values on [`crate::Axis`]
//! remain radians.

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, finite};
use crate::geom::{Pose, Vec3};

/// Component along the rotary axis, after the tool axis is normalised, that makes a pose
/// unreachable with one rotary axis.
const PARALLEL_LIMIT: f64 = 1e-9;

/// A or B table axis, named in the part frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RotaryAxisKind {
    /// Rotation about the part X axis.
    A,
    /// Rotation about the part Y axis.
    B,
}

/// Sign applied to the machine A/B angle before a right-hand rotation about the axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RotaryDirection {
    /// The machine angle is used as the right-hand rotation angle.
    Positive,
    /// The machine angle is negated before the right-hand rotation.
    Negative,
}

/// Inclusive travel limits for a machine A/B angle, in degrees. `min_deg` is strictly less than
/// `max_deg`. Stored angles may exceed 360; the limits apply to that stored value.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RotaryLimits {
    min_deg: f64,
    max_deg: f64,
}

impl RotaryLimits {
    /// `min_deg` must be finite, `max_deg` must be finite, and `min_deg < max_deg`.
    pub fn new(min_deg: f64, max_deg: f64) -> Result<Self> {
        let limits = Self { min_deg, max_deg };
        limits.check()?;
        Ok(limits)
    }

    /// Lower bound, degrees, inclusive.
    pub fn min_deg(self) -> f64 {
        self.min_deg
    }

    /// Upper bound, degrees, inclusive.
    pub fn max_deg(self) -> f64 {
        self.max_deg
    }

    /// Whether `angle_deg` lies in `min_deg..=max_deg`.
    pub fn contains(self, angle_deg: f64) -> bool {
        self.min_deg <= angle_deg && angle_deg <= self.max_deg
    }

    pub(crate) fn check(self) -> Result<()> {
        let min = finite(self.min_deg, "rotary limit minimum")?;
        let max = finite(self.max_deg, "rotary limit maximum")?;
        if min < max { Ok(()) } else { Err(check::invalid("rotary limit minimum is not less than maximum")) }
    }
}

/// Fourth-axis table setup in the part frame.
///
/// `center` is a point on the rotary axis line, in millimetres. The axis direction is part +X for
/// [`RotaryAxisKind::A`] and part +Y for [`RotaryAxisKind::B`]. [`RotaryDirection`] is the sign
/// applied to the machine angle before a right-hand rotation about that direction.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RotarySetup {
    axis: RotaryAxisKind,
    center: Vec3,
    direction: RotaryDirection,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    limits: Option<RotaryLimits>,
}

impl RotarySetup {
    /// Build a setup. `center` must be finite. When `limits` is set, `min_deg < max_deg`.
    pub fn new(axis: RotaryAxisKind, center: Vec3, direction: RotaryDirection, limits: Option<RotaryLimits>) -> Result<Self> {
        let setup = Self { axis, center, direction, limits };
        setup.check()?;
        Ok(setup)
    }

    /// A about part X, or B about part Y.
    pub fn axis(self) -> RotaryAxisKind {
        self.axis
    }

    /// A point on the rotary axis line, millimetres, part frame.
    pub fn center(self) -> Vec3 {
        self.center
    }

    /// Sign applied to the machine A/B angle.
    pub fn direction(self) -> RotaryDirection {
        self.direction
    }

    /// Inclusive travel limits in degrees, when the setup records them.
    pub fn limits(self) -> Option<RotaryLimits> {
        self.limits
    }

    /// Unit direction of the rotary axis line in the part frame: +X for A, +Y for B.
    pub fn axis_direction(self) -> Vec3 {
        match self.axis {
            RotaryAxisKind::A => Vec3::X,
            RotaryAxisKind::B => Vec3::Y,
        }
    }

    pub(crate) fn check(self) -> Result<()> {
        self.center.check("rotary centre")?;
        if let Some(limits) = self.limits {
            limits.check()?;
        }
        Ok(())
    }

    fn sign(self) -> f64 {
        match self.direction {
            RotaryDirection::Positive => 1.0,
            RotaryDirection::Negative => -1.0,
        }
    }

    fn geometric_rad(self, angle_deg: f64) -> f64 {
        (angle_deg * self.sign()).to_radians()
    }
}

/// Rotate point `p` about the setup axis line by `angle_deg` degrees.
///
/// The rotation is right-handed about [`RotarySetup::axis_direction`], with [`RotaryDirection`]
/// applied to `angle_deg`. `p` is millimetres in the part frame.
pub fn rotate_point(setup: &RotarySetup, p: Vec3, angle_deg: f64) -> Vec3 {
    let offset = sub(p, setup.center);
    add(setup.center, rotate_vector(setup, offset, angle_deg))
}

/// Rotate vector `v` about the setup axis by `angle_deg` degrees.
///
/// The rotation is right-handed about [`RotarySetup::axis_direction`], with [`RotaryDirection`]
/// applied to `angle_deg`.
pub fn rotate_vector(setup: &RotarySetup, v: Vec3, angle_deg: f64) -> Vec3 {
    rodrigues(setup.axis_direction(), v, setup.geometric_rad(angle_deg))
}

/// Rotate a part-frame pose by `angle_deg` so the machine sees it.
///
/// Tip and axis are rotated about the setup axis. A rotary angle already stored on `pose` is kept.
pub fn part_to_machine(setup: &RotarySetup, pose: Pose, angle_deg: f64) -> Pose {
    Pose::from_parts(rotate_point(setup, pose.tip(), angle_deg), rotate_vector(setup, pose.axis(), angle_deg), pose.rotary())
}

/// Inverse of [`part_to_machine`]: rotate a machine-frame pose back into the part frame.
pub fn machine_to_part(setup: &RotarySetup, pose: Pose, angle_deg: f64) -> Pose {
    part_to_machine(setup, pose, -angle_deg)
}

/// Machine A/B angle in `(-180, 180]` degrees that turns `tool_axis` onto machine +Z.
///
/// `tool_axis` is a part-frame direction. Returns `None` when the axis cannot be normalised, or
/// when its component along the rotary axis is greater than `1e-9` after normalising, so one
/// rotary axis cannot reach +Z.
pub fn index_angle_for_axis(setup: &RotarySetup, tool_axis: Vec3) -> Option<f64> {
    let tool = normalised(tool_axis)?;
    if dot(tool, setup.axis_direction()).abs() > PARALLEL_LIMIT {
        return None;
    }
    let geometric = match setup.axis {
        RotaryAxisKind::A => tool.y().atan2(tool.z()),
        RotaryAxisKind::B => (-tool.x()).atan2(tool.z()),
    };
    Some(wrap_signed_180(geometric.to_degrees() * setup.sign()))
}

/// Value congruent to `next_deg` modulo 360 that is nearest to `prev_deg`.
///
/// Used to unwrap a principal angle onto a continuous multi-turn move. The result differs from
/// `prev_deg` by at most 180 degrees.
pub fn unwrap_angle(prev_deg: f64, next_deg: f64) -> f64 {
    let mut delta = (next_deg - prev_deg).rem_euclid(360.0);
    if delta > 180.0 {
        delta -= 360.0;
    }
    prev_deg + delta
}

fn wrap_signed_180(deg: f64) -> f64 {
    let mut wrapped = deg.rem_euclid(360.0);
    if wrapped > 180.0 {
        wrapped -= 360.0;
    }
    wrapped
}

fn rodrigues(axis: Vec3, v: Vec3, theta: f64) -> Vec3 {
    let cos = theta.cos();
    let sin = theta.sin();
    let along = scale(axis, dot(axis, v));
    let crossed = cross(axis, v);
    add(add(scale(v, cos), scale(crossed, sin)), scale(along, 1.0 - cos))
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.x() + b.x(), a.y() + b.y(), a.z() + b.z())
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.x() - b.x(), a.y() - b.y(), a.z() - b.z())
}

fn scale(a: Vec3, s: f64) -> Vec3 {
    Vec3::from_xyz(a.x() * s, a.y() * s, a.z() * s)
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x() * b.x() + a.y() * b.y() + a.z() * b.z()
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x())
}

fn length(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

fn normalised(a: Vec3) -> Option<Vec3> {
    let len = length(a);
    if !len.is_finite() || len == 0.0 {
        return None;
    }
    Some(scale(a, 1.0 / len))
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn pt(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z).expect("point")
    }

    fn setup(kind: RotaryAxisKind, direction: RotaryDirection, center: Vec3) -> RotarySetup {
        RotarySetup::new(kind, center, direction, None).expect("setup")
    }

    fn close(left: Vec3, right: Vec3, tol: f64) {
        let dx = left.x() - right.x();
        let dy = left.y() - right.y();
        let dz = left.z() - right.z();
        let gap = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(gap <= tol, "left=({} {} {}) right=({} {} {}) gap={gap}", left.x(), left.y(), left.z(), right.x(), right.y(), right.z());
    }

    fn pose_at(tip: Vec3, axis: Vec3) -> Pose {
        Pose::new(tip, axis).expect("pose")
    }

    #[test]
    fn a_axis_rotates_known_points() {
        let origin = setup(RotaryAxisKind::A, RotaryDirection::Positive, Vec3::ZERO);
        let p = pt(0.0, 1.0, 0.0);
        close(rotate_point(&origin, p, 0.0), pt(0.0, 1.0, 0.0), 1e-12);
        close(rotate_point(&origin, p, 90.0), pt(0.0, 0.0, 1.0), 1e-12);
        close(rotate_point(&origin, p, 180.0), pt(0.0, -1.0, 0.0), 1e-12);
        close(rotate_point(&origin, p, -90.0), pt(0.0, 0.0, -1.0), 1e-12);
        close(rotate_point(&origin, pt(1.0, 0.0, 0.0), 90.0), pt(1.0, 0.0, 0.0), 1e-12);
        close(rotate_vector(&origin, Vec3::Z, 90.0), pt(0.0, -1.0, 0.0), 1e-12);

        let offset = setup(RotaryAxisKind::A, RotaryDirection::Positive, pt(10.0, 0.0, 0.0));
        close(rotate_point(&offset, pt(10.0, 1.0, 0.0), 90.0), pt(10.0, 0.0, 1.0), 1e-12);

        let negative = setup(RotaryAxisKind::A, RotaryDirection::Negative, Vec3::ZERO);
        close(rotate_point(&negative, p, 90.0), pt(0.0, 0.0, -1.0), 1e-12);
    }

    #[test]
    fn b_axis_rotates_known_points() {
        let origin = setup(RotaryAxisKind::B, RotaryDirection::Positive, Vec3::ZERO);
        let p = pt(1.0, 0.0, 0.0);
        close(rotate_point(&origin, p, 0.0), pt(1.0, 0.0, 0.0), 1e-12);
        close(rotate_point(&origin, p, 90.0), pt(0.0, 0.0, -1.0), 1e-12);
        close(rotate_point(&origin, p, 180.0), pt(-1.0, 0.0, 0.0), 1e-12);
        close(rotate_point(&origin, p, -90.0), pt(0.0, 0.0, 1.0), 1e-12);
        close(rotate_point(&origin, pt(0.0, 1.0, 0.0), 90.0), pt(0.0, 1.0, 0.0), 1e-12);
        close(rotate_vector(&origin, Vec3::Z, 90.0), pt(1.0, 0.0, 0.0), 1e-12);

        let offset = setup(RotaryAxisKind::B, RotaryDirection::Positive, pt(0.0, 5.0, 0.0));
        close(rotate_point(&offset, pt(1.0, 5.0, 0.0), -90.0), pt(0.0, 5.0, 1.0), 1e-12);

        let negative = setup(RotaryAxisKind::B, RotaryDirection::Negative, Vec3::ZERO);
        close(rotate_point(&negative, p, 90.0), pt(0.0, 0.0, 1.0), 1e-12);
    }

    #[test]
    fn index_angle_turns_known_axes_onto_z() {
        let a = setup(RotaryAxisKind::A, RotaryDirection::Positive, Vec3::ZERO);
        assert_eq!(index_angle_for_axis(&a, Vec3::Z), Some(0.0));
        assert_eq!(index_angle_for_axis(&a, Vec3::Y), Some(90.0));
        assert_eq!(index_angle_for_axis(&a, pt(0.0, 0.0, -1.0)), Some(180.0));
        assert_eq!(index_angle_for_axis(&a, pt(0.0, -1.0, 0.0)), Some(-90.0));
        assert_eq!(index_angle_for_axis(&a, Vec3::X), None);
        close(rotate_vector(&a, Vec3::Y, index_angle_for_axis(&a, Vec3::Y).expect("a y")), Vec3::Z, 1e-12);

        let b = setup(RotaryAxisKind::B, RotaryDirection::Positive, Vec3::ZERO);
        assert_eq!(index_angle_for_axis(&b, Vec3::Z), Some(0.0));
        assert_eq!(index_angle_for_axis(&b, Vec3::X), Some(-90.0));
        assert_eq!(index_angle_for_axis(&b, pt(0.0, 0.0, -1.0)), Some(180.0));
        assert_eq!(index_angle_for_axis(&b, pt(-1.0, 0.0, 0.0)), Some(90.0));
        assert_eq!(index_angle_for_axis(&b, Vec3::Y), None);
        close(rotate_vector(&b, Vec3::X, index_angle_for_axis(&b, Vec3::X).expect("b x")), Vec3::Z, 1e-12);

        let a_neg = setup(RotaryAxisKind::A, RotaryDirection::Negative, Vec3::ZERO);
        assert_eq!(index_angle_for_axis(&a_neg, Vec3::Y), Some(-90.0));
    }

    #[test]
    fn unwrap_angle_picks_the_nearest_congruent_value() {
        assert_eq!(unwrap_angle(350.0, 10.0), 370.0);
        assert_eq!(unwrap_angle(10.0, 350.0), -10.0);
        assert_eq!(unwrap_angle(0.0, 0.0), 0.0);
        assert_eq!(unwrap_angle(0.0, 180.0), 180.0);
        assert_eq!(unwrap_angle(90.0, 90.0), 90.0);
    }

    #[test]
    fn rotary_setup_rejects_bad_limits_and_centre() {
        assert!(RotaryLimits::new(10.0, 10.0).is_err());
        assert!(RotaryLimits::new(20.0, -5.0).is_err());
        assert!(RotaryLimits::new(f64::NAN, 1.0).is_err());
        let limits = RotaryLimits::new(-90.0, 90.0).expect("limits");
        assert!(limits.contains(-90.0) && limits.contains(90.0) && !limits.contains(90.1));
        assert!(RotarySetup::new(RotaryAxisKind::A, Vec3::from_xyz(f64::INFINITY, 0.0, 0.0), RotaryDirection::Positive, None).is_err());
        assert!(Pose::new(Vec3::ZERO, Vec3::Z).expect("pose").with_rotary(f64::NAN).is_err());
        assert_eq!(Pose::new(Vec3::ZERO, Vec3::Z).expect("pose").with_rotary(370.0).expect("angle").rotary(), Some(370.0));
    }

    #[test]
    fn part_and_machine_frames_swap_a_known_pose() {
        let a = setup(RotaryAxisKind::A, RotaryDirection::Positive, Vec3::ZERO);
        let pose = pose_at(pt(0.0, 1.0, 0.0), Vec3::Y);
        let machine = part_to_machine(&a, pose, 90.0);
        close(machine.tip(), pt(0.0, 0.0, 1.0), 1e-12);
        close(machine.axis(), Vec3::Z, 1e-12);
        let back = machine_to_part(&a, machine, 90.0);
        close(back.tip(), pose.tip(), 1e-12);
        close(back.axis(), pose.axis(), 1e-12);
    }

    fn arb_kind() -> impl Strategy<Value = RotaryAxisKind> {
        prop_oneof![Just(RotaryAxisKind::A), Just(RotaryAxisKind::B)]
    }

    fn arb_dir() -> impl Strategy<Value = RotaryDirection> {
        prop_oneof![Just(RotaryDirection::Positive), Just(RotaryDirection::Negative)]
    }

    fn arb_vec(range: std::ops::Range<f64>) -> impl Strategy<Value = Vec3> {
        (range.clone(), range.clone(), range).prop_filter_map("finite vec", |(x, y, z)| Vec3::new(x, y, z).ok())
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(80))]

        #[test]
        fn part_to_machine_then_machine_to_part_round_trips(
            kind in arb_kind(),
            direction in arb_dir(),
            center in arb_vec(-50.0..50.0),
            tip in arb_vec(-50.0..50.0),
            axis in arb_vec(-5.0..5.0),
            angle in -720.0f64..720.0,
        ) {
            let Ok(setup) = RotarySetup::new(kind, center, direction, None) else {
                return Ok(());
            };
            let Ok(pose) = Pose::new(tip, axis) else {
                return Ok(());
            };
            let back = machine_to_part(&setup, part_to_machine(&setup, pose, angle), angle);
            close(back.tip(), pose.tip(), 1e-9);
            close(back.axis(), pose.axis(), 1e-9);
            prop_assert_eq!(back.rotary(), pose.rotary());
        }

        #[test]
        fn index_angle_then_rotate_gives_machine_z(
            kind in arb_kind(),
            direction in arb_dir(),
            angle in -720.0f64..720.0,
        ) {
            let setup = setup(kind, direction, Vec3::ZERO);
            let part_axis = rotate_vector(&setup, Vec3::Z, -angle);
            let Some(indexed) = index_angle_for_axis(&setup, part_axis) else {
                return Err(TestCaseError::fail("reachable axis had no index angle"));
            };
            close(rotate_vector(&setup, part_axis, indexed), Vec3::Z, 1e-9);
            let wrapped = unwrap_angle(0.0, angle);
            prop_assert!((indexed - wrapped).abs() <= 1e-9 || (indexed - wrapped).abs() >= 360.0 - 1e-9);
        }

        #[test]
        fn unwrap_angle_is_within_180_and_congruent(
            prev in -4000.0f64..4000.0,
            next in -4000.0f64..4000.0,
        ) {
            let result = unwrap_angle(prev, next);
            prop_assert!((result - prev).abs() <= 180.0 + 1e-9);
            let turns = (result - next) / 360.0;
            prop_assert!((turns - turns.round()).abs() <= 1e-9);
        }
    }
}
