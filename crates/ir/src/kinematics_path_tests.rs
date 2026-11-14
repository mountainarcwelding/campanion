//! `solve_path` round-trips reachable poses and stays continuous on a smooth tool axis.

use super::{
    Choice, DEFAULT_SINGULAR_CONE_DEG, FiveAxisSetup, Joints, Kinematics, SolutionPolicy, ToolPose, Vec3, kinematics, solve_path, subdivide_path,
};

const TABLE_TABLE_AC: &str = include_str!("../kinematics/table_table_ac.json");
const TABLE_TABLE_BC: &str = include_str!("../kinematics/table_table_bc.json");
const HEAD_TABLE_BC: &str = include_str!("../kinematics/head_table_bc.json");
const HEAD_HEAD_AC: &str = include_str!("../kinematics/head_head_ac.json");
// Illustrative gantry head. The ±60° limits are example values, not a real machine.
const GANTRY_HEAD_AB: &str = include_str!("../kinematics/gantry_head_ab.json");
const B_HEAD: &str = r#"{
  "name": "b_head",
  "rotaries": [{
    "letter": "B",
    "carrier": "head",
    "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
    "pivot": {"x": 0.0, "y": 0.0, "z": 25.0}
  }],
  "spindle_axis": {"x": 0.0, "y": 0.0, "z": 1.0}
}"#;

/// Tilt is the rotary that is singular at 0°. Spin is the rotary about the tool when tilt is 0.
struct MachineSpec {
    label: &'static str,
    text: &'static str,
    tilt: usize,
    spin: Option<usize>,
    tilt_min: f64,
    tilt_max: f64,
}

fn machines() -> Vec<MachineSpec> {
    vec![
        MachineSpec { label: "table-table", text: TABLE_TABLE_AC, tilt: 0, spin: Some(1), tilt_min: -170.0, tilt_max: 170.0 },
        MachineSpec { label: "table-table", text: TABLE_TABLE_BC, tilt: 0, spin: Some(1), tilt_min: -35.0, tilt_max: 120.0 },
        MachineSpec { label: "head-table", text: HEAD_TABLE_BC, tilt: 1, spin: Some(0), tilt_min: -110.0, tilt_max: 110.0 },
        MachineSpec { label: "head-head", text: HEAD_HEAD_AC, tilt: 1, spin: Some(0), tilt_min: -170.0, tilt_max: 170.0 },
        MachineSpec { label: "mill-turn B head", text: B_HEAD, tilt: 0, spin: None, tilt_min: -170.0, tilt_max: 170.0 },
        // Example ±60° A/B travel; planar cone paths stay well inside those example limits.
        MachineSpec { label: "gantry-head", text: GANTRY_HEAD_AB, tilt: 0, spin: None, tilt_min: -60.0, tilt_max: 60.0 },
    ]
}

fn load(text: &str) -> Kinematics {
    Kinematics::from_json(text).expect("kinematics")
}

fn setup(kin: Kinematics) -> FiveAxisSetup {
    FiveAxisSetup { kinematics: kin, policy: SolutionPolicy::Nearest }
}

fn pt(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z).expect("point")
}

fn lerp(min: f64, max: f64, t: f64) -> f64 {
    min + (max - min) * t
}

fn gap(left: Vec3, right: Vec3) -> f64 {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn axis_angle_rad(left: Vec3, right: Vec3) -> f64 {
    let dot = (left.x() * right.x() + left.y() * right.y() + left.z() * right.z()).clamp(-1.0, 1.0);
    let cross = Vec3::from_xyz(
        left.y() * right.z() - left.z() * right.y(),
        left.z() * right.x() - left.x() * right.z(),
        left.x() * right.y() - left.y() * right.x(),
    );
    let sin = gap(cross, Vec3::ZERO);
    sin.atan2(dot).abs()
}

/// Angle from the tool axis to the nearest singular direction (±Z for these spindles).
fn cone_clearance_deg(axis: Vec3) -> f64 {
    let z = axis.z().clamp(-1.0, 1.0);
    z.acos().to_degrees().min((-z).acos().to_degrees())
}

fn angles_of(spec: &MachineSpec, u: f64, v: f64) -> [f64; 2] {
    let mut angles = [0.0; 2];
    angles[spec.tilt] = lerp(spec.tilt_min, spec.tilt_max, u);
    if let Some(spin) = spec.spin {
        angles[spin] = lerp(-170.0, 170.0, v);
    }
    angles
}

fn assert_forward(kin: &Kinematics, pose: &ToolPose, joints: &Joints) -> Result<(), proptest::test_runner::TestCaseError> {
    let back = kin.forward(joints, 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
    let tip_gap = gap(back.tip, pose.tip);
    let axis_rad = axis_angle_rad(back.axis, pose.axis);
    proptest::prop_assert!(tip_gap <= 1e-9, "{} tip gap {tip_gap}", kin.name);
    proptest::prop_assert!(axis_rad <= 1e-9, "{} axis {axis_rad} rad", kin.name);
    Ok(())
}

fn assert_in_cone(kin: &Kinematics, pose: &ToolPose, joints: &Joints, cone_deg: f64) -> Result<(), proptest::test_runner::TestCaseError> {
    let back = kin.forward(joints, 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
    let tip_gap = gap(back.tip, pose.tip);
    let axis_deg = axis_angle_rad(back.axis, pose.axis).to_degrees();
    proptest::prop_assert!(tip_gap <= 1e-6, "{} tip gap {tip_gap}", kin.name);
    proptest::prop_assert!(axis_deg <= cone_deg + 1e-4, "{} axis error {axis_deg}° cone {cone_deg}", kin.name);
    Ok(())
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(24))]

    #[test]
    fn solve_path_roundtrips_random_reachable_poses(
        u in 0.0f64..1.0,
        v in 0.0f64..1.0,
        x in -200.0f64..200.0,
        y in -200.0f64..200.0,
        z in -200.0f64..200.0,
        u2 in 0.0f64..1.0,
        v2 in 0.0f64..1.0,
        u3 in 0.0f64..1.0,
        v3 in 0.0f64..1.0,
    ) {
        let xyz = pt(x, y, z);
        for spec in machines() {
            let kin = load(spec.text);
            let samples = [angles_of(&spec, u, v), angles_of(&spec, u2, v2), angles_of(&spec, u3, v3)];
            let mut poses = Vec::new();
            let mut seeds = Vec::new();
            for angles in samples {
                let joints = Joints { xyz, angles };
                let pose = kin.forward(&joints, 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
                let solved = solve_path(&setup(kin.clone()), &[pose], Some(joints), DEFAULT_SINGULAR_CONE_DEG)
                    .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
                proptest::prop_assert_eq!(solved.len(), 1);
                assert_forward(&kin, &pose, &solved[0])?;
                poses.push(pose);
                seeds.push(joints);
            }
            // Unlimited machines accept every branch, so a discontinuous list still solves.
            if spec.spin.is_some() && spec.tilt_min < -160.0 && spec.tilt_max > 160.0 {
                let solved = solve_path(&setup(kin.clone()), &poses, Some(seeds[0]), DEFAULT_SINGULAR_CONE_DEG)
                    .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} path {err}", spec.label)))?;
                proptest::prop_assert_eq!(solved.len(), poses.len());
                for (pose, joints) in poses.iter().zip(&solved) {
                    // A discontinuous neighbour can leave this pose inside the cone, where the
                    // primary is held and the axis may move by up to the cone angle. The tip stays.
                    if cone_clearance_deg(pose.axis) <= DEFAULT_SINGULAR_CONE_DEG {
                        assert_in_cone(&kin, pose, joints, DEFAULT_SINGULAR_CONE_DEG)?;
                    } else {
                        assert_forward(&kin, pose, joints)?;
                    }
                }
            }
        }
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(24))]

    /// Orthogonal tilt and spin meet `δ² = Δtilt² + sin²(τ) Δspin²`, where τ is the angle from
    /// the singular axis. Away from that cone, each rotary step is at most `δ / sin(τ)`.
    /// The test allows twice that, because a finite step shares the budget. A single rotary
    /// has no free spin: its step equals the tool-axis step. Neither case may jump by 360°.
    #[test]
    fn smooth_paths_stay_within_the_cone_bound(
        tilt0 in 35.0f64..50.0,
        spin0 in -40.0f64..40.0,
        d_tilt in proptest::collection::vec(-1.5f64..1.5, 4),
        d_spin in proptest::collection::vec(-3.0f64..3.0, 4),
        x in -100.0f64..100.0,
        y in -100.0f64..100.0,
        z in -100.0f64..100.0,
    ) {
        let xyz = pt(x, y, z);
        for spec in machines() {
            let kin = load(spec.text);
            let mut tilt = tilt0;
            let mut spin = spin0;
            let mut poses = Vec::new();
            let mut generated = Vec::new();
            for step in 0..d_tilt.len() {
                if step > 0 {
                    tilt = (tilt + d_tilt[step]).clamp(30.0, 60.0);
                    spin += d_spin[step];
                }
                let mut angles = [0.0; 2];
                angles[spec.tilt] = tilt;
                if let Some(slot) = spec.spin {
                    angles[slot] = spin;
                }
                let joints = Joints { xyz, angles };
                let pose = kin.forward(&joints, 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
                generated.push(joints);
                poses.push(pose);
            }
            let solved = solve_path(&setup(kin.clone()), &poses, Some(generated[0]), DEFAULT_SINGULAR_CONE_DEG)
                .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
            proptest::prop_assert_eq!(solved.len(), poses.len());
            let mut max_step: f64 = 0.0;
            let mut min_cone = f64::INFINITY;
            for (index, pose) in poses.iter().enumerate() {
                assert_forward(&kin, pose, &solved[index])?;
                min_cone = min_cone.min(cone_clearance_deg(pose.axis));
                if index > 0 {
                    max_step = max_step.max(axis_angle_rad(poses[index - 1].axis, pose.axis).to_degrees());
                }
            }
            // The walk keeps the tool axis at least 30° from ±Z, so sin(cone) stays away from 0.
            proptest::prop_assert!(min_cone >= 20.0, "{} cone {min_cone}", spec.label);
            let bound = if spec.spin.is_some() {
                // δ / sin(cone), doubled for a finite step that moves both joints.
                max_step / min_cone.to_radians().sin() * 2.0
            } else {
                // One rotary: the joint travels with the tool axis. No 360° wind-up.
                max_step + 1e-6
            };
            proptest::prop_assert!(bound < 180.0, "{} bound {bound} step {max_step} cone {min_cone}", spec.label);
            for pair in solved.windows(2) {
                for slot in 0..kin.rotaries.len() {
                    let delta = (pair[1].angles[slot] - pair[0].angles[slot]).abs();
                    proptest::prop_assert!(
                        delta <= bound + 1e-4,
                        "{} {} joint {slot} jumped {delta}° (bound {bound}, step {max_step}, cone {min_cone})",
                        spec.label,
                        kin.name
                    );
                    proptest::prop_assert!(delta < 180.0, "{} rotary jump {delta}", spec.label);
                }
            }
        }
    }
}

#[test]
fn continuous_c_passes_180_without_a_360_jump() {
    let kin = load(TABLE_TABLE_AC);
    let xyz = pt(4.0, -6.0, 8.0);
    // 20° steps. 190 is past 180, so a folded angle would jump. 370 and 390 are past 360.
    assert_unwound_c(&kin, xyz, &[150.0, 170.0, 190.0, 210.0]);
    assert_unwound_c(&kin, xyz, &[330.0, 350.0, 370.0, 390.0]);
}

fn assert_unwound_c(kin: &Kinematics, xyz: Vec3, spins: &[f64]) {
    let mut poses = Vec::new();
    let mut first = None;
    for c in spins {
        let joints = Joints { xyz, angles: [25.0, *c] };
        poses.push(kin.forward(&joints, 0.0).expect("forward"));
        if first.is_none() {
            first = Some(joints);
        }
    }
    let solved = solve_path(&setup(kin.clone()), &poses, first, DEFAULT_SINGULAR_CONE_DEG).expect("path");
    let got: Vec<f64> = solved.iter().map(|joints| joints.angles[1]).collect();
    for (index, pair) in got.windows(2).enumerate() {
        let delta = (pair[1] - pair[0]).abs();
        assert!(delta < 30.0, "step {index} spins {got:?}");
        assert!(delta < 180.0, "360° jump at {index}: {got:?}");
    }
    for (got_c, want_c) in got.iter().zip(spins) {
        assert!((got_c - want_c).abs() < 1e-4, "got {got:?} want {spins:?}");
    }
}

#[test]
fn limit_error_names_the_first_pose_and_the_axis() {
    let kin = load(TABLE_TABLE_BC);
    let xyz = pt(1.0, 2.0, 3.0);
    let mut poses = Vec::new();
    let mut start = None;
    for (index, b) in [0.0, 20.0, 130.0].into_iter().enumerate() {
        let joints = Joints { xyz, angles: [b, 5.0 * index as f64] };
        poses.push(kin.forward(&joints, 0.0).expect("forward"));
        if start.is_none() {
            start = Some(joints);
        }
    }
    let err = solve_path(&setup(kin), &poses, start, DEFAULT_SINGULAR_CONE_DEG).expect_err("limit");
    assert_eq!(err.to_string(), "invalid input: pose 2 leaves axis B");
}

#[test]
fn unreachable_error_names_the_pose_index() {
    let kin = load(B_HEAD);
    let xyz = pt(3.0, 4.0, 5.0);
    let start_joints = Joints { xyz, angles: [10.0, 0.0] };
    let ok = kin.forward(&start_joints, 0.0).expect("forward");
    let bad = ToolPose { tip: xyz, axis: Vec3::Y };
    let err = solve_path(&setup(kin.clone()), &[ok, bad], Some(start_joints), DEFAULT_SINGULAR_CONE_DEG).expect_err("reach");
    assert_eq!(err.to_string(), "invalid input: pose 1 is unreachable");

    let zero = ToolPose { tip: xyz, axis: Vec3::ZERO };
    let err = solve_path(&setup(kin), &[zero], None, DEFAULT_SINGULAR_CONE_DEG).expect_err("zero");
    assert_eq!(err.to_string(), "invalid input: pose 0 is unreachable");
}

#[test]
fn only_the_other_branch_inside_the_limits_is_an_error() {
    let text = r#"{
      "name": "asymmetric_bc",
      "rotaries": [
        {
          "letter": "B",
          "carrier": "table",
          "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
          "pivot": {"x": 0.0, "y": 0.0, "z": 0.0},
          "limits": {"min_deg": -80.0, "max_deg": 40.0}
        },
        {
          "letter": "C",
          "carrier": "table",
          "axis": {"x": 0.0, "y": 0.0, "z": 1.0},
          "pivot": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
      ]
    }"#;
    let kin = load(text);
    let xyz = pt(2.0, 0.0, 4.0);
    let start_joints = Joints { xyz, angles: [20.0, 0.0] };
    let first = kin.forward(&start_joints, 0.0).expect("first");
    // B = 55 is past max 40. The flipped branch is near B = -55, which is inside -80.
    let second_joints = Joints { xyz, angles: [55.0, 5.0] };
    let second = kin.forward(&second_joints, 0.0).expect("second");
    let err = solve_path(&setup(kin), &[first, second], Some(start_joints), DEFAULT_SINGULAR_CONE_DEG).expect_err("flip");
    assert_eq!(err.to_string(), "invalid input: pose 1 flips the rotary branch");
}

/// Largest primary-rotary step, in degrees, on a move that enters or leaves the singular cone.
///
/// Inside the cone the primary is held, so that step is 0. Just outside, the two exact branches
/// are a half-turn apart on the primary. With the held tilt still inside the cone, the nearer
/// branch stays within `90 + 2 * singular_cone_deg` of the held primary.
fn primary_crossing_bound_deg(cone_deg: f64) -> f64 {
    90.0 + 2.0 * cone_deg
}

/// The midpoint of a subdivided segment meets the tolerance. A sample at 1/4 or 3/4 meets twice
/// that tolerance: the split only enforces the midpoint, and the joint-space arc stays inside
/// twice the midpoint allowance away from it.
const OFF_MIDPOINT_FACTOR: f64 = 2.0;

fn tilted_axis(tilt_deg: f64, azimuth_deg: f64) -> Vec3 {
    let (tilt_sin, tilt_cos) = tilt_deg.to_radians().sin_cos();
    let (az_sin, az_cos) = azimuth_deg.to_radians().sin_cos();
    Vec3::from_xyz(tilt_sin * az_cos, tilt_sin * az_sin, tilt_cos)
}

fn cone_path(cone: f64, extra: f64, azimuth0: f64, azimuth1: f64, xyz: Vec3, planar: bool) -> Vec<ToolPose> {
    let outside = cone + extra;
    let tilts = [outside, outside * 0.55, cone * 0.65, 0.0, -cone * 0.4, -outside * 0.55, -outside];
    let last = (tilts.len() - 1) as f64;
    tilts
        .into_iter()
        .enumerate()
        .map(|(index, tilt)| {
            let azimuth = if planar { 0.0 } else { azimuth0 + (azimuth1 - azimuth0) * (index as f64 / last) };
            ToolPose { tip: xyz, axis: tilted_axis(tilt, azimuth) }
        })
        .collect()
}

fn assert_cone_path(spec: &MachineSpec, cone: f64, poses: &[ToolPose]) -> Result<(), proptest::test_runner::TestCaseError> {
    let kin = load(spec.text);
    let seed = kin
        .choose(&poses[0], 0.0, Choice::Nearest([0.0, 0.0]))
        .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
    let solved = solve_path(&setup(kin.clone()), poses, Some(seed), cone)
        .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
    proptest::prop_assert_eq!(solved.len(), poses.len());
    for (pose, joints) in poses.iter().zip(&solved) {
        if cone_clearance_deg(pose.axis) <= cone {
            assert_in_cone(&kin, pose, joints, cone)?;
        } else {
            assert_forward(&kin, pose, joints)?;
        }
    }
    let Some(spin) = spec.spin else {
        for pair in solved.windows(2) {
            let delta = (pair[1].angles[spec.tilt] - pair[0].angles[spec.tilt]).abs();
            proptest::prop_assert!(delta < 180.0, "{} rotary jump {delta}", spec.label);
        }
        return Ok(());
    };
    let bound = primary_crossing_bound_deg(cone);
    for (joints, poses) in solved.windows(2).zip(poses.windows(2)) {
        let delta = (joints[1].angles[spin] - joints[0].angles[spin]).abs();
        let entered = cone_clearance_deg(poses[0].axis) <= cone;
        let left = cone_clearance_deg(poses[1].axis) <= cone;
        if entered && left {
            proptest::prop_assert!(delta <= 1e-6, "{} held primary moved {delta}°", spec.label);
        }
        if entered != left {
            proptest::prop_assert!(delta <= bound + 1e-3, "{} crossed the cone by {delta}° (bound {bound})", spec.label);
        }
        proptest::prop_assert!(delta < 180.0, "{} primary jump {delta}", spec.label);
    }
    Ok(())
}

#[test]
fn inside_the_cone_the_tool_axis_stays_within_the_cone_angle() {
    let cone = 1.0;
    // 0.4° from +Z, a quarter-turn away from the pose that seeds the primary.
    let xyz = pt(30.0, -12.0, 8.0);
    for spec in machines() {
        if spec.spin.is_none() {
            continue;
        }
        let kin = load(spec.text);
        let outside = ToolPose { tip: xyz, axis: tilted_axis(6.0, 10.0) };
        let inside = ToolPose { tip: xyz, axis: tilted_axis(0.4, 100.0) };
        let seed = kin.choose(&outside, 0.0, Choice::Nearest([0.0, 0.0])).expect("seed");
        let solved = solve_path(&setup(kin.clone()), &[outside, inside], Some(seed), cone).expect("path");
        let spin = spec.spin.expect("spin");
        let held = (solved[1].angles[spin] - solved[0].angles[spin]).abs();
        assert!(held <= 1e-6, "{} primary moved {held}", spec.label);
        let back = kin.forward(&solved[1], 0.0).expect("forward");
        let axis_deg = axis_angle_rad(back.axis, inside.axis).to_degrees();
        assert!(axis_deg <= cone + 1e-4, "{} axis error {axis_deg}°", spec.label);
        assert!(gap(back.tip, inside.tip) <= 1e-6, "{} tip", spec.label);
        // The request is 0.4° from the pole, so the locked meridian cannot be farther than that.
        assert!(axis_deg <= 0.4 + 1e-3, "{} axis error {axis_deg}° exceeds the pole clearance", spec.label);
    }
}

#[test]
fn entering_and_leaving_the_cone_keeps_the_primary_within_the_bound() {
    let cone = 1.0;
    let bound = primary_crossing_bound_deg(cone);
    let xyz = pt(-20.0, 15.0, 4.0);
    // Azimuth sweeps 140° while the tilt passes through the cone.
    let poses = cone_path(cone, 6.0, -20.0, 120.0, xyz, false);
    for spec in machines() {
        if spec.spin.is_none() {
            let planar = cone_path(cone, 6.0, 0.0, 0.0, xyz, true);
            assert_cone_path(&spec, cone, &planar).unwrap_or_else(|err| panic!("{} {err:?}", spec.label));
            continue;
        }
        assert_cone_path(&spec, cone, &poses).unwrap_or_else(|err| panic!("{} {err:?}", spec.label));
        let kin = load(spec.text);
        let seed = kin.choose(&poses[0], 0.0, Choice::Nearest([0.0, 0.0])).expect("seed");
        let solved = solve_path(&setup(kin), &poses, Some(seed), cone).expect("path");
        let spin = spec.spin.expect("spin");
        let mut crossed = false;
        for (joints, ends) in solved.windows(2).zip(poses.windows(2)) {
            let was_inside = cone_clearance_deg(ends[0].axis) <= cone;
            let now_inside = cone_clearance_deg(ends[1].axis) <= cone;
            if was_inside == now_inside {
                continue;
            }
            crossed = true;
            let delta = (joints[1].angles[spin] - joints[0].angles[spin]).abs();
            assert!(delta <= bound + 1e-3, "{} ΔC {delta}° bound {bound}", spec.label);
        }
        assert!(crossed, "{} never crossed the cone", spec.label);
    }
}

#[test]
fn singular_cone_rejects_values_outside_the_open_range() {
    let kin = load(HEAD_TABLE_BC);
    for bad in [0.0, -1.0, 10.0 + 1e-6, f64::NAN, f64::INFINITY] {
        let err = solve_path(&setup(kin.clone()), &[], None, bad).expect_err("cone");
        assert_eq!(err.to_string(), "invalid input: singular_cone_deg must be in (0, 10]");
    }
    solve_path(&setup(kin), &[], None, 10.0).expect("upper end");
    solve_path(&setup(load(B_HEAD)), &[], None, DEFAULT_SINGULAR_CONE_DEG).expect("default");
}

fn lerp_joints(left: Joints, right: Joints, t: f64) -> Joints {
    let mix = |a: f64, b: f64| a + (b - a) * t;
    Joints {
        xyz: Vec3::from_xyz(mix(left.xyz.x(), right.xyz.x()), mix(left.xyz.y(), right.xyz.y()), mix(left.xyz.z(), right.xyz.z())),
        angles: [mix(left.angles[0], right.angles[0]), mix(left.angles[1], right.angles[1])],
    }
}

fn assert_samples(
    kin: &Kinematics,
    poses: &[ToolPose],
    joints: &[Joints],
    chord_tol_mm: f64,
    axis_tol_deg: f64,
) -> Result<(), proptest::test_runner::TestCaseError> {
    proptest::prop_assert_eq!(poses.len(), joints.len());
    for (pose, joints) in poses.iter().zip(joints) {
        assert_forward(kin, pose, joints)?;
    }
    for (ends, ends_j) in poses.windows(2).zip(joints.windows(2)) {
        for (t, factor) in [(0.25, OFF_MIDPOINT_FACTOR), (0.5, 1.0), (0.75, OFF_MIDPOINT_FACTOR)] {
            let mid =
                kin.forward(&lerp_joints(ends_j[0], ends_j[1], t), 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
            let tip = Vec3::from_xyz(
                ends[0].tip.x() + (ends[1].tip.x() - ends[0].tip.x()) * t,
                ends[0].tip.y() + (ends[1].tip.y() - ends[0].tip.y()) * t,
                ends[0].tip.z() + (ends[1].tip.z() - ends[0].tip.z()) * t,
            );
            let tip_gap = gap(mid.tip, tip);
            let axis_deg = axis_angle_rad(mid.axis, kinematics::slerp_axis(ends[0].axis, ends[1].axis, t)).to_degrees();
            let chord_bound = chord_tol_mm * factor + 1e-6;
            let axis_bound = axis_tol_deg * factor + 1e-4;
            proptest::prop_assert!(tip_gap <= chord_bound, "{} t {t} chord {tip_gap} bound {chord_bound}", kin.name);
            proptest::prop_assert!(axis_deg <= axis_bound, "{} t {t} axis {axis_deg}° bound {axis_bound}", kin.name);
        }
    }
    Ok(())
}

#[test]
fn constant_joints_insert_nothing() {
    for spec in machines() {
        let kin = load(spec.text);
        let joints = Joints { xyz: pt(12.0, -3.0, 8.0), angles: angles_of(&spec, 0.4, 0.3) };
        let pose = kin.forward(&joints, 0.0).expect("forward");
        let poses = vec![pose, pose, pose];
        let joints = vec![joints, joints, joints];
        let (out_poses, out_joints) = subdivide_path(&setup(kin.clone()), spec.label, &poses, &joints, 1.0e-4, 1.0e-3).expect("subdivide");
        assert_eq!(out_poses.len(), poses.len(), "{}", spec.label);
        assert_eq!(out_joints.len(), joints.len(), "{}", spec.label);
        assert_samples(&kin, &out_poses, &out_joints, 1.0e-4, 1.0e-3).unwrap_or_else(|err| panic!("{} {err:?}", spec.label));
    }
}

#[test]
fn fixed_angles_are_not_subdivided() {
    let kin = load(TABLE_TABLE_AC);
    let start = Joints { xyz: pt(-40.0, 10.0, 5.0), angles: [25.0, -15.0] };
    let end = Joints { xyz: pt(70.0, -20.0, 12.0), angles: [25.0, -15.0] };
    let poses = [kin.forward(&start, 0.0).expect("start"), kin.forward(&end, 0.0).expect("end")];
    let (out_poses, out_joints) = subdivide_path(&setup(kin), "translate", &poses, &[start, end], 1.0e-6, 1.0e-4).expect("subdivide");
    assert_eq!(out_poses.len(), 2);
    assert_eq!(out_joints.len(), 2);
}

#[test]
fn an_arc_about_the_table_is_subdivided_until_the_chord_fits() {
    let kin = load(HEAD_TABLE_BC);
    let start = Joints { xyz: pt(150.0, 0.0, 20.0), angles: [0.0, 20.0] };
    let end = Joints { xyz: pt(150.0, 0.0, 20.0), angles: [80.0, 20.0] };
    let poses = [kin.forward(&start, 0.0).expect("start"), kin.forward(&end, 0.0).expect("end")];
    let (out_poses, out_joints) = subdivide_path(&setup(kin.clone()), "arc", &poses, &[start, end], 0.05, 0.2).expect("subdivide");
    assert!(out_poses.len() > 2, "inserts {}", out_poses.len());
    assert_samples(&kin, &out_poses, &out_joints, 0.05, 0.2).unwrap_or_else(|err| panic!("{err:?}"));
}

#[test]
fn subdivision_cap_names_the_toolpath() {
    let kin = load(HEAD_TABLE_BC);
    let start = Joints { xyz: pt(180.0, 20.0, 15.0), angles: [0.0, 0.0] };
    let end = Joints { xyz: pt(180.0, 20.0, 15.0), angles: [120.0, 0.0] };
    let poses = [kin.forward(&start, 0.0).expect("start"), kin.forward(&end, 0.0).expect("end")];
    let err = subdivide_path(&setup(kin), "bowl", &poses, &[start, end], 1.0e-9, 0.5).expect_err("cap");
    assert_eq!(err.to_string(), "invalid input: toolpath `bowl` inserts more than 10000 poses");
}

#[test]
fn subdivision_rejects_a_bad_tolerance_and_a_length_mismatch() {
    let kin = load(B_HEAD);
    let joints = Joints { xyz: pt(1.0, 2.0, 3.0), angles: [10.0, 0.0] };
    let pose = kin.forward(&joints, 0.0).expect("forward");
    let err = subdivide_path(&setup(kin.clone()), "finish", &[pose], &[joints], 0.0, 0.1).expect_err("chord");
    assert_eq!(err.to_string(), "invalid input: chord_tol_mm must be positive");
    let err = subdivide_path(&setup(kin.clone()), "finish", &[pose], &[joints], 0.1, f64::NAN).expect_err("axis");
    assert_eq!(err.to_string(), "invalid input: axis_tol_deg must be positive");
    let err = subdivide_path(&setup(kin), "finish", &[pose, pose], &[joints], 0.1, 0.1).expect_err("len");
    assert_eq!(err.to_string(), "invalid input: toolpath `finish` poses and joints differ in length");
}

#[test]
fn slerp_bisects_a_right_angle() {
    let mid = kinematics::slerp_axis(Vec3::Z, Vec3::X, 0.5);
    let bisector = Vec3::from_xyz(1.0, 0.0, 1.0);
    assert!(axis_angle_rad(mid, bisector) < 1e-9);
    assert!(axis_angle_rad(kinematics::slerp_axis(Vec3::Z, Vec3::X, 0.0), Vec3::Z) < 1e-12);
    assert!(axis_angle_rad(kinematics::slerp_axis(Vec3::Z, Vec3::X, 1.0), Vec3::X) < 1e-12);
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(24))]

    #[test]
    fn paths_through_the_singular_cone_hold_the_primary_and_keep_the_axis(
        cone in 0.25f64..6.0,
        extra in 2.0f64..12.0,
        azimuth0 in -140.0f64..140.0,
        sweep in -100.0f64..100.0,
        x in -80.0f64..80.0,
        y in -80.0f64..80.0,
        z in -40.0f64..40.0,
    ) {
        let xyz = pt(x, y, z);
        let poses = cone_path(cone, extra, azimuth0, azimuth0 + sweep, xyz, false);
        let planar = cone_path(cone, extra, 0.0, 0.0, xyz, true);
        for spec in machines() {
            if spec.spin.is_none() {
                assert_cone_path(&spec, cone, &planar)?;
            } else {
                assert_cone_path(&spec, cone, &poses)?;
            }
        }
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(24))]

    #[test]
    fn subdivided_paths_meet_the_chord_and_axis_tolerances(
        u0 in 0.0f64..1.0,
        v0 in 0.0f64..1.0,
        u1 in 0.0f64..1.0,
        v1 in 0.0f64..1.0,
        u2 in 0.0f64..1.0,
        v2 in 0.0f64..1.0,
        u3 in 0.0f64..1.0,
        v3 in 0.0f64..1.0,
        chord_tol in 0.05f64..1.5,
        axis_tol in 0.1f64..2.0,
        x0 in -60.0f64..60.0,
        y0 in -60.0f64..60.0,
        z0 in -30.0f64..30.0,
        x1 in -60.0f64..60.0,
        y1 in -60.0f64..60.0,
        z1 in -30.0f64..30.0,
        x2 in -60.0f64..60.0,
        y2 in -60.0f64..60.0,
        z2 in -30.0f64..30.0,
        x3 in -60.0f64..60.0,
        y3 in -60.0f64..60.0,
        z3 in -30.0f64..30.0,
    ) {
        let samples = [
            (u0, v0, pt(x0, y0, z0)),
            (u1, v1, pt(x1, y1, z1)),
            (u2, v2, pt(x2, y2, z2)),
            (u3, v3, pt(x3, y3, z3)),
        ];
        for spec in machines() {
            let kin = load(spec.text);
            let mut poses = Vec::new();
            let mut joints = Vec::new();
            for (u, v, xyz) in samples {
                let sample = Joints { xyz, angles: angles_of(&spec, u, v) };
                let pose = kin.forward(&sample, 0.0).map_err(|err| proptest::test_runner::TestCaseError::fail(err.to_string()))?;
                poses.push(pose);
                joints.push(sample);
            }
            let (out_poses, out_joints) = subdivide_path(&setup(kin.clone()), spec.label, &poses, &joints, chord_tol, axis_tol)
                .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
            proptest::prop_assert!(out_poses.len() >= poses.len());
            assert_samples(&kin, &out_poses, &out_joints, chord_tol, axis_tol)?;
        }
    }
}
