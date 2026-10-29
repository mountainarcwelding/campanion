//! `solve_path` round-trips reachable poses and stays continuous on a smooth tool axis.

use super::{FiveAxisSetup, Joints, Kinematics, SolutionPolicy, ToolPose, Vec3, solve_path};

const TABLE_TABLE_AC: &str = include_str!("../kinematics/table_table_ac.json");
const TABLE_TABLE_BC: &str = include_str!("../kinematics/table_table_bc.json");
const HEAD_TABLE_BC: &str = include_str!("../kinematics/head_table_bc.json");
const HEAD_HEAD_AC: &str = include_str!("../kinematics/head_head_ac.json");
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
                let solved = solve_path(&setup(kin.clone()), &[pose], Some(joints))
                    .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} {err}", spec.label)))?;
                proptest::prop_assert_eq!(solved.len(), 1);
                assert_forward(&kin, &pose, &solved[0])?;
                poses.push(pose);
                seeds.push(joints);
            }
            // Unlimited machines accept every branch, so a discontinuous list still solves.
            if spec.spin.is_some() && spec.tilt_min < -160.0 && spec.tilt_max > 160.0 {
                let solved = solve_path(&setup(kin.clone()), &poses, Some(seeds[0]))
                    .map_err(|err| proptest::test_runner::TestCaseError::fail(format!("{} path {err}", spec.label)))?;
                proptest::prop_assert_eq!(solved.len(), poses.len());
                for (pose, joints) in poses.iter().zip(&solved) {
                    assert_forward(&kin, pose, joints)?;
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
            let solved = solve_path(&setup(kin.clone()), &poses, Some(generated[0]))
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
    let solved = solve_path(&setup(kin.clone()), &poses, first).expect("path");
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
    let err = solve_path(&setup(kin), &poses, start).expect_err("limit");
    assert_eq!(err.to_string(), "invalid input: pose 2 leaves axis B");
}

#[test]
fn unreachable_error_names_the_pose_index() {
    let kin = load(B_HEAD);
    let xyz = pt(3.0, 4.0, 5.0);
    let start_joints = Joints { xyz, angles: [10.0, 0.0] };
    let ok = kin.forward(&start_joints, 0.0).expect("forward");
    let bad = ToolPose { tip: xyz, axis: Vec3::Y };
    let err = solve_path(&setup(kin.clone()), &[ok, bad], Some(start_joints)).expect_err("reach");
    assert_eq!(err.to_string(), "invalid input: pose 1 is unreachable");

    let zero = ToolPose { tip: xyz, axis: Vec3::ZERO };
    let err = solve_path(&setup(kin), &[zero], None).expect_err("zero");
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
    let err = solve_path(&setup(kin), &[first, second], Some(start_joints)).expect_err("flip");
    assert_eq!(err.to_string(), "invalid input: pose 1 flips the rotary branch");
}
