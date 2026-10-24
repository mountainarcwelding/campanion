//! Forward kinematics of the illustrative joint files.

use super::{Carrier, Choice, IrError, Joints, Kinematics, ToolPose, Vec3};

const TABLE_TABLE_AC: &str = include_str!("../kinematics/table_table_ac.json");
const TABLE_TABLE_BC: &str = include_str!("../kinematics/table_table_bc.json");
const HEAD_TABLE_BC: &str = include_str!("../kinematics/head_table_bc.json");
const HEAD_HEAD_AC: &str = include_str!("../kinematics/head_head_ac.json");

fn load(text: &str) -> Kinematics {
    Kinematics::from_json(text).expect("kinematics")
}

fn joints(xyz: Vec3, angles: [f64; 2]) -> Joints {
    Joints { xyz, angles }
}

fn pt(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z).expect("point")
}

fn close(left: Vec3, right: Vec3) {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    let gap = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(gap <= 1e-9, "left=({} {} {}) right=({} {} {}) gap={gap}", left.x(), left.y(), left.z(), right.x(), right.y(), right.z());
}

fn assert_unit(axis: Vec3) {
    let len = (axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z()).sqrt();
    assert!((len - 1.0).abs() <= 1e-9, "axis length {len}");
    assert!(axis.x().is_finite() && axis.y().is_finite() && axis.z().is_finite());
}

fn invalid(message: &str) -> IrError {
    IrError::InvalidInput(message.to_string())
}

fn gap(left: Vec3, right: Vec3) -> f64 {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn angles_near(left: [f64; 2], right: [f64; 2], tol: f64) -> bool {
    (left[0] - right[0]).abs() <= tol && (left[1] - right[1]).abs() <= tol
}

fn assert_inverse_contains(kin: &Kinematics, xyz: Vec3, angles: [f64; 2], tool_length: f64) {
    let pose = kin.forward(&joints(xyz, angles), tool_length).expect("forward");
    let sols = kin.inverse(&pose, tool_length, angles).expect("inverse");
    assert!(
        sols.iter().any(|sol| angles_near(sol.angles, angles, 1e-7)),
        "{} angles {angles:?} not in {:?}",
        kin.name,
        sols.iter().map(|sol| sol.angles).collect::<Vec<_>>()
    );
    for sol in &sols {
        let back = kin.forward(sol, tool_length).expect("back");
        let tip_gap = gap(back.tip, pose.tip);
        let axis_gap = gap(back.axis, pose.axis);
        assert!(tip_gap <= 1e-6, "{} tip gap {tip_gap} angles {:?}", kin.name, sol.angles);
        assert!(axis_gap <= 1e-9, "{} axis gap {axis_gap} angles {:?}", kin.name, sol.angles);
    }
}

#[test]
fn examples_load_with_their_joints() {
    let ac = load(TABLE_TABLE_AC);
    assert_eq!(ac.name, "table_table_ac");
    assert_eq!(ac.rotaries.len(), 2);
    assert_eq!(ac.rotaries[0].letter, 'A');
    assert_eq!(ac.rotaries[0].carrier, Carrier::Table);
    assert_eq!(ac.rotaries[0].axis, Vec3::X);
    assert!(ac.rotaries[0].limits.is_none());
    assert_eq!(ac.rotaries[1].letter, 'C');
    assert_eq!(ac.rotaries[1].carrier, Carrier::Table);
    assert_eq!(ac.rotaries[1].pivot, pt(0.0, 120.0, 50.0));
    assert_eq!(ac.spindle_axis, Vec3::Z);

    let bc = load(TABLE_TABLE_BC);
    assert_eq!(bc.name, "table_table_bc");
    assert_eq!(bc.rotaries[0].letter, 'B');
    assert_eq!(bc.rotaries[0].carrier, Carrier::Table);
    let limits = bc.rotaries[0].limits.expect("B limits");
    assert_eq!(limits.min_deg(), -35.0);
    assert_eq!(limits.max_deg(), 120.0);
    assert_eq!(bc.rotaries[1].letter, 'C');
    assert!(bc.rotaries[1].limits.is_none());
    assert_eq!(bc.rotaries[1].pivot, pt(60.0, 0.0, 30.0));

    let mill = load(HEAD_TABLE_BC);
    assert_eq!(mill.name, "head_table_bc");
    assert_eq!(mill.rotaries[0].letter, 'C');
    assert_eq!(mill.rotaries[0].carrier, Carrier::Table);
    assert!(mill.rotaries[0].limits.is_none());
    assert_eq!(mill.rotaries[1].letter, 'B');
    assert_eq!(mill.rotaries[1].carrier, Carrier::Head);
    assert_eq!(mill.rotaries[1].pivot, pt(0.0, 0.0, 150.0));
    let b_limits = mill.rotaries[1].limits.expect("B limits");
    assert_eq!((b_limits.min_deg(), b_limits.max_deg()), (-110.0, 110.0));

    let head = load(HEAD_HEAD_AC);
    assert_eq!(head.name, "head_head_ac");
    assert_eq!(head.rotaries[0].letter, 'C');
    assert_eq!(head.rotaries[0].carrier, Carrier::Head);
    assert_eq!(head.rotaries[0].pivot, pt(0.0, 0.0, 80.0));
    assert_eq!(head.rotaries[1].letter, 'A');
    assert_eq!(head.rotaries[1].carrier, Carrier::Head);
    assert_eq!(head.rotaries[1].pivot, pt(0.0, 40.0, 120.0));
    assert_eq!(head.spindle_axis, Vec3::Z);
}

#[test]
fn zero_angles_are_the_identity_pose() {
    let xyz = pt(5.0, 6.0, 7.0);
    let tool_length = 10.0;
    for text in [TABLE_TABLE_AC, TABLE_TABLE_BC, HEAD_TABLE_BC, HEAD_HEAD_AC] {
        let kin = load(text);
        let pose = kin.forward(&joints(xyz, [0.0, 0.0]), tool_length).expect("forward");
        close(pose.tip, pt(5.0, 6.0, -3.0));
        close(pose.axis, Vec3::Z);
        assert_unit(pose.axis);
    }
}

#[test]
fn omitted_spindle_axis_defaults_to_positive_z_and_scaled_axes_normalise() {
    let text = r#"
    {
      "name": "scaled",
      "rotaries": [
        {
          "letter": "B",
          "carrier": "table",
          "axis": { "x": 0.0, "y": 4.0, "z": 0.0 },
          "pivot": { "x": 1.0, "y": 2.0, "z": 3.0 }
        }
      ],
      "spindle_axis": { "x": 0.0, "y": 0.0, "z": 5.0 }
    }
    "#;
    let kin = load(text);
    assert_eq!(kin.rotaries[0].axis, Vec3::Y);
    assert_eq!(kin.spindle_axis, Vec3::Z);

    let bare = r#"
    {
      "name": "bare",
      "rotaries": [
        {
          "letter": "C",
          "carrier": "table",
          "axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
          "pivot": { "x": 0.0, "y": 0.0, "z": 0.0 }
        }
      ]
    }
    "#;
    let bare = load(bare);
    assert_eq!(bare.spindle_axis, Vec3::Z);
    let pose = bare.forward(&joints(pt(5.0, 6.0, 7.0), [0.0, 40.0]), 10.0).expect("forward");
    close(pose.tip, pt(5.0, 6.0, -3.0));
    close(pose.axis, Vec3::Z);
}

#[test]
fn spindle_axis_sets_the_zero_pose() {
    let text = r#"
    {
      "name": "side",
      "rotaries": [
        {
          "letter": "A",
          "carrier": "table",
          "axis": { "x": 2.0, "y": 0.0, "z": 0.0 },
          "pivot": { "x": 0.0, "y": 0.0, "z": 0.0 }
        }
      ],
      "spindle_axis": { "x": 0.0, "y": 3.0, "z": 0.0 }
    }
    "#;
    let kin = load(text);
    assert_eq!(kin.spindle_axis, Vec3::Y);
    let pose = kin.forward(&joints(pt(5.0, 6.0, 7.0), [0.0, 0.0]), 10.0).expect("forward");
    close(pose.tip, pt(5.0, -4.0, 7.0));
    close(pose.axis, Vec3::Y);
}

#[test]
fn table_ac_at_a90_maps_the_tool_axis_to_positive_y() {
    // A is on the base, about X. C is carried and held at 0, so T = R_A(90°).
    // rot(T)^-1 * +Z = R_X(-90°) * +Z = +Y.
    // xyz (10, 20, 30), L = 50 → tip_machine (10, 20, -20) → R_X(-90°) → (10, -20, -20).
    let kin = load(TABLE_TABLE_AC);
    let pose = kin.forward(&joints(pt(10.0, 20.0, 30.0), [90.0, 0.0]), 50.0).expect("forward");
    close(pose.axis, Vec3::Y);
    close(pose.tip, pt(10.0, -20.0, -20.0));
    assert_unit(pose.axis);
}

#[test]
fn table_ac_composes_carried_c_about_its_zero_pose_pivot() {
    // A = 90° about X through the origin, C = 90° about Z through (0, 120, 50).
    // L = 0, xyz (100, 0, 0). tip_part = R_C(-90°) * R_A(-90°) * tip_machine = (-120, 20, 0).
    // axis_part = R_C(-90°) * R_A(-90°) * +Z = +X.
    let kin = load(TABLE_TABLE_AC);
    let pose = kin.forward(&joints(pt(100.0, 0.0, 0.0), [90.0, 90.0]), 0.0).expect("forward");
    close(pose.axis, Vec3::X);
    close(pose.tip, pt(-120.0, 20.0, 0.0));
}

#[test]
fn head_table_b90_tilts_the_tool_axis_to_positive_x() {
    // B is a head joint about +Y. Right-hand 90° sends +Z to +X. C is held at 0, so the table
    // does not move the pose. Pivot (0, 0, 150), L = 100, xyz (0, 0, 200):
    // H(0, 0, -100) = (-250, 0, 150), tip_machine = (-250, 0, 350).
    let kin = load(HEAD_TABLE_BC);
    let pose = kin.forward(&joints(pt(0.0, 0.0, 200.0), [0.0, 90.0]), 100.0).expect("forward");
    close(pose.axis, Vec3::X);
    close(pose.tip, pt(-250.0, 0.0, 350.0));
    assert_unit(pose.axis);
}

#[test]
fn head_joint_order_does_not_depend_on_interleaving_with_the_table() {
    let swapped = r#"
    {
      "name": "head_table_bc",
      "spindle_axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
      "rotaries": [
        {
          "letter": "B",
          "carrier": "head",
          "axis": { "x": 0.0, "y": 1.0, "z": 0.0 },
          "pivot": { "x": 0.0, "y": 0.0, "z": 150.0 },
          "limits": { "min_deg": -110.0, "max_deg": 110.0 }
        },
        {
          "letter": "C",
          "carrier": "table",
          "axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
          "pivot": { "x": 0.0, "y": 0.0, "z": 0.0 }
        }
      ]
    }
    "#;
    let listed_c_first = load(HEAD_TABLE_BC);
    let listed_b_first = load(swapped);
    let xyz = pt(12.0, -3.0, 40.0);
    let pose_c_first = listed_c_first.forward(&joints(xyz, [25.0, -40.0]), 18.0).expect("c first");
    let pose_b_first = listed_b_first.forward(&joints(xyz, [-40.0, 25.0]), 18.0).expect("b first");
    close(pose_c_first.tip, pose_b_first.tip);
    close(pose_c_first.axis, pose_b_first.axis);
}

#[test]
fn head_head_applies_the_carried_joint_first() {
    // C is on the Z slide (outer). A is carried (inner), pivot (0, 40, 120).
    // C = 90°, A = 90°, L = 100, xyz = 0.
    // Inner A first, then outer C: tip = (-260, 0, 80), axis = +X.
    let kin = load(HEAD_HEAD_AC);
    let pose = kin.forward(&joints(Vec3::ZERO, [90.0, 90.0]), 100.0).expect("forward");
    close(pose.tip, pt(-260.0, 0.0, 80.0));
    close(pose.axis, Vec3::X);
}

#[test]
fn check_errors_are_exact() {
    let joint = |letter: &str, axis: &str| format!(r#"{{"letter":{letter},"carrier":"table","axis":{axis},"pivot":{{"x":0.0,"y":0.0,"z":0.0}}}}"#);
    let body = |rotaries: &str, spindle: &str| format!(r#"{{"name":"demo","rotaries":[{rotaries}],"spindle_axis":{spindle}}}"#);
    let axis = r#"{"x":0.0,"y":0.0,"z":1.0}"#;
    let cases = [
        (r#"{"name":"","rotaries":[]}"#.to_string(), "kinematics name is empty"),
        (body("", axis), "kinematics `demo`: needs one or two rotary joints"),
        (
            body(&format!("{},{},{}", joint(r#""A""#, axis), joint(r#""B""#, axis), joint(r#""C""#, axis)), axis),
            "kinematics `demo`: needs one or two rotary joints",
        ),
        (body(&joint(r#""D""#, axis), axis), "kinematics `demo`: rotary letter must be A, B or C"),
        (body(&format!("{},{}", joint(r#""A""#, axis), joint(r#""A""#, axis)), axis), "kinematics `demo`: rotary A listed twice"),
        (body(&joint(r#""B""#, r#"{"x":0.0,"y":0.0,"z":0.0}"#), axis), "kinematics `demo`: rotary B axis is zero"),
        (body(&joint(r#""A""#, axis), r#"{"x":0.0,"y":0.0,"z":0.0}"#), "kinematics `demo`: spindle axis is zero"),
    ];
    for (text, message) in cases {
        let err = Kinematics::from_json(&text).expect_err(message);
        assert_eq!(err, invalid(message), "{text}");
    }

    let bad_limits = body(
        &format!(r#"{{"letter":"A","carrier":"head","axis":{axis},"pivot":{{"x":0.0,"y":0.0,"z":0.0}},"limits":{{"min_deg":10.0,"max_deg":10.0}}}}"#),
        axis,
    );
    let err = Kinematics::from_json(&bad_limits).expect_err("limits");
    assert_eq!(err, invalid("rotary limit minimum is not less than maximum"));

    let nan_limit = body(
        &format!(r#"{{"letter":"C","carrier":"table","axis":{axis},"pivot":{{"x":0.0,"y":0.0,"z":0.0}},"limits":{{"min_deg":null,"max_deg":1.0}}}}"#),
        axis,
    );
    assert!(matches!(Kinematics::from_json(&nan_limit).expect_err("nan limit"), IrError::Json(_)));
}

#[test]
fn tool_length_must_be_finite_and_non_negative() {
    let kin = load(TABLE_TABLE_AC);
    let at = joints(Vec3::ZERO, [0.0, 0.0]);
    for length in [-0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let err = kin.forward(&at, length).expect_err("length");
        assert_eq!(err, invalid("tool length must be finite and non-negative"));
    }
    assert!(kin.forward(&at, 0.0).is_ok());
}

#[test]
fn within_limits_uses_each_joint_and_ignores_the_unused_slot() {
    let bc = load(TABLE_TABLE_BC);
    assert!(bc.within_limits([0.0, 0.0]));
    assert!(bc.within_limits([-35.0, 900.0]));
    assert!(bc.within_limits([120.0, -900.0]));
    assert!(!bc.within_limits([-35.1, 0.0]));
    assert!(!bc.within_limits([120.1, 0.0]));
    assert!(!bc.within_limits([0.0, f64::NAN]));
    assert!(!bc.within_limits([f64::NAN, 0.0]));

    let one = load(
        r#"{"name":"a","rotaries":[{"letter":"A","carrier":"table","axis":{"x":1.0,"y":0.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0},"limits":{"min_deg":-10.0,"max_deg":10.0}}]}"#,
    );
    assert!(one.within_limits([10.0, 400.0]));
    assert!(!one.within_limits([10.1, 0.0]));

    let mill = load(HEAD_TABLE_BC);
    assert!(mill.within_limits([400.0, 110.0]));
    assert!(!mill.within_limits([0.0, 110.1]));
}

#[test]
fn unknown_fields_are_rejected() {
    let err = Kinematics::from_json(r#"{"name":"demo","rotaries":[],"note":"no"}"#).expect_err("unknown");
    assert!(matches!(err, IrError::Json(_)));
    let err = Kinematics::from_json(
        r#"{"name":"demo","rotaries":[{"letter":"A","carrier":"table","axis":{"x":1.0,"y":0.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0},"sign":1}]}"#,
    )
    .expect_err("unknown joint");
    assert!(matches!(err, IrError::Json(_)));
}

#[test]
fn inverse_roundtrip_on_each_example() {
    let cases: [(&str, [[f64; 2]; 3]); 4] = [
        (TABLE_TABLE_AC, [[0.0, 0.0], [90.0, 0.0], [25.0, -40.0]]),
        (TABLE_TABLE_BC, [[20.0, -15.0], [-10.0, 40.0], [90.0, 10.0]]),
        (HEAD_TABLE_BC, [[0.0, 0.0], [45.0, 90.0], [-30.0, -45.0]]),
        (HEAD_HEAD_AC, [[0.0, 0.0], [90.0, 90.0], [20.0, -30.0]]),
    ];
    let places = [Vec3::ZERO, pt(10.0, -20.0, 30.0), pt(5.0, 6.0, 7.0)];
    let lengths = [0.0, 15.0, 40.0];
    for (text, hands) in cases {
        let kin = load(text);
        for (index, angles) in hands.iter().enumerate() {
            assert!(kin.within_limits(*angles), "{} {angles:?}", kin.name);
            assert_inverse_contains(&kin, places[index], *angles, lengths[index]);
        }
    }
}

#[test]
fn table_table_bc_at_b0_returns_hint_c() {
    let kin = load(TABLE_TABLE_BC);
    let xyz = pt(4.0, -2.0, 8.0);
    let tool_length = 12.5;
    for c in [0.0, 25.0, -40.0, 179.0] {
        let angles = [0.0, c];
        let pose = kin.forward(&joints(xyz, angles), tool_length).expect("forward");
        let sols = kin.inverse(&pose, tool_length, angles).expect("inverse");
        assert!(sols.iter().any(|sol| angles_near(sol.angles, angles, 1e-7)), "{sols:?}");
        for sol in &sols {
            assert!((sol.angles[1] - c).abs() <= 1e-7, "C {} != hint {c}", sol.angles[1]);
            let back = kin.forward(sol, tool_length).expect("back");
            assert!(gap(back.tip, pose.tip) <= 1e-6);
            assert!(gap(back.axis, pose.axis) <= 1e-9);
        }
    }
}

#[test]
fn tilted_target_on_a_c_table_is_not_reachable() {
    let kin = load(
        r#"{"name":"c_table","spindle_axis":{"x":0.0,"y":0.0,"z":1.0},"rotaries":[{"letter":"C","carrier":"table","axis":{"x":0.0,"y":0.0,"z":1.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}}]}"#,
    );
    let pose = ToolPose { tip: pt(1.0, 2.0, 3.0), axis: Vec3::X };
    let err = kin.inverse(&pose, 5.0, [0.0, 0.0]).expect_err("tilted");
    assert_eq!(err, invalid("tool axis (1.000000, 0.000000, 0.000000) is not reachable"));
}

#[test]
fn target_outside_b_limits_reports_the_limit_error() {
    let kin = load(TABLE_TABLE_BC);
    let pose = ToolPose { tip: pt(1.0, 2.0, 3.0), axis: pt(0.0, 0.0, -1.0) };
    let err = kin.inverse(&pose, 10.0, [0.0, 0.0]).expect_err("limits");
    assert_eq!(err, invalid("tool axis (0.000000, 0.000000, -1.000000) is outside the rotary limits"));
}

#[test]
fn choose_picks_the_requested_branch_on_table_table_bc() {
    let kin = load(TABLE_TABLE_BC);
    let tool_length = 12.0;
    let pose = kin.forward(&joints(pt(3.0, -4.0, 5.0), [-30.0, 0.0]), tool_length).expect("forward");
    assert!((pose.axis.x() - 0.5).abs() < 1e-9);
    assert!(pose.axis.y().abs() < 1e-9);
    assert!((pose.axis.z() - 3.0_f64.sqrt() / 2.0).abs() < 1e-9);

    let positive = kin.choose(&pose, tool_length, Choice::FirstPositive).expect("positive");
    let negative = kin.choose(&pose, tool_length, Choice::FirstNegative).expect("negative");
    assert!(positive.angles[0] >= 0.0, "{:?}", positive.angles);
    assert!(negative.angles[0] <= 0.0, "{:?}", negative.angles);
    assert!(positive.angles[0] > 1.0 && negative.angles[0] < -1.0);

    let near_negative = kin.choose(&pose, tool_length, Choice::Nearest([-29.0, 0.0])).expect("near neg");
    let near_positive = kin.choose(&pose, tool_length, Choice::Nearest([29.0, 180.0])).expect("near pos");
    assert!(near_negative.angles[0] < 0.0, "{:?}", near_negative.angles);
    assert!(near_positive.angles[0] > 0.0, "{:?}", near_positive.angles);

    let sols = kin.inverse(&pose, tool_length, [0.0, 0.0]).expect("sols");
    assert!(sols.len() >= 2);
    let preferred = [0.0, 90.0];
    let distance = |angles: [f64; 2]| (angles[0] - preferred[0]).abs().max((angles[1] - preferred[1]).abs());
    let picked = kin.choose(&pose, tool_length, Choice::Nearest(preferred)).expect("tie");
    let best = distance(sols[0].angles).min(distance(sols[1].angles));
    assert!((distance(picked.angles) - best).abs() <= 1e-9);
    if (distance(sols[0].angles) - distance(sols[1].angles)).abs() <= 1e-9 {
        assert!(angles_near(picked.angles, sols[0].angles, 1e-9));
    }
    for sol in [positive, negative, near_negative, near_positive, picked] {
        let back = kin.forward(&sol, tool_length).expect("back");
        assert!(gap(back.tip, pose.tip) <= 1e-6);
        assert!(gap(back.axis, pose.axis) <= 1e-9);
    }
}

#[test]
fn inverse_errors_for_tool_length_and_a_zero_axis() {
    let kin = load(TABLE_TABLE_AC);
    let pose = ToolPose { tip: Vec3::ZERO, axis: Vec3::Z };
    for length in [-0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let err = kin.inverse(&pose, length, [0.0, 0.0]).expect_err("length");
        assert_eq!(err, invalid("tool length must be finite and non-negative"));
    }
    let zero = ToolPose { tip: pt(1.0, 2.0, 3.0), axis: Vec3::ZERO };
    let err = kin.inverse(&zero, 1.0, [0.0, 0.0]).expect_err("zero");
    assert_eq!(err, invalid("tool axis is zero"));
}

#[test]
fn one_joint_inverse_uses_the_carrier_sign_and_the_hint_when_free() {
    let table = load(
        r#"{"name":"a_table","rotaries":[{"letter":"A","carrier":"table","axis":{"x":1.0,"y":0.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}}]}"#,
    );
    assert_inverse_contains(&table, pt(2.0, -3.0, 4.0), [30.0, 0.0], 16.0);

    let head = load(
        r#"{"name":"b_head","rotaries":[{"letter":"B","carrier":"head","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":25.0}}],"spindle_axis":{"x":0.0,"y":0.0,"z":1.0}}"#,
    );
    assert_inverse_contains(&head, pt(4.0, 5.0, 6.0), [-20.0, 0.0], 10.0);

    let c_table = load(
        r#"{"name":"c_table","rotaries":[{"letter":"C","carrier":"table","axis":{"x":0.0,"y":0.0,"z":1.0},"pivot":{"x":1.0,"y":2.0,"z":0.0}}]}"#,
    );
    let angles = [50.0, 0.0];
    let pose = c_table.forward(&joints(pt(1.0, 2.0, 3.0), angles), 8.0).expect("forward");
    let sols = c_table.inverse(&pose, 8.0, [50.0, 7.0]).expect("inverse");
    assert!(sols.iter().any(|sol| angles_near(sol.angles, angles, 1e-7)), "{sols:?}");
}

#[test]
fn parallel_axes_keep_the_hint_split() {
    let heads = load(
        r#"{"name":"parallel_heads","rotaries":[{"letter":"B","carrier":"head","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}},{"letter":"A","carrier":"head","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":0.0,"y":5.0,"z":12.0}}]}"#,
    );
    assert_inverse_contains(&heads, pt(1.0, 2.0, 3.0), [30.0, 10.0], 25.0);

    let tables = load(
        r#"{"name":"parallel_tables","rotaries":[{"letter":"B","carrier":"table","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}},{"letter":"A","carrier":"table","axis":{"x":0.0,"y":1.0,"z":0.0},"pivot":{"x":8.0,"y":0.0,"z":4.0}}]}"#,
    );
    assert_inverse_contains(&tables, pt(-6.0, 3.0, 9.0), [10.0, 25.0], 18.0);

    let free = load(
        r#"{"name":"parallel_c","rotaries":[{"letter":"A","carrier":"table","axis":{"x":0.0,"y":0.0,"z":1.0},"pivot":{"x":0.0,"y":0.0,"z":0.0}},{"letter":"C","carrier":"table","axis":{"x":0.0,"y":0.0,"z":1.0},"pivot":{"x":15.0,"y":-6.0,"z":2.0}}]}"#,
    );
    assert_inverse_contains(&free, pt(7.0, 1.0, -2.0), [12.0, -40.0], 22.0);
}

fn assert_forward_inverse_roundtrip(
    kin: &Kinematics,
    xyz: Vec3,
    angles: [f64; 2],
    tool_length: f64,
) -> Result<(), proptest::test_runner::TestCaseError> {
    proptest::prop_assert!(kin.within_limits(angles), "{} {angles:?}", kin.name);
    let pose = match kin.forward(&joints(xyz, angles), tool_length) {
        Ok(pose) => pose,
        Err(err) => return Err(proptest::test_runner::TestCaseError::fail(format!("{} forward {err}", kin.name))),
    };
    let sols = match kin.inverse(&pose, tool_length, angles) {
        Ok(sols) => sols,
        Err(err) => {
            return Err(proptest::test_runner::TestCaseError::fail(format!(
                "{} inverse {err}; angles {angles:?} tip {:?} axis {:?}",
                kin.name, pose.tip, pose.axis
            )));
        }
    };
    proptest::prop_assert!(!sols.is_empty(), "{} produced no solution", kin.name);
    for sol in &sols {
        let back = match kin.forward(sol, tool_length) {
            Ok(back) => back,
            Err(err) => return Err(proptest::test_runner::TestCaseError::fail(format!("{} forward of {:?} {err}", kin.name, sol.angles))),
        };
        let tip_gap = gap(back.tip, pose.tip);
        let axis_gap = gap(back.axis, pose.axis);
        proptest::prop_assert!(
            tip_gap <= 1e-6 && axis_gap <= 1e-9,
            "{} tip gap {tip_gap} axis gap {axis_gap}; angles {angles:?} solution {:?} tip {:?} axis {:?}",
            kin.name,
            sol.angles,
            back.tip,
            back.axis
        );
    }
    let chosen = match kin.choose(&pose, tool_length, Choice::Nearest(angles)) {
        Ok(chosen) => chosen,
        Err(err) => return Err(proptest::test_runner::TestCaseError::fail(format!("{} choose {err}", kin.name))),
    };
    proptest::prop_assert!(kin.within_limits(chosen.angles), "{} nearest {:?} -> {:?}", kin.name, angles, chosen.angles);
    Ok(())
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(256))]

    #[test]
    fn example_inverse_roundtrip_and_nearest_stays_inside_limits(
        x in -500.0f64..500.0,
        y in -500.0f64..500.0,
        z in -500.0f64..500.0,
        free in -179.0f64..=179.0,
        free_b in -179.0f64..=179.0,
        b_table in -35.0f64..=120.0,
        b_head in -110.0f64..=110.0,
        tool_length in 0.0f64..200.0,
    ) {
        let xyz = pt(x, y, z);
        let samples = [
            (TABLE_TABLE_AC, [free, free_b]),
            (TABLE_TABLE_BC, [b_table, free]),
            (HEAD_TABLE_BC, [free, b_head]),
            (HEAD_HEAD_AC, [free, free_b]),
        ];
        for (text, angles) in samples {
            let kin = load(text);
            assert_forward_inverse_roundtrip(&kin, xyz, angles, tool_length)?;
        }
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(80))]

    #[test]
    fn forward_returns_a_unit_axis_and_a_finite_tip(
        x in -500.0f64..500.0,
        y in -500.0f64..500.0,
        z in -500.0f64..500.0,
        c_deg in -720.0f64..720.0,
        b_deg in -110.0f64..110.0,
        tool_length in 0.0f64..400.0,
    ) {
        let kin = load(HEAD_TABLE_BC);
        let angles = [c_deg, b_deg];
        proptest::prop_assert!(kin.within_limits(angles));
        let pose = kin.forward(&joints(pt(x, y, z), angles), tool_length).expect("forward");
        let len = (pose.axis.x() * pose.axis.x() + pose.axis.y() * pose.axis.y() + pose.axis.z() * pose.axis.z()).sqrt();
        proptest::prop_assert!((len - 1.0).abs() <= 1e-9);
        proptest::prop_assert!(pose.tip.x().is_finite() && pose.tip.y().is_finite() && pose.tip.z().is_finite());
        proptest::prop_assert!(pose.axis.x().is_finite() && pose.axis.y().is_finite() && pose.axis.z().is_finite());
    }
}
