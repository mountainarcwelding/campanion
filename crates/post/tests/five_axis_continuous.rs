//! Continuous five-axis posts: inverse kinematics, inverse-time feed, and read-back replay.
//!
//! Rounding used in the replay checks: `0.5 * 10^{-d}` per linear axis (`d` is `decimals.linear`),
//! Euclidean over XYZ as `sqrt(3)` times that, plus `250 mm * 2 * 0.5 * 10^{-3} deg` from the
//! rotary words (`five_axis.decimals` is 3). IR vertices must appear in the posted stream
//! (axis within `axis_tol_deg + singular_cone_deg + 0.5°`). Joint-space midpoints stay within
//! `2 * chord_tol` of the posted chord. Inverse-time: sum of `1/F` over feed blocks equals
//! total tip length / feed within 2%.

#![allow(clippy::expect_used)]

use std::path::PathBuf;

use campanion_ir::{
    ArcSense, Choice, ClBlock, ClMotion, ClPath, CompensationMode, Document, Event, Feeds, FiveAxisSetup, Joints, Kinematics, MoveRole, Plane, Pose,
    Program, SolutionPolicy, Tool, ToolPose, Vec3,
};
use campanion_post::{PostDef, PostOptions, ReadMove, builtin, load_post, post_program, read_back};
use proptest::prelude::*;
use proptest::test_runner::TestCaseError;

const TABLE_TABLE_AC: &str = include_str!("../../ir/kinematics/table_table_ac.json");
const TABLE_TABLE_BC: &str = include_str!("../../ir/kinematics/table_table_bc.json");
const HEAD_TABLE_BC: &str = include_str!("../../ir/kinematics/head_table_bc.json");
const HEAD_HEAD_AC: &str = include_str!("../../ir/kinematics/head_head_ac.json");

const CHORD_TOL: f64 = 0.005;
const AXIS_TOL_DEG: f64 = 0.1;
const FEED: f64 = 100.0;
const CONE_DEG: f64 = 1.0;
/// Inverse-time: sum of 1/F versus total tip length / feed. Feed words round to the post decimals.
const INVERSE_TIME_REL: f64 = 0.02;
const POST_IDS: [&str; 2] = ["linuxcnc_5ax", "haas_umc"];
const TCP_POST_IDS: [&str; 2] = ["linuxcnc_5ax_tcp", "haas_umc_tcpc"];
/// `five_axis.continuous.tcp.rapid_feed`, millimetres per minute.
const RAPID_FEED: f64 = 5000.0;

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
        MachineSpec { label: "table_table_ac", text: TABLE_TABLE_AC, tilt: 0, spin: Some(1), tilt_min: -170.0, tilt_max: 170.0 },
        MachineSpec { label: "table_table_bc", text: TABLE_TABLE_BC, tilt: 0, spin: Some(1), tilt_min: -35.0, tilt_max: 120.0 },
        MachineSpec { label: "head_table_bc", text: HEAD_TABLE_BC, tilt: 1, spin: Some(0), tilt_min: -110.0, tilt_max: 110.0 },
        MachineSpec { label: "head_head_ac", text: HEAD_HEAD_AC, tilt: 1, spin: Some(0), tilt_min: -170.0, tilt_max: 170.0 },
    ]
}

fn kinematics(text: &str) -> Kinematics {
    Kinematics::from_json(text).expect("kinematics")
}

fn setup_of(text: &str) -> FiveAxisSetup {
    FiveAxisSetup { kinematics: kinematics(text), policy: SolutionPolicy::Nearest }
}

fn pose_of(tip: Vec3, axis: Vec3) -> Pose {
    Pose::new(tip, axis).expect("pose")
}

fn path_of(blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some("cut".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(FEED, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
    .with_continuous(true)
}

fn program_of(five: FiveAxisSetup, blocks: Vec<ClBlock>) -> Program {
    Program::try_new(Some("swarf".into()), Vec::new(), None, vec![path_of(blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(five)
        .expect("five")
}

/// TCP hooks name `{tool}`, so TCP programs start with a tool change to tool 1.
fn tcp_program_of(five: FiveAxisSetup, mut blocks: Vec<ClBlock>) -> Program {
    blocks.insert(0, ClBlock::Event(Event::ToolChange { tool_id: 1 }));
    let tool = Tool::try_new(1, None, None, None).expect("tool");
    Program::try_new(Some("swarf".into()), vec![tool], None, vec![path_of(blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(five)
        .expect("five")
}

/// The same program with a tool change to tool 1 at the start of each toolpath.
fn with_tool_change(program: &Program) -> Program {
    let doc = Document::new(program.clone()).expect("doc");
    let mut raw: serde_json::Value = serde_json::from_str(&doc.to_json_pretty().expect("json")).expect("value");
    raw["program"]["tools"] = serde_json::json!([{"id": 1, "name": null, "diameter": null, "kerf": null}]);
    for path in raw["program"]["toolpaths"].as_array_mut().expect("toolpaths") {
        path["blocks"].as_array_mut().expect("blocks").insert(0, serde_json::json!({"type": "tool_change", "tool_id": 1}));
    }
    Document::from_json(&raw.to_string()).expect("load").program().clone()
}

fn options() -> PostOptions {
    PostOptions { arc_tolerance: CHORD_TOL }
}

fn gap(left: Vec3, right: Vec3) -> f64 {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn axis_deg(left: Vec3, right: Vec3) -> f64 {
    let dot = (left.x() * right.x() + left.y() * right.y() + left.z() * right.z()).clamp(-1.0, 1.0);
    let cx = left.y() * right.z() - left.z() * right.y();
    let cy = left.z() * right.x() - left.x() * right.z();
    let cz = left.x() * right.y() - left.y() * right.x();
    let sine = (cx * cx + cy * cy + cz * cz).sqrt();
    sine.atan2(dot).abs().to_degrees()
}

fn lerp(min: f64, max: f64, t: f64) -> f64 {
    min + (max - min) * t
}

fn joints_at(spec: &MachineSpec, tilt: f64, spin: f64) -> [f64; 2] {
    let mut angles = [0.0; 2];
    angles[spec.tilt] = tilt;
    if let Some(index) = spec.spin {
        angles[index] = spin;
    }
    angles
}

fn forward_pose(kin: &Kinematics, angles: [f64; 2], xyz: Vec3) -> ToolPose {
    kin.forward(&Joints { xyz, angles }, 0.0).expect("forward")
}

/// Half-unit in the last place of a dimensional word, Euclidean over XYZ.
fn xyz_round(post: &PostDef) -> f64 {
    0.5 * 10_f64.powi(-i32::from(post.linear_decimals())) * 3_f64.sqrt()
}

/// Half-unit in the last place of a five-axis rotary word (`five_axis.decimals` is 3).
fn angle_round() -> f64 {
    0.5 * 1e-3
}

/// Rotary rounding mapped through a 250 mm lever, plus XYZ rounding.
fn tip_round(post: &PostDef) -> f64 {
    xyz_round(post) + 250.0 * (2.0 * angle_round()).to_radians()
}

fn five_c_bound(cone_deg: f64) -> f64 {
    90.0 + 2.0 * cone_deg
}

fn slerp_axis(start: Vec3, end: Vec3, t: f64) -> Vec3 {
    let dot = (start.x() * end.x() + start.y() * end.y() + start.z() * end.z()).clamp(-1.0, 1.0);
    let theta = dot.acos();
    let sin_theta = theta.sin();
    if sin_theta <= 1e-12 {
        return start;
    }
    let w0 = ((1.0 - t) * theta).sin() / sin_theta;
    let w1 = (t * theta).sin() / sin_theta;
    let x = start.x() * w0 + end.x() * w1;
    let y = start.y() * w0 + end.y() * w1;
    let z = start.z() * w0 + end.z() * w1;
    let len = (x * x + y * y + z * z).sqrt();
    Vec3::new(x / len, y / len, z / len).unwrap_or(start)
}

fn motion_end(mv: &ReadMove) -> Option<[f64; 3]> {
    match mv {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } => Some(*end),
        _ => None,
    }
}

fn joints_from_move(mv: &ReadMove, kin: &Kinematics) -> Option<Joints> {
    let end = motion_end(mv)?;
    let xyz = Vec3::new(end[0], end[1], end[2]).ok()?;
    let five = mv.five();
    let mut angles = [0.0; 2];
    for (index, joint) in kin.rotaries.iter().enumerate() {
        let slot = match joint.letter {
            'A' => 0,
            'B' => 1,
            'C' => 2,
            _ => return None,
        };
        angles[index] = five[slot]?;
    }
    Some(Joints { xyz, angles })
}

fn lerp_joints(left: Joints, right: Joints, t: f64) -> Joints {
    Joints {
        xyz: Vec3::new(
            left.xyz.x() + (right.xyz.x() - left.xyz.x()) * t,
            left.xyz.y() + (right.xyz.y() - left.xyz.y()) * t,
            left.xyz.z() + (right.xyz.z() - left.xyz.z()) * t,
        )
        .unwrap_or(left.xyz),
        angles: [left.angles[0] + (right.angles[0] - left.angles[0]) * t, left.angles[1] + (right.angles[1] - left.angles[1]) * t],
    }
}

fn replay_ok(kin: &Kinematics, post: &PostDef, ir_tips: &[Vec3], ir_axes: &[Vec3], moves: &[ReadMove], text: &str) -> Result<(), String> {
    let tip_tol = CHORD_TOL + tip_round(post);
    let mid_tol = 2.0 * CHORD_TOL + tip_round(post);
    let axis_tol = AXIS_TOL_DEG + 2.0 * angle_round();
    let jump = five_c_bound(CONE_DEG) + 2.0 * angle_round();
    let mut prev: Option<Joints> = None;
    let mut prev_tip: Option<Vec3> = None;
    let mut posted_tips: Vec<Vec3> = Vec::new();
    let mut posted_axes: Vec<Vec3> = Vec::new();
    let mut feed_minutes = 0.0;
    let mut feed_length = 0.0;
    for mv in moves {
        let Some(joints) = joints_from_move(mv, kin) else {
            continue;
        };
        let pose = kin.forward(&joints, 0.0).map_err(|err| err.to_string())?;
        if let Some(last) = prev {
            for index in 0..kin.rotaries.len() {
                let delta = (joints.angles[index] - last.angles[index]).abs();
                if delta > jump {
                    return Err(format!("rotary jump {delta} > {jump} on {}\n{text}", kin.rotaries[index].letter));
                }
            }
            let last_pose = kin.forward(&last, 0.0).map_err(|err| err.to_string())?;
            let mid = lerp_joints(last, joints, 0.5);
            let mid_pose = kin.forward(&mid, 0.0).map_err(|err| err.to_string())?;
            let chord_mid = Vec3::new(
                (last_pose.tip.x() + pose.tip.x()) * 0.5,
                (last_pose.tip.y() + pose.tip.y()) * 0.5,
                (last_pose.tip.z() + pose.tip.z()) * 0.5,
            )
            .map_err(|err| err.to_string())?;
            let mid_gap = gap(mid_pose.tip, chord_mid);
            if mid_gap > mid_tol {
                return Err(format!("midpoint tip {mid_gap} > {mid_tol}\n{text}"));
            }
            let axis_mid = axis_deg(mid_pose.axis, slerp_axis(last_pose.axis, pose.axis, 0.5));
            if axis_mid > axis_tol + 0.05 {
                return Err(format!("midpoint axis {axis_mid} > {}\n{text}", axis_tol + 0.05));
            }
        }
        posted_tips.push(pose.tip);
        posted_axes.push(pose.axis);
        if let ReadMove::Linear { feed, inverse_time, .. } = mv {
            if !*inverse_time {
                return Err(format!("feed block is not G93\n{text}"));
            }
            let Some(f) = *feed else {
                return Err(format!("feed block has no F\n{text}"));
            };
            if !(f.is_finite() && f > 0.0) {
                return Err(format!("F {f} is not positive and finite\n{text}"));
            }
            feed_minutes += 1.0 / f;
            if let Some(start) = prev_tip {
                feed_length += gap(start, pose.tip);
            }
        }
        prev = Some(joints);
        prev_tip = Some(pose.tip);
    }
    for (ir_tip, ir_axis) in ir_tips.iter().zip(ir_axes) {
        let hit = posted_tips
            .iter()
            .zip(posted_axes.iter())
            .any(|(got, got_axis)| gap(*got, *ir_tip) <= tip_tol && axis_deg(*got_axis, *ir_axis) <= axis_tol + CONE_DEG + 0.5);
        if !hit {
            return Err(format!("IR pose {:?} missing from posted points (tol {tip_tol})\n{text}", (ir_tip.x(), ir_tip.y(), ir_tip.z())));
        }
    }
    if feed_length > 1e-6 {
        let expected = feed_length / FEED;
        let rel = (feed_minutes - expected).abs() / expected.max(1e-9);
        if rel > INVERSE_TIME_REL {
            return Err(format!("inverse time relative error {rel} > {INVERSE_TIME_REL} (sum 1/F {feed_minutes}, L/F {expected})\n{text}"));
        }
    }
    Ok(())
}

fn has_g2g3(text: &str) -> bool {
    text.lines().any(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3"))
}

fn swarf_program() -> Program {
    let kin = kinematics(TABLE_TABLE_BC);
    let five = FiveAxisSetup { kinematics: kin, policy: SolutionPolicy::Nearest };
    let start = pose_of(Vec3::new(20.0, -8.0, 10.0).expect("s"), Vec3::Z);
    let mid = pose_of(Vec3::new(20.0, 0.0, 10.0).expect("m"), Vec3::new(15_f64.to_radians().sin(), 0.0, 15_f64.to_radians().cos()).expect("a15"));
    let end = pose_of(
        Vec3::new(20.0 * 30_f64.to_radians().cos(), 20.0 * 30_f64.to_radians().sin(), 10.0).expect("e"),
        Vec3::new(30_f64.to_radians().sin(), 0.0, 30_f64.to_radians().cos()).expect("a30"),
    );
    let center = Vec3::new(0.0, 0.0, 10.0).expect("c");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: mid, feed: FEED, role: MoveRole::Cut, power: None }),
        ClBlock::Motion(ClMotion::Arc { to: end, plane: Plane::Xy, center, sense: ArcSense::Ccw, feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    program_of(five, blocks)
}

fn manifest_path(parts: &[&str]) -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for part in parts {
        path.push(part);
    }
    path
}

#[test]
fn swarf_cylinder_matches_golden() {
    let update = std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1");
    let program = swarf_program();
    let ir_path = manifest_path(&["tests", "data", "swarf_cylinder.json"]);
    if update {
        let doc = Document::new(program.clone()).expect("doc");
        if let Some(parent) = ir_path.parent() {
            std::fs::create_dir_all(parent).expect("dir");
        }
        std::fs::write(&ir_path, doc.to_json_pretty().expect("json")).expect("write ir");
    }
    let posted = if ir_path.exists() {
        Document::from_json(&std::fs::read_to_string(&ir_path).expect("ir")).expect("load").program().clone()
    } else {
        program.clone()
    };
    let posted_tcp = with_tool_change(&posted);
    for id in POST_IDS.into_iter().chain(TCP_POST_IDS) {
        let post = builtin(id).expect(id);
        let input = if TCP_POST_IDS.contains(&id) { &posted_tcp } else { &posted };
        let text = post_program(input, &post, &options()).expect(id);
        assert!(!has_g2g3(&text), "{id} wrote an arc\n{text}");
        assert!(text.contains("G93") && text.contains("G94"), "{id}\n{text}");
        let golden_path = manifest_path(&["tests", "golden", &format!("swarf_cylinder.{id}.nc")]);
        if update {
            if let Some(parent) = golden_path.parent() {
                std::fs::create_dir_all(parent).expect("dir");
            }
            std::fs::write(&golden_path, &text).expect("write golden");
        } else {
            let golden = std::fs::read_to_string(&golden_path).expect("golden");
            assert_eq!(text, golden, "{id}");
        }
    }
}

#[test]
fn rotary_only_without_rotary_block_is_an_error() {
    let mut raw: serde_json::Value = serde_json::from_str(include_str!("../posts/linuxcnc_5ax.json")).expect("json");
    raw.as_object_mut().expect("obj").remove("rotary");
    let post = load_post(&raw.to_string()).expect("post");
    let axis = Vec3::new(0.05_f64.to_radians().sin(), 0.0, 0.05_f64.to_radians().cos()).expect("axis");
    let tip = Vec3::new(4.0, 5.0, 6.0).expect("tip");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(tip, Vec3::Z), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(tip, axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let program = program_of(setup_of(B_HEAD_JSON), blocks);
    let err = post_program(&program, &post, &options()).expect_err("rotary-only");
    assert_eq!(err.to_string(), "invalid input: toolpath 0 move 1: rotary-only move needs a feed");
    let text = post_program(&program, &builtin("linuxcnc_5ax").expect("5ax"), &options()).expect("fallback");
    assert!(text.contains("G93") && text.contains("G1"), "{text}");
}

/// Two poses inside the cone, with a large commanded spin between them. `solve_path` holds the
/// primary. Subdivision must follow that solved axis. Chasing the requested axis splits the 3 mm
/// move until the post refuses it as a rotary-only feed.
#[test]
fn continuous_move_inside_the_singular_cone_posts() {
    let kin = kinematics(TABLE_TABLE_AC);
    let axis_at = |tilt: f64, spin: f64| kin.forward(&Joints { xyz: Vec3::ZERO, angles: [tilt, spin] }, 0.0).expect("forward").axis;
    let start_axis = axis_at(0.4, 10.0);
    let end_axis = axis_at(0.45, 100.0);
    assert!(axis_deg(start_axis, Vec3::Z) < CONE_DEG, "start left the cone");
    assert!(axis_deg(end_axis, Vec3::Z) < CONE_DEG, "end left the cone");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(5.0, 2.0, 12.0).expect("tip"), start_axis), feed: None }),
        ClBlock::Motion(ClMotion::Linear {
            to: pose_of(Vec3::new(8.0, 2.0, 12.0).expect("tip"), end_axis),
            feed: FEED,
            role: MoveRole::Cut,
            power: None,
        }),
    ];
    let program = program_of(setup_of(TABLE_TABLE_AC), blocks.clone());
    let tcp = tcp_program_of(setup_of(TABLE_TABLE_AC), blocks);
    for id in POST_IDS.into_iter().chain(TCP_POST_IDS) {
        let post = builtin(id).expect(id);
        let input = if TCP_POST_IDS.contains(&id) { &tcp } else { &program };
        let text = post_program(input, &post, &options()).unwrap_or_else(|err| panic!("{id}: {err}"));
        let moves: Vec<_> = read_back(&text, &post)
            .unwrap_or_else(|err| panic!("{id}: {err}"))
            .into_iter()
            .filter(|motion| matches!(motion, ReadMove::Rapid { .. } | ReadMove::Linear { .. }))
            .collect();
        assert!(moves.len() < 8, "{id} split the cone move into {} blocks\n{text}", moves.len());
        let mut primary = Vec::new();
        for motion in &moves {
            let five = match motion {
                ReadMove::Rapid { five, .. } | ReadMove::Linear { five, .. } => five,
                _ => continue,
            };
            if let Some(angle) = five[2] {
                primary.push(angle);
            }
        }
        assert!(primary.len() >= 2, "{id} posted no C word\n{text}");
        let held = primary[0];
        for angle in &primary {
            assert!((angle - held).abs() < 0.05, "{id} primary moved from {held} to {angle}\n{text}");
        }
    }
}

const B_HEAD_JSON: &str = r#"{
  "name": "b_head",
  "rotaries": [{
    "letter": "B",
    "carrier": "head",
    "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
    "pivot": {"x": 0.0, "y": 0.0, "z": 25.0}
  }]
}"#;

const B_TABLE_JSON: &str = r#"{
  "name": "b_table",
  "rotaries": [{
    "letter": "B",
    "carrier": "table",
    "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
    "pivot": {"x": 12.0, "y": 0.0, "z": 7.0}
  }]
}"#;

#[test]
fn continuous_after_indexed_cancels_the_work_frame() {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let indexed = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("a"), axis), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(Vec3::new(1.0, 2.0, 3.0).expect("b"), axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let cont = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("c"), Vec3::Z), feed: None }),
        ClBlock::Motion(ClMotion::Linear {
            to: pose_of(Vec3::new(8.0, 5.0, 6.0).expect("d"), Vec3::Z),
            feed: FEED,
            role: MoveRole::Cut,
            power: None,
        }),
    ];
    let indexed_path = ClPath::try_new(
        Some("index".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(FEED, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        indexed,
    )
    .expect("indexed");
    let cont_path = path_of(cont);
    let program = Program::try_new(Some("mix".into()), Vec::new(), None, vec![indexed_path, cont_path], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(setup_of(B_TABLE_JSON))
        .expect("five");
    let text = post_program(&program, &builtin("haas_umc").expect("umc"), &options()).expect("post");
    assert!(text.contains("G254"), "{text}");
    assert!(text.contains("G255"), "{text}");
    let last_254 = text.rfind("G254").expect("254");
    let last_255 = text.rfind("G255").expect("255");
    assert!(last_255 > last_254, "G255 after G254: {text}");
}

fn word_on(line: &str, letter: char) -> Option<f64> {
    line.split_whitespace().find_map(|token| {
        let rest = token.strip_prefix(letter)?;
        if rest.is_empty() || !rest.starts_with(|ch: char| ch.is_ascii_digit() || matches!(ch, '-' | '+' | '.')) {
            return None;
        }
        rest.parse().ok()
    })
}

fn has_token(line: &str, code: &str) -> bool {
    line.split_whitespace().any(|token| token == code)
}

/// Continuous output leaves the work frame off. A later indexed toolpath on the same axis
/// must turn the frame back on and write frame coordinates, not the raw part tip.
#[test]
fn indexed_after_continuous_restores_the_work_frame() {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let cont = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("c"), axis), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(Vec3::new(8.0, 5.0, 6.0).expect("d"), axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let indexed = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("a"), axis), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(Vec3::new(1.0, 2.0, 3.0).expect("b"), axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let indexed_path = ClPath::try_new(
        Some("index".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(FEED, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        indexed,
    )
    .expect("indexed");
    let program = Program::try_new(Some("mix".into()), Vec::new(), None, vec![path_of(cont), indexed_path], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(setup_of(B_TABLE_JSON))
        .expect("five");
    let kin = kinematics(B_TABLE_JSON);
    let held = kin.choose(&ToolPose { tip: Vec3::new(4.0, 5.0, 6.0).expect("tip"), axis }, 0.0, Choice::Nearest([0.0, 0.0])).expect("choose");
    let feed_tip = Vec3::new(1.0, 2.0, 3.0).expect("feed");
    let machine = kin.choose(&ToolPose { tip: feed_tip, axis }, 0.0, Choice::Nearest(held.angles)).expect("feed joints");

    let haas = post_program(&program, &builtin("haas_umc").expect("umc"), &options()).expect("haas");
    let g94 = haas.find("G94").expect("G94");
    let g254 = haas.rfind("G254").expect("G254 after continuous");
    assert!(g254 > g94, "G254 must follow G94\n{haas}");
    let haas_feed = haas.lines().rev().find(|line| has_token(line, "G1")).expect("haas feed");
    // B90 about +Y, pivots ignored: (x, y, z) -> (z, y, -x).
    assert!((word_on(haas_feed, 'X').expect("x") - 3.0).abs() < 1e-3, "{haas_feed}\n{haas}");
    assert!((word_on(haas_feed, 'Y').expect("y") - 2.0).abs() < 1e-3, "{haas_feed}");
    assert!((word_on(haas_feed, 'Z').expect("z") - -1.0).abs() < 1e-3, "{haas_feed}");

    let linux = post_program(&program, &builtin("linuxcnc_5ax").expect("5ax"), &options()).expect("linux");
    let linux_feed = linux.lines().rev().find(|line| has_token(line, "G1")).expect("linux feed");
    assert!((word_on(linux_feed, 'X').expect("x") - machine.xyz.x()).abs() < 5e-4, "{linux_feed} vs {:?}", machine.xyz);
    assert!((word_on(linux_feed, 'Y').expect("y") - machine.xyz.y()).abs() < 5e-4, "{linux_feed}");
    assert!((word_on(linux_feed, 'Z').expect("z") - machine.xyz.z()).abs() < 5e-4, "{linux_feed}");
    assert!((machine.xyz.x() - 1.0).abs() > 0.5, "fixture must not match the raw tip");
}

fn chord_sagitta(sweep: f64, radius: f64, segments: u32) -> f64 {
    let step = sweep / f64::from(segments);
    radius * (1.0 - (step * 0.5).cos())
}

proptest! {
    #![proptest_config(ProptestConfig::default())]

    #[test]
    fn arc_chords_stay_within_chord_tol(radius in 1.0f64..80.0, sweep in 0.05f64..std::f64::consts::PI) {
        let step = if radius <= CHORD_TOL {
            std::f64::consts::PI
        } else {
            let ratio = (CHORD_TOL / radius).min(2.0);
            let cos_half = (1.0 - ratio).clamp(-1.0, 1.0);
            (2.0 * cos_half.acos()).max(1e-9)
        };
        let mut n = (sweep / step).ceil();
        if (n - n.round()).abs() <= 1e-8 {
            n = n.round();
        }
        let segments = n.max(1.0) as u32;
        let sagitta = chord_sagitta(sweep, radius, segments);
        prop_assert!(sagitta <= CHORD_TOL + 1e-9, "sagitta {sagitta} > {CHORD_TOL} r={radius} sweep={sweep} n={segments}");
    }
}

fn path_strategy() -> impl Strategy<Value = (usize, Vec<f64>, Vec<f64>, bool, bool)> {
    (0usize..4, prop::collection::vec(-0.2f64..1.2, 3..=6), prop::collection::vec(0.0f64..1.0, 3..=6), proptest::bool::ANY, proptest::bool::ANY)
}

enum TipSeg {
    Line,
    Arc { center: Vec3 },
}

fn build_blocks(
    spec: &MachineSpec,
    tilts: &[f64],
    spins: &[f64],
    near_cone: bool,
    with_arc: bool,
) -> Option<(Vec<ClBlock>, Vec<Vec3>, Vec<Vec3>, Vec<TipSeg>)> {
    let kin = kinematics(spec.text);
    let n = tilts.len().min(spins.len());
    if n < 3 {
        return None;
    }
    let mut poses = Vec::new();
    let t0 = if near_cone {
        lerp(-CONE_DEG, CONE_DEG, tilts[0].clamp(0.0, 1.0))
    } else {
        lerp(spec.tilt_min + 8.0, spec.tilt_max - 8.0, tilts[0].clamp(0.0, 1.0))
    };
    let s0 = lerp(-20.0, 20.0, spins[0].clamp(0.0, 1.0));
    let mut tilt = t0;
    let mut spin = s0;
    for i in 0..n {
        if i > 0 {
            tilt = (tilt + lerp(-8.0, 8.0, tilts[i].clamp(0.0, 1.0))).clamp(spec.tilt_min + 2.0, spec.tilt_max - 2.0);
            spin += lerp(-12.0, 12.0, spins[i].clamp(0.0, 1.0));
        }
        let xyz = Vec3::new(8.0 + i as f64 * 3.0, 4.0 + i as f64, 12.0 - i as f64 * 0.4).ok()?;
        let pose = forward_pose(&kin, joints_at(spec, tilt, spin), xyz);
        poses.push(pose);
    }
    let mut blocks = Vec::new();
    let mut ir_tips = Vec::new();
    let mut ir_axes = Vec::new();
    let mut segs = Vec::new();
    blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose_of(poses[0].tip, poses[0].axis), feed: None }));
    ir_tips.push(poses[0].tip);
    ir_axes.push(poses[0].axis);
    for i in 1..n {
        if with_arc && i == n / 2 {
            let start = poses[i - 1].tip;
            let end = poses[i].tip;
            let dx = end.x() - start.x();
            let dy = end.y() - start.y();
            let len = dx.hypot(dy);
            if len > 1.0 && (end.z() - start.z()).abs() < 2.0 {
                let nx = -dy / len;
                let ny = dx / len;
                let cx = (start.x() + end.x()) * 0.5 + nx * 6.0;
                let cy = (start.y() + end.y()) * 0.5 + ny * 6.0;
                let cz = (start.z() + end.z()) * 0.5;
                let center = Vec3::new(cx, cy, cz).ok()?;
                blocks.push(ClBlock::Motion(ClMotion::Arc {
                    to: pose_of(end, poses[i].axis),
                    plane: Plane::Xy,
                    center,
                    sense: ArcSense::Ccw,
                    feed: FEED,
                    role: MoveRole::Cut,
                    power: None,
                }));
                ir_tips.push(end);
                ir_axes.push(poses[i].axis);
                segs.push(TipSeg::Arc { center });
                continue;
            }
        }
        blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose_of(poses[i].tip, poses[i].axis), feed: FEED, role: MoveRole::Cut, power: None }));
        ir_tips.push(poses[i].tip);
        ir_axes.push(poses[i].axis);
        segs.push(TipSeg::Line);
    }
    Some((blocks, ir_tips, ir_axes, segs))
}

proptest! {
    #![proptest_config(ProptestConfig::default())]

    #[test]
    fn posted_continuous_replays_through_forward_kinematics((machine, tilts, spins, near_cone, with_arc) in path_strategy()) {
        let spec = &machines()[machine];
        let Some((blocks, ir_tips, ir_axes, _)) = build_blocks(spec, &tilts, &spins, near_cone, with_arc) else {
            return Ok(());
        };
        let program = program_of(setup_of(spec.text), blocks);
        let kin = kinematics(spec.text);
        for id in POST_IDS {
            let post = builtin(id).expect(id);
            match post_program(&program, &post, &options()) {
                Err(err) => {
                    let message = err.to_string();
                    prop_assert!(
                        message.contains("unreachable")
                            || message.contains("leaves axis")
                            || message.contains("flips the rotary branch")
                            || message.contains("rotary-only")
                            || message.contains("inserts more than")
                            || message.contains("too large"),
                        "{} {}: {message}",
                        spec.label,
                        id
                    );
                }
                Ok(text) => {
                    prop_assert!(!has_g2g3(&text), "{} {} wrote G2/G3\n{text}", spec.label, id);
                    let moves = read_back(&text, &post)
                        .map_err(|err| TestCaseError::fail(format!("{} {} read: {err}\n{text}", spec.label, id)))?;
                    replay_ok(&kin, &post, &ir_tips, &ir_axes, &moves, &text)
                        .map_err(|err| TestCaseError::fail(format!("{} {}: {err}", spec.label, id)))?;
                }
            }
        }
    }
}

fn token_is(line: &str, code: &str) -> bool {
    line.split_whitespace().any(|token| token == code)
}

fn rotary_words_on(line: &str) -> Vec<(char, f64)> {
    line.split_whitespace()
        .filter_map(|token| {
            let mut chars = token.chars();
            let letter = chars.next()?;
            if !matches!(letter, 'A' | 'B' | 'C') {
                return None;
            }
            let rest = chars.as_str();
            if rest.is_empty() || !rest.starts_with(|ch: char| ch.is_ascii_digit() || matches!(ch, '-' | '+' | '.')) {
                return None;
            }
            rest.parse().ok().map(|value| (letter, value))
        })
        .collect()
}

fn line_is_rapid(line: &str) -> bool {
    line.split_whitespace().any(|token| token == "G0" || token == "G00")
}

/// Haas TCPC structure: one G49 per G234, rotaries at 0 on the line before G234, no G234 while
/// G254 is active, and no rapid block carries a rotary word while TCPC is on.
fn haas_tcp_structure(text: &str) -> Result<(), String> {
    let lines: Vec<&str> = text.lines().filter(|line| !line.trim().is_empty()).collect();
    let mut dwo = false;
    let mut tcp = false;
    let mut opens = 0u32;
    let mut closes = 0u32;
    let mut seen = false;
    for (index, line) in lines.iter().enumerate() {
        if token_is(line, "G254") {
            dwo = true;
        }
        if token_is(line, "G255") {
            dwo = false;
        }
        if token_is(line, "G234") {
            if dwo {
                return Err(format!("G234 while G254 is active\n{text}"));
            }
            if tcp {
                return Err(format!("G234 while TCPC is already on\n{text}"));
            }
            let prev = lines.get(index.wrapping_sub(1)).copied().unwrap_or("");
            let rotaries = rotary_words_on(prev);
            if rotaries.is_empty() || rotaries.iter().any(|(_, value)| value.abs() > 1e-6) {
                return Err(format!("rotaries are not 0 on the line before G234: {prev}\n{text}"));
            }
            tcp = true;
            seen = true;
            opens += 1;
        }
        if token_is(line, "G49") {
            if tcp {
                tcp = false;
                closes += 1;
            } else if seen {
                return Err(format!("extra G49 after TCPC\n{text}"));
            }
        }
        if tcp && line_is_rapid(line) && !rotary_words_on(line).is_empty() {
            return Err(format!("rapid with a rotary word while TCPC is on: {line}\n{text}"));
        }
    }
    if tcp || opens != closes {
        return Err(format!("G234/G49 pairs {opens}/{closes}\n{text}"));
    }
    Ok(())
}

fn dist_point_seg(point: Vec3, start: Vec3, end: Vec3) -> f64 {
    let abx = end.x() - start.x();
    let aby = end.y() - start.y();
    let abz = end.z() - start.z();
    let ab2 = abx * abx + aby * aby + abz * abz;
    if ab2 <= 1e-18 {
        return gap(point, start);
    }
    let t = (((point.x() - start.x()) * abx + (point.y() - start.y()) * aby + (point.z() - start.z()) * abz) / ab2).clamp(0.0, 1.0);
    let hit = Vec3::new(start.x() + abx * t, start.y() + aby * t, start.z() + abz * t).unwrap_or(start);
    gap(point, hit)
}

/// XY counter-clockwise arc, Z linear in the sweep, matching the post's `point_at`.
fn arc_point_xy(start: Vec3, end: Vec3, center: Vec3, t: f64) -> Vec3 {
    let a0 = (start.y() - center.y()).atan2(start.x() - center.x());
    let a1 = (end.y() - center.y()).atan2(end.x() - center.x());
    let mut sweep = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    if sweep <= 1e-12 {
        sweep = std::f64::consts::TAU;
    }
    let signed = sweep * t.clamp(0.0, 1.0);
    let ou = start.x() - center.x();
    let ov = start.y() - center.y();
    let (sin_a, cos_a) = signed.sin_cos();
    let x = center.x() + ou * cos_a - ov * sin_a;
    let y = center.y() + ou * sin_a + ov * cos_a;
    let z = start.z() + (end.z() - start.z()) * t.clamp(0.0, 1.0);
    Vec3::new(x, y, z).unwrap_or(start)
}

/// Distance to the XY CCW arc (Z linear in the sweep).
///
/// Samples match `point_at`. The gap is the distance to those sample chords, so it
/// underestimates the curve by at most the sample sagitta (about 1e-5 mm), not by half
/// the sample spacing.
fn dist_arc_xy(point: Vec3, start: Vec3, end: Vec3, center: Vec3) -> f64 {
    let radius = (start.x() - center.x()).hypot(start.y() - center.y()).max(1e-6);
    let step = (2.0 * (1e-5 / radius).sqrt()).max(1e-4);
    let a0 = (start.y() - center.y()).atan2(start.x() - center.x());
    let a1 = (end.y() - center.y()).atan2(end.x() - center.x());
    let mut sweep = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    if sweep <= 1e-12 {
        sweep = std::f64::consts::TAU;
    }
    let n = ((sweep / step).ceil() as u32).clamp(4, 8000);
    let mut best = f64::MAX;
    let mut prev = arc_point_xy(start, end, center, 0.0);
    for index in 1..=n {
        let sample = arc_point_xy(start, end, center, f64::from(index) / f64::from(n));
        best = best.min(dist_point_seg(point, prev, sample));
        prev = sample;
    }
    best
}

fn dist_to_path(point: Vec3, tips: &[Vec3], segs: &[TipSeg]) -> f64 {
    let mut best = f64::MAX;
    for (index, seg) in segs.iter().enumerate() {
        let start = tips[index];
        let end = tips[index + 1];
        let distance = match seg {
            TipSeg::Line => dist_point_seg(point, start, end),
            TipSeg::Arc { center } => dist_arc_xy(point, start, end, *center),
        };
        best = best.min(distance);
    }
    best
}

struct PostedTcp {
    tip: Vec3,
    angles: [f64; 2],
    axis: Vec3,
    /// Inverse-time F on a G1, when the block is in G93.
    inverse_feed: Option<f64>,
    rapid: bool,
}

fn posted_tcp(mv: &ReadMove, kin: &Kinematics) -> Option<PostedTcp> {
    let end = motion_end(mv)?;
    let tip = Vec3::new(end[0], end[1], end[2]).ok()?;
    let five = mv.five();
    let mut angles = [0.0; 2];
    for (index, joint) in kin.rotaries.iter().enumerate() {
        let slot = match joint.letter {
            'A' => 0,
            'B' => 1,
            'C' => 2,
            _ => return None,
        };
        angles[index] = five[slot]?;
    }
    let axis = kin.forward(&Joints { xyz: Vec3::ZERO, angles }, 0.0).ok()?.axis;
    let (inverse_feed, rapid) = match mv {
        ReadMove::Linear { feed, inverse_time: true, .. } => (*feed, false),
        ReadMove::Rapid { .. } => (None, true),
        ReadMove::Linear { .. } => (None, false),
        _ => return None,
    };
    Some(PostedTcp { tip, angles, axis, inverse_feed, rapid })
}

fn lerp_tip(left: Vec3, right: Vec3) -> Vec3 {
    Vec3::new((left.x() + right.x()) * 0.5, (left.y() + right.y()) * 0.5, (left.z() + right.z()) * 0.5).unwrap_or(left)
}

/// TCP replay: posted XYZ is the part-frame tip. The axis is forward kinematics of the rotary words.
/// The engagement from the commanded zero is not a subdivided segment, so checks start once the
/// posted pose has reached the first IR tip and axis.
fn replay_tcp(
    kin: &Kinematics,
    post: &PostDef,
    ir_tips: &[Vec3],
    ir_axes: &[Vec3],
    segs: &[TipSeg],
    moves: &[ReadMove],
    text: &str,
) -> Result<(), String> {
    let tip_tol = CHORD_TOL + xyz_round(post);
    let vertex_axis = AXIS_TOL_DEG + CONE_DEG + angle_round();
    let mid_axis_tol = AXIS_TOL_DEG + 2.0 * angle_round();
    let jump = five_c_bound(CONE_DEG) + 2.0 * angle_round();
    let mut settled = false;
    let mut lead_in = 0u32;
    let mut prev: Option<PostedTcp> = None;
    let mut posted_tips = Vec::new();
    let mut posted_axes = Vec::new();
    let mut minutes = 0.0;
    let mut expected_minutes = 0.0;
    for mv in moves {
        let Some(point) = posted_tcp(mv, kin) else {
            continue;
        };
        if !settled {
            let on_first = gap(point.tip, ir_tips[0]) <= tip_tol;
            let axis_ok = axis_deg(point.axis, ir_axes[0]) <= vertex_axis;
            if on_first && axis_ok {
                settled = true;
                posted_tips.push(point.tip);
                posted_axes.push(point.axis);
                prev = Some(point);
            } else if point.rapid && !on_first {
                lead_in += 1;
                if lead_in > 1 {
                    return Err(format!("more than one off-path lead-in rapid\n{text}"));
                }
            }
            continue;
        }
        let last = prev.ok_or_else(|| "settled pose missing".to_string())?;
        for index in 0..kin.rotaries.len().min(2) {
            let delta = (point.angles[index] - last.angles[index]).abs();
            if delta > jump {
                return Err(format!("rotary jump {delta} > {jump}\n{text}"));
            }
        }
        let mid_tip = lerp_tip(last.tip, point.tip);
        let tip_gap = dist_to_path(point.tip, ir_tips, segs);
        let mid_gap = dist_to_path(mid_tip, ir_tips, segs);
        if tip_gap > tip_tol || mid_gap > tip_tol {
            return Err(format!("tip {tip_gap} or midpoint {mid_gap} > {tip_tol}\n{text}"));
        }
        let mid_angles = [last.angles[0] + (point.angles[0] - last.angles[0]) * 0.5, last.angles[1] + (point.angles[1] - last.angles[1]) * 0.5];
        let mid_axis = kin.forward(&Joints { xyz: Vec3::ZERO, angles: mid_angles }, 0.0).map_err(|err| err.to_string())?.axis;
        let desired = slerp_axis(last.axis, point.axis, 0.5);
        let axis_mid = axis_deg(mid_axis, desired);
        if axis_mid > mid_axis_tol {
            return Err(format!("midpoint axis {axis_mid} > {mid_axis_tol}\n{text}"));
        }
        if !point.rapid {
            let Some(feed) = point.inverse_feed else {
                return Err(format!("feed block is not G93\n{text}"));
            };
            if !(feed.is_finite() && feed > 0.0) {
                return Err(format!("F {feed} is not positive and finite\n{text}"));
            }
            let length = gap(last.tip, point.tip);
            if length > 1e-6 {
                let implied = length * feed;
                let used = if (implied - FEED).abs() <= FEED * 0.08 {
                    FEED
                } else if (implied - RAPID_FEED).abs() <= RAPID_FEED * 0.08 {
                    RAPID_FEED
                } else {
                    return Err(format!("implied feed {implied} is neither {FEED} nor {RAPID_FEED}\n{text}"));
                };
                minutes += 1.0 / feed;
                expected_minutes += length / used;
            }
        }
        posted_tips.push(point.tip);
        posted_axes.push(point.axis);
        prev = Some(point);
    }
    if !settled {
        return Err(format!("posted stream never reached the first pose\n{text}"));
    }
    for (ir_tip, ir_axis) in ir_tips.iter().zip(ir_axes) {
        let hit = posted_tips
            .iter()
            .zip(posted_axes.iter())
            .any(|(got, got_axis)| gap(*got, *ir_tip) <= tip_tol && axis_deg(*got_axis, *ir_axis) <= vertex_axis);
        if !hit {
            return Err(format!("IR pose {:?} missing (tol {tip_tol})\n{text}", (ir_tip.x(), ir_tip.y(), ir_tip.z())));
        }
    }
    if expected_minutes > 1e-9 {
        let rel = (minutes - expected_minutes).abs() / expected_minutes;
        if rel > INVERSE_TIME_REL {
            return Err(format!("inverse time relative error {rel} > {INVERSE_TIME_REL} (sum 1/F {minutes}, expected {expected_minutes})\n{text}"));
        }
    }
    Ok(())
}

fn tilt_blocks(axis: Vec3, second_axis: Vec3) -> Vec<ClBlock> {
    vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("a"), axis), feed: None }),
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(14.0, 5.0, 6.0).expect("b"), second_axis), feed: None }),
    ]
}

#[test]
fn tcp_rapid_that_changes_a_rotary_is_inverse_time_g1() {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let tip = Vec3::new(4.0, 5.0, 6.0).expect("tip");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(tip, Vec3::Z), feed: None }),
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(tip, axis), feed: None }),
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(14.0, 5.0, 6.0).expect("away"), axis), feed: None }),
    ];
    let program = tcp_program_of(setup_of(B_TABLE_JSON), blocks);
    let kin = kinematics(B_TABLE_JSON);
    let joints = kin.choose(&ToolPose { tip, axis }, 0.0, Choice::Nearest([0.0, 0.0])).expect("choose");
    let expected = RAPID_FEED / joints.angles[0].abs();
    let text = post_program(&program, &builtin("haas_umc_tcpc").expect("tcp"), &options()).expect("post");
    haas_tcp_structure(&text).expect("structure");
    let g93 = text.find("G93").expect("G93");
    let g94 = text.find("G94").expect("G94");
    assert!(g93 < g94, "{text}");
    let feed = text.lines().find(|line| token_is(line, "G1") && word_on(line, 'F').is_some()).expect("g1");
    let f = word_on(feed, 'F').expect("f");
    assert!((f - expected).abs() < 5e-4, "{feed} vs {expected}\n{text}");
    assert!(text.contains("G234 H1"), "{text}");
    assert!(text.contains("G43 H1"), "{text}");
    let g49 = text.rfind("G49").expect("G49");
    let g43 = text.rfind("G43").expect("G43");
    assert!(g43 > g49, "restore length after G49\n{text}");
    let linear = text.lines().rev().find(|line| line_is_rapid(line) && word_on(line, 'X').is_some()).expect("linear rapid");
    assert!(rotary_words_on(linear).is_empty(), "{linear}");

    let mut raw: serde_json::Value = serde_json::from_str(include_str!("../posts/haas_umc_tcpc.json")).expect("json");
    raw["five_axis"]["continuous"]["tcp"].as_object_mut().expect("tcp").remove("rapid_feed");
    let post = load_post(&raw.to_string()).expect("default feed");
    let again = post_program(&program, &post, &options()).expect("default");
    assert_eq!(again, text, "omitted rapid_feed must stay 5000");

    raw["five_axis"]["continuous"]["tcp"]["rapid_rotary"] = serde_json::json!("error");
    let post = load_post(&raw.to_string()).expect("error mode");
    let err = post_program(&program, &post, &options()).expect_err("rotary rapid");
    assert!(err.to_string().contains("TCP rapid changes a rotary"), "{err}");
}

#[test]
fn tcp_after_indexed_cancels_g254_before_g234() {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let indexed = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(Vec3::new(4.0, 5.0, 6.0).expect("a"), axis), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(Vec3::new(1.0, 2.0, 3.0).expect("b"), axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let indexed_path = ClPath::try_new(
        Some("index".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(FEED, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        indexed,
    )
    .expect("indexed");
    let mut cont_blocks = tilt_blocks(Vec3::Z, axis);
    cont_blocks.insert(0, ClBlock::Event(Event::ToolChange { tool_id: 7 }));
    let tool = Tool::try_new(7, None, None, None).expect("tool");
    let program = Program::try_new(Some("mix".into()), vec![tool], None, vec![indexed_path, path_of(cont_blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(setup_of(B_TABLE_JSON))
        .expect("five");
    let text = post_program(&program, &builtin("haas_umc_tcpc").expect("tcp"), &options()).expect("post");
    haas_tcp_structure(&text).expect("structure");
    let g255 = text.rfind("G255").expect("G255");
    let g234 = text.rfind("G234").expect("G234");
    assert!(g255 < g234, "G255 before G234\n{text}");
    assert!(text.contains("G234 H7"), "{text}");
    assert!(text.contains("T7 M6"), "{text}");
    let tool_g43 = text.find("G43 H7").expect("tool length");
    assert!(tool_g43 < g234, "tool length before G234\n{text}");
    let linux = post_program(&program, &builtin("linuxcnc_5ax_tcp").expect("tcp"), &options()).expect("linux");
    assert!(linux.contains("(TCP kinematics required)"), "{linux}");
    assert!(!linux.lines().any(|line| token_is(line, "G234")), "{linux}");
    haas_tcp_structure(&linux).expect("linux has no G234 to pair");
}

#[test]
fn tcp_posts_name_the_machine_limit() {
    for id in TCP_POST_IDS {
        let post = builtin(id).expect(id);
        assert!(post.description().ends_with("This output has not been proven on a machine."), "{id}: {}", post.description());
    }
    let linux = builtin("linuxcnc_5ax_tcp").expect("tcp");
    assert!(linux.description().contains("must run TCP kinematics"), "{}", linux.description());
}

proptest! {
    #![proptest_config(ProptestConfig::default())]

    #[test]
    fn posted_tcp_replays_the_part_frame_tip((machine, tilts, spins, near_cone, with_arc) in path_strategy()) {
        let spec = &machines()[machine];
        let Some((blocks, ir_tips, ir_axes, segs)) = build_blocks(spec, &tilts, &spins, near_cone, with_arc) else {
            return Ok(());
        };
        let program = tcp_program_of(setup_of(spec.text), blocks);
        let kin = kinematics(spec.text);
        for id in TCP_POST_IDS {
            let post = builtin(id).expect(id);
            match post_program(&program, &post, &options()) {
                Err(err) => {
                    let message = err.to_string();
                    prop_assert!(
                        message.contains("unreachable")
                            || message.contains("leaves axis")
                            || message.contains("flips the rotary branch")
                            || message.contains("rotary-only")
                            || message.contains("inserts more than")
                            || message.contains("too large"),
                        "{} {}: {message}",
                        spec.label,
                        id
                    );
                }
                Ok(text) => {
                    prop_assert!(!has_g2g3(&text), "{} {} wrote G2/G3\n{text}", spec.label, id);
                    if id == "haas_umc_tcpc" {
                        haas_tcp_structure(&text).map_err(|err| TestCaseError::fail(format!("{}: {err}", spec.label)))?;
                    }
                    if id == "linuxcnc_5ax_tcp" {
                        prop_assert!(text.contains("(TCP kinematics required)"), "{text}");
                    }
                    let moves = read_back(&text, &post)
                        .map_err(|err| TestCaseError::fail(format!("{} {} read: {err}\n{text}", spec.label, id)))?;
                    replay_tcp(&kin, &post, &ir_tips, &ir_axes, &segs, &moves, &text)
                        .map_err(|err| TestCaseError::fail(format!("{} {}: {err}", spec.label, id)))?;
                }
            }
        }
    }
}

#[test]
fn haas_tcpc_without_a_tool_is_refused() {
    let tip = Vec3::new(4.0, 5.0, 6.0).expect("tip");
    let axis = Vec3::new(10_f64.to_radians().sin(), 0.0, 10_f64.to_radians().cos()).expect("axis");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose_of(tip, Vec3::Z), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose_of(Vec3::new(8.0, 5.0, 6.0).expect("b"), axis), feed: FEED, role: MoveRole::Cut, power: None }),
    ];
    let program = program_of(setup_of(B_TABLE_JSON), blocks.clone());
    let err = post_program(&program, &builtin("haas_umc_tcpc").expect("tcp"), &options()).expect_err("no tool");
    assert!(err.to_string().contains("needs a tool number"), "{err}");
    let text = post_program(&tcp_program_of(setup_of(B_TABLE_JSON), blocks), &builtin("haas_umc_tcpc").expect("tcp"), &options()).expect("tool");
    assert!(text.lines().any(|line| line.ends_with("G234 H1")), "{text}");
    assert!(!text.contains("H0"), "{text}");
}
