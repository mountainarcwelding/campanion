//! Indexed 3+2: work-frame rotation, machine-frame inverse, and the posted errors.

#![allow(clippy::expect_used)]

use campanion_ir::{
    ArcSense, Carrier, Choice, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, FiveAxisSetup, Kinematics, MoveRole, Plane, Pose, Program,
    SolutionPolicy, ToolPose, Vec3,
};
use campanion_post::{PostOptions, builtin, post_program};

const B_TABLE: &str = r#"{
  "name": "b_table",
  "rotaries": [{
    "letter": "B",
    "carrier": "table",
    "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
    "pivot": {"x": 12.0, "y": 0.0, "z": 7.0}
  }]
}"#;

const B_HEAD: &str = r#"{
  "name": "b_head",
  "rotaries": [{
    "letter": "B",
    "carrier": "head",
    "axis": {"x": 0.0, "y": 1.0, "z": 0.0},
    "pivot": {"x": 0.0, "y": 0.0, "z": 0.0}
  }]
}"#;

fn kinematics(json: &str) -> Kinematics {
    Kinematics::from_json(json).expect("kinematics")
}

fn setup(json: &str) -> FiveAxisSetup {
    FiveAxisSetup { kinematics: kinematics(json), policy: SolutionPolicy::Nearest }
}

fn pose(tip: Vec3, axis: Vec3) -> Pose {
    Pose::new(tip, axis).expect("pose")
}

fn path(blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some("cut".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
}

fn program(five: Option<FiveAxisSetup>, blocks: Vec<ClBlock>) -> Program {
    let program = Program::try_new(Some("plate".into()), Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new(), None).expect("program");
    match five {
        Some(five) => program.with_five_axis(five).expect("five"),
        None => program,
    }
}

fn options(tol: f64) -> PostOptions {
    PostOptions { arc_tolerance: tol }
}

fn word(line: &str, letter: char) -> Option<f64> {
    line.split_whitespace().find_map(|token| {
        let rest = token.strip_prefix(letter)?;
        if rest.is_empty() || !rest.starts_with(|ch: char| ch.is_ascii_digit() || matches!(ch, '-' | '+' | '.')) {
            return None;
        }
        rest.parse().ok()
    })
}

fn near(actual: f64, expected: f64, tol: f64) -> bool {
    (actual - expected).abs() <= tol
}

fn has_code(line: &str, code: &str) -> bool {
    line.split_whitespace().any(|token| token == code)
}

/// Right-handed 90° about +Y: `(x, y, z) -> (z, y, -x)`.
fn rotate_y_90(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (z, y, -x)
}

fn b90_blocks() -> Vec<ClBlock> {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let first = pose(Vec3::new(4.0, 5.0, 6.0).expect("tip"), axis);
    let second = pose(Vec3::new(1.0, 2.0, 3.0).expect("tip"), axis);
    vec![
        ClBlock::Motion(ClMotion::Rapid { to: first, feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: second, feed: 100.0, role: MoveRole::Cut }),
    ]
}

#[test]
fn work_mode_rotates_a_point_about_the_origin_for_b90() {
    let post = builtin("haas_umc").expect("haas_umc");
    let kin = kinematics(B_TABLE);
    assert_eq!(kin.rotaries[0].carrier, Carrier::Table);
    let text = post_program(&program(Some(setup(B_TABLE)), b90_blocks()), &post, &options(0.05)).expect("post");
    assert!(text.contains("B90."), "{text}");
    assert!(text.contains("G254"), "{text}");
    assert!(text.contains("G255"), "{text}");
    // Pivot (12, 0, 7) must not move the written point. Inverse does use it.
    let tip = Vec3::new(4.0, 5.0, 6.0).expect("tip");
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let joints = kin.choose(&ToolPose { tip, axis }, 0.0, Choice::Nearest([0.0, 0.0])).expect("choose");
    let (x, y, z) = rotate_y_90(4.0, 5.0, 6.0);
    assert!((joints.xyz.x() - x).abs() > 0.5, "inverse should see the pivot: {:?}", joints.xyz);
    let xy = text.lines().find(|line| word(line, 'X') == Some(x) && word(line, 'Y') == Some(y)).expect("xy block");
    assert!(word(xy, 'Z').is_none(), "X and Y share a block: {xy}");
    let z_line = text.lines().find(|line| word(line, 'Z') == Some(z) && word(line, 'X').is_none()).expect("z block");
    assert!(z_line.contains("G0"), "{z_line}");
    let (x2, y2, z2) = rotate_y_90(1.0, 2.0, 3.0);
    let feed = text.lines().find(|line| has_code(line, "G1")).expect("feed");
    assert!(near(word(feed, 'X').expect("x"), x2, 1e-6), "{feed}");
    assert!(near(word(feed, 'Y').expect("y"), y2, 1e-6), "{feed}");
    assert!(near(word(feed, 'Z').expect("z"), z2, 1e-6), "{feed}");
    assert!(text.contains("G00 B0."), "return index: {text}");
}

#[test]
fn machine_mode_matches_inverse_and_splits_xy_then_z() {
    let post = builtin("linuxcnc_5ax").expect("linuxcnc_5ax");
    let kin = kinematics(B_TABLE);
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let tips = [(4.0, 5.0, 6.0), (1.0, 2.0, 3.0)];
    let mut expected = Vec::new();
    let mut angles = [0.0, 0.0];
    for (x, y, z) in tips {
        let tip = Vec3::new(x, y, z).expect("tip");
        let joints = kin.choose(&ToolPose { tip, axis }, 0.0, Choice::Nearest(angles)).expect("choose");
        let again = kin.inverse(&ToolPose { tip, axis }, 0.0, angles).expect("inverse");
        let matched = again.iter().find(|sol| (sol.angles[0] - joints.angles[0]).abs() < 1e-6).expect("solution");
        assert!((matched.xyz.x() - joints.xyz.x()).abs() < 1e-9);
        assert!((matched.xyz.y() - joints.xyz.y()).abs() < 1e-9);
        assert!((matched.xyz.z() - joints.xyz.z()).abs() < 1e-9);
        expected.push(joints);
        angles = joints.angles;
    }
    let text = post_program(&program(Some(setup(B_TABLE)), b90_blocks()), &post, &options(0.05)).expect("post");
    let first = &expected[0];
    assert!(text.contains("B90.000") || text.lines().any(|line| word(line, 'B').is_some_and(|angle| near(angle, first.angles[0], 1e-3))), "{text}");
    let xy = text
        .lines()
        .find(|line| {
            word(line, 'X').is_some_and(|value| near(value, first.xyz.x(), 5e-4))
                && word(line, 'Y').is_some_and(|value| near(value, first.xyz.y(), 5e-4))
        })
        .expect("xy");
    assert!(word(xy, 'Z').is_none(), "X and Y share a block: {xy}");
    let z_line =
        text.lines().find(|line| word(line, 'Z').is_some_and(|value| near(value, first.xyz.z(), 5e-4)) && word(line, 'X').is_none()).expect("z");
    assert!(z_line.contains("G0"), "{z_line}");
    let second = &expected[1];
    let feed = text.lines().find(|line| has_code(line, "G1")).expect("feed");
    assert!(near(word(feed, 'X').expect("x"), second.xyz.x(), 5e-4), "{feed} vs {:?}", second.xyz);
    assert!(near(word(feed, 'Y').expect("y"), second.xyz.y(), 5e-4), "{feed}");
    assert!(near(word(feed, 'Z').expect("z"), second.xyz.z(), 5e-4), "{feed}");
}

#[test]
fn continuous_toolpath_is_rejected_before_indexed_output() {
    let axis_z = Vec3::Z;
    let constant = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(Vec3::new(1.0, 2.0, 3.0).expect("a"), axis_z), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(Vec3::new(4.0, 2.0, 3.0).expect("b"), axis_z), feed: 80.0, role: MoveRole::Cut }),
    ];
    let cases = [(constant, None), (b90_blocks(), Some(setup(B_TABLE)))];
    for (blocks, five) in cases {
        let path = path(blocks).with_continuous(true);
        let program = Program::try_new(Some("plate".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let program = match five {
            Some(five) => program.with_five_axis(five).expect("five"),
            None => program,
        };
        for id in ["haas", "haas_umc", "linuxcnc"] {
            let err = post_program(&program, &builtin(id).expect(id), &options(0.05)).expect_err(id);
            assert_eq!(err.to_string(), format!("not supported: post `{id}` has no continuous five-axis output"));
        }
    }
}

#[test]
fn toolpath_rejects_an_axis_change_inside_a_move() {
    let axis_z = Vec3::Z;
    let axis_y = Vec3::new(0.0, -1.0, 0.0).expect("axis");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(Vec3::new(1.0, 2.0, 3.0).expect("a"), axis_z), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(Vec3::new(1.0, 2.0, 4.0).expect("b"), axis_y), feed: 80.0, role: MoveRole::Cut }),
    ];
    let err = post_program(&program(Some(setup(B_TABLE)), blocks), &builtin("haas_umc").expect("post"), &options(0.05)).expect_err("axis");
    assert_eq!(err.to_string(), "invalid input: toolpath 0 changes tool axis inside a 3+2 move");
}

#[test]
fn exact_five_axis_errors() {
    let tilted = pose(Vec3::new(1.0, 2.0, 3.0).expect("tip"), Vec3::new(0.0, -1.0, 0.0).expect("axis"));
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: tilted, feed: None })];
    let missing = post_program(&program(None, blocks.clone()), &builtin("haas").expect("haas"), &options(0.05)).expect_err("haas");
    assert_eq!(missing.to_string(), "not supported: post `haas` has no tilted-plane output");
    let no_kin = post_program(&program(None, blocks), &builtin("haas_umc").expect("haas_umc"), &options(0.05)).expect_err("kinematics");
    assert_eq!(no_kin.to_string(), "invalid input: program has tilted tool axes but no kinematics");
    let head = pose(Vec3::new(0.0, 0.0, 5.0).expect("tip"), Vec3::Z);
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: head, feed: None })];
    let err = post_program(&program(Some(setup(B_HEAD)), blocks), &builtin("haas_umc").expect("haas_umc"), &options(0.05)).expect_err("head");
    assert_eq!(err.to_string(), "not supported: post `haas_umc` work mode needs table rotaries only");
}

#[test]
fn non_cardinal_arc_uses_the_exact_error_or_chords() {
    let axis = Vec3::new(-0.5, 0.0, 0.8660254037844386).expect("axis");
    let start = pose(Vec3::new(1.0, 0.0, 0.0).expect("start"), axis);
    let end = pose(Vec3::new(0.0, 1.0, 0.0).expect("end"), axis);
    let center = Vec3::new(0.0, 0.0, 0.0).expect("center");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
        ClBlock::Motion(ClMotion::Arc { to: end, plane: Plane::Xy, center, sense: ArcSense::Ccw, feed: 100.0, role: MoveRole::Cut }),
    ];
    let err = post_program(&program(Some(setup(B_TABLE)), blocks.clone()), &builtin("haas_umc").expect("post"), &options(0.0)).expect_err("arc");
    assert_eq!(err.to_string(), "invalid input: arc is not in a cardinal plane after the 3+2 rotation");
    let text = post_program(&program(Some(setup(B_TABLE)), blocks), &builtin("haas_umc").expect("post"), &options(0.05)).expect("chords");
    assert!(!text.lines().any(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")), "{text}");
    assert!(text.lines().any(|line| has_code(line, "G1")), "{text}");
}

#[test]
fn cardinal_arc_after_b90_stays_an_arc() {
    let axis = Vec3::new(-1.0, 0.0, 0.0).expect("axis");
    let start = pose(Vec3::new(1.0, 0.0, 0.0).expect("start"), axis);
    let end = pose(Vec3::new(0.0, 1.0, 0.0).expect("end"), axis);
    let center = Vec3::new(0.0, 0.0, 0.0).expect("center");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
        ClBlock::Motion(ClMotion::Arc { to: end, plane: Plane::Xy, center, sense: ArcSense::Ccw, feed: 100.0, role: MoveRole::Cut }),
    ];
    let text = post_program(&program(Some(setup(B_TABLE)), blocks), &builtin("haas_umc").expect("post"), &options(0.05)).expect("arc");
    let arc = text.lines().find(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")).expect("arc line");
    assert!(arc.contains("G19"), "{arc}");
    assert!(arc.contains("G3"), "{arc}");
    assert!(arc.contains("J0."), "{arc}");
    assert!(arc.contains("K1."), "{arc}");
}

#[test]
fn a_vertical_program_matches_the_post_without_five_axis() {
    let to = pose(Vec3::new(1.0, 2.0, 3.0).expect("tip"), Vec3::Z);
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to, feed: None })];
    let program = program(None, blocks);
    let options = options(0.05);
    let haas = post_program(&program, &builtin("haas").expect("haas"), &options).expect("haas");
    let umc = post_program(&program, &builtin("haas_umc").expect("haas_umc"), &options).expect("umc");
    assert_eq!(haas, umc);
    let linux = post_program(&program, &builtin("linuxcnc").expect("linuxcnc"), &options).expect("linuxcnc");
    let five = post_program(&program, &builtin("linuxcnc_5ax").expect("linuxcnc_5ax"), &options).expect("5ax");
    assert_eq!(linux, five);
}
