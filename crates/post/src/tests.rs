use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Plane, Pose, ProcessKind, ProcessParams, Program, Tool, Vec3,
};

use crate::{PostOptions, builtin, builtin_ids, compare, load_post, post_program, read_back};

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn at(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z).expect("point")
}

fn path(blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some("outer".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
}

fn program(blocks: Vec<ClBlock>) -> Program {
    Program::try_new(Some("plate".into()), Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new()).expect("program")
}

fn options() -> PostOptions {
    PostOptions { arc_tolerance: 0.05 }
}

fn linux() -> crate::PostDef {
    builtin("linuxcnc").expect("linuxcnc")
}

#[test]
fn builtins_load_and_match_ids() {
    assert_eq!(builtin_ids(), &["linuxcnc", "grbl"]);
    for id in builtin_ids() {
        let post = builtin(id).expect(id);
        assert_eq!(post.id(), *id);
        assert_eq!(post.linear_decimals(), 4);
        assert_eq!(post.max_line_length(), Some(256));
    }
    assert!(builtin("fanuc").is_none());
}

#[test]
fn validation_names_the_field() {
    let mut raw = serde_json::from_str::<serde_json::Value>(include_str!("../posts/linuxcnc.json")).expect("json");
    raw["units"] = serde_json::json!("inch");
    let err = load_post(&raw.to_string()).expect_err("units");
    assert!(err.to_string().contains("units"), "{err}");
    raw["units"] = serde_json::json!("mm");
    raw["nope"] = serde_json::json!(1);
    let err = load_post(&raw.to_string()).expect_err("unknown");
    assert!(err.to_string().contains("nope") || err.to_string().contains("unknown field"), "{err}");
    raw.as_object_mut().expect("object").remove("nope");
    raw["arcs"]["max_sweep_deg"] = serde_json::json!(0);
    let err = load_post(&raw.to_string()).expect_err("sweep");
    assert!(err.to_string().contains("arcs.max_sweep_deg"), "{err}");
    raw["arcs"]["max_sweep_deg"] = serde_json::json!(360);
    raw["header"] = serde_json::json!(["{date}"]);
    let err = load_post(&raw.to_string()).expect_err("date");
    assert!(err.to_string().contains("header"), "{err}");
    assert!(err.to_string().contains("date"), "{err}");
    raw["header"] = serde_json::json!(["{not_a_thing}"]);
    let err = load_post(&raw.to_string()).expect_err("placeholder");
    assert!(err.to_string().contains("header"), "{err}");
}

#[test]
fn linuxcnc_preamble_and_modal_lines() {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 5.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(10.0, 0.0, 5.0), feed: 250.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: pose(12.0, 0.0, 5.0), feed: 250.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::Dwell { seconds: 0.5 }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
    ];
    let program = program(blocks);
    let text = post_program(&program, &linux(), &options()).expect("post");
    assert!(text.lines().next().unwrap().starts_with("N10 (plate)"), "{text}");
    assert!(text.contains("G21\n") || text.contains("G21 "), "{text}");
    assert!(text.contains("G64 P0.01"), "{text}");
    assert!(text.contains("G40"), "{text}");
    assert!(text.contains("G49"), "{text}");
    assert!(text.contains("\nN") && text.trim_end().ends_with("M2"), "{text}");
    assert!(text.contains("G1 X10.0000 Y0.0000 Z5.0000 F250.0000"), "{text}");
    assert!(text.contains("X12.0000 Y0.0000 Z5.0000\n") || text.contains("X12.0000 Y0.0000 Z5.0000 "), "{text}");
    assert!(!text.contains("G1 X12.0000"), "{text}");
    assert!(text.contains("G4 P0.500"), "{text}");
    assert!(text.contains("M3 S0.0000"), "{text}");
    assert!(text.contains("M5"), "{text}");
    let moves = read_back(&text, &linux()).expect("read");
    compare(&program, &moves, 1e-3).expect("compare");
    for line in text.lines() {
        assert!(line.chars().count() <= 256, "{line}");
    }
}

#[test]
fn grbl_comments_tool_change_and_has_no_line_numbers() {
    let tool = Tool::try_new(4, None, None, None).expect("tool");
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(1.0, 2.0, 3.0), feed: None }), ClBlock::Event(Event::ToolChange { tool_id: 4 })];
    let path = path(blocks.clone());
    let program = Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new()).expect("program");
    let post = builtin("grbl").expect("grbl");
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(text.starts_with("G21\n"), "{text}");
    assert!(text.contains("(tool 4)"), "{text}");
    assert!(!text.contains("M6"), "{text}");
    assert!(!text.lines().any(|line| line.starts_with('N')), "{text}");
    assert!(text.trim_end().ends_with("M30"), "{text}");
    assert!(text.contains("X1 Y2 Z3") || text.contains("X1.0 Y2.0 Z3.0") || text.contains("X1.0000"), "{text}");
    let moves = read_back(&text, &post).expect("read");
    compare(&program, &moves, 1e-3).expect("compare");
}

fn round_trip(post_json: &str, blocks: Vec<ClBlock>, tol: f64) {
    let post = load_post(post_json).expect("post");
    let program = program(blocks);
    let text = post_program(&program, &post, &options()).expect("post");
    let moves = read_back(&text, &post).expect("read");
    compare(&program, &moves, tol).expect("compare");
}

#[test]
fn ijk_arc_helix_and_full_circle_round_trip() {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 0.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(0.0, 10.0, 0.0),
            plane: Plane::Xy,
            center: at(0.0, 0.0, 0.0),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(0.0, 10.0, 0.0),
            plane: Plane::Xy,
            center: at(0.0, 0.0, 0.0),
            sense: ArcSense::Cw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Helix {
            to: pose(0.0, 10.0, -2.0),
            plane: Plane::Xy,
            center: at(0.0, 0.0, 0.0),
            sense: ArcSense::Ccw,
            sweep: std::f64::consts::TAU * 2.0,
            feed: 40.0,
            role: MoveRole::Cut,
        }),
    ];
    round_trip(include_str!("../posts/linuxcnc.json"), blocks, 1e-3);
}

#[test]
fn zx_arc_and_r_mode_and_lines_round_trip() {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 1.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(1.0, 0.0, 0.0),
            plane: Plane::Zx,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 30.0,
            role: MoveRole::Cut,
        }),
    ];
    round_trip(include_str!("../posts/grbl.json"), blocks, 1e-3);

    let mut raw = serde_json::from_str::<serde_json::Value>(include_str!("../posts/grbl.json")).expect("json");
    raw["arcs"]["mode"] = serde_json::json!("r");
    raw["arcs"]["max_sweep_deg"] = serde_json::json!(180);
    let r_blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(1.0, 0.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(0.0, -1.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 20.0,
            role: MoveRole::Cut,
        }),
    ];
    round_trip(&raw.to_string(), r_blocks, 1e-3);

    raw["arcs"]["mode"] = serde_json::json!("lines");
    raw["arcs"]["helix"] = serde_json::json!(false);
    let line_blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 0.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(-10.0, 0.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 20.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Helix {
            to: pose(-10.0, 0.0, -1.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Cw,
            sweep: std::f64::consts::TAU,
            feed: 15.0,
            role: MoveRole::Cut,
        }),
    ];
    round_trip(&raw.to_string(), line_blocks, 1e-3);
}

#[test]
fn channel_programs_and_controller_compensation_are_rejected() {
    let program: Program =
        serde_json::from_str(r#"{"tools":[],"toolpaths":[],"channel_programs":[{"channel":0,"blocks":[]}],"syncs":[]}"#).expect("json");
    let err = post_program(&program, &linux(), &options()).expect_err("channel");
    assert!(err.to_string().contains("not supported"), "{err}");
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 0.0), feed: None })];
    let path = ClPath::try_new(
        None,
        None,
        CompensationMode::ControllerLeft,
        Feeds::try_new(10.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path");
    let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new()).expect("program");
    let err = post_program(&program, &linux(), &options()).expect_err("comp");
    assert!(err.to_string().contains("not supported"), "{err}");
}

#[test]
fn long_lines_are_rejected() {
    let mut raw = serde_json::from_str::<serde_json::Value>(include_str!("../posts/grbl.json")).expect("json");
    raw["max_line_length"] = serde_json::json!(8);
    let post = load_post(&raw.to_string()).expect("post");
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(100.0, 0.0, 0.0), feed: None })];
    let err = post_program(&program(blocks), &post, &options()).expect_err("length");
    assert!(err.to_string().contains("max_line_length"), "{err}");
}
