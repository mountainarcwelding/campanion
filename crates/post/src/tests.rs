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
    assert_eq!(builtin_ids(), &["linuxcnc", "grbl", "mach3", "fanuc", "haas"]);
    for id in builtin_ids() {
        let post = builtin(id).expect(id);
        assert_eq!(post.id(), *id);
        assert!(!post.name().is_empty());
    }
    assert_eq!(builtin("linuxcnc").expect("linuxcnc").linear_decimals(), 4);
    assert_eq!(builtin("linuxcnc").expect("linuxcnc").max_line_length(), Some(256));
    assert_eq!(builtin("grbl").expect("grbl").linear_decimals(), 4);
    assert_eq!(builtin("mach3").expect("mach3").linear_decimals(), 4);
    assert_eq!(builtin("mach3").expect("mach3").max_line_length(), Some(256));
    assert_eq!(builtin("fanuc").expect("fanuc").linear_decimals(), 3);
    assert_eq!(builtin("fanuc").expect("fanuc").max_line_length(), Some(80));
    assert_eq!(builtin("haas").expect("haas").linear_decimals(), 3);
    assert_eq!(builtin("haas").expect("haas").max_line_length(), Some(80));
    assert!(builtin("centroid").is_none());
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
    raw["header"] = serde_json::json!(["T{tool}"]);
    let err = load_post(&raw.to_string()).expect_err("tool in header");
    assert!(err.to_string().contains("tool"), "{err}");
    raw["header"] = serde_json::json!(["O{program_number}"]);
    let err = load_post(&raw.to_string()).expect_err("number missing");
    assert!(err.to_string().contains("program_number"), "{err}");
    raw["program_number"] = serde_json::json!("12A");
    let err = load_post(&raw.to_string()).expect_err("number digits");
    assert!(err.to_string().contains("program_number"), "{err}");
    raw["program_number"] = serde_json::json!("0001");
    raw["tool_change"] = serde_json::json!(["T{tool} M6", "G43 H{tool}"]);
    load_post(&raw.to_string()).expect("tool and program number");
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
    assert!(text.lines().any(|line| line.split_whitespace().any(|word| word == "M3")), "{text}");
    assert!(!text.contains("M3 S"), "{text}");
    assert!(text.contains("M5"), "{text}");
    let moves = read_back(&text, &linux()).expect("read");
    compare(&program, &moves, 1e-3).expect("compare");
    for line in text.lines() {
        assert!(line.chars().count() <= 256, "{line}");
    }
}

#[test]
fn spindle_speed_is_an_integer() {
    let params = ProcessParams::new().speed(Some(12_000.0)).expect("speed");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 1.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
    ];
    let text = post_program(&program(blocks), &linux(), &options()).expect("post");
    assert!(text.lines().any(|line| line.split_whitespace().any(|word| word == "S12000")), "{text}");
    assert!(!text.contains("S12000."), "{text}");
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

fn word_letters(line: &str) -> Vec<char> {
    line.split_whitespace().filter_map(|word| word.chars().next()).collect()
}

#[test]
fn arc_centre_words_stay_in_plane() {
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
        ClBlock::Motion(ClMotion::Helix {
            to: pose(0.0, 10.0, -2.0),
            plane: Plane::Xy,
            center: at(0.0, 0.0, 0.0),
            sense: ArcSense::Ccw,
            sweep: std::f64::consts::TAU,
            feed: 40.0,
            role: MoveRole::Cut,
        }),
    ];
    let text = post_program(&program(blocks), &linux(), &options()).expect("post");
    let xy: Vec<_> = text.lines().filter(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")).collect();
    assert!(xy.len() >= 2, "{text}");
    for line in &xy {
        let letters = word_letters(line);
        assert!(letters.contains(&'I') && letters.contains(&'J'), "{line}");
        assert!(!letters.contains(&'K'), "{line}");
    }
    let helix = xy.last().copied().expect("helix");
    assert!(helix.contains("Z-2.0000"), "{helix}");

    let zx_blocks = vec![
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
    let zx_program = program(zx_blocks);
    let zx = post_program(&zx_program, &linux(), &options()).expect("post");
    let zx_line = zx.lines().find(|line| line.contains("G3")).expect("arc");
    let letters = word_letters(zx_line);
    assert!(letters.contains(&'I') && letters.contains(&'K'), "{zx_line}");
    assert!(!letters.contains(&'J'), "{zx_line}");
    let moves = read_back(&zx, &linux()).expect("read");
    compare(&zx_program, &moves, 1e-3).expect("compare");

    let yz_blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 1.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(0.0, 0.0, 1.0),
            plane: Plane::Yz,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 20.0,
            role: MoveRole::Cut,
        }),
    ];
    let yz_program = program(yz_blocks);
    let yz = post_program(&yz_program, &linux(), &options()).expect("post");
    let yz_line = yz.lines().find(|line| line.split_whitespace().any(|word| word == "G3")).expect("arc");
    let letters = word_letters(yz_line);
    assert!(letters.contains(&'J') && letters.contains(&'K'), "{yz_line}");
    assert!(!letters.contains(&'I'), "{yz_line}");
    compare(&yz_program, &read_back(&yz, &linux()).expect("read"), 1e-3).expect("compare");
}

fn sequence_numbers(text: &str) -> Vec<u32> {
    text.lines()
        .filter_map(|line| {
            let rest = line.strip_prefix('N')?;
            let token = rest.split_whitespace().next()?;
            token.parse().ok()
        })
        .collect()
}

fn arc_lines(text: &str) -> Vec<&str> {
    text.lines().filter(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")).collect()
}

#[test]
fn mach3_header_is_unnumbered_and_ends_with_m30() {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(1.0, 2.0, 3.0), feed: None }),
        ClBlock::Event(Event::ToolChange { tool_id: 7 }),
        ClBlock::Event(Event::Dwell { seconds: 0.5 }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
    ];
    let posted = Program::try_new(
        Some("plate".into()),
        vec![Tool::try_new(7, None, None, None).expect("tool")],
        None,
        vec![path(blocks)],
        Vec::new(),
        Vec::new(),
    )
    .expect("program");
    let post = builtin("mach3").expect("mach3");
    let text = post_program(&posted, &post, &options()).expect("post");
    for code in ["G21", "G90", "G17", "G40", "G49", "G80"] {
        assert!(text.lines().any(|line| line == code), "{code} missing in {text}");
    }
    assert!(text.contains("T7 M6"), "{text}");
    assert!(!text.contains("G43"), "{text}");
    assert!(text.contains("G4 P0.5"), "{text}");
    assert!(text.lines().any(|line| line == "M3"), "{text}");
    assert!(!text.contains('%'), "{text}");
    assert!(!text.lines().any(|line| line.starts_with('N') || line.starts_with('O')), "{text}");
    assert!(text.trim_end().ends_with("M30"), "{text}");
    let moves = read_back(&text, &post).expect("read");
    compare(&program(vec![ClBlock::Motion(ClMotion::Rapid { to: pose(1.0, 2.0, 3.0), feed: None })]), &moves, 1e-3).expect("compare");
}

#[test]
fn fanuc_splits_full_circles_and_linearises_helices() {
    let params = ProcessParams::new().speed(Some(8_000.0)).expect("speed");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 0.0, 0.0), feed: None }),
        ClBlock::Event(Event::ToolChange { tool_id: 2 }),
        ClBlock::Event(Event::Dwell { seconds: 0.25 }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(10.0, 0.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Helix {
            to: pose(10.0, 0.0, -1.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            sweep: std::f64::consts::TAU,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
    ];
    let posted = Program::try_new(
        Some("plate".into()),
        vec![Tool::try_new(2, None, None, None).expect("tool")],
        None,
        vec![path(blocks)],
        Vec::new(),
        Vec::new(),
    )
    .expect("program");
    let post = builtin("fanuc").expect("fanuc");
    let text = post_program(&posted, &post, &options()).expect("post");
    assert!(text.starts_with("%\nO0001\n"), "{text}");
    assert!(text.trim_end().ends_with('%'), "{text}");
    assert!(text.contains("G21 G90 G17 G40 G49 G80"), "{text}");
    assert!(text.contains("T2 M6"), "{text}");
    assert!(text.contains("G43 H2"), "{text}");
    assert!(text.contains("G4 P250"), "{text}");
    assert!(text.contains("M3 S8000"), "{text}");
    assert!(text.contains("M30\n%"), "{text}");
    let numbers = sequence_numbers(&text);
    assert_eq!(numbers.first().copied(), Some(10), "{text}");
    for pair in numbers.windows(2) {
        assert_eq!(pair[1] - pair[0], 10, "{text}");
    }
    assert!(!text.lines().any(|line| line.starts_with('%') && line.contains('N')), "{text}");
    assert!(text.lines().any(|line| line == "O0001"), "{text}");
    let arcs = arc_lines(&text);
    assert_eq!(arcs.len(), 2, "{text}");
    for line in arcs {
        assert!(line.split_whitespace().any(|word| word.starts_with('R') && word != "R"), "{line}");
        assert!(!line.split_whitespace().any(|word| matches!(word.chars().next(), Some('I' | 'J' | 'K'))), "{line}");
    }
    assert!(text.contains("G1 "), "{text}");
    assert!(text.contains("Z-1\n"), "{text}");
    let moves = read_back(&text, &post).expect("read");
    compare(&posted, &moves, 1e-3).expect("compare");
}

#[test]
fn haas_keeps_ijk_helices_and_five_digit_program_number() {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 0.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(10.0, 0.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Cw,
            feed: 50.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Helix {
            to: pose(10.0, 0.0, -1.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Cw,
            sweep: std::f64::consts::TAU,
            feed: 40.0,
            role: MoveRole::Cut,
        }),
    ];
    let posted = program(blocks);
    let post = builtin("haas").expect("haas");
    let text = post_program(&posted, &post, &options()).expect("post");
    assert!(text.starts_with("%\nO00001\n"), "{text}");
    assert!(text.trim_end().ends_with('%'), "{text}");
    assert!(text.contains("M30\n%"), "{text}");
    let arcs = arc_lines(&text);
    assert_eq!(arcs.len(), 2, "{text}");
    for line in &arcs {
        let letters = word_letters(line);
        assert!(letters.contains(&'I') && letters.contains(&'J'), "{line}");
        assert!(!letters.contains(&'K'), "{line}");
        assert!(!line.split_whitespace().any(|word| word.starts_with('R')), "{line}");
    }
    assert!(arcs[1].contains("Z-1"), "{text}");
    let moves = read_back(&text, &post).expect("read");
    compare(&posted, &moves, 1e-3).expect("compare");
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
