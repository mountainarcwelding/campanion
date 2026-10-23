//! Lathe posts: diameter X, radius I/K, G2/G3, spindle speed and the turning fixture.

#![allow(clippy::expect_used)]

use std::path::PathBuf;

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Document, Event, Feeds, Hand, Head, HeadAxis, Insert, InsertShape, MoveRole,
    Plane, Pose, ProcessKind, ProcessParams, Program, Spindle, SpindleSpeed, Tool, TurnBinding, TurnInsert, TurnStock, TurningSetup, Vec3,
};
use campanion_post::{PostDef, PostOptions, builtin, compare, load_post, post_program, read_back};

fn options() -> PostOptions {
    PostOptions { arc_tolerance: 0.05 }
}

fn xz(x: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, 0.0, z).expect("tip"), Vec3::Z).expect("pose")
}

fn yz(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn feeds() -> Feeds {
    Feeds::try_new(100.0, None, None, None, None).expect("feeds")
}

fn path(name: Option<&str>, tool_id: Option<u32>, blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(name.map(str::to_string), tool_id, CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path")
}

fn binding(speed: Option<SpindleSpeed>) -> TurnBinding {
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding");
    match speed {
        Some(speed) => binding.with_speed(Some(speed)).expect("speed"),
        None => binding,
    }
}

fn turning(speed: Option<SpindleSpeed>, reversed: bool) -> TurningSetup {
    let spindle = Spindle::try_new("main", None, 0.0, 3_000.0).expect("spindle").with_reversed(reversed);
    let head = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("head");
    TurningSetup::try_new(vec![spindle], vec![head], None, Vec::new(), vec![binding(speed)]).expect("turning")
}

fn lathe_program(blocks: Vec<ClBlock>, setup: TurningSetup) -> Program {
    Program::try_new(Some("turn".into()), Vec::new(), None, vec![path(None, None, blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning")
}

fn with_tool(blocks: Vec<ClBlock>, setup: TurningSetup) -> Program {
    let tool = Tool::try_new(1, Some("cnmg".into()), None, None).expect("tool");
    Program::try_new(Some("turn".into()), vec![tool], None, vec![path(Some("finish"), Some(1), blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning")
}

fn linux_lathe() -> PostDef {
    builtin("linuxcnc_lathe").expect("linuxcnc_lathe")
}

fn lathe_json() -> serde_json::Value {
    serde_json::from_str(include_str!("../posts/linuxcnc_lathe.json")).expect("json")
}

fn post_from(value: serde_json::Value) -> PostDef {
    load_post(&value.to_string()).expect("post")
}

fn spindle_on(speed: Option<f64>) -> ClBlock {
    let mut params = ProcessParams::new();
    if let Some(speed) = speed {
        params = params.speed(Some(speed)).expect("speed");
    }
    ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params })
}

fn spindle_off() -> ClBlock {
    ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle })
}

fn rapid(x: f64, z: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: xz(x, z), feed: None })
}

fn linear(x: f64, z: f64, feed: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: xz(x, z), feed, role: MoveRole::Cut })
}

/// Quarter-circle in ZX. Start (10, 0), centre (5, 0), end (5, -5), counter-clockwise, radius 5.
/// `I` is -5. Doubling that offset would write -10.
fn quarter_blocks() -> Vec<ClBlock> {
    vec![
        rapid(10.0, 0.0),
        ClBlock::Motion(ClMotion::Arc {
            to: xz(5.0, -5.0),
            plane: Plane::Zx,
            center: Vec3::new(5.0, 0.0, 0.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
    ]
}

fn words_of(text: &str) -> Vec<String> {
    text.split_whitespace().map(str::to_string).collect()
}

fn has_word(text: &str, word: &str) -> bool {
    words_of(text).iter().any(|token| token == word)
}

#[test]
fn diameter_mode_doubles_x_and_keeps_i_as_a_radius() {
    let program = lathe_program(quarter_blocks(), turning(None, false));
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(has_word(&text, "X20.0000"), "{text}");
    assert!(has_word(&text, "X10.0000"), "{text}");
    assert!(has_word(&text, "Z-5.0000"), "{text}");
    assert!(has_word(&text, "I-5.0000"), "{text}");
    assert!(has_word(&text, "K0.0000"), "{text}");
    assert!(!has_word(&text, "I-10.0000"), "{text}");
    assert!(!text.split_whitespace().any(|word| word.starts_with('Y')), "{text}");
    assert!(text.contains("G7") && text.contains("G18") && text.contains("G94"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let mut raw = lathe_json();
    raw["lathe"]["diameter_mode"] = serde_json::json!(false);
    raw["lathe"]["mode_lines"] = serde_json::json!(["G8"]);
    let radius_post = post_from(raw);
    let radius_text = post_program(&program, &radius_post, &options()).expect("radius");
    assert!(has_word(&radius_text, "X10.0000"), "{radius_text}");
    assert!(!has_word(&radius_text, "X20.0000"), "{radius_text}");
    assert!(has_word(&radius_text, "I-5.0000"), "{radius_text}");
    assert!(radius_text.contains("G8"), "{radius_text}");
    compare(&program, &read_back(&radius_text, &radius_post).expect("read"), 1e-3, &radius_post).expect("radius compare");
}

#[test]
fn arc_sense_follows_the_zx_picture_and_arc_flip_swaps_it() {
    let program = lathe_program(quarter_blocks(), turning(None, false));
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(has_word(&text, "G3"), "{text}");
    assert!(!has_word(&text, "G2"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let mut raw = lathe_json();
    raw["lathe"]["arc_flip"] = serde_json::json!(true);
    let flipped = post_from(raw.clone());
    let flipped_text = post_program(&program, &flipped, &options()).expect("flip");
    assert!(has_word(&flipped_text, "G2"), "{flipped_text}");
    assert!(!has_word(&flipped_text, "G3"), "{flipped_text}");
    assert!(has_word(&flipped_text, "I-5.0000"), "{flipped_text}");
    compare(&program, &read_back(&flipped_text, &flipped).expect("read"), 1e-3, &flipped).expect("flip compare");

    raw["arcs"]["mode"] = serde_json::json!("r");
    let radius_mode = post_from(raw);
    let radius_text = post_program(&program, &radius_mode, &options()).expect("r");
    assert!(has_word(&radius_text, "R5.0000"), "{radius_text}");
    compare(&program, &read_back(&radius_text, &radius_mode).expect("read"), 1e-3, &radius_mode).expect("r compare");
}

#[test]
fn spindle_lines_follow_css_rpm_the_process_param_and_reversed() {
    let css =
        lathe_program(vec![spindle_on(None), rapid(10.0, 1.0), spindle_off()], turning(Some(SpindleSpeed::css(200.0, 2_500.0).expect("css")), false));
    let css_text = post_program(&css, &linux_lathe(), &options()).expect("css");
    assert!(css_text.contains("G96 D2500 S200"), "{css_text}");
    assert!(has_word(&css_text, "M3"), "{css_text}");
    assert!(!css_text.contains("G97"), "{css_text}");
    assert!(css_text.contains("M5"), "{css_text}");
    let css_at = css_text.find("G96").expect("g96");
    let m3_at = css_text.find("M3").expect("m3");
    assert!(css_at < m3_at, "{css_text}");

    let rpm = lathe_program(vec![spindle_on(None), rapid(10.0, 1.0), spindle_off()], turning(Some(SpindleSpeed::rpm(1_500.0).expect("rpm")), false));
    let rpm_text = post_program(&rpm, &linux_lathe(), &options()).expect("rpm");
    assert!(rpm_text.contains("G97 S1500"), "{rpm_text}");
    assert!(has_word(&rpm_text, "M3"), "{rpm_text}");
    assert!(!rpm_text.contains("G96"), "{rpm_text}");

    let fraction =
        lathe_program(vec![spindle_on(None), rapid(10.0, 1.0), spindle_off()], turning(Some(SpindleSpeed::css(12.5, 2_500.0).expect("css")), false));
    let fraction_text = post_program(&fraction, &linux_lathe(), &options()).expect("fraction");
    assert!(fraction_text.contains("G96 D2500 S12.5000"), "{fraction_text}");

    let reversed =
        lathe_program(vec![spindle_on(None), rapid(10.0, 1.0), spindle_off()], turning(Some(SpindleSpeed::rpm(1_000.0).expect("rpm")), true));
    let reversed_text = post_program(&reversed, &linux_lathe(), &options()).expect("reversed");
    assert!(reversed_text.contains("G97 S1000"), "{reversed_text}");
    assert!(has_word(&reversed_text, "M4"), "{reversed_text}");
    assert!(!has_word(&reversed_text, "M3"), "{reversed_text}");

    let from_param = lathe_program(vec![spindle_on(Some(1_800.0)), rapid(10.0, 1.0), spindle_off()], turning(None, false));
    let param_text = post_program(&from_param, &linux_lathe(), &options()).expect("param");
    assert!(param_text.contains("G97 S1800"), "{param_text}");

    let missing = lathe_program(vec![spindle_on(None), rapid(10.0, 1.0)], turning(None, false));
    let err = post_program(&missing, &linux_lathe(), &options()).expect_err("missing");
    assert!(err.to_string().contains("lathe spindle speed is missing"), "{err}");
}

#[test]
fn tool_change_uses_the_lathe_template_and_the_two_digit_offset() {
    let program = with_tool(vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), rapid(10.0, 1.0)], turning(None, false));
    let text = post_program(&program, &linux_lathe(), &options()).expect("post");
    assert!(text.contains("T1 M6"), "{text}");
    assert!(has_word(&text, "G43"), "{text}");

    let mut raw = lathe_json();
    raw["lathe"]["tool_change"] = serde_json::json!(["T{tool2}{offset2}"]);
    let post = post_from(raw);
    let padded = post_program(&program, &post, &options()).expect("padded");
    assert!(has_word(&padded, "T0101"), "{padded}");
}

#[test]
fn coolant_and_dwell_match_the_mill_words() {
    let program = lathe_program(
        vec![
            rapid(10.0, 1.0),
            ClBlock::Event(Event::Coolant { mode: CoolantMode::Flood }),
            ClBlock::Event(Event::Dwell { seconds: 0.5 }),
            ClBlock::Event(Event::Coolant { mode: CoolantMode::Off }),
        ],
        turning(None, false),
    );
    let text = post_program(&program, &linux_lathe(), &options()).expect("post");
    assert!(has_word(&text, "M8"), "{text}");
    assert!(text.contains("G4 P0.500"), "{text}");
    assert!(has_word(&text, "M9"), "{text}");
    compare(&program, &read_back(&text, &linux_lathe()).expect("read"), 1e-3, &linux_lathe()).expect("compare");
}

#[test]
fn mill_and_lathe_posts_reject_the_other_kind_of_program() {
    let turning_program = lathe_program(vec![rapid(10.0, 1.0)], turning(None, false));
    let err = post_program(&turning_program, &builtin("linuxcnc").expect("linuxcnc"), &options()).expect_err("mill post");
    assert!(err.to_string().contains("post `linuxcnc` is not a lathe post"), "{err}");

    let mill = Program::try_new(Some("mill".into()), Vec::new(), None, vec![path(None, None, vec![rapid(1.0, 2.0)])], Vec::new(), Vec::new(), None)
        .expect("mill");
    let err = post_program(&mill, &linux_lathe(), &options()).expect_err("lathe post");
    assert!(err.to_string().contains("lathe post `linuxcnc_lathe` needs a turning program"), "{err}");
}

#[test]
fn a_lathe_pose_off_the_xz_plane_is_rejected() {
    // An unbound path is still a lathe program. The IR only checks Y on a bound toolpath.
    let unbound = TurningSetup::single(None, 3_000.0).expect("turning");
    let off = lathe_program(vec![ClBlock::Motion(ClMotion::Rapid { to: yz(10.0, 0.01, 1.0), feed: None })], unbound);
    let err = post_program(&off, &linux_lathe(), &options()).expect_err("y");
    assert!(err.to_string().contains("lathe pose y must be 0"), "{err}");

    let edge = lathe_program(vec![ClBlock::Motion(ClMotion::Rapid { to: yz(10.0, 1e-6, 1.0), feed: None })], turning(None, false));
    post_program(&edge, &linux_lathe(), &options()).expect("on the limit");

    let xy = lathe_program(
        vec![ClBlock::Motion(ClMotion::Arc {
            to: xz(0.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::new(0.0, 0.0, 0.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 40.0,
            role: MoveRole::Cut,
        })],
        turning(None, false),
    );
    // The arc has no prior position, but the plane is rejected before that.
    let err = post_program(&xy, &linux_lathe(), &options()).expect_err("plane");
    assert!(err.to_string().contains("zx plane"), "{err}");
}

#[test]
fn lathe_templates_reject_an_empty_plane_and_unknown_placeholders() {
    let mut raw = lathe_json();
    raw["lathe"]["plane"] = serde_json::json!("");
    let err = load_post(&raw.to_string()).expect_err("plane");
    assert!(err.to_string().contains("lathe.plane"), "{err}");

    raw = lathe_json();
    raw["lathe"]["css"] = serde_json::json!(["G96 S{nope}"]);
    let err = load_post(&raw.to_string()).expect_err("placeholder");
    assert!(err.to_string().contains("lathe.css"), "{err}");

    raw = lathe_json();
    raw["lathe"]["rpm"] = serde_json::json!(["G97 S{surface}"]);
    let err = load_post(&raw.to_string()).expect_err("surface on rpm");
    assert!(err.to_string().contains("lathe.rpm"), "{err}");

    raw = lathe_json();
    raw["lathe"]["extra"] = serde_json::json!(1);
    let err = load_post(&raw.to_string()).expect_err("unknown");
    assert!(err.to_string().contains("extra") || err.to_string().contains("unknown field"), "{err}");
}

fn turn_document() -> Document {
    let stock = TurnStock::bar(50.0, 60.0).expect("stock");
    let insert = Insert::try_new(InsertShape::C80, 0.8, None, Hand::Right, 45.0).expect("insert");
    let mounted = TurnInsert::try_new(1, "turret1", 1, insert, false).expect("mounted");
    let speed = SpindleSpeed::css(200.0, 2_500.0).expect("css");
    let bound = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(speed)).expect("speed");
    let turning =
        TurningSetup::single(Some(stock), 3_000.0).expect("setup").with_insert(mounted).expect("insert").with_binding(bound).expect("binding");
    let blocks = vec![
        spindle_on(None),
        rapid(26.0, 2.0),
        linear(-0.8, 0.0, 100.0),
        rapid(30.0, 2.0),
        rapid(20.0, 1.0),
        linear(20.0, -8.0, 80.0),
        linear(23.0, -11.0, 80.0),
        ClBlock::Motion(ClMotion::Arc {
            to: xz(26.0, -14.0),
            plane: Plane::Zx,
            center: Vec3::new(23.0, 0.0, -14.0).expect("convex centre"),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Arc {
            to: xz(28.0, -16.0),
            plane: Plane::Zx,
            center: Vec3::new(28.0, 0.0, -14.0).expect("concave centre"),
            sense: ArcSense::Cw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
        rapid(32.0, 5.0),
        spindle_off(),
    ];
    let tool = Tool::try_new(1, Some("cnmg".into()), None, None).expect("tool");
    let path = path(Some("finish"), Some(1), blocks);
    let program = Program::try_new(Some("turn".into()), vec![tool], None, vec![path], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(turning))
        .expect("turning");
    Document::new(program).expect("document")
}

fn pretty_document(doc: &Document) -> String {
    let mut text = doc.to_json_pretty().expect("pretty");
    if !text.ends_with('\n') {
        text.push('\n');
    }
    text
}

fn turn_ir_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/turn.ir.json")
}

#[test]
fn turn_fixture_posts_and_matches_the_helper() {
    let doc = turn_document();
    let path = turn_ir_path();
    let expected = pretty_document(&doc);
    if std::env::var("WRITE_TURN_IR").ok().as_deref() == Some("1") || !path.exists() {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).expect("dir");
        }
        std::fs::write(&path, &expected).expect("write fixture");
    }
    let on_disk = std::fs::read_to_string(&path).expect("read fixture");
    assert_eq!(on_disk, expected, "turn.ir.json drifted from the helper; regenerate with WRITE_TURN_IR=1");

    let post = linux_lathe();
    let text = post_program(doc.program(), &post, &options()).expect("post");
    assert!(text.contains("G96 D2500 S200"), "{text}");
    assert!(has_word(&text, "X52.0000"), "{text}");
    assert!(has_word(&text, "X-1.6000"), "{text}");
    assert!(has_word(&text, "G3"), "{text}");
    assert!(has_word(&text, "G2"), "{text}");
    assert!(has_word(&text, "I0.0000") && has_word(&text, "K-3.0000"), "{text}");
    assert!(has_word(&text, "I2.0000"), "{text}");
    assert!(!has_word(&text, "I4.0000"), "{text}");
    assert!(text.contains("G53 G0 X0 Z0"), "{text}");
    assert!(text.trim_end().ends_with("M2"), "{text}");
    let home = text.find("G53 G0 X0 Z0").expect("home");
    let end = text.rfind("M2").expect("end");
    assert!(home < end, "{text}");
    assert!(!text.split_whitespace().any(|word| word.starts_with('Y')), "{text}");
    compare(doc.program(), &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

fn css_tool_program() -> Program {
    let mut blocks = vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), spindle_on(None)];
    blocks.extend(quarter_blocks());
    blocks.push(spindle_off());
    with_tool(blocks, turning(Some(SpindleSpeed::css(200.0, 2_500.0).expect("css")), false))
}

fn sequence_numbers(text: &str) -> Vec<u32> {
    text.lines()
        .filter_map(|line| {
            let rest = line.strip_prefix('N')?;
            rest.split_whitespace().next()?.parse().ok()
        })
        .collect()
}

fn assert_step_ten(text: &str) {
    let numbers = sequence_numbers(text);
    assert_eq!(numbers.first().copied(), Some(10), "{text}");
    for pair in numbers.windows(2) {
        assert_eq!(pair[1] - pair[0], 10, "{text}");
    }
}

#[test]
fn fanuc_lathe_writes_g50_before_g96_and_the_t_word() {
    let program = css_tool_program();
    let post = builtin("fanuc_lathe").expect("fanuc_lathe");
    let text = post_program(&program, &post, &options()).expect("post");
    assert_eq!(post.name(), "Fanuc lathe");
    assert!(text.starts_with("%\nO0001\nN10 "), "{text}");
    assert!(text.trim_end().ends_with('%'), "{text}");
    assert!(!text.lines().any(|line| line.starts_with('%') && line.contains('N')), "{text}");
    assert!(text.lines().any(|line| line == "O0001"), "{text}");
    assert_step_ten(&text);
    assert!(has_word(&text, "G98"), "{text}");
    assert!(has_word(&text, "G18"), "{text}");
    assert!(has_word(&text, "G21"), "{text}");
    assert!(!has_word(&text, "G7"), "{text}");
    assert!(!has_word(&text, "G90"), "{text}");
    assert!(!has_word(&text, "G99"), "{text}");
    let g50 = text.find("G50 S2500").expect("g50");
    let g96 = text.find("G96 S200").expect("g96");
    let m3 = text.find("M3").expect("m3");
    assert!(g50 < g96 && g96 < m3, "{text}");
    assert!(has_word(&text, "T0101"), "{text}");
    assert!(has_word(&text, "X20."), "{text}");
    assert!(has_word(&text, "X10."), "{text}");
    assert!(has_word(&text, "I-5."), "{text}");
    assert!(has_word(&text, "K0."), "{text}");
    assert!(!has_word(&text, "I-10") && !has_word(&text, "I-10."), "{text}");
    assert!(has_word(&text, "G3"), "{text}");
    assert!(!words_of(&text).iter().any(|word| word.starts_with('R')), "{text}");
    assert!(text.contains("G28 U0 W0"), "{text}");
    let home = text.find("G28 U0 W0").expect("home");
    let end = text.rfind("M30").expect("m30");
    assert!(home < end, "{text}");
    let parked = read_back("G18\nG0 X20 Z1\nG28 U0 W0\n", &post).expect("home read");
    assert_eq!(parked.len(), 1, "{parked:?}");
    let err = read_back("G0 U1\n", &post).expect_err("incremental");
    assert!(err.to_string().contains("G28"), "{err}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn haas_lathe_writes_g98_and_g53_home() {
    let program = css_tool_program();
    let post = builtin("haas_lathe").expect("haas_lathe");
    let text = post_program(&program, &post, &options()).expect("post");
    assert_eq!(post.name(), "Haas lathe");
    assert!(text.starts_with("%\nO00001\nN10 "), "{text}");
    assert!(text.trim_end().ends_with('%'), "{text}");
    assert!(text.lines().any(|line| line == "O00001"), "{text}");
    assert_step_ten(&text);
    assert!(text.contains("G21 G18 G40 G80"), "{text}");
    assert!(has_word(&text, "G98"), "{text}");
    assert!(!has_word(&text, "G99"), "{text}");
    assert!(!has_word(&text, "G7"), "{text}");
    let g50 = text.find("G50 S2500").expect("g50");
    let g96 = text.find("G96 S200").expect("g96");
    assert!(g50 < g96, "{text}");
    assert!(has_word(&text, "T0101"), "{text}");
    assert!(has_word(&text, "I-5.") && has_word(&text, "K0."), "{text}");
    assert!(!words_of(&text).iter().any(|word| word.starts_with('R')), "{text}");
    let x_home = text.find("G53 G0 X0").expect("x home");
    let z_home = text.find("G53 G0 Z0").expect("z home");
    let end = text.rfind("M30").expect("m30");
    assert!(x_home < z_home && z_home < end, "{text}");
    let parked = read_back("G18\nG0 X20 Z1\nG53 G0 X0\nG53 G0 Z0\n", &post).expect("home read");
    assert_eq!(parked.len(), 1, "{parked:?}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

/// Whole Fanuc lathe axes keep a point. Feed and a millisecond dwell do not.
#[test]
fn fanuc_lathe_writes_a_decimal_point_on_axes_only() {
    let blocks = vec![
        rapid(26.0, 2.0),
        ClBlock::Event(Event::Dwell { seconds: 0.5 }),
        linear(26.0, 0.0, 100.0),
        ClBlock::Motion(ClMotion::Arc {
            to: xz(31.0, -5.0),
            plane: Plane::Zx,
            center: Vec3::new(26.0, 0.0, -5.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        }),
    ];
    let program = lathe_program(blocks, turning(None, false));
    let post = builtin("fanuc_lathe").expect("fanuc_lathe");
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(has_word(&text, "X52."), "{text}");
    assert!(has_word(&text, "Z0."), "{text}");
    assert!(has_word(&text, "I0."), "{text}");
    assert!(has_word(&text, "F100"), "{text}");
    assert!(has_word(&text, "P500"), "{text}");
    assert!(!has_word(&text, "X52"), "{text}");
    assert!(!has_word(&text, "Z0"), "{text}");
    assert!(!has_word(&text, "I0"), "{text}");
    assert!(!has_word(&text, "F100."), "{text}");
    assert!(!has_word(&text, "P500."), "{text}");
    assert!(has_word(&text, "K-5."), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let haas = builtin("haas_lathe").expect("haas_lathe");
    let haas_text = post_program(&program, &haas, &options()).expect("haas");
    assert!(has_word(&haas_text, "X52.") && has_word(&haas_text, "Z0.") && has_word(&haas_text, "I0."), "{haas_text}");
    assert!(has_word(&haas_text, "F100") && has_word(&haas_text, "P500"), "{haas_text}");
    assert!(!has_word(&haas_text, "F100.") && !has_word(&haas_text, "P500."), "{haas_text}");
    compare(&program, &read_back(&haas_text, &haas).expect("read"), 1e-3, &haas).expect("haas compare");
}

#[test]
fn grbl_lathe_writes_g7_and_a_manual_tool_change() {
    let program = css_tool_program();
    let post = builtin("grbl_lathe").expect("grbl_lathe");
    let text = post_program(&program, &post, &options()).expect("post");
    assert_eq!(post.name(), "grblHAL lathe");
    assert!(post.description().contains("manual tool change"), "{}", post.description());
    assert!(text.starts_with("(turn)\n"), "{text}");
    assert!(text.trim_end().ends_with("M30"), "{text}");
    assert!(text.lines().all(|line| !line.starts_with('N') && line.trim() != "%" && !line.starts_with('O')), "{text}");
    assert!(has_word(&text, "G7"), "{text}");
    assert!(has_word(&text, "G18"), "{text}");
    assert!(has_word(&text, "G94"), "{text}");
    assert!(has_word(&text, "G90"), "{text}");
    assert!(text.contains("G96 D2500 S200"), "{text}");
    assert!(text.contains("T1 M6"), "{text}");
    assert!(!text.contains("G28"), "{text}");
    assert!(!text.contains("G53"), "{text}");
    assert!(!has_word(&text, "G98"), "{text}");
    assert!(!has_word(&text, "G99"), "{text}");
    assert!(has_word(&text, "X20") && has_word(&text, "I-5") && has_word(&text, "K0"), "{text}");
    assert!(!words_of(&text).iter().any(|word| word.starts_with('R')), "{text}");
    let parked = read_back("G7\nG18\nG94\nG0 X20 Z1\n", &post).expect("mode read");
    assert_eq!(parked.len(), 1, "{parked:?}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}
