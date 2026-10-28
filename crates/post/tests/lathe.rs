//! Lathe posts: diameter X, radius I/K, G2/G3, spindle speed and the turning fixture.

#![allow(clippy::expect_used)]

use std::path::PathBuf;

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Document, Event, Feeds, Hand, Head, HeadAxis, Insert, InsertShape,
    LiveDirection, MoveRole, Plane, Pose, ProcessKind, ProcessParams, Program, Spindle, SpindleSpeed, Tool, TurnBinding, TurnInsert, TurnStock,
    TurningSetup, Vec3,
};
use campanion_post::{PostDef, PostOptions, ReadMove, builtin, channel_balance, compare, load_post, post_program, read_back};

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
    let fanuc = builtin("fanuc_lathe").expect("fanuc_lathe");
    let err = post_program(&off, &fanuc, &options()).expect_err("y");
    assert!(err.to_string().contains("post `fanuc_lathe` has no Y axis"), "{err}");

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
    campanion_testsupport::ir_json_close(&expected, &on_disk)
        .unwrap_or_else(|err| panic!("turn.ir.json drifted from the helper; regenerate with WRITE_TURN_IR=1\n{err}"));

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

fn thread_move(x: f64, z: f64, lead: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Thread { to: xz(x, z), lead })
}

fn rpm_turning() -> TurningSetup {
    turning(Some(SpindleSpeed::rpm(800.0).expect("rpm")), false)
}

#[test]
fn thread_writes_a_diameter_and_a_lead_and_resets_modal_motion() {
    let program = lathe_program(vec![rapid(11.0, 1.0), thread_move(9.0, -12.0, 2.5), rapid(11.0, 1.0)], rpm_turning());
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(has_word(&text, "G33"), "{text}");
    assert!(has_word(&text, "X22.0000"), "{text}");
    assert!(has_word(&text, "X18.0000"), "{text}");
    assert!(has_word(&text, "K2.5000"), "{text}");
    let lines: Vec<&str> = text.lines().collect();
    let thread_at = lines.iter().position(|line| line.contains("G33")).expect("g33 line");
    assert!(lines[thread_at + 1..].iter().any(|line| line.split_whitespace().any(|word| word == "G0")), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let fanuc = builtin("fanuc_lathe").expect("fanuc_lathe");
    let fanuc_program = lathe_program(vec![linear(11.0, 1.0, 2.5), thread_move(9.0, -12.0, 2.5), linear(8.0, -13.0, 2.5)], rpm_turning());
    let fanuc_text = post_program(&fanuc_program, &fanuc, &options()).expect("fanuc");
    assert!(fanuc_text.lines().any(|line| line.contains("G32") && line.split_whitespace().any(|word| word == "F2.5")), "{fanuc_text}");
    let g1 = fanuc_text.lines().skip_while(|line| !line.contains("G32")).skip(1).find(|line| line.contains("G1")).expect("feed after thread");
    assert!(g1.split_whitespace().any(|word| word == "F2.5"), "{fanuc_text}");
    compare(&fanuc_program, &read_back(&fanuc_text, &fanuc).expect("read"), 1e-3, &fanuc).expect("fanuc compare");
}

#[test]
fn thread_under_css_or_without_a_thread_post_errors_without_panic() {
    let blocks = vec![rapid(11.0, 1.0), thread_move(9.0, -12.0, 2.5)];
    let css = turning(Some(SpindleSpeed::css(120.0, 3_000.0).expect("css")), false);
    let css_err = post_program(&lathe_program(blocks.clone(), css), &linux_lathe(), &options()).expect_err("css");
    assert!(css_err.to_string().contains("threading needs a fixed rpm binding"), "{css_err}");

    let unset = post_program(&lathe_program(blocks.clone(), turning(None, false)), &linux_lathe(), &options()).expect_err("unset");
    assert!(unset.to_string().contains("threading needs a fixed rpm binding"), "{unset}");

    let mill = builtin("linuxcnc").expect("linuxcnc");
    let mill_err = post_program(&lathe_program(blocks.clone(), rpm_turning()), &mill, &options()).expect_err("mill");
    assert!(mill_err.to_string().contains("does not support threading"), "{mill_err}");

    let mut raw = lathe_json();
    raw["lathe"].as_object_mut().expect("lathe").remove("thread");
    let bare = post_from(raw);
    let bare_err = post_program(&lathe_program(blocks, rpm_turning()), &bare, &options()).expect_err("bare");
    assert!(bare_err.to_string().contains("does not support synchronised threading"), "{bare_err}");

    for lead in [-1.0, f64::NAN, 1_000.1] {
        let broken = ClPath::try_new(None, None, CompensationMode::Off, feeds(), None, Vec::new(), vec![thread_move(9.0, -1.0, lead)]);
        assert!(broken.is_err(), "lead {lead} should be rejected");
    }
    let edge = ClPath::try_new(None, None, CompensationMode::Off, feeds(), None, Vec::new(), vec![thread_move(9.0, -1.0, 1_000.0)]);
    assert!(edge.is_ok(), "lead 1000 is the limit");
}

fn haas_json() -> serde_json::Value {
    serde_json::from_str(include_str!("../posts/haas_lathe.json")).expect("json")
}

fn haas_lathe() -> PostDef {
    builtin("haas_lathe").expect("haas_lathe")
}

fn c_pose(x: f64, z: f64, c: f64) -> Pose {
    xz(x, z).with_rotary(c).expect("c")
}

fn rapid_c(x: f64, z: f64, c: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: c_pose(x, z, c), feed: None })
}

fn linear_c(x: f64, z: f64, c: f64, feed: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: c_pose(x, z, c), feed, role: MoveRole::Cut })
}

fn millturn_setup() -> TurningSetup {
    millturn_head(vec![HeadAxis::X, HeadAxis::Z])
}

fn millturn_y_setup() -> TurningSetup {
    millturn_head(vec![HeadAxis::X, HeadAxis::Y, HeadAxis::Z])
}

fn millturn_head(axes: Vec<HeadAxis>) -> TurningSetup {
    let spindle = Spindle::try_new("main", Some(0), 0.0, 3_000.0).expect("spindle");
    let head = Head::try_new("turret1", axes, 12, true).expect("head");
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding");
    TurningSetup::try_new(vec![spindle], vec![head], None, Vec::new(), vec![binding]).expect("turning")
}

fn radial_pose(x: f64, y: f64, z: f64, c: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::X).expect("pose").with_rotary(c).expect("c")
}

fn axial_pose(x: f64, y: f64, z: f64, c: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose").with_rotary(c).expect("c")
}

fn millturn_program(blocks: Vec<ClBlock>) -> Program {
    millturn_on(blocks, millturn_setup())
}

fn millturn_y_program(blocks: Vec<ClBlock>) -> Program {
    millturn_on(blocks, millturn_y_setup())
}

fn millturn_on(blocks: Vec<ClBlock>, setup: TurningSetup) -> Program {
    let tool = Tool::try_new(1, Some("live".into()), None, None).expect("tool");
    Program::try_new(Some("millturn".into()), vec![tool], None, vec![path(None, Some(1), blocks)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning")
}

fn mill_program(event: Event) -> Program {
    Program::try_new(Some("mill".into()), Vec::new(), None, vec![path(None, None, vec![ClBlock::Event(event)])], Vec::new(), Vec::new(), None)
        .expect("program")
}

fn assert_in_order(text: &str, needles: &[&str]) {
    let mut rest = text;
    for needle in needles {
        let found = rest.find(needle);
        assert!(found.is_some(), "{needle} missing in {rest}");
        let found = found.expect("present");
        rest = &rest[found + needle.len()..];
    }
}

#[test]
fn haas_millturn_writes_c_brake_and_live_tool_and_reads_back() {
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        linear_c(20.0, -5.0, 0.0, 100.0),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        rapid_c(20.0, 2.0, 90.0),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        linear_c(20.0, -5.0, 90.0, 100.0),
        rapid_c(20.0, 2.0, 90.0),
        ClBlock::Event(Event::LiveToolOff),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_program(blocks);
    let post = haas_lathe();
    assert!(post.description().contains("cited in `references`"), "{}", post.description());
    let text = post_program(&program, &post, &options()).expect("post");
    let golden = include_str!("golden/millturn.haas_lathe.nc");
    assert_eq!(text, golden);
    assert_in_order(&text, &["M154", "G28 H0.", "C0.", "P1500 M133", "M14", "M15", "C90.", "M14", "M135", "M15", "M155"]);
    assert_eq!(text.matches("G98").count(), 1, "{text}");
    assert!(!text.contains("haascnc.com"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn haas_y_axis_writes_g19_y_and_restores_g18() {
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Rapid { to: radial_pose(20.0, 0.0, 2.0, 90.0), feed: None }),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(20.0, -5.0, 2.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(18.0, -5.0, 2.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(18.0, 5.0, -4.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::LiveToolOff),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_y_program(blocks);
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let golden = include_str!("golden/millturn_y.haas_lathe.nc");
    assert_eq!(text, golden);
    let side = text.find("G19").expect("g19");
    let y_word = text.find("Y-5.").expect("y");
    let restored = text.rfind("G18").expect("g18");
    assert!(side < y_word && y_word < restored, "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn haas_axial_xy_arc_writes_g17_and_reads_back() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Rapid { to: axial_pose(10.0, 0.0, -5.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: axial_pose(0.0, 10.0, -5.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::new(0.0, 0.0, -5.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        }),
        // X changes after G17. Diameter mode still doubles it; face interpolation would not.
        ClBlock::Motion(ClMotion::Linear { to: axial_pose(8.0, 10.0, -5.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_y_program(blocks);
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(text.contains("G17"), "{text}");
    assert!(text.contains("Y10."), "{text}");
    assert!(text.contains("G18"), "{text}");
    let g17 = text.find("G17").expect("g17");
    let g18 = text.rfind("G18").expect("g18");
    assert!(g17 < text.find("Y10.").expect("y") && text.find("Y10.").expect("y") < g18, "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn c_and_y_cannot_change_in_one_move() {
    let line = millturn_y_program(vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Rapid { to: radial_pose(20.0, 0.0, 0.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(20.0, 4.0, 0.0, 45.0), feed: 80.0, role: MoveRole::Cut }),
    ]);
    let err = post_program(&line, &haas_lathe(), &options()).expect_err("line");
    assert!(err.to_string().contains("C and Y in one move"), "{err}");

    let arc = millturn_y_program(vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Rapid { to: axial_pose(10.0, 0.0, -5.0, 0.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: axial_pose(0.0, 10.0, -5.0, 30.0),
            plane: Plane::Xy,
            center: Vec3::new(0.0, 0.0, -5.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
    ]);
    let err = post_program(&arc, &haas_lathe(), &options()).expect_err("arc");
    assert!(err.to_string().contains("C and Y in one move"), "{err}");
}

#[test]
fn fanuc_lathe_has_no_y_axis() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(20.0, 3.0, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
    ];
    let err = post_program(&millturn_y_program(blocks), &builtin("fanuc_lathe").expect("fanuc"), &options()).expect_err("y");
    assert!(err.to_string().contains("post `fanuc_lathe` has no Y axis"), "{err}");
}

#[test]
fn haas_non_moves_keep_modal_c() {
    let post = haas_lathe();
    let parked = read_back("M154\nG28 H0.\nP1500 M133\nM14\nM15\nM135\nM155\n", &post).expect("read");
    assert_eq!(parked.len(), 6, "{parked:?}");
    assert!(parked.iter().all(|move_| matches!(move_, ReadMove::Other { .. })), "{parked:?}");
    let held = read_back("G0 X40. Z2. C90.\nG28 H0.\nG1 X40. Z-5. F100\n", &post).expect("held");
    let ReadMove::Linear { rotary, .. } = &held[1] else {
        panic!("expected a feed, got {held:?}");
    };
    assert_eq!(*rotary, Some(90.0));
}

#[test]
fn millturn_rejects_c_without_engage_spindle_while_engaged_and_missing_blocks() {
    let haas = haas_lathe();
    let bare_c = post_program(&millturn_program(vec![rapid_c(20.0, 2.0, 0.0)]), &haas, &options()).expect_err("c");
    assert!(bare_c.to_string().contains("C angle needs the C axis engaged"), "{bare_c}");

    let spinning = millturn_program(vec![ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }), spindle_on(Some(1_000.0))]);
    let spin_err = post_program(&spinning, &haas, &options()).expect_err("spin");
    assert!(spin_err.to_string().contains("turning spindle start while the C axis is engaged"), "{spin_err}");

    let fanuc = builtin("fanuc_lathe").expect("fanuc_lathe");
    let live = lathe_program(
        vec![rapid(10.0, 1.0), ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward })],
        turning(None, false),
    );
    let live_err = post_program(&live, &fanuc, &options()).expect_err("live");
    assert!(live_err.to_string().contains("has no live tooling"), "{live_err}");

    let brake = millturn_program(vec![ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true })]);
    let brake_err = post_program(&brake, &fanuc, &options()).expect_err("brake");
    assert!(brake_err.to_string().contains("has no spindle brake"), "{brake_err}");

    let foreign = post_program(&millturn_program(vec![rapid_c(10.0, 1.0, 15.0)]), &fanuc, &options()).expect_err("no c");
    assert!(foreign.to_string().contains("has no C axis"), "{foreign}");

    let mill = builtin("linuxcnc").expect("linuxcnc");
    let events = [
        Event::CAxis { spindle: "main".into(), engage: true },
        Event::SpindleBrake { spindle: "main".into(), clamp: false },
        Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Reverse },
        Event::LiveToolOff,
        Event::FaceInterpolation { on: true },
        Event::CylinderInterpolation { radius: Some(20.0) },
    ];
    for event in events {
        let err = post_program(&mill_program(event), &mill, &options()).expect_err("mill");
        assert!(err.to_string().contains("does not support mill-turn spindle modes"), "{err}");
    }
}

#[test]
fn millturn_rejects_c_move_while_brake_clamped() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        rapid_c(20.0, 2.0, 90.0),
    ];
    let err = post_program(&millturn_program(blocks), &haas_lathe(), &options()).expect_err("clamped");
    assert!(err.to_string().contains("C move with the spindle brake clamped"), "{err}");
}

#[test]
fn millturn_stops_the_live_tool_and_disengages_c_before_home() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::LiveToolOn { rpm: 1_200.0, direction: LiveDirection::Reverse }),
        linear_c(20.0, -5.0, 0.0, 80.0),
    ];
    let program = millturn_program(blocks);
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(text.contains("P1200 M134"), "{text}");
    let m135 = text.find("M135").expect("stop");
    let m155 = text.find("M155").expect("disengage");
    let home = text.find("G53").expect("home");
    assert!(m135 < m155 && m155 < home, "{text}");
    assert_eq!(text.matches("M135").count(), 1, "{text}");
    assert_eq!(text.matches("M155").count(), 1, "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn engaged_arc_stays_in_zx_without_a_c_word() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(10.0, 0.0, 0.0),
        ClBlock::Motion(ClMotion::Arc {
            to: c_pose(5.0, -5.0, 0.0),
            plane: Plane::Zx,
            center: Vec3::new(5.0, 0.0, 0.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
    ];
    let program = millturn_program(blocks);
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let arc = text.lines().find(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")).expect("arc");
    assert!(!arc.split_whitespace().any(|word| word.starts_with('C')), "{text}");
    assert!(arc.split_whitespace().any(|word| word.starts_with('I')), "{arc}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn disengage_clears_modal_c_and_a_different_feed_mode_is_written_once() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
    ];
    let text = post_program(&millturn_program(blocks), &haas_lathe(), &options()).expect("post");
    assert_eq!(text.lines().filter(|line| line.contains("C0.")).count(), 2, "{text}");

    let mut raw = haas_json();
    raw["lathe"]["live_tool"]["feed_mode"] = serde_json::json!("G99");
    let post = post_from(raw);
    let live = vec![
        ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }),
        ClBlock::Event(Event::LiveToolOn { rpm: 800.5, direction: LiveDirection::Reverse }),
    ];
    let posted = post_program(&millturn_program(live), &post, &options()).expect("feed");
    assert_eq!(posted.matches("G99").count(), 1, "{posted}");
    assert!(posted.contains("P1500 M133"), "{posted}");
    assert!(posted.contains("P801 M134"), "{posted}");
    let g99 = posted.find("G99").expect("g99");
    let forward = posted.find("P1500 M133").expect("forward");
    let reverse = posted.find("P801 M134").expect("reverse");
    assert!(g99 < forward && forward < reverse, "{posted}");
}

#[test]
fn post_rejects_a_bad_c_axis_letter_and_references_that_are_too_long() {
    for letter in ["c", "X", "CC", ""] {
        let mut raw = haas_json();
        raw["lathe"]["c_axis"]["letter"] = serde_json::json!(letter);
        let err = load_post(&raw.to_string()).expect_err(letter);
        let text = err.to_string();
        assert!(text.contains("lathe.c_axis.letter"), "{letter}: {text}");
        assert!(text.contains("must be one uppercase ASCII letter and not X, Y, Z, I, J, K, F, S, T, M, G, N, O, P"), "{letter}: {text}");
    }
    let mut raw = haas_json();
    raw["references"] = serde_json::json!(["x".repeat(257)]);
    let err = load_post(&raw.to_string()).expect_err("long");
    let text = err.to_string();
    assert!(text.contains("references") && text.contains("1..=256"), "{text}");

    let mut raw = haas_json();
    raw["references"] = serde_json::json!(vec!["cited".to_string(); 33]);
    let err = load_post(&raw.to_string()).expect_err("count");
    assert!(err.to_string().contains("at most 32"), "{err}");
}

const FACE_Z: f64 = -1.0;

/// 20×20 square with 2 mm corner arcs on the face, centred at (12, 0).
///
/// A centre at (10, 0) would put the left side on x = 0 and cross the spindle axis.
/// Bounds are x = 2..22, y = -10..10. The path starts at (4, -10) and walks counter-clockwise.
fn face_square() -> Vec<ClBlock> {
    let line = |x, y| ClBlock::Motion(ClMotion::Linear { to: yz(x, y, FACE_Z), feed: 100.0, role: MoveRole::Cut });
    let arc = |x, y, cx, cy| {
        ClBlock::Motion(ClMotion::Arc {
            to: yz(x, y, FACE_Z),
            plane: Plane::Xy,
            center: Vec3::new(cx, cy, FACE_Z).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        })
    };
    vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Event(Event::FaceInterpolation { on: true }),
        ClBlock::Motion(ClMotion::Rapid { to: yz(4.0, -10.0, FACE_Z), feed: None }),
        line(20.0, -10.0),
        arc(22.0, -8.0, 20.0, -8.0),
        line(22.0, 8.0),
        arc(20.0, 10.0, 20.0, 8.0),
        line(4.0, 10.0),
        arc(2.0, 8.0, 4.0, 8.0),
        line(2.0, -8.0),
        arc(4.0, -10.0, 4.0, -8.0),
        ClBlock::Event(Event::FaceInterpolation { on: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ]
}

fn section_between<'a>(text: &'a str, start: &str, end: &str) -> &'a str {
    let from = text.find(start).unwrap_or(text.len());
    let rest = &text[from + start.len()..];
    let to = rest.find(end).unwrap_or(rest.len());
    &rest[..to]
}

fn c_angles(text: &str) -> Vec<f64> {
    text.split_whitespace()
        .filter_map(|word| {
            let rest = word.strip_prefix('C')?;
            rest.chars().next().filter(|ch| ch.is_ascii_digit() || *ch == '-' || *ch == '.')?;
            rest.parse().ok()
        })
        .collect()
}

fn load_temp_post(value: &serde_json::Value, name: &str) -> PostDef {
    let path = std::env::temp_dir().join(format!("campanion-{name}-{}.json", std::process::id()));
    std::fs::write(&path, value.to_string()).expect("write temp post");
    let text = std::fs::read_to_string(&path).expect("read temp post");
    let loaded = load_post(&text);
    std::fs::remove_file(&path).expect("remove temp post");
    loaded.expect("load temp post")
}

fn haas_without_face() -> PostDef {
    let mut raw = haas_json();
    raw["lathe"].as_object_mut().expect("lathe").remove("face_interp");
    load_temp_post(&raw, "haas-lathe-face-fallback")
}

#[test]
fn haas_face_interpolation_writes_xy_arcs_and_reads_back() {
    let program = millturn_program(face_square());
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let golden = include_str!("golden/millturn_face.haas_lathe.nc");
    assert_eq!(text, golden);
    let body = section_between(&text, "G112", "G113");
    assert!(body.contains("G17") || text.contains("G17"), "{text}");
    assert!(body.split_whitespace().any(|word| word == "G2" || word == "G3"), "{body}");
    assert!(text.contains("G112") && text.contains("G113"), "{text}");
    let c_words: Vec<_> = body
        .split_whitespace()
        .filter(|word| word.strip_prefix('C').is_some_and(|rest| rest.starts_with(|ch: char| ch.is_ascii_digit() || ch == '-' || ch == '.')))
        .collect();
    assert!(c_words.is_empty(), "C word inside G112..G113: {c_words:?}\n{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let kept = read_back("G112\nG1 X4. Y-10. Z-1. F100\n", &post).expect("face frame");
    let ReadMove::Linear { end, .. } = &kept[0] else { panic!("expected a feed, got {kept:?}") };
    assert!((end[0] - 4.0).abs() < 1e-9 && (end[1] + 10.0).abs() < 1e-9, "{end:?}");
}

#[test]
fn haas_cylinder_interpolation_writes_g107_and_reads_back() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Event(Event::CylinderInterpolation { radius: Some(20.0) }),
        ClBlock::Motion(ClMotion::Rapid { to: yz(20.0, -5.0, -10.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: yz(20.0, 5.0, -10.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: yz(20.0, 5.0, -20.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::CylinderInterpolation { radius: None }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_program(blocks);
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let golden = include_str!("golden/millturn_wrap.haas_lathe.nc");
    assert_eq!(text, golden);
    assert!(text.contains("G107 Y0 C0 R20."), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let held = read_back("G0 X10. Z0. C45.\nG107 Y0 C0 R20.\nG1 X40. Y5. Z-10. F100\n", &post).expect("wrap");
    let ReadMove::Linear { end, rotary, .. } = &held[1] else { panic!("expected a feed, got {held:?}") };
    assert_eq!(*rotary, Some(45.0));
    assert!((end[0] - 20.0).abs() < 1e-9 && (end[1] - 5.0).abs() < 1e-9, "{end:?}");
}

#[test]
fn face_fallback_writes_continuous_c_and_a_centre_crossing_errors() {
    let program = millturn_program(face_square());
    let post = haas_without_face();
    let text = post_program(&program, &post, &options()).expect("fallback");
    assert!(!text.contains("G112") && !text.contains("G113"), "{text}");
    assert!(!text.contains("G93"), "{text}");
    assert!(!text.split_whitespace().any(|word| word == "G2" || word == "G3"), "{text}");
    assert!(text.split_whitespace().all(|word| !word.starts_with('Y')), "{text}");
    let angles = c_angles(&text);
    assert!(angles.len() >= 2, "{text}");
    for pair in angles.windows(2) {
        let delta = (pair[1] - pair[0]).abs();
        assert!(delta <= 180.0 + 1e-3, "C jumped {delta} in {angles:?}");
    }
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let crossing = millturn_program(vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Event(Event::FaceInterpolation { on: true }),
        ClBlock::Motion(ClMotion::Rapid { to: yz(5.0, 0.0, FACE_Z), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: yz(-5.0, 0.0, FACE_Z), feed: 80.0, role: MoveRole::Cut }),
    ]);
    let message = "face path crosses the spindle centre; split it or use a post with face interpolation";
    let native = post_program(&crossing, &haas_lathe(), &options()).expect_err("native centre");
    assert!(native.to_string().contains(message), "{native}");
    let polar = post_program(&crossing, &post, &options()).expect_err("fallback centre");
    assert!(polar.to_string().contains(message), "{polar}");
}

#[test]
fn interpolation_rejects_a_disengaged_c_axis_a_y_move_and_an_xy_arc() {
    let haas = haas_lathe();
    let bare = post_program(&millturn_program(vec![ClBlock::Event(Event::FaceInterpolation { on: true })]), &haas, &options()).expect_err("face");
    assert!(bare.to_string().contains("face interpolation needs the C axis engaged"), "{bare}");

    let fanuc = builtin("fanuc_lathe").expect("fanuc_lathe");
    let no_axis = post_program(&millturn_program(vec![ClBlock::Event(Event::FaceInterpolation { on: true })]), &fanuc, &options()).expect_err("no c");
    let no_axis_text = no_axis.to_string();
    assert!(no_axis_text.contains("post `fanuc_lathe` has no C axis"), "{no_axis_text}");
    assert!(!no_axis_text.contains("needs the C axis engaged"), "{no_axis_text}");

    let good = path(None, None, vec![ClBlock::Motion(ClMotion::Rapid { to: xz(10.0, 0.0), feed: None })]);
    let off_plane = path(None, None, vec![ClBlock::Motion(ClMotion::Rapid { to: yz(10.0, 2.0, 0.0), feed: None })]);
    let drifted = Program::try_new(Some("millturn".into()), Vec::new(), None, vec![good, off_plane], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(millturn_setup()))
        .expect("turning");
    let y_err = post_program(&drifted, &haas, &options()).expect_err("y");
    assert!(y_err.to_string().contains("Y move needs the C axis engaged"), "{y_err}");

    let arc = millturn_program(vec![
        ClBlock::Motion(ClMotion::Rapid { to: xz(10.0, -1.0), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: yz(12.0, 0.0, -1.0),
            plane: Plane::Xy,
            center: Vec3::new(11.0, 0.0, -1.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 50.0,
            role: MoveRole::Cut,
        }),
    ]);
    let arc_err = post_program(&arc, &haas, &options()).expect_err("xy");
    assert!(arc_err.to_string().contains("lathe arc must be in the zx plane"), "{arc_err}");
}

fn assert_post_golden(name: &str, text: &str) {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        std::fs::write(&path, text).expect("write golden");
        return;
    }
    let message = format!("read {}", path.display());
    let expected = std::fs::read_to_string(&path).expect(&message);
    assert_eq!(text, expected, "golden {name}");
}

fn assert_no_native_interp(text: &str) {
    assert!(!text.contains("G112") && !text.contains("G107"), "{text}");
}

#[test]
fn linuxcnc_millturn_writes_spindle_select_and_reads_back() {
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        linear_c(20.0, -5.0, 0.0, 100.0),
        rapid_c(20.0, 2.0, 0.0),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        rapid_c(20.0, 2.0, 90.0),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        linear_c(20.0, -5.0, 90.0, 100.0),
        rapid_c(20.0, 2.0, 90.0),
        ClBlock::Event(Event::LiveToolOff),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_program(blocks);
    let post = linux_lathe();
    assert!(post.description().contains("custom post can fill them"), "{}", post.description());
    let text = post_program(&program, &post, &options()).expect("post");
    assert_post_golden("millturn.linuxcnc_lathe.nc", &text);
    assert!(text.contains("S1500 $1"), "{text}");
    assert!(text.contains("M3 $1") && text.contains("M5 $1"), "{text}");
    assert!(!text.contains("G93"), "{text}");
    assert!(!text.contains("M154") && !text.contains("M155"), "{text}");
    assert_no_native_interp(&text);
    let parked = read_back("S1500 $1\nM3 $1\nM5 $1\n", &post).expect("spindle select");
    assert_eq!(parked.len(), 2, "{parked:?}");
    assert!(parked.iter().all(|move_| matches!(move_, ReadMove::Other { .. })), "{parked:?}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn linuxcnc_face_fallback_writes_inverse_time_and_reads_back() {
    let program = millturn_program(face_square());
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert_post_golden("millturn_face.linuxcnc_lathe.nc", &text);
    assert!(text.contains("G93"), "{text}");
    assert_no_native_interp(&text);
    assert!(!text.split_whitespace().any(|word| word == "G2" || word == "G3"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn linuxcnc_cylinder_fallback_writes_inverse_time_and_reads_back() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Event(Event::CylinderInterpolation { radius: Some(20.0) }),
        ClBlock::Motion(ClMotion::Rapid { to: yz(20.0, -5.0, -10.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: yz(20.0, 5.0, -10.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: yz(20.0, 5.0, -20.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::CylinderInterpolation { radius: None }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_program(blocks);
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert_post_golden("millturn_wrap.linuxcnc_lathe.nc", &text);
    assert!(text.contains("G93"), "{text}");
    let g93 = text.find("G93").expect("g93");
    assert!(text[g93..].contains("G94"), "{text}");
    assert_no_native_interp(&text);
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn linuxcnc_y_axis_writes_g19_and_live_spindle() {
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        ClBlock::Motion(ClMotion::Rapid { to: radial_pose(20.0, 0.0, 2.0, 90.0), feed: None }),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
        ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(20.0, -5.0, 2.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(18.0, -5.0, 2.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Linear { to: radial_pose(18.0, 5.0, -4.0, 90.0), feed: 100.0, role: MoveRole::Cut }),
        ClBlock::Event(Event::LiveToolOff),
        ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }),
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }),
    ];
    let program = millturn_y_program(blocks);
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert_post_golden("millturn_y.linuxcnc_lathe.nc", &text);
    assert!(text.contains("M3 $1") && text.contains("M5 $1"), "{text}");
    assert!(text.contains("G19"), "{text}");
    assert!(!text.contains("G93"), "{text}");
    assert_no_native_interp(&text);
    let side = text.find("G19").expect("g19");
    let y_word = text.find("Y-5.").expect("y");
    let restored = text.rfind("G18").expect("g18");
    assert!(side < y_word && y_word < restored, "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn linuxcnc_indexed_c_move_uses_inverse_time_then_returns_to_per_minute() {
    let blocks = vec![
        ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
        rapid_c(20.0, 2.0, 0.0),
        linear_c(20.0, 2.0, 90.0, 100.0),
        linear_c(20.0, -5.0, 90.0, 100.0),
    ];
    let program = millturn_program(blocks);
    let post = linux_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(text.contains("G93"), "{text}");
    assert!(text.contains("F3.5355"), "{text}");
    let g93 = text.find("G93").expect("g93");
    let after = &text[g93..];
    let g94 = after.find("G94").expect("return to G94");
    assert!(after[g94..].contains("F100.0000"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn live_tool_lines_cannot_be_empty_and_c_feed_is_named() {
    let mut raw = lathe_json();
    raw["lathe"]["live_tool"]["forward"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty forward");
    assert!(err.to_string().contains("lathe.live_tool.forward") && err.to_string().contains("needs at least one line"), "{err}");

    raw = lathe_json();
    raw["lathe"]["live_tool"]["stop"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty stop");
    assert!(err.to_string().contains("lathe.live_tool.stop"), "{err}");

    raw = lathe_json();
    raw["lathe"]["c_feed"] = serde_json::json!("rpm");
    let err = load_post(&raw.to_string()).expect_err("c feed");
    assert!(err.to_string().contains("lathe.c_feed") && err.to_string().contains("must be per_minute or inverse_time"), "{err}");

    raw = lathe_json();
    raw["lathe"]["c_axis"]["engage"] = serde_json::json!([]);
    raw["lathe"]["brake"]["clamp"] = serde_json::json!([]);
    let post = post_from(raw);
    let text = post_program(
        &millturn_program(vec![
            ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
            ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
            rapid_c(20.0, 2.0, 0.0),
        ]),
        &post,
        &options(),
    )
    .expect("empty lists");
    assert!(text.contains("C0.0000"), "{text}");
    assert!(!text.contains("M154") && !text.contains("M14"), "{text}");
}

fn index_of(text: &str, needle: &str) -> usize {
    let found = text.find(needle);
    assert!(found.is_some(), "missing {needle}\n{text}");
    found.expect("present")
}

/// Byte offset of the first line whose words include `word` exactly.
/// `M11` must not match inside `M111` or `M110`.
fn index_of_word(text: &str, word: &str) -> usize {
    let mut offset = 0;
    let mut found = None;
    for line in text.split_inclusive('\n') {
        if line.split_whitespace().any(|token| token == word) {
            found = Some(offset);
            break;
        }
        offset += line.len();
    }
    assert!(found.is_some(), "missing word {word}\n{text}");
    found.expect("present")
}

fn tool_row(id: u32) -> Tool {
    Tool::try_new(id, None, None, None).expect("tool")
}

fn spindle_named(name: &str, c_axis: Option<u8>, max_rpm: f64) -> Spindle {
    Spindle::try_new(name, c_axis, 0.0, max_rpm).expect("spindle")
}

fn turret() -> Head {
    Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("head")
}

fn bound(index: u32, spindle: &str) -> TurnBinding {
    TurnBinding::try_new(index, spindle, "turret1", 0).expect("binding").with_speed(Some(SpindleSpeed::rpm(800.0).expect("rpm"))).expect("speed")
}

fn program_on(spindles: Vec<Spindle>, bindings: Vec<TurnBinding>, paths: Vec<ClPath>) -> Program {
    let tools: Vec<Tool> = paths.iter().filter_map(|path| path.tool_id()).map(tool_row).collect();
    let setup = TurningSetup::try_new(spindles, vec![turret()], None, Vec::new(), bindings).expect("turning");
    Program::try_new(Some("turn".into()), tools, None, paths, Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning")
}

/// Main op, then a sub op. The sub block starts with G55 G14 before its tool change, and G15 G54
/// precedes the home lines.
#[test]
fn haas_selects_the_sub_before_its_tool_change_and_deselects_before_home() {
    let main = vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), spindle_on(None), rapid(12.0, 1.0), linear(12.0, 0.0, 100.0), spindle_off()];
    let sub = vec![ClBlock::Event(Event::ToolChange { tool_id: 2 }), spindle_on(None), rapid(12.0, 0.5), linear(12.0, -5.0, 100.0), spindle_off()];
    let program = program_on(
        vec![spindle_named("main", None, 3_000.0), spindle_named("sub", None, 2_500.0)],
        vec![bound(0, "main"), bound(1, "sub")],
        vec![path(Some("main_face"), Some(1), main), path(Some("sub_face"), Some(2), sub)],
    );
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let t1 = index_of(&text, "T0101");
    let off = index_of(&text, "M5");
    let g55 = index_of(&text, "G55");
    let g14 = index_of(&text, "G14");
    let t2 = index_of(&text, "T0202");
    let g15 = index_of(&text, "G15");
    let g54 = index_of(&text, "G54");
    let home = index_of(&text, "G53");
    let end = index_of(&text, "M30");
    assert!(t1 < off && off < g55 && g55 < g14 && g14 < t2, "{text}");
    assert!(t2 < g15 && g15 < g54 && g54 < home && home < end, "{text}");
    assert!(has_word(&text, "Z-5."), "{text}");
    assert!(!has_word(&text, "Z5."), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

/// The spindle is still on when the binding changes, so M5 is written before G55.
#[test]
fn haas_stops_the_running_spindle_before_selecting_the_sub() {
    let main = vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), spindle_on(None), rapid(12.0, 1.0)];
    let sub = vec![ClBlock::Event(Event::ToolChange { tool_id: 2 }), rapid(12.0, -5.0)];
    let program = program_on(
        vec![spindle_named("main", None, 3_000.0), spindle_named("sub", None, 2_500.0)],
        vec![bound(0, "main"), bound(1, "sub")],
        vec![path(Some("main_face"), Some(1), main), path(Some("sub_face"), Some(2), sub)],
    );
    let text = post_program(&program, &haas_lathe(), &options()).expect("post");
    let m3 = index_of(&text, "M3");
    let m5 = index_of(&text, "M5");
    let g55 = index_of(&text, "G55");
    let g14 = index_of(&text, "G14");
    let t2 = index_of(&text, "T0202");
    assert!(m3 < m5 && m5 < g55 && g55 < g14 && g14 < t2, "{text}");
}

#[test]
fn haas_rejects_a_spindle_it_does_not_serve() {
    let program =
        program_on(vec![spindle_named("tail", None, 1_000.0)], vec![bound(0, "tail")], vec![path(Some("face"), Some(1), vec![rapid(10.0, 0.0)])]);
    let err = post_program(&program, &haas_lathe(), &options()).expect_err("tail");
    assert!(err.to_string().contains("post `haas_lathe` has no spindle `tail`"), "{err}");
}

#[test]
fn haas_rejects_c_axis_and_live_tooling_on_the_sub() {
    let events = [
        Event::CAxis { spindle: "sub".into(), engage: true },
        Event::SpindleBrake { spindle: "sub".into(), clamp: true },
        Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward },
        Event::FaceInterpolation { on: true },
        Event::CylinderInterpolation { radius: Some(20.0) },
    ];
    for event in events {
        let blocks = vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), ClBlock::Event(event)];
        let program = program_on(
            vec![spindle_named("main", None, 3_000.0), spindle_named("sub", Some(0), 2_500.0)],
            vec![bound(0, "sub")],
            vec![path(Some("sub_face"), Some(1), blocks)],
        );
        let err = post_program(&program, &haas_lathe(), &options()).expect_err("sub cycle");
        assert!(err.to_string().contains("post `haas_lathe`: live tooling and C axis on spindle `sub` are not supported"), "{err}");
    }
}

#[test]
fn mirror_z_negates_z_and_flips_g2_g3() {
    let mut raw = haas_json();
    raw["lathe"]["sub_spindle"]["mirror_z"] = serde_json::json!(true);
    let post = post_from(raw);
    let blocks = vec![
        rapid(10.0, 0.0),
        ClBlock::Motion(ClMotion::Arc {
            to: xz(5.0, -5.0),
            plane: Plane::Zx,
            center: Vec3::new(10.0, 0.0, -5.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 80.0,
            role: MoveRole::Cut,
        }),
    ];
    let program = program_on(
        vec![spindle_named("main", None, 3_000.0), spindle_named("sub", None, 2_500.0)],
        vec![bound(0, "sub")],
        vec![path(Some("sub_arc"), Some(1), blocks)],
    );
    let text = post_program(&program, &post, &options()).expect("post");
    let arc = text.lines().find(|line| line.split_whitespace().any(|word| word == "G2" || word == "G3")).expect("arc");
    assert!(arc.split_whitespace().any(|word| word == "G2"), "{arc}");
    assert!(!arc.split_whitespace().any(|word| word == "G3"), "{arc}");
    assert!(arc.split_whitespace().any(|word| word == "Z5."), "{arc}");
    assert!(arc.split_whitespace().any(|word| word == "K5."), "{arc}");
    assert!(arc.split_whitespace().any(|word| word == "I0."), "{arc}");
    assert!(!has_word(&text, "Z-5."), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

fn sync_block(id: u32) -> ClBlock {
    ClBlock::Event(Event::Sync { id })
}

fn multi_program(paths: Vec<(u16, Vec<ClBlock>)>) -> Program {
    let spindle = Spindle::try_new("main", None, 0.0, 3_000.0).expect("spindle");
    let turret = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("turret");
    let lower = Head::try_new("lower", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("lower");
    let mut toolpaths = Vec::new();
    let mut bindings = Vec::new();
    for (index, (channel, blocks)) in paths.into_iter().enumerate() {
        let head = if channel == 0 { "turret1" } else { "lower" };
        let index = u32::try_from(index).expect("index");
        bindings.push(TurnBinding::try_new(index, "main", head, channel).expect("binding"));
        toolpaths.push(path(Some("op"), None, blocks));
    }
    let turning = TurningSetup::try_new(vec![spindle], vec![turret, lower], None, Vec::new(), bindings).expect("turning");
    Program::try_new(Some("turn".into()), Vec::new(), None, toolpaths, Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(turning))
        .expect("turning")
}

fn two_channel() -> PostDef {
    builtin("generic_lathe_2ch").expect("generic_lathe_2ch")
}

#[test]
fn channels_count_below_two_is_rejected() {
    let mut raw = serde_json::from_str::<serde_json::Value>(include_str!("../posts/generic_lathe_2ch.json")).expect("json");
    raw["lathe"]["channels"]["count"] = serde_json::json!(1);
    let err = load_post(&raw.to_string()).expect_err("count");
    assert!(err.to_string().contains("channels.count must be at least 2"), "{err}");
    assert!(two_channel().description().contains("template"), "description should say this post is a template");
    assert!(two_channel().description().contains("check the control manual"));
}

#[test]
fn haas_rejects_a_second_channel() {
    let program = multi_program(vec![(1, vec![rapid(5.0, 0.0)])]);
    let err = post_program(&program, &haas_lathe(), &options()).expect_err("channel");
    assert!(err.to_string().contains("post `haas_lathe` is single-channel; the program uses channel 1"), "{err}");
}

#[test]
fn two_channel_post_rejects_a_missing_channel() {
    let program = multi_program(vec![(2, vec![rapid(5.0, 0.0)])]);
    let err = post_program(&program, &two_channel(), &options()).expect_err("channel");
    assert!(err.to_string().contains("post `generic_lathe_2ch` has no channel 2"), "{err}");
}

#[test]
fn channel_sync_ids_must_match() {
    let program = multi_program(vec![(0, vec![rapid(10.0, 0.0), sync_block(1), sync_block(2)]), (1, vec![rapid(4.0, 0.0), sync_block(1)])]);
    let err = post_program(&program, &two_channel(), &options()).expect_err("sync");
    assert!(err.to_string().contains("channel sync mismatch: channel 0 waits for [1, 2], channel 1 for [1]"), "{err}");
}

#[test]
fn two_channel_layout_writes_one_header_and_wait_codes() {
    let program = multi_program(vec![
        (0, vec![rapid(10.0, 0.0), linear(10.0, -5.0, 100.0), sync_block(1), linear(8.0, -5.0, 100.0)]),
        (1, vec![rapid(4.0, 0.0), linear(4.0, -2.0, 80.0), sync_block(1), linear(2.0, -2.0, 80.0)]),
    ]);
    let post = two_channel();
    let text = post_program(&program, &post, &options()).expect("post");
    assert_eq!(text.matches("O0001").count(), 1, "{text}");
    assert_eq!(text.matches("G18").count(), 1, "{text}");
    assert_eq!(text.matches("M30").count(), 1, "{text}");
    assert_eq!(text.matches("M101").count(), 2, "{text}");
    assert!(text.contains("(CHANNEL 1)"), "{text}");
    assert!(text.contains("(CHANNEL 2)"), "{text}");
    assert!(!text.contains("(sync 1)"), "{text}");
    let channel_1 = text.find("(CHANNEL 1)").expect("channel 1");
    let channel_2 = text.find("(CHANNEL 2)").expect("channel 2");
    let home = text.find("G28").expect("home");
    assert!(channel_1 < channel_2 && channel_2 < home, "{text}");
    let first_wait = text.find("M101").expect("wait");
    let second_wait = text.rfind("M101").expect("wait 2");
    assert!(channel_1 < first_wait && first_wait < channel_2 && channel_2 < second_wait, "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn single_channel_sync_stays_a_comment_on_the_two_channel_post() {
    let program = lathe_program(vec![rapid(10.0, 0.0), sync_block(1), linear(10.0, -1.0, 50.0)], turning(None, false));
    let post = two_channel();
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(text.contains("(sync 1)"), "{text}");
    assert!(!text.contains("CHANNEL"), "{text}");
    assert!(!text.contains("M101"), "{text}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn channel_balance_splits_time_at_each_sync() {
    let program = multi_program(vec![
        (0, vec![rapid(0.0, 0.0), linear(30.0, 0.0, 60.0), sync_block(1), linear(30.0, -10.0, 60.0)]),
        (1, vec![rapid(0.0, 0.0), linear(0.0, -12.0, 60.0), sync_block(1), linear(6.0, -12.0, 60.0)]),
    ]);
    let report = channel_balance(&program);
    assert_eq!(report.len(), 2);
    assert_eq!(report[0].segment, 0);
    assert_eq!(report[1].segment, 1);
    let near = |got: f64, want: f64| (got - want).abs() < 1e-9;
    assert!(near(report[0].times_s[0], 30.0) && near(report[0].times_s[1], 12.0), "{:?}", report[0].times_s);
    assert!(near(report[0].idle_s[0], 0.0) && near(report[0].idle_s[1], 18.0), "{:?}", report[0].idle_s);
    assert!(near(report[1].times_s[0], 10.0) && near(report[1].times_s[1], 6.0), "{:?}", report[1].times_s);
    assert!(near(report[1].idle_s[0], 0.0) && near(report[1].idle_s[1], 4.0), "{:?}", report[1].idle_s);
}

#[test]
fn interpolation_rejects_a_clamped_spindle_brake() {
    let haas = haas_lathe();
    for event in [Event::FaceInterpolation { on: true }, Event::CylinderInterpolation { radius: Some(20.0) }] {
        let program = millturn_program(vec![
            ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true }),
            ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }),
            ClBlock::Event(event),
        ]);
        let err = post_program(&program, &haas, &options()).expect_err("clamped");
        assert!(err.to_string().contains("interpolation with the spindle brake clamped"), "{err}");
    }
}

fn event_block(event: Event) -> ClBlock {
    ClBlock::Event(event)
}

#[test]
fn haas_writes_part_transfer_and_a_following_rapid_still_compares() {
    let blocks = vec![
        rapid(10.0, 1.0),
        event_block(Event::ChuckClamp { spindle: "sub".into(), clamp: false }),
        event_block(Event::SpindleRun { spindle: "main".into(), rpm: 0.0, reverse: false }),
        event_block(Event::SpindleRun { spindle: "sub".into(), rpm: 0.0, reverse: false }),
        event_block(Event::SpindleSync { engage: true, phase_deg: None }),
        event_block(Event::SpindleSync { engage: true, phase_deg: Some(90.0) }),
        event_block(Event::SpindleRun { spindle: "main".into(), rpm: 1_200.0, reverse: false }),
        event_block(Event::SpindleRun { spindle: "main".into(), rpm: 900.0, reverse: true }),
        event_block(Event::SpindleRun { spindle: "sub".into(), rpm: 500.0, reverse: false }),
        event_block(Event::SpindleRun { spindle: "sub".into(), rpm: 800.0, reverse: true }),
        event_block(Event::SpindleMove { spindle: "sub".into(), z: 5.0, machine: false, feed: None }),
        event_block(Event::Dwell { seconds: 0.5 }),
        event_block(Event::SpindleMove { spindle: "sub".into(), z: -2.5, machine: false, feed: Some(80.0) }),
        event_block(Event::ChuckClamp { spindle: "sub".into(), clamp: true }),
        linear(10.0, 0.0, 100.0),
        rapid(12.0, 0.0),
        event_block(Event::SpindleMove { spindle: "sub".into(), z: 5.0, machine: false, feed: None }),
        event_block(Event::SpindleSync { engage: false, phase_deg: None }),
        event_block(Event::SpindleRun { spindle: "main".into(), rpm: 0.0, reverse: true }),
        event_block(Event::SpindleMove { spindle: "sub".into(), z: -100.0, machine: true, feed: None }),
        event_block(Event::ChuckClamp { spindle: "main".into(), clamp: false }),
        event_block(Event::ChuckClamp { spindle: "main".into(), clamp: true }),
        rapid(14.0, -1.0),
    ];
    let program = lathe_program(blocks, turning(None, false));
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let m111 = index_of(&text, "M111");
    let m5 = index_of(&text, "M5");
    let m145 = index_of(&text, "M145");
    let g199 = index_of(&text, "G199 R90");
    let s1200 = index_of(&text, "G97 S1200");
    let m3 = index_of(&text, "M3");
    let s900 = index_of(&text, "G97 S900");
    let m4 = index_of(&text, "M4");
    let m143 = index_of(&text, "M143 P500");
    let m144 = index_of(&text, "M144 P800");
    let approach = index_of(&text, "G00 B5");
    let dwell = index_of(&text, "G4 P500");
    let grip = index_of(&text, "G01 B-2.5 F80");
    let m110 = index_of(&text, "M110");
    let g198 = index_of(&text, "G198");
    let park = index_of(&text, "G00 G53 B-100");
    let m11 = index_of_word(&text, "M11");
    let m10 = index_of_word(&text, "M10");
    assert!(m111 < m5 && m5 < m145 && m145 < g199, "{text}");
    assert!(text.lines().any(|line| line.split_whitespace().any(|word| word == "G199") && !line.contains('R')), "{text}");
    assert!(g199 < s1200 && s1200 < m3 && m3 < s900 && s900 < m4, "{text}");
    assert!(m4 < m143 && m143 < m144 && m144 < approach && approach < dwell && dwell < grip && grip < m110, "{text}");
    assert!(m110 < g198 && g198 < park && park < m11 && m11 < m10, "{text}");
    let rapid_after_feed = text.lines().find(|line| line.contains("X24.") || line.contains("X24")).expect("rapid after grip");
    assert!(rapid_after_feed.split_whitespace().any(|word| word == "G0" || word == "G00"), "{rapid_after_feed}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
}

#[test]
fn part_transfer_rejects_a_post_without_it_and_an_unknown_spindle() {
    let clamp = event_block(Event::ChuckClamp { spindle: "sub".into(), clamp: false });
    let sync = event_block(Event::SpindleSync { engage: true, phase_deg: Some(10.0) });
    let run = event_block(Event::SpindleRun { spindle: "sub".into(), rpm: 1.0, reverse: false });
    let moved = event_block(Event::SpindleMove { spindle: "sub".into(), z: 1.0, machine: false, feed: None });
    for block in [clamp, sync, run, moved] {
        let err = post_program(&lathe_program(vec![block], turning(None, false)), &linux_lathe(), &options()).expect_err("no transfer");
        assert!(err.to_string().contains("post `linuxcnc_lathe` has no part transfer"), "{err}");
    }
    let mill = builtin("haas").expect("haas");
    let bare = Program::try_new(
        Some("mill".into()),
        Vec::new(),
        None,
        vec![path(None, None, vec![event_block(Event::SpindleSync { engage: false, phase_deg: None })])],
        Vec::new(),
        Vec::new(),
        None,
    )
    .expect("program");
    let err = post_program(&bare, &mill, &options()).expect_err("mill");
    assert!(err.to_string().contains("post `haas` has no part transfer"), "{err}");

    let tail = [
        Event::ChuckClamp { spindle: "tail".into(), clamp: true },
        Event::SpindleMove { spindle: "tail".into(), z: 0.0, machine: true, feed: None },
        Event::SpindleRun { spindle: "tail".into(), rpm: 0.0, reverse: false },
    ];
    for event in tail {
        let err = post_program(&lathe_program(vec![event_block(event)], turning(None, false)), &haas_lathe(), &options()).expect_err("tail");
        assert!(err.to_string().contains("post `haas_lathe` has no spindle `tail`"), "{err}");
    }
    let early = post_program(
        &lathe_program(vec![event_block(Event::ChuckClamp { spindle: "tail".into(), clamp: false })], turning(None, false)),
        &linux_lathe(),
        &options(),
    )
    .expect_err("missing transfer wins");
    assert!(early.to_string().contains("post `linuxcnc_lathe` has no part transfer"), "{early}");

    let mut raw = haas_json();
    raw["lathe"]["transfer"]["sync_on"] = serde_json::json!(["G199 {rpm}"]);
    let err = load_post(&raw.to_string()).expect_err("placeholder");
    assert!(err.to_string().contains("lathe.transfer.sync_on") && err.to_string().contains("unknown placeholder {rpm}"), "{err}");
    raw = haas_json();
    raw["lathe"]["transfer"]["extra"] = serde_json::json!(true);
    assert!(load_post(&raw.to_string()).is_err());
}

#[test]
fn tailstock_writes_m21_m22_and_blocks_a_transfer_while_advanced() {
    let blocks = vec![event_block(Event::Tailstock { engage: true }), linear(20.0, -10.0, 80.0), event_block(Event::Tailstock { engage: false })];
    let program = lathe_program(blocks, turning(None, false));
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let m21 = index_of(&text, "M21");
    let m22 = index_of(&text, "M22");
    assert!(m21 < m22, "{text}");
    let between = &text[m21 + 3..m22];
    assert!(between.lines().all(|line| !line.contains('B')), "{between}");
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    for event in [
        Event::ChuckClamp { spindle: "sub".into(), clamp: false },
        Event::SpindleSync { engage: true, phase_deg: None },
        Event::SpindleMove { spindle: "sub".into(), z: 1.0, machine: false, feed: None },
    ] {
        let blocked = lathe_program(vec![event_block(Event::Tailstock { engage: true }), event_block(event)], turning(None, false));
        let err = post_program(&blocked, &post, &options()).expect_err("transfer while advanced");
        assert!(err.to_string().contains("retract the tailstock before a part transfer"), "{err}");
    }

    let linux = post_program(&lathe_program(vec![event_block(Event::Tailstock { engage: true })], turning(None, false)), &linux_lathe(), &options())
        .expect_err("linuxcnc");
    assert!(linux.to_string().contains("post `linuxcnc_lathe` has no tailstock"), "{linux}");

    let mill = Program::try_new(
        Some("mill".into()),
        Vec::new(),
        None,
        vec![path(None, None, vec![event_block(Event::Tailstock { engage: false })])],
        Vec::new(),
        Vec::new(),
        None,
    )
    .expect("program");
    let err = post_program(&mill, &builtin("haas").expect("haas"), &options()).expect_err("mill");
    assert!(err.to_string().contains("post `haas` has no tailstock"), "{err}");

    let mut raw = haas_json();
    raw["lathe"]["tailstock"]["advance"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty advance");
    assert!(err.to_string().contains("lathe.tailstock.advance") && err.to_string().contains("must not be empty"), "{err}");
    raw = haas_json();
    raw["lathe"]["tailstock"]["retract"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty retract");
    assert!(err.to_string().contains("lathe.tailstock.retract") && err.to_string().contains("must not be empty"), "{err}");
    raw = haas_json();
    raw["lathe"]["tailstock"]["advance"] = serde_json::json!(["M21 {z}"]);
    let err = load_post(&raw.to_string()).expect_err("placeholder");
    assert!(err.to_string().contains("lathe.tailstock.advance") && err.to_string().contains("unknown placeholder {z}"), "{err}");
}

#[test]
fn parts_catcher_writes_m36_m37_and_other_posts_reject_it() {
    let blocks =
        vec![event_block(Event::PartsCatcher { extend: true }), linear(20.0, -10.0, 80.0), event_block(Event::PartsCatcher { extend: false })];
    let program = lathe_program(blocks, turning(None, false));
    let post = haas_lathe();
    let text = post_program(&program, &post, &options()).expect("post");
    let m36 = index_of(&text, "M36");
    let m37 = index_of(&text, "M37");
    assert!(m36 < m37, "{text}");
    let between = &text[m36 + 3..m37];
    assert!(
        between.lines().any(|line| {
            let feed = line.split_whitespace().any(|word| word == "G1" || word == "G01");
            feed && line.contains('X')
        }),
        "{between}"
    );
    compare(&program, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");

    let linux =
        post_program(&lathe_program(vec![event_block(Event::PartsCatcher { extend: true })], turning(None, false)), &linux_lathe(), &options())
            .expect_err("linuxcnc");
    assert!(linux.to_string().contains("post `linuxcnc_lathe` has no parts catcher"), "{linux}");

    let mill = Program::try_new(
        Some("mill".into()),
        Vec::new(),
        None,
        vec![path(None, None, vec![event_block(Event::PartsCatcher { extend: false })])],
        Vec::new(),
        Vec::new(),
        None,
    )
    .expect("program");
    let err = post_program(&mill, &builtin("haas").expect("haas"), &options()).expect_err("mill");
    assert!(err.to_string().contains("post `haas` has no parts catcher"), "{err}");

    let mut raw = haas_json();
    raw["lathe"]["parts_catcher"]["extend"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty extend");
    assert!(err.to_string().contains("lathe.parts_catcher.extend") && err.to_string().contains("must not be empty"), "{err}");
    raw = haas_json();
    raw["lathe"]["parts_catcher"]["retract"] = serde_json::json!([]);
    let err = load_post(&raw.to_string()).expect_err("empty retract");
    assert!(err.to_string().contains("lathe.parts_catcher.retract") && err.to_string().contains("must not be empty"), "{err}");
    raw = haas_json();
    raw["lathe"]["parts_catcher"]["extend"] = serde_json::json!(["M36 {z}"]);
    let err = load_post(&raw.to_string()).expect_err("placeholder");
    assert!(err.to_string().contains("lathe.parts_catcher.extend") && err.to_string().contains("unknown placeholder {z}"), "{err}");
}

#[test]
fn haas_lathe_rejects_a_b_head_index() {
    let program = lathe_program(vec![ClBlock::Event(Event::HeadIndex { head: "turret1".into(), b_deg: 30.0 })], turning(None, false));
    let err = post_program(&program, &builtin("haas_lathe").expect("haas"), &options()).expect_err("b head");
    assert_eq!(err.to_string(), "not supported: post `haas_lathe` has no tilting B head");
}

#[test]
fn pivot_mode_at_b90_shifts_by_the_hand_derived_offset() {
    // L = 100 mm, B = 90°. cos 90 = 0 and sin 90 = 1, so
    // dr = L * (cos B - 1) = -100 and dz = L * sin B = 100.
    // A tip at radius 20, Z -10 is posted at radius -80, Z 90.
    // Diameter mode writes X = 2 * -80 = -160. Read-back returns the tip.
    let mut raw = lathe_json();
    raw["lathe"]["b_head"]["pivot_length"] = serde_json::json!(100.0);
    let post = post_from(raw);
    let program = lathe_program(
        vec![ClBlock::Event(Event::HeadIndex { head: "turret1".into(), b_deg: 90.0 }), linear(20.0, -10.0, 80.0)],
        turning(None, false),
    );
    let text = post_program(&program, &post, &options()).expect("post");
    assert!(has_word(&text, "B90"), "{text}");
    assert!(has_word(&text, "X-160.0000"), "{text}");
    assert!(has_word(&text, "Z90.0000"), "{text}");
    let moves = read_back(&text, &post).expect("read");
    let end = moves
        .iter()
        .find_map(|move_| match move_ {
            ReadMove::Linear { end, .. } | ReadMove::Rapid { end, .. } => Some(*end),
            _ => None,
        })
        .expect("move");
    assert!((end[0] - 20.0).abs() < 1e-6, "radius {end:?}\n{text}");
    assert!(end[1].abs() < 1e-9, "y {end:?}");
    assert!((end[2] + 10.0).abs() < 1e-6, "z {end:?}\n{text}");
    compare(&program, &moves, 1e-3, &post).expect("compare");
}
