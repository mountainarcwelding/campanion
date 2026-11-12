#![allow(clippy::expect_used)]
//! Optional G41/G42 from IR controller-side compensation.

use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Pose, ProcessKind, ProcessParams, Program, Tool, Vec3};
use campanion_post::{PostOptions, audit, builtin, compare, post_program, read_back};
use proptest::prelude::*;

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn options() -> PostOptions {
    PostOptions { arc_tolerance: 0.01 }
}

fn path(compensation: CompensationMode, tool_id: Option<u32>, blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some("profile".into()),
        tool_id,
        compensation,
        Feeds::try_new(1000.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
}

fn tools(id: u32) -> Vec<Tool> {
    vec![Tool::try_new(id, Some("torch".into()), None, Some(1.5)).expect("tool")]
}

fn torch_cut(compensation: CompensationMode, tool_id: Option<u32>) -> Program {
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 5.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() }),
        ClBlock::Motion(ClMotion::Linear { to: pose(2.0, 0.0, 0.0), feed: 1000.0, role: MoveRole::LeadIn, power: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(20.0, 0.0, 0.0), feed: 1000.0, role: MoveRole::Cut, power: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(22.0, 0.0, 0.0), feed: 1000.0, role: MoveRole::LeadOut, power: None }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Torch }),
        ClBlock::Motion(ClMotion::Rapid { to: pose(22.0, 0.0, 5.0), feed: None }),
    ];
    Program::try_new(
        Some("kerf-comp".into()),
        tools(tool_id.unwrap_or(1)),
        None,
        vec![path(compensation, tool_id, blocks)],
        Vec::new(),
        Vec::new(),
        None,
    )
    .expect("program")
}

fn codes(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for line in text.lines() {
        let body = line.split('(').next().unwrap_or(line);
        for word in body.split_whitespace() {
            if word.starts_with('N') && word[1..].chars().all(|ch| ch.is_ascii_digit()) {
                continue;
            }
            if word == "G41" || word == "G42" || word == "G40" || word.starts_with("G41") || word.starts_with("G42") || word.starts_with('D') {
                out.push(word.to_string());
            }
        }
    }
    out
}

fn post_roundtrip(id: &str, program: &Program) -> String {
    let post = builtin(id).expect("post");
    let text = post_program(program, &post, &options()).expect("post");
    let moves = read_back(&text, &post).expect("read");
    compare(program, &moves, 0.02, &post).expect("compare");
    audit(&text, &post).expect("audit");
    text
}

#[test]
fn plasma_writes_g41_before_the_first_feed_and_g40_at_path_end() {
    let text = post_roundtrip("plasma", &torch_cut(CompensationMode::ControllerLeft, Some(1)));
    let words = codes(&text);
    assert!(words.contains(&"G41".to_string()), "{text}");
    assert!(words.contains(&"G40".to_string()), "{text}");
    assert!(!words.contains(&"G42".to_string()), "{text}");
    let start = text.find("\nN110 G41\n").or_else(|| text.find("G41\n")).expect("G41");
    let feed = text.find("G1 X").expect("G1 feed");
    let end = text.rfind("G40").expect("G40");
    assert!(start < feed, "{text}");
    assert!(feed < end, "{text}");
}

#[test]
fn plasma_writes_g42_for_controller_right() {
    let text = post_roundtrip("plasma", &torch_cut(CompensationMode::ControllerRight, Some(1)));
    assert!(text.contains("G42"), "{text}");
    assert!(!text.contains("G41"), "{text}");
    assert!(text.contains("G40"), "{text}");
}

#[test]
fn computed_plasma_program_does_not_write_g41_or_g42() {
    let text = post_roundtrip("plasma", &torch_cut(CompensationMode::Computed, Some(1)));
    assert!(!text.contains("G41"), "{text}");
    assert!(!text.contains("G42"), "{text}");
}

#[test]
fn linuxcnc_writes_g41_without_a_d_word() {
    let text = post_roundtrip("linuxcnc", &torch_cut(CompensationMode::ControllerLeft, Some(1)));
    assert!(text.contains("G41"), "{text}");
    assert!(!codes(&text).iter().any(|word| word.starts_with('D')), "{text}");
}

#[test]
fn fanuc_writes_g41_with_the_tool_offset() {
    let text = post_roundtrip("fanuc", &torch_cut(CompensationMode::ControllerLeft, Some(3)));
    assert!(text.contains("G41"), "{text}");
    assert!(text.contains("D3"), "{text}");
    assert!(text.contains("G40"), "{text}");
}

#[test]
fn fanuc_refuses_controller_comp_without_a_tool() {
    let program = torch_cut(CompensationMode::ControllerLeft, None);
    // tools() still added a tool table entry, but the path has no tool_id
    let post = builtin("fanuc").expect("fanuc");
    let err = post_program(&program, &post, &options()).expect_err("no tool");
    assert!(err.to_string().contains("tool number"), "{err}");
}

#[test]
fn grbl_still_refuses_controller_compensation() {
    let program = torch_cut(CompensationMode::ControllerRight, Some(1));
    let post = builtin("grbl").expect("grbl");
    let err = post_program(&program, &post, &options()).expect_err("grbl");
    assert!(err.to_string().contains("controller right compensation is not supported"), "{err}");
}

#[test]
fn audit_rejects_uncancelled_compensation() {
    let post = builtin("plasma").expect("plasma");
    let cancelled = "N10 G21\nN20 G90\nN30 G17\nN40 G41\nN50 G1 X1. Y0. F1000.\nN60 G40\nN70 M2\n";
    audit(cancelled, &post).expect("cancelled");
    let err = audit("N10 G21\nN20 G90\nN30 G17\nN40 G41\nN50 G1 X1. Y0. F1000.\n", &post).expect_err("open");
    assert!(err.to_string().contains("cutter compensation is not cancelled"), "{err}");
    audit("N10 G21\nN20 G90\nN30 G17\nN40 G41\nN50 G1 X1. Y0. F1000.\nN60 M2\n", &post).expect("m2 cancels");
}

#[test]
fn audit_rejects_a_d_word_without_g41() {
    let post = builtin("fanuc").expect("fanuc");
    let err = audit("%\nO0001\nN10 G21 G90 G17 G40 G49 G80\nN20 G0 X1. Y0. Z0. D3\nN30 M30\n%\n", &post).expect_err("d");
    assert!(err.to_string().contains("D word needs G41 or G42"), "{err}");
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]

    #[test]
    fn left_or_right_roundtrips_on_plasma_and_linuxcnc(left in proptest::bool::ANY, x in 1.0f64..=50.0, y in -20.0f64..=20.0) {
        let mode = if left { CompensationMode::ControllerLeft } else { CompensationMode::ControllerRight };
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 5.0), feed: None }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() }),
            ClBlock::Motion(ClMotion::Linear { to: pose(x, y, 0.0), feed: 800.0, role: MoveRole::Cut, power: None }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Torch }),
            ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, 5.0), feed: None }),
        ];
        let program = Program::try_new(
            Some("prop".into()),
            tools(1),
            None,
            vec![path(mode, Some(1), blocks)],
            Vec::new(),
            Vec::new(),
            None,
        )
        .expect("program");
        for id in ["plasma", "linuxcnc"] {
            let text = post_roundtrip(id, &program);
            let want = if left { "G41" } else { "G42" };
            prop_assert!(text.contains(want), "{id}: {text}");
            prop_assert!(text.contains("G40"), "{id}: {text}");
        }
    }
}
