//! Mach4 v1.1 / Build 3775: public-manual semantics and synthetic geometry only.
#![allow(clippy::expect_used)]

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Document, Event, Feeds, MoveRole, Plane, Pose, ProcessKind, ProcessParams,
    Program, Tool, Vec3,
};
use campanion_post::{PostDef, PostOptions, ReadMove, audit, builtin, compare, load_post, post_program, read_back};
use proptest::prelude::*;
use serde_json::{Value, json};

fn post() -> PostDef {
    builtin("mach4").expect("Mach4 built-in")
}
fn raw_post() -> Value {
    serde_json::from_str(include_str!("../posts/mach4.json")).expect("post JSON")
}
fn pose(p: [f64; 3]) -> Pose {
    Pose::new(Vec3::new(p[0], p[1], p[2]).expect("tip"), Vec3::Z).expect("pose")
}
fn rapid(p: [f64; 3]) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: pose(p), feed: None })
}
fn program(blocks: Vec<ClBlock>) -> Program {
    let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
    let path = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, vec![], blocks).expect("path");
    Program::try_new(Some("mach4-test".into()), vec![Tool::try_new(7, None, None, None).expect("tool")], None, vec![path], vec![], vec![], None)
        .expect("program")
}
fn write(p: &Program, post: &PostDef) -> String {
    post_program(p, post, &PostOptions { arc_tolerance: 0.01 }).expect("post program")
}
fn roundtrip(p: &Program) -> String {
    let text = write(p, &post());
    compare(p, &read_back(&text, &post()).expect("read"), 0.001, &post()).expect("compare");
    audit(&text, &post()).expect("audit");
    text
}
fn dwell_seconds(text: &str, post: &PostDef) -> Vec<f64> {
    read_back(text, post)
        .expect("read")
        .into_iter()
        .filter_map(|m| match m {
            ReadMove::Dwell { seconds } => Some(seconds),
            _ => None,
        })
        .collect()
}

#[test]
fn startup_tool_change_process_and_shutdown() {
    let p = program(vec![
        rapid([0.0, 0.0, 5.0]),
        ClBlock::Event(Event::ToolChange { tool_id: 7 }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new().speed(Some(12000.0)).expect("speed") }),
        ClBlock::Event(Event::Coolant { mode: CoolantMode::Flood }),
        ClBlock::Event(Event::Coolant { mode: CoolantMode::Mist }),
        ClBlock::Motion(ClMotion::Linear { to: pose([10.0, 0.0, 5.0]), feed: 100.0, role: MoveRole::Cut, power: None }),
        // Intentionally leave the spindle and coolant on: program shutdown must stop both.
    ]);
    let text = roundtrip(&p);
    assert!(text.starts_with("(mach4-test)\nG21\nG90\nG91.1\nG94\nG17\nG40\nG49\nG80\n"), "{text}");
    assert!(text.contains("T7 M6\nM3 S12000\nM8\nM7\n"), "{text}");
    assert!(text.ends_with("M5\nM9\nM30\n"), "{text}");
    assert!(!text.contains("G43") && !text.contains("G53"), "{text}");
    assert!(!text.lines().any(|l| l.starts_with('N') || l.starts_with('O')), "{text}");
}

#[test]
fn writer_keeps_decimal_seconds_even_when_zeroes_are_trimmed() {
    let values = [0.0, 2.0, 0.5, 1.25, 2.0001];
    let p = program(values.into_iter().map(|seconds| ClBlock::Event(Event::Dwell { seconds })).collect());
    let text = write(&p, &post());
    let lines: Vec<_> = text.lines().filter(|l| l.starts_with("G4 ")).collect();
    assert_eq!(lines, ["G4 P2.0", "G4 P0.5", "G4 P1.25", "G4 P2.0"]);
    assert_eq!(dwell_seconds(&text, &post()), [2.0, 0.5, 1.25, 2.0]);
    audit(&text, &post()).expect("dwell audit");
    for trailing in [false, true] {
        let mut raw = raw_post();
        raw["decimals"]["dwell"] = json!(0);
        raw["trailing_zeros"] = json!(trailing);
        let post = load_post(&raw.to_string()).expect("zero decimal precision");
        let text = write(&program(vec![ClBlock::Event(Event::Dwell { seconds: 2.0 })]), &post);
        assert!(text.contains("G4 P2.0\n"), "{text}");
        assert_eq!(dwell_seconds(&text, &post), [2.0]);
    }
}

#[test]
fn zero_duration_is_a_no_op_without_changing_surrounding_output() {
    for seconds in [0.0, -0.0] {
        let without = program(vec![rapid([0.0, 0.0, 5.0]), ClBlock::Event(Event::Dwell { seconds: 0.5 }), rapid([1.0, 0.0, 5.0])]);
        let with = program(vec![
            ClBlock::Event(Event::Dwell { seconds }),
            rapid([0.0, 0.0, 5.0]),
            ClBlock::Event(Event::Dwell { seconds }),
            ClBlock::Event(Event::Dwell { seconds: 0.5 }),
            rapid([1.0, 0.0, 5.0]),
            ClBlock::Event(Event::Dwell { seconds }),
        ]);
        assert_eq!(roundtrip(&with), roundtrip(&without));
    }
}

#[test]
fn positive_dwells_that_round_to_zero_fail_for_custom_precisions() {
    for decimals in [0, 1, 3, 6] {
        let increment = 10.0_f64.powi(-decimals);
        for trailing in [false, true] {
            let mut raw = raw_post();
            raw["id"] = json!("custom-dwell-test");
            raw["decimals"]["dwell"] = json!(decimals);
            raw["trailing_zeros"] = json!(trailing);
            let post = load_post(&raw.to_string()).expect("custom post");
            for seconds in [f64::MIN_POSITIVE, 0.4999 * increment] {
                let p = program(vec![ClBlock::Event(Event::Dwell { seconds })]);
                let err = post_program(&p, &post, &PostOptions { arc_tolerance: 0.01 }).expect_err("rounded-zero dwell");
                assert!(err.to_string().contains("positive dwell rounds to zero"), "{err}");
            }
            let p = program(vec![ClBlock::Event(Event::Dwell { seconds: 0.5001 * increment })]);
            let text = write(&p, &post);
            assert_eq!(dwell_seconds(&text, &post), [increment]);
            audit(&text, &post).expect("nonzero rounded dwell");
            assert!(!write(&program(vec![ClBlock::Event(Event::Dwell { seconds: 0.0 })]), &post).contains("G4 "));
        }
    }
}

#[test]
fn legacy_posts_keep_zero_and_rounded_zero_dwells() {
    for id in ["mach3", "grbl", "fanuc"] {
        let post = builtin(id).expect("legacy post");
        let text = write(&program(vec![ClBlock::Event(Event::Dwell { seconds: 0.0 }), ClBlock::Event(Event::Dwell { seconds: 1e-12 })]), &post);
        assert_eq!(dwell_seconds(&text, &post), [0.0, 0.0], "{id}: {text}");
        assert_eq!(text.lines().filter(|l| l.split_whitespace().any(|word| word == "G4")).count(), 2, "{id}: {text}");
        audit(&text, &post).expect("legacy zero audit");
    }
}

#[test]
fn literal_spelling_controls_duration_including_glued_words_and_comments() {
    for (word, seconds) in [("2", 0.002), ("2000", 2.0), ("2.0", 2.0), ("2.", 2.0), (".5", 0.5), ("+0002", 0.002)] {
        let text = format!("g04p{word}(P9.9) ; P8.8\n");
        assert_eq!(dwell_seconds(&text, &post()), [seconds], "{text}");
        audit(&text, &post()).expect("literal dwell audit");
    }
    // Dwell neither moves nor replaces modal G1/feed.
    let text = "G21 G90 G17\nG0 X0 Y0 Z0\nG1 X1 F100\nG4 P2\nX2\n";
    assert_eq!(dwell_seconds(text, &post()), [0.002]);
    let report = audit(text, &post()).expect("modal audit");
    assert_eq!(report.motions.last().expect("last move").end, [2.0, 0.0, 0.0]);
    assert_eq!(report.motions.last().expect("last move").feed, Some(100.0));
}

#[test]
fn custom_posts_inherit_the_setting_and_existing_units_keep_their_meaning() {
    let mut raw = raw_post();
    raw["id"] = json!("custom-mill");
    let custom = load_post(&raw.to_string()).expect("custom");
    assert_eq!(dwell_seconds("G4 P2\nG4 P2.0", &custom), [0.002, 2.0]);
    for (unit, expected, token) in [("s", 2.0, "P2"), ("ms", 0.002, "P2000")] {
        raw["dwell"] = json!({"word": "P", "unit": unit});
        let legacy = load_post(&raw.to_string()).expect("legacy dwell");
        assert_eq!(dwell_seconds("G4 P2\nG4 P2.0", &legacy), [expected, expected]);
        let text = write(&program(vec![ClBlock::Event(Event::Dwell { seconds: 2.0 })]), &legacy);
        assert!(text.contains(&format!("G4 {token}\n")), "{text}");
    }
    for dwell in [json!({"word":"P","unit":"ms","decimal_seconds":true}), json!({"word":"X","unit":"s","decimal_seconds":true})] {
        raw["dwell"] = dwell;
        let err = load_post(&raw.to_string()).expect_err("incompatible dwell");
        assert!(err.to_string().contains("dwell.decimal_seconds"), "{err}");
    }
}

#[test]
fn malformed_dwell_blocks_fail_in_reader_and_audit() {
    for text in [
        "G4",
        "G4 P0",
        "G4 P0.0",
        "G4 P-0.0",
        "G4 P+0000.",
        "G4 P-1",
        "G4 P1 P2.0",
        "G4 G4 P2",
        "G4 G90 P2",
        "G91.1 G4 P2",
        "G4 G53 P2",
        "G53 G4 P2",
        "G4 G0 P2",
        "G4 G1 X1 P2 F100",
        "G4 P2 X1",
        "G4 P2 I1",
        "G4 P2 J1",
        "G4 P2 K1",
        "G4 P2 R1",
        "G4 P2 Q1",
        "G4 X2",
        "G4 U2",
        "G4 P#2",
        "G4 PNaN",
        "G4 Pinf",
        "G4 P1e3",
        "G4 P2..0",
    ] {
        assert!(read_back(text, &post()).is_err(), "reader accepted {text}");
        assert!(audit(text, &post()).is_err(), "audit accepted {text}");
    }
    let overflow = format!("G4 P{}", "9".repeat(400));
    assert!(read_back(&overflow, &post()).is_err());
    assert!(audit(&overflow, &post()).is_err());
    // IR validation or writer validation must reject hostile event values.
    for seconds in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        if let Ok(path) = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, vec![], vec![ClBlock::Event(Event::Dwell { seconds })])
            && let Ok(p) = Program::try_new(None, vec![], None, vec![path], vec![], vec![], None)
        {
            assert!(post_program(&p, &post(), &PostOptions { arc_tolerance: 0.01 }).is_err());
        }
    }
}

#[test]
fn arcs_circles_and_helices_use_incremental_ijk_in_every_plane() {
    for plane in [Plane::Xy, Plane::Zx, Plane::Yz] {
        let map = |u, v, w| match plane {
            Plane::Xy => [u, v, w],
            Plane::Zx => [v, w, u],
            Plane::Yz => [w, u, v],
        };
        // Nonzero centre distinguishes incremental offsets from absolute coordinates.
        let center = map(10.0, 20.0, 3.0);
        for sense in [ArcSense::Cw, ArcSense::Ccw] {
            for full in [false, true] {
                for rise in [0.0, 2.0] {
                    let end =
                        if full { map(12.0, 20.0, 3.0 + rise) } else { map(10.0, if sense == ArcSense::Ccw { 22.0 } else { 18.0 }, 3.0 + rise) };
                    let motion = if rise == 0.0 {
                        ClMotion::Arc {
                            to: pose(end),
                            plane,
                            center: Vec3::new(center[0], center[1], center[2]).expect("centre"),
                            sense,
                            feed: 100.0,
                            role: MoveRole::Cut,
                            power: None,
                        }
                    } else {
                        ClMotion::Helix {
                            to: pose(end),
                            plane,
                            center: Vec3::new(center[0], center[1], center[2]).expect("centre"),
                            sense,
                            sweep: if full { std::f64::consts::TAU } else { std::f64::consts::FRAC_PI_2 },
                            feed: 100.0,
                            role: MoveRole::Cut,
                            power: None,
                        }
                    };
                    let p = program(vec![rapid(map(12.0, 20.0, 3.0)), ClBlock::Motion(motion)]);
                    let text = roundtrip(&p);
                    let arc = text
                        .lines()
                        .find(|l| l.split_whitespace().any(|w| w == "G2" || w == "G3"))
                        .unwrap_or_else(|| panic!("missing arc {plane:?} {sense:?} full={full} rise={rise}\n{text}"));
                    let offsets = match plane {
                        Plane::Xy => "I-2 J0",
                        Plane::Zx => "I0 K-2",
                        Plane::Yz => "J-2 K0",
                    };
                    assert!(arc.contains(offsets), "{text}");
                    assert!(!arc.split_whitespace().any(|w| w.starts_with('R')), "{text}");
                    let read = read_back(&text, &post()).expect("read");
                    let ReadMove::Arc { center: got_center, end: got_end, .. } =
                        read.iter().find(|m| matches!(m, ReadMove::Arc { .. })).expect("arc")
                    else {
                        panic!("expected arc")
                    };
                    assert_eq!(*got_center, center);
                    assert_eq!(*got_end, end);
                }
            }
        }
    }
}

#[test]
fn rotary_five_axis_and_turning_programs_are_rejected() {
    for source in [include_str!("data/indexed.ir.json"), include_str!("data/wrap.ir.json"), include_str!("data/turn.ir.json")] {
        let doc = Document::from_json(source).expect("fixture");
        let err = post_program(doc.program(), &post(), &PostOptions { arc_tolerance: 0.01 }).expect_err("unsupported family");
        assert!(err.to_string().contains("not supported"), "{err}");
    }
    let tilted = Pose::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0).expect("axis")).expect("tilt");
    let p = program(vec![ClBlock::Motion(ClMotion::Rapid { to: tilted, feed: None })]);
    let err = post_program(&p, &post(), &PostOptions { arc_tolerance: 0.01 }).expect_err("tilted tool");
    assert!(err.to_string().contains("tilted-plane"), "{err}");
    for axis in ["A90", "B90", "C90"] {
        let text = format!("G21 G90 G17\nG0 X0 Y0 Z0 {axis}");
        assert!(read_back(&text, &post()).is_err());
        assert!(audit(&text, &post()).is_err());
    }
}

proptest! {
    #[test]
    fn integer_and_decimal_p_differ_by_one_thousand(ms in 1u32..1_000_000) {
        let text = format!("G4 P{ms}\nG4 P{ms}.0");
        prop_assert_eq!(dwell_seconds(&text, &post()), vec![f64::from(ms) / 1000.0, f64::from(ms)]);
        prop_assert!(audit(&text, &post()).is_ok());
    }

    #[test]
    fn written_dwell_rounds_to_milliseconds_without_changing_units(ticks in 1u32..1_000_000, trailing in any::<bool>()) {
        let mut raw = raw_post();
        raw["trailing_zeros"] = json!(trailing);
        let post = load_post(&raw.to_string()).expect("custom post");
        let seconds = f64::from(ticks) / 1000.0;
        let text = write(&program(vec![ClBlock::Event(Event::Dwell { seconds })]), &post);
        let parsed = dwell_seconds(&text, &post);
        prop_assert!((parsed[0] - seconds).abs() < 1e-9);
        prop_assert!(text.lines().filter(|l| l.starts_with("G4 ")).all(|l| l.contains('.')));
    }
}
