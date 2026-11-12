//! Synthetic inch programs: physical round trips, modal replay, and precision boundaries.
#![allow(clippy::expect_used)]

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Plane, Pose, Program, Vec3};
use campanion_post::{
    PostDef, PostOptions, ReadMove, UnitMode, audit, audit_five, audit_lathe, builtin, compare, load_post, post_program, read_back,
};
use proptest::prelude::*;
use serde_json::{Value, json};

fn raw_post() -> Value {
    let mut raw: Value = serde_json::from_str(include_str!("../posts/linuxcnc_router.json")).expect("valid synthetic fixture");
    raw["units"] = json!("inch");
    raw["line_numbers"]["enabled"] = json!(false);
    // Literal template values are in output units, not converted by the writer.
    raw["header"] = json!(["G20", "G90", "G17", "G40", "G49", "G64 P0.0004"]);
    raw
}
fn post() -> PostDef {
    load_post(&raw_post().to_string()).expect("valid synthetic fixture")
}
fn pose(p: [f64; 3]) -> Pose {
    Pose::new(Vec3::new(p[0], p[1], p[2]).expect("valid synthetic fixture"), Vec3::Z).expect("valid synthetic fixture")
}
fn rapid(p: [f64; 3]) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: pose(p), feed: None })
}
fn line(p: [f64; 3], feed: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: pose(p), feed, role: MoveRole::Cut, power: None })
}
fn program(blocks: Vec<ClBlock>) -> Program {
    let feeds = Feeds::try_new(254.0, None, None, None, None).expect("valid synthetic fixture");
    let path = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, vec![], blocks).expect("valid synthetic fixture");
    Program::try_new(None, vec![], None, vec![path], vec![], vec![], None).expect("valid synthetic fixture")
}
fn roundtrip(program: &Program, post: &PostDef) -> String {
    let text = post_program(program, post, &PostOptions { arc_tolerance: 0.05 }).expect("valid synthetic fixture");
    let moves = read_back(&text, post).expect("valid synthetic fixture");
    compare(program, &moves, 0.005, post).expect("valid synthetic fixture");
    let report = audit(&text, post).expect("valid synthetic fixture");
    assert!(report.motions.iter().all(|m| m.units == UnitMode::Inch));
    text
}

#[test]
fn known_lengths_feeds_and_probe_roundtrip() {
    let p = program(vec![
        rapid([0.0; 3]),
        ClBlock::Event(Event::Probe { toward: Some(pose([0.0, 0.0, -25.4])), feed: Some(254.0) }),
        rapid([0.0, 0.0, 25.4]),
        ClBlock::Event(Event::Dwell { seconds: 0.5 }),
        line([25.4, -50.8, 0.0], 254.0),
    ]);
    let text = roundtrip(&p, &post());
    assert_eq!(text, include_str!("golden/inch.router.nc"));
    assert!(text.contains("G38.2 X0.0000 Y0.0000 Z-1.0000 F10.0000"), "{text}");
    assert!(text.contains("X1.0000 Y-2.0000 Z0.0000"), "{text}");
    assert!(text.contains("G4 P0.500"), "{text}");
    assert!(text.contains("G64 P0.0004"), "{text}");
}

#[test]
fn block_order_and_modal_feeds_are_physical() {
    let text = "F10 X1 G1 G20 G90 G17 S12000 M3 T7 M6\nY2\nG4 P0.5\nG1 Z1\n";
    let report = audit(text, &post()).expect("valid synthetic fixture");
    assert_eq!(report.motions[0].end, [25.4, 0.0, 0.0]);
    assert_eq!(report.motions[1].end, [25.4, 50.8, 0.0]);
    assert!(report.motions.iter().all(|m| m.feed == Some(254.0)));
    assert_eq!(report.motions[0].spindle.speed, 12000.0);
    assert_eq!(report.motions[0].tool, Some(7));
    assert!(read_back(text, &post()).expect("valid synthetic fixture").iter().any(|m| matches!(m, ReadMove::Dwell {seconds} if *seconds == 0.5)));
}

#[test]
fn inverse_time_is_not_length_scaled() {
    let text = "G20 G90 G17\nF2 X1 G1 G93\nF10 G94 X2\nX3\n";
    let report = audit(text, &post()).expect("valid synthetic fixture");
    assert_eq!(report.motions[0].feed, Some(2.0));
    assert_eq!(report.motions[1].feed, Some(254.0));
    assert_eq!(report.motions[2].feed, Some(254.0));
}

#[test]
fn feed_mode_changes_require_a_new_dimensional_feed() {
    let text = "G20 G90 G17\nG93 G1 X1 F2\nG94\nG1 X2\n";
    assert!(read_back(text, &post()).is_err());
    assert!(audit(text, &post()).is_err());
}

#[test]
fn units_must_match_before_lengths_and_cannot_switch() {
    for text in ["G90 G1 X1 F10", "G21\nG20 G90 G1 X1 F10", "G20 G90 G1 X1 F10\nG21", "G20 G21", "G1 X1\nG20"] {
        assert!(read_back(text, &post()).is_err(), "{text}");
        assert!(audit(text, &post()).is_err(), "{text}");
    }
    let metric = builtin("linuxcnc_router").expect("valid synthetic fixture");
    assert!(read_back("G20", &metric).is_err());
    assert!(audit("G20", &metric).is_err());
    for header in
        [json!(["G21", "G90"]), json!(["G90"]), json!(["G0 X1", "G20", "G90"]), json!(["G20", "G93"]), json!(["G20", "G95"]), json!(["G20", "G91"])]
    {
        let mut raw = raw_post();
        raw["header"] = header;
        let post = load_post(&raw.to_string()).expect("valid synthetic fixture");
        assert!(post_program(&program(vec![rapid([0.0; 3])]), &post, &PostOptions { arc_tolerance: 0.05 }).is_err());
    }
    let mut raw = raw_post();
    raw["footer"] = json!(["G21"]);
    assert!(
        post_program(
            &program(vec![rapid([0.0; 3])]),
            &load_post(&raw.to_string()).expect("valid synthetic fixture"),
            &PostOptions { arc_tolerance: 0.05 }
        )
        .is_err()
    );
}

#[test]
fn unsupported_families_remain_explicit() {
    for source in [include_str!("../posts/linuxcnc.json"), include_str!("../posts/linuxcnc_5ax.json"), include_str!("../posts/linuxcnc_lathe.json")] {
        let mut raw: Value = serde_json::from_str(source).expect("valid synthetic fixture");
        raw["units"] = json!("inch");
        let err = load_post(&raw.to_string()).unwrap_err().to_string();
        assert!(err.contains("three-axis"), "{err}");
    }
    assert!(audit_five("N10 G20", &builtin("linuxcnc_5ax").expect("valid synthetic fixture")).unwrap_err().to_string().contains("not supported yet"));
    let err = audit_lathe("N10 G20", &builtin("linuxcnc_lathe").expect("valid synthetic fixture")).unwrap_err();
    assert!(err.to_string().contains("not supported yet"), "{err}");
    assert!(audit("G20 G90 G0 A90", &post()).is_err());
}

#[test]
fn feed_rounding_and_comparison_use_inch_quantum() {
    let mut raw = raw_post();
    raw["decimals"]["feed"] = json!(2);
    let post = load_post(&raw.to_string()).expect("valid synthetic fixture");
    roundtrip(&program(vec![rapid([0.0; 3]), line([25.4, 0.0, 0.0], 254.1)]), &post);
    let err = post_program(&program(vec![rapid([0.0; 3]), line([1.0; 3], 0.01)]), &post, &PostOptions { arc_tolerance: 0.05 }).unwrap_err();
    assert!(err.to_string().contains("rounds to zero"));
    let p = program(vec![rapid([0.0; 3]), line([25.4, 0.0, 0.0], 254.1)]);
    let text = post_program(&p, &post, &PostOptions { arc_tolerance: 0.05 }).expect("valid synthetic fixture").replace("F10.00", "F10.02");
    assert!(compare(&p, &read_back(&text, &post).expect("valid synthetic fixture"), 0.005, &post).is_err());
}

fn arc_program(plane: Plane, radius: f64, sweep: f64, rise: f64, feed: f64) -> Program {
    let map = |u, v, w| match plane {
        Plane::Xy => [u, v, w],
        Plane::Zx => [v, w, u],
        Plane::Yz => [w, u, v],
    };
    let end = if sweep == std::f64::consts::TAU { map(radius, 0.0, rise) } else { map(radius * sweep.cos(), radius * sweep.sin(), rise) };
    program(vec![
        rapid(map(radius, 0.0, 0.0)),
        ClBlock::Motion(ClMotion::Arc { to: pose(end), plane, center: Vec3::ZERO, sense: ArcSense::Ccw, feed, role: MoveRole::Cut, power: None }),
    ])
}

#[test]
fn arcs_helices_full_circles_and_lines_in_every_plane() {
    for mode in ["ijk", "r", "lines"] {
        let mut raw = raw_post();
        raw["arcs"]["mode"] = json!(mode);
        let post = load_post(&raw.to_string()).expect("valid synthetic fixture");
        for plane in [Plane::Xy, Plane::Zx, Plane::Yz] {
            for sweep in [std::f64::consts::FRAC_PI_2, std::f64::consts::PI, 4.5, std::f64::consts::TAU] {
                for rise in [0.0, 2.54] {
                    roundtrip(&arc_program(plane, 25.4, sweep, rise, 254.0), &post);
                }
            }
        }
    }
}

#[test]
fn sub_quantum_arc_does_not_turn_into_full_circle() {
    let p = arc_program(Plane::Xy, 25.4, 0.00001, 0.0, 254.0);
    let text = roundtrip(&p, &post());
    assert!(!text.lines().any(|l| l.starts_with("G2 ") || l.starts_with("G3 ")), "{text}");
}

#[test]
fn decimal_point_pins_and_numbering_are_not_scaled() {
    let mut raw = raw_post();
    raw["trailing_zeros"] = json!(false);
    raw["decimal_point"] = json!(true);
    raw["line_numbers"]["enabled"] = json!(true);
    raw["header"] = json!(["G20", "G90", "G17", "M64 P1"]);
    let text =
        roundtrip(&program(vec![rapid([0.0; 3]), line([25.4, 0.0, 0.0], 254.0)]), &load_post(&raw.to_string()).expect("valid synthetic fixture"));
    assert!(text.contains("N40 M64 P1"), "{text}");
    assert!(text.contains("X1. Y0. Z0. F10"), "{text}");
}

#[test]
fn physical_tolerance_is_not_relaxed_to_hide_rounding_error() {
    let p = program(vec![rapid([0.0; 3]), line([25.4008, 0.0, 0.0], 254.0)]);
    let text = post_program(&p, &post(), &PostOptions { arc_tolerance: 0.05 }).expect("valid synthetic fixture");
    let moves = read_back(&text, &post()).expect("valid synthetic fixture");
    assert!(compare(&p, &moves, 0.0001, &post()).is_err());
    compare(&p, &moves, 0.002, &post()).expect("valid synthetic fixture");
}

#[test]
fn scaled_overflow_and_conflicting_feed_modes_are_rejected() {
    let huge = "9".repeat(308);
    for text in [format!("G20 G90 G1 X{huge} F10"), format!("G20 G90 G1 X1 F{huge}"), "G20 G90 G1 X1 F2 G93 G94".into()] {
        assert!(read_back(&text, &post()).is_err(), "{text}");
        assert!(audit(&text, &post()).is_err(), "{text}");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(100))]
    #[test]
    fn arbitrary_lines_roundtrip(x in -500.0..500.0, y in -500.0..500.0, z in -50.0..50.0, feed in 1.0..3000.0) {
        roundtrip(&program(vec![rapid([0.0; 3]), line([x,y,z],feed)]), &post());
    }
    #[test]
    fn arbitrary_arcs_roundtrip(radius in 1.0..100.0, sweep in 0.1..6.2, rise in -10.0..10.0, feed in 1.0..3000.0, r_mode in any::<bool>()) {
        let mut raw = raw_post(); if r_mode { raw["arcs"]["mode"] = json!("r"); }
        roundtrip(&arc_program(Plane::Xy,radius,sweep,rise,feed), &load_post(&raw.to_string()).expect("valid synthetic fixture"));
    }
}
