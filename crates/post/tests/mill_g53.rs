//! Regression for #145: machine retracts are not part-frame motion.

#![allow(clippy::expect_used)]

use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Pose, Program, Tool, Vec3};
use campanion_post::{MotionMode, PostOptions, audit, builtin, compare, load_post, post_program, read_back};
use proptest::prelude::*;

const PROGRAM: &str = "G21 G90 G17\nG0 X12 Y20 Z5\nG1 Z0 F100\nG3 X10 Y22 I-2 J0\nX8 Y20 I0 J-2\n";

#[test]
fn machine_retract_preserves_part_position_and_modal_arc() {
    let post = builtin("mach3").expect("post");
    let baseline = audit(PROGRAM, &post).expect("baseline");
    for retract in ["G53 G0 Z0", "G0 X100 Y200 Z300 G53", "G53 G0 G18 Z0"] {
        let text = PROGRAM.replace("X8 Y20", &format!("{retract}\nX8 Y20"));
        let report = audit(&text, &post).expect("audit G53");
        assert_eq!(report.motions.len(), baseline.motions.len());
        for (mut actual, expected) in report.motions.into_iter().zip(&baseline.motions) {
            actual.line = expected.line;
            assert_eq!(&actual, expected);
        }
        let moves = read_back(&text, &post).expect("read-back G53");
        assert_eq!(moves.len(), 4);
        assert!(matches!(moves.last(), Some(campanion_post::ReadMove::Arc { end, center, .. })
            if *end == [8.0, 20.0, 0.0] && *center == [10.0, 20.0, 0.0]));
    }
}

#[test]
fn custom_tool_change_retract_posts_and_round_trips() {
    let json = include_str!("../posts/mach3.json").replace("\"T{tool} M6\"", "\"G53 G0 Z0\", \"T{tool} M6\"");
    let post = load_post(&json).expect("custom post");
    let pose = |x, y, z| Pose::new(Vec3::new(x, y, z).expect("point"), Vec3::Z).expect("pose");
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Motion(ClMotion::Rapid { to: pose(12.0, 20.0, 5.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(12.0, 20.0, 0.0), feed: 100.0, role: MoveRole::Cut, power: None }),
    ];
    let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
    let path = ClPath::try_new(None, Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
    let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
    let program = Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program");
    let text = post_program(&program, &post, &PostOptions { arc_tolerance: 0.01 }).expect("post");
    assert!(text.contains("G53 G0 Z0\nT1 M6"), "{text}");
    let moves = read_back(&text, &post).expect("read-back");
    compare(&program, &moves, 0.01, &post).expect("IR round-trip");
    let report = audit(&text, &post).expect("audit");
    assert_eq!(report.motions.len(), 2);
    assert_eq!(report.tool, Some(1));
}

#[test]
fn machine_retract_does_not_bypass_word_validation() {
    let post = builtin("mach3").expect("post");
    for block in ["G53 G0 G1 Z0", "G53 G91 Z0", "G53 G123 Z0", "G53 G0 Z0 Z1", "G53 G0 Z1000000001", "G53 G0 Z0 F-1"] {
        let text = format!("G21 G90 G17\n{block}\nG0 X1 Y2 Z3\n");
        assert!(audit(&text, &post).is_err(), "{block}");
    }
}

proptest! {
    #[test]
    fn machine_axes_never_leak_into_following_part_move(x in -1000i32..1000, y in -1000i32..1000, z in -1000i32..1000) {
        let post = builtin("mach3").expect("post");
        let text = format!("G21 G90 G17\nG0 X12 Y20 Z5\nG1 Z-2 F100\nG53 G0 X{x} Y{y} Z{z}\nX14\n");
        let report = audit(&text, &post).expect("audit");
        let moves = read_back(&text, &post).expect("read-back");
        prop_assert_eq!(moves.len(), 3);
        prop_assert_eq!(report.motions.len(), 3);
        let last = report.motions.last().expect("last motion");
        prop_assert_eq!(last.end, [14.0, 20.0, -2.0]);
        prop_assert_eq!(last.mode, MotionMode::Linear);
    }
}
