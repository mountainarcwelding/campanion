//! Regression for #144: declaring incremental IJK must not change interpreted motion.

#![allow(clippy::expect_used)]

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, Program, Vec3};
use campanion_post::{PostDef, PostOptions, ReadMove, audit, audit_five, compare, load_post, post_program, read_back};

#[allow(clippy::panic)]
fn test_post(id: &str) -> PostDef {
    // Keep hand-written blocks readable; sequence numbering is unrelated to arc-centre mode.
    let json = match id {
        "mach3" => include_str!("../posts/mach3.json"),
        "linuxcnc_5ax" => include_str!("../posts/linuxcnc_5ax.json"),
        "linuxcnc_router" => include_str!("../posts/linuxcnc_router.json"),
        _ => panic!("unknown test post"),
    };
    load_post(&json.replace("\"enabled\": true", "\"enabled\": false")).expect("post")
}

// Nonzero start and centre distinguish incremental IJK from absolute centres. The second
// arc omits G3 to exercise modal motion after an arc-centre declaration on the first arc.
const PROGRAM: &str = "G21 G90 G17\nG0 X12 Y20 Z0\nG3 X10 Y22 Z0 I-2 J0 F100\nX8 Y20 Z1 I0 J-2\n";

#[test]
fn incremental_centres_preserve_arcs_and_helix_in_all_readers() {
    for id in ["mach3", "linuxcnc_5ax"] {
        let post = test_post(id);
        let baseline = audit(PROGRAM, &post).expect("baseline audit");
        for text in [PROGRAM.replace("G90", "G90 G91.1"), PROGRAM.replace("G3 X", "G91.1 G3 X"), PROGRAM.replace("G90", "G90 G091.100")] {
            assert_eq!(audit(&text, &post).expect("incremental audit"), baseline);
            if id == "linuxcnc_5ax" {
                assert_eq!(audit_five(&text, &post).expect("five-axis audit"), audit_five(PROGRAM, &post).expect("baseline five"));
            }
            let moves = read_back(&text, &post).expect("read-back");
            assert_eq!(moves.len(), 3);
            let ReadMove::Arc { center, end, .. } = moves.get(1).expect("arc") else { panic!("expected arc") };
            assert_eq!(*center, [10.0, 20.0, 0.0]);
            assert_eq!(*end, [10.0, 22.0, 0.0]);
            let ReadMove::Arc { center, end, .. } = moves.get(2).expect("helix") else { panic!("expected helix") };
            assert_eq!(*center, [10.0, 20.0, 0.0]);
            assert_eq!(*end, [8.0, 20.0, 1.0]);
        }
        // A declaration alone is not a motion, even after G3 has become modal.
        let text = PROGRAM.replace("X8 Y20", "G91.1\nX8 Y20");
        assert_eq!(read_back(&text, &post).expect("standalone read-back").len(), 3);
        assert_eq!(audit(&text, &post).expect("standalone audit").motions.len(), 3);
    }
}

#[test]
fn absolute_centres_and_other_unsupported_codes_are_rejected() {
    for id in ["mach3", "linuxcnc_5ax"] {
        let post = test_post(id);
        for (code, message) in [
            ("G90.1", "G90.1 absolute arc centres are not supported; use G91.1 for incremental IJK"),
            ("G91", "incremental motion is not supported"),
            ("G91.2", "G code is not an integer"),
            ("G91.1001", "G code is not an integer"),
            ("G90.1001", "G code is not an integer"),
            ("M91.1", "M code is not an integer"),
        ] {
            let text = PROGRAM.replace("G90", &format!("G90 {code}"));
            // read_back deliberately skips M codes, while the auditors validate them.
            if !code.starts_with('M') {
                let error = read_back(&text, &post).expect_err("unsupported read-back").to_string();
                assert!(error.contains(message), "{id}: {error}");
            }
            let error = audit(&text, &post).expect_err("unsupported audit").to_string();
            assert!(error.contains(message), "{id}: {error}");
            if id == "linuxcnc_5ax" {
                let error = audit_five(&text, &post).expect_err("unsupported five-axis audit").to_string();
                assert!(error.contains(message), "{error}");
            }
        }
        for codes in ["G90.1 G91.1", "G91.1 G90.1"] {
            let text = PROGRAM.replace("G90", codes);
            assert!(read_back(&text, &post).is_err());
            assert!(audit(&text, &post).is_err());
        }
    }
}

#[test]
fn custom_mach3_header_posts_and_round_trips_a_full_circle() {
    let json = include_str!("../posts/mach3.json").replace("\"G90\"", "\"G90\", \"G91.1\"");
    let post = load_post(&json).expect("custom post");
    let pose = Pose::new(Vec3::new(12.0, 20.0, 0.0).expect("point"), Vec3::Z).expect("pose");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose,
            plane: Plane::Xy,
            center: Vec3::new(10.0, 20.0, 0.0).expect("centre"),
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
            power: None,
        }),
    ];
    let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
    let path = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
    let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
    let text = post_program(&program, &post, &PostOptions { arc_tolerance: 0.01 }).expect("post");
    assert!(text.lines().any(|line| line == "G91.1"), "{text}");
    let moves = read_back(&text, &post).expect("read-back custom header");
    compare(&program, &moves, 0.01, &post).expect("full circle agrees with IR");
    assert_eq!(audit(&text, &post).expect("audit custom header").motions.len(), 2);
}

#[test]
fn router_probe_support_survives_arc_centre_declaration() {
    let post = test_post("linuxcnc_router");
    let baseline = PROGRAM.replace("G0 X12", "G38.2 Z-5 F50\nG0 X12");
    let text = baseline.replace("G90", "G90 G91.1");
    assert_eq!(audit(&text, &post).expect("probe audit"), audit(&baseline, &post).expect("baseline probe"));
    assert_eq!(read_back(&text, &post).expect("probe read-back").len(), 3);
}
