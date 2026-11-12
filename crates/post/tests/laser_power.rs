#![allow(clippy::expect_used)]
//! Per-move laser S words on `grbl_laser`.
//!
//! Vector programs that omit move power keep the existing process-on default (full scale).
//! A raster sets every feed, including S0 on overscan, and never fills a missing value as S1000.

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Plane, Pose, ProcessKind, ProcessParams, Program, Vec3,
};
use campanion_post::{PostOptions, audit, builtin, compare, post_program, read_back};
use proptest::prelude::*;

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn path(blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some("raster".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(1000.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
}

fn program(blocks: Vec<ClBlock>) -> Program {
    Program::try_new(Some("laser-power".into()), Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new(), None).expect("program")
}

fn options() -> PostOptions {
    PostOptions { arc_tolerance: 0.01 }
}

fn feed(x: f64, y: f64, z: f64, power: Option<f64>) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: pose(x, y, z), feed: 1000.0, role: MoveRole::Cut, power })
}

fn raster(powers: &[(f64, f64, Option<f64>)]) -> Program {
    let mut blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 0.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Laser, params: ProcessParams::new() }),
    ];
    for (x, y, power) in powers {
        blocks.push(feed(*x, *y, 0.0, *power));
    }
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }));
    program(blocks)
}

fn motion_s_words(text: &str) -> Vec<i64> {
    let mut values = Vec::new();
    for line in text.lines() {
        let body = line.split('(').next().unwrap_or(line);
        let tokens: Vec<&str> = body.split_whitespace().collect();
        if tokens.iter().any(|word| *word == "G0" || word.starts_with('M')) {
            continue;
        }
        if !tokens.iter().any(|word| matches!(*word, "G1" | "G2" | "G3") || word.starts_with('X') || word.starts_with('Y')) {
            continue;
        }
        for word in tokens {
            if let Some(rest) = word.strip_prefix('S')
                && let Ok(value) = rest.parse::<i64>()
            {
                values.push(value);
            }
        }
    }
    values
}

fn post() -> campanion_post::PostDef {
    builtin("grbl_laser").expect("grbl_laser")
}

fn roundtrip(p: &Program) -> String {
    let post = post();
    let text = post_program(p, &post, &options()).expect("post");
    compare(p, &read_back(&text, &post).expect("read"), 1e-3, &post).expect("compare");
    audit(&text, &post).expect("audit");
    text
}

#[test]
fn vector_program_without_move_power_still_fills_missing_process_power() {
    let p = raster(&[(10.0, 0.0, None)]);
    let text = roundtrip(&p);
    assert!(text.contains("power missing, using S1000"), "{text}");
    assert!(text.contains("M4 S1000"), "{text}");
    assert!(motion_s_words(&text).is_empty(), "vector feeds must not grow an S word\n{text}");
}

#[test]
fn each_raster_segment_writes_its_scaled_s_including_s0_overscan() {
    let p = raster(&[(-1.0, 0.0, Some(0.0)), (5.0, 0.0, Some(0.4)), (11.0, 0.0, Some(0.0))]);
    let text = roundtrip(&p);
    assert!(!text.contains("power missing"), "{text}");
    assert!(text.contains("M4 S0"), "{text}");
    assert!(!text.contains("M4 S1000"), "{text}");
    assert_eq!(motion_s_words(&text), vec![0, 400, 0], "{text}");
}

#[test]
fn missing_move_power_in_a_raster_is_an_error_not_s1000() {
    let p = raster(&[(5.0, 0.0, Some(0.5)), (10.0, 0.0, None)]);
    let err = post_program(&p, &post(), &options()).expect_err("missing");
    let message = err.to_string();
    assert!(message.contains("move power is required"), "{message}");
    assert!(!message.contains("S1000"), "{message}");
}

#[test]
fn per_move_power_is_refused_without_power_scale() {
    let p = raster(&[(5.0, 0.0, Some(0.5))]);
    let mill = builtin("linuxcnc").expect("linuxcnc");
    let err = post_program(&p, &mill, &options()).expect_err("no scale");
    assert!(err.to_string().contains("power_scale"), "{err}");
}

#[test]
fn arc_feeds_carry_the_same_s_word() {
    let p = program(vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 0.0, 0.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Laser, params: ProcessParams::new() }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(0.0, 10.0, 0.0),
            plane: Plane::Xy,
            center: Vec3::ZERO,
            sense: ArcSense::Ccw,
            feed: 800.0,
            role: MoveRole::Cut,
            power: Some(0.25),
        }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }),
    ]);
    let text = roundtrip(&p);
    assert!(text.contains("G3"), "{text}");
    assert_eq!(motion_s_words(&text), vec![250], "{text}");
}

#[test]
fn process_power_is_used_when_present_and_moves_still_write_s() {
    let params = ProcessParams::new().power(Some(0.1)).expect("params");
    let p = program(vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 0.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Laser, params }),
        feed(4.0, 0.0, 0.0, Some(0.8)),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }),
    ]);
    let text = roundtrip(&p);
    assert!(text.contains("M4 S100"), "{text}");
    assert_eq!(motion_s_words(&text), vec![800], "{text}");
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]

    #[test]
    fn arbitrary_scan_powers_round_trip_as_s_words(ticks in proptest::collection::vec(0u16..=1000, 1..=12)) {
        let powers: Vec<(f64, f64, Option<f64>)> = ticks
            .iter()
            .enumerate()
            .map(|(i, tick)| (f64::from(u32::try_from(i + 1).expect("index")), 0.0, Some(f64::from(*tick) / 1000.0)))
            .collect();
        let p = raster(&powers);
        let text = roundtrip(&p);
        prop_assert!(!text.contains("power missing"), "{text}");
        prop_assert!(!text.contains("M4 S1000"), "{text}");
        prop_assert_eq!(motion_s_words(&text), ticks.iter().map(|tick| i64::from(*tick)).collect::<Vec<_>>());
    }
}
