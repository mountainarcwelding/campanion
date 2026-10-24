//! Short fitted arcs post without becoming a full circle, and read back within tolerance.

#![allow(clippy::expect_used)]

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, Program, Vec3};
use campanion_post::{PostOptions, ReadMove, builtin, builtin_ids, compare, post_program, read_back};
use proptest::prelude::*;

const TAU: f64 = std::f64::consts::TAU;
const ARC_TOLERANCE: f64 = 0.01;

#[derive(Clone, Debug)]
struct ArcCase {
    x: f64,
    y: f64,
    z: f64,
    radius: f64,
    sweep: f64,
    ccw: bool,
    feed: f64,
}

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn center_of(case: &ArcCase) -> [f64; 3] {
    [case.x - case.radius, case.y, case.z]
}

fn end_of(case: &ArcCase) -> [f64; 3] {
    let center = center_of(case);
    let signed = if case.ccw { case.sweep } else { -case.sweep };
    [center[0] + case.radius * signed.cos(), center[1] + case.radius * signed.sin(), case.z]
}

fn program_of(case: &ArcCase) -> Program {
    let end = end_of(case);
    let center = center_of(case);
    let sense = if case.ccw { ArcSense::Ccw } else { ArcSense::Cw };
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(case.x, case.y, case.z), feed: None }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(end[0], end[1], end[2]),
            plane: Plane::Xy,
            center: Vec3::new(center[0], center[1], center[2]).expect("center"),
            sense,
            feed: case.feed,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Linear { to: pose(end[0] + 5.0, end[1], end[2]), feed: case.feed, role: MoveRole::Cut }),
    ];
    let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
    let path = ClPath::try_new(Some("arc".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
    Program::try_new(Some("arc".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program")
}

fn arc_strategy() -> impl Strategy<Value = ArcCase> {
    let sweep_hi = (TAU - 1e-3).log10();
    (-2.0..2.0, -6.0..sweep_hi, -200.0..200.0, -200.0..200.0, -20.0..20.0, proptest::bool::ANY, 50.0..1500.0)
        .prop_map(|(u, v, x, y, z, ccw, feed)| ArcCase { x, y, z, radius: 10_f64.powf(u), sweep: 10_f64.powf(v), ccw, feed })
}

fn xy_len(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Controller sweep about `center` from `start` to `end`. Equal XY endpoints are one full turn.
fn directed_sweep(start: [f64; 3], end: [f64; 3], center: [f64; 3], ccw: bool) -> f64 {
    if xy_len(start, end) <= 1e-9 {
        return TAU;
    }
    let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
    let a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
    let delta = (a1 - a0).rem_euclid(TAU);
    if ccw { delta } else { (TAU - delta).rem_euclid(TAU) }
}

fn is_motion(move_: &ReadMove) -> bool {
    matches!(move_, ReadMove::Rapid { .. } | ReadMove::Linear { .. } | ReadMove::Arc { .. } | ReadMove::Thread { .. })
}

fn motion_end(move_: &ReadMove) -> [f64; 3] {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } | ReadMove::Thread { end, .. } => *end,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => [0.0, 0.0, 0.0],
    }
}

/// Sum of controller sweeps, radius words, and chord vertices stay inside the limits.
fn controller_ok(moves: &[ReadMove], case: &ArcCase, quantum: f64) -> Result<(), String> {
    let center = center_of(case);
    let motions: Vec<&ReadMove> = moves.iter().filter(|move_| is_motion(move_)).collect();
    let last = motions.len().saturating_sub(1);
    let mut pos: Option<[f64; 3]> = None;
    let mut arc_sum = 0.0;
    for (index, move_) in motions.iter().enumerate() {
        let end = motion_end(move_);
        match move_ {
            ReadMove::Arc { sense, center: read_center, .. } => {
                let start = pos.ok_or_else(|| "arc has no start".to_string())?;
                let ccw = matches!(sense, ArcSense::Ccw);
                arc_sum += directed_sweep(start, end, *read_center, ccw);
                let start_radius = xy_len(start, *read_center);
                let end_radius = xy_len(end, *read_center);
                if (start_radius - case.radius).abs() > 2.0 * quantum {
                    return Err(format!("start radius {start_radius} vs {}", case.radius));
                }
                if (end_radius - case.radius).abs() > 2.0 * quantum {
                    return Err(format!("end radius {end_radius} vs {}", case.radius));
                }
            }
            ReadMove::Linear { .. } if index > 0 && index < last => {
                let radial = (xy_len(end, center) - case.radius).abs();
                if radial > quantum + ARC_TOLERANCE {
                    return Err(format!("linear radial error {radial}"));
                }
            }
            _ => {}
        }
        pos = Some(end);
    }
    let limit = case.sweep + 4.0 * quantum / case.radius + 1e-3;
    if arc_sum > limit {
        return Err(format!("controller sweep {arc_sum} > {limit}"));
    }
    Ok(())
}

fn has_g2(text: &str) -> bool {
    text.lines().any(|line| {
        line.split_whitespace().any(|word| {
            let word = word.trim_end_matches('.');
            word.eq_ignore_ascii_case("G2") || word.eq_ignore_ascii_case("G02")
        })
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn short_arcs_post_and_read_back(case in arc_strategy()) {
        let program = program_of(&case);
        let options = PostOptions { arc_tolerance: ARC_TOLERANCE };
        for id in builtin_ids() {
            let post = builtin(id).expect(id);
            if post.is_lathe() {
                continue;
            }
            let quantum = 10_f64.powi(-(post.linear_decimals() as i32));
            match post_program(&program, &post, &options) {
                Err(err) => {
                    let message = err.to_string();
                    prop_assert!(message.contains("arc end rounds onto its start"), "{id}: {message}");
                }
                Ok(text) => {
                    let moves = read_back(&text, &post).map_err(|err| format!("{id} read: {err}")).expect("read");
                    compare(&program, &moves, quantum, &post).map_err(|err| format!("{id} compare: {err}\n{text}")).expect("compare");
                    controller_ok(&moves, &case, quantum).map_err(|err| format!("{id} controller: {err}\n{text}")).expect("controller");
                }
            }
        }
    }
}

#[test]
fn zero_chord_arc_is_a_feed_line() {
    let case = ArcCase { x: 1.23456, y: 2.0, z: 0.0, radius: 0.5, sweep: 1e-5, ccw: false, feed: 100.0 };
    let program = program_of(&case);
    let options = PostOptions { arc_tolerance: ARC_TOLERANCE };
    for id in ["linuxcnc", "haas", "grbl"] {
        let post = builtin(id).expect(id);
        let text = post_program(&program, &post, &options).map_err(|err| format!("{id}: {err}")).expect("post");
        assert!(!has_g2(&text), "{id}\n{text}");
        let moves = read_back(&text, &post).map_err(|err| format!("{id} read: {err}")).expect("read");
        let quantum = 10_f64.powi(-(post.linear_decimals() as i32));
        compare(&program, &moves, quantum, &post).map_err(|err| format!("{id} compare: {err}\n{text}")).expect("compare");
    }
}

#[test]
fn fitted_short_arcs_compare_on_every_mill_post() {
    let options = PostOptions { arc_tolerance: ARC_TOLERANCE };
    for sweep in [0.0071_f64, 0.0301] {
        let case = ArcCase { x: 1.23456, y: 2.0, z: 0.0, radius: 0.5, sweep, ccw: false, feed: 100.0 };
        let program = program_of(&case);
        for id in builtin_ids() {
            let post = builtin(id).expect(id);
            if post.is_lathe() {
                continue;
            }
            let text = post_program(&program, &post, &options).map_err(|err| format!("{id} sweep {sweep}: {err}")).expect("post");
            let moves = read_back(&text, &post).map_err(|err| format!("{id} read: {err}")).expect("read");
            let quantum = 10_f64.powi(-(post.linear_decimals() as i32));
            compare(&program, &moves, quantum, &post).map_err(|err| format!("{id} sweep {sweep} compare: {err}\n{text}")).expect("compare");
        }
    }
}
