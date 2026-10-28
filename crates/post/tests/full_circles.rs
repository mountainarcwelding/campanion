//! A closed arc whose end lies within a full-turn chord of its start posts as one full circle
//! on every mill post, and read-back accepts that circle.

#![allow(clippy::expect_used)]

use campanion_geom::{Contour, Point, Polyline, Region, Tolerance, Vertex};
use campanion_import::{ImportOptions, import_svg};
use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, ProcessKind, Program, Vec3};
use campanion_path::{Compensation, CutDirection, CutKind, Move, PocketOp, PocketStrategy, Point3, ProfileOp, ProfileSide, Tool, Toolpath};
use campanion_post::{PostOptions, ReadMove, builtin, builtin_ids, compare, post_program, read_back};
use proptest::prelude::*;

const TAU: f64 = std::f64::consts::TAU;
/// Job chord tolerance. Read-back of these programs uses the same limit.
const TOL: f64 = 0.01;
const TOOL_RADIUS: f64 = 0.01;

#[derive(Clone, Copy, Debug)]
struct FullCircle {
    cx: f64,
    cy: f64,
    radius: f64,
}

fn pose(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn circle_contour(cx: f64, cy: f64, radius: f64) -> Contour {
    let right = Point::new(cx + radius, cy).expect("right");
    let left = Point::new(cx - radius, cy).expect("left");
    Polyline::new(vec![Vertex::new(right, 1.0).expect("vertex"), Vertex::new(left, 1.0).expect("vertex")], true)
        .expect("polyline")
        .to_contour(&Tolerance::default())
        .expect("contour")
}

fn program_from(path: &Toolpath) -> Program {
    let import = campanion_ir::Import::new(ProcessKind::Spindle, 1500.0).expect("import");
    campanion_ir::Document::from_toolpath(path, &import).expect("document").program().clone()
}

fn circle_program(start: [f64; 3], end: [f64; 3], centre: [f64; 3], ccw: bool) -> Program {
    let sense = if ccw { ArcSense::Ccw } else { ArcSense::Cw };
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(start[0], start[1], start[2] + 5.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(start[0], start[1], start[2]), feed: 500.0, role: MoveRole::Plunge }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(end[0], end[1], end[2]),
            plane: Plane::Xy,
            center: Vec3::new(centre[0], centre[1], centre[2]).expect("centre"),
            sense,
            feed: 1500.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Rapid { to: pose(end[0], end[1], end[2] + 5.0), feed: None }),
    ];
    let feeds = Feeds::try_new(1500.0, None, None, None, None).expect("feeds");
    let path = ClPath::try_new(Some("circle".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
    Program::try_new(Some("circle".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program")
}

/// Nudge every closed arc end by `(dx, dy)`. A producer that snapped the end onto the start
/// still reaches the post with a gap inside [`campanion_geom::Arc::full_turn_chord`].
fn perturb_closed(path: &Toolpath, dx: f64, dy: f64) -> Toolpath {
    let mut cursor: Option<Point3> = None;
    let mut moves = Vec::new();
    for mv in path.moves() {
        match *mv {
            Move::Arc { to, centre, ccw, kind } => {
                let from = cursor.expect("arc start");
                let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                let chord = (to.x() - from.x()).hypot(to.y() - from.y());
                let to = if chord <= campanion_geom::Arc::full_turn_chord(radius) {
                    Point3::new(from.x() + dx, from.y() + dy, to.z()).expect("nudged end")
                } else {
                    to
                };
                moves.push(Move::Arc { to, centre, ccw, kind });
                cursor = Some(to);
            }
            Move::Rapid { to } | Move::Linear { to, .. } => {
                moves.push(*mv);
                cursor = Some(to);
            }
            Move::Event(_) | Move::Dwell { .. } => moves.push(*mv),
        }
    }
    Toolpath::new(moves).expect("toolpath")
}

fn flip_closed_sense(path: &Toolpath) -> Toolpath {
    let mut cursor: Option<Point3> = None;
    let mut moves = Vec::new();
    for mv in path.moves() {
        match *mv {
            Move::Arc { to, centre, ccw, kind } => {
                let from = cursor.expect("arc start");
                let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                let chord = (to.x() - from.x()).hypot(to.y() - from.y());
                let ccw = if chord <= campanion_geom::Arc::full_turn_chord(radius) { !ccw } else { ccw };
                moves.push(Move::Arc { to, centre, ccw, kind });
                cursor = Some(to);
            }
            Move::Rapid { to } | Move::Linear { to, .. } => {
                moves.push(*mv);
                cursor = Some(to);
            }
            Move::Event(_) | Move::Dwell { .. } => moves.push(*mv),
        }
    }
    Toolpath::new(moves).expect("toolpath")
}

fn closed_circles(path: &Toolpath) -> Vec<FullCircle> {
    let mut cursor: Option<Point3> = None;
    let mut circles = Vec::new();
    for mv in path.moves() {
        match *mv {
            Move::Arc { to, centre, kind: CutKind::Cut, .. } => {
                let from = cursor.expect("arc start");
                let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                let chord = (to.x() - from.x()).hypot(to.y() - from.y());
                if chord <= campanion_geom::Arc::full_turn_chord(radius) {
                    circles.push(FullCircle { cx: centre.x(), cy: centre.y(), radius });
                }
                cursor = Some(to);
            }
            Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(to),
            Move::Event(_) | Move::Dwell { .. } => {}
        }
    }
    circles
}

fn pocket_disk(cx: f64, cy: f64, radius: f64) -> Toolpath {
    let contour = circle_contour(cx, cy, radius + TOOL_RADIUS);
    let region = Region::from_contour(contour, &Tolerance::default()).expect("region");
    let tool = Tool::Diameter { diameter: TOOL_RADIUS * 2.0 };
    PocketOp::new(tool, PocketStrategy::OffsetRings, radius + 1.0).z(-1.0).retract_z(1.0).generate(&region).expect("pocket").toolpath().clone()
}

fn inside_profile(cx: f64, cy: f64, radius: f64, direction: CutDirection) -> Toolpath {
    let contour = circle_contour(cx, cy, radius + TOOL_RADIUS);
    ProfileOp::new(Tool::Diameter { diameter: TOOL_RADIUS * 2.0 }, Compensation::Computed, ProfileSide::Inside, direction)
        .z(-1.0)
        .generate(&contour)
        .expect("profile")
        .toolpath()
        .clone()
}

fn xy_len(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn motion_end(move_: &ReadMove) -> Option<[f64; 3]> {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } | ReadMove::Thread { end, .. } => Some(*end),
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
    }
}

fn directed_sweep(start: [f64; 3], end: [f64; 3], center: [f64; 3], ccw: bool) -> f64 {
    if xy_len(start, end) <= 1e-9 {
        return TAU;
    }
    let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
    let a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
    let delta = (a1 - a0).rem_euclid(TAU);
    if ccw { delta } else { (TAU - delta).rem_euclid(TAU) }
}

fn on_circle(center: [f64; 3], radius: f64, circle: &FullCircle, quantum: f64) -> bool {
    let centre_tol = TOL.max(4.0 * quantum);
    (center[0] - circle.cx).hypot(center[1] - circle.cy) <= centre_tol && (radius - circle.radius).abs() <= TOL
}

/// Posted arc samples stay on the circle and the controller sweep is one full turn.
///
/// Equal posted endpoints count as τ. The allowance is a few quanta of angle, capped so a
/// missing half turn still fails.
fn trace_circles(moves: &[ReadMove], circles: &[FullCircle], quantum: f64) -> Result<(), String> {
    if circles.is_empty() {
        return Err("toolpath has no closed circle".into());
    }
    let mut swept = vec![0.0; circles.len()];
    let mut pos: Option<[f64; 3]> = None;
    for move_ in moves {
        let Some(end) = motion_end(move_) else {
            continue;
        };
        if let ReadMove::Arc { sense, center, .. } = move_ {
            let start = pos.ok_or_else(|| "arc has no start".to_string())?;
            let read_radius = xy_len(start, *center);
            if let Some((index, circle)) = circles.iter().enumerate().find(|(_, circle)| on_circle(*center, read_radius, circle, quantum)) {
                let ccw = matches!(sense, ArcSense::Ccw);
                let sweep = directed_sweep(start, end, *center, ccw);
                let steps = 8;
                for step in 0..=steps {
                    let theta = sweep * f64::from(step) / f64::from(steps);
                    let signed = if ccw { theta } else { -theta };
                    let (sin_a, cos_a) = signed.sin_cos();
                    let dx = start[0] - center[0];
                    let dy = start[1] - center[1];
                    let x = center[0] + dx * cos_a - dy * sin_a;
                    let y = center[1] + dx * sin_a + dy * cos_a;
                    let radial = (x - circle.cx).hypot(y - circle.cy);
                    if (radial - circle.radius).abs() > TOL {
                        return Err(format!("sample radial error {}", (radial - circle.radius).abs()));
                    }
                }
                swept[index] += sweep;
            }
        }
        pos = Some(end);
    }
    for (index, circle) in circles.iter().enumerate() {
        let allowance = (8.0 * quantum / circle.radius).clamp(1e-3, 0.2);
        let got = swept[index];
        if (got - TAU).abs() > allowance {
            return Err(format!("circle {index} swept {got}, want τ ± {allowance}"));
        }
    }
    Ok(())
}

fn is_arc_line(line: &str) -> bool {
    line.split_whitespace().any(|word| {
        let word = word.trim_end_matches('.');
        matches!(word, "G2" | "G02" | "G3" | "G03")
    })
}

fn word_letter(line: &str, letter: char) -> bool {
    line.split_whitespace().any(|word| {
        let mut chars = word.chars();
        chars.next().is_some_and(|got| got.eq_ignore_ascii_case(&letter)) && chars.next().is_some()
    })
}

fn equal_endpoint_arcs(moves: &[ReadMove]) -> usize {
    let mut count = 0usize;
    let mut pos: Option<[f64; 3]> = None;
    for move_ in moves {
        let Some(end) = motion_end(move_) else {
            continue;
        };
        if let ReadMove::Arc { .. } = move_
            && let Some(start) = pos
            && xy_len(start, end) <= 1e-9
        {
            count += 1;
        }
        pos = Some(end);
    }
    count
}

/// I/J posts write one equal-endpoint full circle per ring. An R post splits the turn and
/// does not write an equal-endpoint arc.
fn check_circle_blocks(text: &str, moves: &[ReadMove], circles: &[FullCircle]) -> Result<(), String> {
    let arcs: Vec<&str> = text.lines().filter(|line| is_arc_line(line)).collect();
    if arcs.is_empty() {
        return Err("posted no arc".into());
    }
    let radius_mode = arcs.iter().any(|line| word_letter(line, 'R'));
    let equal = equal_endpoint_arcs(moves);
    if radius_mode {
        if equal != 0 {
            return Err(format!("R post wrote {equal} equal-endpoint arcs"));
        }
        if arcs.len() < circles.len() * 2 {
            return Err(format!("R post wrote {} arc blocks for {} circles", arcs.len(), circles.len()));
        }
    } else if equal < circles.len() {
        return Err(format!("IJK post wrote {equal} full-circle blocks for {} circles", circles.len()));
    }
    Ok(())
}

fn post_every_mill(program: &Program, circles: &[FullCircle]) -> Result<(), String> {
    let options = PostOptions { arc_tolerance: TOL };
    for id in builtin_ids() {
        let post = builtin(id).ok_or_else(|| format!("missing post {id}"))?;
        if post.is_lathe() {
            continue;
        }
        let text = match post_program(program, &post, &options) {
            Ok(text) => text,
            Err(err) => {
                if *id == "plasma" || *id == "grbl_laser" {
                    continue;
                }
                return Err(format!("{id} post: {err}"));
            }
        };
        let moves = read_back(&text, &post).map_err(|err| format!("{id} read: {err}\n{text}"))?;
        compare(program, &moves, TOL, &post).map_err(|err| format!("{id} compare: {err}\n{text}"))?;
        let quantum = 10_f64.powi(-(i32::from(post.linear_decimals())));
        trace_circles(&moves, circles, quantum).map_err(|err| format!("{id} trace: {err}\n{text}"))?;
        check_circle_blocks(&text, &moves, circles).map_err(|err| format!("{id} blocks: {err}\n{text}"))?;
    }
    Ok(())
}

fn post_path(path: &Toolpath) -> Result<(), String> {
    let circles = closed_circles(path);
    if circles.is_empty() {
        return Err("expected a closed arc".into());
    }
    post_every_mill(&program_from(path), &circles)
}

const RING_SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" width="80mm" height="80mm" viewBox="0 0 80 80"><g id="RING"><circle cx="40" cy="39.94157" r="30"/><circle cx="40" cy="39.94157" r="15"/></g></svg>"#;

fn ring_pocket() -> Toolpath {
    let drawing = import_svg(RING_SVG, &ImportOptions::new(Tolerance::default())).expect("svg");
    let mut rings: Vec<Contour> = drawing.contours().iter().filter(|item| item.layer() == "RING").map(|item| item.contour().clone()).collect();
    assert_eq!(rings.len(), 2, "RING contours");
    rings.sort_by(|left, right| {
        let left_width = left.bounding_box().expect("box").width();
        let right_width = right.bounding_box().expect("box").width();
        right_width.partial_cmp(&left_width).expect("width")
    });
    let outer = rings.first().expect("outer").clone();
    let hole = rings.get(1).expect("hole").clone();
    let mid_y = {
        let bounds = outer.bounding_box().expect("box");
        (bounds.min().y() + bounds.max().y()) * 0.5
    };
    assert!((mid_y - 40.05843).abs() < 1e-4, "flipped centre y {mid_y}");
    let region = Region::try_new(outer, vec![hole], &Tolerance::default()).expect("region");
    PocketOp::new(Tool::Diameter { diameter: 6.0 }, PocketStrategy::OffsetRings, 2.0)
        .z(-3.0)
        .retract_z(2.0)
        .generate(&region)
        .expect("pocket")
        .toolpath()
        .clone()
}

#[test]
fn reported_ring_posts_on_every_mill() {
    let path = ring_pocket();
    let circles = closed_circles(&path);
    assert!(circles.iter().any(|circle| (circle.radius - 27.0).abs() < 0.05), "outer ring {circles:?}");
    assert!(circles.iter().any(|circle| (circle.radius - 18.0).abs() < 0.05), "island ring {circles:?}");
    let cw = path.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw: false, kind: CutKind::Cut, .. }));
    let ccw = path.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw: true, kind: CutKind::Cut, .. }));
    assert!(cw && ccw, "annulus pocket has both windings");
    post_path(&path).expect("posted ring");
    // The same rings with the end a nanometre off the start, which is still one full turn.
    post_path(&perturb_closed(&path, 0.0, -1e-9)).expect("posted drifted ring");
}

#[test]
fn inside_circle_profile_posts_on_every_mill() {
    let contour = circle_contour(15.0, 30.0, 5.0);
    for direction in [CutDirection::Climb, CutDirection::Conventional] {
        let path = ProfileOp::new(Tool::Diameter { diameter: 6.0 }, Compensation::Computed, ProfileSide::Inside, direction)
            .z(-1.0)
            .generate(&contour)
            .expect("profile")
            .toolpath()
            .clone();
        let circles = closed_circles(&path);
        assert!(
            circles.iter().any(|circle| (circle.radius - 2.0).abs() < 0.05 && (circle.cx - 15.0).abs() < 1e-6 && (circle.cy - 30.0).abs() < 1e-6),
            "{circles:?}"
        );
        let want_ccw = direction == CutDirection::Climb;
        assert!(path.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw, kind: CutKind::Cut, .. } if *ccw == want_ccw)));
        post_every_mill(&program_from(&path), &circles).expect("profile");
    }
}

#[test]
fn equal_endpoint_circle_at_the_step_start_posts_on_every_mill() {
    let start = [16.6357, 31.1508, -1.0];
    let centre = [15.0, 30.0, -1.0];
    let radius = xy_len(start, centre);
    for ccw in [true, false] {
        for (dx, dy) in [(0.0, 0.0), (0.0, -1e-9), (1e-9, 0.0)] {
            let end = [start[0] + dx, start[1] + dy, start[2]];
            let program = circle_program(start, end, centre, ccw);
            post_every_mill(&program, &[FullCircle { cx: centre[0], cy: centre[1], radius }]).expect("step circle");
        }
    }
}

#[test]
fn off_grid_full_circle_posts_both_ways() {
    let y = 40.05843;
    let radius = 18.0;
    let centre = [40.0, y, -3.0];
    let start = [centre[0] + radius, centre[1], centre[2]];
    // `sin(τ)` leaves the analytic end a fraction of a nanometre off the start.
    let analytic = [centre[0] + radius * TAU.cos(), centre[1] + radius * TAU.sin(), centre[2]];
    let drifts = [analytic, [start[0], start[1] - 1e-9, start[2]], [start[0], start[1] + 1e-9, start[2]]];
    for end in drifts {
        for ccw in [true, false] {
            let program = circle_program(start, end, centre, ccw);
            post_every_mill(&program, &[FullCircle { cx: centre[0], cy: centre[1], radius }]).expect("off-grid circle");
        }
    }
}

#[test]
fn full_circle_across_a_rounding_boundary_posts() {
    // Haas writes three decimals. These ends sit on either side of the 10.0005 boundary and
    // still within a full-turn chord, so each would round to a different Y on its own.
    let radius = 18.0;
    let y0 = 10.0005 - 1e-9;
    let centre = [40.0, y0, -1.0];
    let start = [centre[0] + radius, y0, -1.0];
    let end = [centre[0] + radius, 10.0005 + 1e-9, -1.0];
    assert!(xy_len(start, end) <= campanion_geom::Arc::full_turn_chord(radius));
    for ccw in [true, false] {
        let program = circle_program(start, end, centre, ccw);
        post_every_mill(&program, &[FullCircle { cx: centre[0], cy: centre[1], radius }]).expect("boundary circle");
    }
}

#[test]
fn trace_rejects_a_missing_half_and_a_near_full_gap() {
    let circle = FullCircle { cx: 0.0, cy: 0.0, radius: 10.0 };
    let rapid = |end: [f64; 3]| ReadMove::Rapid { end, feed: None, rotary: None, inverse_time: false };
    let arc = |end: [f64; 3]| ReadMove::Arc {
        sense: ArcSense::Ccw,
        end,
        center: [0.0, 0.0, 0.0],
        radius: None,
        feed: Some(100.0),
        rotary: None,
        inverse_time: false,
    };
    let half = vec![rapid([10.0, 0.0, 0.0]), arc([-10.0, 0.0, 0.0])];
    let err = trace_circles(&half, &[circle], 1e-4).expect_err("half");
    assert!(err.contains("swept"), "{err}");
    let gap = TAU - 0.5;
    let near = vec![rapid([10.0, 0.0, 0.0]), arc([10.0 * gap.cos(), 10.0 * gap.sin(), 0.0])];
    let err = trace_circles(&near, &[circle], 1e-4).expect_err("near full");
    assert!(err.contains("swept"), "{err}");
    let turns = vec![rapid([10.0, 0.0, 0.0]), arc([-10.0, 0.0, 0.0]), arc([10.0, 0.0, 0.0])];
    trace_circles(&turns, &[circle], 1e-4).expect("two semicircles");
}

fn off_grid(unit: f64, negative: bool) -> f64 {
    let magnitude = unit * 1_000.0 + 0.058430123;
    if negative { -magnitude } else { magnitude }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 16, ..ProptestConfig::default() })]

    #[test]
    fn random_full_circles_post_on_every_mill(
        ux in 0.0..1.0,
        uy in 0.0..1.0,
        neg_x in proptest::bool::ANY,
        neg_y in proptest::bool::ANY,
        radius in 0.05..200.0,
        scale in 0.0..=1.0,
        angle in 0.0..TAU,
    ) {
        let cx = off_grid(ux, neg_x);
        let cy = off_grid(uy, neg_y);
        let radius: f64 = radius;
        let len = scale * 1e-9 * radius.max(1.0);
        let dx = len * angle.cos();
        let dy = len * angle.sin();
        let disk = perturb_closed(&pocket_disk(cx, cy, radius), dx, dy);
        let disk_ccw = disk.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw: true, kind: CutKind::Cut, .. }));
        prop_assert!(disk_ccw);
        post_path(&disk).map_err(TestCaseError::fail)?;
        post_path(&flip_closed_sense(&disk)).map_err(TestCaseError::fail)?;
        let climb = perturb_closed(&inside_profile(cx, cy, radius, CutDirection::Climb), dx, dy);
        let climb_ccw = climb.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw: true, kind: CutKind::Cut, .. }));
        prop_assert!(climb_ccw);
        post_path(&climb).map_err(TestCaseError::fail)?;
        let conventional = perturb_closed(&inside_profile(cx, cy, radius, CutDirection::Conventional), dx, dy);
        let conventional_cw = conventional.moves().iter().any(|mv| matches!(mv, Move::Arc { ccw: false, kind: CutKind::Cut, .. }));
        prop_assert!(conventional_cw);
        post_path(&conventional).map_err(TestCaseError::fail)?;
    }
}
