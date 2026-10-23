//! Analytic checks and gouge properties for facing, roughing, and finishing.

use campanion_geom::{Arc, Line, Point, Segment};
use proptest::prelude::*;

use super::{FaceParams, FinishParams, RoughParams, Side, TipPoint, TurnMove, TurnPath, TurnTool, compensated_profile, face, finish, rough};
use crate::{TurnError, TurnProfile};

fn pt(z: f64, x: f64) -> Point {
    Point::new(z, x).expect("point")
}

fn line(z0: f64, x0: f64, z1: f64, x1: f64) -> Segment {
    Segment::Line(Line::new(pt(z0, x0), pt(z1, x1)).expect("line"))
}

fn profile(segments: Vec<Segment>) -> TurnProfile {
    TurnProfile::try_new(segments, 1e-9).expect("profile")
}

fn centre(nose: f64) -> TurnTool {
    TurnTool { nose_radius: nose, tip: TipPoint::Centre }
}

fn od_tip(nose: f64) -> TurnTool {
    TurnTool { nose_radius: nose, tip: TipPoint::ImaginaryTip { dz: -nose, dx: -nose } }
}

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!((actual - expected).abs() <= tol, "{label}: {actual} != {expected} (tol {tol})");
}

fn target(from: (f64, f64), motion: &TurnMove) -> (f64, f64) {
    match *motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } | TurnMove::Thread { z, x, .. } => (z, x),
        TurnMove::Dwell { .. } => from,
    }
}

fn distance_to_profile(profile: &TurnProfile, z: f64, x: f64) -> f64 {
    let point = Point::new(z, x).expect("sample");
    profile.segments().iter().fold(f64::INFINITY, |best, segment| best.min(segment.distance_to_point(point)))
}

fn sample_move(from: (f64, f64), motion: &TurnMove, n: usize) -> Vec<(f64, f64)> {
    let n = n.max(1);
    match *motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Thread { z, x, .. } => (1..=n)
            .map(|i| {
                let t = i as f64 / n as f64;
                (from.0 + t * (z - from.0), from.1 + t * (x - from.1))
            })
            .collect(),
        TurnMove::Arc { z, x, cz, cx, ccw } => sample_arc(from, (z, x), (cz, cx), ccw, n),
        TurnMove::Dwell { .. } => Vec::new(),
    }
}

fn sample_arc(from: (f64, f64), to: (f64, f64), centre: (f64, f64), ccw: bool, n: usize) -> Vec<(f64, f64)> {
    const TAU: f64 = std::f64::consts::TAU;
    if from.0.hypot(from.1) == 0.0 && (from.0 - to.0).hypot(from.1 - to.1) < 1e-12 {
        return vec![to];
    }
    if (from.0 - to.0).hypot(from.1 - to.1) < 1e-12 {
        return vec![to];
    }
    let radius = (from.0 - centre.0).hypot(from.1 - centre.1);
    let a0 = (from.1 - centre.1).atan2(from.0 - centre.0);
    let a1 = (to.1 - centre.1).atan2(to.0 - centre.0);
    let sweep = if ccw { (a1 - a0).rem_euclid(TAU) } else { -((a0 - a1).rem_euclid(TAU)) };
    (1..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let angle = a0 + sweep * t;
            (centre.0 + radius * angle.cos(), centre.1 + radius * angle.sin())
        })
        .collect()
}

fn cylinder(radius: f64, length: f64) -> TurnProfile {
    profile(vec![line(0.0, radius, -length, radius)])
}

/// Front face from the axis up to `radius`, then a cylinder of `length`.
fn faced_cylinder(radius: f64, length: f64) -> TurnProfile {
    profile(vec![line(0.0, 0.0, 0.0, radius), line(0.0, radius, -length, radius)])
}

fn finish_of(part: &TurnProfile, tool: &TurnTool, side: Side) -> TurnPath {
    finish(part, tool, &FinishParams { side, leave_x: 0.0, leave_z: 0.0, clearance: 0.5, tol: 1e-6 }).expect("finish")
}

/// Straight feeds: constant radius, changing Z. Returns `(z_start, z_end, x)`.
fn axial_feeds(path: &TurnPath) -> Vec<(f64, f64, f64)> {
    let mut cursor = path.start;
    let mut found = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion
            && (x - cursor.1).abs() <= 1e-6
            && (z - cursor.0).abs() > 1e-6
        {
            found.push((cursor.0, z, x));
        }
        cursor = target(cursor, motion);
    }
    found
}

fn radial_feeds(path: &TurnPath) -> Vec<(f64, f64, f64)> {
    let mut cursor = path.start;
    let mut found = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion
            && (z - cursor.0).abs() <= 1e-6
            && (x - cursor.1).abs() > 1e-6
        {
            found.push((z, cursor.1, x));
        }
        cursor = target(cursor, motion);
    }
    found
}

#[test]
fn cylinder_finish_centre_and_imaginary_tip() {
    let nose = 0.8;
    let part = cylinder(20.0, 30.0);
    let finished = finish_of(&part, &centre(nose), Side::Outside);
    let straights: Vec<f64> = axial_feeds(&finished).into_iter().map(|(_, _, x)| x).filter(|x| (*x - 20.8).abs() < 1e-6).collect();
    assert!(!straights.is_empty(), "centre finish has no Ø40 offset line: {:?}", finished.moves);
    for x in straights {
        assert_close(x, 20.8, 1e-9, "centre straight");
    }

    // Imaginary OD tip (−Z, −X). The straight is the part radius; a front face is exact in Z.
    let faced = faced_cylinder(20.0, 30.0);
    let chain = compensated_profile(&faced, &od_tip(nose), Side::Outside, 0.0, 0.0, 1e-6).expect("chain");
    let face_segs: Vec<_> = chain
        .iter()
        .copied()
        .filter(|segment| (segment.start().x() - segment.end().x()).abs() <= 1e-6 && (segment.start().x() - 0.0).abs() <= 1e-6)
        .collect();
    assert!(!face_segs.is_empty(), "imaginary tip face is not at z = 0: {chain:?}");
    let od_segs: Vec<_> = chain
        .iter()
        .copied()
        .filter(|segment| {
            (segment.start().y() - 20.0).abs() <= 1e-6
                && (segment.end().y() - 20.0).abs() <= 1e-6
                && (segment.end().x() - segment.start().x()).abs() > 1.0
        })
        .collect();
    assert!(!od_segs.is_empty(), "imaginary tip OD is not at x = 20: {chain:?}");

    let tipped = finish_of(&faced, &od_tip(nose), Side::Outside);
    let od = axial_feeds(&tipped).into_iter().find(|(z0, z1, x)| (*x - 20.0).abs() < 1e-6 && (*z0 - *z1).abs() > 10.0);
    assert!(od.is_some(), "finish straight missing: {:?}", tipped.moves);
    let face_feed = radial_feeds(&tipped).into_iter().find(|(z, _, _)| (*z - 0.0).abs() < 1e-6);
    assert!(face_feed.is_some(), "finish face missing: {:?}", tipped.moves);
}

#[test]
fn chamfer_and_shoulder_offset_the_convex_corner() {
    let nose = 0.8;
    // 45° chamfer from (0, 10) to (−5, 15), land, 90° shoulder up to 25, then the back land.
    let part =
        profile(vec![line(0.0, 10.0, -5.0, 15.0), line(-5.0, 15.0, -20.0, 15.0), line(-20.0, 15.0, -20.0, 25.0), line(-20.0, 25.0, -40.0, 25.0)]);
    let finished = finish_of(&part, &centre(nose), Side::Outside);
    let face_z = -20.0 + nose;
    let shoulder = radial_feeds(&finished).into_iter().find(|(z, y0, y1)| (*z - face_z).abs() < 1e-5 && (*y1 - *y0).abs() > 1.0);
    assert!(shoulder.is_some(), "shoulder not at z = face + nose: {:?}", finished.moves);

    let mut corner_arcs = 0;
    for motion in &finished.moves {
        if let TurnMove::Arc { z, x, cz, cx, ccw } = *motion {
            let radius = (z - cz).hypot(x - cx);
            if (cz - (-20.0)).abs() < 1e-5 && (cx - 25.0).abs() < 1e-5 {
                assert_close(radius, nose, 1e-5, "outer corner radius");
                assert!(ccw, "convex OD corner walks counter-clockwise");
                corner_arcs += 1;
            }
            // The inside shoulder corner stays sharp: no arc about (−20, 15).
            assert!((cz - (-20.0)).abs() > 1e-3 || (cx - 15.0).abs() > 1e-3, "inside corner was filleted");
        }
    }
    assert_eq!(corner_arcs, 1, "missing arc about the sharp outer corner");

    let mut cursor = finished.start;
    let mut saw_chamfer = false;
    for motion in &finished.moves {
        if let TurnMove::Feed { z, x } = *motion {
            let dz = z - cursor.0;
            let dx = x - cursor.1;
            if dz.abs() > 1.0 && dx.abs() > 1.0 {
                let slope = dx / dz;
                if (slope + 1.0).abs() < 1e-5 {
                    saw_chamfer = true;
                }
            }
        }
        cursor = target(cursor, motion);
    }
    assert!(saw_chamfer, "45° chamfer was not kept: {:?}", finished.moves);
}

#[test]
fn concave_fillet_shrinks_by_the_nose_radius() {
    let nose = 0.8;
    let centre_z = -15.0;
    let centre_x = 15.0;
    let fillet_r = 3.0;
    // Clockwise through the bottom: the centre sits on the free side, so the fillet is concave.
    let fillet = Arc::new(pt(centre_z, centre_x), fillet_r, 0.0, -std::f64::consts::PI).expect("fillet");
    let start = fillet.start_point();
    let end = fillet.end_point();
    let part = profile(vec![line(0.0, centre_x, start.x(), start.y()), Segment::Arc(fillet), line(end.x(), end.y(), -30.0, centre_x)]);
    let finished = finish_of(&part, &centre(nose), Side::Outside);
    let mut matched = false;
    for motion in &finished.moves {
        if let TurnMove::Arc { z, x, cz, cx, ccw } = *motion {
            let radius = (z - cz).hypot(x - cx);
            if (cz - centre_z).abs() < 1e-4 && (cx - centre_x).abs() < 1e-4 {
                assert_close(radius, fillet_r - nose, 1e-4, "concave radius");
                assert!(!ccw, "the fillet keeps its clockwise sense");
                matched = true;
            }
        }
    }
    assert!(matched, "fillet arc missing: {:?}", finished.moves);
}

#[test]
fn rough_shoulder_follows_the_boundary_up_to_the_previous_pass() {
    let part = profile(vec![line(0.0, 10.0, -20.0, 10.0), line(-20.0, 10.0, -20.0, 20.0), line(-20.0, 20.0, -40.0, 20.0)]);
    let path = rough(
        &part,
        &centre(0.8),
        &RoughParams {
            side: Side::Outside,
            stock_outer_radius: 24.0,
            stock_inner_radius: 0.0,
            depth_of_cut: 5.0,
            leave_x: 0.0,
            leave_z: 0.0,
            clearance: 1.0,
            retract: 0.5,
        },
    )
    .expect("rough shoulder");
    // Compensated shoulder is 0.8 mm in front of z = −20. A pass between the two
    // diameters must feed up that face instead of retracting on a staircase.
    let climbs: Vec<(f64, f64, f64)> = radial_feeds(&path).into_iter().filter(|(z, y0, y1)| (*z - (-19.2)).abs() < 1e-3 && *y1 > *y0 + 0.5).collect();
    assert!(!climbs.is_empty(), "no climb up the shoulder: {:?}", path.moves);
    for (z, _, _) in &climbs {
        assert_close(*z, -19.2, 1e-3, "shoulder follow z");
    }
}

#[test]
fn rough_pass_depths_are_equal() {
    // Centre nose 0.8 on a Ø38.4 cylinder puts the programmed straight at radius 20.
    let part = cylinder(19.2, 30.0);
    let tool = centre(0.8);
    let path = rough(
        &part,
        &tool,
        &RoughParams {
            side: Side::Outside,
            stock_outer_radius: 25.0,
            stock_inner_radius: 0.0,
            depth_of_cut: 2.0,
            leave_x: 0.0,
            leave_z: 0.0,
            clearance: 1.0,
            retract: 0.4,
        },
    )
    .expect("rough");
    let feeds = axial_feeds(&path);
    let long: Vec<f64> = feeds.into_iter().filter(|(z0, z1, _)| (*z0 - *z1).abs() > 10.0).map(|(_, _, x)| x).collect();
    assert_eq!(long.len(), 3, "passes: {long:?} moves {:?}", path.moves);
    let step = 5.0 / 3.0;
    let expected = [25.0 - step, 25.0 - 2.0 * step, 20.0];
    for (got, want) in long.iter().zip(expected.iter()) {
        assert_close(*got, *want, 1e-8, "pass radius");
    }
    for pair in long.windows(2) {
        assert_close(pair[0] - pair[1], step, 1e-8, "equal step");
    }
}

#[test]
fn face_pass_count_final_z_and_centre() {
    let params = FaceParams { stock_front_z: 2.0, final_z: 0.0, step_z: 0.7, from_radius: 30.0, to_radius: 0.0, clearance: 1.0 };
    let path = face(&centre(0.8), &params).expect("face");
    assert_eq!(path.moves.len(), 12, "3 passes × 4 moves");
    let mut cursor = path.start;
    let mut cuts = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion
            && (z - cursor.0).abs() <= 1e-9
            && cursor.1 > x
        {
            cuts.push((z, x));
        }
        cursor = target(cursor, motion);
    }
    assert_eq!(cuts.len(), 3, "cutting feeds: {cuts:?}");
    let step = 2.0 / 3.0;
    let expected_z = [2.0 - step + 0.8, 2.0 - 2.0 * step + 0.8, 0.8];
    for (cut, want_z) in cuts.iter().zip(expected_z.iter()) {
        assert_close(cut.0, *want_z, 1e-9, "face z");
        assert_close(cut.1, -0.8, 1e-9, "to centre");
    }
    assert_close(cuts[2].0, 0.8, 1e-12, "final z");

    let tipped = face(&od_tip(0.8), &params).expect("tip face");
    let mut cursor = tipped.start;
    let mut tip_cuts = Vec::new();
    for motion in &tipped.moves {
        if let TurnMove::Feed { z, x } = *motion
            && (z - cursor.0).abs() <= 1e-9
            && cursor.1 > x
        {
            tip_cuts.push((z, x));
        }
        cursor = target(cursor, motion);
    }
    assert_eq!(tip_cuts.len(), 3);
    assert_close(tip_cuts[2].0, 0.0, 1e-12, "imaginary tip is exact in z");
    assert_close(tip_cuts[2].1, 0.0, 1e-12, "imaginary tip uses to_radius");
}

#[test]
fn id_roughing_stays_inside_the_bore_and_above_zero() {
    let nose = 0.8;
    // Ø20 entrance opening to Ø30 toward the chuck. Tube stock is already Ø16 (radius 8).
    let bore = profile(vec![line(0.0, 10.0, -15.0, 10.0), line(-15.0, 10.0, -15.0, 15.0), line(-15.0, 15.0, -35.0, 15.0)]);
    let tool = centre(nose);
    let path = rough(
        &bore,
        &tool,
        &RoughParams {
            side: Side::Inside,
            stock_outer_radius: 25.0,
            stock_inner_radius: 8.0,
            depth_of_cut: 2.0,
            leave_x: 0.0,
            leave_z: 0.0,
            clearance: 0.5,
            retract: 0.3,
        },
    )
    .expect("id rough");
    let boundary = compensated_profile(&bore, &tool, Side::Inside, 0.0, 0.0, 1e-4).expect("boundary");
    let mut cursor = path.start;
    let mut samples = vec![cursor];
    for motion in &path.moves {
        samples.extend(sample_move(cursor, motion, 8));
        cursor = target(cursor, motion);
    }
    for (z, x) in samples {
        assert!(x >= -1e-9, "below zero at ({z}, {x})");
        let clearance = distance_to_profile(&bore, z, x);
        assert!(clearance + 1e-6 >= nose, "ID gouge d={clearance} at ({z}, {x})");
        if let Some(limit) = boundary_max_radius_at(&boundary, z) {
            assert!(x <= limit + 1e-3, "outside the bore at ({z}, {x}), wall {limit}");
        }
    }
    // A pass above the Ø20 entrance must not travel into the Ø30 section.
    let mut cursor = path.start;
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion
            && x > 9.3
            && z < -1.0
            && cursor.0 - z > 1.0
        {
            panic!("pass at x={x} entered past the Ø20 mouth toward z={z}");
        }
        cursor = target(cursor, motion);
    }
}

fn boundary_max_radius_at(segments: &[Segment], z: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    for segment in segments {
        let bounds = segment.bounding_box();
        if z < bounds.min().x() - 1e-6 || z > bounds.max().x() + 1e-6 {
            continue;
        }
        match *segment {
            Segment::Line(line) => {
                let z0 = line.start().x();
                let z1 = line.end().x();
                let dz = z1 - z0;
                let y = if dz.abs() <= 1e-9 {
                    if (z - z0).abs() <= 1e-6 { line.start().y().max(line.end().y()) } else { continue }
                } else {
                    let t = (z - z0) / dz;
                    if !(0.0..=1.0).contains(&t) {
                        continue;
                    }
                    line.start().y() + t * (line.end().y() - line.start().y())
                };
                best = Some(best.map_or(y, |current| current.max(y)));
            }
            Segment::Arc(arc) => {
                for t in [0.0, 0.5, 1.0] {
                    let point = arc.point_at(t);
                    if (point.x() - z).abs() <= 1e-3 {
                        best = Some(best.map_or(point.y(), |current| current.max(point.y())));
                    }
                }
            }
        }
    }
    best
}

#[test]
fn hostile_inputs_are_invalid_and_do_not_panic() {
    let tool = centre(0.8);
    let part = cylinder(20.0, 10.0);
    let face_ok = FaceParams { stock_front_z: 1.0, final_z: 0.0, step_z: 0.5, from_radius: 10.0, to_radius: 0.0, clearance: 0.2 };
    let rough_ok = RoughParams {
        side: Side::Outside,
        stock_outer_radius: 25.0,
        stock_inner_radius: 0.0,
        depth_of_cut: 1.0,
        leave_x: 0.0,
        leave_z: 0.0,
        clearance: 0.2,
        retract: 0.2,
    };
    let finish_ok = FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 0.2, tol: 1e-4 };

    let nan_tool = TurnTool { nose_radius: f64::NAN, tip: TipPoint::Centre };
    assert!(matches!(face(&nan_tool, &face_ok), Err(TurnError::InvalidInput(_))));
    assert!(matches!(rough(&part, &nan_tool, &rough_ok), Err(TurnError::InvalidInput(_))));
    assert!(matches!(finish(&part, &nan_tool, &finish_ok), Err(TurnError::InvalidInput(_))));
    assert!(matches!(TurnTool { nose_radius: 0.0, tip: TipPoint::Centre }.validate(), Err(TurnError::InvalidInput(_))));
    assert!(matches!(
        TurnTool { nose_radius: -0.2, tip: TipPoint::ImaginaryTip { dz: f64::INFINITY, dx: 0.0 } }.validate(),
        Err(TurnError::InvalidInput(_))
    ));

    let mut zero_step = face_ok;
    zero_step.step_z = 0.0;
    assert!(matches!(face(&tool, &zero_step), Err(TurnError::InvalidInput(_))));
    let mut zero_depth = face_ok;
    zero_depth.final_z = zero_depth.stock_front_z;
    assert!(matches!(face(&tool, &zero_depth), Err(TurnError::InvalidInput(_))));
    let mut huge = face_ok;
    huge.stock_front_z = 1.0e9;
    huge.final_z = 0.0;
    huge.step_z = 1.0e-3;
    assert!(matches!(face(&tool, &huge), Err(TurnError::InvalidInput(_))));

    let mut zero_doc = rough_ok;
    zero_doc.depth_of_cut = 0.0;
    assert!(matches!(rough(&part, &tool, &zero_doc), Err(TurnError::InvalidInput(_))));
    let mut neg_leave = rough_ok;
    neg_leave.leave_x = -0.1;
    assert!(matches!(rough(&part, &tool, &neg_leave), Err(TurnError::InvalidInput(_))));
    let mut huge_doc = rough_ok;
    huge_doc.stock_outer_radius = 1.0e8;
    huge_doc.depth_of_cut = 1.0e-4;
    assert!(matches!(rough(&part, &tool, &huge_doc), Err(TurnError::InvalidInput(_))));

    let mut bad_tol = finish_ok;
    bad_tol.tol = f64::NAN;
    assert!(matches!(finish(&part, &tool, &bad_tol), Err(TurnError::InvalidInput(_))));
    bad_tol.tol = 0.0;
    assert!(matches!(finish(&part, &tool, &bad_tol), Err(TurnError::InvalidInput(_))));
    bad_tol.tol = -1.0;
    assert!(matches!(finish(&part, &tool, &bad_tol), Err(TurnError::InvalidInput(_))));
}

fn stepped(steps: &[(f64, f64, u8)]) -> TurnProfile {
    let mut segments = Vec::new();
    let mut z = 0.0;
    let mut radius = steps.first().expect("steps").0;
    for (index, (next_radius, length, kind)) in steps.iter().copied().enumerate() {
        if index > 0 && (next_radius - radius).abs() > 1e-6 {
            if kind == 0 {
                segments.push(line(z, radius, z, next_radius));
            } else {
                let z2 = z - 1.0;
                segments.push(line(z, radius, z2, next_radius));
                z = z2;
            }
            radius = next_radius;
        }
        let z2 = z - length;
        segments.push(line(z, radius, z2, next_radius));
        z = z2;
        radius = next_radius;
    }
    profile(segments)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn finish_and_rough_keep_the_nose_out_of_the_profile(
        steps in proptest::collection::vec((2.5f64..30.0, 2.0f64..10.0, 0u8..2u8), 1..5),
        nose in 0.2f64..1.6,
        leave_x in 0.0f64..0.3,
        leave_z in 0.0f64..0.3,
        doc in 0.4f64..2.5,
    ) {
        let part = stepped(&steps);
        let tool = centre(nose);
        let tol = 1e-4;
        let finished = match finish(&part, &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 0.4, tol }) {
            Ok(path) => path,
            Err(TurnError::InvalidInput(_)) => return Ok(()),
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        };
        let mut cursor = finished.start;
        for motion in &finished.moves {
            let end = target(cursor, motion);
            let d0 = distance_to_profile(&part, cursor.0, cursor.1);
            let d1 = distance_to_profile(&part, end.0, end.1);
            let contact = d0 <= nose + 1e-2 && d1 <= nose + 1e-2;
            for (z, x) in sample_move(cursor, motion, 8) {
                let distance = distance_to_profile(&part, z, x);
                prop_assert!(distance + 1e-6 >= nose, "finish inside the nose at ({z}, {x}) d={distance} nose={nose}");
                if contact {
                    prop_assert!(
                        distance <= nose + 1e-6 + tol,
                        "finish left the profile at ({z}, {x}) d={distance} nose={nose}"
                    );
                }
            }
            cursor = end;
        }

        let stock = part.max_radius() + nose + leave_x + 4.0;
        let roughed = match rough(
            &part,
            &tool,
            &RoughParams {
                side: Side::Outside,
                stock_outer_radius: stock,
                stock_inner_radius: 0.0,
                depth_of_cut: doc,
                leave_x,
                leave_z,
                clearance: 0.4,
                retract: 0.3,
            },
        ) {
            Ok(path) => path,
            Err(TurnError::InvalidInput(_)) => return Ok(()),
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        };
        let limit = nose + leave_x.min(leave_z);
        let mut cursor = roughed.start;
        let mut samples = vec![cursor];
        for motion in &roughed.moves {
            samples.extend(sample_move(cursor, motion, 6));
            cursor = target(cursor, motion);
        }
        for (z, x) in samples {
            prop_assert!(x >= -1e-6, "rough below zero at ({z}, {x})");
            let distance = distance_to_profile(&part, z, x);
            prop_assert!(distance + 1e-6 >= limit, "rough gouge d={distance} limit={limit} at ({z}, {x})");
        }
    }
}
