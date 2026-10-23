//! Analytic checks and properties for threading, grooving, parting and centreline drilling.

use proptest::prelude::*;

use super::{DrillParams, GrooveParams, Infeed, PartParams, ThreadParams, centre_drill, groove, part_off, thread, thread_pass_depths};
use crate::TurnError;
use crate::ops::{Side, TipPoint, TurnMove, TurnPath, TurnTool};

fn tool() -> TurnTool {
    TurnTool { nose_radius: 0.4, tip: TipPoint::Centre }
}

fn end_of(from: (f64, f64), motion: &TurnMove) -> (f64, f64) {
    match *motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } | TurnMove::Thread { z, x, .. } => (z, x),
        TurnMove::Dwell { .. } => from,
    }
}

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!((actual - expected).abs() <= tol, "{label}: {actual} != {expected} (tol {tol})");
}

struct Cut {
    z0: f64,
    x0: f64,
    z1: f64,
    x1: f64,
    lead: f64,
}

fn thread_cuts(path: &TurnPath) -> Vec<Cut> {
    let mut cursor = path.start;
    let mut cuts = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Thread { z, x, lead } = *motion {
            cuts.push(Cut { z0: cursor.0, x0: cursor.1, z1: z, x1: x, lead });
        }
        cursor = end_of(cursor, motion);
    }
    cuts
}

fn steps_of(depths: &[f64]) -> Vec<f64> {
    let mut prev = 0.0;
    let mut steps = Vec::new();
    for depth in depths {
        steps.push(depth - prev);
        prev = *depth;
    }
    steps
}

fn m10_thread(infeed: Infeed, spring: u8, min_depth: f64) -> ThreadParams {
    ThreadParams {
        side: Side::Outside,
        major_radius: 5.0,
        depth: 0.92,
        pitch: 1.5,
        z_start: 0.0,
        z_end: -12.0,
        first_depth: 0.2,
        min_depth,
        spring_passes: spring,
        infeed,
        clearance: 0.5,
        included_angle_deg: 60.0,
    }
}

#[test]
fn pass_depths_are_monotone_and_land_on_depth() {
    let depths = thread_pass_depths(1.0, 0.4, 0.1).expect("depths");
    assert_eq!(*depths.last().expect("last"), 1.0);
    let mut prev = 0.0;
    for depth in &depths {
        assert!(*depth > prev, "not monotone: {depths:?}");
        assert!(*depth - prev + 1e-12 >= 0.1, "step below min: {depths:?}");
        prev = *depth;
    }
    assert_close(depths[0], 0.4, 1e-12, "first");
    assert_close(depths[3], 0.8, 1e-12, "fourth");
    assert_close(depths[4], 0.9, 1e-9, "minimum step");
}

#[test]
fn shallow_thread_is_one_pass_even_when_shorter_than_min() {
    let depths = thread_pass_depths(0.05, 0.2, 0.1).expect("depths");
    assert_eq!(depths, vec![0.05]);
}

#[test]
fn m10x1_5_constant_area_pass_count() {
    // External depth 0.92 mm is the usual M10×1.5 single depth (about 0.61343 × 1.5).
    // With a 0.2 mm first cut, pass n sits at 0.2 * sqrt(n) until that exceeds 0.92.
    // 0.2 * sqrt(21) < 0.92 <= 0.2 * sqrt(22), so the schedule has 22 passes.
    let depths = thread_pass_depths(0.92, 0.2, 0.001).expect("depths");
    assert_eq!(depths.len(), 22, "{depths:?}");
    assert_eq!(*depths.last().expect("last"), 0.92);
    for (index, depth) in depths.iter().enumerate().take(21) {
        let expected = 0.2 * (index as f64 + 1.0).sqrt();
        assert_close(*depth, expected, 1e-12, "sqrt pass");
    }
    for step in steps_of(&depths) {
        assert!(step + 1e-12 >= 0.001, "step {step}");
    }
}

#[test]
fn m10_od_thread_path_and_spring_pass() {
    let params = m10_thread(Infeed::Radial, 1, 0.001);
    let depths = thread_pass_depths(params.depth, params.first_depth, params.min_depth).expect("depths");
    let path = thread(&params).expect("thread");
    let cuts = thread_cuts(&path);
    assert_eq!(cuts.len(), depths.len() + 1);
    assert_close(path.start.0, 0.5, 1e-12, "park z");
    assert_close(path.start.1, 5.5, 1e-12, "park x");
    for (index, cut) in cuts.iter().enumerate() {
        let depth = if index < depths.len() { depths[index] } else { params.depth };
        assert_close(cut.z0, params.z_start + params.clearance, 1e-12, "entry z");
        assert_close(cut.x0, params.major_radius - depth, 1e-12, "cut radius");
        assert_close(cut.z1, params.z_end, 1e-12, "end z");
        assert_close(cut.x1, cut.x0, 1e-12, "constant radius");
        assert_eq!(cut.lead, params.pitch);
    }
    let last = cuts.last().expect("spring");
    let before = cuts.get(cuts.len() - 2).expect("final cut");
    assert_close(last.x0, before.x0, 1e-12, "spring radius");
    assert_close(last.z0, before.z0, 1e-12, "spring z");
    // Retract is out to the major plus clearance, then back to the park Z.
    let mut cursor = path.start;
    let mut saw_retract = false;
    for motion in &path.moves {
        if let TurnMove::Rapid { z, x } = *motion
            && (z - params.z_end).abs() < 1e-12
            && (x - (params.major_radius + params.clearance)).abs() < 1e-12
            && (cursor.1 - x).abs() > 1e-9
        {
            saw_retract = true;
        }
        cursor = end_of(cursor, motion);
    }
    assert!(saw_retract, "missing radial retract");
}

#[test]
fn flank_infeed_shifts_start_by_depth_times_tan() {
    let angle = 29.5;
    let params = m10_thread(Infeed::Flank { angle_deg: angle }, 0, 0.001);
    let depths = thread_pass_depths(params.depth, params.first_depth, params.min_depth).expect("depths");
    let cuts = thread_cuts(&thread(&params).expect("thread"));
    let tan = angle.to_radians().tan();
    assert_eq!(cuts.len(), depths.len());
    for (cut, depth) in cuts.iter().zip(depths) {
        let shift = cut.z0 - (params.z_start + params.clearance);
        assert_close(shift, -depth * tan, 1e-12, "flank shift");
        assert_close(cut.x0, params.major_radius - depth, 1e-12, "radius");
    }
}

#[test]
fn alternating_flank_changes_sign_each_pass() {
    let angle = 29.5;
    let params = ThreadParams {
        side: Side::Outside,
        major_radius: 5.0,
        depth: 0.28,
        pitch: 1.5,
        z_start: 0.0,
        z_end: -8.0,
        first_depth: 0.2,
        min_depth: 0.01,
        spring_passes: 2,
        infeed: Infeed::AlternatingFlank { angle_deg: angle },
        clearance: 0.4,
        included_angle_deg: 60.0,
    };
    let depths = thread_pass_depths(params.depth, params.first_depth, params.min_depth).expect("depths");
    assert!(depths.len() >= 2, "{depths:?}");
    let cuts = thread_cuts(&thread(&params).expect("thread"));
    assert_eq!(cuts.len(), depths.len() + 2);
    let tan = angle.to_radians().tan();
    for (index, cut) in cuts.iter().enumerate() {
        let depth_index = index.min(depths.len() - 1);
        let depth = if index < depths.len() { depths[index] } else { *depths.last().expect("depth") };
        let sign = if depth_index.is_multiple_of(2) { -1.0 } else { 1.0 };
        assert_close(cut.z0 - (params.z_start + params.clearance), sign * depth * tan, 1e-12, "alt shift");
    }
    let spring_a = cuts.get(depths.len()).expect("spring");
    let spring_b = cuts.get(depths.len() + 1).expect("spring");
    let last_cut = cuts.get(depths.len() - 1).expect("last cut");
    assert_close(spring_a.z0, last_cut.z0, 1e-12, "spring copies flank");
    assert_close(spring_b.x0, last_cut.x0, 1e-12, "spring depth");
}

#[test]
fn internal_thread_mirrors_the_radius() {
    let params = ThreadParams {
        side: Side::Inside,
        major_radius: 8.0,
        depth: 0.7,
        pitch: 1.0,
        z_start: -1.0,
        z_end: -14.0,
        first_depth: 0.3,
        min_depth: 0.05,
        spring_passes: 0,
        infeed: Infeed::Radial,
        clearance: 0.25,
        included_angle_deg: 60.0,
    };
    let depths = thread_pass_depths(params.depth, params.first_depth, params.min_depth).expect("depths");
    let path = thread(&params).expect("id thread");
    assert_close(path.start.1, 8.0 - 0.25, 1e-12, "id park");
    let cuts = thread_cuts(&path);
    assert_eq!(cuts.len(), depths.len());
    for (cut, depth) in cuts.iter().zip(depths) {
        assert_close(cut.x0, 8.0 + depth, 1e-12, "id radius");
        assert_close(cut.z1, params.z_end, 1e-12, "id end");
        assert_eq!(cut.lead, 1.0);
    }
}

#[test]
fn groove_plunges_cover_the_walls_and_stay_on_the_floor() {
    let params = GrooveParams {
        side: Side::Outside,
        z_left: 0.0,
        z_right: 10.0,
        bottom_radius: 12.0,
        surface_radius: 18.0,
        insert_width: 4.0,
        peck: Some(2.0),
        stepover_frac: 0.5,
        clearance: 1.0,
        dwell_s: 0.2,
    };
    let path = groove(&tool(), &params).expect("groove");
    let plunges = floor_plunges(&path, params.bottom_radius);
    // step = 2. Stations at 0, 2, 4, then the exact end at 10 - 4 = 6.
    assert_eq!(plunges.first().copied(), Some(0.0));
    assert_eq!(plunges.last().copied(), Some(6.0));
    assert!(plunges.windows(2).all(|pair| pair[1] - pair[0] <= params.insert_width + 1e-9));
    assert_coverage(&plunges, params.insert_width, params.z_left, params.z_right);
    assert_not_past_floor(&path, params.side, params.bottom_radius);
    let dwells = path.moves.iter().filter(|motion| matches!(motion, TurnMove::Dwell { seconds } if *seconds == 0.2)).count();
    assert_eq!(dwells, plunges.len());
    assert!(has_floor_feed(&path, params.bottom_radius, 6.0, 0.0), "finish pass along the floor");
}

#[test]
fn groove_that_matches_the_insert_is_one_plunge() {
    let params = GrooveParams {
        side: Side::Inside,
        z_left: -5.0,
        z_right: -2.0,
        bottom_radius: 14.0,
        surface_radius: 10.0,
        insert_width: 3.0,
        peck: None,
        stepover_frac: 0.8,
        clearance: 0.5,
        dwell_s: 0.0,
    };
    let path = groove(&tool(), &params).expect("id groove");
    let plunges = floor_plunges(&path, params.bottom_radius);
    assert_eq!(plunges, vec![-5.0]);
    assert!(!path.moves.iter().any(|motion| matches!(motion, TurnMove::Dwell { .. })));
    assert_not_past_floor(&path, params.side, params.bottom_radius);
    let mut cursor = path.start;
    for motion in &path.moves {
        let end = end_of(cursor, motion);
        assert!(end.1 <= params.bottom_radius + 1e-9, "past the bore floor at {end:?}");
        assert!(end.1 >= -1e-9, "negative radius at {end:?}");
        cursor = end;
    }
}

#[test]
fn part_off_reaches_the_radius_on_a_45_degree_chamfer() {
    let params =
        PartParams { z: -10.0, surface_radius: 20.0, insert_width: 3.0, to_radius: -0.2, chamfer: Some(2.0), peck: Some(5.0), clearance: 1.0 };
    let path = part_off(&tool(), &params).expect("part");
    let mut cursor = path.start;
    let mut saw_chamfer = false;
    let mut min_x = path.start.1;
    for motion in &path.moves {
        let end = end_of(cursor, motion);
        min_x = min_x.min(end.1);
        if let TurnMove::Feed { z, x } = *motion {
            let dz = z - cursor.0;
            let dx = x - cursor.1;
            if dz.abs() > 1e-9 && (dz.abs() - dx.abs()).abs() < 1e-9 {
                assert!(dz < 0.0 && dx < 0.0, "chamfer direction ({dz}, {dx})");
                assert_close(dz.abs(), 2.0, 1e-12, "chamfer length");
                saw_chamfer = true;
            }
        }
        cursor = end;
    }
    assert!(saw_chamfer, "missing 45° chamfer: {:?}", path.moves);
    assert_close(min_x, params.to_radius, 1e-12, "part-off depth");
    let advances = radial_advances(&path, params.surface_radius - 2.0);
    let sum: f64 = advances.iter().sum();
    assert_close(sum, (params.surface_radius - 2.0) - params.to_radius, 1e-9, "peck sum");
    assert!(advances.iter().rev().skip(1).all(|step| (*step - 5.0).abs() < 1e-9));
    assert!(advances.last().is_some_and(|step| *step <= 5.0 + 1e-9));
}

#[test]
fn centre_drill_pecks_sum_to_depth_and_stay_on_axis() {
    let blind = DrillParams {
        z_start: 0.0,
        depth: 20.0,
        peck: Some(7.0),
        retract: 1.0,
        clearance: 2.0,
        point_angle_deg: 118.0,
        diameter: 10.0,
        through: false,
    };
    let blind_path = centre_drill(&blind).expect("blind");
    assert_x_zero(&blind_path);
    let advances = axial_advances(&blind_path, blind.z_start);
    assert_close(advances.iter().sum(), blind.depth, 1e-9, "blind pecks");
    assert!(advances.iter().rev().skip(1).all(|step| (*step - 7.0).abs() < 1e-9));
    assert_close(deepest(&blind_path), -20.0, 1e-12, "blind tip");

    let mut through = blind;
    through.through = true;
    let tip = (through.diameter * 0.5) / (through.point_angle_deg.to_radians() * 0.5).tan();
    let through_path = centre_drill(&through).expect("through");
    assert_x_zero(&through_path);
    assert_close(axial_advances(&through_path, through.z_start).iter().sum(), through.depth + tip, 1e-9, "through pecks");
    assert_close(deepest(&through_path), through.z_start - through.depth - tip, 1e-12, "through tip");
    // Pull-back is `retract`, and it stops at the clearance plane.
    let mut cursor = through_path.start;
    for motion in &through_path.moves {
        if let TurnMove::Rapid { z, x } = *motion {
            assert_eq!(x, 0.0);
            assert!(z <= through.z_start + through.clearance + 1e-12, "retract past clearance");
            assert!(z + 1e-12 >= cursor.0 || (z - (through.z_start + through.clearance)).abs() < 1e-12);
        }
        cursor = end_of(cursor, motion);
    }
}

#[test]
fn hostile_numbers_are_errors() {
    assert!(thread_pass_depths(f64::NAN, 0.2, 0.01).is_err());
    assert!(thread_pass_depths(1.0, f64::INFINITY, 0.01).is_err());
    assert!(thread_pass_depths(1.0, 0.2, 0.0).is_err());
    assert!(thread_pass_depths(-0.2, 0.1, 0.01).is_err());
    assert!(thread_pass_depths(0.2, -0.1, 0.01).is_err());

    let mut bad = m10_thread(Infeed::Radial, 0, 0.01);
    bad.z_end = bad.z_start;
    assert!(matches!(thread(&bad), Err(TurnError::InvalidInput(_))));
    bad.z_end = -12.0;
    bad.depth = bad.major_radius;
    assert!(thread(&bad).is_err());
    bad.depth = 0.92;
    bad.pitch = 0.0;
    assert!(thread(&bad).is_err());
    bad.pitch = 1.5;
    bad.infeed = Infeed::Flank { angle_deg: 90.0 };
    assert!(thread(&bad).is_err());
    bad.infeed = Infeed::Flank { angle_deg: 80.0 };
    bad.z_end = -0.2;
    bad.clearance = 0.0;
    bad.depth = 2.0;
    bad.major_radius = 10.0;
    bad.first_depth = 2.0;
    assert!(thread(&bad).is_err(), "flank shift must not run past the thread");

    let mut groove_bad = GrooveParams {
        side: Side::Outside,
        z_left: 1.0,
        z_right: 1.0,
        bottom_radius: 4.0,
        surface_radius: 8.0,
        insert_width: 2.0,
        peck: None,
        stepover_frac: 0.8,
        clearance: 0.5,
        dwell_s: 0.0,
    };
    assert!(groove(&tool(), &groove_bad).is_err());
    groove_bad.z_right = 2.5;
    groove_bad.insert_width = 2.0;
    assert!(groove(&tool(), &groove_bad).is_err(), "insert wider than the groove");
    groove_bad.insert_width = 1.0;
    groove_bad.stepover_frac = 0.0;
    assert!(groove(&tool(), &groove_bad).is_err());
    groove_bad.stepover_frac = 1.2;
    assert!(groove(&tool(), &groove_bad).is_err());
    groove_bad.stepover_frac = 0.8;
    groove_bad.bottom_radius = 9.0;
    assert!(groove(&tool(), &groove_bad).is_err());
    groove_bad.bottom_radius = 4.0;
    groove_bad.peck = Some(f64::NAN);
    assert!(groove(&tool(), &groove_bad).is_err());
    assert!(groove(&TurnTool { nose_radius: 0.0, tip: TipPoint::Centre }, &groove_bad).is_err());

    let mut part = PartParams { z: 0.0, surface_radius: 10.0, insert_width: 2.0, to_radius: 10.0, chamfer: Some(1.0), peck: None, clearance: 0.5 };
    assert!(part_off(&tool(), &part).is_err());
    part.to_radius = 0.0;
    part.chamfer = Some(-1.0);
    assert!(part_off(&tool(), &part).is_err());
    part.chamfer = Some(20.0);
    assert!(part_off(&tool(), &part).is_err());

    let mut drill = DrillParams {
        z_start: 0.0,
        depth: 0.0,
        peck: Some(-1.0),
        retract: -0.2,
        clearance: f64::NAN,
        point_angle_deg: 180.0,
        diameter: -3.0,
        through: true,
    };
    assert!(centre_drill(&drill).is_err());
    drill.depth = 5.0;
    drill.peck = Some(1.0);
    drill.retract = 0.2;
    drill.clearance = 1.0;
    drill.point_angle_deg = 0.0;
    drill.diameter = 8.0;
    assert!(centre_drill(&drill).is_err());
}

fn floor_plunges(path: &TurnPath, floor: f64) -> Vec<f64> {
    let mut cursor = path.start;
    let mut plunges = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion {
            let radial = (z - cursor.0).abs() <= 1e-9 && (x - cursor.1).abs() > 1e-9;
            if radial && (x - floor).abs() <= 1e-8 {
                plunges.push(z);
            }
        }
        cursor = end_of(cursor, motion);
    }
    plunges
}

fn assert_coverage(plunges: &[f64], width: f64, z_left: f64, z_right: f64) {
    assert!(!plunges.is_empty());
    let mut ordered = plunges.to_vec();
    ordered.sort_by(|a, b| a.total_cmp(b));
    assert_close(ordered[0], z_left, 1e-9, "left wall");
    assert_close(*ordered.last().expect("last"), z_right - width, 1e-9, "right edge");
    for pair in ordered.windows(2) {
        assert!(pair[1] - pair[0] <= width + 1e-6, "gap in groove: {pair:?}");
    }
    for z in &ordered {
        assert!(*z + 1e-6 >= z_left && *z <= z_right - width + 1e-6, "plunge outside the groove at {z}");
    }
}

fn assert_not_past_floor(path: &TurnPath, side: Side, floor: f64) {
    let mut cursor = path.start;
    let mut points = vec![cursor];
    for motion in &path.moves {
        points.push(end_of(cursor, motion));
        cursor = *points.last().expect("point");
    }
    for (z, x) in points {
        assert!(x.is_finite() && z.is_finite(), "non-finite move");
        match side {
            Side::Outside => assert!(x + 1e-6 >= floor, "below the floor at ({z}, {x})"),
            Side::Inside => assert!(x <= floor + 1e-6, "past the floor at ({z}, {x})"),
        }
    }
}

fn has_floor_feed(path: &TurnPath, floor: f64, z_from: f64, z_to: f64) -> bool {
    let mut cursor = path.start;
    let mut found = false;
    for motion in &path.moves {
        if let TurnMove::Feed { z, x } = *motion
            && (x - floor).abs() < 1e-8
            && (cursor.1 - floor).abs() < 1e-8
            && (cursor.0 - z_from).abs() < 1e-8
            && (z - z_to).abs() < 1e-8
        {
            found = true;
        }
        cursor = end_of(cursor, motion);
    }
    found
}

/// How far each feed drives the radius below `origin` (the plunge after the chamfer).
fn radial_advances(path: &TurnPath, origin: f64) -> Vec<f64> {
    let mut mark = origin;
    let mut advances = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { x, .. } = *motion
            && x < mark - 1e-12
        {
            advances.push(mark - x);
            mark = x;
        }
    }
    advances
}

fn axial_advances(path: &TurnPath, z_start: f64) -> Vec<f64> {
    let mut cursor = path.start;
    let mut deepest = z_start;
    let mut advances = Vec::new();
    for motion in &path.moves {
        if let TurnMove::Feed { z, .. } = *motion
            && z < deepest - 1e-15
        {
            advances.push(deepest - z);
            deepest = z;
        }
        cursor = end_of(cursor, motion);
    }
    advances
}

fn deepest(path: &TurnPath) -> f64 {
    let mut cursor = path.start;
    let mut best = path.start.0;
    for motion in &path.moves {
        cursor = end_of(cursor, motion);
        best = best.min(cursor.0);
    }
    best
}

fn assert_x_zero(path: &TurnPath) {
    assert_eq!(path.start.1, 0.0);
    let mut cursor = path.start;
    for motion in &path.moves {
        let end = end_of(cursor, motion);
        assert_eq!(end.1, 0.0, "left the axis at {end:?} via {motion:?}");
        cursor = end;
    }
}

fn finite_path(path: &TurnPath) -> bool {
    path.start.0.is_finite()
        && path.start.1.is_finite()
        && path.moves.len() <= 100_000
        && path.moves.iter().all(|motion| match *motion {
            TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => z.is_finite() && x.is_finite(),
            TurnMove::Thread { z, x, lead } => z.is_finite() && x.is_finite() && lead.is_finite() && lead > 0.0,
            TurnMove::Dwell { seconds } => seconds.is_finite() && seconds >= 0.0,
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn thread_depths_stay_monotone_and_exact(
        depth in 0.01f64..20.0,
        first in 0.01f64..5.0,
        min in 0.005f64..1.0,
    ) {
        let depths = thread_pass_depths(depth, first, min).expect("depths");
        prop_assert!(!depths.is_empty());
        prop_assert_eq!(*depths.last().expect("last"), depth);
        let mut prev = 0.0;
        for value in &depths {
            prop_assert!(*value > prev);
            prop_assert!(*value <= depth + 1e-12);
            let step = *value - prev;
            if step + 1e-8 < min {
                prop_assert_eq!(depths.len(), 1);
                prop_assert!(depth < min || depth <= first);
            }
            prev = *value;
        }
    }

    #[test]
    fn groove_coverage_holds(
        z_left in -40.0f64..10.0,
        extra in 0.0f64..16.0,
        insert in 0.4f64..5.0,
        surface in 8.0f64..30.0,
        depth in 0.3f64..4.0,
        frac in 0.2f64..1.0,
        clearance in 0.0f64..1.5,
        inside in proptest::bool::ANY,
        peck in proptest::option::of(0.2f64..2.5),
    ) {
        let z_right = z_left + insert + extra;
        let (side, bottom) = if inside {
            (Side::Inside, surface + depth)
        } else {
            (Side::Outside, surface - depth)
        };
        let params = GrooveParams {
            side,
            z_left,
            z_right,
            bottom_radius: bottom,
            surface_radius: surface,
            insert_width: insert,
            peck,
            stepover_frac: frac,
            clearance,
            dwell_s: 0.0,
        };
        let path = match groove(&tool(), &params) {
            Ok(path) => path,
            Err(TurnError::InvalidInput(message)) => return Err(TestCaseError::fail(message)),
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        };
        let plunges = floor_plunges(&path, bottom);
        prop_assert!(!plunges.is_empty());
        prop_assert!((plunges[0] - z_left).abs() < 1e-6);
        prop_assert!((plunges.last().expect("last") - (z_right - insert)).abs() < 1e-6);
        let mut ordered = plunges.clone();
        ordered.sort_by(|a, b| a.total_cmp(b));
        for pair in ordered.windows(2) {
            prop_assert!(pair[1] - pair[0] <= insert + 1e-6);
        }
        for z in &ordered {
            prop_assert!(*z + 1e-6 >= z_left && *z <= z_right - insert + 1e-6);
        }
        let mut cursor = path.start;
        let mut points = vec![cursor];
        for motion in &path.moves {
            points.push(end_of(cursor, motion));
            cursor = *points.last().expect("point");
        }
        for (_, x) in points {
            match side {
                Side::Outside => prop_assert!(x + 1e-6 >= bottom),
                Side::Inside => prop_assert!(x <= bottom + 1e-6),
            }
            prop_assert!(x >= -1e-6);
        }
    }

    #[test]
    fn hostile_cycles_do_not_panic(
        values in proptest::collection::vec(proptest::num::f64::ANY, 14),
        spring in 0u8..6,
        flags in 0u8..8,
    ) {
        let infeed = match flags % 3 {
            0 => Infeed::Radial,
            1 => Infeed::Flank { angle_deg: values[0] },
            _ => Infeed::AlternatingFlank { angle_deg: values[0] },
        };
        let params = ThreadParams {
            side: if flags & 4 == 0 { Side::Outside } else { Side::Inside },
            major_radius: values[1],
            depth: values[2],
            pitch: values[3],
            z_start: values[4],
            z_end: values[5],
            first_depth: values[6],
            min_depth: values[7],
            spring_passes: spring,
            infeed,
            clearance: values[8],
            included_angle_deg: values[9],
        };
        match thread(&params) {
            Ok(path) => prop_assert!(finite_path(&path)),
            Err(TurnError::InvalidInput(_)) => {}
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        }
        match thread_pass_depths(values[2], values[6], values[7]) {
            Ok(depths) => {
                prop_assert!(!depths.is_empty());
                prop_assert!(depths.iter().all(|depth| depth.is_finite()));
            }
            Err(TurnError::InvalidInput(_)) => {}
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        }

        let groove_params = GrooveParams {
            side: if flags & 1 == 0 { Side::Outside } else { Side::Inside },
            z_left: values[4],
            z_right: values[5],
            bottom_radius: values[10],
            surface_radius: values[11],
            insert_width: values[12],
            peck: if flags & 2 == 0 { None } else { Some(values[13]) },
            stepover_frac: values[0],
            clearance: values[8],
            dwell_s: values[3],
        };
        match groove(&tool(), &groove_params) {
            Ok(path) => prop_assert!(finite_path(&path)),
            Err(TurnError::InvalidInput(_)) => {}
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        }

        let part = PartParams {
            z: values[4],
            surface_radius: values[11],
            insert_width: values[12],
            to_radius: values[10],
            chamfer: if flags & 2 == 0 { None } else { Some(values[13]) },
            peck: if flags & 1 == 0 { None } else { Some(values[6]) },
            clearance: values[8],
        };
        match part_off(&tool(), &part) {
            Ok(path) => prop_assert!(finite_path(&path)),
            Err(TurnError::InvalidInput(_)) => {}
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        }

        let drill = DrillParams {
            z_start: values[4],
            depth: values[2],
            peck: if flags & 1 == 0 { None } else { Some(values[6]) },
            retract: values[7],
            clearance: values[8],
            point_angle_deg: values[9],
            diameter: values[12],
            through: flags & 4 != 0,
        };
        match centre_drill(&drill) {
            Ok(path) => {
                prop_assert!(finite_path(&path));
                prop_assert_eq!(path.start.1, 0.0);
            }
            Err(TurnError::InvalidInput(_)) => {}
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        }
    }
}
