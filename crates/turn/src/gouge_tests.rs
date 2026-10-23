//! Gouge checks for a normally oriented outside tool, and for inputs the check must reject.
//!
//! `orientation_deg` 45 points the holder up and toward +Z. That is the usual outside tool
//! turning toward the chuck. 135° points the holder toward the chuck and is not used here.

use campanion_geom::{Line, Point, Segment};
use campanion_ir::{Hand, Insert, InsertShape};
use proptest::prelude::*;

use super::ops::{FaceParams, FinishParams, RoughParams, Side, TipPoint, TurnMove, TurnPath, TurnTool, face, finish, rough};
use super::{GougeKind, HolderGeom, InsertGeom, TurnGouge, check_turn_gouges};
use crate::{TurnError, TurnProfile};

fn pt(z: f64, x: f64) -> Point {
    Point::new(z, x).expect("point")
}

fn line(z0: f64, x0: f64, z1: f64, x1: f64) -> Segment {
    Segment::Line(Line::new(pt(z0, x0), pt(z1, x1)).expect("line"))
}

fn profile(segments: Vec<Segment>) -> TurnProfile {
    TurnProfile::try_new(segments, 1e-6).expect("profile")
}

fn insert_at(orientation: f64, nose: f64) -> InsertGeom {
    let insert = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, orientation).expect("insert");
    InsertGeom::from_insert(&insert, 4.0).expect("geom")
}

fn section_of(part: &TurnProfile) -> Vec<(f64, f64)> {
    part.to_polyline_points(1e-3).into_iter().map(|point| (point.x(), point.y())).collect()
}

fn shift_x(path: &TurnPath, dx: f64) -> TurnPath {
    let move_x = |x: f64| x + dx;
    TurnPath {
        start: (path.start.0, move_x(path.start.1)),
        moves: path
            .moves
            .iter()
            .map(|motion| match *motion {
                TurnMove::Rapid { z, x } => TurnMove::Rapid { z, x: move_x(x) },
                TurnMove::Feed { z, x } => TurnMove::Feed { z, x: move_x(x) },
                TurnMove::Arc { z, x, cz, cx, ccw } => TurnMove::Arc { z, x: move_x(x), cz, cx: move_x(cx), ccw },
                TurnMove::Thread { z, x, lead } => TurnMove::Thread { z, x: move_x(x), lead },
                TurnMove::Dwell { seconds } => TurnMove::Dwell { seconds },
            })
            .collect(),
    }
}

fn checked(section: &[(f64, f64)], side: Side, path: &TurnPath, geom: &InsertGeom, holder: Option<&HolderGeom>) -> Result<Vec<TurnGouge>, TurnError> {
    check_turn_gouges(section, side, path, TipPoint::Centre, geom, holder, 0.5, 1e-3)
}

fn nose_depth(gouges: &[TurnGouge]) -> Option<f64> {
    gouges.iter().filter(|gouge| gouge.what == GougeKind::Nose).map(|gouge| gouge.depth).reduce(f64::max)
}

/// Horizontal lands joined by a vertical step, a 45° chamfer, or a 2:1 taper.
///
/// The chain starts at Z = 0 and runs toward the chuck. The last land is at least 4 mm, longer
/// than the 1 mm finish clearance, so the +Z retract stays over that land.
fn od_profile(steps: &[(f64, f64, u8)]) -> Result<TurnProfile, TurnError> {
    let mut segments = Vec::new();
    let mut z = 0.0;
    let mut radius = match steps.first() {
        Some((radius, _, _)) => *radius,
        None => return Err(TurnError::InvalidInput("profile is empty".into())),
    };
    for (index, &(next_radius, land, kind)) in steps.iter().enumerate() {
        if index > 0 && (next_radius - radius).abs() > 1e-6 {
            let run = (next_radius - radius).abs();
            let dz = match kind {
                0 => 0.0,
                1 => -run,
                _ => -2.0 * run,
            };
            let z2 = z + dz;
            segments.push(line(z, radius, z2, next_radius));
            z = z2;
        }
        let z2 = z - land;
        segments.push(line(z, radius, z2, radius));
        z = z2;
        radius = next_radius;
    }
    TurnProfile::try_new(segments, 1e-6)
}

fn finish_od(part: &TurnProfile, nose: f64) -> Result<TurnPath, TurnError> {
    let tool = TurnTool { nose_radius: nose, tip: TipPoint::Centre };
    finish(part, &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 1.0, tol: 1e-4 })
}

#[test]
fn cylinder_finish_has_no_nose_and_a_shift_noses_by_that_shift() {
    let part = profile(vec![line(0.0, 20.0, -40.0, 20.0)]);
    let geom = insert_at(45.0, 0.8);
    let section = section_of(&part);
    let finished = finish_od(&part, 0.8).expect("finish");
    let clean = checked(&section, Side::Outside, &finished, &geom, None).expect("check");
    assert!(nose_depth(&clean).is_none(), "clean nose: {clean:?}");
    let buried = shift_x(&finished, -0.2);
    let gouges = checked(&section, Side::Outside, &buried, &geom, None).expect("shifted");
    let depth = nose_depth(&gouges).unwrap_or(0.0);
    assert!((depth - 0.2).abs() <= 1e-3, "nose depth {depth}");
}

#[test]
fn eighty_degree_insert_edges_a_back_facing_undercut() {
    // Overhang: the wall climbs back toward +Z, so an 80° insert at 45° cannot follow it.
    let part = profile(vec![line(0.0, 12.0, -30.0, 12.0), line(-30.0, 12.0, -28.0, 22.0), line(-28.0, 22.0, -50.0, 22.0)]);
    let geom = insert_at(45.0, 0.8);
    let finished = finish_od(&part, 0.8).expect("finish");
    let gouges = checked(&section_of(&part), Side::Outside, &finished, &geom, None).expect("check");
    assert!(gouges.iter().any(|gouge| gouge.what == GougeKind::Edge), "expected an edge gouge, got {gouges:?}");
}

#[test]
fn holder_behind_a_deep_bore_reports_holder() {
    // Bore wall at radius 20. The nose stays at radius 8. At 90° the holder runs out along +radius.
    let section = [(0.0, 20.0), (-80.0, 20.0)];
    let path = TurnPath { start: (-20.0, 8.0), moves: vec![TurnMove::Feed { z: -60.0, x: 8.0 }] };
    let geom = insert_at(90.0, 0.8);
    let holder = HolderGeom { width: 12.0, length: 30.0 };
    let gouges = checked(&section, Side::Inside, &path, &geom, Some(&holder)).expect("check");
    assert!(gouges.iter().any(|gouge| gouge.what == GougeKind::Holder), "expected a holder gouge, got {gouges:?}");
}

#[test]
fn hostile_inputs_error_without_panic() {
    let geom = insert_at(45.0, 0.8);
    let path = TurnPath { start: (0.0, 10.0), moves: vec![TurnMove::Feed { z: -10.0, x: 10.0 }] };
    let tip = TipPoint::Centre;
    let empty = check_turn_gouges(&[], Side::Outside, &path, tip, &geom, None, 0.25, 1e-3).expect_err("empty");
    assert!(empty.to_string().contains("section is empty"), "{empty}");
    let nan = check_turn_gouges(&[(0.0, f64::NAN), (-10.0, 10.0)], Side::Outside, &path, tip, &geom, None, 0.25, 1e-3).expect_err("nan");
    assert!(nan.to_string().contains("section point is not finite"), "{nan}");
    let step = check_turn_gouges(&[(0.0, 10.0), (-10.0, 10.0)], Side::Outside, &path, tip, &geom, None, 0.0, 1e-3).expect_err("step");
    assert!(step.to_string().contains("sample step must be finite and positive"), "{step}");
    let nan_step = check_turn_gouges(&[(0.0, 10.0), (-10.0, 10.0)], Side::Outside, &path, tip, &geom, None, f64::NAN, 1e-3).expect_err("nan step");
    assert!(nan_step.to_string().contains("sample step must be finite and positive"), "{nan_step}");
}

#[test]
fn example_shaft_face_rough_and_finish_are_clean_at_45() {
    let dxf_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/shaft.dxf");
    let text = std::fs::read_to_string(&dxf_path).unwrap_or_else(|err| panic!("read {}: {err}", dxf_path.display()));
    let part = TurnProfile::from_dxf(&text, Some("PROFILE"), 1e-6).expect("import");
    let tool = TurnTool { nose_radius: 0.8, tip: TipPoint::Centre };
    let geom = insert_at(45.0, 0.8);
    let section: Vec<(f64, f64)> = part.to_polyline_points(0.01).into_iter().map(|point| (point.x(), point.y())).collect();
    let faced =
        face(&tool, &FaceParams { stock_front_z: 1.0, final_z: 0.0, step_z: 0.5, from_radius: 25.0, to_radius: 0.0, clearance: 2.0 }).expect("face");
    let roughed = rough(
        &part,
        &tool,
        &RoughParams {
            side: Side::Outside,
            stock_outer_radius: 25.0,
            stock_inner_radius: 0.0,
            depth_of_cut: 2.0,
            leave_x: 0.3,
            leave_z: 0.1,
            clearance: 2.0,
            retract: 2.0,
        },
    )
    .expect("rough");
    let finished =
        finish(&part, &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 2.0, tol: 0.001 }).expect("finish");
    for (name, path) in [("face", &faced), ("rough", &roughed), ("finish", &finished)] {
        let gouges = check_turn_gouges(&section, Side::Outside, path, TipPoint::Centre, &geom, None, 0.1, 0.01).expect(name);
        assert!(gouges.is_empty(), "{name} gouged: {gouges:?}");
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn finish_on_stepped_od_has_no_nose_until_the_path_moves_into_the_part(
        steps in proptest::collection::vec((8.0f64..30.0, 4.0f64..10.0, 0u8..3u8), 1..5),
        nose in 0.4f64..1.2,
    ) {
        let part = match od_profile(&steps) {
            Ok(part) => part,
            Err(TurnError::InvalidInput(_)) => return Ok(()),
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        };
        let finished = match finish_od(&part, nose) {
            Ok(path) => path,
            Err(TurnError::InvalidInput(_)) => return Ok(()),
            Err(err) => return Err(TestCaseError::fail(err.to_string())),
        };
        let geom = insert_at(45.0, nose);
        let section = section_of(&part);
        let clean = checked(&section, Side::Outside, &finished, &geom, None).map_err(|err| TestCaseError::fail(err.to_string()))?;
        prop_assert!(nose_depth(&clean).is_none(), "nose on a compensated finish: {clean:?}");
        let buried = shift_x(&finished, -0.2);
        let gouges = checked(&section, Side::Outside, &buried, &geom, None).map_err(|err| TestCaseError::fail(err.to_string()))?;
        let depth = nose_depth(&gouges).unwrap_or(0.0);
        prop_assert!((depth - 0.2).abs() <= 0.05, "shifted nose depth {depth}");
    }
}
