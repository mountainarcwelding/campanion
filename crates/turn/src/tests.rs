//! Profile input: drawings, arcs, meshes, and hostile numbers.

use std::f64::consts::PI;

use campanion_geom::{Arc, Line, Point, Segment};
use campanion_mesh::{Mesh, Triangle, Vec3};
use proptest::prelude::*;

use crate::{MeshAxis, TurnError, TurnProfile};

fn pt(x: f64, y: f64) -> Point {
    Point::new(x, y).expect("point")
}

fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> Segment {
    Segment::Line(Line::new(pt(x0, y0), pt(x1, y1)).expect("line"))
}

fn close_to(actual: f64, expected: f64, tol: f64) -> bool {
    (actual - expected).abs() <= tol
}

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!(close_to(actual, expected, tol), "{label}: {actual} != {expected} (tol {tol})");
}

fn dxf_mm(entities: &str) -> String {
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn line_entity(layer: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!("0\nLINE\n8\n{layer}\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n")
}

fn lines_entity(layer: &str, points: &[(f64, f64)]) -> String {
    let mut body = String::new();
    for pair in points.windows(2) {
        let (Some((x0, y0)), Some((x1, y1))) = (pair.first(), pair.get(1)) else {
            continue;
        };
        body.push_str(&line_entity(layer, *x0, *y0, *x1, *y1));
    }
    body
}

/// Closed stepped shaft: front at Z = 0, radius 8 for 12 mm, then radius 14 to Z = −25.
fn shaft_points(sign: f64) -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (0.0, sign * 8.0), (-12.0, sign * 8.0), (-12.0, sign * 14.0), (-25.0, sign * 14.0), (-25.0, 0.0), (0.0, 0.0)]
}

fn closed_lines(layer: &str, points: &[(f64, f64)]) -> String {
    let mut body = String::new();
    for pair in points.windows(2) {
        let (Some((x0, y0)), Some((x1, y1))) = (pair.first(), pair.get(1)) else {
            continue;
        };
        body.push_str(&line_entity(layer, *x0, *y0, *x1, *y1));
    }
    body
}

fn rectangle_points() -> Vec<(f64, f64)> {
    vec![(0.0, 0.0), (100.0, 0.0), (100.0, 80.0), (0.0, 80.0), (0.0, 0.0)]
}

fn assert_shaft(profile: &TurnProfile) {
    assert_eq!(profile.segments().len(), 5, "axis segment dropped");
    assert!(profile.start().almost_eq(pt(0.0, 0.0), 1e-9), "start {:?}", profile.start());
    assert!(profile.end().almost_eq(pt(-25.0, 0.0), 1e-9), "end {:?}", profile.end());
    assert!(profile.start().x() >= profile.end().x());
    let (zmin, zmax) = profile.z_range();
    assert_close(zmin, -25.0, 1e-9, "zmin");
    assert_close(zmax, 0.0, 1e-9, "zmax");
    assert_close(profile.max_radius(), 14.0, 1e-9, "max r");
    assert_close(profile.min_radius(), 0.0, 1e-9, "min r");
    assert_close(profile.radius_at(0.0).expect("front"), 8.0, 1e-9, "r(0)");
    assert_close(profile.radius_at(-6.0).expect("step"), 8.0, 1e-9, "r(-6)");
    assert_close(profile.radius_at(-12.0).expect("shoulder"), 14.0, 1e-9, "r(-12)");
    assert_close(profile.radius_at(-18.0).expect("second"), 14.0, 1e-9, "r(-18)");
    assert_close(profile.radius_at(-25.0).expect("back"), 14.0, 1e-9, "r(-25)");
    assert!(profile.radius_at(0.1).is_none());
    assert!(profile.radius_at(-25.1).is_none());
    assert!(profile.radius_at(f64::NAN).is_none());
    let points = profile.to_polyline_points(1e-4);
    assert_eq!(points.len(), profile.segments().len() + 1);
    assert!(points.first().expect("first").almost_eq(profile.start(), 1e-9));
    assert!(points.last().expect("last").almost_eq(profile.end(), 1e-9));
}

#[test]
fn stepped_shaft_dxf_opens_from_the_front() {
    let mut entities = closed_lines("BIG", &rectangle_points());
    entities.push_str(&closed_lines("SHAFT", &shaft_points(1.0)));
    let text = dxf_mm(&entities);

    let shaft = TurnProfile::from_dxf(&text, Some("SHAFT"), 1e-6).expect("shaft");
    assert_shaft(&shaft);

    // The rectangle's box is larger, so an unfiltered import follows that contour.
    let biggest = TurnProfile::from_dxf(&text, None, 1e-6).expect("largest");
    assert_close(biggest.max_radius(), 80.0, 1e-9, "rectangle radius");
    assert_close(biggest.start().x(), 100.0, 1e-9, "rectangle front");
    let (zmin, zmax) = biggest.z_range();
    assert_close(zmin, 0.0, 1e-9, "rect zmin");
    assert_close(zmax, 100.0, 1e-9, "rect zmax");

    let missing = TurnProfile::from_dxf(&text, Some("NOPE"), 1e-6).expect_err("missing layer");
    assert!(matches!(missing, TurnError::NoProfile(_)), "{missing}");
}

#[test]
fn reversed_closed_chain_still_starts_at_max_z() {
    let mut points = shaft_points(1.0);
    points.reverse();
    let profile = TurnProfile::from_dxf(&dxf_mm(&closed_lines("SHAFT", &points)), Some("SHAFT"), 1e-6).expect("reversed");
    assert_shaft(&profile);
}

#[test]
fn negative_drawing_is_mirrored_about_the_axis() {
    let profile = TurnProfile::from_dxf(&dxf_mm(&closed_lines("SHAFT", &shaft_points(-1.0))), Some("SHAFT"), 1e-6).expect("mirror");
    assert_shaft(&profile);
    for segment in profile.segments() {
        assert!(segment.bounding_box().min().y() >= -1e-9);
    }
}

#[test]
fn negative_arc_mirrors_and_keeps_the_sweep() {
    // Centre (0, −10), radius 4, 0° to 180°: the half nearer the axis, entirely below it.
    let entity = "0\nARC\n10\n0\n20\n-10\n40\n4\n50\n0\n51\n180\n";
    let profile = TurnProfile::from_dxf(&dxf_mm(entity), None, 1e-6).expect("arc");
    assert!(profile.start().y() > 0.0, "mirrored start {:?}", profile.start());
    assert_close(profile.start().x(), 4.0, 1e-9, "start z");
    assert_close(profile.end().x(), -4.0, 1e-9, "end z");
    assert_close(profile.start().y(), 10.0, 1e-9, "start r");
    // The other circle intersection at Z = 0 is radius 14 and is not on this sweep.
    let radius = profile.radius_at(0.0).expect("mid");
    assert_close(radius, 6.0, 1e-9, "r(0)");
    assert!(radius < 8.0, "{radius}");
}

#[test]
fn open_polyline_is_used_as_is() {
    let points = [(0.0, 5.0), (-10.0, 5.0), (-10.0, 9.0), (-16.0, 9.0)];
    let profile = TurnProfile::from_dxf(&dxf_mm(&lines_entity("OPEN", &points)), Some("OPEN"), 1e-6).expect("open");
    assert_eq!(profile.segments().len(), 3);
    assert!(profile.start().almost_eq(pt(0.0, 5.0), 1e-9));
    assert!(profile.end().almost_eq(pt(-16.0, 9.0), 1e-9));
    assert_close(profile.radius_at(-5.0).expect("first"), 5.0, 1e-9, "r(-5)");
    assert_close(profile.radius_at(-10.0).expect("shoulder"), 9.0, 1e-9, "r(-10)");
    assert_close(profile.radius_at(-13.0).expect("second"), 9.0, 1e-9, "r(-13)");
}

#[test]
fn open_negative_polyline_is_mirrored() {
    let points = [(0.0, -5.0), (-20.0, -5.0)];
    let profile = TurnProfile::from_dxf(&dxf_mm(&lines_entity("OPEN", &points)), None, 1e-6).expect("open mirror");
    assert_close(profile.radius_at(-10.0).expect("r"), 5.0, 1e-9, "r");
    assert!(profile.start().almost_eq(pt(0.0, 5.0), 1e-9), "{:?}", profile.start());
    assert!(profile.min_radius() > 0.0);
}

#[test]
fn svg_polygon_maps_x_to_z_and_y_to_radius() {
    // Page Y flips, so user Y = 40 is the spindle. Front is drawing X = 30.
    let svg = r#"<svg width="40mm" height="40mm" viewBox="0 0 40 40"><polygon points="30,40 30,32 18,32 18,26 5,26 5,40"/></svg>"#;
    let profile = TurnProfile::from_svg(svg, 1e-6).expect("svg");
    assert!(profile.start().x() >= profile.end().x());
    assert_close(profile.z_range().0, 5.0, 1e-6, "zmin");
    assert_close(profile.z_range().1, 30.0, 1e-6, "zmax");
    assert_close(profile.radius_at(24.0).expect("small"), 8.0, 1e-6, "r(24)");
    assert_close(profile.radius_at(10.0).expect("large"), 14.0, 1e-6, "r(10)");
    assert!(profile.min_radius() >= -1e-6);
}

#[test]
fn fillet_radius_matches_the_circle() {
    let arc = Arc::new(pt(2.0, 15.0), 5.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2).expect("arc");
    let arc_start = arc.start_point();
    let arc_end = arc.end_point();
    let profile = TurnProfile::try_new(
        vec![line(10.0, 20.0, arc_start.x(), arc_start.y()), Segment::Arc(arc), line(arc_end.x(), arc_end.y(), arc_end.x() - 12.0, arc_end.y())],
        1e-9,
    )
    .expect("fillet");
    assert!(profile.start().x() > profile.end().x());
    assert_close(profile.radius_at(6.0).expect("od"), 20.0, 1e-9, "line");
    assert_close(profile.radius_at(-10.0).expect("neck"), 15.0, 1e-9, "neck");
    // (z + 3)² is not needed: at z = −1, (z − 2)² = 9, so the outer root is 15 + 4.
    assert_close(profile.radius_at(-1.0).expect("fillet"), 19.0, 1e-9, "fillet");
    let z: f64 = 0.0;
    let expected = 15.0 + (25.0 - (z - 2.0).powi(2)).sqrt();
    assert_close(profile.radius_at(z).expect("analytic"), expected, 1e-9, "analytic");

    let flat = profile.to_polyline_points(0.01);
    assert!(flat.len() > 4, "arc was chorded");
    let only = TurnProfile::try_new(vec![Segment::Arc(arc)], 1e-9).expect("arc profile");
    let chords = only.to_polyline_points(0.01);
    assert!(chords.len() >= 3);
    let center = pt(2.0, 15.0);
    for pair in chords.windows(2) {
        let (Some(a), Some(b)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let mid = pt(0.5 * (a.x() + b.x()), 0.5 * (a.y() + b.y()));
        let sagitta = 5.0 - center.distance(mid);
        assert!(sagitta <= 0.01 + 1e-9, "sagitta {sagitta}");
        assert!(sagitta >= -1e-8, "sagitta {sagitta}");
    }
}

#[test]
fn arc_query_ignores_the_intersection_off_the_sweep() {
    // Lower semicircle of centre (0, 10), radius 8. At Z = 0 the circle also has radius 18.
    let arc = Arc::new(pt(0.0, 10.0), 8.0, 0.0, -PI).expect("arc");
    let profile = TurnProfile::try_new(vec![Segment::Arc(arc)], 1e-9).expect("profile");
    assert_close(profile.radius_at(0.0).expect("bottom"), 2.0, 1e-9, "r(0)");
    let at_four = 10.0 - (64.0 - 16.0_f64).sqrt();
    assert_close(profile.radius_at(4.0).expect("side"), at_four, 1e-9, "r(4)");
}

#[test]
fn arc_extreme_beyond_the_chord_is_inside_z_range() {
    // Sweep 2 → 4 rad passes through π, so min Z is the left tangent, past both ends.
    let arc = Arc::new(pt(0.0, 10.0), 6.0, 2.0, 2.0).expect("arc");
    let profile = TurnProfile::try_new(vec![Segment::Arc(arc)], 1e-9).expect("profile");
    let (zmin, _) = profile.z_range();
    let ends = arc.start_point().x().min(arc.end_point().x());
    assert!(zmin < ends - 1.0, "zmin {zmin} ends {ends}");
    let radius = profile.radius_at(zmin).expect("extreme");
    assert_close(radius, 10.0, 1e-9, "extreme radius");
    assert!(profile.radius_at(zmin - 1e-6).is_none());
}

#[test]
fn chain_running_toward_positive_z_is_reversed() {
    let profile = TurnProfile::try_new(vec![line(-20.0, 4.0, 0.0, 6.0)], 1e-9).expect("reversed");
    assert!(profile.start().almost_eq(pt(0.0, 6.0), 1e-9), "{:?}", profile.start());
    assert!(profile.end().almost_eq(pt(-20.0, 4.0), 1e-9), "{:?}", profile.end());
    assert_close(profile.radius_at(-10.0).expect("mid"), 5.0, 1e-9, "mid");

    let arc = Arc::new(pt(2.0, 15.0), 5.0, std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2).expect("arc");
    let forward = TurnProfile::try_new(
        vec![
            line(10.0, 20.0, arc.start_point().x(), arc.start_point().y()),
            Segment::Arc(arc),
            line(arc.end_point().x(), arc.end_point().y(), -15.0, arc.end_point().y()),
        ],
        1e-9,
    )
    .expect("forward");
    let backward: Vec<Segment> = forward.segments().iter().copied().rev().map(Segment::reversed).collect();
    let again = TurnProfile::try_new(backward, 1e-9).expect("again");
    assert!(again.start().almost_eq(forward.start(), 1e-9));
    assert!(again.end().almost_eq(forward.end(), 1e-9));
    for z in [-10.0, -1.0, 6.0] {
        let left = forward.radius_at(z).expect("forward");
        let right = again.radius_at(z).expect("again");
        assert_close(left, right, 1e-9, "roundtrip");
    }
}

fn revolved(axis: MeshAxis, z0: f64, z1: f64, r0: f64, r1: f64, facets: usize) -> Mesh {
    let place = |along: f64, radius: f64, angle: f64| -> Vec3 {
        let u = radius * angle.cos();
        let v = radius * angle.sin();
        match axis {
            MeshAxis::X => Vec3::new(along, u, v),
            MeshAxis::Y => Vec3::new(v, along, u),
            MeshAxis::Z => Vec3::new(u, v, along),
        }
    };
    let mut triangles = Vec::with_capacity(facets.saturating_mul(4));
    let origin0 = place(z0, 0.0, 0.0);
    let origin1 = place(z1, 0.0, 0.0);
    let tau = std::f64::consts::TAU;
    for i in 0..facets {
        let a0 = (i as f64) * tau / (facets as f64);
        let a1 = ((i + 1) as f64) * tau / (facets as f64);
        let b00 = place(z0, r0, a0);
        let b01 = place(z0, r0, a1);
        let b10 = place(z1, r1, a0);
        let b11 = place(z1, r1, a1);
        triangles.push(Triangle { a: b00, b: b01, c: b10 });
        triangles.push(Triangle { a: b01, b: b11, c: b10 });
        triangles.push(Triangle { a: origin0, b: b01, c: b00 });
        triangles.push(Triangle { a: origin1, b: b10, c: b11 });
    }
    Mesh::new(triangles).expect("mesh")
}

fn assert_revolved(axis: MeshAxis, r0: f64, r1: f64) {
    let length = 30.0;
    let facets = 64usize;
    let mesh = revolved(axis, 0.0, length, r0, r1, facets);
    // Step divides the length, so a sample is never pulled onto the neighbouring ring.
    let profile = TurnProfile::from_mesh(&mesh, axis, 0.5).expect("envelope");
    let (zmin, zmax) = profile.z_range();
    assert_close(zmin, 0.0, 1e-6, "zmin");
    assert_close(zmax, length, 1e-6, "zmax");
    assert!(profile.start().x() + 1e-9 >= profile.end().x());
    assert_close(profile.start().x(), length, 1e-6, "front");
    assert_close(profile.end().x(), 0.0, 1e-6, "back");
    let sagitta = r0.max(r1) * (1.0 - (PI / facets as f64).cos());
    for z in [0.0, 7.5, 15.0, 22.5, 30.0] {
        let expected = r0 + (z / length) * (r1 - r0);
        let radius = profile.radius_at(z).expect("radius");
        assert!(
            (radius - expected).abs() <= sagitta + 1e-9,
            "{axis:?} z {z} r {radius} expected {expected} sagitta {sagitta} segments {}",
            profile.segments().len()
        );
    }
    assert!(profile.radius_at(-0.01).is_none());
    assert!(profile.radius_at(length + 0.01).is_none());
    assert!(profile.segments().len() <= 3, "{axis:?} segments {}", profile.segments().len());
}

#[test]
fn mesh_cylinder_and_frustum_match_within_facet_sagitta() {
    for axis in [MeshAxis::Z, MeshAxis::X, MeshAxis::Y] {
        assert_revolved(axis, 10.0, 10.0);
        assert_revolved(axis, 10.0, 20.0);
    }
}

#[test]
fn hostile_input_returns_errors() {
    let step = TurnProfile::try_new(vec![line(0.0, 5.0, -10.0, 5.0)], 1e-9).expect("step");
    for bad in [f64::NAN, 0.0, -1.0, f64::INFINITY, f64::NEG_INFINITY] {
        let err = TurnProfile::from_mesh(&revolved(MeshAxis::Z, 0.0, 30.0, 10.0, 10.0, 8), MeshAxis::Z, bad).expect_err("step");
        assert!(matches!(err, TurnError::InvalidInput(_)), "{bad} -> {err}");
    }
    let too_fine = TurnProfile::from_mesh(&revolved(MeshAxis::Z, 0.0, 30.0, 10.0, 10.0, 8), MeshAxis::Z, 1e-6).expect_err("count");
    assert!(matches!(too_fine, TurnError::InvalidInput(_)), "{too_fine}");

    let empty_mesh = Mesh::new(Vec::new()).expect("empty mesh");
    let mesh_err = TurnProfile::from_mesh(&empty_mesh, MeshAxis::Z, 1.0).expect_err("empty mesh");
    assert!(matches!(mesh_err, TurnError::Mesh(_)), "{mesh_err}");

    let empty = TurnProfile::from_dxf(&dxf_mm(""), None, 1e-6).expect_err("empty dxf");
    assert!(matches!(empty, TurnError::NoProfile(_)), "{empty}");
    assert!(TurnProfile::from_svg("not svg", 1e-6).is_err());
    assert!(TurnProfile::from_dxf("not dxf", None, 1e-6).is_err());

    let gap = TurnProfile::try_new(vec![line(0.0, 5.0, -10.0, 5.0), line(-10.2, 5.0, -20.0, 5.0)], 1e-3).expect_err("gap");
    assert!(matches!(gap, TurnError::InvalidInput(_)), "{gap}");
    assert!(TurnProfile::try_new(vec![line(0.0, 5.0, -10.0, 5.0), line(-10.0 + 5e-4, 5.0, -20.0, 5.0)], 1e-3).is_ok());

    assert!(matches!(TurnProfile::try_new(Vec::new(), 1e-6).expect_err("empty"), TurnError::InvalidInput(_)));
    assert!(TurnProfile::try_new(vec![line(0.0, 1.0, -1.0, 1.0)], f64::NAN).is_err());
    assert!(TurnProfile::try_new(vec![line(0.0, -0.1, -1.0, -0.1)], 1e-9).is_err());
    assert!(TurnProfile::try_new(vec![line(0.0, -1e-12, -1.0, -1e-12)], 1e-9).is_ok());

    let dipped = Arc::new(pt(0.0, 5.0), 6.0, -2.2, 1.4).expect("dip");
    assert!(dipped.start_point().y() > 0.0 && dipped.end_point().y() > 0.0);
    assert!(dipped.bounding_box().min().y() < -0.5);
    let dip_err = TurnProfile::try_new(vec![Segment::Arc(dipped)], 1e-9).expect_err("dip");
    assert!(matches!(dip_err, TurnError::InvalidInput(_)), "{dip_err}");

    assert!(step.radius_at(f64::NAN).is_none());
    assert!(step.radius_at(f64::INFINITY).is_none());
    assert!(step.radius_at(f64::NEG_INFINITY).is_none());

    let mut many = Vec::new();
    let mut z = 0.0;
    for _ in 0..=100_000 {
        let next = z - 0.01;
        many.push(line(z, 1.0, next, 1.0));
        z = next;
    }
    let too_many = TurnProfile::try_new(many, 1e-9).expect_err("count");
    assert!(matches!(too_many, TurnError::InvalidInput(_)), "{too_many}");
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn random_stepped_shaft_radius_matches_each_step(
        steps in proptest::collection::vec((1.0f64..50.0, 0.5f64..20.0), 2..6)
    ) {
        let mut segments = Vec::new();
        let mut z_cursor = 0.0;
        let mut r_cursor = steps.first().expect("steps").0;
        for (index, (radius, length)) in steps.iter().copied().enumerate() {
            if index > 0 && (radius - r_cursor).abs() > 1e-12 {
                segments.push(line(z_cursor, r_cursor, z_cursor, radius));
                r_cursor = radius;
            }
            let z_next = z_cursor - length;
            segments.push(line(z_cursor, r_cursor, z_next, radius));
            z_cursor = z_next;
            r_cursor = radius;
        }
        let profile = TurnProfile::try_new(segments, 1e-9).expect("profile");
        let mut z = 0.0;
        for (radius, length) in steps.iter().copied() {
            let mid = z - length * 0.5;
            let got = profile.radius_at(mid).expect("mid");
            prop_assert!((got - radius).abs() < 1e-9, "z {mid} radius {got} expected {radius}");
            z -= length;
        }
        let (zmin, zmax) = profile.z_range();
        prop_assert!((zmax - 0.0).abs() < 1e-9, "zmax {zmax}");
        prop_assert!((zmin - z).abs() < 1e-9, "zmin {zmin} expected {z}");
        prop_assert!(profile.start().x() + 1e-12 >= profile.end().x());
    }
}
