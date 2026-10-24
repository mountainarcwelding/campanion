//! Section DXF: one half-profile, including a seam-broken +X side.

use campanion_geom::Segment;

use crate::TurnProfile;

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!((actual - expected).abs() <= tol, "{label}: {actual} != {expected} (tol {tol})");
}

fn dxf_mm(entities: &str) -> String {
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn line_entity(x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!("0\nLINE\n8\n0\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n")
}

fn arc_entity(cx: f64, cy: f64, radius: f64, start_deg: f64, end_deg: f64) -> String {
    format!("0\nARC\n8\n0\n10\n{cx}\n20\n{cy}\n40\n{radius}\n50\n{start_deg}\n51\n{end_deg}\n")
}

fn lines(points: &[(f64, f64)]) -> String {
    let mut body = String::new();
    for pair in points.windows(2) {
        let (Some((x0, y0)), Some((x1, y1))) = (pair.first(), pair.get(1)) else {
            continue;
        };
        body.push_str(&line_entity(*x0, *y0, *x1, *y1));
    }
    body
}

#[test]
fn shaft_seam_keeps_the_exact_half() {
    let profile = TurnProfile::from_section_dxf(include_str!("../tests/fixtures/section_shaft_seam.dxf"), 1e-6).expect("shaft section");
    let (zmin, zmax) = profile.z_range();
    assert_close(zmin, -60.0, 1e-6, "zmin");
    assert_close(zmax, 0.0, 1e-6, "zmax");
    assert_close(profile.radius_at(-10.0).expect("r(-10)"), 15.0, 1e-6, "r(-10)");
    assert_close(profile.radius_at(-50.0).expect("r(-50)"), 20.0, 1e-6, "r(-50)");
    assert_close(profile.max_radius(), 20.0, 1e-6, "max r");
    assert_close(profile.start().y(), 0.0, 1e-6, "start r");
    assert_close(profile.end().y(), 20.0, 1e-6, "end r (back face dropped)");
}

#[test]
fn fillet_seam_keeps_the_outside_arc() {
    let profile = TurnProfile::from_section_dxf(include_str!("../tests/fixtures/section_fillet_seam.dxf"), 1e-6).expect("fillet section");
    assert_close(profile.radius_at(-20.0).expect("r(-20)"), 15.0, 1e-5, "r(-20)");
    let at_fillet = 12.0 + (9.0 - 4.0_f64).sqrt();
    assert_close(profile.radius_at(-1.0).expect("r(-1)"), at_fillet, 1e-4, "r(-1)");
    assert!(profile.segments().iter().any(|segment| matches!(segment, Segment::Arc(_))), "expected an arc");
}

#[test]
fn equal_halves_match_the_upper_half() {
    let both = format!(
        "{}{}{}{}",
        line_entity(10.0, 0.0, -10.0, 0.0),
        line_entity(10.0, 0.0, 10.0, -30.0),
        line_entity(-10.0, 0.0, -10.0, -30.0),
        line_entity(10.0, -30.0, -10.0, -30.0),
    );
    let upper = format!("{}{}{}", line_entity(0.0, 0.0, 10.0, 0.0), line_entity(10.0, 0.0, 10.0, -30.0), line_entity(10.0, -30.0, 0.0, -30.0),);
    let from_both = TurnProfile::from_section_dxf(&dxf_mm(&both), 1e-6).expect("both");
    let from_upper = TurnProfile::from_section_dxf(&dxf_mm(&upper), 1e-6).expect("upper");
    assert_eq!(from_both, from_upper);
}

#[test]
fn halves_that_differ_are_rejected() {
    let body = format!(
        "{}{}{}{}",
        line_entity(10.0, 0.0, -12.0, 0.0),
        line_entity(10.0, 0.0, 10.0, -20.0),
        line_entity(-12.0, 0.0, -12.0, -20.0),
        line_entity(10.0, -20.0, -12.0, -20.0),
    );
    let err = TurnProfile::from_section_dxf(&dxf_mm(&body), 1e-6).expect_err("differ");
    assert!(err.to_string().contains("differ"), "{err}");
}

#[test]
fn a_tube_section_is_not_a_half_profile() {
    let body = format!(
        "{}{}",
        lines(&[(10.0, 0.0), (20.0, 0.0), (20.0, -10.0), (10.0, -10.0), (10.0, 0.0)]),
        lines(&[(-20.0, 0.0), (-10.0, 0.0), (-10.0, -10.0), (-20.0, -10.0), (-20.0, 0.0)]),
    );
    let err = TurnProfile::from_section_dxf(&dxf_mm(&body), 1e-6).expect_err("tube");
    assert!(err.to_string().contains("half-profile"), "{err}");
}

#[test]
fn empty_and_garbage_are_errors() {
    assert!(TurnProfile::from_section_dxf("", 1e-6).is_err());
    assert!(TurnProfile::from_section_dxf("not a dxf", 1e-6).is_err());
    let empty = "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n";
    let err = TurnProfile::from_section_dxf(empty, 1e-6).expect_err("empty entities");
    assert!(err.to_string().contains("the section is empty"), "{err}");
}

#[test]
fn a_counterclockwise_fillet_stays_outside() {
    let body = format!(
        "{}{}{}{}",
        line_entity(0.0, 0.0, 12.0, 0.0),
        arc_entity(12.0, -3.0, 3.0, 0.0, 90.0),
        line_entity(15.0, -3.0, 15.0, -10.0),
        line_entity(15.0, -10.0, 0.0, -10.0),
    );
    let profile = TurnProfile::from_section_dxf(&dxf_mm(&body), 1e-6).expect("fillet");
    let at_fillet = 12.0 + (9.0 - 4.0_f64).sqrt();
    assert_close(profile.radius_at(-1.0).expect("r(-1)"), at_fillet, 1e-6, "r(-1)");
    assert!(profile.segments().iter().any(|segment| matches!(segment, Segment::Arc(_))));
}
