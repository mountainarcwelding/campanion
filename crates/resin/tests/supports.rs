//! Chamfered raft and pillar-support generation. Nothing here has been printed.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use campanion_mesh::{Aabb, Mesh, Vec3, write_stl_binary};
use campanion_resin::supports::{edge_insets, overhangs_are_covered, render_supported_png};
use campanion_resin::{
    InputPoint, LayerParams, M7_MAX, REQUIRED_FLAGS, ResinSettings, SupportParams, SupportParamsOverlay, build_pm7m, generate_supports,
    is_watertight, parse_points_text, parse_support_params_json, read_pm7m,
};

fn box_mesh(min: Vec3, max: Vec3) -> Mesh {
    Mesh::new(Aabb::from_corners(min, max).face_triangles().to_vec()).expect("box")
}

fn params() -> SupportParams {
    SupportParams {
        raft_chamfer_angle_deg: 30.0,
        raft_thickness_mm: 1.5,
        raft_margin_mm: 3.0,
        raft_corner_relief_mm: 0.0,
        overhang_angle_deg: 45.0,
        support_spacing_mm: 4.0,
        tip_diameter_mm: 0.6,
        tip_length_mm: 1.2,
        pillar_diameter_mm: 1.2,
        foot_diameter_mm: 3.0,
        foot_height_mm: 0.8,
        part_lift_mm: 5.0,
    }
}

/// A vertical post with a larger plate on top, so the plate underside is an overhang.
fn table_mesh() -> Mesh {
    let post = box_mesh(Vec3::new(-2.0, -2.0, 0.0), Vec3::new(2.0, 2.0, 8.0));
    let plate = box_mesh(Vec3::new(-10.0, -10.0, 8.0), Vec3::new(10.0, 10.0, 10.0));
    let mut tris = post.triangles().to_vec();
    tris.extend_from_slice(plate.triangles());
    Mesh::new(tris).expect("table")
}

#[test]
fn chamfer_inset_matches_thickness_over_tan_angle() {
    let p = params();
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 15.0, 4.0));
    let result = generate_supports(&part, &p, &[], true).expect("supports");
    let expected = p.raft_thickness_mm / p.raft_chamfer_angle_deg.to_radians().tan();
    assert!((result.outlines.chamfer_inset_mm - expected).abs() < 1e-12, "{} vs {expected}", result.outlines.chamfer_inset_mm);
    let insets = edge_insets(&result.outlines.bottom, &result.outlines.top).expect("insets");
    assert!(!insets.is_empty());
    for inset in &insets {
        assert!((inset - expected).abs() < 1e-9, "edge inset {inset} != {expected}");
    }
    // Bottom is the full footprint; top is smaller. The part is 20 x 15, plus 3 mm margin on each side.
    let (bx, by): (Vec<f64>, Vec<f64>) = result.outlines.bottom.iter().cloned().unzip();
    let (tx, ty): (Vec<f64>, Vec<f64>) = result.outlines.top.iter().cloned().unzip();
    let bottom_w = bx.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - bx.iter().cloned().fold(f64::INFINITY, f64::min);
    let bottom_h = by.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - by.iter().cloned().fold(f64::INFINITY, f64::min);
    let top_w = tx.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - tx.iter().cloned().fold(f64::INFINITY, f64::min);
    let top_h = ty.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - ty.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!((bottom_w - 26.0).abs() < 1e-9, "bottom width {bottom_w}");
    assert!((bottom_h - 21.0).abs() < 1e-9, "bottom height {bottom_h}");
    assert!((bottom_w - top_w - 2.0 * expected).abs() < 1e-9, "width inset {bottom_w} {top_w}");
    assert!((bottom_h - top_h - 2.0 * expected).abs() < 1e-9, "height inset {bottom_h} {top_h}");
    let raft_bounds = result.raft.bounds().expect("raft bounds");
    assert!((raft_bounds.min().z).abs() < 1e-12);
    assert!((raft_bounds.max().z - p.raft_thickness_mm).abs() < 1e-9);
    let part_bounds = result.part.bounds().expect("part bounds");
    assert!((part_bounds.min().z - (p.raft_thickness_mm + p.part_lift_mm)).abs() < 1e-9);
}

#[test]
fn generated_solids_are_watertight_and_have_positive_volume() {
    let result = generate_supports(&table_mesh(), &params(), &[], true).expect("supports");
    assert!(is_watertight(&result.raft).expect("raft"));
    assert!(is_watertight(&result.pillars).expect("pillars") || result.points.is_empty());
    assert!(is_watertight(&result.part).expect("part"));
    assert!(is_watertight(&result.mesh).expect("combined"));
    assert!(campanion_resin::supports::signed_volume(&result.raft) > 0.0);
    assert!(campanion_resin::supports::signed_volume(&result.pillars) > 0.0);
    assert!(!result.points.is_empty(), "table underside should get supports");
}

#[test]
fn supports_reach_every_overhang_above_the_threshold() {
    let p = params();
    let part = table_mesh();
    let result = generate_supports(&part, &p, &[], true).expect("supports");
    assert!(
        overhangs_are_covered(&result.part, &p, &result.points, p.raft_thickness_mm).expect("coverage"),
        "an overhang sample was farther than {} mm from every support",
        p.support_spacing_mm
    );
}

#[test]
fn corner_relief_cuts_each_corner_back() {
    let mut p = params();
    p.raft_corner_relief_mm = 1.0;
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 15.0, 4.0));
    let sharp = generate_supports(&part, &params(), &[], true).expect("sharp");
    let relieved = generate_supports(&part, &p, &[], true).expect("relieved");
    assert_eq!(sharp.outlines.bottom.len(), 4);
    assert_eq!(relieved.outlines.bottom.len(), 8);
    // A cut-back corner sits inward of the original sharp corner.
    let sharp_corners = sharp.outlines.bottom.clone();
    for &(x, y) in &relieved.outlines.bottom {
        let dist = sharp_corners.iter().map(|(sx, sy)| (x - sx).hypot(y - sy)).fold(f64::INFINITY, f64::min);
        assert!(dist > 0.5, "relieved vertex ({x}, {y}) is too close to a sharp corner ({dist})");
    }
}

#[test]
fn missing_parameters_and_bad_values_are_refused() {
    let empty = SupportParamsOverlay::default();
    let err = empty.complete().expect_err("missing");
    assert!(err.to_string().contains("no defaults are assumed"), "{err}");
    for flag in REQUIRED_FLAGS {
        assert!(err.to_string().contains(flag), "missing {flag} in {err}");
    }

    let err = parse_support_params_json("{}").unwrap().complete().expect_err("empty json");
    assert!(err.to_string().contains("no defaults are assumed"), "{err}");

    let typo = parse_support_params_json("{\"raft_thickness_mm\": 1.0, \"unknown\": 2}");
    assert!(typo.is_err(), "{typo:?}");

    let mut p = params();
    p.raft_chamfer_angle_deg = 0.0;
    assert!(p.validate().is_err());
    p = params();
    p.raft_chamfer_angle_deg = 90.0;
    assert!(p.validate().is_err());
    p = params();
    p.part_lift_mm = 1.0;
    let err = p.validate().expect_err("short lift");
    assert!(err.to_string().contains("part lift"), "{err}");
}

#[test]
fn support_params_json_and_points_file_round_trip() {
    let json = r#"{
        "raft_chamfer_angle_deg": 30,
        "raft_thickness_mm": 1.5,
        "raft_margin_mm": 3,
        "raft_corner_relief_mm": 0,
        "overhang_angle_deg": 45,
        "support_spacing_mm": 4,
        "tip_diameter_mm": 0.6,
        "tip_length_mm": 1.2,
        "pillar_diameter_mm": 1.2,
        "foot_diameter_mm": 3,
        "foot_height_mm": 0.8,
        "part_lift_mm": 5
    }"#;
    let parsed = parse_support_params_json(json).unwrap().complete().expect("complete");
    assert_eq!(parsed, params());

    let text = "# comments allowed\n1.5 2.5\n0 0 9.5\n";
    let pts = parse_points_text(text).expect("points");
    assert_eq!(pts, vec![InputPoint { x: 1.5, y: 2.5, z: None }, InputPoint { x: 0.0, y: 0.0, z: Some(9.5) }]);

    let part = box_mesh(Vec3::new(-5.0, -5.0, 0.0), Vec3::new(5.0, 5.0, 3.0));
    let only = generate_supports(&part, &params(), &[InputPoint { x: 0.0, y: 0.0, z: None }], false).expect("explicit");
    assert_eq!(only.points.len(), 1);
    assert!((only.points[0].x).abs() < 1e-9 && (only.points[0].y).abs() < 1e-9);
}

#[test]
fn supported_mesh_slices_to_a_readable_pm7m() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 3.0));
    let mut p = params();
    p.support_spacing_mm = 5.0;
    let result = generate_supports(&part, &p, &[], true).expect("supports");
    let settings = ResinSettings {
        layer_height_mm: 0.5,
        bottom_layers: 2,
        bottom: LayerParams { exposure_s: 20.0, lift_height_mm: 4.0, lift_speed: 2.0 },
        normal: LayerParams { exposure_s: 2.5, lift_height_mm: 8.0, lift_speed: 6.0 },
        resin_name: "campanion".into(),
    };
    let (data, report) = build_pm7m(&result.mesh, &M7_MAX, &settings, campanion_resin::Placement::Center).expect("slice");
    let back = read_pm7m(&data).expect("read-back");
    assert_eq!(back.layers.len(), report.layers);
    assert_eq!(back.machine_name, M7_MAX.name);
    // Raft + lift + part: 1.5 + 5 + 3 = 9.5 mm; 0.5 mm layers -> 19 layers.
    assert_eq!(report.layers, 19, "layers {} height {}", report.layers, report.height_mm);
    assert!(report.volume_ml > 0.0);
    // The first layer is the full raft footprint, later layers include the lifted part.
    assert!(!back.layer_images.is_empty());
    let _ = write_stl_binary(&result.mesh);
    let png = render_supported_png(&result, 320, 240).expect("png");
    assert!(png.starts_with(b"\x89PNG"), "not a png");
}
