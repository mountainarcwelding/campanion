//! Island detection, calibration profiles, and the support optimizer.
//!
//! Fixture constants in this file are test values only. They are not recommended print settings
//! and are not shipped as program defaults.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use campanion_mesh::{Aabb, Mesh, Vec3};
use campanion_resin::{
    CALIBRATION_SCHEMA, CalibrationProfile, GenerateOptions, InputPoint, IslandDetectParams, PointSource, SupportParams, add_support_point,
    delete_support_point, detect_islands, generate_supports, generate_supports_with, move_support_point, parse_calibration, parse_points_text,
    write_points_text,
};

fn box_mesh(min: Vec3, max: Vec3) -> Mesh {
    Mesh::new(Aabb::from_corners(min, max).face_triangles().to_vec()).expect("box")
}

fn join(meshes: &[Mesh]) -> Mesh {
    let mut tris = Vec::new();
    for mesh in meshes {
        tris.extend_from_slice(mesh.triangles());
    }
    Mesh::new(tris).expect("join")
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

fn detect() -> IslandDetectParams {
    IslandDetectParams { layer_height_mm: 0.5, cell_mm: 0.5 }
}

/// A cup that is closed toward the plate and open toward +Z (later layers / the film).
fn cup_mesh() -> Mesh {
    // Floor 20 x 20 x 2, walls leaving a 12 x 12 well from z = 2 to z = 10.
    join(&[
        box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 2.0)),
        box_mesh(Vec3::new(0.0, 0.0, 2.0), Vec3::new(4.0, 20.0, 10.0)),
        box_mesh(Vec3::new(16.0, 0.0, 2.0), Vec3::new(20.0, 20.0, 10.0)),
        box_mesh(Vec3::new(4.0, 0.0, 2.0), Vec3::new(16.0, 4.0, 10.0)),
        box_mesh(Vec3::new(4.0, 16.0, 2.0), Vec3::new(16.0, 20.0, 10.0)),
    ])
}

fn mid_air_mesh() -> Mesh {
    join(&[box_mesh(Vec3::new(-5.0, -5.0, 0.0), Vec3::new(5.0, 5.0, 3.0)), box_mesh(Vec3::new(20.0, 20.0, 15.0), Vec3::new(26.0, 26.0, 18.0))])
}

/// Test-only fixture profile. The numbers are not recommended settings.
fn fixture_profile(capacity_n: f64) -> String {
    format!(
        r#"{{
        "schema": "{CALIBRATION_SCHEMA}",
        "name": "fixture-not-a-default",
        "tip_palette": [
            {{
                "name": "Light",
                "tip_diameter_mm": 0.3,
                "tip_length_mm": 1.0,
                "pillar_diameter_mm": 0.8,
                "foot_diameter_mm": 2.0,
                "foot_height_mm": 0.6,
                "tip_capacity_n": {capacity_n},
                "contact_area_mm2": 0.07
            }},
            {{
                "name": "Heavy",
                "tip_diameter_mm": 0.6,
                "tip_length_mm": 1.2,
                "pillar_diameter_mm": 1.2,
                "foot_diameter_mm": 3.0,
                "foot_height_mm": 0.8,
                "tip_capacity_n": {heavy},
                "contact_area_mm2": 0.28
            }}
        ],
        "peel_coefficient_n_per_mm2": 0.001,
        "suction_coefficient_n_per_mm2": 0.0005,
        "safety_factor": 1.5,
        "resin_density_g_per_cm3": 1.1,
        "gravity_m_s2": 9.80665
    }}"#,
        heavy = capacity_n * 4.0
    )
}

#[test]
fn first_layer_of_a_box_is_one_island() {
    let mesh = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 4.0));
    let features = detect_islands(&mesh, &detect()).expect("detect");
    assert!(!features.islands.is_empty(), "expected at least one island");
    assert_eq!(features.islands.len(), 1, "a solid box should first appear as one region, got {:?}", features.islands);
    let island = &features.islands[0];
    assert!(island.area_mm2 > 50.0, "area {}", island.area_mm2);
    assert!((island.centroid.0 - 5.0).abs() < 1.0, "cx {}", island.centroid.0);
    assert!((island.centroid.1 - 4.0).abs() < 1.0, "cy {}", island.centroid.1);
}

#[test]
fn mid_air_body_is_a_separate_island() {
    let features = detect_islands(&mid_air_mesh(), &detect()).expect("detect");
    assert!(features.islands.len() >= 2, "expected the plate body and the mid-air body, got {}", features.islands.len());
    let high = features.islands.iter().filter(|island| island.z_mm >= 14.0).count();
    assert!(high >= 1, "mid-air island missing: {:?}", features.islands);
}

#[test]
fn cup_opening_toward_later_layers_is_reported() {
    let features = detect_islands(&cup_mesh(), &detect()).expect("detect");
    assert!(!features.cups.is_empty(), "expected a cup closed toward the plate, got none; layers {:?}", features.layers);
    assert!(features.cups.iter().any(|cup| cup.area_mm2 > 20.0), "cup areas {:?}", features.cups);
}

#[test]
fn island_aware_auto_places_a_point_on_a_mid_air_island() {
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(detect());
    let result = generate_supports_with(&mid_air_mesh(), &params(), options).expect("supports");
    let islands = result.features.expect("features");
    assert!(islands.islands.len() >= 2);
    for island in &islands.islands {
        let covered = result.points.iter().any(|point| {
            let dx = point.x - island.centroid.0;
            let dy = point.y - island.centroid.1;
            (point.x >= island.min.0 - 2.0 && point.x <= island.max.0 + 2.0 && point.y >= island.min.1 - 2.0 && point.y <= island.max.1 + 2.0)
                || dx * dx + dy * dy <= 25.0
        });
        assert!(covered, "island {} at {:?} z={} has no support; points {:?}", island.id, island.centroid, island.z_mm, result.points);
    }
}

#[test]
fn lowest_band_uses_denser_spacing() {
    let part = box_mesh(Vec3::new(-8.0, -8.0, 0.0), Vec3::new(8.0, 8.0, 6.0));
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(IslandDetectParams { layer_height_mm: 0.5, cell_mm: 1.0 });
    let uniform = generate_supports_with(&part, &params(), options.clone()).expect("uniform");
    options.lowest_band_height_mm = Some(2.0);
    options.lowest_band_spacing_mm = Some(2.0);
    let dense = generate_supports_with(&part, &params(), options).expect("dense");
    let z_cut = dense.features.as_ref().unwrap().z_min_mm + 2.0;
    let uniform_low = uniform.points.iter().filter(|p| p.contact_z <= z_cut + 1.0).count();
    let dense_low = dense.points.iter().filter(|p| p.contact_z <= z_cut + 1.0).count();
    assert!(dense_low >= uniform_low, "lowest-band spacing should not reduce points ({dense_low} vs {uniform_low})");
    assert!(dense.points.len() >= uniform.points.len());
}

#[test]
fn raft_undercut_is_unchanged_with_island_detection() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 15.0, 4.0));
    let p = params();
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(detect());
    let result = generate_supports_with(&part, &p, options).expect("supports");
    let expected = p.raft_thickness_mm / p.raft_chamfer_angle_deg.to_radians().tan();
    assert!((result.outlines.chamfer_inset_mm - expected).abs() < 1e-12);
    let (bx, _): (Vec<f64>, Vec<f64>) = result.outlines.bottom.iter().cloned().unzip();
    let (tx, _): (Vec<f64>, Vec<f64>) = result.outlines.top.iter().cloned().unzip();
    let bottom_w = bx.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - bx.iter().cloned().fold(f64::INFINITY, f64::min);
    let top_w = tx.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - tx.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(bottom_w < top_w, "bed side must stay narrower than the part side");
}

#[test]
fn points_file_round_trips_optional_preset_and_manual_edits() {
    let text = "# comment\n1.5 2.5\n0 0 9.5 Light\n";
    let pts = parse_points_text(text).expect("parse");
    assert_eq!(pts[0], InputPoint::new(1.5, 2.5, None));
    assert_eq!(pts[1].preset.as_deref(), Some("Light"));
    assert_eq!(pts[1].source, PointSource::Manual);

    let part = box_mesh(Vec3::new(-5.0, -5.0, 0.0), Vec3::new(5.0, 5.0, 3.0));
    let mut result = generate_supports(&part, &params(), &[InputPoint::new(0.0, 0.0, None)], false).expect("explicit");
    assert_eq!(result.points.len(), 1);
    assert_eq!(result.points[0].source, PointSource::Manual);
    add_support_point(&mut result.points, &result.part, 2.0, 0.0, None, params().raft_thickness_mm).expect("add");
    assert_eq!(result.points.len(), 2);
    move_support_point(&mut result.points, 1, &result.part, -2.0, 0.0, None, params().raft_thickness_mm).expect("move");
    assert!((result.points[1].x + 2.0).abs() < 1e-9);
    let removed = delete_support_point(&mut result.points, 1).expect("delete");
    assert!((removed.x + 2.0).abs() < 1e-9);
    assert_eq!(result.points.len(), 1);

    let written = write_points_text(&result.points, Some(&["Light".into()]));
    let back = parse_points_text(&written).expect("round-trip");
    assert_eq!(back.len(), 1);
    assert!((back[0].x - result.points[0].x).abs() < 1e-6);
}

#[test]
fn calibration_profile_lists_missing_constants_and_refuses_to_fit() {
    let missing = parse_calibration(r#"{"schema":"campanion-resin-calibration/1","name":"partial"}"#);
    let err = missing.expect_err("incomplete");
    let text = err.to_string();
    assert!(text.contains("incomplete"), "{text}");
    assert!(text.contains("tip_palette"), "{text}");
    assert!(text.contains("peel_coefficient_n_per_mm2"), "{text}");
    assert!(text.contains("safety_factor"), "{text}");
    assert!(text.contains("gravity_m_s2"), "{text}");

    let profile = parse_calibration(&fixture_profile(0.2)).expect("complete fixture");
    assert!(profile.is_complete());
    let fit = profile.fit_from_measurements().expect_err("stub");
    assert!(fit.to_string().contains("not implemented"), "{fit}");
}

#[test]
fn optimizer_refuses_without_a_calibration_profile() {
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(detect());
    options.optimize = true;
    let err = generate_supports_with(&box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(8.0, 8.0, 4.0)), &params(), options).expect_err("need profile");
    assert!(err.to_string().contains("calibration profile"), "{err}");
}

#[test]
fn uniform_mode_still_works_without_a_calibration_profile() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 8.0, 3.0));
    let result = generate_supports(&part, &params(), &[], true).expect("uniform");
    assert!(result.optimize.is_none());
}

#[test]
fn optimizer_uses_fixture_profile_and_can_upgrade_the_palette() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 8.0));
    let profile: CalibrationProfile = parse_calibration(&fixture_profile(0.01)).expect("profile");
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(detect());
    options.optimize = true;
    options.calibration = Some(&profile);
    let result = generate_supports_with(&part, &params(), options).expect("optimize");
    let forces = result.optimize.expect("forces");
    assert!(forces.demand_n > 0.0, "demand {}", forces.demand_n);
    assert!(forces.peel_n > 0.0, "peel should track layer area");
    assert!(forces.weight_n > 0.0, "weight should track volume and density");
    assert!(!result.points.is_empty());
    // Tiny tip capacity in the fixture forces a heavier class or a short-check warning.
    assert!(
        forces.sufficient || result.warnings.iter().any(|w| w.contains("holding-force check is short")),
        "forces={forces:?} warnings={:?}",
        result.warnings
    );
}

#[test]
fn detect_params_are_validated() {
    let mesh = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0));
    let err = detect_islands(&mesh, &IslandDetectParams { layer_height_mm: 0.0, cell_mm: 0.5 }).expect_err("layer");
    assert!(err.to_string().contains("layer height"), "{err}");
    let err = detect_islands(&mesh, &IslandDetectParams { layer_height_mm: 0.5, cell_mm: -1.0 }).expect_err("cell");
    assert!(err.to_string().contains("cell size"), "{err}");
}

#[test]
fn island_ignore_area_skips_small_islands() {
    let mesh = mid_air_mesh();
    let features = detect_islands(&mesh, &detect()).expect("detect");
    let small = features.islands.iter().map(|i| i.area_mm2).fold(f64::INFINITY, f64::min);
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(detect());
    options.island_ignore_area_mm2 = Some(small + 1.0);
    let result = generate_supports_with(&mesh, &params(), options).expect("supports");
    let ignored = result.features.unwrap().islands.iter().filter(|i| i.area_mm2 <= small + 1.0).count();
    assert!(ignored >= 1);
}
