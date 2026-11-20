//! Painted zones, cross-bracing, tip penetration, preset mixes and slice islands. Nothing here has been printed.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use campanion_mesh::{Aabb, Mesh, Vec3};
use campanion_resin::supports::{SolidOptions, ZoneMark};
use campanion_resin::{
    BracingParams, FaceZones, GenerateOptions, InputPoint, LayerParams, M7_MAX, Placement, PointSource, PresetMix, ResinSettings, SupportParams,
    TipPreset, build_pm7m, generate_supports_with, is_watertight,
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

/// A post with a wide plate on top, so the plate underside is an overhang.
fn table_mesh() -> Mesh {
    let post = box_mesh(Vec3::new(-2.0, -2.0, 0.0), Vec3::new(2.0, 2.0, 8.0));
    let plate = box_mesh(Vec3::new(-10.0, -10.0, 8.0), Vec3::new(10.0, 10.0, 10.0));
    let mut tris = post.triangles().to_vec();
    tris.extend_from_slice(plate.triangles());
    Mesh::new(tris).expect("table")
}

fn settings() -> ResinSettings {
    ResinSettings {
        layer_height_mm: 0.05,
        bottom_layers: 2,
        bottom: LayerParams { exposure_s: 20.0, lift_height_mm: 4.0, lift_speed: 2.0 },
        normal: LayerParams { exposure_s: 2.5, lift_height_mm: 8.0, lift_speed: 6.0 },
        resin_name: "test".to_string(),
    }
}

fn presets() -> Vec<TipPreset> {
    vec![
        TipPreset {
            name: "Light".to_string(),
            tip_diameter_mm: 0.3,
            tip_length_mm: 1.0,
            pillar_diameter_mm: 0.8,
            foot_diameter_mm: 2.0,
            foot_height_mm: 0.5,
            tip_penetration_mm: None,
        },
        TipPreset {
            name: "Heavy".to_string(),
            tip_diameter_mm: 0.8,
            tip_length_mm: 1.4,
            pillar_diameter_mm: 1.6,
            foot_diameter_mm: 3.2,
            foot_height_mm: 0.8,
            tip_penetration_mm: Some(0.3),
        },
    ]
}

#[test]
fn no_support_zone_removes_auto_supports_under_it() {
    let part = table_mesh();
    let plain = generate_supports_with(&part, &params(), GenerateOptions::auto(&[])).unwrap();
    assert!(plain.points.len() > 4);
    let mut zones = FaceZones::new(part.triangles().len());
    // Block the plate underside on the +X half, seen from below.
    zones.paint(&part, Vec3::new(10.0, 0.0, 8.0), 10.5, ZoneMark::Block, Some(Vec3::new(0.0, 0.0, 1.0))).unwrap();
    let mut options = GenerateOptions::auto(&[]);
    options.zones = Some(&zones);
    let blocked = generate_supports_with(&part, &params(), options).unwrap();
    let report = blocked.zone_report.clone().unwrap();
    assert!(report.blocked_removed > 0);
    assert_eq!(blocked.points.len() + report.blocked_removed, plain.points.len());
    assert!(blocked.warnings.iter().any(|w| w.contains("no-support zones")));
}

#[test]
fn painted_support_zone_adds_supports_below_the_overhang_angle() {
    // A shallow-sloped underside the 45° rule ignores: ramp body lifted above the plate.
    let part = table_mesh();
    let mut p = params();
    p.overhang_angle_deg = 90.0; // only exactly flat faces count; still none get removed.
    let base = generate_supports_with(&part, &p, GenerateOptions::auto(&[])).unwrap();
    let mut zones = FaceZones::new(part.triangles().len());
    zones.paint(&part, Vec3::new(0.0, 0.0, 8.0), 30.0, ZoneMark::Support, Some(Vec3::new(0.0, 0.0, 1.0))).unwrap();
    let mut options = GenerateOptions::manual(&[]);
    options.zones = Some(&zones);
    // Spacing is required once support faces are painted.
    assert!(generate_supports_with(&part, &p, options.clone()).is_err());
    options.zone_spacing_mm = Some(2.0);
    let painted = generate_supports_with(&part, &p, options).unwrap();
    assert!(painted.points.iter().all(|pt| pt.source == PointSource::Painted));
    assert!(painted.points.len() > base.points.len(), "{} vs {}", painted.points.len(), base.points.len());
    assert!(is_watertight(&painted.pillars).unwrap());
}

#[test]
fn tip_penetration_sinks_the_tip_into_the_part() {
    let part = table_mesh();
    let mut options = GenerateOptions::auto(&[]);
    let flush = generate_supports_with(&part, &params(), options.clone()).unwrap();
    options.solids = SolidOptions { tip_penetration_mm: Some(0.4), bracing: None };
    let deep = generate_supports_with(&part, &params(), options.clone()).unwrap();
    let top = |m: &Mesh| m.bounds().unwrap().max().z;
    let contact = flush.points.iter().map(|p| p.contact_z).fold(f64::NEG_INFINITY, f64::max);
    assert!((top(&flush.pillars) - contact).abs() < 1e-9);
    assert!((top(&deep.pillars) - (contact + 0.4)).abs() < 1e-9);
    assert!(is_watertight(&deep.pillars).unwrap());
    options.solids.tip_penetration_mm = Some(-0.1);
    assert!(generate_supports_with(&part, &params(), options).is_err());
}

#[test]
fn bracing_links_pillars_and_stays_off_the_part() {
    let part = table_mesh();
    let mut options = GenerateOptions::auto(&[]);
    options.solids.bracing = Some(BracingParams { diameter_mm: 0.5, max_span_mm: 6.0, pitch_mm: 2.0 });
    let braced = generate_supports_with(&part, &params(), options.clone()).unwrap();
    assert!(braced.brace_count > 0);
    // Each strut is a closed 8-sided prism (32 triangles); zig-zag struts meet end to end.
    assert_eq!(braced.braces.triangles().len(), braced.brace_count * 32);
    // No strut reaches the plate underside or the post. Lifted: post from z 6.5, plate underside at 14.5.
    let lift = params().raft_thickness_mm + params().part_lift_mm;
    for tri in braced.braces.triangles() {
        for v in [tri.a, tri.b, tri.c] {
            assert!(v.z < 8.0 + lift, "strut vertex {v:?} is in the plate");
            let in_post_column = v.x.abs() < 2.0 && v.y.abs() < 2.0;
            assert!(!(in_post_column && v.z >= lift), "strut vertex {v:?} is in the post");
        }
    }
    options.solids.bracing = Some(BracingParams { diameter_mm: 0.5, max_span_mm: 0.0, pitch_mm: 2.0 });
    assert!(generate_supports_with(&part, &params(), options).is_err());
}

#[test]
fn preset_mix_assigns_saved_geometry_by_role_and_points_file_names() {
    let part = table_mesh();
    let library = presets();
    let mix = PresetMix::parse("overhang=Light").unwrap();
    let extra = vec![InputPoint { x: 1.0, y: 1.0, z: None, preset: Some("Heavy".to_string()), source: PointSource::Manual }];
    let mut options = GenerateOptions::auto(&extra);
    options.presets = &library;
    options.mix = Some(&mix);
    let mixed = generate_supports_with(&part, &params(), options.clone()).unwrap();
    let manual: Vec<_> = mixed.points.iter().filter(|p| p.source == PointSource::Manual).collect();
    assert!(manual.iter().all(|p| p.preset_index == Some(1)));
    assert!(mixed.points.iter().filter(|p| p.source == PointSource::Overhang).all(|p| p.preset_index == Some(0)));
    assert_eq!(mixed.palette_names, vec!["Light".to_string(), "Heavy".to_string()]);
    // The Heavy preset's own penetration applies to its tip.
    let top = mixed.pillars.bounds().unwrap().max().z;
    let heavy_contact = manual.iter().map(|p| p.contact_z).fold(f64::NEG_INFINITY, f64::max);
    assert!(top >= heavy_contact + 0.3 - 1e-9);
    let bad = PresetMix::parse("island=Missing").unwrap();
    options.mix = Some(&bad);
    assert!(generate_supports_with(&part, &params(), options).is_err());
}

#[test]
fn slice_islands_flag_a_floating_body_and_not_a_supported_one() {
    // A body floating 3 mm above a base, with no supports, starts as an island.
    let base = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 1.0));
    let floating = box_mesh(Vec3::new(2.0, 2.0, 4.0), Vec3::new(6.0, 6.0, 5.0));
    let mut tris = base.triangles().to_vec();
    tris.extend_from_slice(floating.triangles());
    let mesh = Mesh::new(tris).unwrap();
    let (_, report) = build_pm7m(&mesh, &M7_MAX, &settings(), Placement::Center).unwrap();
    assert_eq!(report.islands.total(), 1);
    let island = &report.islands.islands[0];
    assert_eq!(island.layer, 80);
    let area = island.pixels as f64 * report.pixel_area_mm2;
    assert!((area - 16.0).abs() < 1.0, "{area}");
    // A supported part has none.
    let supported = generate_supports_with(&table_mesh(), &params(), GenerateOptions::auto(&[])).unwrap();
    let (_, report) = build_pm7m(&supported.mesh, &M7_MAX, &settings(), Placement::Center).unwrap();
    assert_eq!(report.islands.total(), 0, "{:?}", report.islands.islands.iter().map(|i| (i.layer, i.pixels)).collect::<Vec<_>>());
}
