#![allow(clippy::unwrap_used, clippy::expect_used)]
use campanion_mesh::{Aabb, Mesh, Triangle, Vec3};
use campanion_resin::pm7m::{LayerParams, M7_MAX, ResinSettings, read_pm7m};
use campanion_resin::slice::{PreparedMesh, spans_to_dense};
use campanion_resin::{Placement, build_pm7m, pw0};

fn boxm(a: [f64; 3], b: [f64; 3], inward: bool) -> Vec<Triangle> {
    let tris = Aabb::from_corners(Vec3::new(a[0], a[1], a[2]), Vec3::new(b[0], b[1], b[2])).face_triangles();
    tris.iter().map(|t| if inward { Triangle { a: t.a, b: t.c, c: t.b } } else { *t }).collect()
}

fn settings() -> ResinSettings {
    ResinSettings {
        layer_height_mm: 0.1,
        bottom_layers: 2,
        bottom: LayerParams { exposure_s: 20.0, lift_height_mm: 4.0, lift_speed: 2.0 },
        normal: LayerParams { exposure_s: 2.5, lift_height_mm: 8.0, lift_speed: 6.0 },
        resin_name: "test".into(),
    }
}

fn prep(tris: Vec<Triangle>, placement: Placement) -> PreparedMesh {
    let mesh = Mesh::new(tris).unwrap();
    PreparedMesh::new(&mesh, M7_MAX.grid(), 300.0, 0.1, placement).unwrap()
}

fn area_mm2(px: u64) -> f64 {
    let g = M7_MAX.grid();
    px as f64 * g.pitch_x_mm * g.pitch_y_mm
}

#[test]
fn cube_area_layers_and_centre() {
    let p = prep(boxm([0.0, 0.0, 0.0], [10.0, 10.0, 2.0], false), Placement::Center);
    assert_eq!(p.layer_count(), 20);
    for layer in [0, 10, 19] {
        let spans = p.layer_spans(layer);
        let area = area_mm2(PreparedMesh::lit_pixels(&spans));
        assert!((area - 100.0).abs() < 1.5, "layer {layer}: {area}");
        let (min_c, max_c) = (spans.iter().map(|s| s.start).min().unwrap(), spans.iter().map(|s| s.end).max().unwrap());
        assert!((i64::from(min_c) + i64::from(max_c) - 6480).abs() <= 2);
        let (min_r, max_r) = (spans.iter().map(|s| s.row).min().unwrap(), spans.iter().map(|s| s.row).max().unwrap() + 1);
        assert!((i64::from(min_r) + i64::from(max_r) - 3600).abs() <= 2);
    }
}

#[test]
fn axes_plus_x_right_plus_y_up() {
    let p = prep(boxm([20.0, 10.0, 0.0], [30.0, 20.0, 1.0], false), Placement::Keep);
    let spans = p.layer_spans(0);
    assert!(spans.iter().all(|s| s.start >= 3240 && s.row < 1800));
    assert!(!spans.is_empty());
}

#[test]
fn overlapping_shells_union_and_inverted_shell_is_a_hole() {
    let mut two = boxm([0.0, 0.0, 0.0], [10.0, 10.0, 1.0], false);
    two.extend(boxm([5.0, 0.0, 0.0], [15.0, 10.0, 1.0], false));
    let p = prep(two, Placement::Keep);
    let area = area_mm2(PreparedMesh::lit_pixels(&p.layer_spans(0)));
    assert!((area - 150.0).abs() < 2.0, "{area}");
    let mut holed = boxm([0.0, 0.0, 0.0], [10.0, 10.0, 1.0], false);
    holed.extend(boxm([2.0, 2.0, 0.0], [8.0, 8.0, 1.0], true));
    // the inner shell is a through hole only when it spans every layer: here it does.
    let p = prep(holed, Placement::Keep);
    let area = area_mm2(PreparedMesh::lit_pixels(&p.layer_spans(0)));
    assert!((area - 64.0).abs() < 2.0, "{area}");
}

#[test]
fn rejects_oversize_and_tall_models() {
    let big = Mesh::new(boxm([0.0, 0.0, 0.0], [200.0, 10.0, 1.0], false)).unwrap();
    assert!(PreparedMesh::new(&big, M7_MAX.grid(), 300.0, 0.1, Placement::Center).is_ok());
    let wide = Mesh::new(boxm([0.0, 0.0, 0.0], [10.0, 200.0, 1.0], false)).unwrap();
    assert!(PreparedMesh::new(&wide, M7_MAX.grid(), 300.0, 0.1, Placement::Center).is_err());
    let tall = Mesh::new(boxm([0.0, 0.0, 0.0], [10.0, 10.0, 301.0], false)).unwrap();
    assert!(PreparedMesh::new(&tall, M7_MAX.grid(), 300.0, 0.1, Placement::Center).is_err());
    let m = Mesh::new(boxm([0.0, 0.0, 0.0], [10.0, 10.0, 1.0], false)).unwrap();
    for h in [0.0, -1.0, f64::NAN, 2.0] {
        assert!(PreparedMesh::new(&m, M7_MAX.grid(), 300.0, h, Placement::Center).is_err());
    }
}

#[test]
fn spans_encode_and_decode_as_dense() {
    let p = prep(boxm([0.0, 0.0, 0.0], [3.0, 2.0, 0.5], false), Placement::Center);
    let spans = p.layer_spans(2);
    let bytes = campanion_resin::encode_spans(&spans, 6480, 3600).unwrap();
    let dense = pw0::decode(&bytes, 6480, 3600).unwrap();
    assert_eq!(dense, spans_to_dense(&spans, 6480, 3600));
    assert_eq!(pw0::encode(&dense).unwrap(), bytes);
}

#[test]
fn build_writes_a_readable_pm7m() {
    let mesh = Mesh::new(boxm([0.0, 0.0, 0.0], [6.0, 4.0, 1.0], false)).unwrap();
    let (bytes, report) = build_pm7m(&mesh, &M7_MAX, &settings(), Placement::Center).unwrap();
    assert_eq!(report.layers, 10);
    assert!((report.volume_ml - 0.024).abs() < 0.001, "{}", report.volume_ml);
    let file = read_pm7m(&bytes).unwrap();
    assert_eq!(file.machine_name, "Anycubic Photon Mono M7 Max");
    assert_eq!(file.layers.len(), 10);
    assert_eq!(file.layers[0].exposure_time, 20.0);
    assert_eq!(file.layers[1].zup_height, 4.0);
    assert_eq!(file.layers[2].exposure_time, 2.5);
    assert_eq!(file.layers[9].zup_speed, 6.0);
    assert!((file.layers[9].min_height - 0.9).abs() < 1e-9);
    assert!((f64::from(file.scene_heights[9]) - 1.0).abs() < 1e-6);
    assert_eq!(file.entry_names.first().map(String::as_str), Some("anycubic_photon_resins.pwsp"));
    assert!(file.preview_sizes.contains(&("preview_images/preview_0.png".to_string(), 224, 168)));
    let back = file.settings().unwrap();
    assert_eq!(back.bottom_layers, 2);
    assert_eq!(back.normal, settings().normal);
    // deterministic
    let (again, _) = build_pm7m(&mesh, &M7_MAX, &settings(), Placement::Center).unwrap();
    assert_eq!(bytes, again);
}

#[test]
fn settings_are_validated() {
    let mut s = settings();
    s.normal.exposure_s = 0.0;
    assert!(s.validate().is_err());
    let mut s = settings();
    s.bottom.lift_speed = f64::NAN;
    assert!(s.validate().is_err());
    let mut s = settings();
    s.resin_name.clear();
    assert!(s.validate().is_err());
}
