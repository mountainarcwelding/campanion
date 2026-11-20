//! Pose transforms and cancellable island detection. Synthetic cases only (no private parts).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use campanion_mesh::{Aabb, Mesh, Vec3};
use campanion_resin::{
    CancelFlag, GenerateOptions, GenerateProgress, IslandDetectParams, MeshPose, SupportParams, apply_mesh_pose, center_on_plate, detect_islands,
    detect_islands_with, drop_to_plate, generate_supports_with, snap_to_step,
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

fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9, "{a} != {b}");
}

#[test]
fn empty_pose_leaves_an_off_center_part_where_it_is() {
    // Same situation as a CAD export whose XY is already a plate coordinate, not plate-centred.
    let part = box_mesh(Vec3::new(135.0, 70.0, 0.0), Vec3::new(163.0, 96.0, 61.0));
    let posed = apply_mesh_pose(&part, MeshPose::empty()).expect("pose");
    let (min, max) = {
        let b = posed.bounds().expect("b");
        (b.min(), b.max())
    };
    near(min.x, 135.0);
    near(min.y, 70.0);
    near(max.x, 163.0);
    near(max.y, 96.0);
    near(min.z, 0.0);
    near(max.z, 61.0);
}

#[test]
fn typed_translation_moves_the_bounding_box_centre() {
    let part = box_mesh(Vec3::new(135.0, 70.0, 0.0), Vec3::new(163.0, 96.0, 61.0));
    let pose = MeshPose { position_mm: [Some(0.0), Some(0.0), None], rotation_deg: [None, None, None] };
    let posed = apply_mesh_pose(&part, pose).expect("pose");
    let (min, max) = {
        let b = posed.bounds().expect("b");
        (b.min(), b.max())
    };
    near((min.x + max.x) / 2.0, 0.0);
    near((min.y + max.y) / 2.0, 0.0);
    near(min.z, 0.0);
    near(max.z, 61.0);
}

#[test]
fn center_on_plate_moves_xy_to_the_origin_and_keeps_z() {
    let part = box_mesh(Vec3::new(135.0, 70.0, 5.0), Vec3::new(163.0, 96.0, 66.0));
    let (posed, pose) = center_on_plate(&part).expect("center");
    let (min, max) = {
        let b = posed.bounds().expect("b");
        (b.min(), b.max())
    };
    near((min.x + max.x) / 2.0, 0.0);
    near((min.y + max.y) / 2.0, 0.0);
    near(min.z, 5.0);
    near(max.z, 66.0);
    assert_eq!(pose.position_mm[0], Some(0.0));
    assert_eq!(pose.position_mm[1], Some(0.0));
    assert_eq!(pose.position_mm[2], None);
}

#[test]
fn drop_to_plate_sets_the_lowest_point_to_zero() {
    // A part exported with geometry below and above the plate.
    let part = box_mesh(Vec3::new(-10.0, -10.0, -42.0), Vec3::new(10.0, 10.0, 54.0));
    let (posed, pose) = drop_to_plate(&part).expect("drop");
    let (min, max) = {
        let b = posed.bounds().expect("b");
        (b.min(), b.max())
    };
    near(min.z, 0.0);
    near(max.z, 96.0);
    near((min.x + max.x) / 2.0, 0.0);
    assert_eq!(pose.position_mm[2], Some(48.0));
}

#[test]
fn rotation_about_the_bounding_box_centre_keeps_the_centre() {
    let part = box_mesh(Vec3::new(100.0, 50.0, 0.0), Vec3::new(120.0, 60.0, 10.0));
    let pose = MeshPose { position_mm: [None, None, None], rotation_deg: [None, None, Some(90.0)] };
    let posed = apply_mesh_pose(&part, pose).expect("pose");
    let (min, max) = {
        let b = posed.bounds().expect("b");
        (b.min(), b.max())
    };
    near((min.x + max.x) / 2.0, 110.0);
    near((min.y + max.y) / 2.0, 55.0);
    // 20 x 10 box rotated 90° about Z becomes 10 x 20, still centred.
    near(max.x - min.x, 10.0);
    near(max.y - min.y, 20.0);
}

#[test]
fn snap_to_step_rounds_to_the_nearest_increment() {
    near(snap_to_step(1.24, 0.1).expect("snap"), 1.2);
    near(snap_to_step(1.25, 0.1).expect("snap"), 1.3);
    near(snap_to_step(-1.24, 1.0).expect("snap"), -1.0);
    assert!(snap_to_step(1.0, 0.0).is_err());
    assert!(snap_to_step(1.0, f64::NAN).is_err());
}

#[test]
fn pre_cancelled_detection_returns_cancelled_without_walking_layers() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 50.0));
    let cancel = CancelFlag::new();
    cancel.cancel();
    let start = Instant::now();
    let err =
        detect_islands_with(&part, &IslandDetectParams { layer_height_mm: 0.05, cell_mm: 0.5 }, Some(&cancel), None).expect_err("should cancel");
    assert!(err.is_cancelled(), "{err}");
    assert!(start.elapsed() < Duration::from_secs(1), "pre-cancelled detect took {:?}", start.elapsed());
}

#[test]
fn detection_stops_when_cancelled_mid_walk() {
    // A tall block with a fine layer height is a synthetic stand-in for a dense real part:
    // walking every layer on the UI thread is what made Generate look like a crash.
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 80.0));
    let cancel = Arc::new(CancelFlag::new());
    let progress = Arc::new(GenerateProgress::new());
    let cancel_watch = Arc::clone(&cancel);
    let progress_watch = Arc::clone(&progress);
    thread::spawn(move || {
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if progress_watch.layer() >= 2 {
                cancel_watch.cancel();
                return;
            }
            thread::sleep(Duration::from_millis(1));
        }
        cancel_watch.cancel();
    });
    let start = Instant::now();
    let err = detect_islands_with(&part, &IslandDetectParams { layer_height_mm: 0.01, cell_mm: 0.5 }, Some(cancel.as_ref()), Some(progress.as_ref()))
        .expect_err("should cancel mid-walk");
    assert!(err.is_cancelled(), "{err}");
    // A full walk of 80 / 0.01 = 8000 layers would take much longer than a couple of seconds
    // if it ignored the flag; this must stop promptly once a few layers have started.
    assert!(start.elapsed() < Duration::from_secs(3), "cancel was ignored: {:?}", start.elapsed());
    assert!(progress.layer() < 8000, "walked every layer anyway: {}", progress.layer());
}

#[test]
fn generate_honours_cancel_and_an_off_center_pose() {
    let part = box_mesh(Vec3::new(135.0, 70.0, 0.0), Vec3::new(155.0, 90.0, 12.0));
    let centered = apply_mesh_pose(&part, MeshPose { position_mm: [Some(0.0), Some(0.0), None], rotation_deg: [None, None, None] }).expect("pose");
    let (min, max) = {
        let b = centered.bounds().expect("b");
        (b.min(), b.max())
    };
    near((min.x + max.x) / 2.0, 0.0);
    near((min.y + max.y) / 2.0, 0.0);

    let cancel = CancelFlag::new();
    cancel.cancel();
    let mut options = GenerateOptions::auto(&[]);
    options.detect = Some(IslandDetectParams { layer_height_mm: 0.05, cell_mm: 0.5 });
    options.cancel = Some(&cancel);
    let err = generate_supports_with(&centered, &params(), options).expect_err("cancelled generate");
    assert!(err.is_cancelled(), "{err}");
}

#[test]
fn detect_islands_still_works_without_a_cancel_flag() {
    let part = box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(8.0, 8.0, 4.0));
    let features = detect_islands(&part, &IslandDetectParams { layer_height_mm: 0.5, cell_mm: 0.5 }).expect("detect");
    assert!(!features.layers.is_empty());
}
