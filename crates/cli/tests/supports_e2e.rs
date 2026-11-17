//! `campanion supports` and `campanion resin --supports` end to end. Nothing here has been printed.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::PathBuf;
use std::process::{Command, Output};

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir).join("campanion_supports_e2e");
    std::fs::create_dir_all(&dir).expect("tmp dir");
    dir.join(name)
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output().expect("run campanion")
}

fn box_stl(path: &PathBuf, min: [f32; 3], size: [f32; 3]) {
    let [x0, y0, z0] = min;
    let x1 = x0 + size[0];
    let y1 = y0 + size[1];
    let z1 = z0 + size[2];
    let v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
    let faces = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)];
    let mut bytes = vec![0u8; 80];
    bytes.extend_from_slice(&(faces.len() as u32).to_le_bytes());
    for (a, b, c) in faces {
        bytes.extend_from_slice(&[0u8; 12]);
        for i in [a, b, c] {
            for coord in v[i] {
                bytes.extend_from_slice(&f32::to_le_bytes(coord));
            }
        }
        bytes.extend_from_slice(&[0u8; 2]);
    }
    std::fs::write(path, bytes).expect("write stl");
}

const SUPPORT: [&str; 24] = [
    "--raft-chamfer-angle",
    "30",
    "--raft-thickness",
    "1.5",
    "--raft-margin",
    "3",
    "--raft-corner-relief",
    "0",
    "--overhang-angle",
    "45",
    "--support-spacing",
    "5",
    "--tip-diameter",
    "0.6",
    "--tip-length",
    "1.2",
    "--pillar-diameter",
    "1.2",
    "--foot-diameter",
    "3",
    "--foot-height",
    "0.8",
    "--part-lift",
    "5",
];

const RESIN: [&str; 16] = [
    "--layer-height",
    "0.5",
    "--bottom-layers",
    "2",
    "--bottom-exposure",
    "20",
    "--bottom-lift-height",
    "4",
    "--bottom-lift-speed",
    "2",
    "--exposure",
    "2.5",
    "--lift-height",
    "8",
    "--lift-speed",
    "6",
];

#[test]
fn writes_a_supported_stl_and_refuses_missing_params() {
    let stl = tmp("supports_box.stl");
    let out = tmp("supports_box_supported.stl");
    let preview = tmp("supports_box.png");
    box_stl(&stl, [0.0, 0.0, 0.0], [10.0, 8.0, 3.0]);
    let s = stl.to_str().expect("utf8");
    let o = out.to_str().expect("utf8");
    let p = preview.to_str().expect("utf8");

    let missing = run(&["supports", s, "-o", o, "--raft-thickness", "1.5"]);
    assert_eq!(missing.status.code(), Some(2), "{}", String::from_utf8_lossy(&missing.stderr));
    assert!(String::from_utf8_lossy(&missing.stderr).contains("no defaults are assumed"), "{}", String::from_utf8_lossy(&missing.stderr));
    assert!(!out.exists());

    let mut args = vec!["supports", s, "-o", o, "--preview", p];
    args.extend_from_slice(&SUPPORT);
    let result = run(&args);
    assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
    let err = String::from_utf8_lossy(&result.stderr);
    assert!(err.contains("1.500 mm raft"), "{err}");
    assert!(err.contains("30° chamfer"), "{err}");
    assert!(err.contains("NOT printer-tested"), "{err}");
    assert!(out.exists());
    assert!(preview.exists());
    let png = std::fs::read(&preview).expect("png");
    assert!(png.starts_with(b"\x89PNG"), "preview is not a png");
    let bytes = std::fs::read(&out).expect("stl");
    let mesh = campanion_mesh::parse_stl(&bytes).expect("parse supported stl");
    assert!(mesh.triangles().len() > 12);
}

#[test]
fn resin_can_slice_generated_supports() {
    let stl = tmp("supports_resin.stl");
    let out = tmp("supports_resin.pm7m");
    let supported = tmp("supports_resin_out.stl");
    box_stl(&stl, [0.0, 0.0, 0.0], [10.0, 8.0, 3.0]);
    let s = stl.to_str().expect("utf8");
    let o = out.to_str().expect("utf8");
    let so = supported.to_str().expect("utf8");
    let mut args = vec!["resin", s, "-o", o, "--supports", "--supports-out", so];
    args.extend_from_slice(&RESIN);
    args.extend_from_slice(&SUPPORT);
    let result = run(&args);
    assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
    let err = String::from_utf8_lossy(&result.stderr);
    assert!(err.contains("sliced with a chamfered raft"), "{err}");
    assert!(err.contains("NOT printer-tested"), "{err}");
    assert!(supported.exists());
    let info = run(&["resin-info", o]);
    assert!(info.status.success(), "{}", String::from_utf8_lossy(&info.stderr));
    let text = String::from_utf8_lossy(&info.stdout);
    assert!(text.contains("machine: Anycubic Photon Mono M7 Max"), "{text}");
    // 1.5 + 5 + 3 = 9.5 mm at 0.5 mm layers -> 19 layers
    assert!(text.contains("layers: 19"), "{text}");
}
