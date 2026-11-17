//! `campanion resin` and `resin-info` end to end. Nothing here has been printed.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::PathBuf;
use std::process::{Command, Output};

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output().expect("run campanion")
}

/// Binary STL of an axis-aligned box.
fn box_stl(path: &PathBuf, size: [f32; 3]) {
    let [x, y, z] = size;
    let v = [[0.0, 0.0, 0.0], [x, 0.0, 0.0], [x, y, 0.0], [0.0, y, 0.0], [0.0, 0.0, z], [x, 0.0, z], [x, y, z], [0.0, y, z]];
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

const ALL: [&str; 14] = [
    "--layer-height",
    "0.1",
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
];

#[test]
fn writes_checks_and_reuses_a_pm7m() {
    let stl = tmp("resin_box.stl");
    let out = tmp("resin_box.pm7m");
    box_stl(&stl, [8.0, 6.0, 2.0]);
    let (s, o) = (stl.to_str().expect("utf8"), out.to_str().expect("utf8"));
    let mut args = vec!["resin", s, "-o", o];
    args.extend_from_slice(&ALL);
    args.extend_from_slice(&["--lift-speed", "6"]);
    let result = run(&args);
    assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
    let err = String::from_utf8_lossy(&result.stderr);
    assert!(err.contains("20 layers"), "{err}");
    assert!(err.contains("NOT printer-tested"), "{err}");

    let info = run(&["resin-info", o]);
    assert!(info.status.success());
    let text = String::from_utf8_lossy(&info.stdout);
    assert!(text.contains("machine: Anycubic Photon Mono M7 Max"), "{text}");
    assert!(text.contains("screen: 6480 x 3600"), "{text}");
    assert!(text.contains("layers: 20"), "{text}");
    assert!(text.contains("layer 0: exposure 20 s, lift 4 at speed 2"), "{text}");

    // Settings can come from that file; a flag overrides it.
    let out2 = tmp("resin_box2.pm7m");
    let o2 = out2.to_str().expect("utf8");
    let again = run(&["resin", s, "-o", o2, "--params-from", o, "--exposure", "3"]);
    assert!(again.status.success(), "{}", String::from_utf8_lossy(&again.stderr));
    let info2 = String::from_utf8_lossy(&run(&["resin-info", o2]).stdout).into_owned();
    assert!(info2.contains("last layer: exposure 3 s, lift 8 at speed 6"), "{info2}");
    assert!(info2.contains("layer 0: exposure 20 s"), "{info2}");
}

#[test]
fn nothing_is_assumed_and_bad_input_is_refused() {
    let stl = tmp("resin_box_b.stl");
    box_stl(&stl, [8.0, 6.0, 2.0]);
    let s = stl.to_str().expect("utf8");
    let o = tmp("resin_never.pm7m");
    let ostr = o.to_str().expect("utf8");
    let missing = run(&["resin", s, "-o", ostr, "--layer-height", "0.05"]);
    assert_eq!(missing.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&missing.stderr).contains("no defaults are assumed"));
    assert!(!o.exists());

    let same = run(&["resin", s, "-o", s]);
    assert_eq!(same.status.code(), Some(2));

    let wide = tmp("resin_wide.stl");
    box_stl(&wide, [10.0, 400.0, 2.0]);
    let mut args = vec!["resin", wide.to_str().expect("utf8"), "-o", ostr];
    args.extend_from_slice(&ALL);
    args.extend_from_slice(&["--lift-speed", "6"]);
    let too_big = run(&args);
    assert_eq!(too_big.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&too_big.stderr).contains("does not fit"));
    assert!(!o.exists());

    let not_pm7m = run(&["resin-info", s]);
    assert_eq!(not_pm7m.status.code(), Some(1));
}
