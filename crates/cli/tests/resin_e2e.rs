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

#[test]
fn gui_export_bytes_match_cli_for_the_same_inputs() {
    // Same STL, support geometry, and --params-from settings: library path used by the GUI
    // must write the same bytes as `campanion resin`.
    let stl = tmp("resin_gui_roundtrip.stl");
    box_stl(&stl, [8.0, 6.0, 2.0]);
    let sample = tmp("resin_gui_roundtrip_sample.pm7m");
    let cli_out = tmp("resin_gui_roundtrip_cli.pm7m");
    let (s, sample_s, cli_s) = (stl.to_str().expect("utf8"), sample.to_str().expect("utf8"), cli_out.to_str().expect("utf8"));

    let mut make_sample = vec!["resin", s, "-o", sample_s];
    make_sample.extend_from_slice(&ALL);
    make_sample.extend_from_slice(&["--lift-speed", "6"]);
    let made = run(&make_sample);
    assert!(made.status.success(), "{}", String::from_utf8_lossy(&made.stderr));

    let support = [
        "--supports",
        "--layer-height",
        "0.1",
        "--support-spacing",
        "3",
        "--overhang-angle",
        "45",
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
        "--raft-thickness",
        "1.5",
        "--raft-chamfer-angle",
        "30",
        "--raft-margin",
        "3",
        "--raft-corner-relief",
        "0",
    ];
    let mut cli_args = vec!["resin", s, "-o", cli_s, "--params-from", sample_s];
    cli_args.extend_from_slice(&support);
    let cli = run(&cli_args);
    assert!(cli.status.success(), "{}", String::from_utf8_lossy(&cli.stderr));
    let cli_bytes = std::fs::read(&cli_out).expect("cli bytes");

    let mesh = campanion_mesh::parse_stl(&std::fs::read(&stl).expect("stl")).expect("parse");
    let settings = campanion_resin::load_print_settings(&sample).expect("settings");
    let params = campanion_resin::SupportParams {
        raft_chamfer_angle_deg: 30.0,
        raft_thickness_mm: 1.5,
        raft_margin_mm: 3.0,
        raft_corner_relief_mm: 0.0,
        overhang_angle_deg: 45.0,
        support_spacing_mm: 3.0,
        tip_diameter_mm: 0.6,
        tip_length_mm: 1.2,
        pillar_diameter_mm: 1.2,
        foot_diameter_mm: 3.0,
        foot_height_mm: 0.8,
        part_lift_mm: 5.0,
    };
    let mut options = campanion_resin::GenerateOptions::auto(&[]);
    options.detect = Some(campanion_resin::IslandDetectParams { layer_height_mm: 0.1, cell_mm: 3.0 });
    let generated = campanion_resin::generate_supports_with(&mesh, &params, options).expect("supports");
    let (gui_bytes, report) =
        campanion_resin::build_pm7m(&generated.mesh, &campanion_resin::M7_MAX, &settings, campanion_resin::Placement::Center).expect("gui build");
    assert_eq!(report.layers, campanion_resin::read_pm7m(&cli_bytes).expect("cli read").layers.len());
    assert_eq!(gui_bytes, cli_bytes, "GUI export bytes must match `campanion resin` for the same inputs");
}
