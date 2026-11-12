//! #146: explicit failures and override behavior at the CLI boundary.

#![allow(clippy::expect_used)]

use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Document, Feeds, MoveRole, Pose, Program, Tool, ToolShape, Vec3};
use serde_json::json;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn directory(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("campanion-tool-shape-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("directory");
    dir
}

fn run(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_campanion")).current_dir(dir).args(args).output().expect("CLI")
}

fn write_ir(dir: &Path, shape: Option<ToolShape>) {
    let tool = Tool::try_new(1, None, Some(2.0), None).expect("tool");
    let tool = match shape {
        Some(shape) => tool.with_shape(shape).expect("shape"),
        None => tool,
    };
    let pose = |x| Pose::new(Vec3::new(x, 0.0, 1.0).expect("point"), Vec3::Z).expect("pose");
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: pose(1.0), feed: 100.0, role: MoveRole::Cut, power: None }),
    ];
    let path = ClPath::try_new(
        None,
        Some(1),
        CompensationMode::Off,
        Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path");
    let program = Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program");
    std::fs::write(dir.join("ir.json"), Document::new(program).expect("doc").to_json().expect("json")).expect("IR");
    std::fs::write(
        dir.join("part.stl"),
        "solid plane\nfacet normal 0 0 1\nouter loop\nvertex -10 -10 0\nvertex 10 -10 0\nvertex 0 10 0\nendloop\nendfacet\nendsolid plane\n",
    )
    .expect("mesh");
}

#[test]
fn missing_shape_and_vbit_fail_mesh_check_but_allow_explicit_override() {
    let dir = directory("verify");
    for (shape, expected) in [
        (None, "no cutter shape"),
        (Some(ToolShape::VBit { angle: 1.0, tip_diameter: 0.0 }), "not supported yet: mesh verification for V-bit tool 1"),
    ] {
        write_ir(&dir, shape);
        let ordinary = run(&dir, &["verify", "ir.json"]);
        assert!(ordinary.status.success(), "{}", String::from_utf8_lossy(&ordinary.stderr));
        let automatic = run(&dir, &["verify", "ir.json", "--mesh", "part.stl"]);
        assert!(!automatic.status.success());
        assert!(automatic.stdout.is_empty());
        assert!(String::from_utf8_lossy(&automatic.stderr).contains(expected));
        let override_result = run(&dir, &["verify", "ir.json", "--mesh", "part.stl", "--tool", "flat:1"]);
        assert!(override_result.status.success(), "{}", String::from_utf8_lossy(&override_result.stderr));
        assert!(String::from_utf8_lossy(&override_result.stderr).contains("overrides cutter geometry for all cutting moves"));
    }
    std::fs::remove_dir_all(dir).expect("cleanup");
}

#[test]
fn conflicting_vbit_job_fails_before_any_output_is_written() {
    let dir = directory("conflict");
    let job = json!({"format":"campanion-job/1", "source":{"path":"deliberately-missing.svg","hash":"cbf29ce484222325"}, "units":"mm", "tools":[{"id":1,"diameter":6}],
    "operations":[
        {"name":"a","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":90,"feeds":{"cut":100}},
        {"name":"b","type":"chamfer","layers":["L"],"side":"outside","tool_id":1,"width":0.5,"v_angle_deg":60,"feeds":{"cut":100}}
    ]});
    std::fs::write(dir.join("job.json"), job.to_string()).expect("job");
    let output = run(&dir, &["run", "job.json", "-o", "out.nc", "--ir", "out.json"]);
    assert!(!output.status.success());
    assert!(String::from_utf8_lossy(&output.stderr).contains("conflicting V-bit geometry"));
    assert!(!dir.join("out.nc").exists());
    assert!(!dir.join("out.json").exists());
    std::fs::remove_dir_all(dir).expect("cleanup");
}

#[test]
fn chamfer_job_copies_vbit_geometry_into_emitted_ir() {
    let dir = directory("chamfer");
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("root");
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .current_dir(&root)
        .arg("run")
        .arg("examples/chamfer.job.json")
        .arg("-o")
        .arg(dir.join("out.nc"))
        .arg("--ir")
        .arg(dir.join("out.json"))
        .output()
        .expect("CLI");
    assert!(output.status.success(), "{}", String::from_utf8_lossy(&output.stderr));
    let doc = Document::from_json(&std::fs::read_to_string(dir.join("out.json")).expect("IR")).expect("load");
    assert_eq!(doc.program().tools().first().expect("tool").shape(), Some(ToolShape::VBit { angle: std::f64::consts::FRAC_PI_2, tip_diameter: 0.0 }));
    std::fs::remove_dir_all(dir).expect("cleanup");
}
