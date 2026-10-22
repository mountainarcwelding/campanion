//! Waterline finish of `examples/dome_waterline.job.json`.
//!
//! A 6 mm ball, a 2.5 mm step-down, a 1 mm grid and a 30° minimum slope. `run` must be clean, which includes
//! the STL gouge check, and `verify` must report no issues. Cuts exist at more than one level.
//! Every plunge and cut point stays at or below the drop-cutter tip plus 0.001 mm.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_mesh::{Cutter, DropCutter, parse_stl};
use serde_json::Value;

const TIP_TOL_MM: f64 = 1.0e-3;
const LEVEL_TOL_MM: f64 = 1.0e-6;

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn output_text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(cwd: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

fn run_job(cwd: &Path, job: &str, ngc_path: &Path, ir_path: &Path) -> Result<(), String> {
    let ngc_arg = path_arg(ngc_path)?;
    let ir_arg = path_arg(ir_path)?;
    let output = campanion(cwd, &["run", job, "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{job}: run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("{job}: source hash did not match: {stderr}"));
    }
    Ok(())
}

fn assert_verify_clean(root: &Path, job: &str, ir_path: &Path) -> Result<(), String> {
    let ir_arg = path_arg(ir_path)?;
    let output = campanion(root, &["verify", &ir_arg])?;
    let stdout = output_text(&output.stdout)?;
    let stderr = output_text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("{job}: verify exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    let status_ok = stdout.lines().any(|line| line == "status ok");
    let no_issues = stdout.lines().any(|line| line == "issues 0");
    if status_ok && no_issues { Ok(()) } else { Err(format!("{job}: verify reported errors:\n{stdout}{stderr}")) }
}

fn tip_of(block: &Value) -> Result<(f64, f64, f64), String> {
    let tip = &block["to"]["tip"];
    Ok((tip["x"].as_f64().ok_or("tip x")?, tip["y"].as_f64().ok_or("tip y")?, tip["z"].as_f64().ok_or("tip z")?))
}

fn note_level(levels: &mut Vec<f64>, z: f64) {
    if levels.last().is_none_or(|prev| (prev - z).abs() > LEVEL_TOL_MM) {
        levels.push(z);
    }
}

#[test]
fn dome_waterline_is_clean_and_stays_on_the_contact() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-waterline-e2e.ngc");
    let ir_path = temp_path("campanion-waterline-e2e.json");
    run_job(&root, "examples/dome_waterline.job.json", &ngc_path, &ir_path)?;
    assert_verify_clean(&root, "examples/dome_waterline.job.json", &ir_path)?;

    let stl = std::fs::read(root.join("examples/dome.stl")).map_err(|err| format!("read stl: {err}"))?;
    let mesh = parse_stl(&stl).map_err(|err| format!("parse stl: {err}"))?;
    let floor = mesh.bounds().map_err(|err| format!("stl bounds: {err}"))?.min().z - 1.0;
    let cutter = Cutter::new_ball(3.0).map_err(|err| format!("ball: {err}"))?;
    let dc = DropCutter::new(&mesh, cutter).map_err(|err| format!("drop cutter: {err}"))?;

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir["program"]["toolpaths"].as_array().ok_or("toolpaths is not an array")?;
    if toolpaths.len() != 1 {
        return Err(format!("dome waterline produced {} toolpaths", toolpaths.len()));
    }
    let blocks = toolpaths[0]["blocks"].as_array().ok_or("blocks is not an array")?;

    let mut levels = Vec::new();
    let mut checked = 0usize;
    let mut process_on = 0usize;
    let mut process_off = 0usize;
    let mut failures = Vec::new();
    for block in blocks {
        let kind = block["type"].as_str().unwrap_or("");
        if kind == "process_on" {
            process_on += 1;
            continue;
        }
        if kind == "process_off" {
            process_off += 1;
            continue;
        }
        if kind != "linear" {
            continue;
        }
        let role = block["role"].as_str().unwrap_or("");
        if role != "cut" && role != "plunge" {
            continue;
        }
        let (x, y, z) = tip_of(block)?;
        note_level(&mut levels, z);
        let tip = dc.tip_z_or(x, y, floor);
        checked += 1;
        if tip > z + TIP_TOL_MM && failures.len() < 8 {
            failures.push(format!("({x}, {y}, {z}) tip {tip}"));
        }
    }
    if process_on != 1 || process_off != 1 {
        return Err(format!("process on {process_on} off {process_off}"));
    }
    if levels.len() < 2 {
        return Err(format!("expected cuts on several levels, got {levels:?}"));
    }
    if checked == 0 {
        return Err("no plunge or cut points".to_string());
    }
    if !failures.is_empty() {
        return Err(format!("{} contact failures, first: {}", failures.len(), failures.join("; ")));
    }
    Ok(())
}
