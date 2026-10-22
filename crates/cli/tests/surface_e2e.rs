//! Parallel finish of `examples/dome.job.json`.
//!
//! A 6 mm ball, 1 mm stepover and 0.01 mm chord tolerance. `verify` must be clean. Every cutting
//! point is a drop-cutter sample of the STL, the highest cut is the dome top, and no cut goes
//! below the floor.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_mesh::{Cutter, DropCutter, parse_stl};
use serde_json::Value;

const STOCK_MM: f64 = 0.0;
const DOME_TOP_MM: f64 = 10.0;
const DOME_TOP_TOL_MM: f64 = 0.05;
const SURFACE_TOL_MM: f64 = 1.0e-6;

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

#[test]
fn dome_finish_stays_on_the_drop_cutter_surface() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-surface-e2e.ngc");
    let ir_path = temp_path("campanion-surface-e2e.json");
    run_job(&root, "examples/dome.job.json", &ngc_path, &ir_path)?;
    assert_verify_clean(&root, "examples/dome.job.json", &ir_path)?;

    let stl = std::fs::read(root.join("examples/dome.stl")).map_err(|err| format!("read stl: {err}"))?;
    let mesh = parse_stl(&stl).map_err(|err| format!("parse stl: {err}"))?;
    let bounds = mesh.bounds().map_err(|err| format!("stl bounds: {err}"))?;
    let floor = bounds.min().z;
    let cutter = Cutter::new_ball(3.0).map_err(|err| format!("ball: {err}"))?;
    let dc = DropCutter::new(&mesh, cutter).map_err(|err| format!("drop cutter: {err}"))?;

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir["program"]["toolpaths"].as_array().ok_or("toolpaths is not an array")?;
    if toolpaths.len() != 1 {
        return Err(format!("dome finish produced {} toolpaths", toolpaths.len()));
    }
    let blocks = toolpaths[0]["blocks"].as_array().ok_or("blocks is not an array")?;

    let mut cuts = 0usize;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    let mut failures = Vec::new();
    for block in blocks {
        if block["type"] != "linear" || block["role"] != "cut" {
            continue;
        }
        let x = block["to"]["tip"]["x"].as_f64().ok_or("cut x")?;
        let y = block["to"]["tip"]["y"].as_f64().ok_or("cut y")?;
        let z = block["to"]["tip"]["z"].as_f64().ok_or("cut z")?;
        let expected = dc.tip_z_or(x, y, floor) + STOCK_MM;
        let tol = SURFACE_TOL_MM + STOCK_MM;
        if (z - expected).abs() > tol && failures.len() < 8 {
            failures.push(format!("cut ({x}, {y}, {z}) is {expected} on the drop-cutter surface, tol {tol}"));
        }
        cuts += 1;
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }
    if cuts == 0 {
        failures.push("dome finish produced no cutting moves".into());
    }
    if min_z.is_finite() && min_z < floor {
        failures.push(format!("min cut Z is {min_z}, floor is {floor}"));
    }
    if max_z.is_finite() && (max_z - DOME_TOP_MM).abs() > DOME_TOP_TOL_MM {
        failures.push(format!("max cut Z is {max_z}, expected {DOME_TOP_MM} ± {DOME_TOP_TOL_MM}"));
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}
