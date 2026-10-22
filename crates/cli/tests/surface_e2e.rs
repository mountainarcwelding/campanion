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

/// Flat-floor stepover for a 6 mm ball (radius 3 mm) and a 0.01 mm cusp.
fn dome_scallop_stepover() -> f64 {
    let height = 0.01_f64;
    let radius = 3.0_f64;
    2.0 * (2.0 * radius * height - height * height).sqrt()
}

/// Margin that makes the 40 mm dome plate an integer number of scallop stepovers wide.
///
/// Parallel finishing spaces passes evenly across the XY bounds, so a margin of 0 pitches the
/// lines at `40 / ceil(40 / stepover)`, a hair under the flat-floor stepover. Growing the cross
/// span to that multiple makes consecutive line starts equal the stepover.
fn dome_scallop_margin(stepover: f64) -> Result<f64, String> {
    let span = 40.0;
    let mut intervals = (span / stepover).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(format!("scallop stepover {stepover} does not divide the dome"));
    }
    let mut guard = 0u32;
    while span > stepover * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err("scallop spacing is not usable".into());
        }
    }
    Ok((intervals * stepover - span) / 2.0)
}

fn dome_scallop_job(root: &Path, margin: f64, shape: &str) -> Result<serde_json::Value, String> {
    let text = std::fs::read_to_string(root.join("examples/dome.job.json")).map_err(|err| format!("read dome job: {err}"))?;
    let mut job: serde_json::Value = serde_json::from_str(&text).map_err(|err| format!("dome job json: {err}"))?;
    let tool = job.get_mut("tools").and_then(|tools| tools.get_mut(0)).and_then(|tool| tool.as_object_mut()).ok_or("tool")?;
    tool.insert("shape".to_string(), serde_json::Value::String(shape.to_string()));
    let op = job.get_mut("operations").and_then(|ops| ops.get_mut(0)).and_then(|op| op.as_object_mut()).ok_or("operation")?;
    op.remove("stepover");
    op.insert("scallop_height".to_string(), serde_json::json!(0.01));
    if margin > 0.0 {
        op.insert("margin".to_string(), serde_json::json!(margin));
    }
    Ok(job)
}

/// Job file next to a copy of `examples/dome.stl`, so the relative source path still resolves.
fn write_job_beside_dome(root: &Path, name: &str, job: &serde_json::Value) -> Result<PathBuf, String> {
    let dir = temp_path(name);
    std::fs::create_dir_all(&dir).map_err(|err| format!("temp dir: {err}"))?;
    std::fs::copy(root.join("examples/dome.stl"), dir.join("dome.stl")).map_err(|err| format!("copy stl: {err}"))?;
    let path = dir.join("job.json");
    let text = serde_json::to_string_pretty(job).map_err(|err| format!("job json: {err}"))?;
    std::fs::write(&path, text).map_err(|err| format!("write job: {err}"))?;
    Ok(path)
}

#[test]
fn flat_tool_scallop_fails_at_run_with_a_clear_message() -> Result<(), String> {
    let root = workspace_root()?;
    let job = dome_scallop_job(&root, 0.0, "flat")?;
    let job_path = write_job_beside_dome(&root, "campanion-scallop-flat", &job)?;
    let ngc_path = temp_path("campanion-scallop-flat.ngc");
    let ir_path = temp_path("campanion-scallop-flat.json");
    let output = campanion(&root, &["run", &path_arg(&job_path)?, "-o", &path_arg(&ngc_path)?, "--ir", &path_arg(&ir_path)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() == Some(0) {
        return Err(format!("flat scallop succeeded\nstdout {stdout}\nstderr {stderr}"));
    }
    if stderr.contains("scallop height needs a ball or bull-nose tool") {
        Ok(())
    } else {
        Err(format!("stderr missing the flat-tool message\nstdout {stdout}\nstderr {stderr}"))
    }
}

#[test]
fn dome_scallop_height_sets_the_raster_spacing() -> Result<(), String> {
    let root = workspace_root()?;
    let stepover = dome_scallop_stepover();
    let margin = dome_scallop_margin(stepover)?;
    let job = dome_scallop_job(&root, margin, "ball")?;
    let job_path = write_job_beside_dome(&root, "campanion-scallop-dome", &job)?;
    let ngc_path = temp_path("campanion-scallop-dome.ngc");
    let ir_path = temp_path("campanion-scallop-dome.json");
    run_job(&root, &path_arg(&job_path)?, &ngc_path, &ir_path)?;
    assert_verify_clean(&root, "dome scallop", &ir_path)?;

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let blocks = ir["program"]["toolpaths"][0]["blocks"].as_array().ok_or("blocks is not an array")?;
    let starts = raster_line_starts(blocks)?;
    if starts.len() < 2 {
        return Err(format!("scallop finish produced {} raster lines", starts.len()));
    }
    let mut failures = Vec::new();
    let expected_first = -20.0 - margin;
    let expected_last = 20.0 + margin;
    if (starts[0] - expected_first).abs() > 1e-9 {
        failures.push(format!("first line Y {} != {expected_first}", starts[0]));
    }
    if (starts[starts.len() - 1] - expected_last).abs() > 1e-9 {
        failures.push(format!("last line Y {} != {expected_last}", starts[starts.len() - 1]));
    }
    for pair in starts.windows(2) {
        let gap = pair[1] - pair[0];
        if (gap - stepover).abs() > 1e-9 && failures.len() < 6 {
            failures.push(format!("line start gap {gap} != flat-floor stepover {stepover}"));
        }
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

/// Y of each raster line start. A line is a run of plunge or cut points at one Y.
///
/// Link samples between lines sit at their own Y, so a run of one point is not a line.
fn raster_line_starts(blocks: &[Value]) -> Result<Vec<f64>, String> {
    let mut ys = Vec::new();
    for block in blocks {
        if block["type"] != "linear" {
            continue;
        }
        let role = block["role"].as_str().unwrap_or("");
        if role != "cut" && role != "plunge" {
            continue;
        }
        let y = block["to"]["tip"]["y"].as_f64().ok_or("linear tip y")?;
        ys.push(y);
    }
    if ys.is_empty() {
        return Err("surface produced no plunge or cut".into());
    }
    let mut starts = Vec::new();
    let mut run_y = ys[0];
    let mut run_len = 1usize;
    for &y in ys.iter().skip(1) {
        if y == run_y {
            run_len += 1;
            continue;
        }
        if run_len >= 2 {
            starts.push(run_y);
        }
        run_y = y;
        run_len = 1;
    }
    if run_len >= 2 {
        starts.push(run_y);
    }
    Ok(starts)
}

#[test]
fn lowered_dome_cut_is_reported_as_a_gouge() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-gouge-e2e.ngc");
    let ir_path = temp_path("campanion-gouge-e2e.json");
    run_job(&root, "examples/dome.job.json", &ngc_path, &ir_path)?;

    let ir_arg = path_arg(&ir_path)?;
    let clean = campanion(&root, &["verify", &ir_arg, "--mesh", "examples/dome.stl", "--tool", "ball:3"])?;
    let clean_out = output_text(&clean.stdout)?;
    let clean_err = output_text(&clean.stderr)?;
    if clean.status.code() != Some(0) {
        return Err(format!("clean dome verify --mesh exit {:?} stdout {clean_out} stderr {clean_err}", clean.status.code()));
    }

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let mut ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let blocks = ir["program"]["toolpaths"]
        .get_mut(0)
        .and_then(|path| path.get_mut("blocks"))
        .and_then(|blocks| blocks.as_array_mut())
        .ok_or("blocks is not an array")?;
    let mut lowered = false;
    for block in blocks.iter_mut() {
        if block.get("type").and_then(Value::as_str) == Some("linear") && block.get("role").and_then(Value::as_str) == Some("cut") {
            let z = block.get("to").and_then(|to| to.get("tip")).and_then(|tip| tip.get("z")).and_then(Value::as_f64).ok_or("cut z")?;
            let tip = block.get_mut("to").and_then(|to| to.get_mut("tip")).ok_or("cut tip")?;
            tip["z"] = Value::from(z - 0.5);
            lowered = true;
            break;
        }
    }
    if !lowered {
        return Err("dome finish produced no cutting move".into());
    }
    let edited = temp_path("campanion-gouge-e2e-lowered.json");
    let text = serde_json::to_string_pretty(&ir).map_err(|err| format!("write json: {err}"))?;
    std::fs::write(&edited, text).map_err(|err| format!("write ir: {err}"))?;
    let edited_arg = path_arg(&edited)?;
    let output = campanion(&root, &["verify", &edited_arg, "--mesh", "examples/dome.stl", "--tool", "ball:3"])?;
    let stdout = output_text(&output.stdout)?;
    let stderr = output_text(&output.stderr)?;
    if output.status.code() != Some(1) {
        return Err(format!("lowered dome exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    let combined = format!("{stdout}\n{stderr}");
    if combined.contains("gouge") { Ok(()) } else { Err(format!("expected a gouge report:\n{combined}")) }
}
