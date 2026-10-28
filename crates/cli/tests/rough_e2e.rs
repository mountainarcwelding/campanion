//! Z-level roughing of `examples/dome_rough.job.json`.
//!
//! A 6 mm flat, 2.5 mm step-down, 2.4 mm stepover and 0.3 mm stock to leave. `verify` must be
//! clean. The depth plan runs from the mesh top down to the floor. A level with no legal tool
//! position is not cut: 0.3 mm of stock keeps the flat tool off the plate, so the floor level is
//! empty and the deepest cut is the 2.5 mm level. Every cutting move, sampled every 0.25 mm,
//! stays at or above the drop-cutter tip plus the stock to leave.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_mesh::{Cutter, DropCutter, parse_stl};
use serde_json::Value;

const STEP_DOWN_MM: f64 = 2.5;
const STOCK_MM: f64 = 0.3;
const SAMPLE_MM: f64 = 0.25;
const GOUGE_TOL_MM: f64 = 1.0e-6;
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

fn run_job(cwd: &Path, job: &str, ngc_path: &Path, ir_path: &Path) -> Result<String, String> {
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
    Ok(stderr)
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

/// Z of each roughing level, from `top - step_down` down to exactly `bottom`.
fn planned_levels(top: f64, bottom: f64, step_down: f64) -> Vec<f64> {
    let total = top - bottom;
    let mut done = 0.0;
    let mut heights = Vec::new();
    loop {
        let next = done + step_down;
        if next >= total - 1e-9 {
            heights.push(bottom);
            break;
        }
        done = next;
        heights.push(top - done);
    }
    heights
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

fn push_sample(out: &mut Vec<(f64, f64, f64)>, x: f64, y: f64, z: f64) {
    if out.last().is_some_and(|prev| prev.0 == x && prev.1 == y && prev.2 == z) {
        return;
    }
    out.push((x, y, z));
}

fn sample_line(from: (f64, f64, f64), to: (f64, f64, f64), out: &mut Vec<(f64, f64, f64)>) {
    let len = (to.0 - from.0).hypot(to.1 - from.1).hypot(to.2 - from.2);
    let steps = if len <= SAMPLE_MM { 1.0 } else { (len / SAMPLE_MM).ceil() };
    let steps = steps.max(1.0) as usize;
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        push_sample(out, from.0 + (to.0 - from.0) * t, from.1 + (to.1 - from.1) * t, from.2 + (to.2 - from.2) * t);
    }
}

fn sample_arc(from: (f64, f64, f64), to: (f64, f64, f64), center: (f64, f64, f64), ccw: bool, out: &mut Vec<(f64, f64, f64)>) {
    let radius = (from.0 - center.0).hypot(from.1 - center.1);
    let a0 = (from.1 - center.1).atan2(from.0 - center.0);
    let a1 = (to.1 - center.1).atan2(to.0 - center.0);
    let chord = (to.0 - from.0).hypot(to.1 - from.1);
    let ccw_delta = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    let sweep = if chord <= 1e-12 {
        std::f64::consts::TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { std::f64::consts::TAU } else { ccw_delta }
    } else {
        let cw = std::f64::consts::TAU - ccw_delta;
        if cw <= 1e-15 { std::f64::consts::TAU } else { cw }
    };
    let length = (radius * sweep).hypot(to.2 - from.2);
    let steps = if length <= SAMPLE_MM { 1.0 } else { (length / SAMPLE_MM).ceil() };
    let steps = steps.max(1.0) as usize;
    let sense = if ccw { 1.0 } else { -1.0 };
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        let angle = a0 + sense * sweep * t;
        push_sample(out, center.0 + radius * angle.cos(), center.1 + radius * angle.sin(), from.2 + (to.2 - from.2) * t);
    }
}

#[test]
fn dome_rough_stays_above_stock_to_leave() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-rough-e2e.ngc");
    let ir_path = temp_path("campanion-rough-e2e.json");
    run_job(&root, "examples/dome_rough.job.json", &ngc_path, &ir_path)?;
    assert_verify_clean(&root, "examples/dome_rough.job.json", &ir_path)?;

    let stl = std::fs::read(root.join("examples/dome.stl")).map_err(|err| format!("read stl: {err}"))?;
    let mesh = parse_stl(&stl).map_err(|err| format!("parse stl: {err}"))?;
    let bounds = mesh.bounds().map_err(|err| format!("stl bounds: {err}"))?;
    let floor = bounds.min().z;
    let plan = planned_levels(bounds.max().z, floor, STEP_DOWN_MM);
    // Stock of 0.3 mm puts the plate out of reach at the floor, so that level has no ring.
    let expected = vec![bounds.max().z - STEP_DOWN_MM, bounds.max().z - 2.0 * STEP_DOWN_MM, bounds.max().z - 3.0 * STEP_DOWN_MM];
    let cutter = Cutter::new_flat(3.0).map_err(|err| format!("flat: {err}"))?;
    let dc = DropCutter::new(&mesh, cutter).map_err(|err| format!("drop cutter: {err}"))?;

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir["program"]["toolpaths"].as_array().ok_or("toolpaths is not an array")?;
    if toolpaths.len() != 1 {
        return Err(format!("dome rough produced {} toolpaths", toolpaths.len()));
    }
    let blocks = toolpaths[0]["blocks"].as_array().ok_or("blocks is not an array")?;

    let mut cursor = None;
    let mut levels = Vec::new();
    let mut min_z = f64::INFINITY;
    let mut cuts = 0usize;
    let mut process_on = 0usize;
    let mut process_off = 0usize;
    let mut failures = Vec::new();
    for block in blocks {
        let kind = block["type"].as_str().unwrap_or("");
        if kind == "rapid" {
            cursor = Some(tip_of(block)?);
            continue;
        }
        if kind == "process_on" {
            process_on += 1;
            continue;
        }
        if kind == "process_off" {
            process_off += 1;
            continue;
        }
        if kind != "linear" && kind != "arc" && kind != "helix" {
            continue;
        }
        let to = tip_of(block)?;
        let Some(from) = cursor else {
            cursor = Some(to);
            continue;
        };
        if block["role"] == "cut" {
            note_level(&mut levels, from.2);
            let mut samples = Vec::new();
            if kind == "linear" {
                sample_line(from, to, &mut samples);
            } else {
                let center = &block["center"];
                let cx = center["x"].as_f64().ok_or("arc center x")?;
                let cy = center["y"].as_f64().ok_or("arc center y")?;
                let cz = center["z"].as_f64().ok_or("arc center z")?;
                let ccw = block["sense"] == "ccw";
                sample_arc(from, to, (cx, cy, cz), ccw, &mut samples);
            }
            for (x, y, z) in samples {
                let limit = dc.tip_z_or(x, y, floor) + STOCK_MM - GOUGE_TOL_MM;
                if z < limit && failures.len() < 8 {
                    failures.push(format!("cut ({x}, {y}, {z}) is below drop-cutter tip plus stock {limit}"));
                }
                min_z = min_z.min(z);
            }
            cuts += 1;
        }
        cursor = Some(to);
    }
    if cuts == 0 {
        failures.push("dome rough produced no cutting moves".into());
    }
    if process_on != 1 || process_off != 1 {
        failures.push(format!("process on {process_on} times and off {process_off} times"));
    }
    let Some(&planned_floor) = plan.last() else {
        failures.push("depth plan is empty".into());
        return Err(failures.join("\n"));
    };
    if (planned_floor - floor).abs() > LEVEL_TOL_MM {
        failures.push(format!("depth plan ends at {planned_floor}, floor is {floor}"));
    }
    if levels.len() != expected.len() || levels.iter().zip(expected.iter()).any(|(got, want)| (got - want).abs() > LEVEL_TOL_MM) {
        failures.push(format!("levels {levels:?} expected {expected:?}, plan {plan:?}"));
    }
    // The floor level is planned and empty. The deepest cut is the lowest level that has a ring.
    let Some(&deepest_level) = levels.last() else {
        failures.push("no cutting level".into());
        return Err(failures.join("\n"));
    };
    if min_z.is_finite() && (min_z - deepest_level).abs() > LEVEL_TOL_MM {
        failures.push(format!("deepest cut Z is {min_z}, deepest level is {deepest_level}"));
    }
    if min_z < floor - GOUGE_TOL_MM {
        failures.push(format!("deepest cut Z {min_z} is below the floor {floor}"));
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

/// `examples/dome_rough.job.json` with one `entry` object, next to a copy of the mesh.
fn write_entry_job(root: &Path, dir: &Path, entry_json: &str) -> Result<PathBuf, String> {
    std::fs::create_dir_all(dir).map_err(|err| format!("mkdir: {err}"))?;
    std::fs::copy(root.join("examples/dome.stl"), dir.join("dome.stl")).map_err(|err| format!("copy stl: {err}"))?;
    let text = std::fs::read_to_string(root.join("examples/dome_rough.job.json")).map_err(|err| format!("read job: {err}"))?;
    let mut value: Value = serde_json::from_str(&text).map_err(|err| format!("job json: {err}"))?;
    let entry: Value = serde_json::from_str(entry_json).map_err(|err| format!("entry json: {err}"))?;
    let op = value
        .get_mut("operations")
        .and_then(Value::as_array_mut)
        .and_then(|ops| ops.get_mut(0))
        .and_then(Value::as_object_mut)
        .ok_or("dome rough job has no operation")?;
    op.insert("entry".to_string(), entry);
    let path = dir.join("dome_rough.job.json");
    let body = serde_json::to_string_pretty(&value).map_err(|err| format!("write job json: {err}"))?;
    std::fs::write(&path, body).map_err(|err| format!("write job: {err}"))?;
    Ok(path)
}

fn mesh_top_z(root: &Path) -> Result<f64, String> {
    let stl = std::fs::read(root.join("examples/dome.stl")).map_err(|err| format!("read stl: {err}"))?;
    let mesh = parse_stl(&stl).map_err(|err| format!("parse stl: {err}"))?;
    let bounds = mesh.bounds().map_err(|err| format!("stl bounds: {err}"))?;
    Ok(bounds.max().z)
}

/// A feed that drops more than `1e-6` mm with no XY motion, and ends below `top_z`.
fn vertical_descents_below(ir_path: &Path, top_z: f64) -> Result<Vec<String>, String> {
    let ir_text = std::fs::read_to_string(ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir: Value = serde_json::from_str(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let blocks = ir["program"]["toolpaths"][0]["blocks"].as_array().ok_or("blocks is not an array")?;
    let mut cursor = None;
    let mut failures = Vec::new();
    for block in blocks {
        let kind = block["type"].as_str().unwrap_or("");
        if kind != "linear" && kind != "arc" && kind != "helix" && kind != "rapid" {
            continue;
        }
        let to = tip_of(block)?;
        let Some(from) = cursor else {
            cursor = Some(to);
            continue;
        };
        cursor = Some(to);
        if kind == "rapid" {
            continue;
        }
        let drop = from.2 - to.2;
        if drop <= 1.0e-6 || to.2 >= top_z - 1.0e-6 {
            continue;
        }
        if feed_has_xy(block, from, to)? {
            continue;
        }
        if failures.len() < 8 {
            failures.push(format!("{kind} descent from {from:?} to {to:?} is vertical below top Z {top_z}"));
        }
    }
    Ok(failures)
}

fn feed_has_xy(block: &Value, from: (f64, f64, f64), to: (f64, f64, f64)) -> Result<bool, String> {
    let chord = (to.0 - from.0).hypot(to.1 - from.1);
    if block["type"] == "linear" {
        return Ok(chord > 1.0e-6);
    }
    let center = &block["center"];
    let cx = center["x"].as_f64().ok_or("arc center x")?;
    let cy = center["y"].as_f64().ok_or("arc center y")?;
    let radius = (from.0 - cx).hypot(from.1 - cy);
    Ok(radius > 1.0e-6 || chord > 1.0e-6)
}

#[test]
fn plunge_entry_ir_matches_golden() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-rough-plunge.ngc");
    let ir_path = temp_path("campanion-rough-plunge.json");
    run_job(&root, "examples/dome_rough.job.json", &ngc_path, &ir_path)?;
    let actual = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let golden = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden/dome_rough.ir.json");
    let expected = std::fs::read_to_string(&golden).map_err(|err| format!("read golden: {err}"))?;
    campanion_testsupport::ir_json_close(&actual, &expected)
}

#[test]
fn ramp_and_helix_entries_stay_clear_of_the_mesh() -> Result<(), String> {
    let root = workspace_root()?;
    let top_z = mesh_top_z(&root)?;
    let ramp_dir = temp_path("campanion-rough-ramp");
    let _ramp_job = write_entry_job(&root, &ramp_dir, r#"{"type":"ramp","angle_deg":3}"#)?;
    let ramp_ngc = temp_path("campanion-rough-ramp.ngc");
    let ramp_ir = temp_path("campanion-rough-ramp.json");
    let ramp_stderr = run_job(&ramp_dir, "dome_rough.job.json", &ramp_ngc, &ramp_ir)?;
    assert_verify_clean(&root, "ramp", &ramp_ir)?;
    if ramp_stderr.lines().any(|line| line == "helix does not fit; ramping") {
        return Err(format!("ramp printed a helix warning: {ramp_stderr}"));
    }
    let descents = vertical_descents_below(&ramp_ir, top_z)?;
    if !descents.is_empty() {
        return Err(descents.join("\n"));
    }

    let helix_dir = temp_path("campanion-rough-helix");
    let _helix_job = write_entry_job(&root, &helix_dir, r#"{"type":"helix","radius":2,"angle_deg":3}"#)?;
    let helix_ngc = temp_path("campanion-rough-helix.ngc");
    let helix_ir = temp_path("campanion-rough-helix.json");
    run_job(&helix_dir, "dome_rough.job.json", &helix_ngc, &helix_ir)?;
    assert_verify_clean(&root, "helix", &helix_ir)?;
    let helix_descents = vertical_descents_below(&helix_ir, top_z)?;
    if !helix_descents.is_empty() {
        return Err(helix_descents.join("\n"));
    }

    let fallback_dir = temp_path("campanion-rough-helix-fallback");
    let _fallback_job = write_entry_job(&root, &fallback_dir, r#"{"type":"helix","radius":50,"angle_deg":3}"#)?;
    let fallback_ngc = temp_path("campanion-rough-helix-fallback.ngc");
    let fallback_ir = temp_path("campanion-rough-helix-fallback.json");
    let fallback_stderr = run_job(&fallback_dir, "dome_rough.job.json", &fallback_ngc, &fallback_ir)?;
    assert_verify_clean(&root, "helix fallback", &fallback_ir)?;
    let warnings = fallback_stderr.lines().filter(|line| *line == "helix does not fit; ramping").count();
    if warnings != 1 {
        return Err(format!("helix fallback warnings {warnings}: {fallback_stderr}"));
    }
    Ok(())
}
