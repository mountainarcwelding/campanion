//! Job-file and profile-document routes pick the same start on a closed lead-none profile.

#![allow(clippy::expect_used)]

use std::path::{Path, PathBuf};
use std::process::Command;

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir).join(name);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn line(layer: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!("0\nLINE\n8\n{layer}\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n")
}

fn closed_dxf(layer: &str, points: &[(f64, f64)]) -> String {
    let mut entities = String::new();
    for pair in points.windows(2) {
        entities.push_str(&line(layer, pair[0].0, pair[0].1, pair[1].0, pair[1].1));
    }
    if let (Some(first), Some(last)) = (points.first(), points.last()) {
        entities.push_str(&line(layer, last.0, last.1, first.0, first.1));
    }
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn first_rapid_xy(nc: &str) -> Result<(f64, f64), String> {
    for line in nc.lines() {
        let body = line.split('(').next().unwrap_or(line);
        if !body.split_whitespace().any(|word| matches!(word, "G0" | "G00")) {
            continue;
        }
        let mut x = None;
        let mut y = None;
        for word in body.split_whitespace() {
            if let Some(rest) = word.strip_prefix('X') {
                x = Some(rest.parse().map_err(|err| format!("X {err}"))?);
            } else if let Some(rest) = word.strip_prefix('Y') {
                y = Some(rest.parse().map_err(|err| format!("Y {err}"))?);
            }
        }
        if let (Some(x), Some(y)) = (x, y) {
            return Ok((x, y));
        }
    }
    Err(format!("no G0 XY in\n{nc}"))
}

/// XY of feed moves (G1/G2/G3), carrying forward modal X and Y.
fn feed_xy(nc: &str) -> Result<Vec<(f64, f64)>, String> {
    let mut points = Vec::new();
    let mut x = 0.0;
    let mut y = 0.0;
    for line in nc.lines() {
        let body = line.split('(').next().unwrap_or(line);
        let words: Vec<&str> = body.split_whitespace().collect();
        if words.is_empty() {
            continue;
        }
        let is_feed = words.iter().any(|word| matches!(*word, "G1" | "G01" | "G2" | "G02" | "G3" | "G03"));
        let mut saw_xy = false;
        for word in &words {
            if let Some(rest) = word.strip_prefix('X') {
                x = rest.parse().map_err(|err| format!("X {err}"))?;
                saw_xy = true;
            } else if let Some(rest) = word.strip_prefix('Y') {
                y = rest.parse().map_err(|err| format!("Y {err}"))?;
                saw_xy = true;
            }
        }
        if is_feed && saw_xy {
            points.push((x, y));
        }
    }
    if points.is_empty() {
        return Err(format!("no feed moves in\n{nc}"));
    }
    Ok(points)
}

fn run_ok(dir: &Path, args: &[&str]) -> Result<String, String> {
    let output = Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    Ok(stdout)
}

fn profile_job(name: &str, points: &[(f64, f64)], tabs: bool) -> String {
    let mut pts = String::from("[");
    for (index, (x, y)) in points.iter().enumerate() {
        if index > 0 {
            pts.push(',');
        }
        pts.push_str(&format!("[{x},{y}]"));
    }
    pts.push(']');
    let tabs_json = if tabs { r#","tabs":{"count":2,"width":0.3}"# } else { "" };
    format!(
        r#"{{"name":"{name}","process":"laser","cut_feed":600.0,"tool":{{"id":1,"kerf":0.1}},"params":{{"power":0.05}},"profile":{{"side":"outside","direction":"climb","compensation":"computed","z":0.0,"lead_in":{{"type":"none"}},"lead_out":{{"type":"none"}},"closed":true,"points":{pts}{tabs_json}}}}}"#
    )
}

fn jobfile(hash: &str, tabs: bool) -> String {
    let tabs_json = if tabs { r#","tabs":{"count":2,"width":0.3}"# } else { "" };
    format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"plate.dxf","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"kerf":0.1}}],"operations":[{{"name":"outline","type":"profile","layers":["PLATE"],"side":"outside","direction":"climb","compensation":"computed","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{{"cut":600.0}},"power":0.05,"lead_in":{{"type":"none"}},"lead_out":{{"type":"none"}}{tabs_json}}}]}}"#
    )
}

fn almost(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
    (a.0 - b.0).abs() <= tol && (a.1 - b.1).abs() <= tol
}

fn compare_routes(dir_name: &str, points: &[(f64, f64)], tabs: bool) -> Result<(), String> {
    let dir = temp_dir(dir_name)?;
    let drawing = closed_dxf("PLATE", points);
    std::fs::write(dir.join("plate.dxf"), &drawing).map_err(|err| err.to_string())?;
    let hash = fnv1a64(drawing.as_bytes());
    std::fs::write(dir.join("job.json"), jobfile(&hash, tabs)).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("profile.json"), profile_job(dir_name, points, tabs)).map_err(|err| err.to_string())?;

    let job_nc = dir.join("job.nc");
    let job_arg = path_arg(&job_nc)?;
    run_ok(&dir, &["run", "job.json", "-o", job_arg.as_str(), "--post", "grbl_laser"])?;
    let job_text = std::fs::read_to_string(&job_nc).map_err(|err| err.to_string())?;

    let ir = dir.join("profile.ir.json");
    let ir_arg = path_arg(&ir)?;
    let ir_json = run_ok(&dir, &["ir", "--job", "profile.json"])?;
    std::fs::write(&ir, &ir_json).map_err(|err| err.to_string())?;
    let profile_nc = dir.join("profile.nc");
    let profile_arg = path_arg(&profile_nc)?;
    run_ok(&dir, &["post", ir_arg.as_str(), "--post", "grbl_laser", "-o", profile_arg.as_str()])?;
    let profile_text = std::fs::read_to_string(&profile_nc).map_err(|err| err.to_string())?;

    let job_start = first_rapid_xy(&job_text)?;
    let profile_start = first_rapid_xy(&profile_text)?;
    if !almost(job_start, profile_start, 1e-4) {
        return Err(format!("start mismatch job {job_start:?} profile {profile_start:?}\njob:\n{job_text}\nprofile:\n{profile_text}"));
    }

    let job_feeds = feed_xy(&job_text)?;
    let profile_feeds = feed_xy(&profile_text)?;
    if job_feeds.len() != profile_feeds.len() {
        return Err(format!("feed count job {} profile {}\njob:\n{job_text}\nprofile:\n{profile_text}", job_feeds.len(), profile_feeds.len()));
    }
    for (index, (left, right)) in job_feeds.iter().zip(profile_feeds.iter()).enumerate() {
        if !almost(*left, *right, 1e-4) {
            return Err(format!("feed {index} mismatch job {left:?} profile {right:?}\njob:\n{job_text}\nprofile:\n{profile_text}"));
        }
    }
    Ok(())
}

#[test]
fn lead_none_starts_match_on_a_rectangle() -> Result<(), String> {
    compare_routes("campanion-start-rect", &[(1.0, 1.0), (13.8, 1.0), (13.8, 28.1), (1.0, 28.1)], false)
}

#[test]
fn lead_none_starts_and_tabs_match_on_a_tapered_plate() -> Result<(), String> {
    // Closed outline whose two long verticals are equal, so the documented tie-break must agree.
    let points = [(1.0, 1.0), (13.8, 1.0), (13.8, 28.1), (12.4, 39.1), (2.4, 39.1), (1.0, 28.1)];
    compare_routes("campanion-start-taper", &points, false)?;
    compare_routes("campanion-start-taper-tabs", &points, true)
}
