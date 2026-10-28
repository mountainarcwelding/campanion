//! Pocket area passes honour ramp entry and stay at depth between rings (issue #89).
//!
//! The link rapids to the retract height. Each depth is its own toolpath, so the ramp is
//! sized for the whole drop from that height. A ramp or helix that does not fit is an error.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use serde_json::Value;

const PART_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="170mm" height="100mm" viewBox="0 0 170 100">"#,
    r#"<g id="RING"><circle cx="50" cy="50" r="30"/><circle cx="50" cy="50" r="15"/></g>"#,
    r#"<g id="SQUARE"><rect x="100" y="25" width="50" height="50"/></g>"#,
    "</svg>",
);

const ANGLE_EPS: f64 = 1.0e-6;
/// Test-sink coordinates are printed to 0.0001 mm.
const NGC_ANGLE_EPS: f64 = 2.0e-4;

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let path = dir.join(name);
    std::fs::create_dir_all(&path).map_err(|err| err.to_string())?;
    Ok(path)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(cwd: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn write_job(dir: &Path, operations: &str) -> Result<(), String> {
    let hash = fnv1a64(PART_SVG.as_bytes());
    std::fs::write(dir.join("part.svg"), PART_SVG).map_err(|err| err.to_string())?;
    let body = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"diameter":6.0}}],"operations":[{operations}]}}"#
    );
    std::fs::write(dir.join("job.json"), body).map_err(|err| err.to_string())?;
    Ok(())
}

fn pocket_op(name: &str, layer: &str, entry: &str) -> String {
    format!(
        r#"{{"name":"{name}","type":"pocket","layers":["{layer}"],"tool_id":1,"depth":6,"step_down":3,"stepover":2,"entry":{entry},"feeds":{{"cut":1500,"plunge":500}}}}"#
    )
}

fn run(dir: &Path) -> Result<(String, Value), String> {
    let ngc = dir.join("out.ngc");
    let ir = dir.join("out.json");
    let output = campanion(dir, &["run", "job.json", "-o", &path_arg(&ngc)?, "--ir", &path_arg(&ir)?])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    let ngc_text = std::fs::read_to_string(&ngc).map_err(|err| err.to_string())?;
    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let ir_json: Value = serde_json::from_str(&ir_text).map_err(|err| err.to_string())?;
    Ok((ngc_text, ir_json))
}

#[derive(Clone, Copy)]
struct Xyz {
    x: f64,
    y: f64,
    z: f64,
}

fn num(value: &Value, key: &str) -> Result<f64, String> {
    value.get(key).and_then(Value::as_f64).ok_or_else(|| format!("missing {key}"))
}

fn xyz_of(value: &Value) -> Result<Xyz, String> {
    Ok(Xyz { x: num(value, "x")?, y: num(value, "y")?, z: num(value, "z")? })
}

fn tip_of(pose: &Value) -> Result<Xyz, String> {
    xyz_of(pose.get("tip").ok_or("missing tip")?)
}

fn xy_sweep(from: Xyz, to: Xyz, centre: Xyz, ccw: bool) -> f64 {
    let a0 = (from.y - centre.y).atan2(from.x - centre.x);
    let a1 = (to.y - centre.y).atan2(to.x - centre.x);
    let radius = (from.x - centre.x).hypot(from.y - centre.y);
    let chord = (to.x - from.x).hypot(to.y - from.y);
    let closure = 1e-6_f64.max(radius * 1e-6);
    let ccw_delta = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    if chord <= closure {
        std::f64::consts::TAU
    } else if ccw {
        if ccw_delta <= 1e-8 { std::f64::consts::TAU } else { ccw_delta }
    } else {
        let cw = std::f64::consts::TAU - ccw_delta;
        if cw <= 1e-8 { std::f64::consts::TAU } else { cw }
    }
}

struct Descent {
    angle: f64,
    vertical: bool,
}

fn horiz_of(kind: &str, block: &Value, from: Xyz, to: Xyz) -> Result<f64, String> {
    match kind {
        "linear" => Ok((to.x - from.x).hypot(to.y - from.y)),
        "helix" => {
            let centre = xyz_of(block.get("center").ok_or("helix has no centre")?)?;
            let sweep = num(block, "sweep")?;
            let radius = (from.x - centre.x).hypot(from.y - centre.y);
            Ok(radius * sweep.abs())
        }
        "arc" => {
            let centre = xyz_of(block.get("center").ok_or("arc has no centre")?)?;
            let ccw = block.get("sense").and_then(Value::as_str) == Some("ccw");
            let radius = (from.x - centre.x).hypot(from.y - centre.y);
            Ok(radius * xy_sweep(from, to, centre, ccw))
        }
        _ => Ok(0.0),
    }
}

fn ir_toolpaths(program: &Value) -> Result<Vec<(String, Vec<Descent>, Vec<f64>)>, String> {
    let paths = program.get("program").and_then(|p| p.get("toolpaths")).and_then(Value::as_array).ok_or("ir has no toolpaths")?;
    let mut out = Vec::new();
    for path in paths {
        let name = path.get("name").and_then(Value::as_str).unwrap_or("").to_string();
        let blocks = path.get("blocks").and_then(Value::as_array).ok_or("toolpath has no blocks")?;
        let mut cursor: Option<Xyz> = None;
        let mut descents = Vec::new();
        let mut ends = Vec::new();
        for block in blocks {
            let kind = block.get("type").and_then(Value::as_str).unwrap_or("");
            let next = match kind {
                "rapid" | "linear" | "arc" | "helix" => Some(tip_of(block.get("to").ok_or("motion has no end")?)?),
                _ => None,
            };
            if let Some(to) = next {
                ends.push(to.z);
                if let Some(from) = cursor
                    && kind != "rapid"
                    && to.z < from.z - 1e-9
                {
                    let horiz = horiz_of(kind, block, from, to)?;
                    descents.push(Descent { angle: (from.z - to.z).atan2(horiz), vertical: horiz < 1e-4 && to.z < 0.0 });
                }
                cursor = Some(to);
            }
        }
        out.push((name, descents, ends));
    }
    Ok(out)
}

fn assert_level_stays_down(name: &str, ends: &[f64]) -> Result<(), String> {
    let below: Vec<usize> = ends.iter().enumerate().filter(|(_, z)| **z < -1e-9).map(|(index, _)| index).collect();
    if below.len() < 2 {
        return Err(format!("{name}: expected several positions below Z 0, got {ends:?}"));
    }
    let first = below[0];
    let last = *below.last().ok_or("missing last cut")?;
    for z in ends.iter().take(last).skip(first + 1) {
        if *z >= -1e-9 {
            return Err(format!("{name}: retract to Z {z} between rings, ends {ends:?}"));
        }
    }
    Ok(())
}

fn strip_comments(line: &str) -> String {
    let mut out = String::new();
    let mut depth = 0i32;
    for ch in line.chars() {
        match ch {
            '(' => depth += 1,
            ')' if depth > 0 => depth -= 1,
            _ if depth == 0 => out.push(ch),
            _ => {}
        }
    }
    out
}

fn word_f64(words: &[(char, f64)], letter: char) -> Option<f64> {
    words.iter().find(|(name, _)| *name == letter).map(|(_, value)| *value)
}

struct Motion {
    code: u8,
    to: Xyz,
    i: f64,
    j: f64,
}

fn ngc_sections(ngc: &str) -> Vec<(String, Vec<Motion>)> {
    let mut sections = Vec::new();
    let mut name = String::new();
    let mut motions = Vec::new();
    let mut cursor: Option<Xyz> = None;
    for raw in ngc.lines() {
        let trimmed = raw.trim();
        if let Some(rest) = trimmed.strip_prefix("(toolpath") {
            if !name.is_empty() || !motions.is_empty() {
                sections.push((name.clone(), std::mem::take(&mut motions)));
            }
            name = rest.trim().trim_end_matches(')').trim().to_string();
            cursor = None;
            continue;
        }
        let code = strip_comments(raw);
        if code.is_empty() {
            continue;
        }
        let mut motion = None;
        let mut words = Vec::new();
        for word in code.split_whitespace() {
            let mut chars = word.chars();
            let Some(letter) = chars.next() else {
                continue;
            };
            let Ok(value) = chars.as_str().parse::<f64>() else {
                continue;
            };
            if letter == 'G' && (0.0..=3.0).contains(&value) {
                motion = Some(value as u8);
            }
            words.push((letter, value));
        }
        let Some(motion) = motion else {
            continue;
        };
        let Some(from) = cursor else {
            cursor = Some(Xyz {
                x: word_f64(&words, 'X').unwrap_or(0.0),
                y: word_f64(&words, 'Y').unwrap_or(0.0),
                z: word_f64(&words, 'Z').unwrap_or(0.0),
            });
            continue;
        };
        let to = Xyz {
            x: word_f64(&words, 'X').unwrap_or(from.x),
            y: word_f64(&words, 'Y').unwrap_or(from.y),
            z: word_f64(&words, 'Z').unwrap_or(from.z),
        };
        motions.push(Motion { code: motion, to, i: word_f64(&words, 'I').unwrap_or(0.0), j: word_f64(&words, 'J').unwrap_or(0.0) });
        cursor = Some(to);
    }
    if !name.is_empty() || !motions.is_empty() {
        sections.push((name, motions));
    }
    sections
}

fn ngc_horiz(from: Xyz, motion: &Motion) -> f64 {
    if motion.code == 1 {
        (motion.to.x - from.x).hypot(motion.to.y - from.y)
    } else {
        let centre = Xyz { x: from.x + motion.i, y: from.y + motion.j, z: from.z };
        let radius = motion.i.hypot(motion.j);
        radius * xy_sweep(from, motion.to, centre, motion.code == 3)
    }
}

fn assert_ngc_section(name: &str, motions: &[Motion], max_angle: f64) -> Result<(), String> {
    if motions.is_empty() {
        return Err(format!("{name}: no motion"));
    }
    // The opening rapid sets the cursor and is not stored. Each stored motion starts where the
    // previous one ended, so the first stored move has no descent to measure.
    let mut prev: Option<Xyz> = None;
    let mut plunges = 0usize;
    let mut ends = Vec::new();
    for motion in motions {
        let Some(from) = prev else {
            prev = Some(motion.to);
            ends.push(motion.to.z);
            continue;
        };
        if motion.code != 0 && motion.to.z < from.z - 1e-9 {
            let horiz = ngc_horiz(from, motion);
            let angle = (from.z - motion.to.z).atan2(horiz);
            if angle > max_angle + NGC_ANGLE_EPS {
                return Err(format!("{name}: posted {:.3}° from Z {:.4} to Z {:.4}", angle.to_degrees(), from.z, motion.to.z));
            }
            if horiz < 1e-4 && motion.to.z < 0.0 {
                plunges += 1;
            }
        }
        ends.push(motion.to.z);
        prev = Some(motion.to);
    }
    if plunges != 0 {
        return Err(format!("{name}: {plunges} straight plunges below Z 0"));
    }
    assert_level_stays_down(name, &ends)
}

#[test]
fn ring_and_square_ramp_at_five_degrees_and_link_at_depth() -> Result<(), String> {
    let dir = temp_dir("campanion-pocket-entry")?;
    let ops = format!(
        "{},{}",
        pocket_op("ring", "RING", r#"{"type":"ramp","angle_deg":5}"#),
        pocket_op("square", "SQUARE", r#"{"type":"ramp","angle_deg":5}"#)
    );
    write_job(&dir, &ops)?;
    let (ngc, ir) = run(&dir)?;
    let max_angle = 5.0_f64.to_radians();
    let paths = ir_toolpaths(&ir)?;
    let mut ring = 0;
    let mut square = 0;
    for (name, descents, ends) in &paths {
        if name != "ring" && name != "square" {
            return Err(format!("unexpected toolpath {name}"));
        }
        if name == "ring" {
            ring += 1;
        } else {
            square += 1;
        }
        if descents.is_empty() {
            return Err(format!("{name}: no descent"));
        }
        let plunges = descents.iter().filter(|d| d.vertical).count();
        if plunges != 0 {
            return Err(format!("{name}: {plunges} straight plunges below Z 0"));
        }
        for descent in descents {
            if descent.angle > max_angle + ANGLE_EPS {
                return Err(format!("{name}: descent {:.4}° exceeds 5°", descent.angle.to_degrees()));
            }
        }
        assert_level_stays_down(name, ends)?;
    }
    if ring != 2 || square != 2 {
        return Err(format!("expected two depths each, ring {ring} square {square}"));
    }
    for (name, motions) in ngc_sections(&ngc) {
        assert_ngc_section(&name, &motions, max_angle)?;
    }
    Ok(())
}

#[test]
fn omitted_entry_still_plunges() -> Result<(), String> {
    let dir = temp_dir("campanion-pocket-plunge")?;
    let op =
        r#"{"name":"square","type":"pocket","layers":["SQUARE"],"tool_id":1,"depth":6,"step_down":3,"stepover":2,"feeds":{"cut":1500,"plunge":500}}"#;
    write_job(&dir, op)?;
    let (ngc, _) = run(&dir)?;
    let mut plunges = 0;
    for (_name, motions) in ngc_sections(&ngc) {
        let mut prev: Option<Xyz> = None;
        for motion in motions {
            if let Some(from) = prev
                && motion.code == 1
            {
                let horiz = (motion.to.x - from.x).hypot(motion.to.y - from.y);
                if horiz < 1e-4 && motion.to.z < from.z - 1e-9 && motion.to.z < 0.0 {
                    plunges += 1;
                }
            }
            prev = Some(motion.to);
        }
    }
    if plunges == 0 {
        return Err(format!("plunge pocket produced no vertical feed below Z 0\n{ngc}"));
    }
    Ok(())
}

#[test]
fn entry_that_does_not_fit_names_the_operation() -> Result<(), String> {
    let dir = temp_dir("campanion-pocket-entry-fit")?;
    write_job(&dir, &pocket_op("ring", "RING", r#"{"type":"helix","radius":40,"angle_deg":5}"#))?;
    let output = campanion(&dir, &["run", "job.json", "-o", "out.ngc"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() == Some(0) {
        return Err(format!("helix that leaves the pocket succeeded: {stderr}"));
    }
    if !stderr.contains("operation ring") || !stderr.contains("helix does not fit in the pocket") {
        return Err(format!("stderr did not name the operation and the helix: {stderr}"));
    }

    write_job(&dir, &pocket_op("ring", "RING", r#"{"type":"ramp","angle_deg":0.2}"#))?;
    let output = campanion(&dir, &["run", "job.json", "-o", "out.ngc"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() == Some(0) {
        return Err(format!("ramp longer than the ring succeeded: {stderr}"));
    }
    if !stderr.contains("operation ring") || !stderr.contains("ramp is longer than the pocket path") {
        return Err(format!("stderr did not name the operation and the ramp: {stderr}"));
    }
    Ok(())
}
