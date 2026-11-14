//! Tilting kerf heads: requested lead and taper on the path, travel limits, and post gates.
//!
//! The gantry-head limits, pivot, and any chart entries used here are example values, not
//! measured machine data. Feeds, delays and the kerf are also example values.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let root = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let dir = root.join(name);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn line(layer: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!("0\nLINE\n8\n{layer}\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n")
}

fn rectangle() -> String {
    let points = [(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)];
    let mut entities = String::new();
    for pair in points.windows(2) {
        entities.push_str(&line("PLATE", pair[0].0, pair[0].1, pair[1].0, pair[1].1));
    }
    if let (Some(first), Some(last)) = (points.first(), points.last()) {
        entities.push_str(&line("PLATE", last.0, last.1, first.0, first.1));
    }
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn gantry_kin() -> Result<String, String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/gantry_head_ab.kin.json");
    std::fs::read_to_string(path).map_err(|err| err.to_string())
}

fn write_job(dir: &Path, drawing: &str, extra_op: &str, extra_tool: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("gantry_head_ab.kin.json"), gantry_kin()?).map_err(|err| err.to_string())?;
    let hash = fnv1a64(drawing.as_bytes());
    let job = format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "kinematics": "gantry_head_ab.kin.json",
  "tools": [ {{ "id": 1, "kerf": 2.0{extra_tool} }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "outside",
    "tool_id": 1,
    "depth": 6.0,
    "step_down": 6.0,
    "feeds": {{ "cut": 1000, "plunge": 300 }},
    "timing": {{ "pierce_delay": 0.25 }},
    "lead_in": {{ "type": "none" }},
    "lead_out": {{ "type": "none" }}{extra_op}
  }} ]
}}"#
    );
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn run_job(dir: &Path, post: &str) -> Result<(String, String, String), String> {
    let nc_path = dir.join(format!("{post}.nc"));
    let ir_path = dir.join("job.ir.json");
    let output =
        campanion(dir, &["run", "job.json", "-o", nc_path.to_str().ok_or("path")?, "--ir", ir_path.to_str().ok_or("path")?, "--post", post])?;
    let stderr = text(&output.stderr)?;
    if !output.status.success() {
        return Err(format!("run --post {post} exit {:?} stderr {stderr}", output.status.code()));
    }
    let nc = std::fs::read_to_string(&nc_path).map_err(|err| err.to_string())?;
    let ir = std::fs::read_to_string(&ir_path).map_err(|err| err.to_string())?;
    Ok((nc, ir, stderr))
}

fn run_fails(dir: &Path, post: &str) -> Result<String, String> {
    let nc_path = dir.join("fail.nc");
    let output = campanion(dir, &["run", "job.json", "-o", nc_path.to_str().ok_or("path")?, "--post", post])?;
    let stderr = text(&output.stderr)?;
    if output.status.success() {
        return Err(format!("run --post {post} unexpectedly succeeded\n{stderr}"));
    }
    Ok(stderr)
}

fn has_word(nc: &str, needle: &str) -> bool {
    nc.lines().any(|line| {
        let body = if let Some(idx) = line.find('(') { &line[..idx] } else { line };
        body.split_whitespace().any(|word| word == needle || word.starts_with(needle))
    })
}

fn golden_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name)
}

fn check_golden(name: &str, nc: &str) -> Result<(), String> {
    let path = golden_path(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        std::fs::write(&path, nc).map_err(|err| err.to_string())?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if nc != expected {
        return Err(format!("golden mismatch (set UPDATE_GOLDEN=1 after reviewing)\n--- got ---\n{nc}\n--- want ---\n{expected}"));
    }
    Ok(())
}

#[test]
fn waterjet_taper_stores_the_requested_angle_and_writes_ab_words() -> Result<(), String> {
    // 10° is inside the example ±60° travel on gantry_head_ab.json.
    let dir = temp_dir("campanion-kerf-tilt-waterjet")?;
    write_job(&dir, &rectangle(), r#", "abrasive": true, "tilt": { "lead_deg": 0, "taper_deg": 10 }"#, "")?;
    let (nc, ir, _) = run_job(&dir, "waterjet")?;
    if !ir.contains("requested lead 0 deg, requested taper 10 deg") && !nc.contains("requested lead 0 deg, requested taper 10 deg") {
        return Err(format!("missing requested-angle comment\n{nc}"));
    }
    if !has_word(&nc, "G93") || !has_word(&nc, "G94") {
        return Err(format!("expected inverse-time G93/G94\n{nc}"));
    }
    let has_rotary = nc.lines().any(|line| {
        let body = line.split('(').next().unwrap_or(line);
        body.contains("A")
            && body.chars().any(|ch| ch == 'A')
            && (body.contains("A") || body.contains("B"))
            && (body.contains("A-")
                || body.contains("A") && body.split_whitespace().any(|w| w.starts_with('A') && w.len() > 1)
                || body.split_whitespace().any(|w| w.starts_with('B') && w.len() > 1))
    });
    let has_a = nc
        .split_whitespace()
        .any(|w| w.starts_with('A') && w.len() > 1 && w[1..].starts_with(|ch: char| ch.is_ascii_digit() || ch == '-' || ch == '.'));
    let has_b = nc
        .split_whitespace()
        .any(|w| w.starts_with('B') && w.len() > 1 && w[1..].starts_with(|ch: char| ch.is_ascii_digit() || ch == '-' || ch == '.'));
    if !has_a && !has_b {
        return Err(format!("expected A or B rotary words, has_rotary={has_rotary}\n{nc}"));
    }
    // Wall geometry: some cutting pose should be about 10° from +Z.
    if !ir_has_tilt_near(&ir, 10.0) {
        return Err(format!("IR tool axis was not about 10° from +Z\n{ir}"));
    }
    if nc.matches("M3").count() != 1 || nc.matches("M5").count() != 1 {
        return Err(format!("jet should turn on once and off once after the contour\n{nc}"));
    }
    check_golden("waterjet_taper.waterjet.nc", &nc)?;
    Ok(())
}

fn ir_has_tilt_near(ir: &str, deg: f64) -> bool {
    // Look for axis objects with a Z component near cos(deg).
    let target = deg.to_radians().cos();
    for part in ir.split("\"axis\"") {
        if let Some(z_idx) = part.find("\"z\"") {
            let rest = &part[z_idx + 3..];
            if let Some(num) = rest.split(&[',', '}', '\n'][..]).next() {
                let num = num.trim().trim_start_matches(':').trim();
                if let Ok(z) = num.parse::<f64>()
                    && (z - target).abs() < 0.02
                    && z < 0.999
                {
                    return true;
                }
            }
        }
    }
    false
}

#[test]
fn oxyfuel_bevel_uses_taper_and_writes_ab_words() -> Result<(), String> {
    let dir = temp_dir("campanion-kerf-tilt-oxyfuel")?;
    write_job(&dir, &rectangle(), r#", "preheat": true, "min_preheat": 0.2, "tilt": { "taper_deg": 15 }"#, "")?;
    let (nc, ir, _) = run_job(&dir, "oxyfuel")?;
    if !nc.contains("requested lead 0 deg, requested taper 15 deg") && !ir.contains("requested lead 0 deg, requested taper 15 deg") {
        return Err(format!("missing requested-angle comment\n{nc}"));
    }
    if !has_word(&nc, "G93") {
        return Err(format!("expected G93\n{nc}"));
    }
    if !ir_has_tilt_near(&ir, 15.0) {
        return Err(format!("IR tool axis was not about 15° from +Z\n{ir}"));
    }
    check_golden("oxyfuel_bevel.oxyfuel.nc", &nc)?;
    Ok(())
}

#[test]
fn example_travel_limits_reject_a_seventy_degree_taper() -> Result<(), String> {
    // The ±60° A/B travel is an example value in gantry_head_ab.json, not a real machine.
    let dir = temp_dir("campanion-kerf-tilt-limits")?;
    write_job(&dir, &rectangle(), r#", "tilt": { "taper_deg": 70 }"#, "")?;
    let stderr = run_fails(&dir, "waterjet")?;
    if !stderr.contains("outside the rotary limits") && !stderr.contains("leaves axis") {
        return Err(format!("expected travel-limit error, got {stderr}"));
    }
    Ok(())
}

#[test]
fn plasma_refuses_a_tilted_kerf_path() -> Result<(), String> {
    let dir = temp_dir("campanion-kerf-tilt-plasma")?;
    write_job(&dir, &rectangle(), r#", "tilt": { "taper_deg": 10 }"#, "")?;
    let stderr = run_fails(&dir, "plasma")?;
    if !stderr.contains("has no continuous five-axis output") {
        return Err(format!("expected plasma five-axis refusal, got {stderr}"));
    }
    Ok(())
}
