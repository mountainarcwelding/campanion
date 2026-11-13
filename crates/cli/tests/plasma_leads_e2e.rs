//! Default kerf arc leads, the small-hole rule, and job-file lead keys on `campanion run`.

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

fn dxf(entities: &str) -> String {
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn closed_lines(layer: &str, points: &[(f64, f64)]) -> String {
    let mut entities = String::new();
    for pair in points.windows(2) {
        entities.push_str(&line(layer, pair[0].0, pair[0].1, pair[1].0, pair[1].1));
    }
    if let (Some(first), Some(last)) = (points.first(), points.last()) {
        entities.push_str(&line(layer, last.0, last.1, first.0, first.1));
    }
    dxf(&entities)
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn write_job(dir: &Path, drawing: &str, job: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn profile_job(hash: &str, tool: &str, side: &str, extra: &str) -> String {
    let power = if tool.contains("kerf") { r#", "power": 1"# } else { "" };
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, {tool} }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "{side}",
    "tool_id": 1,
    "depth": 6.0,
    "step_down": 6.0,
    "feeds": {{ "cut": 1000, "plunge": 300 }}{power}{extra}
  }} ]
}}"#
    )
}

fn run_post(dir: &Path, post: &str) -> Result<String, String> {
    let out = dir.join(format!("{post}.nc"));
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    std::fs::read_to_string(&out).map_err(|err| err.to_string())
}

fn run_err(dir: &Path, post: &str) -> Result<String, String> {
    let out = dir.join(format!("{post}.nc"));
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
    if output.status.code() == Some(0) {
        return Err(format!("{post} unexpectedly succeeded"));
    }
    Ok(format!("{}\n{}", text(&output.stdout)?, text(&output.stderr)?))
}

fn is_arc_word(word: &str) -> bool {
    word == "G2" || word == "G3"
}

fn arc_radii(ngc: &str) -> Result<Vec<f64>, String> {
    let mut radii = Vec::new();
    for (index, raw) in ngc.lines().enumerate() {
        let mut is_arc = false;
        let mut i = None;
        let mut j = None;
        for word in raw.split_whitespace() {
            if is_arc_word(word) {
                is_arc = true;
            }
            if let Some(rest) = word.strip_prefix('I') {
                i = Some(rest.parse::<f64>().map_err(|err| format!("line {}: I: {err}", index + 1))?);
            }
            if let Some(rest) = word.strip_prefix('J') {
                j = Some(rest.parse::<f64>().map_err(|err| format!("line {}: J: {err}", index + 1))?);
            }
        }
        if is_arc {
            radii.push(i.unwrap_or(0.0).hypot(j.unwrap_or(0.0)));
        }
    }
    Ok(radii)
}

fn plate() -> &'static [(f64, f64)] {
    &[(0.0, 0.0), (80.0, 0.0), (80.0, 50.0), (0.0, 50.0)]
}

fn small_hole() -> &'static [(f64, f64)] {
    &[(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
}

#[test]
fn a_kerf_outside_profile_posts_default_arc_leads() -> Result<(), String> {
    let dir = temp_dir("campanion-plasma-leads-default")?;
    let drawing = closed_lines("PLATE", plate());
    let job = profile_job(&fnv1a64(drawing.as_bytes()), r#""kerf": 1.5"#, "outside", "");
    write_job(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "plasma")?;
    let radii = arc_radii(&ngc)?;
    if radii.len() < 2 {
        return Err(format!("expected lead arcs, got {radii:?}\n{ngc}"));
    }
    if !radii.iter().any(|radius| (radius - 1.5).abs() <= 1e-3) {
        return Err(format!("expected a 1.5 mm lead arc, got {radii:?}\n{ngc}"));
    }
    Ok(())
}

#[test]
fn an_explicit_none_lead_on_a_kerf_profile_has_no_arcs() -> Result<(), String> {
    let dir = temp_dir("campanion-plasma-leads-none")?;
    let drawing = closed_lines("PLATE", plate());
    let job =
        profile_job(&fnv1a64(drawing.as_bytes()), r#""kerf": 1.5"#, "outside", r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#);
    write_job(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "plasma")?;
    let radii = arc_radii(&ngc)?;
    if radii.iter().any(|radius| (radius - 1.5).abs() <= 1e-3) {
        return Err(format!("explicit none should not keep a 1.5 mm lead arc, got {radii:?}\n{ngc}"));
    }
    if radii.is_empty() || radii.iter().any(|radius| (radius - 0.75).abs() > 1e-3) {
        return Err(format!("expected only 0.75 mm corner arcs, got {radii:?}\n{ngc}"));
    }
    Ok(())
}

#[test]
fn a_diameter_tool_defaults_to_no_leads() -> Result<(), String> {
    let dir = temp_dir("campanion-plasma-leads-mill")?;
    let drawing = closed_lines("PLATE", plate());
    let job = profile_job(&fnv1a64(drawing.as_bytes()), r#""diameter": 6.0, "speed": 12000"#, "outside", "");
    write_job(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "linuxcnc")?;
    let radii = arc_radii(&ngc)?;
    if radii.is_empty() || radii.iter().any(|radius| (radius - 3.0).abs() > 1e-3) {
        return Err(format!("mill rectangle should only have 3 mm corner arcs, got {radii:?}\n{ngc}"));
    }
    Ok(())
}

#[test]
fn a_small_inside_hole_does_not_keep_a_full_kerf_lead() -> Result<(), String> {
    let dir = temp_dir("campanion-plasma-leads-hole")?;
    let drawing = closed_lines("PLATE", small_hole());
    let job = profile_job(&fnv1a64(drawing.as_bytes()), r#""kerf": 2.0"#, "inside", "");
    write_job(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "plasma")?;
    let radii = arc_radii(&ngc)?;
    if radii.iter().any(|radius| (radius - 2.0).abs() <= 1e-3) {
        return Err(format!("small hole kept a full 2 mm lead\n{radii:?}\n{ngc}"));
    }
    Ok(())
}

#[test]
fn leads_are_rejected_with_a_ramp_entry() -> Result<(), String> {
    let dir = temp_dir("campanion-plasma-leads-ramp")?;
    let drawing = closed_lines("PLATE", plate());
    let job = profile_job(
        &fnv1a64(drawing.as_bytes()),
        r#""diameter": 6.0, "speed": 12000"#,
        "outside",
        r#", "lead_in": {"type": "line", "length": 5}, "entry": {"type": "ramp", "angle_deg": 10}"#,
    );
    write_job(&dir, &drawing, &job)?;
    let err = run_err(&dir, "linuxcnc")?;
    if !err.contains("leads are not valid with a ramp or helix entry") {
        return Err(format!("expected ramp refusal, got {err}"));
    }
    Ok(())
}
