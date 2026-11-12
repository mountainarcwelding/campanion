//! Narrow-kerf slot refusal and the freed-slug rapid warning.

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

fn closed_lines(layer: &str, polys: &[&[(f64, f64)]]) -> String {
    let mut entities = String::new();
    for poly in polys {
        for pair in poly.windows(2) {
            entities.push_str(&line(layer, pair[0].0, pair[0].1, pair[1].0, pair[1].1));
        }
        if let (Some(first), Some(last)) = (poly.first(), poly.last()) {
            entities.push_str(&line(layer, last.0, last.1, first.0, first.1));
        }
    }
    dxf(&entities)
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn write_files(dir: &Path, drawing: &str, job: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn run(dir: &Path) -> Result<(i32, String, String), String> {
    let out = dir.join("out.nc");
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", "plasma"])?;
    Ok((output.status.code().unwrap_or(-1), text(&output.stdout)?, text(&output.stderr)?))
}

fn job(hash: &str, tool: &str, side: &str, extra: &str) -> String {
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
    "feeds": {{ "cut": 1000, "plunge": 300 }}{extra}
  }} ]
}}"#
    )
}

#[test]
fn a_slot_narrower_than_the_kerf_is_refused() -> Result<(), String> {
    let dir = temp_dir("campanion-slot-kerf")?;
    let slot: &[(f64, f64)] = &[(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (0.0, 1.0)];
    let drawing = closed_lines("PLATE", &[slot]);
    write_files(
        &dir,
        &drawing,
        &job(&fnv1a64(drawing.as_bytes()), r#""kerf": 2.0"#, "inside", r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#),
    )?;
    let (code, stdout, stderr) = run(&dir)?;
    let err = format!("{stdout}\n{stderr}");
    if code == 0 {
        return Err(format!("narrow slot should have failed\n{err}"));
    }
    if !err.contains("slot is narrower than the kerf") {
        return Err(format!("expected slot error, got {err}"));
    }
    Ok(())
}

#[test]
fn a_diameter_tool_still_says_it_does_not_fit() -> Result<(), String> {
    let dir = temp_dir("campanion-slot-mill")?;
    let slot: &[(f64, f64)] = &[(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (0.0, 1.0)];
    let drawing = closed_lines("PLATE", &[slot]);
    write_files(
        &dir,
        &drawing,
        &job(
            &fnv1a64(drawing.as_bytes()),
            r#""diameter": 6.0, "speed": 12000"#,
            "inside",
            r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#,
        ),
    )?;
    let out = dir.join("out.nc");
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", "linuxcnc"])?;
    let err = format!("{}\n{}", text(&output.stdout)?, text(&output.stderr)?);
    if output.status.code() == Some(0) {
        return Err(format!("narrow mill slot should have failed\n{err}"));
    }
    if !err.contains("tool does not fit inside the contour") {
        return Err(format!("expected mill fit error, got {err}"));
    }
    Ok(())
}

#[test]
fn a_rapid_after_a_tabless_hole_warns_about_the_freed_slug() -> Result<(), String> {
    let dir = temp_dir("campanion-slug-warn")?;
    let plate: &[(f64, f64)] = &[(0.0, 0.0), (80.0, 0.0), (80.0, 50.0), (0.0, 50.0)];
    let hole: &[(f64, f64)] = &[(20.0, 15.0), (40.0, 15.0), (40.0, 35.0), (20.0, 35.0)];
    let drawing = closed_lines("PLATE", &[plate, hole]);
    write_files(
        &dir,
        &drawing,
        &job(&fnv1a64(drawing.as_bytes()), r#""kerf": 2.0"#, "auto", r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#),
    )?;
    let (code, stdout, stderr) = run(&dir)?;
    if code != 0 {
        return Err(format!("slug job failed: {stdout}\n{stderr}"));
    }
    if !stderr.contains("a rapid crosses a freed slug") {
        return Err(format!("expected slug warning, stderr was:\n{stderr}\n{stdout}"));
    }
    Ok(())
}

#[test]
fn tabs_on_a_hole_do_not_warn_about_a_freed_slug() -> Result<(), String> {
    let dir = temp_dir("campanion-slug-tabs")?;
    let plate: &[(f64, f64)] = &[(0.0, 0.0), (80.0, 0.0), (80.0, 50.0), (0.0, 50.0)];
    let hole: &[(f64, f64)] = &[(20.0, 15.0), (40.0, 15.0), (40.0, 35.0), (20.0, 35.0)];
    let drawing = closed_lines("PLATE", &[plate, hole]);
    write_files(
        &dir,
        &drawing,
        &job(
            &fnv1a64(drawing.as_bytes()),
            r#""kerf": 2.0"#,
            "auto",
            r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}, "tabs": {"count": 4, "width": 3, "height": 2}"#,
        ),
    )?;
    let (code, stdout, stderr) = run(&dir)?;
    if code != 0 {
        return Err(format!("tabbed job failed: {stdout}\n{stderr}"));
    }
    if stderr.contains("a rapid crosses a freed slug") {
        return Err(format!("tabs should hold the slug, but warned:\n{stderr}"));
    }
    Ok(())
}
