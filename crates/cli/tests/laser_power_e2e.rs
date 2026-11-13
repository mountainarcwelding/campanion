//! A kerf job that omits `power` posts S0 (beam off), never full power.

#![allow(clippy::expect_used)]

use std::path::{Path, PathBuf};
use std::process::Command;

use campanion_post::{audit, builtin};

const PART_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10">"#,
    r#"<rect x="1" y="1" width="12.8" height="6"/>"#,
    "</svg>",
);

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

fn s_words_above_zero(text: &str) -> Vec<String> {
    let mut values = Vec::new();
    for line in text.lines() {
        let body = line.split('(').next().unwrap_or(line);
        for word in body.split_whitespace() {
            if let Some(rest) = word.strip_prefix('S')
                && let Ok(value) = rest.parse::<f64>()
                && value > 0.0
            {
                values.push(word.to_string());
            }
        }
    }
    values
}

#[test]
fn omitted_power_posts_s0_warns_and_never_full_scale() -> Result<(), String> {
    let dir = temp_dir("campanion-laser-power-e2e")?;
    std::fs::write(dir.join("part.svg"), PART_SVG).map_err(|err| err.to_string())?;
    let hash = fnv1a64(PART_SVG.as_bytes());
    // Evidence-shaped: ungrouped SVG (layer "0"), kerf profile, no power field.
    let job = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"kerf":0.1}}],"operations":[{{"name":"outline","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{{"cut":300.0}}}}]}}"#
    );
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let out_arg = path_arg(&out)?;
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["run", "job.json", "-o", out_arg.as_str(), "--post", "grbl_laser"])
        .current_dir(&dir)
        .output()
        .map_err(|err| err.to_string())?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if !stderr.contains("process power was not set") || !stderr.contains("S0") {
        return Err(format!("expected a stderr warning about unset power and S0, got: {stderr}"));
    }
    let nc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    if !nc.contains("power was not set, using S0") {
        return Err(format!("expected a G-code comment, got:\n{nc}"));
    }
    if !nc.contains("M4 S0") {
        return Err(format!("expected M4 S0, got:\n{nc}"));
    }
    if nc.contains("S1000") {
        return Err(format!("must not write full power:\n{nc}"));
    }
    let above = s_words_above_zero(&nc);
    if !above.is_empty() {
        return Err(format!("S above 0 in defaulted-power output {above:?}\n{nc}"));
    }
    let post = builtin("grbl_laser").expect("grbl_laser");
    audit(&nc, &post).map_err(|err| format!("audit: {err}\n{nc}"))?;

    // Audit must still catch a file that comments the default and then writes S above 0.
    let tampered = nc.replace("M4 S0", "M4 S1000");
    let err = audit(&tampered, &post).err().ok_or_else(|| "tampered S1000 must fail audit".to_string())?;
    if !err.to_string().contains("S1000 is above 0, but power was not set") {
        return Err(format!("unexpected audit error for tampered S1000: {err}"));
    }

    // The same refusal is reachable from the public CLI on a finished file.
    let good_arg = path_arg(&out)?;
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["audit", good_arg.as_str(), "--post", "grbl_laser"])
        .current_dir(&dir)
        .output()
        .map_err(|err| err.to_string())?;
    if output.status.code() != Some(0) {
        return Err(format!("audit of posted S0 file failed: stderr {}", text(&output.stderr)?));
    }
    let bad = dir.join("tampered.nc");
    std::fs::write(&bad, &tampered).map_err(|err| err.to_string())?;
    let bad_arg = path_arg(&bad)?;
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["audit", bad_arg.as_str(), "--post", "grbl_laser"])
        .current_dir(&dir)
        .output()
        .map_err(|err| err.to_string())?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(1) {
        return Err(format!("expected audit exit 1 for tampered S1000, got {:?} stderr {stderr}", output.status.code()));
    }
    if !stderr.contains("S1000 is above 0, but power was not set") {
        return Err(format!("audit CLI missed the missing-power refusal: {stderr}"));
    }
    Ok(())
}

#[test]
fn explicit_power_is_scaled_and_does_not_warn() -> Result<(), String> {
    let dir = temp_dir("campanion-laser-power-e2e-explicit")?;
    std::fs::write(dir.join("part.svg"), PART_SVG).map_err(|err| err.to_string())?;
    let hash = fnv1a64(PART_SVG.as_bytes());
    let job = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"kerf":0.1}}],"operations":[{{"name":"outline","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{{"cut":300.0}},"power":0.2}}]}}"#
    );
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let out_arg = path_arg(&out)?;
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["run", "job.json", "-o", out_arg.as_str(), "--post", "grbl_laser"])
        .current_dir(&dir)
        .output()
        .map_err(|err| err.to_string())?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("process power was not set") {
        return Err(format!("explicit power must not warn: {stderr}"));
    }
    let nc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    if nc.contains("power was not set") {
        return Err(format!("explicit power must not comment a default:\n{nc}"));
    }
    if !nc.contains("M4 S200") {
        return Err(format!("expected M4 S200, got:\n{nc}"));
    }
    let post = builtin("grbl_laser").expect("grbl_laser");
    audit(&nc, &post).map_err(|err| format!("audit: {err}\n{nc}"))?;
    Ok(())
}
