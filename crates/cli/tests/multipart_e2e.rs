//! Multi-part `campanion-job/2` planning and G-code, including an arrayed laser golden.

#![allow(clippy::expect_used)]

use std::path::{Path, PathBuf};
use std::process::Command;

const PART_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="10mm" height="10mm" viewBox="0 0 10 10">"#,
    r#"<rect x="1" y="1" width="8" height="8"/>"#,
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

fn hash() -> String {
    fnv1a64(PART_SVG.as_bytes())
}

fn v1_job() -> String {
    let hash = hash();
    format!(
        r#"{{
            "format": "campanion-job/1",
            "units": "mm",
            "source": {{"path": "part.svg", "hash": "{hash}"}},
            "tools": [{{"id": 1, "kerf": 0.2}}],
            "operations": [{{
                "name": "cut",
                "type": "profile",
                "layers": ["0"],
                "side": "outside",
                "tool_id": 1,
                "depth": 0.26,
                "step_down": 0.26,
                "feeds": {{"cut": 300}},
                "power": 0.8
            }}]
        }}"#
    )
}

fn v2_job(instances: &str, extra_ops: &str) -> String {
    let hash = hash();
    format!(
        r#"{{
            "format": "campanion-job/2",
            "units": "mm",
            "type": "cut_route",
            "parts": [{{
                "id": "bracket",
                "source": {{"path": "part.svg", "hash": "{hash}"}},
                "instances": {instances}
            }}],
            "tools": [{{"id": 1, "kerf": 0.2}}],
            "operations": [{{
                "name": "cut",
                "type": "profile",
                "selector": {{"type": "part", "parts": ["bracket"]}},
                "side": "outside",
                "tool_id": 1,
                "depth": 0.26,
                "step_down": 0.26,
                "feeds": {{"cut": 300}},
                "power": {{"unit": "percent", "value": 80}}
                {extra_ops}
            }}],
            "cut_order": ["cut"]
        }}"#
    )
}

fn write_part(dir: &Path) -> Result<(), String> {
    std::fs::write(dir.join("part.svg"), PART_SVG).map_err(|err| err.to_string())
}

fn run_job(dir: &Path, job: &str, out_name: &str, post: Option<&str>, env: &[(&str, &str)]) -> Result<(i32, String, String, Option<String>), String> {
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join(out_name);
    let out_arg = path_arg(&out)?;
    let mut args = vec!["run", "job.json", "-o", out_arg.as_str()];
    if let Some(post) = post {
        args.extend(["--post", post]);
    }
    let mut command = Command::new(env!("CARGO_BIN_EXE_campanion"));
    command.args(&args).current_dir(dir);
    for (key, value) in env {
        command.env(key, value);
    }
    let output = command.output().map_err(|err| err.to_string())?;
    let nc = if out.exists() { Some(std::fs::read_to_string(&out).map_err(|err| err.to_string())?) } else { None };
    Ok((output.status.code().unwrap_or(-1), text(&output.stdout)?, text(&output.stderr)?, nc))
}

fn assert_golden(name: &str, actual: &str) -> Result<(), String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        std::fs::write(&path, actual).map_err(|err| format!("write {}: {err}", path.display()))?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if expected != actual {
        return Err(format!("golden mismatch for {} (set UPDATE_GOLDEN=1 to regenerate)", path.display()));
    }
    Ok(())
}

#[test]
fn single_part_v1_and_equivalent_v2_match() -> Result<(), String> {
    let dir = temp_dir("campanion-multipart-single")?;
    write_part(&dir)?;
    let instances = r#"[{"id": "1", "placement": {"x": 0, "y": 0, "rotate_deg": 0}}]"#;
    let (code1, stdout1, stderr1, nc1) = run_job(&dir, &v1_job(), "v1.nc", Some("grbl_laser"), &[])?;
    if code1 != 0 {
        return Err(format!("v1 exit {code1} stdout {stdout1} stderr {stderr1}"));
    }
    let (code2, stdout2, stderr2, nc2) = run_job(&dir, &v2_job(instances, ""), "v2.nc", Some("grbl_laser"), &[])?;
    if code2 != 0 {
        return Err(format!("v2 exit {code2} stdout {stdout2} stderr {stderr2}"));
    }
    let nc1 = nc1.ok_or_else(|| "missing v1 g-code".to_string())?;
    let nc2 = nc2.ok_or_else(|| "missing v2 g-code".to_string())?;
    if nc1 != nc2 {
        return Err(format!("v1 and single-instance v2 G-code differ\n--- v1 ---\n{nc1}\n--- v2 ---\n{nc2}"));
    }
    Ok(())
}

#[test]
fn arrayed_two_instance_laser_matches_golden() -> Result<(), String> {
    let dir = temp_dir("campanion-multipart-array")?;
    write_part(&dir)?;
    let instances = r#"[
        {"id": "a", "placement": {"x": 0, "y": 0, "rotate_deg": 0}},
        {"id": "b", "placement": {"x": 20, "y": 0, "rotate_deg": 0}}
    ]"#;
    let (code, stdout, stderr, nc) = run_job(&dir, &v2_job(instances, ""), "array.nc", Some("grbl_laser"), &[])?;
    if code != 0 {
        return Err(format!("exit {code} stdout {stdout} stderr {stderr}"));
    }
    let nc = nc.ok_or_else(|| "missing g-code".to_string())?;
    if !nc.contains("X20") && !nc.contains("X20.") {
        return Err(format!("expected the second instance near X20, got:\n{nc}"));
    }
    assert_golden("multipart_array.grbl_laser.nc", &nc)
}

#[test]
fn unused_contour_is_a_planning_warning() -> Result<(), String> {
    let dir = temp_dir("campanion-multipart-unused")?;
    let svg = concat!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10">"#,
        r#"<g id="cut"><rect x="1" y="1" width="8" height="8"/></g>"#,
        r#"<g id="mark"><rect x="11" y="1" width="8" height="8"/></g>"#,
        "</svg>",
    );
    std::fs::write(dir.join("part.svg"), svg).map_err(|err| err.to_string())?;
    let hash = fnv1a64(svg.as_bytes());
    let job = format!(
        r#"{{
            "format": "campanion-job/2",
            "units": "mm",
            "parts": [{{
                "id": "bracket",
                "source": {{"path": "part.svg", "hash": "{hash}"}},
                "instances": [{{"id": "1"}}]
            }}],
            "tools": [{{"id": 1, "kerf": 0.2}}],
            "operations": [{{
                "name": "cut",
                "type": "profile",
                "selector": {{"type": "layer", "parts": ["bracket"], "layers": ["cut"]}},
                "side": "outside",
                "tool_id": 1,
                "depth": 0.26,
                "step_down": 0.26,
                "feeds": {{"cut": 300}},
                "power": {{"unit": "percent", "value": 80}}
            }}]
        }}"#
    );
    let (code, stdout, stderr, _) = run_job(&dir, &job, "unused.nc", None, &[])?;
    if code != 0 {
        return Err(format!("plan-only should succeed, exit {code} stdout {stdout} stderr {stderr}"));
    }
    if !stderr.contains("has no operation") {
        return Err(format!("expected an unused-contour warning, got: {stderr}"));
    }
    Ok(())
}

#[test]
fn off_bed_warns_in_plan_and_refuses_export() -> Result<(), String> {
    let dir = temp_dir("campanion-multipart-offbed")?;
    write_part(&dir)?;
    let machines = dir.join("config").join("machines");
    std::fs::create_dir_all(&machines).map_err(|err| err.to_string())?;
    let profile = r#"{
        "format": "campanion-machine/2",
        "name": "Shop laser",
        "units": "mm",
        "type": "gantry",
        "bed": {"x": 50, "y": 50},
        "heads": [{
            "id": "laser",
            "name": "CO2",
            "kind": "laser",
            "process": "laser",
            "post": "grbl_laser"
        }]
    }"#;
    std::fs::write(machines.join("shop.json"), profile).map_err(|err| err.to_string())?;
    let hash = hash();
    let job = format!(
        r#"{{
            "format": "campanion-job/2",
            "units": "mm",
            "type": "cut_route",
            "machine": {{"profile": "shop", "head": "laser"}},
            "parts": [{{
                "id": "bracket",
                "source": {{"path": "part.svg", "hash": "{hash}"}},
                "instances": [{{"id": "1", "placement": {{"x": 80, "y": 0, "rotate_deg": 0}}}}]
            }}],
            "tools": [{{"id": 1, "kerf": 0.2}}],
            "operations": [{{
                "name": "cut",
                "type": "profile",
                "selector": {{"type": "part", "parts": ["bracket"]}},
                "side": "outside",
                "tool_id": 1,
                "depth": 0.26,
                "step_down": 0.26,
                "feeds": {{"cut": 300}},
                "power": {{"unit": "percent", "value": 80}}
            }}]
        }}"#
    );
    let config = path_arg(&dir.join("config"))?;
    let env = [("CAMPANION_CONFIG_DIR", config.as_str())];
    let (code, stdout, stderr, _) = run_job(&dir, &job, "plan.ir.json", None, &env)?;
    if code != 0 {
        return Err(format!("plan-only should warn, not fail: exit {code} stdout {stdout} stderr {stderr}"));
    }
    if !stderr.contains("off the bed") {
        return Err(format!("expected an off-bed warning, got: {stderr}"));
    }
    let (code, stdout, stderr, _) = run_job(&dir, &job, "out.nc", Some("grbl_laser"), &env)?;
    if code == 0 {
        return Err(format!("export should refuse an off-bed part, stdout {stdout} stderr {stderr}"));
    }
    if !stderr.contains("refusing to export") || !stderr.contains("off the bed") {
        return Err(format!("expected an off-bed export refusal, got: {stderr}"));
    }
    Ok(())
}

#[test]
fn v2_rejects_raw_fraction_power() -> Result<(), String> {
    let dir = temp_dir("campanion-multipart-power")?;
    write_part(&dir)?;
    let hash = hash();
    let job = format!(
        r#"{{
            "format": "campanion-job/2",
            "units": "mm",
            "parts": [{{
                "id": "bracket",
                "source": {{"path": "part.svg", "hash": "{hash}"}},
                "instances": [{{"id": "1"}}]
            }}],
            "tools": [{{"id": 1, "kerf": 0.2}}],
            "operations": [{{
                "name": "cut",
                "type": "profile",
                "layers": ["0"],
                "side": "outside",
                "tool_id": 1,
                "depth": 0.26,
                "step_down": 0.26,
                "feeds": {{"cut": 300}},
                "power": 0.8
            }}]
        }}"#
    );
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["run", "job.json", "-o", "out.nc"])
        .current_dir(&dir)
        .output()
        .map_err(|err| err.to_string())?;
    if output.status.code() == Some(0) {
        return Err("v2 should refuse a raw 0..=1 power number".to_string());
    }
    let stderr = text(&output.stderr)?;
    if !stderr.contains("unit and value") && !stderr.contains("0..=1") {
        return Err(format!("expected a power-object error, got: {stderr}"));
    }
    Ok(())
}

#[test]
fn cancelled_plan_stops() -> Result<(), String> {
    use campanion_cli::{PlanCancel, PlanOptions, parse_job, plan_job_cancellable};
    let dir = temp_dir("campanion-multipart-cancel")?;
    write_part(&dir)?;
    let job = v2_job(r#"[{"id": "1"}]"#, "");
    let path = dir.join("job.json");
    std::fs::write(&path, &job).map_err(|err| err.to_string())?;
    let loaded = parse_job(&job, &path).map_err(|err| err.to_string())?;
    let cancel = PlanCancel::new();
    cancel.cancel();
    let err = plan_job_cancellable(&loaded, PlanOptions::default(), Some(&cancel)).expect_err("cancelled");
    if !err.to_string().contains("cancelled") {
        return Err(format!("expected a cancelled error, got: {err}"));
    }
    Ok(())
}
