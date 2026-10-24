//! `campanion lathe` on a STEP source, with a fake `solidcraft-cli`.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn output_text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Result<Self, String> {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|elapsed| elapsed.as_nanos()).unwrap_or(0);
        let path = temp_path(&format!("campanion-lathe-step-{name}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&path).map_err(|err| err.to_string())?;
        Ok(Self(path))
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn write_executable(path: &Path, body: &str) -> Result<(), String> {
    use std::os::unix::fs::PermissionsExt;

    std::fs::write(path, body).map_err(|err| err.to_string())?;
    let mut perms = std::fs::metadata(path).map_err(|err| err.to_string())?.permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(path, perms).map_err(|err| err.to_string())?;
    Ok(())
}

fn fixture_dxf() -> Result<PathBuf, String> {
    workspace_root()?.join("crates/turn/tests/fixtures/section_shaft_seam.dxf").canonicalize().map_err(|err| err.to_string())
}

fn fake_cli() -> &'static str {
    r#"#!/bin/sh
set -eu
log=$CAMPANION_FAKE_LOG
{
  printf '%s\n' 'CALL'
  printf '%s\n' "$@"
  printf '%s\n' 'ENDARGS'
  printf '%s\n' 'SCRIPT'
  cat -- "$2"
  printf '\n%s\n' 'ENDSCRIPT'
} >> "$log"
json_out=
prev=
for arg in "$@"; do
  if [ "$prev" = "--json-out" ]; then
    json_out=$arg
  fi
  prev=$arg
done
if [ -n "$json_out" ]; then
  cp "$CAMPANION_FAKE_PROBE" "$json_out"
  exit 0
fi
dxf=$(sed -n 's/.*"path":"\([^"]*\)".*/\1/p' "$2" | tail -n 1)
test -n "$dxf"
cp "$CAMPANION_FAKE_DXF" "$dxf"
"#
}

fn write_job(dir: &Path, body: Option<&str>) -> Result<PathBuf, String> {
    std::fs::write(dir.join("part.step"), b"ISO-10303-21;\n").map_err(|err| err.to_string())?;
    let source = match body {
        Some(name) => format!(r#"{{ "path": "part.step", "body": "{name}" }}"#),
        None => r#"{ "path": "part.step" }"#.to_string(),
    };
    let text = format!(
        r#"{{
  "format": "campanion-lathe/1",
  "source": {source},
  "units": "mm",
  "stock": {{ "kind": "bar", "diameter": 50, "length": 60, "front_allowance": 1.0 }},
  "spindle": {{ "max_rpm": 3000 }},
  "tools": [
    {{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.8, "orientation_deg": 45, "hand": "right", "tip": "centre" }}
  ],
  "operations": [
    {{ "name": "face", "type": "face", "tool_id": 1, "step_z": 0.5, "feed": 150, "speed": {{ "css": 200 }} }},
    {{ "name": "rough", "type": "rough", "tool_id": 1, "side": "outside", "depth_of_cut": 2.0, "leave_x": 0.3, "leave_z": 0.1, "feed": 250, "speed": {{ "css": 180 }} }},
    {{ "name": "finish", "type": "finish", "tool_id": 1, "side": "outside", "feed": 120, "speed": {{ "rpm": 1500 }} }}
  ],
  "tolerance": {{ "linear": 0.001, "chord": 0.01 }},
  "clearance": 2.0
}}
"#
    );
    let job = dir.join("job.json");
    std::fs::write(&job, text).map_err(|err| err.to_string())?;
    Ok(job)
}

fn write_probe(dir: &Path, body: &str) -> Result<PathBuf, String> {
    let path = dir.join("probe.json");
    std::fs::write(&path, body).map_err(|err| err.to_string())?;
    Ok(path)
}

struct Ran {
    status: Option<i32>,
    stdout: String,
    stderr: String,
    calls: Vec<(Vec<String>, String)>,
}

fn run_lathe(name: &str, body: Option<&str>, probe: &str, extra: &[&str]) -> Result<Ran, String> {
    let scratch = Scratch::new(name)?;
    let job = write_job(&scratch.0, body)?;
    let log = scratch.0.join("calls.log");
    let cli = scratch.0.join("solidcraft-cli");
    write_executable(&cli, fake_cli())?;
    let probe_path = write_probe(&scratch.0, probe)?;
    let dxf = fixture_dxf()?;
    let out = scratch.0.join("out.nc");
    let mut args = vec!["lathe".to_string(), path_arg(&job)?, "-o".to_string(), path_arg(&out)?, "--post".to_string(), "linuxcnc_lathe".to_string()];
    args.extend(extra.iter().map(|item| (*item).to_string()));
    let arg_refs: Vec<&str> = args.iter().map(String::as_str).collect();
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(&arg_refs)
        .current_dir(&scratch.0)
        .env("CAMPANION_SOLIDCRAFT_CLI", path_arg(&cli)?)
        .env("CAMPANION_FAKE_LOG", path_arg(&log)?)
        .env("CAMPANION_FAKE_PROBE", path_arg(&probe_path)?)
        .env("CAMPANION_FAKE_DXF", path_arg(&dxf)?)
        .output()
        .map_err(|err| err.to_string())?;
    ran_from(output, &log)
}

fn ran_from(output: Output, log: &Path) -> Result<Ran, String> {
    let log_text = std::fs::read_to_string(log).unwrap_or_default();
    let calls = if log_text.is_empty() { Vec::new() } else { parse_calls(&log_text)? };
    Ok(Ran { status: output.status.code(), stdout: output_text(&output.stdout)?, stderr: output_text(&output.stderr)?, calls })
}

fn parse_calls(text: &str) -> Result<Vec<(Vec<String>, String)>, String> {
    let lines: Vec<&str> = text.lines().collect();
    let mut calls = Vec::new();
    let mut index = 0usize;
    while index < lines.len() {
        if lines.get(index).copied() != Some("CALL") {
            return Err(format!("call log out of order:\n{text}"));
        }
        index += 1;
        let mut args = Vec::new();
        while let Some(line) = lines.get(index).copied() {
            if line == "ENDARGS" {
                break;
            }
            args.push(line.to_string());
            index += 1;
        }
        if lines.get(index).copied() != Some("ENDARGS") {
            return Err(format!("call log missing ENDARGS:\n{text}"));
        }
        index += 1;
        if lines.get(index).copied() != Some("SCRIPT") {
            return Err(format!("call log missing SCRIPT:\n{text}"));
        }
        index += 1;
        let mut script = Vec::new();
        while let Some(line) = lines.get(index).copied() {
            if line == "ENDSCRIPT" {
                break;
            }
            script.push(line.to_string());
            index += 1;
        }
        if lines.get(index).copied() != Some("ENDSCRIPT") {
            return Err(format!("call log missing ENDSCRIPT:\n{text}"));
        }
        index += 1;
        calls.push((args, script.join("\n")));
    }
    Ok(calls)
}

fn section_script(ran: &Ran) -> Result<&str, String> {
    let Some(call) = ran.calls.get(1) else {
        return Err(format!("expected probe then section, calls {:?}", ran.calls.iter().map(|(args, _)| args).collect::<Vec<_>>()));
    };
    Ok(&call.1)
}

const ONE_BODY: &str = r#"[{"bodies":["shaft"],"exporter":"SolidCraft","exporter_up":"y","warnings":[]}]"#;

#[test]
fn one_body_step_posts_linuxcnc_lathe() -> Result<(), String> {
    let ran = run_lathe("one", None, ONE_BODY, &[])?;
    if ran.status != Some(0) {
        return Err(format!("exit {:?} stdout {} stderr {}", ran.status, ran.stdout, ran.stderr));
    }
    Ok(())
}

#[test]
fn two_bodies_without_source_body_ask_for_one() -> Result<(), String> {
    let probe = r#"[{"bodies":["shaft","boss"],"exporter":"SolidCraft","exporter_up":"z"}]"#;
    let ran = run_lathe("two", None, probe, &[])?;
    if ran.status == Some(0) {
        return Err(format!("two bodies exited 0\n{}", ran.stderr));
    }
    if !ran.stderr.contains("set source.body") {
        return Err(format!("stderr missing set source.body: {}", ran.stderr));
    }
    Ok(())
}

#[test]
fn unknown_body_names_the_missing_solid() -> Result<(), String> {
    let ran = run_lathe("missing-body", Some("nope"), ONE_BODY, &[])?;
    if ran.status == Some(0) {
        return Err(format!("unknown body exited 0\n{}", ran.stderr));
    }
    if !ran.stderr.contains("has no body") {
        return Err(format!("stderr missing has no body: {}", ran.stderr));
    }
    Ok(())
}

#[test]
fn up_axis_y_sends_up_y() -> Result<(), String> {
    let ran = run_lathe("turn-y", None, ONE_BODY, &["--up-axis", "y"])?;
    if ran.status != Some(0) {
        return Err(format!("exit {:?} stdout {} stderr {}", ran.status, ran.stdout, ran.stderr));
    }
    let script = section_script(&ran)?;
    if !script.contains("\"up\":\"y\"") {
        return Err(format!("section script did not turn Y up:\n{script}"));
    }
    Ok(())
}

#[test]
fn default_up_axis_does_not_turn_or_note() -> Result<(), String> {
    let ran = run_lathe("default-z", None, ONE_BODY, &[])?;
    if ran.status != Some(0) {
        return Err(format!("exit {:?} stdout {} stderr {}", ran.status, ran.stdout, ran.stderr));
    }
    let script = section_script(&ran)?;
    if script.contains("\"up\"") {
        return Err(format!("default sent up:\n{script}"));
    }
    if ran.stderr.contains("turned Y up") || ran.stderr.contains("note:") {
        return Err(format!("default printed the Y note: {}", ran.stderr));
    }
    Ok(())
}

#[test]
fn missing_cli_uses_the_existing_message() -> Result<(), String> {
    let scratch = Scratch::new("nocli")?;
    let job = write_job(&scratch.0, None)?;
    let empty = scratch.0.join("empty-path");
    std::fs::create_dir_all(&empty).map_err(|err| err.to_string())?;
    let missing = scratch.0.join("missing-solidcraft-cli");
    let out = scratch.0.join("out.nc");
    let output = Command::new(env!("CARGO_BIN_EXE_campanion"))
        .args(["lathe", &path_arg(&job)?, "-o", &path_arg(&out)?, "--post", "linuxcnc_lathe"])
        .current_dir(&scratch.0)
        .env("CAMPANION_SOLIDCRAFT_CLI", path_arg(&missing)?)
        .env("PATH", path_arg(&empty)?)
        .output()
        .map_err(|err| err.to_string())?;
    let stderr = output_text(&output.stderr)?;
    if output.status.success() {
        return Err(format!("missing cli exited 0\n{stderr}"));
    }
    let expected =
        "STEP import needs SolidCraft's solidcraft-cli: install SolidCraft or set CAMPANION_SOLIDCRAFT_CLI to the solidcraft-cli executable";
    if !stderr.contains(expected) {
        return Err(format!("stderr was {stderr}"));
    }
    Ok(())
}
