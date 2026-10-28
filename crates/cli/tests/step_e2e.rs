//! `campanion run` of the STEP sample job.
//!
//! STEP import shells out to `solidcraft-cli`. When that program is missing the STEP job
//! skips or fails with its name, and a DXF job still runs.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

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
    campanion_with_env(cwd, args, &[])
}

fn campanion_with_env(cwd: &Path, args: &[&str], envs: &[(&str, &str)]) -> Result<Output, String> {
    let mut command = Command::new(env!("CARGO_BIN_EXE_campanion"));
    command.args(args).current_dir(cwd);
    for (key, value) in envs {
        command.env(key, value);
    }
    command.output().map_err(|err| err.to_string())
}

/// `CAMPANION_SOLIDCRAFT_CLI` when that file exists, otherwise `solidcraft-cli` on `PATH`.
fn solidcraft_cli() -> Option<PathBuf> {
    if let Some(value) = std::env::var_os("CAMPANION_SOLIDCRAFT_CLI") {
        let path = PathBuf::from(value);
        if path.is_file() {
            return Some(path);
        }
    }
    let paths = std::env::var_os("PATH")?;
    for dir in std::env::split_paths(&paths) {
        let candidate = dir.join("solidcraft-cli");
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    None
}

fn last_nonempty_line(text: &str) -> &str {
    text.lines().rev().map(str::trim).find(|line| !line.is_empty()).unwrap_or("")
}

fn has_g2_or_g3(ngc: &str) -> bool {
    ngc.lines().any(|line| {
        let mut depth = 0i32;
        let mut code = String::new();
        for ch in line.chars() {
            match ch {
                '(' => depth += 1,
                ')' if depth > 0 => depth -= 1,
                _ if depth == 0 => code.push(ch),
                _ => {}
            }
        }
        code.split_whitespace().any(|word| matches!(word.to_ascii_uppercase().as_str(), "G2" | "G02" | "G3" | "G03"))
    })
}

#[test]
fn step_job_runs_when_solidcraft_cli_present() -> Result<(), String> {
    if solidcraft_cli().is_none() {
        eprintln!("skipping: solidcraft-cli not found");
        return Ok(());
    }
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-step-e2e.ngc");
    let ir_path = temp_path("campanion-step-e2e.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&root, &["run", "examples/step.job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("examples/step.job.json: run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("examples/step.job.json: source hash did not match: {stderr}"));
    }
    let ngc = std::fs::read_to_string(&ngc_path).map_err(|err| format!("examples/step.job.json: read ngc: {err}"))?;
    let last = last_nonempty_line(&ngc);
    if last != "M30" {
        return Err(format!("examples/step.job.json: last non-empty line is `{last}`, expected M30"));
    }
    let sections = ngc.matches("(toolpath").count();
    if sections < 2 {
        return Err(format!("examples/step.job.json: ngc has {sections} (toolpath sections, expected at least 2"));
    }
    if !has_g2_or_g3(&ngc) {
        return Err("examples/step.job.json: ngc has no G2 or G3 move".into());
    }
    for post in ["haas", "mach3", "linuxcnc", "grbl", "fanuc"] {
        let posted = temp_path(&format!("campanion-step-e2e-{post}.nc"));
        let posted_arg = path_arg(&posted)?;
        let output = campanion(&root, &["run", "examples/step.job.json", "-o", &posted_arg, "--post", post])?;
        let stderr = output_text(&output.stderr)?;
        let stdout = output_text(&output.stdout)?;
        if output.status.code() != Some(0) {
            return Err(format!("examples/step.job.json --post {post}: exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
        }
    }
    Ok(())
}

#[test]
fn step_job_without_cli_reports_clear_error() -> Result<(), String> {
    let root = workspace_root()?;
    let empty = temp_path("campanion-step-e2e-empty-path");
    if empty.exists() {
        std::fs::remove_dir_all(&empty).map_err(|err| err.to_string())?;
    }
    std::fs::create_dir_all(&empty).map_err(|err| err.to_string())?;
    let empty_arg = path_arg(&empty)?;
    let missing = empty.join("missing-step-reader");
    let missing_arg = path_arg(&missing)?;
    let hidden = [("CAMPANION_SOLIDCRAFT_CLI", missing_arg.as_str()), ("PATH", empty_arg.as_str())];

    let step_ngc = temp_path("campanion-step-e2e-nocli.ngc");
    let step_ir = temp_path("campanion-step-e2e-nocli.json");
    let step_ngc_arg = path_arg(&step_ngc)?;
    let step_ir_arg = path_arg(&step_ir)?;
    let step = campanion_with_env(&root, &["run", "examples/step.job.json", "-o", &step_ngc_arg, "--ir", &step_ir_arg], &hidden)?;
    let step_stderr = output_text(&step.stderr)?;
    let step_stdout = output_text(&step.stdout)?;
    if step.status.success() {
        return Err(format!("examples/step.job.json: run without solidcraft-cli exited 0 stdout {step_stdout} stderr {step_stderr}"));
    }
    if !step_stderr.contains("solidcraft-cli") {
        return Err(format!("examples/step.job.json: stderr does not mention solidcraft-cli\nstdout {step_stdout}\nstderr {step_stderr}"));
    }

    let dxf_ngc = temp_path("campanion-step-e2e-dxf.ngc");
    let dxf_ir = temp_path("campanion-step-e2e-dxf.json");
    let dxf_ngc_arg = path_arg(&dxf_ngc)?;
    let dxf_ir_arg = path_arg(&dxf_ir)?;
    let dxf = campanion_with_env(&root, &["run", "examples/sample.job.json", "-o", &dxf_ngc_arg, "--ir", &dxf_ir_arg], &hidden)?;
    let dxf_stderr = output_text(&dxf.stderr)?;
    let dxf_stdout = output_text(&dxf.stdout)?;
    if dxf.status.code() != Some(0) {
        return Err(format!("examples/sample.job.json: run exit {:?} stdout {dxf_stdout} stderr {dxf_stderr}", dxf.status.code()));
    }
    Ok(())
}

const Y_NOTE: &str = "note: STEP exported by SolidWorks (Y up); turned Y up to Z up. Use --up-axis z to keep the file's axes.";

const STEP_BYTES: &[u8] = b"ISO-10303-21;\n";

const FACE_DXF: &str = "\
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
0
90
4
70
1
10
0
20
0
10
40
20
0
10
40
20
30
10
0
20
30
0
ENDSEC
0
EOF
";

struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Result<Self, String> {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|elapsed| elapsed.as_nanos()).unwrap_or(0);
        let path = temp_path(&format!("campanion-up-{name}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&path).map_err(|err| err.to_string())?;
        Ok(Self(path))
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn fnv1a(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn write_step_job(dir: &Path) -> Result<PathBuf, String> {
    let step = dir.join("part.step");
    std::fs::write(&step, STEP_BYTES).map_err(|err| err.to_string())?;
    let job = dir.join("job.json");
    let text = format!(
        "{{\n  \"format\": \"campanion-job/1\",\n  \"source\": {{ \"path\": \"part.step\", \"hash\": \"{hash}\", \"face\": [10.0, 10.0, 0.0] }},\n  \"units\": \"mm\",\n  \"tools\": [ {{ \"id\": 1, \"diameter\": 6.0 }} ],\n  \"operations\": [ {{\n    \"name\": \"outer\",\n    \"type\": \"profile\",\n    \"layers\": [\"OUTER\"],\n    \"side\": \"outside\",\n    \"tool_id\": 1,\n    \"depth\": 1.0,\n    \"step_down\": 1.0,\n    \"feeds\": {{ \"cut\": 1000.0, \"plunge\": 300.0 }}\n  }} ]\n}}\n",
        hash = fnv1a(STEP_BYTES),
    );
    std::fs::write(&job, text).map_err(|err| err.to_string())?;
    Ok(job)
}

fn shell_quote(path: &Path) -> Result<String, String> {
    let text = path_arg(path)?;
    if text.contains('\'') {
        return Err(format!("path contains a single quote: {text}"));
    }
    Ok(format!("'{text}'"))
}

fn shell_single(text: &str) -> String {
    format!("'{}'", text.replace('\'', "'\\''"))
}

/// Records every invocation. A probe (`--json-out`) writes `probe_body` or fails with `probe_error`.
fn recording_cli(log: &Path, probe_body: Option<&str>, probe_error: Option<&str>) -> Result<String, String> {
    let on_probe = if let Some(body) = probe_body {
        format!("cat > \"$json_out\" << 'ENDJSON'\n{body}\nENDJSON\nexit 0")
    } else if let Some(message) = probe_error {
        format!("printf '%s\\n' {} >&2\nexit 4", shell_single(message))
    } else {
        "printf '%s\\n' 'probe was not expected' >&2\nexit 9".to_string()
    };
    let template = r#"#!/bin/sh
set -eu
log=__LOG__
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
__ON_PROBE__
fi
dxf=$(sed -n 's/.*"path":"\([^"]*\)".*/\1/p' "$2" | tail -n 1)
test -n "$dxf"
cat > "$dxf" << 'ENDDXF'
__DXF__
ENDDXF
"#;
    Ok(template.replace("__LOG__", &shell_quote(log)?).replace("__ON_PROBE__", &on_probe).replace("__DXF__", FACE_DXF.trim_end()))
}

fn write_executable(path: &Path, body: &str) -> Result<(), String> {
    use std::os::unix::fs::PermissionsExt;

    std::fs::write(path, body).map_err(|err| err.to_string())?;
    let mut perms = std::fs::metadata(path).map_err(|err| err.to_string())?.permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(path, perms).map_err(|err| err.to_string())?;
    Ok(())
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

fn assert_probe_call(args: &[String]) -> Result<(), String> {
    if args.len() != 6
        || args.first().map(String::as_str) != Some("run")
        || args.get(2).map(String::as_str) != Some("--results")
        || args.get(3).map(String::as_str) != Some("--json-out")
        || args.get(5).map(String::as_str) != Some("--quiet")
    {
        return Err(format!("probe argv: {args:?}"));
    }
    let out = args.get(4).map(String::as_str).unwrap_or("");
    if !out.ends_with("/probe.json") || args.iter().any(|arg| arg == "--in") {
        return Err(format!("probe argv: {args:?}"));
    }
    Ok(())
}

fn assert_probe_script(script: &str, step: &Path) -> Result<(), String> {
    let value: serde_json::Value = serde_json::from_str(script).map_err(|err| format!("{err}\n{script}"))?;
    let commands = value.as_array().ok_or_else(|| format!("probe script is not an array: {script}"))?;
    if commands.len() != 1 {
        return Err(format!("probe script: {script}"));
    }
    let open = commands.first().ok_or("empty probe")?;
    if open.get("command").and_then(|item| item.as_str()) != Some("doc.open") {
        return Err(format!("probe script: {script}"));
    }
    let params = open.get("params").and_then(|item| item.as_object()).ok_or_else(|| format!("probe params: {script}"))?;
    if params.contains_key("up") {
        return Err(format!("probe sent up: {script}"));
    }
    let path = params.get("path").and_then(|item| item.as_str()).ok_or_else(|| format!("probe path: {script}"))?;
    let absolute = std::path::absolute(step).map_err(|err| err.to_string())?;
    let absolute = absolute.to_str().ok_or("step path is not utf-8")?;
    if path != absolute {
        return Err(format!("probe path {path} != {absolute}"));
    }
    Ok(())
}

fn assert_in_call(args: &[String], step: &Path) -> Result<(), String> {
    let step_text = path_arg(step)?;
    if args.len() != 5
        || args.first().map(String::as_str) != Some("run")
        || args.get(2).map(String::as_str) != Some("--in")
        || args.get(3).map(String::as_str) != Some(step_text.as_str())
        || args.get(4).map(String::as_str) != Some("--quiet")
    {
        return Err(format!("--in argv: {args:?}"));
    }
    let script = args.get(1).map(String::as_str).unwrap_or("");
    if !script.ends_with("/script.json") || args.iter().any(|arg| arg == "--results") {
        return Err(format!("--in argv: {args:?}"));
    }
    Ok(())
}

fn assert_turn_call(args: &[String]) -> Result<(), String> {
    if args.len() != 3 || args.first().map(String::as_str) != Some("run") || args.get(2).map(String::as_str) != Some("--quiet") {
        return Err(format!("turn argv: {args:?}"));
    }
    if args.iter().any(|arg| arg == "--in" || arg == "--results") {
        return Err(format!("turn argv: {args:?}"));
    }
    let script = args.get(1).map(String::as_str).unwrap_or("");
    if !script.ends_with("/script.json") {
        return Err(format!("turn argv: {args:?}"));
    }
    Ok(())
}

fn assert_turn_script(script: &str, step: &Path) -> Result<(), String> {
    let value: serde_json::Value = serde_json::from_str(script).map_err(|err| format!("{err}\n{script}"))?;
    let commands = value.as_array().ok_or_else(|| format!("turn script is not an array: {script}"))?;
    if commands.len() != 3 {
        return Err(format!("turn script: {script}"));
    }
    let open = commands.first().ok_or("empty turn script")?;
    if open.get("command").and_then(|item| item.as_str()) != Some("doc.open")
        || open.pointer("/params/up").and_then(|item| item.as_str()) != Some("y")
    {
        return Err(format!("turn script: {script}"));
    }
    let path = open.pointer("/params/path").and_then(|item| item.as_str()).ok_or_else(|| format!("turn path: {script}"))?;
    let absolute = std::path::absolute(step).map_err(|err| err.to_string())?;
    let absolute = absolute.to_str().ok_or("step path is not utf-8")?;
    if path != absolute {
        return Err(format!("turn path {path} != {absolute}"));
    }
    let face = commands.get(1).and_then(|item| item.pointer("/params/plane/face")).and_then(|item| item.as_array()).ok_or("face")?;
    let coords: Vec<f64> = face.iter().filter_map(|item| item.as_f64()).collect();
    if coords.as_slice() != [10.0, 10.0, 0.0] {
        return Err(format!("face in turned frame: {coords:?}"));
    }
    Ok(())
}

fn assert_plain_script(script: &str) -> Result<(), String> {
    let value: serde_json::Value = serde_json::from_str(script).map_err(|err| format!("{err}\n{script}"))?;
    let commands = value.as_array().ok_or_else(|| format!("script is not an array: {script}"))?;
    if commands.len() != 2 || script.contains("doc.open") || script.contains("\"up\"") {
        return Err(format!("plain script: {script}"));
    }
    Ok(())
}

struct Ran {
    status: Option<i32>,
    stdout: String,
    stderr: String,
    calls: Vec<(Vec<String>, String)>,
}

fn run_with_cli(name: &str, extra: &[&str], probe_body: Option<&str>, probe_error: Option<&str>) -> Result<Ran, String> {
    let scratch = Scratch::new(name)?;
    let job = write_step_job(&scratch.0)?;
    let log = scratch.0.join("calls.log");
    let cli = scratch.0.join("solidcraft-cli");
    write_executable(&cli, &recording_cli(&log, probe_body, probe_error)?)?;
    let ngc = scratch.0.join("out.ngc");
    let mut args = vec!["run".to_string(), path_arg(&job)?, "-o".to_string(), path_arg(&ngc)?];
    args.extend(extra.iter().map(|item| (*item).to_string()));
    let arg_refs: Vec<&str> = args.iter().map(String::as_str).collect();
    let cli_arg = path_arg(&cli)?;
    let output = campanion_with_env(&scratch.0, &arg_refs, &[("CAMPANION_SOLIDCRAFT_CLI", cli_arg.as_str())])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    let log_text = std::fs::read_to_string(&log).unwrap_or_default();
    let calls = if log_text.is_empty() { Vec::new() } else { parse_calls(&log_text)? };
    Ok(Ran { status: output.status.code(), stdout, stderr, calls })
}

fn expect_success(ran: &Ran, label: &str) -> Result<(), String> {
    if ran.status != Some(0) {
        return Err(format!("{label}: exit {:?} stdout {} stderr {}", ran.status, ran.stdout, ran.stderr));
    }
    Ok(())
}

#[test]
fn step_auto_y_turns_and_notes() -> Result<(), String> {
    let reply = r#"[{"bodies":["plate"],"component":1,"exporter":"SolidWorks","exporter_up":"y","exporter_version":"2024","system":"SolidWorks 2024","warnings":[],"stand_up":2,"future":{"keep":true}},{"ignored":true}]"#;
    let ran = run_with_cli("auto-y", &["--up-axis", "auto"], Some(reply), None)?;
    expect_success(&ran, "auto")?;
    if ran.calls.len() != 2 {
        return Err(format!("auto: calls {:?}", ran.calls));
    }
    assert_probe_call(&ran.calls[0].0)?;
    let step = temp_step(&ran.calls[0].1)?;
    assert_probe_script(&ran.calls[0].1, &step)?;
    assert_turn_call(&ran.calls[1].0)?;
    assert_turn_script(&ran.calls[1].1, &step)?;
    if ran.calls[0].0.get(1).and_then(|path| Path::new(path).parent()) != ran.calls[1].0.get(1).and_then(|path| Path::new(path).parent()) {
        return Err("auto: probe and main used different temp dirs".into());
    }
    if ran.stderr.trim() != Y_NOTE {
        return Err(format!("auto: stderr {:?}", ran.stderr));
    }
    Ok(())
}

fn temp_step(probe_script: &str) -> Result<PathBuf, String> {
    let value: serde_json::Value = serde_json::from_str(probe_script).map_err(|err| err.to_string())?;
    let path = value.pointer("/0/params/path").and_then(|item| item.as_str()).ok_or_else(|| format!("step path missing in {probe_script}"))?;
    Ok(PathBuf::from(path))
}

#[test]
fn step_auto_non_y_uses_in() -> Result<(), String> {
    let replies = [
        ("z", r#"[{"exporter":"SolidCraft","exporter_up":"z","system":"SolidCraft","new_field":1}]"#),
        ("missing", r#"[{"bodies":["plate"],"warnings":[]}]"#),
        ("non-object", r#"["nope",{"exporter_up":"y","exporter":"SolidWorks"}]"#),
        ("empty", "[]"),
        ("garbage", "this is not json"),
        ("non-array", r#"{"exporter":"SolidWorks","exporter_up":"y"}"#),
    ];
    for (name, reply) in replies {
        let ran = run_with_cli(&format!("auto-keep-{name}"), &["--up-axis", "auto"], Some(reply), None)?;
        expect_success(&ran, name)?;
        if ran.calls.len() != 2 {
            return Err(format!("{name}: calls {:?}", ran.calls));
        }
        assert_probe_call(&ran.calls[0].0)?;
        let step = temp_step(&ran.calls[0].1)?;
        assert_probe_script(&ran.calls[0].1, &step)?;
        assert_in_call(&ran.calls[1].0, &step)?;
        assert_plain_script(&ran.calls[1].1)?;
        if ran.stderr.contains("note:") || ran.stderr.contains("turned Y up") {
            return Err(format!("{name}: unexpected note {}", ran.stderr));
        }
        if !ran.stderr.is_empty() {
            return Err(format!("{name}: stderr {}", ran.stderr));
        }
    }
    Ok(())
}

#[test]
fn step_up_axis_z_does_not_probe() -> Result<(), String> {
    let ran = run_with_cli("explicit-z", &["--up-axis", "z"], None, None)?;
    expect_success(&ran, "z")?;
    if ran.calls.len() != 1 {
        return Err(format!("z: calls {:?}", ran.calls));
    }
    let step = step_from_in_args(&ran.calls[0].0)?;
    assert_in_call(&ran.calls[0].0, &step)?;
    assert_plain_script(&ran.calls[0].1)?;
    if ran.stderr.contains("note:") {
        return Err(format!("z: stderr {}", ran.stderr));
    }
    Ok(())
}

fn step_from_in_args(args: &[String]) -> Result<PathBuf, String> {
    args.get(3).map(PathBuf::from).ok_or_else(|| format!("--in path missing: {args:?}"))
}

#[test]
fn step_default_is_z_and_does_not_probe() -> Result<(), String> {
    let reply = r#"[{"bodies":["plate"],"component":1,"exporter":"SolidWorks","exporter_up":"y","exporter_version":"2024","system":"SolidWorks 2024","warnings":[],"stand_up":2,"future":{"keep":true}},{"ignored":true}]"#;
    let ran = run_with_cli("default-z", &[], Some(reply), None)?;
    expect_success(&ran, "default")?;
    if ran.calls.len() != 1 {
        return Err(format!("default: calls {:?}", ran.calls));
    }
    let step = step_from_in_args(&ran.calls[0].0)?;
    assert_in_call(&ran.calls[0].0, &step)?;
    assert_plain_script(&ran.calls[0].1)?;
    if ran.stderr.contains("note:") || ran.stderr.contains("turned Y up") {
        return Err(format!("default: stderr {}", ran.stderr));
    }
    Ok(())
}

#[test]
fn step_up_axis_y_does_not_probe() -> Result<(), String> {
    let ran = run_with_cli("explicit-y", &["--up-axis", "y"], None, None)?;
    expect_success(&ran, "y")?;
    if ran.calls.len() != 1 {
        return Err(format!("y: calls {:?}", ran.calls));
    }
    assert_turn_call(&ran.calls[0].0)?;
    let value: serde_json::Value = serde_json::from_str(&ran.calls[0].1).map_err(|err| err.to_string())?;
    let path = value.pointer("/0/params/path").and_then(|item| item.as_str()).ok_or("turn path")?;
    assert_turn_script(&ran.calls[0].1, Path::new(path))?;
    if ran.stderr.contains("note:") {
        return Err(format!("y: stderr {}", ran.stderr));
    }
    Ok(())
}

#[test]
fn step_bad_up_axis_is_usage() -> Result<(), String> {
    let cases: &[(&[&str], &str)] = &[
        (&["run", "job.json", "-o", "out.ngc", "--up-axis"], "--up-axis needs a value"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", ""], "--up-axis needs a value"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", "Z"], "--up-axis must be z, y or auto"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", "Y"], "--up-axis must be z, y or auto"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", "Auto"], "--up-axis must be z, y or auto"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", "x"], "--up-axis must be z, y or auto"),
        (&["run", "job.json", "-o", "out.ngc", "--up-axis", "z", "--up-axis", "y"], "--up-axis is already set"),
    ];
    for (args, needle) in cases {
        let output = campanion(&workspace_root()?, args)?;
        let stderr = output_text(&output.stderr)?;
        if output.status.code() != Some(2) {
            return Err(format!("{args:?}: exit {:?} stderr {stderr}", output.status.code()));
        }
        if !stderr.contains(needle) || !stderr.contains("Usage:") {
            return Err(format!("{args:?}: stderr {stderr}"));
        }
    }
    Ok(())
}

#[test]
fn step_probe_failure_includes_stderr() -> Result<(), String> {
    let ran = run_with_cli("probe-fail", &["--up-axis", "auto"], None, Some("probe broke"))?;
    if ran.status == Some(0) {
        return Err(format!("probe failure exited 0: {}", ran.stderr));
    }
    if !ran.stderr.contains("probe broke") || !ran.stderr.contains("solidcraft-cli failed") {
        return Err(format!("probe failure stderr: {}", ran.stderr));
    }
    if ran.calls.len() != 1 {
        return Err(format!("probe failure continued: {:?}", ran.calls));
    }
    assert_probe_call(&ran.calls[0].0)?;
    Ok(())
}

#[test]
fn dxf_job_ignores_up_axis() -> Result<(), String> {
    let scratch = Scratch::new("dxf-ignore")?;
    let cli = scratch.0.join("solidcraft-cli");
    write_executable(&cli, "#!/bin/sh\nprintf '%s\\n' 'solidcraft should not run' >&2\nexit 7\n")?;
    let root = workspace_root()?;
    let ngc = scratch.0.join("out.ngc");
    let cli_arg = path_arg(&cli)?;
    let ngc_arg = path_arg(&ngc)?;
    for axis in ["z", "y", "auto"] {
        let output = campanion_with_env(
            &root,
            &["run", "examples/sample.job.json", "-o", &ngc_arg, "--up-axis", axis],
            &[("CAMPANION_SOLIDCRAFT_CLI", cli_arg.as_str())],
        )?;
        let stderr = output_text(&output.stderr)?;
        let stdout = output_text(&output.stdout)?;
        if output.status.code() != Some(0) {
            return Err(format!("dxf --up-axis {axis}: exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
        }
        if stderr.contains("solidcraft") {
            return Err(format!("dxf --up-axis {axis} invoked solidcraft: {stderr}"));
        }
    }
    Ok(())
}
