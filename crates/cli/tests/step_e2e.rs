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
