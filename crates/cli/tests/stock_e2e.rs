//! In-process stock on `examples/dome_stock.job.json`.
//!
//! The example roughs and finishes the dome inside an explicit box and prints the leftover
//! summary. A copy whose box rises above the rapid plane fails before anything is written.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let base = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let dir = base.join(name);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn output_text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(cwd: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

/// `to` relative to `from_dir`, using `..` so the job file can sit outside `examples/`.
fn relative_to(from_dir: &Path, to: &Path) -> Result<String, String> {
    let from = from_dir.canonicalize().map_err(|err| format!("{}: {err}", from_dir.display()))?;
    let to = to.canonicalize().map_err(|err| format!("{}: {err}", to.display()))?;
    let mut ups = 0usize;
    let mut base = from.as_path();
    let suffix = loop {
        if let Ok(rest) = to.strip_prefix(base) {
            break rest.to_path_buf();
        }
        ups += 1;
        base = base.parent().ok_or_else(|| format!("no common prefix for {} and {}", from.display(), to.display()))?;
    };
    let mut out = String::new();
    for _ in 0..ups {
        out.push_str("../");
    }
    let text = suffix.to_string_lossy().replace('\\', "/");
    out.push_str(&text);
    Ok(out)
}

#[test]
fn dome_stock_prints_the_leftover_summary() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc = temp_dir("stock-e2e-ok")?.join("out.ngc");
    let output = campanion(&root, &["run", "examples/dome_stock.job.json", "-o", &path_arg(&ngc)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if !stderr.contains("stock: max leftover ") || !stderr.contains("model gouges ") {
        return Err(format!("missing leftover summary:\n{stderr}"));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    if !ngc.is_file() {
        return Err("expected an NGC file".to_string());
    }
    Ok(())
}

#[test]
fn rapid_below_the_stock_top_writes_nothing() -> Result<(), String> {
    let root = workspace_root()?;
    let dir = temp_dir("stock-e2e-rapid")?;
    let stl = relative_to(&dir, &root.join("examples/dome.stl"))?;
    let job_text = std::fs::read_to_string(root.join("examples/dome_stock.job.json")).map_err(|err| err.to_string())?;
    let job_text = job_text.replace("\"path\": \"dome.stl\"", &format!("\"path\": \"{stl}\""));
    // The planner rapids at Z 15. Raising the box above that plane puts the flute in the stock.
    let job_text = job_text.replace("\"max\": [23, 23, 10]", "\"max\": [23, 23, 40]");
    let job = dir.join("rapid_in_stock.job.json");
    std::fs::write(&job, job_text).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let output = campanion(&dir, &["run", "rapid_in_stock.job.json", "-o", &path_arg(&ngc)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() == Some(0) {
        return Err(format!("rapid into stock was accepted\nstdout {stdout}\nstderr {stderr}"));
    }
    if !stderr.contains("rapid into stock at (") || !stderr.contains(", depth ") {
        return Err(format!("missing rapid-into-stock message\nstdout {stdout}\nstderr {stderr}"));
    }
    if ngc.exists() {
        return Err(format!("output was written: {}", ngc.display()));
    }
    Ok(())
}
