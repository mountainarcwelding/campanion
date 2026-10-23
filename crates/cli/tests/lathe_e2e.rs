//! `campanion lathe` on the stepped-shaft example, and hostile job files.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output().map_err(|err| err.to_string())
}

fn run_ok(args: &[&str]) -> Result<Output, String> {
    let output = campanion(args)?;
    if output.status.code() != Some(0) {
        let stdout = text(&output.stdout).unwrap_or_default();
        let stderr = text(&output.stderr).unwrap_or_default();
        return Err(format!("exit {:?} args {args:?}\nstdout {stdout}\nstderr {stderr}", output.status.code()));
    }
    Ok(output)
}

#[test]
fn shaft_posts_linuxcnc_lathe_and_keeps_three_bindings() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/shaft.lathe.json"))?;
    let out = temp_path("campanion-lathe-shaft.nc");
    let ir = temp_path("campanion-lathe-shaft.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "linuxcnc_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    for word in ["G7", "G18", "G96"] {
        if !posted.contains(word) {
            return Err(format!("posted program is missing {word}\n{posted}"));
        }
    }
    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let doc = campanion_ir::Document::from_json(&ir_text).map_err(|err| err.to_string())?;
    let turning = doc.program().turning().ok_or("turning setup is missing")?;
    if turning.bindings().len() != 3 {
        return Err(format!("expected 3 bindings, found {}", turning.bindings().len()));
    }
    if doc.program().toolpaths().len() != 3 {
        return Err(format!("expected 3 toolpaths, found {}", doc.program().toolpaths().len()));
    }
    Ok(())
}

fn assert_rejects(body: &str, file_name: &str, needle: &str) -> Result<(), String> {
    let job = temp_path(file_name);
    std::fs::write(&job, body).map_err(|err| err.to_string())?;
    let out = temp_path(&format!("{file_name}.nc"));
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["lathe", &path_arg(&job)?, "-o", &path_arg(&out)?])?;
    let stderr = text(&output.stderr)?;
    let code = output.status.code();
    if code == Some(101) || stderr.to_ascii_lowercase().contains("panicked") {
        return Err(format!("lathe panicked ({code:?}): {stderr}"));
    }
    if code != Some(1) {
        return Err(format!("exit {code:?}, expected 1\n{stderr}"));
    }
    if !stderr.contains(needle) {
        return Err(format!("stderr missing {needle}: {stderr}"));
    }
    if out.exists() {
        return Err(format!("failed job wrote {}", out.display()));
    }
    Ok(())
}

#[test]
fn hostile_jobs_error_without_panic() -> Result<(), String> {
    let unknown = r#"{
        "format": "campanion-lathe/1",
        "surprise": 1,
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [],
        "operations": [],
        "clearance": 1
    }"#;
    assert_rejects(unknown, "campanion-lathe-unknown.json", "surprise")?;

    let nan = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [],
        "operations": [],
        "clearance": NaN
    }"#;
    assert_rejects(nan, "campanion-lathe-nan.json", "json")?;

    let missing = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }],
        "operations": [{ "name": "face", "type": "face", "tool_id": 99, "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }],
        "clearance": 1
    }"#;
    assert_rejects(missing, "campanion-lathe-missing-tool.json", "missing tool 99")?;
    Ok(())
}
