//! `run --post`, `post`, and `posts`.

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
    let stderr = text(&output.stderr)?;
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    Ok(output)
}

fn sample_job(root: &Path) -> Result<String, String> {
    path_arg(&root.join("examples/sample.job.json"))
}

fn committed_ir(root: &Path) -> Result<String, String> {
    path_arg(&root.join("crates/post/tests/data/sample.ir.json"))
}

fn first_diff(left: &str, right: &str) -> String {
    for (index, (a, b)) in left.lines().zip(right.lines()).enumerate() {
        if a != b {
            return format!("line {}: {a:?} vs {b:?}", index + 1);
        }
    }
    format!("lengths {} vs {}", left.len(), right.len())
}

#[test]
fn sample_job_posts_with_each_builtin() -> Result<(), String> {
    let root = workspace_root()?;
    let job = sample_job(&root)?;
    for id in ["linuxcnc", "grbl"] {
        let out = temp_path(&format!("campanion-post-e2e-{id}.nc"));
        let out_arg = path_arg(&out)?;
        run_ok(&["run", &job, "-o", &out_arg, "--post", id])?;
        let posted = std::fs::read_to_string(&out).map_err(|err| format!("{id}: {err}"))?;
        if posted.trim().is_empty() {
            return Err(format!("{id}: posted output is empty"));
        }
        if posted.contains("(program ") {
            return Err(format!("{id}: output is the test sink\n{posted}"));
        }
        match id {
            "linuxcnc" => {
                if !posted.starts_with("N10 ") || !posted.ends_with("M2\n") || !posted.contains("(sample.job)") {
                    return Err(format!("{id}: unexpected linuxcnc program\n{posted}"));
                }
            }
            "grbl" => {
                if !posted.starts_with("(sample.job)\n") || !posted.ends_with("M30\n") || posted.lines().any(|line| line.starts_with('N')) {
                    return Err(format!("{id}: unexpected grbl program\n{posted}"));
                }
            }
            _ => return Err(format!("untested builtin {id}")),
        }
    }
    Ok(())
}

#[test]
fn custom_post_file_from_temp_dir() -> Result<(), String> {
    let root = workspace_root()?;
    let source = root.join("crates/post/posts/linuxcnc.json");
    let raw = std::fs::read_to_string(&source).map_err(|err| format!("{}: {err}", source.display()))?;
    let mut value: serde_json::Value = serde_json::from_str(&raw).map_err(|err| err.to_string())?;
    let Some(object) = value.as_object_mut() else {
        return Err("linuxcnc.json is not an object".to_string());
    };
    if object.get("program_end").is_none() {
        return Err("linuxcnc.json has no program_end".to_string());
    }
    object.insert("program_end".to_string(), serde_json::json!(["(campanion-custom-post)", "M2"]));
    let dir = temp_path("campanion-custom-post");
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    let post_path = dir.join("custom.json");
    let json = serde_json::to_string_pretty(&value).map_err(|err| err.to_string())?;
    std::fs::write(&post_path, json).map_err(|err| err.to_string())?;

    let job = sample_job(&root)?;
    let out = temp_path("campanion-custom-post.nc");
    run_ok(&["run", &job, "-o", &path_arg(&out)?, "--post", &path_arg(&post_path)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    if !posted.contains("(campanion-custom-post)") || !posted.ends_with("M2\n") {
        return Err(format!("custom post was not applied\n{posted}"));
    }
    Ok(())
}

#[test]
fn unknown_post_id_lists_builtins() -> Result<(), String> {
    let root = workspace_root()?;
    let job = sample_job(&root)?;
    let out = temp_path("campanion-unknown-post.nc");
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["run", &job, "-o", &path_arg(&out)?, "--post", "not-a-post"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(1) {
        return Err(format!("exit {:?} stderr {stderr}", output.status.code()));
    }
    for id in ["not-a-post", "linuxcnc", "grbl"] {
        if !stderr.contains(id) {
            return Err(format!("stderr missing {id}: {stderr}"));
        }
    }
    if out.exists() {
        return Err(format!("unknown post wrote {}", out.display()));
    }

    let ir = committed_ir(&root)?;
    let posted = campanion(&["post", &ir, "--post", "not-a-post"])?;
    let stderr = text(&posted.stderr)?;
    if posted.status.code() != Some(1) || !stderr.contains("linuxcnc") || !stderr.contains("grbl") {
        return Err(format!("post command exit {:?} stderr {stderr}", posted.status.code()));
    }
    Ok(())
}

#[test]
fn posts_lists_every_builtin() -> Result<(), String> {
    let output = run_ok(&["posts"])?;
    let stdout = text(&output.stdout)?;
    let ids: Vec<&str> =
        stdout.lines().filter(|line| !line.starts_with(' ') && !line.is_empty()).filter_map(|line| line.split_whitespace().next()).collect();
    if ids
        != [
            "linuxcnc",
            "linuxcnc_5ax",
            "grbl",
            "mach3",
            "fanuc",
            "haas",
            "haas_umc",
            "plasma",
            "grbl_laser",
            "linuxcnc_lathe",
            "fanuc_lathe",
            "haas_lathe",
            "grbl_lathe",
            "generic_lathe_2ch",
        ]
    {
        return Err(format!("posts ids were {ids:?}\n{stdout}"));
    }
    if !stdout.contains("linuxcnc LinuxCNC\n") || !stdout.contains("cutter compensation cancelled") {
        return Err(format!("linuxcnc listing missing name or description:\n{stdout}"));
    }
    if !stdout.contains("grbl Grbl\n") || !stdout.contains("no line numbers") {
        return Err(format!("grbl listing missing name or description:\n{stdout}"));
    }
    Ok(())
}

#[test]
fn post_on_committed_ir_matches_run() -> Result<(), String> {
    // The post fixtures keep a per-cut process cycle for the golden files. `run` holds a spindle
    // before it writes IR, so this posts the IR that same run just wrote.
    let root = workspace_root()?;
    let job = sample_job(&root)?;
    for id in ["linuxcnc", "grbl"] {
        let from_run_path = temp_path(&format!("campanion-post-match-run-{id}.nc"));
        let ir_path = temp_path(&format!("campanion-post-match-{id}.ir.json"));
        let from_post_path = temp_path(&format!("campanion-post-match-post-{id}.nc"));
        run_ok(&["run", &job, "-o", &path_arg(&from_run_path)?, "--ir", &path_arg(&ir_path)?, "--post", id])?;
        let from_run = std::fs::read_to_string(&from_run_path).map_err(|err| format!("{id}: {err}"))?;
        let ir = path_arg(&ir_path)?;
        let printed = run_ok(&["post", &ir, "--post", id])?;
        let from_post = text(&printed.stdout)?;
        if from_run != from_post {
            return Err(format!("{id}: run --post and post differ: {}", first_diff(&from_run, &from_post)));
        }
        run_ok(&["post", &ir, "--post", id, "-o", &path_arg(&from_post_path)?])?;
        let on_disk = std::fs::read_to_string(&from_post_path).map_err(|err| format!("{id}: {err}"))?;
        if on_disk != from_post {
            return Err(format!("{id}: post -o and stdout differ: {}", first_diff(&on_disk, &from_post)));
        }
    }
    Ok(())
}
