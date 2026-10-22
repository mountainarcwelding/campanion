//! Rotary finish of `examples/rotary.job.json`.
//!
//! A 3 mm ball zigzag-finishes the revolved vase about A. The run succeeds, so the gouge check
//! passed. The IR validates. Every cutting pose carries a rotary angle and those angles advance
//! continuously. LinuxCNC posts the program and reads it back. Grbl, which has no rotary axis,
//! rejects it.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_ir::{ClBlock, ClMotion, Document, MoveRole, Pose};

/// `angle_step_deg` on the example. A later pass is at most one step ahead.
const ANGLE_STEP_DEG: f64 = 30.0;

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
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

fn motion(block: &ClBlock) -> Option<(Pose, Option<MoveRole>)> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, .. }) => Some((*to, None)),
        ClBlock::Motion(ClMotion::Linear { to, role, .. }) => Some((*to, Some(*role))),
        ClBlock::Motion(ClMotion::Arc { .. } | ClMotion::Helix { .. }) => None,
        ClBlock::Event(_) => None,
    }
}

#[test]
fn rotary_vase_finishes_continuously_and_linuxcnc_reads_back() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-rotary-e2e.ngc");
    let ir_path = temp_path("campanion-rotary-e2e.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&root, &["run", "examples/rotary.job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    if stderr.contains("gouge") {
        return Err(format!("gouge check failed: {stderr}"));
    }

    let verify = campanion(&root, &["verify", &ir_arg])?;
    let verify_out = output_text(&verify.stdout)?;
    let verify_err = output_text(&verify.stderr)?;
    if verify.status.code() != Some(0) || !verify_out.lines().any(|line| line == "status ok") || !verify_out.lines().any(|line| line == "issues 0") {
        return Err(format!("verify exit {:?} stdout {verify_out} stderr {verify_err}", verify.status.code()));
    }

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let doc = Document::from_json(&ir_text).map_err(|err| format!("ir: {err}"))?;
    let program = doc.program();
    let setup = program.rotary().ok_or("program has no rotary setup")?;
    if setup.axis() != campanion_ir::RotaryAxisKind::A {
        return Err("rotary example is not about A".into());
    }
    if program.toolpaths().len() != 1 {
        return Err(format!("expected one toolpath, got {}", program.toolpaths().len()));
    }
    let path = &program.toolpaths()[0];
    if path.name() != Some("vase") {
        return Err(format!("toolpath name {:?}", path.name()));
    }

    let mut cutting = Vec::new();
    for block in path.blocks() {
        if matches!(block, ClBlock::Motion(ClMotion::Arc { .. } | ClMotion::Helix { .. })) {
            return Err("rotary finish emitted an arc".into());
        }
        let Some((pose, role)) = motion(block) else {
            continue;
        };
        if role != Some(MoveRole::Cut) {
            continue;
        }
        let Some(angle) = pose.rotary() else {
            return Err("cutting pose has no rotary angle".into());
        };
        cutting.push(angle);
    }
    let Some(first) = cutting.first().copied() else {
        return Err("rotary finish has no cutting moves".into());
    };
    let Some(last) = cutting.last().copied() else {
        return Err("rotary finish has no cutting moves".into());
    };
    if first.abs() > 1e-6 {
        return Err(format!("first cutting angle {first} is not 0"));
    }
    if (last - 360.0).abs() > 1e-4 {
        return Err(format!("last cutting angle {last} is not 360"));
    }
    for pair in cutting.windows(2) {
        let (Some(before), Some(after)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let gap = after - before;
        if !(-1e-6..=ANGLE_STEP_DEG + 1e-4).contains(&gap) {
            return Err(format!("cutting angles jumped by {gap} from {before} to {after}"));
        }
    }

    let posted_ngc = temp_path("campanion-rotary-e2e-post.ngc");
    let posted_arg = path_arg(&posted_ngc)?;
    let posted = campanion(&root, &["run", "examples/rotary.job.json", "-o", &posted_arg, "--post", "linuxcnc"])?;
    let posted_out = output_text(&posted.stdout)?;
    let posted_err = output_text(&posted.stderr)?;
    if posted.status.code() != Some(0) {
        return Err(format!("linuxcnc did not post the rotary program: stdout {posted_out} stderr {posted_err}"));
    }
    if posted_err.contains("gouge") || posted_err.contains("changed") {
        return Err(format!("linuxcnc post reported a problem: {posted_err}"));
    }
    let ngc = std::fs::read_to_string(&posted_ngc).map_err(|err| format!("read posted ngc: {err}"))?;
    if !ngc.lines().any(|line| line.split_whitespace().any(|word| word.starts_with('A') && word.len() > 1)) {
        return Err(format!("linuxcnc output has no A word:\n{ngc}"));
    }

    let grbl_ngc = temp_path("campanion-rotary-e2e-grbl.ngc");
    let grbl_arg = path_arg(&grbl_ngc)?;
    let grbl = campanion(&root, &["run", "examples/rotary.job.json", "-o", &grbl_arg, "--post", "grbl"])?;
    let grbl_err = output_text(&grbl.stderr)?;
    if grbl.status.code() == Some(0) {
        return Err(format!("grbl posted a rotary program: {grbl_err}"));
    }
    if !grbl_err.contains("post `grbl` does not support a rotary axis") {
        return Err(format!("missing rotary post error: {grbl_err}"));
    }
    if grbl_err.contains("changed") {
        return Err(format!("source hash did not match: {grbl_err}"));
    }
    Ok(())
}
