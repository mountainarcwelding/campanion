//! Indexed finish of `examples/indexed.job.json`.
//!
//! Two surface operations on `examples/dome.stl`, at index 0° and 90° about A. The IR validates,
//! every pose carries its operation's index, the index-change tip sits outside the swept radius,
//! the gouge check passes. LinuxCNC posts the A axis. Grbl, which has no rotary axis, rejects it.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_ir::{ClBlock, ClMotion, Document, Pose, Vec3, part_to_machine};
use campanion_mesh::parse_stl;

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

fn motion_pose(block: &ClBlock) -> Option<Pose> {
    match block {
        ClBlock::Motion(
            ClMotion::Rapid { to, .. }
            | ClMotion::Linear { to, .. }
            | ClMotion::Arc { to, .. }
            | ClMotion::Helix { to, .. }
            | ClMotion::Thread { to, .. },
        ) => Some(*to),
        ClBlock::Event(_) => None,
    }
}

fn dist(left: Vec3, right: Vec3) -> f64 {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Distance from a point to the A axis through the origin.
fn distance_to_a(point: Vec3) -> f64 {
    point.y().hypot(point.z())
}

#[test]
fn indexed_dome_keeps_part_frame_poses_and_linuxcnc_posts_the_a_axis() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-indexed-e2e.ngc");
    let ir_path = temp_path("campanion-indexed-e2e.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&root, &["run", "examples/indexed.job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
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
    let face0 = program.toolpaths().iter().find(|path| path.name() == Some("face0")).ok_or("missing face0")?;
    let face90 = program.toolpaths().iter().find(|path| path.name() == Some("face90")).ok_or("missing face90")?;

    for (path, angle) in [(face0, 0.0), (face90, 90.0)] {
        let mut saw = false;
        for block in path.blocks() {
            let Some(pose) = motion_pose(block) else {
                continue;
            };
            saw = true;
            if pose.rotary() != Some(angle) {
                return Err(format!("{} pose rotary {:?} is not {angle}", path.name().unwrap_or("path"), pose.rotary()));
            }
        }
        if !saw {
            return Err(format!("{} has no motion", path.name().unwrap_or("path")));
        }
    }

    let retract = motion_pose(face0.blocks().last().ok_or("face0 empty")?).ok_or("face0 ends without a pose")?;
    let indexed = motion_pose(face90.blocks().first().ok_or("face90 empty")?).ok_or("face90 starts without a pose")?;
    let machine_retract = part_to_machine(setup, retract, 0.0).tip();
    let machine_indexed = part_to_machine(setup, indexed, 90.0).tip();
    let gap = dist(machine_retract, machine_indexed);
    if gap > 1e-6 {
        return Err(format!("index change does not hold the machine tip: gap {gap}"));
    }

    let stl = std::fs::read(root.join("examples/dome.stl")).map_err(|err| format!("read stl: {err}"))?;
    let mesh = parse_stl(&stl).map_err(|err| format!("stl: {err}"))?;
    let mut swept = 0.0_f64;
    for triangle in mesh.triangles() {
        for point in [triangle.a, triangle.b, triangle.c] {
            let distance = point.y.hypot(point.z);
            if distance > swept {
                swept = distance;
            }
        }
    }
    let clearance = distance_to_a(machine_retract);
    if !(clearance > swept) {
        return Err(format!("index-change tip is {clearance} mm from the axis, swept radius is {swept}"));
    }

    let posted_ngc = temp_path("campanion-indexed-e2e-post.ngc");
    let posted_arg = path_arg(&posted_ngc)?;
    let posted = campanion(&root, &["run", "examples/indexed.job.json", "-o", &posted_arg, "--post", "linuxcnc"])?;
    let posted_err = output_text(&posted.stderr)?;
    if posted.status.code() != Some(0) {
        return Err(format!("linuxcnc did not post the indexed program: {posted_err}"));
    }
    let ngc = std::fs::read_to_string(&posted_ngc).map_err(|err| format!("read posted ngc: {err}"))?;
    if !ngc.lines().any(|line| line.split_whitespace().any(|word| word.starts_with('A') && word.len() > 1)) {
        return Err(format!("linuxcnc output has no A word:\n{ngc}"));
    }

    let grbl_ngc = temp_path("campanion-indexed-e2e-grbl.ngc");
    let grbl_arg = path_arg(&grbl_ngc)?;
    let grbl = campanion(&root, &["run", "examples/indexed.job.json", "-o", &grbl_arg, "--post", "grbl"])?;
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
