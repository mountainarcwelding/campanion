//! Continuous tool axis on `examples/dome_lead_tilt.job.json`.
//!
//! A 6 mm ball, lead 10° and tilt 0°, on the table-table kinematics. The toolpath is continuous
//! and the posted files are byte-exact.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_ir::{ClBlock, ClMotion, Document, Pose};

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

fn assert_golden(name: &str, actual: &str) -> Result<(), String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|err| err.to_string())?;
        }
        std::fs::write(&path, actual).map_err(|err| format!("write {}: {err}", path.display()))?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if actual != expected {
        return Err(format!("golden mismatch for {} (set UPDATE_GOLDEN=1 to regenerate)", path.display()));
    }
    Ok(())
}

#[test]
fn dome_lead_tilt_is_one_continuous_path() -> Result<(), String> {
    let root = workspace_root()?;
    let ir_path = temp_path("campanion-dome-lead-tilt.json");
    let ngc_path = temp_path("campanion-dome-lead-tilt.ngc");
    let output = campanion(&root, &["run", "examples/dome_lead_tilt.job.json", "-o", &path_arg(&ngc_path)?, "--ir", &path_arg(&ir_path)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let doc = Document::from_json(&ir_text).map_err(|err| format!("ir: {err}"))?;
    let path = doc.program().toolpaths().iter().find(|path| path.name() == Some("dome")).ok_or("missing dome toolpath")?;
    if !path.continuous() {
        return Err("dome toolpath is not continuous".into());
    }
    let mut tilted = false;
    for block in path.blocks() {
        let Some(pose) = motion_pose(block) else { continue };
        let axis = pose.axis();
        if axis.x().abs() > 1e-6 || axis.y().abs() > 1e-6 {
            tilted = true;
        }
    }
    if !tilted {
        return Err("dome tool axis stayed on +Z".into());
    }
    Ok(())
}

#[test]
fn dome_lead_tilt_posts_five_axis() -> Result<(), String> {
    let root = workspace_root()?;
    let posts = [
        ("linuxcnc_5ax", "dome_lead_tilt.linuxcnc_5ax.nc"),
        ("haas_umc", "dome_lead_tilt.haas_umc.nc"),
        ("linuxcnc_5ax_tcp", "dome_lead_tilt.linuxcnc_5ax_tcp.nc"),
        ("haas_umc_tcpc", "dome_lead_tilt.haas_umc_tcpc.nc"),
    ];
    for (post, golden) in posts {
        let ngc = temp_path(&format!("campanion-dome-lead-{post}.nc"));
        let output = campanion(&root, &["run", "examples/dome_lead_tilt.job.json", "-o", &path_arg(&ngc)?, "--post", post])?;
        let stderr = output_text(&output.stderr)?;
        let stdout = output_text(&output.stdout)?;
        if output.status.code() != Some(0) {
            return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
        }
        let text = std::fs::read_to_string(&ngc).map_err(|err| format!("read {post}: {err}"))?;
        assert_golden(golden, &text)?;
    }
    Ok(())
}
