//! Indexed 3+2 of `examples/tilted.job.json`.
//!
//! `tilt30` keeps the tilted tool axis, `side` keeps the side axis, and a clearance rapid sits
//! between those frames. Haas has no tilted-plane output. A plane past the B travel fails at load.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_ir::{ClBlock, ClMotion, Document, Pose, Vec3};

const CLEARANCE_Z: f64 = 5.0;
const TILT_Y: f64 = -0.5;
const TILT_Z: f64 = 0.8660254037844386;

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

fn near_axis(axis: Vec3, x: f64, y: f64, z: f64) -> bool {
    (axis.x() - x).abs() <= 1e-9 && (axis.y() - y).abs() <= 1e-9 && (axis.z() - z).abs() <= 1e-9
}

fn view_z(tip: Vec3, origin: Vec3, axis: Vec3) -> f64 {
    let dx = tip.x() - origin.x();
    let dy = tip.y() - origin.y();
    let dz = tip.z() - origin.z();
    dx * axis.x() + dy * axis.y() + dz * axis.z()
}

#[test]
fn tilted_job_maps_axes_and_retracts_between_frames() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-tilted-e2e.ngc");
    let ir_path = temp_path("campanion-tilted-e2e.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&root, &["run", "examples/tilted.job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
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
    let program = doc.program();
    let tilt_paths: Vec<_> = program.toolpaths().iter().filter(|path| path.name() == Some("tilt30")).collect();
    let side_paths: Vec<_> = program.toolpaths().iter().filter(|path| path.name() == Some("side")).collect();
    if tilt_paths.is_empty() || side_paths.is_empty() {
        return Err("missing tilt30 or side toolpath".into());
    }

    let mut tilt_motion = false;
    for path in &tilt_paths {
        for block in path.blocks() {
            let Some(pose) = motion_pose(block) else { continue };
            tilt_motion = true;
            if !near_axis(pose.axis(), 0.0, TILT_Y, TILT_Z) {
                return Err(format!("tilt30 axis {} {} {}", pose.axis().x(), pose.axis().y(), pose.axis().z()));
            }
            if pose.rotary().is_some() {
                return Err("tilt30 pose carries a rotary angle".into());
            }
        }
    }
    if !tilt_motion {
        return Err("tilt30 has no motion".into());
    }

    let mut side_motion = false;
    for path in &side_paths {
        for block in path.blocks() {
            let Some(pose) = motion_pose(block) else { continue };
            side_motion = true;
            if !near_axis(pose.axis(), 0.0, -1.0, 0.0) {
                return Err(format!("side axis {} {} {}", pose.axis().x(), pose.axis().y(), pose.axis().z()));
            }
        }
    }
    if !side_motion {
        return Err("side has no motion".into());
    }

    let tilt_axis = Vec3::new(0.0, TILT_Y, TILT_Z).map_err(|err| err.to_string())?;
    let side_axis = Vec3::new(0.0, -1.0, 0.0).map_err(|err| err.to_string())?;
    let side_origin = Vec3::new(0.0, -20.0, -5.0).map_err(|err| err.to_string())?;
    let tilt_end = motion_pose(tilt_paths.last().and_then(|path| path.blocks().last()).ok_or("tilt30 empty")?).ok_or("tilt30 ends without a pose")?;
    let side_start =
        motion_pose(side_paths.first().and_then(|path| path.blocks().first()).ok_or("side empty")?).ok_or("side starts without a pose")?;
    if (view_z(tilt_end.tip(), Vec3::ZERO, tilt_axis) - CLEARANCE_Z).abs() > 1e-6 {
        return Err(format!("tilt30 retract view Z is {}", view_z(tilt_end.tip(), Vec3::ZERO, tilt_axis)));
    }
    if (view_z(side_start.tip(), side_origin, side_axis) - CLEARANCE_Z).abs() > 1e-6 {
        return Err(format!("side approach view Z is {}", view_z(side_start.tip(), side_origin, side_axis)));
    }
    if !matches!(tilt_paths.last().and_then(|path| path.blocks().last()), Some(ClBlock::Motion(ClMotion::Rapid { .. }))) {
        return Err("tilt30 does not end on the clearance rapid".into());
    }
    if !matches!(side_paths.first().and_then(|path| path.blocks().first()), Some(ClBlock::Motion(ClMotion::Rapid { .. }))) {
        return Err("side does not start on the clearance rapid".into());
    }

    let haas_ngc = temp_path("campanion-tilted-e2e-haas.ngc");
    let haas_arg = path_arg(&haas_ngc)?;
    let haas = campanion(&root, &["run", "examples/tilted.job.json", "-o", &haas_arg, "--post", "haas"])?;
    let haas_err = output_text(&haas.stderr)?;
    if haas.status.code() == Some(0) {
        return Err(format!("haas posted a tilted program: {haas_err}"));
    }
    if !haas_err.contains("post `haas` has no tilted-plane output") {
        return Err(format!("missing tilted post error: {haas_err}"));
    }

    let folder = temp_path("campanion-tilted-e2e-limit");
    std::fs::create_dir_all(&folder).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/table_table_bc.kin.json"), folder.join("table_table_bc.kin.json")).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/sample.dxf"), folder.join("sample.dxf")).map_err(|err| err.to_string())?;
    let job = folder.join("job.json");
    // B is -35°..120°, so a tool axis needs z >= cos(120°). [0.866, 0, 0.5] is B = +60°
    // (the B = -60° pose with C = 180°) and is inside that travel. This normal is the B = 150° axis.
    let normal = "[-0.5, 0, -0.8660254037844386]";
    let text = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"sample.dxf","hash":"cee9f36f5ba230a2"}},"units":"mm","kinematics":"table_table_bc.kin.json","tools":[{{"id":1,"diameter":6}}],"operations":[{{"name":"past","type":"profile","layers":["OUTER"],"side":"outside","tool_id":1,"depth":3,"step_down":3,"plane":{{"origin":[0,0,0],"normal":{normal}}},"feeds":{{"cut":1000,"plunge":300}}}}]}}"#
    );
    std::fs::write(&job, text).map_err(|err| err.to_string())?;
    let limit_ngc = folder.join("out.ngc");
    let limit = campanion(&folder, &["run", "job.json", "-o", &path_arg(&limit_ngc)?])?;
    let limit_err = output_text(&limit.stderr)?;
    if limit.status.code() == Some(0) {
        return Err(format!("plane past the rotary limit was planned: {limit_err}"));
    }
    if !limit_err.contains("operation `past`: tool axis") || !limit_err.contains("is outside the rotary limits") {
        return Err(format!("missing rotary-limit error: {limit_err}"));
    }
    Ok(())
}
