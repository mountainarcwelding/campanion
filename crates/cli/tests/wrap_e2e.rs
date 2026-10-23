//! Wrap of `examples/wrap.job.json` onto a 40 mm cylinder about A.
//!
//! The IR validates. Every cutting pose carries a rotary angle, the tool axis is perpendicular
//! to the rotary axis, and unwrapping the pose lands on the flat drawing at the pass depth.
//! Rotary steps stay inside the chord-tolerance subdivision. LinuxCNC posts the A axis with
//! inverse time. Grbl, which has no rotary axis, rejects the program.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use campanion_geom::{Point, Segment, Tolerance};
use campanion_import::{ImportOptions, import_svg};
use campanion_ir::{ClBlock, ClMotion, Cylinder, Document, MoveRole, Pose, RotaryAxisKind, Vec3, unwrap_pose};

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

fn motion(block: &ClBlock) -> Option<(Pose, Option<MoveRole>, Option<f64>)> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, feed }) => Some((*to, None, *feed)),
        ClBlock::Motion(ClMotion::Linear { to, feed, role }) => Some((*to, Some(*role), Some(*feed))),
        ClBlock::Motion(ClMotion::Thread { to, .. }) => Some((*to, Some(MoveRole::Cut), None)),
        ClBlock::Motion(ClMotion::Arc { .. } | ClMotion::Helix { .. }) => None,
        ClBlock::Event(_) => None,
    }
}

/// Largest sweep, in degrees, whose sagitta at `radius` is `tol`. The wrapper stays inside a
/// slightly smaller step, so this is a safe upper bound.
fn max_step_deg(radius: f64, tol: f64) -> f64 {
    let ratio = tol / radius;
    let sweep = if !ratio.is_finite() || ratio <= 0.0 {
        0.0
    } else if ratio >= 2.0 {
        std::f64::consts::TAU
    } else if ratio < 1.0e-8 {
        2.0 * (2.0 * ratio).sqrt()
    } else {
        2.0 * (1.0 - ratio).acos()
    };
    sweep.to_degrees()
}

fn dot(left: Vec3, right: Vec3) -> f64 {
    left.x() * right.x() + left.y() * right.y() + left.z() * right.z()
}

#[test]
fn wrap_example_stays_on_the_drawing_and_linuxcnc_posts_inverse_time() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-wrap-e2e.ngc");
    let ir_path = temp_path("campanion-wrap-e2e.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&root, &["run", "examples/wrap.job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
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

    let svg = std::fs::read_to_string(root.join("examples/wrap.svg")).map_err(|err| format!("read svg: {err}"))?;
    let tolerance = Tolerance::with_chord(0.001, 0.001, 0.01).map_err(|err| format!("tolerance: {err}"))?;
    let drawing = import_svg(&svg, &ImportOptions::new(tolerance)).map_err(|err| format!("svg: {err}"))?;
    let mut contours = Vec::new();
    let mut saw_arc = false;
    for item in drawing.contours() {
        if item.layer() != "LETTERS" {
            continue;
        }
        if item.contour().segments().iter().any(|segment| matches!(segment, Segment::Arc(_))) {
            saw_arc = true;
        }
        contours.push(item.contour().clone());
    }
    if contours.is_empty() {
        return Err("svg has no LETTERS contours".into());
    }
    if !saw_arc {
        return Err("svg arc was not imported as an arc".into());
    }

    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let doc = Document::from_json(&ir_text).map_err(|err| format!("ir: {err}"))?;
    let program = doc.program();
    let setup = program.rotary().ok_or("program has no rotary setup")?;
    if setup.axis() != RotaryAxisKind::A {
        return Err("wrap example is not about A".into());
    }
    let cylinder = Cylinder::new(40.0).map_err(|err| format!("cylinder: {err}"))?;
    let axis = setup.axis_direction();
    let chord = 0.01;
    let u_offset = 6.0;
    let v_offset = 0.0;

    if program.toolpaths().len() != 1 {
        return Err(format!("expected one toolpath, got {}", program.toolpaths().len()));
    }
    let path = &program.toolpaths()[0];
    if path.name() != Some("letters") {
        return Err(format!("toolpath name {:?}", path.name()));
    }

    let mut poses = Vec::new();
    let mut saw_shallow = false;
    let mut saw_deep = false;
    let mut saw_cut = false;
    for block in path.blocks() {
        if matches!(block, ClBlock::Motion(ClMotion::Arc { .. } | ClMotion::Helix { .. })) {
            return Err("wrap emitted an arc".into());
        }
        let Some((pose, role, feed)) = motion(block) else {
            continue;
        };
        let Some(angle) = pose.rotary() else {
            return Err("motion has no rotary angle".into());
        };
        if dot(pose.axis(), axis).abs() > 1e-8 {
            return Err(format!("tool axis is not perpendicular to the rotary axis at {angle}"));
        }
        let (u, v, w) = unwrap_pose(setup, cylinder, pose).map_err(|err| format!("unwrap: {err}"))?;
        if let Some(MoveRole::Cut) = role {
            saw_cut = true;
            if (w + 0.25).abs() <= 1e-6 {
                saw_shallow = true;
            } else if (w + 0.5).abs() <= 1e-6 {
                saw_deep = true;
            } else {
                return Err(format!("cut depth w {w} is not -0.25 or -0.5"));
            }
            let point = Point::new(u - u_offset, v - v_offset).map_err(|err| format!("point: {err}"))?;
            let mut nearest = f64::MAX;
            for contour in &contours {
                for segment in contour.segments() {
                    nearest = nearest.min(segment.distance_to_point(point));
                }
            }
            if nearest > chord + 1e-6 {
                return Err(format!("cut at ({}, {}) is {nearest} mm off the drawing", point.x(), point.y()));
            }
            let Some(feed) = feed else {
                return Err("cut has no feed".into());
            };
            if (feed - 800.0).abs() > 1e-9 {
                return Err(format!("cut feed {feed} is not the surface feed 800"));
            }
        }
        if let Some(MoveRole::Plunge) = role {
            let Some(feed) = feed else {
                return Err("plunge has no feed".into());
            };
            if (feed - 200.0).abs() > 1e-9 {
                return Err(format!("plunge feed {feed} is not the surface feed 200"));
            }
        }
        poses.push((angle, w));
    }
    if !saw_cut || !saw_shallow || !saw_deep {
        return Err(format!("missing a depth pass: cut {saw_cut} 0.25 {saw_shallow} 0.5 {saw_deep}"));
    }
    let Some((first_angle, _)) = poses.first() else {
        return Err("wrap has no motion".into());
    };
    if first_angle.abs() > 1e-6 {
        return Err(format!("first angle {first_angle} is not 0"));
    }
    for pair in poses.windows(2) {
        let (Some((a0, w0)), Some((a1, w1))) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let radius = (20.0 + w0).max(20.0 + w1);
        let limit = max_step_deg(radius, chord) + 1e-6;
        let step = (a1 - a0).abs();
        if step > limit {
            return Err(format!("angle step {step} deg exceeds the subdivision limit {limit} at radius {radius}"));
        }
    }

    let posted_ngc = temp_path("campanion-wrap-e2e-post.ngc");
    let posted_arg = path_arg(&posted_ngc)?;
    let posted = campanion(&root, &["run", "examples/wrap.job.json", "-o", &posted_arg, "--post", "linuxcnc"])?;
    let posted_err = output_text(&posted.stderr)?;
    if posted.status.code() != Some(0) {
        return Err(format!("linuxcnc did not post the wrap program: {posted_err}"));
    }
    let ngc = std::fs::read_to_string(&posted_ngc).map_err(|err| format!("read posted ngc: {err}"))?;
    if !ngc.contains("G93") {
        return Err(format!("linuxcnc wrap output has no inverse-time block:\n{ngc}"));
    }
    if !ngc.lines().any(|line| line.split_whitespace().any(|word| word.starts_with('A') && word.len() > 1)) {
        return Err(format!("linuxcnc output has no A word:\n{ngc}"));
    }

    let grbl_ngc = temp_path("campanion-wrap-e2e-grbl.ngc");
    let grbl_arg = path_arg(&grbl_ngc)?;
    let grbl = campanion(&root, &["run", "examples/wrap.job.json", "-o", &grbl_arg, "--post", "grbl"])?;
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
