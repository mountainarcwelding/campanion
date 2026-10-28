//! Wrap selected contours onto a cylinder about the job rotary axis.
//!
//! Drawing X becomes flat-frame `u` (plus `u_offset`). Drawing Y becomes `v` (plus `v_offset`).
//! `w` is 0 on the cylinder surface and negative into the material. Each contour is cut once
//! per depth pass: rapid at the safe radius, plunge to `w = -level`, follow the contour, rapid
//! back. Feeds stay the tool's surface feeds. The path is already in the part frame.

use campanion_geom::{Arc, Contour, Point, Segment};
use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Cylinder, Event, MoveRole, Pose, ProcessKind, RotarySetup, wrap_polyline};
use campanion_path::{Tool as PathTool, pass_depths};

use crate::error::CliError;
use crate::jobfile::{JobTool, Operation};

/// Samples used when a drawing arc is flattened to the job chord tolerance.
const MAX_ARC_STEPS: usize = 8_192;

/// Build one part-frame toolpath for a wrap operation.
///
/// `safe_z` is the index safe height from [`crate::index::index_safe_z`]. Rapids use
/// `w = safe_z - diameter / 2`, so the tip is `safe_z` from the rotary axis. When the axis
/// centre is at Z = 0, that distance is also the machine Z.
pub(crate) fn cut(
    paths: &mut Vec<ClPath>,
    op: &Operation,
    tool: &JobTool,
    contours: &[Contour],
    setup: &RotarySetup,
    chord_mm: f64,
    safe_z: f64,
) -> Result<(), CliError> {
    if !(chord_mm.is_finite() && chord_mm > 0.0) {
        return Err(op_err(op, "wrap tolerance must be positive"));
    }
    let Some(diameter) = op.cylinder_diameter() else {
        return Err(op_err(op, "wrap needs a diameter"));
    };
    let cylinder = Cylinder::new(diameter).map_err(|err| op_err(op, err))?;
    let w_safe = safe_z - diameter * 0.5;
    if !(w_safe.is_finite() && diameter * 0.5 + w_safe > 0.0) {
        return Err(op_err(op, "wrap radius must be positive"));
    }
    let levels = pass_depths(op.depth(), op.step_down()).map_err(|err| op_err(op, err))?;
    if levels.is_empty() {
        return Err(op_err(op, "depth plan is empty"));
    }
    let process = match tool.path_tool() {
        PathTool::Diameter { .. } => ProcessKind::Spindle,
        PathTool::Kerf { .. } => ProcessKind::Torch,
    };
    let params = op.params_with_speed(tool).map_err(|err| op_err(op, err))?;
    let feeds = op.feeds();
    let plunge_feed = feeds.resolve(MoveRole::Plunge);
    let cut_feed = feeds.cut();
    let u_offset = op.u_offset();
    let v_offset = op.v_offset();

    let mut blocks = Vec::new();
    let mut at: Option<(f64, f64, f64)> = None;
    let mut spinning = false;
    for contour in contours {
        let drawing = flatten_contour(op, contour, chord_mm)?;
        let mut uv = Vec::with_capacity(drawing.len());
        for point in drawing {
            uv.push((point.x() + u_offset, point.y() + v_offset));
        }
        for level in &levels {
            let w_cut = -level;
            let Some((u0, v0)) = uv.first().copied() else {
                return Err(op_err(op, "wrap contour is empty"));
            };
            travel(&mut blocks, setup, cylinder, chord_mm, &mut at, &[(u0, v0, w_safe)], Travel::Rapid).map_err(|err| op_err(op, err))?;
            if !spinning {
                blocks.push(ClBlock::Event(Event::ProcessOn { process: process.clone(), params: params.clone() }));
                spinning = true;
            }
            travel(&mut blocks, setup, cylinder, chord_mm, &mut at, &[(u0, v0, w_cut)], Travel::Feed { feed: plunge_feed, role: MoveRole::Plunge })
                .map_err(|err| op_err(op, err))?;
            let mut cut_pts = Vec::with_capacity(uv.len());
            for (u, v) in &uv {
                cut_pts.push((*u, *v, w_cut));
            }
            travel(&mut blocks, setup, cylinder, chord_mm, &mut at, &cut_pts, Travel::Feed { feed: cut_feed, role: MoveRole::Cut })
                .map_err(|err| op_err(op, err))?;
            let Some((ue, ve, _)) = cut_pts.last().copied() else {
                return Err(op_err(op, "wrap contour is empty"));
            };
            travel(&mut blocks, setup, cylinder, chord_mm, &mut at, &[(ue, ve, w_safe)], Travel::Rapid).map_err(|err| op_err(op, err))?;
        }
    }
    if !spinning {
        return Err(op_err(op, "wrap contour is empty"));
    }
    blocks.push(ClBlock::Event(Event::ProcessOff { process }));
    let path = ClPath::try_new(Some(op.name().to_string()), Some(op.tool_id()), CompensationMode::Computed, feeds, None, Vec::new(), blocks)
        .map_err(|err| op_err(op, err))?;
    paths.push(path);
    Ok(())
}

enum Travel {
    Rapid,
    Feed { feed: f64, role: MoveRole },
}

fn travel(
    blocks: &mut Vec<ClBlock>,
    setup: &RotarySetup,
    cylinder: Cylinder,
    tol: f64,
    at: &mut Option<(f64, f64, f64)>,
    dest: &[(f64, f64, f64)],
    kind: Travel,
) -> Result<(), campanion_ir::IrError> {
    let mut pts = Vec::with_capacity(dest.len().saturating_add(1));
    if let Some(here) = *at {
        pts.push(here);
    }
    pts.extend_from_slice(dest);
    if pts.is_empty() {
        return Ok(());
    }
    let poses = wrap_polyline(setup, cylinder, &pts, tol)?;
    let skip = usize::from(at.is_some());
    for pose in poses.into_iter().skip(skip) {
        blocks.push(motion(pose, &kind));
    }
    if let Some(last) = pts.last().copied() {
        *at = Some(last);
    }
    Ok(())
}

fn motion(pose: Pose, kind: &Travel) -> ClBlock {
    match *kind {
        Travel::Rapid => ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None }),
        Travel::Feed { feed, role } => ClBlock::Motion(ClMotion::Linear { to: pose, feed, role }),
    }
}

fn flatten_contour(op: &Operation, contour: &Contour, tol: f64) -> Result<Vec<Point>, CliError> {
    let mut points = Vec::new();
    for segment in contour.segments() {
        match *segment {
            Segment::Line(line) => {
                push_point(&mut points, line.start());
                push_point(&mut points, line.end());
            }
            Segment::Arc(arc) => {
                for point in arc_points(op, arc, tol)? {
                    push_point(&mut points, point);
                }
            }
        }
    }
    if contour.is_closed()
        && let Some(first) = points.first().copied()
    {
        push_point(&mut points, first);
    }
    if points.len() < 2 {
        return Err(op_err(op, "wrap contour is empty"));
    }
    Ok(points)
}

fn push_point(points: &mut Vec<Point>, point: Point) {
    if let Some(last) = points.last()
        && last.x() == point.x()
        && last.y() == point.y()
    {
        return;
    }
    points.push(point);
}

fn arc_points(op: &Operation, arc: Arc, tol: f64) -> Result<Vec<Point>, CliError> {
    let sweep = arc.sweep().abs();
    let step = angular_step(arc.radius(), tol);
    if !step.is_finite() || step <= 0.0 || !sweep.is_finite() || sweep == 0.0 {
        return Err(op_err(op, "wrap arc spacing is not usable"));
    }
    let count = (sweep / step).ceil();
    if !count.is_finite() || count < 1.0 || count > MAX_ARC_STEPS as f64 {
        return Err(op_err(op, format!("too many wrap arc samples: limit is {MAX_ARC_STEPS}")));
    }
    let steps = count as usize;
    let mut points = Vec::with_capacity(steps.saturating_add(1));
    points.push(arc.point_at(0.0));
    for index in 1..steps {
        let t = index as f64 / steps as f64;
        points.push(arc.point_at(t));
    }
    points.push(arc.point_at(1.0));
    Ok(points)
}

/// Largest sweep whose sagitta at `radius` stays within `tol`. Matches the index-arc step.
fn angular_step(radius: f64, tol: f64) -> f64 {
    if !(radius.is_finite() && radius > 0.0) || tol >= radius {
        return std::f64::consts::FRAC_PI_2;
    }
    let step = 2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos();
    if step.is_finite() && step > 1e-6 { step } else { 1e-3 }
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}
