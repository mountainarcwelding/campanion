//! Mesh gouge check for cutter-location toolpaths.
//!
//! Straight cuts are sampled directly. An arc or helix in the XY plane is sampled along the
//! circle, with Z interpolated from the start to the end. An arc in another plane is rejected:
//! surface operations emit lines, roughing emits XY arcs, and only the XY case has a defined sample.
//!
//! A program with a rotary setup is checked in the indexed view. The mesh is turned by the same
//! angle the operation was planned at, and each pose is mapped back to that view before sampling.
//! An arc stored on a cardinal plane is turned back into machine XY. A 3-axis program is unchanged.
//!
//! A rotary finish changes angle along the path. Each cutting segment at one angle is sampled on
//! the mesh turned by that angle, in the machine view. A segment that changes angle is sampled at
//! intermediate angles, each on the mesh turned by that angle.

use std::borrow::Cow;

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, MoveRole, Plane, Pose, Program, RotarySetup, Vec3 as IrVec, part_to_machine};
use campanion_mesh::{Cutter, DropCutter, Mesh, Vec3 as MeshVec, check_gouges};

use crate::error::CliError;
use crate::index;
use crate::jobfile::{JobFile, JobTool, Operation, OperationKind};
use crate::report::GougeNote;
use crate::rough;
use crate::surface;
use crate::waterline;

/// Deepest gouges printed for one toolpath. The check still fails when there are more.
const MAX_REPORTS: usize = 32;

/// Segments materialised from one toolpath. Each one is at least one gouge sample.
const MAX_SEGMENTS: usize = 50_000_000;

const TAU: f64 = std::f64::consts::TAU;
const FULL_TURN_CHORD_MM: f64 = 1.0e-12;

/// `min(0.25, tolerance * 10)` millimetres. `tolerance` is the job chord tolerance.
pub(crate) fn sample_step_for(tolerance: f64) -> Result<f64, CliError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(CliError::invalid("gouge tolerance must be finite and non-negative"));
    }
    let step = (tolerance * 10.0).min(0.25);
    if step.is_finite() && step > 0.0 { Ok(step) } else { Err(CliError::invalid("gouge sample step must be finite and positive")) }
}

/// `ball:R`, `flat:R`, or `bull:R,C`. The numbers are radii in millimetres.
pub(crate) fn parse_tool(text: &str) -> Result<Cutter, CliError> {
    let Some((kind, rest)) = text.split_once(':') else {
        return Err(CliError::usage("tool must be ball:R, flat:R or bull:R,C"));
    };
    match kind {
        "flat" => Cutter::new_flat(parse_positive(rest, "flat radius")?).map_err(|err| CliError::usage(err.to_string())),
        "ball" => Cutter::new_ball(parse_positive(rest, "ball radius")?).map_err(|err| CliError::usage(err.to_string())),
        "bull" => {
            let Some((radius_text, corner_text)) = rest.split_once(',') else {
                return Err(CliError::usage("bull tool must be bull:R,C"));
            };
            if corner_text.contains(',') {
                return Err(CliError::usage("bull tool must be bull:R,C"));
            }
            let radius = parse_positive(radius_text, "bull radius")?;
            let corner = parse_positive(corner_text, "bull corner radius")?;
            Cutter::new_bull(radius, corner).map_err(|err| CliError::usage(err.to_string()))
        }
        _ => Err(CliError::usage("tool must be ball:R, flat:R or bull:R,C")),
    }
}

/// Check every cutting move of an STL job. Each operation uses its own flat, ball or bull cutter.
///
/// The reference is the drop-cutter tip on the part. `stock_to_leave` is not added. A surface
/// or waterline path already includes its axial stock in Z. A rough path sits above the surface
/// plus its stock, so a gouge is a tip below the part, not below the stock allowance.
///
/// A rotary finish is reported per operation, like a surface. Constant-angle cuts use the mesh
/// turned by that angle. Angle changes are sampled between the endpoints.
pub(crate) fn check_job(job: &JobFile, mesh: &Mesh, program: &Program) -> Result<Vec<GougeNote>, CliError> {
    let tolerance = job.tolerance().chord();
    let sample_step = sample_step_for(tolerance)?;
    let paths = program.toolpaths();
    let setup = job.rotary();
    let mut notes = Vec::new();
    for op in job.operations() {
        let Some(path) = paths.iter().find(|path| path.name() == Some(op.name())) else {
            return Err(CliError::run(format!("gouge check: missing toolpath for operation {}", op.name())));
        };
        let Some(tool) = job.tool(op.tool_id()) else {
            return Err(CliError::run(format!("operation {}: unknown tool {}", op.name(), op.tool_id())));
        };
        let angle = op.index_deg().unwrap_or(0.0);
        notes.extend(check_one(op.name(), path, mesh, tool, sample_step, tolerance, op, setup.as_ref(), angle)?);
    }
    Ok(notes)
}

/// Check every toolpath in `program` with one cutter. Used by `verify --mesh`.
pub(crate) fn check_program(program: &Program, mesh: &Mesh, cutter: Cutter, tolerance: f64) -> Result<Vec<GougeNote>, CliError> {
    let sample_step = sample_step_for(tolerance)?;
    if let Some(setup) = program.rotary() {
        let mut notes = Vec::new();
        for (index, path) in program.toolpaths().iter().enumerate() {
            let name = path_label(path, index);
            let angle = path_angle(path);
            let view = indexed_mesh(mesh, Some(setup), angle)?;
            let dc = DropCutter::new(view.as_ref(), cutter).map_err(|err| CliError::run(format!("mesh: {err}")))?;
            let segments = cutting_segments(path, sample_step, Some(setup))?;
            notes.extend(notes_for(&name, &dc, &segments, sample_step, tolerance)?);
        }
        return Ok(notes);
    }
    let dc = DropCutter::new(mesh, cutter).map_err(|err| CliError::run(format!("mesh: {err}")))?;
    let mut notes = Vec::new();
    for (index, path) in program.toolpaths().iter().enumerate() {
        let name = path_label(path, index);
        let segments = cutting_segments(path, sample_step, None)?;
        notes.extend(notes_for(&name, &dc, &segments, sample_step, tolerance)?);
    }
    Ok(notes)
}

fn path_label(path: &ClPath, index: usize) -> String {
    match path.name() {
        Some(name) if !name.is_empty() => name.to_string(),
        _ => format!("toolpath {index}"),
    }
}

fn check_one(
    name: &str,
    path: &ClPath,
    mesh: &Mesh,
    tool: &JobTool,
    sample_step: f64,
    tolerance: f64,
    op: &Operation,
    setup: Option<&RotarySetup>,
    angle: f64,
) -> Result<Vec<GougeNote>, CliError> {
    let cutter = cutter_for(op, tool)?;
    if matches!(op.kind(), OperationKind::RotaryFinish) {
        let Some(setup) = setup else {
            return Err(CliError::run(format!("operation {name}: rotary finish needs a rotary setup")));
        };
        return check_rotary(name, path, mesh, cutter, sample_step, tolerance, setup);
    }
    let view = indexed_mesh(mesh, setup, angle)?;
    let dc = DropCutter::new(view.as_ref(), cutter).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
    let segments = cutting_segments(path, sample_step, setup)?;
    notes_for(name, &dc, &segments, sample_step, tolerance)
}

/// One angle's cutting segments, already in the machine view.
struct AngleCuts {
    angle: f64,
    segments: Vec<(MeshVec, MeshVec)>,
}

/// Constant-angle cuts share one turned mesh. A cut that changes angle is sampled between the ends.
fn check_rotary(
    name: &str,
    path: &ClPath,
    mesh: &Mesh,
    cutter: Cutter,
    sample_step: f64,
    tolerance: f64,
    setup: &RotarySetup,
) -> Result<Vec<GougeNote>, CliError> {
    let mut groups: Vec<AngleCuts> = Vec::new();
    let mut rotating: Vec<(Pose, Pose)> = Vec::new();
    let mut cursor: Option<Pose> = None;
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        match motion {
            ClMotion::Rapid { to, .. } => cursor = Some(*to),
            ClMotion::Linear { to, role, .. } => {
                if cutting(*role) {
                    let start = cursor.unwrap_or(*to);
                    push_rotary_cut(name, &mut groups, &mut rotating, setup, start, *to)?;
                }
                cursor = Some(*to);
            }
            ClMotion::Arc { role, .. } | ClMotion::Helix { role, .. } => {
                if cutting(*role) {
                    return Err(CliError::run(format!("operation {name}: gouge check cannot sample a rotary arc; rotary finish emits lines")));
                }
                cursor = Some(motion_pose(motion));
            }
        }
    }
    let mut notes = Vec::new();
    for group in &groups {
        notes.extend(check_at_angle(name, mesh, setup, cutter, group.angle, &group.segments, sample_step, tolerance)?);
    }
    for (start, end) in rotating {
        notes.extend(check_rotating(name, mesh, setup, cutter, start, end, sample_step, tolerance)?);
    }
    notes.sort_by(|left, right| right.depth.total_cmp(&left.depth));
    notes.truncate(MAX_REPORTS);
    Ok(notes)
}

fn push_rotary_cut(
    name: &str,
    groups: &mut Vec<AngleCuts>,
    rotating: &mut Vec<(Pose, Pose)>,
    setup: &RotarySetup,
    start: Pose,
    end: Pose,
) -> Result<(), CliError> {
    let start_angle = rotary_angle(name, start)?;
    let end_angle = rotary_angle(name, end)?;
    if index::angles_differ(start_angle, end_angle) {
        rotating.push((start, end));
        return Ok(());
    }
    let from = mesh_vec(part_to_machine(setup, start, end_angle).tip());
    let to = mesh_vec(part_to_machine(setup, end, end_angle).tip());
    if let Some(group) = groups.last_mut()
        && !index::angles_differ(group.angle, end_angle)
    {
        return push_segment(&mut group.segments, from, to);
    }
    let mut segments = Vec::new();
    push_segment(&mut segments, from, to)?;
    groups.push(AngleCuts { angle: end_angle, segments });
    Ok(())
}

fn check_rotating(
    name: &str,
    mesh: &Mesh,
    setup: &RotarySetup,
    cutter: Cutter,
    start: Pose,
    end: Pose,
    sample_step: f64,
    tolerance: f64,
) -> Result<Vec<GougeNote>, CliError> {
    let start_angle = rotary_angle(name, start)?;
    let end_angle = rotary_angle(name, end)?;
    let start_tip = part_to_machine(setup, start, start_angle).tip();
    let end_tip = part_to_machine(setup, end, end_angle).tip();
    let tip_radius = index::distance_to_axis(setup, start_tip).max(index::distance_to_axis(setup, end_tip));
    // A tool parked on the axis still sweeps stock out at the cutter radius.
    let radius = tip_radius.max(cutter.radius().unwrap_or(0.0));
    let span = (end_angle - start_angle).abs();
    let linear = tip_distance(start_tip, end_tip);
    let count = spin_count(name, span, radius, linear, sample_step)?;
    let mut notes = Vec::new();
    let steps = (count - 1) as f64;
    for index in 0..count {
        let t = if count == 1 { 0.0 } else { index as f64 / steps };
        let angle = start_angle + (end_angle - start_angle) * t;
        let tip = lerp_tip(start_tip, end_tip, t).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
        let point = mesh_vec(tip);
        notes.extend(check_at_angle(name, mesh, setup, cutter, angle, &[(point, point)], sample_step, tolerance)?);
    }
    Ok(notes)
}

fn check_at_angle(
    name: &str,
    mesh: &Mesh,
    setup: &RotarySetup,
    cutter: Cutter,
    angle: f64,
    segments: &[(MeshVec, MeshVec)],
    sample_step: f64,
    tolerance: f64,
) -> Result<Vec<GougeNote>, CliError> {
    if segments.is_empty() {
        return Ok(Vec::new());
    }
    let turned = index::rotate_mesh(mesh, setup, angle)?;
    let dc = DropCutter::new(&turned, cutter).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
    notes_for(name, &dc, segments, sample_step, tolerance)
}

fn rotary_angle(name: &str, pose: Pose) -> Result<f64, CliError> {
    match pose.rotary() {
        Some(angle) if angle.is_finite() => Ok(angle),
        _ => Err(CliError::run(format!("operation {name}: rotary finish move has no angle"))),
    }
}

/// Samples along a rotating cut, including both ends. Spacing follows the gouge sample step.
fn spin_count(name: &str, span_deg: f64, radius: f64, linear: f64, sample_step: f64) -> Result<usize, CliError> {
    let arc = radius.max(0.0) * span_deg.to_radians();
    if !arc.is_finite() {
        return Err(CliError::run(format!("operation {name}: rotary arc length is not finite")));
    }
    let length = arc.hypot(linear);
    if !length.is_finite() {
        return Err(CliError::run(format!("operation {name}: rotary sample length is not finite")));
    }
    let count = spaced_points(length, sample_step)?;
    if span_deg > 1e-9 { Ok(count.max(2)) } else { Ok(count.max(1)) }
}

fn tip_distance(start: IrVec, end: IrVec) -> f64 {
    let dx = end.x() - start.x();
    let dy = end.y() - start.y();
    let dz = end.z() - start.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn lerp_tip(start: IrVec, end: IrVec, t: f64) -> Result<IrVec, impl std::fmt::Display> {
    IrVec::new(start.x() + (end.x() - start.x()) * t, start.y() + (end.y() - start.y()) * t, start.z() + (end.z() - start.z()) * t)
}

/// Part mesh, or that mesh turned into the indexed view when `angle` is not zero.
fn indexed_mesh<'a>(mesh: &'a Mesh, setup: Option<&RotarySetup>, angle: f64) -> Result<Cow<'a, Mesh>, CliError> {
    let Some(setup) = setup else {
        return Ok(Cow::Borrowed(mesh));
    };
    if !index::angles_differ(angle, 0.0) {
        return Ok(Cow::Borrowed(mesh));
    }
    Ok(Cow::Owned(index::rotate_mesh(mesh, setup, angle)?))
}

/// Flat, ball or bull nose taken from the job tool. Roughing and waterline use the same shapes.
fn cutter_for(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    match op.kind() {
        OperationKind::Surface => surface::cutter(op, tool),
        OperationKind::Rough => rough::cutter(op, tool),
        OperationKind::Waterline => waterline::cutter(op, tool),
        OperationKind::RotaryFinish => crate::rotary::cutter(op, tool),
        _ => Err(CliError::run(format!("operation {}: gouge check needs a surface, rough, waterline or rotary finish operation", op.name()))),
    }
}

fn notes_for(name: &str, dc: &DropCutter, segments: &[(MeshVec, MeshVec)], sample_step: f64, tolerance: f64) -> Result<Vec<GougeNote>, CliError> {
    let (count, gouges) =
        check_gouges(dc, segments, sample_step, tolerance, MAX_REPORTS).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
    if count == 0 {
        return Ok(Vec::new());
    }
    Ok(gouges
        .iter()
        .map(|gouge| GougeNote { operation: name.to_string(), x: gouge.at.x, y: gouge.at.y, z: gouge.at.z, depth: gouge.depth })
        .collect())
}

fn parse_positive(text: &str, what: &str) -> Result<f64, CliError> {
    let Ok(value) = text.parse::<f64>() else {
        return Err(CliError::usage(format!("{what} must be a number")));
    };
    if value.is_finite() && value > 0.0 { Ok(value) } else { Err(CliError::usage(format!("{what} must be finite and positive"))) }
}

fn cutting(role: MoveRole) -> bool {
    matches!(role, MoveRole::Cut | MoveRole::LeadIn | MoveRole::LeadOut | MoveRole::Plunge)
}

fn mesh_vec(point: IrVec) -> MeshVec {
    MeshVec::new(point.x(), point.y(), point.z())
}

fn cutting_segments(path: &ClPath, sample_step: f64, setup: Option<&RotarySetup>) -> Result<Vec<(MeshVec, MeshVec)>, CliError> {
    let mut cursor: Option<Pose> = None;
    let mut segments = Vec::new();
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        match motion {
            ClMotion::Rapid { to, .. } => cursor = Some(*to),
            ClMotion::Linear { to, role, .. } => {
                if cutting(*role) {
                    let start = cursor.unwrap_or(*to);
                    push_segment(&mut segments, mesh_vec(machine_tip(setup, start)), mesh_vec(machine_tip(setup, *to)))?;
                }
                cursor = Some(*to);
            }
            ClMotion::Arc { to, plane, center, sense, role, .. } => {
                if cutting(*role) {
                    let Some(start) = cursor else {
                        return Err(CliError::run("gouge check: arc needs a start position"));
                    };
                    append_curve(&mut segments, setup, start, *to, *center, *plane, *sense, None, sample_step)?;
                }
                cursor = Some(*to);
            }
            ClMotion::Helix { to, plane, center, sense, sweep, role, .. } => {
                if cutting(*role) {
                    let Some(start) = cursor else {
                        return Err(CliError::run("gouge check: arc needs a start position"));
                    };
                    append_curve(&mut segments, setup, start, *to, *center, *plane, *sense, Some(*sweep), sample_step)?;
                }
                cursor = Some(*to);
            }
        }
    }
    Ok(segments)
}

/// Tip in the frame the drop cutter uses. A rotary pose is turned back into the indexed view.
fn machine_tip(setup: Option<&RotarySetup>, pose: Pose) -> IrVec {
    match setup {
        Some(setup) => part_to_machine(setup, pose, pose.rotary().unwrap_or(0.0)).tip(),
        None => pose.tip(),
    }
}

fn append_curve(
    segments: &mut Vec<(MeshVec, MeshVec)>,
    setup: Option<&RotarySetup>,
    start: Pose,
    end: Pose,
    center: IrVec,
    plane: Plane,
    sense: ArcSense,
    sweep: Option<f64>,
    sample_step: f64,
) -> Result<(), CliError> {
    let Some(setup) = setup else {
        return append_arc(segments, start.tip(), end.tip(), center, plane, sense, sweep, sample_step);
    };
    let angle = end.rotary().unwrap_or(0.0);
    let view = index::arc_to_machine(setup, angle, start.tip(), end, center, plane, sense)?;
    if view.plane != Plane::Xy {
        return Err(non_xy(view.plane));
    }
    append_arc(segments, view.start, view.end, view.center, view.plane, view.sense, sweep, sample_step)
}

fn path_angle(path: &ClPath) -> f64 {
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        if let Some(angle) = motion_pose(motion).rotary() {
            return angle;
        }
    }
    0.0
}

fn motion_pose(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. } | ClMotion::Arc { to, .. } | ClMotion::Helix { to, .. } => *to,
    }
}

fn push_segment(segments: &mut Vec<(MeshVec, MeshVec)>, start: MeshVec, end: MeshVec) -> Result<(), CliError> {
    if segments.len() >= MAX_SEGMENTS {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    segments.push((start, end));
    Ok(())
}

fn non_xy(plane: Plane) -> CliError {
    CliError::run(format!("gouge check cannot sample a {} arc; only XY arcs are sampled, and surface operations emit lines", plane_name(plane)))
}

fn plane_name(plane: Plane) -> &'static str {
    match plane {
        Plane::Xy => "XY",
        Plane::Zx => "ZX",
        Plane::Yz => "YZ",
    }
}

/// Sample an XY arc or helix. Z moves linearly from `start` to `end`.
fn append_arc(
    segments: &mut Vec<(MeshVec, MeshVec)>,
    start: IrVec,
    end: IrVec,
    center: IrVec,
    plane: Plane,
    sense: ArcSense,
    sweep: Option<f64>,
    sample_step: f64,
) -> Result<(), CliError> {
    if plane != Plane::Xy {
        return Err(non_xy(plane));
    }
    let (sx, sy, sz) = (start.x(), start.y(), start.z());
    let (ex, ey, ez) = (end.x(), end.y(), end.z());
    let (cx, cy) = (center.x(), center.y());
    for value in [sx, sy, sz, ex, ey, ez, cx, cy, center.z()] {
        if !value.is_finite() {
            return Err(CliError::invalid("arc coordinate is not finite"));
        }
    }
    let radius = (sx - cx).hypot(sy - cy);
    let end_radius = (ex - cx).hypot(ey - cy);
    if !radius.is_finite() || !end_radius.is_finite() || radius <= 1e-9 {
        return Err(CliError::run("gouge check: arc radius is not usable"));
    }
    if (radius - end_radius).abs() > 1.0e-3_f64.max(radius * 1.0e-4) {
        return Err(CliError::run("gouge check: arc endpoints are not on one circle"));
    }
    let ccw = sense == ArcSense::Ccw;
    let sweep = match sweep {
        Some(sweep) if sweep.is_finite() && sweep > 0.0 => sweep,
        Some(_) => return Err(CliError::run("gouge check: arc sweep is not usable")),
        None => xy_sweep(sx, sy, ex, ey, cx, cy, ccw)?,
    };
    let length = (radius * sweep).hypot(ez - sz);
    let count = spaced_points(length, sample_step)?;
    let a0 = (sy - cy).atan2(sx - cx);
    if !a0.is_finite() {
        return Err(CliError::run("gouge check: arc angle is not finite"));
    }
    let signed = if ccw { sweep } else { -sweep };
    let mut previous = None;
    for index in 0..count {
        let point = if index == 0 {
            MeshVec::new(sx, sy, sz)
        } else if index + 1 == count {
            MeshVec::new(ex, ey, ez)
        } else {
            let t = index as f64 / (count - 1) as f64;
            let angle = a0 + signed * t;
            let x = cx + radius * angle.cos();
            let y = cy + radius * angle.sin();
            let z = sz + (ez - sz) * t;
            if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                return Err(CliError::run("gouge check: arc sample is not finite"));
            }
            MeshVec::new(x, y, z)
        };
        if let Some(prev) = previous {
            push_segment(segments, prev, point)?;
        }
        previous = Some(point);
    }
    if count == 1
        && let Some(point) = previous
    {
        push_segment(segments, point, point)?;
    }
    Ok(())
}

fn xy_sweep(sx: f64, sy: f64, ex: f64, ey: f64, cx: f64, cy: f64, ccw: bool) -> Result<f64, CliError> {
    let a0 = (sy - cy).atan2(sx - cx);
    let a1 = (ey - cy).atan2(ex - cx);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(CliError::run("gouge check: arc angle is not finite"));
    }
    let chord = (ex - sx).hypot(ey - sy);
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= FULL_TURN_CHORD_MM {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(CliError::run("gouge check: arc sweep is not usable")) }
}

fn spaced_points(length: f64, step: f64) -> Result<usize, CliError> {
    if !length.is_finite() || length < 0.0 {
        return Err(CliError::run("gouge check: arc length is not finite"));
    }
    if length == 0.0 {
        return Ok(1);
    }
    let mut intervals = (length / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(CliError::run("gouge check: arc spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_SEGMENTS as f64 {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    Ok(count as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{CompensationMode, Feeds, Pose, RotaryAxisKind, RotaryDirection, RotarySetup, Tool, Vec3, machine_to_part};
    use campanion_mesh::{Mesh, Triangle};

    fn plane() -> Mesh {
        let s = 40.0;
        let z = 5.0;
        Mesh::new(vec![
            Triangle { a: MeshVec::new(-s, -s, z), b: MeshVec::new(s, -s, z), c: MeshVec::new(s, s, z) },
            Triangle { a: MeshVec::new(-s, -s, z), b: MeshVec::new(s, s, z), c: MeshVec::new(-s, s, z) },
        ])
        .expect("plane")
    }

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn ok<T>(result: Result<T, CliError>) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{err}"),
        }
    }

    fn program(name: &str, blocks: Vec<ClBlock>) -> Program {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(Some(name.into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let tool = Tool::try_new(1, None, Some(2.0), None).expect("tool");
        Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program")
    }

    #[test]
    fn tool_spec_parses_ball_flat_and_bull() {
        assert!(matches!(ok(parse_tool("ball:3")), Cutter::Ball { radius } if (radius - 3.0).abs() < 1e-12));
        assert!(matches!(ok(parse_tool("flat:3")), Cutter::Flat { radius } if (radius - 3.0).abs() < 1e-12));
        assert!(
            matches!(ok(parse_tool("bull:3,1")), Cutter::Bull { radius, corner_radius } if (radius - 3.0).abs() < 1e-12 && (corner_radius - 1.0).abs() < 1e-12)
        );
        assert!(parse_tool("ball").is_err());
        assert!(parse_tool("ball:0").is_err());
        assert!(parse_tool("bull:3").is_err());
        assert!(parse_tool("bull:3,4").is_err());
        assert!(parse_tool("taper:3").is_err());
    }

    #[test]
    fn linear_cut_below_the_plane_names_the_operation() {
        let mesh = plane();
        let surface = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index").tip_z(0.0, 0.0).expect("contact");
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, surface), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(0.2, 0.0, surface - 0.1), feed: 100.0, role: MoveRole::Cut }),
        ];
        let notes = ok(check_program(&program("dome", blocks), &mesh, Cutter::new_flat(1.0).expect("flat"), 0.01));
        let worst = notes.first().expect("gouge");
        assert_eq!(worst.operation, "dome");
        assert!((worst.depth - 0.1).abs() < 1e-9, "{}", worst.depth);
        assert!((worst.x - 0.2).abs() < 1e-9 && worst.y.abs() < 1e-9, "{worst:?}");
    }

    #[test]
    fn xy_arc_on_the_plane_is_clean_and_other_planes_are_rejected() {
        let mesh = plane();
        let surface = DropCutter::new(&mesh, Cutter::new_ball(1.0).expect("ball")).expect("index").tip_z(5.0, 0.0).expect("contact");
        let center = Vec3::new(0.0, 0.0, surface).expect("center");
        let arc = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface),
            plane: Plane::Xy,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface), feed: None }), arc];
        let notes = ok(check_program(&program("arc", blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01));
        assert!(notes.is_empty(), "{notes:?}");

        let low = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface - 0.5),
            plane: Plane::Xy,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let low_blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface - 0.5), feed: None }), low];
        let low_notes = ok(check_program(&program("arc", low_blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01));
        assert!(!low_notes.is_empty(), "lowered arc was not a gouge");
        assert!((low_notes[0].depth - 0.5).abs() < 1e-6, "{:?}", low_notes[0].depth);

        let zx = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface),
            plane: Plane::Zx,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let zx_blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface), feed: None }), zx];
        let err = check_program(&program("arc", zx_blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01).expect_err("zx");
        let text = err.to_string();
        assert!(text.contains("XY") && text.contains("lines"), "{text}");
    }

    /// A rough cut between the part and the stock allowance is not a gouge. The reference is the
    /// drop-cutter tip of the job's flat, ball or bull tool, with no `stock_to_leave` added.
    #[test]
    fn rough_gouge_uses_the_job_tool_and_ignores_stock_to_leave() {
        let mesh = plane();
        let stock = 1.0;
        let cases = [
            ("flat", r#""flat""#, Cutter::new_flat(3.0).expect("flat")),
            ("ball", r#""ball""#, Cutter::new_ball(3.0).expect("ball")),
            ("bull", r#"{"bull":1}"#, Cutter::new_bull(3.0, 1.0).expect("bull")),
        ];
        for (name, shape, cutter) in cases {
            let job = load_rough_job(name, shape, stock);
            let contact = DropCutter::new(&mesh, cutter).expect("index").tip_z(0.0, 0.0).expect("contact");
            let above_part = contact + stock * 0.25;
            let clean = ok(check_job(&job, &mesh, &program("rough", cut_at(above_part))));
            assert!(clean.is_empty(), "{name} flagged a cut above the part and below the stock: {clean:?}");
            let into = contact - 0.4;
            let gouges = ok(check_job(&job, &mesh, &program("rough", cut_at(into))));
            let Some(worst) = gouges.first() else {
                panic!("{name} cut below the part was not a gouge");
            };
            assert_eq!(worst.operation, "rough");
            assert!((worst.depth - 0.4).abs() < 1e-6, "{name} depth {}", worst.depth);
        }
    }

    fn cut_at(z: f64) -> Vec<ClBlock> {
        vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, z), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(1.0, 0.0, z), feed: 100.0, role: MoveRole::Cut }),
        ]
    }

    fn load_rough_job(name: &str, shape: &str, stock: f64) -> crate::jobfile::JobFile {
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","tools":[{{"id":1,"diameter":6,"shape":{shape}}}],"operations":[{{"name":"rough","type":"rough","tool_id":1,"step_down":1,"stepover":2,"stock_to_leave":{stock},"feeds":{{"cut":100}}}}]}}"#
        );
        let dir = std::env::temp_dir().join(format!("campanion-gouge-rough-{name}"));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("job.json");
        std::fs::write(&path, text).expect("write job");
        match crate::jobfile::load(&path) {
            Ok(job) => job,
            Err(err) => panic!("rough job {name}: {err}"),
        }
    }

    fn square_bar() -> Mesh {
        let (x0, x1) = (-12.0, 12.0);
        let (y0, y1, z0, z1) = (-8.0, 8.0, -8.0, 8.0);
        let p = |x, y, z| MeshVec::new(x, y, z);
        let (a, b, c, d) = (p(x0, y0, z0), p(x1, y0, z0), p(x1, y1, z0), p(x0, y1, z0));
        let (e, f, g, h) = (p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1));
        let mut triangles = Vec::new();
        let mut face = |a, b, c, d| {
            triangles.push(Triangle { a, b, c });
            triangles.push(Triangle { a, b: c, c: d });
        };
        face(e, f, g, h);
        face(a, d, c, b);
        face(d, h, g, c);
        face(a, b, f, e);
        face(b, c, g, f);
        face(a, e, h, d);
        Mesh::new(triangles).expect("bar")
    }

    fn at_machine(setup: &RotarySetup, x: f64, z: f64, angle: f64) -> Pose {
        let machine = Pose::new(Vec3::new(x, 0.0, z).expect("tip"), Vec3::Z).expect("pose");
        machine_to_part(setup, machine, angle).with_rotary(angle).expect("angle")
    }

    fn cut_block(to: Pose) -> ClBlock {
        ClBlock::Motion(ClMotion::Linear { to, feed: 100.0, role: MoveRole::Cut })
    }

    /// A square bar's corner swings above the flat faces. A rotate at the face height gouges only
    /// between the endpoint angles. A cut below the face at one angle is reported on that operation.
    #[test]
    fn rotary_finish_checks_each_angle_and_samples_angle_changes() {
        let mesh = square_bar();
        let setup = RotarySetup::new(RotaryAxisKind::A, Vec3::new(0.0, 0.0, 0.0).expect("center"), RotaryDirection::Positive, None).expect("setup");
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let spin = vec![
            ClBlock::Motion(ClMotion::Rapid { to: at_machine(&setup, -1.0, 10.0, 0.0), feed: None }),
            cut_block(at_machine(&setup, 1.0, 10.0, 0.0)),
            cut_block(at_machine(&setup, 1.0, 10.0, 90.0)),
            cut_block(at_machine(&setup, -1.0, 10.0, 90.0)),
        ];
        let low = vec![
            ClBlock::Motion(ClMotion::Rapid { to: at_machine(&setup, -1.0, 4.0, 0.0), feed: None }),
            cut_block(at_machine(&setup, 1.0, 4.0, 0.0)),
        ];
        let path = |name: &str, blocks: Vec<ClBlock>| {
            ClPath::try_new(Some(name.into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect(name)
        };
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        let program = Program::try_new(None, vec![tool], None, vec![path("spin", spin), path("low", low)], Vec::new(), Vec::new(), Some(setup))
            .expect("program");
        let text = concat!(
            r#"{"format":"campanion-job/1","source":{"path":"bar.stl","hash":"cbf29ce484222325"},"units":"mm","#,
            r#""rotary":{"axis":"a","center":[0,0,0],"direction":"positive"},"#,
            r#""tools":[{"id":1,"diameter":6,"shape":"flat"}],"operations":["#,
            r#"{"name":"spin","type":"rotary_finish","tool_id":1,"angle_step_deg":90,"feeds":{"cut":100}},"#,
            r#"{"name":"low","type":"rotary_finish","tool_id":1,"angle_step_deg":90,"feeds":{"cut":100}}"#,
            r#"],"tolerance":{"chord":0.05}}"#
        );
        let dir = std::env::temp_dir().join("campanion-gouge-rotary");
        std::fs::create_dir_all(&dir).expect("dir");
        let file = dir.join("job.json");
        std::fs::write(&file, text).expect("job");
        let job = crate::jobfile::load(&file).expect("load");
        let notes = ok(check_job(&job, &mesh, &program));
        let spin_notes: Vec<_> = notes.iter().filter(|note| note.operation == "spin").collect();
        assert!(!spin_notes.is_empty(), "rotate across the corner was not a gouge: {notes:?}");
        assert!(spin_notes.iter().any(|note| note.depth > 0.5), "{spin_notes:?}");
        assert!(spin_notes.iter().all(|note| (note.x - 1.0).abs() < 1e-4 && note.y.abs() < 1e-6 && (note.z - 10.0).abs() < 1e-6), "{spin_notes:?}");
        let low_notes: Vec<_> = notes.iter().filter(|note| note.operation == "low").collect();
        assert!(!low_notes.is_empty(), "lowered constant-angle cut was not a gouge");
        assert!(low_notes.iter().all(|note| (note.depth - 4.0).abs() < 1e-4 && (note.z - 4.0).abs() < 1e-6), "{low_notes:?}");
        assert!(low_notes.iter().all(|note| (-1.0..=1.0).contains(&note.x) && note.y.abs() < 1e-6), "{low_notes:?}");
        assert!(notes.iter().all(|note| note.operation == "spin" || note.operation == "low"), "{notes:?}");
    }
}
