//! Continuous tool axis for a ball-nose surface.
//!
//! The 3-axis drop-cutter tip `p` has ball centre `c = p + r*Z`. Any unit axis `a` with a positive
//! Z component keeps that sphere, and the tilted tip is `c - r*a`. The shank and the holder are
//! not checked here.
//!
//! A rapid that changes a rotary, and a rapid that follows that reorientation, starts at a safe
//! retract. The retract travels along the tool axis until the part is clear and the kinematics
//! machine Z is at least 0, so the posted rapid stays on or above that plane.

use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, DEFAULT_MAX_AXIS_STEP_DEG, DEFAULT_SINGULAR_CONE_DEG, Event, FiveAxisSetup, Joints, MoveRole, Pose,
    ProcessKind, ToolPose, Vec3 as IrVec, solve_path,
};
use campanion_mesh::{DropCutter, Vec3 as MeshVec};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, ToolAxis, ToolAxisMode, ToolShape};

/// One tilted sample. `tip3` is the 3-axis tip, including the floor and axial stock.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TiltedSample {
    pub tip3: MeshVec,
    pub tip: MeshVec,
    pub axis: MeshVec,
}

const DENSIFY_FACTOR: usize = 10;
const AXIS_STEP_TOL_DEG: f64 = 1e-6;
const COLLAPSE_XY: f64 = 1e-9;
/// Below this span a step that still exceeds the limit is a contact-normal jump, not a steep slope.
const BREAK_SPAN: f64 = 1e-3;
/// Added to a step / limit ratio before `ceil`, so platform libm differences cannot change a count.
const RATIO_NUDGE: f64 = 1e-6;
/// Posted non-TCP Z is the gauge point. At least this far above machine Z 0, so a 3-decimal
/// post cannot round the rapid negative. The rapid-safety scan rejects a rotary G0 below Z 0.
const SAFE_MACHINE_Z: f64 = 0.001;

struct Budget {
    limit: usize,
    emitted: usize,
    splits: usize,
}

struct SampleCtx<'a> {
    name: &'a str,
    spec: &'a ToolAxis,
    dc: &'a DropCutter<'a>,
    floor: f64,
    stock: f64,
    radius: f64,
    feed: Option<MeshVec>,
    max_step_deg: f64,
}

/// Raster lines with a tool axis. Each line is one or more runs. A new run starts where the
/// contact normal jumps, so the cut does not feed across that step. `lines` are 3-axis
/// drop-cutter samples; Z is recomputed.
pub(crate) fn tilted_samples(
    name: &str,
    spec: &ToolAxis,
    dc: &DropCutter,
    lines: &[Vec<MeshVec>],
    floor: f64,
    stock: f64,
    radius: f64,
    max_step_deg: f64,
) -> Result<Vec<Vec<Vec<TiltedSample>>>, CliError> {
    if lines.is_empty() || lines.iter().any(Vec::is_empty) {
        return Err(op_err(name, "surface produced no toolpath"));
    }
    let original = lines.iter().map(Vec::len).sum::<usize>();
    let Some(limit) = original.checked_mul(DENSIFY_FACTOR) else {
        return Err(too_many(name));
    };
    let mut budget = Budget { limit, emitted: 0, splits: 0 };
    let mut out = Vec::with_capacity(lines.len());
    for line in lines {
        let ctx = SampleCtx { name, spec, dc, floor, stock, radius, feed: line_feed(line), max_step_deg };
        let Some(first) = line.first() else {
            return Err(op_err(name, "surface produced no toolpath"));
        };
        let mut prev = sample_at(&ctx, first.x, first.y)?;
        note_emit(name, &mut budget)?;
        let mut runs = vec![vec![prev]];
        for point in line.iter().skip(1) {
            let end = sample_at(&ctx, point.x, point.y)?;
            fill_between(&ctx, &mut budget, &mut runs, prev, end)?;
            prev = end;
        }
        out.push(runs);
    }
    Ok(out)
}

/// One continuous toolpath. Subdivision is left to the post.
pub(crate) fn plan(
    job: &JobFile,
    op: &Operation,
    tool: &JobTool,
    dc: &DropCutter,
    lines: &[Vec<MeshVec>],
    floor: f64,
    stock: f64,
    radius: f64,
    base_rapid_z: f64,
) -> Result<ClPath, CliError> {
    let name = op.name();
    let Some(spec) = op.tool_axis() else {
        return Err(op_err(name, "tool_axis is missing"));
    };
    let Some(setup) = job.five_axis() else {
        return Err(op_err(name, "tool_axis needs a kinematics file"));
    };
    if !(matches!(tool.shape(), ToolShape::Ball) && radius.is_finite() && radius > 0.0) {
        return Err(op_err(name, "tool_axis needs a ball tool"));
    }
    let samples = tilted_samples(name, spec, dc, lines, floor, stock, radius, DEFAULT_MAX_AXIS_STEP_DEG)?;
    let mut runs = Vec::new();
    for line in &samples {
        for run in line {
            if !run.is_empty() {
                runs.push(run.as_slice());
            }
        }
    }
    let path = emit(op, tool, setup, &runs, base_rapid_z)?;
    check_reach(name, setup, &path)?;
    Ok(path)
}

fn emit(op: &Operation, tool: &JobTool, setup: &FiveAxisSetup, lines: &[&[TiltedSample]], base_rapid_z: f64) -> Result<ClPath, CliError> {
    let name = op.name();
    let Some(first_line) = lines.first() else {
        return Err(op_err(name, "surface produced no toolpath"));
    };
    let Some(first) = first_line.first().copied() else {
        return Err(op_err(name, "surface produced no toolpath"));
    };
    let mut rapid_z = base_rapid_z;
    for line in lines {
        for sample in *line {
            rapid_z = rapid_z.max(sample.tip.z);
        }
    }
    if !rapid_z.is_finite() {
        return Err(op_err(name, "tool axis retract is not finite"));
    }
    let params = op.params_with_speed(tool).map_err(|err| op_err(name, err))?;
    let plunge_feed = op.feeds().resolve(MoveRole::Plunge);
    let retract_feed = op.feeds().resolve(MoveRole::Retract);
    let cut_feed = op.feeds().resolve(MoveRole::Cut);
    let mut blocks = Vec::new();
    // The post solves from rotary angles at zero. Keep the same branch while lifting.
    let mut hint = [0.0, 0.0];
    let (approach, approach_angles) = safe_retract(name, setup, first.tip, first.axis, rapid_z, hint)?;
    hint = approach_angles;
    blocks.push(rapid(approach, first.axis)?);
    blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }));
    if separated(approach, first.tip) {
        blocks.push(linear(first.tip, first.axis, plunge_feed, MoveRole::Plunge)?);
    }
    let mut cursor_tip = first.tip;
    let mut cursor_axis = first.axis;
    for (line_index, line) in lines.iter().enumerate() {
        for (point_index, sample) in line.iter().copied().enumerate() {
            if line_index == 0 && point_index == 0 {
                continue;
            }
            if point_index == 0 {
                let (up, up_angles) = safe_retract(name, setup, cursor_tip, cursor_axis, rapid_z, hint)?;
                if separated(cursor_tip, up) {
                    blocks.push(linear(up, cursor_axis, retract_feed, MoveRole::Retract)?);
                }
                hint = up_angles;
                let (next_up, next_angles) = safe_retract(name, setup, sample.tip, sample.axis, rapid_z, hint)?;
                if separated(up, next_up) || axis_differs(cursor_axis, sample.axis) {
                    hint = push_orientation_rapids(name, setup, &mut blocks, up, cursor_axis, next_up, sample.axis, rapid_z, hint)?;
                } else {
                    hint = next_angles;
                }
                if separated(next_up, sample.tip) {
                    blocks.push(linear(sample.tip, sample.axis, plunge_feed, MoveRole::Plunge)?);
                }
            } else if separated(cursor_tip, sample.tip) || axis_differs(cursor_axis, sample.axis) {
                blocks.push(linear(sample.tip, sample.axis, cut_feed, MoveRole::Cut)?);
                hint = solve_angles(name, setup, sample.tip, sample.axis, hint)?;
            }
            cursor_tip = sample.tip;
            cursor_axis = sample.axis;
        }
    }
    let (up, _) = safe_retract(name, setup, cursor_tip, cursor_axis, rapid_z, hint)?;
    if separated(cursor_tip, up) {
        blocks.push(linear(up, cursor_axis, retract_feed, MoveRole::Retract)?);
    }
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }));
    let path = ClPath::try_new(Some(name.to_string()), Some(op.tool_id()), CompensationMode::Computed, op.feeds(), None, Vec::new(), blocks)
        .map_err(|err| op_err(name, strip_invalid(&err.to_string())))?;
    Ok(path.with_continuous(true))
}

fn check_reach(name: &str, setup: &FiveAxisSetup, path: &ClPath) -> Result<(), CliError> {
    let mut poses = Vec::new();
    for block in path.blocks() {
        let Some(pose) = motion_pose(block) else {
            continue;
        };
        let axis = unit_ir(pose.axis()).map_err(|_| op_err(name, "tool axis is zero"))?;
        poses.push(ToolPose { tip: pose.tip(), axis });
    }
    let start = Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] };
    match solve_path(setup, &poses, Some(start), path.singular_cone_deg()) {
        Ok(_) => Ok(()),
        Err(err) => Err(reach_error(name, &err.to_string(), &poses)),
    }
}

fn reach_error(name: &str, message: &str, poses: &[ToolPose]) -> CliError {
    let detail = strip_invalid(message);
    let located = match pose_index(detail).and_then(|index| poses.get(index)) {
        Some(pose) => format!("sample ({:.6}, {:.6}, {:.6}): {detail}", pose.tip.x(), pose.tip.y(), pose.tip.z()),
        None => detail.to_string(),
    };
    op_err(name, located)
}

fn pose_index(detail: &str) -> Option<usize> {
    let rest = detail.strip_prefix("pose ")?;
    let digits = rest.split(|ch: char| !ch.is_ascii_digit()).next()?;
    if digits.is_empty() { None } else { digits.parse().ok() }
}

fn fill_between(
    ctx: &SampleCtx<'_>,
    budget: &mut Budget,
    runs: &mut Vec<Vec<TiltedSample>>,
    start: TiltedSample,
    end: TiltedSample,
) -> Result<(), CliError> {
    let mut stack = vec![(start, end)];
    while let Some((left, right)) = stack.pop() {
        if axis_step_ok(left.axis, right.axis, ctx.max_step_deg) {
            note_emit(ctx.name, budget)?;
            push_sample(runs, right, false);
            continue;
        }
        let span = (right.tip3.x - left.tip3.x).hypot(right.tip3.y - left.tip3.y);
        if !(span >= COLLAPSE_XY) {
            note_emit(ctx.name, budget)?;
            push_sample(runs, right, true);
            continue;
        }
        note_split(ctx.name, budget)?;
        let mid = sample_at(ctx, (left.tip3.x + right.tip3.x) * 0.5, (left.tip3.y + right.tip3.y) * 0.5)?;
        if span < BREAK_SPAN && normal_jump(left.axis, mid.axis, right.axis, ctx.max_step_deg) {
            note_emit(ctx.name, budget)?;
            push_sample(runs, right, true);
            continue;
        }
        stack.push((mid, right));
        stack.push((left, mid));
    }
    Ok(())
}

fn push_sample(runs: &mut Vec<Vec<TiltedSample>>, sample: TiltedSample, new_run: bool) {
    if new_run || runs.is_empty() {
        runs.push(vec![sample]);
        return;
    }
    if let Some(run) = runs.last_mut() {
        run.push(sample);
    }
}

/// The step stays large on one side of the midpoint, so finer samples will not shrink it.
fn normal_jump(left: MeshVec, mid: MeshVec, right: MeshVec, max_step_deg: f64) -> bool {
    let full = axis_change_deg(left, right);
    if !full.is_finite() || full <= max_step_deg + AXIS_STEP_TOL_DEG {
        return false;
    }
    let to_left = axis_change_deg(left, mid);
    let to_right = axis_change_deg(mid, right);
    to_left.min(to_right) <= 1.0 && full > max_step_deg + AXIS_STEP_TOL_DEG
}

fn sample_at(ctx: &SampleCtx<'_>, x: f64, y: f64) -> Result<TiltedSample, CliError> {
    let z = ctx.dc.tip_z_or(x, y, ctx.floor) + ctx.stock;
    if !z.is_finite() {
        return Err(op_err(ctx.name, "surface produced a non-finite toolpath"));
    }
    let tip3 = MeshVec::new(x, y, z);
    let centre = MeshVec::new(x, y, z + ctx.radius);
    let normal = contact_normal(ctx.dc, centre);
    let axis = aim(ctx.name, ctx.spec, centre, normal, ctx.feed, tip3)?;
    let tip = centre.sub(axis.scale(ctx.radius));
    if !tip.x.is_finite() || !tip.y.is_finite() || !tip.z.is_finite() {
        return Err(op_err(ctx.name, "surface produced a non-finite toolpath"));
    }
    Ok(TiltedSample { tip3, tip, axis })
}

fn aim(name: &str, spec: &ToolAxis, centre: MeshVec, normal: MeshVec, feed: Option<MeshVec>, sample: MeshVec) -> Result<MeshVec, CliError> {
    let raw = match spec.mode() {
        ToolAxisMode::LeadTilt { lead_deg, tilt_deg } => lead_tilt_axis(normal, feed, *lead_deg, *tilt_deg),
        ToolAxisMode::TowardPoint { point } => unit_sample(name, sample, mesh_of(*point).sub(centre))?,
        ToolAxisMode::AwayFromPoint { point } => unit_sample(name, sample, centre.sub(mesh_of(*point)))?,
        ToolAxisMode::TowardCurve { curve } => {
            let target = closest_on_polyline(curve, centre);
            unit_sample(name, sample, target.sub(centre))?
        }
    };
    clamp_to_vertical(raw, spec.max_tilt_deg()).ok_or_else(|| zero_axis(name, sample))
}

fn lead_tilt_axis(normal: MeshVec, feed: Option<MeshVec>, lead_deg: f64, tilt_deg: f64) -> MeshVec {
    let Some(feed) = feed.and_then(MeshVec::normalized) else {
        return normal;
    };
    let mut axis = normal;
    if let Some(side) = normal.cross(feed).normalized() {
        axis = rodrigues(normal, side, lead_deg.to_radians());
        axis = axis.normalized().unwrap_or(normal);
    }
    axis = rodrigues(axis, feed, tilt_deg.to_radians());
    axis.normalized().unwrap_or(normal)
}

/// Rotate `vector` about a unit `axis` by `radians` (Rodrigues).
fn rodrigues(vector: MeshVec, axis: MeshVec, radians: f64) -> MeshVec {
    let (sin, cos) = radians.sin_cos();
    let along = axis.scale(axis.dot(vector));
    vector.scale(cos).add(axis.cross(vector).scale(sin)).add(along.scale(1.0 - cos))
}

fn clamp_to_vertical(axis: MeshVec, max_tilt_deg: f64) -> Option<MeshVec> {
    let axis = axis.normalized()?;
    let limit = max_tilt_deg.to_radians();
    let cos_limit = limit.cos();
    if axis.z >= cos_limit {
        return Some(axis);
    }
    let horizontal = MeshVec::new(axis.x, axis.y, 0.0);
    let Some(direction) = horizontal.normalized() else {
        return Some(MeshVec::new(0.0, 0.0, 1.0));
    };
    let tilted = MeshVec::new(direction.x * limit.sin(), direction.y * limit.sin(), cos_limit);
    Some(tilted.normalized().unwrap_or(MeshVec::new(0.0, 0.0, 1.0)))
}

fn contact_normal(dc: &DropCutter, centre: MeshVec) -> MeshVec {
    let up = MeshVec::new(0.0, 0.0, 1.0);
    if dc.tip_z(centre.x, centre.y).is_none() {
        return up;
    }
    let Some(contact) = dc.nearest_point(centre.x, centre.y, centre.z) else {
        return up;
    };
    centre.sub(contact).normalized().unwrap_or(up)
}

fn closest_on_polyline(curve: &[IrVec], point: MeshVec) -> MeshVec {
    let mut best: Option<(f64, MeshVec)> = None;
    for pair in curve.windows(2) {
        let Some(start) = pair.first().copied() else {
            continue;
        };
        let Some(end) = pair.get(1).copied() else {
            continue;
        };
        let hit = closest_on_segment(point, mesh_of(start), mesh_of(end));
        let distance = point.sub(hit).length();
        if !distance.is_finite() {
            continue;
        }
        let replace = match best {
            None => true,
            Some((best_distance, best_hit)) => {
                distance < best_distance - 1e-15 || ((distance - best_distance).abs() <= 1e-15 && lex_less(hit, best_hit))
            }
        };
        if replace {
            best = Some((distance, hit));
        }
    }
    best.map(|(_, hit)| hit).unwrap_or(point)
}

fn closest_on_segment(point: MeshVec, start: MeshVec, end: MeshVec) -> MeshVec {
    let span = end.sub(start);
    let len2 = span.dot(span);
    if !(len2 > 0.0) || !len2.is_finite() {
        return start;
    }
    let t = point.sub(start).dot(span) / len2;
    let t = if t.is_finite() { t.clamp(0.0, 1.0) } else { 0.0 };
    start.add(span.scale(t))
}

fn line_feed(line: &[MeshVec]) -> Option<MeshVec> {
    let start = line.first()?;
    let end = line.last()?;
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let span = dx.hypot(dy);
    if !span.is_finite() || span < 1e-12 { None } else { Some(MeshVec::new(dx / span, dy / span, 0.0)) }
}

fn axis_change_deg(left: MeshVec, right: MeshVec) -> f64 {
    let dot = left.dot(right).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

fn axis_step_ok(left: MeshVec, right: MeshVec, max_step_deg: f64) -> bool {
    let degrees = axis_change_deg(left, right);
    degrees.is_finite() && degrees <= max_step_deg + AXIS_STEP_TOL_DEG
}

fn note_emit(name: &str, budget: &mut Budget) -> Result<(), CliError> {
    budget.emitted = budget.emitted.saturating_add(1);
    if budget.emitted > budget.limit { Err(too_many(name)) } else { Ok(()) }
}

fn note_split(name: &str, budget: &mut Budget) -> Result<(), CliError> {
    budget.splits = budget.splits.saturating_add(1);
    if budget.splits > budget.limit { Err(too_many(name)) } else { Ok(()) }
}

/// Move along `axis` until the part Z is at least `part_z` and the solved machine Z is at least
/// [`SAFE_MACHINE_Z`]. Angles stay on the branch of `hint`, which is the previous pose.
fn safe_retract(
    name: &str,
    setup: &FiveAxisSetup,
    tip: MeshVec,
    axis: MeshVec,
    part_z: f64,
    hint: [f64; 2],
) -> Result<(MeshVec, [f64; 2]), CliError> {
    let mut t = lift_to_part_z(name, tip, axis, part_z)?;
    let mut solved = solve_tip(name, setup, along(tip, axis, t), axis, hint)?;
    for _ in 0..2 {
        if solved.xyz.z() >= SAFE_MACHINE_Z {
            return Ok((along(tip, axis, t), solved.angles));
        }
        let probe = solve_tip(name, setup, along(tip, axis, t + 1.0), axis, solved.angles)?;
        let slope = probe.xyz.z() - solved.xyz.z();
        if !(slope.is_finite() && slope > 1e-9) {
            return Err(op_err(name, "tool axis retract is not finite"));
        }
        t += (SAFE_MACHINE_Z - solved.xyz.z()) / slope;
        if !(t.is_finite() && (0.0..=1.0e6).contains(&t)) {
            return Err(op_err(name, "tool axis retract is not finite"));
        }
        solved = solve_tip(name, setup, along(tip, axis, t), axis, solved.angles)?;
    }
    if solved.xyz.z() < SAFE_MACHINE_Z - 1e-6 {
        return Err(op_err(name, "tool axis retract is not finite"));
    }
    Ok((along(tip, axis, t), solved.angles))
}

fn lift_to_part_z(name: &str, tip: MeshVec, axis: MeshVec, part_z: f64) -> Result<f64, CliError> {
    if tip.z >= part_z - 1e-9 {
        return Ok(0.0);
    }
    if !(axis.z > 1e-9) {
        return Err(zero_axis(name, tip));
    }
    let t = (part_z - tip.z) / axis.z;
    if t.is_finite() { Ok(t) } else { Err(op_err(name, "tool axis retract is not finite")) }
}

fn along(tip: MeshVec, axis: MeshVec, t: f64) -> MeshVec {
    MeshVec::new(tip.x + axis.x * t, tip.y + axis.y * t, tip.z + axis.z * t)
}

fn solve_tip(name: &str, setup: &FiveAxisSetup, tip: MeshVec, axis: MeshVec, hint: [f64; 2]) -> Result<Joints, CliError> {
    let tip_v = IrVec::new(tip.x, tip.y, tip.z).map_err(|err| CliError::run(strip_invalid(&err.to_string())))?;
    let axis_v = IrVec::new(axis.x, axis.y, axis.z).map_err(|err| CliError::run(strip_invalid(&err.to_string())))?;
    let pose = ToolPose { tip: tip_v, axis: axis_v };
    let start = Joints { xyz: IrVec::ZERO, angles: hint };
    match solve_path(setup, &[pose], Some(start), DEFAULT_SINGULAR_CONE_DEG) {
        Ok(joints) => joints.into_iter().next().ok_or_else(|| zero_axis(name, tip)),
        Err(err) => Err(reach_error(name, &err.to_string(), &[pose])),
    }
}

fn solve_angles(name: &str, setup: &FiveAxisSetup, tip: MeshVec, axis: MeshVec, hint: [f64; 2]) -> Result<[f64; 2], CliError> {
    Ok(solve_tip(name, setup, tip, axis, hint)?.angles)
}

fn separated(left: MeshVec, right: MeshVec) -> bool {
    let delta = left.sub(right);
    delta.dot(delta) > 1e-24
}

fn axis_differs(left: MeshVec, right: MeshVec) -> bool {
    left.dot(right) < 1.0 - 1e-12
}

fn linear(tip: MeshVec, axis: MeshVec, feed: f64, role: MoveRole) -> Result<ClBlock, CliError> {
    Ok(ClBlock::Motion(ClMotion::Linear { to: pose_of(tip, axis)?, feed, role, power: None }))
}

fn rapid(tip: MeshVec, axis: MeshVec) -> Result<ClBlock, CliError> {
    Ok(ClBlock::Motion(ClMotion::Rapid { to: pose_of(tip, axis)?, feed: None }))
}

/// A link rapid may swing the axis. Steps of at most the program axis limit keep `solve_path` on one branch.
///
/// Each sample is lifted so the rapid, and the next rapid after the reorientation, starts at a
/// safe retract. Joint-space subdivision then stays at or above machine Z 0.
fn push_orientation_rapids(
    name: &str,
    setup: &FiveAxisSetup,
    blocks: &mut Vec<ClBlock>,
    from_tip: MeshVec,
    from_axis: MeshVec,
    to_tip: MeshVec,
    to_axis: MeshVec,
    part_z: f64,
    mut hint: [f64; 2],
) -> Result<[f64; 2], CliError> {
    let step = axis_change_deg(from_axis, to_axis);
    // The angle comes from libm, which differs between platforms in the last bits. A step on a
    // whole multiple of the limit (20 deg at 5 deg) must split the same way everywhere, so the
    // ratio is nudged up: anything within 1e-6 of a multiple takes the extra piece.
    let mut count = 1usize;
    if step.is_finite() {
        let pieces = (step / DEFAULT_MAX_AXIS_STEP_DEG + RATIO_NUDGE).ceil();
        if pieces.is_finite() && pieces > 1.0 {
            count = pieces as usize;
        }
    }
    for index in 1..=count {
        let t = index as f64 / count as f64;
        let tip = along(from_tip, to_tip.sub(from_tip), t);
        let Some(axis) = slerp_unit(from_axis, to_axis, t) else {
            return Err(zero_axis(name, tip));
        };
        let (lifted, next) = safe_retract(name, setup, tip, axis, part_z, hint)?;
        hint = next;
        blocks.push(rapid(lifted, axis)?);
    }
    Ok(hint)
}

fn slerp_unit(start: MeshVec, end: MeshVec, t: f64) -> Option<MeshVec> {
    let start = start.normalized()?;
    let end = end.normalized()?;
    let dot = start.dot(end).clamp(-1.0, 1.0);
    let omega = dot.acos();
    if !(omega > 1e-8) {
        return Some(start);
    }
    let scale = omega.sin();
    if !(scale.abs() > 1e-12) {
        return Some(start);
    }
    let (sin_left, sin_right) = (((1.0 - t) * omega).sin(), (t * omega).sin());
    start.scale(sin_left / scale).add(end.scale(sin_right / scale)).normalized()
}

fn pose_of(tip: MeshVec, axis: MeshVec) -> Result<Pose, CliError> {
    let tip = IrVec::new(tip.x, tip.y, tip.z).map_err(|err| CliError::run(strip_invalid(&err.to_string())))?;
    let axis = IrVec::new(axis.x, axis.y, axis.z).map_err(|err| CliError::run(strip_invalid(&err.to_string())))?;
    Pose::new(tip, axis).map_err(|err| CliError::run(strip_invalid(&err.to_string())))
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

fn unit_ir(axis: IrVec) -> Result<IrVec, ()> {
    let len = (axis.x() * axis.x() + axis.y() * axis.y() + axis.z() * axis.z()).sqrt();
    if !len.is_finite() || len <= 0.0 {
        return Err(());
    }
    IrVec::new(axis.x() / len, axis.y() / len, axis.z() / len).map_err(|_| ())
}

fn unit_sample(name: &str, sample: MeshVec, vector: MeshVec) -> Result<MeshVec, CliError> {
    vector.normalized().ok_or_else(|| zero_axis(name, sample))
}

fn mesh_of(point: IrVec) -> MeshVec {
    MeshVec::new(point.x(), point.y(), point.z())
}

fn lex_less(left: MeshVec, right: MeshVec) -> bool {
    left.x < right.x - 1e-15
        || ((left.x - right.x).abs() <= 1e-15 && (left.y < right.y - 1e-15 || ((left.y - right.y).abs() <= 1e-15 && left.z < right.z)))
}

fn zero_axis(name: &str, sample: MeshVec) -> CliError {
    op_err(name, format!("tool axis is zero at sample ({:.6}, {:.6}, {:.6})", sample.x, sample.y, sample.z))
}

fn too_many(name: &str) -> CliError {
    op_err(name, "tool axis needs more than 10 times the raster samples to stay within the axis step")
}

fn op_err(name: &str, detail: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation `{name}`: {detail}"))
}

fn strip_invalid(message: &str) -> &str {
    message.strip_prefix("invalid input: ").unwrap_or(message)
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{Carrier, CompensationMode, Feeds, Kinematics, RotaryJoint, SolutionPolicy, Vec3 as IrVec};
    use campanion_mesh::{Cutter, Mesh, RasterAngle, RasterParams, Triangle, closest_point_on_triangle, parallel_finish};
    use proptest::prelude::*;

    fn plate() -> Mesh {
        Mesh::new(vec![
            Triangle { a: MeshVec::new(0.0, 0.0, 0.0), b: MeshVec::new(40.0, 0.0, 0.0), c: MeshVec::new(0.0, 40.0, 0.0) },
            Triangle { a: MeshVec::new(40.0, 0.0, 0.0), b: MeshVec::new(40.0, 40.0, 0.0), c: MeshVec::new(0.0, 40.0, 0.0) },
        ])
        .expect("plate")
    }

    fn ball_index(mesh: &Mesh, radius: f64) -> DropCutter<'_> {
        let cutter = Cutter::new_ball(radius).expect("ball");
        DropCutter::new(mesh, cutter).expect("index")
    }

    fn lead_spec(lead: f64, tilt: f64, max_tilt: f64) -> ToolAxis {
        ToolAxis::new(ToolAxisMode::LeadTilt { lead_deg: lead, tilt_deg: tilt }, max_tilt)
    }

    fn angle_between(left: MeshVec, right: MeshVec) -> f64 {
        let left = left.normalized().expect("left");
        let right = right.normalized().expect("right");
        left.cross(right).length().atan2(left.dot(right)).abs()
    }

    #[test]
    fn lead_rotates_toward_the_feed_and_tilt_is_about_the_feed() {
        let mesh = plate();
        let dc = ball_index(&mesh, 3.0);
        let lines = vec![vec![MeshVec::new(5.0, 5.0, 0.0), MeshVec::new(15.0, 5.0, 0.0)]];
        let lead = tilted_samples("face", &lead_spec(30.0, 0.0, 80.0), &dc, &lines, 0.0, 0.0, 3.0, 90.0).unwrap_or_else(|err| panic!("{err}"));
        let axis = lead[0][0][0].axis;
        let expect = 30.0_f64.to_radians();
        assert!((axis.x - expect.sin()).abs() < 1e-9, "lead x {}", axis.x);
        assert!(axis.y.abs() < 1e-9, "lead y {}", axis.y);
        assert!((axis.z - expect.cos()).abs() < 1e-9, "lead z {}", axis.z);

        let tilt = tilted_samples("face", &lead_spec(0.0, 30.0, 80.0), &dc, &lines, 0.0, 0.0, 3.0, 90.0).unwrap_or_else(|err| panic!("{err}"));
        let axis = tilt[0][0][0].axis;
        assert!(axis.x.abs() < 1e-9, "tilt x {}", axis.x);
        assert!((axis.y + expect.sin()).abs() < 1e-9, "tilt y {}", axis.y);
        assert!((axis.z - expect.cos()).abs() < 1e-9, "tilt z {}", axis.z);
    }

    #[test]
    fn toward_point_clamps_a_horizontal_aim_and_away_points_opposite() {
        let mesh = plate();
        let dc = ball_index(&mesh, 3.0);
        let lines = vec![vec![MeshVec::new(5.0, 5.0, 0.0), MeshVec::new(6.0, 5.0, 0.0)]];
        let point = IrVec::new(1000.0, 5.0, 3.0).expect("point");
        let toward = ToolAxis::new(ToolAxisMode::TowardPoint { point }, 60.0);
        let samples = tilted_samples("face", &toward, &dc, &lines, 0.0, 0.0, 3.0, 90.0).unwrap_or_else(|err| panic!("{err}"));
        let axis = samples[0][0][0].axis;
        let limit = 60.0_f64.to_radians();
        assert!((axis.x - limit.sin()).abs() < 1e-9, "{}", axis.x);
        assert!(axis.y.abs() < 1e-9);
        assert!((axis.z - limit.cos()).abs() < 1e-9, "{}", axis.z);
        let away = ToolAxis::new(ToolAxisMode::AwayFromPoint { point }, 60.0);
        let samples = tilted_samples("face", &away, &dc, &lines, 0.0, 0.0, 3.0, 90.0).unwrap_or_else(|err| panic!("{err}"));
        let axis = samples[0][0][0].axis;
        assert!((axis.x + limit.sin()).abs() < 1e-9, "{}", axis.x);
        assert!((axis.z - limit.cos()).abs() < 1e-9, "{}", axis.z);
    }

    #[test]
    fn zero_axis_names_the_sample() {
        let mesh = plate();
        let dc = ball_index(&mesh, 3.0);
        let lines = vec![vec![MeshVec::new(5.0, 5.0, 0.0), MeshVec::new(15.0, 5.0, 0.0)]];
        let point = IrVec::new(5.0, 5.0, 3.0).expect("centre");
        let spec = ToolAxis::new(ToolAxisMode::TowardPoint { point }, 60.0);
        let err = tilted_samples("face", &spec, &dc, &lines, 0.0, 0.0, 3.0, 5.0).expect_err("zero");
        assert_eq!(err.to_string(), "operation `face`: tool axis is zero at sample (5.000000, 5.000000, 0.000000)");
    }

    #[test]
    fn densify_stops_past_ten_times_the_raster() {
        let mesh = plate();
        let dc = ball_index(&mesh, 3.0);
        let lines = vec![vec![MeshVec::new(5.0, 5.0, 0.0), MeshVec::new(25.0, 5.0, 0.0)]];
        let point = IrVec::new(0.0, 80.0, 40.0).expect("point");
        let spec = ToolAxis::new(ToolAxisMode::TowardPoint { point }, 60.0);
        let err = tilted_samples("face", &spec, &dc, &lines, 0.0, 0.0, 3.0, 0.001).expect_err("budget");
        assert_eq!(err.to_string(), "operation `face`: tool axis needs more than 10 times the raster samples to stay within the axis step");
    }

    #[test]
    fn an_unreachable_sample_names_the_operation_and_the_pose() {
        let kin = Kinematics {
            name: "c_only".to_string(),
            rotaries: vec![RotaryJoint { letter: 'C', carrier: Carrier::Table, axis: IrVec::Z, pivot: IrVec::ZERO, limits: None }],
            spindle_axis: IrVec::Z,
            model: None,
        };
        let setup = FiveAxisSetup { kinematics: kin, policy: SolutionPolicy::Nearest };
        let tip = IrVec::new(1.0, 2.0, 3.0).expect("tip");
        let axis = IrVec::new(0.0, 0.5, (0.75_f64).sqrt()).expect("axis");
        let pose = Pose::new(tip, axis).expect("pose");
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(
            Some("dome".to_string()),
            Some(1),
            CompensationMode::Computed,
            feeds,
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Linear { to: pose, feed: 100.0, role: MoveRole::Cut, power: None })],
        )
        .expect("path")
        .with_continuous(true);
        let err = check_reach("dome", &setup, &path).expect_err("reach");
        assert_eq!(err.to_string(), "operation `dome`: sample (1.000000, 2.000000, 3.000000): pose 0 is unreachable");
    }

    fn dome_mesh(radius: f64, slices: usize, rings: usize) -> Mesh {
        let mut triangles = Vec::new();
        let pole = MeshVec::new(0.0, 0.0, radius);
        let point = |ring: usize, slice: usize| {
            let phi = ring as f64 / rings as f64 * std::f64::consts::FRAC_PI_2;
            let theta = slice as f64 / slices as f64 * std::f64::consts::TAU;
            MeshVec::new(radius * phi.sin() * theta.cos(), radius * phi.sin() * theta.sin(), radius * phi.cos())
        };
        for slice in 0..slices {
            triangles.push(Triangle { a: pole, b: point(1, slice), c: point(1, (slice + 1) % slices) });
        }
        for ring in 1..rings {
            for slice in 0..slices {
                let next = (slice + 1) % slices;
                let a = point(ring, slice);
                let b = point(ring, next);
                let c = point(ring + 1, slice);
                let d = point(ring + 1, next);
                triangles.push(Triangle { a, b, c: d });
                triangles.push(Triangle { a, b: d, c });
            }
        }
        Mesh::new(triangles).expect("dome")
    }

    fn slope_mesh(rise: f64) -> Mesh {
        Mesh::new(vec![
            Triangle { a: MeshVec::new(0.0, 0.0, 0.0), b: MeshVec::new(30.0, 0.0, rise), c: MeshVec::new(0.0, 30.0, rise * 0.4) },
            Triangle { a: MeshVec::new(30.0, 0.0, rise), b: MeshVec::new(30.0, 30.0, rise * 1.1), c: MeshVec::new(0.0, 30.0, rise * 0.4) },
        ])
        .expect("slope")
    }

    #[derive(Clone, Debug)]
    enum ShapeGen {
        Dome { slices: usize, rings: usize },
        Slope { rise: f64 },
    }

    #[derive(Clone, Debug)]
    enum ModeGen {
        Lead { lead: f64, tilt: f64 },
        Toward { x: f64, y: f64, z: f64 },
        Away { x: f64, y: f64, z: f64 },
        Curve { x: f64, y: f64 },
    }

    fn shape_strategy() -> impl Strategy<Value = ShapeGen> {
        prop_oneof![
            (24usize..48, 12usize..20).prop_map(|(slices, rings)| ShapeGen::Dome { slices, rings }),
            (0.2f64..3.0).prop_map(|rise| ShapeGen::Slope { rise }),
        ]
    }

    fn mode_strategy() -> impl Strategy<Value = ModeGen> {
        prop_oneof![
            (-40.0f64..40.0, -40.0f64..40.0).prop_map(|(lead, tilt)| ModeGen::Lead { lead, tilt }),
            (-20.0f64..20.0, -20.0f64..20.0, 40.0f64..120.0).prop_map(|(x, y, z)| ModeGen::Toward { x, y, z }),
            (-20.0f64..20.0, -20.0f64..20.0, 40.0f64..120.0).prop_map(|(x, y, z)| ModeGen::Away { x, y, z }),
            (-15.0f64..15.0, -15.0f64..15.0).prop_map(|(x, y)| ModeGen::Curve { x, y }),
        ]
    }

    fn spec_of(mode: &ModeGen, max_tilt: f64) -> ToolAxis {
        let mode = match *mode {
            ModeGen::Lead { lead, tilt } => ToolAxisMode::LeadTilt { lead_deg: lead, tilt_deg: tilt },
            ModeGen::Toward { x, y, z } => ToolAxisMode::TowardPoint { point: IrVec::new(x, y, z).expect("point") },
            ModeGen::Away { x, y, z } => ToolAxisMode::AwayFromPoint { point: IrVec::new(x, y, z).expect("point") },
            ModeGen::Curve { x, y } => ToolAxisMode::TowardCurve {
                curve: vec![
                    IrVec::new(x, y, 70.0).expect("curve"),
                    IrVec::new(x + 12.0, y + 4.0, 90.0).expect("curve"),
                    IrVec::new(x + 20.0, y - 6.0, 80.0).expect("curve"),
                ],
            },
        };
        ToolAxis::new(mode, max_tilt)
    }

    fn mesh_of_gen(shape: &ShapeGen) -> Mesh {
        match *shape {
            ShapeGen::Dome { slices, rings } => dome_mesh(18.0, slices, rings),
            ShapeGen::Slope { rise } => slope_mesh(rise),
        }
    }

    fn clamped_normal(normal: MeshVec, max_tilt_deg: f64) -> MeshVec {
        let normal = normal.normalized().expect("normal");
        let limit = max_tilt_deg.to_radians();
        if normal.z >= limit.cos() {
            return normal;
        }
        let horizontal = MeshVec::new(normal.x, normal.y, 0.0);
        let Some(direction) = horizontal.normalized() else {
            return MeshVec::new(0.0, 0.0, 1.0);
        };
        MeshVec::new(direction.x * limit.sin(), direction.y * limit.sin(), limit.cos())
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(24))]

        #[test]
        fn tilted_samples_keep_the_ball_clear_of_the_mesh(
            shape in shape_strategy(),
            mode in mode_strategy(),
            max_tilt in 20.0f64..70.0,
        ) {
            let mesh = mesh_of_gen(&shape);
            let radius = 2.0;
            let dc = ball_index(&mesh, radius);
            let bounds = mesh.bounds().expect("bounds");
            let floor = bounds.min().z;
            let params = RasterParams {
                stepover: 10.0,
                sample_step: 5.0,
                tolerance: 1.0,
                angle: RasterAngle::AlongX,
                floor_z: floor,
                zigzag: true,
            };
            let lines = parallel_finish(&dc, (bounds.min().x, bounds.min().y), (bounds.max().x, bounds.max().y), &params).expect("raster");
            let spec = spec_of(&mode, max_tilt);
            let samples = match tilted_samples("face", &spec, &dc, &lines, floor, 0.0, radius, DEFAULT_MAX_AXIS_STEP_DEG) {
                Ok(samples) => samples,
                Err(err) => {
                    prop_assume!(!err.to_string().contains("more than 10 times"));
                    panic!("{err}");
                }
            };
            prop_assume!(!samples.is_empty());
            for line in &samples {
                for run in line {
                let mut previous: Option<MeshVec> = None;
                for sample in run {
                    let centre = sample.tip.add(sample.axis.scale(radius));
                    let expected = MeshVec::new(sample.tip3.x, sample.tip3.y, sample.tip3.z + radius);
                    let gap = centre.sub(expected).length();
                    prop_assert!(gap <= 1e-9, "centre gap {gap}");
                    let drop = dc.tip_z_or(centre.x, centre.y, floor) + radius;
                    prop_assert!((centre.z - drop).abs() <= 1e-9, "drop centre");
                    let len = sample.axis.length();
                    prop_assert!((len - 1.0).abs() <= 1e-9, "axis length {len}");
                    let tilt = sample.axis.z.clamp(-1.0, 1.0).acos().to_degrees();
                    prop_assert!(tilt <= max_tilt + 1e-6, "tilt {tilt} max {max_tilt}");
                    if let Some(prev) = previous {
                        let step = prev.dot(sample.axis).clamp(-1.0, 1.0).acos().to_degrees();
                        prop_assert!(step <= DEFAULT_MAX_AXIS_STEP_DEG + AXIS_STEP_TOL_DEG, "step {step}");
                    }
                    for triangle in mesh.triangles() {
                        let hit = closest_point_on_triangle(centre, *triangle);
                        let distance = centre.sub(hit).length();
                        prop_assert!(distance + 1e-6 >= radius, "penetration {}", radius - distance);
                    }
                    match spec.mode() {
                        ToolAxisMode::TowardPoint { point } | ToolAxisMode::AwayFromPoint { point } => {
                            let target = mesh_of(*point);
                            let raw = if matches!(spec.mode(), ToolAxisMode::AwayFromPoint { .. }) { centre.sub(target) } else { target.sub(centre) };
                            assert_aim(sample.axis, raw, max_tilt);
                        }
                        ToolAxisMode::TowardCurve { curve } => {
                            let target = closest_polyline_ref(curve, centre);
                            assert_aim(sample.axis, target.sub(centre), max_tilt);
                        }
                        ToolAxisMode::LeadTilt { .. } => {}
                    }
                    previous = Some(sample.axis);
                }
                }
            }
        }

        #[test]
        fn lead_and_tilt_zero_follows_the_contact_normal(shape in shape_strategy(), max_tilt in 15.0f64..75.0) {
            let mesh = mesh_of_gen(&shape);
            let radius = 2.0;
            let dc = ball_index(&mesh, radius);
            let bounds = mesh.bounds().expect("bounds");
            let floor = bounds.min().z;
            let params = RasterParams {
                stepover: 12.0,
                sample_step: 6.0,
                tolerance: 1.0,
                angle: RasterAngle::AlongX,
                floor_z: floor,
                zigzag: true,
            };
            let lines = parallel_finish(&dc, (bounds.min().x, bounds.min().y), (bounds.max().x, bounds.max().y), &params).expect("raster");
            let spec = lead_spec(0.0, 0.0, max_tilt);
            let samples = match tilted_samples("face", &spec, &dc, &lines, floor, 0.0, radius, DEFAULT_MAX_AXIS_STEP_DEG) {
                Ok(samples) => samples,
                Err(err) => {
                    prop_assume!(!err.to_string().contains("more than 10 times"));
                    panic!("{err}");
                }
            };
            for line in samples {
                for run in line {
                for sample in run {
                    let centre = MeshVec::new(sample.tip3.x, sample.tip3.y, sample.tip3.z + radius);
                    let normal = if dc.tip_z(centre.x, centre.y).is_none() {
                        MeshVec::new(0.0, 0.0, 1.0)
                    } else {
                        match dc.nearest_point(centre.x, centre.y, centre.z) {
                            Some(contact) => centre.sub(contact).normalized().unwrap_or(MeshVec::new(0.0, 0.0, 1.0)),
                            None => MeshVec::new(0.0, 0.0, 1.0),
                        }
                    };
                    let expect = clamped_normal(normal, max_tilt);
                    let gap = angle_between(sample.axis, expect);
                    prop_assert!(gap <= 1e-9, "normal gap {gap}");
                }
                }
            }
        }
    }

    fn assert_aim(axis: MeshVec, raw: MeshVec, max_tilt_deg: f64) {
        let Some(raw) = raw.normalized() else {
            return;
        };
        let raw_tilt = raw.z.clamp(-1.0, 1.0).acos().to_degrees();
        if raw_tilt <= max_tilt_deg + 1e-7 {
            let gap = angle_between(axis, raw);
            assert!(gap <= 1e-9, "aim gap {gap}");
            return;
        }
        let tilt = axis.z.clamp(-1.0, 1.0).acos();
        assert!((tilt - max_tilt_deg.to_radians()).abs() <= 1e-8, "clamped tilt");
        let plane = raw.cross(MeshVec::new(0.0, 0.0, 1.0));
        if plane.length() > 1e-8 {
            assert!(axis.dot(plane).abs() <= 1e-7, "clamped axis left the plane");
        }
    }

    fn closest_polyline_ref(curve: &[IrVec], point: MeshVec) -> MeshVec {
        let mut best: Option<(f64, MeshVec)> = None;
        for pair in curve.windows(2) {
            let start = mesh_of(pair[0]);
            let end = mesh_of(pair[1]);
            let span = end.sub(start);
            let len2 = span.dot(span);
            let hit = if len2 > 0.0 {
                let t = point.sub(start).dot(span) / len2;
                start.add(span.scale(t.clamp(0.0, 1.0)))
            } else {
                start
            };
            let distance = point.sub(hit).length();
            if best.as_ref().is_none_or(|(best_distance, _)| distance < *best_distance) {
                best = Some((distance, hit));
            }
        }
        best.map(|(_, hit)| hit).unwrap_or(point)
    }

    fn table_table() -> FiveAxisSetup {
        let kinematics = Kinematics::from_json(include_str!("../../../examples/table_table_bc.kin.json")).expect("kinematics");
        FiveAxisSetup { kinematics, policy: SolutionPolicy::Nearest }
    }

    #[test]
    fn a_buried_tilted_pose_retracts_until_machine_z_is_safe() {
        let setup = table_table();
        let buried = Joints { xyz: IrVec::new(50.0, 10.0, -15.0).expect("xyz"), angles: [25.0, 40.0] };
        let pose = setup.kinematics.forward(&buried, 0.0).expect("forward");
        let tip = MeshVec::new(pose.tip.x(), pose.tip.y(), pose.tip.z());
        let axis = MeshVec::new(pose.axis.x(), pose.axis.y(), pose.axis.z());
        assert!(buried.xyz.z() < 0.0, "precondition");
        let (lifted, angles) = safe_retract("dome", &setup, tip, axis, tip.z, buried.angles).unwrap_or_else(|err| panic!("retract: {err}"));
        let solved = solve_tip("dome", &setup, lifted, axis, buried.angles).unwrap_or_else(|err| panic!("solve: {err}"));
        assert!(solved.xyz.z() >= SAFE_MACHINE_Z - 1e-6, "machine Z {}", solved.xyz.z());
        let delta = lifted.sub(tip);
        assert!(delta.dot(axis) > 0.0, "retract moves along the axis");
        assert!(delta.cross(axis).length() < 1e-6, "retract left the axis");
        assert!((angles[0] - solved.angles[0]).abs() < 1e-6);
        assert!((angles[1] - solved.angles[1]).abs() < 1e-6);
    }

    #[test]
    fn reorientation_rapids_start_at_a_safe_machine_retract() {
        let dir = std::env::temp_dir().join("campanion-toolaxis-seam");
        std::fs::create_dir_all(&dir).expect("dir");
        std::fs::write(dir.join("table_table_bc.kin.json"), include_str!("../../../examples/table_table_bc.kin.json")).expect("kin");
        let job_path = dir.join("seam.job.json");
        std::fs::write(
            &job_path,
            r#"{"format":"campanion-job/1","source":{"path":"seam.stl","hash":"0000000000000000"},"units":"mm","kinematics":"table_table_bc.kin.json","tools":[{"id":1,"diameter":6,"shape":"ball"}],"operations":[{"name":"seam","type":"surface","tool_id":1,"stepover":3,"sample_step":4,"tool_axis":{"mode":"lead_tilt","lead_deg":10,"tilt_deg":0,"max_tilt_deg":30},"feeds":{"cut":1000,"plunge":300}}]}"#,
        )
        .expect("job");
        let job = crate::jobfile::load(&job_path).expect("load");
        let op = &job.operations()[0];
        let tool = job.tool(op.tool_id()).expect("tool");
        let setup = job.five_axis().expect("kinematics");
        let mesh = Mesh::new(vec![
            Triangle { a: MeshVec::new(0.0, 0.0, 0.0), b: MeshVec::new(40.0, 0.0, 0.0), c: MeshVec::new(0.0, 30.0, 0.0) },
            Triangle { a: MeshVec::new(40.0, 0.0, 0.0), b: MeshVec::new(40.0, 30.0, 0.0), c: MeshVec::new(0.0, 30.0, 0.0) },
            Triangle { a: MeshVec::new(18.0, 0.0, 14.0), b: MeshVec::new(24.0, 0.0, 0.0), c: MeshVec::new(18.0, 30.0, 14.0) },
            Triangle { a: MeshVec::new(24.0, 0.0, 0.0), b: MeshVec::new(24.0, 30.0, 0.0), c: MeshVec::new(18.0, 30.0, 14.0) },
        ])
        .expect("mesh");
        let dc = ball_index(&mesh, 3.0);
        let params = RasterParams { stepover: 8.0, sample_step: 4.0, tolerance: 0.05, angle: RasterAngle::AlongX, floor_z: 0.0, zigzag: true };
        let lines = parallel_finish(&dc, (0.0, 0.0), (40.0, 30.0), &params).expect("raster");
        let path = plan(&job, op, tool, &dc, &lines, 0.0, 0.0, 3.0, 20.0).unwrap_or_else(|err| panic!("plan: {err}"));
        let mut poses = Vec::new();
        let mut rapid = Vec::new();
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else { continue };
            let pose = match motion {
                ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. } => *to,
                _ => continue,
            };
            rapid.push(matches!(motion, ClMotion::Rapid { .. }));
            poses.push(ToolPose { tip: pose.tip(), axis: pose.axis() });
        }
        let start = Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] };
        let joints = solve_path(setup, &poses, Some(start), DEFAULT_SINGULAR_CONE_DEG).expect("solve");
        let mut prev_z = 0.0;
        let mut prev_angles = [0.0, 0.0];
        let mut prev_changed = false;
        let mut saw_rotary_rapid = false;
        for (index, joint) in joints.iter().enumerate() {
            let z = joint.xyz.z();
            let changed = (joint.angles[0] - prev_angles[0]).abs() > 1e-6 || (joint.angles[1] - prev_angles[1]).abs() > 1e-6;
            if rapid[index] {
                assert!(z >= -1e-9, "rapid ends at machine Z {z}");
                if changed || prev_changed {
                    assert!(prev_z >= -1e-9, "rapid starts at machine Z {prev_z}");
                }
                if changed {
                    saw_rotary_rapid = true;
                }
                let mid = (prev_z + z) * 0.5;
                assert!(mid >= -1e-9, "rapid crosses machine Z {mid}");
            }
            prev_changed = changed;
            prev_z = z;
            prev_angles = joint.angles;
        }
        assert!(saw_rotary_rapid, "expected a rapid that changes a rotary");
    }
}
