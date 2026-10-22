//! Parallel finish over an STL mesh.
//!
//! The raster comes from `campanion-mesh`. Each cutter-location Z is raised by the operation's
//! axial stock. Clearance and retract stay at the job heights. Links between raster lines are
//! drop-cutter samples along the straight segment, so the feed stays on the surface.

use campanion_mesh::{Cutter, DropCutter, Mesh, RasterAngle, RasterParams, Vec3 as MeshVec, parallel_finish};
use campanion_path::{CutKind, Move, Point3, ProcessEvent, Toolpath};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, ToolShape};

/// Matches the move cap in `campanion-path`. A longer list is rejected before it is built.
const MAX_MOVES: usize = 65_536;
/// Rapid in, process on, retract, process off and rapid out. The first sample is the plunge.
const PATH_OVERHEAD: usize = 5;
/// Most samples inserted on one link between raster lines.
const MAX_LINK_STEPS: usize = 100_000;

/// One surface toolpath. `mesh` is the job's STL. Z of every cutter location includes axial stock.
pub(crate) fn finish(job: &JobFile, op: &Operation, tool: &JobTool, mesh: &Mesh) -> Result<Toolpath, CliError> {
    let bounds = mesh.bounds().map_err(|err| op_err(op, err))?;
    let stepover = op.stepover().ok_or_else(|| op_err(op, "surface needs a stepover"))?;
    let sample_step = op.sample_step().ok_or_else(|| op_err(op, "surface needs a sample_step"))?;
    let floor = op.floor_z().unwrap_or_else(|| bounds.min().z);
    let stock = op.stock_to_leave().floor();
    let margin = op.margin();
    let cutter = cutter(op, tool)?;
    let dc = DropCutter::new(mesh, cutter).map_err(|err| op_err(op, err))?;
    let angle = if (op.face_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-9 { RasterAngle::AlongY } else { RasterAngle::AlongX };
    let params = RasterParams { stepover, sample_step, tolerance: job.tolerance().chord(), angle, floor_z: floor, zigzag: true };
    let area_min = (bounds.min().x - margin, bounds.min().y - margin);
    let area_max = (bounds.max().x + margin, bounds.max().y + margin);
    let lines = parallel_finish(&dc, area_min, area_max, &params).map_err(|err| op_err(op, err))?;
    let points = collect_points(op, &dc, &lines, sample_step, floor, stock)?;
    let Some(first) = points.first().copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let Some(last) = points.last().copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let mut moves = Vec::with_capacity(points.len().saturating_add(5));
    push_move(&mut moves, Move::Rapid { to: point(op, first.x, first.y, job.clearance())? })?;
    push_move(&mut moves, Move::Event(ProcessEvent::On))?;
    push_move(&mut moves, Move::Linear { to: point(op, first.x, first.y, first.z)?, kind: CutKind::Plunge })?;
    for sample in points.iter().skip(1) {
        push_move(&mut moves, Move::Linear { to: point(op, sample.x, sample.y, sample.z)?, kind: CutKind::Cut })?;
    }
    push_move(&mut moves, Move::Linear { to: point(op, last.x, last.y, job.retract())?, kind: CutKind::Retract })?;
    push_move(&mut moves, Move::Event(ProcessEvent::Off))?;
    push_move(&mut moves, Move::Rapid { to: point(op, last.x, last.y, job.clearance())? })?;
    Toolpath::new(moves).map_err(|err| op_err(op, err))
}

fn cutter(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "surface is not valid with a kerf tool"));
    };
    let radius = diameter * 0.5;
    let cutter = match tool.shape() {
        ToolShape::Flat => Cutter::new_flat(radius),
        ToolShape::Ball => Cutter::new_ball(radius),
        ToolShape::Bull { corner_radius } => Cutter::new_bull(radius, corner_radius),
    };
    cutter.map_err(|err| op_err(op, err))
}

/// Raster vertices, plus drop-cutter samples on each straight link. Every Z includes `stock`.
fn collect_points(
    op: &Operation,
    dc: &DropCutter,
    lines: &[Vec<MeshVec>],
    sample_step: f64,
    floor: f64,
    stock: f64,
) -> Result<Vec<MeshVec>, CliError> {
    if lines.is_empty() || lines.iter().any(Vec::is_empty) {
        return Err(op_err(op, "surface produced no toolpath"));
    }
    let mut points = Vec::new();
    for (index, line) in lines.iter().enumerate() {
        if index > 0 {
            let Some(from) = points.last().copied() else {
                return Err(op_err(op, "surface produced no toolpath"));
            };
            let Some(to) = line.first().copied() else {
                return Err(op_err(op, "surface produced no toolpath"));
            };
            push_link(op, &mut points, dc, from, to, sample_step, floor, stock)?;
        }
        for sample in line {
            push_sample(op, &mut points, MeshVec::new(sample.x, sample.y, sample.z + stock))?;
        }
    }
    Ok(points)
}

/// Samples strictly between `from` and `to`, spaced by `sample_step` along XY.
fn push_link(
    op: &Operation,
    points: &mut Vec<MeshVec>,
    dc: &DropCutter,
    from: MeshVec,
    to: MeshVec,
    sample_step: f64,
    floor: f64,
    stock: f64,
) -> Result<(), CliError> {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let span = dx.hypot(dy);
    if !span.is_finite() || span <= sample_step {
        return Ok(());
    }
    let steps_f = (span / sample_step).ceil();
    if !steps_f.is_finite() || steps_f < 2.0 || steps_f > MAX_LINK_STEPS as f64 {
        return Err(op_err(op, format!("too many surface link samples: limit is {MAX_LINK_STEPS}")));
    }
    let steps = steps_f as usize;
    for index in 1..steps {
        let t = index as f64 / steps as f64;
        let x = from.x + dx * t;
        let y = from.y + dy * t;
        let z = dc.tip_z_or(x, y, floor) + stock;
        push_sample(op, points, MeshVec::new(x, y, z))?;
    }
    Ok(())
}

fn push_sample(op: &Operation, points: &mut Vec<MeshVec>, point: MeshVec) -> Result<(), CliError> {
    if points.last().is_some_and(|prev| prev.x == point.x && prev.y == point.y && prev.z == point.z) {
        return Ok(());
    }
    if points.len().saturating_add(PATH_OVERHEAD) >= MAX_MOVES {
        return Err(op_err(op, format!("too many moves: {} > {MAX_MOVES}", points.len() + PATH_OVERHEAD + 1)));
    }
    points.push(point);
    Ok(())
}

fn push_move(moves: &mut Vec<Move>, mv: Move) -> Result<(), CliError> {
    if moves.len() >= MAX_MOVES {
        return Err(CliError::invalid(format!("too many moves: {} > {MAX_MOVES}", moves.len() + 1)));
    }
    moves.push(mv);
    Ok(())
}

fn point(op: &Operation, x: f64, y: f64, z: f64) -> Result<Point3, CliError> {
    Point3::new(x, y, z).map_err(|err| op_err(op, err))
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}
