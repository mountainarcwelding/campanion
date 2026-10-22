//! Parallel finish over an STL mesh.
//!
//! The raster comes from `campanion-mesh`. Each cutter-location Z is raised by the operation's
//! axial stock. Clearance and retract stay at the job heights. Links between raster lines are
//! drop-cutter samples along the straight segment, so the feed stays on the surface.
//!
//! A 3-axis surface keeps every raster sample, including cutter misses at the mesh floor, so an
//! existing job stays byte-identical. An indexed view (the job has a rotary setup) does not.
//! Samples the cutter misses are dropped. The tool retracts, rapids at clearance, and plunges
//! between the remaining hits, and it does the same when a chord between hits would gouge.

use campanion_mesh::{Cutter, DropCutter, Mesh, RasterAngle, RasterParams, Vec3 as MeshVec, parallel_finish, scallop_stepover};
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
    let floor = op.floor_z().unwrap_or_else(|| bounds.min().z);
    let stock = op.stock_to_leave().floor();
    let margin = op.margin();
    let cutter = cutter(op, tool)?;
    // A scallop height is the flat-floor cusp. The stepover it produces is capped at the tool
    // diameter; an explicit stepover larger than the diameter was already rejected at load.
    let stepover = resolve_stepover(op, tool, &cutter)?;
    let sample_step = op.sample_step().unwrap_or(stepover * 0.5);
    let dc = DropCutter::new(mesh, cutter).map_err(|err| op_err(op, err))?;
    let angle = if (op.face_angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-9 { RasterAngle::AlongY } else { RasterAngle::AlongX };
    let params = RasterParams { stepover, sample_step, tolerance: job.tolerance().chord(), angle, floor_z: floor, zigzag: true };
    let area_min = (bounds.min().x - margin, bounds.min().y - margin);
    let area_max = (bounds.max().x + margin, bounds.max().y + margin);
    let lines = parallel_finish(&dc, area_min, area_max, &params).map_err(|err| op_err(op, err))?;
    let points = collect_points(op, &dc, &lines, sample_step, floor, stock)?;
    if job.rotary().is_some() { emit_indexed(op, job, &dc, &points) } else { emit_plain(op, job, &points) }
}

/// Every raster sample, misses included. This is the 3-axis surface, kept byte-identical.
fn emit_plain(op: &Operation, job: &JobFile, points: &[MeshVec]) -> Result<Toolpath, CliError> {
    let Some(first) = points.first().copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let Some(last) = points.last().copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let mut moves = Vec::with_capacity(points.len().saturating_add(PATH_OVERHEAD));
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

/// Hits only. A cutter miss splits the polyline: retract and plunge, do not cut the floor chord.
fn emit_indexed(op: &Operation, job: &JobFile, dc: &DropCutter, points: &[MeshVec]) -> Result<Toolpath, CliError> {
    let runs = hit_runs(dc, points);
    let Some(first) = runs.first().and_then(|run| run.first()).copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let Some(last) = runs.last().and_then(|run| run.last()).copied() else {
        return Err(op_err(op, "surface produced no toolpath"));
    };
    let mut moves = Vec::with_capacity(points.len().saturating_add(PATH_OVERHEAD));
    push_move(&mut moves, Move::Rapid { to: point(op, first.x, first.y, job.clearance())? })?;
    push_move(&mut moves, Move::Event(ProcessEvent::On))?;
    push_move(&mut moves, Move::Linear { to: point(op, first.x, first.y, first.z)?, kind: CutKind::Plunge })?;
    let mut previous = first;
    for (run_index, run) in runs.iter().enumerate() {
        for (point_index, sample) in run.iter().copied().enumerate() {
            if run_index == 0 && point_index == 0 {
                continue;
            }
            // A new run starts after a miss. The straight chord would be the floor step the
            // raster inserts where the turned mesh has no contact.
            if point_index == 0 || chord_gouges(dc, previous, sample, job.tolerance().chord()) {
                link_over(op, job, dc, &mut moves, previous, sample)?;
            } else {
                push_move(&mut moves, Move::Linear { to: point(op, sample.x, sample.y, sample.z)?, kind: CutKind::Cut })?;
            }
            previous = sample;
        }
    }
    push_move(&mut moves, Move::Linear { to: point(op, last.x, last.y, job.retract())?, kind: CutKind::Retract })?;
    push_move(&mut moves, Move::Event(ProcessEvent::Off))?;
    push_move(&mut moves, Move::Rapid { to: point(op, last.x, last.y, job.clearance())? })?;
    Toolpath::new(moves).map_err(|err| op_err(op, err))
}

/// Maximal runs of samples where the cutter actually touches the mesh.
fn hit_runs(dc: &DropCutter, points: &[MeshVec]) -> Vec<Vec<MeshVec>> {
    let mut runs = Vec::new();
    let mut current = Vec::new();
    for point in points {
        if dc.tip_z(point.x, point.y).is_none() {
            if !current.is_empty() {
                runs.push(std::mem::take(&mut current));
            }
            continue;
        }
        current.push(*point);
    }
    if !current.is_empty() {
        runs.push(current);
    }
    runs
}

/// Explicit `stepover`, or the flat-floor stepover from `scallop_height`, capped at the diameter.
fn resolve_stepover(op: &Operation, tool: &JobTool, cutter: &Cutter) -> Result<f64, CliError> {
    if let Some(stepover) = op.stepover() {
        return Ok(stepover);
    }
    let Some(height) = op.scallop_height() else {
        return Err(op_err(op, "surface needs a stepover or a scallop_height"));
    };
    let stepover = scallop_stepover(cutter, height).map_err(|err| op_err(op, err))?;
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "surface is not valid with a kerf tool"));
    };
    Ok(stepover.min(diameter))
}

pub(crate) fn cutter(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
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

/// Same spacing as the gouge check: `ceil(length / step) + 1` samples, step `min(0.25, tolerance * 10)`.
fn chord_gouges(dc: &DropCutter, from: MeshVec, to: MeshVec, tolerance: f64) -> bool {
    let step = (tolerance * 10.0).min(0.25);
    if !step.is_finite() || step <= 0.0 || !tolerance.is_finite() || tolerance < 0.0 {
        return false;
    }
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let dz = to.z - from.z;
    let length = (dx * dx + dy * dy + dz * dz).sqrt();
    if !length.is_finite() || length == 0.0 {
        return false;
    }
    let mut intervals = (length / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return true;
    }
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return true;
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > 50_000_000.0 {
        return true;
    }
    let samples = count as usize;
    for index in 0..samples {
        let t = if samples <= 1 { 0.0 } else { index as f64 / (samples - 1) as f64 };
        let z = from.z + dz * t;
        if let Some(contact) = dc.tip_z(from.x + dx * t, from.y + dy * t)
            && contact - z > tolerance
        {
            return true;
        }
    }
    false
}

/// Retract, rapid across at a height the destination can plunge from, then plunge.
///
/// The plunge starts at or above the drop-cutter tip, so the entry does not repeat the gouge.
fn link_over(op: &Operation, job: &JobFile, dc: &DropCutter, moves: &mut Vec<Move>, from: MeshVec, to: MeshVec) -> Result<(), CliError> {
    let retract_z = job.retract().max(from.z);
    let contact = dc.tip_z(to.x, to.y).unwrap_or(to.z);
    let safe_z = retract_z.max(job.clearance()).max(to.z).max(contact);
    if retract_z > from.z {
        push_move(moves, Move::Linear { to: point(op, from.x, from.y, retract_z)?, kind: CutKind::Retract })?;
    }
    let landed = (from.x - to.x).abs() <= f64::EPSILON && (from.y - to.y).abs() <= f64::EPSILON && (retract_z - safe_z).abs() <= f64::EPSILON;
    if !landed {
        push_move(moves, Move::Rapid { to: point(op, to.x, to.y, safe_z)? })?;
    }
    if to.z < safe_z {
        push_move(moves, Move::Linear { to: point(op, to.x, to.y, to.z)?, kind: CutKind::Plunge })?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_mesh::{Triangle, check_gouges};
    use campanion_path::CutKind;

    /// High patch over a low vertex, so the bounding-box floor sits below the surface and the
    /// far corner of the box is a cutter miss.
    fn patch() -> Mesh {
        Mesh::new(vec![
            Triangle { a: MeshVec::new(0.0, 0.0, 0.0), b: MeshVec::new(2.0, 0.0, 0.0), c: MeshVec::new(0.0, 2.0, 0.0) },
            Triangle { a: MeshVec::new(0.0, 0.0, 5.0), b: MeshVec::new(20.0, 0.0, 5.0), c: MeshVec::new(0.0, 20.0, 5.0) },
        ])
        .expect("mesh")
    }

    fn job(rotary: bool) -> JobFile {
        let rotary_json = if rotary { r#","rotary":{"axis":"a","center":[0,0,0],"direction":"positive"}"# } else { "" };
        let index = if rotary { r#","index_deg":90"# } else { "" };
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","heights":{{"clearance":20,"retract":15}}{rotary_json},"tools":[{{"id":1,"diameter":6,"shape":"flat"}}],"operations":[{{"name":"face","type":"surface","tool_id":1,"stepover":6,"sample_step":6{index},"feeds":{{"cut":100,"plunge":50}}}}],"tolerance":{{"chord":0.01}}}}"#
        );
        let dir = std::env::temp_dir().join(if rotary { "campanion-surface-indexed" } else { "campanion-surface-plain" });
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("job.json");
        std::fs::write(&path, text).expect("write job");
        crate::jobfile::load(&path).unwrap_or_else(|err| panic!("job: {err}"))
    }

    fn cutting_tip(mv: &Move) -> Option<Point3> {
        match mv {
            Move::Linear { to, kind: CutKind::Cut | CutKind::Plunge } => Some(*to),
            _ => None,
        }
    }

    #[test]
    fn indexed_surface_skips_misses_and_plain_surface_keeps_them() {
        let mesh = patch();
        let tool_cutter = Cutter::new_flat(3.0).expect("flat");
        let dc = DropCutter::new(&mesh, tool_cutter).expect("index");

        let plain_job = job(false);
        let plain =
            finish(&plain_job, &plain_job.operations()[0], plain_job.tool(1).expect("tool"), &mesh).unwrap_or_else(|err| panic!("plain: {err}"));
        let plain_miss = plain.moves().iter().filter_map(cutting_tip).any(|tip| dc.tip_z(tip.x(), tip.y()).is_none());
        assert!(plain_miss, "3-axis surface dropped a cutter miss");

        let indexed_job = job(true);
        let indexed = finish(&indexed_job, &indexed_job.operations()[0], indexed_job.tool(1).expect("tool"), &mesh)
            .unwrap_or_else(|err| panic!("indexed: {err}"));
        let mut cursor: Option<Point3> = None;
        let mut segments = Vec::new();
        for mv in indexed.moves() {
            match mv {
                Move::Rapid { to } | Move::Linear { to, kind: CutKind::Retract } => cursor = Some(*to),
                Move::Linear { to, kind: CutKind::Cut | CutKind::Plunge } => {
                    assert!(dc.tip_z(to.x(), to.y()).is_some(), "indexed cut at a miss {to:?}");
                    let start = cursor.expect("start");
                    segments.push((MeshVec::new(start.x(), start.y(), start.z()), MeshVec::new(to.x(), to.y(), to.z())));
                    cursor = Some(*to);
                }
                _ => {}
            }
        }
        assert!(!segments.is_empty(), "indexed surface produced no cuts");
        let (count, gouges) = check_gouges(&dc, &segments, 0.1, 0.01, 8).expect("gouge check");
        assert_eq!(count, 0, "indexed surface gouged {gouges:?}");
    }
}
