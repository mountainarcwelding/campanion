//! In-process stock check for `campanion run`.
//!
//! When the model is on, every mill toolpath is replayed in order. A rapid whose flute meets the
//! remaining stock by more than [`RAPID_DEPTH_MM`] is an error. One cell of XY quantisation can
//! move a wall by a cell, so that depth allows a grazing touch. With an assembly, the shank, each
//! holder segment and the head are tested the same way, after growing the radius by
//! `collision.clearance`, and the obstacle name is `stock`. Feed moves then subtract the flute.
//!
//! A straight feed with a constant tool axis subtracts the exact swept cylinders and, at the same
//! samples as the 5G check, the ball sphere or bull torus. A rotating axis, an arc or a helix
//! subtracts the whole flute only at those samples. A kerf tool has no flute, so it is skipped.
//! Without an assembly the flute length is the tool diameter.
//!
//! The finished-part and fixture checks are unchanged.
//!
//! Each feed also records the volume removed, the removal rate at the IR feed, the radial
//! engagement and the axial depth. `--stock-report` writes one CSV row per feed. `--stock-out`
//! writes the final height map, and `--stock-snapshots` keeps earlier maps on the same travel
//! axis as the preview. This output has not been proven on a machine.

use campanion_ir::{ArcSense, ClBlock, ClMotion, Plane, Pose, Program};
use campanion_mesh::{Flute, LeftoverReport, Mesh, MeshError, StockFile, StockHit, StockModel, StockSnapshot, Vec3 as MeshVec, default_stock_cell};

use crate::collision::poses_along;
use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, OperationKind, StockBounds, ToolAssembly, ToolShape};

/// Overlap deeper than this, in millimetres, is a rapid into stock or a holder hit.
///
/// Shallower overlaps are the dexel quantisation (one cell) and the analytic boundary.
pub(crate) const RAPID_DEPTH_MM: f64 = 1.0e-3;

/// What to collect besides the leftover summary and the per-operation peaks.
#[derive(Clone, Copy, Debug)]
pub(crate) struct StockWant {
    /// One CSV row per feed move.
    pub csv: bool,
    /// Final height map. Snapshots are stored only when [`Self::snapshot_every`] is set.
    pub file: bool,
    /// A map after every this many motions, plus the uncut map at travel 0. `None` keeps only the final grid.
    pub snapshot_every: Option<usize>,
}

/// Highest removal rate and engagement seen on one operation name.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct StockPeak {
    /// Toolpath name. Empty when the IR omits it.
    pub name: String,
    /// Peak material-removal rate, mm³/min.
    pub peak_mrr: f64,
    /// Peak radial engagement, degrees.
    pub peak_engagement: f64,
}

/// Result of a stock replay. `None` from [`check_job`] means the model was off.
#[derive(Debug)]
pub(crate) struct StockCheck {
    /// Set for an STL part. A drawing has no mesh to measure leftover against.
    pub leftover: Option<LeftoverReport>,
    /// One entry per operation name, in first-seen order.
    pub peaks: Vec<StockPeak>,
    /// CSV text when [`StockWant::csv`] was set.
    pub csv: Option<String>,
    /// Height map when [`StockWant::file`] was set.
    pub file: Option<StockFile>,
}

struct Labeled {
    label: String,
    a: MeshVec,
    b: MeshVec,
    radius: f64,
}

/// Replay `program` when the stock model is enabled.
///
/// `implicit` is the drawing stock box, used only when the file enables the model and omits
/// `stock`. An STL part derives its box from the mesh instead: mesh bounds, with the top grown
/// by the largest rough or waterline margin.
#[cfg(test)]
pub(crate) fn check_job(
    job: &JobFile,
    part: Option<&Mesh>,
    program: &Program,
    implicit: Option<(MeshVec, MeshVec)>,
) -> Result<Option<StockCheck>, CliError> {
    check_job_with(job, part, program, implicit, StockWant { csv: false, file: false, snapshot_every: None })
}

/// Replay like [`check_job`], and also collect the report, the height map, or both.
pub(crate) fn check_job_with(
    job: &JobFile,
    part: Option<&Mesh>,
    program: &Program,
    implicit: Option<(MeshVec, MeshVec)>,
    want: StockWant,
) -> Result<Option<StockCheck>, CliError> {
    let Some(replay) = replay(job, part, program, implicit, want)? else {
        return Ok(None);
    };
    let cell = replay.model.cell();
    let leftover = part.map(|mesh| replay.model.compare_mesh(mesh, job.tolerance().chord() + cell));
    let file = if want.file { Some(StockFile { grid: replay.model.stock_grid(), length: replay.length, snapshots: replay.snapshots }) } else { None };
    let csv = if want.csv { Some(write_csv(&replay.rows)) } else { None };
    Ok(Some(StockCheck { leftover, peaks: replay.peaks, csv, file }))
}

/// Build the stock and subtract every feed. `None` when the model is off.
#[cfg(test)]
fn replay_model(job: &JobFile, part: Option<&Mesh>, program: &Program, implicit: Option<(MeshVec, MeshVec)>) -> Result<Option<StockModel>, CliError> {
    Ok(replay(job, part, program, implicit, StockWant { csv: false, file: false, snapshot_every: None })?.map(|out| out.model))
}

struct Replay {
    model: StockModel,
    rows: Vec<FeedRow>,
    peaks: Vec<StockPeak>,
    length: f64,
    snapshots: Vec<StockSnapshot>,
}

struct FeedRow {
    operation: String,
    toolpath: usize,
    move_index: usize,
    volume: f64,
    mrr: f64,
    engagement_deg: f64,
    axial_doc: f64,
}

struct Travel {
    tip: Option<MeshVec>,
    length: f64,
    motions: usize,
}

fn replay(
    job: &JobFile,
    part: Option<&Mesh>,
    program: &Program,
    implicit: Option<(MeshVec, MeshVec)>,
    want: StockWant,
) -> Result<Option<Replay>, CliError> {
    if !job.stock_model_enabled() {
        return Ok(None);
    }
    let (min, max) = stock_box(job, part, implicit)?;
    let cell = job.stock_model().cell().unwrap_or_else(|| default_stock_cell(smallest_radius(job)));
    let mut model = StockModel::from_box(min, max, cell).map_err(stock_err)?;
    let mut rows = Vec::new();
    let mut peaks: Vec<StockPeak> = Vec::new();
    let mut travel = Travel { tip: None, length: 0.0, motions: 0 };
    let mut snapshots = Vec::new();
    if want.snapshot_every.is_some() {
        snapshots.push(snapshot_of(&model, 0.0));
    }
    for (path_index, path) in program.toolpaths().iter().enumerate() {
        let name = path.name().unwrap_or("");
        let tool = path.tool_id().and_then(|id| job.tool(id));
        replay_path(&mut model, &mut rows, &mut peaks, &mut travel, &mut snapshots, job, name, path_index, tool, path.blocks(), want)?;
    }
    Ok(Some(Replay { model, rows, peaks, length: travel.length, snapshots }))
}

fn replay_path(
    model: &mut StockModel,
    rows: &mut Vec<FeedRow>,
    peaks: &mut Vec<StockPeak>,
    travel: &mut Travel,
    snapshots: &mut Vec<StockSnapshot>,
    job: &JobFile,
    op_name: &str,
    path_index: usize,
    tool: Option<&JobTool>,
    blocks: &[ClBlock],
    want: StockWant,
) -> Result<(), CliError> {
    let flute = tool.and_then(flute_of);
    let settings = job.collision();
    let mut cursor: Option<Pose> = None;
    let mut move_index = 0usize;
    for block in blocks {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        let end = motion_pose(motion);
        let start = cursor.unwrap_or(end);
        let rapid = matches!(motion, ClMotion::Rapid { .. });
        if let Some(tool) = tool {
            let poses = poses_along(start, end, motion, settings)?;
            if rapid
                && let Some(flute) = flute
                && let Some(hit) = flute_hit(model, &poses, flute)?
            {
                return Err(CliError::run(format_rapid(op_name, path_index, move_index, hit)));
            }
            if let Some(assembly) = tool.assembly()
                && let Some((label, hit)) = assembly_hit(model, &poses, assembly, settings.clearance())?
            {
                return Err(CliError::run(format_holder(op_name, path_index, move_index, &label, hit)));
            }
            if !rapid {
                let (volume, measure) = if let Some(flute) = flute {
                    let measure = measure_move(model, start, end, flute)?;
                    let volume = subtract_feed(model, start, end, motion, flute, &poses)?;
                    (volume, measure)
                } else {
                    (0.0, campanion_mesh::CutMeasure { engagement_deg: 0.0, axial_doc: 0.0 })
                };
                let length = move_length(start, end, motion)?;
                let rate = removal_rate(volume, feed_mm_per_min(motion), length);
                note_peak(peaks, op_name, rate, measure.engagement_deg);
                if want.csv {
                    rows.push(FeedRow {
                        operation: op_name.to_string(),
                        toolpath: path_index,
                        move_index,
                        volume,
                        mrr: rate,
                        engagement_deg: measure.engagement_deg,
                        axial_doc: measure.axial_doc,
                    });
                }
            }
        }
        cursor = Some(end);
        advance_travel(travel, end, motion)?;
        if let Some(every) = want.snapshot_every
            && every > 0
            && travel.motions.is_multiple_of(every)
        {
            snapshots.push(snapshot_of(model, travel.length));
        }
        move_index += 1;
    }
    Ok(())
}

/// Engagement and axial depth at the start, the middle and the end, before the cut.
fn measure_move(model: &StockModel, start: Pose, end: Pose, flute: Flute) -> Result<campanion_mesh::CutMeasure, CliError> {
    let (tip0, axis0) = tip_axis(start)?;
    let (tip1, axis1) = tip_axis(end)?;
    let feed = tip1.sub(tip0);
    let mut engagement_deg = 0.0_f64;
    let mut axial_doc = 0.0_f64;
    for t in [0.0, 0.5, 1.0] {
        let tip = lerp_mesh(tip0, tip1, t);
        let axis = lerp_mesh(axis0, axis1, t);
        let sample = model.measure_flute(tip, axis, flute, feed);
        engagement_deg = engagement_deg.max(sample.engagement_deg);
        axial_doc = axial_doc.max(sample.axial_doc);
    }
    Ok(campanion_mesh::CutMeasure { engagement_deg, axial_doc })
}

fn subtract_feed(model: &mut StockModel, start: Pose, end: Pose, motion: &ClMotion, flute: Flute, poses: &[Pose]) -> Result<f64, CliError> {
    let (tip0, axis0) = tip_axis(start)?;
    let (tip1, axis1) = tip_axis(end)?;
    let straight = matches!(motion, ClMotion::Linear { .. } | ClMotion::Thread { .. });
    let mut volume = 0.0;
    if straight && axis0.dot(axis1) >= 1.0 - 1e-8 {
        volume += model.subtract_flute_translation(tip0, tip1, axis0, flute).map_err(stock_err)?;
        for pose in poses {
            let (tip, axis) = tip_axis(*pose)?;
            volume += model.subtract_flute_nose(tip, axis, flute).map_err(stock_err)?;
        }
    } else {
        for pose in poses {
            let (tip, axis) = tip_axis(*pose)?;
            volume += model.subtract_flute(tip, axis, flute).map_err(stock_err)?;
        }
    }
    Ok(if volume.is_finite() { volume } else { 0.0 })
}

fn removal_rate(volume: f64, feed: f64, length: f64) -> f64 {
    if length > 1e-9 && feed > 0.0 && volume.is_finite() {
        let rate = volume * feed / length;
        if rate.is_finite() { rate } else { 0.0 }
    } else {
        0.0
    }
}

fn feed_mm_per_min(motion: &ClMotion) -> f64 {
    match motion {
        ClMotion::Linear { feed, .. } | ClMotion::Arc { feed, .. } | ClMotion::Helix { feed, .. } => {
            if feed.is_finite() && *feed > 0.0 {
                *feed
            } else {
                0.0
            }
        }
        ClMotion::Rapid { .. } | ClMotion::Thread { .. } => 0.0,
    }
}

fn note_peak(peaks: &mut Vec<StockPeak>, name: &str, mrr: f64, engagement: f64) {
    if let Some(peak) = peaks.iter_mut().find(|peak| peak.name == name) {
        peak.peak_mrr = peak.peak_mrr.max(mrr);
        peak.peak_engagement = peak.peak_engagement.max(engagement);
    } else {
        peaks.push(StockPeak { name: name.to_string(), peak_mrr: mrr, peak_engagement: engagement });
    }
}

fn snapshot_of(model: &StockModel, position: f64) -> StockSnapshot {
    let grid = model.stock_grid();
    StockSnapshot { position, top: grid.top, multi: grid.multi }
}

/// Program-wide travel, matching the preview: the first motion only places the cursor.
fn advance_travel(travel: &mut Travel, end: Pose, motion: &ClMotion) -> Result<(), CliError> {
    let (tip, _) = tip_axis(end)?;
    if let Some(start) = travel.tip {
        let step = motion_length(start, tip, motion)?;
        if step.is_finite() && step > 0.0 {
            travel.length += step;
        }
    }
    travel.tip = Some(tip);
    travel.motions += 1;
    Ok(())
}

/// Length of the stroke the preview draws for this motion.
///
/// A straight move is the 3-D tip distance. An XY arc or helix is the analytic arc length
/// (`radius * sweep`, with the height in the hypotenuse). The preview shortens that arc by less
/// than one chord. Any other plane is the tip distance.
fn motion_length(start: MeshVec, end: MeshVec, motion: &ClMotion) -> Result<f64, CliError> {
    let length = match motion {
        ClMotion::Arc { plane: Plane::Xy, center, sense, .. } => xy_arc_length(start, end, center, matches!(sense, ArcSense::Ccw), None),
        ClMotion::Helix { plane: Plane::Xy, center, sense, sweep, .. } => {
            xy_arc_length(start, end, center, matches!(sense, ArcSense::Ccw), Some(*sweep))
        }
        _ => tip_distance(start, end),
    };
    Ok(if length.is_finite() { length } else { 0.0 })
}

/// Geometric length used for the removal rate. A zero-length placement stays zero.
fn move_length(start: Pose, end: Pose, motion: &ClMotion) -> Result<f64, CliError> {
    let (tip0, _) = tip_axis(start)?;
    let (tip1, _) = tip_axis(end)?;
    motion_length(tip0, tip1, motion)
}

fn tip_distance(start: MeshVec, end: MeshVec) -> f64 {
    let length = end.sub(start).length();
    if length.is_finite() { length } else { 0.0 }
}

fn xy_arc_length(start: MeshVec, end: MeshVec, center: &campanion_ir::Vec3, ccw: bool, sweep: Option<f64>) -> f64 {
    let sx = start.x - center.x();
    let sy = start.y - center.y();
    let ex = end.x - center.x();
    let ey = end.y - center.y();
    let r0 = sx.hypot(sy);
    let r1 = ex.hypot(ey);
    if !r0.is_finite() || !r1.is_finite() || r0 <= 1e-12 {
        return tip_distance(start, end);
    }
    let radius = (r0 + r1) * 0.5;
    let sweep = match sweep {
        Some(sweep) if sweep.is_finite() && sweep > 0.0 => sweep,
        _ => match one_turn_sweep(sx, sy, ex, ey, r0, ccw) {
            Some(sweep) => sweep,
            None => return tip_distance(start, end),
        },
    };
    let length = (radius * sweep).hypot(end.z - start.z);
    if length.is_finite() { length } else { tip_distance(start, end) }
}

/// Sweep of one XY arc, at most one full turn. A closed chord is a full turn.
fn one_turn_sweep(sx: f64, sy: f64, ex: f64, ey: f64, radius: f64, ccw: bool) -> Option<f64> {
    let a0 = sy.atan2(sx);
    let a1 = ey.atan2(ex);
    if !a0.is_finite() || !a1.is_finite() {
        return None;
    }
    let chord = (ex - sx).hypot(ey - sy);
    let closure = 1e-9_f64.max(radius * 1e-9);
    let ccw_delta = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    let sweep = if chord <= closure {
        std::f64::consts::TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { std::f64::consts::TAU } else { ccw_delta }
    } else {
        let cw = std::f64::consts::TAU - ccw_delta;
        if cw <= 1e-15 { std::f64::consts::TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Some(sweep) } else { None }
}

fn lerp_mesh(start: MeshVec, end: MeshVec, t: f64) -> MeshVec {
    start.add(end.sub(start).scale(t))
}

fn write_csv(rows: &[FeedRow]) -> String {
    let mut out = String::from("operation,toolpath,move,volume,mrr,engagement_deg,axial_doc\n");
    for row in rows {
        out.push_str(&csv_field(&row.operation));
        out.push(',');
        out.push_str(&row.toolpath.to_string());
        out.push(',');
        out.push_str(&row.move_index.to_string());
        out.push(',');
        out.push_str(&csv_num(row.volume));
        out.push(',');
        out.push_str(&csv_num(row.mrr));
        out.push(',');
        out.push_str(&csv_num(row.engagement_deg));
        out.push(',');
        out.push_str(&csv_num(row.axial_doc));
        out.push('\n');
    }
    out
}

fn csv_field(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) { format!("\"{}\"", text.replace('"', "\"\"")) } else { text.to_string() }
}

fn csv_num(value: f64) -> String {
    if value.is_finite() { format!("{value}") } else { "0".to_string() }
}

fn flute_hit(model: &StockModel, poses: &[Pose], flute: Flute) -> Result<Option<StockHit>, CliError> {
    for pose in poses {
        let (tip, axis) = tip_axis(*pose)?;
        if let Some(hit) = model.query_flute(tip, axis, flute)
            && hit.depth > RAPID_DEPTH_MM
        {
            return Ok(Some(hit));
        }
    }
    Ok(None)
}

fn assembly_hit(model: &StockModel, poses: &[Pose], assembly: &ToolAssembly, clearance: f64) -> Result<Option<(String, StockHit)>, CliError> {
    for pose in poses {
        let (tip, axis) = tip_axis(*pose)?;
        for solid in assembly_solids(tip, axis, assembly, clearance)? {
            if let Some(hit) = model.query_cylinder(solid.a, solid.b, solid.radius)
                && hit.depth > RAPID_DEPTH_MM
            {
                return Ok(Some((solid.label, hit)));
            }
        }
    }
    Ok(None)
}

fn assembly_solids(tip: MeshVec, axis: MeshVec, assembly: &ToolAssembly, clearance: f64) -> Result<Vec<Labeled>, CliError> {
    let mut solids = Vec::new();
    let flute = assembly.flute_length();
    let stickout = assembly.stickout();
    if stickout - flute > 1e-12 {
        let radius = assembly.shank_diameter() * 0.5 + clearance;
        push_solid(&mut solids, "shank", tip.add(axis.scale(flute)), tip.add(axis.scale(stickout)), radius);
    }
    let mut along = stickout;
    for (index, segment) in assembly.holder().iter().enumerate() {
        let radius = segment.diameter() * 0.5 + clearance;
        let label = format!("holder {}", index + 1);
        push_solid(&mut solids, &label, tip.add(axis.scale(along)), tip.add(axis.scale(along + segment.length())), radius);
        along += segment.length();
    }
    if let Some(head) = assembly.head() {
        let radius = head.diameter() * 0.5 + clearance;
        push_solid(&mut solids, "head", tip.add(axis.scale(along)), tip.add(axis.scale(along + head.length())), radius);
    }
    Ok(solids)
}

fn push_solid(solids: &mut Vec<Labeled>, label: &str, a: MeshVec, b: MeshVec, radius: f64) {
    if b.sub(a).length() > 1e-12 && radius >= 0.0 && radius.is_finite() {
        solids.push(Labeled { label: label.to_string(), a, b, radius });
    }
}

fn flute_of(tool: &JobTool) -> Option<Flute> {
    let radius = tool.diameter()? * 0.5;
    if !(radius.is_finite() && radius >= 0.0) {
        return None;
    }
    let length = tool.assembly().map(ToolAssembly::flute_length).unwrap_or(radius * 2.0);
    Some(match tool.shape() {
        ToolShape::Flat => Flute::Flat { radius, length },
        ToolShape::Ball => Flute::Ball { radius, length },
        ToolShape::Bull { corner_radius } => Flute::Bull { radius, corner: corner_radius, length },
    })
}

fn stock_box(job: &JobFile, part: Option<&Mesh>, implicit: Option<(MeshVec, MeshVec)>) -> Result<(MeshVec, MeshVec), CliError> {
    if let Some(stock) = job.stock() {
        return Ok(bounds_vec(stock));
    }
    if let Some(mesh) = part {
        return implicit_mesh_box(job, mesh);
    }
    implicit.ok_or_else(|| CliError::run("stock model needs a stock box"))
}

fn bounds_vec(stock: StockBounds) -> (MeshVec, MeshVec) {
    let min = stock.min();
    let max = stock.max();
    (MeshVec::new(min.x(), min.y(), min.z()), MeshVec::new(max.x(), max.y(), max.z()))
}

fn implicit_mesh_box(job: &JobFile, mesh: &Mesh) -> Result<(MeshVec, MeshVec), CliError> {
    let bounds = mesh.bounds().map_err(|err| CliError::run(format!("stock: {err}")))?;
    let mut margin = 0.0_f64;
    for operation in job.operations() {
        if matches!(operation.kind(), OperationKind::Rough | OperationKind::Waterline) {
            margin = margin.max(operation.margin());
        }
    }
    let min = bounds.min();
    let max = bounds.max();
    let top = max.z + margin;
    if !(top > min.z) {
        return Err(CliError::run("stock model needs a stock box"));
    }
    Ok((min, MeshVec::new(max.x, max.y, top)))
}

fn smallest_radius(job: &JobFile) -> f64 {
    job.tools().iter().filter_map(|tool| tool.diameter()).map(|diameter| diameter * 0.5).fold(f64::INFINITY, f64::min)
}

fn motion_pose(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    }
}

fn tip_axis(pose: Pose) -> Result<(MeshVec, MeshVec), CliError> {
    let tip = pose.tip();
    let axis = pose.axis();
    let axis = MeshVec::new(axis.x(), axis.y(), axis.z()).normalized().ok_or_else(|| CliError::run("stock: tool axis is not usable"))?;
    Ok((MeshVec::new(tip.x(), tip.y(), tip.z()), axis))
}

fn format_rapid(op_name: &str, path_index: usize, move_index: usize, hit: StockHit) -> String {
    format!(
        "operation `{op_name}` toolpath {path_index} move {move_index}: rapid into stock at ({:.3}, {:.3}, {:.3}), depth {:.3}",
        hit.point.x, hit.point.y, hit.point.z, hit.depth
    )
}

fn format_holder(op_name: &str, path_index: usize, move_index: usize, body: &str, hit: StockHit) -> String {
    format!(
        "operation `{op_name}` toolpath {path_index} move {move_index}: {body} hits stock at ({:.3}, {:.3}, {:.3})",
        hit.point.x, hit.point.y, hit.point.z
    )
}

fn stock_err(err: MeshError) -> CliError {
    match err {
        MeshError::InvalidInput(message) => CliError::run(format!("stock: {message}")),
        other => CliError::run(format!("stock: {other}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{ClPath, CompensationMode, Document, Feeds, Import, MoveRole, ProcessKind, Tool, Vec3};
    use campanion_mesh::{Triangle, Vec3 as MeshVec};
    use proptest::prelude::*;

    fn ok<T>(result: Result<T, CliError>) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{err}"),
        }
    }

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn program_of(name: &str, blocks: Vec<ClBlock>) -> Program {
        program_paths(&[(name, blocks)])
    }

    fn program_paths(paths: &[(&str, Vec<ClBlock>)]) -> Program {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let built = paths
            .iter()
            .map(|(name, blocks)| {
                ClPath::try_new(Some((*name).into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks.clone()).expect("path")
            })
            .collect();
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        Program::try_new(None, vec![tool], None, built, Vec::new(), Vec::new(), None).expect("program")
    }

    fn load_job(name: &str, text: &str) -> JobFile {
        use std::sync::atomic::{AtomicU64, Ordering};
        static N: AtomicU64 = AtomicU64::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("campanion-stock-{name}-{n}"));
        std::fs::create_dir_all(&dir).expect("dir");
        let file = dir.join("job.json");
        std::fs::write(&file, text).expect("write");
        crate::jobfile::load(&file).expect("load")
    }

    fn stock_job(extra: &str) -> JobFile {
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","tools":[{{"id":1,"diameter":6,"shape":"flat"}}],"operations":[{{"name":"cut","type":"surface","tool_id":1,"stepover":4,"feeds":{{"cut":100}}}}],"stock":{{"box":{{"min":[0,0,0],"max":[10,10,10]}}}}{extra}}}"#
        );
        load_job("box", &text)
    }

    fn rapid_blocks(z0: f64, z1: f64) -> Vec<ClBlock> {
        vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 5.0, z0), feed: None }),
            ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 5.0, z1), feed: None }),
        ]
    }

    #[test]
    fn rapid_through_the_box_names_the_move_and_one_above_does_not() {
        let job = stock_job("");
        let program = program_of("cut", rapid_blocks(30.0, 2.0));
        let err = check_job(&job, None, &program, None).expect_err("rapid");
        let text = err.to_string();
        assert!(text.contains("operation `cut` toolpath 0 move 1: rapid into stock at ("), "{text}");
        assert!(text.contains(", depth "), "{text}");
        let clear = program_of("cut", rapid_blocks(30.0, 20.0));
        let ran = ok(check_job(&job, None, &clear, None));
        assert!(ran.expect("enabled").leftover.is_none());
    }

    #[test]
    fn holder_hit_names_stock() {
        let text = concat!(
            r#"{"format":"campanion-job/1","source":{"path":"part.stl","hash":"cbf29ce484222325"},"units":"mm","#,
            r#""tools":[{"id":1,"diameter":6,"shape":"flat","assembly":{"stickout":4,"flute_length":4,"holder":[{"diameter":20,"length":12}]}}],"#,
            r#""operations":[{"name":"cut","type":"surface","tool_id":1,"stepover":4,"feeds":{"cut":100}}],"#,
            r#""stock":{"box":{"min":[0,0,0],"max":[10,10,20]}}}"#
        );
        let job = load_job("holder", text);
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 5.0, 40.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(5.0, 5.0, 2.0), feed: 100.0, role: MoveRole::Cut, power: None }),
        ];
        let err = check_job(&job, None, &program_of("cut", blocks), None).expect_err("holder");
        let text = err.to_string();
        assert!(text.contains("operation `cut` toolpath 0 move 1: holder 1 hits stock at ("), "{text}");
    }

    #[test]
    fn the_report_has_one_row_per_feed_move_and_the_map_round_trips() {
        let job = stock_job(r#","stock_model":{"cell":1}"#);
        let cut = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(2.0, 2.0, 30.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(2.0, 2.0, 6.0), feed: 120.0, role: MoveRole::Plunge, power: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(8.0, 2.0, 6.0), feed: 200.0, role: MoveRole::Cut, power: None }),
            ClBlock::Motion(ClMotion::Rapid { to: pose(8.0, 2.0, 30.0), feed: None }),
        ];
        let again = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(8.0, 6.0, 30.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(8.0, 6.0, 6.0), feed: 80.0, role: MoveRole::Cut, power: None }),
        ];
        let program = program_paths(&[("cut", cut), ("finish", again)]);
        let want = StockWant { csv: true, file: true, snapshot_every: Some(2) };
        let check = ok(check_job_with(&job, None, &program, None, want)).expect("on");
        let csv = check.csv.expect("csv");
        let lines: Vec<&str> = csv.lines().filter(|line| !line.is_empty()).collect();
        assert_eq!(lines[0], "operation,toolpath,move,volume,mrr,engagement_deg,axial_doc");
        let data = &lines[1..];
        assert_eq!(data.len(), 3, "one row per feed, rapids omitted: {csv}");
        assert!(data[0].starts_with("cut,0,1,"), "{csv}");
        assert!(data[1].starts_with("cut,0,2,"), "{csv}");
        assert!(data[2].starts_with("finish,1,1,"), "{csv}");
        assert!(check.peaks.iter().any(|peak| peak.name == "cut" && peak.peak_mrr > 0.0), "{:?}", check.peaks);
        assert!(check.peaks.iter().any(|peak| peak.name == "finish"), "{:?}", check.peaks);
        let file = check.file.expect("map");
        let parsed = campanion_mesh::StockFile::from_json(&file.to_json().expect("json")).expect("parse");
        assert_eq!(parsed.grid.nx, file.grid.nx);
        assert_eq!(parsed.grid.ny, file.grid.ny);
        assert_eq!(parsed.snapshots.len(), file.snapshots.len());
        assert!(parsed.snapshots.len() >= 2, "uncut frame plus later frames");
        let mut previous = -1.0_f64;
        for frame in &parsed.snapshots {
            assert!(frame.position + 1e-9 >= previous, "snapshot positions increase");
            previous = frame.position;
        }
        assert!(parsed.length + 1e-9 >= previous);
    }

    #[test]
    fn disabled_model_ignores_a_rapid_in_the_box() {
        let job = stock_job(r#","stock_model":{"enabled":false}"#);
        let program = program_of("cut", rapid_blocks(30.0, 2.0));
        assert!(ok(check_job(&job, None, &program, None)).is_none());
    }

    #[test]
    fn too_many_cells_names_a_larger_cell() {
        let job = stock_job(r#","stock_model":{"cell":0.001}"#);
        let program = program_of("cut", rapid_blocks(30.0, 20.0));
        let err = check_job(&job, None, &program, None).expect_err("cap");
        let text = err.to_string();
        assert!(text.contains("stock model has"), "{text}");
        assert!(text.contains("use a larger stock_model.cell"), "{text}");
    }

    #[test]
    fn surface_and_rough_agree_with_gouge_and_stock_to_leave() {
        let mesh = gentle_dome(8, 30.0, 3.0);
        let bounds = mesh.bounds().expect("bounds");
        let cell = 1.0;
        let stock = format!(
            r#""stock":{{"box":{{"min":[{:.3},{:.3},-1],"max":[{:.3},{:.3},{:.3}]}}}},"stock_model":{{"cell":{cell}}}"#,
            bounds.min().x - 1.0,
            bounds.min().y - 1.0,
            bounds.max().x + 1.0,
            bounds.max().y + 1.0,
            bounds.max().z
        );
        let clean = surface_job(&stock, 0.0, 2.0);
        let (gouges, report) = run_surface(&clean, &mesh, &mesh);
        assert!(gouges.is_empty(), "clean gouges {}", gouges.len());
        let report = report.expect("summary");
        assert_eq!(report.model_gouges, 0, "max leftover {}", report.max_leftover);

        let mut raised_tris = Vec::new();
        for triangle in mesh.triangles() {
            raised_tris.push(Triangle {
                a: MeshVec::new(triangle.a.x, triangle.a.y, triangle.a.z + 2.0),
                b: MeshVec::new(triangle.b.x, triangle.b.y, triangle.b.z + 2.0),
                c: MeshVec::new(triangle.c.x, triangle.c.y, triangle.c.z + 2.0),
            });
        }
        let raised = Mesh::new(raised_tris).expect("raised");
        // The same cutter locations, compared with a mesh raised by more than chord + cell.
        let (bad_gouges, bad_report) = run_surface(&clean, &mesh, &raised);
        assert!(!bad_gouges.is_empty(), "raised mesh should gouge");
        assert!(bad_report.expect("bad").model_gouges > 0);

        let plate = plate_mesh(20.0, 0.0);
        let leave = 0.5;
        let leaving = surface_job(r#""stock":{"box":{"min":[0,0,-1],"max":[20,20,4]}},"stock_model":{"cell":1}"#, leave, 2.0);
        let model = run_model(&leaving, &plate);
        let touched = min_touched_leftover(&model, &plate);
        assert!(touched.samples > 0, "the ball did not touch the plate");
        assert!(touched.value + 1e-6 >= leave - 0.05, "min leftover {} on {} cells", touched.value, touched.samples);

        let blank = plate_mesh(16.0, 4.0);
        let rough_stock = r#""stock":{"box":{"min":[0,0,0],"max":[16,16,8]}},"stock_model":{"cell":1}"#;
        let rough_job = rough_job(rough_stock);
        // The rim of the rectangle stays uncut. The interior is what the flat swept.
        let interior = plate_rect(5.0, 11.0, 4.0);
        let report = run_rough(&rough_job, &blank, &interior).expect("rough");
        assert_eq!(report.model_gouges, 0, "rough gouges {}", report.model_gouges);
        assert!(report.samples > 0);
        assert!(report.max_leftover <= 2.0 + 0.25, "rough leftover {}", report.max_leftover);
    }

    fn surface_job(stock: &str, leave: f64, stepover: f64) -> JobFile {
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","heights":{{"clearance":30,"retract":20}},"tolerance":{{"linear":0.05,"angular":0.05,"chord":0.05}},"tools":[{{"id":1,"diameter":6,"shape":"ball"}}],"operations":[{{"name":"cap","type":"surface","tool_id":1,"stepover":{stepover},"stock_to_leave":{leave},"margin":0,"feeds":{{"cut":100}}}}],{stock}}}"#
        );
        load_job(&format!("surf-{leave}"), &text)
    }

    fn rough_job(stock: &str) -> JobFile {
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","heights":{{"clearance":20,"retract":12}},"tolerance":{{"linear":0.05,"angular":0.05,"chord":0.05}},"tools":[{{"id":1,"diameter":6,"shape":"flat"}}],"operations":[{{"name":"hog","type":"rough","tool_id":1,"step_down":2,"stepover":3,"stock_to_leave":0.3,"margin":0,"top_z":8,"floor_z":4,"grid":1,"feeds":{{"cut":100}}}}],{stock}}}"#
        );
        load_job("rough", &text)
    }

    fn run_surface(job: &JobFile, plan: &Mesh, compare: &Mesh) -> (Vec<crate::report::GougeNote>, Option<LeftoverReport>) {
        let op = &job.operations()[0];
        let tool = job.tool(1).expect("tool");
        let crate::surface::SurfacePaths::Plain(path) = ok(crate::surface::finish(job, op, tool, plan)) else {
            panic!("tilted");
        };
        let doc = document_for(op, tool, &path);
        let gouges = ok(crate::gouge::check_job(job, compare, doc.program()));
        let check = ok(check_job(job, Some(compare), doc.program(), None));
        (gouges, check.expect("on").leftover)
    }

    fn run_model(job: &JobFile, mesh: &Mesh) -> StockModel {
        let op = &job.operations()[0];
        let tool = job.tool(1).expect("tool");
        let crate::surface::SurfacePaths::Plain(path) = ok(crate::surface::finish(job, op, tool, mesh)) else {
            panic!("tilted");
        };
        let doc = document_for(op, tool, &path);
        ok(replay_model(job, Some(mesh), doc.program(), None)).expect("on")
    }

    fn run_rough(job: &JobFile, plan: &Mesh, compare: &Mesh) -> Option<LeftoverReport> {
        let op = &job.operations()[0];
        let tool = job.tool(1).expect("tool");
        let path = ok(crate::rough::rough(job, op, tool, plan));
        let doc = document_for(op, tool, &path);
        ok(check_job(job, Some(compare), doc.program(), None)).expect("on").leftover
    }

    struct Touch {
        value: f64,
        samples: u64,
    }

    /// Smallest leftover on cells whose top dropped, which is where the ball met the stock.
    fn min_touched_leftover(model: &StockModel, mesh: &Mesh) -> Touch {
        let (_low, top_z) = model.z_bounds();
        let (nx, ny) = model.dimensions();
        let mut value = f64::MAX;
        let mut samples = 0_u64;
        for iy in 0..ny {
            for ix in 0..nx {
                let Some(top) = model.height_at(ix, iy) else {
                    continue;
                };
                if top > top_z - 1e-3 {
                    continue;
                }
                let Some(center) = model.cell_center(ix, iy) else {
                    continue;
                };
                let Some(surface) = surface_z(mesh, center.x, center.y) else {
                    continue;
                };
                let Some(spans) = model.column(ix, iy) else {
                    continue;
                };
                let leftover = spans
                    .iter()
                    .find(|span| span.lo <= surface + 1e-9 && span.hi >= surface - 1e-9)
                    .map(|span| (span.hi - surface).max(0.0))
                    .unwrap_or(0.0);
                value = value.min(leftover);
                samples += 1;
            }
        }
        Touch { value, samples }
    }

    fn surface_z(mesh: &Mesh, x: f64, y: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        for triangle in mesh.triangles() {
            let det = (triangle.b.y - triangle.c.y) * (triangle.a.x - triangle.c.x) + (triangle.c.x - triangle.b.x) * (triangle.a.y - triangle.c.y);
            if det.abs() < 1e-14 {
                continue;
            }
            let w1 = ((triangle.b.y - triangle.c.y) * (x - triangle.c.x) + (triangle.c.x - triangle.b.x) * (y - triangle.c.y)) / det;
            let w2 = ((triangle.c.y - triangle.a.y) * (x - triangle.c.x) + (triangle.a.x - triangle.c.x) * (y - triangle.c.y)) / det;
            let w3 = 1.0 - w1 - w2;
            if w1 >= -1e-8 && w2 >= -1e-8 && w3 >= -1e-8 {
                let z = w1 * triangle.a.z + w2 * triangle.b.z + w3 * triangle.c.z;
                best = Some(best.map_or(z, |current| current.max(z)));
            }
        }
        best
    }

    fn document_for(op: &crate::jobfile::Operation, tool: &JobTool, path: &campanion_path::Toolpath) -> Document {
        let import = Import::new(ProcessKind::Spindle, op.feeds().cut())
            .expect("import")
            .feeds(op.feeds())
            .expect("feeds")
            .tool(tool.ir().clone())
            .expect("tool")
            .name(op.name())
            .expect("name");
        Document::from_toolpath(path, &import).expect("doc")
    }

    fn gentle_dome(n: usize, radius: f64, height: f64) -> Mesh {
        let zc = height - radius;
        let base = (radius * radius - zc * zc).sqrt();
        let mut triangles = Vec::new();
        let point = |i: usize, j: usize| {
            let x = -base + 2.0 * base * i as f64 / n as f64;
            let y = -base + 2.0 * base * j as f64 / n as f64;
            let radial = x * x + y * y;
            let z = if radial >= radius * radius { 0.0 } else { zc + (radius * radius - radial).sqrt() };
            MeshVec::new(x, y, z.max(0.0))
        };
        for j in 0..n {
            for i in 0..n {
                let a = point(i, j);
                let b = point(i + 1, j);
                let c = point(i, j + 1);
                let d = point(i + 1, j + 1);
                triangles.push(Triangle { a, b, c });
                triangles.push(Triangle { a: b, b: d, c });
            }
        }
        Mesh::new(triangles).expect("dome")
    }

    fn plate_mesh(size: f64, z: f64) -> Mesh {
        plate_rect(0.0, size, z)
    }

    fn plate_rect(min: f64, max: f64, z: f64) -> Mesh {
        Mesh::new(vec![
            Triangle { a: MeshVec::new(min, min, z), b: MeshVec::new(max, min, z), c: MeshVec::new(max, max, z) },
            Triangle { a: MeshVec::new(min, min, z), b: MeshVec::new(max, max, z), c: MeshVec::new(min, max, z) },
        ])
        .expect("plate")
    }

    fn stock_config() -> ProptestConfig {
        ProptestConfig { cases: 8, max_shrink_iters: 20, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_111D), ..ProptestConfig::default() }
    }

    proptest! {
        #![proptest_config(stock_config())]
        #[test]
        fn a_random_rapid_through_the_box_is_caught_and_one_above_is_not(x in 1.0..9.0_f64, y in 1.0..9.0_f64) {
            let job = stock_job("");
            let through = vec![
                ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, 30.0), feed: None }),
                ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, 1.0), feed: None }),
            ];
            let err = check_job(&job, None, &program_of("cut", through), None).expect_err("through");
            prop_assert!(err.to_string().contains("rapid into stock"));
            let above = vec![
                ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, 30.0), feed: None }),
                ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, 22.0), feed: None }),
            ];
            prop_assert!(ok(check_job(&job, None, &program_of("cut", above), None)).is_some());
        }
    }
}
