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
//! The finished-part and fixture checks are unchanged. This output has not been proven on a machine.

use campanion_ir::{ClBlock, ClMotion, Pose, Program};
use campanion_mesh::{Flute, LeftoverReport, Mesh, MeshError, StockHit, StockModel, Vec3 as MeshVec, default_stock_cell};

use crate::collision::poses_along;
use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, OperationKind, StockBounds, ToolAssembly, ToolShape};

/// Overlap deeper than this, in millimetres, is a rapid into stock or a holder hit.
///
/// Shallower overlaps are the dexel quantisation (one cell) and the analytic boundary.
pub(crate) const RAPID_DEPTH_MM: f64 = 1.0e-3;

/// Result of a stock replay. `None` from [`check_job`] means the model was off.
#[derive(Debug)]
pub(crate) struct StockCheck {
    /// Set for an STL part. A drawing has no mesh to measure leftover against.
    pub leftover: Option<LeftoverReport>,
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
pub(crate) fn check_job(
    job: &JobFile,
    part: Option<&Mesh>,
    program: &Program,
    implicit: Option<(MeshVec, MeshVec)>,
) -> Result<Option<StockCheck>, CliError> {
    let Some(model) = replay_model(job, part, program, implicit)? else {
        return Ok(None);
    };
    let cell = model.cell();
    let leftover = part.map(|mesh| model.compare_mesh(mesh, job.tolerance().chord() + cell));
    Ok(Some(StockCheck { leftover }))
}

/// Build the stock and subtract every feed. `None` when the model is off.
fn replay_model(job: &JobFile, part: Option<&Mesh>, program: &Program, implicit: Option<(MeshVec, MeshVec)>) -> Result<Option<StockModel>, CliError> {
    if !job.stock_model_enabled() {
        return Ok(None);
    }
    let (min, max) = stock_box(job, part, implicit)?;
    let cell = job.stock_model().cell().unwrap_or_else(|| default_stock_cell(smallest_radius(job)));
    let mut model = StockModel::from_box(min, max, cell).map_err(stock_err)?;
    for (path_index, path) in program.toolpaths().iter().enumerate() {
        let Some(tool_id) = path.tool_id() else {
            continue;
        };
        let Some(tool) = job.tool(tool_id) else {
            continue;
        };
        replay_path(&mut model, job, path.name().unwrap_or(""), path_index, tool, path.blocks())?;
    }
    Ok(Some(model))
}

fn replay_path(model: &mut StockModel, job: &JobFile, op_name: &str, path_index: usize, tool: &JobTool, blocks: &[ClBlock]) -> Result<(), CliError> {
    let flute = flute_of(tool);
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
        if !rapid && let Some(flute) = flute {
            subtract_feed(model, start, end, motion, flute, &poses)?;
        }
        cursor = Some(end);
        move_index += 1;
    }
    Ok(())
}

fn subtract_feed(model: &mut StockModel, start: Pose, end: Pose, motion: &ClMotion, flute: Flute, poses: &[Pose]) -> Result<(), CliError> {
    let (tip0, axis0) = tip_axis(start)?;
    let (tip1, axis1) = tip_axis(end)?;
    let straight = matches!(motion, ClMotion::Linear { .. } | ClMotion::Thread { .. });
    if straight && axis0.dot(axis1) >= 1.0 - 1e-8 {
        model.subtract_flute_translation(tip0, tip1, axis0, flute).map_err(stock_err)?;
        for pose in poses {
            let (tip, axis) = tip_axis(*pose)?;
            model.subtract_flute_nose(tip, axis, flute).map_err(stock_err)?;
        }
    } else {
        for pose in poses {
            let (tip, axis) = tip_axis(*pose)?;
            model.subtract_flute(tip, axis, flute).map_err(stock_err)?;
        }
    }
    Ok(())
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
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(Some(name.into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program")
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
            ClBlock::Motion(ClMotion::Linear { to: pose(5.0, 5.0, 2.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let err = check_job(&job, None, &program_of("cut", blocks), None).expect_err("holder");
        let text = err.to_string();
        assert!(text.contains("operation `cut` toolpath 0 move 1: holder 1 hits stock at ("), "{text}");
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
