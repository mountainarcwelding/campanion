//! `run`: load a `campanion-job/1` file, import its DXF, and write toolpaths.
//!
//! The source path is relative to the job file. A hash mismatch is a warning on stderr; the
//! bytes just read are imported and the job file is not rewritten. Each operation keeps the
//! contours on its layers. Profile sides use the same `inside` / `outside` / `on` values as the
//! profile job. Compensation is computed here, and travel is climb. Pocket and drill use the
//! matching `campanion-path` operations. Profile and engrave are depth passes linked per operation.
//!
//! Stock Z is 0. A depth pass cuts at `z = -pass` (Z is up). For a linked profile or engrave the
//! tool rapids at clearance, rapids down to the job retract height, and plunge-feeds each pass.
//! Those link rapids stay at or above the retract height. After linking, the operation's process
//! timing is applied: process on and the pierce dwell at the pierce height before the plunge, and
//! process off and the post delay before the rapid retract to clearance. The job file has no
//! pocket stepover, so rings are spaced by the tool radius. Drill cycles are not in this job
//! version: each hole is a rapid to the final depth, not an expanded peck.
//! `campanion_ir::verify_with` runs before anything is written. Safe height is the job clearance
//! and retract height is the job retract. Issues are printed and the process exits 1.

use std::ffi::{OsStr, OsString};
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use campanion_geom::{Containment, Contour, Orientation, Point, Region, Tolerance};
use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg};
use campanion_ir::{ClBlock, ClPath, CompensationMode, Document, Event, Import, ProcessKind, ProcessParams, Program};
use campanion_path::{
    DepthPlan, DrillCycle, DrillOp, Feature, Move, PocketOp, PocketStrategy, Point3, ProcessEvent, ProcessTiming, ProfileSide, Tool as PathTool,
    Toolpath, apply_timing,
};

use crate::error::CliError;
use crate::job::read_text;
use crate::jobfile::{self, JobFile, JobTool, Operation, OperationKind};
use crate::report;

/// Matches the IR cap. A hostile drawing fails here instead of building an unbounded program.
const MAX_TOOLPATHS: usize = 1_024;
/// Contours one operation will accept. Nesting pockets is quadratic in this count.
const MAX_CONTOURS: usize = 1_024;

struct RunArgs {
    job: PathBuf,
    output: PathBuf,
    ir: Option<PathBuf>,
}

struct Cut<'a> {
    op: &'a Operation,
    tool: &'a JobTool,
    import: &'a Import,
    tolerance: Tolerance,
    contours: &'a [Contour],
    job: &'a JobFile,
}

pub fn run(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_run(args)?;
    let job = jobfile::load(&args.job).map_err(show)?;
    let source = jobfile::source_path(&job);
    let text = read_text(&source)?;
    if jobfile::source_changed(&job, text.as_bytes()) {
        warn(&format!(
            "source {} changed: stored hash {}, file hash {}",
            job.source().path(),
            job.source().hash(),
            jobfile::hash_bytes(text.as_bytes()),
        ))?;
    }
    let is_svg = source.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("svg"));
    let options = ImportOptions::new(job.tolerance());
    let drawing = if is_svg { import_svg(&text, &options) } else { import_dxf(&text, &options) }.map_err(show)?;
    let format = if is_svg { "SVG" } else { "DXF" };
    if drawing.skipped_entities() > 0 {
        let kinds = drawing.skipped_types().iter().map(|(name, count)| format!("{name} x{count}")).collect::<Vec<_>>().join(", ");
        warn(&format!("skipped {format} entities: {kinds}"))?;
    }
    let doc = build(&job, &drawing)?;
    let options = campanion_ir::VerifyOptions { safe_z: Some(job.clearance()), retract_z: Some(job.retract()) };
    let report = campanion_ir::verify_with(doc.program(), &options);
    if !report.issues().is_empty() {
        let mut err_out = io::stderr().lock();
        write_all(&mut err_out, &report::format_report(&report))?;
        return Ok(ExitCode::from(1));
    }
    write_output(&args.output, &campanion_ir::write_ngc(doc.program())?)?;
    if let Some(ir_path) = &args.ir {
        write_output(ir_path, &doc.to_json_pretty()?)?;
    }
    Ok(ExitCode::SUCCESS)
}

fn parse_run(args: &[OsString]) -> Result<RunArgs, CliError> {
    let mut job: Option<&OsStr> = None;
    let mut output: Option<&OsStr> = None;
    let mut ir: Option<&OsStr> = None;
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "-o" || arg.as_os_str() == "--ir" {
            let flag = if arg.as_os_str() == "-o" { "-o" } else { "--ir" };
            let Some(path) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a file")));
            };
            index += 1;
            if path.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a file")));
            }
            let slot = if flag == "-o" { &mut output } else { &mut ir };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(path.as_os_str());
        } else if starts_with_dash(arg) {
            return Err(CliError::usage("unknown argument"));
        } else if job.is_some() || arg.is_empty() {
            return Err(CliError::usage("run takes one job file"));
        } else {
            job = Some(arg.as_os_str());
        }
    }
    let Some(job) = job else {
        return Err(CliError::usage("run needs a job file"));
    };
    let Some(output) = output else {
        return Err(CliError::usage("run needs -o OUT.ngc"));
    };
    if ir.is_some_and(|path| path == output) {
        return Err(CliError::usage("-o and --ir must be different files"));
    }
    Ok(RunArgs { job: PathBuf::from(job), output: PathBuf::from(output), ir: ir.map(PathBuf::from) })
}

fn starts_with_dash(arg: &OsStr) -> bool {
    arg.to_string_lossy().starts_with('-')
}

fn build(job: &JobFile, drawing: &Drawing) -> Result<Document, CliError> {
    let mut paths = Vec::new();
    for op in job.operations() {
        let contours = select(drawing, op.layers());
        if contours.is_empty() {
            return Err(op_err(op, "no geometry on the selected layers"));
        }
        if contours.len() > MAX_CONTOURS {
            return Err(op_err(op, format!("too many contours: {} > {MAX_CONTOURS}", contours.len())));
        }
        let Some(tool) = job.tool(op.tool_id()) else {
            return Err(op_err(op, format!("unknown tool {}", op.tool_id())));
        };
        let import = import_for(op, tool)?;
        let cut = Cut { op, tool, import: &import, tolerance: job.tolerance(), contours: &contours, job };
        match op.kind() {
            OperationKind::Profile { side } => profile_paths(&mut paths, &cut, side)?,
            OperationKind::Pocket => pocket_paths(&mut paths, &cut)?,
            OperationKind::Drill => drill_paths(&mut paths, &cut)?,
            OperationKind::Engrave => engrave_paths(&mut paths, &cut)?,
        }
    }
    let tools = job.tools().iter().map(|tool| tool.ir().clone()).collect();
    let program = Program::try_new(program_name(job), tools, None, paths, Vec::new(), Vec::new())?;
    Ok(Document::new(program)?)
}

fn select(drawing: &Drawing, layers: &[String]) -> Vec<Contour> {
    let mut contours = Vec::new();
    for item in drawing.contours() {
        if layers.iter().any(|layer| layer == item.layer()) {
            contours.push(item.contour().clone());
        }
    }
    contours
}

fn import_for(op: &Operation, tool: &JobTool) -> Result<Import, CliError> {
    import_with_params(op, tool, op.params().clone())
}

fn import_with_params(op: &Operation, tool: &JobTool, params: ProcessParams) -> Result<Import, CliError> {
    // A diameter is a rotating cutter. A kerf is a kerf-only process, same rule as the profile job.
    let process = match tool.path_tool() {
        PathTool::Diameter { .. } => ProcessKind::Spindle,
        PathTool::Kerf { .. } => ProcessKind::Torch,
    };
    let import = Import::new(process, op.feeds().cut())?.feeds(op.feeds())?.params(params)?.tool(tool.ir().clone())?.name(op.name())?;
    Ok(import.compensation(CompensationMode::Computed))
}

fn profile_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>, side: ProfileSide) -> Result<(), CliError> {
    let mut features = Vec::with_capacity(cut.contours.len());
    for contour in cut.contours {
        let shaped = compensate_profile(cut, side, contour)?;
        let toolpath = depth_at_retract(cut, &shaped)?;
        features.push(Feature::new(contour.clone(), toolpath));
    }
    link_operation(paths, cut, &features)
}

fn pocket_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let stepover = cut.tool.path_tool().compensation_distance().map_err(|err| op_err(cut.op, err))?;
    let regions = pocket_regions(cut.op, cut.contours, &cut.tolerance)?;
    let retract = retract_z(cut.job);
    for z in cut_levels(cut.op)? {
        for region in &regions {
            let pocket = PocketOp::new(cut.tool.path_tool(), PocketStrategy::OffsetRings, stepover)
                .z(z)
                .retract_z(retract)
                .tolerance(cut.tolerance)
                .generate(region)
                .map_err(|err| op_err(cut.op, err))?;
            push_doc(paths, Document::from_toolpath(pocket.toolpath(), cut.import)?)?;
        }
    }
    Ok(())
}

fn drill_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let levels = cut_levels(cut.op)?;
    let Some(z) = levels.last().copied() else {
        return Err(op_err(cut.op, "depth plan is empty"));
    };
    let points: &[campanion_geom::Point] = &[];
    let drilling = DrillOp::new(DrillCycle::new()).z(z).tolerance(cut.tolerance).generate(cut.contours, points).map_err(|err| op_err(cut.op, err))?;
    if drilling.hits().is_empty() {
        return Err(op_err(cut.op, "drill produced no holes"));
    }
    push_doc(paths, Document::from_toolpath(drilling.toolpath(), cut.import)?)
}

fn engrave_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let mut features = Vec::with_capacity(cut.contours.len());
    for contour in cut.contours {
        let toolpath = depth_at_retract(cut, contour)?;
        features.push(Feature::new(contour.clone(), toolpath));
    }
    link_operation(paths, cut, &features)
}

/// Depth passes from stock Z 0. The first rapid sits at clearance so the linker does not feed
/// down from there; the next rapid drops to the job retract height. The return to clearance is
/// a rapid inserted after process-off, so a post delay can sit between that stop and the retract.
/// Plunge moves use the operation plunge feed.
fn depth_at_retract(cut: &Cut<'_>, contour: &Contour) -> Result<Toolpath, CliError> {
    let plan = DepthPlan::new(0.0, cut.op.depth(), cut.op.step_down(), cut.op.entry());
    let path = campanion_path::depth_toolpath(contour, &plan, &cut.tolerance).map_err(|err| op_err(cut.op, err))?;
    let retract = cut.job.retract();
    let clearance = cut.job.clearance();
    let mut moves = Vec::with_capacity(path.moves().len().saturating_add(2));
    let mut saw_rapid = false;
    let mut at: Option<Point3> = None;
    for mv in path.moves() {
        if !saw_rapid && let Move::Rapid { to } = *mv {
            let at_clearance = Point3::new(to.x(), to.y(), clearance).map_err(|err| op_err(cut.op, err))?;
            moves.push(Move::Rapid { to: at_clearance });
            at = Some(at_clearance);
            if retract + 1e-9 < clearance {
                let at_retract = Point3::new(to.x(), to.y(), retract).map_err(|err| op_err(cut.op, err))?;
                moves.push(Move::Rapid { to: at_retract });
                at = Some(at_retract);
            }
            saw_rapid = true;
            continue;
        }
        if let Move::Rapid { to } = *mv
            && to.z() + 1e-9 < retract
        {
            return Err(op_err(cut.op, "rapid is below the retract height"));
        }
        let turning_off = matches!(mv, Move::Event(ProcessEvent::Off));
        if let Some(to) = motion_to(*mv) {
            at = Some(to);
        }
        moves.push(*mv);
        if turning_off
            && let Some(here) = at
            && here.z() + 1e-9 < clearance
        {
            let up = Point3::new(here.x(), here.y(), clearance).map_err(|err| op_err(cut.op, err))?;
            moves.push(Move::Rapid { to: up });
            at = Some(up);
        }
    }
    if !saw_rapid {
        return Err(op_err(cut.op, "toolpath needs a rapid to its start"));
    }
    Toolpath::new(moves).map_err(|err| op_err(cut.op, err))
}

fn motion_to(mv: Move) -> Option<Point3> {
    match mv {
        Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
        Move::Event(_) | Move::Dwell { .. } => None,
    }
}

fn link_operation(paths: &mut Vec<ClPath>, cut: &Cut<'_>, features: &[Feature]) -> Result<(), CliError> {
    let origin = Point::new(0.0, 0.0).map_err(|err| op_err(cut.op, err))?;
    let tol = cut.job.tolerance();
    let ordered = campanion_path::order_and_link(features, origin, cut.job.clearance(), &tol).map_err(|err| op_err(cut.op, err))?;
    reject_low_rapids(cut.op, ordered.toolpath(), cut.job.retract())?;
    // `from_toolpath` applies timing again from the import's parameters. Pass a zero-delay
    // import so this call is the one that places pierce and post events, then put the real
    // parameters back on each process-on.
    let timed = apply_op_timing(cut, ordered.toolpath())?;
    let motion = import_with_params(cut.op, cut.tool, ProcessParams::new())?;
    let doc = restore_process_params(Document::from_toolpath(&timed, &motion)?, cut.op.params())?;
    push_doc(paths, doc)
}

/// Pierce on, then the pierce dwell, at the pierce height, then the plunge to cut height.
/// Process off, then the post dwell. Heights and the on-during-delay toggles come from the operation.
fn apply_op_timing(cut: &Cut<'_>, path: &Toolpath) -> Result<Toolpath, CliError> {
    let params = cut.op.params();
    let mut timing = ProcessTiming::new(params.pierce_delay_s(), params.post_delay_s())
        .pierce_process_on_during_delay(params.pierce_process_on_during_delay())
        .post_process_on_during_delay(params.post_process_on_during_delay());
    if let Some(z) = params.pierce_height_z() {
        timing = timing.pierce_height(z);
    }
    if let Some(z) = params.cut_height_z() {
        timing = timing.cut_height(z);
    }
    apply_timing(path, &timing).map_err(|err| op_err(cut.op, err))
}

fn restore_process_params(doc: Document, params: &ProcessParams) -> Result<Document, CliError> {
    let program = doc.program();
    let mut toolpaths = Vec::with_capacity(program.toolpaths().len());
    for path in program.toolpaths() {
        let mut blocks = Vec::with_capacity(path.blocks().len());
        for block in path.blocks() {
            let next = match block {
                ClBlock::Event(Event::ProcessOn { process, .. }) => {
                    ClBlock::Event(Event::ProcessOn { process: process.clone(), params: params.clone() })
                }
                other => other.clone(),
            };
            blocks.push(next);
        }
        toolpaths.push(ClPath::try_new(
            path.name().map(str::to_string),
            path.tool_id(),
            path.compensation(),
            path.feeds(),
            path.pierce(),
            path.tabs().to_vec(),
            blocks,
        )?);
    }
    let rebuilt = Program::try_new(
        program.name().map(str::to_string),
        program.tools().to_vec(),
        program.machine().cloned(),
        toolpaths,
        program.channel_programs().to_vec(),
        program.syncs().to_vec(),
    )?;
    Ok(Document::new(rebuilt)?)
}

fn reject_low_rapids(op: &Operation, path: &Toolpath, retract: f64) -> Result<(), CliError> {
    for mv in path.moves() {
        if let Move::Rapid { to } = *mv
            && to.z() + 1e-9 < retract
        {
            return Err(op_err(op, "rapid is below the retract height"));
        }
    }
    Ok(())
}

fn compensate_profile(cut: &Cut<'_>, side: ProfileSide, contour: &Contour) -> Result<Contour, CliError> {
    let distance = cut.tool.path_tool().compensation_distance().map_err(|err| op_err(cut.op, err))?;
    let path = if matches!(side, ProfileSide::On) || distance <= cut.tolerance.linear() {
        contour.clone()
    } else {
        offset_profile(cut, side, contour, distance)?
    };
    orient_climb(cut.op, side, &path, &cut.tolerance)
}

fn offset_profile(cut: &Cut<'_>, side: ProfileSide, contour: &Contour, distance: f64) -> Result<Contour, CliError> {
    let signed = signed_profile_offset(cut.op, contour, side, distance, &cut.tolerance)?;
    if signed.abs() <= cut.tolerance.linear() {
        return Ok(contour.clone());
    }
    let parts = campanion_geom::offset(contour, signed, &cut.tolerance).map_err(|err| op_err(cut.op, err))?;
    match parts.len() {
        0 => Err(op_err(
            cut.op,
            if matches!(side, ProfileSide::Inside) { "tool does not fit inside the contour" } else { "offset produced no contour" },
        )),
        1 => {
            let Some(path) = parts.into_iter().next() else {
                return Err(op_err(cut.op, "offset produced no contour"));
            };
            if contour.is_closed() && !path.is_closed() {
                return Err(op_err(cut.op, "offset of a closed contour did not stay closed"));
            }
            Ok(path)
        }
        n => Err(op_err(cut.op, format!("offset split the profile into {n} contours"))),
    }
}

fn signed_profile_offset(op: &Operation, contour: &Contour, side: ProfileSide, magnitude: f64, tolerance: &Tolerance) -> Result<f64, CliError> {
    if matches!(side, ProfileSide::On) || magnitude == 0.0 {
        return Ok(0.0);
    }
    if !contour.is_closed() {
        return Ok(match side {
            ProfileSide::Inside => magnitude,
            ProfileSide::Outside => -magnitude,
            ProfileSide::On => 0.0,
        });
    }
    let into_interior = match contour.orientation(tolerance).map_err(|err| op_err(op, err))? {
        Orientation::CounterClockwise => magnitude,
        Orientation::Clockwise => -magnitude,
        Orientation::Degenerate => return Err(op_err(op, "degenerate profile")),
    };
    Ok(match side {
        ProfileSide::Inside => into_interior,
        ProfileSide::Outside => -into_interior,
        ProfileSide::On => 0.0,
    })
}

fn orient_climb(op: &Operation, side: ProfileSide, path: &Contour, tolerance: &Tolerance) -> Result<Contour, CliError> {
    if !path.is_closed() {
        let reverse = !matches!(side, ProfileSide::Inside);
        return Ok(if reverse { path.reversed() } else { path.clone() });
    }
    let want_ccw = matches!(side, ProfileSide::Inside);
    let is_ccw = match path.orientation(tolerance).map_err(|err| op_err(op, err))? {
        Orientation::CounterClockwise => true,
        Orientation::Clockwise => false,
        Orientation::Degenerate => return Err(op_err(op, "degenerate profile")),
    };
    if is_ccw == want_ccw { Ok(path.clone()) } else { Ok(path.reversed()) }
}

/// Depths from [`campanion_path::pass_depths`], as absolute Z below the stock surface.
fn cut_levels(op: &Operation) -> Result<Vec<f64>, CliError> {
    let depths = campanion_path::pass_depths(op.depth(), op.step_down()).map_err(|err| op_err(op, err))?;
    let mut levels = Vec::with_capacity(depths.len());
    for depth in depths {
        let z = -depth;
        if !z.is_finite() {
            return Err(op_err(op, "cut z is not finite"));
        }
        levels.push(z);
    }
    if levels.is_empty() {
        return Err(op_err(op, "depth plan is empty"));
    }
    Ok(levels)
}

fn retract_z(job: &JobFile) -> f64 {
    job.retract()
}

fn pocket_regions(op: &Operation, contours: &[Contour], tolerance: &Tolerance) -> Result<Vec<Region>, CliError> {
    for contour in contours {
        if !contour.is_closed() {
            return Err(op_err(op, "pocket contour is open"));
        }
    }
    if contours.len() == 1 {
        let Some(contour) = contours.first() else {
            return Err(op_err(op, "pocket has no contours"));
        };
        let region = Region::from_contour(contour.clone(), tolerance).map_err(|err| op_err(op, err))?;
        return Ok(vec![region]);
    }
    let n = contours.len();
    let mut areas = Vec::with_capacity(n);
    for contour in contours {
        areas.push(contour.signed_area().map_err(|err| op_err(op, err))?.abs());
    }
    let mut parent = vec![None; n];
    for i in 0..n {
        let Some(inner) = contours.get(i) else {
            continue;
        };
        let mut best: Option<(usize, f64)> = None;
        for j in 0..n {
            if i == j {
                continue;
            }
            let (Some(outer), Some(area)) = (contours.get(j), areas.get(j).copied()) else {
                continue;
            };
            if !strictly_inside(inner, outer, tolerance).map_err(|err| op_err(op, err))? {
                continue;
            }
            let take = match best {
                None => true,
                Some((_, best_area)) => area < best_area,
            };
            if take {
                best = Some((j, area));
            }
        }
        if let Some((j, _)) = best
            && let Some(slot) = parent.get_mut(i)
        {
            *slot = Some(j);
        }
    }
    let depth = nest_depth(op, &parent)?;
    let mut regions = Vec::new();
    for i in 0..n {
        let nest = depth.get(i).copied().unwrap_or(0);
        if nest % 2 != 0 {
            continue;
        }
        let Some(outer) = contours.get(i) else {
            continue;
        };
        let mut holes = Vec::new();
        for j in 0..n {
            if parent.get(j).copied().flatten() == Some(i)
                && depth.get(j).copied().unwrap_or(0) == nest + 1
                && let Some(hole) = contours.get(j)
            {
                holes.push(hole.clone());
            }
        }
        regions.push(Region::try_new(outer.clone(), holes, tolerance).map_err(|err| op_err(op, err))?);
    }
    if regions.is_empty() {
        return Err(op_err(op, "pocket produced no region"));
    }
    Ok(regions)
}

fn nest_depth(op: &Operation, parent: &[Option<usize>]) -> Result<Vec<usize>, CliError> {
    let n = parent.len();
    let mut depth = vec![0usize; n];
    for i in 0..n {
        let mut d = 0usize;
        let mut cursor = parent.get(i).copied().flatten();
        let mut guard = 0usize;
        while let Some(next) = cursor {
            d = d.saturating_add(1);
            guard = guard.saturating_add(1);
            if guard > n {
                return Err(op_err(op, "contour nesting did not finish"));
            }
            cursor = parent.get(next).copied().flatten();
        }
        if let Some(slot) = depth.get_mut(i) {
            *slot = d;
        }
    }
    Ok(depth)
}

fn strictly_inside(inner: &Contour, outer: &Contour, tolerance: &Tolerance) -> Result<bool, campanion_geom::GeomError> {
    if !inner.is_closed() || !outer.is_closed() {
        return Ok(false);
    }
    let mut saw_inside = false;
    for seg in inner.segments() {
        match outer.containment(seg.start(), tolerance)? {
            Containment::Outside => return Ok(false),
            Containment::Inside => saw_inside = true,
            Containment::Boundary => {}
        }
    }
    Ok(saw_inside)
}

fn push_doc(paths: &mut Vec<ClPath>, doc: Document) -> Result<(), CliError> {
    for path in doc.program().toolpaths() {
        if paths.len() >= MAX_TOOLPATHS {
            return Err(CliError::invalid(format!("too many toolpaths: {} > {MAX_TOOLPATHS}", paths.len() + 1)));
        }
        paths.push(path.clone());
    }
    Ok(())
}

fn program_name(job: &JobFile) -> Option<String> {
    let stem = job.path().file_stem().and_then(|stem| stem.to_str())?;
    if stem.is_empty() || stem.len() > 256 { None } else { Some(stem.to_string()) }
}

fn warn(message: &str) -> Result<(), CliError> {
    let mut out = io::stderr().lock();
    writeln!(out, "campanion: warning: {message}")?;
    out.flush()?;
    Ok(())
}

fn write_output(path: &Path, text: &str) -> Result<(), CliError> {
    let mut file = File::create(path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    write_all(&mut file, text).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if !text.ends_with('\n') {
        writeln!(file).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    }
    file.flush().map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    Ok(())
}

fn write_all(out: &mut impl Write, text: &str) -> Result<(), io::Error> {
    out.write_all(text.as_bytes())
}

fn show(err: impl std::fmt::Display) -> CliError {
    CliError::run(err.to_string())
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}
