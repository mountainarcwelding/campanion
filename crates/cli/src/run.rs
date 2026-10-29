//! `run`: load a `campanion-job/1` file, import its DXF, and write toolpaths.
//!
//! The source path is relative to the job file. A hash mismatch is a warning on stderr; the
//! bytes just read are imported and the job file is not rewritten. Each operation keeps the
//! contours on its layers. Profile sides use the same `inside` / `outside` / `on` values as the
//! profile job. Compensation is computed here, and travel is climb. Pocket, drill and face use the
//! matching `campanion-path` operations. Profile and engrave are depth passes linked per operation.
//! Each pocket region, at each depth, is linked the same way, and every region at one depth is
//! linked before the next depth. A finish pass, when the job asks for one, is linked after the
//! roughing passes.
//!
//! Stock Z is 0. A depth pass cuts at `z = -pass` (Z is up). For a linked profile, pocket, or
//! engrave the tool rapids at clearance, rapids down to the job retract height, and then feeds
//! the plunge, ramp or helix. A ramp or helix is planned from that retract height when it is
//! above the stock, so the first descent covers the whole drop and stays at or below the entry
//! angle. On a profile, engrave or pocket wall, later passes start at the previous level and
//! the ramp follows the compensated contour. A profile ramp that would cross a tab below the
//! bridge lifts across the bridge instead of descending through it. A pocket area pass enters
//! from the retract height at every depth, and feeds from one ring to the next at the level
//! when the link stays inside the pocket. A ramp longer than that ring, or a helix that leaves
//! the pocket, is an error. Those link rapids stay at or above the retract height. After linking,
//! process timing is applied. A spindle stays on for the whole tool: one process-on after the
//! tool change, a new process-on only when the speed changes, and one process-off after the
//! retract above the stock, at the next tool change or the end of the program. Torch and other
//! per-cut processes still turn off before that retract, with the pierce dwell at the pierce
//! height and the post delay beside the process-off.
//! The first toolpath, and each later toolpath whose tool id differs from the previous one, starts
//! with a tool-change event. Consecutive toolpaths that share a tool do not get another change.
//! Spindle speed comes from `operations[].speed`, or from `tools[].speed` when the operation omits
//! it, and is stored on process-on. A spindle operation with no speed fails `run --post` when that
//! post's spindle-on template uses `{speed}`.
//! A pocket uses `stepover` when the job sets it, and otherwise the tool radius. A face clears
//! the union of the selected contours' bounding boxes, grown by `margin` on every side. Its
//! stepover is the job value or three quarters of the tool diameter, and its rapids stay at the
//! job retract height. Roughing leaves `stock_to_leave` on the walls and above the floor. When
//! `finish` is set, that stock is cut afterwards: a profile at the tool radius, then a pocket
//! floor pass and wall contours.
//! A chamfer plunges once to the tip along each selected closed contour. An open contour is an
//! error. The V-bit's largest cutting diameter is the tool diameter, stock Z is 0, and the job
//! tolerance is used. The pass is linked the same way as an engrave.
//! Each hole is a rapid to clearance over the hole, a rapid to the job retract height (the R
//! plane), a plunge feed to depth (pecking back to the R plane when `step_down` is less than
//! `depth`), an optional dwell, and a rapid out to clearance. No canned cycle is written.
//! G81/G83 output may become a post option later. A contour that is not a circle is skipped, so a
//! layer can hold holes and other profiles together.
//! `campanion_ir::verify_with` runs before anything is written. Safe height is the job clearance
//! and retract height is the job retract. Issues are printed and the process exits 1. On a 3-axis
//! job, a rapid that ends below the retract height, or that traverses below it, is an error too.
//! Rotary and tilted programs are checked in the planning frame instead.
//!
//! `--post` writes that dialect instead of the test-sink NGC. The text is read back and compared
//! at the job chord tolerance. A mismatch exits 1 and nothing is written. Without `--post` the
//! test sink is unchanged.
//!
//! An STL source is read as bytes, not as text, and is not sent through the DXF or SVG importer.
//! Each `surface` operation is a parallel finish: a rapid at or above the stock top (the job
//! clearance when that is higher), a plunge to the first cutter location, cuts along the raster
//! and along the drop-cutter link between lines, a retract to the job retract height, then a
//! rapid back to that safe height.
//! Each `rough` operation is Z-level roughing. Levels run top to bottom and rings stay in order.
//! For each ring the tool rapids at clearance to the entry, rapids down to the safe plane (the
//! retract height, or the stock top when that is higher), feeds the entry from the previous level
//! (or from that plane when a ramp or helix would otherwise rapid below it), cuts the ring, and
//! feeds back to the retract height. An omitted entry is a plunge. A helix that leaves the
//! ring or the stock allowance ramps instead, and the operation warns once. The process turns on
//! once before the first cut. A spindle's process-off follows the clearance rapid; a torch turns
//! off after the last retract and then the tool rapids to clearance.
//! Each `waterline` operation finishes steep walls. Levels use the same depth plan as roughing.
//! For each polyline the tool rapids at clearance to the first point, rapids down to the safe
//! plane (the retract height, or the stock top when that is higher), plunges to that point's Z
//! plus axial stock, cuts along the polyline, and feeds back to the retract height. The process
//! turns on once. A spindle's process-off follows the clearance rapid; a torch still turns off
//! after the last retract.
//! After the document is built, each cutting move is checked against the mesh with that
//! operation's cutter. The sample step is `min(0.25, chord * 10)` millimetres and the allowance
//! is the job chord tolerance. A gouge is a verify error. Roughing, surfacing, waterline and
//! rotary finish are compared with the drop-cutter surface of the part. `stock_to_leave` is not
//! added to that surface: the cuts sit above the surface plus the stock. A rotary finish checks
//! each constant angle on the mesh turned by that angle, and samples angle-changing links in
//! between.
//!
//! A job with a rotary setup plans each indexed operation in the indexed view at `index_deg`
//! (0° when that field is omitted). Two-dimensional geometry is used as drawn. Mesh operations
//! see the mesh turned by that angle. The resulting poses are mapped back into the part frame
//! and tagged with the angle. When the index changes, a retract to the index safe height is
//! appended to the operation being left and a rotary rapid at the new angle is prepended to the
//! next one. Jobs without `rotary` stay 3-axis and are not mapped.
//!
//! A job with `kinematics` stores the five-axis setup. An operation that sets `plane` is planned
//! in that view and mapped into the part frame. When the frame changes, a rapid to the job
//! clearance along the plane normal is appended to the operation being left and prepended to the
//! next one. The first tilted operation gets only the approach rapid.
//!
//! A wrap needs that rotary setup. Drawing X is the distance along the axis and drawing Y is
//! the arc length around the cylinder, each plus the operation offset. The path is built in
//! the part frame, so it is not mapped again. Rapids sit at the index safe radius, which uses
//! the cylinder radius as well as the stock of the other operations. A wrap next to an indexed
//! operation still takes the index-change retract and rotate. Feeds stay the tool's surface feeds.

//! `--up-axis z|y|auto` applies only to a STEP source. The default is `z`. `auto` is opt-in:
//! the exporter header SolidCraft reports is trusted, and a Y-up file (SolidWorks) is turned
//! so Z is up. `z` and `y` skip that probe. Any other source ignores the flag. The face point
//! is in the frame after the turn. A file point `(x, y, z)` is `(x, -z, y)` once Y up has been
//! turned.

use std::borrow::Cow;
use std::ffi::{OsStr, OsString};
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use campanion_geom::{BoundingBox, Containment, Contour, Orientation, Point, Region, Tolerance};
use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg};
use campanion_ir::{
    ClBlock, ClPath, CompensationMode, Document, Event, Import, ProcessKind, ProcessParams, Program, RotarySetup, Vec3, hold_spindle,
};
use campanion_mesh::Mesh;
use campanion_path::{
    ChamferOp, CutKind, DepthEntry, DepthPlan, DepthTabs, DrillCycle, DrillOp, FaceOp, Feature, Move, PocketOp, PocketStrategy, Point3, ProcessEvent,
    ProcessTiming, ProfileSide, Tool as PathTool, Toolpath, apply_timing,
};
use campanion_post::PostDef;

use crate::error::{CliError, MAX_JSON_BYTES};
use crate::index::{self, IndexedOp};
use crate::job::read_text;
use crate::jobfile::{self, Finish, JobFile, JobTool, Operation, OperationKind};
use crate::plane::{self, ViewFrame};
use crate::report;
use crate::rotary;
use crate::rough;
use crate::safety;
use crate::step;
use crate::surface;
use crate::waterline;
use crate::wrap;

/// Matches the IR cap. A hostile drawing fails here instead of building an unbounded program.
const MAX_TOOLPATHS: usize = 1_024;
/// Contours one operation will accept. Nesting pockets is quadratic in this count.
const MAX_CONTOURS: usize = 1_024;

struct RunArgs {
    job: PathBuf,
    output: PathBuf,
    ir: Option<PathBuf>,
    post: Option<PathBuf>,
    up_axis: step::UpAxis,
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
    let post = match &args.post {
        Some(spec) => Some(crate::post::load_spec(spec)?),
        None => None,
    };
    let job = jobfile::load(&args.job).map_err(show)?;
    let source = jobfile::source_path(&job);
    if job.source().is_stl() {
        let bytes = read_bytes(&source)?;
        warn_if_changed(&job, &bytes)?;
        let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
        let doc = build_surface(&job, &mesh)?;
        let gouges = crate::gouge::check_job(&job, &mesh, doc.program())?;
        return emit_program(&args, &job, doc, &gouges, post.as_ref());
    }
    let text = read_text(&source)?;
    warn_if_changed(&job, text.as_bytes())?;
    let is_svg = source.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("svg"));
    let is_step = job.source().is_step();
    let options = ImportOptions::new(job.tolerance());
    let drawing = if is_step {
        // A loaded STEP source always names a face. The bytes above were only for the hash.
        let Some(face) = job.source().face() else {
            return Err(CliError::run("STEP source has no face"));
        };
        let cli = step::find_cli().ok_or_else(step::missing_cli_error)?;
        // `face` is already in the frame `--up-axis` produces. Y up is turned inside SolidCraft.
        let dxf = step::face_to_dxf(&cli, &source, face, args.up_axis)?;
        import_dxf(&dxf, &options)
    } else if is_svg {
        import_svg(&text, &options)
    } else {
        import_dxf(&text, &options)
    }
    .map_err(show)?;
    let format = if is_step {
        "STEP"
    } else if is_svg {
        "SVG"
    } else {
        "DXF"
    };
    if drawing.skipped_entities() > 0 {
        let kinds = drawing.skipped_types().iter().map(|(name, count)| format!("{name} x{count}")).collect::<Vec<_>>().join(", ");
        warn(&format!("skipped {format} entities: {kinds}"))?;
    }
    let doc = build(&job, &drawing)?;
    emit_program(&args, &job, doc, &[], post.as_ref())
}

fn warn_if_changed(job: &JobFile, bytes: &[u8]) -> Result<(), CliError> {
    if jobfile::source_changed(job, bytes) {
        warn(&format!("source {} changed: stored hash {}, file hash {}", job.source().path(), job.source().hash(), jobfile::hash_bytes(bytes),))?;
    }
    Ok(())
}

/// STL is not text. The hash is these bytes, and the drawing importers are not used.
pub(crate) fn read_bytes(path: &Path) -> Result<Vec<u8>, CliError> {
    use std::io::Read;

    let file = File::open(path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if let Ok(meta) = file.metadata()
        && meta.len() > MAX_JSON_BYTES as u64
    {
        return Err(CliError::invalid(format!("{} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    let mut bytes = Vec::new();
    file.take(MAX_JSON_BYTES as u64 + 1).read_to_end(&mut bytes).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if bytes.len() > MAX_JSON_BYTES {
        return Err(CliError::invalid(format!("{} is larger than {MAX_JSON_BYTES} bytes", path.display())));
    }
    Ok(bytes)
}

fn build_surface(job: &JobFile, mesh: &Mesh) -> Result<Document, CliError> {
    let mut groups = Vec::new();
    for op in job.operations() {
        let Some(tool) = job.tool(op.tool_id()) else {
            return Err(op_err(op, format!("unknown tool {}", op.tool_id())));
        };
        let import = import_for(op, tool)?;
        if matches!(op.kind(), OperationKind::RotaryFinish) {
            let Some(setup) = job.rotary() else {
                return Err(op_err(op, "rotary finish needs a rotary setup"));
            };
            let path = rotary::cut(job, op, tool, mesh, &setup)?;
            safety::check_radial(op.name(), std::slice::from_ref(&path), setup.center(), setup.axis_direction())?;
            groups.push(index::part_frame_op(op.tool_id(), op.feeds(), vec![path])?);
            continue;
        }
        let view = planning_mesh(job, mesh, op)?;
        let path = match op.kind() {
            OperationKind::Surface => surface::finish(job, op, tool, view.as_ref())?,
            OperationKind::Rough => rough::rough(job, op, tool, view.as_ref())?,
            OperationKind::Waterline => waterline::finish(job, op, tool, view.as_ref())?,
            OperationKind::RotaryFinish => return Err(op_err(op, "rotary finish needs a rotary setup")),
            _ => return Err(op_err(op, "an STL source only allows surface, rough, waterline or rotary_finish operations")),
        };
        let mut paths = Vec::new();
        push_doc(&mut paths, Document::from_toolpath(&path, &import)?)?;
        // Stock top is the highest Z of this planning mesh. The R plane is the job retract
        // height. The floor is the higher of the two, so a retract below the mesh still rejects
        // a rapid inside the part under the stock top. Surface rapids are raised to that floor.
        let bounds = view.bounds().map_err(|err| op_err(op, err))?;
        let grow = tool.diameter().map_or(0.0, |d| d * 0.5);
        let footprint = safety::Footprint::from_aabb(bounds.min().x, bounds.min().y, bounds.max().x, bounds.max().y, grow);
        let floor_z = bounds.max().z.max(job.retract());
        safety::check_planar(op.name(), &paths, &footprint, floor_z)?;
        groups.push(indexed_op(op, paths));
    }
    let sweep = if job.rotary().is_some() { mesh_vertices(mesh)? } else { Vec::new() };
    finish_program(job, groups, None, &sweep)
}

fn emit_program(args: &RunArgs, job: &JobFile, doc: Document, gouges: &[report::GougeNote], post: Option<&PostDef>) -> Result<ExitCode, CliError> {
    if let Some(post) = post {
        require_spindle_speeds(job, post)?;
    }
    let options = campanion_ir::VerifyOptions { safe_z: Some(job.clearance()), retract_z: Some(job.retract()) };
    let report = campanion_ir::verify_with(doc.program(), &options);
    // Rotary and 3+2 programs are checked in the planning frame, before indexing. A 3-axis
    // program also blocks verifier findings that end or traverse below the retract height.
    // An XY rapid between the retract and clearance heights stays a non-blocking finding.
    let rapid_lines = if job.rotary().is_none() && job.five_axis().is_none() { safety::blocking_rapids(&report, job.retract()) } else { Vec::new() };
    if !report.issues().is_empty() || !gouges.is_empty() || !rapid_lines.is_empty() {
        let mut err_out = io::stderr().lock();
        if !report.issues().is_empty() || !gouges.is_empty() {
            write_all(&mut err_out, &report::format_report_and_gouges(&report, gouges))?;
        }
        for line in &rapid_lines {
            writeln!(err_out, "{line}")?;
        }
        err_out.flush()?;
        return Ok(ExitCode::from(1));
    }
    let text = match post {
        Some(post) => crate::post::render(doc.program(), post, job.tolerance().chord())?,
        None => campanion_ir::write_ngc(doc.program())?,
    };
    write_output(&args.output, &text)?;
    if let Some(ir_path) = &args.ir {
        write_output(ir_path, &doc.to_json_pretty()?)?;
    }
    Ok(ExitCode::SUCCESS)
}

fn parse_run(args: &[OsString]) -> Result<RunArgs, CliError> {
    let mut job: Option<&OsStr> = None;
    let mut output: Option<&OsStr> = None;
    let mut ir: Option<&OsStr> = None;
    let mut post: Option<&OsStr> = None;
    let mut up_axis: Option<step::UpAxis> = None;
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "--up-axis" {
            let Some(value) = args.get(index) else {
                return Err(CliError::usage("--up-axis needs a value"));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage("--up-axis needs a value"));
            }
            if up_axis.is_some() {
                return Err(CliError::usage("--up-axis is already set"));
            }
            let Some(text) = value.to_str() else {
                return Err(CliError::usage("--up-axis must be z, y or auto"));
            };
            up_axis = Some(step::UpAxis::parse(text)?);
        } else if arg.as_os_str() == "-o" || arg.as_os_str() == "--ir" || arg.as_os_str() == "--post" {
            let flag = if arg.as_os_str() == "-o" {
                "-o"
            } else if arg.as_os_str() == "--ir" {
                "--ir"
            } else {
                "--post"
            };
            let Some(value) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            }
            let slot = if flag == "-o" {
                &mut output
            } else if flag == "--ir" {
                &mut ir
            } else {
                &mut post
            };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(value.as_os_str());
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
    Ok(RunArgs {
        job: PathBuf::from(job),
        output: PathBuf::from(output),
        ir: ir.map(PathBuf::from),
        post: post.map(PathBuf::from),
        up_axis: up_axis.unwrap_or(step::UpAxis::Z),
    })
}

fn starts_with_dash(arg: &OsStr) -> bool {
    arg.to_string_lossy().starts_with('-')
}

fn build(job: &JobFile, drawing: &Drawing) -> Result<Document, CliError> {
    // One nesting pass for the drawing. Operations then pick OUTER / INNER instead of layer "0".
    // Contours are collected first so the swept radius, including every wrap cylinder, is known
    // before a wrap path is built. A wrap's drawing is not stock: its X/Y are cylinder coordinates.
    let step_labels = if job.source().is_step() { Some(step_contour_labels(drawing, &job.tolerance())?) } else { None };
    let mut selected = Vec::new();
    let mut stock = StockBox::default();
    for op in job.operations() {
        let contours = select(drawing, op.layers(), step_labels.as_deref());
        if contours.is_empty() {
            return Err(op_err(op, "no geometry on the selected layers"));
        }
        if contours.len() > MAX_CONTOURS {
            return Err(op_err(op, format!("too many contours: {} > {MAX_CONTOURS}", contours.len())));
        }
        if job.rotary().is_some() && !matches!(op.kind(), OperationKind::Wrap) {
            stock.include(op, &contours)?;
        }
        selected.push((op, contours));
    }
    let corners = if job.rotary().is_some() { Some(stock.corners()?) } else { None };
    let safe_z = job.rotary().map(|setup| swept_clearance(job, &setup, corners.as_deref(), &[]));
    let plane_job = job.five_axis().is_some();
    let mut groups = Vec::new();
    let mut plane_groups = Vec::new();
    for (op, contours) in &selected {
        let Some(tool) = job.tool(op.tool_id()) else {
            return Err(op_err(op, format!("unknown tool {}", op.tool_id())));
        };
        let import = import_for(op, tool)?;
        let cut = Cut { op, tool, import: &import, tolerance: job.tolerance(), contours, job };
        let mut paths = Vec::new();
        match op.kind() {
            OperationKind::Profile { side } => profile_paths(&mut paths, &cut, side)?,
            OperationKind::Pocket => pocket_paths(&mut paths, &cut)?,
            OperationKind::Drill => drill_paths(&mut paths, &cut)?,
            OperationKind::Engrave => engrave_paths(&mut paths, &cut)?,
            OperationKind::Face => face_paths(&mut paths, &cut)?,
            OperationKind::Chamfer { side } => chamfer_paths(&mut paths, &cut, side)?,
            OperationKind::Surface => return Err(op_err(op, "surface finishing needs an STL source")),
            OperationKind::Rough => return Err(op_err(op, "roughing needs an STL source")),
            OperationKind::Waterline => return Err(op_err(op, "waterline finishing needs an STL source")),
            OperationKind::RotaryFinish => return Err(op_err(op, "rotary finish needs an STL source")),
            OperationKind::Wrap => {
                let Some(setup) = job.rotary() else {
                    return Err(op_err(op, "wrap needs a rotary setup"));
                };
                let Some(safe) = safe_z else {
                    return Err(op_err(op, "wrap needs a rotary setup"));
                };
                wrap::cut(&mut paths, op, tool, contours, &setup, job.tolerance().chord(), safe)?;
            }
        }
        if matches!(op.kind(), OperationKind::Wrap) {
            let Some(setup) = job.rotary() else {
                return Err(op_err(op, "wrap needs a rotary setup"));
            };
            safety::check_radial(op.name(), &paths, setup.center(), setup.axis_direction())?;
        } else {
            let grow = tool.diameter().map_or(0.0, |diameter| diameter * 0.5);
            let footprint = safety::Footprint::from_contours(contours, grow).map_err(|err| op_err(op, err))?;
            safety::check_planar(op.name(), &paths, &footprint, job.retract())?;
        }
        if plane_job {
            let frame = match op.plane() {
                Some(plane) => Some(ViewFrame::from_plane(plane)?),
                None => None,
            };
            if let Some(frame) = frame {
                paths = plane::map_paths_to_part(&paths, &frame, job.tolerance().chord())?;
            }
            plane_groups.push((frame, paths));
        } else if matches!(op.kind(), OperationKind::Wrap) {
            groups.push(index::part_frame_op(op.tool_id(), op.feeds(), paths)?);
        } else {
            groups.push(indexed_op(op, paths));
        }
    }
    if plane_job {
        let paths = plane::link_frames(plane_groups, job.clearance())?;
        return finish_paths(job, paths, None);
    }
    finish_program(job, groups, corners.as_deref(), &[])
}

/// Axis-aligned stock for a 2D job: contour bounds in XY, and Z from `-depth` up to the stock surface.
#[derive(Clone, Copy)]
struct StockBox {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    deepest: f64,
    any: bool,
}

impl Default for StockBox {
    fn default() -> Self {
        Self { min_x: f64::INFINITY, min_y: f64::INFINITY, max_x: f64::NEG_INFINITY, max_y: f64::NEG_INFINITY, deepest: 0.0, any: false }
    }
}

impl StockBox {
    fn include(&mut self, op: &Operation, contours: &[Contour]) -> Result<(), CliError> {
        self.deepest = self.deepest.max(op.depth());
        for contour in contours {
            let bbox = contour.bounding_box().map_err(|err| op_err(op, err))?;
            self.any = true;
            self.min_x = self.min_x.min(bbox.min().x());
            self.min_y = self.min_y.min(bbox.min().y());
            self.max_x = self.max_x.max(bbox.max().x());
            self.max_y = self.max_y.max(bbox.max().y());
        }
        Ok(())
    }

    fn corners(&self) -> Result<Vec<Vec3>, CliError> {
        if !self.any {
            return Ok(Vec::new());
        }
        let min = Vec3::new(self.min_x, self.min_y, -self.deepest)?;
        let max = Vec3::new(self.max_x, self.max_y, 0.0)?;
        index::box_corners(min, max)
    }
}

fn indexed_op(op: &Operation, paths: Vec<ClPath>) -> IndexedOp {
    IndexedOp { angle_deg: op.index_deg().unwrap_or(0.0), tool_id: Some(op.tool_id()), feeds: op.feeds(), paths, part_frame: false }
}

/// Mesh the planner sees. Angle 0 and a job with no rotary setup keep the part mesh.
fn planning_mesh<'a>(job: &JobFile, mesh: &'a Mesh, op: &Operation) -> Result<Cow<'a, Mesh>, CliError> {
    let Some(setup) = job.rotary() else {
        return Ok(Cow::Borrowed(mesh));
    };
    let angle = op.index_deg().unwrap_or(0.0);
    if !index::angles_differ(angle, 0.0) {
        return Ok(Cow::Borrowed(mesh));
    }
    Ok(Cow::Owned(index::rotate_mesh(mesh, &setup, angle)?))
}

fn mesh_vertices(mesh: &Mesh) -> Result<Vec<Vec3>, CliError> {
    let mut points = Vec::new();
    for triangle in mesh.triangles() {
        for point in [triangle.a, triangle.b, triangle.c] {
            points.push(Vec3::new(point.x, point.y, point.z)?);
        }
    }
    Ok(points)
}

/// Part-frame program. Without a rotary setup the toolpaths are stored as planned.
fn finish_program(job: &JobFile, groups: Vec<IndexedOp>, stock_corners: Option<&[Vec3]>, mesh_vertices: &[Vec3]) -> Result<Document, CliError> {
    let (paths, rotary) = match job.rotary() {
        None => (groups.into_iter().flat_map(|group| group.paths).collect::<Vec<_>>(), None),
        Some(setup) => {
            let safe_z = swept_clearance(job, &setup, stock_corners, mesh_vertices);
            (index::frame_operations(&setup, groups, safe_z, job.tolerance().chord())?, Some(setup))
        }
    };
    finish_paths(job, paths, rotary)
}

/// Shared tail of [`finish_program`]. `rotary` stays `None` for a three-axis or five-axis job.
fn finish_paths(job: &JobFile, paths: Vec<ClPath>, rotary: Option<RotarySetup>) -> Result<Document, CliError> {
    let tools = job.tools().iter().map(|tool| tool.ir().clone()).collect();
    if paths.len() > MAX_TOOLPATHS {
        return Err(CliError::invalid(format!("too many toolpaths: {} > {MAX_TOOLPATHS}", paths.len())));
    }
    let paths = insert_tool_changes(paths)?;
    let mut program = Program::try_new(program_name(job), tools, None, paths, Vec::new(), Vec::new(), rotary)?;
    if let Some(setup) = job.five_axis() {
        program = program.with_five_axis(setup.clone())?;
    }
    let program = hold_spindle(program)?;
    Ok(Document::new(program)?)
}

/// Index safe height. A wrap cylinder's radius joins the stock swept radius. With no wrap the
/// radius is unchanged, so a job that does not wrap keeps the same retract.
fn swept_clearance(job: &JobFile, setup: &RotarySetup, stock_corners: Option<&[Vec3]>, mesh_vertices: &[Vec3]) -> f64 {
    let mut radius = index::swept_radius(setup, stock_corners, mesh_vertices);
    for op in job.operations() {
        if let Some(diameter) = op.cylinder_diameter() {
            radius = radius.max(diameter * 0.5);
        }
    }
    index::index_safe_z(job.clearance(), radius, job.clearance())
}

fn select(drawing: &Drawing, layers: &[String], labels: Option<&[String]>) -> Vec<Contour> {
    let mut contours = Vec::new();
    for (index, item) in drawing.contours().iter().enumerate() {
        // `labels`, when present, replaces the DXF layer for that contour.
        let matches = match labels.and_then(|names| names.get(index)) {
            Some(label) => layers.iter().any(|layer| layer == label),
            None => layers.iter().any(|layer| layer == item.layer()),
        };
        if matches {
            contours.push(item.contour().clone());
        }
    }
    contours
}

/// STEP face export puts every contour on DXF layer "0". Depth is how many other contours
/// strictly contain this one: even is "OUTER" (outline and islands), odd is "INNER" (holes).
fn step_contour_labels(drawing: &Drawing, tolerance: &Tolerance) -> Result<Vec<String>, CliError> {
    let mut contours = Vec::new();
    for item in drawing.contours() {
        contours.push(item.contour().clone());
    }
    let n = contours.len();
    if n > MAX_CONTOURS {
        return Err(CliError::run(format!("too many contours: {n} > {MAX_CONTOURS}")));
    }
    let mut labels = Vec::with_capacity(n);
    for i in 0..n {
        let Some(inner) = contours.get(i) else {
            labels.push("OUTER".to_string());
            continue;
        };
        let mut depth = 0usize;
        for j in 0..n {
            if i == j {
                continue;
            }
            let Some(outer) = contours.get(j) else {
                continue;
            };
            if strictly_inside(inner, outer, tolerance).map_err(show)? {
                depth = depth.saturating_add(1);
            }
        }
        labels.push(if depth.is_multiple_of(2) { "OUTER".to_string() } else { "INNER".to_string() });
    }
    Ok(labels)
}

fn import_for(op: &Operation, tool: &JobTool) -> Result<Import, CliError> {
    let params = op.params_with_speed(tool).map_err(|err| op_err(op, err))?;
    import_with_params(op, tool, params)
}

/// A spindle-on template that substitutes `{speed}` needs a rpm on every diameter-tool operation.
fn require_spindle_speeds(job: &JobFile, post: &PostDef) -> Result<(), CliError> {
    if !post.spindle_on_needs_speed() {
        return Ok(());
    }
    for op in job.operations() {
        let Some(tool) = job.tool(op.tool_id()) else {
            continue;
        };
        if !matches!(tool.path_tool(), PathTool::Diameter { .. }) {
            continue;
        }
        if op.resolved_speed(tool).is_none() {
            return Err(op_err(op, "no spindle speed; set tools[].speed or operations[].speed"));
        }
    }
    Ok(())
}

/// `Event::ToolChange` on the first toolpath and on each later path whose tool id differs.
fn insert_tool_changes(paths: Vec<ClPath>) -> Result<Vec<ClPath>, CliError> {
    let mut previous: Option<u32> = None;
    let mut out = Vec::with_capacity(paths.len());
    for path in paths {
        let next = match path.tool_id() {
            Some(tool_id) if previous != Some(tool_id) => {
                previous = Some(tool_id);
                prepend_tool_change(&path, tool_id)?
            }
            _ => path,
        };
        out.push(next);
    }
    Ok(out)
}

fn prepend_tool_change(path: &ClPath, tool_id: u32) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(path.blocks().len().saturating_add(1));
    blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
    blocks.extend(path.blocks().iter().cloned());
    index::rebuild(path, blocks)
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
    let radius = tool_radius(cut)?;
    let stock = cut.op.stock_to_leave();
    // Roughing stands off the wall by `wall` and stops `floor` short of the final depth.
    link_profile(paths, cut, side, radius + stock.wall(), cut.op.depth() - stock.floor(), cut.op.step_down())?;
    if let Some(finish) = cut.op.finish() {
        link_profile(paths, cut, side, radius, cut.op.depth(), finish.step_down())?;
    }
    Ok(())
}

/// One linked profile. `distance` is how far the cutter centre sits from the part.
fn link_profile(paths: &mut Vec<ClPath>, cut: &Cut<'_>, side: ProfileSide, distance: f64, total: f64, step_down: f64) -> Result<(), CliError> {
    if !distance.is_finite() {
        return Err(op_err(cut.op, "profile offset is not finite"));
    }
    let mut features = Vec::with_capacity(cut.contours.len());
    for contour in cut.contours {
        let shaped = compensate_profile(cut, side, contour, distance)?;
        let toolpath = depth_with(cut, &shaped, total, step_down)?;
        features.push(Feature::new(contour.clone(), toolpath));
    }
    link_operation(paths, cut, &features)
}

fn pocket_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let stepover = pocket_stepover(cut)?;
    let regions = pocket_regions(cut.op, cut.contours, &cut.tolerance)?;
    let stock = cut.op.stock_to_leave();
    // All regions at one depth, then the next depth. Linking each depth on its own keeps that
    // order; `order_and_link` only chooses the travel order inside the depth.
    for z in depth_levels(cut.op, cut.op.depth() - stock.floor(), cut.op.step_down())? {
        link_pocket_level(paths, cut, &regions, z, stepover, stock.wall())?;
    }
    let Some(finish) = cut.op.finish() else {
        return Ok(());
    };
    // The floor pass still holds the wall allowance. Wall contours come after it.
    if finish.floor() && stock.floor() > 0.0 {
        link_pocket_level(paths, cut, &regions, -cut.op.depth(), stepover, stock.wall())?;
    }
    let walls = pocket_wall_features(cut, &regions, finish)?;
    link_operation(paths, cut, &walls)
}

fn pocket_stepover(cut: &Cut<'_>) -> Result<f64, CliError> {
    match cut.op.stepover() {
        Some(step) => Ok(step),
        None => tool_radius(cut),
    }
}

fn link_pocket_level(paths: &mut Vec<ClPath>, cut: &Cut<'_>, regions: &[Region], z: f64, stepover: f64, allowance: f64) -> Result<(), CliError> {
    let mut features = Vec::with_capacity(regions.len());
    for region in regions {
        let toolpath = pocket_toolpath(cut, region, z, stepover, allowance)?;
        features.push(Feature::new(region.outer().clone(), toolpath));
    }
    link_operation(paths, cut, &features)
}

fn pocket_toolpath(cut: &Cut<'_>, region: &Region, z: f64, stepover: f64, allowance: f64) -> Result<Toolpath, CliError> {
    let pocket = PocketOp::new(cut.tool.path_tool(), PocketStrategy::OffsetRings, stepover)
        .allowance(allowance)
        .z(z)
        .retract_z(retract_z(cut.job))
        .entry(cut.op.entry())
        .tolerance(cut.tolerance)
        .generate(region)
        .map_err(|err| op_err(cut.op, err))?;
    // The generator feeds back to retract after process-off. Drop that move so the same
    // clearance rapid profiles use can follow process-off.
    clearance_retracts(cut, &moves_through_process_off(pocket.toolpath()))
}

/// Wall finish follows the tool-radius offset of each region. The outer is an inside profile
/// and each hole is an outside profile, so both climb.
fn pocket_wall_features(cut: &Cut<'_>, regions: &[Region], finish: Finish) -> Result<Vec<Feature>, CliError> {
    let radius = tool_radius(cut)?;
    let mut features = Vec::new();
    for region in regions {
        let pieces = campanion_geom::offset_region(region, radius, &cut.tolerance).map_err(|err| op_err(cut.op, err))?;
        if pieces.is_empty() {
            return Err(op_err(cut.op, "tool does not fit in the pocket"));
        }
        for piece in pieces {
            push_wall_contour(cut, piece.outer(), ProfileSide::Inside, finish, &mut features)?;
            for hole in piece.holes() {
                push_wall_contour(cut, hole, ProfileSide::Outside, finish, &mut features)?;
            }
        }
    }
    if features.is_empty() {
        return Err(op_err(cut.op, "pocket finish produced no contour"));
    }
    Ok(features)
}

fn push_wall_contour(cut: &Cut<'_>, contour: &Contour, side: ProfileSide, finish: Finish, features: &mut Vec<Feature>) -> Result<(), CliError> {
    let shaped = orient_climb(cut.op, side, contour, &cut.tolerance)?;
    let toolpath = depth_with(cut, &shaped, cut.op.depth(), finish.step_down())?;
    features.push(Feature::new(shaped, toolpath));
    Ok(())
}

/// Clear the contours' bounding box, grown by the face margin.
///
/// `FaceOp` rapids at the job retract height, including the move from one depth to the next.
/// Those rapids share one Z, so `campanion verify` (safe height inferred from the rapids)
/// accepts them. The document is built the same way a drill is.
fn face_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let PathTool::Diameter { diameter } = cut.tool.path_tool() else {
        return Err(op_err(cut.op, "face is not valid with a kerf tool"));
    };
    let Some(stepover) = cut.op.stepover() else {
        return Err(op_err(cut.op, "face needs a stepover"));
    };
    let area = face_area(cut)?;
    let face = FaceOp::new(diameter, stepover)
        .angle(cut.op.face_angle())
        .stock_z(0.0)
        .depth(cut.op.depth())
        .step_down(cut.op.step_down())
        .retract_z(retract_z(cut.job))
        .generate(&area)
        .map_err(|err| op_err(cut.op, err))?;
    push_doc(paths, Document::from_toolpath(face.toolpath(), cut.import)?)
}

/// Union of the selected contours' bounding boxes, expanded by `margin` on every side.
fn face_area(cut: &Cut<'_>) -> Result<BoundingBox, CliError> {
    let mut area: Option<BoundingBox> = None;
    for contour in cut.contours {
        let bounds = contour.bounding_box().map_err(|err| op_err(cut.op, err))?;
        area = Some(match area {
            Some(current) => current.union(bounds),
            None => bounds,
        });
    }
    let Some(area) = area else {
        return Err(op_err(cut.op, "face has no geometry"));
    };
    let margin = cut.op.margin();
    let min = area.min();
    let max = area.max();
    let low = Point::new(min.x() - margin, min.y() - margin).map_err(|err| op_err(cut.op, err))?;
    let high = Point::new(max.x() + margin, max.y() + margin).map_err(|err| op_err(cut.op, err))?;
    Ok(BoundingBox::from_points(low, high))
}

fn drill_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let levels = cut_levels(cut.op)?;
    let Some(z) = levels.last().copied() else {
        return Err(op_err(cut.op, "depth plan is empty"));
    };
    let points: &[campanion_geom::Point] = &[];
    let mut circles = Vec::new();
    for contour in cut.contours {
        match DrillOp::new(DrillCycle::new()).z(z).tolerance(cut.tolerance).generate(std::slice::from_ref(contour), points) {
            Ok(drilling) if !drilling.hits().is_empty() => circles.push(contour.clone()),
            Ok(_) => {}
            Err(err) if is_not_circle(&err) => {}
            Err(err) => return Err(op_err(cut.op, err)),
        }
    }
    // No circles: ask again with the original contours so the error names the first rejection.
    let selected = if circles.is_empty() { cut.contours } else { circles.as_slice() };
    let mut cycle = DrillCycle::new();
    // `step_down` is the peck. A step that reaches the full depth is one feed, not a peck.
    if cut.op.step_down() < cut.op.depth() - 1e-9 {
        cycle = cycle.peck_depth(cut.op.step_down());
    }
    if let Some(seconds) = cut.op.dwell() {
        cycle = cycle.dwell(seconds);
    }
    let drilling = DrillOp::new(cycle).z(z).tolerance(cut.tolerance).generate(selected, points).map_err(|err| op_err(cut.op, err))?;
    if drilling.hits().is_empty() {
        return Err(op_err(cut.op, "drill produced no holes"));
    }
    let path = drilling.expand(0.0, cut.job.retract(), cut.job.clearance(), CutKind::Plunge).map_err(|err| op_err(cut.op, err))?;
    push_doc(paths, Document::from_toolpath(&path, cut.import)?)
}

fn is_not_circle(err: &impl std::fmt::Display) -> bool {
    let text = err.to_string();
    text.contains("not a circle") || text.contains("not a full circle")
}

/// One V-bit plunge per closed contour, then the same clearance link an engrave uses.
fn chamfer_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>, side: ProfileSide) -> Result<(), CliError> {
    let PathTool::Diameter { diameter } = cut.tool.path_tool() else {
        return Err(op_err(cut.op, "chamfer is not valid with a kerf tool"));
    };
    let mut features = Vec::with_capacity(cut.contours.len());
    for contour in cut.contours {
        if !contour.is_closed() {
            return Err(op_err(cut.op, "chamfer contour is open"));
        }
        let chamfer = ChamferOp::new(cut.op.v_angle(), cut.op.tip_diameter(), cut.op.width())
            .side(side)
            .tip_clearance(cut.op.tip_clearance())
            .stock_z(0.0)
            .max_diameter(diameter)
            .tolerance(cut.tolerance)
            .generate(contour)
            .map_err(|err| op_err(cut.op, err))?;
        features.push(Feature::new(contour.clone(), clearance_retracts(cut, chamfer.toolpath().moves())?));
    }
    link_operation(paths, cut, &features)
}

fn engrave_paths(paths: &mut Vec<ClPath>, cut: &Cut<'_>) -> Result<(), CliError> {
    let mut features = Vec::with_capacity(cut.contours.len());
    for contour in cut.contours {
        let toolpath = depth_at_retract(cut, contour)?;
        features.push(Feature::new(contour.clone(), toolpath));
    }
    link_operation(paths, cut, &features)
}

/// Depth passes from stock Z 0, with the same clearance and retract links as a pocket section.
/// Plunge moves use the operation plunge feed.
fn depth_at_retract(cut: &Cut<'_>, contour: &Contour) -> Result<Toolpath, CliError> {
    depth_with(cut, contour, cut.op.depth(), cut.op.step_down())
}

fn depth_with(cut: &Cut<'_>, contour: &Contour, total: f64, step_down: f64) -> Result<Toolpath, CliError> {
    let mut plan = DepthPlan::new(0.0, total, step_down, cut.op.entry());
    // Linking rapids to the retract height and the entry feed starts there. A ramp or helix
    // planned from the stock surface would be posted from that higher Z over a length sized
    // for the stock step, and the first descent would be steeper than angle_deg. Size the
    // first descent for the whole drop. Later passes already start at the previous level.
    // A pocket area pass does not come through here. `pocket_toolpath` applies this entry at
    // each depth, sized from the retract height for the whole drop to that level. A pocket
    // wall finish does come through here, and uses the same entry. Tabs do not change that
    // length: a ramp that meets a bridge lifts across it instead of cutting a steeper slot
    // through it.
    if matches!(cut.op.entry(), DepthEntry::Ramp { .. } | DepthEntry::Helix { .. }) {
        let retract = retract_z(cut.job);
        if retract > 0.0 {
            plan = plan.entry_start(retract);
        }
    }
    if let Some(spec) = cut.op.tabs() {
        let cutter = tool_radius(cut)? * 2.0;
        let on_contour = match cut.op.kind() {
            OperationKind::Profile { side } => matches!(side, ProfileSide::On),
            _ => false,
        };
        // Tab top follows the operation depth, not a roughing link that stops above the floor.
        plan = plan.with_tabs(DepthTabs::new(spec, cut.op.depth(), retract_z(cut.job), cutter).on_contour(on_contour));
    }
    let path = campanion_path::depth_toolpath(contour, &plan, &cut.tolerance).map_err(|err| op_err(cut.op, err))?;
    clearance_retracts(cut, path.moves())
}

/// Moves through the last process-off. Anything after that is the generator retract, which the
/// clearance link replaces with a rapid.
fn moves_through_process_off(path: &Toolpath) -> Vec<Move> {
    let moves = path.moves();
    let end = match moves.iter().rposition(|mv| matches!(mv, Move::Event(ProcessEvent::Off))) {
        Some(index) => index.saturating_add(1),
        None => moves.len(),
    };
    moves.iter().take(end).copied().collect()
}

/// The first rapid sits at clearance so the linker does not feed down from there; the next rapid
/// drops to the job retract height. The return to clearance is a rapid inserted after process-off.
/// A spindle's process-off moves to after that rapid later. A torch or laser keeps the stop, and
/// its post delay, here.
fn clearance_retracts(cut: &Cut<'_>, source: &[Move]) -> Result<Toolpath, CliError> {
    let retract = cut.job.retract();
    let clearance = cut.job.clearance();
    let mut moves = Vec::with_capacity(source.len().saturating_add(2));
    let mut saw_rapid = false;
    let mut at: Option<Point3> = None;
    for mv in source {
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
    let params = cut.op.params_with_speed(cut.tool).map_err(|err| op_err(cut.op, err))?;
    let doc = restore_process_params(Document::from_toolpath(&timed, &motion)?, &params)?;
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
        program.rotary().copied(),
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

fn tool_radius(cut: &Cut<'_>) -> Result<f64, CliError> {
    cut.tool.path_tool().compensation_distance().map_err(|err| op_err(cut.op, err))
}

fn compensate_profile(cut: &Cut<'_>, side: ProfileSide, contour: &Contour, distance: f64) -> Result<Contour, CliError> {
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
    depth_levels(op, op.depth(), op.step_down())
}

fn depth_levels(op: &Operation, total: f64, step_down: f64) -> Result<Vec<f64>, CliError> {
    let depths = campanion_path::pass_depths(total, step_down).map_err(|err| op_err(op, err))?;
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

pub(crate) fn write_output(path: &Path, text: &str) -> Result<(), CliError> {
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

#[cfg(test)]
mod tests {
    use super::run;
    use crate::jobfile::hash_bytes;
    use std::ffi::OsString;
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    fn ticks() -> u128 {
        SystemTime::now().duration_since(UNIX_EPOCH).map(|duration| duration.as_nanos()).unwrap_or(0)
    }

    fn dir(name: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!("campanion-{name}-{}", ticks()));
        fs::create_dir_all(&path).expect("dir");
        path
    }

    fn args(job: &Path, ngc: &Path, ir: &Path) -> Vec<OsString> {
        vec![
            job.as_os_str().to_os_string(),
            OsString::from("-o"),
            ngc.as_os_str().to_os_string(),
            OsString::from("--ir"),
            ir.as_os_str().to_os_string(),
        ]
    }

    fn job_text(hash: &str, kinematics: &str, plane: &str) -> String {
        format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"sample.dxf","hash":"{hash}"}},"units":"mm"{kinematics},"tools":[{{"id":1,"diameter":6}}],"operations":[{{"name":"inner","type":"profile","layers":["INNER"],"side":"inside","tool_id":1,"depth":3,"step_down":3{plane},"feeds":{{"cut":1000,"plunge":300}}}}]}}"#
        )
    }

    #[test]
    fn kinematics_without_a_plane_plans_like_a_three_axis_job() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let plain_dir = dir("kin-plain");
        let kin_dir = dir("kin-five");
        fs::copy(root.join("examples/sample.dxf"), plain_dir.join("sample.dxf")).expect("dxf");
        fs::copy(root.join("examples/sample.dxf"), kin_dir.join("sample.dxf")).expect("dxf");
        fs::copy(root.join("examples/table_table_bc.kin.json"), kin_dir.join("table_table_bc.kin.json")).expect("kin");
        let hash = hash_bytes(&fs::read(plain_dir.join("sample.dxf")).expect("bytes"));
        fs::write(plain_dir.join("job.json"), job_text(&hash, "", "")).expect("job");
        fs::write(kin_dir.join("job.json"), job_text(&hash, r#","kinematics":"table_table_bc.kin.json""#, "")).expect("job");
        let plain_ngc = plain_dir.join("out.ngc");
        let kin_ngc = kin_dir.join("out.ngc");
        let plain_ir = plain_dir.join("out.ir.json");
        let kin_ir = kin_dir.join("out.ir.json");
        must_run(&args(&plain_dir.join("job.json"), &plain_ngc, &plain_ir));
        must_run(&args(&kin_dir.join("job.json"), &kin_ngc, &kin_ir));
        assert_eq!(fs::read(&plain_ngc).expect("plain ngc"), fs::read(&kin_ngc).expect("kin ngc"));
        let plain_value: serde_json::Value = serde_json::from_str(&fs::read_to_string(&plain_ir).expect("plain ir")).expect("plain json");
        let mut kin_value: serde_json::Value = serde_json::from_str(&fs::read_to_string(&kin_ir).expect("kin ir")).expect("kin json");
        let program = kin_value["program"].as_object_mut().expect("program");
        let five = program.remove("five_axis").expect("five_axis");
        assert_eq!(five["policy"], "nearest");
        assert_eq!(five["kinematics"]["name"], "table_table_bc");
        assert_eq!(kin_value, plain_value);
    }

    #[test]
    fn flat_plane_is_planned_in_the_part_frame() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let folder = dir("kin-tilt");
        fs::copy(root.join("examples/sample.dxf"), folder.join("sample.dxf")).expect("dxf");
        fs::copy(root.join("examples/table_table_bc.kin.json"), folder.join("table_table_bc.kin.json")).expect("kin");
        let hash = hash_bytes(&fs::read(folder.join("sample.dxf")).expect("bytes"));
        let plane = r#","plane":{"origin":[0,0,0],"normal":[0,0,1]}"#;
        fs::write(folder.join("job.json"), job_text(&hash, r#","kinematics":"table_table_bc.kin.json""#, plane)).expect("job");
        must_run(&args(&folder.join("job.json"), &folder.join("out.ngc"), &folder.join("out.ir.json")));
        let ir: serde_json::Value = serde_json::from_str(&fs::read_to_string(folder.join("out.ir.json")).expect("ir")).expect("json");
        assert_eq!(ir["program"]["five_axis"]["kinematics"]["name"], "table_table_bc");
        assert!(folder.join("out.ngc").is_file());
    }

    fn must_run(args: &[OsString]) {
        if let Err(err) = run(args) {
            panic!("{err}");
        }
    }
}
