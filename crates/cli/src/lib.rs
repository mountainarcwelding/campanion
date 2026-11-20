//! Command-line interface for CAMpanion, also used as a library by `campanion-gui`.
//!
//! `ir` and `ngc` turn a 2D profile job into IR JSON or test-sink RS274/NGC.
//! `run` loads a `campanion-job/1` or `campanion-job/2` file, imports its source, and writes toolpaths.
//! `lathe` loads a `campanion-lathe/1` file and writes turning toolpaths.
//! `post` writes a dialect for an IR file. `posts` lists the built-in posts.
//! `resin` slices an STL into a `.pm7m` for the Anycubic Photon Mono M7 Max; `resin-info` checks one.
//! `supports` writes a supported STL with a chamfered raft and pillar supports, including island detection.
//! `audit` re-checks a finished G-code file against a post's rules.
//! `verify` reports motion and process problems in an IR file, and can check an STL mesh for
//! gouges. `view` opens the toolpath preview
//! by launching the separate `campanion-view` program. `schema` prints the JSON Schema.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod collision;
mod error;
mod gouge;
mod index;
mod job;
mod job_parts;
mod job_plan;
mod job_setup;
mod kertilt;
mod lathe;
mod live_mill;
mod machine;
mod nest;
mod plane;
mod profiles;
// `run` reads the file and does not rewrite the source hash. Tests cover the rest of the API.
#[allow(dead_code)]
mod jobfile;
mod post;
mod raster;
mod report;
mod resin;
mod rotary;
mod rough;
mod supports;
// Public wrappers such as `plan_loaded_job` stay for the GUI and tests.
#[allow(dead_code)]
mod run;
mod safety;
mod stock;
mod stream;
mod surface;
mod toolaxis;
mod waterline;
mod wrap;
// STEP faces are exported by shelling out to SolidCraft.
pub mod engine;
#[allow(dead_code)]
mod step;

use std::env;
use std::ffi::{OsStr, OsString};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, ExitStatus, Stdio};

pub use engine::{
    ImportedPart, LayerSummary, PlanOptions, PlanResult, PostInfo, Posted, SourceKind, builtin_posts, import_part, parse_job, plan_job,
    plan_job_cancellable, plan_job_with_cancel, post_job, post_job_preview, posts_for_process, suggested_post,
};
pub use error::CliError;
pub use job_parts::{ContourOverride, JobInstance, JobPart, OpSelector, contour_ids};
pub use job_plan::{PlanCancel, PreflightItem, PreflightLevel, export_preflight, export_preflight_with, refuse_off_bed_for_export};
pub use job_setup::{Datum, DatumAnchor, JobOriginAnchor, OrderRules, SetupHooks, StartFrom, ZZero, bed_z_shift, datum_shift};
pub use jobfile::{
    DEFAULT_JOB_CHORD_MM, FORMAT_TAG, FORMAT_TAG_V2, JobFile, JobFileError, JobProfileSide, JobTool, Operation, OperationKind, Placement, Sheet,
    hash_bytes, load, parse, source_path,
};
pub use profiles::{
    BedSize, ChangerTool, FORMAT_TAG_V1, HeadChanger, HeadOffset, JobMachineRef, JobType, MachineFile, MachineHead, MachineKind, MaterialEntry,
    PROCESS_IDS, PowerUnit, PowerValue, ProfileStore, WorkArea, effective_post, job_type_for, kind_from_process, load_job_ref, load_path,
    parse_profile, profile_to_json, selected_head, valid_profile_id, with_store,
};

const HELP: &str = "\
Usage:
  campanion ir [--job FILE]
  campanion ngc [--job FILE]
  campanion run JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto] [--stock-report FILE.csv] [--stock-out FILE.json] [--stock-snapshots N] [--nest-out FILE.json]
  campanion lathe JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]
  campanion post IR.json --post ID|FILE.json [-o OUT]
  campanion posts
  campanion audit FILE.nc --post ID|FILE.json
  campanion resin MODEL.stl -o OUT.pm7m [--params-from SAMPLE.pm7m] [--layer-height MM] [--bottom-layers N] [--bottom-exposure S] [--exposure S]
                  [--bottom-lift-height MM] [--bottom-lift-speed V] [--lift-height MM] [--lift-speed V] [--resin-name NAME] [--keep-xy]
                  [--supports --support-params FILE.json | support flags] [--supports-out FILE.stl] [--preview FILE.png] [--points FILE] [--points-only]
                  [--detect-cell MM] [--island-ignore-area MM2] [--lowest-band-height MM] [--lowest-band-spacing MM]
                  [--calibration FILE.json] [--optimize] [--points-out FILE]
                  [--zones FILE.json --zone-spacing MM] [--tip-penetration MM] [--brace-diameter MM --brace-span MM --brace-pitch MM]
                  [--presets FILE.json] [--preset-mix ROLE=PRESET,...]
  campanion supports MODEL.stl -o OUT.stl [--support-params FILE.json] [--raft-chamfer-angle DEG] [--raft-thickness MM] [--raft-margin MM]
                  [--raft-corner-relief MM] [--overhang-angle DEG] [--support-spacing MM] [--tip-diameter MM] [--tip-length MM]
                  [--pillar-diameter MM] [--foot-diameter MM] [--foot-height MM] [--part-lift MM] [--points FILE] [--points-only] [--preview FILE.png]
                  [--layer-height MM] [--detect-cell MM] [--island-ignore-area MM2] [--lowest-band-height MM] [--lowest-band-spacing MM]
                  [--calibration FILE.json] [--optimize] [--points-out FILE]
                  [--zones FILE.json --zone-spacing MM] [--tip-penetration MM] [--brace-diameter MM --brace-span MM --brace-pitch MM]
                  [--presets FILE.json] [--preset-mix ROLE=PRESET,...]
  campanion resin-info FILE.pm7m
  campanion verify FILE [--mesh PART.stl [--tool ball:3|flat:3|bull:3,1]]
  campanion view IR.json
  campanion gui [FILE]
  campanion schema
  campanion --help
  campanion --version

ir      Print IR JSON for the built-in sample 2D profile, or for a JSON job file.
ngc     Print test-sink RS274/NGC for that job. This is not a post for a real machine.
run     Import the DXF, SVG, STEP or STL (by extension) named by a campanion-job/1 file and write one toolpath program.
        JOB is the job file. Without --post, -o is the RS274/NGC test sink. --ir also writes IR JSON.
        --post ID writes a built-in post. A value ending in .json is a custom post file.
        An unknown id lists the built-in ids. Posted text is read back and compared at the job
        chord tolerance. A mismatch exits 1 and nothing is written.
        --up-axis is z, y or auto (default z) for a STEP source. auto is opt-in and trusts the
        exporter header and turns Y up onto Z. A non-STEP source ignores the flag.
        A changed source hash is a warning on stderr. The job file is not rewritten.
        When the stock model is on, each operation prints its peak removal rate and peak
        engagement on stderr. --stock-report FILE.csv writes one row per feed move
        (operation, toolpath, move, volume, mrr, engagement_deg, axial_doc).
        --stock-out FILE.json writes the final height map. --stock-snapshots N (N >= 1)
        also stores a map at the start and after every Nth motion; it needs --stock-out.
        Those files are written when the replay succeeds. They must differ from -o and --ir.
        The flags are an error when the stock model is off. See docs/job-file.md.
        When the job sets sheet and nest, each part is packed on the sheet before the toolpaths
        are built. Placements are printed on stderr. --nest-out FILE.json writes them after the
        program is accepted. That file must differ from -o, --ir and the stock outputs.
        A part that does not fit, or that fails a clearance check, exits 1 and nothing is written.
        See docs/job-file.md.
        Verify errors are printed and the status is 1. Warnings stay on stderr.
        An STL job checks each cutting move against the mesh with that operation's cutter.
        A gouge is a verify error and the status is 1.
lathe   Import a campanion-lathe/1 job and write turning toolpaths. Units are millimetres.
        JOB is the job file. Without --post, -o is the RS274/NGC test sink. ZX arcs are G18.
        --ir also writes IR JSON. --post is an id or a file ending in .json, same as run.
        Posted text is read back and compared at the job chord tolerance.
        A mismatch exits 1 and nothing is written.
        Verify errors are printed and the status is 1. See docs/job-file.md.
        --up-axis is z, y or auto (default z) for a .step or .stp source. auto trusts the exporter
        header and turns Y up onto Z. A non-STEP source ignores the flag.
        Face, rough and finish paths are checked for insert and holder gouges.
        A gouge is printed and the status is 1. Nothing is written.
post    Post an IR JSON file. --post is an id or a file ending in .json, same as run.
        -o writes a file. Omit -o to print the G-code.
        Read-back uses the default job chord tolerance, 0.01 mm. A mismatch exits 1.
posts   List built-in post ids with their names and descriptions.
audit   Re-audit a finished G-code file. --post is an id or a file ending in .json, same as post.
        The status is 1 when the file is refused, including S above 0 in a file that records
        that power was not set.
resin   Slice an STL into layer bitmaps and write a .pm7m for the Anycubic Photon Mono M7 Max (6480 x 3600, 297.5 x 164 mm).
        NOT printer-tested. No exposure, lift or layer value has a default: give each flag, or --params-from an
        existing .pm7m (for example a slicer export that prints well) to take them from it; flags override it.
        Bottom layers use the --bottom-* values, the rest use --exposure, --lift-height and --lift-speed.
        The model is centred on the plate with its lowest point at the plate (--keep-xy uses the STL X/Y as mm
        from the plate centre). +X is image right and +Y is image up. A model that does not fit is refused.
        Pixels are fully on or off (no anti-aliasing yet). The finished file is read back before it is written.
        Pass --supports to add a chamfered raft and pillar supports first. Every support value is required
        (flags or --support-params FILE.json); nothing is assumed. --supports-out writes the supported STL.
        With --supports, island detection uses this command's --layer-height (and --detect-cell, or --support-spacing
        as the cell size). --optimize requires --calibration FILE.json; uniform and manual supports work without one.
supports    Write a watertight STL with a chamfered raft and vertical pillar supports under overhangs and islands.
        NOT printer-tested. CAMpanion is primarily command-line driven; the GUI is still in development.
        The raft's top outline is the part footprint plus --raft-margin; the bottom (bed side) is inset by
        thickness / tan(--raft-chamfer-angle), measured from the plate, so the raft is wider on the part
        side and forms an undercut a scraper can get under. An inset that collapses the bottom face is refused.
        --raft-corner-relief cuts each top corner back along both edges (0 keeps sharp corners).
        --overhang-angle is measured from vertical (0 supports every downward face; 90 only faces parallel to the plate).
        --part-lift is the gap from the raft top to the lowest point of the part. Units are millimetres.
        No numeric value has a default: give every flag or --support-params FILE.json (flags override it).
        --points FILE adds explicit x y, x y z, or x y z preset contacts; --points-only uses only those points.
        --points-out writes the placed points back in that format.
        --layer-height enables first-appearance island detection (including mid-air islands). --detect-cell sets the
        occupancy cell size; if omitted, --support-spacing is used. --island-ignore-area is optional.
        --lowest-band-height and --lowest-band-spacing must be given together for denser spacing near the first contact.
        --optimize runs the holding-force optimizer and requires --calibration FILE.json (a complete profile; no
        shipped defaults). Fitting measurements to constants is not implemented: enter fitted constants in the file.
        --zones FILE.json (campanion-resin-zones/1, painted in the GUI) marks part faces: support faces get supports at
        --zone-spacing even below the overhang angle; no-support faces never get an automatic support (--points stay).
        --tip-penetration sinks each tip straight into the part by that depth (optional; tips stop at the surface without it).
        --brace-diameter, --brace-span and --brace-pitch together add zig-zag cross-bracing between neighbouring pillars
        no farther apart than the span; struts that would touch the part are left out.
        --presets FILE.json (campanion-resin-presets/1) holds tip presets you saved; --preset-mix assigns them by role
        (overhang, island, lowest-band, painted, manual), e.g. island=Heavy,overhang=Light. A points file may name a
        preset in its fourth column. Roles left out use the base tip and pillar values. Not allowed with --optimize.
resin-info  Print the machine, layer count, height and layer settings of a .pm7m, after checking every layer image.
verify  Verify an IR JSON file and print a report. The status is 1 when the report has errors.
        --mesh checks cutting moves using IR tool shapes. --tool overrides all cutters and needs --mesh.
        The tool is a radius in millimetres: ball:3, flat:3 or bull:3,1 (bull corner after the comma).
        Tolerance is the job-file default chord tolerance, 0.01 mm. Samples are min(0.25, tolerance*10) mm apart.
        A gouge names the operation, the position and the depth. XY arcs are sampled; other arcs are rejected.
view    Open the toolpath preview (the campanion-view program) for an IR JSON file.
        --stock FILE.json draws the height map from --stock-out. S toggles that surface.
        --machine draws boxes and cylinders from the kinematics model. M toggles that sketch.
gui     Open the graphical workspace preview (the campanion-gui program). FILE may be a job file,
        a part, IR JSON, or G-code. CAMpanion is primarily command-line driven; the GUI is still
        in development. The Run tab can stream to a Grbl-class controller over USB serial.
        That path is NOT hardware-tested. First run: low power, hand on the power switch.
stream  Stream a posted G-code file to a Grbl-class controller over a serial port.
        --port PATH --baud RATE are required (Grbl documents 115200 8-N-1; nothing is assumed).
        --rx-buffer N uses character-counting with a size you enter; --machine FILE.json can
        supply that size from a profile. --send-response uses send-response safe mode and does
        not need a buffer size. --list prints serial ports. On Linux, the user typically needs
        to be in the dialout group to open a USB serial device.
        NOT hardware-tested. First run: low power, hand on the power switch.
schema  Print the JSON Schema for campanion-ir/1.

--job FILE
        Read a JSON job instead of the built-in sample. The job is one profile: process,
        cut_feed, tool, optional feeds, params and heights, and profile points in millimetres.
        heights.clearance and heights.retract default to 5 mm and 2 mm.
        Processes: spindle, torch, laser, extruder, knife.
        Sides: inside, outside, on. Directions: climb, conventional.
        Compensation: computed, controller_left, controller_right.
        A lead is line, arc, perpendicular or none.
";

pub fn run_cli() -> ExitCode {
    match try_main() {
        Ok(code) => code,
        Err(err) => {
            let mut err_out = io::stderr().lock();
            let _ = writeln!(err_out, "campanion: {err}");
            if matches!(err, CliError::Usage(_)) {
                let _ = err_out.write_all(HELP.as_bytes());
            }
            err.code()
        }
    }
}

pub fn try_main() -> Result<ExitCode, CliError> {
    let args: Vec<OsString> = env::args_os().skip(1).collect();
    if args.iter().any(|arg| is_help(arg)) {
        return write_stdout(HELP);
    }
    let Some((command, rest)) = args.split_first() else {
        return Err(CliError::usage("missing command"));
    };
    match command.as_os_str() {
        name if name == "help" => {
            if rest.is_empty() {
                write_stdout(HELP)
            } else {
                Err(CliError::usage("unexpected arguments"))
            }
        }
        name if name == "--version" || name == "-V" => {
            if rest.is_empty() {
                write_stdout(&format!("campanion {}\n", env!("CARGO_PKG_VERSION")))
            } else {
                Err(CliError::usage("--version does not take arguments"))
            }
        }
        name if name == "ir" => emit(rest, false),
        name if name == "ngc" => emit(rest, true),
        name if name == "run" => run::run(rest),
        name if name == "lathe" => lathe::lathe(rest),
        name if name == "post" => post::post_command(rest),
        name if name == "posts" => post::posts_command(rest),
        name if name == "audit" => post::audit_command(rest),
        name if name == "resin" => resin::resin(rest),
        name if name == "resin-info" => resin::resin_info(rest),
        name if name == "supports" => supports::supports_command(rest),
        name if name == "verify" => verify(rest),
        name if name == "view" => view(rest),
        name if name == "gui" => gui(rest),
        name if name == "stream" => stream::stream(rest),
        name if name == "schema" => schema(rest),
        _ => Err(CliError::usage("unknown command")),
    }
}

fn is_help(arg: &OsStr) -> bool {
    arg == "--help" || arg == "-h"
}

fn emit(args: &[OsString], ngc: bool) -> Result<ExitCode, CliError> {
    let doc = match take_job(args)? {
        Some(path) => job::load(&job::read_text(Path::new(path))?)?,
        None => job::load_sample()?,
    };
    let text = if ngc { campanion_ir::write_ngc(doc.program())? } else { doc.to_json_pretty()? };
    write_stdout(&text)
}

struct VerifyArgs {
    file: PathBuf,
    mesh: Option<(PathBuf, Option<String>)>,
}

fn verify(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_verify(args)?;
    let text = job::read_text(&args.file)?;
    let doc = campanion_ir::Document::from_json(&text)?;
    let gouges = if let Some((mesh_path, tool)) = &args.mesh {
        let cutter = tool.as_deref().map(gouge::parse_tool).transpose()?;
        if let Some(tool) = tool {
            writeln!(io::stderr().lock(), "mesh verification: --tool {tool} overrides cutter geometry for all cutting moves")?;
        }
        let bytes = run::read_bytes(mesh_path)?;
        let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
        match cutter {
            Some(cutter) => gouge::check_program(doc.program(), &mesh, cutter, jobfile::DEFAULT_JOB_CHORD_MM)?,
            None => gouge::check_program_tools(doc.program(), &mesh, jobfile::DEFAULT_JOB_CHORD_MM)?,
        }
    } else {
        Vec::new()
    };
    let report = campanion_ir::verify(doc.program());
    let failed = !report.issues().is_empty() || !gouges.is_empty();
    let text = if gouges.is_empty() { report::format_report(&report) } else { report::format_report_and_gouges(&report, &gouges) };
    write_stdout(&text)?;
    if failed { Ok(ExitCode::from(1)) } else { Ok(ExitCode::SUCCESS) }
}

fn parse_verify(args: &[OsString]) -> Result<VerifyArgs, CliError> {
    let mut file: Option<&OsStr> = None;
    let mut mesh: Option<&OsStr> = None;
    let mut tool: Option<&OsStr> = None;
    let mut index = 0usize;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "--mesh" || arg.as_os_str() == "--tool" {
            let flag = if arg.as_os_str() == "--mesh" { "--mesh" } else { "--tool" };
            let Some(value) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a value")));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a value")));
            }
            let slot = if flag == "--mesh" { &mut mesh } else { &mut tool };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(value.as_os_str());
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(CliError::usage("unknown argument"));
        } else if file.is_some() || arg.is_empty() {
            return Err(CliError::usage("verify needs one IR JSON file"));
        } else {
            file = Some(arg.as_os_str());
        }
    }
    let Some(file) = file else {
        return Err(CliError::usage("verify needs one IR JSON file"));
    };
    let mesh = match (mesh, tool) {
        (None, None) => None,
        (Some(mesh), Some(tool)) => {
            let Some(tool) = tool.to_str() else {
                return Err(CliError::usage("tool must be ball:R, flat:R or bull:R,C"));
            };
            Some((PathBuf::from(mesh), Some(tool.to_string())))
        }
        (Some(mesh), None) => Some((PathBuf::from(mesh), None)),
        (None, Some(_)) => return Err(CliError::usage("verify --tool needs --mesh")),
    };
    Ok(VerifyArgs { file: PathBuf::from(file), mesh })
}

fn view(args: &[OsString]) -> Result<ExitCode, CliError> {
    if args.first().is_none_or(|path| path.is_empty()) {
        return Err(CliError::usage("view needs an IR JSON file"));
    }
    launch_sidecar(
        "campanion-view",
        args,
        "campanion view needs the campanion-view program next to campanion or on PATH (cargo build -p campanion-view)",
    )
}

fn gui(args: &[OsString]) -> Result<ExitCode, CliError> {
    launch_sidecar("campanion-gui", args, "campanion gui needs the campanion-gui program next to campanion or on PATH (cargo build -p campanion-gui)")
}

fn launch_sidecar(file_stem: &str, args: &[OsString], missing: &str) -> Result<ExitCode, CliError> {
    let program = find_sidecar(file_stem, missing)?;
    let status = Command::new(&program)
        .args(args)
        .stdin(Stdio::inherit())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .map_err(|err| CliError::run(format!("could not start {}: {err}", program.display())))?;
    Ok(exit_code_of(status))
}

/// Same directory as this executable, then `PATH`. No GUI crates are linked into this binary.
fn find_sidecar(file_stem: &str, missing: &str) -> Result<PathBuf, CliError> {
    let file_name = format!("{file_stem}{}", env::consts::EXE_SUFFIX);
    if let Ok(exe) = env::current_exe()
        && let Some(dir) = exe.parent()
    {
        let candidate = dir.join(&file_name);
        if candidate.is_file() {
            return Ok(candidate);
        }
    }
    if let Some(path_var) = env::var_os("PATH") {
        for dir in env::split_paths(&path_var) {
            let candidate = dir.join(&file_name);
            if candidate.is_file() {
                return Ok(candidate);
            }
        }
    }
    Err(CliError::run(missing))
}

fn exit_code_of(status: ExitStatus) -> ExitCode {
    match status.code() {
        Some(0) => ExitCode::SUCCESS,
        // Non-zero statuses stay the same code, clamped into the range `ExitCode::from` can carry.
        Some(code) => ExitCode::from(code.clamp(1, 255) as u8),
        None => ExitCode::from(1),
    }
}

fn schema(args: &[OsString]) -> Result<ExitCode, CliError> {
    if !args.is_empty() {
        return Err(CliError::usage("schema does not take arguments"));
    }
    let value = campanion_ir::json_schema()?;
    let text = serde_json::to_string_pretty(&value).map_err(|err| CliError::run(format!("json: {err}")))?;
    write_stdout(&text)
}

fn take_job(args: &[OsString]) -> Result<Option<&OsStr>, CliError> {
    match args {
        [] => Ok(None),
        [flag] if flag.as_os_str() == "--job" => Err(CliError::usage("--job needs a file")),
        [flag, path] if flag.as_os_str() == "--job" => {
            if path.is_empty() {
                Err(CliError::usage("--job needs a file"))
            } else {
                Ok(Some(path.as_os_str()))
            }
        }
        _ => Err(CliError::usage("unexpected arguments")),
    }
}

pub(crate) fn write_stdout(text: &str) -> Result<ExitCode, CliError> {
    let mut out = io::stdout().lock();
    out.write_all(text.as_bytes())?;
    if !text.ends_with('\n') {
        writeln!(out)?;
    }
    out.flush()?;
    Ok(ExitCode::SUCCESS)
}
