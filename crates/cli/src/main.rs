//! Command-line interface for CAMpanion.
//!
//! `ir` and `ngc` turn a 2D profile job into IR JSON or test-sink RS274/NGC.
//! `run` loads a `campanion-job/1` file, imports its source, and writes toolpaths.
//! `lathe` loads a `campanion-lathe/1` file and writes turning toolpaths.
//! `post` writes a dialect for an IR file. `posts` lists the built-in posts.
//! `verify` reports motion and process problems in an IR file, and can check an STL mesh for
//! gouges. `view` opens the toolpath preview
//! by launching the separate `campanion-view` program. `schema` prints the JSON Schema.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod error;
mod gouge;
mod index;
mod job;
mod lathe;
mod live_mill;
mod machine;
mod plane;
// `run` reads the file and does not rewrite the source hash. Tests cover the rest of the API.
#[allow(dead_code)]
mod jobfile;
mod post;
mod report;
mod rotary;
mod rough;
mod run;
mod safety;
mod surface;
mod waterline;
mod wrap;
// STEP faces are exported by shelling out to SolidCraft.
#[allow(dead_code)]
mod step;

use std::env;
use std::ffi::{OsStr, OsString};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, ExitStatus, Stdio};

use error::CliError;

const HELP: &str = "\
Usage:
  campanion ir [--job FILE]
  campanion ngc [--job FILE]
  campanion run JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]
  campanion lathe JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]
  campanion post IR.json --post ID|FILE.json [-o OUT]
  campanion posts
  campanion verify FILE [--mesh PART.stl --tool ball:3|flat:3|bull:3,1]
  campanion view IR.json
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
verify  Verify an IR JSON file and print a report. The status is 1 when the report has errors.
        --mesh and --tool also check cutting moves against an STL mesh. Both are required together.
        The tool is a radius in millimetres: ball:3, flat:3 or bull:3,1 (bull corner after the comma).
        Tolerance is the job-file default chord tolerance, 0.01 mm. Samples are min(0.25, tolerance*10) mm apart.
        A gouge names the operation, the position and the depth. XY arcs are sampled; other arcs are rejected.
view    Open the toolpath preview (the campanion-view program) for an IR JSON file.
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

fn main() -> ExitCode {
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

fn try_main() -> Result<ExitCode, CliError> {
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
        name if name == "verify" => verify(rest),
        name if name == "view" => view(rest),
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
    mesh: Option<(PathBuf, String)>,
}

fn verify(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_verify(args)?;
    let text = job::read_text(&args.file)?;
    let doc = campanion_ir::Document::from_json(&text)?;
    let gouges = if let Some((mesh_path, tool)) = &args.mesh {
        let cutter = gouge::parse_tool(tool)?;
        let bytes = run::read_bytes(mesh_path)?;
        let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
        gouge::check_program(doc.program(), &mesh, cutter, jobfile::DEFAULT_JOB_CHORD_MM)?
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
            Some((PathBuf::from(mesh), tool.to_string()))
        }
        (Some(_), None) => return Err(CliError::usage("verify --mesh needs --tool")),
        (None, Some(_)) => return Err(CliError::usage("verify --tool needs --mesh")),
    };
    Ok(VerifyArgs { file: PathBuf::from(file), mesh })
}

fn view(args: &[OsString]) -> Result<ExitCode, CliError> {
    if args.first().is_none_or(|path| path.is_empty()) {
        return Err(CliError::usage("view needs an IR JSON file"));
    }
    let program = find_viewer()?;
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
fn find_viewer() -> Result<PathBuf, CliError> {
    let file_name = format!("campanion-view{}", env::consts::EXE_SUFFIX);
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
    Err(CliError::run("campanion view needs the campanion-view program next to campanion or on PATH (cargo build -p campanion-view)"))
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
