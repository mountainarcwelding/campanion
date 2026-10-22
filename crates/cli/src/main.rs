//! Command-line interface for CAMpanion.
//!
//! `ir` and `ngc` turn a 2D profile job into IR JSON or test-sink RS274/NGC.
//! `run` loads a `campanion-job/1` file, imports its DXF, and writes toolpaths.
//! `verify` reports motion and process problems in an IR file. `view` opens the toolpath preview
//! by launching the separate `campanion-view` program. `schema` prints the JSON Schema.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod error;
mod job;
// `run` reads the file and does not rewrite the source hash. Tests cover the rest of the API.
#[allow(dead_code)]
mod jobfile;
mod report;
mod run;
mod surface;
// STEP faces are exported by shelling out to SolidCraft. `run` does not call this yet.
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
  campanion run JOB -o OUT.ngc [--ir OUT.json]
  campanion verify FILE
  campanion view IR.json
  campanion schema
  campanion --help
  campanion --version

ir      Print IR JSON for the built-in sample 2D profile, or for a JSON job file.
ngc     Print test-sink RS274/NGC for that job. This is not a post for a real machine.
run     Import the DXF, SVG, STEP or STL (by extension) named by a campanion-job/1 file and write one toolpath program.
        JOB is the job file. -o is the RS274/NGC test sink. --ir also writes IR JSON.
        A changed source hash is a warning on stderr. The job file is not rewritten.
        Verify errors are printed and the status is 1. Warnings stay on stderr.
verify  Verify an IR JSON file and print a report. The status is 1 when the report has errors.
view    Open the toolpath preview (the campanion-view program) for an IR JSON file.
schema  Print the JSON Schema for campanion-ir/1.

--job FILE
        Read a JSON job instead of the built-in sample. The job is one profile: process,
        cut_feed, tool, optional feeds and params, and profile points in millimetres.
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

fn verify(args: &[OsString]) -> Result<ExitCode, CliError> {
    let [path] = args else {
        return Err(CliError::usage("verify needs one IR JSON file"));
    };
    if path.is_empty() {
        return Err(CliError::usage("verify needs one IR JSON file"));
    }
    let text = job::read_text(Path::new(path))?;
    let doc = campanion_ir::Document::from_json(&text)?;
    let report = campanion_ir::verify(doc.program());
    let failed = !report.issues().is_empty();
    write_stdout(&report::format_report(&report))?;
    if failed { Ok(ExitCode::from(1)) } else { Ok(ExitCode::SUCCESS) }
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

fn write_stdout(text: &str) -> Result<ExitCode, CliError> {
    let mut out = io::stdout().lock();
    out.write_all(text.as_bytes())?;
    if !text.ends_with('\n') {
        writeln!(out)?;
    }
    out.flush()?;
    Ok(ExitCode::SUCCESS)
}
