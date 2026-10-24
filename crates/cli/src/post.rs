//! Built-in and JSON posts.
//!
//! A `--post` value that ends in `.json` is loaded with [`campanion_post::load_post`]. Any other
//! value is a built-in id. Posted text is read back and compared before it is written.

use std::ffi::{OsStr, OsString};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use campanion_ir::Program;
use campanion_post::PostDef;

use crate::error::CliError;
use crate::job::read_text;
use crate::jobfile::DEFAULT_JOB_CHORD_MM;

/// Chord tolerance for [`post_command`], which has no job file. A job file uses this when `tolerance.chord` is omitted.
pub(crate) const DEFAULT_POST_TOLERANCE_MM: f64 = DEFAULT_JOB_CHORD_MM;

/// Load a built-in id, or a custom post whose path ends in `.json`.
pub(crate) fn load_spec(spec: &Path) -> Result<PostDef, CliError> {
    if ends_with_json(spec.as_os_str()) {
        let text = read_text(spec)?;
        return campanion_post::load_post(&text).map_err(|err| CliError::run(format!("post: {err}")));
    }
    let Some(id) = spec.to_str() else {
        return Err(CliError::run("post id must be UTF-8"));
    };
    if id.is_empty() {
        return Err(CliError::usage("--post needs a value"));
    }
    match campanion_post::builtin(id) {
        Some(post) => Ok(post),
        None => {
            let ids = campanion_post::builtin_ids().join(", ");
            Err(CliError::run(format!("unknown post `{id}`; available posts: {ids}")))
        }
    }
}

/// Write `program` and check that reading it back matches within `tolerance_mm` (the job chord tolerance).
pub(crate) fn render(program: &Program, post: &PostDef, tolerance_mm: f64) -> Result<String, CliError> {
    let options = campanion_post::PostOptions { arc_tolerance: tolerance_mm };
    let text = campanion_post::post_program(program, post, &options).map_err(|err| CliError::run(format!("post `{}`: {err}", post.id())))?;
    // Indexed 3+2 writes rotary words and work-offset codes that read-back does not parse.
    if post.has_five_axis() && program.five_axis().is_some() {
        print_channel_balance(program)?;
        return Ok(text);
    }
    let moves =
        campanion_post::read_back(&text, post).map_err(|err| CliError::run(format!("post `{}` output could not be read back: {err}", post.id())))?;
    campanion_post::compare(program, &moves, tolerance_mm, post)
        .map_err(|err| CliError::run(format!("post `{}` does not match the program within {tolerance_mm} mm: {err}", post.id())))?;
    print_channel_balance(program)?;
    Ok(text)
}

/// One stderr line per sync segment, then a total, when any binding uses a channel other than 0.
fn print_channel_balance(program: &Program) -> Result<(), CliError> {
    let multi = program.turning().is_some_and(|turning| turning.bindings().iter().any(|binding| binding.channel() != 0));
    if !multi {
        return Ok(());
    }
    let segments = campanion_post::channel_balance(program);
    let mut err = io::stderr().lock();
    let width = segments.first().map(|segment| segment.times_s.len()).unwrap_or(0);
    let mut sums = vec![0.0; width];
    let mut cycle = 0.0;
    for segment in &segments {
        let mut line = format!("segment {}", segment.segment);
        let mut peak = 0.0;
        for (channel, time) in segment.times_s.iter().enumerate() {
            let idle = segment.idle_s.get(channel).copied().unwrap_or(0.0);
            line.push_str(&format!(" channel {channel} time_s {time:.6} idle_s {idle:.6}"));
            if let Some(slot) = sums.get_mut(channel) {
                *slot += *time;
            }
            if *time > peak {
                peak = *time;
            }
        }
        cycle += peak;
        writeln!(err, "{line}")?;
    }
    let mut total = String::from("total");
    for (channel, time) in sums.iter().enumerate() {
        total.push_str(&format!(" channel {channel} time_s {time:.6}"));
    }
    writeln!(err, "{total} cycle_s {cycle:.6}")?;
    Ok(())
}

pub(crate) fn posts_command(args: &[OsString]) -> Result<ExitCode, CliError> {
    if !args.is_empty() {
        return Err(CliError::usage("posts does not take arguments"));
    }
    let mut text = String::new();
    for id in campanion_post::builtin_ids() {
        let Some(post) = campanion_post::builtin(id) else {
            return Err(CliError::run(format!("built-in post `{id}` did not load")));
        };
        text.push_str(&format!("{} {}\n  {}\n", post.id(), post.name(), post.description()));
    }
    crate::write_stdout(&text)
}

pub(crate) fn post_command(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_post(args)?;
    let text = read_text(&args.ir)?;
    let doc = campanion_ir::Document::from_json(&text)?;
    let post = load_spec(&args.post)?;
    let posted = render(doc.program(), &post, DEFAULT_POST_TOLERANCE_MM)?;
    if let Some(path) = &args.output {
        crate::run::write_output(path, &posted)?;
        Ok(ExitCode::SUCCESS)
    } else {
        crate::write_stdout(&posted)
    }
}

struct PostArgs {
    ir: PathBuf,
    post: PathBuf,
    output: Option<PathBuf>,
}

fn parse_post(args: &[OsString]) -> Result<PostArgs, CliError> {
    let mut ir: Option<&OsStr> = None;
    let mut post: Option<&OsStr> = None;
    let mut output: Option<&OsStr> = None;
    let mut index = 0usize;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "--post" || arg.as_os_str() == "-o" {
            let flag = if arg.as_os_str() == "--post" { "--post" } else { "-o" };
            let Some(value) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a value")));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a value")));
            }
            let slot = if flag == "--post" { &mut post } else { &mut output };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(value.as_os_str());
        } else if starts_with_dash(arg) {
            return Err(CliError::usage("unknown argument"));
        } else if ir.is_some() || arg.is_empty() {
            return Err(CliError::usage("post takes one IR JSON file"));
        } else {
            ir = Some(arg.as_os_str());
        }
    }
    let Some(ir) = ir else {
        return Err(CliError::usage("post needs an IR JSON file"));
    };
    let Some(post) = post else {
        return Err(CliError::usage("post needs --post ID or FILE.json"));
    };
    Ok(PostArgs { ir: PathBuf::from(ir), post: PathBuf::from(post), output: output.map(PathBuf::from) })
}

fn ends_with_json(value: &OsStr) -> bool {
    value.as_encoded_bytes().ends_with(b".json")
}

fn starts_with_dash(arg: &OsStr) -> bool {
    arg.to_string_lossy().starts_with('-')
}
