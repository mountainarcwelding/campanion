//! Window entry for the CAMpanion layout preview.
//!
//! `campanion-gui [FILE] [--screenshot DIR]` opens the shell. FILE may be IR JSON, a job file, or
//! G-code. `--screenshot DIR` writes PNG previews of the main views and exits. The window never
//! opens a serial port or moves a machine.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

use std::ffi::{OsStr, OsString};
use std::path::PathBuf;
use std::process::ExitCode;

use campanion_gui::app::{self, Launch};

fn main() -> ExitCode {
    match try_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("campanion-gui: {err}");
            ExitCode::from(1)
        }
    }
}

fn try_main() -> Result<(), String> {
    let args: Vec<OsString> = std::env::args_os().skip(1).collect();
    if args.iter().any(|arg| is_flag(arg, "--help") || is_flag(arg, "-h")) {
        print!("{HELP}");
        return Ok(());
    }
    if args.iter().any(|arg| is_flag(arg, "--version") || is_flag(arg, "-V")) {
        println!("campanion-gui {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }
    let launch = parse_args(&args)?;
    app::run(launch)
}

fn parse_args(args: &[OsString]) -> Result<Launch, String> {
    let mut open: Option<PathBuf> = None;
    let mut screenshot_dir: Option<PathBuf> = None;
    let mut index = 0usize;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if is_flag(arg, "--screenshot") {
            let Some(value) = args.get(index) else {
                return Err("--screenshot needs a directory".to_string());
            };
            index += 1;
            if value.is_empty() {
                return Err("--screenshot needs a directory".to_string());
            }
            if screenshot_dir.is_some() {
                return Err("--screenshot is already set".to_string());
            }
            screenshot_dir = Some(PathBuf::from(value));
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(format!("unknown argument {}", arg.to_string_lossy()));
        } else if open.is_some() {
            return Err("campanion-gui accepts at most one file".to_string());
        } else if arg.is_empty() {
            return Err("file path is empty".to_string());
        } else {
            open = Some(PathBuf::from(arg));
        }
    }
    Ok(Launch { open, screenshot_dir })
}

fn is_flag(arg: &OsStr, flag: &str) -> bool {
    arg == flag
}

const HELP: &str = "\
Usage:
  campanion-gui [FILE] [--screenshot DIR]
  campanion-gui --help
  campanion-gui --version

  FILE            Optional job, part (DXF/SVG/STEP/STL), IR JSON, or G-code to open.
  --screenshot DIR
                  Write PNG previews of the main views into DIR and exit.

CAMpanion is primarily command-line driven. This GUI is still in development.
It can open a part or job, generate toolpaths and export G-code.
It does not talk to a machine.
";
