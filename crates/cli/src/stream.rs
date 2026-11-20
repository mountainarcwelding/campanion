//! `stream`: send posted G-code to a Grbl-class controller over a serial port.
//!
//! Follows the public Grbl v1.1 interface and commands documents. This command has not
//! been hardware-tested. First run: low power, hand on the power switch.

use std::ffi::OsString;
use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

use campanion_stream::{JobKind, Session, StreamEvent, list_ports, mode_from_profile, open_named, parse_baud, parse_rx_buffer};

use crate::error::CliError;

struct StreamArgs {
    port: String,
    baud: u32,
    rx_buffer: Option<usize>,
    safe_mode: bool,
    file: PathBuf,
    list: bool,
}

pub fn stream(args: &[OsString]) -> Result<std::process::ExitCode, CliError> {
    let parsed = parse_stream(args)?;
    if parsed.list {
        let ports = list_ports(false);
        if ports.is_empty() {
            let _ = writeln!(
                io::stderr(),
                "campanion: no serial ports found. On Linux, membership in the dialout group is typically required to open USB serial devices."
            );
        } else {
            for port in ports {
                println!("{port}");
            }
        }
        return Ok(std::process::ExitCode::SUCCESS);
    }
    if parsed.port.is_empty() {
        return Err(CliError::usage("stream needs --port (or --list)"));
    }
    let mode = mode_from_profile(parsed.rx_buffer, parsed.safe_mode).map_err(|err| CliError::usage(err.to_string()))?;
    let text = fs::read_to_string(&parsed.file).map_err(|err| CliError::usage(format!("cannot read {}: {err}", parsed.file.display())))?;
    let lines: Vec<String> = text.lines().map(str::to_string).collect();
    if lines.is_empty() {
        return Err(CliError::usage("the G-code file is empty"));
    }
    let _ = writeln!(io::stderr(), "campanion: NOT hardware-tested. First run: low power, hand on the power switch.");
    let hint = parsed.rx_buffer.unwrap_or(4096);
    // character-counting still uses parsed.rx_buffer / safe mode, never this number.
    let transport = open_named(&parsed.port, parsed.baud, hint).map_err(|err| CliError::usage(err.to_string()))?;
    let mut session = Session::new(mode, transport);
    for event in session.open().map_err(|err| CliError::usage(err.to_string()))? {
        print_event(&event);
    }
    for event in session.start(lines, JobKind::Program).map_err(|err| CliError::usage(err.to_string()))? {
        print_event(&event);
    }
    let started = Instant::now();
    while matches!(
        session.phase(),
        campanion_stream::HostPhase::Streaming | campanion_stream::HostPhase::Holding | campanion_stream::HostPhase::Stopping
    ) {
        if started.elapsed() > Duration::from_secs(24 * 60 * 60) {
            break;
        }
        match session.poll_status() {
            Ok(events) => {
                for event in events {
                    print_event(&event);
                }
            }
            Err(err) => {
                let _ = writeln!(io::stderr(), "campanion: {err}");
                return Ok(std::process::ExitCode::from(1));
            }
        }
        thread::sleep(Duration::from_millis(50));
    }
    // Final drain.
    if let Ok(events) = session.poll_status() {
        for event in events {
            print_event(&event);
        }
    }
    if session.streamer.last_error().is_some() || session.streamer.last_alarm().is_some() {
        return Ok(std::process::ExitCode::from(1));
    }
    Ok(std::process::ExitCode::SUCCESS)
}

fn print_event(event: &StreamEvent) {
    match event {
        StreamEvent::Welcome(text) | StreamEvent::Push(text) | StreamEvent::Message(text) => {
            println!("{text}");
        }
        StreamEvent::Status(status) => {
            if let Some(pos) = status.work_or_computed() {
                println!("status {} WPos:{:.3},{:.3},{:.3}", status.raw, pos[0], pos[1], pos[2]);
            } else {
                println!("status {}", status.raw);
            }
        }
        StreamEvent::Ack { index, .. } => println!("ok line {}", index + 1),
        StreamEvent::Error { code, text, index } => {
            let where_ = index.map(|i| format!(" on line {}", i + 1)).unwrap_or_default();
            let _ = writeln!(io::stderr(), "campanion: error:{code}{where_}: {text}");
        }
        StreamEvent::Alarm { code, text } => {
            let _ = writeln!(io::stderr(), "campanion: ALARM:{code}: {text}");
        }
        StreamEvent::Stopped { m5_verified, last_line } => {
            println!("stopped; M5 verified: {m5_verified}; last line: {last_line:?}");
        }
        StreamEvent::JobFinished { .. } => println!("job finished"),
    }
}

fn parse_stream(args: &[OsString]) -> Result<StreamArgs, CliError> {
    let mut port = String::new();
    let mut baud_text = String::new();
    let mut rx_buffer = None;
    let mut safe_mode = false;
    let mut machine: Option<PathBuf> = None;
    let mut file: Option<PathBuf> = None;
    let mut list = false;
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg == "--list" {
            list = true;
        } else if arg == "--send-response" {
            safe_mode = true;
        } else if arg == "--port" {
            port = required_value(args, &mut index, "--port")?.to_string_lossy().into_owned();
        } else if arg == "--baud" {
            baud_text = required_value(args, &mut index, "--baud")?.to_string_lossy().into_owned();
        } else if arg == "--rx-buffer" {
            let value = required_value(args, &mut index, "--rx-buffer")?;
            rx_buffer = parse_rx_buffer(&value.to_string_lossy()).map_err(|err| CliError::usage(err.to_string()))?;
        } else if arg == "--machine" {
            machine = Some(PathBuf::from(required_value(args, &mut index, "--machine")?));
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(CliError::usage("unknown argument"));
        } else if file.is_some() {
            return Err(CliError::usage("unexpected arguments"));
        } else {
            file = Some(PathBuf::from(arg));
        }
    }
    if let (None, Some(path)) = (rx_buffer, machine.as_ref()) {
        rx_buffer = load_profile_rx_buffer(path)?;
    }
    if list {
        return Ok(StreamArgs { port, baud: 0, rx_buffer, safe_mode, file: PathBuf::new(), list });
    }
    if baud_text.is_empty() {
        return Err(CliError::usage("stream needs --baud; Grbl documents 115200 8-N-1, but nothing is assumed"));
    }
    let baud = parse_baud(&baud_text).map_err(|err| CliError::usage(err.to_string()))?;
    let Some(file) = file else {
        return Err(CliError::usage("stream needs a G-code file"));
    };
    Ok(StreamArgs { port, baud, rx_buffer, safe_mode, file, list })
}

fn required_value<'a>(args: &'a [OsString], index: &mut usize, flag: &str) -> Result<&'a OsString, CliError> {
    let Some(value) = args.get(*index) else {
        return Err(CliError::usage(format!("{flag} needs a value")));
    };
    *index += 1;
    Ok(value)
}

fn load_profile_rx_buffer(path: &Path) -> Result<Option<usize>, CliError> {
    let text = fs::read_to_string(path).map_err(|err| CliError::usage(format!("cannot read {}: {err}", path.display())))?;
    let profile = crate::profiles::parse_profile(&text).map_err(CliError::usage)?;
    Ok(profile.rx_buffer)
}
