//! A spindle stays on through one tool's depth passes and operations (refs #93).
//!
//! Laser and plasma posts cycle every process, including spindle, off before a rapid and on
//! again before the next feed. A torch process keeps that per-cut cycle in the IR as well.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const POCKETS_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="130mm" height="80mm" viewBox="0 0 130 80">"#,
    r#"<g id="RING"><rect x="10" y="10" width="60" height="50"/>"#,
    r#"<rect x="25" y="22" width="30" height="26"/></g>"#,
    r#"<g id="SQUARE"><rect x="90" y="20" width="24" height="24"/></g>"#,
    "</svg>",
);

const PART_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="120mm" height="80mm" viewBox="0 0 120 80">"#,
    r#"<g id="OUTLINE"><rect x="10" y="10" width="100" height="60"/></g>"#,
    r#"<g id="SLOT"><rect x="40" y="30" width="40" height="20"/></g>"#,
    "</svg>",
);

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let path = dir.join(name);
    std::fs::create_dir_all(&path).map_err(|err| err.to_string())?;
    Ok(path)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(cwd: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn write_svg(dir: &Path, name: &str, svg: &str) -> Result<String, String> {
    std::fs::write(dir.join(name), svg).map_err(|err| err.to_string())?;
    Ok(fnv1a64(svg.as_bytes()))
}

fn job_json(source: &str, hash: &str, tools: &str, operations: &str) -> String {
    format!(r#"{{"format":"campanion-job/1","source":{{"path":"{source}","hash":"{hash}"}},"units":"mm","tools":{tools},"operations":{operations}}}"#)
}

fn run_job(dir: &Path, job: &str, out_name: &str, post: Option<&str>) -> Result<String, String> {
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join(out_name);
    let out_arg = path_arg(&out)?;
    let mut args = vec!["run", "job.json", "-o", out_arg.as_str()];
    if let Some(post) = post {
        args.push("--post");
        args.push(post);
    }
    let output = campanion(dir, &args)?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{post:?} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    std::fs::read_to_string(&out).map_err(|err| err.to_string())
}

fn strip_comments(line: &str) -> String {
    let mut out = String::new();
    let mut depth = 0i32;
    for ch in line.chars() {
        match ch {
            '(' => depth += 1,
            ')' if depth > 0 => depth -= 1,
            _ if depth == 0 => out.push(ch),
            _ => {}
        }
    }
    out.trim().to_string()
}

fn word_number(word: &str, letter: char) -> Option<f64> {
    let rest = word.strip_prefix(letter)?;
    if rest.is_empty() {
        return None;
    }
    rest.parse().ok()
}

fn tool_comment(line: &str) -> Option<u32> {
    let start = line.find("(tool ")?;
    let rest = &line[start + "(tool ".len()..];
    let end = rest.find(')')?;
    rest[..end].trim().parse().ok()
}

#[derive(Clone, Debug, PartialEq)]
enum Ev {
    Tool(u32),
    On(Option<u32>),
    Off,
}

struct Program {
    events: Vec<Ev>,
    /// Z at each process-off. The tool has to be above the stock for a spindle.
    off_z: Vec<f64>,
    /// Z at each process-off that is still at or below the stock, for a torch.
    cut_off_z: Vec<f64>,
    lifts_after_off: usize,
}

/// `lift_after_off` is for a torch or laser: process-off stays at the cut and a rapid follows it.
/// A spindle retracts first, so the program may end on process-off.
fn read_program(nc: &str, lift_after_off: bool) -> Result<Program, String> {
    let mut events = Vec::new();
    let mut off_z = Vec::new();
    let mut cut_off_z = Vec::new();
    let mut lifts_after_off = 0usize;
    let mut motion: Option<u8> = None;
    let mut x: Option<f64> = None;
    let mut y: Option<f64> = None;
    let mut z: Option<f64> = None;
    let mut waiting_lift = false;
    let mut off_at = 0.0f64;
    for (index, raw) in nc.lines().enumerate() {
        let line_no = index + 1;
        let code = strip_comments(raw);
        if code.is_empty() {
            if let Some(id) = tool_comment(raw) {
                events.push(Ev::Tool(id));
            }
            continue;
        }
        let words: Vec<&str> = code.split_whitespace().collect();
        if let Some(code) = words.iter().find_map(|word| word_number(word, 'G'))
            && code.fract() == 0.0
        {
            let g = code as u8;
            if matches!(g, 0..=3) {
                motion = Some(g);
            }
        }
        let next_x = words.iter().find_map(|word| word_number(word, 'X')).or(x);
        let next_y = words.iter().find_map(|word| word_number(word, 'Y')).or(y);
        let next_z = words.iter().find_map(|word| word_number(word, 'Z')).or(z);
        let moved = words.iter().any(|word| matches!(word.chars().next(), Some('X' | 'Y' | 'Z')));
        if moved {
            let mode = motion.ok_or_else(|| format!("line {line_no}: move before a G code\n{raw}"))?;
            let z_now = next_z.ok_or_else(|| format!("line {line_no}: move has no Z\n{raw}"))?;
            if mode == 0 && z_now < 2.0 - 1e-4 {
                return Err(format!("line {line_no}: G0 Z{z_now:.4} is below retract 2\n{raw}"));
            }
            let xy = match (x, y, next_x, next_y) {
                (Some(x0), Some(y0), Some(x1), Some(y1)) => (x0 - x1).abs() > 1e-4 || (y0 - y1).abs() > 1e-4,
                _ => false,
            };
            if mode == 0 && xy && (z_now - 5.0).abs() > 1e-3 {
                return Err(format!("line {line_no}: XY rapid at Z{z_now:.4}, expected clearance 5\n{raw}"));
            }
            if waiting_lift && mode == 0 && z_now > off_at + 1e-4 {
                lifts_after_off += 1;
                waiting_lift = false;
            }
            x = next_x;
            y = next_y;
            z = Some(z_now);
        }
        if words.iter().any(|word| word_number(word, 'M') == Some(6.0)) {
            let id = words
                .iter()
                .find_map(|word| word_number(word, 'T').map(|value| value as u32))
                .ok_or_else(|| format!("line {line_no}: M6 has no tool\n{raw}"))?;
            events.push(Ev::Tool(id));
        } else if let Some(id) = tool_comment(raw) {
            events.push(Ev::Tool(id));
        }
        for word in &words {
            let Some(code) = word_number(word, 'M') else {
                continue;
            };
            if code == 3.0 {
                if waiting_lift {
                    return Err(format!("line {line_no}: process-on before the lift after process-off\n{nc}"));
                }
                let speed = words.iter().find_map(|word| word_number(word, 'S').map(|value| value as u32));
                events.push(Ev::On(speed));
            } else if code == 5.0 {
                let at = z.ok_or_else(|| format!("line {line_no}: M5 before any move\n{nc}"))?;
                events.push(Ev::Off);
                off_z.push(at);
                if at <= 1e-6 {
                    cut_off_z.push(at);
                }
                if lift_after_off {
                    waiting_lift = true;
                    off_at = at;
                }
            }
        }
    }
    if lift_after_off && waiting_lift {
        return Err(format!("process-off was not followed by a lift\n{nc}"));
    }
    Ok(Program { events, off_z, cut_off_z, lifts_after_off })
}

fn count(events: &[Ev], want: &Ev) -> usize {
    events.iter().filter(|event| *event == want).count()
}

fn pocket_ops(layers: &str, speed: &str) -> String {
    format!(
        r#"[{{"name":"pockets","type":"pocket","layers":{layers},"tool_id":1,"depth":6.0,"step_down":3.0{speed},"feeds":{{"cut":1000.0,"plunge":300.0}}}}]"#
    )
}

#[test]
fn two_pockets_start_the_spindle_once_on_every_mill_post() -> Result<(), String> {
    let dir = temp_dir("campanion-spindle-hold")?;
    let hash = write_svg(&dir, "pockets.svg", POCKETS_SVG)?;
    let job = job_json("pockets.svg", &hash, r#"[{"id":1,"diameter":6.0,"speed":12000}]"#, &pocket_ops(r#"["RING","SQUARE"]"#, ""));
    let sink = run_job(&dir, &job, "out.ngc", None)?;
    let sink_program = read_program(&sink, false)?;
    if sink_program.events.iter().filter(|event| matches!(event, Ev::On(_))).count() != 1 || count(&sink_program.events, &Ev::Off) != 1 {
        return Err(format!("test sink events {:?}\n{sink}", sink_program.events));
    }
    if sink_program.off_z.iter().any(|z| *z <= 0.0) {
        return Err(format!("test sink stops at {:?}\n{sink}", sink_program.off_z));
    }
    for post in ["mach3", "linuxcnc", "haas", "grbl"] {
        let nc = run_job(&dir, &job, &format!("out-{post}.nc"), Some(post))?;
        let program = read_program(&nc, false).map_err(|err| format!("{post}: {err}"))?;
        let ons = program.events.iter().filter(|event| matches!(event, Ev::On(_))).count();
        let offs = count(&program.events, &Ev::Off);
        if ons != 1 || offs != 1 {
            return Err(format!("{post}: {ons} M3 and {offs} M5, expected 1 and 1\n{nc}"));
        }
        if program.events.iter().any(|event| matches!(event, Ev::On(Some(speed)) if *speed != 12000)) {
            return Err(format!("{post}: unexpected speed {:?}\n{nc}", program.events));
        }
        if program.off_z.iter().any(|z| *z <= 0.0) {
            return Err(format!("{post}: M5 at {:?}, expected Z above the stock\n{nc}", program.off_z));
        }
    }
    Ok(())
}

#[test]
fn two_tools_stop_before_the_next_tool_change() -> Result<(), String> {
    let dir = temp_dir("campanion-spindle-two-tool")?;
    let hash = write_svg(&dir, "part.svg", PART_SVG)?;
    let tools = r#"[{"id":1,"diameter":6.0,"speed":12000},{"id":2,"diameter":3.0,"speed":8000}]"#;
    let operations = r#"[{
      "name":"slot","type":"pocket","layers":["SLOT"],"tool_id":2,"depth":3.0,"step_down":1.5,
      "feeds":{"cut":1500.0,"plunge":500.0}
    },{
      "name":"outline","type":"profile","layers":["OUTLINE"],"side":"outside","tool_id":1,"depth":6.0,"step_down":3.0,
      "feeds":{"cut":1500.0,"plunge":500.0}
    }]"#;
    let job = job_json("part.svg", &hash, tools, operations);
    for post in ["mach3", "linuxcnc", "haas", "grbl"] {
        let nc = run_job(&dir, &job, &format!("out-{post}.nc"), Some(post))?;
        let program = read_program(&nc, false).map_err(|err| format!("{post}: {err}"))?;
        let want = vec![Ev::Tool(2), Ev::On(Some(8000)), Ev::Off, Ev::Tool(1), Ev::On(Some(12000)), Ev::Off];
        if program.events != want {
            return Err(format!("{post}: events {:?}, expected {want:?}\n{nc}", program.events));
        }
        if program.off_z.iter().any(|z| *z <= 0.0) {
            return Err(format!("{post}: M5 at {:?}\n{nc}", program.off_z));
        }
    }
    Ok(())
}

#[test]
fn a_speed_change_writes_the_new_speed_without_stopping() -> Result<(), String> {
    let dir = temp_dir("campanion-spindle-speed")?;
    let hash = write_svg(&dir, "pockets.svg", POCKETS_SVG)?;
    let operations = r#"[{
      "name":"ring","type":"pocket","layers":["RING"],"tool_id":1,"depth":6.0,"step_down":3.0,"speed":10000,
      "feeds":{"cut":1000.0,"plunge":300.0}
    },{
      "name":"square","type":"pocket","layers":["SQUARE"],"tool_id":1,"depth":6.0,"step_down":3.0,"speed":16000,
      "feeds":{"cut":1000.0,"plunge":300.0}
    }]"#;
    let job = job_json("pockets.svg", &hash, r#"[{"id":1,"diameter":6.0,"speed":12000}]"#, operations);
    for post in ["mach3", "linuxcnc", "haas", "grbl"] {
        let nc = run_job(&dir, &job, &format!("out-{post}.nc"), Some(post))?;
        let program = read_program(&nc, false).map_err(|err| format!("{post}: {err}"))?;
        let want = vec![Ev::Tool(1), Ev::On(Some(10000)), Ev::On(Some(16000)), Ev::Off];
        if program.events != want {
            return Err(format!("{post}: events {:?}, expected {want:?}\n{nc}", program.events));
        }
        if program.off_z.iter().any(|z| *z <= 0.0) {
            return Err(format!("{post}: M5 at {:?}\n{nc}", program.off_z));
        }
    }
    Ok(())
}

#[test]
fn plasma_still_stops_at_each_cut_before_the_lift() -> Result<(), String> {
    let dir = temp_dir("campanion-spindle-plasma")?;
    let hash = write_svg(&dir, "pockets.svg", POCKETS_SVG)?;
    let operations = r#"[{
      "name":"square","type":"pocket","layers":["SQUARE"],"tool_id":1,"depth":6.0,"step_down":3.0,
      "feeds":{"cut":1000.0,"plunge":300.0}
    }]"#;
    let job = job_json("pockets.svg", &hash, r#"[{"id":1,"kerf":1.0}]"#, operations);
    let nc = run_job(&dir, &job, "out-plasma.nc", Some("plasma"))?;
    let program = read_program(&nc, true)?;
    let ons = program.events.iter().filter(|event| matches!(event, Ev::On(_))).count();
    let offs = count(&program.events, &Ev::Off);
    if ons != 2 || offs != 2 {
        return Err(format!("plasma events {:?}, expected two on and two off\n{nc}", program.events));
    }
    if program.cut_off_z.len() != 2 || program.lifts_after_off != 2 {
        return Err(format!("plasma off z {:?} lifts {}\n{nc}", program.off_z, program.lifts_after_off));
    }
    Ok(())
}

fn is_sequence_word(word: &str) -> bool {
    word.starts_with('N') && word[1..].chars().all(|ch| ch.is_ascii_digit())
}

/// Every G0 is dark, and every feed is lit. At least one dark rapid sits between two cuts.
fn assert_process_off_before_travel(nc: &str, post: &str) -> Result<(), String> {
    let mut motion: Option<u8> = None;
    let mut lit = false;
    let mut ons = 0usize;
    let mut offs = 0usize;
    let mut cuts = 0usize;
    let mut travels = 0usize;
    let mut pending = false;
    for (index, raw) in nc.lines().enumerate() {
        let line_no = index + 1;
        let code = strip_comments(raw);
        if code.is_empty() {
            continue;
        }
        let words: Vec<&str> = code.split_whitespace().filter(|word| !is_sequence_word(word)).collect();
        if let Some(code) = words.iter().find_map(|word| word_number(word, 'M'))
            && code.fract() == 0.0
        {
            let m = code as u16;
            let speed = words.iter().find_map(|word| word_number(word, 'S'));
            if m == 5 {
                if !lit {
                    return Err(format!("{post} line {line_no}: M5 while the process is already off\n{raw}"));
                }
                lit = false;
                offs += 1;
            } else if m == 3 || (m == 4 && speed != Some(0.0)) {
                lit = true;
                ons += 1;
            }
        }
        if let Some(code) = words.iter().find_map(|word| word_number(word, 'G'))
            && code.fract() == 0.0
            && (0.0..4.0).contains(&code)
        {
            motion = Some(code as u8);
        }
        let moved = words.iter().any(|word| matches!(word.chars().next(), Some('X' | 'Y' | 'Z')));
        if !moved {
            continue;
        }
        let mode = motion.ok_or_else(|| format!("{post} line {line_no}: move before a G code\n{raw}"))?;
        if mode == 0 {
            if lit {
                return Err(format!("{post} line {line_no}: G0 while the torch or beam is on\n{nc}"));
            }
            if cuts > 0 {
                pending = true;
            }
        } else if matches!(mode, 1..=3) {
            if !lit {
                return Err(format!("{post} line {line_no}: cut while the torch or beam is off\n{nc}"));
            }
            if pending {
                travels += 1;
                pending = false;
            }
            cuts += 1;
        }
    }
    if ons < 2 || offs != ons || travels < 1 {
        return Err(format!("{post}: {ons} starts, {offs} stops, {travels} dark rapids between cuts\n{nc}"));
    }
    Ok(())
}

#[test]
fn spindle_process_cycles_per_cut_on_plasma_and_laser() -> Result<(), String> {
    let dir = temp_dir("campanion-spindle-per-cut")?;
    let hash = write_svg(&dir, "pockets.svg", POCKETS_SVG)?;
    let job = job_json("pockets.svg", &hash, r#"[{"id":1,"diameter":6.0,"speed":12000}]"#, &pocket_ops(r#"["RING","SQUARE"]"#, ""));
    for post in ["plasma", "grbl_laser"] {
        let nc = run_job(&dir, &job, &format!("out-{post}.nc"), Some(post))?;
        assert_process_off_before_travel(&nc, post)?;
    }
    for post in ["mach3", "linuxcnc", "haas", "grbl"] {
        let nc = run_job(&dir, &job, &format!("out-{post}.nc"), Some(post))?;
        let program = read_program(&nc, false).map_err(|err| format!("{post}: {err}"))?;
        let ons = program.events.iter().filter(|event| matches!(event, Ev::On(_))).count();
        let offs = count(&program.events, &Ev::Off);
        if ons != 1 || offs != 1 {
            return Err(format!("{post}: {ons} M3 and {offs} M5, expected 1 and 1\n{nc}"));
        }
        if program.off_z.iter().any(|z| *z <= 0.0) {
            return Err(format!("{post}: M5 at {:?}, expected Z above the stock\n{nc}", program.off_z));
        }
    }
    Ok(())
}
