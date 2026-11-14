//! Straight oxy-fuel cuts: preheat, cutting oxygen, and oxygen off on rapids.
//! Feeds, delays, kerf and `min_preheat` in these jobs are example values, not machine data.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let root = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let dir = root.join(name);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn line(layer: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!("0\nLINE\n8\n{layer}\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n")
}

fn dxf(entities: &str) -> String {
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn closed_lines(layer: &str, points: &[(f64, f64)]) -> String {
    let mut entities = String::new();
    for pair in points.windows(2) {
        entities.push_str(&line(layer, pair[0].0, pair[0].1, pair[1].0, pair[1].1));
    }
    if let (Some(first), Some(last)) = (points.first(), points.last()) {
        entities.push_str(&line(layer, last.0, last.1, first.0, first.1));
    }
    dxf(&entities)
}

fn two_rectangles() -> String {
    let mut entities = String::new();
    for points in [&[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)][..], &[(120.0, 0.0), (200.0, 0.0), (200.0, 40.0), (120.0, 40.0)][..]] {
        for pair in points.windows(2) {
            entities.push_str(&line("PLATE", pair[0].0, pair[0].1, pair[1].0, pair[1].1));
        }
        if let (Some(first), Some(last)) = (points.first(), points.last()) {
            entities.push_str(&line("PLATE", last.0, last.1, first.0, first.1));
        }
    }
    dxf(&entities)
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn write_job(dir: &Path, drawing: &str, job: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn job(hash: &str, extra: &str, delay: f64) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, "kerf": 2.0 }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "outside",
    "tool_id": 1,
    "depth": 6.0,
    "step_down": 6.0,
    "feeds": {{ "cut": 400, "plunge": 200 }},
    "timing": {{ "pierce_delay": {delay} }},
    "lead_in": {{ "type": "none" }},
    "lead_out": {{ "type": "none" }}{extra}
  }} ]
}}"#
    )
}

fn run_post(dir: &Path, post: &str) -> Result<(String, String), String> {
    let out = dir.join(format!("{post}.nc"));
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
    let stderr = text(&output.stderr)?;
    if !output.status.success() {
        return Err(format!("run --post {post} exit {:?} stderr {stderr}", output.status.code()));
    }
    let nc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    Ok((nc, stderr))
}

fn without_comments(line: &str) -> String {
    let mut out = String::new();
    let mut chars = line.chars().peekable();
    while let Some(ch) = chars.next() {
        if ch == '(' {
            for next in chars.by_ref() {
                if next == ')' {
                    break;
                }
            }
        } else if ch == ';' {
            break;
        } else {
            out.push(ch);
        }
    }
    out
}

fn pin_word(line: &str) -> Option<&str> {
    line.split_whitespace().find(|word| word.starts_with('P') && word.len() > 1)
}

fn oxyfuel_marks(text: &str) -> Vec<&'static str> {
    let mut marks = Vec::new();
    for line in text.lines() {
        let stripped = without_comments(line);
        let words: Vec<&str> = stripped.split_whitespace().collect();
        if words.contains(&"M64") || words.contains(&"M65") {
            let pin = pin_word(&stripped).unwrap_or("");
            match (words.contains(&"M64"), pin) {
                (true, "P0") => marks.push("preheat-on"),
                (false, "P0") => marks.push("preheat-off"),
                (true, "P1") => marks.push("oxygen-on"),
                (false, "P1") => marks.push("oxygen-off"),
                _ => marks.push("unknown-aux"),
            }
        } else if words.contains(&"M3") {
            marks.push("torch-on");
        } else if words.contains(&"M5") {
            marks.push("torch-off");
        } else if words.iter().any(|word| *word == "G4" || *word == "G04") {
            marks.push("dwell");
        } else if words.iter().any(|word| *word == "G0" || *word == "G00") {
            marks.push("rapid");
        } else if words.iter().any(|word| *word == "G1" || *word == "G01" || *word == "G2" || *word == "G02" || *word == "G3" || *word == "G03") {
            marks.push("feed");
        }
    }
    marks
}

fn assert_invariants(marks: &[&str], text: &str) -> Result<(), String> {
    let mut torch = false;
    let mut preheat = false;
    let mut oxygen = false;
    for mark in marks {
        match *mark {
            "torch-on" => {
                if torch {
                    return Err(format!("doubled torch on\n{marks:?}\n{text}"));
                }
                torch = true;
            }
            "torch-off" => {
                if oxygen {
                    return Err(format!("oxygen still on at torch off\n{marks:?}\n{text}"));
                }
                if preheat {
                    return Err(format!("preheat still on at torch off\n{marks:?}\n{text}"));
                }
                if !torch {
                    return Err(format!("torch off while off\n{marks:?}\n{text}"));
                }
                torch = false;
            }
            "preheat-on" => {
                if !torch {
                    return Err(format!("preheat on while torch off\n{marks:?}\n{text}"));
                }
                if preheat {
                    return Err(format!("doubled preheat on\n{marks:?}\n{text}"));
                }
                preheat = true;
            }
            "preheat-off" => {
                if oxygen {
                    return Err(format!("oxygen still on at preheat off\n{marks:?}\n{text}"));
                }
                if !preheat {
                    return Err(format!("preheat off while off\n{marks:?}\n{text}"));
                }
                preheat = false;
            }
            "oxygen-on" => {
                if !torch || !preheat {
                    return Err(format!("oxygen on without torch and preheat\n{marks:?}\n{text}"));
                }
                if oxygen {
                    return Err(format!("doubled oxygen on\n{marks:?}\n{text}"));
                }
                oxygen = true;
            }
            "oxygen-off" => {
                if !oxygen {
                    return Err(format!("oxygen off while off\n{marks:?}\n{text}"));
                }
                oxygen = false;
            }
            "rapid" => {
                if torch || preheat || oxygen {
                    return Err(format!("process on during rapid\n{marks:?}\n{text}"));
                }
            }
            "feed" => {
                if !torch || !preheat {
                    return Err(format!("feed without torch and preheat\n{marks:?}\n{text}"));
                }
                // A plunge to cut height can come after the preheat dwell and before cutting oxygen on.
            }
            "dwell" => {}
            other => return Err(format!("unknown mark {other}")),
        }
    }
    if torch || preheat || oxygen {
        return Err(format!("process left on\n{marks:?}\n{text}"));
    }
    Ok(())
}

fn first_xy_feed(text: &str) -> Result<(f64, f64), String> {
    for line in text.lines() {
        let stripped = without_comments(line);
        let words: Vec<&str> = stripped.split_whitespace().collect();
        let is_feed = words.iter().any(|word| matches!(*word, "G1" | "G01" | "G2" | "G02" | "G3" | "G03"));
        if !is_feed {
            continue;
        }
        let mut x = None;
        let mut y = None;
        for word in &words {
            if let Some(rest) = word.strip_prefix('X') {
                x = rest.parse().ok();
            }
            if let Some(rest) = word.strip_prefix('Y') {
                y = rest.parse().ok();
            }
        }
        if let (Some(x), Some(y)) = (x, y) {
            return Ok((x, y));
        }
    }
    Err(format!("no XY feed\n{text}"))
}

fn preheat_extra() -> &'static str {
    r#", "preheat": true, "min_preheat": 5"#
}

#[test]
fn oxyfuel_event_order_is_preheat_dwell_oxygen_cut() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-order")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), preheat_extra(), 8.0))?;
    let (nc, _) = run_post(&dir, "oxyfuel")?;
    let marks = oxyfuel_marks(&nc);
    assert_invariants(&marks, &nc)?;
    let torch = marks.iter().position(|mark| *mark == "torch-on").ok_or_else(|| format!("no torch on\n{nc}"))?;
    let preheat = marks.iter().position(|mark| *mark == "preheat-on").ok_or_else(|| format!("no preheat on\n{nc}"))?;
    let dwell = marks.iter().position(|mark| *mark == "dwell").ok_or_else(|| format!("no preheat dwell\n{nc}"))?;
    let oxygen = marks.iter().position(|mark| *mark == "oxygen-on").ok_or_else(|| format!("no cutting oxygen on\n{nc}"))?;
    let cut = marks.iter().enumerate().position(|(i, mark)| i > oxygen && *mark == "feed").ok_or_else(|| format!("no cut after oxygen\n{nc}"))?;
    let oxygen_off = marks.iter().rposition(|mark| *mark == "oxygen-off").ok_or_else(|| format!("no oxygen off\n{nc}"))?;
    let preheat_off = marks.iter().rposition(|mark| *mark == "preheat-off").ok_or_else(|| format!("no preheat off\n{nc}"))?;
    let torch_off = marks.iter().rposition(|mark| *mark == "torch-off").ok_or_else(|| format!("no torch off\n{nc}"))?;
    if !(torch < preheat
        && preheat < dwell
        && dwell < oxygen
        && oxygen < cut
        && cut < oxygen_off
        && oxygen_off < preheat_off
        && preheat_off < torch_off)
    {
        return Err(format!("order {marks:?}\n{nc}"));
    }
    if !nc.contains("(min preheat: 5)") {
        return Err(format!("missing min preheat comment\n{nc}"));
    }
    if !nc.lines().any(|line| without_comments(line).split_whitespace().any(|word| word == "G4") && line.contains("P8.000"))
        && !nc.lines().any(|line| without_comments(line).split_whitespace().any(|word| word == "G4") && line.contains("P8"))
    {
        return Err(format!("expected G4 P8 for the job-stated preheat dwell\n{nc}"));
    }
    if !nc.contains("M3 S1") {
        return Err(format!("expected M3 S1\n{nc}"));
    }
    Ok(())
}

#[test]
fn oxyfuel_omits_auxiliaries_when_the_job_does_not_ask() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-plain")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), "", 8.0))?;
    let (nc, _) = run_post(&dir, "oxyfuel")?;
    if nc.contains("M64") || nc.contains("M65") || nc.contains("min preheat") {
        return Err(format!("unexpected preheat or cutting-oxygen words\n{nc}"));
    }
    if !nc.contains("M3 S1") || !nc.contains("M5") {
        return Err(format!("expected torch on and off\n{nc}"));
    }
    Ok(())
}

#[test]
fn oxyfuel_offset_is_half_the_kerf_and_stays_on_the_scrap_side() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-offset")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), preheat_extra(), 8.0))?;
    let (nc, _) = run_post(&dir, "oxyfuel")?;
    let (x, y) = first_xy_feed(&nc)?;
    if (x - 40.0).abs() > 1e-3 || (y + 1.0).abs() > 1e-3 {
        return Err(format!("first feed ({x}, {y}), expected (40, -1)\n{nc}"));
    }
    Ok(())
}

#[test]
fn oxyfuel_keeps_cutting_oxygen_off_on_a_rapid_between_parts() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-two")?;
    let drawing = two_rectangles();
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), preheat_extra(), 8.0))?;
    let (nc, _) = run_post(&dir, "oxyfuel")?;
    let marks = oxyfuel_marks(&nc);
    assert_invariants(&marks, &nc)?;
    let dwells = marks.iter().filter(|mark| **mark == "dwell").count();
    if dwells < 2 {
        return Err(format!("expected a preheat dwell on each part, got {marks:?}\n{nc}"));
    }
    Ok(())
}

#[test]
fn oxyfuel_refuses_a_preheat_shorter_than_min_preheat() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-short")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), preheat_extra(), 2.0))?;
    let out = dir.join("oxyfuel.nc");
    let _ = std::fs::remove_file(&out);
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", "oxyfuel"])?;
    if output.status.success() {
        return Err("short preheat was accepted".into());
    }
    let stderr = text(&output.stderr)?;
    if !stderr.contains("preheat delay is shorter than min_preheat") {
        return Err(format!("unexpected refusal: {stderr}"));
    }
    if out.exists() {
        return Err("short preheat refusal overwrote the output".into());
    }
    Ok(())
}

#[test]
fn plasma_and_waterjet_refuse_the_preheat_auxiliary() -> Result<(), String> {
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    for post in ["plasma", "waterjet"] {
        let dir = temp_dir(&format!("campanion-oxyfuel-{post}"))?;
        write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), preheat_extra(), 8.0))?;
        let out = dir.join(format!("{post}.nc"));
        let _ = std::fs::remove_file(&out);
        let out_arg = out.to_str().ok_or("path")?;
        let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
        if output.status.success() {
            return Err(format!("{post} accepted preheat"));
        }
        let stderr = text(&output.stderr)?;
        if !stderr.contains("does not accept auxiliary `preheat`") {
            return Err(format!("{post} unexpected refusal: {stderr}"));
        }
        if out.exists() {
            return Err(format!("{post} refusal overwrote the output"));
        }
    }
    Ok(())
}

#[test]
fn oxyfuel_preheat_golden_records_the_job_stated_dwell() -> Result<(), String> {
    let dir = temp_dir("campanion-oxyfuel-golden")?;
    write_job(&dir, include_str!("fixtures/oxyfuel_plate.dxf"), include_str!("fixtures/oxyfuel_preheat.job.json"))?;
    let (nc, _) = run_post(&dir, "oxyfuel")?;
    let expected = include_str!("golden/oxyfuel_preheat.oxyfuel.nc");
    if nc != expected {
        return Err(format!("golden mismatch (set UPDATE_GOLDEN=1 after reviewing)\n--- got ---\n{nc}\n--- want ---\n{expected}"));
    }
    if !nc.contains("(min preheat: 5)") || !nc.contains("G4 P8.000") {
        return Err(format!("golden is missing the job-stated minimum or the preheat dwell\n{nc}"));
    }
    Ok(())
}
