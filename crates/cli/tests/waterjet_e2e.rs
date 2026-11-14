//! Straight waterjet cuts: abrasive order, half-kerf offset, and jet off on rapids.
//! Feeds, delays and the kerf in these jobs are example values, not machine data.

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

fn job(hash: &str, extra: &str) -> String {
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
    "feeds": {{ "cut": 1000, "plunge": 300 }},
    "timing": {{ "pierce_delay": 0.25 }},
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

fn waterjet_marks(text: &str) -> Vec<&'static str> {
    let mut marks = Vec::new();
    for line in text.lines() {
        let stripped = without_comments(line);
        let words: Vec<&str> = stripped.split_whitespace().collect();
        if words.contains(&"M64") {
            marks.push("abrasive-on");
        } else if words.contains(&"M65") {
            marks.push("abrasive-off");
        } else if words.contains(&"M3") {
            marks.push("jet-on");
        } else if words.contains(&"M5") {
            marks.push("jet-off");
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
    let mut jet = false;
    let mut abrasive = false;
    for mark in marks {
        match *mark {
            "jet-on" => {
                if jet {
                    return Err(format!("doubled jet on\n{marks:?}\n{text}"));
                }
                jet = true;
            }
            "jet-off" => {
                if abrasive {
                    return Err(format!("abrasive still on at jet off\n{marks:?}\n{text}"));
                }
                if !jet {
                    return Err(format!("jet off while off\n{marks:?}\n{text}"));
                }
                jet = false;
            }
            "abrasive-on" => {
                if !jet {
                    return Err(format!("abrasive on while jet off\n{marks:?}\n{text}"));
                }
                if abrasive {
                    return Err(format!("doubled abrasive on\n{marks:?}\n{text}"));
                }
                abrasive = true;
            }
            "abrasive-off" => {
                if !abrasive {
                    return Err(format!("abrasive off while off\n{marks:?}\n{text}"));
                }
                abrasive = false;
            }
            "rapid" => {
                if jet || abrasive {
                    return Err(format!("process on during rapid\n{marks:?}\n{text}"));
                }
            }
            "feed" => {
                if !jet {
                    return Err(format!("feed while jet off\n{marks:?}\n{text}"));
                }
                // A plunge to cut height comes after the pierce dwell and before abrasive on.
            }
            "dwell" => {}
            other => return Err(format!("unknown mark {other}")),
        }
    }
    if jet || abrasive {
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

#[test]
fn waterjet_event_order_is_jet_pierce_abrasive_cut() -> Result<(), String> {
    let dir = temp_dir("campanion-waterjet-order")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), r#", "abrasive": true, "pierce_type": "stationary""#))?;
    let (nc, _) = run_post(&dir, "waterjet")?;
    let marks = waterjet_marks(&nc);
    assert_invariants(&marks, &nc)?;
    let jet = marks.iter().position(|mark| *mark == "jet-on").ok_or_else(|| format!("no jet on\n{nc}"))?;
    let dwell = marks.iter().position(|mark| *mark == "dwell").ok_or_else(|| format!("no pierce dwell\n{nc}"))?;
    let abrasive = marks.iter().position(|mark| *mark == "abrasive-on").ok_or_else(|| format!("no abrasive on\n{nc}"))?;
    let cut = marks.iter().enumerate().position(|(i, mark)| i > abrasive && *mark == "feed").ok_or_else(|| format!("no cut after abrasive\n{nc}"))?;
    let abrasive_off = marks.iter().rposition(|mark| *mark == "abrasive-off").ok_or_else(|| format!("no abrasive off\n{nc}"))?;
    let jet_off = marks.iter().rposition(|mark| *mark == "jet-off").ok_or_else(|| format!("no jet off\n{nc}"))?;
    if !(jet < dwell && dwell < abrasive && abrasive < cut && cut < abrasive_off && abrasive_off < jet_off) {
        return Err(format!("order {marks:?}\n{nc}"));
    }
    if !nc.contains("(pierce type: stationary)") {
        return Err(format!("missing pierce type comment\n{nc}"));
    }
    if !nc.contains("M3 S1")
        || !nc.lines().any(|line| without_comments(line).split_whitespace().any(|word| word == "P0") && without_comments(line).contains("M64"))
    {
        return Err(format!("expected M3 S1 and M64 P0\n{nc}"));
    }
    Ok(())
}

#[test]
fn waterjet_omits_abrasive_when_the_job_does_not_ask() -> Result<(), String> {
    let dir = temp_dir("campanion-waterjet-no-abr")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), ""))?;
    let (nc, _) = run_post(&dir, "waterjet")?;
    if nc.contains("M64") || nc.contains("M65") {
        return Err(format!("unexpected abrasive words\n{nc}"));
    }
    if !nc.contains("M3 S1") || !nc.contains("M5") {
        return Err(format!("expected jet on and off\n{nc}"));
    }
    Ok(())
}

#[test]
fn waterjet_offset_is_half_the_kerf_and_stays_on_the_scrap_side() -> Result<(), String> {
    let dir = temp_dir("campanion-waterjet-offset")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), r#", "abrasive": true"#))?;
    let (nc, _) = run_post(&dir, "waterjet")?;
    let (x, y) = first_xy_feed(&nc)?;
    // Longest compensated edges are the top and bottom, 1 mm outside the 2 mm kerf.
    // The start is the mid-point of the lower edge: (40, -1).
    if (x - 40.0).abs() > 1e-3 || (y + 1.0).abs() > 1e-3 {
        return Err(format!("first feed ({x}, {y}), expected (40, -1)\n{nc}"));
    }
    Ok(())
}

#[test]
fn waterjet_keeps_jet_and_abrasive_off_on_a_rapid_between_parts() -> Result<(), String> {
    let dir = temp_dir("campanion-waterjet-two")?;
    let drawing = two_rectangles();
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), r#", "abrasive": true"#))?;
    let (nc, _) = run_post(&dir, "waterjet")?;
    let marks = waterjet_marks(&nc);
    assert_invariants(&marks, &nc)?;
    let jets = marks.iter().filter(|mark| **mark == "jet-on").count();
    if jets < 2 {
        return Err(format!("expected a pierce on each part, got {marks:?}\n{nc}"));
    }
    Ok(())
}

#[test]
fn plasma_refuses_the_abrasive_auxiliary() -> Result<(), String> {
    let dir = temp_dir("campanion-waterjet-plasma")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), r#", "abrasive": true"#))?;
    let out = dir.join("plasma.nc");
    let _ = std::fs::remove_file(&out);
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", "plasma"])?;
    if output.status.success() {
        return Err("plasma accepted abrasive".into());
    }
    let stderr = text(&output.stderr)?;
    if !stderr.contains("does not accept auxiliary `abrasive`") {
        return Err(format!("unexpected refusal: {stderr}"));
    }
    if out.exists() {
        return Err("plasma refusal overwrote the output".into());
    }
    Ok(())
}
