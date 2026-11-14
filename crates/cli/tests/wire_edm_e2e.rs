//! 2-axis wire EDM: offset is wire radius plus the spark gap named by each pass,
//! a lead from a supplied start hole, open tabs, skim passes, and wire off on rapids.
//! Wire diameter, spark gaps, feeds and power in these jobs are example values, not machine data.

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
    for (layer, points) in [
        ("LEFT", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)][..]),
        ("RIGHT", &[(120.0, 0.0), (200.0, 0.0), (200.0, 40.0), (120.0, 40.0)][..]),
    ] {
        for pair in points.windows(2) {
            entities.push_str(&line(layer, pair[0].0, pair[0].1, pair[1].0, pair[1].1));
        }
        if let (Some(first), Some(last)) = (points.first(), points.last()) {
            entities.push_str(&line(layer, last.0, last.1, first.0, first.1));
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

fn job(hash: &str, extra: &str, hole: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, "wire": 0.25 }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "outside",
    "tool_id": 1,
    "depth": 10.0,
    "step_down": 10.0,
    "feeds": {{ "cut": 200, "plunge": 200 }},
    "power": 0.6,
    "spark_gap": 0.10,
    "start_hole": {hole}{extra}
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

fn parse_xy(line: &str) -> Option<(f64, f64)> {
    let stripped = without_comments(line);
    let mut x = None;
    let mut y = None;
    for word in stripped.split_whitespace() {
        if let Some(rest) = word.strip_prefix('X') {
            x = rest.parse().ok();
        } else if let Some(rest) = word.strip_prefix('Y') {
            y = rest.parse().ok();
        }
    }
    Some((x?, y?))
}

fn motion_kind(line: &str) -> Option<&'static str> {
    let stripped = without_comments(line);
    let words: Vec<&str> = stripped.split_whitespace().collect();
    if words.iter().any(|word| *word == "G0" || *word == "G00") {
        Some("rapid")
    } else if words.iter().any(|word| *word == "G1" || *word == "G01" || *word == "G2" || *word == "G02" || *word == "G3" || *word == "G03") {
        Some("feed")
    } else {
        None
    }
}

fn first_rapid_xy(text: &str) -> Result<(f64, f64), String> {
    for line in text.lines() {
        if motion_kind(line) == Some("rapid")
            && let Some(xy) = parse_xy(line)
        {
            return Ok(xy);
        }
    }
    Err(format!("no rapid\n{text}"))
}

fn rect_distance(x: f64, y: f64, w: f64, h: f64) -> f64 {
    let dx = if x < 0.0 {
        -x
    } else if x > w {
        x - w
    } else {
        0.0
    };
    let dy = if y < 0.0 {
        -y
    } else if y > h {
        y - h
    } else {
        0.0
    };
    if dx > 0.0 && dy > 0.0 {
        (dx * dx + dy * dy).sqrt()
    } else if dx > 0.0 {
        dx
    } else if dy > 0.0 {
        dy
    } else {
        (x.min(w - x)).min(y.min(h - y))
    }
}

fn feed_offsets(text: &str, w: f64, h: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut x = 0.0;
    let mut y = 0.0;
    let mut have = false;
    for line in text.lines() {
        if let Some((nx, ny)) = parse_xy(line) {
            x = nx;
            y = ny;
            have = true;
        }
        if have && motion_kind(line) == Some("feed") {
            out.push(rect_distance(x, y, w, h));
        }
    }
    out
}

fn wire_marks(text: &str) -> Vec<&'static str> {
    let mut marks = Vec::new();
    for line in text.lines() {
        let stripped = without_comments(line);
        let words: Vec<&str> = stripped.split_whitespace().collect();
        if words.contains(&"M8") {
            marks.push("flush-on");
        } else if words.contains(&"M9") {
            marks.push("flush-off");
        } else if words.contains(&"M0") {
            marks.push("stop");
        } else if words.contains(&"M3") {
            marks.push("wire-on");
        } else if words.contains(&"M5") {
            marks.push("wire-off");
        } else if words.iter().any(|word| *word == "G0" || *word == "G00") {
            marks.push("rapid");
        } else if words.iter().any(|word| *word == "G1" || *word == "G01" || *word == "G2" || *word == "G02" || *word == "G3" || *word == "G03") {
            marks.push("feed");
        }
    }
    marks
}

fn assert_wire_off_on_rapids(marks: &[&str], text: &str) -> Result<(), String> {
    let mut wire = false;
    for mark in marks {
        match *mark {
            "wire-on" => {
                if wire {
                    return Err(format!("doubled wire on\n{marks:?}\n{text}"));
                }
                wire = true;
            }
            "wire-off" => {
                if !wire {
                    return Err(format!("wire off while off\n{marks:?}\n{text}"));
                }
                wire = false;
            }
            "rapid" if wire => return Err(format!("wire on during a rapid\n{marks:?}\n{text}")),
            "stop" if wire => return Err(format!("program stop while wire on\n{marks:?}\n{text}")),
            _ => {}
        }
    }
    Ok(())
}

fn z_leaves_zero(text: &str) -> bool {
    let mut z = 0.0;
    let mut seen_motion = false;
    for line in text.lines() {
        let kind = motion_kind(line);
        if kind.is_none() {
            continue;
        }
        let stripped = without_comments(line);
        for word in stripped.split_whitespace() {
            if let Some(rest) = word.strip_prefix('Z')
                && let Ok(value) = rest.parse::<f64>()
            {
                z = value;
            }
        }
        seen_motion = true;
        if z.abs() > 1e-6 {
            return true;
        }
    }
    let _ = seen_motion;
    false
}

#[test]
fn offset_is_radius_plus_the_named_gap_and_the_lead_starts_at_the_hole() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-offset")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), "", r#"{ "x": -5, "y": 20 }"#))?;
    let (nc, _) = run_post(&dir, "wire_edm")?;
    if z_leaves_zero(&nc) {
        return Err(format!("wire path left Z=0\n{nc}"));
    }
    let (x, y) = first_rapid_xy(&nc)?;
    if (x + 5.0).abs() > 1e-4 || (y - 20.0).abs() > 1e-4 {
        return Err(format!("lead did not start at the start hole: {x}, {y}\n{nc}"));
    }
    let offsets = feed_offsets(&nc, 80.0, 40.0);
    if offsets.is_empty() {
        return Err(format!("no feed offsets\n{nc}"));
    }
    let expected = 0.25 / 2.0 + 0.10;
    if offsets.iter().any(|d| (d - expected).abs() > 1e-3) {
        return Err(format!("offset {offsets:?} != {expected}\n{nc}"));
    }
    if !nc.contains("M3 S0.6000") {
        return Err(format!("missing job power\n{nc}"));
    }
    Ok(())
}

#[test]
fn skims_use_a_smaller_named_gap_and_optional_feed() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-skim")?;
    write_job(&dir, include_str!("fixtures/wire_edm_plate.dxf"), include_str!("fixtures/wire_edm_skims.job.json"))?;
    let (nc, _) = run_post(&dir, "wire_edm")?;
    let marks = wire_marks(&nc);
    assert_wire_off_on_rapids(&marks, &nc)?;
    if !nc.contains("M3 S0.6000") || !nc.contains("M3 S0.4000") {
        return Err(format!("missing pass power\n{nc}"));
    }
    if !nc.contains("F200.0000") || !nc.contains("F120.0000") {
        return Err(format!("missing skim feed\n{nc}"));
    }
    if !marks.contains(&"flush-on") || !marks.contains(&"flush-off") {
        return Err(format!("missing flush\n{nc}"));
    }
    let expected_rough = 0.25 / 2.0 + 0.10;
    let expected_skim = 0.25 / 2.0 + 0.05;
    let mut saw_rough = false;
    let mut saw_skim = false;
    for d in feed_offsets(&nc, 80.0, 40.0) {
        if (d - expected_rough).abs() <= 1e-3 {
            saw_rough = true;
        } else if (d - expected_skim).abs() <= 1e-3 {
            saw_skim = true;
        } else {
            return Err(format!("unexpected offset {d}\n{nc}"));
        }
    }
    if !saw_rough || !saw_skim {
        return Err(format!("missing rough or skim offset (rough {saw_rough} skim {saw_skim})\n{nc}"));
    }
    Ok(())
}

#[test]
fn open_tab_does_not_rapid_across_the_tab() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-tab")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    let extra = r#", "tabs": { "count": 1, "width": 4 }"#;
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), extra, r#"{ "x": -5, "y": 20 }"#))?;
    let (nc, _) = run_post(&dir, "wire_edm")?;
    let marks = wire_marks(&nc);
    assert_wire_off_on_rapids(&marks, &nc)?;
    if !marks.contains(&"stop") {
        return Err(format!("missing program stop beside the tab\n{nc}"));
    }
    let mut wire = false;
    let mut x = 0.0;
    let mut y = 0.0;
    let mut have = false;
    for line in nc.lines() {
        let stripped = without_comments(line);
        let words: Vec<&str> = stripped.split_whitespace().collect();
        if words.contains(&"M3") {
            wire = true;
        }
        if words.contains(&"M5") {
            wire = false;
        }
        if let Some((nx, ny)) = parse_xy(line) {
            if motion_kind(line) == Some("rapid") && wire {
                return Err(format!("XY rapid with wire on from {x},{y} to {nx},{ny}\n{nc}"));
            }
            if motion_kind(line) == Some("rapid") && have && !wire {
                let dist = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
                if dist > 0.5 && dist < 6.0 {
                    return Err(format!("rapid of {dist} mm looks like a tab crossing from {x},{y} to {nx},{ny}\n{nc}"));
                }
            }
            x = nx;
            y = ny;
            have = true;
        }
    }
    Ok(())
}

fn two_part_job(hash: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, "wire": 0.25 }} ],
  "operations": [
    {{
      "name": "left",
      "type": "profile",
      "layers": ["LEFT"],
      "side": "outside",
      "tool_id": 1,
      "depth": 10.0,
      "step_down": 10.0,
      "feeds": {{ "cut": 200, "plunge": 200 }},
      "power": 0.6,
      "spark_gap": 0.10,
      "start_hole": {{ "x": -5, "y": 20 }}
    }},
    {{
      "name": "right",
      "type": "profile",
      "layers": ["RIGHT"],
      "side": "outside",
      "tool_id": 1,
      "depth": 10.0,
      "step_down": 10.0,
      "feeds": {{ "cut": 200, "plunge": 200 }},
      "power": 0.6,
      "spark_gap": 0.10,
      "start_hole": {{ "x": 115, "y": 20 }}
    }}
  ]
}}"#
    )
}

#[test]
fn wire_is_off_on_rapids_between_parts() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-parts")?;
    let drawing = two_rectangles();
    write_job(&dir, &drawing, &two_part_job(&fnv1a64(drawing.as_bytes())))?;
    let (nc, _) = run_post(&dir, "wire_edm")?;
    let marks = wire_marks(&nc);
    assert_wire_off_on_rapids(&marks, &nc)?;
    if !marks.windows(3).any(|window| window[0] == "wire-off" && window[1] == "rapid") {
        return Err(format!("no wire-off before a rapid between parts\n{marks:?}\n{nc}"));
    }
    Ok(())
}

#[test]
fn unnamed_or_non_decreasing_skim_gap_is_refused() -> Result<(), String> {
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    let cases = [
        (r#", "skims": [{ "power": 0.4 }]"#, "skims[0] must name spark_gap"),
        (r#", "skims": [{ "spark_gap": 0.10 }]"#, "skim spark_gap must be smaller than the previous pass"),
        (r#", "skims": [{ "spark_gap": 0.20 }]"#, "skim spark_gap must be smaller than the previous pass"),
    ];
    for (index, (extra, needle)) in cases.iter().enumerate() {
        let dir = temp_dir(&format!("campanion-wire-refuse-{index}"))?;
        write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), extra, r#"{ "x": -5, "y": 20 }"#))?;
        let out = dir.join("wire_edm.nc");
        let _ = std::fs::remove_file(&out);
        let out_arg = out.to_str().ok_or("path")?;
        let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", "wire_edm"])?;
        if output.status.success() {
            return Err(format!("case {index} was accepted"));
        }
        let stderr = text(&output.stderr)?;
        if !stderr.contains(needle) {
            return Err(format!("case {index} unexpected refusal: {stderr}"));
        }
        if out.exists() {
            return Err(format!("case {index} overwrote the output"));
        }
    }
    Ok(())
}

#[test]
fn plasma_oxyfuel_and_waterjet_refuse_the_wire_auxiliary() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-posts")?;
    let drawing = closed_lines("PLATE", &[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]);
    write_job(&dir, &drawing, &job(&fnv1a64(drawing.as_bytes()), "", r#"{ "x": -5, "y": 20 }"#))?;
    for post in ["plasma", "oxyfuel", "waterjet"] {
        let out = dir.join(format!("{post}.nc"));
        let _ = std::fs::remove_file(&out);
        let out_arg = out.to_str().ok_or("path")?;
        let output = campanion(&dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
        if output.status.success() {
            return Err(format!("{post} accepted wire_edm"));
        }
        let stderr = text(&output.stderr)?;
        if !stderr.contains("does not accept auxiliary `wire_edm`") {
            return Err(format!("{post} unexpected refusal: {stderr}"));
        }
        if out.exists() {
            return Err(format!("{post} refusal overwrote the output"));
        }
    }
    Ok(())
}

#[test]
fn wire_edm_skims_golden_records_the_job_stated_gaps() -> Result<(), String> {
    let dir = temp_dir("campanion-wire-golden")?;
    write_job(&dir, include_str!("fixtures/wire_edm_plate.dxf"), include_str!("fixtures/wire_edm_skims.job.json"))?;
    let (nc, _) = run_post(&dir, "wire_edm")?;
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden/wire_edm_skims.wire_edm.nc");
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        std::fs::write(&path, &nc).map_err(|err| err.to_string())?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if nc != expected {
        return Err(format!("golden mismatch (set UPDATE_GOLDEN=1 after reviewing)\n--- got ---\n{nc}\n--- want ---\n{expected}"));
    }
    if !nc.contains("M3 S0.6000") || !nc.contains("M3 S0.4000") {
        return Err(format!("golden is missing the job-stated power\n{nc}"));
    }
    Ok(())
}
