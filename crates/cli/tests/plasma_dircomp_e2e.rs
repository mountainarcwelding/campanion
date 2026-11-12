//! Run-job profile direction, auto side, and controller compensation.

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

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn write_files(dir: &Path, drawing: &str, job: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn run_post(dir: &Path, post: &str) -> Result<String, String> {
    let out = dir.join(format!("{post}.nc"));
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    std::fs::read_to_string(&out).map_err(|err| err.to_string())
}

fn plate() -> &'static [(f64, f64)] {
    &[(0.0, 0.0), (80.0, 0.0), (80.0, 50.0), (0.0, 50.0)]
}

fn hole() -> &'static [(f64, f64)] {
    &[(20.0, 15.0), (40.0, 15.0), (40.0, 35.0), (20.0, 35.0)]
}

fn job(hash: &str, tool: &str, side: &str, extra: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, {tool} }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "{side}",
    "tool_id": 1,
    "depth": 6.0,
    "step_down": 6.0,
    "feeds": {{ "cut": 1000, "plunge": 300 }}{extra}
  }} ]
}}"#
    )
}

fn first_cut_xy(ngc: &str) -> Result<(f64, f64, f64, f64), String> {
    // First G1 or G2/G3 at cut Z after the plunge, returning (from implied, we just take first two moved XY at Z-6)
    let mut x: f64 = 0.0;
    let mut y: f64 = 0.0;
    let mut z: f64 = 0.0;
    let mut first: Option<(f64, f64)> = None;
    for raw in ngc.lines() {
        let mut motion = None;
        for word in raw.split_whitespace() {
            match word {
                "G0" | "G00" => motion = Some(0),
                "G1" | "G01" => motion = Some(1),
                "G2" | "G02" | "G3" | "G03" => motion = Some(2),
                _ => {}
            }
            if let Some(v) = word.strip_prefix('X') {
                x = v.parse().map_err(|e| format!("{e}"))?;
            }
            if let Some(v) = word.strip_prefix('Y') {
                y = v.parse().map_err(|e| format!("{e}"))?;
            }
            if let Some(v) = word.strip_prefix('Z') {
                z = v.parse().map_err(|e| format!("{e}"))?;
            }
        }
        if matches!(motion, Some(1 | 2)) && (z + 6.0).abs() < 1e-3 {
            match first {
                None => first = Some((x, y)),
                Some((x0, y0)) if (x - x0).abs() > 1e-4 || (y - y0).abs() > 1e-4 => return Ok((x0, y0, x, y)),
                Some(_) => {}
            }
        }
    }
    Err(format!("no cut move\n{ngc}"))
}

fn has_word(ngc: &str, word: &str) -> bool {
    ngc.split_whitespace().any(|w| w == word)
}

#[test]
fn conventional_travels_opposite_climb_on_an_outside_profile() -> Result<(), String> {
    let drawing = closed_lines("PLATE", plate());
    let hash = fnv1a64(drawing.as_bytes());
    let climb_dir = temp_dir("campanion-dir-climb")?;
    write_files(&climb_dir, &drawing, &job(&hash, r#""diameter": 6.0, "speed": 12000"#, "outside", r#", "direction": "climb""#))?;
    let climb = first_cut_xy(&run_post(&climb_dir, "linuxcnc")?)?;
    let conv_dir = temp_dir("campanion-dir-conv")?;
    write_files(&conv_dir, &drawing, &job(&hash, r#""diameter": 6.0, "speed": 12000"#, "outside", r#", "direction": "conventional""#))?;
    let conv = first_cut_xy(&run_post(&conv_dir, "linuxcnc")?)?;
    if climb == conv {
        return Err(format!("climb and conventional started the same way {climb:?}"));
    }
    // Opposite travel along the same first edge, or a mirrored start.
    let climb_dx = climb.2 - climb.0;
    let conv_dx = conv.2 - conv.0;
    let climb_dy = climb.3 - climb.1;
    let conv_dy = conv.3 - conv.1;
    let opposite = climb_dx * conv_dx + climb_dy * conv_dy < 0.0;
    if !opposite {
        return Err(format!("expected opposite first-cut directions, climb {climb:?} conventional {conv:?}"));
    }
    Ok(())
}

#[test]
fn controller_left_posts_g41_on_the_part_contour() -> Result<(), String> {
    let dir = temp_dir("campanion-comp-g41")?;
    let drawing = closed_lines("PLATE", plate());
    let job = job(
        &fnv1a64(drawing.as_bytes()),
        r#""kerf": 1.5"#,
        "outside",
        r#", "compensation": "controller_left", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#,
    );
    write_files(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "plasma")?;
    if !has_word(&ngc, "G41") || !has_word(&ngc, "G40") {
        return Err(format!("expected G41/G40\n{ngc}"));
    }
    // The tool stays on the part: some cut motion lands on Y 0 or Y 50 or X 0 or X 80, not the 0.75 offset.
    let on_part = ngc.lines().any(|line| {
        let words: Vec<_> = line.split_whitespace().collect();
        if !words.iter().any(|w| *w == "G1" || *w == "G2" || *w == "G3") {
            return false;
        }
        let y = words.iter().find_map(|w| w.strip_prefix('Y').and_then(|v| v.parse::<f64>().ok()));
        let x = words.iter().find_map(|w| w.strip_prefix('X').and_then(|v| v.parse::<f64>().ok()));
        matches!(y, Some(v) if (v - 0.0).abs() < 1e-3 || (v - 50.0).abs() < 1e-3)
            || matches!(x, Some(v) if (v - 0.0).abs() < 1e-3 || (v - 80.0).abs() < 1e-3)
    });
    if !on_part {
        return Err(format!("controller path should follow the part edge\n{ngc}"));
    }
    Ok(())
}

#[test]
fn auto_side_offsets_an_outer_outward_and_a_hole_inward() -> Result<(), String> {
    let dir = temp_dir("campanion-auto-side")?;
    let drawing = {
        let mut entities = String::new();
        for poly in [plate(), hole()] {
            for pair in poly.windows(2) {
                entities.push_str(&line("PLATE", pair[0].0, pair[0].1, pair[1].0, pair[1].1));
            }
            if let (Some(first), Some(last)) = (poly.first(), poly.last()) {
                entities.push_str(&line("PLATE", last.0, last.1, first.0, first.1));
            }
        }
        dxf(&entities)
    };
    let job = job(&fnv1a64(drawing.as_bytes()), r#""kerf": 2.0"#, "auto", r#", "lead_in": {"type": "none"}, "lead_out": {"type": "none"}"#);
    write_files(&dir, &drawing, &job)?;
    let ngc = run_post(&dir, "plasma")?;
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for raw in ngc.lines() {
        for word in raw.split_whitespace() {
            if let Some(v) = word.strip_prefix('X')
                && let Ok(n) = v.parse::<f64>()
            {
                xs.push(n);
            }
            if let Some(v) = word.strip_prefix('Y')
                && let Ok(n) = v.parse::<f64>()
            {
                ys.push(n);
            }
        }
    }
    let max_x = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min_inner_x = xs.iter().copied().filter(|x| *x > 15.0 && *x < 45.0).fold(f64::INFINITY, f64::min);
    // half kerf is 1 mm: outer should reach about 81, hole should pull in from 20 toward 21.
    if max_x < 80.5 {
        return Err(format!("outer was not offset outward, max X {max_x}\n{ngc}"));
    }
    if min_inner_x < 20.5 {
        return Err(format!("hole was not offset inward, min inner X {min_inner_x}\n{ngc}"));
    }
    Ok(())
}
