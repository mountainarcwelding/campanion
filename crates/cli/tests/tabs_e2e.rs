//! Tabs on a `campanion run` profile (#95).
//!
//! The plate is 100 by 60 with a 30 by 20 notch. Six 10 mm bridges, 3 mm high, are cut with a 6 mm
//! tool to Z -12. The tool-centre gap is the bridge plus the tool diameter. The bridge is crossed
//! at Z -9 along the compensated contour.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const RETRACT: f64 = 2.0;
const RADIUS: f64 = 3.0;
const EPS: f64 = 1.0e-3;

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

fn notch() -> &'static [(f64, f64)] {
    &[(0.0, 0.0), (35.0, 0.0), (35.0, 20.0), (65.0, 20.0), (65.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)]
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
}

fn write_job(dir: &Path, drawing: &str, job: &str) -> Result<(), String> {
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    Ok(())
}

fn profile_job(hash: &str, units: &str, diameter: f64, depth: f64, step: f64, side: &str, tabs: &str, extra: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "{units}",
  "tools": [ {{ "id": 1, "diameter": {diameter}, "speed": 12000 }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "{side}",
    "tool_id": 1,
    "depth": {depth},
    "step_down": {step},
    "tabs": {tabs},
    "feeds": {{ "cut": 1000, "plunge": 300 }}{extra}
  }} ]
}}"#
    )
}

struct Motion {
    rapid: bool,
    x: f64,
    y: f64,
    z: f64,
}

fn motions(ngc: &str) -> Result<Vec<Motion>, String> {
    let mut modal: Option<i32> = None;
    let mut x = 0.0;
    let mut y = 0.0;
    let mut z = 0.0;
    let mut out = Vec::new();
    for (index, raw) in ngc.lines().enumerate() {
        let code = strip_comment(raw);
        if code.is_empty() {
            continue;
        }
        let words = words(&code).map_err(|err| format!("line {}: {err}", index + 1))?;
        let mut group = modal;
        for (letter, value) in &words {
            if *letter == 'G' && *value >= 0.0 && *value <= 3.0 && (value - value.round()).abs() < 1e-6 {
                group = Some(*value as i32);
                modal = group;
            }
        }
        let mut moved = false;
        for (letter, value) in &words {
            match letter {
                'X' => {
                    x = *value;
                    moved = true;
                }
                'Y' => {
                    y = *value;
                    moved = true;
                }
                'Z' => {
                    z = *value;
                    moved = true;
                }
                _ => {}
            }
        }
        if moved && let Some(group) = group {
            out.push(Motion { rapid: group == 0, x, y, z });
        }
    }
    Ok(out)
}

fn strip_comment(line: &str) -> String {
    let mut out = String::new();
    let mut skipping = false;
    for ch in line.chars() {
        if ch == '(' {
            skipping = true;
        } else if ch == ')' {
            skipping = false;
        } else if !skipping {
            out.push(ch);
        }
    }
    out.trim().to_string()
}

fn words(line: &str) -> Result<Vec<(char, f64)>, String> {
    let bytes = line.as_bytes();
    let mut index = 0;
    let mut out = Vec::new();
    while index < bytes.len() {
        let ch = bytes[index];
        if ch.is_ascii_alphabetic() {
            let letter = (ch as char).to_ascii_uppercase();
            index += 1;
            let start = index;
            if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
                index += 1;
            }
            let digits = index;
            while index < bytes.len() && (bytes[index].is_ascii_digit() || bytes[index] == b'.') {
                index += 1;
            }
            if index == digits {
                continue;
            }
            let token = std::str::from_utf8(&bytes[start..index]).map_err(|_| "word is not utf-8".to_string())?;
            let value = token.parse::<f64>().map_err(|_| format!("bad number {token}"))?;
            out.push((letter, value));
        } else {
            index += 1;
        }
    }
    Ok(out)
}

fn near(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= EPS
}

fn xy(a: &Motion, b: &Motion) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

fn assert_no_low_rapid(moves: &[Motion]) -> Result<(), String> {
    let mut prev: Option<&Motion> = None;
    for mv in moves {
        if mv.rapid
            && let Some(from) = prev
            && xy(from, mv) > EPS
            && mv.z < RETRACT - EPS
        {
            return Err(format!("XY rapid below retract at ({}, {}, {})", mv.x, mv.y, mv.z));
        }
        prev = Some(mv);
    }
    Ok(())
}

fn segment_distance(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = bx - ax;
    let dy = by - ay;
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 1e-18 { 0.0 } else { ((px - ax) * dx + (py - ay) * dy) / len2 }.clamp(0.0, 1.0);
    (px - (ax + t * dx)).hypot(py - (ay + t * dy))
}

fn boundary_distance(px: f64, py: f64, poly: &[(f64, f64)]) -> f64 {
    let mut best = f64::MAX;
    for index in 0..poly.len() {
        let a = poly[index];
        let b = poly[(index + 1) % poly.len()];
        best = best.min(segment_distance(px, py, a.0, a.1, b.0, b.1));
    }
    best
}

fn assert_no_gouge(moves: &[Motion], poly: &[(f64, f64)], radius: f64) -> Result<(), String> {
    let mut prev_z = 0.0;
    for mv in moves {
        if !mv.rapid && mv.z < -EPS {
            let dist = boundary_distance(mv.x, mv.y, poly);
            if dist + 0.05 < radius {
                return Err(format!("gouge at ({}, {}) z {} distance {dist} radius {radius} from z {prev_z}", mv.x, mv.y, mv.z));
            }
        }
        prev_z = mv.z;
    }
    Ok(())
}

fn xy_at(moves: &[Motion], z: f64) -> f64 {
    let mut prev: Option<&Motion> = None;
    let mut total = 0.0;
    for mv in moves {
        if let Some(from) = prev
            && !mv.rapid
            && near(from.z, z)
            && near(mv.z, z)
        {
            total += xy(from, mv);
        }
        prev = Some(mv);
    }
    total
}

fn run_posts(dir: &Path, posts: &[&str]) -> Result<Vec<(String, String)>, String> {
    let mut posted = Vec::new();
    for post in posts {
        let out = dir.join(format!("{post}.nc"));
        let out_arg = out.to_str().ok_or("path")?;
        let output = campanion(dir, &["run", "job.json", "-o", out_arg, "--post", post])?;
        let stderr = text(&output.stderr)?;
        let stdout = text(&output.stdout)?;
        if output.status.code() != Some(0) {
            return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
        }
        let ngc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
        posted.push(((*post).to_string(), ngc));
    }
    Ok(posted)
}

fn canonical(step: f64, tabs: &str) -> Result<(PathBuf, String), String> {
    let dir = temp_dir(&format!("campanion-tabs-{step}-{tabs}"))?;
    let drawing = closed_lines("PLATE", notch());
    let hash = fnv1a64(drawing.as_bytes());
    let job = profile_job(&hash, "mm", 6.0, 12.0, step, "outside", tabs, "");
    write_job(&dir, &drawing, &job)?;
    Ok((dir, drawing))
}

#[test]
fn notched_plate_tabs_post_on_three_dialects() -> Result<(), String> {
    let (dir, _) = canonical(12.0, r#"{"count":6,"width":10,"height":3}"#)?;
    let posted = run_posts(&dir, &["mach3", "haas", "linuxcnc"])?;
    for (post, ngc) in &posted {
        let moves = motions(ngc)?;
        assert_no_low_rapid(&moves).map_err(|err| format!("{post}: {err}"))?;
        assert_no_gouge(&moves, notch(), RADIUS).map_err(|err| format!("{post}: {err}"))?;
        let bridge = xy_at(&moves, -9.0);
        // Six 10 mm bridges with a 6 mm tool: each straight gap is 16 mm, and a corner can add more.
        if !(bridge > 90.0 && bridge < 180.0) {
            return Err(format!("{post}: six 10 mm bridges with a 6 mm tool, travel at Z -9 was {bridge}"));
        }
        let floor = xy_at(&moves, -12.0);
        if floor < 250.0 {
            return Err(format!("{post}: floor pass is short ({floor})"));
        }
    }
    Ok(())
}

#[test]
fn passes_above_the_tab_top_are_not_interrupted() -> Result<(), String> {
    let (dir, _) = canonical(4.0, r#"{"count":6,"width":10,"height":3}"#)?;
    let posted = run_posts(&dir, &["linuxcnc"])?;
    let ngc = &posted[0].1;
    let moves = motions(ngc)?;
    assert_no_low_rapid(&moves)?;
    assert_no_gouge(&moves, notch(), RADIUS)?;
    let mut seen_floor = false;
    let mut prev: Option<&Motion> = None;
    for mv in &moves {
        if mv.z < -11.0 {
            seen_floor = true;
        }
        if let Some(from) = prev
            && !seen_floor
            && !mv.rapid
            && near(mv.z, -9.0)
            && near(from.z, -9.0)
            && xy(from, mv) > EPS
        {
            return Err("a pass above the tab top crossed a bridge".into());
        }
        prev = Some(mv);
    }
    let top = xy_at(&moves, -4.0);
    let mid = xy_at(&moves, -8.0);
    let bridge = xy_at(&moves, -9.0);
    if top < 300.0 || mid < 300.0 {
        return Err(format!("shallow passes should loop the offset, got {top} and {mid}"));
    }
    if (top - mid).abs() > 1.0 {
        return Err(format!("passes above the tab differ: {top} vs {mid}"));
    }
    if !(bridge > 90.0 && bridge < 180.0) {
        return Err(format!("deep pass bridge length {bridge}"));
    }
    Ok(())
}

#[test]
fn full_depth_tabs_rapid_at_retract() -> Result<(), String> {
    for (name, tabs) in [("omit", r#"{"count":6,"width":10}"#), ("zero", r#"{"count":6,"width":10,"height":0}"#)] {
        let (dir, _) = canonical(12.0, tabs)?;
        let posted = run_posts(&dir, &["linuxcnc"])?;
        let moves = motions(&posted[0].1)?;
        assert_no_low_rapid(&moves).map_err(|err| format!("{name}: {err}"))?;
        let crosses = moves.windows(3).filter(|window| {
            !window[0].rapid
                && near(window[0].z, RETRACT)
                && window[1].rapid
                && near(window[1].z, RETRACT)
                && xy(&window[0], &window[1]) > 1.0
                && !window[2].rapid
                && near(window[2].x, window[1].x)
                && near(window[2].y, window[1].y)
                && near(window[2].z, -12.0)
        });
        let count = crosses.count();
        if count < 6 {
            return Err(format!("{name}: expected at least 6 full-depth crossings, got {count}"));
        }
    }
    Ok(())
}

#[test]
fn inch_tabs_scale_width_and_bridge_height() -> Result<(), String> {
    let dir = temp_dir("campanion-tabs-inch")?;
    let drawing = closed_lines("PLATE", notch());
    let hash = fnv1a64(drawing.as_bytes());
    // 0.5 in deep, 0.125 in bridges, 0.4 in wide. Tool 0.25 in.
    let job = profile_job(&hash, "inch", 0.25, 0.5, 0.5, "outside", r#"{"count":6,"width":0.4,"height":0.125}"#, "");
    write_job(&dir, &drawing, &job)?;
    let posted = run_posts(&dir, &["linuxcnc"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    let radius = 0.25 * 25.4 * 0.5;
    assert_no_gouge(&moves, notch(), radius)?;
    let bridge_z = -(0.5 - 0.125) * 25.4;
    let bridge = xy_at(&moves, bridge_z);
    let width = 0.4 * 25.4;
    let gap = width + 0.25 * 25.4;
    if !(bridge > 6.0 * gap - 5.0 && bridge < 6.0 * gap + 40.0) {
        return Err(format!("inch bridge length {bridge}, expected at least {}", 6.0 * gap));
    }
    Ok(())
}

#[test]
fn ramp_helix_open_inside_and_finish_keep_the_bridge() -> Result<(), String> {
    let dir = temp_dir("campanion-tabs-ramp")?;
    let drawing = closed_lines("PLATE", notch());
    let hash = fnv1a64(drawing.as_bytes());
    let ramp =
        profile_job(&hash, "mm", 6.0, 12.0, 12.0, "outside", r#"{"count":4,"width":8,"height":3}"#, r#","entry":{"type":"ramp","angle_deg":8}"#);
    write_job(&dir, &drawing, &ramp)?;
    let posted = run_posts(&dir, &["mach3"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    assert_no_gouge(&moves, notch(), RADIUS)?;
    if xy_at(&moves, -9.0) < 30.0 {
        return Err("ramp did not feed the bridges at Z -9".into());
    }

    let helix = profile_job(
        &hash,
        "mm",
        6.0,
        6.0,
        6.0,
        "inside",
        r#"{"count":4,"width":6,"height":2}"#,
        r#","entry":{"type":"helix","radius":2,"angle_deg":10}"#,
    );
    write_job(&dir, &drawing, &helix)?;
    let posted = run_posts(&dir, &["haas"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    // Inside, the helix sits in the pocket. The contour crossing is still at z + height = -4.
    if xy_at(&moves, -4.0) < 20.0 {
        return Err("inside helix did not feed the bridges".into());
    }

    let open_dir = temp_dir("campanion-tabs-open")?;
    let open_drawing = dxf(&line("PLATE", 0.0, 0.0, 80.0, 0.0));
    let open_hash = fnv1a64(open_drawing.as_bytes());
    let open = profile_job(&open_hash, "mm", 6.0, 4.0, 2.0, "outside", r#"{"count":2,"width":8,"height":1}"#, "");
    write_job(&open_dir, &open_drawing, &open)?;
    let posted = run_posts(&open_dir, &["linuxcnc"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    if xy_at(&moves, -2.0) < 70.0 {
        return Err(format!("open shallow pass is short: {}", xy_at(&moves, -2.0)));
    }
    if xy_at(&moves, -3.0) < 14.0 {
        return Err(format!("open bridges are short: {}", xy_at(&moves, -3.0)));
    }

    let finish_dir = temp_dir("campanion-tabs-finish")?;
    let finish = profile_job(
        &hash,
        "mm",
        6.0,
        12.0,
        12.0,
        "on",
        r#"{"count":4,"width":8,"height":3}"#,
        r#","stock_to_leave":{"wall":0.2,"floor":4},"finish":{"step_down":12}"#,
    );
    // `on` ignores wall stock. Use outside so the rough link stands off and the finish lands on the radius.
    let finish = finish.replacen(r#""side": "on""#, r#""side": "outside""#, 1);
    write_job(&finish_dir, &drawing, &finish)?;
    let posted = run_posts(&finish_dir, &["linuxcnc"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    assert_no_gouge(&moves, notch(), RADIUS)?;
    let mut seen_floor = false;
    let mut prev: Option<&Motion> = None;
    for mv in &moves {
        if near(mv.z, -12.0) {
            seen_floor = true;
        }
        if let Some(from) = prev
            && !seen_floor
            && !mv.rapid
            && near(from.z, -9.0)
            && near(mv.z, -9.0)
            && xy(from, mv) > EPS
        {
            return Err("roughing pass above the floor crossed a tab".into());
        }
        prev = Some(mv);
    }
    if xy_at(&moves, -9.0) < 30.0 {
        return Err("finish pass did not cross the tabs".into());
    }
    let mut prev: Option<&Motion> = None;
    for mv in &moves {
        if let Some(from) = prev
            && !mv.rapid
            && near(from.z, -9.0)
            && near(mv.z, -9.0)
            && xy(from, mv) > EPS
        {
            let dist = boundary_distance(mv.x, mv.y, notch());
            if (dist - RADIUS).abs() > 0.3 {
                return Err(format!("finish bridge is not on the radius contour: distance {dist}"));
            }
        }
        prev = Some(mv);
    }
    Ok(())
}

#[test]
fn tilted_plane_tabs_follow_the_tool_axis() -> Result<(), String> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let dir = temp_dir("campanion-tabs-tilted")?;
    std::fs::copy(root.join("examples/table_table_bc.kin.json"), dir.join("table_table_bc.kin.json")).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/sample.dxf"), dir.join("sample.dxf")).map_err(|err| err.to_string())?;
    let job = r#"{
      "format": "campanion-job/1",
      "source": { "path": "sample.dxf", "hash": "cee9f36f5ba230a2" },
      "units": "mm",
      "kinematics": "table_table_bc.kin.json",
      "tools": [ { "id": 1, "diameter": 6, "speed": 12000 } ],
      "operations": [ {
        "name": "tilt",
        "type": "profile",
        "layers": ["OUTER"],
        "side": "outside",
        "tool_id": 1,
        "depth": 3,
        "step_down": 3,
        "tabs": { "count": 4, "width": 8, "height": 1 },
        "plane": { "origin": [0, 0, 0], "normal": [0, -0.5, 0.8660254037844386] },
        "feeds": { "cut": 1000, "plunge": 300 }
      } ]
    }"#;
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join("out.ngc");
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(&dir, &["run", "job.json", "-o", out_arg])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("tilted tabs exit {:?}: {stderr}", output.status.code()));
    }
    let ngc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    if !ngc.contains("G1") && !ngc.contains("G01") {
        return Err("tilted tabs produced no feed".into());
    }
    Ok(())
}

#[test]
fn on_side_tabs_follow_the_part_boundary() -> Result<(), String> {
    let dir = temp_dir("campanion-tabs-on")?;
    let drawing = closed_lines("PLATE", notch());
    let hash = fnv1a64(drawing.as_bytes());
    let job = profile_job(&hash, "mm", 6.0, 12.0, 12.0, "on", r#"{"count":4,"width":8,"height":3}"#, "");
    write_job(&dir, &drawing, &job)?;
    let posted = run_posts(&dir, &["linuxcnc"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    let bridge = xy_at(&moves, -9.0);
    // Four 8 mm bridges on the line with a 6 mm tool: each tool-centre gap is 14 mm.
    if !(bridge > 50.0 && bridge < 62.0) {
        return Err(format!("on-contour bridge length {bridge}"));
    }
    let mut prev: Option<&Motion> = None;
    for mv in &moves {
        if let Some(from) = prev
            && !mv.rapid
            && near(from.z, -9.0)
            && near(mv.z, -9.0)
            && xy(from, mv) > EPS
        {
            let dist = boundary_distance(mv.x, mv.y, notch());
            if dist > 0.05 {
                return Err(format!("on-contour bridge left the boundary by {dist} at ({}, {})", mv.x, mv.y));
            }
        }
        prev = Some(mv);
    }
    Ok(())
}

#[test]
fn tabs_that_do_not_fit_the_contour_fail_clearly() -> Result<(), String> {
    let dir = temp_dir("campanion-tabs-fit")?;
    let drawing = dxf(&line("PLATE", 0.0, 0.0, 40.0, 0.0));
    let hash = fnv1a64(drawing.as_bytes());
    for (name, tabs, needle) in [
        ("wide", r#"{"count":8,"width":10}"#, "tabs are wider than the contour"),
        ("spacing", r#"{"spacing":80,"width":4}"#, "tab spacing is longer than the contour"),
    ] {
        let job = profile_job(&hash, "mm", 6.0, 2.0, 2.0, "on", tabs, "");
        write_job(&dir, &drawing, &job)?;
        let out = dir.join("out.ngc");
        let output = campanion(&dir, &["run", "job.json", "-o", out.to_str().unwrap_or("out.ngc")])?;
        let stderr = text(&output.stderr)?;
        if output.status.code() == Some(0) {
            return Err(format!("{name} was accepted"));
        }
        if !stderr.contains(needle) {
            return Err(format!("{name} missing `{needle}` in {stderr}"));
        }
    }
    Ok(())
}

#[test]
fn example_tabs_job_matches_linuxcnc_golden() -> Result<(), String> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())?;
    let out = temp_dir("campanion-tabs-example")?.join("tabs.nc");
    let out_arg = out.to_str().ok_or("path")?;
    let output = campanion(&root, &["run", "examples/tabs.job.json", "-o", out_arg, "--post", "linuxcnc"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("example exit {:?}: {stderr}", output.status.code()));
    }
    let actual = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    let golden = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden/tabs.linuxcnc.nc");
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        std::fs::write(&golden, &actual).map_err(|err| err.to_string())?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&golden).map_err(|err| format!("read golden: {err}"))?;
    if actual != expected {
        return Err(format!("golden mismatch for {} (set UPDATE_GOLDEN=1 to regenerate)", golden.display()));
    }
    Ok(())
}

/// A posted ramp with tabs starts at the retract height. Descents that travel in XY stay within
/// `angle_deg`. A pure Z plunge is the tab return, not a ramp through the bridge.
#[test]
fn ramp_with_tabs_posts_descents_within_the_angle() -> Result<(), String> {
    let dir = temp_dir("campanion-tabs-ramp-angle")?;
    let shape = [(0.0, 0.0), (40.0, 0.0), (40.0, 10.0), (0.0, 10.0)];
    let drawing = closed_lines("PLATE", &shape);
    let hash = fnv1a64(drawing.as_bytes());
    let job = profile_job(&hash, "mm", 6.0, 8.0, 8.0, "on", r#"{"count":4,"width":6,"height":2}"#, r#","entry":{"type":"ramp","angle_deg":5}"#);
    write_job(&dir, &drawing, &job)?;
    let posted = run_posts(&dir, &["linuxcnc"])?;
    let moves = motions(&posted[0].1)?;
    assert_no_low_rapid(&moves)?;
    let max_angle = 5.0_f64.to_radians();
    let mut prev: Option<&Motion> = None;
    let mut saw = false;
    for mv in &moves {
        if let Some(from) = prev
            && !mv.rapid
            && mv.z < from.z - 1e-6
        {
            let horiz = xy(from, mv);
            if horiz > 1e-3 {
                let angle = (from.z - mv.z).atan2(horiz);
                if angle > max_angle + 2.0e-4 {
                    return Err(format!("posted {:.3}° from Z {:.4} to Z {:.4} exceeds 5°", angle.to_degrees(), from.z, mv.z));
                }
                if !saw {
                    if (from.z - RETRACT).abs() > 1e-3 {
                        return Err(format!("first ramp descent starts at Z {:.4}, retract is {RETRACT}", from.z));
                    }
                    saw = true;
                }
            }
        }
        prev = Some(mv);
    }
    if !saw {
        return Err("ramp did not descend".into());
    }
    // Bridge top is depth - height = 6 mm down. The crossing is a feed, not a cut through it.
    if xy_at(&moves, -6.0) < 10.0 {
        return Err(format!("ramp did not feed the bridges at Z -6, travel {}", xy_at(&moves, -6.0)));
    }
    Ok(())
}
