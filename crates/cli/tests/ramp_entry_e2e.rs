//! Ramp and helix entries stay at or below `angle_deg` in the posted toolpath.
//!
//! The link rapids to the retract height. The entry is sized from that height, so a ramp
//! that used to drop from Z 2 to Z -2 over `2 / tan(5°)` mm (9.92°) now covers the whole 4 mm.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use serde_json::Value;

const REPRO_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="170mm" height="80mm" viewBox="0 0 170 80">"#,
    r#"<g id="RECT"><rect x="10" y="10" width="100" height="60"/></g>"#,
    r#"<g id="HOLE"><circle cx="140" cy="40" r="20"/></g>"#,
    "</svg>",
);

const ANGLE_EPS: f64 = 1.0e-6;
/// Test-sink coordinates are printed to 0.0001 mm. This is still far below the old 9.92° ramp.
const NGC_ANGLE_EPS: f64 = 2.0e-4;

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

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

#[derive(Clone, Copy)]
enum Shape {
    Rect,
    Circle,
    Polygon,
}

struct Job {
    svg: String,
    retract: f64,
    depth: f64,
    step: f64,
    side: &'static str,
    entry: String,
}

fn run_job(dir: &Path, name: &str, job: &Job) -> Result<(String, Value), String> {
    let hash = fnv1a64(job.svg.as_bytes());
    std::fs::write(dir.join("part.svg"), &job.svg).map_err(|err| err.to_string())?;
    let clearance = (job.retract + 3.0).max(job.retract);
    let body = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","heights":{{"clearance":{clearance},"retract":{retract}}},"tools":[{{"id":1,"diameter":6.0}}],"operations":[{{"name":"cut","type":"profile","layers":["PART"],"side":"{side}","tool_id":1,"depth":{depth},"step_down":{step},"entry":{entry},"feeds":{{"cut":1500.0,"plunge":500.0}}}}]}}"#,
        retract = job.retract,
        depth = job.depth,
        step = job.step,
        side = job.side,
        entry = job.entry,
    );
    std::fs::write(dir.join("job.json"), body).map_err(|err| err.to_string())?;
    let ngc = dir.join(format!("{name}.ngc"));
    let ir = dir.join(format!("{name}.json"));
    let ngc_arg = path_arg(&ngc)?;
    let ir_arg = path_arg(&ir)?;
    let output = campanion(dir, &["run", "job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{name}: run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("{name}: source hash did not match: {stderr}"));
    }
    let ngc_text = std::fs::read_to_string(&ngc).map_err(|err| err.to_string())?;
    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let ir_json: Value = serde_json::from_str(&ir_text).map_err(|err| format!("{name}: ir: {err}"))?;
    Ok((ngc_text, ir_json))
}

fn levels_of(depth: f64, step: f64) -> Vec<f64> {
    let mut levels = Vec::new();
    let mut done = 0.0;
    loop {
        let next = done + step;
        if next >= depth - 1e-9 {
            levels.push(-depth);
            break;
        }
        done = next;
        levels.push(-done);
    }
    levels
}

#[derive(Clone, Copy)]
struct Xyz {
    x: f64,
    y: f64,
    z: f64,
}

fn num(value: &Value, key: &str) -> Result<f64, String> {
    value.get(key).and_then(Value::as_f64).ok_or_else(|| format!("missing {key}"))
}

fn xyz_of(value: &Value) -> Result<Xyz, String> {
    Ok(Xyz { x: num(value, "x")?, y: num(value, "y")?, z: num(value, "z")? })
}

fn tip_of(pose: &Value) -> Result<Xyz, String> {
    xyz_of(pose.get("tip").ok_or("missing tip")?)
}

fn xy_sweep(from: Xyz, to: Xyz, centre: Xyz, ccw: bool) -> f64 {
    let a0 = (from.y - centre.y).atan2(from.x - centre.x);
    let a1 = (to.y - centre.y).atan2(to.x - centre.x);
    let radius = (from.x - centre.x).hypot(from.y - centre.y);
    let chord = (to.x - from.x).hypot(to.y - from.y);
    let closure = 1e-6_f64.max(radius * 1e-6);
    let ccw_delta = (a1 - a0).rem_euclid(std::f64::consts::TAU);
    if chord <= closure {
        std::f64::consts::TAU
    } else if ccw {
        if ccw_delta <= 1e-8 { std::f64::consts::TAU } else { ccw_delta }
    } else {
        let cw = std::f64::consts::TAU - ccw_delta;
        if cw <= 1e-8 { std::f64::consts::TAU } else { cw }
    }
}

struct Descent {
    start_z: f64,
    end_z: f64,
    angle: f64,
    lead_in: bool,
}

fn ir_descents(program: &Value) -> Result<Vec<Vec<Descent>>, String> {
    let paths = program.get("program").and_then(|p| p.get("toolpaths")).and_then(Value::as_array).ok_or("ir has no toolpaths")?;
    let mut out = Vec::new();
    for path in paths {
        let blocks = path.get("blocks").and_then(Value::as_array).ok_or("toolpath has no blocks")?;
        let mut cursor: Option<Xyz> = None;
        let mut descents = Vec::new();
        for block in blocks {
            let kind = block.get("type").and_then(Value::as_str).unwrap_or("");
            let next = match kind {
                "rapid" | "linear" | "arc" | "helix" => Some(tip_of(block.get("to").ok_or("motion has no end")?)?),
                _ => None,
            };
            if let (Some(from), Some(to)) = (cursor, next)
                && kind != "rapid"
                && to.z < from.z - 1e-9
            {
                let horiz = match kind {
                    "linear" => (to.x - from.x).hypot(to.y - from.y),
                    "helix" => {
                        let centre = xyz_of(block.get("center").ok_or("helix has no centre")?)?;
                        let sweep = num(block, "sweep")?;
                        let radius = (from.x - centre.x).hypot(from.y - centre.y);
                        radius * sweep
                    }
                    "arc" => {
                        let centre = xyz_of(block.get("center").ok_or("arc has no centre")?)?;
                        let ccw = block.get("sense").and_then(Value::as_str) == Some("ccw");
                        let radius = (from.x - centre.x).hypot(from.y - centre.y);
                        radius * xy_sweep(from, to, centre, ccw)
                    }
                    _ => 0.0,
                };
                let angle = (from.z - to.z).atan2(horiz);
                let lead_in = block.get("role").and_then(Value::as_str) == Some("lead_in");
                descents.push(Descent { start_z: from.z, end_z: to.z, angle, lead_in });
            }
            if let Some(to) = next {
                cursor = Some(to);
            }
        }
        out.push(descents);
    }
    Ok(out)
}

fn endpoint_zs(program: &Value) -> Result<Vec<f64>, String> {
    let paths = program.get("program").and_then(|p| p.get("toolpaths")).and_then(Value::as_array).ok_or("ir has no toolpaths")?;
    let mut zs = Vec::new();
    for path in paths {
        let blocks = path.get("blocks").and_then(Value::as_array).ok_or("toolpath has no blocks")?;
        for block in blocks {
            if let Some(to) = block.get("to") {
                zs.push(tip_of(to)?.z);
            }
        }
    }
    Ok(zs)
}

fn assert_ir(name: &str, program: &Value, angle_deg: f64, depth: f64, step: f64, retract: f64) -> Result<(), String> {
    let max_angle = angle_deg.to_radians();
    let final_z = -depth;
    let levels = levels_of(depth, step);
    let paths = ir_descents(program)?;
    if paths.is_empty() {
        return Err(format!("{name}: no toolpath"));
    }
    for (index, descents) in paths.iter().enumerate() {
        if descents.is_empty() {
            return Err(format!("{name}: toolpath {index} has no descending feed"));
        }
        let first = &descents[0];
        if (first.start_z - retract).abs() > 1e-6 {
            return Err(format!("{name}: toolpath {index} first descent starts at Z {}, expected retract {retract}", first.start_z));
        }
        for descent in descents {
            if descent.angle > max_angle + ANGLE_EPS {
                return Err(format!(
                    "{name}: toolpath {index} descent {:.6} rad ({:.3}°) from Z {:.4} to Z {:.4} exceeds {angle_deg}°",
                    descent.angle,
                    descent.angle.to_degrees(),
                    descent.start_z,
                    descent.end_z,
                ));
            }
            if descent.end_z < final_z - 1e-6 {
                return Err(format!("{name}: descent ends at Z {} below the floor {final_z}", descent.end_z));
            }
        }
    }
    let zs = endpoint_zs(program)?;
    for level in levels {
        if !zs.iter().any(|z| (*z - level).abs() < 1e-5) {
            return Err(format!("{name}: missing level {level}, zs {zs:?}"));
        }
    }
    Ok(())
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
    out
}

fn word_f64(words: &[(char, f64)], letter: char) -> Option<f64> {
    words.iter().find(|(name, _)| *name == letter).map(|(_, value)| *value)
}

fn ngc_descents(ngc: &str) -> Result<Vec<Descent>, String> {
    let mut cursor: Option<Xyz> = None;
    let mut descents = Vec::new();
    for raw in ngc.lines() {
        let code = strip_comments(raw);
        if code.is_empty() {
            continue;
        }
        let mut motion = None;
        let mut words = Vec::new();
        for word in code.split_whitespace() {
            let mut chars = word.chars();
            let Some(letter) = chars.next() else {
                continue;
            };
            let rest = chars.as_str();
            let value: f64 = rest.parse().map_err(|_| format!("bad word {word}"))?;
            if letter == 'G' && (0.0..=3.0).contains(&value) {
                motion = Some(value as u8);
            }
            words.push((letter, value));
        }
        let Some(motion) = motion else {
            continue;
        };
        let Some(from) = cursor else {
            cursor = Some(Xyz {
                x: word_f64(&words, 'X').unwrap_or(0.0),
                y: word_f64(&words, 'Y').unwrap_or(0.0),
                z: word_f64(&words, 'Z').unwrap_or(0.0),
            });
            continue;
        };
        let to = Xyz {
            x: word_f64(&words, 'X').unwrap_or(from.x),
            y: word_f64(&words, 'Y').unwrap_or(from.y),
            z: word_f64(&words, 'Z').unwrap_or(from.z),
        };
        if motion != 0 && to.z < from.z - 1e-9 {
            let horiz = if motion == 1 {
                (to.x - from.x).hypot(to.y - from.y)
            } else {
                let i = word_f64(&words, 'I').unwrap_or(0.0);
                let j = word_f64(&words, 'J').unwrap_or(0.0);
                let centre = Xyz { x: from.x + i, y: from.y + j, z: from.z };
                let radius = i.hypot(j);
                radius * xy_sweep(from, to, centre, motion == 3)
            };
            descents.push(Descent { start_z: from.z, end_z: to.z, angle: (from.z - to.z).atan2(horiz), lead_in: false });
        }
        cursor = Some(to);
    }
    Ok(descents)
}

fn assert_ngc(name: &str, ngc: &str, angle_deg: f64) -> Result<(), String> {
    let max_angle = angle_deg.to_radians();
    let descents = ngc_descents(ngc)?;
    if descents.is_empty() {
        return Err(format!("{name}: ngc has no descending feed\n{ngc}"));
    }
    for descent in &descents {
        if descent.angle > max_angle + NGC_ANGLE_EPS {
            return Err(format!(
                "{name}: posted {:.3}° from Z {:.4} to Z {:.4} exceeds {angle_deg}°\n{ngc}",
                descent.angle.to_degrees(),
                descent.start_z,
                descent.end_z,
            ));
        }
    }
    Ok(())
}

fn rect_svg() -> String {
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="120mm" height="80mm" viewBox="0 0 120 80"><g id="PART"><rect x="10" y="10" width="80" height="50"/></g></svg>"#.to_string()
}

fn circle_svg() -> String {
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="100mm" viewBox="0 0 100 100"><g id="PART"><circle cx="50" cy="50" r="25"/></g></svg>"#.to_string()
}

fn polygon_svg() -> String {
    let n = 20;
    let radius = 28.0;
    let (cx, cy) = (50.0, 50.0);
    let mut points = Vec::new();
    for i in 0..n {
        let angle = std::f64::consts::TAU * (i as f64) / (n as f64);
        points.push(format!("{:.4},{:.4}", cx + radius * angle.cos(), cy + radius * angle.sin()));
    }
    format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="120mm" height="120mm" viewBox="0 0 120 120"><g id="PART"><polygon points="{}"/></g></svg>"#,
        points.join(" ")
    )
}

fn svg_of(shape: Shape) -> (String, &'static str) {
    match shape {
        Shape::Rect => (rect_svg(), "outside"),
        Shape::Circle => (circle_svg(), "inside"),
        Shape::Polygon => (polygon_svg(), "outside"),
    }
}

fn unit(state: &mut u64) -> f64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
    ((*state >> 33) as f64) / f64::from(u32::MAX)
}

#[test]
fn posted_ramp_on_the_repro_stays_within_five_degrees() -> Result<(), String> {
    let dir = temp_dir("campanion-ramp-repro")?;
    let hash = fnv1a64(REPRO_SVG.as_bytes());
    std::fs::write(dir.join("part.svg"), REPRO_SVG).map_err(|err| err.to_string())?;
    let job = format!(
        r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":[{{"id":1,"diameter":6.0}}],"operations":[
        {{"name":"rect","type":"profile","layers":["RECT"],"side":"outside","tool_id":1,"depth":6,"step_down":2,"entry":{{"type":"ramp","angle_deg":5}},"feeds":{{"cut":1500.0,"plunge":500.0}}}},
        {{"name":"hole","type":"profile","layers":["HOLE"],"side":"inside","tool_id":1,"depth":6,"step_down":2,"entry":{{"type":"ramp","angle_deg":5}},"feeds":{{"cut":1500.0,"plunge":500.0}}}}
        ]}}"#
    );
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let ngc_path = dir.join("out.ngc");
    let ir_path = dir.join("out.json");
    let ngc_arg = path_arg(&ngc_path)?;
    let ir_arg = path_arg(&ir_path)?;
    let output = campanion(&dir, &["run", "job.json", "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    let ngc = std::fs::read_to_string(&ngc_path).map_err(|err| err.to_string())?;
    let ir: Value = serde_json::from_str(&std::fs::read_to_string(&ir_path).map_err(|err| err.to_string())?).map_err(|err| err.to_string())?;
    assert_ir("repro", &ir, 5.0, 6.0, 2.0, 2.0)?;
    assert_ngc("repro", &ngc, 5.0)?;
    let paths = ir_descents(&ir)?;
    if paths.len() != 2 {
        return Err(format!("repro wrote {} toolpaths, expected the rect and the hole", paths.len()));
    }
    for (index, descents) in paths.iter().enumerate() {
        let band: Vec<_> = descents.iter().filter(|d| d.end_z < -1e-9 && d.start_z > -6.0 + 1e-9).collect();
        if band.is_empty() {
            return Err(format!("repro toolpath {index} has no feed below Z 0 and above the floor"));
        }
        let worst = band.iter().map(|d| d.angle).fold(0.0, f64::max);
        if worst > 5.0_f64.to_radians() + ANGLE_EPS {
            return Err(format!("repro toolpath {index} band angle {:.3}°", worst.to_degrees()));
        }
    }
    let root = workspace_root()?;
    let output = campanion(&root, &["verify", &ir_arg])?;
    if output.status.code() != Some(0) {
        return Err(format!("verify exit {:?} {}", output.status.code(), text(&output.stderr)?));
    }
    Ok(())
}

#[test]
fn posted_ramp_stays_within_angle_for_random_heights_and_shapes() -> Result<(), String> {
    let dir = temp_dir("campanion-ramp-random")?;
    let forced = [
        (0.5, 6.0, 2.0, 5.0, Shape::Rect),
        (20.0, 3.0, 1.0, 1.0, Shape::Polygon),
        (2.0, 6.0, 2.0, 30.0, Shape::Circle),
        (8.0, 5.0, 2.0, 5.0, Shape::Polygon),
        (1.0, 4.0, 0.5, 12.0, Shape::Rect),
        (15.0, 6.0, 3.0, 3.0, Shape::Circle),
    ];
    let mut state = 0x91u64;
    let mut cases = forced.to_vec();
    for _ in 0..12 {
        let retract = 0.5 + unit(&mut state) * 19.5;
        let depth = 1.0 + unit(&mut state) * 6.0;
        let step = 0.3 + unit(&mut state) * (depth - 0.3).max(0.3);
        let angle = 1.0 + unit(&mut state) * 29.0;
        let shape = match (state % 3) as u8 {
            0 => Shape::Rect,
            1 => Shape::Circle,
            _ => Shape::Polygon,
        };
        cases.push((retract, depth, step, angle, shape));
    }
    for (index, (retract, depth, step, angle, shape)) in cases.iter().copied().enumerate() {
        let (svg, side) = svg_of(shape);
        let job = Job { svg, retract, depth, step, side, entry: format!(r#"{{"type":"ramp","angle_deg":{angle}}}"#) };
        let name = format!("case-{index}");
        let (_ngc, ir) = run_job(&dir, &name, &job).map_err(|err| format!("{err} (retract {retract}, depth {depth}, step {step}, angle {angle})"))?;
        assert_ir(&name, &ir, angle, depth, step, retract)
            .map_err(|err| format!("{err} (retract {retract}, depth {depth}, step {step}, angle {angle})"))?;
        if matches!(shape, Shape::Polygon) && angle <= 5.0 && retract >= 8.0 {
            let paths = ir_descents(&ir)?;
            let lead_ins = paths.iter().flat_map(|d| d.iter()).filter(|d| d.lead_in).count();
            if lead_ins < 3 {
                return Err(format!("{name}: short polygon ramp did not wrap corners, {lead_ins} lead-in descents"));
            }
        }
    }
    Ok(())
}

#[test]
fn posted_helix_from_retract_stays_within_the_angle() -> Result<(), String> {
    let dir = temp_dir("campanion-helix-retract")?;
    let cases = [(2.0, 6.0, 2.0, 5.0, 4.0), (12.0, 4.0, 1.5, 3.0, 3.0), (0.5, 5.0, 2.5, 15.0, 5.0)];
    for (index, (retract, depth, step, angle, radius)) in cases.iter().copied().enumerate() {
        let job = Job {
            svg: circle_svg(),
            retract,
            depth,
            step,
            side: "inside",
            entry: format!(r#"{{"type":"helix","radius":{radius},"angle_deg":{angle}}}"#),
        };
        let name = format!("helix-{index}");
        let (_ngc, ir) = run_job(&dir, &name, &job)?;
        assert_ir(&name, &ir, angle, depth, step, retract)?;
    }
    Ok(())
}
