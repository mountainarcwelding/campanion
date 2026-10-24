//! `campanion lathe` on the stepped-shaft example, and hostile job files.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output().map_err(|err| err.to_string())
}

fn run_ok(args: &[&str]) -> Result<Output, String> {
    let output = campanion(args)?;
    if output.status.code() != Some(0) {
        let stdout = text(&output.stdout).unwrap_or_default();
        let stderr = text(&output.stderr).unwrap_or_default();
        return Err(format!("exit {:?} args {args:?}\nstdout {stdout}\nstderr {stderr}", output.status.code()));
    }
    Ok(output)
}

#[test]
fn shaft_posts_linuxcnc_lathe_and_keeps_three_bindings() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/shaft.lathe.json"))?;
    let out = temp_path("campanion-lathe-shaft.nc");
    let ir = temp_path("campanion-lathe-shaft.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "linuxcnc_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    for word in ["G7", "G18", "G96"] {
        if !posted.contains(word) {
            return Err(format!("posted program is missing {word}\n{posted}"));
        }
    }
    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let doc = campanion_ir::Document::from_json(&ir_text).map_err(|err| err.to_string())?;
    let turning = doc.program().turning().ok_or("turning setup is missing")?;
    if turning.bindings().len() != 7 {
        return Err(format!("expected 7 bindings, found {}", turning.bindings().len()));
    }
    let section = turning.section().ok_or("turning section is missing")?;
    let points = section.points();
    if points.len() < 2 {
        return Err(format!("turning section has {} points", points.len()));
    }
    let first_z = points.first().map(|point| point[0]).ok_or("turning section is empty")?;
    let last_z = points.last().map(|point| point[0]).ok_or("turning section is empty")?;
    if first_z < last_z {
        return Err(format!("section z runs from {first_z} to {last_z}"));
    }
    if doc.program().toolpaths().len() != 7 {
        return Err(format!("expected 7 toolpaths, found {}", doc.program().toolpaths().len()));
    }
    Ok(())
}

#[test]
fn shaft_posts_every_lathe_dialect_and_fanuc_threads_at_pitch() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/shaft.lathe.json"))?;
    for id in ["linuxcnc_lathe", "grbl_lathe", "fanuc_lathe", "haas_lathe"] {
        let out = temp_path(&format!("campanion-lathe-shaft-{id}.nc"));
        let _ = std::fs::remove_file(&out);
        run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", id])?;
        let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
        if posted.trim().is_empty() {
            return Err(format!("{id} wrote an empty program"));
        }
        if id == "fanuc_lathe" {
            let threaded = posted.lines().any(|line| line.contains("G32") && line.split_whitespace().any(|word| word == "F2.5"));
            if !threaded {
                return Err(format!("fanuc thread line is missing G32 F2.5\n{posted}"));
            }
            for token in posted.split_whitespace() {
                if token.starts_with('X')
                    && token.chars().nth(1).is_some_and(|ch| ch.is_ascii_digit() || ch == '-' || ch == '.')
                    && !token.contains('.')
                {
                    return Err(format!("fanuc X word has no decimal point: {token}"));
                }
            }
        }
    }
    Ok(())
}

fn assert_rejects(body: &str, file_name: &str, needle: &str) -> Result<(), String> {
    let job = temp_path(file_name);
    std::fs::write(&job, body).map_err(|err| err.to_string())?;
    let out = temp_path(&format!("{file_name}.nc"));
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["lathe", &path_arg(&job)?, "-o", &path_arg(&out)?])?;
    let stderr = text(&output.stderr)?;
    let code = output.status.code();
    if code == Some(101) || stderr.to_ascii_lowercase().contains("panicked") {
        return Err(format!("lathe panicked ({code:?}): {stderr}"));
    }
    if code != Some(1) {
        return Err(format!("exit {code:?}, expected 1\n{stderr}"));
    }
    if !stderr.contains(needle) {
        return Err(format!("stderr missing {needle}: {stderr}"));
    }
    if out.exists() {
        return Err(format!("failed job wrote {}", out.display()));
    }
    Ok(())
}

#[test]
fn hostile_jobs_error_without_panic() -> Result<(), String> {
    let unknown = r#"{
        "format": "campanion-lathe/1",
        "surprise": 1,
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [],
        "operations": [],
        "clearance": 1
    }"#;
    assert_rejects(unknown, "campanion-lathe-unknown.json", "surprise")?;

    let nan = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [],
        "operations": [],
        "clearance": NaN
    }"#;
    assert_rejects(nan, "campanion-lathe-nan.json", "json")?;

    let missing = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }],
        "operations": [{ "name": "face", "type": "face", "tool_id": 99, "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }],
        "clearance": 1
    }"#;
    assert_rejects(missing, "campanion-lathe-missing-tool.json", "missing tool 99")?;

    let tool = r#"{ "id": 1, "station": 1, "shape": "thread", "nose_radius": 0.2, "orientation_deg": 60, "hand": "right", "tip": "centre" }"#;
    let negative = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 20, "length": 30, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 3000 }},
        "tools": [{tool}],
        "operations": [{{ "name": "thread", "type": "thread", "tool_id": 1, "side": "outside", "major_diameter": 20, "pitch": -2.5, "z_start": 0, "z_end": -10, "rpm": 400 }}],
        "clearance": 1
    }}"#
    );
    assert_rejects(&negative, "campanion-lathe-negative-pitch.json", "pitch must be positive")?;

    let css = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 20, "length": 30, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 3000 }},
        "tools": [{tool}],
        "operations": [{{ "name": "thread", "type": "thread", "tool_id": 1, "side": "outside", "major_diameter": 20, "pitch": 2.5, "z_start": 0, "z_end": -10, "rpm": 400, "speed": {{ "css": 100 }} }}],
        "clearance": 1
    }}"#
    );
    assert_rejects(&css, "campanion-lathe-thread-css.json", "unknown operation field `speed`")?;

    let no_rpm = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 20, "length": 30, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 3000 }},
        "tools": [{tool}],
        "operations": [{{ "name": "thread", "type": "thread", "tool_id": 1, "side": "outside", "major_diameter": 20, "pitch": 2.5, "z_start": 0, "z_end": -10 }}],
        "clearance": 1
    }}"#
    );
    assert_rejects(&no_rpm, "campanion-lathe-thread-rpm.json", "missing `rpm`")?;

    let infeed = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 20, "length": 30, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 3000 }},
        "tools": [{tool}],
        "operations": [{{ "name": "thread", "type": "thread", "tool_id": 1, "side": "outside", "major_diameter": 20, "pitch": 2.5, "z_start": 0, "z_end": -10, "rpm": 400, "infeed": "alternating" }}],
        "clearance": 1
    }}"#
    );
    assert_rejects(&infeed, "campanion-lathe-thread-infeed.json", "unknown infeed")?;
    Ok(())
}

#[test]
fn wrong_orientation_reports_a_gouge_and_writes_nothing() -> Result<(), String> {
    let dir = temp_path("campanion-lathe-gouge");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    let dxf = "\
0
SECTION
2
HEADER
9
$INSUNITS
70
4
0
ENDSEC
0
SECTION
2
ENTITIES
0
LINE
8
PROFILE
10
0.0
20
0.0
11
0.0
21
19.0
0
LINE
8
PROFILE
10
0.0
20
19.0
11
-1.0
21
20.0
0
LINE
8
PROFILE
10
-1.0
20
20.0
11
-40.0
21
20.0
0
ENDSEC
0
EOF
";
    std::fs::write(dir.join("cylinder.dxf"), dxf).map_err(|err| err.to_string())?;
    let job = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "cylinder.dxf", "layer": "PROFILE" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 50, "length": 40, "front_allowance": 1.0 },
        "spindle": { "max_rpm": 3000 },
        "tools": [
            { "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.8, "orientation_deg": 315, "hand": "right", "tip": "centre" }
        ],
        "operations": [
            { "name": "finish", "type": "finish", "tool_id": 1, "side": "outside", "feed": 120, "speed": { "rpm": 1000 } }
        ],
        "tolerance": { "linear": 0.001, "chord": 0.01 },
        "clearance": 2.0
    }"#;
    let job_path = dir.join("job.json");
    std::fs::write(&job_path, job).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["lathe", &path_arg(&job_path)?, "-o", &path_arg(&out)?])?;
    let stderr = text(&output.stderr)?;
    let code = output.status.code();
    if code == Some(101) || stderr.to_ascii_lowercase().contains("panicked") {
        return Err(format!("lathe panicked ({code:?}): {stderr}"));
    }
    if code != Some(1) {
        return Err(format!("exit {code:?}, expected 1\n{stderr}"));
    }
    if !stderr.lines().any(|line| line.starts_with("gouge operation finish ") && line.split_whitespace().any(|word| word == "edge")) {
        return Err(format!("stderr missing an edge gouge line: {stderr}"));
    }
    if out.exists() {
        return Err(format!("gouged job wrote {}", out.display()));
    }
    Ok(())
}

fn assert_golden(name: &str, actual: &str) -> Result<(), String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|err| err.to_string())?;
        }
        std::fs::write(&path, actual).map_err(|err| err.to_string())?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if actual != expected {
        return Err(format!("golden mismatch for {} (set UPDATE_GOLDEN=1 to regenerate)", path.display()));
    }
    Ok(())
}

fn assert_in_order(text: &str, needles: &[&str]) -> Result<(), String> {
    let mut from = 0;
    for needle in needles {
        let Some(at) = text[from..].find(needle) else {
            return Err(format!("missing `{needle}` after byte {from}\n{text}"));
        };
        from += at + needle.len();
    }
    Ok(())
}

/// `M133` and `M135` contain the characters M3. Only a whitespace token that is exactly `M3` is a work-spindle start.
fn assert_no_work_spindle_between_c_engage(text: &str) -> Result<(), String> {
    let mut rest = text;
    let mut spans = 0;
    while let Some(start) = rest.find("M154") {
        let after = &rest[start + "M154".len()..];
        let Some(end) = after.find("M155") else {
            return Err(format!("M154 has no following M155\n{text}"));
        };
        for token in after[..end].split_whitespace() {
            if token == "M3" {
                return Err(format!("M3 between M154 and M155\n{}", &after[..end]));
            }
        }
        spans += 1;
        rest = &after[end + "M155".len()..];
    }
    if spans < 2 {
        return Err(format!("expected two C-axis cycles, found {spans}"));
    }
    Ok(())
}

fn live_shell(spindle: &str, tools: &str, operation: &str) -> String {
    format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 40, "length": 30, "front_allowance": 0 }},
        "spindle": {spindle},
        "tools": [{tools}],
        "operations": [{operation}],
        "clearance": 2
    }}"#
    )
}

#[test]
fn millturn_posts_haas_live_drill_and_slot() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/millturn.lathe.json"))?;
    let out = temp_path("campanion-lathe-millturn.nc");
    let ir = temp_path("campanion-lathe-millturn.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "haas_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("millturn.haas_lathe.nc", &posted)?;
    assert_in_order(&posted, &["M154", "G28 H0.", "M133", "M14", "M15", "C90.", "M135", "M155"])?;
    assert_no_work_spindle_between_c_engage(&posted)?;

    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let doc = campanion_ir::Document::from_json(&ir_text).map_err(|err| err.to_string())?;
    let program = doc.program();
    let turning = program.turning().ok_or("turning setup is missing")?;
    if turning.spindles().first().and_then(|spindle| spindle.c_axis()) != Some(0) {
        return Err(format!("spindle C axis is {:?}", turning.spindles().first().map(|spindle| spindle.c_axis())));
    }
    if turning.heads().first().is_none_or(|head| !head.live_tooling()) {
        return Err("head live tooling is off".into());
    }
    if turning.bindings().len() != 3 || program.toolpaths().len() != 3 {
        return Err(format!("expected 3 toolpaths, bindings {} paths {}", turning.bindings().len(), program.toolpaths().len()));
    }
    if turning.bindings()[0].speed().is_none() {
        return Err("face binding has no spindle speed".into());
    }
    if turning.bindings()[1].speed().is_some() || turning.bindings()[2].speed().is_some() {
        return Err("a live binding stored a spindle speed".into());
    }
    if turning.insert_for_tool(1).is_none() || turning.insert_for_tool(2).is_some() || turning.insert_for_tool(3).is_some() {
        return Err("live tools were mounted as inserts, or the facing insert is missing".into());
    }
    let diameter = |id: u32| program.tools().iter().find(|tool| tool.id() == id).and_then(|tool| tool.diameter());
    if diameter(1).is_some() || diameter(2) != Some(6.0) || diameter(3) != Some(8.0) {
        return Err(format!("tool diameters {:?} {:?} {:?}", diameter(1), diameter(2), diameter(3)));
    }
    assert_live_path(&program.toolpaths()[1], campanion_ir::Vec3::Z, &[0.0, 90.0, 180.0, 270.0])?;
    assert_live_path(&program.toolpaths()[2], campanion_ir::Vec3::X, &[0.0])?;
    Ok(())
}

fn assert_live_path(path: &campanion_ir::ClPath, axis: campanion_ir::Vec3, angles: &[f64]) -> Result<(), String> {
    let mut seen = Vec::new();
    for block in path.blocks() {
        match block {
            campanion_ir::ClBlock::Event(campanion_ir::Event::ProcessOn { .. }) => {
                return Err(format!("live path {} turns the work spindle on", path.name().unwrap_or("")));
            }
            campanion_ir::ClBlock::Motion(motion) => {
                let pose = match motion {
                    campanion_ir::ClMotion::Rapid { to, .. } | campanion_ir::ClMotion::Linear { to, .. } => to,
                    _ => return Err("live path has a non-linear move".into()),
                };
                if pose.axis() != axis || pose.tip().y() != 0.0 {
                    return Err(format!("pose axis {:?} y {}", pose.axis(), pose.tip().y()));
                }
                let Some(c) = pose.rotary() else {
                    return Err("live motion has no C angle".into());
                };
                seen.push(c);
            }
            campanion_ir::ClBlock::Event(_) => {}
        }
    }
    for angle in angles {
        if !seen.iter().any(|c| (c - angle).abs() < 1e-9) {
            return Err(format!("C {angle} is missing from {seen:?}"));
        }
    }
    Ok(())
}

#[test]
fn live_jobs_reject_a_missing_c_axis_a_wrong_tool_and_bad_geometry() -> Result<(), String> {
    let axial = r#"{ "id": 2, "station": 2, "live": "axial", "kind": "drill", "diameter": 6, "max_rpm": 5000 }"#;
    let radial = r#"{ "id": 3, "station": 3, "live": "radial", "kind": "endmill", "diameter": 8 }"#;
    let front = r#"{ "name": "holes", "type": "live_drill", "tool_id": 2, "face": "front", "z_start": 0, "positions": [{ "c": 0, "r": 10 }], "depth": 5, "rpm": 1000, "feed": 80 }"#;
    let no_c = live_shell(r#"{ "max_rpm": 3000 }"#, axial, front);
    assert_rejects(&no_c, "campanion-lathe-live-no-c.json", "live operations need spindle.c_axis true")?;

    let od = r#"{ "name": "cross", "type": "live_drill", "tool_id": 2, "face": "od", "diameter": 30, "positions": [{ "c": 0, "z": -8 }], "depth": 4, "rpm": 1000, "feed": 80 }"#;
    let axial_on_od = live_shell(r#"{ "max_rpm": 3000, "c_axis": true }"#, axial, od);
    assert_rejects(&axial_on_od, "campanion-lathe-live-axial-od.json", "operation cross: tool 2 is not a radial live tool")?;

    let empty = r#"{ "name": "holes", "type": "live_drill", "tool_id": 2, "face": "front", "z_start": 0, "positions": [], "depth": 5, "rpm": 1000, "feed": 80 }"#;
    let empty_positions = live_shell(r#"{ "max_rpm": 3000, "c_axis": true }"#, axial, empty);
    assert_rejects(&empty_positions, "campanion-lathe-live-empty.json", "positions is empty")?;

    let deep = r#"{ "name": "cross", "type": "live_drill", "tool_id": 3, "face": "od", "diameter": 20, "positions": [{ "c": 0, "z": -4 }], "depth": 11, "rpm": 1000, "feed": 80 }"#;
    let too_deep = live_shell(r#"{ "max_rpm": 3000, "c_axis": true }"#, radial, deep);
    assert_rejects(&too_deep, "campanion-lathe-live-depth.json", "depth exceeds the surface radius")?;
    Ok(())
}
