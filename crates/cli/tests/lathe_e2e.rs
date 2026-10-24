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

#[test]
fn channel_must_be_zero_or_one() -> Result<(), String> {
    let body = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }],
        "operations": [
            { "name": "face", "type": "face", "tool_id": 1, "step_z": 0.5, "feed": 100, "speed": { "rpm": 1000 }, "channel": 2 }
        ],
        "clearance": 1
    }"#;
    assert_rejects(body, "channel-two.json", "operation face: channel must be 0 or 1")?;
    let fractional = body.replace("\"channel\": 2", "\"channel\": 1.5");
    assert_rejects(&fractional, "channel-fraction.json", "operation face: channel must be 0 or 1")?;
    let wait = r#"{
        "format": "campanion-lathe/1",
        "source": { "path": "missing.dxf" },
        "units": "mm",
        "stock": { "kind": "bar", "diameter": 20, "length": 10, "front_allowance": 1 },
        "spindle": { "max_rpm": 1000 },
        "tools": [],
        "operations": [ { "name": "sync", "type": "wait", "channel": 0 } ],
        "clearance": 1
    }"#;
    assert_rejects(wait, "wait-channel.json", "unknown operation field `channel`")
}

#[test]
fn two_channel_job_posts_and_prints_balance() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/two_channel.lathe.json"))?;
    let out = temp_path("two_channel.generic_lathe_2ch.nc");
    let _ = std::fs::remove_file(&out);
    let output = run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "generic_lathe_2ch"])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("two_channel.generic_lathe_2ch.nc", &posted)?;
    let stderr = text(&output.stderr)?;
    if !stderr.contains("segment 0 channel 0 time_s") || !stderr.contains("channel 1 time_s") || !stderr.contains("cycle_s") {
        return Err(format!("balance report missing:\n{stderr}"));
    }
    assert_in_order(
        &posted,
        &[
            "%",
            "O0001",
            "G21",
            "G18",
            "(CHANNEL 1)",
            "(toolpath face)",
            "(toolpath rough)",
            "M101",
            "(toolpath finish)",
            "(CHANNEL 2)",
            "(toolpath drill)",
            "M101",
            "(toolpath groove)",
            "G28",
            "M30",
            "%",
        ],
    )?;
    if posted.matches("O0001").count() != 1 || posted.matches("M30").count() != 1 || posted.matches("(CHANNEL 1)").count() != 1 {
        return Err(format!("channel layout is not one header and one section each:\n{posted}"));
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

fn flat_shell(head: &str, operation: &str) -> String {
    let head_field = if head.is_empty() { String::new() } else { format!("\"head\": {head},") };
    format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 40, "length": 30, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 3000, "c_axis": true }},
        {head_field}
        "tools": [{{ "id": 1, "station": 1, "live": "radial", "kind": "endmill", "diameter": 8 }}],
        "operations": [{operation}],
        "clearance": 2
    }}"#
    )
}

const FLAT_OP: &str = r#"{ "name": "od_flat", "type": "live_flat", "tool_id": 1, "c": 90, "diameter": 40, "across": 18, "z_from": -10, "z_to": -10, "rpm": 1800, "feed": 120 }"#;

#[test]
fn millturn_flat_posts_haas_y_axis() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/millturn_flat.lathe.json"))?;
    let out = temp_path("campanion-lathe-millturn-flat.nc");
    let ir = temp_path("campanion-lathe-millturn-flat.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "haas_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("millturn_flat.haas_lathe.nc", &posted)?;
    assert_in_order(&posted, &["M154", "C90.", "M14", "P1800 M133", "G19", "Y", "M135", "M15", "G18", "M155"])?;

    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let doc = campanion_ir::Document::from_json(&ir_text).map_err(|err| err.to_string())?;
    let program = doc.program();
    let turning = program.turning().ok_or("turning setup is missing")?;
    let head = turning.heads().first().ok_or("head is missing")?;
    if !head.axes().contains(&campanion_ir::HeadAxis::Y) {
        return Err(format!("head axes are {:?}", head.axes()));
    }
    let path = program.toolpaths().iter().find(|path| path.name() == Some("od_flat")).ok_or("od_flat path is missing")?;
    let off_centre = path.blocks().iter().any(|block| match block {
        campanion_ir::ClBlock::Motion(campanion_ir::ClMotion::Rapid { to, .. } | campanion_ir::ClMotion::Linear { to, .. }) => {
            to.tip().y().abs() > 1e-6
        }
        _ => false,
    });
    if !off_centre {
        return Err("od_flat stays on the centreline".into());
    }
    Ok(())
}

#[test]
fn millturn_posts_linuxcnc_live_drill_and_slot() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/millturn.lathe.json"))?;
    let out = temp_path("campanion-lathe-millturn-linuxcnc.nc");
    let ir = temp_path("campanion-lathe-millturn-linuxcnc.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "linuxcnc_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("millturn.linuxcnc_lathe.nc", &posted)?;
    assert_in_order(&posted, &["S2500 $1", "M3 $1", "M5 $1", "S1800 $1", "M3 $1", "M5 $1"])?;
    if posted.contains("G93") || posted.contains("G112") || posted.contains("G107") {
        return Err(format!("indexed live cycle wrote inverse time or a native interpolation code\n{posted}"));
    }
    Ok(())
}

#[test]
fn millturn_flat_posts_linuxcnc_y_axis() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/millturn_flat.lathe.json"))?;
    let out = temp_path("campanion-lathe-millturn-flat-linuxcnc.nc");
    let ir = temp_path("campanion-lathe-millturn-flat-linuxcnc.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "linuxcnc_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("millturn_flat.linuxcnc_lathe.nc", &posted)?;
    assert_in_order(&posted, &["S1800 $1", "M3 $1", "G19", "Y", "M5 $1", "G18"])?;
    if posted.contains("G93") || posted.contains("G112") || posted.contains("G107") {
        return Err(format!("Y flat wrote inverse time or a native interpolation code\n{posted}"));
    }
    Ok(())
}

#[test]
fn live_flat_rejects_a_head_without_y_and_travel_past_the_limits() -> Result<(), String> {
    let missing = flat_shell("", FLAT_OP);
    assert_rejects(&missing, "campanion-lathe-flat-no-head.json", "operation od_flat: head has no Y axis")?;
    let disabled = flat_shell(r#"{ "y_axis": false }"#, FLAT_OP);
    assert_rejects(&disabled, "campanion-lathe-flat-y-off.json", "operation od_flat: head has no Y axis")?;
    let short = flat_shell(r#"{ "y_axis": true, "y_min": -1, "y_max": 1 }"#, FLAT_OP);
    assert_rejects(&short, "campanion-lathe-flat-travel.json", "outside head travel")?;
    Ok(())
}

#[test]
fn live_flat_fanuc_post_has_no_y_axis() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/millturn_flat.lathe.json"))?;
    let out = temp_path("campanion-lathe-millturn-flat-fanuc.nc");
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "fanuc_lathe"])?;
    let stderr = text(&output.stderr)?;
    let code = output.status.code();
    if code == Some(101) || stderr.to_ascii_lowercase().contains("panicked") {
        return Err(format!("lathe panicked ({code:?}): {stderr}"));
    }
    if code != Some(1) {
        return Err(format!("exit {code:?}, expected 1\n{stderr}"));
    }
    if !stderr.contains("post `fanuc_lathe` has no Y axis") {
        return Err(format!("stderr missing the Y-axis post error: {stderr}"));
    }
    if out.exists() {
        return Err(format!("failed job wrote {}", out.display()));
    }
    Ok(())
}

fn turning_shell(extra: &str, tools: &str, operation: &str) -> String {
    format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 30, "length": 20, "front_allowance": 1 }},
        "spindle": {{ "max_rpm": 3000 }},
        {extra}
        "tools": [{tools}],
        "operations": [{operation}],
        "clearance": 2
    }}"#
    )
}

#[test]
fn sub_spindle_job_rejects_a_missing_block_live_tooling_and_an_unknown_name() -> Result<(), String> {
    let insert = r#"{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }"#;
    let face = r#"{ "name": "back_face", "type": "face", "tool_id": 1, "spindle": "sub", "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }"#;
    assert_rejects(&turning_shell("", insert, face), "campanion-lathe-sub-missing.json", "operation back_face: spindle sub needs a \"sub\" block")?;

    let axial = r#"{ "id": 2, "station": 2, "live": "axial", "kind": "drill", "diameter": 6 }"#;
    let holes = r#"{ "name": "holes", "type": "live_drill", "tool_id": 2, "spindle": "sub", "face": "front", "z_start": 0, "positions": [{ "c": 0, "r": 10 }], "depth": 5, "rpm": 1000, "feed": 80 }"#;
    assert_rejects(
        &turning_shell("", axial, holes),
        "campanion-lathe-sub-live.json",
        "operation holes: live tooling on the sub-spindle is not supported",
    )?;
    let sub = r#""sub": { "max_rpm": 2000, "source": { "path": "missing.dxf" } },"#;
    assert_rejects(
        &turning_shell(sub, axial, holes),
        "campanion-lathe-sub-live-block.json",
        "operation holes: live tooling on the sub-spindle is not supported",
    )?;
    let mill = r#"{ "name": "mill", "type": "live_mill", "spindle": "sub" }"#;
    assert_rejects(
        &turning_shell("", "", mill),
        "campanion-lathe-sub-live-mill.json",
        "operation mill: live tooling on the sub-spindle is not supported",
    )?;

    let tail = r#"{ "name": "back_face", "type": "face", "tool_id": 1, "spindle": "tail", "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }"#;
    assert_rejects(&turning_shell("", insert, tail), "campanion-lathe-sub-tail.json", "operation back_face: unknown spindle `tail`")?;
    Ok(())
}

#[test]
fn subspindle_posts_haas_and_returns_to_the_main_before_the_end() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/subspindle.lathe.json"))?;
    let out = temp_path("campanion-lathe-subspindle.nc");
    let ir = temp_path("campanion-lathe-subspindle.ir.json");
    let _ = std::fs::remove_file(&out);
    let _ = std::fs::remove_file(&ir);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "haas_lathe", "--ir", &path_arg(&ir)?])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("subspindle.haas_lathe.nc", &posted)?;
    assert_in_order(&posted, &["T0101", "G50 S3000", "Z0.9", "G55", "G14", "T0202", "G50 S2500", "Z0.65", "G15", "G54", "G53 G0 X0", "M30"])?;

    let ir_text = std::fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    let doc = campanion_ir::Document::from_json(&ir_text).map_err(|err| err.to_string())?;
    let turning = doc.program().turning().ok_or("turning setup is missing")?;
    let names: Vec<&str> = turning.spindles().iter().map(|spindle| spindle.name()).collect();
    if names != ["main", "sub"] {
        return Err(format!("spindles are {names:?}"));
    }
    let sub = turning.spindle("sub").ok_or("sub spindle is missing")?;
    if sub.c_axis().is_some() || sub.reversed() || (sub.max_rpm() - 2_500.0).abs() > 1e-6 || sub.work_zero_z().abs() > 1e-9 {
        return Err(format!("sub spindle is {sub:?}"));
    }
    let bound: Vec<&str> = turning.bindings().iter().map(|binding| binding.spindle()).collect();
    if bound != ["main", "main", "main", "sub", "sub"] {
        return Err(format!("bindings are {bound:?}"));
    }
    if turning.section().is_none() {
        return Err("main section is missing".into());
    }
    Ok(())
}

const PROFILE_DXF: &str = "\
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
15.0
11
-30.0
21
15.0
0
ENDSEC
0
EOF
";

fn fixture_text(name: &str) -> Result<String, String> {
    let root = workspace_root()?;
    std::fs::read_to_string(root.join("crates/cli/tests/fixtures").join(name)).map_err(|err| err.to_string())
}

fn write_tree(dir_name: &str, files: &[(&str, &str)], job_name: &str, job: &str) -> Result<PathBuf, String> {
    let dir = temp_path(dir_name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    for (name, body) in files {
        std::fs::write(dir.join(name), body).map_err(|err| format!("write {name}: {err}"))?;
    }
    let job_path = dir.join(job_name);
    std::fs::write(&job_path, job).map_err(|err| err.to_string())?;
    Ok(job_path)
}

fn post_lathe(job: &Path, out_name: &str) -> Result<String, String> {
    let out = temp_path(out_name);
    let _ = std::fs::remove_file(&out);
    run_ok(&["lathe", &path_arg(job)?, "-o", &path_arg(&out)?, "--post", "haas_lathe"])?;
    std::fs::read_to_string(&out).map_err(|err| err.to_string())
}

fn assert_rejects_at(job: &Path, needle: &str) -> Result<(), String> {
    let out = job.with_extension("out.nc");
    let _ = std::fs::remove_file(&out);
    let output = campanion(&["lathe", &path_arg(job)?, "-o", &path_arg(&out)?, "--post", "haas_lathe"])?;
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

/// A C word is `C` followed by a number. `G112` and comments are not C words.
fn assert_no_c_word_between(text: &str) -> Result<(), String> {
    let Some(start) = text.find("G112") else {
        return Err(format!("missing G112\n{text}"));
    };
    let after = &text[start + "G112".len()..];
    let Some(end) = after.find("G113") else {
        return Err(format!("missing G113\n{text}"));
    };
    let span = &after[..end];
    for token in span.split_whitespace() {
        let mut chars = token.chars();
        if chars.next() == Some('C') && chars.next().is_some_and(|ch| ch.is_ascii_digit() || ch == '+' || ch == '-' || ch == '.') {
            return Err(format!("C word {token} between G112 and G113\n{span}"));
        }
    }
    Ok(())
}

fn mill_shell(head: &str, tool: &str, operation: &str) -> String {
    let head_field = if head.is_empty() { String::new() } else { format!("\"head\": {head},") };
    format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 50, "length": 40, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 4000, "c_axis": true }},
        {head_field}
        "tools": [{tool}],
        "operations": [{operation}],
        "tolerance": {{ "linear": 0.001, "chord": 0.01 }},
        "clearance": 2
    }}"#
    )
}

const AXIAL_MILL: &str = r#"{ "id": 1, "station": 1, "live": "axial", "kind": "endmill", "diameter": 4 }"#;
const RADIAL_MILL: &str = r#"{ "id": 1, "station": 1, "live": "radial", "kind": "endmill", "diameter": 6 }"#;

fn lathe_with_profile(tool: &str, head: &str, operation: &str) -> String {
    let head_field = if head.is_empty() { String::new() } else { format!("\"head\": {head},") };
    format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "profile.dxf", "layer": "PROFILE" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 50, "length": 40, "front_allowance": 0 }},
        "spindle": {{ "max_rpm": 4000, "c_axis": true }},
        {head_field}
        "tools": [{tool}],
        "operations": [{operation}],
        "tolerance": {{ "linear": 0.001, "chord": 0.01 }},
        "clearance": 2
    }}"#
    )
}

#[test]
fn millturn_face_posts_haas_g112_without_c() -> Result<(), String> {
    let root = workspace_root()?;
    let job = root.join("examples/millturn_face.lathe.json");
    let posted = post_lathe(&job, "campanion-lathe-millturn-face.nc")?;
    assert_golden("millturn_face.haas_lathe.nc", &posted)?;
    if !posted.contains("G112") || !posted.contains("G113") {
        return Err(format!("face pocket is missing G112/G113\n{posted}"));
    }
    assert_no_c_word_between(&posted)?;
    Ok(())
}

#[test]
fn millturn_od_posts_haas_g107() -> Result<(), String> {
    let slot = fixture_text("od_slot.dxf")?;
    let op = r#"{ "name": "od_wrap", "type": "live_mill", "tool_id": 1, "face": "od", "drawing": "od_slot.dxf", "layer": "SLOT", "strategy": "profile", "side": "on", "depth": 1.5, "diameter": 40, "rpm": 2000, "feed": 150 }"#;
    let job = lathe_with_profile(RADIAL_MILL, "", op);
    let path = write_tree("campanion-lathe-mill-od", &[("profile.dxf", PROFILE_DXF), ("od_slot.dxf", &slot)], "millturn_od.json", &job)?;
    let posted = post_lathe(&path, "campanion-lathe-millturn-od.nc")?;
    assert_golden("millturn_od.haas_lathe.nc", &posted)?;
    let marker = "G107 Y0 C0 R20.";
    let Some(at) = posted.find(marker) else {
        return Err(format!("missing {marker}\n{posted}"));
    };
    if !posted[at + marker.len()..].contains("G107") {
        return Err(format!("G107 was not cancelled\n{posted}"));
    }
    Ok(())
}

#[test]
fn millturn_y_axis_mills_across_the_centre() -> Result<(), String> {
    let drawing = fixture_text("y_cross.dxf")?;
    let op = r#"{ "name": "across", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "y_cross.dxf", "layer": "CROSS", "strategy": "profile", "side": "on", "depth": 1, "z_start": 0, "y_axis": true, "c": 30, "rpm": 2500, "feed": 180 }"#;
    let job = lathe_with_profile(AXIAL_MILL, r#"{ "y_axis": true, "y_min": -50.8, "y_max": 50.8 }"#, op);
    let path = write_tree("campanion-lathe-mill-y", &[("profile.dxf", PROFILE_DXF), ("y_cross.dxf", &drawing)], "millturn_y.json", &job)?;
    let posted = post_lathe(&path, "campanion-lathe-millturn-y.nc")?;
    assert_golden("millturn_y.haas_lathe.nc", &posted)?;
    if posted.contains("G112") {
        return Err(format!("y-axis mill posted face interpolation\n{posted}"));
    }
    for needle in ["Y", "C30.", "X-"] {
        if !posted.contains(needle) {
            return Err(format!("missing {needle}\n{posted}"));
        }
    }
    Ok(())
}

#[test]
fn live_mill_rejects_bad_drawings_and_heads() -> Result<(), String> {
    let c_early = r#"{ "name": "face_pocket", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "face.dxf", "strategy": "pocket", "depth": 1, "z_start": 0, "c": 10, "rpm": 1000, "feed": 100 }"#;
    assert_rejects(&mill_shell("", AXIAL_MILL, c_early), "campanion-lathe-mill-c.json", "c is only valid when y_axis is true")?;

    let needs_head = r#"{ "name": "across", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "face.dxf", "strategy": "profile", "side": "on", "depth": 1, "z_start": 0, "y_axis": true, "rpm": 1000, "feed": 100 }"#;
    assert_rejects(&mill_shell("", AXIAL_MILL, needs_head), "campanion-lathe-mill-no-y.json", "operation across: y_axis needs head.y_axis")?;

    let side = r#"{ "name": "face_pocket", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "face.dxf", "strategy": "pocket", "side": "outside", "depth": 1, "z_start": 0, "rpm": 1000, "feed": 100 }"#;
    assert_rejects(&mill_shell("", AXIAL_MILL, side), "campanion-lathe-mill-side.json", "unknown operation field `side`")?;

    let open = fixture_text("open_path.dxf")?;
    let open_op = r#"{ "name": "face_pocket", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "open_path.dxf", "layer": "OPEN", "strategy": "pocket", "depth": 1, "z_start": 0, "rpm": 1000, "feed": 100 }"#;
    let open_job = write_tree(
        "campanion-lathe-mill-open",
        &[("profile.dxf", PROFILE_DXF), ("open_path.dxf", &open)],
        "mill_open.json",
        &lathe_with_profile(AXIAL_MILL, "", open_op),
    )?;
    assert_rejects_at(&open_job, "operation face_pocket: drawing has no closed contour")?;

    let wide = fixture_text("wide_wrap.dxf")?;
    let wide_op = r#"{ "name": "od_wrap", "type": "live_mill", "tool_id": 1, "face": "od", "drawing": "wide_wrap.dxf", "layer": "WRAP", "strategy": "profile", "side": "on", "depth": 1, "diameter": 40, "rpm": 1000, "feed": 100 }"#;
    let wide_job = write_tree(
        "campanion-lathe-mill-wrap",
        &[("profile.dxf", PROFILE_DXF), ("wide_wrap.dxf", &wide)],
        "mill_wrap.json",
        &lathe_with_profile(RADIAL_MILL, "", wide_op),
    )?;
    assert_rejects_at(&wide_job, "operation od_wrap: drawing wraps past one turn")?;

    let high = fixture_text("y_high.dxf")?;
    let high_op = r#"{ "name": "across", "type": "live_mill", "tool_id": 1, "face": "front", "drawing": "y_high.dxf", "layer": "HIGH", "strategy": "profile", "side": "on", "depth": 1, "z_start": 0, "y_axis": true, "rpm": 1000, "feed": 100 }"#;
    let high_job = write_tree(
        "campanion-lathe-mill-high",
        &[("profile.dxf", PROFILE_DXF), ("y_high.dxf", &high)],
        "mill_high.json",
        &lathe_with_profile(AXIAL_MILL, r#"{ "y_axis": true, "y_min": -10, "y_max": 10 }"#, high_op),
    )?;
    assert_rejects_at(&high_job, "operation across: Y ")?;
    assert_rejects_at(&high_job, "outside head travel")?;
    Ok(())
}

#[test]
fn transfer_and_eject_reject_a_missing_sub_block_and_an_early_sub_op() -> Result<(), String> {
    let insert = r#"{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }"#;
    let handoff = r#"{ "name": "handoff", "type": "transfer", "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 }"#;
    assert_rejects(
        &turning_shell("", insert, handoff),
        "campanion-lathe-transfer-missing.json",
        "operation handoff: transfer needs a \"sub\" block",
    )?;
    let drop = r#"{ "name": "drop", "type": "eject" }"#;
    assert_rejects(&turning_shell("", insert, drop), "campanion-lathe-eject-missing.json", "operation drop: eject needs a \"sub\" block")?;

    let sub = r#""sub": { "max_rpm": 2000, "source": { "path": "missing.dxf" } },"#;
    let face = r#"{ "name": "back_face", "type": "face", "tool_id": 1, "spindle": "sub", "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }"#;
    let early = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 30, "length": 20, "front_allowance": 1 }},
        "spindle": {{ "max_rpm": 3000 }},
        {sub}
        "tools": [{insert}],
        "operations": [{face}, {handoff}],
        "clearance": 2
    }}"#
    );
    assert_rejects(&early, "campanion-lathe-transfer-order.json", "operation back_face: sub-spindle op before any transfer")?;
    let eject_first = format!(
        r#"{{
        "format": "campanion-lathe/1",
        "source": {{ "path": "missing.dxf" }},
        "units": "mm",
        "stock": {{ "kind": "bar", "diameter": 30, "length": 20, "front_allowance": 1 }},
        "spindle": {{ "max_rpm": 3000 }},
        {sub}
        "tools": [{insert}],
        "operations": [{drop}, {handoff}],
        "clearance": 2
    }}"#
    );
    assert_rejects(&eject_first, "campanion-lathe-eject-order.json", "operation drop: sub-spindle op before any transfer")?;
    Ok(())
}

#[test]
fn transfer_posts_haas_with_part_off_phase_sub_face_and_eject() -> Result<(), String> {
    let root = workspace_root()?;
    let job = path_arg(&root.join("examples/transfer.lathe.json"))?;
    let out = temp_path("campanion-lathe-transfer.nc");
    let _ = std::fs::remove_file(&out);
    run_ok(&["lathe", &job, "-o", &path_arg(&out)?, "--post", "haas_lathe"])?;
    let posted = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert_golden("transfer.haas_lathe.nc", &posted)?;
    assert_in_order(
        &posted,
        &[
            "M111",
            "M145",
            "G199 R90",
            "G00 B5",
            "G4 P500",
            "G01 B-2 F200",
            "M110",
            "T0202",
            "G00 B5",
            "G198",
            "G00 G53 B-100",
            "G55",
            "G14",
            "G15",
            "G54",
            "M30",
        ],
    )?;
    let g15 = posted.rfind("G15").ok_or("G15 missing")?;
    let g54 = posted.rfind("G54").ok_or("G54 missing")?;
    let end = posted.rfind("M30").ok_or("M30 missing")?;
    if !(g15 < g54 && g54 < end) {
        return Err(format!("G15 G54 are not before the end\n{posted}"));
    }
    Ok(())
}
