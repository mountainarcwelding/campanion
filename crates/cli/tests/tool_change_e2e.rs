//! Tool-change events and spindle speed for `campanion run` (GitHub issue #94).

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const PART_SVG: &str = concat!(
    r#"<svg xmlns="http://www.w3.org/2000/svg" width="120mm" height="80mm" viewBox="0 0 120 80">"#,
    r#"<g id="OUTLINE"><rect x="10" y="10" width="100" height="60"/></g>"#,
    r#"<g id="SLOT"><rect x="40" y="30" width="40" height="20"/></g>"#,
    "</svg>",
);

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

fn write_part(dir: &Path) -> Result<String, String> {
    std::fs::write(dir.join("part.svg"), PART_SVG).map_err(|err| err.to_string())?;
    Ok(fnv1a64(PART_SVG.as_bytes()))
}

fn job_json(hash: &str, tools: &str, operations: &str) -> String {
    format!(r#"{{"format":"campanion-job/1","source":{{"path":"part.svg","hash":"{hash}"}},"units":"mm","tools":{tools},"operations":{operations}}}"#)
}

const SLOT_OP: &str = r#"{
  "name":"slot","type":"pocket","layers":["SLOT"],"tool_id":2,"depth":3.0,"step_down":1.5,"stepover":1.2,
  "entry":{"type":"ramp","angle_deg":5},"feeds":{"cut":1500.0,"plunge":500.0}
}"#;

const OUTLINE_OP: &str = r#"{
  "name":"outline","type":"profile","layers":["OUTLINE"],"side":"outside","tool_id":1,"depth":6.0,"step_down":3.0,
  "entry":{"type":"ramp","angle_deg":5},"feeds":{"cut":1500.0,"plunge":500.0}
}"#;

fn two_tool_job(hash: &str) -> String {
    job_json(hash, r#"[{"id":1,"diameter":6.0,"speed":12000},{"id":2,"diameter":3.0,"speed":8000}]"#, &format!("[{SLOT_OP},{OUTLINE_OP}]"))
}

fn run_posted(dir: &Path, job: &str, out_name: &str, post: &str) -> Result<String, String> {
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join(out_name);
    let out_arg = path_arg(&out)?;
    let output = campanion(dir, &["run", "job.json", "-o", &out_arg, "--post", post])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
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

fn tool_change_ids(nc: &str) -> Vec<u32> {
    let mut ids = Vec::new();
    for line in nc.lines() {
        let code = strip_comments(line);
        let words: Vec<&str> = code.split_whitespace().collect();
        let has_m6 = words.contains(&"M6");
        let id = words.iter().find_map(|word| word.strip_prefix('T').and_then(|n| n.parse::<u32>().ok()));
        if let (true, Some(id)) = (has_m6, id) {
            ids.push(id);
        }
    }
    ids
}

fn m3_speeds(nc: &str) -> Result<Vec<u32>, String> {
    let mut speeds = Vec::new();
    for line in nc.lines() {
        let code = strip_comments(line);
        let words: Vec<&str> = code.split_whitespace().collect();
        if !words.contains(&"M3") {
            continue;
        }
        let Some(speed) = words.iter().find_map(|word| word.strip_prefix('S').and_then(|n| n.parse::<u32>().ok())) else {
            return Err(format!("M3 has no S word: {line}"));
        };
        speeds.push(speed);
    }
    if speeds.is_empty() {
        return Err("posted output has no M3".to_string());
    }
    Ok(speeds)
}

fn first_motion_after<'a>(nc: &'a str, marker: &str) -> Result<&'a str, String> {
    let Some(at) = nc.find(marker) else {
        return Err(format!("missing {marker}"));
    };
    for line in nc[at..].lines().skip(1) {
        let code = strip_comments(line);
        if code.split_whitespace().any(|word| matches!(word, "G0" | "G1" | "G2" | "G3")) {
            return Ok(&nc[at..]);
        }
    }
    Err(format!("no motion after {marker}"))
}

fn change_before_motion(nc: &str, marker: &str, tool: u32) -> Result<(), String> {
    let section = first_motion_after(nc, marker)?;
    let mut saw_change = false;
    let want = format!("T{tool}");
    for line in section.lines().skip(1) {
        let code = strip_comments(line);
        let words: Vec<&str> = code.split_whitespace().collect();
        if words.contains(&"M6") && words.contains(&want.as_str()) {
            saw_change = true;
        }
        if words.iter().any(|word| matches!(*word, "G0" | "G1" | "G2" | "G3")) {
            if saw_change {
                return Ok(());
            }
            return Err(format!("{marker}: motion before T{tool} M6\n{section}"));
        }
    }
    Err(format!("{marker}: T{tool} M6 not found before motion"))
}

fn assert_two_tool_repro(nc: &str, slot_rpm: u32, outline_rpm: u32) -> Result<(), String> {
    let changes = tool_change_ids(nc);
    if changes != [2, 1] {
        return Err(format!("tool changes {changes:?}, expected [2, 1]\n{nc}"));
    }
    change_before_motion(nc, "(toolpath slot)", 2)?;
    change_before_motion(nc, "(toolpath outline)", 1)?;
    let speeds = m3_speeds(nc)?;
    if speeds.iter().any(|rpm| *rpm != slot_rpm && *rpm != outline_rpm) {
        return Err(format!("unexpected M3 speeds {speeds:?}, expected {slot_rpm} and {outline_rpm}\n{nc}"));
    }
    if !speeds.contains(&slot_rpm) || !speeds.contains(&outline_rpm) {
        return Err(format!("M3 speeds {speeds:?} missing {slot_rpm} or {outline_rpm}\n{nc}"));
    }
    Ok(())
}

#[test]
fn two_tools_write_a_change_and_speed_for_mach3_haas_and_linuxcnc() -> Result<(), String> {
    let dir = temp_dir("campanion-tool-change-repro")?;
    let hash = write_part(&dir)?;
    let job = two_tool_job(&hash);
    for post in ["mach3", "haas", "linuxcnc"] {
        let nc = run_posted(&dir, &job, &format!("out-{post}.nc"), post)?;
        assert_two_tool_repro(&nc, 8000, 12000).map_err(|err| format!("{post}: {err}"))?;
    }
    Ok(())
}

#[test]
fn same_tool_two_ops_write_one_change() -> Result<(), String> {
    let root = workspace_root()?;
    let dir = temp_dir("campanion-tool-change-same")?;
    let out = dir.join("out.nc");
    let out_arg = path_arg(&out)?;
    let job = path_arg(&root.join("examples/sample.job.json"))?;
    let output = campanion(&root, &["run", &job, "-o", &out_arg, "--post", "mach3"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stderr {stderr}", output.status.code()));
    }
    let nc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    let changes = tool_change_ids(&nc);
    if changes != [1] {
        return Err(format!("same-tool job wrote {changes:?}, expected [1]\n{nc}"));
    }
    for speed in m3_speeds(&nc)? {
        if speed != 12000 {
            return Err(format!("M3 S{speed}, expected S12000\n{nc}"));
        }
    }
    Ok(())
}

#[test]
fn operation_speed_overrides_tool_speed() -> Result<(), String> {
    let dir = temp_dir("campanion-tool-change-override")?;
    let hash = write_part(&dir)?;
    let outline = r#"{
      "name":"outline","type":"profile","layers":["OUTLINE"],"side":"outside","tool_id":1,"depth":6.0,"step_down":3.0,
      "entry":{"type":"ramp","angle_deg":5},"speed":4500,"feeds":{"cut":1500.0,"plunge":500.0}
    }"#;
    let job = job_json(&hash, r#"[{"id":1,"diameter":6.0,"speed":12000},{"id":2,"diameter":3.0,"speed":8000}]"#, &format!("[{SLOT_OP},{outline}]"));
    let nc = run_posted(&dir, &job, "out.nc", "mach3")?;
    assert_two_tool_repro(&nc, 8000, 4500)
}

#[test]
fn missing_speed_fails_when_the_post_needs_it() -> Result<(), String> {
    let dir = temp_dir("campanion-tool-change-missing")?;
    let hash = write_part(&dir)?;
    let job = job_json(&hash, r#"[{"id":1,"diameter":6.0},{"id":2,"diameter":3.0}]"#, &format!("[{SLOT_OP},{OUTLINE_OP}]"));
    std::fs::write(dir.join("job.json"), &job).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let out_arg = path_arg(&out)?;
    let _ = std::fs::remove_file(&out);
    let output = campanion(&dir, &["run", "job.json", "-o", &out_arg, "--post", "mach3"])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(1) {
        return Err(format!("exit {:?} stderr {stderr}", output.status.code()));
    }
    if !stderr.contains("operation slot: no spindle speed; set tools[].speed or operations[].speed") {
        return Err(format!("missing speed error: {stderr}"));
    }
    if out.exists() {
        return Err(format!("missing speed wrote {}", out.display()));
    }
    let sink = dir.join("sink.ngc");
    let sink_arg = path_arg(&sink)?;
    let output = campanion(&dir, &["run", "job.json", "-o", &sink_arg])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("test sink without speed exit {:?} stderr {stderr}", output.status.code()));
    }
    Ok(())
}

#[test]
fn missing_speed_is_allowed_when_the_template_has_no_speed() -> Result<(), String> {
    let root = workspace_root()?;
    let dir = temp_dir("campanion-tool-change-no-speed-word")?;
    let hash = write_part(&dir)?;
    let job = job_json(&hash, r#"[{"id":1,"diameter":6.0},{"id":2,"diameter":3.0}]"#, &format!("[{SLOT_OP},{OUTLINE_OP}]"));
    std::fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let source = std::fs::read_to_string(root.join("crates/post/posts/mach3.json")).map_err(|err| err.to_string())?;
    let mut value: serde_json::Value = serde_json::from_str(&source).map_err(|err| err.to_string())?;
    value["process"]["spindle"]["on"] = serde_json::json!(["M3"]);
    std::fs::write(dir.join("post.json"), serde_json::to_string(&value).map_err(|err| err.to_string())?).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let out_arg = path_arg(&out)?;
    let post_arg = path_arg(&dir.join("post.json"))?;
    let output = campanion(&dir, &["run", "job.json", "-o", &out_arg, "--post", &post_arg])?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stderr {stderr}", output.status.code()));
    }
    let nc = std::fs::read_to_string(&out).map_err(|err| err.to_string())?;
    let changes = tool_change_ids(&nc);
    if changes != [2, 1] {
        return Err(format!("tool changes {changes:?}, expected [2, 1]\n{nc}"));
    }
    Ok(())
}
