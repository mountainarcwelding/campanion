use std::ffi::OsStr;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn campanion<I, S>(args: I) -> io::Result<Output>
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output()
}

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn success_stdout(args: &[&str]) -> Result<String, String> {
    let output = campanion(args).map_err(|err| err.to_string())?;
    let stderr = text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("exit {:?} stderr {stderr}", output.status.code()));
    }
    text(&output.stdout)
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
    if name.ends_with(".json") {
        return campanion_testsupport::ir_json_close(actual, &expected).map_err(|err| format!("{err} ({})", path.display()));
    }
    // G-code is already rounded to the post's decimals, so the golden stays byte-exact.
    if actual != expected {
        return Err(format!("golden mismatch for {} (set UPDATE_GOLDEN=1 to regenerate)", path.display()));
    }
    Ok(())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

#[test]
fn version_flag_exits_zero_and_prints_name() {
    let output = campanion(["--version"]).expect("failed to run campanion");
    assert_eq!(output.status.code(), Some(0));
    let stdout = text(&output.stdout).expect("utf-8");
    assert!(stdout.starts_with("campanion "), "stdout was {stdout:?}");
}

#[test]
fn help_lists_commands() {
    let stdout = success_stdout(&["--help"]).expect("help");
    for word in ["ir", "ngc", "verify", "schema", "--job", "--version", "--mesh", "bull:3,1", "posts", "--post"] {
        assert!(stdout.contains(word), "help missing {word}: {stdout}");
    }
    assert!(stdout.contains("campanion run JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]"), "{stdout}");
    assert!(stdout.contains("campanion lathe JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]"), "{stdout}");
    assert!(stdout.contains("campanion post IR.json --post ID|FILE.json [-o OUT]"), "{stdout}");
    assert!(stdout.contains("campanion posts"), "{stdout}");
}

#[test]
fn usage_errors_exit_two() {
    for args in [
        Vec::new(),
        vec!["nope"],
        vec!["verify"],
        vec!["ir", "--job"],
        vec!["schema", "extra"],
        vec!["verify", "file.json", "--mesh"],
        vec!["verify", "file.json", "--tool", "ball:3"],
        vec!["post"],
        vec!["posts", "extra"],
        vec!["run", "job.json", "-o", "out.ngc", "--post"],
        vec!["lathe"],
        vec!["lathe", "job.json"],
    ] {
        let output = campanion(&args).expect("run");
        let stderr = text(&output.stderr).expect("utf-8");
        assert_eq!(output.status.code(), Some(2), "{args:?} {stderr}");
        assert!(stderr.contains("Usage:"), "{args:?} {stderr}");
    }
}

#[test]
fn sample_ir_matches_golden() {
    let stdout = success_stdout(&["ir"]).expect("ir");
    assert!(stdout.contains("\"format\": \"campanion-ir/1\""), "{stdout}");
    assert_golden("sample.ir.json", &stdout).expect("golden ir");
}

#[test]
fn sample_ngc_matches_golden() {
    let stdout = success_stdout(&["ngc"]).expect("ngc");
    assert!(stdout.starts_with("G21\nG90\n"), "{stdout}");
    assert_golden("sample.ngc", &stdout).expect("golden ngc");
}

#[test]
fn job_file_matches_builtin_sample() {
    let sample = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/sample.job.json");
    let sample = sample.to_str().expect("sample path");
    let from_file = success_stdout(&["ir", "--job", sample]).expect("ir --job");
    let builtin = success_stdout(&["ir"]).expect("ir");
    assert_eq!(from_file, builtin);
    let ngc = success_stdout(&["ngc", "--job", sample]).expect("ngc --job");
    assert_eq!(ngc, success_stdout(&["ngc"]).expect("ngc"));
}

#[test]
fn profile_tabs_do_not_rapid_through_stock() {
    let ngc = profile_ngc("campanion-tabs-profile.json", None, r#"{"count": 6, "width": 10.0, "height": 3.0}"#, -12.0).expect("ngc");
    let moves = motions(&ngc);
    assert_approach(&moves, 5.0, 2.0, &ngc);
    let mut saw_tab_feed = false;
    let mut prev: Option<(f64, f64)> = None;
    for (rapid, x, y, z) in &moves {
        if !rapid
            && (z + 9.0).abs() < 1e-4
            && let Some((px, py)) = prev
            && ((px - x).abs() > 1e-6 || (py - y).abs() > 1e-6)
        {
            saw_tab_feed = true;
        }
        prev = Some((*x, *y));
    }
    assert!(saw_tab_feed, "tab crossings should be feed at Z -9\n{ngc}");
    let returns = moves.iter().filter(|(rapid, _, _, z)| !rapid && (z + 12.0).abs() < 1e-4).count();
    assert!(returns >= 6, "should re-reach Z -12 after each tab, got {returns}\n{ngc}");
}

#[test]
fn full_depth_tab_crosses_at_the_retract_height() {
    for (name, tabs) in
        [("campanion-tabs-full.json", r#"{"count": 2, "width": 8.0}"#), ("campanion-tabs-zero.json", r#"{"count": 2, "width": 8.0, "height": 0}"#)]
    {
        let ngc = profile_ngc(name, None, tabs, -6.0).expect("ngc");
        let moves = motions(&ngc);
        assert_approach(&moves, 5.0, 2.0, &ngc);
        assert!(full_depth_cross(&moves, 2.0, -6.0), "expected G1 up to Z2, G0 across at Z2, G1 down\n{ngc}");
    }
}

#[test]
fn profile_heights_are_honoured_and_rejected() {
    let custom = profile_ngc("campanion-heights-custom.json", Some(r#"{"clearance": 8.0, "retract": 3.0}"#), r#"{"count": 2, "width": 8.0}"#, -6.0)
        .expect("ngc");
    let moves = motions(&custom);
    assert_approach(&moves, 8.0, 3.0, &custom);
    assert!(full_depth_cross(&moves, 3.0, -6.0), "custom retract should be the tab plane\n{custom}");

    let equal =
        profile_ngc("campanion-heights-equal.json", Some(r#"{"clearance": 4, "retract": 4}"#), r#"{"count": 1, "width": 4.0, "height": 1.0}"#, -5.0)
            .expect("ngc");
    let equal_moves = motions(&equal);
    let (rapid, _, _, z) = equal_moves[0];
    assert!(rapid && (z - 4.0).abs() < 1e-4, "equal heights start at Z4, got {equal_moves:?}\n{equal}");
    assert!(!equal_moves[1].0, "the next motion is the plunge, got {:?}\n{equal}", equal_moves[1]);
    assert!(equal_moves.iter().all(|(is_rapid, _, _, z)| !is_rapid || *z + 1e-6 >= 4.0), "{equal}");

    let retract_only =
        profile_ngc("campanion-heights-retract.json", Some(r#"{"retract": 3.5}"#), r#"{"count": 1, "width": 4.0, "height": 1.0}"#, -5.0)
            .expect("ngc");
    assert_approach(&motions(&retract_only), 5.0, 3.5, &retract_only);

    for (name, heights, needle) in [
        ("campanion-heights-zero.json", r#"{"clearance": 5, "retract": 0}"#, "retract must be above the stock top"),
        ("campanion-heights-negative.json", r#"{"clearance": 5, "retract": -1}"#, "retract must be above the stock top"),
        ("campanion-heights-above.json", r#"{"clearance": 2, "retract": 4}"#, "retract is above clearance"),
        ("campanion-heights-text.json", r#"{"clearance": "high", "retract": 2}"#, "clearance must be a number"),
        ("campanion-heights-huge.json", r#"{"clearance": 1e309, "retract": 2}"#, "number out of range"),
        ("campanion-heights-inf.json", r#"{"clearance": 5, "retract": 1e309}"#, "number out of range"),
    ] {
        let path = write_profile(name, Some(heights), r#"{"count": 1, "width": 4.0}"#, -2.0).expect("write");
        let output = campanion(["ngc", "--job", path.to_str().expect("path")]).expect("run");
        let stderr = text(&output.stderr).expect("utf-8");
        assert_eq!(output.status.code(), Some(1), "{name} {stderr}");
        assert!(stderr.contains(needle), "{name} missing `{needle}` in {stderr}");
    }
}

#[test]
fn spindle_retracts_above_the_stock_before_it_stops() {
    let ngc = profile_ngc("campanion-spindle-stop.json", None, r#"{"count": 1, "width": 4.0, "height": 1.0}"#, -12.0).expect("ngc");
    let before = code_before_process_off(&ngc).expect("m5");
    let z = motion_z(&before).unwrap_or_else(|| panic!("line before M5 is not a move: {before}\n{ngc}"));
    assert!(before.split_whitespace().any(|word| word == "G0"), "expected G0 before M5, got {before}\n{ngc}");
    assert!(z > 0.0, "M5 at Z{z}, expected a retract above the stock\n{ngc}");
}

#[test]
fn laser_stops_at_the_cut_before_the_lift() {
    let path = write_cut("campanion-laser-stop.json", "laser", r#"{"id": 1, "name": "beam", "kerf": 0.2}"#, "on", -12.0).expect("write");
    let path = path.to_str().expect("path");
    let ngc = success_stdout(&["ngc", "--job", path]).expect("ngc");
    let before = code_before_process_off(&ngc).expect("m5");
    let z = motion_z(&before).unwrap_or_else(|| panic!("line before M5 is not a move: {before}\n{ngc}"));
    assert!(z <= 0.0, "laser M5 at Z{z}, expected the beam off at the cut\n{ngc}");
    let after = code_after_process_off(&ngc).expect("lift");
    let lift = motion_z(&after).unwrap_or_else(|| panic!("line after M5 is not a move: {after}\n{ngc}"));
    assert!(after.split_whitespace().any(|word| word == "G0") && lift > 0.0, "expected a lift after M5, got {after}\n{ngc}");
}

fn profile_ngc(name: &str, heights: Option<&str>, tabs: &str, z: f64) -> Result<String, String> {
    let path = write_profile(name, heights, tabs, z)?;
    let path = path.to_str().ok_or_else(|| format!("{name}: path is not utf-8"))?;
    success_stdout(&["ngc", "--job", path]).map_err(|err| format!("{name}: {err}"))
}

fn write_profile(name: &str, heights: Option<&str>, tabs: &str, z: f64) -> Result<PathBuf, String> {
    let path = temp_path(name);
    let heights = match heights {
        Some(heights) => format!("\n  \"heights\": {heights},"),
        None => String::new(),
    };
    std::fs::write(
        &path,
        format!(
            r#"{{
  "name": "tabs",
  "process": "spindle",
  "cut_feed": 1500.0,{heights}
  "tool": {{"id": 1, "name": "6 mm end mill", "diameter": 6.0}},
  "feeds": {{"plunge": 500.0}},
  "params": {{"speed": 18000.0}},
  "profile": {{
    "side": "outside",
    "direction": "climb",
    "compensation": "computed",
    "z": {z},
    "tabs": {tabs},
    "closed": true,
    "points": [[0,0],[35,0],[35,20],[65,20],[65,0],[100,0],[100,60],[0,60]]
  }}
}}"#
        ),
    )
    .map_err(|err| format!("{name}: {err}"))?;
    Ok(path)
}

fn write_cut(name: &str, process: &str, tool: &str, side: &str, z: f64) -> Result<PathBuf, String> {
    let path = temp_path(name);
    std::fs::write(
        &path,
        format!(
            r#"{{
  "name": "cut",
  "process": "{process}",
  "cut_feed": 1500.0,
  "tool": {tool},
  "feeds": {{"plunge": 500.0}},
  "params": {{"power": 0.8, "speed": 18000.0}},
  "profile": {{
    "side": "{side}",
    "direction": "climb",
    "compensation": "computed",
    "z": {z},
    "closed": true,
    "points": [[0,0],[40,0],[40,20],[0,20]]
  }}
}}"#
        ),
    )
    .map_err(|err| format!("{name}: {err}"))?;
    Ok(path)
}

fn code_lines(ngc: &str) -> Vec<&str> {
    ngc.lines().filter(|line| !strip_code(line).is_empty()).collect()
}

fn strip_code(line: &str) -> String {
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

fn is_process_off(line: &str) -> bool {
    strip_code(line).split_whitespace().any(|word| word == "M5")
}

fn code_before_process_off(ngc: &str) -> Result<String, String> {
    let lines = code_lines(ngc);
    let index = lines.iter().position(|line| is_process_off(line)).ok_or_else(|| format!("no M5\n{ngc}"))?;
    lines.get(index.wrapping_sub(1)).map(|line| strip_code(line)).ok_or_else(|| format!("M5 is the first code line\n{ngc}"))
}

fn code_after_process_off(ngc: &str) -> Result<String, String> {
    let lines = code_lines(ngc);
    let index = lines.iter().position(|line| is_process_off(line)).ok_or_else(|| format!("no M5\n{ngc}"))?;
    let next = lines.get(index + 1).ok_or_else(|| format!("nothing after M5\n{ngc}"))?;
    Ok(strip_code(next))
}

fn motion_z(code: &str) -> Option<f64> {
    code.split_whitespace().find_map(|word| word.strip_prefix('Z').and_then(|text| text.parse().ok()))
}

fn motions(ngc: &str) -> Vec<(bool, f64, f64, f64)> {
    ngc.lines().filter_map(parse_xyz).collect()
}

fn assert_approach(moves: &[(bool, f64, f64, f64)], clearance: f64, retract: f64, ngc: &str) {
    assert!(moves.len() >= 3, "expected approach moves\n{ngc}");
    let (rapid, x, y, z) = moves[0];
    assert!(rapid, "first motion should be a rapid\n{ngc}");
    assert!((z - clearance).abs() < 1e-4, "first motion Z {z} expected {clearance}\n{ngc}");
    let (rapid, x2, y2, z2) = moves[1];
    assert!(rapid, "second motion should be a rapid\n{ngc}");
    assert!((x2 - x).abs() < 1e-4 && (y2 - y).abs() < 1e-4, "second rapid should keep the pierce XY\n{ngc}");
    assert!((z2 - retract).abs() < 1e-4, "second motion Z {z2} expected {retract}\n{ngc}");
    assert!(!moves[2].0, "third motion should be a feed\n{ngc}");
    assert!(moves.iter().all(|(is_rapid, _, _, z)| !is_rapid || *z + 1e-6 >= retract), "G0 below retract {retract}\n{ngc}");
}

/// G1 up to the retract height, G0 across at that height, G1 back to depth.
fn full_depth_cross(moves: &[(bool, f64, f64, f64)], retract: f64, depth: f64) -> bool {
    moves.windows(3).any(|window| {
        let (up_rapid, ux, uy, uz) = window[0];
        let (across_rapid, ax, ay, az) = window[1];
        let (down_rapid, dx, dy, dz) = window[2];
        !up_rapid
            && (uz - retract).abs() < 1e-4
            && across_rapid
            && (az - retract).abs() < 1e-4
            && ((ux - ax).abs() > 1e-4 || (uy - ay).abs() > 1e-4)
            && !down_rapid
            && (dx - ax).abs() < 1e-4
            && (dy - ay).abs() < 1e-4
            && (dz - depth).abs() < 1e-4
    })
}

fn parse_xyz(line: &str) -> Option<(bool, f64, f64, f64)> {
    let rapid = if line.starts_with("G0 ") {
        true
    } else if line.starts_with("G1 ") || line.contains("G2 ") || line.contains("G3 ") {
        false
    } else {
        return None;
    };
    let word = |key: &str| {
        line.split_whitespace().find_map(|word| {
            let rest = word.strip_prefix(key)?;
            rest.parse::<f64>().ok()
        })
    };
    Some((rapid, word("X")?, word("Y")?, word("Z")?))
}

#[test]
fn bad_job_reports_invalid_input() {
    let path = temp_path("campanion-bad-job.json");
    std::fs::write(&path, "{}").expect("write job");
    let output = campanion(["ir", "--job", path.to_str().expect("path")]).expect("run");
    let stderr = text(&output.stderr).expect("utf-8");
    assert_eq!(output.status.code(), Some(1), "{stderr}");
    assert!(stderr.contains("invalid input"), "{stderr}");
    assert!(stderr.contains("process"), "{stderr}");
}

#[test]
fn verify_clean_and_broken() {
    let clean_path = temp_path("campanion-verify-clean.json");
    std::fs::write(&clean_path, success_stdout(&["ir"]).expect("ir")).expect("write clean ir");
    let clean = campanion(["verify", clean_path.to_str().expect("clean path")]).expect("verify clean");
    let clean_stdout = text(&clean.stdout).expect("utf-8");
    assert_eq!(clean.status.code(), Some(0), "stderr {} stdout {clean_stdout}", text(&clean.stderr).expect("utf-8"));
    assert!(clean_stdout.contains("status ok"), "{clean_stdout}");
    assert!(clean_stdout.contains("issues 0"), "{clean_stdout}");
    assert!(clean_stdout.contains("bounds_min_mm"), "{clean_stdout}");

    let broken_path = temp_path("campanion-verify-broken.json");
    std::fs::write(&broken_path, BROKEN_IR).expect("write broken ir");
    let broken = campanion(["verify", broken_path.to_str().expect("broken path")]).expect("verify broken");
    let broken_stdout = text(&broken.stdout).expect("utf-8");
    assert_eq!(broken.status.code(), Some(1), "stderr {} stdout {broken_stdout}", text(&broken.stderr).expect("utf-8"));
    assert!(broken_stdout.contains("status errors"), "{broken_stdout}");
    assert!(broken_stdout.contains("process_not_closed"), "{broken_stdout}");
    assert!(broken_stdout.contains("torch"), "{broken_stdout}");

    let invalid_path = temp_path("campanion-verify-invalid.json");
    std::fs::write(&invalid_path, "{").expect("write invalid");
    let invalid = campanion(["verify", invalid_path.to_str().expect("invalid path")]).expect("verify invalid");
    let stderr = text(&invalid.stderr).expect("utf-8");
    assert_eq!(invalid.status.code(), Some(1), "{stderr}");
    assert!(stderr.contains("json"), "{stderr}");
}

#[test]
fn schema_prints_json_schema() {
    let stdout = success_stdout(&["schema"]).expect("schema");
    assert!(stdout.contains("\"$schema\""), "{stdout}");
    assert!(stdout.contains("format"), "{stdout}");
}

const BROKEN_IR: &str = r#"{
  "format": "campanion-ir/1",
  "program": {
    "tools": [],
    "toolpaths": [
      {
        "compensation": "off",
        "feeds": { "cut": 100.0 },
        "tabs": [],
        "blocks": [
          {
            "type": "process_on",
            "process": "torch",
            "params": {
              "pierce_delay_s": 0.0,
              "pierce_process_on_during_delay": true,
              "post_delay_s": 0.0,
              "post_process_on_during_delay": false,
              "extra": {}
            }
          }
        ]
      }
    ],
    "channel_programs": [],
    "syncs": []
  }
}
"#;
