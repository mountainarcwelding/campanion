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
    let path = temp_path("campanion-tabs-profile.json");
    std::fs::write(
        &path,
        r#"{
  "name": "tabs",
  "process": "spindle",
  "cut_feed": 1500.0,
  "tool": {"id": 1, "name": "6 mm end mill", "diameter": 6.0},
  "feeds": {"plunge": 500.0},
  "params": {"speed": 18000.0},
  "profile": {
    "side": "outside",
    "direction": "climb",
    "compensation": "computed",
    "z": -12.0,
    "tabs": {"count": 6, "width": 10.0, "height": 3.0},
    "closed": true,
    "points": [[0,0],[35,0],[35,20],[65,20],[65,0],[100,0],[100,60],[0,60]]
  }
}"#,
    )
    .expect("write job");
    let ngc = success_stdout(&["ngc", "--job", path.to_str().expect("path")]).expect("ngc --job");
    let mut first_motion = None;
    let mut prev: Option<(f64, f64, f64)> = None;
    let mut saw_tab_feed = false;
    let mut returns = 0;
    for line in ngc.lines() {
        let Some((rapid, x, y, z)) = parse_xyz(line) else {
            continue;
        };
        if first_motion.is_none() {
            first_motion = Some((rapid, z));
        }
        if rapid {
            if let Some((px, py, pz)) = prev {
                let xy = (px - x).abs() > 1e-6 || (py - y).abs() > 1e-6;
                assert!(!(xy && pz.min(z) < 0.0), "XY rapid below the stock: {line}");
            }
        } else if (z + 9.0).abs() < 1e-4
            && let Some((px, py, _)) = prev
            && ((px - x).abs() > 1e-6 || (py - y).abs() > 1e-6)
        {
            saw_tab_feed = true;
        }
        if !rapid && (z + 12.0).abs() < 1e-4 {
            returns += 1;
        }
        prev = Some((x, y, z));
    }
    let (first_rapid, first_z) = first_motion.expect("motion");
    assert!(first_rapid, "first motion should be a rapid, got {ngc}");
    assert!(first_z >= 0.0, "first motion Z {first_z} is below the stock top");
    assert!(saw_tab_feed, "tab crossings should be feed at Z -9\n{ngc}");
    assert!(returns >= 6, "should re-reach Z -12 after each tab, got {returns}\n{ngc}");
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
