//! Holder collision on `examples/dome_holder.job.json`.
//!
//! The example is the lead/tilt dome finish plus an assembly and a vise box that clears.
//! A short stickout on the same path fails and names the holder and the move.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn workspace_root() -> Result<PathBuf, String> {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().map_err(|err| err.to_string())
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    dir.join(name)
}

fn output_text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn path_arg(path: &Path) -> Result<String, String> {
    path.to_str().map(str::to_string).ok_or_else(|| format!("path is not utf-8: {}", path.display()))
}

fn campanion(cwd: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(cwd).output().map_err(|err| err.to_string())
}

fn assert_golden(name: &str, actual: &str) -> Result<(), String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden").join(name);
    if std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1") {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|err| err.to_string())?;
        }
        std::fs::write(&path, actual).map_err(|err| format!("write {}: {err}", path.display()))?;
        return Ok(());
    }
    let expected = std::fs::read_to_string(&path).map_err(|err| format!("read {}: {err} (set UPDATE_GOLDEN=1 to create)", path.display()))?;
    if actual == expected {
        return Ok(());
    }
    let actual_lines: Vec<&str> = actual.lines().collect();
    let expected_lines: Vec<&str> = expected.lines().collect();
    if actual_lines.len() != expected_lines.len() {
        return Err(format!(
            "golden mismatch for {}: {} lines, expected {} (set UPDATE_GOLDEN=1 to regenerate)",
            path.display(),
            actual_lines.len(),
            expected_lines.len()
        ));
    }
    for (index, (got, want)) in actual_lines.iter().zip(&expected_lines).enumerate() {
        if !lines_match_last_digit(got, want) {
            let from = index.saturating_sub(10);
            let to = (index + 40).min(actual_lines.len());
            eprintln!("--- got lines {}..{} of {}\n{}", from + 1, to, path.display(), actual_lines[from..to].join("\n"));
            return Err(format!(
                "golden mismatch for {} line {}:\n  got  {got}\n  want {want}\n(set UPDATE_GOLDEN=1 to regenerate)",
                path.display(),
                index + 1
            ));
        }
    }
    Ok(())
}

fn lines_match_last_digit(got: &str, want: &str) -> bool {
    let got_words: Vec<&str> = got.split_whitespace().collect();
    let want_words: Vec<&str> = want.split_whitespace().collect();
    got_words.len() == want_words.len() && got_words.iter().zip(&want_words).all(|(left, right)| left == right || words_match_last_digit(left, right))
}

fn words_match_last_digit(left: &str, right: &str) -> bool {
    let split = |word: &str| -> Option<(char, f64, usize)> {
        let mut chars = word.chars();
        let letter = chars.next().filter(char::is_ascii_alphabetic)?;
        let number = chars.as_str();
        let value: f64 = number.parse().ok()?;
        let decimals = number.split_once('.').map_or(0, |(_, frac)| frac.len());
        Some((letter, value, decimals))
    };
    let (Some((left_letter, left_value, left_decimals)), Some((right_letter, right_value, right_decimals))) = (split(left), split(right)) else {
        return false;
    };
    let decimals = left_decimals.max(right_decimals).max(1);
    let unit = 10_f64.powi(-i32::try_from(decimals).unwrap_or(9));
    left_letter == right_letter && (left_value - right_value).abs() <= unit * 1.000_001
}

#[test]
fn dome_holder_passes_and_posts() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-dome-holder.ngc");
    let ir_path = temp_path("campanion-dome-holder.json");
    let output = campanion(&root, &["run", "examples/dome_holder.job.json", "-o", &path_arg(&ngc_path)?, "--ir", &path_arg(&ir_path)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("source hash did not match: {stderr}"));
    }
    let posts = [("linuxcnc_5ax", "dome_holder.linuxcnc_5ax.nc"), ("haas_umc_tcpc", "dome_holder.haas_umc_tcpc.nc")];
    for (post, golden) in posts {
        let ngc = temp_path(&format!("campanion-dome-holder-{post}.nc"));
        let output = campanion(&root, &["run", "examples/dome_holder.job.json", "-o", &path_arg(&ngc)?, "--post", post])?;
        let stderr = output_text(&output.stderr)?;
        let stdout = output_text(&output.stdout)?;
        if output.status.code() != Some(0) {
            return Err(format!("{post} exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
        }
        let text = std::fs::read_to_string(&ngc).map_err(|err| format!("read {post}: {err}"))?;
        assert_golden(golden, &text)?;
    }
    Ok(())
}

#[test]
fn short_stickout_fails_naming_the_holder_and_the_move() -> Result<(), String> {
    let root = workspace_root()?;
    let example = std::fs::read_to_string(root.join("examples/dome_holder.job.json")).map_err(|err| err.to_string())?;
    let short = example
        .replace(r#""stickout": 50"#, r#""stickout": 6"#)
        .replace(r#""flute_length": 15"#, r#""flute_length": 6"#)
        .replace(r#""diameter": 16"#, r#""diameter": 40"#);
    let dir = temp_path("campanion-dome-holder-short");
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    let job = dir.join("dome_holder.job.json");
    std::fs::write(&job, short).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/dome.stl"), dir.join("dome.stl")).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/table_table_bc.kin.json"), dir.join("table_table_bc.kin.json")).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let output = campanion(&dir, &["run", &path_arg(&job)?, "-o", &path_arg(&ngc)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() == Some(0) {
        return Err(format!("short stickout was expected to fail; stdout {stdout} stderr {stderr}"));
    }
    if ngc.exists() {
        return Err("short stickout wrote output".into());
    }
    if !stderr.contains("holder 1") {
        return Err(format!("error does not name the holder: {stderr}"));
    }
    if !stderr.contains("move ") {
        return Err(format!("error does not name the move: {stderr}"));
    }
    if !stderr.contains("operation `dome`") {
        return Err(format!("error does not name the operation: {stderr}"));
    }
    Ok(())
}

#[test]
fn millturn_holder_passes() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc = temp_path("campanion-millturn-holder.ngc");
    let output = campanion(&root, &["lathe", "examples/millturn_holder.lathe.json", "-o", &path_arg(&ngc)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("lathe exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if !ngc.is_file() {
        return Err("millturn holder wrote no output".into());
    }
    Ok(())
}

#[test]
fn wide_holder_near_the_chuck_fails_and_writes_nothing() -> Result<(), String> {
    let root = workspace_root()?;
    let example = std::fs::read_to_string(root.join("examples/millturn_holder.lathe.json")).map_err(|err| err.to_string())?;
    let wide = example
        .replace(r#""from": -40"#, r#""from": -54"#)
        .replace(r#""to": -50"#, r#""to": -55"#)
        .replace(r#""stickout": 28"#, r#""stickout": 12"#)
        .replace(r#""flute_length": 16"#, r#""flute_length": 8"#)
        .replace(r#""diameter": 20"#, r#""diameter": 32"#);
    let dir = temp_path("campanion-millturn-holder-wide");
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/shaft.dxf"), dir.join("shaft.dxf")).map_err(|err| err.to_string())?;
    let job = dir.join("millturn_holder.lathe.json");
    std::fs::write(&job, wide).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let output = campanion(&dir, &["lathe", &path_arg(&job)?, "-o", &path_arg(&ngc)?])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() == Some(0) {
        return Err(format!("wide holder was expected to fail; stdout {stdout} stderr {stderr}"));
    }
    if ngc.exists() {
        return Err("wide holder wrote output".into());
    }
    for needle in ["holder 1", "chuck", "move ", "od_slot"] {
        if !stderr.contains(needle) {
            return Err(format!("error does not contain {needle}: {stderr}"));
        }
    }
    Ok(())
}
