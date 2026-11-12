//! Job + planner e2e for a PGM laser raster posted with `grbl_laser`.

#![allow(clippy::expect_used)]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn fnv(bytes: &[u8]) -> String {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x1000_0000_01b3);
    }
    format!("{hash:016x}")
}

fn campanion() -> Command {
    Command::new(env!("CARGO_BIN_EXE_campanion"))
}

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let root = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let dir = root.join(name);
    if dir.exists() {
        fs::remove_dir_all(&dir).map_err(|err| err.to_string())?;
    }
    fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn motion_s_words(text: &str) -> Vec<i64> {
    let mut values = Vec::new();
    for line in text.lines() {
        let body = line.split('(').next().unwrap_or(line);
        let tokens: Vec<&str> = body.split_whitespace().collect();
        if tokens.iter().any(|word| *word == "G0" || word.starts_with('M')) {
            continue;
        }
        if !tokens.iter().any(|word| matches!(*word, "G1" | "G2" | "G3") || word.starts_with('X') || word.starts_with('Y')) {
            continue;
        }
        for word in tokens {
            if let Some(rest) = word.strip_prefix('S')
                && let Ok(value) = rest.parse::<i64>()
            {
                values.push(value);
            }
        }
    }
    values
}

#[test]
fn raster_job_posts_per_move_s_including_s0_overscan() -> Result<(), String> {
    let dir = temp_dir("campanion-laser-raster-e2e")?;
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" width="10mm" height="10mm" viewBox="0 0 10 10"><g id="PART"><rect x="0" y="0" width="1" height="1"/></g></svg>"#;
    fs::write(dir.join("part.svg"), svg).map_err(|err| err.to_string())?;
    // 4×2 checker: black, white, black, white / white, black, white, black. Max 255.
    let mut pgm = b"P5\n4 2\n255\n".to_vec();
    pgm.extend_from_slice(&[0, 255, 0, 255, 255, 0, 255, 0]);
    fs::write(dir.join("art.pgm"), &pgm).map_err(|err| err.to_string())?;
    let hash = fnv(svg.as_bytes());
    let job = format!(
        r#"{{
          "format": "campanion-job/1",
          "source": {{"path": "part.svg", "hash": "{hash}"}},
          "units": "mm",
          "heights": {{"clearance": 5, "retract": 2}},
          "tools": [{{"id": 1, "kerf": 0.2}}],
          "operations": [{{
            "name": "engrave",
            "type": "laser_raster",
            "tool_id": 1,
            "image": "art.pgm",
            "origin": [0, 0],
            "width": 4,
            "height": 2,
            "step": 1,
            "overscan": 0.5,
            "scanning_offset": 0.2,
            "feeds": {{"cut": 2000}}
          }}]
        }}"#
    );
    fs::write(dir.join("job.json"), job).map_err(|err| err.to_string())?;
    let out = dir.join("out.nc");
    let ir = dir.join("out.json");
    let result = campanion()
        .args([
            "run",
            dir.join("job.json").to_str().expect("utf8"),
            "-o",
            out.to_str().expect("utf8"),
            "--ir",
            ir.to_str().expect("utf8"),
            "--post",
            "grbl_laser",
        ])
        .output()
        .map_err(|err| err.to_string())?;
    let stderr = String::from_utf8_lossy(&result.stderr);
    if result.status.code() != Some(0) {
        return Err(format!("run failed: {stderr}"));
    }
    let text = fs::read_to_string(&out).map_err(|err| err.to_string())?;
    assert!(!text.contains("power missing"), "{text}");
    assert!(!text.contains("M4 S1000"), "{text}");
    assert!(text.contains("M4 S0"), "{text}");
    let words = motion_s_words(&text);
    assert!(words.contains(&0), "missing S0 overscan in {words:?}\n{text}");
    assert!(words.contains(&1000), "missing full-power burn in {words:?}\n{text}");
    assert!(words.iter().all(|s| *s == 0 || *s == 1000), "{words:?}\n{text}");
    // IR must carry per-move power, including 0 on the overscan.
    let ir_text = fs::read_to_string(&ir).map_err(|err| err.to_string())?;
    assert!(ir_text.contains("\"power\": 0.0") || ir_text.contains("\"power\":0.0") || ir_text.contains("\"power\": 0"), "{ir_text}");
    assert!(ir_text.contains("\"power\": 1.0") || ir_text.contains("\"power\":1.0") || ir_text.contains("\"power\": 1"), "{ir_text}");
    let _ = Path::new(".");
    Ok(())
}
