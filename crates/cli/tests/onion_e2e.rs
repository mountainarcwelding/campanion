//! A small outside profile with `onion_skin` posts the skin as its own last pass.
//!
//! Depth 6 mm at a 2 mm step with a 1 mm skin is Z −2, −4, −5, then −6. The same job
//! without the field is Z −2, −4, −6. `verify` reports no issues. This output has not
//! been proven on a machine.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

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

fn square_dxf() -> String {
    let pts = [(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0), (0.0, 0.0)];
    let mut entities = String::new();
    for pair in pts.windows(2) {
        let (x0, y0) = pair[0];
        let (x1, y1) = pair[1];
        entities.push_str(&format!("0\nLINE\n8\nPLATE\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n"));
    }
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn job(hash: &str, onion: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, "diameter": 6, "speed": 12000 }} ],
  "operations": [ {{
    "name": "plate",
    "type": "profile",
    "layers": ["PLATE"],
    "side": "outside",
    "tool_id": 1,
    "depth": 6,
    "step_down": 2{onion},
    "feeds": {{ "cut": 1000, "plunge": 300 }}
  }} ]
}}"#
    )
}

fn campanion(dir: &Path, args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).current_dir(dir).output().map_err(|err| err.to_string())
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

/// Distinct feed Z levels, in order. A later feed that stays at the same Z is one level.
fn feed_levels(ngc: &str) -> Result<Vec<f64>, String> {
    let mut modal: Option<i32> = None;
    let mut z = 0.0;
    let mut levels: Vec<f64> = Vec::new();
    for (index, raw) in ngc.lines().enumerate() {
        let code = strip_comment(raw);
        if code.is_empty() {
            continue;
        }
        let parsed = words(&code).map_err(|err| format!("line {}: {err}", index + 1))?;
        let mut group = modal;
        let mut moved = false;
        let mut line_z = None;
        for (letter, value) in &parsed {
            if *letter == 'G' && *value >= 0.0 && *value <= 3.0 && (value - value.round()).abs() < 1e-6 {
                group = Some(*value as i32);
                modal = group;
            }
            if *letter == 'Z' {
                z = *value;
                line_z = Some(*value);
                moved = true;
            }
            if *letter == 'X' || *letter == 'Y' {
                moved = true;
            }
        }
        if moved && group == Some(1) {
            let level = line_z.unwrap_or(z);
            let fresh = match levels.last() {
                None => true,
                Some(&prev) => (level - prev).abs() > EPS,
            };
            if fresh {
                levels.push(level);
            }
        }
    }
    Ok(levels)
}

fn near_list(actual: &[f64], expected: &[f64]) -> bool {
    actual.len() == expected.len() && actual.iter().zip(expected).all(|(a, b)| (a - b).abs() <= EPS)
}

fn post(dir: &Path, onion: &str) -> Result<Vec<f64>, String> {
    let drawing = square_dxf();
    let hash = fnv1a64(drawing.as_bytes());
    std::fs::write(dir.join("plate.dxf"), drawing).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("job.json"), job(&hash, onion)).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let ir = dir.join("out.json");
    let ngc_arg = ngc.to_str().ok_or("ngc path")?;
    let ir_arg = ir.to_str().ok_or("ir path")?;
    let output = campanion(dir, &["run", "job.json", "-o", ngc_arg, "--ir", ir_arg, "--post", "linuxcnc"])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("run exit {:?}\n{stdout}\n{stderr}", output.status.code()));
    }
    let posted = std::fs::read_to_string(&ngc).map_err(|err| err.to_string())?;
    let verify = campanion(dir, &["verify", ir_arg])?;
    let verify_out = text(&verify.stdout)?;
    let verify_err = text(&verify.stderr)?;
    if verify.status.code() != Some(0) {
        return Err(format!("verify exit {:?}\n{verify_out}\n{verify_err}", verify.status.code()));
    }
    if !verify_out.lines().any(|line| line == "status ok") || !verify_out.lines().any(|line| line == "issues 0") {
        return Err(format!("verify reported errors:\n{verify_out}{verify_err}"));
    }
    feed_levels(&posted)
}

#[test]
fn profile_onion_skin_posts_the_skin_last() -> Result<(), String> {
    let skinned = post(&temp_dir("campanion-onion-skin")?, r#","onion_skin":1"#)?;
    if !near_list(&skinned, &[-2.0, -4.0, -5.0, -6.0]) {
        return Err(format!("skinned levels {skinned:?}"));
    }
    let plain = post(&temp_dir("campanion-onion-plain")?, "")?;
    if !near_list(&plain, &[-2.0, -4.0, -6.0]) {
        return Err(format!("plain levels {plain:?}"));
    }
    Ok(())
}
