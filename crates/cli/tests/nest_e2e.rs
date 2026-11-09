//! Sheet nesting on `campanion run`.
//!
//! `examples/nest.job.json` engraves two 20 mm squares onto a 50 by 30 mm sheet with a 4 mm gap.
//! The placements file is the bytes `--nest-out` writes for that job.

use std::path::PathBuf;
use std::process::{Command, Output};

fn text(bytes: &[u8]) -> Result<String, String> {
    String::from_utf8(bytes.to_vec()).map_err(|err| err.to_string())
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn temp_dir(name: &str) -> Result<PathBuf, String> {
    let root = std::env::var_os("CARGO_TARGET_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let dir = root.join(name);
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    Ok(dir)
}

fn campanion(args: &[&str]) -> Result<Output, String> {
    Command::new(env!("CARGO_BIN_EXE_campanion")).args(args).output().map_err(|err| err.to_string())
}

fn square_dxf() -> String {
    let mut entities = String::new();
    let points = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)];
    for pair in points.windows(2) {
        let (x0, y0) = pair[0];
        let (x1, y1) = pair[1];
        entities.push_str(&format!("0\nLINE\n8\nPLATE\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n"));
    }
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

#[test]
fn golden_nest_matches_the_example() -> Result<(), String> {
    let root = workspace_root();
    let job = root.join("examples/nest.job.json");
    let dir = temp_dir("nest-golden")?;
    let ngc = dir.join("out.ngc");
    let placed = dir.join("placements.json");
    let output = campanion(&[
        "run",
        job.to_str().ok_or("job path")?,
        "-o",
        ngc.to_str().ok_or("ngc path")?,
        "--nest-out",
        placed.to_str().ok_or("placements path")?,
    ])?;
    let stderr = text(&output.stderr)?;
    let stdout = text(&output.stdout)?;
    assert_eq!(output.status.code(), Some(0), "stdout {stdout} stderr {stderr}");
    assert!(stderr.contains("nest: `left` at (0, 0) mm, 20 x 20 mm, rotation 0 deg"), "{stderr}");
    assert!(stderr.contains("nest: `right` at (24, 0) mm, 20 x 20 mm, rotation 0 deg"), "{stderr}");
    assert!(!stderr.contains("warning"), "{stderr}");
    let ngc_text = std::fs::read_to_string(&ngc).map_err(|err| err.to_string())?;
    assert!(ngc_text.contains("X24.0000"), "{ngc_text}");
    assert!(ngc_text.contains("X44.0000"), "{ngc_text}");
    let actual = std::fs::read(&placed).map_err(|err| err.to_string())?;
    let expected = std::fs::read(root.join("examples/nest.placements.json")).map_err(|err| err.to_string())?;
    assert_eq!(actual, expected);
    Ok(())
}

#[test]
fn a_part_larger_than_the_sheet_writes_nothing() -> Result<(), String> {
    let dir = temp_dir("nest-too-big")?;
    let drawing = square_dxf();
    let hash = fnv1a64(drawing.as_bytes());
    std::fs::write(dir.join("plate.dxf"), &drawing).map_err(|err| err.to_string())?;
    let job = format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{ "path": "plate.dxf", "hash": "{hash}" }},
  "units": "mm",
  "tools": [ {{ "id": 1, "diameter": 6, "speed": 1000 }} ],
  "operations": [ {{
    "name": "plate",
    "type": "engrave",
    "layers": ["PLATE"],
    "tool_id": 1,
    "depth": 1,
    "step_down": 1,
    "feeds": {{ "cut": 100 }}
  }} ],
  "sheet": {{ "width": 40, "height": 40 }},
  "nest": {{ "parts": [ {{ "name": "block", "width": 50, "height": 10 }} ] }}
}}"#
    );
    let job_path = dir.join("job.json");
    std::fs::write(&job_path, job).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let placed = dir.join("placements.json");
    let output =
        campanion(&["run", job_path.to_str().ok_or("job")?, "-o", ngc.to_str().ok_or("ngc")?, "--nest-out", placed.to_str().ok_or("placements")?])?;
    let stderr = text(&output.stderr)?;
    assert_eq!(output.status.code(), Some(1), "{stderr}");
    assert!(stderr.contains("block"), "{stderr}");
    assert!(stderr.contains("does not fit"), "{stderr}");
    assert!(!ngc.exists(), "ngc was written");
    assert!(!placed.exists(), "placements were written");
    Ok(())
}

#[test]
fn nest_out_without_a_sheet_is_an_error() -> Result<(), String> {
    let dir = temp_dir("nest-flag")?;
    let job = r#"{
  "format": "campanion-job/1",
  "source": { "path": "missing.dxf", "hash": "cbf29ce484222325" },
  "units": "mm",
  "tools": [],
  "operations": []
}"#;
    let job_path = dir.join("job.json");
    std::fs::write(&job_path, job).map_err(|err| err.to_string())?;
    let ngc = dir.join("out.ngc");
    let placed = dir.join("placements.json");
    let output =
        campanion(&["run", job_path.to_str().ok_or("job")?, "-o", ngc.to_str().ok_or("ngc")?, "--nest-out", placed.to_str().ok_or("placements")?])?;
    let stderr = text(&output.stderr)?;
    assert_eq!(output.status.code(), Some(1), "{stderr}");
    assert!(stderr.contains("--nest-out needs a job with sheet and nest"), "{stderr}");
    assert!(!ngc.exists(), "ngc was written");
    assert!(!placed.exists(), "placements were written");
    Ok(())
}
