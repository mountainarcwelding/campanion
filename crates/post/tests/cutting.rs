//! Torch program with a pierce dwell and power, posted by the plasma and laser dialects.

use std::path::PathBuf;

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Document, Event, Feeds, MoveRole, Plane, Pose, ProcessKind, ProcessParams, Program, Tool,
    Vec3,
};
use campanion_post::{PostOptions, builtin, compare, post_program, read_back};

fn manifest_path(parts: &[&str]) -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for part in parts {
        path.push(part);
    }
    path
}

fn pose(x: f64, y: f64, z: f64) -> Result<Pose, String> {
    let tip = Vec3::new(x, y, z).map_err(|err| err.to_string())?;
    Pose::new(tip, Vec3::Z).map_err(|err| err.to_string())
}

fn at(x: f64, y: f64, z: f64) -> Result<Vec3, String> {
    Vec3::new(x, y, z).map_err(|err| err.to_string())
}

/// Plate cut: pierce at Z3, dwell 0.5 s with the torch on, cut at Z1.2, then a post dwell.
fn plasma_document() -> Result<Document, String> {
    let params = ProcessParams::new()
        .power(Some(0.8))
        .map_err(|err| err.to_string())?
        .pierce_delay(0.5, true)
        .map_err(|err| err.to_string())?
        .post_delay(0.25, false)
        .map_err(|err| err.to_string())?
        .pierce_height(Some(3.0))
        .map_err(|err| err.to_string())?
        .cut_height(Some(1.2))
        .map_err(|err| err.to_string())?;
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 10.0, 8.0)?, feed: None }),
        ClBlock::Motion(ClMotion::Rapid { to: pose(10.0, 10.0, 3.0)?, feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params }),
        ClBlock::Event(Event::Dwell { seconds: 0.5 }),
        ClBlock::Motion(ClMotion::Linear { to: pose(10.0, 10.0, 1.2)?, feed: 300.0, role: MoveRole::Plunge }),
        ClBlock::Motion(ClMotion::Linear { to: pose(40.0, 10.0, 1.2)?, feed: 1200.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(50.0, 20.0, 1.2)?,
            plane: Plane::Xy,
            center: at(40.0, 20.0, 1.2)?,
            sense: ArcSense::Ccw,
            feed: 1200.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Torch }),
        ClBlock::Event(Event::Dwell { seconds: 0.25 }),
        ClBlock::Motion(ClMotion::Rapid { to: pose(50.0, 20.0, 8.0)?, feed: None }),
    ];
    let tool = Tool::try_new(1, Some("torch".into()), None, Some(1.2)).map_err(|err| err.to_string())?;
    let feeds = Feeds::try_new(1200.0, None, None, Some(300.0), None).map_err(|err| err.to_string())?;
    let path = ClPath::try_new(Some("plate".into()), Some(1), CompensationMode::Off, feeds, Some(at(10.0, 10.0, 3.0)?), Vec::new(), blocks)
        .map_err(|err| err.to_string())?;
    let program = Program::try_new(Some("plasma".into()), vec![tool], None, vec![path], Vec::new(), Vec::new()).map_err(|err| err.to_string())?;
    Document::new(program).map_err(|err| err.to_string())
}

fn without_comments(line: &str) -> String {
    let mut out = String::new();
    let mut chars = line.chars();
    while let Some(ch) = chars.next() {
        if ch == '(' {
            for inner in chars.by_ref() {
                if inner == ')' {
                    break;
                }
            }
            continue;
        }
        if ch == ';' {
            break;
        }
        out.push(ch);
    }
    out
}

#[test]
fn plasma_fixture_matches_builder_and_posts() -> Result<(), String> {
    let built = plasma_document()?;
    let path = manifest_path(&["tests", "data", "plasma.ir.json"]);
    if std::env::var("UPDATE_PLASMA_IR").ok().as_deref() == Some("1") {
        let mut json = built.to_json_pretty().expect("json");
        if !json.ends_with('\n') {
            json.push('\n');
        }
        std::fs::write(&path, json).expect("write");
    }
    let text = std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("{}: {err}", path.display()));
    let loaded = Document::from_json(&text).unwrap_or_else(|err| panic!("plasma ir: {err}"));
    assert_eq!(loaded.program(), built.program());
    let mut torch = false;
    for toolpath in loaded.program().toolpaths() {
        for block in toolpath.blocks() {
            let ClBlock::Event(Event::ProcessOn { process, params }) = block else { continue };
            assert!(matches!(process, ProcessKind::Torch));
            assert_eq!(params.power_value(), Some(0.8));
            assert_eq!(params.pierce_delay_s(), 0.5);
            assert_eq!(params.post_delay_s(), 0.25);
            torch = true;
        }
    }
    assert!(torch);

    let options = PostOptions { arc_tolerance: 0.01 };
    let program = loaded.program();
    let plasma = builtin("plasma").expect("plasma");
    let posted = post_program(program, &plasma, &options).expect("plasma");
    assert!(posted.contains("M3 S1"), "{posted}");
    assert!(posted.contains("G4 P0.500"), "{posted}");
    assert!(posted.contains("G4 P0.250"), "{posted}");
    assert!(posted.contains("G64 P0.05"), "{posted}");
    assert!(!posted.contains("S800") && !posted.contains("S0.8"), "{posted}");
    compare(program, &read_back(&posted, &plasma).expect("read"), 1e-3, &plasma).expect("compare");

    let laser = builtin("grbl_laser").expect("grbl_laser");
    let laser_text = post_program(program, &laser, &options).expect("laser");
    assert!(laser_text.contains("M4 S800"), "{laser_text}");
    assert!(!laser_text.contains("power missing"), "{laser_text}");
    for line in laser_text.lines() {
        for word in without_comments(line).split_whitespace() {
            assert!(!word.starts_with('Z'), "unexpected Z word in {line}");
        }
    }
    compare(program, &read_back(&laser_text, &laser).expect("read"), 1e-3, &laser).expect("compare");
    Ok(())
}
