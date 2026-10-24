//! Every mill example, and the stored mill NC, stays above the rapid safety rule.

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

fn is_rapid_safety(stderr: &str) -> bool {
    stderr.contains("below the safe plane")
        || stderr.contains("below the cut surface")
        || stderr.contains("rapid ends")
        || stderr.contains("rapid traverse")
        || stderr.contains("rapid is below")
}

fn is_post_skip(stderr: &str) -> bool {
    // A post that has no axis, or that rejects rotary or tilted output, is not a safety failure.
    stderr.contains("has no") || stderr.contains("does not support") || stderr.contains("not supported") || stderr.contains("rejects")
}

fn mill_post_ids() -> Result<Vec<String>, String> {
    let output = campanion(&["posts"])?;
    if output.status.code() != Some(0) {
        return Err(format!("posts exited {:?}: {}", output.status.code(), text(&output.stderr).unwrap_or_default()));
    }
    let mut ids = Vec::new();
    for line in text(&output.stdout)?.lines() {
        if line.starts_with(' ') || line.is_empty() {
            continue;
        }
        let Some(id) = line.split_whitespace().next() else {
            continue;
        };
        let lower = id.to_ascii_lowercase();
        if lower.contains("lathe") || lower.contains("2ch") {
            continue;
        }
        ids.push(id.to_string());
    }
    if ids.is_empty() {
        return Err("no mill posts".to_string());
    }
    Ok(ids)
}

fn step_source(path: &Path) -> Result<bool, String> {
    let text = std::fs::read_to_string(path).map_err(|err| format!("{}: {err}", path.display()))?;
    let value: serde_json::Value = serde_json::from_str(&text).map_err(|err| format!("{}: {err}", path.display()))?;
    let source = value["source"]["path"].as_str().unwrap_or("");
    let lower = source.to_ascii_lowercase();
    Ok(lower.ends_with(".step") || lower.ends_with(".stp"))
}

fn example_jobs(root: &Path) -> Result<Vec<PathBuf>, String> {
    let mut jobs = Vec::new();
    let dir = root.join("examples");
    for entry in std::fs::read_dir(&dir).map_err(|err| format!("{}: {err}", dir.display()))? {
        let path = entry.map_err(|err| err.to_string())?.path();
        let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
            continue;
        };
        if !name.ends_with(".job.json") || step_source(&path)? {
            continue;
        }
        jobs.push(path);
    }
    jobs.sort();
    if jobs.is_empty() {
        return Err("no example jobs".to_string());
    }
    Ok(jobs)
}

fn run_job(job: &Path, post: Option<&str>) -> Result<(), String> {
    let stem = job.file_stem().and_then(|stem| stem.to_str()).unwrap_or("job");
    let label = post.unwrap_or("ngc").replace('/', "_");
    let out = temp_path(&format!("rapid-safety-{stem}-{label}.nc"));
    let job_arg = path_arg(job)?;
    let out_arg = path_arg(&out)?;
    let mut args = vec!["run", job_arg.as_str(), "-o", out_arg.as_str()];
    if let Some(post) = post {
        args.push("--post");
        args.push(post);
    }
    let output = campanion(&args)?;
    let stderr = text(&output.stderr).unwrap_or_default();
    if is_rapid_safety(&stderr) {
        return Err(format!("{} post {post:?}\n{stderr}", job.display()));
    }
    if output.status.code() == Some(0) {
        return Ok(());
    }
    if post.is_some() && is_post_skip(&stderr) {
        return Ok(());
    }
    let stdout = text(&output.stdout).unwrap_or_default();
    Err(format!("exit {:?} {} post {post:?}\nstdout {stdout}\nstderr {stderr}", output.status.code(), job.display()))
}

#[test]
fn examples_reject_no_low_rapid() -> Result<(), String> {
    let root = workspace_root()?;
    let posts = mill_post_ids()?;
    for job in example_jobs(&root)? {
        run_job(&job, None)?;
        for post in &posts {
            run_job(&job, Some(post))?;
        }
    }
    Ok(())
}

fn skipped_golden(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    !lower.ends_with(".nc") || lower.contains("lathe") || lower.contains("turn") || lower.contains("2ch") || lower.contains("millturn")
}

fn scan_dir(dir: &Path) -> Result<(), String> {
    let mut saw = false;
    for entry in std::fs::read_dir(dir).map_err(|err| format!("{}: {err}", dir.display()))? {
        let path = entry.map_err(|err| err.to_string())?.path();
        let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
            continue;
        };
        if skipped_golden(name) {
            continue;
        }
        saw = true;
        let text = std::fs::read_to_string(&path).map_err(|err| format!("{}: {err}", path.display()))?;
        scan(&text).map_err(|err| format!("{}: {err}", path.display()))?;
    }
    if !saw {
        return Err(format!("no mill nc in {}", dir.display()));
    }
    Ok(())
}

#[test]
fn golden_nc_rejects_no_low_rapid() -> Result<(), String> {
    let root = workspace_root()?;
    scan_dir(&root.join("crates/post/tests/golden"))?;
    scan_dir(&root.join("crates/cli/tests/golden"))?;
    Ok(())
}

#[test]
fn scan_rejects_the_old_drill_output() {
    let err = scan("G0 X15 Y30 Z-3\nG0 X85 Y30 Z-3\n").expect_err("low rapid");
    assert!(err.contains("Z -3.000"), "{err}");
}

#[test]
fn scan_accepts_an_r_plane_plunge() {
    let program = "G0 X15 Y30 Z5\nG0 X15 Y30 Z2\nG1 X15 Y30 Z-3 F100\nG0 X15 Y30 Z5\n";
    scan(program).expect("plunge");
}

#[test]
fn scan_rejects_g91_and_keeps_g91_1() {
    assert!(scan("G91\nG0 X1 Y1 Z1\n").is_err());
    scan("G91.1\nG0 X1 Y1 Z5\nG1 Z-1 F100\n").expect("g91.1 is not incremental");
}

/// One Cartesian frame. A toolpath comment, a tool change, a rotary word, or a tilted-plane
/// code starts a new frame and drops the carried X Y Z. Modal G0/G1/G2/G3 stays, because the
/// next toolpath often continues it. A 3+2 program rapids in X and Y before the first Z of the
/// new frame, so a Z carried from the previous frame would look like a plunge. A line with A,
/// B, or C is its own frame: a 4-axis move is not judged as a Cartesian plunge. A rapid is
/// checked only once X, Y, and Z are all known. The feed-start Z is the Z before the move, and
/// only when that Z is known.
struct Section {
    feed_starts: Vec<f64>,
    feed_xy: Vec<(f64, f64)>,
    rapids: Vec<(f64, f64, f64)>,
    saw_feed: bool,
    /// Lowest rapid Z in a section that never feeds. `None` when every rapid stays at or above 0.
    low_rapid: Option<f64>,
}

impl Section {
    fn new() -> Self {
        Self { feed_starts: Vec::new(), feed_xy: Vec::new(), rapids: Vec::new(), saw_feed: false, low_rapid: None }
    }
}

struct Pos {
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
}

fn scan(text: &str) -> Result<(), String> {
    let mut modal: Option<u8> = None;
    let mut pos = Pos { x: None, y: None, z: None };
    let mut in_feed = false;
    let mut section = Section::new();
    for (index, raw) in text.lines().enumerate() {
        let line_no = index + 1;
        let (words, toolpath) = tokenize(raw);
        if words.iter().any(|(letter, value)| *letter == 'G' && near(*value, 91.0)) {
            return Err(format!("line {line_no}: G91 incremental motion"));
        }
        let g53 = words.iter().any(|(letter, value)| *letter == 'G' && near(*value, 53.0));
        let boundary = toolpath || words.iter().any(|(letter, value)| is_boundary(*letter, *value));
        if boundary {
            finish_section(&section)?;
            section = Section::new();
            pos = Pos { x: None, y: None, z: None };
            in_feed = false;
        }
        if g53 {
            continue;
        }
        for (letter, value) in &words {
            if *letter == 'G'
                && let Some(group) = motion_group(*value)
            {
                modal = Some(group);
            }
        }
        let motion = words.iter().any(|(letter, _)| matches!(letter, 'X' | 'Y' | 'Z' | 'I' | 'J' | 'K'));
        let Some(group) = modal else {
            continue;
        };
        if !motion {
            continue;
        }
        let before_z = pos.z;
        for (letter, value) in &words {
            match letter {
                'X' => pos.x = Some(*value),
                'Y' => pos.y = Some(*value),
                'Z' => pos.z = Some(*value),
                _ => {}
            }
        }
        match group {
            0 => {
                in_feed = false;
                if let (Some(x), Some(y), Some(z)) = (pos.x, pos.y, pos.z) {
                    section.rapids.push((x, y, z));
                }
                if let Some(z) = pos.z.filter(|z| *z < 0.0) {
                    section.low_rapid = Some(section.low_rapid.map_or(z, |low| low.min(z)));
                }
            }
            1..=3 => {
                section.saw_feed = true;
                if !in_feed {
                    if let Some(z) = before_z {
                        section.feed_starts.push(z);
                    }
                    in_feed = true;
                }
                if let (Some(x), Some(y)) = (pos.x, pos.y) {
                    section.feed_xy.push((x, y));
                }
            }
            _ => {}
        }
    }
    finish_section(&section)
}

fn finish_section(section: &Section) -> Result<(), String> {
    if !section.saw_feed {
        if let Some(z) = section.low_rapid {
            return Err(format!("rapid ends at Z {z:.3} below Z 0 in a section with no feed"));
        }
        return Ok(());
    }
    let Some(plane) = section.feed_starts.iter().copied().reduce(f64::min) else {
        return Ok(());
    };
    let Some((min_x, min_y, max_x, max_y)) = feed_bbox(&section.feed_xy) else {
        return Ok(());
    };
    for (x, y, z) in &section.rapids {
        let inside = *x >= min_x - 1e-6 && *x <= max_x + 1e-6 && *y >= min_y - 1e-6 && *y <= max_y + 1e-6;
        if inside && *z < plane - 1e-6 {
            return Err(format!("rapid ends at X {x:.3} Y {y:.3} Z {z:.3} below the section R plane {plane:.3}"));
        }
    }
    Ok(())
}

fn feed_bbox(points: &[(f64, f64)]) -> Option<(f64, f64, f64, f64)> {
    let mut points = points.iter();
    let (mut min_x, mut min_y) = *points.next()?;
    let (mut max_x, mut max_y) = (min_x, min_y);
    for (x, y) in points {
        min_x = min_x.min(*x);
        min_y = min_y.min(*y);
        max_x = max_x.max(*x);
        max_y = max_y.max(*y);
    }
    Some((min_x, min_y, max_x, max_y))
}

fn is_boundary(letter: char, value: f64) -> bool {
    match letter {
        'A' | 'B' | 'C' => true,
        'M' => near(value, 6.0),
        'G' => near(value, 254.0) || near(value, 255.0) || near(value, 68.2) || near(value, 69.0),
        _ => false,
    }
}

fn motion_group(value: f64) -> Option<u8> {
    [0.0, 1.0, 2.0, 3.0].into_iter().find(|group| near(value, *group)).map(|group| group as u8)
}

fn near(value: f64, target: f64) -> bool {
    (value - target).abs() <= 1e-6
}

fn tokenize(line: &str) -> (Vec<(char, f64)>, bool) {
    let bytes = line.as_bytes();
    let mut index = 0;
    while index < bytes.len() && bytes[index].is_ascii_whitespace() {
        index += 1;
    }
    if index < bytes.len() && bytes[index] == b'/' {
        return (Vec::new(), false);
    }
    let mut words = Vec::new();
    let mut toolpath = false;
    while index < bytes.len() {
        let byte = bytes[index];
        if byte.is_ascii_whitespace() || byte == b'%' {
            index += 1;
            continue;
        }
        if byte == b';' {
            break;
        }
        if byte == b'(' {
            index += 1;
            let start = index;
            while index < bytes.len() && bytes[index] != b')' {
                index += 1;
            }
            let comment = std::str::from_utf8(&bytes[start..index]).unwrap_or("");
            if comment.trim().to_ascii_lowercase().starts_with("toolpath") {
                toolpath = true;
            }
            if index < bytes.len() && bytes[index] == b')' {
                index += 1;
            }
            continue;
        }
        if byte.is_ascii_alphabetic() {
            let letter = (byte as char).to_ascii_uppercase();
            index += 1;
            while index < bytes.len() && bytes[index].is_ascii_whitespace() {
                index += 1;
            }
            let Some((len, value)) = take_number(&line[index..]) else {
                continue;
            };
            index += len;
            if letter != 'N' {
                words.push((letter, value));
            }
            continue;
        }
        index += 1;
    }
    (words, toolpath)
}

fn take_number(text: &str) -> Option<(usize, f64)> {
    let bytes = text.as_bytes();
    let mut index = 0;
    if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
        index += 1;
    }
    let mut digits = false;
    while index < bytes.len() && bytes[index].is_ascii_digit() {
        digits = true;
        index += 1;
    }
    if index < bytes.len() && bytes[index] == b'.' {
        index += 1;
        while index < bytes.len() && bytes[index].is_ascii_digit() {
            digits = true;
            index += 1;
        }
    }
    if !digits {
        return None;
    }
    let value = text[..index].parse::<f64>().ok()?;
    Some((index, value))
}
