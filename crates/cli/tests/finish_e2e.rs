//! Finish passes on `examples/finish.job.json`, and the unchanged curves job.
//!
//! The integration-test crate has no serde dependency, so IR JSON is parsed here. `verify`
//! then has to report an empty issue list.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const DEPTH_MM: f64 = 3.0;
const FLOOR_MM: f64 = 0.3;
const WALL_MM: f64 = 0.5;
/// Offset of an axis-aligned rectangle should land on these coordinates.
const BBOX_EPS: f64 = 1.0e-6;
const Z_EPS: f64 = 1.0e-6;

/// Inserted into both operations of `finish.job.json`. Removing it recovers `curves.job.json`.
const STOCK_AND_FINISH: &str = r#",
      "stock_to_leave": {
        "wall": 0.5,
        "floor": 0.3
      },
      "finish": {
        "step_down": 3.0
      }"#;

const RECT_SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" width="80mm" height="40mm" viewBox="0 0 80 40">
  <g id="PART">
    <path d="M 10 10 H 50 V 30 H 10 Z"/>
  </g>
</svg>
"#;

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

fn near(actual: f64, expected: f64, tol: f64) -> bool {
    (actual - expected).abs() <= tol
}

fn run_job(cwd: &Path, job: &str, ngc_path: &Path, ir_path: &Path) -> Result<(), String> {
    let ngc_arg = path_arg(ngc_path)?;
    let ir_arg = path_arg(ir_path)?;
    let output = campanion(cwd, &["run", job, "-o", &ngc_arg, "--ir", &ir_arg])?;
    let stderr = output_text(&output.stderr)?;
    let stdout = output_text(&output.stdout)?;
    if output.status.code() != Some(0) {
        return Err(format!("{job}: run exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    if stderr.contains("changed") {
        return Err(format!("{job}: source hash did not match: {stderr}"));
    }
    Ok(())
}

fn assert_verify_clean(root: &Path, job: &str, ir_path: &Path) -> Result<(), String> {
    let ir_arg = path_arg(ir_path)?;
    let output = campanion(root, &["verify", &ir_arg])?;
    let stdout = output_text(&output.stdout)?;
    let stderr = output_text(&output.stderr)?;
    if output.status.code() != Some(0) {
        return Err(format!("{job}: verify exit {:?} stdout {stdout} stderr {stderr}", output.status.code()));
    }
    let status_ok = stdout.lines().any(|line| line == "status ok");
    let no_issues = stdout.lines().any(|line| line == "issues 0");
    if status_ok && no_issues { Ok(()) } else { Err(format!("{job}: verify reported errors:\n{stdout}{stderr}")) }
}

/// Roughing stops `floor` short of the final depth. The finish pass, including the pocket floor, reaches it.
#[test]
fn finish_job_roughs_above_the_floor_then_cuts_to_depth() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-finish-e2e.ngc");
    let ir_path = temp_path("campanion-finish-e2e.json");
    run_job(&root, "examples/finish.job.json", &ngc_path, &ir_path)?;
    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir = JsonParser::parse(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir.object_get("program")?.object_get("toolpaths")?.as_array()?;
    if toolpaths.is_empty() {
        return Err("finish job produced no toolpaths".into());
    }
    let full = -DEPTH_MM;
    let rough_floor = -(DEPTH_MM - FLOOR_MM);
    let mut overall = f64::INFINITY;
    let mut rough = f64::INFINITY;
    let mut saw_rough = false;
    for path in toolpaths {
        let z = toolpath_min_z(path)?;
        overall = overall.min(z);
        if z > full + Z_EPS {
            saw_rough = true;
            rough = rough.min(z);
        }
    }
    let mut failures = Vec::new();
    if !near(overall, full, Z_EPS) {
        failures.push(format!("deepest Z is {overall}, expected {full}"));
    }
    if !saw_rough || !near(rough, rough_floor, Z_EPS) {
        failures.push(format!("deepest roughing Z is {rough}, expected {rough_floor}"));
    }
    if let Err(err) = assert_verify_clean(&root, "examples/finish.job.json", &ir_path) {
        failures.push(err);
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

/// A job that does not set the new keys matches `curves.job.json`, which is that job.
#[test]
fn job_without_stock_or_finish_matches_curves() -> Result<(), String> {
    let root = workspace_root()?;
    let curves = include_str!("../../../examples/curves.job.json");
    let finish = include_str!("../../../examples/finish.job.json");
    let plain = finish.replace(STOCK_AND_FINISH, "");
    if plain != curves {
        return Err("examples/finish.job.json without stock_to_leave and finish does not match examples/curves.job.json".into());
    }
    if plain.contains("stock_to_leave") || plain.contains("\"finish\"") {
        return Err("curves job still names stock_to_leave or finish".into());
    }

    let baseline_ngc = temp_path("campanion-finish-e2e-curves.ngc");
    let baseline_ir = temp_path("campanion-finish-e2e-curves.json");
    run_job(&root, "examples/curves.job.json", &baseline_ngc, &baseline_ir)?;

    let dir = temp_path("campanion-finish-plain");
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    std::fs::write(dir.join("curves.job.json"), plain.as_bytes()).map_err(|err| err.to_string())?;
    std::fs::copy(root.join("examples/curves.svg"), dir.join("curves.svg")).map_err(|err| err.to_string())?;
    let copy_ngc = temp_path("campanion-finish-e2e-plain.ngc");
    let copy_ir = temp_path("campanion-finish-e2e-plain.json");
    run_job(&dir, "curves.job.json", &copy_ngc, &copy_ir)?;

    assert_same_text("ngc", &baseline_ngc, &copy_ngc)?;
    assert_same_text("ir", &baseline_ir, &copy_ir)?;
    Ok(())
}

/// Roughing an outside profile with 0.5 mm of wall stock sits 0.5 mm further out than the finish pass.
#[test]
fn profile_rough_pass_stands_off_the_wall() -> Result<(), String> {
    let dir = temp_path("campanion-finish-rect");
    std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
    let svg = RECT_SVG.as_bytes();
    std::fs::write(dir.join("rect.svg"), svg).map_err(|err| err.to_string())?;
    let job = rect_job(&fnv1a64(svg));
    std::fs::write(dir.join("rect.job.json"), job.as_bytes()).map_err(|err| err.to_string())?;

    let ngc_path = temp_path("campanion-finish-e2e-rect.ngc");
    let ir_path = temp_path("campanion-finish-e2e-rect.json");
    run_job(&dir, "rect.job.json", &ngc_path, &ir_path)?;
    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir = JsonParser::parse(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir.object_get("program")?.object_get("toolpaths")?.as_array()?;
    if toolpaths.len() != 2 {
        return Err(format!("rectangle profile produced {} toolpaths, expected rough then finish", toolpaths.len()));
    }
    let rough = toolpaths.first().ok_or_else(|| "missing rough toolpath".to_string())?;
    let finish = toolpaths.get(1).ok_or_else(|| "missing finish toolpath".to_string())?;
    let rough_box = cut_bbox(rough)?;
    let finish_box = cut_bbox(finish)?;
    let width_delta = rough_box.width() - finish_box.width();
    let height_delta = rough_box.height() - finish_box.height();
    let expect = 2.0 * WALL_MM;
    if !near(width_delta, expect, BBOX_EPS) || !near(height_delta, expect, BBOX_EPS) {
        return Err(format!(
            "rough bbox {rough_box:?} finish bbox {finish_box:?}: width delta {width_delta}, height delta {height_delta}, expected {expect}"
        ));
    }
    Ok(())
}

fn rect_job(hash: &str) -> String {
    format!(
        r#"{{
  "format": "campanion-job/1",
  "source": {{
    "path": "rect.svg",
    "hash": "{hash}"
  }},
  "units": "mm",
  "tools": [
    {{
      "id": 1,
      "diameter": 6.0
    }}
  ],
  "operations": [
    {{
      "name": "outer",
      "type": "profile",
      "layers": ["PART"],
      "side": "outside",
      "tool_id": 1,
      "depth": 1.0,
      "step_down": 1.0,
      "feeds": {{
        "cut": 1000.0,
        "plunge": 300.0
      }},
      "stock_to_leave": {{
        "wall": 0.5
      }},
      "finish": {{
        "step_down": 1.0
      }}
    }}
  ],
  "tolerance": {{
    "linear": 0.001,
    "angular": 0.001,
    "chord": 0.01
  }}
}}
"#
    )
}

/// FNV-1a 64, 16 lowercase hex digits. Matches `jobfile::hash_bytes`.
fn fnv1a64(bytes: &[u8]) -> String {
    let mut hash = 0xcbf29ce484222325u64;
    for byte in bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{hash:016x}")
}

fn assert_same_text(label: &str, left_path: &Path, right_path: &Path) -> Result<(), String> {
    let left = std::fs::read_to_string(left_path).map_err(|err| format!("read {label}: {err}"))?;
    let right = std::fs::read_to_string(right_path).map_err(|err| format!("read {label}: {err}"))?;
    if left == right {
        return Ok(());
    }
    let left_lines: Vec<&str> = left.lines().collect();
    let right_lines: Vec<&str> = right.lines().collect();
    let shared = left_lines.len().min(right_lines.len());
    for index in 0..shared {
        if left_lines[index] != right_lines[index] {
            return Err(format!("{label} differs at line {}: {:?} vs {:?}", index + 1, left_lines[index], right_lines[index]));
        }
    }
    Err(format!("{label} differs in length: {} vs {}", left_lines.len(), right_lines.len()))
}

fn toolpath_min_z(toolpath: &Json) -> Result<f64, String> {
    let blocks = toolpath.object_get("blocks")?.as_array()?;
    let mut min_z = f64::INFINITY;
    for block in blocks {
        let Some(z) = motion_tip(block)?.map(|tip| tip.2) else {
            continue;
        };
        min_z = min_z.min(z);
    }
    if min_z.is_finite() { Ok(min_z) } else { Err("toolpath has no motion".into()) }
}

#[derive(Clone, Copy, Debug)]
struct Bbox {
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
}

impl Bbox {
    fn width(self) -> f64 {
        self.max_x - self.min_x
    }

    fn height(self) -> f64 {
        self.max_y - self.min_y
    }
}

/// XY extent of cutting moves. Rapids include the link from the origin, so they are left out.
fn cut_bbox(toolpath: &Json) -> Result<Bbox, String> {
    let blocks = toolpath.object_get("blocks")?.as_array()?;
    let mut bbox: Option<Bbox> = None;
    for block in blocks {
        let kind = block.object_get("type")?.as_str()?;
        if !matches!(kind, "linear" | "arc" | "helix") {
            continue;
        }
        if block.object_get("role")?.as_str()? != "cut" {
            continue;
        }
        let Some((x, y, _)) = motion_tip(block)? else {
            continue;
        };
        bbox = Some(match bbox {
            None => Bbox { min_x: x, max_x: x, min_y: y, max_y: y },
            Some(mut next) => {
                next.min_x = next.min_x.min(x);
                next.max_x = next.max_x.max(x);
                next.min_y = next.min_y.min(y);
                next.max_y = next.max_y.max(y);
                next
            }
        });
    }
    bbox.ok_or_else(|| "toolpath has no cut moves".to_string())
}

fn motion_tip(block: &Json) -> Result<Option<(f64, f64, f64)>, String> {
    let kind = block.object_get("type")?.as_str()?;
    if !matches!(kind, "rapid" | "linear" | "arc" | "helix") {
        return Ok(None);
    }
    let tip = block.object_get("to")?.object_get("tip")?;
    let x = tip.object_get("x")?.as_f64()?;
    let y = tip.object_get("y")?.as_f64()?;
    let z = tip.object_get("z")?.as_f64()?;
    Ok(Some((x, y, z)))
}

enum Json {
    Null,
    Bool,
    Number(f64),
    String(String),
    Array(Vec<Json>),
    Object(Vec<(String, Json)>),
}

impl Json {
    fn object_get(&self, key: &str) -> Result<&Json, String> {
        match self {
            Self::Object(fields) => fields.iter().find(|(name, _)| name == key).map(|(_, value)| value).ok_or_else(|| format!("missing `{key}`")),
            _ => Err(format!("`{key}` is not inside an object")),
        }
    }

    fn as_str(&self) -> Result<&str, String> {
        match self {
            Self::String(text) => Ok(text),
            _ => Err("expected a string".into()),
        }
    }

    fn as_f64(&self) -> Result<f64, String> {
        match self {
            Self::Number(value) => Ok(*value),
            _ => Err("expected a number".into()),
        }
    }

    fn as_array(&self) -> Result<&[Json], String> {
        match self {
            Self::Array(items) => Ok(items),
            _ => Err("expected an array".into()),
        }
    }
}

struct JsonParser<'a> {
    bytes: &'a [u8],
    index: usize,
}

impl<'a> JsonParser<'a> {
    fn parse(text: &'a str) -> Result<Json, String> {
        let mut parser = Self { bytes: text.as_bytes(), index: 0 };
        let value = parser.value()?;
        parser.skip();
        if parser.index != parser.bytes.len() {
            return Err("trailing data".into());
        }
        Ok(value)
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.index).copied()
    }

    fn bump(&mut self) -> Result<u8, String> {
        let byte = self.peek().ok_or_else(|| "unexpected end".to_string())?;
        self.index += 1;
        Ok(byte)
    }

    fn skip(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.index += 1;
        }
    }

    fn expect(&mut self, want: u8) -> Result<(), String> {
        let got = self.bump()?;
        if got == want { Ok(()) } else { Err(format!("expected byte {want}, found {got}")) }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.skip();
        match self.peek() {
            Some(b'n') => self.literal(b"null", Json::Null),
            Some(b't') => self.literal(b"true", Json::Bool),
            Some(b'f') => self.literal(b"false", Json::Bool),
            Some(b'"') => Ok(Json::String(self.string()?)),
            Some(b'[') => self.array(),
            Some(b'{') => self.object(),
            Some(b'-' | b'0'..=b'9') => self.number(),
            Some(other) => Err(format!("unexpected byte {other}")),
            None => Err("unexpected end".into()),
        }
    }

    fn literal(&mut self, text: &[u8], value: Json) -> Result<Json, String> {
        for want in text {
            let got = self.bump()?;
            if got != *want {
                return Err(format!("expected `{want}`, found {got}"));
            }
        }
        Ok(value)
    }

    fn array(&mut self) -> Result<Json, String> {
        self.expect(b'[')?;
        self.skip();
        let mut items = Vec::new();
        if self.peek() == Some(b']') {
            self.index += 1;
            return Ok(Json::Array(items));
        }
        loop {
            items.push(self.value()?);
            self.skip();
            match self.peek() {
                Some(b',') => self.index += 1,
                Some(b']') => {
                    self.index += 1;
                    break;
                }
                _ => return Err("array is not closed".into()),
            }
        }
        Ok(Json::Array(items))
    }

    fn object(&mut self) -> Result<Json, String> {
        self.expect(b'{')?;
        self.skip();
        let mut fields = Vec::new();
        if self.peek() == Some(b'}') {
            self.index += 1;
            return Ok(Json::Object(fields));
        }
        loop {
            self.skip();
            if self.peek() != Some(b'"') {
                return Err("object key must be a string".into());
            }
            let key = self.string()?;
            self.skip();
            self.expect(b':')?;
            fields.push((key, self.value()?));
            self.skip();
            match self.peek() {
                Some(b',') => self.index += 1,
                Some(b'}') => {
                    self.index += 1;
                    break;
                }
                _ => return Err("object is not closed".into()),
            }
        }
        Ok(Json::Object(fields))
    }

    fn string(&mut self) -> Result<String, String> {
        self.expect(b'"')?;
        let mut out = Vec::new();
        loop {
            match self.bump()? {
                b'"' => break,
                b'\\' => self.escape(&mut out)?,
                byte if byte < 0x20 => return Err("control character in string".into()),
                byte => out.push(byte),
            }
        }
        String::from_utf8(out).map_err(|_| "string is not utf-8".into())
    }

    fn escape(&mut self, out: &mut Vec<u8>) -> Result<(), String> {
        match self.bump()? {
            b'"' => out.push(b'"'),
            b'\\' => out.push(b'\\'),
            b'/' => out.push(b'/'),
            b'b' => out.push(0x08),
            b'f' => out.push(0x0c),
            b'n' => out.push(b'\n'),
            b'r' => out.push(b'\r'),
            b't' => out.push(b'\t'),
            b'u' => {
                let code = self.hex4()?;
                let ch = char::from_u32(code).ok_or_else(|| format!("bad unicode scalar {code}"))?;
                let mut buf = [0u8; 4];
                out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
            }
            other => return Err(format!("bad escape {other}")),
        }
        Ok(())
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let mut code = 0u32;
        for _ in 0..4 {
            let byte = self.bump()?;
            let digit = match byte {
                b'0'..=b'9' => u32::from(byte - b'0'),
                b'a'..=b'f' => u32::from(byte - b'a' + 10),
                b'A'..=b'F' => u32::from(byte - b'A' + 10),
                _ => return Err("bad unicode escape".into()),
            };
            code = code.saturating_mul(16).saturating_add(digit);
        }
        Ok(code)
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.index;
        if self.peek() == Some(b'-') {
            self.index += 1;
        }
        self.digits()?;
        if self.peek() == Some(b'.') {
            self.index += 1;
            self.digits()?;
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.index += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.index += 1;
            }
            self.digits()?;
        }
        let bytes = self.bytes.get(start..self.index).ok_or_else(|| "number is out of range".to_string())?;
        let text = std::str::from_utf8(bytes).map_err(|_| "number is not utf-8".to_string())?;
        let value: f64 = text.parse().map_err(|_| format!("bad number `{text}`"))?;
        if value.is_finite() { Ok(Json::Number(value)) } else { Err(format!("non-finite number `{text}`")) }
    }

    fn digits(&mut self) -> Result<(), String> {
        let start = self.index;
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.index += 1;
        }
        if self.index == start { Err("number needs a digit".into()) } else { Ok(()) }
    }
}
