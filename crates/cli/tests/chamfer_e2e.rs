//! Chamfer passes on `examples/chamfer.job.json`.
//!
//! The integration-test crate has no serde dependency, so IR JSON is parsed here. `verify`
//! then has to report an empty issue list. A 90° V-bit with no tip clearance puts the tip at
//! minus the chamfer width.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// Outside chamfer of the `OUTER` outline, width 0.5 mm.
const OUTSIDE_Z: f64 = -0.5;
/// Inside chamfer of the `POCKET` hole, width 0.3 mm.
const INSIDE_Z: f64 = -0.3;
const Z_EPS: f64 = 1.0e-6;

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

/// The outside pass bottoms at −0.5 mm and the inside pass at −0.3 mm. `verify` is clean.
#[test]
fn chamfer_job_cuts_outline_and_holes_at_the_tip() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-chamfer-e2e.ngc");
    let ir_path = temp_path("campanion-chamfer-e2e.json");
    let job = "examples/chamfer.job.json";
    run_job(&root, job, &ngc_path, &ir_path)?;
    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir = JsonParser::parse(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir.object_get("program")?.object_get("toolpaths")?.as_array()?;
    if toolpaths.is_empty() {
        return Err("chamfer job produced no toolpaths".into());
    }
    let mut failures = Vec::new();
    match named_min_z(toolpaths, "outer") {
        Ok(z) if near(z, OUTSIDE_Z, Z_EPS) => {}
        Ok(z) => failures.push(format!("outside deepest Z is {z}, expected {OUTSIDE_Z}")),
        Err(err) => failures.push(err),
    }
    match named_min_z(toolpaths, "holes") {
        Ok(z) if near(z, INSIDE_Z, Z_EPS) => {}
        Ok(z) => failures.push(format!("inside deepest Z is {z}, expected {INSIDE_Z}")),
        Err(err) => failures.push(err),
    }
    if let Err(err) = assert_verify_clean(&root, job, &ir_path) {
        failures.push(err);
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

fn named_min_z(toolpaths: &[Json], name: &str) -> Result<f64, String> {
    let mut min_z = f64::INFINITY;
    let mut found = false;
    for path in toolpaths {
        if path.object_get("name")?.as_str()? != name {
            continue;
        }
        found = true;
        min_z = min_z.min(toolpath_min_z(path)?);
    }
    if found && min_z.is_finite() { Ok(min_z) } else { Err(format!("no toolpath named {name}")) }
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
