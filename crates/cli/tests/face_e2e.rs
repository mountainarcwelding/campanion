//! Facing pass on `examples/face.job.json`.
//!
//! The integration-test crate has no serde dependency, so IR JSON is parsed here. `verify`
//! then has to report an empty issue list.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const DEPTH_MM: f64 = 0.5;
const ROUGH_MM: f64 = 0.25;
const MARGIN_MM: f64 = 2.0;
/// Half of the 20 mm face tool. A point on the faced rectangle is within this of a cut.
const TOOL_RADIUS_MM: f64 = 10.0;
const GRID_MM: f64 = 1.0;
const Z_EPS: f64 = 1.0e-6;
/// `curves.svg` page height. Import maps user `y` to `height - y`.
const PAGE_HEIGHT_MM: f64 = 80.0;

/// `OUTER` path in `examples/curves.svg`, four cubics, user space before the Y flip.
const OUTER_CUBICS: [[(f64, f64); 4]; 4] = [
    [(60.0, 14.0), (85.41, 14.0), (106.0, 25.64), (106.0, 40.0)],
    [(106.0, 40.0), (106.0, 82.0), (76.0, 66.0), (60.0, 66.0)],
    [(60.0, 66.0), (34.59, 66.0), (14.0, 54.36), (14.0, 40.0)],
    [(14.0, 40.0), (14.0, 25.64), (34.59, 14.0), (60.0, 14.0)],
];

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

/// Two passes. The final level covers the outline box grown by the margin, and plunges stay outside it.
#[test]
fn face_job_covers_the_outline_and_plunges_outside() -> Result<(), String> {
    let root = workspace_root()?;
    let ngc_path = temp_path("campanion-face-e2e.ngc");
    let ir_path = temp_path("campanion-face-e2e.json");
    let job = "examples/face.job.json";
    run_job(&root, job, &ngc_path, &ir_path)?;
    let ir_text = std::fs::read_to_string(&ir_path).map_err(|err| format!("read ir: {err}"))?;
    let ir = JsonParser::parse(&ir_text).map_err(|err| format!("ir json: {err}"))?;
    let toolpaths = ir.object_get("program")?.object_get("toolpaths")?.as_array()?;
    if toolpaths.is_empty() {
        return Err("face job produced no toolpaths".into());
    }
    let survey = survey(toolpaths)?;
    let faced = faced_box();
    let mut failures = Vec::new();
    if !near(survey.min_z, -DEPTH_MM, Z_EPS) {
        failures.push(format!("deepest Z is {}, expected {}", survey.min_z, -DEPTH_MM));
    }
    if !survey.saw_rough {
        failures.push(format!("no cutting move at the first pass Z {}", -ROUGH_MM));
    }
    if survey.cuts.is_empty() {
        failures.push("no cutting moves at the final level".into());
    }
    if let Some(point) = uncovered(&faced, &survey.cuts) {
        failures.push(format!("({}, {}) on the faced box is more than {TOOL_RADIUS_MM} mm from a final cut", point.0, point.1));
    }
    for (x, y, z) in &survey.plunges {
        if !outside(&faced, *x, *y) {
            failures.push(format!("plunge at ({x}, {y}, {z}) is not outside the faced box"));
        }
    }
    if survey.plunges.is_empty() {
        failures.push("face produced no plunges".into());
    }
    if let Err(err) = assert_verify_clean(&root, job, &ir_path) {
        failures.push(err);
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

struct Survey {
    min_z: f64,
    saw_rough: bool,
    cuts: Vec<Seg>,
    plunges: Vec<(f64, f64, f64)>,
}

struct Seg {
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
}

fn survey(toolpaths: &[Json]) -> Result<Survey, String> {
    let mut min_z = f64::INFINITY;
    let mut saw_rough = false;
    let mut cuts = Vec::new();
    let mut plunges = Vec::new();
    for path in toolpaths {
        let blocks = path.object_get("blocks")?.as_array()?;
        let mut cursor: Option<(f64, f64, f64)> = None;
        for block in blocks {
            let Some(tip) = motion_tip(block)? else {
                continue;
            };
            min_z = min_z.min(tip.2);
            let kind = block.object_get("type")?.as_str()?;
            if matches!(kind, "linear" | "arc" | "helix") {
                let role = block.object_get("role")?.as_str()?;
                if role == "plunge" {
                    plunges.push(tip);
                }
                if role == "cut" && near(tip.2, -ROUGH_MM, Z_EPS) {
                    saw_rough = true;
                }
                if role == "cut" && near(tip.2, -DEPTH_MM, Z_EPS) {
                    let Some(start) = cursor else {
                        return Err("final cut has no start".into());
                    };
                    cuts.push(Seg { x0: start.0, y0: start.1, x1: tip.0, y1: tip.1 });
                }
            }
            cursor = Some(tip);
        }
    }
    if min_z.is_finite() { Ok(Survey { min_z, saw_rough, cuts, plunges }) } else { Err("toolpath has no motion".into()) }
}

struct Box2 {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

/// Outline bounding box in page millimetres, grown by `MARGIN_MM` on every side.
fn faced_box() -> Box2 {
    let (min_x, min_y, max_x, max_y) = outer_user_bbox();
    Box2 {
        min_x: min_x - MARGIN_MM,
        max_x: max_x + MARGIN_MM,
        min_y: (PAGE_HEIGHT_MM - max_y) - MARGIN_MM,
        max_y: (PAGE_HEIGHT_MM - min_y) + MARGIN_MM,
    }
}

fn outer_user_bbox() -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for cubic in OUTER_CUBICS {
        let (x0, y0, x1, y1) = cubic_bounds(cubic);
        min_x = min_x.min(x0);
        min_y = min_y.min(y0);
        max_x = max_x.max(x1);
        max_y = max_y.max(y1);
    }
    (min_x, min_y, max_x, max_y)
}

fn cubic_bounds(points: [(f64, f64); 4]) -> (f64, f64, f64, f64) {
    let xs = [points[0].0, points[1].0, points[2].0, points[3].0];
    let ys = [points[0].1, points[1].1, points[2].1, points[3].1];
    let (min_x, max_x) = axis_bounds(xs);
    let (min_y, max_y) = axis_bounds(ys);
    (min_x, min_y, max_x, max_y)
}

fn axis_bounds(p: [f64; 4]) -> (f64, f64) {
    let mut min_v = p[0].min(p[3]);
    let mut max_v = p[0].max(p[3]);
    for t in cubic_extrema(p) {
        let value = cubic_at(p, t);
        min_v = min_v.min(value);
        max_v = max_v.max(value);
    }
    (min_v, max_v)
}

/// Roots of the cubic derivative in `(0, 1)`.
fn cubic_extrema(p: [f64; 4]) -> Vec<f64> {
    let a = p[1] - p[0];
    let b = p[2] - p[1];
    let c = p[3] - p[2];
    let quad_a = a - 2.0 * b + c;
    let quad_b = -2.0 * a + 2.0 * b;
    let quad_c = a;
    let mut roots = Vec::new();
    if quad_a.abs() < 1e-12 {
        if quad_b.abs() > 1e-12 {
            let t = -quad_c / quad_b;
            if (0.0..1.0).contains(&t) {
                roots.push(t);
            }
        }
        return roots;
    }
    let disc = quad_b * quad_b - 4.0 * quad_a * quad_c;
    if disc < 0.0 {
        return roots;
    }
    let root = disc.sqrt();
    for t in [(-quad_b + root) / (2.0 * quad_a), (-quad_b - root) / (2.0 * quad_a)] {
        if (0.0..1.0).contains(&t) {
            roots.push(t);
        }
    }
    roots
}

fn cubic_at(p: [f64; 4], t: f64) -> f64 {
    let u = 1.0 - t;
    u * u * u * p[0] + 3.0 * u * u * t * p[1] + 3.0 * u * t * t * p[2] + t * t * t * p[3]
}

fn uncovered(area: &Box2, segs: &[Seg]) -> Option<(f64, f64)> {
    for x in axis_samples(area.min_x, area.max_x, GRID_MM) {
        for y in axis_samples(area.min_y, area.max_y, GRID_MM) {
            let best = segs.iter().fold(f64::INFINITY, |best, seg| best.min(dist_to_seg(x, y, seg)));
            if best > TOOL_RADIUS_MM + 1e-6 {
                return Some((x, y));
            }
        }
    }
    None
}

fn outside(area: &Box2, x: f64, y: f64) -> bool {
    const EPS: f64 = 1.0e-6;
    x < area.min_x - EPS || x > area.max_x + EPS || y < area.min_y - EPS || y > area.max_y + EPS
}

fn axis_samples(min: f64, max: f64, step: f64) -> Vec<f64> {
    let mut values = Vec::new();
    let mut cursor = min;
    let mut guard = 0usize;
    while cursor < max - 1e-9 {
        values.push(cursor);
        cursor += step;
        guard += 1;
        if guard > 100_000 {
            break;
        }
    }
    values.push(max);
    values
}

fn dist_to_seg(x: f64, y: f64, seg: &Seg) -> f64 {
    let dx = seg.x1 - seg.x0;
    let dy = seg.y1 - seg.y0;
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-18 {
        return (x - seg.x0).hypot(y - seg.y0);
    }
    let t = (((x - seg.x0) * dx + (y - seg.y0) * dy) / len2).clamp(0.0, 1.0);
    (x - (seg.x0 + t * dx)).hypot(y - (seg.y0 + t * dy))
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
