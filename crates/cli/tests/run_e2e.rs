//! `campanion run` of the workspace sample jobs.
//!
//! The integration-test crate has no serde dependency, so IR JSON is parsed here. `verify`
//! then has to report an empty issue list.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const RETRACT_Z: f64 = 2.0;
const CLEARANCE_Z: f64 = 5.0;
const PIERCE_Z: f64 = 3.0;
const PLUNGE_FEED: f64 = 300.0;
const PIERCE_DWELL_S: f64 = 0.5;
const POST_DWELL_S: f64 = 0.25;
/// NGC prints four decimal places. This is looser than one ulp of that text.
const EPS: f64 = 1.0e-4;

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

fn near(actual: f64, expected: f64) -> bool {
    (actual - expected).abs() <= EPS
}

fn stem(job: &str) -> String {
    let name = job.rsplit('/').next().unwrap_or(job);
    name.trim_end_matches(".json").trim_end_matches(".job").to_string()
}

#[test]
fn sample_jobs_write_ngc_and_clean_ir() -> Result<(), String> {
    let root = workspace_root()?;
    let mut failures = Vec::new();
    for (job, delays) in [("examples/sample.job.json", false), ("examples/sample_delays.job.json", true), ("examples/sample_svg.job.json", false)] {
        if let Err(err) = check_job(&root, job, delays) {
            failures.push(err);
        }
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

fn check_job(root: &Path, job: &str, delays: bool) -> Result<(), String> {
    let name = stem(job);
    let ngc_path = temp_path(&format!("campanion-run-e2e-{name}.ngc"));
    let ir_path = temp_path(&format!("campanion-run-e2e-{name}.json"));
    run_job(root, job, &ngc_path, &ir_path)?;
    let ngc = std::fs::read_to_string(&ngc_path).map_err(|err| format!("{job}: read ngc: {err}"))?;
    let ir = std::fs::read_to_string(&ir_path).map_err(|err| format!("{job}: read ir: {err}"))?;
    let mut failures = Vec::new();
    if let Err(err) = assert_ngc(job, &ngc, delays) {
        failures.push(err);
    }
    if let Err(err) = assert_ir_json(job, &ir) {
        failures.push(err);
    }
    if let Err(err) = assert_verify_clean(root, job, &ir_path) {
        failures.push(err);
    }
    if failures.is_empty() { Ok(()) } else { Err(failures.join("\n")) }
}

fn run_job(root: &Path, job: &str, ngc_path: &Path, ir_path: &Path) -> Result<(), String> {
    let ngc_arg = path_arg(ngc_path)?;
    let ir_arg = path_arg(ir_path)?;
    let output = campanion(root, &["run", job, "-o", &ngc_arg, "--ir", &ir_arg])?;
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

fn assert_ngc(job: &str, ngc: &str, delays: bool) -> Result<(), String> {
    if ngc.trim().is_empty() {
        return Err(format!("{job}: ngc is empty"));
    }
    let lines: Vec<&str> = ngc.lines().collect();
    let last = lines.iter().rev().map(|line| line.trim()).find(|line| !line.is_empty()).unwrap_or("");
    if last != "M30" {
        return Err(format!("{job}: last non-empty line is `{last}`, expected M30"));
    }
    let mut scan = Scan::new(job, &lines, delays);
    scan.walk()?;
    scan.finish()
}

struct Scan<'a> {
    job: &'a str,
    lines: &'a [&'a str],
    delays: bool,
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
    /// Feed is modal inside one `(toolpath` section. The writer clears it at each section.
    feed: Option<f64>,
    sections: usize,
    section_open: bool,
    section_cut: bool,
    /// Line number of an `M5` that has not yet been followed by a rapid to clearance.
    m5_line: Option<usize>,
}

impl<'a> Scan<'a> {
    fn new(job: &'a str, lines: &'a [&'a str], delays: bool) -> Self {
        Self { job, lines, delays, x: None, y: None, z: None, feed: None, sections: 0, section_open: false, section_cut: false, m5_line: None }
    }

    fn walk(&mut self) -> Result<(), String> {
        for (index, line) in self.lines.iter().enumerate() {
            let raw = line.trim();
            if raw.is_empty() {
                continue;
            }
            if raw.contains("(toolpath") {
                self.open_section(index + 1)?;
                continue;
            }
            let code = strip_comments(raw);
            if code.is_empty() {
                continue;
            }
            let words = parse_words(&code).map_err(|err| format!("{}:{}: {err}\n{raw}", self.job, index + 1))?;
            if let Some(motion) = words.motion {
                self.on_motion(index + 1, raw, motion, &words)?;
            }
            if let Some(m) = words.m {
                self.on_m(index, m)?;
            }
        }
        Ok(())
    }

    fn open_section(&mut self, line_no: usize) -> Result<(), String> {
        if self.section_open && !self.section_cut {
            return Err(format!("{job}:{line_no}: previous (toolpath section has no G1/G2/G3 cutting move at Z<0", job = self.job));
        }
        self.section_open = true;
        self.section_cut = false;
        self.sections += 1;
        self.feed = None;
        Ok(())
    }

    fn on_motion(&mut self, line_no: usize, raw: &str, motion: u8, words: &Words) -> Result<(), String> {
        let x = words.x.or(self.x).ok_or_else(|| self.err(line_no, "move has no X"))?;
        let y = words.y.or(self.y).ok_or_else(|| self.err(line_no, "move has no Y"))?;
        let z = words.z.or(self.z).ok_or_else(|| self.err(line_no, "move has no Z"))?;
        let xy_changed = match (self.x, self.y) {
            (Some(x0), Some(y0)) => !near(x0, x) || !near(y0, y),
            _ => false,
        };
        if motion == 0 {
            self.check_rapid(line_no, raw, z, xy_changed)?;
        } else {
            self.check_cut(line_no, raw, z, words.feed)?;
        }
        if let Some(feed) = words.feed {
            self.feed = Some(feed);
        }
        self.x = Some(x);
        self.y = Some(y);
        self.z = Some(z);
        Ok(())
    }

    /// No rapid may end below the job retract. The allowed exception list is empty.
    /// An XY rapid has to be at clearance. Absolute `G90` coordinates carry across sections.
    fn check_rapid(&mut self, line_no: usize, raw: &str, z: f64, xy_changed: bool) -> Result<(), String> {
        if z < RETRACT_Z - EPS {
            return Err(format!("{job}:{line_no}: G0 Z{z:.4} is below retract height {RETRACT_Z:.1}; exceptions: none\n{raw}", job = self.job));
        }
        if xy_changed && !near(z, CLEARANCE_Z) {
            return Err(format!("{job}:{line_no}: G0 changes X or Y at Z{z:.4}, expected clearance Z{CLEARANCE_Z:.1}\n{raw}", job = self.job));
        }
        if let Some(m5_at) = self.m5_line {
            if xy_changed {
                return Err(format!(
                    "{job}:{line_no}: G0 changes X or Y before the G0 to Z{CLEARANCE_Z:.1} required after M5 at line {m5_at}\n{raw}",
                    job = self.job
                ));
            }
            if near(z, CLEARANCE_Z) {
                self.m5_line = None;
            }
        }
        Ok(())
    }

    /// A feed move that descends and finishes below Z0 is a plunge and uses the plunge feed.
    fn check_cut(&mut self, line_no: usize, raw: &str, z: f64, word_feed: Option<f64>) -> Result<(), String> {
        if z < 0.0 {
            if !self.section_open {
                return Err(self.err(line_no, "cutting move is outside a (toolpath section"));
            }
            self.section_cut = true;
        }
        let Some(start_z) = self.z else {
            return Ok(());
        };
        if z < 0.0 && z < start_z - EPS {
            let feed = word_feed.or(self.feed).ok_or_else(|| self.err(line_no, "plunge into Z<0 has no feed"))?;
            if !near(feed, PLUNGE_FEED) {
                return Err(format!("{job}:{line_no}: plunge into Z{z:.4} uses F{feed}, expected F{PLUNGE_FEED:.0}\n{raw}", job = self.job));
            }
        }
        Ok(())
    }

    fn on_m(&mut self, index: usize, code: u32) -> Result<(), String> {
        match code {
            3 => self.on_m3(index),
            5 => self.on_m5(index),
            _ => Ok(()),
        }
    }

    fn on_m3(&self, index: usize) -> Result<(), String> {
        if !self.delays {
            return Ok(());
        }
        let line_no = index + 1;
        let prev = prev_code(self.lines, index).ok_or_else(|| format!("{}:{line_no}: M3 has no previous move", self.job))?;
        let words = parse_words(&strip_comments(prev)).map_err(|err| format!("{}:{line_no}: move before M3: {err}\n{prev}", self.job))?;
        if words.motion.is_none() {
            return Err(format!("{}:{line_no}: move before M3 is not a G0/G1/G2/G3\n{prev}", self.job));
        }
        let Some(z) = words.z else {
            return Err(format!("{}:{line_no}: move before M3 has no Z word\n{prev}", self.job));
        };
        if !near(z, PIERCE_Z) {
            return Err(format!("{}:{line_no}: move before M3 is at Z{z:.4}, expected pierce height Z{PIERCE_Z:.1}\n{prev}", self.job));
        }
        let next = next_code(self.lines, index).ok_or_else(|| format!("{}:{line_no}: M3 is not followed by G4 P{PIERCE_DWELL_S}", self.job))?;
        expect_dwell(self.job, line_no, next, PIERCE_DWELL_S, "M3")
    }

    fn on_m5(&mut self, index: usize) -> Result<(), String> {
        let line_no = index + 1;
        if self.m5_line.is_some() {
            return Err(format!("{}:{line_no}: M5 while a previous M5 is still waiting for a G0 to Z{CLEARANCE_Z:.1}", self.job));
        }
        self.m5_line = Some(line_no);
        if !self.delays {
            return Ok(());
        }
        let next = next_code(self.lines, index).ok_or_else(|| format!("{}:{line_no}: M5 is not followed by G4 P{POST_DWELL_S}", self.job))?;
        expect_dwell(self.job, line_no, next, POST_DWELL_S, "M5")
    }

    fn finish(&self) -> Result<(), String> {
        if self.sections == 0 {
            return Err(format!("{}: ngc has no (toolpath section", self.job));
        }
        if self.section_open && !self.section_cut {
            return Err(format!("{}: a (toolpath section has no G1/G2/G3 cutting move at Z<0", self.job));
        }
        if let Some(line) = self.m5_line {
            let tail = self.lines.iter().skip(line.saturating_sub(1)).take(6).copied().collect::<Vec<_>>().join("\n");
            return Err(format!(
                "{}: M5 at line {line} is not followed by a G0 to Z{CLEARANCE_Z:.1} before the next G0 that changes X or Y\n{tail}",
                self.job
            ));
        }
        Ok(())
    }

    fn err(&self, line_no: usize, message: &str) -> String {
        format!("{}:{line_no}: {message}", self.job)
    }
}

fn expect_dwell(job: &str, line_no: usize, next: &str, seconds: f64, after: &str) -> Result<(), String> {
    let words = parse_words(&strip_comments(next)).map_err(|err| format!("{job}:{line_no}: dwell after {after}: {err}\n{next}"))?;
    let Some(p) = words.p else {
        return Err(format!("{job}:{line_no}: {after} is not followed by G4 P{seconds}\n{next}"));
    };
    if !words.dwell || !near(p, seconds) {
        return Err(format!("{job}:{line_no}: {after} is followed by `{next}`, expected G4 P{seconds}"));
    }
    Ok(())
}

fn prev_code<'a>(lines: &[&'a str], index: usize) -> Option<&'a str> {
    lines.iter().take(index).rev().map(|line| line.trim()).find(|line| !line.is_empty())
}

fn next_code<'a>(lines: &[&'a str], index: usize) -> Option<&'a str> {
    lines.iter().skip(index.saturating_add(1)).map(|line| line.trim()).find(|line| !line.is_empty())
}

fn strip_comments(line: &str) -> String {
    let mut out = String::new();
    let mut depth = 0i32;
    for ch in line.chars() {
        match ch {
            '(' => depth += 1,
            ')' if depth > 0 => depth -= 1,
            _ if depth == 0 => out.push(ch),
            _ => {}
        }
    }
    out.trim().to_string()
}

struct Words {
    motion: Option<u8>,
    dwell: bool,
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
    feed: Option<f64>,
    p: Option<f64>,
    m: Option<u32>,
}

fn parse_words(code: &str) -> Result<Words, String> {
    let mut words = Words { motion: None, dwell: false, x: None, y: None, z: None, feed: None, p: None, m: None };
    for word in code.split_whitespace() {
        let mut chars = word.chars();
        let Some(letter) = chars.next() else {
            continue;
        };
        let rest = chars.as_str();
        match letter {
            'G' => parse_g(rest, &mut words)?,
            'M' => {
                let code = parse_code(rest)?;
                if words.m.is_some() {
                    return Err(format!("repeated M word in `{word}`"));
                }
                words.m = Some(code);
            }
            'X' => set_word(&mut words.x, rest, "X")?,
            'Y' => set_word(&mut words.y, rest, "Y")?,
            'Z' => set_word(&mut words.z, rest, "Z")?,
            'F' => set_word(&mut words.feed, rest, "F")?,
            'P' => set_word(&mut words.p, rest, "P")?,
            'I' | 'J' | 'K' => {
                parse_number(rest)?;
            }
            _ => return Err(format!("unsupported word `{word}`")),
        }
    }
    Ok(words)
}

fn parse_g(rest: &str, words: &mut Words) -> Result<(), String> {
    let code = parse_code(rest)?;
    match code {
        0..=3 => {
            if words.motion.is_some() || words.dwell {
                return Err(format!("two motion codes in one block (G{code})"));
            }
            words.motion = Some(u8::try_from(code).map_err(|_| format!("G{code} does not fit in u8"))?);
        }
        4 => {
            if words.motion.is_some() || words.dwell {
                return Err("two motion codes in one block (G4)".into());
            }
            words.dwell = true;
        }
        17 | 18 | 19 | 20 | 21 | 90 | 91 => {}
        other => return Err(format!("unsupported G{other}")),
    }
    Ok(())
}

fn set_word(slot: &mut Option<f64>, rest: &str, name: &str) -> Result<(), String> {
    if slot.is_some() {
        return Err(format!("repeated {name} word"));
    }
    *slot = Some(parse_number(rest)?);
    Ok(())
}

fn parse_code(text: &str) -> Result<u32, String> {
    let value = parse_number(text)?;
    if value < 0.0 {
        return Err(format!("negative word value {text}"));
    }
    let rounded = value.round();
    if (value - rounded).abs() > 1.0e-6 {
        return Err(format!("word value {text} is not an integer code"));
    }
    u32::try_from(rounded as u64).map_err(|_| format!("word value {text} is out of range"))
}

fn parse_number(text: &str) -> Result<f64, String> {
    let value: f64 = text.parse().map_err(|_| format!("bad number `{text}`"))?;
    if value.is_finite() { Ok(value) } else { Err(format!("non-finite number `{text}`")) }
}

fn assert_ir_json(job: &str, text: &str) -> Result<(), String> {
    let root = JsonParser::parse(text).map_err(|err| format!("{job}: ir json: {err}"))?;
    let format = root.object_get("format")?.as_str().map_err(|err| format!("{job}: ir format: {err}"))?;
    if format != "campanion-ir/1" {
        return Err(format!("{job}: ir format is `{format}`"));
    }
    let toolpaths = root.object_get("program")?.object_get("toolpaths")?.as_array().map_err(|err| format!("{job}: ir toolpaths: {err}"))?;
    if toolpaths.is_empty() {
        return Err(format!("{job}: ir program has no toolpaths"));
    }
    let tally = json_tally(&root)?;
    if tally.numbers == 0 || tally.trues == 0 || tally.falses == 0 {
        return Err(format!("{job}: ir json is missing a number, true, or false"));
    }
    Ok(())
}

struct JsonTally {
    numbers: usize,
    trues: usize,
    falses: usize,
}

/// Reads every number and boolean so a parsed payload is not only a shape check.
fn json_tally(value: &Json) -> Result<JsonTally, String> {
    let mut tally = JsonTally { numbers: 0, trues: 0, falses: 0 };
    json_tally_into(value, &mut tally)?;
    Ok(tally)
}

fn json_tally_into(value: &Json, tally: &mut JsonTally) -> Result<(), String> {
    match value {
        Json::Null | Json::String(_) => Ok(()),
        Json::Bool(true) => {
            tally.trues += 1;
            Ok(())
        }
        Json::Bool(false) => {
            tally.falses += 1;
            Ok(())
        }
        Json::Number(number) => {
            if !number.is_finite() {
                return Err("number is not finite".into());
            }
            tally.numbers += 1;
            Ok(())
        }
        Json::Array(items) => {
            for item in items {
                json_tally_into(item, tally)?;
            }
            Ok(())
        }
        Json::Object(fields) => {
            for (_, item) in fields {
                json_tally_into(item, tally)?;
            }
            Ok(())
        }
    }
}

enum Json {
    Null,
    Bool(bool),
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
            Some(b't') => self.literal(b"true", Json::Bool(true)),
            Some(b'f') => self.literal(b"false", Json::Bool(false)),
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
        let value = parse_number(text)?;
        Ok(Json::Number(value))
    }

    fn digits(&mut self) -> Result<(), String> {
        let start = self.index;
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.index += 1;
        }
        if self.index == start { Err("number needs a digit".into()) } else { Ok(()) }
    }
}
