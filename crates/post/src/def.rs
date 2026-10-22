//! Post definition loaded from JSON.
//!
//! [`PostDef`] is validated by [`crate::load_post`]. Unknown fields are rejected. Templates may
//! use `{program_name}`, `{program_number}`, `{speed}`, `{power}`, `{tool}` and `{date-free!}`.
//! `{program_number}` is the post's `program_number` digits (`O{program_number}` becomes `O0001`).
//! `{tool}` is the tool id on a tool-change template. `{date-free!}` expands to nothing. Date and
//! time placeholders are rejected so output stays stable.

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::error::{PostError, Result, field};

const MAX_POST_BYTES: usize = 256 * 1024;
const MAX_LINES: usize = 64;
const MAX_TEMPLATE_CHARS: usize = 400;
const HEADER_KEYS: &[&str] = &["program_name", "program_number", "date-free!"];
const PROCESS_KEYS: &[&str] = &["program_name", "date-free!", "speed", "power"];
const TOOL_KEYS: &[&str] = &["program_name", "date-free!", "tool"];

/// How circular moves are written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ArcMode {
    /// Centre offsets `I` `J` `K` from the start of the block.
    Ijk,
    /// Signed radius `R`. A negative `R` is the long way around.
    R,
    /// Chords within [`crate::PostOptions::arc_tolerance`].
    Lines,
}

/// One machine dialect.
///
/// Build it with [`crate::load_post`] or [`crate::builtin`]. Fields stay private so a value that
/// escaped validation cannot be assembled by hand.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PostDef {
    pub(crate) id: String,
    pub(crate) name: String,
    pub(crate) description: String,
    /// Digits written for `{program_number}`, such as `0001`. Omit the field when no template uses it.
    #[serde(default)]
    pub(crate) program_number: Option<String>,
    pub(crate) units: String,
    pub(crate) line_numbers: LineNumbers,
    pub(crate) decimals: Decimals,
    pub(crate) trailing_zeros: bool,
    pub(crate) header: Vec<String>,
    pub(crate) footer: Vec<String>,
    pub(crate) modal_motion: bool,
    pub(crate) arcs: Arcs,
    pub(crate) planes: bool,
    pub(crate) dwell: Dwell,
    pub(crate) process: BTreeMap<String, ProcessWords>,
    pub(crate) tool_change: Vec<String>,
    pub(crate) coolant: CoolantWords,
    pub(crate) comments: Comments,
    pub(crate) program_end: Vec<String>,
    #[serde(default)]
    pub(crate) max_line_length: Option<u32>,
    #[serde(default = "default_axis_letters")]
    pub(crate) axis_letters: String,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LineNumbers {
    pub(crate) enabled: bool,
    pub(crate) start: u32,
    pub(crate) step: u32,
    pub(crate) prefix: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Decimals {
    pub(crate) linear: u8,
    pub(crate) feed: u8,
    pub(crate) dwell: u8,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Arcs {
    pub(crate) mode: String,
    pub(crate) max_sweep_deg: f64,
    pub(crate) helix: bool,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Dwell {
    pub(crate) word: String,
    pub(crate) unit: String,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ProcessWords {
    pub(crate) on: Vec<String>,
    pub(crate) off: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CoolantWords {
    pub(crate) flood: Vec<String>,
    pub(crate) mist: Vec<String>,
    pub(crate) off: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Comments {
    pub(crate) style: String,
}

fn default_axis_letters() -> String {
    "XYZ".to_string()
}

impl PostDef {
    pub(crate) fn checked(mut self) -> Result<Self> {
        check_token("id", &self.id, 64)?;
        check_text("name", &self.name, 1, 128)?;
        check_text("description", &self.description, 0, 1024)?;
        if self.units != "mm" {
            return Err(field("units", "only mm is supported"));
        }
        check_line_numbers(&mut self.line_numbers)?;
        check_decimals(&self.decimals)?;
        check_templates("header", &self.header, HEADER_KEYS)?;
        check_templates("footer", &self.footer, HEADER_KEYS)?;
        check_program_number(&self)?;
        check_arcs(&self.arcs)?;
        self.dwell.word = one_letter("dwell.word", &self.dwell.word)?;
        if !matches!(self.dwell.unit.as_str(), "s" | "ms") {
            return Err(field("dwell.unit", "must be s or ms"));
        }
        check_process(&self.process)?;
        check_templates("tool_change", &self.tool_change, TOOL_KEYS)?;
        check_coolant(&self.coolant)?;
        if !matches!(self.comments.style.as_str(), "paren" | "semicolon" | "none") {
            return Err(field("comments.style", "must be paren, semicolon or none"));
        }
        check_templates("program_end", &self.program_end, HEADER_KEYS)?;
        if self.program_end.is_empty() {
            return Err(field("program_end", "needs at least one line"));
        }
        if let Some(limit) = self.max_line_length
            && (limit == 0 || limit > 4096)
        {
            return Err(field("max_line_length", "must be 1..=4096"));
        }
        self.axis_letters = check_axes(&self.axis_letters)?;
        check_dwell_word(&self.dwell.word, &self.axis_letters, &self.line_numbers.prefix)?;
        Ok(self)
    }

    /// Short id, such as `linuxcnc`.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Human-readable name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// What the dialect is for.
    pub fn description(&self) -> &str {
        &self.description
    }

    /// Maximum characters on one output line, when the post sets a limit.
    pub fn max_line_length(&self) -> Option<usize> {
        self.max_line_length.map(|n| n as usize)
    }

    /// Digits after the decimal point on `X` `Y` `Z` `I` `J` `K` `R`.
    pub fn linear_decimals(&self) -> u8 {
        self.decimals.linear
    }

    pub(crate) fn arc_mode(&self) -> ArcMode {
        match self.arcs.mode.as_str() {
            "r" => ArcMode::R,
            "lines" => ArcMode::Lines,
            _ => ArcMode::Ijk,
        }
    }
}

fn check_token(field_name: &str, text: &str, max: usize) -> Result<()> {
    let ok = !text.is_empty() && text.len() <= max && text.bytes().all(|byte| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-');
    if ok { Ok(()) } else { Err(field(field_name, format!("use 1..={max} characters of letters, digits, '_' or '-'"))) }
}

fn check_text(field_name: &str, text: &str, min: usize, max: usize) -> Result<()> {
    let count = text.chars().count();
    if count < min {
        return Err(field(field_name, "must not be empty"));
    }
    if count > max {
        return Err(field(field_name, format!("longer than {max} characters")));
    }
    if text.chars().any(char::is_control) {
        return Err(field(field_name, "contains a control character"));
    }
    Ok(())
}

fn check_line_numbers(numbers: &mut LineNumbers) -> Result<()> {
    numbers.prefix = one_letter("line_numbers.prefix", &numbers.prefix)?;
    if numbers.step == 0 || numbers.step > 1_000_000 {
        return Err(field("line_numbers.step", "must be 1..=1000000"));
    }
    if numbers.start > 1_000_000_000 {
        return Err(field("line_numbers.start", "must be <= 1000000000"));
    }
    Ok(())
}

fn one_letter(field_name: &str, text: &str) -> Result<String> {
    let mut chars = text.chars();
    let Some(ch) = chars.next() else {
        return Err(field(field_name, "must be one ASCII letter"));
    };
    if chars.next().is_some() || !ch.is_ascii_alphabetic() {
        return Err(field(field_name, "must be one ASCII letter"));
    }
    Ok(ch.to_ascii_uppercase().to_string())
}

fn check_decimals(decimals: &Decimals) -> Result<()> {
    for (name, value) in [("decimals.linear", decimals.linear), ("decimals.feed", decimals.feed), ("decimals.dwell", decimals.dwell)] {
        if value > 8 {
            return Err(field(name, "must be 0..=8"));
        }
    }
    Ok(())
}

fn check_arcs(arcs: &Arcs) -> Result<()> {
    if !matches!(arcs.mode.as_str(), "ijk" | "r" | "lines") {
        return Err(field("arcs.mode", "must be ijk, r or lines"));
    }
    if !(arcs.max_sweep_deg.is_finite() && arcs.max_sweep_deg > 0.0 && arcs.max_sweep_deg <= 360.0) {
        return Err(field("arcs.max_sweep_deg", "must be > 0 and <= 360"));
    }
    Ok(())
}

fn check_axes(text: &str) -> Result<String> {
    let upper = text.to_ascii_uppercase();
    let chars: Vec<char> = upper.chars().collect();
    if chars.len() != 3 || !chars.iter().all(char::is_ascii_alphabetic) {
        return Err(field("axis_letters", "must be three ASCII letters"));
    }
    let Some((first, rest)) = chars.split_first() else {
        return Err(field("axis_letters", "must be three ASCII letters"));
    };
    if rest.len() != 2 || rest.contains(first) || rest[0] == rest[1] {
        return Err(field("axis_letters", "letters must be distinct"));
    }
    const RESERVED: &str = "IJKRFGMNSTP";
    if chars.iter().any(|ch| RESERVED.contains(*ch)) {
        return Err(field("axis_letters", "letters collide with G-code words"));
    }
    Ok(upper)
}

fn check_dwell_word(word: &str, axes: &str, prefix: &str) -> Result<()> {
    let Some(letter) = word.chars().next() else {
        return Err(field("dwell.word", "must be one ASCII letter"));
    };
    if axes.contains(letter) || "IJKRFGMNST".contains(letter) || prefix.starts_with(letter) {
        return Err(field("dwell.word", "collides with a G-code word"));
    }
    Ok(())
}

fn check_process(map: &BTreeMap<String, ProcessWords>) -> Result<()> {
    if map.len() > 32 {
        return Err(field("process", "too many entries"));
    }
    for (key, words) in map {
        check_token("process", key, 64).map_err(|_| field("process", format!("invalid key `{key}`")))?;
        check_templates(&format!("process.{key}.on"), &words.on, PROCESS_KEYS)?;
        check_templates(&format!("process.{key}.off"), &words.off, PROCESS_KEYS)?;
    }
    Ok(())
}

fn check_program_number(post: &PostDef) -> Result<()> {
    let used = post.header.iter().chain(post.footer.iter()).chain(post.program_end.iter()).any(|line| line.contains("{program_number}"));
    match &post.program_number {
        Some(number) => {
            let digits = (1..=8).contains(&number.len()) && number.bytes().all(|byte| byte.is_ascii_digit());
            if !digits {
                return Err(field("program_number", "must be 1..=8 digits"));
            }
        }
        None if used => return Err(field("program_number", "required when a template uses {program_number}")),
        None => {}
    }
    Ok(())
}

fn check_coolant(coolant: &CoolantWords) -> Result<()> {
    check_templates("coolant.flood", &coolant.flood, HEADER_KEYS)?;
    check_templates("coolant.mist", &coolant.mist, HEADER_KEYS)?;
    check_templates("coolant.off", &coolant.off, HEADER_KEYS)?;
    if coolant.flood.is_empty() {
        return Err(field("coolant.flood", "needs at least one line"));
    }
    if coolant.mist.is_empty() {
        return Err(field("coolant.mist", "needs at least one line"));
    }
    if coolant.off.is_empty() {
        return Err(field("coolant.off", "needs at least one line"));
    }
    Ok(())
}

fn check_templates(field_name: &str, lines: &[String], allowed: &[&str]) -> Result<()> {
    if lines.len() > MAX_LINES {
        return Err(field(field_name, format!("more than {MAX_LINES} lines")));
    }
    for line in lines {
        if line.chars().count() > MAX_TEMPLATE_CHARS {
            return Err(field(field_name, format!("line is longer than {MAX_TEMPLATE_CHARS} characters")));
        }
        check_placeholders(field_name, line, allowed)?;
    }
    Ok(())
}

fn check_placeholders(field_name: &str, line: &str, allowed: &[&str]) -> Result<()> {
    let mut rest = line;
    while let Some(start) = rest.find('{') {
        let after = &rest[start + 1..];
        let Some(end) = after.find('}') else {
            return Err(field(field_name, "unclosed '{'"));
        };
        let key = &after[..end];
        let dated = key != "date-free!" && (key.contains("date") || key.contains("time") || key == "now" || key == "timestamp");
        if dated {
            return Err(field(field_name, format!("placeholder {{{key}}} is not allowed; posts do not emit dates or timestamps")));
        }
        if !allowed.contains(&key) {
            return Err(field(field_name, format!("unknown placeholder {{{key}}}")));
        }
        rest = &after[end + 1..];
    }
    Ok(())
}

/// Parse and validate a post definition.
pub fn load_post(json: &str) -> Result<PostDef> {
    if json.len() > MAX_POST_BYTES {
        return Err(field("post", format!("json is larger than {MAX_POST_BYTES} bytes")));
    }
    let post: PostDef = serde_json::from_str(json).map_err(|err| PostError::Json(err.to_string()))?;
    post.checked()
}
