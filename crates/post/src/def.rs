//! Post definition loaded from JSON.
//!
//! [`PostDef`] is validated by [`crate::load_post`]. Unknown fields are rejected. Templates may
//! use `{program_name}`, `{program_number}`, `{speed}`, `{power}`, `{tool}` and `{date-free!}`.
//! `{program_number}` is the post's `program_number` digits (`O{program_number}` becomes `O0001`).
//! `{tool}` is the tool id on a tool-change template. `{date-free!}` expands to nothing. Date and
//! time placeholders are rejected so output stays stable.
//!
//! `{power}` on a process line is the IR power. With `power_scale` it is written as an integer,
//! `ir_power * power_scale`. A missing IR power then uses full scale (`1 * power_scale`) and the
//! writer adds a comment. `process_feeds.rapid_z` drops rapids that only change Z. `z_mode`
//! `drop` omits Z words. `thc.on` is written on the first feed move after process on and a dwell,
//! and `thc.off` is written before process off.
//!
//! `rotary` is optional. Without it, a program that carries a rotary angle is rejected. `axes`
//! lists `a`, `b`, or both. The A/B word uses [`crate::fmt::format_fixed`] with `decimals`
//! (degrees). `feed_mode` is `inverse_time`, `degrees_per_minute`, or `surface`. Inverse time
//! requires `inverse_time_on` and `inverse_time_off` (LinuxCNC uses `G93` and `G94`).
//!
//! `lathe` is optional. A turning program requires it, and a lathe post requires a turning
//! program. Lathe templates may use `{surface}`, `{max_rpm}`, `{rpm}`, `{tool}`, `{tool2}`,
//! `{offset2}` and `{date-free!}`, each only on the lines that name that value.

use std::collections::BTreeMap;

use campanion_ir::RotaryAxisKind;
use serde::Deserialize;

use crate::error::{PostError, Result, field};

const MAX_POST_BYTES: usize = 256 * 1024;
const MAX_LINES: usize = 64;
const MAX_TEMPLATE_CHARS: usize = 400;
const HEADER_KEYS: &[&str] = &["program_name", "program_number", "date-free!"];
const PROCESS_KEYS: &[&str] = &["program_name", "date-free!", "speed", "power"];
const TOOL_KEYS: &[&str] = &["program_name", "date-free!", "tool"];
const LATHE_BARE: &[&str] = &["date-free!"];
const LATHE_CSS: &[&str] = &["surface", "max_rpm", "date-free!"];
const LATHE_RPM: &[&str] = &["rpm", "date-free!"];
const LATHE_TOOL: &[&str] = &["tool", "tool2", "offset2", "date-free!"];

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
    /// When `rapid_z` is false, a rapid that does not change X or Y is left out.
    #[serde(default)]
    pub(crate) process_feeds: ProcessFeeds,
    /// `{power}` becomes the integer `ir_power * power_scale`. Omit to keep the raw power text.
    #[serde(default)]
    pub(crate) power_scale: Option<f64>,
    /// `keep` writes Z. `drop` omits the Z word on every motion block.
    #[serde(default = "default_z_mode")]
    pub(crate) z_mode: String,
    pub(crate) process: BTreeMap<String, ProcessWords>,
    /// Torch-height control. `on` follows the pierce dwell. `off` precedes process off.
    #[serde(default)]
    pub(crate) thc: Option<ThcWords>,
    pub(crate) tool_change: Vec<String>,
    pub(crate) coolant: CoolantWords,
    pub(crate) comments: Comments,
    pub(crate) program_end: Vec<String>,
    #[serde(default)]
    pub(crate) max_line_length: Option<u32>,
    #[serde(default = "default_axis_letters")]
    pub(crate) axis_letters: String,
    /// Fourth-axis words. Absent on a post that rejects rotary programs.
    #[serde(default)]
    pub(crate) rotary: Option<RotaryOutput>,
    /// Lathe words. Absent on a mill post. A turning program requires this block.
    #[serde(default)]
    pub(crate) lathe: Option<LatheOutput>,
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

/// Whether positioning rapids along Z are written.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ProcessFeeds {
    /// False drops a rapid whose rounded X and Y match the previous point.
    pub(crate) rapid_z: bool,
}

impl Default for ProcessFeeds {
    fn default() -> Self {
        Self { rapid_z: true }
    }
}

/// Lines for torch-height control.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ThcWords {
    /// Written on the first feed move after process on and a pierce dwell.
    pub(crate) on: Vec<String>,
    /// Written immediately before process off.
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

/// How a feed that changes the rotary angle is written.
///
/// `inverse_time` switches to [`RotaryOutput::inverse_time_on`] and sets `F` to `1 / minutes`.
/// `degrees_per_minute` and `surface` write the IR feed number unchanged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RotaryFeedMode {
    /// `F` is `1 / T` with `T` in minutes. The move length is the part-frame tip distance.
    InverseTime,
    /// `F` is the IR feed. Controllers that mix millimetres and degrees read that number as-is.
    DegreesPerMinute,
    /// `F` is the IR surface feed, millimetres per minute.
    Surface,
}

/// A/B word and the feed mode used when the angle changes.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct RotaryOutput {
    /// `"a"`, `"b"`, or both, in the order the post lists them.
    pub(crate) axes: Vec<String>,
    /// Digits after the decimal point on the A/B word. The angle is degrees.
    pub(crate) decimals: u8,
    /// `inverse_time`, `degrees_per_minute`, or `surface`.
    pub(crate) feed_mode: String,
    /// Modal code that selects inverse time, such as `G93`. Required for `inverse_time`.
    #[serde(default)]
    pub(crate) inverse_time_on: Option<String>,
    /// Modal code that returns to units per minute, such as `G94`. Required for `inverse_time`.
    #[serde(default)]
    pub(crate) inverse_time_off: Option<String>,
}

impl RotaryOutput {
    pub(crate) fn mode(&self) -> RotaryFeedMode {
        match self.feed_mode.as_str() {
            "degrees_per_minute" => RotaryFeedMode::DegreesPerMinute,
            "surface" => RotaryFeedMode::Surface,
            _ => RotaryFeedMode::InverseTime,
        }
    }

    pub(crate) fn supports(&self, kind: RotaryAxisKind) -> bool {
        let name = match kind {
            RotaryAxisKind::A => "a",
            RotaryAxisKind::B => "b",
        };
        self.axes.iter().any(|axis| axis == name)
    }

    pub(crate) fn on_code(&self) -> Option<u16> {
        self.inverse_time_on.as_deref().and_then(g_code_number)
    }

    pub(crate) fn off_code(&self) -> Option<u16> {
        self.inverse_time_off.as_deref().and_then(g_code_number)
    }
}

pub(crate) fn g_code_number(text: &str) -> Option<u16> {
    text.strip_prefix('G')?.parse().ok()
}

/// How a lathe post writes X, spindle speed, tool changes and the ZX plane.
///
/// `diameter_mode` doubles X words. `I` stays a radius offset either way. `arc_flip` swaps G2
/// and G3 for a front turret. `home` is written before `program_end` and may be empty.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LatheOutput {
    /// When true, X words are diameters (`2 * radius`). Read-back divides X by two.
    pub(crate) diameter_mode: bool,
    /// Written once after the header. LinuxCNC uses `G7` (diameter) or `G8` (radius). Fanuc uses none.
    pub(crate) mode_lines: Vec<String>,
    /// Plane word written once after `mode_lines`, such as `G18`.
    pub(crate) plane: String,
    /// Feed-mode word written once after the plane, such as `G94` or `G98`.
    pub(crate) feed_mode: String,
    /// Constant surface speed. `{surface}` is metres per minute. `{max_rpm}` is the cap.
    pub(crate) css: Vec<String>,
    /// Direct revolutions per minute. `{rpm}` is the spindle speed.
    pub(crate) rpm: Vec<String>,
    /// Spindle forward, after the css or rpm lines.
    pub(crate) spindle_cw: Vec<String>,
    /// Spindle reverse, when the bound spindle is reversed.
    pub(crate) spindle_ccw: Vec<String>,
    /// Spindle stop.
    pub(crate) spindle_off: Vec<String>,
    /// Replaces the mill `tool_change` lines for a turning program. `{tool}` is the id, `{tool2}` and
    /// `{offset2}` are that id in two digits.
    pub(crate) tool_change: Vec<String>,
    /// When true, G2 and G3 are swapped (front turret, X-down pictures).
    #[serde(default)]
    pub(crate) arc_flip: bool,
    /// Machine-coordinate retract written before `program_end`. May be empty.
    #[serde(default)]
    pub(crate) home: Vec<String>,
}

fn default_z_mode() -> String {
    "keep".to_string()
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
        check_power_scale(self.power_scale)?;
        if !matches!(self.z_mode.as_str(), "keep" | "drop") {
            return Err(field("z_mode", "must be keep or drop"));
        }
        check_process(&self.process)?;
        if let Some(thc) = &self.thc {
            check_templates("thc.on", &thc.on, PROCESS_KEYS)?;
            check_templates("thc.off", &thc.off, PROCESS_KEYS)?;
        }
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
        check_rotary(&mut self)?;
        check_lathe(&self)?;
        Ok(self)
    }

    /// True when this post writes a turning program.
    pub fn is_lathe(&self) -> bool {
        self.lathe.is_some()
    }

    /// Rotary axes this post writes, in the order the JSON lists them.
    ///
    /// Empty when the post has no `rotary` block. A program whose setup axis is missing from
    /// this list is rejected by name.
    pub fn rotary_axes(&self) -> Vec<RotaryAxisKind> {
        let Some(rotary) = &self.rotary else {
            return Vec::new();
        };
        rotary
            .axes
            .iter()
            .filter_map(|axis| match axis.as_str() {
                "a" => Some(RotaryAxisKind::A),
                "b" => Some(RotaryAxisKind::B),
                _ => None,
            })
            .collect()
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

    pub(crate) fn drop_z(&self) -> bool {
        self.z_mode == "drop"
    }
}

fn check_lathe(post: &PostDef) -> Result<()> {
    let Some(lathe) = &post.lathe else {
        return Ok(());
    };
    if lathe.plane.is_empty() {
        return Err(field("lathe.plane", "must not be empty"));
    }
    check_templates("lathe.mode_lines", &lathe.mode_lines, LATHE_BARE)?;
    check_templates("lathe.plane", std::slice::from_ref(&lathe.plane), LATHE_BARE)?;
    check_templates("lathe.feed_mode", std::slice::from_ref(&lathe.feed_mode), LATHE_BARE)?;
    check_templates("lathe.css", &lathe.css, LATHE_CSS)?;
    check_templates("lathe.rpm", &lathe.rpm, LATHE_RPM)?;
    check_templates("lathe.spindle_cw", &lathe.spindle_cw, LATHE_BARE)?;
    check_templates("lathe.spindle_ccw", &lathe.spindle_ccw, LATHE_BARE)?;
    check_templates("lathe.spindle_off", &lathe.spindle_off, LATHE_BARE)?;
    check_templates("lathe.tool_change", &lathe.tool_change, LATHE_TOOL)?;
    check_templates("lathe.home", &lathe.home, LATHE_BARE)?;
    Ok(())
}

fn check_power_scale(scale: Option<f64>) -> Result<()> {
    let Some(scale) = scale else {
        return Ok(());
    };
    if scale.is_finite() && (0.0..=1.0e9).contains(&scale) { Ok(()) } else { Err(field("power_scale", "must be finite and in 0..=1e9")) }
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

fn check_rotary(post: &mut PostDef) -> Result<()> {
    let Some(rotary) = post.rotary.as_mut() else {
        return Ok(());
    };
    if rotary.axes.is_empty() || rotary.axes.len() > 2 {
        return Err(field("rotary.axes", "must list a, b, or both"));
    }
    let mut seen_a = false;
    let mut seen_b = false;
    for axis in &mut rotary.axes {
        *axis = axis.to_ascii_lowercase();
        match axis.as_str() {
            "a" if !seen_a => seen_a = true,
            "b" if !seen_b => seen_b = true,
            "a" | "b" => return Err(field("rotary.axes", "lists the same axis twice")),
            _ => return Err(field("rotary.axes", "must be a and/or b")),
        }
    }
    if rotary.decimals > 8 {
        return Err(field("rotary.decimals", "must be 0..=8"));
    }
    match rotary.feed_mode.as_str() {
        "inverse_time" | "degrees_per_minute" | "surface" => {}
        _ => return Err(field("rotary.feed_mode", "must be inverse_time, degrees_per_minute or surface")),
    }
    if rotary.feed_mode == "inverse_time" {
        let on = rotary.inverse_time_on.as_deref().ok_or_else(|| field("rotary.inverse_time_on", "required for inverse_time"))?;
        let off = rotary.inverse_time_off.as_deref().ok_or_else(|| field("rotary.inverse_time_off", "required for inverse_time"))?;
        let on = canonical_g(on, "rotary.inverse_time_on")?;
        let off = canonical_g(off, "rotary.inverse_time_off")?;
        if on == off {
            return Err(field("rotary.inverse_time_off", "must differ from inverse_time_on"));
        }
        rotary.inverse_time_on = Some(on);
        rotary.inverse_time_off = Some(off);
    } else if rotary.inverse_time_on.is_some() || rotary.inverse_time_off.is_some() {
        return Err(field("rotary.inverse_time_on", "only valid when feed_mode is inverse_time"));
    }
    for axis in &rotary.axes {
        let letter = if axis == "a" { 'A' } else { 'B' };
        if post.axis_letters.contains(letter) {
            return Err(field("rotary.axes", format!("letter {letter} collides with axis_letters")));
        }
        if post.dwell.word.contains(letter) || post.line_numbers.prefix.contains(letter) {
            return Err(field("rotary.axes", format!("letter {letter} collides with a G-code word")));
        }
    }
    Ok(())
}

/// `G93` and `g94` both become `G93` / `G94`. The number is an integer from 0 to 999.
fn canonical_g(text: &str, field_name: &str) -> Result<String> {
    let upper = text.trim().to_ascii_uppercase();
    let Some(rest) = upper.strip_prefix('G') else {
        return Err(field(field_name, "must be a G code such as G93"));
    };
    if rest.is_empty() || rest.len() > 3 || !rest.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err(field(field_name, "must be a G code such as G93"));
    }
    let number: u16 = rest.parse().map_err(|_| field(field_name, "must be a G code such as G93"))?;
    Ok(format!("G{number}"))
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
