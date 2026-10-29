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
//! `process_cycle` is `hold` or `per_cut`. The default is `hold`. A hold post does not write a
//! second spindle-on when that spindle is already running at the same speed, so the spindle stays
//! on through later rapids. A per_cut post writes every process-on. When a process is on and the
//! next motion is a rapid, it writes that process's off lines and `thc.off` before the rapid, then
//! the on lines again before the next feed. Those on lines reuse the speed and power already in
//! effect. A pierce dwell is written only when the IR has a dwell at that cut; the post does not
//! invent one. Plasma and the Grbl laser post set `per_cut`, so a torch or beam mapped from any
//! process, including `spindle`, is off during G0 travel.
//!
//! `rotary` is optional. Without it, a program that carries a rotary angle is rejected. `axes`
//! lists `a`, `b`, or both. The A/B word uses the dimensional formatter with `decimals`
//! (degrees). `decimal_point` keeps a point on that word and on `X` `Y` `Z` `I` `J` `K` `R`
//! (`90.`, `0.`). `feed_mode` is `inverse_time`, `degrees_per_minute`, or `surface`. Inverse time
//! requires `inverse_time_on` and `inverse_time_off` (LinuxCNC uses `G93` and `G94`).
//!
//! `lathe` is optional. A turning program requires it, and a lathe post requires a turning
//! program. Lathe templates may use `{surface}`, `{max_rpm}`, `{rpm}`, `{tool}`, `{tool2}`,
//! `{offset2}`, `{phase}`, `{z}`, `{feed}`, `{b}` and `{date-free!}`, each only on the lines that name that value. `c_axis`, `brake`, `live_tool`, `face_interp` and
//! `cylinder_interp` are optional mill-turn blocks. `b_head` is an optional tilting-B-head block. `transfer` is an optional part-transfer block. `c_axis.engage`, `c_axis.disengage`,
//! `brake.clamp` and `brake.release` may be empty; an empty list writes nothing. Live-tool
//! `forward`, `reverse` and `stop` must not be empty. Live-tool `{rpm}` is a whole number.
//! `c_feed` is `per_minute` or `inverse_time`. Cylinder `on` lines may use `{radius}` and
//! `{diameter}`. `references` cites the codes and is never written to the program.

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
const LATHE_CYLINDER: &[&str] = &["radius", "diameter", "date-free!"];
const LATHE_PHASE: &[&str] = &["phase", "date-free!"];
const LATHE_MOVE: &[&str] = &["z", "date-free!"];
const LATHE_MOVE_FEED: &[&str] = &["z", "feed", "date-free!"];
const LATHE_B: &[&str] = &["b", "date-free!"];

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
    /// When true, dimensional words always include a decimal point (`52.`, `0.`).
    ///
    /// Fanuc and Haas read a word with no point as least-input increments (`X52` is 0.052 mm,
    /// `A90` is 0.09 degrees). Feed, spindle, dwell, line numbers, and tool and program numbers
    /// are unchanged. Defaults to false so a dialect that reads plain decimals can omit the key.
    #[serde(default)]
    pub(crate) decimal_point: bool,
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
    /// `hold` keeps a running spindle on across rapids. `per_cut` cycles every process off before a rapid.
    #[serde(default = "default_process_cycle")]
    pub(crate) process_cycle: String,
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
    /// Indexed 3+2. Absent when a tilted tool axis is rejected.
    #[serde(default)]
    pub(crate) five_axis: Option<FiveAxisOutput>,
    /// Lathe words. Absent on a mill post. A turning program requires this block.
    #[serde(default)]
    pub(crate) lathe: Option<LatheOutput>,
    /// Citations for the codes this post writes. Never copied into the program.
    #[serde(default)]
    pub(crate) references: Vec<String>,
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

/// Synchronised feed for single-point threading.
///
/// Absent on a lathe post: thread moves are rejected.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ThreadOutput {
    /// Motion code, for example `G33` or `G32`.
    pub(crate) code: String,
    /// Word letter carrying the lead, for example `K` or `F`.
    pub(crate) lead_word: String,
    /// Feed-mode word written before the first thread move of a run, such as `G99`, and left
    /// with `lathe.feed_mode` before the next feed move or at the end of the toolpath. Empty: none.
    #[serde(default)]
    pub(crate) feed_mode: String,
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
    /// Synchronised feed for threading. Absent: the post rejects thread moves.
    #[serde(default)]
    pub(crate) thread: Option<ThreadOutput>,
    /// Positioning C axis. Absent: a C angle or a C-axis event is rejected.
    #[serde(default)]
    pub(crate) c_axis: Option<CAxisOutput>,
    /// Spindle brake while the C axis holds. Absent: a brake event is rejected.
    #[serde(default)]
    pub(crate) brake: Option<BrakeOutput>,
    /// Driven tool. Absent: a live-tool event is rejected.
    #[serde(default)]
    pub(crate) live_tool: Option<LiveToolOutput>,
    /// Cartesian XY to XC face interpolation. Absent: a face event falls back to X and C when
    /// `c_axis` is set.
    #[serde(default)]
    pub(crate) face_interp: Option<FaceInterpOutput>,
    /// Y to C cylinder interpolation. Absent: a cylinder event falls back to X and C when
    /// `c_axis` is set.
    #[serde(default)]
    pub(crate) cylinder_interp: Option<CylInterpOutput>,
    /// Cross axis for live milling. Absent: a pose with Y off the centreline is rejected.
    #[serde(default)]
    pub(crate) y_axis: Option<YAxisOutput>,
    /// Feed for a `G1`/`G2`/`G3` that changes C. `per_minute` writes the IR feed. `inverse_time`
    /// writes `G93` and `F = 1 / minutes`.
    #[serde(default = "default_c_feed")]
    pub(crate) c_feed: String,
    /// Second work spindle. Absent: a binding other than `main` is rejected.
    ///
    /// `select` is written before the first output of a toolpath that enters this spindle.
    /// `deselect` is written when a later toolpath leaves it, and again before the home lines
    /// when the program ends on it. `mirror_z` negates Z (and arc K, and swaps G2/G3) for moves
    /// on this spindle. Haas leaves `mirror_z` false because G14 mirrors Z on the control.
    #[serde(default)]
    pub(crate) sub_spindle: Option<SubSpindleOutput>,
    /// Part transfer between `main` and [`TransferOutput::spindle`].
    ///
    /// Absent: chuck, sync, spindle-run and spindle-move events are rejected.
    #[serde(default)]
    pub(crate) transfer: Option<TransferOutput>,
    /// Tailstock advance and retract lines. Absent: tailstock events are rejected.
    #[serde(default)]
    pub(crate) tailstock: Option<TailstockOutput>,
    /// Steady-rest clamp and unclamp lines. Absent: steady-rest events are rejected.
    #[serde(default)]
    pub(crate) steady_rest: Option<SteadyRestOutput>,
    /// Parts catcher extend and retract lines. Absent: parts-catcher events are rejected.
    #[serde(default)]
    pub(crate) parts_catcher: Option<PartsCatcherOutput>,
    /// Bar-feed command written after the home lines. Absent: bar-feed events are rejected.
    #[serde(default)]
    pub(crate) bar_feed: Option<BarFeedOutput>,
    /// Two or more channel sections. Absent: a binding whose channel is not 0 is rejected.
    #[serde(default)]
    pub(crate) channels: Option<ChannelsOutput>,
    /// Tilting B head. Absent: a [`campanion_ir::Event::HeadIndex`] is rejected.
    ///
    /// `index` may use `{b}`, written with the signed lathe number format. `mode` `tip` writes the
    /// tool tip. `mode` `pivot` adds `pivot_length * ((cos B - 1), sin B)` to `(radius, Z)` while
    /// B is not 0. Diameter mode doubles that radius part.
    #[serde(default)]
    pub(crate) b_head: Option<BHeadOutput>,
}

/// Lines that index a tilting B head, and how X and Z relate to the tool tip.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BHeadOutput {
    /// Lines written for [`campanion_ir::Event::HeadIndex`]. `{b}` is the angle in degrees.
    pub(crate) index: Vec<String>,
    /// `tip` or `pivot`.
    pub(crate) mode: String,
    /// Pivot-to-tip length along the tool axis, millimetres. Zero leaves X and Z on the tip.
    #[serde(default)]
    pub(crate) pivot_length: f64,
}

/// Radius and Z added to a tip while a pivot-mode head sits at `b_deg`.
///
/// `B = 0` and `mode` `tip` and a zero length all add nothing. At 90° the shift is `(-length, length)`.
pub(crate) fn b_pivot_offset(b_deg: f64, mode: &str, pivot_length: f64) -> (f64, f64) {
    if mode != "pivot" || b_deg == 0.0 || pivot_length == 0.0 {
        return (0.0, 0.0);
    }
    let radians = b_deg.to_radians();
    (pivot_length * (radians.cos() - 1.0), pivot_length * radians.sin())
}

/// How a lathe post splits a program across channels.
///
/// `count` is at least 2. `start` may use `{channel}`, written as the 1-based channel number.
/// `wait` may use `{code}`, which is `wait_base` plus the sync id. `end` takes no placeholders.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChannelsOutput {
    /// How many channel sections this post writes. At least 2.
    pub count: u16,
    /// Lines before one channel's toolpaths. `{channel}` is 1-based.
    pub start: Vec<String>,
    /// Lines after one channel's toolpaths.
    pub end: Vec<String>,
    /// Lines for one [`campanion_ir::Event::Sync`]. `{code}` is `wait_base + id`.
    pub wait: Vec<String>,
    /// Added to the sync id to form `{code}`.
    pub wait_base: u32,
}

/// Advance and retract lines for a lathe tailstock.
///
/// Both lists are bare. Hold force and travel are machine settings, not placeholders.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct TailstockOutput {
    /// Written for [`campanion_ir::Event::Tailstock`] with `engage` true.
    pub(crate) advance: Vec<String>,
    /// Written for [`campanion_ir::Event::Tailstock`] with `engage` false.
    pub(crate) retract: Vec<String>,
}

/// Clamp and unclamp lines for a lathe steady rest.
///
/// Both lists are bare. Tow-along P positions are not placeholders; the rest is positioned in setup.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SteadyRestOutput {
    /// Written for [`campanion_ir::Event::SteadyRest`] with `clamp` true.
    pub(crate) clamp: Vec<String>,
    /// Written for [`campanion_ir::Event::SteadyRest`] with `clamp` false.
    pub(crate) unclamp: Vec<String>,
}

/// Extend and retract lines for a lathe parts catcher.
///
/// Both lists are bare. The event carries no position or part size.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct PartsCatcherOutput {
    /// Written for [`campanion_ir::Event::PartsCatcher`] with `extend` true.
    pub(crate) extend: Vec<String>,
    /// Written for [`campanion_ir::Event::PartsCatcher`] with `extend` false.
    pub(crate) retract: Vec<String>,
}

/// One bar-feed line, written after [`LatheOutput::home`].
///
/// `code` is a bare word. The two address words are single uppercase letters. Lengths use the
/// same format as other length words of the program. The line carries no comment.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BarFeedOutput {
    /// Command, for example `G105`.
    pub(crate) code: String,
    /// Word for the part length plus cutoff and facing allowance.
    pub(crate) part_length_word: String,
    /// Word for the minimum clamping length.
    pub(crate) min_clamp_word: String,
}

/// Chuck, sync, secondary-spindle run and housing-move lines for a part transfer.
///
/// `{rpm}` is allowed on `sub_cw` and `sub_ccw`. `{phase}` is allowed on `sync_on_phase`.
/// `{z}` is allowed on every move list. `{feed}` is allowed on `move_feed`. Every list may be
/// empty, and then that event writes nothing. `spindle` is the IR name served by the sub lines.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct TransferOutput {
    /// IR spindle name these secondary lines serve, for example `sub`.
    pub(crate) spindle: String,
    /// Clamp the main chuck.
    pub(crate) clamp_main: Vec<String>,
    /// Unclamp the main chuck.
    pub(crate) unclamp_main: Vec<String>,
    /// Clamp the secondary chuck.
    pub(crate) clamp_sub: Vec<String>,
    /// Unclamp the secondary chuck.
    pub(crate) unclamp_sub: Vec<String>,
    /// Secondary spindle forward. `{rpm}` is the speed.
    pub(crate) sub_cw: Vec<String>,
    /// Secondary spindle reverse. `{rpm}` is the speed.
    pub(crate) sub_ccw: Vec<String>,
    /// Secondary spindle stop.
    pub(crate) sub_stop: Vec<String>,
    /// Engage synchronous spindle control with no phase.
    pub(crate) sync_on: Vec<String>,
    /// Engage synchronous spindle control at `{phase}` degrees.
    pub(crate) sync_on_phase: Vec<String>,
    /// Cancel synchronous spindle control.
    pub(crate) sync_off: Vec<String>,
    /// Rapid the spindle housing. `{z}` is the position.
    pub(crate) move_rapid: Vec<String>,
    /// Feed the spindle housing. `{z}` is the position and `{feed}` is the feed.
    pub(crate) move_feed: Vec<String>,
    /// Rapid the spindle housing in machine coordinates. `{z}` is the position.
    pub(crate) move_machine: Vec<String>,
}

fn default_c_feed() -> String {
    "per_minute".to_string()
}

impl LatheOutput {
    pub(crate) fn c_feed_inverse(&self) -> bool {
        self.c_feed == "inverse_time"
    }

    /// Word that leaves inverse time. Empty `feed_mode` means `G94`.
    pub(crate) fn c_feed_off_word(&self) -> String {
        if self.feed_mode.is_empty() { "G94".to_string() } else { self.feed_mode.clone() }
    }

    pub(crate) fn c_feed_off_code(&self) -> Option<u16> {
        g_code_number(&self.c_feed_off_word())
    }
}

/// Select and deselect lines for one secondary spindle.
///
/// `spindle` is the IR spindle name these lines serve. `select` and `deselect` take no
/// placeholders, the same rule as [`LatheOutput::spindle_off`].
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct SubSpindleOutput {
    /// IR spindle name, for example `sub`.
    pub(crate) spindle: String,
    /// Written when a toolpath enters this spindle, before its tool change.
    pub(crate) select: Vec<String>,
    /// Written when a toolpath leaves this spindle, and before home if the program ends on it.
    pub(crate) deselect: Vec<String>,
    /// When true, the writer negates Z and arc K and swaps G2/G3. Default false.
    #[serde(default)]
    pub(crate) mirror_z: bool,
}

/// Y-axis live milling on a lathe.
///
/// `letter` is the modal cross-axis word. It is never doubled in diameter mode. `face_plane` is
/// written before an axial tool mills in XY. `side_plane` is written before a radial tool mills
/// in YZ. `turn_plane` is written when that milling ends, before C disengages or the next
/// turning move. Haas uses `Y`, `G17`, `G19` and `G18`, and no activation code.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct YAxisOutput {
    /// One uppercase ASCII letter, for example `Y`.
    pub(crate) letter: String,
    /// Plane word for face milling (axial tool, XY).
    pub(crate) face_plane: String,
    /// Plane word for side milling (radial tool, YZ).
    pub(crate) side_plane: String,
    /// Plane word restored for turning.
    pub(crate) turn_plane: String,
}

/// Engage and disengage a spindle as a positioning axis.
///
/// `letter` is the modal word on `G0` and `G1` while the axis is engaged. `engage` and
/// `disengage` take no placeholders and may be empty. An empty list writes nothing.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CAxisOutput {
    /// One uppercase ASCII letter, for example `C`.
    pub(crate) letter: String,
    /// Written when the spindle becomes a positioning axis.
    pub(crate) engage: Vec<String>,
    /// Written when the spindle returns to turning. Clears the modal C word.
    pub(crate) disengage: Vec<String>,
}

/// Clamp or release the spindle brake.
///
/// Both line lists take no placeholders and may be empty. An empty list writes nothing.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BrakeOutput {
    /// Written to clamp the brake.
    pub(crate) clamp: Vec<String>,
    /// Written to release the brake.
    pub(crate) release: Vec<String>,
}

/// Start and stop a live (driven) tool.
///
/// `forward`, `reverse` and `stop` must each contain at least one line. `forward` and `reverse`
/// may use `{rpm}`, written as a whole number. `stop` and `feed_mode` take no placeholders.
/// `feed_mode` is written once before the first start when it is non-empty and differs from
/// [`LatheOutput::feed_mode`].
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct LiveToolOutput {
    /// Live tool forward. `{rpm}` is the speed.
    pub(crate) forward: Vec<String>,
    /// Live tool reverse. `{rpm}` is the speed.
    pub(crate) reverse: Vec<String>,
    /// Live tool stop.
    pub(crate) stop: Vec<String>,
    /// Feed-mode word for live tooling, such as `G98`.
    pub(crate) feed_mode: String,
}

/// Cartesian face interpolation.
///
/// `on` and `off` take no placeholders. `x_diameter` selects whether X words in this mode are
/// diameters. Haas G112 writes a radius, so `x_diameter` is false there.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct FaceInterpOutput {
    /// Written when face interpolation turns on.
    pub(crate) on: Vec<String>,
    /// Written when face interpolation turns off.
    pub(crate) off: Vec<String>,
    /// When true, X is a diameter. When false, X is the Cartesian radius stored on the pose.
    pub(crate) x_diameter: bool,
}

/// Cylindrical interpolation along an unrolled circumference.
///
/// `on` may use `{radius}` and `{diameter}`, both formatted as linear numbers. `off` takes no
/// placeholders.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct CylInterpOutput {
    /// Written when cylinder interpolation turns on.
    pub(crate) on: Vec<String>,
    /// Written when cylinder interpolation turns off.
    pub(crate) off: Vec<String>,
}

fn default_z_mode() -> String {
    "keep".to_string()
}

fn default_process_cycle() -> String {
    "hold".to_string()
}

impl PostDef {
    pub(crate) fn checked(mut self) -> Result<Self> {
        check_token("id", &self.id, 64)?;
        check_text("name", &self.name, 1, 128)?;
        check_text("description", &self.description, 0, 1024)?;
        check_references(&self.references)?;
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
        if !matches!(self.process_cycle.as_str(), "hold" | "per_cut") {
            return Err(field("process_cycle", "must be hold or per_cut"));
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
        check_five_axis(&self)?;
        Ok(self)
    }

    /// True when this post writes a turning program.
    pub fn is_lathe(&self) -> bool {
        self.lathe.is_some()
    }

    /// True when this post writes indexed 3+2 (`five_axis`).
    pub fn has_five_axis(&self) -> bool {
        self.five_axis.is_some()
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

    /// True when the spindle-on template substitutes `{speed}`.
    pub fn spindle_on_needs_speed(&self) -> bool {
        self.process.get("spindle").is_some_and(|words| words.on.iter().any(|line| line.contains("{speed}")))
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

    /// True when every running process turns off before a rapid and on again before the next feed.
    pub(crate) fn cycles_per_cut(&self) -> bool {
        self.process_cycle == "per_cut"
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
    if let Some(thread) = &lathe.thread {
        check_thread_output(thread, lathe)?;
    }
    if !matches!(lathe.c_feed.as_str(), "per_minute" | "inverse_time") {
        return Err(field("lathe.c_feed", "must be per_minute or inverse_time"));
    }
    if let Some(c_axis) = &lathe.c_axis {
        check_c_letter(&c_axis.letter)?;
        // Empty engage or disengage writes nothing. LinuxCNC fills these in HAL or a user M-code.
        check_templates("lathe.c_axis.engage", &c_axis.engage, LATHE_BARE)?;
        check_templates("lathe.c_axis.disengage", &c_axis.disengage, LATHE_BARE)?;
    }
    if let Some(brake) = &lathe.brake {
        check_templates("lathe.brake.clamp", &brake.clamp, LATHE_BARE)?;
        check_templates("lathe.brake.release", &brake.release, LATHE_BARE)?;
    }
    if let Some(live) = &lathe.live_tool {
        require_lines("lathe.live_tool.forward", &live.forward)?;
        require_lines("lathe.live_tool.reverse", &live.reverse)?;
        require_lines("lathe.live_tool.stop", &live.stop)?;
        check_templates("lathe.live_tool.forward", &live.forward, LATHE_RPM)?;
        check_templates("lathe.live_tool.reverse", &live.reverse, LATHE_RPM)?;
        check_templates("lathe.live_tool.stop", &live.stop, LATHE_BARE)?;
        check_templates("lathe.live_tool.feed_mode", std::slice::from_ref(&live.feed_mode), LATHE_BARE)?;
    }
    if let Some(face) = &lathe.face_interp {
        check_templates("lathe.face_interp.on", &face.on, LATHE_BARE)?;
        check_templates("lathe.face_interp.off", &face.off, LATHE_BARE)?;
    }
    if let Some(cylinder) = &lathe.cylinder_interp {
        check_templates("lathe.cylinder_interp.on", &cylinder.on, LATHE_CYLINDER)?;
        check_templates("lathe.cylinder_interp.off", &cylinder.off, LATHE_BARE)?;
    }
    if let Some(y_axis) = &lathe.y_axis {
        let c_letter = lathe.c_axis.as_ref().map(|c_axis| c_axis.letter.as_str());
        check_y_letter(&y_axis.letter, c_letter)?;
        check_plane_word("lathe.y_axis.face_plane", &y_axis.face_plane)?;
        check_plane_word("lathe.y_axis.side_plane", &y_axis.side_plane)?;
        check_plane_word("lathe.y_axis.turn_plane", &y_axis.turn_plane)?;
    }
    if let Some(sub) = &lathe.sub_spindle {
        check_token("lathe.sub_spindle.spindle", &sub.spindle, 64)?;
        check_templates("lathe.sub_spindle.select", &sub.select, LATHE_BARE)?;
        check_templates("lathe.sub_spindle.deselect", &sub.deselect, LATHE_BARE)?;
    }
    if let Some(transfer) = &lathe.transfer {
        check_token("lathe.transfer.spindle", &transfer.spindle, 64)?;
        check_templates("lathe.transfer.clamp_main", &transfer.clamp_main, LATHE_BARE)?;
        check_templates("lathe.transfer.unclamp_main", &transfer.unclamp_main, LATHE_BARE)?;
        check_templates("lathe.transfer.clamp_sub", &transfer.clamp_sub, LATHE_BARE)?;
        check_templates("lathe.transfer.unclamp_sub", &transfer.unclamp_sub, LATHE_BARE)?;
        check_templates("lathe.transfer.sub_cw", &transfer.sub_cw, LATHE_RPM)?;
        check_templates("lathe.transfer.sub_ccw", &transfer.sub_ccw, LATHE_RPM)?;
        check_templates("lathe.transfer.sub_stop", &transfer.sub_stop, LATHE_BARE)?;
        check_templates("lathe.transfer.sync_on", &transfer.sync_on, LATHE_BARE)?;
        check_templates("lathe.transfer.sync_on_phase", &transfer.sync_on_phase, LATHE_PHASE)?;
        check_templates("lathe.transfer.sync_off", &transfer.sync_off, LATHE_BARE)?;
        check_templates("lathe.transfer.move_rapid", &transfer.move_rapid, LATHE_MOVE)?;
        check_templates("lathe.transfer.move_feed", &transfer.move_feed, LATHE_MOVE_FEED)?;
        check_templates("lathe.transfer.move_machine", &transfer.move_machine, LATHE_MOVE)?;
    }
    if let Some(tailstock) = &lathe.tailstock {
        if tailstock.advance.is_empty() {
            return Err(field("lathe.tailstock.advance", "must not be empty"));
        }
        if tailstock.retract.is_empty() {
            return Err(field("lathe.tailstock.retract", "must not be empty"));
        }
        check_templates("lathe.tailstock.advance", &tailstock.advance, LATHE_BARE)?;
        check_templates("lathe.tailstock.retract", &tailstock.retract, LATHE_BARE)?;
    }
    if let Some(steady_rest) = &lathe.steady_rest {
        if steady_rest.clamp.is_empty() {
            return Err(field("lathe.steady_rest.clamp", "must not be empty"));
        }
        if steady_rest.unclamp.is_empty() {
            return Err(field("lathe.steady_rest.unclamp", "must not be empty"));
        }
        check_templates("lathe.steady_rest.clamp", &steady_rest.clamp, LATHE_BARE)?;
        check_templates("lathe.steady_rest.unclamp", &steady_rest.unclamp, LATHE_BARE)?;
    }
    if let Some(parts_catcher) = &lathe.parts_catcher {
        if parts_catcher.extend.is_empty() {
            return Err(field("lathe.parts_catcher.extend", "must not be empty"));
        }
        if parts_catcher.retract.is_empty() {
            return Err(field("lathe.parts_catcher.retract", "must not be empty"));
        }
        check_templates("lathe.parts_catcher.extend", &parts_catcher.extend, LATHE_BARE)?;
        check_templates("lathe.parts_catcher.retract", &parts_catcher.retract, LATHE_BARE)?;
    }
    if let Some(bar_feed) = &lathe.bar_feed {
        check_bare_code("lathe.bar_feed.code", &bar_feed.code)?;
        check_one_uppercase_letter("lathe.bar_feed.part_length_word", &bar_feed.part_length_word)?;
        check_one_uppercase_letter("lathe.bar_feed.min_clamp_word", &bar_feed.min_clamp_word)?;
    }
    if let Some(channels) = &lathe.channels {
        if channels.count < 2 {
            return Err(field("lathe.channels.count", "channels.count must be at least 2"));
        }
        check_templates("lathe.channels.start", &channels.start, &["channel", "date-free!"])?;
        check_templates("lathe.channels.end", &channels.end, LATHE_BARE)?;
        check_templates("lathe.channels.wait", &channels.wait, &["code", "date-free!"])?;
    }
    if let Some(b_head) = &lathe.b_head {
        if !matches!(b_head.mode.as_str(), "tip" | "pivot") {
            return Err(field("lathe.b_head.mode", "must be tip or pivot"));
        }
        if !(b_head.pivot_length.is_finite() && b_head.pivot_length >= 0.0) {
            return Err(field("lathe.b_head.pivot_length", "must be zero or positive"));
        }
        check_templates("lathe.b_head.index", &b_head.index, LATHE_B)?;
    }
    Ok(())
}

fn require_lines(field_name: &str, lines: &[String]) -> Result<()> {
    if lines.is_empty() {
        return Err(field(field_name, "needs at least one line"));
    }
    Ok(())
}

/// A non-empty word with no whitespace, comment parenthesis or semicolon, and no unknown placeholder.
fn check_bare_code(field_name: &str, text: &str) -> Result<()> {
    if text.is_empty() {
        return Err(field(field_name, "must not be empty"));
    }
    if text.chars().any(|ch| ch.is_whitespace() || ch == '(' || ch == ';') {
        return Err(field(field_name, "must be a bare word"));
    }
    let lines = [text.to_string()];
    check_templates(field_name, &lines, LATHE_BARE)
}

/// One ASCII uppercase letter. Empty, lowercase and multi-character values share one message.
fn check_one_uppercase_letter(field_name: &str, text: &str) -> Result<()> {
    let mut chars = text.chars();
    match chars.next() {
        Some(ch) if chars.next().is_none() && ch.is_ascii_uppercase() => Ok(()),
        _ => Err(field(field_name, "must be one uppercase letter")),
    }
}

fn check_plane_word(field_name: &str, word: &str) -> Result<()> {
    if word.is_empty() {
        return Err(field(field_name, "must not be empty"));
    }
    let lines = [word.to_string()];
    check_templates(field_name, &lines, LATHE_BARE)
}

/// `Y` is allowed. Axis letters already used by X and Z, arc words, and the C letter are not.
fn check_y_letter(letter: &str, c_letter: Option<&str>) -> Result<()> {
    const MESSAGE: &str = "must be one uppercase ASCII letter and not X, Z, I, J, K, F, S, T, M, G, N, O, P, or the C-axis letter";
    const BANNED: &str = "XZIJKFSTMGNOP";
    let mut chars = letter.chars();
    let Some(ch) = chars.next() else {
        return Err(field("lathe.y_axis.letter", MESSAGE));
    };
    let c_hit = c_letter.is_some_and(|c| c.starts_with(ch));
    if chars.next().is_some() || !ch.is_ascii_uppercase() || BANNED.contains(ch) || c_hit {
        return Err(field("lathe.y_axis.letter", MESSAGE));
    }
    Ok(())
}

/// `C` is allowed. Axis letters and the words a block already uses are not.
fn check_c_letter(letter: &str) -> Result<()> {
    const MESSAGE: &str = "must be one uppercase ASCII letter and not X, Y, Z, I, J, K, F, S, T, M, G, N, O, P";
    const BANNED: &str = "XYZIJKFSTMGNOP";
    let mut chars = letter.chars();
    let Some(ch) = chars.next() else {
        return Err(field("lathe.c_axis.letter", MESSAGE));
    };
    if chars.next().is_some() || !ch.is_ascii_uppercase() || BANNED.contains(ch) {
        return Err(field("lathe.c_axis.letter", MESSAGE));
    }
    Ok(())
}

fn check_references(refs: &[String]) -> Result<()> {
    if refs.len() > 32 {
        return Err(field("references", "at most 32 entries"));
    }
    for entry in refs {
        let count = entry.chars().count();
        if !(1..=256).contains(&count) {
            return Err(field("references", "each entry must be 1..=256 characters"));
        }
        if entry.chars().any(char::is_control) {
            return Err(field("references", "contains a control character"));
        }
    }
    Ok(())
}

fn check_thread_output(thread: &ThreadOutput, lathe: &LatheOutput) -> Result<()> {
    if g_code_number(&thread.code).is_none() {
        return Err(field("lathe.thread.code", "must be a G code such as G33"));
    }
    let mut chars = thread.lead_word.chars();
    let Some(letter) = chars.next() else {
        return Err(field("lathe.thread.lead_word", "must be one uppercase ASCII letter"));
    };
    if chars.next().is_some() || !letter.is_ascii_uppercase() {
        return Err(field("lathe.thread.lead_word", "must be one uppercase ASCII letter"));
    }
    if !thread.feed_mode.is_empty() {
        check_templates("lathe.thread.feed_mode", std::slice::from_ref(&thread.feed_mode), LATHE_BARE)?;
        if g_code_number(&thread.feed_mode).is_none() {
            return Err(field("lathe.thread.feed_mode", "must be a G code such as G99"));
        }
        if lathe.feed_mode.is_empty() {
            return Err(field("lathe.thread.feed_mode", "needs lathe.feed_mode to return to"));
        }
        if thread.feed_mode == lathe.feed_mode {
            return Err(field("lathe.thread.feed_mode", "must differ from lathe.feed_mode"));
        }
    }
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

/// Indexed 3+2. `mode` `work` rotates the part tip about the origin. `mode` `machine` writes
/// the gauge point from the kinematics inverse. `{rotaries}` is the only index placeholder.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct FiveAxisOutput {
    /// `work` or `machine`.
    pub(crate) mode: FiveAxisMode,
    /// Digits on each rotary word.
    pub(crate) decimals: u8,
    /// Machine retract before a rotary index. No placeholders.
    pub(crate) retract: Vec<String>,
    /// Rotary index. Every line contains `{rotaries}`.
    pub(crate) index: Vec<String>,
    /// Frame on, after the index. Empty writes nothing. No placeholders.
    pub(crate) on: Vec<String>,
    /// Frame off, before the next index. Empty writes nothing. No placeholders.
    pub(crate) off: Vec<String>,
}

/// Where a 3+2 post reads coordinates while the frame is active.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum FiveAxisMode {
    /// Part tip rotated by the table angles about the work origin. Pivots are left to the control.
    Work,
    /// Gauge point from [`campanion_ir::Kinematics::inverse`] with tool length 0.
    Machine,
}

fn check_five_axis(post: &PostDef) -> Result<()> {
    let Some(five) = &post.five_axis else {
        return Ok(());
    };
    if five.decimals > 8 {
        return Err(field("five_axis.decimals", "must be 0..=8"));
    }
    check_templates("five_axis.retract", &five.retract, &[])?;
    check_templates("five_axis.on", &five.on, &[])?;
    check_templates("five_axis.off", &five.off, &[])?;
    if five.index.is_empty() {
        return Err(field("five_axis.index", "needs at least one line"));
    }
    check_templates("five_axis.index", &five.index, &["rotaries"])?;
    for line in &five.index {
        if !line.contains("{rotaries}") {
            return Err(field("five_axis.index", "requires {rotaries}"));
        }
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
