//! Modal replay of a five-axis post.
//!
//! [`audit_five`] reads the text a five-axis post wrote and tracks the mill modal state plus the
//! work frame, TCP, tool-length offset, inverse time, rotary words, and `G53` machine retracts.
//! `G53` is not a motion and does not change the modal position or the motion mode. Rotary words
//! and inverse-time codes on that line still update, matching [`crate::read_back`].
//! `G91.1` confirms incremental IJK without changing positioning; `G90.1` is rejected.
//!
//! An index block is rotary words with no `X` `Y` `Z`. It is not a motion. LinuxCNC leaves the
//! work-frame and TCP hooks empty, so those flags stay off; the machine-mode transform and the
//! part-frame tip are properties of the coordinates, checked by the audit tests.
//!
//! A `decimal_point` post must put a decimal point on `A`, `B`, and `C` as well as on `X` `Y` `Z`.
//! A post with `digital_out` accepts `M64` and `M65` when `P` is a declared auxiliary pin, the
//! same rule as [`crate::audit`]. Those codes do not change motion or modal state.

use campanion_ir::{ArcSense, Plane};

use crate::audit::{
    ArcReplay, AuditReport, CoolantState, DistanceMode, FeedMode, LengthOffset, MotionMode, MotionState, SpindleState, UnitMode, WorkOffset,
};
use crate::def::{FiveAxisMode, PostDef};
use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};
use crate::read::{ReadMove, read_back};

const MAX_ABS: f64 = 1.0e9;
const LINE_NUMBER_LIMIT: u64 = 1_000_000_000;

/// One motion block after modal five-axis state has been applied.
#[derive(Clone, Debug, PartialEq)]
pub struct FiveMotion {
    /// 1-based line number in the posted text.
    pub line: usize,
    /// Motion mode in effect for this block.
    pub mode: MotionMode,
    /// Selected plane.
    pub plane: Option<Plane>,
    /// Distance mode.
    pub distance: DistanceMode,
    /// Units.
    pub units: UnitMode,
    /// Feed mode after this block.
    pub feed_mode: FeedMode,
    /// Active feed, including a modal `F` from an earlier block.
    pub feed: Option<f64>,
    /// This block carried its own `F` word.
    pub feed_on_block: bool,
    /// Work offset. `None` when the program has not written `G54`–`G59`.
    pub work: Option<WorkOffset>,
    /// Tool in the spindle.
    pub tool: Option<u32>,
    /// Tool-length offset. `G234` cancels it. `G43 H` selects it.
    pub length: LengthOffset,
    /// Spindle, torch, or beam.
    pub spindle: SpindleState,
    /// Coolant.
    pub coolant: CoolantState,
    /// Absolute end. An omitted axis stays at the previous value.
    pub end: [f64; 3],
    /// Which of `X` `Y` `Z` this block wrote.
    pub axes_written: [bool; 3],
    /// Set on `G2` and `G3`.
    pub arc: Option<ArcReplay>,
    /// Work frame after this block. `G254` turns it on and `G255` turns it off.
    ///
    /// Empty hooks (LinuxCNC machine mode) leave this false. The transform is then the posted
    /// coordinates themselves.
    pub work_frame: bool,
    /// TCP after this block. `G234` turns it on. `G49` and `G43` turn it off.
    ///
    /// Empty TCP hooks leave this false.
    pub tcp: bool,
    /// `H` word on the TCP-on block, while TCP is still on.
    pub tcp_h: Option<u32>,
    /// Modal `A` `B` `C` after this block. `None` until that letter is written.
    pub rotaries: [Option<f64>; 3],
    /// Which of `A` `B` `C` this block wrote.
    pub rotary_written: [bool; 3],
}

/// A rotary index: `A`/`B`/`C` and no linear axis. Not a part-frame move.
#[derive(Clone, Debug, PartialEq)]
pub struct FiveIndex {
    /// 1-based line number.
    pub line: usize,
    /// Modal `A` `B` `C` after this block.
    pub rotaries: [Option<f64>; 3],
    /// Which of `A` `B` `C` this block wrote.
    pub written: [bool; 3],
    /// Work frame after this block. The frame-on code is the following line, so this is still off
    /// when the post writes `G254` after the index.
    pub work_frame: bool,
    /// TCP after this block.
    pub tcp: bool,
}

/// A `G53` machine retract. The explicit axis words are machine coordinates, not the modal position.
#[derive(Clone, Debug, PartialEq)]
pub struct FiveRetract {
    /// 1-based line number.
    pub line: usize,
    /// Axis words on the `G53` line. An omitted axis is `None`.
    pub axes: [Option<f64>; 3],
}

/// Modal replay of one five-axis program.
#[derive(Clone, Debug, PartialEq)]
pub struct FiveAudit {
    /// Motion blocks, in order. `G53` and rotary-only index lines are not included.
    pub motions: Vec<FiveMotion>,
    /// Rotary-only index lines, in order.
    pub indexes: Vec<FiveIndex>,
    /// `G53` lines, in order.
    pub retracts: Vec<FiveRetract>,
    /// Spindle after the last line.
    pub spindle: SpindleState,
    /// Coolant after the last line.
    pub coolant: CoolantState,
    /// Tool left in the spindle.
    pub tool: Option<u32>,
    /// Units, once `G20` or `G21` has been seen.
    pub units: Option<UnitMode>,
    /// Work offset, or `None` when the post never selects one.
    pub work: Option<WorkOffset>,
    /// Distance mode, once `G90` or `G91` has been seen.
    pub distance: Option<DistanceMode>,
    /// Tool-length offset after the last line.
    pub length: LengthOffset,
    /// Feed mode after the last line.
    pub feed_mode: FeedMode,
    /// Work frame after the last line.
    pub work_frame: bool,
    /// TCP after the last line.
    pub tcp: bool,
    /// `H` from the TCP-on block, while TCP is still on.
    pub tcp_h: Option<u32>,
    /// The post's work-frame on template names a `G` code (`G254`).
    pub frame_coded: bool,
    /// The post's TCP on template names a `G` code (`G234`).
    pub tcp_coded: bool,
    /// The post has `five_axis.continuous.tcp`, including empty LinuxCNC hooks.
    pub tcp_configured: bool,
    /// `five_axis.mode` is `machine`. Work mode is the table rotation about the origin.
    pub machine_mode: bool,
    /// `five_axis.continuous.chord_tol`, millimetres. `0.005` when the post has no continuous block.
    pub chord_tol: f64,
    /// `five_axis.continuous.axis_tol_deg`. `0.1` when the post has no continuous block.
    pub axis_tol_deg: f64,
    /// Digits on `A` `B` `C`.
    pub rotary_decimals: u8,
    /// Digits on `X` `Y` `Z`.
    pub linear_decimals: u8,
    /// Digits on `F`.
    pub feed_decimals: u8,
    /// TCP rapid feed, millimetres per minute. `5000` when the post has no TCP block.
    pub rapid_feed: f64,
}

/// Replay `text` as the five-axis control named by `post` would read it.
///
/// The post must have `five_axis`. Unknown words, a feed move before a positive `F`, and an
/// inverse-time feed block with no `F` are errors. `G53` is recorded in [`FiveAudit::retracts`]
/// and is not a motion.
pub fn audit_five(text: &str, post: &PostDef) -> Result<FiveAudit> {
    if !post.has_five_axis() {
        return Err(invalid("audit_five reads five-axis posts"));
    }
    if text.len() > 64 * 1024 * 1024 {
        return Err(invalid("g-code is too large"));
    }
    let mut parser = Parser::new(post)?;
    for (index, line) in text.lines().enumerate() {
        if index > 2_000_000 {
            return Err(invalid("g-code has too many lines"));
        }
        parser.line(index + 1, line)?;
    }
    parser.cross_check(text, post)?;
    Ok(parser.report())
}

pub(crate) fn audit_as_mill(text: &str, post: &PostDef) -> Result<AuditReport> {
    let five = audit_five(text, post)?;
    let motions = five
        .motions
        .iter()
        .map(|motion| MotionState {
            line: motion.line,
            mode: motion.mode,
            plane: motion.plane,
            distance: motion.distance,
            units: motion.units,
            feed_mode: motion.feed_mode,
            feed: motion.feed,
            feed_on_block: motion.feed_on_block,
            work: motion.work,
            tool: motion.tool,
            length: motion.length,
            spindle: motion.spindle,
            coolant: motion.coolant,
            end: motion.end,
            arc: motion.arc,
            rotary: motion.rotaries[0],
        })
        .collect();
    Ok(AuditReport {
        motions,
        spindle: five.spindle,
        coolant: five.coolant,
        tool: five.tool,
        units: five.units,
        work: five.work,
        distance: five.distance,
        length: five.length,
        feed_mode: five.feed_mode,
    })
}

struct Word {
    letter: char,
    value: f64,
    raw: String,
}

struct Codes {
    frame_on: Vec<u16>,
    frame_off: Vec<u16>,
    tcp_on: Vec<u16>,
    tcp_off: Vec<u16>,
    tcp_on_wants_h: bool,
}

struct Parser<'a> {
    post: &'a PostDef,
    codes: Codes,
    frame_coded: bool,
    tcp_coded: bool,
    tcp_configured: bool,
    machine_mode: bool,
    chord_tol: f64,
    axis_tol_deg: f64,
    rotary_decimals: u8,
    rapid_feed: f64,
    pos: Xyz,
    motion: Option<MotionMode>,
    plane: Option<Plane>,
    distance: Option<DistanceMode>,
    units: Option<UnitMode>,
    feed_mode: FeedMode,
    feed: Option<f64>,
    work: Option<WorkOffset>,
    pending_tool: Option<u32>,
    tool: Option<u32>,
    length: LengthOffset,
    spindle: SpindleState,
    coolant: CoolantState,
    work_frame: bool,
    tcp: bool,
    tcp_h: Option<u32>,
    rotaries: [Option<f64>; 3],
    next_number: Option<u64>,
    tool_patterns: Vec<(String, String)>,
    motions: Vec<FiveMotion>,
    indexes: Vec<FiveIndex>,
    retracts: Vec<FiveRetract>,
}

struct Parsed {
    g: Vec<u16>,
    m: Vec<u16>,
    x: Option<f64>,
    y: Option<f64>,
    z: Option<f64>,
    i: Option<f64>,
    j: Option<f64>,
    k: Option<f64>,
    r: Option<f64>,
    f: Option<f64>,
    s: Option<f64>,
    p: Option<f64>,
    h: Option<u32>,
    tool: Option<u32>,
    rotaries: [Option<f64>; 3],
    axes_written: [bool; 3],
    rotary_written: [bool; 3],
}

impl<'a> Parser<'a> {
    fn new(post: &'a PostDef) -> Result<Self> {
        let five = post.five_axis.as_ref().ok_or_else(|| invalid("audit_five reads five-axis posts"))?;
        let codes = Codes {
            frame_on: template_g_codes(&five.on),
            frame_off: template_g_codes(&five.off),
            tcp_on: five
                .continuous
                .as_ref()
                .map(|cont| cont.tcp.as_ref().map(|tcp| template_g_codes(&tcp.on)).unwrap_or_default())
                .unwrap_or_default(),
            tcp_off: five
                .continuous
                .as_ref()
                .map(|cont| cont.tcp.as_ref().map(|tcp| template_g_codes(&tcp.off)).unwrap_or_default())
                .unwrap_or_default(),
            tcp_on_wants_h: five.continuous.as_ref().and_then(|cont| cont.tcp.as_ref()).is_some_and(|tcp| template_has_h(&tcp.on)),
        };
        let frame_coded = !codes.frame_on.is_empty();
        let tcp_coded = !codes.tcp_on.is_empty();
        let continuous = five.continuous.as_ref();
        let tcp = continuous.and_then(|cont| cont.tcp.as_ref());
        let next_number = post.line_numbers.enabled.then_some(u64::from(post.line_numbers.start));
        Ok(Self {
            post,
            frame_coded,
            tcp_coded,
            tcp_configured: tcp.is_some(),
            machine_mode: five.mode == FiveAxisMode::Machine,
            chord_tol: continuous.map(|cont| cont.chord_tol).unwrap_or(0.005),
            axis_tol_deg: continuous.map(|cont| cont.axis_tol_deg).unwrap_or(0.1),
            rotary_decimals: five.decimals,
            rapid_feed: tcp.map(|tcp| tcp.rapid_feed).unwrap_or(5000.0),
            codes,
            pos: [0.0, 0.0, 0.0],
            motion: None,
            plane: None,
            distance: None,
            units: None,
            feed_mode: FeedMode::PerMinute,
            feed: None,
            work: None,
            pending_tool: None,
            tool: None,
            length: LengthOffset::Unset,
            spindle: SpindleState::off(),
            coolant: CoolantState::Off,
            work_frame: false,
            tcp: false,
            tcp_h: None,
            rotaries: [None; 3],
            next_number,
            tool_patterns: tool_patterns(post),
            motions: Vec::new(),
            indexes: Vec::new(),
            retracts: Vec::new(),
        })
    }

    fn report(self) -> FiveAudit {
        FiveAudit {
            motions: self.motions,
            indexes: self.indexes,
            retracts: self.retracts,
            spindle: self.spindle,
            coolant: self.coolant,
            tool: self.tool,
            units: self.units,
            work: self.work,
            distance: self.distance,
            length: self.length,
            feed_mode: self.feed_mode,
            work_frame: self.work_frame,
            tcp: self.tcp,
            tcp_h: self.tcp_h,
            frame_coded: self.frame_coded,
            tcp_coded: self.tcp_coded,
            tcp_configured: self.tcp_configured,
            machine_mode: self.machine_mode,
            chord_tol: self.chord_tol,
            axis_tol_deg: self.axis_tol_deg,
            rotary_decimals: self.rotary_decimals,
            linear_decimals: self.post.linear_decimals(),
            feed_decimals: self.post.feed_decimals(),
            rapid_feed: self.rapid_feed,
        }
    }

    fn line(&mut self, line_no: usize, raw: &str) -> Result<()> {
        if let Some(limit) = self.post.max_line_length()
            && raw.chars().count() > limit
        {
            return Err(fail(line_no, format!("line exceeds max_line_length {limit}")));
        }
        if let Some(tool) = comment_tool(raw, &self.tool_patterns) {
            self.tool = Some(tool);
        }
        let stripped = strip_comments(raw);
        let trimmed = stripped.trim();
        if trimmed.is_empty() {
            if raw.trim().is_empty() || !self.post.line_numbers.enabled {
                return Ok(());
            }
            return Err(fail(line_no, "line is missing its sequence number"));
        }
        if trimmed == "%" {
            return Ok(());
        }
        if let Some(digits) = program_digits(trimmed) {
            if self.post.program_number.as_deref() != Some(digits) {
                return Err(fail(line_no, "unexpected program number"));
            }
            return Ok(());
        }
        let mut words = lex(trimmed, line_no, self.post)?;
        if self.post.line_numbers.enabled {
            self.take_sequence(line_no, &mut words)?;
        }
        if words.is_empty() {
            return Ok(());
        }
        let parsed = parse_words(line_no, &words)?;
        self.apply(line_no, &parsed)
    }

    fn take_sequence(&mut self, line_no: usize, words: &mut Vec<Word>) -> Result<()> {
        let prefix = self.post.line_numbers.prefix.chars().next().unwrap_or('N');
        let Some(first) = words.first() else {
            return Err(fail(line_no, "line is missing its sequence number"));
        };
        if first.letter != prefix {
            return Err(fail(line_no, "line is missing its sequence number"));
        }
        if first.raw.contains('.') || first.value < 0.0 || (first.value - first.value.round()).abs() > 1e-6 {
            return Err(fail(line_no, "line number must be a whole number"));
        }
        let number = first.value.round() as u64;
        if number > LINE_NUMBER_LIMIT {
            return Err(fail(line_no, "line number is above 1000000000"));
        }
        let expected = self.next_number.unwrap_or(number);
        if number != expected {
            return Err(fail(line_no, format!("line number {number} is not the next sequence number {expected}")));
        }
        let step = u64::from(self.post.line_numbers.step);
        self.next_number = Some(number.saturating_add(step));
        words.remove(0);
        if words.iter().any(|word| word.letter == prefix) {
            return Err(fail(line_no, "duplicate sequence number"));
        }
        Ok(())
    }

    fn apply(&mut self, line_no: usize, parsed: &Parsed) -> Result<()> {
        self.apply_tools(line_no, parsed)?;
        if parsed.g.contains(&53) {
            self.apply_feed_codes(line_no, parsed)?;
            self.take_rotaries(parsed);
            self.retracts.push(FiveRetract { line: line_no, axes: [parsed.x, parsed.y, parsed.z] });
            return Ok(());
        }
        let line_g = self.apply_g(line_no, parsed)?;
        self.finish_length_and_tcp(line_no, parsed, line_g)?;
        self.take_rotaries(parsed);
        if parsed.g.contains(&4) {
            if parsed.axes_written.iter().any(|wrote| *wrote)
                || parsed.rotary_written.iter().any(|wrote| *wrote)
                || parsed.g.iter().any(|code| matches!(code, 0..=3))
            {
                return Err(fail(line_no, "dwell cannot share a block with motion"));
            }
            if parsed.p.is_none() {
                return Err(fail(line_no, "dwell is missing its value"));
            }
            return Ok(());
        }
        let has_axis = parsed.axes_written.iter().any(|wrote| *wrote);
        let has_rotary = parsed.rotary_written.iter().any(|wrote| *wrote);
        if !has_axis && !has_rotary {
            return Ok(());
        }
        let Some(distance) = self.distance else {
            return Err(fail(line_no, "motion before distance mode is set"));
        };
        if distance != DistanceMode::Absolute {
            return Err(fail(line_no, "incremental motion is not supported"));
        }
        let Some(units) = self.units else {
            return Err(fail(line_no, "motion before units are set"));
        };
        let Some(mode) = self.motion else {
            return Err(fail(line_no, "motion has no mode"));
        };
        if !has_axis {
            self.indexes.push(FiveIndex {
                line: line_no,
                rotaries: self.rotaries,
                written: parsed.rotary_written,
                work_frame: self.work_frame,
                tcp: self.tcp,
            });
            return Ok(());
        }
        self.check_feed(line_no, mode, parsed.f.is_some())?;
        let mut end = self.pos;
        if let Some(value) = parsed.x {
            end[0] = value;
        }
        if let Some(value) = parsed.y {
            end[1] = value;
        }
        if let Some(value) = parsed.z {
            end[2] = value;
        }
        let arc = if matches!(mode, MotionMode::Cw | MotionMode::Ccw) {
            Some(self.arc_spec(line_no, mode, end, parsed)?)
        } else if parsed.i.is_some() || parsed.j.is_some() || parsed.k.is_some() || parsed.r.is_some() {
            return Err(fail(line_no, "centre word on a non-arc block"));
        } else {
            None
        };
        self.motions.push(FiveMotion {
            line: line_no,
            mode,
            plane: self.plane,
            distance,
            units,
            feed_mode: self.feed_mode,
            feed: self.feed,
            feed_on_block: parsed.f.is_some(),
            work: self.work,
            tool: self.tool,
            length: self.length,
            spindle: self.spindle,
            coolant: self.coolant,
            end,
            axes_written: parsed.axes_written,
            arc,
            work_frame: self.work_frame,
            tcp: self.tcp,
            tcp_h: self.tcp_h,
            rotaries: self.rotaries,
            rotary_written: parsed.rotary_written,
        });
        self.pos = end;
        Ok(())
    }

    fn apply_tools(&mut self, line_no: usize, parsed: &Parsed) -> Result<()> {
        if let Some(tool) = parsed.tool {
            self.pending_tool = Some(tool);
        }
        if let Some(speed) = parsed.s {
            if speed < 0.0 {
                return Err(fail(line_no, "S word must be >= 0"));
            }
            self.spindle.speed = speed;
        }
        for code in &parsed.m {
            match *code {
                0 | 1 => {}
                2 | 30 => {
                    self.spindle.on = false;
                    self.coolant = CoolantState::Off;
                }
                3 => {
                    self.spindle.on = true;
                    self.spindle.clockwise = true;
                }
                4 => {
                    self.spindle.on = true;
                    self.spindle.clockwise = false;
                }
                5 => self.spindle.on = false,
                6 => {
                    let Some(tool) = self.pending_tool else {
                        return Err(fail(line_no, "M6 with no tool"));
                    };
                    self.tool = Some(tool);
                }
                7 => self.coolant = CoolantState::Mist,
                8 => self.coolant = CoolantState::Flood,
                9 => self.coolant = CoolantState::Off,
                64 | 65 => {
                    let Some(pin) = parsed.p else {
                        return Err(fail(line_no, "digital output needs a P word"));
                    };
                    if !self.post.digital_pin(pin) {
                        return Err(fail(line_no, "digital output P is not a declared pin"));
                    }
                }
                other => return Err(fail(line_no, format!("unsupported M{other}"))),
            }
        }
        Ok(())
    }

    fn apply_feed_codes(&mut self, line_no: usize, parsed: &Parsed) -> Result<()> {
        let mut block_feed = None;
        for code in &parsed.g {
            let mode = match *code {
                93 => Some(FeedMode::InverseTime),
                94 => Some(FeedMode::PerMinute),
                _ => None,
            };
            let Some(mode) = mode else {
                continue;
            };
            if let Some(prev) = block_feed
                && prev != mode
            {
                return Err(fail(line_no, "line has conflicting G codes"));
            }
            block_feed = Some(mode);
            self.feed_mode = mode;
        }
        if let Some(feed) = parsed.f {
            self.feed = Some(feed);
        }
        Ok(())
    }

    fn apply_g(&mut self, line_no: usize, parsed: &Parsed) -> Result<LineG> {
        let mut block_motion = None;
        let mut block_plane = None;
        let mut saw_frame: Option<bool> = None;
        let mut tcp_on = false;
        let mut tcp_off = false;
        let mut want_length = false;
        let mut cancel_length = false;
        for code in &parsed.g {
            if self.codes.frame_on.contains(code) {
                if saw_frame == Some(false) {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                saw_frame = Some(true);
                self.work_frame = true;
            }
            if self.codes.frame_off.contains(code) {
                if saw_frame == Some(true) {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                saw_frame = Some(false);
                self.work_frame = false;
            }
            if self.codes.tcp_on.contains(code) {
                tcp_on = true;
            }
            if self.codes.tcp_off.contains(code) {
                tcp_off = true;
            }
            match *code {
                0 => self.set_motion(line_no, MotionMode::Rapid, &mut block_motion)?,
                1 => self.set_motion(line_no, MotionMode::Linear, &mut block_motion)?,
                2 => self.set_motion(line_no, MotionMode::Cw, &mut block_motion)?,
                3 => self.set_motion(line_no, MotionMode::Ccw, &mut block_motion)?,
                4 => {}
                17 => self.set_plane(line_no, Plane::Xy, &mut block_plane)?,
                18 => self.set_plane(line_no, Plane::Zx, &mut block_plane)?,
                19 => self.set_plane(line_no, Plane::Yz, &mut block_plane)?,
                20 => return Err(fail(line_no, "inch five-axis verification is not supported yet")),
                21 => self.set_units(line_no, UnitMode::Millimetre)?,
                40 | 41 | 42 | 64 | 80 => {}
                43 => want_length = true,
                49 => cancel_length = true,
                53 => {}
                54 => self.set_work(line_no, WorkOffset::G54)?,
                55 => self.set_work(line_no, WorkOffset::G55)?,
                56 => self.set_work(line_no, WorkOffset::G56)?,
                57 => self.set_work(line_no, WorkOffset::G57)?,
                58 => self.set_work(line_no, WorkOffset::G58)?,
                59 => self.set_work(line_no, WorkOffset::G59)?,
                90 => self.set_distance(line_no, DistanceMode::Absolute)?,
                91 => return Err(fail(line_no, "incremental motion is not supported")),
                93 | 94 => {}
                other if self.known_extra(other) => {}
                other => return Err(fail(line_no, format!("unsupported G{other}"))),
            }
        }
        if want_length && (cancel_length || tcp_on) {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        if tcp_on && (tcp_off || cancel_length) {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        self.apply_feed_codes(line_no, parsed)?;
        Ok(LineG { want_length, cancel_length, tcp_on, tcp_off })
    }

    fn known_extra(&self, code: u16) -> bool {
        self.codes.frame_on.contains(&code)
            || self.codes.frame_off.contains(&code)
            || self.codes.tcp_on.contains(&code)
            || self.codes.tcp_off.contains(&code)
    }

    fn finish_length_and_tcp(&mut self, line_no: usize, parsed: &Parsed, line_g: LineG) -> Result<()> {
        if line_g.want_length {
            let Some(h) = parsed.h else {
                return Err(fail(line_no, "G43 needs an H word"));
            };
            self.length = LengthOffset::Active(h);
            self.tcp = false;
            self.tcp_h = None;
        } else if line_g.cancel_length {
            self.length = LengthOffset::Cancelled;
            self.tcp = false;
            self.tcp_h = None;
        }
        if line_g.tcp_on {
            self.tcp = true;
            self.length = LengthOffset::Cancelled;
            if self.codes.tcp_on_wants_h {
                let Some(h) = parsed.h else {
                    return Err(fail(line_no, "TCP on needs an H word"));
                };
                self.tcp_h = Some(h);
            }
        } else if line_g.tcp_off {
            self.tcp = false;
            self.tcp_h = None;
            if !line_g.cancel_length && !line_g.want_length {
                self.length = LengthOffset::Cancelled;
            }
        } else if parsed.h.is_some() && !line_g.want_length {
            return Err(fail(line_no, "unexpected H word"));
        }
        Ok(())
    }

    fn take_rotaries(&mut self, parsed: &Parsed) {
        for index in 0..3 {
            if let Some(value) = parsed.rotaries[index] {
                self.rotaries[index] = Some(value);
            }
        }
    }

    fn check_feed(&self, line_no: usize, mode: MotionMode, wrote_f: bool) -> Result<()> {
        if matches!(mode, MotionMode::Rapid) {
            return Ok(());
        }
        match self.feed {
            Some(feed) if feed > 0.0 => {}
            _ => return Err(fail(line_no, "feed move has no positive feed")),
        }
        if self.feed_mode == FeedMode::InverseTime && !wrote_f {
            return Err(fail(line_no, "inverse-time block has no F"));
        }
        Ok(())
    }

    fn arc_spec(&self, line_no: usize, mode: MotionMode, end: Xyz, parsed: &Parsed) -> Result<ArcReplay> {
        let Some(plane) = self.plane else {
            return Err(fail(line_no, "arc has no plane"));
        };
        if parsed.r.is_some() && (parsed.i.is_some() || parsed.j.is_some() || parsed.k.is_some()) {
            return Err(fail(line_no, "arc has both R and IJK"));
        }
        let sense = match mode {
            MotionMode::Cw => ArcSense::Cw,
            MotionMode::Ccw => ArcSense::Ccw,
            MotionMode::Rapid | MotionMode::Linear => return Err(fail(line_no, "arc has no plane")),
        };
        let (center, radius) = if let Some(radius) = parsed.r {
            if radius == 0.0 {
                return Err(fail(line_no, "arc radius must be non-zero"));
            }
            let quantum = 10_f64.powi(-i32::from(self.post.linear_decimals()));
            let center = geom::center_from_radius(self.pos, end, radius, sense, plane, quantum)?;
            (center, Some(radius))
        } else {
            let (need_i, need_j, need_k) = match plane {
                Plane::Xy => (true, true, false),
                Plane::Zx => (true, false, true),
                Plane::Yz => (false, true, true),
            };
            if parsed.i.is_some() != need_i || parsed.j.is_some() != need_j || parsed.k.is_some() != need_k {
                return Err(fail(line_no, "arc is missing a full centre specification"));
            }
            let center = [self.pos[0] + parsed.i.unwrap_or(0.0), self.pos[1] + parsed.j.unwrap_or(0.0), self.pos[2] + parsed.k.unwrap_or(0.0)];
            (center, None)
        };
        Ok(ArcReplay { sense, center, radius })
    }

    fn set_motion(&mut self, line_no: usize, mode: MotionMode, block: &mut Option<MotionMode>) -> Result<()> {
        if let Some(prev) = *block
            && prev != mode
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        *block = Some(mode);
        self.motion = Some(mode);
        Ok(())
    }

    fn set_plane(&mut self, line_no: usize, plane: Plane, block: &mut Option<Plane>) -> Result<()> {
        if let Some(prev) = *block
            && prev != plane
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        *block = Some(plane);
        self.plane = Some(plane);
        Ok(())
    }

    fn set_units(&mut self, line_no: usize, units: UnitMode) -> Result<()> {
        if let Some(prev) = self.units
            && prev != units
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        self.units = Some(units);
        Ok(())
    }

    fn set_distance(&mut self, line_no: usize, distance: DistanceMode) -> Result<()> {
        if let Some(prev) = self.distance
            && prev != distance
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        self.distance = Some(distance);
        Ok(())
    }

    fn set_work(&mut self, line_no: usize, work: WorkOffset) -> Result<()> {
        if let Some(prev) = self.work
            && prev != work
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        self.work = Some(work);
        Ok(())
    }

    fn cross_check(&self, text: &str, post: &PostDef) -> Result<()> {
        let moves = read_back(text, post).map_err(|err| invalid(format!("read_back: {err}")))?;
        let read: Vec<&ReadMove> = moves.iter().filter(|move_| is_motion(move_)).collect();
        if read.len() != self.motions.len() {
            return Err(invalid(format!("audit saw {} motions and read_back saw {}", self.motions.len(), read.len())));
        }
        for (state, move_) in self.motions.iter().zip(read) {
            if dist3(state.end, end_of(move_)) > 1e-6 {
                return Err(invalid(format!("line {}: audit end does not match read_back", state.line)));
            }
            let (read_feed, inverse, sense, center, five) = match move_ {
                ReadMove::Rapid { feed, inverse_time, five, .. } => {
                    if state.mode != MotionMode::Rapid {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, None, None, *five)
                }
                ReadMove::Linear { feed, inverse_time, five, .. } => {
                    if state.mode != MotionMode::Linear {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, None, None, *five)
                }
                ReadMove::Arc { feed, inverse_time, sense, center, five, .. } => {
                    let mode = match sense {
                        ArcSense::Cw => MotionMode::Cw,
                        ArcSense::Ccw => MotionMode::Ccw,
                    };
                    if state.mode != mode {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, Some(*sense), Some(*center), *five)
                }
                ReadMove::Dwell { .. } | ReadMove::Thread { .. } | ReadMove::Other { .. } => continue,
            };
            if feed_gap(state.feed, read_feed) > 1e-6 {
                return Err(invalid(format!("line {}: audit feed does not match read_back", state.line)));
            }
            if (state.feed_mode == FeedMode::InverseTime) != inverse {
                return Err(invalid(format!("line {}: audit feed mode does not match read_back", state.line)));
            }
            if !rotaries_match(state.rotaries, five) {
                return Err(invalid(format!("line {}: audit rotaries do not match read_back", state.line)));
            }
            match (state.arc, sense, center) {
                (Some(arc), Some(sense), Some(center)) if arc.sense != sense || dist3(arc.center, center) > 1e-4 => {
                    return Err(invalid(format!("line {}: audit arc does not match read_back", state.line)));
                }
                (Some(_), Some(_), Some(_)) | (None, None, None) => {}
                _ => return Err(invalid(format!("line {}: audit arc does not match read_back", state.line))),
            }
        }
        Ok(())
    }
}

struct LineG {
    want_length: bool,
    cancel_length: bool,
    tcp_on: bool,
    tcp_off: bool,
}

fn parse_words(line_no: usize, words: &[Word]) -> Result<Parsed> {
    let mut parsed = Parsed {
        g: Vec::new(),
        m: Vec::new(),
        x: None,
        y: None,
        z: None,
        i: None,
        j: None,
        k: None,
        r: None,
        f: None,
        s: None,
        p: None,
        h: None,
        tool: None,
        rotaries: [None; 3],
        axes_written: [false; 3],
        rotary_written: [false; 3],
    };
    let mut saw_t = false;
    for word in words {
        match word.letter {
            // IJK is already incremental; keep arc-centre mode separate from positioning.
            'G' if word.value == 91.1 => {}
            'G' if word.value == 90.1 => {
                return Err(fail(line_no, "G90.1 absolute arc centres are not supported; use G91.1 for incremental IJK"));
            }
            'G' => parsed.g.push(g_or_m(line_no, word, 'G')?),
            'M' => parsed.m.push(g_or_m(line_no, word, 'M')?),
            'F' => {
                if parsed.f.is_some() {
                    return Err(fail(line_no, "duplicate F word"));
                }
                if !(word.value.is_finite() && word.value > 0.0) {
                    return Err(fail(line_no, "feed is not positive"));
                }
                parsed.f = Some(word.value);
            }
            'S' => set_unique_f(line_no, &mut parsed.s, word)?,
            'T' => {
                if saw_t {
                    return Err(fail(line_no, "duplicate T word"));
                }
                saw_t = true;
                parsed.tool = Some(whole_word(line_no, word, 'T')?);
            }
            'H' => {
                if parsed.h.is_some() {
                    return Err(fail(line_no, "duplicate H word"));
                }
                parsed.h = Some(whole_word(line_no, word, 'H')?);
            }
            'P' => {
                if parsed.p.is_some() {
                    return Err(fail(line_no, "duplicate P word"));
                }
                if !(word.value.is_finite() && word.value >= 0.0) {
                    return Err(fail(line_no, "P word must be >= 0"));
                }
                parsed.p = Some(word.value);
            }
            'I' => set_unique_f(line_no, &mut parsed.i, word)?,
            'J' => set_unique_f(line_no, &mut parsed.j, word)?,
            'K' => set_unique_f(line_no, &mut parsed.k, word)?,
            'R' => set_unique_f(line_no, &mut parsed.r, word)?,
            'X' => set_axis(line_no, &mut parsed.x, &mut parsed.axes_written, 0, word)?,
            'Y' => set_axis(line_no, &mut parsed.y, &mut parsed.axes_written, 1, word)?,
            'Z' => set_axis(line_no, &mut parsed.z, &mut parsed.axes_written, 2, word)?,
            'A' => set_rotary(line_no, &mut parsed, 0, word)?,
            'B' => set_rotary(line_no, &mut parsed, 1, word)?,
            'C' => set_rotary(line_no, &mut parsed, 2, word)?,
            letter => return Err(fail(line_no, format!("unexpected word {letter}"))),
        }
    }
    Ok(parsed)
}

fn set_axis(line_no: usize, slot: &mut Option<f64>, written: &mut [bool; 3], index: usize, word: &Word) -> Result<()> {
    if slot.is_some() {
        return Err(fail(line_no, format!("duplicate {} word", word.letter)));
    }
    *slot = Some(word.value);
    if let Some(flag) = written.get_mut(index) {
        *flag = true;
    }
    Ok(())
}

fn set_rotary(line_no: usize, parsed: &mut Parsed, index: usize, word: &Word) -> Result<()> {
    if parsed.rotaries.get(index).and_then(|value| *value).is_some() {
        return Err(fail(line_no, format!("duplicate {} word", word.letter)));
    }
    if let Some(slot) = parsed.rotaries.get_mut(index) {
        *slot = Some(word.value);
    }
    if let Some(flag) = parsed.rotary_written.get_mut(index) {
        *flag = true;
    }
    Ok(())
}

fn is_motion(move_: &ReadMove) -> bool {
    matches!(move_, ReadMove::Rapid { .. } | ReadMove::Linear { .. } | ReadMove::Arc { .. })
}

fn end_of(move_: &ReadMove) -> Xyz {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } => *end,
        ReadMove::Thread { end, .. } => *end,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => [0.0, 0.0, 0.0],
    }
}

fn feed_gap(left: Option<f64>, right: Option<f64>) -> f64 {
    match (left, right) {
        (Some(left), Some(right)) => (left - right).abs(),
        (None, None) => 0.0,
        _ => f64::MAX,
    }
}

fn rotaries_match(left: [Option<f64>; 3], right: [Option<f64>; 3]) -> bool {
    left.iter().zip(right).all(|(lhs, rhs)| match (lhs, rhs) {
        (None, None) => true,
        (Some(lhs), Some(rhs)) => (lhs - rhs).abs() <= 1e-6,
        _ => false,
    })
}

fn dist3(a: Xyz, b: Xyz) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}

fn fail(line_no: usize, message: impl Into<String>) -> crate::PostError {
    invalid(format!("line {line_no}: {}", message.into()))
}

fn g_or_m(line_no: usize, word: &Word, letter: char) -> Result<u16> {
    if !(word.value.is_finite() && (0.0..=999.0).contains(&word.value)) {
        return Err(fail(line_no, format!("{letter} code is out of range")));
    }
    let rounded = word.value.round();
    if (word.value - rounded).abs() > 1e-6 {
        return Err(fail(line_no, format!("{letter} code is not an integer")));
    }
    Ok(rounded as u16)
}

fn whole_word(line_no: usize, word: &Word, letter: char) -> Result<u32> {
    if !(word.value.is_finite() && word.value >= 0.0 && word.value <= f64::from(u32::MAX)) {
        return Err(fail(line_no, format!("{letter} word is out of range")));
    }
    let rounded = word.value.round();
    if (word.value - rounded).abs() > 1e-6 {
        return Err(fail(line_no, format!("{letter} word must be a whole number")));
    }
    Ok(rounded as u32)
}

fn set_unique_f(line_no: usize, slot: &mut Option<f64>, word: &Word) -> Result<()> {
    if slot.is_some() {
        return Err(fail(line_no, format!("duplicate {} word", word.letter)));
    }
    *slot = Some(word.value);
    Ok(())
}

fn template_g_codes(lines: &[String]) -> Vec<u16> {
    let mut codes = Vec::new();
    for line in lines {
        let bytes = line.as_bytes();
        let mut index = 0;
        while index < bytes.len() {
            if bytes[index].eq_ignore_ascii_case(&b'G') {
                index += 1;
                let start = index;
                while index < bytes.len() && bytes[index].is_ascii_digit() {
                    index += 1;
                }
                if start < index
                    && let Ok(code) = line[start..index].parse::<u16>()
                {
                    codes.push(code);
                }
            } else {
                index += 1;
            }
        }
    }
    codes
}

fn template_has_h(lines: &[String]) -> bool {
    lines.iter().any(|line| line.split_whitespace().any(|word| word.starts_with('H') || word.starts_with('h')))
}

fn tool_patterns(post: &PostDef) -> Vec<(String, String)> {
    let mut patterns = Vec::new();
    for line in &post.tool_change {
        let line = line.trim();
        let Some(inner) = line.strip_prefix('(').and_then(|rest| rest.strip_suffix(')')) else {
            continue;
        };
        let Some((prefix, suffix)) = inner.split_once("{tool}") else {
            continue;
        };
        patterns.push((prefix.to_string(), suffix.to_string()));
    }
    patterns
}

fn comment_tool(line: &str, patterns: &[(String, String)]) -> Option<u32> {
    if patterns.is_empty() {
        return None;
    }
    let mut rest = line;
    let mut found = None;
    while let Some(start) = rest.find('(') {
        let after = rest.get(start + 1..)?;
        let end = after.find(')')?;
        let body = after.get(..end)?;
        for (prefix, suffix) in patterns {
            let Some(mid) = body.strip_prefix(prefix.as_str()).and_then(|text| text.strip_suffix(suffix.as_str())) else {
                continue;
            };
            if !mid.is_empty() && mid.bytes().all(|byte| byte.is_ascii_digit()) {
                found = mid.parse().ok();
            }
        }
        rest = after.get(end + 1..)?;
    }
    found
}

fn program_digits(trimmed: &str) -> Option<&str> {
    let digits = trimmed.strip_prefix('O')?;
    if !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()) { Some(digits) } else { None }
}

fn strip_comments(line: &str) -> String {
    let mut out = String::with_capacity(line.len());
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

fn lex(line: &str, line_no: usize, post: &PostDef) -> Result<Vec<Word>> {
    let bytes = line.as_bytes();
    let mut index = 0;
    let mut words = Vec::new();
    while index < bytes.len() {
        if bytes[index].is_ascii_whitespace() {
            index += 1;
            continue;
        }
        let letter = bytes[index];
        if !letter.is_ascii_alphabetic() {
            return Err(fail(line_no, "unexpected character in g-code"));
        }
        let letter = (letter as char).to_ascii_uppercase();
        index += 1;
        if !letter_allowed(post, letter) {
            return Err(fail(line_no, format!("unexpected word {letter}")));
        }
        let (value, raw) = take_number(line, bytes, &mut index, line_no)?;
        check_number(line_no, post, letter, value, &raw)?;
        words.push(Word { letter, value, raw });
    }
    Ok(words)
}

fn letter_allowed(post: &PostDef, letter: char) -> bool {
    if post.axis_letters.contains(letter) || matches!(letter, 'G' | 'M' | 'F' | 'S' | 'T' | 'H' | 'P' | 'I' | 'J' | 'K' | 'R' | 'A' | 'B' | 'C') {
        return true;
    }
    if letter == 'N' && post.line_numbers.enabled {
        return true;
    }
    post.dwell.word.chars().next().is_some_and(|dwell| dwell == letter)
}

fn take_number(line: &str, bytes: &[u8], index: &mut usize, line_no: usize) -> Result<(f64, String)> {
    let start = *index;
    if *index < bytes.len() && (bytes[*index] == b'+' || bytes[*index] == b'-') {
        *index += 1;
    }
    let mut saw_digit = false;
    let mut saw_dot = false;
    while *index < bytes.len() {
        match bytes[*index] {
            b'0'..=b'9' => {
                saw_digit = true;
                *index += 1;
            }
            b'.' if !saw_dot => {
                saw_dot = true;
                *index += 1;
            }
            _ => break,
        }
    }
    if !saw_digit {
        return Err(fail(line_no, "g-code word is missing a number"));
    }
    let raw = line.get(start..*index).unwrap_or("").to_string();
    if raw.bytes().any(|byte| byte == b'e' || byte == b'E') {
        return Err(fail(line_no, "number must be a plain decimal"));
    }
    let value: f64 = raw.parse().map_err(|_| fail(line_no, format!("invalid number {raw}")))?;
    if !value.is_finite() {
        return Err(fail(line_no, "g-code number must be finite"));
    }
    Ok((value, raw))
}

fn check_number(line_no: usize, post: &PostDef, letter: char, value: f64, raw: &str) -> Result<()> {
    if is_negative_zero(raw) {
        return Err(fail(line_no, "negative zero is not allowed"));
    }
    if value.abs() > MAX_ABS {
        return Err(fail(line_no, "word is larger than 1e9"));
    }
    if post.decimal_point && dimensional(post, letter) && !raw.contains('.') {
        return Err(fail(line_no, format!("dimensional word {letter} is missing a decimal point")));
    }
    Ok(())
}

fn dimensional(post: &PostDef, letter: char) -> bool {
    post.axis_letters.contains(letter) || matches!(letter, 'I' | 'J' | 'K' | 'R' | 'A' | 'B' | 'C')
}

fn is_negative_zero(raw: &str) -> bool {
    let Some(rest) = raw.strip_prefix('-') else {
        return false;
    };
    !rest.is_empty() && rest.bytes().all(|byte| byte == b'0' || byte == b'.')
}
