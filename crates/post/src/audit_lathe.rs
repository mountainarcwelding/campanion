//! Modal replay of posted lathe and mill-turn G-code.
//!
//! [`audit_lathe`] reads the text a lathe post wrote and tracks the modal state a control would
//! keep: diameter or radius X, constant surface speed and its cap, feed per minute or per
//! revolution or inverse time, the turret tool and offset, nose compensation, the work spindle,
//! the live tool, C-axis engage, the spindle brake, face and cylinder interpolation, the
//! sub-spindle select, and two-channel wait codes.
//!
//! End points are in the part frame. Diameter-mode X is halved, unless face interpolation says
//! X is already Cartesian. `I` `J` `K` stay radius offsets. Arc sense is the IR sense: `arc_flip`
//! on ZX, and a mirrored Z, are undone the same way [`crate::read_back`] undoes them.
//!
//! A `G53` or lathe `G28` line is not a part move. Neither is a bar-feed line, nor a cylinder
//! `on` line that only sets the mapping. `G28 H0.` does not change modal C. A CSS cap is a `G50 S`
//! before `G96`, or a `D` word on the `G96` line. Empty engage, disengage, and brake templates
//! write nothing, so those flags stay off.

use std::collections::BTreeSet;

use campanion_ir::{ArcSense, Plane};

use crate::audit::{ArcReplay, MotionMode, UnitMode, WorkOffset};
use crate::def::{LatheOutput, PostDef, g_code_number};
use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};
use crate::read::{ReadMove, read_back};

const MAX_ABS: f64 = 1.0e9;
const LINE_NUMBER_LIMIT: u64 = 1_000_000_000;

/// `G7` diameter or `G8` radius. The power-up value is the post's `diameter_mode`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum XMode {
    /// `G7`, or a post whose `diameter_mode` is set and that has not written `G8`.
    Diameter,
    /// `G8`, or a post whose `diameter_mode` is clear and that has not written `G7`.
    Radius,
}

/// Feed mode a lathe control keeps. Fanuc `G98` is per minute. `G99` is per revolution.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LatheFeedMode {
    /// `G94` or `G98`. Millimetres per minute.
    PerMinute,
    /// `G95` or `G99`. Millimetres per revolution.
    PerRevolution,
    /// `G93`. `F` is `1 / minutes` on that block.
    InverseTime,
}

/// Tool-nose compensation. `G40` cancels it. `G41` and `G42` select a side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoseComp {
    /// No nose word yet.
    Unset,
    /// `G40`.
    Off,
    /// `G41`.
    Left,
    /// `G42`.
    Right,
}

/// Face or cylinder interpolation, from the post's on and off templates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterpFrame {
    /// Turning, or indexed live milling.
    Off,
    /// Face interpolation (`G112` on Haas).
    Face,
    /// Cylinder interpolation (`G107` with the mapping word).
    Cylinder,
}

/// Modal state after one part-frame motion, including a thread block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatheMotion {
    /// 1-based line number in the posted text.
    pub line: usize,
    /// 0-based channel section. `0` when the program is one channel.
    pub channel: u16,
    /// `G0`, `G1`, `G2`, or `G3`. A thread block uses `G1` here and sets [`Self::thread`].
    pub mode: MotionMode,
    /// Selected plane.
    pub plane: Option<Plane>,
    /// `G7` or `G8`, or the post default when neither has been written.
    pub x_mode: XMode,
    /// Part-frame end. Diameter X is halved. Face mode follows `face_interp.x_diameter`.
    pub end: [f64; 3],
    /// Arc in the part frame. Sense is the IR sense.
    pub arc: Option<ArcReplay>,
    /// Feed mode after this block.
    pub feed_mode: LatheFeedMode,
    /// Modal cutting feed. A thread lead on `F` is not stored here.
    pub feed: Option<f64>,
    /// This block carried a cutting `F` (not a thread lead).
    pub feed_on_block: bool,
    /// Spindle-synchronised thread (`G32` or `G33`).
    pub thread: bool,
    /// Thread lead, millimetres per revolution.
    pub lead: Option<f64>,
    /// `G96` is in effect.
    pub css: bool,
    /// Latest cap from `G50 S` or `G96 D`.
    pub css_cap: Option<f64>,
    /// Work spindle is running (`M3` or `M4` without a live-tool spindle select).
    pub spindle_on: bool,
    /// `M3` is clockwise. Meaningless while the spindle is off.
    pub spindle_clockwise: bool,
    /// Live tool is running.
    pub live_on: bool,
    /// Latest live-tool speed from `S` with `$` or from `P` on a live start.
    pub live_rpm: Option<f64>,
    /// Live tool forward. Meaningless while the live tool is off.
    pub live_forward: bool,
    /// C axis is engaged, from a non-empty engage template. Empty templates never set this.
    pub c_engaged: bool,
    /// Modal C, degrees.
    pub c: Option<f64>,
    /// This block wrote a C word.
    pub c_on_block: bool,
    /// Main spindle brake clamped.
    pub brake: bool,
    /// Secondary spindle brake clamped.
    pub sub_brake: bool,
    /// Turret tool. From `T` plus `M6`, or from `T0101` when the post writes an offset.
    pub tool: Option<u32>,
    /// Offset register. `Some(1)` for `T0101`. `None` when the post has no offset word.
    pub offset: Option<u32>,
    /// Nose compensation.
    pub nose: NoseComp,
    /// Interpolation frame.
    pub frame: InterpFrame,
    /// Sub-spindle select is active (`G14` when that word is the select mode).
    pub on_sub: bool,
    /// Tailstock advanced.
    pub tailstock: bool,
    /// Steady rest clamped.
    pub steady_rest: bool,
    /// Parts catcher extended.
    pub parts_catcher: bool,
    /// Units, once `G21` has been seen.
    pub units: Option<UnitMode>,
    /// Work offset, once `G54`–`G59` has been seen.
    pub work: Option<WorkOffset>,
}

/// One channel section and the wait codes written inside it, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelAudit {
    /// 0-based channel index.
    pub index: u16,
    /// Wait `M` numbers in program order, such as `101` for sync id 1 when `wait_base` is 100.
    pub waits: Vec<u32>,
}

/// A spindle-housing move (`B` on the transfer templates). Not a part-frame point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HousingMove {
    /// 1-based line number.
    pub line: usize,
    /// Housing position from the transfer axis word.
    pub z: f64,
    /// Feed on a `G1` housing move.
    pub feed: Option<f64>,
    /// `G53` machine move.
    pub machine: bool,
    /// Sub-spindle select was active on this line.
    pub on_sub: bool,
}

/// Modal replay of one posted lathe program.
#[derive(Clone, Debug, PartialEq)]
pub struct LatheAudit {
    /// One entry per part-frame motion, in order. Threads are included. `G53`, `G28`, bar feed,
    /// and interpolation mode lines are not.
    pub motions: Vec<LatheMotion>,
    /// Channel sections in the order their start lines appear. Empty for a one-channel program.
    pub channels: Vec<ChannelAudit>,
    /// Spindle-housing moves, in order.
    pub housing: Vec<HousingMove>,
    /// Work spindle after the last line.
    pub spindle_on: bool,
    /// Live tool after the last line.
    pub live_on: bool,
    /// C axis engaged after the last line.
    pub c_engaged: bool,
    /// Sub-spindle select after the last line. The writer deselects before the home lines.
    pub on_sub: bool,
    /// `G96` after the last line.
    pub css: bool,
    /// Latest CSS cap.
    pub css_cap: Option<f64>,
    /// A `G96` ran with no `G50 S` before it and no `D` on that line.
    pub g96_without_cap: bool,
    /// A bar-feed line was written.
    pub bar_feed: bool,
    /// Tool left selected.
    pub tool: Option<u32>,
    /// Offset left selected.
    pub offset: Option<u32>,
    /// Nose compensation after the last line.
    pub nose: NoseComp,
    /// Tailstock advanced after the last line.
    pub tailstock: bool,
    /// Steady rest clamped after the last line.
    pub steady_rest: bool,
    /// Parts catcher extended after the last line.
    pub parts_catcher: bool,
    /// X mode after the last line.
    pub x_mode: XMode,
    /// Feed mode after the last line.
    pub feed_mode: LatheFeedMode,
}

/// Replay `text` as the lathe named by `post` would read it.
///
/// Motion ends, arc sense, feeds, inverse time, thread leads, and C are checked against
/// [`crate::read_back`]. Unknown letters and unknown `G` or `M` codes are errors. `G20` and `G91`
/// are errors. A feed move in the thread feed mode (`G99` on Haas, before `G98` is restored) is
/// an error. An inverse-time feed block must carry its own `F`.
pub fn audit_lathe(text: &str, post: &PostDef) -> Result<LatheAudit> {
    let Some(lathe) = post.lathe.as_ref() else {
        return Err(invalid("audit_lathe reads lathe posts"));
    };
    if text.len() > 64 * 1024 * 1024 {
        return Err(invalid("g-code is too large"));
    }
    let mut parser = Parser::new(post, lathe);
    for (index, line) in text.lines().enumerate() {
        if index > 2_000_000 {
            return Err(invalid("g-code has too many lines"));
        }
        parser.line(index + 1, line)?;
    }
    parser.cross_check(text, post)?;
    Ok(parser.report())
}

struct Word {
    letter: char,
    value: f64,
    raw: String,
}

struct Codes {
    known_g: BTreeSet<u16>,
    known_m: BTreeSet<u16>,
    spindle_cw: BTreeSet<u16>,
    spindle_ccw: BTreeSet<u16>,
    spindle_off: BTreeSet<u16>,
    live_fwd: BTreeSet<u16>,
    live_rev: BTreeSet<u16>,
    live_stop: BTreeSet<u16>,
    c_engage: BTreeSet<u16>,
    c_disengage: BTreeSet<u16>,
    brake_clamp: BTreeSet<u16>,
    brake_release: BTreeSet<u16>,
    sub_brake_clamp: BTreeSet<u16>,
    sub_brake_release: BTreeSet<u16>,
    tail_advance: BTreeSet<u16>,
    tail_retract: BTreeSet<u16>,
    steady_clamp: BTreeSet<u16>,
    steady_release: BTreeSet<u16>,
    catch_extend: BTreeSet<u16>,
    catch_retract: BTreeSet<u16>,
    engage_bodies: Vec<String>,
    disengage_bodies: Vec<String>,
    select_mode: Vec<String>,
    deselect_mode: Vec<String>,
    channel_starts: Vec<(u16, String)>,
    wait_base: Option<u32>,
    wait_is_m: bool,
    thread_g: Option<u16>,
    thread_lead: Option<char>,
    thread_feed_g: Option<u16>,
    lathe_feed_g: Option<u16>,
    bar_g: Option<u16>,
    c_letter: Option<char>,
    y_letter: Option<char>,
    transfer_letter: Option<char>,
    b_head: bool,
    face_x_diameter: bool,
    mirror_z: bool,
    tool_offset: bool,
    tool_needs_m6: bool,
    face_on: Vec<String>,
    face_off: Vec<String>,
    cyl_on: Vec<String>,
    cyl_off: Vec<String>,
}

struct Block {
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
    d: Option<f64>,
    p: Option<f64>,
    c: Option<f64>,
    b: Option<f64>,
    dollar: Option<u32>,
    saw_t: bool,
    tool_word: Option<u32>,
}

struct Parser<'a> {
    post: &'a PostDef,
    lathe: &'a LatheOutput,
    codes: Codes,
    pos: Xyz,
    motion: Option<MotionMode>,
    plane: Option<Plane>,
    units: Option<UnitMode>,
    x_mode: XMode,
    feed_mode: LatheFeedMode,
    feed: Option<f64>,
    thread_feed_on: bool,
    css: bool,
    css_cap: Option<f64>,
    g96_without_cap: bool,
    spindle_on: bool,
    spindle_cw: bool,
    live_on: bool,
    live_rpm: Option<f64>,
    live_forward: bool,
    c_engaged: bool,
    c: Option<f64>,
    brake: bool,
    sub_brake: bool,
    pending_tool: Option<u32>,
    tool: Option<u32>,
    offset: Option<u32>,
    nose: NoseComp,
    frame: InterpFrame,
    on_sub: bool,
    tailstock: bool,
    steady: bool,
    catcher: bool,
    bar_feed: bool,
    b_deg: f64,
    work: Option<WorkOffset>,
    channel: u16,
    channels: Vec<ChannelAudit>,
    motions: Vec<LatheMotion>,
    housing: Vec<HousingMove>,
    next_number: Option<u64>,
}

impl<'a> Parser<'a> {
    fn new(post: &'a PostDef, lathe: &'a LatheOutput) -> Self {
        let x_mode = if lathe.diameter_mode { XMode::Diameter } else { XMode::Radius };
        let feed_mode = feed_mode_of(&lathe.feed_mode);
        let next_number = post.line_numbers.enabled.then_some(u64::from(post.line_numbers.start));
        Self {
            post,
            lathe,
            codes: Codes::from_post(post, lathe),
            pos: [0.0, 0.0, 0.0],
            motion: None,
            plane: None,
            units: None,
            x_mode,
            feed_mode,
            feed: None,
            thread_feed_on: false,
            css: false,
            css_cap: None,
            g96_without_cap: false,
            spindle_on: false,
            spindle_cw: true,
            live_on: false,
            live_rpm: None,
            live_forward: true,
            c_engaged: false,
            c: None,
            brake: false,
            sub_brake: false,
            pending_tool: None,
            tool: None,
            offset: None,
            nose: NoseComp::Unset,
            frame: InterpFrame::Off,
            on_sub: false,
            tailstock: false,
            steady: false,
            catcher: false,
            bar_feed: false,
            b_deg: 0.0,
            work: None,
            channel: 0,
            channels: Vec::new(),
            motions: Vec::new(),
            housing: Vec::new(),
            next_number,
        }
    }

    fn report(self) -> LatheAudit {
        LatheAudit {
            motions: self.motions,
            channels: self.channels,
            housing: self.housing,
            spindle_on: self.spindle_on,
            live_on: self.live_on,
            c_engaged: self.c_engaged,
            on_sub: self.on_sub,
            css: self.css,
            css_cap: self.css_cap,
            g96_without_cap: self.g96_without_cap,
            bar_feed: self.bar_feed,
            tool: self.tool,
            offset: self.offset,
            nose: self.nose,
            tailstock: self.tailstock,
            steady_rest: self.steady,
            parts_catcher: self.catcher,
            x_mode: self.x_mode,
            feed_mode: self.feed_mode,
        }
    }

    fn line(&mut self, line_no: usize, raw: &str) -> Result<()> {
        if let Some(limit) = self.post.max_line_length()
            && raw.chars().count() > limit
        {
            return Err(fail(line_no, format!("line exceeds max_line_length {limit}")));
        }
        self.note_channel(raw);
        let stripped = strip_comments(raw);
        let trimmed = stripped.trim();
        if trimmed.is_empty() || trimmed == "%" {
            return Ok(());
        }
        if let Some(digits) = program_digits(trimmed) {
            if self.post.program_number.as_deref() != Some(digits) {
                return Err(fail(line_no, "unexpected program number"));
            }
            return Ok(());
        }
        let mut words = lex(trimmed, line_no, self)?;
        if self.post.line_numbers.enabled {
            self.take_sequence(line_no, &mut words)?;
        }
        if words.is_empty() {
            return Ok(());
        }
        let block = self.bundle(line_no, &words)?;
        self.interpret(line_no, raw, &block)
    }

    fn note_channel(&mut self, raw: &str) {
        let body = block_body(self.post, raw);
        let Some((index, _)) = self.codes.channel_starts.iter().find(|(_, rendered)| rendered == &body) else {
            return;
        };
        self.channel = *index;
        self.channels.push(ChannelAudit { index: *index, waits: Vec::new() });
    }

    fn note_sub(&mut self, raw: &str) {
        let body = strip_comments(&block_body(self.post, raw));
        let body = body.trim();
        if self.codes.select_mode.iter().any(|line| line == body) {
            self.on_sub = true;
        } else if self.codes.deselect_mode.iter().any(|line| line == body) {
            self.on_sub = false;
        }
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
        self.next_number = Some(number.saturating_add(u64::from(self.post.line_numbers.step)));
        words.remove(0);
        Ok(())
    }

    fn bundle(&self, line_no: usize, words: &[Word]) -> Result<Block> {
        let mut block = Block {
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
            d: None,
            p: None,
            c: None,
            b: None,
            dollar: None,
            saw_t: false,
            tool_word: None,
        };
        let axes = axis_letters(self.post);
        for word in words {
            match word.letter {
                'G' => block.g.push(g_or_m(line_no, word, 'G')?),
                'M' => block.m.push(g_or_m(line_no, word, 'M')?),
                'F' => set_unique(line_no, &mut block.f, word)?,
                'S' => set_unique(line_no, &mut block.s, word)?,
                'D' => set_unique(line_no, &mut block.d, word)?,
                'P' => set_unique(line_no, &mut block.p, word)?,
                'I' => set_unique(line_no, &mut block.i, word)?,
                'J' => set_unique(line_no, &mut block.j, word)?,
                'K' => set_unique(line_no, &mut block.k, word)?,
                'R' => set_unique(line_no, &mut block.r, word)?,
                'T' => {
                    block.saw_t = true;
                    block.tool_word = Some(whole_word(line_no, word, 'T')?);
                }
                'H' | 'U' | 'W' => {}
                '$' => {
                    if block.dollar.is_some() {
                        return Err(fail(line_no, "duplicate spindle select"));
                    }
                    block.dollar = Some(whole_word(line_no, word, '$')?);
                }
                letter if Some(letter) == axes[0] => set_unique(line_no, &mut block.x, word)?,
                letter if Some(letter) == axes[1] => set_unique(line_no, &mut block.y, word)?,
                letter if Some(letter) == axes[2] => set_unique(line_no, &mut block.z, word)?,
                letter if self.codes.c_letter == Some(letter) => set_unique(line_no, &mut block.c, word)?,
                letter if self.codes.y_letter == Some(letter) && self.codes.y_letter != axes[1] => set_unique(line_no, &mut block.y, word)?,
                letter if (self.codes.transfer_letter == Some(letter) || self.codes.b_head) && letter == 'B' => {
                    set_unique(line_no, &mut block.b, word)?;
                }
                letter => return Err(fail(line_no, format!("unexpected word {letter}"))),
            }
        }
        Ok(block)
    }

    fn interpret(&mut self, line_no: usize, raw: &str, block: &Block) -> Result<()> {
        self.note_sub(raw);
        for &code in &block.g {
            if !self.codes.known_g.contains(&code) {
                return Err(fail(line_no, format!("unsupported G{code}")));
            }
        }
        for &code in &block.m {
            if !self.known_m(code) {
                return Err(fail(line_no, format!("unsupported M{code}")));
            }
        }
        self.apply_g_state(line_no, block)?;
        self.apply_m_and_tool(line_no, block)?;
        self.apply_speed_words(block);
        let machine = block.g.contains(&53) || block.g.contains(&28);
        let interp = self.take_interpolation(block)?;
        let dwell = block.g.contains(&4);
        let bar = block.g.iter().any(|code| self.codes.bar_g == Some(*code));
        let threading = block.g.iter().any(|code| self.codes.thread_g == Some(*code));
        if !interp && !machine {
            if self.codes.b_head
                && let Some(angle) = block.b
            {
                self.b_deg = angle;
            }
            if let Some(angle) = block.c {
                self.c = Some(angle);
            }
        }
        if machine || interp || dwell || bar {
            if bar {
                self.bar_feed = true;
            }
            if dwell && (block.x.is_some() || block.y.is_some() || block.z.is_some() || threading) {
                return Err(fail(line_no, "dwell cannot share a block with motion"));
            }
            self.note_housing(line_no, block, machine);
            if !threading {
                self.note_cutting_feed(block);
            }
            return Ok(());
        }
        if block.g.iter().any(|code| matches!(*code, 0..=3)) && self.motion.is_none() {
            return Err(fail(line_no, "motion has no mode"));
        }
        let has_axis = block.x.is_some() || block.y.is_some() || block.z.is_some();
        if !has_axis && !threading {
            self.note_housing(line_no, block, false);
            self.note_cutting_feed(block);
            return Ok(());
        }
        if self.units.is_none() {
            return Err(fail(line_no, "motion before units are set"));
        }
        if threading {
            return self.record_thread(line_no, block);
        }
        let Some(mode) = self.motion else {
            return Err(fail(line_no, "motion has no mode"));
        };
        let feed_move = !matches!(mode, MotionMode::Rapid);
        if feed_move && self.thread_feed_on {
            return Err(fail(line_no, "feed move in thread feed mode"));
        }
        self.note_cutting_feed(block);
        if feed_move {
            match self.feed {
                Some(feed) if feed > 0.0 => {}
                _ => return Err(fail(line_no, "feed move has no positive feed")),
            }
            if self.feed_mode == LatheFeedMode::InverseTime && block.f.is_none() {
                return Err(fail(line_no, "inverse-time block has no F"));
            }
        }
        if matches!(mode, MotionMode::Cw | MotionMode::Ccw) {
            if block.r.is_some() && (block.i.is_some() || block.j.is_some() || block.k.is_some()) {
                return Err(fail(line_no, "arc has both R and IJK"));
            }
        } else if block.i.is_some() || block.j.is_some() || block.k.is_some() || block.r.is_some() {
            return Err(fail(line_no, "centre word on a non-arc block"));
        }
        let start = self.pos;
        let end = self.part_end(block);
        let arc = if matches!(mode, MotionMode::Cw | MotionMode::Ccw) { Some(self.arc_spec(line_no, mode, start, end, block)?) } else { None };
        let c_on_block = block.c.is_some();
        self.push_motion(line_no, mode, end, arc, false, None, block.f.is_some(), c_on_block);
        self.pos = end;
        Ok(())
    }

    fn record_thread(&mut self, line_no: usize, block: &Block) -> Result<()> {
        let letter = self.codes.thread_lead.ok_or_else(|| fail(line_no, "thread lead word is missing"))?;
        let lead = match letter {
            'F' => block.f,
            'I' => block.i,
            'J' => block.j,
            'K' => block.k,
            _ => None,
        }
        .ok_or_else(|| fail(line_no, "thread lead is missing"))?;
        if !(lead.is_finite() && lead > 0.0) {
            return Err(fail(line_no, "thread lead must be positive"));
        }
        if letter == 'F' {
            self.feed = None;
        }
        let end = self.part_end(block);
        self.push_motion(line_no, MotionMode::Linear, end, None, true, Some(lead), false, block.c.is_some());
        self.pos = end;
        self.motion = None;
        Ok(())
    }

    fn push_motion(
        &mut self,
        line_no: usize,
        mode: MotionMode,
        end: Xyz,
        arc: Option<ArcReplay>,
        thread: bool,
        lead: Option<f64>,
        feed_on_block: bool,
        c_on_block: bool,
    ) {
        let state = LatheMotion {
            line: line_no,
            channel: self.channel,
            mode,
            plane: self.plane,
            x_mode: self.x_mode,
            end,
            arc,
            feed_mode: self.feed_mode,
            feed: self.feed,
            feed_on_block,
            thread,
            lead,
            css: self.css,
            css_cap: self.css_cap,
            spindle_on: self.spindle_on,
            spindle_clockwise: self.spindle_cw,
            live_on: self.live_on,
            live_rpm: self.live_rpm,
            live_forward: self.live_forward,
            c_engaged: self.c_engaged,
            c: self.c,
            c_on_block,
            brake: self.brake,
            sub_brake: self.sub_brake,
            tool: self.tool,
            offset: self.offset,
            nose: self.nose,
            frame: self.frame,
            on_sub: self.on_sub,
            tailstock: self.tailstock,
            steady_rest: self.steady,
            parts_catcher: self.catcher,
            units: self.units,
            work: self.work,
        };
        self.motions.push(state);
    }

    fn part_end(&self, block: &Block) -> Xyz {
        let mut end = self.pos;
        if let Some(x) = block.x {
            end[0] = self.stored_x(x);
        }
        let face = self.frame == InterpFrame::Face;
        let cyl = self.frame == InterpFrame::Cylinder;
        if face || cyl {
            if let Some(y) = block.y {
                end[1] = y;
            }
        } else if self.codes.y_letter.is_none() {
            end[1] = 0.0;
        } else if let Some(y) = block.y {
            end[1] = y;
        }
        if let Some(z) = block.z {
            end[2] = if self.mirroring() { -z } else { z };
        }
        self.unpivot(end, block.x.is_some(), block.z.is_some())
    }

    fn stored_x(&self, x: f64) -> f64 {
        if self.frame == InterpFrame::Face {
            if self.codes.face_x_diameter { x * 0.5 } else { x }
        } else if self.lathe.diameter_mode {
            x * 0.5
        } else {
            x
        }
    }

    fn unpivot(&self, mut end: Xyz, has_x: bool, has_z: bool) -> Xyz {
        let Some(b_head) = self.lathe.b_head.as_ref() else {
            return end;
        };
        let (dr, dz) = crate::def::b_pivot_offset(self.b_deg, &b_head.mode, b_head.pivot_length);
        if has_x {
            end[0] -= dr;
        }
        if has_z {
            end[2] -= dz;
        }
        end
    }

    fn mirroring(&self) -> bool {
        self.on_sub && self.codes.mirror_z
    }

    fn arc_spec(&self, line_no: usize, mode: MotionMode, start: Xyz, end: Xyz, block: &Block) -> Result<ArcReplay> {
        let Some(plane) = self.plane else {
            return Err(fail(line_no, "arc has no plane"));
        };
        let written = match mode {
            MotionMode::Cw => ArcSense::Cw,
            MotionMode::Ccw => ArcSense::Ccw,
            MotionMode::Rapid | MotionMode::Linear => return Err(fail(line_no, "arc has no plane")),
        };
        let sense = self.ir_sense(written, plane);
        let (center, radius) = if let Some(radius) = block.r {
            if radius == 0.0 {
                return Err(fail(line_no, "arc radius must be non-zero"));
            }
            let quantum = 10_f64.powi(-i32::from(self.post.linear_decimals()));
            let center = geom::center_from_radius(start, end, radius, sense, plane, quantum)?;
            (center, Some(radius))
        } else {
            let (need_i, need_j, need_k) = match plane {
                Plane::Xy => (true, true, false),
                Plane::Zx => (true, false, true),
                Plane::Yz => (false, true, true),
            };
            if block.i.is_some() != need_i || block.j.is_some() != need_j || block.k.is_some() != need_k {
                return Err(fail(line_no, "arc is missing a full centre specification"));
            }
            let k = match block.k {
                Some(value) if self.mirroring() => -value,
                other => other.unwrap_or(0.0),
            };
            let center = [start[0] + block.i.unwrap_or(0.0), start[1] + block.j.unwrap_or(0.0), start[2] + k];
            (center, None)
        };
        Ok(ArcReplay { sense, center, radius })
    }

    fn ir_sense(&self, written: ArcSense, plane: Plane) -> ArcSense {
        let flip = plane == Plane::Zx && self.lathe.arc_flip;
        let mirror = self.mirroring() && matches!(plane, Plane::Zx | Plane::Yz);
        if flip == mirror { written } else { flip_sense(written) }
    }

    fn apply_g_state(&mut self, line_no: usize, block: &Block) -> Result<()> {
        let machine = block.g.contains(&53) || block.g.contains(&28);
        let mut motions = 0u8;
        let mut planes = 0u8;
        for &code in &block.g {
            if !machine && matches!(code, 0..=3) {
                let mode = match code {
                    0 => MotionMode::Rapid,
                    1 => MotionMode::Linear,
                    2 => MotionMode::Cw,
                    _ => MotionMode::Ccw,
                };
                if self.motion.is_some() && motions > 0 && self.motion != Some(mode) {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                if motions > 0 && self.motion != Some(mode) {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                motions += 1;
                self.motion = Some(mode);
            }
            if matches!(code, 17..=19) {
                let plane = match code {
                    17 => Plane::Xy,
                    18 => Plane::Zx,
                    _ => Plane::Yz,
                };
                if planes > 0 && self.plane != Some(plane) {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                planes += 1;
                self.plane = Some(plane);
            }
            match code {
                7 => self.x_mode = XMode::Diameter,
                8 => self.x_mode = XMode::Radius,
                20 => return Err(fail(line_no, "inches are not supported")),
                21 => self.units = Some(UnitMode::Millimetre),
                40 => self.nose = NoseComp::Off,
                41 => self.nose = NoseComp::Left,
                42 => self.nose = NoseComp::Right,
                54 => self.work = Some(WorkOffset::G54),
                55 => self.work = Some(WorkOffset::G55),
                56 => self.work = Some(WorkOffset::G56),
                57 => self.work = Some(WorkOffset::G57),
                58 => self.work = Some(WorkOffset::G58),
                59 => self.work = Some(WorkOffset::G59),
                90 => {}
                91 => return Err(fail(line_no, "incremental motion is not supported")),
                93 => {
                    self.feed_mode = LatheFeedMode::InverseTime;
                    self.feed = None;
                }
                94 | 98 => {
                    self.feed_mode = LatheFeedMode::PerMinute;
                    if self.codes.lathe_feed_g == Some(code) {
                        self.thread_feed_on = false;
                        self.feed = None;
                    }
                }
                95 | 99 => {
                    self.feed_mode = LatheFeedMode::PerRevolution;
                    if self.codes.lathe_feed_g == Some(code) {
                        self.thread_feed_on = false;
                    }
                    if self.codes.thread_feed_g == Some(code) {
                        self.thread_feed_on = true;
                        self.feed = None;
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn apply_m_and_tool(&mut self, line_no: usize, block: &Block) -> Result<()> {
        if let Some(value) = block.tool_word {
            self.stage_tool(value);
        }
        let dollar = block.dollar.is_some();
        for &code in &block.m {
            if self.codes.wait_is_m && self.codes.wait_base.is_some_and(|base| u32::from(code) >= base) && !self.codes.known_m.contains(&code) {
                self.push_wait(code);
                continue;
            }
            self.apply_one_m(line_no, code, dollar, block.p)?;
        }
        if block.m.contains(&6) {
            let Some(tool) = self.pending_tool.or(self.tool) else {
                return Err(fail(line_no, "M6 with no tool"));
            };
            if !self.codes.tool_offset {
                self.tool = Some(tool);
                self.offset = None;
            }
        }
        Ok(())
    }

    fn apply_one_m(&mut self, line_no: usize, code: u16, dollar: bool, p: Option<f64>) -> Result<()> {
        if self.is_wait(code) {
            self.push_wait(code);
            return Ok(());
        }
        if self.live_start(code, dollar, true) {
            if let Some(rpm) = p {
                self.live_rpm = Some(rpm);
            }
            self.live_on = true;
            self.live_forward = true;
            return Ok(());
        }
        if self.live_start(code, dollar, false) {
            if let Some(rpm) = p {
                self.live_rpm = Some(rpm);
            }
            self.live_on = true;
            self.live_forward = false;
            return Ok(());
        }
        if self.live_stop(code, dollar) {
            self.live_on = false;
            return Ok(());
        }
        if self.codes.c_engage.contains(&code) {
            self.c_engaged = true;
        }
        if self.codes.c_disengage.contains(&code) {
            self.c_engaged = false;
            self.c = None;
        }
        if self.codes.brake_clamp.contains(&code) {
            self.brake = true;
        }
        if self.codes.brake_release.contains(&code) {
            self.brake = false;
        }
        if self.codes.sub_brake_clamp.contains(&code) {
            self.sub_brake = true;
        }
        if self.codes.sub_brake_release.contains(&code) {
            self.sub_brake = false;
        }
        if self.codes.tail_advance.contains(&code) {
            self.tailstock = true;
        }
        if self.codes.tail_retract.contains(&code) {
            self.tailstock = false;
        }
        if self.codes.steady_clamp.contains(&code) {
            self.steady = true;
        }
        if self.codes.steady_release.contains(&code) {
            self.steady = false;
        }
        if self.codes.catch_extend.contains(&code) {
            self.catcher = true;
        }
        if self.codes.catch_retract.contains(&code) {
            self.catcher = false;
        }
        if self.codes.spindle_cw.contains(&code) && !dollar {
            self.spindle_on = true;
            self.spindle_cw = true;
        } else if self.codes.spindle_ccw.contains(&code) && !dollar {
            self.spindle_on = true;
            self.spindle_cw = false;
        } else if self.codes.spindle_off.contains(&code) && !dollar {
            self.spindle_on = false;
        }
        if code == 6 && self.codes.tool_needs_m6 {
            let Some(tool) = self.pending_tool else {
                return Err(fail(line_no, "M6 with no tool"));
            };
            self.tool = Some(tool);
            self.offset = None;
        }
        Ok(())
    }

    fn live_start(&self, code: u16, dollar: bool, forward: bool) -> bool {
        let live = if forward { &self.codes.live_fwd } else { &self.codes.live_rev };
        let spindle = if forward { &self.codes.spindle_cw } else { &self.codes.spindle_ccw };
        live.contains(&code) && (dollar || !spindle.contains(&code))
    }

    fn live_stop(&self, code: u16, dollar: bool) -> bool {
        self.codes.live_stop.contains(&code) && (dollar || !self.codes.spindle_off.contains(&code))
    }

    fn is_wait(&self, code: u16) -> bool {
        self.codes.wait_is_m && self.codes.wait_base.is_some_and(|base| u32::from(code) >= base) && !self.reserved_m(code)
    }

    fn reserved_m(&self, code: u16) -> bool {
        self.codes.spindle_cw.contains(&code)
            || self.codes.spindle_ccw.contains(&code)
            || self.codes.spindle_off.contains(&code)
            || self.codes.live_fwd.contains(&code)
            || self.codes.live_rev.contains(&code)
            || self.codes.live_stop.contains(&code)
            || self.codes.c_engage.contains(&code)
            || self.codes.c_disengage.contains(&code)
            || self.codes.brake_clamp.contains(&code)
            || self.codes.brake_release.contains(&code)
            || matches!(code, 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 30)
    }

    fn known_m(&self, code: u16) -> bool {
        self.codes.known_m.contains(&code) || self.is_wait(code)
    }

    fn push_wait(&mut self, code: u16) {
        if self.channels.is_empty() {
            self.channels.push(ChannelAudit { index: self.channel, waits: Vec::new() });
        }
        if let Some(channel) = self.channels.last_mut() {
            channel.waits.push(u32::from(code));
        }
    }

    fn stage_tool(&mut self, value: u32) {
        if self.codes.tool_offset {
            if value >= 100 {
                self.tool = Some(value / 100);
                self.offset = Some(value % 100);
            } else {
                self.tool = Some(value);
                self.offset = None;
            }
            return;
        }
        self.pending_tool = Some(value);
        if !self.codes.tool_needs_m6 {
            self.tool = Some(value);
            self.offset = None;
        }
    }

    fn apply_speed_words(&mut self, block: &Block) {
        let g96 = block.g.contains(&96);
        let g50 = block.g.contains(&50);
        let g97 = block.g.contains(&97);
        if g97 {
            self.css = false;
        }
        if g50 && let Some(cap) = block.s {
            self.css_cap = Some(cap);
        }
        if g96 {
            if let Some(cap) = block.d {
                self.css_cap = Some(cap);
            }
            if self.css_cap.is_none() {
                self.g96_without_cap = true;
            }
            self.css = true;
        }
        if block.dollar.is_some()
            && let Some(rpm) = block.s
        {
            self.live_rpm = Some(rpm);
        }
        let engage = self.template_body_hit(&self.codes.engage_bodies, block);
        if engage {
            self.c_engaged = true;
        }
        if self.template_body_hit(&self.codes.disengage_bodies, block) {
            self.c_engaged = false;
            self.c = None;
        }
    }

    fn template_body_hit(&self, bodies: &[String], block: &Block) -> bool {
        // M-code templates are applied from the M word. Multi-word lines such as `G28 H0.` are
        // matched when every G code on the template is on this block and the block has no axis word.
        if bodies.is_empty() {
            return false;
        }
        let no_axis = block.x.is_none() && block.y.is_none() && block.z.is_none() && block.c.is_none();
        bodies.iter().any(|line| {
            let needles = g_codes_in(line);
            !needles.is_empty()
                && needles.iter().all(|code| block.g.contains(code))
                && no_axis
                && m_codes_in(line).iter().all(|code| block.m.contains(code))
        })
    }

    fn note_cutting_feed(&mut self, block: &Block) {
        if self.codes.thread_lead == Some('F') && block.g.iter().any(|code| self.codes.thread_g == Some(*code)) {
            return;
        }
        if let Some(feed) = block.f {
            self.feed = Some(feed);
        }
    }

    fn note_housing(&mut self, line_no: usize, block: &Block, machine: bool) {
        let Some(letter) = self.codes.transfer_letter else {
            return;
        };
        if letter != 'B' {
            return;
        }
        let Some(z) = block.b else {
            return;
        };
        self.housing.push(HousingMove { line: line_no, z, feed: block.f, machine: machine || block.g.contains(&53), on_sub: self.on_sub });
    }

    /// True when this line only changes interpolation, matching [`crate::read_back`].
    fn take_interpolation(&mut self, block: &Block) -> Result<bool> {
        if block.g.iter().any(|code| matches!(*code, 0..=3) || self.codes.thread_g == Some(*code)) {
            return Ok(false);
        }
        let mut consumed = false;
        if !self.codes.face_on.is_empty() || !self.codes.face_off.is_empty() {
            let on = face_on_hit(&block.g, &self.codes.face_on);
            let off = g_hits(&block.g, &self.codes.face_off);
            if on && !off {
                self.frame = InterpFrame::Face;
                consumed = true;
            } else if off {
                if self.frame == InterpFrame::Face {
                    self.frame = InterpFrame::Off;
                }
                consumed = true;
            }
        }
        if !self.codes.cyl_on.is_empty() || !self.codes.cyl_off.is_empty() {
            let on = g_hits(&block.g, &self.codes.cyl_on);
            let off = g_hits(&block.g, &self.codes.cyl_off);
            if on || off {
                let enter = on && mapping_present(&self.codes.cyl_on, block)?;
                if enter {
                    self.frame = InterpFrame::Cylinder;
                    consumed = true;
                } else if off && self.frame == InterpFrame::Cylinder {
                    self.frame = InterpFrame::Off;
                    consumed = true;
                } else if off {
                    consumed = true;
                }
            }
        }
        Ok(consumed)
    }

    fn cross_check(&self, text: &str, post: &PostDef) -> Result<()> {
        let moves = read_back(text, post).map_err(|err| invalid(format!("read_back: {err}")))?;
        let read: Vec<&ReadMove> = moves.iter().filter(|move_| is_part_motion(move_)).collect();
        if read.len() != self.motions.len() {
            return Err(invalid(format!("audit saw {} motions and read_back saw {}", self.motions.len(), read.len())));
        }
        for (state, move_) in self.motions.iter().zip(read) {
            if dist3(state.end, end_of(move_)) > 1e-5 {
                return Err(invalid(format!("line {}: audit end does not match read_back", state.line)));
            }
            match move_ {
                ReadMove::Thread { lead, inverse_time, rotary, .. } => {
                    if !state.thread {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    if (state.lead.unwrap_or(0.0) - lead).abs() > 1e-5 {
                        return Err(invalid(format!("line {}: audit lead does not match read_back", state.line)));
                    }
                    self.check_flags(state, *inverse_time, *rotary)?;
                }
                ReadMove::Rapid { feed, inverse_time, rotary, .. } => {
                    if state.mode != MotionMode::Rapid || state.thread {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    self.check_feed_move(state, *feed, *inverse_time, *rotary, None, None)?;
                }
                ReadMove::Linear { feed, inverse_time, rotary, .. } => {
                    if state.mode != MotionMode::Linear || state.thread {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    self.check_feed_move(state, *feed, *inverse_time, *rotary, None, None)?;
                }
                ReadMove::Arc { feed, inverse_time, rotary, sense, center, .. } => {
                    let mode = match sense {
                        ArcSense::Cw => MotionMode::Cw,
                        ArcSense::Ccw => MotionMode::Ccw,
                    };
                    if state.mode != mode {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    self.check_feed_move(state, *feed, *inverse_time, *rotary, Some(*sense), Some(*center))?;
                }
                ReadMove::Dwell { .. } | ReadMove::Other { .. } => {}
            }
        }
        Ok(())
    }

    fn check_feed_move(
        &self,
        state: &LatheMotion,
        feed: Option<f64>,
        inverse: bool,
        rotary: Option<f64>,
        sense: Option<ArcSense>,
        center: Option<Xyz>,
    ) -> Result<()> {
        if feed_gap(state.feed, feed) > 1e-5 {
            return Err(invalid(format!("line {}: audit feed does not match read_back", state.line)));
        }
        self.check_flags(state, inverse, rotary)?;
        match (state.arc, sense, center) {
            (Some(arc), Some(sense), Some(center)) if arc.sense != sense || dist3(arc.center, center) > 1e-4 => {
                Err(invalid(format!("line {}: audit arc does not match read_back", state.line)))
            }
            (Some(_), Some(_), Some(_)) | (None, None, None) => Ok(()),
            _ => Err(invalid(format!("line {}: audit arc does not match read_back", state.line))),
        }
    }

    fn check_flags(&self, state: &LatheMotion, inverse: bool, rotary: Option<f64>) -> Result<()> {
        let inverse_state = state.feed_mode == LatheFeedMode::InverseTime;
        if inverse_state != inverse {
            return Err(invalid(format!("line {}: audit feed mode does not match read_back", state.line)));
        }
        match (state.c, rotary) {
            (Some(left), Some(right)) if (left - right).abs() > 1e-4 => {
                Err(invalid(format!("line {}: audit C does not match read_back", state.line)))
            }
            (None, None) | (Some(_), Some(_)) => Ok(()),
            _ => Err(invalid(format!("line {}: audit C does not match read_back", state.line))),
        }
    }
}

impl Codes {
    fn from_post(post: &PostDef, lathe: &LatheOutput) -> Self {
        let mut known_g = BTreeSet::from([
            0, 1, 2, 3, 4, 7, 8, 17, 18, 19, 21, 28, 40, 41, 42, 43, 49, 50, 53, 54, 55, 56, 57, 58, 59, 61, 64, 80, 90, 93, 94, 95, 96, 97, 98, 99,
        ]);
        let mut known_m = BTreeSet::from([2, 3, 4, 5, 6, 7, 8, 9, 30]);
        let mut absorb = |lines: &[String]| {
            for line in lines {
                known_g.extend(g_codes_in(line));
                known_m.extend(m_codes_in(line));
            }
        };
        absorb(&lathe.mode_lines);
        absorb(std::slice::from_ref(&lathe.plane));
        absorb(std::slice::from_ref(&lathe.feed_mode));
        absorb(&lathe.css);
        absorb(&lathe.rpm);
        absorb(&lathe.spindle_cw);
        absorb(&lathe.spindle_ccw);
        absorb(&lathe.spindle_off);
        absorb(&lathe.tool_change);
        absorb(&lathe.home);
        absorb(&post.program_end);
        absorb(&post.header);
        absorb(&post.coolant.flood);
        absorb(&post.coolant.mist);
        absorb(&post.coolant.off);
        if let Some(thread) = &lathe.thread {
            absorb(std::slice::from_ref(&thread.code));
            absorb(std::slice::from_ref(&thread.feed_mode));
        }
        if let Some(c_axis) = &lathe.c_axis {
            absorb(&c_axis.engage);
            absorb(&c_axis.disengage);
        }
        if let Some(brake) = &lathe.brake {
            absorb(&brake.clamp);
            absorb(&brake.release);
        }
        if let Some(brake) = &lathe.brake_sub {
            absorb(&brake.clamp);
            absorb(&brake.release);
        }
        if let Some(orient) = &lathe.orient {
            absorb(std::slice::from_ref(&orient.main));
            absorb(std::slice::from_ref(&orient.sub));
        }
        if let Some(live) = &lathe.live_tool {
            absorb(&live.forward);
            absorb(&live.reverse);
            absorb(&live.stop);
            absorb(std::slice::from_ref(&live.feed_mode));
        }
        if let Some(face) = &lathe.face_interp {
            absorb(&face.on);
            absorb(&face.off);
        }
        if let Some(cylinder) = &lathe.cylinder_interp {
            absorb(&cylinder.on);
            absorb(&cylinder.off);
        }
        if let Some(y_axis) = &lathe.y_axis {
            absorb(std::slice::from_ref(&y_axis.face_plane));
            absorb(std::slice::from_ref(&y_axis.side_plane));
            absorb(std::slice::from_ref(&y_axis.turn_plane));
        }
        if let Some(sub) = &lathe.sub_spindle {
            absorb(&sub.select);
            absorb(&sub.deselect);
        }
        if let Some(transfer) = &lathe.transfer {
            for lines in [
                &transfer.clamp_main,
                &transfer.unclamp_main,
                &transfer.clamp_sub,
                &transfer.unclamp_sub,
                &transfer.sub_cw,
                &transfer.sub_ccw,
                &transfer.sub_stop,
                &transfer.sync_on,
                &transfer.sync_on_phase,
                &transfer.sync_off,
                &transfer.move_rapid,
                &transfer.move_feed,
                &transfer.move_machine,
            ] {
                absorb(lines);
            }
        }
        if let Some(tail) = &lathe.tailstock {
            absorb(&tail.advance);
            absorb(&tail.retract);
        }
        if let Some(steady) = &lathe.steady_rest {
            absorb(&steady.clamp);
            absorb(&steady.unclamp);
        }
        if let Some(catcher) = &lathe.parts_catcher {
            absorb(&catcher.extend);
            absorb(&catcher.retract);
        }
        if let Some(bar) = &lathe.bar_feed {
            absorb(std::slice::from_ref(&bar.code));
        }
        if let Some(channels) = &lathe.channels {
            absorb(&channels.start);
            absorb(&channels.end);
            absorb(&channels.wait);
        }
        if let Some(b_head) = &lathe.b_head {
            absorb(&b_head.index);
        }
        let tool_offset = lathe.tool_change.iter().any(|line| line.contains("{offset2}"));
        let tool_needs_m6 = lathe.tool_change.iter().any(|line| line.split_whitespace().any(|word| word.eq_ignore_ascii_case("M6")));
        let thread_g = lathe.thread.as_ref().and_then(|thread| g_code_number(&thread.code));
        let thread_lead = lathe.thread.as_ref().and_then(|thread| thread.lead_word.chars().next());
        let thread_feed_g =
            lathe.thread.as_ref().and_then(|thread| if thread.feed_mode.is_empty() { None } else { g_code_number(&thread.feed_mode) });
        let lathe_feed_g = if lathe.feed_mode.is_empty() { None } else { g_code_number(&lathe.feed_mode) };
        if let Some(code) = thread_g {
            known_g.insert(code);
        }
        Self {
            known_g,
            known_m,
            spindle_cw: m_set(&lathe.spindle_cw),
            spindle_ccw: m_set(&lathe.spindle_ccw),
            spindle_off: m_set(&lathe.spindle_off),
            live_fwd: lathe.live_tool.as_ref().map(|live| m_set(&live.forward)).unwrap_or_default(),
            live_rev: lathe.live_tool.as_ref().map(|live| m_set(&live.reverse)).unwrap_or_default(),
            live_stop: lathe.live_tool.as_ref().map(|live| m_set(&live.stop)).unwrap_or_default(),
            c_engage: lathe.c_axis.as_ref().map(|c_axis| m_set(&c_axis.engage)).unwrap_or_default(),
            c_disengage: lathe.c_axis.as_ref().map(|c_axis| m_set(&c_axis.disengage)).unwrap_or_default(),
            brake_clamp: lathe.brake.as_ref().map(|brake| m_set(&brake.clamp)).unwrap_or_default(),
            brake_release: lathe.brake.as_ref().map(|brake| m_set(&brake.release)).unwrap_or_default(),
            sub_brake_clamp: lathe.brake_sub.as_ref().map(|brake| m_set(&brake.clamp)).unwrap_or_default(),
            sub_brake_release: lathe.brake_sub.as_ref().map(|brake| m_set(&brake.release)).unwrap_or_default(),
            tail_advance: lathe.tailstock.as_ref().map(|tail| m_set(&tail.advance)).unwrap_or_default(),
            tail_retract: lathe.tailstock.as_ref().map(|tail| m_set(&tail.retract)).unwrap_or_default(),
            steady_clamp: lathe.steady_rest.as_ref().map(|steady| m_set(&steady.clamp)).unwrap_or_default(),
            steady_release: lathe.steady_rest.as_ref().map(|steady| m_set(&steady.unclamp)).unwrap_or_default(),
            catch_extend: lathe.parts_catcher.as_ref().map(|catcher| m_set(&catcher.extend)).unwrap_or_default(),
            catch_retract: lathe.parts_catcher.as_ref().map(|catcher| m_set(&catcher.retract)).unwrap_or_default(),
            engage_bodies: lathe.c_axis.as_ref().map(|c_axis| c_axis.engage.clone()).unwrap_or_default(),
            disengage_bodies: lathe.c_axis.as_ref().map(|c_axis| c_axis.disengage.clone()).unwrap_or_default(),
            select_mode: lathe.sub_spindle.as_ref().map(|sub| mode_lines(&sub.select)).unwrap_or_default(),
            deselect_mode: lathe.sub_spindle.as_ref().map(|sub| mode_lines(&sub.deselect)).unwrap_or_default(),
            channel_starts: channel_starts(lathe),
            wait_base: lathe.channels.as_ref().map(|channels| channels.wait_base),
            wait_is_m: lathe.channels.as_ref().is_some_and(|channels| channels.wait.iter().any(|line| line.contains("M{code}"))),
            thread_g,
            thread_lead,
            thread_feed_g,
            lathe_feed_g,
            bar_g: lathe.bar_feed.as_ref().and_then(|bar| g_code_number(&bar.code)),
            c_letter: lathe.c_axis.as_ref().and_then(|c_axis| c_axis.letter.chars().next()),
            y_letter: lathe.y_axis.as_ref().and_then(|y_axis| y_axis.letter.chars().next()),
            transfer_letter: transfer_letter(lathe),
            b_head: lathe.b_head.is_some(),
            face_x_diameter: lathe.face_interp.as_ref().is_some_and(|face| face.x_diameter),
            mirror_z: lathe.sub_spindle.as_ref().is_some_and(|sub| sub.mirror_z),
            tool_offset,
            tool_needs_m6,
            face_on: lathe.face_interp.as_ref().map(|face| face.on.clone()).unwrap_or_default(),
            face_off: lathe.face_interp.as_ref().map(|face| face.off.clone()).unwrap_or_default(),
            cyl_on: lathe.cylinder_interp.as_ref().map(|cylinder| cylinder.on.clone()).unwrap_or_default(),
            cyl_off: lathe.cylinder_interp.as_ref().map(|cylinder| cylinder.off.clone()).unwrap_or_default(),
        }
    }
}

fn feed_mode_of(word: &str) -> LatheFeedMode {
    match g_code_number(word) {
        Some(93) => LatheFeedMode::InverseTime,
        Some(95 | 99) => LatheFeedMode::PerRevolution,
        _ => LatheFeedMode::PerMinute,
    }
}

fn mode_lines(lines: &[String]) -> Vec<String> {
    let extras: Vec<String> = lines.iter().filter(|line| g_codes_in(line).iter().any(|code| !matches!(*code, 54..=59))).cloned().collect();
    if extras.is_empty() { lines.to_vec() } else { extras }
}

fn channel_starts(lathe: &LatheOutput) -> Vec<(u16, String)> {
    let Some(channels) = &lathe.channels else {
        return Vec::new();
    };
    let mut starts = Vec::new();
    for number in 1..=channels.count {
        let label = number.to_string();
        for line in &channels.start {
            starts.push((number - 1, line.replace("{channel}", &label)));
        }
    }
    starts
}

fn transfer_letter(lathe: &LatheOutput) -> Option<char> {
    let transfer = lathe.transfer.as_ref()?;
    for lines in [&transfer.move_rapid, &transfer.move_feed, &transfer.move_machine] {
        for line in lines {
            let Some(index) = line.find("{z}") else {
                continue;
            };
            let letter = line[..index].chars().next_back()?;
            if letter.is_ascii_alphabetic() {
                return Some(letter.to_ascii_uppercase());
            }
        }
    }
    None
}

fn m_set(lines: &[String]) -> BTreeSet<u16> {
    lines.iter().flat_map(|line| m_codes_in(line)).collect()
}

fn g_codes_in(line: &str) -> Vec<u16> {
    codes_in(line, b'G')
}

fn m_codes_in(line: &str) -> Vec<u16> {
    codes_in(line, b'M')
}

fn codes_in(line: &str, letter: u8) -> Vec<u16> {
    let bytes = line.as_bytes();
    let mut index = 0;
    let mut codes = Vec::new();
    while index < bytes.len() {
        if bytes[index].eq_ignore_ascii_case(&letter) {
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
    codes
}

fn g_hits(g_codes: &[u16], lines: &[String]) -> bool {
    let needles = lines.iter().flat_map(|line| g_codes_in(line)).collect::<Vec<_>>();
    g_codes.iter().any(|code| needles.contains(code))
}

fn face_on_hit(g_codes: &[u16], lines: &[String]) -> bool {
    let needles = lines.iter().flat_map(|line| g_codes_in(line)).collect::<Vec<_>>();
    let shares_plane = needles.iter().any(|code| matches!(*code, 17..=19));
    let has_mode = needles.iter().any(|code| !matches!(*code, 17..=19));
    g_codes.iter().any(|code| {
        if shares_plane && has_mode && matches!(*code, 17..=19) {
            return false;
        }
        needles.contains(code)
    })
}

fn mapping_letter(lines: &[String]) -> Option<char> {
    for line in lines {
        for key in ["{radius}", "{diameter}"] {
            if let Some(at) = line.find(key) {
                let letter = line[..at].chars().rev().find(|ch| ch.is_ascii_alphabetic())?;
                return Some(letter.to_ascii_uppercase());
            }
        }
    }
    None
}

fn mapping_present(lines: &[String], block: &Block) -> Result<bool> {
    let Some(letter) = mapping_letter(lines) else {
        return Ok(true);
    };
    let present = match letter {
        'R' => block.r.is_some(),
        'X' => block.x.is_some(),
        'Y' => block.y.is_some(),
        'Z' => block.z.is_some(),
        'I' => block.i.is_some(),
        'J' => block.j.is_some(),
        'K' => block.k.is_some(),
        'C' => block.c.is_some(),
        'D' => block.d.is_some(),
        _ => false,
    };
    Ok(present)
}

fn is_part_motion(move_: &ReadMove) -> bool {
    matches!(move_, ReadMove::Rapid { .. } | ReadMove::Linear { .. } | ReadMove::Arc { .. } | ReadMove::Thread { .. })
}

fn end_of(move_: &ReadMove) -> Xyz {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } | ReadMove::Thread { end, .. } => *end,
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

fn dist3(a: Xyz, b: Xyz) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}

fn flip_sense(sense: ArcSense) -> ArcSense {
    match sense {
        ArcSense::Cw => ArcSense::Ccw,
        ArcSense::Ccw => ArcSense::Cw,
    }
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

fn set_unique(line_no: usize, slot: &mut Option<f64>, word: &Word) -> Result<()> {
    if slot.is_some() {
        return Err(fail(line_no, format!("duplicate {} word", word.letter)));
    }
    *slot = Some(word.value);
    Ok(())
}

fn axis_letters(post: &PostDef) -> [Option<char>; 3] {
    let mut chars = post.axis_letters.chars();
    [chars.next(), chars.next(), chars.next()]
}

fn block_body(post: &PostDef, raw: &str) -> String {
    let raw = raw.trim();
    if !post.line_numbers.enabled {
        return raw.to_string();
    }
    let Some(rest) = raw.strip_prefix(post.line_numbers.prefix.as_str()) else {
        return raw.to_string();
    };
    let rest = rest.trim_start();
    let after = rest.trim_start_matches(|ch: char| ch.is_ascii_digit());
    if after.len() == rest.len() { raw.to_string() } else { after.trim().to_string() }
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

fn lex(line: &str, line_no: usize, parser: &Parser<'_>) -> Result<Vec<Word>> {
    let bytes = line.as_bytes();
    let mut index = 0;
    let mut words = Vec::new();
    while index < bytes.len() {
        if bytes[index].is_ascii_whitespace() {
            index += 1;
            continue;
        }
        if bytes[index] == b'$' {
            index += 1;
            let (value, raw) = take_number(line, bytes, &mut index, line_no)?;
            check_number(line_no, parser.post, '$', value, &raw)?;
            words.push(Word { letter: '$', value, raw });
            continue;
        }
        let letter = bytes[index];
        if !letter.is_ascii_alphabetic() {
            return Err(fail(line_no, "unexpected character in g-code"));
        }
        let letter = (letter as char).to_ascii_uppercase();
        index += 1;
        if !letter_allowed(parser, letter) {
            return Err(fail(line_no, format!("unexpected word {letter}")));
        }
        let (value, raw) = take_number(line, bytes, &mut index, line_no)?;
        check_number(line_no, parser.post, letter, value, &raw)?;
        words.push(Word { letter, value, raw });
    }
    Ok(words)
}

fn letter_allowed(parser: &Parser<'_>, letter: char) -> bool {
    if parser.post.axis_letters.contains(letter)
        || matches!(letter, 'G' | 'M' | 'F' | 'S' | 'T' | 'H' | 'P' | 'I' | 'J' | 'K' | 'R' | 'D' | 'U' | 'W')
    {
        return true;
    }
    if letter == 'N' && parser.post.line_numbers.enabled {
        return true;
    }
    if parser.codes.c_letter == Some(letter) || parser.codes.y_letter == Some(letter) {
        return true;
    }
    if letter == 'B' && (parser.codes.b_head || parser.codes.transfer_letter == Some('B')) {
        return true;
    }
    parser.post.dwell.word.chars().next().is_some_and(|dwell| dwell == letter)
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
    if post.decimal_point && dimensional(letter) && value != 0.0 && !raw.contains('.') {
        return Err(fail(line_no, format!("dimensional word {letter} is missing a decimal point")));
    }
    Ok(())
}

fn dimensional(letter: char) -> bool {
    matches!(letter, 'X' | 'Y' | 'Z' | 'I' | 'J' | 'K' | 'C')
}

fn is_negative_zero(raw: &str) -> bool {
    let Some(rest) = raw.strip_prefix('-') else {
        return false;
    };
    !rest.is_empty() && rest.bytes().all(|byte| byte == b'0' || byte == b'.')
}
