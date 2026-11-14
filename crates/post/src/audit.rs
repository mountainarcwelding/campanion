//! Modal replay of posted mill G-code.
//!
//! [`audit`] reads the text a post wrote and tracks the modal state a control would keep:
//! motion, plane, distance, units, feed mode, the active feed, work offset, the tool in the
//! spindle, tool length, cutter compensation (`G40`/`G41`/`G42`), spindle or beam, and coolant.
//! Each motion block is one [`MotionState`].
//!
//! A word this checker does not know is an error, so a typo in a template fails here. Geometry
//! is also checked against [`crate::read_back`] so the two readers stay aligned.
//! `G91.1` confirms incremental IJK without changing positioning; `G90.1` is rejected.
//! `G53` is a machine-coordinate move, not a part point; it preserves part position, motion
//! mode and plane, matching read-back.
//!
//! Lathe posts are rejected; lathe and mill-turn text is replayed by [`crate::audit_lathe`].
//! Five-axis posts are replayed by [`crate::audit_five`]. Work offset
//! stays unset when the program never writes `G54`–`G59`. Feed mode stays
//! units per minute until `G93` or `G94`. `M2` and `M30` turn the spindle and coolant off.
//! `G41` and `G42` select cutter compensation; `G40` cancels it. A `D` word is accepted only on a
//! `G41` or `G42` block. Compensation must be off at the end of the program (`G40`, or `M2`/`M30`).
//! `M3` or `M4` with `S0` is running with the beam or torch off: [`SpindleState::cutting`] is false.
//! A post with `probe` treats its `G38.2` or `G38.3` as a non-motion: position and motion mode
//! stay put, and `F` on that block updates the modal feed. A post with `digital_out` accepts
//! `M64` and `M65` when `P` is a declared auxiliary pin (`vacuum_pin`, `dust_pin`, `abrasive_pin`, `preheat_pin`, or `cutting_oxygen_pin`). Other posts still reject those words.

use campanion_ir::{ArcSense, Plane, RotaryAxisKind};

use crate::def::PostDef;
use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};
use crate::read::{ReadMove, read_back};

const MAX_ABS: f64 = 1.0e9;
const LINE_NUMBER_LIMIT: u64 = 1_000_000_000;

/// `G0`, `G1`, `G2`, or `G3`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionMode {
    /// `G0`.
    Rapid,
    /// `G1`.
    Linear,
    /// `G2`.
    Cw,
    /// `G3`.
    Ccw,
}

/// `G90` or `G91`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DistanceMode {
    /// `G90`.
    Absolute,
    /// `G91`.
    Incremental,
}

/// `G20` or `G21`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnitMode {
    /// `G20`.
    Inch,
    /// `G21`.
    Millimetre,
}

/// `G93` or `G94`. Power-up is units per minute.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedMode {
    /// Millimetres per minute.
    PerMinute,
    /// `F` is `1 / minutes` on this block.
    InverseTime,
}

/// `G54` through `G59`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkOffset {
    /// `G54`.
    G54,
    /// `G55`.
    G55,
    /// `G56`.
    G56,
    /// `G57`.
    G57,
    /// `G58`.
    G58,
    /// `G59`.
    G59,
}

/// Tool-length state. `G49` cancels it. `G43 Hn` selects offset `n`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LengthOffset {
    /// No length word yet.
    Unset,
    /// `G49`.
    Cancelled,
    /// `G43` with this `H` number.
    Active(u32),
}

/// Coolant from `M7`, `M8`, and `M9`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoolantState {
    /// `M9`, or no coolant word yet.
    Off,
    /// `M8`.
    Flood,
    /// `M7`.
    Mist,
}

/// Spindle, torch, or beam.
///
/// `on` follows `M3` / `M4` / `M5`. `speed` is the latest `S` word. A laser header `M4 S0` is on
/// with speed 0, which is not cutting.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpindleState {
    /// `M3` or `M4` is in effect.
    pub on: bool,
    /// `M3` is clockwise. `M4` is not. Meaningless while off.
    pub clockwise: bool,
    /// Latest `S` word, or 0 when none has been written.
    pub speed: f64,
}

impl SpindleState {
    /// True when the process is on and `S` is greater than zero.
    pub fn cutting(self) -> bool {
        self.on && self.speed > 0.0
    }

    pub(crate) fn off() -> Self {
        Self { on: false, clockwise: true, speed: 0.0 }
    }
}

/// Arc data recovered from one `G2` or `G3` block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcReplay {
    /// `G2` or `G3`.
    pub sense: ArcSense,
    /// Absolute centre in millimetres.
    pub center: [f64; 3],
    /// Signed radius in millimetres when the block used `R`.
    pub radius: Option<f64>,
}

/// Modal state after one motion block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionState {
    /// 1-based line number in the posted text.
    pub line: usize,
    /// Motion mode in effect for this block.
    pub mode: MotionMode,
    /// Selected plane, after any `G17` / `G18` / `G19` on this block.
    pub plane: Option<Plane>,
    /// Distance mode.
    pub distance: DistanceMode,
    /// Units.
    pub units: UnitMode,
    /// Feed mode after this block.
    pub feed_mode: FeedMode,
    /// Active feed in mm/min (G94) or reciprocal minutes (G93), including a modal `F`.
    pub feed: Option<f64>,
    /// This block carried its own `F` word.
    pub feed_on_block: bool,
    /// Work offset. `None` when the program has not written `G54`–`G59`.
    pub work: Option<WorkOffset>,
    /// Tool in the spindle after `T` + `M6`, or the post's tool-change comment.
    pub tool: Option<u32>,
    /// Tool-length offset.
    pub length: LengthOffset,
    /// Spindle, torch, or beam.
    pub spindle: SpindleState,
    /// Coolant.
    pub coolant: CoolantState,
    /// Absolute end in millimetres, in the frame the post wrote. Omitted axes retain their value.
    pub end: [f64; 3],
    /// Set on `G2` and `G3`.
    pub arc: Option<ArcReplay>,
    /// Modal rotary angle, degrees, when the block or an earlier block wrote one.
    pub rotary: Option<f64>,
}

/// Cutter radius compensation. Power-up and the usual header are [`Self::Off`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CutterComp {
    /// `G40`.
    Off,
    /// `G41`, tool left of travel.
    Left,
    /// `G42`, tool right of travel.
    Right,
}

/// Modal replay of one posted program.
#[derive(Clone, Debug, PartialEq)]
pub struct AuditReport {
    /// One entry per motion block, in order.
    pub motions: Vec<MotionState>,
    /// Spindle after the last line. `M2` and `M30` turn it off.
    pub spindle: SpindleState,
    /// Coolant after the last line. `M2` and `M30` turn it off.
    pub coolant: CoolantState,
    /// Tool left in the spindle.
    pub tool: Option<u32>,
    /// Units, once a `G20` or `G21` has been seen.
    pub units: Option<UnitMode>,
    /// Work offset, or `None` when the post never selects one.
    pub work: Option<WorkOffset>,
    /// Distance mode, once `G90` or `G91` has been seen.
    pub distance: Option<DistanceMode>,
    /// Tool-length offset.
    pub length: LengthOffset,
    /// Feed mode after the last line.
    pub feed_mode: FeedMode,
}

/// Replay `text` as the control named by `post` would read it.
///
/// Mill and five-axis posts. A lathe post is rejected. Unknown letters, unknown `G` and `M`
/// codes, a `G2`/`G3` without a plane and a full centre or radius, a feed move before a positive
/// `F`, and an inverse-time feed block with no `F` on that block are errors. A straight-probe
/// block (`G38.2` or `G38.3` on a post that sets `probe`) is not a motion. `M64` and `M65` are
/// accepted only when the post sets `digital_out` and `P` is one of its pins. Line-number
/// sequence, line length, Fanuc/Haas decimal points, negative zero, and word magnitudes above
/// `1e9` are checked as the text is read. Five-axis text is replayed by [`crate::audit_five`]
/// and returned as the same mill report, with `A` in [`MotionState::rotary`].
pub fn audit(text: &str, post: &PostDef) -> Result<AuditReport> {
    if post.is_lathe() {
        return Err(invalid("audit reads mill posts"));
    }
    if post.has_five_axis() {
        return crate::audit_5ax::audit_as_mill(text, post);
    }
    if text.len() > 64 * 1024 * 1024 {
        return Err(invalid("g-code is too large"));
    }
    let mut parser = Parser::new(post);
    for (index, line) in text.lines().enumerate() {
        if index > 2_000_000 {
            return Err(invalid("g-code has too many lines"));
        }
        parser.line(index + 1, line)?;
    }
    parser.finish()?;
    parser.cross_check(text, post)?;
    Ok(parser.report())
}

struct Word {
    letter: char,
    value: f64,
    raw: String,
}

struct Parser<'a> {
    post: &'a PostDef,
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
    cutter_comp: CutterComp,
    rotary: Option<f64>,
    next_number: Option<u64>,
    tool_patterns: Vec<(String, String)>,
    motions: Vec<MotionState>,
}

impl<'a> Parser<'a> {
    fn is_probe_word(&self, word: &Word) -> bool {
        self.post.probe_number().is_some_and(|code| (word.value - code).abs() < 1e-6)
    }

    fn is_immediate_digital(&self, word: &Word) -> bool {
        if self.post.digital_out.is_none() {
            return false;
        }
        if !(word.value.is_finite() && (0.0..=999.0).contains(&word.value)) {
            return false;
        }
        let rounded = word.value.round();
        if (word.value - rounded).abs() > 1e-6 {
            return false;
        }
        matches!(rounded as u16, 64 | 65)
    }

    fn new(post: &'a PostDef) -> Self {
        let next_number = post.line_numbers.enabled.then_some(u64::from(post.line_numbers.start));
        Self {
            post,
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
            cutter_comp: CutterComp::Off,
            rotary: None,
            next_number,
            tool_patterns: tool_patterns(post),
            motions: Vec::new(),
        }
    }

    fn finish(&self) -> Result<()> {
        if self.cutter_comp != CutterComp::Off {
            return Err(invalid("cutter compensation is not cancelled"));
        }
        Ok(())
    }

    fn report(self) -> AuditReport {
        AuditReport {
            motions: self.motions,
            spindle: self.spindle,
            coolant: self.coolant,
            tool: self.tool,
            units: self.units,
            work: self.work,
            distance: self.distance,
            length: self.length,
            feed_mode: self.feed_mode,
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
            if raw.trim().is_empty() {
                return Ok(());
            }
            if self.post.line_numbers.enabled {
                return Err(fail(line_no, "line is missing its sequence number"));
            }
            return Ok(());
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
        self.interpret(line_no, &words)
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

    fn interpret(&mut self, line_no: usize, words: &[Word]) -> Result<()> {
        let previous_feed_mode = self.feed_mode;
        let previous_motion = self.motion;
        let previous_plane = self.plane;
        let mut machine_move = false;
        let mut saw_dwell = false;
        let mut want_length = false;
        let mut h_word: Option<u32> = None;
        let mut block_motion: Option<MotionMode> = None;
        let mut block_plane: Option<Plane> = None;
        let mut x: Option<f64> = None;
        let mut y: Option<f64> = None;
        let mut z: Option<f64> = None;
        let mut i: Option<f64> = None;
        let mut j: Option<f64> = None;
        let mut k: Option<f64> = None;
        let mut r: Option<f64> = None;
        let mut feed_on_block = false;
        let mut saw_rotary = false;
        let mut saw_s = false;
        let mut p_value: Option<f64> = None;
        let mut probe_words = 0u32;
        let mut digital = false;
        let mut block_feed: Option<FeedMode> = None;
        let mut block_comp: Option<CutterComp> = None;
        let mut d_word: Option<u32> = None;
        let axes = axis_letters(self.post);

        for word in words {
            match word.letter {
                // IJK is already incremental; this declaration does not change G90/G91.
                'G' if word.value == 91.1 => {}
                'G' if word.value == 90.1 => {
                    return Err(fail(line_no, "G90.1 absolute arc centres are not supported; use G91.1 for incremental IJK"));
                }
                'G' if self.is_probe_word(word) => probe_words += 1,
                'G' => {
                    let code = g_or_m(line_no, word, 'G')?;
                    machine_move |= code == 53;
                    self.apply_g(
                        line_no,
                        code,
                        &mut saw_dwell,
                        &mut want_length,
                        &mut block_motion,
                        &mut block_plane,
                        &mut block_feed,
                        &mut block_comp,
                    )?;
                }
                'M' if self.is_immediate_digital(word) => {
                    if digital {
                        return Err(fail(line_no, "duplicate digital output"));
                    }
                    digital = true;
                }
                'M' => {
                    let code = g_or_m(line_no, word, 'M')?;
                    self.apply_m(line_no, code)?;
                }
                'F' => {
                    if feed_on_block {
                        return Err(fail(line_no, "duplicate F word"));
                    }
                    if !(word.value.is_finite() && word.value > 0.0) {
                        return Err(fail(line_no, "feed is not positive"));
                    }
                    self.feed = Some(word.value);
                    feed_on_block = true;
                }
                'S' => {
                    if saw_s {
                        return Err(fail(line_no, "duplicate S word"));
                    }
                    if word.value < 0.0 {
                        return Err(fail(line_no, "S word must be >= 0"));
                    }
                    saw_s = true;
                    self.spindle.speed = word.value;
                }
                'T' => {
                    let tool = whole_word(line_no, word, 'T')?;
                    self.pending_tool = Some(tool);
                }
                'H' => {
                    if h_word.is_some() {
                        return Err(fail(line_no, "duplicate H word"));
                    }
                    h_word = Some(whole_word(line_no, word, 'H')?);
                }
                'D' => {
                    if d_word.is_some() {
                        return Err(fail(line_no, "duplicate D word"));
                    }
                    d_word = Some(whole_word(line_no, word, 'D')?);
                }
                'P' => {
                    if p_value.is_some() {
                        return Err(fail(line_no, "duplicate P word"));
                    }
                    if !(word.value.is_finite() && word.value >= 0.0) {
                        return Err(fail(line_no, "P word must be >= 0"));
                    }
                    p_value = Some(word.value);
                }
                'I' => set_unique(line_no, &mut i, word)?,
                'J' => set_unique(line_no, &mut j, word)?,
                'K' => set_unique(line_no, &mut k, word)?,
                'R' => set_unique(line_no, &mut r, word)?,
                'A' | 'B' => {
                    if self.rotary.is_some() && saw_rotary {
                        return Err(fail(line_no, format!("duplicate {} word", word.letter)));
                    }
                    self.rotary = Some(word.value);
                    saw_rotary = true;
                }
                letter if Some(letter) == axes[0] => set_unique(line_no, &mut x, word)?,
                letter if Some(letter) == axes[1] => set_unique(line_no, &mut y, word)?,
                letter if Some(letter) == axes[2] => set_unique(line_no, &mut z, word)?,
                letter => return Err(fail(line_no, format!("unexpected word {letter}"))),
            }
        }

        if d_word.is_some() && !matches!(block_comp, Some(CutterComp::Left | CutterComp::Right)) {
            return Err(fail(line_no, "D word needs G41 or G42"));
        }

        // Resolve modal G words before normalizing this block's lengths and feed.
        if self.post.is_inch() {
            if previous_feed_mode != self.feed_mode && !feed_on_block {
                self.feed = None;
            }
            if self.units.is_none()
                && (feed_on_block || x.is_some() || y.is_some() || z.is_some() || i.is_some() || j.is_some() || k.is_some() || r.is_some())
            {
                return Err(fail(line_no, "length or feed before units are set"));
            }
            for v in [&mut x, &mut y, &mut z, &mut i, &mut j, &mut k, &mut r].into_iter().flatten() {
                *v *= self.post.length_scale();
                if !v.is_finite() {
                    return Err(fail(line_no, "scaled length is not finite"));
                }
            }
            if feed_on_block && self.feed_mode == FeedMode::PerMinute {
                self.feed = self.feed.map(|f| f * self.post.length_scale());
                if self.feed.is_some_and(|f| !f.is_finite()) {
                    return Err(fail(line_no, "scaled feed is not finite"));
                }
            }
            if saw_rotary {
                return Err(fail(line_no, "inch rotary motion is not supported"));
            }
        }

        if digital {
            let Some(pin) = p_value else {
                return Err(fail(line_no, "digital output needs a P word"));
            };
            if !self.post.digital_pin(pin) {
                return Err(fail(line_no, "digital output P is not a declared pin"));
            }
        }
        if probe_words > 0 {
            if probe_words > 1 || block_motion.is_some() || saw_dwell || i.is_some() || j.is_some() || k.is_some() || r.is_some() || saw_rotary {
                return Err(fail(line_no, "probe cannot share a block with another motion"));
            }
            if x.is_none() && y.is_none() && z.is_none() {
                return Err(fail(line_no, "probe needs an axis word"));
            }
            if !feed_on_block {
                return Err(fail(line_no, "probe needs a feed"));
            }
            return Ok(());
        }

        if want_length {
            let Some(h) = h_word else {
                return Err(fail(line_no, "G43 needs an H word"));
            };
            self.length = LengthOffset::Active(h);
        }
        if saw_dwell {
            if self.post.dwell.decimal_seconds {
                if words.iter().filter(|word| word.letter == 'G').count() != 1 {
                    return Err(fail(line_no, "dwell must be the only G code in its block"));
                }
                // P was checked for finiteness, sign and duplication above. The audit
                // validates the block; read_back reports its decimal-sensitive duration.
                match p_value {
                    None => return Err(fail(line_no, "dwell is missing its value")),
                    Some(0.0) => return Err(fail(line_no, "literal-P dwell must be greater than zero")),
                    Some(_) => {}
                }
                if i.is_some() || j.is_some() || k.is_some() || r.is_some() {
                    return Err(fail(line_no, "literal-P dwell does not accept arc or cycle words"));
                }
            }
            if block_motion.is_some() || x.is_some() || y.is_some() || z.is_some() || saw_rotary {
                return Err(fail(line_no, "dwell cannot share a block with motion"));
            }
            return Ok(());
        }

        if machine_move {
            // Validate the whole block first, but do not turn machine coordinates into a
            // part-frame point or let its G0/G1 replace the part's modal motion.
            self.motion = previous_motion;
            self.plane = previous_plane;
            return Ok(());
        }

        let has_axis = x.is_some() || y.is_some() || z.is_some() || saw_rotary;
        if !has_axis {
            return Ok(());
        }
        let Some(mode) = self.motion else {
            return Err(fail(line_no, "motion has no mode"));
        };
        let Some(distance) = self.distance else {
            return Err(fail(line_no, "motion before distance mode is set"));
        };
        if distance != DistanceMode::Absolute {
            return Err(fail(line_no, "incremental motion is not supported"));
        }
        let Some(units) = self.units else {
            return Err(fail(line_no, "motion before units are set"));
        };
        let mut end = self.pos;
        if let Some(value) = x {
            end[0] = value;
        }
        if let Some(value) = y {
            end[1] = value;
        }
        if let Some(value) = z {
            end[2] = value;
        }

        let feed_move = !matches!(mode, MotionMode::Rapid);
        if feed_move {
            match self.feed {
                Some(feed) if feed > 0.0 => {}
                _ => return Err(fail(line_no, "feed move has no positive feed")),
            }
            if self.feed_mode == FeedMode::InverseTime && !feed_on_block {
                return Err(fail(line_no, "inverse-time block has no F"));
            }
        }

        let arc = if matches!(mode, MotionMode::Cw | MotionMode::Ccw) {
            Some(self.arc_spec(line_no, mode, end, i, j, k, r)?)
        } else if i.is_some() || j.is_some() || k.is_some() || r.is_some() {
            return Err(fail(line_no, "centre word on a non-arc block"));
        } else {
            None
        };

        let state = MotionState {
            line: line_no,
            mode,
            plane: self.plane,
            distance,
            units,
            feed_mode: self.feed_mode,
            feed: self.feed,
            feed_on_block,
            work: self.work,
            tool: self.tool,
            length: self.length,
            spindle: self.spindle,
            coolant: self.coolant,
            end,
            arc,
            rotary: self.rotary,
        };
        self.pos = end;
        self.motions.push(state);
        Ok(())
    }

    fn apply_g(
        &mut self,
        line_no: usize,
        code: u16,
        saw_dwell: &mut bool,
        want_length: &mut bool,
        block_motion: &mut Option<MotionMode>,
        block_plane: &mut Option<Plane>,
        block_feed: &mut Option<FeedMode>,
        block_comp: &mut Option<CutterComp>,
    ) -> Result<()> {
        match code {
            0 => self.set_motion(line_no, MotionMode::Rapid, block_motion)?,
            1 => self.set_motion(line_no, MotionMode::Linear, block_motion)?,
            2 => self.set_motion(line_no, MotionMode::Cw, block_motion)?,
            3 => self.set_motion(line_no, MotionMode::Ccw, block_motion)?,
            4 => *saw_dwell = true,
            17 => self.set_plane(line_no, Plane::Xy, block_plane)?,
            18 => self.set_plane(line_no, Plane::Zx, block_plane)?,
            19 => self.set_plane(line_no, Plane::Yz, block_plane)?,
            20 => self.set_units(line_no, UnitMode::Inch)?,
            21 => self.set_units(line_no, UnitMode::Millimetre)?,
            40 => self.set_cutter_comp(line_no, CutterComp::Off, block_comp)?,
            41 => self.set_cutter_comp(line_no, CutterComp::Left, block_comp)?,
            42 => self.set_cutter_comp(line_no, CutterComp::Right, block_comp)?,
            53 | 64 | 80 => {}
            43 => *want_length = true,
            49 => {
                if *want_length {
                    return Err(fail(line_no, "line has conflicting G codes"));
                }
                self.length = LengthOffset::Cancelled;
            }
            54 => self.set_work(line_no, WorkOffset::G54)?,
            55 => self.set_work(line_no, WorkOffset::G55)?,
            56 => self.set_work(line_no, WorkOffset::G56)?,
            57 => self.set_work(line_no, WorkOffset::G57)?,
            58 => self.set_work(line_no, WorkOffset::G58)?,
            59 => self.set_work(line_no, WorkOffset::G59)?,
            90 => self.set_distance(line_no, DistanceMode::Absolute)?,
            91 => return Err(fail(line_no, "incremental motion is not supported")),
            93 => self.set_feed_mode(line_no, FeedMode::InverseTime, block_feed)?,
            94 => self.set_feed_mode(line_no, FeedMode::PerMinute, block_feed)?,
            other => return Err(fail(line_no, format!("unsupported G{other}"))),
        }
        Ok(())
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
        if (units == UnitMode::Inch) != self.post.is_inch() {
            return Err(fail(line_no, "units disagree with post"));
        }
        if let Some(prev) = self.units
            && prev != units
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        self.units = Some(units);
        Ok(())
    }

    fn set_cutter_comp(&mut self, line_no: usize, mode: CutterComp, block: &mut Option<CutterComp>) -> Result<()> {
        if let Some(prev) = *block
            && prev != mode
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        *block = Some(mode);
        self.cutter_comp = mode;
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

    fn set_feed_mode(&mut self, line_no: usize, mode: FeedMode, block: &mut Option<FeedMode>) -> Result<()> {
        if let Some(prev) = *block
            && prev != mode
        {
            return Err(fail(line_no, "line has conflicting G codes"));
        }
        *block = Some(mode);
        self.feed_mode = mode;
        Ok(())
    }

    fn apply_m(&mut self, line_no: usize, code: u16) -> Result<()> {
        match code {
            2 | 30 => {
                self.spindle.on = false;
                self.coolant = CoolantState::Off;
                self.cutter_comp = CutterComp::Off;
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
            other => return Err(fail(line_no, format!("unsupported M{other}"))),
        }
        Ok(())
    }

    fn arc_spec(
        &self,
        line_no: usize,
        mode: MotionMode,
        end: Xyz,
        i: Option<f64>,
        j: Option<f64>,
        k: Option<f64>,
        r: Option<f64>,
    ) -> Result<ArcReplay> {
        let Some(plane) = self.plane else {
            return Err(fail(line_no, "arc has no plane"));
        };
        if r.is_some() && (i.is_some() || j.is_some() || k.is_some()) {
            return Err(fail(line_no, "arc has both R and IJK"));
        }
        let sense = match mode {
            MotionMode::Cw => ArcSense::Cw,
            MotionMode::Ccw => ArcSense::Ccw,
            MotionMode::Rapid | MotionMode::Linear => return Err(fail(line_no, "arc has no plane")),
        };
        let (center, radius) = if let Some(radius) = r {
            if radius == 0.0 {
                return Err(fail(line_no, "arc radius must be non-zero"));
            }
            let quantum = self.post.linear_quantum();
            let center = geom::center_from_radius(self.pos, end, radius, sense, plane, quantum)?;
            (center, Some(radius))
        } else {
            let (need_i, need_j, need_k) = match plane {
                Plane::Xy => (true, true, false),
                Plane::Zx => (true, false, true),
                Plane::Yz => (false, true, true),
            };
            if i.is_some() != need_i || j.is_some() != need_j || k.is_some() != need_k {
                return Err(fail(line_no, "arc is missing a full centre specification"));
            }
            let center = [self.pos[0] + i.unwrap_or(0.0), self.pos[1] + j.unwrap_or(0.0), self.pos[2] + k.unwrap_or(0.0)];
            (center, None)
        };
        Ok(ArcReplay { sense, center, radius })
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
            let (read_feed, inverse, sense, center) = match move_ {
                ReadMove::Rapid { feed, inverse_time, .. } => {
                    if state.mode != MotionMode::Rapid {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, None, None)
                }
                ReadMove::Linear { feed, inverse_time, .. } => {
                    if state.mode != MotionMode::Linear {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, None, None)
                }
                ReadMove::Arc { feed, inverse_time, sense, center, .. } => {
                    let mode = match sense {
                        ArcSense::Cw => MotionMode::Cw,
                        ArcSense::Ccw => MotionMode::Ccw,
                    };
                    if state.mode != mode {
                        return Err(invalid(format!("line {}: audit motion does not match read_back", state.line)));
                    }
                    (*feed, *inverse_time, Some(*sense), Some(*center))
                }
                ReadMove::Dwell { .. } | ReadMove::Thread { .. } | ReadMove::Other { .. } => continue,
            };
            if feed_gap(state.feed, read_feed) > 1e-6 {
                return Err(invalid(format!("line {}: audit feed does not match read_back", state.line)));
            }
            let inverse_state = state.feed_mode == FeedMode::InverseTime;
            if inverse_state != inverse {
                return Err(invalid(format!("line {}: audit feed mode does not match read_back", state.line)));
            }
            match (state.arc, sense, center) {
                (Some(arc), Some(sense), Some(center)) if arc.sense != sense || dist3(arc.center, center) > 1e-4 => {
                    return Err(invalid(format!("line {}: audit arc does not match read_back", state.line)));
                }
                (Some(_), Some(_), Some(_)) | (None, None, None) => {}
                _ => return Err(invalid(format!("line {}: audit arc does not match read_back", state.line))),
            }
        }
        check_missing_power_s_words(text)?;
        Ok(())
    }
}

/// A comment that records a defaulted (not explicit) process power.
fn power_not_set_comment(text: &str) -> bool {
    text.contains("power was not set") || text.contains("power missing")
}

/// First `S` word above zero, ignoring comments. Used when power was not set explicitly.
fn first_s_above_zero(text: &str) -> Option<(usize, String)> {
    for (index, line) in text.lines().enumerate() {
        for word in strip_comments(line).split_whitespace() {
            let Some(rest) = word.strip_prefix('S').or_else(|| word.strip_prefix('s')) else {
                continue;
            };
            if let Ok(value) = rest.parse::<f64>()
                && value > 0.0
            {
                return Some((index + 1, word.to_string()));
            }
        }
    }
    None
}

fn check_missing_power_s_words(text: &str) -> Result<()> {
    if !power_not_set_comment(text) {
        return Ok(());
    }
    if let Some((line_no, word)) = first_s_above_zero(text) {
        return Err(fail(line_no, format!("{word} is above 0, but power was not set")));
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
    if post.axis_letters.contains(letter) || matches!(letter, 'G' | 'M' | 'F' | 'S' | 'T' | 'H' | 'D' | 'P' | 'I' | 'J' | 'K' | 'R') {
        return true;
    }
    if letter == 'N' && post.line_numbers.enabled {
        return true;
    }
    let rotary = post.rotary_axes();
    if letter == 'A' && rotary.contains(&RotaryAxisKind::A) {
        return true;
    }
    if letter == 'B' && rotary.contains(&RotaryAxisKind::B) {
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
    post.axis_letters.contains(letter)
        || matches!(letter, 'I' | 'J' | 'K' | 'R')
        || (letter == 'A' && post.rotary_axes().contains(&RotaryAxisKind::A))
        || (letter == 'B' && post.rotary_axes().contains(&RotaryAxisKind::B))
}

fn is_negative_zero(raw: &str) -> bool {
    let Some(rest) = raw.strip_prefix('-') else {
        return false;
    };
    !rest.is_empty() && rest.bytes().all(|byte| byte == b'0' || byte == b'.')
}

#[cfg(test)]
mod tests {
    use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Pose, ProcessKind, ProcessParams, Program, Tool, Vec3};

    use super::audit;
    use crate::{PostOptions, builtin, post_program};

    fn grbl() -> crate::PostDef {
        builtin("grbl").expect("grbl")
    }

    #[test]
    fn unknown_word_is_an_error() {
        let err = audit("G21\nG90\nG17\nQ1\n", &grbl()).expect_err("q");
        assert!(err.to_string().contains("unexpected word Q"), "{err}");
        let err = audit("G21\nG90\nG99\n", &grbl()).expect_err("g99");
        assert!(err.to_string().contains("unsupported G99"), "{err}");
        let err = audit("G21\nG90\nM99\n", &grbl()).expect_err("m99");
        assert!(err.to_string().contains("unsupported M99"), "{err}");
    }

    #[test]
    fn feed_must_be_positive_and_present() {
        let err = audit("G21\nG90\nG17\nG1 X1 F0\n", &grbl()).expect_err("zero");
        assert!(err.to_string().contains("feed is not positive"), "{err}");
        let err = audit("G21\nG90\nG17\nG1 X1\n", &grbl()).expect_err("missing");
        assert!(err.to_string().contains("feed move has no positive feed"), "{err}");
    }

    #[test]
    fn arc_needs_a_plane_and_a_full_centre() {
        let err = audit("G21\nG90\nG17\nG1 X1 F100\nG2 X1 Y1 I0\n", &grbl()).expect_err("centre");
        assert!(err.to_string().contains("full centre"), "{err}");
        let err = audit("G21\nG90\nG1 X1 F100\nG2 X0 Y1 R1\n", &grbl()).expect_err("plane");
        assert!(err.to_string().contains("arc has no plane"), "{err}");
        let err = audit("G21\nG90\nG0 X-0 Y0\n", &grbl()).expect_err("neg zero");
        assert!(err.to_string().contains("negative zero"), "{err}");
    }

    #[test]
    fn fanuc_dimensional_words_need_a_decimal_point() {
        let fanuc = builtin("fanuc").expect("fanuc");
        let bad = "%\nO0001\nN10 G21 G90 G17 G40 G49 G80\nN20 G0 X1 Y0. Z0.\nN30 M30\n%\n";
        let err = audit(bad, &fanuc).expect_err("point");
        assert!(err.to_string().contains("decimal point"), "{err}");
        let good = "%\nO0001\nN10 G21 G90 G17 G40 G49 G80\nN20 G0 X1. Y0. Z0.\nN30 M30\n%\n";
        let report = audit(good, &fanuc).expect("good");
        assert_eq!(report.motions.len(), 1);
        assert_eq!(report.motions[0].length, super::LengthOffset::Cancelled);
        assert!(report.motions[0].work.is_none());
    }

    #[test]
    fn posted_mill_programs_replay() {
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let start = Pose::new(Vec3::new(-0.0, 1.0, 5.0).expect("tip"), Vec3::Z).expect("pose");
        let cut = Pose::new(Vec3::new(52.0, 1.0, 5.0).expect("tip"), Vec3::Z).expect("pose");
        let params = ProcessParams::new().speed(Some(12_000.0)).expect("speed").power(Some(0.8)).expect("power");
        let blocks = vec![
            ClBlock::Event(Event::ToolChange { tool_id: 1 }),
            ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }),
            ClBlock::Motion(ClMotion::Linear { to: cut, feed: 100.0, role: MoveRole::Cut, power: None }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
        ];
        let path = ClPath::try_new(Some("outer".into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(Some("plate".into()), vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let options = PostOptions { arc_tolerance: 0.05 };
        for id in ["linuxcnc", "linuxcnc_router", "grbl", "mach3", "mach4", "fanuc", "haas", "plasma", "grbl_laser"] {
            let post = builtin(id).expect(id);
            let text = post_program(&program, &post, &options).unwrap_or_else(|err| panic!("{id}: {err}"));
            assert!(!text.contains("-0"), "{id}: {text}");
            let report = audit(&text, &post).unwrap_or_else(|err| panic!("{id}: {err}\n{text}"));
            assert!(report.motions.len() >= 2, "{id}");
            assert!(report.motions.iter().all(|state| state.tool == Some(1)), "{id}");
            assert!(report.motions.iter().all(|state| state.units == super::UnitMode::Millimetre), "{id}");
            assert!(report.motions.iter().all(|state| state.distance == super::DistanceMode::Absolute), "{id}");
            assert!(report.motions.iter().all(|state| state.work.is_none()), "{id}");
            assert!(!report.spindle.on, "{id}");
        }
    }

    #[test]
    fn linuxcnc_router_probe_and_auxiliaries_replay() {
        let post = builtin("linuxcnc_router").expect("router");
        let toward = Pose::new(Vec3::new(1.0, 2.0, -10.0).expect("tip"), Vec3::Z).expect("pose");
        let approach = Pose::new(Vec3::new(0.0, 0.0, 5.0).expect("tip"), Vec3::Z).expect("pose");
        let cut = Pose::new(Vec3::new(10.0, 0.0, 5.0).expect("tip"), Vec3::Z).expect("pose");
        let spindle = ProcessParams::new().speed(Some(12_000.0)).expect("speed");
        let blocks = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Other("dust".into()), params: ProcessParams::new() }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Other("vacuum".into()), params: ProcessParams::new() }),
            ClBlock::Event(Event::Probe { toward: Some(toward), feed: Some(100.0) }),
            ClBlock::Motion(ClMotion::Rapid { to: approach, feed: None }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: spindle }),
            ClBlock::Motion(ClMotion::Linear { to: cut, feed: 100.0, role: MoveRole::Cut, power: None }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Other("vacuum".into()) }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Other("dust".into()) }),
        ];
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(Some("outer".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(Some("plate".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let text = crate::post_program(&program, &post, &crate::PostOptions { arc_tolerance: 0.05 }).expect("post");
        let at = |token: &str| {
            text.lines().position(|line| line.split_whitespace().any(|word| word == token)).unwrap_or_else(|| panic!("{token} missing\n{text}"))
        };
        let dust_on = at("P1");
        let vacuum_on = text.lines().position(|line| line.split_whitespace().any(|word| word == "M64") && line.contains("P0")).expect("vacuum on");
        let probe = at("G38.2");
        let rapid = at("G0");
        let spindle_on = at("M3");
        let cut_at = at("G1");
        let spindle_off = at("M5");
        let vacuum_off = text.lines().position(|line| line.split_whitespace().any(|word| word == "M65") && line.contains("P0")).expect("vacuum off");
        let dust_off = text.lines().position(|line| line.split_whitespace().any(|word| word == "M65") && line.contains("P1")).expect("dust off");
        assert!(dust_on < vacuum_on && vacuum_on < probe && probe < rapid && rapid < spindle_on && spindle_on < cut_at, "{text}");
        assert!(cut_at < spindle_off && spindle_off < vacuum_off && vacuum_off < dust_off, "{text}");
        let probe_line = text.lines().nth(probe).expect("probe line");
        assert!(
            probe_line.split_whitespace().any(|word| word == "X1.0000")
                && probe_line.split_whitespace().any(|word| word == "Y2.0000")
                && probe_line.split_whitespace().any(|word| word == "Z-10.0000")
                && probe_line.split_whitespace().any(|word| word == "F100.0000"),
            "{probe_line}"
        );
        let cut_line = text.lines().nth(cut_at).expect("cut line");
        assert!(cut_line.split_whitespace().all(|word| !word.starts_with('F')), "{cut_line}");
        let report = audit(&text, &post).unwrap_or_else(|err| panic!("{err}\n{text}"));
        assert_eq!(report.motions.len(), 2, "{text}");
        assert_eq!(report.motions[0].mode, super::MotionMode::Rapid);
        assert!(!report.motions[0].spindle.on);
        assert_eq!(report.motions[1].mode, super::MotionMode::Linear);
        assert!(report.motions[1].spindle.on);
        assert!((report.motions[1].feed.expect("feed") - 100.0).abs() < 1e-6);
        assert!(!report.spindle.on);
        let moves = crate::read_back(&text, &post).expect("read");
        crate::compare(&program, &moves, 1e-3, &post).expect("compare");
    }

    #[test]
    fn audit_rejects_s_above_zero_when_power_was_not_set() {
        let post = builtin("grbl_laser").expect("grbl_laser");
        let ok = "G21\nG90\nG17\nM4 S0\n(power was not set, using S0)\nG0 X1 Y1\nM4 S0\nG1 X2 F100\nM5\nM30\n";
        audit(ok, &post).unwrap_or_else(|err| panic!("{err}"));
        let bad = "G21\nG90\nG17\nM4 S0\n(power was not set, using S0)\nG0 X1 Y1\nM4 S1000\nG1 X2 F100\nM5\nM30\n";
        let err = audit(bad, &post).expect_err("s1000");
        assert!(err.to_string().contains("S1000 is above 0, but power was not set"), "{err}");
        let legacy = "G21\nG90\nG17\nM4 S0\n(power missing, using S1000)\nG0 X1 Y1\nM4 S1000\nG1 X2 F100\nM5\nM30\n";
        let err = audit(legacy, &post).expect_err("legacy");
        assert!(err.to_string().contains("S1000 is above 0, but power was not set"), "{err}");
        let explicit = "G21\nG90\nG17\nM4 S0\nG0 X1 Y1\nM4 S200\nG1 X2 F100\nM5\nM30\n";
        audit(explicit, &post).unwrap_or_else(|err| panic!("explicit power must still audit: {err}"));
    }
}
