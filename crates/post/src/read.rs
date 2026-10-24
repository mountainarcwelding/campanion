//! Read the G-code dialect a post just wrote.
//!
//! The parser is modal and absolute. `I` `J` `K` are centre offsets from the start of the block.
//! A signed `R` is the radius. Comments, line numbers, `%` tape edges and `O` program numbers are
//! skipped. `G4` is a dwell. A post with a rotary block also accepts its A/B word and the
//! inverse-time codes (`G93` / `G94` on LinuxCNC). Those stay modal the way `X` `Y` `Z` do.
//!
//! A lathe post returns part-frame positions. In diameter mode each X word is divided by two.
//! Y is 0, except while face or cylinder interpolation (from the post's `face_interp` /
//! `cylinder_interp` templates) is on, or when the post has `y_axis`, in which case Y is modal
//! like X and Z. In face mode X is a radius unless `x_diameter` is set, and Y is kept. In
//! cylinder mode X still follows diameter mode and Y is the unrolled distance.
//! `I` `J` `K` are centre offsets from the stored start. `arc_flip` swaps G2 and G3 back to the
//! IR sense on ZX arcs only. `G53` is a machine move and is not returned. `G28` is a reference
//! return and is not returned. `U` and `W` are accepted only on that block. `G50` (spindle cap)
//! and `G98` (Fanuc and Haas feed per minute) are not moves. A cylinder `on` line such as
//! `G107 Y0 C0 R20.` is not a move and does not change the modal C word.
//!
//! A lathe post with `c_axis` treats that letter as a modal angle. Rapid and linear moves return
//! it on `rotary`. `M` lines, `P` and `H` words, and `G28 H0.` are not moves. `G28 H0.` does not
//! change the modal C word: the axis letter is `C`, not `H`. `$n` selects a spindle (LinuxCNC
//! live tool `$1`) and is not a move. When `lathe.c_feed` is `inverse_time`, `G93` turns inverse
//! time on and `lathe.feed_mode` (`G94` when that word is empty) turns it off.

use campanion_ir::{ArcSense, Plane, RotaryAxisKind};

use crate::def::{PostDef, g_code_number};
use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};

/// One block that carries motion, a dwell, or some other word.
#[derive(Clone, Debug)]
pub enum ReadMove {
    /// `G0`.
    Rapid {
        /// Absolute end position, in the frame the post wrote (machine frame when `rotary` is set).
        end: Xyz,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
        /// Modal A/B or lathe C angle in degrees after this block.
        rotary: Option<f64>,
        /// Modal inverse-time state (`G93` on LinuxCNC) after this block.
        inverse_time: bool,
    },
    /// `G1`.
    Linear {
        /// Absolute end position, in the frame the post wrote (machine frame when `rotary` is set).
        end: Xyz,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
        /// Modal A/B or lathe C angle in degrees after this block.
        rotary: Option<f64>,
        /// Modal inverse-time state (`G93` on LinuxCNC) after this block.
        inverse_time: bool,
    },
    /// `G2` or `G3`.
    Arc {
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Absolute end position, in the frame the post wrote (machine frame when `rotary` is set).
        end: Xyz,
        /// Absolute centre. With an `R` word the perpendicular axis is the start position.
        center: Xyz,
        /// Signed radius when the block used `R`.
        radius: Option<f64>,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
        /// Modal A/B or lathe C angle in degrees after this block.
        rotary: Option<f64>,
        /// Modal inverse-time state (`G93` on LinuxCNC) after this block.
        inverse_time: bool,
    },
    /// `G4`, in seconds.
    Dwell {
        /// Seconds.
        seconds: f64,
    },
    /// Spindle-synchronised straight feed (`G33`/`K` or `G32`/`F` on a lathe post).
    Thread {
        /// Absolute end position, in the part frame. Diameter-mode X has already been halved.
        end: Xyz,
        /// Lead, millimetres per spindle revolution.
        lead: f64,
        /// Modal A/B or lathe C angle in degrees after this block.
        rotary: Option<f64>,
        /// Modal inverse-time state after this block.
        inverse_time: bool,
    },
    /// An `M` or `T` block that does not move.
    Other {
        /// The block with comments and the line number removed.
        text: String,
    },
}

#[derive(Clone, Copy)]
enum Motion {
    Rapid,
    Linear,
    Cw,
    Ccw,
}

struct Parser<'a> {
    post: &'a PostDef,
    pos: Xyz,
    motion: Option<Motion>,
    plane: Plane,
    feed: Option<f64>,
    /// Modal rotary angle. Updated when the block carries the post's A or B word.
    angle: Option<f64>,
    /// Modal inverse time. `G94` (or the post's off code) clears it.
    inverse_time: bool,
    /// Cartesian face frame. X scaling follows `face_interp.x_diameter` and Y is kept.
    face_mode: bool,
    /// Unrolled cylinder frame. Y is kept and X follows lathe diameter mode.
    cyl_mode: bool,
    /// True after a `sub_spindle.select` line, until a `deselect` line.
    on_sub: bool,
}

impl<'a> Parser<'a> {
    fn new(post: &'a PostDef) -> Self {
        Self {
            post,
            pos: [0.0, 0.0, 0.0],
            motion: None,
            plane: Plane::Xy,
            feed: None,
            angle: None,
            inverse_time: false,
            face_mode: false,
            cyl_mode: false,
            on_sub: false,
        }
    }

    fn line(&mut self, raw: &str) -> Result<Option<ReadMove>> {
        let stripped = strip_comments(raw);
        if stripped.trim() == "%" {
            return Ok(None);
        }
        let mut words = tokenize(&stripped)?;
        if self.post.line_numbers.enabled {
            strip_line_number(&mut words, self.post.line_numbers.prefix.chars().next().unwrap_or('N'))?;
        }
        if words.is_empty() {
            return Ok(None);
        }
        self.interpret(words, stripped.trim())
    }
}

/// Parse `text` with the same post that produced it.
pub fn read_back(text: &str, post: &PostDef) -> Result<Vec<ReadMove>> {
    if text.len() > 64 * 1024 * 1024 {
        return Err(invalid("g-code is too large"));
    }
    let mut parser = Parser::new(post);
    let mut moves = Vec::new();
    for (index, line) in text.lines().enumerate() {
        if index > 2_000_000 {
            return Err(invalid("g-code has too many lines"));
        }
        if line.len() > 8192 {
            return Err(invalid(format!("line {}: line is too long", index + 1)));
        }
        if let Some(move_) = parser.line(line).map_err(|err| annotate(index + 1, err))? {
            moves.push(move_);
        }
    }
    Ok(moves)
}

impl<'a> Parser<'a> {
    fn interpret(&mut self, words: Vec<(char, f64)>, raw: &str) -> Result<Option<ReadMove>> {
        self.note_sub_line(raw);
        if let Some((letter, value)) = words.first().copied()
            && letter == 'O'
            && words.len() == 1
        {
            if !(value.is_finite() && value >= 0.0 && (value - value.round()).abs() <= 1e-6) {
                return Err(invalid("program number must be a whole number"));
            }
            return Ok(None);
        }
        let axes = axis_letters(self.post);
        let dwell_letter = self.post.dwell.word.chars().next().unwrap_or('P');
        let mut g_codes = Vec::new();
        let mut saw_m = false;
        let mut saw_t = false;
        for (letter, value) in &words {
            match *letter {
                'G' => g_codes.push(g_number(*value)?),
                'M' => saw_m = true,
                'T' => saw_t = true,
                'I' | 'J' | 'K' | 'R' | 'Q' | 'F' | 'S' | 'H' | 'D' | 'P' => {}
                'U' | 'W' if self.post.is_lathe() => {}
                letter
                    if axes.contains(&letter)
                        || letter == dwell_letter
                        || self.is_rotary_letter(letter)
                        || self.is_c_letter(letter)
                        || self.is_y_letter(letter)
                        || self.is_transfer_letter(letter) => {}
                letter => return Err(invalid(format!("unexpected word {letter}"))),
            }
        }
        let f = unique(&words, 'F')?;
        // Mode lines are recognised before C is applied, so `G107 Y0 C0 R…` does not move and
        // does not change the modal C word.
        if self.take_interpolation(&words, &g_codes)? {
            self.apply_plane_codes(&g_codes);
            return Ok(None);
        }
        if let Some(angle) = self.rotary_word(&words)? {
            self.angle = Some(angle);
        }
        let mut plane = self.plane;
        let mut motion = self.motion;
        let mut dwell = false;
        let mut threading = false;
        let mut planes = 0u8;
        let mut motions = 0u8;
        let mut inverse_time = self.inverse_time;
        let thread_code = self.thread_code();
        for &code in &g_codes {
            if self.post.rotary.as_ref().is_some_and(|rotary| rotary.on_code() == Some(code)) {
                inverse_time = true;
                continue;
            }
            if self.post.rotary.as_ref().is_some_and(|rotary| rotary.off_code() == Some(code)) {
                inverse_time = false;
                continue;
            }
            if self.is_lathe_inverse_on(code) {
                inverse_time = true;
                continue;
            }
            if self.is_lathe_inverse_off(code) {
                inverse_time = false;
                continue;
            }
            match code {
                0 => {
                    motion = Some(Motion::Rapid);
                    motions += 1;
                }
                1 => {
                    motion = Some(Motion::Linear);
                    motions += 1;
                }
                2 => {
                    motion = Some(Motion::Cw);
                    motions += 1;
                }
                3 => {
                    motion = Some(Motion::Ccw);
                    motions += 1;
                }
                4 => dwell = true,
                17 => {
                    plane = Plane::Xy;
                    planes += 1;
                }
                18 => {
                    plane = Plane::Zx;
                    planes += 1;
                }
                19 => {
                    plane = Plane::Yz;
                    planes += 1;
                }
                20 => return Err(invalid("inches are not supported")),
                91 => return Err(invalid("incremental motion is not supported")),
                7 | 8 | 50 | 96 | 97 | 98 if self.post.is_lathe() => {}
                28 | 53 if self.post.is_lathe() => {}
                21 | 40 | 43 | 49 | 54 | 55 | 56 | 57 | 58 | 59 | 61 | 64 | 80 | 90 | 94 => {}
                code if self.is_sub_spindle_g(code) => {}
                code if self.is_transfer_g(code) => {}
                code if thread_code == Some(code) => {
                    threading = true;
                    motions += 1;
                }
                other => return Err(invalid(format!("unsupported G{other}"))),
            }
        }
        if motions > 1 || planes > 1 {
            return Err(invalid("line has conflicting G codes"));
        }
        // U and W are incremental lathe axes. Fanuc home is `G28 U0 W0`. Any other use would
        // move in a frame this parser does not track.
        if words.iter().any(|(letter, _)| *letter == 'U' || *letter == 'W') && !g_codes.contains(&28) {
            return Err(invalid("incremental U or W is only accepted on lathe G28"));
        }
        self.inverse_time = inverse_time;
        // G53 is a non-modal machine-coordinate move. G28 is a reference return (Fanuc writes
        // U0 W0). Neither block is a part-frame point.
        if self.post.is_lathe() && (g_codes.contains(&28) || g_codes.contains(&53)) {
            return Ok(None);
        }
        let axis_values = [unique(&words, axes[0])?, unique(&words, axes[1])?, unique(&words, axes[2])?];
        let has_axis = axis_values.iter().any(Option::is_some);
        if dwell {
            if has_axis || motions > 0 {
                return Err(invalid("dwell cannot share a block with motion"));
            }
            let word = unique(&words, dwell_letter)?.ok_or_else(|| invalid("dwell is missing its value"))?;
            let seconds = if self.post.dwell.unit == "ms" { word / 1000.0 } else { word };
            if !(seconds.is_finite() && seconds >= 0.0) {
                return Err(invalid("dwell must be >= 0"));
            }
            self.plane = plane;
            return Ok(Some(ReadMove::Dwell { seconds }));
        }
        if threading {
            if !has_axis {
                return Err(invalid("thread move needs an end position"));
            }
            return self.thread_block(plane, axis_values, &words);
        }
        if has_axis {
            return self.motion_block(motion, plane, axis_values, &words, f);
        }
        if let Some(feed) = f {
            self.feed = Some(feed);
        }
        self.motion = motion;
        self.plane = plane;
        if saw_m || saw_t {
            return Ok(Some(ReadMove::Other { text: raw.to_string() }));
        }
        Ok(None)
    }

    fn motion_block(
        &mut self,
        motion: Option<Motion>,
        plane: Plane,
        axes: [Option<f64>; 3],
        words: &[(char, f64)],
        f: Option<f64>,
    ) -> Result<Option<ReadMove>> {
        let Some(motion) = motion else {
            return Err(invalid("motion needs a G code"));
        };
        let start = self.pos;
        let mut end = self.pos;
        for (index, value) in axes.iter().enumerate() {
            if let Some(value) = value {
                end[index] = *value;
            }
        }
        if self.post.is_lathe() {
            end = self.lathe_end(end, axes, words)?;
        }
        if let Some(feed) = f {
            self.feed = Some(feed);
        }
        let feed = self.feed;
        let rotary = self.angle;
        let inverse_time = self.inverse_time;
        let move_ = match motion {
            Motion::Rapid => ReadMove::Rapid { end, feed, rotary, inverse_time },
            Motion::Linear => ReadMove::Linear { end, feed, rotary, inverse_time },
            Motion::Cw | Motion::Ccw => {
                let mut sense = if matches!(motion, Motion::Cw) { ArcSense::Cw } else { ArcSense::Ccw };
                if plane == Plane::Zx && self.post.lathe.as_ref().is_some_and(|lathe| lathe.arc_flip) {
                    sense = flip_sense(sense);
                }
                if self.mirroring() && matches!(plane, Plane::Zx | Plane::Yz) {
                    sense = flip_sense(sense);
                }
                let radius = unique(words, 'R')?;
                let (i, j, mut k) = (unique(words, 'I')?, unique(words, 'J')?, unique(words, 'K')?);
                if self.mirroring() {
                    k = k.map(|value| -value);
                }
                if radius.is_some() && (i.is_some() || j.is_some() || k.is_some()) {
                    return Err(invalid("arc has both R and IJK"));
                }
                let center = if let Some(radius) = radius {
                    let quantum = 10_f64.powi(-i32::from(self.post.decimals.linear));
                    geom::center_from_radius(start, end, radius, sense, plane, quantum)?
                } else if i.is_some() || j.is_some() || k.is_some() {
                    [start[0] + i.unwrap_or(0.0), start[1] + j.unwrap_or(0.0), start[2] + k.unwrap_or(0.0)]
                } else {
                    return Err(invalid("arc needs IJK or R"));
                };
                ReadMove::Arc { sense, end, center, radius, feed, rotary, inverse_time }
            }
        };
        self.pos = end;
        self.motion = Some(motion);
        self.plane = plane;
        Ok(Some(move_))
    }

    /// G33/G32. The lead word is not a modal feed, and the motion code is not left modal.
    fn thread_block(&mut self, plane: Plane, axes: [Option<f64>; 3], words: &[(char, f64)]) -> Result<Option<ReadMove>> {
        let letter = self.thread_lead_letter().ok_or_else(|| invalid("thread lead word is missing"))?;
        let mut end = self.pos;
        for (index, value) in axes.iter().enumerate() {
            if let Some(value) = value {
                end[index] = *value;
            }
        }
        if self.post.is_lathe() {
            end = self.lathe_end(end, axes, words)?;
        }
        let lead = unique(words, letter)?.ok_or_else(|| invalid("thread lead is missing"))?;
        if !(lead.is_finite() && lead > 0.0) {
            return Err(invalid("thread lead must be positive"));
        }
        let move_ = ReadMove::Thread { end, lead, rotary: self.angle, inverse_time: self.inverse_time };
        self.pos = end;
        self.motion = None;
        self.plane = plane;
        Ok(Some(move_))
    }

    fn thread_code(&self) -> Option<u16> {
        let thread = self.post.lathe.as_ref()?.thread.as_ref()?;
        g_code_number(&thread.code)
    }

    fn thread_lead_letter(&self) -> Option<char> {
        self.post.lathe.as_ref()?.thread.as_ref()?.lead_word.chars().next()
    }

    fn stored_x(&self, x: f64) -> f64 {
        if self.face_mode {
            let diameter = self.post.lathe.as_ref().and_then(|lathe| lathe.face_interp.as_ref()).is_some_and(|face| face.x_diameter);
            if diameter { x * 0.5 } else { x }
        } else if self.post.lathe.as_ref().is_some_and(|lathe| lathe.diameter_mode) {
            x * 0.5
        } else {
            x
        }
    }

    fn apply_plane_codes(&mut self, g_codes: &[u16]) {
        for code in g_codes {
            match code {
                17 => self.plane = Plane::Xy,
                18 => self.plane = Plane::Zx,
                19 => self.plane = Plane::Yz,
                _ => {}
            }
        }
    }

    /// True when this line is only an interpolation mode change, and the mode was applied.
    ///
    /// A line that also carries G0/G1/G2/G3 (or the thread code) is motion and is left alone.
    /// `G19` in a cylinder `on` template is a plane select unless the mapping word is on that
    /// same line. `G107` with `{radius}` or `{diameter}` enters; a bare `G107` cancels.
    /// `G17` in a face `on` template is the plane Y-axis milling also uses. It does not enter
    /// face mode when the template names another G code, so X stays a diameter until `G112`.
    fn take_interpolation(&mut self, words: &[(char, f64)], g_codes: &[u16]) -> Result<bool> {
        if g_codes.iter().any(|code| is_motion_g(*code, self.thread_code())) {
            return Ok(false);
        }
        let Some(lathe) = self.post.lathe.as_ref() else {
            return Ok(false);
        };
        let mut consumed = false;
        if let Some(face) = &lathe.face_interp {
            let on = face_on_hit(g_codes, &face.on);
            let off = g_hits(g_codes, &face.off);
            if on && !off {
                self.face_mode = true;
                self.cyl_mode = false;
                consumed = true;
            } else if off {
                self.face_mode = false;
                consumed = true;
            }
        }
        if let Some(cylinder) = &lathe.cylinder_interp {
            let on = g_hits(g_codes, &cylinder.on);
            let off = g_hits(g_codes, &cylinder.off);
            if on || off {
                let enter = on && mapping_present(&cylinder.on, words)?;
                if enter {
                    self.cyl_mode = true;
                    self.face_mode = false;
                    consumed = true;
                } else if off {
                    self.cyl_mode = false;
                    consumed = true;
                }
            }
        }
        Ok(consumed)
    }

    fn is_rotary_letter(&self, letter: char) -> bool {
        let Some(rotary) = &self.post.rotary else {
            return false;
        };
        (letter == 'A' && rotary.supports(RotaryAxisKind::A)) || (letter == 'B' && rotary.supports(RotaryAxisKind::B))
    }

    fn c_axis_letter(&self) -> Option<char> {
        self.post.lathe.as_ref()?.c_axis.as_ref()?.letter.chars().next()
    }

    fn is_c_letter(&self, letter: char) -> bool {
        self.c_axis_letter() == Some(letter)
    }

    fn y_axis_letter(&self) -> Option<char> {
        self.post.lathe.as_ref()?.y_axis.as_ref()?.letter.chars().next()
    }

    fn is_y_letter(&self, letter: char) -> bool {
        self.y_axis_letter() == Some(letter)
    }

    fn is_lathe_inverse_on(&self, code: u16) -> bool {
        self.post.lathe.as_ref().is_some_and(|lathe| lathe.c_feed_inverse()) && code == 93
    }

    fn is_lathe_inverse_off(&self, code: u16) -> bool {
        self.post.lathe.as_ref().is_some_and(|lathe| lathe.c_feed_inverse() && lathe.c_feed_off_code() == Some(code))
    }

    /// Diameter mode halves X. Without `y_axis`, Y is centreline 0. With it, Y is modal, and a
    /// letter that is not the second axis letter is read into Y.
    fn lathe_end(&self, mut end: Xyz, axes_present: [Option<f64>; 3], words: &[(char, f64)]) -> Result<Xyz> {
        if axes_present[0].is_some() {
            end[0] = self.stored_x(end[0]);
        }
        if self.face_mode || self.cyl_mode {
            return Ok(end);
        }
        let Some(letter) = self.y_axis_letter() else {
            end[1] = 0.0;
            if self.mirroring() && axes_present[2].is_some() {
                end[2] = -end[2];
            }
            return Ok(end);
        };
        let second = axis_letters(self.post)[1];
        if letter != second
            && let Some(value) = unique(words, letter)?
        {
            end[1] = value;
        }
        if self.mirroring() && axes_present[2].is_some() {
            end[2] = -end[2];
        }
        Ok(end)
    }

    /// A select line enters the sub frame. A deselect line leaves it. Moves in between are un-mirrored
    /// when `mirror_z` is set, so read-back returns the part frame the writer started from.
    fn note_sub_line(&mut self, raw: &str) {
        let Some(sub) = self.post.lathe.as_ref().and_then(|lathe| lathe.sub_spindle.as_ref()) else {
            return;
        };
        let body = block_body(self.post, raw);
        if sub.select.iter().any(|line| line.trim() == body) {
            self.on_sub = true;
        } else if sub.deselect.iter().any(|line| line.trim() == body) {
            self.on_sub = false;
        }
    }

    fn mirroring(&self) -> bool {
        self.on_sub && self.post.lathe.as_ref().and_then(|lathe| lathe.sub_spindle.as_ref()).is_some_and(|sub| sub.mirror_z)
    }

    fn is_sub_spindle_g(&self, code: u16) -> bool {
        let Some(sub) = self.post.lathe.as_ref().and_then(|lathe| lathe.sub_spindle.as_ref()) else {
            return false;
        };
        template_g_codes(&sub.select).contains(&code) || template_g_codes(&sub.deselect).contains(&code)
    }

    fn is_transfer_letter(&self, letter: char) -> bool {
        transfer_axis_letter(self.post) == Some(letter)
    }

    fn is_transfer_g(&self, code: u16) -> bool {
        let Some(transfer) = self.post.lathe.as_ref().and_then(|lathe| lathe.transfer.as_ref()) else {
            return false;
        };
        let groups = [
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
        ];
        groups.iter().any(|lines| template_g_codes(lines).contains(&code))
    }

    fn rotary_word(&self, words: &[(char, f64)]) -> Result<Option<f64>> {
        let mill = self.mill_rotary_word(words)?;
        let c = match self.c_axis_letter() {
            Some(letter) => unique(words, letter)?,
            None => None,
        };
        match (mill, c) {
            (Some(_), Some(_)) => Err(invalid("block has both a rotary word and a C word")),
            (Some(value), None) | (None, Some(value)) => Ok(Some(value)),
            (None, None) => Ok(None),
        }
    }

    fn mill_rotary_word(&self, words: &[(char, f64)]) -> Result<Option<f64>> {
        let Some(rotary) = &self.post.rotary else {
            return Ok(None);
        };
        let a = if rotary.supports(RotaryAxisKind::A) { unique(words, 'A')? } else { None };
        let b = if rotary.supports(RotaryAxisKind::B) { unique(words, 'B')? } else { None };
        match (a, b) {
            (Some(_), Some(_)) => Err(invalid("block has both an A word and a B word")),
            (Some(value), None) | (None, Some(value)) => Ok(Some(value)),
            (None, None) => Ok(None),
        }
    }
}

fn axis_letters(post: &PostDef) -> [char; 3] {
    let mut chars = post.axis_letters.chars();
    [chars.next().unwrap_or('X'), chars.next().unwrap_or('Y'), chars.next().unwrap_or('Z')]
}

fn strip_line_number(words: &mut Vec<(char, f64)>, prefix: char) -> Result<()> {
    let Some((letter, value)) = words.first().copied() else {
        return Ok(());
    };
    if letter != prefix {
        return Ok(());
    }
    if !(value.is_finite() && value >= 0.0 && (value - value.round()).abs() <= 1e-6) {
        return Err(invalid("line number must be a whole number"));
    }
    words.remove(0);
    Ok(())
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

fn tokenize(line: &str) -> Result<Vec<(char, f64)>> {
    let bytes = line.as_bytes();
    let mut index = 0;
    let mut words = Vec::new();
    while index < bytes.len() {
        if bytes[index].is_ascii_whitespace() {
            index += 1;
            continue;
        }
        // LinuxCNC `$n` selects spindle n. It is not an axis and not a move.
        if bytes[index] == b'$' {
            index += 1;
            let _value = take_number(line, bytes, &mut index)?;
            continue;
        }
        let letter = bytes[index];
        if !letter.is_ascii_alphabetic() {
            return Err(invalid("unexpected character in g-code"));
        }
        index += 1;
        let value = take_number(line, bytes, &mut index)?;
        words.push(((letter as char).to_ascii_uppercase(), value));
    }
    Ok(words)
}

fn take_number(line: &str, bytes: &[u8], index: &mut usize) -> Result<f64> {
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
        return Err(invalid("g-code word is missing a number"));
    }
    let text = &line[start..*index];
    let value: f64 = text.parse().map_err(|_| invalid(format!("invalid number {text}")))?;
    if !value.is_finite() {
        return Err(invalid("g-code number must be finite"));
    }
    Ok(value)
}

fn unique(words: &[(char, f64)], letter: char) -> Result<Option<f64>> {
    let mut found = None;
    for (name, value) in words {
        if *name == letter {
            if found.is_some() {
                return Err(invalid(format!("duplicate {letter} word")));
            }
            found = Some(*value);
        }
    }
    Ok(found)
}

fn flip_sense(sense: ArcSense) -> ArcSense {
    match sense {
        ArcSense::Cw => ArcSense::Ccw,
        ArcSense::Ccw => ArcSense::Cw,
    }
}

/// Drop a leading sequence number so `N40 G55` matches the template `G55`.
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
    if after.len() == rest.len() {
        return raw.to_string();
    }
    after.trim().to_string()
}

fn is_motion_g(code: u16, thread: Option<u16>) -> bool {
    matches!(code, 0..=3) || thread == Some(code)
}

/// Axis letter written immediately before `{z}` on a transfer move, for example `B` in `G00 B{z}`.
fn transfer_axis_letter(post: &PostDef) -> Option<char> {
    let transfer = post.lathe.as_ref()?.transfer.as_ref()?;
    for lines in [&transfer.move_rapid, &transfer.move_feed, &transfer.move_machine] {
        for line in lines {
            let Some(index) = line.find("{z}") else {
                continue;
            };
            let Some(letter) = line[..index].chars().next_back() else {
                continue;
            };
            if letter.is_ascii_alphabetic() {
                return Some(letter.to_ascii_uppercase());
            }
        }
    }
    None
}

/// G codes named in interpolation templates, such as `G112` or `G107`.
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

fn g_hits(g_codes: &[u16], lines: &[String]) -> bool {
    let needles = template_g_codes(lines);
    g_codes.iter().any(|code| needles.contains(code))
}

/// Face interpolation on. A plane word is not enough when the template also names a mode word.
///
/// Haas writes `G17` before `G112`, and the same `G17` before a real Y move. Only `G112` makes
/// X a radius. A template that is only a plane select still enters on that plane.
fn face_on_hit(g_codes: &[u16], lines: &[String]) -> bool {
    let needles = template_g_codes(lines);
    let shares_plane = needles.iter().any(|code| is_plane_select(*code));
    let has_mode = needles.iter().any(|code| !is_plane_select(*code));
    g_codes.iter().any(|code| {
        if shares_plane && has_mode && is_plane_select(*code) {
            return false;
        }
        needles.contains(code)
    })
}

fn is_plane_select(code: u16) -> bool {
    matches!(code, 17..=19)
}

/// Letter that carries `{radius}` or `{diameter}` in a cylinder `on` template.
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

/// True when the cylinder `on` line is present. No placeholder means the G code alone enters.
fn mapping_present(lines: &[String], words: &[(char, f64)]) -> Result<bool> {
    let Some(letter) = mapping_letter(lines) else {
        return Ok(true);
    };
    Ok(unique(words, letter)?.is_some())
}

fn g_number(value: f64) -> Result<u16> {
    if !(value.is_finite() && (0.0..=200.0).contains(&value)) {
        return Err(invalid("G code is out of range"));
    }
    let rounded = value.round();
    if (value - rounded).abs() > 1e-6 {
        return Err(invalid("G code is not an integer"));
    }
    Ok(rounded as u16)
}

fn annotate(line: usize, err: crate::PostError) -> crate::PostError {
    match err {
        crate::PostError::InvalidInput(message) => invalid(format!("line {line}: {message}")),
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::tokenize;

    #[test]
    fn trailing_decimal_point_stays_with_the_number() {
        let words = tokenize("X52. Z0. Y0.").expect("words");
        assert_eq!(words, vec![('X', 52.0), ('Z', 0.0), ('Y', 0.0)]);
        assert_eq!(tokenize("X0.").expect("zero"), vec![('X', 0.0)]);
        assert_eq!(tokenize("A90.").expect("angle"), vec![('A', 90.0)]);
        assert_eq!(tokenize("G1 X52.Y0.Z0.").expect("glued"), vec![('G', 1.0), ('X', 52.0), ('Y', 0.0), ('Z', 0.0)]);
        assert_eq!(tokenize("I0. K-5.").expect("offsets"), vec![('I', 0.0), ('K', -5.0)]);
    }
}
