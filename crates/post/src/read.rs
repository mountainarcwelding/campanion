//! Read the G-code dialect a post just wrote.
//!
//! The parser is modal and absolute. `I` `J` `K` are centre offsets from the start of the block.
//! A signed `R` is the radius. Comments and line numbers are skipped. `G4` is a dwell.

use campanion_ir::{ArcSense, Plane};

use crate::def::PostDef;
use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};

/// One block that carries motion, a dwell, or some other word.
#[derive(Clone, Debug)]
pub enum ReadMove {
    /// `G0`.
    Rapid {
        /// Absolute end position.
        end: Xyz,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
    },
    /// `G1`.
    Linear {
        /// Absolute end position.
        end: Xyz,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
    },
    /// `G2` or `G3`.
    Arc {
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Absolute end position.
        end: Xyz,
        /// Absolute centre. With an `R` word the perpendicular axis is the start position.
        center: Xyz,
        /// Signed radius when the block used `R`.
        radius: Option<f64>,
        /// Feed word on this block, or the modal feed when the block omits `F`.
        feed: Option<f64>,
    },
    /// `G4`, in seconds.
    Dwell {
        /// Seconds.
        seconds: f64,
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
}

impl<'a> Parser<'a> {
    fn new(post: &'a PostDef) -> Self {
        Self { post, pos: [0.0, 0.0, 0.0], motion: None, plane: Plane::Xy, feed: None }
    }

    fn line(&mut self, raw: &str) -> Result<Option<ReadMove>> {
        let stripped = strip_comments(raw);
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
                'I' | 'J' | 'K' | 'R' | 'F' | 'S' | 'H' | 'D' | 'P' => {}
                letter if axes.contains(&letter) || letter == dwell_letter => {}
                letter => return Err(invalid(format!("unexpected word {letter}"))),
            }
        }
        let f = unique(&words, 'F')?;
        let mut plane = self.plane;
        let mut motion = self.motion;
        let mut dwell = false;
        let mut planes = 0u8;
        let mut motions = 0u8;
        for code in g_codes {
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
                21 | 40 | 43 | 49 | 54 | 55 | 56 | 57 | 58 | 59 | 61 | 64 | 80 | 90 | 94 => {}
                other => return Err(invalid(format!("unsupported G{other}"))),
            }
        }
        if motions > 1 || planes > 1 {
            return Err(invalid("line has conflicting G codes"));
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
        if let Some(feed) = f {
            self.feed = Some(feed);
        }
        let feed = self.feed;
        let move_ = match motion {
            Motion::Rapid => ReadMove::Rapid { end, feed },
            Motion::Linear => ReadMove::Linear { end, feed },
            Motion::Cw | Motion::Ccw => {
                let sense = if matches!(motion, Motion::Cw) { ArcSense::Cw } else { ArcSense::Ccw };
                let radius = unique(words, 'R')?;
                let (i, j, k) = (unique(words, 'I')?, unique(words, 'J')?, unique(words, 'K')?);
                if radius.is_some() && (i.is_some() || j.is_some() || k.is_some()) {
                    return Err(invalid("arc has both R and IJK"));
                }
                let center = if let Some(radius) = radius {
                    geom::center_from_radius(start, end, radius, sense, plane)?
                } else if i.is_some() || j.is_some() || k.is_some() {
                    [start[0] + i.unwrap_or(0.0), start[1] + j.unwrap_or(0.0), start[2] + k.unwrap_or(0.0)]
                } else {
                    return Err(invalid("arc needs IJK or R"));
                };
                ReadMove::Arc { sense, end, center, radius, feed }
            }
        };
        self.pos = end;
        self.motion = Some(motion);
        self.plane = plane;
        Ok(Some(move_))
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
        let letter = bytes[index];
        if !letter.is_ascii_alphabetic() {
            return Err(invalid("unexpected character in g-code"));
        }
        index += 1;
        let start = index;
        if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
            index += 1;
        }
        let mut saw_digit = false;
        let mut saw_dot = false;
        while index < bytes.len() {
            match bytes[index] {
                b'0'..=b'9' => {
                    saw_digit = true;
                    index += 1;
                }
                b'.' if !saw_dot => {
                    saw_dot = true;
                    index += 1;
                }
                _ => break,
            }
        }
        if !saw_digit {
            return Err(invalid("g-code word is missing a number"));
        }
        let text = &line[start..index];
        let value: f64 = text.parse().map_err(|_| invalid(format!("invalid number {text}")))?;
        if !value.is_finite() {
            return Err(invalid("g-code number must be finite"));
        }
        words.push(((letter as char).to_ascii_uppercase(), value));
    }
    Ok(words)
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
