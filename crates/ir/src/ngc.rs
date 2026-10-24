//! RS274/NGC text for tests.
//!
//! [`write_ngc`] is a test sink. It prints a [`Program`] so a test can see motion and events
//! as text. It is not a post for any real machine. It does not know a controller, a tool table,
//! kinematics or work offsets. Do not send the text to a machine.
//!
//! The dialect is fixed so expected strings stay exact:
//! - The text starts with `G21` (millimetres) and `G90` (absolute coordinates).
//! - Tool-space moves write the tool tip as `X` `Y` `Z`. The tool axis is not written.
//! - `G0` is a rapid, `G1` a line, `G2` a clockwise arc and `G3` a counter-clockwise arc.
//! - `G17`, `G18` and `G19` select the XY, ZX and YZ planes. The plane word is written only
//!   when it changes.
//! - `I`, `J` and `K` are the stored centre minus the position at the start of that block.
//!   They are offsets, not absolute centre coordinates, and they are not projected into the plane.
//! - A helix is one or more `G2` or `G3` blocks, including the axis perpendicular to the plane.
//!   A sweep longer than one turn is split into full turns. Each full turn returns to the
//!   in-plane start and the axial word advances in proportion to the angle. The last block ends
//!   on the stored pose.
//! - `F` is written only when its 4-decimal text differs from the previous `F` in that toolpath
//!   or channel program. A rapid with no feed does not change `F`.
//! - `G4 P` is a dwell. `P` is seconds.
//! - Process on is `M3` and process off is `M5`, each with a comment naming the process type.
//!   Speed, power and the delays stored on the event are not written. Timing already lives in
//!   the block list. A process that is still on at the end of the program is stopped with `M5`
//!   before the program-end line, in the order those processes were turned on.
//! - The program ends with `M30`. [`NgcOptions`] selects `M2` instead.
//! - A tool change is `T` then `M6`. The tool table, compensation, tabs and pierce point are
//!   not written.
//! - Comments use parentheses. Parentheses in the source text are removed. Line breaks in that
//!   text become spaces.
//! - Coolant, probe, custom, sync, C-axis, spindle-brake, live-tool, face-interpolation and
//!   cylinder-interpolation events are comments, so the block order stays visible. They are not
//!   machine codes.
//! - Every coordinate, offset, feed and dwell is written with four digits after the decimal point.
//!   A number that would print as `-0.0000`, including a negative zero and a value that rounds to
//!   zero at that precision, is written as `0.0000`.
//! - A program name is `(program …)`, a toolpath name is `(toolpath …)`, and a channel program
//!   is `(channel <id> <name>)`. Each of those sections starts with no position, no plane and no
//!   feed. An arc before that position exists is an error.
//! - Axis words use the machine's letters, in the order X Y Z A B C U V W. An indexed axis has
//!   no letter here.

use crate::check::invalid;
use crate::geom::{ArcSense, Plane, Vec3};
use crate::machine::{AxisLetter, AxisName, Machine};
use crate::motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath};
use crate::process::{CoolantMode, Event, LiveDirection, ProcessKind};
use crate::program::Program;
use crate::{AXIS_LETTER_COUNT, IrError, Result};

/// Full turns a helix may be split into. One input block must not expand without a bound.
const MAX_HELIX_TURNS: u32 = 10_000;

/// Letters in RS274 word order. The index is the slot in [`Writer::axes`].
const LETTERS: [&str; AXIS_LETTER_COUNT] = ["X", "Y", "Z", "A", "B", "C", "U", "V", "W"];

/// Program-end word. `M30` is the default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ProgramEnd {
    /// End the program with `M30`.
    #[default]
    M30,
    /// End the program with `M2`.
    M2,
}

impl ProgramEnd {
    fn word(self) -> &'static str {
        match self {
            ProgramEnd::M30 => "M30",
            ProgramEnd::M2 => "M2",
        }
    }
}

/// Options for [`write_ngc_with`]. [`write_ngc`] uses [`NgcOptions::default`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NgcOptions {
    /// Word written after any process that is still on has been stopped.
    pub program_end: ProgramEnd,
}

/// Write `program` as RS274/NGC text for tests.
///
/// This is a test sink, not a post for any real machine. The module docs define the dialect.
/// The string ends with a newline. The last line is `M30`. [`write_ngc_with`] can end with `M2`.
///
/// An arc with no prior position, a non-finite number, a channel program with no machine, or a
/// helix with too many turns returns [`IrError::InvalidInput`]. An indexed axis returns
/// [`IrError::NotSupported`].
pub fn write_ngc(program: &Program) -> Result<String> {
    write_ngc_with(program, NgcOptions::default())
}

/// Write `program` with [`NgcOptions`].
///
/// See [`write_ngc`]. A process that is still on is stopped with `M5` before the end word.
pub fn write_ngc_with(program: &Program, options: NgcOptions) -> Result<String> {
    let mut writer = Writer::new(options);
    writer.line("G21");
    writer.line("G90");
    if let Some(name) = program.name() {
        let label = strip_parens(name);
        if !label.is_empty() {
            writer.line(&format!("(program {label})"));
        }
    }
    for path in program.toolpaths() {
        writer.write_toolpath(path)?;
    }
    for channel in program.channel_programs() {
        writer.write_channel(channel, program.machine())?;
    }
    writer.finish();
    Ok(writer.out)
}

struct Writer {
    out: String,
    /// Latest value of X Y Z A B C U V W. `None` means that letter has not been set in this section.
    axes: [Option<f64>; AXIS_LETTER_COUNT],
    plane: Option<Plane>,
    /// Last feed word, without the `F`, using the same 4-decimal text that was written.
    feed: Option<String>,
    /// Processes turned on and not yet turned off, in the order they were turned on.
    processes_on: Vec<ProcessKind>,
    options: NgcOptions,
}

impl Writer {
    fn new(options: NgcOptions) -> Self {
        Self { out: String::new(), axes: [None; AXIS_LETTER_COUNT], plane: None, feed: None, processes_on: Vec::new(), options }
    }

    fn finish(&mut self) {
        for process in std::mem::take(&mut self.processes_on) {
            self.emit_process("M5", &process);
        }
        self.line(self.options.program_end.word());
    }

    fn reset_motion(&mut self) {
        self.axes = [None; AXIS_LETTER_COUNT];
        self.plane = None;
        self.feed = None;
    }

    fn line(&mut self, text: &str) {
        self.out.push_str(text);
        self.out.push('\n');
    }

    fn emit(&mut self, words: &[String]) {
        for (index, word) in words.iter().enumerate() {
            if index > 0 {
                self.out.push(' ');
            }
            self.out.push_str(word);
        }
        self.out.push('\n');
    }

    fn write_toolpath(&mut self, path: &ClPath) -> Result<()> {
        self.reset_motion();
        if let Some(name) = path.name() {
            let label = strip_parens(name);
            if label.is_empty() { self.line("(toolpath)") } else { self.line(&format!("(toolpath {label})")) }
        }
        for block in path.blocks() {
            match block {
                ClBlock::Motion(motion) => self.write_cl_motion(motion)?,
                ClBlock::Event(event) => self.write_event(event)?,
            }
        }
        Ok(())
    }

    fn write_channel(&mut self, program: &ChannelProgram, machine: Option<&Machine>) -> Result<()> {
        let Some(machine) = machine else {
            return Err(invalid("channel program needs a machine"));
        };
        self.reset_motion();
        let name = machine.channels().iter().find(|channel| channel.id() == program.channel()).map(|channel| channel.name());
        match name {
            Some(name) => {
                let label = strip_parens(name);
                if label.is_empty() {
                    self.line(&format!("(channel {})", program.channel()));
                } else {
                    self.line(&format!("(channel {} {label})", program.channel()));
                }
            }
            None => self.line(&format!("(channel {})", program.channel())),
        }
        for block in program.blocks() {
            match block {
                AxisBlock::Motion(motion) => self.write_axis_motion(motion, machine)?,
                AxisBlock::Event(event) => self.write_event(event)?,
            }
        }
        Ok(())
    }

    fn write_cl_motion(&mut self, motion: &ClMotion) -> Result<()> {
        match motion {
            ClMotion::Rapid { to, feed } => self.write_xyz("G0", to.tip(), *feed),
            ClMotion::Linear { to, feed, .. } => self.write_xyz("G1", to.tip(), Some(*feed)),
            ClMotion::Arc { to, plane, center, sense, feed, .. } => self.write_arc(*sense, *plane, to.tip(), *center, *feed, &[]),
            ClMotion::Helix { to, plane, center, sense, sweep, feed, .. } => self.write_helix(*sense, *plane, to.tip(), *center, *sweep, *feed, &[]),
            ClMotion::Thread { .. } => Err(invalid("synchronised thread moves are not written by the NGC test sink")),
        }
    }

    fn write_axis_motion(&mut self, motion: &AxisMotion, machine: &Machine) -> Result<()> {
        match motion {
            AxisMotion::Rapid { targets, feed } => self.write_axis_point("G0", targets, *feed, machine),
            AxisMotion::Linear { targets, feed } => self.write_axis_point("G1", targets, Some(*feed), machine),
            AxisMotion::Arc { targets, plane, center, sense, feed } => self.write_axis_curve(*sense, *plane, targets, *center, None, *feed, machine),
            AxisMotion::Helix { targets, plane, center, sense, sweep, feed } => {
                self.write_axis_curve(*sense, *plane, targets, *center, Some(*sweep), *feed, machine)
            }
        }
    }

    fn write_xyz(&mut self, g: &str, tip: Vec3, feed: Option<f64>) -> Result<()> {
        let mut words = vec![g.to_string(), word("X", tip.x())?, word("Y", tip.y())?, word("Z", tip.z())?];
        self.push_feed(&mut words, feed)?;
        self.set_xyz(tip)?;
        self.emit(&words);
        Ok(())
    }

    fn write_axis_point(&mut self, g: &str, targets: &[AxisTarget], feed: Option<f64>, machine: &Machine) -> Result<()> {
        let mut named = Vec::new();
        for target in targets {
            named.push((axis_slot(machine, target.axis())?, target.value()));
        }
        named.sort_by_key(|(slot, _)| *slot);
        let mut words = vec![g.to_string()];
        for (slot, value) in named {
            words.push(word(letter_text(slot)?, value)?);
            self.set_slot(slot, value)?;
        }
        self.push_feed(&mut words, feed)?;
        self.emit(&words);
        Ok(())
    }

    fn write_axis_curve(
        &mut self,
        sense: ArcSense,
        plane: Plane,
        targets: &[AxisTarget],
        center: Vec3,
        sweep: Option<f64>,
        feed: f64,
        machine: &Machine,
    ) -> Result<()> {
        let mut end = self.xyz()?;
        let mut extras = Vec::new();
        for target in targets {
            let slot = axis_slot(machine, target.axis())?;
            if let Some(component) = end.get_mut(slot) {
                *component = target.value();
            } else {
                extras.push((slot, target.value()));
            }
        }
        extras.sort_by_key(|(slot, _)| *slot);
        let end = vec3_from(end)?;
        match sweep {
            Some(sweep) => self.write_helix(sense, plane, end, center, sweep, feed, &extras),
            None => self.write_arc(sense, plane, end, center, feed, &extras),
        }
    }

    fn write_helix(&mut self, sense: ArcSense, plane: Plane, end: Vec3, center: Vec3, sweep: f64, feed: f64, extras: &[(usize, f64)]) -> Result<()> {
        let segments = helix_segments(sweep)?;
        let start = vec3_from(self.xyz()?)?;
        for index in 0..segments {
            let last = index + 1 == segments;
            let tip = helix_tip(start, end, plane, sweep, index, segments)?;
            let extra = if last { extras } else { &[] };
            self.write_arc(sense, plane, tip, center, feed, extra)?;
        }
        Ok(())
    }

    fn write_arc(&mut self, sense: ArcSense, plane: Plane, end: Vec3, center: Vec3, feed: f64, extras: &[(usize, f64)]) -> Result<()> {
        let start = self.xyz()?;
        let mut words = Vec::new();
        if self.plane != Some(plane) {
            words.push(plane_word(plane).to_string());
        }
        words.push(sense_word(sense).to_string());
        words.push(word("X", end.x())?);
        words.push(word("Y", end.y())?);
        words.push(word("Z", end.z())?);
        for (slot, value) in extras {
            words.push(word(letter_text(*slot)?, *value)?);
        }
        let origin_x = start.first().copied().ok_or_else(|| invalid("missing x"))?;
        let origin_y = start.get(1).copied().ok_or_else(|| invalid("missing y"))?;
        let origin_z = start.get(2).copied().ok_or_else(|| invalid("missing z"))?;
        words.push(word("I", center.x() - origin_x)?);
        words.push(word("J", center.y() - origin_y)?);
        words.push(word("K", center.z() - origin_z)?);
        self.push_feed(&mut words, Some(feed))?;
        self.plane = Some(plane);
        for (slot, value) in extras {
            self.set_slot(*slot, *value)?;
        }
        self.set_xyz(end)?;
        self.emit(&words);
        Ok(())
    }

    fn write_event(&mut self, event: &Event) -> Result<()> {
        match event {
            Event::ProcessOn { process, .. } => {
                self.note_on(process);
                self.emit_process("M3", process);
            }
            Event::ProcessOff { process } => {
                self.note_off(process);
                self.emit_process("M5", process);
            }
            Event::Dwell { seconds } => {
                if !seconds.is_finite() || *seconds < 0.0 {
                    return Err(invalid("dwell must be >= 0"));
                }
                self.line(&format!("G4 P{}", fmt_num(*seconds)?));
            }
            Event::ToolChange { tool_id } => self.line(&format!("T{tool_id} M6")),
            Event::Comment { text } => self.line(&format!("({})", strip_parens(text))),
            Event::Coolant { mode } => self.line(&format!("(coolant {})", coolant_label(*mode))),
            Event::Probe { .. } => self.line("(probe)"),
            Event::Custom { name, .. } => {
                let label = strip_parens(name);
                if label.is_empty() { self.line("(custom)") } else { self.line(&format!("(custom {label})")) }
            }
            Event::Sync { id } => self.line(&format!("(sync {id})")),
            Event::CAxis { spindle, engage } => {
                let verb = if *engage { "engage" } else { "disengage" };
                self.line(&format!("(c axis {verb} {})", strip_parens(spindle)));
            }
            Event::SpindleBrake { spindle, clamp } => {
                let verb = if *clamp { "clamp" } else { "release" };
                self.line(&format!("(spindle brake {verb} {})", strip_parens(spindle)));
            }
            Event::LiveToolOn { rpm, direction } => {
                let verb = match direction {
                    LiveDirection::Forward => "forward",
                    LiveDirection::Reverse => "reverse",
                };
                self.line(&format!("(live tool {verb} {rpm})"));
            }
            Event::LiveToolOff => self.line("(live tool off)"),
            Event::FaceInterpolation { on } => {
                let verb = if *on { "on" } else { "off" };
                self.line(&format!("(face interpolation {verb})"));
            }
            Event::CylinderInterpolation { radius } => match radius {
                Some(radius) => self.line(&format!("(cylinder interpolation {radius})")),
                None => self.line("(cylinder interpolation off)"),
            },
        }
        Ok(())
    }

    fn emit_process(&mut self, code: &str, process: &ProcessKind) {
        let label = process_label(process);
        if label.is_empty() { self.line(code) } else { self.line(&format!("{code} ({label})")) }
    }

    fn note_on(&mut self, process: &ProcessKind) {
        if self.processes_on.iter().any(|on| on == process) {
            return;
        }
        self.processes_on.push(process.clone());
    }

    fn note_off(&mut self, process: &ProcessKind) {
        self.processes_on.retain(|on| on != process);
    }

    fn push_feed(&mut self, words: &mut Vec<String>, feed: Option<f64>) -> Result<()> {
        let Some(feed) = feed else {
            return Ok(());
        };
        let text = fmt_num(feed)?;
        if self.feed.as_deref() == Some(text.as_str()) {
            return Ok(());
        }
        words.push(format!("F{text}"));
        self.feed = Some(text);
        Ok(())
    }

    fn xyz(&self) -> Result<[f64; 3]> {
        Ok([self.require_slot(0)?, self.require_slot(1)?, self.require_slot(2)?])
    }

    fn require_slot(&self, slot: usize) -> Result<f64> {
        if let Some(value) = self.axes.get(slot).copied().flatten() {
            return Ok(value);
        }
        Err(invalid(format!("ngc arc needs a prior {} position", letter_text(slot)?)))
    }

    fn set_xyz(&mut self, tip: Vec3) -> Result<()> {
        self.set_slot(0, tip.x())?;
        self.set_slot(1, tip.y())?;
        self.set_slot(2, tip.z())?;
        Ok(())
    }

    fn set_slot(&mut self, slot: usize, value: f64) -> Result<()> {
        let value = canonical_zero(value);
        if !value.is_finite() {
            return Err(invalid("axis position must be finite"));
        }
        let Some(cell) = self.axes.get_mut(slot) else {
            return Err(invalid("axis letter is out of range"));
        };
        *cell = Some(value);
        Ok(())
    }
}

fn plane_word(plane: Plane) -> &'static str {
    match plane {
        Plane::Xy => "G17",
        Plane::Zx => "G18",
        Plane::Yz => "G19",
    }
}

fn sense_word(sense: ArcSense) -> &'static str {
    match sense {
        ArcSense::Cw => "G2",
        ArcSense::Ccw => "G3",
    }
}

fn process_label(kind: &ProcessKind) -> String {
    match kind {
        ProcessKind::Spindle => "spindle".to_string(),
        ProcessKind::Torch => "torch".to_string(),
        ProcessKind::Laser => "laser".to_string(),
        ProcessKind::Extruder => "extruder".to_string(),
        ProcessKind::Knife => "knife".to_string(),
        ProcessKind::Other(name) => strip_parens(name),
    }
}

fn coolant_label(mode: CoolantMode) -> &'static str {
    match mode {
        CoolantMode::Off => "off",
        CoolantMode::Flood => "flood",
        CoolantMode::Mist => "mist",
        CoolantMode::Air => "air",
        CoolantMode::ThroughTool => "through tool",
        CoolantMode::Assist => "assist",
    }
}

/// How many G2/G3 blocks a helix needs. A fractional turn is one block. N exact turns are N blocks.
fn helix_segments(sweep: f64) -> Result<u32> {
    if !sweep.is_finite() || sweep <= 0.0 {
        return Err(invalid("helix sweep must be positive"));
    }
    let turns = sweep / std::f64::consts::TAU;
    if !turns.is_finite() || turns <= 0.0 {
        return Err(invalid("helix sweep is too large"));
    }
    let nearest = turns.round();
    let turns = if (turns - nearest).abs() <= 1.0e-8 { nearest } else { turns };
    if turns > f64::from(MAX_HELIX_TURNS) {
        return Err(invalid(format!("helix has more than {MAX_HELIX_TURNS} turns")));
    }
    let segments = if turns < 1.0 { 1.0 } else { turns.ceil() };
    if segments > f64::from(MAX_HELIX_TURNS) {
        return Err(invalid(format!("helix has more than {MAX_HELIX_TURNS} turns")));
    }
    let whole = segments as u32;
    if (f64::from(whole) - segments).abs() > 1.0e-6 || whole == 0 {
        return Err(invalid("helix turn count is not a whole number of blocks"));
    }
    Ok(whole)
}

fn helix_tip(start: Vec3, end: Vec3, plane: Plane, sweep: f64, index: u32, segments: u32) -> Result<Vec3> {
    if index + 1 == segments {
        return Ok(end);
    }
    let fraction = f64::from(index + 1) * std::f64::consts::TAU / sweep;
    if !fraction.is_finite() {
        return Err(invalid("helix fraction is not finite"));
    }
    let next = axial(plane, start) + (axial(plane, end) - axial(plane, start)) * fraction;
    if !next.is_finite() {
        return Err(invalid("helix axis position is not finite"));
    }
    put_axial(plane, start, next)
}

fn axial(plane: Plane, point: Vec3) -> f64 {
    match plane {
        Plane::Xy => point.z(),
        Plane::Zx => point.y(),
        Plane::Yz => point.x(),
    }
}

fn put_axial(plane: Plane, point: Vec3, value: f64) -> Result<Vec3> {
    match plane {
        Plane::Xy => Vec3::new(point.x(), point.y(), value),
        Plane::Zx => Vec3::new(point.x(), value, point.z()),
        Plane::Yz => Vec3::new(value, point.y(), point.z()),
    }
}

fn axis_slot(machine: &Machine, id: u8) -> Result<usize> {
    let axis = machine.axes().iter().find(|axis| axis.id() == id).ok_or_else(|| invalid(format!("axis {id} is not on the machine")))?;
    match axis.name() {
        AxisName::Letter(letter) => Ok(letter_slot(letter)),
        AxisName::Index(_) => Err(IrError::NotSupported("indexed axis in the ngc test sink".into())),
    }
}

fn letter_slot(letter: AxisLetter) -> usize {
    match letter {
        AxisLetter::X => 0,
        AxisLetter::Y => 1,
        AxisLetter::Z => 2,
        AxisLetter::A => 3,
        AxisLetter::B => 4,
        AxisLetter::C => 5,
        AxisLetter::U => 6,
        AxisLetter::V => 7,
        AxisLetter::W => 8,
    }
}

fn letter_text(slot: usize) -> Result<&'static str> {
    LETTERS.get(slot).copied().ok_or_else(|| invalid("axis letter is out of range"))
}

fn vec3_from(values: [f64; 3]) -> Result<Vec3> {
    let x = values.first().copied().ok_or_else(|| invalid("missing x"))?;
    let y = values.get(1).copied().ok_or_else(|| invalid("missing y"))?;
    let z = values.get(2).copied().ok_or_else(|| invalid("missing z"))?;
    Vec3::new(x, y, z)
}

fn word(letter: &str, value: f64) -> Result<String> {
    Ok(format!("{letter}{}", fmt_num(value)?))
}

fn fmt_num(value: f64) -> Result<String> {
    if !value.is_finite() {
        return Err(invalid("ngc number must be finite"));
    }
    let text = format!("{:.4}", canonical_zero(value));
    // `{:.4}` keeps a minus on values that round to zero, so `-0.0000` is normalized here.
    if text == "-0.0000" { Ok("0.0000".to_string()) } else { Ok(text) }
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

/// Drop `(` and `)` so a comment cannot close early. Line breaks become spaces.
fn strip_parens(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '(' | ')' => {}
            '\n' | '\r' => out.push(' '),
            other => out.push(other),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use super::{NgcOptions, ProgramEnd, write_ngc_with};
    use crate::geom::{ArcSense, Plane, Pose, Vec3};
    use crate::machine::{Axis, AxisKind, AxisLetter, AxisName, Channel, KinematicNode, Machine};
    use crate::motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole};
    use crate::process::{CoolantMode, Event, ProcessKind, ProcessParams, Tool};
    use crate::program::{Program, SyncPoint};
    use crate::{IrError, write_ngc};

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn at(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z).expect("point")
    }

    fn path(name: Option<&str>, blocks: Vec<ClBlock>) -> ClPath {
        ClPath::try_new(
            name.map(str::to_string),
            None,
            CompensationMode::Off,
            Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
            None,
            Vec::new(),
            blocks,
        )
        .expect("path")
    }

    fn linear(x: f64, y: f64, z: f64, feed: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Linear { to: pose(x, y, z), feed, role: MoveRole::Cut })
    }

    fn rapid(x: f64, y: f64, z: f64, feed: Option<f64>) -> ClBlock {
        ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, z), feed })
    }

    fn arc(to: (f64, f64, f64), plane: Plane, center: (f64, f64, f64), sense: ArcSense, feed: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Arc {
            to: pose(to.0, to.1, to.2),
            plane,
            center: at(center.0, center.1, center.2),
            sense,
            feed,
            role: MoveRole::Cut,
        })
    }

    fn axis(id: u8, letter: AxisLetter, kind: AxisKind) -> Axis {
        Axis::try_new(id, AxisName::Letter(letter), kind, None, KinematicNode::try_new(None, Vec3::Z, Vec3::ZERO).expect("chain")).expect("axis")
    }

    #[test]
    fn empty_program_is_metric_absolute() {
        assert_eq!(write_ngc(&Program::default()).expect("ngc"), "G21\nG90\nM30\n");
    }

    #[test]
    fn default_ends_with_m30() {
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, vec![rapid(1.0, 2.0, 3.0, None)])], Vec::new(), Vec::new(), None)
            .expect("program");
        assert_eq!(
            write_ngc(&program).expect("ngc"),
            "\
G21
G90
G0 X1.0000 Y2.0000 Z3.0000
M30
"
        );
    }

    #[test]
    fn m2_option_ends_with_m2() {
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, vec![rapid(1.0, 2.0, 3.0, None)])], Vec::new(), Vec::new(), None)
            .expect("program");
        let options = NgcOptions { program_end: ProgramEnd::M2 };
        assert_eq!(
            write_ngc_with(&program, options).expect("ngc"),
            "\
G21
G90
G0 X1.0000 Y2.0000 Z3.0000
M2
"
        );
    }

    #[test]
    fn open_process_is_stopped_before_program_end() {
        let blocks = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Laser, params: ProcessParams::new() }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
        ];
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, blocks)], Vec::new(), Vec::new(), None).expect("program");
        assert_eq!(
            write_ngc(&program).expect("ngc"),
            "\
G21
G90
M3 (laser)
M3 (torch)
M5 (laser)
M3 (spindle)
M5 (torch)
M5 (spindle)
M30
"
        );
    }

    #[test]
    fn small_program_matches_exactly() {
        let outer = path(
            Some("outer (left)"),
            vec![
                rapid(0.0, 0.0, 5.0, None),
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
                linear(10.0, 0.0, 0.0, 250.0),
                linear(12.0, 0.0, 0.0, 250.0),
                arc((12.0, 2.0, 0.0), Plane::Xy, (12.0, 1.0, 0.0), ArcSense::Ccw, 100.0),
                arc((10.0, 2.0, 0.0), Plane::Xy, (11.0, 2.0, 0.0), ArcSense::Ccw, 100.0),
                arc((11.0, 2.0, 1.0), Plane::Zx, (11.0, 2.0, 0.0), ArcSense::Cw, 100.0),
                arc((11.0, 3.0, 2.0), Plane::Yz, (11.0, 2.0, 2.0), ArcSense::Ccw, 100.0),
                ClBlock::Motion(ClMotion::Helix {
                    to: pose(11.0, 1.0, 1.0),
                    plane: Plane::Xy,
                    center: at(11.0, 2.0, 2.0),
                    sense: ArcSense::Ccw,
                    sweep: TAU / 2.0,
                    feed: 80.0,
                    role: MoveRole::Plunge,
                }),
                ClBlock::Event(Event::Dwell { seconds: 0.5 }),
                ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
                ClBlock::Event(Event::ToolChange { tool_id: 4 }),
                ClBlock::Event(Event::Comment { text: "use (finish) pass\nnow".into() }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::Flood }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::Mist }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::Air }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::Off }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::ThroughTool }),
                ClBlock::Event(Event::Coolant { mode: CoolantMode::Assist }),
                ClBlock::Event(Event::Probe { toward: None, feed: None }),
                ClBlock::Event(Event::Custom { name: "mark(er)".into(), params: std::collections::BTreeMap::new() }),
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() }),
                ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }),
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Extruder, params: ProcessParams::new() }),
                ClBlock::Event(Event::ProcessOff { process: ProcessKind::Knife }),
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Other("a(b)".into()), params: ProcessParams::new() }),
                ClBlock::Event(Event::ProcessOff { process: ProcessKind::Other("water jet".into()) }),
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Other("()".into()), params: ProcessParams::new() }),
                ClBlock::Event(Event::Comment { text: "()".into() }),
                rapid(0.0, 0.0, 5.0, Some(1000.0)),
                linear(1.0, 0.0, 5.0, 1000.0),
                linear(2.0, 0.0, 5.0, 80.0),
                rapid(2.0, 0.0, 8.0, None),
                linear(3.0, 0.0, 8.0, 80.0),
            ],
        );
        let hole = path(Some("hole"), vec![rapid(-1.5, 0.0, 0.0, None), linear(-1.5, 2.0, 0.0, 80.0)]);
        let tools = vec![Tool::try_new(4, None, None, None).expect("tool")];
        let program = Program::try_new(Some("plate (rev 1)".into()), tools, None, vec![outer, hole], Vec::new(), Vec::new(), None).expect("program");

        let text = write_ngc(&program).expect("ngc");
        assert_eq!(
            text,
            "\
G21
G90
(program plate rev 1)
(toolpath outer left)
G0 X0.0000 Y0.0000 Z5.0000
M3 (spindle)
G1 X10.0000 Y0.0000 Z0.0000 F250.0000
G1 X12.0000 Y0.0000 Z0.0000
G17 G3 X12.0000 Y2.0000 Z0.0000 I0.0000 J1.0000 K0.0000 F100.0000
G3 X10.0000 Y2.0000 Z0.0000 I-1.0000 J0.0000 K0.0000
G18 G2 X11.0000 Y2.0000 Z1.0000 I1.0000 J0.0000 K0.0000
G19 G3 X11.0000 Y3.0000 Z2.0000 I0.0000 J0.0000 K1.0000
G17 G3 X11.0000 Y1.0000 Z1.0000 I0.0000 J-1.0000 K0.0000 F80.0000
G4 P0.5000
M5 (spindle)
T4 M6
(use finish pass now)
(coolant flood)
(coolant mist)
(coolant air)
(coolant off)
(coolant through tool)
(coolant assist)
(probe)
(custom marker)
M3 (torch)
M5 (laser)
M3 (extruder)
M5 (knife)
M3 (ab)
M5 (water jet)
M3
()
G0 X0.0000 Y0.0000 Z5.0000 F1000.0000
G1 X1.0000 Y0.0000 Z5.0000
G1 X2.0000 Y0.0000 Z5.0000 F80.0000
G0 X2.0000 Y0.0000 Z8.0000
G1 X3.0000 Y0.0000 Z8.0000
(toolpath hole)
G0 X-1.5000 Y0.0000 Z0.0000
G1 X-1.5000 Y2.0000 Z0.0000 F80.0000
M5 (torch)
M5 (extruder)
M5 (ab)
M5
M30
"
        );
    }

    #[test]
    fn helix_turns_and_axial_words_match_exactly() {
        let blocks = vec![
            rapid(10.0, 0.0, 0.0, None),
            ClBlock::Motion(ClMotion::Helix {
                to: pose(10.0, 0.0, -2.0),
                plane: Plane::Xy,
                center: Vec3::ZERO,
                sense: ArcSense::Ccw,
                sweep: TAU * 2.0,
                feed: 60.0,
                role: MoveRole::Cut,
            }),
            ClBlock::Motion(ClMotion::Helix {
                to: pose(10.0, -4.0, 0.0),
                plane: Plane::Zx,
                center: Vec3::ZERO,
                sense: ArcSense::Cw,
                sweep: TAU,
                feed: 25.0,
                role: MoveRole::Cut,
            }),
        ];
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, blocks)], Vec::new(), Vec::new(), None).expect("program");
        let text = write_ngc(&program).expect("ngc");
        assert_eq!(
            text,
            "\
G21
G90
G0 X10.0000 Y0.0000 Z0.0000
G17 G3 X10.0000 Y0.0000 Z-1.0000 I-10.0000 J0.0000 K0.0000 F60.0000
G3 X10.0000 Y0.0000 Z-2.0000 I-10.0000 J0.0000 K1.0000
G18 G2 X10.0000 Y-4.0000 Z0.0000 I-10.0000 J0.0000 K2.0000 F25.0000
M30
"
        );
    }

    #[test]
    fn channel_program_matches_exactly() {
        let machine = Machine::try_new(
            vec![
                axis(0, AxisLetter::X, AxisKind::Linear),
                axis(1, AxisLetter::Y, AxisKind::Linear),
                axis(2, AxisLetter::Z, AxisKind::Linear),
                axis(3, AxisLetter::A, AxisKind::Rotary),
                axis(4, AxisLetter::B, AxisKind::Rotary),
            ],
            vec![Channel::try_new(0, "he(ad)", vec![0, 1, 2, 3]).expect("head"), Channel::try_new(1, "a(ux)", vec![4]).expect("aux")],
        )
        .expect("machine");
        let target = |id, value| AxisTarget::try_new(id, value).expect("target");
        let head = ChannelProgram::try_new(
            0,
            vec![
                AxisBlock::Motion(AxisMotion::Rapid { targets: vec![target(2, 3.0), target(0, 1.0), target(1, 2.0)], feed: None }),
                AxisBlock::Motion(AxisMotion::Linear { targets: vec![target(3, 0.5), target(0, 5.0)], feed: 40.0 }),
                AxisBlock::Motion(AxisMotion::Arc {
                    targets: vec![target(1, 6.0)],
                    plane: Plane::Xy,
                    center: at(5.0, 4.0, 3.0),
                    sense: ArcSense::Ccw,
                    feed: 40.0,
                }),
                AxisBlock::Motion(AxisMotion::Helix {
                    targets: vec![target(2, -1.0), target(3, 2.0)],
                    plane: Plane::Xy,
                    center: at(0.0, 6.0, 3.0),
                    sense: ArcSense::Ccw,
                    sweep: TAU * 2.0,
                    feed: 15.0,
                }),
                AxisBlock::Event(Event::Sync { id: 3 }),
            ],
        )
        .expect("head");
        let aux = ChannelProgram::try_new(1, vec![AxisBlock::Event(Event::Sync { id: 3 })]).expect("aux");
        let program = Program::try_new(
            None,
            Vec::new(),
            Some(machine),
            Vec::new(),
            vec![head, aux],
            vec![SyncPoint::try_new(3, vec![0, 1]).expect("sync")],
            None,
        )
        .expect("program");
        let text = write_ngc(&program).expect("ngc");
        assert_eq!(
            text,
            "\
G21
G90
(channel 0 head)
G0 X1.0000 Y2.0000 Z3.0000
G1 X5.0000 A0.5000 F40.0000
G17 G3 X5.0000 Y6.0000 Z3.0000 I0.0000 J2.0000 K0.0000
G3 X5.0000 Y6.0000 Z1.0000 I-5.0000 J0.0000 K0.0000 F15.0000
G3 X5.0000 Y6.0000 Z-1.0000 A2.0000 I-5.0000 J0.0000 K2.0000
(sync 3)
(channel 1 aux)
(sync 3)
M30
"
        );
    }

    #[test]
    fn arc_without_a_position_is_rejected() {
        let arc =
            ClMotion::Arc { to: pose(1.0, 0.0, 0.0), plane: Plane::Xy, center: Vec3::ZERO, sense: ArcSense::Ccw, feed: 10.0, role: MoveRole::Cut };
        let only = Program::try_new(None, Vec::new(), None, vec![path(None, vec![ClBlock::Motion(arc.clone())])], Vec::new(), Vec::new(), None)
            .expect("program");
        let err = write_ngc(&only).expect_err("arc");
        assert!(err.to_string().contains("prior X"), "{err}");

        let moved = path(None, vec![rapid(0.0, 0.0, 0.0, None)]);
        let again = path(None, vec![ClBlock::Motion(arc)]);
        let split = Program::try_new(None, Vec::new(), None, vec![moved, again], Vec::new(), Vec::new(), None).expect("program");
        let err = write_ngc(&split).expect_err("reset");
        assert!(err.to_string().contains("prior X"), "{err}");
    }

    #[test]
    fn axis_arc_needs_each_centre_axis() {
        let machine = Machine::try_new(
            vec![axis(0, AxisLetter::X, AxisKind::Linear), axis(1, AxisLetter::Y, AxisKind::Linear), axis(2, AxisLetter::Z, AxisKind::Linear)],
            vec![Channel::try_new(0, "mill", vec![0, 1, 2]).expect("channel")],
        )
        .expect("machine");
        let target = |id, value| AxisTarget::try_new(id, value).expect("target");
        let channel = ChannelProgram::try_new(
            0,
            vec![
                AxisBlock::Motion(AxisMotion::Rapid { targets: vec![target(0, 1.0), target(1, 0.0)], feed: None }),
                AxisBlock::Motion(AxisMotion::Arc {
                    targets: vec![target(0, 0.0), target(1, 1.0)],
                    plane: Plane::Xy,
                    center: Vec3::ZERO,
                    sense: ArcSense::Ccw,
                    feed: 10.0,
                }),
            ],
        )
        .expect("channel");
        let program = Program::try_new(None, Vec::new(), Some(machine), Vec::new(), vec![channel], Vec::new(), None).expect("program");
        let err = write_ngc(&program).expect_err("z");
        assert!(err.to_string().contains("prior Z"), "{err}");
    }

    #[test]
    fn indexed_axis_is_not_supported() {
        let axis = Axis::try_new(
            0,
            AxisName::for_ordinal(9).expect("index"),
            AxisKind::Linear,
            None,
            KinematicNode::try_new(None, Vec3::Z, Vec3::ZERO).expect("chain"),
        )
        .expect("axis");
        let machine = Machine::try_new(vec![axis], vec![Channel::try_new(0, "extra", vec![0]).expect("channel")]).expect("machine");
        let motion = AxisMotion::Linear { targets: vec![AxisTarget::try_new(0, 1.0).expect("target")], feed: 10.0 };
        let channel = ChannelProgram::try_new(0, vec![AxisBlock::Motion(motion)]).expect("channel");
        let program = Program::try_new(None, Vec::new(), Some(machine), Vec::new(), vec![channel], Vec::new(), None).expect("program");
        let err = write_ngc(&program).expect_err("index");
        assert_eq!(err, IrError::NotSupported("indexed axis in the ngc test sink".into()));
    }

    #[test]
    fn too_many_helix_turns_are_rejected() {
        let blocks = vec![
            rapid(1.0, 0.0, 0.0, None),
            ClBlock::Motion(ClMotion::Helix {
                to: pose(1.0, 0.0, -1.0),
                plane: Plane::Xy,
                center: Vec3::ZERO,
                sense: ArcSense::Cw,
                sweep: TAU * 10_001.0,
                feed: 10.0,
                role: MoveRole::Cut,
            }),
        ];
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, blocks)], Vec::new(), Vec::new(), None).expect("program");
        let err = write_ngc(&program).expect_err("turns");
        assert!(err.to_string().contains("10000"), "{err}");
    }

    #[test]
    fn negative_zero_prints_as_zero() {
        let blocks = vec![
            rapid(-0.0, -4.440892098500626e-16, -0.00004, None),
            linear(-0.00006, -0.0, -7.669517010585678e-17, 8.0),
            linear(-1.0, 0.0, 0.0, 8.0),
            arc((-1.0, 1.0, -1.0e-16), Plane::Xy, (-1.0, -4.440892098500626e-16, 0.0), ArcSense::Ccw, 8.0),
        ];
        let program = Program::try_new(None, Vec::new(), None, vec![path(None, blocks)], Vec::new(), Vec::new(), None).expect("program");
        let text = write_ngc(&program).expect("ngc");
        assert_eq!(
            text,
            "\
G21
G90
G0 X0.0000 Y0.0000 Z0.0000
G1 X-0.0001 Y0.0000 Z0.0000 F8.0000
G1 X-1.0000 Y0.0000 Z0.0000
G17 G3 X-1.0000 Y1.0000 Z0.0000 I0.0000 J0.0000 K0.0000
M30
"
        );
    }

    #[test]
    fn channel_program_without_a_machine_is_rejected() {
        let program: Program =
            serde_json::from_str(r#"{"tools":[],"toolpaths":[],"channel_programs":[{"channel":0,"blocks":[]}],"syncs":[]}"#).expect("json");
        let err = write_ngc(&program).expect_err("machine");
        assert!(err.to_string().contains("needs a machine"), "{err}");
    }
}
