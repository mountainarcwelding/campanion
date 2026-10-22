//! Write cutter-location programs as RS274/NGC text.

use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Event, Plane, Pose, ProcessKind, Program, Vec3};

use crate::def::{ArcMode, PostDef, ProcessWords};
use crate::error::{Result, invalid, unsupported};
use crate::fmt::format_fixed;
use crate::geom::{self, Xyz};

/// Controls how a [`PostDef`] writes arcs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PostOptions {
    /// Chord error, in millimetres, when an arc or helix is written as lines.
    pub arc_tolerance: f64,
}

/// Write `program` with `post`.
///
/// Only cutter-location toolpaths are written. A channel program returns
/// [`crate::PostError::NotSupported`]. The string ends with a newline.
pub fn post_program(program: &Program, post: &PostDef, options: &PostOptions) -> Result<String> {
    if !options.arc_tolerance.is_finite() || options.arc_tolerance < 0.0 {
        return Err(invalid("arc_tolerance must be finite and >= 0"));
    }
    if !program.channel_programs().is_empty() {
        return Err(unsupported("channel programs are not supported; only cutter-location toolpaths are posted"));
    }
    let mut writer = Writer::new(post, options);
    let name = sanitize(program.name().unwrap_or(""));
    let header_vars = [("program_name", name.as_str()), ("date-free!", "")];
    writer.templates("header", &post.header, &header_vars)?;
    for path in program.toolpaths() {
        writer.toolpath(path)?;
    }
    writer.stop_processes()?;
    writer.templates("footer", &post.footer, &header_vars)?;
    writer.templates("program_end", &post.program_end, &header_vars)?;
    Ok(writer.out)
}

#[derive(Clone)]
struct Active {
    key: String,
    speed: String,
    power: String,
}

struct Writer<'a> {
    post: &'a PostDef,
    options: &'a PostOptions,
    out: String,
    /// Last block's end, rounded to the digits that were written. The next `I` `J` `K` uses this.
    pos: Option<Xyz>,
    plane: Option<Plane>,
    /// Last `G0` or `G1`, cleared by an arc so the next line is not modal.
    linear: Option<&'static str>,
    feed: Option<String>,
    line_no: u32,
    on: Vec<Active>,
}

impl<'a> Writer<'a> {
    fn new(post: &'a PostDef, options: &'a PostOptions) -> Self {
        let plane = if post.planes { header_plane(&post.header) } else { header_plane(&post.header).or(Some(Plane::Xy)) };
        Self { post, options, out: String::new(), pos: None, plane, linear: None, feed: None, line_no: post.line_numbers.start, on: Vec::new() }
    }

    fn toolpath(&mut self, path: &ClPath) -> Result<()> {
        match path.compensation() {
            CompensationMode::ControllerLeft => return Err(unsupported("controller left compensation is not supported")),
            CompensationMode::ControllerRight => return Err(unsupported("controller right compensation is not supported")),
            CompensationMode::Computed | CompensationMode::Off => {}
        }
        if let Some(name) = path.name() {
            self.comment(&format!("toolpath {}", sanitize(name)))?;
        }
        for block in path.blocks() {
            match block {
                ClBlock::Motion(motion) => self.motion(motion)?,
                ClBlock::Event(event) => self.event(event)?,
            }
        }
        Ok(())
    }

    fn motion(&mut self, motion: &ClMotion) -> Result<()> {
        match motion {
            ClMotion::Rapid { to, feed } => self.linear_move("G0", tip(*to), *feed),
            ClMotion::Linear { to, feed, .. } => self.linear_move("G1", tip(*to), Some(*feed)),
            ClMotion::Arc { to, plane, center, sense, feed, .. } => {
                let end = tip(*to);
                let center = xyz(*center);
                let sweep = geom::arc_sweep(self.pos.ok_or_else(|| invalid("arc needs a prior position"))?, end, center, *plane, *sense)?;
                self.circular(*sense, *plane, end, center, sweep, *feed, false)
            }
            ClMotion::Helix { to, plane, center, sense, sweep, feed, .. } => {
                let end = tip(*to);
                let center = xyz(*center);
                let start = self.pos.ok_or_else(|| invalid("helix needs a prior position"))?;
                geom::same_radius(start, end, center, *plane)?;
                if !(sweep.is_finite() && *sweep > 0.0) {
                    return Err(invalid("helix sweep must be positive"));
                }
                if *sweep > std::f64::consts::TAU * 10_000.0 {
                    return Err(invalid("helix has more than 10000 turns"));
                }
                self.circular(*sense, *plane, end, center, *sweep, *feed, true)
            }
        }
    }

    fn circular(&mut self, sense: campanion_ir::ArcSense, plane: Plane, end: Xyz, center: Xyz, sweep: f64, feed: f64, helix: bool) -> Result<()> {
        let start = self.pos.ok_or_else(|| invalid("arc needs a prior position"))?;
        if !self.post.planes {
            let fixed = self.plane.unwrap_or(Plane::Xy);
            if plane != fixed {
                return Err(unsupported("planes is false and the arc is not in the header plane"));
            }
        }
        let (_, _, start_axial) = geom::plane_uv(plane, start);
        let (_, _, end_axial) = geom::plane_uv(plane, end);
        let helical = helix || (end_axial - start_axial).abs() > 1e-9;
        if self.post.arc_mode() == ArcMode::Lines || (helical && !self.post.arcs.helix) {
            return self.linearise_curve(sense, plane, start, end, center, sweep, feed);
        }
        let predicted = geom::point_at(start, end, center, plane, sense, sweep, 1.0, false);
        let radius = geom::in_plane_len(plane, start, center);
        if geom::dist3(predicted, end) > 1e-3_f64.max(radius * 1e-5) {
            return Err(invalid("arc end does not match the centre and sweep"));
        }
        let max_sweep = self.post.arcs.max_sweep_deg.to_radians();
        let pieces = geom::piece_count(sweep, max_sweep)?;
        for index in 0..pieces {
            let last = index + 1 == pieces;
            let t1 = if last { 1.0 } else { max_sweep * f64::from(index + 1) / sweep };
            let tip = geom::point_at(start, end, center, plane, sense, sweep, t1, true);
            let from = self.pos.ok_or_else(|| invalid("arc needs a prior position"))?;
            let piece = if last { sweep - max_sweep * f64::from(index) } else { max_sweep };
            self.one_arc(sense, plane, from, tip, center, piece, feed)?;
        }
        Ok(())
    }

    fn linearise_curve(
        &mut self,
        sense: campanion_ir::ArcSense,
        plane: Plane,
        start: Xyz,
        end: Xyz,
        center: Xyz,
        sweep: f64,
        feed: f64,
    ) -> Result<()> {
        let radius = geom::in_plane_len(plane, start, center);
        let segments = geom::chord_segments(sweep, radius, self.options.arc_tolerance)?;
        for index in 1..=segments {
            let t = if index == segments { 1.0 } else { f64::from(index) / f64::from(segments) };
            let tip = geom::point_at(start, end, center, plane, sense, sweep, t, true);
            self.linear_move("G1", tip, Some(feed))?;
        }
        Ok(())
    }

    fn one_arc(&mut self, sense: campanion_ir::ArcSense, plane: Plane, start: Xyz, end: Xyz, center: Xyz, sweep: f64, feed: f64) -> Result<()> {
        let mut words = Vec::new();
        if self.post.planes && self.plane != Some(plane) {
            words.push(plane_word(plane).to_string());
        }
        words.push(sense_word(sense).to_string());
        let rounded = self.push_axes(&mut words, end)?;
        match self.post.arc_mode() {
            ArcMode::R => {
                let radius = geom::in_plane_len(plane, start, center);
                let signed = if sweep > std::f64::consts::PI + 1e-8 { -radius } else { radius };
                words.push(format!("R{}", format_fixed(signed, self.post.decimals.linear, self.post.trailing_zeros)?));
            }
            ArcMode::Ijk => {
                let offsets = [center[0] - start[0], center[1] - start[1], center[2] - start[2]];
                for (letter, value) in [("I", offsets[0]), ("J", offsets[1]), ("K", offsets[2])] {
                    words.push(format!("{letter}{}", format_fixed(value, self.post.decimals.linear, self.post.trailing_zeros)?));
                }
            }
            ArcMode::Lines => return Err(invalid("line mode reached an arc block")),
        }
        self.push_feed(&mut words, Some(feed))?;
        if self.post.planes {
            self.plane = Some(plane);
        }
        self.linear = None;
        self.pos = Some(rounded);
        self.emit_words(&words)
    }

    fn linear_move(&mut self, code: &str, end: Xyz, feed: Option<f64>) -> Result<()> {
        let mut words = Vec::new();
        if !(self.post.modal_motion && self.linear == Some(code)) {
            words.push(code.to_string());
        }
        let rounded = self.push_axes(&mut words, end)?;
        self.push_feed(&mut words, feed)?;
        self.linear = Some(if code == "G0" { "G0" } else { "G1" });
        self.pos = Some(rounded);
        self.emit_words(&words)
    }

    fn push_axes(&mut self, words: &mut Vec<String>, end: Xyz) -> Result<Xyz> {
        let letters = self.post.axis_letters.chars().collect::<Vec<_>>();
        let mut rounded = [0.0; 3];
        for (index, letter) in letters.iter().enumerate() {
            let Some(value) = end.get(index).copied() else {
                return Err(invalid("axis value is missing"));
            };
            let text = format_fixed(value, self.post.decimals.linear, self.post.trailing_zeros)?;
            rounded[index] = text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
            words.push(format!("{letter}{text}"));
        }
        Ok(rounded)
    }

    fn push_feed(&mut self, words: &mut Vec<String>, feed: Option<f64>) -> Result<()> {
        let Some(feed) = feed else {
            return Ok(());
        };
        if !(feed.is_finite() && feed > 0.0) {
            return Err(invalid("feed must be positive"));
        }
        let text = format_fixed(feed, self.post.decimals.feed, self.post.trailing_zeros)?;
        if self.feed.as_deref() == Some(text.as_str()) {
            return Ok(());
        }
        words.push(format!("F{text}"));
        self.feed = Some(text);
        Ok(())
    }

    fn event(&mut self, event: &Event) -> Result<()> {
        match event {
            Event::ProcessOn { process, params } => {
                let lines = self.process_words(process)?.on.clone();
                let speed = format_fixed(format_param(params.speed_value())?, self.post.decimals.feed, self.post.trailing_zeros)?;
                let power = format_fixed(format_param(params.power_value())?, self.post.decimals.feed, self.post.trailing_zeros)?;
                self.note_on(process, &speed, &power);
                self.templates(
                    "process.on",
                    &lines,
                    &[("speed", speed.as_str()), ("power", power.as_str()), ("program_name", ""), ("date-free!", "")],
                )
            }
            Event::ProcessOff { process } => {
                let lines = self.process_words(process)?.off.clone();
                let active = self.on.iter().find(|on| on.key == process_key(process)).cloned();
                let speed = active.as_ref().map(|on| on.speed.clone()).unwrap_or_else(|| "0".to_string());
                let power = active.as_ref().map(|on| on.power.clone()).unwrap_or_else(|| "0".to_string());
                self.note_off(process);
                self.templates(
                    "process.off",
                    &lines,
                    &[("speed", speed.as_str()), ("power", power.as_str()), ("program_name", ""), ("date-free!", "")],
                )
            }
            Event::Dwell { seconds } => self.dwell(*seconds),
            Event::ToolChange { tool_id } => {
                let lines = self.post.tool_change.clone();
                let tool = tool_id.to_string();
                self.templates("tool_change", &lines, &[("tool", tool.as_str()), ("program_name", ""), ("date-free!", "")])
            }
            Event::Coolant { mode } => self.coolant(*mode),
            Event::Comment { text } => self.comment(text),
            Event::Probe { .. } => self.comment("probe"),
            Event::Custom { name, .. } => self.comment(&format!("custom {}", sanitize(name))),
            Event::Sync { id } => self.comment(&format!("sync {id}")),
        }
    }

    fn process_words(&self, process: &ProcessKind) -> Result<&ProcessWords> {
        let key = process_key(process);
        if let Some(words) = self.post.process.get(key) {
            return Ok(words);
        }
        if matches!(process, ProcessKind::Other(_))
            && let Some(words) = self.post.process.get("other")
        {
            return Ok(words);
        }
        Err(invalid(format!("process.{key} has no template")))
    }

    fn note_on(&mut self, process: &ProcessKind, speed: &str, power: &str) {
        let key = process_key(process).to_string();
        if let Some(active) = self.on.iter_mut().find(|on| on.key == key) {
            active.speed = speed.to_string();
            active.power = power.to_string();
            return;
        }
        self.on.push(Active { key, speed: speed.to_string(), power: power.to_string() });
    }

    fn note_off(&mut self, process: &ProcessKind) {
        let key = process_key(process);
        self.on.retain(|on| on.key != key);
    }

    fn stop_processes(&mut self) -> Result<()> {
        let on = std::mem::take(&mut self.on);
        for active in on {
            let Some(words) = self.post.process.get(&active.key).or_else(|| self.post.process.get("other")) else {
                return Err(invalid(format!("process.{} has no template", active.key)));
            };
            let off = words.off.clone();
            self.templates(
                "process.off",
                &off,
                &[("speed", active.speed.as_str()), ("power", active.power.as_str()), ("program_name", ""), ("date-free!", "")],
            )?;
        }
        Ok(())
    }

    fn dwell(&mut self, seconds: f64) -> Result<()> {
        if !(seconds.is_finite() && seconds >= 0.0) {
            return Err(invalid("dwell must be >= 0"));
        }
        let value = if self.post.dwell.unit == "ms" { seconds * 1000.0 } else { seconds };
        let text = format_fixed(value, self.post.decimals.dwell, self.post.trailing_zeros)?;
        self.emit_line(&format!("G4 {}{text}", self.post.dwell.word))
    }

    fn coolant(&mut self, mode: CoolantMode) -> Result<()> {
        let lines = match mode {
            CoolantMode::Flood => self.post.coolant.flood.clone(),
            CoolantMode::Mist => self.post.coolant.mist.clone(),
            CoolantMode::Off => self.post.coolant.off.clone(),
            CoolantMode::Air => return self.comment("coolant air"),
            CoolantMode::ThroughTool => return self.comment("coolant through tool"),
            CoolantMode::Assist => return self.comment("coolant assist"),
        };
        self.templates("coolant", &lines, &[("program_name", ""), ("date-free!", "")])
    }

    fn comment(&mut self, text: &str) -> Result<()> {
        let clean = sanitize(text);
        if clean.is_empty() {
            return Ok(());
        }
        match self.post.comments.style.as_str() {
            "paren" => self.emit_line(&format!("({clean})")),
            "semicolon" => self.emit_line(&format!("; {clean}")),
            _ => Ok(()),
        }
    }

    fn templates(&mut self, field_name: &str, lines: &[String], vars: &[(&str, &str)]) -> Result<()> {
        for line in lines {
            let text = substitute(field_name, line, vars)?;
            if meaningless(&text) {
                continue;
            }
            self.emit_line(&text)?;
        }
        Ok(())
    }

    fn emit_words(&mut self, words: &[String]) -> Result<()> {
        self.emit_line(&words.join(" "))
    }

    fn emit_line(&mut self, body: &str) -> Result<()> {
        let mut line = String::new();
        if self.post.line_numbers.enabled {
            line.push_str(&self.post.line_numbers.prefix);
            line.push_str(&self.line_no.to_string());
            self.line_no = self.line_no.saturating_add(self.post.line_numbers.step);
            if !body.is_empty() {
                line.push(' ');
                line.push_str(body);
            }
        } else {
            line.push_str(body);
        }
        if let Some(limit) = self.post.max_line_length
            && line.chars().count() > limit as usize
        {
            return Err(invalid(format!("line exceeds max_line_length {limit}")));
        }
        self.out.push_str(&line);
        self.out.push('\n');
        Ok(())
    }
}

fn format_param(value: Option<f64>) -> Result<f64> {
    let value = value.unwrap_or(0.0);
    if value.is_finite() && value >= 0.0 { Ok(value) } else { Err(invalid("process speed or power must be >= 0")) }
}

fn process_key(kind: &ProcessKind) -> &str {
    match kind {
        ProcessKind::Spindle => "spindle",
        ProcessKind::Torch => "torch",
        ProcessKind::Laser => "laser",
        ProcessKind::Extruder => "extruder",
        ProcessKind::Knife => "knife",
        ProcessKind::Other(name) => name.as_str(),
    }
}

fn tip(pose: Pose) -> Xyz {
    let point = pose.tip();
    [point.x(), point.y(), point.z()]
}

fn xyz(point: Vec3) -> Xyz {
    [point.x(), point.y(), point.z()]
}

fn plane_word(plane: Plane) -> &'static str {
    match plane {
        Plane::Xy => "G17",
        Plane::Zx => "G18",
        Plane::Yz => "G19",
    }
}

fn sense_word(sense: campanion_ir::ArcSense) -> &'static str {
    match sense {
        campanion_ir::ArcSense::Cw => "G2",
        campanion_ir::ArcSense::Ccw => "G3",
    }
}

fn header_plane(lines: &[String]) -> Option<Plane> {
    let mut plane = None;
    for line in lines {
        for word in line.split_whitespace() {
            match word.to_ascii_uppercase().as_str() {
                "G17" => plane = Some(Plane::Xy),
                "G18" => plane = Some(Plane::Zx),
                "G19" => plane = Some(Plane::Yz),
                _ => {}
            }
        }
    }
    plane
}

fn substitute(field_name: &str, line: &str, vars: &[(&str, &str)]) -> Result<String> {
    let mut out = String::with_capacity(line.len());
    let mut rest = line;
    while let Some(start) = rest.find('{') {
        out.push_str(&rest[..start]);
        let after = &rest[start + 1..];
        let Some(end) = after.find('}') else {
            return Err(invalid(format!("{field_name} has an unclosed '{{'")));
        };
        let key = &after[..end];
        let Some((_, value)) = vars.iter().find(|(name, _)| *name == key) else {
            return Err(invalid(format!("{field_name} has unknown placeholder {{{key}}}")));
        };
        out.push_str(value);
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

fn meaningless(line: &str) -> bool {
    let trimmed = line.trim();
    trimmed.is_empty() || trimmed == "()" || trimmed == ";"
}

fn sanitize(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '(' | ')' | ';' => {}
            '\n' | '\r' | '\t' => out.push(' '),
            ch if ch.is_control() => {}
            ch => out.push(ch),
        }
    }
    out
}
