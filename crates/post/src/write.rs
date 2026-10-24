//! Write cutter-location programs as RS274/NGC text.
//!
//! A pose with a rotary angle is written in the machine frame. `X` `Y` `Z` are
//! [`campanion_ir::part_to_machine`] of the pose, and the A or B word is the continuous angle
//! stored on the IR. Feeds for those moves are described in the post crate README: inverse time
//! uses the straight-line distance between the part-frame tips.
//!
//! A turning program is written in the part frame. X is a radius until the lathe block asks for
//! diameters. Y is omitted, except during face and cylinder interpolation. While the C axis is
//! engaged, `G0` and `G1` append that letter after Z. Outside those modes, arcs stay in the ZX
//! plane and do not carry a C word. Arc offsets stay in the radius frame: `I` along X, `K` along Z.
//! Face interpolation writes Cartesian X Y Z (and XY arcs). Cylinder interpolation writes X Y Z
//! with Y unrolled (and YZ arcs). Without those post blocks the same poses are split into X and C.

use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Event, LiveDirection, Plane, Pose, ProcessKind, ProcessParams, Program, RotaryAxisKind,
    RotarySetup, SpindleSpeed, TurningSetup, Vec3, part_to_machine,
};

use crate::def::{ArcMode, PostDef, ProcessWords, RotaryFeedMode};
use crate::error::{Result, invalid, unsupported};
use crate::fmt::{format_dim, format_fixed, xy_text_eq};
use crate::geom::{self, Xyz};
use crate::interp::{self, PolarCurve, PolarFrame, PolarTarget};
use crate::rotary::{self, angles_equal};

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
    check_thread_support(program, post)?;
    check_lathe_program(program, post)?;
    check_rotary_program(program, post)?;
    let mut writer = Writer::new(post, options, program.rotary().copied(), program.turning());
    let name = sanitize(program.name().unwrap_or(""));
    let number = post.program_number.clone().unwrap_or_default();
    let header_vars = [("program_name", name.as_str()), ("program_number", number.as_str()), ("date-free!", "")];
    writer.templates("header", &post.header, &header_vars)?;
    writer.begin_lathe()?;
    for (index, path) in program.toolpaths().iter().enumerate() {
        let index = u32::try_from(index).map_err(|_| invalid("too many toolpaths"))?;
        writer.toolpath(index, path)?;
    }
    writer.stop_processes()?;
    writer.templates("footer", &post.footer, &header_vars)?;
    writer.finish_mill_turn()?;
    writer.lathe_home()?;
    writer.templates("program_end", &post.program_end, &header_vars)?;
    Ok(writer.out)
}

#[derive(Clone)]
struct Active {
    key: String,
    speed: String,
    power: String,
}

/// Where torch-height control sits relative to the pierce dwell and process off.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ThcPhase {
    /// No process is waiting on a pierce.
    Idle,
    /// Process is on. The pierce dwell has not been written yet.
    AwaitDwell,
    /// Pierce dwell is written. The next feed move turns height control on.
    AwaitCut,
    /// Height control is on.
    On,
}

/// One motion end as the writer emits it.
struct Posted {
    /// `X` `Y` `Z` on the block. Machine frame when the pose carries an angle.
    xyz: Xyz,
    /// Part-frame tip. Inverse-time length is the distance between these.
    part: Xyz,
    /// Continuous A/B angle from the IR, when this pose has one.
    angle: Option<f64>,
    /// C angle in degrees on a turning pose. Written only on `G0` and `G1` while engaged.
    c: Option<f64>,
}

fn plain(end: Xyz) -> Posted {
    Posted { xyz: end, part: end, angle: None, c: None }
}

/// Active face or cylinder frame, and whether the post has a native block for it.
#[derive(Clone, Copy, PartialEq)]
enum CartMode {
    Off,
    Face { native: bool },
    Cylinder { radius: f64, native: bool },
}

struct Writer<'a> {
    post: &'a PostDef,
    options: &'a PostOptions,
    out: String,
    /// Last block's end, rounded to the digits that were written. The next `I` `J` `K` uses this.
    pos: Option<Xyz>,
    /// Last programmed point before rounding. Split arcs are sampled from this so they stay on the
    /// original curve; `pos` is only where the machine was told to be.
    exact: Option<Xyz>,
    /// Last part-frame tip. Equals [`Self::exact`] when the pose has no rotary angle.
    part_tip: Option<Xyz>,
    /// Last IR angle, degrees. `None` until a pose carries one.
    angle: Option<f64>,
    /// True after `CAxis` engage, until disengage.
    c_engaged: bool,
    /// Continuous C in degrees. Reset to 0 on engage (Haas `G28 H0.` unwinds). Face fallback
    /// unwraps against this so a sweep does not jump by more than 180°.
    c_continuous: Option<f64>,
    /// Formatted C word last written. Cleared on disengage so the next engage writes it again.
    c_modal: Option<String>,
    /// Face or cylinder interpolation. `Off` is ordinary XZ turning.
    cart: CartMode,
    /// After interpolation turns off, the next move rewrites X, Y and Z. C is rewritten when the pose has one.
    force_axes: bool,
    /// Segments emitted for the fallback move in progress.
    polar_count: u32,
    /// True after a spindle brake clamp, until release. C may not move while it is set.
    brake_clamped: bool,
    /// True after a live-tool start, until stop.
    live_on: bool,
    /// True once the live-tool feed mode has been considered for this program.
    live_feed_written: bool,
    /// Modal inverse-time feed (`G93` on LinuxCNC). Units per minute is the default.
    inverse_time: bool,
    setup: Option<RotarySetup>,
    /// Turning setup when this program is a lathe program. X in [`Self::pos`] stays a radius.
    turning: Option<&'a TurningSetup>,
    /// Index of the toolpath currently being written. Bindings use this index.
    path_index: u32,
    plane: Option<Plane>,
    /// Last `G0` or `G1`, cleared by an arc so the next line is not modal.
    linear: Option<&'static str>,
    feed: Option<String>,
    line_no: u32,
    on: Vec<Active>,
    thc_phase: ThcPhase,
    last_speed: String,
    last_power: String,
}

impl<'a> Writer<'a> {
    fn new(post: &'a PostDef, options: &'a PostOptions, setup: Option<RotarySetup>, turning: Option<&'a TurningSetup>) -> Self {
        let plane = if post.planes { header_plane(&post.header) } else { header_plane(&post.header).or(Some(Plane::Xy)) };
        Self {
            post,
            options,
            out: String::new(),
            pos: None,
            exact: None,
            part_tip: None,
            angle: None,
            c_engaged: false,
            c_continuous: None,
            brake_clamped: false,
            c_modal: None,
            cart: CartMode::Off,
            force_axes: false,
            polar_count: 0,
            live_on: false,
            live_feed_written: false,
            inverse_time: header_inverse(post),
            setup,
            turning,
            path_index: 0,
            plane,
            linear: None,
            feed: None,
            line_no: post.line_numbers.start,
            on: Vec::new(),
            thc_phase: ThcPhase::Idle,
            last_speed: String::new(),
            last_power: "0".to_string(),
        }
    }

    /// Diameter or radius mode, the ZX plane, and the feed-mode word. Once, after the header.
    fn begin_lathe(&mut self) -> Result<()> {
        if self.turning.is_none() {
            return Ok(());
        }
        let Some(lathe) = self.post.lathe.clone() else {
            return Ok(());
        };
        self.templates("lathe.mode_lines", &lathe.mode_lines, &[("date-free!", "")])?;
        let plane_line = lathe.plane.clone();
        self.templates("lathe.plane", &[plane_line], &[("date-free!", "")])?;
        if !lathe.feed_mode.is_empty() {
            let feed_mode = lathe.feed_mode.clone();
            self.templates("lathe.feed_mode", &[feed_mode], &[("date-free!", "")])?;
        }
        if let Some(plane) = header_plane(&[lathe.plane]) {
            self.plane = Some(plane);
        }
        Ok(())
    }

    fn lathe_home(&mut self) -> Result<()> {
        if self.turning.is_none() {
            return Ok(());
        }
        let Some(lathe) = self.post.lathe.clone() else {
            return Ok(());
        };
        self.templates("lathe.home", &lathe.home, &[("date-free!", "")])
    }

    fn curve_start(&self) -> Result<Xyz> {
        self.exact.or(self.pos).ok_or_else(|| invalid("arc needs a prior position"))
    }

    fn toolpath(&mut self, index: u32, path: &ClPath) -> Result<()> {
        self.path_index = index;
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
        if !matches!(self.cart, CartMode::Off) && matches!(motion, ClMotion::Thread { .. }) {
            return Err(invalid("thread move during interpolation"));
        }
        if self.interp_fallback() {
            return self.fallback_motion(motion);
        }
        if self.face_active() {
            self.reject_centre_motion(motion)?;
        }
        match motion {
            ClMotion::Rapid { to, feed } => self.linear_move("G0", self.posted_end(*to)?, *feed),
            ClMotion::Linear { to, feed, .. } => {
                self.thc_before_cut()?;
                self.linear_move("G1", self.posted_end(*to)?, Some(*feed))
            }
            ClMotion::Arc { to, plane, center, sense, feed, .. } => {
                self.thc_before_cut()?;
                self.arc(*to, *plane, xyz(*center), *sense, None, *feed)
            }
            ClMotion::Helix { to, plane, center, sense, sweep, feed, .. } => {
                self.thc_before_cut()?;
                self.arc(*to, *plane, xyz(*center), *sense, Some(*sweep), *feed)
            }
            ClMotion::Thread { to, lead } => self.thread_move(*to, *lead),
        }
    }

    /// `<code> X.. Z.. <lead_word><lead>`. The code is never modal. The next G0 or G1 is written
    /// in full. When the lead word is `F`, the modal feed is cleared so the next feed repeats it.
    fn thread_move(&mut self, to: Pose, lead: f64) -> Result<()> {
        self.require_rpm_binding()?;
        let lathe = self.post.lathe.as_ref().ok_or_else(|| unsupported(format!("post `{}` does not support threading", self.post.id())))?;
        let thread =
            lathe.thread.as_ref().ok_or_else(|| unsupported(format!("post `{}` does not support synchronised threading", self.post.id())))?;
        if !(lead.is_finite() && lead > 0.0) {
            return Err(invalid("thread lead must be positive"));
        }
        let posted = self.posted_end(to)?;
        let mut words = vec![thread.code.clone()];
        let rounded = self.push_lathe_axes(&mut words, posted.xyz)?;
        // A thread stays a ZX move, so it carries no C word.
        let lead_text = self.format_axis(lead, self.post.decimals.linear)?;
        words.push(format!("{}{lead_text}", thread.lead_word));
        self.linear = None;
        if thread.lead_word == "F" {
            self.feed = None;
        }
        self.pos = Some(rounded);
        self.exact = Some(posted.xyz);
        self.part_tip = Some(posted.part);
        self.angle = posted.angle;
        self.force_axes = false;
        self.emit_words(&words)
    }

    fn require_rpm_binding(&self) -> Result<()> {
        let speed = self.turning.and_then(|turning| turning.binding_for(self.path_index)).and_then(|binding| binding.speed());
        match speed {
            Some(SpindleSpeed::Rpm { .. }) => Ok(()),
            _ => Err(invalid("threading needs a fixed rpm binding")),
        }
    }

    fn posted_end(&self, to: Pose) -> Result<Posted> {
        let part = tip(to);
        if self.turning.is_some() {
            if !matches!(self.cart, CartMode::Off) {
                self.reject_interp_rotary(to)?;
                return Ok(Posted { xyz: part, part, angle: None, c: None });
            }
            self.guard_c(to.rotary())?;
            return Ok(Posted { xyz: part, part, angle: None, c: to.rotary() });
        }
        let Some(angle) = to.rotary() else {
            return Ok(Posted { xyz: part, part, angle: None, c: None });
        };
        let setup = self.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?;
        Ok(Posted { xyz: tip(part_to_machine(&setup, to, angle)), part, angle: Some(angle), c: None })
    }

    /// A turning pose with a C angle needs a `c_axis` block and an engaged axis.
    ///
    /// The missing block is reported before the not-engaged state.
    fn guard_c(&self, angle: Option<f64>) -> Result<()> {
        if self.turning.is_none() || angle.is_none() {
            return Ok(());
        }
        let has_axis = self.post.lathe.as_ref().is_some_and(|lathe| lathe.c_axis.is_some());
        if !has_axis {
            return Err(unsupported(format!("post `{}` has no C axis", self.post.id())));
        }
        if !self.c_engaged {
            return Err(invalid("C angle needs the C axis engaged"));
        }
        Ok(())
    }

    /// Part-frame arc, or the same arc rotated into a principal machine plane.
    ///
    /// Equal start and end angles stay arcs when the rotated plane is XY, ZX, or YZ and the post
    /// can emit that arc. Any other rotary arc is chorded at [`PostOptions::arc_tolerance`]
    /// before the points are mapped with [`part_to_machine`].
    fn arc(&mut self, to: Pose, plane: Plane, center: Xyz, sense: campanion_ir::ArcSense, sweep: Option<f64>, feed: f64) -> Result<()> {
        if self.turning.is_some() && !self.plane_allowed(plane) {
            return Err(invalid("lathe arc must be in the zx plane"));
        }
        if self.turning.is_some() {
            if matches!(self.cart, CartMode::Off) {
                self.guard_c(to.rotary())?;
            } else {
                self.reject_interp_rotary(to)?;
            }
        }
        // A turning arc outside face and cylinder interpolation stays in the part ZX plane.
        // The C word is not written on that arc.
        if self.turning.is_some() || (self.angle.is_none() && to.rotary().is_none()) {
            let end = tip(to);
            if let Some(sweep) = sweep {
                let start = self.curve_start()?;
                geom::same_radius(start, end, center, plane, 0.0)?;
                if !(sweep.is_finite() && sweep > 0.0) {
                    return Err(invalid("helix sweep must be positive"));
                }
                if sweep > std::f64::consts::TAU * 10_000.0 {
                    return Err(invalid("helix has more than 10000 turns"));
                }
                return self.circular(sense, plane, end, center, sweep, feed, true, None);
            }
            let sweep = geom::arc_sweep(self.curve_start()?, end, center, plane, sense, 0.0)?;
            return self.circular(sense, plane, end, center, sweep, feed, false, None);
        }
        let part_start = self.part_tip.ok_or_else(|| invalid("arc needs a prior position"))?;
        let part_end = tip(to);
        let helical = sweep.is_some();
        let sweep = if let Some(sweep) = sweep {
            geom::same_radius(part_start, part_end, center, plane, 0.0)?;
            if !(sweep.is_finite() && sweep > 0.0) {
                return Err(invalid("helix sweep must be positive"));
            }
            if sweep > std::f64::consts::TAU * 10_000.0 {
                return Err(invalid("helix has more than 10000 turns"));
            }
            sweep
        } else {
            geom::arc_sweep(part_start, part_end, center, plane, sense, 0.0)?
        };
        let (start_angle, end_angle) = match (self.angle, to.rotary()) {
            (Some(start), Some(end)) => (start, end),
            _ => return Err(invalid("rotary arc needs an angle on both endpoints")),
        };
        if angles_equal(start_angle, end_angle)
            && let Some(mapped) =
                rotary::map_arc(&self.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?, end_angle, plane, sense, part_end, center)?
        {
            let start_m = self.exact.ok_or_else(|| invalid("arc needs a prior position"))?;
            let (_, _, start_axial) = geom::plane_uv(mapped.plane, start_m);
            let (_, _, end_axial) = geom::plane_uv(mapped.plane, mapped.end);
            let moves_axial = helical || (end_axial - start_axial).abs() > 1e-9;
            if self.can_emit_arc(mapped.plane, moves_axial) {
                self.leave_inverse()?;
                self.circular(mapped.sense, mapped.plane, mapped.end, mapped.center, sweep, feed, helical, Some(end_angle))?;
                self.part_tip = Some(part_end);
                self.angle = Some(end_angle);
                return Ok(());
            }
        }
        self.linearise_rotary(part_start, part_end, center, plane, sense, sweep, start_angle, end_angle, feed)
    }

    fn can_emit_arc(&self, plane: Plane, helical: bool) -> bool {
        if self.post.arc_mode() == ArcMode::Lines || (helical && !self.post.arcs.helix) {
            return false;
        }
        if !self.post.planes {
            let fixed = self.plane.unwrap_or(Plane::Xy);
            if plane != fixed {
                return false;
            }
        }
        true
    }

    fn linearise_rotary(
        &mut self,
        start: Xyz,
        end: Xyz,
        center: Xyz,
        plane: Plane,
        sense: campanion_ir::ArcSense,
        sweep: f64,
        start_angle: f64,
        end_angle: f64,
        feed: f64,
    ) -> Result<()> {
        let setup = self.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?;
        let samples = rotary::sample_curve(start, end, center, plane, sense, sweep, start_angle, end_angle, self.options.arc_tolerance)?;
        for sample in samples {
            let pose = rotary::pose_at(sample.part)?.with_rotary(sample.angle).map_err(|err| invalid(err.to_string()))?;
            let machine = tip(part_to_machine(&setup, pose, sample.angle));
            self.linear_move("G1", Posted { xyz: machine, part: sample.part, angle: Some(sample.angle), c: None }, Some(feed))?;
        }
        Ok(())
    }

    fn circular(
        &mut self,
        sense: campanion_ir::ArcSense,
        plane: Plane,
        end: Xyz,
        center: Xyz,
        sweep: f64,
        feed: f64,
        helix: bool,
        angle: Option<f64>,
    ) -> Result<()> {
        let start = self.curve_start()?;
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
            self.one_arc(sense, plane, from, tip, center, piece, feed, angle)?;
        }
        if angle.is_none() {
            self.part_tip = self.exact;
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
            self.linear_move("G1", plain(tip), Some(feed))?;
        }
        Ok(())
    }

    fn one_arc(
        &mut self,
        sense: campanion_ir::ArcSense,
        plane: Plane,
        start: Xyz,
        end: Xyz,
        center: Xyz,
        sweep: f64,
        feed: f64,
        angle: Option<f64>,
    ) -> Result<()> {
        let mut words = Vec::new();
        if self.post.planes && self.plane != Some(plane) {
            words.push(plane_word(plane).to_string());
        }
        words.push(sense_word(self.written_sense(sense, plane)).to_string());
        let rounded = self.push_axes(&mut words, end)?;
        self.push_rotary(&mut words, angle)?;
        match self.post.arc_mode() {
            ArcMode::R => {
                let radius = geom::in_plane_len(plane, start, center);
                let chord = geom::in_plane_len(plane, start, rounded);
                // Rounding can make the chord slightly longer than the diameter. Bump R so the
                // block still fits; a semicircle then reads back with its centre on the midpoint.
                let magnitude = if chord > 0.0 { radius.max(chord * 0.5) } else { radius };
                let signed = if sweep > std::f64::consts::PI + 1e-8 { -magnitude } else { magnitude };
                words.push(format!("R{}", self.format_axis(signed, self.post.decimals.linear)?));
            }
            ArcMode::Ijk => {
                let offsets = [center[0] - start[0], center[1] - start[1], center[2] - start[2]];
                for (letter, index) in plane_centre_words(plane) {
                    let Some(value) = offsets.get(*index).copied() else {
                        return Err(invalid("arc offset is missing"));
                    };
                    words.push(format!("{letter}{}", self.format_axis(value, self.post.decimals.linear)?));
                }
            }
            ArcMode::Lines => return Err(invalid("line mode reached an arc block")),
        }
        self.push_feed(&mut words, Some(feed), false)?;
        if self.post.planes {
            self.plane = Some(plane);
        }
        self.linear = None;
        self.pos = Some(rounded);
        self.exact = Some(end);
        self.force_axes = false;
        self.emit_words(&words)
    }

    fn linear_move(&mut self, code: &str, posted: Posted, feed: Option<f64>) -> Result<()> {
        let rotating = rotary::angle_changes(self.angle, posted.angle);
        let c_moves = self.c_text_changes(posted.c)?;
        if self.omit_z_only(code, posted.xyz)? && !rotating && !c_moves {
            self.pos = Some(self.round_xyz(posted.xyz)?);
            self.exact = Some(posted.xyz);
            self.part_tip = Some(posted.part);
            self.angle = posted.angle;
            if let Some(c) = posted.c {
                self.c_continuous = Some(c);
            }
            return Ok(());
        }
        let (feed, force) = self.resolve_feed(code, feed, &posted, rotating)?;
        let mut words = Vec::new();
        if !(self.post.modal_motion && self.linear == Some(code)) {
            words.push(code.to_string());
        }
        let rounded = self.push_axes(&mut words, posted.xyz)?;
        self.push_rotary(&mut words, posted.angle)?;
        self.push_c(&mut words, posted.c)?;
        self.push_feed(&mut words, feed, force)?;
        self.linear = Some(if code == "G0" { "G0" } else { "G1" });
        self.pos = Some(rounded);
        self.exact = Some(posted.xyz);
        self.part_tip = Some(posted.part);
        self.angle = posted.angle;
        if let Some(c) = posted.c {
            self.c_continuous = Some(c);
        }
        self.force_axes = false;
        self.emit_words(&words)
    }

    /// Feed word for this block, and whether inverse time requires `F` on every block.
    ///
    /// A feed that changes the angle uses the post's rotary feed mode. Inverse time sets
    /// `F = feed / length`, where `length` is the straight-line distance between the part-frame
    /// tips (millimetres) and `feed` is the IR feed (millimetres per minute). That is `1 / T`
    /// with `T` in minutes. `degrees_per_minute` and `surface` write the IR feed unchanged.
    /// A feed that keeps the angle stays in millimetres per minute, which means leaving inverse
    /// time before the block. Rapids do not change the feed mode.
    fn resolve_feed(&mut self, code: &str, feed: Option<f64>, posted: &Posted, rotating: bool) -> Result<(Option<f64>, bool)> {
        if code == "G0" {
            return Ok((feed, false));
        }
        let Some(feed) = feed else {
            return Ok((None, false));
        };
        let inverse = self.post.rotary.as_ref().is_some_and(|rotary| rotary.mode() == RotaryFeedMode::InverseTime);
        if rotating && inverse {
            let start = self.part_tip.ok_or_else(|| invalid("inverse time feed needs a prior position"))?;
            let value = rotary::inverse_minutes_feed(feed, geom::dist3(start, posted.part))?;
            self.enter_inverse()?;
            Ok((Some(value), true))
        } else {
            self.leave_inverse()?;
            Ok((Some(feed), false))
        }
    }

    fn enter_inverse(&mut self) -> Result<()> {
        if self.inverse_time {
            return Ok(());
        }
        let code = self
            .post
            .rotary
            .as_ref()
            .and_then(|rotary| rotary.inverse_time_on.clone())
            .ok_or_else(|| invalid("inverse time mode needs inverse_time_on"))?;
        self.emit_line(&code)?;
        self.inverse_time = true;
        self.feed = None;
        Ok(())
    }

    fn leave_inverse(&mut self) -> Result<()> {
        if !self.inverse_time {
            return Ok(());
        }
        let code = self
            .post
            .rotary
            .as_ref()
            .and_then(|rotary| rotary.inverse_time_off.clone())
            .ok_or_else(|| invalid("inverse time mode needs inverse_time_off"))?;
        self.emit_line(&code)?;
        self.inverse_time = false;
        self.feed = None;
        Ok(())
    }

    fn push_rotary(&self, words: &mut Vec<String>, angle: Option<f64>) -> Result<()> {
        let Some(angle) = angle else {
            return Ok(());
        };
        let rotary = self.post.rotary.as_ref().ok_or_else(|| unsupported(format!("post `{}` does not support a rotary axis", self.post.id())))?;
        let letter = match self.setup.map(|setup| setup.axis()) {
            Some(RotaryAxisKind::A) => 'A',
            Some(RotaryAxisKind::B) => 'B',
            None => return Err(invalid("rotary angle needs a rotary setup")),
        };
        let text = self.format_axis(angle, rotary.decimals)?;
        words.push(format!("{letter}{text}"));
        Ok(())
    }

    /// A Z-only rapid is omitted when `rapid_z` is false. With Z words omitted, a Z-only feed is
    /// omitted too: the block would have no axis word.
    fn omit_z_only(&self, code: &str, end: Xyz) -> Result<bool> {
        if self.force_axes {
            return Ok(false);
        }
        let Some(pos) = self.pos else {
            return Ok(false);
        };
        if !xy_text_eq(pos, end, self.post.decimals.linear, self.post.trailing_zeros, self.post.decimal_point)? {
            return Ok(false);
        }
        let drop_rapid = code == "G0" && !self.post.process_feeds.rapid_z;
        Ok(drop_rapid || self.post.drop_z())
    }

    /// `X` `Y` `Z` `I` `J` `K` `R`, rotary `A`/`B`, and lathe `X`/`Z`.
    ///
    /// Feed, spindle, dwell, and sequence numbers stay on [`format_fixed`].
    fn format_axis(&self, value: f64, decimals: u8) -> Result<String> {
        format_dim(value, decimals, self.post.trailing_zeros, self.post.decimal_point)
    }

    fn round_xyz(&self, end: Xyz) -> Result<Xyz> {
        let mut rounded = [0.0; 3];
        for (index, slot) in rounded.iter_mut().enumerate() {
            let Some(value) = end.get(index).copied() else {
                return Err(invalid("axis value is missing"));
            };
            let text = self.format_axis(value, self.post.decimals.linear)?;
            *slot = text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        }
        Ok(rounded)
    }

    fn push_axes(&self, words: &mut Vec<String>, end: Xyz) -> Result<Xyz> {
        if self.native_interp() {
            return self.push_interp_axes(words, end);
        }
        if self.turning.is_some() {
            return self.push_lathe_axes(words, end);
        }
        let letters = self.post.axis_letters.chars().collect::<Vec<_>>();
        let rounded = self.round_xyz(end)?;
        for (index, letter) in letters.iter().enumerate() {
            if self.post.drop_z() && index == 2 {
                continue;
            }
            let Some(value) = rounded.get(index).copied() else {
                return Err(invalid("axis value is missing"));
            };
            let text = self.format_axis(value, self.post.decimals.linear)?;
            words.push(format!("{letter}{text}"));
        }
        Ok(rounded)
    }

    /// X is a diameter when the post asks for one. Y is written only after interpolation has
    /// cleared the modal axes. The stored point stays in the part frame, so the next `I` is a
    /// radius offset from the rounded start.
    fn push_lathe_axes(&self, words: &mut Vec<String>, end: Xyz) -> Result<Xyz> {
        let lathe = self.post.lathe.as_ref().ok_or_else(|| invalid("lathe motion needs a lathe post"))?;
        let letters = self.post.axis_letters.chars().collect::<Vec<_>>();
        let x_letter = letters.first().copied().ok_or_else(|| invalid("axis value is missing"))?;
        let z_letter = letters.get(2).copied().ok_or_else(|| invalid("axis value is missing"))?;
        let x_out = if lathe.diameter_mode { end[0] * 2.0 } else { end[0] };
        let x_text = self.format_axis(x_out, self.post.decimals.linear)?;
        let z_text = self.format_axis(end[2], self.post.decimals.linear)?;
        words.push(format!("{x_letter}{x_text}"));
        let stored_y = if self.force_axes {
            let y_letter = letters.get(1).copied().ok_or_else(|| invalid("axis value is missing"))?;
            let y_text = self.format_axis(end[1], self.post.decimals.linear)?;
            words.push(format!("{y_letter}{y_text}"));
            y_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?
        } else {
            0.0
        };
        words.push(format!("{z_letter}{z_text}"));
        let written_x = x_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        let stored_x = if lathe.diameter_mode { written_x * 0.5 } else { written_x };
        let stored_z = z_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        Ok([stored_x, stored_y, stored_z])
    }

    /// X Y Z in the active interpolation frame. Face X is a diameter only when the block says so.
    /// Cylinder X follows `diameter_mode`. Stored coordinates stay in the IR frame, so `I` `J` `K`
    /// are offsets from that point and are not doubled.
    fn push_interp_axes(&self, words: &mut Vec<String>, end: Xyz) -> Result<Xyz> {
        let letters = self.post.axis_letters.chars().collect::<Vec<_>>();
        let x_letter = letters.first().copied().ok_or_else(|| invalid("axis value is missing"))?;
        let y_letter = letters.get(1).copied().ok_or_else(|| invalid("axis value is missing"))?;
        let z_letter = letters.get(2).copied().ok_or_else(|| invalid("axis value is missing"))?;
        let x_diameter = self.interp_x_diameter();
        let x_out = if x_diameter { end[0] * 2.0 } else { end[0] };
        let x_text = self.format_axis(x_out, self.post.decimals.linear)?;
        let y_text = self.format_axis(end[1], self.post.decimals.linear)?;
        let z_text = self.format_axis(end[2], self.post.decimals.linear)?;
        words.push(format!("{x_letter}{x_text}"));
        words.push(format!("{y_letter}{y_text}"));
        words.push(format!("{z_letter}{z_text}"));
        let written_x = x_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        let stored_x = if x_diameter { written_x * 0.5 } else { written_x };
        let stored_y = y_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        let stored_z = z_text.parse::<f64>().map_err(|_| invalid("axis value is not a number"))?;
        Ok([stored_x, stored_y, stored_z])
    }

    fn interp_x_diameter(&self) -> bool {
        let Some(lathe) = self.post.lathe.as_ref() else {
            return false;
        };
        match self.cart {
            CartMode::Face { native: true } => lathe.face_interp.as_ref().is_some_and(|face| face.x_diameter),
            CartMode::Cylinder { native: true, .. } => lathe.diameter_mode,
            _ => lathe.diameter_mode,
        }
    }

    /// G2/G3 in the IR picture. `arc_flip` swaps them only for the ZX lathe view, not for a face or a cylinder.
    fn written_sense(&self, sense: campanion_ir::ArcSense, plane: Plane) -> campanion_ir::ArcSense {
        let flip = plane == Plane::Zx && self.turning.is_some() && self.post.lathe.as_ref().is_some_and(|lathe| lathe.arc_flip);
        if !flip {
            return sense;
        }
        match sense {
            campanion_ir::ArcSense::Cw => campanion_ir::ArcSense::Ccw,
            campanion_ir::ArcSense::Ccw => campanion_ir::ArcSense::Cw,
        }
    }

    fn push_feed(&mut self, words: &mut Vec<String>, feed: Option<f64>, force: bool) -> Result<()> {
        let Some(feed) = feed else {
            return Ok(());
        };
        if !(feed.is_finite() && feed > 0.0) {
            return Err(invalid("feed must be positive"));
        }
        let text = format_fixed(feed, self.post.decimals.feed, self.post.trailing_zeros)?;
        if !force && self.feed.as_deref() == Some(text.as_str()) {
            return Ok(());
        }
        words.push(format!("F{text}"));
        self.feed = Some(text);
        Ok(())
    }

    fn event(&mut self, event: &Event) -> Result<()> {
        if self.turning.is_none()
            && matches!(
                event,
                Event::CAxis { .. }
                    | Event::SpindleBrake { .. }
                    | Event::LiveToolOn { .. }
                    | Event::LiveToolOff
                    | Event::FaceInterpolation { .. }
                    | Event::CylinderInterpolation { .. }
            )
        {
            return Err(unsupported(format!("post `{}` does not support mill-turn spindle modes", self.post.id())));
        }
        if self.turning.is_some() {
            match event {
                Event::ProcessOn { process: ProcessKind::Spindle, params } => return self.lathe_spindle_on(params),
                Event::ProcessOff { process: ProcessKind::Spindle } => return self.lathe_spindle_off(),
                Event::ToolChange { tool_id } => return self.lathe_tool_change(*tool_id),
                Event::CAxis { engage, .. } => return self.c_axis(*engage),
                Event::SpindleBrake { clamp, .. } => return self.spindle_brake(*clamp),
                Event::LiveToolOn { rpm, direction } => return self.live_tool_on(*rpm, *direction),
                Event::LiveToolOff => return self.live_tool_off(),
                Event::FaceInterpolation { on } => return self.face_interpolation(*on),
                Event::CylinderInterpolation { radius } => return self.cylinder_interpolation(*radius),
                _ => {}
            }
        }
        match event {
            Event::ProcessOn { process, params } => {
                let lines = self.process_words(process)?.on.clone();
                let speed = format_speed(params.speed_value())?;
                let (power, missing) = format_power(self.post, params.power_value())?;
                self.note_on(process, &speed, &power);
                self.arm_thc();
                if missing {
                    self.comment(&format!("power missing, using S{power}"))?;
                }
                self.templates(
                    "process.on",
                    &lines,
                    &[("speed", speed.as_str()), ("power", power.as_str()), ("program_name", ""), ("date-free!", "")],
                )
            }
            Event::ProcessOff { process } => {
                let lines = self.process_words(process)?.off.clone();
                let active = self.on.iter().find(|on| on.key == process_key(process)).cloned();
                let speed = active.as_ref().map(|on| on.speed.clone()).unwrap_or_default();
                let power = active.as_ref().map(|on| on.power.clone()).unwrap_or_else(|| "0".to_string());
                self.thc_before_off()?;
                self.note_off(process);
                self.templates(
                    "process.off",
                    &lines,
                    &[("speed", speed.as_str()), ("power", power.as_str()), ("program_name", ""), ("date-free!", "")],
                )
            }
            Event::Dwell { seconds } => {
                self.note_pierce_dwell();
                self.dwell(*seconds)
            }
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
            Event::CAxis { .. }
            | Event::SpindleBrake { .. }
            | Event::LiveToolOn { .. }
            | Event::LiveToolOff
            | Event::FaceInterpolation { .. }
            | Event::CylinderInterpolation { .. } => Err(unsupported(format!("post `{}` does not support mill-turn spindle modes", self.post.id()))),
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
        self.last_speed = speed.to_string();
        self.last_power = power.to_string();
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

    fn arm_thc(&mut self) {
        if self.post.thc.is_none() {
            return;
        }
        if self.thc_phase == ThcPhase::Idle {
            self.thc_phase = ThcPhase::AwaitDwell;
        }
    }

    fn note_pierce_dwell(&mut self) {
        if self.thc_phase == ThcPhase::AwaitDwell {
            self.thc_phase = ThcPhase::AwaitCut;
        }
    }

    fn thc_before_cut(&mut self) -> Result<()> {
        if self.thc_phase != ThcPhase::AwaitCut {
            return Ok(());
        }
        self.emit_thc(true)?;
        self.thc_phase = ThcPhase::On;
        Ok(())
    }

    fn thc_before_off(&mut self) -> Result<()> {
        if self.thc_phase == ThcPhase::On {
            self.emit_thc(false)?;
        }
        self.thc_phase = ThcPhase::Idle;
        Ok(())
    }

    fn emit_thc(&mut self, on: bool) -> Result<()> {
        let Some(thc) = self.post.thc.clone() else {
            return Ok(());
        };
        let lines = if on { thc.on } else { thc.off };
        let field_name = if on { "thc.on" } else { "thc.off" };
        let speed = self.last_speed.clone();
        let power = self.last_power.clone();
        self.templates(field_name, &lines, &[("speed", speed.as_str()), ("power", power.as_str()), ("program_name", ""), ("date-free!", "")])
    }

    fn lathe_spindle_on(&mut self, params: &ProcessParams) -> Result<()> {
        if self.c_engaged {
            return Err(invalid("turning spindle start while the C axis is engaged"));
        }
        let lathe = self.post.lathe.clone().ok_or_else(|| invalid("lathe spindle needs a lathe post"))?;
        let speed = self.turning.and_then(|turning| turning.binding_for(self.path_index)).and_then(|binding| binding.speed());
        match speed {
            Some(SpindleSpeed::Css { surface_m_per_min, max_rpm }) => {
                let surface = format_lathe_number(surface_m_per_min, self.post)?;
                let max_rpm = format_lathe_number(max_rpm, self.post)?;
                self.templates("lathe.css", &lathe.css, &[("surface", surface.as_str()), ("max_rpm", max_rpm.as_str()), ("date-free!", "")])?;
            }
            Some(SpindleSpeed::Rpm { rpm }) => {
                let rpm = format_lathe_number(rpm, self.post)?;
                self.templates("lathe.rpm", &lathe.rpm, &[("rpm", rpm.as_str()), ("date-free!", "")])?;
            }
            None => {
                let Some(value) = params.speed_value() else {
                    return Err(invalid("lathe spindle speed is missing"));
                };
                let rpm = format_lathe_number(value, self.post)?;
                self.templates("lathe.rpm", &lathe.rpm, &[("rpm", rpm.as_str()), ("date-free!", "")])?;
            }
        }
        let lines = if self.lathe_reversed() { lathe.spindle_ccw } else { lathe.spindle_cw };
        self.templates("lathe.spindle", &lines, &[("date-free!", "")])?;
        self.note_on(&ProcessKind::Spindle, "", "0");
        self.arm_thc();
        Ok(())
    }

    fn lathe_spindle_off(&mut self) -> Result<()> {
        let lines = self.post.lathe.as_ref().map(|lathe| lathe.spindle_off.clone()).unwrap_or_default();
        self.thc_before_off()?;
        self.note_off(&ProcessKind::Spindle);
        self.templates("lathe.spindle_off", &lines, &[("date-free!", "")])
    }

    fn c_axis(&mut self, engage: bool) -> Result<()> {
        let lathe = self.post.lathe.clone().ok_or_else(|| invalid("C axis needs a lathe post"))?;
        let c_axis = lathe.c_axis.ok_or_else(|| unsupported(format!("post `{}` has no C axis", self.post.id())))?;
        if engage {
            self.templates("lathe.c_axis.engage", &c_axis.engage, &[("date-free!", "")])?;
            self.c_engaged = true;
            // Engage unwinds the spindle (Haas writes `G28 H0.`), so the next face angle starts at 0.
            self.c_continuous = Some(0.0);
        } else {
            self.templates("lathe.c_axis.disengage", &c_axis.disengage, &[("date-free!", "")])?;
            self.c_engaged = false;
            self.c_modal = None;
        }
        Ok(())
    }

    fn spindle_brake(&mut self, clamp: bool) -> Result<()> {
        let lathe = self.post.lathe.clone().ok_or_else(|| invalid("spindle brake needs a lathe post"))?;
        let brake = lathe.brake.ok_or_else(|| unsupported(format!("post `{}` has no spindle brake", self.post.id())))?;
        let (field_name, lines) = if clamp { ("lathe.brake.clamp", brake.clamp) } else { ("lathe.brake.release", brake.release) };
        self.templates(field_name, &lines, &[("date-free!", "")])?;
        self.brake_clamped = clamp;
        Ok(())
    }

    fn live_tool_on(&mut self, rpm: f64, direction: LiveDirection) -> Result<()> {
        let lathe = self.post.lathe.clone().ok_or_else(|| invalid("live tool needs a lathe post"))?;
        let live = lathe.live_tool.ok_or_else(|| unsupported(format!("post `{}` has no live tooling", self.post.id())))?;
        let rpm_text = format_live_rpm(rpm)?;
        if !self.live_feed_written {
            if !live.feed_mode.is_empty() && live.feed_mode != lathe.feed_mode {
                let line = live.feed_mode.clone();
                self.templates("lathe.live_tool.feed_mode", &[line], &[("date-free!", "")])?;
            }
            self.live_feed_written = true;
        }
        let (field_name, lines) = match direction {
            LiveDirection::Forward => ("lathe.live_tool.forward", live.forward),
            LiveDirection::Reverse => ("lathe.live_tool.reverse", live.reverse),
        };
        self.templates(field_name, &lines, &[("rpm", rpm_text.as_str()), ("date-free!", "")])?;
        self.live_on = true;
        Ok(())
    }

    fn live_tool_off(&mut self) -> Result<()> {
        let lathe = self.post.lathe.clone().ok_or_else(|| invalid("live tool needs a lathe post"))?;
        let live = lathe.live_tool.ok_or_else(|| unsupported(format!("post `{}` has no live tooling", self.post.id())))?;
        self.templates("lathe.live_tool.stop", &live.stop, &[("date-free!", "")])?;
        self.live_on = false;
        Ok(())
    }

    fn face_active(&self) -> bool {
        matches!(self.cart, CartMode::Face { .. })
    }

    fn native_interp(&self) -> bool {
        matches!(self.cart, CartMode::Face { native: true } | CartMode::Cylinder { native: true, .. })
    }

    fn interp_fallback(&self) -> bool {
        matches!(self.cart, CartMode::Face { native: false } | CartMode::Cylinder { native: false, .. })
    }

    fn plane_allowed(&self, plane: Plane) -> bool {
        match self.cart {
            CartMode::Face { .. } => matches!(plane, Plane::Xy | Plane::Zx),
            CartMode::Cylinder { .. } => matches!(plane, Plane::Yz | Plane::Zx),
            CartMode::Off => plane == Plane::Zx,
        }
    }

    fn reject_interp_rotary(&self, pose: Pose) -> Result<()> {
        if pose.rotary().is_none() {
            return Ok(());
        }
        let message = match self.cart {
            CartMode::Cylinder { .. } => "cylinder interpolation pose must not carry a rotary angle",
            _ => "face interpolation pose must not carry a rotary angle",
        };
        Err(invalid(message))
    }

    fn face_interpolation(&mut self, on: bool) -> Result<()> {
        if !on {
            if matches!(self.cart, CartMode::Face { .. }) {
                self.end_interpolation()?;
            }
            return Ok(());
        }
        if matches!(self.cart, CartMode::Cylinder { .. }) {
            return Err(invalid("face interpolation while cylinder interpolation is on"));
        }
        if self.prepare_interp(true)? {
            let lines = self.face_lines(true);
            self.templates("lathe.face_interp.on", &lines, &[("date-free!", "")])?;
            self.note_plane(&lines);
            self.cart = CartMode::Face { native: true };
        } else {
            self.cart = CartMode::Face { native: false };
        }
        Ok(())
    }

    fn cylinder_interpolation(&mut self, radius: Option<f64>) -> Result<()> {
        let Some(radius) = radius else {
            if matches!(self.cart, CartMode::Cylinder { .. }) {
                self.end_interpolation()?;
            }
            return Ok(());
        };
        if !(radius.is_finite() && radius > 0.0) {
            return Err(invalid("cylinder interpolation radius must be positive"));
        }
        if matches!(self.cart, CartMode::Face { .. }) {
            return Err(invalid("cylinder interpolation while face interpolation is on"));
        }
        if self.prepare_interp(false)? {
            let lines = self.cylinder_lines(true);
            let radius_text = self.format_axis(radius, self.post.decimals.linear)?;
            let diameter_text = self.format_axis(radius * 2.0, self.post.decimals.linear)?;
            self.templates(
                "lathe.cylinder_interp.on",
                &lines,
                &[("radius", radius_text.as_str()), ("diameter", diameter_text.as_str()), ("date-free!", "")],
            )?;
            self.note_plane(&lines);
            self.cart = CartMode::Cylinder { radius, native: true };
        } else {
            self.cart = CartMode::Cylinder { radius, native: false };
        }
        Ok(())
    }

    /// `true` when the post has a native block. A missing block with no C axis is reported before
    /// the not-engaged error.
    fn prepare_interp(&self, face: bool) -> Result<bool> {
        let lathe = self.post.lathe.as_ref().ok_or_else(|| invalid("interpolation needs a lathe post"))?;
        let native = if face { lathe.face_interp.is_some() } else { lathe.cylinder_interp.is_some() };
        if !native && lathe.c_axis.is_none() {
            return Err(unsupported(format!("post `{}` has no C axis", self.post.id())));
        }
        if !self.c_engaged {
            let message = if face { "face interpolation needs the C axis engaged" } else { "cylinder interpolation needs the C axis engaged" };
            return Err(invalid(message));
        }
        if self.brake_clamped {
            return Err(invalid("interpolation with the spindle brake clamped; release it first"));
        }
        Ok(native)
    }

    fn face_lines(&self, on: bool) -> Vec<String> {
        self.post
            .lathe
            .as_ref()
            .and_then(|lathe| lathe.face_interp.as_ref())
            .map(|face| if on { face.on.clone() } else { face.off.clone() })
            .unwrap_or_default()
    }

    fn cylinder_lines(&self, on: bool) -> Vec<String> {
        self.post
            .lathe
            .as_ref()
            .and_then(|lathe| lathe.cylinder_interp.as_ref())
            .map(|cylinder| if on { cylinder.on.clone() } else { cylinder.off.clone() })
            .unwrap_or_default()
    }

    fn end_interpolation(&mut self) -> Result<()> {
        match self.cart {
            CartMode::Off => return Ok(()),
            CartMode::Face { native: true } => {
                let lines = self.face_lines(false);
                self.templates("lathe.face_interp.off", &lines, &[("date-free!", "")])?;
                self.note_plane(&lines);
            }
            CartMode::Cylinder { native: true, .. } => {
                let lines = self.cylinder_lines(false);
                self.templates("lathe.cylinder_interp.off", &lines, &[("date-free!", "")])?;
                self.note_plane(&lines);
            }
            CartMode::Face { native: false } | CartMode::Cylinder { native: false, .. } => {}
        }
        self.leave_interp_state();
        Ok(())
    }

    fn leave_interp_state(&mut self) {
        if let Some(tip) = self.part_tip
            && let Ok(rounded) = self.round_xyz(tip)
        {
            self.pos = Some(rounded);
            self.exact = Some(tip);
        }
        self.cart = CartMode::Off;
        self.force_axes = true;
        self.c_modal = None;
        self.linear = None;
    }

    fn note_plane(&mut self, lines: &[String]) {
        if let Some(plane) = header_plane(lines) {
            self.plane = Some(plane);
        }
    }

    fn reject_centre_motion(&self, motion: &ClMotion) -> Result<()> {
        if !self.face_active() {
            return Ok(());
        }
        let end = tip(motion_pose(motion));
        let clearance = match motion {
            ClMotion::Rapid { .. } | ClMotion::Linear { .. } | ClMotion::Thread { .. } => match self.part_tip {
                Some(start) => interp::xy_segment_min(start, end),
                None => end[0].hypot(end[1]),
            },
            ClMotion::Arc { plane, center, sense, .. } => {
                let start = self.part_tip.ok_or_else(|| invalid("arc needs a prior position"))?;
                let center = xyz(*center);
                let sweep = geom::arc_sweep(start, end, center, *plane, *sense, 0.0)?;
                interp::xy_arc_min(start, end, center, *plane, *sense, sweep)
            }
            ClMotion::Helix { plane, center, sense, sweep, .. } => {
                let start = self.part_tip.ok_or_else(|| invalid("arc needs a prior position"))?;
                interp::xy_arc_min(start, end, xyz(*center), *plane, *sense, *sweep)
            }
        };
        interp::reject_centre(clearance)
    }

    fn fallback_motion(&mut self, motion: &ClMotion) -> Result<()> {
        let frame = match self.cart {
            CartMode::Face { native: false } => PolarFrame::Face,
            CartMode::Cylinder { radius, native: false } => PolarFrame::Cylinder { radius },
            _ => return Err(invalid("interpolation fallback is not active")),
        };
        match motion {
            ClMotion::Rapid { to, feed } => {
                self.reject_interp_rotary(*to)?;
                let end = tip(*to);
                self.reject_centre_span(self.part_tip, end)?;
                self.emit_fallback(frame, "G0", line_curve(self.part_tip, end), *feed)
            }
            ClMotion::Linear { to, feed, .. } => {
                self.thc_before_cut()?;
                self.reject_interp_rotary(*to)?;
                let end = tip(*to);
                self.reject_centre_span(self.part_tip, end)?;
                self.emit_fallback(frame, "G1", line_curve(self.part_tip, end), Some(*feed))
            }
            ClMotion::Arc { to, plane, center, sense, feed, .. } => {
                self.thc_before_cut()?;
                self.fallback_arc(frame, *to, *plane, xyz(*center), *sense, None, *feed)
            }
            ClMotion::Helix { to, plane, center, sense, sweep, feed, .. } => {
                self.thc_before_cut()?;
                self.fallback_arc(frame, *to, *plane, xyz(*center), *sense, Some(*sweep), *feed)
            }
            ClMotion::Thread { .. } => Err(invalid("thread move during interpolation")),
        }
    }

    fn fallback_arc(
        &mut self,
        frame: PolarFrame,
        to: Pose,
        plane: Plane,
        center: Xyz,
        sense: campanion_ir::ArcSense,
        sweep: Option<f64>,
        feed: f64,
    ) -> Result<()> {
        self.reject_interp_rotary(to)?;
        if !self.plane_allowed(plane) {
            return Err(invalid("lathe arc must be in the zx plane"));
        }
        let end = tip(to);
        let start = self.part_tip.ok_or_else(|| invalid("arc needs a prior position"))?;
        let sweep = if let Some(sweep) = sweep {
            geom::same_radius(start, end, center, plane, 0.0)?;
            if !(sweep.is_finite() && sweep > 0.0) {
                return Err(invalid("helix sweep must be positive"));
            }
            if sweep > std::f64::consts::TAU * 10_000.0 {
                return Err(invalid("helix has more than 10000 turns"));
            }
            sweep
        } else {
            geom::arc_sweep(start, end, center, plane, sense, 0.0)?
        };
        if self.face_active() {
            interp::reject_centre(interp::xy_arc_min(start, end, center, plane, sense, sweep))?;
        }
        self.emit_fallback(frame, "G1", PolarCurve::Arc { start, end, center, plane, sense, sweep }, Some(feed))
    }

    fn reject_centre_span(&self, start: Option<Xyz>, end: Xyz) -> Result<()> {
        if !self.face_active() {
            return Ok(());
        }
        let clearance = match start {
            Some(start) => interp::xy_segment_min(start, end),
            None => end[0].hypot(end[1]),
        };
        interp::reject_centre(clearance)
    }

    fn emit_fallback(&mut self, frame: PolarFrame, code: &str, curve: PolarCurve, feed: Option<f64>) -> Result<()> {
        self.polar_count = 0;
        let prev = self.c_continuous.unwrap_or(0.0);
        if matches!(curve, PolarCurve::Point { .. }) {
            let end = curve.at(1.0);
            let polar = interp::to_polar(frame, end, prev)?;
            return self.emit_polar(code, polar, end, feed);
        }
        self.polar_count = 0;
        self.emit_span(frame, curve, 0.0, 1.0, prev, code, feed, 0)?;
        Ok(())
    }

    fn emit_span(
        &mut self,
        frame: PolarFrame,
        curve: PolarCurve,
        t0: f64,
        t1: f64,
        prev_c: f64,
        code: &str,
        feed: Option<f64>,
        depth: u32,
    ) -> Result<f64> {
        let start = curve.at(t0);
        let end = curve.at(t1);
        let fits = self.span_fits(frame, start, end, |t| curve.at(t0 + (t1 - t0) * t), prev_c)?;
        if fits || geom::dist3(start, end) <= 1e-9 {
            let polar = interp::to_polar(frame, end, prev_c)?;
            self.emit_polar(code, polar, end, feed)?;
            return Ok(polar.c_deg);
        }
        if !(self.options.arc_tolerance.is_finite() && self.options.arc_tolerance > 0.0) {
            return Err(invalid("arc_tolerance must be positive to linearise an arc"));
        }
        if depth >= 24 || self.polar_count > 100_000 {
            return Err(invalid("arc linearises to more than 100000 segments"));
        }
        let mid = (t0 + t1) * 0.5;
        let mid_c = self.emit_span(frame, curve, t0, mid, prev_c, code, feed, depth + 1)?;
        self.emit_span(frame, curve, mid, t1, mid_c, code, feed, depth + 1)
    }

    /// True when linear interpolation in the polar frame stays within `arc_tolerance` of the real curve.
    fn span_fits<F>(&self, frame: PolarFrame, start: Xyz, end: Xyz, at: F, prev_c: f64) -> Result<bool>
    where
        F: Fn(f64) -> Xyz,
    {
        let tol = self.options.arc_tolerance.max(0.0);
        let p0 = interp::to_polar(frame, start, prev_c)?;
        let p1 = interp::to_polar(frame, end, p0.c_deg)?;
        for t in [0.25_f64, 0.5, 0.75] {
            let radius = p0.radius + (p1.radius - p0.radius) * t;
            let c_deg = p0.c_deg + (p1.c_deg - p0.c_deg) * t;
            let z = p0.z + (p1.z - p0.z) * t;
            if geom::dist3(interp::polar_to_cart(frame, radius, c_deg, z), at(t)) > tol {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn emit_polar(&mut self, code: &str, polar: PolarTarget, cart: Xyz, feed: Option<f64>) -> Result<()> {
        self.polar_count = self.polar_count.saturating_add(1);
        if self.polar_count > 100_000 {
            return Err(invalid("arc linearises to more than 100000 segments"));
        }
        let posted = Posted { xyz: [polar.radius, 0.0, polar.z], part: cart, angle: None, c: Some(polar.c_deg) };
        self.linear_move(code, posted, feed)?;
        // `linear_move` stores the written X radius. The part tip stays Cartesian so the next
        // chord is sampled from the original curve.
        self.exact = Some(cart);
        self.part_tip = Some(cart);
        self.c_continuous = Some(polar.c_deg);
        Ok(())
    }

    /// Cancel interpolation, stop a live tool that is still running, release a clamped brake,
    /// then disengage a C axis that is still engaged.
    ///
    /// Those lines are written after the footer and before [`Self::lathe_home`]. Interpolation
    /// ends first so G112/G107 cancel while the C axis is still engaged.
    fn finish_mill_turn(&mut self) -> Result<()> {
        self.end_interpolation()?;
        if self.live_on {
            self.live_tool_off()?;
        }
        if self.brake_clamped {
            self.spindle_brake(false)?;
        }
        if self.c_engaged {
            self.c_axis(false)?;
        }
        Ok(())
    }

    /// True when this block would write a C word that is not already modal.
    fn c_text_changes(&self, angle: Option<f64>) -> Result<bool> {
        if self.turning.is_none() || !self.c_engaged {
            return Ok(false);
        }
        let Some(angle) = angle else {
            return Ok(false);
        };
        let text = self.format_axis(angle, self.post.decimals.linear)?;
        Ok(self.c_modal.as_deref() != Some(text.as_str()))
    }

    /// `<letter><angle>` after Z. Modal on the formatted linear-decimal text.
    fn push_c(&mut self, words: &mut Vec<String>, angle: Option<f64>) -> Result<()> {
        if !self.c_text_changes(angle)? {
            return Ok(());
        }
        let Some(angle) = angle else {
            return Ok(());
        };
        if self.brake_clamped && self.c_modal.is_some() {
            return Err(invalid("C move with the spindle brake clamped; release it first"));
        }
        let letter = self
            .post
            .lathe
            .as_ref()
            .and_then(|lathe| lathe.c_axis.as_ref())
            .map(|c_axis| c_axis.letter.clone())
            .ok_or_else(|| unsupported(format!("post `{}` has no C axis", self.post.id())))?;
        let text = self.format_axis(angle, self.post.decimals.linear)?;
        words.push(format!("{letter}{text}"));
        self.c_modal = Some(text);
        self.c_continuous = Some(angle);
        Ok(())
    }

    fn lathe_tool_change(&mut self, tool_id: u32) -> Result<()> {
        let lines = self.post.lathe.as_ref().map(|lathe| lathe.tool_change.clone()).unwrap_or_default();
        let tool = tool_id.to_string();
        let tool2 = format!("{tool_id:02}");
        self.templates(
            "lathe.tool_change",
            &lines,
            &[("tool", tool.as_str()), ("tool2", tool2.as_str()), ("offset2", tool2.as_str()), ("date-free!", "")],
        )
    }

    fn lathe_reversed(&self) -> bool {
        let Some(turning) = self.turning else {
            return false;
        };
        let Some(binding) = turning.binding_for(self.path_index) else {
            return false;
        };
        turning.spindle(binding.spindle()).is_some_and(|spindle| spindle.reversed())
    }

    fn stop_processes(&mut self) -> Result<()> {
        self.thc_before_off()?;
        let on = std::mem::take(&mut self.on);
        for active in on {
            if self.turning.is_some() && active.key == "spindle" {
                let lines = self.post.lathe.as_ref().map(|lathe| lathe.spindle_off.clone()).unwrap_or_default();
                self.templates("lathe.spindle_off", &lines, &[("date-free!", "")])?;
                continue;
            }
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
            let prepared = omit_absent_speed(line, vars);
            let text = substitute(field_name, &prepared, vars)?;
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
        if self.post.line_numbers.enabled && !unnumbered_tape_line(body) {
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

/// `{power}` text, and whether a missing IR power was filled in at full scale.
///
/// With `power_scale`, the text is the integer `ir_power * power_scale`. A missing power is
/// treated as `1`, so scale `1000` writes `1000`. Without a scale the raw power is written with
/// the feed decimals, and a missing power is `0`.
fn format_power(post: &PostDef, power: Option<f64>) -> Result<(String, bool)> {
    let Some(scale) = post.power_scale else {
        let text = format_fixed(format_param(power)?, post.decimals.feed, post.trailing_zeros)?;
        return Ok((text, false));
    };
    let (value, missing) = match power {
        Some(value) => (value, false),
        None => (1.0, true),
    };
    if !(value.is_finite() && value >= 0.0) {
        return Err(invalid("process speed or power must be >= 0"));
    }
    let text = format_fixed(value * scale, 0, false)?;
    Ok((text, missing))
}

/// Live-tool `{rpm}`: rounded half away from zero, written with no decimal point.
fn format_live_rpm(rpm: f64) -> Result<String> {
    if !(rpm.is_finite() && rpm > 0.0 && rpm <= 100_000.0) {
        return Err(invalid("live tool rpm must be positive and at most 100000"));
    }
    let whole = rpm.round();
    if !(whole.is_finite() && (0.0..=100_000.0).contains(&whole)) {
        return Err(invalid("live tool rpm must be positive and at most 100000"));
    }
    Ok(format!("{}", whole as u64))
}

/// `{surface}`, `{max_rpm}` and `{rpm}`. A whole number is an integer. Anything else uses the feed decimals.
fn format_lathe_number(value: f64, post: &PostDef) -> Result<String> {
    if !(value.is_finite() && value >= 0.0) {
        return Err(invalid("spindle speed must be >= 0"));
    }
    let nearest = value.round();
    if (value - nearest).abs() <= 1e-6 { format_fixed(nearest, 0, false) } else { format_fixed(value, post.decimals.feed, post.trailing_zeros) }
}

/// Spindle speed is a whole number of revolutions (`S12000`). An absent speed is an empty string
/// so [`omit_absent_speed`] can drop the `S` word instead of writing `S0`.
fn format_speed(value: Option<f64>) -> Result<String> {
    let Some(value) = value else {
        return Ok(String::new());
    };
    if !(value.is_finite() && value >= 0.0) {
        return Err(invalid("process speed or power must be >= 0"));
    }
    format_fixed(value, 0, false)
}

/// `M3 S{speed}` becomes `M3` when the process has no speed. A present speed, including zero, stays.
fn omit_absent_speed(line: &str, vars: &[(&str, &str)]) -> String {
    let absent = vars.iter().any(|(key, value)| *key == "speed" && value.is_empty());
    if !absent {
        return line.to_string();
    }
    let mut out = String::new();
    for word in line.split_whitespace() {
        if word == "S{speed}" {
            continue;
        }
        if !out.is_empty() {
            out.push(' ');
        }
        out.push_str(word);
    }
    out
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

fn line_curve(start: Option<Xyz>, end: Xyz) -> PolarCurve {
    match start {
        Some(start) => PolarCurve::Line { start, end },
        None => PolarCurve::Point { end },
    }
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

/// In-plane centre offsets only: `I J` on G17, `I K` on G18, `J K` on G19.
/// The helix still carries its axial endpoint (`Z` on G17) from the move itself.
fn plane_centre_words(plane: Plane) -> &'static [(&'static str, usize)] {
    match plane {
        Plane::Xy => &[("I", 0), ("J", 1)],
        Plane::Zx => &[("I", 0), ("K", 2)],
        Plane::Yz => &[("J", 1), ("K", 2)],
    }
}

/// `%` and `O0001` are tape framing, not sequence-numbered blocks.
fn unnumbered_tape_line(body: &str) -> bool {
    let body = body.trim();
    if body == "%" {
        return true;
    }
    let Some(digits) = body.strip_prefix('O') else {
        return false;
    };
    !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit())
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

/// Mill posts, and lathe posts with no `thread` block, reject a program that contains a thread move.
fn check_thread_support(program: &Program, post: &PostDef) -> Result<()> {
    let has_thread =
        program.toolpaths().iter().any(|path| path.blocks().iter().any(|block| matches!(block, ClBlock::Motion(ClMotion::Thread { .. }))));
    if !has_thread {
        return Ok(());
    }
    match post.lathe.as_ref() {
        None => Err(unsupported(format!("post `{}` does not support threading", post.id()))),
        Some(lathe) if lathe.thread.is_none() => Err(unsupported(format!("post `{}` does not support synchronised threading", post.id()))),
        Some(_) => Ok(()),
    }
}

/// A turning program needs a lathe post, and a lathe post needs a turning program.
///
/// Lathe poses are part-frame XZ: `|y|` above `1e-6` is rejected outside face and cylinder
/// interpolation, and a rotary setup is rejected. XY arcs are allowed only in face mode. YZ arcs
/// are allowed only in cylinder mode.
fn check_lathe_program(program: &Program, post: &PostDef) -> Result<()> {
    match (program.turning().is_some(), post.is_lathe()) {
        (true, false) => return Err(unsupported(format!("post `{}` is not a lathe post", post.id()))),
        (false, true) => return Err(unsupported(format!("lathe post `{}` needs a turning program", post.id()))),
        _ => {}
    }
    if program.turning().is_none() {
        return Ok(());
    }
    if program.rotary().is_some() {
        return Err(invalid("lathe program cannot have a rotary setup"));
    }
    for (path_i, path) in program.toolpaths().iter().enumerate() {
        let mut face = false;
        let mut cylinder = false;
        for (block_i, block) in path.blocks().iter().enumerate() {
            match block {
                ClBlock::Event(Event::FaceInterpolation { on }) => {
                    face = *on;
                    if *on {
                        cylinder = false;
                    }
                }
                ClBlock::Event(Event::CylinderInterpolation { radius }) => {
                    cylinder = radius.is_some();
                    if cylinder {
                        face = false;
                    }
                }
                _ => {}
            }
            if let Some(pose) = pose_of_block(block) {
                let y = pose.tip().y();
                if !face && !cylinder && (!y.is_finite() || y.abs() > 1e-6) {
                    return Err(invalid(format!("toolpath {path_i} block {block_i} lathe pose y must be 0")));
                }
            }
            if let ClBlock::Motion(ClMotion::Arc { plane, .. } | ClMotion::Helix { plane, .. }) = block {
                let allowed = if face {
                    matches!(plane, Plane::Xy | Plane::Zx)
                } else if cylinder {
                    matches!(plane, Plane::Yz | Plane::Zx)
                } else {
                    *plane == Plane::Zx
                };
                if !allowed {
                    return Err(invalid("lathe arc must be in the zx plane"));
                }
            }
        }
    }
    Ok(())
}

fn pose_of_block(block: &ClBlock) -> Option<Pose> {
    match block {
        ClBlock::Motion(motion) => Some(motion_pose(motion)),
        ClBlock::Event(Event::Probe { toward, .. }) => *toward,
        ClBlock::Event(_) => None,
    }
}

fn motion_pose(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    }
}

fn check_rotary_program(program: &Program, post: &PostDef) -> Result<()> {
    // Turning C is a lathe angle, not a mill A/B axis.
    if program.turning().is_some() {
        return Ok(());
    }
    if !poses_use_rotary(program) {
        return Ok(());
    }
    let Some(rotary) = &post.rotary else {
        return Err(unsupported(format!("post `{}` does not support a rotary axis", post.id())));
    };
    let Some(setup) = program.rotary() else {
        return Err(invalid("rotary angle needs a rotary setup"));
    };
    if rotary.supports(setup.axis()) {
        return Ok(());
    }
    let name = match setup.axis() {
        RotaryAxisKind::A => "a",
        RotaryAxisKind::B => "b",
    };
    Err(unsupported(format!("post `{}` does not support rotary axis `{name}`", post.id())))
}

/// True when the header selects inverse time and does not later cancel it.
fn header_inverse(post: &PostDef) -> bool {
    let Some(rotary) = &post.rotary else {
        return false;
    };
    let on = rotary.inverse_time_on.as_deref();
    let off = rotary.inverse_time_off.as_deref();
    let mut enabled = false;
    for line in &post.header {
        for word in line.split_whitespace() {
            let word = word.to_ascii_uppercase();
            if Some(word.as_str()) == on {
                enabled = true;
            }
            if Some(word.as_str()) == off {
                enabled = false;
            }
        }
    }
    enabled
}

/// True when any cutter-location pose, including a probe target, carries a rotary angle.
fn poses_use_rotary(program: &Program) -> bool {
    program.toolpaths().iter().any(|path| path.blocks().iter().any(block_uses_rotary))
}

fn block_uses_rotary(block: &ClBlock) -> bool {
    match block {
        ClBlock::Motion(motion) => motion_uses_rotary(motion),
        ClBlock::Event(Event::Probe { toward: Some(pose), .. }) => pose.rotary().is_some(),
        ClBlock::Event(_) => false,
    }
}

fn motion_uses_rotary(motion: &ClMotion) -> bool {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => to.rotary().is_some(),
    }
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
