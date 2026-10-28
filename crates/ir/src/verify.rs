//! Report motion and process problems in a [`Program`].
//!
//! [`verify`] never rejects the program and never panics. It collects typed issues and a simple
//! length, time and bounds estimate. Tool-space length is the tool-tip path in millimetres,
//! including arcs and helices. Axis-space length is the chord of linear joint travel. Feeds are
//! millimetres per minute, so a move's time is `length / feed * 60` seconds. Dwells add their
//! own seconds. A rapid with no feed counts toward length and not toward time.
//!
//! A rapid that changes X or Y while Z is below the safe height is an error, listed by
//! [`VerifyReport::rapid_safe_errors`]. A rapid that ends below the retract height is a warning,
//! listed by [`VerifyReport::rapid_retract_warnings`]. [`VerifyOptions`] supplies those heights.
//! When safe height is unset, it is the highest Z any rapid reaches. On a 2.5D mill program (no
//! turning, rotary table, or five-axis setup) that value is at least the stock top Z 0, so an XY
//! rapid through the stock cannot hide as "every rapid was already low". When retract height is
//! unset, the warning is skipped.

use crate::MAX_AXES;
use crate::check::{self, CONTINUITY_MM, MIN_COMPONENT};
use crate::geom::{ArcSense, Plane, Pose, Vec3};
use crate::machine::{AxisKind, AxisLetter, AxisName, Channel, Machine};
use crate::motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath};
use crate::process::{Event, ProcessKind};
use crate::program::Program;

const TAU: f64 = std::f64::consts::TAU;
/// Stock top for 2.5D mill programs. Z is up.
const STOCK_TOP_Z: f64 = 0.0;

/// Heights used by the rapid checks. Millimetres. Z is up.
///
/// `verify` uses [`VerifyOptions::default`]: safe height is inferred and the retract warning is off.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct VerifyOptions {
    /// Clearance height. `None` uses the highest Z any rapid reaches.
    pub safe_z: Option<f64>,
    /// Retract height. `None` skips the retract warning.
    pub retract_z: Option<f64>,
}

/// Safe and retract heights after a non-finite caller value has been dropped.
#[derive(Clone, Copy)]
struct Heights {
    safe_z: Option<f64>,
    retract_z: Option<f64>,
}

/// Inclusive axis-aligned bounds of the tool tips and linear joint positions that were finite.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    min: Vec3,
    max: Vec3,
}

impl Bounds {
    /// Minimum corner, millimetres.
    pub fn min(self) -> Vec3 {
        self.min
    }

    /// Maximum corner, millimetres.
    pub fn max(self) -> Vec3 {
        self.max
    }
}

/// An axis or channel id that the machine does not define.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RefKind {
    /// Joint id.
    Axis(u8),
    /// Channel id.
    Channel(u16),
}

/// One problem found while verifying a program.
#[derive(Clone, Debug, PartialEq)]
pub enum Issue {
    /// A joint target is outside that axis's inclusive limits.
    AxisLimit {
        /// Channel that commanded the target, when the channel is known.
        channel: Option<u16>,
        /// Axis id.
        axis: u8,
        /// Commanded position. Millimetres or radians, matching the axis.
        value: f64,
        /// Inclusive minimum.
        min: f64,
        /// Inclusive maximum.
        max: f64,
    },
    /// An arc or helix does not leave from the previous end within the tolerance.
    ///
    /// The gap is the radial mismatch, in millimetres, between the previous tip and the circle
    /// through the commanded end. For a helix whose radius does match, the gap is the in-plane
    /// distance from the commanded end to the point the sweep actually reaches.
    Discontinuity {
        /// Index into [`Program::toolpaths`].
        toolpath: usize,
        /// Index into that path's blocks.
        block: usize,
        /// Absolute gap, millimetres.
        gap: f64,
        /// Gaps above this value are reported.
        tolerance: f64,
    },
    /// [`Event::ProcessOn`] has no later [`Event::ProcessOff`] for the same process.
    /// A mill spindle may stay open across toolpaths and is reported only at the end of the program
    /// (`toolpath` is [`None`]). Any other process must close on the toolpath that opened it.
    /// A spindle also closes when [`Event::SpindleRun`] stops the bound spindle.
    ProcessNotClosed {
        /// Toolpath index, when the process was left open on a toolpath.
        toolpath: Option<usize>,
        /// Channel id, when the process was left open on a channel program.
        channel: Option<u16>,
        /// Process that stayed on.
        process: ProcessKind,
    },
    /// A stored number or a derived length is not finite, so it was left out of the totals.
    NonFinite {
        /// What was read.
        what: String,
    },
    /// A linear, arc or helix move has a feed that is not strictly positive.
    BadFeed {
        /// Toolpath index, when the move is tool-space.
        toolpath: Option<usize>,
        /// Channel id, when the move is axis-space.
        channel: Option<u16>,
        /// Index into that path or channel program.
        block: usize,
        /// The feed that was rejected, millimetres per minute.
        feed: f64,
    },
    /// The program names an axis or channel the machine does not have.
    UnknownRef(RefKind),
}

/// Lengths, a time estimate, bounds and every issue [`verify`] found.
///
/// Rapid-height results sit beside [`Issue`] so existing matches on that enum stay valid.
/// [`Self::rapid_safe_errors`] are errors. [`Self::rapid_retract_warnings`] are warnings.
#[derive(Clone, Debug, PartialEq)]
pub struct VerifyReport {
    issues: Vec<Issue>,
    total_length: f64,
    rapid_length: f64,
    time_s: f64,
    bounds: Option<Bounds>,
    /// `(toolpath, channel, block, lowest Z, safe Z)` for each unsafe XY rapid.
    rapid_safe_errors: Vec<(Option<usize>, Option<u16>, usize, f64, f64)>,
    /// `(toolpath, channel, block, end Z, retract Z)` for each low rapid.
    rapid_retract_warnings: Vec<(Option<usize>, Option<u16>, usize, f64, f64)>,
}

impl VerifyReport {
    /// Issues, in toolpath order and then channel-program order.
    pub fn issues(&self) -> &[Issue] {
        &self.issues
    }

    /// Tool-tip path length plus linear joint travel, millimetres. Rapids are included.
    pub fn total_length(&self) -> f64 {
        self.total_length
    }

    /// Rapid travel included in [`Self::total_length`], millimetres.
    pub fn rapid_length(&self) -> f64 {
        self.rapid_length
    }

    /// Seconds. Feed moves use `length / feed * 60`. Dwells are added. Rapids without a feed are free.
    pub fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Bounds of every finite position the estimate visited.
    pub fn bounds(&self) -> Option<Bounds> {
        self.bounds
    }

    /// Rapids that change X or Y while Z is below the safe height. These are errors.
    ///
    /// Each entry is `(toolpath, channel, block, lowest_z, safe_z)`, in millimetres for the Z values.
    /// `toolpath` is set for cutter-location motion. `channel` is set for axis-space motion.
    pub fn rapid_safe_errors(&self) -> &[(Option<usize>, Option<u16>, usize, f64, f64)] {
        &self.rapid_safe_errors
    }

    /// Rapids that end below the retract height. These are warnings.
    ///
    /// Each entry is `(toolpath, channel, block, end_z, retract_z)`, in millimetres for the Z values.
    /// Empty when [`VerifyOptions::retract_z`] was `None` or not finite.
    pub fn rapid_retract_warnings(&self) -> &[(Option<usize>, Option<u16>, usize, f64, f64)] {
        &self.rapid_retract_warnings
    }
}

/// Walk `program` with [`VerifyOptions::default`].
///
/// Safe height is the highest Z any rapid reaches, and at least the stock top Z 0 on a 2.5D mill
/// program. The retract warning is off.
///
/// Structural checks in [`Program::try_new`](crate::Program::try_new) already reject some of
/// these problems. This still reports them, because a deserialized program can bypass those
/// constructors. Totals stay finite: a non-finite sample is reported and skipped.
pub fn verify(program: &Program) -> VerifyReport {
    verify_with(program, &VerifyOptions::default())
}

/// Walk `program` using `options` for the safe and retract heights.
pub fn verify_with(program: &Program, options: &VerifyOptions) -> VerifyReport {
    let mut out = VerifyReport {
        issues: Vec::new(),
        total_length: 0.0,
        rapid_length: 0.0,
        time_s: 0.0,
        bounds: None,
        rapid_safe_errors: Vec::new(),
        rapid_retract_warnings: Vec::new(),
    };
    let heights = resolve_heights(program, options, &mut out);
    // The tip carries across toolpaths in program order. The program's first motion still has no start.
    // A mill spindle does too: depth passes of one tool share one process-on. Turning still closes
    // the spindle on the toolpath that opened it.
    let mut cursor = None;
    let mut carry = SpindleCarry { span: program.turning().is_none(), open: Vec::new() };
    for (index, path) in program.toolpaths().iter().enumerate() {
        let bound = program.turning().and_then(|turning| turning.binding_for(index as u32)).map(|binding| binding.spindle());
        verify_toolpath(index, path, bound, heights, &mut cursor, &mut out, &mut carry);
    }
    if carry.span {
        close_processes(&carry.open, None, None, &mut out);
    }
    for channel_program in program.channel_programs() {
        verify_channel(program.machine(), channel_program, heights, &mut out);
    }
    out
}

fn resolve_heights(program: &Program, options: &VerifyOptions, out: &mut VerifyReport) -> Heights {
    let safe_z = match options.safe_z {
        Some(z) if z.is_finite() => Some(z),
        Some(_) => {
            out.issues.push(Issue::NonFinite { what: "safe height".into() });
            None
        }
        None => infer_safe_z(program),
    };
    let retract_z = match options.retract_z {
        Some(z) if z.is_finite() => Some(z),
        Some(_) => {
            out.issues.push(Issue::NonFinite { what: "retract height".into() });
            None
        }
        None => None,
    };
    Heights { safe_z: floor_mill_safe_z(program, safe_z), retract_z }
}

/// 2.5D mill stock is Z 0. Floor the safe height so an XY rapid below the stock is always an error.
fn floor_mill_safe_z(program: &Program, safe_z: Option<f64>) -> Option<f64> {
    if program.turning().is_some() || program.rotary().is_some() || program.five_axis().is_some() {
        return safe_z;
    }
    Some(safe_z.unwrap_or(STOCK_TOP_Z).max(STOCK_TOP_Z))
}

/// Highest finite Z on any rapid, including the Z the rapid starts from.
fn infer_safe_z(program: &Program) -> Option<f64> {
    let mut max_z = None;
    for path in program.toolpaths() {
        let mut cursor: Option<Vec3> = None;
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else { continue };
            let end_pose = motion_end(motion);
            if !pose_is_finite(end_pose) {
                continue;
            }
            let end = end_pose.tip();
            if matches!(motion, ClMotion::Rapid { .. }) {
                if let Some(start) = cursor {
                    raise_z(&mut max_z, start.z());
                }
                raise_z(&mut max_z, end.z());
            }
            cursor = Some(end);
        }
    }
    let machine = program.machine();
    for channel_program in program.channel_programs() {
        let mut joints = [None; MAX_AXES];
        for block in channel_program.blocks() {
            let AxisBlock::Motion(motion) = block else { continue };
            let (targets, rapid) = axis_targets(motion);
            if rapid && let Some(z) = linear_letter(machine, &joints, AxisLetter::Z) {
                raise_z(&mut max_z, z);
            }
            for target in targets {
                store_known_joint(machine, &mut joints, target.axis(), target.value());
            }
            if rapid && let Some(z) = linear_letter(machine, &joints, AxisLetter::Z) {
                raise_z(&mut max_z, z);
            }
        }
    }
    max_z
}

fn raise_z(max_z: &mut Option<f64>, z: f64) {
    if !z.is_finite() {
        return;
    }
    *max_z = Some(match *max_z {
        Some(current) => current.max(z),
        None => z,
    });
}

/// Mill spindles still running when a toolpath ends. Other processes never go in `open`.
struct SpindleCarry {
    span: bool,
    open: Vec<ProcessKind>,
}

fn verify_toolpath(
    index: usize,
    path: &ClPath,
    bound_spindle: Option<&str>,
    heights: Heights,
    cursor: &mut Option<Vec3>,
    out: &mut VerifyReport,
    carry: &mut SpindleCarry,
) {
    let mut open = if carry.span { std::mem::take(&mut carry.open) } else { Vec::new() };
    for (block_index, block) in path.blocks().iter().enumerate() {
        match block {
            ClBlock::Motion(motion) => verify_cl_motion(index, block_index, motion, cursor, heights, out),
            ClBlock::Event(event) => verify_event(event, bound_spindle, &mut open, out),
        }
    }
    if carry.span {
        let mut rest = Vec::new();
        for process in open {
            if matches!(process, ProcessKind::Spindle) {
                carry.open.push(process);
            } else {
                rest.push(process);
            }
        }
        close_processes(&rest, Some(index), None, out);
    } else {
        close_processes(&open, Some(index), None, out);
    }
}

fn verify_channel(machine: Option<&Machine>, program: &ChannelProgram, heights: Heights, out: &mut VerifyReport) {
    let channel_id = program.channel();
    let channel = machine.and_then(|machine| machine.channels().iter().find(|channel| channel.id() == channel_id));
    if channel.is_none() {
        out.issues.push(Issue::UnknownRef(RefKind::Channel(channel_id)));
    }
    let mut joints = [None; MAX_AXES];
    let mut open: Vec<ProcessKind> = Vec::new();
    for (block_index, block) in program.blocks().iter().enumerate() {
        match block {
            AxisBlock::Motion(motion) => verify_axis_motion(machine, channel, channel_id, block_index, motion, &mut joints, heights, out),
            AxisBlock::Event(event) => verify_event(event, None, &mut open, out),
        }
    }
    close_processes(&open, None, Some(channel_id), out);
}

fn verify_event(event: &Event, bound_spindle: Option<&str>, open: &mut Vec<ProcessKind>, out: &mut VerifyReport) {
    match event {
        Event::ProcessOn { process, .. } => {
            // A second spindle-on is a new speed, not another start. One process-off closes it.
            let already = matches!(process, ProcessKind::Spindle) && open.iter().any(|open_process| open_process == process);
            if !already {
                open.push(process.clone());
            }
        }
        Event::ProcessOff { process } => close_process(open, process),
        // A transfer part-off turns the spindle on and leaves it running. The later stop of the
        // bound spindle is the close. Stopping the other spindle does not.
        Event::SpindleRun { spindle, rpm, .. } if *rpm == 0.0 && bound_spindle == Some(spindle.as_str()) => {
            close_process(open, &ProcessKind::Spindle);
        }
        Event::Dwell { seconds } => match check::non_negative(*seconds, "dwell") {
            Ok(seconds) => add_time(out, seconds),
            Err(_) if seconds.is_finite() => {}
            Err(_) => out.issues.push(Issue::NonFinite { what: "dwell".into() }),
        },
        Event::Probe { feed, toward, .. } => {
            if let Some(feed) = feed
                && !feed.is_finite()
            {
                out.issues.push(Issue::NonFinite { what: "probe feed".into() });
            }
            if let Some(pose) = toward {
                note_pose(*pose, out);
            }
        }
        Event::ToolChange { .. }
        | Event::Coolant { .. }
        | Event::Comment { .. }
        | Event::Custom { .. }
        | Event::Sync { .. }
        | Event::CAxis { .. }
        | Event::SpindleBrake { .. }
        | Event::LiveToolOn { .. }
        | Event::LiveToolOff
        | Event::FaceInterpolation { .. }
        | Event::CylinderInterpolation { .. }
        | Event::ChuckClamp { .. }
        | Event::SpindleSync { .. }
        | Event::SpindleRun { .. }
        | Event::SpindleMove { .. }
        | Event::HeadIndex { .. }
        | Event::Tailstock { .. }
        | Event::PartsCatcher { .. } => {}
    }
}

fn close_process(open: &mut Vec<ProcessKind>, process: &ProcessKind) {
    if let Some(index) = open.iter().rposition(|open_process| open_process == process) {
        open.remove(index);
    }
}

fn close_processes(open: &[ProcessKind], toolpath: Option<usize>, channel: Option<u16>, out: &mut VerifyReport) {
    for process in open {
        out.issues.push(Issue::ProcessNotClosed { toolpath, channel, process: process.clone() });
    }
}

fn verify_cl_motion(toolpath: usize, block: usize, motion: &ClMotion, cursor: &mut Option<Vec3>, heights: Heights, out: &mut VerifyReport) {
    let end = motion_end(motion);
    if !note_pose(end, out) {
        return;
    }
    let tip = end.tip();
    let Some(start) = *cursor else {
        if matches!(motion, ClMotion::Rapid { .. }) {
            note_tool_rapid(None, tip, Some(toolpath), None, block, heights, out);
        }
        include_point(out, tip);
        *cursor = Some(tip);
        return;
    };
    match motion {
        ClMotion::Rapid { feed, .. } => {
            note_optional_feed(*feed, out);
            note_tool_rapid(Some(start), tip, Some(toolpath), None, block, heights, out);
            let length = distance(start, tip);
            add_motion(out, length, true, *feed);
        }
        ClMotion::Linear { feed, .. } => {
            note_cutting_feed(*feed, Some(toolpath), None, block, out);
            let length = distance(start, tip);
            add_motion(out, length, false, Some(*feed));
        }
        ClMotion::Arc { to: _, plane, center, sense, feed, .. } => {
            note_cutting_feed(*feed, Some(toolpath), None, block, out);
            let length = arc_or_line(toolpath, block, start, tip, *center, *plane, *sense, None, out);
            add_motion(out, length, false, Some(*feed));
        }
        ClMotion::Helix { plane, center, sense, sweep, feed, .. } => {
            note_cutting_feed(*feed, Some(toolpath), None, block, out);
            note_sweep(*sweep, out);
            let length = arc_or_line(toolpath, block, start, tip, *center, *plane, *sense, Some(*sweep), out);
            add_motion(out, length, false, Some(*feed));
        }
        ClMotion::Thread { .. } => {
            let length = distance(start, tip);
            add_motion(out, length, false, None);
        }
    }
    include_point(out, tip);
    *cursor = Some(tip);
}

/// XY travel at a Z below `safe_z` is an error. Ending below `retract_z` is a warning.
///
/// Z on a straight rapid stays between the endpoints, so the move is below the safe height
/// when the lower endpoint is. The first point has no previous X or Y, so it is not a traverse.
fn note_tool_rapid(
    start: Option<Vec3>,
    end: Vec3,
    toolpath: Option<usize>,
    channel: Option<u16>,
    block: usize,
    heights: Heights,
    out: &mut VerifyReport,
) {
    if let (Some(safe_z), Some(start)) = (heights.safe_z, start)
        && (axis_moved(start.x(), end.x()) || axis_moved(start.y(), end.y()))
    {
        let low = start.z().min(end.z());
        if below_height(low, safe_z) {
            out.rapid_safe_errors.push((toolpath, channel, block, low, safe_z));
        }
    }
    if let Some(retract_z) = heights.retract_z
        && below_height(end.z(), retract_z)
    {
        out.rapid_retract_warnings.push((toolpath, channel, block, end.z(), retract_z));
    }
}

fn note_axis_rapid(
    machine: Option<&Machine>,
    before: &[Option<f64>; MAX_AXES],
    after: &[Option<f64>; MAX_AXES],
    channel: u16,
    block: usize,
    heights: Heights,
    out: &mut VerifyReport,
) {
    let xy = letter_moved(machine, before, after, AxisLetter::X) || letter_moved(machine, before, after, AxisLetter::Y);
    let z0 = linear_letter(machine, before, AxisLetter::Z);
    let z1 = linear_letter(machine, after, AxisLetter::Z);
    if let Some(safe_z) = heights.safe_z
        && xy
        && let Some(low) = z_low(z0, z1)
        && below_height(low, safe_z)
    {
        out.rapid_safe_errors.push((None, Some(channel), block, low, safe_z));
    }
    if let Some(retract_z) = heights.retract_z
        && let Some(end_z) = z1.or(z0)
        && below_height(end_z, retract_z)
    {
        out.rapid_retract_warnings.push((None, Some(channel), block, end_z, retract_z));
    }
}

fn z_low(start: Option<f64>, end: Option<f64>) -> Option<f64> {
    match (start, end) {
        (Some(start), Some(end)) => Some(start.min(end)),
        (None, Some(end)) => Some(end),
        (Some(start), None) => Some(start),
        (None, None) => None,
    }
}

fn letter_moved(machine: Option<&Machine>, before: &[Option<f64>; MAX_AXES], after: &[Option<f64>; MAX_AXES], letter: AxisLetter) -> bool {
    match (linear_letter(machine, before, letter), linear_letter(machine, after, letter)) {
        (Some(before), Some(after)) => axis_moved(before, after),
        _ => false,
    }
}

fn axis_moved(from: f64, to: f64) -> bool {
    let delta = (to - from).abs();
    delta.is_finite() && delta > MIN_COMPONENT
}

fn below_height(z: f64, limit: f64) -> bool {
    z.is_finite() && limit.is_finite() && z < limit
}

fn linear_letter(machine: Option<&Machine>, joints: &[Option<f64>; MAX_AXES], letter: AxisLetter) -> Option<f64> {
    let axis = machine?.axes().iter().find(|axis| axis.kind() == AxisKind::Linear && axis.name() == AxisName::Letter(letter))?;
    joints.get(usize::from(axis.id())).copied().flatten()
}

fn store_known_joint(machine: Option<&Machine>, joints: &mut [Option<f64>; MAX_AXES], axis_id: u8, value: f64) {
    if !value.is_finite() {
        return;
    }
    let known = machine.is_some_and(|machine| machine.axes().iter().any(|axis| axis.id() == axis_id));
    if !known {
        return;
    }
    if let Some(slot) = joints.get_mut(usize::from(axis_id)) {
        *slot = Some(value);
    }
}

fn axis_targets(motion: &AxisMotion) -> (&[AxisTarget], bool) {
    match motion {
        AxisMotion::Rapid { targets, .. } => (targets.as_slice(), true),
        AxisMotion::Linear { targets, .. } | AxisMotion::Arc { targets, .. } | AxisMotion::Helix { targets, .. } => (targets.as_slice(), false),
    }
}

fn pose_is_finite(pose: Pose) -> bool {
    vec_is_finite(pose.tip()) && vec_is_finite(pose.axis())
}

fn vec_is_finite(value: Vec3) -> bool {
    value.x().is_finite() && value.y().is_finite() && value.z().is_finite()
}

fn motion_end(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    }
}

fn verify_axis_motion(
    machine: Option<&Machine>,
    channel: Option<&Channel>,
    channel_id: u16,
    block: usize,
    motion: &AxisMotion,
    joints: &mut [Option<f64>; MAX_AXES],
    heights: Heights,
    out: &mut VerifyReport,
) {
    let (targets, feed, rapid) = match motion {
        AxisMotion::Rapid { targets, feed } => {
            note_optional_feed(*feed, out);
            (targets.as_slice(), *feed, true)
        }
        AxisMotion::Linear { targets, feed } => {
            note_cutting_feed(*feed, None, Some(channel_id), block, out);
            (targets.as_slice(), Some(*feed), false)
        }
        AxisMotion::Arc { targets, feed, center, .. } => {
            note_cutting_feed(*feed, None, Some(channel_id), block, out);
            note_vec3(*center, "arc centre", out);
            (targets.as_slice(), Some(*feed), false)
        }
        AxisMotion::Helix { targets, feed, center, sweep, .. } => {
            note_cutting_feed(*feed, None, Some(channel_id), block, out);
            note_vec3(*center, "helix centre", out);
            note_sweep(*sweep, out);
            (targets.as_slice(), Some(*feed), false)
        }
    };
    let before = *joints;
    let mut length_sq = 0.0;
    let mut moved = false;
    for target in targets {
        if let Some(delta) = apply_target(machine, channel, channel_id, target, joints, out) {
            moved = true;
            length_sq += delta * delta;
        }
    }
    if rapid {
        note_axis_rapid(machine, &before, joints, channel_id, block, heights, out);
    }
    if !moved {
        return;
    }
    let length = if length_sq.is_finite() { length_sq.sqrt() } else { f64::INFINITY };
    add_motion(out, length, rapid, feed);
}

fn apply_target(
    machine: Option<&Machine>,
    channel: Option<&Channel>,
    channel_id: u16,
    target: &AxisTarget,
    joints: &mut [Option<f64>; MAX_AXES],
    out: &mut VerifyReport,
) -> Option<f64> {
    let axis_id = target.axis();
    let value = target.value();
    if !value.is_finite() {
        out.issues.push(Issue::NonFinite { what: "axis target".into() });
        return None;
    }
    let Some(axis) = machine.and_then(|machine| machine.axes().iter().find(|axis| axis.id() == axis_id)) else {
        out.issues.push(Issue::UnknownRef(RefKind::Axis(axis_id)));
        return None;
    };
    if channel.is_some_and(|channel| !channel.axes().contains(&axis_id)) {
        out.issues.push(Issue::UnknownRef(RefKind::Axis(axis_id)));
    }
    if let Some(limits) = axis.limits() {
        if !limits.min().is_finite() || !limits.max().is_finite() {
            out.issues.push(Issue::NonFinite { what: "axis limits".into() });
        } else if value < limits.min() || value > limits.max() {
            out.issues.push(Issue::AxisLimit { channel: Some(channel_id), axis: axis_id, value, min: limits.min(), max: limits.max() });
        }
    }
    let slot = usize::from(axis_id);
    let previous = joints.get(slot).copied()?;
    let delta = match previous {
        Some(previous) if axis.kind() == AxisKind::Linear => value - previous,
        Some(_) => 0.0,
        None => 0.0,
    };
    if let Some(slot) = joints.get_mut(slot) {
        *slot = Some(value);
    }
    if delta.is_finite() { Some(delta) } else { None }
}

/// Arc length when the previous tip lies on the commanded circle. Otherwise the straight chord,
/// plus a [`Issue::Discontinuity`] for the radial or sweep gap.
fn arc_or_line(
    toolpath: usize,
    block: usize,
    start: Vec3,
    end: Vec3,
    center: Vec3,
    plane: Plane,
    sense: ArcSense,
    sweep: Option<f64>,
    out: &mut VerifyReport,
) -> f64 {
    if !note_vec3(center, "arc centre", out) {
        return distance(start, end);
    }
    let (su, sv, sn) = plane_parts(start, plane);
    let (eu, ev, en) = plane_parts(end, plane);
    let (cu, cv, _) = plane_parts(center, plane);
    let r0 = (su - cu).hypot(sv - cv);
    let r1 = (eu - cu).hypot(ev - cv);
    if !r0.is_finite() || !r1.is_finite() {
        out.issues.push(Issue::NonFinite { what: "arc radius".into() });
        return distance(start, end);
    }
    let radial = check::abs_gap(r0, r1);
    let angular = helix_angle_gap(su, sv, eu, ev, cu, cv, r0, sense, sweep);
    let gap = radial.max(angular);
    if gap > CONTINUITY_MM {
        out.issues.push(Issue::Discontinuity { toolpath, block, gap, tolerance: CONTINUITY_MM });
        return distance(start, end);
    }
    let Some(turns) = sweep.or_else(|| planar_sweep(su, sv, eu, ev, cu, cv, sense == ArcSense::Ccw)) else {
        return distance(start, end);
    };
    if !turns.is_finite() || turns < 0.0 {
        return distance(start, end);
    }
    include_arc_bounds(out, center, plane, r0, su, sv, sn, en, sense, turns);
    let planar = r0 * turns;
    let dz = en - sn;
    let length = (planar * planar + dz * dz).sqrt();
    if length.is_finite() { length } else { f64::INFINITY }
}

fn helix_angle_gap(su: f64, sv: f64, eu: f64, ev: f64, cu: f64, cv: f64, radius: f64, sense: ArcSense, sweep: Option<f64>) -> f64 {
    let Some(sweep) = sweep else {
        return 0.0;
    };
    if !sweep.is_finite() || radius <= MIN_COMPONENT {
        return 0.0;
    }
    let a0 = (sv - cv).atan2(su - cu);
    let a1 = (ev - cv).atan2(eu - cu);
    if !a0.is_finite() || !a1.is_finite() {
        return f64::INFINITY;
    }
    let signed = match sense {
        ArcSense::Ccw => sweep,
        ArcSense::Cw => -sweep,
    };
    let err = (a1 - (a0 + signed)).rem_euclid(TAU);
    let err = err.min(TAU - err);
    if err.is_finite() { radius * err } else { f64::INFINITY }
}

fn planar_sweep(su: f64, sv: f64, eu: f64, ev: f64, cu: f64, cv: f64, ccw: bool) -> Option<f64> {
    let r0 = (su - cu).hypot(sv - cv);
    if !r0.is_finite() || r0 <= MIN_COMPONENT {
        return None;
    }
    let a0 = (sv - cv).atan2(su - cu);
    let a1 = (ev - cv).atan2(eu - cu);
    if !a0.is_finite() || !a1.is_finite() {
        return None;
    }
    let chord = (eu - su).hypot(ev - sv);
    // Same closure as `campanion_geom::Arc::full_turn_chord`: 1e-9 mm, or 1e-9 radii.
    let closure = 1e-9_f64.max(r0 * 1e-9);
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= closure {
        TAU
    } else if ccw {
        if ccw_delta <= MIN_COMPONENT { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= MIN_COMPONENT { TAU } else { cw }
    };
    sweep.is_finite().then_some(sweep)
}

fn include_arc_bounds(
    out: &mut VerifyReport,
    center: Vec3,
    plane: Plane,
    radius: f64,
    su: f64,
    sv: f64,
    start_n: f64,
    end_n: f64,
    sense: ArcSense,
    sweep: f64,
) {
    if radius <= MIN_COMPONENT || !sweep.is_finite() || sweep <= 0.0 {
        return;
    }
    let a0 = (sv - plane_parts(center, plane).1).atan2(su - plane_parts(center, plane).0);
    if !a0.is_finite() {
        return;
    }
    let (cu, cv, _) = plane_parts(center, plane);
    let ccw = sense == ArcSense::Ccw;
    for quadrant in [0.0, TAU * 0.25, TAU * 0.5, TAU * 0.75] {
        let delta = if ccw { (quadrant - a0).rem_euclid(TAU) } else { (a0 - quadrant).rem_euclid(TAU) };
        if !delta.is_finite() || delta > sweep {
            continue;
        }
        let fraction = if sweep > MIN_COMPONENT { (delta / sweep).clamp(0.0, 1.0) } else { 0.0 };
        let normal = start_n + (end_n - start_n) * fraction;
        let u = cu + radius * quadrant.cos();
        let v = cv + radius * quadrant.sin();
        if let Some(point) = from_plane(u, v, normal, plane) {
            include_point(out, point);
        }
    }
}

fn plane_parts(point: Vec3, plane: Plane) -> (f64, f64, f64) {
    match plane {
        Plane::Xy => (point.x(), point.y(), point.z()),
        Plane::Zx => (point.z(), point.x(), point.y()),
        Plane::Yz => (point.y(), point.z(), point.x()),
    }
}

fn from_plane(u: f64, v: f64, normal: f64, plane: Plane) -> Option<Vec3> {
    let (x, y, z) = match plane {
        Plane::Xy => (u, v, normal),
        Plane::Zx => (v, normal, u),
        Plane::Yz => (normal, u, v),
    };
    Vec3::new(x, y, z).ok()
}

fn note_pose(pose: Pose, out: &mut VerifyReport) -> bool {
    note_vec3(pose.tip(), "tool tip", out) && note_vec3(pose.axis(), "tool axis", out)
}

fn note_vec3(value: Vec3, what: &str, out: &mut VerifyReport) -> bool {
    if value.x().is_finite() && value.y().is_finite() && value.z().is_finite() {
        true
    } else {
        out.issues.push(Issue::NonFinite { what: what.into() });
        false
    }
}

fn note_optional_feed(feed: Option<f64>, out: &mut VerifyReport) {
    if let Some(feed) = feed
        && !feed.is_finite()
    {
        out.issues.push(Issue::NonFinite { what: "rapid feed".into() });
    }
}

fn note_cutting_feed(feed: f64, toolpath: Option<usize>, channel: Option<u16>, block: usize, out: &mut VerifyReport) {
    if check::positive(feed, "feed").is_ok() {
        return;
    }
    if feed.is_finite() {
        out.issues.push(Issue::BadFeed { toolpath, channel, block, feed });
    } else {
        out.issues.push(Issue::NonFinite { what: "feed".into() });
    }
}

fn note_sweep(sweep: f64, out: &mut VerifyReport) {
    if !sweep.is_finite() {
        out.issues.push(Issue::NonFinite { what: "helix sweep".into() });
    }
}

fn distance(from: Vec3, to: Vec3) -> f64 {
    let dx = to.x() - from.x();
    let dy = to.y() - from.y();
    let dz = to.z() - from.z();
    let sum = dx * dx + dy * dy + dz * dz;
    if sum.is_finite() { sum.sqrt() } else { f64::INFINITY }
}

fn add_motion(out: &mut VerifyReport, length: f64, rapid: bool, feed: Option<f64>) {
    if !add_labelled(&mut out.total_length, length) {
        out.issues.push(Issue::NonFinite { what: "move length".into() });
        return;
    }
    if rapid && !add_labelled(&mut out.rapid_length, length) {
        out.issues.push(Issue::NonFinite { what: "move length".into() });
    }
    if let Some(feed) = feed
        && check::positive(feed, "feed").is_ok()
        && !add_labelled(&mut out.time_s, length / feed * 60.0)
    {
        out.issues.push(Issue::NonFinite { what: "time".into() });
    }
}

fn add_time(out: &mut VerifyReport, seconds: f64) {
    if !add_labelled(&mut out.time_s, seconds) {
        out.issues.push(Issue::NonFinite { what: "time".into() });
    }
}

/// Add `extra` when the sum stays finite. Returns false when `extra` or the sum is not finite.
fn add_labelled(slot: &mut f64, extra: f64) -> bool {
    let sum = *slot + extra;
    if extra.is_finite() && sum.is_finite() {
        *slot = sum;
        true
    } else {
        false
    }
}

fn include_point(out: &mut VerifyReport, point: Vec3) {
    if !point.x().is_finite() || !point.y().is_finite() || !point.z().is_finite() {
        out.issues.push(Issue::NonFinite { what: "position".into() });
        return;
    }
    out.bounds = Some(match out.bounds {
        None => Bounds { min: point, max: point },
        Some(bounds) => Bounds {
            min: Vec3::new(bounds.min.x().min(point.x()), bounds.min.y().min(point.y()), bounds.min.z().min(point.z())).unwrap_or(bounds.min),
            max: Vec3::new(bounds.max.x().max(point.x()), bounds.max.y().max(point.y()), bounds.max.z().max(point.z())).unwrap_or(bounds.max),
        },
    });
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use serde_json::Value;

    use super::*;
    use crate::geom::{ArcSense, Plane, Pose, Vec3};
    use crate::machine::{Axis, AxisKind, AxisLetter, AxisLimits, AxisName, Channel, KinematicNode, Machine};
    use crate::motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole};
    use crate::process::{Event, ProcessKind, ProcessParams};
    use crate::program::Program;
    use crate::turning::{TurnBinding, TurningSetup};

    fn tip(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn feeds(cut: f64) -> Feeds {
        Feeds::try_new(cut, None, None, None, None).expect("feeds")
    }

    fn path(blocks: Vec<ClBlock>) -> ClPath {
        ClPath::try_new(None, None, CompensationMode::Off, feeds(600.0), None, Vec::new(), blocks).expect("path")
    }

    fn program_with_path(blocks: Vec<ClBlock>) -> Program {
        Program::try_new(None, Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new(), None).expect("program")
    }

    fn program_with_paths(paths: Vec<Vec<ClBlock>>) -> Program {
        let paths = paths.into_iter().map(path).collect();
        Program::try_new(None, Vec::new(), None, paths, Vec::new(), Vec::new(), None).expect("program")
    }

    fn axis_machine(limits: Option<(f64, f64)>) -> Machine {
        let limits = limits.map(|(min, max)| AxisLimits::try_new(min, max).expect("limits"));
        let axis = Axis::try_new(
            0,
            AxisName::Letter(AxisLetter::X),
            AxisKind::Linear,
            limits,
            KinematicNode::try_new(None, Vec3::X, Vec3::ZERO).expect("node"),
        )
        .expect("axis");
        let channel = Channel::try_new(0, "main", vec![0]).expect("channel");
        Machine::try_new(vec![axis], vec![channel]).expect("machine")
    }

    fn patched(program: &Program, edit: impl FnOnce(&mut Value)) -> Program {
        let mut value = serde_json::to_value(program).expect("value");
        edit(&mut value);
        serde_json::from_value(value).expect("program")
    }

    #[test]
    fn clean_program_has_length_time_and_bounds() {
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: tip(0.0, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Rapid { to: tip(0.0, 0.0, 5.0), feed: Some(3000.0) }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
            ClBlock::Motion(ClMotion::Linear { to: tip(0.0, 0.0, 0.0), feed: 300.0, role: MoveRole::Plunge }),
            ClBlock::Motion(ClMotion::Linear { to: tip(10.0, 0.0, 0.0), feed: 600.0, role: MoveRole::Cut }),
            ClBlock::Motion(ClMotion::Arc {
                to: tip(-10.0, 0.0, 0.0),
                plane: Plane::Xy,
                center: Vec3::ZERO,
                sense: ArcSense::Ccw,
                feed: 600.0,
                role: MoveRole::Cut,
            }),
            ClBlock::Event(Event::Dwell { seconds: 1.5 }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
        ];
        let report = verify(&program_with_path(blocks));
        assert!(report.issues().is_empty(), "{:?}", report.issues());
        let expected_len = 20.0 + 10.0 * PI;
        assert!((report.total_length() - expected_len).abs() < 1e-9, "{}", report.total_length());
        assert!((report.rapid_length() - 5.0).abs() < 1e-9, "{}", report.rapid_length());
        let expected_time = 3.6 + PI;
        assert!((report.time_s() - expected_time).abs() < 1e-9, "{}", report.time_s());
        let bounds = report.bounds().expect("bounds");
        assert!((bounds.min().x() - -10.0).abs() < 1e-6);
        assert!((bounds.max().x() - 10.0).abs() < 1e-6);
        assert!(bounds.min().y().abs() < 1e-6);
        assert!((bounds.max().y() - 10.0).abs() < 1e-6);
        assert!(bounds.min().z().abs() < 1e-9);
        assert!((bounds.max().z() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn axis_limit_is_reported() {
        let motion = AxisMotion::Linear { targets: vec![AxisTarget::try_new(0, 25.0).expect("target")], feed: 100.0 };
        let channel = ChannelProgram::try_new(0, vec![AxisBlock::Motion(motion)]).expect("channel");
        let program = Program::try_new(None, Vec::new(), Some(axis_machine(Some((-10.0, 10.0)))), Vec::new(), vec![channel], Vec::new(), None)
            .expect("program");
        let report = verify(&program);
        assert_eq!(report.issues().len(), 1, "{:?}", report.issues());
        assert!(matches!(
            report.issues()[0],
            Issue::AxisLimit { channel: Some(0), axis: 0, value, min, max } if value == 25.0 && min == -10.0 && max == 10.0
        ));
    }

    #[test]
    fn discontinuity_is_reported_when_arc_radii_differ() {
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: tip(10.0, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Arc {
                to: tip(0.0, 5.0, 0.0),
                plane: Plane::Xy,
                center: Vec3::ZERO,
                sense: ArcSense::Ccw,
                feed: 100.0,
                role: MoveRole::Cut,
            }),
        ];
        let report = verify(&program_with_path(blocks));
        assert_eq!(report.issues().len(), 1, "{:?}", report.issues());
        match &report.issues()[0] {
            Issue::Discontinuity { gap, tolerance, toolpath, block } => {
                assert_eq!((*toolpath, *block), (0, 1));
                assert!((*gap - 5.0).abs() < 1e-9, "{gap}");
                assert!(*gap > *tolerance);
            }
            other => panic!("expected discontinuity, got {other:?}"),
        }
    }

    fn bound_main(blocks: Vec<ClBlock>) -> Program {
        let setup = TurningSetup::single(None, 3_000.0)
            .expect("setup")
            .with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding"))
            .expect("binding");
        Program::try_new(None, Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new(), None)
            .expect("program")
            .with_turning(Some(setup))
            .expect("turning")
    }

    #[test]
    fn spindle_stop_on_the_bound_spindle_closes_the_process() {
        let stopped = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
            ClBlock::Event(Event::SpindleRun { spindle: "sub".into(), rpm: 0.0, reverse: false }),
            ClBlock::Event(Event::SpindleRun { spindle: "main".into(), rpm: 0.0, reverse: false }),
        ];
        let report = verify(&bound_main(stopped));
        assert!(report.issues().iter().all(|issue| !matches!(issue, Issue::ProcessNotClosed { .. })), "{:?}", report.issues());

        let other_spindle = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
            ClBlock::Event(Event::SpindleRun { spindle: "sub".into(), rpm: 0.0, reverse: false }),
        ];
        let open_report = verify(&bound_main(other_spindle));
        assert!(
            open_report
                .issues()
                .iter()
                .any(|issue| matches!(issue, Issue::ProcessNotClosed { process: ProcessKind::Spindle, toolpath: Some(0), .. }))
        );
    }

    #[test]
    fn mill_spindle_left_on_is_reported_at_program_end() {
        let blocks = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }),
            ClBlock::Motion(ClMotion::Linear { to: tip(1.0, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let report = verify(&program_with_path(blocks));
        assert!(
            report
                .issues()
                .iter()
                .any(|issue| matches!(issue, Issue::ProcessNotClosed { process: ProcessKind::Spindle, toolpath: None, channel: None }))
        );
    }

    #[test]
    fn a_spindle_speed_change_does_not_need_a_stop_between_the_ons() {
        let slow = ProcessParams::new().speed(Some(10_000.0)).expect("speed");
        let fast = ProcessParams::new().speed(Some(16_000.0)).expect("speed");
        let blocks = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: slow }),
            ClBlock::Motion(ClMotion::Linear { to: tip(1.0, 0.0, -1.0), feed: 100.0, role: MoveRole::Cut }),
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: fast }),
            ClBlock::Motion(ClMotion::Linear { to: tip(2.0, 0.0, -1.0), feed: 100.0, role: MoveRole::Cut }),
            ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
        ];
        let report = verify(&program_with_path(blocks));
        assert!(report.issues().iter().all(|issue| !matches!(issue, Issue::ProcessNotClosed { .. })), "{:?}", report.issues());
    }

    #[test]
    fn mill_spindle_may_cross_toolpaths_when_a_later_path_stops_it() {
        let on = ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() });
        let off = ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle });
        let cut = ClBlock::Motion(ClMotion::Linear { to: tip(1.0, 0.0, -1.0), feed: 100.0, role: MoveRole::Cut });
        let report = verify(&program_with_paths(vec![vec![on, cut.clone()], vec![cut, off]]));
        assert!(report.issues().iter().all(|issue| !matches!(issue, Issue::ProcessNotClosed { .. })), "{:?}", report.issues());
    }

    #[test]
    fn process_left_on_is_reported() {
        let blocks = vec![
            ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() }),
            ClBlock::Motion(ClMotion::Linear { to: tip(1.0, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let report = verify(&program_with_path(blocks));
        assert!(report.issues().iter().any(|issue| matches!(issue, Issue::ProcessNotClosed { process: ProcessKind::Torch, toolpath: Some(0), .. })));
    }

    #[test]
    fn non_finite_length_is_reported() {
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: tip(1.0e308, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: tip(-1.0e308, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let report = verify(&program_with_path(blocks));
        assert!(report.issues().iter().any(|issue| matches!(issue, Issue::NonFinite { what } if what == "move length")));
        assert!(report.total_length().is_finite());
        assert!(report.rapid_length().is_finite());
        assert!(report.time_s().is_finite());
    }

    #[test]
    fn bad_feed_on_a_cutting_move_is_reported() {
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: tip(0.0, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: tip(10.0, 0.0, 0.0), feed: 600.0, role: MoveRole::Cut }),
        ];
        let program = patched(&program_with_path(blocks), |value| {
            let Some(feed) = value.pointer_mut("/toolpaths/0/blocks/1/feed") else {
                panic!("feed");
            };
            *feed = Value::from(0.0);
        });
        let report = verify(&program);
        assert!(report.issues().iter().any(|issue| matches!(issue, Issue::BadFeed { feed, block: 1, .. } if *feed == 0.0)));
    }

    #[test]
    fn unknown_axis_is_reported() {
        let motion = AxisMotion::Linear { targets: vec![AxisTarget::try_new(0, 1.0).expect("target")], feed: 100.0 };
        let channel = ChannelProgram::try_new(0, vec![AxisBlock::Motion(motion)]).expect("channel");
        let program = Program::try_new(None, Vec::new(), Some(axis_machine(None)), Vec::new(), vec![channel], Vec::new(), None).expect("program");
        let program = patched(&program, |value| {
            let Some(axis) = value.pointer_mut("/channel_programs/0/blocks/0/targets/0/axis") else {
                panic!("axis");
            };
            *axis = Value::from(9);
        });
        let report = verify(&program);
        assert!(report.issues().iter().any(|issue| matches!(issue, Issue::UnknownRef(RefKind::Axis(9)))));
    }

    fn rapid(x: f64, y: f64, z: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Rapid { to: tip(x, y, z), feed: None })
    }

    fn heights(safe_z: Option<f64>, retract_z: Option<f64>) -> VerifyOptions {
        VerifyOptions { safe_z, retract_z }
    }

    fn xyz_machine() -> Machine {
        let axis = |id, letter, direction| {
            Axis::try_new(id, AxisName::Letter(letter), AxisKind::Linear, None, KinematicNode::try_new(None, direction, Vec3::ZERO).expect("node"))
                .expect("axis")
        };
        let axes = vec![axis(0, AxisLetter::X, Vec3::X), axis(1, AxisLetter::Y, Vec3::Y), axis(2, AxisLetter::Z, Vec3::Z)];
        let channel = Channel::try_new(7, "head", vec![0, 1, 2]).expect("channel");
        Machine::try_new(axes, vec![channel]).expect("machine")
    }

    fn axis_rapid(targets: &[(u8, f64)]) -> AxisBlock {
        let targets = targets.iter().map(|(axis, value)| AxisTarget::try_new(*axis, *value).expect("target")).collect();
        AxisBlock::Motion(AxisMotion::Rapid { targets, feed: None })
    }

    fn axis_program(blocks: Vec<AxisBlock>) -> Program {
        let channel = ChannelProgram::try_new(7, blocks).expect("channel");
        Program::try_new(None, Vec::new(), Some(xyz_machine()), Vec::new(), vec![channel], Vec::new(), None).expect("program")
    }

    #[test]
    fn rapid_that_changes_x_or_y_below_safe_z_is_an_error() {
        let flat = vec![rapid(0.0, 0.0, 4.0), rapid(0.0, 6.0, 4.0)];
        let report = verify_with(&program_with_path(flat), &heights(Some(8.0), None));
        assert!(report.issues().is_empty(), "{:?}", report.issues());
        assert_eq!(report.rapid_safe_errors(), &[(Some(0), None, 1, 4.0, 8.0)]);
        assert!(report.rapid_retract_warnings().is_empty());

        let diagonal = vec![rapid(0.0, 0.0, 10.0), rapid(4.0, 2.0, 1.0)];
        let report = verify_with(&program_with_path(diagonal), &heights(Some(10.0), None));
        assert_eq!(report.rapid_safe_errors(), &[(Some(0), None, 1, 1.0, 10.0)]);

        let axis = axis_program(vec![axis_rapid(&[(0, 0.0), (1, 0.0), (2, 12.0)]), axis_rapid(&[(2, 2.0)]), axis_rapid(&[(0, 5.0), (1, 1.0)])]);
        let report = verify_with(&axis, &heights(Some(12.0), None));
        assert_eq!(report.rapid_safe_errors(), &[(None, Some(7), 2, 2.0, 12.0)]);
    }

    #[test]
    fn rapid_at_safe_z_or_without_xy_travel_is_allowed() {
        let blocks = vec![
            rapid(0.0, 0.0, 10.0),
            rapid(3.0, 4.0, 10.0),
            rapid(3.0, 4.0, 0.0),
            ClBlock::Motion(ClMotion::Linear { to: tip(30.0, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let report = verify_with(&program_with_path(blocks), &heights(Some(10.0), None));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert!(report.issues().is_empty(), "{:?}", report.issues());

        let axis = axis_program(vec![axis_rapid(&[(0, 0.0), (1, 0.0), (2, 12.0)]), axis_rapid(&[(0, 9.0), (2, 12.0)])]);
        let report = verify_with(&axis, &heights(Some(12.0), None));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
    }

    #[test]
    fn missing_safe_z_is_the_highest_z_a_rapid_reaches() {
        let low = vec![rapid(0.0, 0.0, 18.0), rapid(7.0, 0.0, 2.0)];
        let report = verify(&program_with_path(low));
        assert_eq!(report.rapid_safe_errors(), &[(Some(0), None, 1, 2.0, 18.0)]);
        assert!(report.rapid_retract_warnings().is_empty());

        let at_clearance = vec![rapid(0.0, 0.0, 18.0), rapid(7.0, 0.0, 18.0)];
        let report = verify(&program_with_path(at_clearance));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());

        // The rapid leaves the feed height, so that Z is reached by the rapid and becomes the clearance.
        let fed = vec![
            rapid(0.0, 0.0, 3.0),
            ClBlock::Motion(ClMotion::Linear { to: tip(0.0, 0.0, 40.0), feed: 100.0, role: MoveRole::Cut }),
            rapid(5.0, 0.0, 3.0),
        ];
        let report = verify(&program_with_path(fed));
        assert_eq!(report.rapid_safe_errors(), &[(Some(0), None, 2, 3.0, 40.0)]);
    }

    #[test]
    fn rapid_ending_below_retract_z_is_a_warning() {
        let blocks = vec![rapid(0.0, 0.0, 20.0), rapid(10.0, 0.0, 8.0)];
        let report = verify_with(&program_with_path(blocks), &heights(Some(5.0), Some(15.0)));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert_eq!(report.rapid_retract_warnings(), &[(Some(0), None, 1, 8.0, 15.0)]);

        let plunge = vec![rapid(1.0, 1.0, 12.0), rapid(1.0, 1.0, 4.0)];
        let report = verify_with(&program_with_path(plunge), &heights(Some(12.0), Some(12.0)));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert_eq!(report.rapid_retract_warnings(), &[(Some(0), None, 1, 4.0, 12.0)]);

        let axis = axis_program(vec![axis_rapid(&[(0, 0.0), (1, 0.0), (2, 12.0)]), axis_rapid(&[(2, 3.0)])]);
        let report = verify_with(&axis, &heights(Some(12.0), Some(12.0)));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert_eq!(report.rapid_retract_warnings(), &[(None, Some(7), 1, 3.0, 12.0)]);
    }

    #[test]
    fn rapid_ending_at_retract_z_is_allowed_and_unset_retract_skips_the_warning() {
        let at_retract = vec![rapid(0.0, 0.0, 5.0), rapid(4.0, 0.0, 9.0)];
        let report = verify_with(&program_with_path(at_retract), &heights(Some(5.0), Some(5.0)));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert!(report.rapid_retract_warnings().is_empty(), "{:?}", report.rapid_retract_warnings());

        let low = vec![rapid(0.0, 0.0, 1.0), rapid(0.0, 0.0, 0.5)];
        let report = verify(&program_with_path(low));
        assert!(report.rapid_retract_warnings().is_empty(), "{:?}", report.rapid_retract_warnings());
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
    }

    fn ended_at_retract_then(second: Vec<ClBlock>) -> Program {
        // First toolpath ends with a rapid to Z5 at the origin. The next path continues from that tip.
        program_with_paths(vec![vec![rapid(0.0, 0.0, 5.0)], second])
    }

    #[test]
    fn sideways_rapid_at_the_start_of_the_next_toolpath_is_an_error() {
        let report = verify_with(&ended_at_retract_then(vec![rapid(50.0, 40.0, 2.0)]), &heights(Some(5.0), None));
        assert!(report.issues().is_empty(), "{:?}", report.issues());
        assert_eq!(report.rapid_safe_errors(), &[(Some(1), None, 0, 2.0, 5.0)]);
    }

    #[test]
    fn rapid_to_safe_z_then_down_at_the_start_of_the_next_toolpath_is_allowed() {
        let second = vec![rapid(50.0, 40.0, 5.0), rapid(50.0, 40.0, 2.0)];
        let report = verify_with(&ended_at_retract_then(second), &heights(Some(5.0), None));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
        assert!(report.rapid_retract_warnings().is_empty(), "{:?}", report.rapid_retract_warnings());
        assert!(report.issues().is_empty(), "{:?}", report.issues());
    }

    #[test]
    fn inferred_safe_z_flags_a_sideways_rapid_at_the_start_of_the_next_toolpath() {
        let report = verify(&ended_at_retract_then(vec![rapid(50.0, 40.0, 2.0)]));
        assert_eq!(report.rapid_safe_errors(), &[(Some(1), None, 0, 2.0, 5.0)]);
    }

    #[test]
    fn xy_rapid_below_the_stock_top_is_an_error_on_a_mill_program() {
        let through_stock = vec![rapid(0.0, 0.0, -12.0), rapid(10.0, 0.0, -12.0)];
        let report = verify(&program_with_path(through_stock));
        assert_eq!(report.rapid_safe_errors(), &[(Some(0), None, 1, -12.0, 0.0)]);

        let at_stock = vec![rapid(0.0, 0.0, 0.0), rapid(10.0, 0.0, 0.0)];
        let report = verify(&program_with_path(at_stock));
        assert!(report.rapid_safe_errors().is_empty(), "{:?}", report.rapid_safe_errors());
    }
}
