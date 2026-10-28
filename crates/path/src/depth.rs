//! Depth passes, with a linear ramp, a helix, or a plunge.
//!
//! Depth is a positive distance downward from the stock surface. Z is up, so pass `d` is cut
//! at `stock_z - d`. Passes are spaced by the step-down. The last pass is the total depth and
//! is not deeper: 5 mm at a 2 mm step is 2, 4, 5.
//!
//! A ramp follows the contour. Its angle with the XY plane never exceeds the given maximum.
//! A helix is a counter-clockwise circle about the contour's bounding-box centre; each full
//! turn lowers Z by the pitch. A plunge is a vertical move at the contour start. The contour
//! is then cut once at the pass height. Without tabs the contour must be closed. With tabs an
//! open contour is cut forward, then back, and a pass lifts across each bridge.
//!
//! [`DepthPlan::entry_start`] is where a ramp or helix begins. It defaults to the stock
//! surface. When the tool is already at a retract height above the stock, set it to that
//! height so the first descent is long enough for the whole drop. Later passes still start
//! at the previous level. A plunge ignores it. A ramp that would cross a tab below the bridge
//! top lifts across that bridge instead of descending through it.

use campanion_geom::{Contour, Point, Segment, Tolerance};

use crate::Result;
use crate::check::{self, finite, positive};
use crate::profile::{Tab, TabSpec, covering_tab_end, emit_crossing, emit_tabbed_span, layout_tabs, move_entry_past_tab, room_to_tab};
use crate::toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};

const MAX_PASSES: usize = 1024;
const MAX_LAPS: usize = 1024;
const MAX_MM: f64 = 1.0e6;

/// How the tool enters each depth pass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DepthEntry {
    /// Follow the contour while descending. `max_angle` is radians from the XY plane.
    Ramp { max_angle: f64 },
    /// Counter-clockwise helix about the contour's bounding-box centre.
    ///
    /// `radius` and `pitch` are millimetres. A full turn is one arc and lowers Z by `pitch`.
    Helix { radius: f64, pitch: f64 },
    /// Straight down at the contour start.
    Plunge,
}

/// Bridges left while a profile is cut to depth.
///
/// `depth` is the operation depth in millimetres downward from the stock surface, which can be
/// deeper than this plan's total when a roughing link stops short of the floor. A positive tab
/// height leaves a bridge whose top is `stock_z - (depth - height)`. Passes above that top are
/// not interrupted. Omitted or zero height lifts every pass to `retract`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthTabs {
    spec: TabSpec,
    depth: f64,
    retract: f64,
}

impl DepthTabs {
    /// `depth` and `retract` are millimetres. `retract` is an absolute Z.
    pub fn new(spec: TabSpec, depth: f64, retract: f64) -> Self {
        Self { spec, depth, retract }
    }

    pub fn spec(self) -> TabSpec {
        self.spec
    }

    /// Operation depth downward from the stock surface, millimetres.
    pub fn depth(self) -> f64 {
        self.depth
    }

    /// Absolute Z of a full-depth crossing.
    pub fn retract(self) -> f64 {
        self.retract
    }
}

/// Stock height, total depth, step-down, and the entry move.
///
/// `entry_start` is the Z of the first ramp or helix. It defaults to the stock surface.
/// A value above the stock lengthens that first descent so the angle stays within the
/// limit when the posted move actually starts at the retract height. Later passes start
/// at the previous level. A plunge ignores it and still starts on the stock. Tabs, when
/// set, lift a ramp across each bridge instead of letting that descent cut through it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DepthPlan {
    stock_z: f64,
    total_depth: f64,
    step_down: f64,
    entry: DepthEntry,
    entry_start: f64,
    tabs: Option<DepthTabs>,
}

impl DepthPlan {
    /// `total_depth` and `step_down` are millimetres downward from `stock_z`.
    ///
    /// The first ramp or helix starts on the stock surface. Values are checked when the plan is used.
    /// Tabs are off until [`Self::with_tabs`].
    pub fn new(stock_z: f64, total_depth: f64, step_down: f64, entry: DepthEntry) -> Self {
        Self { stock_z, total_depth, step_down, entry, entry_start: stock_z, tabs: None }
    }

    /// Start a ramp or helix at `z` instead of the stock surface.
    ///
    /// `z` may be above the stock. The first descent is sized for the drop from `z` to the
    /// first pass. A plunge still starts on the stock. Checked when the plan is used.
    pub fn entry_start(mut self, z: f64) -> Self {
        self.entry_start = z;
        self
    }

    /// Leave bridges on this plan. Replaces a tab spec already set.
    pub fn with_tabs(mut self, tabs: DepthTabs) -> Self {
        self.tabs = Some(tabs);
        self
    }

    pub fn stock_z(self) -> f64 {
        self.stock_z
    }

    pub fn total_depth(self) -> f64 {
        self.total_depth
    }

    pub fn step_down(self) -> f64 {
        self.step_down
    }

    pub fn entry(self) -> DepthEntry {
        self.entry
    }

    /// Z of the first ramp or helix. The stock surface, unless [`Self::entry_start`] set another.
    pub fn entry_start_z(self) -> f64 {
        self.entry_start
    }

    pub fn tabs(self) -> Option<DepthTabs> {
        self.tabs
    }
}

/// Depth of each pass, in millimetres downward from the stock surface.
///
/// Passes are `step_down` apart until the last, which is `total_depth` and is not deeper.
/// Five millimetres at a two millimetre step is `[2, 4, 5]`.
pub fn pass_depths(total_depth: f64, step_down: f64) -> Result<Vec<f64>> {
    let total_depth = capped_positive(total_depth, "total depth")?;
    let step_down = capped_positive(step_down, "step-down")?;
    let steps = (total_depth / step_down).ceil();
    if !steps.is_finite() || steps > MAX_PASSES as f64 {
        return Err(check::invalid(format!("too many depth passes: limit is {MAX_PASSES}")));
    }
    let mut depths = Vec::new();
    let mut done = 0.0;
    loop {
        if depths.len() >= MAX_PASSES {
            return Err(check::invalid(format!("too many depth passes: limit is {MAX_PASSES}")));
        }
        let next = done + step_down;
        if !next.is_finite() {
            return Err(check::invalid("depth pass is not finite"));
        }
        if next >= total_depth - 1e-9 {
            depths.push(total_depth);
            break;
        }
        done = next;
        depths.push(done);
    }
    Ok(depths)
}

/// Cut `contour` once at each depth, entering with `plan.entry`.
///
/// Tabs allow an open contour. Without them the contour must be closed.
pub fn depth_toolpath(contour: &Contour, plan: &DepthPlan, tolerance: &Tolerance) -> Result<Toolpath> {
    if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
        return Err(check::invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
    }
    let stock_z = capped_abs(plan.stock_z, "stock z")?;
    let entry_start = capped_abs(plan.entry_start, "entry start")?;
    if entry_start + 1e-9 < stock_z {
        return Err(check::invalid("entry start is below the stock"));
    }
    if plan.tabs.is_some() {
        return tabbed_toolpath(contour, plan, tolerance, entry_start);
    }
    if !contour.is_closed() {
        return Err(check::invalid("depth passes need a closed contour"));
    }
    let depths = pass_depths(plan.total_depth, plan.step_down)?;
    let length = contour.length()?;
    if !length.is_finite() || length <= tolerance.linear() {
        return Err(check::invalid("contour length is too small"));
    }
    let floor = stock_z - plan.total_depth;
    finite(floor, "final depth")?;
    match plan.entry {
        DepthEntry::Ramp { max_angle } => ramp_passes(contour, length, stock_z, entry_start, &depths, max_angle, tolerance),
        DepthEntry::Helix { radius, pitch } => helix_passes(contour, stock_z, entry_start, &depths, radius, pitch),
        DepthEntry::Plunge => plunge_passes(contour, stock_z, &depths),
    }
}

struct Pen {
    moves: Vec<Move>,
    at: Option<Point3>,
}

impl Pen {
    fn new() -> Self {
        Self { moves: Vec::new(), at: None }
    }

    fn z(&self) -> Result<f64> {
        self.at.map(Point3::z).ok_or_else(|| check::invalid("toolpath has no position"))
    }

    fn rapid(&mut self, to: Point3) -> Result<()> {
        if self.at.is_some_and(|at| at.almost_eq(to, 1e-9)) {
            self.at = Some(to);
            return Ok(());
        }
        check::push(&mut self.moves, Move::Rapid { to })?;
        self.at = Some(to);
        Ok(())
    }

    fn linear(&mut self, to: Point3, kind: CutKind) -> Result<()> {
        if self.at.is_some_and(|at| at.almost_eq(to, 1e-9)) {
            self.at = Some(to);
            return Ok(());
        }
        check::push(&mut self.moves, Move::Linear { to, kind })?;
        self.at = Some(to);
        Ok(())
    }

    fn arc(&mut self, to: Point3, centre: Point3, ccw: bool, kind: CutKind) -> Result<()> {
        check::push(&mut self.moves, Move::Arc { to, centre, ccw, kind })?;
        self.at = Some(to);
        Ok(())
    }

    fn event(&mut self, event: ProcessEvent) -> Result<()> {
        check::push(&mut self.moves, Move::Event(event))
    }

    fn finish(self) -> Result<Toolpath> {
        Toolpath::new(self.moves)
    }
}

fn plunge_passes(contour: &Contour, stock_z: f64, depths: &[f64]) -> Result<Toolpath> {
    let mut pen = Pen::new();
    let start = first_point(contour)?;
    pen.rapid(Point3::new(start.x(), start.y(), stock_z)?)?;
    pen.event(ProcessEvent::On)?;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        pen.linear(Point3::new(start.x(), start.y(), target)?, CutKind::Plunge)?;
        cut_contour(&mut pen, contour, target)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn helix_passes(contour: &Contour, stock_z: f64, entry_start: f64, depths: &[f64], radius: f64, pitch: f64) -> Result<Toolpath> {
    let radius = capped_positive(radius, "helix radius")?;
    let pitch = capped_positive(pitch, "helix pitch")?;
    let bounds = contour.bounding_box()?;
    let cx = (bounds.min().x() + bounds.max().x()) * 0.5;
    let cy = (bounds.min().y() + bounds.max().y()) * 0.5;
    if !cx.is_finite() || !cy.is_finite() {
        return Err(check::invalid("helix centre is not finite"));
    }
    let mut pen = Pen::new();
    // The same pitch covers the air above the stock when the tool starts at the retract height.
    let start = helix_point(cx, cy, radius, 0.0, entry_start)?;
    pen.rapid(start)?;
    pen.event(ProcessEvent::On)?;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        let z = pen.z()?;
        pen.linear(helix_point(cx, cy, radius, 0.0, z)?, CutKind::LeadIn)?;
        descend_helix(&mut pen, cx, cy, radius, pitch, target)?;
        cut_contour(&mut pen, contour, target)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn descend_helix(pen: &mut Pen, cx: f64, cy: f64, radius: f64, pitch: f64, z_target: f64) -> Result<()> {
    let mut guard = 0;
    loop {
        let z = pen.z()?;
        let remain = z - z_target;
        if remain <= 1e-9 {
            break;
        }
        guard += 1;
        if guard > MAX_LAPS {
            return Err(check::invalid(format!("too many helix turns: limit is {MAX_LAPS}")));
        }
        if remain >= pitch - 1e-9 {
            let next = z - pitch;
            let to = helix_point(cx, cy, radius, 0.0, next)?;
            let centre = Point3::new(cx, cy, z)?;
            pen.arc(to, centre, true, CutKind::LeadIn)?;
        } else {
            let sweep = std::f64::consts::TAU * (remain / pitch);
            if !sweep.is_finite() || sweep <= 0.0 || sweep > std::f64::consts::TAU + 1e-6 {
                return Err(check::invalid("helix sweep is not usable"));
            }
            let to = helix_point(cx, cy, radius, sweep, z_target)?;
            let centre = Point3::new(cx, cy, z)?;
            pen.arc(to, centre, true, CutKind::LeadIn)?;
            break;
        }
    }
    Ok(())
}

fn helix_point(cx: f64, cy: f64, radius: f64, angle: f64, z: f64) -> Result<Point3> {
    Point3::new(cx + radius * angle.cos(), cy + radius * angle.sin(), z)
}

fn ramp_passes(
    contour: &Contour,
    length: f64,
    stock_z: f64,
    entry_start: f64,
    depths: &[f64],
    max_angle: f64,
    tolerance: &Tolerance,
) -> Result<Toolpath> {
    let max_angle = finite(max_angle, "ramp angle")?;
    if max_angle <= 0.0 || max_angle >= std::f64::consts::FRAC_PI_2 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let slope = max_angle.tan();
    if !slope.is_finite() || slope <= 0.0 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let eps = tolerance.linear().max(1e-9);
    let mut pen = Pen::new();
    let start = first_point(contour)?;
    // Length is `(pen.z - target) / tan(angle)`. Starting here, not on the stock, keeps that
    // length matched to the drop the post will actually cut.
    pen.rapid(Point3::new(start.x(), start.y(), entry_start)?)?;
    pen.event(ProcessEvent::On)?;
    let mut s = 0.0;
    for depth in depths {
        let target = pass_height(stock_z, *depth)?;
        s = descend_along(&mut pen, contour, length, s, target, slope, eps)?;
        s = flat_lap(&mut pen, contour, length, s, target, eps)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn descend_along(pen: &mut Pen, contour: &Contour, length: f64, mut s: f64, z_target: f64, slope: f64, eps: f64) -> Result<f64> {
    let mut laps = 0;
    loop {
        let z = pen.z()?;
        if z <= z_target + eps {
            break;
        }
        laps += 1;
        if laps > MAX_LAPS {
            return Err(check::invalid(format!("too many ramp turns: limit is {MAX_LAPS}")));
        }
        if s >= length - eps {
            s = 0.0;
        }
        let room = length - s;
        if room <= eps {
            return Err(check::invalid("ramp span is empty"));
        }
        let need = (z - z_target) / slope;
        if !need.is_finite() {
            return Err(check::invalid("ramp length is not finite"));
        }
        if need <= 1e-9 {
            break;
        }
        let step = room.min(need);
        if step <= 1e-9 {
            return Err(check::invalid("ramp span is empty"));
        }
        let s1 = (s + step).min(length);
        let before = z;
        emit_sloped(pen, contour, s, s1, z_target, slope, CutKind::LeadIn)?;
        let after = pen.z()?;
        if after > before - 1e-12 {
            return Err(check::invalid("ramp did not descend"));
        }
        s = if s1 >= length - eps { 0.0 } else { s1 };
    }
    Ok(s)
}

fn flat_lap(pen: &mut Pen, contour: &Contour, length: f64, s: f64, z: f64, eps: f64) -> Result<f64> {
    if s <= eps || length - s <= eps {
        emit_at_height(pen, contour, 0.0, length, z, CutKind::Cut)?;
        return Ok(0.0);
    }
    emit_at_height(pen, contour, s, length, z, CutKind::Cut)?;
    emit_at_height(pen, contour, 0.0, s, z, CutKind::Cut)?;
    Ok(s)
}

/// Descend at most `slope` millimetres of Z per millimetre of path, and not below `z_floor`.
fn emit_sloped(pen: &mut Pen, contour: &Contour, s0: f64, s1: f64, z_floor: f64, slope: f64, kind: CutKind) -> Result<()> {
    let z0 = pen.z()?;
    let segs = spans(contour, s0, s1)?;
    let total = span_length(&segs)?;
    let drop_limit = (z0 - z_floor).max(0.0);
    let drop = (slope * total).min(drop_limit);
    let z1 = z0 - drop;
    emit_pieces(pen, &segs, z0, z1, kind)
}

fn emit_at_height(pen: &mut Pen, contour: &Contour, s0: f64, s1: f64, z: f64, kind: CutKind) -> Result<()> {
    let segs = spans(contour, s0, s1)?;
    let z0 = pen.z()?;
    emit_pieces(pen, &segs, z0, z, kind)
}

fn spans(contour: &Contour, s0: f64, s1: f64) -> Result<Vec<Segment>> {
    if !s0.is_finite() || !s1.is_finite() || s1 < s0 - 1e-9 {
        return Err(check::invalid("ramp span is empty"));
    }
    let segs = crate::curve::subdivide(contour, s0, s1)?;
    if segs.is_empty() {
        return Err(check::invalid("ramp span is empty"));
    }
    Ok(segs)
}

fn span_length(segs: &[Segment]) -> Result<f64> {
    let mut total = 0.0;
    for seg in segs {
        let len = seg.length();
        if !len.is_finite() || len < 0.0 {
            return Err(check::invalid("segment length is not finite"));
        }
        total += len;
    }
    if total <= 1e-15 {
        return Err(check::invalid("ramp span is empty"));
    }
    if !total.is_finite() {
        return Err(check::invalid("ramp span is not finite"));
    }
    Ok(total)
}

fn emit_pieces(pen: &mut Pen, segs: &[Segment], z0: f64, z1: f64, kind: CutKind) -> Result<()> {
    let total = span_length(segs)?;
    if !z1.is_finite() {
        return Err(check::invalid("pass height is not finite"));
    }
    let mut acc = 0.0;
    for seg in segs {
        let len = seg.length();
        acc += len;
        let mut z = z0 + (z1 - z0) * (acc / total);
        if (acc - total).abs() <= 1e-9 {
            z = z1;
        }
        if !z.is_finite() {
            return Err(check::invalid("pass height is not finite"));
        }
        emit_segment(pen, *seg, z, kind)?;
    }
    Ok(())
}

fn emit_segment(pen: &mut Pen, seg: Segment, z: f64, kind: CutKind) -> Result<()> {
    match seg {
        Segment::Line(line) => {
            pen.linear(Point3::new(line.end().x(), line.end().y(), z)?, kind)?;
        }
        Segment::Arc(arc) => {
            let to = Point3::new(arc.end_point().x(), arc.end_point().y(), z)?;
            let centre = Point3::new(arc.center().x(), arc.center().y(), z)?;
            pen.arc(to, centre, arc.sweep() > 0.0, kind)?;
        }
    }
    Ok(())
}

fn cut_contour(pen: &mut Pen, contour: &Contour, z: f64) -> Result<()> {
    let start = first_point(contour)?;
    pen.linear(Point3::new(start.x(), start.y(), z)?, CutKind::LeadIn)?;
    for seg in contour.segments() {
        emit_segment(pen, *seg, z, CutKind::Cut)?;
    }
    Ok(())
}

fn pass_height(stock_z: f64, depth: f64) -> Result<f64> {
    let z = stock_z - depth;
    finite(z, "pass height")
}

fn first_point(contour: &Contour) -> Result<Point> {
    contour.segments().first().map(|seg| seg.start()).ok_or_else(|| check::invalid("contour is empty"))
}

fn capped_positive(value: f64, what: &str) -> Result<f64> {
    let value = positive(value, what)?;
    if value > MAX_MM {
        return Err(check::invalid(format!("{what} is larger than {MAX_MM} mm")));
    }
    Ok(value)
}

fn capped_abs(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value.abs() > MAX_MM {
        return Err(check::invalid(format!("{what} is larger than {MAX_MM} mm")));
    }
    Ok(value)
}

/// One compensated contour with its bridges placed from the cut start.
struct TabCut {
    contour: Contour,
    tabs: Vec<Tab>,
    length: f64,
    /// Z at which a partial bridge is crossed. Full-depth tabs use the stock Z so every pass lifts.
    tab_top: f64,
    full: bool,
    retract: f64,
    stock_z: f64,
}

impl TabCut {
    /// A pass at `z` cuts into a bridge when it is strictly below the tab top.
    fn applies(&self, z: f64) -> bool {
        z < self.tab_top - 1e-9
    }
}

fn tabbed_toolpath(contour: &Contour, plan: &DepthPlan, tolerance: &Tolerance, entry_start: f64) -> Result<Toolpath> {
    let cut = prepare_tabs(contour, plan, tolerance)?;
    let depths = pass_depths(plan.total_depth, plan.step_down)?;
    let floor = cut.stock_z - plan.total_depth;
    finite(floor, "final depth")?;
    match plan.entry {
        DepthEntry::Ramp { max_angle } => tabbed_ramp(&cut, entry_start, &depths, max_angle, tolerance),
        DepthEntry::Helix { radius, pitch } => tabbed_helix(&cut, entry_start, &depths, radius, pitch),
        DepthEntry::Plunge => tabbed_plunge(&cut, &depths),
    }
}

fn prepare_tabs(contour: &Contour, plan: &DepthPlan, tolerance: &Tolerance) -> Result<TabCut> {
    let Some(requested) = plan.tabs else {
        return Err(check::invalid("tabs are missing"));
    };
    let stock_z = capped_abs(plan.stock_z, "stock z")?;
    let retract = finite(requested.retract(), "retract")?;
    if retract.abs() > MAX_MM {
        return Err(check::invalid(format!("retract is larger than {MAX_MM} mm")));
    }
    let op_depth = finite(requested.depth(), "tab depth")?;
    if op_depth <= 0.0 || op_depth > MAX_MM {
        return Err(check::invalid("tab depth must be positive"));
    }
    let mut tabs = layout_tabs(contour, requested.spec(), 0.0)?;
    let contour = move_entry_past_tab(contour, &mut tabs, tolerance)?;
    let length = contour.length()?;
    if !length.is_finite() || length <= tolerance.linear() {
        return Err(check::invalid("contour length is too small"));
    }
    let height = requested.spec().tab_height().filter(|height| *height > 0.0);
    let (full, tab_top) = match height {
        None => (true, stock_z),
        Some(height) => {
            if height >= op_depth {
                return Err(check::invalid("tab height must be less than depth"));
            }
            let top = stock_z - (op_depth - height);
            (false, finite(top, "tab top")?)
        }
    };
    Ok(TabCut { contour, tabs, length, tab_top, full, retract, stock_z })
}

fn tabbed_plunge(cut: &TabCut, depths: &[f64]) -> Result<Toolpath> {
    let mut pen = Pen::new();
    let mut cursor = Cursor::at_start(cut)?;
    let start = cursor.point(cut)?;
    pen.rapid(Point3::new(start.x(), start.y(), cut.stock_z)?)?;
    pen.event(ProcessEvent::On)?;
    for depth in depths {
        let target = pass_height(cut.stock_z, *depth)?;
        cursor = plunge_pass(&mut pen, cut, cursor, target)?;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn plunge_pass(pen: &mut Pen, cut: &TabCut, mut cursor: Cursor, target: f64) -> Result<Cursor> {
    if cut.applies(target) {
        cursor = leave_tab(pen, cut, cursor)?;
    }
    let xy = cursor.point(cut)?;
    pen.linear(Point3::new(xy.x(), xy.y(), target)?, CutKind::Plunge)?;
    if cut.contour.is_closed() {
        cut_pass(pen, cut, cursor.s, cut.length, target)?;
        if cursor.s > 1e-6 {
            cut_pass(pen, cut, 0.0, cursor.s, target)?;
        }
        Ok(Cursor { s: cursor.s, forward: true })
    } else {
        cover_open(pen, cut, cursor, target)
    }
}

fn tabbed_ramp(cut: &TabCut, entry_start: f64, depths: &[f64], max_angle: f64, tolerance: &Tolerance) -> Result<Toolpath> {
    let slope = ramp_slope(max_angle)?;
    let eps = tolerance.linear().max(1e-9);
    let mut pen = Pen::new();
    let mut cursor = Cursor::at_start(cut)?;
    let start = cursor.point(cut)?;
    // The posted feed starts at this height. Sizing the slope from the stock instead would
    // make the first descent steeper than max_angle. A span that meets a tab below the bridge
    // lifts across the bridge; it does not keep descending through the gap.
    pen.rapid(Point3::new(start.x(), start.y(), entry_start)?)?;
    pen.event(ProcessEvent::On)?;
    let walk = Walk { cut, slope, eps };
    for depth in depths {
        let target = pass_height(cut.stock_z, *depth)?;
        cursor = walk.ramp_pass(&mut pen, cursor, target)?;
        cursor = if cut.contour.is_closed() {
            Cursor { s: flat_closed(&mut pen, cut, cursor.s, target, eps)?, forward: true }
        } else {
            cover_open(&mut pen, cut, cursor, target)?
        };
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn ramp_slope(max_angle: f64) -> Result<f64> {
    let max_angle = finite(max_angle, "ramp angle")?;
    if max_angle <= 0.0 || max_angle >= std::f64::consts::FRAC_PI_2 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let slope = max_angle.tan();
    if !slope.is_finite() || slope <= 0.0 {
        return Err(check::invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    Ok(slope)
}

fn tabbed_helix(cut: &TabCut, entry_start: f64, depths: &[f64], radius: f64, pitch: f64) -> Result<Toolpath> {
    let radius = capped_positive(radius, "helix radius")?;
    let pitch = capped_positive(pitch, "helix pitch")?;
    let bounds = cut.contour.bounding_box()?;
    let cx = (bounds.min().x() + bounds.max().x()) * 0.5;
    let cy = (bounds.min().y() + bounds.max().y()) * 0.5;
    if !cx.is_finite() || !cy.is_finite() {
        return Err(check::invalid("helix centre is not finite"));
    }
    let mut pen = Pen::new();
    let mut cursor = Cursor::at_start(cut)?;
    // The same pitch covers the air above the stock, so the first turn is not steeper.
    pen.rapid(helix_point(cx, cy, radius, 0.0, entry_start)?)?;
    pen.event(ProcessEvent::On)?;
    let mut started = false;
    for depth in depths {
        let target = pass_height(cut.stock_z, *depth)?;
        if started && !cut.contour.is_closed() {
            let at = pen.at.ok_or_else(|| check::invalid("toolpath has no position"))?;
            let up = cut.retract.max(at.z());
            pen.linear(Point3::new(at.x(), at.y(), up)?, CutKind::Retract)?;
            pen.rapid(helix_point(cx, cy, radius, 0.0, up)?)?;
        }
        let z = pen.z()?;
        pen.linear(helix_point(cx, cy, radius, 0.0, z)?, CutKind::LeadIn)?;
        descend_helix(&mut pen, cx, cy, radius, pitch, target)?;
        cursor = join_and_cut(&mut pen, cut, cursor, target)?;
        started = true;
    }
    pen.event(ProcessEvent::Off)?;
    pen.finish()
}

fn join_and_cut(pen: &mut Pen, cut: &TabCut, mut cursor: Cursor, target: f64) -> Result<Cursor> {
    if cut.applies(target) {
        cursor = skip_entry(cut, cursor);
    }
    let xy = cursor.point(cut)?;
    pen.linear(Point3::new(xy.x(), xy.y(), target)?, CutKind::LeadIn)?;
    if cut.contour.is_closed() {
        cut_pass(pen, cut, cursor.s, cut.length, target)?;
        if cursor.s > 1e-6 {
            cut_pass(pen, cut, 0.0, cursor.s, target)?;
        }
        Ok(cursor)
    } else {
        cover_open(pen, cut, cursor, target)
    }
}

/// Arc length and the direction of the next open-contour pass.
struct Cursor {
    s: f64,
    forward: bool,
}

impl Cursor {
    fn at_start(cut: &TabCut) -> Result<Self> {
        let mut cursor = Self { s: 0.0, forward: true };
        cursor = skip_entry(cut, cursor);
        let _ = cursor.point(cut)?;
        Ok(cursor)
    }

    fn point(&self, cut: &TabCut) -> Result<Point> {
        crate::curve::point_at_length(&cut.contour, self.s)
    }
}

/// Move an entry that lands inside a bridge to the far side of that bridge.
fn skip_entry(cut: &TabCut, mut cursor: Cursor) -> Cursor {
    if let Some(leave) = covering_tab_end(&cut.tabs, cursor.s, cursor.forward) {
        cursor.s = leave;
    }
    cursor
}

fn leave_tab(pen: &mut Pen, cut: &TabCut, cursor: Cursor) -> Result<Cursor> {
    let Some(leave) = covering_tab_end(&cut.tabs, cursor.s, cursor.forward) else {
        return Ok(cursor);
    };
    cross_gap(pen, cut, cursor.s, leave)?;
    Ok(Cursor { s: leave, forward: cursor.forward })
}

/// Feed up, follow the bridge, and return to the Z the ramp or plunge was at.
fn cross_gap(pen: &mut Pen, cut: &TabCut, s_from: f64, s_to: f64) -> Result<()> {
    if (s_from - s_to).abs() <= 1e-9 {
        return Ok(());
    }
    let z = pen.z()?;
    let mut moves = Vec::new();
    if cut.full {
        emit_crossing(&cut.contour, s_from, s_to, z, None, cut.retract, &mut moves)?;
    } else if z + 1e-9 >= cut.tab_top {
        emit_tabbed_span(&cut.contour, s_from, s_to, z, CutKind::LeadIn, &[], cut.retract, &mut moves)?;
    } else {
        let lift = cut.tab_top - z;
        emit_crossing(&cut.contour, s_from, s_to, z, Some(lift), cut.retract, &mut moves)?;
    }
    play(pen, &moves)
}

fn cut_pass(pen: &mut Pen, cut: &TabCut, s_from: f64, s_to: f64, z: f64) -> Result<()> {
    let mut moves = Vec::new();
    let tabs: &[Tab] = if cut.applies(z) { &cut.tabs } else { &[] };
    emit_tabbed_span(&cut.contour, s_from, s_to, z, CutKind::Cut, tabs, cut.retract, &mut moves)?;
    play(pen, &moves)
}

/// Cut the rest of an open contour at `z` and leave the tool at the far end, ready to reverse.
fn cover_open(pen: &mut Pen, cut: &TabCut, cursor: Cursor, z: f64) -> Result<Cursor> {
    let end = if cursor.forward { cut.length } else { 0.0 };
    let origin = if cursor.forward { 0.0 } else { cut.length };
    cut_pass(pen, cut, cursor.s, end, z)?;
    if (cursor.s - origin).abs() > 1e-6 {
        cut_pass(pen, cut, end, origin, z)?;
        cut_pass(pen, cut, origin, end, z)?;
    }
    Ok(Cursor { s: end, forward: !cursor.forward })
}

fn flat_closed(pen: &mut Pen, cut: &TabCut, s: f64, z: f64, eps: f64) -> Result<f64> {
    if !cut.applies(z) {
        return flat_lap(pen, &cut.contour, cut.length, s, z, eps);
    }
    if s <= eps || cut.length - s <= eps {
        cut_pass(pen, cut, 0.0, cut.length, z)?;
        return Ok(0.0);
    }
    cut_pass(pen, cut, s, cut.length, z)?;
    cut_pass(pen, cut, 0.0, s, z)?;
    Ok(s)
}

struct Walk<'a> {
    cut: &'a TabCut,
    slope: f64,
    eps: f64,
}

impl Walk<'_> {
    fn ramp_pass(&self, pen: &mut Pen, mut cursor: Cursor, target: f64) -> Result<Cursor> {
        let closed = self.cut.contour.is_closed();
        if self.cut.applies(target) && pen.z()? > self.cut.tab_top + self.eps {
            cursor = if closed {
                Cursor { s: descend_along(pen, &self.cut.contour, self.cut.length, cursor.s, self.cut.tab_top, self.slope, self.eps)?, forward: true }
            } else {
                self.descend(pen, cursor, self.cut.tab_top, false)?
            };
        }
        if !self.cut.applies(target) {
            cursor.s = if closed {
                descend_along(pen, &self.cut.contour, self.cut.length, cursor.s, target, self.slope, self.eps)?
            } else {
                return self.descend(pen, cursor, target, false);
            };
            return Ok(cursor);
        }
        if closed {
            cursor.s = self.descend(pen, Cursor { s: cursor.s, forward: true }, target, true)?.s;
            Ok(Cursor { s: cursor.s, forward: true })
        } else {
            self.descend(pen, cursor, target, true)
        }
    }

    fn descend(&self, pen: &mut Pen, mut cursor: Cursor, z_target: f64, avoid: bool) -> Result<Cursor> {
        let wrap = self.cut.contour.is_closed();
        let mut laps = 0;
        loop {
            let z = pen.z()?;
            if z <= z_target + self.eps {
                break;
            }
            laps += 1;
            if laps > MAX_LAPS {
                return Err(check::invalid(format!("too many ramp turns: limit is {MAX_LAPS}")));
            }
            if wrap {
                if cursor.forward && cursor.s >= self.cut.length - self.eps {
                    cursor.s = 0.0;
                } else if !cursor.forward && cursor.s <= self.eps {
                    cursor.s = self.cut.length;
                }
            }
            if avoid && let Some(leave) = covering_tab_end(&self.cut.tabs, cursor.s, cursor.forward) {
                cross_gap(pen, self.cut, cursor.s, leave)?;
                cursor.s = leave;
                continue;
            }
            let mut room = if cursor.forward { (self.cut.length - cursor.s).max(0.0) } else { cursor.s.max(0.0) };
            if avoid {
                room = room.min(room_to_tab(&self.cut.tabs, cursor.s, cursor.forward, self.cut.length));
            }
            if room <= self.eps {
                if wrap {
                    cursor.s = if cursor.forward { 0.0 } else { self.cut.length };
                    continue;
                }
                cursor.forward = !cursor.forward;
                continue;
            }
            let need = (z - z_target) / self.slope;
            if !need.is_finite() {
                return Err(check::invalid("ramp length is not finite"));
            }
            if need <= 1e-9 {
                break;
            }
            let step = room.min(need);
            if step <= 1e-9 {
                return Err(check::invalid("ramp span is empty"));
            }
            let s1 = if cursor.forward { (cursor.s + step).min(self.cut.length) } else { (cursor.s - step).max(0.0) };
            let before = z;
            emit_sloped_between(pen, &self.cut.contour, cursor.s, s1, z_target, self.slope, CutKind::LeadIn)?;
            if pen.z()? > before - 1e-12 {
                return Err(check::invalid("ramp did not descend"));
            }
            cursor.s = s1;
        }
        Ok(cursor)
    }
}

fn emit_sloped_between(pen: &mut Pen, contour: &Contour, s0: f64, s1: f64, z_floor: f64, slope: f64, kind: CutKind) -> Result<()> {
    if s1 + 1e-9 >= s0 {
        return emit_sloped(pen, contour, s0, s1, z_floor, slope, kind);
    }
    let z0 = pen.z()?;
    let segs = backward_spans(contour, s0, s1)?;
    let total = span_length(&segs)?;
    let drop = (slope * total).min((z0 - z_floor).max(0.0));
    emit_pieces(pen, &segs, z0, z0 - drop, kind)
}

fn backward_spans(contour: &Contour, s_from: f64, s_to: f64) -> Result<Vec<Segment>> {
    let segs = spans(contour, s_to, s_from)?;
    Ok(segs.into_iter().rev().map(Segment::reversed).collect())
}

fn play(pen: &mut Pen, moves: &[Move]) -> Result<()> {
    for mv in moves {
        match *mv {
            Move::Rapid { to } => pen.rapid(to)?,
            Move::Linear { to, kind } => pen.linear(to, kind)?,
            Move::Arc { to, centre, ccw, kind } => pen.arc(to, centre, ccw, kind)?,
            Move::Event(_) | Move::Dwell { .. } => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_geom::{Line, Point, Segment};
    use proptest::prelude::*;

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-6, "{a} != {b}");
    }

    fn rect(w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![edge(p(0.0, 0.0), p(w, 0.0)), edge(p(w, 0.0), p(w, h)), edge(p(w, h), p(0.0, h)), edge(p(0.0, h), p(0.0, 0.0))],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    #[test]
    fn five_millimetres_at_two_millimetre_step_is_two_four_five() {
        let depths = pass_depths(5.0, 2.0).unwrap();
        assert_eq!(depths, vec![2.0, 4.0, 5.0]);
        assert!(depths.iter().all(|depth| *depth <= 5.0));
        assert_eq!(depths.last().copied(), Some(5.0));

        // A step that lands on the total does not add another pass past it.
        assert_eq!(pass_depths(4.0, 2.0).unwrap(), vec![2.0, 4.0]);
        assert_eq!(pass_depths(2.0, 2.0).unwrap(), vec![2.0]);
        assert_eq!(pass_depths(2.0, 5.0).unwrap(), vec![2.0]);
        assert_eq!(pass_depths(6.0, 3.0).unwrap(), vec![3.0, 6.0]);

        let shape = rect(10.0, 10.0);
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Plunge);
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        let mut plunges = Vec::new();
        for mv in path.moves() {
            if let Move::Linear { to, kind: CutKind::Plunge } = *mv {
                plunges.push(to.z());
                assert!(to.z() >= -5.0 - 1e-9);
            }
        }
        assert_eq!(plunges.len(), 3);
        near(plunges[0], -2.0);
        near(plunges[1], -4.0);
        near(plunges[2], -5.0);
    }

    #[test]
    fn ramp_angle_is_never_exceeded() {
        let shape = rect(30.0, 10.0);
        let max_angle = 0.1_f64.atan();
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Ramp { max_angle });
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();

        let mut cursor: Option<Point3> = None;
        let mut steepest = 0.0_f64;
        let mut cut_z = Vec::new();
        for mv in path.moves() {
            let next = match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let (Some(from), Some(to)) = (cursor, next) {
                let horiz = (to.x() - from.x()).hypot(to.y() - from.y());
                let dz = (to.z() - from.z()).abs();
                if horiz > 1e-12 || dz > 1e-12 {
                    let angle = dz.atan2(horiz);
                    assert!(angle <= max_angle + 1e-6, "angle {angle} exceeds {max_angle}");
                    steepest = steepest.max(angle);
                }
            }
            if let Some(to) = next {
                if let Move::Linear { kind: CutKind::Cut, .. } = *mv {
                    cut_z.push(to.z());
                    assert!(to.z() >= -5.0 - 1e-6, "cut below the total depth at {}", to.z());
                }
                cursor = Some(to);
            }
        }
        assert!(steepest + 1e-6 >= max_angle * 0.5, "ramp did not use the allowed angle");
        for target in [-2.0, -4.0, -5.0] {
            assert!(cut_z.iter().any(|z| (*z - target).abs() < 1e-6), "missing cut at {target}: {cut_z:?}");
        }
    }

    #[test]
    fn helix_descends_by_pitch_per_turn() {
        let shape = rect(10.0, 10.0);
        let pitch = 2.0;
        let plan = DepthPlan::new(0.0, 5.0, 2.0, DepthEntry::Helix { radius: 1.0, pitch });
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();

        match path.moves().first() {
            Some(Move::Rapid { to }) => {
                near(to.x(), 6.0);
                near(to.y(), 5.0);
                near(to.z(), 0.0);
            }
            other => panic!("expected a rapid onto the helix, got {other:?}"),
        }

        let mut cursor: Option<Point3> = None;
        let mut full_drops = Vec::new();
        let mut arc_drop = 0.0;
        for mv in path.moves() {
            match *mv {
                Move::Arc { to, centre, ccw, kind: CutKind::LeadIn } => {
                    let from = cursor.expect("arc has a start");
                    assert!(ccw);
                    near(centre.x(), 5.0);
                    near(centre.y(), 5.0);
                    let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                    near(radius, 1.0);
                    near((to.x() - centre.x()).hypot(to.y() - centre.y()), 1.0);
                    let drop = from.z() - to.z();
                    assert!(drop >= -1e-9, "helix climbed");
                    let chord = (to.x() - from.x()).hypot(to.y() - from.y());
                    if chord <= 1e-6 {
                        full_drops.push(drop);
                    } else {
                        assert!(drop < pitch + 1e-8, "partial turn dropped {drop}, pitch is {pitch}");
                    }
                    arc_drop += drop;
                    cursor = Some(to);
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        assert_eq!(full_drops.len(), 2, "{full_drops:?}");
        for drop in full_drops {
            near(drop, pitch);
        }
        near(arc_drop, 5.0);
    }

    #[test]
    fn hostile_depth_input_is_an_error() {
        let shape = rect(10.0, 4.0);
        let tol = Tolerance::default();
        for (total, step) in [
            (f64::NAN, 2.0),
            (5.0, f64::NAN),
            (5.0, f64::INFINITY),
            (f64::NEG_INFINITY, 1.0),
            (0.0, 2.0),
            (-5.0, 2.0),
            (5.0, 0.0),
            (5.0, -1.0),
            (1.0e7, 1.0),
            (100.0, 1.0e-4),
        ] {
            assert!(pass_depths(total, step).is_err(), "accepted {total}, {step}");
        }

        let plunge = DepthEntry::Plunge;
        assert!(depth_toolpath(&shape, &DepthPlan::new(f64::NAN, 1.0, 1.0, plunge), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, plunge).entry_start(-0.5), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, plunge).entry_start(f64::NAN), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: 0.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: -0.2 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: std::f64::consts::FRAC_PI_2 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Ramp { max_angle: f64::NAN }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: 0.0, pitch: 1.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: -1.0, pitch: 1.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: 1.0, pitch: 0.0 }), &tol).is_err());
        assert!(depth_toolpath(&shape, &DepthPlan::new(0.0, 1.0, 1.0, DepthEntry::Helix { radius: f64::INFINITY, pitch: 1.0 }), &tol).is_err());

        let open =
            Contour::try_new(vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(5.0, 0.0).unwrap()).unwrap())], false, &tol)
                .unwrap();
        assert!(depth_toolpath(&open, &DepthPlan::new(0.0, 1.0, 1.0, plunge), &tol).is_err());

        // A shallow angle on a short contour would spiral for far too many laps.
        let tiny = DepthPlan::new(0.0, 10.0, 10.0, DepthEntry::Ramp { max_angle: 1e-4 });
        let err = depth_toolpath(&shape, &tiny, &tol).unwrap_err();
        assert!(err.to_string().contains("ramp"), "{err}");

        let fine = DepthPlan::new(0.0, 10.0, 10.0, DepthEntry::Helix { radius: 1.0, pitch: 1.0e-4 });
        let err = depth_toolpath(&shape, &fine, &tol).unwrap_err();
        assert!(err.to_string().contains("helix"), "{err}");
    }

    /// A ramp planned from Z 0 and then started at the retract height used to drop 4 mm over
    /// the 2 mm length (`2 / tan 5°` = 22.86 mm), which is 9.92°. The length has to cover the
    /// whole drop from the retract height.
    #[test]
    fn ramp_from_the_retract_height_is_not_steeper_than_the_angle() {
        let shape = rect(100.0, 60.0);
        let angle = 5.0_f64.to_radians();
        let plan = DepthPlan::new(0.0, 6.0, 2.0, DepthEntry::Ramp { max_angle: angle }).entry_start(2.0);
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        assert_descent_bound(&path, angle, -6.0, &[-2.0, -4.0, -6.0]);
        assert_lead_stays_on(&path, &shape);

        let mut cursor: Option<Point3> = None;
        let mut first_len = None;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor
                && matches!(*mv, Move::Linear { kind: CutKind::LeadIn, .. })
                && to.z() < from.z() - 1e-9
            {
                first_len = Some((to.x() - from.x()).hypot(to.y() - from.y()));
                near(from.z(), 2.0);
                break;
            }
            cursor = Some(to);
        }
        let len = first_len.expect("first ramp");
        // Drop from Z 2 to Z -2, not the 2 mm stock step.
        near(len, 4.0 / angle.tan());
    }

    #[test]
    fn ramp_along_short_edges_wraps_corners_without_leaving_the_contour() {
        let shape = short_polygon(40.0, 28.0, 4.0);
        let angle = 5.0_f64.to_radians();
        let plan = DepthPlan::new(0.0, 6.0, 2.0, DepthEntry::Ramp { max_angle: angle }).entry_start(2.0);
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        assert_descent_bound(&path, angle, -6.0, &[-2.0, -4.0, -6.0]);
        assert_lead_stays_on(&path, &shape);

        let mut segments = 0;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if matches!(*mv, Move::Linear { kind: CutKind::LeadIn, .. }) {
                segments += 1;
            }
            if to.z() <= -2.0 + 1e-6 {
                break;
            }
        }
        assert!(segments >= 4, "ramp did not wrap corners, lead-in segments {segments}");
    }

    #[test]
    fn helix_from_above_the_stock_keeps_its_pitch() {
        let shape = rect(30.0, 20.0);
        let radius = 3.0;
        let angle = 5.0_f64.to_radians();
        let pitch = std::f64::consts::TAU * radius * angle.tan();
        let plan = DepthPlan::new(0.0, 6.0, 2.0, DepthEntry::Helix { radius, pitch }).entry_start(2.0);
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        assert_descent_bound(&path, angle, -6.0, &[-2.0, -4.0, -6.0]);

        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor
                && let Move::Arc { centre, .. } = *mv
                && to.z() < from.z() - 1e-9
            {
                let sweep = xy_sweep(from, to, centre, true);
                let drop = from.z() - to.z();
                // A full turn drops one pitch. A partial turn drops the same fraction of it.
                // Starting at the retract height must not pile that gap onto the first turn.
                if sweep >= std::f64::consts::TAU - 1e-6 {
                    near(drop, pitch);
                } else {
                    near(drop, pitch * sweep / std::f64::consts::TAU);
                }
            }
            cursor = Some(to);
        }
    }

    fn motion_end(mv: Move) -> Option<Point3> {
        match mv {
            Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
            Move::Event(_) | Move::Dwell { .. } => None,
        }
    }

    fn xy_sweep(from: Point3, to: Point3, centre: Point3, ccw: bool) -> f64 {
        let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
        let a1 = (to.y() - centre.y()).atan2(to.x() - centre.x());
        let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
        let chord = (to.x() - from.x()).hypot(to.y() - from.y());
        let closure = 1e-9_f64.max(radius * 1e-9);
        let ccw_delta = (a1 - a0).rem_euclid(std::f64::consts::TAU);
        if chord <= closure {
            std::f64::consts::TAU
        } else if ccw {
            if ccw_delta <= 1e-15 { std::f64::consts::TAU } else { ccw_delta }
        } else {
            let cw = std::f64::consts::TAU - ccw_delta;
            if cw <= 1e-15 { std::f64::consts::TAU } else { cw }
        }
    }

    fn travel_xy(from: Point3, mv: Move) -> f64 {
        match mv {
            Move::Linear { to, .. } | Move::Rapid { to } => (to.x() - from.x()).hypot(to.y() - from.y()),
            Move::Arc { to, centre, ccw, .. } => {
                let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                radius * xy_sweep(from, to, centre, ccw)
            }
            Move::Event(_) | Move::Dwell { .. } => 0.0,
        }
    }

    fn assert_descent_bound(path: &Toolpath, max_angle: f64, final_z: f64, levels: &[f64]) {
        let mut cursor: Option<Point3> = None;
        let mut saw = false;
        let mut zs = Vec::new();
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            zs.push(to.z());
            if let Some(from) = cursor
                && !matches!(*mv, Move::Rapid { .. })
                && to.z() < from.z() - 1e-9
            {
                let horiz = travel_xy(from, *mv);
                let angle = (from.z() - to.z()).atan2(horiz);
                assert!(angle <= max_angle + 1e-6, "angle {angle} rad exceeds {max_angle} from {} to {}", from.z(), to.z());
                saw = true;
            }
            assert!(to.z() >= final_z - 1e-6, "below the floor at {}", to.z());
            cursor = Some(to);
        }
        assert!(saw, "entry did not descend");
        for level in levels {
            assert!(zs.iter().any(|z| (*z - level).abs() < 1e-6), "missing level {level}: {zs:?}");
        }
    }

    fn assert_lead_stays_on(path: &Toolpath, contour: &Contour) {
        let tol = Tolerance::default();
        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor
                && matches!(*mv, Move::Linear { kind: CutKind::LeadIn, .. } | Move::Arc { kind: CutKind::LeadIn, .. })
            {
                for (x, y) in lead_samples(from, *mv) {
                    let point = Point::new(x, y).unwrap();
                    let place = contour.containment(point, &tol).unwrap();
                    assert!(matches!(place, campanion_geom::Containment::Boundary), "ramp left the contour at ({x}, {y})");
                }
            }
            cursor = Some(to);
        }
    }

    fn lead_samples(from: Point3, mv: Move) -> Vec<(f64, f64)> {
        match mv {
            Move::Linear { to, .. } => vec![((from.x() + to.x()) * 0.5, (from.y() + to.y()) * 0.5), (to.x(), to.y())],
            Move::Arc { to, centre, ccw, .. } => {
                let radius = (from.x() - centre.x()).hypot(from.y() - centre.y());
                let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
                let sweep = xy_sweep(from, to, centre, ccw);
                let signed = if ccw { sweep * 0.5 } else { -sweep * 0.5 };
                let mid = a0 + signed;
                vec![(centre.x() + radius * mid.cos(), centre.y() + radius * mid.sin()), (to.x(), to.y())]
            }
            Move::Rapid { .. } | Move::Event(_) | Move::Dwell { .. } => Vec::new(),
        }
    }

    fn short_polygon(w: f64, h: f64, step: f64) -> Contour {
        let mut pts = Vec::new();
        push_run(&mut pts, 0.0, 0.0, w, 0.0, step);
        push_run(&mut pts, w, 0.0, w, h, step);
        push_run(&mut pts, w, h, 0.0, h, step);
        push_run(&mut pts, 0.0, h, 0.0, 0.0, step);
        let p = |xy: (f64, f64)| Point::new(xy.0, xy.1).unwrap();
        let mut segs = Vec::new();
        for window in pts.windows(2) {
            segs.push(Segment::Line(Line::new(p(window[0]), p(window[1])).unwrap()));
        }
        let first = pts[0];
        let last = pts[pts.len() - 1];
        segs.push(Segment::Line(Line::new(p(last), p(first)).unwrap()));
        Contour::try_new(segs, true, &Tolerance::default()).unwrap()
    }

    fn push_run(pts: &mut Vec<(f64, f64)>, x0: f64, y0: f64, x1: f64, y1: f64, step: f64) {
        let len = (x1 - x0).hypot(y1 - y0);
        let n = (len / step).round().max(1.0) as usize;
        for i in 0..n {
            let t = i as f64 / n as f64;
            pts.push((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t));
        }
    }

    fn circle(radius: f64) -> Contour {
        let centre = Point::new(0.0, 0.0).unwrap();
        let arc = |start| Segment::Arc(campanion_geom::Arc::new(centre, radius, start, std::f64::consts::PI).unwrap());
        Contour::try_new(vec![arc(0.0), arc(std::f64::consts::PI)], true, &Tolerance::default()).unwrap()
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(40))]

        #[test]
        fn random_entries_from_above_the_stock_stay_within_the_angle(
            entry_start in 0.0f64..=20.0,
            angle_deg in 1.0f64..=30.0,
            depth in 1.0f64..=6.0,
            step_frac in 0.25f64..=1.25,
            shape in 0..4u32,
        ) {
            let angle = angle_deg.to_radians();
            let step = (depth * step_frac).max(0.25);
            let levels = pass_depths(depth, step).unwrap();
            let zs: Vec<f64> = levels.iter().map(|level| -level).collect();
            let final_z = -depth;
            let tol = Tolerance::default();
            match shape {
                0 => {
                    let contour = rect(80.0, 50.0);
                    let plan = DepthPlan::new(0.0, depth, step, DepthEntry::Ramp { max_angle: angle }).entry_start(entry_start);
                    let path = depth_toolpath(&contour, &plan, &tol).unwrap();
                    assert_descent_bound(&path, angle, final_z, &zs);
                    assert_lead_stays_on(&path, &contour);
                }
                1 => {
                    let contour = circle(30.0);
                    let plan = DepthPlan::new(0.0, depth, step, DepthEntry::Ramp { max_angle: angle }).entry_start(entry_start);
                    let path = depth_toolpath(&contour, &plan, &tol).unwrap();
                    assert_descent_bound(&path, angle, final_z, &zs);
                    assert_lead_stays_on(&path, &contour);
                }
                2 => {
                    let contour = short_polygon(60.0, 40.0, 5.0);
                    let plan = DepthPlan::new(0.0, depth, step, DepthEntry::Ramp { max_angle: angle }).entry_start(entry_start);
                    let path = depth_toolpath(&contour, &plan, &tol).unwrap();
                    assert_descent_bound(&path, angle, final_z, &zs);
                    assert_lead_stays_on(&path, &contour);
                }
                _ => {
                    let contour = rect(80.0, 50.0);
                    let radius = 4.0;
                    let pitch = std::f64::consts::TAU * radius * angle.tan();
                    let plan = DepthPlan::new(0.0, depth, step, DepthEntry::Helix { radius, pitch }).entry_start(entry_start);
                    let path = depth_toolpath(&contour, &plan, &tol).unwrap();
                    assert_descent_bound(&path, angle, final_z, &zs);
                }
            }
        }
    }

    fn xy_travel_at(path: &Toolpath, z: f64) -> f64 {
        let mut cursor: Option<Point3> = None;
        let mut total = 0.0;
        for mv in path.moves() {
            let next = match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let (Some(from), Some(to)) = (cursor, next) {
                let rapid = matches!(mv, Move::Rapid { .. });
                if !rapid && (from.z() - z).abs() < 1e-6 && (to.z() - z).abs() < 1e-6 {
                    total += (to.x() - from.x()).hypot(to.y() - from.y());
                }
                if rapid && to.z() < 2.0 - 1e-6 && (to.x() - from.x()).hypot(to.y() - from.y()) > 1e-6 {
                    panic!("XY rapid below retract at {to:?}");
                }
            }
            if next.is_some() {
                cursor = next;
            }
        }
        total
    }

    #[test]
    fn tabs_spare_passes_above_the_bridge_and_follow_the_contour() {
        let shape = rect(80.0, 40.0);
        let spec = crate::TabSpec::count(4, 8.0).height(3.0);
        let plan = DepthPlan::new(0.0, 12.0, 4.0, DepthEntry::Plunge).with_tabs(crate::DepthTabs::new(spec, 12.0, 2.0));
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        let shallow = xy_travel_at(&path, -4.0);
        let mid = xy_travel_at(&path, -8.0);
        let bridge = xy_travel_at(&path, -9.0);
        let floor = xy_travel_at(&path, -12.0);
        assert!(shallow > 200.0, "shallow pass {shallow}");
        assert!(mid > 200.0, "mid pass {mid}");
        assert!(bridge > 30.0 && bridge < 40.0, "four 8 mm bridges, got {bridge}");
        assert!(floor > 150.0, "floor pass {floor}");
        assert!((shallow - mid).abs() < 1e-6, "passes above the tab differ: {shallow} {mid}");
    }

    #[test]
    fn full_depth_tabs_rapid_at_retract_and_open_contours_reverse() {
        let shape = rect(40.0, 20.0);
        let spec = crate::TabSpec::count(2, 6.0).height(0.0);
        let plan = DepthPlan::new(0.0, 6.0, 6.0, DepthEntry::Plunge).with_tabs(crate::DepthTabs::new(spec, 6.0, 2.0));
        let path = depth_toolpath(&shape, &plan, &Tolerance::default()).unwrap();
        let _ = xy_travel_at(&path, -6.0);
        let mut saw = false;
        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            let next = match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let (Some(from), Some(to)) = (cursor, next)
                && matches!(mv, Move::Rapid { .. })
                && (to.z() - 2.0).abs() < 1e-6
                && (to.x() - from.x()).hypot(to.y() - from.y()) > 1e-4
            {
                saw = true;
            }
            cursor = next.or(cursor);
        }
        assert!(saw, "full-depth tab should rapid at retract");

        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(100.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let spec = crate::TabSpec::count(2, 5.0).height(2.0);
        let plan = DepthPlan::new(0.0, 4.0, 2.0, DepthEntry::Plunge).with_tabs(crate::DepthTabs::new(spec, 4.0, 2.0));
        let path = depth_toolpath(&open, &plan, &Tolerance::default()).unwrap();
        // Tab top is -2, so the -2 pass is whole and the -4 pass lifts along that same Z.
        assert!(xy_travel_at(&path, -2.0) > 100.0, "open pass at the tab top");
        assert!(xy_travel_at(&path, -4.0) > 80.0, "open floor");
        assert!(depth_toolpath(&open, &DepthPlan::new(0.0, 4.0, 2.0, DepthEntry::Plunge), &Tolerance::default()).is_err());
    }

    #[test]
    fn ramp_and_helix_keep_the_bridge() {
        let shape = rect(60.0, 30.0);
        let spec = crate::TabSpec::count(3, 6.0).height(2.0);
        let ramp =
            DepthPlan::new(0.0, 8.0, 8.0, DepthEntry::Ramp { max_angle: 10.0_f64.to_radians() }).with_tabs(crate::DepthTabs::new(spec, 8.0, 2.0));
        let path = depth_toolpath(&shape, &ramp, &Tolerance::default()).unwrap();
        assert!(xy_travel_at(&path, -6.0) > 15.0, "ramp bridge");
        assert!(xy_travel_at(&path, -8.0) > 100.0, "ramp floor");

        let helix = DepthPlan::new(0.0, 8.0, 8.0, DepthEntry::Helix { radius: 2.0, pitch: 4.0 }).with_tabs(crate::DepthTabs::new(spec, 8.0, 2.0));
        let path = depth_toolpath(&shape, &helix, &Tolerance::default()).unwrap();
        assert!(xy_travel_at(&path, -6.0) > 15.0, "helix bridge");
        assert!(path.moves().iter().any(|mv| matches!(mv, Move::Arc { .. })), "helix arc");
    }

    /// A profile ramp from the retract height, with tabs in the way.
    ///
    /// The first drop is sized from that height, so every descent that travels in XY stays within
    /// `angle_deg`. Where that ramp would cross a tab below the bridge, the existing tab behaviour
    /// wins: the span is followed at the bridge top, and the only steeper descent is the vertical
    /// plunge at the tab edge. Nothing inside the span is cut below the bridge top.
    #[test]
    fn ramp_from_retract_with_tabs_keeps_the_bridge_and_the_angle() {
        // 40 by 10, perimeter 100. From Z 2 at 5°, the bridge top (Z -6) is about 91 mm along,
        // just past the last tab. The rest of the drop meets the first tab while still below it.
        let shape = rect(40.0, 10.0);
        let angle = 5.0_f64.to_radians();
        let spec = crate::TabSpec::count(4, 6.0).height(2.0);
        let depth = 8.0;
        let entry_z = 2.0;
        let tab_top = -(depth - 2.0);
        let floor = -depth;
        let tol = Tolerance::default();
        let mut tabs = super::layout_tabs(&shape, spec, 0.0).unwrap();
        let contour = super::move_entry_past_tab(&shape, &mut tabs, &tol).unwrap();

        let ramp = DepthPlan::new(0.0, depth, depth, DepthEntry::Ramp { max_angle: angle })
            .entry_start(entry_z)
            .with_tabs(crate::DepthTabs::new(spec, depth, entry_z));
        let path = depth_toolpath(&shape, &ramp, &tol).unwrap();
        assert_bridge_and_angle(&path, &contour, &tabs, tab_top, angle, entry_z, &tol);
        let drop = entry_z - tab_top;
        let along = xy_until(&path, tab_top);
        assert!((along - drop / angle.tan()).abs() < 1e-3, "ramp to the bridge was {along}, drop {drop}");
        assert!(intermediate_tab_plunge(&path, tab_top, floor), "ramp never met a tab below the bridge");

        let radius = 2.0;
        let pitch = std::f64::consts::TAU * radius * angle.tan();
        let helix = DepthPlan::new(0.0, depth, depth, DepthEntry::Helix { radius, pitch })
            .entry_start(entry_z)
            .with_tabs(crate::DepthTabs::new(spec, depth, entry_z));
        let path = depth_toolpath(&shape, &helix, &tol).unwrap();
        assert_bridge_and_angle(&path, &contour, &tabs, tab_top, angle, entry_z, &tol);
        assert!(path.moves().iter().any(|mv| matches!(mv, Move::Arc { .. })), "helix arc");
    }

    fn contour_s(contour: &Contour, point: Point3, tol: &Tolerance) -> Option<f64> {
        let at = Point::new(point.x(), point.y()).ok()?;
        let loc = crate::curve::closest_location(contour, at, tol).ok()?;
        if loc.point.distance(at) > 1e-3 {
            return None;
        }
        let mut s = 0.0;
        for (index, seg) in contour.segments().iter().copied().enumerate() {
            let len = seg.length();
            if index == loc.index {
                return Some(s + len * loc.t.clamp(0.0, 1.0));
            }
            s += len;
        }
        None
    }

    fn tab_interior(tabs: &[crate::Tab], s: f64) -> bool {
        tabs.iter().any(|tab| {
            let start = tab.start();
            let end = start + tab.width();
            s > start + 1e-3 && s < end - 1e-3
        })
    }

    fn on_tab_edge(tabs: &[crate::Tab], s: f64) -> bool {
        tabs.iter().any(|tab| {
            let start = tab.start();
            let end = start + tab.width();
            (s - start).abs() <= 1e-3 || (s - end).abs() <= 1e-3
        })
    }

    /// No sample inside a tab span is below the bridge. XY descents stay within `max_angle`.
    /// A vertical plunge is the existing tab return, and it sits on a tab edge.
    fn assert_bridge_and_angle(path: &Toolpath, contour: &Contour, tabs: &[crate::Tab], tab_top: f64, max_angle: f64, entry_z: f64, tol: &Tolerance) {
        let mut cursor: Option<Point3> = None;
        let mut saw_descent = false;
        let mut saw_bridge = false;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor {
                if !matches!(*mv, Move::Rapid { .. }) && to.z() < from.z() - 1e-9 {
                    let horiz = travel_xy(from, *mv);
                    if horiz > 1e-4 {
                        let angle = (from.z() - to.z()).atan2(horiz);
                        assert!(angle <= max_angle + 1e-5, "descent {angle} rad from {} to {} exceeds {max_angle}", from.z(), to.z());
                        if !saw_descent {
                            assert!((from.z() - entry_z).abs() < 1e-6, "first descent starts at {} not {entry_z}", from.z());
                            saw_descent = true;
                        }
                    } else if let Some(s) = contour_s(contour, to, tol) {
                        assert!(on_tab_edge(tabs, s), "vertical descent at s {s} is not a tab edge, z {} to {}", from.z(), to.z());
                    } else {
                        panic!("vertical descent is off the contour at {to:?}");
                    }
                }
                for sample in [from, to] {
                    if let Some(s) = contour_s(contour, sample, tol)
                        && tab_interior(tabs, s)
                    {
                        assert!(sample.z() >= tab_top - 1e-4, "point inside a tab at s {s} is below the bridge: z {}", sample.z());
                        if (sample.z() - tab_top).abs() < 1e-3 {
                            saw_bridge = true;
                        }
                    }
                }
                if let (Some(s0), Some(s1)) = (contour_s(contour, from, tol), contour_s(contour, to, tol)) {
                    let along = travel_xy(from, *mv);
                    if (along - (s1 - s0).abs()).abs() <= 1e-2 || along <= 1e-4 {
                        saw_bridge |= span_stays_above(s0, from.z(), s1, to.z(), tabs, tab_top);
                    }
                }
            }
            cursor = Some(to);
        }
        assert!(saw_descent, "entry did not descend");
        assert!(saw_bridge, "tab bridge was not followed");
    }

    /// True when this span feeds the bridge interior at the bridge top.
    fn span_stays_above(s0: f64, z0: f64, s1: f64, z1: f64, tabs: &[crate::Tab], tab_top: f64) -> bool {
        if (s1 - s0).abs() <= 1e-9 {
            if tab_interior(tabs, s0) {
                assert!(z0.min(z1) >= tab_top - 1e-4, "vertical cut inside a tab at s {s0}, z {z0} {z1}");
            }
            return false;
        }
        let (a, za, b, zb) = if s0 <= s1 { (s0, z0, s1, z1) } else { (s1, z1, s0, z0) };
        let mut bridge = false;
        for tab in tabs {
            let lo = (tab.start() + 1e-3).max(a);
            let hi = (tab.start() + tab.width() - 1e-3).min(b);
            if hi <= lo {
                continue;
            }
            let z_at = |s: f64| {
                let t = (s - a) / (b - a);
                za + (zb - za) * t
            };
            let zmin = z_at(lo).min(z_at(hi));
            assert!(zmin >= tab_top - 1e-4, "cut below the bridge inside a tab at s {lo}..{hi}, z {zmin} < {tab_top}");
            if (z_at(lo) - tab_top).abs() < 1e-3 && (z_at(hi) - tab_top).abs() < 1e-3 {
                bridge = true;
            }
        }
        bridge
    }

    fn xy_until(path: &Toolpath, z_stop: f64) -> f64 {
        let mut cursor: Option<Point3> = None;
        let mut total = 0.0;
        let mut done = false;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor
                && !done
                && !matches!(*mv, Move::Rapid { .. })
                && to.z() < from.z() - 1e-9
                && from.z() > z_stop + 1e-6
            {
                let horiz = travel_xy(from, *mv);
                if to.z() >= z_stop - 1e-6 {
                    total += horiz;
                } else {
                    let frac = (from.z() - z_stop) / (from.z() - to.z());
                    total += horiz * frac;
                }
                if to.z() <= z_stop + 1e-6 {
                    done = true;
                }
            }
            cursor = Some(to);
        }
        total
    }

    fn intermediate_tab_plunge(path: &Toolpath, tab_top: f64, floor: f64) -> bool {
        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            let Some(to) = motion_end(*mv) else {
                continue;
            };
            if let Some(from) = cursor
                && matches!(*mv, Move::Linear { kind: CutKind::Plunge, .. })
                && travel_xy(from, *mv) <= 1e-4
                && (from.z() - tab_top).abs() < 1e-4
                && to.z() < tab_top - 0.05
                && to.z() > floor + 0.05
            {
                return true;
            }
            cursor = Some(to);
        }
        false
    }
}
