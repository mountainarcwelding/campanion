//! Pocket clearing inside a [`campanion_geom::Region`].
//!
//! The region is one outer boundary plus any island holes. The tool centre stays at least the
//! tool radius (or half the kerf) inside that boundary, and the same distance off every island.
//!
//! [`PocketStrategy::OffsetRings`] cuts successive inward offsets through [`offset_region`].
//! The first offset is the tool radius plus the finishing allowance. Each later offset moves in
//! by at most one stepover. A band between the outer wall and an island that is narrower than
//! two stepovers still gets a ring, so those passes stay within one stepover. A corner the full
//! step leaves past the tool, and a remainder left when an inset splits, gets a shorter pass.
//! A solid remainder thinner than two tool radii, whose middle is farther than the tool radius
//! from the last ring, gets one more pass. A wider remainder that is still thinner than a stepover
//! stays on the current ring: stepping by the tool radius would be one ring per radius and overflows
//! the stack on a large off-grid disk. The ring count is capped by the region span divided by half
//! a stepover. Rings are emitted outside-in ([`RingOrder::OutsideIn`]).
//! Two rings are joined by a straight cut when that segment stays inside the region the tool
//! centre may enter. Otherwise the tool retracts and enters the next ring again. A plunge entry
//! re-enters with a plunge. A ramp follows that ring for one circuit, sized for the full drop
//! from the retract height, and fails when the circuit is shorter than the ramp. A helix is a
//! circle about the ring's bounding-box centre and fails when the circle or the feed onto the
//! ring leaves the pocket. Neither ramp nor helix is replaced by a plunge.
//!
//! [`PocketStrategy::Zigzag`] fills the inset region with parallel lines at `angle` radians from
//! +X, spaced by the stepover and clipped to that region. Successive lines alternate direction.
//!
//! [`PocketOp::finish`] appends a contour on the tool-radius offset so a positive allowance is
//! cleared afterwards. With offset rings and a zero allowance that contour is already the first
//! ring, so the extra pass is skipped.
//!
//! Corner clean-up is not implemented. [`PocketOp::corner_cleanup`] makes [`PocketOp::generate`]
//! return [`crate::PathError::NotSupported`].

use campanion_geom::{
    Arc, BooleanOp, BoundingBox, Containment, Contour, Line, Point, Region, Segment, Tolerance, Vec2, boolean_regions, cmp_float_key, offset_region,
};

use crate::check::{self, finite, non_negative, positive};
use crate::tool::Tool;
use crate::toolpath::{CutKind, Move, ProcessEvent, Toolpath};
use crate::{DepthEntry, DepthPlan, PathError, Result, depth_toolpath};

/// Upper bound on offset levels and on zigzag lines. A hostile stepover fails instead of looping.
const MAX_PASSES: usize = 1024;

/// Offset curves, including a shorter pass that closes a band under two stepovers.
const MAX_RING_LOOPS: usize = MAX_PASSES * 2;

/// Extra levels above `span / min_inset` for the boundary ring and one core pass.
const RING_CAP_SLACK: usize = 2;

/// Orphan patches thinner than this are offset noise, not an uncut band.
const ORPHAN_CLEARANCE_MM: f64 = 1.0e-3;

/// Largest accepted stepover or finishing allowance, in millimetres.
const MAX_DISTANCE_MM: f64 = 1.0e6;

/// How the pocket is cleared.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PocketStrategy {
    /// Successive inward offsets, largest ring first.
    OffsetRings,
    /// Parallel lines at `angle` radians from +X, alternating direction.
    Zigzag { angle: f64 },
}

/// Order of offset rings. This crate emits [`RingOrder::OutsideIn`] only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RingOrder {
    /// Largest offset first, then each step inward.
    OutsideIn,
}

/// Parameters for one pocket.
#[derive(Clone, Debug, PartialEq)]
pub struct PocketOp {
    tool: Tool,
    strategy: PocketStrategy,
    stepover: f64,
    allowance: f64,
    finish: bool,
    corner_cleanup: bool,
    z: f64,
    retract_z: f64,
    tolerance: Tolerance,
    entry: DepthEntry,
}

impl PocketOp {
    /// `stepover` is the distance between rings, or between zigzag lines, in millimetres.
    /// It is checked when the operation runs.
    pub fn new(tool: Tool, strategy: PocketStrategy, stepover: f64) -> Self {
        Self {
            tool,
            strategy,
            stepover,
            allowance: 0.0,
            finish: false,
            corner_cleanup: false,
            z: 0.0,
            retract_z: 0.0,
            tolerance: Tolerance::default(),
            entry: DepthEntry::Plunge,
        }
    }

    /// How the tool enters each depth, and any later ring that cannot be reached at depth.
    ///
    /// [`DepthEntry::Plunge`] feeds straight down. A ramp follows that ring for one circuit
    /// and fails when the drop from the retract height needs a longer path. A helix circles
    /// the ring's bounding-box centre and fails when that circle or the feed onto the ring
    /// leaves the pocket.
    pub fn entry(mut self, entry: DepthEntry) -> Self {
        self.entry = entry;
        self
    }

    /// Extra stock left on walls and islands during roughing, in millimetres.
    ///
    /// Added to the tool radius for the first offset and for the zigzag clip. A finishing
    /// pass removes it.
    pub fn allowance(mut self, allowance: f64) -> Self {
        self.allowance = allowance;
        self
    }

    /// Cut a final contour at the tool radius after roughing.
    pub fn finish(mut self, enabled: bool) -> Self {
        self.finish = enabled;
        self
    }

    /// Request a corner clean-up pass.
    ///
    /// Clean-up is not implemented. [`Self::generate`] returns [`PathError::NotSupported`]
    /// when this is set.
    pub fn corner_cleanup(mut self, enabled: bool) -> Self {
        self.corner_cleanup = enabled;
        self
    }

    /// Cut height. Z is up, so a pocket below the stock surface is negative.
    pub fn z(mut self, z: f64) -> Self {
        self.z = z;
        self
    }

    /// Retract height for links that cannot stay inside the pocket. Must be at least `z`.
    pub fn retract_z(mut self, z: f64) -> Self {
        self.retract_z = z;
        self
    }

    pub fn tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Build the pocket toolpath for `pocket`.
    pub fn generate(&self, pocket: &Region) -> Result<Pocket> {
        let z = finite(self.z, "z")?;
        let retract_z = finite(self.retract_z, "retract z")?;
        if retract_z + 1e-9 < z {
            return Err(invalid("retract z is below the cut"));
        }
        let stepover = positive(self.stepover, "stepover")?;
        if stepover > MAX_DISTANCE_MM {
            return Err(invalid(format!("stepover is larger than {MAX_DISTANCE_MM} mm")));
        }
        let allowance = non_negative(self.allowance, "finishing allowance")?;
        if allowance > MAX_DISTANCE_MM {
            return Err(invalid(format!("finishing allowance is larger than {MAX_DISTANCE_MM} mm")));
        }
        let radius = self.tool.compensation_distance()?;
        if let PocketStrategy::Zigzag { angle } = self.strategy {
            finite(angle, "zigzag angle")?;
        }
        if self.corner_cleanup {
            return Err(PathError::NotSupported("corner clean-up".into()));
        }
        let span = outer_span(pocket)?;
        if span / stepover > MAX_PASSES as f64 {
            let what = match self.strategy {
                PocketStrategy::OffsetRings => "offset rings",
                PocketStrategy::Zigzag { .. } => "zigzag lines",
            };
            return Err(invalid(format!("too many {what}: limit is {MAX_PASSES}")));
        }

        let rough = radius + allowance;
        if !rough.is_finite() {
            return Err(invalid("offset distance is not finite"));
        }
        let eps = linear_eps(&self.tolerance);
        let clearance = non_empty(offset_region(pocket, radius, &self.tolerance)?)?;
        let clip = if allowance <= eps { clearance.clone() } else { non_empty(offset_region(pocket, rough, &self.tolerance)?)? };

        let mut pieces = Vec::new();
        let rings = match self.strategy {
            PocketStrategy::OffsetRings => {
                let loops = offset_rings(&clip, stepover, radius, &self.tolerance)?;
                let count = loops.len();
                pieces.extend(loops.into_iter().map(CutPiece::Loop));
                count
            }
            PocketStrategy::Zigzag { angle } => {
                let spans = zigzag_spans(angle, stepover, &clip, &self.tolerance)?;
                pieces.extend(spans.into_iter().map(|(start, end)| CutPiece::Span(start, end)));
                0
            }
        };
        // Offset rings with no allowance already cut the tool-radius contour first.
        let duplicate_wall = matches!(self.strategy, PocketStrategy::OffsetRings) && allowance <= eps;
        if self.finish && !duplicate_wall {
            let mut wall = Vec::new();
            append_loops(&clearance, &mut wall, &self.tolerance)?;
            if wall.is_empty() {
                return Err(invalid("tool does not fit in the pocket"));
            }
            pieces.extend(wall.into_iter().map(CutPiece::Loop));
        }
        if pieces.is_empty() {
            return Err(invalid("pocket produced no toolpath"));
        }

        let mut pen = Pen::new(z, retract_z);
        emit(&mut pen, &pieces, &clearance, &self.tolerance, self.entry)?;
        Ok(Pocket { toolpath: Toolpath::new(pen.moves)?, rings, ring_order: RingOrder::OutsideIn })
    }
}

/// A finished pocket.
#[derive(Clone, Debug, PartialEq)]
pub struct Pocket {
    toolpath: Toolpath,
    rings: usize,
    ring_order: RingOrder,
}

impl Pocket {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Closed offset rings that were cut, not counting a separate finishing contour.
    ///
    /// Zigzag leaves this at zero. The order is [`Self::ring_order`].
    pub fn ring_count(&self) -> usize {
        self.rings
    }

    pub fn ring_order(&self) -> RingOrder {
        self.ring_order
    }
}

enum CutPiece {
    Loop(Contour),
    Span(Point, Point),
}

struct Pen {
    moves: Vec<Move>,
    xy: Option<Point>,
    z_now: f64,
    cut_z: f64,
    retract_z: f64,
}

impl Pen {
    fn new(cut_z: f64, retract_z: f64) -> Self {
        Self { moves: Vec::new(), xy: None, z_now: retract_z, cut_z, retract_z }
    }
}

fn emit(pen: &mut Pen, pieces: &[CutPiece], clearance: &[Region], tolerance: &Tolerance, entry: DepthEntry) -> Result<()> {
    if matches!(entry, DepthEntry::Plunge) {
        return emit_plunge(pen, pieces, clearance, tolerance);
    }
    emit_shaped(pen, pieces, clearance, tolerance, entry)
}

fn emit_plunge(pen: &mut Pen, pieces: &[CutPiece], clearance: &[Region], tolerance: &Tolerance) -> Result<()> {
    let Some(first) = pieces.first() else {
        return Err(invalid("pocket produced no toolpath"));
    };
    let start = piece_start(first)?;
    rapid_above(pen, start)?;
    check::push(&mut pen.moves, Move::Event(ProcessEvent::On))?;
    plunge(pen)?;
    emit_body(pen, first)?;
    for piece in pieces.iter().skip(1) {
        link(pen, piece_start(piece)?, clearance, tolerance)?;
        emit_body(pen, piece)?;
    }
    finish_pen(pen)
}

fn emit_shaped(pen: &mut Pen, pieces: &[CutPiece], clearance: &[Region], tolerance: &Tolerance, entry: DepthEntry) -> Result<()> {
    if let DepthEntry::Ramp { max_angle } = entry {
        ramp_slope(max_angle)?;
    }
    let Some(first) = pieces.first() else {
        return Err(invalid("pocket produced no toolpath"));
    };
    emit_entry(pen, first, clearance, tolerance, entry, false)?;
    for piece in pieces.iter().skip(1) {
        link_shaped(pen, piece, clearance, tolerance, entry)?;
    }
    finish_pen(pen)
}

fn finish_pen(pen: &mut Pen) -> Result<()> {
    check::push(&mut pen.moves, Move::Event(ProcessEvent::Off))?;
    if (pen.z_now - pen.retract_z).abs() > 1e-12 {
        let xy = pen.xy.ok_or_else(|| invalid("toolpath has no position"))?;
        let up = check::point3(xy, pen.retract_z)?;
        check::push(&mut pen.moves, Move::Linear { to: up, kind: CutKind::Retract })?;
        pen.z_now = pen.retract_z;
    }
    Ok(())
}

fn piece_start(piece: &CutPiece) -> Result<Point> {
    match piece {
        CutPiece::Loop(contour) => contour.segments().first().map(|seg| seg.start()).ok_or_else(|| invalid("contour is empty")),
        CutPiece::Span(start, _) => Ok(*start),
    }
}

fn emit_body(pen: &mut Pen, piece: &CutPiece) -> Result<()> {
    match piece {
        CutPiece::Loop(contour) => {
            for seg in contour.segments() {
                check::push_segment(*seg, pen.cut_z, CutKind::Cut, &mut pen.moves)?;
                pen.xy = Some(seg.end());
                pen.z_now = pen.cut_z;
            }
        }
        CutPiece::Span(_, end) => {
            let dest = check::point3(*end, pen.cut_z)?;
            check::push(&mut pen.moves, Move::Linear { to: dest, kind: CutKind::Cut })?;
            pen.xy = Some(*end);
            pen.z_now = pen.cut_z;
        }
    }
    Ok(())
}

fn rapid_above(pen: &mut Pen, to: Point) -> Result<()> {
    let up = check::point3(to, pen.retract_z)?;
    check::push(&mut pen.moves, Move::Rapid { to: up })?;
    pen.xy = Some(to);
    pen.z_now = pen.retract_z;
    Ok(())
}

fn plunge(pen: &mut Pen) -> Result<()> {
    if (pen.retract_z - pen.cut_z).abs() <= 1e-12 {
        return Ok(());
    }
    let xy = pen.xy.ok_or_else(|| invalid("toolpath has no position"))?;
    let down = check::point3(xy, pen.cut_z)?;
    check::push(&mut pen.moves, Move::Linear { to: down, kind: CutKind::Plunge })?;
    pen.z_now = pen.cut_z;
    Ok(())
}

/// Straight cut when the segment stays in `clearance`. Otherwise retract, rapid, and plunge.
fn link(pen: &mut Pen, to: Point, clearance: &[Region], tolerance: &Tolerance) -> Result<()> {
    let Some(from) = pen.xy else {
        rapid_above(pen, to)?;
        return plunge(pen);
    };
    let eps = linear_eps(tolerance);
    if from.almost_eq(to, eps) && (pen.z_now - pen.cut_z).abs() <= eps {
        pen.xy = Some(to);
        return Ok(());
    }
    if (pen.z_now - pen.cut_z).abs() <= eps && stays_inside(from, to, clearance, tolerance)? {
        let dest = check::point3(to, pen.cut_z)?;
        check::push(&mut pen.moves, Move::Linear { to: dest, kind: CutKind::Cut })?;
        pen.xy = Some(to);
        pen.z_now = pen.cut_z;
        return Ok(());
    }
    if (pen.z_now - pen.retract_z).abs() > eps {
        let up = check::point3(from, pen.retract_z)?;
        check::push(&mut pen.moves, Move::Linear { to: up, kind: CutKind::Retract })?;
        pen.z_now = pen.retract_z;
    }
    rapid_above(pen, to)?;
    plunge(pen)
}

/// At-depth link when the segment stays in the pocket. Otherwise retract and use `entry` again.
fn link_shaped(pen: &mut Pen, piece: &CutPiece, clearance: &[Region], tolerance: &Tolerance, entry: DepthEntry) -> Result<()> {
    let to = piece_start(piece)?;
    let eps = linear_eps(tolerance);
    if let Some(from) = pen.xy {
        let at_depth = (pen.z_now - pen.cut_z).abs() <= eps;
        if at_depth && from.almost_eq(to, eps) {
            pen.xy = Some(to);
            return emit_body(pen, piece);
        }
        if at_depth && stays_inside(from, to, clearance, tolerance)? {
            let dest = check::point3(to, pen.cut_z)?;
            check::push(&mut pen.moves, Move::Linear { to: dest, kind: CutKind::Cut })?;
            pen.xy = Some(to);
            pen.z_now = pen.cut_z;
            return emit_body(pen, piece);
        }
    }
    retract_up(pen)?;
    emit_entry(pen, piece, clearance, tolerance, entry, true)
}

fn emit_entry(pen: &mut Pen, piece: &CutPiece, clearance: &[Region], tolerance: &Tolerance, entry: DepthEntry, already_on: bool) -> Result<()> {
    match entry {
        DepthEntry::Plunge => Err(invalid("plunge entry is not a shaped entry")),
        DepthEntry::Ramp { max_angle } => {
            let start = piece_start(piece)?;
            rapid_at_retract(pen, start)?;
            if !already_on {
                check::push(&mut pen.moves, Move::Event(ProcessEvent::On))?;
            }
            ramp_piece(pen, piece, max_angle, tolerance)
        }
        DepthEntry::Helix { radius, pitch } => helix_piece(pen, piece, clearance, tolerance, radius, pitch, already_on),
    }
}

fn ramp_slope(max_angle: f64) -> Result<f64> {
    let max_angle = finite(max_angle, "ramp angle")?;
    if max_angle <= 0.0 || max_angle >= std::f64::consts::FRAC_PI_2 {
        return Err(invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    let slope = max_angle.tan();
    if !slope.is_finite() || slope <= 0.0 {
        return Err(invalid("ramp angle must be greater than 0 and less than a right angle"));
    }
    Ok(slope)
}

/// Ramp along one circuit of `piece`. The slope is the full drop from the retract height.
fn ramp_piece(pen: &mut Pen, piece: &CutPiece, max_angle: f64, tolerance: &Tolerance) -> Result<()> {
    let drop = pen.retract_z - pen.cut_z;
    if drop <= linear_eps(tolerance) {
        return emit_body(pen, piece);
    }
    let slope = ramp_slope(max_angle)?;
    let segs = match piece {
        CutPiece::Loop(contour) => contour_segments(contour)?,
        CutPiece::Span(start, end) => vec![Segment::Line(Line::new(*start, *end)?)],
    };
    emit_ramp_segments(pen, &segs, slope)?;
    // A partial arc's end can sit a fraction off the ring start, on the seam where
    // containment misses the radial link back to the next ring. Land on the start.
    snap_to_start(pen, piece, tolerance);
    Ok(())
}

fn snap_to_start(pen: &mut Pen, piece: &CutPiece, tolerance: &Tolerance) {
    let Ok(start) = piece_start(piece) else {
        return;
    };
    let Some(xy) = pen.xy else {
        return;
    };
    if xy.almost_eq(start, linear_eps(tolerance).max(1e-6)) {
        pen.xy = Some(start);
    }
}

fn contour_segments(contour: &Contour) -> Result<Vec<Segment>> {
    let length = contour.length()?;
    if !length.is_finite() || length <= 1e-9 {
        return Err(invalid("ramp is longer than the pocket path"));
    }
    let segs = crate::curve::subdivide(contour, 0.0, length)?;
    if segs.is_empty() {
        return Err(invalid("ramp is longer than the pocket path"));
    }
    Ok(segs)
}

/// Descend at `slope` millimetres of Z per millimetre of path, then cut the rest flat.
///
/// One circuit only. A drop that needs more length is an error, not another lap and not a plunge.
fn emit_ramp_segments(pen: &mut Pen, segs: &[Segment], slope: f64) -> Result<()> {
    let mut total = 0.0;
    for seg in segs {
        let len = seg.length();
        if !len.is_finite() || len < 0.0 {
            return Err(invalid("segment length is not finite"));
        }
        total += len;
    }
    if !total.is_finite() || total <= 1e-12 {
        return Err(invalid("ramp is longer than the pocket path"));
    }
    let drop = pen.z_now - pen.cut_z;
    if drop > slope * total + 1e-6 {
        return Err(invalid("ramp is longer than the pocket path"));
    }
    let target = pen.cut_z;
    let mut index = 0;
    while index < segs.len() && pen.z_now > target + 1e-9 {
        let seg = segs[index];
        let len = seg.length();
        if len <= 1e-12 {
            index += 1;
            continue;
        }
        let need = pen.z_now - target;
        let can = slope * len;
        if can >= need - 1e-9 {
            let travel = (need / slope).min(len);
            if len - travel > 1e-8 {
                let (head, tail) = split_segment(seg, travel / len)?;
                emit_seg(pen, head, target, CutKind::LeadIn)?;
                emit_seg(pen, tail, target, CutKind::Cut)?;
            } else {
                emit_seg(pen, seg, target, CutKind::LeadIn)?;
            }
            index += 1;
            for rest in segs.iter().skip(index) {
                if rest.length() > 1e-12 {
                    emit_seg(pen, *rest, target, CutKind::Cut)?;
                }
            }
            return Ok(());
        }
        let z1 = pen.z_now - can;
        emit_seg(pen, seg, z1, CutKind::LeadIn)?;
        index += 1;
    }
    if pen.z_now > target + 1e-6 {
        return Err(invalid("ramp is longer than the pocket path"));
    }
    for rest in segs.iter().skip(index) {
        if rest.length() > 1e-12 {
            emit_seg(pen, *rest, target, CutKind::Cut)?;
        }
    }
    Ok(())
}

fn split_segment(seg: Segment, t: f64) -> Result<(Segment, Segment)> {
    let t = t.clamp(1e-9, 1.0 - 1e-9);
    match seg {
        Segment::Line(line) => {
            let mid = line.start() + (line.end() - line.start()) * t;
            Ok((Segment::Line(Line::new(line.start(), mid)?), Segment::Line(Line::new(mid, line.end())?)))
        }
        Segment::Arc(arc) => {
            let sweep = arc.sweep();
            let head = Arc::new(arc.center(), arc.radius(), arc.start_angle(), sweep * t)?;
            let tail = Arc::new(arc.center(), arc.radius(), arc.start_angle() + sweep * t, sweep * (1.0 - t))?;
            Ok((Segment::Arc(head), Segment::Arc(tail)))
        }
    }
}

fn emit_seg(pen: &mut Pen, seg: Segment, z: f64, kind: CutKind) -> Result<()> {
    match seg {
        Segment::Line(line) => {
            let to = check::point3(line.end(), z)?;
            check::push(&mut pen.moves, Move::Linear { to, kind })?;
            pen.xy = Some(line.end());
            pen.z_now = z;
        }
        Segment::Arc(arc) => {
            let to = check::point3(arc.end_point(), z)?;
            let centre = check::point3(arc.center(), z)?;
            check::push(&mut pen.moves, Move::Arc { to, centre, ccw: arc.sweep() > 0.0, kind })?;
            pen.xy = Some(arc.end_point());
            pen.z_now = z;
        }
    }
    Ok(())
}

fn helix_piece(
    pen: &mut Pen,
    piece: &CutPiece,
    clearance: &[Region],
    tolerance: &Tolerance,
    radius: f64,
    pitch: f64,
    already_on: bool,
) -> Result<()> {
    let CutPiece::Loop(contour) = piece else {
        return Err(invalid("helix does not fit in the pocket"));
    };
    if !contour.is_closed() {
        return Err(invalid("helix does not fit in the pocket"));
    }
    let drop = pen.retract_z - pen.cut_z;
    if drop <= linear_eps(tolerance) {
        let start = piece_start(piece)?;
        rapid_at_retract(pen, start)?;
        if !already_on {
            check::push(&mut pen.moves, Move::Event(ProcessEvent::On))?;
        }
        return emit_body(pen, piece);
    }
    let plan = DepthPlan::new(pen.retract_z, drop, drop, DepthEntry::Helix { radius, pitch }).entry_start(pen.retract_z);
    let path = depth_toolpath(contour, &plan, tolerance)?;
    if !helix_fits_pocket(&path, clearance, tolerance)? {
        return Err(invalid("helix does not fit in the pocket"));
    }
    if pen.z_now + 1e-9 < pen.retract_z {
        retract_up(pen)?;
    }
    play_entry(pen, &path, already_on)?;
    snap_to_start(pen, piece, tolerance);
    Ok(())
}

/// Lead-in arcs are the helix. The straight feed onto the ring has to stay in the pocket too.
fn helix_fits_pocket(path: &Toolpath, clearance: &[Region], tolerance: &Tolerance) -> Result<bool> {
    let mut cursor: Option<Point> = None;
    let mut saw_arc = false;
    let eps = linear_eps(tolerance);
    for mv in path.moves() {
        match *mv {
            Move::Rapid { to } => cursor = Some(Point::new(to.x(), to.y())?),
            Move::Linear { to, kind } => {
                let end = Point::new(to.x(), to.y())?;
                if kind == CutKind::LeadIn {
                    let inside = match cursor {
                        Some(from) if from.almost_eq(end, eps) => covered(end, clearance, tolerance)?,
                        Some(from) => stays_inside(from, end, clearance, tolerance)?,
                        None => false,
                    };
                    if !inside {
                        return Ok(false);
                    }
                }
                cursor = Some(end);
            }
            Move::Arc { to, centre, ccw, kind } => {
                let end = Point::new(to.x(), to.y())?;
                if kind == CutKind::LeadIn {
                    saw_arc = true;
                    let Some(from) = cursor else {
                        return Ok(false);
                    };
                    let centre_xy = Point::new(centre.x(), centre.y())?;
                    if !arc_covered(from, end, centre_xy, ccw, clearance, tolerance)? {
                        return Ok(false);
                    }
                }
                cursor = Some(end);
            }
            Move::Event(_) | Move::Dwell { .. } => {}
        }
    }
    Ok(saw_arc)
}

fn arc_covered(from: Point, to: Point, centre: Point, ccw: bool, clearance: &[Region], tolerance: &Tolerance) -> Result<bool> {
    let radius = from.distance(centre);
    if !radius.is_finite() || radius < 1e-9 {
        return covered(from, clearance, tolerance);
    }
    let sweep = xy_sweep(from, to, centre, ccw);
    let chord = (8.0 * radius * 1.0e-4).sqrt().clamp(1.0e-3, 0.5);
    let steps_f = (radius * sweep / chord).ceil().clamp(8.0, 8192.0);
    if !steps_f.is_finite() {
        return Ok(false);
    }
    let steps = steps_f as usize;
    let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        let angle = if ccw { a0 + sweep * t } else { a0 - sweep * t };
        let point = Point::new(centre.x() + radius * angle.cos(), centre.y() + radius * angle.sin())?;
        if !covered(point, clearance, tolerance)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn xy_sweep(from: Point, to: Point, centre: Point, ccw: bool) -> f64 {
    let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
    let a1 = (to.y() - centre.y()).atan2(to.x() - centre.x());
    let mut sweep = if ccw { (a1 - a0).rem_euclid(std::f64::consts::TAU) } else { (a0 - a1).rem_euclid(std::f64::consts::TAU) };
    if from.almost_eq(to, 1e-8) || sweep < 1e-8 {
        sweep = std::f64::consts::TAU;
    }
    sweep
}

fn rapid_at_retract(pen: &mut Pen, to: Point) -> Result<()> {
    if pen.z_now + 1e-9 < pen.retract_z {
        return Err(invalid("rapid is below the retract height"));
    }
    let dest = check::point3(to, pen.retract_z)?;
    let same = pen.xy.is_some_and(|xy| xy.almost_eq(to, 1e-9)) && (pen.z_now - pen.retract_z).abs() <= 1e-9;
    if !same {
        check::push(&mut pen.moves, Move::Rapid { to: dest })?;
    }
    pen.xy = Some(to);
    pen.z_now = pen.retract_z;
    Ok(())
}

fn retract_up(pen: &mut Pen) -> Result<()> {
    let Some(xy) = pen.xy else {
        pen.z_now = pen.retract_z;
        return Ok(());
    };
    if (pen.z_now - pen.retract_z).abs() <= 1e-12 {
        return Ok(());
    }
    let up = check::point3(xy, pen.retract_z)?;
    check::push(&mut pen.moves, Move::Linear { to: up, kind: CutKind::Retract })?;
    pen.z_now = pen.retract_z;
    Ok(())
}

fn play_entry(pen: &mut Pen, path: &Toolpath, mut already_on: bool) -> Result<()> {
    for mv in path.moves() {
        match *mv {
            Move::Event(ProcessEvent::Off) => break,
            Move::Event(ProcessEvent::On) => {
                if !already_on {
                    check::push(&mut pen.moves, Move::Event(ProcessEvent::On))?;
                    already_on = true;
                }
            }
            Move::Rapid { to } => {
                if pen.z_now + 1e-9 < pen.retract_z {
                    retract_up(pen)?;
                }
                check::push(&mut pen.moves, Move::Rapid { to })?;
                pen.xy = Some(Point::new(to.x(), to.y())?);
                pen.z_now = to.z();
            }
            Move::Linear { to, kind } => {
                check::push(&mut pen.moves, Move::Linear { to, kind })?;
                pen.xy = Some(Point::new(to.x(), to.y())?);
                pen.z_now = to.z();
            }
            Move::Arc { to, centre, ccw, kind } => {
                check::push(&mut pen.moves, Move::Arc { to, centre, ccw, kind })?;
                pen.xy = Some(Point::new(to.x(), to.y())?);
                pen.z_now = to.z();
            }
            Move::Dwell { seconds } => check::push(&mut pen.moves, Move::Dwell { seconds })?,
        }
    }
    Ok(())
}

fn offset_rings(start: &[Region], stepover: f64, radius: f64, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    let mut loops = Vec::new();
    for region in start {
        cover_region(region, stepover, radius, tolerance, &mut loops)?;
    }
    if loops.is_empty() {
        return Err(invalid("tool does not fit in the pocket"));
    }
    Ok(loops)
}

/// Emit `region`'s loops, then each inward ring, on an explicit stack.
///
/// A full stepover inset moves every boundary, so the gap between an outer wall and an island
/// closes at twice the stepover. When that inset would drop a band that is still wider than one
/// stepover, or a solid core that the tool radius does not reach from the current ring, the next
/// ring is a shorter inset. Later rings use a full stepover again where the remaining region allows it.
///
/// The walk is iterative. A remainder thinner than one stepover used to recurse by the coverage
/// limit (the tool radius on a solid disk). An off-grid disk whose stepover is larger than its
/// radius then built one frame per tool radius and overflowed the stack.
fn cover_region(region: &Region, stepover: f64, radius: f64, tolerance: &Tolerance, loops: &mut Vec<Contour>) -> Result<()> {
    let cap = ring_cap(region, stepover, tolerance)?;
    let started = loops.len();
    let mut pending = vec![region.clone()];
    while let Some(region) = pending.pop() {
        let before = loops.len();
        append_loops(std::slice::from_ref(&region), loops, tolerance)?;
        if loops.len() == before {
            continue;
        }
        if loops.len() - started > cap || loops.len() > MAX_RING_LOOPS {
            let limit = cap.min(MAX_RING_LOOPS);
            return Err(invalid(format!("too many offset rings: limit is {limit}")));
        }
        let mut next = next_insets(&region, stepover, radius, tolerance)?;
        // Push in reverse so the first child is next, matching the old outside-in recursion.
        next.reverse();
        pending.extend(next);
    }
    Ok(())
}

/// Most loops `region` can emit.
///
/// Each pass moves in by at least half a stepover. `span / that inset` is the longest
/// outside-in chain; a core pass and the boundary ring sit in [`RING_CAP_SLACK`]. A walk
/// that keeps shaving a sliver off an off-grid ring hits this and returns an error.
fn ring_cap(region: &Region, stepover: f64, tolerance: &Tolerance) -> Result<usize> {
    let span = outer_span(region)?;
    let min_inset = (stepover * 0.5).max(linear_eps(tolerance));
    let levels = (span / min_inset).ceil();
    if !levels.is_finite() || levels < 0.0 || levels > MAX_RING_LOOPS as f64 {
        return Err(invalid(format!("too many offset rings: limit is {MAX_RING_LOOPS}")));
    }
    let per = region.holes().len().saturating_add(1);
    let cap = (levels as usize).saturating_add(RING_CAP_SLACK).saturating_mul(per);
    Ok(cap.max(per).min(MAX_RING_LOOPS))
}

/// Regions for the next inward ring, or empty when this ring is the last.
fn next_insets(region: &Region, stepover: f64, radius: f64, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let limit = ring_limit(region, stepover, radius);
    let children = inward(region, stepover, tolerance)?;
    if !children.is_empty() && !facing_gap(region, &children, limit, stepover, tolerance)? {
        return Ok(children);
    }
    let step = if children.is_empty() {
        let Some(step) = core_inset(region, limit, stepover, tolerance)? else {
            return Ok(Vec::new());
        };
        step
    } else {
        let step = limit.min(stepover);
        if step <= linear_eps(tolerance) {
            return Ok(children);
        }
        step
    };
    let mids = progressed_insets(region, step, tolerance)?;
    if mids.is_empty() { Ok(children) } else { Ok(mids) }
}

/// Inset for a remainder that a full stepover erases.
///
/// At most two coverage limits thick: one pass of `limit`, unless the middle is already
/// inside the tool (an off-grid offset can leave a sliver just past `limit`). Wider than
/// that, the current ring is already within one stepover of the whole remainder. Another
/// ring every `limit` is the walk that overflowed the stack, and those rings sit too close
/// for a post to tell apart.
fn core_inset(region: &Region, limit: f64, stepover: f64, tolerance: &Tolerance) -> Result<Option<f64>> {
    let eps = linear_eps(tolerance);
    let limit = limit.min(stepover);
    if !(limit > eps) {
        return Ok(None);
    }
    let twice = limit * 2.0;
    if twice < stepover && !inward(region, twice, tolerance)?.is_empty() {
        return Ok(None);
    }
    if inward(region, limit + ORPHAN_CLEARANCE_MM, tolerance)?.is_empty() {
        return Ok(None);
    }
    Ok(Some(limit))
}

fn progressed_insets(region: &Region, step: f64, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut kept = Vec::new();
    for mid in inward(region, step, tolerance)? {
        if smaller(region, &mid, tolerance)? {
            kept.push(mid);
        }
    }
    Ok(kept)
}

/// How far a point may sit from the rings already emitted for this region.
///
/// A solid loop's opposite sides are the same ring. Only the middle has to stay within a tool
/// radius of that ring. An outer loop and an island loop are both cut, and the material between
/// them is twice the clearance, so that clearance must also stay within half a stepover.
fn ring_limit(region: &Region, stepover: f64, radius: f64) -> f64 {
    if region.holes().is_empty() { radius } else { (stepover * 0.5).min(radius) }
}

/// `true` when a full stepover inset leaves a patch that `ring_limit` says still needs a ring.
///
/// The patch is the part of the `limit` inset that a grow of the full-step children does not
/// cover. Corner skins thinner than [`ORPHAN_CLEARANCE_MM`] are offset noise. A sharp corner of
/// that inset is smaller than the area test in region classification, so it is checked on its own.
fn facing_gap(region: &Region, children: &[Region], limit: f64, stepover: f64, tolerance: &Tolerance) -> Result<bool> {
    let eps = linear_eps(tolerance);
    if children.is_empty() || limit + eps >= stepover {
        return Ok(false);
    }
    let deep = inward(region, limit, tolerance)?;
    if deep.is_empty() {
        return Ok(false);
    }
    let reach = stepover - limit;
    let mut shadows = Vec::new();
    for child in children {
        for piece in offset_region(child, -reach, tolerance)? {
            if region_alive(&piece, tolerance)? {
                shadows.push(piece);
            }
        }
    }
    if shadows.is_empty() {
        return Ok(true);
    }
    let pending = subtract_all(&deep, &shadows, tolerance)?;
    for piece in &pending {
        if !inward(piece, ORPHAN_CLEARANCE_MM, tolerance)?.is_empty() {
            return Ok(true);
        }
    }
    // Region classification treats a corner whose area is under `scale * 1e-4` as fully covered
    // and drops it. The vertex of a sharp inset corner still sits past the round grow of the child.
    corners_left_out(&deep, &shadows, tolerance)
}

/// Sharp corners of `deep` that the round grow of the children does not reach.
fn corners_left_out(deep: &[Region], shadows: &[Region], tolerance: &Tolerance) -> Result<bool> {
    for region in deep {
        if contour_corner_left_out(region.outer(), shadows, tolerance)? {
            return Ok(true);
        }
        for hole in region.holes() {
            if contour_corner_left_out(hole, shadows, tolerance)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn contour_corner_left_out(contour: &Contour, shadows: &[Region], tolerance: &Tolerance) -> Result<bool> {
    let segs = contour.segments();
    let n = segs.len();
    if n == 0 {
        return Ok(false);
    }
    let ccw = contour.signed_area()? > 0.0;
    for i in 0..n {
        let Some(prev) = segs.get((i + n - 1) % n).copied() else {
            continue;
        };
        let Some(curr) = segs.get(i).copied() else {
            continue;
        };
        let Some(turn) = material_turn(prev, curr, ccw) else {
            continue;
        };
        // Reflex corners fillet. Only a turn into the material leaves a sharp offset corner.
        if turn <= 1e-3 {
            continue;
        }
        if !shadow_covers(curr.start(), shadows, tolerance)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn distance_to_loop(point: Point, contour: &Contour) -> f64 {
    contour.segments().iter().map(|seg| seg.distance_to_point(point)).fold(f64::INFINITY, f64::min)
}

fn shadow_covers(point: Point, shadows: &[Region], tolerance: &Tolerance) -> Result<bool> {
    for shadow in shadows {
        if in_region(point, shadow, tolerance)? {
            return Ok(true);
        }
        let mut nearest = distance_to_loop(point, shadow.outer());
        for hole in shadow.holes() {
            nearest = nearest.min(distance_to_loop(point, hole));
        }
        if nearest <= ORPHAN_CLEARANCE_MM {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Turn into the material at the join, in radians. Positive is a convex corner.
fn material_turn(prev: Segment, curr: Segment, ccw: bool) -> Option<f64> {
    let from = end_heading(prev)?;
    let to = start_heading(curr)?;
    let left = signed_turn(from, to);
    Some(if ccw { left } else { -left })
}

fn signed_turn(from: f64, to: f64) -> f64 {
    let mut delta = (to - from).rem_euclid(std::f64::consts::TAU);
    if delta > std::f64::consts::PI {
        delta -= std::f64::consts::TAU;
    }
    delta
}

fn start_heading(seg: Segment) -> Option<f64> {
    segment_heading(seg, false)
}

fn end_heading(seg: Segment) -> Option<f64> {
    segment_heading(seg, true)
}

fn segment_heading(seg: Segment, at_end: bool) -> Option<f64> {
    match seg {
        Segment::Line(line) => {
            let dir = line.end() - line.start();
            let len = dir.length();
            if !len.is_finite() || len <= f64::EPSILON { None } else { Some(dir.angle()) }
        }
        Segment::Arc(arc) => {
            let theta = if at_end { arc.end_angle() } else { arc.start_angle() };
            let offset = if arc.sweep() >= 0.0 { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 };
            Some(theta + offset)
        }
    }
}

fn subtract_all(sources: &[Region], cuts: &[Region], tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut pending = sources.to_vec();
    for cut in cuts {
        if pending.is_empty() {
            break;
        }
        let mut next = Vec::new();
        for piece in &pending {
            next.extend(boolean_regions(BooleanOp::Difference, piece, cut, tolerance)?);
        }
        pending = next;
    }
    Ok(pending)
}

fn inward(region: &Region, distance: f64, tolerance: &Tolerance) -> Result<Vec<Region>> {
    if !(distance > 0.0) {
        return Ok(Vec::new());
    }
    let mut next = Vec::new();
    for piece in offset_region(region, distance, tolerance)? {
        if region_alive(&piece, tolerance)? {
            next.push(piece);
        }
    }
    Ok(next)
}

fn smaller(before: &Region, after: &Region, tolerance: &Tolerance) -> Result<bool> {
    let earlier = before.area()?.abs();
    let later = after.area()?.abs();
    let eps = linear_eps(tolerance).max(1e-6);
    Ok(later + eps < earlier)
}

fn append_loops(regions: &[Region], out: &mut Vec<Contour>, tolerance: &Tolerance) -> Result<()> {
    for region in regions {
        if loop_alive(region.outer(), tolerance)? {
            out.push(region.outer().clone());
        }
        for hole in region.holes() {
            if loop_alive(hole, tolerance)? {
                out.push(hole.clone());
            }
        }
    }
    Ok(())
}

fn zigzag_spans(angle: f64, stepover: f64, regions: &[Region], tolerance: &Tolerance) -> Result<Vec<(Point, Point)>> {
    let bounds = regions_bounds(regions)?;
    let center = Point::new((bounds.min().x() + bounds.max().x()) * 0.5, (bounds.min().y() + bounds.max().y()) * 0.5)?;
    let (sin_a, cos_a) = angle.sin_cos();
    let dir = Vec2::new(cos_a, sin_a)?;
    let normal = Vec2::new(-sin_a, cos_a)?;
    let (min_n, max_n, min_d, max_d) = project_bounds(center, dir, normal, bounds)?;
    let mut spans = Vec::new();
    let mut index = 0usize;
    let limit = max_n + linear_eps(tolerance);
    loop {
        if index > MAX_PASSES {
            return Err(invalid(format!("too many zigzag lines: limit is {MAX_PASSES}")));
        }
        let n = min_n + index as f64 * stepover;
        if !n.is_finite() || n > limit {
            break;
        }
        let mut line = clip_line(center, dir, normal, n, min_d, max_d, regions, tolerance)?;
        if index % 2 == 1 {
            line.reverse();
            for span in &mut line {
                std::mem::swap(&mut span.0, &mut span.1);
            }
        }
        spans.extend(line);
        index += 1;
    }
    if spans.is_empty() {
        return Err(invalid("zigzag produced no cuts"));
    }
    Ok(spans)
}

fn project_bounds(center: Point, dir: Vec2, normal: Vec2, bounds: BoundingBox) -> Result<(f64, f64, f64, f64)> {
    let min = bounds.min();
    let max = bounds.max();
    let corners = [Point::new(min.x(), min.y())?, Point::new(max.x(), min.y())?, Point::new(max.x(), max.y())?, Point::new(min.x(), max.y())?];
    let mut min_n = f64::INFINITY;
    let mut max_n = f64::NEG_INFINITY;
    let mut min_d = f64::INFINITY;
    let mut max_d = f64::NEG_INFINITY;
    for corner in corners {
        let n = (corner - center).dot(normal);
        let d = (corner - center).dot(dir);
        if !n.is_finite() || !d.is_finite() {
            return Err(invalid("pocket bounds are not finite"));
        }
        min_n = min_n.min(n);
        max_n = max_n.max(n);
        min_d = min_d.min(d);
        max_d = max_d.max(d);
    }
    Ok((min_n, max_n, min_d, max_d))
}

/// Intersect the infinite line `center + normal * n + dir * s` with `regions` and keep the inside spans.
fn clip_line(
    center: Point,
    dir: Vec2,
    normal: Vec2,
    n: f64,
    min_d: f64,
    max_d: f64,
    regions: &[Region],
    tolerance: &Tolerance,
) -> Result<Vec<(Point, Point)>> {
    let anchor = checked(center + normal * n)?;
    let pad = (max_d - min_d).abs().max(1.0);
    let s0 = min_d - pad;
    let s1 = max_d + pad;
    let mut params = Vec::new();
    for region in regions {
        params.extend(line_hits(anchor, dir, s0, s1, region.outer(), tolerance)?);
        for hole in region.holes() {
            params.extend(line_hits(anchor, dir, s0, s1, hole, tolerance)?);
        }
    }
    sort_params(&mut params);
    let eps = linear_eps(tolerance);
    let unique = dedup_sorted(&params, eps);
    let mut spans = Vec::new();
    for pair in unique.windows(2) {
        let (Some(&left), Some(&right)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        if right - left <= eps {
            continue;
        }
        let mid = checked(anchor + dir * ((left + right) * 0.5))?;
        if !covered(mid, regions, tolerance)? {
            continue;
        }
        let start = checked(anchor + dir * left)?;
        let end = checked(anchor + dir * right)?;
        if start.distance(end) <= eps {
            continue;
        }
        spans.push((start, end));
    }
    Ok(spans)
}

fn line_hits(anchor: Point, dir: Vec2, s0: f64, s1: f64, contour: &Contour, tolerance: &Tolerance) -> Result<Vec<f64>> {
    let start = checked(anchor + dir * s0)?;
    let end = checked(anchor + dir * s1)?;
    let mut found = Vec::new();
    for seg in contour.segments() {
        match *seg {
            Segment::Line(line) => {
                if let Some(t) = segment_param(start, end, line.start(), line.end()) {
                    found.push(s0 + t * (s1 - s0));
                }
            }
            Segment::Arc(arc) => {
                for t in arc_params(start, end, arc, tolerance)? {
                    found.push(s0 + t * (s1 - s0));
                }
            }
        }
    }
    Ok(found)
}

fn stays_inside(start: Point, end: Point, clearance: &[Region], tolerance: &Tolerance) -> Result<bool> {
    let eps = linear_eps(tolerance);
    if start.almost_eq(end, eps) {
        return covered(start, clearance, tolerance);
    }
    if !covered(start, clearance, tolerance)? || !covered(end, clearance, tolerance)? {
        return Ok(false);
    }
    let steps = ((start.distance(end) / 0.5).ceil() as usize).clamp(8, 64);
    for i in 1..steps {
        let t = i as f64 / steps as f64;
        let sample = lerp(start, end, t)?;
        if !covered(sample, clearance, tolerance)? && !seam_covered(sample, clearance, tolerance)? {
            return Ok(false);
        }
    }
    if crosses_boundary(start, end, clearance, tolerance)? {
        return Ok(false);
    }
    Ok(true)
}

fn crosses_boundary(start: Point, end: Point, clearance: &[Region], tolerance: &Tolerance) -> Result<bool> {
    let eps = 1e-5;
    for region in clearance {
        if segment_crosses(start, end, region.outer(), tolerance, eps)? {
            return Ok(true);
        }
        for hole in region.holes() {
            if segment_crosses(start, end, hole, tolerance, eps)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn segment_crosses(start: Point, end: Point, contour: &Contour, tolerance: &Tolerance, eps: f64) -> Result<bool> {
    for seg in contour.segments() {
        let hits = match *seg {
            Segment::Line(line) => segment_param(start, end, line.start(), line.end()).into_iter().collect(),
            Segment::Arc(arc) => arc_params(start, end, arc, tolerance)?,
        };
        for t in hits {
            if t > eps && t < 1.0 - eps {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// Parameter of the intersection along `start → end`, if the segments cross or touch.
fn segment_param(start: Point, end: Point, c: Point, d: Point) -> Option<f64> {
    let r = end - start;
    let s = d - c;
    let denom = r.cross(s);
    let scale = r.length() * s.length();
    if !denom.is_finite() || denom.abs() <= 1e-12 * scale.max(1.0) {
        return None;
    }
    let q = c - start;
    let t = q.cross(s) / denom;
    let u = q.cross(r) / denom;
    if !t.is_finite() || !u.is_finite() {
        return None;
    }
    let slack = 1e-8;
    if (-slack..=1.0 + slack).contains(&t) && (-slack..=1.0 + slack).contains(&u) { Some(t.clamp(0.0, 1.0)) } else { None }
}

fn arc_params(start: Point, end: Point, arc: Arc, tolerance: &Tolerance) -> Result<Vec<f64>> {
    let chord = end - start;
    let a_q = chord.dot(chord);
    if !a_q.is_finite() || a_q <= 1e-24 {
        return Ok(Vec::new());
    }
    let f = start - arc.center();
    let b_q = 2.0 * f.dot(chord);
    let radius = arc.radius();
    let c_q = f.dot(f) - radius * radius;
    let disc = b_q * b_q - 4.0 * a_q * c_q;
    let scale = radius.max(1.0);
    if !disc.is_finite() {
        return Err(invalid("arc intersection is not finite"));
    }
    // `disc` is length⁴. Ignore a clearly negative discriminant and treat a near miss as tangent.
    if disc < -(1e-8 * scale * scale) * a_q {
        return Ok(Vec::new());
    }
    let root = disc.max(0.0).sqrt();
    let mut params = Vec::new();
    for sign in [-1.0, 1.0] {
        let t = (-b_q + sign * root) / (2.0 * a_q);
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            continue;
        }
        let hit = lerp(start, end, t)?;
        let limit = tolerance.linear().max(1e-5);
        if arc.distance_to_point(hit) <= limit {
            params.push(t);
        }
    }
    Ok(params)
}

/// A circle that starts on +X reports interior points on that seam as outside.
/// A 1 µm step off the seam is still inside the same face, and far too small to hide a gouge.
fn seam_covered(point: Point, regions: &[Region], tolerance: &Tolerance) -> Result<bool> {
    for (dx, dy) in [(0.0, 1e-6), (0.0, -1e-6), (1e-6, 0.0), (-1e-6, 0.0)] {
        let nudged = Point::new(point.x() + dx, point.y() + dy)?;
        if covered(nudged, regions, tolerance)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn covered(point: Point, regions: &[Region], tolerance: &Tolerance) -> Result<bool> {
    for region in regions {
        if in_region(point, region, tolerance)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn in_region(point: Point, region: &Region, tolerance: &Tolerance) -> Result<bool> {
    match region.outer().containment(point, tolerance)? {
        Containment::Outside => return Ok(false),
        Containment::Inside | Containment::Boundary => {}
    }
    for hole in region.holes() {
        if matches!(hole.containment(point, tolerance)?, Containment::Inside) {
            return Ok(false);
        }
    }
    Ok(true)
}

fn non_empty(regions: Vec<Region>) -> Result<Vec<Region>> {
    if regions.is_empty() { Err(invalid("tool does not fit in the pocket")) } else { Ok(regions) }
}

fn region_alive(region: &Region, tolerance: &Tolerance) -> Result<bool> {
    if !loop_alive(region.outer(), tolerance)? {
        return Ok(false);
    }
    let area = region.area()?;
    Ok(area.is_finite() && area.abs() > linear_eps(tolerance))
}

fn loop_alive(contour: &Contour, tolerance: &Tolerance) -> Result<bool> {
    if !contour.is_closed() {
        return Ok(false);
    }
    let length = contour.length()?;
    Ok(length.is_finite() && length > linear_eps(tolerance))
}

fn outer_span(region: &Region) -> Result<f64> {
    let bounds = region.outer().bounding_box()?;
    let span = bounds.width().max(bounds.height());
    if span.is_finite() { Ok(span) } else { Err(invalid("pocket bounds are not finite")) }
}

fn regions_bounds(regions: &[Region]) -> Result<BoundingBox> {
    let mut bounds: Option<BoundingBox> = None;
    for region in regions {
        let next = region.outer().bounding_box()?;
        bounds = Some(match bounds {
            None => next,
            Some(acc) => acc.union(next),
        });
    }
    bounds.ok_or_else(|| invalid("offset produced no region"))
}

/// Ascending parameters. Keys inside the float-key band keep the earlier hit.
fn sort_params(params: &mut Vec<f64>) {
    let mut indexed: Vec<(usize, f64)> = params.iter().copied().enumerate().collect();
    indexed.sort_by(|left, right| cmp_float_key(left.1, right.1).then(left.0.cmp(&right.0)));
    params.clear();
    params.extend(indexed.into_iter().map(|(_, value)| value));
}

fn dedup_sorted(values: &[f64], eps: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for &value in values {
        if !value.is_finite() {
            continue;
        }
        if out.last().is_none_or(|prev| (value - prev).abs() > eps) {
            out.push(value);
        }
    }
    out
}

fn lerp(start: Point, end: Point, t: f64) -> Result<Point> {
    checked(start + (end - start) * t)
}

fn checked(point: Point) -> Result<Point> {
    Ok(Point::new(point.x(), point.y())?)
}

fn linear_eps(tolerance: &Tolerance) -> f64 {
    tolerance.linear().max(1e-6)
}

fn invalid(message: impl Into<String>) -> PathError {
    PathError::InvalidInput(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;
    use campanion_geom::{Line, Polyline, Vertex};
    use proptest::prelude::*;
    use std::f64::consts::{PI, TAU};

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} != {b} (tol {tol})");
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

    fn circle(cx: f64, cy: f64, r: f64) -> Contour {
        Polyline::new(
            vec![Vertex::new(Point::new(cx + r, cy).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(cx - r, cy).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap()
        .to_contour(&Tolerance::default())
        .unwrap()
    }

    fn square_pocket(side: f64) -> Region {
        Region::from_contour(rect(side, side), &Tolerance::default()).unwrap()
    }

    fn tool_diameter(diameter: f64) -> Tool {
        Tool::Diameter { diameter }
    }

    fn rings_op(stepover: f64) -> PocketOp {
        PocketOp::new(tool_diameter(4.0), PocketStrategy::OffsetRings, stepover)
    }

    /// Cut-only polylines. A rapid or a plunge breaks the chain.
    fn cut_polylines(path: &Toolpath) -> Vec<Vec<(f64, f64)>> {
        let mut lines = Vec::new();
        let mut current = Vec::new();
        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut } | Move::Arc { to, kind: CutKind::Cut, .. } => {
                    let Some(from) = cursor else {
                        continue;
                    };
                    let gap = current.last().is_none_or(|p: &(f64, f64)| (p.0 - from.x()).hypot(p.1 - from.y()) > 1e-4);
                    if gap {
                        if !current.is_empty() {
                            lines.push(std::mem::take(&mut current));
                        }
                        current.push((from.x(), from.y()));
                    }
                    current.push((to.x(), to.y()));
                    cursor = Some(to);
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        if !current.is_empty() {
            lines.push(current);
        }
        lines
    }

    /// Split a cut polyline into closed rings. One straight link may sit between two rings.
    fn closed_rings(points: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let mut rings = Vec::new();
        let mut i = 0usize;
        while i + 3 < points.len() {
            let Some(&(sx, sy)) = points.get(i) else {
                break;
            };
            let mut found = None;
            let mut j = i + 3;
            while j < points.len() {
                let Some(&(x, y)) = points.get(j) else {
                    break;
                };
                if (x - sx).hypot(y - sy) <= 1e-4 {
                    found = Some(j);
                    break;
                }
                j += 1;
            }
            let Some(j) = found else {
                break;
            };
            rings.push(points[i..=j].to_vec());
            i = j + 1;
        }
        rings
    }

    fn ring_bounds(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for (x, y) in points {
            min_x = min_x.min(*x);
            min_y = min_y.min(*y);
            max_x = max_x.max(*x);
            max_y = max_y.max(*y);
        }
        (min_x, min_y, max_x, max_y)
    }

    fn ring_area(points: &[(f64, f64)]) -> f64 {
        let mut area = 0.0;
        for pair in points.windows(2) {
            let (Some(&(x0, y0)), Some(&(x1, y1))) = (pair.first(), pair.get(1)) else {
                continue;
            };
            area += x0 * y1 - x1 * y0;
        }
        area.abs() * 0.5
    }

    fn samples_of(path: &Toolpath) -> Vec<Point> {
        let mut out = Vec::new();
        let mut cursor: Option<Point> = None;
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut } => {
                    if let Some(from) = cursor {
                        let end = Point::new(to.x(), to.y()).unwrap();
                        let len = from.distance(end);
                        let n = ((len / 0.25).ceil() as usize).clamp(1, 400);
                        for i in 0..=n {
                            let t = i as f64 / n as f64;
                            out.push(Point::new(from.x() + (end.x() - from.x()) * t, from.y() + (end.y() - from.y()) * t).unwrap());
                        }
                    }
                    cursor = Some(Point::new(to.x(), to.y()).unwrap());
                }
                Move::Arc { to, centre, ccw, kind: CutKind::Cut } => {
                    if let Some(from) = cursor {
                        sample_arc(from, Point::new(to.x(), to.y()).unwrap(), Point::new(centre.x(), centre.y()).unwrap(), ccw, &mut out);
                    }
                    cursor = Some(Point::new(to.x(), to.y()).unwrap());
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => {
                    cursor = Some(Point::new(to.x(), to.y()).unwrap());
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        out
    }

    fn sample_arc(from: Point, to: Point, centre: Point, ccw: bool, out: &mut Vec<Point>) {
        let r = from.distance(centre);
        if r < 1e-9 {
            return;
        }
        let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
        let a1 = (to.y() - centre.y()).atan2(to.x() - centre.x());
        let mut sweep = if ccw { (a1 - a0).rem_euclid(TAU) } else { (a0 - a1).rem_euclid(TAU) };
        if from.almost_eq(to, 1e-8) || sweep < 1e-8 {
            sweep = TAU;
        }
        let n = ((sweep / (PI / 12.0)).ceil() as usize).clamp(8, 96);
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let angle = if ccw { a0 + sweep * t } else { a0 - sweep * t };
            out.push(Point::new(centre.x() + r * angle.cos(), centre.y() + r * angle.sin()).unwrap());
        }
    }

    /// Distance from `point` to the material of a filled contour.
    ///
    /// Inside and on the boundary the distance is zero. Outside it is the distance to the edges.
    fn distance_to_filled(point: Point, contour: &Contour) -> f64 {
        let tol = Tolerance::default();
        match contour.containment(point, &tol).unwrap() {
            Containment::Inside | Containment::Boundary => 0.0,
            Containment::Outside => contour.segments().iter().map(|seg| seg.distance_to_point(point)).fold(f64::INFINITY, f64::min),
        }
    }

    /// Distance from an interior point to the contour. Outside and on the boundary are zero.
    fn distance_inside_to_wall(point: Point, contour: &Contour) -> f64 {
        let tol = Tolerance::default();
        match contour.containment(point, &tol).unwrap() {
            Containment::Outside | Containment::Boundary => 0.0,
            Containment::Inside => contour.segments().iter().map(|seg| seg.distance_to_point(point)).fold(f64::INFINITY, f64::min),
        }
    }

    #[test]
    fn square_pocket_rings_match_the_hand_calc() {
        // Square [0, 20] × [0, 20]. Tool diameter 4 mm, so the radius is 2. Stepover 2. No allowance.
        // The tool centre stays 2 mm inside the wall, so the first ring is [2, 18] × [2, 18], a 16 × 16 square.
        // Later rings step in by 2 mm: insets 4, 6 and 8, sides 12, 8 and 4.
        // The next inset is 10, which is half the side, so the square vanishes and is not a ring.
        // When (side/2 - radius) divides the stepover exactly, the ring count is that quotient:
        // (10 - 2) / 2 = 4.
        let pocket = square_pocket(20.0);
        let cut = rings_op(2.0).z(-1.0).retract_z(0.0).generate(&pocket).unwrap();
        assert_eq!(cut.ring_order(), RingOrder::OutsideIn);
        assert_eq!(cut.ring_count(), 4);

        let polylines = cut_polylines(cut.toolpath());
        assert_eq!(polylines.len(), 1, "concentric rings of a convex pocket join by straight cuts");
        let rings = closed_rings(&polylines[0]);
        assert_eq!(rings.len(), 4, "parsed {rings:?}");
        let expected_inset = [2.0, 4.0, 6.0, 8.0];
        for (ring, inset) in rings.iter().zip(expected_inset) {
            let (min_x, min_y, max_x, max_y) = ring_bounds(ring);
            near(min_x, inset, 1e-4);
            near(min_y, inset, 1e-4);
            near(max_x, 20.0 - inset, 1e-4);
            near(max_y, 20.0 - inset, 1e-4);
            let side = 20.0 - 2.0 * inset;
            near(max_x - min_x, side, 1e-4);
            near(max_y - min_y, side, 1e-4);
            near(ring_area(ring), side * side, 1e-3);
            for &(x, y) in ring {
                let on_vertical = (x - inset).abs() <= 1e-4 || (x - (20.0 - inset)).abs() <= 1e-4;
                let on_horizontal = (y - inset).abs() <= 1e-4 || (y - (20.0 - inset)).abs() <= 1e-4;
                assert!(on_vertical || on_horizontal, "({x}, {y}) is off the {side} mm ring");
            }
        }
        // Four perimeters: 64 + 48 + 32 + 16 = 160. Three straight links add a little more.
        let length = cut.toolpath().cutting_length().unwrap();
        assert!(length > 160.0 - 1e-3, "{length}");
        assert!(length < 160.0 + 80.0, "{length}");
        assert!(cut.toolpath().moves().iter().all(|mv| match mv {
            Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => to.z() == 0.0 || (to.z() + 1.0).abs() < 1e-12,
            Move::Event(_) | Move::Dwell { .. } => true,
        }));
        let cuts_at_depth =
            cut.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Linear { kind: CutKind::Cut, to } if (to.z() + 1.0).abs() < 1e-12)).count();
        assert!(cuts_at_depth > 4, "{cuts_at_depth}");
    }

    #[test]
    fn circular_island_stays_a_tool_radius_away() {
        // 20 mm square, circular island of radius 3 at the centre. Tool radius 2.
        // Every sampled cut must stay at least 2 mm from the island and from the outer wall.
        let tol = Tolerance::default();
        let outer = rect(20.0, 20.0);
        let island = circle(10.0, 10.0, 3.0);
        let pocket = Region::try_new(outer.clone(), vec![island.clone()], &tol).unwrap();
        let tool_radius = 2.0;
        let cut = rings_op(2.0).generate(&pocket).unwrap();
        let samples = samples_of(cut.toolpath());
        assert!(samples.len() > 16, "samples {}", samples.len());
        for point in &samples {
            let island_clearance = distance_to_filled(*point, &island);
            let wall_clearance = distance_inside_to_wall(*point, &outer);
            assert!(island_clearance + 1e-3 >= tool_radius, "({:.4}, {:.4}) is {island_clearance:.4} from the island", point.x(), point.y());
            assert!(wall_clearance + 1e-3 >= tool_radius, "({:.4}, {:.4}) is {wall_clearance:.4} from the wall", point.x(), point.y());
        }
    }

    #[test]
    fn zigzag_points_stay_inside_and_alternate() {
        let side = 20.0;
        let tool_radius = 2.0;
        let pocket = square_pocket(side);
        let angle = 0.4;
        let cut = PocketOp::new(tool_diameter(4.0), PocketStrategy::Zigzag { angle }, 2.0).generate(&pocket).unwrap();
        assert_eq!(cut.ring_count(), 0);
        let samples = samples_of(cut.toolpath());
        assert!(samples.len() > 8, "samples {}", samples.len());
        let outer = rect(side, side);
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for point in &samples {
            min_x = min_x.min(point.x());
            min_y = min_y.min(point.y());
            max_x = max_x.max(point.x());
            max_y = max_y.max(point.y());
            let clearance = distance_inside_to_wall(*point, &outer);
            assert!(clearance + 1e-3 >= tool_radius, "({:.4}, {:.4}) is {clearance:.4} inside the wall", point.x(), point.y());
        }
        assert!(max_x - min_x > 10.0 && max_y - min_y > 10.0, "span {min_x}..{max_x}, {min_y}..{max_y}");

        // Horizontal passes on the same square. Long cuts alternate +X and -X.
        let flat = PocketOp::new(tool_diameter(4.0), PocketStrategy::Zigzag { angle: 0.0 }, 3.0).generate(&pocket).unwrap();
        let mut signs = Vec::new();
        let mut cursor: Option<(f64, f64)> = None;
        for mv in flat.toolpath().moves() {
            if let Move::Linear { to, kind: CutKind::Cut } = mv
                && let Some((x, y)) = cursor
            {
                let dx = to.x() - x;
                let dy = to.y() - y;
                if dx.abs() > 5.0 && dy.abs() < 1e-3 {
                    signs.push(dx.signum());
                }
            }
            match mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some((to.x(), to.y())),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        assert!(signs.len() >= 3, "{signs:?}");
        for pair in signs.windows(2) {
            let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            assert!(a * b < 0.0, "expected alternating directions, got {signs:?}");
        }

        // A circular island must keep angled zigzag samples a tool radius away as well.
        let island = circle(10.0, 10.0, 3.0);
        let with_island = Region::try_new(outer, vec![island.clone()], &Tolerance::default()).unwrap();
        let island_cut = PocketOp::new(tool_diameter(4.0), PocketStrategy::Zigzag { angle }, 2.0).generate(&with_island).unwrap();
        let island_samples = samples_of(island_cut.toolpath());
        assert!(island_samples.len() > 8);
        for point in &island_samples {
            let clearance = distance_to_filled(*point, &island);
            assert!(clearance + 1e-3 >= tool_radius, "zigzag ({:.4}, {:.4}) is {clearance:.4} from the island", point.x(), point.y());
        }
    }

    #[test]
    fn finishing_pass_cuts_the_wall_the_allowance_left() {
        // Radius 2 + allowance 1 = 3. Rough rings stay on [3, 17]. The finish contour is the 16 mm
        // offset at inset 2.
        let pocket = square_pocket(20.0);
        let rough = rings_op(2.0).allowance(1.0).generate(&pocket).unwrap();
        let finished = rings_op(2.0).allowance(1.0).finish(true).generate(&pocket).unwrap();
        assert_eq!(rough.ring_count(), finished.ring_count());
        assert!(rough.ring_count() >= 1);
        let rough_min = samples_of(rough.toolpath()).iter().map(|point| point.x()).fold(f64::INFINITY, f64::min);
        let finish_min = samples_of(finished.toolpath()).iter().map(|point| point.x()).fold(f64::INFINITY, f64::min);
        near(rough_min, 3.0, 1e-3);
        near(finish_min, 2.0, 1e-3);
    }

    #[test]
    fn hostile_input_is_an_error() {
        let pocket = square_pocket(20.0);
        let tool = tool_diameter(4.0);
        let op = rings_op(2.0);
        let bad_steps = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -2.0, 1.0e9];
        for step in bad_steps {
            let err = PocketOp::new(tool, PocketStrategy::OffsetRings, step).generate(&pocket).expect_err("bad stepover");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        for built in [op.clone().z(f64::NAN), op.clone().z(f64::INFINITY), op.clone().retract_z(f64::NAN), op.clone().retract_z(-1.0).z(0.0)] {
            let err = built.generate(&pocket).expect_err("bad height");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        for built in [op.clone().allowance(-0.2), op.clone().allowance(f64::NAN), op.clone().allowance(f64::INFINITY)] {
            let err = built.generate(&pocket).expect_err("bad allowance");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        for bad_tool in [tool_diameter(0.0), tool_diameter(-4.0), tool_diameter(f64::NAN), tool_diameter(1.0e9), Tool::Kerf { width: 0.0 }] {
            let err = PocketOp::new(bad_tool, PocketStrategy::OffsetRings, 2.0).generate(&pocket).expect_err("bad tool");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }
        // Diameter 20 on a 20 mm square: the radius equals half the side and the pocket vanishes.
        let err = PocketOp::new(tool_diameter(20.0), PocketStrategy::OffsetRings, 2.0).generate(&pocket).expect_err("tool too big");
        assert!(err.to_string().contains("fit"), "{err}");
        let err = PocketOp::new(tool, PocketStrategy::Zigzag { angle: f64::NAN }, 2.0).generate(&pocket).expect_err("bad angle");
        assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        let err = PocketOp::new(tool, PocketStrategy::Zigzag { angle: f64::INFINITY }, 1.0).generate(&pocket).unwrap_err();
        assert!(err.to_string().contains("angle"), "{err}");
        let err = op.clone().corner_cleanup(true).generate(&pocket).unwrap_err();
        assert!(matches!(err, PathError::NotSupported(_)), "{err:?}");
        assert_eq!(err.to_string(), "not supported yet: corner clean-up");
        let err = PocketOp::new(tool, PocketStrategy::OffsetRings, 1.0e-6).generate(&pocket).unwrap_err();
        assert!(err.to_string().contains("too many"), "{err}");
    }

    fn annulus(outer_r: f64, island_r: f64) -> Region {
        let tol = Tolerance::default();
        Region::try_new(circle(0.0, 0.0, outer_r), vec![circle(0.0, 0.0, island_r)], &tol).unwrap()
    }

    fn centred_ring_radii(path: &Toolpath) -> Vec<f64> {
        let mut radii = Vec::new();
        for mv in path.moves() {
            if let Move::Arc { to, centre, kind: CutKind::Cut, .. } = *mv {
                let radius = (to.x() - centre.x()).hypot(to.y() - centre.y());
                if radius.is_finite() {
                    radii.push(radius);
                }
            }
        }
        radii.sort_by(f64::total_cmp);
        let mut unique = Vec::new();
        for radius in radii {
            if unique.last().is_none_or(|prev: &f64| (radius - *prev).abs() > 1e-3) {
                unique.push(radius);
            }
        }
        unique
    }

    fn assert_close_radii(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len(), "radii {actual:?} expected {expected:?}");
        for (got, want) in actual.iter().zip(expected) {
            near(*got, *want, 1e-3);
        }
    }

    fn max_radius_gap(radii: &[f64]) -> f64 {
        radii.windows(2).fold(0.0, |best, pair| {
            let (Some(&left), Some(&right)) = (pair.first(), pair.get(1)) else {
                return best;
            };
            best.max(right - left)
        })
    }

    /// Every radius in `[inner, outer]` is within `tool_r` of a tool-centre ring.
    fn assert_annulus_covered(radii: &[f64], inner: f64, outer: f64, tool_r: f64) {
        for step in 0..=400 {
            let radius = inner + (outer - inner) * f64::from(step) / 400.0;
            let best = radii.iter().fold(f64::INFINITY, |best, ring| best.min((radius - *ring).abs()));
            assert!(best <= tool_r + 1e-6, "r {radius} is {best} from {radii:?}, tool radius {tool_r}");
        }
    }

    fn assert_samples_in_band(path: &Toolpath, inner_wall: f64, outer_wall: f64, tool_r: f64) {
        for point in samples_of(path) {
            let radius = point.x().hypot(point.y());
            assert!(radius + 1e-3 >= inner_wall + tool_r, "sample r {radius} gouges the island");
            assert!(radius <= outer_wall - tool_r + 1e-3, "sample r {radius} gouges the wall");
        }
    }

    #[test]
    fn annulus_under_two_stepovers_keeps_the_middle_band() {
        // Outer 30, island 15, tool radius 3, stepover 5. Tool centres span r 18..=27 (9 mm).
        // A full stepover inset closes that band from both sides and used to drop every ring
        // between them, leaving r 21..=24 uncut.
        let tool_r = 3.0;
        let stepover = 5.0;
        let cut = PocketOp::new(tool_diameter(tool_r * 2.0), PocketStrategy::OffsetRings, stepover).generate(&annulus(30.0, 15.0)).unwrap();
        let radii = centred_ring_radii(cut.toolpath());
        assert_close_radii(&radii, &[18.0, 20.5, 24.5, 27.0]);
        assert!(max_radius_gap(&radii) <= stepover + 1e-6, "gap {} radii {radii:?}", max_radius_gap(&radii));
        assert_annulus_covered(&radii, 15.0, 30.0, tool_r);
        assert_samples_in_band(cut.toolpath(), 15.0, 30.0, tool_r);
    }

    #[test]
    fn five_millimetre_band_stays_within_a_three_millimetre_stepover() {
        // Outer 30, island 19, tool radius 3, stepover 3. Tool centres span r 22..=27 (5 mm).
        // The two wall passes alone are 5 mm apart, so each would engage 5 of the 6 mm diameter.
        let tool_r = 3.0;
        let stepover = 3.0;
        let cut = PocketOp::new(tool_diameter(tool_r * 2.0), PocketStrategy::OffsetRings, stepover).generate(&annulus(30.0, 19.0)).unwrap();
        let radii = centred_ring_radii(cut.toolpath());
        assert_close_radii(&radii, &[22.0, 23.5, 25.5, 27.0]);
        assert!(max_radius_gap(&radii) <= stepover + 1e-6, "gap {} radii {radii:?}", max_radius_gap(&radii));
        assert_annulus_covered(&radii, 19.0, 30.0, tool_r);
        assert_samples_in_band(cut.toolpath(), 19.0, 30.0, tool_r);
    }

    #[test]
    fn wide_annulus_keeps_a_full_stepover_until_the_last_band() {
        // Clip radii 47 and 13, band 34 mm, stepover 5. Full steps land on a 4 mm remainder,
        // which is already within one stepover, so no shorter pass is inserted.
        let cut = PocketOp::new(tool_diameter(6.0), PocketStrategy::OffsetRings, 5.0).generate(&annulus(50.0, 10.0)).unwrap();
        let radii = centred_ring_radii(cut.toolpath());
        assert_close_radii(&radii, &[13.0, 18.0, 23.0, 28.0, 32.0, 37.0, 42.0, 47.0]);
        assert!(max_radius_gap(&radii) <= 5.0 + 1e-6, "{radii:?}");
    }

    #[test]
    fn off_centre_island_gap_under_two_stepovers_is_filled() {
        // 80 mm square, island radius 8 centred at (21, 40). Tool radius 3, stepover 5.
        // On the left the tool-centre gap is 7 mm: wider than one stepover and narrower than two,
        // while the rest of the pocket still accepts a full step. The midpoint of that gap must
        // land within half a stepover of a cutting pass.
        let tol = Tolerance::default();
        let outer = rect(80.0, 80.0);
        let island = circle(21.0, 40.0, 8.0);
        let pocket = Region::try_new(outer, vec![island], &tol).unwrap();
        let stepover = 5.0;
        let cut = PocketOp::new(tool_diameter(6.0), PocketStrategy::OffsetRings, stepover).generate(&pocket).unwrap();
        let segs = cut_segments(cut.toolpath()).unwrap();
        let mid = Point::new(6.5, 40.0).unwrap();
        let best = segs.iter().fold(f64::INFINITY, |best, seg| best.min(seg.distance_to_point(mid)));
        assert!(best <= stepover * 0.5 + 1e-3, "gap midpoint is {best} from the toolpath");
        for point in samples_of(cut.toolpath()) {
            let wall = dist_to_contour(point, pocket.outer());
            let island_clear = dist_to_contour(point, &pocket.holes()[0]);
            assert!(wall.min(island_clear) + 1e-3 >= 3.0, "({:.3}, {:.3}) gouges", point.x(), point.y());
        }
    }

    #[test]
    fn solid_core_farther_than_the_tool_radius_is_cut() {
        // Clip radius 8.5, stepover 5, tool radius 3. Rings at 8.5 and 3.5 leave the centre
        // 3.5 mm from the inner ring, past the tool radius, so a shorter core pass is required.
        let cut = PocketOp::new(tool_diameter(6.0), PocketStrategy::OffsetRings, 5.0)
            .generate(&Region::from_contour(circle(0.0, 0.0, 11.5), &Tolerance::default()).unwrap())
            .unwrap();
        let radii = centred_ring_radii(cut.toolpath());
        assert_close_radii(&radii, &[0.5, 3.5, 8.5]);
        assert!(max_radius_gap(&radii) <= 5.0 + 1e-6, "{radii:?}");
        assert_annulus_covered(&radii, 0.0, 11.5, 3.0);
    }

    #[test]
    fn off_grid_disk_with_coarse_stepover_does_not_walk_by_tool_radius() {
        // Shrunk from the full-circle post: centre 0.058430123 mm off the grid, radius 0.05 mm,
        // stepover = radius + 1, tool diameter 0.02 mm. A larger disk with the same pattern
        // recursed once per tool radius until the thread overflowed its stack.
        let centre = 0.058430123;
        let tool_r = 0.01;
        for radius in [0.05, 40.0] {
            let region = Region::from_contour(circle(centre, centre, radius + tool_r), &Tolerance::default()).unwrap();
            let stepover = radius + 1.0;
            let cut =
                PocketOp::new(tool_diameter(tool_r * 2.0), PocketStrategy::OffsetRings, stepover).z(-1.0).retract_z(1.0).generate(&region).unwrap();
            let span = outer_span(&region).unwrap();
            let min_inset = stepover * 0.5;
            let cap = (span / min_inset).ceil() as usize + 2;
            assert_eq!(cut.ring_count(), 1, "radius {radius} rings {} cap {cap}", cut.ring_count());
            let radii = centred_ring_radii(cut.toolpath());
            assert!(max_radius_gap(&radii) <= stepover + 1e-3, "radius {radius} gap {} radii {radii:?}", max_radius_gap(&radii));
        }
    }

    fn l_shape(width: f64, height: f64, arm: f64, tol: &Tolerance) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![
                edge(p(0.0, 0.0), p(width, 0.0)),
                edge(p(width, 0.0), p(width, arm)),
                edge(p(width, arm), p(arm, arm)),
                edge(p(arm, arm), p(arm, height)),
                edge(p(arm, height), p(0.0, height)),
                edge(p(0.0, height), p(0.0, 0.0)),
            ],
            true,
            tol,
        )
        .unwrap()
    }

    fn dist_to_contour(point: Point, contour: &Contour) -> f64 {
        contour.segments().iter().map(|seg| seg.distance_to_point(point)).fold(f64::INFINITY, f64::min)
    }

    /// `in_material`, plus a nudge off a circle seam. A point on the +X seam of a circle that
    /// starts there is reported outside even when it sits in the interior.
    fn in_pocket(point: Point, outer: &Contour, holes: &[Contour], tol: &Tolerance) -> Result<bool> {
        if in_material(point, outer, holes, tol)? {
            return Ok(true);
        }
        for (dx, dy) in [(0.0, 1e-6), (0.0, -1e-6), (1e-6, 0.0), (-1e-6, 0.0)] {
            let nudged = Point::new(point.x() + dx, point.y() + dy)?;
            if in_material(nudged, outer, holes, tol)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn in_material(point: Point, outer: &Contour, holes: &[Contour], tol: &Tolerance) -> Result<bool> {
        if !matches!(outer.containment(point, tol)?, Containment::Inside | Containment::Boundary) {
            return Ok(false);
        }
        for hole in holes {
            if matches!(hole.containment(point, tol)?, Containment::Inside) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// A round tool can reach `point` without crossing a wall when the point lies in the
    /// tool-radius inset, or within a tool radius of that inset's boundary.
    fn tool_can_reach(point: Point, clearance: &[Region], radius: f64, tol: &Tolerance) -> Result<bool> {
        let mut nearest = f64::INFINITY;
        for region in clearance {
            if in_region(point, region, tol)? {
                return Ok(true);
            }
            nearest = nearest.min(dist_to_contour(point, region.outer()));
            for hole in region.holes() {
                nearest = nearest.min(dist_to_contour(point, hole));
            }
        }
        Ok(nearest <= radius + 1e-3)
    }

    fn grid_count(span: f64, step: f64) -> usize {
        let count = (span / step).ceil();
        if count.is_finite() && count > 0.0 { count as usize + 1 } else { 1 }
    }

    fn arc_segment(from: Point, to: Point, centre: Point, ccw: bool) -> Result<Segment> {
        let radius = from.distance(centre);
        let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
        let a1 = (to.y() - centre.y()).atan2(to.x() - centre.x());
        let mut sweep = if ccw { (a1 - a0).rem_euclid(TAU) } else { (a0 - a1).rem_euclid(TAU) };
        if from.almost_eq(to, 1e-8) || sweep < 1e-8 {
            sweep = TAU;
        }
        let start = if ccw { a0 } else { a0 - sweep };
        Ok(Segment::Arc(Arc::new(centre, radius, start, sweep)?))
    }

    fn cut_segments(path: &Toolpath) -> Result<Vec<Segment>> {
        let mut segs = Vec::new();
        let mut cursor: Option<Point> = None;
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut } => {
                    let end = Point::new(to.x(), to.y())?;
                    if let Some(from) = cursor
                        && from.distance(end) > 1e-9
                    {
                        segs.push(Segment::Line(Line::new(from, end)?));
                    }
                    cursor = Some(end);
                }
                Move::Arc { to, centre, ccw, kind: CutKind::Cut } => {
                    let end = Point::new(to.x(), to.y())?;
                    if let Some(from) = cursor {
                        segs.push(arc_segment(from, end, Point::new(centre.x(), centre.y())?, ccw)?);
                    }
                    cursor = Some(end);
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => {
                    cursor = Some(Point::new(to.x(), to.y())?);
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        Ok(segs)
    }

    fn assert_pocket_covers(outer: &Contour, holes: &[Contour], diameter: f64, stepover: f64) -> Result<()> {
        let tol = Tolerance::default();
        let radius = diameter * 0.5;
        let region =
            if holes.is_empty() { Region::from_contour(outer.clone(), &tol)? } else { Region::try_new(outer.clone(), holes.to_vec(), &tol)? };
        let clearance = offset_region(&region, radius, &tol)?;
        let cut = PocketOp::new(Tool::Diameter { diameter }, PocketStrategy::OffsetRings, stepover).tolerance(tol).generate(&region)?;
        let segs = cut_segments(cut.toolpath())?;
        let samples = samples_of(cut.toolpath());
        if segs.is_empty() || samples.is_empty() {
            return Err(invalid("pocket produced no toolpath"));
        }
        for point in &samples {
            let mut clear = dist_to_contour(*point, outer);
            for hole in holes {
                clear = clear.min(dist_to_contour(*point, hole));
            }
            if clear + 1e-3 < radius {
                return Err(invalid(format!("sample ({:.4}, {:.4}) is {clear:.4} from a wall, tool radius {radius}", point.x(), point.y())));
            }
        }
        let bounds = outer.bounding_box()?;
        let step = (stepover * 0.5).min(radius * 0.5).clamp(0.35, 1.25);
        let nx = grid_count(bounds.width(), step);
        let ny = grid_count(bounds.height(), step);
        let min = bounds.min();
        for ix in 0..nx {
            for iy in 0..ny {
                let point = Point::new(min.x() + ix as f64 * step, min.y() + iy as f64 * step)?;
                if !in_material(point, outer, holes, &tol)? || !tool_can_reach(point, &clearance, radius, &tol)? {
                    continue;
                }
                let best = segs.iter().fold(f64::INFINITY, |best, seg| best.min(seg.distance_to_point(point)));
                if best > radius + 1e-3 {
                    return Err(invalid(format!(
                        "({:.3}, {:.3}) is {best:.3} from the toolpath, tool radius {radius}, stepover {stepover}",
                        point.x(),
                        point.y()
                    )));
                }
            }
        }
        Ok(())
    }

    #[test]
    fn l_pocket_split_by_a_corner_island_is_covered() {
        // The island opens the inner corner and the inset splits into two arms. A wall sample
        // sat on the ray through a vertex the offset stored twice, so it looked reachable.
        let tol = Tolerance::default();
        let width: f64 = 22.343343428381743;
        let height: f64 = 48.828696746338174;
        let diameter = 4.750103730694528;
        let fraction = 0.6464345802283563;
        let shift_y = 0.08538001473554664;
        let stepover = diameter * fraction;
        let arm = width.min(height) * 0.45;
        let outer = l_shape(width, height, arm, &tol);
        let hole = circle(arm * 0.5, arm * 0.5 + shift_y * arm * 0.12, arm * 0.16);
        assert_pocket_covers(&outer, &[hole], diameter, stepover).unwrap();
    }

    #[test]
    fn rectangle_corner_leftover_is_covered() {
        // Stepover is wider than the tool radius, so the sharp corner between two rectangular
        // rings used to leave a cusp the tool does not reach. The leftover corner gets a pass.
        let width: f64 = 61.145906674186016;
        let height: f64 = 60.49276885871193;
        let diameter = 1.0;
        let fraction = 0.9096455307358682;
        assert_pocket_covers(&rect(width, height), &[], diameter, diameter * fraction).unwrap();
    }

    fn ramp_entry(angle_deg: f64) -> DepthEntry {
        DepthEntry::Ramp { max_angle: angle_deg.to_radians() }
    }

    fn count_kind(path: &Toolpath, want: CutKind) -> usize {
        path.moves()
            .iter()
            .filter(|mv| match **mv {
                Move::Linear { kind, .. } | Move::Arc { kind, .. } => kind == want,
                Move::Rapid { .. } | Move::Event(_) | Move::Dwell { .. } => false,
            })
            .count()
    }

    fn straight_plunges(path: &Toolpath) -> usize {
        let mut cursor: Option<Point3> = None;
        let mut count = 0;
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind } if kind != CutKind::Retract => {
                    if let Some(from) = cursor {
                        let horiz = (to.x() - from.x()).hypot(to.y() - from.y());
                        if horiz < 1e-4 && to.z() < from.z() - 1e-9 && to.z() < 0.0 {
                            count += 1;
                        }
                    }
                    cursor = Some(to);
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        count
    }

    fn xy_motion_len(from: Point3, to: Point3, centre: Option<(Point3, bool)>) -> f64 {
        match centre {
            None => (to.x() - from.x()).hypot(to.y() - from.y()),
            Some((centre, ccw)) => {
                let start = Point::new(from.x(), from.y()).unwrap();
                let end = Point::new(to.x(), to.y()).unwrap();
                let at = Point::new(centre.x(), centre.y()).unwrap();
                start.distance(at) * xy_sweep(start, end, at, ccw)
            }
        }
    }

    fn assert_descent_bound(path: &Toolpath, max_angle: f64) {
        let mut cursor: Option<Point3> = None;
        for mv in path.moves() {
            let next = match *mv {
                Move::Linear { to, kind } if kind != CutKind::Retract => Some((to, None)),
                Move::Arc { to, centre, ccw, kind } if kind != CutKind::Retract => Some((to, Some((centre, ccw)))),
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => {
                    cursor = Some(to);
                    None
                }
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let Some((to, centre)) = next {
                if let Some(from) = cursor
                    && to.z() < from.z() - 1e-9
                {
                    let angle = (from.z() - to.z()).atan2(xy_motion_len(from, to, centre));
                    assert!(
                        angle <= max_angle + 1e-6,
                        "descent {:.6} rad ({:.4}°) exceeds {:.4}°",
                        angle,
                        angle.to_degrees(),
                        max_angle.to_degrees()
                    );
                }
                cursor = Some(to);
            }
        }
    }

    /// No motion ends at or above Z 0 between the first and last feed that finishes below Z 0.
    fn assert_no_retract_between_rings(path: &Toolpath) {
        let mut below = Vec::new();
        for (index, mv) in path.moves().iter().enumerate() {
            let z = match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => to.z(),
                Move::Event(_) | Move::Dwell { .. } => continue,
            };
            if z < -1e-9 {
                below.push(index);
            }
        }
        assert!(below.len() >= 2, "expected several cuts below Z 0");
        let first = below[0];
        let last = *below.last().unwrap();
        for mv in &path.moves()[first + 1..last] {
            if let Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } = *mv {
                assert!(to.z() < -1e-9, "retract to Z {} between rings", to.z());
            }
        }
    }

    fn feed_samples(path: &Toolpath) -> Vec<Point> {
        let mut out = Vec::new();
        let mut cursor: Option<Point> = None;
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut | CutKind::LeadIn } => {
                    if let Some(from) = cursor {
                        let end = Point::new(to.x(), to.y()).unwrap();
                        let len = from.distance(end);
                        let n = ((len / 0.25).ceil() as usize).clamp(1, 400);
                        for i in 0..=n {
                            let t = i as f64 / n as f64;
                            out.push(Point::new(from.x() + (end.x() - from.x()) * t, from.y() + (end.y() - from.y()) * t).unwrap());
                        }
                    }
                    cursor = Some(Point::new(to.x(), to.y()).unwrap());
                }
                Move::Arc { to, centre, ccw, kind: CutKind::Cut | CutKind::LeadIn } => {
                    if let Some(from) = cursor {
                        sample_arc(from, Point::new(to.x(), to.y()).unwrap(), Point::new(centre.x(), centre.y()).unwrap(), ccw, &mut out);
                    }
                    cursor = Some(Point::new(to.x(), to.y()).unwrap());
                }
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => cursor = Some(Point::new(to.x(), to.y()).unwrap()),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        out
    }

    fn assert_samples_clear(path: &Toolpath, outer: &Contour, holes: &[Contour], radius: f64) -> Result<()> {
        let tol = Tolerance::default();
        let samples = feed_samples(path);
        if samples.is_empty() {
            return Err(invalid("pocket produced no toolpath"));
        }
        for point in samples {
            if !in_pocket(point, outer, holes, &tol)? {
                return Err(invalid(format!("sample ({:.4}, {:.4}) leaves the pocket", point.x(), point.y())));
            }
            let mut clear = dist_to_contour(point, outer);
            for hole in holes {
                clear = clear.min(dist_to_contour(point, hole));
            }
            if clear + 1e-3 < radius {
                return Err(invalid(format!("sample ({:.4}, {:.4}) is {clear:.4} from a wall, tool radius {radius}", point.x(), point.y())));
            }
        }
        Ok(())
    }

    #[test]
    fn omitted_entry_still_plunges() {
        let cut = rings_op(2.0).z(-3.0).retract_z(2.0).generate(&square_pocket(30.0)).unwrap();
        assert!(count_kind(cut.toolpath(), CutKind::Plunge) >= 1);
        assert!(straight_plunges(cut.toolpath()) >= 1);
    }

    #[test]
    fn ramp_on_annulus_and_square_links_at_depth() {
        let angle = 5.0_f64.to_radians();
        let annulus = PocketOp::new(tool_diameter(6.0), PocketStrategy::OffsetRings, 2.0)
            .z(-3.0)
            .retract_z(2.0)
            .entry(ramp_entry(5.0))
            .generate(&annulus(30.0, 15.0))
            .unwrap();
        let square = {
            let contour = {
                let p = |x, y| Point::new(x, y).unwrap();
                let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
                Contour::try_new(
                    vec![
                        edge(p(100.0, 25.0), p(150.0, 25.0)),
                        edge(p(150.0, 25.0), p(150.0, 75.0)),
                        edge(p(150.0, 75.0), p(100.0, 75.0)),
                        edge(p(100.0, 75.0), p(100.0, 25.0)),
                    ],
                    true,
                    &Tolerance::default(),
                )
                .unwrap()
            };
            PocketOp::new(tool_diameter(6.0), PocketStrategy::OffsetRings, 2.0)
                .z(-3.0)
                .retract_z(2.0)
                .entry(ramp_entry(5.0))
                .generate(&Region::from_contour(contour, &Tolerance::default()).unwrap())
                .unwrap()
        };
        for (label, cut) in [("annulus", &annulus), ("square", &square)] {
            assert!(cut.ring_count() > 1, "{label} rings {}", cut.ring_count());
            assert_eq!(count_kind(cut.toolpath(), CutKind::Plunge), 0, "{label}");
            assert_eq!(straight_plunges(cut.toolpath()), 0, "{label}");
            assert_descent_bound(cut.toolpath(), angle);
            assert_no_retract_between_rings(cut.toolpath());
        }
    }

    #[test]
    fn ramp_longer_than_the_ring_is_an_error() {
        let err = PocketOp::new(tool_diameter(4.0), PocketStrategy::OffsetRings, 2.0)
            .z(-10.0)
            .retract_z(2.0)
            .entry(ramp_entry(1.0))
            .generate(&square_pocket(20.0))
            .unwrap_err();
        assert!(err.to_string().contains("ramp is longer than the pocket path"), "{err}");
    }

    #[test]
    fn helix_that_leaves_the_pocket_is_an_error() {
        let err = PocketOp::new(tool_diameter(4.0), PocketStrategy::OffsetRings, 2.0)
            .z(-3.0)
            .retract_z(2.0)
            .entry(DepthEntry::Helix { radius: 40.0, pitch: 2.0 })
            .generate(&square_pocket(20.0))
            .unwrap_err();
        assert!(err.to_string().contains("helix does not fit in the pocket"), "{err}");
    }

    #[test]
    fn helix_inside_a_square_stays_within_the_angle() {
        let radius = 2.0;
        let angle = 5.0_f64.to_radians();
        let pitch = radius * TAU * angle.tan();
        let cut = PocketOp::new(tool_diameter(4.0), PocketStrategy::OffsetRings, 2.0)
            .z(-3.0)
            .retract_z(2.0)
            .entry(DepthEntry::Helix { radius, pitch })
            .generate(&square_pocket(40.0))
            .unwrap();
        assert_eq!(straight_plunges(cut.toolpath()), 0);
        assert_eq!(count_kind(cut.toolpath(), CutKind::Plunge), 0);
        assert_descent_bound(cut.toolpath(), angle);
        assert_no_retract_between_rings(cut.toolpath());
        let outer = rect(40.0, 40.0);
        assert_samples_clear(cut.toolpath(), &outer, &[], 2.0).unwrap();
    }

    #[test]
    fn right_angle_ramp_is_rejected() {
        let err = rings_op(2.0)
            .z(-1.0)
            .retract_z(1.0)
            .entry(DepthEntry::Ramp { max_angle: std::f64::consts::FRAC_PI_2 })
            .generate(&square_pocket(20.0))
            .unwrap_err();
        assert!(err.to_string().contains("ramp angle"), "{err}");
    }

    fn assert_shaped_entry(outer: &Contour, holes: &[Contour], diameter: f64, stepover: f64, entry: DepthEntry) -> Result<()> {
        let tol = Tolerance::default();
        let region =
            if holes.is_empty() { Region::from_contour(outer.clone(), &tol)? } else { Region::try_new(outer.clone(), holes.to_vec(), &tol)? };
        let max_angle = match entry {
            DepthEntry::Ramp { max_angle } => max_angle,
            DepthEntry::Helix { radius, pitch } => (pitch / (radius * TAU)).atan(),
            DepthEntry::Plunge => return Err(invalid("expected a ramp or helix")),
        };
        let cut = PocketOp::new(Tool::Diameter { diameter }, PocketStrategy::OffsetRings, stepover)
            .z(-1.0)
            .retract_z(2.0)
            .entry(entry)
            .tolerance(tol)
            .generate(&region)?;
        if straight_plunges(cut.toolpath()) != 0 || count_kind(cut.toolpath(), CutKind::Plunge) != 0 {
            return Err(invalid("shaped entry plunged"));
        }
        let mut cursor: Option<Point3> = None;
        for mv in cut.toolpath().moves() {
            let stepped = match *mv {
                Move::Linear { to, kind } if kind != CutKind::Retract => Some((to, None)),
                Move::Arc { to, centre, ccw, kind } if kind != CutKind::Retract => Some((to, Some((centre, ccw)))),
                Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => {
                    cursor = Some(to);
                    None
                }
                Move::Event(_) | Move::Dwell { .. } => None,
            };
            if let Some((to, centre)) = stepped {
                if let Some(from) = cursor
                    && to.z() < from.z() - 1e-9
                {
                    let angle = (from.z() - to.z()).atan2(xy_motion_len(from, to, centre));
                    if angle > max_angle + 1e-6 {
                        return Err(invalid(format!("descent {:.4}° exceeds {:.4}°", angle.to_degrees(), max_angle.to_degrees())));
                    }
                }
                cursor = Some(to);
            }
        }
        assert_samples_clear(cut.toolpath(), outer, holes, diameter * 0.5)
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(24))]

        #[test]
        fn random_pockets_are_covered_without_gouging(
            width in 16.0..64.0,
            height in 16.0..64.0,
            kind in 0u8..3,
            diameter in 1.0..12.0,
            fraction in 0.1..0.95,
            island in proptest::bool::ANY,
            shift_x in -0.7..0.7,
            shift_y in -0.7..0.7,
        ) {
            let stepover = diameter * fraction;
            let tol = Tolerance::default();
            let outer = match kind {
                0 => rect(width, height),
                1 => circle(0.0, 0.0, width * 0.5),
                _ => l_shape(width, height, width.min(height) * 0.45, &tol),
            };
            let mut holes = Vec::new();
            if island {
                let hole = match kind {
                    0 => {
                        let radius = width.min(height) * 0.15;
                        let cx = width * 0.5 + shift_x * (width * 0.5 - radius - 1.0);
                        let cy = height * 0.5 + shift_y * (height * 0.5 - radius - 1.0);
                        circle(cx, cy, radius)
                    }
                    1 => {
                        let outer_r = width * 0.5;
                        let radius = outer_r * 0.28;
                        let room = outer_r - radius - 1.0;
                        circle(shift_x * room, shift_y * room, radius)
                    }
                    _ => {
                        let arm = width.min(height) * 0.45;
                        let radius = arm * 0.16;
                        circle(arm * 0.5 + shift_x * arm * 0.12, arm * 0.5 + shift_y * arm * 0.12, radius)
                    }
                };
                holes.push(hole);
            }
            match assert_pocket_covers(&outer, &holes, diameter, stepover) {
                Ok(()) => {}
                Err(err) if err.to_string().contains("fit") || err.to_string().contains("no toolpath") || err.to_string().contains("too many") => {
                    prop_assume!(false);
                }
                Err(err) => prop_assert!(false, "{err}"),
            }
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(16))]

        #[test]
        fn random_pockets_ramp_or_helix_without_gouging(
            width in 24.0..64.0,
            height in 24.0..64.0,
            kind in 0u8..3,
            diameter in 2.0..8.0,
            fraction in 0.35..0.8,
            island in proptest::bool::ANY,
            shift_x in -0.7..0.7,
            shift_y in -0.7..0.7,
            angle_deg in 20.0..50.0,
            helix in proptest::bool::ANY,
        ) {
            let stepover = diameter * fraction;
            let tol = Tolerance::default();
            let outer = match kind {
                0 => rect(width, height),
                1 => circle(0.0, 0.0, width * 0.5),
                _ => l_shape(width, height, width.min(height) * 0.45, &tol),
            };
            let mut holes = Vec::new();
            if island {
                let hole = match kind {
                    0 => {
                        let radius = width.min(height) * 0.15;
                        let cx = width * 0.5 + shift_x * (width * 0.5 - radius - 1.0);
                        let cy = height * 0.5 + shift_y * (height * 0.5 - radius - 1.0);
                        circle(cx, cy, radius)
                    }
                    1 => {
                        let outer_r = width * 0.5;
                        let radius = outer_r * 0.28;
                        let room = outer_r - radius - 1.0;
                        circle(shift_x * room, shift_y * room, radius)
                    }
                    _ => {
                        let arm = width.min(height) * 0.45;
                        let radius = arm * 0.16;
                        circle(arm * 0.5 + shift_x * arm * 0.12, arm * 0.5 + shift_y * arm * 0.12, radius)
                    }
                };
                holes.push(hole);
            }
            let entry = if helix {
                let radius = 1.5_f64;
                let angle = f64::to_radians(angle_deg);
                DepthEntry::Helix { radius, pitch: radius * std::f64::consts::TAU * angle.tan() }
            } else {
                ramp_entry(angle_deg)
            };
            match assert_shaped_entry(&outer, &holes, diameter, stepover, entry) {
                Ok(()) => {}
                Err(err)
                    if err.to_string().contains("fit")
                        || err.to_string().contains("longer")
                        || err.to_string().contains("too many")
                        || err.to_string().contains("no toolpath")
                        || err.to_string().contains("right angle") =>
                {
                    prop_assume!(false);
                }
                Err(err) => prop_assert!(false, "{err}"),
            }
        }
    }
}
