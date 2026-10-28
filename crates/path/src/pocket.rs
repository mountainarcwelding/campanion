//! Pocket clearing inside a [`campanion_geom::Region`].
//!
//! The region is one outer boundary plus any island holes. The tool centre stays at least the
//! tool radius (or half the kerf) inside that boundary, and the same distance off every island.
//!
//! [`PocketStrategy::OffsetRings`] cuts successive inward offsets through [`offset_region`].
//! The first offset is the tool radius plus the finishing allowance. Each later offset moves in
//! by at most one stepover. A band between the outer wall and an island that is narrower than
//! two stepovers still gets a ring, so those passes stay within one stepover. A solid remainder
//! whose middle is farther than the tool radius from the last ring gets another pass. Rings are
//! emitted outside-in ([`RingOrder::OutsideIn`]). Two rings are joined by a straight cut when
//! that segment stays inside the region the tool centre may enter. Otherwise the tool retracts,
//! rapids, and plunges.
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

use campanion_geom::{Arc, BooleanOp, BoundingBox, Containment, Contour, Point, Region, Segment, Tolerance, Vec2, boolean_regions, cmp_float_key, offset_region};

use crate::check::{self, finite, non_negative, positive};
use crate::tool::Tool;
use crate::toolpath::{CutKind, Move, ProcessEvent, Toolpath};
use crate::{PathError, Result};

/// Upper bound on offset levels and on zigzag lines. A hostile stepover fails instead of looping.
const MAX_PASSES: usize = 1024;

/// Offset curves, including a shorter pass that closes a band under two stepovers.
const MAX_RING_LOOPS: usize = MAX_PASSES * 2;

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
        }
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
        emit(&mut pen, &pieces, &clearance, &self.tolerance)?;
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

fn emit(pen: &mut Pen, pieces: &[CutPiece], clearance: &[Region], tolerance: &Tolerance) -> Result<()> {
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

/// Emit `region`'s loops, then the next inward rings.
///
/// A full stepover inset moves every boundary, so the gap between an outer wall and an island
/// closes at twice the stepover. When that inset would drop a band that is still wider than one
/// stepover, or a solid core that the tool radius does not reach from the current ring, the next
/// ring is a shorter inset. Later rings use a full stepover again where the remaining region allows it.
fn cover_region(region: &Region, stepover: f64, radius: f64, tolerance: &Tolerance, loops: &mut Vec<Contour>) -> Result<()> {
    if loops.len() > MAX_RING_LOOPS {
        return Err(invalid(format!("too many offset rings: limit is {MAX_RING_LOOPS}")));
    }
    let before = loops.len();
    append_loops(std::slice::from_ref(region), loops, tolerance)?;
    if loops.len() == before {
        return Ok(());
    }
    if loops.len() > MAX_RING_LOOPS {
        return Err(invalid(format!("too many offset rings: limit is {MAX_RING_LOOPS}")));
    }

    let limit = ring_limit(region, stepover, radius);
    let children = inward(region, stepover, tolerance)?;
    if !children.is_empty() && !facing_gap(region, &children, limit, stepover, tolerance)? {
        return descend(&children, stepover, radius, tolerance, loops);
    }

    let step = limit.min(stepover);
    if step <= linear_eps(tolerance) {
        return descend(&children, stepover, radius, tolerance, loops);
    }
    let mids = inward(region, step, tolerance)?;
    let mut progressed = Vec::new();
    for mid in &mids {
        if smaller(region, mid, tolerance)? {
            progressed.push(mid);
        }
    }
    if progressed.is_empty() {
        return descend(&children, stepover, radius, tolerance, loops);
    }
    for mid in progressed {
        cover_region(mid, stepover, radius, tolerance, loops)?;
    }
    Ok(())
}

fn descend(children: &[Region], stepover: f64, radius: f64, tolerance: &Tolerance, loops: &mut Vec<Contour>) -> Result<()> {
    for child in children {
        cover_region(child, stepover, radius, tolerance, loops)?;
    }
    Ok(())
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
/// cover. Corner skins thinner than [`ORPHAN_CLEARANCE_MM`] are offset noise.
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
    Ok(false)
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
        if !covered(sample, clearance, tolerance)? {
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
}
