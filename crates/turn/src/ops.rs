//! Lathe facing, OD/ID roughing, and finishing.
//!
//! Every coordinate is the **programmed point**. For [`TipPoint::Centre`] that point is the
//! nose centre. For [`TipPoint::ImaginaryTip`] it is the imaginary sharp corner, and
//! `dz`/`dx` are that corner minus the nose centre. Nose-radius compensation is applied
//! in this module. Nothing here emits G41/G42.
//!
//! The profile runs from the front (larger Z) toward the chuck. With +Z to the right and
//! radius up, that travel's left side is −radius and its right side is +radius.
//! [`campanion_geom::offset`] is positive to the left, so an OD profile (free side
//! +radius) is offset by a **negative** distance and an ID profile by a **positive** one.
//!
//! Stock to leave is `offset distance = nose_radius + leave_x`, then a Z shift of
//! `leave_z - leave_x`. When the two allowances are equal the shift is zero and the curve
//! is the exact parallel at distance `nose_radius + leave`. The tip offset is added after
//! that, so the returned chain is the programmed point.
//!
//! M10 roughing is longitudinal and stops at the **first** boundary hit when cutting from
//! the front toward the chuck. A groove, or any smaller diameter hidden behind a larger
//! section that the pass cannot enter, is left for a later operation. The pass does not
//! cross that section.

use campanion_geom::{Arc, Contour, Line, Point, Segment, Tolerance};

use crate::{Result, TurnError, TurnProfile};

/// Most moves one facing, roughing, or finishing call may emit.
const MAX_MOVES: usize = 100_000;

/// A boundary this far past the pass radius, in millimetres, actually blocks the pass.
///
/// The last pass sits on the target surface. Float and offset noise of a few nanometres
/// must not look like material still standing above that pass.
const BLOCK_EPS: f64 = 1e-7;

/// Joints of a compensated chain may open up to this far before the chain is rejected.
const CONTIGUOUS_SLACK: f64 = 1e-4;

/// Roughing has no caller tolerance. This is loose enough for the offset cleanup.
const ROUGH_TOL: f64 = 1e-4;

/// One lathe move in the part half-plane: Z along the spindle, X = radius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TurnMove {
    /// Rapid to `(z, x)`.
    Rapid { z: f64, x: f64 },
    /// Linear feed to `(z, x)`.
    Feed { z: f64, x: f64 },
    /// Circular feed to `(z, x)` about centre `(cz, cx)`.
    ///
    /// `ccw` is counter-clockwise when +Z points right and +X (radius) points up.
    Arc { z: f64, x: f64, cz: f64, cx: f64, ccw: bool },
}

/// A programmed-point path. `start` is `(z, x)` before `moves`.
#[derive(Clone, Debug, PartialEq)]
pub struct TurnPath {
    /// Moves in order. Each one ends at its own `(z, x)`.
    pub moves: Vec<TurnMove>,
    /// Position before the first move, `(z, x)`.
    pub start: (f64, f64),
}

/// Which side of the profile the tool stands on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// OD. Material is toward the axis. Cutting reduces the radius.
    Outside,
    /// ID bore. Material is away from the axis. Cutting grows the radius.
    Inside,
}

/// Where the programmed point sits on the insert.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TipPoint {
    /// The programmed point is the nose centre.
    Centre,
    /// Imaginary sharp corner minus the nose centre, in `(dz, dx)`.
    ///
    /// A typical OD tool has the corner in the −Z, −X quadrant, so both components
    /// are `-nose_radius` when the edges are parallel to the axes.
    ImaginaryTip { dz: f64, dx: f64 },
}

/// Insert used to compensate a turning path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TurnTool {
    /// Nose radius in millimetres. Must be finite and positive.
    pub nose_radius: f64,
    /// Programmed point relative to the nose.
    pub tip: TipPoint,
}

/// Parameters for [`face`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaceParams {
    /// Z of the stock face before this operation. Larger than [`Self::final_z`].
    pub stock_front_z: f64,
    /// Z of the finished face.
    pub final_z: f64,
    /// Maximum axial depth of one pass. Passes split the depth into equal steps no larger than this.
    pub step_z: f64,
    /// Radius at which each pass starts (stock or beyond).
    pub from_radius: f64,
    /// Radius at which the face should be cut to. Often `0`, or a negative overshoot of about one nose radius.
    pub to_radius: f64,
    /// Axial pull-off after each pass, and the extra radius used to rapid in. Millimetres, `>= 0`.
    pub clearance: f64,
}

/// Parameters for [`rough`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoughParams {
    /// OD or ID.
    pub side: Side,
    /// Outer radius of the stock bar or tube.
    pub stock_outer_radius: f64,
    /// Inner radius of tube stock. Zero for a solid bar. Must be below [`Self::stock_outer_radius`].
    pub stock_inner_radius: f64,
    /// Maximum radial depth of one pass. Passes split the radial stock into equal steps no larger than this.
    pub depth_of_cut: f64,
    /// Radial stock left for finishing.
    pub leave_x: f64,
    /// Axial stock left for finishing.
    pub leave_z: f64,
    /// How far in front of the profile (larger Z) each pass starts.
    pub clearance: f64,
    /// Radial pull-off after a pass, away from the material.
    pub retract: f64,
}

/// Parameters for [`finish`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FinishParams {
    /// OD or ID.
    pub side: Side,
    /// Radial stock left on the part. Zero finishes to the profile.
    pub leave_x: f64,
    /// Axial stock left on the part. Zero finishes to the profile.
    pub leave_z: f64,
    /// Approach and retract gap, on the free side and toward +Z.
    pub clearance: f64,
    /// Linear tolerance for the nose-radius offset, in millimetres.
    pub tol: f64,
}

impl TurnTool {
    /// Reject a non-finite or non-positive nose radius, and a non-finite tip offset.
    pub fn validate(&self) -> Result<()> {
        positive(self.nose_radius, "nose radius")?;
        let (dz, dx) = tip_offset(self.tip);
        finite(dz, "tip dz")?;
        finite(dx, "tip dx")?;
        Ok(())
    }
}

impl FaceParams {
    /// Reject non-finite numbers, a non-positive step, a negative clearance, or a zero depth.
    pub fn validate(&self) -> Result<()> {
        finite(self.stock_front_z, "stock front z")?;
        finite(self.final_z, "final z")?;
        positive(self.step_z, "face step")?;
        finite(self.from_radius, "from radius")?;
        finite(self.to_radius, "to radius")?;
        non_negative(self.clearance, "clearance")?;
        if self.stock_front_z <= self.final_z {
            return Err(invalid("facing depth must be positive"));
        }
        Ok(())
    }
}

impl RoughParams {
    /// Reject non-finite numbers, a non-positive depth of cut, or a negative allowance.
    pub fn validate(&self) -> Result<()> {
        positive(self.stock_outer_radius, "stock outer radius")?;
        non_negative(self.stock_inner_radius, "stock inner radius")?;
        if self.stock_inner_radius >= self.stock_outer_radius {
            return Err(invalid("stock inner radius must be below stock outer radius"));
        }
        positive(self.depth_of_cut, "depth of cut")?;
        non_negative(self.leave_x, "leave x")?;
        non_negative(self.leave_z, "leave z")?;
        non_negative(self.clearance, "clearance")?;
        non_negative(self.retract, "retract")?;
        Ok(())
    }
}

impl FinishParams {
    /// Reject non-finite numbers, a negative allowance, or a non-positive tolerance.
    pub fn validate(&self) -> Result<()> {
        non_negative(self.leave_x, "leave x")?;
        non_negative(self.leave_z, "leave z")?;
        non_negative(self.clearance, "clearance")?;
        positive(self.tol, "finish tolerance")?;
        Ok(())
    }
}

/// Programmed-point chain the tool may ride without crossing the allowance.
///
/// The chain is contiguous and ordered like `profile` (front toward the chuck).
/// If the offset splits, the longest piece is kept. Losing more than 1% of the
/// offset length is [`TurnError::InvalidInput`].
pub fn compensated_profile(profile: &TurnProfile, tool: &TurnTool, side: Side, leave_x: f64, leave_z: f64, tol: f64) -> Result<Vec<Segment>> {
    tool.validate()?;
    non_negative(leave_x, "leave x")?;
    non_negative(leave_z, "leave z")?;
    positive(tol, "tolerance")?;
    let magnitude = tool.nose_radius + leave_x;
    positive(magnitude, "offset distance")?;
    // Right of front-to-back travel is +radius (OD). Left is −radius (ID).
    let distance = match side {
        Side::Outside => -magnitude,
        Side::Inside => magnitude,
    };
    let tolerance = Tolerance::new(tol, Tolerance::DEFAULT_ANGULAR_RAD)?;
    let contour = Contour::try_new(profile.segments().to_vec(), false, &tolerance)?;
    let pieces = campanion_geom::offset(&contour, distance, &tolerance)?;
    let segments = longest_piece(&pieces)?;
    let segments = orient_like(profile, segments);
    let (tip_z, tip_x) = tip_offset(tool.tip);
    // Radial allowance is inside the offset. The Z shift tops axial faces up to `leave_z`
    // and is zero when `leave_x == leave_z`, which keeps that case an exact parallel.
    let shift_z = (leave_z - leave_x) + tip_z;
    let shift_x = tip_x;
    let shifted = translate_chain(&segments, shift_z, shift_x)?;
    ensure_contiguous(&shifted, tol)?;
    Ok(shifted)
}

/// Face from [`FaceParams::stock_front_z`] down to [`FaceParams::final_z`].
///
/// Passes divide the depth into equal steps no larger than [`FaceParams::step_z`].
/// Each pass rapids to `(z_cut, from_radius + clearance)`, feeds to the compensated
/// end radius, feeds out by `clearance` in +Z, and rapids back out in X.
///
/// [`TipPoint::Centre`] cuts with the nose centre at `z_pass + nose_radius` and stops
/// at `to_radius - nose_radius`, so the nose wipes the face and reaches the centre.
/// [`TipPoint::ImaginaryTip`] programs the tip itself: the face Z and `to_radius` are
/// used as given, which is exact in Z for a tip that already lies on the nose's −Z tangent.
pub fn face(tool: &TurnTool, p: &FaceParams) -> Result<TurnPath> {
    tool.validate()?;
    p.validate()?;
    let depth = p.stock_front_z - p.final_z;
    let (count, step) = equal_steps(depth, p.step_z)?;
    if count.saturating_mul(4) > MAX_MOVES {
        return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
    }
    let (z_shift, x_end_shift) = match tool.tip {
        TipPoint::Centre => (tool.nose_radius, -tool.nose_radius),
        TipPoint::ImaginaryTip { .. } => (0.0, 0.0),
    };
    let x_start = p.from_radius + p.clearance;
    let x_end = p.to_radius + x_end_shift;
    finite(x_start, "face start radius")?;
    finite(x_end, "face end radius")?;
    let z_first = p.stock_front_z - step + z_shift;
    let start = (z_first + p.clearance, x_start);
    finite(start.0, "face start z")?;
    let mut moves = Vec::with_capacity(count.saturating_mul(4));
    for index in 1..=count {
        let z_pass = if index == count { p.final_z } else { p.stock_front_z - step * index as f64 };
        let z_cut = z_pass + z_shift;
        push(&mut moves, TurnMove::Rapid { z: z_cut, x: x_start })?;
        push(&mut moves, TurnMove::Feed { z: z_cut, x: x_end })?;
        push(&mut moves, TurnMove::Feed { z: z_cut + p.clearance, x: x_end })?;
        push(&mut moves, TurnMove::Rapid { z: z_cut + p.clearance, x: x_start })?;
    }
    Ok(TurnPath { moves, start })
}

/// Longitudinal roughing from the stock radius toward the compensated profile.
///
/// OD passes step inward from [`RoughParams::stock_outer_radius`]. ID passes step
/// outward from [`RoughParams::stock_inner_radius`]. Steps are equal and no larger
/// than [`RoughParams::depth_of_cut`]. The last pass lands on the target radius.
///
/// A pass feeds from the profile's maximum Z plus [`RoughParams::clearance`] toward
/// the chuck at constant radius until the compensated boundary blocks it, then feeds
/// along that boundary toward the previous pass radius so the shoulder is not left
/// as a staircase. It then pulls off by [`RoughParams::retract`] (OD increases
/// radius, ID decreases it, never below 0) and rapids back to the front.
///
/// Undercuts are not cut. The pass stops at the first boundary it meets. A diameter
/// that sits behind a larger front section (OD) or behind a smaller entrance (ID)
/// is not reached by a pass that cannot enter that section.
pub fn rough(profile: &TurnProfile, tool: &TurnTool, p: &RoughParams) -> Result<TurnPath> {
    p.validate()?;
    let boundary = compensated_profile(profile, tool, p.side, p.leave_x, p.leave_z, ROUGH_TOL)?;
    let (z_min, z_max) = chain_z_range(&boundary);
    let z_front = z_max + p.clearance;
    finite(z_front, "rough front z")?;
    let start_y = boundary.first().ok_or_else(|| invalid("compensated profile is empty"))?.start().y();
    let (stock_r, target_r) = match p.side {
        Side::Outside => (p.stock_outer_radius, chain_min_y(&boundary)),
        Side::Inside => (p.stock_inner_radius, chain_max_y(&boundary)),
    };
    if !target_r.is_finite() || target_r < -BLOCK_EPS {
        return Err(invalid("compensated radius is negative"));
    }
    let span = match p.side {
        Side::Outside => stock_r - target_r,
        Side::Inside => target_r - stock_r,
    };
    if span <= BLOCK_EPS {
        return Err(invalid("no material to remove"));
    }
    let (count, step) = equal_steps(span, p.depth_of_cut)?;
    let radii = pass_radii(p.side, stock_r, target_r, count, step)?;
    let free = free_sign(p.side);
    let safe = |radius: f64| (radius + free * p.retract).max(0.0);
    let start = (z_front, safe(stock_r));
    let mut moves = Vec::new();
    let mut cursor = start;
    let mut previous = stock_r;
    for radius in radii {
        // A pass whose own radius is already inside the front section cannot enter.
        // M10 does not jump past that section into an undercut or a hidden journal.
        if !pass_can_enter(p.side, start_y, radius) {
            previous = radius;
            continue;
        }
        let z_block = blocking_z(&boundary, radius, p.side);
        let z_cut = z_block.unwrap_or(z_min).clamp(z_min, z_max);
        move_to(&mut moves, &mut cursor, TurnMove::Rapid { z: z_front, x: radius }, false)?;
        if z_front - z_cut > 1e-12 {
            move_to(&mut moves, &mut cursor, TurnMove::Feed { z: z_cut, x: radius }, false)?;
        }
        if z_block.is_some() {
            follow_to_previous(&boundary, z_cut, radius, previous, p.side, &mut moves, &mut cursor)?;
        }
        let pull = safe_from(cursor.1, free, p.retract);
        let pull_z = cursor.0;
        move_to(&mut moves, &mut cursor, TurnMove::Feed { z: pull_z, x: pull }, false)?;
        let back = safe(previous);
        move_to(&mut moves, &mut cursor, TurnMove::Rapid { z: z_front, x: back }, false)?;
        previous = radius;
    }
    Ok(TurnPath { moves, start })
}

/// Finish along the compensated profile.
///
/// The path starts at the approach point `(front z + clearance, free-side radius + clearance)`,
/// feeds onto and along the compensated chain (lines as [`TurnMove::Feed`], arcs as
/// [`TurnMove::Arc`]), retracts radially by [`FinishParams::clearance`], and rapids out in +Z.
/// `leave_x` and `leave_z` are normally zero.
pub fn finish(profile: &TurnProfile, tool: &TurnTool, p: &FinishParams) -> Result<TurnPath> {
    p.validate()?;
    let boundary = compensated_profile(profile, tool, p.side, p.leave_x, p.leave_z, p.tol)?;
    let first = boundary.first().ok_or_else(|| invalid("compensated profile is empty"))?;
    let last = boundary.last().ok_or_else(|| invalid("compensated profile is empty"))?;
    let free = free_sign(p.side);
    let approach_x = (first.start().y() + free * p.clearance).max(0.0);
    let approach = (first.start().x() + p.clearance, approach_x);
    finite(approach.0, "finish approach z")?;
    finite(approach.1, "finish approach x")?;
    let mut moves = Vec::new();
    let mut cursor = approach;
    // Radial engage at the front, then the chain itself.
    move_to(&mut moves, &mut cursor, TurnMove::Feed { z: first.start().x(), x: approach_x }, true)?;
    move_to(&mut moves, &mut cursor, TurnMove::Feed { z: first.start().x(), x: first.start().y() }, true)?;
    for segment in &boundary {
        push_segment(&mut moves, &mut cursor, *segment)?;
    }
    let end_z = last.end().x();
    let retract_x = (last.end().y() + free * p.clearance).max(0.0);
    move_to(&mut moves, &mut cursor, TurnMove::Feed { z: end_z, x: retract_x }, true)?;
    move_to(&mut moves, &mut cursor, TurnMove::Rapid { z: end_z + p.clearance, x: retract_x }, true)?;
    Ok(TurnPath { moves, start: approach })
}

fn tip_offset(tip: TipPoint) -> (f64, f64) {
    match tip {
        TipPoint::Centre => (0.0, 0.0),
        TipPoint::ImaginaryTip { dz, dx } => (dz, dx),
    }
}

/// +1 when the free side is larger radius, −1 when it is smaller.
fn free_sign(side: Side) -> f64 {
    match side {
        Side::Outside => 1.0,
        Side::Inside => -1.0,
    }
}

fn invalid(message: impl Into<String>) -> TurnError {
    TurnError::InvalidInput(message.into())
}

fn finite(value: f64, what: &str) -> Result<()> {
    if value.is_finite() { Ok(()) } else { Err(invalid(format!("{what} must be finite"))) }
}

fn positive(value: f64, what: &str) -> Result<()> {
    finite(value, what)?;
    if value > 0.0 { Ok(()) } else { Err(invalid(format!("{what} must be positive"))) }
}

fn non_negative(value: f64, what: &str) -> Result<()> {
    finite(value, what)?;
    if value >= 0.0 { Ok(()) } else { Err(invalid(format!("{what} must be >= 0"))) }
}

/// Equal steps covering `span`, each at most `max_step`. The last step lands on the end.
fn equal_steps(span: f64, max_step: f64) -> Result<(usize, f64)> {
    positive(span, "span")?;
    positive(max_step, "step")?;
    let ratio = span / max_step;
    if !ratio.is_finite() || ratio > MAX_MOVES as f64 {
        return Err(invalid(format!("too many passes for a span of {span}")));
    }
    let rounded = ratio.round();
    let count_f = if (ratio - rounded).abs() <= 1e-9 * ratio.max(1.0) { rounded } else { ratio.ceil() };
    if !count_f.is_finite() || count_f < 1.0 || count_f > MAX_MOVES as f64 {
        return Err(invalid(format!("too many passes for a span of {span}")));
    }
    let count = count_f as usize;
    if count == 0 || count > MAX_MOVES {
        return Err(invalid(format!("too many passes for a span of {span}")));
    }
    Ok((count, span / count as f64))
}

fn pass_radii(side: Side, stock: f64, target: f64, count: usize, step: f64) -> Result<Vec<f64>> {
    let mut radii = Vec::with_capacity(count);
    for index in 1..=count {
        let radius = if index == count {
            target
        } else {
            match side {
                Side::Outside => stock - step * index as f64,
                Side::Inside => stock + step * index as f64,
            }
        };
        if !radius.is_finite() || radius < -BLOCK_EPS {
            return Err(invalid("pass radius is negative"));
        }
        radii.push(radius.max(0.0));
    }
    Ok(radii)
}

fn longest_piece(pieces: &[Contour]) -> Result<Vec<Segment>> {
    if pieces.is_empty() {
        return Err(invalid("offset produced no curve"));
    }
    let mut best_index = 0usize;
    let mut best_len = f64::NEG_INFINITY;
    let mut total = 0.0;
    for (index, piece) in pieces.iter().enumerate() {
        let len = piece.length()?;
        total += len;
        if len > best_len {
            best_len = len;
            best_index = index;
        }
    }
    if !(total > 0.0) || best_len + 1e-9 < 0.99 * total {
        return Err(invalid("offset lost more than 1% of the length"));
    }
    let Some(piece) = pieces.get(best_index) else {
        return Err(invalid("offset piece missing"));
    };
    if piece.segments().is_empty() {
        return Err(invalid("offset produced no curve"));
    }
    Ok(piece.segments().to_vec())
}

fn orient_like(profile: &TurnProfile, mut segments: Vec<Segment>) -> Vec<Segment> {
    let (Some(first), Some(last)) = (segments.first(), segments.last()) else {
        return segments;
    };
    let along = last.end() - first.start();
    let original = profile.end() - profile.start();
    if along.dot(original) < 0.0 {
        segments = segments.into_iter().rev().map(Segment::reversed).collect();
    }
    segments
}

fn translate_chain(segments: &[Segment], shift_z: f64, shift_x: f64) -> Result<Vec<Segment>> {
    let mut out = Vec::with_capacity(segments.len());
    for segment in segments {
        out.push(translate_segment(*segment, shift_z, shift_x)?);
    }
    Ok(out)
}

fn translate_segment(segment: Segment, shift_z: f64, shift_x: f64) -> Result<Segment> {
    match segment {
        Segment::Line(line) => {
            let start = Point::new(line.start().x() + shift_z, line.start().y() + shift_x)?;
            let end = Point::new(line.end().x() + shift_z, line.end().y() + shift_x)?;
            Ok(Segment::Line(Line::new(start, end)?))
        }
        Segment::Arc(arc) => {
            let centre = Point::new(arc.center().x() + shift_z, arc.center().y() + shift_x)?;
            Ok(Segment::Arc(Arc::new(centre, arc.radius(), arc.start_angle(), arc.sweep())?))
        }
    }
}

fn ensure_contiguous(segments: &[Segment], tol: f64) -> Result<()> {
    let limit = tol.max(CONTIGUOUS_SLACK);
    for pair in segments.windows(2) {
        let (Some(prev), Some(next)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        let gap = prev.end().distance(next.start());
        if !gap.is_finite() || gap > limit {
            return Err(invalid(format!("compensated profile is not contiguous (gap {gap})")));
        }
    }
    Ok(())
}

fn chain_z_range(segments: &[Segment]) -> (f64, f64) {
    let mut z_min = f64::INFINITY;
    let mut z_max = f64::NEG_INFINITY;
    for segment in segments {
        let bounds = segment.bounding_box();
        z_min = z_min.min(bounds.min().x());
        z_max = z_max.max(bounds.max().x());
    }
    if z_min.is_finite() && z_max.is_finite() { (z_min, z_max) } else { (0.0, 0.0) }
}

fn chain_min_y(segments: &[Segment]) -> f64 {
    segments.iter().map(|segment| segment.bounding_box().min().y()).fold(f64::INFINITY, f64::min)
}

fn chain_max_y(segments: &[Segment]) -> f64 {
    segments.iter().map(|segment| segment.bounding_box().max().y()).fold(f64::NEG_INFINITY, f64::max)
}

/// The front of the chain is still on the free side of this pass, so the tool can start cutting.
fn pass_can_enter(side: Side, start_y: f64, radius: f64) -> bool {
    match side {
        Side::Outside => start_y <= radius + BLOCK_EPS,
        Side::Inside => start_y >= radius - BLOCK_EPS,
    }
}

/// Largest Z at which the boundary has crossed onto the material side of `radius`.
///
/// `None` means the whole chain is on the free side: the pass can run to the back.
fn blocking_z(segments: &[Segment], radius: f64, side: Side) -> Option<f64> {
    let mut best: Option<f64> = None;
    for segment in segments {
        let z = segment_blocking_z(*segment, radius, side);
        if let Some(z) = z {
            best = Some(best.map_or(z, |current| current.max(z)));
        }
    }
    best
}

fn segment_blocking_z(segment: Segment, radius: f64, side: Side) -> Option<f64> {
    match segment {
        Segment::Line(line) => line_blocking_z(line, radius, side),
        Segment::Arc(arc) => arc_blocking_z(arc, radius, side),
    }
}

fn line_blocking_z(line: Line, radius: f64, side: Side) -> Option<f64> {
    let z0 = line.start().x();
    let y0 = line.start().y();
    let z1 = line.end().x();
    let y1 = line.end().y();
    let blocks = |y: f64| blocks_radius(side, y, radius);
    let hit0 = blocks(y0);
    let hit1 = blocks(y1);
    if !hit0 && !hit1 {
        return None;
    }
    if hit0 && hit1 {
        return Some(z0.max(z1));
    }
    let dy = y1 - y0;
    if dy.abs() <= 1e-15 {
        return if hit0 { Some(z0.max(z1)) } else { None };
    }
    // Supremum of the open blocking set sits on the threshold crossing.
    let threshold = match side {
        Side::Outside => radius + BLOCK_EPS,
        Side::Inside => radius - BLOCK_EPS,
    };
    let t = ((threshold - y0) / dy).clamp(0.0, 1.0);
    let z_cross = z0 + t * (z1 - z0);
    if hit0 { Some(z0.max(z_cross)) } else { Some(z1.max(z_cross)) }
}

fn blocks_radius(side: Side, y: f64, radius: f64) -> bool {
    match side {
        Side::Outside => y > radius + BLOCK_EPS,
        Side::Inside => y < radius - BLOCK_EPS,
    }
}

fn arc_blocking_z(arc: Arc, radius: f64, side: Side) -> Option<f64> {
    let threshold = match side {
        Side::Outside => radius + BLOCK_EPS,
        Side::Inside => radius - BLOCK_EPS,
    };
    let mut cuts = vec![0.0, 1.0];
    for theta in angles_at_y(arc, threshold) {
        if let Some(t) = param_of_angle(arc.start_angle(), arc.sweep(), theta) {
            cuts.push(t);
        }
    }
    if let Some(t) = param_of_angle(arc.start_angle(), arc.sweep(), 0.0) {
        cuts.push(t);
    }
    cuts.sort_by(|a, b| a.total_cmp(b));
    let mut best: Option<f64> = None;
    let mut previous: Option<f64> = None;
    for t_end in cuts {
        let Some(t_start) = previous else {
            previous = Some(t_end);
            continue;
        };
        previous = Some(t_end);
        if t_end - t_start <= 1e-12 {
            continue;
        }
        let mid = 0.5 * (t_start + t_end);
        let y_mid = arc.point_at(mid).y();
        if !blocks_radius(side, y_mid, radius) {
            continue;
        }
        for t in [t_start, t_end] {
            let z = arc.point_at(t).x();
            best = Some(best.map_or(z, |current| current.max(z)));
        }
        if let Some(t_right) = param_of_angle(arc.start_angle(), arc.sweep(), 0.0)
            && t_right >= t_start - 1e-9
            && t_right <= t_end + 1e-9
        {
            let z = arc.point_at(t_right).x();
            best = Some(best.map_or(z, |current| current.max(z)));
        }
    }
    best
}

fn angles_at_y(arc: Arc, y: f64) -> Vec<f64> {
    let ratio = (y - arc.center().y()) / arc.radius();
    if !ratio.is_finite() || ratio.abs() > 1.0 + 1e-9 {
        return Vec::new();
    }
    let sine = ratio.clamp(-1.0, 1.0);
    let angle = sine.asin();
    vec![angle, std::f64::consts::PI - angle]
}

/// `t` in `[0, 1]` when `theta` lies on the signed sweep, otherwise `None`.
fn param_of_angle(start: f64, sweep: f64, theta: f64) -> Option<f64> {
    const TAU: f64 = std::f64::consts::TAU;
    if !theta.is_finite() || !start.is_finite() || !sweep.is_finite() {
        return None;
    }
    let abs_sweep = sweep.abs();
    if abs_sweep < 1e-15 {
        return None;
    }
    if abs_sweep >= TAU - 1e-8 {
        let turn = if sweep >= 0.0 { (theta - start).rem_euclid(TAU) } else { (start - theta).rem_euclid(TAU) };
        return Some((turn / TAU).clamp(0.0, 1.0));
    }
    let along = if sweep > 0.0 { (theta - start).rem_euclid(TAU) } else { (start - theta).rem_euclid(TAU) };
    let slack = 1e-8;
    let adjusted = if TAU - along <= slack { 0.0 } else { along };
    if adjusted <= abs_sweep + slack { Some((adjusted / abs_sweep).clamp(0.0, 1.0)) } else { None }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Dir {
    Forward,
    Back,
}

/// Feed from the hit toward `previous` along the boundary, climbing out of the pass.
fn follow_to_previous(
    segments: &[Segment],
    z_hit: f64,
    radius: f64,
    previous: f64,
    side: Side,
    moves: &mut Vec<TurnMove>,
    cursor: &mut (f64, f64),
) -> Result<()> {
    let Some(hit) = locate_hit(segments, z_hit, radius) else {
        return Ok(());
    };
    let dir = choose_dir(segments, hit.0, hit.1, side, previous);
    let mut index = hit.0;
    let mut t = hit.1;
    let mut guard = 0usize;
    let limit = segments.len().saturating_add(2);
    while guard < limit {
        guard += 1;
        let Some(segment) = segments.get(index).copied() else {
            break;
        };
        let t_far = match dir {
            Dir::Forward => 1.0,
            Dir::Back => 0.0,
        };
        if (t - t_far).abs() <= 1e-12 {
            match advance(index, dir, segments.len()) {
                Some(next) => {
                    index = next;
                    t = match dir {
                        Dir::Forward => 0.0,
                        Dir::Back => 1.0,
                    };
                    continue;
                }
                None => break,
            }
        }
        let y_here = point_at(segment, t).1;
        if reached(side, y_here, previous) {
            break;
        }
        let t_best = arg_best_y(segment, t, t_far, side);
        let y_best = point_at(segment, t_best).1;
        if !improves(side, y_here, y_best) {
            break;
        }
        let t_stop = crossing_between(segment, previous, t, t_best).unwrap_or(t_best);
        emit_partial(segment, t, t_stop, moves, cursor)?;
        let y_stop = point_at(segment, t_stop).1;
        let done = reached(side, y_stop, previous) || (t_stop - t_far).abs() > 1e-9;
        t = t_stop;
        if done {
            break;
        }
    }
    Ok(())
}

fn locate_hit(segments: &[Segment], z_hit: f64, radius: f64) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64, f64)> = None;
    for (index, segment) in segments.iter().copied().enumerate() {
        for t in params_at_y(segment, radius) {
            let (z, x) = point_at(segment, t);
            let gap = (z - z_hit).hypot(x - radius);
            if gap > 1e-3 {
                continue;
            }
            let replace = best.is_none_or(|(_, _, current)| gap < current);
            if replace {
                best = Some((index, t, gap));
            }
        }
    }
    best.map(|(index, t, _)| (index, t))
}

fn choose_dir(segments: &[Segment], index: usize, t: f64, side: Side, previous: f64) -> Dir {
    let Some(segment) = segments.get(index).copied() else {
        return Dir::Forward;
    };
    let y0 = point_at(segment, t).1;
    let score = |y: Option<f64>| -> f64 {
        let Some(y) = y else {
            return f64::NEG_INFINITY;
        };
        match side {
            Side::Outside => y - y0,
            Side::Inside => y0 - y,
        }
    };
    // Prefer the direction that actually reaches the previous level when both climb.
    let forward = neighbor_y(segments, index, t, Dir::Forward);
    let back = neighbor_y(segments, index, t, Dir::Back);
    let forward_score = score(forward);
    let back_score = score(back);
    if (forward_score - back_score).abs() <= 1e-9 {
        let forward_hit = forward.is_some_and(|y| reached(side, y, previous));
        let back_hit = back.is_some_and(|y| reached(side, y, previous));
        if back_hit && !forward_hit { Dir::Back } else { Dir::Forward }
    } else if forward_score >= back_score {
        Dir::Forward
    } else {
        Dir::Back
    }
}

fn neighbor_y(segments: &[Segment], index: usize, t: f64, dir: Dir) -> Option<f64> {
    const STEP: f64 = 1e-3;
    let segment = segments.get(index).copied()?;
    let t2 = match dir {
        Dir::Forward => t + STEP,
        Dir::Back => t - STEP,
    };
    if (0.0..=1.0).contains(&t2) {
        return Some(point_at(segment, t2).1);
    }
    let next = match dir {
        Dir::Forward => index.checked_add(1)?,
        Dir::Back => index.checked_sub(1)?,
    };
    let segment = segments.get(next).copied()?;
    let t_next = match dir {
        Dir::Forward => STEP,
        Dir::Back => 1.0 - STEP,
    };
    Some(point_at(segment, t_next.clamp(0.0, 1.0)).1)
}

fn advance(index: usize, dir: Dir, len: usize) -> Option<usize> {
    match dir {
        Dir::Forward => {
            let next = index.checked_add(1)?;
            if next < len { Some(next) } else { None }
        }
        Dir::Back => index.checked_sub(1),
    }
}

fn reached(side: Side, y: f64, previous: f64) -> bool {
    match side {
        Side::Outside => y >= previous - 1e-6,
        Side::Inside => y <= previous + 1e-6,
    }
}

fn improves(side: Side, y_from: f64, y_to: f64) -> bool {
    match side {
        Side::Outside => y_to > y_from + 1e-9,
        Side::Inside => y_to < y_from - 1e-9,
    }
}

fn arg_best_y(segment: Segment, t_from: f64, t_to: f64, side: Side) -> f64 {
    let mut best_t = t_from;
    let mut best_y = point_at(segment, t_from).1;
    let consider = |t: f64, best_t: &mut f64, best_y: &mut f64| {
        if !between_directed(t, t_from, t_to) {
            return;
        }
        let y = point_at(segment, t).1;
        let better = match side {
            Side::Outside => y > *best_y,
            Side::Inside => y < *best_y,
        };
        if better {
            *best_t = t;
            *best_y = y;
        }
    };
    consider(t_to, &mut best_t, &mut best_y);
    if let Segment::Arc(arc) = segment {
        let theta = match side {
            Side::Outside => std::f64::consts::FRAC_PI_2,
            Side::Inside => -std::f64::consts::FRAC_PI_2,
        };
        if let Some(t) = param_of_angle(arc.start_angle(), arc.sweep(), theta) {
            consider(t, &mut best_t, &mut best_y);
        }
    }
    best_t
}

fn between_directed(t: f64, t_from: f64, t_to: f64) -> bool {
    let low = t_from.min(t_to) - 1e-9;
    let high = t_from.max(t_to) + 1e-9;
    t >= low && t <= high
}

fn crossing_between(segment: Segment, y: f64, t_from: f64, t_to: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    for t in params_at_y(segment, y) {
        if !between_directed(t, t_from, t_to) {
            continue;
        }
        let gap = (t - t_from).abs();
        let replace = best.is_none_or(|current| gap < (current - t_from).abs());
        if replace {
            best = Some(t);
        }
    }
    best.filter(|t| (*t - t_from).abs() > 1e-12)
}

fn params_at_y(segment: Segment, y: f64) -> Vec<f64> {
    match segment {
        Segment::Line(line) => line_params_at_y(line, y),
        Segment::Arc(arc) => {
            let mut found = Vec::new();
            for theta in angles_at_y(arc, y) {
                if let Some(t) = param_of_angle(arc.start_angle(), arc.sweep(), theta) {
                    found.push(t);
                }
            }
            found
        }
    }
}

fn line_params_at_y(line: Line, y: f64) -> Vec<f64> {
    let y0 = line.start().y();
    let y1 = line.end().y();
    let dy = y1 - y0;
    if dy.abs() <= 1e-12 {
        if (y0 - y).abs() <= 1e-6 {
            let z0 = line.start().x();
            let z1 = line.end().x();
            // A horizontal span lies on the pass. Report both ends; the caller picks the Z.
            if (z0 - z1).abs() <= 1e-12 { vec![0.0] } else { vec![0.0, 1.0] }
        } else {
            Vec::new()
        }
    } else {
        let t = (y - y0) / dy;
        if (0.0..=1.0).contains(&t) { vec![t] } else { Vec::new() }
    }
}

fn point_at(segment: Segment, t: f64) -> (f64, f64) {
    match segment {
        Segment::Line(line) => {
            let z = line.start().x() + t * (line.end().x() - line.start().x());
            let x = line.start().y() + t * (line.end().y() - line.start().y());
            (z, x)
        }
        Segment::Arc(arc) => {
            let point = arc.point_at(t.clamp(0.0, 1.0));
            (point.x(), point.y())
        }
    }
}

fn emit_partial(segment: Segment, t_from: f64, t_to: f64, moves: &mut Vec<TurnMove>, cursor: &mut (f64, f64)) -> Result<()> {
    if (t_to - t_from).abs() <= 1e-12 {
        return Ok(());
    }
    let (z, x) = point_at(segment, t_to);
    let motion = match segment {
        Segment::Line(_) => TurnMove::Feed { z, x },
        Segment::Arc(arc) => {
            let along = t_to >= t_from;
            let ccw = if along { arc.sweep() > 0.0 } else { arc.sweep() < 0.0 };
            TurnMove::Arc { z, x, cz: arc.center().x(), cx: arc.center().y(), ccw }
        }
    };
    move_to(moves, cursor, motion, false)
}

fn push_segment(moves: &mut Vec<TurnMove>, cursor: &mut (f64, f64), segment: Segment) -> Result<()> {
    let end = segment.end();
    let motion = match segment {
        Segment::Line(_) => TurnMove::Feed { z: end.x(), x: end.y() },
        Segment::Arc(arc) => TurnMove::Arc { z: end.x(), x: end.y(), cz: arc.center().x(), cx: arc.center().y(), ccw: arc.sweep() > 0.0 },
    };
    move_to(moves, cursor, motion, true)
}

fn safe_from(x: f64, free: f64, retract: f64) -> f64 {
    (x + free * retract).max(0.0)
}

fn move_target(motion: TurnMove) -> (f64, f64) {
    match motion {
        TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => (z, x),
    }
}

fn move_to(moves: &mut Vec<TurnMove>, cursor: &mut (f64, f64), motion: TurnMove, allow_negative: bool) -> Result<()> {
    let (z, x) = move_target(motion);
    if !z.is_finite() || !x.is_finite() {
        return Err(invalid("move coordinate is not finite"));
    }
    if !allow_negative && x < -1e-8 {
        return Err(invalid("radius is negative"));
    }
    if (z - cursor.0).hypot(x - cursor.1) <= 1e-12 {
        return Ok(());
    }
    push(moves, motion)?;
    *cursor = (z, x);
    Ok(())
}

fn push(moves: &mut Vec<TurnMove>, motion: TurnMove) -> Result<()> {
    if moves.len() >= MAX_MOVES {
        return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
    }
    let (z, x) = move_target(motion);
    finite(z, "move z")?;
    finite(x, "move x")?;
    moves.push(motion);
    Ok(())
}

#[cfg(test)]
#[path = "ops_tests.rs"]
mod tests;
