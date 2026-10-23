//! Threading, grooving, parting and centreline drilling.
//!
//! Coordinates match [`crate::ops`]: Z along the spindle, positive away from the chuck, with
//! Z = 0 on the finished front face, and X as a radius in millimetres. Paths are the programmed
//! point. Nothing here emits G41/G42.
//!
//! # Threading
//!
//! [`thread`] writes one explicit pass per radial depth from [`thread_pass_depths`], then
//! [`ThreadParams::spring_passes`] extra passes at the final depth. Each pass rapids to the cut
//! radius in front of the start, runs a spindle-synchronised [`TurnMove::Thread`] at the pitch,
//! rapids out to the clearance radius and rapids back to the start Z.
//!
//! Radial infeed keeps that start Z. Flank infeed shifts it by `-depth * tan(angle)` so the tool
//! follows one flank (29.5° is the usual choice for a 60° metric form). Alternating flank flips
//! the sign each cutting pass, starting with the same negative shift. Spring passes copy the last
//! cutting pass, including its flank side.
//!
//! `major_radius` is the external major radius, or the internal minor (the bore) for
//! [`Side::Inside`]. Inside passes mirror the radius: the tool steps out from the bore and
//! retracts toward the axis. Canned G76 is a post option for later. This module does not emit it.
//!
//! # Grooving and parting
//!
//! The programmed point is the **left corner** of the insert (the corner at smaller Z). An insert
//! of width `w` with that corner at `z` covers `[z, z + w]`. [`groove`] plunges at stepped Z so
//! those footprints union to `[z_left, z_right]`. The last plunge puts the corner at
//! `z_right - w`, so the right edge lands on `z_right`. A usual stepover is `0.8` of the width.
//! When the groove is wider than the insert, a finish feed then runs along the floor. Nose-radius
//! compensation is not applied: the caller already programs the corner. [`part_off`] uses the
//! same corner and plunges at `z` (the left edge of the kerf).
//!
//! # Centreline drilling
//!
//! [`centre_drill`] keeps X at 0. `depth` is how far the **tip** goes past `z_start` along −Z.
//! `through` adds the conical tip, `radius / tan(point_angle / 2)`, so the full diameter
//! advances by `depth`. 118° is the usual twist-drill point. Between pecks the tip rapids back by
//! `retract`, and never past the clearance plane.
//!
//! # IR
//!
//! [`crate::to_cl_path`] writes these paths as cutter-location blocks. [`TurnMove::Dwell`] is an
//! [`campanion_ir::Event::Dwell`]. [`TurnMove::Thread`] is [`campanion_ir::ClMotion::Thread`]: a
//! straight move with a lead in millimetres per spindle revolution. It is not a linear cut, so a
//! post cannot drop the spindle lock by emitting an ordinary feed.
//!
//! This crate is clean-room. It does not use OCCT, FreeCAD, planegcs, or OpenCAMLib.

use crate::ops::{self, MAX_MOVES, Side, TurnMove, TurnPath, TurnTool};
use crate::{Result, TurnError};

/// A step this close to the minimum depth still counts as a full minimum step.
///
/// `0.1` is not exact in binary, so `1.0 - 0.9` can fall a couple of ulps short of `min`
/// and would otherwise fold a legal pass away.
const STEP_EPS: f64 = 1e-9;

/// How the threading tool steps into the flank.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Infeed {
    /// Straight in, at the same Z every pass.
    Radial,
    /// Compound infeed along one flank. `angle_deg` is usually 29.5 for a 60° metric thread.
    Flank { angle_deg: f64 },
    /// Same as [`Infeed::Flank`], but the axial shift changes sign on each cutting pass.
    AlternatingFlank { angle_deg: f64 },
}

/// Single-point thread, external or internal.
///
/// Depths are radial. `pitch` is the lead in millimetres per revolution. The cut runs toward
/// the chuck, so `z_start` is greater than `z_end`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThreadParams {
    /// OD or ID. ID mirrors the radius.
    pub side: Side,
    /// OD major radius, or ID minor radius (the bore).
    pub major_radius: f64,
    /// Radial thread depth. On an OD this must be less than [`Self::major_radius`].
    pub depth: f64,
    /// Lead, millimetres per revolution.
    pub pitch: f64,
    /// Z where the thread starts, on the tailstock side of [`Self::z_end`].
    pub z_start: f64,
    /// Z where the synchronised cut ends, toward the chuck.
    pub z_end: f64,
    /// Radial depth of the first constant-area pass.
    pub first_depth: f64,
    /// Smallest radial increment. A shorter remainder is folded into the previous pass.
    pub min_depth: f64,
    /// Extra passes at the final depth, after the pass that first reaches [`Self::depth`].
    pub spring_passes: u8,
    /// Radial, one flank, or alternating flanks.
    pub infeed: Infeed,
    /// Axial room in front of [`Self::z_start`], and the radial pull-off past the reference radius.
    pub clearance: f64,
    /// Included thread angle, degrees. 60 for a metric form. Flank infeed uses [`Self::infeed`]'s
    /// own angle. A later G76 post reads this value.
    pub included_angle_deg: f64,
}

/// Radial grooving with a flat insert.
///
/// `z_left < z_right` are the groove walls. The programmed point is the insert's left corner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GrooveParams {
    /// OD groove cuts inward. ID groove cuts outward.
    pub side: Side,
    /// Left wall, smaller Z.
    pub z_left: f64,
    /// Right wall, larger Z.
    pub z_right: f64,
    /// Floor radius. Below [`Self::surface_radius`] on an OD, above it in a bore.
    pub bottom_radius: f64,
    /// Radius of the surface the groove is cut from.
    pub surface_radius: f64,
    /// Width of the insert along Z. Must be at most the groove width.
    pub insert_width: f64,
    /// Radial peck. `None` feeds to the floor in one move.
    pub peck: Option<f64>,
    /// Axial step as a fraction of [`Self::insert_width`], in `(0, 1]`. `0.8` is the usual overlap.
    pub stepover_frac: f64,
    /// Radial room on the free side of the surface.
    pub clearance: f64,
    /// Pause at the floor of each plunge, in seconds. Zero emits no dwell.
    pub dwell_s: f64,
}

/// Part-off plunge. The programmed point is the insert's left corner, at [`Self::z`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartParams {
    /// Left edge of the kerf. The insert covers `[z, z + insert_width]`.
    pub z: f64,
    /// Bar radius before the cut.
    pub surface_radius: f64,
    /// Insert width along Z. The path does not offset by it; the corner stays at [`Self::z`].
    pub insert_width: f64,
    /// Radius to feed to. Zero reaches the axis. Negative passes the centre. A small positive value leaves a pip.
    pub to_radius: f64,
    /// 45° corner break before the plunge, in millimetres along Z and along the radius. `None` skips it.
    pub chamfer: Option<f64>,
    /// Radial peck. `None` feeds to [`Self::to_radius`] in one move.
    pub peck: Option<f64>,
    /// Extra radius beyond [`Self::surface_radius`] for the approach and the retract.
    pub clearance: f64,
}

/// Centreline drill. Every move is at X = 0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DrillParams {
    /// Z of the face the drill starts from.
    pub z_start: f64,
    /// Tip travel past [`Self::z_start`] along −Z. [`Self::through`] adds the point length on top.
    pub depth: f64,
    /// Axial peck. `None` is one feed.
    pub peck: Option<f64>,
    /// Pull-back between pecks, toward +Z. Clamped so the tip does not pass the clearance plane.
    pub retract: f64,
    /// Distance in front of [`Self::z_start`] for the approach and the final retract.
    pub clearance: f64,
    /// Included point angle, degrees. Pass `118.0` for a standard twist drill.
    pub point_angle_deg: f64,
    /// Drill diameter, millimetres. Used for the through-hole point allowance.
    pub diameter: f64,
    /// When set, the tip goes deeper by the point length so the full diameter advances by [`Self::depth`].
    pub through: bool,
}

impl ThreadParams {
    /// Reject non-finite numbers, a non-positive pitch or depth, a reversed cut, or an OD depth
    /// that would cross the axis.
    pub fn validate(&self) -> Result<()> {
        positive(self.major_radius, "major radius")?;
        positive(self.depth, "thread depth")?;
        positive(self.pitch, "pitch")?;
        positive(self.first_depth, "first thread depth")?;
        positive(self.min_depth, "minimum thread depth")?;
        finite(self.z_start, "thread start z")?;
        finite(self.z_end, "thread end z")?;
        non_negative(self.clearance, "clearance")?;
        finite(self.included_angle_deg, "included angle")?;
        if !(self.included_angle_deg > 0.0 && self.included_angle_deg < 180.0) {
            return Err(invalid("included angle must be between 0 and 180 degrees"));
        }
        if !(self.z_start > self.z_end) {
            return Err(invalid("thread z_start must be greater than z_end"));
        }
        if self.side == Side::Outside && self.depth >= self.major_radius {
            return Err(invalid("thread depth must be less than the major radius"));
        }
        if let Some(angle) = infeed_angle(self.infeed) {
            finite(angle, "infeed angle")?;
            if !(angle > 0.0 && angle < 90.0) {
                return Err(invalid("infeed angle must be between 0 and 90 degrees"));
            }
        }
        Ok(())
    }
}

impl GrooveParams {
    /// Reject a reversed groove, an insert that does not fit, or a floor on the wrong side of the surface.
    pub fn validate(&self) -> Result<()> {
        finite(self.z_left, "groove z")?;
        finite(self.z_right, "groove z")?;
        if !(self.z_left < self.z_right) {
            return Err(invalid("groove z_left must be less than z_right"));
        }
        positive(self.insert_width, "insert width")?;
        let span = self.z_right - self.z_left;
        if !span.is_finite() || self.insert_width > span {
            return Err(invalid("insert is wider than the groove"));
        }
        positive(self.surface_radius, "surface radius")?;
        non_negative(self.bottom_radius, "bottom radius")?;
        match self.side {
            Side::Outside => {
                if !(self.bottom_radius < self.surface_radius) {
                    return Err(invalid("groove bottom must be below the surface"));
                }
            }
            Side::Inside => {
                if !(self.bottom_radius > self.surface_radius) {
                    return Err(invalid("internal groove bottom must be outside the bore"));
                }
            }
        }
        finite(self.stepover_frac, "stepover fraction")?;
        if !(self.stepover_frac > 0.0 && self.stepover_frac <= 1.0) {
            return Err(invalid("stepover fraction must be in (0, 1]"));
        }
        non_negative(self.clearance, "clearance")?;
        non_negative(self.dwell_s, "dwell")?;
        if let Some(peck) = self.peck {
            positive(peck, "peck")?;
        }
        Ok(())
    }
}

impl PartParams {
    /// Reject a non-positive surface, a target that is not below it, or a chamfer that passes the target.
    pub fn validate(&self) -> Result<()> {
        finite(self.z, "part z")?;
        positive(self.surface_radius, "surface radius")?;
        positive(self.insert_width, "insert width")?;
        finite(self.to_radius, "part-off radius")?;
        if !(self.to_radius < self.surface_radius) {
            return Err(invalid("part-off radius must be below the surface"));
        }
        non_negative(self.clearance, "clearance")?;
        if let Some(chamfer) = self.chamfer {
            positive(chamfer, "chamfer")?;
            if self.surface_radius - chamfer < self.to_radius {
                return Err(invalid("chamfer passes the part-off radius"));
            }
        }
        if let Some(peck) = self.peck {
            positive(peck, "peck")?;
        }
        Ok(())
    }
}

impl DrillParams {
    /// Reject a non-positive depth or diameter, a point angle that is not in `(0, 180)`, or a negative pull-back.
    pub fn validate(&self) -> Result<()> {
        finite(self.z_start, "drill start z")?;
        positive(self.depth, "drill depth")?;
        non_negative(self.retract, "retract")?;
        non_negative(self.clearance, "clearance")?;
        positive(self.diameter, "drill diameter")?;
        finite(self.point_angle_deg, "point angle")?;
        if !(self.point_angle_deg > 0.0 && self.point_angle_deg < 180.0) {
            return Err(invalid("point angle must be between 0 and 180 degrees"));
        }
        if let Some(peck) = self.peck {
            positive(peck, "peck")?;
        }
        Ok(())
    }
}

/// Cumulative radial depths for constant-area thread infeed.
///
/// Pass `n` (1-based) would ideally sit at `first * n.sqrt()`. The chip area stays roughly
/// constant because the increment is `first * (n.sqrt() - (n - 1).sqrt())`. Values are capped
/// at `depth`. An increment smaller than `min` is raised to `min`, and once that happens later
/// passes keep stepping by `min`. A remainder smaller than `min` is folded into the previous
/// pass, so the last value is exactly `depth`.
///
/// When `depth <= first` there is one pass at `depth`. That step may be shorter than `min`.
/// A single pass is also used when `depth < min`, because no legal step can both reach `depth`
/// and stay at least `min`.
pub fn thread_pass_depths(depth: f64, first: f64, min: f64) -> Result<Vec<f64>> {
    positive(depth, "thread depth")?;
    positive(first, "first thread depth")?;
    positive(min, "minimum thread depth")?;
    if depth <= first {
        return Ok(vec![depth]);
    }

    let mut depths = Vec::new();
    let mut prev = 0.0;
    let mut n: u32 = 1;
    let mut minimum_steps = false;
    loop {
        if depths.len() >= MAX_MOVES {
            return Err(invalid(format!("too many thread passes: limit is {MAX_MOVES}")));
        }
        let remaining = depth - prev;
        // Not enough stock left for its own pass. A nanometre short of `min` is still a full step.
        if remaining + STEP_EPS < min {
            if let Some(last) = depths.last_mut() {
                *last = depth;
            } else {
                depths.push(depth);
            }
            break;
        }
        let mut next = if minimum_steps {
            prev + min
        } else {
            let ideal = first * f64::from(n).sqrt();
            if !ideal.is_finite() {
                return Err(invalid("thread pass depth is not finite"));
            }
            if ideal - prev + STEP_EPS < min {
                minimum_steps = true;
                prev + min
            } else {
                ideal
            }
        };
        if !next.is_finite() {
            return Err(invalid("thread pass depth is not finite"));
        }
        if next > depth {
            next = depth;
        }
        // Taking `next` would leave a crumb shorter than `min`. Finish on this pass.
        if depth - next + STEP_EPS < min {
            next = depth;
        }
        if next <= prev {
            return Err(invalid("thread pass did not advance"));
        }
        let finished = next >= depth;
        depths.push(if finished { depth } else { next });
        if finished {
            break;
        }
        prev = next;
        if !minimum_steps {
            n = n.checked_add(1).ok_or_else(|| invalid("too many thread passes"))?;
        }
    }
    Ok(depths)
}

/// Explicit threading passes for `p`.
///
/// The path starts at `(z_start + clearance, clearance radius)`. The clearance radius is
/// `major_radius + clearance` outside and `(major_radius - clearance).max(0)` inside.
/// Each cutting pass, and each spring pass, ends with the tool back at that park point.
pub fn thread(p: &ThreadParams) -> Result<TurnPath> {
    p.validate()?;
    let cutting = thread_pass_depths(p.depth, p.first_depth, p.min_depth)?;
    let last_index = cutting.len().saturating_sub(1);
    let Some(final_depth) = cutting.last().copied() else {
        return Err(invalid("thread pass depths are empty"));
    };
    let mut depths = cutting;
    let spring = usize::from(p.spring_passes);
    if depths.len().saturating_add(spring) > MAX_MOVES / 4 {
        return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
    }
    for _ in 0..spring {
        depths.push(final_depth);
    }
    let tan = flank_tan(p.infeed)?;
    let safe_x = match p.side {
        Side::Outside => p.major_radius + p.clearance,
        Side::Inside => (p.major_radius - p.clearance).max(0.0),
    };
    let park_z = p.z_start + p.clearance;
    finite(safe_x, "thread clearance radius")?;
    finite(park_z, "thread start")?;
    let mut path = Builder::new((park_z, safe_x), false)?;
    for (index, depth) in depths.iter().copied().enumerate() {
        // Spring passes repeat the last cutting pass, flank side included.
        let shift_index = index.min(last_index);
        let z_entry = park_z + flank_offset(p.infeed, tan, shift_index, depth)?;
        finite(z_entry, "thread entry")?;
        if !(z_entry > p.z_end) {
            return Err(invalid("infeed shift reaches the thread end"));
        }
        let x_cut = match p.side {
            Side::Outside => p.major_radius - depth,
            Side::Inside => p.major_radius + depth,
        };
        finite(x_cut, "thread radius")?;
        if x_cut < 0.0 {
            return Err(invalid("thread radius is negative"));
        }
        path.rapid(z_entry, safe_x)?;
        path.rapid(z_entry, x_cut)?;
        path.thread_to(p.z_end, x_cut, p.pitch)?;
        path.rapid(p.z_end, safe_x)?;
        path.rapid(park_z, safe_x)?;
    }
    Ok(path.into_path())
}

/// Groove from [`GrooveParams::z_left`] to [`GrooveParams::z_right`].
///
/// Plunges run from the left wall toward the right. Between pecks the tool rapids to the
/// clearance radius and back to the depth it just reached. After the last plunge, when the
/// insert is narrower than the groove, the corner feeds along the floor back to `z_left`.
pub fn groove(tool: &TurnTool, p: &GrooveParams) -> Result<TurnPath> {
    tool.validate()?;
    p.validate()?;
    let positions = plunge_positions(p.z_left, p.z_right, p.insert_width, p.stepover_frac)?;
    let x_floor = p.bottom_radius;
    let x_safe = clearance_radius(p.side, p.surface_radius, p.clearance)?;
    let Some(first) = positions.first().copied() else {
        return Err(invalid("groove has no plunge"));
    };
    let mut path = Builder::new((first, x_safe), false)?;
    let count = positions.len();
    for (index, z) in positions.iter().copied().enumerate() {
        let finish_along_floor = index + 1 == count && count > 1;
        path.cut_radial(z, x_floor, x_safe, p.peck, p.dwell_s, !finish_along_floor)?;
        if finish_along_floor {
            path.feed(first, x_floor)?;
            path.rapid(first, x_safe)?;
        }
    }
    Ok(path.into_path())
}

/// Part off at [`PartParams::z`].
///
/// A chamfer of length `c` is a feed from `(z + c, surface)` to `(z, surface - c)`, equal steps
/// in Z and in radius, before the straight plunge. The plunge reaches `to_radius` exactly,
/// including a negative value past the centre. Each peck pulls back to the clearance radius.
pub fn part_off(tool: &TurnTool, p: &PartParams) -> Result<TurnPath> {
    tool.validate()?;
    p.validate()?;
    let chamfer = p.chamfer.unwrap_or(0.0);
    let x_safe = p.surface_radius + p.clearance;
    finite(x_safe, "clearance radius")?;
    // Past-centre is a radius with a sign so the corner can cross X = 0.
    let mut path = Builder::new((p.z + chamfer, x_safe), true)?;
    if chamfer > 0.0 {
        path.feed(p.z + chamfer, p.surface_radius)?;
        path.feed(p.z, p.surface_radius - chamfer)?;
    }
    path.cut_radial(p.z, p.to_radius, x_safe, p.peck, 0.0, true)?;
    Ok(path.into_path())
}

/// Drill on the spindle axis.
///
/// The path starts at `(z_start + clearance, 0)`. Peck feeds advance the tip; their new depth
/// sums to the travel. `through` sets that travel to `depth + radius / tan(point_angle / 2)`.
/// A blind hole stops the tip at `z_start - depth`.
pub fn centre_drill(p: &DrillParams) -> Result<TurnPath> {
    p.validate()?;
    let tip = tip_length(p.diameter, p.point_angle_deg)?;
    let travel = if p.through { p.depth + tip } else { p.depth };
    finite(travel, "drill travel")?;
    if travel <= 0.0 {
        return Err(invalid("drill travel must be positive"));
    }
    let z_clear = p.z_start + p.clearance;
    let z_end = p.z_start - travel;
    finite(z_clear, "drill clearance")?;
    finite(z_end, "drill end")?;
    let mut path = Builder::new((z_clear, 0.0), false)?;
    let mut done = 0.0;
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > MAX_MOVES {
            return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
        }
        let remaining = travel - done;
        if remaining <= 1e-15 {
            break;
        }
        let step = match p.peck {
            Some(peck) => peck.min(remaining),
            None => remaining,
        };
        if !(step > 0.0) {
            return Err(invalid("peck did not advance"));
        }
        done += step;
        let at_end = travel - done <= 1e-12;
        if at_end {
            done = travel;
        }
        let z = if at_end { z_end } else { p.z_start - done };
        path.feed(z, 0.0)?;
        if at_end {
            break;
        }
        if p.retract > 0.0 {
            let z_back = (z + p.retract).min(z_clear);
            path.rapid(z_back, 0.0)?;
        }
    }
    path.rapid(z_clear, 0.0)?;
    Ok(path.into_path())
}

/// Axial length from the drill point to the full diameter.
fn tip_length(diameter: f64, point_angle_deg: f64) -> Result<f64> {
    positive(diameter, "drill diameter")?;
    finite(point_angle_deg, "point angle")?;
    if !(point_angle_deg > 0.0 && point_angle_deg < 180.0) {
        return Err(invalid("point angle must be between 0 and 180 degrees"));
    }
    let half = point_angle_deg.to_radians() * 0.5;
    let tangent = half.tan();
    if !tangent.is_finite() || tangent <= 0.0 {
        return Err(invalid("point angle has no usable tangent"));
    }
    let length = (diameter * 0.5) / tangent;
    if !length.is_finite() || length < 0.0 {
        return Err(invalid("drill tip length is not finite"));
    }
    Ok(length)
}

fn infeed_angle(infeed: Infeed) -> Option<f64> {
    match infeed {
        Infeed::Radial => None,
        Infeed::Flank { angle_deg } | Infeed::AlternatingFlank { angle_deg } => Some(angle_deg),
    }
}

fn flank_tan(infeed: Infeed) -> Result<Option<f64>> {
    let Some(angle) = infeed_angle(infeed) else {
        return Ok(None);
    };
    let tan = angle.to_radians().tan();
    if !tan.is_finite() || tan <= 0.0 {
        return Err(invalid("infeed angle has no usable tangent"));
    }
    Ok(Some(tan))
}

fn flank_offset(infeed: Infeed, tan: Option<f64>, index: usize, depth: f64) -> Result<f64> {
    let Some(tan) = tan else {
        return Ok(0.0);
    };
    let sign = match infeed {
        Infeed::Radial => 0.0,
        Infeed::Flank { .. } => -1.0,
        Infeed::AlternatingFlank { .. } => {
            if index.is_multiple_of(2) {
                -1.0
            } else {
                1.0
            }
        }
    };
    let shift = sign * depth * tan;
    finite(shift, "infeed shift")?;
    Ok(shift)
}

/// Left-corner Z of each plunge. The last entry is exactly `z_right - width`.
fn plunge_positions(z_left: f64, z_right: f64, width: f64, stepover_frac: f64) -> Result<Vec<f64>> {
    let last = z_right - width;
    if last + 1e-9 < z_left {
        return Err(invalid("insert is wider than the groove"));
    }
    if last <= z_left + 1e-9 {
        return Ok(vec![z_left]);
    }
    let step = stepover_frac * width;
    positive(step, "groove stepover")?;
    let mut positions = Vec::new();
    let mut z = z_left;
    loop {
        if positions.len() >= MAX_MOVES {
            return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
        }
        if z + 1e-9 >= last {
            break;
        }
        positions.push(z);
        let next = z + step;
        if !(next > z) {
            return Err(invalid("groove stepover did not advance"));
        }
        z = next;
    }
    positions.push(last);
    Ok(positions)
}

fn clearance_radius(side: Side, surface: f64, clearance: f64) -> Result<f64> {
    let safe = match side {
        Side::Outside => surface + clearance,
        Side::Inside => (surface - clearance).max(0.0),
    };
    finite(safe, "clearance radius")?;
    if safe < -1e-9 {
        return Err(invalid("radius is negative"));
    }
    Ok(safe)
}

struct Builder {
    moves: Vec<TurnMove>,
    start: (f64, f64),
    cursor: (f64, f64),
    allow_negative: bool,
}

impl Builder {
    fn new(start: (f64, f64), allow_negative: bool) -> Result<Self> {
        finite(start.0, "start z")?;
        finite(start.1, "start x")?;
        if !allow_negative && start.1 < -1e-9 {
            return Err(invalid("radius is negative"));
        }
        Ok(Self { moves: Vec::new(), start, cursor: start, allow_negative })
    }

    fn into_path(self) -> TurnPath {
        TurnPath { moves: self.moves, start: self.start }
    }

    fn rapid(&mut self, z: f64, x: f64) -> Result<()> {
        self.go(TurnMove::Rapid { z, x })
    }

    fn feed(&mut self, z: f64, x: f64) -> Result<()> {
        self.go(TurnMove::Feed { z, x })
    }

    fn thread_to(&mut self, z: f64, x: f64, lead: f64) -> Result<()> {
        self.go(TurnMove::Thread { z, x, lead })
    }

    fn dwell(&mut self, seconds: f64) -> Result<()> {
        self.go(TurnMove::Dwell { seconds })
    }

    /// Feed from the current radius to `x_target` at `z`, in pecks of `peck` when set.
    ///
    /// Intermediate pecks rapid out to `x_safe` and back. The move that reaches `x_target`
    /// is followed by a dwell when `dwell_s > 0`, then by a rapid to `x_safe` when
    /// `retract_after` is set. The floor itself is not crossed.
    fn cut_radial(&mut self, z: f64, x_target: f64, x_safe: f64, peck: Option<f64>, dwell_s: f64, retract_after: bool) -> Result<()> {
        self.rapid(z, self.cursor.1)?;
        finite(x_target, "plunge radius")?;
        finite(x_safe, "clearance radius")?;
        if !self.allow_negative && x_target < -1e-9 {
            return Err(invalid("radius is negative"));
        }
        let mut guard = 0usize;
        loop {
            guard += 1;
            if guard > MAX_MOVES {
                return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
            }
            let here = self.cursor.1;
            let delta = x_target - here;
            if delta.abs() <= 1e-15 {
                break;
            }
            let span = delta.abs();
            let step = match peck {
                Some(peck) => peck.min(span),
                None => span,
            };
            if !(step > 0.0) {
                return Err(invalid("peck did not advance"));
            }
            let next = if span - step <= 1e-12 { x_target } else { here + delta.signum() * step };
            if next == here {
                return Err(invalid("peck did not advance"));
            }
            self.feed(z, next)?;
            if (self.cursor.1 - x_target).abs() <= 1e-15 {
                break;
            }
            self.rapid(z, x_safe)?;
            self.rapid(z, next)?;
        }
        if dwell_s > 0.0 {
            self.dwell(dwell_s)?;
        }
        if retract_after {
            self.rapid(z, x_safe)?;
        }
        Ok(())
    }

    fn go(&mut self, motion: TurnMove) -> Result<()> {
        let end = match motion {
            TurnMove::Dwell { seconds } => {
                non_negative(seconds, "dwell")?;
                self.push(motion)?;
                return Ok(());
            }
            TurnMove::Thread { z, x, lead } => {
                positive(lead, "thread lead")?;
                self.check_point(z, x)?;
                (z, x)
            }
            TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } => {
                self.check_point(z, x)?;
                (z, x)
            }
            TurnMove::Arc { .. } => return Err(invalid("internal cycle emitted an arc")),
        };
        if end.0 == self.cursor.0 && end.1 == self.cursor.1 {
            return Ok(());
        }
        self.push(motion)?;
        self.cursor = end;
        Ok(())
    }

    fn check_point(&self, z: f64, x: f64) -> Result<()> {
        finite(z, "move z")?;
        finite(x, "move x")?;
        if !self.allow_negative && x < -1e-9 {
            return Err(invalid("radius is negative"));
        }
        Ok(())
    }

    fn push(&mut self, motion: TurnMove) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
        }
        self.moves.push(motion);
        Ok(())
    }
}

fn invalid(message: impl Into<String>) -> TurnError {
    ops::invalid(message)
}

fn finite(value: f64, what: &str) -> Result<()> {
    ops::finite(value, what)
}

fn positive(value: f64, what: &str) -> Result<()> {
    ops::positive(value, what)
}

fn non_negative(value: f64, what: &str) -> Result<()> {
    ops::non_negative(value, what)
}

#[cfg(test)]
#[path = "cycles_tests.rs"]
mod tests;
