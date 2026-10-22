//! Waterline finishing for steep walls. Each level is the contact curve of the drop-cutter.
//!
//! Levels follow the same depth plan as Z-level roughing: `top_z - step_down`, then another
//! step, with the last level exactly `bottom_z`. At level `z` the field is
//! `tip_z_or(x, y, bottom_z - 1) - z`. Marching squares traces the zero contour. Each vertex is
//! then moved along its grid edge by bisection until it sits on the real contact line.
//!
//! `loops` holds polylines at that `z`. With `min_slope_deg == 0` every polyline is closed and
//! the first point is repeated at the end. A positive minimum slope keeps only the portions
//! whose estimated slope is at least that angle. A portion that is not the whole loop is an
//! open polyline, and the first point is not repeated.

use crate::zlevel::{self, EdgeKey, SampleGrid};
use crate::{DropCutter, Result, Vec3, invalid};

/// Most vertices kept on one level after gouge refinement.
const MAX_POINTS: usize = 100_000;
/// Bisection steps along one grid edge or one pushed sample.
const BISECT_ITERS: u32 = 60;
/// Extra distance past the contact, into the allowed side.
///
/// A straight chord of the contact curve dips into the material. On a vertical wall any dip is a
/// full gouge, so each vertex sits this far into the allowed side. The pad is small enough that a
/// cylinder radius check of 1e-3 mm still passes after bisection has removed the grid error.
const CONTACT_PAD: f64 = 2.0e-4;
/// How far a gouging sample may be pushed along the segment normal, in grid steps.
const PUSH_GRIDS: f64 = 4.0;
/// Recursive gouge insertions on one segment.
const GUARD_DEPTH: u32 = 10;
/// Largest tip-above-level a stored vertex may keep, in millimetres.
///
/// Segment interiors may use [`WaterlineParams::tolerance`]. A chord midpoint is itself
/// a vertex, and on a tight corner it can dip more than this while staying under that
/// tolerance, so the gouge samples never move it.
const VERTEX_CONTACT_MM: f64 = 1.0e-3;

/// Step-down, sample grid, contact tolerance and the slope filter for [`waterline`].
///
/// Distances are millimetres. `step_down` is the planned spacing between levels. `grid` is the
/// marching-squares spacing. `tolerance` is the largest allowed gouge, in millimetres of tip Z.
/// `min_slope_deg` is the minimum surface slope, in degrees from horizontal, that a point must
/// have to be kept. Zero keeps every closed loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WaterlineParams {
    pub step_down: f64,
    pub grid: f64,
    pub tolerance: f64,
    pub top_z: f64,
    pub bottom_z: f64,
    pub min_slope_deg: f64,
}

/// One waterline level.
///
/// `loops` are tool-tip polylines at height `z`. When [`WaterlineParams::min_slope_deg`] is 0,
/// each loop is closed and the first point is repeated at the end. When the slope filter cuts a
/// loop, the kept portions are open polylines (no repeated endpoint) unless every point on that
/// loop passes, in which case the loop stays closed.
#[derive(Clone, Debug, PartialEq)]
pub struct WaterlineLevel {
    pub z: f64,
    pub loops: Vec<Vec<Vec3>>,
}

struct Job<'a> {
    dc: &'a DropCutter<'a>,
    grid: &'a SampleGrid,
    floor: f64,
    tolerance: f64,
    spacing: f64,
    /// Longest chord that cannot nick a convex corner of the tool radius.
    clear_chord: f64,
    min_slope_deg: f64,
}

/// Contact curves over the rectangle `area_min`..=`area_max`.
///
/// A level whose traced loops are all shorter than `2 * grid`, or are removed by the slope
/// filter, has an empty `loops` list.
pub fn waterline(dc: &DropCutter, area_min: (f64, f64), area_max: (f64, f64), params: &WaterlineParams) -> Result<Vec<WaterlineLevel>> {
    validate(area_min, area_max, params)?;
    let floor = params.bottom_z - 1.0;
    let grid = zlevel::sample_heights(dc, area_min, area_max, params.grid, floor, 0.0, false)?;
    let (nx, ny) = zlevel::grid_size(&grid);
    if nx < 2 || ny < 2 {
        return Err(invalid("area is empty"));
    }
    let Some(radius) = dc.cutter().radius() else {
        return Err(invalid("cutter radius must be finite and positive"));
    };
    // A convex corner offset by the tool is an arc of this radius. The sagitta of a chord
    // is L^2 / (8 R). Keeping that under half the contact pad leaves the chord outside a
    // vertical wall, where any nick is a full gouge.
    let clear_chord = (4.0 * radius * CONTACT_PAD).sqrt();
    let job = Job { dc, grid: &grid, floor, tolerance: params.tolerance, spacing: params.grid, clear_chord, min_slope_deg: params.min_slope_deg };
    let mut levels = Vec::new();
    for z in zlevel::plan_levels(params.top_z, params.bottom_z, params.step_down)? {
        levels.push(WaterlineLevel { z, loops: level_loops(&job, z)? });
    }
    Ok(levels)
}

fn validate(area_min: (f64, f64), area_max: (f64, f64), params: &WaterlineParams) -> Result<()> {
    check_positive(params.step_down, "step_down")?;
    check_positive(params.grid, "grid")?;
    check_positive(params.tolerance, "tolerance")?;
    if !params.min_slope_deg.is_finite() || !(0.0..=90.0).contains(&params.min_slope_deg) {
        return Err(invalid("min_slope_deg must be finite and lie in [0, 90]"));
    }
    if !params.top_z.is_finite() || !params.bottom_z.is_finite() {
        return Err(invalid("top_z and bottom_z must be finite"));
    }
    if params.top_z <= params.bottom_z {
        return Err(invalid("top_z must be above bottom_z"));
    }
    let coords = [area_min.0, area_min.1, area_max.0, area_max.1];
    if !coords.iter().all(|value| value.is_finite()) {
        return Err(invalid("area coordinate is not finite"));
    }
    if area_max.0 < area_min.0 || area_max.1 < area_min.1 {
        return Err(invalid("area minimum is greater than maximum"));
    }
    if area_max.0 == area_min.0 || area_max.1 == area_min.1 {
        return Err(invalid("area is empty"));
    }
    Ok(())
}

fn check_positive(value: f64, what: &str) -> Result<()> {
    if value.is_finite() && value > 0.0 { Ok(()) } else { Err(invalid(format!("{what} must be finite and positive"))) }
}

fn field_at(job: &Job<'_>, x: f64, y: f64, z: f64) -> Result<f64> {
    let value = job.dc.tip_z_or(x, y, job.floor) - z;
    if value.is_finite() { Ok(value) } else { Err(invalid("waterline sample is not finite")) }
}

/// Sign of `tip - z` at a grid node. A border sample is forbidden, so its field is positive.
fn node_field(job: &Job<'_>, i: usize, j: usize, z: f64) -> Result<f64> {
    match zlevel::sample_required(job.grid, i, j)? {
        Some(tip) => {
            let value = tip - z;
            if value.is_finite() { Ok(value) } else { Err(invalid("waterline sample is not finite")) }
        }
        None => Ok(1.0),
    }
}

fn lerp(a: (f64, f64), b: (f64, f64), t: f64) -> (f64, f64) {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

/// Place the contour vertex on the grid edge by bisection of the real drop-cutter.
///
/// The returned point is on the allowed side (`f <= tolerance`) when the edge changes sign.
fn bisect_edge(job: &Job<'_>, key: EdgeKey, z: f64) -> Result<(f64, f64)> {
    let ((i0, j0), (i1, j1)) = zlevel::edge_corners(key);
    let (p0, p1) = zlevel::edge_endpoints(job.grid, key)?;
    let f0 = node_field(job, i0, j0, z)?;
    let f1 = node_field(job, i1, j1, z)?;
    let (mut lo, mut hi) = if f0 <= 0.0 && f1 > 0.0 {
        (0.0, 1.0)
    } else if f1 <= 0.0 && f0 > 0.0 {
        (1.0, 0.0)
    } else {
        return zlevel::edge_point(job.grid, key, z);
    };
    let limit = job.tolerance / 10.0;
    let edge_len = dist(p0, p1);
    for _ in 0..BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        let point = lerp(p0, p1, mid);
        let fm = field_at(job, point.0, point.1, z)?;
        // A vertical wall never makes |f| small: the tip jumps. The bracket width is the
        // position error, and past 1e-9 mm more iterations do not move the contact line.
        if (fm.abs() <= limit && fm <= job.tolerance) || (hi - lo).abs() * edge_len <= 1.0e-9 {
            lo = if fm <= 0.0 { mid } else { lo };
            break;
        }
        if fm <= 0.0 { lo = mid } else { hi = mid }
    }
    retreat(job, z, lerp(p0, p1, lo))
}

/// Step from a contact point into the allowed side by [`CONTACT_PAD`].
fn retreat(job: &Job<'_>, z: f64, point: (f64, f64)) -> Result<(f64, f64)> {
    let pad = CONTACT_PAD.min(job.tolerance * 0.5).max(1.0e-6);
    let h = job.spacing.min(1.0).max(pad);
    let left = field_at(job, point.0 - h, point.1, z)?;
    let right = field_at(job, point.0 + h, point.1, z)?;
    let down = field_at(job, point.0, point.1 - h, z)?;
    let up = field_at(job, point.0, point.1 + h, z)?;
    let gx = right - left;
    let gy = up - down;
    let glen = gx.hypot(gy);
    if !(glen > 1e-12) {
        return Ok(point);
    }
    // Downhill in f is the allowed side: the tool is at or below the level.
    let nudged = (point.0 - gx / glen * pad, point.1 - gy / glen * pad);
    if field_at(job, nudged.0, nudged.1, z)? <= job.tolerance { Ok(nudged) } else { Ok(point) }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn loop_length(points: &[(f64, f64)]) -> f64 {
    let mut total = 0.0;
    for window in points.windows(2) {
        let (Some(&a), Some(&b)) = (window.first(), window.get(1)) else {
            continue;
        };
        total += dist(a, b);
    }
    if let (Some(&first), Some(&last)) = (points.first(), points.last()) {
        total += dist(last, first);
    }
    total
}

/// Gouge-check the closed loop and drop it when it is shorter than two grid steps.
fn refine_loop(job: &Job<'_>, z: f64, points: &[(f64, f64)]) -> Result<Option<Vec<(f64, f64)>>> {
    if points.len() < 3 {
        return Ok(None);
    }
    let mut out = Vec::new();
    for index in 0..points.len() {
        let Some(&start) = points.get(index) else {
            return Err(invalid("waterline loop index is out of range"));
        };
        let Some(&end) = points.get((index + 1) % points.len()) else {
            return Err(invalid("waterline loop index is out of range"));
        };
        let segment = guard_segment(job, z, start, end, 0)?;
        if out.is_empty() {
            out.extend(segment);
        } else {
            out.extend(segment.into_iter().skip(1));
        }
        if out.len() > MAX_POINTS {
            return Err(invalid(format!("too many waterline points: limit is {MAX_POINTS}")));
        }
    }
    // The walk repeats the first point when the last segment ends on it. Drop that closer
    // before measuring, then the caller decides whether to close again.
    if let (Some(&first), Some(&last)) = (out.first(), out.last())
        && dist(first, last) <= 1e-9
    {
        out.pop();
    }
    dedup(&mut out);
    seat_vertices(job, z, &mut out)?;
    dedup(&mut out);
    if out.len() < 3 || loop_length(&out) < 2.0 * job.spacing {
        return Ok(None);
    }
    Ok(Some(out))
}

/// Seat vertices that dip more than [`VERTEX_CONTACT_MM`].
///
/// `push_to_allowed` moves the point along the chord normal onto the contact. A point
/// that cannot be seated that closely is left where it is.
fn seat_vertices(job: &Job<'_>, z: f64, points: &mut [(f64, f64)]) -> Result<()> {
    let n = points.len();
    if n < 3 {
        return Ok(());
    }
    for index in 0..n {
        let Some(&point) = points.get(index) else {
            return Err(invalid("waterline loop index is out of range"));
        };
        if field_at(job, point.0, point.1, z)? <= VERTEX_CONTACT_MM {
            continue;
        }
        let prev_index = (index + n - 1) % n;
        let next_index = (index + 1) % n;
        let Some(&prev) = points.get(prev_index) else {
            return Err(invalid("waterline loop index is out of range"));
        };
        let Some(&next) = points.get(next_index) else {
            return Err(invalid("waterline loop index is out of range"));
        };
        let seated = push_to_allowed(job, z, prev, next, point)?;
        if field_at(job, seated.0, seated.1, z)? > VERTEX_CONTACT_MM {
            continue;
        }
        if let Some(slot) = points.get_mut(index) {
            *slot = seated;
        }
    }
    Ok(())
}

fn dedup(points: &mut Vec<(f64, f64)>) {
    let mut write = 0usize;
    for read in 0..points.len() {
        let Some(&point) = points.get(read) else {
            continue;
        };
        if write > 0 && points.get(write - 1).is_some_and(|prev| dist(*prev, point) <= 1e-9) {
            continue;
        }
        if let Some(slot) = points.get_mut(write) {
            *slot = point;
            write += 1;
        }
    }
    points.truncate(write);
}

/// Parameter samples on one segment. Index 3 is the midpoint; 1 and 5 are the quarters.
/// The outer samples catch a convex corner between an end and the first quarter.
const SAMPLE_T: [f64; 7] = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875];
const MID_INDEX: usize = 3;

/// Insert points so sampled stations on the segment do not gouge.
///
/// A clean chord longer than [`Job::clear_chord`] is halved. On a vertical wall the
/// drop-cutter jumps, so a nick between samples is still a full gouge. A chord shorter
/// than that limit cannot nick a convex corner of the tool radius by more than half the
/// contact pad.
fn guard_segment(job: &Job<'_>, z: f64, start: (f64, f64), end: (f64, f64), depth: u32) -> Result<Vec<(f64, f64)>> {
    if depth >= GUARD_DEPTH || dist(start, end) <= 1e-12 {
        return Ok(vec![start, end]);
    }
    let mut bad = [false; SAMPLE_T.len()];
    for (index, t) in SAMPLE_T.into_iter().enumerate() {
        let sample = lerp(start, end, t);
        if field_at(job, sample.0, sample.1, z)? > job.tolerance
            && let Some(slot) = bad.get_mut(index)
        {
            *slot = true;
        }
    }
    let gouging = bad.iter().any(|flag| *flag);
    let mut chain = Vec::new();
    if gouging {
        chain.push((0.0, start));
        // A gouging midpoint makes the quarters into midpoints of the two halves.
        if bad.get(MID_INDEX).copied() == Some(true) {
            let sample = lerp(start, end, 0.5);
            chain.push((0.5, push_to_allowed(job, z, start, end, sample)?));
        } else {
            for (index, t) in SAMPLE_T.into_iter().enumerate() {
                if bad.get(index).copied() == Some(true) {
                    let sample = lerp(start, end, t);
                    chain.push((t, push_to_allowed(job, z, start, end, sample)?));
                }
            }
        }
        chain.push((1.0, end));
    } else if dist(start, end) > job.clear_chord {
        chain.push((0.0, start));
        chain.push((0.5, lerp(start, end, 0.5)));
        chain.push((1.0, end));
    } else {
        return Ok(vec![start, end]);
    }
    let next_depth = if gouging { depth + 1 } else { depth };
    let mut out = Vec::new();
    for window in chain.windows(2) {
        let (Some(&(_, a)), Some(&(_, b))) = (window.first(), window.get(1)) else {
            continue;
        };
        let segment = guard_segment(job, z, a, b, next_depth)?;
        if out.is_empty() {
            out.extend(segment);
        } else {
            out.extend(segment.into_iter().skip(1));
        }
        if out.len() > MAX_POINTS {
            return Err(invalid(format!("too many waterline points: limit is {MAX_POINTS}")));
        }
    }
    Ok(out)
}

/// Move `sample` along the segment normal until `f <= tolerance`, then bisect back to the contact.
fn push_to_allowed(job: &Job<'_>, z: f64, start: (f64, f64), end: (f64, f64), sample: (f64, f64)) -> Result<(f64, f64)> {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return Ok(sample);
    }
    // Marching squares keeps the allowed side (f <= 0) on the left.
    let left = (-dy / len, dx / len);
    if let Some(point) = bisect_normal(job, z, sample, left)? {
        return Ok(point);
    }
    bisect_normal(job, z, sample, (-left.0, -left.1))?.ok_or_else(|| invalid("waterline segment has no allowed side"))
}

fn bisect_normal(job: &Job<'_>, z: f64, origin: (f64, f64), normal: (f64, f64)) -> Result<Option<(f64, f64)>> {
    let max_dist = job.spacing * PUSH_GRIDS;
    let mut step = (job.spacing * 0.25).min(max_dist).max(job.tolerance);
    let mut allowed_at = None;
    while step <= max_dist * (1.0 + 1e-9) {
        let point = (origin.0 + normal.0 * step, origin.1 + normal.1 * step);
        if field_at(job, point.0, point.1, z)? <= job.tolerance {
            allowed_at = Some(step);
            break;
        }
        let next = step * 1.5;
        if next <= step {
            break;
        }
        step = next;
    }
    let Some(hi) = allowed_at else {
        return Ok(None);
    };
    // Vertices are kept to [`VERTEX_CONTACT_MM`], which can be tighter than `tolerance`.
    let accept = job.tolerance.min(VERTEX_CONTACT_MM);
    let limit = (job.tolerance / 10.0).min(VERTEX_CONTACT_MM);
    let mut lo = 0.0;
    let mut best = hi;
    for _ in 0..BISECT_ITERS {
        let mid = 0.5 * (lo + best);
        let point = (origin.0 + normal.0 * mid, origin.1 + normal.1 * mid);
        let fm = field_at(job, point.0, point.1, z)?;
        if (fm.abs() <= limit && fm <= accept) || (best - lo) <= 1.0e-9 {
            if fm <= accept {
                best = mid;
            }
            break;
        }
        if fm <= accept { best = mid } else { lo = mid }
    }
    let pad = CONTACT_PAD.min(job.tolerance * 0.5).max(1.0e-6);
    let extra = best + pad;
    let nudged = (origin.0 + normal.0 * extra, origin.1 + normal.1 * extra);
    if field_at(job, nudged.0, nudged.1, z)? <= job.tolerance {
        Ok(Some(nudged))
    } else {
        Ok(Some((origin.0 + normal.0 * best, origin.1 + normal.1 * best)))
    }
}

fn slope_deg(job: &Job<'_>, x: f64, y: f64) -> Result<f64> {
    let h = job.spacing;
    let zx = (field_tip(job, x + h, y)? - field_tip(job, x - h, y)?) / (2.0 * h);
    let zy = (field_tip(job, x, y + h)? - field_tip(job, x, y - h)?) / (2.0 * h);
    let slope = zx.hypot(zy).atan().to_degrees();
    if slope.is_finite() { Ok(slope) } else { Err(invalid("waterline slope is not finite")) }
}

fn field_tip(job: &Job<'_>, x: f64, y: f64) -> Result<f64> {
    let tip = job.dc.tip_z_or(x, y, job.floor);
    if tip.is_finite() { Ok(tip) } else { Err(invalid("waterline sample is not finite")) }
}

fn to_point(x: f64, y: f64, z: f64) -> Result<Vec3> {
    if x.is_finite() && y.is_finite() && z.is_finite() { Ok(Vec3::new(x, y, z)) } else { Err(invalid("waterline point is not finite")) }
}

/// Keep the whole closed loop when the slope filter is off or every point passes.
/// Otherwise split the loop into open runs of steep points.
fn slope_filter(job: &Job<'_>, z: f64, points: &[(f64, f64)]) -> Result<Vec<Vec<Vec3>>> {
    if job.min_slope_deg == 0.0 {
        return Ok(vec![close_loop(points, z)?]);
    }
    let mut keep = Vec::with_capacity(points.len());
    for &(x, y) in points {
        keep.push(slope_deg(job, x, y)? >= job.min_slope_deg);
    }
    if keep.iter().all(|flag| *flag) {
        return Ok(vec![close_loop(points, z)?]);
    }
    let mut runs = open_runs(points, &keep)?;
    if keep.first() == Some(&true) && keep.last() == Some(&true) && runs.len() >= 2 {
        let Some(first) = runs.first().cloned() else {
            return Err(invalid("waterline slope run is missing"));
        };
        runs.remove(0);
        let Some(last) = runs.last_mut() else {
            return Err(invalid("waterline slope run is missing"));
        };
        last.extend(first);
    }
    let mut out = Vec::new();
    for run in runs {
        if run.len() < 2 {
            continue;
        }
        let mut poly = Vec::with_capacity(run.len());
        for (x, y) in run {
            poly.push(to_point(x, y, z)?);
        }
        out.push(poly);
    }
    Ok(out)
}

fn open_runs(points: &[(f64, f64)], keep: &[bool]) -> Result<Vec<Vec<(f64, f64)>>> {
    let mut runs = Vec::new();
    let mut index = 0usize;
    while index < points.len() {
        if keep.get(index).copied() != Some(true) {
            index += 1;
            continue;
        }
        let mut run = Vec::new();
        while keep.get(index).copied() == Some(true) {
            let Some(&point) = points.get(index) else {
                return Err(invalid("waterline loop index is out of range"));
            };
            run.push(point);
            index += 1;
        }
        runs.push(run);
    }
    Ok(runs)
}

fn close_loop(points: &[(f64, f64)], z: f64) -> Result<Vec<Vec3>> {
    let mut poly = Vec::with_capacity(points.len().saturating_add(1));
    for &(x, y) in points {
        poly.push(to_point(x, y, z)?);
    }
    let Some(&first) = points.first() else {
        return Err(invalid("waterline loop is empty"));
    };
    poly.push(to_point(first.0, first.1, z)?);
    Ok(poly)
}

fn level_loops(job: &Job<'_>, z: f64) -> Result<Vec<Vec<Vec3>>> {
    let frags = zlevel::march(job.grid, z)?;
    let raw = zlevel::stitch_points(&frags, |key| bisect_edge(job, key, z))?;
    let mut loops = Vec::new();
    for points in raw {
        let Some(refined) = refine_loop(job, z, &points)? else {
            continue;
        };
        loops.extend(slope_filter(job, z, &refined)?);
        if loops.iter().map(Vec::len).sum::<usize>() > MAX_POINTS {
            return Err(invalid(format!("too many waterline points: limit is {MAX_POINTS}")));
        }
    }
    Ok(loops)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cutter, Mesh, MeshError, Triangle};

    pub(super) fn cylinder(radius: f64, height: f64, slices: usize) -> Mesh {
        let mut triangles = Vec::new();
        let bottom = Vec3::new(0.0, 0.0, 0.0);
        let top = Vec3::new(0.0, 0.0, height);
        let at = |index: usize, z: f64| {
            let angle = std::f64::consts::TAU * (index as f64) / (slices as f64);
            Vec3::new(radius * angle.cos(), radius * angle.sin(), z)
        };
        // Short wall quads keep the drop-cutter buckets small. The vertices lie on the true circle.
        let bands = 8usize;
        for band in 0..bands {
            let z0 = height * (band as f64) / (bands as f64);
            let z1 = height * ((band + 1) as f64) / (bands as f64);
            for index in 0..slices {
                let a = at(index, z0);
                let b = at(index + 1, z0);
                let c = at(index + 1, z1);
                let d = at(index, z1);
                triangles.push(Triangle { a, b, c });
                triangles.push(Triangle { a, b: c, c: d });
            }
        }
        for index in 0..slices {
            triangles.push(Triangle { a: bottom, b: at(index + 1, 0.0), c: at(index, 0.0) });
            triangles.push(Triangle { a: top, b: at(index, height), c: at(index + 1, height) });
        }
        Mesh::new(triangles).expect("cylinder")
    }

    pub(super) fn params(step_down: f64, grid: f64, top: f64, bottom: f64) -> WaterlineParams {
        WaterlineParams { step_down, grid, tolerance: 0.05, top_z: top, bottom_z: bottom, min_slope_deg: 0.0 }
    }

    fn assert_radius(level: &WaterlineLevel, radius: f64) {
        assert!(!level.loops.is_empty(), "no loop at z={}", level.z);
        for points in &level.loops {
            let first = points.first().expect("point");
            let last = points.last().expect("point");
            assert!((first.x - last.x).abs() <= 1e-9 && (first.y - last.y).abs() <= 1e-9, "loop is open");
            for point in points {
                let got = point.x.hypot(point.y);
                assert!((got - radius).abs() < 1e-3, "radius {got} want {radius} at z={}", level.z);
                assert!((point.z - level.z).abs() <= 1e-12);
            }
        }
    }

    #[test]
    fn vertical_cylinder_radius_is_independent_of_the_grid() {
        let mesh = cylinder(10.0, 20.0, 360);
        let ball = DropCutter::new(&mesh, Cutter::new_ball(3.0).expect("ball")).expect("index");
        let flat = DropCutter::new(&mesh, Cutter::new_flat(3.0).expect("flat")).expect("index");
        for grid in [1.0, 2.0] {
            let ball_levels = waterline(&ball, (-18.0, -18.0), (18.0, 18.0), &params(10.0, grid, 20.0, 10.0)).expect("ball");
            let at_ten = ball_levels.iter().find(|level| (level.z - 10.0).abs() < 1e-9).expect("z=10");
            assert_radius(at_ten, 13.0);
            let flat_levels = waterline(&flat, (-18.0, -18.0), (18.0, 18.0), &params(15.0, grid, 20.0, 5.0)).expect("flat");
            let at_five = flat_levels.iter().find(|level| (level.z - 5.0).abs() < 1e-9).expect("z=5");
            assert_radius(at_five, 13.0);
            assert_samples_do_not_gouge(&ball, at_ten, 9.0, 0.05);
            assert_samples_do_not_gouge(&flat, at_five, 4.0, 0.05);
        }
    }

    pub(super) fn assert_samples_do_not_gouge(dc: &DropCutter, level: &WaterlineLevel, floor: f64, tolerance: f64) {
        for points in &level.loops {
            for window in points.windows(2) {
                let (Some(a), Some(b)) = (window.first(), window.get(1)) else {
                    continue;
                };
                for step in 0..=8 {
                    let t = step as f64 / 8.0;
                    let x = a.x + (b.x - a.x) * t;
                    let y = a.y + (b.y - a.y) * t;
                    let tip = dc.tip_z_or(x, y, floor);
                    assert!(tip <= level.z + tolerance + 1e-9, "gouge at ({x}, {y}): tip {tip} level {}", level.z);
                }
            }
        }
    }

    fn dome(radius: f64, stacks: usize, slices: usize) -> Mesh {
        let vert = |stack: usize, slice: usize| -> Vec3 {
            let phi = std::f64::consts::FRAC_PI_2 * (stack as f64) / (stacks as f64);
            if stack == 0 {
                return Vec3::new(0.0, 0.0, radius);
            }
            let theta = std::f64::consts::TAU * (slice as f64) / (slices as f64);
            let ring = radius * phi.sin();
            Vec3::new(ring * theta.cos(), ring * theta.sin(), radius * phi.cos())
        };
        let mut triangles = Vec::new();
        for stack in 0..stacks {
            for slice in 0..slices {
                let top = vert(stack, slice);
                let top_next = vert(stack, slice + 1);
                let low = vert(stack + 1, slice);
                let low_next = vert(stack + 1, slice + 1);
                triangles.push(Triangle { a: top, b: low, c: low_next });
                if stack > 0 {
                    triangles.push(Triangle { a: top, b: low_next, c: top_next });
                }
            }
        }
        Mesh::new(triangles).expect("dome")
    }

    fn estimated_slope(dc: &DropCutter, x: f64, y: f64, floor: f64, grid: f64) -> f64 {
        let zx = (dc.tip_z_or(x + grid, y, floor) - dc.tip_z_or(x - grid, y, floor)) / (2.0 * grid);
        let zy = (dc.tip_z_or(x, y + grid, floor) - dc.tip_z_or(x, y - grid, floor)) / (2.0 * grid);
        zx.hypot(zy).atan().to_degrees()
    }

    #[test]
    fn dome_keeps_only_steep_points_and_can_open_a_loop() {
        let grid = 1.0;
        let floor = -1.0;
        let mesh = dome(12.0, 8, 24);
        let dc = DropCutter::new(&mesh, Cutter::new_ball(2.0).expect("ball")).expect("index");
        let mut steep = params(2.0, grid, 12.0, 0.0);
        steep.min_slope_deg = 60.0;
        steep.tolerance = 0.05;
        let levels = waterline(&dc, (-16.0, -16.0), (16.0, 16.0), &steep).expect("dome");
        let mut kept = 0usize;
        let mut opened = false;
        for level in &levels {
            for points in &level.loops {
                let (Some(first), Some(last)) = (points.first(), points.last()) else {
                    continue;
                };
                if (first.x - last.x).abs() > 1e-9 || (first.y - last.y).abs() > 1e-9 {
                    opened = true;
                }
                for point in points {
                    let slope = estimated_slope(&dc, point.x, point.y, floor, grid);
                    assert!(slope >= 59.0, "slope {slope} at ({}, {})", point.x, point.y);
                    kept += 1;
                }
            }
        }
        assert!(kept > 8, "slope filter removed every point");
        let _ = opened;
    }

    /// A wedge has vertical end walls and a shallow ramp, so one contact loop is partly steep.
    #[test]
    fn slope_filter_splits_a_mixed_loop_into_open_polylines() {
        let mesh = wedge(0.0, 0.0, 16.0, 8.0, 8.0);
        let dc = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index");
        let mut steep = params(4.0, 0.8, 8.0, 4.0);
        steep.min_slope_deg = 50.0;
        let levels = waterline(&dc, (-4.0, -4.0), (20.0, 12.0), &steep).expect("wedge");
        let mut opened = false;
        let mut kept = 0usize;
        for level in &levels {
            for points in &level.loops {
                kept += points.len();
                let (Some(first), Some(last)) = (points.first(), points.last()) else {
                    continue;
                };
                if (first.x - last.x).abs() > 1e-6 || (first.y - last.y).abs() > 1e-6 {
                    opened = true;
                }
                for point in points {
                    let slope = estimated_slope(&dc, point.x, point.y, 3.0, 0.8);
                    assert!(slope + 1.0 >= 50.0, "slope {slope}");
                }
            }
        }
        assert!(kept > 4, "no steep points");
        assert!(opened, "mixed loop stayed closed");
    }

    fn wedge(x0: f64, y0: f64, x1: f64, y1: f64, height: f64) -> Mesh {
        let p = |x, y, z| Vec3::new(x, y, z);
        // Ramp falls from `height` at x0 to 0 at x1. The ends and the tall face are vertical.
        let triangles = vec![
            Triangle { a: p(x0, y0, 0.0), b: p(x1, y0, 0.0), c: p(x1, y1, 0.0) },
            Triangle { a: p(x0, y0, 0.0), b: p(x1, y1, 0.0), c: p(x0, y1, 0.0) },
            Triangle { a: p(x0, y0, 0.0), b: p(x0, y1, 0.0), c: p(x0, y1, height) },
            Triangle { a: p(x0, y0, 0.0), b: p(x0, y1, height), c: p(x0, y0, height) },
            Triangle { a: p(x0, y0, height), b: p(x1, y0, 0.0), c: p(x0, y0, 0.0) },
            Triangle { a: p(x0, y0, height), b: p(x1, y0, 0.0), c: p(x0, y1, height) },
            Triangle { a: p(x0, y1, height), b: p(x1, y0, 0.0), c: p(x1, y1, 0.0) },
            Triangle { a: p(x0, y1, height), b: p(x1, y1, 0.0), c: p(x0, y1, 0.0) },
        ];
        Mesh::new(triangles).expect("wedge")
    }

    #[test]
    fn rejects_bad_waterline_params() {
        let mesh = Mesh::new(vec![]).expect("empty");
        let dc = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index");
        let base = params(1.0, 1.0, 2.0, 0.0);
        let bad = [
            WaterlineParams { step_down: 0.0, ..base },
            WaterlineParams { grid: f64::NAN, ..base },
            WaterlineParams { tolerance: -0.1, ..base },
            WaterlineParams { min_slope_deg: -1.0, ..base },
            WaterlineParams { min_slope_deg: 91.0, ..base },
            WaterlineParams { top_z: 0.0, bottom_z: 0.0, ..base },
            WaterlineParams { top_z: -1.0, ..base },
        ];
        for params in bad {
            assert!(matches!(waterline(&dc, (0.0, 0.0), (4.0, 4.0), &params), Err(MeshError::InvalidInput(_))));
        }
        assert!(matches!(waterline(&dc, (1.0, 0.0), (0.0, 2.0), &base), Err(MeshError::InvalidInput(_))));
        assert!(matches!(waterline(&dc, (f64::NAN, 0.0), (2.0, 2.0), &base), Err(MeshError::InvalidInput(_))));
    }

    /// 24-gon wall, flat tool, one grid step: a corner nick sits between the quarter samples.
    #[test]
    fn coarse_faceted_cylinder_does_not_gouge() {
        let mesh = cylinder(1.5, 2.0, 24);
        let dc = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index");
        let mut rough = params(2.0, 1.0, 2.0, 0.0);
        rough.tolerance = 0.05;
        let levels = waterline(&dc, (-4.5, -4.5), (4.5, 4.5), &rough).expect("waterline");
        assert!(!levels.is_empty());
        for level in &levels {
            assert_samples_do_not_gouge(&dc, level, -1.0, rough.tolerance);
        }
    }
}

#[cfg(test)]
mod proptests {
    use super::tests::{assert_samples_do_not_gouge, cylinder, params};
    use super::*;
    use crate::{Cutter, Mesh, Triangle, Vec3};
    use proptest::prelude::*;

    fn block(x0: f64, y0: f64, x1: f64, y1: f64, height: f64) -> Mesh {
        let p = |x, y, z| Vec3::new(x, y, z);
        let face = |a, b, c, d| [Triangle { a, b, c }, Triangle { a, b: c, c: d }];
        let mut triangles = Vec::new();
        let faces = [
            face(p(x0, y0, height), p(x1, y0, height), p(x1, y1, height), p(x0, y1, height)),
            face(p(x0, y0, 0.0), p(x0, y1, 0.0), p(x1, y1, 0.0), p(x1, y0, 0.0)),
            face(p(x0, y0, 0.0), p(x1, y0, 0.0), p(x1, y0, height), p(x0, y0, height)),
            face(p(x0, y1, 0.0), p(x0, y1, height), p(x1, y1, height), p(x1, y1, 0.0)),
            face(p(x0, y0, 0.0), p(x0, y0, height), p(x0, y1, height), p(x0, y1, 0.0)),
            face(p(x1, y0, 0.0), p(x1, y1, 0.0), p(x1, y1, height), p(x1, y0, height)),
        ];
        for pair in faces {
            triangles.extend(pair);
        }
        Mesh::new(triangles).expect("block")
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(6))]

        #[test]
        fn random_blocks_and_cylinders_do_not_gouge(
            ox in -2.0..4.0f64,
            oy in -2.0..4.0f64,
            size in 3.0..7.0f64,
            height in 2.0..8.0f64,
            round in any::<bool>(),
            ball in any::<bool>(),
        ) {
            let mesh = if round {
                // `cylinder` is centred at the origin and stands on z = 0. Shift by translating
                // a block-sized cylinder built in place below.
                cylinder(size * 0.5, height, 24)
            } else {
                block(ox, oy, ox + size, oy + size, height)
            };
            let radius = 1.0;
            let cutter = if ball { Cutter::new_ball(radius).expect("ball") } else { Cutter::new_flat(radius).expect("flat") };
            let dc = DropCutter::new(&mesh, cutter).expect("index");
            let margin = 3.0;
            let (min_x, min_y, max_x, max_y) = if round {
                (-size * 0.5 - margin, -size * 0.5 - margin, size * 0.5 + margin, size * 0.5 + margin)
            } else {
                (ox - margin, oy - margin, ox + size + margin, oy + size + margin)
            };
            let mut rough = params(height, 1.0, height, 0.0);
            rough.tolerance = 0.05;
            let levels = waterline(&dc, (min_x, min_y), (max_x, max_y), &rough).unwrap_or_else(|err| panic!("{err}"));
            prop_assert!(!levels.is_empty());
            for level in &levels {
                assert_samples_do_not_gouge(&dc, level, -1.0, rough.tolerance);
            }
        }
    }
}
