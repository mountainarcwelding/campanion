//! Z-level roughing. Each level is a set of closed tool-centre rings on one Z plane.
//!
//! Levels follow a depth plan: `top_z - step_down`, then another step, with the last level
//! exactly `bottom_z`. At each level the tool centre may sit where the drop-cutter tip, plus
//! stock to leave, is at or below that Z. The allowed set is sampled on a grid clipped to the
//! stock rectangle (border samples are forbidden so every loop closes), traced with marching
//! squares, and inset by one grid step before the stepover rings are cut.

use std::collections::HashMap;

use campanion_geom::{Containment, Contour, Line, Orientation, Point, Region, Segment, Tolerance, offset_region};

use crate::{DropCutter, Result, invalid};

/// Most Z levels in one roughing call.
const MAX_LEVELS: usize = 1024;

/// Most drop-cutter samples in the stock grid.
const MAX_SAMPLES: usize = 4_000_000;

/// Most inset rings cut from one region, including the boundary itself.
const MAX_RINGS: usize = 1000;

/// Step-down, stepover, sample grid, and the Z range for [`zlevel_rough`].
///
/// Distances are millimetres. `step_down` is the planned spacing between levels. `stepover` is
/// the spacing between successive inset rings. `grid` is the sample spacing used to find where
/// the tool centre is allowed. `stock_to_leave` is added to the drop-cutter tip before that
/// test. `top_z` is the stock surface and `bottom_z` is the deepest level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZLevelParams {
    pub step_down: f64,
    pub stepover: f64,
    pub grid: f64,
    pub stock_to_leave: f64,
    pub top_z: f64,
    pub bottom_z: f64,
}

/// One roughing level: tool-centre rings at height `z`, outermost first.
#[derive(Clone, Debug, PartialEq)]
pub struct ZLevel {
    pub z: f64,
    pub rings: Vec<campanion_geom::Contour>,
}

/// Closed roughing rings at each depth-plan level over the stock rectangle `area_min`..=`area_max`.
///
/// A level whose sample grid has no allowed tool-centre position has an empty `rings` list.
pub fn zlevel_rough(dc: &DropCutter, area_min: (f64, f64), area_max: (f64, f64), params: &ZLevelParams, tol: &Tolerance) -> Result<Vec<ZLevel>> {
    validate(area_min, area_max, params)?;
    let grid = sample_grid(dc, area_min, area_max, params)?;
    if grid.xs.len() < 2 || grid.ys.len() < 2 {
        return Err(invalid("area is empty"));
    }
    let mut levels = Vec::new();
    for z in level_heights(params)? {
        let loops = trace_allowed(&grid, z, tol)?;
        let regions = group_regions(&loops, tol)?;
        // Inset by one grid step so a cell-edge interpolation cannot reach a forbidden centre.
        let mut rings = Vec::new();
        for region in &regions {
            let shrunk = offset_region(region, params.grid, tol).map_err(from_geom)?;
            for piece in &shrunk {
                append_stepovers(piece, params.stepover, tol, &mut rings)?;
            }
        }
        levels.push(ZLevel { z, rings });
    }
    Ok(levels)
}

fn validate(area_min: (f64, f64), area_max: (f64, f64), params: &ZLevelParams) -> Result<()> {
    check_positive(params.step_down, "step_down")?;
    check_positive(params.stepover, "stepover")?;
    check_positive(params.grid, "grid")?;
    if !params.stock_to_leave.is_finite() || params.stock_to_leave < 0.0 {
        return Err(invalid("stock_to_leave must be finite and non-negative"));
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

/// Z of each level, from `top_z - step_down` down to exactly `bottom_z`.
fn level_heights(params: &ZLevelParams) -> Result<Vec<f64>> {
    let total = params.top_z - params.bottom_z;
    let steps = (total / params.step_down).ceil();
    if !steps.is_finite() || steps > MAX_LEVELS as f64 {
        return Err(invalid(format!("too many z levels: limit is {MAX_LEVELS}")));
    }
    let mut heights = Vec::new();
    let mut done = 0.0;
    loop {
        if heights.len() >= MAX_LEVELS {
            return Err(invalid(format!("too many z levels: limit is {MAX_LEVELS}")));
        }
        let next = done + params.step_down;
        if !next.is_finite() {
            return Err(invalid("z level is not finite"));
        }
        if next >= total - 1e-9 {
            heights.push(params.bottom_z);
            break;
        }
        done = next;
        let z = params.top_z - done;
        if !z.is_finite() {
            return Err(invalid("z level is not finite"));
        }
        heights.push(z);
    }
    Ok(heights)
}

struct SampleGrid {
    xs: Vec<f64>,
    ys: Vec<f64>,
    /// Required tip Z. `None` is a border sample, forbidden at every level.
    required: Vec<Option<f64>>,
}

fn sample_grid(dc: &DropCutter, area_min: (f64, f64), area_max: (f64, f64), params: &ZLevelParams) -> Result<SampleGrid> {
    let xs = axis_coords(area_min.0, area_max.0, params.grid)?;
    let ys = axis_coords(area_min.1, area_max.1, params.grid)?;
    let count = xs.len().checked_mul(ys.len()).ok_or_else(|| invalid(format!("too many samples: limit is {MAX_SAMPLES}")))?;
    if count > MAX_SAMPLES {
        return Err(invalid(format!("too many samples: {count} > {MAX_SAMPLES}")));
    }
    let floor = params.bottom_z - 1.0;
    if !floor.is_finite() {
        return Err(invalid("drop-cutter floor is not finite"));
    }
    let mut required = Vec::with_capacity(count);
    for (j, y) in ys.iter().copied().enumerate() {
        for (i, x) in xs.iter().copied().enumerate() {
            if i == 0 || j == 0 || i + 1 == xs.len() || j + 1 == ys.len() {
                required.push(None);
                continue;
            }
            let tip = dc.tip_z_or(x, y, floor) + params.stock_to_leave;
            if !tip.is_finite() {
                return Err(invalid("required tip z is not finite"));
            }
            required.push(Some(tip));
        }
    }
    Ok(SampleGrid { xs, ys, required })
}

/// Coordinates from `min` to `max` spaced by at most `grid`, including both ends.
fn axis_coords(min: f64, max: f64, grid: f64) -> Result<Vec<f64>> {
    let span = max - min;
    if !(span > 0.0) {
        return Err(invalid("area is empty"));
    }
    let intervals = (span / grid).ceil();
    if !intervals.is_finite() || intervals < 1.0 || intervals > MAX_SAMPLES as f64 {
        return Err(invalid(format!("too many samples: limit is {MAX_SAMPLES}")));
    }
    let mut coords = Vec::new();
    for i in 0..intervals as usize {
        let x = min + grid * i as f64;
        if !x.is_finite() {
            return Err(invalid("grid coordinate is not finite"));
        }
        if x >= max {
            break;
        }
        coords.push(x);
    }
    if let Some(prev) = coords.last().copied()
        && (max - prev).abs() <= grid * 1e-9
    {
        coords.pop();
    }
    if coords.is_empty() {
        return Err(invalid("area is empty"));
    }
    coords.push(max);
    Ok(coords)
}

fn from_geom(err: campanion_geom::GeomError) -> crate::MeshError {
    invalid(err.to_string())
}

/// Scalar used by marching squares. Border samples sit strictly above `level`.
fn field_z(required: Option<f64>, level: f64) -> f64 {
    match required {
        Some(z) => z,
        None => {
            let frame = level + 1.0 + level.abs();
            if frame.is_finite() { frame } else { f64::MAX / 2.0 }
        }
    }
}

fn sample_required(grid: &SampleGrid, i: usize, j: usize) -> Result<Option<f64>> {
    let nx = grid.xs.len();
    let index = j.checked_mul(nx).and_then(|row| row.checked_add(i)).ok_or_else(|| invalid("sample index overflow"))?;
    grid.required.get(index).copied().ok_or_else(|| invalid("sample index is out of range"))
}

fn coord_at(coords: &[f64], index: usize) -> Result<f64> {
    coords.get(index).copied().ok_or_else(|| invalid("grid index is out of range"))
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct EdgeKey {
    i: u32,
    j: u32,
    vertical: bool,
}

struct Frag {
    from: EdgeKey,
    to: EdgeKey,
}

#[derive(Clone, Copy)]
enum Side {
    Bottom,
    Right,
    Top,
    Left,
}

fn trace_allowed(grid: &SampleGrid, level: f64, tol: &Tolerance) -> Result<Vec<Contour>> {
    let frags = march(grid, level)?;
    let loops = stitch(&frags, grid, level)?;
    loops_to_contours(&loops, tol)
}

fn march(grid: &SampleGrid, level: f64) -> Result<Vec<Frag>> {
    let nx = grid.xs.len();
    let ny = grid.ys.len();
    let mut frags = Vec::new();
    if nx < 2 || ny < 2 {
        return Ok(frags);
    }
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let zbl = field_z(sample_required(grid, i, j)?, level);
            let zbr = field_z(sample_required(grid, i + 1, j)?, level);
            let ztr = field_z(sample_required(grid, i + 1, j + 1)?, level);
            let ztl = field_z(sample_required(grid, i, j + 1)?, level);
            let case = u8::from(zbl <= level) | (u8::from(zbr <= level) << 1) | (u8::from(ztr <= level) << 2) | (u8::from(ztl <= level) << 3);
            let connected = saddle_connected(zbl, zbr, ztr, ztl, level);
            for (from, to) in cell_segments(case, connected) {
                frags.push(Frag { from: side_key(from, i, j)?, to: side_key(to, i, j)? });
            }
        }
    }
    Ok(frags)
}

/// Centre of the cell joins the allowed corners when the bilinear sample is allowed.
fn saddle_connected(zbl: f64, zbr: f64, ztr: f64, ztl: f64, level: f64) -> bool {
    let center = 0.25 * (zbl + zbr + ztr + ztl);
    center.is_finite() && center <= level
}

/// Directed crossings with the allowed side on the left. Cases 5 and 10 are the saddles.
fn cell_segments(case: u8, connected: bool) -> Vec<(Side, Side)> {
    match case {
        1 => vec![(Side::Bottom, Side::Left)],
        2 => vec![(Side::Right, Side::Bottom)],
        3 => vec![(Side::Right, Side::Left)],
        4 => vec![(Side::Top, Side::Right)],
        5 if connected => vec![(Side::Top, Side::Left), (Side::Bottom, Side::Right)],
        5 => vec![(Side::Bottom, Side::Left), (Side::Top, Side::Right)],
        6 => vec![(Side::Top, Side::Bottom)],
        7 => vec![(Side::Top, Side::Left)],
        8 => vec![(Side::Left, Side::Top)],
        9 => vec![(Side::Bottom, Side::Top)],
        10 if connected => vec![(Side::Left, Side::Bottom), (Side::Right, Side::Top)],
        10 => vec![(Side::Right, Side::Bottom), (Side::Left, Side::Top)],
        11 => vec![(Side::Right, Side::Top)],
        12 => vec![(Side::Left, Side::Right)],
        13 => vec![(Side::Bottom, Side::Right)],
        14 => vec![(Side::Left, Side::Bottom)],
        _ => Vec::new(),
    }
}

fn side_key(side: Side, i: usize, j: usize) -> Result<EdgeKey> {
    let (ei, ej, vertical) = match side {
        Side::Bottom => (i, j, false),
        Side::Right => (i + 1, j, true),
        Side::Top => (i, j + 1, false),
        Side::Left => (i, j, true),
    };
    let i = u32::try_from(ei).map_err(|_| invalid("grid index does not fit"))?;
    let j = u32::try_from(ej).map_err(|_| invalid("grid index does not fit"))?;
    Ok(EdgeKey { i, j, vertical })
}

fn stitch(frags: &[Frag], grid: &SampleGrid, level: f64) -> Result<Vec<Vec<(f64, f64)>>> {
    let mut start_at: HashMap<EdgeKey, usize> = HashMap::new();
    for (idx, frag) in frags.iter().enumerate() {
        if start_at.insert(frag.from, idx).is_some() {
            return Err(invalid("marching squares edge has two exits"));
        }
    }
    let mut seen = vec![false; frags.len()];
    let mut loops = Vec::new();
    for start in 0..frags.len() {
        if seen.get(start).copied() != Some(false) {
            continue;
        }
        let mut idx = start;
        let mut points = Vec::new();
        loop {
            let flag = seen.get_mut(idx).ok_or_else(|| invalid("marching squares index is out of range"))?;
            if *flag {
                return Err(invalid("marching squares loop did not close"));
            }
            *flag = true;
            let frag = frags.get(idx).ok_or_else(|| invalid("marching squares index is out of range"))?;
            points.push(edge_point(grid, frag.from, level)?);
            let next = start_at.get(&frag.to).copied().ok_or_else(|| invalid("open marching squares loop"))?;
            if next == start {
                break;
            }
            idx = next;
            if points.len() > frags.len() {
                return Err(invalid("marching squares loop did not close"));
            }
        }
        loops.push(points);
    }
    Ok(loops)
}

/// Linear crossing of `level` along the edge, kept off the corners so loops do not collapse.
fn edge_point(grid: &SampleGrid, key: EdgeKey, level: f64) -> Result<(f64, f64)> {
    let i = key.i as usize;
    let j = key.j as usize;
    let (i1, j1) = if key.vertical { (i, j + 1) } else { (i + 1, j) };
    let z0 = field_z(sample_required(grid, i, j)?, level);
    let z1 = field_z(sample_required(grid, i1, j1)?, level);
    let denom = z1 - z0;
    let mut t = if !denom.is_finite() || denom.abs() <= 1e-15 { 0.5 } else { (level - z0) / denom };
    if !t.is_finite() {
        t = 0.5;
    }
    t = t.clamp(1.0e-4, 1.0 - 1.0e-4);
    let x0 = coord_at(&grid.xs, i)?;
    let y0 = coord_at(&grid.ys, j)?;
    let x1 = coord_at(&grid.xs, i1)?;
    let y1 = coord_at(&grid.ys, j1)?;
    Ok((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
}

fn loops_to_contours(loops: &[Vec<(f64, f64)>], tol: &Tolerance) -> Result<Vec<Contour>> {
    let mut out = Vec::new();
    for points in loops {
        let Some(contour) = contour_from_points(points, tol)? else {
            continue;
        };
        match contour.orientation(tol).map_err(from_geom)? {
            Orientation::Degenerate => {}
            Orientation::CounterClockwise | Orientation::Clockwise => out.push(contour),
        }
    }
    Ok(out)
}

fn contour_from_points(points: &[(f64, f64)], tol: &Tolerance) -> Result<Option<Contour>> {
    let eps = tol.linear().max(1e-9);
    let mut cleaned: Vec<(f64, f64)> = Vec::new();
    for &(x, y) in points {
        if !x.is_finite() || !y.is_finite() {
            return Err(invalid("marching squares point is not finite"));
        }
        if cleaned.last().is_some_and(|&(px, py)| (x - px).hypot(y - py) <= eps) {
            continue;
        }
        cleaned.push((x, y));
    }
    if let (Some(&(x0, y0)), Some(&(x1, y1))) = (cleaned.first(), cleaned.last())
        && (x0 - x1).hypot(y0 - y1) <= eps
    {
        cleaned.pop();
    }
    if cleaned.len() < 3 || cleaned.len() > campanion_geom::MAX_SEGMENTS {
        if cleaned.len() > campanion_geom::MAX_SEGMENTS {
            return Err(invalid(format!("too many segments: {} > {}", cleaned.len(), campanion_geom::MAX_SEGMENTS)));
        }
        return Ok(None);
    }
    let mut segs = Vec::with_capacity(cleaned.len());
    for i in 0..cleaned.len() {
        let (x0, y0) = *cleaned.get(i).ok_or_else(|| invalid("loop index is out of range"))?;
        let next = (i + 1) % cleaned.len();
        let (x1, y1) = *cleaned.get(next).ok_or_else(|| invalid("loop index is out of range"))?;
        let a = Point::new(x0, y0).map_err(from_geom)?;
        let b = Point::new(x1, y1).map_err(from_geom)?;
        segs.push(Segment::Line(Line::new(a, b).map_err(from_geom)?));
    }
    Ok(Some(Contour::try_new(segs, true, tol).map_err(from_geom)?))
}

/// Pair even-depth loops (outers, forced CCW) with their odd-depth holes (forced CW).
fn group_regions(loops: &[Contour], tol: &Tolerance) -> Result<Vec<Region>> {
    let n = loops.len();
    let parent = nest_parents(loops, tol)?;
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (index, maybe) in parent.iter().enumerate() {
        if let Some(p) = *maybe
            && let Some(list) = children.get_mut(p)
        {
            list.push(index);
        }
    }
    let mut regions = Vec::new();
    for index in 0..n {
        if nest_depth(index, &parent)? % 2 != 0 {
            continue;
        }
        let outer_raw = loops.get(index).ok_or_else(|| invalid("z-level loop is missing"))?;
        let outer = orient(outer_raw, true, tol)?;
        let mut holes = Vec::new();
        let kids = children.get(index).map(Vec::as_slice).unwrap_or(&[]);
        for &child in kids {
            if nest_depth(child, &parent)? % 2 != 1 {
                continue;
            }
            let hole = loops.get(child).ok_or_else(|| invalid("z-level hole is missing"))?;
            holes.push(orient(hole, false, tol)?);
        }
        regions.push(Region::try_new(outer, holes, tol).map_err(from_geom)?);
    }
    Ok(regions)
}

fn nest_parents(loops: &[Contour], tol: &Tolerance) -> Result<Vec<Option<usize>>> {
    let n = loops.len();
    let mut parent = vec![None; n];
    for i in 0..n {
        let inner = loops.get(i).ok_or_else(|| invalid("z-level loop is missing"))?;
        let mut best: Option<usize> = None;
        for j in 0..n {
            if i == j {
                continue;
            }
            let outer = loops.get(j).ok_or_else(|| invalid("z-level loop is missing"))?;
            if !contains_loop(outer, inner, tol)? {
                continue;
            }
            let tighter = match best {
                None => true,
                Some(b) => loop_area(outer) < loop_area(loops.get(b).ok_or_else(|| invalid("z-level loop is missing"))?),
            };
            if tighter {
                best = Some(j);
            }
        }
        let slot = parent.get_mut(i).ok_or_else(|| invalid("z-level loop is missing"))?;
        *slot = best;
    }
    Ok(parent)
}

fn nest_depth(index: usize, parent: &[Option<usize>]) -> Result<usize> {
    let mut depth = 0usize;
    let mut cursor = index;
    let mut guard = 0usize;
    while let Some(next) = parent.get(cursor).copied().flatten() {
        depth += 1;
        guard += 1;
        if guard > parent.len() {
            return Err(invalid("z-level loop nesting cycled"));
        }
        cursor = next;
    }
    Ok(depth)
}

fn contains_loop(outer: &Contour, inner: &Contour, tol: &Tolerance) -> Result<bool> {
    for seg in inner.segments() {
        let a = seg.start();
        let b = seg.end();
        let mid = Point::new((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5).map_err(from_geom)?;
        match outer.containment(mid, tol).map_err(from_geom)? {
            Containment::Inside => return Ok(true),
            Containment::Outside => return Ok(false),
            Containment::Boundary => {}
        }
    }
    Ok(false)
}

fn loop_area(contour: &Contour) -> f64 {
    let mut sum = 0.0;
    for seg in contour.segments() {
        let a = seg.start();
        let b = seg.end();
        sum += a.x() * b.y() - b.x() * a.y();
    }
    (0.5 * sum).abs()
}

fn orient(contour: &Contour, want_ccw: bool, tol: &Tolerance) -> Result<Contour> {
    match contour.orientation(tol).map_err(from_geom)? {
        Orientation::CounterClockwise => Ok(if want_ccw { contour.clone() } else { contour.reversed() }),
        Orientation::Clockwise => Ok(if want_ccw { contour.reversed() } else { contour.clone() }),
        Orientation::Degenerate => Err(invalid("degenerate z-level loop")),
    }
}

/// Boundary first, then insets at `stepover`, `2 * stepover`, … until the region is gone.
fn append_stepovers(region: &Region, stepover: f64, tol: &Tolerance, out: &mut Vec<Contour>) -> Result<()> {
    push_boundaries(region, out);
    for step in 1..MAX_RINGS {
        let distance = stepover * step as f64;
        if !distance.is_finite() {
            return Err(invalid("offset distance is not finite"));
        }
        let pieces = offset_region(region, distance, tol).map_err(from_geom)?;
        if pieces.is_empty() {
            return Ok(());
        }
        let before = out.len();
        for piece in &pieces {
            push_boundaries(piece, out);
        }
        if out.len() == before {
            return Ok(());
        }
    }
    let distance = stepover * MAX_RINGS as f64;
    if !distance.is_finite() {
        return Err(invalid("offset distance is not finite"));
    }
    let extra = offset_region(region, distance, tol).map_err(from_geom)?;
    if extra.is_empty() { Ok(()) } else { Err(invalid(format!("too many offset rings: limit is {MAX_RINGS}"))) }
}

fn push_boundaries(region: &Region, out: &mut Vec<Contour>) {
    out.push(region.outer().clone());
    for hole in region.holes() {
        out.push(hole.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cutter, Mesh, MeshError, Triangle, Vec3};

    fn vec3(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z)
    }

    fn triangle(a: Vec3, b: Vec3, c: Vec3) -> Triangle {
        Triangle { a, b, c }
    }

    fn quad(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> [Triangle; 2] {
        [triangle(a, b, c), triangle(a, c, d)]
    }

    /// Plane at z = 0 plus a block. The block alone is 12 triangles.
    pub(super) fn plane_and_block(x0: f64, y0: f64, x1: f64, y1: f64, height: f64) -> Mesh {
        let mut tris = Vec::new();
        let s = 200.0;
        let [a, b] = quad(vec3(-s, -s, 0.0), vec3(s, -s, 0.0), vec3(s, s, 0.0), vec3(-s, s, 0.0));
        tris.push(a);
        tris.push(b);
        let p = |x, y, z| vec3(x, y, z);
        let faces = [
            [p(x0, y0, 0.0), p(x1, y0, 0.0), p(x1, y1, 0.0), p(x0, y1, 0.0)],
            [p(x0, y0, height), p(x0, y1, height), p(x1, y1, height), p(x1, y0, height)],
            [p(x0, y0, 0.0), p(x0, y0, height), p(x1, y0, height), p(x1, y0, 0.0)],
            [p(x0, y1, 0.0), p(x1, y1, 0.0), p(x1, y1, height), p(x0, y1, height)],
            [p(x0, y0, 0.0), p(x0, y1, 0.0), p(x0, y1, height), p(x0, y0, height)],
            [p(x1, y0, 0.0), p(x1, y0, height), p(x1, y1, height), p(x1, y1, 0.0)],
        ];
        for face in faces {
            let [a, b] = quad(face[0], face[1], face[2], face[3]);
            tris.push(a);
            tris.push(b);
        }
        Mesh::new(tris).expect("block mesh")
    }

    fn params(step_down: f64, stepover: f64, grid: f64, top: f64, bottom: f64) -> ZLevelParams {
        ZLevelParams { step_down, stepover, grid, stock_to_leave: 0.0, top_z: top, bottom_z: bottom }
    }

    pub(super) fn tol() -> Tolerance {
        Tolerance::new(1e-6, 1e-6).expect("tolerance")
    }

    fn axis_like(min: f64, max: f64, grid: f64) -> Vec<f64> {
        let intervals = ((max - min) / grid).ceil() as usize;
        let mut coords = Vec::new();
        for i in 0..intervals {
            let x = min + grid * i as f64;
            if x >= max {
                break;
            }
            coords.push(x);
        }
        if let Some(prev) = coords.last().copied()
            && (max - prev).abs() <= grid * 1e-9
        {
            coords.pop();
        }
        coords.push(max);
        coords
    }

    fn push_samples(seg: Segment, step: f64, out: &mut Vec<(f64, f64)>) {
        match seg {
            Segment::Line(line) => {
                let a = line.start();
                let b = line.end();
                let len = (b.x() - a.x()).hypot(b.y() - a.y());
                let n = ((len / step).ceil() as usize).clamp(1, 200_000);
                for i in 0..=n {
                    let t = i as f64 / n as f64;
                    out.push((a.x() + (b.x() - a.x()) * t, a.y() + (b.y() - a.y()) * t));
                }
            }
            Segment::Arc(arc) => {
                let len = (arc.radius() * arc.sweep().abs()).max(step);
                let n = ((len / step).ceil() as usize).clamp(1, 200_000);
                for i in 0..=n {
                    let p = arc.point_at(i as f64 / n as f64);
                    out.push((p.x(), p.y()));
                }
            }
        }
    }

    pub(super) fn ring_samples(rings: &[Contour], step: f64) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        for ring in rings {
            for seg in ring.segments() {
                push_samples(*seg, step, &mut out);
            }
        }
        out
    }

    fn assert_no_gouge(dc: &DropCutter, rings: &[Contour], level: f64) {
        for (x, y) in ring_samples(rings, 0.1) {
            let tip = dc.tip_z_or(x, y, -1.0);
            assert!(tip <= level + 1e-9, "gouge at ({x}, {y}): tip {tip} > {level}");
        }
    }

    #[test]
    fn levels_match_the_depth_plan_and_empty_area_errors() {
        let mesh = plane_and_block(0.0, 0.0, 10.0, 10.0, 5.0);
        assert_eq!(mesh.triangles().len(), 14);
        let dc = DropCutter::new(&mesh, Cutter::new_flat(3.0).expect("flat")).expect("index");
        let levels = zlevel_rough(&dc, (0.0, 0.0), (6.0, 6.0), &params(3.0, 2.0, 2.0, 10.0, 0.0), &tol()).expect("levels");
        let zs: Vec<f64> = levels.iter().map(|level| level.z).collect();
        assert_eq!(zs, vec![7.0, 4.0, 1.0, 0.0]);
        let empty = zlevel_rough(&dc, (0.0, 0.0), (0.0, 6.0), &params(3.0, 2.0, 2.0, 10.0, 0.0), &tol()).expect_err("empty");
        assert!(empty.to_string().contains("empty"), "{empty}");
        assert!(matches!(zlevel_rough(&dc, (2.0, 0.0), (1.0, 4.0), &params(3.0, 2.0, 2.0, 10.0, 0.0), &tol()), Err(MeshError::InvalidInput(_))));
        let below = zlevel_rough(&dc, (0.0, 0.0), (6.0, 6.0), &params(2.0, 2.0, 2.0, 0.0, -2.0), &tol()).expect("below");
        assert_eq!(below.iter().map(|level| level.z).collect::<Vec<_>>(), vec![-2.0]);
        assert!(below.iter().all(|level| level.rings.is_empty()));
    }

    #[test]
    fn block_level_is_gouge_free_and_covers_the_interior() {
        let mesh = plane_and_block(0.0, 0.0, 10.0, 10.0, 5.0);
        let dc = DropCutter::new(&mesh, Cutter::new_flat(3.0).expect("flat")).expect("index");
        assert!((dc.tip_z_or(5.0, 5.0, -1.0) - 5.0).abs() < 1e-6);
        assert!(dc.tip_z_or(-6.0, -6.0, -1.0).abs() < 1e-6);
        let area_min = (-8.0, -8.0);
        let area_max = (18.0, 18.0);
        let grid = 1.0;
        let stepover = 2.0;
        let rough = params(2.5, stepover, grid, 5.0, 0.0);
        let levels = zlevel_rough(&dc, area_min, area_max, &rough, &tol()).expect("rough");
        assert_eq!(levels.iter().map(|level| level.z).collect::<Vec<_>>(), vec![2.5, 0.0]);
        let level = levels.iter().find(|level| (level.z - 2.5).abs() < 1e-9).expect("z = 2.5");
        assert!(!level.rings.is_empty());
        assert_no_gouge(&dc, &level.rings, 2.5);
        let samples = ring_samples(&level.rings, 0.05);
        let xs = axis_like(area_min.0, area_max.0, grid);
        let ys = axis_like(area_min.1, area_max.1, grid);
        for (j, y) in ys.iter().copied().enumerate() {
            for (i, x) in xs.iter().copied().enumerate() {
                let border = i == 0 || j == 0 || i + 1 == xs.len() || j + 1 == ys.len();
                let allowed = !border && dc.tip_z_or(x, y, -1.0) <= 2.5;
                if !allowed {
                    continue;
                }
                let mut band = false;
                'near: for (jb, yb) in ys.iter().copied().enumerate() {
                    for (ib, xb) in xs.iter().copied().enumerate() {
                        let other_border = ib == 0 || jb == 0 || ib + 1 == xs.len() || jb + 1 == ys.len();
                        let other_ok = !other_border && dc.tip_z_or(xb, yb, -1.0) <= 2.5;
                        if !other_ok && (x - xb).hypot(y - yb) <= 2.0 * grid {
                            band = true;
                            break 'near;
                        }
                    }
                }
                if band {
                    continue;
                }
                let dist = samples.iter().map(|(sx, sy)| (x - sx).hypot(y - sy)).fold(f64::INFINITY, f64::min);
                assert!(dist <= stepover, "({x}, {y}) is {dist} from every ring");
            }
        }
        let floor = levels.iter().find(|level| level.z.abs() < 1e-9).expect("z = 0");
        assert_no_gouge(&dc, &floor.rings, 0.0);
    }

    #[test]
    fn rejects_bad_inputs_and_hostile_counts() {
        let mesh = Mesh::new(vec![]).expect("empty");
        let dc = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index");
        let tolerance = tol();
        let base = params(1.0, 1.0, 1.0, 1.0, 0.0);
        let bad = [
            params(0.0, 1.0, 1.0, 1.0, 0.0),
            params(1.0, 0.0, 1.0, 1.0, 0.0),
            params(1.0, 1.0, f64::NAN, 1.0, 0.0),
            ZLevelParams { stock_to_leave: -0.1, ..base },
            ZLevelParams { top_z: 0.0, bottom_z: 0.0, ..base },
            ZLevelParams { top_z: -1.0, bottom_z: 0.0, ..base },
            ZLevelParams { step_down: 1e-9, ..base },
        ];
        for params in bad {
            assert!(matches!(zlevel_rough(&dc, (0.0, 0.0), (4.0, 4.0), &params, &tolerance), Err(MeshError::InvalidInput(_))));
        }
        assert!(matches!(zlevel_rough(&dc, (f64::NAN, 0.0), (2.0, 2.0), &base, &tolerance), Err(MeshError::InvalidInput(_))));
        let huge = zlevel_rough(&dc, (0.0, 0.0), (1.0e6, 1.0e6), &params(1.0, 1.0, 0.1, 1.0, 0.0), &tolerance).expect_err("samples");
        assert!(huge.to_string().contains("too many samples"), "{huge}");
    }
}

#[cfg(test)]
mod proptests {
    use super::tests::{plane_and_block, ring_samples, tol};
    use super::*;
    use crate::Cutter;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(16))]

        #[test]
        fn random_block_rings_do_not_gouge(
            x in -4.0..12.0f64,
            y in -4.0..12.0f64,
            w in 1.5..10.0f64,
            h in 1.5..10.0f64,
            height in 0.8..8.0f64,
        ) {
            let mesh = plane_and_block(x, y, x + w, y + h, height);
            let dc = DropCutter::new(&mesh, Cutter::new_flat(3.0).expect("flat")).expect("index");
            let top = height.max(1.0);
            let rough = ZLevelParams {
                step_down: top / 2.0,
                stepover: 2.0,
                grid: 1.0,
                stock_to_leave: 0.0,
                top_z: top,
                bottom_z: 0.0,
            };
            let levels = zlevel_rough(&dc, (-10.0, -10.0), (28.0, 28.0), &rough, &tol()).unwrap_or_else(|err| panic!("{err}"));
            prop_assert_eq!(levels.len(), 2);
            for level in &levels {
                for (px, py) in ring_samples(&level.rings, 0.1) {
                    let tip = dc.tip_z_or(px, py, -1.0);
                    prop_assert!(tip <= level.z + 1e-9, "gouge at ({px}, {py}): tip {tip} > {}", level.z);
                }
            }
        }
    }
}
