//! Parallel (raster) finishing. Each pass is a straight tool-tip polyline on a [`crate::DropCutter`].

use crate::{DropCutter, Result, Vec3, invalid};

/// Most raster lines in one finish. A hostile stepover fails instead of allocating without bound.
const MAX_LINES: usize = 100_000;

/// Most tool-tip samples, including those inserted by refinement, across every line.
const MAX_POINTS: usize = 1_000_000;

/// Deepest midpoint bisection. Depth 0 is the span between two regular samples.
const MAX_DEPTH: u32 = 12;

/// Direction of the raster passes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RasterAngle {
    /// Passes run parallel to X and are spaced along Y.
    AlongX,
    /// Passes run parallel to Y and are spaced along X.
    AlongY,
}

/// Spacing, tolerance, and floor for [`parallel_finish`].
///
/// Distances are millimetres. `stepover` is the maximum gap between passes. `sample_step` is the
/// maximum gap between the regular samples that refinement starts from. `tolerance` is the largest
/// tip-Z error allowed when a chord replaces the true drop-cutter height. `floor_z` is the tip
/// height used where the cutter misses the mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RasterParams {
    pub stepover: f64,
    pub sample_step: f64,
    pub tolerance: f64,
    pub angle: RasterAngle,
    pub floor_z: f64,
    pub zigzag: bool,
}

/// One tool-tip polyline per raster line over the axis-aligned rectangle `area_min`..=`area_max`.
///
/// Lines are ordered from the low cross coordinate to the high one and include both edges. With
/// `zigzag`, line 0 runs from the low along coordinate to the high one and each later line flips.
/// Otherwise every line runs low to high. A zero-width cross span is a single line.
///
/// Points on a line are `DropCutter::tip_z_or` samples. Midpoints whose true tip Z differs from
/// the chord by more than `tolerance` are inserted, up to [`MAX_DEPTH`] bisections. Interior
/// points are then dropped when every dropped point stays within `tolerance` of the remaining chord.
pub fn parallel_finish(dc: &DropCutter, area_min: (f64, f64), area_max: (f64, f64), params: &RasterParams) -> Result<Vec<Vec<Vec3>>> {
    validate(area_min, area_max, params)?;
    let (along, cross) = axis_spans(params.angle, area_min, area_max);
    let line_count = position_count(cross.end - cross.start, params.stepover, MAX_LINES, "raster lines")?;
    let sample_count = position_count(along.end - along.start, params.sample_step, MAX_POINTS, "raster samples")?;
    if line_count.saturating_mul(sample_count) > MAX_POINTS {
        return Err(invalid(format!("too many raster points: limit is {MAX_POINTS}")));
    }
    let along_at = coordinates(along.start, along.end, sample_count)?;
    let cross_at = coordinates(cross.start, cross.end, line_count)?;
    let mut lines = Vec::with_capacity(line_count);
    let mut total = 0usize;
    for (index, &cross_value) in cross_at.iter().enumerate() {
        let mut line = sample_line(dc, params, &along_at, cross_value, &mut total)?;
        line = simplify(dc, &line, params.tolerance, params.floor_z)?;
        if params.zigzag && index % 2 == 1 {
            line.reverse();
        }
        lines.push(line);
    }
    Ok(lines)
}

struct Span {
    start: f64,
    end: f64,
}

fn axis_spans(angle: RasterAngle, area_min: (f64, f64), area_max: (f64, f64)) -> (Span, Span) {
    match angle {
        RasterAngle::AlongX => (Span { start: area_min.0, end: area_max.0 }, Span { start: area_min.1, end: area_max.1 }),
        RasterAngle::AlongY => (Span { start: area_min.1, end: area_max.1 }, Span { start: area_min.0, end: area_max.0 }),
    }
}

fn validate(area_min: (f64, f64), area_max: (f64, f64), params: &RasterParams) -> Result<()> {
    check_positive(params.stepover, "stepover")?;
    check_positive(params.sample_step, "sample_step")?;
    check_positive(params.tolerance, "tolerance")?;
    if !params.floor_z.is_finite() {
        return Err(invalid("floor_z must be finite"));
    }
    let coords = [area_min.0, area_min.1, area_max.0, area_max.1];
    if !coords.iter().all(|value| value.is_finite()) {
        return Err(invalid("area coordinate is not finite"));
    }
    if area_min.0 > area_max.0 || area_min.1 > area_max.1 {
        return Err(invalid("area minimum is greater than maximum"));
    }
    Ok(())
}

fn check_positive(value: f64, what: &str) -> Result<()> {
    if value.is_finite() && value > 0.0 { Ok(()) } else { Err(invalid(format!("{what} must be finite and positive"))) }
}

/// `ceil(span / step) + 1` positions, or one position when `span` is zero. The count is at most `limit`.
fn position_count(span: f64, step: f64, limit: usize, what: &str) -> Result<usize> {
    if span == 0.0 {
        return Ok(1);
    }
    let mut intervals = (span / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(invalid(format!("too many {what}: limit is {limit}")));
    }
    // `ceil` can leave `span / intervals` a fraction over `step`.
    let mut guard = 0u32;
    while span > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(invalid(format!("{what} spacing is not usable")));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > limit as f64 {
        return Err(invalid(format!("too many {what}: limit is {limit}")));
    }
    Ok(count as usize)
}

fn coordinates(start: f64, end: f64, count: usize) -> Result<Vec<f64>> {
    if count == 0 {
        return Err(invalid("raster produced no positions"));
    }
    if count == 1 {
        return Ok(vec![start]);
    }
    let mut out = Vec::with_capacity(count);
    let gaps = (count - 1) as f64;
    let span = end - start;
    for index in 0..count - 1 {
        let value = start + span * (index as f64) / gaps;
        if !value.is_finite() {
            return Err(invalid("raster coordinate is not finite"));
        }
        out.push(value);
    }
    out.push(end);
    Ok(out)
}

fn to_xy(angle: RasterAngle, along: f64, cross: f64) -> (f64, f64) {
    match angle {
        RasterAngle::AlongX => (along, cross),
        RasterAngle::AlongY => (cross, along),
    }
}

fn tip_point(dc: &DropCutter, x: f64, y: f64, floor: f64) -> Result<Vec3> {
    let z = dc.tip_z_or(x, y, floor);
    if z.is_finite() { Ok(Vec3::new(x, y, z)) } else { Err(invalid("drop-cutter tip is not finite")) }
}

fn push_point(points: &mut Vec<Vec3>, point: Vec3, total: &mut usize) -> Result<()> {
    points.push(point);
    *total += 1;
    if *total > MAX_POINTS { Err(invalid(format!("too many raster points: limit is {MAX_POINTS}"))) } else { Ok(()) }
}

fn sample_line(dc: &DropCutter, params: &RasterParams, along: &[f64], cross: f64, total: &mut usize) -> Result<Vec<Vec3>> {
    let mut points = Vec::with_capacity(along.len());
    for &along_at in along {
        let (x, y) = to_xy(params.angle, along_at, cross);
        let point = tip_point(dc, x, y, params.floor_z)?;
        if let Some(previous) = points.last().copied() {
            refine_between(dc, previous, point, params.floor_z, params.tolerance, 0, &mut points, total)?;
        }
        push_point(&mut points, point, total)?;
    }
    Ok(points)
}

/// Insert the true midpoint while its tip Z misses the chord by more than `tolerance`.
fn refine_between(
    dc: &DropCutter,
    a: Vec3,
    b: Vec3,
    floor: f64,
    tolerance: f64,
    depth: u32,
    points: &mut Vec<Vec3>,
    total: &mut usize,
) -> Result<()> {
    if depth >= MAX_DEPTH {
        return Ok(());
    }
    let mid_x = (a.x + b.x) * 0.5;
    let mid_y = (a.y + b.y) * 0.5;
    let interpolated = (a.z + b.z) * 0.5;
    let true_z = dc.tip_z_or(mid_x, mid_y, floor);
    if (true_z - interpolated).abs() <= tolerance {
        return Ok(());
    }
    let mid = tip_point(dc, mid_x, mid_y, floor)?;
    refine_between(dc, a, mid, floor, tolerance, depth + 1, points, total)?;
    push_point(points, mid, total)?;
    refine_between(dc, mid, b, floor, tolerance, depth + 1, points, total)
}

/// Drop interior points when the surviving chord stays within `tolerance` of each dropped point.
///
/// A span is kept whole only when every interior sample is within `tolerance` of that chord and
/// dyadic samples of the chord agree with [`DropCutter::tip_z_or`]. A point whose removal would
/// leave a neighbour chord outside that band is kept.
fn simplify(dc: &DropCutter, points: &[Vec3], tolerance: f64, floor: f64) -> Result<Vec<Vec3>> {
    let n = points.len();
    if n <= 2 {
        return Ok(points.to_vec());
    }
    let mut keep = vec![false; n];
    mark(&mut keep, 0)?;
    mark(&mut keep, n - 1)?;
    let mut stack = vec![(0usize, n - 1)];
    while let Some((start, end)) = stack.pop() {
        if end <= start.saturating_add(1) {
            continue;
        }
        let (a, b) = ends(points, start, end)?;
        if let Some(split) = split_at(dc, points, start, end, a, b, tolerance, floor)? {
            mark(&mut keep, split)?;
            stack.push((start, split));
            stack.push((split, end));
        }
    }
    let mut out = Vec::new();
    for (index, point) in points.iter().enumerate() {
        if keep.get(index) == Some(&true) {
            out.push(*point);
        }
    }
    Ok(out)
}

fn mark(keep: &mut [bool], index: usize) -> Result<()> {
    let slot = keep.get_mut(index).ok_or_else(|| invalid("raster index is out of range"))?;
    *slot = true;
    Ok(())
}

fn ends(points: &[Vec3], start: usize, end: usize) -> Result<(Vec3, Vec3)> {
    match (points.get(start), points.get(end)) {
        (Some(a), Some(b)) => Ok((*a, *b)),
        _ => Err(invalid("raster index is out of range")),
    }
}

/// Index to keep, or `None` when `start..=end` is already within tolerance.
fn split_at(dc: &DropCutter, points: &[Vec3], start: usize, end: usize, a: Vec3, b: Vec3, tolerance: f64, floor: f64) -> Result<Option<usize>> {
    let offender = farthest_over(points, start, end, a, b, tolerance);
    if offender.is_none() && chord_tracks_surface(dc, a, b, tolerance, floor) {
        return Ok(None);
    }
    let split = match offender {
        Some(index) => index,
        None => nearest_to_mid(points, start, end)?,
    };
    if split <= start || split >= end { Ok(None) } else { Ok(Some(split)) }
}

fn farthest_over(points: &[Vec3], start: usize, end: usize, a: Vec3, b: Vec3, tolerance: f64) -> Option<usize> {
    let mut max_dev = tolerance;
    let mut found = None;
    for index in (start + 1)..end {
        let dev = points.get(index).map(|point| z_deviation(*point, a, b)).unwrap_or(0.0);
        if dev > max_dev {
            max_dev = dev;
            found = Some(index);
        }
    }
    found
}

fn nearest_to_mid(points: &[Vec3], start: usize, end: usize) -> Result<usize> {
    let (a, b) = ends(points, start, end)?;
    let mid_x = (a.x + b.x) * 0.5;
    let mid_y = (a.y + b.y) * 0.5;
    let mut best_index = None;
    let mut best_dist = f64::INFINITY;
    for index in (start + 1)..end {
        let Some(point) = points.get(index) else {
            continue;
        };
        let dist = (point.x - mid_x).hypot(point.y - mid_y);
        if dist < best_dist {
            best_dist = dist;
            best_index = Some(index);
        }
    }
    best_index.ok_or_else(|| invalid("raster segment has no interior point"))
}

/// Dyadic samples of chord `a`–`b` stay within `tolerance` of [`DropCutter::tip_z_or`].
///
/// The limit sits a hair under `tolerance` so the unsampled bow between those checks cannot
/// push a smooth tool surface past the caller's tolerance.
fn chord_tracks_surface(dc: &DropCutter, a: Vec3, b: Vec3, tolerance: f64, floor: f64) -> bool {
    let margin = 1e-4_f64.min(tolerance * 0.05);
    let limit = tolerance - margin;
    let span = (b.x - a.x).hypot(b.y - a.y);
    tracks_span(dc, a, b, limit, floor, span, 0.0, 1.0, 0)
}

fn tracks_span(dc: &DropCutter, a: Vec3, b: Vec3, limit: f64, floor: f64, full_span: f64, t0: f64, t1: f64, depth: u32) -> bool {
    let t = (t0 + t1) * 0.5;
    let x = a.x + (b.x - a.x) * t;
    let y = a.y + (b.y - a.y) * t;
    let interpolated = a.z + (b.z - a.z) * t;
    if (dc.tip_z_or(x, y, floor) - interpolated).abs() > limit {
        return false;
    }
    let piece = full_span * (t1 - t0);
    if depth >= 8 || piece <= 1e-3 {
        return true;
    }
    tracks_span(dc, a, b, limit, floor, full_span, t0, t, depth + 1) && tracks_span(dc, a, b, limit, floor, full_span, t, t1, depth + 1)
}

/// Absolute tip-Z distance from `point` to the chord `a`–`b`.
fn z_deviation(point: Vec3, a: Vec3, b: Vec3) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 1e-24 { ((point.x - a.x) * dx + (point.y - a.y) * dy) / len2 } else { 0.5 };
    let dev = (point.z - (a.z + (b.z - a.z) * t)).abs();
    if dev.is_finite() { dev } else { f64::INFINITY }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cutter, Mesh, MeshError, Triangle};
    use proptest::prelude::*;

    fn plane_at(z: f64) -> Mesh {
        let s = 40.0;
        Mesh::new(vec![
            Triangle { a: Vec3::new(-s, -s, z), b: Vec3::new(s, -s, z), c: Vec3::new(s, s, z) },
            Triangle { a: Vec3::new(-s, -s, z), b: Vec3::new(s, s, z), c: Vec3::new(-s, s, z) },
        ])
        .expect("plane")
    }

    fn params(stepover: f64, sample_step: f64, angle: RasterAngle, zigzag: bool) -> RasterParams {
        RasterParams { stepover, sample_step, tolerance: 0.05, angle, floor_z: 0.0, zigzag }
    }

    fn finish(area_min: (f64, f64), area_max: (f64, f64), params: &RasterParams) -> Vec<Vec<Vec3>> {
        let mesh = plane_at(5.0);
        let dc = DropCutter::new(&mesh, Cutter::new_ball(3.0).expect("ball")).expect("index");
        parallel_finish(&dc, area_min, area_max, params).expect("finish")
    }

    #[test]
    fn flat_plane_stays_at_the_surface_and_filters_to_endpoints() {
        let stepover = 1.5;
        let lines = finish((0.0, 0.0), (4.0, 4.0), &params(stepover, 0.5, RasterAngle::AlongX, false));
        let expected = (4.0_f64 / stepover).ceil() as usize + 1;
        assert_eq!(lines.len(), expected);
        assert!(expected >= 3, "interior samples must exist for the filter to matter");
        for (index, line) in lines.iter().enumerate() {
            assert_eq!(line.len(), 2, "line {index} kept interior samples: {line:?}");
            assert!((line[0].x - 0.0).abs() < 1e-9);
            assert!((line[1].x - 4.0).abs() < 1e-9);
            let y = 4.0 * index as f64 / (expected - 1) as f64;
            for point in line {
                assert!((point.z - 5.0).abs() < 1e-9, "{point:?}");
                assert!((point.y - y).abs() < 1e-9, "line {index} y {} != {y}", point.y);
            }
        }
        for pair in lines.windows(2) {
            let gap = pair[1][0].y - pair[0][0].y;
            assert!(gap > 0.0 && gap <= stepover + 1e-9, "gap {gap}");
        }
        let one = finish((1.0, 2.5), (3.0, 2.5), &params(1.0, 0.25, RasterAngle::AlongX, true));
        assert_eq!(one.len(), 1);
        assert_eq!(one[0].len(), 2);
        assert!((one[0][0].y - 2.5).abs() < 1e-9);
    }

    #[test]
    fn zigzag_alternates_and_along_y_swaps_axes() {
        let x_lines = finish((0.0, 0.0), (4.0, 4.0), &params(2.0, 1.0, RasterAngle::AlongX, true));
        assert_eq!(x_lines.len(), 3);
        for (index, line) in x_lines.iter().enumerate() {
            assert_eq!(line.len(), 2);
            let (start_x, end_x) = if index % 2 == 0 { (0.0, 4.0) } else { (4.0, 0.0) };
            assert!((line[0].x - start_x).abs() < 1e-9 && (line[1].x - end_x).abs() < 1e-9);
            assert!((line[0].y - line[1].y).abs() < 1e-9);
        }
        let straight = finish((0.0, 0.0), (4.0, 4.0), &params(2.0, 1.0, RasterAngle::AlongX, false));
        assert!(straight.iter().all(|line| line[0].x < line[1].x));

        let y_lines = finish((0.0, 0.0), (2.0, 6.0), &params(2.0, 1.0, RasterAngle::AlongY, true));
        assert_eq!(y_lines.len(), (2.0_f64 / 2.0).ceil() as usize + 1);
        assert_eq!(y_lines.len(), 2);
        assert!((y_lines[0][0].x - 0.0).abs() < 1e-9 && (y_lines[1][0].x - 2.0).abs() < 1e-9);
        assert!((y_lines[0][0].y - 0.0).abs() < 1e-9 && (y_lines[0][1].y - 6.0).abs() < 1e-9);
        assert!((y_lines[1][0].y - 6.0).abs() < 1e-9 && (y_lines[1][1].y - 0.0).abs() < 1e-9);
        assert!(y_lines[0].iter().all(|point| (point.z - 5.0).abs() < 1e-9));
    }

    #[test]
    fn rejects_bad_inputs_and_hostile_spacing() {
        let mesh = plane_at(5.0);
        let dc = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index");
        let base = params(1.0, 1.0, RasterAngle::AlongX, false);
        let mut cases = Vec::new();
        for stepover in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut bad = base;
            bad.stepover = stepover;
            cases.push(bad);
        }
        for sample_step in [0.0, -0.2, f64::NAN] {
            let mut bad = base;
            bad.sample_step = sample_step;
            cases.push(bad);
        }
        for tolerance in [0.0, -0.01, f64::NAN, f64::NEG_INFINITY] {
            let mut bad = base;
            bad.tolerance = tolerance;
            cases.push(bad);
        }
        let mut floor = base;
        floor.floor_z = f64::NAN;
        cases.push(floor);
        for params in cases {
            let err = parallel_finish(&dc, (0.0, 0.0), (1.0, 1.0), &params).expect_err("params");
            assert!(matches!(err, MeshError::InvalidInput(_)), "{err}");
        }
        assert!(matches!(parallel_finish(&dc, (1.0, 0.0), (0.0, 1.0), &base), Err(MeshError::InvalidInput(_))));
        assert!(matches!(parallel_finish(&dc, (0.0, 2.0), (1.0, 1.0), &base), Err(MeshError::InvalidInput(_))));
        assert!(matches!(parallel_finish(&dc, (f64::NAN, 0.0), (1.0, 1.0), &base), Err(MeshError::InvalidInput(_))));
        let many_lines = params(0.005, 10.0, RasterAngle::AlongX, false);
        assert!(matches!(parallel_finish(&dc, (0.0, 0.0), (1.0, 1_000.0), &many_lines), Err(MeshError::InvalidInput(_))));
        let many_points = params(10.0, 0.001, RasterAngle::AlongX, false);
        assert!(matches!(parallel_finish(&dc, (0.0, 0.0), (2_000.0, 0.0), &many_points), Err(MeshError::InvalidInput(_))));
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

    #[test]
    fn hemisphere_chords_stay_within_tolerance_of_the_drop_cutter() {
        let tolerance = 0.05;
        let floor = -1.0;
        let mesh = dome(10.0, 6, 12);
        let dc = DropCutter::new(&mesh, Cutter::new_ball(2.0).expect("ball")).expect("index");
        let params = RasterParams { stepover: 1.0, sample_step: 1.0, tolerance, angle: RasterAngle::AlongX, floor_z: floor, zigzag: true };
        let lines = parallel_finish(&dc, (-7.0, -7.0), (7.0, 7.0), &params).expect("dome finish");
        assert!(lines.len() >= 2);
        let mut saw_crown = false;
        let mut state = 0x1234_5678_9abc_def0_u64;
        for line in &lines {
            assert!(line.len() >= 2);
            for point in line {
                let z = dc.tip_z_or(point.x, point.y, floor);
                assert!((point.z - z).abs() <= 1e-9, "stored {} true {z}", point.z);
                if point.x.abs() <= 1.0 && point.y.abs() <= 1.0 {
                    saw_crown = saw_crown || point.z > 6.0;
                }
            }
            for window in line.windows(2) {
                let (a, b) = (window[0], window[1]);
                let mid_z = (a.z + b.z) * 0.5;
                let true_mid = dc.tip_z_or((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, floor);
                assert!((mid_z - true_mid).abs() <= tolerance + 1e-9, "mid {mid_z} true {true_mid}");
                for _ in 0..4 {
                    state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                    let t = ((state >> 11) as f64 / (1u64 << 53) as f64).clamp(0.05, 0.95);
                    let x = a.x + (b.x - a.x) * t;
                    let y = a.y + (b.y - a.y) * t;
                    let interpolated = a.z + (b.z - a.z) * t;
                    let true_z = dc.tip_z_or(x, y, floor);
                    assert!(interpolated >= true_z - tolerance - 1e-9, "gouge interp {interpolated} true {true_z} at ({x}, {y})");
                }
            }
        }
        assert!(saw_crown, "raster missed the bump");
        assert_ne!(lines[0][0].x <= lines[0].last().expect("end").x, lines[1][0].x <= lines[1].last().expect("end").x);
    }

    fn mild_triangle(coords: (f64, f64, f64, f64, f64, f64, f64, f64, f64)) -> Option<Triangle> {
        let (ax, ay, az, bx, by, bz, cx, cy, cz) = coords;
        let triangle = Triangle { a: Vec3::new(ax, ay, az), b: Vec3::new(bx, by, bz), c: Vec3::new(cx, cy, cz) };
        let normal = triangle.normal()?;
        let up = if normal.z < 0.0 { normal.scale(-1.0) } else { normal };
        if up.z < 0.25 {
            return None;
        }
        for (p, q) in [(triangle.a, triangle.b), (triangle.b, triangle.c), (triangle.c, triangle.a)] {
            let horiz = (p.x - q.x).hypot(p.y - q.y);
            if horiz < 1e-3 || (p.z - q.z).abs() > 20.0 * horiz {
                return None;
            }
        }
        Some(triangle)
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(24))]

        #[test]
        fn random_meshes_match_the_drop_cutter_within_tolerance(
            raw in proptest::collection::vec(
                (-1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10, -1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10, -1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10),
                1..4usize,
            ),
            radius in 0.5f64..1.5,
            kind in 0..3u8,
            corner_frac in 0.25f64..0.75,
            stepover in 0.6f64..1.2,
            sample_step in 0.6f64..1.2,
            // Flat cutters step from the floor to the part. Half of that step stays inside
            // `tolerance` when the part is at most 0.10 above the floor and tolerance is >= 0.06.
            tolerance in 0.06f64..0.15,
            along_y in any::<bool>(),
            zigzag in any::<bool>(),
        ) {
            let triangles: Vec<Triangle> = raw.iter().copied().filter_map(mild_triangle).collect();
            prop_assume!(!triangles.is_empty());
            let mesh = Mesh::new(triangles).expect("mesh");
            prop_assume!(!mesh.triangles().is_empty());
            let floor = 0.0;
            let cutter = match kind {
                0 => Cutter::new_flat(radius).expect("flat"),
                1 => Cutter::new_ball(radius).expect("ball"),
                _ => Cutter::new_bull(radius, radius * corner_frac).expect("bull"),
            };
            let dc = DropCutter::new(&mesh, cutter).expect("index");
            let params = RasterParams {
                stepover,
                sample_step,
                tolerance,
                angle: if along_y { RasterAngle::AlongY } else { RasterAngle::AlongX },
                floor_z: floor,
                zigzag,
            };
            let lines = parallel_finish(&dc, (-1.2, -1.2), (1.2, 1.2), &params).expect("finish");
            prop_assert!(!lines.is_empty());
            for line in &lines {
                prop_assert!(!line.is_empty());
                for point in line {
                    let z = dc.tip_z_or(point.x, point.y, floor);
                    prop_assert!((point.z - z).abs() <= 1e-9, "point {} tip {z}", point.z);
                }
                for window in line.windows(2) {
                    let (a, b) = (window[0], window[1]);
                    let interpolated = (a.z + b.z) * 0.5;
                    let true_z = dc.tip_z_or((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, floor);
                    prop_assert!(
                        (interpolated - true_z).abs() <= tolerance + 1e-9,
                        "mid {interpolated} true {true_z} tol {tolerance}"
                    );
                }
            }
        }
    }
}
