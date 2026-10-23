//! Outside-diameter envelope of a triangle mesh centred on a spindle axis.

use campanion_geom::{Line, Point, Segment};
use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::{MAX_MESH_SAMPLES, Result, TurnError};

/// Collinear points closer than this, in millimetres, collapse to one span.
const COLLINEAR_TOL: f64 = 1e-9;

/// Which mesh coordinate is the spindle axis.
///
/// Profile Z is this coordinate. The other two coordinates are the radial plane.
/// The mesh is assumed to be centred on the axis already.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshAxis {
    /// Spindle along mesh X. Radius is `hypot(y, z)`.
    X,
    /// Spindle along mesh Y. Radius is `hypot(x, z)`.
    Y,
    /// Spindle along mesh Z. Radius is `hypot(x, y)`.
    Z,
}

/// Sample the outer envelope and return line segments from maximum Z toward minimum Z.
pub(crate) fn envelope(mesh: &Mesh, axis: MeshAxis, step: f64) -> Result<Vec<Segment>> {
    let bounds = mesh.bounds().map_err(|err| TurnError::Mesh(err.to_string()))?;
    let zmin = along(bounds.min(), axis);
    let zmax = along(bounds.max(), axis);
    let samples = sample_positions(zmin, zmax, step)?;
    let mut points = Vec::with_capacity(samples.len());
    for z in samples.iter().rev() {
        let Some(radius) = radius_on_plane(mesh, axis, *z, step) else {
            continue;
        };
        let point = Point::new(*z, radius)?;
        if points.last().is_some_and(|prev: &Point| prev.distance(point) <= COLLINEAR_TOL) {
            continue;
        }
        points.push(point);
    }
    if points.len() < 2 {
        return Err(TurnError::NoProfile("mesh does not meet the sample planes".into()));
    }
    let points = collapse_collinear(points);
    segments_from_points(&points)
}

fn sample_positions(zmin: f64, zmax: f64, step: f64) -> Result<Vec<f64>> {
    if !zmin.is_finite() || !zmax.is_finite() || !(zmax > zmin) {
        return Err(TurnError::InvalidInput("mesh axis extent must be finite and non-zero".into()));
    }
    if !step.is_finite() || !(step > 0.0) {
        return Err(TurnError::InvalidInput("step must be finite and > 0".into()));
    }
    let ratio = (zmax - zmin) / step;
    if !ratio.is_finite() || ratio > MAX_MESH_SAMPLES as f64 {
        return Err(TurnError::InvalidInput(format!("step {step} produces more than {MAX_MESH_SAMPLES} samples")));
    }
    let mut samples = Vec::new();
    let mut i = 0u32;
    let step_index_limit = u32::try_from(MAX_MESH_SAMPLES).unwrap_or(u32::MAX);
    loop {
        if samples.len() > MAX_MESH_SAMPLES || i > step_index_limit {
            return Err(TurnError::InvalidInput(format!("step {step} produces more than {MAX_MESH_SAMPLES} samples")));
        }
        let z = zmin + f64::from(i) * step;
        if !z.is_finite() {
            return Err(TurnError::InvalidInput("sample position is not finite".into()));
        }
        // A grid point that lands on the end is replaced by the exact maximum.
        if z >= zmax || (zmax - z) <= step * 1e-12 {
            break;
        }
        samples.push(z);
        i = i.checked_add(1).ok_or_else(|| TurnError::InvalidInput(format!("step {step} produces more than {MAX_MESH_SAMPLES} samples")))?;
    }
    samples.push(zmax);
    if samples.len() > MAX_MESH_SAMPLES {
        return Err(TurnError::InvalidInput(format!("step {step} produces {} samples, the limit is {MAX_MESH_SAMPLES}", samples.len())));
    }
    if samples.len() < 2 {
        return Err(TurnError::InvalidInput("mesh axis extent is too small to sample".into()));
    }
    Ok(samples)
}

/// Maximum `sqrt(u² + v²)` of edge crossings and of vertices within `step / 2`.
fn radius_on_plane(mesh: &Mesh, axis: MeshAxis, z: f64, step: f64) -> Option<f64> {
    let window = (step * 0.5).max(1e-9);
    let mut best: Option<f64> = None;
    for triangle in mesh.triangles() {
        if !triangle_near_plane(*triangle, axis, z, window) {
            continue;
        }
        for vertex in [triangle.a, triangle.b, triangle.c] {
            let dz = (along(vertex, axis) - z).abs();
            // Vertices on the plane, and vertices within half a step (sharp shoulders).
            if dz <= step * 0.5 || dz <= 1e-9 {
                keep_max(&mut best, radial(vertex, axis));
            }
        }
        let verts = [triangle.a, triangle.b, triangle.c];
        for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
            let (Some(p0), Some(p1)) = (verts.get(i).copied(), verts.get(j).copied()) else {
                continue;
            };
            edge_crossing(p0, p1, z, axis, &mut best);
        }
    }
    best
}

fn triangle_near_plane(triangle: Triangle, axis: MeshAxis, z: f64, window: f64) -> bool {
    let z0 = along(triangle.a, axis);
    let z1 = along(triangle.b, axis);
    let z2 = along(triangle.c, axis);
    let low = z0.min(z1).min(z2);
    let high = z0.max(z1).max(z2);
    z >= low - window && z <= high + window
}

fn edge_crossing(p0: Vec3, p1: Vec3, z: f64, axis: MeshAxis, best: &mut Option<f64>) {
    let z0 = along(p0, axis);
    let z1 = along(p1, axis);
    let dz = z1 - z0;
    // An edge lying in the plane has its maximum radius at a vertex (the radial
    // distance is convex), and those vertices are collected separately.
    if !(dz.abs() > 1e-15) || (z0 - z) * (z1 - z) >= 0.0 {
        return;
    }
    let t = (z - z0) / dz;
    if !t.is_finite() || !(0.0..=1.0).contains(&t) {
        return;
    }
    let point = Vec3::new(p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t, p0.z + (p1.z - p0.z) * t);
    keep_max(best, radial(point, axis));
}

fn keep_max(best: &mut Option<f64>, radius: f64) {
    if !radius.is_finite() || radius < 0.0 {
        return;
    }
    *best = Some(best.map_or(radius, |current| current.max(radius)));
}

fn along(point: Vec3, axis: MeshAxis) -> f64 {
    match axis {
        MeshAxis::X => point.x,
        MeshAxis::Y => point.y,
        MeshAxis::Z => point.z,
    }
}

fn radial(point: Vec3, axis: MeshAxis) -> f64 {
    match axis {
        MeshAxis::X => point.y.hypot(point.z),
        MeshAxis::Y => point.x.hypot(point.z),
        MeshAxis::Z => point.x.hypot(point.y),
    }
}

fn collapse_collinear(points: Vec<Point>) -> Vec<Point> {
    if points.len() <= 2 {
        return points;
    }
    let mut kept = Vec::new();
    let Some(first) = points.first().copied() else {
        return points;
    };
    kept.push(first);
    let last_index = points.len() - 1;
    for index in 1..last_index {
        let (Some(point), Some(next)) = (points.get(index).copied(), points.get(index + 1).copied()) else {
            break;
        };
        let Some(prev) = kept.last().copied() else {
            break;
        };
        if point.distance(prev) <= COLLINEAR_TOL {
            continue;
        }
        if deviation(prev, next, point) <= COLLINEAR_TOL {
            continue;
        }
        kept.push(point);
    }
    if let Some(last) = points.last().copied()
        && kept.last().is_none_or(|prev| prev.distance(last) > COLLINEAR_TOL)
    {
        kept.push(last);
    }
    kept
}

/// Perpendicular distance from `p` to the infinite line through `a` and `b`.
fn deviation(a: Point, b: Point, p: Point) -> f64 {
    let abx = b.x() - a.x();
    let aby = b.y() - a.y();
    let len = abx.hypot(aby);
    if !(len > 1e-15) {
        return p.distance(a);
    }
    let cross = abx * (p.y() - a.y()) - aby * (p.x() - a.x());
    cross.abs() / len
}

fn segments_from_points(points: &[Point]) -> Result<Vec<Segment>> {
    if points.len() < 2 {
        return Err(TurnError::NoProfile("mesh envelope needs at least two points".into()));
    }
    let mut segments = Vec::new();
    for pair in points.windows(2) {
        let (Some(start), Some(end)) = (pair.first().copied(), pair.get(1).copied()) else {
            continue;
        };
        if !(start.distance(end) > 0.0) {
            continue;
        }
        segments.push(Segment::Line(Line::new(start, end)?));
    }
    if segments.is_empty() { Err(TurnError::NoProfile("mesh envelope needs at least two points".into())) } else { Ok(segments) }
}
