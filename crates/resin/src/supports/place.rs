//! Overhang detection and vertical pillar placement.

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};
use crate::supports::params::SupportParams;

/// Most auto-generated plus explicit support points accepted in one run.
pub const MAX_SUPPORT_POINTS: usize = 20_000;

/// A contact where a support tip meets the part.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportPoint {
    pub x: f64,
    pub y: f64,
    pub contact_z: f64,
}

/// An explicit point from a file. `z` is used when present; otherwise the lowest downward hit is taken.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputPoint {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
}

/// A downward-facing triangle whose angle from vertical is at or above the overhang threshold.
pub fn is_overhang(triangle: &Triangle, overhang_angle_deg: f64) -> bool {
    let Some(normal) = triangle.normal() else {
        return false;
    };
    if normal.z >= 0.0 {
        return false;
    }
    let from_vertical = (-normal.z).clamp(0.0, 1.0).asin().to_degrees();
    from_vertical + 1e-9 >= overhang_angle_deg
}

/// Shoot a ray `origin + t * dir` (`dir` need not be unit). Returns the smallest `t >= 0` and the triangle index.
pub fn first_hit(mesh: &Mesh, origin: Vec3, dir: Vec3, skip: Option<usize>) -> Option<(f64, usize)> {
    if dir.length_sq() <= 0.0 {
        return None;
    }
    let mut best: Option<(f64, usize)> = None;
    for (index, triangle) in mesh.triangles().iter().enumerate() {
        if skip == Some(index) {
            continue;
        }
        let Some(t) = ray_triangle(origin, dir, triangle) else {
            continue;
        };
        if t < 1e-9 {
            continue;
        }
        match best {
            Some((t_best, _)) if t >= t_best => {}
            _ => best = Some((t, index)),
        }
    }
    best
}

/// Möller–Trumbore. Returns `t` along `dir` when the hit is in the triangle and `t >= 0`.
fn ray_triangle(origin: Vec3, dir: Vec3, tri: &Triangle) -> Option<f64> {
    let eps = 1e-12;
    let edge1 = tri.b.sub(tri.a);
    let edge2 = tri.c.sub(tri.a);
    let pvec = dir.cross(edge2);
    let det = edge1.dot(pvec);
    if det.abs() < eps {
        return None;
    }
    let inv = 1.0 / det;
    let tvec = origin.sub(tri.a);
    let u = tvec.dot(pvec) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(edge1);
    let v = dir.dot(qvec) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = edge2.dot(qvec) * inv;
    if t.is_finite() && t >= 0.0 { Some(t) } else { None }
}

/// Lowest downward face of `mesh` at `(x, y)` with `z` strictly above `z_min`.
pub fn lowest_hit_above(mesh: &Mesh, x: f64, y: f64, z_min: f64) -> Option<(Vec3, usize)> {
    let origin = Vec3::new(x, y, z_min);
    let dir = Vec3::new(0.0, 0.0, 1.0);
    let (t, index) = first_hit(mesh, origin, dir, None)?;
    let z = z_min + t;
    if !z.is_finite() {
        return None;
    }
    Some((Vec3::new(x, y, z), index))
}

/// The part already occupies the column below `point` when a downward ray hits another triangle.
pub fn is_self_supported(mesh: &Mesh, point: Vec3, source: Option<usize>) -> bool {
    // Start a short way below the surface so the source face and its neighbours are skipped.
    let origin = Vec3::new(point.x, point.y, point.z - 0.05);
    matches!(first_hit(mesh, origin, Vec3::new(0.0, 0.0, -1.0), source), Some((t, _)) if t > 1e-6)
}

fn sample_triangle(triangle: Triangle, spacing: f64, out: &mut Vec<(Vec3, usize)>, index: usize) -> Result<()> {
    let ab = triangle.b.sub(triangle.a).length();
    let bc = triangle.c.sub(triangle.b).length();
    let ca = triangle.a.sub(triangle.c).length();
    let max_edge = ab.max(bc).max(ca);
    if !max_edge.is_finite() {
        return Err(ResinError::invalid("overhang triangle has a non-finite edge"));
    }
    let n = ((max_edge / spacing).ceil() as usize).max(1);
    if n > 10_000 {
        return Err(ResinError::invalid("support spacing is too small for this overhang"));
    }
    for i in 0..=n {
        for j in 0..=(n - i) {
            let k = n - i - j;
            let s = n as f64;
            let p = triangle.a.scale(i as f64 / s).add(triangle.b.scale(j as f64 / s)).add(triangle.c.scale(k as f64 / s));
            out.push((p, index));
        }
    }
    Ok(())
}

/// Auto-generate contact points on overhangs, then add any explicit points, thinned to `spacing`.
pub fn place_supports(mesh: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool, z_min: f64) -> Result<Vec<SupportPoint>> {
    let mut samples: Vec<(Vec3, usize)> = Vec::new();
    if auto {
        for (index, triangle) in mesh.triangles().iter().enumerate() {
            if !is_overhang(triangle, params.overhang_angle_deg) {
                continue;
            }
            sample_triangle(*triangle, params.support_spacing_mm, &mut samples, index)?;
        }
    }
    let mut needed: Vec<SupportPoint> = Vec::new();
    for (point, index) in samples {
        if is_self_supported(mesh, point, Some(index)) {
            continue;
        }
        needed.push(SupportPoint { x: point.x, y: point.y, contact_z: point.z });
    }
    for point in extra {
        if !point.x.is_finite() || !point.y.is_finite() {
            return Err(ResinError::invalid("a support point is not finite"));
        }
        let contact = match point.z {
            Some(z) => {
                if !z.is_finite() {
                    return Err(ResinError::invalid("a support point is not finite"));
                }
                SupportPoint { x: point.x, y: point.y, contact_z: z }
            }
            None => match lowest_hit_above(mesh, point.x, point.y, z_min) {
                Some((hit, _)) => SupportPoint { x: hit.x, y: hit.y, contact_z: hit.z },
                None => return Err(ResinError::invalid(format!("explicit support at ({}, {}) does not hit the part", point.x, point.y))),
            },
        };
        if contact.contact_z <= z_min {
            return Err(ResinError::invalid("explicit support contact is at or below the raft"));
        }
        needed.push(contact);
    }
    if needed.is_empty() && auto {
        return Ok(Vec::new());
    }
    if needed.is_empty() {
        return Err(ResinError::invalid("no support points were placed; give --points or enable auto generation"));
    }
    let thinned = thin_points(&needed, params.support_spacing_mm)?;
    if thinned.len() > MAX_SUPPORT_POINTS {
        return Err(ResinError::invalid(format!("too many supports: {} > {MAX_SUPPORT_POINTS}; increase --support-spacing", thinned.len())));
    }
    Ok(thinned)
}

fn thin_points(points: &[SupportPoint], spacing: f64) -> Result<Vec<SupportPoint>> {
    if !spacing.is_finite() || spacing <= 0.0 {
        return Err(ResinError::invalid("support spacing must be greater than 0"));
    }
    let mut ordered = points.to_vec();
    ordered.sort_by(|a, b| {
        a.contact_z.partial_cmp(&b.contact_z).unwrap_or(std::cmp::Ordering::Equal).then_with(|| {
            a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
        })
    });
    let spacing_sq = spacing * spacing;
    let mut kept: Vec<SupportPoint> = Vec::new();
    for point in ordered {
        let close = kept.iter().any(|other| {
            let dx = point.x - other.x;
            let dy = point.y - other.y;
            dx * dx + dy * dy <= spacing_sq
        });
        if !close {
            kept.push(point);
        }
    }
    Ok(kept)
}

/// Every needed overhang sample is within `spacing` of a placed support, or is self-supported.
pub fn overhangs_are_covered(mesh: &Mesh, params: &SupportParams, points: &[SupportPoint], z_min: f64) -> Result<bool> {
    let _ = z_min;
    let spacing_sq = params.support_spacing_mm * params.support_spacing_mm;
    for (index, triangle) in mesh.triangles().iter().enumerate() {
        if !is_overhang(triangle, params.overhang_angle_deg) {
            continue;
        }
        let mut samples = Vec::new();
        sample_triangle(*triangle, params.support_spacing_mm, &mut samples, index)?;
        for (point, source) in samples {
            if is_self_supported(mesh, point, Some(source)) {
                continue;
            }
            let covered = points.iter().any(|s| {
                let dx = point.x - s.x;
                let dy = point.y - s.y;
                dx * dx + dy * dy <= spacing_sq + 1e-12
            });
            if !covered {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Parse a points file: one `x y` or `x y z` per line, millimetres. Blank lines and `#` comments are ignored.
pub fn parse_points_text(text: &str) -> Result<Vec<InputPoint>> {
    let mut points = Vec::new();
    for (index, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(x) = parts.next() else {
            continue;
        };
        let Some(y) = parts.next() else {
            return Err(ResinError::invalid(format!("support point on line {} needs at least x and y", index + 1)));
        };
        let z = parts.next();
        if parts.next().is_some() {
            return Err(ResinError::invalid(format!("support point on line {} has more than three values", index + 1)));
        }
        let parse = |s: &str, axis: &str| -> Result<f64> {
            s.parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| ResinError::invalid(format!("support point on line {} has a bad {axis} value `{s}`", index + 1)))
        };
        points.push(InputPoint { x: parse(x, "x")?, y: parse(y, "y")?, z: z.map(|s| parse(s, "z")).transpose()? });
        if points.len() > MAX_SUPPORT_POINTS {
            return Err(ResinError::invalid(format!("too many support points: {} > {MAX_SUPPORT_POINTS}", points.len())));
        }
    }
    Ok(points)
}
