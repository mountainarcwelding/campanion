//! Closed triangle solids used for the raft and the support pillars.

use std::collections::HashMap;

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};

/// Circumferential segments used for cylinders and cones. This is mesh resolution, not a process parameter.
pub const RING_SIDES: usize = 24;

const WELD_MM: f64 = 1e-6;

pub fn translate_mesh(mesh: &Mesh, delta: Vec3) -> Result<Mesh> {
    let triangles = mesh.triangles().iter().map(|tri| Triangle { a: tri.a.add(delta), b: tri.b.add(delta), c: tri.c.add(delta) }).collect();
    Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("mesh: {err}")))
}

pub fn combine_meshes(parts: &[&Mesh]) -> Result<Mesh> {
    let mut triangles = Vec::new();
    for part in parts {
        triangles.extend_from_slice(part.triangles());
    }
    Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("mesh: {err}")))
}

/// Unit circle point `i / sides` of the way around, starting at +X, going toward +Y.
pub fn ring_point(center_x: f64, center_y: f64, radius: f64, side: usize, sides: usize) -> Result<(f64, f64)> {
    if sides < 3 {
        return Err(ResinError::invalid("a solid of revolution needs at least 3 sides"));
    }
    let theta = 2.0 * std::f64::consts::PI * (side as f64) / (sides as f64);
    let x = center_x + radius * theta.cos();
    let y = center_y + radius * theta.sin();
    if !x.is_finite() || !y.is_finite() {
        return Err(ResinError::invalid("solid ring point is not finite"));
    }
    Ok((x, y))
}

/// Closed solid of revolution from a (z, radius) profile. `rings` must be strictly increasing in z.
pub fn profile_solid(center_x: f64, center_y: f64, rings: &[(f64, f64)], sides: usize) -> Result<Mesh> {
    if rings.len() < 2 {
        return Err(ResinError::invalid("a support solid needs at least two profile rings"));
    }
    let mut zs = Vec::with_capacity(rings.len());
    let mut rads = Vec::with_capacity(rings.len());
    let mut last_z = f64::NEG_INFINITY;
    for &(z, r) in rings {
        if !z.is_finite() || !r.is_finite() || r < 0.0 {
            return Err(ResinError::invalid("support profile is not finite"));
        }
        if z <= last_z {
            return Err(ResinError::invalid("support profile z values must increase"));
        }
        last_z = z;
        zs.push(z);
        rads.push(r);
    }
    let (Some(&r_first), Some(&r_last)) = (rads.first(), rads.last()) else {
        return Err(ResinError::invalid("support profile is empty"));
    };
    if r_first <= 0.0 && r_last <= 0.0 {
        return Err(ResinError::invalid("support profile is closed to a line"));
    }
    let mut rings_pts: Vec<Vec<Vec3>> = Vec::new();
    for (z, r) in zs.iter().zip(rads.iter()) {
        let mut ring = Vec::with_capacity(sides);
        for side in 0..sides {
            let (x, y) = ring_point(center_x, center_y, *r, side, sides)?;
            ring.push(Vec3::new(x, y, *z));
        }
        rings_pts.push(ring);
    }
    let mut triangles = Vec::new();
    for pair in 0..rings_pts.len() - 1 {
        let (Some(lower), Some(upper)) = (rings_pts.get(pair), rings_pts.get(pair + 1)) else {
            return Err(ResinError::invalid("support profile is short"));
        };
        for side in 0..sides {
            let next = (side + 1) % sides;
            let (Some(&b0), Some(&b1), Some(&t0), Some(&t1)) = (lower.get(side), lower.get(next), upper.get(side), upper.get(next)) else {
                return Err(ResinError::invalid("support ring is short"));
            };
            triangles.push(Triangle { a: b0, b: b1, c: t1 });
            triangles.push(Triangle { a: b0, b: t1, c: t0 });
        }
    }
    let (Some(bottom), Some(top), Some(&z0), Some(&z1)) = (rings_pts.first(), rings_pts.last(), zs.first(), zs.last()) else {
        return Err(ResinError::invalid("support profile is short"));
    };
    if r_first > 0.0 {
        let center = Vec3::new(center_x, center_y, z0);
        for side in 0..sides {
            let next = (side + 1) % sides;
            let (Some(&b0), Some(&b1)) = (bottom.get(side), bottom.get(next)) else {
                return Err(ResinError::invalid("support ring is short"));
            };
            triangles.push(Triangle { a: center, b: b1, c: b0 });
        }
    }
    if r_last > 0.0 {
        let center = Vec3::new(center_x, center_y, z1);
        for side in 0..sides {
            let next = (side + 1) % sides;
            let (Some(&t0), Some(&t1)) = (top.get(side), top.get(next)) else {
                return Err(ResinError::invalid("support ring is short"));
            };
            triangles.push(Triangle { a: center, b: t0, c: t1 });
        }
    }
    Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("support solid: {err}")))
}

/// Loft a closed convex (or at least fan-triangulable) outline from `z0` to `z1`.
///
/// `bottom` and `top` have the same vertex count, both counter-clockwise in XY.
pub fn loft_outlines(bottom: &[(f64, f64)], top: &[(f64, f64)], z0: f64, z1: f64) -> Result<Mesh> {
    if bottom.len() != top.len() || bottom.len() < 3 {
        return Err(ResinError::invalid("raft outlines must have the same vertex count, at least 3"));
    }
    if !(z1 > z0) {
        return Err(ResinError::invalid("raft thickness must be positive"));
    }
    let n = bottom.len();
    let mut triangles = Vec::new();
    let (Some(&b0), Some(&t0)) = (bottom.first(), top.first()) else {
        return Err(ResinError::invalid("raft outlines are empty"));
    };
    for i in 1..n - 1 {
        let (Some(&bi), Some(&bj)) = (bottom.get(i), bottom.get(i + 1)) else {
            return Err(ResinError::invalid("raft bottom outline is short"));
        };
        // Bottom cap, normal -Z: (b0, bj, bi) for a CCW outline.
        triangles.push(Triangle { a: Vec3::new(b0.0, b0.1, z0), b: Vec3::new(bj.0, bj.1, z0), c: Vec3::new(bi.0, bi.1, z0) });
        let (Some(&ti), Some(&tj)) = (top.get(i), top.get(i + 1)) else {
            return Err(ResinError::invalid("raft top outline is short"));
        };
        triangles.push(Triangle { a: Vec3::new(t0.0, t0.1, z1), b: Vec3::new(ti.0, ti.1, z1), c: Vec3::new(tj.0, tj.1, z1) });
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let (Some(&bi), Some(&bj), Some(&ti), Some(&tj)) = (bottom.get(i), bottom.get(j), top.get(i), top.get(j)) else {
            return Err(ResinError::invalid("raft outline is short"));
        };
        let b0 = Vec3::new(bi.0, bi.1, z0);
        let b1 = Vec3::new(bj.0, bj.1, z0);
        let t0v = Vec3::new(ti.0, ti.1, z1);
        let t1 = Vec3::new(tj.0, tj.1, z1);
        triangles.push(Triangle { a: b0, b: b1, c: t1 });
        triangles.push(Triangle { a: b0, b: t1, c: t0v });
    }
    Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("raft: {err}")))
}

/// Every welded edge is used by exactly two triangles. Degenerate (zero-length) edges are ignored.
pub fn is_watertight(mesh: &Mesh) -> Result<bool> {
    let mut vertices: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut ids: Vec<u32> = Vec::new();
    let mut next = 0u32;
    let key = |p: Vec3| -> Result<(i64, i64, i64)> {
        if !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite() {
            return Err(ResinError::invalid("mesh coordinate is not finite"));
        }
        let q = |v: f64| -> Result<i64> {
            let n = (v / WELD_MM).round();
            if !n.is_finite() {
                return Err(ResinError::invalid("mesh coordinate is not finite"));
            }
            #[allow(clippy::cast_possible_truncation)]
            Ok(n as i64)
        };
        Ok((q(p.x)?, q(p.y)?, q(p.z)?))
    };
    for tri in mesh.triangles() {
        for p in [tri.a, tri.b, tri.c] {
            let k = key(p)?;
            if let Some(&id) = vertices.get(&k) {
                ids.push(id);
            } else {
                let id = next;
                next = next.checked_add(1).ok_or_else(|| ResinError::invalid("too many vertices"))?;
                vertices.insert(k, id);
                ids.push(id);
            }
        }
    }
    let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
    let mut i = 0usize;
    while i + 2 < ids.len() {
        let (Some(&a), Some(&b), Some(&c)) = (ids.get(i), ids.get(i + 1), ids.get(i + 2)) else {
            break;
        };
        for (u, v) in [(a, b), (b, c), (c, a)] {
            if u == v {
                continue;
            }
            let edge = if u < v { (u, v) } else { (v, u) };
            let count = counts.entry(edge).or_insert(0);
            *count = count.saturating_add(1);
        }
        i += 3;
    }
    Ok(!counts.is_empty() && counts.values().all(|c| *c == 2))
}

/// Signed volume in cubic millimetres. Positive when triangle winding is outward.
pub fn signed_volume(mesh: &Mesh) -> f64 {
    let mut vol = 0.0;
    for tri in mesh.triangles() {
        vol += tri.a.dot(tri.b.cross(tri.c)) / 6.0;
    }
    vol
}
