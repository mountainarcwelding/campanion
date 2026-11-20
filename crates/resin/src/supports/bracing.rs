//! Cross-bracing between neighbouring pillars.
//!
//! Pillars closer than the user's maximum span are joined into a minimum spanning tree (shortest
//! links first, no loops), so every pillar that has a neighbour in reach is braced with the fewest
//! braces. Each link gets a zig-zag of straight struts between the two shafts, one strut per
//! vertical pitch. A strut that would pass through or over the part is left out.

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};
use crate::supports::grid::VerticalHitGrid;

/// Most struts built in one run.
pub const MAX_STRUTS: usize = 200_000;
/// Sides of a strut's cross-section.
const STRUT_SIDES: usize = 8;

/// User values for cross-bracing. All are required when bracing is on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BracingParams {
    /// Strut diameter, millimetres.
    pub diameter_mm: f64,
    /// Longest XY distance between two pillars that are braced together, millimetres.
    pub max_span_mm: f64,
    /// Vertical rise of one strut, millimetres.
    pub pitch_mm: f64,
}

impl BracingParams {
    pub fn validate(&self) -> Result<()> {
        for (value, name) in [(self.diameter_mm, "brace diameter"), (self.max_span_mm, "brace max span"), (self.pitch_mm, "brace pitch")] {
            if !value.is_finite() || value <= 0.0 {
                return Err(ResinError::invalid(format!("{name} must be greater than 0 mm")));
            }
        }
        Ok(())
    }
}

/// The straight part of one pillar that braces may attach to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shaft {
    pub x: f64,
    pub y: f64,
    /// Top of the foot, millimetres.
    pub z_bottom: f64,
    /// Bottom of the tip cone, millimetres.
    pub z_top: f64,
}

/// One strut, from `a` to `b`, in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Strut {
    pub a: Vec3,
    pub b: Vec3,
}

/// Plan the struts. `part` is the lifted part; `raft_top` is the raft's top surface.
pub fn plan_braces(shafts: &[Shaft], part: &Mesh, raft_top: f64, params: &BracingParams) -> Result<(Vec<Strut>, usize)> {
    params.validate()?;
    let links = spanning_links(shafts, params.max_span_mm)?;
    let grid = VerticalHitGrid::build(part)?;
    let mut struts = Vec::new();
    let mut skipped = 0usize;
    for (i, j) in links {
        let (Some(a), Some(b)) = (shafts.get(i), shafts.get(j)) else {
            continue;
        };
        let z0 = a.z_bottom.max(b.z_bottom);
        let z1 = a.z_top.min(b.z_top);
        if !(z1 - z0 >= params.pitch_mm) {
            continue;
        }
        let rungs = ((z1 - z0) / params.pitch_mm).floor() as usize;
        for k in 0..rungs {
            let za = z0 + k as f64 * params.pitch_mm;
            let zb = za + params.pitch_mm;
            let (from, to) = if k % 2 == 0 { (a, b) } else { (b, a) };
            let strut = Strut { a: Vec3::new(from.x, from.y, za), b: Vec3::new(to.x, to.y, zb) };
            if strut_clear(&strut, part, &grid, raft_top, params.diameter_mm) {
                struts.push(strut);
            } else {
                skipped += 1;
            }
            if struts.len() > MAX_STRUTS {
                return Err(ResinError::invalid(format!("too many braces: more than {MAX_STRUTS}; increase the brace pitch")));
            }
        }
    }
    Ok((struts, skipped))
}

/// A strut is clear when, along its length, the part's lowest face is at least one strut
/// diameter above the strut's centre line.
fn strut_clear(strut: &Strut, part: &Mesh, grid: &VerticalHitGrid, raft_top: f64, diameter: f64) -> bool {
    let length = strut.b.sub(strut.a).length();
    let step = (diameter / 2.0).max(1e-3);
    let samples = ((length / step).ceil() as usize).clamp(2, 4096);
    for s in 0..=samples {
        let t = s as f64 / samples as f64;
        let p = strut.a.add(strut.b.sub(strut.a).scale(t));
        if let Some((hit, _)) = grid.lowest_hit_above(part, p.x, p.y, raft_top)
            && hit.z <= p.z + diameter
        {
            return false;
        }
    }
    true
}

/// Minimum spanning forest over pillars closer than `max_span` in XY.
pub fn spanning_links(shafts: &[Shaft], max_span: f64) -> Result<Vec<(usize, usize)>> {
    if !max_span.is_finite() || max_span <= 0.0 {
        return Err(ResinError::invalid("brace max span must be greater than 0 mm"));
    }
    let mut edges: Vec<(f64, usize, usize)> = Vec::new();
    let inv = 1.0 / max_span;
    let key = |s: &Shaft| ((s.x * inv).floor() as i64, (s.y * inv).floor() as i64);
    let mut cells: std::collections::HashMap<(i64, i64), Vec<usize>> = std::collections::HashMap::new();
    for (index, shaft) in shafts.iter().enumerate() {
        if !(shaft.x.is_finite() && shaft.y.is_finite()) {
            return Err(ResinError::invalid("a pillar position is not finite"));
        }
        cells.entry(key(shaft)).or_default().push(index);
    }
    let span_sq = max_span * max_span;
    for (index, shaft) in shafts.iter().enumerate() {
        let (cx, cy) = key(shaft);
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(list) = cells.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &other in list {
                    if other <= index {
                        continue;
                    }
                    let Some(o) = shafts.get(other) else {
                        continue;
                    };
                    let d2 = (o.x - shaft.x).powi(2) + (o.y - shaft.y).powi(2);
                    if d2 <= span_sq && d2 > 1e-12 {
                        edges.push((d2, index, other));
                    }
                }
            }
        }
        if edges.len() > 50_000_000 {
            return Err(ResinError::invalid("too many brace candidates; reduce the brace max span"));
        }
    }
    edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut parent: Vec<usize> = (0..shafts.len()).collect();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while let Some(&p) = parent.get(i) {
            if p == i {
                break;
            }
            let grand = parent.get(p).copied().unwrap_or(p);
            if let Some(slot) = parent.get_mut(i) {
                *slot = grand;
            }
            i = grand;
        }
        i
    }
    let mut links = Vec::new();
    for (_, a, b) in edges {
        let ra = find(&mut parent, a);
        let rb = find(&mut parent, b);
        if ra != rb {
            if let Some(slot) = parent.get_mut(ra) {
                *slot = rb;
            }
            links.push((a, b));
        }
    }
    Ok(links)
}

/// Closed prism of `diameter` around the segment, with flat end caps.
pub fn strut_solid(strut: &Strut, diameter: f64) -> Result<Vec<Triangle>> {
    let axis = strut.b.sub(strut.a);
    let Some(dir) = axis.normalized() else {
        return Err(ResinError::invalid("a brace has zero length"));
    };
    let helper = if dir.z.abs() < 0.9 { Vec3::new(0.0, 0.0, 1.0) } else { Vec3::new(1.0, 0.0, 0.0) };
    let u = dir.cross(helper).normalized().ok_or_else(|| ResinError::invalid("brace direction is degenerate"))?;
    let v = dir.cross(u);
    let r = diameter / 2.0;
    let ring = |center: Vec3| -> Vec<Vec3> {
        (0..STRUT_SIDES)
            .map(|k| {
                let angle = std::f64::consts::TAU * k as f64 / STRUT_SIDES as f64;
                center.add(u.scale(r * angle.cos())).add(v.scale(r * angle.sin()))
            })
            .collect()
    };
    let lower = ring(strut.a);
    let upper = ring(strut.b);
    let mut triangles = Vec::with_capacity(STRUT_SIDES * 4);
    for k in 0..STRUT_SIDES {
        let n = (k + 1) % STRUT_SIDES;
        let (Some(&b0), Some(&b1), Some(&t0), Some(&t1)) = (lower.get(k), lower.get(n), upper.get(k), upper.get(n)) else {
            return Err(ResinError::invalid("brace ring is short"));
        };
        // u, v, dir is right-handed, so counter-clockwise around dir faces outward with this winding.
        triangles.push(Triangle { a: b0, b: b1, c: t1 });
        triangles.push(Triangle { a: b0, b: t1, c: t0 });
        triangles.push(Triangle { a: strut.a, b: b1, c: b0 });
        triangles.push(Triangle { a: strut.b, b: t0, c: t1 });
    }
    Ok(triangles)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use campanion_mesh::Aabb;

    use super::*;
    use crate::supports::solid::{is_watertight, signed_volume};

    fn shaft(x: f64, y: f64) -> Shaft {
        Shaft { x, y, z_bottom: 1.0, z_top: 9.0 }
    }

    #[test]
    fn spanning_links_join_neighbours_without_loops() {
        let shafts = vec![shaft(0.0, 0.0), shaft(2.0, 0.0), shaft(0.0, 2.0), shaft(2.0, 2.0), shaft(20.0, 20.0)];
        let links = spanning_links(&shafts, 2.5).unwrap();
        // Four close pillars need three links; the far one stays alone.
        assert_eq!(links.len(), 3);
        assert!(links.iter().all(|(a, b)| *a != 4 && *b != 4));
        assert!(spanning_links(&shafts, 0.0).is_err());
    }

    #[test]
    fn strut_is_a_closed_outward_solid() {
        let strut = Strut { a: Vec3::new(0.0, 0.0, 1.0), b: Vec3::new(3.0, 1.0, 3.0) };
        let tris = strut_solid(&strut, 0.4).unwrap();
        let mesh = Mesh::new(tris).unwrap();
        assert!(is_watertight(&mesh).unwrap());
        assert!(signed_volume(&mesh) > 0.0);
    }

    #[test]
    fn braces_zig_zag_between_pillars_and_avoid_the_part() {
        // Part far above the pillars: every rung fits.
        let high = Mesh::new(Aabb::from_corners(Vec3::new(-1.0, -1.0, 20.0), Vec3::new(3.0, 1.0, 22.0)).face_triangles().to_vec()).unwrap();
        let params = BracingParams { diameter_mm: 0.3, max_span_mm: 3.0, pitch_mm: 2.0 };
        let (struts, skipped) = plan_braces(&[shaft(0.0, 0.0), shaft(2.0, 0.0)], &high, 0.5, &params).unwrap();
        assert_eq!(struts.len(), 4);
        assert_eq!(skipped, 0);
        assert!((struts[0].a.x - 0.0).abs() < 1e-9 && (struts[1].a.x - 2.0).abs() < 1e-9);
        // A part body hanging between the pillars down to z = 4 blocks the rungs that would hit it.
        let low = Mesh::new(Aabb::from_corners(Vec3::new(0.5, -0.5, 4.0), Vec3::new(1.5, 0.5, 22.0)).face_triangles().to_vec()).unwrap();
        let (struts, skipped) = plan_braces(&[shaft(0.0, 0.0), shaft(2.0, 0.0)], &low, 0.5, &params).unwrap();
        assert!(skipped > 0);
        for strut in &struts {
            assert!(strut.a.z.max(strut.b.z) + 0.3 < 4.0 + 1e-9);
        }
        let bad = BracingParams { diameter_mm: 0.0, ..params };
        assert!(plan_braces(&[], &high, 0.5, &bad).is_err());
    }
}
