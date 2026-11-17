//! Chamfered raft outlines: convex footprint, margin, corner relief, and a sloped perimeter.

use crate::error::{ResinError, Result};
use crate::supports::params::SupportParams;

/// Bottom (z = 0) and top (z = thickness) outlines of the raft, both counter-clockwise.
#[derive(Clone, Debug, PartialEq)]
pub struct RaftOutlines {
    pub bottom: Vec<(f64, f64)>,
    pub top: Vec<(f64, f64)>,
    pub chamfer_inset_mm: f64,
}

/// Convex hull of XY points, then an outward margin, optional corner cut-back, and an inward
/// chamfer inset of `thickness / tan(angle)` for the top outline.
pub fn raft_outlines(points: &[(f64, f64)], params: &SupportParams) -> Result<RaftOutlines> {
    let hull = convex_hull(points)?;
    let bottom_full = offset_convex(&hull, params.raft_margin_mm)?;
    let bottom = corner_relief(&bottom_full, params.raft_corner_relief_mm)?;
    let inset = params.chamfer_inset_mm()?;
    let top = offset_convex(&bottom, -inset)?;
    if signed_area(&top) <= 0.0 {
        return Err(ResinError::invalid("raft chamfer inset removes the top face; use a thicker margin, a thinner raft, or a steeper chamfer angle"));
    }
    Ok(RaftOutlines { bottom, top, chamfer_inset_mm: inset })
}

pub fn signed_area(points: &[(f64, f64)]) -> f64 {
    let n = points.len();
    let mut area = 0.0;
    for i in 0..n {
        let Some(&(x1, y1)) = points.get(i) else {
            continue;
        };
        let Some(&(x2, y2)) = points.get((i + 1) % n) else {
            continue;
        };
        area += x1 * y2 - x2 * y1;
    }
    area * 0.5
}

/// Andrew's monotone chain. Returns a counter-clockwise hull without the repeated closing vertex.
pub fn convex_hull(points: &[(f64, f64)]) -> Result<Vec<(f64, f64)>> {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for &(x, y) in points {
        if !x.is_finite() || !y.is_finite() {
            return Err(ResinError::invalid("footprint coordinate is not finite"));
        }
        pts.push((x, y));
    }
    if pts.is_empty() {
        return Err(ResinError::invalid("part has no XY footprint"));
    }
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    pts.dedup();
    if pts.len() < 3 {
        return Err(ResinError::invalid("part XY footprint is degenerate (needs at least three non-collinear points)"));
    }

    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let mut lower = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 {
            let Some(&a) = lower.get(lower.len() - 2) else { break };
            let Some(&b) = lower.last() else { break };
            if cross(a, b, p) > 0.0 {
                break;
            }
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 {
            let Some(&a) = upper.get(upper.len() - 2) else { break };
            let Some(&b) = upper.last() else { break };
            if cross(a, b, p) > 0.0 {
                break;
            }
            upper.pop();
        }
        upper.push(p);
    }
    if !lower.is_empty() {
        lower.pop();
    }
    if !upper.is_empty() {
        upper.pop();
    }
    lower.extend(upper);
    if lower.len() < 3 || signed_area(&lower) <= 0.0 {
        return Err(ResinError::invalid("part XY footprint is degenerate (needs at least three non-collinear points)"));
    }
    Ok(lower)
}

/// Parallel offset of a convex counter-clockwise polygon. Positive `distance` grows the shape.
pub fn offset_convex(points: &[(f64, f64)], distance: f64) -> Result<Vec<(f64, f64)>> {
    if !distance.is_finite() {
        return Err(ResinError::invalid("raft offset is not finite"));
    }
    if points.len() < 3 {
        return Err(ResinError::invalid("cannot offset a degenerate outline"));
    }
    if distance == 0.0 {
        return Ok(points.to_vec());
    }
    let n = points.len();
    let mut normals = Vec::with_capacity(n);
    for i in 0..n {
        let Some(&(x0, y0)) = points.get(i) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let Some(&(x1, y1)) = points.get((i + 1) % n) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let (dx, dy) = (x1 - x0, y1 - y0);
        let len = dx.hypot(dy);
        if !len.is_finite() || len <= 0.0 {
            return Err(ResinError::invalid("raft outline has a zero-length edge"));
        }
        // Outward normal for a CCW ring: (dy, -dx) / len.
        normals.push((dy / len, -dx / len));
    }
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let (Some(&(x, y)), Some(&n0), Some(&n1)) = (points.get(i), normals.get(prev), normals.get(i)) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let dot = n0.0 * n1.0 + n0.1 * n1.1;
        let denom = 1.0 + dot;
        if !denom.is_finite() || denom.abs() < 1e-12 {
            return Err(ResinError::invalid("raft outline has a corner that cannot be offset"));
        }
        let ox = x + (n0.0 + n1.0) * distance / denom;
        let oy = y + (n0.1 + n1.1) * distance / denom;
        if !ox.is_finite() || !oy.is_finite() {
            return Err(ResinError::invalid("raft offset produced a non-finite vertex"));
        }
        out.push((ox, oy));
    }
    if distance < 0.0 && signed_area(&out) <= 0.0 {
        return Err(ResinError::invalid("raft chamfer inset removes the top face; use a thicker margin, a thinner raft, or a steeper chamfer angle"));
    }
    Ok(out)
}

/// Cut each convex corner back by `relief` millimetres along both edges. `0` leaves the outline unchanged.
pub fn corner_relief(points: &[(f64, f64)], relief: f64) -> Result<Vec<(f64, f64)>> {
    if !relief.is_finite() || relief < 0.0 {
        return Err(ResinError::invalid("raft corner relief must be finite and at least 0"));
    }
    if relief == 0.0 {
        return Ok(points.to_vec());
    }
    let n = points.len();
    if n < 3 {
        return Err(ResinError::invalid("cannot apply corner relief to a degenerate outline"));
    }
    // Each edge is shared by two corners, so the cut on each end must leave a mid-edge remainder.
    for i in 0..n {
        let Some(&(x0, y0)) = points.get(i) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let Some(&(x1, y1)) = points.get((i + 1) % n) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let len = (x1 - x0).hypot(y1 - y0);
        if !len.is_finite() || len <= 2.0 * relief {
            return Err(ResinError::invalid(format!(
                "raft corner relief ({relief} mm) is too large for an edge of {len:.6} mm; use a smaller relief"
            )));
        }
    }
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let prev = if i == 0 { n - 1 } else { i - 1 };
        let next = (i + 1) % n;
        let (Some(&p), Some(&a), Some(&b)) = (points.get(i), points.get(prev), points.get(next)) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let (ax, ay) = (a.0 - p.0, a.1 - p.1);
        let (bx, by) = (b.0 - p.0, b.1 - p.1);
        let la = ax.hypot(ay);
        let lb = bx.hypot(by);
        if la <= 0.0 || lb <= 0.0 {
            return Err(ResinError::invalid("raft outline has a zero-length edge"));
        }
        out.push((p.0 + ax / la * relief, p.1 + ay / la * relief));
        out.push((p.0 + bx / lb * relief, p.1 + by / lb * relief));
    }
    Ok(out)
}

/// Perpendicular distance from each top edge to the corresponding bottom edge. For a uniform inset these are equal.
pub fn edge_insets(bottom: &[(f64, f64)], top: &[(f64, f64)]) -> Result<Vec<f64>> {
    if bottom.len() != top.len() || bottom.len() < 3 {
        return Err(ResinError::invalid("outlines must have the same vertex count"));
    }
    let n = bottom.len();
    let mut insets = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let (Some(&b0), Some(&b1), Some(&t0)) = (bottom.get(i), bottom.get(j), top.get(i)) else {
            return Err(ResinError::invalid("outline is short"));
        };
        let (dx, dy) = (b1.0 - b0.0, b1.1 - b0.1);
        let len = dx.hypot(dy);
        if !len.is_finite() || len <= 0.0 {
            return Err(ResinError::invalid("raft outline has a zero-length edge"));
        }
        // Signed distance of t0 from the bottom edge, using the inward (left) normal of a CCW ring: (-dy, dx).
        let dist = ((t0.0 - b0.0) * (-dy) + (t0.1 - b0.1) * dx) / len;
        insets.push(dist);
    }
    Ok(insets)
}
