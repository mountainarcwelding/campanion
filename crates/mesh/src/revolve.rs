//! Signed distance to a solid of revolution.
//!
//! The section is a polyline in `(Z, radius)`. The solid is that section revolved about the
//! spindle axis. A point's distance is the 2D signed distance of its `(radius, Z)` — the closest
//! point on a solid of revolution shares the meridional plane, because any angular separation
//! only adds length. Negative is inside, zero is on the boundary.
//!
//! A sphere is that point distance at the centre minus the radius. That is exact while the
//! centre is outside the solid. Inside, it is a lower bound (the medial axis can stop the
//! decrease), and it stays negative wherever the sphere overlaps the solid.
//!
//! A finite cylinder whose axis is parallel to Z has an axis-aligned rectangular footprint in
//! `(radius, Z)`. One whose axis is parallel to X has a band `radius ∈ [r_min(Z), r_max(Z)]`
//! between the flat caps. The band distance is the minimum signed distance over that region,
//! bounded by rectangles and refined until the bounds agree within 1e-9 mm. Both are flat-capped:
//! growing the diameter does not extend the cylinder along its axis.

use crate::Vec3;
use crate::collide::{CollisionHit, Cylinder, Sphere};

/// Agreement required between the radial-band bounds, in millimetres.
const BAND_TOL: f64 = 1e-9;

/// A boundary sample this close to an edge is on the boundary.
const BOUNDARY: f64 = 1e-12;

/// Strict interior tests stay this far from a shared face so exact contact stays a touch.
const INTERIOR: f64 = 1e-9;

/// Most band subdivisions in one cylinder query.
const MAX_BAND_STEPS: u32 = 8_192;

/// Solid of revolution of a half-section about Z.
#[derive(Clone, Debug)]
pub struct RevolvedSolid {
    /// Closed polygon in `(Z, radius)`, without a repeated closing vertex.
    points: Vec<(f64, f64)>,
}

impl RevolvedSolid {
    /// Revolve `section` and close it to the spindle axis.
    ///
    /// `section` points are `(Z, radius)`, ordered along the half-profile. The closing edges run
    /// from the last point to the axis, along the axis to the first point's Z, and back out to
    /// the first point. `None` when a coordinate is not finite, a radius is negative, or the
    /// closed polygon has fewer than three vertices.
    pub fn try_from_section(section: &[(f64, f64)]) -> Option<Self> {
        if section.len() < 2 {
            return None;
        }
        let mut points = Vec::with_capacity(section.len().saturating_add(2));
        for point in section {
            if !point.0.is_finite() || !point.1.is_finite() || point.1 < -1e-9 {
                return None;
            }
            push_dedup(&mut points, (point.0, point.1.max(0.0)));
        }
        let first = *points.first()?;
        let last = *points.last()?;
        push_dedup(&mut points, (last.0, 0.0));
        push_dedup(&mut points, (first.0, 0.0));
        if points.len() < 3 {
            return None;
        }
        Some(Self { points })
    }

    /// Use `polygon` as the meridional section. The polygon is already closed in `(Z, radius)`.
    ///
    /// A repeated closing vertex is dropped. `None` for the same reasons as [`Self::try_from_section`].
    pub fn try_from_polygon(polygon: &[(f64, f64)]) -> Option<Self> {
        if polygon.len() < 3 {
            return None;
        }
        let mut points = Vec::with_capacity(polygon.len());
        for point in polygon {
            if !point.0.is_finite() || !point.1.is_finite() {
                return None;
            }
            push_dedup(&mut points, *point);
        }
        if points.len() >= 2 && same_point(points[0], points[points.len() - 1]) {
            points.pop();
        }
        if points.len() < 3 { None } else { Some(Self { points }) }
    }

    /// Signed distance from `point` to the solid. `None` when `point` is not finite.
    pub fn signed_distance(&self, point: Vec3) -> Option<f64> {
        if !finite_vec(point) {
            return None;
        }
        Some(self.distance_zr(point.z, point.x.hypot(point.y)).0)
    }

    /// Signed distance from the sphere, and a point on the section boundary lifted into the
    /// centre's meridional plane.
    pub fn query_sphere(&self, sphere: Sphere) -> Option<CollisionHit> {
        if !finite_vec(sphere.center) || !sphere.radius.is_finite() || sphere.radius < 0.0 {
            return None;
        }
        let (radial, witness) = self.distance_zr(sphere.center.z, sphere.center.x.hypot(sphere.center.y));
        Some(CollisionHit { point: lift(sphere.center.x, sphere.center.y, witness.0, witness.1), distance: radial - sphere.radius })
    }

    /// Signed distance from a flat-capped cylinder whose axis is parallel to Z or to X.
    ///
    /// `None` when the cylinder is not finite or its axis is neither parallel to Z nor to X.
    /// The witness lies on the section boundary, lifted into the meridional plane of the axis.
    pub fn query_cylinder(&self, cylinder: Cylinder) -> Option<CollisionHit> {
        let dir = axis_dir(cylinder)?;
        if dir.z.abs() >= 1.0 - 1e-8 {
            return self.query_axial(cylinder);
        }
        if dir.z.abs() <= 1e-8 && dir.x.abs() >= 1.0 - 1e-8 {
            return self.query_radial(cylinder);
        }
        None
    }

    fn query_axial(&self, cylinder: Cylinder) -> Option<CollisionHit> {
        let cx = (cylinder.a.x + cylinder.b.x) * 0.5;
        let cy = (cylinder.a.y + cylinder.b.y) * 0.5;
        let radial = cx.hypot(cy);
        let r0 = (radial - cylinder.radius).max(0.0);
        let r1 = radial + cylinder.radius;
        let z0 = cylinder.a.z.min(cylinder.b.z);
        let z1 = cylinder.a.z.max(cylinder.b.z);
        let (distance, witness) = aabb_polygon_distance(z0, z1, r0, r1, &self.points)?;
        Some(CollisionHit { point: lift(cx, cy, witness.0, witness.1), distance })
    }

    fn query_radial(&self, cylinder: Cylinder) -> Option<CollisionHit> {
        let band = RadialBand::from_cylinder(cylinder)?;
        let (distance, witness) = self.band_distance(&band)?;
        let anchor_x = (band.x0 + band.x1) * 0.5;
        Some(CollisionHit { point: lift(anchor_x, band.yc, witness.0, witness.1), distance })
    }

    fn distance_zr(&self, z: f64, radius: f64) -> (f64, (f64, f64)) {
        signed_distance_2d(&self.points, z, radius)
    }

    fn band_distance(&self, band: &RadialBand) -> Option<(f64, (f64, f64))> {
        let z0 = band.zc - band.radius;
        let z1 = band.zc + band.radius;
        if !(z0.is_finite() && z1.is_finite()) {
            return None;
        }
        if band.radius <= 1e-15 {
            let (r0, r1) = band.span_at_rho(0.0);
            return aabb_polygon_distance(band.zc, band.zc, r0, r1, &self.points);
        }
        let mut best_hi = f64::INFINITY;
        let mut best_lo = f64::INFINITY;
        let mut best_w = (band.zc, 0.0);
        let samples = 16i32;
        for index in 0..=samples {
            let t = f64::from(index) / f64::from(samples);
            let z = z0 + (z1 - z0) * t;
            let (rmin, rmax) = band.span_at_z(z);
            for radius in [rmin, rmax, (rmin + rmax) * 0.5] {
                let (signed, witness) = signed_distance_2d(&self.points, z, radius);
                if signed < best_hi {
                    best_hi = signed;
                    best_w = witness;
                }
            }
        }
        let mut stack = vec![(z0, z1)];
        let mut guard = 0u32;
        let mut exhausted = false;
        while let Some((za, zb)) = stack.pop() {
            guard += 1;
            if guard > MAX_BAND_STEPS {
                exhausted = true;
                break;
            }
            let (rho_lo, rho_hi) = band.rho_bounds(za, zb);
            let (rmin_hi_rho, rmax_hi_rho) = band.span_at_rho(rho_hi);
            let (rmin_lo_rho, rmax_lo_rho) = band.span_at_rho(rho_lo);
            let rmin_lo = rmin_hi_rho.min(rmin_lo_rho);
            let rmin_hi = rmin_hi_rho.max(rmin_lo_rho);
            let rmax_hi = rmax_hi_rho.max(rmax_lo_rho);
            let variation = (rmin_hi - rmin_lo).max(rmax_hi - rmax_lo_rho.min(rmax_hi_rho)).max(0.0);
            let (signed, witness) = aabb_polygon_distance(za, zb, rmin_lo, rmax_hi, &self.points)?;
            best_lo = best_lo.min(signed);
            if signed > best_hi + 1e-12 {
                continue;
            }
            if signed + variation < best_hi {
                best_hi = signed + variation;
                best_w = witness;
            }
            if variation > BAND_TOL && zb - za > BAND_TOL {
                let mid = (za + zb) * 0.5;
                stack.push((za, mid));
                stack.push((mid, zb));
            }
        }
        let distance = if exhausted { best_lo } else { best_hi };
        if distance.is_finite() { Some((distance, best_w)) } else { None }
    }
}

/// Signed distance between an axis-aligned rectangle and a polygon in `(Z, radius)`.
///
/// Negative when the interiors overlap, zero on contact. The witness is on the polygon.
/// `None` when the rectangle or the polygon is not usable.
pub fn aabb_polygon_distance(z0: f64, z1: f64, r0: f64, r1: f64, polygon: &[(f64, f64)]) -> Option<(f64, (f64, f64))> {
    if polygon.len() < 3 || !z0.is_finite() || !z1.is_finite() || !r0.is_finite() || !r1.is_finite() || z1 < z0 || r1 < r0 {
        return None;
    }
    let mut gap = f64::INFINITY;
    let mut witness = polygon[0];
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        let (dist, on_edge) = segment_aabb_distance(a, b, z0, z1, r0, r1);
        if dist < gap {
            gap = dist;
            witness = on_edge;
        }
    }
    let mut penetration = 0.0;
    let centre = ((z0 + z1) * 0.5, (r0 + r1) * 0.5);
    let probes = [centre, (z0, r0), (z1, r0), (z1, r1), (z0, r1)];
    for probe in probes {
        let (signed, boundary) = signed_distance_2d(polygon, probe.0, probe.1);
        if signed < penetration {
            penetration = signed;
            witness = boundary;
        }
    }
    let mut boundary_inside = false;
    for point in polygon {
        if strictly_inside_aabb(point.0, point.1, z0, z1, r0, r1) {
            boundary_inside = true;
            witness = *point;
        }
    }
    let mut crosses = false;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if segment_through_interior(a, b, z0, z1, r0, r1) {
            crosses = true;
            break;
        }
    }
    if penetration < -INTERIOR {
        return Some((penetration, witness));
    }
    if boundary_inside || crosses {
        return Some((-1e-8, witness));
    }
    if gap <= BOUNDARY {
        gap = 0.0;
    }
    Some((gap, witness))
}

/// Signed distance between a convex polygon and a general polygon in `(Z, radius)`.
///
/// Negative when the interiors overlap. Separated pairs use the minimum edge-to-edge distance.
/// The witness is on `polygon`.
pub fn convex_polygon_distance(convex: &[(f64, f64)], polygon: &[(f64, f64)]) -> Option<(f64, (f64, f64))> {
    if convex.len() < 3 || polygon.len() < 3 {
        return None;
    }
    let mut gap = f64::INFINITY;
    let mut witness = polygon[0];
    for i in 0..convex.len() {
        let a = convex[i];
        let b = convex[(i + 1) % convex.len()];
        for j in 0..polygon.len() {
            let c = polygon[j];
            let d = polygon[(j + 1) % polygon.len()];
            let (dist, on_polygon) = seg_seg_point_on_first(c, d, a, b);
            if dist < gap {
                gap = dist;
                witness = on_polygon;
            }
        }
    }
    let mut penetration = 0.0;
    for point in convex {
        let (signed, boundary) = signed_distance_2d(polygon, point.0, point.1);
        if signed < penetration {
            penetration = signed;
            witness = boundary;
        }
    }
    for point in polygon {
        let signed = signed_distance_convex(convex, point.0, point.1);
        if signed < penetration {
            penetration = signed;
            witness = *point;
        }
    }
    if penetration < -INTERIOR {
        return Some((penetration, witness));
    }
    for i in 0..convex.len() {
        let a = convex[i];
        let b = convex[(i + 1) % convex.len()];
        for j in 0..polygon.len() {
            let c = polygon[j];
            let d = polygon[(j + 1) % polygon.len()];
            if proper_cross(a, b, c, d) {
                return Some((-1e-8, witness));
            }
        }
    }
    if gap <= BOUNDARY {
        gap = 0.0;
    }
    Some((gap, witness))
}

struct RadialBand {
    x0: f64,
    x1: f64,
    yc: f64,
    zc: f64,
    radius: f64,
}

impl RadialBand {
    fn from_cylinder(cylinder: Cylinder) -> Option<Self> {
        let y = (cylinder.a.y + cylinder.b.y) * 0.5;
        let z = (cylinder.a.z + cylinder.b.z) * 0.5;
        if !y.is_finite() || !z.is_finite() {
            return None;
        }
        Some(Self { x0: cylinder.a.x.min(cylinder.b.x), x1: cylinder.a.x.max(cylinder.b.x), yc: y, zc: z, radius: cylinder.radius })
    }

    fn rho_at(&self, z: f64) -> f64 {
        let dz = z - self.zc;
        let value = self.radius * self.radius - dz * dz;
        if value <= 0.0 { 0.0 } else { value.sqrt() }
    }

    fn rho_bounds(&self, za: f64, zb: f64) -> (f64, f64) {
        let mid = self.zc.clamp(za, zb);
        let max = self.rho_at(mid);
        let min = self.rho_at(za).min(self.rho_at(zb));
        (min, max)
    }

    fn span_at_z(&self, z: f64) -> (f64, f64) {
        self.span_at_rho(self.rho_at(z))
    }

    fn span_at_rho(&self, rho: f64) -> (f64, f64) {
        radius_span(self.x0, self.x1, self.yc - rho, self.yc + rho)
    }
}

fn radius_span(x0: f64, x1: f64, y0: f64, y1: f64) -> (f64, f64) {
    let inside = x0 <= 0.0 && 0.0 <= x1 && y0 <= 0.0 && 0.0 <= y1;
    let cx = if 0.0 < x0 {
        x0
    } else if 0.0 > x1 {
        x1
    } else {
        0.0
    };
    let cy = if 0.0 < y0 {
        y0
    } else if 0.0 > y1 {
        y1
    } else {
        0.0
    };
    let rmin = if inside { 0.0 } else { cx.hypot(cy) };
    let rmax = x0.hypot(y0).max(x0.hypot(y1)).max(x1.hypot(y0)).max(x1.hypot(y1));
    (rmin, rmax)
}

fn signed_distance_2d(polygon: &[(f64, f64)], z: f64, radius: f64) -> (f64, (f64, f64)) {
    // Edges on the spindle axis revolve to nothing. The centreline is inside the solid, so the
    // distance is taken to the profile and the end caps only.
    let mut best = f64::INFINITY;
    let mut witness = polygon.first().copied().unwrap_or((z, radius));
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if !edge_is_surface(a, b) {
            continue;
        }
        let (dist, point) = point_segment((z, radius), a, b);
        if dist < best {
            best = dist;
            witness = point;
        }
    }
    if !best.is_finite() {
        return (f64::INFINITY, witness);
    }
    if best <= BOUNDARY {
        return (0.0, witness);
    }
    if in_section(polygon, z, radius) { (-best, witness) } else { (best, witness) }
}

/// An edge with both ends on the axis does not generate a revolved surface.
fn edge_is_surface(a: (f64, f64), b: (f64, f64)) -> bool {
    a.1.abs() > 1e-9 || b.1.abs() > 1e-9
}

fn in_section(polygon: &[(f64, f64)], z: f64, radius: f64) -> bool {
    if point_inside(polygon, z, radius) {
        return true;
    }
    // The open axis segment is the centreline, inside the solid. Its endpoints lie on the end caps.
    if radius > 1e-8 {
        return false;
    }
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if edge_is_surface(a, b) {
            continue;
        }
        let z0 = a.0.min(b.0);
        let z1 = a.0.max(b.0);
        if z > z0 + 1e-9 && z < z1 - 1e-9 {
            return true;
        }
    }
    false
}

fn signed_distance_convex(convex: &[(f64, f64)], z: f64, radius: f64) -> f64 {
    let mut best = f64::INFINITY;
    for index in 0..convex.len() {
        let a = convex[index];
        let b = convex[(index + 1) % convex.len()];
        best = best.min(point_segment((z, radius), a, b).0);
    }
    if !best.is_finite() || best <= BOUNDARY {
        return 0.0;
    }
    if point_in_convex(convex, z, radius) { -best } else { best }
}

fn point_inside(polygon: &[(f64, f64)], z: f64, radius: f64) -> bool {
    let mut crossings = 0i32;
    for index in 0..polygon.len() {
        let (z1, r1) = polygon[index];
        let (z2, r2) = polygon[(index + 1) % polygon.len()];
        if (r1 > radius) == (r2 > radius) {
            continue;
        }
        let denom = r2 - r1;
        if denom.abs() <= 1e-18 {
            continue;
        }
        let z_cross = z1 + (z2 - z1) * (radius - r1) / denom;
        if z_cross > z {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

fn point_in_convex(convex: &[(f64, f64)], z: f64, radius: f64) -> bool {
    if convex.len() < 3 {
        return false;
    }
    let mut sign = 0.0;
    for index in 0..convex.len() {
        let a = convex[index];
        let b = convex[(index + 1) % convex.len()];
        let cross = (b.0 - a.0) * (radius - a.1) - (b.1 - a.1) * (z - a.0);
        if cross.abs() <= 1e-9 {
            continue;
        }
        let side = cross.signum();
        if sign == 0.0 {
            sign = side;
        } else if side != sign {
            return false;
        }
    }
    true
}

fn point_segment(point: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, (f64, f64)) {
    let dz = b.0 - a.0;
    let dr = b.1 - a.1;
    let len2 = dz * dz + dr * dr;
    if len2 <= 1e-24 {
        return ((point.0 - a.0).hypot(point.1 - a.1), a);
    }
    let t = ((point.0 - a.0) * dz + (point.1 - a.1) * dr) / len2;
    let t = t.clamp(0.0, 1.0);
    let q = (a.0 + dz * t, a.1 + dr * t);
    ((point.0 - q.0).hypot(point.1 - q.1), q)
}

fn segment_aabb_distance(a: (f64, f64), b: (f64, f64), z0: f64, z1: f64, r0: f64, r1: f64) -> (f64, (f64, f64)) {
    if let Some((enter, _)) = clip_segment(a, b, z0, z1, r0, r1) {
        return (0.0, enter);
    }
    let corners = [(z0, r0), (z1, r0), (z1, r1), (z0, r1)];
    let mut best = f64::INFINITY;
    let mut witness = a;
    for index in 0..4 {
        let c = corners[index];
        let d = corners[(index + 1) % 4];
        let (dist, on_first) = seg_seg_point_on_first(a, b, c, d);
        if dist < best {
            best = dist;
            witness = on_first;
        }
    }
    (best, witness)
}

fn clip_segment(a: (f64, f64), b: (f64, f64), z0: f64, z1: f64, r0: f64, r1: f64) -> Option<((f64, f64), (f64, f64))> {
    let mut t0 = 0.0;
    let mut t1 = 1.0;
    let dz = b.0 - a.0;
    let dr = b.1 - a.1;
    let ps = [-dz, dz, -dr, dr];
    let qs = [a.0 - z0, z1 - a.0, a.1 - r0, r1 - a.1];
    for index in 0..4 {
        let p = ps[index];
        let q = qs[index];
        if p.abs() <= 1e-18 {
            if q < -1e-15 {
                return None;
            }
            continue;
        }
        let t = q / p;
        if p < 0.0 {
            if t > t1 {
                return None;
            }
            if t > t0 {
                t0 = t;
            }
        } else {
            if t < t0 {
                return None;
            }
            if t < t1 {
                t1 = t;
            }
        }
    }
    if t0 > t1 {
        return None;
    }
    Some((lerp(a, b, t0), lerp(a, b, t1)))
}

fn segment_through_interior(a: (f64, f64), b: (f64, f64), z0: f64, z1: f64, r0: f64, r1: f64) -> bool {
    let Some((start, end)) = clip_segment(a, b, z0, z1, r0, r1) else {
        return false;
    };
    let mid = ((start.0 + end.0) * 0.5, (start.1 + end.1) * 0.5);
    strictly_inside_aabb(mid.0, mid.1, z0, z1, r0, r1)
}

fn strictly_inside_aabb(z: f64, radius: f64, z0: f64, z1: f64, r0: f64, r1: f64) -> bool {
    z > z0 + INTERIOR && z < z1 - INTERIOR && radius > r0 + INTERIOR && radius < r1 - INTERIOR
}

/// Distance, and the closest point on segment `a → b`.
fn seg_seg_point_on_first(p1: (f64, f64), q1: (f64, f64), p2: (f64, f64), q2: (f64, f64)) -> (f64, (f64, f64)) {
    let d1 = (q1.0 - p1.0, q1.1 - p1.1);
    let d2 = (q2.0 - p2.0, q2.1 - p2.1);
    let r = (p1.0 - p2.0, p1.1 - p2.1);
    let a = dot2(d1, d1);
    let e = dot2(d2, d2);
    let f = dot2(d2, r);
    let (s, t) = if a <= 1e-24 && e <= 1e-24 {
        (0.0, 0.0)
    } else if a <= 1e-24 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = dot2(d1, r);
        if e <= 1e-24 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = dot2(d1, d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-18 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let on_first = (p1.0 + d1.0 * s, p1.1 + d1.1 * s);
    let on_second = (p2.0 + d2.0 * t, p2.1 + d2.1 * t);
    ((on_first.0 - on_second.0).hypot(on_first.1 - on_second.1), on_first)
}

fn proper_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}

fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

fn dot2(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn lerp(a: (f64, f64), b: (f64, f64), t: f64) -> (f64, f64) {
    (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
}

fn axis_dir(cylinder: Cylinder) -> Option<Vec3> {
    let delta = cylinder.b.sub(cylinder.a);
    let len = delta.length();
    if !len.is_finite() {
        return None;
    }
    if len <= 0.0 {
        return Some(Vec3::new(0.0, 0.0, 1.0));
    }
    Some(delta.scale(1.0 / len))
}

fn lift(x: f64, y: f64, z: f64, radius: f64) -> Vec3 {
    let angle = y.atan2(x);
    let radius = radius.max(0.0);
    Vec3::new(radius * angle.cos(), radius * angle.sin(), z)
}

fn push_dedup(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
    if points.last().is_none_or(|prev| !same_point(*prev, point)) {
        points.push(point);
    }
}

fn same_point(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() <= 1e-12 && (a.1 - b.1).abs() <= 1e-12
}

fn finite_vec(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collide::{Cylinder, Sphere, collides};
    use proptest::prelude::*;

    fn bar() -> RevolvedSolid {
        RevolvedSolid::try_from_section(&[(0.0, 10.0), (-20.0, 10.0)]).expect("bar")
    }

    fn near(actual: f64, expected: f64) {
        assert!((actual - expected).abs() < 1e-6, "{actual} vs {expected}");
    }

    #[test]
    fn point_distance_matches_the_bar() {
        let solid = bar();
        near(solid.signed_distance(Vec3::new(12.0, 0.0, -10.0)).expect("d"), 2.0);
        near(solid.signed_distance(Vec3::new(0.0, 0.0, -10.0)).expect("d"), -10.0);
        near(solid.signed_distance(Vec3::new(0.0, 0.0, 3.0)).expect("d"), 3.0);
        near(solid.signed_distance(Vec3::new(6.0, 8.0, -10.0)).expect("d"), 0.0);
    }

    #[test]
    fn sphere_gap_is_the_centre_distance_minus_the_radius() {
        let solid = bar();
        let sphere = Sphere::new(Vec3::new(12.0, 0.0, -10.0), 1.0).expect("sphere");
        near(solid.query_sphere(sphere).expect("hit").distance, 1.0);
    }

    #[test]
    fn axial_cylinder_inside_the_bar_is_a_hit_and_one_outside_keeps_the_gap() {
        let solid = bar();
        let inside = Cylinder::new(Vec3::new(0.0, 0.0, -5.0), Vec3::new(0.0, 0.0, -4.0), 1.0).expect("cyl");
        assert!(solid.query_cylinder(inside).expect("hit").distance < 0.0);
        let outside = Cylinder::new(Vec3::new(12.0, 0.0, -5.0), Vec3::new(12.0, 0.0, -4.0), 1.0).expect("cyl");
        near(solid.query_cylinder(outside).expect("hit").distance, 1.0);
    }

    #[test]
    fn radial_cylinder_gap_is_the_face_clearance_and_a_grown_radius_crosses_it() {
        let section = [(0.0, 20.0), (-30.0, 20.0)];
        let solid = RevolvedSolid::try_from_section(&section).expect("part");
        let clear = Cylinder::new(Vec3::new(10.0, 0.0, 5.0), Vec3::new(40.0, 0.0, 5.0), 4.0).expect("cyl");
        near(solid.query_cylinder(clear).expect("hit").distance, 1.0);
        let hit = Cylinder::new(Vec3::new(10.0, 0.0, 5.0), Vec3::new(40.0, 0.0, 5.0), 6.0).expect("cyl");
        assert!(solid.query_cylinder(hit).expect("hit").distance < 0.0);
        let pushed = Cylinder::new(Vec3::new(25.0, 0.0, 5.0), Vec3::new(50.0, 0.0, 5.0), 100.0).expect("cyl");
        assert!(solid.query_cylinder(pushed).expect("hit").distance > 1.0);
    }

    #[test]
    fn rectangles_that_only_share_a_face_do_not_count_as_overlap() {
        let chuck = [(0.0, 0.0), (0.0, 30.0), (-20.0, 30.0), (-20.0, 0.0)];
        let holder = [(0.0, 5.0), (0.0, 15.0), (8.0, 15.0), (8.0, 5.0)];
        near(convex_polygon_distance(&holder, &chuck).expect("d").0, 0.0);
        assert!(!collides(0.0, 0.0));
    }

    fn revolve_config() -> ProptestConfig {
        ProptestConfig { cases: 32, max_shrink_iters: 200, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_111D), ..ProptestConfig::default() }
    }

    fn reference_sdf(polygon: &[(f64, f64)], z: f64, radius: f64) -> f64 {
        let mut best = f64::INFINITY;
        for index in 0..polygon.len() {
            let a = polygon[index];
            let b = polygon[(index + 1) % polygon.len()];
            if a.1.abs() <= 1e-9 && b.1.abs() <= 1e-9 {
                continue;
            }
            best = best.min(point_segment((z, radius), a, b).0);
        }
        let mut angle = 0.0;
        for index in 0..polygon.len() {
            let a = polygon[index];
            let b = polygon[(index + 1) % polygon.len()];
            let a1 = (a.1 - radius).atan2(a.0 - z);
            let a2 = (b.1 - radius).atan2(b.0 - z);
            let mut turn = a2 - a1;
            while turn > std::f64::consts::PI {
                turn -= std::f64::consts::TAU;
            }
            while turn < -std::f64::consts::PI {
                turn += std::f64::consts::TAU;
            }
            angle += turn;
        }
        let on_axis = radius <= 1e-8
            && polygon
                .iter()
                .zip(polygon.iter().cycle().skip(1))
                .any(|(a, b)| a.1.abs() <= 1e-9 && b.1.abs() <= 1e-9 && z > a.0.min(b.0) + 1e-9 && z < a.0.max(b.0) - 1e-9);
        if best <= 1e-8 {
            0.0
        } else if angle.abs() > std::f64::consts::PI || on_axis {
            -best
        } else {
            best
        }
    }

    proptest! {
        #![proptest_config(revolve_config())]
        #[test]
        fn revolved_point_distance_matches_a_winding_reference(
            radii in proptest::collection::vec(0.5f64..25.0, 2..5),
            steps in proptest::collection::vec(0.4f64..6.0, 2..5),
            qx in -15.0f64..30.0,
            qy in -15.0f64..15.0,
            qz in -40.0f64..15.0,
        ) {
            let mut section = Vec::new();
            let mut z = 0.0;
            for (radius, step) in radii.iter().zip(steps.iter()) {
                section.push((z, *radius));
                z -= *step;
            }
            let solid = RevolvedSolid::try_from_section(&section).expect("section");
            let point = Vec3::new(qx, qy, qz);
            let got = solid.signed_distance(point).expect("distance");
            let radius = qx.hypot(qy);
            let reference = reference_sdf(&solid.points, qz, radius);
            let boundary = reference.abs() < 1e-3 || got.abs() < 1e-3;
            if !boundary {
                prop_assert!((got - reference).abs() < 1e-6, "got {got} reference {reference}");
            }
            if got > 1e-4 {
                let mut brute = f64::INFINITY;
                for index in 0..solid.points.len() {
                    let a = solid.points[index];
                    let b = solid.points[(index + 1) % solid.points.len()];
                    if a.1.abs() <= 1e-9 && b.1.abs() <= 1e-9 {
                        continue;
                    }
                    for step in 0..24 {
                        let t = f64::from(step) / 23.0;
                        let sample = lerp(a, b, t);
                        for turn in 0..48 {
                            let angle = std::f64::consts::TAU * f64::from(turn) / 48.0;
                            let surface = Vec3::new(sample.1 * angle.cos(), sample.1 * angle.sin(), sample.0);
                            let dist = (surface.x - qx).hypot(surface.y - qy).hypot(surface.z - qz);
                            brute = brute.min(dist);
                        }
                    }
                }
                prop_assert!(got <= brute + 1e-6, "exact {got} exceeded sampled surface {brute}");
            }
        }
    }

    proptest! {
        #![proptest_config(revolve_config())]
        #[test]
        fn radial_holder_grown_toward_the_face_hits_only_past_the_gap(radius in 0.2f64..20.0) {
            let solid = RevolvedSolid::try_from_section(&[(0.0, 20.0), (-30.0, 20.0)]).expect("part");
            let reaching = Cylinder::new(Vec3::new(10.0, 0.0, 5.0), Vec3::new(40.0, 0.0, 5.0), radius).expect("cyl");
            let distance = solid.query_cylinder(reaching).expect("hit").distance;
            if radius < 5.0 - 1e-4 {
                prop_assert!((distance - (5.0 - radius)).abs() < 1e-4, "{distance} radius {radius}");
                prop_assert!(!collides(distance, 0.0));
            }
            if radius > 5.0 + 1e-3 {
                prop_assert!(collides(distance, 0.0), "radius {radius} distance {distance}");
            }
            let clear = Cylinder::new(Vec3::new(25.0, 0.0, 5.0), Vec3::new(50.0, 0.0, 5.0), radius).expect("cyl");
            let gap = solid.query_cylinder(clear).expect("hit").distance;
            prop_assert!(!collides(gap, 0.0), "pushed tool distance {gap} radius {radius}");
        }
    }
}
