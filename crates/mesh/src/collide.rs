//! Tool-assembly collision: finite cylinders and spheres against triangles and boxes.
//!
//! A finite cylinder is the solid of radius `R` about a segment `a → b`, with flat caps.
//! Signed distance is negative inside the solid, zero on the boundary, and positive outside.
//! A body touches an obstacle when that distance is strictly less than the clearance, with
//! [`TOUCH_SLACK`] millimetres of slack so exact contact is allowed.
//!
//! Cylinder versus triangle clips the triangle to the slab between the end planes, then takes
//! the minimum distance from the clipped polygon to the axis segment (including a polygon that
//! crosses the axis). Caps are the two end disks. Sphere versus triangle is the closest point
//! on the triangle minus the radius. Cylinder versus an axis-aligned box uses the twelve face
//! triangles and a containment test.
//!
//! [`CollisionIndex`] is a bounding-volume hierarchy over a mesh so a 100k-triangle part checks
//! quickly. More than [`MAX_COLLISION_TESTS`] triangle tests in one query is an error.

use crate::{Aabb, Mesh, Result, Triangle, Vec3, closest_point_on_triangle, finite_vec, invalid};

/// Contact at exactly the clearance distance is allowed. Collision is `distance + slack < clearance`.
pub const TOUCH_SLACK: f64 = 1e-9;

/// Most triangle tests accepted in one index query.
const MAX_COLLISION_TESTS: usize = 50_000_000;

/// Most vertices after clipping a triangle to the cylinder slab.
const MAX_CLIP_VERTS: usize = 12;

/// Leaf size for the AABB tree.
const BVH_LEAF: usize = 8;

/// Recursion cap while building the tree. A median split of 5e6 triangles is far below this.
const BVH_DEPTH: usize = 48;

/// A finite right circular cylinder. `a` and `b` are the cap centres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cylinder {
    pub a: Vec3,
    pub b: Vec3,
    pub radius: f64,
}

/// A solid sphere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f64,
}

/// Signed distance from a body to an obstacle, with a witness point on the obstacle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionHit {
    pub point: Vec3,
    pub distance: f64,
}

/// True when a signed distance is a collision against `clearance`.
///
/// Exact contact (`distance == clearance`) is allowed. Values within [`TOUCH_SLACK`] millimetres
/// of that boundary are treated as contact.
pub fn collides(distance: f64, clearance: f64) -> bool {
    distance + TOUCH_SLACK < clearance
}

impl Cylinder {
    /// `None` when a coordinate or the radius is not finite, or the radius is negative.
    pub fn new(a: Vec3, b: Vec3, radius: f64) -> Option<Self> {
        if !finite_vec(a) || !finite_vec(b) || !radius.is_finite() || radius < 0.0 {
            return None;
        }
        Some(Self { a, b, radius })
    }

    fn direction(self) -> Option<(Vec3, f64)> {
        let delta = self.b.sub(self.a);
        let len = delta.length();
        if !len.is_finite() {
            return None;
        }
        if len <= 0.0 {
            return Some((Vec3::new(0.0, 0.0, 1.0), 0.0));
        }
        Some((delta.scale(1.0 / len), len))
    }

    fn aabb(self) -> Aabb {
        Aabb::from_corners(self.a, self.b).expand(self.radius)
    }
}

impl Sphere {
    /// `None` when a coordinate or the radius is not finite, or the radius is negative.
    pub fn new(center: Vec3, radius: f64) -> Option<Self> {
        if !finite_vec(center) || !radius.is_finite() || radius < 0.0 {
            return None;
        }
        Some(Self { center, radius })
    }

    fn aabb(self) -> Aabb {
        Aabb::from_corners(self.center, self.center).expand(self.radius)
    }
}

/// Signed distance from the solid cylinder to the triangle, and a point on the triangle.
pub fn cylinder_triangle(cylinder: Cylinder, triangle: Triangle) -> Option<CollisionHit> {
    if cylinder.radius < 0.0 || !cylinder.radius.is_finite() {
        return None;
    }
    let mut best: Option<CollisionHit> = None;
    consider(&mut best, lateral_hit(cylinder, triangle));
    consider(&mut best, vertex_hits(cylinder, triangle));
    consider(&mut best, edge_hits(cylinder, triangle));
    consider(&mut best, cap_hit(cylinder, triangle, true));
    consider(&mut best, cap_hit(cylinder, triangle, false));
    best
}

/// Signed distance from the solid sphere to the triangle, and a point on the triangle.
pub fn sphere_triangle(sphere: Sphere, triangle: Triangle) -> Option<CollisionHit> {
    if sphere.radius < 0.0 || !sphere.radius.is_finite() || !finite_vec(sphere.center) {
        return None;
    }
    let point = closest_point_on_triangle(sphere.center, triangle);
    if !finite_vec(point) {
        return None;
    }
    let distance = point.sub(sphere.center).length() - sphere.radius;
    if !distance.is_finite() {
        return None;
    }
    Some(CollisionHit { point, distance })
}

/// Signed distance from the solid cylinder to the solid axis-aligned box.
pub fn cylinder_aabb(cylinder: Cylinder, bounds: Aabb) -> Option<CollisionHit> {
    let mut best: Option<CollisionHit> = None;
    for triangle in bounds.face_triangles() {
        consider(&mut best, cylinder_triangle(cylinder, triangle));
    }
    if axis_hits_aabb(cylinder.a, cylinder.b, bounds) || bounds.contains(cylinder.a) || bounds.contains(cylinder.b) {
        let inside = point_aabb_signed(segment_mid(cylinder.a, cylinder.b), bounds);
        if let Some(inside) = inside {
            let distance = if inside < 0.0 { (-cylinder.radius).min(inside) } else { -cylinder.radius };
            let point = clamp_to_aabb(cylinder.a, bounds);
            consider(&mut best, Some(CollisionHit { point, distance }));
        }
    }
    best
}

fn consider(best: &mut Option<CollisionHit>, hit: Option<CollisionHit>) {
    let Some(hit) = hit else {
        return;
    };
    if !hit.distance.is_finite() || !finite_vec(hit.point) {
        return;
    }
    match *best {
        Some(current) if current.distance <= hit.distance => {}
        _ => *best = Some(hit),
    }
}

fn segment_mid(a: Vec3, b: Vec3) -> Vec3 {
    a.lerp(b, 0.5)
}

fn clamp_to_aabb(point: Vec3, bounds: Aabb) -> Vec3 {
    Vec3::new(
        point.x.clamp(bounds.min().x, bounds.max().x),
        point.y.clamp(bounds.min().y, bounds.max().y),
        point.z.clamp(bounds.min().z, bounds.max().z),
    )
}

fn point_aabb_signed(point: Vec3, bounds: Aabb) -> Option<f64> {
    if bounds.contains(point) {
        let dx = (point.x - bounds.min().x).min(bounds.max().x - point.x);
        let dy = (point.y - bounds.min().y).min(bounds.max().y - point.y);
        let dz = (point.z - bounds.min().z).min(bounds.max().z - point.z);
        let d = dx.min(dy).min(dz);
        if d.is_finite() { Some(-d) } else { None }
    } else {
        let clamped = clamp_to_aabb(point, bounds);
        let d = point.sub(clamped).length();
        if d.is_finite() { Some(d) } else { None }
    }
}

fn axis_hits_aabb(a: Vec3, b: Vec3, bounds: Aabb) -> bool {
    let mut tmin = 0.0_f64;
    let mut tmax = 1.0_f64;
    for axis in 0..3 {
        let (pa, pb, lo, hi) = match axis {
            0 => (a.x, b.x, bounds.min().x, bounds.max().x),
            1 => (a.y, b.y, bounds.min().y, bounds.max().y),
            _ => (a.z, b.z, bounds.min().z, bounds.max().z),
        };
        let delta = pb - pa;
        if delta.abs() <= 1e-18 {
            if pa < lo || pa > hi {
                return false;
            }
            continue;
        }
        let mut t0 = (lo - pa) / delta;
        let mut t1 = (hi - pa) / delta;
        if t0 > t1 {
            core::mem::swap(&mut t0, &mut t1);
        }
        tmin = tmin.max(t0);
        tmax = tmax.min(t1);
        if tmin > tmax {
            return false;
        }
    }
    true
}

fn vertex_hits(cylinder: Cylinder, triangle: Triangle) -> Option<CollisionHit> {
    let mut best = None;
    for vertex in [triangle.a, triangle.b, triangle.c] {
        let distance = point_to_cylinder(vertex, cylinder)?;
        consider(&mut best, Some(CollisionHit { point: vertex, distance }));
    }
    best
}

fn edge_hits(cylinder: Cylinder, triangle: Triangle) -> Option<CollisionHit> {
    let mut best = None;
    for (p, q) in [(triangle.a, triangle.b), (triangle.b, triangle.c), (triangle.c, triangle.a)] {
        consider(&mut best, segment_to_cylinder(p, q, cylinder));
    }
    best
}

fn lateral_hit(cylinder: Cylinder, triangle: Triangle) -> Option<CollisionHit> {
    let (dir, len) = cylinder.direction()?;
    if len <= 0.0 {
        return None;
    }
    let poly = clip_triangle_to_slab(triangle, cylinder.a, dir, len)?;
    if poly.len() < 2 {
        return None;
    }
    let (dist_axis, point) = polygon_to_axis(&poly, cylinder.a, dir, len)?;
    let distance = dist_axis - cylinder.radius;
    if distance.is_finite() { Some(CollisionHit { point, distance }) } else { None }
}

fn cap_hit(cylinder: Cylinder, triangle: Triangle, at_a: bool) -> Option<CollisionHit> {
    let (dir, len) = cylinder.direction()?;
    let (center, normal) = if at_a {
        (cylinder.a, dir.scale(-1.0))
    } else if len <= 0.0 {
        (cylinder.a, dir)
    } else {
        (cylinder.b, dir)
    };
    triangle_to_disk(triangle, center, normal, cylinder.radius)
}

/// Clip `triangle` to `0 <= (p - origin) · dir <= len`. Result is a convex polygon, at most a hexagon.
fn clip_triangle_to_slab(triangle: Triangle, origin: Vec3, dir: Vec3, len: f64) -> Option<Vec<Vec3>> {
    let mut poly = vec![triangle.a, triangle.b, triangle.c];
    poly = clip_poly(&poly, origin, dir, 0.0, true)?;
    clip_poly(&poly, origin, dir, len, false)
}

fn clip_poly(poly: &[Vec3], origin: Vec3, dir: Vec3, plane: f64, keep_greater: bool) -> Option<Vec<Vec3>> {
    if poly.is_empty() {
        return Some(Vec::new());
    }
    let mut out = Vec::new();
    let Some(first) = poly.first() else {
        return Some(Vec::new());
    };
    let mut prev = *poly.last().unwrap_or(first);
    let mut prev_t = prev.sub(origin).dot(dir);
    let mut prev_in = inside_slab(prev_t, plane, keep_greater);
    for &curr in poly {
        if out.len() >= MAX_CLIP_VERTS {
            break;
        }
        let curr_t = curr.sub(origin).dot(dir);
        let curr_in = inside_slab(curr_t, plane, keep_greater);
        if curr_in != prev_in
            && let Some(hit) = intersect_plane(prev, prev_t, curr, curr_t, plane)
        {
            push_unique(&mut out, hit);
        }
        if curr_in {
            push_unique(&mut out, curr);
        }
        prev = curr;
        prev_t = curr_t;
        prev_in = curr_in;
    }
    Some(out)
}

fn inside_slab(t: f64, plane: f64, keep_greater: bool) -> bool {
    if keep_greater { t >= plane - 1e-14 } else { t <= plane + 1e-14 }
}

fn intersect_plane(p: Vec3, pt: f64, q: Vec3, qt: f64, plane: f64) -> Option<Vec3> {
    let denom = qt - pt;
    if denom.abs() <= 1e-18 {
        return if (pt - plane).abs() <= 1e-14 { Some(p) } else { None };
    }
    let s = (plane - pt) / denom;
    if !s.is_finite() {
        return None;
    }
    let hit = p.lerp(q, s.clamp(0.0, 1.0));
    if finite_vec(hit) { Some(hit) } else { None }
}

fn push_unique(out: &mut Vec<Vec3>, point: Vec3) {
    if let Some(last) = out.last()
        && last.sub(point).length_sq() <= 1e-24
    {
        return;
    }
    if let Some(first) = out.first()
        && first.sub(point).length_sq() <= 1e-24
    {
        return;
    }
    out.push(point);
}

fn polygon_to_axis(poly: &[Vec3], origin: Vec3, dir: Vec3, len: f64) -> Option<(f64, Vec3)> {
    let (u, v) = perp_basis(dir)?;
    let mut pts2 = Vec::with_capacity(poly.len());
    for p in poly {
        let d = p.sub(origin);
        pts2.push((d.dot(u), d.dot(v), *p));
    }
    let area = polygon_area_2d(&pts2);
    if area.abs() > 1e-16 && origin_in_convex_2d(&pts2) {
        let point = axis_plane_hit(poly, origin, dir, len).or_else(|| poly.first().copied())?;
        return Some((0.0, point));
    }
    let mut best_d = f64::INFINITY;
    let mut best_p = *poly.first()?;
    let n = pts2.len();
    for i in 0..n {
        let Some(&(ax, ay, ap)) = pts2.get(i) else {
            continue;
        };
        let Some(&(bx, by, bp)) = pts2.get((i + 1) % n) else {
            continue;
        };
        let (d, t) = dist_origin_to_seg_2d(ax, ay, bx, by);
        if d < best_d {
            best_d = d;
            best_p = ap.lerp(bp, t);
        }
    }
    if best_d.is_finite() { Some((best_d, best_p)) } else { None }
}

fn polygon_area_2d(pts: &[(f64, f64, Vec3)]) -> f64 {
    let mut area = 0.0;
    let n = pts.len();
    if n < 3 {
        return 0.0;
    }
    for i in 0..n {
        let Some(&(ax, ay, _)) = pts.get(i) else {
            continue;
        };
        let Some(&(bx, by, _)) = pts.get((i + 1) % n) else {
            continue;
        };
        area += ax * by - ay * bx;
    }
    0.5 * area
}

fn origin_in_convex_2d(pts: &[(f64, f64, Vec3)]) -> bool {
    let n = pts.len();
    if n < 3 {
        return false;
    }
    let mut sign = 0.0;
    for i in 0..n {
        let Some(&(ax, ay, _)) = pts.get(i) else {
            continue;
        };
        let Some(&(bx, by, _)) = pts.get((i + 1) % n) else {
            continue;
        };
        let ex = bx - ax;
        let ey = by - ay;
        let cross = ax * ey - ay * ex;
        if cross.abs() <= 1e-16 {
            continue;
        }
        let s = cross.signum();
        if sign == 0.0 {
            sign = s;
        } else if s != sign {
            return false;
        }
    }
    sign != 0.0
}

fn dist_origin_to_seg_2d(ax: f64, ay: f64, bx: f64, by: f64) -> (f64, f64) {
    let abx = bx - ax;
    let aby = by - ay;
    let len2 = abx * abx + aby * aby;
    if len2 <= 1e-24 {
        return ((ax * ax + ay * ay).sqrt(), 0.0);
    }
    let t = ((-ax) * abx + (-ay) * aby) / len2;
    let t = t.clamp(0.0, 1.0);
    let px = ax + abx * t;
    let py = ay + aby * t;
    ((px * px + py * py).sqrt(), t)
}

fn axis_plane_hit(poly: &[Vec3], origin: Vec3, dir: Vec3, len: f64) -> Option<Vec3> {
    let &a = poly.first()?;
    let Some(&b) = poly.get(1) else {
        return Some(a);
    };
    let Some(&c) = poly.get(2).or(poly.first()) else {
        return Some(a);
    };
    let n = b.sub(a).cross(c.sub(a));
    let denom = n.dot(dir);
    if denom.abs() <= 1e-16 {
        return closest_on_segment(a, origin, origin.add(dir.scale(len))).map(|(p, _)| p).or(Some(a));
    }
    let t = n.dot(a.sub(origin)) / denom;
    if !t.is_finite() {
        return Some(a);
    }
    let hit = origin.add(dir.scale(t.clamp(0.0, len)));
    if finite_vec(hit) { Some(hit) } else { Some(a) }
}

fn perp_basis(dir: Vec3) -> Option<(Vec3, Vec3)> {
    let helper = if dir.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let u = dir.cross(helper).normalized()?;
    let v = dir.cross(u).normalized()?;
    Some((u, v))
}

fn triangle_to_disk(triangle: Triangle, center: Vec3, normal: Vec3, radius: f64) -> Option<CollisionHit> {
    let n = normal.normalized()?;
    let mut best: Option<CollisionHit> = None;
    for vertex in [triangle.a, triangle.b, triangle.c] {
        consider(&mut best, Some(point_to_disk(vertex, center, n, radius)?));
    }
    for (p, q) in [(triangle.a, triangle.b), (triangle.b, triangle.c), (triangle.c, triangle.a)] {
        consider(&mut best, segment_to_disk(p, q, center, n, radius));
    }
    if let Some(hit) = plane_disk_overlap(triangle, center, n, radius) {
        consider(&mut best, Some(hit));
    }
    best
}

fn point_to_disk(point: Vec3, center: Vec3, normal: Vec3, radius: f64) -> Option<CollisionHit> {
    let rel = point.sub(center);
    let axial = rel.dot(normal);
    let inplane = rel.sub(normal.scale(axial));
    let rho = inplane.length();
    let distance = if rho <= radius {
        axial.abs()
    } else {
        let rim = if rho > 0.0 { inplane.scale(radius / rho) } else { Vec3::new(0.0, 0.0, 0.0) };
        point.sub(center.add(rim)).length()
    };
    if !distance.is_finite() {
        return None;
    }
    Some(CollisionHit { point, distance })
}

fn segment_to_disk(p: Vec3, q: Vec3, center: Vec3, normal: Vec3, radius: f64) -> Option<CollisionHit> {
    let mut best = None;
    consider(&mut best, point_to_disk(p, center, normal, radius));
    consider(&mut best, point_to_disk(q, center, normal, radius));
    let pn = p.sub(center).dot(normal);
    let qn = q.sub(center).dot(normal);
    if pn * qn <= 0.0 {
        let denom = qn - pn;
        if denom.abs() > 1e-18 {
            let t = (-pn) / denom;
            if (0.0..=1.0).contains(&t) {
                let hit = p.lerp(q, t);
                consider(&mut best, point_to_disk(hit, center, normal, radius));
            }
        }
    }
    best
}

fn plane_disk_overlap(triangle: Triangle, center: Vec3, normal: Vec3, radius: f64) -> Option<CollisionHit> {
    let closest = closest_point_on_triangle(center, triangle);
    let rel = closest.sub(center);
    let axial = rel.dot(normal);
    let inplane = rel.sub(normal.scale(axial));
    if inplane.length() <= radius + 1e-12 {
        let distance = axial.abs();
        if distance.is_finite() {
            return Some(CollisionHit { point: closest, distance });
        }
    }
    None
}

/// Signed distance from `point` to the solid cylinder.
fn point_to_cylinder(point: Vec3, cylinder: Cylinder) -> Option<f64> {
    let (dir, len) = cylinder.direction()?;
    let rel = point.sub(cylinder.a);
    let t = rel.dot(dir);
    let radial_vec = rel.sub(dir.scale(t));
    let radial = radial_vec.length();
    let r = cylinder.radius;
    let d = if t < 0.0 {
        if radial <= r { -t } else { (radial - r).hypot(t) }
    } else if t > len {
        let along = t - len;
        if radial <= r { along } else { (radial - r).hypot(along) }
    } else {
        radial - r
    };
    if d.is_finite() { Some(d) } else { None }
}

fn segment_to_cylinder(p: Vec3, q: Vec3, cylinder: Cylinder) -> Option<CollisionHit> {
    let mut best = None;
    if let Some(d) = point_to_cylinder(p, cylinder) {
        consider(&mut best, Some(CollisionHit { point: p, distance: d }));
    }
    if let Some(d) = point_to_cylinder(q, cylinder) {
        consider(&mut best, Some(CollisionHit { point: q, distance: d }));
    }
    let (dir, len) = cylinder.direction()?;
    let pt = p.sub(cylinder.a).dot(dir);
    let qt = q.sub(cylinder.a).dot(dir);
    if let Some(clipped) = clip_segment_to_slab(p, pt, q, qt, 0.0, len) {
        let (dist, pa, _) = segment_segment(clipped.0, clipped.1, cylinder.a, cylinder.b);
        if dist.is_finite() {
            consider(&mut best, Some(CollisionHit { point: pa, distance: dist - cylinder.radius }));
        }
    }
    consider(&mut best, segment_to_disk(p, q, cylinder.a, dir.scale(-1.0), cylinder.radius));
    if len > 0.0 {
        consider(&mut best, segment_to_disk(p, q, cylinder.b, dir, cylinder.radius));
    }
    best
}

fn clip_segment_to_slab(p: Vec3, pt: f64, q: Vec3, qt: f64, lo: f64, hi: f64) -> Option<(Vec3, Vec3)> {
    let mut a = p;
    let mut at = pt;
    let mut b = q;
    let mut bt = qt;
    if at > bt {
        core::mem::swap(&mut a, &mut b);
        core::mem::swap(&mut at, &mut bt);
    }
    if bt < lo - 1e-14 || at > hi + 1e-14 {
        return None;
    }
    if at < lo {
        a = match intersect_plane(a, at, b, bt, lo) {
            Some(hit) => hit,
            None => a,
        };
        at = lo;
    }
    if bt > hi {
        b = match intersect_plane(a, at, b, bt, hi) {
            Some(hit) => hit,
            None => b,
        };
    }
    Some((a, b))
}

fn closest_on_segment(point: Vec3, a: Vec3, b: Vec3) -> Option<(Vec3, f64)> {
    let ab = b.sub(a);
    let len2 = ab.length_sq();
    if !len2.is_finite() {
        return None;
    }
    let t = if len2 <= 1e-24 { 0.0 } else { point.sub(a).dot(ab) / len2 };
    let t = t.clamp(0.0, 1.0);
    let p = a.lerp(b, t);
    if finite_vec(p) { Some((p, t)) } else { None }
}

/// Closest points of two segments. Returns `(distance, point on pq, point on ab)`.
fn segment_segment(p: Vec3, q: Vec3, a: Vec3, b: Vec3) -> (f64, Vec3, Vec3) {
    let u = q.sub(p);
    let v = b.sub(a);
    let w = p.sub(a);
    let uu = u.dot(u);
    let vv = v.dot(v);
    let uv = u.dot(v);
    let uw = u.dot(w);
    let vw = v.dot(w);
    let den = uu * vv - uv * uv;
    let mut s = 0.0;
    let mut t = 0.0;
    if den.abs() > 1e-18 && uu > 1e-24 && vv > 1e-24 {
        s = (uv * vw - vv * uw) / den;
        t = (uu * vw - uv * uw) / den;
    } else if uu > 1e-24 {
        s = -uw / uu;
    } else if vv > 1e-24 {
        t = vw / vv;
    }
    s = s.clamp(0.0, 1.0);
    t = t.clamp(0.0, 1.0);
    if uu > 1e-24 {
        s = (-uw + uv * t) / uu;
        s = s.clamp(0.0, 1.0);
    }
    if vv > 1e-24 {
        t = (vw + uv * s) / vv;
        t = t.clamp(0.0, 1.0);
    }
    if uu > 1e-24 {
        s = (-uw + uv * t) / uu;
        s = s.clamp(0.0, 1.0);
    }
    let pa = p.lerp(q, s);
    let pb = a.lerp(b, t);
    (pa.sub(pb).length(), pa, pb)
}

/// AABB tree over a mesh. Queries test only triangles whose bounds overlap the body.
pub struct CollisionIndex<'a> {
    mesh: &'a Mesh,
    nodes: Vec<BvhNode>,
}

struct BvhNode {
    bounds: Aabb,
    /// Leaf: start index into `tris`. Inner: left child.
    left: usize,
    /// Leaf: triangle count. Inner: right child.
    right: usize,
    leaf: bool,
    tris: Vec<usize>,
}

impl<'a> CollisionIndex<'a> {
    /// Index `mesh`. An empty mesh is allowed and every query misses.
    pub fn new(mesh: &'a Mesh) -> Result<Self> {
        if mesh.triangles().is_empty() {
            return Ok(Self { mesh, nodes: Vec::new() });
        }
        let mut indices: Vec<usize> = (0..mesh.triangles().len()).collect();
        let mut nodes = Vec::new();
        build_bvh(mesh, &mut indices, 0, &mut nodes)?;
        Ok(Self { mesh, nodes })
    }

    pub fn mesh(&self) -> &'a Mesh {
        self.mesh
    }

    /// Closest signed hit of `cylinder` against the mesh that is strictly under `clearance`.
    pub fn query_cylinder(&self, cylinder: Cylinder, clearance: f64) -> Result<Option<CollisionHit>> {
        let Some(bounds) = query_bounds(cylinder.aabb(), clearance) else {
            return Ok(None);
        };
        self.query(bounds, clearance, |triangle| cylinder_triangle(cylinder, triangle))
    }

    /// Closest signed hit of `sphere` against the mesh that is strictly under `clearance`.
    pub fn query_sphere(&self, sphere: Sphere, clearance: f64) -> Result<Option<CollisionHit>> {
        let Some(bounds) = query_bounds(sphere.aabb(), clearance) else {
            return Ok(None);
        };
        self.query(bounds, clearance, |triangle| sphere_triangle(sphere, triangle))
    }

    fn query(&self, bounds: Aabb, clearance: f64, mut test: impl FnMut(Triangle) -> Option<CollisionHit>) -> Result<Option<CollisionHit>> {
        if self.nodes.is_empty() {
            return Ok(None);
        }
        let mut stack = vec![0usize];
        let mut tests = 0usize;
        let mut best: Option<CollisionHit> = None;
        while let Some(index) = stack.pop() {
            let Some(node) = self.nodes.get(index) else {
                continue;
            };
            if !node.bounds.overlaps(bounds) {
                continue;
            }
            if node.leaf {
                for &tri_index in &node.tris {
                    tests = match tests.checked_add(1) {
                        Some(count) if count <= MAX_COLLISION_TESTS => count,
                        _ => return Err(invalid(format!("too many collision tests: limit is {MAX_COLLISION_TESTS}"))),
                    };
                    let Some(triangle) = self.mesh.triangles().get(tri_index).copied() else {
                        continue;
                    };
                    let Some(hit) = test(triangle) else {
                        continue;
                    };
                    if collides(hit.distance, clearance) {
                        consider(&mut best, Some(hit));
                    }
                }
            } else {
                stack.push(node.right);
                stack.push(node.left);
            }
        }
        Ok(best)
    }
}

fn query_bounds(bounds: Aabb, clearance: f64) -> Option<Aabb> {
    if !clearance.is_finite() {
        return None;
    }
    Some(bounds.expand(clearance.max(0.0) + 1e-6))
}

fn build_bvh(mesh: &Mesh, indices: &mut [usize], depth: usize, nodes: &mut Vec<BvhNode>) -> Result<usize> {
    let bounds = bounds_of(mesh, indices)?;
    if indices.len() <= BVH_LEAF || depth >= BVH_DEPTH {
        let id = nodes.len();
        nodes.push(BvhNode { bounds, left: 0, right: indices.len(), leaf: true, tris: indices.to_vec() });
        return Ok(id);
    }
    let axis = bounds.longest_axis();
    indices.sort_by(|left, right| {
        let lc = mesh.triangles().get(*left).map(|tri| tri.bbox().centroid());
        let rc = mesh.triangles().get(*right).map(|tri| tri.bbox().centroid());
        let (Some(lc), Some(rc)) = (lc, rc) else {
            return core::cmp::Ordering::Equal;
        };
        let (lv, rv) = match axis {
            0 => (lc.x, rc.x),
            1 => (lc.y, rc.y),
            _ => (lc.z, rc.z),
        };
        lv.total_cmp(&rv)
    });
    let mid = indices.len() / 2;
    if mid == 0 || mid == indices.len() {
        let id = nodes.len();
        nodes.push(BvhNode { bounds, left: 0, right: indices.len(), leaf: true, tris: indices.to_vec() });
        return Ok(id);
    }
    let id = nodes.len();
    nodes.push(BvhNode { bounds, left: 0, right: 0, leaf: false, tris: Vec::new() });
    let (left_idx, right_idx) = indices.split_at_mut(mid);
    let left = build_bvh(mesh, left_idx, depth + 1, nodes)?;
    let right = build_bvh(mesh, right_idx, depth + 1, nodes)?;
    if let Some(node) = nodes.get_mut(id) {
        node.left = left;
        node.right = right;
    }
    Ok(id)
}

fn bounds_of(mesh: &Mesh, indices: &[usize]) -> Result<Aabb> {
    let mut bounds: Option<Aabb> = None;
    for &index in indices {
        let Some(triangle) = mesh.triangles().get(index) else {
            continue;
        };
        bounds = Some(match bounds {
            Some(current) => current.union(triangle.bbox()),
            None => triangle.bbox(),
        });
    }
    bounds.ok_or_else(|| invalid("empty collision index"))
}

/// An oriented box. `axes` are unit vectors and `half` is the extent along each one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obb {
    pub center: Vec3,
    pub axes: [Vec3; 3],
    pub half: [f64; 3],
}

impl Obb {
    /// Axis-aligned box written as an oriented box.
    pub fn from_aabb(bounds: Aabb) -> Self {
        let min = bounds.min();
        let max = bounds.max();
        Self {
            center: Vec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5),
            axes: [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)],
            half: [(max.x - min.x) * 0.5, (max.y - min.y) * 0.5, (max.z - min.z) * 0.5],
        }
    }

    /// `None` when a coordinate is not finite or a half-extent is negative.
    pub fn try_new(center: Vec3, axes: [Vec3; 3], half: [f64; 3]) -> Option<Self> {
        if !finite_vec(center) || axes.iter().any(|axis| !finite_vec(*axis)) || half.iter().any(|extent| !extent.is_finite() || *extent < 0.0) {
            return None;
        }
        Some(Self { center, axes, half })
    }

    fn radius_along(self, axis: Vec3) -> f64 {
        self.half[0] * self.axes[0].dot(axis).abs() + self.half[1] * self.axes[1].dot(axis).abs() + self.half[2] * self.axes[2].dot(axis).abs()
    }
}

/// Signed distance from the solid sphere to the solid axis-aligned box.
pub fn sphere_aabb(sphere: Sphere, bounds: Aabb) -> Option<CollisionHit> {
    if sphere.radius < 0.0 || !sphere.radius.is_finite() || !finite_vec(sphere.center) {
        return None;
    }
    let mut best = None;
    for triangle in bounds.face_triangles() {
        consider(&mut best, sphere_triangle(sphere, triangle));
    }
    if bounds.contains(sphere.center) {
        consider(&mut best, Some(CollisionHit { point: sphere.center, distance: -sphere.radius }));
    }
    best
}

/// Signed distance from the solid sphere to an oriented box.
pub fn sphere_obb(sphere: Sphere, obb: Obb) -> Option<CollisionHit> {
    let local = into_obb(obb, sphere.center);
    let local_sphere = Sphere { center: local, radius: sphere.radius };
    let bounds = aabb_of_obb(obb);
    let hit = sphere_aabb(local_sphere, bounds)?;
    Some(CollisionHit { point: from_obb(obb, hit.point), distance: hit.distance })
}

/// Signed distance from the solid cylinder to an oriented box.
pub fn cylinder_obb(cylinder: Cylinder, obb: Obb) -> Option<CollisionHit> {
    let local = Cylinder { a: into_obb(obb, cylinder.a), b: into_obb(obb, cylinder.b), radius: cylinder.radius };
    let hit = cylinder_aabb(local, aabb_of_obb(obb))?;
    Some(CollisionHit { point: from_obb(obb, hit.point), distance: hit.distance })
}

/// Signed distance between two oriented boxes.
///
/// The separating axes are the three face normals of each box and the nine cross products of
/// those normals. A near-zero cross product is skipped. The witness is the projection of the
/// first centre onto the plane that bisects the overlap along the extreme axis.
pub fn obb_obb(a: Obb, b: Obb) -> Option<CollisionHit> {
    if !obb_finite(a) || !obb_finite(b) {
        return None;
    }
    let sep = b.center.sub(a.center);
    let mut best_gap = f64::NEG_INFINITY;
    let mut best_axis = Vec3::new(1.0, 0.0, 0.0);
    let mut saw = false;
    for axis in obb_axes(a, b) {
        let Some(normal) = axis.normalized() else {
            continue;
        };
        let signed = sep.dot(normal);
        let direction = if signed < 0.0 { normal.scale(-1.0) } else { normal };
        let distance = signed.abs();
        let gap = distance - a.radius_along(direction) - b.radius_along(direction);
        if !gap.is_finite() {
            return None;
        }
        if gap > best_gap {
            best_gap = gap;
            best_axis = direction;
            saw = true;
        }
    }
    if !saw || !best_gap.is_finite() {
        return None;
    }
    let point = obb_witness(a, b, best_axis, best_gap);
    if finite_vec(point) { Some(CollisionHit { point, distance: best_gap }) } else { None }
}

/// Signed distance from the solid sphere to the solid cylinder.
pub fn sphere_cylinder(sphere: Sphere, cylinder: Cylinder) -> Option<CollisionHit> {
    if sphere.radius < 0.0 || !sphere.radius.is_finite() || !finite_vec(sphere.center) {
        return None;
    }
    let radial = point_to_cylinder(sphere.center, cylinder)?;
    let distance = radial - sphere.radius;
    if !distance.is_finite() {
        return None;
    }
    Some(CollisionHit { point: sphere.center, distance })
}

/// Signed distance between two finite cylinders.
///
/// Side-to-side contact whose closest points lie in the interior of both axes is exact. A gap
/// at least as large as the two radii separates the cylinders. An axis point inside the other
/// solid is a hit. Otherwise an inscribed tessellation is tested, and a negative distance from
/// that is a hit. A remaining overlap of the surrounding capsules is reported as distance 0,
/// which is contact and not a hit.
pub fn cylinder_cylinder(a: Cylinder, b: Cylinder) -> Option<CollisionHit> {
    if a.radius < 0.0 || b.radius < 0.0 || !a.radius.is_finite() || !b.radius.is_finite() {
        return None;
    }
    let (dir_a, len_a) = a.direction()?;
    let (dir_b, len_b) = b.direction()?;
    let (dist, pa, pb) = segment_segment(a.a, a.b, b.a, b.b);
    if !dist.is_finite() || !finite_vec(pa) || !finite_vec(pb) {
        return None;
    }
    let gap = dist - a.radius - b.radius;
    let ta = if len_a > 0.0 { pa.sub(a.a).dot(dir_a) } else { 0.0 };
    let tb = if len_b > 0.0 { pb.sub(b.a).dot(dir_b) } else { 0.0 };
    let interior_a = len_a > 0.0 && ta > 1e-9 && ta < len_a - 1e-9;
    let interior_b = len_b > 0.0 && tb > 1e-9 && tb < len_b - 1e-9;
    if gap >= 0.0 {
        return Some(CollisionHit { point: pa, distance: gap });
    }
    if interior_a && interior_b {
        return Some(CollisionHit { point: pa, distance: gap });
    }
    if let Some(hit) = axis_inside(a, b) {
        return Some(hit);
    }
    if let Some(hit) = axis_inside(b, a) {
        return Some(hit);
    }
    if let Some(hit) = inscribed_cylinder_hit(a, b)
        && hit.distance < 0.0
    {
        return Some(hit);
    }
    Some(CollisionHit { point: pa, distance: 0.0 })
}

fn obb_finite(obb: Obb) -> bool {
    finite_vec(obb.center) && obb.axes.iter().all(|axis| finite_vec(*axis)) && obb.half.iter().all(|extent| extent.is_finite() && *extent >= 0.0)
}

fn aabb_of_obb(obb: Obb) -> Aabb {
    Aabb::from_corners(Vec3::new(-obb.half[0], -obb.half[1], -obb.half[2]), Vec3::new(obb.half[0], obb.half[1], obb.half[2]))
}

fn into_obb(obb: Obb, point: Vec3) -> Vec3 {
    let delta = point.sub(obb.center);
    Vec3::new(delta.dot(obb.axes[0]), delta.dot(obb.axes[1]), delta.dot(obb.axes[2]))
}

fn from_obb(obb: Obb, point: Vec3) -> Vec3 {
    obb.center.add(obb.axes[0].scale(point.x)).add(obb.axes[1].scale(point.y)).add(obb.axes[2].scale(point.z))
}

fn obb_axes(a: Obb, b: Obb) -> [Vec3; 15] {
    let mut axes = [Vec3::new(0.0, 0.0, 0.0); 15];
    axes[0] = a.axes[0];
    axes[1] = a.axes[1];
    axes[2] = a.axes[2];
    axes[3] = b.axes[0];
    axes[4] = b.axes[1];
    axes[5] = b.axes[2];
    let mut slot = 6;
    for left in a.axes {
        for right in b.axes {
            axes[slot] = left.cross(right);
            slot += 1;
        }
    }
    axes
}

fn obb_witness(a: Obb, b: Obb, axis: Vec3, gap: f64) -> Vec3 {
    let ra = a.radius_along(axis);
    let rb = b.radius_along(axis);
    let distance = b.center.sub(a.center).dot(axis);
    if gap >= 0.0 {
        return a.center.add(axis.scale(ra));
    }
    let lo = (-ra).max(distance - rb);
    let hi = ra.min(distance + rb);
    a.center.add(axis.scale((lo + hi) * 0.5))
}

fn axis_inside(probe: Cylinder, solid: Cylinder) -> Option<CollisionHit> {
    let (dir, len) = probe.direction()?;
    let steps = 8;
    let mut best = None;
    for index in 0..=steps {
        let t = if steps == 0 { 0.0 } else { usize_f64(index) / usize_f64(steps) };
        let point = probe.a.add(dir.scale(len * t));
        let Some(distance) = point_to_cylinder(point, solid) else {
            continue;
        };
        if distance < 0.0 {
            consider(&mut best, Some(CollisionHit { point, distance }));
        }
    }
    best
}

fn inscribed_cylinder_hit(a: Cylinder, b: Cylinder) -> Option<CollisionHit> {
    let mut best = None;
    for triangle in cylinder_shell(a, 24) {
        consider(&mut best, cylinder_triangle(b, triangle));
    }
    best
}

fn usize_f64(value: usize) -> f64 {
    u32::try_from(value).map_or(0.0, f64::from)
}

fn cylinder_shell(cylinder: Cylinder, sides: usize) -> Vec<Triangle> {
    let Some((dir, len)) = cylinder.direction() else {
        return Vec::new();
    };
    let Some((u, v)) = perp_basis(dir) else {
        return Vec::new();
    };
    let sides = sides.max(3);
    let mut ring_a = Vec::with_capacity(sides);
    let mut ring_b = Vec::with_capacity(sides);
    for index in 0..sides {
        let angle = usize_f64(index) * std::f64::consts::TAU / usize_f64(sides);
        let offset = u.scale(cylinder.radius * angle.cos()).add(v.scale(cylinder.radius * angle.sin()));
        ring_a.push(cylinder.a.add(offset));
        ring_b.push(cylinder.b.add(dir.scale(len)).add(offset));
    }
    let mut triangles = Vec::with_capacity(sides * 4);
    for index in 0..sides {
        let next = (index + 1) % sides;
        let (Some(a0), Some(a1), Some(b0), Some(b1)) = (ring_a.get(index), ring_a.get(next), ring_b.get(index), ring_b.get(next)) else {
            continue;
        };
        triangles.push(Triangle { a: *a0, b: *a1, c: *b1 });
        triangles.push(Triangle { a: *a0, b: *b1, c: *b0 });
        triangles.push(Triangle { a: cylinder.a, b: *a0, c: *a1 });
        triangles.push(Triangle { a: cylinder.b, b: *b1, c: *b0 });
    }
    triangles
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Mesh;
    use proptest::prelude::*;

    fn close(actual: f64, expected: f64, tol: f64) {
        assert!((actual - expected).abs() <= tol, "{actual} != {expected} ± {tol}");
    }

    fn tri(a: Vec3, b: Vec3, c: Vec3) -> Triangle {
        Triangle { a, b, c }
    }

    fn collide_config() -> ProptestConfig {
        ProptestConfig { cases: 32, max_shrink_iters: 200, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_111D), ..ProptestConfig::default() }
    }

    fn point_cyl_ref(point: Vec3, cylinder: Cylinder) -> f64 {
        point_to_cylinder(point, cylinder).unwrap_or(f64::INFINITY)
    }

    fn sample_triangle(triangle: Triangle, n: usize) -> Vec<Vec3> {
        let mut out = Vec::new();
        if n == 0 {
            return out;
        }
        for i in 0..=n {
            for j in 0..=(n - i) {
                let u = i as f64 / n as f64;
                let v = j as f64 / n as f64;
                let w = 1.0 - u - v;
                out.push(triangle.a.scale(w).add(triangle.b.scale(u)).add(triangle.c.scale(v)));
            }
        }
        out
    }

    fn sample_cylinder_surface(cylinder: Cylinder, n_theta: usize, n_len: usize) -> Vec<Vec3> {
        let mut out = Vec::new();
        let Some((dir, len)) = cylinder.direction() else {
            return out;
        };
        let Some((u, v)) = perp_basis(dir) else {
            return out;
        };
        let r = cylinder.radius;
        for i in 0..=n_len {
            let t = if n_len == 0 { 0.0 } else { i as f64 / n_len as f64 };
            let center = cylinder.a.lerp(cylinder.b, t);
            for k in 0..n_theta {
                let a = (k as f64) * std::f64::consts::TAU / n_theta as f64;
                out.push(center.add(u.scale(r * a.cos())).add(v.scale(r * a.sin())));
            }
        }
        for cap in [cylinder.a, cylinder.b] {
            for i in 0..=n_len {
                let rho = if n_len == 0 { 0.0 } else { r * i as f64 / n_len as f64 };
                for k in 0..n_theta {
                    let a = (k as f64) * std::f64::consts::TAU / n_theta as f64;
                    out.push(cap.add(u.scale(rho * a.cos())).add(v.scale(rho * a.sin())));
                }
            }
        }
        let _ = len;
        out
    }

    fn brute_cylinder_triangle(cylinder: Cylinder, triangle: Triangle, n: usize) -> f64 {
        let mut best = f64::INFINITY;
        for p in sample_triangle(triangle, n) {
            best = best.min(point_cyl_ref(p, cylinder));
        }
        for p in sample_cylinder_surface(cylinder, n.max(8), n.max(4)) {
            let q = closest_point_on_triangle(p, triangle);
            let d = p.sub(q).length();
            if point_cyl_ref(q, cylinder) >= -1e-9 {
                best = best.min(d);
            }
        }
        best
    }

    fn brute_sphere_triangle(sphere: Sphere, triangle: Triangle, n: usize) -> f64 {
        let mut best = f64::INFINITY;
        for p in sample_triangle(triangle, n) {
            best = best.min(p.sub(sphere.center).length() - sphere.radius);
        }
        best
    }

    fn brute_cylinder_box(cylinder: Cylinder, bounds: Aabb, n: usize) -> f64 {
        let mut best = f64::INFINITY;
        for triangle in bounds.face_triangles() {
            best = best.min(brute_cylinder_triangle(cylinder, triangle, n));
        }
        if bounds.contains(cylinder.a) || bounds.contains(cylinder.b) || axis_hits_aabb(cylinder.a, cylinder.b, bounds) {
            best = best.min((-cylinder.radius).min(0.0));
        }
        best
    }

    fn rotate(point: Vec3, axis: Vec3, angle: f64) -> Vec3 {
        let Some(k) = axis.normalized() else {
            return point;
        };
        let c = angle.cos();
        let s = angle.sin();
        point.scale(c).add(k.cross(point).scale(s)).add(k.scale(k.dot(point) * (1.0 - c)))
    }

    fn rigid(point: Vec3, axis: Vec3, angle: f64, trans: Vec3) -> Vec3 {
        rotate(point, axis, angle).add(trans)
    }

    #[test]
    fn unit_cylinder_misses_a_far_triangle_and_hits_one_through_the_axis() {
        let cyl = Cylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 4.0), 1.0).expect("cyl");
        let far = tri(Vec3::new(5.0, 0.0, 1.0), Vec3::new(6.0, 1.0, 1.0), Vec3::new(6.0, -1.0, 2.0));
        let hit = cylinder_triangle(cyl, far).expect("hit");
        close(hit.distance, 4.0, 1e-9);
        assert!(!collides(hit.distance, 0.0));
        assert!(collides(hit.distance, 4.1));
        assert!(!collides(hit.distance, 4.0));

        let through = tri(Vec3::new(-2.0, 0.0, 2.0), Vec3::new(2.0, 0.0, 2.0), Vec3::new(0.0, 2.0, 2.0));
        let hit = cylinder_triangle(cyl, through).expect("through");
        assert!(hit.distance < -0.5, "{}", hit.distance);
        assert!(collides(hit.distance, 0.0));
    }

    #[test]
    fn triangle_beyond_the_cap_sees_the_disk() {
        let cyl = Cylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 2.0), 1.0).expect("cyl");
        let beyond = tri(Vec3::new(-0.2, -0.2, 3.0), Vec3::new(0.2, -0.2, 3.0), Vec3::new(0.0, 0.2, 3.0));
        let hit = cylinder_triangle(cyl, beyond).expect("cap");
        close(hit.distance, 1.0, 1e-6);
    }

    #[test]
    fn triangle_inside_the_cylinder_is_negative() {
        let cyl = Cylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 4.0), 2.0).expect("cyl");
        let inside = tri(Vec3::new(0.1, 0.0, 1.0), Vec3::new(0.2, 0.1, 1.5), Vec3::new(0.0, 0.2, 2.0));
        let hit = cylinder_triangle(cyl, inside).expect("inside");
        assert!(hit.distance < 0.0, "{}", hit.distance);
        assert!(collides(hit.distance, 0.0));
    }

    #[test]
    fn sphere_on_a_plane_matches_the_offset() {
        let sphere = Sphere::new(Vec3::new(0.0, 0.0, 3.0), 1.0).expect("sphere");
        let plane = tri(Vec3::new(-2.0, -2.0, 0.0), Vec3::new(2.0, -2.0, 0.0), Vec3::new(0.0, 2.0, 0.0));
        let hit = sphere_triangle(sphere, plane).expect("plane");
        close(hit.distance, 2.0, 1e-9);
        assert!(!collides(hit.distance, 2.0));
        assert!(collides(hit.distance, 2.1));
    }

    #[test]
    fn cylinder_versus_a_box_beside_and_around() {
        let cyl = Cylinder::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 4.0), 1.0).expect("cyl");
        let beside = Aabb::from_corners(Vec3::new(3.0, -1.0, 1.0), Vec3::new(4.0, 1.0, 3.0));
        let hit = cylinder_aabb(cyl, beside).expect("beside");
        close(hit.distance, 2.0, 1e-6);
        let around = Aabb::from_corners(Vec3::new(-2.0, -2.0, -1.0), Vec3::new(2.0, 2.0, 5.0));
        let hit = cylinder_aabb(cyl, around).expect("around");
        assert!(hit.distance < 0.0, "{}", hit.distance);
    }

    #[test]
    fn index_finds_a_held_out_triangle_and_respects_clearance() {
        let mesh = Mesh::new(vec![
            tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), Vec3::new(0.0, 4.0, 0.0)),
            tri(Vec3::new(10.0, 10.0, 0.0), Vec3::new(12.0, 10.0, 0.0), Vec3::new(10.0, 12.0, 0.0)),
        ])
        .expect("mesh");
        let index = CollisionIndex::new(&mesh).expect("index");
        let cyl = Cylinder::new(Vec3::new(1.0, 1.0, 2.0), Vec3::new(1.0, 1.0, 5.0), 0.5).expect("cyl");
        let miss = index.query_cylinder(cyl, 1.0).expect("query");
        assert!(miss.is_none(), "{miss:?}");
        let hit = index.query_cylinder(cyl, 2.1).expect("clear");
        assert!(hit.is_some(), "clearance 2.1 should reach the floor");
        let empty = Mesh::new(vec![]).expect("empty");
        let empty_index = CollisionIndex::new(&empty).expect("empty index");
        assert!(empty_index.query_cylinder(cyl, 10.0).expect("empty query").is_none());
    }

    #[test]
    fn collides_uses_strict_clearance_with_slack() {
        assert!(!collides(0.0, 0.0));
        assert!(collides(-1e-8, 0.0));
        assert!(!collides(-1e-12, 0.0));
        assert!(!collides(0.1, 0.1));
        assert!(collides(0.1 - 1e-8, 0.1));
    }

    proptest! {
        #![proptest_config(collide_config())]

        #[test]
        fn cylinder_triangle_matches_dense_samples(
            kind in 0u8..5,
            ax in -2.0f64..2.0,
            ay in -2.0f64..2.0,
            az in -1.0f64..1.0,
            bx in -2.0f64..2.0,
            by in -2.0f64..2.0,
            bz in 1.5f64..4.0,
            radius in 0.2f64..1.8,
            t in 0.0f64..1.0,
            ux in -2.0f64..2.0,
            uy in -2.0f64..2.0,
            uz in -2.0f64..2.0,
            vx in -2.0f64..2.0,
            vy in -2.0f64..2.0,
            vz in -2.0f64..2.0,
        ) {
            let a = Vec3::new(ax, ay, az);
            let b = Vec3::new(bx, by, bz);
            let Some(cyl) = Cylinder::new(a, b, radius) else {
                return Ok(());
            };
            let dir = b.sub(a);
            let Some(u) = dir.normalized() else {
                return Ok(());
            };
            let triangle = match kind {
                0 => tri(Vec3::new(ux, uy, uz), Vec3::new(vx, vy, vz), Vec3::new(ux + 0.4, vy, uz + 0.3)),
                1 => {
                    let mid = a.lerp(b, 0.4);
                    tri(mid.add(Vec3::new(-1.5, 0.2, 0.0)), mid.add(Vec3::new(1.5, 0.2, 0.0)), mid.add(Vec3::new(0.0, 1.2, 0.1)))
                }
                2 => {
                    let mid = a.lerp(b, 0.5);
                    let r = radius * 0.3;
                    tri(mid.add(Vec3::new(r, 0.0, 0.0)), mid.add(Vec3::new(0.0, r, 0.0)), mid.add(Vec3::new(0.0, 0.0, r * 0.2)))
                }
                3 => {
                    let origin = a.add(u.scale(0.5 * cyl.b.sub(cyl.a).length()));
                    tri(origin.add(Vec3::new(radius + 0.5, -0.4, 0.0)), origin.add(Vec3::new(radius + 0.5, 0.4, 0.0)), origin.add(u.scale(0.8)).add(Vec3::new(radius + 0.5, 0.0, 0.0)))
                }
                _ => {
                    let beyond = b.add(u.scale(1.0 + t));
                    tri(beyond.add(Vec3::new(-0.3, -0.3, 0.0)), beyond.add(Vec3::new(0.3, -0.3, 0.0)), beyond.add(Vec3::new(0.0, 0.3, 0.0)))
                }
            };
            let Some(hit) = cylinder_triangle(cyl, triangle) else {
                return Ok(());
            };
            let brute = brute_cylinder_triangle(cyl, triangle, 10);
            prop_assume!(brute.is_finite());
            let bound = 0.45_f64.max(radius * 0.15);
            if hit.distance >= 0.0 {
                prop_assert!((hit.distance - brute).abs() <= bound, "exact {} brute {} kind {kind}", hit.distance, brute);
            } else {
                prop_assert!(brute <= bound, "inside exact {} brute {} kind {kind}", hit.distance, brute);
                prop_assert!(hit.distance <= brute + 1e-6, "exact {} should be <= brute {}", hit.distance, brute);
            }
        }

        #[test]
        fn sphere_triangle_matches_dense_samples(
            cx in -1.0f64..1.0,
            cy in -1.0f64..1.0,
            cz in -1.0f64..2.0,
            radius in 0.2f64..1.5,
            ax in -2.0f64..2.0,
            ay in -2.0f64..2.0,
            az in -2.0f64..2.0,
            bx in -2.0f64..2.0,
            by in -2.0f64..2.0,
            bz in -2.0f64..2.0,
            cx2 in -2.0f64..2.0,
            cy2 in -2.0f64..2.0,
            cz2 in -2.0f64..2.0,
        ) {
            let Some(sphere) = Sphere::new(Vec3::new(cx, cy, cz), radius) else {
                return Ok(());
            };
            let triangle = tri(Vec3::new(ax, ay, az), Vec3::new(bx, by, bz), Vec3::new(cx2, cy2, cz2));
            prop_assume!(triangle.normal().is_some());
            let Some(hit) = sphere_triangle(sphere, triangle) else {
                return Ok(());
            };
            let brute = brute_sphere_triangle(sphere, triangle, 12);
            prop_assume!(brute.is_finite());
            let bound = 0.35_f64;
            prop_assert!(hit.distance <= brute + 1e-6, "exact {} brute {}", hit.distance, brute);
            prop_assert!(brute <= hit.distance + bound, "exact {} brute {}", hit.distance, brute);
        }

        #[test]
        fn cylinder_box_matches_dense_samples(
            ax in -1.0f64..1.0,
            ay in -1.0f64..1.0,
            az in -1.0f64..1.0,
            bx in -1.0f64..1.0,
            by in -1.0f64..1.0,
            bz in 1.0f64..3.0,
            radius in 0.2f64..1.2,
            x0 in -2.0f64..0.5,
            y0 in -2.0f64..0.5,
            z0 in -1.0f64..0.5,
            sx in 0.4f64..2.5,
            sy in 0.4f64..2.5,
            sz in 0.4f64..2.5,
        ) {
            let Some(cyl) = Cylinder::new(Vec3::new(ax, ay, az), Vec3::new(bx, by, bz), radius) else {
                return Ok(());
            };
            let bounds = Aabb::from_corners(Vec3::new(x0, y0, z0), Vec3::new(x0 + sx, y0 + sy, z0 + sz));
            let Some(hit) = cylinder_aabb(cyl, bounds) else {
                return Ok(());
            };
            let brute = brute_cylinder_box(cyl, bounds, 6);
            prop_assume!(brute.is_finite());
            let bound = 0.6_f64.max(radius * 0.25);
            if hit.distance >= 0.0 {
                prop_assert!((hit.distance - brute).abs() <= bound, "exact {} brute {}", hit.distance, brute);
            } else {
                prop_assert!(brute <= bound || brute <= 0.0, "overlap exact {} brute {}", hit.distance, brute);
            }
        }

        #[test]
        fn rigid_transform_preserves_cylinder_triangle_distance(
            ax in -1.0f64..1.0,
            ay in -1.0f64..1.0,
            az in 0.0f64..1.0,
            bx in -1.0f64..1.0,
            by in -1.0f64..1.0,
            bz in 2.0f64..3.5,
            radius in 0.3f64..1.4,
            tx in -1.0f64..1.0,
            ty in -1.0f64..1.0,
            tz in -1.0f64..1.0,
            kx in -1.0f64..1.0,
            ky in -1.0f64..1.0,
            kz in -1.0f64..1.0,
            angle in -1.2f64..1.2,
            ox in -4.0f64..4.0,
            oy in -4.0f64..4.0,
            oz in -4.0f64..4.0,
        ) {
            let Some(cyl) = Cylinder::new(Vec3::new(ax, ay, az), Vec3::new(bx, by, bz), radius) else {
                return Ok(());
            };
            let triangle = tri(Vec3::new(tx, ty, tz), Vec3::new(tx + 0.8, ty + 0.1, tz + 0.2), Vec3::new(tx + 0.1, ty + 0.7, tz - 0.1));
            let Some(before) = cylinder_triangle(cyl, triangle) else {
                return Ok(());
            };
            let axis = Vec3::new(kx, ky, kz);
            let trans = Vec3::new(ox, oy, oz);
            let Some(moved) = Cylinder::new(rigid(cyl.a, axis, angle, trans), rigid(cyl.b, axis, angle, trans), radius) else {
                return Ok(());
            };
            let moved_tri = tri(rigid(triangle.a, axis, angle, trans), rigid(triangle.b, axis, angle, trans), rigid(triangle.c, axis, angle, trans));
            let Some(after) = cylinder_triangle(moved, moved_tri) else {
                return Ok(());
            };
            prop_assert!((before.distance - after.distance).abs() <= 1e-7, "before {} after {}", before.distance, after.distance);
        }
    }

    fn plate_with_target(target: Triangle) -> Mesh {
        Mesh::new(vec![
            tri(Vec3::new(-20.0, -20.0, 0.0), Vec3::new(20.0, -20.0, 0.0), Vec3::new(20.0, 20.0, 0.0)),
            tri(Vec3::new(-20.0, -20.0, 0.0), Vec3::new(20.0, 20.0, 0.0), Vec3::new(-20.0, 20.0, 0.0)),
            target,
        ])
        .expect("plate")
    }

    proptest! {
        #![proptest_config(collide_config())]

        #[test]
        fn tilted_holder_clears_when_pushed_and_hits_when_grown(
            lead in -0.35f64..0.35,
            tilt in -0.35f64..0.35,
            x in -3.0f64..3.0,
            y in -3.0f64..3.0,
        ) {
            let z_axis = Vec3::new(0.0, 0.0, 1.0);
            let mut axis = rotate(z_axis, Vec3::new(1.0, 0.0, 0.0), lead);
            axis = rotate(axis, Vec3::new(0.0, 1.0, 0.0), tilt);
            let Some(axis) = axis.normalized() else {
                return Ok(());
            };
            prop_assume!(axis.z > 0.4);
            let Some((u, v)) = super::perp_basis(axis) else {
                return Ok(());
            };
            let tip = Vec3::new(x, y, 2.0);
            let stickout = 6.0;
            let holder_a = tip.add(axis.scale(stickout));
            let holder_b = holder_a.add(axis.scale(16.0));
            let offset = 5.0;
            let mid = holder_a.lerp(holder_b, 0.5);
            let center = mid.add(u.scale(offset));
            let target = tri(center.add(v.scale(1.2)), center.add(v.scale(-1.0)).add(axis.scale(0.8)), center.add(axis.scale(-0.8)));
            prop_assume!(target.normal().is_some());
            let mesh = plate_with_target(target);
            let index = CollisionIndex::new(&mesh).expect("index");

            let push = axis.scale(40.0);
            let Some(clear_cyl) = Cylinder::new(holder_a.add(push), holder_b.add(push), offset + 2.0) else {
                return Ok(());
            };
            let clear = index.query_cylinder(clear_cyl, 0.0).expect("clear query");
            prop_assert!(clear.is_none(), "pushed-clear pose collided: {clear:?}");

            let Some(hit_cyl) = Cylinder::new(holder_a, holder_b, offset + 1.5) else {
                return Ok(());
            };
            let hit = index.query_cylinder(hit_cyl, 0.0).expect("hit query");
            prop_assert!(hit.is_some(), "grown holder should reach the known triangle at offset {offset}");
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 48,
            max_shrink_iters: 64,
            rng_seed: proptest::test_runner::RngSeed::Fixed(0x0BB0_0B01),
            ..ProptestConfig::default()
        })]

        #[test]
        fn boxes_overlap_when_interiors_meet_and_a_gap_or_a_touch_does_not(
            hx in 0.5f64..20.0,
            hy in 0.5f64..20.0,
            hz in 0.5f64..20.0,
            ox in 0.5f64..20.0,
            oy in 0.5f64..20.0,
            oz in 0.5f64..20.0,
            margin in 0.5f64..8.0,
        ) {
            let left = Obb::from_aabb(Aabb::from_corners(Vec3::new(-hx, -hy, -hz), Vec3::new(hx, hy, hz)));
            let overlap = Obb::from_aabb(Aabb::from_corners(Vec3::new(-ox, -oy, -oz), Vec3::new(ox, oy, oz)));
            let hit = obb_obb(left, overlap).expect("overlap");
            prop_assert!(hit.distance < -1e-6, "overlap distance {}", hit.distance);
            prop_assert!(collides(hit.distance, 0.0));

            let gap_box = Obb::from_aabb(Aabb::from_corners(
                Vec3::new(hx + margin, -oy, -oz),
                Vec3::new(hx + margin + 2.0 * ox, oy, oz),
            ));
            let gap = obb_obb(left, gap_box).expect("gap");
            prop_assert!((gap.distance - margin).abs() <= 1e-6, "gap {} != {margin}", gap.distance);
            prop_assert!(!collides(gap.distance, 0.0));

            let touch_box = Obb::from_aabb(Aabb::from_corners(Vec3::new(hx, -oy, -oz), Vec3::new(hx + 2.0 * ox, oy, oz)));
            let touch = obb_obb(left, touch_box).expect("touch");
            prop_assert!(touch.distance.abs() <= 1e-6, "touch distance {}", touch.distance);
            prop_assert!(!collides(touch.distance, 0.0));
        }
    }
}
