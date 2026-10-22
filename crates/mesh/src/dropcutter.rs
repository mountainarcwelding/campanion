//! Drop-cutter: the highest tool-tip Z at which a cutter lowered along −Z first touches a triangle.
//!
//! The tool axis is +Z. The cutter location is the tip, the lowest point on that axis. For a
//! surface point at radial distance `d` from the axis, the tip sits at `z - h(d)`, where `h` is
//! the tool profile. The drop height is the maximum of those values over the triangle.

use crate::{Mesh, Result, Triangle, Vec3, invalid};

/// Most cells in the uniform XY bucket grid.
const MAX_BUCKET_CELLS: usize = 1_000_000;

/// A round shank tool spinning about +Z, described by its radial profile.
///
/// `corner_radius` of a bull nose lies in `(0, radius)`. The profile limits are a ball
/// (`corner_radius == radius`) and a flat (`corner_radius == 0`); [`Cutter::new_bull`] rejects
/// those and the dedicated variants should be used instead. The limit shapes still follow the
/// bull profile, so direct `Bull` values at the endpoints match [`Cutter::Ball`] and [`Cutter::Flat`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cutter {
    /// Flat end mill. The bottom is a disk of `radius` at the tip.
    Flat { radius: f64 },
    /// Ball nose. Sphere of `radius` whose lowest point is the tip.
    Ball { radius: f64 },
    /// Bull nose. Flat disk of radius `radius - corner_radius`, then a toroidal corner.
    Bull { radius: f64, corner_radius: f64 },
}

impl Cutter {
    /// Flat end mill. `radius` must be finite and positive.
    pub fn new_flat(radius: f64) -> Result<Self> {
        check_positive(radius, "flat cutter radius")?;
        Ok(Self::Flat { radius })
    }

    /// Ball nose. `radius` must be finite and positive.
    pub fn new_ball(radius: f64) -> Result<Self> {
        check_positive(radius, "ball cutter radius")?;
        Ok(Self::Ball { radius })
    }

    /// Bull nose. `corner_radius` must lie in `(0, radius)`.
    pub fn new_bull(radius: f64, corner_radius: f64) -> Result<Self> {
        check_positive(radius, "bull cutter radius")?;
        if !corner_radius.is_finite() || corner_radius <= 0.0 || corner_radius >= radius {
            return Err(invalid("bull corner radius must lie in (0, radius)"));
        }
        Ok(Self::Bull { radius, corner_radius })
    }

    /// Outer radius when it is finite and positive.
    pub fn radius(self) -> Option<f64> {
        self.profile().map(|(radius, _)| radius)
    }

    /// `(outer radius, corner radius)`. Flat uses `0`; ball uses the outer radius.
    ///
    /// A bull corner outside `[0, radius]` is rejected. The endpoints are kept so a bull can
    /// be compared with the flat and ball limits.
    fn profile(self) -> Option<(f64, f64)> {
        match self {
            Self::Flat { radius } if finite_positive(radius) => Some((radius, 0.0)),
            Self::Ball { radius } if finite_positive(radius) => Some((radius, radius)),
            Self::Bull { radius, corner_radius }
                if finite_positive(radius) && corner_radius.is_finite() && (0.0..=radius).contains(&corner_radius) =>
            {
                Some((radius, corner_radius))
            }
            _ => None,
        }
    }

    /// Height of the cutting surface above the tip at radial distance `d`.
    ///
    /// `None` when `d` is outside the tool or the cutter is not usable.
    /// Flat: `0` for `d <= R`. Ball: `R - sqrt(R² - d²)`. Bull: `0` for `d <= R - c`, otherwise
    /// `c - sqrt(c² - (d - (R - c))²)`.
    pub fn height(self, radial_distance: f64) -> Option<f64> {
        let (radius, corner) = self.profile()?;
        if !radial_distance.is_finite() {
            return None;
        }
        let d = radial_distance.max(0.0);
        if d > radius {
            return None;
        }
        let flat = radius - corner;
        if corner == 0.0 || d <= flat {
            return Some(0.0);
        }
        let u = d - flat;
        let under = corner * corner - u * u;
        if under < 0.0 {
            // `d == radius` can undershoot by a couple of ulps.
            return if under >= -1e-9 * corner * corner { Some(corner) } else { None };
        }
        let h = corner - under.sqrt();
        if h.is_finite() { Some(h.max(0.0)) } else { None }
    }
}

fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn check_positive(value: f64, what: &str) -> Result<()> {
    if finite_positive(value) { Ok(()) } else { Err(invalid(format!("{what} must be finite and positive"))) }
}

fn maximise(best: &mut Option<f64>, candidate: Option<f64>) {
    let Some(z) = candidate else {
        return;
    };
    if !z.is_finite() {
        return;
    }
    *best = Some(match *best {
        Some(current) => current.max(z),
        None => z,
    });
}

/// Tip Z that puts the tool surface through `point`. `None` when the point is outside the radius.
fn point_contact(cutter: &Cutter, x: f64, y: f64, point: Vec3) -> Option<f64> {
    let d = (point.x - x).hypot(point.y - y);
    let h = cutter.height(d)?;
    let z = point.z - h;
    if z.is_finite() { Some(z) } else { None }
}

/// Highest tip Z at which `cutter`, lowered along −Z at `(x, y)`, first touches `tri`.
///
/// `None` when nothing on the triangle is within the tool's radial reach. The value is the
/// maximum of vertex contacts, an upward facet contact, and one contact per edge.
pub fn drop_cutter_triangle(cutter: &Cutter, x: f64, y: f64, tri: &Triangle) -> Option<f64> {
    if !x.is_finite() || !y.is_finite() || cutter.profile().is_none() {
        return None;
    }
    let mut best = None;
    maximise(&mut best, point_contact(cutter, x, y, tri.a));
    maximise(&mut best, point_contact(cutter, x, y, tri.b));
    maximise(&mut best, point_contact(cutter, x, y, tri.c));
    maximise(&mut best, facet_contact(cutter, x, y, tri));
    maximise(&mut best, edge_contact(cutter, x, y, tri.a, tri.b));
    maximise(&mut best, edge_contact(cutter, x, y, tri.b, tri.c));
    maximise(&mut best, edge_contact(cutter, x, y, tri.c, tri.a));
    best
}

/// Edge contact. Flat and ball are closed form. Bull nose is a numeric 1D maximisation
/// ([`numeric_edge_tip`]); a torus against a line segment has no short radical used here.
fn edge_contact(cutter: &Cutter, x: f64, y: f64, a: Vec3, b: Vec3) -> Option<f64> {
    match *cutter {
        Cutter::Flat { radius } => flat_edge_tip(radius, x, y, a, b),
        Cutter::Ball { radius } => ball_edge_tip(radius, x, y, a, b),
        Cutter::Bull { .. } => numeric_edge_tip(cutter, x, y, a, b),
    }
}

/// Parameter interval of the segment whose XY distance to `(x, y)` is at most `radius`.
///
/// `d²(t)` along a straight segment is a convex parabola, so the feasible set is one interval.
fn segment_within_radius(x: f64, y: f64, a: Vec3, b: Vec3, radius: f64) -> Option<(f64, f64)> {
    if !finite_positive(radius) {
        return None;
    }
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let fx = a.x - x;
    let fy = a.y - y;
    let aa = dx * dx + dy * dy;
    let bb = 2.0 * (fx * dx + fy * dy);
    let cc = fx * fx + fy * fy - radius * radius;
    if aa <= 1e-24 {
        return if fx * fx + fy * fy <= radius * radius { Some((0.0, 1.0)) } else { None };
    }
    let disc = bb * bb - 4.0 * aa * cc;
    if disc < 0.0 {
        return None;
    }
    let root = disc.sqrt();
    let inv = 0.5 / aa;
    let mut t_lo = (-bb - root) * inv;
    let mut t_hi = (-bb + root) * inv;
    if t_lo > t_hi {
        std::mem::swap(&mut t_lo, &mut t_hi);
    }
    let lo = t_lo.max(0.0);
    let hi = t_hi.min(1.0);
    if lo <= hi { Some((lo, hi)) } else { None }
}

/// Flat edge: `h = 0` inside the disk, so the tip is the highest segment point within radius `R`.
/// Z is linear, so the maximum is at an endpoint of that clipped interval (a vertex, or where
/// the segment crosses the circle of radius `R`).
fn flat_edge_tip(radius: f64, x: f64, y: f64, a: Vec3, b: Vec3) -> Option<f64> {
    let (lo, hi) = segment_within_radius(x, y, a, b, radius)?;
    let z_lo = a.z + (b.z - a.z) * lo;
    let z_hi = a.z + (b.z - a.z) * hi;
    let z = z_lo.max(z_hi);
    if z.is_finite() { Some(z) } else { None }
}

/// Ball edge: critical points of `z(t) - h(d(t))` on the segment.
///
/// `d²` and `z` are quadratic and linear in the segment parameter. Setting the derivative to
/// zero and clearing the remaining square root yields a quadratic. Squaring adds the roots of
/// the negated derivative; those are rejected by substituting back.
fn ball_edge_tip(radius: f64, x: f64, y: f64, a: Vec3, b: Vec3) -> Option<f64> {
    if !finite_positive(radius) {
        return None;
    }
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let dz = b.z - a.z;
    let fx = a.x - x;
    let fy = a.y - y;
    let alpha = dx * dx + dy * dy;
    let beta = fx * dx + fy * dy;
    let gamma = fx * fx + fy * fy;
    let edge_sq = alpha + dz * dz;
    // A vertical edge has constant radial distance, so the higher vertex wins.
    if edge_sq <= 1e-24 || alpha <= 1e-16 * edge_sq.max(1.0) {
        return None;
    }
    let c_coef = (beta * beta - dz * dz * (radius * radius - gamma)) / edge_sq;
    let disc = 4.0 * beta * beta - 4.0 * alpha * c_coef;
    let disc_scale = alpha * radius * radius + beta * beta + 1.0;
    if disc < -1e-9 * disc_scale {
        return None;
    }
    let root = disc.max(0.0).sqrt();
    let mut best = None;
    for numer in [-2.0 * beta - root, -2.0 * beta + root] {
        let Some(t) = clamp_unit(numer / (2.0 * alpha)) else {
            continue;
        };
        maximise(&mut best, ball_edge_at(radius, a, dz, alpha, beta, gamma, t));
    }
    best
}

fn ball_edge_at(radius: f64, a: Vec3, dz: f64, alpha: f64, beta: f64, gamma: f64, t: f64) -> Option<f64> {
    let d2 = alpha * t * t + 2.0 * beta * t + gamma;
    let slack = 1e-8 * radius * radius + 1e-12;
    if d2 > radius * radius + slack {
        return None;
    }
    let s = (radius * radius - d2.max(0.0)).max(0.0).sqrt();
    // `dz * s` is the derivative condition after solving `g' = 0` for the positive square root.
    let side = alpha * t + beta;
    let target = dz * s;
    let tol = 1e-6 * (1.0 + radius + side.abs() + target.abs());
    if (side - target).abs() > tol {
        return None;
    }
    let z = a.z + t * dz;
    let tip = z - (radius - s);
    if tip.is_finite() { Some(tip) } else { None }
}

/// Samples along an edge used to seed the bull-nose golden-section search.
const EDGE_SAMPLES: usize = 64;

/// Absolute tolerance on the edge parameter `t` for [`numeric_edge_tip`].
const EDGE_T_TOL: f64 = 1e-9;

/// Bull-nose (torus) against a segment: numeric maximisation of `z(t) - h(d_xy(t))`.
///
/// There is no closed radical here. The feasible set `d_xy <= radius` is one interval because
/// `d²(t)` is convex. That interval is sampled at 64 uniform parameters. Each discrete local
/// maximum, and the best sample, is refined by golden-section search to [`EDGE_T_TOL`].
/// The same search cross-checks the flat and ball closed forms in tests.
fn numeric_edge_tip(cutter: &Cutter, x: f64, y: f64, a: Vec3, b: Vec3) -> Option<f64> {
    let radius = cutter.radius()?;
    let (lo, hi) = segment_within_radius(x, y, a, b, radius)?;
    if (hi - lo).abs() <= EDGE_T_TOL {
        return point_on_edge(cutter, x, y, a, b, lo);
    }
    let mut ts = [0.0; EDGE_SAMPLES];
    let mut zs = [f64::NEG_INFINITY; EDGE_SAMPLES];
    let mut best = f64::NEG_INFINITY;
    let steps = (EDGE_SAMPLES - 1) as f64;
    for i in 0..EDGE_SAMPLES {
        let t = lo + (hi - lo) * (i as f64) / steps;
        ts[i] = t;
        if let Some(z) = point_on_edge(cutter, x, y, a, b, t) {
            zs[i] = z;
            best = best.max(z);
        }
    }
    // The XY-closest point is often the mode when the edge passes near the axis. Seed it too.
    let near = closest_t_xy(x, y, a, b).clamp(lo, hi);
    if let Some(z) = point_on_edge(cutter, x, y, a, b, near) {
        best = best.max(z);
    }
    if !best.is_finite() {
        return None;
    }
    let eval = |t: f64| point_on_edge(cutter, x, y, a, b, t).unwrap_or(f64::NEG_INFINITY);
    let half = (hi - lo) / steps;
    best = best.max(golden_max((near - half).max(lo), (near + half).min(hi), eval));
    for i in 0..EDGE_SAMPLES {
        if !zs[i].is_finite() {
            continue;
        }
        let left = if i == 0 { zs[i] } else { zs[i - 1] };
        let right = if i + 1 == EDGE_SAMPLES { zs[i] } else { zs[i + 1] };
        let peak = zs[i] >= left && zs[i] >= right;
        if peak || zs[i] == best {
            let t_left = if i == 0 { ts[i] } else { ts[i - 1] };
            let t_right = if i + 1 == EDGE_SAMPLES { ts[i] } else { ts[i + 1] };
            best = best.max(golden_max(t_left, t_right, eval));
        }
    }
    if best.is_finite() { Some(best) } else { None }
}

fn point_on_edge(cutter: &Cutter, x: f64, y: f64, a: Vec3, b: Vec3, t: f64) -> Option<f64> {
    let point = Vec3::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
    point_contact(cutter, x, y, point)
}

fn closest_t_xy(x: f64, y: f64, a: Vec3, b: Vec3) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len2 = dx * dx + dy * dy;
    if len2 <= 1e-24 { 0.0 } else { (((x - a.x) * dx + (y - a.y) * dy) / len2).clamp(0.0, 1.0) }
}

/// Golden-section maximisation of a unimodal sample of `f` on `[a, b]`.
///
/// The bracket width stops at [`EDGE_T_TOL`]. The best value seen is returned, so a flat or
/// monotonic bracket still yields an endpoint sample.
fn golden_max(mut left: f64, mut right: f64, mut f: impl FnMut(f64) -> f64) -> f64 {
    if !(left < right) {
        return f(left);
    }
    let inv_phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut c = right - inv_phi * (right - left);
    let mut d = left + inv_phi * (right - left);
    let mut fc = f(c);
    let mut fd = f(d);
    let mut best = fc.max(fd);
    for _ in 0..80 {
        if right - left <= EDGE_T_TOL {
            break;
        }
        if fc > fd {
            right = d;
            d = c;
            fd = fc;
            c = right - inv_phi * (right - left);
            fc = f(c);
        } else {
            left = c;
            c = d;
            fc = fd;
            d = left + inv_phi * (right - left);
            fd = f(d);
        }
        best = best.max(fc).max(fd);
    }
    best.max(f(left)).max(f(right))
}

fn clamp_unit(t: f64) -> Option<f64> {
    if !t.is_finite() {
        return None;
    }
    if (0.0..=1.0).contains(&t) {
        Some(t)
    } else if t > -1e-8 && t < 0.0 {
        Some(0.0)
    } else if t > 1.0 && t < 1.0 + 1e-8 {
        Some(1.0)
    } else {
        None
    }
}

/// Facet contact: the tool touches the plane where its surface normal matches the upward
/// facet normal. Vertical facets are skipped (edges cover them). A stored downward normal is
/// reversed; matching that downward direction is the underside tangency, not a drop from above.
fn facet_contact(cutter: &Cutter, x: f64, y: f64, tri: &Triangle) -> Option<f64> {
    let (radius, corner) = cutter.profile()?;
    let mut normal = tri.normal()?;
    if normal.z < 0.0 {
        normal = normal.scale(-1.0);
    }
    if normal.z <= 0.0 {
        return None;
    }
    let nxy = normal.x.hypot(normal.y);
    let (cc_x, cc_y, radial) = if nxy <= 1e-15 {
        // Horizontal bottom: the tip (or the flat disk's centre) carries the upward normal.
        (x, y, 0.0)
    } else {
        // Contact sits uphill of the axis. Flat: on the rim. Ball: offset `R n_xy`.
        // Bull: flat radius `R - c` plus `c |n_xy|` along the corner.
        // `nxy` can exceed 1 by an ulp; the contact cannot sit outside the tool.
        let radial = ((radius - corner) + corner * nxy).min(radius);
        let scale = radial / nxy;
        (x - normal.x * scale, y - normal.y * scale, radial)
    };
    let (u, v, w) = barycentric_xy(tri, cc_x, cc_y)?;
    // Generous by a few ulps so a contact on an edge is not dropped between facet and edge tests.
    const EPS: f64 = 1e-8;
    if u < -EPS || v < -EPS || w < -EPS {
        return None;
    }
    let cc_z = u * tri.a.z + v * tri.b.z + w * tri.c.z;
    let h = cutter.height(radial)?;
    let tip = cc_z - h;
    if tip.is_finite() { Some(tip) } else { None }
}

/// Barycentric coordinates of `(x, y)` in the XY projection. `None` when the projection is degenerate.
fn barycentric_xy(tri: &Triangle, x: f64, y: f64) -> Option<(f64, f64, f64)> {
    let ax = tri.b.x - tri.a.x;
    let ay = tri.b.y - tri.a.y;
    let bx = tri.c.x - tri.a.x;
    let by = tri.c.y - tri.a.y;
    let denom = ax * by - ay * bx;
    let scale = ax.abs() + ay.abs() + bx.abs() + by.abs();
    if !denom.is_finite() || denom.abs() <= 1e-14 * scale * scale.max(1.0) {
        return None;
    }
    let px = x - tri.a.x;
    let py = y - tri.a.y;
    let v = (px * by - py * bx) / denom;
    let w = (py * ax - px * ay) / denom;
    if !v.is_finite() || !w.is_finite() {
        return None;
    }
    Some((1.0 - v - w, v, w))
}

/// Drop-cutter over a whole mesh.
///
/// Triangles are stored in a uniform XY grid. The cell size is the mean XY diagonal of the
/// triangle bounds, grown if needed so the grid has at most one million cells. A query tests
/// only the triangles whose bounds, expanded by the tool radius, overlap that query's cell.
#[derive(Debug)]
pub struct DropCutter<'a> {
    mesh: &'a Mesh,
    cutter: Cutter,
    origin_x: f64,
    origin_y: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    buckets: Vec<Vec<usize>>,
}

impl<'a> DropCutter<'a> {
    /// Index `mesh` for `cutter`. An empty mesh is allowed and every query misses.
    ///
    /// The cutter must have a finite positive radius. A bull corner must lie in `[0, radius]`
    /// (the open interval is what [`Cutter::new_bull`] accepts; the endpoints match flat and ball).
    pub fn new(mesh: &'a Mesh, cutter: Cutter) -> Result<Self> {
        let Some((radius, _)) = cutter.profile() else {
            return Err(invalid("cutter radius must be finite and positive, and a bull corner radius must lie in [0, radius]"));
        };
        if mesh.triangles().is_empty() {
            return Ok(Self { mesh, cutter, origin_x: 0.0, origin_y: 0.0, cell: 1.0, nx: 0, ny: 0, buckets: Vec::new() });
        }
        let bounds = mesh.bounds()?;
        let pad = radius * (1.0 + 1e-9) + 1e-8;
        let origin_x = bounds.min().x - pad;
        let origin_y = bounds.min().y - pad;
        let width = (bounds.max().x + pad - origin_x).max(0.0);
        let height = (bounds.max().y + pad - origin_y).max(0.0);
        let (mut nx, mut ny, mut cell) = bucket_layout(width, height, mean_xy_span(mesh));
        let count = match nx.checked_mul(ny) {
            Some(count) if count <= MAX_BUCKET_CELLS => count,
            _ => {
                nx = 1;
                ny = 1;
                cell = width.max(height).max(1.0);
                1
            }
        };
        let mut index = Self { mesh, cutter, origin_x, origin_y, cell, nx, ny, buckets: vec![Vec::new(); count] };
        index.fill_buckets(pad);
        Ok(index)
    }

    /// Highest tip Z at `(x, y)`, or `None` when the tool misses every triangle.
    pub fn tip_z(&self, x: f64, y: f64) -> Option<f64> {
        if !x.is_finite() || !y.is_finite() || self.nx == 0 || self.ny == 0 {
            return None;
        }
        let ix = cell_of(self.origin_x, self.cell, self.nx, x)?;
        let iy = cell_of(self.origin_y, self.cell, self.ny, y)?;
        let slot = iy.checked_mul(self.nx)?.checked_add(ix)?;
        let mut best = None;
        let bucket = self.buckets.get(slot)?;
        for &index in bucket {
            if let Some(tri) = self.mesh.triangles().get(index) {
                maximise(&mut best, drop_cutter_triangle(&self.cutter, x, y, tri));
            }
        }
        best
    }

    /// Cutter this index was built with.
    pub fn cutter(&self) -> Cutter {
        self.cutter
    }

    /// [`Self::tip_z`], or `floor` when the tool misses. A hit is the maximum of the tip and `floor`.
    pub fn tip_z_or(&self, x: f64, y: f64, floor: f64) -> f64 {
        match self.tip_z(x, y) {
            Some(z) => z.max(floor),
            None => floor,
        }
    }

    fn fill_buckets(&mut self, pad: f64) {
        for (index, tri) in self.mesh.triangles().iter().enumerate() {
            let bb = tri.bbox();
            let ix0 = cell_clamped(self.origin_x, self.cell, self.nx, bb.min().x - pad);
            let ix1 = cell_clamped(self.origin_x, self.cell, self.nx, bb.max().x + pad);
            let iy0 = cell_clamped(self.origin_y, self.cell, self.ny, bb.min().y - pad);
            let iy1 = cell_clamped(self.origin_y, self.cell, self.ny, bb.max().y + pad);
            for iy in iy0..=iy1 {
                for ix in ix0..=ix1 {
                    let Some(slot) = iy.checked_mul(self.nx).and_then(|row| row.checked_add(ix)) else {
                        continue;
                    };
                    if let Some(bucket) = self.buckets.get_mut(slot) {
                        bucket.push(index);
                    }
                }
            }
        }
    }
}

fn mean_xy_span(mesh: &Mesh) -> f64 {
    let tris = mesh.triangles();
    if tris.is_empty() {
        return 1.0;
    }
    let mut sum = 0.0;
    for tri in tris {
        let bb = tri.bbox();
        sum += (bb.max().x - bb.min().x).hypot(bb.max().y - bb.min().y);
    }
    let mean = sum / tris.len() as f64;
    if mean.is_finite() && mean > 0.0 { mean } else { 1.0 }
}

/// Cell size about `mean`, increased until `nx * ny` stays within [`MAX_BUCKET_CELLS`].
fn bucket_layout(width: f64, height: f64, mean: f64) -> (usize, usize, f64) {
    let mut cell = if mean.is_finite() && mean > 0.0 { mean } else { 1.0 };
    let mut nx = span_count(width, cell);
    let mut ny = span_count(height, cell);
    let mut guard = 0;
    while nx.saturating_mul(ny) > MAX_BUCKET_CELLS && guard < 60 {
        let product = (nx as f64) * (ny as f64);
        let factor = (product / MAX_BUCKET_CELLS as f64).sqrt().max(1.01);
        cell *= factor;
        nx = span_count(width, cell);
        ny = span_count(height, cell);
        guard += 1;
    }
    if nx.saturating_mul(ny) > MAX_BUCKET_CELLS { (1, 1, width.max(height).max(cell)) } else { (nx.max(1), ny.max(1), cell) }
}

fn span_count(span: f64, cell: f64) -> usize {
    if !(span.is_finite() && cell > 0.0) {
        return 1;
    }
    let n = (span / cell).ceil();
    if !n.is_finite() || n <= 1.0 {
        1
    } else if n >= MAX_BUCKET_CELLS as f64 {
        MAX_BUCKET_CELLS
    } else {
        n as usize
    }
}

fn cell_of(origin: f64, cell: f64, n: usize, value: f64) -> Option<usize> {
    if n == 0 || !(cell > 0.0) || !value.is_finite() {
        return None;
    }
    let t = (value - origin) / cell;
    if t < 0.0 {
        return None;
    }
    if t >= n as f64 { if t <= n as f64 * (1.0 + 1e-12) + 1e-9 { Some(n - 1) } else { None } } else { Some(t.floor() as usize) }
}

fn cell_clamped(origin: f64, cell: f64, n: usize, value: f64) -> usize {
    if n == 0 || value < origin { 0 } else { cell_of(origin, cell, n, value).unwrap_or(n - 1) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn tri(a: Vec3, b: Vec3, c: Vec3) -> Triangle {
        Triangle { a, b, c }
    }

    fn close(actual: f64, expect: f64, tol: f64) {
        assert!((actual - expect).abs() <= tol, "{actual} != {expect} ± {tol}");
    }

    #[test]
    fn constructors_reject_non_positive_and_bull_limits() {
        assert!(Cutter::new_flat(0.0).is_err());
        assert!(Cutter::new_flat(-2.0).is_err());
        assert!(Cutter::new_ball(f64::NAN).is_err());
        assert!(Cutter::new_ball(f64::INFINITY).is_err());
        assert!(Cutter::new_bull(2.0, 0.0).is_err());
        assert!(Cutter::new_bull(2.0, 2.0).is_err());
        assert!(Cutter::new_bull(2.0, -0.1).is_err());
        assert!(Cutter::new_bull(2.0, 2.1).is_err());
        assert!(Cutter::new_flat(1.5).is_ok());
        assert!(Cutter::new_ball(1.5).is_ok());
        assert!(Cutter::new_bull(2.0, 0.4).is_ok());
    }

    #[test]
    fn profile_height_matches_flat_ball_and_bull() {
        let flat = Cutter::new_flat(2.0).expect("flat");
        assert_eq!(flat.height(0.0), Some(0.0));
        assert_eq!(flat.height(2.0), Some(0.0));
        assert_eq!(flat.height(2.0001), None);
        let ball = Cutter::new_ball(2.0).expect("ball");
        let d = 1.2;
        close(ball.height(d).expect("in radius"), 2.0 - (4.0 - d * d).sqrt(), 1e-12);
        close(ball.height(0.0).expect("tip"), 0.0, 1e-15);
        close(ball.height(2.0).expect("equator"), 2.0, 1e-12);
        let bull = Cutter::new_bull(2.0, 0.5).expect("bull");
        assert_eq!(bull.height(1.5), Some(0.0));
        let u = 0.3;
        close(bull.height(1.8).expect("corner"), 0.5 - (0.25_f64 - u * u).sqrt(), 1e-12);
        // Endpoints of the bull family reproduce flat and ball.
        let as_ball = Cutter::Bull { radius: 2.0, corner_radius: 2.0 };
        let as_flat = Cutter::Bull { radius: 2.0, corner_radius: 0.0 };
        close(as_ball.height(d).expect("ball limit"), ball.height(d).expect("ball"), 1e-12);
        assert_eq!(as_flat.height(d), Some(0.0));
    }

    #[test]
    fn ball_on_a_horizontal_plane_sits_at_the_plane() {
        let plane = tri(Vec3::new(0.0, 0.0, 5.0), Vec3::new(8.0, 0.0, 5.0), Vec3::new(0.0, 8.0, 5.0));
        let ball = Cutter::new_ball(1.25).expect("ball");
        close(drop_cutter_triangle(&ball, 1.0, 1.5, &plane).expect("hit"), 5.0, 1e-12);
        assert!(drop_cutter_triangle(&ball, 40.0, 40.0, &plane).is_none());
        assert!(drop_cutter_triangle(&ball, f64::NAN, 1.0, &plane).is_none());
    }

    #[test]
    fn ball_above_a_vertex_follows_the_profile() {
        let peak = tri(Vec3::new(0.0, 0.0, 2.0), Vec3::new(40.0, 0.0, -80.0), Vec3::new(0.0, 40.0, -80.0));
        let ball = Cutter::new_ball(1.0).expect("ball");
        close(drop_cutter_triangle(&ball, 0.0, 0.0, &peak).expect("on vertex"), 2.0, 1e-12);
        let d = 0.6;
        let h = 1.0 - (1.0_f64 - d * d).sqrt();
        close(drop_cutter_triangle(&ball, d, 0.0, &peak).expect("offset"), 2.0 - h, 1e-9);
    }

    #[test]
    fn flat_on_a_45_degree_ramp_contacts_upslope() {
        // z = x, so tan(slope) = 1. Tip = plane Z at the axis + R.
        let ramp = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0));
        let flat = Cutter::new_flat(2.0).expect("flat");
        let tip = drop_cutter_triangle(&flat, 3.0, 1.0, &ramp).expect("ramp");
        close(tip, 3.0 + 2.0, 1e-9);
        let flat_small = Cutter::new_flat(1.5).expect("flat");
        close(drop_cutter_triangle(&flat_small, 4.0, 2.0, &ramp).expect("ramp"), 4.0 + 1.5, 1e-9);
    }

    #[test]
    fn bull_on_a_45_degree_ramp_matches_the_corner_offset() {
        let ramp = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0));
        let radius = 2.0;
        let corner = 0.5;
        let bull = Cutter::new_bull(radius, corner).expect("bull");
        let plane_z = 3.0;
        let nz = std::f64::consts::FRAC_1_SQRT_2;
        let expect = plane_z + (radius - corner) + corner * (1.0 / nz - 1.0);
        close(drop_cutter_triangle(&bull, 3.0, 1.0, &ramp).expect("bull ramp"), expect, 1e-9);
    }

    #[test]
    fn bull_limits_match_ball_and_flat() {
        let ramp = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 10.0), Vec3::new(0.0, 10.0, 0.0));
        let peak = tri(Vec3::new(0.0, 0.0, 2.0), Vec3::new(30.0, 1.0, -10.0), Vec3::new(-1.0, 25.0, -12.0));
        let radius = 1.7;
        let ball = Cutter::Ball { radius };
        let flat = Cutter::Flat { radius };
        let as_ball = Cutter::Bull { radius, corner_radius: radius };
        let as_flat = Cutter::Bull { radius, corner_radius: 0.0 };
        for (x, y, triangle) in [(2.5, 1.2, ramp), (0.4, 0.2, peak), (0.0, 0.0, peak)] {
            let ball_z = drop_cutter_triangle(&ball, x, y, &triangle);
            let flat_z = drop_cutter_triangle(&flat, x, y, &triangle);
            let bull_ball = drop_cutter_triangle(&as_ball, x, y, &triangle);
            let bull_flat = drop_cutter_triangle(&as_flat, x, y, &triangle);
            match (ball_z, bull_ball) {
                (Some(a), Some(b)) => close(a, b, 1e-8),
                (None, None) => {}
                other => panic!("ball limit disagreed: {other:?}"),
            }
            match (flat_z, bull_flat) {
                (Some(a), Some(b)) => close(a, b, 1e-8),
                (None, None) => {}
                other => panic!("flat limit disagreed: {other:?}"),
            }
        }
    }

    #[test]
    fn ball_edge_closed_form_matches_a_hand_derivative() {
        // Edge (0,0,0) → (2,0,2), tool radius 1 at the origin. Critical tip is √2 − 1.
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(2.0, 0.0, 2.0);
        let tip = ball_edge_tip(1.0, 0.0, 0.0, a, b).expect("edge");
        close(tip, std::f64::consts::SQRT_2 - 1.0, 1e-12);
    }

    fn edge_closed_form(cutter: &Cutter, x: f64, y: f64, a: Vec3, b: Vec3) -> Option<f64> {
        let mut best = None;
        maximise(&mut best, point_contact(cutter, x, y, a));
        maximise(&mut best, point_contact(cutter, x, y, b));
        maximise(&mut best, edge_contact(cutter, x, y, a, b));
        best
    }

    #[test]
    fn flat_and_ball_edges_match_the_numeric_search() {
        let edges = [
            (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 2.0)),
            (Vec3::new(-2.0, -1.0, 1.0), Vec3::new(3.0, 0.5, -0.5)),
            (Vec3::new(0.0, 4.0, 2.0), Vec3::new(0.2, -4.0, 2.5)),
            (Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 5.0)),
        ];
        let flat = Cutter::new_flat(1.5).expect("flat");
        let ball = Cutter::new_ball(1.5).expect("ball");
        for (a, b) in edges {
            for (x, y) in [(0.0, 0.0), (0.4, -0.3), (1.2, 0.8)] {
                for cutter in [flat, ball] {
                    let closed = edge_closed_form(&cutter, x, y, a, b);
                    let numeric = numeric_edge_tip(&cutter, x, y, a, b);
                    match (closed, numeric) {
                        (None, None) => {}
                        (Some(exact), Some(approx)) => close(approx, exact, 1e-6),
                        other => panic!("closed form and numeric search disagreed: {other:?}"),
                    }
                }
            }
        }
    }

    fn push_oriented(tris: &mut Vec<Triangle>, a: Vec3, b: Vec3, c: Vec3, hint: Vec3) {
        let cross = b.sub(a).cross(c.sub(a));
        if cross.dot(hint) >= 0.0 {
            tris.push(Triangle { a, b, c });
        } else {
            tris.push(Triangle { a, b: c, c: b });
        }
    }

    fn uv_sphere(radius: f64, stacks: usize, slices: usize) -> Mesh {
        let mut tris = Vec::new();
        let point = |phi: f64, theta: f64| Vec3::new(radius * phi.sin() * theta.cos(), radius * phi.sin() * theta.sin(), radius * phi.cos());
        for i in 0..stacks {
            let phi0 = std::f64::consts::PI * i as f64 / stacks as f64;
            let phi1 = std::f64::consts::PI * (i + 1) as f64 / stacks as f64;
            for j in 0..slices {
                let th0 = std::f64::consts::TAU * j as f64 / slices as f64;
                let th1 = std::f64::consts::TAU * (j + 1) as f64 / slices as f64;
                let a = point(phi0, th0);
                let b = point(phi1, th0);
                let c = point(phi0, th1);
                let d = point(phi1, th1);
                if i == 0 {
                    push_oriented(&mut tris, a, b, d, a.add(b).add(d));
                } else if i + 1 == stacks {
                    push_oriented(&mut tris, a, c, b, a.add(b).add(c));
                } else {
                    push_oriented(&mut tris, a, b, d, a.add(b).add(d));
                    push_oriented(&mut tris, a, d, c, a.add(d).add(c));
                }
            }
        }
        Mesh::new(tris).expect("sphere mesh")
    }

    fn cylinder_along_x(radius: f64, half_len: f64, rings: usize, slices: usize) -> Mesh {
        let mut tris = Vec::new();
        let point = |i: usize, j: usize| {
            let x = -half_len + 2.0 * half_len * i as f64 / rings as f64;
            let theta = std::f64::consts::TAU * j as f64 / slices as f64;
            Vec3::new(x, radius * theta.cos(), radius * theta.sin())
        };
        for i in 0..rings {
            for j in 0..slices {
                let a = point(i, j);
                let b = point(i, j + 1);
                let c = point(i + 1, j);
                let d = point(i + 1, j + 1);
                let hint = Vec3::new(0.0, a.y + b.y + d.y, a.z + b.z + d.z);
                push_oriented(&mut tris, a, b, d, hint);
                push_oriented(&mut tris, a, d, c, hint);
            }
        }
        Mesh::new(tris).expect("cylinder mesh")
    }

    fn full_scan(cutter: &Cutter, mesh: &Mesh, x: f64, y: f64) -> Option<f64> {
        let mut best = None;
        for triangle in mesh.triangles() {
            maximise(&mut best, drop_cutter_triangle(cutter, x, y, triangle));
        }
        best
    }

    #[test]
    fn ball_on_a_sphere_mesh_stays_within_tessellation_error() {
        let sphere_r = 10.0;
        let ball_r = 3.0;
        let stacks = 32;
        let slices = 64;
        let mesh = uv_sphere(sphere_r, stacks, slices);
        let cutter = Cutter::new_ball(ball_r).expect("ball");
        let angle = std::f64::consts::PI / stacks as f64;
        let tol = 4.0 * sphere_r * (1.0 - (angle * 0.5).cos()) + 1e-4;
        let index = DropCutter::new(&mesh, cutter).expect("index");
        for (x, y) in [(0.0_f64, 0.0_f64), (3.0, 0.0), (4.0, -2.0), (1.5, 2.5)] {
            let d = x.hypot(y);
            let analytic = ((sphere_r + ball_r).powi(2) - d * d).sqrt() - ball_r;
            let tip = index.tip_z(x, y).expect("sphere hit");
            let scan = full_scan(&cutter, &mesh, x, y).expect("scan");
            close(tip, scan, 1e-12);
            assert!(tip <= analytic + 1e-4, "mesh sticks out: {tip} > {analytic}");
            assert!(tip >= analytic - tol, "tessellation error {analytic} - {tip} > {tol}");
        }
    }

    #[test]
    fn ball_on_a_cylinder_along_x_matches_the_offset() {
        let cyl_r = 8.0;
        let ball_r = 2.0;
        let slices = 48;
        let mesh = cylinder_along_x(cyl_r, 20.0, 8, slices);
        let cutter = Cutter::new_ball(ball_r).expect("ball");
        let angle = std::f64::consts::TAU / slices as f64;
        let tol = 4.0 * cyl_r * (1.0 - (angle * 0.5).cos()) + 1e-4;
        let index = DropCutter::new(&mesh, cutter).expect("index");
        for (x, y) in [(0.0_f64, 0.0_f64), (1.0, 2.0), (-3.0, -1.0), (0.5, 3.0)] {
            let analytic = ((cyl_r + ball_r).powi(2) - y * y).sqrt() - ball_r;
            let tip = index.tip_z(x, y).expect("cylinder hit");
            close(tip, full_scan(&cutter, &mesh, x, y).expect("scan"), 1e-12);
            assert!(tip <= analytic + 1e-4, "mesh sticks out: {tip} > {analytic}");
            assert!(tip >= analytic - tol, "tessellation error {analytic} - {tip} > {tol}");
        }
    }

    #[test]
    fn index_reaches_separated_triangles_and_respects_the_floor() {
        let low = tri(Vec3::new(0.0, 0.0, 1.0), Vec3::new(2.0, 0.0, 1.0), Vec3::new(0.0, 2.0, 1.0));
        let high = tri(Vec3::new(100.0, 100.0, 5.0), Vec3::new(102.0, 100.0, 5.0), Vec3::new(100.0, 102.0, 5.0));
        let mesh = Mesh::new(vec![low, high]).expect("mesh");
        let cutter = Cutter::new_ball(1.0).expect("ball");
        let index = DropCutter::new(&mesh, cutter).expect("index");
        close(index.tip_z(0.4, 0.4).expect("low"), 1.0, 1e-12);
        close(index.tip_z(100.3, 100.2).expect("high"), 5.0, 1e-12);
        assert!(index.tip_z(50.0, 50.0).is_none());
        assert_eq!(index.tip_z_or(50.0, 50.0, 3.5), 3.5);
        close(index.tip_z_or(0.4, 0.4, -10.0), 1.0, 1e-12);
        close(index.tip_z_or(0.4, 0.4, 4.0), 4.0, 1e-12);
        let empty = Mesh::new(vec![]).expect("empty");
        let empty_index = DropCutter::new(&empty, cutter).expect("empty index");
        assert!(empty_index.tip_z(0.0, 0.0).is_none());
        assert_eq!(empty_index.tip_z_or(0.0, 0.0, 1.25), 1.25);
        assert!(DropCutter::new(&mesh, Cutter::Flat { radius: 0.0 }).is_err());
        assert!(DropCutter::new(&mesh, Cutter::Bull { radius: 1.0, corner_radius: 2.0 }).is_err());
    }

    /// Barycentric 60×60 grid plus the three edges. This is a lower bound on the true drop.
    fn each_sample(tri: &Triangle, mut visit: impl FnMut(Vec3)) {
        const N: usize = 60;
        let n = N as f64;
        for i in 0..=N {
            for j in 0..=(N - i) {
                let u = i as f64 / n;
                let v = j as f64 / n;
                let w = 1.0 - u - v;
                visit(Vec3::new(
                    w * tri.a.x + u * tri.b.x + v * tri.c.x,
                    w * tri.a.y + u * tri.b.y + v * tri.c.y,
                    w * tri.a.z + u * tri.b.z + v * tri.c.z,
                ));
            }
        }
        for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
            for k in 0..=N {
                let t = k as f64 / n;
                visit(Vec3::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t));
            }
        }
    }

    fn sampled_lower_bound(cutter: &Cutter, x: f64, y: f64, tri: &Triangle) -> Option<f64> {
        let mut best = None;
        each_sample(tri, |point| maximise(&mut best, point_contact(cutter, x, y, point)));
        best
    }

    /// Positive when a sample rises above the tool surface at `tip`.
    fn sample_penetration(cutter: &Cutter, x: f64, y: f64, tri: &Triangle, tip: f64) -> f64 {
        let mut worst = 0.0_f64;
        each_sample(tri, |point| {
            let d = (point.x - x).hypot(point.y - y);
            if let Some(h) = cutter.height(d) {
                worst = worst.max(point.z - (tip + h));
            }
        });
        worst
    }

    fn cutter_from_kind(kind: u8, radius: f64, corner_frac: f64) -> Cutter {
        match kind {
            0 => Cutter::new_flat(radius).expect("flat"),
            1 => Cutter::new_ball(radius).expect("ball"),
            _ => Cutter::new_bull(radius, radius * corner_frac).expect("bull"),
        }
    }
    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        #[test]
        fn random_triangle_matches_sampling_and_does_not_gouge(
            ax in -1.5f64..1.5,
            ay in -1.5f64..1.5,
            az in -1.5f64..1.5,
            bx in -1.5f64..1.5,
            by in -1.5f64..1.5,
            bz in -1.5f64..1.5,
            cx in -1.5f64..1.5,
            cy in -1.5f64..1.5,
            cz in -1.5f64..1.5,
            x in -2.5f64..2.5,
            y in -2.5f64..2.5,
            radius in 0.6f64..2.5,
            kind in 0..3u8,
            corner_frac in 0.15f64..0.85,
        ) {
            let triangle = Triangle {
                a: Vec3::new(ax, ay, az),
                b: Vec3::new(bx, by, bz),
                c: Vec3::new(cx, cy, cz),
            };
            prop_assume!(triangle.normal().is_some());
            let cutter = cutter_from_kind(kind, radius, corner_frac);
            let result = drop_cutter_triangle(&cutter, x, y, &triangle);
            let brute = sampled_lower_bound(&cutter, x, y, &triangle);
            match (result, brute) {
                (None, None) => {}
                (None, Some(lower)) => prop_assert!(false, "missed a sampled contact at {lower}"),
                (Some(_), None) => {}
                (Some(got), Some(lower)) => {
                    prop_assert!(got >= lower - 1e-9, "below samples: got {got}, samples {lower}");
                    prop_assert!(got <= lower + 0.05 * radius, "above samples: got {got}, samples {lower}, radius {radius}");
                }
            }
            if let Some(tip) = result {
                let penetration = sample_penetration(&cutter, x, y, &triangle, tip);
                prop_assert!(penetration <= 1e-7, "gouge {penetration} at tip {tip}");
            }
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(32))]

        #[test]
        fn buckets_match_the_loop_over_all_triangles(
            coords in proptest::collection::vec(
                (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0),
                1..5,
            ),
            queries in proptest::collection::vec((-5.0f64..5.0, -5.0f64..5.0), 1..6),
            radius in 0.5f64..2.0,
            kind in 0..3u8,
            corner_frac in 0.2f64..0.8,
        ) {
            let triangles = coords
                .into_iter()
                .map(|(ax, ay, az, bx, by, bz, cx, cy, cz)| Triangle {
                    a: Vec3::new(ax, ay, az),
                    b: Vec3::new(bx, by, bz),
                    c: Vec3::new(cx, cy, cz),
                })
                .collect();
            let mesh = Mesh::new(triangles).expect("finite");
            let cutter = cutter_from_kind(kind, radius, corner_frac);
            let index = DropCutter::new(&mesh, cutter).expect("index");
            for (x, y) in queries {
                let fast = index.tip_z(x, y);
                let slow = full_scan(&cutter, &mesh, x, y);
                match (fast, slow) {
                    (None, None) => {}
                    (Some(a), Some(b)) => prop_assert!((a - b).abs() <= 1e-12, "bucket {a} scan {b}"),
                    other => prop_assert!(false, "bucket and full scan disagreed: {other:?}"),
                }
            }
        }
    }
}
