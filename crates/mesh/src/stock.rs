//! In-process stock as a multi-interval Z-dexel.
//!
//! The grid is regular in XY. Each cell holds a sorted list of disjoint solid Z intervals, so a
//! tilted tool can leave an undercut instead of filling the column back in. [`StockModel::from_box`]
//! is the build used now. A revolved profile is not built here.
//!
//! What is exact, per vertical dexel ray:
//! - a sphere, any centre;
//! - a finite cylinder, any orientation (the ray interval inside the solid, from the infinite
//!   cylinder's quadratic clipped to the end-cap slab);
//! - the swept solid of that cylinder for a straight translation with a constant axis. The swept
//!   body is convex, so the ray hits one interval. Its ends are the real roots of the overlap
//!   conditions (quadratics in Z), confirmed by the closed-form `exists t in [0, 1]` predicate and
//!   polished by bisection.
//!
//! What is sampled:
//! - a move whose tool axis changes, and any arc or helix: the flute is subtracted only at the
//!   collision-check samples (tip step and axis step). There is no chord sweep;
//! - the ball sphere and the bull torus between those samples. On a straight move with a constant
//!   axis the cylinders are the exact sweep above, and the sphere or torus is still only at the
//!   samples, so a long ball cut can leave a scallop between samples on the nose.
//!
//! A Z-dexel has no extra resolution on a steep wall: a wall between two cell centres moves by a
//! whole cell. Edge cells extend past the stock box, and the dexel sits at the cell centre, which
//! may fall just outside the box. Callers that compare a hit to a tolerance should allow one cell
//! of that quantisation.
//!
//! The removed volume of a call is the union of the Z lengths cut from the columns, times the cell
//! area. Overlapping bodies in one call are not counted twice.
//!
//! [`StockModel::measure_flute`] estimates the radial engagement and the axial depth from the
//! dexels the flute meets, before a subtraction. The angle is the share of the leading half of
//! the cutter circumference that still lies in solid stock at the flute's lowest active Z.

use crate::stock_map::StockGrid;
use crate::{Aabb, MAX_TRIANGLES, Mesh, Result, Triangle, Vec3, invalid};

/// Most XY cells in one stock model.
///
/// A larger grid is rejected with a message that names a larger `stock_model.cell`.
pub const MAX_STOCK_CELLS: usize = 16_000_000;

/// Most disjoint solid intervals stored in one column.
pub const MAX_COLUMN_INTERVALS: usize = 4096;

/// Lower clamp for the default cell size, in millimetres. An explicit cell is not clamped.
pub const STOCK_CELL_MIN_MM: f64 = 0.05;

/// Upper clamp for the default cell size, in millimetres. An explicit cell is not clamped.
pub const STOCK_CELL_MAX_MM: f64 = 2.0;

/// Spans shorter than this, in millimetres, are dropped after a subtraction.
const MIN_SPAN_MM: f64 = 1e-9;

/// A solid run of stock along one dexel, in millimetres. `lo <= hi`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZInterval {
    pub lo: f64,
    pub hi: f64,
}

/// Deepest overlap of a query body with the remaining solid.
///
/// `depth` is the longest Z overlap on any dexel the body reaches, in millimetres. It is not a
/// Euclidean penetration. `point` is that cell's centre at the middle of the overlap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StockHit {
    pub point: Vec3,
    pub depth: f64,
}

/// Leftover stock above an STL surface, and cells cut too far below it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LeftoverReport {
    /// Thickest remaining solid sitting on the surface, in millimetres.
    pub max_leftover: f64,
    /// Mean of that thickness over cells whose centre hits the mesh.
    pub mean_leftover: f64,
    /// Cells whose stock was removed below the surface by more than the allowance.
    pub model_gouges: u64,
    /// Cell centres that hit the mesh.
    pub samples: u64,
}

/// Cutter flute used when a pose is subtracted or queried.
///
/// Lengths are millimetres. The cylinder runs from the tip along the tool axis for `length`
/// (a ball starts its cylinder one radius up). A bull adds the toroidal corner. The torus is
/// subtracted at a pose; it is not part of the exact swept solid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Flute {
    /// Sphere of `radius` centred one radius above the tip, plus a cylinder above that.
    Ball { radius: f64, length: f64 },
    /// Cylinder of `radius` from the tip for `length`.
    Flat { radius: f64, length: f64 },
    /// Inner cylinder, outer cylinder above the corner, and a torus of minor radius `corner`.
    Bull { radius: f64, corner: f64, length: f64 },
}

/// Radial engagement and axial depth of one flute pose, read from the dexels before a cut.
///
/// `engagement_deg` is the arc of the cutter circumference in contact, in degrees. It is the
/// share of the leading half (the side facing the XY feed) that still holds stock at the lowest
/// Z the flute meets, times 180. A plunge, with no XY feed, uses the whole circle and 360. A
/// slot through solid stock is 180. A straight bite of radial depth `s` into an existing wall is
/// `acos(1 - 2s/D)`, where `D` is the diameter.
///
/// The sample sits on the circumference and the wall sits on a dexel, so both are about one cell
/// off. `axial_doc` is the longest Z overlap of the flute with solid stock, in millimetres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CutMeasure {
    /// Contact arc, degrees.
    pub engagement_deg: f64,
    /// Axial depth of cut, millimetres.
    pub axial_doc: f64,
}

/// Default cell: `smallest_radius / 4`, clamped to [[`STOCK_CELL_MIN_MM`], [`STOCK_CELL_MAX_MM`]].
///
/// A missing or non-positive radius uses the upper clamp.
#[must_use]
pub fn default_stock_cell(smallest_radius: f64) -> f64 {
    if !smallest_radius.is_finite() || smallest_radius <= 0.0 {
        return STOCK_CELL_MAX_MM;
    }
    (smallest_radius / 4.0).clamp(STOCK_CELL_MIN_MM, STOCK_CELL_MAX_MM)
}

/// Z interval of the vertical ray `(x, y)` inside the sphere. `None` when the ray misses.
#[must_use]
pub fn ray_sphere_interval(x: f64, y: f64, center: Vec3, radius: f64) -> Option<(f64, f64)> {
    if !x.is_finite() || !y.is_finite() || !finite_vec(center) || !radius.is_finite() || radius < 0.0 {
        return None;
    }
    let dx = x - center.x;
    let dy = y - center.y;
    let radial = radius * radius - dx * dx - dy * dy;
    if radial < 0.0 {
        return None;
    }
    let half = radial.sqrt();
    if !half.is_finite() {
        return None;
    }
    Some((center.z - half, center.z + half))
}

/// Z interval of the vertical ray `(x, y)` inside a finite cylinder. `a` and `b` are the cap centres.
///
/// Exact for any orientation. A zero-length axis returns `None`.
#[must_use]
pub fn ray_cylinder_interval(x: f64, y: f64, a: Vec3, b: Vec3, radius: f64) -> Option<(f64, f64)> {
    if !x.is_finite() || !y.is_finite() || !finite_vec(a) || !finite_vec(b) || !radius.is_finite() || radius < 0.0 {
        return None;
    }
    let axis = b.sub(a);
    let length = axis.length();
    if !length.is_finite() || length <= 1e-15 {
        return None;
    }
    let u = axis.scale(1.0 / length);
    cylinder_interval(x, y, a, u, length, radius)
}

/// Z interval of the vertical ray inside the solid swept by translating a finite cylinder.
///
/// `axis` is the unit tool axis (it is normalised here). `length` is the cylinder length.
/// `motion` is the translation of every point. A zero motion is the cylinder itself. Motion
/// parallel to the axis is one longer cylinder. Any other motion uses the exact convex overlap.
#[must_use]
pub fn ray_swept_cylinder_interval(x: f64, y: f64, base: Vec3, axis: Vec3, length: f64, radius: f64, motion: Vec3) -> Option<(f64, f64)> {
    if !x.is_finite()
        || !y.is_finite()
        || !finite_vec(base)
        || !finite_vec(motion)
        || !length.is_finite()
        || length <= 1e-15
        || !radius.is_finite()
        || radius < 0.0
    {
        return None;
    }
    let axis = axis.normalized()?;
    if motion.length() <= 1e-12 {
        return cylinder_interval(x, y, base, axis, length, radius);
    }
    let vu = motion.dot(axis);
    let normal = motion.sub(axis.scale(vu));
    if normal.length_sq() <= 1e-18 {
        let lo = vu.min(0.0);
        let hi = length + vu.max(0.0);
        return cylinder_interval(x, y, base.add(axis.scale(lo)), axis, hi - lo, radius);
    }
    let sweep = Sweep::new(base, axis, length, radius, motion, normal, vu);
    sweep.interval(x, y)
}

/// Multi-interval Z-dexel stock.
#[derive(Clone, Debug)]
pub struct StockModel {
    origin_x: f64,
    origin_y: f64,
    z_min: f64,
    z_max: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    columns: Vec<Vec<ZInterval>>,
}

impl StockModel {
    /// Solid box. Every cell whose rectangle overlaps the box starts as `[min.z, max.z]`.
    ///
    /// `cell` must be finite and positive. The cell count is `ceil(width / cell) * ceil(height / cell)`,
    /// at least 1 by 1, and at most [`MAX_STOCK_CELLS`].
    pub fn from_box(min: Vec3, max: Vec3, cell: f64) -> Result<Self> {
        if !finite_vec(min) || !finite_vec(max) {
            return Err(invalid("stock box coordinate is not finite"));
        }
        if !(max.x > min.x && max.y > min.y && max.z > min.z) {
            return Err(invalid("stock box max must be greater than min"));
        }
        if !cell.is_finite() || cell <= 0.0 {
            return Err(invalid("stock cell must be finite and greater than 0"));
        }
        let nx = cell_count(max.x - min.x, cell)?;
        let ny = cell_count(max.y - min.y, cell)?;
        let count = nx.checked_mul(ny).ok_or_else(|| cell_limit_error(usize::MAX))?;
        if count > MAX_STOCK_CELLS {
            return Err(cell_limit_error(count));
        }
        let column = vec![ZInterval { lo: min.z, hi: max.z }];
        Ok(Self { origin_x: min.x, origin_y: min.y, z_min: min.z, z_max: max.z, cell, nx, ny, columns: vec![column; count] })
    }

    /// Cell size in millimetres.
    #[must_use]
    pub fn cell(&self) -> f64 {
        self.cell
    }

    /// Grid width and height, in cells.
    #[must_use]
    pub fn dimensions(&self) -> (usize, usize) {
        (self.nx, self.ny)
    }

    /// Original box Z range. Every column started as this interval.
    #[must_use]
    pub fn z_bounds(&self) -> (f64, f64) {
        (self.z_min, self.z_max)
    }

    /// Cell centre in XY. Z is 0. `None` when the index is outside the grid.
    #[must_use]
    pub fn cell_center(&self, ix: usize, iy: usize) -> Option<Vec3> {
        if ix >= self.nx || iy >= self.ny {
            return None;
        }
        Some(Vec3::new(self.center_x(ix), self.center_y(iy), 0.0))
    }

    /// Solid intervals in one column, low to high. `None` when the index is outside the grid.
    #[must_use]
    pub fn column(&self, ix: usize, iy: usize) -> Option<&[ZInterval]> {
        if ix >= self.nx || iy >= self.ny {
            return None;
        }
        self.columns.get(self.index(ix, iy)).map(Vec::as_slice)
    }

    /// Highest solid Z in the column. `None` when the column is empty or the index is outside.
    #[must_use]
    pub fn height_at(&self, ix: usize, iy: usize) -> Option<f64> {
        self.column(ix, iy).and_then(|spans| spans.last().map(|span| span.hi))
    }

    /// Top Z of every column, row-major (`iy * nx + ix`). An empty column is `None`.
    #[must_use]
    pub fn height_map(&self) -> Vec<Option<f64>> {
        self.columns.iter().map(|spans| spans.last().map(|span| span.hi)).collect()
    }

    /// Height map plus the columns that hold more than one interval.
    #[must_use]
    pub fn stock_grid(&self) -> StockGrid {
        let mut top = Vec::with_capacity(self.columns.len());
        let mut multi = Vec::new();
        for (index, column) in self.columns.iter().enumerate() {
            top.push(column.last().filter(|span| span.hi.is_finite()).map(|span| span.hi));
            if column.len() > 1 {
                multi.push(index);
            }
        }
        StockGrid { origin: [self.origin_x, self.origin_y], cell: self.cell, nx: self.nx, ny: self.ny, top, multi }
    }

    /// Engagement and axial depth of the flute at one pose, from the stock still there.
    ///
    /// `feed` is the tool-tip displacement of the move, in millimetres. A zero XY feed is a
    /// plunge. A flute that misses the stock measures zero.
    #[must_use]
    pub fn measure_flute(&self, tip: Vec3, axis: Vec3, flute: Flute, feed: Vec3) -> CutMeasure {
        let none = CutMeasure { engagement_deg: 0.0, axial_doc: 0.0 };
        if check_flute(flute).is_err() || !finite_vec(tip) || !finite_vec(feed) {
            return none;
        }
        let Some(axis) = axis.normalized() else {
            return none;
        };
        let Some(stats) = self.flute_contact(tip, axis, flute) else {
            return none;
        };
        if !(stats.axial > 1e-9) || !(stats.radius > 1e-9) {
            return CutMeasure { engagement_deg: 0.0, axial_doc: stats.axial.max(0.0) };
        }
        let engagement = self.contact_angle(stats, feed);
        CutMeasure { engagement_deg: engagement, axial_doc: stats.axial }
    }

    /// True when the column that contains `(x, y)` is solid at `z`.
    #[must_use]
    pub fn covers(&self, x: f64, y: f64, z: f64) -> bool {
        let Some(ix) = self.containing(self.origin_x, x, self.nx) else {
            return false;
        };
        let Some(iy) = self.containing(self.origin_y, y, self.ny) else {
            return false;
        };
        self.column(ix, iy).is_some_and(|spans| spans.iter().any(|span| span.lo <= z && z <= span.hi))
    }

    /// Two triangles on the top of every solid interval, in the cell square, normal +Z.
    ///
    /// More than [`MAX_TRIANGLES`] triangles is an error.
    pub fn top_surface(&self) -> Result<Mesh> {
        let mut triangles = Vec::new();
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let Some(spans) = self.column(ix, iy) else {
                    continue;
                };
                let x0 = self.origin_x + ix as f64 * self.cell;
                let y0 = self.origin_y + iy as f64 * self.cell;
                let x1 = x0 + self.cell;
                let y1 = y0 + self.cell;
                for span in spans {
                    let z = span.hi;
                    triangles.push(Triangle { a: Vec3::new(x0, y0, z), b: Vec3::new(x1, y0, z), c: Vec3::new(x1, y1, z) });
                    triangles.push(Triangle { a: Vec3::new(x0, y0, z), b: Vec3::new(x1, y1, z), c: Vec3::new(x0, y1, z) });
                    if triangles.len() > MAX_TRIANGLES {
                        return Err(invalid(format!("too many triangles: {} > {MAX_TRIANGLES}", triangles.len())));
                    }
                }
            }
        }
        Mesh::new(triangles)
    }

    /// Remove a sphere. The returned volume is the stock that was solid and is not, in mm³.
    pub fn subtract_sphere(&mut self, center: Vec3, radius: f64) -> Result<f64> {
        if !finite_vec(center) || !radius.is_finite() || radius < 0.0 {
            return Err(invalid("sphere is not usable"));
        }
        if radius == 0.0 {
            return Ok(0.0);
        }
        let bounds = Aabb::from_corners(center, center).expand(radius);
        self.subtract_bounds(bounds, |x, y| ray_sphere_interval(x, y, center, radius))
    }

    /// Remove a finite cylinder. `a` and `b` are the cap centres.
    pub fn subtract_cylinder(&mut self, a: Vec3, b: Vec3, radius: f64) -> Result<f64> {
        if !finite_vec(a) || !finite_vec(b) || !radius.is_finite() || radius < 0.0 {
            return Err(invalid("cylinder is not usable"));
        }
        if a.sub(b).length() <= 1e-15 || radius == 0.0 {
            return Ok(0.0);
        }
        let bounds = Aabb::from_corners(a, b).expand(radius);
        self.subtract_bounds(bounds, |x, y| ray_cylinder_interval(x, y, a, b, radius))
    }

    /// Remove the exact swept solid of one finite cylinder. `axis` need not be a unit vector.
    pub fn subtract_swept_cylinder(&mut self, base: Vec3, axis: Vec3, length: f64, radius: f64, motion: Vec3) -> Result<f64> {
        if !finite_vec(base) || !finite_vec(axis) || !finite_vec(motion) || !length.is_finite() || length < 0.0 || !radius.is_finite() || radius < 0.0
        {
            return Err(invalid("swept cylinder is not usable"));
        }
        let Some(axis) = axis.normalized() else {
            return Err(invalid("swept cylinder axis is not usable"));
        };
        if length <= 1e-15 || radius == 0.0 {
            return Ok(0.0);
        }
        let top = base.add(axis.scale(length));
        let end = base.add(motion);
        let end_top = top.add(motion);
        let bounds = Aabb::from_corners(base, top).expand(radius).union(Aabb::from_corners(end, end_top).expand(radius));
        self.subtract_bounds(bounds, |x, y| ray_swept_cylinder_interval(x, y, base, axis, length, radius, motion))
    }

    /// Remove the flute body at one pose. `axis` points from the tip toward the spindle.
    pub fn subtract_flute(&mut self, tip: Vec3, axis: Vec3, flute: Flute) -> Result<f64> {
        let axis = unit_axis(axis)?;
        check_flute(flute)?;
        let mut volume = 0.0;
        for (a, b, radius) in flute_cylinders(tip, axis, flute) {
            volume += self.subtract_cylinder(a, b, radius)?;
        }
        if let Some((center, radius)) = flute_sphere(tip, axis, flute) {
            volume += self.subtract_sphere(center, radius)?;
        }
        if let Some(torus) = flute_torus(tip, axis, flute) {
            volume += self.subtract_torus(torus)?;
        }
        Ok(volume)
    }

    fn subtract_torus(&mut self, torus: Torus) -> Result<f64> {
        let reach = torus.major + torus.minor;
        let bounds = Aabb::from_corners(torus.center, torus.center).expand(reach);
        let (ix0, ix1, iy0, iy1) = self.cell_span(bounds.min().x, bounds.max().x, bounds.min().y, bounds.max().y);
        let area = self.cell * self.cell;
        let mut volume = 0.0;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let spans = torus_intervals(self.center_x(ix), self.center_y(iy), &torus);
                let index = self.index(ix, iy);
                let Some(column) = self.columns.get_mut(index) else {
                    continue;
                };
                for (lo, hi) in spans {
                    volume += subtract_column(column, lo, hi)? * area;
                }
            }
        }
        Ok(volume)
    }

    /// Remove only the ball sphere or the bull torus at one pose.
    ///
    /// Used with [`Self::subtract_flute_translation`] so the nose is sampled and the cylinders
    /// are the exact sweep.
    pub fn subtract_flute_nose(&mut self, tip: Vec3, axis: Vec3, flute: Flute) -> Result<f64> {
        let axis = unit_axis(axis)?;
        check_flute(flute)?;
        let mut volume = 0.0;
        if let Some((center, radius)) = flute_sphere(tip, axis, flute) {
            volume += self.subtract_sphere(center, radius)?;
        }
        if let Some(torus) = flute_torus(tip, axis, flute) {
            volume += self.subtract_torus(torus)?;
        }
        Ok(volume)
    }

    /// Exact swept cylinders of a straight move with a constant axis. The nose is not included.
    pub fn subtract_flute_translation(&mut self, tip0: Vec3, tip1: Vec3, axis: Vec3, flute: Flute) -> Result<f64> {
        let axis = unit_axis(axis)?;
        check_flute(flute)?;
        if !finite_vec(tip0) || !finite_vec(tip1) {
            return Err(invalid("flute move is not usable"));
        }
        let motion = tip1.sub(tip0);
        let mut volume = 0.0;
        for (a, b, radius) in flute_cylinders(tip0, axis, flute) {
            let length = b.sub(a).length();
            volume += self.subtract_swept_cylinder(a, axis, length, radius, motion)?;
        }
        Ok(volume)
    }

    /// Deepest Z overlap of a sphere with the remaining solid.
    #[must_use]
    pub fn query_sphere(&self, center: Vec3, radius: f64) -> Option<StockHit> {
        if !finite_vec(center) || !radius.is_finite() || radius < 0.0 {
            return None;
        }
        let bounds = Aabb::from_corners(center, center).expand(radius);
        self.query_bounds(bounds, |x, y| ray_sphere_interval(x, y, center, radius))
    }

    /// Deepest Z overlap of a finite cylinder with the remaining solid.
    #[must_use]
    pub fn query_cylinder(&self, a: Vec3, b: Vec3, radius: f64) -> Option<StockHit> {
        if !finite_vec(a) || !finite_vec(b) || !radius.is_finite() || radius < 0.0 {
            return None;
        }
        if a.sub(b).length() <= 1e-15 {
            return None;
        }
        let bounds = Aabb::from_corners(a, b).expand(radius);
        self.query_bounds(bounds, |x, y| ray_cylinder_interval(x, y, a, b, radius))
    }

    /// Deepest Z overlap of the flute body with the remaining solid.
    #[must_use]
    pub fn query_flute(&self, tip: Vec3, axis: Vec3, flute: Flute) -> Option<StockHit> {
        let axis = axis.normalized()?;
        if check_flute(flute).is_err() || !finite_vec(tip) {
            return None;
        }
        let mut best: Option<StockHit> = None;
        for (a, b, radius) in flute_cylinders(tip, axis, flute) {
            take_hit(&mut best, self.query_cylinder(a, b, radius));
        }
        if let Some((center, radius)) = flute_sphere(tip, axis, flute) {
            take_hit(&mut best, self.query_sphere(center, radius));
        }
        if let Some(torus) = flute_torus(tip, axis, flute) {
            take_hit(&mut best, self.query_torus(torus));
        }
        best
    }

    /// Compare the remaining stock with the part mesh at each cell centre.
    ///
    /// The part surface is the highest non-vertical triangle hit. When a solid interval covers
    /// that Z, the leftover is the top of that interval minus the surface. Otherwise the leftover
    /// is 0. A model gouge is stock removed below the surface, measured from the surface down to
    /// the next remaining solid (or the original box floor), deeper than `gouge_allowance`.
    pub fn compare_mesh(&self, mesh: &Mesh, gouge_allowance: f64) -> LeftoverReport {
        let allowance = if gouge_allowance.is_finite() && gouge_allowance > 0.0 { gouge_allowance } else { 0.0 };
        let mut buckets = vec![Vec::new(); self.nx * self.ny];
        for (index, triangle) in mesh.triangles().iter().enumerate() {
            if triangle_xy_det(*triangle).abs() < 1e-14 {
                continue;
            }
            let bbox = triangle.bbox();
            let (ix0, ix1, iy0, iy1) = self.cell_span(bbox.min().x, bbox.max().x, bbox.min().y, bbox.max().y);
            for iy in iy0..=iy1 {
                for ix in ix0..=ix1 {
                    if let Some(bucket) = buckets.get_mut(self.index(ix, iy)) {
                        bucket.push(index);
                    }
                }
            }
        }
        let mut max_leftover = 0.0_f64;
        let mut sum = 0.0_f64;
        let mut samples = 0_u64;
        let mut model_gouges = 0_u64;
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let x = self.center_x(ix);
                let y = self.center_y(iy);
                let Some(bucket) = buckets.get(self.index(ix, iy)) else {
                    continue;
                };
                let Some(surface_z) = highest_hit(mesh, bucket, x, y) else {
                    continue;
                };
                let Some(spans) = self.column(ix, iy) else {
                    continue;
                };
                let leftover = leftover_above(spans, surface_z);
                let gouge = gouge_below(spans, surface_z, self.z_min);
                max_leftover = max_leftover.max(leftover);
                sum += leftover;
                samples += 1;
                if gouge > allowance {
                    model_gouges += 1;
                }
            }
        }
        let mean_leftover = if samples == 0 { 0.0 } else { sum / samples as f64 };
        LeftoverReport { max_leftover, mean_leftover, model_gouges, samples }
    }

    fn subtract_bounds(&mut self, bounds: Aabb, mut cut: impl FnMut(f64, f64) -> Option<(f64, f64)>) -> Result<f64> {
        let (ix0, ix1, iy0, iy1) = self.cell_span(bounds.min().x, bounds.max().x, bounds.min().y, bounds.max().y);
        let area = self.cell * self.cell;
        let mut volume = 0.0;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let Some((lo, hi)) = cut(self.center_x(ix), self.center_y(iy)) else {
                    continue;
                };
                let index = self.index(ix, iy);
                let Some(column) = self.columns.get_mut(index) else {
                    continue;
                };
                volume += subtract_column(column, lo, hi)? * area;
            }
        }
        Ok(volume)
    }

    fn query_bounds(&self, bounds: Aabb, mut cut: impl FnMut(f64, f64) -> Option<(f64, f64)>) -> Option<StockHit> {
        let (ix0, ix1, iy0, iy1) = self.cell_span(bounds.min().x, bounds.max().x, bounds.min().y, bounds.max().y);
        let mut best: Option<StockHit> = None;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let x = self.center_x(ix);
                let y = self.center_y(iy);
                let Some((lo, hi)) = cut(x, y) else {
                    continue;
                };
                let Some(spans) = self.column(ix, iy) else {
                    continue;
                };
                if let Some((depth, mid)) = deepest_overlap(spans, lo, hi) {
                    take_hit(&mut best, Some(StockHit { point: Vec3::new(x, y, mid), depth }));
                }
            }
        }
        best
    }

    fn query_torus(&self, torus: Torus) -> Option<StockHit> {
        let reach = torus.major + torus.minor;
        let bounds = Aabb::from_corners(torus.center, torus.center).expand(reach);
        // A torus can contribute two intervals. Query each and keep the deeper one.
        let (ix0, ix1, iy0, iy1) = self.cell_span(bounds.min().x, bounds.max().x, bounds.min().y, bounds.max().y);
        let mut best: Option<StockHit> = None;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let x = self.center_x(ix);
                let y = self.center_y(iy);
                let Some(spans) = self.column(ix, iy) else {
                    continue;
                };
                for (lo, hi) in torus_intervals(x, y, &torus) {
                    if let Some((depth, mid)) = deepest_overlap(spans, lo, hi) {
                        take_hit(&mut best, Some(StockHit { point: Vec3::new(x, y, mid), depth }));
                    }
                }
            }
        }
        best
    }

    fn flute_contact(&self, tip: Vec3, axis: Vec3, flute: Flute) -> Option<FluteContact> {
        let bounds = flute_bounds(tip, axis, flute);
        let (ix0, ix1, iy0, iy1) = self.cell_span(bounds.min().x, bounds.max().x, bounds.min().y, bounds.max().y);
        let mut axial = 0.0_f64;
        let mut lowest: Option<f64> = None;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let Some(solid) = self.column(ix, iy) else {
                    continue;
                };
                if solid.is_empty() {
                    continue;
                }
                let cut = flute_z_intervals(self.center_x(ix), self.center_y(iy), tip, axis, flute);
                let (length, low) = overlap_stats(solid, &cut);
                if length > axial {
                    axial = length;
                }
                if let Some(low) = low {
                    lowest = Some(lowest.map_or(low, |current| current.min(low)));
                }
            }
        }
        let floor = lowest?;
        // The overlap starts on the floor, and that plane is still solid. Sit just above it so
        // the circumference sample sees the wall, not the floor. Stay inside the overlap.
        let lift = (axial * 0.25).clamp(1e-9, 1e-4);
        let z = floor + lift;
        let radius = section_radius(tip, axis, flute, z);
        let center = pierce_xy(tip, axis, z)?;
        Some(FluteContact { center, radius, z, axial })
    }

    /// Share of the circumference, on the leading side of `feed`, whose dexel is still solid.
    fn contact_angle(&self, stats: FluteContact, feed: Vec3) -> f64 {
        let radius = stats.radius;
        let around = std::f64::consts::TAU * radius / self.cell;
        let samples = if around.is_finite() { (around.ceil() as usize).clamp(72, 720) } else { 72 };
        let horizontal = feed.x.hypot(feed.y);
        let plunge = !(horizontal > 1e-6);
        let mut engaged = 0usize;
        let mut considered = 0usize;
        let steps = samples as f64;
        for index in 0..samples {
            let theta = std::f64::consts::TAU * index as f64 / steps;
            let dx = theta.cos();
            let dy = theta.sin();
            if !plunge && dx * feed.x + dy * feed.y < 0.0 {
                continue;
            }
            considered += 1;
            let x = stats.center[0] + radius * dx;
            let y = stats.center[1] + radius * dy;
            if self.covers(x, y, stats.z) {
                engaged += 1;
            }
        }
        if considered == 0 {
            return 0.0;
        }
        let span = if plunge { 360.0 } else { 180.0 };
        engaged as f64 / considered as f64 * span
    }

    fn containing(&self, origin: f64, coord: f64, count: usize) -> Option<usize> {
        if !coord.is_finite() || count == 0 || !self.cell.is_finite() || self.cell <= 0.0 {
            return None;
        }
        let raw = ((coord - origin) / self.cell).floor();
        if !raw.is_finite() {
            return None;
        }
        if raw >= 0.0 && (raw as usize) < count {
            return Some(raw as usize);
        }
        let far = origin + count as f64 * self.cell;
        if (coord - far).abs() <= self.cell * 1e-9 { Some(count - 1) } else { None }
    }

    fn index(&self, ix: usize, iy: usize) -> usize {
        iy * self.nx + ix
    }

    fn center_x(&self, ix: usize) -> f64 {
        self.origin_x + (ix as f64 + 0.5) * self.cell
    }

    fn center_y(&self, iy: usize) -> f64 {
        self.origin_y + (iy as f64 + 0.5) * self.cell
    }

    /// Inclusive cell range whose squares overlap the XY rectangle. Empty ranges collapse to a
    /// valid empty iteration by returning `1, 0`.
    fn cell_span(&self, min_x: f64, max_x: f64, min_y: f64, max_y: f64) -> (usize, usize, usize, usize) {
        let ix0 = self.cell_floor(self.origin_x, min_x, self.nx);
        let ix1 = self.cell_ceil(self.origin_x, max_x, self.nx);
        let iy0 = self.cell_floor(self.origin_y, min_y, self.ny);
        let iy1 = self.cell_ceil(self.origin_y, max_y, self.ny);
        (ix0, ix1, iy0, iy1)
    }

    fn cell_floor(&self, origin: f64, coord: f64, count: usize) -> usize {
        if !coord.is_finite() || count == 0 {
            return 0;
        }
        let raw = ((coord - origin) / self.cell).floor();
        if raw <= 0.0 {
            0
        } else if raw >= count as f64 {
            count.saturating_sub(1)
        } else {
            raw as usize
        }
    }

    fn cell_ceil(&self, origin: f64, coord: f64, count: usize) -> usize {
        if !coord.is_finite() || count == 0 {
            return 0;
        }
        // The cell that contains `coord`, inclusive of the left edge of the next cell so a body
        // that ends on a grid line still visits the cell it touches.
        let raw = ((coord - origin) / self.cell - 1e-12).floor();
        if raw < 0.0 {
            0
        } else if raw >= count as f64 {
            count.saturating_sub(1)
        } else {
            raw as usize
        }
    }
}

fn cell_count(span: f64, cell: f64) -> Result<usize> {
    let count = (span / cell).ceil();
    if !count.is_finite() || count < 1.0 || count > MAX_STOCK_CELLS as f64 {
        let reported = if count.is_finite() && count > 0.0 { count as usize } else { usize::MAX };
        return Err(cell_limit_error(reported));
    }
    Ok(count as usize)
}

fn cell_limit_error(count: usize) -> crate::MeshError {
    invalid(format!("stock model has {count} cells, above the limit of {MAX_STOCK_CELLS}; use a larger stock_model.cell"))
}

fn finite_vec(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn unit_axis(axis: Vec3) -> Result<Vec3> {
    axis.normalized().ok_or_else(|| invalid("tool axis is not usable"))
}

fn check_flute(flute: Flute) -> Result<()> {
    let (radius, length, corner) = match flute {
        Flute::Ball { radius, length } | Flute::Flat { radius, length } => (radius, length, 0.0),
        Flute::Bull { radius, corner, length } => (radius, length, corner),
    };
    if !radius.is_finite() || radius < 0.0 || !length.is_finite() || length < 0.0 || !corner.is_finite() || corner < 0.0 || corner > radius + 1e-9 {
        return Err(invalid("flute is not usable"));
    }
    Ok(())
}

fn flute_cylinders(tip: Vec3, axis: Vec3, flute: Flute) -> Vec<(Vec3, Vec3, f64)> {
    let mut out = Vec::new();
    match flute {
        Flute::Ball { radius, length } => {
            if length > radius + 1e-12 && radius > 0.0 {
                out.push((tip.add(axis.scale(radius)), tip.add(axis.scale(length)), radius));
            }
        }
        Flute::Flat { radius, length } => {
            if length > 1e-12 && radius > 0.0 {
                out.push((tip, tip.add(axis.scale(length)), radius));
            }
        }
        Flute::Bull { radius, corner, length } => {
            let inner = radius - corner;
            if length > 1e-12 && inner > 1e-12 {
                out.push((tip, tip.add(axis.scale(length)), inner));
            }
            if length > corner + 1e-12 && radius > 0.0 {
                out.push((tip.add(axis.scale(corner)), tip.add(axis.scale(length)), radius));
            }
        }
    }
    out
}

fn flute_sphere(tip: Vec3, axis: Vec3, flute: Flute) -> Option<(Vec3, f64)> {
    match flute {
        Flute::Ball { radius, .. } if radius > 0.0 => Some((tip.add(axis.scale(radius)), radius)),
        _ => None,
    }
}

#[derive(Clone, Copy)]
struct FluteContact {
    center: [f64; 2],
    radius: f64,
    z: f64,
    axial: f64,
}

fn flute_bounds(tip: Vec3, axis: Vec3, flute: Flute) -> Aabb {
    let mut bounds = Aabb::from_corners(tip, tip);
    for (a, b, radius) in flute_cylinders(tip, axis, flute) {
        bounds = bounds.union(Aabb::from_corners(a, b).expand(radius));
    }
    if let Some((center, radius)) = flute_sphere(tip, axis, flute) {
        bounds = bounds.union(Aabb::from_corners(center, center).expand(radius));
    }
    if let Some(torus) = flute_torus(tip, axis, flute) {
        let reach = torus.major + torus.minor;
        bounds = bounds.union(Aabb::from_corners(torus.center, torus.center).expand(reach));
    }
    bounds
}

fn flute_z_intervals(x: f64, y: f64, tip: Vec3, axis: Vec3, flute: Flute) -> Vec<(f64, f64)> {
    let mut spans = Vec::new();
    for (a, b, radius) in flute_cylinders(tip, axis, flute) {
        if let Some(interval) = ray_cylinder_interval(x, y, a, b, radius) {
            spans.push(interval);
        }
    }
    if let Some((center, radius)) = flute_sphere(tip, axis, flute)
        && let Some(interval) = ray_sphere_interval(x, y, center, radius)
    {
        spans.push(interval);
    }
    if let Some(torus) = flute_torus(tip, axis, flute) {
        spans.extend(torus_intervals(x, y, &torus));
    }
    merge_spans(&mut spans);
    spans
}

fn overlap_stats(solid: &[ZInterval], cut: &[(f64, f64)]) -> (f64, Option<f64>) {
    let mut total = 0.0;
    let mut lowest: Option<f64> = None;
    for (lo, hi) in cut {
        for span in solid {
            let a = span.lo.max(*lo);
            let b = span.hi.min(*hi);
            if b > a {
                total += b - a;
                lowest = Some(lowest.map_or(a, |current| current.min(a)));
            }
        }
    }
    (total, lowest)
}

fn pierce_xy(tip: Vec3, axis: Vec3, z: f64) -> Option<[f64; 2]> {
    if axis.z.abs() < 1e-8 {
        return Some([tip.x, tip.y]);
    }
    let t = (z - tip.z) / axis.z;
    if !t.is_finite() {
        return None;
    }
    let point = tip.add(axis.scale(t));
    if point.x.is_finite() && point.y.is_finite() { Some([point.x, point.y]) } else { None }
}

/// Horizontal section radius of the flute at world `z`. A vertical flat is the tool radius.
fn section_radius(tip: Vec3, axis: Vec3, flute: Flute, z: f64) -> f64 {
    let mut best = 0.0_f64;
    for (a, b, radius) in flute_cylinders(tip, axis, flute) {
        best = best.max(cylinder_section_radius(a, b, radius, z));
    }
    if let Some((center, radius)) = flute_sphere(tip, axis, flute) {
        let dz = z - center.z;
        let radial = radius * radius - dz * dz;
        if radial > 0.0 {
            best = best.max(radial.sqrt());
        }
    }
    if let Some(torus) = flute_torus(tip, axis, flute) {
        best = best.max(torus_section_radius(&torus, z));
    }
    if best.is_finite() { best } else { 0.0 }
}

fn cylinder_section_radius(a: Vec3, b: Vec3, radius: f64, z: f64) -> f64 {
    let axis = b.sub(a);
    let length = axis.length();
    if !length.is_finite() || length <= 1e-15 || !radius.is_finite() || radius <= 0.0 {
        return 0.0;
    }
    let u = axis.scale(1.0 / length);
    if u.z.abs() >= 0.999 {
        let z0 = a.z.min(b.z);
        let z1 = a.z.max(b.z);
        if z >= z0 - 1e-9 && z <= z1 + 1e-9 { radius } else { 0.0 }
    } else {
        let lean = (1.0 - u.z * u.z).sqrt();
        let z0 = a.z.min(b.z) - radius * lean;
        let z1 = a.z.max(b.z) + radius * lean;
        if z >= z0 && z <= z1 { radius } else { 0.0 }
    }
}

fn torus_section_radius(torus: &Torus, z: f64) -> f64 {
    if torus.axis.z.abs() < 0.999 {
        return torus.major + torus.minor;
    }
    let dz = z - torus.center.z;
    if dz.abs() > torus.minor {
        return 0.0;
    }
    let tube = torus.minor * torus.minor - dz * dz;
    if tube > 0.0 { torus.major + tube.sqrt() } else { torus.major }
}

fn flute_torus(tip: Vec3, axis: Vec3, flute: Flute) -> Option<Torus> {
    let Flute::Bull { radius, corner, .. } = flute else {
        return None;
    };
    if corner <= 1e-12 || radius <= corner + 1e-12 {
        // A full corner is the ball sphere, which is not this path. A zero corner is a flat.
        return None;
    }
    Some(Torus { center: tip.add(axis.scale(corner)), axis, major: radius - corner, minor: corner })
}

struct Torus {
    center: Vec3,
    axis: Vec3,
    major: f64,
    minor: f64,
}

fn torus_contains(x: f64, y: f64, z: f64, torus: &Torus) -> bool {
    let delta = (x - torus.center.x) * torus.axis.x + (y - torus.center.y) * torus.axis.y + (z - torus.center.z) * torus.axis.z;
    let rx = x - torus.center.x - delta * torus.axis.x;
    let ry = y - torus.center.y - delta * torus.axis.y;
    let rz = z - torus.center.z - delta * torus.axis.z;
    let rho = (rx * rx + ry * ry + rz * rz).sqrt();
    let radial = rho - torus.major;
    radial * radial + delta * delta <= torus.minor * torus.minor + 1e-9
}

/// Up to two Z spans. A vertical ray meets a torus in at most four points.
fn torus_intervals(x: f64, y: f64, torus: &Torus) -> Vec<(f64, f64)> {
    if torus.minor <= 1e-12 {
        return Vec::new();
    }
    if torus.major <= 1e-9 {
        return ray_sphere_interval(x, y, torus.center, torus.minor).into_iter().collect();
    }
    let reach = torus.major + torus.minor;
    // Reject rays that cannot reach the tube in XY before walking Z.
    let dx = x - torus.center.x;
    let dy = y - torus.center.y;
    // A vertical ray outside the bounding sphere misses the torus.
    if dx * dx + dy * dy > reach * reach + 1e-6 {
        return Vec::new();
    }
    let z0 = torus.center.z - reach;
    let z1 = torus.center.z + reach;
    let step = (torus.minor / 8.0).max((z1 - z0) / 512.0);
    if !(step.is_finite() && step > 0.0) {
        return Vec::new();
    }
    let mut spans = Vec::new();
    let mut z = z0;
    let mut guard = 0u32;
    while z < z1 && guard < 4096 {
        guard += 1;
        let next = (z + step).min(z1);
        push_torus_step(&mut spans, x, y, torus, z, next);
        z = next;
    }
    merge_spans(&mut spans);
    spans
}

fn push_torus_step(spans: &mut Vec<(f64, f64)>, x: f64, y: f64, torus: &Torus, z0: f64, z1: f64) {
    let a = torus_contains(x, y, z0, torus);
    let b = torus_contains(x, y, z1, torus);
    let mid = 0.5 * (z0 + z1);
    let m = torus_contains(x, y, mid, torus);
    if a && b && m {
        spans.push((z0, z1));
        return;
    }
    if !a && !b && !m {
        return;
    }
    if !a && !b && m {
        let enter = bisect_torus(x, y, torus, z0, mid, false);
        let exit = bisect_torus(x, y, torus, mid, z1, true);
        spans.push((enter, exit));
        return;
    }
    if a && !b {
        spans.push((z0, bisect_torus(x, y, torus, z0, z1, true)));
    } else if !a && b {
        spans.push((bisect_torus(x, y, torus, z0, z1, false), z1));
    } else if a && b && !m {
        // Left the tube and came back inside one step.
        let exit = bisect_torus(x, y, torus, z0, mid, true);
        let enter = bisect_torus(x, y, torus, mid, z1, false);
        spans.push((z0, exit));
        spans.push((enter, z1));
    }
}

fn bisect_torus(x: f64, y: f64, torus: &Torus, mut lo: f64, mut hi: f64, leaving: bool) -> f64 {
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        let inside = torus_contains(x, y, mid, torus);
        if leaving {
            if inside { lo = mid } else { hi = mid }
        } else if inside {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    if leaving { lo } else { hi }
}

fn merge_spans(spans: &mut Vec<(f64, f64)>) {
    if spans.is_empty() {
        return;
    }
    spans.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut merged = Vec::with_capacity(spans.len());
    let mut cur = spans[0];
    for span in spans.iter().skip(1) {
        if span.0 <= cur.1 + 1e-6 {
            cur.1 = cur.1.max(span.1);
        } else {
            merged.push(cur);
            cur = *span;
        }
    }
    merged.push(cur);
    *spans = merged;
}

fn subtract_column(column: &mut Vec<ZInterval>, lo: f64, hi: f64) -> Result<f64> {
    if !(hi > lo) {
        return Ok(0.0);
    }
    let mut removed = 0.0;
    let mut next = Vec::with_capacity(column.len() + 1);
    for span in column.iter().copied() {
        if span.hi <= lo || span.lo >= hi {
            push_span(&mut next, span.lo, span.hi);
            continue;
        }
        let overlap_lo = span.lo.max(lo);
        let overlap_hi = span.hi.min(hi);
        if overlap_hi > overlap_lo {
            removed += overlap_hi - overlap_lo;
        }
        if span.lo < lo {
            push_span(&mut next, span.lo, lo.min(span.hi));
        }
        if span.hi > hi {
            push_span(&mut next, hi.max(span.lo), span.hi);
        }
    }
    compact_spans(&mut next);
    if next.len() > MAX_COLUMN_INTERVALS {
        return Err(invalid(format!("stock column has more than {MAX_COLUMN_INTERVALS} solid intervals")));
    }
    *column = next;
    Ok(removed)
}

fn push_span(spans: &mut Vec<ZInterval>, lo: f64, hi: f64) {
    if hi - lo > MIN_SPAN_MM {
        spans.push(ZInterval { lo, hi });
    }
}

fn compact_spans(spans: &mut Vec<ZInterval>) {
    if spans.len() < 2 {
        return;
    }
    let mut compact = Vec::with_capacity(spans.len());
    let mut cur = spans[0];
    for span in spans.iter().skip(1) {
        if span.lo <= cur.hi + 1e-12 {
            cur.hi = cur.hi.max(span.hi);
        } else {
            compact.push(cur);
            cur = *span;
        }
    }
    compact.push(cur);
    *spans = compact;
}

fn deepest_overlap(spans: &[ZInterval], lo: f64, hi: f64) -> Option<(f64, f64)> {
    let mut best: Option<(f64, f64)> = None;
    for span in spans {
        let a = span.lo.max(lo);
        let b = span.hi.min(hi);
        let depth = b - a;
        if depth > 0.0 && best.is_none_or(|(current, _)| depth > current) {
            best = Some((depth, 0.5 * (a + b)));
        }
    }
    best
}

fn take_hit(best: &mut Option<StockHit>, hit: Option<StockHit>) {
    let Some(hit) = hit else {
        return;
    };
    if !hit.depth.is_finite() || !finite_vec(hit.point) {
        return;
    }
    match *best {
        Some(current) if current.depth >= hit.depth => {}
        _ => *best = Some(hit),
    }
}

fn leftover_above(spans: &[ZInterval], surface_z: f64) -> f64 {
    spans.iter().find(|span| span.lo <= surface_z + 1e-9 && span.hi >= surface_z - 1e-9).map(|span| (span.hi - surface_z).max(0.0)).unwrap_or(0.0)
}

fn gouge_below(spans: &[ZInterval], surface_z: f64, z_min: f64) -> f64 {
    if !(surface_z > z_min) {
        return 0.0;
    }
    for span in spans {
        if span.lo <= surface_z + 1e-9 && span.hi >= surface_z - 1e-9 && span.lo < surface_z - 1e-12 {
            return 0.0;
        }
    }
    let mut cover = z_min;
    for span in spans {
        if span.hi <= surface_z + 1e-9 {
            cover = cover.max(span.hi);
        }
    }
    (surface_z - cover).max(0.0)
}

fn triangle_xy_det(triangle: Triangle) -> f64 {
    let abx = triangle.b.x - triangle.a.x;
    let aby = triangle.b.y - triangle.a.y;
    let acx = triangle.c.x - triangle.a.x;
    let acy = triangle.c.y - triangle.a.y;
    abx * acy - aby * acx
}

fn highest_hit(mesh: &Mesh, bucket: &[usize], x: f64, y: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    for index in bucket {
        let Some(triangle) = mesh.triangles().get(*index) else {
            continue;
        };
        if let Some(z) = ray_triangle_z(*triangle, x, y) {
            best = Some(best.map_or(z, |current| current.max(z)));
        }
    }
    best
}

fn ray_triangle_z(triangle: Triangle, x: f64, y: f64) -> Option<f64> {
    let denom = triangle_xy_det(triangle);
    if denom.abs() < 1e-14 {
        return None;
    }
    let a = triangle.a;
    let b = triangle.b;
    let c = triangle.c;
    let alpha = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / denom;
    let beta = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / denom;
    let gamma = 1.0 - alpha - beta;
    if alpha < -1e-9 || beta < -1e-9 || gamma < -1e-9 {
        return None;
    }
    let z = alpha * a.z + beta * b.z + gamma * c.z;
    if z.is_finite() { Some(z) } else { None }
}

#[derive(Clone, Copy)]
enum Range {
    Empty,
    All,
    Interval(f64, f64),
}

fn range_intersect(left: Range, right: Range) -> Range {
    match (left, right) {
        (Range::Empty, _) | (_, Range::Empty) => Range::Empty,
        (Range::All, other) | (other, Range::All) => other,
        (Range::Interval(a0, a1), Range::Interval(b0, b1)) => {
            let lo = a0.max(b0);
            let hi = a1.min(b1);
            if hi < lo { Range::Empty } else { Range::Interval(lo, hi) }
        }
    }
}

fn range_pair(range: Range) -> Option<(f64, f64)> {
    match range {
        Range::Interval(lo, hi) if hi >= lo && lo.is_finite() && hi.is_finite() => Some((lo, hi)),
        _ => None,
    }
}

fn cylinder_interval(x: f64, y: f64, base: Vec3, axis: Vec3, length: f64, radius: f64) -> Option<(f64, f64)> {
    let dx = x - base.x;
    let dy = y - base.y;
    let s_const = dx * axis.x + dy * axis.y - base.z * axis.z;
    let uz = axis.z;
    let a = 1.0 - uz * uz;
    let b = -2.0 * base.z - 2.0 * s_const * uz;
    let c = dx * dx + dy * dy + base.z * base.z - s_const * s_const - radius * radius;
    let radial = quadratic_le(a, b, c);
    let slab = axial_slab(s_const, uz, length);
    range_pair(range_intersect(radial, slab))
}

fn axial_slab(s_const: f64, uz: f64, length: f64) -> Range {
    if uz.abs() <= 1e-15 {
        return if (-1e-8..=length + 1e-8).contains(&s_const) { Range::All } else { Range::Empty };
    }
    let t0 = -s_const / uz;
    let t1 = (length - s_const) / uz;
    Range::Interval(t0.min(t1), t0.max(t1))
}

/// Solution of `a t^2 + b t + c <= 0`.
fn quadratic_le(a: f64, b: f64, c: f64) -> Range {
    const A_EPS: f64 = 1e-12;
    const B_EPS: f64 = 1e-12;
    if a.abs() <= A_EPS {
        if b.abs() <= B_EPS {
            return if c <= 1e-9 { Range::All } else { Range::Empty };
        }
        let t = -c / b;
        return if b > 0.0 { Range::Interval(f64::NEG_INFINITY, t) } else { Range::Interval(t, f64::INFINITY) };
    }
    let disc = b * b - 4.0 * a * c;
    let scale = b * b + a.abs() * c.abs() + 1.0;
    if disc < 0.0 {
        if disc > -1e-9 * scale {
            let t = -b / (2.0 * a);
            return Range::Interval(t, t);
        }
        return if a > 0.0 { Range::Empty } else { Range::All };
    }
    let root = disc.sqrt();
    let inv = 0.5 / a;
    let t0 = (-b - root) * inv;
    let t1 = (-b + root) * inv;
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    // `a < 0` would be the complement. Cylinder `a` is `1 - uz²`, so it is non-negative.
    Range::Interval(lo, hi)
}

struct Sweep {
    base: Vec3,
    axis: Vec3,
    length: f64,
    radius: f64,
    normal: Vec3,
    vu: f64,
    vn2: f64,
    z_lo: f64,
    z_hi: f64,
}

impl Sweep {
    fn new(base: Vec3, axis: Vec3, length: f64, radius: f64, motion: Vec3, normal: Vec3, vu: f64) -> Self {
        let top = base.add(axis.scale(length));
        let end = base.add(motion);
        let end_top = top.add(motion);
        let bounds = Aabb::from_corners(base, top).expand(radius).union(Aabb::from_corners(end, end_top).expand(radius));
        Self { base, axis, length, radius, normal, vu, vn2: normal.length_sq(), z_lo: bounds.min().z, z_hi: bounds.max().z }
    }

    fn interval(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        if self.vn2 <= 1e-18 {
            return None;
        }
        let mut roots = [0.0; 96];
        let mut count = 0usize;
        let pad = 1e-4_f64.max(self.radius * 1e-9);
        let bound_lo = self.z_lo - pad;
        let bound_hi = self.z_hi + pad;
        push_root(&mut roots, &mut count, bound_lo);
        push_root(&mut roots, &mut count, bound_hi);
        self.push_analytic_roots(x, y, &mut roots, &mut count);
        let steps = 32u32;
        for step in 0..=steps {
            let z = bound_lo + (bound_hi - bound_lo) * f64::from(step) / f64::from(steps);
            push_root(&mut roots, &mut count, z);
        }
        if count == 0 {
            return None;
        }
        roots[..count].sort_unstable_by(|p, q| p.partial_cmp(q).unwrap_or(std::cmp::Ordering::Equal));
        count = dedup_roots(&mut roots, count);
        let mut inside_lo: Option<f64> = None;
        let mut inside_hi: Option<f64> = None;
        let mut note = |z: f64| {
            if self.contains(x, y, z) {
                inside_lo = Some(inside_lo.map_or(z, |lo| lo.min(z)));
                inside_hi = Some(inside_hi.map_or(z, |hi| hi.max(z)));
            }
        };
        for index in 0..count {
            note(roots[index]);
            if index + 1 < count {
                note(0.5 * (roots[index] + roots[index + 1]));
            }
        }
        let (Some(lo), Some(hi)) = (inside_lo, inside_hi) else {
            return None;
        };
        let outside_lo = bound_lo.min(lo - pad);
        let outside_hi = bound_hi.max(hi + pad);
        let lo = if self.contains(x, y, outside_lo) { outside_lo } else { bisect(|z| self.contains(x, y, z), outside_lo, lo, true) };
        let hi = if self.contains(x, y, outside_hi) { outside_hi } else { bisect(|z| self.contains(x, y, z), hi, outside_hi, false) };
        if hi + 1e-12 >= lo { Some((lo, hi)) } else { None }
    }

    fn contains(&self, x: f64, y: f64, z: f64) -> bool {
        let (p_dot, q) = self.radial_terms(x, y, z);
        let disc = p_dot * p_dot - self.vn2 * q;
        let scale = self.vn2 * self.radius * self.radius + 1.0;
        if disc < -1e-8 * scale {
            return false;
        }
        let root = disc.max(0.0).sqrt();
        let r0 = (p_dot - root) / self.vn2;
        let r1 = (p_dot + root) / self.vn2;
        let (a0, a1) = self.axial_t(x, y, z);
        let lo = r0.max(a0).max(0.0);
        let hi = r1.min(a1).min(1.0);
        lo <= hi + 1e-9
    }

    fn radial_terms(&self, x: f64, y: f64, z: f64) -> (f64, f64) {
        let w = Vec3::new(x - self.base.x, y - self.base.y, z - self.base.z);
        let alpha = w.dot(self.axis);
        let n = w.sub(self.axis.scale(alpha));
        let p_dot = n.dot(self.normal);
        let q = n.length_sq() - self.radius * self.radius;
        (p_dot, q)
    }

    fn axial_t(&self, x: f64, y: f64, z: f64) -> (f64, f64) {
        let alpha = (x - self.base.x) * self.axis.x + (y - self.base.y) * self.axis.y + (z - self.base.z) * self.axis.z;
        if self.vu.abs() <= 1e-12 {
            return if (0.0..=self.length).contains(&alpha) { (0.0, 1.0) } else { (1.0, 0.0) };
        }
        let t_at = |s: f64| (alpha - s) / self.vu;
        let t0 = t_at(0.0);
        let t1 = t_at(self.length);
        (t0.min(t1), t0.max(t1))
    }

    fn push_analytic_roots(&self, x: f64, y: f64, roots: &mut [f64; 96], count: &mut usize) {
        // Coefficients of N(z) = N0 + z * nz, p = N·Vn, q = |N|^2 - R^2.
        let (q2, q1, q0, p1, p0, alpha1, alpha0) = self.polynomials(x, y);
        // D = p^2 - vn2 q = 0.
        let d2 = p1 * p1 - self.vn2 * q2;
        let d1 = 2.0 * p0 * p1 - self.vn2 * q1;
        let d0 = p0 * p0 - self.vn2 * q0;
        push_quadratic_roots(roots, count, d2, d1, d0);
        // q = 0: a radial endpoint at t = 0.
        push_quadratic_roots(roots, count, q2, q1, q0);
        // q - 2 p + vn2 = 0: a radial endpoint at t = 1.
        push_quadratic_roots(roots, count, q2, q1 - 2.0 * p1, q0 - 2.0 * p0 + self.vn2);
        if self.vu.abs() <= 1e-12 {
            // Axial window is a Z slab, independent of t.
            if alpha1.abs() <= 1e-15 {
                return;
            }
            push_root(roots, count, (0.0 - alpha0) / alpha1);
            push_root(roots, count, (self.length - alpha0) / alpha1);
            return;
        }
        let inv = 1.0 / self.vu;
        let lo_c = (alpha0 - self.length) * inv;
        let lo_s = alpha1 * inv;
        let hi_c = alpha0 * inv;
        let hi_s = alpha1 * inv;
        let (a0c, a0s, a1c, a1s) = if self.vu > 0.0 { (lo_c, lo_s, hi_c, hi_s) } else { (hi_c, hi_s, lo_c, lo_s) };
        self.push_radial_meets(roots, count, q2, q1, q0, p1, p0, a0c, a0s);
        self.push_radial_meets(roots, count, q2, q1, q0, p1, p0, a1c, a1s);
        for (c, s, target) in [(a0c, a0s, 0.0), (a0c, a0s, 1.0), (a1c, a1s, 0.0), (a1c, a1s, 1.0)] {
            if s.abs() > 1e-18 {
                push_root(roots, count, (target - c) / s);
            }
        }
    }

    fn push_radial_meets(&self, roots: &mut [f64; 96], count: &mut usize, q2: f64, q1: f64, q0: f64, p1: f64, p0: f64, ac: f64, as_: f64) {
        // q - 2 p a + vn2 a^2 = 0, a = ac + as z.
        let a2 = as_ * as_;
        let a1 = 2.0 * ac * as_;
        let a0 = ac * ac;
        let pa2 = p1 * as_;
        let pa1 = p1 * ac + p0 * as_;
        let pa0 = p0 * ac;
        push_quadratic_roots(roots, count, q2 - 2.0 * pa2 + self.vn2 * a2, q1 - 2.0 * pa1 + self.vn2 * a1, q0 - 2.0 * pa0 + self.vn2 * a0);
    }

    fn polynomials(&self, x: f64, y: f64) -> (f64, f64, f64, f64, f64, f64, f64) {
        // W(z) = W0 + z e_z, alpha = W·axis, N = W - alpha axis.
        let w0 = Vec3::new(x - self.base.x, y - self.base.y, -self.base.z);
        let alpha0 = w0.dot(self.axis);
        let alpha1 = self.axis.z;
        let ez = Vec3::new(0.0, 0.0, 1.0);
        let nz = ez.sub(self.axis.scale(alpha1));
        let n0 = w0.sub(self.axis.scale(alpha0));
        let q2 = nz.length_sq();
        let q1 = 2.0 * n0.dot(nz);
        let q0 = n0.length_sq() - self.radius * self.radius;
        let p1 = nz.dot(self.normal);
        let p0 = n0.dot(self.normal);
        (q2, q1, q0, p1, p0, alpha1, alpha0)
    }
}

fn push_root(roots: &mut [f64; 96], count: &mut usize, z: f64) {
    if z.is_finite() && *count < roots.len() {
        roots[*count] = z;
        *count += 1;
    }
}

fn push_quadratic_roots(roots: &mut [f64; 96], count: &mut usize, a: f64, b: f64, c: f64) {
    const A_EPS: f64 = 1e-14;
    if a.abs() <= A_EPS {
        if b.abs() > 1e-18 {
            push_root(roots, count, -c / b);
        }
        return;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        if disc > -1e-8 * (b * b + a.abs() * c.abs() + 1.0) {
            push_root(roots, count, -b / (2.0 * a));
        }
        return;
    }
    let root = disc.sqrt();
    let inv = 0.5 / a;
    push_root(roots, count, (-b - root) * inv);
    push_root(roots, count, (-b + root) * inv);
}

fn dedup_roots(roots: &mut [f64; 96], count: usize) -> usize {
    if count == 0 {
        return 0;
    }
    let mut kept = 1usize;
    for index in 1..count {
        if (roots[index] - roots[kept - 1]).abs() > 1e-9 {
            roots[kept] = roots[index];
            kept += 1;
        }
    }
    kept
}

fn bisect(mut pred: impl FnMut(f64) -> bool, mut lo: f64, mut hi: f64, entering: bool) -> f64 {
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        let inside = pred(mid);
        if entering {
            if inside { hi = mid } else { lo = mid }
        } else if inside {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    if entering { hi } else { lo }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn box_model(cell: f64) -> StockModel {
        StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 20.0), cell).expect("box")
    }

    fn near(actual: f64, expected: f64, tol: f64) {
        assert!((actual - expected).abs() <= tol, "{actual} != {expected} ± {tol}");
    }

    #[test]
    fn default_cell_clamps_the_radius_rule() {
        near(default_stock_cell(0.0), 2.0, 1e-12);
        near(default_stock_cell(0.1), 0.05, 1e-12);
        near(default_stock_cell(2.0), 0.5, 1e-12);
        near(default_stock_cell(20.0), 2.0, 1e-12);
    }

    #[test]
    fn flute_translations_are_order_invariant() {
        let mut left = box_model(1.0);
        let mut right = box_model(1.0);
        let flute = Flute::Flat { radius: 2.0, length: 6.0 };
        let axis = Vec3::new(0.0, 0.0, 1.0);
        left.subtract_flute_translation(Vec3::new(4.0, 5.0, 2.0), Vec3::new(12.0, 5.0, 2.0), axis, flute).expect("a");
        left.subtract_flute_translation(Vec3::new(6.0, 3.0, 4.0), Vec3::new(6.0, 11.0, 4.0), axis, flute).expect("b");
        right.subtract_flute_translation(Vec3::new(6.0, 3.0, 4.0), Vec3::new(6.0, 11.0, 4.0), axis, flute).expect("b");
        right.subtract_flute_translation(Vec3::new(4.0, 5.0, 2.0), Vec3::new(12.0, 5.0, 2.0), axis, flute).expect("a");
        assert_same_stock(&left, &right);
    }

    #[test]
    fn cell_cap_names_a_larger_cell() {
        let err = StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10_000.0, 10_000.0, 1.0), 0.05).expect_err("cap");
        let text = err.to_string();
        assert!(text.contains("use a larger stock_model.cell"), "{text}");
        assert!(text.contains("16000000"), "{text}");
    }

    #[test]
    fn rejects_an_empty_box_and_a_bad_cell() {
        assert!(StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 1.0), 0.5).is_err());
        assert!(StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0), 0.0).is_err());
        assert!(StockModel::from_box(Vec3::new(0.0, 0.0, f64::NAN), Vec3::new(1.0, 1.0, 1.0), 0.5).is_err());
    }

    #[test]
    fn vertical_ray_sphere_and_cylinder_match_known_intervals() {
        let sphere = ray_sphere_interval(1.0, 0.0, Vec3::new(0.0, 0.0, 5.0), 2.0).expect("sphere");
        near(sphere.0, 5.0 - (4.0_f64 - 1.0).sqrt(), 1e-9);
        near(sphere.1, 5.0 + (3.0_f64).sqrt(), 1e-9);
        assert!(ray_sphere_interval(3.0, 0.0, Vec3::new(0.0, 0.0, 5.0), 2.0).is_none());

        let vertical = ray_cylinder_interval(1.0, 0.0, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), 2.0).expect("vertical");
        near(vertical.0, 0.0, 1e-8);
        near(vertical.1, 10.0, 1e-8);
        assert!(ray_cylinder_interval(3.0, 0.0, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 10.0), 2.0).is_none());

        let flat = ray_cylinder_interval(5.0, 0.0, Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 2.0).expect("flat");
        near(flat.0, -2.0, 1e-8);
        near(flat.1, 2.0, 1e-8);
    }

    #[test]
    fn swept_cylinder_includes_the_intermediate_cap_rim() {
        // Vertical cylinder r=2, length 10, motion (5, 0, 3). The extreme on x=3 is the rim of
        // an intermediate cap, not a start or end face.
        let hit = ray_swept_cylinder_interval(3.0, 0.0, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 10.0, 2.0, Vec3::new(5.0, 0.0, 3.0))
            .expect("sweep");
        assert!(hit.0 <= 0.6 + 1e-4, "lo {}", hit.0);
        assert!(hit.1 >= 2.0, "hi {}", hit.1);
        assert!(hit.0 >= 0.4, "lo drifted {}", hit.0);
    }

    #[test]
    fn one_cylinder_matches_the_analytic_interval_on_every_dexel() {
        let mut stock = box_model(1.0);
        let a = Vec3::new(2.0, 4.0, 3.0);
        let b = Vec3::new(15.0, 12.0, 16.0);
        stock.subtract_cylinder(a, b, 2.5).expect("cut");
        for iy in 0..stock.ny {
            for ix in 0..stock.nx {
                let center = stock.cell_center(ix, iy).expect("center");
                let expected = remaining_after_cut(0.0, 20.0, ray_cylinder_interval(center.x, center.y, a, b, 2.5));
                assert_spans(stock.column(ix, iy).expect("col"), &expected);
            }
        }
    }

    #[test]
    fn straight_sweep_volume_stays_within_a_cell_of_the_analytic_solid() {
        // Cylinder along X, swept along Y. Section area is pi R^2 + 2 R D, times length.
        let radius = 2.0;
        let length = 10.0;
        let distance = 6.0;
        let cell = 0.5;
        let mut stock = StockModel::from_box(Vec3::new(-2.0, -2.0, -4.0), Vec3::new(16.0, 14.0, 8.0), cell).expect("box");
        let volume = stock
            .subtract_swept_cylinder(Vec3::new(1.0, 2.0, 0.0), Vec3::new(1.0, 0.0, 0.0), length, radius, Vec3::new(0.0, distance, 0.0))
            .expect("sweep");
        let analytic = length * (std::f64::consts::PI * radius * radius + 2.0 * radius * distance);
        let bound =
            cell * (2.0 * std::f64::consts::PI * radius * length + 2.0 * length * distance + 8.0 * radius * radius) + 4.0 * cell * cell * length;
        assert!((volume - analytic).abs() <= bound, "volume {volume} analytic {analytic} bound {bound}");
    }

    #[test]
    fn subtracting_orders_leave_the_same_stock() {
        let cuts =
            [(Vec3::new(4.0, 4.0, 6.0), 3.0), (Vec3::new(8.0, 7.0, 10.0), 4.0), (Vec3::new(6.0, 12.0, 8.0), 2.5), (Vec3::new(12.0, 5.0, 14.0), 3.5)];
        let mut forward = box_model(1.0);
        let mut reverse = box_model(1.0);
        for (center, radius) in cuts {
            forward.subtract_sphere(center, radius).expect("cut");
        }
        for (center, radius) in cuts.into_iter().rev() {
            reverse.subtract_sphere(center, radius).expect("cut");
        }
        assert_same_stock(&forward, &reverse);
    }

    #[test]
    fn ball_pose_removes_the_sphere_and_the_shank_cylinder() {
        let mut stock = box_model(1.0);
        let tip = Vec3::new(10.0, 10.0, 4.0);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        stock.subtract_flute(tip, axis, Flute::Ball { radius: 3.0, length: 8.0 }).expect("flute");
        // Cell size 1, origin 0: index 9 is centre 9.5, inside the radius-3 ball at (10, 10).
        let spans = stock.column(9, 9).expect("col");
        let removed = 20.0 - spans.iter().map(|span| span.hi - span.lo).sum::<f64>();
        assert!(removed > 1.0, "removed {removed} spans {spans:?}");
    }

    #[test]
    fn height_map_and_top_surface_follow_the_remaining_solid() {
        let mut stock = StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 5.0), 1.0).expect("box");
        stock.subtract_sphere(Vec3::new(0.5, 0.5, 5.0), 2.0).expect("cut");
        let map = stock.height_map();
        assert_eq!(map.len(), 4);
        assert!(map[0].expect("top") < 5.0);
        let mesh = stock.top_surface().expect("mesh");
        assert!(!mesh.triangles().is_empty());
    }

    #[test]
    fn compare_reports_leftover_and_a_model_gouge() {
        let plate =
            Mesh::new(vec![Triangle { a: Vec3::new(0.0, 0.0, 1.0), b: Vec3::new(4.0, 0.0, 1.0), c: Vec3::new(0.0, 4.0, 1.0) }]).expect("mesh");
        let mut stock = StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(4.0, 4.0, 3.0), 1.0).expect("box");
        let before = stock.compare_mesh(&plate, 0.2);
        assert!(before.samples > 0);
        assert!(before.max_leftover > 1.5, "{}", before.max_leftover);
        assert_eq!(before.model_gouges, 0);
        stock.subtract_cylinder(Vec3::new(0.5, 0.5, -1.0), Vec3::new(0.5, 0.5, 2.0), 0.2).expect("gouge");
        let after = stock.compare_mesh(&plate, 0.05);
        assert!(after.model_gouges >= 1, "gouges {}", after.model_gouges);
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore)]
    fn release_sweep_stays_under_a_few_seconds() {
        let mut stock = StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(80.0, 80.0, 30.0), 0.5).expect("box");
        let started = std::time::Instant::now();
        for index in 0..40 {
            let y = 4.0 + f64::from(index);
            stock.subtract_swept_cylinder(Vec3::new(2.0, y, 5.0), Vec3::new(1.0, 0.0, 0.0), 70.0, 2.0, Vec3::new(0.0, 0.4, 1.0)).expect("sweep");
        }
        assert!(started.elapsed().as_secs() < 5, "elapsed {:?}", started.elapsed());
    }

    fn remaining_after_cut(z0: f64, z1: f64, cut: Option<(f64, f64)>) -> Vec<ZInterval> {
        let mut column = vec![ZInterval { lo: z0, hi: z1 }];
        if let Some((lo, hi)) = cut {
            subtract_column(&mut column, lo, hi).expect("cut");
        }
        column
    }

    fn assert_spans(actual: &[ZInterval], expected: &[ZInterval]) {
        assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
        for (left, right) in actual.iter().zip(expected) {
            near(left.lo, right.lo, 1e-6);
            near(left.hi, right.hi, 1e-6);
        }
    }

    fn assert_same_stock(left: &StockModel, right: &StockModel) {
        assert_eq!(left.dimensions(), right.dimensions());
        for iy in 0..left.ny {
            for ix in 0..left.nx {
                assert_spans(left.column(ix, iy).expect("l"), right.column(ix, iy).expect("r"));
            }
        }
    }

    fn point_in_sphere(p: Vec3, center: Vec3, radius: f64) -> bool {
        p.sub(center).length() <= radius + 1e-7
    }

    fn point_in_cylinder(p: Vec3, a: Vec3, b: Vec3, radius: f64) -> bool {
        // Flat caps: the axial parameter has to land on the segment. Clamping it would test a capsule.
        let ab = b.sub(a);
        let len2 = ab.length_sq();
        if len2 <= 1e-18 {
            return false;
        }
        let t = p.sub(a).dot(ab) / len2;
        if !(-1e-7..=1.0 + 1e-7).contains(&t) {
            return false;
        }
        let closest = a.add(ab.scale(t.clamp(0.0, 1.0)));
        p.sub(closest).length() <= radius + 1e-6
    }

    fn point_in_swept(p: Vec3, base: Vec3, axis: Vec3, length: f64, radius: f64, motion: Vec3) -> bool {
        let axis = axis.normalized().expect("axis");
        let top = base.add(axis.scale(length));
        for step in 0..=80 {
            let t = f64::from(step) / 80.0;
            let shift = motion.scale(t);
            if point_in_cylinder(p, base.add(shift), top.add(shift), radius) {
                return true;
            }
        }
        false
    }

    fn stock_config() -> ProptestConfig {
        ProptestConfig { cases: 24, max_shrink_iters: 80, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_111D), ..ProptestConfig::default() }
    }

    proptest! {
        #![proptest_config(stock_config())]
        #[test]
        fn ray_intervals_match_dense_point_samples(
            cx in -8.0..8.0_f64,
            cy in -8.0..8.0_f64,
            cz in -8.0..8.0_f64,
            radius in 0.4..4.0_f64,
            ax in -6.0..6.0_f64,
            ay in -6.0..6.0_f64,
            az in -6.0..6.0_f64,
            ux in -1.0..1.0_f64,
            uy in -1.0..1.0_f64,
            uz in -1.0..1.0_f64,
            length in 0.5..8.0_f64,
            ray_x in -10.0..10.0_f64,
            ray_y in -10.0..10.0_f64,
            mx in -4.0..4.0_f64,
            my in -4.0..4.0_f64,
            mz in -4.0..4.0_f64,
        ) {
            let center = Vec3::new(cx, cy, cz);
            let sphere = ray_sphere_interval(ray_x, ray_y, center, radius);
            for step in 0..60 {
                let z = cz - radius - 1.0 + f64::from(step) * (2.0 * radius + 2.0) / 59.0;
                let inside = point_in_sphere(Vec3::new(ray_x, ray_y, z), center, radius);
                let covered = sphere.is_some_and(|(lo, hi)| z >= lo - 1e-6 && z <= hi + 1e-6);
                if inside {
                    prop_assert!(covered, "sphere z {z} interval {sphere:?}");
                }
                if let Some((lo, hi)) = sphere
                    && z > lo + 0.05
                    && z < hi - 0.05
                {
                    prop_assert!(inside, "sphere interval covers {z} but the point is outside");
                }
            }
            let axis = Vec3::new(ux, uy, uz);
            if axis.length() > 0.2 {
                let base = Vec3::new(ax, ay, az);
                let end = base.add(axis.normalized().expect("axis").scale(length));
                let interval = ray_cylinder_interval(ray_x, ray_y, base, end, radius);
                let z_mid = 0.5 * (base.z + end.z);
                let span = length + radius + 2.0;
                for step in 0..50 {
                    let z = z_mid - span + f64::from(step) * (2.0 * span) / 49.0;
                    let inside = point_in_cylinder(Vec3::new(ray_x, ray_y, z), base, end, radius);
                    let covered = interval.is_some_and(|(lo, hi)| z >= lo - 1e-5 && z <= hi + 1e-5);
                    if inside {
                        prop_assert!(covered, "cylinder z {z} interval {interval:?}");
                    }
                }
                let motion = Vec3::new(mx, my, mz);
                let swept = ray_swept_cylinder_interval(ray_x, ray_y, base, axis, length, radius, motion);
                for step in 0..40 {
                    let z = z_mid + mz * 0.5 - span - 2.0 + f64::from(step) * (2.0 * span + 4.0) / 39.0;
                    let point = Vec3::new(ray_x, ray_y, z);
                    let inside = point_in_swept(point, base, axis, length, radius, motion);
                    let covered = swept.is_some_and(|(lo, hi)| z >= lo - 1e-4 && z <= hi + 1e-4);
                    if inside {
                        prop_assert!(covered, "swept z {z} interval {swept:?}");
                    }
                }
            }
        }
    }

    proptest! {
        #![proptest_config(stock_config())]
        #[test]
        fn sphere_subtraction_is_order_invariant(
            ax in 2.0..8.0_f64,
            ay in 2.0..8.0_f64,
            az in 2.0..8.0_f64,
            bx in 6.0..14.0_f64,
            by in 6.0..14.0_f64,
            bz in 6.0..14.0_f64,
        ) {
            let first = Vec3::new(ax, ay, az);
            let second = Vec3::new(bx, by, bz);
            let mut left = box_model(2.0);
            let mut right = box_model(2.0);
            left.subtract_sphere(first, 2.5).expect("a");
            left.subtract_sphere(second, 3.0).expect("b");
            right.subtract_sphere(second, 3.0).expect("b");
            right.subtract_sphere(first, 2.5).expect("a");
            assert_same_stock(&left, &right);
        }
    }

    fn flat(radius: f64) -> Flute {
        Flute::Flat { radius, length: 30.0 }
    }

    fn ideal_engagement_deg(step: f64, diameter: f64) -> f64 {
        let u = (1.0 - 2.0 * step / diameter).clamp(-1.0, 1.0);
        u.acos().to_degrees()
    }

    /// One cell of wall error, doubled because the sample and the wall each sit on a dexel,
    /// plus the angle one cell subtends at the cutter radius.
    fn engagement_tol(step: f64, diameter: f64, cell: f64) -> f64 {
        let radius = diameter * 0.5;
        let low = (step - cell).max(diameter * 0.02);
        let high = (step + cell).min(diameter * 0.98);
        let spread = (ideal_engagement_deg(high, diameter) - ideal_engagement_deg(low, diameter)).abs();
        let angular = (cell / radius).atan().to_degrees();
        spread * 2.0 + angular + 2.0
    }

    #[test]
    fn a_slot_in_solid_stock_engages_half_the_cutter() {
        let cell = 0.5;
        let mut model = StockModel::from_box(Vec3::new(-15.0, -15.0, 0.0), Vec3::new(40.0, 15.0, 8.0), cell).expect("box");
        let radius = 4.0;
        let depth = 2.0;
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let tip = Vec3::new(10.0, 0.0, 8.0 - depth);
        let feed = Vec3::new(20.0, 0.0, 0.0);
        let measured = model.measure_flute(tip, axis, flat(radius), feed);
        near(measured.engagement_deg, 180.0, 1.0);
        near(measured.axial_doc, depth, 1e-6);
        // A plunge opens the start. The straight cut then removes a slot of width D.
        let start = Vec3::new(2.0, 0.0, tip.z);
        let end = Vec3::new(32.0, 0.0, tip.z);
        model.subtract_flute(start, axis, flat(radius)).expect("plunge");
        let volume = model.subtract_flute_translation(start, end, axis, flat(radius)).expect("slot");
        let length = 30.0;
        let feed_rate = 120.0;
        let mrr = volume * feed_rate / length;
        let ideal = (radius * 2.0) * depth * feed_rate;
        let bound = cell * depth * feed_rate;
        assert!((mrr - ideal).abs() <= bound, "mrr {mrr} ideal {ideal} bound {bound} volume {volume}");
    }

    #[test]
    fn a_sphere_splits_a_column_and_the_height_map_round_trips() {
        let mut model = StockModel::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(6.0, 4.0, 10.0), 1.0).expect("box");
        model.subtract_sphere(Vec3::new(2.5, 1.5, 5.0), 2.0).expect("sphere");
        let grid = model.stock_grid();
        assert!(!grid.multi.is_empty(), "the sphere should leave an undercut");
        let file = crate::StockFile {
            grid: grid.clone(),
            length: 12.5,
            snapshots: vec![crate::StockSnapshot { position: 0.0, top: grid.top.clone(), multi: Vec::new() }],
        };
        let text = file.to_json().expect("json");
        let back = crate::StockFile::from_json(&text).expect("parse");
        assert_eq!(back.grid.nx, grid.nx);
        assert_eq!(back.grid.ny, grid.ny);
        assert!((back.grid.cell - grid.cell).abs() < 1e-12);
        assert!((back.length - 12.5).abs() < 1e-12);
        assert_eq!(back.grid.multi, grid.multi);
        assert_eq!(back.snapshots.len(), 1);
        for (left, right) in back.grid.top.iter().zip(grid.top.iter()) {
            match (left, right) {
                (None, None) => {}
                (Some(a), Some(b)) => assert!((a - b).abs() < 1e-9, "{a} {b}"),
                pair => panic!("top mismatch {pair:?}"),
            }
        }
    }

    fn metric_config() -> ProptestConfig {
        ProptestConfig { cases: 8, max_shrink_iters: 20, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_232B), ..ProptestConfig::default() }
    }

    fn shoulder_angle(step: f64, side: f64, feed_sign: f64, cell: f64) -> (f64, f64) {
        let diameter = 8.0;
        let radius = diameter * 0.5;
        let depth = 2.0;
        let y_min = -diameter * 1.5;
        let y_max = diameter * 2.5;
        let mut model = StockModel::from_box(Vec3::new(-4.0, y_min, 0.0), Vec3::new(48.0, y_max, depth + 3.0), cell).expect("box");
        let flute = flat(radius);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let tip_z = depth + 3.0 - depth;
        let start = Vec3::new(0.0, 0.0, tip_z);
        let end = Vec3::new(40.0, 0.0, tip_z);
        model.subtract_flute(start, axis, flute).expect("plunge");
        model.subtract_flute_translation(start, end, axis, flute).expect("slot");
        let s0 = Vec3::new(4.0, side * step, tip_z);
        let s1 = Vec3::new(36.0, side * step, tip_z);
        let mid = s0.lerp(s1, 0.5);
        let measured = model.measure_flute(mid, axis, flute, Vec3::new(feed_sign, 0.0, 0.0));
        (measured.engagement_deg, measured.axial_doc)
    }

    proptest! {
        #![proptest_config(metric_config())]
        #[test]
        fn a_side_cut_matches_the_stepover_angle(
            step in 1.6..7.2_f64,
            side in proptest::prop_oneof![Just(1.0_f64), Just(-1.0)],
            feed_sign in proptest::prop_oneof![Just(1.0_f64), Just(-1.0)],
        ) {
            let cell = 0.5;
            let diameter = 8.0;
            let (angle, axial) = shoulder_angle(step, side, feed_sign, cell);
            let ideal = ideal_engagement_deg(step, diameter);
            let tol = engagement_tol(step, diameter, cell);
            prop_assert!((angle - ideal).abs() <= tol, "angle {angle} ideal {ideal} tol {tol} step {step}");
            prop_assert!((axial - 2.0).abs() < 1e-6, "axial {axial}");
        }
    }

    proptest! {
        #![proptest_config(metric_config())]
        #[test]
        fn straight_cut_mrr_is_width_times_depth_times_feed(depth in 1.0..3.5_f64, feed in 40.0..240.0_f64, length in 24.0..40.0_f64) {
            let cell = 0.5;
            let diameter = 8.0;
            let radius = diameter * 0.5;
            let top = depth + 4.0;
            let mut model = StockModel::from_box(Vec3::new(-6.0, -12.0, 0.0), Vec3::new(length + 10.0, 12.0, top), cell).expect("box");
            let flute = flat(radius);
            let axis = Vec3::new(0.0, 0.0, 1.0);
            let start = Vec3::new(2.0, 0.0, top - depth);
            let end = Vec3::new(2.0 + length, 0.0, top - depth);
            model.subtract_flute(start, axis, flute).expect("plunge");
            let volume = model.subtract_flute_translation(start, end, axis, flute).expect("cut");
            let mrr = volume * feed / length;
            let ideal = diameter * depth * feed;
            let bound = cell * depth * feed;
            prop_assert!((mrr - ideal).abs() <= bound, "mrr {mrr} ideal {ideal} bound {bound} volume {volume}");
        }
    }
}
