//! Uniform XY grid for vertical ray queries used while placing supports.
//!
//! Support placement only shoots axis-aligned vertical rays. A conservative XY cell
//! list lets those rays test the triangles whose bounding boxes overlap the ray,
//! with the same Möller–Trumbore test as a full mesh walk.

use campanion_mesh::{Mesh, Vec3};

use crate::error::{ResinError, Result};

/// Maximum cells in the uniform grid. Keeps memory bounded on very large footprints.
const MAX_CELLS: usize = 256 * 256;

/// Default cell edge, millimetres, before the grid is clamped to [`MAX_CELLS`].
const DEFAULT_CELL_MM: f64 = 0.5;

/// Uniform XY bins of triangle indices for vertical (`+Z` / `-Z`) rays.
#[derive(Clone, Debug)]
pub struct VerticalHitGrid {
    x0: f64,
    y0: f64,
    inv_cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
    z_lo: Vec<f32>,
    z_hi: Vec<f32>,
}

impl VerticalHitGrid {
    /// Build a grid over `mesh`. An empty mesh yields an empty grid.
    pub fn build(mesh: &Mesh) -> Result<Self> {
        let triangles = mesh.triangles();
        if triangles.is_empty() {
            return Ok(Self { x0: 0.0, y0: 0.0, inv_cell: 1.0, nx: 1, ny: 1, cells: vec![Vec::new()], z_lo: Vec::new(), z_hi: Vec::new() });
        }
        let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
        let (min, max) = (bounds.min(), bounds.max());
        let pad = 1e-9;
        if ![min.x, min.y, max.x, max.y].iter().all(|v| v.is_finite()) {
            return Err(ResinError::invalid("part bounds are not finite"));
        }
        let x0 = min.x - pad;
        let y0 = min.y - pad;
        let width = (max.x - min.x) + 2.0 * pad;
        let height = (max.y - min.y) + 2.0 * pad;
        let long = width.max(height).max(DEFAULT_CELL_MM);
        // Prefer ~0.5 mm cells, but keep at least 64 cells along the longer side when the
        // part is large, without exceeding MAX_CELLS.
        let mut cell = DEFAULT_CELL_MM.min(long / 64.0).max(0.25);
        if !cell.is_finite() || cell <= 0.0 {
            return Err(ResinError::invalid("hit-grid cell size must be greater than 0 mm"));
        }
        let mut nx = ((width / cell).ceil() as usize).max(1);
        let mut ny = ((height / cell).ceil() as usize).max(1);
        let total = nx.saturating_mul(ny);
        if total > MAX_CELLS {
            let scale = (total as f64 / MAX_CELLS as f64).sqrt();
            cell *= scale;
            nx = ((width / cell).ceil() as usize).max(1);
            ny = ((height / cell).ceil() as usize).max(1);
        }
        let mut cells = vec![Vec::new(); nx.saturating_mul(ny).max(1)];
        let mut z_lo = vec![f32::INFINITY; triangles.len()];
        let mut z_hi = vec![f32::NEG_INFINITY; triangles.len()];
        let inv_cell = 1.0 / cell;
        for (index, tri) in triangles.iter().enumerate() {
            if index > u32::MAX as usize {
                return Err(ResinError::invalid("mesh has too many triangles for the hit grid"));
            }
            let min_x = tri.a.x.min(tri.b.x).min(tri.c.x);
            let max_x = tri.a.x.max(tri.b.x).max(tri.c.x);
            let min_y = tri.a.y.min(tri.b.y).min(tri.c.y);
            let max_y = tri.a.y.max(tri.b.y).max(tri.c.y);
            let min_z = tri.a.z.min(tri.b.z).min(tri.c.z);
            let max_z = tri.a.z.max(tri.b.z).max(tri.c.z);
            z_lo[index] = min_z as f32;
            z_hi[index] = max_z as f32;
            if ![min_x, max_x, min_y, max_y].iter().all(|v| v.is_finite()) {
                continue;
            }
            let ix0 = cell_coord(min_x, x0, inv_cell, nx);
            let ix1 = cell_coord(max_x, x0, inv_cell, nx);
            let iy0 = cell_coord(min_y, y0, inv_cell, ny);
            let iy1 = cell_coord(max_y, y0, inv_cell, ny);
            let (ix0, ix1) = if ix0 <= ix1 { (ix0, ix1) } else { (ix1, ix0) };
            let (iy0, iy1) = if iy0 <= iy1 { (iy0, iy1) } else { (iy1, iy0) };
            for iy in iy0..=iy1 {
                let row = iy * nx;
                for ix in ix0..=ix1 {
                    cells[row + ix].push(index as u32);
                }
            }
        }
        if crate::supports::profile::enabled() {
            let occupied = cells.iter().filter(|c| !c.is_empty()).count();
            let stored: usize = cells.iter().map(Vec::len).sum();
            let max_occ = cells.iter().map(Vec::len).max().unwrap_or(0);
            crate::supports::profile::log_count(
                &format!("hit-grid {nx}x{ny} cell={cell:.3} mm occupied={occupied} entries={stored} max={max_occ}"),
                triangles.len() as u64,
            );
        }
        Ok(Self { x0, y0, inv_cell, nx, ny, cells, z_lo, z_hi })
    }

    /// Closest hit of `origin + t * dir` with `t >= 0`.
    ///
    /// Non-vertical directions fall back to a full mesh walk so the result stays exact.
    pub fn first_hit(&self, mesh: &Mesh, origin: Vec3, dir: Vec3, skip: Option<usize>) -> Option<(f64, usize)> {
        if dir.length_sq() <= 0.0 {
            return None;
        }
        if dir.x.abs() > 1e-15 || dir.y.abs() > 1e-15 {
            return crate::supports::place::first_hit(mesh, origin, dir, skip);
        }
        let cell = self.cell_at(origin.x, origin.y)?;
        let mut best: Option<(f64, usize)> = None;
        let mut tested = 0u64;
        let going_up = dir.z >= 0.0;
        for &index in &self.cells[cell] {
            let index = index as usize;
            if skip == Some(index) {
                continue;
            }
            if going_up {
                if (self.z_hi.get(index).copied().unwrap_or(f32::NEG_INFINITY) as f64) + 1e-9 < origin.z {
                    continue;
                }
            } else if (self.z_lo.get(index).copied().unwrap_or(f32::INFINITY) as f64) > origin.z + 1e-9 {
                continue;
            }
            tested += 1;
            let Some(triangle) = mesh.triangles().get(index) else {
                continue;
            };
            let Some(t) = crate::supports::place::ray_triangle(origin, dir, triangle) else {
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
        crate::supports::profile::record_hit(tested);
        best
    }

    /// Lowest downward face of `mesh` at `(x, y)` with `z` strictly above `z_min`.
    pub fn lowest_hit_above(&self, mesh: &Mesh, x: f64, y: f64, z_min: f64) -> Option<(Vec3, usize)> {
        let origin = Vec3::new(x, y, z_min);
        let dir = Vec3::new(0.0, 0.0, 1.0);
        let (t, index) = self.first_hit(mesh, origin, dir, None)?;
        let z = z_min + t;
        if !z.is_finite() {
            return None;
        }
        Some((Vec3::new(x, y, z), index))
    }

    /// The part already occupies the column below `point` when a downward ray hits another triangle.
    pub fn is_self_supported(&self, mesh: &Mesh, point: Vec3, source: Option<usize>) -> bool {
        let origin = Vec3::new(point.x, point.y, point.z - 0.05);
        matches!(self.first_hit(mesh, origin, Vec3::new(0.0, 0.0, -1.0), source), Some((t, _)) if t > 1e-6)
    }

    fn cell_at(&self, x: f64, y: f64) -> Option<usize> {
        if !x.is_finite() || !y.is_finite() || self.cells.is_empty() {
            return None;
        }
        let ix = cell_coord(x, self.x0, self.inv_cell, self.nx);
        let iy = cell_coord(y, self.y0, self.inv_cell, self.ny);
        Some(iy * self.nx + ix)
    }
}

fn cell_coord(value: f64, origin: f64, inv_cell: f64, count: usize) -> usize {
    if count <= 1 {
        return 0;
    }
    let raw = (value - origin) * inv_cell;
    if !raw.is_finite() {
        return 0;
    }
    let idx = raw.floor() as i64;
    if idx <= 0 {
        0
    } else if idx >= count as i64 {
        count - 1
    } else {
        idx as usize
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use campanion_mesh::{Aabb, Mesh, Triangle, Vec3};

    use crate::supports::params::SupportParams;
    use crate::supports::place::{first_hit, place_supports, place_supports_linear};

    fn box_mesh(min: Vec3, max: Vec3) -> Mesh {
        Mesh::new(Aabb::from_corners(min, max).face_triangles().to_vec()).expect("box")
    }

    fn params() -> SupportParams {
        SupportParams {
            raft_chamfer_angle_deg: 30.0,
            raft_thickness_mm: 1.5,
            raft_margin_mm: 3.0,
            raft_corner_relief_mm: 0.0,
            overhang_angle_deg: 45.0,
            support_spacing_mm: 4.0,
            tip_diameter_mm: 0.6,
            tip_length_mm: 1.2,
            pillar_diameter_mm: 1.2,
            foot_diameter_mm: 3.0,
            foot_height_mm: 0.8,
            part_lift_mm: 5.0,
        }
    }

    /// Two stacked boxes: the upper plate hangs over the post, so some samples are self-supported.
    fn table_mesh() -> Mesh {
        let post = box_mesh(Vec3::new(-2.0, -2.0, 0.0), Vec3::new(2.0, 2.0, 8.0));
        let plate = box_mesh(Vec3::new(-10.0, -10.0, 8.0), Vec3::new(10.0, 10.0, 10.0));
        let mut tris = post.triangles().to_vec();
        tris.extend_from_slice(plate.triangles());
        Mesh::new(tris).expect("table")
    }

    /// A tessellated downward-facing roof over a solid floor, plus vertical posts.
    fn shelter_mesh(div: usize) -> Mesh {
        let mut tris = Vec::new();
        // Floor 30 x 30 x 2.
        tris.extend_from_slice(box_mesh(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 30.0, 2.0)).triangles());
        // Corner posts that hold the roof, so the centre of the roof is not self-supported
        // only where a post sits. The floor self-supports the whole roof footprint.
        for (x, y) in [(2.0, 2.0), (26.0, 2.0), (2.0, 26.0), (26.0, 26.0)] {
            tris.extend_from_slice(box_mesh(Vec3::new(x, y, 2.0), Vec3::new(x + 2.0, y + 2.0, 10.0)).triangles());
        }
        // Tessellated roof slab at z = 10..11.
        let n = div.max(1);
        for iy in 0..n {
            for ix in 0..n {
                let x0 = 30.0 * ix as f64 / n as f64;
                let x1 = 30.0 * (ix + 1) as f64 / n as f64;
                let y0 = 30.0 * iy as f64 / n as f64;
                let y1 = 30.0 * (iy + 1) as f64 / n as f64;
                // Top (up) and bottom (down) of the thin slab, two triangles each.
                tris.push(Triangle { a: Vec3::new(x0, y0, 11.0), b: Vec3::new(x1, y0, 11.0), c: Vec3::new(x1, y1, 11.0) });
                tris.push(Triangle { a: Vec3::new(x0, y0, 11.0), b: Vec3::new(x1, y1, 11.0), c: Vec3::new(x0, y1, 11.0) });
                tris.push(Triangle { a: Vec3::new(x0, y0, 10.0), b: Vec3::new(x1, y1, 10.0), c: Vec3::new(x1, y0, 10.0) });
                tris.push(Triangle { a: Vec3::new(x0, y0, 10.0), b: Vec3::new(x0, y1, 10.0), c: Vec3::new(x1, y1, 10.0) });
            }
        }
        Mesh::new(tris).expect("shelter")
    }

    fn hits_match(linear: Option<(f64, usize)>, grid: Option<(f64, usize)>) -> bool {
        match (linear, grid) {
            (None, None) => true,
            (Some((t_a, _)), Some((t_b, _))) => (t_a - t_b).abs() <= 1e-12,
            _ => false,
        }
    }

    #[test]
    fn vertical_grid_hits_match_a_full_mesh_walk_on_a_table() {
        let mesh = table_mesh();
        let grid = VerticalHitGrid::build(&mesh).expect("grid");
        let mut compared = 0u32;
        for ix in -12..=12 {
            for iy in -12..=12 {
                let x = f64::from(ix);
                let y = f64::from(iy);
                let up = first_hit(&mesh, Vec3::new(x, y, -1.0), Vec3::new(0.0, 0.0, 1.0), None);
                let up_g = grid.first_hit(&mesh, Vec3::new(x, y, -1.0), Vec3::new(0.0, 0.0, 1.0), None);
                assert!(hits_match(up, up_g), "up {x},{y}: {up:?} vs {up_g:?}");
                let down = first_hit(&mesh, Vec3::new(x, y, 12.0), Vec3::new(0.0, 0.0, -1.0), None);
                let down_g = grid.first_hit(&mesh, Vec3::new(x, y, 12.0), Vec3::new(0.0, 0.0, -1.0), None);
                assert!(hits_match(down, down_g), "down {x},{y}: {down:?} vs {down_g:?}");
                compared += 2;
            }
        }
        assert!(compared > 100);
    }

    #[test]
    fn vertical_grid_hits_match_a_full_mesh_walk_on_a_tessellated_shelter() {
        let mesh = shelter_mesh(12);
        let grid = VerticalHitGrid::build(&mesh).expect("grid");
        for i in 0..40 {
            for j in 0..40 {
                let x = (f64::from(i) + 0.25) * 30.0 / 40.0;
                let y = (f64::from(j) + 0.5) * 30.0 / 40.0;
                let origin = Vec3::new(x, y, 6.0);
                let up = first_hit(&mesh, origin, Vec3::new(0.0, 0.0, 1.0), None);
                let up_g = grid.first_hit(&mesh, origin, Vec3::new(0.0, 0.0, 1.0), None);
                assert!(hits_match(up, up_g), "up {x},{y}: {up:?} vs {up_g:?}");
                let down = first_hit(&mesh, origin, Vec3::new(0.0, 0.0, -1.0), None);
                let down_g = grid.first_hit(&mesh, origin, Vec3::new(0.0, 0.0, -1.0), None);
                assert!(hits_match(down, down_g), "down {x},{y}: {down:?} vs {down_g:?}");
            }
        }
    }

    fn points_match(a: &[crate::supports::place::SupportPoint], b: &[crate::supports::place::SupportPoint]) -> std::result::Result<(), String> {
        if a.len() != b.len() {
            return Err(format!("point count {} vs {}", a.len(), b.len()));
        }
        for (i, (pa, pb)) in a.iter().zip(b.iter()).enumerate() {
            if (pa.x - pb.x).abs() > 1e-12 || (pa.y - pb.y).abs() > 1e-12 || (pa.contact_z - pb.contact_z).abs() > 1e-12 {
                return Err(format!("point {i}: ({}, {}, {}) vs ({}, {}, {})", pa.x, pa.y, pa.contact_z, pb.x, pb.y, pb.contact_z));
            }
            if pa.source != pb.source {
                return Err(format!("point {i} source {:?} vs {:?}", pa.source, pb.source));
            }
        }
        Ok(())
    }

    #[test]
    fn place_supports_matches_the_linear_scan_on_synthetic_meshes() {
        let cases = [table_mesh(), shelter_mesh(8), box_mesh(Vec3::new(-5.0, 3.0, 1.0), Vec3::new(12.0, 18.0, 9.0))];
        for (i, mesh) in cases.iter().enumerate() {
            let linear = place_supports_linear(mesh, &params(), &[], true, 0.0).expect("linear");
            let grid = place_supports(mesh, &params(), &[], true, 0.0).expect("grid");
            points_match(&linear, &grid).unwrap_or_else(|err| panic!("mesh {i}: {err}"));
        }
    }

    #[test]
    fn optional_env_stl_vertical_hits_match_a_full_mesh_walk() {
        let path = match std::env::var("CAMPANION_PERF_STL") {
            Ok(path) if !path.is_empty() => path,
            _ => return,
        };
        let bytes = std::fs::read(&path).unwrap_or_else(|err| panic!("read {path}: {err}"));
        let mesh = campanion_mesh::parse_stl(&bytes).unwrap_or_else(|err| panic!("parse {path}: {err}"));
        assert!(mesh.triangles().len() > 1_000, "optional STL is too small to be a useful check");
        let grid = VerticalHitGrid::build(&mesh).expect("grid");
        let bounds = mesh.bounds().expect("bounds");
        let (min, max) = (bounds.min(), bounds.max());
        let mut compared = 0u32;
        let steps = 16;
        for i in 0..=steps {
            for j in 0..=steps {
                let tx = f64::from(i) / f64::from(steps);
                let ty = f64::from(j) / f64::from(steps);
                let x = min.x + (max.x - min.x) * tx;
                let y = min.y + (max.y - min.y) * ty;
                let mid_z = (min.z + max.z) / 2.0;
                for (origin, dir) in [
                    (Vec3::new(x, y, min.z - 1.0), Vec3::new(0.0, 0.0, 1.0)),
                    (Vec3::new(x, y, max.z + 1.0), Vec3::new(0.0, 0.0, -1.0)),
                    (Vec3::new(x, y, mid_z), Vec3::new(0.0, 0.0, 1.0)),
                    (Vec3::new(x, y, mid_z), Vec3::new(0.0, 0.0, -1.0)),
                ] {
                    let linear = first_hit(&mesh, origin, dir, None);
                    let accelerated = grid.first_hit(&mesh, origin, dir, None);
                    assert!(hits_match(linear, accelerated), "({:.6},{:.6}) dir={:?}: {linear:?} vs {accelerated:?}", origin.x, origin.y, dir);
                    compared += 1;
                }
            }
        }
        assert!(compared >= 16 * 16 * 4);
    }
}
