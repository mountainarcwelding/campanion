//! First-appearance island detection and related per-layer features.
//!
//! A layer is a plane at `z = z_min + (i + 0.5) * layer_height`, sliced in the mesh's own XY
//! (millimetres). Occupied cells are filled with non-zero winding, the same rule the resin
//! slicer uses, but on a user-entered cell size rather than a printer pixel grid.
//!
//! An *island* is a 4-connected occupied region on a layer that does not overlap any occupied
//! cell of the previous layer. Mid-air bodies therefore appear as islands at their first layer.
//!
//! Cups, area jumps, and thin features are reported alongside islands for the classifier. Cup
//! cavities are interior holes that are closed toward lower Z (the build plate) and therefore
//! open toward later layers (+Z, the film in this coordinate system).

use campanion_mesh::{Mesh, Triangle};

use crate::error::{ResinError, Result};
use crate::supports::cancel::{CancelFlag, GenerateProgress, check_cancel, screenshot_hold};
use crate::supports::profile;

/// Hard cap on occupancy grid width or height, in cells.
pub const MAX_GRID_SIDE: u32 = 2048;
/// Hard cap on the number of layers island detection will walk.
pub const MAX_DETECT_LAYERS: u32 = 10_000;

/// User-entered sampling for island detection. Nothing here is defaulted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IslandDetectParams {
    /// Layer spacing, millimetres. Must be finite and greater than zero.
    pub layer_height_mm: f64,
    /// Occupancy cell size, millimetres. Must be finite and greater than zero.
    pub cell_mm: f64,
}

impl IslandDetectParams {
    /// Reject non-finite or non-positive sampling values.
    pub fn validate(&self) -> Result<()> {
        if !self.layer_height_mm.is_finite() || self.layer_height_mm <= 0.0 {
            return Err(ResinError::invalid("island detection layer height must be greater than 0 mm"));
        }
        if !self.cell_mm.is_finite() || self.cell_mm <= 0.0 {
            return Err(ResinError::invalid("island detection cell size must be greater than 0 mm"));
        }
        Ok(())
    }
}

/// A first-appearance occupied region on one layer.
#[derive(Clone, Debug, PartialEq)]
pub struct Island {
    /// 1-based identifier, stable for a given mesh and sampling.
    pub id: u32,
    /// 0-based layer index in the detection walk.
    pub layer: u32,
    /// Plane height of this layer, millimetres.
    pub z_mm: f64,
    /// Occupied area, millimetres squared (`cells * cell_mm^2`).
    pub area_mm2: f64,
    /// Average of occupied cell centres, millimetres.
    pub centroid: (f64, f64),
    /// Occupied cell count.
    pub cells: usize,
    /// Inclusive minimum of occupied cell centres, millimetres.
    pub min: (f64, f64),
    /// Inclusive maximum of occupied cell centres, millimetres.
    pub max: (f64, f64),
}

/// One detection layer's occupied area and the islands that first appear on it.
#[derive(Clone, Debug, PartialEq)]
pub struct LayerStats {
    pub layer: u32,
    pub z_mm: f64,
    pub area_mm2: f64,
    /// Island ids that first appear on this layer.
    pub island_ids: Vec<u32>,
    /// `area_mm2 / previous area` when the previous layer had area; `None` otherwise.
    pub area_ratio: Option<f64>,
}

/// An interior cavity that is closed toward lower Z and therefore opens toward later layers.
#[derive(Clone, Debug, PartialEq)]
pub struct Cup {
    pub layer: u32,
    pub z_mm: f64,
    pub area_mm2: f64,
    pub centroid: (f64, f64),
}

/// A layer whose occupied area changed relative to the previous layer by at least a caller-supplied ratio.
#[derive(Clone, Debug, PartialEq)]
pub struct AreaJump {
    pub layer: u32,
    pub z_mm: f64,
    pub previous_area_mm2: f64,
    pub area_mm2: f64,
    pub ratio: f64,
}

/// Occupancy grid covering the mesh XY bounds, padded by one cell.
#[derive(Clone, Debug, PartialEq)]
pub struct DetectGrid {
    pub origin_x: f64,
    pub origin_y: f64,
    pub cell_mm: f64,
    pub width: u32,
    pub height: u32,
    /// One dense 0/1 bitmap per layer, row-major, length `width * height`.
    pub layers: Vec<Vec<u8>>,
}

impl DetectGrid {
    /// World X of the centre of column `col`.
    pub fn cell_x(&self, col: u32) -> f64 {
        self.origin_x + (f64::from(col) + 0.5) * self.cell_mm
    }

    /// World Y of the centre of row `row`.
    pub fn cell_y(&self, row: u32) -> f64 {
        self.origin_y + (f64::from(row) + 0.5) * self.cell_mm
    }

    pub fn index(&self, col: u32, row: u32) -> Option<usize> {
        if col >= self.width || row >= self.height {
            return None;
        }
        Some(row as usize * self.width as usize + col as usize)
    }
}

/// Islands, cups, and per-layer statistics for a mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct DetectedFeatures {
    pub islands: Vec<Island>,
    pub layers: Vec<LayerStats>,
    pub cups: Vec<Cup>,
    pub grid: DetectGrid,
    /// Mesh Z bounds used for the walk, millimetres.
    pub z_min_mm: f64,
    pub z_max_mm: f64,
}

impl DetectedFeatures {
    /// Area-jump records at or above `min_ratio`.
    ///
    /// The ratio is supplied by the caller (typically a calibration profile). This function
    /// does not invent one.
    pub fn area_jumps(&self, min_ratio: f64) -> Result<Vec<AreaJump>> {
        if !min_ratio.is_finite() || min_ratio <= 0.0 {
            return Err(ResinError::invalid("area-jump ratio must be greater than 0"));
        }
        let mut jumps = Vec::new();
        let mut previous = 0.0;
        for layer in &self.layers {
            if previous > 0.0
                && let Some(ratio) = layer.area_ratio
                && ratio >= min_ratio
            {
                jumps.push(AreaJump { layer: layer.layer, z_mm: layer.z_mm, previous_area_mm2: previous, area_mm2: layer.area_mm2, ratio });
            }
            previous = layer.area_mm2;
        }
        Ok(jumps)
    }

    /// Uniform layer height recovered from the walk, millimetres.
    pub fn layer_height_mm(&self) -> f64 {
        if let (Some(first), Some(second)) = (self.layers.first(), self.layers.get(1)) {
            second.z_mm - first.z_mm
        } else {
            (self.z_max_mm - self.z_min_mm).max(0.0)
        }
    }
}

/// Detect first-appearance islands and related layer features.
pub fn detect_islands(mesh: &Mesh, params: &IslandDetectParams) -> Result<DetectedFeatures> {
    detect_islands_with(mesh, params, None, None)
}

/// [`detect_islands`] with a cancel flag and optional progress reporting.
pub fn detect_islands_with(
    mesh: &Mesh,
    params: &IslandDetectParams,
    cancel: Option<&CancelFlag>,
    progress: Option<&GenerateProgress>,
) -> Result<DetectedFeatures> {
    check_cancel(cancel)?;
    params.validate()?;
    let t_all = std::time::Instant::now();
    let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let z_min = bounds.min().z;
    let z_max = bounds.max().z;
    if !z_min.is_finite() || !z_max.is_finite() || z_max < z_min {
        return Err(ResinError::invalid("part bounds are not finite"));
    }
    let height = z_max - z_min;
    let layer_count_f = (height / params.layer_height_mm).ceil();
    if !layer_count_f.is_finite() || layer_count_f > f64::from(MAX_DETECT_LAYERS) {
        return Err(ResinError::invalid(format!(
            "island detection would walk too many layers (layer height {} mm, part height {height} mm; maximum {MAX_DETECT_LAYERS})",
            params.layer_height_mm
        )));
    }
    let layer_count = (layer_count_f as u32).max(1);
    let pad = params.cell_mm;
    let origin_x = floor_to_cell(bounds.min().x - pad, params.cell_mm);
    let origin_y = floor_to_cell(bounds.min().y - pad, params.cell_mm);
    let max_x = bounds.max().x + pad;
    let max_y = bounds.max().y + pad;
    let width_f = ((max_x - origin_x) / params.cell_mm).ceil();
    let height_f = ((max_y - origin_y) / params.cell_mm).ceil();
    if !width_f.is_finite() || !height_f.is_finite() {
        return Err(ResinError::invalid("island detection grid size is not finite"));
    }
    if width_f > f64::from(MAX_GRID_SIDE) || height_f > f64::from(MAX_GRID_SIDE) {
        return Err(ResinError::invalid(format!(
            "island detection grid is too large ({width_f} x {height_f} cells; maximum side {MAX_GRID_SIDE}). Use a larger detection cell size"
        )));
    }
    let width = (width_f as u32).max(1);
    let height_cells = (height_f as u32).max(1);
    let mut grid =
        DetectGrid { origin_x, origin_y, cell_mm: params.cell_mm, width, height: height_cells, layers: Vec::with_capacity(layer_count as usize) };

    let triangles = mesh.triangles();
    let t_bucket = std::time::Instant::now();
    let mut buckets: Vec<Vec<usize>> = vec![Vec::new(); layer_count as usize];
    for (index, tri) in triangles.iter().enumerate() {
        let tz_min = tri.a.z.min(tri.b.z).min(tri.c.z);
        let tz_max = tri.a.z.max(tri.b.z).max(tri.c.z);
        if tz_max < z_min || tz_min > z_max {
            continue;
        }
        let first = (((tz_min - z_min) / params.layer_height_mm).floor() as i64).max(0) as u32;
        let last = (((tz_max - z_min) / params.layer_height_mm).floor() as i64).max(0) as u32;
        let last = last.min(layer_count.saturating_sub(1));
        let first = first.min(last);
        if let Some(bucket) = buckets.get_mut(first as usize..=last as usize) {
            for slot in bucket {
                slot.push(index);
            }
        }
    }

    profile::log("detect bucket", t_bucket);
    let t_raster = std::time::Instant::now();
    if let Some(progress) = progress {
        progress.set_layers(layer_count);
        progress.set_layer(0);
    }
    for i in 0..layer_count {
        if i == layer_count / 2 {
            screenshot_hold();
        }
        check_cancel(cancel)?;
        if let Some(progress) = progress {
            progress.set_layer(i + 1);
        }
        let z = z_min + (f64::from(i) + 0.5) * params.layer_height_mm;
        let bucket = buckets.get(i as usize).map_or(&[][..], Vec::as_slice);
        let bitmap = rasterize_layer(triangles, bucket, z, &grid);
        grid.layers.push(bitmap);
    }
    check_cancel(cancel)?;
    profile::log("detect raster", t_raster);
    let t_components = std::time::Instant::now();

    let cell_area = params.cell_mm * params.cell_mm;
    let mut islands = Vec::new();
    let mut layers = Vec::new();
    let mut cups = Vec::new();
    let mut next_id = 1u32;
    let mut previous: Option<&[u8]> = None;
    let mut previous_area = 0.0;
    for (i, bitmap) in grid.layers.iter().enumerate() {
        let z = z_min + (f64::from(i as u32) + 0.5) * params.layer_height_mm;
        let occupied = bitmap.iter().filter(|cell| **cell != 0).count();
        let area = occupied as f64 * cell_area;
        let area_ratio = if previous_area > 0.0 { Some(area / previous_area) } else { None };
        let components = connected_components(bitmap, grid.width, grid.height, true);
        let mut island_ids = Vec::new();
        for component in &components {
            let overlaps_previous = previous.is_some_and(|prev| component.cells.iter().any(|&idx| prev.get(idx).copied().unwrap_or(0) != 0));
            if overlaps_previous {
                continue;
            }
            let island = component_to_island(next_id, i as u32, z, component, &grid, cell_area)?;
            island_ids.push(next_id);
            islands.push(island);
            next_id = next_id.checked_add(1).ok_or_else(|| ResinError::invalid("too many islands"))?;
        }
        layers.push(LayerStats { layer: i as u32, z_mm: z, area_mm2: area, island_ids, area_ratio });
        let holes = connected_components(bitmap, grid.width, grid.height, false);
        for hole in &holes {
            if hole.touches_border {
                continue;
            }
            let closed_below = previous.is_some_and(|prev| hole.cells.iter().any(|&idx| prev.get(idx).copied().unwrap_or(0) != 0));
            if !closed_below {
                continue;
            }
            cups.push(component_to_cup(i as u32, z, hole, &grid, cell_area)?);
        }
        previous = Some(bitmap.as_slice());
        previous_area = area;
    }

    profile::log("detect components", t_components);
    profile::log("detect total", t_all);
    Ok(DetectedFeatures { islands, layers, cups, grid, z_min_mm: z_min, z_max_mm: z_max })
}

/// Occupied components whose smaller XY bounding-box side is strictly below `thin_width_mm`.
///
/// `thin_width_mm` must come from the caller (a calibration profile or an explicit flag).
pub fn thin_components(features: &DetectedFeatures, thin_width_mm: f64) -> Result<Vec<Island>> {
    if !thin_width_mm.is_finite() || thin_width_mm <= 0.0 {
        return Err(ResinError::invalid("thin-feature width must be greater than 0 mm"));
    }
    let layer_height = features.layer_height_mm();
    let mut out = Vec::new();
    let cell_area = features.grid.cell_mm * features.grid.cell_mm;
    for (i, bitmap) in features.grid.layers.iter().enumerate() {
        let z = features.z_min_mm + (f64::from(i as u32) + 0.5) * layer_height;
        let components = connected_components(bitmap, features.grid.width, features.grid.height, true);
        for (offset, component) in components.iter().enumerate() {
            let width = (component.max_col.saturating_sub(component.min_col) + 1) as f64 * features.grid.cell_mm;
            let height = (component.max_row.saturating_sub(component.min_row) + 1) as f64 * features.grid.cell_mm;
            if width.min(height) >= thin_width_mm {
                continue;
            }
            let id = (offset as u32).saturating_add(1);
            out.push(component_to_island(id, i as u32, z, component, &features.grid, cell_area)?);
        }
    }
    Ok(out)
}

/// Fill an island with candidate XY locations at `spacing_mm`, always including the centroid.
pub fn island_fill_points(island: &Island, spacing_mm: f64) -> Result<Vec<(f64, f64)>> {
    if !spacing_mm.is_finite() || spacing_mm <= 0.0 {
        return Err(ResinError::invalid("support spacing must be greater than 0 mm"));
    }
    let mut points = vec![island.centroid];
    if island.max.0 < island.min.0 || island.max.1 < island.min.1 {
        return Ok(points);
    }
    let mut x = island.min.0;
    let mut guard = 0usize;
    while x <= island.max.0 + 1e-12 {
        let mut y = island.min.1;
        while y <= island.max.1 + 1e-12 {
            points.push((x, y));
            y += spacing_mm;
            guard += 1;
            if guard > 20_000 {
                return Err(ResinError::invalid("island fill produced too many candidate points; increase spacing"));
            }
        }
        x += spacing_mm;
    }
    Ok(points)
}

fn floor_to_cell(value: f64, cell: f64) -> f64 {
    (value / cell).floor() * cell
}

struct Component {
    cells: Vec<usize>,
    min_col: u32,
    max_col: u32,
    min_row: u32,
    max_row: u32,
    sum_x: f64,
    sum_y: f64,
    touches_border: bool,
}

fn connected_components(bitmap: &[u8], width: u32, height: u32, occupied: bool) -> Vec<Component> {
    let want: u8 = u8::from(occupied);
    let total = width as usize * height as usize;
    let mut seen = vec![false; total];
    let mut out = Vec::new();
    for start in 0..total {
        if seen.get(start).copied().unwrap_or(true) {
            continue;
        }
        if bitmap.get(start).copied().unwrap_or(1) != want {
            continue;
        }
        let mut stack = vec![start];
        if let Some(flag) = seen.get_mut(start) {
            *flag = true;
        }
        let mut cells = Vec::new();
        let mut min_col = width;
        let mut max_col = 0;
        let mut min_row = height;
        let mut max_row = 0;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut touches_border = false;
        while let Some(idx) = stack.pop() {
            let col = (idx as u32) % width;
            let row = (idx as u32) / width;
            cells.push(idx);
            min_col = min_col.min(col);
            max_col = max_col.max(col);
            min_row = min_row.min(row);
            max_row = max_row.max(row);
            sum_x += f64::from(col);
            sum_y += f64::from(row);
            if col == 0 || row == 0 || col + 1 == width || row + 1 == height {
                touches_border = true;
            }
            let left = col.saturating_sub(1);
            let right = (col + 1).min(width.saturating_sub(1));
            let up = row.saturating_sub(1);
            let down = (row + 1).min(height.saturating_sub(1));
            for (nc, nr) in [(left, row), (right, row), (col, up), (col, down)] {
                let nidx = nr as usize * width as usize + nc as usize;
                if seen.get(nidx).copied().unwrap_or(true) {
                    continue;
                }
                if bitmap.get(nidx).copied().unwrap_or(1) != want {
                    continue;
                }
                if let Some(flag) = seen.get_mut(nidx) {
                    *flag = true;
                }
                stack.push(nidx);
            }
        }
        out.push(Component { cells, min_col, max_col, min_row, max_row, sum_x, sum_y, touches_border });
    }
    out
}

fn component_to_island(id: u32, layer: u32, z: f64, component: &Component, grid: &DetectGrid, cell_area: f64) -> Result<Island> {
    let n = component.cells.len();
    if n == 0 {
        return Err(ResinError::invalid("empty island component"));
    }
    let nf = n as f64;
    let cx = grid.origin_x + (component.sum_x / nf + 0.5) * grid.cell_mm;
    let cy = grid.origin_y + (component.sum_y / nf + 0.5) * grid.cell_mm;
    Ok(Island {
        id,
        layer,
        z_mm: z,
        area_mm2: nf * cell_area,
        centroid: (cx, cy),
        cells: n,
        min: (grid.cell_x(component.min_col), grid.cell_y(component.min_row)),
        max: (grid.cell_x(component.max_col), grid.cell_y(component.max_row)),
    })
}

fn component_to_cup(layer: u32, z: f64, component: &Component, grid: &DetectGrid, cell_area: f64) -> Result<Cup> {
    let n = component.cells.len();
    if n == 0 {
        return Err(ResinError::invalid("empty cup component"));
    }
    let nf = n as f64;
    let cx = grid.origin_x + (component.sum_x / nf + 0.5) * grid.cell_mm;
    let cy = grid.origin_y + (component.sum_y / nf + 0.5) * grid.cell_mm;
    Ok(Cup { layer, z_mm: z, area_mm2: nf * cell_area, centroid: (cx, cy) })
}

fn rasterize_layer(triangles: &[Triangle], bucket: &[usize], z: f64, grid: &DetectGrid) -> Vec<u8> {
    let mut crossings: Vec<(u32, f64, i8)> = Vec::new();
    let rows = f64::from(grid.height);
    for &id in bucket {
        let Some(tri) = triangles.get(id) else {
            continue;
        };
        let Some((a, b, weight)) = plane_segment_grid(tri, z, grid) else {
            continue;
        };
        if (b.1 - a.1).abs() < f64::EPSILON {
            continue;
        }
        let (y_lo, y_hi) = (a.1.min(b.1), a.1.max(b.1));
        let first = ((y_lo - 0.5).ceil()).max(0.0);
        let last = ((y_hi - 0.5).ceil() - 1.0).min(rows - 1.0);
        if last < first {
            continue;
        }
        let slope = (b.0 - a.0) / (b.1 - a.1);
        let mut row = first;
        while row <= last {
            let yc = row + 0.5;
            crossings.push((row as u32, a.0 + (yc - a.1) * slope, weight));
            row += 1.0;
        }
    }
    crossings.sort_unstable_by(|left, right| left.0.cmp(&right.0).then(left.1.total_cmp(&right.1)));
    let width = f64::from(grid.width);
    let mut bitmap = vec![0u8; grid.width as usize * grid.height as usize];
    let mut i = 0usize;
    while i < crossings.len() {
        let row = crossings.get(i).map_or(0, |c| c.0);
        let mut wind = 0i32;
        let mut start_x = 0.0f64;
        while let Some(&(r, x, w)) = crossings.get(i) {
            if r != row {
                break;
            }
            let before = wind;
            wind += i32::from(w);
            if before == 0 && wind != 0 {
                start_x = x;
            } else if before != 0 && wind == 0 {
                let c0 = (start_x - 0.5).ceil().clamp(0.0, width);
                let c1 = (x - 0.5).ceil().clamp(0.0, width);
                if c1 > c0 {
                    for col in c0 as u32..c1 as u32 {
                        if let Some(idx) = grid.index(col, row)
                            && let Some(cell) = bitmap.get_mut(idx)
                        {
                            *cell = 1;
                        }
                    }
                }
            }
            i += 1;
        }
    }
    bitmap
}

fn plane_segment_grid(tri: &Triangle, z: f64, grid: &DetectGrid) -> Option<((f64, f64), (f64, f64), i8)> {
    let v = [tri.a, tri.b, tri.c];
    let above = [v[0].z >= z, v[1].z >= z, v[2].z >= z];
    let mut pts = [(0.0, 0.0); 2];
    let mut found = 0usize;
    for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
        if above[i] != above[j] && found < 2 {
            let (p, q) = (v[i], v[j]);
            let denom = q.z - p.z;
            if denom.abs() < f64::EPSILON {
                continue;
            }
            let s = (z - p.z) / denom;
            if let Some(slot) = pts.get_mut(found) {
                *slot = (((p.x + (q.x - p.x) * s) - grid.origin_x) / grid.cell_mm, ((p.y + (q.y - p.y) * s) - grid.origin_y) / grid.cell_mm);
            }
            found += 1;
        }
    }
    if found != 2 {
        return None;
    }
    let e1 = v[1].sub(v[0]);
    let e2 = v[2].sub(v[0]);
    let nx = e1.y * e2.z - e1.z * e2.y;
    let ny = e1.z * e2.x - e1.x * e2.z;
    let (p0, p1) = (pts[0], pts[1]);
    let dir = (-ny, nx);
    let (a, b) = if (p1.0 - p0.0) * dir.0 + (p1.1 - p0.1) * dir.1 >= 0.0 { (p0, p1) } else { (p1, p0) };
    let weight: i8 = if b.1 > a.1 { 1 } else { -1 };
    Some((a, b, weight))
}
