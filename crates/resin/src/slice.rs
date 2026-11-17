//! Slice a triangle mesh into per-layer pixel spans.
//!
//! Layer `i` is sampled at the plane `z = (i + 0.5) * layer_height` above the build plate. Each
//! triangle that crosses the plane contributes one directed segment (interior on its left, from the
//! triangle normal). Segments are scan-converted at pixel-row centres and filled by non-zero
//! winding, so overlapping shells (a body touching its supports) merge instead of cancelling.
//! Pixels are fully on or fully off. Anti-aliasing is not implemented.
//!
//! Image axes: pixel column grows with model +X and pixel row grows with model -Y, so +Y points to
//! the top of the image. Whether that matches how the printer's screen is mounted has not been
//! checked against a printed part.

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};

/// Most layers we will produce.
pub const MAX_LAYERS: usize = 60_000;
/// Most (triangle, layer) pairs we will bucket.
const MAX_PAIRS: usize = 400_000_000;

/// Pixel grid of the LCD, with the physical pixel pitch in millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SliceGrid {
    pub width: u32,
    pub height: u32,
    pub pitch_x_mm: f64,
    pub pitch_y_mm: f64,
}

/// Where the model goes on the plate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Placement {
    /// Centre the model's XY bounds on the plate centre.
    Center,
    /// Use the mesh XY as millimetres from the plate centre.
    Keep,
}

/// Horizontal run of lit pixels: row, first column, one past the last column.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Span {
    pub row: u32,
    pub start: u32,
    pub end: u32,
}

/// A mesh moved onto the plate with the bottom at Z = 0, ready to slice.
#[derive(Debug, Clone)]
pub struct PreparedMesh {
    grid: SliceGrid,
    layer_height: f64,
    layer_count: usize,
    triangles: Vec<[f64; 9]>,
    buckets: Vec<Vec<u32>>,
    height_mm: f64,
    shift: (f64, f64),
}

impl PreparedMesh {
    /// Place `mesh` on the plate. Fails when it does not fit the plate (it is never clipped).
    pub fn new(mesh: &Mesh, grid: SliceGrid, plate_height_mm: f64, layer_height_mm: f64, placement: Placement) -> Result<Self> {
        if !(layer_height_mm.is_finite() && layer_height_mm > 0.0 && layer_height_mm <= 1.0) {
            return Err(ResinError::invalid("layer height must be above 0 and at most 1 mm"));
        }
        if grid.width == 0 || grid.height == 0 || !(grid.pitch_x_mm > 0.0 && grid.pitch_y_mm > 0.0) {
            return Err(ResinError::invalid("bad pixel grid"));
        }
        let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("mesh: {err}")))?;
        let (min, max) = (bounds.min(), bounds.max());
        let (shift_x, shift_y) = match placement {
            Placement::Center => (-(min.x + max.x) / 2.0, -(min.y + max.y) / 2.0),
            Placement::Keep => (0.0, 0.0),
        };
        let half_w = f64::from(grid.width) * grid.pitch_x_mm / 2.0;
        let half_h = f64::from(grid.height) * grid.pitch_y_mm / 2.0;
        let (lo_x, hi_x, lo_y, hi_y) = (min.x + shift_x, max.x + shift_x, min.y + shift_y, max.y + shift_y);
        if lo_x < -half_w || hi_x > half_w || lo_y < -half_h || hi_y > half_h {
            return Err(ResinError::invalid(format!(
                "model is {:.2} x {:.2} mm after placement and does not fit the {:.1} x {:.1} mm plate",
                hi_x - lo_x,
                hi_y - lo_y,
                half_w * 2.0,
                half_h * 2.0
            )));
        }
        let height_mm = max.z - min.z;
        if !(height_mm > 0.0) {
            return Err(ResinError::invalid("model has no height"));
        }
        if height_mm > plate_height_mm {
            return Err(ResinError::invalid(format!("model is {height_mm:.2} mm tall; the machine allows {plate_height_mm:.1} mm")));
        }
        let layer_count_f = (height_mm / layer_height_mm - 1e-9).ceil();
        if !(layer_count_f >= 1.0) || layer_count_f > MAX_LAYERS as f64 {
            return Err(ResinError::invalid(format!("{layer_count_f} layers is out of range (1 to {MAX_LAYERS})")));
        }
        let layer_count = layer_count_f as usize;

        let cx = f64::from(grid.width) / 2.0;
        let cy = f64::from(grid.height) / 2.0;
        let to_px = |v: Vec3| -> (f64, f64, f64) { (cx + (v.x + shift_x) / grid.pitch_x_mm, cy - (v.y + shift_y) / grid.pitch_y_mm, v.z - min.z) };
        let mut triangles = Vec::with_capacity(mesh.triangles().len());
        let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); layer_count];
        let mut pairs = 0usize;
        for (index, tri) in mesh.triangles().iter().enumerate() {
            let (a, b, c) = (to_px(tri.a), to_px(tri.b), to_px(tri.c));
            let n = model_normal_xy(tri);
            triangles.push([a.0, a.1, a.2, b.0, b.1, b.2, c.0, c.1, c.2]);
            let (z_lo, z_hi) = (a.2.min(b.2).min(c.2), a.2.max(b.2).max(c.2));
            if n.is_none() || z_lo >= z_hi {
                continue;
            }
            // Layers i with z_lo < (i + 0.5) h <= z_hi.
            let first = ((z_lo / layer_height_mm - 0.5).floor() + 1.0).max(0.0) as usize;
            let last_f = (z_hi / layer_height_mm - 0.5).floor();
            if last_f < 0.0 {
                continue;
            }
            let last = (last_f as usize).min(layer_count - 1);
            if first > last {
                continue;
            }
            pairs += last - first + 1;
            if pairs > MAX_PAIRS {
                return Err(ResinError::invalid("mesh is too large to slice"));
            }
            let id = u32::try_from(index).map_err(|_| ResinError::invalid("too many triangles"))?;
            for layer in first..=last {
                if let Some(bucket) = buckets.get_mut(layer) {
                    bucket.push(id);
                }
            }
        }
        Ok(Self { grid, layer_height: layer_height_mm, layer_count, triangles, buckets, height_mm, shift: (shift_x, shift_y) })
    }

    pub fn layer_count(&self) -> usize {
        self.layer_count
    }

    pub fn layer_height_mm(&self) -> f64 {
        self.layer_height
    }

    pub fn height_mm(&self) -> f64 {
        self.height_mm
    }

    pub fn grid(&self) -> SliceGrid {
        self.grid
    }

    /// XY shift applied to the mesh, in millimetres.
    pub fn xy_shift_mm(&self) -> (f64, f64) {
        self.shift
    }

    /// Lit pixel spans for layer `index`, sorted by row then column.
    pub fn layer_spans(&self, index: usize) -> Vec<Span> {
        let Some(bucket) = self.buckets.get(index) else {
            return Vec::new();
        };
        let zc = (index as f64 + 0.5) * self.layer_height;
        let rows = f64::from(self.grid.height);
        let mut crossings: Vec<(u32, f64, i8)> = Vec::new();
        for &id in bucket {
            let Some(t) = self.triangles.get(id as usize) else {
                continue;
            };
            let v = [(t[0], t[1], t[2]), (t[3], t[4], t[5]), (t[6], t[7], t[8])];
            let above = [v[0].2 >= zc, v[1].2 >= zc, v[2].2 >= zc];
            let mut pts: [(f64, f64); 2] = [(0.0, 0.0); 2];
            let mut found = 0usize;
            for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
                if above[i] != above[j] && found < 2 {
                    let (p, q) = (v[i], v[j]);
                    let s = (zc - p.2) / (q.2 - p.2);
                    if let Some(slot) = pts.get_mut(found) {
                        *slot = (p.0 + (q.0 - p.0) * s, p.1 + (q.1 - p.1) * s);
                    }
                    found += 1;
                }
            }
            if found != 2 {
                continue;
            }
            // Orient every segment by the triangle normal computed in pixel space. The Y flip
            // mirrors that normal the same way for every triangle, so the interior stays on one
            // side. Non-zero winding does not care which side that is.
            let e1 = (v[1].0 - v[0].0, v[1].1 - v[0].1, v[1].2 - v[0].2);
            let e2 = (v[2].0 - v[0].0, v[2].1 - v[0].1, v[2].2 - v[0].2);
            let nx = e1.1 * e2.2 - e1.2 * e2.1;
            let ny = e1.2 * e2.0 - e1.0 * e2.2;
            let (p0, p1) = (pts[0], pts[1]);
            let dir = (-ny, nx);
            let (a, b) = if (p1.0 - p0.0) * dir.0 + (p1.1 - p0.1) * dir.1 >= 0.0 { (p0, p1) } else { (p1, p0) };
            if (b.1 - a.1).abs() < f64::EPSILON {
                continue;
            }
            let weight: i8 = if b.1 > a.1 { 1 } else { -1 };
            let (y_lo, y_hi) = (a.1.min(b.1), a.1.max(b.1));
            let first = ((y_lo - 0.5).ceil()).max(0.0);
            let last = ((y_hi - 0.5).ceil() - 1.0).min(rows - 1.0);
            let slope = (b.0 - a.0) / (b.1 - a.1);
            let mut row = first;
            while row <= last {
                let yc = row + 0.5;
                crossings.push((row as u32, a.0 + (yc - a.1) * slope, weight));
                row += 1.0;
            }
        }
        crossings.sort_unstable_by(|l, r| l.0.cmp(&r.0).then(l.1.total_cmp(&r.1)));
        let width = f64::from(self.grid.width);
        let mut spans = Vec::new();
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
                        spans.push(Span { row, start: c0 as u32, end: c1 as u32 });
                    }
                }
                i += 1;
            }
        }
        spans
    }

    /// Lit pixel count of a span list.
    pub fn lit_pixels(spans: &[Span]) -> u64 {
        spans.iter().map(|s| u64::from(s.end - s.start)).sum()
    }
}

fn model_normal_xy(tri: &Triangle) -> Option<(f64, f64)> {
    let n = tri.normal()?;
    if n.x.hypot(n.y) < 1e-12 { None } else { Some((n.x, n.y)) }
}

/// Merge touching spans on the same row and encode them as a layer image.
pub fn encode_spans(spans: &[Span], width: u32, height: u32) -> Result<Vec<u8>> {
    use crate::pw0::{MAX_GRAY, RunWriter};
    let w = u64::from(width);
    let total = w * u64::from(height);
    let mut writer = RunWriter::new();
    let mut cursor = 0u64;
    for span in spans {
        if span.row >= height || span.end > width || span.start >= span.end {
            return Err(ResinError::invalid("span is outside the image"));
        }
        let begin = u64::from(span.row) * w + u64::from(span.start);
        let end = u64::from(span.row) * w + u64::from(span.end);
        if begin < cursor {
            return Err(ResinError::invalid("spans overlap or are out of order"));
        }
        writer.push(0, begin - cursor)?;
        writer.push(MAX_GRAY, end - begin)?;
        cursor = end;
    }
    writer.push(0, total - cursor)?;
    Ok(writer.finish())
}

/// Dense 0/15 bitmap of a span list, for tests and tools.
pub fn spans_to_dense(spans: &[Span], width: u32, height: u32) -> Vec<u8> {
    let mut out = vec![0u8; width as usize * height as usize];
    for s in spans {
        let base = s.row as usize * width as usize;
        if let Some(slice) = out.get_mut(base + s.start as usize..base + s.end as usize) {
            slice.fill(crate::pw0::MAX_GRAY);
        }
    }
    out
}
