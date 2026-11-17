//! Simple thumbnails: a shaded top-down view of the model on the sample's preview background.

use campanion_mesh::Mesh;

use crate::error::{ResinError, Result};

/// Colours as 0..1 RGB.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PreviewStyle {
    pub back: [f64; 3],
    pub model: [f64; 3],
}

fn byte(v: f64) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Render `mesh` looking down -Z, fitted to `width` x `height` with a margin, and encode as PNG.
pub fn render_png(mesh: &Mesh, width: u32, height: u32, style: PreviewStyle) -> Result<Vec<u8>> {
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err(ResinError::invalid("bad preview size"));
    }
    let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("mesh: {err}")))?;
    let (min, max) = (bounds.min(), bounds.max());
    let (w, h) = (f64::from(width), f64::from(height));
    let margin = 0.1;
    let span_x = (max.x - min.x).max(1e-9);
    let span_y = (max.y - min.y).max(1e-9);
    let scale = (w * (1.0 - 2.0 * margin) / span_x).min(h * (1.0 - 2.0 * margin) / span_y);
    let off_x = (w - span_x * scale) / 2.0;
    let off_y = (h - span_y * scale) / 2.0;
    let to_px = |x: f64, y: f64| ((x - min.x) * scale + off_x, h - ((y - min.y) * scale + off_y));
    let mut depth = vec![f64::NEG_INFINITY; width as usize * height as usize];
    let back = [byte(style.back[0]), byte(style.back[1]), byte(style.back[2]), 255u8];
    let mut rgba = Vec::with_capacity(depth.len() * 4);
    for _ in 0..depth.len() {
        rgba.extend_from_slice(&back);
    }
    for tri in mesh.triangles() {
        let shade = tri.normal().map_or(0.4, |n| 0.35 + 0.65 * n.z.abs());
        let (a, b, c) = (to_px(tri.a.x, tri.a.y), to_px(tri.b.x, tri.b.y), to_px(tri.c.x, tri.c.y));
        let area = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
        if area.abs() < 1e-12 {
            continue;
        }
        let x0 = a.0.min(b.0).min(c.0).floor().max(0.0) as u32;
        let x1 = (a.0.max(b.0).max(c.0).ceil().min(w - 1.0)).max(0.0) as u32;
        let y0 = a.1.min(b.1).min(c.1).floor().max(0.0) as u32;
        let y1 = (a.1.max(b.1).max(c.1).ceil().min(h - 1.0)).max(0.0) as u32;
        for py in y0..=y1 {
            for px in x0..=x1 {
                let (sx, sy) = (f64::from(px) + 0.5, f64::from(py) + 0.5);
                let w0 = ((b.0 - sx) * (c.1 - sy) - (b.1 - sy) * (c.0 - sx)) / area;
                let w1 = ((c.0 - sx) * (a.1 - sy) - (c.1 - sy) * (a.0 - sx)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                    continue;
                }
                let z = w0 * tri.a.z + w1 * tri.b.z + w2 * tri.c.z;
                let idx = py as usize * width as usize + px as usize;
                if let Some(slot) = depth.get_mut(idx)
                    && z > *slot
                {
                    *slot = z;
                    let height = 0.55 + 0.45 * ((z - min.z) / (max.z - min.z).max(1e-9)).clamp(0.0, 1.0);
                    let k = shade * height;
                    let color = [byte(style.model[0] * k), byte(style.model[1] * k), byte(style.model[2] * k), 255u8];
                    if let Some(dst) = rgba.get_mut(idx * 4..idx * 4 + 4) {
                        dst.copy_from_slice(&color);
                    }
                }
            }
        }
    }
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().map_err(|err| ResinError::Image(err.to_string()))?;
        writer.write_image_data(&rgba).map_err(|err| ResinError::Image(err.to_string()))?;
    }
    Ok(out)
}
