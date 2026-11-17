//! Resin support and chamfered-raft generation.
//!
//! This module builds a watertight raft with a sloped perimeter and vertical pillar supports
//! under overhangs. It does not invent parameter values; every length and angle comes from
//! [`SupportParams`]. Branching tree supports, painted no-support zones, per-region settings,
//! scaffolding, and GUI editing are not implemented.

mod params;
mod place;
mod raft;
mod solid;

pub use params::{REQUIRED_FLAGS, SupportParams, SupportParamsOverlay, parse_support_params_json};
pub use place::{InputPoint, SupportPoint, is_overhang, overhangs_are_covered, parse_points_text, place_supports};
pub use raft::{RaftOutlines, edge_insets, raft_outlines};
pub use solid::{combine_meshes, is_watertight, signed_volume, translate_mesh};

use campanion_mesh::{Mesh, Vec3};

use crate::error::{ResinError, Result};
use crate::supports::solid::{RING_SIDES, loft_outlines, profile_solid};

/// The lifted part, the raft, the pillars, and the combined mesh.
#[derive(Clone, Debug)]
pub struct SupportedMesh {
    pub mesh: Mesh,
    pub part: Mesh,
    pub raft: Mesh,
    pub pillars: Mesh,
    pub points: Vec<SupportPoint>,
    pub outlines: RaftOutlines,
}

/// Lift `part` off the plate, add a chamfered raft, and place vertical supports.
///
/// The raft sits on z = 0. The lowest point of the part is at `raft thickness + part lift`.
/// When `auto` is true, overhangs at or above the threshold get supports. `extra` points are
/// always added. When `auto` is false, `extra` must not be empty.
pub fn generate_supports(part: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool) -> Result<SupportedMesh> {
    params.validate()?;
    if !auto && extra.is_empty() {
        return Err(ResinError::invalid("auto generation is off and no explicit --points were given"));
    }
    let bounds = part.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let lift_z = params.raft_thickness_mm + params.part_lift_mm - bounds.min().z;
    if !lift_z.is_finite() {
        return Err(ResinError::invalid("part lift is not finite"));
    }
    let part = translate_mesh(part, Vec3::new(0.0, 0.0, lift_z))?;
    let raft_top = params.raft_thickness_mm;
    let points = place_supports(&part, params, extra, auto, raft_top)?;
    for point in &points {
        let needed = raft_top + params.foot_height_mm + params.tip_length_mm;
        if point.contact_z + 1e-9 < needed {
            return Err(ResinError::invalid(format!(
                "support at ({}, {}) meets the part at z = {:.6} mm, which is below the {} mm needed for the foot and tip",
                point.x, point.y, point.contact_z, needed
            )));
        }
    }
    let mut footprint: Vec<(f64, f64)> = Vec::new();
    for triangle in part.triangles() {
        for vertex in [triangle.a, triangle.b, triangle.c] {
            footprint.push((vertex.x, vertex.y));
        }
    }
    for point in &points {
        footprint.push((point.x, point.y));
    }
    let outlines = raft_outlines(&footprint, params)?;
    let raft = loft_outlines(&outlines.bottom, &outlines.top, 0.0, params.raft_thickness_mm)?;
    let mut pillar_meshes = Vec::new();
    for point in &points {
        pillar_meshes.push(pillar_solid(point, params, raft_top)?);
    }
    let pillars = if pillar_meshes.is_empty() {
        Mesh::new(Vec::new()).map_err(|err| ResinError::invalid(format!("supports: {err}")))?
    } else {
        let refs: Vec<&Mesh> = pillar_meshes.iter().collect();
        combine_meshes(&refs)?
    };
    let mesh = combine_meshes(&[&part, &raft, &pillars])?;
    Ok(SupportedMesh { mesh, part, raft, pillars, points, outlines })
}

fn pillar_solid(point: &SupportPoint, params: &SupportParams, raft_top: f64) -> Result<Mesh> {
    let foot_r = params.foot_diameter_mm / 2.0;
    let pillar_r = params.pillar_diameter_mm / 2.0;
    let tip_r = params.tip_diameter_mm / 2.0;
    let mut rings: Vec<(f64, f64)> = Vec::new();
    let push = |rings: &mut Vec<(f64, f64)>, z: f64, r: f64| -> Result<()> {
        if let Some(&(last_z, last_r)) = rings.last() {
            if (z - last_z).abs() <= 1e-9 {
                if (r - last_r).abs() > 1e-9 {
                    return Err(ResinError::invalid("support profile has two radii at the same height"));
                }
                return Ok(());
            }
            if z < last_z {
                return Err(ResinError::invalid("support profile z values must increase"));
            }
        }
        rings.push((z, r));
        Ok(())
    };
    push(&mut rings, raft_top, foot_r)?;
    if params.foot_height_mm > 0.0 {
        push(&mut rings, raft_top + params.foot_height_mm, pillar_r)?;
    } else {
        // No foot cone: start the pillar at the raft with the pillar radius.
        rings.clear();
        push(&mut rings, raft_top, pillar_r)?;
    }
    push(&mut rings, point.contact_z - params.tip_length_mm, pillar_r)?;
    push(&mut rings, point.contact_z, tip_r)?;
    profile_solid(point.x, point.y, &rings, RING_SIDES)
}

/// Shaded 3/4 view of the supported part, from a low angle so the raft chamfer is visible.
pub fn render_supported_png(result: &SupportedMesh, width: u32, height: u32) -> Result<Vec<u8>> {
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err(ResinError::invalid("bad preview size"));
    }
    let bounds = result.mesh.bounds().map_err(|err| ResinError::invalid(format!("mesh: {err}")))?;
    let (min, max) = (bounds.min(), bounds.max());
    let center = Vec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5);
    let span = (max.x - min.x).max(max.y - min.y).max(max.z - min.z).max(1.0);
    // Look from the front-right, slightly above the plate, so the sloped raft edge reads clearly.
    let look = Vec3::new(-1.2, 1.45, -0.42);
    let look = look.normalized().ok_or_else(|| ResinError::invalid("preview camera is degenerate"))?;
    let world_up = Vec3::new(0.0, 0.0, 1.0);
    let right = look.cross(world_up).normalized().ok_or_else(|| ResinError::invalid("preview camera is degenerate"))?;
    let up = right.cross(look).normalized().ok_or_else(|| ResinError::invalid("preview camera is degenerate"))?;
    let (w, h) = (f64::from(width), f64::from(height));
    let scale = (w.min(h) * 0.82) / span;
    let to_px = |p: Vec3| -> (f64, f64, f64) {
        let d = p.sub(center);
        let x = d.dot(right) * scale + w * 0.5;
        let y = h * 0.5 - d.dot(up) * scale;
        let z = d.dot(look);
        (x, y, z)
    };
    let light = Vec3::new(0.35, -0.45, 0.82).normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let pixels = (width as usize).saturating_mul(height as usize);
    let mut depth = vec![f64::NEG_INFINITY; pixels];
    let back = [28u8, 30, 34, 255];
    let mut rgba = vec![0u8; pixels.saturating_mul(4)];
    for i in 0..pixels {
        if let Some(dst) = rgba.get_mut(i * 4..i * 4 + 4) {
            dst.copy_from_slice(&back);
        }
    }
    let draw = |mesh: &Mesh, color: [f64; 3], depth: &mut [f64], rgba: &mut [u8]| {
        for tri in mesh.triangles() {
            let shade = tri.normal().map_or(0.35, |n| 0.28 + 0.72 * n.dot(light).max(0.0));
            let (a, b, c) = (to_px(tri.a), to_px(tri.b), to_px(tri.c));
            let area = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
            if area.abs() < 1e-12 {
                continue;
            }
            let x0 = a.0.min(b.0).min(c.0).floor().max(0.0) as u32;
            let x1 = a.0.max(b.0).max(c.0).ceil().min(w - 1.0).max(0.0) as u32;
            let y0 = a.1.min(b.1).min(c.1).floor().max(0.0) as u32;
            let y1 = a.1.max(b.1).max(c.1).ceil().min(h - 1.0).max(0.0) as u32;
            for py in y0..=y1 {
                for px in x0..=x1 {
                    let (sx, sy) = (f64::from(px) + 0.5, f64::from(py) + 0.5);
                    let w0 = ((b.0 - sx) * (c.1 - sy) - (b.1 - sy) * (c.0 - sx)) / area;
                    let w1 = ((c.0 - sx) * (a.1 - sy) - (c.1 - sy) * (a.0 - sx)) / area;
                    let w2 = 1.0 - w0 - w1;
                    if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                        continue;
                    }
                    let z = w0 * a.2 + w1 * b.2 + w2 * c.2;
                    let idx = py as usize * width as usize + px as usize;
                    if let Some(slot) = depth.get_mut(idx)
                        && z > *slot
                    {
                        *slot = z;
                        let k = shade;
                        let pixel = [byte(color[0] * k), byte(color[1] * k), byte(color[2] * k), 255u8];
                        if let Some(dst) = rgba.get_mut(idx * 4..idx * 4 + 4) {
                            dst.copy_from_slice(&pixel);
                        }
                    }
                }
            }
        }
    };
    draw(&result.raft, [0.72, 0.74, 0.78], &mut depth, &mut rgba);
    draw(&result.pillars, [0.95, 0.72, 0.22], &mut depth, &mut rgba);
    draw(&result.part, [0.40, 0.74, 0.90], &mut depth, &mut rgba);
    encode_png(width, height, &rgba)
}

fn byte(v: f64) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

fn encode_png(width: u32, height: u32, rgba: &[u8]) -> Result<Vec<u8>> {
    let mut out = Vec::new();
    {
        let mut encoder = png::Encoder::new(&mut out, width, height);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().map_err(|err| ResinError::Image(err.to_string()))?;
        writer.write_image_data(rgba).map_err(|err| ResinError::Image(err.to_string()))?;
    }
    Ok(out)
}
