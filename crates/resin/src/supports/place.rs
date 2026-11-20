//! Overhang detection and vertical pillar placement.

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};
use crate::supports::grid::VerticalHitGrid;
use crate::supports::params::SupportParams;
use crate::supports::profile;

/// Most auto-generated plus explicit support points accepted in one run.
pub const MAX_SUPPORT_POINTS: usize = 20_000;

/// How a support point was created.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PointSource {
    /// Auto-placed on an overhang.
    #[default]
    Overhang,
    /// User-supplied (points file or manual edit).
    Manual,
    /// Auto-placed on a first-appearance island.
    Island,
    /// Added by the optimizer to meet the holding-force check.
    Optimizer,
}

/// A contact where a support tip meets the part.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SupportPoint {
    pub x: f64,
    pub y: f64,
    pub contact_z: f64,
    pub source: PointSource,
    /// Index into the user's tip palette, when a class has been assigned.
    pub preset_index: Option<u16>,
}

impl SupportPoint {
    /// A point with no palette assignment. `source` defaults to overhang (the historical auto path).
    pub fn new(x: f64, y: f64, contact_z: f64) -> Self {
        Self { x, y, contact_z, source: PointSource::Overhang, preset_index: None }
    }

    /// A user-supplied contact.
    pub fn manual(x: f64, y: f64, contact_z: f64) -> Self {
        Self { x, y, contact_z, source: PointSource::Manual, preset_index: None }
    }
}

/// An explicit point from a file or from the GUI. `z` is used when present; otherwise the lowest downward hit is taken.
#[derive(Clone, Debug, PartialEq)]
pub struct InputPoint {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
    /// Palette class name, when the points file named one.
    pub preset: Option<String>,
    pub source: PointSource,
}

impl InputPoint {
    /// A manual point with no palette class.
    pub fn new(x: f64, y: f64, z: Option<f64>) -> Self {
        Self { x, y, z, preset: None, source: PointSource::Manual }
    }
}

/// A downward-facing triangle whose angle from vertical is at or above the overhang threshold.
pub fn is_overhang(triangle: &Triangle, overhang_angle_deg: f64) -> bool {
    let Some(normal) = triangle.normal() else {
        return false;
    };
    if normal.z >= 0.0 {
        return false;
    }
    let from_vertical = (-normal.z).clamp(0.0, 1.0).asin().to_degrees();
    from_vertical + 1e-9 >= overhang_angle_deg
}

/// Shoot a ray `origin + t * dir` (`dir` need not be unit). Returns the smallest `t >= 0` and the triangle index.
pub fn first_hit(mesh: &Mesh, origin: Vec3, dir: Vec3, skip: Option<usize>) -> Option<(f64, usize)> {
    if dir.length_sq() <= 0.0 {
        return None;
    }
    let mut best: Option<(f64, usize)> = None;
    let mut tested = 0u64;
    for (index, triangle) in mesh.triangles().iter().enumerate() {
        tested += 1;
        if skip == Some(index) {
            continue;
        }
        let Some(t) = ray_triangle(origin, dir, triangle) else {
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
    profile::record_hit(tested);
    best
}

/// Möller–Trumbore. Returns `t` along `dir` when the hit is in the triangle and `t >= 0`.
pub(crate) fn ray_triangle(origin: Vec3, dir: Vec3, tri: &Triangle) -> Option<f64> {
    let eps = 1e-12;
    let edge1 = tri.b.sub(tri.a);
    let edge2 = tri.c.sub(tri.a);
    let pvec = dir.cross(edge2);
    let det = edge1.dot(pvec);
    if det.abs() < eps {
        return None;
    }
    let inv = 1.0 / det;
    let tvec = origin.sub(tri.a);
    let u = tvec.dot(pvec) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(edge1);
    let v = dir.dot(qvec) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = edge2.dot(qvec) * inv;
    if t.is_finite() && t >= 0.0 { Some(t) } else { None }
}

/// Lowest downward face of `mesh` at `(x, y)` with `z` strictly above `z_min`.
pub fn lowest_hit_above(mesh: &Mesh, x: f64, y: f64, z_min: f64) -> Option<(Vec3, usize)> {
    lowest_hit_above_with(mesh, None, x, y, z_min)
}

fn lowest_hit_above_with(mesh: &Mesh, grid: Option<&VerticalHitGrid>, x: f64, y: f64, z_min: f64) -> Option<(Vec3, usize)> {
    match grid {
        Some(grid) => grid.lowest_hit_above(mesh, x, y, z_min),
        None => {
            let origin = Vec3::new(x, y, z_min);
            let dir = Vec3::new(0.0, 0.0, 1.0);
            let (t, index) = first_hit(mesh, origin, dir, None)?;
            let z = z_min + t;
            if !z.is_finite() {
                return None;
            }
            Some((Vec3::new(x, y, z), index))
        }
    }
}

/// The part already occupies the column below `point` when a downward ray hits another triangle.
pub fn is_self_supported(mesh: &Mesh, point: Vec3, source: Option<usize>) -> bool {
    is_self_supported_with(mesh, None, point, source)
}

fn is_self_supported_with(mesh: &Mesh, grid: Option<&VerticalHitGrid>, point: Vec3, source: Option<usize>) -> bool {
    match grid {
        Some(grid) => grid.is_self_supported(mesh, point, source),
        None => {
            // Start a short way below the surface so the source face and its neighbours are skipped.
            let origin = Vec3::new(point.x, point.y, point.z - 0.05);
            matches!(first_hit(mesh, origin, Vec3::new(0.0, 0.0, -1.0), source), Some((t, _)) if t > 1e-6)
        }
    }
}

fn sample_triangle(triangle: Triangle, spacing: f64, out: &mut Vec<(Vec3, usize)>, index: usize) -> Result<()> {
    let ab = triangle.b.sub(triangle.a).length();
    let bc = triangle.c.sub(triangle.b).length();
    let ca = triangle.a.sub(triangle.c).length();
    let max_edge = ab.max(bc).max(ca);
    if !max_edge.is_finite() {
        return Err(ResinError::invalid("overhang triangle has a non-finite edge"));
    }
    let n = ((max_edge / spacing).ceil() as usize).max(1);
    if n > 10_000 {
        return Err(ResinError::invalid("support spacing is too small for this overhang"));
    }
    for i in 0..=n {
        for j in 0..=(n - i) {
            let k = n - i - j;
            let s = n as f64;
            let p = triangle.a.scale(i as f64 / s).add(triangle.b.scale(j as f64 / s)).add(triangle.c.scale(k as f64 / s));
            out.push((p, index));
        }
    }
    Ok(())
}

/// Auto-generate contact points on overhangs, then add any explicit points, thinned to `spacing`.
pub fn place_supports(mesh: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool, z_min: f64) -> Result<Vec<SupportPoint>> {
    let grid = VerticalHitGrid::build(mesh)?;
    place_supports_with(mesh, Some(&grid), params, extra, auto, z_min)
}

/// Linear-scan placement used as the correctness reference for the XY hit grid.
#[cfg(test)]
pub(crate) fn place_supports_linear(mesh: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool, z_min: f64) -> Result<Vec<SupportPoint>> {
    place_supports_with(mesh, None, params, extra, auto, z_min)
}

fn place_supports_with(
    mesh: &Mesh,
    grid: Option<&VerticalHitGrid>,
    params: &SupportParams,
    extra: &[InputPoint],
    auto: bool,
    z_min: f64,
) -> Result<Vec<SupportPoint>> {
    profile::reset_hits();
    let t_place = std::time::Instant::now();
    let mut samples: Vec<(Vec3, usize)> = Vec::new();
    if auto {
        let t_sample = std::time::Instant::now();
        for (index, triangle) in mesh.triangles().iter().enumerate() {
            if !is_overhang(triangle, params.overhang_angle_deg) {
                continue;
            }
            sample_triangle(*triangle, params.support_spacing_mm, &mut samples, index)?;
        }
        profile::log("place sample overhangs", t_sample);
        profile::log_count("place overhang samples", samples.len() as u64);
    }
    let mut needed: Vec<SupportPoint> = Vec::new();
    let t_self = std::time::Instant::now();
    for (point, index) in samples {
        if is_self_supported_with(mesh, grid, point, Some(index)) {
            continue;
        }
        needed.push(SupportPoint::new(point.x, point.y, point.z));
    }
    profile::log("place self-support rays", t_self);
    profile::log_count("place needed before extras", needed.len() as u64);
    for point in extra {
        if !point.x.is_finite() || !point.y.is_finite() {
            return Err(ResinError::invalid("a support point is not finite"));
        }
        let contact = match point.z {
            Some(z) => {
                if !z.is_finite() {
                    return Err(ResinError::invalid("a support point is not finite"));
                }
                {
                    let mut contact = SupportPoint::manual(point.x, point.y, z);
                    contact.source = point.source;
                    contact.preset_index = None;
                    contact
                }
            }
            None => match lowest_hit_above_with(mesh, grid, point.x, point.y, z_min) {
                Some((hit, _)) => {
                    let mut contact = SupportPoint::manual(hit.x, hit.y, hit.z);
                    contact.source = point.source;
                    contact
                }
                None => return Err(ResinError::invalid(format!("explicit support at ({}, {}) does not hit the part", point.x, point.y))),
            },
        };
        if contact.contact_z <= z_min {
            return Err(ResinError::invalid("explicit support contact is at or below the raft"));
        }
        needed.push(contact);
    }
    if needed.is_empty() && auto {
        return Ok(Vec::new());
    }
    if needed.is_empty() {
        return Err(ResinError::invalid("no support points were placed; give --points or enable auto generation"));
    }
    let t_thin = std::time::Instant::now();
    let thinned = thin_points(&needed, params.support_spacing_mm)?;
    profile::log("place thin", t_thin);
    profile::log_hits();
    profile::log("place total", t_place);
    if thinned.len() > MAX_SUPPORT_POINTS {
        return Err(ResinError::invalid(format!("too many supports: {} > {MAX_SUPPORT_POINTS}; increase --support-spacing", thinned.len())));
    }
    profile::log_count("place kept", thinned.len() as u64);
    Ok(thinned)
}

pub(crate) fn thin_points(points: &[SupportPoint], spacing: f64) -> Result<Vec<SupportPoint>> {
    thin_points_with(points, spacing, None)
}

/// Thin points to `spacing`. When `band` is `Some((z_cut, band_spacing))`, points at or below
/// `z_cut` are thinned to `band_spacing` and are considered first so the lowest band stays denser.
pub fn thin_points_with(points: &[SupportPoint], spacing: f64, band: Option<(f64, f64)>) -> Result<Vec<SupportPoint>> {
    if !spacing.is_finite() || spacing <= 0.0 {
        return Err(ResinError::invalid("support spacing must be greater than 0 mm"));
    }
    if let Some((z_cut, band_spacing)) = band {
        if !z_cut.is_finite() {
            return Err(ResinError::invalid("lowest-band height is not finite"));
        }
        if !band_spacing.is_finite() || band_spacing <= 0.0 {
            return Err(ResinError::invalid("lowest-band spacing must be greater than 0 mm"));
        }
        let mut low: Vec<SupportPoint> = points.iter().copied().filter(|p| p.contact_z <= z_cut).collect();
        let mut high: Vec<SupportPoint> = points.iter().copied().filter(|p| p.contact_z > z_cut).collect();
        let mut kept = thin_sorted(&mut low, band_spacing)?;
        kept.extend(thin_against(&mut high, spacing, &kept)?);
        return Ok(kept);
    }
    let mut ordered = points.to_vec();
    thin_sorted(&mut ordered, spacing)
}

fn sort_points(points: &mut [SupportPoint]) {
    points.sort_by(|a, b| {
        a.contact_z.partial_cmp(&b.contact_z).unwrap_or(std::cmp::Ordering::Equal).then_with(|| {
            a.x.partial_cmp(&b.x).unwrap_or(std::cmp::Ordering::Equal).then_with(|| a.y.partial_cmp(&b.y).unwrap_or(std::cmp::Ordering::Equal))
        })
    });
}

fn thin_sorted(points: &mut [SupportPoint], spacing: f64) -> Result<Vec<SupportPoint>> {
    sort_points(points);
    thin_against(points, spacing, &[])
}

fn thin_against(points: &mut [SupportPoint], spacing: f64, existing: &[SupportPoint]) -> Result<Vec<SupportPoint>> {
    if !spacing.is_finite() || spacing <= 0.0 {
        return Err(ResinError::invalid("support spacing must be greater than 0 mm"));
    }
    sort_points(points);
    let spacing_sq = spacing * spacing;
    let mut kept: Vec<SupportPoint> = Vec::new();
    for point in points.iter().copied() {
        let close_existing = existing.iter().any(|other| {
            let dx = point.x - other.x;
            let dy = point.y - other.y;
            dx * dx + dy * dy <= spacing_sq
        });
        let close_kept = kept.iter().any(|other| {
            let dx = point.x - other.x;
            let dy = point.y - other.y;
            dx * dx + dy * dy <= spacing_sq
        });
        if !close_existing && !close_kept {
            kept.push(point);
        }
    }
    Ok(kept)
}

/// Every needed overhang sample is within `spacing` of a placed support, or is self-supported.
pub fn overhangs_are_covered(mesh: &Mesh, params: &SupportParams, points: &[SupportPoint], z_min: f64) -> Result<bool> {
    let _ = z_min;
    let grid = VerticalHitGrid::build(mesh)?;
    let spacing_sq = params.support_spacing_mm * params.support_spacing_mm;
    for (index, triangle) in mesh.triangles().iter().enumerate() {
        if !is_overhang(triangle, params.overhang_angle_deg) {
            continue;
        }
        let mut samples = Vec::new();
        sample_triangle(*triangle, params.support_spacing_mm, &mut samples, index)?;
        for (point, source) in samples {
            if is_self_supported_with(mesh, Some(&grid), point, Some(source)) {
                continue;
            }
            let covered = points.iter().any(|s| {
                let dx = point.x - s.x;
                let dy = point.y - s.y;
                dx * dx + dy * dy <= spacing_sq + 1e-12
            });
            if !covered {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Parse a points file: one `x y`, `x y z`, or `x y z preset` per line, millimetres.
///
/// Blank lines and `#` comments are ignored. The optional fourth token is a palette class name
/// (no spaces). Older two- and three-column files still load.
pub fn parse_points_text(text: &str) -> Result<Vec<InputPoint>> {
    let mut points = Vec::new();
    for (index, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(x) = parts.next() else {
            continue;
        };
        let Some(y) = parts.next() else {
            return Err(ResinError::invalid(format!("support point on line {} needs at least x and y", index + 1)));
        };
        let z = parts.next();
        let preset = parts.next();
        if parts.next().is_some() {
            return Err(ResinError::invalid(format!("support point on line {} has more than four values", index + 1)));
        }
        let parse = |s: &str, axis: &str| -> Result<f64> {
            s.parse::<f64>()
                .ok()
                .filter(|v| v.is_finite())
                .ok_or_else(|| ResinError::invalid(format!("support point on line {} has a bad {axis} value `{s}`", index + 1)))
        };
        let preset = match preset {
            Some("") => None,
            Some(name) => Some(name.to_string()),
            None => None,
        };
        // A three-token line is `x y z`. A four-token line is `x y z preset`.
        points.push(InputPoint {
            x: parse(x, "x")?,
            y: parse(y, "y")?,
            z: z.map(|s| parse(s, "z")).transpose()?,
            preset,
            source: PointSource::Manual,
        });
        if points.len() > MAX_SUPPORT_POINTS {
            return Err(ResinError::invalid(format!("too many support points: {} > {MAX_SUPPORT_POINTS}", points.len())));
        }
    }
    Ok(points)
}

/// Write a points file that [`parse_points_text`] will read back.
///
/// When `preset_names` is set, a fourth column is written for points that have a palette index.
pub fn write_points_text(points: &[SupportPoint], preset_names: Option<&[String]>) -> String {
    let mut out = String::from("# campanion support points, millimetres\n# columns: x y [z] [preset]\n");
    for point in points {
        let preset = point.preset_index.and_then(|index| preset_names.and_then(|names| names.get(index as usize)).cloned());
        match preset {
            Some(name) => {
                out.push_str(&format!("{:.6} {:.6} {:.6} {name}\n", point.x, point.y, point.contact_z));
            }
            None => out.push_str(&format!("{:.6} {:.6} {:.6}\n", point.x, point.y, point.contact_z)),
        }
    }
    out
}

/// Add a manual point at `(x, y)`, using `z` when given, otherwise the lowest hit above `z_min`.
pub fn add_support_point(points: &mut Vec<SupportPoint>, mesh: &Mesh, x: f64, y: f64, z: Option<f64>, z_min: f64) -> Result<usize> {
    if !x.is_finite() || !y.is_finite() {
        return Err(ResinError::invalid("a support point is not finite"));
    }
    let contact = match z {
        Some(z) if z.is_finite() => SupportPoint::manual(x, y, z),
        Some(_) => return Err(ResinError::invalid("a support point is not finite")),
        None => match lowest_hit_above(mesh, x, y, z_min) {
            Some((hit, _)) => SupportPoint::manual(hit.x, hit.y, hit.z),
            None => return Err(ResinError::invalid(format!("support at ({x}, {y}) does not hit the part"))),
        },
    };
    if contact.contact_z <= z_min {
        return Err(ResinError::invalid("explicit support contact is at or below the raft"));
    }
    if points.len() >= MAX_SUPPORT_POINTS {
        return Err(ResinError::invalid(format!("too many supports: {} > {MAX_SUPPORT_POINTS}", points.len() + 1)));
    }
    points.push(contact);
    Ok(points.len() - 1)
}

/// Move the point at `index` to `(x, y)`, re-resolving Z when `z` is `None`.
pub fn move_support_point(points: &mut [SupportPoint], index: usize, mesh: &Mesh, x: f64, y: f64, z: Option<f64>, z_min: f64) -> Result<()> {
    let Some(existing) = points.get(index).copied() else {
        return Err(ResinError::invalid(format!("support point {index} does not exist")));
    };
    let mut scratch = Vec::new();
    add_support_point(&mut scratch, mesh, x, y, z, z_min)?;
    let Some(mut moved) = scratch.pop() else {
        return Err(ResinError::invalid("failed to move support point"));
    };
    moved.source = existing.source;
    moved.preset_index = existing.preset_index;
    if let Some(slot) = points.get_mut(index) {
        *slot = moved;
    }
    Ok(())
}

/// Remove the point at `index`.
pub fn delete_support_point(points: &mut Vec<SupportPoint>, index: usize) -> Result<SupportPoint> {
    if index >= points.len() {
        return Err(ResinError::invalid(format!("support point {index} does not exist")));
    }
    Ok(points.remove(index))
}
