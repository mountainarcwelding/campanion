//! Resin support and chamfered-raft generation.
//!
//! This module builds a watertight raft with a sloped perimeter and vertical pillar supports.
//! It does not invent parameter values; every length and angle comes from [`SupportParams`] or,
//! for the optimizer, from a user calibration profile. The undercut chamfered raft (narrow on
//! the bed) is unchanged. Painted support and no-support zones, cross-bracing, tip penetration and
//! user-saved tip presets are optional and take their values from the user. Branching tree
//! supports and scaffolding are not implemented.

mod bracing;
mod calibration;
mod cancel;
mod classify;
mod grid;
mod islands;
mod optimize;
mod params;
mod place;
mod pose;
mod presets;
mod profile;
mod raft;
mod solid;
mod zones;

pub use bracing::{BracingParams, MAX_STRUTS, Shaft, Strut, plan_braces, spanning_links, strut_solid};
pub use calibration::{
    CALIBRATION_SCHEMA, CalibratedTip, CalibrationMeasurement, CalibrationProfile, MissingConstant, REQUIRED_CONSTANTS, REQUIRED_TIP_FIELDS,
    STANDARD_GRAVITY_M_S2, constant_label, import_calibration, load_calibration, missing_constants_in, parse_calibration,
};
pub use cancel::{CancelFlag, GenerateProgress};
pub use classify::{ClassifiedRegion, ClassifyThresholds, FeatureClass, classify_features};
pub use islands::{
    DetectGrid, DetectedFeatures, Island, IslandDetectParams, LayerStats, MAX_DETECT_LAYERS, MAX_GRID_SIDE, detect_islands, detect_islands_with,
    island_fill_points, thin_components,
};
pub use optimize::{ForceBreakdown, OptimizeInput, OptimizeResult, optimize_supports, place_with_islands};
pub use params::{REQUIRED_FLAGS, SupportParams, SupportParamsOverlay, parse_support_params_json};
pub use place::{
    InputPoint, PointSource, SupportPoint, add_support_point, delete_support_point, is_overhang, move_support_point, overhangs_are_covered,
    parse_points_text, place_supports, thin_points_with, write_points_text,
};
pub use pose::{MeshPose, apply_mesh_pose, center_on_plate, drop_to_plate, snap_to_step};
pub use presets::{
    MAX_PRESETS, PRESETS_SCHEMA, PresetMix, SupportRole, TipGeometry, TipPreset, apply_mix, check_penetration, forget_preset, parse_presets,
    preset_index, resolve_named_points, role_of, write_presets,
};
pub use raft::{RaftOutlines, edge_insets, raft_outlines};
pub use solid::{combine_meshes, is_watertight, signed_volume, translate_mesh};
pub use zones::{FaceZones, ZONES_SCHEMA, ZoneMark, ZoneReport, apply_zones, remove_blocked};

use campanion_mesh::{Mesh, Vec3};

use crate::error::{ResinError, Result};
use crate::supports::presets::params_for_point;
use crate::supports::solid::{RING_SIDES, loft_outlines, profile_solid};

/// The lifted part, the raft, the pillars, and the combined mesh.
#[derive(Clone, Debug)]
pub struct SupportedMesh {
    pub mesh: Mesh,
    pub part: Mesh,
    pub raft: Mesh,
    pub pillars: Mesh,
    /// Cross-bracing struts, empty when bracing is off.
    pub braces: Mesh,
    pub points: Vec<SupportPoint>,
    pub outlines: RaftOutlines,
    /// Island detection results, when detection was requested.
    pub features: Option<DetectedFeatures>,
    /// Holding-force breakdown, when the optimizer ran.
    pub optimize: Option<ForceBreakdown>,
    pub warnings: Vec<String>,
    /// Geometry that each point's `preset_index` refers to, and the names shown for it.
    pub palette: Vec<TipGeometry>,
    pub palette_names: Vec<String>,
    /// What painted zones changed, when zones were given.
    pub zone_report: Option<ZoneReport>,
    /// Struts built, and struts left out because they would touch the part.
    pub brace_count: usize,
    pub braces_skipped: usize,
}

/// Optional solid-building settings. Each is off when `None`; nothing is assumed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SolidOptions {
    /// Depth the tip continues straight into the part past the contact, millimetres.
    pub tip_penetration_mm: Option<f64>,
    pub bracing: Option<BracingParams>,
}

impl SolidOptions {
    pub fn validate(&self) -> Result<()> {
        if let Some(depth) = self.tip_penetration_mm {
            check_penetration(depth)?;
        }
        if let Some(bracing) = &self.bracing {
            bracing.validate()?;
        }
        Ok(())
    }
}

/// Options for [`generate_supports_with`]. Uniform and manual modes work without a calibration profile.
#[derive(Clone, Debug)]
pub struct GenerateOptions<'a> {
    pub auto: bool,
    pub extra: &'a [InputPoint],
    /// When set, first-appearance islands are detected and supported (unless ignored by area).
    pub detect: Option<IslandDetectParams>,
    pub island_ignore_area_mm2: Option<f64>,
    pub lowest_band_height_mm: Option<f64>,
    pub lowest_band_spacing_mm: Option<f64>,
    /// When true, the optimizer runs and requires a complete [`calibration`].
    pub optimize: bool,
    pub calibration: Option<&'a CalibrationProfile>,
    /// When set, generation stops at the next layer or stage check.
    pub cancel: Option<&'a CancelFlag>,
    /// When set, the worker publishes the current stage and layer index.
    pub progress: Option<&'a GenerateProgress>,
    /// Painted zones, one mark per part triangle.
    pub zones: Option<&'a FaceZones>,
    /// XY spacing inside painted support zones. Required when any support face is painted.
    pub zone_spacing_mm: Option<f64>,
    /// User-saved tip presets that the mix and points-file names refer to.
    pub presets: &'a [TipPreset],
    /// Which preset each kind of support uses. Not allowed with the optimizer, which picks its own tips.
    pub mix: Option<&'a PresetMix>,
    pub solids: SolidOptions,
}

impl<'a> GenerateOptions<'a> {
    pub fn manual(extra: &'a [InputPoint]) -> Self {
        Self {
            auto: false,
            extra,
            detect: None,
            island_ignore_area_mm2: None,
            lowest_band_height_mm: None,
            lowest_band_spacing_mm: None,
            optimize: false,
            calibration: None,
            cancel: None,
            progress: None,
            zones: None,
            zone_spacing_mm: None,
            presets: &[],
            mix: None,
            solids: SolidOptions::default(),
        }
    }

    pub fn auto(extra: &'a [InputPoint]) -> Self {
        let mut options = Self::manual(extra);
        options.auto = true;
        options
    }
}

/// Lift `part` off the plate, add a chamfered raft, and place vertical supports.
///
/// The raft sits on z = 0. The lowest point of the part is at `raft thickness + part lift`.
/// When `auto` is true, overhangs at or above the threshold get supports. `extra` points are
/// always added. When `auto` is false, `extra` must not be empty.
///
/// This path is uniform / manual and does not require a calibration profile. For island-aware
/// placement or the optimizer, use [`generate_supports_with`].
pub fn generate_supports(part: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool) -> Result<SupportedMesh> {
    generate_supports_with(part, params, if auto { GenerateOptions::auto(extra) } else { GenerateOptions::manual(extra) })
}

/// Lift `part`, add the undercut chamfered raft, and place supports according to `options`.
pub fn generate_supports_with(part: &Mesh, params: &SupportParams, options: GenerateOptions<'_>) -> Result<SupportedMesh> {
    params.validate()?;
    options.solids.validate()?;
    if options.optimize && options.mix.is_some_and(|mix| !mix.is_empty()) {
        return Err(ResinError::invalid(
            "the optimizer picks its own tips from the calibration palette; clear the preset mix or turn the optimizer off",
        ));
    }
    if let Some(mix) = options.mix {
        mix.check(options.presets)?;
    }
    let painted_support = options.zones.is_some_and(|zones| zones.counts().0 > 0);
    if painted_support {
        match options.zone_spacing_mm {
            Some(spacing) if spacing.is_finite() && spacing > 0.0 => {}
            Some(_) => return Err(ResinError::invalid("painted zone spacing must be greater than 0 mm")),
            None => return Err(ResinError::invalid("support zones are painted; enter the painted zone spacing")),
        }
    }
    if options.optimize && options.calibration.is_none() {
        return Err(ResinError::invalid(
            "optimizer requires a complete calibration profile; load a profile or use uniform / manual supports without --optimize",
        ));
    }
    if !options.auto && options.extra.is_empty() && !options.optimize && !painted_support {
        return Err(ResinError::invalid("auto generation is off and no explicit --points were given"));
    }
    match (options.lowest_band_height_mm, options.lowest_band_spacing_mm) {
        (None, None) => {}
        (Some(height), Some(spacing)) => {
            if !height.is_finite() || height <= 0.0 {
                return Err(ResinError::invalid("lowest-band height must be greater than 0 mm"));
            }
            if !spacing.is_finite() || spacing <= 0.0 {
                return Err(ResinError::invalid("lowest-band spacing must be greater than 0 mm"));
            }
        }
        _ => return Err(ResinError::invalid("lowest-band height and lowest-band spacing must be supplied together")),
    }
    if let Some(area) = options.island_ignore_area_mm2
        && (!area.is_finite() || area < 0.0)
    {
        return Err(ResinError::invalid("island ignore area must be 0 mm² or greater"));
    }
    crate::supports::cancel::check_cancel(options.cancel)?;
    if let Some(progress) = options.progress {
        progress.set_stage("Lifting part");
        progress.set_layer(0);
        progress.set_layers(0);
    }
    let bounds = part.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let lift_z = params.raft_thickness_mm + params.part_lift_mm - bounds.min().z;
    if !lift_z.is_finite() {
        return Err(ResinError::invalid("part lift is not finite"));
    }
    let t_gen = std::time::Instant::now();
    let part = translate_mesh(part, Vec3::new(0.0, 0.0, lift_z))?;
    if let Some(zones) = options.zones {
        zones.check_mesh(&part)?;
    }
    crate::supports::profile::log("lift part", t_gen);
    crate::supports::cancel::check_cancel(options.cancel)?;
    let raft_top = params.raft_thickness_mm;
    let band = match (options.lowest_band_height_mm, options.lowest_band_spacing_mm) {
        (Some(height), Some(spacing)) => Some((height, spacing)),
        _ => None,
    };
    let (mut points, features, mut optimize, mut warnings) = if options.optimize {
        let calibration = options.calibration.ok_or_else(|| {
            ResinError::invalid(
                "optimizer requires a complete calibration profile; load a profile or use uniform / manual supports without --optimize",
            )
        })?;
        let detect = options
            .detect
            .ok_or_else(|| ResinError::invalid("optimizer requires --layer-height and a detection cell size (--detect-cell or --support-spacing)"))?;
        let result = optimize_supports(OptimizeInput {
            mesh: &part,
            params,
            detect,
            calibration,
            extra: options.extra,
            lowest_band_height_mm: options.lowest_band_height_mm,
            lowest_band_spacing_mm: options.lowest_band_spacing_mm,
            island_ignore_area_mm2: options.island_ignore_area_mm2,
        })?;
        (result.points, Some(result.features), Some(result.forces), result.warnings)
    } else if let Some(detect) = options.detect {
        if let Some(progress) = options.progress {
            progress.set_stage("Detecting islands");
        }
        let t_detect = std::time::Instant::now();
        let features = detect_islands_with(&part, &detect, options.cancel, options.progress)?;
        crate::supports::profile::log("generate detect", t_detect);
        crate::supports::cancel::check_cancel(options.cancel)?;
        if let Some(progress) = options.progress {
            progress.set_stage("Placing supports");
        }
        let t_place = std::time::Instant::now();
        let points = place_with_islands(&part, params, &features, options.extra, options.auto, options.island_ignore_area_mm2, band, raft_top)?;
        crate::supports::profile::log("generate place", t_place);
        (points, Some(features), None, Vec::new())
    } else {
        crate::supports::cancel::check_cancel(options.cancel)?;
        if let Some(progress) = options.progress {
            progress.set_stage("Placing supports");
        }
        let mut points = if !options.auto && options.extra.is_empty() {
            Vec::new()
        } else {
            place_supports(&part, params, options.extra, options.auto, raft_top)?
        };
        if let Some((height, spacing)) = band {
            let z_cut = part.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?.min().z + height;
            points = thin_points_with(&points, params.support_spacing_mm, Some((z_cut, spacing)))?;
        }
        (points, None, None, Vec::new())
    };
    crate::supports::cancel::check_cancel(options.cancel)?;
    let mut zone_report = None;
    if let Some(zones) = options.zones
        && !zones.is_empty()
    {
        if let Some(progress) = options.progress {
            progress.set_stage("Applying painted zones");
        }
        let t_zones = std::time::Instant::now();
        let spacing = options.zone_spacing_mm.unwrap_or(params.support_spacing_mm);
        let (kept, report) = apply_zones(&part, zones, points, spacing, raft_top)?;
        points = kept;
        crate::supports::profile::log("generate zones", t_zones);
        if report.blocked_removed > 0 {
            warnings.push(format!("{} automatic support(s) removed by no-support zones", report.blocked_removed));
        }
        if report.painted_unreachable > 0 {
            warnings.push(format!(
                "{} painted support face(s) face up or sideways, or have part below them, and cannot take a vertical support",
                report.painted_unreachable
            ));
        }
        zone_report = Some(report);
    }
    let (palette, palette_names): (Vec<TipGeometry>, Vec<String>) = if options.optimize {
        let tips = options.calibration.map(|profile| profile.tip_palette.as_slice()).unwrap_or(&[]);
        (tips.iter().map(TipGeometry::from).collect(), tips.iter().map(|tip| tip.name.clone()).collect())
    } else {
        (options.presets.iter().map(TipGeometry::from).collect(), options.presets.iter().map(|preset| preset.name.clone()).collect())
    };
    if options.optimize {
        if let Some(forces) = optimize.as_mut()
            && zone_report.is_some()
        {
            let chosen = palette_names.iter().position(|name| *name == forces.preset_name);
            let per_tip = options
                .calibration
                .and_then(|profile| chosen.and_then(|index| profile.tip_palette.get(index)))
                .map(|tip| (tip.tip_capacity_n, tip.contact_area_mm2));
            if let (Some(index), Some((capacity, area))) = (chosen, per_tip) {
                for point in points.iter_mut().filter(|p| p.preset_index.is_none()) {
                    point.preset_index = u16::try_from(index).ok();
                }
                forces.tip_count = points.len();
                forces.capacity_n = points.len() as f64 * capacity;
                forces.scar_area_mm2 = points.len() as f64 * area;
                forces.sufficient = forces.capacity_n + 1e-12 >= forces.demand_n;
                if !forces.sufficient {
                    warnings.push(format!(
                        "after painted zones, capacity {:.4} N is below the demand {:.4} N; paint fewer no-support faces or use a stronger tip",
                        forces.capacity_n, forces.demand_n
                    ));
                }
            }
        }
    } else {
        if !options.presets.is_empty() {
            resolve_named_points(&mut points, options.extra, options.presets)?;
        }
        if let Some(mix) = options.mix {
            let band_z = match (options.lowest_band_height_mm, part.bounds()) {
                (Some(height), Ok(bounds)) => Some(bounds.min().z + height),
                _ => None,
            };
            apply_mix(&mut points, mix, options.presets, band_z)?;
        }
    }
    if points.is_empty() && !options.auto && !options.optimize {
        return Err(ResinError::invalid("no support points were placed; paint a reachable support zone, give --points, or enable auto generation"));
    }
    crate::supports::cancel::check_cancel(options.cancel)?;
    if let Some(progress) = options.progress {
        progress.set_stage("Building solids");
    }
    let t_assemble = std::time::Instant::now();
    let mut out = assemble_supported(part, params, points, features, optimize, warnings, &palette, &options.solids)?;
    out.palette_names = palette_names;
    out.zone_report = zone_report;
    crate::supports::profile::log("generate assemble", t_assemble);
    crate::supports::profile::log("generate total", t_gen);
    Ok(out)
}

/// Lift `part` and rebuild the raft and pillars from already-placed `points`.
///
/// Point Z values must already be in the lifted frame used by [`generate_supports_with`].
pub fn rebuild_supported(part: &Mesh, params: &SupportParams, points: &[SupportPoint], palette: &[CalibratedTip]) -> Result<SupportedMesh> {
    let geometry: Vec<TipGeometry> = palette.iter().map(TipGeometry::from).collect();
    let names: Vec<String> = palette.iter().map(|tip| tip.name.clone()).collect();
    rebuild_supported_with(part, params, points, &geometry, &names, &SolidOptions::default())
}

/// [`rebuild_supported`] with an explicit tip palette and optional penetration and bracing.
pub fn rebuild_supported_with(
    part: &Mesh,
    params: &SupportParams,
    points: &[SupportPoint],
    palette: &[TipGeometry],
    palette_names: &[String],
    solids: &SolidOptions,
) -> Result<SupportedMesh> {
    params.validate()?;
    solids.validate()?;
    let bounds = part.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let lift_z = params.raft_thickness_mm + params.part_lift_mm - bounds.min().z;
    if !lift_z.is_finite() {
        return Err(ResinError::invalid("part lift is not finite"));
    }
    let t_gen = std::time::Instant::now();
    let part = translate_mesh(part, Vec3::new(0.0, 0.0, lift_z))?;
    crate::supports::profile::log("lift part", t_gen);
    let mut out = assemble_supported(part, params, points.to_vec(), None, None, Vec::new(), palette, solids)?;
    out.palette_names = palette_names.to_vec();
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn assemble_supported(
    part: Mesh,
    params: &SupportParams,
    points: Vec<SupportPoint>,
    features: Option<DetectedFeatures>,
    optimize: Option<ForceBreakdown>,
    mut warnings: Vec<String>,
    palette: &[TipGeometry],
    solids: &SolidOptions,
) -> Result<SupportedMesh> {
    let raft_top = params.raft_thickness_mm;
    for point in &points {
        let (used, _) = params_for_point(params, point, palette)?;
        let needed = raft_top + used.foot_height_mm + used.tip_length_mm;
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
    let mut pillar_triangles = Vec::new();
    let mut shafts = Vec::with_capacity(points.len());
    for point in &points {
        let (used, preset_depth) = params_for_point(params, point, palette)?;
        let depth = preset_depth.or(solids.tip_penetration_mm).unwrap_or(0.0);
        check_penetration(depth)?;
        let pillar = pillar_solid(point, &used, raft_top, depth)?;
        pillar_triangles.extend_from_slice(pillar.triangles());
        shafts.push(Shaft { x: point.x, y: point.y, z_bottom: raft_top + used.foot_height_mm, z_top: point.contact_z - used.tip_length_mm });
    }
    let pillars = Mesh::new(pillar_triangles).map_err(|err| ResinError::invalid(format!("supports: {err}")))?;
    let (braces, brace_count, braces_skipped) = match &solids.bracing {
        Some(bracing) => {
            let (struts, skipped) = plan_braces(&shafts, &part, raft_top, bracing)?;
            let mut triangles = Vec::with_capacity(struts.len() * 32);
            for strut in &struts {
                triangles.extend(strut_solid(strut, bracing.diameter_mm)?);
            }
            if skipped > 0 {
                warnings.push(format!("{skipped} brace(s) left out because they would touch the part"));
            }
            (Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("braces: {err}")))?, struts.len(), skipped)
        }
        None => (Mesh::new(Vec::new()).map_err(|err| ResinError::invalid(format!("braces: {err}")))?, 0, 0),
    };
    let mesh = combine_meshes(&[&part, &raft, &pillars, &braces])?;
    Ok(SupportedMesh {
        mesh,
        part,
        raft,
        pillars,
        braces,
        points,
        outlines,
        features,
        optimize,
        warnings,
        palette: palette.to_vec(),
        palette_names: Vec::new(),
        zone_report: None,
        brace_count,
        braces_skipped,
    })
}

fn pillar_solid(point: &SupportPoint, params: &SupportParams, raft_top: f64, penetration_mm: f64) -> Result<Mesh> {
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
    if penetration_mm > 0.0 {
        // The tip continues straight into the part, at the tip diameter.
        push(&mut rings, point.contact_z + penetration_mm, tip_r)?;
    }
    profile_solid(point.x, point.y, &rings, RING_SIDES)
}

/// Shaded 3/4 view of the supported part, from a low side angle so the raft undercut is visible.
pub fn render_supported_png(result: &SupportedMesh, width: u32, height: u32) -> Result<Vec<u8>> {
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err(ResinError::invalid("bad preview size"));
    }
    let bounds = result.mesh.bounds().map_err(|err| ResinError::invalid(format!("mesh: {err}")))?;
    let (min, max) = (bounds.min(), bounds.max());
    let center = Vec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5);
    let span = (max.x - min.x).max(max.y - min.y).max(max.z - min.z).max(1.0);
    // Look from a low side angle, slightly upward, so the wider top of the raft reads as an undercut.
    let look = Vec3::new(-1.35, 1.7, 0.22);
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
    draw(&result.braces, [0.95, 0.60, 0.18], &mut depth, &mut rgba);
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
