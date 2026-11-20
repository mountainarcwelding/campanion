//! Automated optimized supports: capacity check and placement within a user palette.
//!
//! The holding-force check is
//! `sum(tip capacity) >= safety_factor * (peel + suction + weight)`.
//! Every constant comes from a [`CalibrationProfile`]. The optimizer refuses to run without a
//! complete profile. Manual and uniform modes do not use this module and work without one.
//!
//! Placement starts from island and overhang seeds at the user's spacing (with optional denser
//! lowest-band spacing). If the check fails, spacing is tightened or a heavier palette class is
//! chosen so that the required tip count can be placed, preferring the result with the smallest
//! scar area. No numeric default is introduced: required counts come from demand / tip capacity,
//! and spacing from `sqrt(area / n)` capped by the user's spacing (never loosened).

use campanion_mesh::Mesh;

use crate::error::{ResinError, Result};
use crate::supports::calibration::STANDARD_GRAVITY_M_S2;
use crate::supports::calibration::{CalibratedTip, CalibrationProfile};
use crate::supports::classify::{ClassifiedRegion, ClassifyThresholds, classify_features};
use crate::supports::islands::{DetectedFeatures, IslandDetectParams, detect_islands, island_fill_points};
use crate::supports::params::SupportParams;
use crate::supports::place::{
    InputPoint, MAX_SUPPORT_POINTS, PointSource, SupportPoint, is_self_supported, lowest_hit_above, place_supports, thin_points_with,
};
use crate::supports::solid::signed_volume;

/// Inputs for one optimizer run.
pub struct OptimizeInput<'a> {
    pub mesh: &'a Mesh,
    pub params: &'a SupportParams,
    pub detect: IslandDetectParams,
    pub calibration: &'a CalibrationProfile,
    pub extra: &'a [InputPoint],
    /// Overrides the profile's lowest-band height when set. Pair with [`Self::lowest_band_spacing_mm`].
    pub lowest_band_height_mm: Option<f64>,
    pub lowest_band_spacing_mm: Option<f64>,
    /// Overrides the profile's island ignore area when set.
    pub island_ignore_area_mm2: Option<f64>,
}

/// Peel, suction, weight, and the resulting demand and capacity, all in newtons.
#[derive(Clone, Debug, PartialEq)]
pub struct ForceBreakdown {
    pub peel_n: f64,
    pub suction_n: f64,
    pub weight_n: f64,
    /// `safety_factor * (peel + suction + weight)`.
    pub demand_n: f64,
    pub capacity_n: f64,
    pub scar_area_mm2: f64,
    pub sufficient: bool,
    /// Palette class assigned to generated points.
    pub preset_name: String,
    pub tip_count: usize,
}

/// Result of an optimizer run, including warnings when capacity still falls short.
#[derive(Clone, Debug, PartialEq)]
pub struct OptimizeResult {
    pub points: Vec<SupportPoint>,
    pub features: DetectedFeatures,
    pub regions: Vec<ClassifiedRegion>,
    pub forces: ForceBreakdown,
    pub warnings: Vec<String>,
}

/// Run the optimizer. Refuses without a complete calibration profile.
pub fn optimize_supports(input: OptimizeInput<'_>) -> Result<OptimizeResult> {
    if let Err(err) = input.calibration.validate() {
        return Err(ResinError::invalid(format!("optimizer requires a complete calibration profile: {err}")));
    }
    let band = resolve_band(input.lowest_band_height_mm, input.lowest_band_spacing_mm, input.calibration)?;
    let ignore_area = match input.island_ignore_area_mm2.or(input.calibration.island_ignore_area_mm2) {
        Some(area) if !area.is_finite() || area < 0.0 => {
            return Err(ResinError::invalid("island ignore area must be 0 mm² or greater"));
        }
        other => other,
    };
    let features = detect_islands(input.mesh, &input.detect)?;
    let thresholds = ClassifyThresholds {
        lowest_band_height_mm: band.map(|(height, _)| height),
        thin_width_mm: input.calibration.thin_width_mm,
        area_jump_ratio: input.calibration.area_jump_ratio,
    };
    let regions = classify_features(input.mesh, &features, input.params.overhang_angle_deg, thresholds)?;
    let forces0 = demand_forces(input.mesh, &features, input.calibration)?;
    let palette = sort_palette(&input.calibration.tip_palette);
    if palette.is_empty() {
        return Err(ResinError::invalid("optimizer requires a complete calibration profile: tip_palette (needs at least one tip class)"));
    }

    let z_min = input.params.raft_thickness_mm;
    let seeds = seed_points(input.mesh, input.params, &features, input.extra, ignore_area, band, z_min)?;
    let mut best: Option<(OptimizeResult, f64)> = None;
    let mut last_insufficient: Option<OptimizeResult> = None;

    for (preset_index, tip) in palette {
        let n_needed = tips_needed(forces0.demand_n, tip.tip_capacity_n)?;
        let points = place_for_count(
            input.mesh,
            input.params,
            &features,
            input.extra,
            ignore_area,
            band,
            z_min,
            seeds.clone(),
            n_needed,
            preset_index as u16,
        )?;
        let capacity = points.len() as f64 * tip.tip_capacity_n;
        let scar = points.len() as f64 * tip.contact_area_mm2;
        let sufficient = capacity + 1e-12 >= forces0.demand_n;
        let mut warnings = Vec::new();
        if !sufficient {
            warnings.push(format!(
                "holding-force check is short: capacity {capacity:.6} N is below demand {demand:.6} N (safety factor {} × (peel {} + suction {} + weight {}) N) even with preset `{}` ({n_needed} tips requested, {} placed)",
                input.calibration.safety_factor,
                forces0.peel_n,
                forces0.suction_n,
                forces0.weight_n,
                tip.name,
                points.len(),
                demand = forces0.demand_n
            ));
        }
        let result = OptimizeResult {
            points,
            features: features.clone(),
            regions: regions.clone(),
            forces: ForceBreakdown {
                peel_n: forces0.peel_n,
                suction_n: forces0.suction_n,
                weight_n: forces0.weight_n,
                demand_n: forces0.demand_n,
                capacity_n: capacity,
                scar_area_mm2: scar,
                sufficient,
                preset_name: tip.name.clone(),
                tip_count: 0,
            },
            warnings,
        };
        let tip_count = result.points.len();
        let mut result = result;
        result.forces.tip_count = tip_count;
        if sufficient {
            match &best {
                Some((_, best_scar)) if scar >= *best_scar => {}
                _ => best = Some((result, scar)),
            }
        } else {
            last_insufficient = Some(result);
        }
    }

    if let Some((result, _)) = best {
        return Ok(result);
    }
    last_insufficient.ok_or_else(|| ResinError::invalid("optimizer produced no support points"))
}

fn demand_forces(mesh: &Mesh, features: &DetectedFeatures, calibration: &CalibrationProfile) -> Result<ForceBreakdown> {
    let max_area = features.layers.iter().map(|layer| layer.area_mm2).fold(0.0, f64::max);
    let peel_n = calibration.peel_coefficient_n_per_mm2 * max_area;
    let mut cup_by_layer: Vec<f64> = vec![0.0; features.layers.len().max(1)];
    for cup in &features.cups {
        if let Some(slot) = cup_by_layer.get_mut(cup.layer as usize) {
            *slot += cup.area_mm2;
        }
    }
    let max_cup_area = cup_by_layer.iter().copied().fold(0.0, f64::max);
    let suction_n = calibration.suction_coefficient_n_per_mm2 * max_cup_area;
    let volume_mm3 = signed_volume(mesh).abs();
    // density g/cm³ × (mm³ / 1000) = grams; grams / 1000 = kilograms; × gravity = newtons.
    let weight_n = calibration.resin_density_g_per_cm3 * volume_mm3 / 1_000_000.0 * STANDARD_GRAVITY_M_S2;
    if ![peel_n, suction_n, weight_n].iter().all(|v| v.is_finite()) {
        return Err(ResinError::invalid("holding-force terms are not finite"));
    }
    let demand_n = calibration.safety_factor * (peel_n + suction_n + weight_n);
    Ok(ForceBreakdown {
        peel_n,
        suction_n,
        weight_n,
        demand_n,
        capacity_n: 0.0,
        scar_area_mm2: 0.0,
        sufficient: false,
        preset_name: String::new(),
        tip_count: 0,
    })
}

fn tips_needed(demand_n: f64, tip_capacity_n: f64) -> Result<usize> {
    if !tip_capacity_n.is_finite() || tip_capacity_n <= 0.0 {
        return Err(ResinError::invalid("tip_capacity_n must be greater than 0 N"));
    }
    if !demand_n.is_finite() || demand_n < 0.0 {
        return Err(ResinError::invalid("holding-force demand is not finite"));
    }
    let n = (demand_n / tip_capacity_n).ceil();
    if !n.is_finite() || n > MAX_SUPPORT_POINTS as f64 {
        return Err(ResinError::invalid(format!(
            "holding-force check would require too many tips ({n}); increase tip capacity in the calibration profile or check the demand terms"
        )));
    }
    Ok((n as usize).max(1))
}

fn sort_palette(palette: &[CalibratedTip]) -> Vec<(usize, &CalibratedTip)> {
    let mut indexed: Vec<(usize, &CalibratedTip)> = palette.iter().enumerate().collect();
    indexed.sort_by(|left, right| {
        left.1
            .tip_capacity_n
            .partial_cmp(&right.1.tip_capacity_n)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(left.1.contact_area_mm2.partial_cmp(&right.1.contact_area_mm2).unwrap_or(std::cmp::Ordering::Equal))
            .then(left.0.cmp(&right.0))
    });
    indexed
}

fn resolve_band(flag_height: Option<f64>, flag_spacing: Option<f64>, calibration: &CalibrationProfile) -> Result<Option<(f64, f64)>> {
    let pair = match (flag_height, flag_spacing) {
        (None, None) => match (calibration.lowest_band_height_mm, calibration.lowest_band_spacing_mm) {
            (None, None) => None,
            (Some(height), Some(spacing)) => Some((height, spacing)),
            _ => {
                return Err(ResinError::invalid("calibration profile lowest_band_height_mm and lowest_band_spacing_mm must be supplied together"));
            }
        },
        (Some(height), Some(spacing)) => Some((height, spacing)),
        _ => return Err(ResinError::invalid("lowest-band height and lowest-band spacing must be supplied together")),
    };
    let Some((height, spacing)) = pair else {
        return Ok(None);
    };
    if !height.is_finite() || height <= 0.0 {
        return Err(ResinError::invalid("lowest-band height must be greater than 0 mm"));
    }
    if !spacing.is_finite() || spacing <= 0.0 {
        return Err(ResinError::invalid("lowest-band spacing must be greater than 0 mm"));
    }
    Ok(Some((height, spacing)))
}

fn seed_points(
    mesh: &Mesh,
    params: &SupportParams,
    features: &DetectedFeatures,
    extra: &[InputPoint],
    ignore_area: Option<f64>,
    band: Option<(f64, f64)>,
    z_min: f64,
) -> Result<Vec<SupportPoint>> {
    place_with_islands(mesh, params, features, extra, true, ignore_area, band, z_min)
}

fn place_for_count(
    mesh: &Mesh,
    params: &SupportParams,
    features: &DetectedFeatures,
    extra: &[InputPoint],
    ignore_area: Option<f64>,
    band: Option<(f64, f64)>,
    z_min: f64,
    seeds: Vec<SupportPoint>,
    n_needed: usize,
    preset_index: u16,
) -> Result<Vec<SupportPoint>> {
    if seeds.len() >= n_needed {
        let mut points = seeds;
        assign_preset(&mut points, preset_index);
        return Ok(points);
    }
    let covered_area: f64 =
        features.islands.iter().map(|island| if ignore_area.is_some_and(|limit| island.area_mm2 <= limit) { 0.0 } else { island.area_mm2 }).sum();
    let area = if covered_area > 0.0 { covered_area } else { params.support_spacing_mm * params.support_spacing_mm * n_needed as f64 };
    let computed = (area / n_needed as f64).sqrt();
    if !computed.is_finite() || computed <= 0.0 {
        return Err(ResinError::invalid("computed support spacing is not finite"));
    }
    let tight = computed.min(params.support_spacing_mm);
    let tight_band = band.map(|(height, spacing)| (height, spacing.min(tight)));
    let mut params_tight = params.clone();
    params_tight.support_spacing_mm = tight;
    let mut points = place_with_islands(mesh, &params_tight, features, extra, true, ignore_area, tight_band, z_min)?;
    if points.len() < n_needed {
        points = add_fill_points(mesh, features, ignore_area, points, n_needed, z_min)?;
    }
    if points.len() > MAX_SUPPORT_POINTS {
        return Err(ResinError::invalid(format!("too many supports: {} > {MAX_SUPPORT_POINTS}", points.len())));
    }
    assign_preset(&mut points, preset_index);
    Ok(points)
}

fn assign_preset(points: &mut [SupportPoint], preset_index: u16) {
    for point in points {
        if point.preset_index.is_none() {
            point.preset_index = Some(preset_index);
        }
        if point.source == PointSource::Overhang || point.source == PointSource::Island {
            // leave source; optimizer may have added extras as Optimizer
        }
    }
}

fn add_fill_points(
    mesh: &Mesh,
    features: &DetectedFeatures,
    ignore_area: Option<f64>,
    mut points: Vec<SupportPoint>,
    n_needed: usize,
    z_min: f64,
) -> Result<Vec<SupportPoint>> {
    for island in &features.islands {
        if points.len() >= n_needed {
            break;
        }
        if ignore_area.is_some_and(|limit| island.area_mm2 <= limit) {
            continue;
        }
        let hit = lowest_hit_above(mesh, island.centroid.0, island.centroid.1, z_min);
        if let Some((pos, _)) = hit {
            let candidate = SupportPoint { x: pos.x, y: pos.y, contact_z: pos.z, source: PointSource::Optimizer, preset_index: None };
            let close = points.iter().any(|other| {
                let dx = candidate.x - other.x;
                let dy = candidate.y - other.y;
                dx * dx + dy * dy <= 1e-12
            });
            if !close {
                points.push(candidate);
            }
        }
    }
    Ok(points)
}

/// Island-aware placement used by uniform auto (no calibration) and as the optimizer seed.
pub fn place_with_islands(
    mesh: &Mesh,
    params: &SupportParams,
    features: &DetectedFeatures,
    extra: &[InputPoint],
    auto: bool,
    ignore_area: Option<f64>,
    band: Option<(f64, f64)>,
    z_min: f64,
) -> Result<Vec<SupportPoint>> {
    let mut extra_all = extra.to_vec();
    if auto {
        for island in &features.islands {
            if ignore_area.is_some_and(|limit| island.area_mm2 <= limit) {
                continue;
            }
            let spacing = spacing_for_z(island.z_mm, features.z_min_mm, params.support_spacing_mm, band);
            for (x, y) in island_fill_points(island, spacing)? {
                extra_all.push(InputPoint { x, y, z: Some(island.z_mm), preset: None, source: PointSource::Island });
            }
        }
    }
    let mut points = place_supports_typed(mesh, params, &extra_all, auto, z_min)?;
    if let Some((height, band_spacing)) = band {
        let z_cut = features.z_min_mm + height;
        points = thin_points_with(&points, params.support_spacing_mm, Some((z_cut, band_spacing)))?;
    }
    Ok(points)
}

fn spacing_for_z(z: f64, z_min: f64, spacing: f64, band: Option<(f64, f64)>) -> f64 {
    match band {
        Some((height, band_spacing)) if z <= z_min + height => band_spacing,
        _ => spacing,
    }
}

fn place_supports_typed(mesh: &Mesh, params: &SupportParams, extra: &[InputPoint], auto: bool, z_min: f64) -> Result<Vec<SupportPoint>> {
    // Reuse place_supports then restore sources from InputPoint by matching XY.
    let placed = place_supports(mesh, params, extra, auto, z_min)?;
    let mut out = placed;
    for point in &mut out {
        if let Some(source) =
            extra.iter().find(|input| (input.x - point.x).abs() < 1e-9 && (input.y - point.y).abs() < 1e-9).map(|input| input.source)
        {
            if source != PointSource::Overhang {
                point.source = source;
            }
            if let Some(name) = extra
                .iter()
                .find(|input| (input.x - point.x).abs() < 1e-9 && (input.y - point.y).abs() < 1e-9)
                .and_then(|input| input.preset.as_ref())
            {
                let _ = name; // palette index is assigned after placement
            }
        }
    }
    let _ = is_self_supported;
    Ok(out)
}
