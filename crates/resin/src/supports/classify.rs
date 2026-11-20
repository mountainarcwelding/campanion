//! Feature classifier used by the support optimizer.
//!
//! Regions are labelled as islands, lowest-band, area jumps, overhangs, cups/suction, or thin
//! features. Numeric thresholds that are not already on the mesh (thin width, area-jump ratio,
//! lowest-band height) come from the caller — typically a calibration profile — and are never
//! invented here.

use campanion_mesh::Mesh;

use crate::error::{ResinError, Result};
use crate::supports::islands::{AreaJump, Cup, DetectedFeatures, Island, thin_components};
use crate::supports::place::is_overhang;

/// Kind of demand region the optimizer reasons about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeatureClass {
    Island,
    LowestBand,
    AreaJump,
    Overhang,
    Cup,
    ThinFeature,
}

impl FeatureClass {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Island => "island",
            Self::LowestBand => "lowest_band",
            Self::AreaJump => "area_jump",
            Self::Overhang => "overhang",
            Self::Cup => "cup",
            Self::ThinFeature => "thin_feature",
        }
    }
}

/// Optional thresholds supplied by a calibration profile or by explicit flags.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClassifyThresholds {
    /// Height above the part's minimum Z treated as the lowest band, millimetres.
    pub lowest_band_height_mm: Option<f64>,
    /// Components whose smaller bounding-box side is below this width are thin.
    pub thin_width_mm: Option<f64>,
    /// Minimum layer-area ratio counted as an area jump.
    pub area_jump_ratio: Option<f64>,
}

/// A classified region with the geometric quantities the holding-force check can use.
#[derive(Clone, Debug, PartialEq)]
pub struct ClassifiedRegion {
    pub class: FeatureClass,
    pub z_mm: f64,
    pub area_mm2: f64,
    pub centroid: (f64, f64),
    pub min: (f64, f64),
    pub max: (f64, f64),
    /// Island id when this region came from island detection.
    pub island_id: Option<u32>,
}

/// Run the classifier on detected islands plus overhangs of `mesh`.
pub fn classify_features(
    mesh: &Mesh,
    features: &DetectedFeatures,
    overhang_angle_deg: f64,
    thresholds: ClassifyThresholds,
) -> Result<Vec<ClassifiedRegion>> {
    if !overhang_angle_deg.is_finite() || !(0.0..=90.0).contains(&overhang_angle_deg) {
        return Err(ResinError::invalid("overhang angle must be between 0 and 90 degrees"));
    }
    if let Some(height) = thresholds.lowest_band_height_mm
        && (!height.is_finite() || height <= 0.0)
    {
        return Err(ResinError::invalid("lowest-band height must be greater than 0 mm"));
    }
    let mut regions = Vec::new();
    for island in &features.islands {
        regions.push(region_from_island(FeatureClass::Island, island));
        if let Some(height) = thresholds.lowest_band_height_mm
            && island.z_mm <= features.z_min_mm + height
        {
            regions.push(region_from_island(FeatureClass::LowestBand, island));
        }
    }
    if let Some(ratio) = thresholds.area_jump_ratio {
        for jump in features.area_jumps(ratio)? {
            regions.push(region_from_jump(&jump));
        }
    }
    if let Some(width) = thresholds.thin_width_mm {
        for thin in thin_components(features, width)? {
            regions.push(region_from_island(FeatureClass::ThinFeature, &thin));
        }
    }
    for cup in &features.cups {
        regions.push(region_from_cup(cup));
    }
    regions.extend(overhang_regions(mesh, overhang_angle_deg)?);
    Ok(regions)
}

fn region_from_island(class: FeatureClass, island: &Island) -> ClassifiedRegion {
    ClassifiedRegion {
        class,
        z_mm: island.z_mm,
        area_mm2: island.area_mm2,
        centroid: island.centroid,
        min: island.min,
        max: island.max,
        island_id: Some(island.id),
    }
}

fn region_from_jump(jump: &AreaJump) -> ClassifiedRegion {
    ClassifiedRegion {
        class: FeatureClass::AreaJump,
        z_mm: jump.z_mm,
        area_mm2: jump.area_mm2,
        centroid: (0.0, 0.0),
        min: (0.0, 0.0),
        max: (0.0, 0.0),
        island_id: None,
    }
}

fn region_from_cup(cup: &Cup) -> ClassifiedRegion {
    ClassifiedRegion {
        class: FeatureClass::Cup,
        z_mm: cup.z_mm,
        area_mm2: cup.area_mm2,
        centroid: cup.centroid,
        min: cup.centroid,
        max: cup.centroid,
        island_id: None,
    }
}

fn overhang_regions(mesh: &Mesh, overhang_angle_deg: f64) -> Result<Vec<ClassifiedRegion>> {
    let mut regions = Vec::new();
    for tri in mesh.triangles() {
        if !is_overhang(tri, overhang_angle_deg) {
            continue;
        }
        let area = triangle_xy_area(tri);
        let cx = (tri.a.x + tri.b.x + tri.c.x) / 3.0;
        let cy = (tri.a.y + tri.b.y + tri.c.y) / 3.0;
        let cz = (tri.a.z + tri.b.z + tri.c.z) / 3.0;
        let min = (tri.a.x.min(tri.b.x).min(tri.c.x), tri.a.y.min(tri.b.y).min(tri.c.y));
        let max = (tri.a.x.max(tri.b.x).max(tri.c.x), tri.a.y.max(tri.b.y).max(tri.c.y));
        regions.push(ClassifiedRegion { class: FeatureClass::Overhang, z_mm: cz, area_mm2: area, centroid: (cx, cy), min, max, island_id: None });
    }
    Ok(regions)
}

fn triangle_xy_area(tri: &campanion_mesh::Triangle) -> f64 {
    let abx = tri.b.x - tri.a.x;
    let aby = tri.b.y - tri.a.y;
    let acx = tri.c.x - tri.a.x;
    let acy = tri.c.y - tri.a.y;
    0.5 * (abx * acy - aby * acx).abs()
}
