//! Versioned resin support calibration profile.
//!
//! Every constant the optimizer uses comes from a profile the user supplies. This module does
//! not ship numeric defaults. A missing field is reported by name. Fitting measurements to
//! constants is a stub: import of a complete profile works, and `fit_from_measurements`
//! returns a clear error until a later release implements the fit.
//!
//! The geometry of a calibration test print is intentionally not defined here.

use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::error::{ResinError, Result};

/// Schema identifier for this profile version.
pub const CALIBRATION_SCHEMA: &str = "campanion-resin-calibration/1";

/// Required fields of [`CalibrationProfile`] that the optimizer needs.
pub const REQUIRED_CONSTANTS: &[&str] = &[
    "schema",
    "name",
    "tip_palette",
    "peel_coefficient_n_per_mm2",
    "suction_coefficient_n_per_mm2",
    "safety_factor",
    "resin_density_g_per_cm3",
    "gravity_m_s2",
];

/// Required fields of each [`CalibratedTip`] entry.
pub const REQUIRED_TIP_FIELDS: &[&str] =
    &["name", "tip_diameter_mm", "tip_length_mm", "pillar_diameter_mm", "foot_diameter_mm", "foot_height_mm", "tip_capacity_n", "contact_area_mm2"];

/// A user-calibrated tip class (for example Light, Medium, or Heavy).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibratedTip {
    pub name: String,
    pub tip_diameter_mm: f64,
    pub tip_length_mm: f64,
    pub pillar_diameter_mm: f64,
    pub foot_diameter_mm: f64,
    pub foot_height_mm: f64,
    /// Holding force this tip can take at the contact, newtons, from the user's calibration.
    pub tip_capacity_n: f64,
    /// Contact area used when minimizing scar, millimetres squared, from the user's calibration.
    pub contact_area_mm2: f64,
}

/// One recorded measurement. Fitting these to constants is not implemented yet.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationMeasurement {
    /// Kind of observation, for example `tip_pull`, `peel`, `suction`, or `density`.
    pub kind: String,
    /// Tip palette name, when the measurement applies to one class.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preset: Option<String>,
    /// Outcome word supplied by the user, for example `held` or `failed`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub outcome: Option<String>,
    /// Measured force, newtons, when recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub force_n: Option<f64>,
    /// Measured area, millimetres squared, when recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub area_mm2: Option<f64>,
    /// Measured mass, grams, when recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mass_g: Option<f64>,
    /// Measured volume, millimetres cubed, when recorded.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume_mm3: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Optional raw document used to list missing fields before validation.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
struct CalibrationProfileRaw {
    schema: Option<String>,
    name: Option<String>,
    tip_palette: Option<Vec<CalibratedTipRaw>>,
    peel_coefficient_n_per_mm2: Option<f64>,
    suction_coefficient_n_per_mm2: Option<f64>,
    safety_factor: Option<f64>,
    resin_density_g_per_cm3: Option<f64>,
    gravity_m_s2: Option<f64>,
    #[serde(default)]
    island_ignore_area_mm2: Option<f64>,
    #[serde(default)]
    thin_width_mm: Option<f64>,
    #[serde(default)]
    area_jump_ratio: Option<f64>,
    #[serde(default)]
    lowest_band_height_mm: Option<f64>,
    #[serde(default)]
    lowest_band_spacing_mm: Option<f64>,
    #[serde(default)]
    measurements: Vec<CalibrationMeasurement>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
struct CalibratedTipRaw {
    name: Option<String>,
    tip_diameter_mm: Option<f64>,
    tip_length_mm: Option<f64>,
    pillar_diameter_mm: Option<f64>,
    foot_diameter_mm: Option<f64>,
    foot_height_mm: Option<f64>,
    tip_capacity_n: Option<f64>,
    contact_area_mm2: Option<f64>,
}

/// User calibration results: measurements plus the fitted constants the optimizer reads.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationProfile {
    pub schema: String,
    pub name: String,
    pub tip_palette: Vec<CalibratedTip>,
    /// Peel force per unit layer area, newtons per millimetre squared.
    pub peel_coefficient_n_per_mm2: f64,
    /// Suction force per unit cup area, newtons per millimetre squared.
    pub suction_coefficient_n_per_mm2: f64,
    /// Multiplier applied to (peel + suction + weight) in the holding-force check.
    pub safety_factor: f64,
    pub resin_density_g_per_cm3: f64,
    pub gravity_m_s2: f64,
    /// Islands at or below this area, millimetres squared, are ignored when present.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub island_ignore_area_mm2: Option<f64>,
    /// Components whose smaller bounding-box side is below this width are classified as thin.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thin_width_mm: Option<f64>,
    /// Minimum layer-area ratio counted as an area jump.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub area_jump_ratio: Option<f64>,
    /// Height of the lowest-band region above the part's minimum Z, millimetres, when used.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lowest_band_height_mm: Option<f64>,
    /// Support spacing inside the lowest band, millimetres, when used.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lowest_band_spacing_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub measurements: Vec<CalibrationMeasurement>,
}

/// A required constant that is missing or invalid, with a short reason.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MissingConstant {
    pub name: String,
    pub reason: String,
}

impl MissingConstant {
    fn new(name: impl Into<String>, reason: impl Into<String>) -> Self {
        Self { name: name.into(), reason: reason.into() }
    }
}

/// Parse a calibration profile from JSON text and report every missing or invalid required field.
pub fn parse_calibration(text: &str) -> Result<CalibrationProfile> {
    let raw: CalibrationProfileRaw = serde_json::from_str(text).map_err(|err| ResinError::invalid(format!("calibration profile: {err}")))?;
    let missing = missing_from_raw(&raw);
    if !missing.is_empty() {
        return Err(incomplete_error(&missing));
    }
    let profile = profile_from_raw(raw)?;
    profile.validate()?;
    Ok(profile)
}

/// Load a calibration profile from a JSON file.
pub fn load_calibration(path: &Path) -> Result<CalibrationProfile> {
    let text = std::fs::read_to_string(path).map_err(|err| ResinError::invalid(format!("calibration profile {}: {err}", path.display())))?;
    parse_calibration(&text)
}

/// List required constants that are missing or invalid in `text` without failing the parse of optional fields.
pub fn missing_constants_in(text: &str) -> Result<Vec<MissingConstant>> {
    let raw: CalibrationProfileRaw = serde_json::from_str(text).map_err(|err| ResinError::invalid(format!("calibration profile: {err}")))?;
    Ok(missing_from_raw(&raw))
}

impl CalibrationProfile {
    /// Required constants that are missing or fail validation, in a stable order.
    pub fn missing_constants(&self) -> Vec<MissingConstant> {
        let mut missing = Vec::new();
        push_schema(&self.schema, &mut missing);
        push_name(&self.name, &mut missing);
        if self.tip_palette.is_empty() {
            missing.push(MissingConstant::new("tip_palette", "needs at least one tip class with geometry and tip_capacity_n"));
        }
        for (index, tip) in self.tip_palette.iter().enumerate() {
            missing.extend(tip_missing(index, tip));
        }
        push_positive("peel_coefficient_n_per_mm2", self.peel_coefficient_n_per_mm2, &mut missing);
        push_non_negative("suction_coefficient_n_per_mm2", self.suction_coefficient_n_per_mm2, &mut missing);
        push_positive("safety_factor", self.safety_factor, &mut missing);
        push_positive("resin_density_g_per_cm3", self.resin_density_g_per_cm3, &mut missing);
        push_positive("gravity_m_s2", self.gravity_m_s2, &mut missing);
        push_optional_positive("island_ignore_area_mm2", self.island_ignore_area_mm2, &mut missing);
        push_optional_positive("thin_width_mm", self.thin_width_mm, &mut missing);
        push_optional_positive("area_jump_ratio", self.area_jump_ratio, &mut missing);
        match (self.lowest_band_height_mm, self.lowest_band_spacing_mm) {
            (None, None) => {}
            (Some(height), Some(spacing)) => {
                push_positive("lowest_band_height_mm", height, &mut missing);
                push_positive("lowest_band_spacing_mm", spacing, &mut missing);
            }
            (Some(_), None) => missing.push(MissingConstant::new("lowest_band_spacing_mm", "required when lowest_band_height_mm is set")),
            (None, Some(_)) => missing.push(MissingConstant::new("lowest_band_height_mm", "required when lowest_band_spacing_mm is set")),
        }
        missing
    }

    /// `Ok` when every required constant is present and valid.
    pub fn validate(&self) -> Result<()> {
        let missing = self.missing_constants();
        if missing.is_empty() { Ok(()) } else { Err(incomplete_error(&missing)) }
    }

    /// Whether the optimizer may run against this profile.
    pub fn is_complete(&self) -> bool {
        self.missing_constants().is_empty()
    }

    /// Serialize the profile as pretty JSON.
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(|err| ResinError::invalid(format!("calibration profile: {err}")))
    }

    /// Stub: fitting measurements to constants is not implemented.
    ///
    /// A later release will take [`Self::measurements`] and write fitted constants. Until then,
    /// enter the fitted constants directly in the profile. This function never invents values.
    pub fn fit_from_measurements(&self) -> Result<CalibrationProfile> {
        Err(ResinError::invalid("fitting measurements to calibration constants is not implemented; enter fitted constants directly in the profile"))
    }
}

/// Stub import path: read a profile, and if the caller asked to fit measurements, refuse clearly.
pub fn import_calibration(path: &Path, fit_measurements: bool) -> Result<CalibrationProfile> {
    let profile = load_calibration(path)?;
    if fit_measurements {
        return profile.fit_from_measurements();
    }
    Ok(profile)
}

fn incomplete_error(missing: &[MissingConstant]) -> ResinError {
    let list = missing.iter().map(|item| format!("{} ({})", item.name, item.reason)).collect::<Vec<_>>().join("; ");
    ResinError::invalid(format!("calibration profile is incomplete: {list}"))
}

fn missing_from_raw(raw: &CalibrationProfileRaw) -> Vec<MissingConstant> {
    let mut missing = Vec::new();
    match raw.schema.as_deref() {
        None => missing.push(MissingConstant::new("schema", "missing; expected campanion-resin-calibration/1")),
        Some(schema) => push_schema(schema, &mut missing),
    }
    match raw.name.as_deref() {
        None => missing.push(MissingConstant::new("name", "missing")),
        Some(name) => push_name(name, &mut missing),
    }
    match &raw.tip_palette {
        None => missing.push(MissingConstant::new("tip_palette", "missing; needs at least one tip class with geometry and tip_capacity_n")),
        Some(tips) if tips.is_empty() => {
            missing.push(MissingConstant::new("tip_palette", "needs at least one tip class with geometry and tip_capacity_n"));
        }
        Some(tips) => {
            for (index, tip) in tips.iter().enumerate() {
                missing.extend(tip_raw_missing(index, tip));
            }
        }
    }
    push_required_positive("peel_coefficient_n_per_mm2", raw.peel_coefficient_n_per_mm2, &mut missing);
    push_required_non_negative("suction_coefficient_n_per_mm2", raw.suction_coefficient_n_per_mm2, &mut missing);
    push_required_positive("safety_factor", raw.safety_factor, &mut missing);
    push_required_positive("resin_density_g_per_cm3", raw.resin_density_g_per_cm3, &mut missing);
    push_required_positive("gravity_m_s2", raw.gravity_m_s2, &mut missing);
    push_optional_positive("island_ignore_area_mm2", raw.island_ignore_area_mm2, &mut missing);
    push_optional_positive("thin_width_mm", raw.thin_width_mm, &mut missing);
    push_optional_positive("area_jump_ratio", raw.area_jump_ratio, &mut missing);
    match (raw.lowest_band_height_mm, raw.lowest_band_spacing_mm) {
        (None, None) => {}
        (Some(height), Some(spacing)) => {
            push_positive("lowest_band_height_mm", height, &mut missing);
            push_positive("lowest_band_spacing_mm", spacing, &mut missing);
        }
        (Some(_), None) => missing.push(MissingConstant::new("lowest_band_spacing_mm", "required when lowest_band_height_mm is set")),
        (None, Some(_)) => missing.push(MissingConstant::new("lowest_band_height_mm", "required when lowest_band_spacing_mm is set")),
    }
    missing
}

fn profile_from_raw(raw: CalibrationProfileRaw) -> Result<CalibrationProfile> {
    let tips_raw = raw.tip_palette.ok_or_else(|| ResinError::invalid("calibration profile is incomplete: tip_palette (missing)"))?;
    let mut tip_palette = Vec::new();
    for tip in tips_raw {
        tip_palette.push(CalibratedTip {
            name: tip.name.unwrap_or_default(),
            tip_diameter_mm: tip.tip_diameter_mm.unwrap_or(f64::NAN),
            tip_length_mm: tip.tip_length_mm.unwrap_or(f64::NAN),
            pillar_diameter_mm: tip.pillar_diameter_mm.unwrap_or(f64::NAN),
            foot_diameter_mm: tip.foot_diameter_mm.unwrap_or(f64::NAN),
            foot_height_mm: tip.foot_height_mm.unwrap_or(f64::NAN),
            tip_capacity_n: tip.tip_capacity_n.unwrap_or(f64::NAN),
            contact_area_mm2: tip.contact_area_mm2.unwrap_or(f64::NAN),
        });
    }
    Ok(CalibrationProfile {
        schema: raw.schema.unwrap_or_default(),
        name: raw.name.unwrap_or_default(),
        tip_palette,
        peel_coefficient_n_per_mm2: raw.peel_coefficient_n_per_mm2.unwrap_or(f64::NAN),
        suction_coefficient_n_per_mm2: raw.suction_coefficient_n_per_mm2.unwrap_or(f64::NAN),
        safety_factor: raw.safety_factor.unwrap_or(f64::NAN),
        resin_density_g_per_cm3: raw.resin_density_g_per_cm3.unwrap_or(f64::NAN),
        gravity_m_s2: raw.gravity_m_s2.unwrap_or(f64::NAN),
        island_ignore_area_mm2: raw.island_ignore_area_mm2,
        thin_width_mm: raw.thin_width_mm,
        area_jump_ratio: raw.area_jump_ratio,
        lowest_band_height_mm: raw.lowest_band_height_mm,
        lowest_band_spacing_mm: raw.lowest_band_spacing_mm,
        measurements: raw.measurements,
    })
}

fn tip_raw_missing(index: usize, tip: &CalibratedTipRaw) -> Vec<MissingConstant> {
    let prefix = format!("tip_palette[{index}]");
    let mut missing = Vec::new();
    match tip.name.as_deref() {
        None | Some("") => missing.push(MissingConstant::new(format!("{prefix}.name"), "missing")),
        Some(_) => {}
    }
    push_required_positive(&format!("{prefix}.tip_diameter_mm"), tip.tip_diameter_mm, &mut missing);
    push_required_positive(&format!("{prefix}.tip_length_mm"), tip.tip_length_mm, &mut missing);
    push_required_positive(&format!("{prefix}.pillar_diameter_mm"), tip.pillar_diameter_mm, &mut missing);
    push_required_positive(&format!("{prefix}.foot_diameter_mm"), tip.foot_diameter_mm, &mut missing);
    push_required_positive(&format!("{prefix}.foot_height_mm"), tip.foot_height_mm, &mut missing);
    push_required_positive(&format!("{prefix}.tip_capacity_n"), tip.tip_capacity_n, &mut missing);
    push_required_positive(&format!("{prefix}.contact_area_mm2"), tip.contact_area_mm2, &mut missing);
    missing
}

fn tip_missing(index: usize, tip: &CalibratedTip) -> Vec<MissingConstant> {
    let prefix = format!("tip_palette[{index}]");
    let mut missing = Vec::new();
    if tip.name.trim().is_empty() {
        missing.push(MissingConstant::new(format!("{prefix}.name"), "missing"));
    }
    push_positive(&format!("{prefix}.tip_diameter_mm"), tip.tip_diameter_mm, &mut missing);
    push_positive(&format!("{prefix}.tip_length_mm"), tip.tip_length_mm, &mut missing);
    push_positive(&format!("{prefix}.pillar_diameter_mm"), tip.pillar_diameter_mm, &mut missing);
    push_positive(&format!("{prefix}.foot_diameter_mm"), tip.foot_diameter_mm, &mut missing);
    push_positive(&format!("{prefix}.foot_height_mm"), tip.foot_height_mm, &mut missing);
    push_positive(&format!("{prefix}.tip_capacity_n"), tip.tip_capacity_n, &mut missing);
    push_positive(&format!("{prefix}.contact_area_mm2"), tip.contact_area_mm2, &mut missing);
    missing
}

fn push_schema(schema: &str, missing: &mut Vec<MissingConstant>) {
    if schema != CALIBRATION_SCHEMA {
        missing.push(MissingConstant::new("schema", format!("expected {CALIBRATION_SCHEMA}, found {schema}")));
    }
}

fn push_name(name: &str, missing: &mut Vec<MissingConstant>) {
    if name.trim().is_empty() {
        missing.push(MissingConstant::new("name", "missing"));
    }
}

fn push_required_positive(name: &str, value: Option<f64>, missing: &mut Vec<MissingConstant>) {
    match value {
        None => missing.push(MissingConstant::new(name, "missing")),
        Some(value) => push_positive(name, value, missing),
    }
}

fn push_required_non_negative(name: &str, value: Option<f64>, missing: &mut Vec<MissingConstant>) {
    match value {
        None => missing.push(MissingConstant::new(name, "missing")),
        Some(value) => push_non_negative(name, value, missing),
    }
}

fn push_positive(name: &str, value: f64, missing: &mut Vec<MissingConstant>) {
    if !value.is_finite() || value <= 0.0 {
        missing.push(MissingConstant::new(name, "must be a finite number greater than 0"));
    }
}

fn push_non_negative(name: &str, value: f64, missing: &mut Vec<MissingConstant>) {
    if !value.is_finite() || value < 0.0 {
        missing.push(MissingConstant::new(name, "must be a finite number of 0 or greater"));
    }
}

fn push_optional_positive(name: &str, value: Option<f64>, missing: &mut Vec<MissingConstant>) {
    if let Some(value) = value {
        push_positive(name, value, missing);
    }
}
