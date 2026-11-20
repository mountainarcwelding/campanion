//! User-saved tip presets and the mix that assigns them to support roles.
//!
//! Nothing here ships a value. A preset is geometry the user typed and saved. A mix names which
//! saved preset each kind of support uses; a role left out of the mix uses the base Tip and pillar
//! values. Holding-force capacities are not part of a preset; they live in a calibration profile.

use serde::{Deserialize, Serialize};

use crate::error::{ResinError, Result};
use crate::supports::calibration::CalibratedTip;
use crate::supports::params::SupportParams;
use crate::supports::place::{InputPoint, PointSource, SupportPoint};

/// Schema tag for a saved preset library.
pub const PRESETS_SCHEMA: &str = "campanion-resin-presets/1";
/// Most presets accepted in one library.
pub const MAX_PRESETS: usize = 64;

/// Tip geometry used to build one support. Built from a preset, a calibrated tip, or the base parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct TipGeometry {
    pub tip_diameter_mm: f64,
    pub tip_length_mm: f64,
    pub pillar_diameter_mm: f64,
    pub foot_diameter_mm: f64,
    pub foot_height_mm: f64,
    /// Depth the tip continues into the part past the contact, millimetres. `None` uses the run's setting.
    pub tip_penetration_mm: Option<f64>,
}

impl From<&CalibratedTip> for TipGeometry {
    fn from(tip: &CalibratedTip) -> Self {
        Self {
            tip_diameter_mm: tip.tip_diameter_mm,
            tip_length_mm: tip.tip_length_mm,
            pillar_diameter_mm: tip.pillar_diameter_mm,
            foot_diameter_mm: tip.foot_diameter_mm,
            foot_height_mm: tip.foot_height_mm,
            tip_penetration_mm: None,
        }
    }
}

impl From<&TipPreset> for TipGeometry {
    fn from(preset: &TipPreset) -> Self {
        Self {
            tip_diameter_mm: preset.tip_diameter_mm,
            tip_length_mm: preset.tip_length_mm,
            pillar_diameter_mm: preset.pillar_diameter_mm,
            foot_diameter_mm: preset.foot_diameter_mm,
            foot_height_mm: preset.foot_height_mm,
            tip_penetration_mm: preset.tip_penetration_mm,
        }
    }
}

/// One saved preset. Every length is millimetres and comes from the user.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TipPreset {
    pub name: String,
    pub tip_diameter_mm: f64,
    pub tip_length_mm: f64,
    pub pillar_diameter_mm: f64,
    pub foot_diameter_mm: f64,
    pub foot_height_mm: f64,
    /// Optional per-preset tip penetration. When absent, the run's tip penetration applies.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_penetration_mm: Option<f64>,
}

impl TipPreset {
    pub fn validate(&self) -> Result<()> {
        let name = self.name.trim();
        if name.is_empty() {
            return Err(ResinError::invalid("a tip preset needs a name"));
        }
        if name.chars().any(char::is_whitespace) || name.contains(',') || name.contains('=') {
            return Err(ResinError::invalid(format!("tip preset name `{name}` must not contain spaces, commas or `=`")));
        }
        let label = |what: &str| format!("tip preset `{name}`: {what}");
        for (value, what) in [
            (self.tip_diameter_mm, "tip diameter"),
            (self.tip_length_mm, "tip length"),
            (self.pillar_diameter_mm, "pillar diameter"),
            (self.foot_diameter_mm, "foot diameter"),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(ResinError::invalid(label(&format!("{what} must be greater than 0 mm"))));
            }
        }
        if !self.foot_height_mm.is_finite() || self.foot_height_mm < 0.0 {
            return Err(ResinError::invalid(label("foot height must be 0 mm or greater")));
        }
        if self.foot_diameter_mm < self.pillar_diameter_mm {
            return Err(ResinError::invalid(label("foot diameter must be at least the pillar diameter")));
        }
        if let Some(depth) = self.tip_penetration_mm {
            check_penetration(depth)?;
        }
        Ok(())
    }
}

/// Tip penetration must be a finite depth of 0 mm or more.
pub fn check_penetration(depth: f64) -> Result<()> {
    if depth.is_finite() && depth >= 0.0 { Ok(()) } else { Err(ResinError::invalid("tip penetration must be 0 mm or greater")) }
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct PresetFile {
    schema: String,
    presets: Vec<TipPreset>,
}

/// Read a preset library. Names must be unique; every preset is validated.
pub fn parse_presets(text: &str) -> Result<Vec<TipPreset>> {
    let file: PresetFile = serde_json::from_str(text).map_err(|err| ResinError::invalid(format!("tip presets: {err}")))?;
    if file.schema != PRESETS_SCHEMA {
        return Err(ResinError::invalid(format!("tip presets: schema must be `{PRESETS_SCHEMA}`, not `{}`", file.schema)));
    }
    check_library(&file.presets)?;
    Ok(file.presets)
}

/// Write a preset library that [`parse_presets`] reads back.
pub fn write_presets(presets: &[TipPreset]) -> Result<String> {
    check_library(presets)?;
    let file = PresetFile { schema: PRESETS_SCHEMA.to_string(), presets: presets.to_vec() };
    serde_json::to_string_pretty(&file).map_err(|err| ResinError::invalid(format!("tip presets: {err}")))
}

fn check_library(presets: &[TipPreset]) -> Result<()> {
    if presets.len() > MAX_PRESETS {
        return Err(ResinError::invalid(format!("too many tip presets: {} > {MAX_PRESETS}", presets.len())));
    }
    for (index, preset) in presets.iter().enumerate() {
        preset.validate()?;
        if presets.iter().take(index).any(|other| other.name.trim() == preset.name.trim()) {
            return Err(ResinError::invalid(format!("tip preset name `{}` is used twice", preset.name.trim())));
        }
    }
    Ok(())
}

/// Index of the preset called `name`.
pub fn preset_index(presets: &[TipPreset], name: &str) -> Option<u16> {
    presets.iter().position(|preset| preset.name.trim() == name.trim()).and_then(|index| u16::try_from(index).ok())
}

/// Which kind of support a mix entry applies to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SupportRole {
    /// Auto-placed on an overhang above the lowest band.
    Overhang,
    /// Auto-placed on a first-appearance island.
    Island,
    /// Any auto-placed support inside the lowest band.
    LowestBand,
    /// Placed on a painted support zone.
    Painted,
    /// Added by hand.
    Manual,
}

impl SupportRole {
    pub const ALL: [SupportRole; 5] = [Self::LowestBand, Self::Island, Self::Overhang, Self::Painted, Self::Manual];

    pub fn key(self) -> &'static str {
        match self {
            Self::Overhang => "overhang",
            Self::Island => "island",
            Self::LowestBand => "lowest-band",
            Self::Painted => "painted",
            Self::Manual => "manual",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Overhang => "Overhangs",
            Self::Island => "Islands",
            Self::LowestBand => "Lowest band",
            Self::Painted => "Painted zones",
            Self::Manual => "Added by hand",
        }
    }

    fn from_key(key: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|role| role.key() == key)
    }
}

/// Role → preset name. A role with no entry keeps the base Tip and pillar values.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PresetMix {
    pub entries: Vec<(SupportRole, String)>,
}

impl PresetMix {
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn get(&self, role: SupportRole) -> Option<&str> {
        self.entries.iter().find(|(r, _)| *r == role).map(|(_, name)| name.as_str())
    }

    pub fn set(&mut self, role: SupportRole, name: Option<String>) {
        self.entries.retain(|(r, _)| *r != role);
        if let Some(name) = name.filter(|n| !n.trim().is_empty()) {
            self.entries.push((role, name.trim().to_string()));
        }
    }

    /// Parse `role=preset,role=preset`. Roles: overhang, island, lowest-band, painted, manual.
    pub fn parse(text: &str) -> Result<Self> {
        let mut mix = Self::default();
        for item in text.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let Some((role, name)) = item.split_once('=') else {
                return Err(ResinError::invalid(format!("preset mix entry `{item}` must be role=preset")));
            };
            let Some(role) = SupportRole::from_key(role.trim()) else {
                return Err(ResinError::invalid(format!(
                    "preset mix role `{}` is not one of overhang, island, lowest-band, painted, manual",
                    role.trim()
                )));
            };
            if mix.get(role).is_some() {
                return Err(ResinError::invalid(format!("preset mix names `{}` twice", role.key())));
            }
            let name = name.trim();
            if name.is_empty() {
                return Err(ResinError::invalid(format!("preset mix entry `{item}` has no preset name")));
            }
            mix.entries.push((role, name.to_string()));
        }
        Ok(mix)
    }

    /// Every named preset exists in `presets`.
    pub fn check(&self, presets: &[TipPreset]) -> Result<()> {
        for (role, name) in &self.entries {
            if preset_index(presets, name).is_none() {
                return Err(ResinError::invalid(format!("preset mix: `{}` uses preset `{name}`, which is not saved", role.key())));
            }
        }
        Ok(())
    }
}

/// The role a placed support plays. `band_z` is the lowest-band cut height in the lifted frame.
pub fn role_of(point: &SupportPoint, band_z: Option<f64>) -> SupportRole {
    match point.source {
        PointSource::Manual => SupportRole::Manual,
        PointSource::Painted => SupportRole::Painted,
        _ if band_z.is_some_and(|z| point.contact_z <= z) => SupportRole::LowestBand,
        PointSource::Island | PointSource::Optimizer => SupportRole::Island,
        PointSource::Overhang => SupportRole::Overhang,
    }
}

/// Assign presets from `mix` to points that have none yet. Returns how many points changed.
pub fn apply_mix(points: &mut [SupportPoint], mix: &PresetMix, presets: &[TipPreset], band_z: Option<f64>) -> Result<usize> {
    mix.check(presets)?;
    let mut changed = 0usize;
    for point in points.iter_mut() {
        if point.preset_index.is_some() {
            continue;
        }
        let role = role_of(point, band_z);
        if let Some(name) = mix.get(role)
            && let Some(index) = preset_index(presets, name)
        {
            point.preset_index = Some(index);
            changed += 1;
        }
    }
    Ok(changed)
}

/// Give hand-placed points the preset their input line named. Unknown names are an error.
pub fn resolve_named_points(points: &mut [SupportPoint], extra: &[InputPoint], presets: &[TipPreset]) -> Result<()> {
    for input in extra {
        let Some(name) = input.preset.as_deref() else {
            continue;
        };
        let Some(index) = preset_index(presets, name) else {
            return Err(ResinError::invalid(format!(
                "support point ({}, {}) names preset `{name}`, which is not in the tip presets",
                input.x, input.y
            )));
        };
        for point in points.iter_mut() {
            if point.source == PointSource::Manual && point.x == input.x && point.y == input.y && point.preset_index.is_none() {
                point.preset_index = Some(index);
            }
        }
    }
    Ok(())
}

/// After removing preset `removed`, shift higher indices down and clear points that used it.
pub fn forget_preset(points: &mut [SupportPoint], removed: u16) {
    for point in points.iter_mut() {
        match point.preset_index {
            Some(index) if index == removed => point.preset_index = None,
            Some(index) if index > removed => point.preset_index = Some(index - 1),
            _ => {}
        }
    }
}

/// Geometry for one point: the base parameters overlaid with its preset, validated together.
pub fn params_for_point(base: &SupportParams, point: &SupportPoint, palette: &[TipGeometry]) -> Result<(SupportParams, Option<f64>)> {
    let Some(index) = point.preset_index else {
        return Ok((base.clone(), None));
    };
    let Some(tip) = palette.get(index as usize) else {
        return Err(ResinError::invalid(format!("support preset index {index} is not in the tip palette")));
    };
    let mut used = base.clone();
    used.tip_diameter_mm = tip.tip_diameter_mm;
    used.tip_length_mm = tip.tip_length_mm;
    used.pillar_diameter_mm = tip.pillar_diameter_mm;
    used.foot_diameter_mm = tip.foot_diameter_mm;
    used.foot_height_mm = tip.foot_height_mm;
    used.validate()?;
    Ok((used, tip.tip_penetration_mm))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn preset(name: &str, tip: f64) -> TipPreset {
        TipPreset {
            name: name.to_string(),
            tip_diameter_mm: tip,
            tip_length_mm: 1.0,
            pillar_diameter_mm: 1.0,
            foot_diameter_mm: 2.0,
            foot_height_mm: 0.5,
            tip_penetration_mm: None,
        }
    }

    #[test]
    fn library_round_trips_and_refuses_duplicates() {
        let presets = vec![preset("Light", 0.3), preset("Heavy", 0.6)];
        let text = write_presets(&presets).unwrap();
        assert_eq!(parse_presets(&text).unwrap(), presets);
        let dup = vec![preset("Light", 0.3), preset("Light", 0.6)];
        assert!(write_presets(&dup).is_err());
        assert!(parse_presets(r#"{"schema":"other","presets":[]}"#).is_err());
        assert!(parse_presets(&format!(r#"{{"schema":"{PRESETS_SCHEMA}","presets":[{{"name":"A"}}]}}"#)).is_err());
    }

    #[test]
    fn mix_parses_and_assigns_by_role() {
        let presets = vec![preset("Light", 0.3), preset("Heavy", 0.6)];
        let mix = PresetMix::parse("overhang=Light, island=Heavy,lowest-band=Heavy").unwrap();
        assert!(PresetMix::parse("legs=Light").is_err());
        assert!(PresetMix::parse("island=Light,island=Heavy").is_err());
        let mut points = vec![
            SupportPoint { x: 0.0, y: 0.0, contact_z: 5.0, source: PointSource::Overhang, preset_index: None },
            SupportPoint { x: 1.0, y: 0.0, contact_z: 5.0, source: PointSource::Island, preset_index: None },
            SupportPoint { x: 2.0, y: 0.0, contact_z: 1.0, source: PointSource::Overhang, preset_index: None },
            SupportPoint { x: 3.0, y: 0.0, contact_z: 5.0, source: PointSource::Manual, preset_index: None },
        ];
        let changed = apply_mix(&mut points, &mix, &presets, Some(2.0)).unwrap();
        assert_eq!(changed, 3);
        assert_eq!(points.iter().map(|p| p.preset_index).collect::<Vec<_>>(), vec![Some(0), Some(1), Some(1), None]);
        let bad = PresetMix::parse("manual=Missing").unwrap();
        assert!(apply_mix(&mut points, &bad, &presets, None).is_err());
    }

    #[test]
    fn forgetting_a_preset_shifts_later_indices() {
        let mut points = vec![
            SupportPoint { x: 0.0, y: 0.0, contact_z: 1.0, source: PointSource::Manual, preset_index: Some(0) },
            SupportPoint { x: 0.0, y: 0.0, contact_z: 1.0, source: PointSource::Manual, preset_index: Some(1) },
            SupportPoint { x: 0.0, y: 0.0, contact_z: 1.0, source: PointSource::Manual, preset_index: Some(2) },
        ];
        forget_preset(&mut points, 1);
        assert_eq!(points.iter().map(|p| p.preset_index).collect::<Vec<_>>(), vec![Some(0), None, Some(1)]);
    }

    #[test]
    fn preset_names_and_values_are_checked() {
        assert!(preset("has space", 0.3).validate().is_err());
        assert!(preset("", 0.3).validate().is_err());
        assert!(preset("Zero", 0.0).validate().is_err());
        let mut p = preset("Deep", 0.3);
        p.tip_penetration_mm = Some(-0.1);
        assert!(p.validate().is_err());
        p.tip_penetration_mm = Some(0.2);
        assert!(p.validate().is_ok());
    }
}
