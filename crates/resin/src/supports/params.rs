//! User-supplied resin support and raft parameters. Nothing here is defaulted.

use serde::Deserialize;

use crate::error::{ResinError, Result};

/// CLI flags that must be supplied (or present in `--support-params`) before supports are generated.
pub const REQUIRED_FLAGS: &[&str] = &[
    "--raft-chamfer-angle",
    "--raft-thickness",
    "--raft-margin",
    "--raft-corner-relief",
    "--overhang-angle",
    "--support-spacing",
    "--tip-diameter",
    "--tip-length",
    "--pillar-diameter",
    "--foot-diameter",
    "--foot-height",
    "--part-lift",
];

/// Every length is millimetres; every angle is degrees. All fields are required and validated by [`SupportParams::validate`].
#[derive(Clone, Debug, PartialEq)]
pub struct SupportParams {
    /// Chamfer angle of the raft perimeter, measured from the build plate (horizontal). Exclusive of 0 and 90.
    ///
    /// The top outline is inset from the bottom outline by `thickness / tan(angle)`.
    pub raft_chamfer_angle_deg: f64,
    pub raft_thickness_mm: f64,
    /// Outward offset of the raft's bottom outline around the part (and support) footprint.
    pub raft_margin_mm: f64,
    /// Length, in millimetres, by which each raft corner is cut back along both edges. `0` keeps sharp corners.
    pub raft_corner_relief_mm: f64,
    /// Minimum angle from vertical at which a downward-facing triangle is treated as an overhang.
    ///
    /// `0` includes every downward face (including nearly vertical walls); `90` includes only faces
    /// parallel to the plate. A mid value such as `45` includes faces at 45° or more from vertical.
    pub overhang_angle_deg: f64,
    pub support_spacing_mm: f64,
    pub tip_diameter_mm: f64,
    pub tip_length_mm: f64,
    pub pillar_diameter_mm: f64,
    pub foot_diameter_mm: f64,
    pub foot_height_mm: f64,
    /// Distance from the top of the raft to the lowest point of the part.
    pub part_lift_mm: f64,
}

/// Partial parameter set from a JSON file or from CLI flags. Missing fields stay `None`.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SupportParamsOverlay {
    pub raft_chamfer_angle_deg: Option<f64>,
    pub raft_thickness_mm: Option<f64>,
    pub raft_margin_mm: Option<f64>,
    pub raft_corner_relief_mm: Option<f64>,
    pub overhang_angle_deg: Option<f64>,
    pub support_spacing_mm: Option<f64>,
    pub tip_diameter_mm: Option<f64>,
    pub tip_length_mm: Option<f64>,
    pub pillar_diameter_mm: Option<f64>,
    pub foot_diameter_mm: Option<f64>,
    pub foot_height_mm: Option<f64>,
    pub part_lift_mm: Option<f64>,
}

impl SupportParamsOverlay {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    /// Flags override file values.
    pub fn merge(&self, flags: &Self) -> Self {
        Self {
            raft_chamfer_angle_deg: flags.raft_chamfer_angle_deg.or(self.raft_chamfer_angle_deg),
            raft_thickness_mm: flags.raft_thickness_mm.or(self.raft_thickness_mm),
            raft_margin_mm: flags.raft_margin_mm.or(self.raft_margin_mm),
            raft_corner_relief_mm: flags.raft_corner_relief_mm.or(self.raft_corner_relief_mm),
            overhang_angle_deg: flags.overhang_angle_deg.or(self.overhang_angle_deg),
            support_spacing_mm: flags.support_spacing_mm.or(self.support_spacing_mm),
            tip_diameter_mm: flags.tip_diameter_mm.or(self.tip_diameter_mm),
            tip_length_mm: flags.tip_length_mm.or(self.tip_length_mm),
            pillar_diameter_mm: flags.pillar_diameter_mm.or(self.pillar_diameter_mm),
            foot_diameter_mm: flags.foot_diameter_mm.or(self.foot_diameter_mm),
            foot_height_mm: flags.foot_height_mm.or(self.foot_height_mm),
            part_lift_mm: flags.part_lift_mm.or(self.part_lift_mm),
        }
    }

    /// Build a complete, validated parameter set. Missing fields are listed; nothing is assumed.
    pub fn complete(&self) -> Result<SupportParams> {
        let mut missing: Vec<&str> = Vec::new();
        let mut pick = |flag: &'static str, value: Option<f64>| -> f64 {
            match value {
                Some(v) => v,
                None => {
                    missing.push(flag);
                    0.0
                }
            }
        };
        let params = SupportParams {
            raft_chamfer_angle_deg: pick("--raft-chamfer-angle", self.raft_chamfer_angle_deg),
            raft_thickness_mm: pick("--raft-thickness", self.raft_thickness_mm),
            raft_margin_mm: pick("--raft-margin", self.raft_margin_mm),
            raft_corner_relief_mm: pick("--raft-corner-relief", self.raft_corner_relief_mm),
            overhang_angle_deg: pick("--overhang-angle", self.overhang_angle_deg),
            support_spacing_mm: pick("--support-spacing", self.support_spacing_mm),
            tip_diameter_mm: pick("--tip-diameter", self.tip_diameter_mm),
            tip_length_mm: pick("--tip-length", self.tip_length_mm),
            pillar_diameter_mm: pick("--pillar-diameter", self.pillar_diameter_mm),
            foot_diameter_mm: pick("--foot-diameter", self.foot_diameter_mm),
            foot_height_mm: pick("--foot-height", self.foot_height_mm),
            part_lift_mm: pick("--part-lift", self.part_lift_mm),
        };
        if !missing.is_empty() {
            return Err(ResinError::invalid(format!("no defaults are assumed; give {} (or --support-params FILE.json)", missing.join(", "))));
        }
        params.validate()?;
        Ok(params)
    }
}

impl SupportParams {
    /// Horizontal inset of the raft top outline relative to the bottom, `thickness / tan(angle)`.
    pub fn chamfer_inset_mm(&self) -> Result<f64> {
        let angle = self.raft_chamfer_angle_deg;
        if !angle.is_finite() || angle <= 0.0 || angle >= 90.0 {
            return Err(ResinError::invalid("raft chamfer angle must be between 0 and 90 degrees, exclusive"));
        }
        let inset = self.raft_thickness_mm / angle.to_radians().tan();
        if !inset.is_finite() || inset <= 0.0 {
            return Err(ResinError::invalid("raft chamfer inset is not a usable length"));
        }
        Ok(inset)
    }

    pub fn validate(&self) -> Result<()> {
        fn finite_name(value: f64, name: &str) -> Result<f64> {
            if value.is_finite() { Ok(value) } else { Err(ResinError::invalid(format!("{name} is not a finite number"))) }
        }
        fn positive(value: f64, name: &str) -> Result<()> {
            let value = finite_name(value, name)?;
            if value > 0.0 { Ok(()) } else { Err(ResinError::invalid(format!("{name} must be greater than 0"))) }
        }
        fn non_negative(value: f64, name: &str) -> Result<()> {
            let value = finite_name(value, name)?;
            if value >= 0.0 { Ok(()) } else { Err(ResinError::invalid(format!("{name} must be at least 0"))) }
        }

        positive(self.raft_thickness_mm, "raft thickness")?;
        positive(self.raft_margin_mm, "raft margin")?;
        non_negative(self.raft_corner_relief_mm, "raft corner relief")?;
        finite_name(self.raft_chamfer_angle_deg, "raft chamfer angle")?;
        self.chamfer_inset_mm()?;
        finite_name(self.overhang_angle_deg, "overhang angle")?;
        if !(0.0..=90.0).contains(&self.overhang_angle_deg) {
            return Err(ResinError::invalid("overhang angle must be between 0 and 90 degrees, inclusive"));
        }
        positive(self.support_spacing_mm, "support spacing")?;
        positive(self.tip_diameter_mm, "tip diameter")?;
        positive(self.tip_length_mm, "tip length")?;
        positive(self.pillar_diameter_mm, "pillar diameter")?;
        positive(self.foot_diameter_mm, "foot diameter")?;
        non_negative(self.foot_height_mm, "foot height")?;
        positive(self.part_lift_mm, "part lift")?;
        if self.foot_diameter_mm < self.pillar_diameter_mm {
            return Err(ResinError::invalid("foot diameter must be at least the pillar diameter"));
        }
        if self.part_lift_mm < self.tip_length_mm + self.foot_height_mm {
            return Err(ResinError::invalid(format!(
                "part lift ({:.6} mm) must be at least tip length plus foot height ({:.6} mm)",
                self.part_lift_mm,
                self.tip_length_mm + self.foot_height_mm
            )));
        }
        Ok(())
    }
}

/// Read a JSON object of support parameters. Unknown keys are refused so typoes are not ignored.
pub fn parse_support_params_json(text: &str) -> Result<SupportParamsOverlay> {
    serde_json::from_str(text).map_err(|err| ResinError::invalid(format!("support params: {err}")))
}
