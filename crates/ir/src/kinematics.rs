//! Rotary joints as data, and the forward map from joint angles to a tool pose.
//!
//! Angles in this API are degrees. Rotation is right-handed about the joint axis. Lengths are
//! millimetres. The JSON files under `crates/ir/kinematics/` are illustrative examples. They are
//! not a description of any real machine.
//!
//! # Frames
//!
//! - At all angles zero the part frame equals the machine frame.
//! - rotaries are listed outer first: within the table joints, the one mounted on the machine base
//!   comes first; within the head joints, the one mounted on the Z slide comes first. Table and head
//!   joints may be interleaved in the list; only the order within each carrier matters.
//! - Table joint pivot: machine coordinates at all angles zero. A second table joint's axis and
//!   pivot are given at zero angles and are carried by the first.
//! - Head joint pivot: relative to the gauge point (the point XYZ positions) at zero head angles.
//!   A second head joint is carried by the first.
//! - Tool tip at zero head angles = gauge point - tool_length * spindle_axis. Tool axis = spindle_axis.
//! - Forward: T_table = R_t1 * R_t2 (each R about its pivot/axis by its angle, zero-pose definitions);
//!   head H = R_h1 * R_h2 (about pivots relative to gauge). tip_machine = xyz + H(-L*spindle_axis);
//!   axis_machine = rot(H)*spindle_axis. tip_part = T_table^-1 tip_machine;
//!   axis_part = rot(T_table)^-1 axis_machine.

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, finite, required_name};
use crate::geom::Vec3;
use crate::rotary::RotaryLimits;

/// Which slide carries a rotary joint.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Carrier {
    /// The joint moves the work. Its pivot is in machine coordinates at zero angles.
    Table,
    /// The joint moves the tool. Its pivot is relative to the gauge point at zero head angles.
    Head,
}

/// One rotary joint. `letter` is `A`, `B` or `C`.
///
/// `axis` is normalised on load. Rotation is right-handed about it. `pivot` is a point on the
/// axis line. See the module docs for which frame that point is written in.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RotaryJoint {
    /// `A`, `B` or `C`.
    pub letter: char,
    /// Table (work) or head (tool).
    pub carrier: Carrier,
    /// Rotation axis. Stored as a unit vector after [`Kinematics::from_json`].
    pub axis: Vec3,
    /// A point on the axis line. See the module docs for the frame.
    pub pivot: Vec3,
    /// Inclusive travel limits in degrees. Absent means the joint is unlimited.
    #[serde(default)]
    pub limits: Option<RotaryLimits>,
}

fn default_spindle_axis() -> Vec3 {
    Vec3::Z
}

/// One or two rotary joints and the spindle direction at zero head angles.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Kinematics {
    /// Non-empty name. It is quoted in validation errors.
    pub name: String,
    /// Outer joint first within each [`Carrier`]. One or two entries.
    pub rotaries: Vec<RotaryJoint>,
    /// Tool axis at zero head angles. Defaults to +Z. Normalised on load.
    #[serde(default = "default_spindle_axis")]
    pub spindle_axis: Vec3,
}

/// A joint solution. `angles[i]` belongs to `rotaries[i]`, in degrees. The unused slot is 0.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Joints {
    /// Gauge-point position, millimetres, machine frame.
    pub xyz: Vec3,
    /// Rotary angles in degrees, in `rotaries` order.
    pub angles: [f64; 2],
}

/// Tool pose in the part frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToolPose {
    /// Tool tip, millimetres.
    pub tip: Vec3,
    /// Unit tool axis, from the tip toward the spindle.
    pub axis: Vec3,
}

impl Kinematics {
    /// Parse JSON, normalise each rotary axis and `spindle_axis`, then [`Self::check`].
    pub fn from_json(text: &str) -> Result<Self> {
        if text.len() > check::MAX_JSON_BYTES {
            return Err(check::invalid(format!("json is larger than {} bytes", check::MAX_JSON_BYTES)));
        }
        let mut kinematics: Self = serde_json::from_str(text).map_err(|err| crate::IrError::Json(err.to_string()))?;
        kinematics.normalise_directions();
        kinematics.check()?;
        Ok(kinematics)
    }

    /// Require a name, one or two rotary joints, letters `A`/`B`/`C` used once, and a usable axis.
    ///
    /// Travel limits are checked with [`RotaryLimits`].
    pub fn check(&self) -> Result<()> {
        required_name(&self.name, "kinematics name")?;
        if self.rotaries.len() != 1 && self.rotaries.len() != 2 {
            return Err(check::invalid(format!("kinematics `{}`: needs one or two rotary joints", self.name)));
        }
        let mut seen = [false; 3];
        for joint in &self.rotaries {
            let slot = match joint.letter {
                'A' => 0,
                'B' => 1,
                'C' => 2,
                _ => return Err(check::invalid(format!("kinematics `{}`: rotary letter must be A, B or C", self.name))),
            };
            if seen[slot] {
                return Err(check::invalid(format!("kinematics `{}`: rotary {} listed twice", self.name, joint.letter)));
            }
            seen[slot] = true;
            require_direction(
                joint.axis,
                &format!("kinematics `{}`: rotary {} axis", self.name, joint.letter),
                &format!("kinematics `{}`: rotary {} axis is zero", self.name, joint.letter),
            )?;
            joint.pivot.check(&format!("kinematics `{}`: rotary {} pivot", self.name, joint.letter))?;
            if let Some(limits) = joint.limits {
                limits.check()?;
            }
        }
        require_direction(
            self.spindle_axis,
            &format!("kinematics `{}`: spindle axis", self.name),
            &format!("kinematics `{}`: spindle axis is zero", self.name),
        )?;
        Ok(())
    }

    /// Tool tip and axis in the part frame.
    ///
    /// `tool_length` is millimetres along `spindle_axis` from the gauge point to the tip at zero
    /// head angles. It must be finite and non-negative.
    pub fn forward(&self, joints: &Joints, tool_length: f64) -> Result<ToolPose> {
        if !tool_length.is_finite() || tool_length < 0.0 {
            return Err(check::invalid("tool length must be finite and non-negative"));
        }
        self.check()?;
        joints.xyz.check("joint position")?;
        for (index, _) in self.rotaries.iter().enumerate() {
            finite(angle_at(joints.angles, index), "joint angle")?;
        }

        let spindle = unit_direction(self.spindle_axis).ok_or_else(|| check::invalid(format!("kinematics `{}`: spindle axis is zero", self.name)))?;
        let head = self.chain(joints, Carrier::Head)?;
        let table = self.chain(joints, Carrier::Table)?;
        let tip_in_gauge = scale(spindle, -tool_length);
        let tip_machine = add(joints.xyz, map_point(&head, tip_in_gauge));
        let axis_machine = map_direction(&head, spindle);
        let tip = map_point_inverse(&table, tip_machine);
        let axis_part = map_direction_inverse(&table, axis_machine);
        let axis = unit_direction(axis_part).ok_or_else(|| check::invalid(format!("kinematics `{}`: spindle axis is zero", self.name)))?;
        Ok(ToolPose { tip, axis })
    }

    /// Whether each recorded rotary angle lies inside that joint's travel limits.
    ///
    /// A joint with no limits accepts any finite angle. Angles past the recorded joints are ignored.
    pub fn within_limits(&self, angles: [f64; 2]) -> bool {
        self.rotaries.iter().enumerate().all(|(index, joint)| {
            let angle = angle_at(angles, index);
            match joint.limits {
                Some(limits) => limits.contains(angle),
                None => angle.is_finite(),
            }
        })
    }

    fn normalise_directions(&mut self) {
        for joint in &mut self.rotaries {
            if let Some(axis) = unit_direction(joint.axis) {
                joint.axis = axis;
            }
        }
        if let Some(axis) = unit_direction(self.spindle_axis) {
            self.spindle_axis = axis;
        }
    }

    fn chain(&self, joints: &Joints, carrier: Carrier) -> Result<Vec<Spin>> {
        let mut chain = Vec::with_capacity(self.rotaries.len());
        for (index, joint) in self.rotaries.iter().enumerate() {
            if joint.carrier != carrier {
                continue;
            }
            let axis = unit_direction(joint.axis)
                .ok_or_else(|| check::invalid(format!("kinematics `{}`: rotary {} axis is zero", self.name, joint.letter)))?;
            chain.push(Spin { axis, pivot: joint.pivot, angle_deg: angle_at(joints.angles, index) });
        }
        Ok(chain)
    }
}

/// One joint after its axis has been normalised. `angle_deg` is the commanded angle.
struct Spin {
    axis: Vec3,
    pivot: Vec3,
    angle_deg: f64,
}

fn angle_at(angles: [f64; 2], index: usize) -> f64 {
    match index {
        0 => angles[0],
        1 => angles[1],
        _ => 0.0,
    }
}

fn require_direction(axis: Vec3, what: &str, zero_message: &str) -> Result<()> {
    axis.check(what)?;
    if unit_direction(axis).is_none() {
        return Err(check::invalid(zero_message.to_string()));
    }
    Ok(())
}

/// Active right-hand rotation of `v` about a unit `axis` by `angle_deg` degrees (Rodrigues).
fn rotate_vector(axis: Vec3, v: Vec3, angle_deg: f64) -> Vec3 {
    let theta = angle_deg.to_radians();
    let (sin, cos) = theta.sin_cos();
    let along = dot(axis, v);
    let crossed = cross(axis, v);
    add(add(scale(v, cos), scale(crossed, sin)), scale(axis, along * (1.0 - cos)))
}

fn rotate_point(axis: Vec3, pivot: Vec3, point: Vec3, angle_deg: f64) -> Vec3 {
    add(pivot, rotate_vector(axis, sub(point, pivot), angle_deg))
}

/// `chain` is outer-first. `R_outer * R_inner` applies the inner joint first.
fn map_point(chain: &[Spin], point: Vec3) -> Vec3 {
    let mut point = point;
    for spin in chain.iter().rev() {
        point = rotate_point(spin.axis, spin.pivot, point, spin.angle_deg);
    }
    point
}

fn map_point_inverse(chain: &[Spin], point: Vec3) -> Vec3 {
    let mut point = point;
    for spin in chain {
        point = rotate_point(spin.axis, spin.pivot, point, -spin.angle_deg);
    }
    point
}

fn map_direction(chain: &[Spin], direction: Vec3) -> Vec3 {
    let mut direction = direction;
    for spin in chain.iter().rev() {
        direction = rotate_vector(spin.axis, direction, spin.angle_deg);
    }
    direction
}

fn map_direction_inverse(chain: &[Spin], direction: Vec3) -> Vec3 {
    let mut direction = direction;
    for spin in chain {
        direction = rotate_vector(spin.axis, direction, -spin.angle_deg);
    }
    direction
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.x() + b.x(), a.y() + b.y(), a.z() + b.z())
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.x() - b.x(), a.y() - b.y(), a.z() - b.z())
}

fn scale(a: Vec3, factor: f64) -> Vec3 {
    Vec3::from_xyz(a.x() * factor, a.y() * factor, a.z() * factor)
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x() * b.x() + a.y() * b.y() + a.z() * b.z()
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::from_xyz(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x())
}

fn unit_direction(v: Vec3) -> Option<Vec3> {
    if !v.x().is_finite() || !v.y().is_finite() || !v.z().is_finite() {
        return None;
    }
    if v.x().abs() <= check::MIN_COMPONENT && v.y().abs() <= check::MIN_COMPONENT && v.z().abs() <= check::MIN_COMPONENT {
        return None;
    }
    let len = dot(v, v).sqrt();
    if !len.is_finite() || len == 0.0 {
        return None;
    }
    Some(scale(v, 1.0 / len))
}
