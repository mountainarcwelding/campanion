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
//!
//! # Inverse
//!
//! [`Kinematics::inverse`] solves rotary angles for the tool axis, then the gauge position that
//! puts the tip on the requested point. [`Kinematics::choose`] picks one of those solutions.

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

/// How [`Kinematics::choose`] picks one inverse solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Choice {
    /// Minimum of `max(|a0 - p0|, |a1 - p1|)`. Equal distances keep inverse order.
    Nearest([f64; 2]),
    /// First solution whose first angle is at least `-1e-9` degrees, otherwise the first solution.
    FirstPositive,
    /// First solution whose first angle is at most `1e-9` degrees, otherwise the first solution.
    FirstNegative,
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

    /// Joint solutions whose forward pose is `pose`.
    ///
    /// `hint` is used for a joint the tool axis leaves free. Solutions are sorted by
    /// `(|angles[0]|, angles[0], angles[1])`. No orientation solution is
    /// `tool axis (I, J, K) is not reachable`. An orientation that misses every travel limit is
    /// `tool axis (I, J, K) is outside the rotary limits`. `I`, `J` and `K` are the normalised
    /// target axis, six digits past the decimal.
    pub fn inverse(&self, pose: &ToolPose, tool_length: f64, hint: [f64; 2]) -> Result<Vec<Joints>> {
        if !tool_length.is_finite() || tool_length < 0.0 {
            return Err(check::invalid("tool length must be finite and non-negative"));
        }
        let Some(target) = unit_direction(pose.axis) else {
            return Err(check::invalid("tool axis is zero"));
        };
        self.check()?;
        pose.tip.check("tool tip")?;
        let spindle = unit_direction(self.spindle_axis).ok_or_else(|| check::invalid(format!("kinematics `{}`: spindle axis is zero", self.name)))?;

        let raw = self.orientation_angles(spindle, target, hint)?;
        if raw.is_empty() {
            return Err(tool_axis_error(target, "not reachable"));
        }
        let wrapped = self.expand_wraps(&raw);
        if wrapped.is_empty() {
            return Err(tool_axis_error(target, "outside the rotary limits"));
        }

        let mut solutions = Vec::with_capacity(wrapped.len());
        for angles in wrapped {
            let at_origin = Joints { xyz: Vec3::ZERO, angles };
            let pose0 = self.forward(&at_origin, tool_length)?;
            // Drop a branch that does not put the spindle on the target axis.
            if norm(sub(pose0.axis, target)) > 1e-9 {
                continue;
            }
            let table = self.chain(&at_origin, Carrier::Table)?;
            let xyz = map_direction(&table, sub(pose.tip, pose0.tip));
            xyz.check("joint position")?;
            solutions.push(Joints { xyz, angles });
        }
        if solutions.is_empty() {
            return Err(tool_axis_error(target, "not reachable"));
        }
        solutions.sort_by(|left, right| {
            left.angles[0]
                .abs()
                .total_cmp(&right.angles[0].abs())
                .then(left.angles[0].total_cmp(&right.angles[0]))
                .then(left.angles[1].total_cmp(&right.angles[1]))
        });
        Ok(solutions)
    }

    /// One solution from [`Self::inverse`].
    ///
    /// [`Choice::Nearest`] passes its angles as the hint. The other choices hint `[0.0, 0.0]`.
    pub fn choose(&self, pose: &ToolPose, tool_length: f64, choice: Choice) -> Result<Joints> {
        let hint = match choice {
            Choice::Nearest(hint) => hint,
            Choice::FirstPositive | Choice::FirstNegative => [0.0, 0.0],
        };
        let sols = self.inverse(pose, tool_length, hint)?;
        let picked = match choice {
            Choice::Nearest(preferred) => nearest(&sols, preferred),
            Choice::FirstPositive => sols.iter().copied().find(|sol| sol.angles[0] >= -EPS).or_else(|| sols.first().copied()),
            Choice::FirstNegative => sols.iter().copied().find(|sol| sol.angles[0] <= EPS).or_else(|| sols.first().copied()),
        };
        picked.ok_or_else(|| tool_axis_error(unit_direction(pose.axis).unwrap_or(Vec3::ZERO), "not reachable"))
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

    fn orientation_angles(&self, spindle: Vec3, target: Vec3, hint: [f64; 2]) -> Result<Vec<[f64; 2]>> {
        let (tables, heads) = self.carrier_axes()?;
        if self.rotaries.len() == 1 {
            return Ok(one_joint_angles(&tables, &heads, spindle, target, hint));
        }
        let Some(map) = axis_map(&tables, &heads) else {
            return Ok(Vec::new());
        };
        if norm(cross(map.wa, map.wb)) < EPS {
            return Ok(parallel_angles(&map, spindle, target, hint));
        }
        let mut raw = Vec::new();
        for (x, y) in sp2(map.wa, map.wb, spindle, target) {
            raw.push(map.angles(x, y, hint));
        }
        Ok(raw)
    }

    fn expand_wraps(&self, raw: &[[f64; 2]]) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for angles in raw {
            let first = angle_candidates(angles[0], self.rotaries.first().and_then(|joint| joint.limits));
            let second = angle_candidates(angles[1], self.rotaries.get(1).and_then(|joint| joint.limits));
            for a0 in first {
                for a1 in &second {
                    let pair = [a0, *a1];
                    if !out.iter().any(|have| same_angles(*have, pair)) {
                        out.push(pair);
                    }
                }
            }
        }
        out
    }

    fn carrier_axes(&self) -> Result<(Vec<(usize, Vec3)>, Vec<(usize, Vec3)>)> {
        let mut tables = Vec::new();
        let mut heads = Vec::new();
        for (index, joint) in self.rotaries.iter().enumerate() {
            let axis = unit_direction(joint.axis)
                .ok_or_else(|| check::invalid(format!("kinematics `{}`: rotary {} axis is zero", self.name, joint.letter)))?;
            match joint.carrier {
                Carrier::Table => tables.push((index, axis)),
                Carrier::Head => heads.push((index, axis)),
            }
        }
        Ok((tables, heads))
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

const EPS: f64 = 1e-9;
const SP1_TOL: f64 = 1e-7;

/// Angle about `w` taking `u` to `v`. `Free` means `u` is parallel to `w`, so any angle works.
#[derive(Clone, Copy)]
enum SolvedAngle {
    Fixed(f64),
    Free,
}

/// `R(wa, x) R(wb, y)` in the orientation equation, and how `(x, y)` map onto joint angles.
#[derive(Clone, Copy)]
struct AxisMap {
    wa: Vec3,
    wb: Vec3,
    x_index: usize,
    y_index: usize,
    x_sign: f64,
    y_sign: f64,
    inner_index: usize,
}

impl AxisMap {
    fn angles(self, x: SolvedAngle, y: SolvedAngle, hint: [f64; 2]) -> [f64; 2] {
        let mut angles = [0.0; 2];
        put_angle(&mut angles, self.x_index, apply_angle(x, self.x_sign, hint_at(hint, self.x_index)));
        put_angle(&mut angles, self.y_index, apply_angle(y, self.y_sign, hint_at(hint, self.y_index)));
        angles
    }
}

fn one_joint_angles(tables: &[(usize, Vec3)], heads: &[(usize, Vec3)], spindle: Vec3, target: Vec3, hint: [f64; 2]) -> Vec<[f64; 2]> {
    let (index, axis, sign) = if let Some((index, axis)) = lone(tables) {
        (index, axis, -1.0)
    } else if let Some((index, axis)) = lone(heads) {
        (index, axis, 1.0)
    } else {
        return Vec::new();
    };
    let Some(solved) = sp1(axis, spindle, target) else {
        return Vec::new();
    };
    let mut angles = [0.0; 2];
    put_angle(&mut angles, index, apply_angle(solved, sign, hint_at(hint, index)));
    vec![angles]
}

fn axis_map(tables: &[(usize, Vec3)], heads: &[(usize, Vec3)]) -> Option<AxisMap> {
    if let Some(((outer, axis_o), (inner, axis_i))) = two(tables) {
        // T^-1 = R(inner, -a_i) R(outer, -a_o).
        Some(AxisMap { wa: axis_i, wb: axis_o, x_index: inner, y_index: outer, x_sign: -1.0, y_sign: -1.0, inner_index: inner })
    } else if let Some(((outer, axis_o), (inner, axis_i))) = two(heads) {
        // H = R(outer, a_o) R(inner, a_i).
        Some(AxisMap { wa: axis_o, wb: axis_i, x_index: outer, y_index: inner, x_sign: 1.0, y_sign: 1.0, inner_index: inner })
    } else if let (Some((table, axis_t)), Some((head, axis_h))) = (lone(tables), lone(heads)) {
        // T^-1 H = R(table, -a_t) R(head, a_h). The head is the inner factor.
        Some(AxisMap { wa: axis_t, wb: axis_h, x_index: table, y_index: head, x_sign: -1.0, y_sign: 1.0, inner_index: head })
    } else {
        None
    }
}

fn parallel_angles(map: &AxisMap, spindle: Vec3, target: Vec3, hint: [f64; 2]) -> Vec<[f64; 2]> {
    let Some(total) = sp1(map.wa, spindle, target) else {
        return Vec::new();
    };
    // wb = sigma * wa, so R(wa, x) R(wb, y) = R(wa, x + sigma * y).
    let sigma = if dot(map.wa, map.wb) >= 0.0 { 1.0 } else { -1.0 };
    let mut angles = [0.0; 2];
    put_angle(&mut angles, map.inner_index, hint_at(hint, map.inner_index));
    match total {
        SolvedAngle::Free => {
            let other = if map.inner_index == map.x_index { map.y_index } else { map.x_index };
            put_angle(&mut angles, other, hint_at(hint, other));
        }
        SolvedAngle::Fixed(total) => {
            if map.inner_index == map.x_index {
                let x = map.x_sign * hint_at(hint, map.x_index);
                let y = sigma * (total - x);
                put_angle(&mut angles, map.y_index, map.y_sign * y);
            } else {
                let y = map.y_sign * hint_at(hint, map.y_index);
                let x = total - sigma * y;
                put_angle(&mut angles, map.x_index, map.x_sign * x);
            }
        }
    }
    vec![angles]
}

/// Rotation angle in degrees about a unit `w` taking `u` to `v`.
fn sp1(w: Vec3, u: Vec3, v: Vec3) -> Option<SolvedAngle> {
    let along_u = dot(w, u);
    let along_v = dot(w, v);
    if (along_u - along_v).abs() > SP1_TOL {
        return None;
    }
    let u_perp = sub(u, scale(w, along_u));
    let v_perp = sub(v, scale(w, along_v));
    let u_len = norm(u_perp);
    let v_len = norm(v_perp);
    if (u_len - v_len).abs() > SP1_TOL {
        return None;
    }
    if u_len < EPS {
        return Some(SolvedAngle::Free);
    }
    let sin_term = dot(w, cross(u_perp, v_perp));
    let cos_term = dot(u_perp, v_perp);
    Some(SolvedAngle::Fixed(sin_term.atan2(cos_term).to_degrees()))
}

/// Solutions `(x, y)` of `R(wa, x) R(wb, y) p = q`. `wa` and `wb` are unit and not parallel.
///
/// `c = R(wb, y) p` shares its `wa` component with `q`. The old residual
/// `sqrt(|p|^2 - |c_plane|^2)` underflows once the perpendicular part is below about `1e-8`:
/// the squared length rounds it away and the tilt comes back as 0, so the axis misses by more
/// than `1e-9`. Build `c` from the perpendicular part of `q` instead, which keeps a small tilt.
fn sp2(wa: Vec3, wb: Vec3, p: Vec3, q: Vec3) -> Vec<(SolvedAngle, SolvedAngle)> {
    let d = dot(wa, wb);
    let qa = dot(wa, q);
    let pb = dot(wb, p);
    let q_perp = sub(q, scale(wa, qa));
    let rho = norm(q_perp);
    let wb_perp = sub(wb, scale(wa, d));
    let wb_len = norm(wb_perp);
    if wb_len <= EPS {
        return Vec::new();
    }
    if rho == 0.0 {
        // `q` is parallel to `wa`, so `c` is too. The angle about `wa` is free.
        return solved_pair(wa, wb, p, q, scale(wa, qa)).into_iter().collect();
    }
    let cos_phi = (pb - d * qa) / (rho * wb_len);
    if cos_phi.abs() > 1.0 + SP1_TOL {
        return Vec::new();
    }
    let cos_phi = cos_phi.clamp(-1.0, 1.0);
    let sin_abs = ((1.0 - cos_phi) * (1.0 + cos_phi)).max(0.0).sqrt();
    let e1 = scale(wb_perp, 1.0 / wb_len);
    let e2 = cross(wa, e1);
    let e2_len = norm(e2);
    if e2_len <= EPS {
        return Vec::new();
    }
    let e2 = scale(e2, 1.0 / e2_len);
    let signs: &[f64] = if sin_abs <= EPS { &[1.0] } else { &[1.0, -1.0] };
    let mut out = Vec::new();
    for sign in signs {
        let dir = add(scale(e1, cos_phi), scale(e2, sign * sin_abs));
        let c = add(scale(wa, qa), scale(dir, rho));
        if let Some(pair) = solved_pair(wa, wb, p, q, c) {
            out.push(pair);
        }
    }
    out
}

fn solved_pair(wa: Vec3, wb: Vec3, p: Vec3, q: Vec3, c: Vec3) -> Option<(SolvedAngle, SolvedAngle)> {
    let y = sp1(wb, p, c)?;
    let x = sp1(wa, c, q)?;
    Some((x, y))
}

fn apply_angle(solved: SolvedAngle, sign: f64, hint: f64) -> f64 {
    match solved {
        SolvedAngle::Free => hint,
        SolvedAngle::Fixed(angle) => sign * angle,
    }
}

fn lone(items: &[(usize, Vec3)]) -> Option<(usize, Vec3)> {
    match items {
        [item] => Some(*item),
        _ => None,
    }
}

fn two(items: &[(usize, Vec3)]) -> Option<((usize, Vec3), (usize, Vec3))> {
    match items {
        [first, second] => Some((*first, *second)),
        _ => None,
    }
}

fn hint_at(hint: [f64; 2], index: usize) -> f64 {
    match index {
        0 => hint[0],
        _ => hint[1],
    }
}

fn put_angle(angles: &mut [f64; 2], index: usize, value: f64) {
    if let Some(slot) = angles.get_mut(index) {
        *slot = value;
    }
}

fn same_angles(left: [f64; 2], right: [f64; 2]) -> bool {
    (left[0] - right[0]).abs() < EPS && (left[1] - right[1]).abs() < EPS
}

fn angle_candidates(angle: f64, limits: Option<RotaryLimits>) -> Vec<f64> {
    let mut out = Vec::new();
    for k in -2..=2 {
        let candidate = angle + 360.0 * f64::from(k);
        let keep = match limits {
            Some(limits) => limits.contains(candidate),
            None => candidate > -180.0 && candidate <= 180.0,
        };
        if keep {
            out.push(candidate);
        }
    }
    out
}

fn nearest(sols: &[Joints], preferred: [f64; 2]) -> Option<Joints> {
    let mut best: Option<(Joints, f64)> = None;
    for sol in sols {
        let distance = (sol.angles[0] - preferred[0]).abs().max((sol.angles[1] - preferred[1]).abs());
        match best {
            Some((_, best_distance)) if distance >= best_distance => {}
            _ => best = Some((*sol, distance)),
        }
    }
    best.map(|(sol, _)| sol)
}

fn norm(v: Vec3) -> f64 {
    dot(v, v).sqrt()
}

fn comp_for_error(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

fn tool_axis_error(axis: Vec3, kind: &str) -> crate::IrError {
    check::invalid(
        format!("tool axis ({:.6}, {:.6}, {:.6}) is {kind}", comp_for_error(axis.x()), comp_for_error(axis.y()), comp_for_error(axis.z()),),
    )
}

#[cfg(test)]
mod solve_tests {
    use super::{SolvedAngle, Vec3, rotate_vector, scale, sp2, sub};

    fn gap(left: Vec3, right: Vec3) -> f64 {
        let delta = sub(left, right);
        super::dot(delta, delta).sqrt()
    }

    fn fixed(angle: SolvedAngle) -> f64 {
        match angle {
            SolvedAngle::Fixed(value) => value,
            SolvedAngle::Free => 0.0,
        }
    }

    #[test]
    fn sp2_rotates_back_to_the_target() {
        let wa = Vec3::Z;
        let wb = Vec3::Y;
        let p = Vec3::X;
        let x_deg = 40.0;
        let y_deg = -25.0;
        let q = rotate_vector(wa, rotate_vector(wb, p, y_deg), x_deg);
        let sols = sp2(wa, wb, p, q);
        assert!(!sols.is_empty());
        let mut matched = false;
        for (x, y) in sols {
            let x = fixed(x);
            let y = fixed(y);
            let back = rotate_vector(wa, rotate_vector(wb, p, y), x);
            let gap = gap(back, q);
            assert!(gap <= 1e-9, "rotated q gap {gap}");
            if (x - x_deg).abs() <= 1e-7 && (y - y_deg).abs() <= 1e-7 {
                matched = true;
            }
        }
        assert!(matched);
        assert!(sp2(wa, wb, p, scale(p, 2.0)).is_empty());
    }
}
