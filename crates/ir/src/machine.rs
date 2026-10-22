//! Axis-space machine description: joints, limits, kinematic chain and channels.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::check::{self, finite, required_name};
use crate::geom::Vec3;
use crate::{AXIS_LETTER_COUNT, MAX_AXES, MAX_CHANNELS, Result};

/// X Y Z A B C U V W, in that order. Further axes use an index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, schemars::JsonSchema)]
pub enum AxisLetter {
    /// Linear X.
    X,
    /// Linear Y.
    Y,
    /// Linear Z.
    Z,
    /// Rotary about X.
    A,
    /// Rotary about Y.
    B,
    /// Rotary about Z.
    C,
    /// Secondary linear parallel to X.
    U,
    /// Secondary linear parallel to Y.
    V,
    /// Secondary linear parallel to Z.
    W,
}

const AXIS_LETTERS: [AxisLetter; AXIS_LETTER_COUNT] =
    [AxisLetter::X, AxisLetter::Y, AxisLetter::Z, AxisLetter::A, AxisLetter::B, AxisLetter::C, AxisLetter::U, AxisLetter::V, AxisLetter::W];

/// Zero-based index of an axis past the nine letters. Valid values are `9..MAX_AXES`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AxisIndex {
    index: u16,
}

impl AxisIndex {
    pub fn try_new(index: u16) -> Result<Self> {
        let value = Self { index };
        value.check()?;
        Ok(value)
    }

    pub fn get(self) -> u16 {
        self.index
    }

    pub(crate) fn check(self) -> Result<()> {
        let index = usize::from(self.index);
        if !(AXIS_LETTER_COUNT..MAX_AXES).contains(&index) {
            Err(check::invalid(format!("indexed axis {index} is outside {AXIS_LETTER_COUNT}..{MAX_AXES}")))
        } else {
            Ok(())
        }
    }
}

/// Letter or index that names an axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum AxisName {
    /// One of X Y Z A B C U V W.
    Letter(AxisLetter),
    /// Indexed extra. The index is the zero-based ordinal, from 9 upward.
    Index(AxisIndex),
}

impl AxisName {
    /// Letter for ordinals `0..9`, then [`AxisIndex`] for ordinals `9..MAX_AXES`.
    pub fn for_ordinal(ordinal: usize) -> Result<Self> {
        if ordinal >= MAX_AXES {
            return Err(check::invalid(format!("axis ordinal {ordinal} is outside 0..{MAX_AXES}")));
        }
        match AXIS_LETTERS.get(ordinal) {
            Some(letter) => Ok(Self::Letter(*letter)),
            None => {
                let index = u16::try_from(ordinal).map_err(|_| check::invalid("axis ordinal does not fit in u16"))?;
                Ok(Self::Index(AxisIndex::try_new(index)?))
            }
        }
    }

    pub(crate) fn check(self) -> Result<()> {
        match self {
            Self::Letter(_) => Ok(()),
            Self::Index(index) => index.check(),
        }
    }
}

/// Linear travel or rotation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum AxisKind {
    /// Millimetres along the kinematic direction.
    Linear,
    /// Radians about the kinematic direction, right-handed.
    Rotary,
}

/// Inclusive travel limits. Millimetres when linear, radians when rotary.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AxisLimits {
    min: f64,
    max: f64,
}

impl AxisLimits {
    pub fn try_new(min: f64, max: f64) -> Result<Self> {
        let limits = Self { min, max };
        limits.check()?;
        Ok(limits)
    }

    pub fn min(self) -> f64 {
        self.min
    }

    pub fn max(self) -> f64 {
        self.max
    }

    pub(crate) fn check(self) -> Result<()> {
        let min = finite(self.min, "axis minimum")?;
        let max = finite(self.max, "axis maximum")?;
        if min > max { Err(check::invalid("axis minimum is greater than maximum")) } else { Ok(()) }
    }
}

/// Where a joint sits on its parent.
///
/// `direction` is the positive linear travel, or the rotary axis, in the parent frame.
/// `offset` is the joint origin relative to the parent origin, in millimetres.
/// `parent` is `None` when the joint is mounted on the machine base.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct KinematicNode {
    #[serde(default)]
    parent: Option<u8>,
    direction: Vec3,
    offset: Vec3,
}

impl KinematicNode {
    pub fn try_new(parent: Option<u8>, direction: Vec3, offset: Vec3) -> Result<Self> {
        let node = Self { parent, direction, offset };
        node.check()?;
        Ok(node)
    }

    pub fn parent(self) -> Option<u8> {
        self.parent
    }

    pub fn direction(self) -> Vec3 {
        self.direction
    }

    pub fn offset(self) -> Vec3 {
        self.offset
    }

    pub(crate) fn check(self) -> Result<()> {
        if let Some(parent) = self.parent
            && usize::from(parent) >= MAX_AXES
        {
            return Err(check::invalid(format!("parent axis id {parent} is outside 0..{MAX_AXES}")));
        }
        self.direction.check_non_zero("axis direction")?;
        self.offset.check("axis offset")?;
        Ok(())
    }
}

/// One joint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Axis {
    id: u8,
    name: AxisName,
    kind: AxisKind,
    #[serde(default)]
    limits: Option<AxisLimits>,
    chain: KinematicNode,
}

impl Axis {
    pub fn try_new(id: u8, name: AxisName, kind: AxisKind, limits: Option<AxisLimits>, chain: KinematicNode) -> Result<Self> {
        let axis = Self { id, name, kind, limits, chain };
        axis.check()?;
        Ok(axis)
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn name(&self) -> AxisName {
        self.name
    }

    pub fn kind(&self) -> AxisKind {
        self.kind
    }

    pub fn limits(&self) -> Option<AxisLimits> {
        self.limits
    }

    pub fn chain(&self) -> KinematicNode {
        self.chain
    }

    pub(crate) fn check(&self) -> Result<()> {
        if usize::from(self.id) >= MAX_AXES {
            return Err(check::invalid(format!("axis id {} is outside 0..{MAX_AXES}", self.id)));
        }
        self.name.check()?;
        if let Some(limits) = self.limits {
            limits.check()?;
        }
        self.chain.check()?;
        Ok(())
    }
}

/// A group of axes that move together: one head, spindle or turret.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Channel {
    id: u16,
    name: String,
    axes: Vec<u8>,
}

impl Channel {
    pub fn try_new(id: u16, name: impl Into<String>, axes: Vec<u8>) -> Result<Self> {
        let channel = Self { id, name: name.into(), axes };
        channel.check()?;
        Ok(channel)
    }

    pub fn id(&self) -> u16 {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn axes(&self) -> &[u8] {
        &self.axes
    }

    pub(crate) fn check(&self) -> Result<()> {
        required_name(&self.name, "channel name")?;
        if self.axes.is_empty() {
            return Err(check::invalid("channel has no axes"));
        }
        if self.axes.len() > MAX_AXES {
            return Err(check::invalid(format!("too many axes on channel {}: {} > {MAX_AXES}", self.id, self.axes.len())));
        }
        let mut seen = BTreeSet::new();
        for id in &self.axes {
            if usize::from(*id) >= MAX_AXES {
                return Err(check::invalid(format!("axis id {id} is outside 0..{MAX_AXES}")));
            }
            if !seen.insert(*id) {
                return Err(check::invalid(format!("channel {} lists axis {id} twice", self.id)));
            }
        }
        Ok(())
    }
}

/// The joints of one machine and the channels that group them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Machine {
    axes: Vec<Axis>,
    channels: Vec<Channel>,
}

impl Machine {
    pub fn try_new(axes: Vec<Axis>, channels: Vec<Channel>) -> Result<Self> {
        let machine = Self { axes, channels };
        machine.check()?;
        Ok(machine)
    }

    pub fn axes(&self) -> &[Axis] {
        &self.axes
    }

    pub fn channels(&self) -> &[Channel] {
        &self.channels
    }

    pub(crate) fn check(&self) -> Result<()> {
        if self.axes.len() > MAX_AXES {
            return Err(check::invalid(format!("too many axes: {} > {MAX_AXES}", self.axes.len())));
        }
        let mut ids = BTreeSet::new();
        let mut names = BTreeSet::new();
        for axis in &self.axes {
            axis.check()?;
            if !ids.insert(axis.id) {
                return Err(check::invalid(format!("duplicate axis id {}", axis.id)));
            }
            if !names.insert(axis.name) {
                return Err(check::invalid(format!("duplicate axis name {:?}", axis.name)));
            }
        }
        check_chain(&self.axes, &ids)?;
        if self.channels.len() > MAX_CHANNELS {
            return Err(check::invalid(format!("too many channels: {} > {MAX_CHANNELS}", self.channels.len())));
        }
        let mut channel_ids = BTreeSet::new();
        let mut channel_names = BTreeSet::new();
        let mut assigned = BTreeSet::new();
        for channel in &self.channels {
            channel.check()?;
            if !channel_ids.insert(channel.id) {
                return Err(check::invalid(format!("duplicate channel id {}", channel.id)));
            }
            if !channel_names.insert(channel.name.clone()) {
                return Err(check::invalid(format!("duplicate channel name {}", channel.name)));
            }
            for axis_id in &channel.axes {
                if !ids.contains(axis_id) {
                    return Err(check::invalid(format!("channel {} references missing axis {axis_id}", channel.id)));
                }
                if !assigned.insert(*axis_id) {
                    return Err(check::invalid(format!("axis {axis_id} is in more than one channel")));
                }
            }
        }
        Ok(())
    }
}

fn check_chain(axes: &[Axis], ids: &BTreeSet<u8>) -> Result<()> {
    for axis in axes {
        let mut seen = BTreeSet::from([axis.id]);
        let mut cursor = axis.chain.parent;
        let mut hops = 0;
        while let Some(parent) = cursor {
            hops += 1;
            if hops > MAX_AXES {
                return Err(check::invalid(format!("kinematic chain from axis {} is too long", axis.id)));
            }
            if !seen.insert(parent) {
                return Err(check::invalid(format!("kinematic cycle at axis {parent}")));
            }
            if !ids.contains(&parent) {
                return Err(check::invalid(format!("axis {} parent {parent} does not exist", axis.id)));
            }
            let Some(parent_axis) = axes.iter().find(|candidate| candidate.id == parent) else {
                return Err(check::invalid(format!("axis {} parent {parent} does not exist", axis.id)));
            };
            cursor = parent_axis.chain.parent;
        }
    }
    Ok(())
}
