//! Tool-space and axis-space motion.
//!
//! Tool-space moves carry a [`Pose`]: tip position plus tool axis. Axis-space moves carry joint
//! targets. Feeds are millimetres per minute. An arc lies in one plane and is at most one full
//! turn, the same rule as `campanion-path`. A helix adds a sweep in radians along the plane normal.

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, positive};
use crate::geom::{ArcSense, Plane, Pose, Vec3};
use crate::process::Event;

/// Why a feed move exists. Rapids have no role.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum MoveRole {
    /// Cutting feed.
    Cut,
    /// Approach onto the profile.
    LeadIn,
    /// Departure off the profile.
    LeadOut,
    /// Vertical entry.
    Plunge,
    /// Vertical exit.
    Retract,
}

/// Where compensation was applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CompensationMode {
    /// The path is already offset by the tool radius or half the kerf.
    Computed,
    /// A later post should keep the tool to the left of travel.
    ControllerLeft,
    /// A later post should keep the tool to the right of travel.
    ControllerRight,
    /// No radius compensation.
    Off,
}

/// Operation feeds, millimetres per minute.
///
/// Each cutting move also stores its own feed. Role-specific fields override `cut` when a path
/// is converted. `None` means "use `cut`".
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Feeds {
    cut: f64,
    #[serde(default)]
    lead_in: Option<f64>,
    #[serde(default)]
    lead_out: Option<f64>,
    #[serde(default)]
    plunge: Option<f64>,
    #[serde(default)]
    retract: Option<f64>,
}

impl Feeds {
    pub fn try_new(cut: f64, lead_in: Option<f64>, lead_out: Option<f64>, plunge: Option<f64>, retract: Option<f64>) -> Result<Self> {
        let feeds = Self { cut, lead_in, lead_out, plunge, retract };
        feeds.check()?;
        Ok(feeds)
    }

    pub fn cut(self) -> f64 {
        self.cut
    }

    pub fn lead_in(self) -> Option<f64> {
        self.lead_in
    }

    pub fn lead_out(self) -> Option<f64> {
        self.lead_out
    }

    pub fn plunge(self) -> Option<f64> {
        self.plunge
    }

    pub fn retract(self) -> Option<f64> {
        self.retract
    }

    /// Feed for `role`, falling back to the cutting feed.
    pub fn resolve(self, role: MoveRole) -> f64 {
        let specific = match role {
            MoveRole::Cut => None,
            MoveRole::LeadIn => self.lead_in,
            MoveRole::LeadOut => self.lead_out,
            MoveRole::Plunge => self.plunge,
            MoveRole::Retract => self.retract,
        };
        specific.unwrap_or(self.cut)
    }

    pub(crate) fn check(self) -> Result<()> {
        positive(self.cut, "cut feed")?;
        for (feed, what) in
            [(self.lead_in, "lead-in feed"), (self.lead_out, "lead-out feed"), (self.plunge, "plunge feed"), (self.retract, "retract feed")]
        {
            if let Some(feed) = feed {
                positive(feed, what)?;
            }
        }
        Ok(())
    }
}

/// One tab left uncut along a profile. Lengths are millimetres along the path. Height is a lift in Z.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ProfileTab {
    start: f64,
    width: f64,
    #[serde(default)]
    height: Option<f64>,
}

impl ProfileTab {
    pub fn try_new(start: f64, width: f64, height: Option<f64>) -> Result<Self> {
        let tab = Self { start, width, height };
        tab.check()?;
        Ok(tab)
    }

    pub fn start(self) -> f64 {
        self.start
    }

    pub fn width(self) -> f64 {
        self.width
    }

    pub fn height(self) -> Option<f64> {
        self.height
    }

    pub(crate) fn check(self) -> Result<()> {
        check::non_negative(self.start, "tab start")?;
        check::non_negative(self.width, "tab width")?;
        if let Some(height) = self.height {
            check::finite(height, "tab height")?;
        }
        Ok(())
    }
}

/// Tool-space motion. `feed` is millimetres per minute. Rapid feed is `None` for a machine rapid.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum ClMotion {
    /// Non-cutting positioning move.
    Rapid {
        /// End pose.
        to: Pose,
        /// Optional rapid feed.
        #[serde(default)]
        feed: Option<f64>,
    },
    /// Straight feed.
    Linear {
        /// End pose.
        to: Pose,
        /// Feed.
        feed: f64,
        /// Cut, lead, plunge or retract.
        role: MoveRole,
    },
    /// Circular feed of at most one full turn. The sense and the endpoints pick the sweep.
    Arc {
        /// End pose.
        to: Pose,
        /// Plane.
        plane: Plane,
        /// Arc centre, millimetres.
        center: Vec3,
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Feed.
        feed: f64,
        /// Cut, lead, plunge or retract.
        role: MoveRole,
    },
    /// Circular feed plus travel along the plane normal. `sweep` is radians and may exceed one turn.
    Helix {
        /// End pose.
        to: Pose,
        /// Plane.
        plane: Plane,
        /// A point on the helix axis, millimetres.
        center: Vec3,
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Absolute sweep in radians. One full turn is `2π`.
        sweep: f64,
        /// Feed.
        feed: f64,
        /// Cut, lead, plunge or retract.
        role: MoveRole,
    },
    /// Spindle-synchronised straight feed (single-point threading). `lead` is millimetres per spindle revolution. Turning programs only.
    Thread {
        /// End pose.
        to: Pose,
        /// Lead, millimetres per spindle revolution.
        lead: f64,
    },
}

impl ClMotion {
    pub(crate) fn check(&self) -> Result<()> {
        match self {
            Self::Rapid { to, feed } => {
                to.check()?;
                if let Some(feed) = feed {
                    positive(*feed, "rapid feed")?;
                }
            }
            Self::Linear { to, feed, .. } => {
                to.check()?;
                positive(*feed, "feed")?;
            }
            Self::Arc { to, center, feed, .. } => {
                to.check()?;
                center.check("arc centre")?;
                positive(*feed, "feed")?;
            }
            Self::Helix { to, center, sweep, feed, .. } => {
                to.check()?;
                center.check("helix centre")?;
                positive(*sweep, "helix sweep")?;
                positive(*feed, "feed")?;
            }
            Self::Thread { to, lead } => {
                to.check()?;
                positive(*lead, "thread lead")?;
                if *lead > 1000.0 {
                    return Err(check::invalid("thread lead must be <= 1000"));
                }
            }
        }
        Ok(())
    }
}

/// One tool-space step: a motion or an event.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum ClBlock {
    /// Geometric move.
    Motion(ClMotion),
    /// Process, dwell, tool change or other event.
    Event(Event),
}

impl ClBlock {
    pub(crate) fn check(&self) -> Result<()> {
        match self {
            Self::Motion(motion) => motion.check(),
            Self::Event(event) => event.check(),
        }
    }

    pub(crate) fn sync_id(&self) -> Option<u32> {
        match self {
            Self::Event(event) => event.sync_id(),
            Self::Motion(_) => None,
        }
    }

    pub(crate) fn tool_change_id(&self) -> Option<u32> {
        match self {
            Self::Event(event) => event.tool_change_id(),
            Self::Motion(_) => None,
        }
    }
}

/// A cutter-location path: what an operation produces before kinematics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ClPath {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    tool_id: Option<u32>,
    compensation: CompensationMode,
    feeds: Feeds,
    /// Where the process turns on, when the source operation recorded it.
    #[serde(default)]
    pierce: Option<Vec3>,
    tabs: Vec<ProfileTab>,
    blocks: Vec<ClBlock>,
    /// Continuous five-axis motion. False keeps the toolpath on one indexed tool axis.
    ///
    /// Omitted on disk when false, so a file written before this flag still loads.
    #[serde(default, skip_serializing_if = "is_false")]
    continuous: bool,
}

fn is_false(value: &bool) -> bool {
    !*value
}

impl ClPath {
    pub fn try_new(
        name: Option<String>,
        tool_id: Option<u32>,
        compensation: CompensationMode,
        feeds: Feeds,
        pierce: Option<Vec3>,
        tabs: Vec<ProfileTab>,
        blocks: Vec<ClBlock>,
    ) -> Result<Self> {
        let path = Self { name, tool_id, compensation, feeds, pierce, tabs, blocks, continuous: false };
        path.check()?;
        Ok(path)
    }

    /// Whether this toolpath moves with a changing tool axis.
    ///
    /// False is the indexed 3+2 rule: one tool axis for the whole toolpath.
    pub fn continuous(&self) -> bool {
        self.continuous
    }

    /// Set [`Self::continuous`]. The blocks are unchanged.
    pub fn with_continuous(mut self, continuous: bool) -> Self {
        self.continuous = continuous;
        self
    }

    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn tool_id(&self) -> Option<u32> {
        self.tool_id
    }

    pub fn compensation(&self) -> CompensationMode {
        self.compensation
    }

    pub fn feeds(&self) -> Feeds {
        self.feeds
    }

    pub fn pierce(&self) -> Option<Vec3> {
        self.pierce
    }

    pub fn tabs(&self) -> &[ProfileTab] {
        &self.tabs
    }

    pub fn blocks(&self) -> &[ClBlock] {
        &self.blocks
    }

    pub(crate) fn check(&self) -> Result<()> {
        check::optional_name(&self.name, "toolpath name")?;
        if self.blocks.len() > check::MAX_BLOCKS {
            return Err(check::invalid(format!("too many blocks: {} > {}", self.blocks.len(), check::MAX_BLOCKS)));
        }
        if self.tabs.len() > check::MAX_BLOCKS {
            return Err(check::invalid(format!("too many tabs: {} > {}", self.tabs.len(), check::MAX_BLOCKS)));
        }
        self.feeds.check()?;
        if let Some(pierce) = self.pierce {
            pierce.check("pierce")?;
        }
        for tab in &self.tabs {
            tab.check()?;
        }
        for block in &self.blocks {
            block.check()?;
        }
        Ok(())
    }
}

/// One joint position at the end of an axis-space move.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AxisTarget {
    axis: u8,
    /// Millimetres or radians, matching the axis kind.
    value: f64,
}

impl AxisTarget {
    pub fn try_new(axis: u8, value: f64) -> Result<Self> {
        let target = Self { axis, value };
        target.check_local()?;
        Ok(target)
    }

    pub fn axis(self) -> u8 {
        self.axis
    }

    pub fn value(self) -> f64 {
        self.value
    }

    pub(crate) fn check_local(self) -> Result<()> {
        if usize::from(self.axis) >= crate::MAX_AXES {
            return Err(check::invalid(format!("axis id {} is outside 0..{}", self.axis, crate::MAX_AXES)));
        }
        check::finite(self.value, "axis target")?;
        Ok(())
    }
}

/// Axis-space motion. Targets are joint values at the end of the move.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum AxisMotion {
    /// Joint rapid.
    Rapid {
        /// End joint values.
        targets: Vec<AxisTarget>,
        /// Optional rapid feed.
        #[serde(default)]
        feed: Option<f64>,
    },
    /// Joint linear feed.
    Linear {
        /// End joint values.
        targets: Vec<AxisTarget>,
        /// Feed.
        feed: f64,
    },
    /// Arc expressed in the machine base frame, with joint values at the end.
    Arc {
        /// End joint values.
        targets: Vec<AxisTarget>,
        /// Plane.
        plane: Plane,
        /// Arc centre in the base frame, millimetres.
        center: Vec3,
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Feed.
        feed: f64,
    },
    /// Helix in the machine base frame, with joint values at the end.
    Helix {
        /// End joint values.
        targets: Vec<AxisTarget>,
        /// Plane.
        plane: Plane,
        /// A point on the helix axis, millimetres.
        center: Vec3,
        /// Clockwise or counter-clockwise.
        sense: ArcSense,
        /// Absolute sweep in radians.
        sweep: f64,
        /// Feed.
        feed: f64,
    },
}

impl AxisMotion {
    pub(crate) fn check(&self, channel_axes: &[u8]) -> Result<()> {
        match self {
            Self::Rapid { targets, feed } => {
                check_targets(targets, channel_axes)?;
                if let Some(feed) = feed {
                    positive(*feed, "rapid feed")?;
                }
            }
            Self::Linear { targets, feed } => {
                check_targets(targets, channel_axes)?;
                positive(*feed, "feed")?;
            }
            Self::Arc { targets, center, feed, .. } => {
                check_targets(targets, channel_axes)?;
                center.check("arc centre")?;
                positive(*feed, "feed")?;
            }
            Self::Helix { targets, center, sweep, feed, .. } => {
                check_targets(targets, channel_axes)?;
                center.check("helix centre")?;
                positive(*sweep, "helix sweep")?;
                positive(*feed, "feed")?;
            }
        }
        Ok(())
    }
}

fn check_targets(targets: &[AxisTarget], channel_axes: &[u8]) -> Result<()> {
    if targets.is_empty() {
        return Err(check::invalid("axis move has no targets"));
    }
    if targets.len() > crate::MAX_AXES {
        return Err(check::invalid(format!("too many axis targets: {} > {}", targets.len(), crate::MAX_AXES)));
    }
    let mut seen = std::collections::BTreeSet::new();
    for target in targets {
        target.check_local()?;
        if !channel_axes.contains(&target.axis) {
            return Err(check::invalid(format!("axis {} is not on this channel", target.axis)));
        }
        if !seen.insert(target.axis) {
            return Err(check::invalid(format!("duplicate target for axis {}", target.axis)));
        }
    }
    Ok(())
}

/// One axis-space step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum AxisBlock {
    /// Joint move.
    Motion(AxisMotion),
    /// The same events a tool-space path can carry.
    Event(Event),
}

impl AxisBlock {
    pub(crate) fn check(&self, channel_axes: &[u8]) -> Result<()> {
        match self {
            Self::Motion(motion) => motion.check(channel_axes),
            Self::Event(event) => event.check(),
        }
    }

    pub(crate) fn sync_id(&self) -> Option<u32> {
        match self {
            Self::Event(event) => event.sync_id(),
            Self::Motion(_) => None,
        }
    }

    pub(crate) fn tool_change_id(&self) -> Option<u32> {
        match self {
            Self::Event(event) => event.tool_change_id(),
            Self::Motion(_) => None,
        }
    }
}

/// Motion for one channel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ChannelProgram {
    channel: u16,
    blocks: Vec<AxisBlock>,
}

impl ChannelProgram {
    pub fn try_new(channel: u16, blocks: Vec<AxisBlock>) -> Result<Self> {
        if blocks.len() > check::MAX_BLOCKS {
            return Err(check::invalid(format!("too many blocks: {} > {}", blocks.len(), check::MAX_BLOCKS)));
        }
        Ok(Self { channel, blocks })
    }

    pub fn channel(&self) -> u16 {
        self.channel
    }

    pub fn blocks(&self) -> &[AxisBlock] {
        &self.blocks
    }
}
