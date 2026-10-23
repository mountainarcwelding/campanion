//! On-disk document: format tag, tools, machine, toolpaths, channel programs and sync points.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::check::{self, optional_name};
use crate::geom::{Pose, Vec3};
use crate::machine::{Channel, Machine};
use crate::motion::{ChannelProgram, ClBlock, ClMotion, ClPath};
use crate::process::{Event, Tool};
use crate::rotary::{RotarySetup, machine_to_part};
use crate::turning::TurningSetup;
use crate::{FORMAT_TAG, Result, check_format};

/// How close a pose tool axis must be to the part-frame image of machine +Z at that rotary angle.
const ROTARY_AXIS_TOL: f64 = 1e-6;

/// Slack for a turning pose. |Y| may be at most this, and X may be this far below zero.
const TURNING_PLANE_TOL: f64 = 1e-6;

/// Channels that must arrive together. A sync event waits on this id.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SyncPoint {
    id: u32,
    channels: Vec<u16>,
}

impl SyncPoint {
    pub fn try_new(id: u32, channels: Vec<u16>) -> Result<Self> {
        let sync = Self { id, channels };
        sync.check_local()?;
        Ok(sync)
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn channels(&self) -> &[u16] {
        &self.channels
    }

    pub(crate) fn check_local(&self) -> Result<()> {
        if self.channels.len() < 2 {
            return Err(check::invalid(format!("sync {} needs at least two channels", self.id)));
        }
        if self.channels.len() > crate::MAX_CHANNELS {
            return Err(check::invalid(format!("sync {} lists too many channels", self.id)));
        }
        let mut seen = BTreeSet::new();
        for channel in &self.channels {
            if !seen.insert(*channel) {
                return Err(check::invalid(format!("sync {} lists channel {channel} twice", self.id)));
            }
        }
        Ok(())
    }
}

/// Tools, machine, toolpaths and channel programs.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Program {
    #[serde(default)]
    name: Option<String>,
    tools: Vec<Tool>,
    #[serde(default)]
    machine: Option<Machine>,
    toolpaths: Vec<ClPath>,
    channel_programs: Vec<ChannelProgram>,
    syncs: Vec<SyncPoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    rotary: Option<RotarySetup>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    turning: Option<TurningSetup>,
}

impl Program {
    pub fn try_new(
        name: Option<String>,
        tools: Vec<Tool>,
        machine: Option<Machine>,
        toolpaths: Vec<ClPath>,
        channel_programs: Vec<ChannelProgram>,
        syncs: Vec<SyncPoint>,
        rotary: Option<RotarySetup>,
    ) -> Result<Self> {
        let program = Self { name, tools, machine, toolpaths, channel_programs, syncs, rotary, turning: None };
        program.check()?;
        Ok(program)
    }

    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn tools(&self) -> &[Tool] {
        &self.tools
    }

    pub fn machine(&self) -> Option<&Machine> {
        self.machine.as_ref()
    }

    pub fn toolpaths(&self) -> &[ClPath] {
        &self.toolpaths
    }

    pub fn channel_programs(&self) -> &[ChannelProgram] {
        &self.channel_programs
    }

    pub fn syncs(&self) -> &[SyncPoint] {
        &self.syncs
    }

    /// Fourth-axis table setup, when this program uses A or B.
    pub fn rotary(&self) -> Option<&RotarySetup> {
        self.rotary.as_ref()
    }

    /// Set or clear the fourth-axis setup and re-validate the program.
    pub fn with_rotary(self, rotary: Option<RotarySetup>) -> Result<Self> {
        let program = Self { rotary, ..self };
        program.check()?;
        Ok(program)
    }

    /// Lathe setup, when this program is a turning program.
    pub fn turning(&self) -> Option<&TurningSetup> {
        self.turning.as_ref()
    }

    /// Set or clear the lathe setup and re-validate the program.
    pub fn with_turning(self, turning: Option<TurningSetup>) -> Result<Self> {
        let program = Self { turning, ..self };
        program.check()?;
        Ok(program)
    }

    pub(crate) fn check(&self) -> Result<()> {
        optional_name(&self.name, "program name")?;
        check_tools(&self.tools)?;
        if let Some(machine) = &self.machine {
            machine.check()?;
        }
        if self.rotary.is_some() && self.turning.is_some() {
            return Err(check::invalid("rotary and turning setups cannot be combined yet"));
        }
        if let Some(setup) = self.rotary {
            setup.check()?;
        }
        check_rotary_poses(self.rotary.as_ref(), &self.toolpaths)?;
        if let Some(turning) = &self.turning {
            turning.check()?;
        }
        if self.toolpaths.len() > check::MAX_TOOLPATHS {
            return Err(check::invalid(format!("too many toolpaths: {} > {}", self.toolpaths.len(), check::MAX_TOOLPATHS)));
        }
        let tool_ids: BTreeSet<u32> = self.tools.iter().map(Tool::id).collect();
        for path in &self.toolpaths {
            path.check()?;
            if let Some(tool_id) = path.tool_id()
                && !tool_ids.contains(&tool_id)
            {
                return Err(check::invalid(format!("toolpath references missing tool {tool_id}")));
            }
            for block in path.blocks() {
                if let Some(tool_id) = block.tool_change_id()
                    && !tool_ids.contains(&tool_id)
                {
                    return Err(check::invalid(format!("tool change references missing tool {tool_id}")));
                }
            }
        }
        if let Some(turning) = &self.turning {
            check_turning(turning, &self.toolpaths, &tool_ids, self.machine.as_ref())?;
        }
        check_channels(&self.machine, &self.channel_programs, &tool_ids)?;
        check_syncs(&self.machine, &self.syncs, &self.channel_programs, &self.toolpaths)?;
        Ok(())
    }
}

fn check_tools(tools: &[Tool]) -> Result<()> {
    if tools.len() > check::MAX_TOOLS {
        return Err(check::invalid(format!("too many tools: {} > {}", tools.len(), check::MAX_TOOLS)));
    }
    let mut ids = BTreeSet::new();
    for tool in tools {
        tool.check()?;
        if !ids.insert(tool.id()) {
            return Err(check::invalid(format!("duplicate tool id {}", tool.id())));
        }
    }
    Ok(())
}

fn check_channels(machine: &Option<Machine>, programs: &[ChannelProgram], tool_ids: &BTreeSet<u32>) -> Result<()> {
    if programs.is_empty() {
        return Ok(());
    }
    let Some(machine) = machine else {
        return Err(check::invalid("channel program needs a machine"));
    };
    let mut seen = BTreeSet::new();
    for program in programs {
        if !seen.insert(program.channel()) {
            return Err(check::invalid(format!("duplicate program for channel {}", program.channel())));
        }
        let Some(channel) = machine.channels().iter().find(|channel| channel.id() == program.channel()) else {
            return Err(check::invalid(format!("channel program references missing channel {}", program.channel())));
        };
        if program.blocks().len() > check::MAX_BLOCKS {
            return Err(check::invalid(format!("too many blocks: {} > {}", program.blocks().len(), check::MAX_BLOCKS)));
        }
        for block in program.blocks() {
            block.check(channel.axes())?;
            if let Some(tool_id) = block.tool_change_id()
                && !tool_ids.contains(&tool_id)
            {
                return Err(check::invalid(format!("tool change references missing tool {tool_id}")));
            }
        }
    }
    Ok(())
}

fn check_syncs(machine: &Option<Machine>, syncs: &[SyncPoint], programs: &[ChannelProgram], toolpaths: &[ClPath]) -> Result<()> {
    if syncs.len() > check::MAX_SYNCS {
        return Err(check::invalid(format!("too many sync points: {} > {}", syncs.len(), check::MAX_SYNCS)));
    }
    if syncs.is_empty() && !mentions_sync(programs, toolpaths) {
        return Ok(());
    }
    let Some(machine) = machine else {
        return Err(check::invalid("sync point needs a machine"));
    };
    let channel_ids: BTreeSet<u16> = machine.channels().iter().map(Channel::id).collect();
    let mut sync_ids = BTreeSet::new();
    let mut members: BTreeMap<u32, BTreeSet<u16>> = BTreeMap::new();
    for sync in syncs {
        sync.check_local()?;
        if !sync_ids.insert(sync.id) {
            return Err(check::invalid(format!("duplicate sync id {}", sync.id)));
        }
        for channel in sync.channels() {
            if !channel_ids.contains(channel) {
                return Err(check::invalid(format!("sync {} references missing channel {channel}", sync.id)));
            }
        }
        members.insert(sync.id, sync.channels().iter().copied().collect());
    }
    let mut seen_blocks: BTreeSet<(u16, u32)> = BTreeSet::new();
    for program in programs {
        for block in program.blocks() {
            let Some(id) = block.sync_id() else {
                continue;
            };
            let Some(channels) = members.get(&id) else {
                return Err(check::invalid(format!("sync block {id} has no sync point")));
            };
            if !channels.contains(&program.channel()) {
                return Err(check::invalid(format!("channel {} is not a member of sync {id}", program.channel())));
            }
            if !seen_blocks.insert((program.channel(), id)) {
                return Err(check::invalid(format!("channel {} waits at sync {id} more than once", program.channel())));
            }
        }
    }
    for sync in syncs {
        for channel in sync.channels() {
            if !seen_blocks.contains(&(*channel, sync.id)) {
                return Err(check::invalid(format!("channel {channel} never waits at sync {}", sync.id)));
            }
        }
    }
    for path in toolpaths {
        for block in path.blocks() {
            if let Some(id) = block.sync_id()
                && !sync_ids.contains(&id)
            {
                return Err(check::invalid(format!("sync block {id} has no sync point")));
            }
        }
    }
    Ok(())
}

fn check_rotary_poses(setup: Option<&RotarySetup>, toolpaths: &[ClPath]) -> Result<()> {
    for (path_i, path) in toolpaths.iter().enumerate() {
        for (block_i, block) in path.blocks().iter().enumerate() {
            let Some(pose) = block_pose(block) else {
                continue;
            };
            let Some(angle) = pose.rotary() else {
                continue;
            };
            let Some(setup) = setup else {
                return Err(check::invalid(format!("toolpath {path_i} block {block_i} rotary angle needs a rotary setup")));
            };
            if let Some(limits) = setup.limits()
                && !limits.contains(angle)
            {
                return Err(check::invalid(format!(
                    "toolpath {path_i} block {block_i} rotary angle {angle} is outside limits {}..{}",
                    limits.min_deg(),
                    limits.max_deg()
                )));
            }
            let machine_z = Pose::from_parts(Vec3::ZERO, Vec3::Z, None);
            let expected = machine_to_part(setup, machine_z, angle).axis();
            if !vec_within(pose.axis(), expected, ROTARY_AXIS_TOL) {
                return Err(check::invalid(format!("toolpath {path_i} block {block_i} tool axis does not match rotary angle")));
            }
        }
    }
    Ok(())
}

fn check_turning(turning: &TurningSetup, toolpaths: &[ClPath], tool_ids: &BTreeSet<u32>, machine: Option<&Machine>) -> Result<()> {
    for binding in turning.bindings() {
        let index = binding.toolpath();
        if bound_path(toolpaths, index).is_none() {
            return Err(check::invalid(format!("turning binding toolpath {index} is outside 0..{}", toolpaths.len())));
        }
    }
    for insert in turning.inserts() {
        if !tool_ids.contains(&insert.tool_id()) {
            return Err(check::invalid(format!("turning insert references missing tool {}", insert.tool_id())));
        }
    }
    if let Some(machine) = machine {
        let axes: BTreeSet<u8> = machine.axes().iter().map(|axis| axis.id()).collect();
        let channels: BTreeSet<u16> = machine.channels().iter().map(|channel| channel.id()).collect();
        for spindle in turning.spindles() {
            if let Some(axis) = spindle.c_axis()
                && !axes.contains(&axis)
            {
                return Err(check::invalid(format!("turning spindle {} references missing axis {axis}", spindle.name())));
            }
        }
        for binding in turning.bindings() {
            if !channels.contains(&binding.channel()) {
                return Err(check::invalid(format!("turning binding references missing channel {}", binding.channel())));
            }
        }
    }
    for binding in turning.bindings() {
        let index = binding.toolpath();
        let Some(path) = bound_path(toolpaths, index) else {
            return Err(check::invalid(format!("turning binding toolpath {index} is outside 0..{}", toolpaths.len())));
        };
        check_turning_poses(index, path)?;
    }
    Ok(())
}

fn bound_path(toolpaths: &[ClPath], index: u32) -> Option<&ClPath> {
    usize::try_from(index).ok().and_then(|index| toolpaths.get(index))
}

fn check_turning_poses(index: u32, path: &ClPath) -> Result<()> {
    for (block_i, block) in path.blocks().iter().enumerate() {
        let Some(pose) = block_pose(block) else {
            continue;
        };
        let tip = pose.tip();
        if tip.y().abs() > TURNING_PLANE_TOL {
            return Err(check::invalid(format!("toolpath {index} block {block_i} turning pose Y is not zero")));
        }
        if tip.x() < -TURNING_PLANE_TOL {
            return Err(check::invalid(format!("toolpath {index} block {block_i} turning pose X radius is negative")));
        }
    }
    Ok(())
}

fn block_pose(block: &ClBlock) -> Option<&Pose> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. } | ClMotion::Arc { to, .. } | ClMotion::Helix { to, .. }) => Some(to),
        ClBlock::Event(Event::Probe { toward, .. }) => toward.as_ref(),
        ClBlock::Event(_) => None,
    }
}

fn vec_within(left: Vec3, right: Vec3, tol: f64) -> bool {
    let dx = left.x() - right.x();
    let dy = left.y() - right.y();
    let dz = left.z() - right.z();
    (dx * dx + dy * dy + dz * dz).sqrt() <= tol
}

fn mentions_sync(programs: &[ChannelProgram], toolpaths: &[ClPath]) -> bool {
    programs.iter().any(|program| program.blocks().iter().any(|block| block.sync_id().is_some()))
        || toolpaths.iter().any(|path| path.blocks().iter().any(|block| block.sync_id().is_some()))
}

/// A toolpath document. `format` is `campanion-ir/1` for documents this build writes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Document {
    format: String,
    program: Program,
}

impl Document {
    /// Wrap `program` with [`FORMAT_TAG`].
    pub fn new(program: Program) -> Result<Self> {
        let doc = Self { format: FORMAT_TAG.to_string(), program };
        doc.check()?;
        Ok(doc)
    }

    pub fn format(&self) -> &str {
        &self.format
    }

    pub fn program(&self) -> &Program {
        &self.program
    }

    /// Compact JSON.
    pub fn to_json(&self) -> Result<String> {
        self.check()?;
        serde_json::to_string(self).map_err(|err| crate::IrError::Json(err.to_string()))
    }

    /// Indented JSON.
    pub fn to_json_pretty(&self) -> Result<String> {
        self.check()?;
        serde_json::to_string_pretty(self).map_err(|err| crate::IrError::Json(err.to_string()))
    }

    /// Parse JSON, reject an unknown major version, then validate the document.
    pub fn from_json(text: &str) -> Result<Self> {
        if text.len() > check::MAX_JSON_BYTES {
            return Err(check::invalid(format!("json is larger than {} bytes", check::MAX_JSON_BYTES)));
        }
        let value: serde_json::Value = serde_json::from_str(text).map_err(|err| crate::IrError::Json(err.to_string()))?;
        let tag = match value.get("format") {
            Some(serde_json::Value::String(tag)) => tag.clone(),
            Some(_) => return Err(check::invalid("format tag must be a string")),
            None => return Err(check::invalid("missing format tag")),
        };
        check_format(&tag)?;
        let doc: Self = serde_json::from_value(value).map_err(|err| crate::IrError::Json(err.to_string()))?;
        doc.check()?;
        Ok(doc)
    }

    fn check(&self) -> Result<()> {
        check_format(&self.format)?;
        self.program.check()
    }
}
