//! Convert `campanion-path` output into tool-space IR.
//!
//! [`Document::from_profile`] and [`Document::from_toolpath`] run
//! [`campanion_path::apply_timing`] with the delays, toggles and heights on [`crate::ProcessParams`].
//! The resulting process events and dwells are copied in that order. A post does not re-derive them.
//! Pass a toolpath that does not already contain those dwells, or leave the delays at zero.

use campanion_path::{ControllerSide, CutKind, Move, Point3, ProcessEvent, ProcessTiming, Profile, Toolpath};

use crate::check::{self, non_negative};
use crate::geom::{ArcSense, Plane, Pose, Vec3};
use crate::motion::{ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, ProfileTab};
use crate::process::{Event, ProcessKind, ProcessParams, Tool};
use crate::program::{Document, Program};
use crate::{IrError, Result};

const TAU: f64 = std::f64::consts::TAU;

/// How a path operation becomes a tool-space document.
#[derive(Clone, Debug, PartialEq)]
pub struct Import {
    name: Option<String>,
    process: ProcessKind,
    params: ProcessParams,
    feeds: Feeds,
    tool_axis: Vec3,
    tool: Option<Tool>,
    compensation: CompensationMode,
}

impl Import {
    /// `cut_feed` is the cutting feed in millimetres per minute.
    ///
    /// The tool axis defaults to +Z. Pierce delay keeps the process on. Post delay runs after it turns off.
    pub fn new(process: ProcessKind, cut_feed: f64) -> Result<Self> {
        let import = Self {
            name: None,
            process,
            params: ProcessParams::new(),
            feeds: Feeds::try_new(cut_feed, None, None, None, None)?,
            tool_axis: Vec3::Z,
            tool: None,
            compensation: CompensationMode::Computed,
        };
        import.check()?;
        Ok(import)
    }

    pub fn name(mut self, name: impl Into<String>) -> Result<Self> {
        self.name = Some(name.into());
        self.check()?;
        Ok(self)
    }

    pub fn params(mut self, params: ProcessParams) -> Result<Self> {
        self.params = params;
        self.check()?;
        Ok(self)
    }

    pub fn feeds(mut self, feeds: Feeds) -> Result<Self> {
        self.feeds = feeds;
        self.check()?;
        Ok(self)
    }

    pub fn tool(mut self, tool: Tool) -> Result<Self> {
        self.tool = Some(tool);
        self.check()?;
        Ok(self)
    }

    pub fn tool_axis(mut self, axis: Vec3) -> Result<Self> {
        self.tool_axis = axis;
        self.check()?;
        Ok(self)
    }

    /// Used by [`Document::from_toolpath`]. [`Document::from_profile`] takes the side from the profile.
    pub fn compensation(mut self, compensation: CompensationMode) -> Self {
        self.compensation = compensation;
        self
    }

    pub fn process(&self) -> &ProcessKind {
        &self.process
    }

    pub fn params_ref(&self) -> &ProcessParams {
        &self.params
    }

    fn check(&self) -> Result<()> {
        self.process.check()?;
        check::optional_name(&self.name, "toolpath name")?;
        self.params.check()?;
        self.feeds.check()?;
        self.tool_axis.check_non_zero("tool axis")?;
        if let Some(tool) = &self.tool {
            tool.check()?;
        }
        Ok(())
    }
}

impl Document {
    /// Convert one profile.
    ///
    /// Compensation, the pierce point and tabs come from `profile`. Timing on `import` is applied
    /// to the profile toolpath first.
    pub fn from_profile(profile: &Profile, import: &Import) -> Result<Self> {
        import.check()?;
        let timed = apply(profile.toolpath(), &import.params)?;
        let compensation = match profile.controller_side() {
            Some(ControllerSide::Left) => CompensationMode::ControllerLeft,
            Some(ControllerSide::Right) => CompensationMode::ControllerRight,
            None => import.compensation,
        };
        let pierce = {
            let point = profile.pierce();
            Some(Vec3::new(point.x(), point.y(), point.z())?)
        };
        let mut tabs = Vec::with_capacity(profile.tabs().len());
        for tab in profile.tabs() {
            tabs.push(ProfileTab::try_new(tab.start(), tab.width(), tab.height())?);
        }
        assemble(import, &timed, compensation, pierce, tabs)
    }

    /// Convert one toolpath. Timing on `import` is applied first. Compensation comes from `import`.
    pub fn from_toolpath(path: &Toolpath, import: &Import) -> Result<Self> {
        import.check()?;
        let timed = apply(path, &import.params)?;
        assemble(import, &timed, import.compensation, None, Vec::new())
    }
}

fn apply(path: &Toolpath, params: &ProcessParams) -> Result<Toolpath> {
    let mut timing = ProcessTiming::new(params.pierce_delay_s(), params.post_delay_s())
        .pierce_process_on_during_delay(params.pierce_process_on_during_delay())
        .post_process_on_during_delay(params.post_process_on_during_delay());
    if let Some(z) = params.pierce_height_z() {
        timing = timing.pierce_height(z);
    }
    if let Some(z) = params.cut_height_z() {
        timing = timing.cut_height(z);
    }
    campanion_path::apply_timing(path, &timing).map_err(IrError::from)
}

fn assemble(import: &Import, path: &Toolpath, compensation: CompensationMode, pierce: Option<Vec3>, tabs: Vec<ProfileTab>) -> Result<Document> {
    let blocks = convert_moves(path, import)?;
    let tool_id = import.tool.as_ref().map(Tool::id);
    let cl = ClPath::try_new(import.name.clone(), tool_id, compensation, import.feeds, pierce, tabs, blocks)?;
    let mut tools = Vec::new();
    if let Some(tool) = import.tool.clone() {
        tools.push(tool);
    }
    let program = Program::try_new(import.name.clone(), tools, None, vec![cl], Vec::new(), Vec::new())?;
    Document::new(program)
}

fn convert_moves(path: &Toolpath, import: &Import) -> Result<Vec<ClBlock>> {
    let mut blocks = Vec::with_capacity(path.moves().len());
    let mut cursor: Option<Point3> = None;
    for mv in path.moves() {
        match *mv {
            Move::Rapid { to } => {
                blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose(to, import.tool_axis)?, feed: None }));
                cursor = Some(to);
            }
            Move::Linear { to, kind } => {
                let role = role_of(kind);
                blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(to, import.tool_axis)?, feed: import.feeds.resolve(role), role }));
                cursor = Some(to);
            }
            Move::Arc { to, centre, ccw, kind } => {
                let Some(start) = cursor else {
                    return Err(check::invalid("arc needs a start position"));
                };
                let role = role_of(kind);
                let sense = if ccw { ArcSense::Ccw } else { ArcSense::Cw };
                let center = Vec3::new(centre.x(), centre.y(), centre.z())?;
                let feed = import.feeds.resolve(role);
                let to_pose = pose(to, import.tool_axis)?;
                if start.z() != to.z() {
                    let sweep = xy_sweep(start, to, centre, ccw)?;
                    blocks.push(ClBlock::Motion(ClMotion::Helix { to: to_pose, plane: Plane::Xy, center, sense, sweep, feed, role }));
                } else {
                    blocks.push(ClBlock::Motion(ClMotion::Arc { to: to_pose, plane: Plane::Xy, center, sense, feed, role }));
                }
                cursor = Some(to);
            }
            Move::Event(ProcessEvent::On) => {
                blocks.push(ClBlock::Event(Event::ProcessOn { process: import.process.clone(), params: import.params.clone() }));
            }
            Move::Event(ProcessEvent::Off) => {
                blocks.push(ClBlock::Event(Event::ProcessOff { process: import.process.clone() }));
            }
            Move::Dwell { seconds } => {
                let seconds = non_negative(seconds, "dwell")?;
                blocks.push(ClBlock::Event(Event::Dwell { seconds }));
            }
        }
    }
    Ok(blocks)
}

fn pose(point: Point3, axis: Vec3) -> Result<Pose> {
    Pose::new(Vec3::new(point.x(), point.y(), point.z())?, axis)
}

fn role_of(kind: CutKind) -> MoveRole {
    match kind {
        CutKind::Cut => MoveRole::Cut,
        CutKind::LeadIn => MoveRole::LeadIn,
        CutKind::LeadOut => MoveRole::LeadOut,
        CutKind::Plunge => MoveRole::Plunge,
        CutKind::Retract => MoveRole::Retract,
    }
}

/// XY sweep used by `campanion-path` for a single arc move: at most one full turn.
/// A full turn is used when the chord is negligible, or when the requested sense is the long way.
fn xy_sweep(start: Point3, end: Point3, centre: Point3, ccw: bool) -> Result<f64> {
    let a0 = (start.y() - centre.y()).atan2(start.x() - centre.x());
    let a1 = (end.y() - centre.y()).atan2(end.x() - centre.x());
    if !a0.is_finite() || !a1.is_finite() {
        return Err(check::invalid("arc angle is not finite"));
    }
    let chord = (end.x() - start.x()).hypot(end.y() - start.y());
    if !chord.is_finite() {
        return Err(check::invalid("arc chord is not finite"));
    }
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= 1e-12 {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(check::invalid("arc sweep is not positive")) }
}
