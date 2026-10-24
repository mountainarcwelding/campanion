//! Tools, process parameters and non-motion events.
//!
//! Process kinds are generic. A post maps them onto a spindle, a torch, a laser or whatever the
//! machine actually has. Timing fields match `campanion-path`: pierce delay at process start and
//! post delay at process stop, each with a toggle for whether the process runs during the dwell.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, finite, non_negative, optional_name, positive, required_name, text_len};
use crate::geom::Pose;

/// A cutter or a kerf-only process.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Tool {
    id: u32,
    #[serde(default)]
    name: Option<String>,
    /// Rotating tool diameter, millimetres.
    #[serde(default)]
    diameter: Option<f64>,
    /// Kerf width, millimetres. Set for a process that has no rotating cutter.
    #[serde(default)]
    kerf: Option<f64>,
}

impl Tool {
    pub fn try_new(id: u32, name: Option<String>, diameter: Option<f64>, kerf: Option<f64>) -> Result<Self> {
        let tool = Self { id, name, diameter, kerf };
        tool.check()?;
        Ok(tool)
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn diameter(&self) -> Option<f64> {
        self.diameter
    }

    pub fn kerf(&self) -> Option<f64> {
        self.kerf
    }

    pub(crate) fn check(&self) -> Result<()> {
        optional_name(&self.name, "tool name")?;
        if let Some(diameter) = self.diameter {
            check_cutter(diameter, "tool diameter")?;
        }
        if let Some(kerf) = self.kerf {
            check_cutter(kerf, "kerf width")?;
        }
        Ok(())
    }
}

fn check_cutter(value: f64, what: &str) -> Result<()> {
    let value = positive(value, what)?;
    if value > check::MAX_CUTTER_MM { Err(check::invalid(format!("{what} is larger than {} mm", check::MAX_CUTTER_MM))) } else { Ok(()) }
}

/// What a process-on event turns on.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum ProcessKind {
    /// Rotating cutter.
    Spindle,
    /// Plasma or fuel torch.
    Torch,
    /// Laser.
    Laser,
    /// Filament or paste extruder.
    Extruder,
    /// Drag knife.
    Knife,
    /// Any later process. The string is a short name, not a machine id.
    Other(String),
}

impl ProcessKind {
    pub(crate) fn check(&self) -> Result<()> {
        if let Self::Other(name) = self {
            required_name(name, "process name")?;
        }
        Ok(())
    }
}

/// A JSON parameter: a bool, a finite number, or text.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum ParamValue {
    /// Boolean.
    Bool(bool),
    /// Finite number.
    Number(f64),
    /// Text.
    Text(String),
}

impl ParamValue {
    pub fn bool(value: bool) -> Self {
        Self::Bool(value)
    }

    pub fn number(value: f64) -> Result<Self> {
        Ok(Self::Number(finite(value, "parameter")?))
    }

    pub fn text(value: impl Into<String>) -> Result<Self> {
        let value = value.into();
        text_len(&value, check::MAX_TEXT, "parameter")?;
        Ok(Self::Text(value))
    }

    pub(crate) fn check(&self) -> Result<()> {
        match self {
            Self::Bool(_) => Ok(()),
            Self::Number(value) => {
                finite(*value, "parameter")?;
                Ok(())
            }
            Self::Text(value) => text_len(value, check::MAX_TEXT, "parameter"),
        }
    }
}

/// Parameters stored with a process-on event.
///
/// Delays are seconds. Heights are absolute Z in millimetres. `pierce_process_on_during_delay`
/// defaults to true and `post_process_on_during_delay` defaults to false, matching
/// `campanion_path::ProcessTiming`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ProcessParams {
    /// Spindle revolutions per minute, or a process-defined rate.
    #[serde(default)]
    speed: Option<f64>,
    /// Process power. The unit depends on the process (watts, a fraction, a current).
    #[serde(default)]
    power: Option<f64>,
    pierce_delay_s: f64,
    pierce_process_on_during_delay: bool,
    post_delay_s: f64,
    post_process_on_during_delay: bool,
    #[serde(default)]
    pierce_height: Option<f64>,
    #[serde(default)]
    cut_height: Option<f64>,
    extra: BTreeMap<String, ParamValue>,
}

impl ProcessParams {
    /// Zero delays. Pierce delay runs with the process on. Post delay runs after it turns off.
    pub fn new() -> Self {
        Self {
            speed: None,
            power: None,
            pierce_delay_s: 0.0,
            pierce_process_on_during_delay: true,
            post_delay_s: 0.0,
            post_process_on_during_delay: false,
            pierce_height: None,
            cut_height: None,
            extra: BTreeMap::new(),
        }
    }

    pub fn speed(mut self, speed: Option<f64>) -> Result<Self> {
        self.speed = speed;
        self.check()?;
        Ok(self)
    }

    pub fn power(mut self, power: Option<f64>) -> Result<Self> {
        self.power = power;
        self.check()?;
        Ok(self)
    }

    /// Pierce delay at each process start. `process_on_during_delay` keeps the process on while waiting.
    pub fn pierce_delay(mut self, seconds: f64, process_on_during_delay: bool) -> Result<Self> {
        self.pierce_delay_s = seconds;
        self.pierce_process_on_during_delay = process_on_during_delay;
        self.check()?;
        Ok(self)
    }

    /// Post delay at each process stop. `process_on_during_delay` keeps the process on while waiting.
    pub fn post_delay(mut self, seconds: f64, process_on_during_delay: bool) -> Result<Self> {
        self.post_delay_s = seconds;
        self.post_process_on_during_delay = process_on_during_delay;
        self.check()?;
        Ok(self)
    }

    pub fn pierce_height(mut self, z: Option<f64>) -> Result<Self> {
        self.pierce_height = z;
        self.check()?;
        Ok(self)
    }

    pub fn cut_height(mut self, z: Option<f64>) -> Result<Self> {
        self.cut_height = z;
        self.check()?;
        Ok(self)
    }

    pub fn extra(mut self, extra: BTreeMap<String, ParamValue>) -> Result<Self> {
        self.extra = extra;
        self.check()?;
        Ok(self)
    }

    pub fn speed_value(&self) -> Option<f64> {
        self.speed
    }

    pub fn power_value(&self) -> Option<f64> {
        self.power
    }

    pub fn pierce_delay_s(&self) -> f64 {
        self.pierce_delay_s
    }

    pub fn pierce_process_on_during_delay(&self) -> bool {
        self.pierce_process_on_during_delay
    }

    pub fn post_delay_s(&self) -> f64 {
        self.post_delay_s
    }

    pub fn post_process_on_during_delay(&self) -> bool {
        self.post_process_on_during_delay
    }

    pub fn pierce_height_z(&self) -> Option<f64> {
        self.pierce_height
    }

    pub fn cut_height_z(&self) -> Option<f64> {
        self.cut_height
    }

    pub fn extra_params(&self) -> &BTreeMap<String, ParamValue> {
        &self.extra
    }

    pub(crate) fn check(&self) -> Result<()> {
        if let Some(speed) = self.speed {
            non_negative(speed, "process speed")?;
        }
        if let Some(power) = self.power {
            non_negative(power, "process power")?;
        }
        non_negative(self.pierce_delay_s, "pierce delay")?;
        non_negative(self.post_delay_s, "post delay")?;
        if let Some(z) = self.pierce_height {
            finite(z, "pierce height")?;
        }
        if let Some(z) = self.cut_height {
            finite(z, "cut height")?;
        }
        check_params(&self.extra)
    }
}

impl Default for ProcessParams {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn check_params(params: &BTreeMap<String, ParamValue>) -> Result<()> {
    if params.len() > check::MAX_PARAMS {
        return Err(check::invalid(format!("too many parameters: {} > {}", params.len(), check::MAX_PARAMS)));
    }
    for (key, value) in params {
        required_name(key, "parameter name")?;
        value.check()?;
    }
    Ok(())
}

/// Direction of a live (driven) tool.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LiveDirection {
    /// Tool spins in the forward direction.
    Forward,
    /// Tool spins in the reverse direction.
    Reverse,
}

/// Coolant, air or process assist.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CoolantMode {
    /// Coolant off.
    Off,
    /// Flood coolant.
    Flood,
    /// Mist coolant.
    Mist,
    /// Air blast.
    Air,
    /// Through-tool coolant.
    ThroughTool,
    /// Gas or air assist for a laser, plasma or similar process.
    Assist,
}

/// One non-motion step.
///
/// In a turning program [`Event::CAxis`] engages a spindle as a positioning C axis and
/// [`Event::SpindleBrake`] clamps that spindle. [`Event::LiveToolOn`] starts the driven tool.
/// [`Event::FaceInterpolation`] and [`Event::CylinderInterpolation`] select a Cartesian face frame
/// or an unrolled cylinder frame; both require the C axis to be engaged when a post writes them.
/// `Pose::rotary` on a motion in that program is the C angle in degrees: finite, absolute value
/// at most 99999. A live-tool event does not require a turning setup.
/// [`Event::ChuckClamp`], [`Event::SpindleSync`], [`Event::SpindleRun`] and [`Event::SpindleMove`]
/// hand a part between spindles. `SpindleRun` runs a spindle that is not the active binding's spindle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Event {
    /// Turn a process on. `params` includes pierce and post timing.
    ProcessOn {
        /// Which process.
        process: ProcessKind,
        /// Speeds, delays and toggles.
        params: ProcessParams,
    },
    /// Turn a process off.
    ProcessOff {
        /// Which process.
        process: ProcessKind,
    },
    /// Pause, in seconds. The tool does not move.
    Dwell {
        /// Seconds. Zero is allowed.
        seconds: f64,
    },
    /// Select a tool by its id.
    ToolChange {
        /// Tool id.
        tool_id: u32,
    },
    /// Coolant or assist.
    Coolant {
        /// Mode.
        mode: CoolantMode,
    },
    /// Probe. `toward` is the expected contact pose when the path states one.
    Probe {
        /// Expected contact, when known.
        #[serde(default)]
        toward: Option<Pose>,
        /// Probe feed, millimetres per minute.
        #[serde(default)]
        feed: Option<f64>,
    },
    /// Human-readable note. Posts may emit it as a comment.
    Comment {
        /// Note text.
        text: String,
    },
    /// Named extension point. `name` is not a machine code.
    Custom {
        /// Event name.
        name: String,
        /// Named parameters.
        params: BTreeMap<String, ParamValue>,
    },
    /// Wait until every channel listed on this sync id arrives.
    Sync {
        /// Sync point id.
        id: u32,
    },
    /// Use the named spindle as a positioning C axis (`engage: true`) or return it to turning (`false`).
    CAxis {
        /// Spindle name.
        spindle: String,
        /// `true` engages the C axis. `false` returns the spindle to turning.
        engage: bool,
    },
    /// Clamp (`true`) or release (`false`) the named spindle's brake while the C axis holds position.
    SpindleBrake {
        /// Spindle name.
        spindle: String,
        /// `true` clamps the brake. `false` releases it.
        clamp: bool,
    },
    /// Start the live (driven) tool. `rpm` > 0, finite, at most 100000.
    LiveToolOn {
        /// Revolutions per minute.
        rpm: f64,
        /// Forward or reverse.
        direction: LiveDirection,
    },
    /// Stop the live tool.
    LiveToolOff,
    /// Cartesian face interpolation. While on, a pose tip (x, y, z) is a point on the part face
    /// frame: x, y Cartesian about the spindle axis, z axial. Poses carry no rotary. Arcs may use
    /// [`crate::Plane::Xy`].
    FaceInterpolation {
        /// `true` selects the face frame. `false` leaves it.
        on: bool,
    },
    /// Cylinder interpolation. While on (`radius` Some, > 0), tip.x is the radius of the tool tip,
    /// tip.y is the distance unrolled along the circumference at `radius` (C degrees = y / radius
    /// in radians), z axial. Arcs may use [`crate::Plane::Yz`]. `None` turns it off.
    CylinderInterpolation {
        /// Mapping radius in millimetres. Absent or `None` leaves cylinder interpolation.
        #[serde(default)]
        radius: Option<f64>,
    },
    /// Clamp (`true`) or unclamp (`false`) the named chuck.
    ChuckClamp {
        /// Spindle name.
        spindle: String,
        /// `true` clamps. `false` unclamps.
        clamp: bool,
    },
    /// Engage or cancel synchronous spindle control.
    ///
    /// `phase_deg` is only allowed when `engage` is true. It is finite and in `0..360`.
    SpindleSync {
        /// `true` engages sync. `false` cancels it.
        engage: bool,
        /// Phase in degrees. Absent when sync has no phase, and when sync is cancelled.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        phase_deg: Option<f64>,
    },
    /// Run or stop a spindle that is not the active binding's spindle.
    ///
    /// `rpm` 0 stops the spindle. `reverse` selects the reverse direction when `rpm` is above 0.
    SpindleRun {
        /// Spindle name.
        spindle: String,
        /// Revolutions per minute. Zero stops the spindle.
        rpm: f64,
        /// `true` runs reverse. Ignored when `rpm` is 0.
        reverse: bool,
    },
    /// Move a spindle housing along Z.
    ///
    /// `feed` absent is a rapid. `machine` selects machine coordinates.
    SpindleMove {
        /// Spindle name.
        spindle: String,
        /// Housing position along Z, millimetres.
        z: f64,
        /// `true` is a machine-coordinate move.
        machine: bool,
        /// Feed, millimetres per minute. Absent is a rapid.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        feed: Option<f64>,
    },
}

impl Event {
    pub(crate) fn check(&self) -> Result<()> {
        match self {
            Self::ProcessOn { process, params } => {
                process.check()?;
                params.check()?;
            }
            Self::ProcessOff { process } => process.check()?,
            Self::Dwell { seconds } => {
                non_negative(*seconds, "dwell")?;
            }
            Self::ToolChange { .. } | Self::Coolant { .. } | Self::Sync { .. } => {}
            Self::Probe { toward, feed } => {
                if let Some(pose) = toward {
                    pose.check()?;
                }
                if let Some(feed) = feed {
                    positive(*feed, "probe feed")?;
                }
            }
            Self::Comment { text } => text_len(text, check::MAX_TEXT, "comment")?,
            Self::Custom { name, params } => {
                required_name(name, "custom event name")?;
                check_params(params)?;
            }
            Self::CAxis { spindle, .. } | Self::SpindleBrake { spindle, .. } => {
                required_name(spindle, "spindle name")?;
            }
            Self::LiveToolOn { rpm, .. } => {
                let rpm = positive(*rpm, "live tool rpm")?;
                if rpm > 100_000.0 {
                    return Err(check::invalid("live tool rpm is above 100000"));
                }
            }
            Self::LiveToolOff | Self::FaceInterpolation { .. } => {}
            Self::CylinderInterpolation { radius } => {
                if let Some(radius) = radius {
                    positive(*radius, "cylinder interpolation radius")?;
                }
            }
            Self::ChuckClamp { spindle, .. } => {
                required_name(spindle, "spindle name")?;
            }
            Self::SpindleSync { engage, phase_deg } => {
                if let Some(phase) = phase_deg {
                    if !engage {
                        return Err(check::invalid("phase is only allowed when sync engages"));
                    }
                    let phase = finite(*phase, "phase")?;
                    if !(0.0..360.0).contains(&phase) {
                        return Err(check::invalid("phase must be in 0..360"));
                    }
                }
            }
            Self::SpindleRun { spindle, rpm, .. } => {
                required_name(spindle, "spindle name")?;
                non_negative(*rpm, "spindle rpm")?;
            }
            Self::SpindleMove { spindle, z, feed, .. } => {
                required_name(spindle, "spindle name")?;
                finite(*z, "spindle position")?;
                if let Some(feed) = feed {
                    positive(*feed, "spindle move feed")?;
                }
            }
        }
        Ok(())
    }

    pub(crate) fn sync_id(&self) -> Option<u32> {
        match self {
            Self::Sync { id } => Some(*id),
            _ => None,
        }
    }

    pub(crate) fn tool_change_id(&self) -> Option<u32> {
        match self {
            Self::ToolChange { tool_id } => Some(*tool_id),
            _ => None,
        }
    }
}
