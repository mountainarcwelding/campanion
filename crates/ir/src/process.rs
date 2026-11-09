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

/// Helix sense of a rotating router cutter.
///
/// Stored on a diameter tool. Omitted, compensation stays the flat diameter offset. Sense is
/// data for the job and the verifier; it does not change the 2D offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CutterSense {
    /// Chips climb the flute toward the shank.
    Upcut,
    /// Chips are pushed toward the tip.
    Downcut,
    /// Upcut at the tip, downcut above that. Requires [`Tool`]'s `upcut_length`.
    Compression,
}

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
    /// Holder and head envelope. Absent on a tool that does not name one, so older files stay unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    assembly: Option<ToolAssembly>,
    /// Router flute helix. Omitted from JSON when absent, so older files stay unchanged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    cutter_sense: Option<CutterSense>,
    /// Upcut length at the tip, millimetres. Only with [`CutterSense::Compression`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    upcut_length: Option<f64>,
}

impl Tool {
    pub fn try_new(id: u32, name: Option<String>, diameter: Option<f64>, kerf: Option<f64>) -> Result<Self> {
        let tool = Self { id, name, diameter, kerf, assembly: None, cutter_sense: None, upcut_length: None };
        tool.check()?;
        Ok(tool)
    }

    /// Store a tool assembly and re-validate. A kerf tool cannot carry one.
    pub fn with_assembly(self, assembly: ToolAssembly) -> Result<Self> {
        let tool = Self { assembly: Some(assembly), ..self };
        tool.check()?;
        Ok(tool)
    }

    /// Store router cutter sense and re-validate. A kerf tool cannot carry one.
    ///
    /// [`CutterSense::Compression`] needs a positive finite `upcut_length` at most the flute
    /// length and stickout when an assembly is present, otherwise at most `diameter * 100`.
    /// Other senses reject `upcut_length`.
    pub fn with_cutter_sense(self, cutter_sense: Option<CutterSense>, upcut_length: Option<f64>) -> Result<Self> {
        let tool = Self { cutter_sense, upcut_length, ..self };
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

    /// Holder stack, when this tool names one.
    pub fn assembly(&self) -> Option<&ToolAssembly> {
        self.assembly.as_ref()
    }

    /// Router flute helix. `None` when the file omits `cutter_sense`.
    pub fn cutter_sense(&self) -> Option<CutterSense> {
        self.cutter_sense
    }

    /// Upcut length at the tip, millimetres. `None` unless the sense is compression.
    pub fn upcut_length(&self) -> Option<f64> {
        self.upcut_length
    }

    pub(crate) fn check(&self) -> Result<()> {
        optional_name(&self.name, "tool name")?;
        if let Some(diameter) = self.diameter {
            check_cutter(diameter, "tool diameter")?;
        }
        if let Some(kerf) = self.kerf {
            check_cutter(kerf, "kerf width")?;
        }
        if let Some(assembly) = &self.assembly {
            if self.kerf.is_some() || self.diameter.is_none() {
                return Err(check::invalid("assembly is not valid on a kerf tool"));
            }
            assembly.check()?;
        }
        check_tool_cutter_sense(self.diameter, self.kerf, self.assembly.as_ref(), self.cutter_sense, self.upcut_length)
    }
}

/// Most holder segments, including the optional head, accepted on one tool.
pub const MAX_HOLDER_SEGMENTS: usize = 32;

/// One cylinder in a holder stack. Diameter and length are millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AssemblySegment {
    diameter: f64,
    length: f64,
}

impl AssemblySegment {
    pub fn try_new(diameter: f64, length: f64) -> Result<Self> {
        let segment = Self { diameter, length };
        segment.check("holder")?;
        Ok(segment)
    }

    pub fn diameter(self) -> f64 {
        self.diameter
    }

    pub fn length(self) -> f64 {
        self.length
    }

    fn check(self, what: &str) -> Result<()> {
        check_cutter(self.diameter, &format!("{what} diameter"))?;
        check_cutter(self.length, &format!("{what} length"))?;
        Ok(())
    }
}

/// Mill tool envelope from the tip toward the spindle. Lengths are millimetres.
///
/// `flute_length` and `shank_diameter` are omitted from JSON when absent. A reader then uses
/// `stickout` for the flute and the tool diameter for the shank.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToolAssembly {
    stickout: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    flute_length: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    shank_diameter: Option<f64>,
    holder: Vec<AssemblySegment>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    head: Option<AssemblySegment>,
}

impl ToolAssembly {
    /// `holder` needs at least one segment and at most [`MAX_HOLDER_SEGMENTS`].
    pub fn try_new(
        stickout: f64,
        flute_length: Option<f64>,
        shank_diameter: Option<f64>,
        holder: Vec<AssemblySegment>,
        head: Option<AssemblySegment>,
    ) -> Result<Self> {
        let assembly = Self { stickout, flute_length, shank_diameter, holder, head };
        assembly.check()?;
        Ok(assembly)
    }

    pub fn stickout(&self) -> f64 {
        self.stickout
    }

    /// Flute length, or [`Self::stickout`] when the file omitted it.
    pub fn flute_length(&self) -> f64 {
        self.flute_length.unwrap_or(self.stickout)
    }

    /// Shank diameter, or `tool_diameter` when the file omitted it.
    pub fn shank_diameter(&self, tool_diameter: f64) -> f64 {
        self.shank_diameter.unwrap_or(tool_diameter)
    }

    pub fn holder(&self) -> &[AssemblySegment] {
        &self.holder
    }

    pub fn head(&self) -> Option<AssemblySegment> {
        self.head
    }

    fn check(&self) -> Result<()> {
        check_cutter(self.stickout, "stickout")?;
        if let Some(flute) = self.flute_length {
            check_cutter(flute, "flute_length")?;
            if flute > self.stickout {
                return Err(check::invalid("flute_length must be at most stickout"));
            }
        }
        if let Some(shank) = self.shank_diameter {
            let shank = non_negative(shank, "shank_diameter")?;
            if shank > check::MAX_CUTTER_MM {
                return Err(check::invalid(format!("shank_diameter is larger than {} mm", check::MAX_CUTTER_MM)));
            }
        }
        if self.holder.is_empty() {
            return Err(check::invalid("holder needs at least one segment"));
        }
        if self.holder.len() > MAX_HOLDER_SEGMENTS {
            return Err(check::invalid(format!("more than {MAX_HOLDER_SEGMENTS} holder segments")));
        }
        for segment in &self.holder {
            segment.check("holder")?;
        }
        if let Some(head) = self.head {
            head.check("head")?;
        }
        Ok(())
    }
}

fn check_cutter(value: f64, what: &str) -> Result<()> {
    let value = positive(value, what)?;
    if value > check::MAX_CUTTER_MM { Err(check::invalid(format!("{what} is larger than {} mm", check::MAX_CUTTER_MM))) } else { Ok(()) }
}

fn check_tool_cutter_sense(
    diameter: Option<f64>,
    kerf: Option<f64>,
    assembly: Option<&ToolAssembly>,
    cutter_sense: Option<CutterSense>,
    upcut_length: Option<f64>,
) -> Result<()> {
    if cutter_sense.is_none() && upcut_length.is_none() {
        return Ok(());
    }
    let Some(diameter) = diameter.filter(|_| kerf.is_none()) else {
        return Err(check::invalid("cutter_sense is not valid on a kerf tool"));
    };
    match cutter_sense {
        None | Some(CutterSense::Upcut) | Some(CutterSense::Downcut) => {
            if upcut_length.is_some() {
                return Err(check::invalid("upcut_length is only valid with compression"));
            }
            Ok(())
        }
        Some(CutterSense::Compression) => {
            let Some(length) = upcut_length else {
                return Err(check::invalid("compression needs upcut_length"));
            };
            let length = positive(length, "upcut_length")?;
            let (cap, what) = match assembly {
                Some(assembly) if assembly.flute_length() < assembly.stickout() => (assembly.flute_length(), "flute_length"),
                Some(assembly) => (assembly.stickout(), "stickout"),
                None => (diameter * 100.0, "diameter * 100"),
            };
            if length > cap {
                return Err(check::invalid(format!("upcut_length must be at most {what}")));
            }
            Ok(())
        }
    }
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
/// [`Event::SpindleOrient`] turns a spindle to an angle in `0..360` degrees. A secondary-spindle
/// orient is written while that spindle is not selected.
/// [`Event::ChuckClamp`], [`Event::SpindleSync`], [`Event::SpindleRun`] and [`Event::SpindleMove`]
/// hand a part between spindles. `SpindleRun` runs a spindle that is not the active binding's spindle.
/// [`Event::HeadIndex`] sets the named head's B angle. The tool axis of the moves that follow is
/// `(cos b_deg, 0, sin b_deg)`. [`Event::Tailstock`] advances or retracts the tailstock. Hold
/// force and travel are machine settings; the event carries no position.
/// [`Event::SteadyRest`] clamps or unclamps the steady rest. The rest is positioned in setup;
/// the event carries no position.
/// [`Event::PartsCatcher`] moves the parts catcher into position to catch a part, or out of the
/// work envelope. [`Event::BarFeed`] commands a bar feeder. Optional lengths are one-line
/// overrides in millimetres: part length plus cutoff and facing allowance, and minimum clamping
/// length.
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
    ///
    /// Spindle `sub` is the secondary caliper brake and does not need a C axis. Any other spindle
    /// must have one.
    SpindleBrake {
        /// Spindle name.
        spindle: String,
        /// `true` clamps the brake. `false` releases it.
        clamp: bool,
    },
    /// Orient the named spindle. `angle_deg` is finite and in `0..360`.
    ///
    /// The post writes a secondary-spindle orient while that spindle is not selected.
    SpindleOrient {
        /// Spindle name.
        spindle: String,
        /// Angle in degrees, from 0 inclusive to 360 exclusive.
        angle_deg: f64,
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
    /// Index the named head's B axis, in degrees.
    ///
    /// `b_deg` is finite. The tool axis from the tip toward the pivot is `(cos b_deg, 0, sin b_deg)`.
    HeadIndex {
        /// Head name.
        head: String,
        /// Head angle, degrees.
        b_deg: f64,
    },
    /// Advance (`engage: true`) or retract (`false`) the lathe tailstock. Hold force and
    /// travel are machine settings; the event carries no position.
    Tailstock {
        /// `true` advances the tailstock. `false` retracts it.
        engage: bool,
    },
    /// Clamp (`clamp: true`) or unclamp (`false`) the lathe steady rest. The rest is
    /// positioned in setup; the event carries no position.
    SteadyRest {
        /// `true` clamps the steady rest. `false` unclamps it.
        clamp: bool,
    },
    /// Move the parts catcher into position to catch a part (`extend: true`) or out of the
    /// work envelope (`false`).
    PartsCatcher {
        /// `true` moves the catcher in to catch a part. `false` moves it out of the work envelope.
        extend: bool,
    },
    /// Bar feed (Haas G105) with optional one-line overrides in millimetres: part length plus
    /// cutoff and facing allowance, and minimum clamping length.
    BarFeed {
        /// Part length plus cutoff and facing allowance, millimetres.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        part_length: Option<f64>,
        /// Minimum clamping length, millimetres.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        min_clamp: Option<f64>,
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
            Self::SpindleOrient { spindle, angle_deg } => {
                required_name(spindle, "spindle name")?;
                let angle = finite(*angle_deg, "orient angle")?;
                if !(0.0..360.0).contains(&angle) {
                    return Err(check::invalid("orient angle must be in 0..360"));
                }
            }
            Self::LiveToolOn { rpm, .. } => {
                let rpm = positive(*rpm, "live tool rpm")?;
                if rpm > 100_000.0 {
                    return Err(check::invalid("live tool rpm is above 100000"));
                }
            }
            Self::LiveToolOff | Self::FaceInterpolation { .. } | Self::Tailstock { .. } | Self::SteadyRest { .. } | Self::PartsCatcher { .. } => {}
            Self::BarFeed { part_length, min_clamp } => {
                if let Some(length) = part_length {
                    positive(*length, "bar feed part length")?;
                }
                if let Some(clamp) = min_clamp {
                    positive(*clamp, "bar feed minimum clamping length")?;
                }
            }
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
            Self::HeadIndex { head, b_deg } => {
                required_name(head, "head name")?;
                finite(*b_deg, "head angle")?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{Document, Program};

    fn mill(id: u32, diameter: f64) -> Tool {
        Tool::try_new(id, None, Some(diameter), None).expect("mill")
    }

    fn holder(stickout: f64, flute: Option<f64>) -> ToolAssembly {
        ToolAssembly::try_new(stickout, flute, None, vec![AssemblySegment::try_new(16.0, 10.0).expect("holder")], None).expect("assembly")
    }

    fn round_trip(tool: &Tool) -> Tool {
        let program = Program::try_new(None, vec![tool.clone()], None, Vec::new(), Vec::new(), Vec::new(), None).expect("program");
        let doc = Document::new(program).expect("doc");
        let json = doc.to_json().expect("json");
        let loaded = Document::from_json(&json).expect("load");
        loaded.program().tools()[0].clone()
    }

    #[test]
    fn omitted_cutter_sense_is_dropped_from_json() {
        let tool = mill(1, 6.0);
        let json = serde_json::to_string(&tool).expect("json");
        assert!(!json.contains("cutter_sense"), "{json}");
        assert!(!json.contains("upcut_length"), "{json}");
        let back = round_trip(&tool);
        assert_eq!(back.cutter_sense(), None);
        assert_eq!(back.upcut_length(), None);
    }

    #[test]
    fn cutter_sense_round_trips() {
        let upcut = mill(1, 6.0).with_cutter_sense(Some(CutterSense::Upcut), None).expect("upcut");
        let back = round_trip(&upcut);
        assert_eq!(back.cutter_sense(), Some(CutterSense::Upcut));
        assert_eq!(back.upcut_length(), None);
        let json = serde_json::to_string(&upcut).expect("json");
        assert!(json.contains("\"cutter_sense\":\"upcut\""), "{json}");
        assert!(!json.contains("upcut_length"), "{json}");

        let compression = mill(2, 6.0).with_cutter_sense(Some(CutterSense::Compression), Some(12.0)).expect("compression");
        let back = round_trip(&compression);
        assert_eq!(back.cutter_sense(), Some(CutterSense::Compression));
        assert_eq!(back.upcut_length(), Some(12.0));
        let json = serde_json::to_string(&compression).expect("json");
        assert!(json.contains("\"cutter_sense\":\"compression\""), "{json}");
        assert!(json.contains("\"upcut_length\":12"), "{json}");
    }

    #[test]
    fn cutter_sense_rejects_illegal_combinations() {
        let tool = mill(1, 6.0);
        assert!(tool.clone().with_cutter_sense(Some(CutterSense::Upcut), Some(1.0)).is_err());
        assert!(tool.clone().with_cutter_sense(Some(CutterSense::Downcut), Some(1.0)).is_err());
        assert!(tool.clone().with_cutter_sense(Some(CutterSense::Compression), None).is_err());
        assert!(tool.clone().with_cutter_sense(None, Some(1.0)).is_err());
        assert!(tool.clone().with_cutter_sense(Some(CutterSense::Compression), Some(0.0)).is_err());
        assert!(tool.clone().with_cutter_sense(Some(CutterSense::Compression), Some(601.0)).is_err());
        assert!(tool.with_cutter_sense(Some(CutterSense::Compression), Some(600.0)).is_ok());

        let kerf = Tool::try_new(2, None, None, Some(1.2)).expect("kerf");
        assert!(kerf.with_cutter_sense(Some(CutterSense::Upcut), None).is_err());

        let short =
            mill(3, 6.0).with_assembly(holder(20.0, Some(8.0))).expect("assembly").with_cutter_sense(Some(CutterSense::Compression), Some(9.0));
        assert!(short.is_err());
        let ok = mill(4, 6.0).with_assembly(holder(20.0, Some(8.0))).expect("assembly").with_cutter_sense(Some(CutterSense::Compression), Some(8.0));
        assert!(ok.is_ok());
    }
}
