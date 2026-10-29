//! `lathe`: load a `campanion-lathe/1` file and write turning toolpaths.
//!
//! The source path is relative to the job file. `.dxf` and `.svg` are half-profiles in the
//! drawing plane (drawing X is Z, drawing Y is the radius). `.stl` is an outside-diameter
//! envelope. `.step` and `.stp` are sectioned through the spindle axis by `solidcraft-cli`.
//! Units are millimetres. Feeds are millimetres per minute. Constant surface speed is
//! metres per minute and is capped at the spindle `max_rpm`.
//!
//! Z = 0 is the finished front face. `stock.front_allowance` is how far the blank sticks out in
//! +Z; facing cuts that down to 0. The stock record itself still occupies Z in `[-length, 0]`.
//! The optional `sub` block is a second spindle. Its profile uses that same frame (back face at
//! Z 0, part toward −Z). `back_allowance` is the facing depth on the sub. An operation's
//! `spindle` is `main` or `sub`. The stored section stays the main profile.
//! A turning operation is one toolpath: a tool change the first time that tool id is used on
//! that operation's channel, spindle on, the moves, then spindle off. When the job has
//! `head.b_axis`, a change of B writes [`Event::HeadIndex`] after that tool change (or at the
//! start of the path when the tool does not change) and the pose axis is `(cos B, 0, sin B)`.
//! A `transfer` is one
//! main-spindle toolpath on channel 0 that hands the part to the sub, optionally parting it off
//! without a spindle stop. An `eject` stops the sub and unclamps it. A `tailstock` operation
//! advances or retracts the tailstock: one event on channel 0, main spindle, no tool. While it
//! is advanced, motion tips stay outside the quill cylinder. A `steady_rest` operation clamps or
//! unclamps the steady rest the same way. While it is clamped, motion stays outside its Z band.
//! A `bar_feed` operation is the last
//! one: one event on channel 0, main spindle, no tool, written after the home lines. An `orient`
//! operation turns a spindle to an angle: one event on channel 0, bound to the main spindle, no
//! tool. The event names `main` or `sub`. A `sub_brake` operation clamps or releases the secondary
//! spindle brake the same way. A `part`
//! operation with `catch`
//! extends the parts catcher after the approach rapid and retracts it after the final retract,
//! when the part fits the job's `parts_catcher` limits. Speed is stored on the
//! turning binding, which names the operation's spindle and channel. Channel 1 uses head `lower`.
//! A `wait` operation inserts the same sync event on channel 0 and channel 1. An indexed live drill or slot
//! changes the tool the same way, engages C, starts the live tool, clamps the brake around each
//! feed, then stops the live tool and disengages C. `live_flat` engages C, rapids to the flat,
//! clamps, then starts the live tool, and mills in Y. `live_mill` engages C, rapids to clearance,
//! starts the live tool, and cuts a drawing under face or cylinder interpolation (or in Y when
//! the operation sets `y_axis`). That binding stores no spindle speed. The IR
//! stores the main finished section on the turning setup, sampled from the main profile at the
//! job chord tolerance.
//! An optional `machine` block names a Haas limit profile. The cited speeds and travels are
//! checked only when that block is present. X and Z are an extent check of main-frame tips,
//! because the work offset is not known. The profile's C-axis rpm is recorded and not checked.
//!
//! `campanion_ir::verify` runs before anything is written. Issues are printed and the process
//! exits 1. Face, rough and finish paths are checked against that spindle's finished profile for
//! insert and holder gouges. A sub operation uses the sub profile. A gouge is printed and the
//! process exits 1. Nothing is written in either case.
//! `--post` writes that dialect instead of the test-sink NGC and compares the read-back
//! at the job chord tolerance. A mismatch exits 1 and nothing is written. ZX arcs in the test
//! sink are `G18`. The test sink does not write synchronised thread moves, so a job that threads
//! needs `--post`.

use std::collections::BTreeSet;
use std::ffi::{OsStr, OsString};
use std::io::{self, Write};
use std::path::{Component, Path, PathBuf};
use std::process::ExitCode;

use campanion_geom::Tolerance;
use campanion_import::{ImportOptions, import_svg};
use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, Document, Event, Feeds, Hand, Head, HeadAxis, Insert, InsertShape, LiveDirection, MoveRole, Pose,
    ProcessKind, ProcessParams, Program, Spindle, SpindleSpeed, Tool, TurnBinding, TurnInsert, TurnSection, TurnStock, TurningSetup, Vec3,
};
use campanion_turn::{
    DrillParams, FaceParams, FinishParams, FlatPath, FlatStep, GrooveParams, HolderGeom, Infeed, InsertGeom, LiveDrillParams, LiveFace,
    LiveFlatParams, LivePath, LiveSlotParams, LiveStation, LiveStep, MAX_LIVE_POSITIONS, MeshAxis, PartParams, RoughParams, Side, ThreadParams,
    TipPoint, TurnGouge, TurnPath, TurnProfile, TurnTool, centre_drill, check_turn_gouges, face, finish, groove, live_drill, live_flat, live_slot,
    part_off, rough, thread, to_cl_path,
};
use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_bool, as_f64, as_str, as_u32, check_keys, object, optional, optional_f64, read_text, require, required_name};
use crate::jobfile::DEFAULT_JOB_CHORD_MM;
use crate::live_mill::{MillFrame, MillInterp, MillProgram, MillSpec, MillStrategy, build as build_mill, load_contours, profile_side};

const FORMAT_TAG: &str = "campanion-lathe/1";
const MAX_TOOLS: usize = 256;
const MAX_OPERATIONS: usize = 1_024;
const MAX_SOURCE_PATH: usize = 4_096;
/// [`SpindleSpeed::css`] rejects a cap above this, and the job uses the spindle cap as that limit.
/// A live-tool rpm uses the same ceiling because [`Event::LiveToolOn`] rejects a faster speed.
const MAX_SPINDLE_RPM: f64 = 100_000.0;
/// [`Tool::try_new`] rejects a cutter diameter above this.
const MAX_TOOL_DIAMETER_MM: f64 = 1.0e6;
/// Keys a live tool accepts. An insert field on the same object uses this sentence.
const LIVE_TOOL_FIELDS: &str = "id, station, live, kind, diameter, point_angle_deg, max_rpm";
const HEAD: &str = "turret1";
/// Second turret, added only when an operation uses channel 1 or a wait is present.
const LOWER: &str = "lower";
const SPINDLE: &str = "main";
const SUB_SPINDLE: &str = "sub";
/// [`TurningSetup::single`] builds a 12-station turret.
const STATIONS: u32 = 12;
/// Radial depth of a metric 60° thread, as a fraction of the pitch.
const METRIC_THREAD_DEPTH_FACTOR: f64 = 0.6134;
/// Compound infeed for a 60° metric form.
const DEFAULT_FLANK_DEG: f64 = 29.5;
/// Default first-pass depth is `min(depth, 0.35)`.
const DEFAULT_FIRST_DEPTH_CAP: f64 = 0.35;
/// Thread cycles emit rapids and synchronised moves. The path still needs a positive cut feed.
const THREAD_RECORD_FEED: f64 = 100.0;
/// Eject, tailstock, steady rest, bar feed, orient, sub brake and a transfer with no cut still need a positive recorded feed. Nothing posts it.
const EVENT_ONLY_FEED: f64 = 1.0;
/// Flank length, millimetres, used when a face, rough or finish path is checked for gouges.
const TURN_EDGE_MM: f64 = 4.0;

struct LatheArgs {
    job: PathBuf,
    output: PathBuf,
    ir: Option<PathBuf>,
    post: Option<PathBuf>,
    up_axis: crate::step::UpAxis,
}

struct LatheJob {
    source: Source,
    stock: Stock,
    front_allowance: f64,
    max_rpm: f64,
    c_axis: bool,
    /// Second spindle. Its profile is drawn in the sub frame: back face at Z 0, part toward −Z.
    sub: Option<SubBlock>,
    /// Centre or quill at the finished face. Absent when the job has no tailstock block.
    tailstock: Option<TailstockBlock>,
    /// Jaw centre on the part. Absent when the job has no steady_rest block.
    steady_rest: Option<SteadyRestBlock>,
    /// Diameter and length the parts catcher will take. Absent when the job has no parts_catcher block.
    parts_catcher: Option<PartsCatcherBlock>,
    head: LatheHead,
    tools: Vec<LatheTool>,
    operations: Vec<Operation>,
    /// Cited Haas limit profile. Absent: none of those limits are checked.
    machine: Option<crate::machine::MachineConfig>,
    linear: f64,
    chord: f64,
    clearance: f64,
}

/// Optional sub-spindle. Stock diameter and length come from the main stock.
struct SubBlock {
    max_rpm: f64,
    source: Source,
    back_allowance: f64,
}

/// Which work spindle an operation cuts on. The default in the job file is main.
#[derive(Clone, Copy, PartialEq, Eq)]
enum WorkSpindle {
    Main,
    Sub,
}

impl WorkSpindle {
    fn name(self) -> &'static str {
        match self {
            Self::Main => "main",
            Self::Sub => "sub",
        }
    }
}

/// Optional Y travel and an optional tilting B head on turret `turret1`.
struct LatheHead {
    y_axis: bool,
    y_min: Option<f64>,
    y_max: Option<f64>,
    b_axis: Option<BAxis>,
}

/// B rotation of the head in the XZ plane. `pivot_length` is millimetres from the pivot to the tip.
struct BAxis {
    min_deg: f64,
    max_deg: f64,
    pivot_length: f64,
}

struct Source {
    path: String,
    kind: SourceKind,
}

enum SourceKind {
    Dxf { layer: Option<String> },
    Svg { layer: Option<String> },
    Stl { axis: MeshAxis, step: f64 },
    Step { body: Option<String> },
}

enum Stock {
    Bar { diameter: f64, length: f64 },
    Tube { outer: f64, inner: f64, length: f64 },
}

struct LatheTool {
    id: u32,
    station: u16,
    body: ToolBody,
}

/// An insert, or a driven tool that is not mounted as an insert.
enum ToolBody {
    Insert {
        insert: Insert,
        tip: TipPoint,
        nose_radius: f64,
        holder: Option<HolderGeom>,
    },
    /// `max_rpm` limits the live-tool rpm of an operation that uses this tool.
    Live {
        orientation: LiveOrientation,
        kind: LiveKind,
        diameter: f64,
        max_rpm: Option<f64>,
    },
}

enum LiveOrientation {
    Axial,
    Radial,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LiveKind {
    Drill,
    Endmill,
}

impl ToolBody {
    fn is_live(&self) -> bool {
        matches!(self, Self::Live { .. })
    }
}

impl LatheTool {
    fn live_max_rpm(&self) -> Option<f64> {
        match &self.body {
            ToolBody::Live { max_rpm, .. } => *max_rpm,
            ToolBody::Insert { .. } => None,
        }
    }
}

enum Operation {
    Face {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        step_z: f64,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
        b_deg: Option<f64>,
    },
    Rough {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        depth_of_cut: f64,
        leave_x: f64,
        leave_z: f64,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
        b_deg: Option<f64>,
    },
    Finish {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        leave_x: f64,
        leave_z: f64,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
        b_deg: Option<f64>,
    },
    Thread {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        major_diameter: f64,
        pitch: f64,
        z_start: f64,
        z_end: f64,
        depth: f64,
        first_depth: f64,
        min_depth: f64,
        spring_passes: u8,
        infeed: Infeed,
        clearance: f64,
        speed: SpindleSpeed,
        channel: u8,
    },
    Groove {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        side: Side,
        z_left: f64,
        z_right: f64,
        bottom_diameter: f64,
        surface_diameter: Option<f64>,
        insert_width: Option<f64>,
        peck: Option<f64>,
        stepover_frac: f64,
        clearance: f64,
        dwell_s: f64,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
        b_deg: Option<f64>,
    },
    Part {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        z: f64,
        to_diameter: f64,
        chamfer: Option<f64>,
        peck: Option<f64>,
        clearance: f64,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
        b_deg: Option<f64>,
        /// Swing the parts catcher in around this part-off. Default false.
        catch: bool,
    },
    Drill {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        z_start: f64,
        depth: f64,
        diameter: f64,
        peck: Option<f64>,
        retract: f64,
        point_angle_deg: f64,
        through: bool,
        feed: f64,
        speed: SpindleSpeed,
        channel: u8,
    },
    LiveDrill {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        params: LiveDrillParams,
        channel: u8,
    },
    LiveSlot {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        params: LiveSlotParams,
        channel: u8,
    },
    LiveFlat {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        c_deg: f64,
        diameter: f64,
        across: f64,
        z_from: f64,
        z_to: f64,
        width: Option<f64>,
        step_down: Option<f64>,
        stepover: f64,
        channel: u8,
    },
    LiveMill {
        name: String,
        spindle: WorkSpindle,
        tool_id: u32,
        rpm: f64,
        feed: f64,
        face: LiveFace,
        drawing: String,
        layer: Option<String>,
        strategy: MillStrategy,
        depth: f64,
        step_down: f64,
        z_start: f64,
        diameter: f64,
        y_axis: bool,
        c_deg: f64,
        channel: u8,
    },
    /// Sync point written into both channels. Ids start at 1 in job order.
    Wait {
        name: String,
    },
    Transfer {
        name: String,
        rpm: f64,
        phase: Option<f64>,
        approach_z: f64,
        grip_z: f64,
        park_z: f64,
        feed: f64,
        dwell: f64,
        part_off: Option<PartOff>,
    },
    Eject {
        name: String,
    },
    /// Advance or retract the tailstock. No tool, channel 0, main spindle.
    Tailstock {
        name: String,
        /// `true` advances. `false` retracts.
        engage: bool,
    },
    /// Clamp or unclamp the steady rest. No tool, channel 0, main spindle.
    SteadyRest {
        name: String,
        /// `true` clamps. `false` unclamps.
        clamp: bool,
    },
    /// Bar feed at program end. No tool, channel 0, main spindle.
    BarFeed {
        name: String,
        /// Part length plus cutoff and facing allowance. Job units, stored as millimetres.
        part_length: Option<f64>,
        /// Minimum clamping length. Job units, stored as millimetres.
        min_clamp: Option<f64>,
    },
    /// Orient a spindle. No tool, channel 0, bound to the main spindle.
    ///
    /// The event names `target`. A sub orient can sit before the handoff.
    Orient {
        name: String,
        target: WorkSpindle,
        angle_deg: f64,
    },
    /// Clamp or release the secondary spindle brake. No tool, channel 0, bound to main.
    SubBrake {
        name: String,
        /// `true` clamps. `false` releases.
        clamp: bool,
    },
}

/// Centre or quill diameter at the part face, and the clearance around that cylinder.
struct TailstockBlock {
    /// Diameter of the centre or quill where it meets the finished face. Positive.
    diameter: f64,
    /// Extra gap around that cylinder, in job units. Default 1.
    clearance: f64,
}

/// Jaw centre and the axial band the turret must stay clear of.
struct SteadyRestBlock {
    /// Jaw centre in the part frame. Z 0 is the finished face. Finite and negative.
    z: f64,
    /// Jaw width along Z. Positive.
    width: f64,
    /// Extra gap on each side of the jaws, in job units. Default 1.
    clearance: f64,
}

/// Largest part the catcher will take. Both limits are job units and must be positive.
struct PartsCatcherBlock {
    /// Maximum stock outer diameter.
    max_diameter: f64,
    /// Maximum part length, `|z|` plus the part-off insert width.
    max_length: f64,
}

/// Part-off nested in a transfer. Same fields as a `part` operation, without name, type or spindle.
struct PartOff {
    tool_id: u32,
    z: f64,
    to_diameter: f64,
    chamfer: Option<f64>,
    peck: Option<f64>,
    clearance: f64,
    feed: f64,
    speed: SpindleSpeed,
}

impl Operation {
    fn name(&self) -> &str {
        match self {
            Self::Wait { name }
            | Self::Face { name, .. }
            | Self::Rough { name, .. }
            | Self::Finish { name, .. }
            | Self::Thread { name, .. }
            | Self::Groove { name, .. }
            | Self::Part { name, .. }
            | Self::Drill { name, .. }
            | Self::LiveDrill { name, .. }
            | Self::LiveSlot { name, .. }
            | Self::LiveFlat { name, .. }
            | Self::LiveMill { name, .. }
            | Self::Transfer { name, .. }
            | Self::Eject { name }
            | Self::Tailstock { name, .. }
            | Self::SteadyRest { name, .. }
            | Self::BarFeed { name, .. }
            | Self::Orient { name, .. }
            | Self::SubBrake { name, .. } => name,
        }
    }

    fn spindle(&self) -> WorkSpindle {
        match self {
            Self::Wait { .. } => WorkSpindle::Main,
            Self::Face { spindle, .. }
            | Self::Rough { spindle, .. }
            | Self::Finish { spindle, .. }
            | Self::Thread { spindle, .. }
            | Self::Groove { spindle, .. }
            | Self::Part { spindle, .. }
            | Self::Drill { spindle, .. }
            | Self::LiveDrill { spindle, .. }
            | Self::LiveSlot { spindle, .. }
            | Self::LiveFlat { spindle, .. }
            | Self::LiveMill { spindle, .. } => *spindle,
            Self::Transfer { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. } => WorkSpindle::Main,
            Self::Eject { .. } => WorkSpindle::Sub,
        }
    }

    fn tool_id(&self) -> u32 {
        match self {
            Self::Wait { .. } => 0,
            Self::Face { tool_id, .. }
            | Self::Rough { tool_id, .. }
            | Self::Finish { tool_id, .. }
            | Self::Thread { tool_id, .. }
            | Self::Groove { tool_id, .. }
            | Self::Part { tool_id, .. }
            | Self::Drill { tool_id, .. }
            | Self::LiveDrill { tool_id, .. }
            | Self::LiveSlot { tool_id, .. }
            | Self::LiveFlat { tool_id, .. }
            | Self::LiveMill { tool_id, .. } => *tool_id,
            Self::Transfer { part_off, .. } => part_off.as_ref().map(|part| part.tool_id).unwrap_or(0),
            Self::Eject { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. } => 0,
        }
    }

    /// Tool the job file must name. Eject, tailstock, steady rest, bar feed, orient, sub brake and a transfer with no part-off name none.
    fn checked_tool(&self) -> Option<u32> {
        match self {
            Self::Eject { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. } => None,
            Self::Transfer { part_off, .. } => part_off.as_ref().map(|part| part.tool_id),
            _ => Some(self.tool_id()),
        }
    }

    /// 0 or 1. A wait is written on both channels and reports 0 here.
    fn channel(&self) -> u8 {
        match self {
            Self::Wait { .. }
            | Self::Transfer { .. }
            | Self::Eject { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. } => 0,
            Self::Face { channel, .. }
            | Self::Rough { channel, .. }
            | Self::Finish { channel, .. }
            | Self::Thread { channel, .. }
            | Self::Groove { channel, .. }
            | Self::Part { channel, .. }
            | Self::Drill { channel, .. }
            | Self::LiveDrill { channel, .. }
            | Self::LiveSlot { channel, .. }
            | Self::LiveFlat { channel, .. }
            | Self::LiveMill { channel, .. } => *channel,
        }
    }

    fn is_wait(&self) -> bool {
        matches!(self, Self::Wait { .. })
    }

    fn feed(&self) -> f64 {
        match self {
            Self::Face { feed, .. }
            | Self::Rough { feed, .. }
            | Self::Finish { feed, .. }
            | Self::Groove { feed, .. }
            | Self::Part { feed, .. }
            | Self::Drill { feed, .. }
            | Self::LiveDrill { feed, .. }
            | Self::LiveSlot { feed, .. }
            | Self::LiveFlat { feed, .. }
            | Self::LiveMill { feed, .. } => *feed,
            Self::Thread { .. } => THREAD_RECORD_FEED,
            Self::Transfer { feed, part_off, .. } => part_off.as_ref().map(|part| part.feed).unwrap_or(*feed),
            Self::Eject { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. } => EVENT_ONLY_FEED,
            Self::Wait { .. } => 1.0,
        }
    }

    /// Spindle speed for a turning pass. A live cycle returns `None`.
    fn turning_speed(&self) -> Option<SpindleSpeed> {
        match self {
            Self::Face { speed, .. }
            | Self::Rough { speed, .. }
            | Self::Finish { speed, .. }
            | Self::Thread { speed, .. }
            | Self::Groove { speed, .. }
            | Self::Part { speed, .. }
            | Self::Drill { speed, .. } => Some(*speed),
            Self::LiveDrill { .. }
            | Self::LiveSlot { .. }
            | Self::LiveFlat { .. }
            | Self::LiveMill { .. }
            | Self::Eject { .. }
            | Self::Tailstock { .. }
            | Self::SteadyRest { .. }
            | Self::BarFeed { .. }
            | Self::Orient { .. }
            | Self::SubBrake { .. }
            | Self::Wait { .. } => None,
            Self::Transfer { part_off, .. } => part_off.as_ref().map(|part| part.speed),
        }
    }

    fn is_live(&self) -> bool {
        matches!(self, Self::LiveDrill { .. } | Self::LiveSlot { .. })
    }

    fn is_transfer(&self) -> bool {
        matches!(self, Self::Transfer { .. })
    }

    fn is_eject(&self) -> bool {
        matches!(self, Self::Eject { .. })
    }

    fn is_tailstock(&self) -> bool {
        matches!(self, Self::Tailstock { .. })
    }

    fn is_steady_rest(&self) -> bool {
        matches!(self, Self::SteadyRest { .. })
    }

    fn is_bar_feed(&self) -> bool {
        matches!(self, Self::BarFeed { .. })
    }

    /// `part` with `catch` true. A nested transfer part-off never sets it.
    fn catches(&self) -> bool {
        matches!(self, Self::Part { catch: true, .. })
    }

    fn is_flat(&self) -> bool {
        matches!(self, Self::LiveFlat { .. })
    }

    fn is_mill(&self) -> bool {
        matches!(self, Self::LiveMill { .. })
    }

    fn live_rpm(&self) -> Option<f64> {
        match self {
            Self::LiveDrill { rpm, .. } | Self::LiveSlot { rpm, .. } | Self::LiveFlat { rpm, .. } | Self::LiveMill { rpm, .. } => Some(*rpm),
            _ => None,
        }
    }

    /// Synchronized handoff speed. Only the transfer `rpm` field, not the nested part-off speed.
    fn transfer_rpm(&self) -> Option<f64> {
        match self {
            Self::Transfer { rpm, .. } => Some(*rpm),
            _ => None,
        }
    }

    fn uses_secondary_spindle(&self) -> bool {
        match self {
            Self::SubBrake { .. } => true,
            Self::Orient { target, .. } => *target == WorkSpindle::Sub,
            _ => self.is_transfer() || self.spindle() == WorkSpindle::Sub,
        }
    }

    fn is_orient(&self) -> bool {
        matches!(self, Self::Orient { .. })
    }

    fn is_sub_brake(&self) -> bool {
        matches!(self, Self::SubBrake { .. })
    }

    /// Head angle on a face, rough, finish, groove or part. Absent means the tool table orientation.
    fn b_deg(&self) -> Option<f64> {
        match self {
            Self::Face { b_deg, .. }
            | Self::Rough { b_deg, .. }
            | Self::Finish { b_deg, .. }
            | Self::Groove { b_deg, .. }
            | Self::Part { b_deg, .. } => *b_deg,
            _ => None,
        }
    }

    /// B this operation leaves on its channel. Operations that cannot name `b_deg` return to 0.
    fn b_target(&self) -> f64 {
        self.b_deg().unwrap_or(0.0)
    }

    fn live_face(&self) -> Option<LiveFace> {
        match self {
            Self::LiveDrill { params, .. } => Some(params.face),
            Self::LiveSlot { params, .. } => Some(params.face),
            Self::LiveFlat { .. } => Some(LiveFace::Od),
            Self::LiveMill { face, .. } => Some(*face),
            _ => None,
        }
    }
}

pub fn lathe(args: &[OsString]) -> Result<ExitCode, CliError> {
    let args = parse_lathe(args)?;
    let post = match &args.post {
        Some(spec) => Some(crate::post::load_spec(spec)?),
        None => None,
    };
    let text = read_text(&args.job)?;
    let job = parse_job(&text)?;
    let profile = load_profile(&args.job, &job, args.up_axis)?;
    let sub_profile = match &job.sub {
        Some(sub) => Some(load_source(&args.job, &sub.source, job.linear, args.up_axis)?),
        None => None,
    };
    let (doc, gouges) = build_document(&args.job, &job, &profile, sub_profile.as_ref())?;
    if !gouges.is_empty() {
        let mut err_out = io::stderr().lock();
        for line in &gouges {
            writeln!(err_out, "{line}")?;
        }
        return Ok(ExitCode::from(1));
    }
    emit(&args, &job, &doc, post.as_ref())
}

fn parse_lathe(args: &[OsString]) -> Result<LatheArgs, CliError> {
    let mut job: Option<&OsStr> = None;
    let mut output: Option<&OsStr> = None;
    let mut ir: Option<&OsStr> = None;
    let mut post: Option<&OsStr> = None;
    let mut up_axis: Option<crate::step::UpAxis> = None;
    let mut index = 0;
    while index < args.len() {
        let Some(arg) = args.get(index) else {
            break;
        };
        index += 1;
        if arg.as_os_str() == "--up-axis" {
            let Some(value) = args.get(index) else {
                return Err(CliError::usage("--up-axis needs a value"));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage("--up-axis needs a value"));
            }
            if up_axis.is_some() {
                return Err(CliError::usage("--up-axis is already set"));
            }
            let Some(text) = value.to_str() else {
                return Err(CliError::usage("--up-axis must be z, y or auto"));
            };
            up_axis = Some(crate::step::UpAxis::parse(text)?);
        } else if arg.as_os_str() == "-o" || arg.as_os_str() == "--ir" || arg.as_os_str() == "--post" {
            let flag = if arg.as_os_str() == "-o" {
                "-o"
            } else if arg.as_os_str() == "--ir" {
                "--ir"
            } else {
                "--post"
            };
            let Some(value) = args.get(index) else {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            };
            index += 1;
            if value.is_empty() {
                return Err(CliError::usage(format!("{flag} needs a {}", if flag == "--post" { "value" } else { "file" })));
            }
            let slot = if flag == "-o" {
                &mut output
            } else if flag == "--ir" {
                &mut ir
            } else {
                &mut post
            };
            if slot.is_some() {
                return Err(CliError::usage(format!("{flag} is already set")));
            }
            *slot = Some(value.as_os_str());
        } else if arg.to_string_lossy().starts_with('-') {
            return Err(CliError::usage("unknown argument"));
        } else if job.is_some() || arg.is_empty() {
            return Err(CliError::usage("lathe takes one job file"));
        } else {
            job = Some(arg.as_os_str());
        }
    }
    let Some(job) = job else {
        return Err(CliError::usage("lathe needs a job file"));
    };
    let Some(output) = output else {
        return Err(CliError::usage("lathe needs -o OUT"));
    };
    if ir.is_some_and(|path| path == output) {
        return Err(CliError::usage("-o and --ir must be different files"));
    }
    Ok(LatheArgs {
        job: PathBuf::from(job),
        output: PathBuf::from(output),
        ir: ir.map(PathBuf::from),
        post: post.map(PathBuf::from),
        up_axis: up_axis.unwrap_or(crate::step::UpAxis::Z),
    })
}

fn parse_job(text: &str) -> Result<LatheJob, CliError> {
    let value: Value = serde_json::from_str(text).map_err(|err| CliError::run(format!("json: {err}")))?;
    let map = object(&value, "job")?;
    let tag = as_str(require(map, "format")?, "format tag")?;
    check_format(tag)?;
    check_keys(
        map,
        &[
            "format",
            "source",
            "units",
            "stock",
            "spindle",
            "sub",
            "tailstock",
            "steady_rest",
            "parts_catcher",
            "head",
            "tools",
            "operations",
            "tolerance",
            "clearance",
            "machine",
        ],
        "job",
    )?;
    let units = as_str(require(map, "units")?, "units")?;
    if units != "mm" {
        return Err(CliError::invalid(format!("units must be mm, not `{units}`")));
    }
    let source = parse_source(require(map, "source")?)?;
    let (stock, front_allowance) = parse_stock(require(map, "stock")?)?;
    let (max_rpm, c_axis) = parse_spindle(require(map, "spindle")?)?;
    let sub = match optional(map, "sub") {
        Some(value) => Some(parse_sub(value)?),
        None => None,
    };
    let tailstock = match optional(map, "tailstock") {
        Some(value) => Some(parse_tailstock_block(value)?),
        None => None,
    };
    let steady_rest = match optional(map, "steady_rest") {
        Some(value) => Some(parse_steady_rest_block(value)?),
        None => None,
    };
    let parts_catcher = match optional(map, "parts_catcher") {
        Some(value) => Some(parse_parts_catcher_block(value)?),
        None => None,
    };
    if sub.is_some() && tailstock.is_some() {
        return Err(CliError::invalid("\"tailstock\" is not allowed with a \"sub\" block (the B axis is the secondary spindle)"));
    }
    if sub.is_some() && steady_rest.is_some() {
        return Err(CliError::invalid("\"steady_rest\" is not allowed with a \"sub\" block"));
    }
    let head = parse_head(optional(map, "head"))?;
    let machine = match optional(map, "machine") {
        Some(value) => Some(crate::machine::parse_machine(value)?),
        None => None,
    };
    if let Some(machine) = &machine
        && tailstock.is_some()
        && !machine.profile.tailstock_option
    {
        return Err(crate::machine::no_tailstock(machine.profile));
    }
    let tools = parse_tools(require(map, "tools")?)?;
    let (linear, chord) = parse_tolerance(optional(map, "tolerance"))?;
    let clearance = non_negative(require(map, "clearance")?, "clearance")?;
    let sub_rpm = sub.as_ref().map(|block| block.max_rpm);
    let operations = parse_operations(require(map, "operations")?, max_rpm, sub_rpm, clearance, machine.as_ref())?;
    check_bar_feed(&operations, sub.is_some())?;
    check_tailstock_ops(&operations, tailstock.is_some(), parts_catcher.is_some())?;
    check_steady_rest_ops(&operations, steady_rest.is_some())?;
    for operation in &operations {
        check_b_deg(operation, &head)?;
        if operation.is_wait() {
            continue;
        }
        let Some(tool_id) = operation.checked_tool() else {
            continue;
        };
        let Some(tool) = tools.iter().find(|tool| tool.id == tool_id) else {
            return Err(CliError::run(format!("operation {}: missing tool {tool_id}", operation.name())));
        };
        check_tool_use(operation, tool, c_axis)?;
        check_flat(operation, tool, &head, clearance)?;
        check_mill_head(operation, &head)?;
    }
    if let Some(machine) = &machine {
        check_machine_limits(machine, max_rpm, sub.as_ref(), &head, &tools, &operations)?;
    }
    Ok(LatheJob {
        source,
        stock,
        front_allowance,
        max_rpm,
        c_axis,
        sub,
        tailstock,
        steady_rest,
        parts_catcher,
        head,
        tools,
        operations,
        machine,
        linear,
        chord,
        clearance,
    })
}

fn parse_tailstock_block(value: &Value) -> Result<TailstockBlock, CliError> {
    let map = object(value, "tailstock")?;
    check_keys(map, &["diameter", "clearance"], "tailstock")?;
    let diameter = positive(require(map, "diameter")?, "diameter")?;
    let clearance = match optional(map, "clearance") {
        Some(value) => non_negative(value, "clearance")?,
        None => 1.0,
    };
    Ok(TailstockBlock { diameter, clearance })
}

fn parse_steady_rest_block(value: &Value) -> Result<SteadyRestBlock, CliError> {
    let map = object(value, "steady_rest")?;
    check_keys(map, &["z", "width", "clearance"], "steady_rest")?;
    let z = as_f64(require(map, "z")?, "z")?;
    if !(z < 0.0) {
        return Err(CliError::invalid("z must be < 0"));
    }
    let width = positive(require(map, "width")?, "width")?;
    let clearance = match optional(map, "clearance") {
        Some(value) => non_negative(value, "clearance")?,
        None => 1.0,
    };
    Ok(SteadyRestBlock { z, width, clearance })
}

fn parse_parts_catcher_block(value: &Value) -> Result<PartsCatcherBlock, CliError> {
    let map = object(value, "parts_catcher")?;
    check_keys(map, &["max_diameter", "max_length"], "parts_catcher")?;
    let max_diameter = positive(require(map, "max_diameter")?, "max_diameter")?;
    let max_length = positive(require(map, "max_length")?, "max_length")?;
    Ok(PartsCatcherBlock { max_diameter, max_length })
}

/// Advance and retract must alternate. A transfer, a sub-spindle operation or a bar feed while the
/// tailstock is advanced is rejected. Ending the program still advanced is allowed. A `part` with
/// `catch` needs the parts-catcher block, and it is rejected while the tailstock is still advanced.
fn check_tailstock_ops(operations: &[Operation], has_block: bool, has_catcher: bool) -> Result<(), CliError> {
    let mut advanced = false;
    for operation in operations {
        if let Operation::Part { name, catch: true, .. } = operation {
            if !has_catcher {
                return Err(CliError::invalid(format!("operation {name}: catch needs a \"parts_catcher\" block")));
            }
            if advanced {
                return Err(CliError::invalid(format!("operation {name}: retract the tailstock before catching the part")));
            }
        }
        if let Operation::Tailstock { name, engage } = operation {
            if !has_block {
                return Err(CliError::invalid(format!("operation {name}: tailstock needs a \"tailstock\" block")));
            }
            if *engage && advanced {
                return Err(CliError::invalid(format!("operation {name}: tailstock is already advanced")));
            }
            if !*engage && !advanced {
                return Err(CliError::invalid(format!("operation {name}: tailstock is not advanced")));
            }
            advanced = *engage;
            continue;
        }
        if advanced && (operation.is_transfer() || operation.spindle() == WorkSpindle::Sub || operation.is_bar_feed()) {
            return Err(CliError::invalid(format!("operation {}: retract the tailstock first", operation.name())));
        }
    }
    Ok(())
}

/// Clamp and unclamp must alternate. A transfer, a sub-spindle operation or a part while the
/// steady rest is clamped is rejected. Ending the program still clamped is allowed.
fn check_steady_rest_ops(operations: &[Operation], has_block: bool) -> Result<(), CliError> {
    let mut clamped = false;
    for operation in operations {
        if let Operation::SteadyRest { name, clamp } = operation {
            if !has_block {
                return Err(CliError::invalid(format!("operation {name}: steady_rest needs a \"steady_rest\" block")));
            }
            if *clamp && clamped {
                return Err(CliError::invalid(format!("operation {name}: steady rest is already clamped")));
            }
            if !*clamp && !clamped {
                return Err(CliError::invalid(format!("operation {name}: steady rest is not clamped")));
            }
            clamped = *clamp;
            continue;
        }
        if !clamped {
            continue;
        }
        if matches!(operation, Operation::Part { .. }) {
            return Err(CliError::invalid(format!("operation {}: unclamp the steady rest before parting off", operation.name())));
        }
        if operation.is_transfer() || operation.spindle() == WorkSpindle::Sub {
            return Err(CliError::invalid(format!("operation {}: unclamp the steady rest first", operation.name())));
        }
    }
    Ok(())
}

fn parse_sub(value: &Value) -> Result<SubBlock, CliError> {
    let map = object(value, "sub")?;
    check_keys(map, &["max_rpm", "source", "back_allowance"], "sub")?;
    let max_rpm = positive(require(map, "max_rpm")?, "max_rpm")?;
    if max_rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("sub max_rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    let source = parse_source(require(map, "source")?)?;
    let back_allowance = match optional(map, "back_allowance") {
        Some(value) => non_negative(value, "back_allowance")?,
        None => 0.0,
    };
    Ok(SubBlock { max_rpm, source, back_allowance })
}

fn parse_head(value: Option<&Value>) -> Result<LatheHead, CliError> {
    let Some(value) = value else {
        return Ok(LatheHead { y_axis: false, y_min: None, y_max: None, b_axis: None });
    };
    let map = object(value, "head")?;
    check_keys(map, &["y_axis", "y_min", "y_max", "b_axis"], "head")?;
    let y_axis = match optional(map, "y_axis") {
        Some(value) => as_bool(value, "y_axis")?,
        None => false,
    };
    if !y_axis && (map.contains_key("y_min") || map.contains_key("y_max")) {
        return Err(CliError::invalid("y_min and y_max need head.y_axis true"));
    }
    let (y_min, y_max) = if y_axis {
        let y_min = as_f64(require(map, "y_min")?, "y_min")?;
        let y_max = as_f64(require(map, "y_max")?, "y_max")?;
        if !(y_min < 0.0 && y_max > 0.0) {
            return Err(CliError::invalid("y_min must be < 0 and y_max must be > 0"));
        }
        (Some(y_min), Some(y_max))
    } else {
        (None, None)
    };
    let b_axis = match optional(map, "b_axis") {
        Some(value) => Some(parse_b_axis(value)?),
        None => None,
    };
    Ok(LatheHead { y_axis, y_min, y_max, b_axis })
}

fn parse_b_axis(value: &Value) -> Result<BAxis, CliError> {
    let map = object(value, "head.b_axis")?;
    check_keys(map, &["min_deg", "max_deg", "pivot_length"], "head.b_axis")?;
    let min_deg = as_f64(require(map, "min_deg")?, "min_deg")?;
    let max_deg = as_f64(require(map, "max_deg")?, "max_deg")?;
    if !(min_deg < max_deg) {
        return Err(CliError::invalid("head.b_axis: min_deg must be less than max_deg"));
    }
    let pivot_length = match optional(map, "pivot_length") {
        Some(value) => as_f64(value, "pivot_length")?,
        None => 0.0,
    };
    if pivot_length < 0.0 {
        return Err(CliError::invalid("head.b_axis: pivot_length must be zero or positive"));
    }
    Ok(BAxis { min_deg, max_deg, pivot_length })
}

fn check_b_deg(operation: &Operation, head: &LatheHead) -> Result<(), CliError> {
    let Some(b_deg) = operation.b_deg() else {
        return Ok(());
    };
    let Some(axis) = &head.b_axis else {
        return Err(CliError::invalid(format!("operation {}: b_deg needs head.b_axis", operation.name())));
    };
    if b_deg < axis.min_deg || b_deg > axis.max_deg {
        return Err(CliError::invalid(format!("operation {}: b_deg {} outside head B travel", operation.name(), format_travel(b_deg))));
    }
    Ok(())
}

fn check_format(tag: &str) -> Result<(), CliError> {
    if tag == FORMAT_TAG {
        return Ok(());
    }
    let Some(rest) = tag.strip_prefix("campanion-lathe/") else {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    };
    if rest.is_empty() || !rest.bytes().all(|byte| byte.is_ascii_digit()) || (rest.len() > 1 && rest.starts_with('0')) {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    }
    let Ok(major) = rest.parse::<u32>() else {
        return Err(CliError::invalid(format!("unrecognised format tag `{tag}`")));
    };
    if major == 1 { Ok(()) } else { Err(CliError::invalid(format!("unsupported lathe job major version {major}; this build reads major 1"))) }
}

fn parse_source(value: &Value) -> Result<Source, CliError> {
    let map = object(value, "source")?;
    let path = source_path_text(require(map, "path")?)?;
    let kind = source_kind(path)?;
    let allowed: &[&str] = match kind {
        SourceKind::Dxf { .. } | SourceKind::Svg { .. } => &["path", "layer"],
        SourceKind::Stl { .. } => &["path", "axis", "step"],
        SourceKind::Step { .. } => &["path", "body"],
    };
    check_keys(map, allowed, "source")?;
    let source_kind = match kind {
        SourceKind::Dxf { .. } => SourceKind::Dxf { layer: optional_layer(map)? },
        SourceKind::Svg { .. } => SourceKind::Svg { layer: optional_layer(map)? },
        SourceKind::Stl { .. } => {
            let axis = parse_axis(as_str(require(map, "axis")?, "axis")?)?;
            let step = positive(require(map, "step")?, "step")?;
            SourceKind::Stl { axis, step }
        }
        SourceKind::Step { .. } => SourceKind::Step { body: optional_body(map)? },
    };
    Ok(Source { path: path.to_string(), kind: source_kind })
}

fn source_kind(path: &str) -> Result<SourceKind, CliError> {
    let ext = Path::new(path).extension().and_then(|ext| ext.to_str()).unwrap_or("");
    match ext.to_ascii_lowercase().as_str() {
        "dxf" => Ok(SourceKind::Dxf { layer: None }),
        "svg" => Ok(SourceKind::Svg { layer: None }),
        "stl" => Ok(SourceKind::Stl { axis: MeshAxis::Z, step: 1.0 }),
        "step" | "stp" => Ok(SourceKind::Step { body: None }),
        _ => Err(CliError::invalid("source path must end in .dxf, .svg, .stl, .step or .stp")),
    }
}

fn optional_body(map: &Map<String, Value>) -> Result<Option<String>, CliError> {
    match optional(map, "body") {
        Some(value) => Ok(Some(required_name(as_str(value, "body")?, "body")?)),
        None => Ok(None),
    }
}

fn optional_layer(map: &Map<String, Value>) -> Result<Option<String>, CliError> {
    match optional(map, "layer") {
        Some(value) => Ok(Some(required_name(as_str(value, "layer")?, "layer")?)),
        None => Ok(None),
    }
}

fn parse_axis(text: &str) -> Result<MeshAxis, CliError> {
    match text {
        "x" => Ok(MeshAxis::X),
        "y" => Ok(MeshAxis::Y),
        "z" => Ok(MeshAxis::Z),
        other => Err(CliError::invalid(format!("unknown axis `{other}`"))),
    }
}

fn source_path_text(value: &Value) -> Result<&str, CliError> {
    relative_path_text(value, "source")
}

fn drawing_path_text(value: &Value) -> Result<&str, CliError> {
    let text = relative_path_text(value, "drawing")?;
    let ext = Path::new(text).extension().and_then(|ext| ext.to_str()).unwrap_or("");
    match ext.to_ascii_lowercase().as_str() {
        "dxf" | "svg" => Ok(text),
        _ => Err(CliError::invalid("drawing path must end in .dxf or .svg")),
    }
}

fn relative_path_text<'a>(value: &'a Value, what: &str) -> Result<&'a str, CliError> {
    let Some(text) = value.as_str() else {
        return Err(CliError::invalid(format!("{what} path must be a string")));
    };
    if text.is_empty() || text.len() > MAX_SOURCE_PATH {
        return Err(CliError::invalid(format!("{what} path must be 1 to {MAX_SOURCE_PATH} bytes")));
    }
    if text.bytes().any(|byte| byte < 0x20 || byte == 0x7f) {
        return Err(CliError::invalid(format!("{what} path has a control character")));
    }
    let path = Path::new(text);
    if path.is_absolute() {
        return Err(CliError::invalid(format!("{what} path must be relative to the job file")));
    }
    let mut named = false;
    for component in path.components() {
        match component {
            Component::Normal(_) => named = true,
            Component::CurDir | Component::ParentDir => {}
            Component::RootDir | Component::Prefix(_) => {
                return Err(CliError::invalid(format!("{what} path must be relative to the job file")));
            }
        }
    }
    if !named {
        return Err(CliError::invalid(format!("{what} path must name a file")));
    }
    Ok(text)
}

fn parse_stock(value: &Value) -> Result<(Stock, f64), CliError> {
    let map = object(value, "stock")?;
    let kind = as_str(require(map, "kind")?, "stock kind")?;
    let allowed: &[&str] = match kind {
        "bar" => &["kind", "diameter", "length", "front_allowance"],
        "tube" => &["kind", "outer_diameter", "inner_diameter", "length", "front_allowance"],
        other => return Err(CliError::invalid(format!("unknown stock kind `{other}`"))),
    };
    check_keys(map, allowed, "stock")?;
    let length = positive(require(map, "length")?, "length")?;
    let front_allowance = non_negative(require(map, "front_allowance")?, "front_allowance")?;
    let stock = match kind {
        "bar" => Stock::Bar { diameter: positive(require(map, "diameter")?, "diameter")?, length },
        "tube" => Stock::Tube {
            outer: positive(require(map, "outer_diameter")?, "outer_diameter")?,
            inner: positive(require(map, "inner_diameter")?, "inner_diameter")?,
            length,
        },
        _ => return Err(CliError::invalid(format!("unknown stock kind `{kind}`"))),
    };
    Ok((stock, front_allowance))
}

fn parse_spindle(value: &Value) -> Result<(f64, bool), CliError> {
    let map = object(value, "spindle")?;
    check_keys(map, &["max_rpm", "c_axis"], "spindle")?;
    let max_rpm = positive(require(map, "max_rpm")?, "max_rpm")?;
    if max_rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("spindle max_rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    let c_axis = match optional(map, "c_axis") {
        Some(value) => as_bool(value, "c_axis")?,
        None => false,
    };
    Ok((max_rpm, c_axis))
}

fn parse_tools(value: &Value) -> Result<Vec<LatheTool>, CliError> {
    let list = array(value, "tools", MAX_TOOLS)?;
    let mut ids = BTreeSet::new();
    let mut stations = BTreeSet::new();
    let mut tools = Vec::with_capacity(list.len());
    for item in list {
        let tool = parse_tool(item)?;
        if !ids.insert(tool.id) {
            return Err(CliError::invalid(format!("duplicate tool id {}", tool.id)));
        }
        if !stations.insert(tool.station) {
            return Err(CliError::invalid(format!("duplicate tool station {}", tool.station)));
        }
        tools.push(tool);
    }
    Ok(tools)
}

fn parse_tool(value: &Value) -> Result<LatheTool, CliError> {
    let map = object(value, "tool")?;
    if map.contains_key("live") {
        return parse_live_tool(map);
    }
    check_keys(map, &["id", "station", "shape", "nose_radius", "orientation_deg", "hand", "tip", "width", "holder"], "tool")?;
    let id = as_u32(require(map, "id")?, "tool id")?;
    let station = parse_station(map)?;
    let shape = parse_shape(as_str(require(map, "shape")?, "shape")?)?;
    let nose_radius = positive(require(map, "nose_radius")?, "nose_radius")?;
    let orientation_deg = as_f64(require(map, "orientation_deg")?, "orientation_deg")?;
    let hand = parse_hand(as_str(require(map, "hand")?, "hand")?)?;
    let tip_name = as_str(require(map, "tip")?, "tip")?;
    let width = match optional(map, "width") {
        Some(value) => Some(positive(value, "width")?),
        None => None,
    };
    let insert = Insert::try_new(shape, nose_radius, width, hand, orientation_deg).map_err(|err| CliError::run(format!("tool {id}: {err}")))?;
    let tip = programmed_tip(&insert, tip_name)?;
    let holder = match optional(map, "holder") {
        Some(value) => Some(parse_holder(value)?),
        None => None,
    };
    Ok(LatheTool { id, station, body: ToolBody::Insert { insert, tip, nose_radius, holder } })
}

fn parse_station(map: &Map<String, Value>) -> Result<u16, CliError> {
    let station_u = as_u32(require(map, "station")?, "station")?;
    if !(1..=STATIONS).contains(&station_u) {
        return Err(CliError::invalid(format!("station must be from 1 to {STATIONS}")));
    }
    u16::try_from(station_u).map_err(|_| CliError::invalid(format!("station must be from 1 to {STATIONS}")))
}

/// A tool that carries `live` is a driven drill or endmill. Insert fields are rejected, and the
/// point angle is checked for a drill then dropped: the hole depth is the tip depth.
fn parse_live_tool(map: &Map<String, Value>) -> Result<LatheTool, CliError> {
    let id = as_u32(require(map, "id")?, "tool id")?;
    const INSERT_KEYS: &[&str] = &["shape", "nose_radius", "orientation_deg", "hand", "tip", "width", "holder"];
    if INSERT_KEYS.iter().any(|key| map.contains_key(*key)) {
        return Err(CliError::invalid(format!("tool {id}: live tools take {LIVE_TOOL_FIELDS}")));
    }
    check_keys(map, &["id", "station", "live", "kind", "diameter", "point_angle_deg", "max_rpm"], "tool")?;
    let station = parse_station(map)?;
    let orientation = match as_str(require(map, "live")?, "live")? {
        "axial" => LiveOrientation::Axial,
        "radial" => LiveOrientation::Radial,
        other => return Err(CliError::invalid(format!("unknown live `{other}`"))),
    };
    let kind = match as_str(require(map, "kind")?, "kind")? {
        "drill" => LiveKind::Drill,
        "endmill" => LiveKind::Endmill,
        other => return Err(CliError::invalid(format!("unknown kind `{other}`"))),
    };
    match (kind, optional(map, "point_angle_deg")) {
        (LiveKind::Endmill, Some(_)) => {
            return Err(CliError::invalid(format!("tool {id}: point_angle_deg is only for a drill")));
        }
        (LiveKind::Drill, Some(value)) => {
            let angle = as_f64(value, "point_angle_deg")?;
            if !(angle > 0.0 && angle < 180.0) {
                return Err(CliError::invalid("point_angle_deg must be between 0 and 180"));
            }
        }
        (LiveKind::Drill, None) | (LiveKind::Endmill, None) => {}
    }
    let diameter = positive(require(map, "diameter")?, "diameter")?;
    if diameter > MAX_TOOL_DIAMETER_MM {
        return Err(CliError::invalid(format!("diameter is larger than {MAX_TOOL_DIAMETER_MM} mm")));
    }
    let max_rpm = match optional(map, "max_rpm") {
        Some(value) => {
            let rpm = positive(value, "max_rpm")?;
            if rpm > MAX_SPINDLE_RPM {
                return Err(CliError::invalid(format!("max_rpm must be at most {MAX_SPINDLE_RPM}")));
            }
            Some(rpm)
        }
        None => None,
    };
    Ok(LatheTool { id, station, body: ToolBody::Live { orientation, kind, diameter, max_rpm } })
}

fn parse_holder(value: &Value) -> Result<HolderGeom, CliError> {
    let map = object(value, "holder")?;
    check_keys(map, &["width", "length"], "holder")?;
    let width = positive(require(map, "width")?, "width")?;
    let length = positive(require(map, "length")?, "length")?;
    Ok(HolderGeom { width, length })
}

fn parse_shape(text: &str) -> Result<InsertShape, CliError> {
    match text {
        "c80" => Ok(InsertShape::C80),
        "d55" => Ok(InsertShape::D55),
        "v35" => Ok(InsertShape::V35),
        "w80" => Ok(InsertShape::W80),
        "t60" => Ok(InsertShape::T60),
        "s90" => Ok(InsertShape::S90),
        "r" => Ok(InsertShape::R),
        "groove" => Ok(InsertShape::Groove),
        "thread" => Ok(InsertShape::Thread),
        other => Err(CliError::invalid(format!("unknown insert shape `{other}`"))),
    }
}

fn parse_hand(text: &str) -> Result<Hand, CliError> {
    match text {
        "left" => Ok(Hand::Left),
        "right" => Ok(Hand::Right),
        "neutral" => Ok(Hand::Neutral),
        other => Err(CliError::invalid(format!("unknown hand `{other}`"))),
    }
}

/// Programmed point for one insert.
///
/// [`Insert::tip_offset`] is `(dx, dz)` in `(X, Z)`. [`TipPoint::ImaginaryTip`] stores that
/// same offset as `{ dz, dx }`: the imaginary corner minus the nose centre. At 135° the holder
/// bisector has positive X and negative Z, so the offset is `dx = -nose` and `dz = +nose`.
fn programmed_tip(insert: &Insert, tip: &str) -> Result<TipPoint, CliError> {
    match tip {
        "centre" => Ok(TipPoint::Centre),
        "imaginary" => {
            let (dx, dz) = insert.tip_offset();
            Ok(TipPoint::ImaginaryTip { dz, dx })
        }
        other => Err(CliError::invalid(format!("unknown tip `{other}`"))),
    }
}

fn parse_operations(
    value: &Value,
    main_rpm: f64,
    sub_rpm: Option<f64>,
    clearance: f64,
    machine: Option<&crate::machine::MachineConfig>,
) -> Result<Vec<Operation>, CliError> {
    let list = array(value, "operations", MAX_OPERATIONS)?;
    let mut names = BTreeSet::new();
    let mut operations = Vec::with_capacity(list.len());
    for item in list {
        operations.push(parse_operation(item, main_rpm, sub_rpm, clearance, &mut names)?);
    }
    let mut saw_transfer = false;
    for operation in &operations {
        if let Some(machine) = machine {
            check_operation_fits(machine, operation)?;
        }
        if operation.spindle() == WorkSpindle::Sub && !saw_transfer {
            return Err(CliError::invalid(format!("operation {}: sub-spindle op before any transfer", operation.name())));
        }
        if operation.is_transfer() {
            saw_transfer = true;
        }
    }
    Ok(operations)
}

/// A sub, transfer or eject needs the sub-spindle option. A tailstock move needs the tailstock option.
fn check_operation_fits(machine: &crate::machine::MachineConfig, operation: &Operation) -> Result<(), CliError> {
    let model = machine.profile.id;
    if !machine.sub_spindle && operation.uses_secondary_spindle() {
        return Err(CliError::invalid(format!("operation {}: {model} has no sub-spindle (machine.sub_spindle is false)", operation.name())));
    }
    if operation.is_tailstock() && !machine.tailstock {
        if !machine.profile.tailstock_option {
            return Err(crate::machine::no_tailstock(machine.profile));
        }
        return Err(CliError::invalid(format!("operation {}: machine has no tailstock (machine.tailstock is false)", operation.name())));
    }
    Ok(())
}

fn check_machine_limits(
    machine: &crate::machine::MachineConfig,
    main_rpm: f64,
    sub: Option<&SubBlock>,
    head: &LatheHead,
    tools: &[LatheTool],
    operations: &[Operation],
) -> Result<(), CliError> {
    let sub_rpm = if machine.sub_spindle { sub.map(|block| block.max_rpm) } else { None };
    let sync: Vec<(&str, f64)> = if machine.sub_spindle {
        operations.iter().filter_map(|operation| operation.transfer_rpm().map(|rpm| (operation.name(), rpm))).collect()
    } else {
        Vec::new()
    };
    let head_y = match (head.y_axis, head.y_min, head.y_max) {
        (true, Some(y_min), Some(y_max)) => Some((y_min, y_max)),
        _ => None,
    };
    let live_tools: Vec<(u32, f64)> = tools.iter().filter_map(|tool| tool.live_max_rpm().map(|rpm| (tool.id, rpm))).collect();
    let live_ops: Vec<(&str, f64)> = operations.iter().filter_map(|operation| operation.live_rpm().map(|rpm| (operation.name(), rpm))).collect();
    let programmed = crate::machine::ProgrammedLimits { main_rpm, sub_rpm, sync: &sync, head_y, live_tools: &live_tools, live_ops: &live_ops };
    crate::machine::check_programmed(machine, &programmed)
}

fn parse_work_spindle(map: &Map<String, Value>, name: &str, has_sub: bool, live: bool) -> Result<WorkSpindle, CliError> {
    let text = match optional(map, "spindle") {
        Some(value) => as_str(value, "spindle")?,
        None => "main",
    };
    match text {
        "main" => Ok(WorkSpindle::Main),
        "sub" if live => Err(CliError::invalid(format!("operation {name}: live tooling on the sub-spindle is not supported"))),
        "sub" if !has_sub => Err(CliError::invalid(format!("operation {name}: spindle sub needs a \"sub\" block"))),
        "sub" => Ok(WorkSpindle::Sub),
        other => Err(CliError::invalid(format!("operation {name}: unknown spindle `{other}`"))),
    }
}

fn speed_cap(spindle: WorkSpindle, main_rpm: f64, sub_rpm: Option<f64>) -> Result<f64, CliError> {
    match spindle {
        WorkSpindle::Main => Ok(main_rpm),
        WorkSpindle::Sub => sub_rpm.ok_or_else(|| CliError::run("internal: sub spindle is missing")),
    }
}

fn parse_operation(value: &Value, main_rpm: f64, sub_rpm: Option<f64>, clearance: f64, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    let map = object(value, "operation")?;
    let op_type = as_str(require(map, "type")?, "operation type")?;
    let has_sub = sub_rpm.is_some();
    if op_type == "wait" {
        return parse_wait(map, names);
    }
    if op_type == "live_drill" || op_type == "live_slot" {
        return parse_live_operation(map, op_type, clearance, names, has_sub);
    }
    if op_type == "live_flat" {
        return parse_live_flat(map, names, has_sub);
    }
    if op_type == "live_mill" {
        return parse_live_mill(map, names, has_sub);
    }
    if op_type == "transfer" {
        return parse_transfer(map, names, has_sub, main_rpm);
    }
    if op_type == "eject" {
        return parse_eject(map, names, has_sub);
    }
    if op_type == "tailstock" {
        return parse_tailstock(map, names);
    }
    if op_type == "steady_rest" {
        return parse_steady_rest(map, names);
    }
    if op_type == "bar_feed" {
        return parse_bar_feed(map, names);
    }
    if op_type == "orient" {
        return parse_orient(map, names, has_sub);
    }
    if op_type == "sub_brake" {
        return parse_sub_brake(map, names, has_sub);
    }
    let allowed: &[&str] = match op_type {
        "face" => &["name", "type", "tool_id", "spindle", "channel", "step_z", "feed", "speed", "b_deg"],
        "rough" => &["name", "type", "tool_id", "spindle", "channel", "side", "depth_of_cut", "leave_x", "leave_z", "feed", "speed", "b_deg"],
        "finish" => &["name", "type", "tool_id", "spindle", "channel", "side", "feed", "speed", "leave_x", "leave_z", "b_deg"],
        "thread" => &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "channel",
            "side",
            "major_diameter",
            "pitch",
            "z_start",
            "z_end",
            "depth",
            "first_depth",
            "min_depth",
            "spring_passes",
            "infeed",
            "clearance",
            "rpm",
        ],
        "groove" => &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "channel",
            "side",
            "z_left",
            "z_right",
            "bottom_diameter",
            "surface_diameter",
            "insert_width",
            "peck",
            "stepover_frac",
            "clearance",
            "dwell_s",
            "feed",
            "speed",
            "b_deg",
        ],
        "part" => {
            &["name", "type", "tool_id", "spindle", "channel", "z", "to_diameter", "chamfer", "peck", "clearance", "feed", "speed", "b_deg", "catch"]
        }
        "drill" => &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "channel",
            "z_start",
            "depth",
            "diameter",
            "peck",
            "retract",
            "point_angle_deg",
            "through",
            "feed",
            "rpm",
        ],
        other => return Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, false)?;
    let channel = parse_channel(map, &name)?;
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
    let cap = speed_cap(spindle, main_rpm, sub_rpm)?;
    match op_type {
        "face" | "rough" | "finish" | "groove" | "part" => {
            let feed = positive(require(map, "feed")?, "feed")?;
            let speed = parse_speed(require(map, "speed")?, cap)?;
            let b_deg = optional_f64(map, "b_deg")?;
            match op_type {
                "face" => {
                    Ok(Operation::Face { name, spindle, tool_id, step_z: positive(require(map, "step_z")?, "step_z")?, feed, speed, channel, b_deg })
                }
                "rough" => Ok(Operation::Rough {
                    name,
                    spindle,
                    tool_id,
                    side: parse_side(as_str(require(map, "side")?, "side")?)?,
                    depth_of_cut: positive(require(map, "depth_of_cut")?, "depth_of_cut")?,
                    leave_x: non_negative(require(map, "leave_x")?, "leave_x")?,
                    leave_z: non_negative(require(map, "leave_z")?, "leave_z")?,
                    feed,
                    speed,
                    channel,
                    b_deg,
                }),
                "finish" => Ok(Operation::Finish {
                    name,
                    spindle,
                    tool_id,
                    side: parse_side(as_str(require(map, "side")?, "side")?)?,
                    leave_x: optional_non_negative(map, "leave_x")?.unwrap_or(0.0),
                    leave_z: optional_non_negative(map, "leave_z")?.unwrap_or(0.0),
                    feed,
                    speed,
                    channel,
                    b_deg,
                }),
                "groove" => parse_groove(map, name, spindle, tool_id, feed, speed, channel, b_deg),
                "part" => parse_part(map, name, spindle, tool_id, feed, speed, channel, b_deg),
                _ => Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
            }
        }
        "thread" => parse_thread(map, name, spindle, tool_id, channel),
        "drill" => parse_drill(map, name, spindle, tool_id, channel),
        other => Err(CliError::invalid(format!("unknown operation type `{other}`"))),
    }
}

fn parse_channel(map: &Map<String, Value>, name: &str) -> Result<u8, CliError> {
    let Some(value) = optional(map, "channel") else {
        return Ok(0);
    };
    let number = as_f64(value, "channel")?;
    if number == 0.0 {
        return Ok(0);
    }
    if number == 1.0 {
        return Ok(1);
    }
    Err(CliError::invalid(format!("operation {name}: channel must be 0 or 1")))
}

fn parse_wait(map: &Map<String, Value>, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    Ok(Operation::Wait { name })
}

fn parse_thread(map: &Map<String, Value>, name: String, spindle: WorkSpindle, tool_id: u32, channel: u8) -> Result<Operation, CliError> {
    let pitch = positive(require(map, "pitch")?, "pitch")?;
    let depth = match optional(map, "depth") {
        Some(value) => positive(value, "depth")?,
        None => METRIC_THREAD_DEPTH_FACTOR * pitch,
    };
    let first_depth = match optional(map, "first_depth") {
        Some(value) => positive(value, "first_depth")?,
        None => depth.min(DEFAULT_FIRST_DEPTH_CAP),
    };
    let min_depth = match optional(map, "min_depth") {
        Some(value) => positive(value, "min_depth")?,
        None => 0.05,
    };
    Ok(Operation::Thread {
        name,
        spindle,
        tool_id,
        side: parse_side(as_str(require(map, "side")?, "side")?)?,
        major_diameter: positive(require(map, "major_diameter")?, "major_diameter")?,
        pitch,
        z_start: as_f64(require(map, "z_start")?, "z_start")?,
        z_end: as_f64(require(map, "z_end")?, "z_end")?,
        depth,
        first_depth,
        min_depth,
        spring_passes: optional_u8(map, "spring_passes")?.unwrap_or(1),
        infeed: match optional(map, "infeed") {
            Some(value) => parse_infeed(as_str(value, "infeed")?)?,
            None => Infeed::Flank { angle_deg: DEFAULT_FLANK_DEG },
        },
        clearance: match optional(map, "clearance") {
            Some(value) => non_negative(value, "clearance")?,
            None => 1.0,
        },
        speed: SpindleSpeed::rpm(positive(require(map, "rpm")?, "rpm")?)?,
        channel,
    })
}

fn parse_groove(
    map: &Map<String, Value>,
    name: String,
    spindle: WorkSpindle,
    tool_id: u32,
    feed: f64,
    speed: SpindleSpeed,
    channel: u8,
    b_deg: Option<f64>,
) -> Result<Operation, CliError> {
    let stepover_frac = match optional(map, "stepover_frac") {
        Some(value) => as_f64(value, "stepover_frac")?,
        None => 0.8,
    };
    if !(stepover_frac > 0.0 && stepover_frac <= 1.0) {
        return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
    }
    Ok(Operation::Groove {
        name,
        spindle,
        tool_id,
        side: parse_side(as_str(require(map, "side")?, "side")?)?,
        z_left: as_f64(require(map, "z_left")?, "z_left")?,
        z_right: as_f64(require(map, "z_right")?, "z_right")?,
        bottom_diameter: positive(require(map, "bottom_diameter")?, "bottom_diameter")?,
        surface_diameter: optional_positive(map, "surface_diameter")?,
        insert_width: optional_positive(map, "insert_width")?,
        peck: optional_positive(map, "peck")?,
        stepover_frac,
        clearance: match optional(map, "clearance") {
            Some(value) => non_negative(value, "clearance")?,
            None => 1.0,
        },
        dwell_s: optional_non_negative(map, "dwell_s")?.unwrap_or(0.0),
        feed,
        speed,
        channel,
        b_deg,
    })
}

fn parse_part(
    map: &Map<String, Value>,
    name: String,
    spindle: WorkSpindle,
    tool_id: u32,
    feed: f64,
    speed: SpindleSpeed,
    channel: u8,
    b_deg: Option<f64>,
) -> Result<Operation, CliError> {
    Ok(Operation::Part {
        name,
        spindle,
        tool_id,
        z: as_f64(require(map, "z")?, "z")?,
        to_diameter: match optional(map, "to_diameter") {
            Some(value) => as_f64(value, "to_diameter")?,
            None => -0.5,
        },
        chamfer: optional_positive(map, "chamfer")?,
        peck: optional_positive(map, "peck")?,
        clearance: non_negative(require(map, "clearance")?, "clearance")?,
        feed,
        speed,
        channel,
        b_deg,
        catch: match optional(map, "catch") {
            Some(value) => as_bool(value, "catch")?,
            None => false,
        },
    })
}

fn parse_transfer(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool, main_rpm: f64) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "rpm", "phase", "approach_z", "grip_z", "park_z", "feed", "dwell", "part_off"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    if !has_sub {
        return Err(CliError::invalid(format!("operation {name}: transfer needs a \"sub\" block")));
    }
    let rpm = match optional(map, "rpm") {
        Some(value) => non_negative(value, "rpm")?,
        None => 0.0,
    };
    let phase = match optional(map, "phase") {
        Some(value) => {
            let phase = as_f64(value, "phase")?;
            if !(0.0..360.0).contains(&phase) {
                return Err(CliError::invalid("phase must be in 0..360"));
            }
            Some(phase)
        }
        None => None,
    };
    let part_off = match optional(map, "part_off") {
        Some(value) => Some(parse_part_off(value, main_rpm)?),
        None => None,
    };
    Ok(Operation::Transfer {
        name,
        rpm,
        phase,
        approach_z: as_f64(require(map, "approach_z")?, "approach_z")?,
        grip_z: as_f64(require(map, "grip_z")?, "grip_z")?,
        park_z: as_f64(require(map, "park_z")?, "park_z")?,
        feed: positive(require(map, "feed")?, "feed")?,
        dwell: match optional(map, "dwell") {
            Some(value) => non_negative(value, "dwell")?,
            None => 0.5,
        },
        part_off,
    })
}

fn parse_part_off(value: &Value, main_rpm: f64) -> Result<PartOff, CliError> {
    let map = object(value, "part_off")?;
    check_keys(map, &["tool_id", "z", "to_diameter", "chamfer", "peck", "clearance", "feed", "speed"], "part_off")?;
    Ok(PartOff {
        tool_id: as_u32(require(map, "tool_id")?, "tool_id")?,
        z: as_f64(require(map, "z")?, "z")?,
        to_diameter: match optional(map, "to_diameter") {
            Some(value) => as_f64(value, "to_diameter")?,
            None => -0.5,
        },
        chamfer: optional_positive(map, "chamfer")?,
        peck: optional_positive(map, "peck")?,
        clearance: non_negative(require(map, "clearance")?, "clearance")?,
        feed: positive(require(map, "feed")?, "feed")?,
        speed: parse_speed(require(map, "speed")?, main_rpm)?,
    })
}

fn parse_eject(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    if !has_sub {
        return Err(CliError::invalid(format!("operation {name}: eject needs a \"sub\" block")));
    }
    Ok(Operation::Eject { name })
}

fn parse_tailstock(map: &Map<String, Value>, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "action"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let action = as_str(require(map, "action")?, "action")?;
    let engage = match action {
        "advance" => true,
        "retract" => false,
        _ => return Err(CliError::invalid(format!("operation {name}: tailstock action must be advance or retract"))),
    };
    Ok(Operation::Tailstock { name, engage })
}

fn parse_steady_rest(map: &Map<String, Value>, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "action"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let action = as_str(require(map, "action")?, "action")?;
    let clamp = match action {
        "clamp" => true,
        "unclamp" => false,
        _ => return Err(CliError::invalid(format!("operation {name}: steady_rest action must be clamp or unclamp"))),
    };
    Ok(Operation::SteadyRest { name, clamp })
}

fn parse_bar_feed(map: &Map<String, Value>, names: &mut BTreeSet<String>) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "part_length", "min_clamp"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    // Lathe jobs are millimetres, so the file number is already the IR length.
    let part_length = optional_positive(map, "part_length")?;
    let min_clamp = optional_positive(map, "min_clamp")?;
    Ok(Operation::BarFeed { name, part_length, min_clamp })
}

fn parse_orient(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "spindle", "angle_deg"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = as_str(require(map, "spindle")?, "spindle")?;
    let target = match spindle {
        "main" => WorkSpindle::Main,
        "sub" if !has_sub => return Err(CliError::invalid(format!("operation {name}: spindle sub needs a \"sub\" block"))),
        "sub" => WorkSpindle::Sub,
        other => return Err(CliError::invalid(format!("operation {name}: unknown spindle `{other}`"))),
    };
    let angle_deg = as_f64(require(map, "angle_deg")?, "angle_deg")?;
    if !(0.0..360.0).contains(&angle_deg) {
        return Err(CliError::invalid(format!("operation {name}: orient angle must be in 0..360")));
    }
    Ok(Operation::Orient { name, target, angle_deg })
}

fn parse_sub_brake(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    check_keys(map, &["name", "type", "action"], "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    if !has_sub {
        return Err(CliError::invalid(format!("operation {name}: sub_brake needs a \"sub\" block")));
    }
    let action = as_str(require(map, "action")?, "action")?;
    let clamp = match action {
        "clamp" => true,
        "release" => false,
        _ => return Err(CliError::invalid(format!("operation {name}: sub_brake action must be clamp or release"))),
    };
    Ok(Operation::SubBrake { name, clamp })
}

/// A bar feed is the last operation, after a `part`, and not on a job that has a sub spindle.
fn check_bar_feed(operations: &[Operation], has_sub: bool) -> Result<(), CliError> {
    let Some(index) = operations.iter().position(|operation| operation.is_bar_feed()) else {
        return Ok(());
    };
    let name = operations[index].name();
    if index + 1 != operations.len() {
        return Err(CliError::invalid(format!("operation {name}: bar_feed must be the last operation")));
    }
    let has_part = operations[..index].iter().any(|operation| matches!(operation, Operation::Part { .. }));
    if !has_part {
        return Err(CliError::invalid(format!("operation {name}: bar_feed needs a part operation before it")));
    }
    if has_sub {
        return Err(CliError::invalid(format!("operation {name}: bar_feed is not supported with a \"sub\" block")));
    }
    Ok(())
}

fn parse_drill(map: &Map<String, Value>, name: String, spindle: WorkSpindle, tool_id: u32, channel: u8) -> Result<Operation, CliError> {
    let point_angle_deg = match optional(map, "point_angle_deg") {
        Some(value) => as_f64(value, "point_angle_deg")?,
        None => 118.0,
    };
    Ok(Operation::Drill {
        name,
        spindle,
        tool_id,
        z_start: match optional(map, "z_start") {
            Some(value) => as_f64(value, "z_start")?,
            None => 0.0,
        },
        depth: positive(require(map, "depth")?, "depth")?,
        diameter: positive(require(map, "diameter")?, "diameter")?,
        peck: optional_positive(map, "peck")?,
        retract: match optional(map, "retract") {
            Some(value) => non_negative(value, "retract")?,
            None => 1.0,
        },
        point_angle_deg,
        through: match optional(map, "through") {
            Some(value) => as_bool(value, "through")?,
            None => false,
        },
        feed: positive(require(map, "feed")?, "feed")?,
        speed: SpindleSpeed::rpm(positive(require(map, "rpm")?, "rpm")?)?,
        channel,
    })
}

fn parse_live_operation(
    map: &Map<String, Value>,
    op_type: &str,
    clearance: f64,
    names: &mut BTreeSet<String>,
    has_sub: bool,
) -> Result<Operation, CliError> {
    let face = parse_live_face(as_str(require(map, "face")?, "face")?)?;
    let allowed: &[&str] = match (op_type, face) {
        ("live_drill", LiveFace::Front) => {
            &["name", "type", "tool_id", "spindle", "channel", "face", "positions", "depth", "peck", "retract", "rpm", "feed", "z_start"]
        }
        ("live_drill", LiveFace::Od) => {
            &["name", "type", "tool_id", "spindle", "channel", "face", "positions", "depth", "peck", "retract", "rpm", "feed", "diameter"]
        }
        ("live_slot", LiveFace::Front) => {
            &["name", "type", "tool_id", "spindle", "channel", "face", "c", "from", "to", "depth", "step_down", "rpm", "feed", "z_start"]
        }
        ("live_slot", LiveFace::Od) => {
            &["name", "type", "tool_id", "spindle", "channel", "face", "c", "from", "to", "depth", "step_down", "rpm", "feed", "diameter"]
        }
        _ => return Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
    };
    check_keys(map, allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, true)?;
    let channel = parse_channel(map, &name)?;
    let tool_id = as_u32(require(map, "tool_id")?, "tool_id")?;
    let depth = positive(require(map, "depth")?, "depth")?;
    let rpm = parse_live_rpm(require(map, "rpm")?)?;
    let feed = positive(require(map, "feed")?, "feed")?;
    let (z_start, diameter) = match face {
        LiveFace::Front => (as_f64(require(map, "z_start")?, "z_start")?, 0.0),
        LiveFace::Od => (0.0, positive(require(map, "diameter")?, "diameter")?),
    };
    match op_type {
        "live_drill" => {
            let retract = match optional(map, "retract") {
                Some(value) => non_negative(value, "retract")?,
                None => clearance,
            };
            let params = LiveDrillParams {
                face,
                positions: parse_live_positions(require(map, "positions")?, face)?,
                depth,
                peck: optional_positive(map, "peck")?,
                retract,
                z_start,
                diameter,
            };
            live_drill(&params).map_err(|err| geometry_err(&name, err))?;
            Ok(Operation::LiveDrill { name, spindle, tool_id, rpm, feed, params, channel })
        }
        "live_slot" => {
            let step_down = match optional(map, "step_down") {
                Some(value) => positive(value, "step_down")?,
                None => depth,
            };
            let params = LiveSlotParams {
                face,
                c_deg: as_f64(require(map, "c")?, "c")?,
                from: as_f64(require(map, "from")?, "from")?,
                to: as_f64(require(map, "to")?, "to")?,
                depth,
                step_down,
                retract: clearance,
                z_start,
                diameter,
            };
            live_slot(&params).map_err(|err| geometry_err(&name, err))?;
            Ok(Operation::LiveSlot { name, spindle, tool_id, rpm, feed, params, channel })
        }
        _ => Err(CliError::invalid(format!("unknown operation type `{op_type}`"))),
    }
}

fn parse_live_flat(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    check_keys(
        map,
        &[
            "name",
            "type",
            "tool_id",
            "spindle",
            "channel",
            "c",
            "diameter",
            "across",
            "z_from",
            "z_to",
            "width",
            "step_down",
            "stepover",
            "rpm",
            "feed",
        ],
        "operation",
    )?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, true)?;
    let channel = parse_channel(map, &name)?;
    let stepover = match optional(map, "stepover") {
        Some(value) => as_f64(value, "stepover")?,
        None => 0.6,
    };
    if !(stepover > 0.0 && stepover <= 1.0) {
        return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
    }
    Ok(Operation::LiveFlat {
        name,
        spindle,
        tool_id: as_u32(require(map, "tool_id")?, "tool_id")?,
        rpm: parse_live_rpm(require(map, "rpm")?)?,
        feed: positive(require(map, "feed")?, "feed")?,
        c_deg: as_f64(require(map, "c")?, "c")?,
        diameter: positive(require(map, "diameter")?, "diameter")?,
        across: positive(require(map, "across")?, "across")?,
        z_from: as_f64(require(map, "z_from")?, "z_from")?,
        z_to: as_f64(require(map, "z_to")?, "z_to")?,
        width: optional_positive(map, "width")?,
        step_down: optional_positive(map, "step_down")?,
        stepover,
        channel,
    })
}

fn parse_live_mill(map: &Map<String, Value>, names: &mut BTreeSet<String>, has_sub: bool) -> Result<Operation, CliError> {
    if map.contains_key("spindle") {
        let early = map.get("name").and_then(Value::as_str).unwrap_or("live_mill");
        parse_work_spindle(map, early, has_sub, true)?;
    }
    let face = parse_live_face(as_str(require(map, "face")?, "face")?)?;
    let strategy_text = as_str(require(map, "strategy")?, "strategy")?;
    if strategy_text != "profile" && strategy_text != "pocket" {
        return Err(CliError::invalid(format!("unknown strategy `{strategy_text}`")));
    }
    let y_axis = if face == LiveFace::Front {
        match optional(map, "y_axis") {
            Some(value) => as_bool(value, "y_axis")?,
            None => false,
        }
    } else {
        false
    };
    if face == LiveFace::Front && !y_axis && map.contains_key("c") {
        return Err(CliError::invalid("c is only valid when y_axis is true"));
    }
    let mut allowed =
        vec!["name", "type", "tool_id", "spindle", "channel", "face", "drawing", "layer", "strategy", "depth", "step_down", "rpm", "feed"];
    match face {
        LiveFace::Front => {
            allowed.push("z_start");
            allowed.push("y_axis");
            if y_axis {
                allowed.push("c");
            }
        }
        LiveFace::Od => allowed.push("diameter"),
    }
    if strategy_text == "profile" {
        allowed.push("side");
    } else {
        allowed.push("stepover");
    }
    check_keys(map, &allowed, "operation")?;
    let name = required_name(as_str(require(map, "name")?, "operation name")?, "operation name")?;
    if !names.insert(name.clone()) {
        return Err(CliError::invalid(format!("duplicate operation name `{name}`")));
    }
    let spindle = parse_work_spindle(map, &name, has_sub, true)?;
    let channel = parse_channel(map, &name)?;
    let depth = positive(require(map, "depth")?, "depth")?;
    let step_down = match optional(map, "step_down") {
        Some(value) => positive(value, "step_down")?,
        None => depth,
    };
    let (z_start, diameter) = match face {
        LiveFace::Front => (as_f64(require(map, "z_start")?, "z_start")?, 0.0),
        LiveFace::Od => (0.0, positive(require(map, "diameter")?, "diameter")?),
    };
    let c_deg = if y_axis {
        match optional(map, "c") {
            Some(value) => as_f64(value, "c")?,
            None => 0.0,
        }
    } else {
        0.0
    };
    if y_axis && c_deg.abs() > 99_999.0 {
        return Err(CliError::invalid(format!("operation {name}: c is outside -99999..=99999")));
    }
    let strategy = if strategy_text == "profile" {
        MillStrategy::Profile { side: profile_side(as_str(require(map, "side")?, "side")?)? }
    } else {
        let stepover = match optional(map, "stepover") {
            Some(value) => as_f64(value, "stepover")?,
            None => 0.4,
        };
        if !(stepover > 0.0 && stepover <= 1.0) {
            return Err(CliError::invalid("stepover fraction must be in (0, 1]"));
        }
        MillStrategy::Pocket { stepover }
    };
    Ok(Operation::LiveMill {
        name,
        spindle,
        tool_id: as_u32(require(map, "tool_id")?, "tool_id")?,
        rpm: parse_live_rpm(require(map, "rpm")?)?,
        feed: positive(require(map, "feed")?, "feed")?,
        face,
        drawing: drawing_path_text(require(map, "drawing")?)?.to_string(),
        layer: optional_layer(map)?,
        strategy,
        depth,
        step_down,
        z_start,
        diameter,
        y_axis,
        c_deg,
        channel,
    })
}

fn parse_live_face(text: &str) -> Result<LiveFace, CliError> {
    match text {
        "front" => Ok(LiveFace::Front),
        "od" => Ok(LiveFace::Od),
        other => Err(CliError::invalid(format!("unknown face `{other}`"))),
    }
}

fn parse_live_positions(value: &Value, face: LiveFace) -> Result<Vec<LiveStation>, CliError> {
    let list = array(value, "positions", MAX_LIVE_POSITIONS)?;
    let mut positions = Vec::with_capacity(list.len());
    for item in list {
        let map = object(item, "position")?;
        let allowed: &[&str] = match face {
            LiveFace::Front => &["c", "r"],
            LiveFace::Od => &["c", "z"],
        };
        check_keys(map, allowed, "position")?;
        let c_deg = as_f64(require(map, "c")?, "c")?;
        let at = match face {
            LiveFace::Front => non_negative(require(map, "r")?, "r")?,
            LiveFace::Od => as_f64(require(map, "z")?, "z")?,
        };
        positions.push(LiveStation { c_deg, at });
    }
    Ok(positions)
}

fn parse_live_rpm(value: &Value) -> Result<f64, CliError> {
    let rpm = positive(value, "rpm")?;
    if rpm > MAX_SPINDLE_RPM {
        return Err(CliError::invalid(format!("rpm must be at most {MAX_SPINDLE_RPM}")));
    }
    Ok(rpm)
}

/// `c_axis` is checked before the tool orientation, so a live cycle on a spindle with no C axis
/// reports that even when the tool also faces the wrong way.
fn check_tool_use(operation: &Operation, tool: &LatheTool, c_axis: bool) -> Result<(), CliError> {
    let Some(face) = operation.live_face() else {
        return match &tool.body {
            ToolBody::Live { .. } => Err(CliError::run(format!("operation {}: tool {} is a live tool", operation.name(), tool.id))),
            ToolBody::Insert { .. } => Ok(()),
        };
    };
    if !c_axis {
        return Err(CliError::invalid("live operations need spindle.c_axis true"));
    }
    let ToolBody::Live { orientation, kind, max_rpm, .. } = &tool.body else {
        let which = if face.axial() { "an axial" } else { "a radial" };
        return Err(CliError::invalid(format!("operation {}: tool {} is not {which} live tool", operation.name(), tool.id)));
    };
    let axial = matches!(orientation, LiveOrientation::Axial);
    if axial != face.axial() {
        let which = if face.axial() { "an axial" } else { "a radial" };
        return Err(CliError::invalid(format!("operation {}: tool {} is not {which} live tool", operation.name(), tool.id)));
    }
    if operation.is_flat() && *kind != LiveKind::Endmill {
        return Err(CliError::invalid(format!("operation {}: tool {} is not an endmill", operation.name(), tool.id)));
    }
    if let Some(cap) = max_rpm
        && let Some(rpm) = operation.live_rpm()
        && rpm > *cap
    {
        return Err(CliError::invalid(format!("operation {}: rpm exceeds tool {} max_rpm", operation.name(), tool.id)));
    }
    Ok(())
}

/// A flat needs a Y head. Every tool-centre Y on the path must sit in `[y_min, y_max]`.
fn check_flat(operation: &Operation, tool: &LatheTool, head: &LatheHead, clearance: f64) -> Result<(), CliError> {
    let Operation::LiveFlat { name, .. } = operation else {
        return Ok(());
    };
    if !head.y_axis {
        return Err(CliError::invalid(format!("operation {name}: head has no Y axis")));
    }
    let (Some(y_min), Some(y_max)) = (head.y_min, head.y_max) else {
        return Err(CliError::invalid(format!("operation {name}: head has no Y axis")));
    };
    let ToolBody::Live { diameter, .. } = &tool.body else {
        return Ok(());
    };
    let path = live_flat(&flat_params(operation, *diameter, clearance)?).map_err(|err| geometry_err(name, err))?;
    for step in &path.steps {
        let y = step.y();
        if y < y_min - 1e-9 || y > y_max + 1e-9 {
            return Err(CliError::invalid(format!("operation {name}: Y {} outside head travel", format_travel(y))));
        }
    }
    Ok(())
}

/// `y_axis` on a front mill needs a head that lists Y. Travel is checked after the drawing is cut.
fn check_mill_head(operation: &Operation, head: &LatheHead) -> Result<(), CliError> {
    let Operation::LiveMill { name, y_axis, .. } = operation else {
        return Ok(());
    };
    if *y_axis && !head.y_axis {
        return Err(CliError::invalid(format!("operation {name}: y_axis needs head.y_axis")));
    }
    Ok(())
}

fn format_travel(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    let text = format!("{value:.9}");
    let Some((head, frac)) = text.split_once('.') else {
        return text;
    };
    let frac = frac.trim_end_matches('0');
    if frac.is_empty() { head.to_string() } else { format!("{head}.{frac}") }
}

fn geometry_err(name: &str, err: impl std::fmt::Display) -> CliError {
    let text = err.to_string();
    let detail = text.strip_prefix("invalid input: ").unwrap_or(text.as_str());
    CliError::invalid(format!("operation {name}: {detail}"))
}

fn parse_infeed(text: &str) -> Result<Infeed, CliError> {
    match text {
        "radial" => Ok(Infeed::Radial),
        "flank" => Ok(Infeed::Flank { angle_deg: DEFAULT_FLANK_DEG }),
        other => Err(CliError::invalid(format!("unknown infeed `{other}`"))),
    }
}

fn parse_side(text: &str) -> Result<Side, CliError> {
    match text {
        "outside" => Ok(Side::Outside),
        "inside" => Ok(Side::Inside),
        other => Err(CliError::invalid(format!("unknown side `{other}`"))),
    }
}

fn parse_speed(value: &Value, max_rpm: f64) -> Result<SpindleSpeed, CliError> {
    let map = object(value, "speed")?;
    check_keys(map, &["css", "rpm"], "speed")?;
    match (optional(map, "css"), optional(map, "rpm")) {
        (Some(css), None) => Ok(SpindleSpeed::css(positive(css, "css")?, max_rpm)?),
        (None, Some(rpm)) => Ok(SpindleSpeed::rpm(positive(rpm, "rpm")?)?),
        _ => Err(CliError::invalid("speed needs exactly one of `css` or `rpm`")),
    }
}

fn parse_tolerance(value: Option<&Value>) -> Result<(f64, f64), CliError> {
    let Some(value) = value else {
        return Ok((Tolerance::DEFAULT_LINEAR_MM, DEFAULT_JOB_CHORD_MM));
    };
    let map = object(value, "tolerance")?;
    check_keys(map, &["linear", "angular", "chord"], "tolerance")?;
    let linear = match optional(map, "linear") {
        Some(value) => positive(value, "linear")?,
        None => Tolerance::DEFAULT_LINEAR_MM,
    };
    if let Some(angular) = optional(map, "angular") {
        non_negative(angular, "angular")?;
    }
    let chord = match optional(map, "chord") {
        Some(value) => non_negative(value, "chord")?,
        None => DEFAULT_JOB_CHORD_MM,
    };
    Ok((linear, chord))
}

fn load_profile(job_path: &Path, job: &LatheJob, up_axis: crate::step::UpAxis) -> Result<TurnProfile, CliError> {
    load_source(job_path, &job.source, job.linear, up_axis)
}

fn load_source(job_path: &Path, source: &Source, linear: f64, up_axis: crate::step::UpAxis) -> Result<TurnProfile, CliError> {
    let path = resolve_source(job_path, &source.path);
    match &source.kind {
        SourceKind::Dxf { layer } => {
            let text = read_text(&path)?;
            TurnProfile::from_dxf(&text, layer.as_deref(), linear).map_err(show)
        }
        SourceKind::Svg { layer: None } => {
            let text = read_text(&path)?;
            TurnProfile::from_svg(&text, linear).map_err(show)
        }
        SourceKind::Svg { layer: Some(layer) } => {
            let text = read_text(&path)?;
            let tolerance = Tolerance::new(linear, Tolerance::DEFAULT_ANGULAR_RAD).map_err(show)?;
            let drawing = import_svg(&text, &ImportOptions::new(tolerance)).map_err(show)?;
            TurnProfile::from_drawing(&drawing, Some(layer), linear).map_err(show)
        }
        SourceKind::Stl { axis, step } => {
            let bytes = crate::run::read_bytes(&path)?;
            let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
            TurnProfile::from_mesh(&mesh, *axis, *step).map_err(show)
        }
        SourceKind::Step { body } => {
            let cli = crate::step::find_cli().ok_or_else(crate::step::missing_cli_error)?;
            let text = crate::step::section_to_dxf(&cli, &path, body.as_deref(), up_axis)?;
            TurnProfile::from_section_dxf(&text, linear).map_err(show)
        }
    }
}

fn resolve_source(job: &Path, relative: &str) -> PathBuf {
    match job.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(relative),
        _ => PathBuf::from(relative),
    }
}

fn build_document(
    job_path: &Path,
    job: &LatheJob,
    profile: &TurnProfile,
    sub_profile: Option<&TurnProfile>,
) -> Result<(Document, Vec<String>), CliError> {
    let stock = turn_stock(&job.stock)?;
    let points = profile.to_polyline_points(job.chord).into_iter().map(|point| [point.x(), point.y()]).collect();
    let mut setup = turning_setup(job, stock)?.with_section(Some(TurnSection::try_new(points)?))?;
    for tool in &job.tools {
        // Live tools are not mounted as inserts and are not gouge-checked. The cutter diameter
        // is stored on the tool table. A placeholder insert is not used.
        let ToolBody::Insert { insert, .. } = &tool.body else {
            continue;
        };
        let mounted_insert = mount_insert(job, tool.id, insert)?;
        let mounted = TurnInsert::try_new(tool.id, HEAD, tool.station, mounted_insert, false)?;
        setup = setup.with_insert(mounted)?;
    }
    let check = job.operations.iter().any(|operation| gouge_side(operation).is_some());
    let sample_step = if check { Some(crate::gouge::sample_step_for(job.chord)?) } else { None };
    let section = sampled_section(profile, job.chord, sample_step.is_some());
    let sub_section = match (sample_step, sub_profile) {
        (Some(_), Some(sub)) => sampled_section(sub, job.chord, true),
        _ => Vec::new(),
    };
    let mut gouge_lines = Vec::new();
    let mut toolpaths = Vec::with_capacity(job.operations.len());
    let mut previous = [None, None];
    let mut b_now = [0.0, 0.0];
    let mut sync_id = 0u32;
    let mut tailstock_advanced = false;
    let mut steady_rest_clamped = false;
    let mut zone_start: Option<Vec3> = None;
    // Main-frame tip range. Sub-spindle operations are excluded: their tips are in the sub frame.
    // The work offset is not known, so this stays an extent and is not a machine coordinate.
    let mut travel = crate::machine::AxisExtent::new();
    for operation in &job.operations {
        if operation.is_wait() {
            sync_id = sync_id.saturating_add(1);
            for channel in [0u8, 1] {
                let path = wait_toolpath(operation.name(), sync_id)?;
                let path = with_head_index(path, head_for(channel), &mut b_now[usize::from(channel)], 0.0)?;
                let index = u32::try_from(toolpaths.len()).map_err(|_| CliError::run("too many toolpaths"))?;
                toolpaths.push(path);
                let binding = TurnBinding::try_new(index, SPINDLE, head_for(channel), u16::from(channel))?;
                setup = setup.with_binding(binding)?;
            }
            continue;
        }
        let channel = operation.channel();
        let previous_slot = if channel == 0 { &mut previous[0] } else { &mut previous[1] };
        let active_profile = match operation.spindle() {
            WorkSpindle::Main => profile,
            WorkSpindle::Sub => sub_profile.ok_or_else(|| CliError::run("internal: sub profile is missing"))?,
        };
        let gouge_section = match operation.spindle() {
            WorkSpindle::Main => section.as_slice(),
            WorkSpindle::Sub => sub_section.as_slice(),
        };
        let path_index = toolpaths.len();
        let toolpath = if operation.is_flat() {
            flat_toolpath(job, operation, previous_slot)?
        } else if operation.is_mill() {
            mill_toolpath(job_path, job, operation, previous_slot)?
        } else if operation.is_live() {
            let motion = live_motion(operation)?;
            let rpm = operation.live_rpm().ok_or_else(|| CliError::run(format!("internal: operation {} has no live rpm", operation.name())))?;
            with_live_events(motion, operation.tool_id(), rpm, previous_slot)?
        } else if operation.is_transfer() {
            transfer_toolpath(job, active_profile, operation, previous_slot)?
        } else if operation.is_eject() {
            eject_toolpath(operation)?
        } else if operation.is_tailstock() {
            let (path, engage) = tailstock_toolpath(operation)?;
            tailstock_advanced = engage;
            path
        } else if operation.is_steady_rest() {
            let (path, clamp) = steady_rest_toolpath(operation)?;
            steady_rest_clamped = clamp;
            path
        } else if operation.is_bar_feed() {
            bar_feed_toolpath(operation)?
        } else if operation.is_orient() {
            orient_toolpath(operation)?
        } else if operation.is_sub_brake() {
            sub_brake_toolpath(operation)?
        } else {
            let turned = turn_operation(job, active_profile, operation)?;
            if let Some(step) = sample_step
                && let Some(side) = gouge_side(operation)
            {
                gouge_lines.extend(turn_gouges(job, operation, path_index, &turned, side, gouge_section, step)?);
            }
            let motion = to_cl_path(&turned, Some(operation.name().to_string()), operation.tool_id(), operation.feed(), Some(job.clearance))
                .map_err(|err| op_fail(operation.name(), err))?;
            let motion = apply_b_axis(job, motion, operation.b_target())?;
            let motion = if operation.catches() { insert_parts_catcher(motion)? } else { motion };
            with_events(motion, operation.tool_id(), previous_slot)?
        };
        let toolpath = with_head_index(toolpath, head_for(channel), &mut b_now[usize::from(channel)], operation.b_target())?;
        if counts_toward_travel(operation) {
            record_main_tips(&toolpath, &mut travel);
        }
        if tailstock_advanced
            && !operation.is_tailstock()
            && let Some(tailstock) = &job.tailstock
        {
            check_tailstock_zone(operation.name(), &toolpath, tailstock, zone_start)?;
        }
        if steady_rest_clamped
            && !operation.is_steady_rest()
            && let Some(steady_rest) = &job.steady_rest
        {
            check_steady_rest_zone(operation.name(), &toolpath, steady_rest, zone_start)?;
        }
        if channel == 0
            && operation.spindle() == WorkSpindle::Main
            && let Some(tip) = last_motion_tip(&toolpath)
        {
            zone_start = Some(tip);
        }
        let index = u32::try_from(toolpaths.len()).map_err(|_| CliError::run("too many toolpaths"))?;
        toolpaths.push(toolpath);
        let binding =
            TurnBinding::try_new(index, operation.spindle().name(), head_for(channel), u16::from(channel))?.with_speed(operation.turning_speed())?;
        setup = setup.with_binding(binding)?;
    }
    if let Some(machine) = &job.machine {
        travel.check(machine)?;
    }
    let tools = job
        .tools
        .iter()
        .map(|tool| {
            let diameter = match &tool.body {
                ToolBody::Live { diameter, .. } => Some(*diameter),
                ToolBody::Insert { .. } => None,
            };
            Tool::try_new(tool.id, None, diameter, None)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let program = Program::try_new(program_name(job_path), tools, None, toolpaths, Vec::new(), Vec::new(), None)?.with_turning(Some(setup))?;
    Ok((Document::new(program)?, gouge_lines))
}

/// Face is always outside. Rough and finish use the operation side. Thread, groove, part, drill,
/// transfer, eject, tailstock, steady rest, bar feed, orient, sub brake and live cycles are skipped: those tools are meant to enter the profile, or they are not inserts.
fn gouge_side(operation: &Operation) -> Option<Side> {
    match operation {
        Operation::Face { .. } => Some(Side::Outside),
        Operation::Rough { side, .. } | Operation::Finish { side, .. } => Some(*side),
        Operation::Thread { .. }
        | Operation::Groove { .. }
        | Operation::Part { .. }
        | Operation::Drill { .. }
        | Operation::LiveDrill { .. }
        | Operation::LiveSlot { .. }
        | Operation::LiveFlat { .. }
        | Operation::LiveMill { .. }
        | Operation::Transfer { .. }
        | Operation::Eject { .. }
        | Operation::Tailstock { .. }
        | Operation::SteadyRest { .. }
        | Operation::BarFeed { .. }
        | Operation::Orient { .. }
        | Operation::SubBrake { .. }
        | Operation::Wait { .. } => None,
    }
}

fn sampled_section(profile: &TurnProfile, chord: f64, sample: bool) -> Vec<(f64, f64)> {
    if !sample {
        return Vec::new();
    }
    profile.to_polyline_points(chord).into_iter().map(|point| (point.x(), point.y())).collect()
}

/// Spindle `main`, plus spindle `sub` when the job has a sub block. C axis id 0 when the job
/// asks for it. The head is `turret1`, X and Z, plus Y when `head.y_axis` is true, 12 stations.
/// `live_tooling` is set when any tool is live. Head `lower` is added, with the same axes,
/// stations and live-tooling flag, when a channel 1 operation or a wait is present. A plain
/// turning job with no sub and no second channel keeps [`TurningSetup::single`], which has no
/// C axis, no Y and no live tooling. The sub spindle has no C axis, work zero 0 and is not
/// reversed. Stock geometry is the main blank either way.
fn needs_lower_head(job: &LatheJob) -> bool {
    job.operations.iter().any(|operation| operation.is_wait() || operation.channel() == 1)
}

fn head_for(channel: u8) -> &'static str {
    if channel == 1 { LOWER } else { HEAD }
}

/// Event-only path. The cut feed is a placeholder: a cutter-location path requires one, and a sync adds no time.
fn wait_toolpath(name: &str, id: u32) -> Result<ClPath, CliError> {
    let feeds = Feeds::try_new(1.0, None, None, None, None)?;
    let blocks = vec![ClBlock::Event(Event::Sync { id })];
    Ok(ClPath::try_new(Some(name.to_string()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?)
}

fn turning_setup(job: &LatheJob, stock: TurnStock) -> Result<TurningSetup, CliError> {
    let live_tooling = job.tools.iter().any(|tool| tool.body.is_live());
    let lower = needs_lower_head(job);
    if !job.c_axis && !live_tooling && !job.head.y_axis && job.head.b_axis.is_none() && job.sub.is_none() && !lower {
        return Ok(TurningSetup::single(Some(stock), job.max_rpm)?);
    }
    let c_axis = if job.c_axis { Some(0) } else { None };
    let mut spindles = vec![Spindle::try_new(SPINDLE, c_axis, 0.0, job.max_rpm)?];
    if let Some(sub) = &job.sub {
        spindles.push(Spindle::try_new(SUB_SPINDLE, None, 0.0, sub.max_rpm)?);
    }
    let stations = u16::try_from(STATIONS).map_err(|_| CliError::run("turret station count is out of range"))?;
    let mut axes = if job.head.y_axis { vec![HeadAxis::X, HeadAxis::Y, HeadAxis::Z] } else { vec![HeadAxis::X, HeadAxis::Z] };
    if let Some(b_axis) = &job.head.b_axis {
        let _ = b_axis.pivot_length;
        axes.push(HeadAxis::B);
    }
    let mut heads = vec![Head::try_new(HEAD, axes.clone(), stations, live_tooling)?];
    if lower {
        heads.push(Head::try_new(LOWER, axes, stations, live_tooling)?);
    }
    Ok(TurningSetup::try_new(spindles, heads, Some(stock), Vec::new(), Vec::new())?)
}

fn turn_gouges(
    job: &LatheJob,
    operation: &Operation,
    index: usize,
    path: &TurnPath,
    side: Side,
    section: &[(f64, f64)],
    sample_step: f64,
) -> Result<Vec<String>, CliError> {
    let (insert, tip, _nose_radius, holder) = planning_insert(job, operation)?;
    let outline = InsertGeom::from_insert(&insert, TURN_EDGE_MM).map_err(|err| op_fail(operation.name(), err))?;
    let mut found =
        check_turn_gouges(section, side, path, tip, &outline, holder, sample_step, job.chord).map_err(|err| op_fail(operation.name(), err))?;
    for gouge in &mut found {
        gouge.toolpath = index;
    }
    Ok(found.iter().map(|gouge| format_turn_gouge(operation.name(), gouge)).collect())
}

fn format_turn_gouge(name: &str, gouge: &TurnGouge) -> String {
    let mut clean = String::new();
    for ch in name.chars() {
        if ch.is_control() {
            clean.push(' ');
        } else {
            clean.push(ch);
        }
    }
    format!("gouge operation {clean} z {} x {} depth {} {}", num6(gouge.z), num6(gouge.x), num6(gouge.depth), gouge.what.name())
}

fn num6(value: f64) -> String {
    if value.is_finite() { format!("{value:.6}") } else { "non_finite".to_string() }
}

fn turn_stock(stock: &Stock) -> Result<TurnStock, CliError> {
    match *stock {
        Stock::Bar { diameter, length } => Ok(TurnStock::bar(diameter, length)?),
        Stock::Tube { outer, inner, length } => Ok(TurnStock::tube(outer, inner, length)?),
    }
}

/// Tool-table orientation, unless every use of the tool shares one B. A shared angle is added
/// to `orientation_deg` on the mounted copy. The table entry itself is left as written.
fn mount_insert(job: &LatheJob, tool_id: u32, insert: &Insert) -> Result<Insert, CliError> {
    let mut agreed: Option<f64> = None;
    for operation in &job.operations {
        let Some(angle) = mount_angle(operation, tool_id) else {
            continue;
        };
        match agreed {
            None => agreed = Some(angle),
            Some(previous) if (previous - angle).abs() > 1e-9 => return Ok(*insert),
            Some(_) => {}
        }
    }
    match agreed {
        Some(angle) if angle != 0.0 => effective_insert(insert, angle),
        _ => Ok(*insert),
    }
}

fn mount_angle(operation: &Operation, tool_id: u32) -> Option<f64> {
    let uses = match operation {
        Operation::Transfer { part_off, .. } => part_off.as_ref().is_some_and(|part| part.tool_id == tool_id),
        Operation::Wait { .. }
        | Operation::Eject { .. }
        | Operation::Tailstock { .. }
        | Operation::SteadyRest { .. }
        | Operation::BarFeed { .. }
        | Operation::Orient { .. }
        | Operation::SubBrake { .. } => false,
        _ => operation.checked_tool() == Some(tool_id),
    };
    if uses { Some(operation.b_deg().unwrap_or(0.0)) } else { None }
}

/// Insert and programmed tip for one operation. `b_deg` replaces the orientation with
/// `orientation_deg + b_deg`, wrapped into −360..=360, and rebuilds an imaginary tip from that insert.
fn planning_insert<'a>(job: &'a LatheJob, operation: &Operation) -> Result<(Insert, TipPoint, f64, Option<&'a HolderGeom>), CliError> {
    let (insert, tip, nose_radius, holder) = require_insert(job, operation)?;
    let Some(b_deg) = operation.b_deg() else {
        return Ok((*insert, tip, nose_radius, holder));
    };
    let effective = effective_insert(insert, b_deg)?;
    let tip = match tip {
        TipPoint::Centre => TipPoint::Centre,
        TipPoint::ImaginaryTip { .. } => {
            let (dx, dz) = effective.tip_offset();
            TipPoint::ImaginaryTip { dz, dx }
        }
    };
    Ok((effective, tip, nose_radius, holder))
}

fn effective_insert(insert: &Insert, b_deg: f64) -> Result<Insert, CliError> {
    let orientation = normalise_orientation(insert.orientation_deg() + b_deg);
    Ok(Insert::try_new(insert.shape(), insert.nose_radius(), insert.width(), insert.hand(), orientation)?)
}

fn normalise_orientation(mut degrees: f64) -> f64 {
    while degrees > 360.0 {
        degrees -= 360.0;
    }
    while degrees < -360.0 {
        degrees += 360.0;
    }
    degrees
}

/// Pose axis `(cos B, 0, sin B)` on a turning path when the job has a tilting B head.
fn apply_b_axis(job: &LatheJob, path: ClPath, b_deg: f64) -> Result<ClPath, CliError> {
    if job.head.b_axis.is_none() {
        return Ok(path);
    }
    let radians = b_deg.to_radians();
    let axis = Vec3::new(radians.cos(), 0.0, radians.sin())?;
    let blocks = path
        .blocks()
        .iter()
        .map(|block| match block {
            ClBlock::Motion(motion) => Ok(ClBlock::Motion(map_motion(motion, axis)?)),
            other => Ok(other.clone()),
        })
        .collect::<Result<Vec<_>, CliError>>()?;
    rebuild_path(path, blocks)
}

fn map_motion(motion: &ClMotion, axis: Vec3) -> Result<ClMotion, CliError> {
    Ok(match motion {
        ClMotion::Rapid { to, feed } => ClMotion::Rapid { to: map_pose(*to, axis)?, feed: *feed },
        ClMotion::Linear { to, feed, role } => ClMotion::Linear { to: map_pose(*to, axis)?, feed: *feed, role: *role },
        ClMotion::Arc { to, plane, center, sense, feed, role } => {
            ClMotion::Arc { to: map_pose(*to, axis)?, plane: *plane, center: *center, sense: *sense, feed: *feed, role: *role }
        }
        ClMotion::Helix { to, plane, center, sense, sweep, feed, role } => {
            ClMotion::Helix { to: map_pose(*to, axis)?, plane: *plane, center: *center, sense: *sense, sweep: *sweep, feed: *feed, role: *role }
        }
        ClMotion::Thread { to, lead } => ClMotion::Thread { to: map_pose(*to, axis)?, lead: *lead },
    })
}

fn map_pose(pose: Pose, axis: Vec3) -> Result<Pose, CliError> {
    let mut next = Pose::new(pose.tip(), axis)?;
    if let Some(rotary) = pose.rotary() {
        next = next.with_rotary(rotary)?;
    }
    Ok(next)
}

/// `HeadIndex` after the first tool change, or at the start of the path, when B changes.
fn with_head_index(path: ClPath, head: &str, current: &mut f64, target: f64) -> Result<ClPath, CliError> {
    if (target - *current).abs() <= 1e-9 {
        return Ok(path);
    }
    *current = target;
    let event = ClBlock::Event(Event::HeadIndex { head: head.to_string(), b_deg: target });
    let mut blocks = path.blocks().to_vec();
    let at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::ToolChange { .. }))).map(|index| index + 1).unwrap_or(0);
    blocks.insert(at, event);
    rebuild_path(path, blocks)
}

fn rebuild_path(path: ClPath, blocks: Vec<ClBlock>) -> Result<ClPath, CliError> {
    Ok(ClPath::try_new(
        path.name().map(str::to_string),
        path.tool_id(),
        path.compensation(),
        path.feeds(),
        path.pierce(),
        path.tabs().to_vec(),
        blocks,
    )?)
}

fn require_insert<'a>(job: &'a LatheJob, operation: &Operation) -> Result<(&'a Insert, TipPoint, f64, Option<&'a HolderGeom>), CliError> {
    let tool = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?;
    match &tool.body {
        ToolBody::Insert { insert, tip, nose_radius, holder } => Ok((insert, *tip, *nose_radius, holder.as_ref())),
        ToolBody::Live { .. } => Err(CliError::run(format!("operation {}: tool {} is a live tool", operation.name(), tool.id))),
    }
}

/// Facing depth for this spindle. The sub uses `back_allowance` and the same stock diameters.
fn front_allowance(job: &LatheJob, spindle: WorkSpindle) -> Result<f64, CliError> {
    match spindle {
        WorkSpindle::Main => Ok(job.front_allowance),
        WorkSpindle::Sub => job.sub.as_ref().map(|block| block.back_allowance).ok_or_else(|| CliError::run("internal: sub spindle is missing")),
    }
}

fn turn_operation(job: &LatheJob, profile: &TurnProfile, operation: &Operation) -> Result<TurnPath, CliError> {
    if operation.is_wait()
        || operation.is_live()
        || operation.is_mill()
        || operation.is_transfer()
        || operation.is_eject()
        || operation.is_tailstock()
        || operation.is_steady_rest()
        || operation.is_bar_feed()
        || operation.is_orient()
        || operation.is_sub_brake()
    {
        return Err(CliError::run(format!("internal: operation {} has no turning path", operation.name())));
    }
    let (insert, tip, nose_radius, _) = planning_insert(job, operation)?;
    let turn_tool = TurnTool { nose_radius, tip };
    let stock = turn_stock(&job.stock)?;
    match operation {
        Operation::Face { name, spindle, step_z, .. } => {
            let params = FaceParams {
                stock_front_z: front_allowance(job, *spindle)?,
                final_z: 0.0,
                step_z: *step_z,
                from_radius: stock.outer_radius(),
                to_radius: face_to_radius(&turn_tool),
                clearance: job.clearance,
            };
            face(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Rough { name, side, depth_of_cut, leave_x, leave_z, .. } => {
            let params = RoughParams {
                side: *side,
                stock_outer_radius: stock.outer_radius(),
                stock_inner_radius: stock.inner_radius(),
                depth_of_cut: *depth_of_cut,
                leave_x: *leave_x,
                leave_z: *leave_z,
                clearance: job.clearance,
                retract: job.clearance,
            };
            rough(profile, &turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Finish { name, side, leave_x, leave_z, .. } => {
            let params = FinishParams { side: *side, leave_x: *leave_x, leave_z: *leave_z, clearance: job.clearance, tol: job.linear };
            finish(profile, &turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Thread {
            name,
            side,
            major_diameter,
            pitch,
            z_start,
            z_end,
            depth,
            first_depth,
            min_depth,
            spring_passes,
            infeed,
            clearance,
            ..
        } => {
            let params = ThreadParams {
                side: *side,
                major_radius: *major_diameter * 0.5,
                depth: *depth,
                pitch: *pitch,
                z_start: *z_start,
                z_end: *z_end,
                first_depth: *first_depth,
                min_depth: *min_depth,
                spring_passes: *spring_passes,
                infeed: *infeed,
                clearance: *clearance,
                included_angle_deg: 60.0,
            };
            thread(&params).map_err(|err| op_fail(name, err))
        }
        Operation::Groove {
            name,
            side,
            z_left,
            z_right,
            bottom_diameter,
            surface_diameter,
            insert_width,
            peck,
            stepover_frac,
            clearance,
            dwell_s,
            ..
        } => {
            let width = match insert_width {
                Some(width) => *width,
                None => insert.width().ok_or_else(|| op_fail(name, "groove insert needs a width"))?,
            };
            let params = GrooveParams {
                side: *side,
                z_left: *z_left,
                z_right: *z_right,
                bottom_radius: *bottom_diameter * 0.5,
                surface_radius: surface_diameter.map(|diameter| diameter * 0.5).unwrap_or_else(|| stock.outer_radius()),
                insert_width: width,
                peck: *peck,
                stepover_frac: *stepover_frac,
                clearance: *clearance,
                dwell_s: *dwell_s,
            };
            groove(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Part { name, z, to_diameter, chamfer, peck, clearance, catch, .. } => {
            let width = insert.width().ok_or_else(|| op_fail(name, "part-off insert needs a width"))?;
            let surface_radius = stock.outer_radius();
            if *catch {
                let catcher = job
                    .parts_catcher
                    .as_ref()
                    .ok_or_else(|| CliError::invalid(format!("operation {name}: catch needs a \"parts_catcher\" block")))?;
                // The part op already uses the stock outer radius. Fall back to the job stock
                // diameter only when that radius is not a usable number.
                let diameter = if surface_radius.is_finite() { surface_radius * 2.0 } else { stock_outer_diameter(&job.stock) };
                let length = z.abs() + width;
                if diameter > catcher.max_diameter {
                    return Err(CliError::invalid(format!(
                        "operation {name}: part diameter {diameter:.3} exceeds parts_catcher.max_diameter {:.3}",
                        catcher.max_diameter
                    )));
                }
                if length > catcher.max_length {
                    return Err(CliError::invalid(format!(
                        "operation {name}: part length {length:.3} exceeds parts_catcher.max_length {:.3}",
                        catcher.max_length
                    )));
                }
            }
            let params = PartParams {
                z: *z,
                surface_radius,
                insert_width: width,
                to_radius: *to_diameter * 0.5,
                chamfer: *chamfer,
                peck: *peck,
                clearance: *clearance,
            };
            part_off(&turn_tool, &params).map_err(|err| op_fail(name, err))
        }
        Operation::Drill { name, z_start, depth, diameter, peck, retract, point_angle_deg, through, .. } => {
            let params = DrillParams {
                z_start: *z_start,
                depth: *depth,
                peck: *peck,
                retract: *retract,
                clearance: job.clearance,
                point_angle_deg: *point_angle_deg,
                diameter: *diameter,
                through: *through,
            };
            centre_drill(&params).map_err(|err| op_fail(name, err))
        }
        Operation::LiveDrill { .. }
        | Operation::LiveSlot { .. }
        | Operation::LiveFlat { .. }
        | Operation::LiveMill { .. }
        | Operation::Transfer { .. }
        | Operation::Eject { .. }
        | Operation::Tailstock { .. }
        | Operation::SteadyRest { .. }
        | Operation::BarFeed { .. }
        | Operation::Orient { .. }
        | Operation::SubBrake { .. }
        | Operation::Wait { .. } => Err(CliError::run(format!("internal: operation {} has no turning path", operation.name()))),
    }
}

fn transfer_toolpath(job: &LatheJob, profile: &TurnProfile, operation: &Operation, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let Operation::Transfer { name, rpm, phase, approach_z, grip_z, park_z, feed, dwell, part_off } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a transfer", operation.name())));
    };
    let mut blocks = vec![
        chuck(SUB_SPINDLE, false),
        spindle_run(SPINDLE, 0.0, false),
        spindle_run(SUB_SPINDLE, 0.0, false),
        ClBlock::Event(Event::SpindleSync { engage: true, phase_deg: *phase }),
    ];
    if *rpm > 0.0 {
        // Jobs build the main spindle with `reversed` false, so a running handoff is forward.
        blocks.push(spindle_run(SPINDLE, *rpm, false));
    }
    blocks.push(housing(SUB_SPINDLE, *approach_z, false, None));
    blocks.push(ClBlock::Event(Event::Dwell { seconds: *dwell }));
    blocks.push(housing(SUB_SPINDLE, *grip_z, false, Some(*feed)));
    blocks.push(chuck(SUB_SPINDLE, true));
    blocks.push(ClBlock::Event(Event::Dwell { seconds: *dwell }));
    let (tool_id, cut_feed, compensation) = if let Some(part) = part_off {
        blocks.extend(part_off_blocks(job, profile, name, part, previous)?);
        (Some(part.tool_id), part.feed, CompensationMode::Computed)
    } else {
        blocks.push(chuck(SPINDLE, false));
        (None, *feed, CompensationMode::Off)
    };
    blocks.push(housing(SUB_SPINDLE, *approach_z, false, None));
    blocks.push(ClBlock::Event(Event::SpindleSync { engage: false, phase_deg: None }));
    blocks.push(spindle_run(SPINDLE, 0.0, false));
    blocks.push(housing(SUB_SPINDLE, *park_z, true, None));
    if part_off.is_none() {
        blocks.push(chuck(SPINDLE, true));
    }
    let feeds = Feeds::try_new(cut_feed, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.clone()), tool_id, compensation, feeds, None, Vec::new(), blocks)?)
}

fn part_off_blocks(job: &LatheJob, profile: &TurnProfile, name: &str, part: &PartOff, previous: &mut Option<u32>) -> Result<Vec<ClBlock>, CliError> {
    let part_op = Operation::Part {
        name: name.to_string(),
        spindle: WorkSpindle::Main,
        tool_id: part.tool_id,
        z: part.z,
        to_diameter: part.to_diameter,
        chamfer: part.chamfer,
        peck: part.peck,
        clearance: part.clearance,
        feed: part.feed,
        speed: part.speed,
        channel: 0,
        b_deg: None,
        catch: false,
    };
    let turned = turn_operation(job, profile, &part_op)?;
    let motion = to_cl_path(&turned, Some(name.to_string()), part.tool_id, part.feed, Some(job.clearance)).map_err(|err| op_fail(name, err))?;
    let motion = apply_b_axis(job, motion, 0.0)?;
    let with = with_events(motion, part.tool_id, previous)?;
    let mut blocks = with.blocks().to_vec();
    match blocks.pop() {
        Some(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle })) => Ok(blocks),
        _ => Err(CliError::run(format!("internal: operation {name} part-off did not end on spindle stop"))),
    }
}

/// Extend after the approach rapid and before the first feed that changes X. Retract after the
/// last motion, which is the part-off's final retract.
///
/// The retract block is inserted first. Its index is past every motion, so the later extend
/// insert shifts it by one and it stays after that retract.
fn insert_parts_catcher(path: ClPath) -> Result<ClPath, CliError> {
    let blocks = path.blocks().to_vec();
    let mut approach = None;
    let mut prev_x = None;
    let mut feed_toward_x = None;
    let mut last_motion = None;
    for (index, block) in blocks.iter().enumerate() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        last_motion = Some(index);
        let x = motion_tip(motion).x();
        if approach.is_none() {
            if !matches!(motion, ClMotion::Rapid { .. }) {
                return Err(CliError::run("internal: part-off approach is not a rapid"));
            }
            approach = Some(index);
            prev_x = Some(x);
            continue;
        }
        if feed_toward_x.is_none() && matches!(motion, ClMotion::Linear { .. }) && prev_x.is_some_and(|previous| (x - previous).abs() > 1e-9) {
            feed_toward_x = Some(index);
        }
        prev_x = Some(x);
    }
    let approach = approach.ok_or_else(|| CliError::run("internal: part-off has no approach rapid"))?;
    let feed_toward_x = feed_toward_x.ok_or_else(|| CliError::run("internal: part-off has no feed toward X"))?;
    let last_motion = last_motion.ok_or_else(|| CliError::run("internal: part-off has no retract"))?;
    if feed_toward_x != approach + 1 {
        return Err(CliError::run("internal: part-off feed toward X does not follow the approach rapid"));
    }
    let mut blocks = blocks;
    blocks.insert(last_motion + 1, ClBlock::Event(Event::PartsCatcher { extend: false }));
    blocks.insert(approach + 1, ClBlock::Event(Event::PartsCatcher { extend: true }));
    rebuild_path(path, blocks)
}

fn stock_outer_diameter(stock: &Stock) -> f64 {
    match *stock {
        Stock::Bar { diameter, .. } => diameter,
        Stock::Tube { outer, .. } => outer,
    }
}

fn bar_feed_toolpath(operation: &Operation) -> Result<ClPath, CliError> {
    let Operation::BarFeed { name, part_length, min_clamp } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a bar feed", operation.name())));
    };
    let blocks = vec![ClBlock::Event(Event::BarFeed { part_length: *part_length, min_clamp: *min_clamp })];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?)
}

fn orient_toolpath(operation: &Operation) -> Result<ClPath, CliError> {
    let Operation::Orient { name, target, angle_deg } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not an orient", operation.name())));
    };
    let blocks = vec![ClBlock::Event(Event::SpindleOrient { spindle: target.name().to_string(), angle_deg: *angle_deg })];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?)
}

fn sub_brake_toolpath(operation: &Operation) -> Result<ClPath, CliError> {
    let Operation::SubBrake { name, clamp } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a sub brake", operation.name())));
    };
    let blocks = vec![ClBlock::Event(Event::SpindleBrake { spindle: SUB_SPINDLE.to_string(), clamp: *clamp })];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?)
}

fn tailstock_toolpath(operation: &Operation) -> Result<(ClPath, bool), CliError> {
    let Operation::Tailstock { name, engage } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a tailstock move", operation.name())));
    };
    let blocks = vec![ClBlock::Event(Event::Tailstock { engage: *engage })];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    let path = ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?;
    Ok((path, *engage))
}

fn steady_rest_toolpath(operation: &Operation) -> Result<(ClPath, bool), CliError> {
    let Operation::SteadyRest { name, clamp } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a steady rest move", operation.name())));
    };
    let blocks = vec![ClBlock::Event(Event::SteadyRest { clamp: *clamp })];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    let path = ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?;
    Ok((path, *clamp))
}

/// A tip in the quill cylinder is rejected. X of an ordinary turning pose is a radius. Face
/// interpolation and a Y move are Cartesian, so the radius is the distance from the spindle axis.
/// Cylinder interpolation keeps the radius in X and the unrolled distance in Y.
/// Rejects any move of `path` that passes through the tailstock zone: `z > -clearance` and radius
/// below `diameter / 2 + clearance`. Each move is checked as the straight segment from the previous
/// tip (`start` for the first move, the last tip of the previous main-channel operation), so a
/// diagonal rapid between two points outside the zone still fails. Arcs are checked by their chord;
/// the clearance covers the chord error.
fn check_tailstock_zone(name: &str, path: &ClPath, tailstock: &TailstockBlock, start: Option<Vec3>) -> Result<(), CliError> {
    let limit = tailstock.diameter * 0.5 + tailstock.clearance;
    let z_min = -tailstock.clearance;
    let mut face = false;
    let mut cylinder = false;
    let mut prev = start;
    for block in path.blocks() {
        match block {
            ClBlock::Event(Event::FaceInterpolation { on }) => {
                face = *on;
                if *on {
                    cylinder = false;
                }
            }
            ClBlock::Event(Event::CylinderInterpolation { radius }) => {
                cylinder = radius.is_some();
                if cylinder {
                    face = false;
                }
            }
            ClBlock::Motion(motion) => {
                let tip = motion_tip(motion);
                let from = prev.unwrap_or(tip);
                prev = Some(tip);
                if let Some((radius, z)) = segment_zone_hit(from, tip, face, cylinder, z_min, limit) {
                    let diameter = radius * 2.0;
                    return Err(CliError::invalid(format!("operation {name}: move at X{diameter:.3} Z{z:.3} enters the tailstock zone")));
                }
            }
            _ => {}
        }
    }
    Ok(())
}

/// Rejects any move of `path` whose Z range overlaps the steady-rest band
/// `[z - width / 2 - clearance, z + width / 2 + clearance]`. Each move is the straight segment
/// from the previous tip (`start` for the first move, the last tip of the previous main-channel
/// operation), so a rapid that starts on one side of the band and ends on the other still fails.
/// Arcs are checked by their chord. The reported point is the segment end inside the band, or
/// the end nearest the band when neither end is inside. X in the message is the diameter.
fn check_steady_rest_zone(name: &str, path: &ClPath, rest: &SteadyRestBlock, start: Option<Vec3>) -> Result<(), CliError> {
    let (z_lo, z_hi) = steady_rest_band(rest);
    let mut face = false;
    let mut cylinder = false;
    let mut prev = start;
    for block in path.blocks() {
        match block {
            ClBlock::Event(Event::FaceInterpolation { on }) => {
                face = *on;
                if *on {
                    cylinder = false;
                }
            }
            ClBlock::Event(Event::CylinderInterpolation { radius }) => {
                cylinder = radius.is_some();
                if cylinder {
                    face = false;
                }
            }
            ClBlock::Motion(motion) => {
                let tip = motion_tip(motion);
                let from = prev.unwrap_or(tip);
                prev = Some(tip);
                if segment_crosses_band(from.z(), tip.z(), z_lo, z_hi) {
                    let at = band_report_tip(from, tip, z_lo, z_hi);
                    let diameter = reported_radius(at, face, cylinder) * 2.0;
                    let z = at.z();
                    return Err(CliError::invalid(format!("operation {name}: move at X{diameter:.3} Z{z:.3} crosses the steady rest band")));
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn steady_rest_band(rest: &SteadyRestBlock) -> (f64, f64) {
    let half = rest.width * 0.5;
    (rest.z - half - rest.clearance, rest.z + half + rest.clearance)
}

fn segment_crosses_band(z0: f64, z1: f64, z_lo: f64, z_hi: f64) -> bool {
    let seg_lo = z0.min(z1);
    let seg_hi = z0.max(z1);
    seg_lo <= z_hi && seg_hi >= z_lo
}

/// The segment end inside the band, or the end nearest the band. A tie reports `to`.
fn band_report_tip(from: Vec3, to: Vec3, z_lo: f64, z_hi: f64) -> Vec3 {
    let inside = |z: f64| z >= z_lo && z <= z_hi;
    let from_in = inside(from.z());
    let to_in = inside(to.z());
    if to_in != from_in {
        return if to_in { to } else { from };
    }
    if from_in {
        return to;
    }
    let distance = |z: f64| if z < z_lo { z_lo - z } else { z - z_hi };
    if distance(from.z()) < distance(to.z()) { from } else { to }
}

/// Radius is X in turning and cylinder mode, and the distance from the spindle axis when the tip
/// is off the XZ plane or face interpolation is on.
fn reported_radius(tip: Vec3, face: bool, cylinder: bool) -> f64 {
    let planar = cylinder || (!face && tip.y().abs() <= 1.0e-9);
    if planar { tip.x() } else { tip.x().hypot(tip.y()) }
}

/// The point of the segment `from`→`to` with the smallest radius among those with `z > z_min`,
/// when that radius is below `limit`. Radius is X in turning and cylinder mode and |XY| when the
/// tip is off the XZ plane or face interpolation is on.
fn segment_zone_hit(from: Vec3, to: Vec3, face: bool, cylinder: bool, z_min: f64, limit: f64) -> Option<(f64, f64)> {
    let dz = to.z() - from.z();
    // Interval of t in [0, 1] where z(t) > z_min (open at a crossing).
    let (lo, hi) = if dz.abs() <= 1.0e-12 {
        if from.z() > z_min { (0.0, 1.0) } else { return None }
    } else {
        let t = (z_min - from.z()) / dz;
        if dz > 0.0 { (t.max(0.0), 1.0) } else { (0.0, t.min(1.0)) }
    };
    if lo > hi || (lo == hi && !(from.z() + dz * lo > z_min)) {
        return None;
    }
    let planar = cylinder || (!face && from.y().abs() <= 1.0e-9 && to.y().abs() <= 1.0e-9);
    let t = if planar {
        if to.x() - from.x() >= 0.0 { lo } else { hi }
    } else {
        let (px, py) = (from.x(), from.y());
        let (dx, dy) = (to.x() - from.x(), to.y() - from.y());
        let len2 = dx * dx + dy * dy;
        let t0 = if len2 <= 1.0e-24 { lo } else { -(px * dx + py * dy) / len2 };
        t0.clamp(lo, hi)
    };
    let x = from.x() + (to.x() - from.x()) * t;
    let y = from.y() + (to.y() - from.y()) * t;
    let z = from.z() + dz * t;
    let radius = if planar { x } else { x.hypot(y) };
    // A hit exactly on the z boundary only counts if the segment continues inside.
    let inside_z = z > z_min || (hi > lo && radius < limit);
    (inside_z && radius < limit).then_some((radius, z))
}

/// Sub-spindle motions are in the sub frame, so only main-spindle toolpaths count.
fn counts_toward_travel(operation: &Operation) -> bool {
    operation.spindle() == WorkSpindle::Main
}

/// Tip X is the radius stored on the cutter-location pose.
fn record_main_tips(path: &ClPath, travel: &mut crate::machine::AxisExtent) {
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        let tip = motion_tip(motion);
        travel.include_radius(tip.x(), tip.z());
    }
}

fn last_motion_tip(path: &ClPath) -> Option<Vec3> {
    path.blocks().iter().rev().find_map(|block| match block {
        ClBlock::Motion(motion) => Some(motion_tip(motion)),
        _ => None,
    })
}

fn motion_tip(motion: &ClMotion) -> Vec3 {
    let pose = match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => to,
    };
    pose.tip()
}

fn eject_toolpath(operation: &Operation) -> Result<ClPath, CliError> {
    let Operation::Eject { name } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not an eject", operation.name())));
    };
    let blocks = vec![spindle_run(SUB_SPINDLE, 0.0, false), chuck(SUB_SPINDLE, false)];
    let feeds = Feeds::try_new(EVENT_ONLY_FEED, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.clone()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks)?)
}

fn chuck(spindle: &str, clamp: bool) -> ClBlock {
    ClBlock::Event(Event::ChuckClamp { spindle: spindle.to_string(), clamp })
}

fn spindle_run(spindle: &str, rpm: f64, reverse: bool) -> ClBlock {
    ClBlock::Event(Event::SpindleRun { spindle: spindle.to_string(), rpm, reverse })
}

fn housing(spindle: &str, z: f64, machine: bool, feed: Option<f64>) -> ClBlock {
    ClBlock::Event(Event::SpindleMove { spindle: spindle.to_string(), z, machine, feed })
}

/// End radius passed to [`face`].
///
/// A centre tip is shifted by `−nose_radius` inside [`face`], so `to_radius` 0 programs the nose
/// centre at `−nose_radius`. An imaginary tip is used as given, so `to_radius` is already
/// `−nose_radius`. Either way the cutting feed ends one nose radius past the spindle axis.
/// [`face`] also adds [`FaceParams::clearance`] to [`FaceParams::from_radius`], so the pass
/// starts at the stock outer radius plus the job clearance.
fn face_to_radius(tool: &TurnTool) -> f64 {
    match tool.tip {
        TipPoint::Centre => 0.0,
        TipPoint::ImaginaryTip { .. } => -tool.nose_radius,
    }
}

fn live_motion(operation: &Operation) -> Result<ClPath, CliError> {
    let (name, tool_id, feed, path) = match operation {
        Operation::LiveDrill { name, tool_id, feed, params, .. } => {
            let path = live_drill(params).map_err(|err| geometry_err(name, err))?;
            (name.as_str(), *tool_id, *feed, path)
        }
        Operation::LiveSlot { name, tool_id, feed, params, .. } => {
            let path = live_slot(params).map_err(|err| geometry_err(name, err))?;
            (name.as_str(), *tool_id, *feed, path)
        }
        _ => return Err(CliError::run(format!("internal: operation {} is not a live cycle", operation.name()))),
    };
    live_cl_path(name, tool_id, feed, &path)
}

/// Every motion carries the current C. The pose axis is +Z for a front (axial) tool and +X for
/// an OD (radial) tool. Rapids have no feed. Feeds are [`MoveRole::Cut`].
fn live_cl_path(name: &str, tool_id: u32, feed: f64, path: &LivePath) -> Result<ClPath, CliError> {
    let axis = if path.axial { Vec3::Z } else { Vec3::X };
    let feeds = Feeds::try_new(feed, None, None, None, None)?;
    let mut blocks = Vec::with_capacity(path.steps.len());
    for step in &path.steps {
        let block = match *step {
            LiveStep::Rapid { z, x, c_deg } => ClBlock::Motion(ClMotion::Rapid { to: live_pose(z, x, c_deg, axis)?, feed: None }),
            LiveStep::Feed { z, x, c_deg } => ClBlock::Motion(ClMotion::Linear { to: live_pose(z, x, c_deg, axis)?, feed, role: MoveRole::Cut }),
            LiveStep::Clamp => ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: true }),
            LiveStep::Release => ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: false }),
        };
        blocks.push(block);
    }
    Ok(ClPath::try_new(Some(name.to_string()), Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)?)
}

fn flat_params(operation: &Operation, tool_diameter: f64, clearance: f64) -> Result<LiveFlatParams, CliError> {
    let Operation::LiveFlat { c_deg, diameter, across, z_from, z_to, width, step_down, stepover, .. } = operation else {
        return Err(CliError::run(format!("internal: operation {} is not a flat", operation.name())));
    };
    Ok(LiveFlatParams {
        c_deg: *c_deg,
        stock_diameter: *diameter,
        across: *across,
        z_from: *z_from,
        z_to: *z_to,
        width: *width,
        step_down: *step_down,
        stepover: *stepover,
        tool_diameter,
        clearance,
    })
}

/// Tool change, C engage, the Y=0 rapid, brake clamp, live tool on, the feeds, live tool off,
/// brake release, C disengage.
fn flat_toolpath(job: &LatheJob, operation: &Operation, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let ToolBody::Live { diameter, .. } = job
        .tools
        .iter()
        .find(|tool| tool.id == operation.tool_id())
        .map(|tool| &tool.body)
        .ok_or_else(|| CliError::run(format!("operation {}: missing tool {}", operation.name(), operation.tool_id())))?
    else {
        return Err(CliError::run(format!("operation {}: tool {} is not a live tool", operation.name(), operation.tool_id())));
    };
    let path = live_flat(&flat_params(operation, *diameter, job.clearance)?).map_err(|err| geometry_err(operation.name(), err))?;
    let Some(rpm) = operation.live_rpm() else {
        return Err(CliError::run(format!("internal: operation {} has no live rpm", operation.name())));
    };
    flat_blocks(operation.name(), operation.tool_id(), operation.feed(), rpm, &path, previous)
}

fn flat_blocks(name: &str, tool_id: u32, feed: f64, rpm: f64, path: &FlatPath, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let Some((first, rest)) = path.steps.split_first() else {
        return Err(CliError::run(format!("operation {name}: flat path is empty")));
    };
    let FlatStep::Rapid { .. } = first else {
        return Err(CliError::run(format!("operation {name}: flat path does not start on a rapid")));
    };
    let mut blocks = Vec::with_capacity(path.steps.len().saturating_add(6));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: true }));
    blocks.push(flat_block(first, feed)?);
    blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: true }));
    blocks.push(ClBlock::Event(Event::LiveToolOn { rpm, direction: LiveDirection::Forward }));
    for step in rest {
        blocks.push(flat_block(step, feed)?);
    }
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: SPINDLE.to_string(), clamp: false }));
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: false }));
    let feeds = Feeds::try_new(feed, None, None, None, None)?;
    Ok(ClPath::try_new(Some(name.to_string()), Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)?)
}

fn mill_toolpath(job_path: &Path, job: &LatheJob, operation: &Operation, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let Operation::LiveMill { name, tool_id, rpm, feed, face, drawing, layer, strategy, depth, step_down, z_start, diameter, y_axis, c_deg, .. } =
        operation
    else {
        return Err(CliError::run(format!("internal: operation {} is not a live mill", operation.name())));
    };
    let tool = job.tools.iter().find(|tool| tool.id == *tool_id).ok_or_else(|| CliError::run(format!("operation {name}: missing tool {tool_id}")))?;
    let ToolBody::Live { diameter: tool_diameter, .. } = &tool.body else {
        return Err(CliError::run(format!("operation {name}: tool {tool_id} is not a live tool")));
    };
    let contours = load_contours(name, job_path, drawing, layer.as_deref(), job.linear)?;
    let frame = match (*face, *y_axis) {
        (LiveFace::Od, _) => MillFrame::Od { diameter: *diameter },
        (LiveFace::Front, true) => MillFrame::FrontY { z_start: *z_start, c_deg: *c_deg },
        (LiveFace::Front, false) => MillFrame::Front { z_start: *z_start },
    };
    let y_limits = if *y_axis { job.head.y_min.zip(job.head.y_max) } else { None };
    let spec = MillSpec {
        name: name.clone(),
        tool_diameter: *tool_diameter,
        feed: *feed,
        depth: *depth,
        step_down: *step_down,
        clearance: job.clearance,
        tolerance: job.linear,
        strategy: *strategy,
        frame,
        contours,
        y_limits,
    };
    let program = build_mill(&spec)?;
    mill_blocks(name, *tool_id, *feed, *rpm, &program, previous)
}

/// Tool change, C engage, the clearance rapid, live tool on, interpolation, the cut, interpolation
/// off, live tool off, C disengage. The brake stays released: interpolation rejects a clamp.
fn mill_blocks(name: &str, tool_id: u32, feed: f64, rpm: f64, program: &MillProgram, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(program.approach.len().saturating_add(program.motions.len()).saturating_add(6));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: true }));
    for motion in &program.approach {
        blocks.push(ClBlock::Motion(motion.clone()));
    }
    blocks.push(ClBlock::Event(Event::LiveToolOn { rpm, direction: LiveDirection::Forward }));
    match program.interpolation {
        Some(MillInterp::Face) => blocks.push(ClBlock::Event(Event::FaceInterpolation { on: true })),
        Some(MillInterp::Cylinder { radius }) => blocks.push(ClBlock::Event(Event::CylinderInterpolation { radius: Some(radius) })),
        None => {}
    }
    for motion in &program.motions {
        blocks.push(ClBlock::Motion(motion.clone()));
    }
    match program.interpolation {
        Some(MillInterp::Face) => blocks.push(ClBlock::Event(Event::FaceInterpolation { on: false })),
        Some(MillInterp::Cylinder { .. }) => blocks.push(ClBlock::Event(Event::CylinderInterpolation { radius: None })),
        None => {}
    }
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: false }));
    let feeds = Feeds::try_new(feed, None, None, None, None).map_err(|err| geometry_err(name, err))?;
    ClPath::try_new(Some(name.to_string()), Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)
        .map_err(|err| geometry_err(name, err))
}

fn flat_block(step: &FlatStep, feed: f64) -> Result<ClBlock, CliError> {
    let pose = Pose::new(Vec3::new(step.x(), step.y(), step.z())?, Vec3::X)?.with_rotary(step.c_deg())?;
    let motion = match step {
        FlatStep::Rapid { .. } => ClMotion::Rapid { to: pose, feed: None },
        FlatStep::Feed { .. } => ClMotion::Linear { to: pose, feed, role: MoveRole::Cut },
    };
    Ok(ClBlock::Motion(motion))
}

fn live_pose(z: f64, x: f64, c_deg: f64, axis: Vec3) -> Result<Pose, CliError> {
    Ok(Pose::new(Vec3::new(x, 0.0, z)?, axis)?.with_rotary(c_deg)?)
}

/// Tool change when the id changes, then C engage, live tool forward, the cycle, live tool off,
/// C disengage. No turning-spindle [`Event::ProcessOn`]: the post rejects that while C is engaged.
fn with_live_events(motion: ClPath, tool_id: u32, rpm: f64, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(motion.blocks().len().saturating_add(5));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: true }));
    blocks.push(ClBlock::Event(Event::LiveToolOn { rpm, direction: LiveDirection::Forward }));
    blocks.extend(motion.blocks().iter().cloned());
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    blocks.push(ClBlock::Event(Event::CAxis { spindle: SPINDLE.to_string(), engage: false }));
    Ok(ClPath::try_new(motion.name().map(str::to_string), motion.tool_id(), motion.compensation(), motion.feeds(), None, Vec::new(), blocks)?)
}

fn with_events(motion: ClPath, tool_id: u32, previous: &mut Option<u32>) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(motion.blocks().len().saturating_add(3));
    if *previous != Some(tool_id) {
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id }));
        *previous = Some(tool_id);
    }
    blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() }));
    blocks.extend(motion.blocks().iter().cloned());
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }));
    Ok(ClPath::try_new(motion.name().map(str::to_string), motion.tool_id(), motion.compensation(), motion.feeds(), None, Vec::new(), blocks)?)
}

fn program_name(path: &Path) -> Option<String> {
    let stem = path.file_stem().and_then(|stem| stem.to_str())?;
    if stem.is_empty() || stem.len() > 256 { None } else { Some(stem.to_string()) }
}

fn emit(args: &LatheArgs, job: &LatheJob, doc: &Document, post: Option<&campanion_post::PostDef>) -> Result<ExitCode, CliError> {
    let report = campanion_ir::verify(doc.program());
    if !report.issues().is_empty() {
        let mut err_out = io::stderr().lock();
        err_out.write_all(crate::report::format_report(&report).as_bytes())?;
        return Ok(ExitCode::from(1));
    }
    let text = match post {
        Some(post) => crate::post::render(doc.program(), post, job.chord)?,
        None => campanion_ir::write_ngc(doc.program())?,
    };
    crate::run::write_output(&args.output, &text)?;
    if let Some(ir_path) = &args.ir {
        crate::run::write_output(ir_path, &doc.to_json_pretty()?)?;
    }
    Ok(ExitCode::SUCCESS)
}

fn array<'a>(value: &'a Value, what: &str, max: usize) -> Result<&'a [Value], CliError> {
    let Some(list) = value.as_array() else {
        return Err(CliError::invalid(format!("{what} must be an array")));
    };
    if list.len() > max {
        return Err(CliError::invalid(format!("too many {what}: {} > {max}", list.len())));
    }
    Ok(list)
}

fn positive(value: &Value, what: &str) -> Result<f64, CliError> {
    let number = as_f64(value, what)?;
    if number > 0.0 { Ok(number) } else { Err(CliError::invalid(format!("{what} must be positive"))) }
}

fn non_negative(value: &Value, what: &str) -> Result<f64, CliError> {
    let number = as_f64(value, what)?;
    if number >= 0.0 { Ok(number) } else { Err(CliError::invalid(format!("{what} must be >= 0"))) }
}

fn optional_non_negative(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, CliError> {
    match optional(map, key) {
        Some(value) => Ok(Some(non_negative(value, key)?)),
        None => Ok(None),
    }
}

fn optional_positive(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, CliError> {
    match optional(map, key) {
        Some(value) => Ok(Some(positive(value, key)?)),
        None => Ok(None),
    }
}

fn optional_u8(map: &Map<String, Value>, key: &str) -> Result<Option<u8>, CliError> {
    let Some(value) = optional(map, key) else {
        return Ok(None);
    };
    let number = as_f64(value, key)?;
    if !(0.0..=255.0).contains(&number) {
        return Err(CliError::invalid(format!("{key} is out of range")));
    }
    let rounded = number.round();
    if (number - rounded).abs() > 1e-6 {
        return Err(CliError::invalid(format!("{key} must be an integer")));
    }
    Ok(Some(rounded as u8))
}

fn show(err: impl std::fmt::Display) -> CliError {
    CliError::run(err.to_string())
}

fn op_fail(name: &str, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {name}: {err}"))
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use campanion_geom::{Line, Point, Segment};
    use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, Hand, Insert, InsertShape, Pose, Vec3};
    use campanion_turn::{FaceParams, FinishParams, Side, TipPoint, TurnMove, TurnProfile, TurnTool, face, finish};

    use super::{SourceKind, face_to_radius, parse_source, programmed_tip, source_kind};

    fn ok_source(value: serde_json::Value) -> super::Source {
        parse_source(&value).unwrap_or_else(|err| panic!("{err}"))
    }

    #[test]
    fn step_and_stp_sources_take_an_optional_body() {
        assert!(matches!(source_kind("Part.STEP").unwrap_or_else(|err| panic!("{err}")), SourceKind::Step { body: None }));
        assert!(matches!(source_kind("shaft.Stp").unwrap_or_else(|err| panic!("{err}")), SourceKind::Step { body: None }));
        let rejected = match source_kind("part.igs") {
            Ok(_) => panic!("igs was accepted"),
            Err(err) => err,
        };
        assert!(rejected.to_string().contains("source path must end in .dxf, .svg, .stl, .step or .stp"), "{rejected}");
        let named = ok_source(serde_json::json!({"path": "a.step", "body": "shaft"}));
        assert!(matches!(named.kind, SourceKind::Step { body: Some(name) } if name == "shaft"));
        let omitted = ok_source(serde_json::json!({"path": "a.stp"}));
        assert!(matches!(omitted.kind, SourceKind::Step { body: None }));
        let empty = match parse_source(&serde_json::json!({"path": "a.step", "body": ""})) {
            Ok(_) => panic!("empty body was accepted"),
            Err(err) => err,
        };
        assert!(empty.to_string().contains("body is empty"), "{empty}");
        let layer = match parse_source(&serde_json::json!({"path": "a.step", "layer": "0"})) {
            Ok(_) => panic!("layer was accepted"),
            Err(err) => err,
        };
        assert!(layer.to_string().contains("layer"), "{layer}");
    }

    fn close(actual: f64, expected: f64) -> bool {
        (actual - expected).abs() <= 1e-9
    }

    fn tip_ok(insert: &Insert, name: &str) -> TipPoint {
        match programmed_tip(insert, name) {
            Ok(tip) => tip,
            Err(err) => panic!("{name}: {err}"),
        }
    }

    fn cylinder(radius: f64, length: f64) -> TurnProfile {
        let start = Point::new(0.0, radius).expect("start");
        let end = Point::new(-length, radius).expect("end");
        let line = Line::new(start, end).expect("line");
        TurnProfile::try_new(vec![Segment::Line(line)], 1e-6).expect("profile")
    }

    #[test]
    fn imaginary_tip_uses_insert_offset_signs() {
        let nose = 0.8;
        let at_45 = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 45.0).expect("insert");
        let (dx, dz) = at_45.tip_offset();
        assert!(close(dx, -nose) && close(dz, -nose), "45° offset dx {dx} dz {dz}");
        assert_eq!(tip_ok(&at_45, "imaginary"), TipPoint::ImaginaryTip { dz, dx });
        assert_eq!(tip_ok(&at_45, "centre"), TipPoint::Centre);

        // 135°: sin > 0 and cos < 0, so (dx, dz) = (−nose, +nose). Swapping the pair is visible.
        let at_135 = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 135.0).expect("insert");
        let (dx, dz) = at_135.tip_offset();
        assert!(close(dx, -nose), "135° dx {dx}");
        assert!(close(dz, nose), "135° dz {dz}");
        let tip = tip_ok(&at_135, "imaginary");
        assert_eq!(tip, TipPoint::ImaginaryTip { dz, dx });

        let tool = TurnTool { nose_radius: nose, tip };
        let finished =
            finish(&cylinder(20.0, 30.0), &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 0.5, tol: 1e-6 })
                .expect("finish");
        let mut cursor = finished.start;
        let mut straight = None;
        for motion in &finished.moves {
            if let TurnMove::Feed { z, x } = *motion
                && (x - cursor.1).abs() <= 1e-6
                && (z - cursor.0).abs() > 10.0
                && (x - 20.0).abs() < 1e-4
            {
                straight = Some((cursor.0, z, x));
            }
            cursor = match *motion {
                TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => (z, x),
                TurnMove::Thread { z, x, .. } => (z, x),
                TurnMove::Dwell { .. } => cursor,
            };
        }
        let Some((z0, z1, x)) = straight else {
            panic!("imaginary finish straight is not on the part radius: {:?}", finished.moves);
        };
        assert!(close(x, 20.0) || (x - 20.0).abs() < 1e-4, "straight x {x}");
        assert!(z0.max(z1) > 0.2, "Z should shift by +nose, high end was {z0} {z1}");
    }

    #[test]
    fn facing_ends_one_nose_past_centre() {
        let nose = 0.8;
        let insert = Insert::try_new(InsertShape::C80, nose, None, Hand::Right, 135.0).expect("insert");
        let tools = [TurnTool { nose_radius: nose, tip: TipPoint::Centre }, TurnTool { nose_radius: nose, tip: tip_ok(&insert, "imaginary") }];
        for tool in tools {
            let path = face(
                &tool,
                &FaceParams { stock_front_z: 1.0, final_z: 0.0, step_z: 0.5, from_radius: 25.0, to_radius: face_to_radius(&tool), clearance: 2.0 },
            )
            .expect("face");
            assert!((path.start.1 - 27.0).abs() < 1e-9, "start radius {}", path.start.1);
            let mut cursor = path.start;
            let mut saw_end = false;
            for motion in &path.moves {
                if let TurnMove::Feed { z, x } = *motion
                    && (z - cursor.0).abs() <= 1e-9
                    && (x - cursor.1).abs() > 1.0
                    && x < 0.0
                {
                    assert!((x + nose).abs() < 1e-9, "face end radius {x}");
                    saw_end = true;
                }
                cursor = match *motion {
                    TurnMove::Rapid { z, x } | TurnMove::Feed { z, x } | TurnMove::Arc { z, x, .. } => (z, x),
                    TurnMove::Thread { z, x, .. } => (z, x),
                    TurnMove::Dwell { .. } => cursor,
                };
            }
            assert!(saw_end, "facing feed did not cross the centre: {:?}", path.moves);
        }
    }

    fn tailstock_job(extra: &str, operations: &str) -> String {
        format!(
            r#"{{
            "format": "campanion-lathe/1",
            "source": {{ "path": "shaft.dxf" }},
            "units": "mm",
            "stock": {{ "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 }},
            "spindle": {{ "max_rpm": 3000 }},
            {extra}
            "tools": [{{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }}],
            "operations": [{operations}],
            "clearance": 2
        }}"#
        )
    }

    fn parse_err(body: &str) -> String {
        match super::parse_job(body) {
            Ok(_) => panic!("job was accepted"),
            Err(err) => err.to_string(),
        }
    }

    fn unwrap_ok<T>(result: Result<T, super::CliError>, what: &str) -> T {
        result.unwrap_or_else(|err| panic!("{what}: {err}"))
    }

    #[test]
    fn tailstock_block_rejects_a_sub_spindle_a_missing_block_and_a_bad_action() {
        let advance = r#"{ "name": "in", "type": "tailstock", "action": "advance" }"#;
        let both = tailstock_job(r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } }, "tailstock": { "diameter": 12 },"#, advance);
        let err = parse_err(&both);
        assert!(err.contains("\"tailstock\" is not allowed with a \"sub\" block (the B axis is the secondary spindle)"), "{err}");

        let missing = parse_err(&tailstock_job("", advance));
        assert!(missing.contains("operation in: tailstock needs a \"tailstock\" block"), "{missing}");

        let bad = parse_err(&tailstock_job(r#""tailstock": { "diameter": 12 },"#, r#"{ "name": "in", "type": "tailstock", "action": "park" }"#));
        assert!(bad.contains("operation in: tailstock action must be advance or retract"), "{bad}");

        let job = unwrap_ok(super::parse_job(&tailstock_job(r#""tailstock": { "diameter": 12 },"#, advance)), "default clearance");
        let tailstock = job.tailstock.expect("block");
        assert!((tailstock.diameter - 12.0).abs() < 1e-12);
        assert!((tailstock.clearance - 1.0).abs() < 1e-12);
    }

    #[test]
    fn orient_and_sub_brake_parse_before_a_transfer_and_reject_bad_values() {
        let sub = r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } },"#;
        let ops = r#"{ "name": "aim", "type": "orient", "spindle": "sub", "angle_deg": 90 },
            { "name": "hold", "type": "sub_brake", "action": "clamp" },
            { "name": "free", "type": "sub_brake", "action": "release" },
            { "name": "handoff", "type": "transfer", "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 },
            { "name": "main_aim", "type": "orient", "spindle": "main", "angle_deg": 0 }"#;
        let job = unwrap_ok(super::parse_job(&tailstock_job(sub, ops)), "orient before transfer");
        assert!(matches!(
            job.operations[0],
            super::Operation::Orient { target: super::WorkSpindle::Sub, angle_deg, .. } if (angle_deg - 90.0).abs() < 1e-12
        ));
        assert!(job.operations[0].spindle() == super::WorkSpindle::Main);
        assert!(job.operations[0].uses_secondary_spindle());
        assert!(matches!(job.operations[1], super::Operation::SubBrake { clamp: true, .. }));
        assert!(job.operations[1].spindle() == super::WorkSpindle::Main);
        assert!(job.operations[1].uses_secondary_spindle());
        assert!(matches!(
            job.operations[4],
            super::Operation::Orient { target: super::WorkSpindle::Main, angle_deg, .. } if angle_deg.abs() < 1e-12
        ));

        let missing_orient = parse_err(&tailstock_job("", r#"{ "name": "aim", "type": "orient", "spindle": "sub", "angle_deg": 90 }"#));
        assert!(missing_orient.contains("operation aim: spindle sub needs a \"sub\" block"), "{missing_orient}");
        let missing_brake = parse_err(&tailstock_job("", r#"{ "name": "hold", "type": "sub_brake", "action": "clamp" }"#));
        assert!(missing_brake.contains("operation hold: sub_brake needs a \"sub\" block"), "{missing_brake}");
        let bad_action = parse_err(&tailstock_job(sub, r#"{ "name": "hold", "type": "sub_brake", "action": "park" }"#));
        assert!(bad_action.contains("operation hold: sub_brake action must be clamp or release"), "{bad_action}");
        let high = parse_err(&tailstock_job(sub, r#"{ "name": "aim", "type": "orient", "spindle": "sub", "angle_deg": 360 }"#));
        assert!(high.contains("operation aim: orient angle must be in 0..360"), "{high}");
        let low = parse_err(&tailstock_job("", r#"{ "name": "aim", "type": "orient", "spindle": "main", "angle_deg": -1 }"#));
        assert!(low.contains("operation aim: orient angle must be in 0..360"), "{low}");
        let unknown = parse_err(&tailstock_job(sub, r#"{ "name": "aim", "type": "orient", "spindle": "tail", "angle_deg": 10 }"#));
        assert!(unknown.contains("operation aim: unknown spindle `tail`"), "{unknown}");
        unwrap_ok(
            super::parse_job(&tailstock_job("", r#"{ "name": "aim", "type": "orient", "spindle": "main", "angle_deg": 359.5 }"#)),
            "main orient",
        );
    }

    #[test]
    fn tailstock_advance_and_retract_must_alternate_and_a_transfer_waits() {
        let block = r#""tailstock": { "diameter": 12, "clearance": 1 },"#;
        let twice = parse_err(&tailstock_job(
            block,
            r#"{ "name": "in", "type": "tailstock", "action": "advance" }, { "name": "again", "type": "tailstock", "action": "advance" }"#,
        ));
        assert!(twice.contains("operation again: tailstock is already advanced"), "{twice}");

        let early = parse_err(&tailstock_job(block, r#"{ "name": "out", "type": "tailstock", "action": "retract" }"#));
        assert!(early.contains("operation out: tailstock is not advanced"), "{early}");

        unwrap_ok(super::parse_job(&tailstock_job(block, r#"{ "name": "in", "type": "tailstock", "action": "advance" }"#)), "end advanced");

        let ops = [
            super::Operation::Tailstock { name: "in".into(), engage: true },
            super::Operation::Transfer {
                name: "handoff".into(),
                rpm: 0.0,
                phase: None,
                approach_z: 4.0,
                grip_z: -1.0,
                park_z: -50.0,
                feed: 200.0,
                dwell: 0.5,
                part_off: None,
            },
        ];
        let err = super::check_tailstock_ops(&ops, true, false).expect_err("transfer");
        assert!(err.to_string().contains("operation handoff: retract the tailstock first"), "{err}");

        let eject = [super::Operation::Tailstock { name: "in".into(), engage: true }, super::Operation::Eject { name: "drop".into() }];
        let err = super::check_tailstock_ops(&eject, true, false).expect_err("eject");
        assert!(err.to_string().contains("operation drop: retract the tailstock first"), "{err}");
    }

    fn rapid_at(x: f64, y: f64, z: f64) -> ClBlock {
        let pose = Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose");
        ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None })
    }

    fn path_of(blocks: Vec<ClBlock>) -> ClPath {
        let feeds = Feeds::try_new(1.0, None, None, None, None).expect("feeds");
        ClPath::try_new(Some("cut".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path")
    }

    #[test]
    fn tailstock_zone_allows_the_exact_radius_and_rejects_the_inside() {
        let tailstock = super::TailstockBlock { diameter: 18.0, clearance: 1.0 };
        let limit = tailstock.diameter * 0.5 + tailstock.clearance;
        assert!((limit - 10.0).abs() < 1e-12);
        unwrap_ok(super::check_tailstock_zone("cut", &path_of(vec![rapid_at(limit, 0.0, 0.0)]), &tailstock, None), "exact radius passes");
        unwrap_ok(
            super::check_tailstock_zone("cut", &path_of(vec![rapid_at(0.0, 0.0, -tailstock.clearance)]), &tailstock, None),
            "z = -clearance passes",
        );

        let inside = limit - 0.001;
        let err = super::check_tailstock_zone("cut", &path_of(vec![rapid_at(inside, 0.0, -0.25)]), &tailstock, None).expect_err("inside");
        let expected = format!("operation cut: move at X{:.3} Z{:.3} enters the tailstock zone", inside * 2.0, -0.25);
        assert!(err.to_string().contains(&expected), "{err}");

        let face = vec![ClBlock::Event(Event::FaceInterpolation { on: true }), rapid_at(0.0, limit, 0.0)];
        unwrap_ok(super::check_tailstock_zone("face", &path_of(face), &tailstock, None), "face radius on the boundary passes");
        let face_in = vec![ClBlock::Event(Event::FaceInterpolation { on: true }), rapid_at(0.0, inside, 0.0)];
        let err = super::check_tailstock_zone("face", &path_of(face_in), &tailstock, None).expect_err("face inside");
        assert!(err.to_string().contains("operation face: move at X"), "{err}");

        let cylinder = vec![ClBlock::Event(Event::CylinderInterpolation { radius: Some(20.0) }), rapid_at(limit, 80.0, 0.0)];
        unwrap_ok(super::check_tailstock_zone("wrap", &path_of(cylinder), &tailstock, None), "unrolled Y is not a radius");
    }

    #[test]
    fn tailstock_zone_checks_whole_moves_not_only_their_end_points() {
        let tailstock = super::TailstockBlock { diameter: 18.0, clearance: 1.0 };
        // Both ends are outside (Z-5 below the zone, then X30 outside the radius) but the diagonal
        // rapid cuts the zone corner.
        let diagonal = vec![rapid_at(0.0, 0.0, -5.0), rapid_at(15.0, 0.0, 5.0)];
        let err = super::check_tailstock_zone("approach", &path_of(diagonal), &tailstock, None).expect_err("diagonal");
        assert!(err.to_string().contains("operation approach: move at X12.000 Z-1.000 enters the tailstock zone"), "{err}");

        // The same move with the corner kept clear passes.
        let clear = vec![rapid_at(0.0, 0.0, -5.0), rapid_at(15.0, 0.0, -5.0), rapid_at(15.0, 0.0, 5.0)];
        unwrap_ok(super::check_tailstock_zone("approach", &path_of(clear), &tailstock, None), "square approach passes");

        // A move along z = -clearance touches the zone boundary only.
        let along = vec![rapid_at(30.0, 0.0, -1.0), rapid_at(0.0, 0.0, -1.0)];
        unwrap_ok(super::check_tailstock_zone("face", &path_of(along), &tailstock, None), "boundary move passes");

        // The first move starts where the previous operation ended.
        let first = vec![rapid_at(15.0, 0.0, 5.0)];
        let start = Vec3::new(0.0, 0.0, -5.0).expect("start");
        let err = super::check_tailstock_zone("next", &path_of(first.clone()), &tailstock, Some(start)).expect_err("carried start");
        assert!(err.to_string().contains("operation next: move at X"), "{err}");
        unwrap_ok(super::check_tailstock_zone("next", &path_of(first), &tailstock, None), "no start passes");

        // A face-interpolated move through the centre is caught even though both ends are at radius 12.
        let face = vec![ClBlock::Event(Event::FaceInterpolation { on: true }), rapid_at(12.0, 0.0, 0.0), rapid_at(-12.0, 0.0, 0.0)];
        let err = super::check_tailstock_zone("face", &path_of(face), &tailstock, None).expect_err("through centre");
        assert!(err.to_string().contains("move at X0.000 Z0.000"), "{err}");
    }

    #[test]
    fn a_centre_drill_and_an_exact_finish_follow_the_zone() {
        let block = r#""tailstock": { "diameter": 18, "clearance": 1 },"#;
        let drill = tailstock_job(
            block,
            r#"{ "name": "in", "type": "tailstock", "action": "advance" },
              { "name": "centre", "type": "drill", "tool_id": 1, "z_start": 0, "depth": 5, "diameter": 4, "feed": 80, "rpm": 800 }"#,
        );
        let job = unwrap_ok(super::parse_job(&drill), "drill job");
        let err = super::build_document(Path::new("job.lathe.json"), &job, &cylinder(15.0, 30.0), None).expect_err("drill enters");
        assert!(err.to_string().contains("operation centre: move at X0.000 Z2.000 enters the tailstock zone"), "{err}");

        let finish = tailstock_job(
            block,
            r#"{ "name": "in", "type": "tailstock", "action": "advance" },
              { "name": "finish", "type": "finish", "tool_id": 1, "side": "outside", "feed": 120, "speed": { "rpm": 800 } }"#,
        );
        let job = unwrap_ok(super::parse_job(&finish), "finish job");
        // Profile radius 10 equals D/2 + clearance. The finish stays on that radius.
        unwrap_ok(super::build_document(Path::new("job.lathe.json"), &job, &cylinder(10.0, 20.0), None), "exact radius finish passes");
    }

    #[test]
    fn steady_rest_block_rejects_a_sub_spindle_a_missing_block_and_a_bad_action() {
        let clamp = r#"{ "name": "in", "type": "steady_rest", "action": "clamp" }"#;
        let both = tailstock_job(r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } }, "steady_rest": { "z": -20, "width": 8 },"#, clamp);
        let err = parse_err(&both);
        assert!(err.contains("\"steady_rest\" is not allowed with a \"sub\" block"), "{err}");

        let missing = parse_err(&tailstock_job("", clamp));
        assert!(missing.contains("operation in: steady_rest needs a \"steady_rest\" block"), "{missing}");

        let bad =
            parse_err(&tailstock_job(r#""steady_rest": { "z": -20, "width": 8 },"#, r#"{ "name": "in", "type": "steady_rest", "action": "park" }"#));
        assert!(bad.contains("operation in: steady_rest action must be clamp or unclamp"), "{bad}");

        let unknown = parse_err(&tailstock_job(r#""steady_rest": { "z": -20, "width": 8, "p": 1 },"#, clamp));
        assert!(unknown.contains("unknown steady_rest field `p`"), "{unknown}");

        let forward = parse_err(&tailstock_job(r#""steady_rest": { "z": 0, "width": 8 },"#, clamp));
        assert!(forward.contains("z must be < 0"), "{forward}");

        let job = unwrap_ok(super::parse_job(&tailstock_job(r#""steady_rest": { "z": -20, "width": 8 },"#, clamp)), "default clearance");
        let rest = job.steady_rest.expect("block");
        assert!((rest.z + 20.0).abs() < 1e-12);
        assert!((rest.width - 8.0).abs() < 1e-12);
        assert!((rest.clearance - 1.0).abs() < 1e-12);
    }

    #[test]
    fn steady_rest_clamp_and_unclamp_must_alternate_and_a_transfer_or_part_waits() {
        let block = r#""steady_rest": { "z": -20, "width": 8, "clearance": 1 },"#;
        let twice = parse_err(&tailstock_job(
            block,
            r#"{ "name": "in", "type": "steady_rest", "action": "clamp" }, { "name": "again", "type": "steady_rest", "action": "clamp" }"#,
        ));
        assert!(twice.contains("operation again: steady rest is already clamped"), "{twice}");

        let early = parse_err(&tailstock_job(block, r#"{ "name": "out", "type": "steady_rest", "action": "unclamp" }"#));
        assert!(early.contains("operation out: steady rest is not clamped"), "{early}");

        unwrap_ok(super::parse_job(&tailstock_job(block, r#"{ "name": "in", "type": "steady_rest", "action": "clamp" }"#)), "end clamped");

        let parted = parse_err(&tailstock_job(
            block,
            r#"{ "name": "in", "type": "steady_rest", "action": "clamp" }, { "name": "cutoff", "type": "part", "tool_id": 1, "z": -20, "clearance": 1, "feed": 60, "speed": { "rpm": 400 } }"#,
        ));
        assert!(parted.contains("operation cutoff: unclamp the steady rest before parting off"), "{parted}");

        let ops = [
            super::Operation::SteadyRest { name: "in".into(), clamp: true },
            super::Operation::Transfer {
                name: "handoff".into(),
                rpm: 0.0,
                phase: None,
                approach_z: 4.0,
                grip_z: -1.0,
                park_z: -50.0,
                feed: 200.0,
                dwell: 0.5,
                part_off: None,
            },
        ];
        let err = super::check_steady_rest_ops(&ops, true).expect_err("transfer");
        assert!(err.to_string().contains("operation handoff: unclamp the steady rest first"), "{err}");

        let eject = [super::Operation::SteadyRest { name: "in".into(), clamp: true }, super::Operation::Eject { name: "drop".into() }];
        let err = super::check_steady_rest_ops(&eject, true).expect_err("eject");
        assert!(err.to_string().contains("operation drop: unclamp the steady rest first"), "{err}");
    }

    #[test]
    fn steady_rest_band_includes_the_edge_and_a_segment_that_only_crosses_it() {
        let rest = super::SteadyRestBlock { z: -40.0, width: 10.0, clearance: 1.0 };
        let (z_lo, z_hi) = super::steady_rest_band(&rest);
        assert!((z_lo + 46.0).abs() < 1e-12 && (z_hi + 34.0).abs() < 1e-12, "{z_lo} {z_hi}");

        let on_edge = vec![rapid_at(10.0, 0.0, z_lo - 10.0), rapid_at(12.0, 0.0, z_lo)];
        let err = super::check_steady_rest_zone("cut", &path_of(on_edge), &rest, None).expect_err("edge");
        let expected = format!("operation cut: move at X{:.3} Z{:.3} crosses the steady rest band", 24.0, z_lo);
        assert!(err.to_string().contains(&expected), "{err}");

        let clear = vec![rapid_at(10.0, 0.0, z_lo - 10.0), rapid_at(12.0, 0.0, z_lo - 0.001)];
        unwrap_ok(super::check_steady_rest_zone("cut", &path_of(clear), &rest, None), "just outside the band passes");

        let cross = vec![rapid_at(4.0, 0.0, z_lo - 5.0), rapid_at(9.0, 0.0, z_hi + 1.0)];
        let err = super::check_steady_rest_zone("cross", &path_of(cross), &rest, None).expect_err("crossing");
        let expected = format!("operation cross: move at X{:.3} Z{:.3} crosses the steady rest band", 18.0, z_hi + 1.0);
        assert!(err.to_string().contains(&expected), "{err}");

        let start = Vec3::new(5.0, 0.0, rest.z).expect("start");
        let away = vec![rapid_at(20.0, 0.0, z_lo - 10.0)];
        let err = super::check_steady_rest_zone("next", &path_of(away), &rest, Some(start)).expect_err("carried start");
        let expected = format!("operation next: move at X{:.3} Z{:.3} crosses the steady rest band", 10.0, rest.z);
        assert!(err.to_string().contains(&expected), "{err}");
    }

    /// Stock Ø40, part `z` -10, insert width 3, so the caught length is 13.
    fn catcher_job(extra: &str, part_extra: &str) -> String {
        format!(
            r#"{{
            "format": "campanion-lathe/1",
            "source": {{ "path": "shaft.dxf" }},
            "units": "mm",
            "stock": {{ "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 }},
            "spindle": {{ "max_rpm": 3000 }},
            {extra}
            "tools": [{{ "id": 1, "station": 1, "shape": "groove", "width": 3, "nose_radius": 0.2, "orientation_deg": 90, "hand": "neutral", "tip": "centre" }}],
            "operations": [{{ "name": "cutoff", "type": "part", "tool_id": 1, "z": -10, "clearance": 1, "feed": 60, "speed": {{ "rpm": 400 }}{part_extra} }}],
            "clearance": 2
        }}"#
        )
    }

    fn catcher_events(path: &ClPath) -> Vec<bool> {
        path.blocks()
            .iter()
            .filter_map(|block| match block {
                ClBlock::Event(Event::PartsCatcher { extend }) => Some(*extend),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn parts_catcher_block_uses_the_same_field_errors_as_tailstock() {
        let unknown = parse_err(&catcher_job(r#""parts_catcher": { "max_diameter": 76, "max_length": 102, "weight": 3.6 },"#, ""));
        assert!(unknown.contains("unknown parts_catcher field `weight`"), "{unknown}");

        let zero = parse_err(&catcher_job(r#""parts_catcher": { "max_diameter": 0, "max_length": 102 },"#, ""));
        assert!(zero.contains("max_diameter must be positive"), "{zero}");

        let missing = parse_err(&catcher_job(r#""parts_catcher": { "max_diameter": 76 },"#, ""));
        assert!(missing.contains("missing `max_length`"), "{missing}");

        let negative = parse_err(&catcher_job(r#""parts_catcher": { "max_diameter": 76, "max_length": -1 },"#, ""));
        assert!(negative.contains("max_length must be positive"), "{negative}");
    }

    #[test]
    fn catch_needs_a_parts_catcher_block_and_a_retracted_tailstock() {
        let missing = parse_err(&catcher_job("", r#", "catch": true"#));
        assert!(missing.contains("operation cutoff: catch needs a \"parts_catcher\" block"), "{missing}");

        let advanced = r#"{
            "format": "campanion-lathe/1",
            "source": { "path": "shaft.dxf" },
            "units": "mm",
            "stock": { "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 },
            "spindle": { "max_rpm": 3000 },
            "tailstock": { "diameter": 12 },
            "parts_catcher": { "max_diameter": 76, "max_length": 102 },
            "tools": [{ "id": 1, "station": 1, "shape": "groove", "width": 3, "nose_radius": 0.2, "orientation_deg": 90, "hand": "neutral", "tip": "centre" }],
            "operations": [
                { "name": "in", "type": "tailstock", "action": "advance" },
                { "name": "cutoff", "type": "part", "tool_id": 1, "z": -10, "clearance": 1, "feed": 60, "speed": { "rpm": 400 }, "catch": true }
            ],
            "clearance": 2
        }"#
        .to_string();
        let err = parse_err(&advanced);
        assert!(err.contains("operation cutoff: retract the tailstock before catching the part"), "{err}");

        let retracted = advanced.replace(
            r#"{ "name": "in", "type": "tailstock", "action": "advance" },"#,
            r#"{ "name": "in", "type": "tailstock", "action": "advance" }, { "name": "out", "type": "tailstock", "action": "retract" },"#,
        );
        unwrap_ok(super::parse_job(&retracted), "retract then catch");

        let transfer = r#"{
            "format": "campanion-lathe/1",
            "source": { "path": "shaft.dxf" },
            "units": "mm",
            "stock": { "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 },
            "spindle": { "max_rpm": 3000 },
            "sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } },
            "parts_catcher": { "max_diameter": 76, "max_length": 102 },
            "tools": [{ "id": 1, "station": 1, "shape": "groove", "width": 3, "nose_radius": 0.2, "orientation_deg": 90, "hand": "neutral", "tip": "centre" }],
            "operations": [{ "name": "handoff", "type": "transfer", "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200,
                "part_off": { "tool_id": 1, "z": -10, "clearance": 1, "feed": 60, "speed": { "rpm": 400 }, "catch": true } }],
            "clearance": 2
        }"#
        .to_string();
        let err = parse_err(&transfer);
        assert!(err.contains("unknown part_off field `catch`"), "{err}");
    }

    #[test]
    fn part_size_must_fit_the_catcher_and_the_exact_length_passes() {
        let job = unwrap_ok(
            super::parse_job(&catcher_job(r#""parts_catcher": { "max_diameter": 39, "max_length": 13 },"#, r#", "catch": true"#)),
            "fat job",
        );
        let err = super::build_document(Path::new("job.lathe.json"), &job, &cylinder(15.0, 30.0), None).expect_err("diameter");
        assert!(err.to_string().contains("operation cutoff: part diameter 40.000 exceeds parts_catcher.max_diameter 39.000"), "{err}");

        let long = unwrap_ok(
            super::parse_job(&catcher_job(r#""parts_catcher": { "max_diameter": 40, "max_length": 12 },"#, r#", "catch": true"#)),
            "long job",
        );
        let err = super::build_document(Path::new("job.lathe.json"), &long, &cylinder(15.0, 30.0), None).expect_err("length");
        assert!(err.to_string().contains("operation cutoff: part length 13.000 exceeds parts_catcher.max_length 12.000"), "{err}");

        let exact =
            unwrap_ok(super::parse_job(&catcher_job(r#""parts_catcher": { "max_diameter": 40, "max_length": 13 },"#, r#", "catch": true"#)), "exact");
        let (doc, gouges) = unwrap_ok(super::build_document(Path::new("job.lathe.json"), &exact, &cylinder(15.0, 30.0), None), "length equal to L");
        assert!(gouges.is_empty());
        let path = doc.program().toolpaths().last().expect("part path");
        assert_eq!(catcher_events(path), vec![true, false]);
        let blocks = path.blocks();
        let extend_at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::PartsCatcher { extend: true }))).expect("extend");
        let retract_at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::PartsCatcher { extend: false }))).expect("retract");
        let (before, after) = match (&blocks[extend_at - 1], &blocks[extend_at + 1]) {
            (ClBlock::Motion(before @ ClMotion::Rapid { .. }), ClBlock::Motion(after @ ClMotion::Linear { .. })) => (before, after),
            _ => panic!("extend is not between the approach rapid and a feed"),
        };
        assert!(super::motion_tip(after).x() < super::motion_tip(before).x() - 1e-9, "first feed moves toward X");
        assert!(matches!(blocks[retract_at - 1], ClBlock::Motion(_)), "retract follows the last motion");
        assert!(matches!(blocks.last(), Some(ClBlock::Event(Event::ProcessOff { .. }))));
        assert_eq!(retract_at + 1, blocks.len() - 1);

        let plain = unwrap_ok(super::parse_job(&catcher_job("", "")), "no catch");
        match &plain.operations[0] {
            super::Operation::Part { catch: false, .. } => {}
            _ => panic!("catch defaulted to true"),
        }
        let (doc, _) = unwrap_ok(super::build_document(Path::new("job.lathe.json"), &plain, &cylinder(15.0, 30.0), None), "uncatched part");
        assert!(catcher_events(doc.program().toolpaths().last().expect("part")).is_empty());
    }

    fn bar_job(extra: &str, operations: &str) -> String {
        format!(
            r#"{{
            "format": "campanion-lathe/1",
            "source": {{ "path": "shaft.dxf" }},
            "units": "mm",
            "stock": {{ "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 }},
            "spindle": {{ "max_rpm": 3000 }},
            {extra}
            "tools": [{{ "id": 1, "station": 1, "shape": "groove", "width": 3, "nose_radius": 0.2, "orientation_deg": 90, "hand": "neutral", "tip": "centre" }}],
            "operations": [{operations}],
            "clearance": 2
        }}"#
        )
    }

    const PART: &str = r#"{ "name": "cutoff", "type": "part", "tool_id": 1, "z": -10, "clearance": 1, "feed": 60, "speed": { "rpm": 400 } }"#;
    const FEED: &str = r#"{ "name": "feed", "type": "bar_feed" }"#;

    #[test]
    fn bar_feed_must_be_last_after_a_part_and_not_with_a_sub_or_an_advanced_tailstock() {
        let later = bar_job("", &format!("{PART}, {FEED}, {{ \"name\": \"later\", \"type\": \"tailstock\", \"action\": \"advance\" }}"));
        let not_last = parse_err(&later);
        assert!(not_last.contains("operation feed: bar_feed must be the last operation"), "{not_last}");

        let twice = bar_job("", &format!(r#"{PART}, {FEED}, {{ "name": "again", "type": "bar_feed" }}"#));
        let two = parse_err(&twice);
        assert!(two.contains("operation feed: bar_feed must be the last operation"), "{two}");
        assert!(!two.contains("operation again"), "{two}");

        let alone = parse_err(&bar_job("", FEED));
        assert!(alone.contains("operation feed: bar_feed needs a part operation before it"), "{alone}");

        let handoff = bar_job(
            r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } },"#,
            r#"{ "name": "handoff", "type": "transfer", "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200,
                "part_off": { "tool_id": 1, "z": -10, "clearance": 1, "feed": 60, "speed": { "rpm": 400 } } },
              { "name": "feed", "type": "bar_feed" }"#,
        );
        let nested = parse_err(&handoff);
        assert!(nested.contains("operation feed: bar_feed needs a part operation before it"), "{nested}");

        let with_sub = bar_job(r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } },"#, &format!("{PART}, {FEED}"));
        let sub = parse_err(&with_sub);
        assert!(sub.contains("operation feed: bar_feed is not supported with a \"sub\" block"), "{sub}");

        let advanced = bar_job(
            r#""tailstock": { "diameter": 12 },"#,
            &format!(r#"{PART}, {{ "name": "in", "type": "tailstock", "action": "advance" }}, {FEED}"#),
        );
        let quill = parse_err(&advanced);
        assert!(quill.contains("operation feed: retract the tailstock first"), "{quill}");

        let zero = parse_err(&bar_job("", &format!(r#"{PART}, {{ "name": "feed", "type": "bar_feed", "part_length": 0 }}"#)));
        assert!(zero.contains("part_length must be positive"), "{zero}");
        let negative = parse_err(&bar_job("", &format!(r#"{PART}, {{ "name": "feed", "type": "bar_feed", "min_clamp": -1 }}"#)));
        assert!(negative.contains("min_clamp must be positive"), "{negative}");
        let unknown = parse_err(&bar_job("", &format!(r#"{PART}, {{ "name": "feed", "type": "bar_feed", "push": 1 }}"#)));
        assert!(unknown.contains("unknown operation field `push`"), "{unknown}");
    }

    #[test]
    fn bar_feed_stores_millimetre_overrides_on_one_event() {
        let body = bar_job("", &format!(r#"{PART}, {{ "name": "feed", "type": "bar_feed", "part_length": 30, "min_clamp": 12 }}"#));
        let job = unwrap_ok(super::parse_job(&body), "bar feed");
        let (doc, gouges) = unwrap_ok(super::build_document(Path::new("job.lathe.json"), &job, &cylinder(15.0, 30.0), None), "document");
        assert!(gouges.is_empty());
        let path = doc.program().toolpaths().last().expect("feed path");
        assert_eq!(path.tool_id(), None);
        let [ClBlock::Event(Event::BarFeed { part_length: Some(length), min_clamp: Some(clamp) })] = path.blocks() else {
            panic!("feed path is not one bar-feed event: {:?}", path.blocks());
        };
        assert!(close(*length, 30.0) && close(*clamp, 12.0), "length {length} clamp {clamp}");
        let binding = doc.program().turning().expect("turning").bindings().last().expect("binding");
        assert_eq!(binding.spindle(), "main");
        assert_eq!(binding.channel(), 0);
        assert_eq!(binding.toolpath() as usize, doc.program().toolpaths().len() - 1);
    }

    fn machine_shell(spindle_rpm: f64, machine: &str, extra: &str, tools: &str, operations: &str) -> String {
        format!(
            r#"{{
            "format": "campanion-lathe/1",
            "source": {{ "path": "shaft.dxf" }},
            "units": "mm",
            "stock": {{ "kind": "bar", "diameter": 40, "length": 40, "front_allowance": 1 }},
            "spindle": {{ "max_rpm": {spindle_rpm} }},
            {machine}
            {extra}
            "tools": [{tools}],
            "operations": [{operations}],
            "clearance": 2
        }}"#
        )
    }

    const TURN_TOOL: &str =
        r#"{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }"#;
    const FACE_OP: &str = r#"{ "name": "face", "type": "face", "tool_id": 1, "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }"#;

    #[test]
    fn machine_block_is_optional_and_unknown_models_are_rejected() {
        unwrap_ok(super::parse_job(&machine_shell(4_500.0, "", "", TURN_TOOL, FACE_OP)), "no machine");
        let unknown = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_st10" },"#, "", TURN_TOOL, FACE_OP));
        assert!(unknown.contains("machine.model must be haas_st15y, haas_st20y or haas_ds30y"), "{unknown}");
        let extra = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_st20y", "serial": 1 },"#, "", TURN_TOOL, FACE_OP));
        assert!(extra.contains("unknown machine field `serial`"), "{extra}");
    }

    #[test]
    fn machine_spindle_sub_sync_and_y_pass_at_the_limit() {
        let at = machine_shell(
            4_000.0,
            r#""machine": { "model": "haas_st20y", "sub_spindle": true },"#,
            r#""sub": { "max_rpm": 6000, "source": { "path": "shaft.dxf" } },"#,
            TURN_TOOL,
            r#"{ "name": "handoff", "type": "transfer", "rpm": 4800, "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 }"#,
        );
        unwrap_ok(super::parse_job(&at), "at the limits");

        let over = machine_shell(4_000.001, r#""machine": { "model": "haas_st20y" },"#, "", TURN_TOOL, FACE_OP);
        let message = parse_err(&over);
        assert!(message.contains("machine haas_st20y: main spindle max_rpm 4000.001 exceeds 4000"), "{message}");

        let sub_over = machine_shell(
            4_000.0,
            r#""machine": { "model": "haas_st20y", "sub_spindle": true },"#,
            r#""sub": { "max_rpm": 6000.001, "source": { "path": "shaft.dxf" } },"#,
            TURN_TOOL,
            r#"{ "name": "handoff", "type": "transfer", "rpm": 1000, "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 }"#,
        );
        let message = parse_err(&sub_over);
        assert!(message.contains("machine haas_st20y: sub spindle max_rpm 6000.001 exceeds 6000"), "{message}");

        let sync_over = machine_shell(
            4_000.0,
            r#""machine": { "model": "haas_st15y", "sub_spindle": true },"#,
            r#""sub": { "max_rpm": 6000, "source": { "path": "shaft.dxf" } },"#,
            TURN_TOOL,
            r#"{ "name": "handoff", "type": "transfer", "rpm": 4800.001, "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 }"#,
        );
        let message = parse_err(&sync_over);
        assert!(message.contains("machine haas_st15y: operation handoff: synchronized rpm 4800.001 exceeds 4800"), "{message}");

        let y_ok = machine_shell(
            3_000.0,
            r#""machine": { "model": "haas_ds30y" },"#,
            r#""head": { "y_axis": true, "y_min": -51, "y_max": 51 },"#,
            TURN_TOOL,
            FACE_OP,
        );
        let job = unwrap_ok(super::parse_job(&y_ok), "y at the limit");
        assert!(job.machine.is_some_and(|machine| machine.sub_spindle), "ds sub-spindle defaults on");

        let y_over = machine_shell(
            3_000.0,
            r#""machine": { "model": "haas_st20y" },"#,
            r#""head": { "y_axis": true, "y_min": -51, "y_max": 51.001 },"#,
            TURN_TOOL,
            FACE_OP,
        );
        let message = parse_err(&y_over);
        assert!(message.contains("machine haas_st20y: head Y travel [-51, 51.001] exceeds [-51, 51]"), "{message}");
    }

    #[test]
    fn live_upgrade_and_missing_options_use_the_profile() {
        let live_tool = r#"{ "id": 2, "station": 2, "live": "axial", "kind": "drill", "diameter": 6, "max_rpm": 4000 }"#;
        let holes = r#"{ "name": "holes", "type": "live_drill", "tool_id": 2, "face": "front", "z_start": 0, "positions": [{ "c": 0, "r": 10 }], "depth": 5, "rpm": 4000, "feed": 80 }"#;
        let at = machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, "", live_tool, holes)
            .replace(r#""spindle": { "max_rpm": 3000 },"#, r#""spindle": { "max_rpm": 3000, "c_axis": true },"#);
        unwrap_ok(super::parse_job(&at), "live at 4000");

        let fast_tool = live_tool.replace("4000", "4000.001");
        let tool_over = machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, "", &fast_tool, holes)
            .replace(r#""spindle": { "max_rpm": 3000 },"#, r#""spindle": { "max_rpm": 3000, "c_axis": true },"#);
        let message = parse_err(&tool_over);
        assert!(message.contains("machine haas_st15y: tool 2: live tool max_rpm 4000.001 exceeds 4000"), "{message}");

        // The tool cap is checked first, so this operation uses a live tool with no max_rpm.
        let uncapped = r#"{ "id": 2, "station": 2, "live": "axial", "kind": "drill", "diameter": 6 }"#;
        let fast_op = holes.replace("\"rpm\": 4000", "\"rpm\": 4000.001");
        let op_over = machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, "", uncapped, &fast_op)
            .replace(r#""spindle": { "max_rpm": 3000 },"#, r#""spindle": { "max_rpm": 3000, "c_axis": true },"#);
        let message = parse_err(&op_over);
        assert!(message.contains("machine haas_st15y: operation holes: live tool rpm 4000.001 exceeds 4000"), "{message}");

        let upgraded = holes.replace("\"rpm\": 4000", "\"rpm\": 6000");
        let tool = live_tool.replace("4000", "6000");
        let raised = machine_shell(3_000.0, r#""machine": { "model": "haas_st20y", "live_tool_upgrade": true },"#, "", &tool, &upgraded)
            .replace(r#""spindle": { "max_rpm": 3000 },"#, r#""spindle": { "max_rpm": 3000, "c_axis": true },"#);
        unwrap_ok(super::parse_job(&raised), "6000 with the upgrade");

        let ds = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_ds30y", "live_tool_upgrade": true },"#, "", TURN_TOOL, FACE_OP));
        assert!(ds.contains("machine.live_tool_upgrade: haas_ds30y has no 6000 rpm live tool option"), "{ds}");
    }

    #[test]
    fn sub_and_tailstock_options_gate_operations_and_the_ds_has_no_tailstock() {
        let handoff = r#"{ "name": "handoff", "type": "transfer", "approach_z": 4, "grip_z": -1, "park_z": -50, "feed": 200 }"#;
        let sub = r#""sub": { "max_rpm": 2000, "source": { "path": "shaft.dxf" } },"#;
        let bare = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, sub, TURN_TOOL, handoff));
        assert!(bare.contains("operation handoff: haas_st15y has no sub-spindle (machine.sub_spindle is false)"), "{bare}");

        let fitted = machine_shell(3_000.0, r#""machine": { "model": "haas_st20y", "sub_spindle": true },"#, sub, TURN_TOOL, handoff);
        let job = unwrap_ok(super::parse_job(&fitted), "sub option");
        assert!(super::counts_toward_travel(&job.operations[0]), "a transfer is main-frame");

        let early = r#"{ "name": "back", "type": "face", "tool_id": 1, "spindle": "sub", "step_z": 0.5, "feed": 100, "speed": { "rpm": 800 } }"#;
        let before = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, sub, TURN_TOOL, early));
        assert!(before.contains("operation back: haas_st15y has no sub-spindle (machine.sub_spindle is false)"), "{before}");

        let advance = r#"{ "name": "in", "type": "tailstock", "action": "advance" }"#;
        let no_option =
            parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_st20y" },"#, r#""tailstock": { "diameter": 12 },"#, TURN_TOOL, advance));
        assert!(no_option.contains("operation in: machine has no tailstock (machine.tailstock is false)"), "{no_option}");

        let with_option = machine_shell(
            3_000.0,
            r#""machine": { "model": "haas_st15y", "tailstock": true },"#,
            r#""tailstock": { "diameter": 12 },"#,
            TURN_TOOL,
            advance,
        );
        unwrap_ok(super::parse_job(&with_option), "tailstock option");

        let ds_block =
            parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_ds30y" },"#, r#""tailstock": { "diameter": 12 },"#, TURN_TOOL, FACE_OP));
        assert!(ds_block.contains("machine haas_ds30y: haas_ds30y has no tailstock (the B axis is the secondary spindle)"), "{ds_block}");
        let ds_flag = parse_err(&machine_shell(3_000.0, r#""machine": { "model": "haas_ds30y", "tailstock": true },"#, "", TURN_TOOL, FACE_OP));
        assert!(ds_flag.contains("machine haas_ds30y: haas_ds30y has no tailstock (the B axis is the secondary spindle)"), "{ds_flag}");

        let both = parse_err(&machine_shell(
            3_000.0,
            r#""machine": { "model": "haas_st20y", "sub_spindle": true, "tailstock": true },"#,
            "",
            TURN_TOOL,
            FACE_OP,
        ));
        assert!(both.contains("haas_st20y: the sub-spindle option is not compatible with the tailstock option (Haas ST-20Y spec page)"), "{both}");
    }

    #[test]
    fn program_extents_use_main_frame_tip_radii() {
        let wide = machine_shell(3_000.0, r#""machine": { "model": "haas_st15y" },"#, "", TURN_TOOL, FACE_OP)
            .replace(r#""diameter": 40"#, r#""diameter": 500"#);
        let job = unwrap_ok(super::parse_job(&wide), "wide face");
        let err = match super::build_document(Path::new("job.lathe.json"), &job, &cylinder(15.0, 20.0), None) {
            Err(err) => err.to_string(),
            Ok(_) => panic!("x travel was accepted"),
        };
        assert!(err.contains("machine haas_st15y: program X extent"), "{err}");
        assert!(err.contains("exceeds the X travel 200.000"), "{err}");

        let long = r#"{
            "format": "campanion-lathe/1",
            "source": { "path": "shaft.dxf" },
            "units": "mm",
            "stock": { "kind": "bar", "diameter": 30, "length": 520, "front_allowance": 1 },
            "spindle": { "max_rpm": 3000 },
            "machine": { "model": "haas_st15y" },
            "tools": [{ "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.4, "orientation_deg": 45, "hand": "right", "tip": "centre" }],
            "operations": [{ "name": "finish", "type": "finish", "tool_id": 1, "side": "outside", "feed": 100, "speed": { "rpm": 800 } }],
            "clearance": 2
        }"#;
        let job = unwrap_ok(super::parse_job(long), "long finish");
        let err = match super::build_document(Path::new("job.lathe.json"), &job, &cylinder(15.0, 500.0), None) {
            Err(err) => err.to_string(),
            Ok(_) => panic!("z travel was accepted"),
        };
        assert!(err.contains("machine haas_st15y: program Z extent"), "{err}");
        assert!(err.contains("exceeds the Z travel 406.000"), "{err}");

        let small = unwrap_ok(super::parse_job(&machine_shell(3_000.0, r#""machine": { "model": "haas_st20y" },"#, "", TURN_TOOL, FACE_OP)), "small");
        let (doc, _) = unwrap_ok(super::build_document(Path::new("job.lathe.json"), &small, &cylinder(15.0, 30.0), None), "inside travel");
        assert_eq!(doc.program().toolpaths().len(), 1);
    }
}
