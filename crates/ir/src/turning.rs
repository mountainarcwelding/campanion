//! Lathe setup stored on a [`crate::Program`].
//!
//! Turning toolpaths stay ordinary [`crate::ClPath`]s. Their poses are in the part frame:
//!
//! - Z is the spindle axis. +Z points away from the chuck, toward the tailstock and the tool.
//! - X is a radius in millimetres, never a diameter. It may be negative when a face crosses the
//!   spindle axis (the tip is past centre). Y is 0 for XZ work. A head that lists Y may move off
//!   that plane while C is engaged and the pose carries a C angle.
//! - Diameter mode is a post setting only ([`TurningSetup::diameter_mode`]). The IR never stores
//!   a diameter in X.
//! - The pose axis is not interpreted for XZ turning moves. Callers pass [`crate::Vec3::Z`]
//!   because a pose requires a unit axis.
//!
//! A lathe has named spindles and heads. Each [`Spindle`] has its own optional C axis (a machine
//! axis id) and a work-zero offset along Z. The main spindle is named `main`; a sub-spindle is
//! another entry in the same list. A [`Head`] is a turret or a B-axis head. It carries X and Z,
//! and optionally Y and B, plus a station count and a live-tooling flag. Every turning toolpath
//! has a [`TurnBinding`] that names its spindle, head and channel. An M10 program uses one
//! spindle `main`, one head `turret1` and channel 0. [`TurningSetup::single`] builds that spindle
//! and head. Live tooling, Y, B and a second spindle use these same types: the lists and the
//! optional fields are the extension points.
//!
//! Stock Z is relative to the spindle work zero. The blank occupies Z in `[-length, 0]`, and
//! Z = 0 is the finished front face. Facing allowance comes from the operation, not from
//! [`TurnStock`].
//!
//! Units are millimetres, degrees, metres per minute and revolutions per minute.

use std::collections::BTreeSet;
use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use crate::Result;
use crate::check::{self, finite, non_negative, positive, required_name};

const MAX_SPINDLES: usize = 4;
const MAX_HEADS: usize = 8;
const MAX_HEAD_AXES: usize = 4;
const MAX_STATIONS: u16 = 256;
const MAX_BINDINGS: usize = 1_024;
const MAX_INSERTS: usize = 1_024;
const MAX_CSS_RPM: f64 = 100_000.0;
/// Below this radius, constant surface speed is the spindle cap.
const CSS_MIN_RADIUS_MM: f64 = 1e-9;
/// A tip-offset component this close to zero counts as zero (`sign(0) = 0`).
const TIP_ZERO: f64 = 1e-12;

/// Spindle speed for a turning pass: constant surface speed, or a direct rpm.
///
/// Stored on [`TurnBinding`] when that toolpath names a speed. A binding with no speed leaves
/// the number to the process-on parameter.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum SpindleSpeed {
    /// Constant surface speed. `surface_m_per_min` is metres per minute. `max_rpm` caps the speed.
    Css {
        /// Cutting speed, metres per minute.
        surface_m_per_min: f64,
        /// Spindle cap for this pass, revolutions per minute.
        max_rpm: f64,
    },
    /// Direct spindle speed, revolutions per minute.
    Rpm {
        /// Revolutions per minute.
        rpm: f64,
    },
}

impl SpindleSpeed {
    /// Constant surface speed. Both values must be finite and positive, and `max_rpm` must be at most 100000.
    pub fn css(surface_m_per_min: f64, max_rpm: f64) -> Result<Self> {
        let speed = Self::Css { surface_m_per_min, max_rpm };
        speed.check()?;
        Ok(speed)
    }

    /// Direct spindle speed. `rpm` must be finite and positive.
    pub fn rpm(rpm: f64) -> Result<Self> {
        let speed = Self::Rpm { rpm };
        speed.check()?;
        Ok(speed)
    }

    /// Revolutions per minute at `radius_mm`.
    ///
    /// Constant surface speed is `1000 * v / (2 * π * r)`, clamped to `max_rpm`. A radius of
    /// `1e-9` mm or less (and any non-finite radius) returns `max_rpm`. Direct rpm ignores the radius.
    pub fn rpm_at_radius(&self, radius_mm: f64) -> f64 {
        match *self {
            Self::Rpm { rpm } => rpm,
            Self::Css { surface_m_per_min, max_rpm } => {
                if !radius_mm.is_finite() || radius_mm <= CSS_MIN_RADIUS_MM {
                    return max_rpm;
                }
                let rpm = 1000.0 * surface_m_per_min / (2.0 * PI * radius_mm);
                if !rpm.is_finite() || rpm > max_rpm { max_rpm } else { rpm }
            }
        }
    }

    pub(crate) fn check(&self) -> Result<()> {
        match *self {
            Self::Css { surface_m_per_min, max_rpm } => {
                positive(surface_m_per_min, "surface speed")?;
                positive(max_rpm, "css max rpm")?;
                if max_rpm > MAX_CSS_RPM {
                    return Err(check::invalid("css max rpm is above 100000"));
                }
            }
            Self::Rpm { rpm } => {
                positive(rpm, "rpm")?;
            }
        }
        Ok(())
    }
}

/// One lathe spindle.
///
/// `name` is unique on the setup (`main`, and further names such as `sub`). `c_axis` is that spindle's
/// C axis as a machine axis id, when the machine has one. `work_zero_z` is the work-zero offset
/// along Z, in millimetres. Stock Z is measured from that zero.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Spindle {
    name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    c_axis: Option<u8>,
    /// Work-zero offset along Z, millimetres.
    work_zero_z: f64,
    /// Fastest programmed speed, revolutions per minute.
    max_rpm: f64,
    /// Positive rotation is opposite the default when true.
    #[serde(default)]
    reversed: bool,
}

impl Spindle {
    /// `name` is required. `work_zero_z` must be finite. `max_rpm` must be finite and positive.
    /// `reversed` starts false; use [`Self::with_reversed`].
    pub fn try_new(name: impl Into<String>, c_axis: Option<u8>, work_zero_z: f64, max_rpm: f64) -> Result<Self> {
        let spindle = Self { name: name.into(), c_axis, work_zero_z, max_rpm, reversed: false };
        spindle.check()?;
        Ok(spindle)
    }

    /// Set whether positive spindle rotation is reversed.
    pub fn with_reversed(mut self, reversed: bool) -> Self {
        self.reversed = reversed;
        self
    }

    /// Spindle name. Unique in the setup.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Machine axis id of this spindle's C axis, when the machine has one.
    pub fn c_axis(&self) -> Option<u8> {
        self.c_axis
    }

    /// Work-zero offset along Z, millimetres.
    pub fn work_zero_z(&self) -> f64 {
        self.work_zero_z
    }

    /// Fastest programmed speed, revolutions per minute.
    pub fn max_rpm(&self) -> f64 {
        self.max_rpm
    }

    /// Whether positive spindle rotation is opposite the default.
    pub fn reversed(&self) -> bool {
        self.reversed
    }

    pub(crate) fn check(&self) -> Result<()> {
        required_name(&self.name, "spindle name")?;
        finite(self.work_zero_z, "spindle work zero")?;
        positive(self.max_rpm, "spindle max rpm")?;
        Ok(())
    }
}

/// Axis a turning head can carry.
///
/// X and Z are required. Y and B are optional, so a later live-tool or B-axis head uses this
/// same enum.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum HeadAxis {
    /// Radial axis. Toolpaths store this as a radius.
    X,
    /// Optional cross axis.
    Y,
    /// Spindle axis.
    Z,
    /// Optional head rotation.
    B,
}

/// A turret or a B-axis head.
///
/// `stations` is the number of tool pockets, from 1 to 256. `live_tooling` is true when the
/// head can drive a spinning tool. Axes always include X and Z. Y and B are optional. The same
/// struct is a plain turret today and a Y/B live head later.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Head {
    name: String,
    axes: Vec<HeadAxis>,
    stations: u16,
    live_tooling: bool,
}

impl Head {
    /// `axes` must include X and Z, must not repeat an axis, and must hold at most four.
    /// `stations` is 1 to 256.
    pub fn try_new(name: impl Into<String>, axes: Vec<HeadAxis>, stations: u16, live_tooling: bool) -> Result<Self> {
        let head = Self { name: name.into(), axes, stations, live_tooling };
        head.check()?;
        Ok(head)
    }

    /// Head name. Unique in the setup.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Axes this head carries, in the order given at construction.
    pub fn axes(&self) -> &[HeadAxis] {
        &self.axes
    }

    /// Tool pockets, from 1 to 256.
    pub fn stations(&self) -> u16 {
        self.stations
    }

    /// Whether this head can drive a live tool.
    pub fn live_tooling(&self) -> bool {
        self.live_tooling
    }

    pub(crate) fn check(&self) -> Result<()> {
        required_name(&self.name, "head name")?;
        if self.axes.len() > MAX_HEAD_AXES {
            return Err(check::invalid(format!("head has more than {MAX_HEAD_AXES} axes")));
        }
        let mut seen = BTreeSet::new();
        for axis in &self.axes {
            if !seen.insert(*axis) {
                return Err(check::invalid("head lists an axis twice"));
            }
        }
        if !self.axes.contains(&HeadAxis::X) || !self.axes.contains(&HeadAxis::Z) {
            return Err(check::invalid("head axes must include x and z"));
        }
        if !(1..=MAX_STATIONS).contains(&self.stations) {
            return Err(check::invalid(format!("head stations must be from 1 to {MAX_STATIONS}")));
        }
        Ok(())
    }
}

/// Bar or tube blank.
///
/// The stock occupies Z in `[-length, 0]` relative to the spindle work zero. Z = 0 is the
/// finished front face, the face toward the tailstock. The chuck is toward -Z. Diameters are
/// millimetres. Facing stock allowance is not stored here; an operation supplies it.
///
/// X toolpaths use radii ([`Self::outer_radius`], [`Self::inner_radius`]), not these diameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TurnStock {
    /// Solid bar. `diameter` and `length` are millimetres.
    Bar {
        /// Outside diameter, millimetres.
        diameter: f64,
        /// Blank length, millimetres. The blank occupies Z in `[-length, 0]`.
        length: f64,
    },
    /// Tube. Diameters and `length` are millimetres. The inner diameter is less than the outer.
    Tube {
        /// Outside diameter, millimetres.
        outer_diameter: f64,
        /// Hole diameter, millimetres. Must be less than `outer_diameter`.
        inner_diameter: f64,
        /// Blank length, millimetres. The blank occupies Z in `[-length, 0]`.
        length: f64,
    },
}

impl TurnStock {
    /// Solid bar. `diameter` and `length` must be finite and positive.
    pub fn bar(diameter: f64, length: f64) -> Result<Self> {
        let stock = Self::Bar { diameter, length };
        stock.check()?;
        Ok(stock)
    }

    /// Tube. All three values must be finite and positive, and `inner_diameter` must be less than `outer_diameter`.
    pub fn tube(outer_diameter: f64, inner_diameter: f64, length: f64) -> Result<Self> {
        let stock = Self::Tube { outer_diameter, inner_diameter, length };
        stock.check()?;
        Ok(stock)
    }

    /// Outside radius, millimetres. Half the outside diameter.
    pub fn outer_radius(&self) -> f64 {
        match *self {
            Self::Bar { diameter, .. } => diameter * 0.5,
            Self::Tube { outer_diameter, .. } => outer_diameter * 0.5,
        }
    }

    /// Hole radius, millimetres. Zero for a bar.
    pub fn inner_radius(&self) -> f64 {
        match *self {
            Self::Bar { .. } => 0.0,
            Self::Tube { inner_diameter, .. } => inner_diameter * 0.5,
        }
    }

    /// Blank length, millimetres.
    pub fn length(&self) -> f64 {
        match *self {
            Self::Bar { length, .. } | Self::Tube { length, .. } => length,
        }
    }

    pub(crate) fn check(&self) -> Result<()> {
        match *self {
            Self::Bar { diameter, length } => {
                positive(diameter, "bar diameter")?;
                positive(length, "bar length")?;
            }
            Self::Tube { outer_diameter, inner_diameter, length } => {
                positive(outer_diameter, "tube outer diameter")?;
                positive(inner_diameter, "tube inner diameter")?;
                positive(length, "tube length")?;
                if inner_diameter >= outer_diameter {
                    return Err(check::invalid("tube inner diameter must be less than outer diameter"));
                }
            }
        }
        Ok(())
    }
}

/// ISO insert shape. The number is the included angle in degrees, when the shape has one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum InsertShape {
    /// 80° rhombic.
    C80,
    /// 55° rhombic.
    D55,
    /// 35° rhombic.
    V35,
    /// 80° trigon.
    W80,
    /// 60° triangular.
    T60,
    /// 90° square.
    S90,
    /// Round. No single included angle.
    R,
    /// Parting or grooving insert. No included angle.
    Groove,
    /// Threading insert. Included angle 60°.
    Thread,
}

impl InsertShape {
    /// Included angle in degrees. Round and groove inserts return `None`.
    pub fn included_angle_deg(&self) -> Option<f64> {
        match self {
            Self::C80 | Self::W80 => Some(80.0),
            Self::D55 => Some(55.0),
            Self::V35 => Some(35.0),
            Self::T60 | Self::Thread => Some(60.0),
            Self::S90 => Some(90.0),
            Self::R | Self::Groove => None,
        }
    }
}

/// Which way the cutting edge faces along the spindle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Hand {
    /// Left-hand insert.
    Left,
    /// Right-hand insert.
    Right,
    /// Symmetric insert, such as a groove or a round.
    Neutral,
}

/// One turning insert.
///
/// `orientation_deg` is the angle in the XZ plane, in degrees, from +Z toward +X, of the
/// insert's bisector. The bisector points from the nose into the holder. `hand` records which
/// way the edge faces; it does not change [`Self::tip_offset`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Insert {
    shape: InsertShape,
    /// Nose radius, millimetres. Zero is a sharp corner. A round insert must be greater than zero.
    nose_radius: f64,
    /// Insert width, millimetres. Required for a groove insert. Positive when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    width: Option<f64>,
    hand: Hand,
    /// Bisector from the nose into the holder, degrees in the XZ plane, from +Z toward +X.
    orientation_deg: f64,
}

impl Insert {
    /// `nose_radius` must be zero or positive, and positive for a round insert.
    /// A groove insert requires a positive `width`. `orientation_deg` must be finite and in `-360..=360`.
    pub fn try_new(shape: InsertShape, nose_radius: f64, width: Option<f64>, hand: Hand, orientation_deg: f64) -> Result<Self> {
        let insert = Self { shape, nose_radius, width, hand, orientation_deg };
        insert.check()?;
        Ok(insert)
    }

    /// Insert shape.
    pub fn shape(&self) -> InsertShape {
        self.shape
    }

    /// Nose radius, millimetres.
    pub fn nose_radius(&self) -> f64 {
        self.nose_radius
    }

    /// Insert width, millimetres, when the insert records one.
    pub fn width(&self) -> Option<f64> {
        self.width
    }

    /// Which way the cutting edge faces.
    pub fn hand(&self) -> Hand {
        self.hand
    }

    /// Bisector from the nose into the holder, degrees.
    pub fn orientation_deg(&self) -> f64 {
        self.orientation_deg
    }

    /// Imaginary tip offset in X (radius) and Z, millimetres.
    ///
    /// The programmed point is the theoretical sharp tip. The nose is an arc of [`Self::nose_radius`]
    /// whose centre sits one radius away from that tip on each axis that the holder occupies.
    /// The direction from the nose toward the holder is `u = (sin o, cos o)` in `(X, Z)`, with
    /// `o` the orientation in radians. The offset from the nose-radius centre to the sharp tip
    /// is `(-r * sign(u.x), -r * sign(u.z))`, and `sign(0) = 0`. A component whose absolute
    /// value is at most `1e-12` counts as zero, so a bisector that lies on one axis has no
    /// offset on that axis.
    pub fn tip_offset(&self) -> (f64, f64) {
        let radians = self.orientation_deg.to_radians();
        let toward_holder_x = radians.sin();
        let toward_holder_z = radians.cos();
        let radius = self.nose_radius;
        (-radius * tip_sign(toward_holder_x), -radius * tip_sign(toward_holder_z))
    }

    pub(crate) fn check(&self) -> Result<()> {
        non_negative(self.nose_radius, "nose radius")?;
        if self.shape == InsertShape::R && self.nose_radius <= 0.0 {
            return Err(check::invalid("round insert nose radius must be positive"));
        }
        match self.width {
            Some(width) => {
                positive(width, "insert width")?;
            }
            None => {
                if self.shape == InsertShape::Groove {
                    return Err(check::invalid("groove insert needs a width"));
                }
            }
        }
        let orientation = finite(self.orientation_deg, "insert orientation")?;
        if !(-360.0..=360.0).contains(&orientation) {
            return Err(check::invalid("insert orientation must be from -360 to 360 degrees"));
        }
        Ok(())
    }
}

fn tip_sign(component: f64) -> f64 {
    if component.abs() <= TIP_ZERO {
        0.0
    } else if component > 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// An insert mounted on a head station.
///
/// `live` is true for a driven tool. The head named here must then have [`Head::live_tooling`].
/// `station` is a pocket number from 1 through the head's station count.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TurnInsert {
    tool_id: u32,
    head: String,
    station: u16,
    insert: Insert,
    live: bool,
}

impl TurnInsert {
    /// `head` is required. `station` is 1 to 256. The setup check also requires the station to
    /// exist on that head.
    pub fn try_new(tool_id: u32, head: impl Into<String>, station: u16, insert: Insert, live: bool) -> Result<Self> {
        let mounted = Self { tool_id, head: head.into(), station, insert, live };
        mounted.check()?;
        Ok(mounted)
    }

    /// [`crate::Tool`] id this insert belongs to.
    pub fn tool_id(&self) -> u32 {
        self.tool_id
    }

    /// Head that holds this insert.
    pub fn head(&self) -> &str {
        &self.head
    }

    /// Pocket number, from 1.
    pub fn station(&self) -> u16 {
        self.station
    }

    /// Insert geometry.
    pub fn insert(&self) -> Insert {
        self.insert
    }

    /// Whether this is a driven tool.
    pub fn live(&self) -> bool {
        self.live
    }

    pub(crate) fn check(&self) -> Result<()> {
        required_name(&self.head, "insert head")?;
        if !(1..=MAX_STATIONS).contains(&self.station) {
            return Err(check::invalid(format!("insert station must be from 1 to {MAX_STATIONS}")));
        }
        self.insert.check()?;
        Ok(())
    }
}

/// Binds one toolpath to a spindle, a head and a channel.
///
/// `toolpath` is an index into [`crate::Program::toolpaths`]. `channel` is a [`crate::Channel`]
/// id. An M10 binding uses spindle `main`, head `turret1` and channel 0. `speed` is the spindle
/// speed for that toolpath. Absent means the post uses the process-on speed parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TurnBinding {
    toolpath: u32,
    spindle: String,
    head: String,
    channel: u16,
    /// Constant surface speed or direct rpm for this toolpath.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    speed: Option<SpindleSpeed>,
}

impl TurnBinding {
    /// `spindle` and `head` are required names. The setup check requires they exist.
    /// `speed` starts unset; use [`Self::with_speed`].
    pub fn try_new(toolpath: u32, spindle: impl Into<String>, head: impl Into<String>, channel: u16) -> Result<Self> {
        let binding = Self { toolpath, spindle: spindle.into(), head: head.into(), channel, speed: None };
        binding.check()?;
        Ok(binding)
    }

    /// Index into [`crate::Program::toolpaths`].
    pub fn toolpath(&self) -> u32 {
        self.toolpath
    }

    /// Spindle that turns the work for this toolpath.
    pub fn spindle(&self) -> &str {
        &self.spindle
    }

    /// Head that carries the tool for this toolpath.
    pub fn head(&self) -> &str {
        &self.head
    }

    /// Channel id for this toolpath.
    pub fn channel(&self) -> u16 {
        self.channel
    }

    /// Spindle speed for this toolpath, when the binding names one.
    pub fn speed(&self) -> Option<SpindleSpeed> {
        self.speed
    }

    /// Set or clear the spindle speed and re-validate.
    pub fn with_speed(mut self, speed: Option<SpindleSpeed>) -> Result<Self> {
        self.speed = speed;
        self.check()?;
        Ok(self)
    }

    pub(crate) fn check(&self) -> Result<()> {
        required_name(&self.spindle, "binding spindle")?;
        required_name(&self.head, "binding head")?;
        if let Some(speed) = self.speed {
            speed.check()?;
        }
        Ok(())
    }
}

fn default_diameter_mode() -> bool {
    true
}

/// Finished part half-profile, stored for preview and for insert and holder checks.
///
/// `points` is a polyline of `(z, radius)` in millimetres, ordered from the front (the end
/// toward the tailstock, larger Z) toward the chuck. The radius is never negative. The
/// polyline is the finished surface. It is not stock, and it does not add a leg down to the
/// axis unless the caller included one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TurnSection {
    points: Vec<[f64; 2]>,
}

/// Fewest samples [`TurnSection::try_new`] accepts.
const MIN_SECTION_POINTS: usize = 2;
/// Most samples [`TurnSection::try_new`] accepts.
const MAX_SECTION_POINTS: usize = 200_000;

impl TurnSection {
    /// `points` holds 2 to 200000 positions. Each Z is finite and each radius is finite and `>= 0`.
    pub fn try_new(points: Vec<[f64; 2]>) -> Result<Self> {
        let section = Self { points };
        section.check()?;
        Ok(section)
    }

    /// `(z, radius)` points, front to back.
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }

    pub(crate) fn check(&self) -> Result<()> {
        let count = self.points.len();
        if !(MIN_SECTION_POINTS..=MAX_SECTION_POINTS).contains(&count) {
            return Err(check::invalid(format!("turning section needs {MIN_SECTION_POINTS} to {MAX_SECTION_POINTS} points, found {count}")));
        }
        for point in &self.points {
            finite(point[0], "section z")?;
            non_negative(point[1], "section radius")?;
        }
        Ok(())
    }
}

/// Spindles, heads, stock, inserts and the toolpath bindings for one lathe program.
///
/// `diameter_mode` is only a hint for posts. When true, a post may write X as a diameter.
/// Toolpath X stays a radius either way. The default hint is true. There are 1 to 4 spindles
/// and 1 to 8 heads, so a sub-spindle or a second head does not need a new type.
/// [`Self::section`] is the finished half-profile when a caller stored one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TurningSetup {
    spindles: Vec<Spindle>,
    heads: Vec<Head>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    stock: Option<TurnStock>,
    /// Finished half-profile, `(z, radius)`, front to back.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    section: Option<TurnSection>,
    inserts: Vec<TurnInsert>,
    bindings: Vec<TurnBinding>,
    /// Post hint only. Toolpath X is still a radius.
    #[serde(default = "default_diameter_mode")]
    diameter_mode: bool,
}

impl TurningSetup {
    /// Build a setup. `diameter_mode` starts true. Use [`Self::with_diameter_mode`] to change it.
    pub fn try_new(
        spindles: Vec<Spindle>,
        heads: Vec<Head>,
        stock: Option<TurnStock>,
        inserts: Vec<TurnInsert>,
        bindings: Vec<TurnBinding>,
    ) -> Result<Self> {
        let setup = Self { spindles, heads, stock, section: None, inserts, bindings, diameter_mode: true };
        setup.check()?;
        Ok(setup)
    }

    /// M10 arrangement: spindle `main` (no C axis, work zero at 0), head `turret1` with X and Z,
    /// 12 stations and no live tooling. No inserts and no bindings. `max_rpm` is the spindle cap.
    pub fn single(stock: Option<TurnStock>, max_rpm: f64) -> Result<Self> {
        let spindle = Spindle::try_new("main", None, 0.0, max_rpm)?;
        let head = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false)?;
        Self::try_new(vec![spindle], vec![head], stock, Vec::new(), Vec::new())
    }

    /// Add an insert and re-validate.
    pub fn with_insert(mut self, insert: TurnInsert) -> Result<Self> {
        self.inserts.push(insert);
        self.check()?;
        Ok(self)
    }

    /// Add a toolpath binding and re-validate.
    pub fn with_binding(mut self, binding: TurnBinding) -> Result<Self> {
        self.bindings.push(binding);
        self.check()?;
        Ok(self)
    }

    /// Set the post hint for diameter-mode X words.
    pub fn with_diameter_mode(mut self, diameter_mode: bool) -> Self {
        self.diameter_mode = diameter_mode;
        self
    }

    /// Store or clear the finished half-profile and re-validate.
    pub fn with_section(mut self, section: Option<TurnSection>) -> Result<Self> {
        self.section = section;
        self.check()?;
        Ok(self)
    }

    /// Spindles, main first when built with [`Self::single`].
    pub fn spindles(&self) -> &[Spindle] {
        &self.spindles
    }

    /// Heads.
    pub fn heads(&self) -> &[Head] {
        &self.heads
    }

    /// Blank, when the setup records one.
    pub fn stock(&self) -> Option<TurnStock> {
        self.stock
    }

    /// Finished half-profile, when the setup records one.
    pub fn section(&self) -> Option<&TurnSection> {
        self.section.as_ref()
    }

    /// Inserts mounted on heads.
    pub fn inserts(&self) -> &[TurnInsert] {
        &self.inserts
    }

    /// Toolpath bindings.
    pub fn bindings(&self) -> &[TurnBinding] {
        &self.bindings
    }

    /// Post hint: write X as a diameter when true. The IR still stores radii.
    pub fn diameter_mode(&self) -> bool {
        self.diameter_mode
    }

    /// Spindle with this name.
    pub fn spindle(&self, name: &str) -> Option<&Spindle> {
        self.spindles.iter().find(|spindle| spindle.name == name)
    }

    /// Head with this name.
    pub fn head(&self, name: &str) -> Option<&Head> {
        self.heads.iter().find(|head| head.name == name)
    }

    /// Insert mounted for this tool id.
    pub fn insert_for_tool(&self, tool_id: u32) -> Option<&TurnInsert> {
        self.inserts.iter().find(|insert| insert.tool_id == tool_id)
    }

    /// Binding for this toolpath index.
    pub fn binding_for(&self, toolpath: u32) -> Option<&TurnBinding> {
        self.bindings.iter().find(|binding| binding.toolpath == toolpath)
    }

    pub(crate) fn check(&self) -> Result<()> {
        self.check_spindles()?;
        self.check_heads()?;
        if let Some(stock) = self.stock {
            stock.check()?;
        }
        if let Some(section) = &self.section {
            section.check()?;
        }
        self.check_inserts()?;
        self.check_bindings()?;
        Ok(())
    }

    fn check_spindles(&self) -> Result<()> {
        if !(1..=MAX_SPINDLES).contains(&self.spindles.len()) {
            return Err(check::invalid(format!("turning setup needs 1 to {MAX_SPINDLES} spindles, found {}", self.spindles.len())));
        }
        let mut names = BTreeSet::new();
        for spindle in &self.spindles {
            spindle.check()?;
            if !names.insert(spindle.name.as_str()) {
                return Err(check::invalid(format!("duplicate spindle name {}", spindle.name)));
            }
        }
        Ok(())
    }

    fn check_heads(&self) -> Result<()> {
        if !(1..=MAX_HEADS).contains(&self.heads.len()) {
            return Err(check::invalid(format!("turning setup needs 1 to {MAX_HEADS} heads, found {}", self.heads.len())));
        }
        let mut names = BTreeSet::new();
        for head in &self.heads {
            head.check()?;
            if !names.insert(head.name.as_str()) {
                return Err(check::invalid(format!("duplicate head name {}", head.name)));
            }
        }
        Ok(())
    }

    fn check_inserts(&self) -> Result<()> {
        if self.inserts.len() > MAX_INSERTS {
            return Err(check::invalid(format!("too many turning inserts: {} > {MAX_INSERTS}", self.inserts.len())));
        }
        let mut tool_ids = BTreeSet::new();
        for insert in &self.inserts {
            insert.check()?;
            let Some(head) = self.head(&insert.head) else {
                return Err(check::invalid(format!("insert head {} is not on this setup", insert.head)));
            };
            if insert.station > head.stations {
                return Err(check::invalid(format!("insert station {} is past head {} ({} stations)", insert.station, head.name, head.stations)));
            }
            if insert.live && !head.live_tooling {
                return Err(check::invalid(format!("live insert for tool {} needs live tooling on head {}", insert.tool_id, insert.head)));
            }
            if !tool_ids.insert(insert.tool_id) {
                return Err(check::invalid(format!("duplicate turning insert for tool {}", insert.tool_id)));
            }
        }
        Ok(())
    }

    fn check_bindings(&self) -> Result<()> {
        if self.bindings.len() > MAX_BINDINGS {
            return Err(check::invalid(format!("too many turning bindings: {} > {MAX_BINDINGS}", self.bindings.len())));
        }
        let mut seen = BTreeSet::new();
        for binding in &self.bindings {
            binding.check()?;
            let Some(spindle) = self.spindle(&binding.spindle) else {
                return Err(check::invalid(format!("binding spindle {} is not on this setup", binding.spindle)));
            };
            if let Some(SpindleSpeed::Css { max_rpm, .. }) = binding.speed()
                && max_rpm > spindle.max_rpm()
            {
                return Err(check::invalid(format!("css max rpm is above spindle {} max rpm", spindle.name())));
            }
            if self.head(&binding.head).is_none() {
                return Err(check::invalid(format!("binding head {} is not on this setup", binding.head)));
            }
            if !seen.insert(binding.toolpath) {
                return Err(check::invalid(format!("toolpath {} has more than one turning binding", binding.toolpath)));
            }
        }
        Ok(())
    }
}
