//! User machine profiles stored in the config directory.
//!
//! A profile is a `campanion-machine/2` JSON file. `campanion-machine/1` files are migrated
//! on load. Every numeric value is entered by the user. This module does not ship machine
//! limits, posts, or material defaults.

use std::fs;
use std::path::{Path, PathBuf};

use serde_json::{Map, Value, json};

use crate::error::CliError;
use crate::job::{as_f64, as_str, check_keys, object, optional};

/// Format tag written in every new profile file.
pub const FORMAT_TAG: &str = "campanion-machine/2";

/// Previous profile format, still accepted and migrated on load.
pub const FORMAT_TAG_V1: &str = "campanion-machine/1";

/// Recognised head process ids, stored lowercase in the file.
pub const PROCESS_IDS: &[&str] = &["milling", "routing", "turning", "laser", "plasma", "waterjet", "oxyfuel", "wire_edm", "sla", "fdm"];

/// A job's reference to a user machine profile. This is not the lathe `machine.model` block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JobMachineRef {
    /// File stem in the machines directory, when the job names a stored profile.
    pub profile: Option<String>,
    /// Explicit profile file path, used when `profile` is omitted.
    pub path: Option<PathBuf>,
    /// Head id on that profile. When omitted, the first head is used.
    pub head: Option<String>,
}

/// One user machine profile.
#[derive(Debug, Clone, PartialEq)]
pub struct MachineFile {
    /// Display name.
    pub name: String,
    /// `mm` or `inch`. Lengths on the loaded value are millimetres.
    pub units: String,
    /// Gantry (heads on a bed) or resin (no heads).
    pub kind: MachineKind,
    /// Bed size the user entered, millimetres. Optional after a v1 migrate when the work area
    /// did not name both X and Y extents.
    pub bed: Option<BedSize>,
    /// Resin screen size, millimetres, when the user entered it.
    pub screen: Option<BedSize>,
    /// Work envelope, when the user entered one or more limits.
    pub work_area: Option<WorkArea>,
    /// Controller name, when entered.
    pub controller: Option<String>,
    /// Default post id or path, when entered.
    pub post: Option<String>,
    /// Heads on a gantry. Empty on a resin machine.
    pub heads: Vec<MachineHead>,
}

/// Machine family stored in `type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MachineKind {
    /// A bed with one or more heads.
    Gantry,
    /// A resin printer. No heads.
    Resin,
}

impl MachineKind {
    /// File value: `gantry` or `resin`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Gantry => "gantry",
            Self::Resin => "resin",
        }
    }

    fn parse(text: &str) -> Result<Self, String> {
        match text {
            "gantry" => Ok(Self::Gantry),
            "resin" => Ok(Self::Resin),
            other => Err(format!("machine type must be gantry or resin, not `{other}`")),
        }
    }
}

/// Job type derived from a machine and the selected head's capabilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobType {
    /// Cutting or routing (laser, plasma, spindle, and other 2D cut processes).
    CutRoute,
    /// Printing (resin machine, or an extruder head).
    Print,
}

impl JobType {
    /// File value: `cut_route` or `print`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CutRoute => "cut_route",
            Self::Print => "print",
        }
    }

    /// Parse a stored job type.
    pub fn parse(text: &str) -> Result<Self, String> {
        match text {
            "cut_route" => Ok(Self::CutRoute),
            "print" => Ok(Self::Print),
            other => Err(format!("job type must be cut_route or print, not `{other}`")),
        }
    }
}

/// User-entered bed or screen size, millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BedSize {
    pub x: f64,
    pub y: f64,
    /// Optional height (resin Z), millimetres.
    pub z: Option<f64>,
}

/// Axis travel the user entered, in millimetres. An omitted end is not checked.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkArea {
    pub x_min: Option<f64>,
    pub x_max: Option<f64>,
    pub y_min: Option<f64>,
    pub y_max: Option<f64>,
    pub z_min: Option<f64>,
    pub z_max: Option<f64>,
}

/// One head on a machine, with its process, post, and material library.
#[derive(Debug, Clone, PartialEq)]
pub struct MachineHead {
    /// Stable id unique within the profile.
    pub id: String,
    /// Display name. Empty when the user has not entered one.
    pub name: String,
    /// Head kind: `laser`, `plasma`, `spindle`, `extruder`, or another process id (extensible).
    pub kind: String,
    /// Process id from [`PROCESS_IDS`].
    pub process: String,
    /// Post for this head, when entered. Otherwise the machine-level post is used.
    pub post: Option<String>,
    /// Per-head origin offset, millimetres, when the user entered all three axes.
    pub offset: Option<HeadOffset>,
    /// Safe retract height for this head, millimetres, when entered.
    pub safe_z: Option<f64>,
    /// Rated power used to map watt presets onto the planner's 0..=1 fraction.
    pub max_power: Option<PowerValue>,
    /// User-entered material and tool-library entries. Empty until the user adds one.
    pub materials: Vec<MaterialEntry>,
}

/// Per-head XYZ offset, millimetres. All three axes are user-entered together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadOffset {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A power preset in real units. The planner maps this to an IR fraction in 0..=1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerValue {
    pub unit: PowerUnit,
    pub value: f64,
}

/// Units for [`PowerValue`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerUnit {
    /// Percent of full power (0..=100).
    Percent,
    /// Watts. Mapping to a fraction needs a user-entered `max_power` on the head.
    Watts,
}

impl PowerUnit {
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Percent => "percent",
            Self::Watts => "w",
        }
    }

    fn parse(text: &str) -> Result<Self, String> {
        match text {
            "percent" | "%" => Ok(Self::Percent),
            "w" | "W" | "watts" => Ok(Self::Watts),
            other => Err(format!("power unit must be percent or w, not `{other}`")),
        }
    }
}

impl PowerValue {
    /// A percent-of-full-power preset.
    #[must_use]
    pub fn percent(value: f64) -> Self {
        Self { unit: PowerUnit::Percent, value }
    }

    /// A watt preset.
    #[must_use]
    pub fn watts(value: f64) -> Self {
        Self { unit: PowerUnit::Watts, value }
    }

    /// Convert a v1 `0..=1` fraction into a percent preset.
    #[must_use]
    pub fn from_v1_fraction(fraction: f64) -> Self {
        Self::percent(fraction * 100.0)
    }

    /// Map this preset to the planner's 0..=1 IR fraction.
    ///
    /// Watts need `max_watts` from the head. Nothing is invented when it is missing.
    pub fn to_fraction(self, max_watts: Option<f64>) -> Result<f64, String> {
        match self.unit {
            PowerUnit::Percent => {
                if !self.value.is_finite() || !(0.0..=100.0).contains(&self.value) {
                    return Err("power percent must be a finite number in 0..=100".to_string());
                }
                Ok(self.value / 100.0)
            }
            PowerUnit::Watts => {
                if !self.value.is_finite() || self.value < 0.0 {
                    return Err("power in watts must be a finite number that is not negative".to_string());
                }
                let Some(max) = max_watts else {
                    return Err("power in watts needs a user-entered max_power (watts) on the head".to_string());
                };
                if !max.is_finite() || max <= 0.0 {
                    return Err("head max_power must be a finite number greater than zero".to_string());
                }
                if self.value > max {
                    return Err(format!("power {0} W is greater than the head max_power {max} W", self.value));
                }
                Ok(self.value / max)
            }
        }
    }
}

/// One material in a head library. Only fields the user typed are present.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MaterialEntry {
    pub name: String,
    pub kerf: Option<f64>,
    pub diameter: Option<f64>,
    pub wire: Option<f64>,
    pub depth: Option<f64>,
    pub step_down: Option<f64>,
    pub cut_feed: Option<f64>,
    pub plunge_feed: Option<f64>,
    /// Power in percent or watts. Never a raw 0..=1 fraction on disk for v2.
    pub power: Option<PowerValue>,
    /// Number of cut passes, when the user entered one.
    pub passes: Option<u32>,
    pub pierce_delay: Option<f64>,
    pub post_delay: Option<f64>,
}

/// Root of the user config tree (`.../campanion`), which holds `machines/*.json`.
///
/// Override with `CAMPANION_CONFIG_DIR`. Otherwise `XDG_CONFIG_HOME/campanion`, or
/// `$HOME/.config/campanion`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProfileStore {
    root: PathBuf,
}

impl ProfileStore {
    /// Discover the config directory from the environment, without creating it.
    pub fn from_env() -> Result<Self, CliError> {
        Ok(Self { root: discover_config_dir()? })
    }

    /// Use `root` as the config directory. Intended for tests and for a caller that already
    /// resolved a path.
    #[must_use]
    pub fn from_dir(root: PathBuf) -> Self {
        Self { root }
    }

    /// Config directory.
    #[must_use]
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// `root/machines`.
    #[must_use]
    pub fn machines_dir(&self) -> PathBuf {
        self.root.join("machines")
    }

    /// List `*.json` profiles. A missing directory yields an empty list.
    pub fn list(&self) -> Result<Vec<(String, PathBuf)>, CliError> {
        let dir = self.machines_dir();
        if !dir.exists() {
            return Ok(Vec::new());
        }
        let mut entries = Vec::new();
        let read = fs::read_dir(&dir).map_err(|err| CliError::run(format!("{}: {err}", dir.display())))?;
        for item in read {
            let item = item.map_err(|err| CliError::run(format!("{}: {err}", dir.display())))?;
            let path = item.path();
            if path.extension().and_then(|ext| ext.to_str()) != Some("json") {
                continue;
            }
            let Some(id) = path.file_stem().and_then(|stem| stem.to_str()) else {
                continue;
            };
            if valid_profile_id(id) {
                entries.push((id.to_string(), path));
            }
        }
        entries.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(entries)
    }

    /// Load a profile by file stem.
    pub fn load(&self, id: &str) -> Result<MachineFile, CliError> {
        if !valid_profile_id(id) {
            return Err(CliError::run(format!("invalid machine profile id `{id}`")));
        }
        let path = self.machines_dir().join(format!("{id}.json"));
        if !path.exists() {
            return Err(CliError::run(format!("machine profile `{id}` was not found in {}", self.machines_dir().display())));
        }
        load_path(&path)
    }

    /// Write `profile` as `machines/{id}.json`, creating the directory when needed.
    pub fn save(&self, id: &str, profile: &MachineFile) -> Result<PathBuf, CliError> {
        if !valid_profile_id(id) {
            return Err(CliError::run("machine profile id must start with a letter or digit and contain only letters, digits, '.', '_' or '-'"));
        }
        let dir = self.machines_dir();
        fs::create_dir_all(&dir).map_err(|err| CliError::run(format!("{}: {err}", dir.display())))?;
        let path = dir.join(format!("{id}.json"));
        let text = serde_json::to_string_pretty(&profile_to_json(profile)).map_err(|err| CliError::run(format!("machine profile: {err}")))?;
        fs::write(&path, format!("{text}\n")).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
        Ok(path)
    }

    /// Delete `machines/{id}.json`.
    pub fn delete(&self, id: &str) -> Result<(), CliError> {
        if !valid_profile_id(id) {
            return Err(CliError::run("invalid machine profile id"));
        }
        let path = self.machines_dir().join(format!("{id}.json"));
        if !path.exists() {
            return Err(CliError::run(format!("machine profile `{id}` was not found")));
        }
        fs::remove_file(&path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))
    }
}

fn discover_config_dir() -> Result<PathBuf, CliError> {
    if let Some(dir) = std::env::var_os("CAMPANION_CONFIG_DIR") {
        let path = PathBuf::from(dir);
        if path.as_os_str().is_empty() {
            return Err(CliError::run("CAMPANION_CONFIG_DIR is empty"));
        }
        return Ok(path);
    }
    if let Some(xdg) = std::env::var_os("XDG_CONFIG_HOME") {
        let path = PathBuf::from(xdg);
        if !path.as_os_str().is_empty() {
            return Ok(path.join("campanion"));
        }
    }
    let home = std::env::var_os("HOME").ok_or_else(|| CliError::run("HOME is not set; cannot find the machine profile directory"))?;
    Ok(PathBuf::from(home).join(".config").join("campanion"))
}

/// A profile id is the file stem; it may contain letters, digits, `.`, `_`, and `-`.
pub fn valid_profile_id(id: &str) -> bool {
    let mut chars = id.chars();
    chars.next().is_some_and(|first| first.is_ascii_alphanumeric())
        && id.chars().all(|ch| ch.is_ascii_alphanumeric() || ch == '.' || ch == '_' || ch == '-')
}

/// Load `path` as a `campanion-machine/1` file.
pub fn load_path(path: &Path) -> Result<MachineFile, CliError> {
    let text = fs::read_to_string(path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    parse_profile(&text).map_err(|err| CliError::run(format!("{}: {err}", path.display())))
}

/// Parse a `campanion-machine/1` document. Lengths in the result are millimetres.
pub fn parse_profile(text: &str) -> Result<MachineFile, String> {
    let value: Value = serde_json::from_str(text).map_err(|err| err.to_string())?;
    parse_profile_value(&value)
}

fn parse_profile_value(value: &Value) -> Result<MachineFile, String> {
    let map = object(value, "machine profile").map_err(|err| err.to_string())?;
    let tag = as_str(require(map, "format")?, "format").map_err(|err| err.to_string())?;
    match tag {
        FORMAT_TAG_V1 => parse_profile_v1(map),
        FORMAT_TAG => parse_profile_v2(map),
        other => Err(format!("unsupported machine profile format `{other}`; expected {FORMAT_TAG} or {FORMAT_TAG_V1}")),
    }
}

fn parse_profile_v1(map: &Map<String, Value>) -> Result<MachineFile, String> {
    check_keys(map, &["format", "name", "units", "work_area", "controller", "post", "heads"], "machine profile").map_err(|err| err.to_string())?;
    let (name, units, scale, work_area, controller, post) = parse_profile_common(map)?;
    let heads = parse_heads(require(map, "heads")?, scale, true)?;
    if heads.is_empty() {
        return Err("a machine profile needs at least one head".to_string());
    }
    Ok(MachineFile {
        name,
        units,
        kind: MachineKind::Gantry,
        bed: bed_from_work_area(work_area),
        screen: None,
        work_area,
        controller,
        post,
        heads: migrate_v1_heads(heads),
    })
}

fn parse_profile_v2(map: &Map<String, Value>) -> Result<MachineFile, String> {
    check_keys(map, &["format", "name", "units", "type", "bed", "screen", "work_area", "controller", "post", "heads"], "machine profile")
        .map_err(|err| err.to_string())?;
    let (name, units, scale, work_area, controller, post) = parse_profile_common(map)?;
    let kind = MachineKind::parse(as_str(require(map, "type")?, "type").map_err(|err| err.to_string())?)?;
    let bed = match optional(map, "bed") {
        Some(value) => Some(parse_bed(value, scale, "bed")?),
        None => None,
    };
    let screen = match optional(map, "screen") {
        Some(value) => Some(parse_bed(value, scale, "screen")?),
        None => None,
    };
    let heads = match optional(map, "heads") {
        Some(value) => parse_heads(value, scale, false)?,
        None => Vec::new(),
    };
    match kind {
        MachineKind::Gantry => {
            if heads.is_empty() {
                return Err("a gantry machine needs at least one head".to_string());
            }
            if screen.is_some() {
                return Err("screen is only valid on a resin machine".to_string());
            }
        }
        MachineKind::Resin => {
            if !heads.is_empty() {
                return Err("a resin machine has no heads".to_string());
            }
        }
    }
    Ok(MachineFile { name, units, kind, bed, screen, work_area, controller, post, heads })
}

fn parse_profile_common(map: &Map<String, Value>) -> Result<(String, String, f64, Option<WorkArea>, Option<String>, Option<String>), String> {
    let name = as_str(require(map, "name")?, "name").map_err(|err| err.to_string())?.to_string();
    if name.trim().is_empty() {
        return Err("machine profile name is empty".to_string());
    }
    let units = as_str(require(map, "units")?, "units").map_err(|err| err.to_string())?;
    let scale = match units {
        "mm" => 1.0,
        "inch" => 25.4,
        other => return Err(format!("units must be mm or inch, not `{other}`")),
    };
    let work_area = match optional(map, "work_area") {
        Some(value) => Some(parse_work_area(value, scale)?),
        None => None,
    };
    let controller = optional_string(map, "controller")?;
    let post = optional_string(map, "post")?;
    Ok((name, units.to_string(), scale, work_area, controller, post))
}

fn parse_heads(value: &Value, scale: f64, v1: bool) -> Result<Vec<MachineHead>, String> {
    let Some(heads_arr) = value.as_array() else {
        return Err("heads must be an array".to_string());
    };
    let mut heads = Vec::new();
    let mut seen = Vec::new();
    for (index, item) in heads_arr.iter().enumerate() {
        let head = parse_head(item, scale, index, v1)?;
        if seen.iter().any(|id: &String| id == &head.id) {
            return Err(format!("duplicate head id `{}`", head.id));
        }
        seen.push(head.id.clone());
        heads.push(head);
    }
    Ok(heads)
}

fn migrate_v1_heads(mut heads: Vec<MachineHead>) -> Vec<MachineHead> {
    for head in &mut heads {
        if head.kind.is_empty() {
            head.kind = kind_from_process(&head.process);
        }
        for material in &mut head.materials {
            if let Some(power) = material.power
                && power.unit == PowerUnit::Percent
                && (0.0..=1.0).contains(&power.value)
            {
                // Already converted in parse_material_v1.
            }
        }
    }
    heads
}

/// Infer a head kind from a v1 process id.
#[must_use]
pub fn kind_from_process(process: &str) -> String {
    match process {
        "laser" => "laser".to_string(),
        "plasma" => "plasma".to_string(),
        "milling" | "routing" => "spindle".to_string(),
        "fdm" => "extruder".to_string(),
        other => other.to_string(),
    }
}

/// Job type implied by a machine and (for a gantry) the selected head.
#[must_use]
pub fn job_type_for(machine: &MachineFile, head: Option<&MachineHead>) -> JobType {
    if machine.kind == MachineKind::Resin {
        return JobType::Print;
    }
    match head.map(|head| head.kind.as_str()) {
        Some("extruder") | Some("sla") | Some("resin") => JobType::Print,
        _ => JobType::CutRoute,
    }
}

fn bed_from_work_area(area: Option<WorkArea>) -> Option<BedSize> {
    let area = area?;
    let (Some(x_min), Some(x_max), Some(y_min), Some(y_max)) = (area.x_min, area.x_max, area.y_min, area.y_max) else {
        return None;
    };
    Some(BedSize {
        x: x_max - x_min,
        y: y_max - y_min,
        z: match (area.z_min, area.z_max) {
            (Some(z_min), Some(z_max)) => Some(z_max - z_min),
            _ => None,
        },
    })
}

/// Serialise a profile for disk. Lengths are written in the profile's `units`.
pub fn profile_to_json(profile: &MachineFile) -> Value {
    let scale = if profile.units == "inch" { 25.4 } else { 1.0 };
    let mut map = Map::new();
    map.insert("format".to_string(), json!(FORMAT_TAG));
    map.insert("name".to_string(), json!(profile.name));
    map.insert("units".to_string(), json!(profile.units));
    map.insert("type".to_string(), json!(profile.kind.as_str()));
    if let Some(bed) = profile.bed {
        map.insert("bed".to_string(), bed_to_json(bed, scale));
    }
    if let Some(screen) = profile.screen {
        map.insert("screen".to_string(), bed_to_json(screen, scale));
    }
    if let Some(area) = profile.work_area {
        map.insert("work_area".to_string(), work_area_to_json(area, scale));
    }
    if let Some(controller) = &profile.controller {
        map.insert("controller".to_string(), json!(controller));
    }
    if let Some(post) = &profile.post {
        map.insert("post".to_string(), json!(post));
    }
    if profile.kind == MachineKind::Gantry || !profile.heads.is_empty() {
        let heads: Vec<Value> = profile.heads.iter().map(|head| head_to_json(head, scale)).collect();
        map.insert("heads".to_string(), Value::Array(heads));
    }
    Value::Object(map)
}

fn bed_to_json(bed: BedSize, scale: f64) -> Value {
    let mut map = Map::new();
    map.insert("x".to_string(), json!(bed.x / scale));
    map.insert("y".to_string(), json!(bed.y / scale));
    if let Some(z) = bed.z {
        map.insert("z".to_string(), json!(z / scale));
    }
    Value::Object(map)
}

fn parse_bed(value: &Value, scale: f64, what: &str) -> Result<BedSize, String> {
    let map = object(value, what).map_err(|err| err.to_string())?;
    check_keys(map, &["x", "y", "z"], what).map_err(|err| err.to_string())?;
    let x = require_length(map, "x", scale, what)?;
    let y = require_length(map, "y", scale, what)?;
    if x <= 0.0 || y <= 0.0 {
        return Err(format!("{what} x and y must be greater than zero"));
    }
    let z = optional_length(map, "z", scale)?;
    if let Some(z) = z
        && z <= 0.0
    {
        return Err(format!("{what} z must be greater than zero"));
    }
    Ok(BedSize { x, y, z })
}

fn require_length(map: &Map<String, Value>, key: &str, scale: f64, what: &str) -> Result<f64, String> {
    let value = require(map, key).map_err(|err| err.to_string())?;
    let number = as_f64(value, &format!("{what}.{key}")).map_err(|err| err.to_string())?;
    if !number.is_finite() {
        return Err(format!("{what}.{key} is not finite"));
    }
    Ok(number * scale)
}

fn work_area_to_json(area: WorkArea, scale: f64) -> Value {
    let mut map = Map::new();
    for (key, value) in
        [("x_min", area.x_min), ("x_max", area.x_max), ("y_min", area.y_min), ("y_max", area.y_max), ("z_min", area.z_min), ("z_max", area.z_max)]
    {
        if let Some(value) = value {
            map.insert(key.to_string(), json!(value / scale));
        }
    }
    Value::Object(map)
}

fn head_to_json(head: &MachineHead, scale: f64) -> Value {
    let mut map = Map::new();
    map.insert("id".to_string(), json!(head.id));
    if !head.name.is_empty() {
        map.insert("name".to_string(), json!(head.name));
    }
    map.insert("kind".to_string(), json!(head.kind));
    map.insert("process".to_string(), json!(head.process));
    if let Some(post) = &head.post {
        map.insert("post".to_string(), json!(post));
    }
    if let Some(offset) = head.offset {
        let mut off = Map::new();
        off.insert("x".to_string(), json!(offset.x / scale));
        off.insert("y".to_string(), json!(offset.y / scale));
        off.insert("z".to_string(), json!(offset.z / scale));
        map.insert("offset".to_string(), Value::Object(off));
    }
    if let Some(safe_z) = head.safe_z {
        map.insert("safe_z".to_string(), json!(safe_z / scale));
    }
    if let Some(max_power) = head.max_power {
        map.insert("max_power".to_string(), power_to_json(max_power));
    }
    if !head.materials.is_empty() {
        let materials: Vec<Value> = head.materials.iter().map(|item| material_to_json(item, scale)).collect();
        map.insert("materials".to_string(), Value::Array(materials));
    }
    Value::Object(map)
}

fn power_to_json(power: PowerValue) -> Value {
    let mut map = Map::new();
    map.insert("unit".to_string(), json!(power.unit.as_str()));
    map.insert("value".to_string(), json!(power.value));
    Value::Object(map)
}

fn material_to_json(entry: &MaterialEntry, scale: f64) -> Value {
    let mut map = Map::new();
    map.insert("name".to_string(), json!(entry.name));
    let length_keys =
        [("kerf", entry.kerf), ("diameter", entry.diameter), ("wire", entry.wire), ("depth", entry.depth), ("step_down", entry.step_down)];
    for (key, value) in length_keys {
        if let Some(value) = value {
            map.insert(key.to_string(), json!(value / scale));
        }
    }
    if let Some(value) = entry.cut_feed {
        map.insert("cut_feed".to_string(), json!(value));
    }
    if let Some(value) = entry.plunge_feed {
        map.insert("plunge_feed".to_string(), json!(value));
    }
    if let Some(value) = entry.power {
        map.insert("power".to_string(), power_to_json(value));
    }
    if let Some(value) = entry.passes {
        map.insert("passes".to_string(), json!(value));
    }
    if let Some(value) = entry.pierce_delay {
        map.insert("pierce_delay".to_string(), json!(value));
    }
    if let Some(value) = entry.post_delay {
        map.insert("post_delay".to_string(), json!(value));
    }
    Value::Object(map)
}

fn parse_work_area(value: &Value, scale: f64) -> Result<WorkArea, String> {
    let map = object(value, "work_area").map_err(|err| err.to_string())?;
    check_keys(map, &["x_min", "x_max", "y_min", "y_max", "z_min", "z_max"], "work_area").map_err(|err| err.to_string())?;
    let area = WorkArea {
        x_min: optional_length(map, "x_min", scale)?,
        x_max: optional_length(map, "x_max", scale)?,
        y_min: optional_length(map, "y_min", scale)?,
        y_max: optional_length(map, "y_max", scale)?,
        z_min: optional_length(map, "z_min", scale)?,
        z_max: optional_length(map, "z_max", scale)?,
    };
    if area.x_min.is_none() && area.x_max.is_none() && area.y_min.is_none() && area.y_max.is_none() && area.z_min.is_none() && area.z_max.is_none() {
        return Err("work_area needs at least one limit".to_string());
    }
    check_pair("x", area.x_min, area.x_max)?;
    check_pair("y", area.y_min, area.y_max)?;
    check_pair("z", area.z_min, area.z_max)?;
    Ok(area)
}

fn check_pair(axis: &str, min: Option<f64>, max: Option<f64>) -> Result<(), String> {
    if let (Some(min), Some(max)) = (min, max)
        && min > max
    {
        return Err(format!("work_area.{axis}_min is greater than {axis}_max"));
    }
    Ok(())
}

fn parse_head(value: &Value, scale: f64, index: usize, v1: bool) -> Result<MachineHead, String> {
    let map = object(value, &format!("heads[{index}]")).map_err(|err| err.to_string())?;
    let what = format!("heads[{index}]");
    let allowed = if v1 {
        ["id", "name", "process", "post", "materials"].as_slice()
    } else {
        ["id", "name", "kind", "process", "post", "offset", "safe_z", "max_power", "materials"].as_slice()
    };
    check_keys(map, allowed, &what).map_err(|err| err.to_string())?;
    let id = as_str(require(map, "id")?, &format!("{what}.id")).map_err(|err| err.to_string())?.to_string();
    if id.trim().is_empty() {
        return Err(format!("{what}.id is empty"));
    }
    let name = optional_string(map, "name")?.unwrap_or_default();
    let process = as_str(require(map, "process")?, &format!("{what}.process")).map_err(|err| err.to_string())?.to_string();
    if !PROCESS_IDS.contains(&process.as_str()) {
        return Err(format!("{what}.process must be one of {}", PROCESS_IDS.join(", ")));
    }
    let kind = match optional_string(map, "kind")? {
        Some(kind) => {
            if kind.trim().is_empty() {
                return Err(format!("{what}.kind is empty"));
            }
            kind
        }
        None => kind_from_process(&process),
    };
    let post = optional_string(map, "post")?;
    let offset = match optional(map, "offset") {
        Some(value) => Some(parse_offset(value, scale, &format!("{what}.offset"))?),
        None => None,
    };
    let safe_z = optional_length(map, "safe_z", scale)?;
    let max_power = match optional(map, "max_power") {
        Some(value) => Some(parse_power(value, &format!("{what}.max_power"), false)?),
        None => None,
    };
    let materials = match optional(map, "materials") {
        Some(value) => {
            let Some(items) = value.as_array() else {
                return Err(format!("{what}.materials must be an array"));
            };
            let mut materials = Vec::new();
            for (mat_index, item) in items.iter().enumerate() {
                materials.push(parse_material(item, scale, &format!("{what}.materials[{mat_index}]"), v1)?);
            }
            materials
        }
        None => Vec::new(),
    };
    Ok(MachineHead { id, name, kind, process, post, offset, safe_z, max_power, materials })
}

fn parse_offset(value: &Value, scale: f64, what: &str) -> Result<HeadOffset, String> {
    let map = object(value, what).map_err(|err| err.to_string())?;
    check_keys(map, &["x", "y", "z"], what).map_err(|err| err.to_string())?;
    Ok(HeadOffset { x: require_length(map, "x", scale, what)?, y: require_length(map, "y", scale, what)?, z: require_length(map, "z", scale, what)? })
}

fn parse_material(value: &Value, scale: f64, what: &str, v1: bool) -> Result<MaterialEntry, String> {
    let map = object(value, what).map_err(|err| err.to_string())?;
    let keys = if v1 {
        ["name", "kerf", "diameter", "wire", "depth", "step_down", "cut_feed", "plunge_feed", "power", "pierce_delay", "post_delay"].as_slice()
    } else {
        ["name", "kerf", "diameter", "wire", "depth", "step_down", "cut_feed", "plunge_feed", "power", "passes", "pierce_delay", "post_delay"]
            .as_slice()
    };
    check_keys(map, keys, what).map_err(|err| err.to_string())?;
    let name = as_str(require(map, "name")?, &format!("{what}.name")).map_err(|err| err.to_string())?.to_string();
    if name.trim().is_empty() {
        return Err(format!("{what}.name is empty"));
    }
    let power = match optional(map, "power") {
        Some(value) => Some(parse_power(value, &format!("{what}.power"), v1)?),
        None => None,
    };
    let passes = match optional(map, "passes") {
        Some(value) => Some(parse_passes(value, &format!("{what}.passes"))?),
        None => None,
    };
    Ok(MaterialEntry {
        name,
        kerf: optional_length(map, "kerf", scale)?,
        diameter: optional_length(map, "diameter", scale)?,
        wire: optional_length(map, "wire", scale)?,
        depth: optional_length(map, "depth", scale)?,
        step_down: optional_length(map, "step_down", scale)?,
        cut_feed: optional_plain(map, "cut_feed")?,
        plunge_feed: optional_plain(map, "plunge_feed")?,
        power,
        passes,
        pierce_delay: optional_plain(map, "pierce_delay")?,
        post_delay: optional_plain(map, "post_delay")?,
    })
}

fn parse_passes(value: &Value, what: &str) -> Result<u32, String> {
    let number = as_f64(value, what).map_err(|err| err.to_string())?;
    if !number.is_finite() || number < 1.0 || number.fract() != 0.0 || number > 1_000.0 {
        return Err(format!("{what} must be a whole number in 1..=1000"));
    }
    Ok(number as u32)
}

/// Parse a v2 `{unit, value}` power object, or a v1 `0..=1` fraction.
fn parse_power(value: &Value, what: &str, v1: bool) -> Result<PowerValue, String> {
    if let Some(number) = value.as_f64() {
        if !v1 {
            return Err(format!("{what} must be an object with unit and value, not a bare number"));
        }
        if !number.is_finite() || !(0.0..=1.0).contains(&number) {
            return Err(format!("{what} must be a finite number in 0..=1"));
        }
        return Ok(PowerValue::from_v1_fraction(number));
    }
    let map = object(value, what).map_err(|err| err.to_string())?;
    check_keys(map, &["unit", "value"], what).map_err(|err| err.to_string())?;
    let unit = PowerUnit::parse(as_str(require(map, "unit")?, &format!("{what}.unit")).map_err(|err| err.to_string())?)?;
    let number = as_f64(require(map, "value")?, &format!("{what}.value")).map_err(|err| err.to_string())?;
    let power = PowerValue { unit, value: number };
    // Validate using a placeholder max for watts; the value itself must still be finite and non-negative.
    match unit {
        PowerUnit::Percent => {
            let _ = power.to_fraction(None)?;
        }
        PowerUnit::Watts => {
            if !number.is_finite() || number < 0.0 {
                return Err(format!("{what}.value must be a finite number that is not negative"));
            }
        }
    }
    Ok(power)
}

/// Parse the `machine` object on a `campanion-job/1` file.
pub fn parse_job_machine_ref(value: &Value) -> Result<JobMachineRef, String> {
    let map = object(value, "machine").map_err(|err| err.to_string())?;
    if map.contains_key("model") {
        return Err("machine.model is only valid on a lathe job; use machine.profile or machine.path for a user machine profile".to_string());
    }
    check_keys(map, &["profile", "path", "head"], "machine").map_err(|err| err.to_string())?;
    let profile = optional_string(map, "profile")?;
    let path = optional_string(map, "path")?.map(PathBuf::from);
    let head = optional_string(map, "head")?;
    if profile.is_none() && path.is_none() {
        return Err("machine needs a profile or a path".to_string());
    }
    if profile.is_some() && path.is_some() {
        return Err("machine cannot set both profile and path".to_string());
    }
    Ok(JobMachineRef { profile, path, head })
}

/// Load the profile named by `reference`.
pub fn load_job_ref(reference: &JobMachineRef, store: &ProfileStore) -> Result<MachineFile, CliError> {
    if let Some(path) = &reference.path {
        return load_path(path);
    }
    let id = reference.profile.as_deref().ok_or_else(|| CliError::run("machine needs a profile or a path"))?;
    store.load(id)
}

/// The head selected by `reference`, or the first head when `head` is omitted.
pub fn selected_head<'a>(profile: &'a MachineFile, reference: &JobMachineRef) -> Result<&'a MachineHead, CliError> {
    match &reference.head {
        Some(id) => profile
            .heads
            .iter()
            .find(|head| head.id == *id)
            .ok_or_else(|| CliError::run(format!("machine profile `{}` has no head `{id}`", profile.name))),
        None => {
            if profile.kind == MachineKind::Resin {
                return Err(CliError::run(format!(
                    "machine profile `{}` is a resin printer and has no heads; a cutting job cannot select a head on it",
                    profile.name
                )));
            }
            profile.heads.first().ok_or_else(|| CliError::run(format!("machine profile `{}` has no heads", profile.name)))
        }
    }
}

/// Effective post for the selected head: head post, else machine post.
pub fn effective_post(profile: &MachineFile, head: &MachineHead) -> Option<String> {
    head.post.clone().or_else(|| profile.post.clone())
}

/// Compare toolpath bounds to a user-entered work area. Only axes the user filled are checked.
///
/// A value equal to the limit passes. A value outside the entered range is a refusal.
pub fn check_work_area(profile_name: &str, area: WorkArea, min: [f64; 3], max: [f64; 3]) -> Result<(), CliError> {
    let checks =
        [("X", min[0], max[0], area.x_min, area.x_max), ("Y", min[1], max[1], area.y_min, area.y_max), ("Z", min[2], max[2], area.z_min, area.z_max)];
    for (axis, span_min, span_max, limit_min, limit_max) in checks {
        if let Some(limit) = limit_min
            && span_min < limit
        {
            return Err(CliError::run(format!(
                "machine `{profile_name}`: toolpath {axis} minimum {span_min} mm is below the work area minimum {limit} mm"
            )));
        }
        if let Some(limit) = limit_max
            && span_max > limit
        {
            return Err(CliError::run(format!(
                "machine `{profile_name}`: toolpath {axis} maximum {span_max} mm exceeds the work area maximum {limit} mm"
            )));
        }
    }
    Ok(())
}

fn require<'a>(map: &'a Map<String, Value>, key: &str) -> Result<&'a Value, String> {
    map.get(key).ok_or_else(|| format!("missing {key}"))
}

fn optional_string(map: &Map<String, Value>, key: &str) -> Result<Option<String>, String> {
    match optional(map, key) {
        Some(value) => {
            let text = as_str(value, key).map_err(|err| err.to_string())?.to_string();
            if text.trim().is_empty() {
                return Err(format!("{key} is empty"));
            }
            Ok(Some(text))
        }
        None => Ok(None),
    }
}

fn optional_length(map: &Map<String, Value>, key: &str, scale: f64) -> Result<Option<f64>, String> {
    match optional(map, key) {
        Some(value) => {
            let number = as_f64(value, key).map_err(|err| err.to_string())?;
            if !number.is_finite() {
                return Err(format!("{key} is not finite"));
            }
            Ok(Some(number * scale))
        }
        None => Ok(None),
    }
}

fn optional_plain(map: &Map<String, Value>, key: &str) -> Result<Option<f64>, String> {
    match optional(map, key) {
        Some(value) => {
            let number = as_f64(value, key).map_err(|err| err.to_string())?;
            if !number.is_finite() {
                return Err(format!("{key} is not finite"));
            }
            Ok(Some(number))
        }
        None => Ok(None),
    }
}

/// Resolve the job's machine reference and refuse a toolpath that leaves the entered work area.
pub fn validate_job_machine(job: &crate::jobfile::JobFile, program: &campanion_ir::Program) -> Result<(), CliError> {
    let Some(reference) = job.machine() else {
        return Ok(());
    };
    let store = ProfileStore::from_env()?;
    let profile = load_job_ref(reference, &store)?;
    let head = selected_head(&profile, reference)?;
    let _post = effective_post(&profile, head);
    let Some(area) = profile.work_area else {
        return Ok(());
    };
    let report = campanion_ir::verify(program);
    let Some(bounds) = report.bounds() else {
        return Ok(());
    };
    let min = bounds.min();
    let max = bounds.max();
    check_work_area(&profile.name, area, [min.x(), min.y(), min.z()], [max.x(), max.y(), max.z()])
}

impl MachineFile {
    /// An empty profile the GUI can fill in. No numeric defaults are invented.
    pub fn blank(name: impl Into<String>, process: &str) -> Self {
        let process = if PROCESS_IDS.contains(&process) { process } else { "laser" };
        Self {
            name: name.into(),
            units: "mm".to_string(),
            kind: MachineKind::Gantry,
            bed: None,
            screen: None,
            work_area: None,
            controller: None,
            post: None,
            heads: vec![MachineHead {
                id: "head1".to_string(),
                name: String::new(),
                kind: kind_from_process(process),
                process: process.to_string(),
                post: None,
                offset: None,
                safe_z: None,
                max_power: None,
                materials: Vec::new(),
            }],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> &'static str {
        r#"{
            "format": "campanion-machine/1",
            "name": "Shop laser",
            "units": "mm",
            "work_area": { "x_min": 0, "x_max": 600, "y_min": 0, "y_max": 400 },
            "controller": "grbl",
            "post": "grbl_laser",
            "heads": [{
                "id": "laser",
                "name": "CO2",
                "process": "laser",
                "post": "grbl_laser",
                "materials": [{ "name": "3 mm acrylic", "kerf": 0.15, "cut_feed": 300, "power": 0.8, "depth": 3 }]
            }]
        }"#
    }

    #[test]
    fn parse_round_trip_keeps_user_values_and_scales_inches() {
        let profile = parse_profile(sample()).expect("parse");
        assert_eq!(profile.name, "Shop laser");
        assert_eq!(profile.kind, MachineKind::Gantry);
        assert_eq!(profile.bed, Some(BedSize { x: 600.0, y: 400.0, z: None }));
        assert_eq!(profile.heads[0].process, "laser");
        assert_eq!(profile.heads[0].kind, "laser");
        assert_eq!(effective_post(&profile, &profile.heads[0]).as_deref(), Some("grbl_laser"));
        assert_eq!(profile.heads[0].materials[0].kerf, Some(0.15));
        assert_eq!(profile.heads[0].materials[0].cut_feed, Some(300.0));
        assert_eq!(profile.heads[0].materials[0].power, Some(PowerValue::percent(80.0)));
        let area = profile.work_area.expect("area");
        assert_eq!(area.x_max, Some(600.0));
        let json = profile_to_json(&profile);
        assert_eq!(json["format"], "campanion-machine/2");
        assert_eq!(json["type"], "gantry");
        assert_eq!(json["heads"][0]["materials"][0]["power"]["unit"], "percent");
        assert!((json["heads"][0]["materials"][0]["power"]["value"].as_f64().unwrap() - 80.0).abs() < 1e-9);
        let again = parse_profile_value(&json).expect("reparse");
        assert_eq!(again, profile);

        let inch = r#"{
            "format": "campanion-machine/1",
            "name": "Inch machine",
            "units": "inch",
            "work_area": { "x_max": 12 },
            "heads": [{ "id": "h", "process": "routing" }]
        }"#;
        let profile = parse_profile(inch).expect("inch");
        assert!((profile.work_area.unwrap().x_max.unwrap() - 304.8).abs() < 1e-9);
        let written = profile_to_json(&profile);
        assert!((written["work_area"]["x_max"].as_f64().unwrap() - 12.0).abs() < 1e-9);
    }

    #[test]
    fn rejects_unknown_keys_bad_process_and_empty_library_is_ok() {
        let err = parse_profile(r#"{"format":"campanion-machine/1","name":"A","units":"mm","heads":[{"id":"h","process":"laser","color":"red"}]}"#)
            .expect_err("key");
        assert!(err.contains("heads[0]"), "{err}");
        let err =
            parse_profile(r#"{"format":"campanion-machine/1","name":"A","units":"mm","heads":[{"id":"h","process":"spark"}]}"#).expect_err("proc");
        assert!(err.contains("process must be one of"), "{err}");
        let profile =
            parse_profile(r#"{"format":"campanion-machine/1","name":"A","units":"mm","heads":[{"id":"h","process":"plasma"}]}"#).expect("ok");
        assert!(profile.heads[0].materials.is_empty());
        assert!(profile.work_area.is_none());
        assert!(profile.post.is_none());
    }

    #[test]
    fn job_ref_rejects_lathe_model_and_needs_profile_or_path() {
        let err = parse_job_machine_ref(&json!({"model":"haas_st20y"})).expect_err("model");
        assert!(err.contains("machine.model is only valid on a lathe job"), "{err}");
        let err = parse_job_machine_ref(&json!({})).expect_err("empty");
        assert!(err.contains("profile or a path"), "{err}");
        let reference = parse_job_machine_ref(&json!({"profile":"shop-laser","head":"laser"})).expect("ref");
        assert_eq!(reference.profile.as_deref(), Some("shop-laser"));
        assert_eq!(reference.head.as_deref(), Some("laser"));
    }

    #[test]
    fn work_area_check_refuses_overtravel_and_allows_equal_limit() {
        let area = WorkArea { x_min: Some(0.0), x_max: Some(100.0), y_min: None, y_max: None, z_min: None, z_max: None };
        check_work_area("shop", area, [0.0, 0.0, 0.0], [100.0, 10.0, 0.0]).map_err(|err| err.to_string()).expect("equal");
        let err = check_work_area("shop", area, [-0.001, 0.0, 0.0], [10.0, 0.0, 0.0]).map_err(|err| err.to_string()).expect_err("under");
        let text = err;
        assert!(text.contains("X minimum"), "{text}");
        let err = check_work_area("shop", area, [0.0, 0.0, 0.0], [100.001, 0.0, 0.0]).map_err(|err| err.to_string()).expect_err("over");
        let text = err;
        assert!(text.contains("X maximum"), "{text}");
        assert!(text.contains("shop"), "{text}");
    }

    #[test]
    fn store_round_trip_file() {
        let dir = std::env::temp_dir().join(format!("campanion-profile-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("dir");
        let store = ProfileStore::from_dir(dir.clone());
        let profile = parse_profile(sample()).expect("parse");
        let path = store.save("shop-laser", &profile).map_err(|err| err.to_string()).expect("save");
        assert_eq!(path, dir.join("machines").join("shop-laser.json"));
        let listed = store.list().map_err(|err| err.to_string()).expect("list");
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].0, "shop-laser");
        let loaded = store.load("shop-laser").map_err(|err| err.to_string()).expect("load");
        assert_eq!(loaded.name, "Shop laser");
        store.delete("shop-laser").map_err(|err| err.to_string()).expect("delete");
        assert!(store.list().map_err(|err| err.to_string()).expect("empty").is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn blank_profile_has_no_invented_values() {
        let profile = MachineFile::blank("New machine", "laser");
        assert!(profile.work_area.is_none());
        assert!(profile.post.is_none());
        assert!(profile.heads[0].materials.is_empty());
        assert_eq!(profile.heads[0].process, "laser");
        assert_eq!(profile.kind, MachineKind::Gantry);
        assert!(profile.bed.is_none());
        assert!(profile.heads[0].offset.is_none());
        assert!(profile.heads[0].safe_z.is_none());
        assert!(effective_post(&profile, &profile.heads[0]).is_none());
    }

    #[test]
    fn v1_incomplete_work_area_does_not_invent_a_bed() {
        let profile = parse_profile(
            r#"{
                "format":"campanion-machine/1",
                "name":"Inch machine",
                "units":"inch",
                "work_area":{"x_max":12},
                "heads":[{"id":"h","process":"routing"}]
            }"#,
        )
        .expect("parse");
        assert_eq!(profile.kind, MachineKind::Gantry);
        assert!(profile.bed.is_none());
        assert_eq!(profile.heads[0].kind, "spindle");
    }

    #[test]
    fn v2_gantry_keeps_offsets_safe_z_and_watt_power() {
        let text = r#"{
            "format": "campanion-machine/2",
            "name": "Mix",
            "units": "mm",
            "type": "gantry",
            "bed": { "x": 400, "y": 300 },
            "heads": [
                {
                    "id": "laser",
                    "kind": "laser",
                    "process": "laser",
                    "offset": { "x": 1, "y": 2, "z": 3 },
                    "safe_z": 15,
                    "max_power": { "unit": "w", "value": 40 },
                    "materials": [{ "name": "ply", "cut_feed": 400, "power": { "unit": "w", "value": 32 }, "passes": 2 }]
                },
                { "id": "spindle", "kind": "spindle", "process": "routing" }
            ]
        }"#;
        let profile = parse_profile(text).expect("parse");
        assert_eq!(profile.kind, MachineKind::Gantry);
        assert_eq!(profile.bed, Some(BedSize { x: 400.0, y: 300.0, z: None }));
        assert_eq!(profile.heads[0].offset, Some(HeadOffset { x: 1.0, y: 2.0, z: 3.0 }));
        assert_eq!(profile.heads[0].safe_z, Some(15.0));
        assert_eq!(profile.heads[0].max_power, Some(PowerValue::watts(40.0)));
        assert_eq!(profile.heads[0].materials[0].power, Some(PowerValue::watts(32.0)));
        assert_eq!(profile.heads[0].materials[0].passes, Some(2));
        let fraction = profile.heads[0].materials[0].power.unwrap().to_fraction(Some(40.0)).expect("fraction");
        assert!((fraction - 0.8).abs() < 1e-12);
        assert_eq!(job_type_for(&profile, Some(&profile.heads[0])), JobType::CutRoute);
        assert_eq!(job_type_for(&profile, Some(&profile.heads[1])), JobType::CutRoute);
        let again = parse_profile_value(&profile_to_json(&profile)).expect("round trip");
        assert_eq!(again, profile);
    }

    #[test]
    fn v2_resin_has_no_heads_and_is_print() {
        let text = r#"{
            "format": "campanion-machine/2",
            "name": "Mars",
            "units": "mm",
            "type": "resin",
            "bed": { "x": 130, "y": 80, "z": 160 },
            "screen": { "x": 130, "y": 80 }
        }"#;
        let profile = parse_profile(text).expect("resin");
        assert_eq!(profile.kind, MachineKind::Resin);
        assert_eq!(profile.bed, Some(BedSize { x: 130.0, y: 80.0, z: Some(160.0) }));
        assert_eq!(profile.screen, Some(BedSize { x: 130.0, y: 80.0, z: None }));
        assert!(profile.heads.is_empty());
        assert_eq!(job_type_for(&profile, None), JobType::Print);
        let err = parse_profile(
            r#"{
                "format":"campanion-machine/2","name":"Bad","units":"mm","type":"resin",
                "heads":[{"id":"h","process":"sla"}]
            }"#,
        )
        .expect_err("heads");
        assert!(err.contains("no heads"), "{err}");
        let err = selected_head(&profile, &JobMachineRef { profile: Some("mars".into()), path: None, head: None }).expect_err("select");
        assert!(err.to_string().contains("resin printer"), "{err}");
    }

    #[test]
    fn v2_gantry_rejects_empty_heads_and_bare_power_number() {
        let err = parse_profile(r#"{"format":"campanion-machine/2","name":"A","units":"mm","type":"gantry"}"#).expect_err("heads");
        assert!(err.contains("at least one head"), "{err}");
        let err = parse_profile(
            r#"{
                "format":"campanion-machine/2","name":"A","units":"mm","type":"gantry",
                "heads":[{"id":"h","process":"laser","materials":[{"name":"a","power":0.8}]}]
            }"#,
        )
        .expect_err("bare");
        assert!(err.contains("unit and value"), "{err}");
    }

    #[test]
    fn watt_power_refuses_to_invent_a_max() {
        let power = PowerValue::watts(32.0);
        let err = power.to_fraction(None).expect_err("max");
        assert!(err.contains("max_power"), "{err}");
        assert!((power.to_fraction(Some(40.0)).expect("ok") - 0.8).abs() < 1e-12);
    }

    #[test]
    fn extensible_head_kind_is_kept() {
        let profile = parse_profile(
            r#"{
                "format":"campanion-machine/2","name":"Jet","units":"mm","type":"gantry",
                "heads":[{"id":"wj","kind":"waterjet","process":"waterjet"}]
            }"#,
        )
        .expect("kind");
        assert_eq!(profile.heads[0].kind, "waterjet");
        assert_eq!(job_type_for(&profile, Some(&profile.heads[0])), JobType::CutRoute);
    }
}
