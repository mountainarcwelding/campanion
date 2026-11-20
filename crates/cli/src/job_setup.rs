//! Optional Setup hooks on `campanion-job/2` and helpers that do not change omitted files.
//!
//! These fields are omitted by default. A job that does not name them plans and posts the same
//! way as before. When present, they are stored and round-tripped. Datum, Z-zero, start-from /
//! job origin, and automatic `order_rules` are applied at planning time only when set.
//! Explicit `cut_order` still wins over `order_rules`.

use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_f64, as_str, check_keys, object, optional, require};
use crate::jobfile::JobFileError;

/// Z origin for a cutting job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZZero {
    /// Stock top is Z 0 (the existing convention).
    StockTop,
    /// The machine bed is Z 0. Requires a user-entered stock thickness.
    Bed,
}

impl ZZero {
    /// File value.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::StockTop => "stock_top",
            Self::Bed => "bed",
        }
    }

    fn parse(text: &str) -> Result<Self, JobFileError> {
        match text {
            "stock_top" => Ok(Self::StockTop),
            "bed" => Ok(Self::Bed),
            other => Err(JobFileError::invalid(format!("z_zero must be stock_top or bed, not `{other}`"))),
        }
    }
}

/// Which corner or centre of the stock or sheet is X0,Y0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DatumAnchor {
    /// Minimum X and minimum Y (the existing origin when the rectangle starts at 0,0).
    MinMin,
    /// Maximum X and minimum Y.
    MaxMin,
    /// Minimum X and maximum Y.
    MinMax,
    /// Maximum X and maximum Y.
    MaxMax,
    /// Centre of the rectangle.
    Center,
}

impl DatumAnchor {
    /// File value.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::MinMin => "min_min",
            Self::MaxMin => "max_min",
            Self::MinMax => "min_max",
            Self::MaxMax => "max_max",
            Self::Center => "center",
        }
    }

    fn parse(text: &str) -> Result<Self, JobFileError> {
        match text {
            "min_min" => Ok(Self::MinMin),
            "max_min" => Ok(Self::MaxMin),
            "min_max" => Ok(Self::MinMax),
            "max_max" => Ok(Self::MaxMax),
            "center" => Ok(Self::Center),
            other => Err(JobFileError::invalid(format!("datum.anchor must be min_min, max_min, min_max, max_max, or center, not `{other}`"))),
        }
    }
}

/// Job XY origin on the stock or sheet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Datum {
    /// Corner or centre used as X0,Y0.
    pub anchor: DatumAnchor,
    /// Additional typed offset, millimetres, when entered.
    pub offset: Option<[f64; 2]>,
}

/// How Run and Frame place the job relative to the machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartFrom {
    /// Workspace coordinates match the machine bed.
    Absolute,
    /// Output is relative to the head position at start.
    CurrentPosition,
    /// Output is relative to a stored user origin.
    UserOrigin,
}

impl StartFrom {
    /// File value.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Absolute => "absolute",
            Self::CurrentPosition => "current_position",
            Self::UserOrigin => "user_origin",
        }
    }

    fn parse(text: &str) -> Result<Self, JobFileError> {
        match text {
            "absolute" => Ok(Self::Absolute),
            "current_position" => Ok(Self::CurrentPosition),
            "user_origin" => Ok(Self::UserOrigin),
            other => Err(JobFileError::invalid(format!("start_from must be absolute, current_position, or user_origin, not `{other}`"))),
        }
    }
}

/// Nine-position anchor of the job bounds for current-position or user-origin starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobOriginAnchor {
    MinMin,
    MidMin,
    MaxMin,
    MinMid,
    Center,
    MaxMid,
    MinMax,
    MidMax,
    MaxMax,
}

impl JobOriginAnchor {
    /// File value.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::MinMin => "min_min",
            Self::MidMin => "mid_min",
            Self::MaxMin => "max_min",
            Self::MinMid => "min_mid",
            Self::Center => "center",
            Self::MaxMid => "max_mid",
            Self::MinMax => "min_max",
            Self::MidMax => "mid_max",
            Self::MaxMax => "max_max",
        }
    }

    fn parse(text: &str) -> Result<Self, JobFileError> {
        match text {
            "min_min" => Ok(Self::MinMin),
            "mid_min" => Ok(Self::MidMin),
            "max_min" => Ok(Self::MaxMin),
            "min_mid" => Ok(Self::MinMid),
            "center" => Ok(Self::Center),
            "max_mid" => Ok(Self::MaxMid),
            "min_max" => Ok(Self::MinMax),
            "mid_max" => Ok(Self::MidMax),
            "max_max" => Ok(Self::MaxMax),
            other => Err(JobFileError::invalid(format!("job_origin `{other}` is not a nine-position anchor"))),
        }
    }
}

/// Default cut-order rules. Applied when present and `cut_order` is empty. Explicit `cut_order`
/// still wins.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct OrderRules {
    /// Cut inner contours before outer ones.
    pub inner_first: bool,
    /// Finish marking or engraving before a through cut.
    pub cuts_last: bool,
}

/// Optional Setup fields on a job file.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SetupHooks {
    /// Z origin, when the file sets `z_zero`.
    pub z_zero: Option<ZZero>,
    /// Stock thickness, millimetres, when entered.
    pub stock_thickness: Option<f64>,
    /// XY datum, when entered.
    pub datum: Option<Datum>,
    /// Run / Frame start-from mode, when entered.
    pub start_from: Option<StartFrom>,
    /// Nine-dot job origin, when entered.
    pub job_origin: Option<JobOriginAnchor>,
    /// Automatic order rules, when entered.
    pub order_rules: Option<OrderRules>,
}

/// Parse the optional Setup hook keys from a job object. Lengths in `map` are in file units
/// and are returned in millimetres.
pub fn parse_setup_hooks(map: &Map<String, Value>, scale: f64, stock_height: Option<f64>) -> Result<SetupHooks, JobFileError> {
    let z_zero = match optional(map, "z_zero") {
        Some(value) => Some(ZZero::parse(as_str(value, "z_zero").map_err(from_cli)?)?),
        None => None,
    };
    let stock_thickness = match optional(map, "stock_thickness") {
        Some(value) => {
            let number = as_f64(value, "stock_thickness").map_err(from_cli)? * scale;
            if !number.is_finite() || number <= 0.0 {
                return Err(JobFileError::invalid("stock_thickness must be greater than zero"));
            }
            Some(number)
        }
        None => None,
    };
    if z_zero == Some(ZZero::Bed) && stock_thickness.is_none() && stock_height.is_none() {
        return Err(JobFileError::invalid("z_zero bed needs stock_thickness or a stock box with height"));
    }
    let datum = match optional(map, "datum") {
        Some(value) => Some(parse_datum(value, scale)?),
        None => None,
    };
    let start_from = match optional(map, "start_from") {
        Some(value) => Some(StartFrom::parse(as_str(value, "start_from").map_err(from_cli)?)?),
        None => None,
    };
    let job_origin = match optional(map, "job_origin") {
        Some(value) => Some(JobOriginAnchor::parse(as_str(value, "job_origin").map_err(from_cli)?)?),
        None => None,
    };
    let order_rules = match optional(map, "order_rules") {
        Some(value) => Some(parse_order_rules(value)?),
        None => None,
    };
    Ok(SetupHooks { z_zero, stock_thickness, datum, start_from, job_origin, order_rules })
}

/// Keys these hooks add to the top-level job object.
pub const JOB_HOOK_KEYS: &[&str] = &["z_zero", "stock_thickness", "datum", "start_from", "job_origin", "order_rules"];

/// Translation to add to job XY so the chosen datum sits at the origin.
/// `rect` is `[min_x, min_y, max_x, max_y]` of the stock or sheet, in millimetres.
#[must_use]
pub fn datum_shift(datum: Datum, rect: [f64; 4]) -> [f64; 2] {
    let [min_x, min_y, max_x, max_y] = rect;
    let (x, y) = match datum.anchor {
        DatumAnchor::MinMin => (min_x, min_y),
        DatumAnchor::MaxMin => (max_x, min_y),
        DatumAnchor::MinMax => (min_x, max_y),
        DatumAnchor::MaxMax => (max_x, max_y),
        DatumAnchor::Center => ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0),
    };
    let offset = datum.offset.unwrap_or([0.0, 0.0]);
    [-(x + offset[0]), -(y + offset[1])]
}

/// Amount to add to planned Z so the bed is Z 0, when `z_zero` is bed.
#[must_use]
pub fn bed_z_shift(hooks: &SetupHooks, stock_height: Option<f64>) -> f64 {
    if hooks.z_zero != Some(ZZero::Bed) {
        return 0.0;
    }
    hooks.stock_thickness.or(stock_height).unwrap_or(0.0)
}

/// Point of a nine-position `job_origin` on `rect` (`min_x`, `min_y`, `max_x`, `max_y`).
#[must_use]
pub fn job_origin_point(anchor: JobOriginAnchor, rect: [f64; 4]) -> [f64; 2] {
    let [min_x, min_y, max_x, max_y] = rect;
    let mid_x = (min_x + max_x) * 0.5;
    let mid_y = (min_y + max_y) * 0.5;
    match anchor {
        JobOriginAnchor::MinMin => [min_x, min_y],
        JobOriginAnchor::MidMin => [mid_x, min_y],
        JobOriginAnchor::MaxMin => [max_x, min_y],
        JobOriginAnchor::MinMid => [min_x, mid_y],
        JobOriginAnchor::Center => [mid_x, mid_y],
        JobOriginAnchor::MaxMid => [max_x, mid_y],
        JobOriginAnchor::MinMax => [min_x, max_y],
        JobOriginAnchor::MidMax => [mid_x, max_y],
        JobOriginAnchor::MaxMax => [max_x, max_y],
    }
}

/// XY addend that places the chosen job-origin anchor at program (0, 0).
#[must_use]
pub fn job_origin_shift(anchor: JobOriginAnchor, rect: [f64; 4]) -> [f64; 2] {
    let [x, y] = job_origin_point(anchor, rect);
    [-x, -y]
}

/// Rank for `order_rules.cuts_last`: marking and engraving first, then through cuts.
#[must_use]
pub fn cuts_last_rank(kind: &crate::jobfile::OperationKind) -> u8 {
    use crate::jobfile::OperationKind;
    match kind {
        OperationKind::Engrave | OperationKind::LaserRaster => 0,
        _ => 1,
    }
}

fn parse_datum(value: &Value, scale: f64) -> Result<Datum, JobFileError> {
    let map = object(value, "datum").map_err(from_cli)?;
    check_keys(map, &["anchor", "offset"], "datum").map_err(from_cli)?;
    let anchor = DatumAnchor::parse(as_str(require(map, "anchor").map_err(from_cli)?, "datum.anchor").map_err(from_cli)?)?;
    let offset = match optional(map, "offset") {
        Some(value) => Some(parse_xy(value, scale, "datum.offset")?),
        None => None,
    };
    Ok(Datum { anchor, offset })
}

fn parse_xy(value: &Value, scale: f64, what: &str) -> Result<[f64; 2], JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid(format!("{what} must be an array of two numbers")));
    };
    if items.len() != 2 {
        return Err(JobFileError::invalid(format!("{what} must be an array of two numbers")));
    }
    let x = as_f64(&items[0], &format!("{what}[0]")).map_err(from_cli)? * scale;
    let y = as_f64(&items[1], &format!("{what}[1]")).map_err(from_cli)? * scale;
    if !x.is_finite() || !y.is_finite() {
        return Err(JobFileError::invalid(format!("{what} values must be finite")));
    }
    Ok([x, y])
}

fn parse_order_rules(value: &Value) -> Result<OrderRules, JobFileError> {
    let map = object(value, "order_rules").map_err(from_cli)?;
    check_keys(map, &["inner_first", "cuts_last"], "order_rules").map_err(from_cli)?;
    let inner_first = optional_bool(map, "inner_first")?;
    let cuts_last = optional_bool(map, "cuts_last")?;
    if inner_first.is_none() && cuts_last.is_none() {
        return Err(JobFileError::invalid("order_rules needs inner_first or cuts_last"));
    }
    Ok(OrderRules { inner_first: inner_first.unwrap_or(false), cuts_last: cuts_last.unwrap_or(false) })
}

fn optional_bool(map: &Map<String, Value>, key: &str) -> Result<Option<bool>, JobFileError> {
    match optional(map, key) {
        Some(value) => Ok(Some(crate::job::as_bool(value, &format!("order_rules.{key}")).map_err(from_cli)?)),
        None => Ok(None),
    }
}

fn from_cli(err: CliError) -> JobFileError {
    JobFileError::invalid(err.to_string())
}

/// Serialise hook fields that are present, in file units (`scale` is mm per file unit).
#[cfg(test)]
pub fn hooks_to_json(hooks: &SetupHooks, scale: f64, map: &mut Map<String, Value>) {
    if let Some(z_zero) = hooks.z_zero {
        map.insert("z_zero".into(), Value::String(z_zero.as_str().to_string()));
    }
    if let Some(thickness) = hooks.stock_thickness {
        map.insert("stock_thickness".into(), json_number(thickness / scale));
    }
    if let Some(datum) = hooks.datum {
        let mut datum_map = Map::new();
        datum_map.insert("anchor".into(), Value::String(datum.anchor.as_str().to_string()));
        if let Some([x, y]) = datum.offset {
            datum_map.insert("offset".into(), Value::Array(vec![json_number(x / scale), json_number(y / scale)]));
        }
        map.insert("datum".into(), Value::Object(datum_map));
    }
    if let Some(start_from) = hooks.start_from {
        map.insert("start_from".into(), Value::String(start_from.as_str().to_string()));
    }
    if let Some(job_origin) = hooks.job_origin {
        map.insert("job_origin".into(), Value::String(job_origin.as_str().to_string()));
    }
    if let Some(rules) = hooks.order_rules {
        let mut rules_map = Map::new();
        rules_map.insert("inner_first".into(), Value::Bool(rules.inner_first));
        rules_map.insert("cuts_last".into(), Value::Bool(rules.cuts_last));
        map.insert("order_rules".into(), Value::Object(rules_map));
    }
}

#[cfg(test)]
fn json_number(value: f64) -> Value {
    serde_json::Number::from_f64(value).map(Value::Number).unwrap_or(Value::Null)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jobfile::{self, FORMAT_TAG, FORMAT_TAG_V2};
    use std::path::Path;

    fn parse_hooks(extra: &str) -> Result<SetupHooks, JobFileError> {
        let text = format!(
            r#"{{"format":"{FORMAT_TAG_V2}","units":"mm","parts":[{{"id":"p","source":{{"path":"a.svg","hash":"cbf29ce484222325"}}}}],"tools":[{{"id":1,"kerf":0.1}}],"operations":[{{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{{"cut":1}}}}]{extra}}}"#
        );
        let job = jobfile::parse(&text, Path::new("hooks.job.json"))?;
        Ok(job.setup_hooks().clone())
    }

    #[test]
    fn omitted_hooks_are_empty() {
        let hooks = parse_hooks("").expect("parse");
        assert_eq!(hooks, SetupHooks::default());
    }

    #[test]
    fn parses_and_round_trips_hooks() {
        let extra = r#","z_zero":"bed","stock_thickness":12,"datum":{"anchor":"center","offset":[1,2]},"start_from":"current_position","job_origin":"center","order_rules":{"inner_first":true,"cuts_last":true}"#;
        let hooks = parse_hooks(extra).expect("parse");
        assert_eq!(hooks.z_zero, Some(ZZero::Bed));
        assert_eq!(hooks.stock_thickness, Some(12.0));
        assert_eq!(hooks.datum.unwrap().anchor, DatumAnchor::Center);
        assert_eq!(hooks.datum.unwrap().offset, Some([1.0, 2.0]));
        assert_eq!(hooks.start_from, Some(StartFrom::CurrentPosition));
        assert_eq!(hooks.job_origin, Some(JobOriginAnchor::Center));
        assert_eq!(hooks.order_rules, Some(OrderRules { inner_first: true, cuts_last: true }));
        let mut map = Map::new();
        hooks_to_json(&hooks, 1.0, &mut map);
        let again = parse_setup_hooks(&map, 1.0, None).expect("reparse");
        assert_eq!(again, hooks);
    }

    #[test]
    fn bed_z_zero_without_thickness_is_refused() {
        let err = parse_hooks(r#","z_zero":"bed""#).expect_err("thickness");
        assert!(err.to_string().contains("stock_thickness"), "{err}");
    }

    #[test]
    fn unknown_hook_field_is_rejected() {
        let err = parse_hooks(r#","datum":{"anchor":"center","foo":1}"#).expect_err("unknown");
        assert!(err.to_string().contains("unknown"), "{err}");
    }

    #[test]
    fn datum_shift_moves_center_to_origin() {
        let datum = Datum { anchor: DatumAnchor::Center, offset: None };
        assert_eq!(datum_shift(datum, [0.0, 0.0, 100.0, 40.0]), [-50.0, -20.0]);
    }

    #[test]
    fn v1_format_tag_is_unchanged() {
        assert_eq!(FORMAT_TAG, "campanion-job/1");
    }
}
