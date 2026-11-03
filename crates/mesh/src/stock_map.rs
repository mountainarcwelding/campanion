//! Compact height map of an in-process stock model.
//!
//! The grid matches [`crate::StockModel`]: `origin` is the minimum corner, `cell` is the XY pitch,
//! and `top` is row-major (`iy * nx + ix`). An empty column is null. `multi` lists columns that
//! hold more than one solid interval, so a viewer can mark an undercut the single top Z hides.
//!
//! `length` is tool travel in millimetres, the same axis the preview scrub uses. Straight moves
//! match that scrub. An XY arc uses its arc length, which the preview shortens by less than a
//! chord. `snapshots` are earlier maps. The top-level `top` is the stock at `length`.
//!
//! This file is not a program for a machine.

use std::str::FromStr;

use serde_json::{Map, Number, Value};

use crate::{MeshError, invalid, parse_error};

/// On-disk label for [`StockFile`].
pub const STOCK_FORMAT: &str = "campanion-stock/1";

/// One XY grid of top heights.
#[derive(Clone, Debug, PartialEq)]
pub struct StockGrid {
    /// Minimum corner of cell `(0, 0)`, millimetres.
    pub origin: [f64; 2],
    /// Cell pitch, millimetres.
    pub cell: f64,
    /// Columns.
    pub nx: usize,
    /// Rows.
    pub ny: usize,
    /// Top Z, row-major. `None` is an empty column.
    pub top: Vec<Option<f64>>,
    /// Column indices (`iy * nx + ix`) with more than one solid interval, ascending.
    pub multi: Vec<usize>,
}

/// Stock at one tool-travel position.
#[derive(Clone, Debug, PartialEq)]
pub struct StockSnapshot {
    /// Tool travel from the start of the program, in millimetres.
    pub position: f64,
    /// Top Z, same order and length as [`StockGrid::top`].
    pub top: Vec<Option<f64>>,
    /// Columns with more than one solid interval.
    pub multi: Vec<usize>,
}

/// Final stock, plus optional earlier maps.
#[derive(Clone, Debug, PartialEq)]
pub struct StockFile {
    /// Grid of the final stock.
    pub grid: StockGrid,
    /// Tool travel at the final stock, millimetres.
    pub length: f64,
    /// Earlier states, in increasing position. Empty when the run did not ask for them.
    pub snapshots: Vec<StockSnapshot>,
}

impl StockFile {
    /// JSON text. Heights use the shortest round-trip decimal.
    pub fn to_json(&self) -> Result<String, MeshError> {
        let value = Value::Object(file_object(self)?);
        serde_json::to_string_pretty(&value).map_err(|err| invalid(format!("stock file: {err}")))
    }

    /// Parse a [`STOCK_FORMAT`] document.
    pub fn from_json(text: &str) -> Result<Self, MeshError> {
        let value: Value = serde_json::from_str(text).map_err(|err| parse_error(format!("stock file: {err}")))?;
        let map = object(&value, "stock file")?;
        check_keys(map, &["format", "origin", "cell", "nx", "ny", "length", "top", "multi", "snapshots"], "stock file")?;
        let format = string(require(map, "format")?, "format")?;
        if format != STOCK_FORMAT {
            return Err(parse_error(format!("stock file format is `{format}`, expected `{STOCK_FORMAT}`")));
        }
        let origin = pair(require(map, "origin")?, "origin")?;
        let cell = finite_positive(number(require(map, "cell")?, "cell")?, "cell")?;
        let nx = count(require(map, "nx")?, "nx")?;
        let ny = count(require(map, "ny")?, "ny")?;
        let cells = nx.checked_mul(ny).ok_or_else(|| parse_error("stock file grid is too large"))?;
        if cells > crate::MAX_STOCK_CELLS {
            return Err(parse_error(format!("stock file has {cells} cells, above {}", crate::MAX_STOCK_CELLS)));
        }
        let length = finite_nonneg(number(require(map, "length")?, "length")?, "length")?;
        let top = heights(require(map, "top")?, cells, "top")?;
        let multi = indices(require(map, "multi")?, cells, "multi")?;
        let snapshots = match map.get("snapshots") {
            None => Vec::new(),
            Some(value) => snapshots(value, cells)?,
        };
        Ok(Self { grid: StockGrid { origin, cell, nx, ny, top, multi }, length, snapshots })
    }
}

fn file_object(file: &StockFile) -> Result<Map<String, Value>, MeshError> {
    let mut map = Map::new();
    map.insert("format".to_string(), Value::String(STOCK_FORMAT.to_string()));
    map.insert("origin".to_string(), Value::Array(vec![json_f64(file.grid.origin[0])?, json_f64(file.grid.origin[1])?]));
    map.insert("cell".to_string(), json_f64(file.grid.cell)?);
    map.insert("nx".to_string(), Value::Number(Number::from(file.grid.nx as u64)));
    map.insert("ny".to_string(), Value::Number(Number::from(file.grid.ny as u64)));
    map.insert("length".to_string(), json_f64(file.length)?);
    map.insert("top".to_string(), heights_json(&file.grid.top)?);
    map.insert("multi".to_string(), indices_json(&file.grid.multi));
    let mut frames = Vec::with_capacity(file.snapshots.len());
    for frame in &file.snapshots {
        let mut one = Map::new();
        one.insert("position".to_string(), json_f64(frame.position)?);
        one.insert("top".to_string(), heights_json(&frame.top)?);
        one.insert("multi".to_string(), indices_json(&frame.multi));
        frames.push(Value::Object(one));
    }
    map.insert("snapshots".to_string(), Value::Array(frames));
    Ok(map)
}

fn heights_json(top: &[Option<f64>]) -> Result<Value, MeshError> {
    let mut values = Vec::with_capacity(top.len());
    for height in top {
        match *height {
            Some(z) => values.push(json_f64(z)?),
            None => values.push(Value::Null),
        }
    }
    Ok(Value::Array(values))
}

fn indices_json(multi: &[usize]) -> Value {
    Value::Array(multi.iter().copied().map(|index| Value::Number(Number::from(index as u64))).collect())
}

fn json_f64(value: f64) -> Result<Value, MeshError> {
    if !value.is_finite() {
        return Err(invalid("stock file number is not finite"));
    }
    let text = format!("{value}");
    let number = Number::from_str(&text).map_err(|_| invalid(format!("stock file number `{text}` is not json")))?;
    Ok(Value::Number(number))
}

fn object<'a>(value: &'a Value, what: &str) -> Result<&'a Map<String, Value>, MeshError> {
    value.as_object().ok_or_else(|| parse_error(format!("{what} must be an object")))
}

fn require<'a>(map: &'a Map<String, Value>, key: &str) -> Result<&'a Value, MeshError> {
    map.get(key).ok_or_else(|| parse_error(format!("stock file needs `{key}`")))
}

fn check_keys(map: &Map<String, Value>, allowed: &[&str], what: &str) -> Result<(), MeshError> {
    for key in map.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(parse_error(format!("unknown {what} field `{key}`")));
        }
    }
    Ok(())
}

fn string<'a>(value: &'a Value, what: &str) -> Result<&'a str, MeshError> {
    value.as_str().ok_or_else(|| parse_error(format!("{what} must be a string")))
}

fn number(value: &Value, what: &str) -> Result<f64, MeshError> {
    value.as_f64().ok_or_else(|| parse_error(format!("{what} must be a number")))
}

fn finite_positive(value: f64, what: &str) -> Result<f64, MeshError> {
    if value.is_finite() && value > 0.0 { Ok(value) } else { Err(parse_error(format!("{what} must be finite and greater than 0"))) }
}

fn finite_nonneg(value: f64, what: &str) -> Result<f64, MeshError> {
    if value.is_finite() && value >= 0.0 { Ok(value) } else { Err(parse_error(format!("{what} must be finite and at least 0"))) }
}

fn count(value: &Value, what: &str) -> Result<usize, MeshError> {
    let number = value.as_u64().ok_or_else(|| parse_error(format!("{what} must be an integer")))?;
    usize::try_from(number).map_err(|_| parse_error(format!("{what} is too large")))
}

fn pair(value: &Value, what: &str) -> Result<[f64; 2], MeshError> {
    let items = value.as_array().ok_or_else(|| parse_error(format!("{what} must be an array of two numbers")))?;
    if items.len() != 2 {
        return Err(parse_error(format!("{what} must be an array of two numbers")));
    }
    let x = finite(number(&items[0], what)?, what)?;
    let y = finite(number(items.get(1).ok_or_else(|| parse_error(format!("{what} must be an array of two numbers")))?, what)?, what)?;
    Ok([x, y])
}

fn finite(value: f64, what: &str) -> Result<f64, MeshError> {
    if value.is_finite() { Ok(value) } else { Err(parse_error(format!("{what} must be finite"))) }
}

fn heights(value: &Value, cells: usize, what: &str) -> Result<Vec<Option<f64>>, MeshError> {
    let items = value.as_array().ok_or_else(|| parse_error(format!("{what} must be an array")))?;
    if items.len() != cells {
        return Err(parse_error(format!("{what} has {} values, expected {cells}", items.len())));
    }
    let mut top = Vec::with_capacity(cells);
    for item in items {
        if item.is_null() {
            top.push(None);
            continue;
        }
        top.push(Some(finite(number(item, what)?, what)?));
    }
    Ok(top)
}

fn indices(value: &Value, cells: usize, what: &str) -> Result<Vec<usize>, MeshError> {
    let items = value.as_array().ok_or_else(|| parse_error(format!("{what} must be an array")))?;
    let mut multi = Vec::with_capacity(items.len());
    for item in items {
        let index = count(item, what)?;
        if index >= cells {
            return Err(parse_error(format!("{what} index {index} is outside the grid")));
        }
        multi.push(index);
    }
    multi.sort_unstable();
    multi.dedup();
    Ok(multi)
}

fn snapshots(value: &Value, cells: usize) -> Result<Vec<StockSnapshot>, MeshError> {
    let items = value.as_array().ok_or_else(|| parse_error("snapshots must be an array"))?;
    let mut frames = Vec::with_capacity(items.len());
    for item in items {
        let map = object(item, "snapshot")?;
        check_keys(map, &["position", "top", "multi"], "snapshot")?;
        let position = finite_nonneg(number(require(map, "position")?, "snapshot position")?, "snapshot position")?;
        let top = heights(require(map, "top")?, cells, "snapshot top")?;
        let multi = indices(require(map, "multi")?, cells, "snapshot multi")?;
        frames.push(StockSnapshot { position, top, multi });
    }
    Ok(frames)
}
