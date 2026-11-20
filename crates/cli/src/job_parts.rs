//! Parts, instances, selectors and related `campanion-job/2` fields.

use campanion_geom::Contour;
use campanion_import::Drawing;
use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_str, check_keys, object, optional, require};
use crate::jobfile::{JobFileError, Placement, Source, hash_bytes};
use crate::profiles::{JobType, PowerValue};

/// One part in a job: a source drawing and the linked instances that share it.
#[derive(Clone, Debug, PartialEq)]
pub struct JobPart {
    pub id: String,
    pub source: Source,
    pub instances: Vec<JobInstance>,
}

/// One placed instance of a part. Linked instances share the part's source and operations.
#[derive(Clone, Debug, PartialEq)]
pub struct JobInstance {
    pub id: String,
    pub placement: Placement,
}

/// How an operation picks geometry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OpSelector {
    /// Whole part(s).
    Parts { parts: Vec<String> },
    /// SVG/DXF layers on the named parts. Empty `parts` means every part.
    Layers { parts: Vec<String>, layers: Vec<String> },
    /// SVG stroke or fill color, a `#rrggbb` or named string stored on the contour.
    Color { parts: Vec<String>, color: String },
    /// Stable contour ids on one part.
    Contours { part: String, ids: Vec<String> },
}

/// A per-contour override stored in the job (spec v0.1).
#[derive(Clone, Debug, PartialEq)]
pub struct ContourOverride {
    pub part: String,
    pub contour: String,
    pub side: Option<String>,
    pub lead_in: Option<Value>,
    pub direction: Option<String>,
}

/// Fields a user overrode on an operation relative to a material preset.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct FieldOverrides {
    pub names: Vec<String>,
}

/// v2-only operation metadata, keyed by operation name.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct OperationExtras {
    pub selector: Option<OpSelector>,
    pub power: Option<PowerValue>,
    pub passes: Option<u32>,
    pub overrides: FieldOverrides,
    /// Plasma small-hole rule, when the user entered both a diameter and an action.
    pub small_hole: Option<SmallHoleRule>,
}

/// User-entered plasma small-hole intent. Posts do not invent THC codes for this.
#[derive(Clone, Debug, PartialEq)]
pub struct SmallHoleRule {
    /// Maximum hole diameter, millimetres, as entered.
    pub max_diameter: f64,
    /// Recorded action (`pierce_only`, `thc_off`, or another user-entered token).
    pub action: String,
}

/// Build the default single-part list from a v1 source and placement.
#[must_use]
pub fn migrate_v1_parts(source: Source, placement: Option<Placement>) -> Vec<JobPart> {
    Vec::from([JobPart {
        id: "part".to_string(),
        source,
        instances: Vec::from([JobInstance { id: "1".to_string(), placement: placement.unwrap_or_else(Placement::identity) }]),
    }])
}

/// Parse the v2 `parts` array. Each part needs a unique id, a source, and at least one instance.
pub fn parse_parts(value: &Value, scale: f64) -> Result<Vec<JobPart>, JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid("parts must be an array"));
    };
    if items.is_empty() {
        return Err(JobFileError::invalid("a job needs at least one part"));
    }
    let mut parts = Vec::new();
    let mut seen = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let part = parse_part(item, scale, index)?;
        if seen.iter().any(|id: &String| id == &part.id) {
            return Err(JobFileError::invalid(format!("duplicate part id `{}`", part.id)));
        }
        seen.push(part.id.clone());
        parts.push(part);
    }
    Ok(parts)
}

fn parse_part(value: &Value, scale: f64, index: usize) -> Result<JobPart, JobFileError> {
    let what = format!("parts[{index}]");
    let map = object(value, &what).map_err(from_cli)?;
    check_keys(map, &["id", "source", "instances", "placement"], &what).map_err(from_cli)?;
    let id = as_str(require(map, "id").map_err(from_cli)?, &format!("{what}.id")).map_err(from_cli)?.to_string();
    if id.trim().is_empty() {
        return Err(JobFileError::invalid(format!("{what}.id is empty")));
    }
    let source = crate::jobfile::parse_source_for_parts(require(map, "source").map_err(from_cli)?)?;
    let instances = match optional(map, "instances") {
        Some(value) => parse_instances(value, scale, &what)?,
        None => {
            let placement = crate::jobfile::parse_placement_value(optional(map, "placement"), scale)?.unwrap_or_else(Placement::identity);
            Vec::from([JobInstance { id: "1".to_string(), placement }])
        }
    };
    if instances.is_empty() {
        return Err(JobFileError::invalid(format!("{what} needs at least one instance")));
    }
    if optional(map, "instances").is_some() && optional(map, "placement").is_some() {
        return Err(JobFileError::invalid(format!("{what} cannot set both placement and instances")));
    }
    Ok(JobPart { id, source, instances })
}

fn parse_instances(value: &Value, scale: f64, part: &str) -> Result<Vec<JobInstance>, JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid(format!("{part}.instances must be an array")));
    };
    let mut instances = Vec::new();
    let mut seen = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let what = format!("{part}.instances[{index}]");
        let map = object(item, &what).map_err(from_cli)?;
        check_keys(map, &["id", "placement"], &what).map_err(from_cli)?;
        let id = as_str(require(map, "id").map_err(from_cli)?, &format!("{what}.id")).map_err(from_cli)?.to_string();
        if id.trim().is_empty() {
            return Err(JobFileError::invalid(format!("{what}.id is empty")));
        }
        if seen.iter().any(|seen: &String| seen == &id) {
            return Err(JobFileError::invalid(format!("duplicate instance id `{id}` on {part}")));
        }
        seen.push(id.clone());
        let placement = crate::jobfile::parse_placement_value(optional(map, "placement"), scale)?.unwrap_or_else(Placement::identity);
        instances.push(JobInstance { id, placement });
    }
    Ok(instances)
}

/// Parse a user-entered small-hole rule. Both a positive `max_diameter` and an `action` are required.
pub fn parse_small_hole(value: &Value, what: &str, scale: f64) -> Result<SmallHoleRule, JobFileError> {
    let map = object(value, what).map_err(from_cli)?;
    check_keys(map, &["max_diameter", "action"], what).map_err(from_cli)?;
    let max = crate::job::as_f64(require(map, "max_diameter").map_err(from_cli)?, &format!("{what}.max_diameter")).map_err(from_cli)? * scale;
    if !max.is_finite() || max <= 0.0 {
        return Err(JobFileError::invalid(format!("{what}.max_diameter must be greater than zero")));
    }
    let action = as_str(require(map, "action").map_err(from_cli)?, &format!("{what}.action")).map_err(from_cli)?.trim().to_string();
    if action.is_empty() {
        return Err(JobFileError::invalid(format!("{what}.action is empty")));
    }
    Ok(SmallHoleRule { max_diameter: max, action })
}

/// Parse a v2 operation selector.
pub fn parse_selector(value: &Value, what: &str) -> Result<OpSelector, JobFileError> {
    let map = object(value, what).map_err(from_cli)?;
    let typ = as_str(require(map, "type").map_err(from_cli)?, &format!("{what}.type")).map_err(from_cli)?;
    match typ {
        "part" | "parts" => {
            check_keys(map, &["type", "parts", "part"], what).map_err(from_cli)?;
            Ok(OpSelector::Parts { parts: selector_parts(map, what)? })
        }
        "layer" | "layers" => {
            check_keys(map, &["type", "parts", "part", "layers", "layer"], what).map_err(from_cli)?;
            let layers = string_list(map, "layers", "layer", what)?;
            if layers.is_empty() {
                return Err(JobFileError::invalid(format!("{what} needs a layer or layers array")));
            }
            Ok(OpSelector::Layers { parts: optional_selector_parts(map, what)?, layers })
        }
        "color" => {
            check_keys(map, &["type", "parts", "part", "color"], what).map_err(from_cli)?;
            let color = as_str(require(map, "color").map_err(from_cli)?, &format!("{what}.color")).map_err(from_cli)?.to_string();
            if color.trim().is_empty() {
                return Err(JobFileError::invalid(format!("{what}.color is empty")));
            }
            Ok(OpSelector::Color { parts: optional_selector_parts(map, what)?, color })
        }
        "contours" | "contour" => {
            check_keys(map, &["type", "part", "ids", "contours"], what).map_err(from_cli)?;
            let part = as_str(require(map, "part").map_err(from_cli)?, &format!("{what}.part")).map_err(from_cli)?.to_string();
            let ids = string_list(map, "ids", "contours", what)?;
            if ids.is_empty() {
                return Err(JobFileError::invalid(format!("{what} needs at least one contour id")));
            }
            Ok(OpSelector::Contours { part, ids })
        }
        other => Err(JobFileError::invalid(format!("{what}.type must be part, layer, color or contours, not `{other}`"))),
    }
}

fn selector_parts(map: &Map<String, Value>, what: &str) -> Result<Vec<String>, JobFileError> {
    let parts = optional_selector_parts(map, what)?;
    if parts.is_empty() {
        return Err(JobFileError::invalid(format!("{what} needs a part or parts array")));
    }
    Ok(parts)
}

fn optional_selector_parts(map: &Map<String, Value>, what: &str) -> Result<Vec<String>, JobFileError> {
    string_list(map, "parts", "part", what)
}

fn string_list(map: &Map<String, Value>, array_key: &str, single_key: &str, what: &str) -> Result<Vec<String>, JobFileError> {
    if let Some(value) = optional(map, array_key) {
        let Some(items) = value.as_array() else {
            return Err(JobFileError::invalid(format!("{what}.{array_key} must be an array")));
        };
        let mut out = Vec::new();
        for (index, item) in items.iter().enumerate() {
            let text = as_str(item, &format!("{what}.{array_key}[{index}]")).map_err(from_cli)?.to_string();
            if text.trim().is_empty() {
                return Err(JobFileError::invalid(format!("{what}.{array_key}[{index}] is empty")));
            }
            out.push(text);
        }
        return Ok(out);
    }
    if let Some(value) = optional(map, single_key) {
        let text = as_str(value, &format!("{what}.{single_key}")).map_err(from_cli)?.to_string();
        if text.trim().is_empty() {
            return Err(JobFileError::invalid(format!("{what}.{single_key} is empty")));
        }
        return Ok(Vec::from([text]));
    }
    Ok(Vec::new())
}

/// Parse `cut_order` as a list of unique operation names.
pub fn parse_cut_order(value: &Value, operations: &[String]) -> Result<Vec<String>, JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid("cut_order must be an array"));
    };
    let mut order = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let name = as_str(item, &format!("cut_order[{index}]")).map_err(from_cli)?.to_string();
        if !operations.iter().any(|op| op == &name) {
            return Err(JobFileError::invalid(format!("cut_order names unknown operation `{name}`")));
        }
        if order.iter().any(|seen: &String| seen == &name) {
            return Err(JobFileError::invalid(format!("cut_order repeats operation `{name}`")));
        }
        order.push(name);
    }
    Ok(order)
}

/// Parse the optional `contour_overrides` array.
pub fn parse_contour_overrides(value: &Value) -> Result<Vec<ContourOverride>, JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid("contour_overrides must be an array"));
    };
    let mut out = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let what = format!("contour_overrides[{index}]");
        let map = object(item, &what).map_err(from_cli)?;
        check_keys(map, &["part", "contour", "side", "lead_in", "direction"], &what).map_err(from_cli)?;
        let part = as_str(require(map, "part").map_err(from_cli)?, &format!("{what}.part")).map_err(from_cli)?.to_string();
        let contour = as_str(require(map, "contour").map_err(from_cli)?, &format!("{what}.contour")).map_err(from_cli)?.to_string();
        let side = optional_string(map, "side")?;
        let direction = optional_string(map, "direction")?;
        let lead_in = optional(map, "lead_in").cloned();
        if side.is_none() && direction.is_none() && lead_in.is_none() {
            return Err(JobFileError::invalid(format!("{what} needs a side, lead_in or direction override")));
        }
        out.push(ContourOverride { part, contour, side, lead_in, direction });
    }
    Ok(out)
}

fn optional_string(map: &Map<String, Value>, key: &str) -> Result<Option<String>, JobFileError> {
    match optional(map, key) {
        Some(value) => Ok(Some(as_str(value, key).map_err(from_cli)?.to_string())),
        None => Ok(None),
    }
}

/// Parse `overrides` as an array of field names.
pub fn parse_overrides(value: &Value, what: &str) -> Result<FieldOverrides, JobFileError> {
    let Some(items) = value.as_array() else {
        return Err(JobFileError::invalid(format!("{what} must be an array of field names")));
    };
    let mut names = Vec::new();
    for (index, item) in items.iter().enumerate() {
        let name = as_str(item, &format!("{what}[{index}]")).map_err(from_cli)?.to_string();
        if name.trim().is_empty() {
            return Err(JobFileError::invalid(format!("{what}[{index}] is empty")));
        }
        names.push(name);
    }
    Ok(FieldOverrides { names })
}

/// Parse a v2 job `type` field.
pub fn parse_job_type(value: &Value) -> Result<JobType, JobFileError> {
    let text = as_str(value, "type").map_err(from_cli)?;
    JobType::parse(text).map_err(JobFileError::invalid)
}

/// Stable contour ids for a drawing, unique within the file, derived from layer and geometry.
#[must_use]
pub fn contour_ids(drawing: &Drawing) -> Vec<String> {
    let mut ids = Vec::with_capacity(drawing.contours().len());
    let mut used: Vec<String> = Vec::new();
    for item in drawing.contours() {
        let base = format!("{}:{}", item.layer(), geometry_tag(item.contour()));
        let mut id = base.clone();
        let mut n = 2;
        while used.iter().any(|seen| seen == &id) {
            id = format!("{base}:{n}");
            n += 1;
        }
        used.push(id.clone());
        ids.push(id);
    }
    ids
}

fn geometry_tag(contour: &Contour) -> String {
    let mut bytes = Vec::new();
    bytes.extend(if contour.is_closed() { b"C" } else { b"O" });
    for segment in contour.segments() {
        match segment {
            campanion_geom::Segment::Line(line) => {
                bytes.extend(b"L");
                push_xy(&mut bytes, line.start().x(), line.start().y());
                push_xy(&mut bytes, line.end().x(), line.end().y());
            }
            campanion_geom::Segment::Arc(arc) => {
                bytes.extend(b"A");
                push_xy(&mut bytes, arc.center().x(), arc.center().y());
                push_f(&mut bytes, arc.radius());
                push_f(&mut bytes, arc.start_angle());
                push_f(&mut bytes, arc.sweep());
            }
        }
    }
    hash_bytes(&bytes)
}

fn push_xy(bytes: &mut Vec<u8>, x: f64, y: f64) {
    push_f(bytes, x);
    push_f(bytes, y);
}

fn push_f(bytes: &mut Vec<u8>, value: f64) {
    // Micro-metre rounding keeps ids stable across tiny float noise.
    let rounded = (value * 1_000_000.0).round() as i64;
    bytes.extend(rounded.to_le_bytes());
}

fn from_cli(err: CliError) -> JobFileError {
    JobFileError::invalid(err.to_string())
}

/// True when `selector` includes `part_id`.
#[must_use]
#[allow(dead_code)]
pub fn selector_includes_part(selector: Option<&OpSelector>, part_id: &str, _fallback_layers: &[String]) -> bool {
    match selector {
        Some(OpSelector::Parts { parts }) => parts.iter().any(|id| id == part_id),
        Some(OpSelector::Layers { parts, .. }) | Some(OpSelector::Color { parts, .. }) => parts.is_empty() || parts.iter().any(|id| id == part_id),
        Some(OpSelector::Contours { part, .. }) => part == part_id,
        None => true, // v1 operations apply to the (only) part; layers are filtered later
    }
}

/// Whether a v1-style `layers` list is still the effective selector.
#[must_use]
#[allow(dead_code)]
pub fn uses_legacy_layers(selector: Option<&OpSelector>, fallback_layers: &[String]) -> bool {
    selector.is_none() && !fallback_layers.is_empty()
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;
    use crate::jobfile::{self, FORMAT_TAG, FORMAT_TAG_V2};

    fn parse(text: &str) -> jobfile::JobFile {
        jobfile::parse(text, Path::new("/work/job.json")).unwrap_or_else(|err| panic!("{err}"))
    }

    #[test]
    fn v1_job_migrates_to_a_single_part_and_instance() {
        let job = parse(
            r#"{
                "format":"campanion-job/1",
                "source":{"path":"plate.svg","hash":"cbf29ce484222325"},
                "units":"mm",
                "placement":{"x":10,"y":20,"rotate_deg":90},
                "tools":[{"id":1,"kerf":0.1}],
                "operations":[{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":100}}]
            }"#,
        );
        assert_eq!(job.format(), FORMAT_TAG);
        assert_eq!(job.parts().len(), 1);
        assert_eq!(job.parts()[0].id, "part");
        assert_eq!(job.parts()[0].instances.len(), 1);
        assert_eq!(job.parts()[0].instances[0].id, "1");
        assert!((job.parts()[0].instances[0].placement.x() - 10.0).abs() < 1e-9);
        assert!((job.placement().unwrap().y() - 20.0).abs() < 1e-9);
        assert_eq!(job.source().path(), "plate.svg");
    }

    #[test]
    fn v2_parts_and_linked_instances_parse() {
        let job = parse(
            r#"{
                "format":"campanion-job/2",
                "units":"mm",
                "type":"cut_route",
                "parts":[{
                    "id":"bracket",
                    "source":{"path":"/tmp/bracket.svg","hash":"cbf29ce484222325"},
                    "instances":[
                        {"id":"a","placement":{"x":0,"y":0}},
                        {"id":"b","placement":{"x":40,"y":0,"rotate_deg":90,"mirror":true,"scale":1.5}}
                    ]
                }],
                "tools":[{"id":1,"kerf":0.1}],
                "operations":[{
                    "name":"cut",
                    "type":"profile",
                    "layers":["0"],
                    "side":"outside",
                    "tool_id":1,
                    "depth":1,
                    "step_down":1,
                    "feeds":{"cut":100},
                    "power":{"unit":"percent","value":80},
                    "passes":2,
                    "overrides":["power"],
                    "selector":{"type":"part","parts":["bracket"]}
                }],
                "cut_order":["cut"]
            }"#,
        );
        assert_eq!(job.format(), FORMAT_TAG_V2);
        assert_eq!(job.job_type(), Some(crate::profiles::JobType::CutRoute));
        assert_eq!(job.parts().len(), 1);
        assert_eq!(job.parts()[0].instances.len(), 2);
        let second = &job.parts()[0].instances[1].placement;
        assert!(second.mirror());
        assert!((second.scale() - 1.5).abs() < 1e-12);
        assert!((second.x() - 40.0).abs() < 1e-9);
        let extras = job.operation_extras("cut").expect("extras");
        assert!(matches!(extras.selector, Some(OpSelector::Parts { .. })));
        assert_eq!(extras.power, Some(PowerValue::percent(80.0)));
        assert_eq!(extras.passes, Some(2));
        assert_eq!(extras.overrides.names, ["power"]);
        assert_eq!(job.cut_order(), &["cut".to_string()]);
    }

    #[test]
    fn v2_rejects_a_top_level_source_and_multiple_heads() {
        let err = jobfile::parse(
            r#"{"format":"campanion-job/2","units":"mm","source":{"path":"a.svg","hash":"cbf29ce484222325"},"parts":[{"id":"p","source":{"path":"a.svg","hash":"cbf29ce484222325"}}],"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]}"#,
            Path::new("/work/job.json"),
        )
        .expect_err("source");
        assert!(err.to_string().contains("parts[].source"), "{err}");
        let err = jobfile::parse(
            r#"{"format":"campanion-job/2","units":"mm","machine":{"profile":"shop","heads":["a","b"]},"parts":[{"id":"p","source":{"path":"a.svg","hash":"cbf29ce484222325"}}],"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":1,"step_down":1,"feeds":{"cut":1}}]}"#,
            Path::new("/work/job.json"),
        )
        .expect_err("heads");
        assert!(err.to_string().contains("single head"), "{err}");
    }

    #[test]
    fn contour_ids_are_stable_and_unique() {
        let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10"><rect x="1" y="1" width="8" height="6"/><rect x="11" y="1" width="8" height="6"/></svg>"#;
        let drawing = campanion_import::import_svg(svg, &campanion_import::ImportOptions::new(campanion_geom::Tolerance::default())).expect("svg");
        let ids = contour_ids(&drawing);
        assert_eq!(ids.len(), drawing.contours().len());
        assert_eq!(ids, contour_ids(&drawing));
        let unique: std::collections::BTreeSet<_> = ids.iter().collect();
        assert_eq!(unique.len(), ids.len());
    }
}
