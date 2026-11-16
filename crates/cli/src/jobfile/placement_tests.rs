use super::*;
use serde_json::json;
use std::path::Path;

fn parse_job(value: &serde_json::Value) -> Result<JobFile, JobFileError> {
    parse(&value.to_string(), Path::new("placement-test.json"))
}

fn base() -> serde_json::Value {
    json!({
        "format": "campanion-job/1",
        "source": {"path": "part.svg", "hash": "cbf29ce484222325"},
        "units": "mm",
        "tools": [{"id": 1, "kerf": 0.1}],
        "operations": [{
            "name": "cut",
            "type": "profile",
            "layers": ["0"],
            "side": "outside",
            "tool_id": 1,
            "depth": 0.26,
            "step_down": 0.26,
            "feeds": {"cut": 300}
        }]
    })
}

#[test]
fn omitted_placement_is_none() {
    let job = parse_job(&base()).expect("parse");
    assert!(job.placement().is_none());
}

#[test]
fn placement_is_scaled_from_file_units() {
    let mut value = base();
    value["units"] = json!("inch");
    value["placement"] = json!({"x": 1.0, "y": -0.5, "rotate_deg": 90.0});
    let job = parse_job(&value).expect("parse");
    let placement = job.placement().expect("placement");
    assert!((placement.x() - 25.4).abs() < 1e-9);
    assert!((placement.y() + 12.7).abs() < 1e-9);
    assert!((placement.rotate_deg() - 90.0).abs() < 1e-9);
}

#[test]
fn unknown_placement_key_is_rejected() {
    let mut value = base();
    value["placement"] = json!({"x": 1.0, "scale": 2.0});
    let error = parse_job(&value).expect_err("unknown").to_string();
    assert!(error.contains("placement"), "{error}");
}

#[test]
fn stl_source_rejects_placement() {
    let mut value = base();
    value["source"]["path"] = json!("part.stl");
    value["operations"] = json!([{
        "name": "surf",
        "type": "surface",
        "tool_id": 1,
        "stepover": 1.0,
        "feeds": {"cut": 300}
    }]);
    value["tools"] = json!([{"id": 1, "diameter": 6}]);
    value["placement"] = json!({"x": 1.0});
    let error = parse_job(&value).expect_err("stl").to_string();
    assert!(error.contains("placement is only supported"), "{error}");
}

#[test]
fn job_machine_profile_is_parsed() {
    let mut value = base();
    value["machine"] = serde_json::json!({"profile": "shop-laser", "head": "laser"});
    let job = parse_job(&value).expect("parse");
    let machine = job.machine().expect("machine");
    assert_eq!(machine.profile.as_deref(), Some("shop-laser"));
    assert_eq!(machine.head.as_deref(), Some("laser"));
}

#[test]
fn job_machine_model_is_rejected_on_a_cutting_job() {
    let mut value = base();
    value["machine"] = serde_json::json!({"model": "haas_st20y"});
    let error = parse_job(&value).expect_err("model").to_string();
    assert!(error.contains("machine.model is only valid on a lathe job"), "{error}");
}
