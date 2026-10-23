use std::f64::consts::PI;

use proptest::prelude::*;

use crate::Result;
use crate::geom::{ArcSense, Plane, Pose, Vec3};
use crate::machine::{Axis, AxisKind, AxisLetter, AxisName, Channel, KinematicNode, Machine};
use crate::motion::{ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole};
use crate::process::{Event, Tool};
use crate::program::{Document, Program};
use crate::rotary::{RotaryAxisKind, RotaryDirection, RotarySetup};
use crate::turning::{
    Hand, Head, HeadAxis, Insert, InsertShape, Spindle, SpindleSpeed, TurnBinding, TurnInsert, TurnSection, TurnStock, TurningSetup,
};

fn assert_err(result: Result<impl std::fmt::Debug>, text: &str) {
    let err = result.expect_err("expected an error");
    assert!(err.to_string().contains(text), "{err} should contain {text}");
}

fn pose_xz(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn feeds() -> Feeds {
    Feeds::try_new(100.0, None, None, None, None).expect("feeds")
}

fn path_with(tool_id: Option<u32>, blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(Some("turn".into()), tool_id, CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path")
}

fn rapid(x: f64, y: f64, z: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: pose_xz(x, y, z), feed: None })
}

fn linear(x: f64, y: f64, z: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: pose_xz(x, y, z), feed: 80.0, role: MoveRole::Cut })
}

fn tool(id: u32) -> Tool {
    Tool::try_new(id, Some("cnmg".into()), None, None).expect("tool")
}

fn insert_c80() -> Insert {
    Insert::try_new(InsertShape::C80, 0.8, None, Hand::Right, 45.0).expect("insert")
}

fn mounted(tool_id: u32, head: &str, station: u16, live: bool) -> TurnInsert {
    TurnInsert::try_new(tool_id, head, station, insert_c80(), live).expect("mounted")
}

fn m10(stock: Option<TurnStock>) -> TurningSetup {
    TurningSetup::single(stock, 4_000.0).expect("m10")
}

fn program_with(tools: Vec<Tool>, paths: Vec<ClPath>, turning: TurningSetup, machine: Option<Machine>) -> Result<Program> {
    Program::try_new(Some("turn".into()), tools, machine, paths, Vec::new(), Vec::new(), None)?.with_turning(Some(turning))
}

fn round_trip(doc: &Document) {
    let json = doc.to_json().expect("json");
    let loaded = Document::from_json(&json).expect("load");
    assert_eq!(&loaded, doc);
    assert_eq!(loaded.to_json().expect("again"), json);
}

fn lathe_machine(axis_ids: &[u8], channel_id: u16) -> Machine {
    let mut axes = Vec::new();
    for (ordinal, id) in axis_ids.iter().copied().enumerate() {
        let (letter, kind, direction) = match ordinal % 3 {
            0 => (AxisLetter::X, AxisKind::Linear, Vec3::X),
            1 => (AxisLetter::Z, AxisKind::Linear, Vec3::Z),
            _ => (AxisLetter::C, AxisKind::Rotary, Vec3::Z),
        };
        let chain = KinematicNode::try_new(None, direction, Vec3::ZERO).expect("chain");
        axes.push(Axis::try_new(id, AxisName::Letter(letter), kind, None, chain).expect("axis"));
    }
    let channel = Channel::try_new(channel_id, "main", axis_ids.to_vec()).expect("channel");
    Machine::try_new(axes, vec![channel]).expect("machine")
}

fn spindle_named(name: &str) -> Spindle {
    Spindle::try_new(name, None, 0.0, 3_000.0).expect("spindle")
}

fn head_named(name: &str, stations: u16, live: bool) -> Head {
    Head::try_new(name, vec![HeadAxis::X, HeadAxis::Z], stations, live).expect("head")
}

fn setup_with(spindles: Vec<Spindle>, heads: Vec<Head>, inserts: Vec<TurnInsert>, bindings: Vec<TurnBinding>) -> Result<TurningSetup> {
    TurningSetup::try_new(spindles, heads, None, inserts, bindings)
}

#[test]
fn turning_program_round_trips_and_keeps_radius_mode() {
    let stock = TurnStock::bar(50.0, 80.0).expect("stock");
    assert_eq!(stock.outer_radius(), 25.0);
    assert_eq!(stock.inner_radius(), 0.0);
    assert_eq!(stock.length(), 80.0);
    let setup = m10(Some(stock))
        .with_insert(mounted(1, "turret1", 1, false))
        .expect("insert")
        .with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding"))
        .expect("binding")
        .with_diameter_mode(false);
    let blocks = vec![rapid(30.0, 0.0, 2.0), linear(25.0, 0.0, -40.0)];
    let program = program_with(vec![tool(1)], vec![path_with(Some(1), blocks)], setup, None).expect("program");
    let doc = Document::new(program).expect("document");
    round_trip(&doc);

    let json = doc.to_json().expect("json");
    assert!(json.contains("\"turning\":{"), "{json}");
    assert!(json.contains("\"name\":\"main\""), "{json}");
    assert!(json.contains("\"name\":\"turret1\""), "{json}");
    assert!(json.contains("\"kind\":\"bar\""), "{json}");
    assert!(json.contains("\"shape\":\"c80\""), "{json}");
    assert!(json.contains("\"diameter_mode\":false"), "{json}");
    let turning = doc.program().turning().expect("turning");
    assert!(!turning.diameter_mode());
    assert_eq!(turning.spindle("main").expect("main").max_rpm(), 4_000.0);
    assert_eq!(turning.head("turret1").expect("turret").stations(), 12);
    assert_eq!(turning.insert_for_tool(1).expect("insert").station(), 1);
    assert_eq!(turning.binding_for(0).expect("binding").channel(), 0);
    assert!(turning.binding_for(1).is_none());
    let tip = match &doc.program().toolpaths()[0].blocks()[1] {
        ClBlock::Motion(ClMotion::Linear { to, .. }) => to.tip(),
        other => panic!("linear, got {other:?}"),
    };
    assert_eq!(tip.x(), 25.0);
    assert_eq!(tip.y(), 0.0);
    assert!(Program::default().turning().is_none());
}

#[test]
fn old_documents_without_turning_still_parse() {
    let json = r#"{"format":"campanion-ir/1","program":{"tools":[],"toolpaths":[],"channel_programs":[],"syncs":[]}}"#;
    let doc = Document::from_json(json).expect("old");
    assert!(doc.program().turning().is_none());
    assert!(!doc.to_json().expect("json").contains("\"turning\""));

    let sample = turning_document();
    let mut value = serde_json::from_str::<serde_json::Value>(&sample.to_json().expect("json")).expect("value");
    value["program"].as_object_mut().expect("program").remove("turning");
    let loaded = Document::from_json(&serde_json::to_string(&value).expect("json")).expect("stripped");
    assert!(loaded.program().turning().is_none());

    let mut with_turning = serde_json::from_str::<serde_json::Value>(&sample.to_json().expect("json")).expect("value");
    with_turning["program"]["turning"].as_object_mut().expect("turning").remove("diameter_mode");
    let loaded = Document::from_json(&serde_json::to_string(&with_turning).expect("json")).expect("default mode");
    assert!(loaded.program().turning().expect("turning").diameter_mode());
    let spindle = &with_turning["program"]["turning"]["spindles"][0];
    assert!(spindle.get("reversed").is_some());
    with_turning["program"]["turning"]["spindles"][0].as_object_mut().expect("spindle").remove("reversed");
    let loaded = Document::from_json(&serde_json::to_string(&with_turning).expect("json")).expect("default reversed");
    assert!(!loaded.program().turning().expect("turning").spindles()[0].reversed());
}

#[test]
fn live_tooling_y_b_and_sub_spindle_use_the_same_types() {
    let main = Spindle::try_new("main", Some(2), 0.0, 4_000.0).expect("main").with_reversed(true);
    let sub = Spindle::try_new("sub", Some(4), -120.0, 3_000.0).expect("sub");
    assert!(main.reversed());
    assert_eq!(main.c_axis(), Some(2));
    assert_eq!(sub.work_zero_z(), -120.0);
    assert!(!sub.clone().with_reversed(false).reversed());
    let turret = head_named("turret1", 12, false);
    let bhead = Head::try_new("bhead", vec![HeadAxis::Z, HeadAxis::X, HeadAxis::Y, HeadAxis::B], 1, true).expect("b head");
    let groove = Insert::try_new(InsertShape::Groove, 0.0, Some(3.0), Hand::Neutral, 90.0).expect("groove");
    let live = TurnInsert::try_new(1, "bhead", 1, groove, true).expect("live");
    let binding = TurnBinding::try_new(0, "sub", "bhead", 0).expect("binding");
    let tube = TurnStock::tube(40.0, 20.0, 60.0).expect("tube");
    assert_eq!((tube.outer_radius(), tube.inner_radius(), tube.length()), (20.0, 10.0, 60.0));
    let setup = TurningSetup::try_new(vec![main, sub], vec![turret, bhead], Some(tube), vec![live], vec![binding]).expect("setup");
    assert!(setup.diameter_mode());
    assert_eq!(setup.head("bhead").expect("b").axes(), &[HeadAxis::Z, HeadAxis::X, HeadAxis::Y, HeadAxis::B]);
    assert!(setup.insert_for_tool(1).expect("tool").live());
    assert_eq!(setup.binding_for(0).expect("binding").spindle(), "sub");
    assert!(setup.spindle("nope").is_none());
    assert!(setup.head("nope").is_none());
    assert!(setup.insert_for_tool(9).is_none());
    let odd_axis = Pose::new(Vec3::new(12.0, 0.0, -5.0).expect("tip"), Vec3::X).expect("pose");
    let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: odd_axis, feed: None })];
    let program = program_with(vec![tool(1)], vec![path_with(Some(1), blocks)], setup, None).expect("program");
    round_trip(&Document::new(program).expect("document"));
}

#[test]
fn css_rpm_at_radius_matches_surface_speed_and_clamps() {
    let speed = SpindleSpeed::css(200.0, 6_000.0).expect("css");
    let rpm = speed.rpm_at_radius(25.0);
    assert!((rpm - 1273.2).abs() < 0.1, "{rpm}");
    assert!((rpm - 4_000.0 / PI).abs() < 1e-9, "{rpm}");

    let capped = SpindleSpeed::css(200.0, 500.0).expect("cap");
    assert_eq!(capped.rpm_at_radius(25.0), 500.0);
    assert_eq!(capped.rpm_at_radius(0.0), 500.0);
    assert_eq!(capped.rpm_at_radius(1e-9), 500.0);
    assert_eq!(capped.rpm_at_radius(-2.0), 500.0);
    assert_eq!(capped.rpm_at_radius(f64::NAN), 500.0);
    assert_eq!(capped.rpm_at_radius(f64::INFINITY), 500.0);
    let direct = SpindleSpeed::rpm(900.0).expect("rpm");
    assert_eq!(direct.rpm_at_radius(25.0), 900.0);
    assert_eq!(direct.rpm_at_radius(f64::NAN), 900.0);

    assert_err(SpindleSpeed::css(0.0, 1_000.0), "positive");
    assert_err(SpindleSpeed::css(-1.0, 1_000.0), "positive");
    assert_err(SpindleSpeed::css(f64::NAN, 1_000.0), "finite");
    assert_err(SpindleSpeed::css(10.0, f64::INFINITY), "finite");
    assert_err(SpindleSpeed::css(10.0, 0.0), "positive");
    assert!(SpindleSpeed::css(10.0, 100_000.0).is_ok());
    assert_err(SpindleSpeed::css(10.0, 100_001.0), "100000");
    assert_err(SpindleSpeed::rpm(0.0), "positive");
    assert_err(SpindleSpeed::rpm(-10.0), "positive");
    assert_err(SpindleSpeed::rpm(f64::NAN), "finite");

    let css_json = serde_json::to_string(&SpindleSpeed::css(200.0, 3_000.0).expect("css")).expect("json");
    assert!(css_json.contains("\"mode\":\"css\""), "{css_json}");
    assert_eq!(serde_json::from_str::<SpindleSpeed>(&css_json).expect("css"), SpindleSpeed::css(200.0, 3_000.0).expect("css"));
    let rpm_json = serde_json::to_string(&direct).expect("json");
    assert!(rpm_json.contains("\"mode\":\"rpm\""), "{rpm_json}");
    let hostile: SpindleSpeed = serde_json::from_str(r#"{"mode":"css","surface_m_per_min":10.0,"max_rpm":100001.0}"#).expect("decode");
    assert!(hostile.check().is_err());
}

#[test]
fn tip_offset_uses_the_holder_quadrant() {
    let offset = |deg: f64| Insert::try_new(InsertShape::C80, 0.8, None, Hand::Right, deg).expect("insert").tip_offset();
    assert_eq!(offset(45.0), (-0.8, -0.8));
    assert_eq!(offset(135.0), (-0.8, 0.8));
    assert_eq!(offset(225.0), (0.8, 0.8));
    assert_eq!(offset(315.0), (0.8, -0.8));
    assert_eq!(offset(0.0), (0.0, -0.8));
    assert_eq!(offset(90.0), (-0.8, 0.0));
    assert_eq!(offset(180.0), (0.0, 0.8));
    assert_eq!(offset(270.0), (0.8, 0.0));
    assert_eq!(offset(360.0), (0.0, -0.8));
    let left = Insert::try_new(InsertShape::C80, 0.8, None, Hand::Left, 45.0).expect("left").tip_offset();
    assert_eq!(left, offset(45.0));
}

#[test]
fn insert_shapes_have_included_angles_and_snake_case_names() {
    let cases = [
        (InsertShape::C80, Some(80.0), "c80"),
        (InsertShape::D55, Some(55.0), "d55"),
        (InsertShape::V35, Some(35.0), "v35"),
        (InsertShape::W80, Some(80.0), "w80"),
        (InsertShape::T60, Some(60.0), "t60"),
        (InsertShape::S90, Some(90.0), "s90"),
        (InsertShape::R, None, "r"),
        (InsertShape::Groove, None, "groove"),
        (InsertShape::Thread, Some(60.0), "thread"),
    ];
    for (shape, angle, name) in cases {
        assert_eq!(shape.included_angle_deg(), angle);
        assert_eq!(serde_json::to_string(&shape).expect("json"), format!("\"{name}\""));
    }
    assert_eq!(serde_json::to_string(&Hand::Left).expect("json"), "\"left\"");
    assert_eq!(serde_json::to_string(&Hand::Right).expect("json"), "\"right\"");
    assert_eq!(serde_json::to_string(&Hand::Neutral).expect("json"), "\"neutral\"");
    assert_eq!(serde_json::to_string(&HeadAxis::X).expect("json"), "\"x\"");
    assert_eq!(serde_json::to_string(&HeadAxis::B).expect("json"), "\"b\"");
}

#[test]
fn single_builds_the_m10_spindle_and_turret() {
    let setup = TurningSetup::single(Some(TurnStock::bar(40.0, 100.0).expect("bar")), 3_500.0).expect("single");
    let spindle = setup.spindle("main").expect("main");
    assert_eq!(spindle.c_axis(), None);
    assert_eq!(spindle.work_zero_z(), 0.0);
    assert_eq!(spindle.max_rpm(), 3_500.0);
    assert!(!spindle.reversed());
    let head = setup.head("turret1").expect("turret");
    assert_eq!(head.axes(), &[HeadAxis::X, HeadAxis::Z]);
    assert_eq!(head.stations(), 12);
    assert!(!head.live_tooling());
    assert!(setup.inserts().is_empty());
    assert!(setup.bindings().is_empty());
    assert!(setup.diameter_mode());
    assert_eq!(setup.stock().expect("stock").outer_radius(), 20.0);
    assert_err(TurningSetup::single(None, 0.0), "positive");
    assert_err(TurningSetup::single(None, f64::NAN), "finite");
}

#[test]
fn spindle_head_stock_and_insert_checks_reject_bad_values() {
    assert_err(Spindle::try_new("", None, 0.0, 1.0), "empty");
    assert_err(Spindle::try_new("main", None, f64::NAN, 1.0), "finite");
    assert_err(Spindle::try_new("main", None, f64::INFINITY, 1.0), "finite");
    assert_err(Spindle::try_new("main", None, 0.0, 0.0), "positive");
    assert_err(Spindle::try_new("main", None, 0.0, -5.0), "positive");
    assert_err(Spindle::try_new("main", None, 0.0, f64::NAN), "finite");
    assert!(Spindle::try_new("main", None, -12.5, 1.0e6).is_ok());
    assert_err(Spindle::try_new("a".repeat(257), None, 0.0, 1.0), "longer");

    assert_err(Head::try_new("", vec![HeadAxis::X, HeadAxis::Z], 1, false), "empty");
    assert_err(Head::try_new("turret", vec![HeadAxis::Z, HeadAxis::Y], 1, false), "x and z");
    assert_err(Head::try_new("turret", vec![HeadAxis::X], 1, false), "x and z");
    assert_err(Head::try_new("turret", Vec::new(), 1, false), "x and z");
    assert_err(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::X], 1, false), "twice");
    assert_err(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::Z, HeadAxis::Y, HeadAxis::B, HeadAxis::X], 1, false), "more than 4");
    assert_err(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::Z], 0, false), "1 to 256");
    assert_err(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::Z], 257, false), "1 to 256");
    assert_err(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::Z], u16::MAX, false), "1 to 256");
    assert!(Head::try_new("turret", vec![HeadAxis::Z, HeadAxis::X], 1, false).is_ok());
    assert!(Head::try_new("turret", vec![HeadAxis::X, HeadAxis::Z], 256, true).is_ok());

    assert_err(TurnStock::bar(0.0, 10.0), "positive");
    assert_err(TurnStock::bar(-1.0, 10.0), "positive");
    assert_err(TurnStock::bar(f64::NAN, 10.0), "finite");
    assert_err(TurnStock::bar(10.0, 0.0), "positive");
    assert_err(TurnStock::bar(10.0, f64::INFINITY), "finite");
    assert_err(TurnStock::tube(0.0, 1.0, 10.0), "positive");
    assert_err(TurnStock::tube(10.0, 0.0, 10.0), "positive");
    assert_err(TurnStock::tube(10.0, 4.0, -1.0), "positive");
    assert_err(TurnStock::tube(10.0, f64::NAN, 10.0), "finite");
    assert_err(TurnStock::tube(10.0, 10.0, 10.0), "less than outer");
    assert_err(TurnStock::tube(10.0, 12.0, 10.0), "less than outer");
    let bad_stock = TurnStock::Bar { diameter: f64::NAN, length: 1.0 };
    assert!(bad_stock.check().is_err());

    assert_err(Insert::try_new(InsertShape::C80, -0.1, None, Hand::Neutral, 0.0), "nose radius");
    assert_err(Insert::try_new(InsertShape::C80, f64::NAN, None, Hand::Neutral, 0.0), "finite");
    assert!(Insert::try_new(InsertShape::C80, 0.0, None, Hand::Neutral, 0.0).is_ok());
    assert_err(Insert::try_new(InsertShape::R, 0.0, None, Hand::Neutral, 0.0), "round insert");
    assert!(Insert::try_new(InsertShape::R, 0.2, None, Hand::Neutral, 0.0).is_ok());
    assert_err(Insert::try_new(InsertShape::Groove, 0.0, None, Hand::Neutral, 0.0), "width");
    assert_err(Insert::try_new(InsertShape::Groove, 0.0, Some(0.0), Hand::Neutral, 0.0), "positive");
    assert_err(Insert::try_new(InsertShape::Groove, 0.0, Some(-1.0), Hand::Neutral, 0.0), "positive");
    assert!(Insert::try_new(InsertShape::Groove, 0.0, Some(2.5), Hand::Neutral, 0.0).is_ok());
    assert_err(Insert::try_new(InsertShape::T60, 0.4, Some(0.0), Hand::Right, 0.0), "positive");
    assert!(Insert::try_new(InsertShape::T60, 0.4, Some(1.0), Hand::Right, 0.0).is_ok());
    assert_err(Insert::try_new(InsertShape::C80, 0.4, None, Hand::Right, f64::NAN), "finite");
    assert_err(Insert::try_new(InsertShape::C80, 0.4, None, Hand::Right, 360.1), "360");
    assert_err(Insert::try_new(InsertShape::C80, 0.4, None, Hand::Right, -360.1), "360");
    assert!(Insert::try_new(InsertShape::C80, 0.4, None, Hand::Right, -360.0).is_ok());
    assert!(Insert::try_new(InsertShape::C80, 0.4, None, Hand::Right, 360.0).is_ok());

    assert_err(TurnInsert::try_new(1, "", 1, insert_c80(), false), "empty");
    assert_err(TurnInsert::try_new(1, "turret1", 0, insert_c80(), false), "1 to 256");
    assert_err(TurnInsert::try_new(1, "turret1", u16::MAX, insert_c80(), false), "1 to 256");
    assert_err(TurnBinding::try_new(0, "", "turret1", 0), "empty");
    assert_err(TurnBinding::try_new(0, "main", "", 0), "empty");
}

#[test]
fn setup_checks_reject_bad_structure() {
    let spindle = || spindle_named("main");
    let head = || head_named("turret1", 12, false);
    assert_err(setup_with(vec![], vec![head()], vec![], vec![]), "1 to 4 spindles");
    let five: Vec<_> = (0..5).map(|i| spindle_named(&format!("s{i}"))).collect();
    assert_err(setup_with(five, vec![head()], vec![], vec![]), "1 to 4 spindles");
    assert_err(setup_with(vec![spindle(), spindle()], vec![head()], vec![], vec![]), "duplicate spindle");
    assert_err(setup_with(vec![spindle()], vec![], vec![], vec![]), "1 to 8 heads");
    let nine: Vec<_> = (0..9).map(|i| head_named(&format!("h{i}"), 1, false)).collect();
    assert_err(setup_with(vec![spindle()], nine, vec![], vec![]), "1 to 8 heads");
    assert_err(setup_with(vec![spindle()], vec![head(), head()], vec![], vec![]), "duplicate head");

    let four: Vec<_> = (0..4).map(|i| spindle_named(&format!("s{i}"))).collect();
    let eight: Vec<_> = (0..8).map(|i| head_named(&format!("h{i}"), 1, false)).collect();
    assert!(setup_with(four, eight, vec![], vec![]).is_ok());

    assert_err(setup_with(vec![spindle()], vec![head()], vec![mounted(1, "missing", 1, false)], vec![]), "insert head");
    assert_err(setup_with(vec![spindle()], vec![head()], vec![mounted(1, "turret1", 13, false)], vec![]), "past head");
    assert!(setup_with(vec![spindle()], vec![head()], vec![mounted(1, "turret1", 12, false)], vec![]).is_ok());
    let dup = vec![mounted(1, "turret1", 1, false), mounted(1, "turret1", 2, false)];
    assert_err(setup_with(vec![spindle()], vec![head()], dup, vec![]), "duplicate turning insert");
    assert_err(setup_with(vec![spindle()], vec![head()], vec![mounted(1, "turret1", 1, true)], vec![]), "live tooling");
    let live_head = head_named("turret1", 12, true);
    assert!(setup_with(vec![spindle()], vec![live_head], vec![mounted(1, "turret1", 1, true)], vec![]).is_ok());

    let bad_spindle = TurnBinding::try_new(0, "sub", "turret1", 0).expect("binding");
    assert_err(setup_with(vec![spindle()], vec![head()], vec![], vec![bad_spindle]), "binding spindle");
    let bad_head = TurnBinding::try_new(0, "main", "other", 0).expect("binding");
    assert_err(setup_with(vec![spindle()], vec![head()], vec![], vec![bad_head]), "binding head");
    let twice = vec![TurnBinding::try_new(3, "main", "turret1", 0).expect("a"), TurnBinding::try_new(3, "main", "turret1", 0).expect("b")];
    assert_err(setup_with(vec![spindle()], vec![head()], vec![], twice), "more than one");

    let many_bindings: Vec<_> = (0..1025).map(|i| TurnBinding::try_new(i, "main", "turret1", 0).expect("binding")).collect();
    assert_err(setup_with(vec![spindle()], vec![head()], vec![], many_bindings), "too many turning bindings");
    let enough: Vec<_> = (0..1024).map(|i| TurnBinding::try_new(i, "main", "turret1", 0).expect("binding")).collect();
    assert!(setup_with(vec![spindle()], vec![head()], vec![], enough).is_ok());

    let many_inserts: Vec<_> = (0..1025).map(|id| mounted(id, "turret1", 1, false)).collect();
    assert_err(setup_with(vec![spindle()], vec![head()], many_inserts, vec![]), "too many turning inserts");
}

#[test]
fn program_checks_link_tools_channels_axes_and_poses() {
    let setup = m10(None)
        .with_insert(mounted(2, "turret1", 1, false))
        .expect("insert")
        .with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding"))
        .expect("binding");
    let path = path_with(Some(1), vec![rapid(10.0, 0.0, 1.0), linear(8.0, 0.0, -5.0)]);
    assert_err(program_with(vec![tool(1)], vec![path], setup, None), "missing tool 2");

    let setup = m10(None).with_binding(TurnBinding::try_new(1, "main", "turret1", 0).expect("binding")).expect("binding");
    let path = path_with(None, vec![rapid(10.0, 0.0, 0.0)]);
    assert_err(program_with(vec![], vec![path], setup, None), "outside 0..1");

    let setup = m10(None).with_binding(TurnBinding::try_new(0, "main", "turret1", 4).expect("binding")).expect("binding");
    let path = path_with(None, vec![rapid(10.0, 0.0, 0.0)]);
    let machine = lathe_machine(&[0, 1, 2], 0);
    assert_err(program_with(vec![], vec![path], setup, Some(machine)), "missing channel 4");

    let spindle = Spindle::try_new("main", Some(9), 0.0, 3_000.0).expect("spindle");
    let head = head_named("turret1", 12, false);
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding");
    let setup = setup_with(vec![spindle], vec![head], vec![], vec![binding]).expect("setup");
    let path = path_with(None, vec![rapid(10.0, 0.0, 0.0)]);
    let machine = lathe_machine(&[0, 1, 2], 0);
    assert_err(program_with(vec![], vec![path.clone()], setup.clone(), Some(machine)), "missing axis 9");
    assert!(program_with(vec![], vec![path], setup, None).is_ok());

    let spindle = Spindle::try_new("main", Some(2), 0.0, 3_000.0).expect("spindle");
    let head = head_named("turret1", 12, false);
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding");
    let setup = setup_with(vec![spindle], vec![head], vec![], vec![binding]).expect("setup");
    let machine = lathe_machine(&[0, 1, 2], 0);
    let path = path_with(None, vec![rapid(10.0, 0.0, 0.0)]);
    assert!(program_with(vec![], vec![path], setup, Some(machine)).is_ok());

    let setup = m10(None).with_binding(TurnBinding::try_new(0, "main", "turret1", 7).expect("binding")).expect("binding");
    let path = path_with(None, vec![rapid(1.0, 0.0, 0.0)]);
    assert!(program_with(vec![], vec![path], setup, None).is_ok());
}

#[test]
fn turning_poses_stay_on_the_radius_plane() {
    let bind = |index: u32| m10(None).with_binding(TurnBinding::try_new(index, "main", "turret1", 0).expect("binding")).expect("binding");

    assert_err(program_with(vec![], vec![path_with(None, vec![rapid(10.0, 2e-6, 0.0)])], bind(0), None), "turning pose Y");
    assert_err(program_with(vec![], vec![path_with(None, vec![linear(10.0, -2e-6, 0.0)])], bind(0), None), "turning pose Y");
    // A face may cross the spindle axis. The tip radius is then negative by about the nose radius.
    assert!(program_with(vec![], vec![path_with(None, vec![rapid(-0.8, 0.0, 0.0)])], bind(0), None).is_ok());
    assert!(program_with(vec![], vec![path_with(None, vec![rapid(0.0, 1e-6, -1.0)])], bind(0), None).is_ok());
    assert!(program_with(vec![], vec![path_with(None, vec![rapid(0.0, -1e-6, -1.0)])], bind(0), None).is_ok());
    assert!(program_with(vec![], vec![path_with(None, vec![rapid(-1e-6, 0.0, -1.0)])], bind(0), None).is_ok());

    let center = Vec3::new(20.0, 5.0, -10.0).expect("center");
    let arc = ClBlock::Motion(ClMotion::Arc {
        to: pose_xz(20.0, 0.0, -5.0),
        plane: Plane::Zx,
        center,
        sense: ArcSense::Cw,
        feed: 50.0,
        role: MoveRole::Cut,
    });
    assert!(program_with(vec![], vec![path_with(None, vec![arc])], bind(0), None).is_ok());

    let helix = ClBlock::Motion(ClMotion::Helix {
        to: pose_xz(15.0, 0.0, -8.0),
        plane: Plane::Zx,
        center: Vec3::new(15.0, 9.0, 0.0).expect("center"),
        sense: ArcSense::Ccw,
        sweep: 1.0,
        feed: 40.0,
        role: MoveRole::Cut,
    });
    assert!(program_with(vec![], vec![path_with(None, vec![helix])], bind(0), None).is_ok());

    let probe = ClBlock::Event(Event::Probe { toward: Some(pose_xz(4.0, 1.0, -1.0)), feed: Some(20.0) });
    assert_err(program_with(vec![], vec![path_with(None, vec![probe])], bind(0), None), "turning pose Y");
    let note = ClBlock::Event(Event::Comment { text: "face".into() });
    assert!(program_with(vec![], vec![path_with(None, vec![note])], bind(0), None).is_ok());

    let good = path_with(None, vec![rapid(5.0, 0.0, 0.0)]);
    let unbound_bad = path_with(None, vec![rapid(5.0, 3.0, 0.0)]);
    assert!(program_with(vec![], vec![good, unbound_bad], bind(0), None).is_ok());
    let unbound_bad = path_with(None, vec![rapid(5.0, 3.0, 0.0)]);
    let bound_good = path_with(None, vec![linear(5.0, 0.0, -2e-6)]);
    assert!(program_with(vec![], vec![unbound_bad, bound_good], bind(1), None).is_ok());
    let unbound_good = path_with(None, vec![rapid(5.0, 3.0, 0.0)]);
    let bound_bad = path_with(None, vec![linear(1.0, 1.0, -2.0)]);
    assert_err(program_with(vec![], vec![unbound_good, bound_bad], bind(1), None), "toolpath 1");
}

#[test]
fn rotary_and_turning_cannot_be_combined() {
    let turning = m10(None);
    let rotary = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, None).expect("rotary");
    let with_rotary = Program::try_new(None, Vec::new(), None, Vec::new(), Vec::new(), Vec::new(), Some(rotary)).expect("program");
    assert_err(with_rotary.with_turning(Some(turning.clone())), "rotary and turning setups cannot be combined yet");
    let with_turning = Program::try_new(None, Vec::new(), None, Vec::new(), Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(turning))
        .expect("turning");
    assert_err(with_turning.clone().with_rotary(Some(rotary)), "rotary and turning setups cannot be combined yet");
    assert!(with_turning.with_turning(None).expect("clear").turning().is_none());
}

#[test]
fn thread_motion_needs_turning_and_a_positive_lead() {
    let thread = ClBlock::Motion(ClMotion::Thread { to: pose_xz(10.0, 0.0, -4.0), lead: 2.5 });
    let program = program_with(
        vec![],
        vec![path_with(None, vec![rapid(10.0, 0.0, 0.0), thread])],
        m10(None).with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding")).expect("bind"),
        None,
    )
    .expect("program");
    let doc = Document::new(program).expect("document");
    let json = doc.to_json().expect("json");
    assert!(json.contains("\"type\":\"thread\""), "{json}");
    let loaded = Document::from_json(&json).expect("load");
    assert!(matches!(loaded.program().toolpaths()[0].blocks()[1], ClBlock::Motion(ClMotion::Thread { lead, .. }) if lead == 2.5));
    let report = crate::verify(loaded.program());
    assert!(report.issues().is_empty(), "{:?}", report.issues());
    assert!(report.total_length() > 0.0);
    assert_eq!(report.rapid_length(), 0.0);
    let ngc = crate::write_ngc(loaded.program()).expect_err("ngc");
    assert!(ngc.to_string().contains("thread"), "{ngc}");

    let bare_path = path_with(None, vec![ClBlock::Motion(ClMotion::Thread { to: pose_xz(8.0, 0.0, -1.0), lead: 1.25 })]);
    let bare = Program::try_new(None, Vec::new(), None, vec![bare_path], Vec::new(), Vec::new(), None).expect("intermediate");
    assert_err(Document::new(bare.clone()), "turning program");
    assert_err(bare.with_turning(None), "turning program");

    let off_plane = ClBlock::Motion(ClMotion::Thread { to: pose_xz(10.0, 0.01, -1.0), lead: 1.5 });
    let bind = m10(None).with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding")).expect("bind");
    assert_err(program_with(vec![], vec![path_with(None, vec![off_plane])], bind, None), "turning pose Y");

    for lead in [-1.0, f64::NAN, 1_000.1] {
        let broken = ClPath::try_new(
            None,
            None,
            CompensationMode::Off,
            feeds(),
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Thread { to: pose_xz(1.0, 0.0, 0.0), lead })],
        );
        assert!(broken.is_err(), "lead {lead}");
    }
    assert!(
        ClPath::try_new(
            None,
            None,
            CompensationMode::Off,
            feeds(),
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Thread { to: pose_xz(1.0, 0.0, 0.0), lead: 1_000.0 })],
        )
        .is_ok()
    );
}

#[test]
fn deserialized_turning_is_rechecked_and_hostile_input_does_not_panic() {
    let doc = turning_document();
    let mut value = serde_json::from_str::<serde_json::Value>(&doc.to_json().expect("json")).expect("value");
    value["program"]["turning"]["spindles"][0]["max_rpm"] = serde_json::json!(0);
    assert!(Document::from_json(&serde_json::to_string(&value).expect("json")).is_err());

    let mut value = serde_json::from_str::<serde_json::Value>(&doc.to_json().expect("json")).expect("value");
    value["program"]["turning"]["inserts"][0]["insert"]["nose_radius"] = serde_json::json!(-1.0);
    assert!(Document::from_json(&serde_json::to_string(&value).expect("json")).is_err());

    let mut value = serde_json::from_str::<serde_json::Value>(&doc.to_json().expect("json")).expect("value");
    value["program"]["turning"]["inserts"][0]["station"] = serde_json::json!(400);
    assert!(Document::from_json(&serde_json::to_string(&value).expect("json")).is_err());

    let mut value = serde_json::from_str::<serde_json::Value>(&doc.to_json().expect("json")).expect("value");
    value["program"]["turning"]["spindles"][0]["name"] = serde_json::json!("");
    assert!(Document::from_json(&serde_json::to_string(&value).expect("json")).is_err());

    let hostile = [
        "",
        "null",
        "{",
        r#"{"format":"campanion-ir/1","program":{"turning":{"spindles":[]}}}"#,
        r#"{"format":"campanion-ir/1","program":{"tools":[],"toolpaths":[],"channel_programs":[],"syncs":[],"turning":{"nope":1}}}"#,
        r#"{"format":"campanion-ir/1","program":{"tools":[],"toolpaths":[],"channel_programs":[],"syncs":[],"turning":{"spindles":[{"name":"main","work_zero_z":0,"max_rpm":1}],"heads":[{"name":"turret1","axes":["x","z","q"],"stations":12,"live_tooling":false}],"inserts":[],"bindings":[]}}}"#,
        r#"{"format":"campanion-ir/1","program":{"tools":[],"toolpaths":[],"channel_programs":[],"syncs":[],"turning":{"spindles":[{"name":"main","work_zero_z":0,"max_rpm":1}],"heads":[{"name":"turret1","axes":["x","z"],"stations":12,"live_tooling":false}],"inserts":[{"tool_id":1,"head":"turret1","station":99999,"insert":{"shape":"c80","nose_radius":0.4,"hand":"right","orientation_deg":0},"live":false}],"bindings":[]}}}"#,
    ];
    for blob in hostile {
        let _ = Document::from_json(blob);
    }

    let mut state = 0x1234_5678_u64;
    for _ in 0..80 {
        let a = fuzz_f64(&mut state);
        let b = fuzz_f64(&mut state);
        let c = fuzz_f64(&mut state);
        let name = fuzz_name(&mut state);
        let station = (lcg(&mut state) % 4_000) as u16;
        if let Ok(speed) = SpindleSpeed::css(a, b) {
            let rpm = speed.rpm_at_radius(c);
            assert!(rpm.is_finite() && rpm >= 0.0 && rpm <= b);
        }
        let _ = SpindleSpeed::css(a, b);
        if let Ok(speed) = SpindleSpeed::rpm(a) {
            assert_eq!(speed.rpm_at_radius(c), a);
        }
        let _ = Spindle::try_new(name.clone(), None, a, b);
        let _ = Head::try_new(name, vec![HeadAxis::X, HeadAxis::Z, HeadAxis::Y, HeadAxis::B, HeadAxis::X], station, false);
        let _ = TurnStock::bar(a, b);
        let _ = TurnStock::tube(a, b, c);
        let _ = Insert::try_new(InsertShape::R, a, Some(b), Hand::Neutral, c);
        let _ = Insert::try_new(InsertShape::Groove, a, None, Hand::Left, c);
        let _ = TurningSetup::single(None, a);
        if c.is_finite()
            && let Ok(insert) = Insert::try_new(InsertShape::C80, a.abs().min(10.0), None, Hand::Right, c.clamp(-360.0, 360.0))
        {
            let (dx, dz) = insert.tip_offset();
            assert!(dx.is_finite() && dz.is_finite());
        }
    }
}

#[test]
fn binding_speed_round_trips_and_css_respects_the_spindle_cap() {
    let css = SpindleSpeed::css(200.0, 2500.0).expect("css");
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(css)).expect("speed");
    assert_eq!(binding.speed(), Some(css));
    let cleared = binding.clone().with_speed(None).expect("clear");
    assert!(cleared.speed().is_none());

    let setup = m10(None).with_binding(binding).expect("binding");
    assert_eq!(setup.binding_for(0).expect("binding").speed(), Some(css));
    let json = serde_json::to_string(setup.binding_for(0).expect("binding")).expect("json");
    assert!(json.contains("\"mode\":\"css\""), "{json}");
    assert!(json.contains("\"surface_m_per_min\":200"), "{json}");

    let bare = TurnBinding::try_new(0, "main", "turret1", 0).expect("bare");
    let bare_json = serde_json::to_string(&bare).expect("json");
    assert!(!bare_json.contains("speed"), "{bare_json}");
    let loaded: TurnBinding = serde_json::from_str(&bare_json).expect("load");
    assert!(loaded.speed().is_none());

    let hot = TurnBinding::try_new(0, "main", "turret1", 0)
        .expect("binding")
        .with_speed(Some(SpindleSpeed::css(200.0, 5_000.0).expect("css")))
        .expect("speed");
    assert_err(m10(None).with_binding(hot), "css max rpm");

    let capped = TurnBinding::try_new(0, "main", "turret1", 0)
        .expect("binding")
        .with_speed(Some(SpindleSpeed::css(10.0, 4_000.0).expect("css")))
        .expect("speed");
    assert!(m10(None).with_binding(capped).is_ok());

    let direct =
        TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(SpindleSpeed::rpm(9_000.0).expect("rpm"))).expect("speed");
    assert!(m10(None).with_binding(direct).is_ok());

    let bad = SpindleSpeed::Css { surface_m_per_min: -1.0, max_rpm: 100.0 };
    assert_err(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(bad)), "surface speed");
}

#[test]
fn section_bounds_round_trip_and_old_setups_still_load() {
    assert_err(TurnSection::try_new(vec![[0.0, 1.0]]), "found 1");
    assert_err(TurnSection::try_new(vec![[f64::NAN, 1.0], [-1.0, 1.0]]), "finite");
    assert_err(TurnSection::try_new(vec![[0.0, -1.0], [-5.0, 2.0]]), ">=");
    let points = [[0.0, 0.0], [-12.0, 8.0]];
    let section = TurnSection::try_new(points.to_vec()).expect("two points");
    assert_eq!(section.points(), &points);

    let setup = m10(None).with_section(Some(section)).expect("section");
    let json = serde_json::to_string(&setup).expect("json");
    let loaded: TurningSetup = serde_json::from_str(&json).expect("round trip");
    assert_eq!(loaded, setup);
    assert_eq!(loaded.section().expect("section").points(), &points);
    let program = program_with(Vec::new(), Vec::new(), setup, None).expect("program");
    round_trip(&Document::new(program).expect("document"));

    let bare = m10(None);
    let bare_json = serde_json::to_string(&bare).expect("bare json");
    assert!(!bare_json.contains("\"section\""), "{bare_json}");
    let loaded: TurningSetup = serde_json::from_str(&bare_json).expect("setup without section");
    assert!(loaded.section().is_none());
    let old = r#"{"format":"campanion-ir/1","program":{"tools":[],"toolpaths":[],"channel_programs":[],"syncs":[],"turning":{"spindles":[{"name":"main","work_zero_z":0,"max_rpm":1}],"heads":[{"name":"turret1","axes":["x","z"],"stations":12,"live_tooling":false}],"inserts":[],"bindings":[]}}}"#;
    let doc = Document::from_json(old).expect("old setup");
    assert!(doc.program().turning().expect("turning").section().is_none());

    let unknown = r#"{"spindles":[{"name":"main","work_zero_z":0.0,"max_rpm":1000.0}],"heads":[{"name":"turret1","axes":["x","z"],"stations":12,"live_tooling":false}],"inserts":[],"bindings":[],"nope":1}"#;
    let err = serde_json::from_str::<TurningSetup>(unknown).expect_err("unknown setup field");
    assert!(err.to_string().contains("unknown field"), "{err}");
    let bad_section = r#"{"points":[[0.0,1.0],[-1.0,1.0]],"nope":1}"#;
    let err = serde_json::from_str::<TurnSection>(bad_section).expect_err("unknown section field");
    assert!(err.to_string().contains("unknown field"), "{err}");
}

fn turning_document() -> Document {
    let setup = m10(Some(TurnStock::bar(50.0, 80.0).expect("stock")))
        .with_insert(mounted(1, "turret1", 1, false))
        .expect("insert")
        .with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding"))
        .expect("binding");
    let blocks = vec![rapid(30.0, 0.0, 1.0), linear(25.0, 0.0, -20.0)];
    let program = program_with(vec![tool(1)], vec![path_with(Some(1), blocks)], setup, None).expect("program");
    Document::new(program).expect("document")
}

fn lcg(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
    *state
}

fn fuzz_f64(state: &mut u64) -> f64 {
    f64::from_bits(lcg(state))
}

fn fuzz_name(state: &mut u64) -> String {
    let n = (lcg(state) % 8) as usize;
    if n == 0 {
        return String::new();
    }
    (0..n).map(|_| char::from(b'a' + (lcg(state) % 26) as u8)).collect()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn css_rpm_is_positive_and_capped(
        surface in 0.01f64..5_000.0,
        max_rpm in 0.01f64..100_000.0,
        radius in -20.0f64..2_000.0,
    ) {
        let speed = SpindleSpeed::css(surface, max_rpm).expect("css");
        let rpm = speed.rpm_at_radius(radius);
        prop_assert!(rpm.is_finite());
        prop_assert!(rpm > 0.0);
        prop_assert!(rpm <= max_rpm);
    }

    #[test]
    fn direct_rpm_ignores_radius(rpm in 0.01f64..100_000.0, radius in -1_000.0f64..1_000.0) {
        let speed = SpindleSpeed::rpm(rpm).expect("rpm");
        prop_assert!((speed.rpm_at_radius(radius) - rpm).abs() <= 1e-9);
    }

    #[test]
    fn tip_offset_stays_on_the_radius_corners(deg in -360.0f64..360.0, radius in 0.0f64..20.0) {
        let insert = Insert::try_new(InsertShape::W80, radius, None, Hand::Neutral, deg).expect("insert");
        let (dx, dz) = insert.tip_offset();
        prop_assert!(near_radius(dx, radius) && near_radius(dz, radius));
    }
}

fn near_radius(value: f64, radius: f64) -> bool {
    value == 0.0 || (value - radius).abs() <= 1e-9 || (value + radius).abs() <= 1e-9
}
