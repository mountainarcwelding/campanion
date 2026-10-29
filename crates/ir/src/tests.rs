use std::collections::BTreeMap;
use std::f64::consts::PI;

use campanion_geom::{Contour, Line, Point, Segment, Tolerance};
use campanion_path::{
    Compensation, ControllerSide, CutDirection, CutKind, Lead, Move, Point3, ProcessEvent, ProfileOp, ProfileSide, TabSpec, Tool as PathTool,
    Toolpath,
};

use crate::geom::{ArcSense, Plane, Pose, Vec3};
use crate::machine::{Axis, AxisKind, AxisLetter, AxisLimits, AxisName, Channel, KinematicNode, Machine};
use crate::motion::{AxisBlock, AxisMotion, AxisTarget, ChannelProgram, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, ProfileTab};
use crate::process::{CoolantMode, Event, ParamValue, ProcessKind, ProcessParams, Tool};
use crate::program::{Document, Program, SyncPoint};
use crate::rotary::{RotaryAxisKind, RotaryDirection, RotaryLimits, RotarySetup, machine_to_part};
use crate::{FORMAT_MAJOR, FORMAT_TAG, FiveAxisSetup, IrError, Kinematics, MAX_AXES, SolutionPolicy, check_format, json_schema};

#[test]
fn ir_constants_and_error_display() {
    assert_eq!(FORMAT_TAG, "campanion-ir/1");
    assert_eq!(FORMAT_MAJOR, 1);
    const {
        assert!(MAX_AXES >= 13);
        assert!(MAX_AXES == 16);
    }
    assert_eq!(IrError::NotSupported("gcode".into()).to_string(), "not supported yet: gcode");
    assert_eq!(IrError::InvalidInput("axis".into()).to_string(), "invalid input: axis");
    assert_eq!(IrError::UnsupportedVersion { found: 2, supported: 1 }.to_string(), "unsupported IR major version 2; this build reads major 1");
    assert_eq!(IrError::UnknownFormat("nope".into()).to_string(), "unrecognised format tag `nope`");
}

#[test]
fn json_round_trip_is_unchanged() {
    let empty = Document::new(Program::default()).expect("empty");
    assert_eq!(empty.format(), FORMAT_TAG);
    round_trip(&empty);

    let sample = thirteen_axis_sample();
    round_trip(&sample);

    let json = sample.to_json().expect("json");
    let minor = json.replacen("\"format\":\"campanion-ir/1\"", "\"format\":\"campanion-ir/1.0\"", 1);
    let loaded = Document::from_json(&minor).expect("minor");
    assert_eq!(loaded.format(), "campanion-ir/1.0");
    assert_eq!(loaded.program().machine().expect("machine").axes().len(), 13);
    assert_eq!(loaded.program().machine().expect("machine").channels().len(), 2);
}

#[test]
fn format_tag_is_checked_on_load() {
    let err = Document::from_json(r#"{"format":"campanion-ir/2"}"#).expect_err("major 2");
    assert_eq!(err, IrError::UnsupportedVersion { found: 2, supported: FORMAT_MAJOR });
    let err = Document::from_json(r#"{"format":"campanion-ir/2.0","program":{}}"#).expect_err("major 2.0");
    assert_eq!(err, IrError::UnsupportedVersion { found: 2, supported: FORMAT_MAJOR });
    let err = Document::from_json(r#"{"format":"campanion-ir/0"}"#).expect_err("major 0");
    assert_eq!(err, IrError::UnsupportedVersion { found: 0, supported: FORMAT_MAJOR });
    assert_eq!(Document::from_json(r#"{"format":"nope"}"#).expect_err("tag"), IrError::UnknownFormat("nope".into()));
    assert!(Document::from_json(r#"{"format":"campanion-ir/"}"#).is_err());
    assert!(Document::from_json(r#"{"format":"campanion-ir/01"}"#).is_err());
    assert!(check_format("campanion-ir/1.2.3").is_ok());
    assert!(matches!(check_format("campanion-ir/3"), Err(IrError::UnsupportedVersion { found: 3, supported: 1 })));
    assert!(Document::from_json("{}").is_err());
    assert!(Vec3::new(f64::NAN, 0.0, 0.0).is_err());
    assert!(Feeds::try_new(f64::INFINITY, None, None, None, None).is_err());
}

#[test]
fn thirteen_axis_two_channel_sample_loads() {
    let sample = thirteen_axis_sample();
    let loaded = Document::from_json(&sample.to_json().expect("json")).expect("load");
    let machine = loaded.program().machine().expect("machine");
    assert_eq!(machine.axes().len(), 13);
    assert_eq!(machine.channels().len(), 2);
    assert_eq!(machine.channels()[0].name(), "primary");
    assert_eq!(machine.channels()[0].axes(), &[0, 1, 2, 3, 4, 5, 6]);
    assert_eq!(machine.channels()[1].name(), "secondary");
    assert_eq!(machine.channels()[1].axes(), &[7, 8, 9, 10, 11, 12]);
    assert!(matches!(machine.axes()[0].name(), AxisName::Letter(AxisLetter::X)));
    assert!(matches!(machine.axes()[8].name(), AxisName::Letter(AxisLetter::W)));
    assert_eq!(axis_index(machine.axes()[9].name()), Some(9));
    assert_eq!(axis_index(machine.axes()[12].name()), Some(12));
    assert_eq!(machine.axes()[3].kind(), AxisKind::Rotary);
    assert_eq!(machine.axes()[2].kind(), AxisKind::Linear);
    let limits = machine.axes()[0].limits().expect("x limits");
    assert_eq!((limits.min(), limits.max()), (-400.0, 400.0));
    assert_eq!(machine.axes()[1].chain().parent(), Some(0));
    assert_eq!(machine.axes()[6].chain().offset().z(), 50.0);
    assert_eq!(loaded.program().syncs().len(), 1);
    assert_eq!(loaded.program().syncs()[0].channels(), &[0, 1]);
    assert_eq!(loaded.program().channel_programs().len(), 2);
    assert_eq!(loaded.program().toolpaths().len(), 1);
    assert!(loaded.program().toolpaths()[0].blocks().iter().any(|block| matches!(block, ClBlock::Motion(ClMotion::Helix { .. }))));
    assert!(loaded.program().toolpaths()[0].blocks().iter().any(|block| matches!(block, ClBlock::Event(Event::Custom { .. }))));
}

#[test]
fn thirteen_axis_two_channel_sample_verifies_with_no_errors() {
    let report = crate::verify(thirteen_axis_sample().program());
    assert!(report.issues().is_empty(), "{:?}", report.issues());
}

#[test]
fn ngc_writes_both_pierce_and_post_delay_orders() {
    // Default is process on, then the pierce dwell, and process off, then the post dwell.
    assert_eq!(
        timed_ngc(true, false),
        "\
G21
G90
G0 X0.0000 Y0.0000 Z0.0000
M3 (spindle)
G4 P0.2500
G1 X1.0000 Y0.0000 Z0.0000 F100.0000
M5 (spindle)
G4 P0.1000
M30
"
    );
    assert_eq!(
        timed_ngc(false, true),
        "\
G21
G90
G0 X0.0000 Y0.0000 Z0.0000
G4 P0.2500
M3 (spindle)
G1 X1.0000 Y0.0000 Z0.0000 F100.0000
G4 P0.1000
M5 (spindle)
M30
"
    );
    assert_eq!(
        timed_ngc(true, true),
        "\
G21
G90
G0 X0.0000 Y0.0000 Z0.0000
M3 (spindle)
G4 P0.2500
G1 X1.0000 Y0.0000 Z0.0000 F100.0000
G4 P0.1000
M5 (spindle)
M30
"
    );
    assert_eq!(
        timed_ngc(false, false),
        "\
G21
G90
G0 X0.0000 Y0.0000 Z0.0000
G4 P0.2500
M3 (spindle)
G1 X1.0000 Y0.0000 Z0.0000 F100.0000
M5 (spindle)
G4 P0.1000
M30
"
    );
}

fn timed_ngc(pierce_on: bool, post_on: bool) -> String {
    let point = |x, y, z| Point3::new(x, y, z).expect("point");
    let path = Toolpath::new(vec![
        Move::Rapid { to: point(0.0, 0.0, 0.0) },
        Move::Event(ProcessEvent::On),
        Move::Linear { to: point(1.0, 0.0, 0.0), kind: CutKind::Cut },
        Move::Event(ProcessEvent::Off),
    ])
    .expect("path");
    let params = ProcessParams::new().pierce_delay(0.25, pierce_on).expect("pierce").post_delay(0.10, post_on).expect("post");
    let import = crate::Import::new(ProcessKind::Spindle, 100.0).expect("import").params(params).expect("params");
    let doc = Document::from_toolpath(&path, &import).expect("convert");
    crate::write_ngc(doc.program()).expect("ngc")
}

#[test]
fn too_many_axes_are_rejected() {
    let mut axes = Vec::new();
    for ordinal in 0..MAX_AXES {
        axes.push(sample_axis(ordinal, None));
    }
    let ids: Vec<u8> = (0..MAX_AXES).map(|ordinal| u8::try_from(ordinal).expect("id")).collect();
    let channel = Channel::try_new(0, "all", ids).expect("channel");
    let machine = Machine::try_new(axes, vec![channel]).expect("16 axes");
    let program = Program::try_new(None, Vec::new(), Some(machine), Vec::new(), Vec::new(), Vec::new(), None).expect("program");
    let doc = Document::new(program).expect("document");
    assert_eq!(doc.program().machine().expect("machine").axes().len(), MAX_AXES);

    let err = Machine::try_new(vec![sample_axis(0, None); 17], Vec::new()).expect_err("17");
    assert!(err.to_string().contains("too many axes: 17 > 16"), "{err}");

    let mut value: serde_json::Value = serde_json::from_str(&doc.to_json().expect("json")).expect("value");
    let Some(list) = value.pointer_mut("/program/machine/axes").and_then(|axes| axes.as_array_mut()) else {
        panic!("axes array");
    };
    let extra = list.first().expect("axis").clone();
    list.push(extra);
    let err = Document::from_json(&serde_json::to_string(&value).expect("json")).expect_err("load 17");
    assert!(err.to_string().contains("too many axes"), "{err}");
    assert!(AxisName::for_ordinal(MAX_AXES).is_err());
}

#[test]
fn profile_converts_to_ir() {
    let profile = ProfileOp::new(
        PathTool::Diameter { diameter: 6.0 },
        Compensation::Controller(ControllerSide::Left),
        ProfileSide::On,
        CutDirection::Conventional,
    )
    .lead_in(Lead::Arc { radius: 3.0 })
    .tabs(TabSpec::count(1, 4.0).height(2.0))
    .generate(&rect(40.0, 20.0))
    .expect("profile");
    let feeds = Feeds::try_new(1000.0, Some(600.0), Some(700.0), Some(300.0), Some(800.0)).expect("feeds");
    let params =
        ProcessParams::new().speed(Some(12_000.0)).expect("speed").pierce_delay(0.3, false).expect("pierce").post_delay(0.15, true).expect("post");
    let tool = Tool::try_new(4, Some("flat".into()), Some(6.0), None).expect("tool");
    let import = crate::Import::new(ProcessKind::Spindle, 1000.0)
        .expect("import")
        .name("outline")
        .expect("name")
        .feeds(feeds)
        .expect("feeds")
        .params(params)
        .expect("params")
        .tool(tool)
        .expect("tool");
    let doc = Document::from_profile(&profile, &import).expect("convert");
    round_trip(&doc);

    let path = &doc.program().toolpaths()[0];
    assert_eq!(path.name(), Some("outline"));
    assert_eq!(path.compensation(), CompensationMode::ControllerLeft);
    assert_eq!(path.tool_id(), Some(4));
    assert_eq!(doc.program().tools()[0].diameter(), Some(6.0));
    let pierce = path.pierce().expect("pierce");
    assert_eq!((pierce.x(), pierce.y(), pierce.z()), (profile.pierce().x(), profile.pierce().y(), profile.pierce().z()));
    assert_eq!(path.tabs().len(), 1);
    // Side on, 6 mm tool: the tool-centre gap is the 4 mm bridge plus the diameter.
    let gap = path.tabs()[0].width();
    assert!((gap - 10.0).abs() < 1e-9, "tool-centre gap {gap}");
    assert_eq!(path.tabs()[0].height(), Some(2.0));

    let blocks = path.blocks();
    assert!(
        blocks
            .iter()
            .any(|block| matches!(block, ClBlock::Motion(ClMotion::Arc { plane: Plane::Xy, role: MoveRole::LeadIn, feed, .. }) if *feed == 600.0))
    );
    assert!(blocks.iter().any(|block| matches!(block, ClBlock::Motion(ClMotion::Linear { role: MoveRole::Cut, feed, .. }) if *feed == 1000.0)));

    let on_at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::ProcessOn { .. }))).expect("process on");
    let ClBlock::Event(Event::ProcessOn { process, params }) = &blocks[on_at] else {
        panic!("process on");
    };
    assert_eq!(process, &ProcessKind::Spindle);
    assert_eq!(params.pierce_delay_s(), 0.3);
    assert!(!params.pierce_process_on_during_delay());
    assert_eq!(params.post_delay_s(), 0.15);
    assert!(params.post_process_on_during_delay());
    assert_eq!(params.speed_value(), Some(12_000.0));
    assert!(params.pierce_height_z().is_none());
    assert!(matches!(blocks.get(on_at - 1), Some(ClBlock::Event(Event::Dwell { seconds })) if *seconds == 0.3));
    assert!(matches!(blocks.get(on_at + 1), Some(ClBlock::Motion(ClMotion::Linear { role: MoveRole::Plunge, .. }))));
    assert!(matches!(blocks.get(on_at + 2), Some(ClBlock::Motion(ClMotion::Arc { role: MoveRole::LeadIn, .. }))));
    assert!(blocks.iter().any(|block| matches!(block, ClBlock::Motion(ClMotion::Linear { role: MoveRole::Plunge, feed, .. }) if *feed == 300.0)));

    let off_at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }))).expect("off");
    assert!(matches!(blocks.get(off_at - 1), Some(ClBlock::Event(Event::Dwell { seconds })) if *seconds == 0.15));
    let ons = blocks.iter().filter(|block| matches!(block, ClBlock::Event(Event::ProcessOn { .. }))).count();
    let offs = blocks.iter().filter(|block| matches!(block, ClBlock::Event(Event::ProcessOff { .. }))).count();
    let dwells = blocks.iter().filter(|block| matches!(block, ClBlock::Event(Event::Dwell { .. }))).count();
    assert_eq!(dwells, ons + offs);

    let helix = helix_toolpath();
    let imported =
        Document::from_toolpath(&helix, &crate::Import::new(ProcessKind::Laser, 500.0).expect("import").feeds(feeds).expect("feeds")).expect("helix");
    let motion = imported.program().toolpaths()[0].blocks().iter().find_map(|block| match block {
        ClBlock::Motion(motion @ ClMotion::Helix { .. }) => Some(motion),
        _ => None,
    });
    let Some(ClMotion::Helix { plane, sense, sweep, role, feed, .. }) = motion else {
        panic!("expected a helix, got {:?}", imported.program().toolpaths()[0].blocks());
    };
    assert_eq!(*plane, Plane::Xy);
    assert_eq!(*sense, ArcSense::Ccw);
    assert_eq!(*role, MoveRole::Plunge);
    assert_eq!(*feed, 300.0);
    assert!((sweep - PI / 2.0).abs() < 1e-9, "{sweep}");
}

#[test]
fn committed_schema_matches_generated_schema() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/schema/campanion-ir-1.schema.json");
    let generated = json_schema().expect("schema");
    let mut pretty = serde_json::to_string_pretty(&generated).expect("pretty");
    pretty.push('\n');
    if std::env::var("UPDATE_IR_SCHEMA").ok().as_deref() == Some("1") {
        std::fs::write(&path, &pretty).expect("write schema");
    }
    let committed_text = std::fs::read_to_string(&path).expect("docs/schema/campanion-ir-1.schema.json");
    let committed: serde_json::Value = serde_json::from_str(&committed_text).unwrap_or(serde_json::Value::Null);
    if committed != generated {
        panic!("schema mismatch for {}. generated schema follows\n{pretty}", path.display());
    }
    assert!(generated.get("$schema").is_some());
    assert!(generated.get("properties").and_then(|props| props.get("format")).is_some());
}

#[test]
fn program_with_rotary_data_round_trips() {
    let doc = rotary_sample();
    round_trip(&doc);
    let json = doc.to_json().expect("json");
    assert!(json.contains("\"rotary\":{"));
    assert!(json.contains("\"axis\":\"a\""));
    assert!(json.contains("\"direction\":\"positive\""));
    let pose_json = json.split("\"to\":").nth(1).expect("pose");
    assert!(pose_json.contains("\"rotary\":90.0") || pose_json.contains("\"rotary\":90"));
}

#[test]
fn old_ir_files_load_and_reserialize_byte_identically() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let files = [
        "crates/post/tests/data/sample.ir.json",
        "crates/post/tests/data/curves.ir.json",
        "crates/post/tests/data/dome.ir.json",
        "crates/post/tests/data/plasma.ir.json",
        "crates/cli/tests/golden/sample.ir.json",
        "crates/cli/tests/golden/dome_rough.ir.json",
    ];
    for rel in files {
        let path = root.join(rel);
        let original = std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("{}: {err}", path.display()));
        let doc = Document::from_json(&original).unwrap_or_else(|err| panic!("{rel}: {err}"));
        assert!(doc.program().rotary().is_none(), "{rel} should have no rotary setup");
        assert!(doc.program().turning().is_none(), "{rel} should have no turning setup");
        assert!(doc.program().five_axis().is_none(), "{rel} should have no five-axis setup");
        let json = doc.to_json().expect("json");
        assert!(!json.contains("\"rotary\""), "{rel} gained a rotary field");
        assert!(!json.contains("\"turning\""), "{rel} gained a turning field");
        assert!(!json.contains("\"five_axis\""), "{rel} gained a five_axis field");
        assert!(!json.contains("\"continuous\""), "{rel} gained a continuous field");
        assert!(!json.contains("\"max_axis_step_deg\""), "{rel} gained max_axis_step_deg");
        assert!(doc.program().toolpaths().iter().all(|path| !path.continuous()), "{rel}");
        assert_eq!(doc.program().max_axis_step_deg(), 5.0, "{rel}");
        campanion_testsupport::ir_json_close(&json, &original).unwrap_or_else(|err| panic!("{rel}: {err}"));
        assert_eq!(Document::from_json(&json).expect("reload").to_json().expect("again"), json, "{rel}");
    }
}

#[test]
fn continuous_flag_defaults_are_omitted_and_out_of_range_steps_are_rejected() {
    let empty = Document::new(Program::default()).expect("empty");
    let json = empty.to_json().expect("json");
    assert!(!json.contains("continuous"), "{json}");
    assert!(!json.contains("max_axis_step_deg"), "{json}");
    assert_eq!(Document::from_json(&json).expect("load").program().max_axis_step_deg(), 5.0);

    let pose = Pose::new(Vec3::ZERO, Vec3::Z).expect("pose");
    let path = rapid_path(pose).with_continuous(true);
    let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_max_axis_step_deg(12.5)
        .expect("step");
    let doc = Document::new(program).expect("doc");
    let json = doc.to_json().expect("json");
    assert!(json.contains("\"continuous\":true"), "{json}");
    assert!(json.contains("\"max_axis_step_deg\":12.5"), "{json}");
    let loaded = Document::from_json(&json).expect("reload");
    assert!(loaded.program().toolpaths()[0].continuous());
    assert!((loaded.program().max_axis_step_deg() - 12.5).abs() < 1e-12);

    // An older file omits both keys. They load as the defaults and stay omitted on the way back out.
    let mut value = serde_json::to_value(&loaded).expect("value");
    value["program"].as_object_mut().expect("program").remove("max_axis_step_deg");
    value["program"]["toolpaths"][0].as_object_mut().expect("path").remove("continuous");
    let text = serde_json::to_string(&value).expect("text");
    let restored = Document::from_json(&text).expect("old shape");
    assert!(!restored.program().toolpaths()[0].continuous());
    assert_eq!(restored.program().max_axis_step_deg(), 5.0);
    let again = restored.to_json().expect("again");
    assert!(!again.contains("continuous"), "{again}");
    assert!(!again.contains("max_axis_step_deg"), "{again}");

    for bad in [0.0, -4.0, f64::NAN, 90.0 + 1e-6] {
        let err = Program::default().with_max_axis_step_deg(bad).expect_err("range");
        assert_eq!(err.to_string(), "invalid input: max_axis_step_deg must be in (0, 90]");
    }
    Program::default().with_max_axis_step_deg(90.0).expect("90");

    let mut zero = serde_json::to_value(Document::new(Program::default()).expect("empty")).expect("value");
    zero["program"]["max_axis_step_deg"] = serde_json::json!(0.0);
    let err = Document::from_json(&serde_json::to_string(&zero).expect("json")).expect_err("zero");
    assert!(err.to_string().contains("max_axis_step_deg must be in (0, 90]"), "{err}");
}

#[test]
fn rotary_pose_without_setup_is_rejected() {
    let pose = Pose::new(Vec3::ZERO, Vec3::Z).expect("pose").with_rotary(0.0).expect("angle");
    let program = Program::try_new(None, Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), None).expect("intermediate");
    let err = Document::new(program).expect_err("setup");
    assert!(err.to_string().contains("block 0"), "{err}");
    assert!(err.to_string().contains("rotary setup"), "{err}");
}

#[test]
fn rotary_angle_outside_limits_is_rejected() {
    let setup = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, Some(RotaryLimits::new(-90.0, 90.0).expect("limits")))
        .expect("setup");
    let pose = indexed_pose(Vec3::ZERO, &setup, 120.0);
    let err = Program::try_new(None, Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), Some(setup)).expect_err("limits");
    assert!(err.to_string().contains("block 0"), "{err}");
    assert!(err.to_string().contains("outside limits"), "{err}");
}

#[test]
fn rotary_pose_with_inconsistent_tool_axis_is_rejected() {
    let setup = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, None).expect("setup");
    let pose = Pose::new(Vec3::ZERO, Vec3::Z).expect("pose").with_rotary(90.0).expect("angle");
    let err = Program::try_new(None, Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), Some(setup)).expect_err("axis");
    assert!(err.to_string().contains("block 0"), "{err}");
    assert!(err.to_string().contains("tool axis"), "{err}");
}

#[test]
fn five_axis_round_trips_and_rejects_a_rotary_setup() {
    let kinematics = Kinematics::from_json(include_str!("../kinematics/table_table_bc.json")).expect("kinematics");
    let setup = FiveAxisSetup { kinematics, policy: SolutionPolicy::FirstPositive };
    let pose = Pose::new(Vec3::X, Vec3::X).expect("tilted");
    let program = Program::try_new(Some("tilt".into()), Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), None)
        .expect("program")
        .with_five_axis(setup.clone())
        .expect("five axis");
    assert_eq!(program.five_axis().map(|item| &item.policy), Some(&SolutionPolicy::FirstPositive));
    assert_eq!(program.five_axis().map(|item| item.kinematics.name.as_str()), Some("table_table_bc"));
    let doc = Document::new(program).expect("document");
    round_trip(&doc);
    let kept = doc.program().toolpaths()[0].blocks().iter().any(|block| match block {
        ClBlock::Motion(ClMotion::Rapid { to, .. }) => to.rotary().is_none() && to.axis() == Vec3::X,
        _ => false,
    });
    assert!(kept);
    let json = doc.to_json().expect("json");
    assert!(json.contains("\"five_axis\":{"));
    assert!(json.contains("\"policy\":\"first_positive\""));
    assert!(!json.contains("\"rotary\""));

    let rotary = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, None).expect("rotary");
    let with_rotary = Program::try_new(None, Vec::new(), None, Vec::new(), Vec::new(), Vec::new(), Some(rotary)).expect("rotary program");
    let err = with_rotary.with_five_axis(setup.clone()).expect_err("both");
    assert_eq!(err.to_string(), "invalid input: a program cannot have both a rotary setup and five-axis kinematics");
    let again = Program::try_new(None, Vec::new(), None, Vec::new(), Vec::new(), Vec::new(), None)
        .expect("bare")
        .with_five_axis(setup)
        .expect("five axis")
        .with_rotary(Some(rotary));
    let err = again.expect_err("rotary after five axis");
    assert_eq!(err.to_string(), "invalid input: a program cannot have both a rotary setup and five-axis kinematics");
}

#[test]
fn three_axis_poses_are_unchanged_when_a_setup_is_present() {
    let setup = RotarySetup::new(RotaryAxisKind::B, Vec3::ZERO, RotaryDirection::Negative, None).expect("setup");
    let pose = Pose::new(Vec3::new(1.0, 2.0, 3.0).expect("tip"), Vec3::Z).expect("pose");
    let program = Program::try_new(None, Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), Some(setup)).expect("program");
    assert!(program.toolpaths()[0].blocks().iter().any(|block| match block {
        ClBlock::Motion(ClMotion::Rapid { to, .. }) => to.rotary().is_none() && to.axis() == Vec3::Z,
        _ => false,
    }));
}

fn indexed_pose(tip: Vec3, setup: &RotarySetup, angle: f64) -> Pose {
    let expected = machine_to_part(setup, Pose::new(Vec3::ZERO, Vec3::Z).expect("z"), angle).axis();
    Pose::new(tip, expected).expect("pose").with_rotary(angle).expect("rotary")
}

fn rapid_path(pose: Pose) -> ClPath {
    ClPath::try_new(
        None,
        None,
        CompensationMode::Off,
        Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        vec![ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None })],
    )
    .expect("path")
}

fn rotary_sample() -> Document {
    let setup = RotarySetup::new(
        RotaryAxisKind::A,
        Vec3::new(0.0, 0.0, 0.0).expect("centre"),
        RotaryDirection::Positive,
        Some(RotaryLimits::new(-180.0, 180.0).expect("limits")),
    )
    .expect("setup");
    let pose = indexed_pose(Vec3::new(1.0, 2.0, 3.0).expect("tip"), &setup, 90.0);
    let program = Program::try_new(Some("indexed".into()), Vec::new(), None, vec![rapid_path(pose)], Vec::new(), Vec::new(), Some(setup))
        .expect("program")
        .with_rotary(Some(setup))
        .expect("with rotary");
    Document::new(program).expect("document")
}

fn round_trip(doc: &Document) {
    let json = doc.to_json().expect("json");
    let loaded = Document::from_json(&json).expect("load");
    assert_eq!(&loaded, doc);
    assert_eq!(loaded.to_json().expect("json"), json);
    let pretty = doc.to_json_pretty().expect("pretty");
    assert_eq!(Document::from_json(&pretty).expect("pretty load"), *doc);
}

fn axis_index(name: AxisName) -> Option<u16> {
    match name {
        AxisName::Letter(_) => None,
        AxisName::Index(index) => Some(index.get()),
    }
}

fn sample_axis(ordinal: usize, parent: Option<u8>) -> Axis {
    let kind = match ordinal {
        3 | 4 | 5 | 9 | 10 | 11 => AxisKind::Rotary,
        _ => AxisKind::Linear,
    };
    let direction = match ordinal % 3 {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    };
    let offset = if ordinal == 6 { Vec3::new(0.0, 0.0, 50.0).expect("offset") } else { Vec3::ZERO };
    let limits = match ordinal {
        0 => Some(AxisLimits::try_new(-400.0, 400.0).expect("limits")),
        3 => Some(AxisLimits::try_new(-PI, PI).expect("limits")),
        _ => None,
    };
    Axis::try_new(
        u8::try_from(ordinal).expect("id"),
        AxisName::for_ordinal(ordinal).expect("name"),
        kind,
        limits,
        KinematicNode::try_new(parent, direction, offset).expect("chain"),
    )
    .expect("axis")
}

fn thirteen_axis_sample() -> Document {
    let mut axes = Vec::new();
    for ordinal in 0..13 {
        let parent = if ordinal == 0 { None } else { Some(u8::try_from(ordinal - 1).expect("parent")) };
        axes.push(sample_axis(ordinal, parent));
    }
    let channels = vec![
        Channel::try_new(0, "primary", (0..7).collect()).expect("primary"),
        Channel::try_new(1, "secondary", (7..13).collect()).expect("secondary"),
    ];
    let machine = Machine::try_new(axes, channels).expect("machine");
    let pose = |x, y, z| Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose");
    let target = |axis, value| AxisTarget::try_new(axis, value).expect("target");
    let feeds = Feeds::try_new(1000.0, Some(600.0), None, Some(250.0), None).expect("feeds");
    let params =
        ProcessParams::new().speed(Some(18_000.0)).expect("speed").pierce_delay(0.4, true).expect("pierce").post_delay(0.2, false).expect("post");
    let mut custom = BTreeMap::new();
    custom.insert("slot".into(), ParamValue::number(1.0).expect("number"));
    custom.insert("note".into(), ParamValue::text("plate").expect("text"));
    custom.insert("ok".into(), ParamValue::bool(true));
    let blocks = vec![
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 5.0), feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: params.clone() }),
        ClBlock::Motion(ClMotion::Linear { to: pose(10.0, 0.0, 5.0), feed: 1000.0, role: MoveRole::Cut }),
        ClBlock::Motion(ClMotion::Arc {
            to: pose(10.0, 10.0, 5.0),
            plane: Plane::Xy,
            center: Vec3::new(10.0, 5.0, 5.0).expect("center"),
            sense: ArcSense::Ccw,
            feed: 1000.0,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Helix {
            to: pose(0.0, 0.0, 8.0),
            plane: Plane::Xy,
            center: Vec3::new(5.0, 5.0, 5.0).expect("center"),
            sense: ArcSense::Cw,
            sweep: PI,
            feed: 250.0,
            role: MoveRole::Plunge,
        }),
        ClBlock::Event(Event::Dwell { seconds: 0.2 }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Event(Event::Coolant { mode: CoolantMode::Assist }),
        ClBlock::Event(Event::Probe { toward: Some(pose(1.0, 2.0, 3.0)), feed: Some(100.0) }),
        ClBlock::Event(Event::Comment { text: "plate".into() }),
        ClBlock::Event(Event::Custom { name: "marker".into(), params: custom }),
        ClBlock::Event(Event::Sync { id: 1 }),
    ];
    let toolpath = ClPath::try_new(
        Some("sample".into()),
        Some(1),
        CompensationMode::Computed,
        feeds,
        Some(Vec3::new(0.0, 0.0, 5.0).expect("pierce")),
        vec![ProfileTab::try_new(12.0, 4.0, Some(2.0)).expect("tab")],
        blocks,
    )
    .expect("toolpath");
    let primary = ChannelProgram::try_new(
        0,
        vec![
            AxisBlock::Motion(AxisMotion::Rapid { targets: vec![target(0, 10.0), target(1, 0.0), target(2, 5.0)], feed: None }),
            AxisBlock::Motion(AxisMotion::Linear { targets: vec![target(0, 20.0)], feed: 1500.0 }),
            AxisBlock::Motion(AxisMotion::Arc {
                targets: vec![target(0, 20.0), target(1, 10.0)],
                plane: Plane::Xy,
                center: Vec3::new(10.0, 0.0, 0.0).expect("center"),
                sense: ArcSense::Ccw,
                feed: 800.0,
            }),
            AxisBlock::Event(Event::Sync { id: 1 }),
        ],
    )
    .expect("primary program");
    let secondary = ChannelProgram::try_new(
        1,
        vec![
            AxisBlock::Motion(AxisMotion::Rapid { targets: vec![target(7, 0.0)], feed: Some(20_000.0) }),
            AxisBlock::Motion(AxisMotion::Helix {
                targets: vec![target(8, 5.0)],
                plane: Plane::Xy,
                center: Vec3::ZERO,
                sense: ArcSense::Cw,
                sweep: PI,
                feed: 400.0,
            }),
            AxisBlock::Event(Event::ProcessOn { process: ProcessKind::Other("waterjet".into()), params: ProcessParams::new() }),
            AxisBlock::Event(Event::ProcessOff { process: ProcessKind::Other("waterjet".into()) }),
            AxisBlock::Event(Event::Sync { id: 1 }),
        ],
    )
    .expect("secondary program");
    let program = Program::try_new(
        Some("sample".into()),
        vec![Tool::try_new(1, Some("flat".into()), Some(6.0), None).expect("tool")],
        Some(machine),
        vec![toolpath],
        vec![primary, secondary],
        vec![SyncPoint::try_new(1, vec![0, 1]).expect("sync")],
        None,
    )
    .expect("program");
    Document::new(program).expect("document")
}

fn helix_toolpath() -> Toolpath {
    let point = |x, y, z| Point3::new(x, y, z).expect("point");
    Toolpath::new(vec![
        Move::Rapid { to: point(1.0, 0.0, 0.0) },
        Move::Event(ProcessEvent::On),
        Move::Arc { to: point(0.0, 1.0, 2.0), centre: point(0.0, 0.0, 0.0), ccw: true, kind: CutKind::Plunge },
        Move::Event(ProcessEvent::Off),
    ])
    .expect("path")
}

fn rect(w: f64, h: f64) -> Contour {
    let point = |x, y| Point::new(x, y).expect("point");
    let edge = |a, b| Segment::Line(Line::new(a, b).expect("line"));
    Contour::try_new(
        vec![
            edge(point(0.0, 0.0), point(w, 0.0)),
            edge(point(w, 0.0), point(w, h)),
            edge(point(w, h), point(0.0, h)),
            edge(point(0.0, h), point(0.0, 0.0)),
        ],
        true,
        &Tolerance::default(),
    )
    .expect("rect")
}
