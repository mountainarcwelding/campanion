//! Modal replay of five-axis and 3+1 rotary programs.
//!
//! Each program is posted, read back by [`audit_five`] (or [`audit`] for a 3+1 rotary), and
//! compared with the kinematics the post claims to follow: `solve_path` / `subdivide_path` for a
//! continuous toolpath, `Kinematics::choose` and the work-frame map for an index. A posted feed
//! that the post rejects on purpose (branch flip, travel limit, head rotary in work mode) is
//! accepted. Any other mismatch is a post or checker bug.

#![allow(clippy::expect_used)]

use campanion_ir::{
    Carrier, Choice, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, FiveAxisSetup, Joints, Kinematics, MoveRole, Pose, Program,
    RotaryAxisKind, RotaryDirection, RotarySetup, SolutionPolicy, Tool, ToolPose, Vec3, machine_to_part, part_to_machine, solve_path, subdivide_path,
};
use campanion_post::{
    DistanceMode, FeedMode, FiveAudit, FiveIndex, FiveMotion, FiveRetract, LengthOffset, MotionMode, PostOptions, UnitMode, audit, audit_five,
    builtin, post_program,
};
use proptest::prelude::*;
use proptest::test_runner::{RngSeed, TestCaseError};

const TABLE_TABLE_AC: &str = include_str!("../../ir/kinematics/table_table_ac.json");
const TABLE_TABLE_BC: &str = include_str!("../../ir/kinematics/table_table_bc.json");
const HEAD_TABLE_BC: &str = include_str!("../../ir/kinematics/head_table_bc.json");
const HEAD_HEAD_AC: &str = include_str!("../../ir/kinematics/head_head_ac.json");
const EXAMPLE_BC: &str = include_str!("../../../examples/table_table_bc.kin.json");
const EXAMPLE_BC_MODEL: &str = include_str!("../../../examples/table_table_bc_model.kin.json");

const CHORD: f64 = 0.005;
const AXIS_TOL: f64 = 0.1;
const RAPID_FEED: f64 = 5000.0;
const CONE: f64 = 1.0;
const POSTS: [&str; 4] = ["linuxcnc_5ax", "linuxcnc_5ax_tcp", "haas_umc", "haas_umc_tcpc"];
const ROTARY_POSTS: [&str; 4] = ["linuxcnc", "haas", "fanuc", "mach3"];

fn options() -> PostOptions {
    PostOptions { arc_tolerance: 0.05 }
}

fn kinematics(text: &str) -> Kinematics {
    Kinematics::from_json(text).expect("kinematics")
}

fn kin_files() -> Vec<(&'static str, &'static str)> {
    vec![
        ("table_table_ac", TABLE_TABLE_AC),
        ("table_table_bc", TABLE_TABLE_BC),
        ("head_table_bc", HEAD_TABLE_BC),
        ("head_head_ac", HEAD_HEAD_AC),
        ("example_bc", EXAMPLE_BC),
        ("example_bc_model", EXAMPLE_BC_MODEL),
    ]
}

fn has_head(kin: &Kinematics) -> bool {
    kin.rotaries.iter().any(|joint| joint.carrier == Carrier::Head)
}

fn machine_mode(id: &str) -> bool {
    id.starts_with("linuxcnc")
}

fn frame_coded(id: &str) -> bool {
    id.starts_with("haas_umc")
}

fn tcp_configured(id: &str) -> bool {
    id.ends_with("tcp") || id.ends_with("tcpc")
}

fn tcp_coded(id: &str) -> bool {
    id == "haas_umc_tcpc"
}

fn sets_g43(id: &str) -> bool {
    id.starts_with("haas")
}

fn work_rejected(id: &str, kin: &Kinematics, indexed: bool) -> bool {
    frame_coded(id) && has_head(kin) && indexed
}

fn quant(decimals: u8) -> f64 {
    0.5 * 10_f64.powi(-i32::from(decimals))
}

fn tip_round(linear_decimals: u8) -> f64 {
    let xyz = quant(linear_decimals) * 3_f64.sqrt();
    let ang = 0.5e-3_f64;
    xyz + 250.0 * (2.0 * ang).to_radians()
}

fn acceptable(message: &str) -> bool {
    message.contains("unreachable")
        || message.contains("leaves axis")
        || message.contains("flips the rotary branch")
        || message.contains("rotary-only")
        || message.contains("inserts more than")
        || message.contains("too large")
        || message.contains("work mode needs table rotaries only")
        || message.contains("feed rounds to zero")
        || message.contains("TCP output needs a tool number")
        || message.contains("arc is not in a cardinal plane")
        || message.contains("exceeds max_line_length")
        || message.contains("inverse time feed needs")
}

fn snippet(text: &str) -> String {
    text.lines().take(60).collect::<Vec<_>>().join("\n")
}

fn pose_at(kin: &Kinematics, angles: [f64; 2], tip: Vec3) -> Pose {
    let axis = kin.forward(&Joints { xyz: Vec3::ZERO, angles }, 0.0).expect("forward").axis;
    Pose::new(tip, axis).expect("pose")
}

fn tip_at(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z).expect("tip")
}

fn path_with(name: &str, feed: f64, tool: Option<u32>, continuous: bool, blocks: Vec<ClBlock>) -> ClPath {
    ClPath::try_new(
        Some(name.into()),
        tool,
        CompensationMode::Off,
        Feeds::try_new(feed, None, None, None, None).expect("feeds"),
        None,
        Vec::new(),
        blocks,
    )
    .expect("path")
    .with_continuous(continuous)
}

fn program_with(kin: Kinematics, tools: Vec<Tool>, paths: Vec<ClPath>) -> Program {
    let setup = FiveAxisSetup { kinematics: kin, policy: SolutionPolicy::Nearest };
    Program::try_new(Some("t".into()), tools, None, paths, Vec::new(), Vec::new(), None).expect("program").with_five_axis(setup).expect("five")
}

fn tool(id: u32) -> Tool {
    Tool::try_new(id, None, None, None).expect("tool")
}

fn rapid(pose: Pose) -> ClBlock {
    ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None })
}

fn linear(pose: Pose, feed: f64) -> ClBlock {
    ClBlock::Motion(ClMotion::Linear { to: pose, feed, role: MoveRole::Cut, power: None })
}

fn change(id: u32) -> ClBlock {
    ClBlock::Event(Event::ToolChange { tool_id: id })
}

#[test]
fn golden_five_axis_replays() {
    let cases = [
        ("linuxcnc_5ax", include_str!("golden/swarf_cylinder.linuxcnc_5ax.nc"), false, false),
        ("linuxcnc_5ax_tcp", include_str!("golden/swarf_cylinder.linuxcnc_5ax_tcp.nc"), true, false),
        ("haas_umc", include_str!("golden/swarf_cylinder.haas_umc.nc"), false, false),
        ("haas_umc_tcpc", include_str!("golden/swarf_cylinder.haas_umc_tcpc.nc"), true, true),
    ];
    for (id, text, tcp, coded) in cases {
        let post = builtin(id).expect(id);
        let five = audit_five(text, &post).unwrap_or_else(|err| panic!("{id}: {err}"));
        let mill = audit(text, &post).unwrap_or_else(|err| panic!("{id} audit: {err}"));
        assert_eq!(five.motions.len(), mill.motions.len(), "{id}");
        assert!(!five.motions.is_empty(), "{id}");
        assert_eq!(five.feed_mode, FeedMode::PerMinute, "{id}");
        assert!(!five.work_frame, "{id}");
        assert!(!five.tcp, "{id}");
        assert_eq!(five.tcp_configured, tcp, "{id}");
        assert_eq!(five.tcp_coded, coded, "{id}");
        assert_eq!(five.machine_mode, machine_mode(id), "{id}");
        assert_eq!(five.frame_coded, frame_coded(id), "{id}");
        assert!((five.chord_tol - CHORD).abs() < 1e-12, "{id}");
        assert!((five.axis_tol_deg - AXIS_TOL).abs() < 1e-12, "{id}");
        if coded {
            assert!(five.motions.iter().any(|motion| motion.tcp && motion.tcp_h == Some(1)), "{id}\n{text}");
            assert_eq!(five.length, LengthOffset::Active(1), "{id}");
            assert!(five.retracts.iter().any(|retract| retract.axes[2].is_some_and(|z| z.abs() < 1e-9)), "{id}");
        }
        if id == "linuxcnc_5ax" {
            assert!(five.retracts.is_empty(), "{id}");
            assert!(five.indexes.is_empty(), "{id}");
        }
        if id == "linuxcnc_5ax_tcp" {
            assert!(text.contains("(TCP kinematics required)"));
            assert_eq!(five.length, LengthOffset::Cancelled, "{id}");
            assert!(five.indexes.iter().any(|index| index.rotaries[1].is_some_and(|b| b.abs() < 1e-9)));
        }
    }
}

#[test]
fn haas_rotary_word_needs_a_decimal_point() {
    let post = builtin("haas_umc").expect("haas");
    let text = "%\nO00001\nN10 G21 G90 G17 G40 G49 G80\nN20 G0 X0. Y0. Z1. C90\n";
    let err = audit_five(text, &post).expect_err("decimal");
    assert!(err.to_string().contains("decimal point"), "{err}");
}

#[test]
fn inverse_time_block_needs_its_own_f() {
    let post = builtin("haas_umc").expect("haas");
    let text = "%\nO00001\nN10 G21 G90 G17 G40 G49 G80\nN20 G1 X0. Y0. Z0. F100.\nN30 G93\nN40 G1 X1. Y0. Z0.\n";
    let err = audit_five(text, &post).expect_err("feed");
    assert!(err.to_string().contains("inverse-time"), "{err}");
}

#[test]
fn simple_continuous_posts_round_trip() {
    for (label, text) in kin_files() {
        let kin = kinematics(text);
        let angles0 = [8.0_f64, 5.0];
        let angles1 = [12.0, 9.0];
        if !kin.within_limits(angles0) || !kin.within_limits(angles1) {
            continue;
        }
        let blocks = vec![rapid(pose_at(&kin, angles0, tip_at(6.0, 4.0, 10.0))), linear(pose_at(&kin, angles1, tip_at(10.0, 7.0, 12.0)), 100.0)];
        for id in POSTS {
            if work_rejected(id, &kin, false) {
                continue;
            }
            let mut blocks = blocks.clone();
            let mut tools = Vec::new();
            if tcp_configured(id) {
                blocks.insert(0, change(1));
                tools.push(tool(1));
            }
            let program = program_with(kin.clone(), tools, vec![path_with("c", 100.0, tcp_configured(id).then_some(1), true, blocks)]);
            post_strict(&program, id).unwrap_or_else(|err| panic!("{label} {id}: {err}"));
        }
    }
}

#[test]
fn indexed_then_same_axis_does_not_reindex_until_program_end() {
    for (label, text) in kin_files() {
        let kin = kinematics(text);
        if has_head(&kin) {
            continue;
        }
        let angles = [15.0, 10.0];
        if !kin.within_limits(angles) {
            continue;
        }
        let axis_pose = pose_at(&kin, angles, tip_at(4.0, 3.0, 11.0));
        let axis = axis_pose.axis();
        let second = Pose::new(tip_at(9.0, 3.0, 11.0), axis).expect("pose");
        let third = Pose::new(tip_at(9.0, 6.0, 12.0), axis).expect("pose");
        let first = path_with("a", 100.0, None, false, vec![rapid(axis_pose), linear(second, 100.0)]);
        let next = path_with("b", 180.0, None, false, vec![rapid(third)]);
        for id in POSTS {
            let program = program_with(kin.clone(), Vec::new(), vec![first.clone(), next.clone()]);
            post_strict(&program, id).unwrap_or_else(|err| panic!("{label} {id}: {err}"));
        }
    }
}

#[test]
fn continuous_then_indexed_on_the_same_axis_reindexes() {
    for (label, text) in kin_files() {
        let kin = kinematics(text);
        if has_head(&kin) {
            continue;
        }
        let a0 = [8.0, 5.0];
        let a1 = [12.0, 9.0];
        if !kin.within_limits(a0) || !kin.within_limits(a1) {
            continue;
        }
        let end = pose_at(&kin, a1, tip_at(10.0, 7.0, 12.0));
        let axis = end.axis();
        let indexed = Pose::new(tip_at(2.0, -4.0, 14.0), axis).expect("pose");
        let indexed_to = Pose::new(tip_at(6.0, -4.0, 14.0), axis).expect("pose");
        let again = pose_at(&kin, [16.0, 4.0], tip_at(-2.0, 5.0, 9.0));
        let cont = path_with("c", 100.0, None, true, vec![rapid(pose_at(&kin, a0, tip_at(6.0, 4.0, 10.0))), linear(end, 100.0)]);
        let indexed_path = path_with("i", 180.0, None, false, vec![rapid(indexed), linear(indexed_to, 180.0)]);
        let cont2 = path_with("d", 240.0, None, true, vec![rapid(again), linear(pose_at(&kin, [18.0, 6.0], tip_at(1.0, 5.0, 9.0)), 240.0)]);
        for id in POSTS {
            let mut tools = Vec::new();
            let mut paths = vec![cont.clone(), indexed_path.clone(), cont2.clone()];
            if tcp_configured(id) {
                tools.push(tool(1));
                for path in &mut paths {
                    let mut blocks = path.blocks().to_vec();
                    if path.continuous() {
                        blocks.insert(0, change(1));
                        *path = path_with(path.name().unwrap_or("c"), 100.0, Some(1), true, blocks);
                    }
                }
            }
            let program = program_with(kin.clone(), tools, paths);
            post_strict(&program, id).unwrap_or_else(|err| panic!("{label} {id}: {err}"));
        }
    }
}

#[test]
fn haas_rejects_a_head_rotary_on_an_indexed_path() {
    let kin = kinematics(HEAD_TABLE_BC);
    let pose = pose_at(&kin, [0.0, 10.0], tip_at(4.0, 3.0, 12.0));
    let program = program_with(kin, Vec::new(), vec![path_with("i", 100.0, None, false, vec![rapid(pose)])]);
    for id in ["haas_umc", "haas_umc_tcpc"] {
        let err = post_program(&program, &builtin(id).expect(id), &options()).expect_err("head");
        assert!(err.to_string().contains("work mode needs table rotaries only"), "{id} {err}");
    }
}

#[test]
fn cone_holds_the_primary_across_a_large_commanded_spin() {
    let kin = kinematics(TABLE_TABLE_AC);
    let primary = primary_index(&kin);
    let a = pose_at(&kin, [0.4, 10.0], tip_at(5.0, 2.0, 12.0));
    let b = pose_at(&kin, [0.45, 100.0], tip_at(8.0, 2.0, 12.0));
    assert!(axis_deg(a.axis(), Vec3::Z) < CONE, "generator left the cone");
    assert!(axis_deg(b.axis(), Vec3::Z) < CONE, "generator left the cone");
    let blocks = vec![rapid(a), linear(b, 100.0)];
    for id in POSTS {
        let mut owned = blocks.clone();
        let mut tools = Vec::new();
        let tool_id = tcp_configured(id).then_some(1);
        if tool_id.is_some() {
            owned.insert(0, change(1));
            tools.push(tool(1));
        }
        let program = program_with(kin.clone(), tools, vec![path_with("c", 100.0, tool_id, true, owned)]);
        let text = post_program(&program, &builtin(id).expect(id), &options()).unwrap_or_else(|err| panic!("{id}: {err}"));
        let report = audit_five(&text, &builtin(id).expect(id)).unwrap_or_else(|err| panic!("{id}: {err}\n{text}"));
        post_strict(&program, id).unwrap_or_else(|err| panic!("{id}: {err}"));
        let mut held = Vec::new();
        for motion in &report.motions {
            if !motion.rotary_written.iter().any(|wrote| *wrote) {
                continue;
            }
            let angles = posted_angles(motion, &kin).expect("angles");
            let tip = if tcp_configured(id) {
                vec3(motion.end)
            } else {
                let xyz = vec3(motion.end);
                kin.forward(&Joints { xyz, angles }, 0.0).expect("fk").tip
            };
            if gap(tip, a.tip()) < 0.05 || gap(tip, b.tip()) < 0.05 {
                held.push(angles[primary]);
            }
        }
        assert!(held.len() >= 2, "{id} vertices missing\n{text}");
        let first = held[0];
        for value in &held {
            assert!((value - first).abs() < 0.02, "{id} primary moved from {first} to {value}\n{text}");
        }
    }
}

#[test]
fn unwound_rotary_is_not_folded_into_a_turn() {
    let kin = kinematics(TABLE_TABLE_AC);
    let setup = FiveAxisSetup { kinematics: kin.clone(), policy: SolutionPolicy::Nearest };
    // A single pose whose generator is past 180° is posted on the nearer wrapped branch.
    // Several short steps, starting where +C is already nearer than the wrap, stay on that
    // branch and leave (-180, 180].
    let tilt = 25.0;
    // The other AC branch flips A and shifts C by 180°, so a start past ±90° is nearer to
    // zero on that branch. Start at 80°, where this branch is nearer, and step across 180°.
    let spins: Vec<f64> = (0..12).map(|index| 80.0 + 10.0 * f64::from(index)).collect();
    let poses: Vec<ToolPose> =
        spins.iter().enumerate().map(|(index, spin)| tool_pose(pose_at(&kin, [tilt, *spin], tip_at(6.0 + index as f64, 3.0, 11.0)))).collect();
    let solved = solve_path(&setup, &poses, Some(Joints { xyz: Vec3::ZERO, angles: [0.0, 0.0] }), CONE).expect("solve across 180");
    assert!(solved.iter().any(|joints| joints.angles.iter().any(|angle| angle.abs() > 180.0)), "solved {solved:?}");
    let blocks: Vec<ClBlock> = spins
        .iter()
        .enumerate()
        .map(|(index, spin)| {
            let pose = pose_at(&kin, [tilt, *spin], tip_at(6.0 + index as f64, 3.0, 11.0));
            if index == 0 { rapid(pose) } else { linear(pose, 100.0) }
        })
        .collect();
    for id in POSTS {
        let mut owned = blocks.clone();
        let mut tools = Vec::new();
        let tool_id = tcp_configured(id).then_some(1);
        if tool_id.is_some() {
            owned.insert(0, change(1));
            tools.push(tool(1));
        }
        let program = program_with(kin.clone(), tools, vec![path_with("c", 100.0, tool_id, true, owned)]);
        post_strict(&program, id).unwrap_or_else(|err| panic!("{id}: {err}"));
        let text = post_program(&program, &builtin(id).expect(id), &options()).expect("post");
        let report = audit_five(&text, &builtin(id).expect(id)).expect("audit");
        let unwound = report.motions.iter().any(|motion| motion.rotaries.iter().flatten().any(|angle| angle.abs() > 180.0));
        assert!(unwound, "{id} folded the rotary\n{text}");
    }
}

#[test]
fn branch_flip_is_rejected_instead_of_posted() {
    let kin = kinematics(TABLE_TABLE_BC);
    let program = find_branch_flip(&kin).expect("no branch flip on table_table_bc");
    for id in POSTS {
        if work_rejected(id, &kin, false) {
            continue;
        }
        let mut program = program.clone();
        if tcp_configured(id) {
            program = with_tool(&program, 1);
        }
        let err = post_program(&program, &builtin(id).expect(id), &options()).expect_err("flip");
        assert!(err.to_string().contains("flips the rotary branch"), "{id} {err}");
    }
}

#[test]
fn tool_change_between_tcp_and_indexed_keeps_length_and_h() {
    let kin = kinematics(TABLE_TABLE_BC);
    let a0 = [0.0, 8.0];
    let a1 = [0.0, 12.0];
    let end = pose_at(&kin, a1, tip_at(8.0, 4.0, 11.0));
    let axis = end.axis();
    let indexed = Pose::new(tip_at(1.0, 5.0, 13.0), axis).expect("pose");
    let cont = vec![change(1), rapid(pose_at(&kin, a0, tip_at(4.0, 4.0, 11.0))), linear(end, 100.0)];
    let indexed_blocks = vec![change(2), rapid(indexed), linear(Pose::new(tip_at(5.0, 5.0, 13.0), axis).expect("pose"), 180.0)];
    for id in ["haas_umc_tcpc", "linuxcnc_5ax_tcp"] {
        let paths = vec![path_with("c", 100.0, Some(1), true, cont.clone()), path_with("i", 180.0, Some(2), false, indexed_blocks.clone())];
        let program = program_with(kin.clone(), vec![tool(1), tool(2)], paths);
        post_strict(&program, id).unwrap_or_else(|err| panic!("{id}: {err}"));
    }
}

#[test]
fn rotary_3_plus_1_matches_the_machine_frame() {
    for id in ROTARY_POSTS {
        check_fixed_rotary(id).unwrap_or_else(|err| panic!("{id}: {err}"));
    }
}

fn audit_config(cases: u32) -> ProptestConfig {
    ProptestConfig { cases, max_shrink_iters: 40, rng_seed: RngSeed::Fixed(0x00C2_2A05), ..ProptestConfig::default() }
}

proptest! {
    #![proptest_config(audit_config(24))]

    #[test]
    fn random_five_axis_replays((kin_i, post_i, pattern, nums) in (0usize..6, 0usize..4, 0usize..6, prop::collection::vec(0.0f64..1.0, 18))) {
        let (label, text) = kin_files()[kin_i];
        let id = POSTS[post_i];
        let kin = kinematics(text);
        let built = build_random(&kin, id, pattern, &nums);
        let Some(program) = built else {
            return Ok(());
        };
        let indexed = program.toolpaths().iter().any(|path| !path.continuous());
        if work_rejected(id, &kin, indexed) {
            let err = post_program(&program, &builtin(id).expect(id), &options()).expect_err("head");
            prop_assert!(err.to_string().contains("work mode needs table rotaries only"), "{label} {id} {err}");
            return Ok(());
        }
        if let Err(err) = post_loose(&program, id) {
            return Err(TestCaseError::fail(format!("{label} {id}: {err}")));
        }
    }
}

proptest! {
    #![proptest_config(audit_config(16))]

    #[test]
    fn random_rotary_replays((post_i, negative, steps) in (0usize..4, any::<bool>(), prop::collection::vec(0.0f64..1.0, 12))) {
        let id = ROTARY_POSTS[post_i];
        if let Err(err) = check_random_rotary(id, negative, &steps) {
            return Err(TestCaseError::fail(format!("{id}: {err}")));
        }
    }
}

fn post_strict(program: &Program, id: &str) -> Result<(), String> {
    let post = builtin(id).ok_or_else(|| format!("missing post {id}"))?;
    let text = post_program(program, &post, &options()).map_err(|err| err.to_string())?;
    let report = audit_five(&text, &post).map_err(|err| format!("{err}\n{}", snippet(&text)))?;
    check_flags(id, &report)?;
    let kin = program.five_axis().ok_or("missing kinematics")?.kinematics.clone();
    let expected = build_expected(program, id).map_err(|err| format!("oracle: {err}\n{}", snippet(&text)))?;
    match_all(&report, &expected, &kin, &text)
}

fn post_loose(program: &Program, id: &str) -> Result<(), String> {
    let post = builtin(id).ok_or_else(|| format!("missing post {id}"))?;
    let posted = post_program(program, &post, &options());
    let built = build_expected(program, id);
    match (built, posted) {
        (Ok(expected), Ok(text)) => {
            let report = audit_five(&text, &post).map_err(|err| format!("{err}\n{}", snippet(&text)))?;
            check_flags(id, &report)?;
            let kin = program.five_axis().ok_or("missing kinematics")?.kinematics.clone();
            match_all(&report, &expected, &kin, &text)
        }
        (Err(oracle), Err(err)) => {
            let message = err.to_string();
            if acceptable(&message) || message.contains(&oracle) || oracle.contains("flips") && message.contains("flips") {
                Ok(())
            } else {
                Err(format!("post `{message}` / oracle `{oracle}`"))
            }
        }
        (Ok(_), Err(err)) => Err(format!("post rejected a reachable program: {err}")),
        (Err(oracle), Ok(text)) => Err(format!("post emitted code the kinematics reject: {oracle}\n{}", snippet(&text))),
    }
}

fn check_flags(id: &str, report: &FiveAudit) -> Result<(), String> {
    if report.machine_mode != machine_mode(id)
        || report.frame_coded != frame_coded(id)
        || report.tcp_coded != tcp_coded(id)
        || report.tcp_configured != tcp_configured(id)
    {
        return Err(format!("post flags for {id} do not match the audit"));
    }
    if (report.chord_tol - CHORD).abs() > 1e-12 || (report.axis_tol_deg - AXIS_TOL).abs() > 1e-12 {
        return Err(format!("{id} tolerances {} {}", report.chord_tol, report.axis_tol_deg));
    }
    if tcp_configured(id) && (report.rapid_feed - RAPID_FEED).abs() > 1e-9 {
        return Err(format!("{id} rapid feed {}", report.rapid_feed));
    }
    Ok(())
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Part {
    Full,
    Xy,
    Z,
}

struct MoveExp {
    posted: [f64; 3],
    partial: Part,
    angles: [f64; 2],
    write_rotaries: bool,
    rapid: bool,
    inverse: bool,
    feed: Option<f64>,
    require_feed_word: bool,
    work_frame: bool,
    tcp: bool,
    tcp_h: Option<u32>,
    length: LengthOffset,
    tool: Option<u32>,
    continuous: bool,
    tcp_section: bool,
    sample_axis: Vec3,
    axis_allow: f64,
}

struct Expectation {
    events: Vec<Exp>,
    final_length: LengthOffset,
    final_tool: Option<u32>,
}

enum Exp {
    Retract,
    Index { angles: [f64; 2] },
    Move(MoveExp),
}

struct Built {
    angles: [f64; 2],
    frame: bool,
    started: bool,
    tool_axis: Vec3,
    part_tip: Option<[f64; 3]>,
    tool: Option<u32>,
    length: LengthOffset,
    inverse: bool,
    modal_feed: Option<f64>,
    saw_feed: bool,
    events: Vec<Exp>,
}

fn build_expected(program: &Program, id: &str) -> Result<Expectation, String> {
    let setup = program.five_axis().ok_or("missing kinematics")?;
    let kin = &setup.kinematics;
    if frame_coded(id) && has_head(kin) && program.toolpaths().iter().any(|path| !path.continuous()) {
        return Err("work mode needs table rotaries only".into());
    }
    let mut built = Built {
        angles: [0.0, 0.0],
        frame: false,
        started: false,
        tool_axis: Vec3::Z,
        part_tip: None,
        tool: None,
        length: LengthOffset::Cancelled,
        inverse: false,
        modal_feed: None,
        saw_feed: false,
        events: Vec::new(),
    };
    for path in program.toolpaths() {
        emit_path(&mut built, path, kin, id)?;
    }
    if built.frame {
        built.events.push(Exp::Retract);
        built.events.push(Exp::Index { angles: [0.0, 0.0] });
        built.angles = [0.0, 0.0];
        built.frame = false;
    }
    Ok(Expectation { events: built.events, final_length: built.length, final_tool: built.tool })
}

fn emit_path(built: &mut Built, path: &ClPath, kin: &Kinematics, id: &str) -> Result<(), String> {
    let mut motions = Vec::new();
    for block in path.blocks() {
        match block {
            ClBlock::Event(Event::ToolChange { tool_id }) => {
                if !motions.is_empty() {
                    return Err("tool change after a motion".into());
                }
                built.tool = Some(*tool_id);
                if sets_g43(id) {
                    built.length = LengthOffset::Active(*tool_id);
                }
            }
            ClBlock::Event(_) => return Err("unsupported event".into()),
            ClBlock::Motion(motion) => motions.push(motion_parts(motion)?),
        }
    }
    if motions.is_empty() {
        return Ok(());
    }
    if path.continuous() { emit_continuous(built, path, kin, id, &motions) } else { emit_indexed(built, kin, id, &motions) }
}

fn emit_continuous(built: &mut Built, path: &ClPath, kin: &Kinematics, id: &str, motions: &[(Pose, bool, Option<f64>)]) -> Result<(), String> {
    let in_tcp = tcp_configured(id);
    built.frame = false;
    built.started = true;
    built.saw_feed = false;
    let setup = FiveAxisSetup { kinematics: kin.clone(), policy: SolutionPolicy::Nearest };
    let raws: Vec<Raw> = motions.iter().map(|(pose, rapid, feed)| Raw { pose: tool_pose(*pose), rapid: *rapid, feed: *feed }).collect();
    let poses: Vec<ToolPose> = raws.iter().map(|raw| raw.pose).collect();
    let start = if in_tcp { [0.0, 0.0] } else { built.angles };
    let solved =
        solve_path(&setup, &poses, Some(Joints { xyz: Vec3::ZERO, angles: start }), path.singular_cone_deg()).map_err(|err| err.to_string())?;
    let samples = if in_tcp {
        let subdivided = tcp_subdivide(kin, path.name().unwrap_or(""), &poses, &solved)?;
        attach(&subdivided, &raws)?
    } else {
        let (sub_poses, sub_joints) =
            subdivide_path(&setup, path.name().unwrap_or(""), &poses, &solved, CHORD, AXIS_TOL).map_err(|err| err.to_string())?;
        let subdivided: Vec<(ToolPose, Joints)> = sub_poses.into_iter().zip(sub_joints).collect();
        attach(&subdivided, &raws)?
    };
    if in_tcp {
        let tip = poses[0].tip;
        built.events.push(Exp::Retract);
        built.angles = [0.0, 0.0];
        built.tool_axis = Vec3::Z;
        built.events.push(Exp::Index { angles: [0.0, 0.0] });
        if tcp_coded(id) {
            built.length = LengthOffset::Cancelled;
        }
        let target = [tip.x(), tip.y(), tip.z()];
        push_approach(built, target, id, true);
        built.part_tip = Some(target);
    }
    for sample in &samples {
        emit_sample(built, sample, id, in_tcp, path.singular_cone_deg())?;
    }
    if built.saw_feed {
        built.inverse = false;
        built.modal_feed = None;
        built.saw_feed = false;
    }
    if in_tcp && tcp_coded(id) {
        built.length = match built.tool {
            Some(tool_id) => LengthOffset::Active(tool_id),
            None => LengthOffset::Cancelled,
        };
    }
    Ok(())
}

fn emit_indexed(built: &mut Built, kin: &Kinematics, id: &str, motions: &[(Pose, bool, Option<f64>)]) -> Result<(), String> {
    let axis = motions[0].0.axis();
    let reindex = !built.frame || !built.started || axis_apart(axis, built.tool_axis);
    if reindex {
        let tool_pose = ToolPose { tip: motions[0].0.tip(), axis };
        let joints = kin.choose(&tool_pose, 0.0, Choice::Nearest(built.angles)).map_err(|err| err.to_string())?;
        built.events.push(Exp::Retract);
        built.events.push(Exp::Index { angles: joints.angles });
        built.angles = joints.angles;
        built.tool_axis = axis;
        built.frame = true;
        built.started = true;
        limits_ok(kin, built.angles)?;
    } else if axis_apart(axis, built.tool_axis) {
        return Err("tool axis changed inside an index".into());
    }
    let mut first = true;
    for (pose, rapid, feed) in motions {
        if axis_apart(pose.axis(), built.tool_axis) {
            return Err("tool axis changed inside an index".into());
        }
        let posted = frame_xyz(kin, id, built.angles, built.tool_axis, pose.tip())?;
        if first && reindex {
            push_approach(built, posted, id, false);
            built.part_tip = Some([pose.tip().x(), pose.tip().y(), pose.tip().z()]);
            first = false;
            continue;
        }
        first = false;
        let inverse = false;
        let (feed_value, require) = if *rapid { (None, false) } else { per_minute(built, feed.expect("feed")) };
        built.events.push(Exp::Move(MoveExp {
            posted,
            partial: Part::Full,
            angles: built.angles,
            write_rotaries: false,
            rapid: *rapid,
            inverse,
            feed: feed_value,
            require_feed_word: require,
            work_frame: built.frame && frame_coded(id),
            tcp: false,
            tcp_h: None,
            length: built.length,
            tool: built.tool,
            continuous: false,
            tcp_section: false,
            sample_axis: pose.axis(),
            axis_allow: 0.05,
        }));
        built.part_tip = Some([pose.tip().x(), pose.tip().y(), pose.tip().z()]);
        let _ = inverse;
    }
    Ok(())
}

fn emit_sample(built: &mut Built, sample: &Sample, id: &str, in_tcp: bool, cone: f64) -> Result<(), String> {
    let part = [sample.pose.tip.x(), sample.pose.tip.y(), sample.pose.tip.z()];
    let length = built.part_tip.map(|start| dist3(start, part)).unwrap_or(0.0);
    let da0 = (sample.joints.angles[0] - built.angles[0]).abs();
    let da1 = (sample.joints.angles[1] - built.angles[1]).abs();
    let angle = da0.hypot(da1);
    let zero = length <= 1e-6 && angle <= 1e-9;
    if zero && (!sample.rapid || in_tcp) {
        built.part_tip = Some(part);
        built.angles = sample.joints.angles;
        built.tool_axis = sample.pose.axis;
        return Ok(());
    }
    let rotary_rapid = sample.rapid && angle > 1e-9;
    let write_g0 = sample.rapid && !(in_tcp && rotary_rapid);
    let mut feed = None;
    let mut require = false;
    if !write_g0 {
        built.inverse = true;
        built.saw_feed = true;
        let programmed = if rotary_rapid { RAPID_FEED } else { sample.feed.ok_or("feed missing")? };
        if length <= 1e-6 && angle <= 1e-6 {
            return Err("rotary-only move needs a feed".into());
        }
        let value = if length > 1e-6 { programmed / length } else { programmed / angle };
        feed = Some(value);
        require = true;
        built.modal_feed = Some(value);
    }
    let posted = if in_tcp { part } else { [sample.joints.xyz.x(), sample.joints.xyz.y(), sample.joints.xyz.z()] };
    let allow = if sample.original { cone + 0.05 } else { 0.05 };
    built.events.push(Exp::Move(MoveExp {
        posted,
        partial: Part::Full,
        angles: sample.joints.angles,
        write_rotaries: !(write_g0 && in_tcp),
        rapid: write_g0,
        inverse: built.inverse,
        feed,
        require_feed_word: require,
        work_frame: false,
        tcp: in_tcp && tcp_coded(id),
        tcp_h: (in_tcp && tcp_coded(id)).then_some(built.tool).flatten(),
        length: built.length,
        tool: built.tool,
        continuous: true,
        tcp_section: in_tcp,
        sample_axis: sample.pose.axis,
        axis_allow: allow,
    }));
    built.part_tip = Some(part);
    built.angles = sample.joints.angles;
    built.tool_axis = sample.pose.axis;
    Ok(())
}

fn push_approach(built: &mut Built, target: [f64; 3], id: &str, tcp_section: bool) {
    let tcp = tcp_section && tcp_coded(id);
    let tcp_h = tcp.then_some(built.tool).flatten();
    let work_frame = built.frame && frame_coded(id) && !tcp_section;
    for partial in [Part::Xy, Part::Z] {
        built.events.push(Exp::Move(MoveExp {
            posted: target,
            partial,
            angles: built.angles,
            write_rotaries: false,
            rapid: true,
            inverse: built.inverse,
            feed: None,
            require_feed_word: false,
            work_frame,
            tcp,
            tcp_h,
            length: built.length,
            tool: built.tool,
            continuous: false,
            tcp_section,
            sample_axis: built.tool_axis,
            axis_allow: 0.05,
        }));
    }
}

fn per_minute(built: &mut Built, feed: f64) -> (Option<f64>, bool) {
    built.inverse = false;
    let require = match built.modal_feed {
        Some(prev) => (prev - feed).abs() > 1e-6,
        None => true,
    };
    if require {
        built.modal_feed = Some(feed);
    }
    (Some(feed), require)
}

fn frame_xyz(kin: &Kinematics, id: &str, angles: [f64; 2], axis: Vec3, tip: Vec3) -> Result<[f64; 3], String> {
    if machine_mode(id) {
        let joints = kin.choose(&ToolPose { tip, axis }, 0.0, Choice::Nearest(angles)).map_err(|err| err.to_string())?;
        Ok([joints.xyz.x(), joints.xyz.y(), joints.xyz.z()])
    } else {
        let point = table_map(kin, angles, tip);
        Ok([point.x(), point.y(), point.z()])
    }
}

fn limits_ok(kin: &Kinematics, angles: [f64; 2]) -> Result<(), String> {
    for (index, joint) in kin.rotaries.iter().enumerate() {
        if let Some(limits) = joint.limits {
            let angle = angles[index];
            if angle < limits.min_deg() - 1e-6 || angle > limits.max_deg() + 1e-6 {
                return Err(format!("angle {angle} leaves {} limits", joint.letter));
            }
        }
    }
    Ok(())
}

struct Raw {
    pose: ToolPose,
    rapid: bool,
    feed: Option<f64>,
}

struct Sample {
    pose: ToolPose,
    joints: Joints,
    rapid: bool,
    feed: Option<f64>,
    original: bool,
}

fn attach(subs: &[(ToolPose, Joints)], raws: &[Raw]) -> Result<Vec<Sample>, String> {
    let mut out = Vec::new();
    let mut sub_i = 0;
    for raw in raws {
        while sub_i < subs.len() && subs[sub_i].0 != raw.pose {
            out.push(Sample { pose: subs[sub_i].0, joints: subs[sub_i].1, rapid: raw.rapid, feed: raw.feed, original: false });
            sub_i += 1;
        }
        if sub_i >= subs.len() || subs[sub_i].0 != raw.pose {
            return Err("continuous subdivision lost a pose".into());
        }
        out.push(Sample { pose: subs[sub_i].0, joints: subs[sub_i].1, rapid: raw.rapid, feed: raw.feed, original: true });
        sub_i += 1;
    }
    if sub_i != subs.len() {
        return Err("continuous subdivision lost a pose".into());
    }
    Ok(out)
}

fn tcp_subdivide(kin: &Kinematics, name: &str, poses: &[ToolPose], joints: &[Joints]) -> Result<Vec<(ToolPose, Joints)>, String> {
    if poses.len() != joints.len() {
        return Err(format!("toolpath `{name}` poses and joints differ"));
    }
    let (Some((&first_pose, rest_poses)), Some((&first_joints, rest_joints))) = (poses.split_first(), joints.split_first()) else {
        return Ok(Vec::new());
    };
    let mut out = vec![(first_pose, first_joints)];
    let mut prev_pose = first_pose;
    let mut prev_joints = first_joints;
    let mut inserted = 0usize;
    for (&pose, &end_joints) in rest_poses.iter().zip(rest_joints) {
        let mut stack = vec![(prev_pose, prev_joints, pose, end_joints)];
        while let Some((left_pose, left_joints, right_pose, right_joints)) = stack.pop() {
            if tcp_segment_ok(kin, left_pose, left_joints, right_pose, right_joints)? {
                out.push((right_pose, right_joints));
                continue;
            }
            let mid_angles = lerp2(left_joints.angles, right_joints.angles);
            if angles_close(mid_angles, left_joints.angles) || angles_close(mid_angles, right_joints.angles) {
                out.push((right_pose, right_joints));
                continue;
            }
            inserted += 1;
            if inserted > 10_000 {
                return Err(format!("toolpath `{name}` inserts more than 10000 poses"));
            }
            let mid_tip = lerp_vec(left_pose.tip, right_pose.tip);
            let mid_axis = tcp_fk_axis(kin, mid_angles)?;
            let mid_pose = ToolPose { tip: mid_tip, axis: mid_axis };
            let mid_joints = Joints { xyz: mid_tip, angles: mid_angles };
            stack.push((mid_pose, mid_joints, right_pose, right_joints));
            stack.push((left_pose, left_joints, mid_pose, mid_joints));
        }
        prev_pose = pose;
        prev_joints = end_joints;
    }
    Ok(out)
}

fn tcp_segment_ok(kin: &Kinematics, left_pose: ToolPose, left_joints: Joints, right_pose: ToolPose, right_joints: Joints) -> Result<bool, String> {
    if angles_close(left_joints.angles, right_joints.angles) {
        return Ok(true);
    }
    let left_axis = tcp_fk_axis(kin, left_joints.angles)?;
    let right_axis = tcp_fk_axis(kin, right_joints.angles)?;
    let mid_axis = tcp_fk_axis(kin, lerp2(left_joints.angles, right_joints.angles))?;
    let desired = slerp_axis(left_axis, right_axis, 0.5)?;
    let axis_gap = axis_deg(mid_axis, desired);
    let control_tip = lerp_vec(left_pose.tip, right_pose.tip);
    let chord_mid = lerp_vec(left_pose.tip, right_pose.tip);
    Ok(gap(control_tip, chord_mid) <= CHORD && axis_gap <= AXIS_TOL)
}

fn tcp_fk_axis(kin: &Kinematics, angles: [f64; 2]) -> Result<Vec3, String> {
    kin.forward(&Joints { xyz: Vec3::ZERO, angles }, 0.0).map(|pose| pose.axis).map_err(|err| err.to_string())
}

enum Got<'a> {
    Retract(&'a FiveRetract),
    Index(&'a FiveIndex),
    Move(&'a FiveMotion),
}

fn match_all(report: &FiveAudit, expected: &Expectation, kin: &Kinematics, text: &str) -> Result<(), String> {
    let mut got = Vec::new();
    for retract in &report.retracts {
        got.push((retract.line, Got::Retract(retract)));
    }
    for index in &report.indexes {
        got.push((index.line, Got::Index(index)));
    }
    for motion in &report.motions {
        got.push((motion.line, Got::Move(motion)));
    }
    got.sort_by_key(|(line, _)| *line);
    if got.len() != expected.events.len() {
        return Err(format!("audit has {} events, oracle {} \n{}", got.len(), expected.events.len(), snippet(text)));
    }
    let mut modal = [0.0; 3];
    let mut prev_continuous: Option<&FiveMotion> = None;
    let mut prev_tcp = false;
    for ((line, item), exp) in got.iter().zip(&expected.events) {
        match (item, exp) {
            (Got::Retract(retract), Exp::Retract) => check_retract(retract, report.linear_decimals)?,
            (Got::Index(index), Exp::Index { angles }) => {
                check_index(index, kin, *angles, report.rotary_decimals)?;
                prev_continuous = None;
            }
            (Got::Move(motion), Exp::Move(expect)) => {
                check_move(motion, expect, kin, report, &mut modal)?;
                if expect.continuous && expect.partial == Part::Full {
                    if let Some(prev) = prev_continuous
                        && prev_tcp == expect.tcp_section
                    {
                        check_chord(kin, prev, motion, expect.tcp_section, report)?;
                    }
                    prev_continuous = Some(motion);
                    prev_tcp = expect.tcp_section;
                } else {
                    prev_continuous = None;
                }
            }
            _ => return Err(format!("line {line}: event kind does not match the oracle\n{}", snippet(text))),
        }
    }
    if report.feed_mode != FeedMode::PerMinute {
        return Err(format!("feed mode left inverse time on\n{}", snippet(text)));
    }
    if report.work_frame || report.tcp {
        return Err(format!("frame or TCP left on at program end\n{}", snippet(text)));
    }
    if report.length != expected.final_length || report.tool != expected.final_tool {
        return Err(format!(
            "final length {:?} tool {:?} expected {:?} {:?}\n{}",
            report.length,
            report.tool,
            expected.final_length,
            expected.final_tool,
            snippet(text)
        ));
    }
    Ok(())
}

fn check_retract(retract: &FiveRetract, decimals: u8) -> Result<(), String> {
    let Some(z) = retract.axes[2] else {
        return Err(format!("line {}: retract has no Z", retract.line));
    };
    if z.abs() > quant(decimals) + 1e-9 {
        return Err(format!("line {}: retract Z {z}", retract.line));
    }
    Ok(())
}

fn check_index(index: &FiveIndex, kin: &Kinematics, angles: [f64; 2], decimals: u8) -> Result<(), String> {
    if index.work_frame || index.tcp {
        return Err(format!("line {}: index is inside the work frame or TCP", index.line));
    }
    let posted = posted_index(index, kin)?;
    for (got, expect) in posted.iter().zip(angles) {
        if (got - expect).abs() > quant(decimals) + 1e-9 {
            return Err(format!("line {}: index angle {got} != {expect}", index.line));
        }
    }
    limits_ok(kin, posted)?;
    Ok(())
}

fn check_move(motion: &FiveMotion, expect: &MoveExp, kin: &Kinematics, report: &FiveAudit, modal: &mut [f64; 3]) -> Result<(), String> {
    if motion.distance != DistanceMode::Absolute || motion.units != UnitMode::Millimetre || motion.work.is_some() {
        return Err(format!("line {}: modal setup", motion.line));
    }
    if motion.work_frame != expect.work_frame || motion.tcp != expect.tcp || motion.tcp_h != expect.tcp_h {
        return Err(format!(
            "line {}: frame {} tcp {} h {:?} expected {} {} {:?}",
            motion.line, motion.work_frame, motion.tcp, motion.tcp_h, expect.work_frame, expect.tcp, expect.tcp_h
        ));
    }
    if motion.length != expect.length || motion.tool != expect.tool {
        return Err(format!(
            "line {}: length {:?} tool {:?} expected {:?} {:?}",
            motion.line, motion.length, motion.tool, expect.length, expect.tool
        ));
    }
    if motion.spindle.on {
        return Err(format!("line {}: spindle is on", motion.line));
    }
    let rapid = motion.mode == MotionMode::Rapid;
    if rapid != expect.rapid {
        return Err(format!("line {}: rapid {rapid}", motion.line));
    }
    let inverse = motion.feed_mode == FeedMode::InverseTime;
    if inverse != expect.inverse {
        return Err(format!("line {}: feed mode inverse {inverse}", motion.line));
    }
    match expect.partial {
        Part::Full => {
            if motion.axes_written != [true, true, true] {
                return Err(format!("line {}: partial axes {:?}", motion.line, motion.axes_written));
            }
            near_xyz(motion.line, motion.end, expect.posted, report.linear_decimals)?;
        }
        Part::Xy => {
            if motion.axes_written != [true, true, false] {
                return Err(format!("line {}: XY approach {:?}", motion.line, motion.axes_written));
            }
            if (motion.end[0] - expect.posted[0]).abs() > quant(report.linear_decimals) + 1e-9
                || (motion.end[1] - expect.posted[1]).abs() > quant(report.linear_decimals) + 1e-9
            {
                return Err(format!("line {}: XY {:?} != {:?}", motion.line, motion.end, expect.posted));
            }
            if (motion.end[2] - modal[2]).abs() > 1e-9 {
                return Err(format!("line {}: XY rapid changed Z", motion.line));
            }
        }
        Part::Z => {
            if motion.axes_written != [false, false, true] {
                return Err(format!("line {}: Z approach {:?}", motion.line, motion.axes_written));
            }
            if (motion.end[2] - expect.posted[2]).abs() > quant(report.linear_decimals) + 1e-9 {
                return Err(format!("line {}: Z {} != {}", motion.line, motion.end[2], expect.posted[2]));
            }
        }
    }
    let posted = posted_angles(motion, kin)?;
    for (index, (got, expect_angle)) in posted.iter().zip(expect.angles).enumerate() {
        if (got - expect_angle).abs() > quant(report.rotary_decimals) + 1e-9 {
            return Err(format!("line {}: rotary {index} {got} != {expect_angle}", motion.line));
        }
    }
    for (index, joint) in kin.rotaries.iter().enumerate() {
        let slot = letter_slot(joint.letter)?;
        if motion.rotary_written[slot] != expect.write_rotaries {
            return Err(format!("line {}: rotary word {} written {}", motion.line, joint.letter, motion.rotary_written[slot]));
        }
        let _ = index;
    }
    if expect.tcp && expect.rapid && motion.rotary_written.iter().any(|wrote| *wrote) {
        return Err(format!("line {}: TCP rapid writes a rotary", motion.line));
    }
    limits_ok(kin, posted)?;
    if !expect.rapid {
        let Some(feed) = expect.feed else {
            return Err(format!("line {}: feed missing from the oracle", motion.line));
        };
        let Some(got) = motion.feed else {
            return Err(format!("line {}: feed missing", motion.line));
        };
        if (got - feed).abs() > quant(report.feed_decimals) + 1e-6 {
            return Err(format!("line {}: F {got} != {feed}", motion.line));
        }
        if expect.require_feed_word && !motion.feed_on_block {
            return Err(format!("line {}: F was left modal", motion.line));
        }
    }
    if expect.partial == Part::Full {
        let achieved = kin.forward(&Joints { xyz: Vec3::ZERO, angles: expect.angles }, 0.0).map_err(|err| err.to_string())?.axis;
        let gap = axis_deg(achieved, expect.sample_axis);
        if gap > expect.axis_allow {
            return Err(format!("line {}: axis {gap} > {}", motion.line, expect.axis_allow));
        }
    }
    *modal = motion.end;
    Ok(())
}

fn check_chord(kin: &Kinematics, prev: &FiveMotion, next: &FiveMotion, tcp: bool, report: &FiveAudit) -> Result<(), String> {
    let ang_a = posted_angles(prev, kin)?;
    let ang_b = posted_angles(next, kin)?;
    let mid_angles = lerp2(ang_a, ang_b);
    if tcp {
        let mid_axis = tcp_fk_axis(kin, mid_angles)?;
        let left = tcp_fk_axis(kin, ang_a)?;
        let right = tcp_fk_axis(kin, ang_b)?;
        let desired = slerp_axis(left, right, 0.5)?;
        let gap = axis_deg(mid_axis, desired);
        if gap > report.axis_tol_deg + 0.02 {
            return Err(format!("line {}: TCP midpoint axis {gap}", next.line));
        }
        return Ok(());
    }
    let xyz_a = vec3(prev.end);
    let xyz_b = vec3(next.end);
    let mid_xyz = lerp_vec(xyz_a, xyz_b);
    let mid = kin.forward(&Joints { xyz: mid_xyz, angles: mid_angles }, 0.0).map_err(|err| err.to_string())?;
    let end_a = kin.forward(&Joints { xyz: xyz_a, angles: ang_a }, 0.0).map_err(|err| err.to_string())?;
    let end_b = kin.forward(&Joints { xyz: xyz_b, angles: ang_b }, 0.0).map_err(|err| err.to_string())?;
    let chord_mid = lerp_vec(end_a.tip, end_b.tip);
    let tip_gap = gap(mid.tip, chord_mid);
    let allow = 2.0 * report.chord_tol + tip_round(report.linear_decimals);
    if tip_gap > allow {
        return Err(format!("line {}: chord {tip_gap} > {allow}", next.line));
    }
    let desired = slerp_axis(end_a.axis, end_b.axis, 0.5)?;
    let axis_gap = axis_deg(mid.axis, desired);
    if axis_gap > report.axis_tol_deg + 0.02 {
        return Err(format!("line {}: midpoint axis {axis_gap}", next.line));
    }
    Ok(())
}

fn near_xyz(line: usize, got: [f64; 3], expect: [f64; 3], decimals: u8) -> Result<(), String> {
    let q = quant(decimals) + 1e-9;
    for (index, (left, right)) in got.iter().zip(expect).enumerate() {
        if (left - right).abs() > q {
            return Err(format!("line {line}: axis {index} {left} != {right}"));
        }
    }
    Ok(())
}

fn posted_angles(motion: &FiveMotion, kin: &Kinematics) -> Result<[f64; 2], String> {
    let mut angles = [0.0; 2];
    for (index, joint) in kin.rotaries.iter().enumerate() {
        let slot = letter_slot(joint.letter)?;
        angles[index] = motion.rotaries[slot].ok_or_else(|| format!("line {}: missing {}", motion.line, joint.letter))?;
    }
    Ok(angles)
}

fn posted_index(index: &FiveIndex, kin: &Kinematics) -> Result<[f64; 2], String> {
    let mut angles = [0.0; 2];
    for (index_i, joint) in kin.rotaries.iter().enumerate() {
        let slot = letter_slot(joint.letter)?;
        if !index.written[slot] {
            return Err(format!("line {}: index omits {}", index.line, joint.letter));
        }
        angles[index_i] = index.rotaries[slot].ok_or_else(|| format!("line {}: missing {}", index.line, joint.letter))?;
    }
    Ok(angles)
}

fn letter_slot(letter: char) -> Result<usize, String> {
    match letter {
        'A' => Ok(0),
        'B' => Ok(1),
        'C' => Ok(2),
        _ => Err(format!("rotary letter {letter}")),
    }
}

fn motion_parts(motion: &ClMotion) -> Result<(Pose, bool, Option<f64>), String> {
    match motion {
        ClMotion::Rapid { to, .. } => Ok((*to, true, None)),
        ClMotion::Linear { to, feed, .. } => Ok((*to, false, Some(*feed))),
        _ => Err("oracle covers linear moves".into()),
    }
}

fn tool_pose(pose: Pose) -> ToolPose {
    ToolPose { tip: pose.tip(), axis: pose.axis() }
}

fn primary_index(kin: &Kinematics) -> usize {
    let spindle = kin.spindle_axis;
    kin.rotaries
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| dot(left.axis, spindle).abs().total_cmp(&dot(right.axis, spindle).abs()))
        .map(|(index, _)| index)
        .unwrap_or(0)
}

fn find_branch_flip(kin: &Kinematics) -> Option<Program> {
    let setup = FiveAxisSetup { kinematics: kin.clone(), policy: SolutionPolicy::Nearest };
    for tilt in [-80.0, -50.0, -40.0, -20.0, 0.0, 20.0, 40.0, 80.0, 100.0, 130.0, 150.0, 170.0] {
        for spin in [-170.0, -120.0, -60.0, -20.0, 0.0, 20.0, 60.0, 90.0, 140.0, 170.0] {
            let angles = [tilt, spin];
            let Ok(posed) = kin.forward(&Joints { xyz: Vec3::ZERO, angles }, 0.0) else {
                continue;
            };
            let tip = tip_at(5.0, 4.0, 12.0);
            let pose = Pose::new(tip, posed.axis).ok()?;
            let next = Pose::new(tip_at(8.0, 4.0, 12.0), posed.axis).ok()?;
            let tools = [tool_pose(pose), tool_pose(next)];
            let Err(err) = solve_path(&setup, &tools, Some(Joints { xyz: Vec3::ZERO, angles: [0.0, 0.0] }), CONE) else {
                continue;
            };
            if err.to_string().contains("flips the rotary branch") {
                let blocks = vec![rapid(pose), linear(next, 100.0)];
                return Some(program_with(kin.clone(), Vec::new(), vec![path_with("c", 100.0, None, true, blocks)]));
            }
        }
    }
    None
}

fn with_tool(program: &Program, id: u32) -> Program {
    let kin = program.five_axis().expect("kin").kinematics.clone();
    let mut paths = Vec::new();
    for path in program.toolpaths() {
        let mut blocks = path.blocks().to_vec();
        blocks.insert(0, change(id));
        paths.push(path_with(path.name().unwrap_or("c"), 100.0, Some(id), path.continuous(), blocks));
    }
    program_with(kin, vec![tool(id)], paths)
}

fn build_random(kin: &Kinematics, id: &str, pattern: usize, nums: &[f64]) -> Option<Program> {
    let tilt_i = tilt_index(kin);
    let spin_i = primary_index(kin);
    let near = pattern == 5;
    let mut angles = Vec::new();
    let mut cursor = [0.0, 0.0];
    for step in 0..4 {
        let u = nums.get(step).copied().unwrap_or(0.3);
        let v = nums.get(step + 4).copied().unwrap_or(0.3);
        if near {
            cursor[tilt_i] = 0.35 + 0.4 * u;
            cursor[spin_i] += -6.0 + 12.0 * v;
        } else {
            cursor[tilt_i] = stepped(cursor[tilt_i], &kin.rotaries[tilt_i], -4.0 + 8.0 * u);
            cursor[spin_i] = stepped(cursor[spin_i], &kin.rotaries[spin_i], -4.0 + 8.0 * v);
        }
        if !kin.within_limits(cursor) {
            return None;
        }
        angles.push(cursor);
    }
    let tips = [
        tip_at(-8.0 + 10.0 * nums.first().copied().unwrap_or(0.2), -6.0, 10.0),
        tip_at(-4.0, -2.0 + 8.0 * nums.get(1).copied().unwrap_or(0.4), 11.0),
        tip_at(2.0, 3.0, 12.0),
        tip_at(7.0, 5.0, 13.0),
    ];
    let poses: Vec<Pose> = angles.iter().zip(tips).map(|(angles, tip)| pose_at(kin, *angles, tip)).collect();
    let other = pose_at(kin, [angles[0][0] + 12.0, angles[0][1]], tip_at(-3.0, 4.0, 15.0));
    if !kin.within_limits([angles[0][0] + 12.0, angles[0][1]]) && pattern == 2 {
        return None;
    }
    let tcp = tcp_configured(id);
    let want_first = tcp || pattern % 2 == 1;
    let want_second = pattern == 1 || pattern == 2;
    let mut tools = Vec::new();
    if want_first {
        tools.push(tool(1));
    }
    if want_second {
        tools.push(tool(2));
    }
    let tool_for = |which: u32| match which {
        1 if want_first => Some(1),
        2 if want_second => Some(2),
        _ => None,
    };
    let lead = |id_opt: Option<u32>, blocks: &mut Vec<ClBlock>| {
        if let Some(tool_id) = id_opt {
            blocks.insert(0, change(tool_id));
        }
    };
    let cont = |feed: f64, from: usize, tool_id: Option<u32>| {
        let mut blocks = vec![rapid(poses[from]), linear(poses[from + 1], feed)];
        lead(tool_id, &mut blocks);
        path_with("c", feed, tool_id, true, blocks)
    };
    let indexed = |feed: f64, pose: Pose, tool_id: Option<u32>| {
        let to = Pose::new(tip_at(pose.tip().x() + 3.0, pose.tip().y(), pose.tip().z()), pose.axis()).expect("pose");
        let mut blocks = vec![rapid(pose), linear(to, feed)];
        lead(tool_id, &mut blocks);
        path_with("i", feed, tool_id, false, blocks)
    };
    let paths = match pattern {
        0 | 5 => vec![cont(100.0, 0, tool_for(1))],
        1 => vec![cont(100.0, 0, tool_for(1)), indexed(180.0, poses[1], tool_for(2)), cont(240.0, 2, tool_for(1))],
        2 => vec![cont(100.0, 0, tool_for(1)), indexed(180.0, other, tool_for(2)), cont(240.0, 2, tool_for(1))],
        3 => {
            vec![indexed(100.0, poses[0], tool_for(1)), indexed(180.0, Pose::new(tip_at(-1.0, 2.0, 16.0), poses[0].axis()).expect("p"), tool_for(1))]
        }
        _ => vec![indexed(100.0, poses[0], tool_for(1)), indexed(180.0, poses[2], tool_for(1))],
    };
    Some(program_with(kin.clone(), tools, paths))
}

fn tilt_index(kin: &Kinematics) -> usize {
    let primary = primary_index(kin);
    if primary == 0 { 1 } else { 0 }
}

fn stepped(prev: f64, joint: &campanion_ir::RotaryJoint, delta: f64) -> f64 {
    let next = prev + delta;
    if let Some(limits) = joint.limits {
        let lo = limits.min_deg() + 1.0;
        let hi = limits.max_deg() - 1.0;
        if hi > lo { next.clamp(lo, hi) } else { (limits.min_deg() + limits.max_deg()) * 0.5 }
    } else {
        next.clamp(-40.0, 40.0)
    }
}

struct RotaryStep {
    tip: Vec3,
    angle: f64,
    feed: Option<f64>,
}

fn check_fixed_rotary(id: &str) -> Result<(), String> {
    let setup = RotarySetup::new(RotaryAxisKind::A, tip_at(0.0, 0.0, 25.0), RotaryDirection::Positive, None).map_err(|err| err.to_string())?;
    let mut tip = tip_at(10.0, 0.0, 15.0);
    let mut steps = vec![RotaryStep { tip, angle: 0.0, feed: None }];
    let machine = part_to_machine(&setup, turned(&setup, tip, 0.0), 0.0).tip();
    let held = machine_to_part(&setup, Pose::new(machine, Vec3::Z).expect("m").with_rotary(20.0).expect("a"), 20.0).tip();
    steps.push(RotaryStep { tip: held, angle: 20.0, feed: None });
    tip = held;
    tip = shift(tip, 8.0, 0.0, 0.0);
    steps.push(RotaryStep { tip, angle: 20.0, feed: Some(120.0) });
    tip = shift(tip, 6.0, 0.0, 0.0);
    steps.push(RotaryStep { tip, angle: 200.0, feed: Some(120.0) });
    tip = shift(tip, 0.0, 4.0, 0.0);
    steps.push(RotaryStep { tip, angle: 200.0, feed: Some(80.0) });
    tip = shift(tip, 0.0, 3.0, 0.0);
    steps.push(RotaryStep { tip, angle: 200.0, feed: Some(80.0) });
    tip = shift(tip, 0.0, 0.0, 5.0);
    steps.push(RotaryStep { tip, angle: 400.0, feed: Some(80.0) });
    check_rotary(id, &setup, &steps)
}

fn check_random_rotary(id: &str, negative: bool, nums: &[f64]) -> Result<(), String> {
    let direction = if negative { RotaryDirection::Negative } else { RotaryDirection::Positive };
    let center = tip_at(-5.0 + 10.0 * nums.first().copied().unwrap_or(0.2), 4.0, 20.0);
    let setup = RotarySetup::new(RotaryAxisKind::A, center, direction, None).map_err(|err| err.to_string())?;
    let mut steps = Vec::new();
    let mut tip = tip_at(8.0, 1.0, 12.0);
    let mut angle = -20.0 + 40.0 * nums.get(1).copied().unwrap_or(0.5);
    steps.push(RotaryStep { tip, angle, feed: None });
    for index in 0..3 {
        let feed = if nums.get(2 + index).copied().unwrap_or(0.0) > 0.45 { Some(80.0 + 40.0 * f64::from(index as u32)) } else { None };
        angle += -30.0 + 80.0 * nums.get(6 + index).copied().unwrap_or(0.5);
        if feed.is_some() {
            tip = shift(tip, 2.0 + f64::from(index as u32), 0.0, 0.0);
        } else if index == 1 {
            let machine = part_to_machine(&setup, turned(&setup, tip, steps.last().expect("prev").angle), steps.last().expect("prev").angle).tip();
            tip = machine_to_part(&setup, Pose::new(machine, Vec3::Z).expect("m").with_rotary(angle).expect("a"), angle).tip();
        }
        steps.push(RotaryStep { tip, angle, feed });
    }
    check_rotary(id, &setup, &steps)
}

fn check_rotary(id: &str, setup: &RotarySetup, steps: &[RotaryStep]) -> Result<(), String> {
    let post = builtin(id).ok_or_else(|| format!("missing post {id}"))?;
    let mut blocks = Vec::new();
    for step in steps {
        let pose = turned(setup, step.tip, step.angle);
        let motion = match step.feed {
            None => ClMotion::Rapid { to: pose, feed: None },
            Some(feed) => ClMotion::Linear { to: pose, feed, role: MoveRole::Cut, power: None },
        };
        blocks.push(ClBlock::Motion(motion));
    }
    let path = ClPath::try_new(
        Some("r".into()),
        None,
        CompensationMode::Off,
        Feeds::try_new(100.0, None, None, None, None).map_err(|err| err.to_string())?,
        None,
        Vec::new(),
        blocks,
    )
    .map_err(|err| err.to_string())?;
    let program =
        Program::try_new(Some("r".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), Some(*setup)).map_err(|err| err.to_string())?;
    let text = post_program(&program, &post, &options()).map_err(|err| err.to_string())?;
    let report = audit(&text, &post).map_err(|err| format!("{err}\n{}", snippet(&text)))?;
    if report.motions.len() != steps.len() {
        return Err(format!("{} motions for {} steps\n{}", report.motions.len(), steps.len(), snippet(&text)));
    }
    let inverse_post = id != "mach3";
    let mut prev_angle: Option<f64> = None;
    let mut prev_tip = None;
    let mut prev_inverse = false;
    let mut modal_feed = None;
    for (motion, step) in report.motions.iter().zip(steps) {
        let machine = part_to_machine(setup, turned(setup, step.tip, step.angle), step.angle).tip();
        let end = vec3(motion.end);
        if gap(end, machine) > quant(post.linear_decimals()) * 3_f64.sqrt() + 1e-9 {
            return Err(format!("line {}: machine {:?} != {:?}", motion.line, motion.end, [machine.x(), machine.y(), machine.z()]));
        }
        let Some(posted) = motion.rotary else {
            return Err(format!("line {}: missing rotary", motion.line));
        };
        if (posted - step.angle).abs() > quant(3) + 1e-9 {
            return Err(format!("line {}: A {posted} != {}", motion.line, step.angle));
        }
        let changed = prev_angle.is_some_and(|prev| (prev - step.angle).abs() > 1e-9) || prev_angle.is_none();
        match step.feed {
            None => {
                if motion.mode != MotionMode::Rapid {
                    return Err(format!("line {}: not a rapid", motion.line));
                }
                if inverse_post && prev_inverse && motion.feed_mode != FeedMode::InverseTime {
                    return Err(format!("line {}: rapid cleared inverse time", motion.line));
                }
                if !inverse_post && motion.feed_mode != FeedMode::PerMinute {
                    return Err(format!("line {}: mach3 left per-minute", motion.line));
                }
            }
            Some(feed) => {
                if motion.mode != MotionMode::Linear {
                    return Err(format!("line {}: not a feed", motion.line));
                }
                let length = prev_tip.map(|prev| gap(prev, step.tip)).unwrap_or(0.0);
                let use_inverse = inverse_post && changed && prev_angle.is_some();
                if use_inverse {
                    if motion.feed_mode != FeedMode::InverseTime || !motion.feed_on_block {
                        return Err(format!("line {}: inverse feed missing\n{}", motion.line, snippet(&text)));
                    }
                    let expect = feed / length;
                    let got = motion.feed.ok_or_else(|| format!("line {}: no F", motion.line))?;
                    if (got - expect).abs() > quant(post.feed_decimals()) + 1e-6 {
                        return Err(format!("line {}: F {got} != {expect}", motion.line));
                    }
                    prev_inverse = true;
                    modal_feed = Some(expect);
                } else {
                    if motion.feed_mode != FeedMode::PerMinute {
                        return Err(format!("line {}: still inverse time", motion.line));
                    }
                    let got = motion.feed.ok_or_else(|| format!("line {}: no F", motion.line))?;
                    if (got - feed).abs() > quant(post.feed_decimals()) + 1e-6 {
                        return Err(format!("line {}: F {got} != {feed}", motion.line));
                    }
                    let require = prev_inverse || modal_feed.is_none_or(|prev| (prev - feed).abs() > 1e-6);
                    if require && !motion.feed_on_block {
                        return Err(format!("line {}: G94 kept the previous F", motion.line));
                    }
                    prev_inverse = false;
                    modal_feed = Some(feed);
                }
            }
        }
        prev_angle = Some(step.angle);
        prev_tip = Some(step.tip);
    }
    if inverse_post && report.feed_mode != FeedMode::PerMinute && steps.iter().any(|step| step.feed.is_some()) {
        // A program that ends on an inverse-time block stays in G93 until the next same-angle feed.
        // The fixed sequence ends on an angle change, so G93 is still modal. That matches the post.
    }
    Ok(())
}

fn turned(setup: &RotarySetup, tip: Vec3, angle: f64) -> Pose {
    let spindle = Pose::new(Vec3::ZERO, Vec3::Z).expect("spindle");
    let axis = machine_to_part(setup, spindle, angle).axis();
    Pose::new(tip, axis).expect("pose").with_rotary(angle).expect("angle")
}

fn shift(tip: Vec3, x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(tip.x() + x, tip.y() + y, tip.z() + z).expect("shift")
}

fn table_map(kin: &Kinematics, angles: [f64; 2], mut point: Vec3) -> Vec3 {
    let mut table = Vec::new();
    for (index, joint) in kin.rotaries.iter().enumerate() {
        if joint.carrier == Carrier::Table {
            table.push((index, joint.axis));
        }
    }
    for (index, axis) in table.into_iter().rev() {
        let angle = if index == 0 { angles[0] } else { angles[1] };
        point = rotate_about(axis, point, angle);
    }
    point
}

fn rotate_about(axis: Vec3, point: Vec3, angle_deg: f64) -> Vec3 {
    let Some(axis) = unit(axis) else {
        return point;
    };
    let theta = angle_deg.to_radians();
    let (sin, cos) = theta.sin_cos();
    let along = dot(axis, point);
    let cx = axis.y() * point.z() - axis.z() * point.y();
    let cy = axis.z() * point.x() - axis.x() * point.z();
    let cz = axis.x() * point.y() - axis.y() * point.x();
    let x = point.x() * cos + cx * sin + axis.x() * along * (1.0 - cos);
    let y = point.y() * cos + cy * sin + axis.y() * along * (1.0 - cos);
    let z = point.z() * cos + cz * sin + axis.z() * along * (1.0 - cos);
    Vec3::new(x, y, z).unwrap_or(point)
}

fn unit(v: Vec3) -> Option<Vec3> {
    let len = (v.x() * v.x() + v.y() * v.y() + v.z() * v.z()).sqrt();
    if !len.is_finite() || len < 1e-12 {
        return None;
    }
    Vec3::new(v.x() / len, v.y() / len, v.z() / len).ok()
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a.x() * b.x() + a.y() * b.y() + a.z() * b.z()
}

fn gap(a: Vec3, b: Vec3) -> f64 {
    let d = [a.x() - b.x(), a.y() - b.y(), a.z() - b.z()];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn axis_deg(a: Vec3, b: Vec3) -> f64 {
    dot(a, b).clamp(-1.0, 1.0).acos().to_degrees()
}

fn axis_apart(a: Vec3, b: Vec3) -> bool {
    let d = [a.x() - b.x(), a.y() - b.y(), a.z() - b.z()];
    d[0] * d[0] + d[1] * d[1] + d[2] * d[2] > 1e-18
}

fn lerp2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * 0.5, a[1] + (b[1] - a[1]) * 0.5]
}

fn lerp_vec(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5, (a.z() + b.z()) * 0.5).unwrap_or(a)
}

fn angles_close(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).abs() <= 1e-9 && (a[1] - b[1]).abs() <= 1e-9
}

fn vec3(point: [f64; 3]) -> Vec3 {
    Vec3::new(point[0], point[1], point[2]).unwrap_or(Vec3::ZERO)
}

fn slerp_axis(start: Vec3, end: Vec3, t: f64) -> Result<Vec3, String> {
    let start = unit(start).ok_or("axis")?;
    let end = unit(end).ok_or("axis")?;
    let cosine = dot(start, end).clamp(-1.0, 1.0);
    let theta = cosine.acos();
    let sin_theta = theta.sin();
    if sin_theta <= 1e-12 {
        return Ok(if cosine > 0.0 { start } else { end });
    }
    let w0 = ((1.0 - t) * theta).sin() / sin_theta;
    let w1 = (t * theta).sin() / sin_theta;
    unit(Vec3::new(start.x() * w0 + end.x() * w1, start.y() * w0 + end.y() * w1, start.z() * w0 + end.z() * w1).unwrap_or(start)).ok_or("axis".into())
}
