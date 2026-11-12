//! Modal replay of random lathe and mill-turn programs.
//!
//! Each case is posted with every lathe post that documents the feature, read back, and checked
//! against the IR. A posted move the post rejects on purpose (a radius that will not fit, a feed
//! that rounds to zero) is skipped. Any other failure is a post or checker bug.
//!
//! LinuxCNC leaves the C-axis switch and the spindle brake empty, so a G-code replay cannot see
//! engage or clamp. Those flags stay off. Haas writes them. Ordinary turning feed is per minute
//! on every built-in lathe post. Haas `G99` applies only around `G32`. Past-centre facing is
//! posted when the IR radius is negative.

#![allow(clippy::expect_used)]

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, Head, HeadAxis, LiveDirection, MoveRole, Plane, Pose, ProcessKind,
    ProcessParams, Program, Spindle, SpindleSpeed, Tool, TurnBinding, TurningSetup, Vec3,
};
use campanion_post::{InterpFrame, LatheFeedMode, LatheMotion, NoseComp, PostOptions, XMode, audit_lathe, builtin, compare, post_program, read_back};
use proptest::prelude::*;
use proptest::test_runner::{RngSeed, TestCaseError};

const TAU: f64 = std::f64::consts::TAU;
const ARC_TOL: f64 = 0.05;
const END_TOL: f64 = 1e-3;

const TURN_POSTS: &[&str] = &["linuxcnc_lathe", "fanuc_lathe", "haas_lathe", "grbl_lathe", "generic_lathe_2ch"];
const LIVE_POSTS: &[&str] = &["linuxcnc_lathe", "haas_lathe"];

fn options() -> PostOptions {
    PostOptions { arc_tolerance: ARC_TOL }
}

fn post(id: &str) -> campanion_post::PostDef {
    builtin(id).expect("built-in post")
}

fn xz(x: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, 0.0, z).expect("tip"), Vec3::Z).expect("pose")
}

fn xyz(x: f64, y: f64, z: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
}

fn tip_of(pose: Pose) -> [f64; 3] {
    let tip = pose.tip();
    [tip.x(), tip.y(), tip.z()]
}

fn feeds() -> Feeds {
    Feeds::try_new(100.0, None, None, None, None).expect("feeds")
}

fn tool_row(id: u32) -> Tool {
    Tool::try_new(id, Some(format!("t{id}")), None, None).expect("tool")
}

fn spindle_on() -> ClBlock {
    ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params: ProcessParams::new() })
}

fn spindle_off() -> ClBlock {
    ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle })
}

fn turning_setup(css: bool, reversed: bool) -> TurningSetup {
    let speed = if css { SpindleSpeed::css(180.0, 3_000.0).expect("css") } else { SpindleSpeed::rpm(900.0).expect("rpm") };
    let spindle = Spindle::try_new("main", None, 0.0, 3_000.0).expect("spindle").with_reversed(reversed);
    let head = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("head");
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(speed)).expect("speed");
    TurningSetup::try_new(vec![spindle], vec![head], None, Vec::new(), vec![binding]).expect("turning")
}

fn live_setup() -> TurningSetup {
    let spindle = Spindle::try_new("main", Some(0), 0.0, 3_000.0).expect("spindle");
    let head = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Y, HeadAxis::Z], 12, true).expect("head");
    let binding = TurnBinding::try_new(0, "main", "turret1", 0).expect("binding");
    TurningSetup::try_new(vec![spindle], vec![head], None, Vec::new(), vec![binding]).expect("turning")
}

fn program_with(name: &str, tools: Vec<Tool>, paths: Vec<ClPath>, setup: TurningSetup) -> Program {
    Program::try_new(Some(name.into()), tools, None, paths, Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning")
}

fn one_path(name: &str, tool: u32, blocks: Vec<ClBlock>, setup: TurningSetup) -> Program {
    let path = ClPath::try_new(Some(name.into()), Some(tool), CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path");
    program_with(name, vec![tool_row(tool)], vec![path], setup)
}

#[derive(Clone, Debug)]
struct ArcExpect {
    center: [f64; 3],
    sense: ArcSense,
    sweep: f64,
    plane: Plane,
}

#[derive(Clone, Debug)]
struct Expect {
    end: [f64; 3],
    rapid: bool,
    feed: f64,
    lead: Option<f64>,
    arc: Option<ArcExpect>,
    tool: u32,
    live: bool,
    /// Modal C after this move, when the cycle is indexed.
    c: Option<f64>,
    /// This move is a rapid that writes a new C word.
    c_word: bool,
    on_sub: bool,
    brake: bool,
}

struct Built {
    program: Program,
    moves: Vec<Expect>,
}

fn push_motion(blocks: &mut Vec<ClBlock>, moves: &mut Vec<Expect>, motion: ClMotion, expect: Expect) {
    blocks.push(ClBlock::Motion(motion));
    moves.push(expect);
}

fn rapid_at(blocks: &mut Vec<ClBlock>, moves: &mut Vec<Expect>, pose: Pose, tool: u32, c: Option<f64>, c_word: bool, on_sub: bool) {
    let end = tip_of(pose);
    push_motion(
        blocks,
        moves,
        ClMotion::Rapid { to: pose, feed: None },
        Expect { end, rapid: true, feed: 0.0, lead: None, arc: None, tool, live: false, c, c_word, on_sub, brake: false },
    );
}

fn linear_at(
    blocks: &mut Vec<ClBlock>,
    moves: &mut Vec<Expect>,
    pose: Pose,
    feed: f64,
    tool: u32,
    live: bool,
    c: Option<f64>,
    on_sub: bool,
    brake: bool,
) {
    let end = tip_of(pose);
    push_motion(
        blocks,
        moves,
        ClMotion::Linear { to: pose, feed, role: MoveRole::Cut, power: None },
        Expect { end, rapid: false, feed, lead: None, arc: None, tool, live, c, c_word: false, on_sub, brake },
    );
}

fn tolerated(err: &str) -> bool {
    const NEEDLES: &[&str] = &[
        "arc end rounds onto its start",
        "arc radius",
        "arc end radius",
        "arc end does not match",
        "arc linearises",
        "feed rounds to zero",
        "face path crosses the spindle centre",
    ];
    NEEDLES.iter().any(|needle| err.contains(needle))
}

fn quantum(decimals: u8) -> f64 {
    10_f64.powi(-i32::from(decimals))
}

fn near3(got: [f64; 3], expect: [f64; 3], tol: f64) -> bool {
    (got[0] - expect[0]).abs() <= tol && (got[1] - expect[1]).abs() <= tol && (got[2] - expect[2]).abs() <= tol
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2])
}

fn uv(plane: Plane, point: [f64; 3]) -> (f64, f64) {
    match plane {
        Plane::Xy => (point[0], point[1]),
        Plane::Zx => (point[2], point[0]),
        Plane::Yz => (point[1], point[2]),
    }
}

fn angle_of(point: [f64; 3], center: [f64; 3], plane: Plane) -> f64 {
    let (u, v) = uv(plane, point);
    let (cu, cv) = uv(plane, center);
    (v - cv).atan2(u - cu)
}

fn forward_delta(a0: f64, a1: f64, ccw: bool) -> f64 {
    let delta = (a1 - a0).rem_euclid(TAU);
    if ccw { delta } else { (TAU - delta).rem_euclid(TAU) }
}

/// Sweep a controller cuts from the written centre. Equal endpoints with a real chord are a full turn.
fn controller_sweep(start: [f64; 3], end: [f64; 3], center: [f64; 3], plane: Plane, sense: ArcSense) -> f64 {
    let (su, sv) = uv(plane, start);
    let (eu, ev) = uv(plane, end);
    let (cu, cv) = uv(plane, center);
    let radius = (su - cu).hypot(sv - cv);
    let chord = (su - eu).hypot(sv - ev);
    if chord <= 1e-9_f64.max(radius * 1e-9) {
        return TAU;
    }
    let delta = forward_delta(angle_of(start, center, plane), angle_of(end, center, plane), sense == ArcSense::Ccw);
    if delta <= 1e-12 { TAU } else { delta }
}

fn offset_tool(id: &str) -> bool {
    matches!(id, "fanuc_lathe" | "haas_lathe" | "generic_lathe_2ch")
}

fn haas(id: &str) -> bool {
    id == "haas_lathe"
}

fn linux(id: &str) -> bool {
    id == "linuxcnc_lathe"
}

fn check_common(motion: &LatheMotion, expect: &Expect, id: &str, css: bool, reversed: bool) -> Result<(), String> {
    if motion.x_mode != XMode::Diameter {
        return Err(format!("line {}: X mode is not diameter", motion.line));
    }
    if matches!(motion.nose, NoseComp::Left | NoseComp::Right) {
        return Err(format!("line {}: nose compensation {} is not written by these posts", motion.line, motion.nose.nose_name()));
    }
    if motion.tool != Some(expect.tool) {
        return Err(format!("line {}: tool {:?} want {}", motion.line, motion.tool, expect.tool));
    }
    let want_offset = if offset_tool(id) { Some(expect.tool) } else { None };
    if motion.offset != want_offset {
        return Err(format!("line {}: offset {:?} want {want_offset:?}", motion.line, motion.offset));
    }
    if css && (!motion.css || motion.css_cap.is_none()) {
        return Err(format!("line {}: CSS feed without a cap", motion.line));
    }
    if !css && motion.css {
        return Err(format!("line {}: CSS is on for an rpm program", motion.line));
    }
    if expect.live {
        if !motion.live_on {
            return Err(format!("line {}: live tool is off during a live feed", motion.line));
        }
        if motion.spindle_on {
            return Err(format!("line {}: work spindle is running during a live feed", motion.line));
        }
    } else if !expect.rapid && !motion.spindle_on {
        return Err(format!("line {}: work spindle is off during a turning feed", motion.line));
    }
    if motion.c_engaged && motion.spindle_on {
        return Err(format!("line {}: work spindle is running while C is engaged", motion.line));
    }
    if !expect.rapid && motion.spindle_on && motion.spindle_clockwise == reversed {
        return Err(format!("line {}: spindle direction does not follow the binding", motion.line));
    }
    if motion.on_sub != expect.on_sub {
        return Err(format!("line {}: on_sub {} want {}", motion.line, motion.on_sub, expect.on_sub));
    }
    if motion.end[0] < -END_TOL && expect.end[0] >= -END_TOL && motion.frame != InterpFrame::Face {
        return Err(format!("line {}: posted radius {} is below 0 and the IR radius is not", motion.line, motion.end[0]));
    }
    Ok(())
}

trait NoseName {
    fn nose_name(self) -> &'static str;
}

impl NoseName for NoseComp {
    fn nose_name(self) -> &'static str {
        match self {
            NoseComp::Unset => "unset",
            NoseComp::Off => "off",
            NoseComp::Left => "left",
            NoseComp::Right => "right",
        }
    }
}

fn check_feed_value(motion: &LatheMotion, feed: f64, decimals: u8) -> Result<(), String> {
    let scale = quantum(decimals);
    let Some(got) = motion.feed else {
        return Err(format!("line {}: feed move has no F", motion.line));
    };
    if (got - feed).abs() > scale + 1e-9 {
        return Err(format!("line {}: feed {got} want {feed}", motion.line));
    }
    if motion.feed_mode != LatheFeedMode::PerMinute {
        return Err(format!("line {}: feed mode {:?} want per minute", motion.line, motion.feed_mode));
    }
    Ok(())
}

/// Walk posted motions against IR ends. A linearised arc may be several `G1` blocks.
fn match_moves(motions: &[LatheMotion], expects: &[Expect], id: &str, css: bool, reversed: bool, cartesian: bool) -> Result<(), String> {
    let post = post(id);
    let q = quantum(post.linear_decimals());
    let mut cursor = 0usize;
    let mut prev = [0.0, 0.0, 0.0];
    for expect in expects {
        if !cartesian {
            cursor = motions.len();
            break;
        }
        let start_cursor = cursor;
        let mut hit = None;
        while cursor < motions.len() {
            let motion = &motions[cursor];
            cursor += 1;
            if near3(motion.end, expect.end, END_TOL) {
                hit = Some(cursor - 1);
                break;
            }
            if motion.mode != campanion_post::MotionMode::Linear || motion.thread || motion.arc.is_some() {
                return Err(format!("line {}: unexpected motion while looking for {:?} (at {start_cursor})", motion.line, expect.end));
            }
            if cursor - start_cursor > 40 {
                return Err(format!("no posted end near {:?} starting at motion {start_cursor}", expect.end));
            }
        }
        let Some(index) = hit else {
            return Err(format!("missing posted end {:?} after {start_cursor} motions", expect.end));
        };
        let motion = &motions[index];
        check_common(motion, expect, id, css, reversed)?;
        let linearised = index != start_cursor;
        if expect.rapid {
            if motion.mode != campanion_post::MotionMode::Rapid {
                return Err(format!("line {}: rapid posted as {:?}", motion.line, motion.mode));
            }
        } else if let Some(lead) = expect.lead {
            if !motion.thread {
                return Err(format!("line {}: thread was not posted as a thread", motion.line));
            }
            let got = motion.lead.unwrap_or(0.0);
            if (got - lead).abs() > q + 1e-9 {
                return Err(format!("line {}: lead {got} want {lead}", motion.line));
            }
            let want = if haas(id) { LatheFeedMode::PerRevolution } else { LatheFeedMode::PerMinute };
            if motion.feed_mode != want {
                return Err(format!("line {}: thread feed mode {:?}", motion.line, motion.feed_mode));
            }
        } else if !linearised {
            check_feed_value(motion, expect.feed, post.feed_decimals())?;
        }
        if let Some(arc) = &expect.arc
            && !linearised
        {
            let Some(posted) = motion.arc else {
                return Err(format!("line {}: arc posted as a line", motion.line));
            };
            if posted.sense != arc.sense {
                return Err(format!("line {}: arc sense {:?} want {:?}", motion.line, posted.sense, arc.sense));
            }
            if dist3(posted.center, arc.center) > 0.05 {
                return Err(format!("line {}: arc centre {:?} want {:?}", motion.line, posted.center, arc.center));
            }
            let swept = controller_sweep(prev, motion.end, posted.center, arc.plane, posted.sense);
            let (u, v) = uv(arc.plane, prev);
            let (cu, cv) = uv(arc.plane, posted.center);
            let radius = (u - cu).hypot(v - cv).max(1e-6);
            let allowance = 4.0 * q / radius + 1e-3;
            if swept > arc.sweep + allowance {
                return Err(format!("line {}: controller sweep {swept} exceeds program {} by more than {allowance}", motion.line, arc.sweep));
            }
        }
        if let Some(c) = expect.c
            && let Some(got) = motion.c
            && (got - c).abs() > END_TOL
        {
            return Err(format!("line {}: C {got} want {c}", motion.line));
        }
        if !expect.rapid && expect.c.is_some() && motion.c_on_block {
            return Err(format!("line {}: indexed feed writes C", motion.line));
        }
        if expect.c_word && !motion.c_on_block {
            return Err(format!("line {}: indexed rapid did not write C", motion.line));
        }
        let brake_visible = haas(id);
        if brake_visible && motion.brake != expect.brake {
            return Err(format!("line {}: brake {} want {}", motion.line, motion.brake, expect.brake));
        }
        if !brake_visible && motion.brake {
            return Err(format!("line {}: brake is clamped but this post writes no brake word", motion.line));
        }
        if linux(id) && motion.c_engaged {
            return Err(format!("line {}: C engage is set but linuxcnc writes no engage word", motion.line));
        }
        if haas(id) && expect.c.is_some() && !motion.c_engaged {
            return Err(format!("line {}: indexed move without C engaged", motion.line));
        }
        prev = motion.end;
    }
    if cartesian && cursor != motions.len() {
        return Err(format!("posted {} motions and matched {cursor}", motions.len()));
    }
    Ok(())
}

fn replay(program: &Program, id: &str, expects: &[Expect], css: bool, reversed: bool, cartesian: bool) -> Result<(), String> {
    let post = post(id);
    let text = match post_program(program, &post, &options()) {
        Ok(text) => text,
        Err(err) if tolerated(&err.to_string()) => return Ok(()),
        Err(err) => return Err(format!("post: {err}")),
    };
    let audit = audit_lathe(&text, &post).map_err(|err| format!("audit: {err}\n{text}"))?;
    if audit.g96_without_cap {
        return Err(format!("G96 without a CSS cap\n{text}"));
    }
    compare(program, &read_back(&text, &post).map_err(|err| format!("read_back: {err}"))?, END_TOL, &post)
        .map_err(|err| format!("compare: {err}\n{text}"))?;
    match_moves(&audit.motions, expects, id, css, reversed, cartesian)?;
    Ok(())
}

fn finish(audit_on_sub: bool, id: &str, text_note: &str) -> Result<(), String> {
    if audit_on_sub {
        return Err(format!("{id}: program ended still on the sub-spindle\n{text_note}"));
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct TurnSpec {
    css: bool,
    reversed: bool,
    tool: u32,
    start_r: f64,
    sweep: f64,
    ccw: bool,
    groove_depth: f64,
    groove_width: f64,
    thread: bool,
    lead: f64,
    past_centre: bool,
    feed: f64,
}

fn build_turn(spec: &TurnSpec) -> Built {
    let mut blocks = vec![ClBlock::Event(Event::ToolChange { tool_id: spec.tool }), spindle_on()];
    let mut moves = Vec::new();
    let tool = spec.tool;
    let feed = spec.feed;
    let mut z = 1.0;
    rapid_at(&mut blocks, &mut moves, xz(spec.start_r, z), tool, None, false, false);
    z = -12.0;
    linear_at(&mut blocks, &mut moves, xz(spec.start_r, z), feed, tool, false, None, false, false);

    let radius = (spec.start_r * 0.35).clamp(2.0, 10.0);
    let cx = spec.start_r - radius;
    let cz = z;
    let end_x = spec.start_r - radius * (1.0 - spec.sweep.cos());
    let end_z = if spec.ccw { z - radius * spec.sweep.sin() } else { z + radius * spec.sweep.sin() };
    let sense = if spec.ccw { ArcSense::Ccw } else { ArcSense::Cw };
    let center = Vec3::new(cx, 0.0, cz).expect("centre");
    let arc_end = xz(end_x, end_z);
    push_motion(
        &mut blocks,
        &mut moves,
        ClMotion::Arc { to: arc_end, plane: Plane::Zx, center, sense, feed, role: MoveRole::Cut, power: None },
        Expect {
            end: tip_of(arc_end),
            rapid: false,
            feed,
            lead: None,
            arc: Some(ArcExpect { center: [cx, 0.0, cz], sense, sweep: spec.sweep, plane: Plane::Zx }),
            tool,
            live: false,
            c: None,
            c_word: false,
            on_sub: false,
            brake: false,
        },
    );

    // Quarter-circle onto the axis: centre on Z, start at +X, counter-clockwise a quarter turn.
    let quarter_r = end_x;
    let quarter_end_z = end_z - quarter_r;
    let quarter_to = xz(0.0, quarter_end_z);
    let quarter_c = Vec3::new(0.0, 0.0, end_z).expect("quarter centre");
    push_motion(
        &mut blocks,
        &mut moves,
        ClMotion::Arc { to: quarter_to, plane: Plane::Zx, center: quarter_c, sense: ArcSense::Ccw, feed, role: MoveRole::Cut, power: None },
        Expect {
            end: tip_of(quarter_to),
            rapid: false,
            feed,
            lead: None,
            arc: Some(ArcExpect { center: [0.0, 0.0, end_z], sense: ArcSense::Ccw, sweep: TAU / 4.0, plane: Plane::Zx }),
            tool,
            live: false,
            c: None,
            c_word: false,
            on_sub: false,
            brake: false,
        },
    );

    let face_z = quarter_end_z - 1.0;
    rapid_at(&mut blocks, &mut moves, xz(16.0, face_z), tool, None, false, false);
    linear_at(&mut blocks, &mut moves, xz(0.0, face_z), feed, tool, false, None, false, false);

    let groove_z = face_z - 2.0;
    let floor = 16.0 - spec.groove_depth;
    rapid_at(&mut blocks, &mut moves, xz(16.0, groove_z), tool, None, false, false);
    linear_at(&mut blocks, &mut moves, xz(floor, groove_z), feed, tool, false, None, false, false);
    linear_at(&mut blocks, &mut moves, xz(floor, groove_z - spec.groove_width), feed, tool, false, None, false, false);
    linear_at(&mut blocks, &mut moves, xz(16.0, groove_z - spec.groove_width), feed, tool, false, None, false, false);

    let mut z_now = groove_z - spec.groove_width;
    if spec.thread && !spec.css {
        rapid_at(&mut blocks, &mut moves, xz(12.0, z_now), tool, None, false, false);
        let thread_to = xz(11.0, z_now - 10.0);
        push_motion(
            &mut blocks,
            &mut moves,
            ClMotion::Thread { to: thread_to, lead: spec.lead },
            Expect {
                end: tip_of(thread_to),
                rapid: false,
                feed,
                lead: Some(spec.lead),
                arc: None,
                tool,
                live: false,
                c: None,
                c_word: false,
                on_sub: false,
                brake: false,
            },
        );
        z_now -= 10.0;
        linear_at(&mut blocks, &mut moves, xz(14.0, z_now), feed, tool, false, None, false, false);
    }
    if spec.past_centre {
        rapid_at(&mut blocks, &mut moves, xz(2.0, z_now), tool, None, false, false);
        linear_at(&mut blocks, &mut moves, xz(-0.8, z_now), feed, tool, false, None, false, false);
    }
    blocks.push(spindle_off());
    let setup = turning_setup(spec.css, spec.reversed);
    Built { program: one_path("turn", tool, blocks, setup), moves }
}

fn check_turn(spec: &TurnSpec) -> Result<(), TestCaseError> {
    let built = build_turn(spec);
    for id in TURN_POSTS {
        if let Err(message) = replay(&built.program, id, &built.moves, spec.css, spec.reversed, true) {
            return Err(TestCaseError::fail(format!("{id}: {message}\n{spec:?}")));
        }
        let post = post(id);
        let text = match post_program(&built.program, &post, &options()) {
            Ok(text) => text,
            Err(err) if tolerated(&err.to_string()) => continue,
            Err(err) => return Err(TestCaseError::fail(format!("{id}: {err}"))),
        };
        let audit = audit_lathe(&text, &post).map_err(|err| TestCaseError::fail(format!("{id}: {err}")))?;
        if audit.spindle_on || audit.live_on || audit.on_sub || audit.c_engaged {
            return Err(TestCaseError::fail(format!("{id}: machine left running\n{text}")));
        }
        if spec.css && audit.css_cap.is_none() {
            return Err(TestCaseError::fail(format!("{id}: missing CSS cap\n{text}")));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
enum LiveKind {
    IndexedFront,
    IndexedOd,
    Face,
    Cylinder,
}

#[derive(Clone, Debug)]
struct LiveSpec {
    kind: LiveKind,
    c0: f64,
    c1: f64,
    y: f64,
    feed: f64,
}

fn radial(x: f64, y: f64, z: f64, c: f64) -> Pose {
    Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::X).expect("pose").with_rotary(c).expect("c")
}

fn axial(x: f64, y: f64, z: f64, c: f64) -> Pose {
    xyz(x, y, z).with_rotary(c).expect("c")
}

fn build_live(spec: &LiveSpec) -> Built {
    let mut blocks = vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: true })];
    let mut moves = Vec::new();
    let feed = spec.feed;
    match spec.kind {
        LiveKind::IndexedFront => {
            rapid_at(&mut blocks, &mut moves, axial(12.0, 0.0, -4.0, spec.c0), 1, Some(spec.c0), true, false);
            blocks.push(ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }));
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }));
            linear_live(&mut blocks, &mut moves, axial(12.0, spec.y, -4.0, spec.c0), feed, Some(spec.c0), true);
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }));
            // Y returns at the same C. The next rapid is the only C change.
            rapid_at(&mut blocks, &mut moves, axial(12.0, 0.0, -4.0, spec.c0), 1, Some(spec.c0), false, false);
            rapid_at(&mut blocks, &mut moves, axial(12.0, 0.0, -4.0, spec.c1), 1, Some(spec.c1), true, false);
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }));
            linear_live(&mut blocks, &mut moves, axial(9.0, 0.0, -4.0, spec.c1), feed, Some(spec.c1), true);
        }
        LiveKind::IndexedOd => {
            rapid_at(&mut blocks, &mut moves, radial(22.0, 0.0, -6.0, spec.c0), 1, Some(spec.c0), true, false);
            blocks.push(ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }));
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }));
            linear_live(&mut blocks, &mut moves, radial(22.0, spec.y, -6.0, spec.c0), feed, Some(spec.c0), true);
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }));
            rapid_at(&mut blocks, &mut moves, radial(22.0, 0.0, -6.0, spec.c0), 1, Some(spec.c0), false, false);
            rapid_at(&mut blocks, &mut moves, radial(22.0, 0.0, -12.0, spec.c1), 1, Some(spec.c1), true, false);
            blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: true }));
            linear_live(&mut blocks, &mut moves, radial(18.0, 0.0, -12.0, spec.c1), feed, Some(spec.c1), true);
        }
        LiveKind::Face => {
            blocks.push(ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }));
            blocks.push(ClBlock::Event(Event::FaceInterpolation { on: true }));
            rapid_at(&mut blocks, &mut moves, xyz(6.0, -4.0, -1.0), 1, None, false, false);
            linear_live(&mut blocks, &mut moves, xyz(14.0, -4.0, -1.0), feed, None, false);
            linear_live(&mut blocks, &mut moves, xyz(14.0, 4.0, -1.0), feed, None, false);
            linear_live(&mut blocks, &mut moves, xyz(6.0, 4.0, -1.0), feed, None, false);
            blocks.push(ClBlock::Event(Event::FaceInterpolation { on: false }));
        }
        LiveKind::Cylinder => {
            blocks.push(ClBlock::Event(Event::LiveToolOn { rpm: 1_500.0, direction: LiveDirection::Forward }));
            blocks.push(ClBlock::Event(Event::CylinderInterpolation { radius: Some(20.0) }));
            rapid_at(&mut blocks, &mut moves, xyz(20.0, -6.0, -8.0), 1, None, false, false);
            linear_live(&mut blocks, &mut moves, xyz(20.0, 6.0, -8.0), feed, None, false);
            linear_live(&mut blocks, &mut moves, xyz(20.0, 6.0, -16.0), feed, None, false);
            blocks.push(ClBlock::Event(Event::CylinderInterpolation { radius: None }));
        }
    }
    blocks.push(ClBlock::Event(Event::LiveToolOff));
    if matches!(spec.kind, LiveKind::IndexedFront | LiveKind::IndexedOd) {
        blocks.push(ClBlock::Event(Event::SpindleBrake { spindle: "main".into(), clamp: false }));
    }
    blocks.push(ClBlock::Event(Event::CAxis { spindle: "main".into(), engage: false }));
    // Mark live feeds. `linear_live` already set `live`.
    let path = ClPath::try_new(Some("live".into()), Some(1), CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path");
    Built { program: program_with("millturn", vec![tool_row(1)], vec![path], live_setup()), moves }
}

fn linear_live(blocks: &mut Vec<ClBlock>, moves: &mut Vec<Expect>, pose: Pose, feed: f64, c: Option<f64>, brake: bool) {
    let end = tip_of(pose);
    push_motion(
        blocks,
        moves,
        ClMotion::Linear { to: pose, feed, role: MoveRole::Cut, power: None },
        Expect { end, rapid: false, feed, lead: None, arc: None, tool: 1, live: true, c, c_word: false, on_sub: false, brake },
    );
}

fn check_live(spec: &LiveSpec) -> Result<(), TestCaseError> {
    let built = build_live(spec);
    let native_only = matches!(spec.kind, LiveKind::Face | LiveKind::Cylinder);
    for id in LIVE_POSTS {
        // LinuxCNC has no G112 or G107, so a face or cylinder path is posted in X and C.
        let cartesian = !(linux(id) && native_only);
        if let Err(message) = replay(&built.program, id, &built.moves, false, false, cartesian) {
            return Err(TestCaseError::fail(format!("{id}: {message}\n{spec:?}")));
        }
        let posted = post(id);
        let text = match post_program(&built.program, &posted, &options()) {
            Ok(text) => text,
            Err(err) if tolerated(&err.to_string()) => continue,
            Err(err) => return Err(TestCaseError::fail(format!("{id}: post: {err}"))),
        };
        let audit = audit_lathe(&text, &posted).map_err(|err| TestCaseError::fail(format!("{id}: audit: {err}\n{text}")))?;
        if audit.live_on || audit.spindle_on || audit.on_sub {
            return Err(TestCaseError::fail(format!("{id}: live cycle left a spindle running\n{text}")));
        }
        if haas(id) && audit.c_engaged {
            return Err(TestCaseError::fail(format!("{id}: C still engaged\n{text}")));
        }
        if native_only {
            check_interp(&audit.motions, id, spec)?;
        }
        if audit.motions.iter().any(|motion| motion.end[0] < -END_TOL && motion.frame != InterpFrame::Face) {
            return Err(TestCaseError::fail(format!("{id}: radius below 0\n{text}")));
        }
    }
    Ok(())
}

fn check_interp(motions: &[LatheMotion], id: &str, spec: &LiveSpec) -> Result<(), TestCaseError> {
    let feeds: Vec<_> = motions.iter().filter(|motion| !matches!(motion.mode, campanion_post::MotionMode::Rapid)).collect();
    if feeds.is_empty() {
        return Err(TestCaseError::fail(format!("{id}: interpolation posted no feed")));
    }
    for motion in &feeds {
        if motion.brake {
            return Err(TestCaseError::fail(format!("{id}: line {}: brake clamped during interpolation", motion.line)));
        }
        if !motion.live_on {
            return Err(TestCaseError::fail(format!("{id}: line {}: live tool off during interpolation", motion.line)));
        }
        if motion.spindle_on {
            return Err(TestCaseError::fail(format!("{id}: line {}: work spindle on during interpolation", motion.line)));
        }
    }
    if haas(id) {
        let want = match spec.kind {
            LiveKind::Face => InterpFrame::Face,
            LiveKind::Cylinder => InterpFrame::Cylinder,
            LiveKind::IndexedFront | LiveKind::IndexedOd => InterpFrame::Off,
        };
        for motion in &feeds {
            if motion.frame != want {
                return Err(TestCaseError::fail(format!("{id}: line {}: frame {:?} want {want:?}", motion.line, motion.frame)));
            }
            if motion.feed_mode != LatheFeedMode::PerMinute {
                return Err(TestCaseError::fail(format!("{id}: line {}: interpolation feed is {:?}", motion.line, motion.feed_mode)));
            }
            if motion.c_on_block {
                return Err(TestCaseError::fail(format!("{id}: line {}: C word inside interpolation", motion.line)));
            }
        }
    }
    if linux(id) {
        let inverse = feeds.iter().any(|motion| motion.feed_mode == LatheFeedMode::InverseTime && motion.feed_on_block);
        if !inverse {
            return Err(TestCaseError::fail(format!("{id}: fallback interpolation wrote no inverse-time feed")));
        }
        for motion in &feeds {
            if motion.c_on_block && motion.feed_mode != LatheFeedMode::InverseTime {
                return Err(TestCaseError::fail(format!("{id}: line {}: C feed is not inverse time", motion.line)));
            }
            if motion.frame != InterpFrame::Off {
                return Err(TestCaseError::fail(format!("{id}: line {}: linuxcnc has no interpolation frame", motion.line)));
            }
        }
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct SubSpec {
    main_r: f64,
    sub_r: f64,
    feed: f64,
}

fn build_sub(spec: &SubSpec) -> Built {
    let main_blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        spindle_on(),
        ClBlock::Motion(ClMotion::Rapid { to: xz(spec.main_r, 1.0), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: xz(spec.main_r, -6.0), feed: spec.feed, role: MoveRole::Cut, power: None }),
        spindle_off(),
    ];
    let sub_blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 2 }),
        spindle_on(),
        ClBlock::Motion(ClMotion::Rapid { to: xz(spec.sub_r, 0.5), feed: None }),
        ClBlock::Motion(ClMotion::Linear { to: xz(spec.sub_r - 1.0, -4.0), feed: spec.feed, role: MoveRole::Cut, power: None }),
        spindle_off(),
    ];
    let main_path = ClPath::try_new(Some("main_face".into()), Some(1), CompensationMode::Off, feeds(), None, Vec::new(), main_blocks).expect("main");
    let sub_path = ClPath::try_new(Some("sub_face".into()), Some(2), CompensationMode::Off, feeds(), None, Vec::new(), sub_blocks).expect("sub");
    let spindles = vec![Spindle::try_new("main", None, 0.0, 3_000.0).expect("main"), Spindle::try_new("sub", None, 0.0, 2_500.0).expect("sub")];
    let head = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("head");
    let bindings = vec![
        TurnBinding::try_new(0, "main", "turret1", 0).expect("b0").with_speed(Some(SpindleSpeed::rpm(800.0).expect("rpm"))).expect("s0"),
        TurnBinding::try_new(1, "sub", "turret1", 0).expect("b1").with_speed(Some(SpindleSpeed::rpm(700.0).expect("rpm"))).expect("s1"),
    ];
    let setup = TurningSetup::try_new(spindles, vec![head], None, Vec::new(), bindings).expect("turning");
    let program = program_with("turn", vec![tool_row(1), tool_row(2)], vec![main_path, sub_path], setup);
    let moves = vec![
        Expect {
            end: [spec.main_r, 0.0, 1.0],
            rapid: true,
            feed: 0.0,
            lead: None,
            arc: None,
            tool: 1,
            live: false,
            c: None,
            c_word: false,
            on_sub: false,
            brake: false,
        },
        Expect {
            end: [spec.main_r, 0.0, -6.0],
            rapid: false,
            feed: spec.feed,
            lead: None,
            arc: None,
            tool: 1,
            live: false,
            c: None,
            c_word: false,
            on_sub: false,
            brake: false,
        },
        Expect {
            end: [spec.sub_r, 0.0, 0.5],
            rapid: true,
            feed: 0.0,
            lead: None,
            arc: None,
            tool: 2,
            live: false,
            c: None,
            c_word: false,
            on_sub: true,
            brake: false,
        },
        Expect {
            end: [spec.sub_r - 1.0, 0.0, -4.0],
            rapid: false,
            feed: spec.feed,
            lead: None,
            arc: None,
            tool: 2,
            live: false,
            c: None,
            c_word: false,
            on_sub: true,
            brake: false,
        },
    ];
    Built { program, moves }
}

fn check_sub(spec: &SubSpec) -> Result<(), TestCaseError> {
    let built = build_sub(spec);
    let id = "haas_lathe";
    replay(&built.program, id, &built.moves, false, false, true).map_err(|message| TestCaseError::fail(format!("{id}: {message}\n{spec:?}")))?;
    let posted = post(id);
    let text = post_program(&built.program, &posted, &options()).map_err(|err| TestCaseError::fail(format!("{err}")))?;
    let audit = audit_lathe(&text, &posted).map_err(|err| TestCaseError::fail(format!("{err}\n{text}")))?;
    finish(audit.on_sub, id, &text).map_err(TestCaseError::fail)?;
    if audit.motions.iter().any(|motion| motion.on_sub) && audit.motions.iter().any(|motion| !motion.on_sub) {
        let mut seen_sub = false;
        for motion in &audit.motions {
            if motion.on_sub {
                seen_sub = true;
            } else if seen_sub {
                return Err(TestCaseError::fail(format!("{id}: main move after a sub move\n{text}")));
            }
        }
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct ChanSpec {
    r0: f64,
    r1: f64,
    feed: f64,
    syncs: u8,
}

fn build_channels(spec: &ChanSpec) -> (Program, Vec<u32>) {
    let mut waits = Vec::new();
    let mut left = vec![ClBlock::Motion(ClMotion::Rapid { to: xz(spec.r0, 0.0), feed: None })];
    let mut right = vec![ClBlock::Motion(ClMotion::Rapid { to: xz(spec.r1, 0.0), feed: None })];
    for index in 1..=spec.syncs {
        let id = u32::from(index);
        waits.push(100 + id);
        left.push(ClBlock::Motion(ClMotion::Linear { to: xz(spec.r0, -f64::from(index)), feed: spec.feed, role: MoveRole::Cut, power: None }));
        right.push(ClBlock::Motion(ClMotion::Linear { to: xz(spec.r1, -f64::from(index) - 0.5), feed: spec.feed, role: MoveRole::Cut, power: None }));
        left.push(ClBlock::Event(Event::Sync { id }));
        right.push(ClBlock::Event(Event::Sync { id }));
    }
    let spindle = Spindle::try_new("main", None, 0.0, 3_000.0).expect("spindle");
    let upper = Head::try_new("turret1", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("upper");
    let lower = Head::try_new("lower", vec![HeadAxis::X, HeadAxis::Z], 12, false).expect("lower");
    let bindings = vec![
        TurnBinding::try_new(0, "main", "turret1", 0).expect("c0").with_speed(Some(SpindleSpeed::rpm(800.0).expect("rpm"))).expect("s0"),
        TurnBinding::try_new(1, "main", "lower", 1).expect("c1").with_speed(Some(SpindleSpeed::rpm(800.0).expect("rpm"))).expect("s1"),
    ];
    let setup = TurningSetup::try_new(vec![spindle], vec![upper, lower], None, Vec::new(), bindings).expect("turning");
    let paths = vec![
        ClPath::try_new(Some("upper".into()), None, CompensationMode::Off, feeds(), None, Vec::new(), left).expect("left"),
        ClPath::try_new(Some("lower".into()), None, CompensationMode::Off, feeds(), None, Vec::new(), right).expect("right"),
    ];
    let program = Program::try_new(Some("turn".into()), Vec::new(), None, paths, Vec::new(), Vec::new(), None)
        .expect("program")
        .with_turning(Some(setup))
        .expect("turning");
    (program, waits)
}

fn check_channels(spec: &ChanSpec) -> Result<(), TestCaseError> {
    let (program, waits) = build_channels(spec);
    let id = "generic_lathe_2ch";
    let posted = post(id);
    let text = match post_program(&program, &posted, &options()) {
        Ok(text) => text,
        Err(err) if tolerated(&err.to_string()) => return Ok(()),
        Err(err) => return Err(TestCaseError::fail(format!("{id}: {err}\n{spec:?}"))),
    };
    let audit = audit_lathe(&text, &posted).map_err(|err| TestCaseError::fail(format!("{id}: audit: {err}\n{text}")))?;
    compare(&program, &read_back(&text, &posted).map_err(|err| TestCaseError::fail(format!("{err}")))?, END_TOL, &posted)
        .map_err(|err| TestCaseError::fail(format!("{id}: compare: {err}\n{text}")))?;
    if audit.channels.len() != 2 {
        return Err(TestCaseError::fail(format!("{id}: channels {:?}\n{text}", audit.channels)));
    }
    if audit.channels[0].waits != waits || audit.channels[1].waits != waits {
        return Err(TestCaseError::fail(format!(
            "{id}: waits {:?} and {:?} want {waits:?}\n{text}",
            audit.channels[0].waits, audit.channels[1].waits
        )));
    }
    if audit.g96_without_cap {
        return Err(TestCaseError::fail(format!("{id}: G96 without a cap")));
    }
    for motion in &audit.motions {
        if motion.end[0] < -END_TOL {
            return Err(TestCaseError::fail(format!("{id}: line {}: radius below 0", motion.line)));
        }
        if !motion.rapid_like() && motion.feed_mode != LatheFeedMode::PerMinute {
            return Err(TestCaseError::fail(format!("{id}: line {}: feed mode {:?}", motion.line, motion.feed_mode)));
        }
    }
    Ok(())
}

trait RapidLike {
    fn rapid_like(&self) -> bool;
}

impl RapidLike for LatheMotion {
    fn rapid_like(&self) -> bool {
        self.mode == campanion_post::MotionMode::Rapid
    }
}

fn check_haas_accessories() -> Result<(), String> {
    let mut blocks = vec![
        ClBlock::Event(Event::Tailstock { engage: true }),
        ClBlock::Motion(ClMotion::Linear { to: xz(18.0, -8.0), feed: 80.0, role: MoveRole::Cut, power: None }),
        ClBlock::Event(Event::Tailstock { engage: false }),
        ClBlock::Event(Event::SteadyRest { clamp: true }),
        ClBlock::Motion(ClMotion::Linear { to: xz(16.0, -9.0), feed: 80.0, role: MoveRole::Cut, power: None }),
        ClBlock::Event(Event::SteadyRest { clamp: false }),
        ClBlock::Event(Event::PartsCatcher { extend: true }),
        ClBlock::Motion(ClMotion::Linear { to: xz(14.0, -10.0), feed: 80.0, role: MoveRole::Cut, power: None }),
        ClBlock::Event(Event::PartsCatcher { extend: false }),
        ClBlock::Event(Event::ChuckClamp { spindle: "sub".into(), clamp: false }),
        ClBlock::Event(Event::SpindleRun { spindle: "main".into(), rpm: 0.0, reverse: false }),
        ClBlock::Event(Event::SpindleRun { spindle: "sub".into(), rpm: 0.0, reverse: false }),
        ClBlock::Event(Event::SpindleSync { engage: true, phase_deg: None }),
        ClBlock::Event(Event::SpindleMove { spindle: "sub".into(), z: 5.0, machine: false, feed: None }),
        ClBlock::Event(Event::SpindleMove { spindle: "sub".into(), z: -2.0, machine: false, feed: Some(60.0) }),
        ClBlock::Event(Event::ChuckClamp { spindle: "sub".into(), clamp: true }),
        ClBlock::Motion(ClMotion::Rapid { to: xz(12.0, -1.0), feed: None }),
        ClBlock::Event(Event::SpindleSync { engage: false, phase_deg: None }),
        ClBlock::Event(Event::SpindleMove { spindle: "sub".into(), z: -80.0, machine: true, feed: None }),
        ClBlock::Event(Event::BarFeed { part_length: Some(30.0), min_clamp: Some(12.0) }),
    ];
    blocks.insert(0, ClBlock::Event(Event::ToolChange { tool_id: 1 }));
    blocks.insert(1, spindle_on());
    blocks.insert(2, ClBlock::Motion(ClMotion::Rapid { to: xz(18.0, 1.0), feed: None }));
    // Spindle is on for the tailstock cut. Stop it before the transfer, which stops both spindles itself.
    let off_at = blocks.iter().position(|block| matches!(block, ClBlock::Event(Event::ChuckClamp { .. }))).expect("chuck");
    blocks.insert(off_at, spindle_off());
    let setup = turning_setup(false, false);
    let program = one_path("turn", 1, blocks, setup);
    let id = "haas_lathe";
    let posted = post(id);
    let text = post_program(&program, &posted, &options()).map_err(|err| format!("post: {err}"))?;
    let audit = audit_lathe(&text, &posted).map_err(|err| format!("audit: {err}\n{text}"))?;
    compare(&program, &read_back(&text, &posted).map_err(|err| format!("read: {err}"))?, END_TOL, &posted)
        .map_err(|err| format!("compare: {err}\n{text}"))?;
    let tail = audit.motions.iter().find(|motion| near3(motion.end, [18.0, 0.0, -8.0], END_TOL)).ok_or("missing tailstock cut")?;
    if !tail.tailstock {
        return Err(format!("tailstock cut is not marked advanced\n{text}"));
    }
    let steady = audit.motions.iter().find(|motion| near3(motion.end, [16.0, 0.0, -9.0], END_TOL)).ok_or("missing steady cut")?;
    if !steady.steady_rest || steady.tailstock {
        return Err(format!("steady rest cut flags are wrong\n{text}"));
    }
    let caught = audit.motions.iter().find(|motion| near3(motion.end, [14.0, 0.0, -10.0], END_TOL)).ok_or("missing catcher cut")?;
    if !caught.parts_catcher || caught.steady_rest {
        return Err(format!("catcher cut flags are wrong\n{text}"));
    }
    if audit.tailstock || audit.steady_rest || audit.parts_catcher {
        return Err(format!("accessory left engaged\n{text}"));
    }
    if !audit.bar_feed {
        return Err(format!("bar feed was not seen\n{text}"));
    }
    if audit.housing.len() < 3 {
        return Err(format!("housing moves {:?}\n{text}", audit.housing));
    }
    let zs: Vec<f64> = audit.housing.iter().map(|move_| move_.z).collect();
    for want in [5.0, -2.0, -80.0] {
        if !zs.iter().any(|z| (z - want).abs() < 1e-6) {
            return Err(format!("housing positions {zs:?} missing {want}\n{text}"));
        }
    }
    if !audit.housing.iter().any(|move_| move_.machine) {
        return Err(format!("machine housing move missing\n{text}"));
    }
    Ok(())
}

/// Diameter-mode ZX arcs, including short ones and quarters that end on the axis.
/// The posted centre has to sweep the program way. A long-way centre still compares when the
/// chord is long, so this check is the one that sees it.
#[test]
fn diameter_arcs_keep_the_short_sweep() {
    let feed = 80.0;
    for &radius in &[0.8_f64, 1.25, 3.0, 10.0, 24.0] {
        for &sweep in &[0.015, 0.25, TAU / 4.0, 2.4, std::f64::consts::PI - 0.03] {
            for &phase in &[0.4, TAU / 4.0, 2.2, 4.2] {
                for ccw in [false, true] {
                    let end_phase = if ccw { phase + sweep } else { phase - sweep };
                    let cx = radius;
                    let start_x = radius * (1.0 + phase.sin());
                    let start_z = radius * phase.cos();
                    let end_x = radius * (1.0 + end_phase.sin());
                    let end_z = radius * end_phase.cos();
                    // A quarter that finishes on the axis: centre at Z, radius along +X.
                    let (start, end, center, used_sweep) = if (sweep - TAU / 4.0).abs() < 1e-9 && (phase - TAU / 4.0).abs() < 1e-9 && ccw {
                        (xz(radius, 0.0), xz(0.0, -radius), Vec3::new(0.0, 0.0, 0.0).expect("centre"), TAU / 4.0)
                    } else {
                        (xz(start_x, start_z), xz(end_x, end_z), Vec3::new(cx, 0.0, 0.0).expect("centre"), sweep)
                    };
                    let blocks = vec![
                        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
                        spindle_on(),
                        ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
                        ClBlock::Motion(ClMotion::Arc {
                            to: end,
                            plane: Plane::Zx,
                            center,
                            sense: if ccw { ArcSense::Ccw } else { ArcSense::Cw },
                            feed,
                            role: MoveRole::Cut,
                            power: None,
                        }),
                        spindle_off(),
                    ];
                    let program = one_path("arc", 1, blocks, turning_setup(false, false));
                    for id in TURN_POSTS {
                        let posted = post(id);
                        let text = match post_program(&program, &posted, &options()) {
                            Ok(text) => text,
                            Err(err) if tolerated(&err.to_string()) => continue,
                            Err(err) => panic!("{id}: {err}"),
                        };
                        compare(&program, &read_back(&text, &posted).expect("read"), END_TOL, &posted)
                            .unwrap_or_else(|err| panic!("{id}: compare: {err}\n{text}"));
                        let audit = audit_lathe(&text, &posted).unwrap_or_else(|err| panic!("{id}: audit: {err}\n{text}"));
                        let Some(index) = audit.motions.iter().position(|motion| motion.arc.is_some()) else {
                            continue;
                        };
                        if index == 0 {
                            panic!("{id}: arc has no start\n{text}");
                        }
                        let motion = &audit.motions[index];
                        let Some(arc) = motion.arc else { continue };
                        let start_tip = audit.motions[index - 1].end;
                        let swept = controller_sweep(start_tip, motion.end, arc.center, Plane::Zx, arc.sense);
                        let q = quantum(posted.linear_decimals());
                        let (u, v) = uv(Plane::Zx, start_tip);
                        let (cu, cv) = uv(Plane::Zx, arc.center);
                        let posted_r = (u - cu).hypot(v - cv).max(1e-6);
                        let allowance = 4.0 * q / posted_r + 1e-3;
                        assert!(
                            swept <= used_sweep + allowance,
                            "{id}: sweep {swept} program {used_sweep} allowance {allowance} radius {radius}\n{text}"
                        );
                    }
                }
            }
        }
    }
}

fn audit_config(cases: u32) -> ProptestConfig {
    ProptestConfig { cases, max_shrink_iters: 80, rng_seed: RngSeed::Fixed(0x00C2_2B02), ..ProptestConfig::default() }
}

fn turn_strategy() -> impl Strategy<Value = TurnSpec> {
    (
        any::<bool>(),
        any::<bool>(),
        1u32..=2,
        12.0..36.0,
        0.35..1.05,
        any::<bool>(),
        0.8..3.0,
        0.8..4.0,
        any::<bool>(),
        0.5..2.5,
        any::<bool>(),
        20.0..280.0,
    )
        .prop_map(|(css, reversed, tool, start_r, sweep, ccw, groove_depth, groove_width, thread, lead, past_centre, feed)| TurnSpec {
            css,
            reversed,
            tool,
            start_r,
            sweep,
            ccw,
            groove_depth,
            groove_width,
            thread: thread && !css,
            lead,
            past_centre,
            feed,
        })
}

fn live_strategy() -> impl Strategy<Value = LiveSpec> {
    (0u8..4, 0.0..120.0, 140.0..300.0, prop_oneof![Just(2.5), Just(-3.5), Just(5.0), Just(-1.5)], 30.0..200.0).prop_map(|(kind, c0, c1, y, feed)| {
        let kind = match kind {
            0 => LiveKind::IndexedFront,
            1 => LiveKind::IndexedOd,
            2 => LiveKind::Face,
            _ => LiveKind::Cylinder,
        };
        LiveSpec { kind, c0, c1, y, feed }
    })
}

fn sub_strategy() -> impl Strategy<Value = SubSpec> {
    (8.0..30.0, 8.0..24.0, 20.0..200.0).prop_map(|(main_r, sub_r, feed): (f64, f64, f64)| SubSpec { main_r, sub_r, feed })
}

fn chan_strategy() -> impl Strategy<Value = ChanSpec> {
    (6.0..28.0, 4.0..18.0, 20.0..200.0, 1u8..=3).prop_map(|(r0, r1, feed, syncs)| ChanSpec { r0, r1, feed, syncs })
}

#[test]
fn smoke_turning_on_every_lathe_post() {
    let spec = TurnSpec {
        css: true,
        reversed: false,
        tool: 1,
        start_r: 20.0,
        sweep: 0.7,
        ccw: true,
        groove_depth: 1.5,
        groove_width: 2.0,
        thread: false,
        lead: 1.5,
        past_centre: true,
        feed: 100.0,
    };
    check_turn(&spec).unwrap_or_else(|err| panic!("{err}"));
    let threaded = TurnSpec { css: false, thread: true, past_centre: false, reversed: true, tool: 2, ..spec };
    check_turn(&threaded).unwrap_or_else(|err| panic!("{err}"));
}

#[test]
fn smoke_live_indexed_and_interpolated() {
    for kind in [LiveKind::IndexedFront, LiveKind::IndexedOd, LiveKind::Face, LiveKind::Cylinder] {
        let spec = LiveSpec { kind, c0: 0.0, c1: 90.0, y: 4.0, feed: 100.0 };
        check_live(&spec).unwrap_or_else(|err| panic!("{err}"));
    }
}

#[test]
fn smoke_sub_spindle_restores_the_main() {
    check_sub(&SubSpec { main_r: 14.0, sub_r: 11.0, feed: 90.0 }).unwrap_or_else(|err| panic!("{err}"));
}

#[test]
fn smoke_haas_transfer_tailstock_steady_catcher_and_bar_feed() {
    check_haas_accessories().unwrap_or_else(|err| panic!("{err}"));
}

#[test]
fn smoke_two_channel_waits_match() {
    check_channels(&ChanSpec { r0: 12.0, r1: 6.0, feed: 80.0, syncs: 2 }).unwrap_or_else(|err| panic!("{err}"));
}

#[test]
fn audit_lathe_rejects_a_mill_post() {
    let err = audit_lathe("G21\nG0 X0 Y0 Z0\n", &post("linuxcnc")).expect_err("mill");
    assert!(err.to_string().contains("audit_lathe reads lathe posts"), "{err}");
}

proptest! {
    #![proptest_config(audit_config(32))]
    #[test]
    fn random_turning_programs(spec in turn_strategy()) {
        check_turn(&spec)?;
    }
}

proptest! {
    #![proptest_config(audit_config(24))]
    #[test]
    fn random_live_cycles(spec in live_strategy()) {
        check_live(&spec)?;
    }
}

proptest! {
    #![proptest_config(audit_config(16))]
    #[test]
    fn random_sub_spindle_work(spec in sub_strategy()) {
        check_sub(&spec)?;
    }
}

proptest! {
    #![proptest_config(audit_config(12))]
    #[test]
    fn random_two_channel_waits(spec in chan_strategy()) {
        check_channels(&spec)?;
    }
}
