//! Modal replay of random mill programs.
//!
//! Each case is posted with every mill-family post, read back, and checked against the IR.
//! A posted feed that the post rejects on purpose (a radius that will not fit, a feed that
//! rounds to zero) is skipped. Any other failure is a post or checker bug.

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, CoolantMode, Event, Feeds, MoveRole, Plane, Pose, ProcessKind, ProcessParams, Program,
    RotaryAxisKind, RotaryDirection, RotarySetup, Tool, Vec3, machine_to_part,
};
use campanion_post::{
    CoolantState, DistanceMode, FeedMode, LengthOffset, MotionMode, MotionState, PostOptions, UnitMode, audit, builtin, compare, post_program,
    read_back,
};
use proptest::prelude::*;
use proptest::test_runner::{RngSeed, TestCaseError};

const TAU: f64 = std::f64::consts::TAU;
const ARC_TOL: f64 = 0.05;

#[derive(Clone, Copy, Debug)]
enum LengthKind {
    Unset,
    Cancelled,
    Active,
}

struct Mill {
    id: &'static str,
    per_cut: bool,
    laser: bool,
    drop_z: bool,
    rapid_z: bool,
    numbered: bool,
    decimal_point: bool,
    length: LengthKind,
}

const MILLS: &[Mill] = &[
    Mill {
        id: "linuxcnc",
        per_cut: false,
        laser: false,
        drop_z: false,
        rapid_z: true,
        numbered: true,
        decimal_point: false,
        length: LengthKind::Cancelled,
    },
    Mill { id: "grbl", per_cut: false, laser: false, drop_z: false, rapid_z: true, numbered: false, decimal_point: false, length: LengthKind::Unset },
    Mill {
        id: "mach3",
        per_cut: false,
        laser: false,
        drop_z: false,
        rapid_z: true,
        numbered: false,
        decimal_point: false,
        length: LengthKind::Cancelled,
    },
    Mill { id: "fanuc", per_cut: false, laser: false, drop_z: false, rapid_z: true, numbered: true, decimal_point: true, length: LengthKind::Active },
    Mill { id: "haas", per_cut: false, laser: false, drop_z: false, rapid_z: true, numbered: true, decimal_point: true, length: LengthKind::Active },
    Mill {
        id: "plasma",
        per_cut: true,
        laser: false,
        drop_z: false,
        rapid_z: true,
        numbered: true,
        decimal_point: false,
        length: LengthKind::Cancelled,
    },
    Mill {
        id: "grbl_laser",
        per_cut: true,
        laser: true,
        drop_z: true,
        rapid_z: false,
        numbered: false,
        decimal_point: false,
        length: LengthKind::Unset,
    },
];

#[derive(Clone, Debug)]
enum CutKind {
    Line { x: f64, y: f64, z: f64 },
    Arc { plane: Plane, radius: f64, sweep: f64, ccw: bool, axial: f64 },
    Helix { plane: Plane, radius: f64, sweep: f64, ccw: bool, axial: f64 },
}

#[derive(Clone, Debug)]
struct Cut {
    kind: CutKind,
    feed: f64,
}

#[derive(Clone, Debug)]
struct PathSpec {
    tool: u32,
    speed: f64,
    power: f64,
    coolant: u8,
    dwell_s: Option<f64>,
    approach: [f64; 3],
    cuts: Vec<Cut>,
    second_speed: Option<f64>,
    extra: Vec<Cut>,
    depart: bool,
}

#[derive(Clone, Debug)]
struct Spec {
    paths: Vec<PathSpec>,
}

fn pose_at(x: f64, y: f64, z: f64) -> Option<Pose> {
    let tip = Vec3::new(x, y, z).ok()?;
    Pose::new(tip, Vec3::Z).ok()
}

fn tip_of(pose: Pose) -> [f64; 3] {
    let tip = pose.tip();
    [tip.x(), tip.y(), tip.z()]
}

fn build(spec: &Spec) -> Option<Program> {
    let tools = (1..=3).map(|id| Tool::try_new(id, Some(format!("t{id}")), Some(6.0), None).ok()).collect::<Option<Vec<_>>>()?;
    let mut paths = Vec::new();
    for path in &spec.paths {
        let mut blocks = Vec::new();
        blocks.push(ClBlock::Event(Event::ToolChange { tool_id: path.tool }));
        let approach = pose_at(path.approach[0], path.approach[1], path.approach[2])?;
        blocks.push(ClBlock::Motion(ClMotion::Rapid { to: approach, feed: None }));
        match path.coolant {
            1 => blocks.push(ClBlock::Event(Event::Coolant { mode: CoolantMode::Flood })),
            2 => blocks.push(ClBlock::Event(Event::Coolant { mode: CoolantMode::Mist })),
            3 => blocks.push(ClBlock::Event(Event::Coolant { mode: CoolantMode::Off })),
            _ => {}
        }
        let params = ProcessParams::new().speed(Some(path.speed)).ok()?.power(Some(path.power)).ok()?;
        blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }));
        if let Some(seconds) = path.dwell_s {
            blocks.push(ClBlock::Event(Event::Dwell { seconds }));
        }
        let mut pos = path.approach;
        pos = push_cuts(&mut blocks, pos, &path.cuts)?;
        if let Some(speed) = path.second_speed {
            let params = ProcessParams::new().speed(Some(speed)).ok()?.power(Some(path.power)).ok()?;
            blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }));
        }
        pos = push_cuts(&mut blocks, pos, &path.extra)?;
        blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }));
        if path.depart {
            let depart = pose_at(0.0, 0.0, pos[2] + 5.0)?;
            blocks.push(ClBlock::Motion(ClMotion::Rapid { to: depart, feed: None }));
        }
        let feeds = Feeds::try_new(100.0, None, None, None, None).ok()?;
        let built = ClPath::try_new(Some("prop".into()), Some(path.tool), CompensationMode::Off, feeds, None, Vec::new(), blocks).ok()?;
        paths.push(built);
    }
    Program::try_new(Some("prop".into()), tools, None, paths, Vec::new(), Vec::new(), None).ok()
}

fn push_cuts(blocks: &mut Vec<ClBlock>, mut pos: [f64; 3], cuts: &[Cut]) -> Option<[f64; 3]> {
    for cut in cuts {
        let (motion, end) = motion_from(pos, cut)?;
        blocks.push(ClBlock::Motion(motion));
        pos = end;
    }
    Some(pos)
}

fn motion_from(pos: [f64; 3], cut: &Cut) -> Option<(ClMotion, [f64; 3])> {
    match cut.kind {
        CutKind::Line { x, y, z } => {
            let end = [x, y, z];
            let motion = ClMotion::Linear { to: pose_at(x, y, z)?, feed: cut.feed, role: MoveRole::Cut };
            Some((motion, end))
        }
        CutKind::Arc { plane, radius, sweep, ccw, axial } => arc_motion(pos, plane, radius, sweep, ccw, axial, cut.feed, false),
        CutKind::Helix { plane, radius, sweep, ccw, axial } => arc_motion(pos, plane, radius, sweep, ccw, axial, cut.feed, true),
    }
}

fn arc_motion(pos: [f64; 3], plane: Plane, radius: f64, sweep: f64, ccw: bool, axial: f64, feed: f64, helix: bool) -> Option<(ClMotion, [f64; 3])> {
    if !(radius.is_finite() && radius > 1e-6 && sweep.is_finite() && sweep > 0.0 && sweep <= TAU + 1e-9) {
        return None;
    }
    let signed = if ccw { sweep.min(TAU) } else { -sweep.min(TAU) };
    let (sin_a, cos_a) = signed.sin_cos();
    let mut end = match plane {
        Plane::Xy => [pos[0] - radius + radius * cos_a, pos[1] + radius * sin_a, pos[2] + axial],
        Plane::Zx => [pos[0] + radius * sin_a, pos[1] + axial, pos[2] - radius + radius * cos_a],
        Plane::Yz => [pos[0] + axial, pos[1] - radius + radius * cos_a, pos[2] + radius * sin_a],
    };
    let center = match plane {
        Plane::Xy => [pos[0] - radius, pos[1], pos[2]],
        Plane::Zx => [pos[0], pos[1], pos[2] - radius],
        Plane::Yz => [pos[0], pos[1] - radius, pos[2]],
    };
    if sweep >= TAU - 1e-6 {
        match plane {
            Plane::Xy => {
                end[0] = pos[0];
                end[1] = pos[1];
            }
            Plane::Zx => {
                end[2] = pos[2];
                end[0] = pos[0];
            }
            Plane::Yz => {
                end[1] = pos[1];
                end[2] = pos[2];
            }
        }
    }
    let sense = if ccw { ArcSense::Ccw } else { ArcSense::Cw };
    let center = Vec3::new(center[0], center[1], center[2]).ok()?;
    let to = pose_at(end[0], end[1], end[2])?;
    let motion = if helix {
        ClMotion::Helix { to, plane, center, sense, sweep, feed, role: MoveRole::Cut }
    } else {
        ClMotion::Arc { to, plane, center, sense, feed, role: MoveRole::Cut }
    };
    Some((motion, end))
}

fn options() -> PostOptions {
    PostOptions { arc_tolerance: ARC_TOL }
}

fn tolerated(err: &str) -> bool {
    const NEEDLES: &[&str] = &[
        "arc end rounds onto its start",
        "arc radius",
        "arc end radius",
        "arc end does not match",
        "arc is not",
        "arc linearises",
        "arc splits into too many",
        "arc offset",
        "feed rounds to zero",
        "feed must be positive",
        "number is too large",
        "not finite",
        "helix has more than",
        "too many segments",
        "line exceeds max_line_length",
        "inverse time feed",
        "z_mode drop",
    ];
    NEEDLES.iter().any(|needle| err.contains(needle))
}

fn check_all(program: &Program, spec: &Spec) -> Result<(), TestCaseError> {
    for mill in MILLS {
        let Some(post) = builtin(mill.id) else {
            return Err(TestCaseError::fail(format!("missing post {}", mill.id)));
        };
        if let Err(message) = check_one(program, &post, mill) {
            let text = post_program(program, &post, &options()).unwrap_or_default();
            let snippet: String = text.chars().take(2000).collect();
            return Err(TestCaseError::fail(format!("{}: {message}\n{spec:?}\n{snippet}", mill.id)));
        }
    }
    Ok(())
}

fn check_one(program: &Program, post: &campanion_post::PostDef, mill: &Mill) -> Result<(), String> {
    let text = match post_program(program, post, &options()) {
        Ok(text) => text,
        Err(err) if tolerated(&err.to_string()) => return Ok(()),
        Err(err) => return Err(format!("post: {err}")),
    };
    check_text(&text, mill, post.max_line_length())?;
    let report = audit(&text, post).map_err(|err| format!("audit: {err}\n{text}"))?;
    // A short following arc ends inside the chord tolerance, and `compare` then treats it as
    // another block of the previous arc. Those programs stay on the replay matcher, which checks
    // each chord against `ARC_TOL`. Longer arcs go through `compare` as well.
    if chords_outside_tolerance(program) {
        let moves = read_back(&text, post).map_err(|err| format!("read_back: {err}"))?;
        compare(program, &moves, ARC_TOL, post).map_err(|err| format!("compare: {err}\n{text}"))?;
    }
    check_states(program, &report, mill, post)?;
    if report.spindle.on {
        return Err("spindle is still on at program end".into());
    }
    if report.coolant != CoolantState::Off {
        return Err("coolant is still on at program end".into());
    }
    Ok(())
}

fn chords_outside_tolerance(program: &Program) -> bool {
    for path in program.toolpaths() {
        let mut prev: Option<[f64; 3]> = None;
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else { continue };
            let end = tip_of(pose_of(motion));
            if let Some(start) = prev {
                let full_turn = match motion {
                    ClMotion::Arc { plane, center, sense, .. } | ClMotion::Helix { plane, center, sense, .. } => {
                        let center = [center.x(), center.y(), center.z()];
                        sweep_of(start, end, center, *plane, *sense) >= TAU - 1e-3
                    }
                    _ => false,
                };
                let chord = (end[0] - start[0]).hypot(end[1] - start[1]).hypot(end[2] - start[2]);
                if !full_turn && chord <= ARC_TOL {
                    return false;
                }
            }
            prev = Some(end);
        }
    }
    true
}

fn check_text(text: &str, mill: &Mill, max_line: Option<usize>) -> Result<(), String> {
    if let Some(limit) = max_line {
        for (index, line) in text.lines().enumerate() {
            if line.chars().count() > limit {
                return Err(format!("line {} exceeds max_line_length {limit}", index + 1));
            }
        }
    }
    for token in text.split_whitespace() {
        let number = token.trim_start_matches(|ch: char| ch.is_ascii_alphabetic());
        if negative_zero(number) {
            return Err(format!("negative zero in {token}"));
        }
        if mill.decimal_point {
            let mut chars = token.chars();
            if let Some(letter) = chars.next()
                && matches!(letter, 'X' | 'Y' | 'Z' | 'I' | 'J' | 'K' | 'R' | 'A' | 'B')
            {
                let rest: String = chars.collect();
                if !rest.is_empty() && !rest.contains('.') {
                    return Err(format!("dimensional word {token} is missing a decimal point"));
                }
            }
        }
    }
    if mill.numbered {
        let mut expect = 10u64;
        for line in text.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed == "%" || is_program_number(trimmed) {
                continue;
            }
            let Some(rest) = trimmed.strip_prefix('N') else {
                return Err(format!("line is missing its sequence number: {trimmed}"));
            };
            let digits: String = rest.chars().take_while(|ch| ch.is_ascii_digit()).collect();
            if digits.is_empty() {
                return Err(format!("line is missing its sequence number: {trimmed}"));
            }
            let number: u64 = digits.parse().map_err(|_| format!("bad line number {digits}"))?;
            if number != expect {
                return Err(format!("line number {number} is not {expect}"));
            }
            if number > 1_000_000_000 {
                return Err("line number is above 1000000000".into());
            }
            expect += 10;
        }
    } else {
        for line in text.lines() {
            let trimmed = line.trim();
            if let Some(rest) = trimmed.strip_prefix('N')
                && rest.chars().next().is_some_and(|ch| ch.is_ascii_digit())
            {
                return Err(format!("unexpected line number: {trimmed}"));
            }
        }
    }
    Ok(())
}

fn is_program_number(trimmed: &str) -> bool {
    let Some(digits) = trimmed.strip_prefix('O') else {
        return false;
    };
    !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit())
}

fn negative_zero(raw: &str) -> bool {
    let Some(rest) = raw.strip_prefix('-') else {
        return false;
    };
    !rest.is_empty() && rest.bytes().all(|byte| byte == b'0' || byte == b'.')
}

struct Live {
    tool: Option<u32>,
    cutting: bool,
    speed: f64,
    power: f64,
    coolant: CoolantState,
}

fn check_states(program: &Program, report: &campanion_post::AuditReport, mill: &Mill, post: &campanion_post::PostDef) -> Result<(), String> {
    let decimals = post.linear_decimals();
    let end_tol = quantum(decimals) * 0.51 + 1e-9;
    let mut cursor = 0usize;
    let mut prev = [0.0, 0.0, 0.0];
    let mut posted = [0.0, 0.0, 0.0];
    let mut live = Live { tool: None, cutting: false, speed: 0.0, power: 0.0, coolant: CoolantState::Off };
    let mut seen_feed_word = false;
    for state in &report.motions {
        if matches!(state.mode, MotionMode::Linear | MotionMode::Cw | MotionMode::Ccw) && !seen_feed_word && !state.feed_on_block {
            return Err(format!("line {}: feed move before the first F", state.line));
        }
        if state.feed_on_block {
            seen_feed_word = true;
        }
        if state.feed_mode == FeedMode::InverseTime
            && matches!(state.mode, MotionMode::Linear | MotionMode::Cw | MotionMode::Ccw)
            && !state.feed_on_block
        {
            return Err(format!("line {}: inverse-time block has no F", state.line));
        }
    }
    for path in program.toolpaths() {
        for block in path.blocks() {
            match block {
                ClBlock::Event(Event::ToolChange { tool_id }) => live.tool = Some(*tool_id),
                ClBlock::Event(Event::ProcessOn { params, .. }) => {
                    live.cutting = true;
                    if let Some(speed) = params.speed_value() {
                        live.speed = speed;
                    }
                    if let Some(power) = params.power_value() {
                        live.power = power;
                    }
                }
                ClBlock::Event(Event::ProcessOff { .. }) => live.cutting = false,
                ClBlock::Event(Event::Coolant { mode }) => {
                    live.coolant = match mode {
                        CoolantMode::Flood => CoolantState::Flood,
                        CoolantMode::Mist => CoolantState::Mist,
                        _ => CoolantState::Off,
                    };
                }
                ClBlock::Event(_) => {}
                ClBlock::Motion(motion) => {
                    let end = tip_of(pose_of(motion));
                    let rapid = matches!(motion, ClMotion::Rapid { .. });
                    match motion {
                        ClMotion::Rapid { .. } | ClMotion::Linear { .. } => {
                            if omitted_linear(prev, end, rapid, mill, decimals) {
                                prev = end;
                                continue;
                            }
                            let state = report.motions.get(cursor).ok_or_else(|| format!("missing posted block for {end:?}"))?;
                            cursor += 1;
                            check_modal(state, &live, rapid, mill, path.tool_id())?;
                            check_feed(state, motion, post)?;
                            if state.mode != if rapid { MotionMode::Rapid } else { MotionMode::Linear } {
                                return Err(format!(
                                    "line {}: motion mode {:?} for a {}",
                                    state.line,
                                    state.mode,
                                    if rapid { "rapid" } else { "line" }
                                ));
                            }
                            if !near(state.end, end, end_tol, mill.drop_z) {
                                return Err(format!("line {}: end {:?} is not {:?}", state.line, state.end, end));
                            }
                            posted = state.end;
                        }
                        ClMotion::Arc { plane, center, sense, .. } | ClMotion::Helix { plane, center, sense, .. } => {
                            let center = [center.x(), center.y(), center.z()];
                            let sweep = sweep_of(prev, end, center, *plane, *sense);
                            let radius = in_plane(prev, center, *plane);
                            // The writer drops an arc whose rounded ends meet when the sagitta
                            // fits in the chord tolerance. Two exact points can sit almost a
                            // full quantum apart and still round to the same word.
                            if collapsed(prev, end, sweep, decimals, mill.drop_z)
                                && !report.motions.get(cursor).is_some_and(|state| {
                                    near(state.end, end, end_tol, mill.drop_z) && (in_plane(state.end, center, *plane) - radius).abs() <= ARC_TOL
                                })
                            {
                                prev = end;
                                continue;
                            }
                            cursor = consume_curve(
                                cursor,
                                report,
                                posted,
                                end,
                                center,
                                *plane,
                                *sense,
                                sweep,
                                &live,
                                rapid,
                                mill,
                                post,
                                path.tool_id(),
                                motion,
                            )?;
                            posted = report.motions.get(cursor - 1).map(|state| state.end).unwrap_or(end);
                        }
                        ClMotion::Thread { .. } => return Err("mill post was given a thread".into()),
                    }
                    prev = end;
                }
            }
        }
    }
    if cursor != report.motions.len() {
        return Err(format!("posted {} motions but matched {cursor}", report.motions.len()));
    }
    Ok(())
}

fn consume_curve(
    mut cursor: usize,
    report: &campanion_post::AuditReport,
    mut at: [f64; 3],
    end: [f64; 3],
    center: [f64; 3],
    plane: Plane,
    sense: ArcSense,
    sweep: f64,
    live: &Live,
    rapid: bool,
    mill: &Mill,
    post: &campanion_post::PostDef,
    path_tool: Option<u32>,
    motion: &ClMotion,
) -> Result<usize, String> {
    let end_tol = quantum(post.linear_decimals()) * 0.51 + 1e-9;
    let radius = in_plane(at, center, plane);
    let mut acc = 0.0;
    let mut guard = 0u32;
    loop {
        let state = report.motions.get(cursor).ok_or_else(|| format!("g-code ended before the arc to {end:?}"))?;
        cursor += 1;
        guard += 1;
        if guard > 10_000 {
            return Err("arc used more than 10000 blocks".into());
        }
        check_modal(state, live, rapid, mill, path_tool)?;
        check_feed(state, motion, post)?;
        let chord = in_plane(at, state.end, plane);
        let travel = match state.mode {
            MotionMode::Rapid => return Err(format!("line {}: arc was posted as a rapid", state.line)),
            MotionMode::Cw | MotionMode::Ccw => {
                let arc = state.arc.ok_or_else(|| format!("line {}: arc block has no centre", state.line))?;
                if arc.sense != sense {
                    return Err(format!("line {}: arc sense differs", state.line));
                }
                if state.plane != Some(plane) {
                    return Err(format!("line {}: plane {:?} is not {plane:?}", state.line, state.plane));
                }
                if arc.radius.is_none() {
                    let gap = in_plane(arc.center, center, plane);
                    if gap > end_tol * 4.0 + 1e-9 {
                        return Err(format!("line {}: arc centre differs by {gap}", state.line));
                    }
                }
                if chord <= end_tol.max(1e-9) && sweep < TAU - 0.2 {
                    return Err(format!("line {}: arc block has equal endpoints and is not a full turn", state.line));
                }
                if chord <= end_tol.max(1e-9) && sweep >= TAU - 1e-3 { TAU } else { angle_travel(at, state.end, center, plane, sense) }
            }
            MotionMode::Linear => {
                if (in_plane(state.end, center, plane) - radius).abs() > ARC_TOL {
                    return Err(format!("line {}: chord is off the arc", state.line));
                }
                if !mill.drop_z && !axial_ok(state.end, at, end, plane, ARC_TOL) {
                    return Err(format!("line {}: chord axial is off the helix", state.line));
                }
                angle_travel(at, state.end, center, plane, sense)
            }
        };
        // In-plane ends that round together become one line when the sagitta fits in the
        // chord tolerance. That line does not accumulate the programmed sweep.
        let collapsed_line =
            state.mode == MotionMode::Linear && sweep <= std::f64::consts::PI + 1e-6 && same_in_plane(at, state.end, plane, post.linear_decimals());
        // A radius of a few output quanta cannot resolve the programmed sweep. The end
        // point is still required to land on the rounded target.
        let tiny = radius <= quantum(post.linear_decimals()) * 3.0 && sweep <= std::f64::consts::PI + 1e-6;
        acc += travel;
        at = state.end;
        if near(state.end, end, end_tol, mill.drop_z) && (acc + 0.15 >= sweep || collapsed_line || tiny) {
            return Ok(cursor);
        }
    }
}

fn check_modal(state: &MotionState, live: &Live, rapid: bool, mill: &Mill, path_tool: Option<u32>) -> Result<(), String> {
    if state.units != UnitMode::Millimetre {
        return Err(format!("line {}: units are not millimetres", state.line));
    }
    if state.distance != DistanceMode::Absolute {
        return Err(format!("line {}: distance mode is not absolute", state.line));
    }
    if state.work.is_some() {
        return Err(format!("line {}: work offset {:?} was not in the program", state.line, state.work));
    }
    if state.tool != live.tool || state.tool != path_tool {
        return Err(format!("line {}: tool {:?} is not {:?} / {path_tool:?}", state.line, state.tool, live.tool));
    }
    let length_ok = match mill.length {
        LengthKind::Unset => state.length == LengthOffset::Unset,
        LengthKind::Cancelled => state.length == LengthOffset::Cancelled,
        LengthKind::Active => live.tool.is_some_and(|tool| state.length == LengthOffset::Active(tool)),
    };
    if !length_ok {
        return Err(format!("line {}: tool length {:?} does not match the post", state.line, state.length));
    }
    if state.coolant != live.coolant {
        return Err(format!("line {}: coolant {:?} is not {:?}", state.line, state.coolant, live.coolant));
    }
    let want_cutting = if rapid && mill.per_cut { false } else { live.cutting };
    if want_cutting {
        if !state.spindle.cutting() {
            return Err(format!("line {}: process is off during a cut", state.line));
        }
        let expect = if mill.laser { (live.power * 1000.0).round() } else { live.speed.round() };
        if (state.spindle.speed - expect).abs() > 0.51 {
            return Err(format!("line {}: S{} is not {expect}", state.line, state.spindle.speed));
        }
    } else if state.spindle.cutting() {
        return Err(format!("line {}: process is on during a rapid or before the process starts", state.line));
    }
    Ok(())
}

fn check_feed(state: &MotionState, motion: &ClMotion, post: &campanion_post::PostDef) -> Result<(), String> {
    let Some(ir_feed) = motion_feed(motion) else {
        return Ok(());
    };
    if state.feed_mode != FeedMode::PerMinute {
        return Err(format!("line {}: feed mode changed on a 3-axis move", state.line));
    }
    let Some(got) = state.feed else {
        return Err(format!("line {}: feed move has no F", state.line));
    };
    if got <= 0.0 {
        return Err(format!("line {}: feed {got} is not positive", state.line));
    }
    let limit = quantum(post.feed_decimals()) * 0.51 + ir_feed.abs() * 1e-12;
    if (got - ir_feed).abs() > limit {
        return Err(format!("line {}: feed {got} differs from {ir_feed}", state.line));
    }
    Ok(())
}

fn motion_feed(motion: &ClMotion) -> Option<f64> {
    match motion {
        ClMotion::Linear { feed, .. } | ClMotion::Arc { feed, .. } | ClMotion::Helix { feed, .. } => Some(*feed),
        ClMotion::Rapid { .. } | ClMotion::Thread { .. } => None,
    }
}

fn pose_of(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    }
}

fn omitted_linear(prev: [f64; 3], end: [f64; 3], rapid: bool, mill: &Mill, decimals: u8) -> bool {
    if !xy_same(prev, end, decimals) {
        return false;
    }
    (rapid && !mill.rapid_z) || mill.drop_z
}

fn collapsed(prev: [f64; 3], end: [f64; 3], sweep: f64, decimals: u8, drop_z: bool) -> bool {
    sweep <= std::f64::consts::PI + 1e-6 && same_rounded(prev, end, decimals, drop_z)
}

fn same_rounded(a: [f64; 3], b: [f64; 3], decimals: u8, drop_z: bool) -> bool {
    let axes = if drop_z { 2 } else { 3 };
    (0..axes).all(|index| rounded(a[index], decimals) == rounded(b[index], decimals))
}

fn same_in_plane(a: [f64; 3], b: [f64; 3], plane: Plane, decimals: u8) -> bool {
    let (au, av) = uv(plane, a);
    let (bu, bv) = uv(plane, b);
    rounded(au, decimals) == rounded(bu, decimals) && rounded(av, decimals) == rounded(bv, decimals)
}

fn xy_same(a: [f64; 3], b: [f64; 3], decimals: u8) -> bool {
    rounded(a[0], decimals) == rounded(b[0], decimals) && rounded(a[1], decimals) == rounded(b[1], decimals)
}

fn rounded(value: f64, decimals: u8) -> f64 {
    let scale = 10_f64.powi(i32::from(decimals));
    (value * scale).round() / scale
}

fn quantum(decimals: u8) -> f64 {
    10_f64.powi(-i32::from(decimals))
}

fn near(got: [f64; 3], expect: [f64; 3], tol: f64, drop_z: bool) -> bool {
    let axes = if drop_z { 2 } else { 3 };
    (0..axes).all(|index| (got[index] - expect[index]).abs() <= tol)
}

fn uv(plane: Plane, point: [f64; 3]) -> (f64, f64) {
    match plane {
        Plane::Xy => (point[0], point[1]),
        Plane::Zx => (point[2], point[0]),
        Plane::Yz => (point[1], point[2]),
    }
}

fn in_plane(a: [f64; 3], b: [f64; 3], plane: Plane) -> f64 {
    let (au, av) = uv(plane, a);
    let (bu, bv) = uv(plane, b);
    (au - bu).hypot(av - bv)
}

fn axial_ok(point: [f64; 3], start: [f64; 3], end: [f64; 3], plane: Plane, tol: f64) -> bool {
    let axial = match plane {
        Plane::Xy => point[2],
        Plane::Zx => point[1],
        Plane::Yz => point[0],
    };
    let start = match plane {
        Plane::Xy => start[2],
        Plane::Zx => start[1],
        Plane::Yz => start[0],
    };
    let end = match plane {
        Plane::Xy => end[2],
        Plane::Zx => end[1],
        Plane::Yz => end[0],
    };
    axial + tol >= start.min(end) && axial - tol <= start.max(end)
}

fn angle_travel(from: [f64; 3], to: [f64; 3], center: [f64; 3], plane: Plane, sense: ArcSense) -> f64 {
    let a0 = angle_of(from, center, plane);
    let a1 = angle_of(to, center, plane);
    forward_delta(a0, a1, sense == ArcSense::Ccw)
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

fn sweep_of(start: [f64; 3], end: [f64; 3], center: [f64; 3], plane: Plane, sense: ArcSense) -> f64 {
    let radius = in_plane(start, center, plane).abs();
    let chord = in_plane(start, end, plane);
    if chord <= 1e-9_f64.max(radius * 1e-9) {
        return TAU;
    }
    let delta = angle_travel(start, end, center, plane, sense);
    if delta <= 1e-12 { TAU } else { delta }
}

fn feed_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        2 => Just(0.0005),
        2 => Just(0.0015),
        2 => Just(1.2345),
        2 => Just(10.0004),
        2 => Just(100.0005),
        2 => Just(100.00005),
        2 => Just(999.9995),
        24 => 1.0..4000.0,
    ]
}

fn cut_strategy() -> impl Strategy<Value = Cut> {
    let line = (coord(), coord(), z_coord(), feed_strategy()).prop_map(|(x, y, z, feed)| Cut { kind: CutKind::Line { x, y, z }, feed });
    let arc = (plane_strategy(), radius_strategy(), sweep_strategy(), any::<bool>(), axial_strategy(), feed_strategy()).prop_map(
        |(plane, radius, sweep, ccw, axial, feed)| Cut {
            kind: CutKind::Arc { plane, radius, sweep, ccw, axial: if sweep >= TAU - 1e-6 { 0.0 } else { axial } },
            feed,
        },
    );
    let helix = (plane_strategy(), 1.0..30.0, 0.4..TAU, any::<bool>(), -8.0..8.0, feed_strategy())
        .prop_map(|(plane, radius, sweep, ccw, axial, feed)| Cut { kind: CutKind::Helix { plane, radius, sweep, ccw, axial }, feed });
    prop_oneof![4 => line, 3 => arc, 2 => helix]
}

fn plane_strategy() -> impl Strategy<Value = Plane> {
    prop_oneof![6 => Just(Plane::Xy), 1 => Just(Plane::Zx), 1 => Just(Plane::Yz)]
}

fn radius_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![1 => 0.002..0.02, 2 => 0.02..1.0, 5 => 1.0..40.0, 1 => 40.0..80.0]
}

fn sweep_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![2 => 0.05..0.3, 3 => 0.3..std::f64::consts::PI, 2 => std::f64::consts::PI..TAU - 0.05, 1 => TAU - 1e-3..TAU - 1e-4, 1 => Just(TAU)]
}

fn coord() -> impl Strategy<Value = f64> {
    prop_oneof![1 => Just(0.0), 1 => Just(-0.0), 1 => Just(-0.00001), 1 => Just(1.0), 1 => Just(-1.0), 8 => -80.0..80.0]
}

fn z_coord() -> impl Strategy<Value = f64> {
    prop_oneof![1 => Just(0.0), 1 => Just(-0.0), 1 => Just(5.0), 4 => -20.0..20.0]
}

fn axial_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![3 => Just(0.0), 1 => -4.0..4.0]
}

fn path_strategy() -> impl Strategy<Value = PathSpec> {
    (
        1u32..=3,
        500.0..18_000.0,
        0.05..1.0,
        0u8..4,
        prop::option::of(0.0..2.0),
        (1.0..60.0, -40.0..40.0, 1.0..20.0),
        prop::collection::vec(cut_strategy(), 1..=4),
        prop::option::of(500.0..18_000.0),
        prop::collection::vec(cut_strategy(), 0..=2),
        any::<bool>(),
    )
        .prop_map(|(tool, speed, power, coolant, dwell_s, (x, y, z), cuts, second_speed, extra, depart)| PathSpec {
            tool,
            speed,
            power,
            coolant,
            dwell_s,
            approach: [x, y, z],
            cuts,
            second_speed,
            extra,
            depart,
        })
}

fn spec_strategy() -> impl Strategy<Value = Spec> {
    prop::collection::vec(path_strategy(), 1..=3).prop_map(|paths| Spec { paths })
}

fn audit_config(cases: u32) -> ProptestConfig {
    ProptestConfig { cases, max_shrink_iters: 80, rng_seed: RngSeed::Fixed(0x00C2_2A01), ..ProptestConfig::default() }
}

proptest! {
    #![proptest_config(audit_config(96))]

    #[test]
    fn mill_posts_replay(spec in spec_strategy()) {
        let Some(program) = build(&spec) else {
            return Ok(());
        };
        check_all(&program, &spec)?;
    }
}

fn fixed_program(other_plane: bool) -> Option<Program> {
    let spec = Spec {
        paths: vec![
            PathSpec {
                tool: 1,
                speed: 12_000.0,
                power: 0.8,
                coolant: 1,
                dwell_s: Some(0.25),
                approach: [10.0, 0.0, 5.0],
                cuts: vec![
                    Cut { kind: CutKind::Line { x: 30.0, y: 5.0, z: 5.0 }, feed: 100.0 },
                    Cut { kind: CutKind::Arc { plane: Plane::Xy, radius: 10.0, sweep: TAU / 4.0, ccw: true, axial: 0.0 }, feed: 80.0 },
                    Cut { kind: CutKind::Arc { plane: Plane::Xy, radius: 8.0, sweep: TAU, ccw: true, axial: 0.0 }, feed: 90.0 },
                    Cut { kind: CutKind::Helix { plane: Plane::Xy, radius: 6.0, sweep: 4.0, ccw: false, axial: 2.0 }, feed: 70.0 },
                ],
                second_speed: None,
                extra: Vec::new(),
                depart: true,
            },
            PathSpec {
                tool: 2,
                speed: 8_000.0,
                power: 0.5,
                coolant: 2,
                dwell_s: None,
                approach: [5.0, 5.0, 6.0],
                cuts: vec![Cut { kind: CutKind::Line { x: 15.0, y: 8.0, z: 6.0 }, feed: 120.0 }],
                second_speed: Some(9_000.0),
                extra: vec![Cut {
                    kind: if other_plane {
                        CutKind::Arc { plane: Plane::Zx, radius: 8.0, sweep: TAU / 3.0, ccw: true, axial: 0.0 }
                    } else {
                        CutKind::Arc { plane: Plane::Xy, radius: 8.0, sweep: TAU / 3.0, ccw: true, axial: 0.0 }
                    },
                    feed: 60.0,
                }],
                depart: true,
            },
        ],
    };
    build(&spec)
}

#[test]
fn fixed_mill_program_replays_on_every_mill_post() {
    for mill in MILLS {
        let program = fixed_program(!mill.drop_z).expect("fixed program");
        let post = builtin(mill.id).expect(mill.id);
        check_one(&program, &post, mill).unwrap_or_else(|err| panic!("{}: {err}", mill.id));
    }
}

#[derive(Clone, Debug)]
struct RotarySpec {
    x: f64,
    y: f64,
    z: f64,
    distance: f64,
    angle: f64,
    feed: f64,
}

fn rotary_strategy() -> impl Strategy<Value = RotarySpec> {
    (0.0..20.0, 0.0..20.0, 5.0..15.0, 10.0..80.0, 10.0..90.0, 80.0..400.0).prop_map(|(x, y, z, distance, angle, feed)| RotarySpec {
        x,
        y,
        z,
        distance,
        angle,
        feed,
    })
}

fn turned(setup: RotarySetup, x: f64, y: f64, z: f64, angle: f64) -> Option<Pose> {
    let tip = Vec3::new(x, y, z).ok()?;
    let spindle = Pose::new(Vec3::ZERO, Vec3::Z).ok()?;
    let axis = machine_to_part(&setup, spindle, angle).axis();
    Pose::new(tip, axis).ok()?.with_rotary(angle).ok()
}

fn rotary_program(spec: &RotarySpec) -> Option<Program> {
    let setup = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, None).ok()?;
    let tool = Tool::try_new(1, Some("t1".into()), Some(6.0), None).ok()?;
    let params = ProcessParams::new().speed(Some(10_000.0)).ok()?.power(Some(0.5)).ok()?;
    let blocks = vec![
        ClBlock::Event(Event::ToolChange { tool_id: 1 }),
        ClBlock::Motion(ClMotion::Rapid { to: turned(setup, spec.x, spec.y, spec.z, 0.0)?, feed: None }),
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }),
        ClBlock::Motion(ClMotion::Linear {
            to: turned(setup, spec.x + spec.distance, spec.y, spec.z, spec.angle)?,
            feed: spec.feed,
            role: MoveRole::Cut,
        }),
        ClBlock::Motion(ClMotion::Linear {
            to: turned(setup, spec.x + spec.distance + 10.0, spec.y, spec.z, spec.angle)?,
            feed: spec.feed,
            role: MoveRole::Cut,
        }),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }),
    ];
    let feeds = Feeds::try_new(100.0, None, None, None, None).ok()?;
    let path = ClPath::try_new(Some("prop".into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).ok()?;
    Program::try_new(Some("prop".into()), vec![tool], None, vec![path], Vec::new(), Vec::new(), Some(setup)).ok()
}

proptest! {
    #![proptest_config(audit_config(24))]

    #[test]
    fn inverse_time_feeds_are_on_the_block(spec in rotary_strategy()) {
        let Some(program) = rotary_program(&spec) else {
            return Ok(());
        };
        for id in ["linuxcnc", "fanuc", "haas", "mach3"] {
            let post = builtin(id).expect(id);
            let text = match post_program(&program, &post, &options()) {
                Ok(text) => text,
                Err(err) if tolerated(&err.to_string()) => continue,
                Err(err) => return Err(TestCaseError::fail(format!("{id}: {err}\n{spec:?}"))),
            };
            let mill = MILLS.iter().find(|mill| mill.id == id).expect("mill");
            if let Err(message) = check_text(&text, mill, post.max_line_length()) {
                return Err(TestCaseError::fail(format!("{id}: {message}\n{text}")));
            }
            let report = match audit(&text, &post) {
                Ok(report) => report,
                Err(err) => return Err(TestCaseError::fail(format!("{id}: audit: {err}\n{text}"))),
            };
            let moves = read_back(&text, &post).expect("read");
            // Angle words round, and the part-frame tip moves by about radius times that
            // angle. 0.001 mm covers the built-in rotary decimals at the radii this case uses.
            if let Err(err) = compare(&program, &moves, 1e-3, &post) {
                return Err(TestCaseError::fail(format!("{id}: compare: {err}\n{text}")));
            }
            let feeds: Vec<&MotionState> = report.motions.iter().filter(|state| state.mode == MotionMode::Linear).collect();
            if feeds.len() != 2 {
                return Err(TestCaseError::fail(format!("{id}: expected two feeds, saw {}\n{text}", feeds.len())));
            }
            let limit = quantum(post.feed_decimals()) * 0.51 + spec.feed.abs() * 1e-12;
            if id == "mach3" {
                if text.contains("G93") || text.contains("G94") {
                    return Err(TestCaseError::fail(format!("mach3 entered inverse time\n{text}")));
                }
                for state in &feeds {
                    if state.feed_mode != FeedMode::PerMinute {
                        return Err(TestCaseError::fail(format!("mach3 feed mode {:?}", state.feed_mode)));
                    }
                    let got = state.feed.unwrap_or(0.0);
                    if (got - spec.feed).abs() > limit {
                        return Err(TestCaseError::fail(format!("mach3 feed {got} is not {}", spec.feed)));
                    }
                }
            } else {
                let inverse = feeds[0];
                if inverse.feed_mode != FeedMode::InverseTime || !inverse.feed_on_block {
                    return Err(TestCaseError::fail(format!("{id}: rotating feed is not inverse time with its own F\n{text}")));
                }
                let expect = spec.feed / spec.distance;
                let got = inverse.feed.unwrap_or(0.0);
                let inverse_limit = quantum(post.feed_decimals()) * 0.51 + expect.abs() * 1e-12;
                if (got - expect).abs() > inverse_limit {
                    return Err(TestCaseError::fail(format!("{id}: inverse F {got} is not {expect}")));
                }
                let plain = feeds[1];
                if plain.feed_mode != FeedMode::PerMinute || !plain.feed_on_block {
                    return Err(TestCaseError::fail(format!("{id}: feed after inverse time did not return to G94 with its own F\n{text}")));
                }
                let got = plain.feed.unwrap_or(0.0);
                if (got - spec.feed).abs() > limit {
                    return Err(TestCaseError::fail(format!("{id}: feed {got} is not {}", spec.feed)));
                }
            }
            if report.spindle.on {
                return Err(TestCaseError::fail(format!("{id}: spindle still on")));
            }
        }
    }
}
