//! Random lines and arcs survive posting and read-back.
//!
//! Mill steps are posted with every built-in that is not a lathe post. XZ steps are a turning
//! program and are posted with every lathe post.

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, Program, SpindleSpeed, TurnBinding, TurningSetup, Vec3,
};
use campanion_post::{PostOptions, builtin, builtin_ids, compare, post_program, read_back};
use proptest::prelude::*;

const TAU: f64 = std::f64::consts::TAU;

#[derive(Clone, Debug)]
enum Step {
    Rapid { x: f64, y: f64, z: f64 },
    Linear { x: f64, y: f64, z: f64, feed: f64 },
    Arc { radius: f64, sweep: f64, ccw: bool, feed: f64 },
}

fn pose(x: f64, y: f64, z: f64) -> Option<Pose> {
    let tip = Vec3::new(x, y, z).ok()?;
    Pose::new(tip, Vec3::Z).ok()
}

fn program_from(steps: &[Step]) -> Option<Program> {
    let mut blocks = Vec::new();
    let mut pos = [0.0_f64, 0.0, 0.0];
    let mut placed = false;
    for step in steps {
        match *step {
            Step::Rapid { x, y, z } => {
                blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, z)?, feed: None }));
                pos = [x, y, z];
                placed = true;
            }
            Step::Linear { x, y, z, feed } => {
                if !placed {
                    return None;
                }
                blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(x, y, z)?, feed, role: MoveRole::Cut }));
                pos = [x, y, z];
            }
            Step::Arc { radius, sweep, ccw, feed } => {
                if !placed {
                    return None;
                }
                let center = [pos[0] - radius, pos[1], pos[2]];
                let signed = if ccw { sweep } else { -sweep };
                let mut end = [center[0] + radius * signed.cos(), center[1] + radius * signed.sin(), pos[2]];
                let chord = (end[0] - pos[0]).hypot(end[1] - pos[1]);
                if chord <= 0.05 && sweep < TAU - 0.05 {
                    return None;
                }
                if sweep > TAU - 1e-4 {
                    end = pos;
                }
                let sense = if ccw { ArcSense::Ccw } else { ArcSense::Cw };
                blocks.push(ClBlock::Motion(ClMotion::Arc {
                    to: pose(end[0], end[1], end[2])?,
                    plane: Plane::Xy,
                    center: Vec3::new(center[0], center[1], center[2]).ok()?,
                    sense,
                    feed,
                    role: MoveRole::Cut,
                }));
                pos = end;
            }
        }
    }
    let feeds = Feeds::try_new(100.0, None, None, None, None).ok()?;
    let path = ClPath::try_new(Some("prop".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).ok()?;
    Program::try_new(Some("prop".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).ok()
}

fn steps_strategy() -> impl Strategy<Value = Vec<Step>> {
    let rapid = (-200.0..200.0, -200.0..200.0, -40.0..40.0).prop_map(|(x, y, z)| Step::Rapid { x, y, z });
    let linear = (-200.0..200.0, -200.0..200.0, -40.0..40.0, 20.0..1500.0).prop_map(|(x, y, z, feed)| Step::Linear { x, y, z, feed });
    let arc = (1.0..40.0, 0.4..TAU, proptest::bool::ANY, 20.0..1500.0).prop_map(|(radius, sweep, ccw, feed)| Step::Arc { radius, sweep, ccw, feed });
    let rest = prop::collection::vec(prop_oneof![linear, arc], 0..6);
    (rapid, rest).prop_map(|(first, rest)| {
        let mut steps = vec![first];
        steps.extend(rest);
        steps
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(80))]

    #[test]
    fn linear_and_arc_round_trip(steps in steps_strategy()) {
        let Some(program) = program_from(&steps) else {
            return Ok(());
        };
        let options = PostOptions { arc_tolerance: 0.05 };
        for id in builtin_ids() {
            let post = builtin(id).expect(id);
            if post.is_lathe() {
                continue;
            }
            let text = post_program(&program, &post, &options).expect("post");
            let moves = read_back(&text, &post).expect("read");
            let tol = 10_f64.powi(-(post.linear_decimals() as i32));
            compare(&program, &moves, tol, &post).expect("compare");
        }
    }
}

#[derive(Clone, Debug)]
enum XzStep {
    Rapid { x: f64, z: f64 },
    Linear { x: f64, z: f64, feed: f64 },
    Arc { radius: f64, sweep: f64, ccw: bool, feed: f64 },
    Thread { x: f64, z: f64, lead: f64 },
}

fn xz_program(steps: &[XzStep]) -> Option<Program> {
    let mut blocks = Vec::new();
    let mut pos = [0.0_f64, 0.0];
    let mut placed = false;
    for step in steps {
        match *step {
            XzStep::Rapid { x, z } => {
                blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose(x, 0.0, z)?, feed: None }));
                pos = [x, z];
                placed = true;
            }
            XzStep::Linear { x, z, feed } => {
                if !placed {
                    return None;
                }
                blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(x, 0.0, z)?, feed, role: MoveRole::Cut }));
                pos = [x, z];
            }
            XzStep::Arc { radius, sweep, ccw, feed } => {
                if !placed {
                    return None;
                }
                // Centre one radius toward -Z, so the start sits on the +Z side of the circle.
                let center = [pos[0], pos[1] - radius];
                let signed = if ccw { sweep } else { -sweep };
                let mut end = [center[0] + radius * signed.sin(), center[1] + radius * signed.cos()];
                let chord = (end[0] - pos[0]).hypot(end[1] - pos[1]);
                if chord <= 0.05 && sweep < TAU - 0.05 {
                    return None;
                }
                if sweep > TAU - 1e-4 {
                    end = pos;
                }
                let sense = if ccw { ArcSense::Ccw } else { ArcSense::Cw };
                blocks.push(ClBlock::Motion(ClMotion::Arc {
                    to: pose(end[0], 0.0, end[1])?,
                    plane: Plane::Zx,
                    center: Vec3::new(center[0], 0.0, center[1]).ok()?,
                    sense,
                    feed,
                    role: MoveRole::Cut,
                }));
                pos = end;
            }
            XzStep::Thread { x, z, lead } => {
                if !placed {
                    return None;
                }
                blocks.push(ClBlock::Motion(ClMotion::Thread { to: pose(x, 0.0, z)?, lead }));
                pos = [x, z];
            }
        }
    }
    let feeds = Feeds::try_new(100.0, None, None, None, None).ok()?;
    let path = ClPath::try_new(Some("turn".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).ok()?;
    let mut turning = TurningSetup::single(None, 6_000.0).ok()?;
    if steps.iter().any(|step| matches!(step, XzStep::Thread { .. })) {
        let speed = SpindleSpeed::rpm(800.0).ok()?;
        let binding = TurnBinding::try_new(0, "main", "turret1", 0).ok()?.with_speed(Some(speed)).ok()?;
        turning = turning.with_binding(binding).ok()?;
    }
    Program::try_new(Some("turn".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).ok()?.with_turning(Some(turning)).ok()
}

fn xz_steps() -> impl Strategy<Value = Vec<XzStep>> {
    let rapid = (-80.0..80.0, -80.0..40.0).prop_map(|(x, z)| XzStep::Rapid { x, z });
    let linear = (-80.0..80.0, -80.0..40.0, 20.0..1500.0).prop_map(|(x, z, feed)| XzStep::Linear { x, z, feed });
    let arc =
        (1.0..30.0, 0.4..TAU, proptest::bool::ANY, 20.0..1500.0).prop_map(|(radius, sweep, ccw, feed)| XzStep::Arc { radius, sweep, ccw, feed });
    let thread = (1.0..40.0, -80.0..40.0, 0.2..5.0).prop_map(|(x, z, lead)| XzStep::Thread { x, z, lead });
    let rest = prop::collection::vec(prop_oneof![linear, arc, thread], 0..6);
    (rapid, rest).prop_map(|(first, rest)| {
        let mut steps = vec![first];
        steps.extend(rest);
        steps
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(80))]

    #[test]
    fn lathe_xz_round_trip(steps in xz_steps()) {
        let Some(program) = xz_program(&steps) else {
            return Ok(());
        };
        let options = PostOptions { arc_tolerance: 0.05 };
        for id in builtin_ids() {
            let post = builtin(id).expect(id);
            if !post.is_lathe() {
                continue;
            }
            let text = post_program(&program, &post, &options).unwrap_or_else(|err| panic!("{id}: {err}"));
            let moves = read_back(&text, &post).unwrap_or_else(|err| panic!("{id}: {err}"));
            let tol = 10_f64.powi(-(post.linear_decimals() as i32));
            compare(&program, &moves, tol, &post).unwrap_or_else(|err| panic!("{id}: {err}"));
        }
    }
}
