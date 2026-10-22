//! Random lines and arcs survive posting and read-back.

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, Program, Vec3};
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
    Program::try_new(Some("prop".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new()).ok()
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
            let text = post_program(&program, &post, &options).expect("post");
            let moves = read_back(&text, &post).expect("read");
            let tol = 10_f64.powi(-(post.linear_decimals() as i32));
            compare(&program, &moves, tol, &post).expect("compare");
        }
    }
}
