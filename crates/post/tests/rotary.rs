//! Random A-axis programs post with LinuxCNC and read back within tolerance.

use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Pose, Program, RotaryAxisKind, RotaryDirection, RotarySetup, Vec3, machine_to_part,
};
use campanion_post::{PostOptions, builtin, compare, post_program, read_back};
use proptest::prelude::*;

#[derive(Clone, Debug)]
struct Step {
    x: f64,
    y: f64,
    z: f64,
    angle: f64,
    /// Feed in mm/min. `None` is a rapid.
    feed: Option<f64>,
}

fn finite_vec(x: f64, y: f64, z: f64) -> Option<Vec3> {
    Vec3::new(x, y, z).ok()
}

fn turned(setup: &RotarySetup, tip: Vec3, angle: f64) -> Option<Pose> {
    let spindle = Pose::new(Vec3::ZERO, Vec3::Z).ok()?;
    let axis = machine_to_part(setup, spindle, angle).axis();
    Pose::new(tip, axis).ok()?.with_rotary(angle).ok()
}

/// A feed that changes the angle needs a positive part-frame tip distance for inverse time.
fn separate(prev: [f64; 3], next: [f64; 3], prev_angle: f64, angle: f64, feed: Option<f64>) -> [f64; 3] {
    if feed.is_none() || (angle - prev_angle).abs() <= 1e-9 {
        return next;
    }
    let dx = next[0] - prev[0];
    let dy = next[1] - prev[1];
    let dz = next[2] - prev[2];
    if dx * dx + dy * dy + dz * dz >= 0.25 {
        return next;
    }
    [prev[0] + 0.5, next[1], next[2]]
}

fn program_from(center: Vec3, negative: bool, steps: &[Step]) -> Result<Program, String> {
    let direction = if negative { RotaryDirection::Negative } else { RotaryDirection::Positive };
    let setup = RotarySetup::new(RotaryAxisKind::A, center, direction, None).map_err(|err| err.to_string())?;
    let mut blocks = Vec::new();
    let mut prev: Option<([f64; 3], f64)> = None;
    for step in steps {
        let mut point = [step.x, step.y, step.z];
        if let Some((was, was_angle)) = prev {
            point = separate(was, point, was_angle, step.angle, step.feed);
        }
        let tip = finite_vec(point[0], point[1], point[2]).ok_or_else(|| "tip".to_string())?;
        let pose = turned(&setup, tip, step.angle).ok_or_else(|| format!("pose at {}", step.angle))?;
        let motion = match step.feed {
            None => ClMotion::Rapid { to: pose, feed: None },
            Some(feed) => ClMotion::Linear { to: pose, feed, role: MoveRole::Cut },
        };
        blocks.push(ClBlock::Motion(motion));
        prev = Some((point, step.angle));
    }
    let feeds = Feeds::try_new(100.0, None, None, None, None).map_err(|err| err.to_string())?;
    let path = ClPath::try_new(Some("prop".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).map_err(|err| err.to_string())?;
    Program::try_new(Some("prop".into()), Vec::new(), None, vec![path], Vec::new(), Vec::new(), Some(setup)).map_err(|err| err.to_string())
}

fn step_strategy() -> impl Strategy<Value = Step> {
    (-30.0..30.0, -30.0..30.0, -30.0..30.0, -500.0..500.0, any::<bool>(), 20.0..800.0).prop_map(|(x, y, z, angle, linear, feed)| Step {
        x,
        y,
        z,
        angle,
        feed: if linear { Some(feed) } else { None },
    })
}

fn steps_strategy() -> impl Strategy<Value = Vec<Step>> {
    let first = (-30.0..30.0, -30.0..30.0, -30.0..30.0, -500.0..500.0).prop_map(|(x, y, z, angle)| Step { x, y, z, angle, feed: None });
    (first, prop::collection::vec(step_strategy(), 0..6)).prop_map(|(rapid, rest)| {
        let mut steps = vec![rapid];
        steps.extend(rest);
        steps
    })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn linuxcnc_rotary_round_trip(
        cx in -20.0..20.0,
        cy in -20.0..20.0,
        cz in -20.0..20.0,
        negative in any::<bool>(),
        steps in steps_strategy(),
    ) {
        let center = finite_vec(cx, cy, cz).expect("center");
        let program = program_from(center, negative, &steps).unwrap_or_else(|err| panic!("{err}"));
        let post = builtin("linuxcnc").expect("linuxcnc");
        let options = PostOptions { arc_tolerance: 0.01 };
        let text = post_program(&program, &post, &options).unwrap_or_else(|err| panic!("{err}"));
        let moves = read_back(&text, &post).unwrap_or_else(|err| panic!("{err}\n{text}"));
        compare(&program, &moves, 1e-3, &post).unwrap_or_else(|err| panic!("{err}\n{text}"));
    }
}
