//! Continuous rotary (4-axis) finishing.
//!
//! Each pass is a [`campanion_mesh::rotary_lines`] sample at one machine angle. Turning the mesh
//! costs one drop-cutter build per angle. The geometric angle is the machine angle with the
//! setup's direction sign, matching [`crate::index::rotate_mesh`]. The stored radius is the tip
//! height above the axis. Zigzag joins passes with a rotate at the shared end when that radius
//! stays at or above the drop-cutter radius at both angles and at the probed angles between
//! them. Otherwise the tool retracts to the safe radius, rotates, and plunges. One-way always
//! retracts. Angles run from 0° through a full turn and close at 360°.

use campanion_ir::{
    ClBlock, ClMotion, ClPath, CompensationMode, Event, MoveRole, Pose, ProcessKind, ProcessParams, RotaryAxisKind, RotaryDirection, RotarySetup,
    Vec3,
};
use campanion_mesh::{Cutter, DropCutter, Mesh, RotaryLineAxis, rotary_lines, scallop_stepover};

use crate::error::CliError;
use crate::gouge;
use crate::index;
use crate::jobfile::{JobFile, JobTool, Operation, RotaryTravel, ToolShape};

/// Matches the IR block cap. A longer program is rejected before it is built.
const MAX_MOVES: usize = 65_536;

/// Most interior angles probed on one contact link. A wider probe retracts instead.
const MAX_LINK_PROBES: usize = 128;

/// Most steps used to keep one rotary chord inside the chord tolerance.
const MAX_ROTATE_STEPS: usize = 8_192;

/// Tip radius where the cutter misses the turned mesh.
const FLOOR_RADIUS: f64 = 0.0;

/// One part-frame toolpath. `mesh` is the unrotated STL.
pub(crate) fn cut(job: &JobFile, op: &Operation, tool: &JobTool, mesh: &Mesh, setup: &RotarySetup) -> Result<ClPath, CliError> {
    let cutter = cutter(op, tool)?;
    let _bounds = mesh.bounds().map_err(|err| op_err(op, err))?;
    let (u_min, u_max) = u_window(op, mesh, setup)?;
    let step = angle_step(op, job, tool, &cutter, setup, mesh)?;
    let u_step = match op.sample_step() {
        Some(step) => step,
        None => gouge::sample_step_for(job.tolerance().chord())?,
    };
    let angles = full_turn_angles(step).map_err(|err| op_err(op, err))?;
    let geometric: Vec<f64> = angles.iter().map(|angle| geometric_deg(setup, *angle)).collect();
    let axis = match setup.axis() {
        RotaryAxisKind::A => RotaryLineAxis::X,
        RotaryAxisKind::B => RotaryLineAxis::Y,
    };
    let center = setup.center();
    let axis_point = (center.x(), center.y(), center.z());
    let lines = rotary_lines(mesh, cutter, axis, axis_point, (u_min, u_max), &geometric, u_step, FLOOR_RADIUS).map_err(|err| op_err(op, err))?;
    if lines.len() != angles.len() || lines.iter().any(|line| line.samples.is_empty()) {
        return Err(op_err(op, "rotary finish produced no toolpath"));
    }
    let safe_radius = safe_radius(job, setup, mesh, &lines)?;
    let travel = op.rotary_travel().unwrap_or(RotaryTravel::Zigzag);
    let tolerance = job.tolerance().chord();
    let blocks = build_blocks(op, tool, mesh, cutter, setup, &angles, &lines, safe_radius, travel, tolerance, u_step)?;
    ClPath::try_new(Some(op.name().to_string()), Some(op.tool_id()), CompensationMode::Computed, op.feeds(), None, Vec::new(), blocks)
        .map_err(|err| op_err(op, err))
}

/// Flat, ball or bull nose. A kerf tool is rejected.
pub(crate) fn cutter(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "rotary finish is not valid with a kerf tool"));
    };
    let radius = diameter * 0.5;
    let cutter = match tool.shape() {
        ToolShape::Flat => Cutter::new_flat(radius),
        ToolShape::Ball => Cutter::new_ball(radius),
        ToolShape::Bull { corner_radius } => Cutter::new_bull(radius, corner_radius),
    };
    cutter.map_err(|err| op_err(op, err))
}

/// `0, step, 2*step, …` and a closing `360`. The last gap is at most `step`.
fn full_turn_angles(step: f64) -> Result<Vec<f64>, &'static str> {
    if !(step.is_finite() && step > 0.0) {
        return Err("rotary angle step must be finite and positive");
    }
    let step = step.min(360.0);
    let mut angles = Vec::new();
    let mut angle = 0.0;
    loop {
        angles.push(angle);
        if angles.len() > 100_000 {
            return Err("too many rotary angles");
        }
        if angle >= 360.0 - 1e-9 {
            break;
        }
        let next = angle + step;
        if !(next.is_finite() && next > angle) {
            return Err("rotary angle step is not usable");
        }
        if next >= 360.0 - 1e-9 {
            angles.push(360.0);
            break;
        }
        angle = next;
    }
    Ok(angles)
}

fn geometric_deg(setup: &RotarySetup, machine_deg: f64) -> f64 {
    match setup.direction() {
        RotaryDirection::Positive => machine_deg,
        RotaryDirection::Negative => -machine_deg,
    }
}

fn u_window(op: &Operation, mesh: &Mesh, setup: &RotarySetup) -> Result<(f64, f64), CliError> {
    let (mut low, mut high) = axis_extent(mesh, setup).ok_or_else(|| op_err(op, "rotary finish needs a mesh extent"))?;
    if let Some(value) = op.u_min() {
        low = value;
    }
    if let Some(value) = op.u_max() {
        high = value;
    }
    if !(low.is_finite() && high.is_finite()) || low > high {
        return Err(op_err(op, "rotary u range is not usable"));
    }
    Ok((low, high))
}

fn axis_extent(mesh: &Mesh, setup: &RotarySetup) -> Option<(f64, f64)> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for triangle in mesh.triangles() {
        for point in [triangle.a, triangle.b, triangle.c] {
            let u = match setup.axis() {
                RotaryAxisKind::A => point.x,
                RotaryAxisKind::B => point.y,
            };
            low = low.min(u);
            high = high.max(u);
        }
    }
    if low.is_finite() && high.is_finite() && low <= high { Some((low, high)) } else { None }
}

fn angle_step(op: &Operation, job: &JobFile, tool: &JobTool, cutter: &Cutter, setup: &RotarySetup, mesh: &Mesh) -> Result<f64, CliError> {
    let radius = largest_radius(setup, mesh)?;
    if radius <= 1e-9 {
        return Err(op_err(op, "rotary finish needs a part off the axis"));
    }
    if let Some(stepover) = op.stepover() {
        return degrees_from_stepover(stepover, radius).map_err(|err| op_err(op, err));
    }
    if let Some(degrees) = op.angle_step_deg() {
        return Ok(degrees.clamp(f64::MIN_POSITIVE, 360.0));
    }
    let raw = scallop_stepover(cutter, job.tolerance().chord()).map_err(|err| op_err(op, err))?;
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "rotary finish is not valid with a kerf tool"));
    };
    degrees_from_stepover(raw.min(diameter), radius).map_err(|err| op_err(op, err))
}

fn degrees_from_stepover(stepover: f64, radius: f64) -> Result<f64, &'static str> {
    let degrees = (stepover / radius).to_degrees();
    if !(degrees.is_finite() && degrees > 0.0) {
        return Err("rotary stepover does not make an angle step");
    }
    Ok(degrees.min(360.0))
}

fn largest_radius(setup: &RotarySetup, mesh: &Mesh) -> Result<f64, CliError> {
    let mut points = Vec::new();
    for triangle in mesh.triangles() {
        for point in [triangle.a, triangle.b, triangle.c] {
            points.push(Vec3::new(point.x, point.y, point.z)?);
        }
    }
    Ok(index::swept_radius(setup, None, &points))
}

fn safe_radius(job: &JobFile, setup: &RotarySetup, mesh: &Mesh, lines: &[campanion_mesh::RotaryLine]) -> Result<f64, CliError> {
    let swept = largest_radius(setup, mesh)?;
    let clearance = job.clearance();
    let safe_z = index::index_safe_z(clearance, swept, clearance);
    let mut max_cut = 0.0_f64;
    for line in lines {
        for &(_, radius) in &line.samples {
            max_cut = max_cut.max(radius);
        }
    }
    let safe = (safe_z - setup.center().z()).max(swept + clearance).max(max_cut + clearance);
    if safe.is_finite() { Ok(safe) } else { Err(CliError::run("rotary safe radius is not finite")) }
}

struct Cursor {
    u: f64,
    radius: f64,
    angle: f64,
}

enum Travel {
    Rapid,
    Feed { feed: f64, role: MoveRole },
}

fn build_blocks(
    op: &Operation,
    tool: &JobTool,
    mesh: &Mesh,
    cutter: Cutter,
    setup: &RotarySetup,
    angles: &[f64],
    lines: &[campanion_mesh::RotaryLine],
    safe_radius: f64,
    travel: RotaryTravel,
    tolerance: f64,
    u_step: f64,
) -> Result<Vec<ClBlock>, CliError> {
    let plunge = op.feeds().resolve(MoveRole::Plunge);
    let cut_feed = op.feeds().resolve(MoveRole::Cut);
    let retract_feed = op.feeds().resolve(MoveRole::Retract);
    let zigzag = matches!(travel, RotaryTravel::Zigzag);
    let mut blocks = Vec::new();
    let mut cursor: Option<Cursor> = None;

    let Some(first_line) = lines.first() else {
        return Err(op_err(op, "rotary finish produced no toolpath"));
    };
    let Some(&first_angle) = angles.first() else {
        return Err(op_err(op, "rotary finish produced no toolpath"));
    };
    let first = ordered(&first_line.samples, false);
    let Some(&(u0, r0)) = first.first() else {
        return Err(op_err(op, "rotary finish produced no toolpath"));
    };
    move_to(op, setup, &mut blocks, &mut cursor, u0, safe_radius, first_angle, tolerance, &Travel::Rapid)?;
    blocks.push(ClBlock::Event(Event::ProcessOn { process: process_kind(tool), params: ProcessParams::new() }));
    move_to(op, setup, &mut blocks, &mut cursor, u0, r0, first_angle, tolerance, &Travel::Feed { feed: plunge, role: MoveRole::Plunge })?;

    for index in 0..lines.len() {
        let Some(line) = lines.get(index) else {
            break;
        };
        let Some(&angle) = angles.get(index) else {
            break;
        };
        let reverse = zigzag && index % 2 == 1;
        let samples = ordered(&line.samples, reverse);
        for &(u, radius) in &samples {
            move_to(op, setup, &mut blocks, &mut cursor, u, radius, angle, tolerance, &Travel::Feed { feed: cut_feed, role: MoveRole::Cut })?;
        }
        let Some(next_index) = index.checked_add(1) else {
            break;
        };
        if next_index >= lines.len() {
            break;
        }
        let Some(next_line) = lines.get(next_index) else {
            break;
        };
        let Some(&next_angle) = angles.get(next_index) else {
            break;
        };
        let next_reverse = zigzag && next_index % 2 == 1;
        let next_samples = ordered(&next_line.samples, next_reverse);
        let Some(&(next_u, next_r)) = next_samples.first() else {
            return Err(op_err(op, "rotary finish produced no toolpath"));
        };
        let Some(here) = cursor.as_ref() else {
            return Err(op_err(op, "rotary finish produced no toolpath"));
        };
        let here_u = here.u;
        let here_r = here.radius;
        let here_angle = here.angle;
        let contact = zigzag && contact_clears(mesh, cutter, setup, here_u, here_r, here_angle, next_r, next_angle, tolerance, u_step)?;
        if contact {
            move_to(
                op,
                setup,
                &mut blocks,
                &mut cursor,
                here_u,
                here_r,
                next_angle,
                tolerance,
                &Travel::Feed { feed: cut_feed, role: MoveRole::Cut },
            )?;
        } else {
            move_to(
                op,
                setup,
                &mut blocks,
                &mut cursor,
                here_u,
                safe_radius,
                here_angle,
                tolerance,
                &Travel::Feed { feed: retract_feed, role: MoveRole::Retract },
            )?;
            move_to(op, setup, &mut blocks, &mut cursor, here_u, safe_radius, next_angle, tolerance, &Travel::Rapid)?;
            if (here_u - next_u).abs() > 1e-9 {
                move_to(op, setup, &mut blocks, &mut cursor, next_u, safe_radius, next_angle, tolerance, &Travel::Rapid)?;
            }
            move_to(
                op,
                setup,
                &mut blocks,
                &mut cursor,
                next_u,
                next_r,
                next_angle,
                tolerance,
                &Travel::Feed { feed: plunge, role: MoveRole::Plunge },
            )?;
        }
    }

    let Some(end) = cursor.as_ref() else {
        return Err(op_err(op, "rotary finish produced no toolpath"));
    };
    let end_u = end.u;
    let end_angle = end.angle;
    move_to(
        op,
        setup,
        &mut blocks,
        &mut cursor,
        end_u,
        safe_radius,
        end_angle,
        tolerance,
        &Travel::Feed { feed: retract_feed, role: MoveRole::Retract },
    )?;
    blocks.push(ClBlock::Event(Event::ProcessOff { process: process_kind(tool) }));
    if blocks.len() > MAX_MOVES {
        return Err(op_err(op, format!("too many rotary moves: limit is {MAX_MOVES}")));
    }
    Ok(blocks)
}

fn process_kind(tool: &JobTool) -> ProcessKind {
    match tool.path_tool() {
        campanion_path::Tool::Diameter { .. } => ProcessKind::Spindle,
        campanion_path::Tool::Kerf { .. } => ProcessKind::Torch,
    }
}

fn ordered(samples: &[(f64, f64)], reverse: bool) -> Vec<(f64, f64)> {
    if reverse { samples.iter().rev().copied().collect() } else { samples.to_vec() }
}

/// True when a rotate at `radius` stays above the drop-cutter at both angles and the probes between them.
fn contact_clears(
    mesh: &Mesh,
    cutter: Cutter,
    setup: &RotarySetup,
    u: f64,
    radius: f64,
    from_angle: f64,
    next_radius: f64,
    to_angle: f64,
    tolerance: f64,
    sample_step: f64,
) -> Result<bool, CliError> {
    let margin = tolerance * 0.5;
    if radius + margin < next_radius {
        return Ok(false);
    }
    let probes = probe_angles(from_angle, to_angle, radius.max(next_radius), sample_step);
    if probes.len() > MAX_LINK_PROBES {
        return Ok(false);
    }
    for angle in probes {
        let hit = radius_at(mesh, cutter, setup, angle, u)?;
        if radius + margin < hit {
            return Ok(false);
        }
    }
    Ok(true)
}

fn probe_angles(from: f64, to: f64, radius: f64, sample_step: f64) -> Vec<f64> {
    let span = (to - from).abs();
    if !(span.is_finite() && span > 1e-9) {
        return Vec::new();
    }
    let r = radius.max(1e-6);
    let step = sample_step.max(1e-6);
    let mut degrees = (step / r).to_degrees();
    if !(degrees.is_finite() && degrees > 0.0) {
        degrees = span;
    }
    let count = (span / degrees).ceil();
    if !(count.is_finite() && count >= 2.0) {
        return Vec::new();
    }
    let count = count as usize;
    if count > MAX_LINK_PROBES.saturating_add(1) {
        // The caller treats an oversized list as "do not contact".
        return vec![0.0; MAX_LINK_PROBES.saturating_add(1)];
    }
    let mut out = Vec::new();
    for index in 1..count {
        out.push(from + (to - from) * (index as f64) / (count as f64));
    }
    out
}

fn radius_at(mesh: &Mesh, cutter: Cutter, setup: &RotarySetup, machine_angle: f64, u: f64) -> Result<f64, CliError> {
    let turned = index::rotate_mesh(mesh, setup, machine_angle)?;
    let dc = DropCutter::new(&turned, cutter).map_err(|err| CliError::run(format!("rotary link: {err}")))?;
    let (x, y) = machine_xy(setup, u);
    let radius = match dc.tip_z(x, y) {
        Some(z) => z - setup.center().z(),
        None => FLOOR_RADIUS,
    };
    if radius.is_finite() { Ok(radius) } else { Err(CliError::run("rotary link radius is not finite")) }
}

fn machine_xy(setup: &RotarySetup, u: f64) -> (f64, f64) {
    let center = setup.center();
    match setup.axis() {
        RotaryAxisKind::A => (u, center.y()),
        RotaryAxisKind::B => (center.x(), u),
    }
}

fn move_to(
    op: &Operation,
    setup: &RotarySetup,
    blocks: &mut Vec<ClBlock>,
    cursor: &mut Option<Cursor>,
    u: f64,
    radius: f64,
    angle: f64,
    tolerance: f64,
    travel: &Travel,
) -> Result<(), CliError> {
    if let Some(here) = cursor.as_ref()
        && (here.u - u).abs() <= 1e-9
        && (here.radius - radius).abs() <= 1e-9
        && (here.angle - angle).abs() <= 1e-9
    {
        return Ok(());
    }
    let angles = match cursor.as_ref() {
        Some(here) if (here.angle - angle).abs() > 1e-9 && (here.u - u).abs() <= 1e-9 && (here.radius - radius).abs() <= 1e-9 => {
            rotate_angles(here.angle, angle, radius, tolerance)?
        }
        _ => vec![angle],
    };
    for angle in angles {
        if blocks.len() >= MAX_MOVES {
            return Err(op_err(op, format!("too many rotary moves: limit is {MAX_MOVES}")));
        }
        let pose = posed(setup, u, radius, angle)?;
        blocks.push(match *travel {
            Travel::Rapid => ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None }),
            Travel::Feed { feed, role } => ClBlock::Motion(ClMotion::Linear { to: pose, feed, role }),
        });
        *cursor = Some(Cursor { u, radius, angle });
    }
    Ok(())
}

/// Angles strictly after `from`, ending at `to`, with each chord's sagitta at most half the tolerance.
fn rotate_angles(from: f64, to: f64, radius: f64, tolerance: f64) -> Result<Vec<f64>, CliError> {
    let span = (to - from).abs();
    if !(span.is_finite() && span > 1e-12) {
        return Ok(vec![to]);
    }
    let margin = (tolerance * 0.5).max(1e-9);
    let r = radius.abs().max(1e-6);
    let cos_limit = (1.0 - margin / r).clamp(-1.0, 1.0);
    let mut step = (2.0 * cos_limit.acos()).to_degrees();
    if !(step.is_finite() && step > 1e-9) {
        step = span;
    }
    let count = (span / step).ceil();
    if !(count.is_finite() && count >= 1.0) || count > MAX_ROTATE_STEPS as f64 {
        return Err(CliError::run(format!("too many rotary link steps: limit is {MAX_ROTATE_STEPS}")));
    }
    let count = count as usize;
    let mut out = Vec::with_capacity(count);
    for index in 1..=count {
        out.push(from + (to - from) * (index as f64) / (count as f64));
    }
    if let Some(last) = out.last_mut() {
        *last = to;
    }
    Ok(out)
}

fn posed(setup: &RotarySetup, u: f64, radius: f64, angle: f64) -> Result<Pose, CliError> {
    let center = setup.center();
    let (x, y, z) = match setup.axis() {
        RotaryAxisKind::A => (u, center.y(), center.z() + radius),
        RotaryAxisKind::B => (center.x(), u, center.z() + radius),
    };
    let machine = Pose::new(Vec3::new(x, y, z)?, Vec3::Z)?;
    index::map_machine_pose(setup, machine, angle)
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{MoveRole as Role, RotarySetup, Vec3, part_to_machine};
    use campanion_mesh::{Triangle, Vec3 as MeshVec};

    #[test]
    fn full_turn_covers_ninety_and_one_hundred() {
        assert_eq!(full_turn_angles(90.0).expect("90"), vec![0.0, 90.0, 180.0, 270.0, 360.0]);
        let angles = full_turn_angles(100.0).expect("100");
        assert_eq!(angles.first().copied(), Some(0.0));
        assert_eq!(angles.last().copied(), Some(360.0));
        for window in angles.windows(2) {
            let gap = window[1] - window[0];
            assert!(gap > 0.0 && gap <= 100.0 + 1e-6, "{gap}");
        }
        assert!(full_turn_angles(0.0).is_err());
        assert_eq!(full_turn_angles(400.0).expect("clamp").len(), 2);
    }

    fn cylinder(radius: f64, segments: usize) -> Mesh {
        let mut rings = Vec::new();
        for u in [-15.0, 15.0] {
            let mut ring = Vec::new();
            for index in 0..segments {
                let theta = index as f64 * std::f64::consts::TAU / segments as f64;
                ring.push(MeshVec::new(u, radius * theta.sin(), radius * theta.cos()));
            }
            rings.push(ring);
        }
        let mut triangles = Vec::new();
        for index in 0..segments {
            let next = (index + 1) % segments;
            let a = rings[0][index];
            let b = rings[1][index];
            let c = rings[0][next];
            let d = rings[1][next];
            triangles.push(Triangle { a, b, c });
            triangles.push(Triangle { a: b, b: d, c });
        }
        Mesh::new(triangles).expect("cylinder")
    }

    #[test]
    fn zigzag_on_a_cylinder_rotates_at_the_cylinder_radius() {
        let mesh = cylinder(10.0, 72);
        let bytes = campanion_mesh::write_stl_binary(&mesh);
        let hash = crate::jobfile::hash_bytes(&bytes);
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"cyl.stl","hash":"{hash}"}},"units":"mm","heights":{{"clearance":30,"retract":20}},"rotary":{{"axis":"a","center":[0,0,0],"direction":"positive"}},"tools":[{{"id":1,"diameter":6,"shape":"ball"}}],"operations":[{{"name":"round","type":"rotary_finish","tool_id":1,"angle_step_deg":90,"sample_step":10,"u_min":-5,"u_max":5,"direction":"zigzag","feeds":{{"cut":100,"plunge":50}}}}],"tolerance":{{"chord":0.05}}}}"#
        );
        let ticks = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|duration| duration.as_nanos()).unwrap_or(0);
        let dir = std::env::temp_dir().join(format!("campanion-rotary-{ticks}"));
        std::fs::create_dir_all(&dir).expect("dir");
        let path = dir.join("job.json");
        std::fs::write(&path, text).expect("job");
        let job = crate::jobfile::load(&path).expect("load");
        let op = &job.operations()[0];
        let tool = job.tool(1).expect("tool");
        let setup = job.rotary().expect("setup");
        let cut = must(cut(&job, op, tool, &mesh, &setup), "cut");
        let mut saw_rotate = false;
        let mut previous: Option<Pose> = None;
        for block in cut.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let (pose, role) = match motion {
                ClMotion::Linear { to, role, .. } => (*to, Some(*role)),
                ClMotion::Rapid { to, .. } => (*to, None),
                _ => continue,
            };
            if let Some(prev) = previous {
                let gap = pose.rotary().expect("angle") - prev.rotary().expect("prev");
                assert!((-1e-9..=90.0 + 1e-6).contains(&gap), "{gap}");
                if gap > 1.0 && role == Some(Role::Cut) {
                    let machine = part_to_machine(&setup, pose, pose.rotary().expect("angle")).tip();
                    assert!((machine.z() - 10.0).abs() < 0.05, "link left the cylinder: {}", machine.z());
                    saw_rotate = true;
                }
            }
            previous = Some(pose);
        }
        assert!(saw_rotate, "expected a contact rotate");
        let angles: Vec<f64> = cut
            .blocks()
            .iter()
            .filter_map(|block| match block {
                ClBlock::Motion(ClMotion::Linear { to, .. } | ClMotion::Rapid { to, .. }) => to.rotary(),
                _ => None,
            })
            .collect();
        assert!((angles.first().copied().unwrap_or(-1.0)).abs() < 1e-9);
        assert!((angles.last().copied().unwrap_or(0.0) - 360.0).abs() < 1e-6);
    }

    fn must<T>(result: Result<T, CliError>, what: &str) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{what}: {err}"),
        }
    }

    fn plan(label: &str, mesh: &Mesh, shape: &str, direction: &str, sample_step: f64) -> (crate::jobfile::JobFile, ClPath) {
        let bytes = campanion_mesh::write_stl_binary(mesh);
        let hash = crate::jobfile::hash_bytes(&bytes);
        let text = format!(
            concat!(
                r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"{hash}"}},"#,
                r#""units":"mm","heights":{{"clearance":30,"retract":20}},"#,
                r#""rotary":{{"axis":"a","center":[0,0,0],"direction":"positive"}},"#,
                r#""tools":[{{"id":1,"diameter":6,"shape":"{shape}"}}],"operations":[{{"name":"round","type":"rotary_finish","#,
                r#""tool_id":1,"angle_step_deg":90,"sample_step":{sample},"u_min":-5,"u_max":5,"direction":"{direction}","#,
                r#""feeds":{{"cut":100,"plunge":50}}}}],"tolerance":{{"chord":0.05}}}}"#
            ),
            hash = hash,
            shape = shape,
            sample = sample_step,
            direction = direction,
        );
        let ticks = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|duration| duration.as_nanos()).unwrap_or(0);
        let dir = std::env::temp_dir().join(format!("campanion-rotary-{label}-{ticks}"));
        std::fs::create_dir_all(&dir).expect("dir");
        let path = dir.join("job.json");
        std::fs::write(&path, text).expect("job");
        let job = crate::jobfile::load(&path).expect("load");
        let op = &job.operations()[0];
        let tool = job.tool(1).expect("tool");
        let setup = job.rotary().expect("setup");
        let cut = must(cut(&job, op, tool, mesh, &setup), "cut");
        (job, cut)
    }

    fn machine_tip(setup: &RotarySetup, pose: Pose) -> Vec3 {
        let angle = pose.rotary().expect("angle");
        part_to_machine(setup, pose, angle).tip()
    }

    fn push_face(triangles: &mut Vec<Triangle>, a: MeshVec, b: MeshVec, c: MeshVec, d: MeshVec) {
        triangles.push(Triangle { a, b, c });
        triangles.push(Triangle { a, b: c, c: d });
    }

    /// Square bar along X. A 90° link crosses a corner that stands above either flat face.
    fn square_bar(half: f64) -> Mesh {
        let (x0, x1) = (-15.0, 15.0);
        let (y0, y1, z0, z1) = (-half, half, -half, half);
        let p = |x, y, z| MeshVec::new(x, y, z);
        let (a, b, c, d) = (p(x0, y0, z0), p(x1, y0, z0), p(x1, y1, z0), p(x0, y1, z0));
        let (e, f, g, h) = (p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1));
        let mut triangles = Vec::new();
        push_face(&mut triangles, e, f, g, h);
        push_face(&mut triangles, a, d, c, b);
        push_face(&mut triangles, d, h, g, c);
        push_face(&mut triangles, a, b, f, e);
        push_face(&mut triangles, b, c, g, f);
        push_face(&mut triangles, a, e, h, d);
        Mesh::new(triangles).expect("bar")
    }

    fn motion_angles(path: &ClPath) -> Vec<f64> {
        let mut angles = Vec::new();
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let pose = match motion {
                ClMotion::Linear { to, .. } | ClMotion::Rapid { to, .. } => *to,
                _ => continue,
            };
            angles.push(pose.rotary().expect("angle"));
        }
        angles
    }

    fn flush_pass(deltas: &mut Vec<f64>, open_angle: &mut Option<f64>, open_delta: &mut f64) {
        if open_angle.take().is_some() && open_delta.abs() > 1e-6 {
            deltas.push(*open_delta);
        }
        *open_delta = 0.0;
    }

    /// Signed u change of each constant-angle cutting pass, in order.
    fn pass_deltas(setup: &RotarySetup, path: &ClPath) -> Vec<f64> {
        let mut deltas = Vec::new();
        let mut previous: Option<Pose> = None;
        let mut open_angle: Option<f64> = None;
        let mut open_delta = 0.0;
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let (pose, role) = match motion {
                ClMotion::Linear { to, role, .. } => (*to, Some(*role)),
                ClMotion::Rapid { to, .. } => (*to, None),
                _ => continue,
            };
            if let Some(prev) = previous
                && role == Some(Role::Cut)
            {
                let angle = pose.rotary().expect("angle");
                let prev_angle = prev.rotary().expect("previous angle");
                let u = machine_tip(setup, pose).x();
                let prev_u = machine_tip(setup, prev).x();
                let along = !index::angles_differ(angle, prev_angle) && (u - prev_u).abs() > 1e-6;
                if along {
                    match open_angle {
                        Some(current) if !index::angles_differ(current, angle) => open_delta += u - prev_u,
                        _ => {
                            flush_pass(&mut deltas, &mut open_angle, &mut open_delta);
                            open_angle = Some(angle);
                            open_delta = u - prev_u;
                        }
                    }
                } else {
                    flush_pass(&mut deltas, &mut open_angle, &mut open_delta);
                }
            }
            previous = Some(pose);
        }
        flush_pass(&mut deltas, &mut open_angle, &mut open_delta);
        deltas
    }

    fn program_of(setup: RotarySetup, path: ClPath) -> campanion_ir::Program {
        let tool = campanion_ir::Tool::try_new(1, None, Some(6.0), None).expect("tool");
        campanion_ir::Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), Some(setup)).expect("program")
    }

    #[test]
    fn angles_are_continuous_across_the_operation() {
        let mesh = cylinder(10.0, 72);
        let (_job, path) = plan("continuous", &mesh, "ball", "zigzag", 10.0);
        let angles = motion_angles(&path);
        let first = angles.first().copied().expect("first angle");
        let last = angles.last().copied().expect("last angle");
        assert!(first.abs() < 1e-9, "{first}");
        assert!((last - 360.0).abs() < 1e-6, "{last}");
        for window in angles.windows(2) {
            let gap = window[1] - window[0];
            assert!(gap >= -1e-9, "angle stepped back by {gap}");
            assert!(gap <= 90.0 + 1e-6, "angle jumped by {gap}");
        }
        for station in [0.0, 90.0, 180.0, 270.0, 360.0] {
            assert!(angles.iter().any(|angle| (angle - station).abs() < 1e-6), "missing {station} in {angles:?}");
        }
    }

    #[test]
    fn zigzag_alternates_direction() {
        let mesh = cylinder(10.0, 72);
        let (job, path) = plan("zigzag", &mesh, "ball", "zigzag", 10.0);
        let setup = job.rotary().expect("setup");
        let deltas = pass_deltas(&setup, &path);
        assert_eq!(deltas.len(), 5, "{deltas:?}");
        assert!(deltas[0] > 0.0, "first pass runs toward +u: {deltas:?}");
        for pair in deltas.windows(2) {
            assert!(pair[0] * pair[1] < 0.0, "zigzag did not alternate: {deltas:?}");
        }
    }

    #[test]
    fn ball_on_a_cylinder_holds_the_tip_radius() {
        let radius = 10.0;
        let mesh = cylinder(radius, 72);
        let (job, path) = plan("radius", &mesh, "ball", "zigzag", 10.0);
        let setup = job.rotary().expect("setup");
        let tolerance = job.tolerance().chord();
        let mut previous: Option<Pose> = None;
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let (pose, role) = match motion {
                ClMotion::Linear { to, role, .. } => (*to, Some(*role)),
                ClMotion::Rapid { to, .. } => (*to, None),
                _ => continue,
            };
            if role == Some(Role::Cut)
                && let Some(prev) = previous
            {
                for tip in [machine_tip(&setup, prev), machine_tip(&setup, pose)] {
                    let radial = index::distance_to_axis(&setup, tip);
                    assert!((radial - radius).abs() <= tolerance, "tip radius {radial} tolerance {tolerance}");
                }
            }
            previous = Some(pose);
        }
        let notes = must(crate::gouge::check_job(&job, &mesh, &program_of(setup, path)), "gouge");
        assert!(notes.is_empty(), "{notes:?}");
    }

    #[derive(Clone, Copy)]
    enum Step {
        Rapid(Pose),
        Feed(Pose, Role),
    }

    fn steps_of(path: &ClPath) -> Vec<Step> {
        let mut steps = Vec::new();
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            match motion {
                ClMotion::Rapid { to, .. } => steps.push(Step::Rapid(*to)),
                ClMotion::Linear { to, role, .. } => steps.push(Step::Feed(*to, *role)),
                _ => {}
            }
        }
        steps
    }

    #[test]
    fn gouging_link_retracts_to_the_safe_radius_then_rotates_and_plunges() {
        let mesh = square_bar(8.0);
        let (job, path) = plan("corner", &mesh, "flat", "zigzag", 10.0);
        let setup = job.rotary().expect("setup");
        let swept = must(largest_radius(&setup, &mesh), "swept");
        let safe = index::index_safe_z(job.clearance(), swept, job.clearance());
        let steps = steps_of(&path);
        let mut links = 0;
        let mut index = 0;
        while index < steps.len() {
            let Step::Feed(retract, Role::Retract) = steps[index] else {
                index += 1;
                continue;
            };
            if (machine_tip(&setup, retract).z() - safe).abs() > 1e-6 {
                index += 1;
                continue;
            }
            let retract_angle = retract.rotary().expect("retract angle");
            let mut angle = retract_angle;
            let mut rotated = false;
            let mut next = index + 1;
            while next < steps.len() {
                let Step::Rapid(pose) = steps[next] else {
                    break;
                };
                if (machine_tip(&setup, pose).z() - safe).abs() > 1e-6 {
                    break;
                }
                let next_angle = pose.rotary().expect("rapid angle");
                if next_angle > angle + 1e-6 {
                    rotated = true;
                }
                angle = next_angle;
                next += 1;
            }
            if rotated && let Some(Step::Feed(plunge, Role::Plunge)) = steps.get(next).copied() {
                let plunge_z = machine_tip(&setup, plunge).z();
                assert!((plunge_z - 8.0).abs() < 1e-3, "plunge landed at {plunge_z}, safe {safe}");
                assert!((plunge.rotary().expect("plunge angle") - angle).abs() < 1e-6);
                assert!((angle - retract_angle - 90.0).abs() < 1e-4, "rotated from {retract_angle} to {angle}");
                links += 1;
                index = next + 1;
                continue;
            }
            index += 1;
        }
        assert_eq!(links, 4, "expected every corner link to retract, rotate and plunge");
        let mut previous: Option<Pose> = None;
        for step in &steps {
            let pose = match *step {
                Step::Rapid(pose) | Step::Feed(pose, _) => pose,
            };
            if let Step::Feed(_, Role::Cut) = step
                && let Some(before) = previous
            {
                let gap = pose.rotary().expect("cut") - before.rotary().expect("before");
                assert!(gap.abs() <= 1e-6, "cutting move changed angle by {gap}");
            }
            previous = Some(pose);
        }
        let notes = must(crate::gouge::check_job(&job, &mesh, &program_of(setup, path)), "gouge");
        assert!(notes.is_empty(), "{notes:?}");
    }
}
