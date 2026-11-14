//! Tool axis along a kerf profile: waterjet lead and taper, and oxy-fuel bevel.
//!
//! The path stores the requested lead and taper. A cited chart, when the job names one, remaps
//! only the taper to a head angle; the core never supplies a chart. Head travel limits come
//! from the user-supplied kinematics file.

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, Document, Event, FiveAxisSetup, Joints, MoveRole, Plane, Pose, Program, ToolPose, Vec3, solve_path,
};
use campanion_path::CutDirection;

use crate::error::CliError;
use crate::index;
use crate::jobfile::{JobFile, KerfTilt, Operation};

/// Half-kerf and other XY gaps treated as no travel when choosing a tangent.
const XY_EPS: f64 = 1e-9;
/// Densify a tilted arc so each chord swings the tool axis by less than the 5° program limit.
const ARC_STEP_DEG: f64 = 3.0;

/// Apply the operation's `tilt` to every path in `doc`, or return `doc` when the operation has none.
pub(crate) fn apply(doc: Document, job: &JobFile, op: &Operation) -> Result<Document, CliError> {
    let Some(tilt) = op.tilt() else {
        return Ok(doc);
    };
    let setup = job.five_axis().ok_or_else(|| op_err(op.name(), "tilt needs a kinematics file"))?;
    let waste_left = matches!(op.cut_direction(), CutDirection::Climb);
    let program = doc.program();
    let mut toolpaths = Vec::with_capacity(program.toolpaths().len());
    for path in program.toolpaths() {
        toolpaths.push(apply_path(path, op.name(), tilt, waste_left, setup, job.clearance(), job.retract())?);
    }
    rebuild_document(&doc, toolpaths)
}

/// Axis for travel `tx, ty` (any non-zero XY length) and a waste side.
///
/// The axis starts at +Z. A positive `lead_deg` rotates about (Z × T) toward the feed.
/// A positive `head_taper_deg` then rotates about (Z × W) toward the waste. Those two
/// rotations use the fixed plate axes, not a successive body frame.
pub(crate) fn kerf_axis(tx: f64, ty: f64, waste_left: bool, lead_deg: f64, head_taper_deg: f64) -> Result<Vec3, CliError> {
    let travel = unit_xy(tx, ty).ok_or_else(|| CliError::invalid("tilt needs a travel direction"))?;
    let waste = if waste_left { (-travel.1, travel.0) } else { (travel.1, -travel.0) };
    let lead_axis = (-travel.1, travel.0, 0.0); // Z × T
    let taper_axis = (-waste.1, waste.0, 0.0); // Z × W
    let mut axis = (0.0, 0.0, 1.0);
    axis = rodrigues(axis, lead_axis, lead_deg.to_radians());
    axis = rodrigues(axis, taper_axis, head_taper_deg.to_radians());
    vec3(axis.0, axis.1, axis.2)
}

fn apply_path(
    path: &ClPath,
    name: &str,
    tilt: &KerfTilt,
    waste_left: bool,
    setup: &FiveAxisSetup,
    _clearance: f64,
    _retract: f64,
) -> Result<ClPath, CliError> {
    let motions = collect_motions(path)?;
    if motions.is_empty() {
        return Ok(path.clone());
    }
    let tangents = assign_tangents(&motions)?;
    let mut axes = Vec::with_capacity(motions.len());
    for tangent in &tangents {
        axes.push(kerf_axis(tangent.0, tangent.1, waste_left, tilt.lead_deg(), tilt.head_taper_deg())?);
    }
    let anchors: Vec<Vec3> = motions.iter().map(|motion| motion.to.tip()).collect();
    let mut blocks = Vec::new();
    blocks.push(ClBlock::Event(Event::Comment { text: requested_comment(tilt) }));
    let mut emitted_axis: Option<Vec3> = None;
    let mut cursor: Option<Vec3> = None;
    let mut motion_index = 0usize;
    for block in path.blocks() {
        match block {
            ClBlock::Event(event) => blocks.push(ClBlock::Event(event.clone())),
            ClBlock::Motion(motion) => {
                let axis = axes[motion_index];
                let anchor = anchors[motion_index];
                let new_blocks = remap_motion(name, motion, cursor, axis, anchor, emitted_axis, tilt, waste_left)?;
                if let Some(last) = new_blocks.iter().rev().find_map(motion_pose) {
                    cursor = Some(last.tip());
                    emitted_axis = Some(last.axis());
                }
                blocks.extend(new_blocks);
                motion_index += 1;
            }
        }
    }
    let rebuilt = index::rebuild(path, blocks)?;
    let path = rebuilt.with_continuous(true);
    check_reach(name, setup, &path)?;
    Ok(path)
}

fn requested_comment(tilt: &KerfTilt) -> String {
    let mut text = format!("requested lead {} deg, requested taper {} deg", trim_num(tilt.lead_deg()), trim_num(tilt.requested_taper_deg()));
    if let Some(reference) = tilt.chart_reference() {
        text.push_str(&format!(", head taper {} deg from cited chart: {}", trim_num(tilt.head_taper_deg()), sanitize_comment(reference)));
    }
    text
}

fn trim_num(value: f64) -> String {
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.').to_string();
    if text == "-0" { "0".into() } else { text }
}

fn sanitize_comment(text: &str) -> String {
    text.replace(['(', ')'], " ").split_whitespace().collect::<Vec<_>>().join(" ")
}

struct MotionInfo {
    from: Option<Vec3>,
    to: Pose,
    role: Option<MoveRole>,
    xy_travel: bool,
    /// Unit start-tangent for an arc, when the start and centre are known.
    start_tangent: Option<(f64, f64)>,
}

fn collect_motions(path: &ClPath) -> Result<Vec<MotionInfo>, CliError> {
    let mut out = Vec::new();
    let mut cursor: Option<Vec3> = None;
    for block in path.blocks() {
        let Some(motion) = motion_of(block) else {
            continue;
        };
        let to = motion_end(motion);
        let from = cursor;
        let xy_travel = match from {
            Some(start) => (to.tip().x() - start.x()).hypot(to.tip().y() - start.y()) > XY_EPS,
            None => false,
        };
        out.push(MotionInfo { from, to, role: motion_role(motion), xy_travel, start_tangent: arc_start_tangent(motion, from) });
        cursor = Some(to.tip());
    }
    Ok(out)
}

fn assign_tangents(motions: &[MotionInfo]) -> Result<Vec<(f64, f64)>, CliError> {
    let mut tangents = vec![(0.0, 0.0); motions.len()];
    for (index, motion) in motions.iter().enumerate() {
        if let Some((tx, ty)) = motion_tangent(motion) {
            tangents[index] = (tx, ty);
        }
    }
    // Prefer the next cutting tangent, then the previous, so plunges and retracts follow the wall.
    let mut last = None;
    for index in (0..motions.len()).rev() {
        if unit_xy(tangents[index].0, tangents[index].1).is_some() {
            last = Some(tangents[index]);
        } else if let Some(value) = last {
            tangents[index] = value;
        }
    }
    let mut last = None;
    for tangent in &mut tangents {
        if unit_xy(tangent.0, tangent.1).is_some() {
            last = Some(*tangent);
        } else if let Some(value) = last {
            *tangent = value;
        }
    }
    if tangents.iter().all(|(tx, ty)| unit_xy(*tx, *ty).is_none()) {
        return Err(CliError::invalid("tilt needs a travel direction"));
    }
    Ok(tangents)
}

fn motion_tangent(motion: &MotionInfo) -> Option<(f64, f64)> {
    if let Some(tangent) = motion.start_tangent {
        return Some(tangent);
    }
    let start = motion.from?;
    let end = motion.to.tip();
    let tx = end.x() - start.x();
    let ty = end.y() - start.y();
    if tx.hypot(ty) <= XY_EPS {
        return None;
    }
    // Vertical plunges and retracts do not define a wall tangent even if they wander slightly.
    if matches!(motion.role, Some(MoveRole::Plunge | MoveRole::Retract)) && !motion.xy_travel {
        return None;
    }
    Some((tx, ty))
}

fn arc_start_tangent(motion: &ClMotion, from: Option<Vec3>) -> Option<(f64, f64)> {
    let ClMotion::Arc { center, sense, plane, .. } = motion else {
        return None;
    };
    if *plane != Plane::Xy {
        return None;
    }
    let start = from?;
    let rx = start.x() - center.x();
    let ry = start.y() - center.y();
    if rx.hypot(ry) <= XY_EPS {
        return None;
    }
    // CCW tangent of (rx, ry) is (-ry, rx); CW is (ry, -rx).
    let (tx, ty) = if matches!(sense, ArcSense::Ccw) { (-ry, rx) } else { (ry, -rx) };
    unit_xy(tx, ty)
}

fn remap_motion(
    name: &str,
    motion: &ClMotion,
    cursor: Option<Vec3>,
    axis: Vec3,
    anchor: Vec3,
    previous_axis: Option<Vec3>,
    tilt: &KerfTilt,
    waste_left: bool,
) -> Result<Vec<ClBlock>, CliError> {
    let dest = remap_tip(motion_end(motion).tip(), cursor, axis, anchor)?;
    let mut blocks = Vec::new();
    if let (Some(here), Some(prev)) = (cursor, previous_axis)
        && !matches!(motion, ClMotion::Rapid { .. })
    {
        let feed = motion_feed(motion).unwrap_or(1.0);
        blocks.extend(reorient_feeds(here, prev, axis, feed)?);
    }
    match motion {
        ClMotion::Rapid { feed, .. } => {
            // A mid-path no-op G0 would suspend a per_cut torch or jet. Skip it.
            if let (Some(start), Some(prev)) = (cursor, previous_axis)
                && (dest.x() - start.x()).hypot(dest.y() - start.y()).hypot((dest.z() - start.z()).abs()) <= XY_EPS
                && !axis_differs(prev, axis)
            {
                // keep cursor/axis unchanged
            } else {
                blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose(dest, axis)?, feed: *feed }));
            }
        }
        ClMotion::Linear { feed, role, power, .. } => {
            blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(dest, axis)?, feed: *feed, role: *role, power: *power }));
        }
        ClMotion::Arc { to, plane, center, sense, feed, role, power } => {
            if *plane != Plane::Xy {
                return Err(op_err(name, "tilt needs an XY arc"));
            }
            let start = cursor.ok_or_else(|| op_err(name, "tilt arc is missing a start"))?;
            blocks.extend(linearise_arc(start, to.tip(), *center, *sense, *feed, *role, *power, dest, tilt, waste_left)?);
        }
        ClMotion::Helix { .. } | ClMotion::Thread { .. } => return Err(op_err(name, "tilt does not write a helix or thread")),
    }
    Ok(blocks)
}

fn remap_tip(original: Vec3, cursor: Option<Vec3>, axis: Vec3, anchor: Vec3) -> Result<Vec3, CliError> {
    // Same XY as the related cut (plunge, retract, or a vertical rapid): travel along the tool axis.
    if (original.x() - anchor.x()).hypot(original.y() - anchor.y()) <= XY_EPS && (original.z() - anchor.z()).abs() > XY_EPS {
        return along_axis_at_z(anchor, axis, original.z());
    }
    // XY rapid at a constant height: sit on the tool axis above the destination cut.
    if let Some(start) = cursor {
        let dz = (original.z() - start.z()).abs();
        let xy = (original.x() - start.x()).hypot(original.y() - start.y());
        if xy > XY_EPS && dz <= XY_EPS {
            return along_axis_at_z(anchor, axis, original.z());
        }
    }
    // Cutting and lead moves keep the compensated tip and only change the axis.
    if (original.x() - anchor.x()).hypot(original.y() - anchor.y()) <= XY_EPS {
        return Ok(original);
    }
    Ok(original)
}

fn along_axis_at_z(tip: Vec3, axis: Vec3, z: f64) -> Result<Vec3, CliError> {
    if axis.z().abs() < 1e-9 {
        return Err(CliError::invalid("tool axis is too shallow to retract along the axis"));
    }
    let t = (z - tip.z()) / axis.z();
    vec3(tip.x() + t * axis.x(), tip.y() + t * axis.y(), z)
}

fn linearise_arc(
    start: Vec3,
    end: Vec3,
    center: Vec3,
    sense: ArcSense,
    feed: f64,
    role: MoveRole,
    power: Option<f64>,
    end_tip: Vec3,
    tilt: &KerfTilt,
    waste_left: bool,
) -> Result<Vec<ClBlock>, CliError> {
    let start_xy = (start.x() - center.x(), start.y() - center.y());
    let end_xy = (end.x() - center.x(), end.y() - center.y());
    let start_ang = start_xy.1.atan2(start_xy.0);
    let mut end_ang = end_xy.1.atan2(end_xy.0);
    let clockwise = matches!(sense, ArcSense::Cw);
    if clockwise {
        if end_ang >= start_ang {
            end_ang -= 2.0 * std::f64::consts::PI;
        }
    } else if end_ang <= start_ang {
        end_ang += 2.0 * std::f64::consts::PI;
    }
    let sweep = end_ang - start_ang;
    let radius = start_xy.0.hypot(start_xy.1);
    let steps = ((sweep.abs().to_degrees() / ARC_STEP_DEG).ceil() as usize).max(1);
    let mut blocks = Vec::new();
    for step in 1..=steps {
        let t = step as f64 / steps as f64;
        let ang = start_ang + sweep * t;
        let is_last = step == steps;
        let tip = if is_last { end_tip } else { vec3(center.x() + radius * ang.cos(), center.y() + radius * ang.sin(), start.z())? };
        let prev_ang = start_ang + sweep * ((step - 1) as f64 / steps as f64);
        let tx = radius * (ang.cos() - prev_ang.cos());
        let ty = radius * (ang.sin() - prev_ang.sin());
        let axis = kerf_axis(tx, ty, waste_left, tilt.lead_deg(), tilt.head_taper_deg())?;
        blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(tip, axis)?, feed, role, power }));
    }
    Ok(blocks)
}

fn check_reach(name: &str, setup: &FiveAxisSetup, path: &ClPath) -> Result<(), CliError> {
    let mut poses = Vec::new();
    for block in path.blocks() {
        let Some(pose) = motion_pose(block) else {
            continue;
        };
        poses.push(ToolPose { tip: pose.tip(), axis: pose.axis() });
    }
    let start = Joints { xyz: Vec3::ZERO, angles: [0.0, 0.0] };
    match solve_path(setup, &poses, Some(start), path.singular_cone_deg()) {
        Ok(_) => Ok(()),
        Err(err) => Err(op_err(name, reach_detail(&err.to_string()))),
    }
}

fn reach_detail(message: &str) -> String {
    let detail = message.strip_prefix("invalid input: ").unwrap_or(message);
    if detail.contains("outside the rotary limits") || detail.contains("leaves axis") {
        format!("{detail} (head travel limits come from the kinematics file)")
    } else {
        detail.to_string()
    }
}

fn rebuild_document(doc: &Document, toolpaths: Vec<ClPath>) -> Result<Document, CliError> {
    let program = doc.program();
    let mut rebuilt = Program::try_new(
        program.name().map(str::to_string),
        program.tools().to_vec(),
        program.machine().cloned(),
        toolpaths,
        program.channel_programs().to_vec(),
        program.syncs().to_vec(),
        program.rotary().copied(),
    )?;
    if let Some(setup) = program.five_axis() {
        rebuilt = rebuilt.with_five_axis(setup.clone())?;
    }
    if !program.fixtures().is_empty() {
        rebuilt = rebuilt.with_fixtures(program.fixtures().to_vec())?;
    }
    Ok(Document::new(rebuilt)?)
}

fn motion_of(block: &ClBlock) -> Option<&ClMotion> {
    match block {
        ClBlock::Motion(motion) => Some(motion),
        ClBlock::Event(_) => None,
    }
}

fn motion_end(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    }
}

fn motion_role(motion: &ClMotion) -> Option<MoveRole> {
    match motion {
        ClMotion::Linear { role, .. } | ClMotion::Arc { role, .. } | ClMotion::Helix { role, .. } => Some(*role),
        ClMotion::Rapid { .. } | ClMotion::Thread { .. } => None,
    }
}

fn motion_pose(block: &ClBlock) -> Option<Pose> {
    motion_of(block).map(motion_end)
}

fn motion_feed(motion: &ClMotion) -> Option<f64> {
    match motion {
        ClMotion::Linear { feed, .. } | ClMotion::Arc { feed, .. } | ClMotion::Helix { feed, .. } => Some(*feed),
        ClMotion::Rapid { feed, .. } => *feed,
        ClMotion::Thread { .. } => None,
    }
}

/// Rotate the head at the current tip in feed moves of at most 5°, so a per_cut jet stays on.
fn reorient_feeds(here: Vec3, from: Vec3, to: Vec3, feed: f64) -> Result<Vec<ClBlock>, CliError> {
    let angle = axis_change_deg(from, to);
    if angle <= 0.05 {
        return Ok(Vec::new());
    }
    let steps = ((angle / 5.0).ceil() as usize).max(1);
    let mut blocks = Vec::with_capacity(steps);
    for step in 1..=steps {
        let t = step as f64 / steps as f64;
        let axis = slerp_axis(from, to, t)?;
        blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose(here, axis)?, feed, role: MoveRole::Cut, power: None }));
    }
    Ok(blocks)
}

fn slerp_axis(from: Vec3, to: Vec3, t: f64) -> Result<Vec3, CliError> {
    let dot = (from.x() * to.x() + from.y() * to.y() + from.z() * to.z()).clamp(-1.0, 1.0);
    let angle = dot.acos();
    if angle <= 1e-12 {
        return Ok(to);
    }
    let sin = angle.sin();
    let a = (angle * (1.0 - t)).sin() / sin;
    let b = (angle * t).sin() / sin;
    vec3(from.x() * a + to.x() * b, from.y() * a + to.y() * b, from.z() * a + to.z() * b)
}

fn axis_change_deg(left: Vec3, right: Vec3) -> f64 {
    let dot = (left.x() * right.x() + left.y() * right.y() + left.z() * right.z()).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

fn axis_differs(left: Vec3, right: Vec3) -> bool {
    axis_change_deg(left, right) > 0.05
}

fn unit_xy(x: f64, y: f64) -> Option<(f64, f64)> {
    let len = x.hypot(y);
    if !len.is_finite() || len <= XY_EPS {
        return None;
    }
    Some((x / len, y / len))
}

fn rodrigues(vector: (f64, f64, f64), axis: (f64, f64, f64), radians: f64) -> (f64, f64, f64) {
    let (sin, cos) = radians.sin_cos();
    let along = axis.0 * vector.0 + axis.1 * vector.1 + axis.2 * vector.2;
    let cross = (axis.1 * vector.2 - axis.2 * vector.1, axis.2 * vector.0 - axis.0 * vector.2, axis.0 * vector.1 - axis.1 * vector.0);
    (
        vector.0 * cos + cross.0 * sin + axis.0 * along * (1.0 - cos),
        vector.1 * cos + cross.1 * sin + axis.1 * along * (1.0 - cos),
        vector.2 * cos + cross.2 * sin + axis.2 * along * (1.0 - cos),
    )
}

fn pose(tip: Vec3, axis: Vec3) -> Result<Pose, CliError> {
    Pose::new(tip, axis).map_err(CliError::from)
}

fn vec3(x: f64, y: f64, z: f64) -> Result<Vec3, CliError> {
    Vec3::new(x, y, z).map_err(CliError::from)
}

fn op_err(name: &str, message: impl std::fmt::Display) -> CliError {
    CliError::invalid(format!("operation `{name}`: {message}"))
}

#[cfg(test)]
mod tests {
    use super::kerf_axis;
    use proptest::prelude::*;

    fn angle_deg(left: (f64, f64, f64), right: (f64, f64, f64)) -> f64 {
        let dot = (left.0 * right.0 + left.1 * right.1 + left.2 * right.2).clamp(-1.0, 1.0);
        dot.acos().to_degrees()
    }

    fn components(axis: campanion_ir::Vec3) -> (f64, f64, f64) {
        (axis.x(), axis.y(), axis.z())
    }

    #[test]
    fn lead_only_leans_toward_the_feed() {
        // Travel +X, climb so waste is +Y. A 10° lead must stay in the XZ plane, toward +X.
        let axis = components(kerf_axis(1.0, 0.0, true, 10.0, 0.0).map_err(|err| err.to_string()).expect("axis"));
        assert!((axis.1).abs() < 1e-12, "{axis:?}");
        assert!(axis.0 > 0.0, "{axis:?}");
        assert!((angle_deg(axis, (0.0, 0.0, 1.0)) - 10.0).abs() < 1e-9);
    }

    #[test]
    fn taper_only_leans_toward_the_waste() {
        // Travel +X, climb so waste is +Y. A 10° taper must stay in the YZ plane, toward +Y.
        let axis = components(kerf_axis(1.0, 0.0, true, 0.0, 10.0).map_err(|err| err.to_string()).expect("axis"));
        assert!((axis.0).abs() < 1e-12, "{axis:?}");
        assert!(axis.1 > 0.0, "{axis:?}");
        assert!((angle_deg(axis, (0.0, 0.0, 1.0)) - 10.0).abs() < 1e-9);

        let conventional = components(kerf_axis(1.0, 0.0, false, 0.0, 10.0).map_err(|err| err.to_string()).expect("axis"));
        assert!((conventional.0).abs() < 1e-12, "{conventional:?}");
        assert!(conventional.1 < 0.0, "{conventional:?}");
        assert!((angle_deg(conventional, (0.0, 0.0, 1.0)) - 10.0).abs() < 1e-9);
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        #[test]
        fn wall_angles_match_the_requested_lead_and_taper_on_random_travel(
            heading in 0.0f64..std::f64::consts::TAU,
            lead in -40.0f64..40.0,
            taper in -40.0f64..40.0,
            waste_left in proptest::bool::ANY,
        ) {
            let tx = heading.cos();
            let ty = heading.sin();
            let axis = components(kerf_axis(tx, ty, waste_left, lead, taper).map_err(|err| err.to_string()).expect("axis"));
            let travel = (tx, ty, 0.0);
            let waste = if waste_left { (-ty, tx, 0.0) } else { (ty, -tx, 0.0) };
            // Lead about (Z × T), then taper about (Z × W), on the fixed plate axes.
            let expected_lead = rodrigues_ref((0.0, 0.0, 1.0), (-ty, tx, 0.0), lead.to_radians());
            let expected = rodrigues_ref(expected_lead, (-waste.1, waste.0, 0.0), taper.to_radians());
            prop_assert!((axis.0 - expected.0).abs() < 1e-9);
            prop_assert!((axis.1 - expected.1).abs() < 1e-9);
            prop_assert!((axis.2 - expected.2).abs() < 1e-9);
            if taper.abs() < 1e-12 {
                let plane = (axis.0 * waste.0 + axis.1 * waste.1).abs();
                prop_assert!(plane < 1e-9, "lead-only left the travel plane {axis:?} T={travel:?}");
                prop_assert!((angle_deg(axis, (0.0, 0.0, 1.0)) - lead.abs()).abs() < 1e-9);
            }
            if lead.abs() < 1e-12 {
                let plane = (axis.0 * travel.0 + axis.1 * travel.1).abs();
                prop_assert!(plane < 1e-9, "taper-only left the waste plane {axis:?}");
                prop_assert!((angle_deg(axis, (0.0, 0.0, 1.0)) - taper.abs()).abs() < 1e-9);
                if taper > 1e-9 {
                    let toward = axis.0 * waste.0 + axis.1 * waste.1;
                    prop_assert!(toward > 0.0, "positive taper must lean toward the waste");
                }
            }
        }
    }

    fn rodrigues_ref(vector: (f64, f64, f64), axis: (f64, f64, f64), radians: f64) -> (f64, f64, f64) {
        super::rodrigues(vector, axis, radians)
    }
}
