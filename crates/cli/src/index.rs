//! Indexed 3+1 moves: safe retracts between index angles, and the part-frame mapping.
//!
//! An operation is planned in the indexed view (machine XY, tool axis +Z) at its index angle.
//! [`map_toolpath_to_part`] writes that view back into the part frame and tags every pose with
//! the angle. [`index_change`] is the retract-and-rotate pair inserted when the angle changes.
//! The program is taken to start at [`PROGRAM_START_DEG`].
//!
//! The safe retract height is [`index_safe_z`]: the job safe height, or the swept radius plus
//! the rapid clearance, whichever is higher. The swept radius is the largest distance from the
//! rotary axis line to the stock-box corners, or to the mesh vertices when there is no stock box.
//!
//! A wrap operation is already in the part frame. [`frame_operations`] does not map it again.
//! Its first and last rotary angles still take the same retract-and-rotate path as an index change.

use campanion_ir::{
    ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Plane, Pose, RotarySetup, Vec3, machine_to_part, part_to_machine,
    rotate_point, rotate_vector,
};
use campanion_mesh::{Mesh, Triangle, Vec3 as MeshVec};

use crate::error::CliError;

/// Machine A/B angle before the first operation, in degrees.
pub(crate) const PROGRAM_START_DEG: f64 = 0.0;

/// Angles closer than this are the same index.
const ANGLE_EPS_DEG: f64 = 1e-9;

/// A rotated plane normal this close to a unit axis still lies in that cardinal plane.
const PLANE_EPS: f64 = 1e-6;

/// Samples used when an arc does not land in a cardinal plane after the index rotation.
const MAX_ARC_STEPS: usize = 8_192;

/// Retract in the current index, then the same machine tip at the new index.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct IndexChange {
    /// Part-frame pose of the retract, at the angle the machine is leaving.
    pub retract: Pose,
    /// Part-frame pose of the rotary rapid, at the angle the machine is entering.
    pub indexed: Pose,
    /// Machine-frame tip shared by both rapids. Z is the safe retract height.
    pub machine_tip: Vec3,
}

/// One operation's toolpaths.
///
/// Indexed work is still in the machine view, at one [`Self::angle_deg`]. A wrap is already in
/// the part frame (`part_frame`): [`Self::angle_deg`] is the first motion's rotary angle, and
/// the last motion may sit at a different angle.
pub(crate) struct IndexedOp {
    pub angle_deg: f64,
    pub tool_id: Option<u32>,
    pub feeds: Feeds,
    pub paths: Vec<ClPath>,
    pub part_frame: bool,
}

/// An arc or helix as the indexed view sees it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ViewCurve {
    pub start: Vec3,
    pub end: Vec3,
    pub center: Vec3,
    pub plane: Plane,
    pub sense: ArcSense,
}

/// True when `left` and `right` are not the same index angle.
pub(crate) fn angles_differ(left: f64, right: f64) -> bool {
    (left - right).abs() > ANGLE_EPS_DEG
}

/// Distance from `point` to the rotary axis line, in millimetres.
pub(crate) fn distance_to_axis(setup: &RotarySetup, point: Vec3) -> f64 {
    let axis = setup.axis_direction();
    let center = setup.center();
    let dx = point.x() - center.x();
    let dy = point.y() - center.y();
    let dz = point.z() - center.z();
    let along = dx * axis.x() + dy * axis.y() + dz * axis.z();
    let px = dx - axis.x() * along;
    let py = dy - axis.y() * along;
    let pz = dz - axis.z() * along;
    (px * px + py * py + pz * pz).sqrt()
}

/// Largest distance from the rotary axis line to the stock.
///
/// `stock_corners` wins when it is set. Otherwise every mesh vertex is measured. An empty
/// set of points has radius 0: there is nothing to sweep.
pub(crate) fn swept_radius(setup: &RotarySetup, stock_corners: Option<&[Vec3]>, mesh_vertices: &[Vec3]) -> f64 {
    let points = stock_corners.unwrap_or(mesh_vertices);
    let mut best = 0.0;
    for point in points {
        let distance = distance_to_axis(setup, *point);
        if distance > best {
            best = distance;
        }
    }
    best
}

/// Eight corners of an axis-aligned box.
pub(crate) fn box_corners(min: Vec3, max: Vec3) -> Result<Vec<Vec3>, CliError> {
    let mut corners = Vec::with_capacity(8);
    for x in [min.x(), max.x()] {
        for y in [min.y(), max.y()] {
            for z in [min.z(), max.z()] {
                corners.push(Vec3::new(x, y, z)?);
            }
        }
    }
    Ok(corners)
}

/// `max(safe_height, swept_radius + clearance)`.
///
/// `safe_height` is the job safe height. `clearance` is the job rapid clearance. Both are the
/// job's clearance height: rapids already sit on that plane, and the same gap is kept outside
/// the swept radius.
pub(crate) fn index_safe_z(safe_height: f64, swept_radius: f64, clearance: f64) -> f64 {
    safe_height.max(swept_radius + clearance)
}

/// Map one machine-view pose into the part frame and tag it with `angle_deg`.
pub(crate) fn map_machine_pose(setup: &RotarySetup, machine: Pose, angle_deg: f64) -> Result<Pose, CliError> {
    machine_to_part(setup, machine, angle_deg).with_rotary(angle_deg).map_err(CliError::from)
}

/// Retract to `safe_z` in the current indexed view, then hold that machine tip at `to_deg`.
///
/// Both poses are in the part frame. `current_machine_tip` is where the tool sits before the
/// retract; only its Z changes.
pub(crate) fn index_change(setup: &RotarySetup, from_deg: f64, to_deg: f64, current_machine_tip: Vec3, safe_z: f64) -> Result<IndexChange, CliError> {
    let machine_tip = Vec3::new(current_machine_tip.x(), current_machine_tip.y(), safe_z)?;
    let machine = Pose::new(machine_tip, Vec3::Z)?;
    let retract = map_machine_pose(setup, machine, from_deg)?;
    let indexed = map_machine_pose(setup, machine, to_deg)?;
    Ok(IndexChange { retract, indexed, machine_tip })
}

/// Turn every triangle by `angle_deg` about the rotary axis. The result is the indexed view.
pub(crate) fn rotate_mesh(mesh: &Mesh, setup: &RotarySetup, angle_deg: f64) -> Result<Mesh, CliError> {
    let mut triangles = Vec::with_capacity(mesh.triangles().len());
    for triangle in mesh.triangles() {
        triangles.push(Triangle {
            a: mesh_from_ir(rotate_point(setup, ir_from_mesh(triangle.a)?, angle_deg)),
            b: mesh_from_ir(rotate_point(setup, ir_from_mesh(triangle.b)?, angle_deg)),
            c: mesh_from_ir(rotate_point(setup, ir_from_mesh(triangle.c)?, angle_deg)),
        });
    }
    Mesh::new(triangles).map_err(|err| CliError::run(format!("rotated mesh: {err}")))
}

/// Write `path` into the part frame at `angle_deg`.
///
/// Straight moves and cardinal arcs are rotated. An arc whose plane is no longer XY, ZX or YZ
/// is replaced by lines within `chord_mm` of the machine-view curve.
pub(crate) fn map_toolpath_to_part(path: &ClPath, setup: &RotarySetup, angle_deg: f64, chord_mm: f64) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(path.blocks().len());
    let mut cursor: Option<Vec3> = None;
    for block in path.blocks() {
        blocks.extend(map_block(block, setup, angle_deg, chord_mm, &mut cursor)?);
    }
    let pierce = path.pierce().map(|point| rotate_point(setup, point, -angle_deg));
    ClPath::try_new(path.name().map(str::to_string), path.tool_id(), path.compensation(), path.feeds(), pierce, path.tabs().to_vec(), blocks)
        .map_err(CliError::from)
}

/// Part-frame arc back into the indexed view. The inverse of the rotation [`map_toolpath_to_part`] applies.
pub(crate) fn arc_to_machine(
    setup: &RotarySetup,
    angle_deg: f64,
    start: Vec3,
    end: Pose,
    center: Vec3,
    plane: Plane,
    sense: ArcSense,
) -> Result<ViewCurve, CliError> {
    let turned = rotate_vector(setup, plane_normal(plane), angle_deg);
    let (view_plane, flip) = plane_from_normal(turned).ok_or_else(|| CliError::run("indexed arc does not land in the machine view"))?;
    Ok(ViewCurve {
        start: rotate_point(setup, start, angle_deg),
        end: part_to_machine(setup, end, angle_deg).tip(),
        center: rotate_point(setup, center, angle_deg),
        plane: view_plane,
        sense: apply_flip(sense, flip),
    })
}

/// Map every indexed operation into the part frame and insert an index change where the angle
/// differs from the previous one, including from [`PROGRAM_START_DEG`] before the first operation.
///
/// A [`IndexedOp::part_frame`] operation is stored as given. Its machine tip, for the index
/// change, is the unwrapped machine view of the pose at that pose's rotary angle.
pub(crate) fn frame_operations(setup: &RotarySetup, ops: Vec<IndexedOp>, safe_z: f64, chord_mm: f64) -> Result<Vec<ClPath>, CliError> {
    let mut out = Vec::new();
    let mut prev_angle = PROGRAM_START_DEG;
    let mut prev_tip: Option<Vec3> = None;
    for op in ops {
        let IndexedOp { angle_deg, tool_id, feeds, paths, part_frame } = op;
        let (mut mapped, start_angle, end_angle, start_tip, end_tip) = if part_frame {
            let first = first_motion_pose(&paths)?;
            let last = last_motion_pose(&paths)?;
            let start_angle = rotary_angle(first)?;
            let end_angle = rotary_angle(last)?;
            (paths, start_angle, end_angle, machine_tip(setup, first)?, machine_tip(setup, last)?)
        } else {
            let end_tip = last_motion_tip(&paths)?;
            let start_tip = first_motion_tip(&paths)?;
            let mapped = map_op_paths(&paths, setup, angle_deg, chord_mm)?;
            (mapped, angle_deg, angle_deg, start_tip, end_tip)
        };
        if angles_differ(prev_angle, start_angle) {
            let from = match prev_tip {
                Some(tip) => tip,
                None => start_tip,
            };
            let change = index_change(setup, prev_angle, start_angle, from, safe_z)?;
            insert_index_change(&mut out, &mut mapped, tool_id, feeds, change)?;
        }
        out.extend(mapped);
        prev_tip = Some(end_tip);
        prev_angle = end_angle;
    }
    Ok(out)
}

/// A wrap toolpath: already in the part frame, entered at its first rotary angle.
pub(crate) fn part_frame_op(tool_id: u32, feeds: Feeds, paths: Vec<ClPath>) -> Result<IndexedOp, CliError> {
    let angle = rotary_angle(first_motion_pose(&paths)?)?;
    Ok(IndexedOp { angle_deg: angle, tool_id: Some(tool_id), feeds, paths, part_frame: true })
}

fn map_op_paths(paths: &[ClPath], setup: &RotarySetup, angle_deg: f64, chord_mm: f64) -> Result<Vec<ClPath>, CliError> {
    let mut mapped = Vec::with_capacity(paths.len());
    for path in paths {
        mapped.push(map_toolpath_to_part(path, setup, angle_deg, chord_mm)?);
    }
    Ok(mapped)
}

fn insert_index_change(
    out: &mut Vec<ClPath>,
    mapped: &mut [ClPath],
    tool_id: Option<u32>,
    feeds: Feeds,
    change: IndexChange,
) -> Result<(), CliError> {
    if let Some(last) = out.last_mut() {
        *last = append_rapid(last, change.retract)?;
    } else {
        out.push(rapid_path(tool_id, feeds, change.retract)?);
    }
    let Some(first) = mapped.first_mut() else {
        return Err(CliError::run("index change needs a toolpath"));
    };
    *first = prepend_rapid(first, change.indexed)?;
    Ok(())
}

fn map_block(block: &ClBlock, setup: &RotarySetup, angle_deg: f64, chord_mm: f64, cursor: &mut Option<Vec3>) -> Result<Vec<ClBlock>, CliError> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, feed }) => {
            let mapped = map_machine_pose(setup, *to, angle_deg)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Rapid { to: mapped, feed: *feed })])
        }
        ClBlock::Motion(ClMotion::Linear { to, feed, role }) => {
            let mapped = map_machine_pose(setup, *to, angle_deg)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Linear { to: mapped, feed: *feed, role: *role })])
        }
        ClBlock::Motion(ClMotion::Thread { to, lead }) => {
            let mapped = map_machine_pose(setup, *to, angle_deg)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Thread { to: mapped, lead: *lead })])
        }
        ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, feed, role }) => {
            map_curve(Curve::Arc, *to, *plane, *center, *sense, None, *feed, *role, setup, angle_deg, chord_mm, cursor)
        }
        ClBlock::Motion(ClMotion::Helix { to, plane, center, sense, sweep, feed, role }) => {
            map_curve(Curve::Helix, *to, *plane, *center, *sense, Some(*sweep), *feed, *role, setup, angle_deg, chord_mm, cursor)
        }
        ClBlock::Event(Event::Probe { toward, feed }) => {
            let toward = match toward {
                Some(pose) => Some(map_machine_pose(setup, *pose, angle_deg)?),
                None => None,
            };
            Ok(vec![ClBlock::Event(Event::Probe { toward, feed: *feed })])
        }
        ClBlock::Event(other) => Ok(vec![ClBlock::Event(other.clone())]),
    }
}

#[derive(Clone, Copy)]
enum Curve {
    Arc,
    Helix,
}

fn map_curve(
    curve: Curve,
    to: Pose,
    plane: Plane,
    center: Vec3,
    sense: ArcSense,
    sweep: Option<f64>,
    feed: f64,
    role: MoveRole,
    setup: &RotarySetup,
    angle_deg: f64,
    chord_mm: f64,
    cursor: &mut Option<Vec3>,
) -> Result<Vec<ClBlock>, CliError> {
    let turned = rotate_vector(setup, plane_normal(plane), -angle_deg);
    if let Some((new_plane, flip)) = plane_from_normal(turned) {
        let block = curve_block(
            curve,
            map_machine_pose(setup, to, angle_deg)?,
            new_plane,
            rotate_point(setup, center, -angle_deg),
            apply_flip(sense, flip),
            sweep,
            feed,
            role,
        )?;
        *cursor = Some(to.tip());
        return Ok(vec![ClBlock::Motion(block)]);
    }
    linearize_curve(to, plane, center, sense, sweep, feed, role, setup, angle_deg, chord_mm, cursor)
}

fn curve_block(
    curve: Curve,
    to: Pose,
    plane: Plane,
    center: Vec3,
    sense: ArcSense,
    sweep: Option<f64>,
    feed: f64,
    role: MoveRole,
) -> Result<ClMotion, CliError> {
    match curve {
        Curve::Arc => Ok(ClMotion::Arc { to, plane, center, sense, feed, role }),
        Curve::Helix => {
            let sweep = sweep.ok_or_else(|| CliError::run("index helix needs a sweep"))?;
            Ok(ClMotion::Helix { to, plane, center, sense, sweep, feed, role })
        }
    }
}

fn linearize_curve(
    to: Pose,
    plane: Plane,
    center: Vec3,
    sense: ArcSense,
    sweep: Option<f64>,
    feed: f64,
    role: MoveRole,
    setup: &RotarySetup,
    angle_deg: f64,
    chord_mm: f64,
    cursor: &mut Option<Vec3>,
) -> Result<Vec<ClBlock>, CliError> {
    let Some(start) = *cursor else {
        return Err(CliError::run("index arc needs a start position"));
    };
    let samples = curve_samples(start, to.tip(), plane, center, sense, sweep, chord_mm)?;
    let mut blocks = Vec::with_capacity(samples.len());
    for (index, sample) in samples.iter().enumerate() {
        let pose = if index + 1 == samples.len() {
            map_machine_pose(setup, to, angle_deg)?
        } else {
            let machine = Pose::new(*sample, to.axis())?;
            map_machine_pose(setup, machine, angle_deg)?
        };
        blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose, feed, role }));
    }
    *cursor = Some(to.tip());
    Ok(blocks)
}

fn curve_samples(
    start: Vec3,
    end: Vec3,
    plane: Plane,
    center: Vec3,
    sense: ArcSense,
    sweep: Option<f64>,
    chord_mm: f64,
) -> Result<Vec<Vec3>, CliError> {
    let (su, sv, sn) = plane_parts(start, plane);
    let (eu, ev, en) = plane_parts(end, plane);
    let (cu, cv, _) = plane_parts(center, plane);
    let radius = (su - cu).hypot(sv - cv);
    let sweep = match sweep {
        Some(sweep) if sweep.is_finite() && sweep > 0.0 => sweep,
        Some(_) => return Err(CliError::run("index arc sweep is not usable")),
        None => plane_sweep(su, sv, eu, ev, cu, cv, sense)?,
    };
    let steps = arc_steps(radius, sweep, chord_mm)?;
    let a0 = (sv - cv).atan2(su - cu);
    if !a0.is_finite() {
        return Err(CliError::run("index arc angle is not finite"));
    }
    let signed = match sense {
        ArcSense::Ccw => sweep,
        ArcSense::Cw => -sweep,
    };
    let mut samples = Vec::with_capacity(steps);
    for index in 1..=steps {
        if index == steps {
            samples.push(end);
            break;
        }
        let t = index as f64 / steps as f64;
        let angle = a0 + signed * t;
        let normal = sn + (en - sn) * t;
        samples.push(from_plane(cu + radius * angle.cos(), cv + radius * angle.sin(), normal, plane)?);
    }
    Ok(samples)
}

fn arc_steps(radius: f64, sweep: f64, chord_mm: f64) -> Result<usize, CliError> {
    let step = angular_step(radius, chord_mm);
    if !step.is_finite() || step <= 0.0 || !sweep.is_finite() {
        return Err(CliError::run("index arc spacing is not usable"));
    }
    let count = (sweep / step).ceil();
    if !count.is_finite() || count < 1.0 || count > MAX_ARC_STEPS as f64 {
        return Err(CliError::run(format!("too many index arc samples: limit is {MAX_ARC_STEPS}")));
    }
    Ok(count as usize)
}

fn angular_step(radius: f64, tol: f64) -> f64 {
    let tol = if tol.is_finite() && tol > 0.0 { tol } else { 0.01 };
    if !(radius.is_finite() && radius > 0.0) || tol >= radius {
        return std::f64::consts::FRAC_PI_2;
    }
    let step = 2.0 * (1.0 - tol / radius).clamp(-1.0, 1.0).acos();
    if step.is_finite() && step > 1e-6 { step } else { 1e-3 }
}

fn plane_sweep(su: f64, sv: f64, eu: f64, ev: f64, cu: f64, cv: f64, sense: ArcSense) -> Result<f64, CliError> {
    let a0 = (sv - cv).atan2(su - cu);
    let a1 = (ev - cv).atan2(eu - cu);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(CliError::run("index arc angle is not finite"));
    }
    let tau = std::f64::consts::TAU;
    let chord = (eu - su).hypot(ev - sv);
    let ccw_delta = (a1 - a0).rem_euclid(tau);
    let sweep = if chord <= 1e-12 {
        tau
    } else if sense == ArcSense::Ccw {
        if ccw_delta <= 1e-15 { tau } else { ccw_delta }
    } else {
        let cw = tau - ccw_delta;
        if cw <= 1e-15 { tau } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(CliError::run("index arc sweep is not usable")) }
}

fn append_rapid(path: &ClPath, to: Pose) -> Result<ClPath, CliError> {
    let mut blocks = path.blocks().to_vec();
    blocks.push(ClBlock::Motion(ClMotion::Rapid { to, feed: None }));
    rebuild(path, blocks)
}

fn prepend_rapid(path: &ClPath, to: Pose) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(path.blocks().len().saturating_add(1));
    blocks.push(ClBlock::Motion(ClMotion::Rapid { to, feed: None }));
    blocks.extend(path.blocks().iter().cloned());
    rebuild(path, blocks)
}

fn rapid_path(tool_id: Option<u32>, feeds: Feeds, to: Pose) -> Result<ClPath, CliError> {
    ClPath::try_new(None, tool_id, CompensationMode::Off, feeds, None, Vec::new(), vec![ClBlock::Motion(ClMotion::Rapid { to, feed: None })])
        .map_err(CliError::from)
}

fn rebuild(path: &ClPath, blocks: Vec<ClBlock>) -> Result<ClPath, CliError> {
    ClPath::try_new(path.name().map(str::to_string), path.tool_id(), path.compensation(), path.feeds(), path.pierce(), path.tabs().to_vec(), blocks)
        .map_err(CliError::from)
}

fn first_motion_tip(paths: &[ClPath]) -> Result<Vec3, CliError> {
    Ok(first_motion_pose(paths)?.tip())
}

fn last_motion_tip(paths: &[ClPath]) -> Result<Vec3, CliError> {
    Ok(last_motion_pose(paths)?.tip())
}

fn first_motion_pose(paths: &[ClPath]) -> Result<Pose, CliError> {
    for path in paths {
        for block in path.blocks() {
            if let Some(pose) = block_pose(block) {
                return Ok(pose);
            }
        }
    }
    Err(CliError::run("index change needs a tool position"))
}

fn last_motion_pose(paths: &[ClPath]) -> Result<Pose, CliError> {
    for path in paths.iter().rev() {
        for block in path.blocks().iter().rev() {
            if let Some(pose) = block_pose(block) {
                return Ok(pose);
            }
        }
    }
    Err(CliError::run("index change needs a tool position"))
}

fn rotary_angle(pose: Pose) -> Result<f64, CliError> {
    pose.rotary().ok_or_else(|| CliError::run("wrap pose has no rotary angle"))
}

/// Machine-view tip of a part-frame pose, using the angle stored on that pose.
fn machine_tip(setup: &RotarySetup, pose: Pose) -> Result<Vec3, CliError> {
    let angle = rotary_angle(pose)?;
    Ok(part_to_machine(setup, pose, angle).tip())
}

fn block_pose(block: &ClBlock) -> Option<Pose> {
    match block {
        ClBlock::Motion(
            ClMotion::Rapid { to, .. }
            | ClMotion::Linear { to, .. }
            | ClMotion::Arc { to, .. }
            | ClMotion::Helix { to, .. }
            | ClMotion::Thread { to, .. },
        ) => Some(*to),
        ClBlock::Event(_) => None,
    }
}

fn plane_normal(plane: Plane) -> Vec3 {
    match plane {
        Plane::Xy => Vec3::Z,
        Plane::Zx => Vec3::Y,
        Plane::Yz => Vec3::X,
    }
}

/// `(plane, flip)`. `flip` is set when `normal` points opposite the plane's view direction.
fn plane_from_normal(normal: Vec3) -> Option<(Plane, bool)> {
    let axes = [
        (0.0, 0.0, 1.0, Plane::Xy, false),
        (0.0, 0.0, -1.0, Plane::Xy, true),
        (0.0, 1.0, 0.0, Plane::Zx, false),
        (0.0, -1.0, 0.0, Plane::Zx, true),
        (1.0, 0.0, 0.0, Plane::Yz, false),
        (-1.0, 0.0, 0.0, Plane::Yz, true),
    ];
    for (x, y, z, plane, flip) in axes {
        if near(normal, x, y, z) {
            return Some((plane, flip));
        }
    }
    None
}

fn near(normal: Vec3, x: f64, y: f64, z: f64) -> bool {
    let dx = normal.x() - x;
    let dy = normal.y() - y;
    let dz = normal.z() - z;
    dx * dx + dy * dy + dz * dz <= PLANE_EPS * PLANE_EPS
}

fn apply_flip(sense: ArcSense, flip: bool) -> ArcSense {
    if !flip {
        return sense;
    }
    match sense {
        ArcSense::Cw => ArcSense::Ccw,
        ArcSense::Ccw => ArcSense::Cw,
    }
}

fn plane_parts(point: Vec3, plane: Plane) -> (f64, f64, f64) {
    match plane {
        Plane::Xy => (point.x(), point.y(), point.z()),
        Plane::Zx => (point.z(), point.x(), point.y()),
        Plane::Yz => (point.y(), point.z(), point.x()),
    }
}

fn from_plane(u: f64, v: f64, normal: f64, plane: Plane) -> Result<Vec3, CliError> {
    let (x, y, z) = match plane {
        Plane::Xy => (u, v, normal),
        Plane::Zx => (v, normal, u),
        Plane::Yz => (normal, u, v),
    };
    Vec3::new(x, y, z).map_err(CliError::from)
}

fn ir_from_mesh(point: MeshVec) -> Result<Vec3, CliError> {
    Vec3::new(point.x, point.y, point.z).map_err(CliError::from)
}

fn mesh_from_ir(point: Vec3) -> MeshVec {
    MeshVec::new(point.x(), point.y(), point.z())
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{Document, Program, RotaryAxisKind, RotaryDirection, RotaryLimits};
    use proptest::prelude::*;

    fn must<T>(result: Result<T, CliError>, what: &str) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{what}: {err}"),
        }
    }

    fn vec(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z).expect("vec")
    }

    fn setup_at(kind: RotaryAxisKind, direction: RotaryDirection, center: Vec3) -> RotarySetup {
        RotarySetup::new(kind, center, direction, None).expect("setup")
    }

    fn a_pos() -> RotarySetup {
        setup_at(RotaryAxisKind::A, RotaryDirection::Positive, Vec3::ZERO)
    }

    fn dist(left: Vec3, right: Vec3) -> f64 {
        let dx = left.x() - right.x();
        let dy = left.y() - right.y();
        let dz = left.z() - right.z();
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn close(left: Vec3, right: Vec3) {
        let gap = dist(left, right);
        assert!(gap <= 1e-9, "gap {gap} left {} {} {} right {} {} {}", left.x(), left.y(), left.z(), right.x(), right.y(), right.z());
    }

    fn feeds() -> Feeds {
        Feeds::try_new(100.0, None, None, None, None).expect("feeds")
    }

    fn rapid(name: &str, tip: Vec3) -> ClPath {
        let pose = Pose::new(tip, Vec3::Z).expect("pose");
        ClPath::try_new(
            Some(name.into()),
            Some(1),
            CompensationMode::Off,
            feeds(),
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None })],
        )
        .expect("path")
    }

    fn op(name: &str, angle: f64, tip: Vec3) -> IndexedOp {
        IndexedOp { angle_deg: angle, tool_id: Some(1), feeds: feeds(), paths: vec![rapid(name, tip)], part_frame: false }
    }

    #[test]
    fn distance_and_swept_radius_follow_the_axis_line() {
        let a = a_pos();
        assert!((distance_to_axis(&a, vec(5.0, 3.0, 4.0)) - 5.0).abs() < 1e-12);
        let offset = setup_at(RotaryAxisKind::A, RotaryDirection::Positive, vec(0.0, 1.0, 1.0));
        assert!((distance_to_axis(&offset, vec(0.0, 4.0, 5.0)) - 5.0).abs() < 1e-12);
        let b = setup_at(RotaryAxisKind::B, RotaryDirection::Negative, Vec3::ZERO);
        assert!((distance_to_axis(&b, vec(3.0, 9.0, 4.0)) - 5.0).abs() < 1e-12);

        let corners = must(box_corners(vec(-1.0, -2.0, -3.0), vec(4.0, 5.0, 6.0)), "corners");
        let vertices = [vec(0.0, 100.0, 0.0)];
        let from_box = swept_radius(&a, Some(&corners), &vertices);
        assert!((from_box - (5.0_f64.hypot(6.0))).abs() < 1e-12, "{from_box}");
        assert!((swept_radius(&a, None, &vertices) - 100.0).abs() < 1e-12);
        assert_eq!(swept_radius(&a, None, &[]), 0.0);
    }

    #[test]
    fn safe_height_is_the_larger_of_the_job_height_and_the_swept_clearance() {
        assert_eq!(index_safe_z(5.0, 20.0, 5.0), 25.0);
        assert_eq!(index_safe_z(40.0, 10.0, 5.0), 40.0);
        assert_eq!(index_safe_z(5.0, 0.0, 5.0), 5.0);
    }

    #[test]
    fn index_change_holds_the_machine_tip_and_tags_each_angle() {
        let setup = a_pos();
        let change = must(index_change(&setup, 0.0, 90.0, vec(1.0, 2.0, 3.0), 25.0), "change");
        close(change.machine_tip, vec(1.0, 2.0, 25.0));
        close(change.retract.tip(), vec(1.0, 2.0, 25.0));
        close(change.retract.axis(), Vec3::Z);
        assert_eq!(change.retract.rotary(), Some(0.0));
        close(change.indexed.tip(), vec(1.0, 25.0, -2.0));
        close(change.indexed.axis(), Vec3::Y);
        assert_eq!(change.indexed.rotary(), Some(90.0));
        let limits = RotaryLimits::new(-180.0, 180.0).expect("limits");
        let limited = RotarySetup::new(RotaryAxisKind::A, Vec3::ZERO, RotaryDirection::Positive, Some(limits)).expect("limited");
        let program = Program::try_new(
            None,
            Vec::new(),
            None,
            vec![must(rapid_path(None, feeds(), change.retract), "retract"), must(rapid_path(None, feeds(), change.indexed), "indexed")],
            Vec::new(),
            Vec::new(),
            Some(limited),
        )
        .expect("program");
        let report = campanion_ir::verify(&program);
        assert!(report.issues().is_empty(), "{:?}", report.issues());
    }

    #[test]
    fn quarter_turn_arc_lands_in_the_zx_plane_and_round_trips() {
        let setup = a_pos();
        let start = Pose::new(vec(1.0, 0.0, 0.0), Vec3::Z).expect("start");
        let end = Pose::new(vec(0.0, 1.0, 0.0), Vec3::Z).expect("end");
        let center = Vec3::ZERO;
        let path = ClPath::try_new(
            Some("arc".into()),
            None,
            CompensationMode::Off,
            feeds(),
            Some(vec(1.0, 0.0, 0.0)),
            Vec::new(),
            vec![
                ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
                ClBlock::Motion(ClMotion::Arc { to: end, plane: Plane::Xy, center, sense: ArcSense::Ccw, feed: 20.0, role: MoveRole::Cut }),
            ],
        )
        .expect("path");
        let mapped = must(map_toolpath_to_part(&path, &setup, 90.0, 0.01), "map");
        let ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, .. }) = &mapped.blocks()[1] else {
            panic!("arc was linearized");
        };
        assert_eq!(*plane, Plane::Zx);
        assert_eq!(*sense, ArcSense::Ccw);
        close(to.tip(), vec(0.0, 0.0, -1.0));
        close(*center, Vec3::ZERO);
        assert_eq!(to.rotary(), Some(90.0));
        close(to.axis(), Vec3::Y);
        let pierce = mapped.pierce().expect("pierce");
        close(pierce, vec(1.0, 0.0, 0.0));
        let start_tip = mapped.blocks()[0].motion_tip().expect("start").tip();
        let view = must(arc_to_machine(&setup, 90.0, start_tip, *to, *center, *plane, *sense), "view");
        assert_eq!(view.plane, Plane::Xy);
        assert_eq!(view.sense, ArcSense::Ccw);
        close(view.start, vec(1.0, 0.0, 0.0));
        close(view.end, vec(0.0, 1.0, 0.0));
        close(view.center, Vec3::ZERO);

        let program = Program::try_new(None, Vec::new(), None, vec![mapped], Vec::new(), Vec::new(), Some(setup)).expect("program");
        let doc = Document::new(program).expect("document");
        assert!(campanion_ir::verify(doc.program()).issues().is_empty(), "{:?}", campanion_ir::verify(doc.program()).issues());
    }

    #[test]
    fn a_non_cardinal_index_linearizes_and_keeps_the_end_point() {
        let setup = a_pos();
        let start = Pose::new(vec(10.0, 0.0, 0.0), Vec3::Z).expect("start");
        let end = Pose::new(vec(0.0, 10.0, 0.0), Vec3::Z).expect("end");
        let path = ClPath::try_new(
            Some("arc".into()),
            None,
            CompensationMode::Off,
            feeds(),
            None,
            Vec::new(),
            vec![
                ClBlock::Motion(ClMotion::Rapid { to: start, feed: None }),
                ClBlock::Motion(ClMotion::Arc {
                    to: end,
                    plane: Plane::Xy,
                    center: Vec3::ZERO,
                    sense: ArcSense::Ccw,
                    feed: 20.0,
                    role: MoveRole::Cut,
                }),
            ],
        )
        .expect("path");
        let mapped = must(map_toolpath_to_part(&path, &setup, 20.0, 0.05), "map");
        assert!(mapped.blocks().len() > 2, "expected chords, got {}", mapped.blocks().len());
        let ClBlock::Motion(ClMotion::Linear { to, .. }) = mapped.blocks().last().expect("end") else {
            panic!("end is not a line");
        };
        let back = part_to_machine(&setup, *to, 20.0);
        close(back.tip(), end.tip());
        assert_eq!(to.rotary(), Some(20.0));
        let program = Program::try_new(None, Vec::new(), None, vec![mapped], Vec::new(), Vec::new(), Some(setup)).expect("program");
        assert!(campanion_ir::verify(&program).issues().is_empty(), "{:?}", campanion_ir::verify(&program).issues());
    }

    #[test]
    fn frame_operations_inserts_the_change_between_zero_and_ninety() {
        let setup = a_pos();
        let paths =
            must(frame_operations(&setup, vec![op("face0", 0.0, vec(2.0, 3.0, 4.0)), op("face90", 90.0, vec(6.0, 1.0, 8.0))], 25.0, 0.01), "frame");
        assert_eq!(paths.len(), 2);
        assert_eq!(paths[0].name(), Some("face0"));
        assert_eq!(paths[1].name(), Some("face90"));
        let retract = last_pose(&paths[0]);
        let indexed = first_pose(&paths[1]);
        assert_eq!(retract.rotary(), Some(0.0));
        assert_eq!(indexed.rotary(), Some(90.0));
        close(part_to_machine(&setup, retract, 0.0).tip(), vec(2.0, 3.0, 25.0));
        close(part_to_machine(&setup, indexed, 90.0).tip(), vec(2.0, 3.0, 25.0));
        for block in paths[0].blocks() {
            assert_eq!(block_pose(block).and_then(|pose| pose.rotary()), Some(0.0));
        }
        for block in paths[1].blocks() {
            assert_eq!(block_pose(block).and_then(|pose| pose.rotary()), Some(90.0));
        }
    }

    #[test]
    fn a_first_operation_away_from_zero_gets_a_leading_retract() {
        let setup = a_pos();
        let paths = must(frame_operations(&setup, vec![op("side", 90.0, vec(4.0, 5.0, 1.0))], 12.0, 0.01), "frame");
        assert_eq!(paths.len(), 2);
        assert!(paths[0].name().is_none());
        assert_eq!(first_pose(&paths[0]).rotary(), Some(0.0));
        assert_eq!(paths[1].name(), Some("side"));
        for block in paths[1].blocks() {
            assert_eq!(block_pose(block).and_then(|pose| pose.rotary()), Some(90.0));
        }
        close(part_to_machine(&setup, first_pose(&paths[0]), 0.0).tip(), vec(4.0, 5.0, 12.0));
        close(part_to_machine(&setup, first_pose(&paths[1]), 90.0).tip(), vec(4.0, 5.0, 12.0));
    }

    fn wrapped(name: &str, poses: &[Pose]) -> IndexedOp {
        let blocks = poses.iter().map(|pose| ClBlock::Motion(ClMotion::Linear { to: *pose, feed: 100.0, role: MoveRole::Cut })).collect();
        let path = ClPath::try_new(Some(name.into()), Some(1), CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path");
        let angle = poses[0].rotary().expect("angle");
        IndexedOp { angle_deg: angle, tool_id: Some(1), feeds: feeds(), paths: vec![path], part_frame: true }
    }

    #[test]
    fn a_wrap_between_indexes_retracts_from_its_own_end_angle() {
        let setup = a_pos();
        let cylinder = campanion_ir::Cylinder::new(40.0).expect("cylinder");
        let start = campanion_ir::wrap_point(&setup, cylinder, 1.0, 0.0, 5.0).expect("start");
        let v90 = 0.25 * std::f64::consts::PI * 40.0;
        let end = campanion_ir::wrap_point(&setup, cylinder, 4.0, v90, 5.0).expect("end");
        let paths = must(
            frame_operations(
                &setup,
                vec![op("face0", 0.0, vec(2.0, 3.0, 4.0)), wrapped("around", &[start, end]), op("face180", 180.0, vec(8.0, 1.0, 2.0))],
                30.0,
                0.01,
            ),
            "frame",
        );
        assert_eq!(paths.len(), 3);
        assert_eq!(paths[0].blocks().len(), 1, "no retract while the angle stays 0");
        close(part_to_machine(&setup, last_pose(&paths[0]), 0.0).tip(), vec(2.0, 3.0, 4.0));
        assert_eq!(paths[1].blocks().len(), 3);
        close(part_to_machine(&setup, first_pose(&paths[1]), 0.0).tip(), vec(1.0, 0.0, 25.0));
        let wrap_end = block_pose(paths[1].blocks().get(1).expect("end")).expect("pose");
        assert!((wrap_end.rotary().expect("angle") - 90.0).abs() < 1e-9);
        close(part_to_machine(&setup, wrap_end, 90.0).tip(), vec(4.0, 0.0, 25.0));
        let retract = last_pose(&paths[1]);
        assert_eq!(retract.rotary(), Some(90.0));
        close(part_to_machine(&setup, retract, 90.0).tip(), vec(4.0, 0.0, 30.0));
        let indexed = first_pose(&paths[2]);
        assert_eq!(indexed.rotary(), Some(180.0));
        close(part_to_machine(&setup, indexed, 180.0).tip(), vec(4.0, 0.0, 30.0));
        close(part_to_machine(&setup, last_pose(&paths[2]), 180.0).tip(), vec(8.0, 1.0, 2.0));
    }

    #[test]
    fn a_wrap_that_starts_away_from_zero_gets_a_leading_retract() {
        let setup = a_pos();
        let cylinder = campanion_ir::Cylinder::new(40.0).expect("cylinder");
        let v90 = 0.25 * std::f64::consts::PI * 40.0;
        let pose = campanion_ir::wrap_point(&setup, cylinder, 4.0, v90, 5.0).expect("pose");
        let paths = must(frame_operations(&setup, vec![wrapped("side", &[pose])], 30.0, 0.01), "frame");
        assert_eq!(paths.len(), 2);
        assert!(paths[0].name().is_none());
        assert_eq!(first_pose(&paths[0]).rotary(), Some(0.0));
        close(part_to_machine(&setup, first_pose(&paths[0]), 0.0).tip(), vec(4.0, 0.0, 30.0));
        assert_eq!(paths[1].name(), Some("side"));
        assert_eq!(first_pose(&paths[1]).rotary(), Some(90.0));
        close(part_to_machine(&setup, first_pose(&paths[1]), 90.0).tip(), vec(4.0, 0.0, 30.0));
        let cut = last_pose(&paths[1]);
        assert!((cut.rotary().expect("angle") - 90.0).abs() < 1e-9);
        close(part_to_machine(&setup, cut, 90.0).tip(), vec(4.0, 0.0, 25.0));
    }

    #[test]
    fn quarter_turn_rotates_a_triangle_about_a() {
        let setup = a_pos();
        let mesh = Mesh::new(vec![Triangle { a: MeshVec::new(0.0, 1.0, 0.0), b: MeshVec::new(0.0, 0.0, 1.0), c: MeshVec::new(1.0, 0.0, 0.0) }])
            .expect("mesh");
        let turned = must(rotate_mesh(&mesh, &setup, 90.0), "turn");
        let first = turned.triangles()[0];
        // A-positive +90°: (x, y, z) -> (x, -z, y).
        close(vec(first.a.x, first.a.y, first.a.z), vec(0.0, 0.0, 1.0));
        close(vec(first.b.x, first.b.y, first.b.z), vec(0.0, -1.0, 0.0));
        close(vec(first.c.x, first.c.y, first.c.z), vec(1.0, 0.0, 0.0));
    }

    fn first_pose(path: &ClPath) -> Pose {
        block_pose(path.blocks().first().expect("block")).expect("pose")
    }

    fn last_pose(path: &ClPath) -> Pose {
        block_pose(path.blocks().last().expect("block")).expect("pose")
    }

    trait MotionTip {
        fn motion_tip(&self) -> Option<Pose>;
    }

    impl MotionTip for ClBlock {
        fn motion_tip(&self) -> Option<Pose> {
            block_pose(self)
        }
    }

    fn arb_kind() -> impl Strategy<Value = RotaryAxisKind> {
        prop_oneof![Just(RotaryAxisKind::A), Just(RotaryAxisKind::B)]
    }

    fn arb_dir() -> impl Strategy<Value = RotaryDirection> {
        prop_oneof![Just(RotaryDirection::Positive), Just(RotaryDirection::Negative)]
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(80))]

        #[test]
        fn machine_view_pose_round_trips_through_the_part_frame(
            kind in arb_kind(),
            direction in arb_dir(),
            center in (-80.0f64..80.0, -80.0f64..80.0, -80.0f64..80.0),
            tip in (-80.0f64..80.0, -80.0f64..80.0, -80.0f64..80.0),
            axis in (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0),
            angle in -720.0f64..720.0,
        ) {
            let Ok(center) = Vec3::new(center.0, center.1, center.2) else { return Ok(()); };
            let Ok(setup) = RotarySetup::new(kind, center, direction, None) else { return Ok(()); };
            let Ok(tip) = Vec3::new(tip.0, tip.1, tip.2) else { return Ok(()); };
            let Ok(axis) = Vec3::new(axis.0, axis.1, axis.2) else { return Ok(()); };
            let Ok(machine) = Pose::new(tip, axis) else { return Ok(()); };
            let part = must(map_machine_pose(&setup, machine, angle), "map");
            let back = part_to_machine(&setup, part, angle);
            prop_assert!(dist(back.tip(), machine.tip()) <= 1e-8);
            prop_assert!(dist(back.axis(), machine.axis()) <= 1e-8);
            prop_assert_eq!(part.rotary(), Some(angle));
            let again = must(map_machine_pose(&setup, back, angle), "again");
            prop_assert!(dist(again.tip(), part.tip()) <= 1e-8);
            prop_assert!(dist(again.axis(), part.axis()) <= 1e-8);
        }
    }
}
