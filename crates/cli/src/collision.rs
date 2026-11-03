//! Collision check for a mill tool assembly against the finished part and the fixtures.
//!
//! Run after the gouge check, on every mill toolpath whose tool has an `assembly`. The flute is
//! the cutter body from the tip to `flute_length`. The shank, holder and head sit above that.
//! Those bodies, and the flute on a rapid, must not come closer than `collision.clearance` to the
//! part; no body may touch a fixture on any move. Touching exactly at the clearance is allowed.
//!
//! Sampling uses the IR poses in the part frame: the tip is lerped, the axis is slerped, and an
//! arc or helix is sampled along the curve. A non-TCP control interpolates the joints linearly,
//! so between samples the real path can differ from this; the sample density is the bound.
//! In-process stock is not modelled.

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, Plane, Pose, Program, Vec3 as IrVec};
use campanion_mesh::{Aabb, CollisionHit, CollisionIndex, Cylinder, Mesh, Sphere, Vec3 as MeshVec, collides, cylinder_aabb, parse_stl};

use crate::error::CliError;
use crate::jobfile::{CollisionSettings, Fixture, FixtureKind, HolderSegment, JobFile, JobTool, ToolAssembly, ToolShape, hash_bytes, join_job_path};
use crate::run::{read_bytes, warn};

const MAX_COLLISION_SAMPLES: usize = 5_000_000;
const TAU: f64 = std::f64::consts::TAU;

enum Obstacle<'a> {
    Part(&'a CollisionIndex<'a>),
    Mesh { index: &'a CollisionIndex<'a> },
    Box { bounds: Aabb },
}

struct LoadedFixture {
    name: String,
    mesh: Option<Mesh>,
    bounds: Option<Aabb>,
}

struct Body {
    name: &'static str,
    holder_index: Option<usize>,
    sphere: Option<Sphere>,
    cylinder: Option<Cylinder>,
    /// Flute on a feed against the part is owned by the gouge check.
    skip_part: bool,
}

/// Check every mill toolpath whose tool has an `assembly`. Tools without an assembly, and jobs
/// without fixtures and assembly, return immediately.
pub(crate) fn check_job(job: &JobFile, part: Option<&Mesh>, program: &Program) -> Result<(), CliError> {
    if !job.has_assembly() {
        return Ok(());
    }
    let settings = job.collision();
    let fixtures = load_fixtures(job)?;
    let part_index = match part {
        Some(mesh) => Some(CollisionIndex::new(mesh).map_err(|err| CliError::run(format!("collision: {err}")))?),
        None => None,
    };
    let mut fixture_indexes = Vec::new();
    for fixture in &fixtures {
        let index = match &fixture.mesh {
            Some(mesh) => Some(CollisionIndex::new(mesh).map_err(|err| CliError::run(format!("collision: {err}")))?),
            None => None,
        };
        fixture_indexes.push(index);
    }
    for (path_index, path) in program.toolpaths().iter().enumerate() {
        let Some(tool_id) = path.tool_id() else {
            continue;
        };
        let Some(tool) = job.tool(tool_id) else {
            continue;
        };
        let Some(assembly) = tool.assembly() else {
            continue;
        };
        if let Some(hit) = check_path(path, path_index, tool, assembly, settings, part_index.as_ref(), &fixtures, &fixture_indexes)? {
            return Err(CliError::run(hit));
        }
    }
    Ok(())
}

fn load_fixtures(job: &JobFile) -> Result<Vec<LoadedFixture>, CliError> {
    let mut out = Vec::new();
    for fixture in job.fixtures() {
        out.push(load_fixture(job, fixture)?);
    }
    Ok(out)
}

fn load_fixture(job: &JobFile, fixture: &Fixture) -> Result<LoadedFixture, CliError> {
    match fixture.kind() {
        FixtureKind::Stl { path, hash } => {
            let file = join_job_path(job, path);
            let bytes = read_bytes(&file)?;
            let digest = hash_bytes(&bytes);
            if digest != *hash {
                warn(&format!("fixture {} changed: stored hash {hash}, file hash {digest}", fixture.name()))?;
            }
            let mesh = parse_stl(&bytes).map_err(|err| CliError::run(format!("fixture `{}`: {err}", fixture.name())))?;
            Ok(LoadedFixture { name: fixture.name().to_string(), mesh: Some(mesh), bounds: None })
        }
        FixtureKind::Box { min, max } => {
            let bounds = Aabb::from_corners(MeshVec::new(min.x(), min.y(), min.z()), MeshVec::new(max.x(), max.y(), max.z()));
            Ok(LoadedFixture { name: fixture.name().to_string(), mesh: None, bounds: Some(bounds) })
        }
    }
}

fn check_path(
    path: &ClPath,
    path_index: usize,
    tool: &JobTool,
    assembly: &ToolAssembly,
    settings: CollisionSettings,
    part: Option<&CollisionIndex<'_>>,
    fixtures: &[LoadedFixture],
    fixture_indexes: &[Option<CollisionIndex<'_>>],
) -> Result<Option<String>, CliError> {
    let op_name = path.name().unwrap_or("");
    let mut cursor: Option<Pose> = None;
    let mut move_index = 0usize;
    let mut samples = 0usize;
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        let (end, rapid, arc) = motion_end(motion)?;
        let start = cursor.unwrap_or(end);
        if let Some(message) = check_move(
            op_name,
            path_index,
            move_index,
            start,
            end,
            rapid,
            arc,
            tool,
            assembly,
            settings,
            part,
            fixtures,
            fixture_indexes,
            &mut samples,
        )? {
            return Ok(Some(message));
        }
        cursor = Some(end);
        move_index += 1;
    }
    Ok(None)
}

struct ArcSample {
    center: IrVec,
    plane: Plane,
    sense: ArcSense,
    sweep: Option<f64>,
}

fn motion_end(motion: &ClMotion) -> Result<(Pose, bool, Option<ArcSample>), CliError> {
    match motion {
        ClMotion::Rapid { to, .. } => Ok((*to, true, None)),
        ClMotion::Linear { to, .. } | ClMotion::Thread { to, .. } => Ok((*to, false, None)),
        ClMotion::Arc { to, plane, center, sense, .. } => {
            Ok((*to, false, Some(ArcSample { center: *center, plane: *plane, sense: *sense, sweep: None })))
        }
        ClMotion::Helix { to, plane, center, sense, sweep, .. } => {
            Ok((*to, false, Some(ArcSample { center: *center, plane: *plane, sense: *sense, sweep: Some(*sweep) })))
        }
    }
}

fn check_move(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    start: Pose,
    end: Pose,
    rapid: bool,
    arc: Option<ArcSample>,
    tool: &JobTool,
    assembly: &ToolAssembly,
    settings: CollisionSettings,
    part: Option<&CollisionIndex<'_>>,
    fixtures: &[LoadedFixture],
    fixture_indexes: &[Option<CollisionIndex<'_>>],
    samples: &mut usize,
) -> Result<Option<String>, CliError> {
    let poses = sample_move(start, end, arc, settings)?;
    bump_samples(samples, poses.len())?;
    for pose in poses {
        if let Some(message) = check_pose(op_name, path_index, move_index, pose, rapid, tool, assembly, settings, part, fixtures, fixture_indexes)? {
            return Ok(Some(message));
        }
    }
    Ok(None)
}

fn bump_samples(total: &mut usize, add: usize) -> Result<(), CliError> {
    *total = match total.checked_add(add) {
        Some(sum) if sum <= MAX_COLLISION_SAMPLES => sum,
        _ => return Err(CliError::invalid(format!("too many collision samples: limit is {MAX_COLLISION_SAMPLES}"))),
    };
    Ok(())
}

/// Poses along one IR move, including both ends. Arcs and helices follow the curve. The tip is
/// lerped and the axis is slerped on a line. Rotary angles are lerped.
pub(crate) fn poses_along(start: Pose, end: Pose, motion: &ClMotion, settings: CollisionSettings) -> Result<Vec<Pose>, CliError> {
    let (_, _, arc) = motion_end(motion)?;
    sample_move(start, end, arc, settings)
}

fn sample_move(start: Pose, end: Pose, arc: Option<ArcSample>, settings: CollisionSettings) -> Result<Vec<Pose>, CliError> {
    match arc {
        Some(arc) => sample_arc(start, end, arc, settings),
        None => sample_linear(start, end, settings),
    }
}

fn sample_linear(start: Pose, end: Pose, settings: CollisionSettings) -> Result<Vec<Pose>, CliError> {
    let tip0 = mesh_tip(start);
    let tip1 = mesh_tip(end);
    let axis0 = mesh_axis(start)?;
    let axis1 = mesh_axis(end)?;
    let length = tip1.sub(tip0).length();
    let angle = axis_angle_deg(axis0, axis1);
    let count = sample_count(length, angle, settings.sample_step(), settings.max_angle_step_deg())?;
    let mut poses = Vec::with_capacity(count);
    for index in 0..count {
        let t = if count == 1 { 0.0 } else { index as f64 / (count - 1) as f64 };
        let tip = tip0.lerp(tip1, t);
        let axis = slerp(axis0, axis1, t)?;
        poses.push(pose_from_mesh(tip, axis, start, end, t)?);
    }
    Ok(poses)
}

fn sample_arc(start: Pose, end: Pose, arc: ArcSample, settings: CollisionSettings) -> Result<Vec<Pose>, CliError> {
    let (u, v, n) = plane_basis(arc.plane);
    let s = ir_to_mesh(start.tip());
    let e = ir_to_mesh(end.tip());
    let c = ir_to_mesh(arc.center);
    let su = s.sub(c).dot(u);
    let sv = s.sub(c).dot(v);
    let eu = e.sub(c).dot(u);
    let ev = e.sub(c).dot(v);
    let radius = su.hypot(sv);
    let end_radius = eu.hypot(ev);
    if !radius.is_finite() || radius <= 1e-9 {
        return Err(CliError::run("collision: arc radius is not usable"));
    }
    if (radius - end_radius).abs() > 1.0e-3_f64.max(radius * 1.0e-4) {
        return Err(CliError::run("collision: arc endpoints are not on one circle"));
    }
    let a0 = sv.atan2(su);
    let a1 = ev.atan2(eu);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(CliError::run("collision: arc angle is not finite"));
    }
    let ccw = arc.sense == ArcSense::Ccw;
    let sweep = match arc.sweep {
        Some(sweep) if sweep.is_finite() && sweep > 0.0 => sweep,
        Some(_) => return Err(CliError::run("collision: arc sweep is not usable")),
        None => xy_sweep(a0, a1, s.sub(e).length(), radius, ccw)?,
    };
    let sn = s.sub(c).dot(n);
    let en = e.sub(c).dot(n);
    let length = (radius * sweep).hypot(en - sn);
    let axis0 = mesh_axis(start)?;
    let axis1 = mesh_axis(end)?;
    let angle = axis_angle_deg(axis0, axis1);
    let count = sample_count(length, angle, settings.sample_step(), settings.max_angle_step_deg())?;
    let signed = if ccw { sweep } else { -sweep };
    let mut poses = Vec::with_capacity(count);
    for index in 0..count {
        let t = if count == 1 { 0.0 } else { index as f64 / (count - 1) as f64 };
        let tip = if index == 0 {
            s
        } else if index + 1 == count {
            e
        } else {
            let ang = a0 + signed * t;
            let along = n.scale(sn + (en - sn) * t);
            c.add(u.scale(radius * ang.cos())).add(v.scale(radius * ang.sin())).add(along)
        };
        let axis = slerp(axis0, axis1, t)?;
        poses.push(pose_from_mesh(tip, axis, start, end, t)?);
    }
    Ok(poses)
}

fn xy_sweep(a0: f64, a1: f64, chord: f64, radius: f64, ccw: bool) -> Result<f64, CliError> {
    let closure = 1e-9_f64.max(radius * 1e-9);
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= closure {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(CliError::run("collision: arc sweep is not usable")) }
}

fn plane_basis(plane: Plane) -> (MeshVec, MeshVec, MeshVec) {
    match plane {
        Plane::Xy => (MeshVec::new(1.0, 0.0, 0.0), MeshVec::new(0.0, 1.0, 0.0), MeshVec::new(0.0, 0.0, 1.0)),
        Plane::Zx => (MeshVec::new(0.0, 0.0, 1.0), MeshVec::new(1.0, 0.0, 0.0), MeshVec::new(0.0, 1.0, 0.0)),
        Plane::Yz => (MeshVec::new(0.0, 1.0, 0.0), MeshVec::new(0.0, 0.0, 1.0), MeshVec::new(1.0, 0.0, 0.0)),
    }
}

fn sample_count(length: f64, angle_deg: f64, step: f64, max_angle_deg: f64) -> Result<usize, CliError> {
    if !length.is_finite() || length < 0.0 {
        return Err(CliError::run("collision: sample length is not finite"));
    }
    let mut intervals = 1.0_f64;
    if length > 0.0 {
        intervals = intervals.max((length / step).ceil());
    }
    if angle_deg > 0.0 {
        intervals = intervals.max((angle_deg / max_angle_deg).ceil());
    }
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(CliError::invalid(format!("too many collision samples: limit is {MAX_COLLISION_SAMPLES}")));
    }
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(CliError::run("collision: sample spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_COLLISION_SAMPLES as f64 {
        return Err(CliError::invalid(format!("too many collision samples: limit is {MAX_COLLISION_SAMPLES}")));
    }
    Ok(count as usize)
}

fn axis_angle_deg(a: MeshVec, b: MeshVec) -> f64 {
    let dot = a.dot(b).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

fn slerp(a: MeshVec, b: MeshVec, t: f64) -> Result<MeshVec, CliError> {
    let dot = a.dot(b).clamp(-1.0, 1.0);
    if dot > 0.9995 {
        return a.lerp(b, t).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"));
    }
    if dot < -0.9995 {
        let helper = if a.x.abs() < 0.9 { MeshVec::new(1.0, 0.0, 0.0) } else { MeshVec::new(0.0, 1.0, 0.0) };
        let ortho = a.cross(helper).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"))?;
        let ang = std::f64::consts::PI * t;
        return a.scale(ang.cos()).add(ortho.scale(ang.sin())).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"));
    }
    let omega = dot.acos();
    let sin = omega.sin();
    if sin.abs() <= 1e-18 {
        return a.lerp(b, t).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"));
    }
    let wa = ((1.0 - t) * omega).sin() / sin;
    let wb = (t * omega).sin() / sin;
    a.scale(wa).add(b.scale(wb)).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"))
}

fn mesh_tip(pose: Pose) -> MeshVec {
    ir_to_mesh(pose.tip())
}

fn mesh_axis(pose: Pose) -> Result<MeshVec, CliError> {
    ir_to_mesh(pose.axis()).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"))
}

fn ir_to_mesh(v: IrVec) -> MeshVec {
    MeshVec::new(v.x(), v.y(), v.z())
}

fn pose_from_mesh(tip: MeshVec, axis: MeshVec, start: Pose, end: Pose, t: f64) -> Result<Pose, CliError> {
    let tip = IrVec::new(tip.x, tip.y, tip.z).map_err(|err| CliError::run(format!("collision: {err}")))?;
    let axis = IrVec::new(axis.x, axis.y, axis.z).map_err(|err| CliError::run(format!("collision: {err}")))?;
    let mut pose = Pose::new(tip, axis).map_err(|err| CliError::run(format!("collision: {err}")))?;
    if let (Some(a), Some(b)) = (start.rotary(), end.rotary()) {
        pose = pose.with_rotary(a + (b - a) * t).map_err(|err| CliError::run(format!("collision: {err}")))?;
    }
    Ok(pose)
}

fn check_pose(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    pose: Pose,
    rapid: bool,
    tool: &JobTool,
    assembly: &ToolAssembly,
    settings: CollisionSettings,
    part: Option<&CollisionIndex<'_>>,
    fixtures: &[LoadedFixture],
    fixture_indexes: &[Option<CollisionIndex<'_>>],
) -> Result<Option<String>, CliError> {
    let bodies = bodies_at(pose, tool, assembly, rapid)?;
    let clearance = settings.clearance();
    for body in &bodies {
        if !body.skip_part
            && let Some(part) = part
            && let Some(hit) = query_body(body, Obstacle::Part(part), clearance)?
        {
            return Ok(Some(format_hit(op_name, path_index, move_index, body, "part", hit)));
        }
        for (fixture, index) in fixtures.iter().zip(fixture_indexes.iter()) {
            let obstacle = if let Some(index) = index {
                Obstacle::Mesh { index }
            } else if let Some(bounds) = fixture.bounds {
                Obstacle::Box { bounds }
            } else {
                continue;
            };
            if let Some(hit) = query_body(body, obstacle, clearance)? {
                let obstacle = format!("fixture `{}`", fixture.name);
                return Ok(Some(format_hit(op_name, path_index, move_index, body, &obstacle, hit)));
            }
        }
    }
    Ok(None)
}

fn query_body(body: &Body, obstacle: Obstacle<'_>, clearance: f64) -> Result<Option<CollisionHit>, CliError> {
    match obstacle {
        Obstacle::Part(index) | Obstacle::Mesh { index } => query_index(body, index, clearance),
        Obstacle::Box { bounds } => query_box(body, bounds, clearance),
    }
}

fn query_index(body: &Body, index: &CollisionIndex<'_>, clearance: f64) -> Result<Option<CollisionHit>, CliError> {
    let mut best: Option<CollisionHit> = None;
    if let Some(sphere) = body.sphere {
        take_hit(&mut best, index.query_sphere(sphere, clearance).map_err(|err| CliError::run(format!("collision: {err}")))?);
    }
    if let Some(cylinder) = body.cylinder {
        take_hit(&mut best, index.query_cylinder(cylinder, clearance).map_err(|err| CliError::run(format!("collision: {err}")))?);
    }
    Ok(best)
}

fn query_box(body: &Body, bounds: Aabb, clearance: f64) -> Result<Option<CollisionHit>, CliError> {
    let mut best: Option<CollisionHit> = None;
    if let Some(sphere) = body.sphere {
        let triangle_hits = bounds.face_triangles();
        for triangle in triangle_hits {
            if let Some(hit) = campanion_mesh::sphere_triangle(sphere, triangle)
                && collides(hit.distance, clearance)
            {
                take_hit(&mut best, Some(hit));
            }
        }
        if bounds.contains(sphere.center) {
            take_hit(&mut best, Some(CollisionHit { point: sphere.center, distance: -sphere.radius }));
        }
    }
    if let Some(cylinder) = body.cylinder
        && let Some(hit) = cylinder_aabb(cylinder, bounds)
        && collides(hit.distance, clearance)
    {
        take_hit(&mut best, Some(hit));
    }
    Ok(best)
}

fn take_hit(best: &mut Option<CollisionHit>, hit: Option<CollisionHit>) {
    let Some(hit) = hit else {
        return;
    };
    match *best {
        Some(current) if current.distance <= hit.distance => {}
        _ => *best = Some(hit),
    }
}

fn bodies_at(pose: Pose, tool: &JobTool, assembly: &ToolAssembly, rapid: bool) -> Result<Vec<Body>, CliError> {
    let tip = mesh_tip(pose);
    let axis = mesh_axis(pose)?;
    let Some(diameter) = tool.diameter() else {
        return Err(CliError::run("collision: assembly is not valid on a kerf tool"));
    };
    let tool_r = diameter * 0.5;
    let stickout = assembly.stickout();
    let flute = assembly.flute_length();
    let mut bodies = Vec::new();
    match tool.shape() {
        ToolShape::Ball => {
            let center = tip.add(axis.scale(tool_r));
            let sphere = Sphere::new(center, tool_r).ok_or_else(|| CliError::run("collision: flute sphere is not usable"))?;
            let cylinder = if flute > tool_r + 1e-12 { Cylinder::new(tip.add(axis.scale(tool_r)), tip.add(axis.scale(flute)), tool_r) } else { None };
            bodies.push(Body {
                name: if rapid { "flute (rapid)" } else { "flute" },
                holder_index: None,
                sphere: Some(sphere),
                cylinder,
                skip_part: !rapid,
            });
        }
        ToolShape::Flat | ToolShape::Bull { .. } => {
            if flute > 1e-12 {
                let cylinder =
                    Cylinder::new(tip, tip.add(axis.scale(flute)), tool_r).ok_or_else(|| CliError::run("collision: flute is not usable"))?;
                bodies.push(Body {
                    name: if rapid { "flute (rapid)" } else { "flute" },
                    holder_index: None,
                    sphere: None,
                    cylinder: Some(cylinder),
                    skip_part: !rapid,
                });
            }
        }
    }
    if stickout - flute > 1e-12 {
        let shank_r = assembly.shank_diameter() * 0.5;
        let cylinder = Cylinder::new(tip.add(axis.scale(flute)), tip.add(axis.scale(stickout)), shank_r)
            .ok_or_else(|| CliError::run("collision: shank is not usable"))?;
        bodies.push(Body { name: "shank", holder_index: None, sphere: None, cylinder: Some(cylinder), skip_part: false });
    }
    let mut along = stickout;
    for (index, segment) in assembly.holder().iter().enumerate() {
        bodies.push(stack_cylinder(tip, axis, along, segment, None, index + 1)?);
        along += segment.length();
    }
    if let Some(head) = assembly.head() {
        bodies.push(stack_cylinder(tip, axis, along, &head, Some("head"), 0)?);
    }
    Ok(bodies)
}

fn stack_cylinder(
    tip: MeshVec,
    axis: MeshVec,
    start: f64,
    segment: &HolderSegment,
    name: Option<&'static str>,
    holder_index: usize,
) -> Result<Body, CliError> {
    let a = tip.add(axis.scale(start));
    let b = tip.add(axis.scale(start + segment.length()));
    let cylinder = Cylinder::new(a, b, segment.diameter() * 0.5).ok_or_else(|| CliError::run("collision: holder is not usable"))?;
    Ok(Body {
        name: name.unwrap_or("holder"),
        holder_index: if name.is_some() { None } else { Some(holder_index) },
        sphere: None,
        cylinder: Some(cylinder),
        skip_part: false,
    })
}

fn body_label(body: &Body) -> String {
    match body.holder_index {
        Some(index) => format!("holder {index}"),
        None => body.name.to_string(),
    }
}

fn format_hit(op_name: &str, path_index: usize, move_index: usize, body: &Body, obstacle: &str, hit: CollisionHit) -> String {
    format!(
        "operation `{op_name}` toolpath {path_index} move {move_index}: {} hits {obstacle} at ({:.3}, {:.3}, {:.3})",
        body_label(body),
        hit.point.x,
        hit.point.y,
        hit.point.z
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{CompensationMode, Feeds, MoveRole, Tool, Vec3};
    use campanion_mesh::{Mesh, Triangle};

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn pose_axis(x: f64, y: f64, z: f64, i: f64, j: f64, k: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::new(i, j, k).expect("axis")).expect("pose")
    }

    #[test]
    fn message_names_the_body_obstacle_and_point() {
        let body = Body { name: "holder", holder_index: Some(1), sphere: None, cylinder: None, skip_part: false };
        let hit = CollisionHit { point: MeshVec::new(12.3, -4.0, 8.1), distance: -0.2 };
        assert_eq!(
            format_hit("dome", 0, 37, &body, "fixture `vise`", hit),
            "operation `dome` toolpath 0 move 37: holder 1 hits fixture `vise` at (12.300, -4.000, 8.100)"
        );
        let flute = Body { name: "flute (rapid)", holder_index: None, sphere: None, cylinder: None, skip_part: false };
        let text = format_hit("dome", 1, 0, &flute, "part", hit);
        assert_eq!(text, "operation `dome` toolpath 1 move 0: flute (rapid) hits part at (12.300, -4.000, 8.100)");
    }

    #[test]
    fn linear_samples_include_both_ends_and_bound_the_tip_and_axis() -> Result<(), String> {
        let settings = CollisionSettings::default_values();
        let start = pose(0.0, 0.0, 0.0);
        let end = pose(3.0, 0.0, 0.0);
        let poses = sample_linear(start, end, settings).map_err(|err| err.to_string())?;
        assert!(poses.len() >= 4, "{}", poses.len());
        let first = poses.first().expect("first").tip();
        let last = poses.last().expect("last").tip();
        assert!((first.x() - 0.0).abs() < 1e-12 && (last.x() - 3.0).abs() < 1e-12);
        let tilted = sample_linear(pose(0.0, 0.0, 0.0), pose_axis(0.0, 0.0, 0.0, 0.2, 0.0, 1.0), settings).map_err(|err| err.to_string())?;
        assert!(tilted.len() >= 2);
        Ok(())
    }

    #[test]
    fn jobs_without_assembly_do_not_check() -> Result<(), String> {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(
            Some("dome".into()),
            Some(1),
            CompensationMode::Off,
            feeds,
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 5.0), feed: None })],
        )
        .expect("path");
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        let program = Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let text = r#"{"format":"campanion-job/1","source":{"path":"part.stl","hash":"cbf29ce484222325"},"units":"mm","tools":[{"id":1,"diameter":6,"shape":"ball"}],"operations":[{"name":"dome","type":"surface","tool_id":1,"stepover":2,"feeds":{"cut":100}}]}"#;
        let dir = std::env::temp_dir().join("campanion-collision-skip");
        std::fs::create_dir_all(&dir).expect("dir");
        let file = dir.join("job.json");
        std::fs::write(&file, text).expect("write");
        let job = crate::jobfile::load(&file).expect("load");
        let mesh = Mesh::new(vec![Triangle { a: MeshVec::new(0.0, 0.0, 0.0), b: MeshVec::new(1.0, 0.0, 0.0), c: MeshVec::new(0.0, 1.0, 0.0) }])
            .expect("mesh");
        check_job(&job, Some(&mesh), &program).map_err(|err| err.to_string())?;
        Ok(())
    }

    #[test]
    fn holder_against_a_box_fixture_names_the_move() -> Result<(), String> {
        let text = concat!(
            r#"{"format":"campanion-job/1","source":{"path":"part.stl","hash":"cbf29ce484222325"},"units":"mm","#,
            r#""tools":[{"id":1,"diameter":6,"shape":"flat","assembly":{"stickout":5,"holder":[{"diameter":20,"length":10}]}}],"#,
            r#""operations":[{"name":"cut","type":"surface","tool_id":1,"stepover":2,"feeds":{"cut":100}}],"#,
            r#""fixtures":[{"name":"vise","box":{"min":[-5,-5,8],"max":[5,5,20]}}]}"#
        );
        let dir = std::env::temp_dir().join("campanion-collision-holder");
        std::fs::create_dir_all(&dir).expect("dir");
        let file = dir.join("job.json");
        std::fs::write(&file, text).expect("write");
        let job = crate::jobfile::load(&file).expect("load");
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 0.0), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(0.0, 0.0, 0.0), feed: 100.0, role: MoveRole::Cut }),
        ];
        let path = ClPath::try_new(Some("cut".into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let tool = Tool::try_new(1, None, Some(6.0), None).expect("tool");
        let program = Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let err = match check_job(&job, None, &program) {
            Err(err) => err,
            Ok(()) => return Err("expected a holder collision".into()),
        };
        let text = err.to_string();
        assert!(text.contains("operation `cut`"), "{text}");
        assert!(text.contains("holder 1"), "{text}");
        assert!(text.contains("fixture `vise`"), "{text}");
        assert!(text.contains("move "), "{text}");
        Ok(())
    }
}
