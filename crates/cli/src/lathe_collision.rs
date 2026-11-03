//! Collision check for mill-turn live tools and insert holders.
//!
//! Runs after the gouge check, and only when the job has `chuck`, `sub_chuck`, or a live-tool
//! `assembly`. The part is the finished profile revolved about the spindle. C does not change a
//! solid of revolution, so a body is tested in the stored pose. The tip is left where the path
//! stored it.
//!
//! An axial cylinder (axis parallel to Z) is an exact rectangle in `(radius, Z)`. A radial
//! cylinder (axis parallel to X) is the `(radius, Z)` band of that flat-capped cylinder, refined
//! until the bounds agree within 1e-9 mm. Any other axis is an error. Insert holders and inserts
//! are half-section polygons in `(Z, radius)`.
//!
//! The chuck face is at `front_allowance - stock.length` on the main spindle (finished face Z 0
//! minus the stock length, plus the front allowance). The sub chuck face is
//! `back_allowance - stock.length` in the sub part frame. The cylinder runs from that face toward
//! −Z. While a steady rest is clamped, its jaws are a ring over the existing Z band, from the
//! finished radius out to 1e6 mm. `collision.clearance` is an extra keep-out on top of that band.
//! The tailstock block has a diameter and a clearance and no length, so this check builds no
//! quill solid. In-process stock is not modelled.

use campanion_ir::{ClBlock, ClMotion, ClPath, Pose, Program};
use campanion_mesh::{CollisionHit, Cylinder, RevolvedSolid, Vec3 as MeshVec, collides, convex_polygon_distance};
use campanion_turn::{HolderGeom, InsertGeom, TipPoint, TurnProfile, holder_corners};

use crate::collision::poses_along;
use crate::error::CliError;
use crate::jobfile::{CollisionSettings, HolderSegment, ToolAssembly};

use super::{ChuckBody, LatheJob, Operation, SteadyRestBlock, ToolBody, WorkSpindle, stock_length};

const MAX_COLLISION_SAMPLES: usize = 5_000_000;
/// Outer radius of the steady-rest jaw ring, millimetres.
const STEADY_OUTER_MM: f64 = 1.0e6;

struct SectionBody {
    name: &'static str,
    /// Closed section in `(Z, radius)`, without a repeated closing vertex.
    section: Vec<(f64, f64)>,
    solid: RevolvedSolid,
}

struct CylBody {
    label: String,
    cylinder: Cylinder,
    /// Flute on a feed against the part is allowed. The flute still may not touch a chuck or jaws.
    skip_part: bool,
}

struct Scene {
    main_part: RevolvedSolid,
    sub_part: Option<RevolvedSolid>,
    main_chuck: Option<SectionBody>,
    sub_chuck: Option<SectionBody>,
    steady: Option<SectionBody>,
}

/// First collision on each toolpath. `Ok` when the job has nothing to test, or when every body clears.
pub(crate) fn check(job: &LatheJob, profile: &TurnProfile, sub_profile: Option<&TurnProfile>, program: &Program) -> Result<(), CliError> {
    if !enabled(job) {
        return Ok(());
    }
    let scene = Scene {
        main_part: revolved(profile, job.chord)?,
        sub_part: match sub_profile {
            Some(profile) => Some(revolved(profile, job.chord)?),
            None => None,
        },
        main_chuck: match &job.chuck {
            Some(body) => Some(chuck_body(job, WorkSpindle::Main, body)?),
            None => None,
        },
        sub_chuck: match &job.sub_chuck {
            Some(body) => Some(chuck_body(job, WorkSpindle::Sub, body)?),
            None => None,
        },
        steady: match &job.steady_rest {
            Some(rest) => steady_body(profile, rest, job.chord),
            None => None,
        },
    };
    let paths = program.toolpaths();
    let mut index = 0usize;
    let mut steady_clamped = false;
    for operation in &job.operations {
        if operation.is_wait() {
            index = index.checked_add(2).ok_or_else(|| CliError::run("collision: toolpath index overflow"))?;
            continue;
        }
        if let Operation::SteadyRest { clamp, .. } = operation {
            steady_clamped = *clamp;
        }
        let Some(path) = paths.get(index) else {
            return Err(CliError::run("collision: toolpath count does not match the job"));
        };
        if let Some(message) = check_operation(job, operation, path, index, steady_clamped, &scene)? {
            return Err(CliError::run(message));
        }
        index += 1;
    }
    if index != paths.len() {
        return Err(CliError::run("collision: toolpath count does not match the job"));
    }
    Ok(())
}

fn enabled(job: &LatheJob) -> bool {
    job.chuck.is_some() || job.sub_chuck.is_some() || job.tools.iter().any(|tool| matches!(tool.body, ToolBody::Live { assembly: Some(_), .. }))
}

fn revolved(profile: &TurnProfile, chord: f64) -> Result<RevolvedSolid, CliError> {
    let section: Vec<(f64, f64)> = profile.to_polyline_points(chord).into_iter().map(|point| (point.x(), point.y())).collect();
    RevolvedSolid::try_from_section(&section).ok_or_else(|| CliError::run("collision: finished profile is not a solid"))
}

fn chuck_body(job: &LatheJob, spindle: WorkSpindle, body: &ChuckBody) -> Result<SectionBody, CliError> {
    let face = super::front_allowance(job, spindle)? - stock_length(&job.stock);
    let radius = body.diameter * 0.5;
    let back = face - body.length;
    let section = vec![(face, 0.0), (face, radius), (back, radius), (back, 0.0)];
    let name = match spindle {
        WorkSpindle::Main => "chuck",
        WorkSpindle::Sub => "sub chuck",
    };
    section_body(name, section)
}

fn steady_body(profile: &TurnProfile, rest: &SteadyRestBlock, chord: f64) -> Option<SectionBody> {
    let section = steady_section(profile, rest, chord)?;
    section_body("steady rest", section).ok()
}

fn section_body(name: &'static str, section: Vec<(f64, f64)>) -> Result<SectionBody, CliError> {
    let solid = RevolvedSolid::try_from_polygon(&section).ok_or_else(|| CliError::run(format!("collision: {name} is not a solid")))?;
    Ok(SectionBody { name, section, solid })
}

/// Jaw ring over the steady-rest band where that band overlaps the finished profile.
///
/// The band is `[Z - width / 2 - clearance, Z + width / 2 + clearance]`. Radius runs from the
/// finished profile out to [`STEADY_OUTER_MM`].
fn steady_section(profile: &TurnProfile, rest: &SteadyRestBlock, chord: f64) -> Option<Vec<(f64, f64)>> {
    let (band_lo, band_hi) = super::steady_rest_band(rest);
    let (part_lo, part_hi) = profile.z_range();
    let z_lo = band_lo.max(part_lo);
    let z_hi = band_hi.min(part_hi);
    if !(z_hi - z_lo > 1e-9) {
        return None;
    }
    let r_hi = profile.radius_at(z_hi)?;
    let r_lo = profile.radius_at(z_lo)?;
    let mut poly: Vec<(f64, f64)> = profile.to_polyline_points(chord).into_iter().map(|point| (point.x(), point.y())).collect();
    if poly.len() >= 2 && poly[0].0 < poly[poly.len() - 1].0 {
        poly.reverse();
    }
    let mut floor = Vec::new();
    push_apart(&mut floor, (z_hi, r_hi));
    for point in poly {
        if point.0 < z_hi - 1e-9 && point.0 > z_lo + 1e-9 {
            push_apart(&mut floor, point);
        }
    }
    push_apart(&mut floor, (z_lo, r_lo));
    if floor.len() < 2 {
        return None;
    }
    let start_z = floor[0].0;
    let end_z = floor[floor.len() - 1].0;
    floor.push((end_z, STEADY_OUTER_MM));
    floor.push((start_z, STEADY_OUTER_MM));
    Some(floor)
}

fn push_apart(points: &mut Vec<(f64, f64)>, point: (f64, f64)) {
    if points.last().is_none_or(|prev| (prev.0 - point.0).hypot(prev.1 - point.1) > 1e-9) {
        points.push(point);
    }
}

fn check_operation(
    job: &LatheJob,
    operation: &Operation,
    path: &ClPath,
    path_index: usize,
    steady_clamped: bool,
    scene: &Scene,
) -> Result<Option<String>, CliError> {
    let Some(tool_id) = operation.checked_tool() else {
        return Ok(None);
    };
    let Some(tool) = job.tools.iter().find(|tool| tool.id == tool_id) else {
        return Ok(None);
    };
    match &tool.body {
        ToolBody::Live { assembly: Some(assembly), diameter, .. } => {
            check_live(job, operation, path, path_index, *diameter, assembly, steady_clamped, scene)
        }
        ToolBody::Live { assembly: None, .. } => Ok(None),
        ToolBody::Insert { .. } => check_insert(job, operation, path, path_index, steady_clamped, scene),
    }
}

fn check_live(
    job: &LatheJob,
    operation: &Operation,
    path: &ClPath,
    path_index: usize,
    diameter: f64,
    assembly: &ToolAssembly,
    steady_clamped: bool,
    scene: &Scene,
) -> Result<Option<String>, CliError> {
    let part = match operation.spindle() {
        WorkSpindle::Main => &scene.main_part,
        WorkSpindle::Sub => scene.sub_part.as_ref().ok_or_else(|| CliError::run("collision: sub profile is missing"))?,
    };
    let chuck = chuck_of(operation.spindle(), scene);
    let steady = steady_of(operation.spindle(), steady_clamped, scene);
    let clearance = job.collision.clearance();
    let op_name = operation.name();
    walk_moves(path, job.collision, |move_index, pose, rapid| {
        let bodies = live_bodies(pose, diameter, assembly, rapid)?;
        query_bodies(op_name, path_index, move_index, &bodies, part, chuck, steady, clearance)
    })
}

fn check_insert(
    job: &LatheJob,
    operation: &Operation,
    path: &ClPath,
    path_index: usize,
    steady_clamped: bool,
    scene: &Scene,
) -> Result<Option<String>, CliError> {
    let chuck = chuck_of(operation.spindle(), scene);
    let steady = steady_of(operation.spindle(), steady_clamped, scene);
    if chuck.is_none() && steady.is_none() {
        return Ok(None);
    }
    let (insert, tip, _nose, holder) = super::planning_insert(job, operation)?;
    let geom = InsertGeom::from_insert(&insert, super::TURN_EDGE_MM).map_err(|err| CliError::run(format!("collision: {err}")))?;
    if chuck.is_none() && holder.is_none() {
        return Ok(None);
    }
    let clearance = job.collision.clearance();
    let chord = job.chord;
    let op_name = operation.name();
    walk_moves(path, job.collision, |move_index, pose, _rapid| {
        check_insert_pose(op_name, path_index, move_index, pose, tip, &geom, holder, chuck, steady, clearance, chord)
    })
}

fn check_insert_pose(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    pose: Pose,
    tip: TipPoint,
    geom: &InsertGeom,
    holder: Option<&HolderGeom>,
    chuck: Option<&SectionBody>,
    steady: Option<&SectionBody>,
    clearance: f64,
    chord: f64,
) -> Result<Option<String>, CliError> {
    let centre = nose_centre(pose, tip);
    if let Some(holder) = holder {
        let corners = holder_corners(geom, holder, centre).map_err(|err| CliError::run(format!("collision: {err}")))?;
        if let Some(chuck) = chuck
            && let Some(message) = polygon_hit(op_name, path_index, move_index, "holder", &corners, chuck, clearance)
        {
            return Ok(Some(message));
        }
        if let Some(steady) = steady
            && let Some(message) = polygon_hit(op_name, path_index, move_index, "holder", &corners, steady, clearance)
        {
            return Ok(Some(message));
        }
    }
    if let Some(chuck) = chuck {
        let outline = geom.outline(centre, chord);
        if outline.len() >= 3
            && let Some(message) = polygon_hit(op_name, path_index, move_index, "insert", &outline, chuck, clearance)
        {
            return Ok(Some(message));
        }
    }
    Ok(None)
}

fn chuck_of(spindle: WorkSpindle, scene: &Scene) -> Option<&SectionBody> {
    match spindle {
        WorkSpindle::Main => scene.main_chuck.as_ref(),
        WorkSpindle::Sub => scene.sub_chuck.as_ref(),
    }
}

fn steady_of(spindle: WorkSpindle, clamped: bool, scene: &Scene) -> Option<&SectionBody> {
    if clamped && spindle == WorkSpindle::Main { scene.steady.as_ref() } else { None }
}

fn nose_centre(pose: Pose, tip: TipPoint) -> (f64, f64) {
    let z = pose.tip().z();
    let radius = pose.tip().x();
    match tip {
        TipPoint::Centre => (z, radius),
        TipPoint::ImaginaryTip { dz, dx } => (z - dz, radius - dx),
    }
}

fn walk_moves(
    path: &ClPath,
    settings: CollisionSettings,
    mut visit: impl FnMut(usize, Pose, bool) -> Result<Option<String>, CliError>,
) -> Result<Option<String>, CliError> {
    let mut cursor = None;
    let mut move_index = 0usize;
    let mut samples = 0usize;
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        let (end, rapid) = motion_pose(motion);
        let start = cursor.unwrap_or(end);
        let poses = poses_along(start, end, motion, settings)?;
        add_samples(&mut samples, poses.len())?;
        for pose in poses {
            if let Some(message) = visit(move_index, pose, rapid)? {
                return Ok(Some(message));
            }
        }
        cursor = Some(end);
        move_index += 1;
    }
    Ok(None)
}

fn motion_pose(motion: &ClMotion) -> (Pose, bool) {
    match motion {
        ClMotion::Rapid { to, .. } => (*to, true),
        ClMotion::Linear { to, .. } | ClMotion::Arc { to, .. } | ClMotion::Helix { to, .. } | ClMotion::Thread { to, .. } => (*to, false),
    }
}

fn add_samples(total: &mut usize, add: usize) -> Result<(), CliError> {
    *total = match total.checked_add(add) {
        Some(sum) if sum <= MAX_COLLISION_SAMPLES => sum,
        _ => return Err(CliError::invalid(format!("too many collision samples: limit is {MAX_COLLISION_SAMPLES}"))),
    };
    Ok(())
}

fn live_bodies(pose: Pose, diameter: f64, assembly: &ToolAssembly, rapid: bool) -> Result<Vec<CylBody>, CliError> {
    let tip = mesh_vec(pose.tip());
    let axis = mesh_vec(pose.axis()).normalized().ok_or_else(|| CliError::run("collision: tool axis is not usable"))?;
    let tool_r = diameter * 0.5;
    let stickout = assembly.stickout();
    let flute = assembly.flute_length();
    let mut bodies = Vec::new();
    if flute > 1e-12 {
        let cylinder = Cylinder::new(tip, tip.add(axis.scale(flute)), tool_r).ok_or_else(|| CliError::run("collision: flute is not usable"))?;
        bodies.push(CylBody { label: if rapid { "flute (rapid)".to_string() } else { "flute".to_string() }, cylinder, skip_part: !rapid });
    }
    if stickout - flute > 1e-12 {
        let cylinder = Cylinder::new(tip.add(axis.scale(flute)), tip.add(axis.scale(stickout)), assembly.shank_diameter() * 0.5)
            .ok_or_else(|| CliError::run("collision: shank is not usable"))?;
        bodies.push(CylBody { label: "shank".to_string(), cylinder, skip_part: false });
    }
    let mut along = stickout;
    for (index, segment) in assembly.holder().iter().enumerate() {
        bodies.push(stack_cylinder(tip, axis, along, segment, format!("holder {}", index + 1))?);
        along += segment.length();
    }
    if let Some(head) = assembly.head() {
        bodies.push(stack_cylinder(tip, axis, along, &head, "head".to_string())?);
    }
    Ok(bodies)
}

fn stack_cylinder(tip: MeshVec, axis: MeshVec, start: f64, segment: &HolderSegment, label: String) -> Result<CylBody, CliError> {
    let a = tip.add(axis.scale(start));
    let b = tip.add(axis.scale(start + segment.length()));
    let cylinder = Cylinder::new(a, b, segment.diameter() * 0.5).ok_or_else(|| CliError::run("collision: holder is not usable"))?;
    Ok(CylBody { label, cylinder, skip_part: false })
}

fn mesh_vec(v: campanion_ir::Vec3) -> MeshVec {
    MeshVec::new(v.x(), v.y(), v.z())
}

fn query_bodies(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    bodies: &[CylBody],
    part: &RevolvedSolid,
    chuck: Option<&SectionBody>,
    steady: Option<&SectionBody>,
    clearance: f64,
) -> Result<Option<String>, CliError> {
    for body in bodies {
        if !body.skip_part
            && let Some(message) = cylinder_hit(op_name, path_index, move_index, body, part, "part", clearance)?
        {
            return Ok(Some(message));
        }
        if let Some(chuck) = chuck
            && let Some(message) = cylinder_hit(op_name, path_index, move_index, body, &chuck.solid, chuck.name, clearance)?
        {
            return Ok(Some(message));
        }
        if let Some(steady) = steady
            && let Some(message) = cylinder_hit(op_name, path_index, move_index, body, &steady.solid, steady.name, clearance)?
        {
            return Ok(Some(message));
        }
    }
    Ok(None)
}

fn cylinder_hit(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    body: &CylBody,
    solid: &RevolvedSolid,
    obstacle: &str,
    clearance: f64,
) -> Result<Option<String>, CliError> {
    let Some(hit) = solid.query_cylinder(body.cylinder) else {
        return Err(CliError::run("collision: live tool axis is not axial or radial"));
    };
    if hit.distance.is_finite() && collides(hit.distance, clearance) {
        Ok(Some(format_hit(op_name, path_index, move_index, &body.label, obstacle, hit)))
    } else {
        Ok(None)
    }
}

fn polygon_hit(
    op_name: &str,
    path_index: usize,
    move_index: usize,
    body: &str,
    shape: &[(f64, f64)],
    obstacle: &SectionBody,
    clearance: f64,
) -> Option<String> {
    let (distance, witness) = convex_polygon_distance(shape, &obstacle.section)?;
    if distance.is_finite() && collides(distance, clearance) {
        let hit = CollisionHit { point: MeshVec::new(witness.1, 0.0, witness.0), distance };
        Some(format_hit(op_name, path_index, move_index, body, obstacle.name, hit))
    } else {
        None
    }
}

fn format_hit(op_name: &str, path_index: usize, move_index: usize, body: &str, obstacle: &str, hit: CollisionHit) -> String {
    format!(
        "operation `{op_name}` toolpath {path_index} move {move_index}: {body} hits {obstacle} at ({:.3}, {:.3}, {:.3})",
        hit.point.x, hit.point.y, hit.point.z
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_mesh::TOUCH_SLACK;

    fn groove_holder(nose_z: f64, nose_r: f64, holder_width: f64, holder_length: f64) -> [(f64, f64); 4] {
        let insert = InsertGeom { nose_radius: 0.4, included_angle_deg: 0.0, orientation_deg: 90.0, edge_length: 4.0, width: Some(3.0) };
        let holder = HolderGeom { width: holder_width, length: holder_length };
        holder_corners(&insert, &holder, (nose_z, nose_r)).expect("holder corners")
    }

    fn chuck_section(radius: f64) -> Vec<(f64, f64)> {
        vec![(0.0, 0.0), (0.0, radius), (-50.0, radius), (-50.0, 0.0)]
    }

    #[test]
    fn message_names_the_body_obstacle_and_point() {
        let hit = CollisionHit { point: MeshVec::new(12.3, -4.0, 8.1), distance: -0.2 };
        assert_eq!(
            format_hit("od_slot", 2, 4, "holder 1", "chuck", hit),
            "operation `od_slot` toolpath 2 move 4: holder 1 hits chuck at (12.300, -4.000, 8.100)"
        );
        assert_eq!(
            format_hit("face", 0, 0, "insert", "sub chuck", hit),
            "operation `face` toolpath 0 move 0: insert hits sub chuck at (12.300, -4.000, 8.100)"
        );
    }

    #[test]
    fn groove_holder_at_ninety_degrees_is_an_axis_aligned_rectangle() {
        let corners = groove_holder(10.0, 12.0, 8.0, 15.0);
        let (z_min, z_max, r_min, r_max) = bounds(&corners);
        assert!((z_min - 6.0).abs() < 1e-9 && (z_max - 14.0).abs() < 1e-9, "{z_min} {z_max}");
        assert!((r_min - 16.0).abs() < 1e-9 && (r_max - 31.0).abs() < 1e-9, "{r_min} {r_max}");
    }

    #[test]
    fn holder_touches_the_chuck_face_at_the_clearance_and_crosses_just_inside_it() {
        let chuck = chuck_section(40.0);
        let touching = groove_holder(5.0, 10.0, 8.0, 10.0);
        let (distance, _) = convex_polygon_distance(&touching, &chuck).expect("touch");
        assert!((distance - 1.0).abs() < 1e-6, "{distance}");
        assert!(!collides(distance, 1.0));
        let crossing = groove_holder(4.5, 10.0, 8.0, 10.0);
        let (distance, _) = convex_polygon_distance(&crossing, &chuck).expect("cross");
        assert!(collides(distance, 1.0), "{distance}");
        let outside = groove_holder(-2.0, 50.0, 8.0, 10.0);
        let (distance, _) = convex_polygon_distance(&outside, &chuck).expect("outside");
        assert!(distance > 1.0, "{distance}");
        assert!(!collides(distance, 0.0));
    }

    fn bounds(corners: &[(f64, f64)]) -> (f64, f64, f64, f64) {
        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        let mut r_min = f64::INFINITY;
        let mut r_max = f64::NEG_INFINITY;
        for corner in corners {
            z_min = z_min.min(corner.0);
            z_max = z_max.max(corner.0);
            r_min = r_min.min(corner.1);
            r_max = r_max.max(corner.1);
        }
        (z_min, z_max, r_min, r_max)
    }

    fn holder_config() -> proptest::test_runner::Config {
        proptest::test_runner::Config {
            cases: 32,
            max_shrink_iters: 200,
            rng_seed: proptest::test_runner::RngSeed::Fixed(0x1A7E_0C11),
            ..proptest::test_runner::Config::default()
        }
    }

    proptest::proptest! {
        #![proptest_config(holder_config())]
        #[test]
        fn holder_crosses_the_chuck_face_exactly_inside_the_clearance(nose_z in 0.0f64..16.0, clearance in 0.0f64..3.0) {
            let chuck = chuck_section(40.0);
            let corners = groove_holder(nose_z, 10.0, 8.0, 10.0);
            let (z_min, _, r_min, r_max) = bounds(&corners);
            proptest::prop_assert!(r_min > 1.0 && r_max < 40.0, "radius {r_min} {r_max}");
            proptest::prop_assume!(z_min.abs() > 1e-3);
            proptest::prop_assume!((z_min - clearance).abs() > 1e-3);
            let (distance, _) = convex_polygon_distance(&corners, &chuck).expect("distance");
            if z_min >= 0.0 {
                proptest::prop_assert!((distance - z_min).abs() < 1e-5, "gap {distance} z_min {z_min}");
            } else {
                proptest::prop_assert!(distance < 0.0, "overlap {distance} z_min {z_min}");
            }
            let crosses = z_min + TOUCH_SLACK < clearance;
            proptest::prop_assert_eq!(collides(distance, clearance), crosses, "distance {} z_min {} clearance {}", distance, z_min, clearance);
        }
    }

    proptest::proptest! {
        #![proptest_config(holder_config())]
        #[test]
        fn holder_radially_outside_the_chuck_misses_the_face_plane(nose_z in -10.0f64..6.0, length in 4.0f64..20.0) {
            let chuck = chuck_section(40.0);
            let corners = groove_holder(nose_z, 50.0, 8.0, length);
            let (_, _, r_min, _) = bounds(&corners);
            proptest::prop_assert!(r_min > 40.0, "{r_min}");
            let (distance, _) = convex_polygon_distance(&corners, &chuck).expect("distance");
            proptest::prop_assert!(distance > 1.0, "{distance}");
            proptest::prop_assert!(!collides(distance, 0.0));
        }
    }
}
