//! Machine sketch for a five-axis program.
//!
//! [`MachineSim`] solves the toolpath once with [`solve_path`] and [`subdivide_path`], the same
//! pair the continuous posts use, and keeps those joint samples. A playback position returns the
//! posted joints (exact on a sample, including a fraction of a nanometre of rounding), the part-frame
//! wireframe of each link and of the tool assembly, and the collisions at that sample.
//!
//! The solids are boxes and cylinders. This is not a CAD model of a machine, and the posted joints
//! have not been proven on a machine. A program with no five-axis setup has no simulation.

use campanion_ir::{
    Carrier, ClBlock, ClMotion, Document, FiveAxisSetup, Fixture, Joints, Kinematics, LinkSolid, Program, Tool, ToolAssembly, ToolPose,
    Vec3 as IrVec, solve_path, subdivide_path,
};
use campanion_mesh::{Aabb, Cylinder, Obb, Vec3 as MeshVec, collides, cylinder_aabb, cylinder_cylinder, cylinder_obb, obb_obb};

use super::{Projection, project};

/// Chord tolerance the five-axis posts use when they subdivide, in millimetres.
const POST_CHORD_TOL_MM: f64 = 0.005;
/// Tool-axis tolerance the five-axis posts use when they subdivide, in degrees.
const POST_AXIS_TOL_DEG: f64 = 0.1;
/// A playback position this close to a stored sample uses that sample's joints unchanged.
const SNAP_MM: f64 = 1e-9;
/// Sides on a cylinder wireframe. Vertices sit on the true radius.
const CYLINDER_SIDES: usize = 16;
/// Skip a cylinder shorter than this. It has no volume to test.
const MIN_BODY_MM: f64 = 1e-12;

/// Which side of the machine a solid belongs to. Base solids are drawn and not tested.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    Base,
    Table,
    Head,
    Tool,
}

/// Cached joint replay for one program.
#[derive(Clone, Debug)]
pub struct MachineSim {
    kinematics: Kinematics,
    links: Vec<LinkDef>,
    fixtures: Vec<FixtureDef>,
    tools: Vec<ToolDef>,
    samples: Vec<Sample>,
    originals: Vec<(f64, Joints)>,
    total: f64,
    error: Option<String>,
    has_model: bool,
}

#[derive(Clone, Debug)]
struct LinkDef {
    name: String,
    carrier: String,
    side: Side,
    shape: HomeShape,
    edges: Vec<[[f64; 3]; 2]>,
}

#[derive(Clone, Debug)]
enum HomeShape {
    Box { min: [f64; 3], max: [f64; 3] },
    Cylinder { base: [f64; 3], axis: [f64; 3], radius: f64, length: f64 },
}

#[derive(Clone, Debug)]
struct FixtureDef {
    name: String,
    bounds: Aabb,
}

#[derive(Clone, Debug)]
struct ToolDef {
    id: u32,
    diameter: f64,
    assembly: ToolAssembly,
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    position: f64,
    joints: Joints,
    tool_id: Option<u32>,
}

/// One solid the window can draw. Edges are in the part frame.
#[derive(Clone, Debug, PartialEq)]
pub struct SimSolid {
    /// Link name, or a tool-body name (`flute`, `shank`, `holder 1`, `head`).
    pub name: String,
    /// `base`, `table`, `head`, or a rotary letter. Absent on a tool body.
    pub carrier: Option<String>,
    /// Wireframe edges, each `[start, end]`, in millimetres.
    pub edges: Vec<[[f64; 3]; 2]>,
    /// True when this solid is named by a collision at this sample.
    pub colliding: bool,
}

/// One overlapping pair at a sample. `point` is in the part frame.
#[derive(Clone, Debug, PartialEq)]
pub struct SimHit {
    pub name: String,
    pub other: String,
    pub point: [f64; 3],
}

/// Joints, wireframes and collisions at one playback position.
#[derive(Clone, Debug, PartialEq)]
pub struct SimSample {
    /// Posted joints. `None` when the path could not be solved.
    pub joints: Option<Joints>,
    /// `X Y Z` and each rotary letter, degrees to three decimal places, or the solve error.
    pub joint_text: String,
    /// Link solids, then tool-assembly bodies.
    pub solids: Vec<SimSolid>,
    /// Every hit at this sample, in test order.
    pub hits: Vec<SimHit>,
    /// Set when `solve_path` or `subdivide_path` rejected the program.
    pub error: Option<String>,
}

impl SimSample {
    /// Joint text, then the first collision as `{name} hits {other} at (x, y, z)`.
    pub fn status_suffix(&self) -> String {
        let mut text = self.joint_text.clone();
        if let Some(hit) = self.hits.first() {
            text.push_str(&format!("   {} hits {} at ({:.3}, {:.3}, {:.3})", hit.name, hit.other, hit.point[0], hit.point[1], hit.point[2]));
        }
        text
    }

    /// Projected bounds of the wireframe. `None` when there is no finite edge.
    pub fn projected_bounds(&self, projection: Projection) -> Option<([f64; 2], [f64; 2])> {
        let mut min = [f64::INFINITY, f64::INFINITY];
        let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
        let mut any = false;
        for solid in &self.solids {
            for edge in &solid.edges {
                for point in edge {
                    let projected = project([point[0], point[1]], point[2], projection);
                    if !projected[0].is_finite() || !projected[1].is_finite() {
                        continue;
                    }
                    min[0] = min[0].min(projected[0]);
                    min[1] = min[1].min(projected[1]);
                    max[0] = max[0].max(projected[0]);
                    max[1] = max[1].max(projected[1]);
                    any = true;
                }
            }
        }
        if any { Some((min, max)) } else { None }
    }
}

impl MachineSim {
    /// `None` when the program has no five-axis kinematics. The scene is unchanged in that case.
    ///
    /// `arc_tol` is the viewer's chord tolerance, used only so a stroke's length matches the
    /// preview. Joint subdivision uses the post tolerances.
    pub fn from_program(program: &Program, arc_tol: f64) -> Option<Self> {
        program.five_axis()?;
        Some(Self::build(program, arc_tol))
    }

    /// Parse a document, then [`Self::from_program`].
    ///
    /// # Errors
    ///
    /// Returns the IR parser error when `json` is not a document.
    pub fn from_json(json: &str, arc_tol: f64) -> Result<Option<Self>, String> {
        let doc = Document::from_json(json).map_err(|err| err.to_string())?;
        Ok(Self::from_program(doc.program(), arc_tol))
    }

    /// True when the kinematics file included a `model` block.
    pub fn has_model(&self) -> bool {
        self.has_model
    }

    /// Joints of every pose passed to `solve_path`, with the playback position of that pose.
    ///
    /// The position is millimetres along the same strokes the preview draws. The first motion of
    /// the program is at position 0.
    pub fn posted_joints(&self) -> &[(f64, Joints)] {
        &self.originals
    }

    /// Solids and collisions at `position` millimetres along the preview.
    pub fn sample(&self, position: f64) -> SimSample {
        if let Some(err) = &self.error {
            let home = Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] };
            return SimSample {
                joints: None,
                joint_text: err.clone(),
                solids: self.draw_at(&home, None),
                hits: Vec::new(),
                error: Some(err.clone()),
            };
        }
        let position = clamp_position(position, self.total);
        let (joints, tool_id) = self.joints_at(position);
        let mut solids = self.draw_at(&joints, tool_id);
        let hits = collisions(&self.kinematics, &joints, &self.links, &self.fixtures, tool_id.and_then(|id| self.tool(id)));
        for solid in &mut solids {
            solid.colliding = hits.iter().any(|hit| hit.name == solid.name || hit.other == solid.name);
        }
        SimSample { joints: Some(joints), joint_text: joint_text(&self.kinematics, &joints), solids, hits, error: None }
    }

    fn build(program: &Program, arc_tol: f64) -> Self {
        let Some(setup) = program.five_axis() else {
            return Self::empty(Kinematics { name: String::new(), rotaries: Vec::new(), spindle_axis: IrVec::Z, model: None });
        };
        let kinematics = setup.kinematics.clone();
        let has_model = kinematics.model.is_some();
        let links = link_defs(&kinematics);
        let fixtures = fixture_defs(program);
        let tools = tool_defs(program);
        match replay(program, setup, arc_tol) {
            Ok(played) => Self {
                kinematics,
                links,
                fixtures,
                tools,
                samples: played.samples,
                originals: played.originals,
                total: played.total,
                error: None,
                has_model,
            },
            Err(err) => {
                Self { kinematics, links, fixtures, tools, samples: Vec::new(), originals: Vec::new(), total: 0.0, error: Some(err), has_model }
            }
        }
    }

    fn empty(kinematics: Kinematics) -> Self {
        Self {
            kinematics,
            links: Vec::new(),
            fixtures: Vec::new(),
            tools: Vec::new(),
            samples: Vec::new(),
            originals: Vec::new(),
            total: 0.0,
            error: None,
            has_model: false,
        }
    }

    fn tool(&self, id: u32) -> Option<&ToolDef> {
        self.tools.iter().find(|tool| tool.id == id)
    }

    fn joints_at(&self, position: f64) -> (Joints, Option<u32>) {
        let Some(first) = self.samples.first() else {
            return (Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] }, None);
        };
        let mut best = 0usize;
        let mut best_gap = f64::MAX;
        for (index, sample) in self.samples.iter().enumerate() {
            let gap = (sample.position - position).abs();
            if gap <= best_gap {
                best_gap = gap;
                best = index;
            }
        }
        if best_gap <= SNAP_MM
            && let Some(sample) = self.samples.get(best)
        {
            return (sample.joints, sample.tool_id);
        }
        let mut hi = self.samples.len() - 1;
        for (index, sample) in self.samples.iter().enumerate() {
            if sample.position + SNAP_MM >= position {
                hi = index;
                break;
            }
        }
        if hi == 0 {
            return (first.joints, first.tool_id);
        }
        let Some(right) = self.samples.get(hi) else {
            return (first.joints, first.tool_id);
        };
        let Some(left) = self.samples.get(hi - 1) else {
            return (right.joints, right.tool_id);
        };
        let span = right.position - left.position;
        if !(span > SNAP_MM) {
            return (right.joints, right.tool_id);
        }
        let t = ((position - left.position) / span).clamp(0.0, 1.0);
        (lerp_joints(left.joints, right.joints, t), right.tool_id)
    }

    fn draw_at(&self, joints: &Joints, tool_id: Option<u32>) -> Vec<SimSolid> {
        let mut solids = Vec::new();
        for link in &self.links {
            let Ok(rigid) = self.kinematics.link_in_part(joints, &link.carrier) else {
                continue;
            };
            solids.push(SimSolid {
                name: link.name.clone(),
                carrier: Some(link.carrier.clone()),
                edges: map_edges(rigid, &link.edges),
                colliding: false,
            });
        }
        if let Some(tool) = tool_id.and_then(|id| self.tool(id))
            && let Ok(pose) = self.kinematics.forward(joints, 0.0)
        {
            solids.extend(tool_solids(&pose, tool));
        }
        solids
    }
}

struct Played {
    samples: Vec<Sample>,
    originals: Vec<(f64, Joints)>,
    total: f64,
}

fn replay(program: &Program, setup: &FiveAxisSetup, arc_tol: f64) -> Result<Played, String> {
    let tol = super::sanitize_tol(arc_tol);
    let lathe = program.turning().is_some();
    let mut cursor: Option<ToolPose> = None;
    let mut position = 0.0;
    let mut start = Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] };
    let mut samples = Vec::new();
    let mut originals = Vec::new();
    for path in program.toolpaths() {
        let mut batch: Vec<(ToolPose, f64)> = Vec::new();
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let Some(end) = tool_pose(motion) else {
                continue;
            };
            if cursor.is_none() {
                batch.push((end, 0.0));
                cursor = Some(end);
                continue;
            }
            let Some(start_pose) = cursor else {
                continue;
            };
            let length = motion_length(motion, start_pose, end, tol, lathe);
            batch.push((end, length));
            cursor = Some(end);
        }
        if batch.is_empty() {
            continue;
        }
        let poses: Vec<ToolPose> = batch.iter().map(|(pose, _)| *pose).collect();
        let joints = solve_path(setup, &poses, Some(start), path.singular_cone_deg()).map_err(|err| err.to_string())?;
        if joints.len() != poses.len() {
            return Err("solve_path returned a different number of joints".to_string());
        }
        let mut origins = Vec::with_capacity(batch.len());
        for (index, (_, length)) in batch.iter().enumerate() {
            position += length;
            origins.push(position);
            if let Some(solved) = joints.get(index) {
                originals.push((position, *solved));
            }
        }
        let name = path.name().unwrap_or("");
        let expanded = expand_path(setup, name, &poses, &joints, &origins, path.tool_id())?;
        samples.extend(expanded);
        if let Some(last) = joints.last() {
            start = *last;
        }
    }
    Ok(Played { samples, originals, total: position })
}

fn expand_path(
    setup: &FiveAxisSetup,
    name: &str,
    poses: &[ToolPose],
    joints: &[Joints],
    origins: &[f64],
    tool_id: Option<u32>,
) -> Result<Vec<Sample>, String> {
    let (sub_joints, fallback) = match subdivide_path(setup, name, poses, joints, POST_CHORD_TOL_MM, POST_AXIS_TOL_DEG) {
        Ok((_, sub_joints)) => (sub_joints, false),
        Err(err) if err.to_string().contains("inserts more than") => (joints.to_vec(), true),
        Err(err) => return Err(err.to_string()),
    };
    if fallback {
        return Ok(exact_samples(joints, origins, tool_id));
    }
    let mut samples = Vec::new();
    let mut orig = 0usize;
    for (index, at) in sub_joints.iter().enumerate() {
        if index == 0 {
            push_exact(&mut samples, origins, joints, 0, tool_id);
            continue;
        }
        if orig + 1 < joints.len() && same_joints(at, &joints[orig + 1]) {
            orig += 1;
            push_exact(&mut samples, origins, joints, orig, tool_id);
            continue;
        }
        let Some(left) = joints.get(orig) else {
            continue;
        };
        let Some(right) = joints.get(orig + 1) else {
            continue;
        };
        let Some(left_pos) = origins.get(orig).copied() else {
            continue;
        };
        let Some(right_pos) = origins.get(orig + 1).copied() else {
            continue;
        };
        let t = lerp_param(*left, *right, *at).clamp(0.0, 1.0);
        samples.push(Sample { position: left_pos + (right_pos - left_pos) * t, joints: *at, tool_id });
    }
    if samples.is_empty() {
        return Ok(exact_samples(joints, origins, tool_id));
    }
    Ok(samples)
}

fn exact_samples(joints: &[Joints], origins: &[f64], tool_id: Option<u32>) -> Vec<Sample> {
    let mut samples = Vec::new();
    for index in 0..joints.len() {
        push_exact(&mut samples, origins, joints, index, tool_id);
    }
    samples
}

fn push_exact(samples: &mut Vec<Sample>, origins: &[f64], joints: &[Joints], index: usize, tool_id: Option<u32>) {
    let (Some(position), Some(joints)) = (origins.get(index).copied(), joints.get(index).copied()) else {
        return;
    };
    samples.push(Sample { position, joints, tool_id });
}

fn tool_pose(motion: &ClMotion) -> Option<ToolPose> {
    let pose = super::motion_pose(motion);
    let tip = pose.tip();
    let axis = pose.axis();
    if [tip.x(), tip.y(), tip.z(), axis.x(), axis.y(), axis.z()].into_iter().all(f64::is_finite) { Some(ToolPose { tip, axis }) } else { None }
}

fn motion_length(motion: &ClMotion, start: ToolPose, end: ToolPose, tol: f64, lathe: bool) -> f64 {
    let start_tip = [start.tip.x(), start.tip.y(), start.tip.z()];
    let end_tip = [end.tip.x(), end.tip.y(), end.tip.z()];
    let stroke = super::stroke_from_motion(0, motion, start_tip, end_tip, None, None, tol, lathe);
    if stroke.length.is_finite() && stroke.length > 0.0 { stroke.length } else { 0.0 }
}

fn link_defs(kinematics: &Kinematics) -> Vec<LinkDef> {
    let Some(model) = &kinematics.model else {
        return Vec::new();
    };
    let mut links = Vec::new();
    for link in &model.links {
        let Some(shape) = home_shape(link.solid) else {
            continue;
        };
        let edges = match shape {
            HomeShape::Box { min, max } => tessellate_box(min, max),
            HomeShape::Cylinder { base, axis, radius, length } => tessellate_cylinder(base, axis, radius, length),
        };
        links.push(LinkDef { name: link.name.clone(), carrier: link.carrier.clone(), side: side_of(kinematics, &link.carrier), shape, edges });
    }
    links
}

fn home_shape(solid: LinkSolid) -> Option<HomeShape> {
    match solid {
        LinkSolid::Box { min, max } => Some(HomeShape::Box { min: [min.x(), min.y(), min.z()], max: [max.x(), max.y(), max.z()] }),
        LinkSolid::Cylinder { base, axis, radius, length } => {
            Some(HomeShape::Cylinder { base: [base.x(), base.y(), base.z()], axis: [axis.x(), axis.y(), axis.z()], radius, length })
        }
    }
}

fn side_of(kinematics: &Kinematics, carrier: &str) -> Side {
    match carrier {
        "base" => Side::Base,
        "head" => Side::Head,
        "table" => Side::Table,
        other => {
            let Some(letter) = other.chars().next() else {
                return Side::Base;
            };
            match kinematics.rotaries.iter().find(|joint| joint.letter == letter).map(|joint| joint.carrier) {
                Some(Carrier::Head) => Side::Head,
                Some(Carrier::Table) => Side::Table,
                None => Side::Base,
            }
        }
    }
}

fn fixture_defs(program: &Program) -> Vec<FixtureDef> {
    let mut fixtures = Vec::new();
    for fixture in program.fixtures() {
        if let Some(def) = fixture_def(fixture) {
            fixtures.push(def);
        }
    }
    fixtures
}

fn fixture_def(fixture: &Fixture) -> Option<FixtureDef> {
    let bounds = fixture.bounds()?;
    let min = bounds.min();
    let max = bounds.max();
    Some(FixtureDef {
        name: fixture.name().to_string(),
        bounds: Aabb::from_corners(MeshVec::new(min.x(), min.y(), min.z()), MeshVec::new(max.x(), max.y(), max.z())),
    })
}

fn tool_defs(program: &Program) -> Vec<ToolDef> {
    let mut tools = Vec::new();
    for tool in program.tools() {
        if let Some(def) = tool_def(tool) {
            tools.push(def);
        }
    }
    tools
}

fn tool_def(tool: &Tool) -> Option<ToolDef> {
    Some(ToolDef { id: tool.id(), diameter: tool.diameter()?, assembly: tool.assembly()?.clone() })
}

fn tool_solids(pose: &ToolPose, tool: &ToolDef) -> Vec<SimSolid> {
    let axis = pose.axis;
    let Some((axis_x, axis_y)) = perp_axes(axis) else {
        return Vec::new();
    };
    let mut solids = Vec::new();
    let radius = tool.diameter * 0.5;
    let flute = tool.assembly.flute_length();
    let stickout = tool.assembly.stickout();
    if flute > MIN_BODY_MM {
        push_tool(&mut solids, pose.tip, axis, axis_x, axis_y, 0.0, flute, radius, "flute");
    }
    if stickout - flute > MIN_BODY_MM {
        let shank_r = tool.assembly.shank_diameter(tool.diameter) * 0.5;
        push_tool(&mut solids, pose.tip, axis, axis_x, axis_y, flute, stickout, shank_r, "shank");
    }
    let mut along = stickout;
    for (index, segment) in tool.assembly.holder().iter().enumerate() {
        let name = format!("holder {}", index + 1);
        push_tool(&mut solids, pose.tip, axis, axis_x, axis_y, along, along + segment.length(), segment.diameter() * 0.5, &name);
        along += segment.length();
    }
    if let Some(head) = tool.assembly.head() {
        push_tool(&mut solids, pose.tip, axis, axis_x, axis_y, along, along + head.length(), head.diameter() * 0.5, "head");
    }
    solids
}

fn push_tool(solids: &mut Vec<SimSolid>, tip: IrVec, axis: IrVec, axis_x: IrVec, axis_y: IrVec, start: f64, end: f64, radius: f64, name: &str) {
    let length = end - start;
    if length <= MIN_BODY_MM || radius < 0.0 {
        return;
    }
    let Some(shift) = scale_vec(axis, start) else {
        return;
    };
    let origin = add_vec(tip, shift);
    let rigid = campanion_ir::Rigid::from_axes(origin, axis_x, axis_y, axis);
    let edges = map_edges(rigid, &tessellate_cylinder([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], radius, length));
    solids.push(SimSolid { name: name.to_string(), carrier: None, edges, colliding: false });
}

struct Placed {
    name: String,
    side: Side,
    shape: WorldShape,
}

#[derive(Clone, Copy)]
enum WorldShape {
    Aabb(Aabb),
    Obb(Obb),
    Cylinder(Cylinder),
}

fn collisions(kinematics: &Kinematics, joints: &Joints, links: &[LinkDef], fixtures: &[FixtureDef], tool: Option<&ToolDef>) -> Vec<SimHit> {
    let mut placed = Vec::new();
    for link in links {
        let Ok(rigid) = kinematics.link_in_part(joints, &link.carrier) else {
            continue;
        };
        let Some(shape) = place_link(rigid, &link.shape) else {
            continue;
        };
        placed.push(Placed { name: link.name.clone(), side: link.side, shape });
    }
    let mut tools = Vec::new();
    if let Some(tool) = tool
        && let Ok(pose) = kinematics.forward(joints, 0.0)
    {
        tools.extend(place_tool(&pose, tool));
    }
    let mut hits = Vec::new();
    for body in &tools {
        for fixture in fixtures {
            push_hit(&mut hits, &body.name, &fixture.name, shapes_hit(body.shape, WorldShape::Aabb(fixture.bounds)));
        }
        for link in placed.iter().filter(|link| link.side == Side::Table) {
            push_hit(&mut hits, &body.name, &link.name, shapes_hit(body.shape, link.shape));
        }
    }
    for head in placed.iter().filter(|link| link.side == Side::Head) {
        for table in placed.iter().filter(|link| link.side == Side::Table) {
            push_hit(&mut hits, &head.name, &table.name, shapes_hit(head.shape, table.shape));
        }
        for fixture in fixtures {
            push_hit(&mut hits, &head.name, &fixture.name, shapes_hit(head.shape, WorldShape::Aabb(fixture.bounds)));
        }
    }
    hits
}

fn push_hit(hits: &mut Vec<SimHit>, name: &str, other: &str, hit: Option<campanion_mesh::CollisionHit>) {
    let Some(hit) = hit else {
        return;
    };
    if !collides(hit.distance, 0.0) || !hit.point.x.is_finite() || !hit.point.y.is_finite() || !hit.point.z.is_finite() {
        return;
    }
    hits.push(SimHit { name: name.to_string(), other: other.to_string(), point: [hit.point.x, hit.point.y, hit.point.z] });
}

fn shapes_hit(left: WorldShape, right: WorldShape) -> Option<campanion_mesh::CollisionHit> {
    match (left, right) {
        (WorldShape::Aabb(a), WorldShape::Aabb(b)) => obb_obb(Obb::from_aabb(a), Obb::from_aabb(b)),
        (WorldShape::Aabb(a), WorldShape::Obb(b)) | (WorldShape::Obb(b), WorldShape::Aabb(a)) => obb_obb(Obb::from_aabb(a), b),
        (WorldShape::Obb(a), WorldShape::Obb(b)) => obb_obb(a, b),
        (WorldShape::Cylinder(cylinder), WorldShape::Aabb(bounds)) | (WorldShape::Aabb(bounds), WorldShape::Cylinder(cylinder)) => {
            cylinder_aabb(cylinder, bounds)
        }
        (WorldShape::Cylinder(cylinder), WorldShape::Obb(obb)) | (WorldShape::Obb(obb), WorldShape::Cylinder(cylinder)) => {
            cylinder_obb(cylinder, obb)
        }
        (WorldShape::Cylinder(a), WorldShape::Cylinder(b)) => cylinder_cylinder(a, b),
    }
}

fn place_link(rigid: campanion_ir::Rigid, shape: &HomeShape) -> Option<WorldShape> {
    match *shape {
        HomeShape::Box { min, max } => Some(WorldShape::Obb(place_box(rigid, min, max)?)),
        HomeShape::Cylinder { base, axis, radius, length } => Some(WorldShape::Cylinder(place_cylinder(rigid, base, axis, radius, length)?)),
    }
}

fn place_box(rigid: campanion_ir::Rigid, min: [f64; 3], max: [f64; 3]) -> Option<Obb> {
    let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5, (min[2] + max[2]) * 0.5];
    let half = [(max[0] - min[0]) * 0.5, (max[1] - min[1]) * 0.5, (max[2] - min[2]) * 0.5];
    let origin = rigid.map_point(ir_point(center)?);
    let axis_x = rigid.map_direction(IrVec::X);
    let axis_y = rigid.map_direction(IrVec::Y);
    let axis_z = rigid.map_direction(IrVec::Z);
    Obb::try_new(mesh_vec(origin), [mesh_vec(axis_x), mesh_vec(axis_y), mesh_vec(axis_z)], half)
}

fn place_cylinder(rigid: campanion_ir::Rigid, base: [f64; 3], axis: [f64; 3], radius: f64, length: f64) -> Option<Cylinder> {
    let start = rigid.map_point(ir_point(base)?);
    let direction = unit_vec(ir_point(axis)?)?;
    let end = add_vec(start, scale_vec(direction, length)?);
    Cylinder::new(mesh_vec(start), mesh_vec(end), radius)
}

fn place_tool(pose: &ToolPose, tool: &ToolDef) -> Vec<Placed> {
    let mut placed = Vec::new();
    let radius = tool.diameter * 0.5;
    let flute = tool.assembly.flute_length();
    let stickout = tool.assembly.stickout();
    if flute > MIN_BODY_MM {
        push_placed(&mut placed, pose, 0.0, flute, radius, "flute");
    }
    if stickout - flute > MIN_BODY_MM {
        push_placed(&mut placed, pose, flute, stickout, tool.assembly.shank_diameter(tool.diameter) * 0.5, "shank");
    }
    let mut along = stickout;
    for (index, segment) in tool.assembly.holder().iter().enumerate() {
        let name = format!("holder {}", index + 1);
        push_placed(&mut placed, pose, along, along + segment.length(), segment.diameter() * 0.5, &name);
        along += segment.length();
    }
    if let Some(head) = tool.assembly.head() {
        push_placed(&mut placed, pose, along, along + head.length(), head.diameter() * 0.5, "head");
    }
    placed
}

fn push_placed(placed: &mut Vec<Placed>, pose: &ToolPose, start: f64, end: f64, radius: f64, name: &str) {
    let length = end - start;
    if length <= MIN_BODY_MM || radius < 0.0 {
        return;
    }
    let Some(shift) = scale_vec(pose.axis, start) else {
        return;
    };
    let Some(span) = scale_vec(pose.axis, length) else {
        return;
    };
    let a = add_vec(pose.tip, shift);
    let b = add_vec(a, span);
    let Some(cylinder) = Cylinder::new(mesh_vec(a), mesh_vec(b), radius) else {
        return;
    };
    placed.push(Placed { name: name.to_string(), side: Side::Tool, shape: WorldShape::Cylinder(cylinder) });
}

/// Twelve edges of a box. Each edge changes exactly one coordinate.
pub fn tessellate_box(min: [f64; 3], max: [f64; 3]) -> Vec<[[f64; 3]; 2]> {
    let mut corners = [[0.0; 3]; 8];
    let mut count = 0usize;
    for x in [min[0], max[0]] {
        for y in [min[1], max[1]] {
            for z in [min[2], max[2]] {
                if let Some(slot) = corners.get_mut(count) {
                    *slot = [x, y, z];
                    count += 1;
                }
            }
        }
    }
    let mut edges = Vec::with_capacity(12);
    for i in 0..count {
        for j in (i + 1)..count {
            let (Some(a), Some(b)) = (corners.get(i), corners.get(j)) else {
                continue;
            };
            let changes = (0..3).filter(|axis| a[*axis] != b[*axis]).count();
            if changes == 1 {
                edges.push([*a, *b]);
            }
        }
    }
    edges
}

/// Rim and longitudinal edges of a cylinder. `CYLINDER_SIDES` vertices lie on the radius.
pub fn tessellate_cylinder(base: [f64; 3], axis: [f64; 3], radius: f64, length: f64) -> Vec<[[f64; 3]; 2]> {
    let Some(base) = ir_point(base) else {
        return Vec::new();
    };
    let Some(axis) = ir_point(axis).and_then(unit_vec) else {
        return Vec::new();
    };
    let Some((u, v)) = perp_axes(axis) else {
        return Vec::new();
    };
    let Some(span) = scale_vec(axis, length) else {
        return Vec::new();
    };
    let top = add_vec(base, span);
    let mut lower = Vec::with_capacity(CYLINDER_SIDES);
    let mut upper = Vec::with_capacity(CYLINDER_SIDES);
    for index in 0..CYLINDER_SIDES {
        let angle = usize_f64(index) * std::f64::consts::TAU / usize_f64(CYLINDER_SIDES);
        let Some(offset) =
            scale_vec(u, radius * angle.cos()).and_then(|along| scale_vec(v, radius * angle.sin()).map(|across| add_vec(along, across)))
        else {
            return Vec::new();
        };
        lower.push(add_vec(base, offset));
        upper.push(add_vec(top, offset));
    }
    let mut edges = Vec::with_capacity(CYLINDER_SIDES * 3);
    for index in 0..CYLINDER_SIDES {
        let next = (index + 1) % CYLINDER_SIDES;
        let (Some(a0), Some(a1), Some(b0), Some(b1)) = (lower.get(index), lower.get(next), upper.get(index), upper.get(next)) else {
            continue;
        };
        edges.push([point_array(*a0), point_array(*a1)]);
        edges.push([point_array(*b0), point_array(*b1)]);
        edges.push([point_array(*a0), point_array(*b0)]);
    }
    edges
}

fn map_edges(rigid: campanion_ir::Rigid, edges: &[[[f64; 3]; 2]]) -> Vec<[[f64; 3]; 2]> {
    let mut out = Vec::with_capacity(edges.len());
    for edge in edges {
        let (Some(a), Some(b)) = (edge.first().and_then(|point| ir_point(*point)), edge.get(1).and_then(|point| ir_point(*point))) else {
            continue;
        };
        out.push([point_array(rigid.map_point(a)), point_array(rigid.map_point(b))]);
    }
    out
}

fn joint_text(kinematics: &Kinematics, joints: &Joints) -> String {
    let mut text = format!("X {:.3} Y {:.3} Z {:.3}", joints.xyz.x(), joints.xyz.y(), joints.xyz.z());
    for (index, rotary) in kinematics.rotaries.iter().enumerate() {
        let angle = match index {
            0 => joints.angles[0],
            1 => joints.angles[1],
            _ => 0.0,
        };
        text.push_str(&format!(" {} {:.3}", rotary.letter, angle));
    }
    text
}

fn clamp_position(position: f64, total: f64) -> f64 {
    if !position.is_finite() || position < 0.0 {
        0.0
    } else if position > total {
        total
    } else {
        position
    }
}

fn same_joints(left: &Joints, right: &Joints) -> bool {
    left.xyz.x() == right.xyz.x()
        && left.xyz.y() == right.xyz.y()
        && left.xyz.z() == right.xyz.z()
        && left.angles[0] == right.angles[0]
        && left.angles[1] == right.angles[1]
}

fn lerp_param(left: Joints, right: Joints, mid: Joints) -> f64 {
    let parts = [
        (left.xyz.x(), right.xyz.x(), mid.xyz.x()),
        (left.xyz.y(), right.xyz.y(), mid.xyz.y()),
        (left.xyz.z(), right.xyz.z(), mid.xyz.z()),
        (left.angles[0], right.angles[0], mid.angles[0]),
        (left.angles[1], right.angles[1], mid.angles[1]),
    ];
    for (start, end, at) in parts {
        let span = end - start;
        if span.abs() > 1e-12 {
            return (at - start) / span;
        }
    }
    0.0
}

fn lerp_joints(left: Joints, right: Joints, t: f64) -> Joints {
    Joints {
        xyz: add_vec(left.xyz, scale_vec(sub_vec(right.xyz, left.xyz), t).unwrap_or(IrVec::ZERO)),
        angles: [left.angles[0] + (right.angles[0] - left.angles[0]) * t, left.angles[1] + (right.angles[1] - left.angles[1]) * t],
    }
}

fn perp_axes(axis: IrVec) -> Option<(IrVec, IrVec)> {
    let hint = if axis.x().abs() < 0.9 { IrVec::X } else { IrVec::Y };
    let x = unit_vec(cross_vec(axis, hint)?)?;
    let y = unit_vec(cross_vec(axis, x)?)?;
    Some((x, y))
}

fn cross_vec(a: IrVec, b: IrVec) -> Option<IrVec> {
    IrVec::new(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x()).ok()
}

fn add_vec(a: IrVec, b: IrVec) -> IrVec {
    IrVec::new(a.x() + b.x(), a.y() + b.y(), a.z() + b.z()).unwrap_or(IrVec::ZERO)
}

fn sub_vec(a: IrVec, b: IrVec) -> IrVec {
    IrVec::new(a.x() - b.x(), a.y() - b.y(), a.z() - b.z()).unwrap_or(IrVec::ZERO)
}

fn scale_vec(a: IrVec, factor: f64) -> Option<IrVec> {
    IrVec::new(a.x() * factor, a.y() * factor, a.z() * factor).ok()
}

fn unit_vec(a: IrVec) -> Option<IrVec> {
    let len = (a.x() * a.x() + a.y() * a.y() + a.z() * a.z()).sqrt();
    if !len.is_finite() || len <= 0.0 { None } else { scale_vec(a, 1.0 / len) }
}

fn ir_point(point: [f64; 3]) -> Option<IrVec> {
    IrVec::new(point[0], point[1], point[2]).ok()
}

fn point_array(point: IrVec) -> [f64; 3] {
    [point.x(), point.y(), point.z()]
}

fn mesh_vec(point: IrVec) -> MeshVec {
    MeshVec::new(point.x(), point.y(), point.z())
}

fn usize_f64(value: usize) -> f64 {
    u32::try_from(value).map_or(0.0, f64::from)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use campanion_ir::{ClPath, CompensationMode, Feeds, MoveRole, Pose, SolutionPolicy};

    fn vec3(x: f64, y: f64, z: f64) -> IrVec {
        IrVec::new(x, y, z).expect("point")
    }

    fn kin_with(model_links: &str) -> String {
        format!(
            r#"{{"name":"demo","spindle_axis":{{"x":0.0,"y":0.0,"z":1.0}},"rotaries":[{{"letter":"B","carrier":"table","axis":{{"x":0.0,"y":1.0,"z":0.0}},"pivot":{{"x":0.0,"y":0.0,"z":0.0}},"limits":{{"min_deg":-35.0,"max_deg":120.0}}}},{{"letter":"C","carrier":"table","axis":{{"x":0.0,"y":0.0,"z":1.0}},"pivot":{{"x":0.0,"y":0.0,"z":0.0}}}}],"model":{{"links":[{model_links}]}}}}"#
        )
    }

    fn box_link(name: &str, carrier: &str, min: [f64; 3], max: [f64; 3]) -> String {
        format!(
            r#"{{"name":"{name}","carrier":"{carrier}","solid":{{"box":{{"min":{{"x":{},"y":{},"z":{}}},"max":{{"x":{},"y":{},"z":{}}}}}}}}}"#,
            min[0], min[1], min[2], max[0], max[1], max[2]
        )
    }

    fn vertical(json: &str, tips: &[[f64; 3]]) -> Program {
        let kinematics = Kinematics::from_json(json).expect("kinematics");
        let setup = FiveAxisSetup { kinematics, policy: SolutionPolicy::Nearest };
        let mut blocks = Vec::new();
        for tip in tips {
            let pose = Pose::new(vec3(tip[0], tip[1], tip[2]), IrVec::Z).expect("pose");
            blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose, feed: 100.0, role: MoveRole::Cut, power: None }));
        }
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(Some("cut".into()), None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program").with_five_axis(setup).expect("five")
    }

    #[test]
    fn a_program_without_kinematics_has_no_simulation() {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let pose = Pose::new(IrVec::ZERO, IrVec::Z).expect("pose");
        let path = ClPath::try_new(
            None,
            None,
            CompensationMode::Off,
            feeds,
            None,
            Vec::new(),
            vec![ClBlock::Motion(ClMotion::Linear { to: pose, feed: 100.0, role: MoveRole::Cut, power: None })],
        )
        .expect("path");
        let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        assert!(MachineSim::from_program(&program, 0.01).is_none());
    }

    #[test]
    fn posted_joints_match_solve_path_at_every_pose() {
        let json = kin_with("");
        let tips = [[0.0, 0.0, 0.0], [30.0, 0.0, 5.0], [30.0, 12.0, 5.0]];
        let program = vertical(&json, &tips);
        let sim = MachineSim::from_program(&program, 0.01).expect("sim");
        let setup = program.five_axis().expect("setup");
        let poses: Vec<ToolPose> = tips.iter().map(|tip| ToolPose { tip: vec3(tip[0], tip[1], tip[2]), axis: IrVec::Z }).collect();
        let solved = solve_path(setup, &poses, Some(Joints { xyz: IrVec::ZERO, angles: [0.0, 0.0] }), 1.0).expect("solve");
        let posted = sim.posted_joints();
        assert_eq!(posted.len(), solved.len());
        for (index, (position, joints)) in posted.iter().enumerate() {
            assert_eq!(joints, solved.get(index).expect("joint"));
            assert_eq!(sim.sample(*position).joints, Some(*joints));
            assert_eq!(sim.sample(position + 1e-10).joints, Some(*joints));
            assert_eq!(sim.sample(position - 1e-10).joints, Some(*joints));
        }
        let text = sim.sample(0.0).joint_text;
        assert!(text.starts_with("X "), "{text}");
        assert!(text.contains(" B "), "{text}");
        assert!(text.contains(" C "), "{text}");
    }

    fn hit_pair(sample: &SimSample, name: &str, other: &str) -> bool {
        sample.hits.iter().any(|hit| hit.name == name && hit.other == other)
    }

    fn boxes(head: ([f64; 3], [f64; 3]), table: ([f64; 3], [f64; 3])) -> MachineSim {
        let links = format!("{},{}", box_link("head_box", "head", head.0, head.1), box_link("table_box", "table", table.0, table.1));
        let program = vertical(&kin_with(&links), &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0]]);
        MachineSim::from_program(&program, 0.01).expect("sim")
    }

    #[test]
    fn an_overlapping_head_is_marked_and_a_gap_is_not() {
        let table = ([-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]);
        let overlap = boxes(([-2.0, -2.0, -2.0], [2.0, 2.0, 2.0]), table);
        let at_start = overlap.sample(0.0);
        let beside = overlap.sample(1e-10);
        assert!(hit_pair(&at_start, "head_box", "table_box"), "{:?}", at_start.hits);
        assert_eq!(at_start.hits, beside.hits);
        assert!(at_start.solids.iter().any(|solid| solid.name == "head_box" && solid.colliding));
        let clear_end = overlap.sample(40.0);
        let clear_near = overlap.sample(40.0 - 1e-10);
        assert!(clear_end.hits.is_empty(), "{:?}", clear_end.hits);
        assert_eq!(clear_end.hits, clear_near.hits);

        let separated = boxes(([12.0, -2.0, -2.0], [18.0, 2.0, 2.0]), table);
        assert!(separated.sample(0.0).hits.is_empty(), "{:?}", separated.sample(0.0).hits);
        assert!(separated.sample(1e-10).hits.is_empty());
    }

    #[test]
    fn box_wireframe_has_twelve_edges() {
        let edges = tessellate_box([-1.0, 2.0, 3.0], [4.0, 6.0, 10.0]);
        assert_eq!(edges.len(), 12);
        for edge in &edges {
            let diffs = (0..3).filter(|axis| edge[0][*axis] != edge[1][*axis]).count();
            assert_eq!(diffs, 1);
        }
    }

    #[test]
    fn cylinder_wireframe_keeps_the_radius() {
        let radius = 12.5;
        let edges = tessellate_cylinder([1.0, 2.0, 3.0], [0.0, 0.0, 2.0], radius, 8.0);
        assert_eq!(edges.len(), CYLINDER_SIDES * 3);
        let mut vertices = Vec::new();
        for edge in &edges {
            vertices.push(edge[0]);
            vertices.push(edge[1]);
        }
        for point in vertices {
            let radial = (point[0] - 1.0).hypot(point[1] - 2.0);
            assert!((radial - radius).abs() <= 1e-9, "radius {radial} != {radius} at {point:?}");
        }
    }

    fn sim_config() -> proptest::prelude::ProptestConfig {
        proptest::prelude::ProptestConfig {
            cases: 32,
            max_shrink_iters: 64,
            rng_seed: proptest::test_runner::RngSeed::Fixed(0x514D_0001),
            ..proptest::prelude::ProptestConfig::default()
        }
    }

    proptest::proptest! {
        #![proptest_config(sim_config())]

        #[test]
        fn a_random_head_overlap_is_marked_and_a_margin_is_clear(
            half_table in 6.0f64..24.0,
            half_head in 0.5f64..2.0,
            margin in 0.5f64..8.0,
        ) {
            let table = ([-half_table, -half_table, -half_table], [half_table, half_table, half_table]);
            let head = ([-half_head, -half_head, -half_head], [half_head, half_head, half_head]);
            let shift = half_table + half_head + margin;
            let links = format!("{},{}", box_link("head_box", "head", head.0, head.1), box_link("table_box", "table", table.0, table.1));
            let program = vertical(&kin_with(&links), &[[0.0, 0.0, 0.0], [shift, 0.0, 0.0]]);
            let sim = MachineSim::from_program(&program, 0.01).expect("sim");
            let at_start = sim.sample(0.0);
            let beside = sim.sample(1e-10);
            let before = sim.sample(-1e-10);
            proptest::prop_assert!(hit_pair(&at_start, "head_box", "table_box"), "{:?}", at_start.hits);
            proptest::prop_assert!(beside.hits == at_start.hits);
            proptest::prop_assert!(before.hits == at_start.hits);
            let clear = sim.sample(shift);
            let clear_before = sim.sample(shift - 1e-10);
            let clear_after = sim.sample(shift + 1e-10);
            proptest::prop_assert!(clear.hits.is_empty(), "{:?} margin {margin}", clear.hits);
            proptest::prop_assert!(clear_before.hits == clear.hits);
            proptest::prop_assert!(clear_after.hits == clear.hits);
        }
    }
}
