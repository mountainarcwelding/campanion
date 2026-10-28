//! Live milling from a drawing on a lathe.
//!
//! A front drawing is face X/Y about the spindle centre. Its toolpath `(u, v, d)`
//! (`d` is 0 on the surface and negative into the part) becomes the pose
//! `(u, v, z_start + d)` inside face interpolation. An OD drawing is unrolled at the
//! cut diameter: X is the distance around the circumference and Y is part Z, so
//! `(u, v, d)` becomes `(diameter / 2 + d, u, v)` inside cylinder interpolation.
//! YZ in-plane coordinates are `(y, z)`, which is that same `(u, v)`, so arc sense
//! is unchanged.
//!
//! Outside face or cylinder interpolation the turning IR rejects a non-zero Y unless
//! the head has a Y axis and the pose names C. The clearance rapid before
//! interpolation therefore parks that coordinate at 0. The move onto the real point
//! is a rapid at clearance after interpolation is on. `y_axis` mode does not
//! interpolate: it indexes C and cuts the drawing's Y directly, which is how a Y
//! machine mills across the spindle centre.

use std::path::{Path, PathBuf};

use campanion_geom::{BoundingBox, Containment, Contour, Region, Tolerance};
use campanion_import::{ImportOptions, import_dxf, import_svg};
use campanion_ir::{ArcSense, ClMotion, MoveRole, Plane, Pose, Vec3};
use campanion_path::{Compensation, CutDirection, CutKind, Move, PocketOp, PocketStrategy, ProfileOp, ProfileSide, Tool, pass_depths};

use crate::error::CliError;
use crate::job::read_text;

/// Below this, a pose Y is still on the turning centreline.
const PARK_MM: f64 = 1e-6;
/// Moves shorter than this are dropped so a repeated retract does not become a zero block.
const MOVE_EPS: f64 = 1e-9;
const TAU: f64 = std::f64::consts::TAU;

/// Profile keeps `side`. A pocket uses `stepover` as a fraction of the cutter diameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MillStrategy {
    Profile { side: ProfileSide },
    Pocket { stepover: f64 },
}

/// Where the drawing sits on the machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MillFrame {
    /// Face interpolation. Drawing origin is the spindle centre.
    Front { z_start: f64 },
    /// Indexed X/Y on a head that has a Y axis. No face interpolation.
    FrontY { z_start: f64, c_deg: f64 },
    /// Cylinder interpolation. `diameter` is the surface being cut.
    Od { diameter: f64 },
}

/// One parsed `live_mill`, plus the contours selected from its drawing.
pub(crate) struct MillSpec {
    pub name: String,
    pub tool_diameter: f64,
    pub feed: f64,
    pub depth: f64,
    pub step_down: f64,
    pub clearance: f64,
    pub tolerance: f64,
    pub strategy: MillStrategy,
    pub frame: MillFrame,
    pub contours: Vec<Contour>,
    /// `(y_min, y_max)` when the cut is a real head Y. Absent for interpolation frames.
    pub y_limits: Option<(f64, f64)>,
}

/// Clearance rapids before the live tool, then the interpolation mode, then the cut.
pub(crate) struct MillProgram {
    pub approach: Vec<ClMotion>,
    pub interpolation: Option<MillInterp>,
    pub motions: Vec<ClMotion>,
}

/// Interpolation wrapped around [`MillProgram::motions`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MillInterp {
    Face,
    Cylinder { radius: f64 },
}

#[derive(Clone, Copy)]
struct Uv {
    u: f64,
    v: f64,
    d: f64,
}

#[derive(Clone, Copy)]
enum Step {
    Rapid { u: f64, v: f64, d: f64 },
    Linear { u: f64, v: f64, d: f64, role: MoveRole },
    Arc { u: f64, v: f64, d: f64, cu: f64, cv: f64, cd: f64, ccw: bool, role: MoveRole },
}

pub(crate) fn profile_side(text: &str) -> Result<ProfileSide, CliError> {
    match text {
        "outside" => Ok(ProfileSide::Outside),
        "inside" => Ok(ProfileSide::Inside),
        "on" => Ok(ProfileSide::On),
        other => Err(CliError::invalid(format!("unknown side `{other}`"))),
    }
}

/// Read the drawing next to the job file. `layer` keeps one layer when it is set.
pub(crate) fn load_contours(name: &str, job_path: &Path, relative: &str, layer: Option<&str>, linear: f64) -> Result<Vec<Contour>, CliError> {
    let path = resolve(job_path, relative);
    let text = read_text(&path)?;
    let tolerance = Tolerance::new(linear, Tolerance::DEFAULT_ANGULAR_RAD).map_err(|err| op_err(name, err))?;
    let options = ImportOptions::new(tolerance);
    let ext = path.extension().and_then(|ext| ext.to_str()).unwrap_or("");
    let drawing = match ext.to_ascii_lowercase().as_str() {
        "dxf" => import_dxf(&text, &options).map_err(|err| op_err(name, err))?,
        "svg" => import_svg(&text, &options).map_err(|err| op_err(name, err))?,
        _ => return Err(CliError::invalid(format!("operation {name}: drawing path must end in .dxf or .svg"))),
    };
    let mut contours = Vec::new();
    for item in drawing.contours() {
        if layer.is_some_and(|want| item.layer() != want) {
            continue;
        }
        contours.push(item.contour().clone());
    }
    Ok(contours)
}

/// Build the clearance approach and the cut. The caller wraps C and the live tool around it.
pub(crate) fn build(spec: &MillSpec) -> Result<MillProgram, CliError> {
    if matches!(spec.frame, MillFrame::FrontY { .. }) && spec.y_limits.is_none() {
        return Err(CliError::invalid(format!("operation {}: y_axis needs head.y_axis", spec.name)));
    }
    if let MillFrame::Od { diameter } = spec.frame {
        let span = x_span(&spec.name, &spec.contours)?;
        if span > std::f64::consts::PI * diameter {
            return Err(CliError::invalid(format!("operation {}: drawing wraps past one turn", spec.name)));
        }
    }
    let contours = cutting_contours(&spec.name, spec.strategy, &spec.contours)?;
    let moves = toolpath_moves(spec, &contours)?;
    let (start, steps) = link(&spec.name, &moves, spec.clearance)?;
    let approach = approach_motions(spec, start)?;
    let mut motions = Vec::new();
    if needs_unpark(spec.frame, start) {
        motions.push(rapid(spec, start.u, start.v, start.d)?);
    }
    let mut prev = start;
    for step in steps {
        motions.push(emit_step(spec, step, prev)?);
        prev = end_uv(step);
    }
    let has_feed = motions.iter().any(|motion| matches!(motion, ClMotion::Linear { .. } | ClMotion::Arc { .. } | ClMotion::Helix { .. }));
    if !has_feed {
        return Err(CliError::invalid(format!("operation {}: toolpath is empty", spec.name)));
    }
    Ok(MillProgram { approach, interpolation: interpolation(spec.frame), motions })
}

/// `(u, v, d)` to a machine tip `(x, y, z)`. `d <= 0` is depth into the part.
pub(crate) fn map_point(frame: &MillFrame, u: f64, v: f64, d: f64) -> (f64, f64, f64) {
    match *frame {
        MillFrame::Front { z_start } | MillFrame::FrontY { z_start, .. } => (u, v, z_start + d),
        MillFrame::Od { diameter } => (diameter * 0.5 + d, u, v),
    }
}

pub(crate) fn map_plane(frame: &MillFrame) -> Plane {
    match frame {
        MillFrame::Front { .. } | MillFrame::FrontY { .. } => Plane::Xy,
        MillFrame::Od { .. } => Plane::Yz,
    }
}

pub(crate) fn map_sense(ccw: bool) -> ArcSense {
    if ccw { ArcSense::Ccw } else { ArcSense::Cw }
}

fn interpolation(frame: MillFrame) -> Option<MillInterp> {
    match frame {
        MillFrame::Front { .. } => Some(MillInterp::Face),
        MillFrame::FrontY { .. } => None,
        MillFrame::Od { diameter } => Some(MillInterp::Cylinder { radius: diameter * 0.5 }),
    }
}

fn cutting_contours(name: &str, strategy: MillStrategy, contours: &[Contour]) -> Result<Vec<Contour>, CliError> {
    let needs_closed = matches!(strategy, MillStrategy::Pocket { .. } | MillStrategy::Profile { side: ProfileSide::Inside });
    if needs_closed {
        let closed: Vec<Contour> = contours.iter().filter(|contour| contour.is_closed()).cloned().collect();
        if closed.is_empty() {
            return Err(CliError::invalid(format!("operation {name}: drawing has no closed contour")));
        }
        return Ok(closed);
    }
    if contours.is_empty() {
        return Err(CliError::invalid(format!("operation {name}: drawing has no contour")));
    }
    Ok(contours.to_vec())
}

fn x_span(name: &str, contours: &[Contour]) -> Result<f64, CliError> {
    let mut bounds: Option<BoundingBox> = None;
    for contour in contours {
        let next = contour.bounding_box().map_err(|err| op_err(name, err))?;
        bounds = Some(match bounds {
            Some(have) => have.union(next),
            None => next,
        });
    }
    Ok(bounds.map(|bounds| bounds.width()).unwrap_or(0.0))
}

fn toolpath_moves(spec: &MillSpec, contours: &[Contour]) -> Result<Vec<Move>, CliError> {
    let depths = pass_depths(spec.depth, spec.step_down).map_err(|err| op_err(&spec.name, err))?;
    let tolerance = Tolerance::new(spec.tolerance, Tolerance::DEFAULT_ANGULAR_RAD).map_err(|err| op_err(&spec.name, err))?;
    let tool = Tool::Diameter { diameter: spec.tool_diameter };
    let mut moves = Vec::new();
    match spec.strategy {
        MillStrategy::Profile { side } => {
            for depth in depths {
                let z = -depth;
                for contour in contours {
                    // Drawing Z is depth into the part. The lathe clearance is the approach
                    // plane, so both heights are that clearance and the link stays one plunge
                    // from clearance to depth.
                    let profile = ProfileOp::new(tool, Compensation::Computed, side, CutDirection::Climb)
                        .z(z)
                        .heights(spec.clearance, spec.clearance)
                        .tolerance(tolerance)
                        .generate(contour)
                        .map_err(|err| op_err(&spec.name, err))?;
                    moves.extend(profile.toolpath().moves().iter().copied());
                }
            }
        }
        MillStrategy::Pocket { stepover } => {
            let regions = pocket_regions(&spec.name, contours, &tolerance)?;
            let step_mm = stepover * spec.tool_diameter;
            for depth in depths {
                let z = -depth;
                for region in &regions {
                    let pocket = PocketOp::new(tool, PocketStrategy::OffsetRings, step_mm)
                        .z(z)
                        .retract_z(spec.clearance)
                        .tolerance(tolerance)
                        .generate(region)
                        .map_err(|err| op_err(&spec.name, err))?;
                    moves.extend(pocket.toolpath().moves().iter().copied());
                }
            }
        }
    }
    Ok(moves)
}

/// Even nesting depth is a pocket. The next depth in is a hole. Same rule as a mill pocket.
fn pocket_regions(name: &str, contours: &[Contour], tolerance: &Tolerance) -> Result<Vec<Region>, CliError> {
    if contours.len() == 1 {
        let Some(contour) = contours.first() else {
            return Err(CliError::invalid(format!("operation {name}: drawing has no closed contour")));
        };
        let region = Region::from_contour(contour.clone(), tolerance).map_err(|err| op_err(name, err))?;
        return Ok(vec![region]);
    }
    let mut areas = Vec::with_capacity(contours.len());
    for contour in contours {
        areas.push(contour.signed_area().map_err(|err| op_err(name, err))?.abs());
    }
    let n = contours.len();
    let mut parent = vec![None; n];
    for i in 0..n {
        let Some(inner) = contours.get(i) else {
            continue;
        };
        let mut best: Option<(usize, f64)> = None;
        for j in 0..n {
            if i == j {
                continue;
            }
            let (Some(outer), Some(area)) = (contours.get(j), areas.get(j).copied()) else {
                continue;
            };
            if !strictly_inside(inner, outer, tolerance).map_err(|err| op_err(name, err))? {
                continue;
            }
            let take = match best {
                None => true,
                Some((_, best_area)) => area < best_area,
            };
            if take {
                best = Some((j, area));
            }
        }
        if let Some((j, _)) = best
            && let Some(slot) = parent.get_mut(i)
        {
            *slot = Some(j);
        }
    }
    let depth = nest_depth(name, &parent)?;
    let mut regions = Vec::new();
    for i in 0..n {
        let nest = depth.get(i).copied().unwrap_or(0);
        if nest % 2 != 0 {
            continue;
        }
        let Some(outer) = contours.get(i) else {
            continue;
        };
        let mut holes = Vec::new();
        for j in 0..n {
            if parent.get(j).copied().flatten() == Some(i)
                && depth.get(j).copied().unwrap_or(0) == nest + 1
                && let Some(hole) = contours.get(j)
            {
                holes.push(hole.clone());
            }
        }
        regions.push(Region::try_new(outer.clone(), holes, tolerance).map_err(|err| op_err(name, err))?);
    }
    if regions.is_empty() {
        return Err(CliError::invalid(format!("operation {name}: drawing has no closed contour")));
    }
    Ok(regions)
}

fn nest_depth(name: &str, parent: &[Option<usize>]) -> Result<Vec<usize>, CliError> {
    let n = parent.len();
    let mut depth = vec![0usize; n];
    for i in 0..n {
        let mut d = 0usize;
        let mut cursor = parent.get(i).copied().flatten();
        let mut guard = 0usize;
        while let Some(next) = cursor {
            d = d.saturating_add(1);
            guard = guard.saturating_add(1);
            if guard > n {
                return Err(CliError::invalid(format!("operation {name}: contour nesting did not finish")));
            }
            cursor = parent.get(next).copied().flatten();
        }
        if let Some(slot) = depth.get_mut(i) {
            *slot = d;
        }
    }
    Ok(depth)
}

fn strictly_inside(inner: &Contour, outer: &Contour, tolerance: &Tolerance) -> Result<bool, campanion_geom::GeomError> {
    if !inner.is_closed() || !outer.is_closed() {
        return Ok(false);
    }
    let mut saw_inside = false;
    for seg in inner.segments() {
        match outer.containment(seg.start(), tolerance)? {
            Containment::Outside => return Ok(false),
            Containment::Inside => saw_inside = true,
            Containment::Boundary => {}
        }
    }
    Ok(saw_inside)
}

fn link(name: &str, moves: &[Move], clearance: f64) -> Result<(Uv, Vec<Step>), CliError> {
    let steps_in = collect(moves);
    let Some(first) = steps_in.first().copied() else {
        return Err(CliError::invalid(format!("operation {name}: toolpath is empty")));
    };
    let start_at = end_uv(first);
    let approach = Uv { u: start_at.u, v: start_at.v, d: clearance };
    let mut cursor = approach;
    let mut out = Vec::new();
    let mut index = 0;
    if matches!(first, Step::Rapid { .. }) {
        index = 1;
        if start_at.d < clearance - MOVE_EPS {
            push_linear(&mut out, &mut cursor, start_at, MoveRole::Plunge);
        } else if !same_point(cursor, start_at) {
            out.push(Step::Rapid { u: start_at.u, v: start_at.v, d: start_at.d });
            cursor = start_at;
        }
    }
    for step in steps_in.iter().skip(index).copied() {
        match step {
            Step::Rapid { u, v, d } => {
                let to = Uv { u, v, d };
                if same_uv(cursor, to) {
                    if to.d < cursor.d - MOVE_EPS && to.d < clearance - MOVE_EPS {
                        push_linear(&mut out, &mut cursor, to, MoveRole::Plunge);
                    } else if !same_point(cursor, to) {
                        out.push(Step::Rapid { u, v, d });
                        cursor = to;
                    }
                } else {
                    if cursor.d < clearance - MOVE_EPS {
                        let up = Uv { u: cursor.u, v: cursor.v, d: clearance };
                        out.push(Step::Rapid { u: up.u, v: up.v, d: up.d });
                        cursor = up;
                    }
                    let across = Uv { u, v, d: clearance };
                    if !same_point(cursor, across) {
                        out.push(Step::Rapid { u, v, d: clearance });
                        cursor = across;
                    }
                    if to.d < clearance - MOVE_EPS {
                        push_linear(&mut out, &mut cursor, to, MoveRole::Plunge);
                    }
                }
            }
            Step::Linear { u, v, d, .. } => {
                let to = Uv { u, v, d };
                if !same_point(cursor, to) {
                    out.push(step);
                    cursor = to;
                }
            }
            Step::Arc { .. } => {
                out.push(step);
                cursor = end_uv(step);
            }
        }
    }
    if cursor.d < clearance - MOVE_EPS {
        out.push(Step::Rapid { u: cursor.u, v: cursor.v, d: clearance });
    }
    Ok((approach, out))
}

fn push_linear(out: &mut Vec<Step>, cursor: &mut Uv, to: Uv, role: MoveRole) {
    if same_point(*cursor, to) {
        return;
    }
    out.push(Step::Linear { u: to.u, v: to.v, d: to.d, role });
    *cursor = to;
}

fn collect(moves: &[Move]) -> Vec<Step> {
    let mut steps = Vec::new();
    for mv in moves {
        match *mv {
            Move::Rapid { to } => steps.push(Step::Rapid { u: to.x(), v: to.y(), d: to.z() }),
            Move::Linear { to, kind } => steps.push(Step::Linear { u: to.x(), v: to.y(), d: to.z(), role: role_of(kind) }),
            Move::Arc { to, centre, ccw, kind } => {
                steps.push(Step::Arc { u: to.x(), v: to.y(), d: to.z(), cu: centre.x(), cv: centre.y(), cd: centre.z(), ccw, role: role_of(kind) })
            }
            Move::Event(_) | Move::Dwell { .. } => {}
        }
    }
    steps
}

fn role_of(kind: CutKind) -> MoveRole {
    match kind {
        CutKind::Cut => MoveRole::Cut,
        CutKind::LeadIn => MoveRole::LeadIn,
        CutKind::LeadOut => MoveRole::LeadOut,
        CutKind::Plunge => MoveRole::Plunge,
        CutKind::Retract => MoveRole::Retract,
    }
}

fn end_uv(step: Step) -> Uv {
    match step {
        Step::Rapid { u, v, d } | Step::Linear { u, v, d, .. } | Step::Arc { u, v, d, .. } => Uv { u, v, d },
    }
}

fn same_uv(a: Uv, b: Uv) -> bool {
    (a.u - b.u).hypot(a.v - b.v) <= MOVE_EPS
}

fn same_point(a: Uv, b: Uv) -> bool {
    let du = a.u - b.u;
    let dv = a.v - b.v;
    let dd = a.d - b.d;
    du * du + dv * dv + dd * dd <= MOVE_EPS * MOVE_EPS
}

fn needs_unpark(frame: MillFrame, start: Uv) -> bool {
    match frame {
        MillFrame::Front { .. } => start.v.abs() > PARK_MM,
        MillFrame::Od { .. } => start.u.abs() > PARK_MM,
        MillFrame::FrontY { .. } => false,
    }
}

fn approach_motions(spec: &MillSpec, start: Uv) -> Result<Vec<ClMotion>, CliError> {
    match spec.frame {
        MillFrame::Front { .. } => {
            let v = if start.v.abs() > PARK_MM { 0.0 } else { start.v };
            Ok(vec![rapid(spec, start.u, v, start.d)?])
        }
        MillFrame::Od { .. } => {
            let u = if start.u.abs() > PARK_MM { 0.0 } else { start.u };
            Ok(vec![rapid(spec, u, start.v, start.d)?])
        }
        MillFrame::FrontY { .. } => {
            if start.v.abs() > PARK_MM {
                Ok(vec![rapid(spec, start.u, 0.0, start.d)?, rapid(spec, start.u, start.v, start.d)?])
            } else {
                Ok(vec![rapid(spec, start.u, start.v, start.d)?])
            }
        }
    }
}

fn emit_step(spec: &MillSpec, step: Step, prev: Uv) -> Result<ClMotion, CliError> {
    match step {
        Step::Rapid { u, v, d } => rapid(spec, u, v, d),
        Step::Linear { u, v, d, role } => {
            let to = pose_at(spec, u, v, d)?;
            Ok(ClMotion::Linear { to, feed: spec.feed, role })
        }
        Step::Arc { u, v, d, cu, cv, cd, ccw, role } => {
            if matches!(spec.frame, MillFrame::Front { .. } | MillFrame::FrontY { .. }) {
                for y in arc_extreme_ys(&spec.name, prev, u, v, cu, cv, ccw)? {
                    check_y(&spec.name, y, spec.y_limits)?;
                }
            }
            let to = pose_at(spec, u, v, d)?;
            let (cx, cy, cz) = map_point(&spec.frame, cu, cv, cd);
            let center = Vec3::new(cx, cy, cz).map_err(|err| op_err(&spec.name, err))?;
            let plane = map_plane(&spec.frame);
            let sense = map_sense(ccw);
            let feed = spec.feed;
            if (prev.d - d).abs() > MOVE_EPS {
                let sweep = planar_sweep(&spec.name, prev.u, prev.v, u, v, cu, cv, ccw)?;
                Ok(ClMotion::Helix { to, plane, center, sense, sweep, feed, role })
            } else {
                Ok(ClMotion::Arc { to, plane, center, sense, feed, role })
            }
        }
    }
}

fn rapid(spec: &MillSpec, u: f64, v: f64, d: f64) -> Result<ClMotion, CliError> {
    Ok(ClMotion::Rapid { to: pose_at(spec, u, v, d)?, feed: None })
}

fn pose_at(spec: &MillSpec, u: f64, v: f64, d: f64) -> Result<Pose, CliError> {
    let (x, y, z) = map_point(&spec.frame, u, v, d);
    check_y(&spec.name, y, spec.y_limits)?;
    let axis = match spec.frame {
        MillFrame::Front { .. } | MillFrame::FrontY { .. } => Vec3::Z,
        MillFrame::Od { .. } => Vec3::X,
    };
    let pose = Pose::new(Vec3::new(x, y, z).map_err(|err| op_err(&spec.name, err))?, axis).map_err(|err| op_err(&spec.name, err))?;
    match spec.frame {
        MillFrame::FrontY { c_deg, .. } => pose.with_rotary(c_deg).map_err(|err| op_err(&spec.name, err)),
        MillFrame::Front { .. } | MillFrame::Od { .. } => Ok(pose),
    }
}

fn check_y(name: &str, y: f64, limits: Option<(f64, f64)>) -> Result<(), CliError> {
    let Some((min, max)) = limits else {
        return Ok(());
    };
    if y < min - 1e-9 || y > max + 1e-9 {
        return Err(CliError::invalid(format!("operation {name}: Y {} outside head travel", format_travel(y))));
    }
    Ok(())
}

fn arc_extreme_ys(name: &str, start: Uv, eu: f64, ev: f64, cu: f64, cv: f64, ccw: bool) -> Result<Vec<f64>, CliError> {
    let radius = (start.u - cu).hypot(start.v - cv);
    if !(radius.is_finite() && radius > 1e-15) {
        return Ok(Vec::new());
    }
    let a0 = (start.v - cv).atan2(start.u - cu);
    let sweep = planar_sweep(name, start.u, start.v, eu, ev, cu, cv, ccw)?;
    let mut ys = Vec::new();
    for quadrant in [std::f64::consts::FRAC_PI_2, 3.0 * std::f64::consts::FRAC_PI_2] {
        let delta = if ccw { (quadrant - a0).rem_euclid(TAU) } else { (a0 - quadrant).rem_euclid(TAU) };
        if delta.is_finite() && delta <= sweep + 1e-9 {
            ys.push(cv + radius * quadrant.sin());
        }
    }
    Ok(ys)
}

fn planar_sweep(name: &str, su: f64, sv: f64, eu: f64, ev: f64, cu: f64, cv: f64, ccw: bool) -> Result<f64, CliError> {
    let a0 = (sv - cv).atan2(su - cu);
    let a1 = (ev - cv).atan2(eu - cu);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(CliError::invalid(format!("operation {name}: arc angle is not finite")));
    }
    let chord = (eu - su).hypot(ev - sv);
    if !chord.is_finite() {
        return Err(CliError::invalid(format!("operation {name}: arc chord is not finite")));
    }
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= 1e-12 {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(CliError::invalid(format!("operation {name}: arc sweep is not positive"))) }
}

fn format_travel(value: f64) -> String {
    if value == 0.0 {
        return "0".to_string();
    }
    let text = format!("{value:.9}");
    let Some((head, frac)) = text.split_once('.') else {
        return text;
    };
    let frac = frac.trim_end_matches('0');
    if frac.is_empty() { head.to_string() } else { format!("{head}.{frac}") }
}

fn resolve(job: &Path, relative: &str) -> PathBuf {
    match job.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(relative),
        _ => PathBuf::from(relative),
    }
}

fn op_err(name: &str, err: impl std::fmt::Display) -> CliError {
    let text = err.to_string();
    let detail = text.strip_prefix("invalid input: ").unwrap_or(text.as_str());
    CliError::invalid(format!("operation {name}: {detail}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_geom::{Arc, Line, Point, Segment};

    fn spec(frame: MillFrame, strategy: MillStrategy, contours: Vec<Contour>, y_limits: Option<(f64, f64)>) -> MillSpec {
        MillSpec {
            name: "mill".into(),
            tool_diameter: 4.0,
            feed: 100.0,
            depth: 1.0,
            step_down: 1.0,
            clearance: 2.0,
            tolerance: 1e-4,
            strategy,
            frame,
            contours,
            y_limits,
        }
    }

    fn open_line(x0: f64, y0: f64, x1: f64, y1: f64) -> Contour {
        let line = Line::new(Point::new(x0, y0).unwrap(), Point::new(x1, y1).unwrap()).unwrap();
        Contour::try_new(vec![Segment::Line(line)], false, &Tolerance::default()).unwrap()
    }

    fn pose(motion: &ClMotion) -> &Pose {
        match motion {
            ClMotion::Rapid { to, .. }
            | ClMotion::Linear { to, .. }
            | ClMotion::Arc { to, .. }
            | ClMotion::Helix { to, .. }
            | ClMotion::Thread { to, .. } => to,
        }
    }

    fn tip(motion: &ClMotion) -> Vec3 {
        pose(motion).tip()
    }

    #[test]
    fn maps_front_and_od_points() {
        let front = MillFrame::Front { z_start: 0.0 };
        assert_eq!(map_point(&front, 15.0, -3.0, -2.0), (15.0, -3.0, -2.0));
        let raised = MillFrame::Front { z_start: 1.0 };
        assert_eq!(map_point(&raised, 15.0, -3.0, -2.0), (15.0, -3.0, -1.0));
        let od = MillFrame::Od { diameter: 40.0 };
        let (x, y, z) = map_point(&od, 10.0, -5.0, -1.5);
        assert!((x - 18.5).abs() < 1e-12 && (y - 10.0).abs() < 1e-12 && (z + 5.0).abs() < 1e-12);
        let (clear_x, _, _) = map_point(&od, 0.0, 0.0, 2.0);
        assert!((clear_x - 22.0).abs() < 1e-12);
    }

    #[test]
    fn od_arc_sense_survives_the_yz_mapping() {
        let frame = MillFrame::Od { diameter: 40.0 };
        assert_eq!(map_plane(&frame), Plane::Yz);
        assert_eq!(map_sense(true), ArcSense::Ccw);
        let start = map_point(&frame, 1.0, 0.0, 0.0);
        let end = map_point(&frame, 0.0, 1.0, 0.0);
        let centre = map_point(&frame, 0.0, 0.0, 0.0);
        assert!((start.1 - 1.0).abs() < 1e-12 && start.2.abs() < 1e-12);
        assert!(end.1.abs() < 1e-12 && (end.2 - 1.0).abs() < 1e-12);
        let ccw_delta = yz_ccw_delta(start, end, centre);
        assert!((ccw_delta - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert!(ccw_delta < std::f64::consts::PI);

        assert_eq!(map_sense(false), ArcSense::Cw);
        let cw_end = map_point(&frame, 0.0, -1.0, 0.0);
        let cw_delta = yz_ccw_delta(start, cw_end, centre);
        let cw_sweep = TAU - cw_delta;
        assert!((cw_sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{cw_sweep}");
        assert!((cw_delta - 3.0 * std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }

    fn yz_ccw_delta(start: (f64, f64, f64), end: (f64, f64, f64), centre: (f64, f64, f64)) -> f64 {
        let a0 = (start.2 - centre.2).atan2(start.1 - centre.1);
        let a1 = (end.2 - centre.2).atan2(end.1 - centre.1);
        (a1 - a0).rem_euclid(TAU)
    }

    #[test]
    fn od_profile_arc_keeps_the_short_sense() {
        let arc = Arc::new(Point::new(0.0, 0.0).unwrap(), 10.0, 0.0, std::f64::consts::FRAC_PI_2).unwrap();
        let contour = Contour::try_new(vec![Segment::Arc(arc)], false, &Tolerance::default()).unwrap();
        let frame = MillFrame::Od { diameter: 40.0 };
        let program = build(&spec(frame, MillStrategy::Profile { side: ProfileSide::On }, vec![contour], None)).unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(program.interpolation, Some(MillInterp::Cylinder { radius: 20.0 }));
        let mut prev = tip(program.approach.last().unwrap());
        let mut found = false;
        for motion in &program.motions {
            if let ClMotion::Arc { to, plane, center, sense, .. } = motion {
                assert_eq!(*plane, Plane::Yz);
                assert!((center.x() - 19.0).abs() < 1e-6, "{}", center.x());
                assert!(center.y().abs() < 1e-6 && center.z().abs() < 1e-6);
                let sweep = match sense {
                    ArcSense::Ccw => {
                        yz_ccw_delta((prev.x(), prev.y(), prev.z()), (to.tip().x(), to.tip().y(), to.tip().z()), (center.x(), center.y(), center.z()))
                    }
                    ArcSense::Cw => {
                        TAU - yz_ccw_delta(
                            (prev.x(), prev.y(), prev.z()),
                            (to.tip().x(), to.tip().y(), to.tip().z()),
                            (center.x(), center.y(), center.z()),
                        )
                    }
                };
                assert!((sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-6, "{sweep}");
                found = true;
            }
            prev = tip(motion);
        }
        assert!(found);
    }

    #[test]
    fn front_approach_parks_y_until_interpolation() {
        let contour = open_line(15.0, -5.0, 15.0, 5.0);
        let program = build(&spec(MillFrame::Front { z_start: 0.0 }, MillStrategy::Profile { side: ProfileSide::On }, vec![contour], None))
            .unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(program.interpolation, Some(MillInterp::Face));
        let approach = tip(program.approach.first().unwrap());
        assert!((approach.x() - 15.0).abs() < 1e-9);
        assert!(approach.y().abs() < 1e-12);
        assert!((approach.z() - 2.0).abs() < 1e-9);
        assert!(program.approach.iter().all(|motion| pose(motion).rotary().is_none()));
        let unpark = tip(program.motions.first().unwrap());
        assert!((unpark.y() - 5.0).abs() < 1e-6, "{}", unpark.y());
        assert!((unpark.z() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn errors_for_wrap_open_pocket_and_y_travel() {
        let wide = open_line(0.0, 0.0, 200.0, 0.0);
        let Err(wrapped) = build(&spec(MillFrame::Od { diameter: 40.0 }, MillStrategy::Profile { side: ProfileSide::On }, vec![wide], None)) else {
            panic!("expected a wrap error");
        };
        assert!(wrapped.to_string().contains("drawing wraps past one turn"), "{wrapped}");

        let open = open_line(0.0, 0.0, 10.0, 0.0);
        let Err(pocket) = build(&spec(MillFrame::Front { z_start: 0.0 }, MillStrategy::Pocket { stepover: 0.4 }, vec![open], None)) else {
            panic!("expected a closed-contour error");
        };
        assert!(pocket.to_string().contains("drawing has no closed contour"), "{pocket}");

        let high = open_line(0.0, 40.0, 10.0, 40.0);
        let Err(travel) = build(&spec(
            MillFrame::FrontY { z_start: 0.0, c_deg: 0.0 },
            MillStrategy::Profile { side: ProfileSide::On },
            vec![high],
            Some((-10.0, 10.0)),
        )) else {
            panic!("expected a travel error");
        };
        assert!(travel.to_string().contains("Y 40 outside head travel"), "{travel}");

        let Err(missing) = build(&spec(
            MillFrame::FrontY { z_start: 0.0, c_deg: 30.0 },
            MillStrategy::Profile { side: ProfileSide::On },
            vec![open_line(0.0, 0.0, 5.0, 0.0)],
            None,
        )) else {
            panic!("expected a missing-head error");
        };
        assert!(missing.to_string().contains("y_axis needs head.y_axis"), "{missing}");
    }
}
