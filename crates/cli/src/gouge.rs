//! Mesh gouge check for cutter-location toolpaths.
//!
//! Straight cuts are sampled directly. An arc or helix in the XY plane is sampled along the
//! circle, with Z interpolated from the start to the end. An arc in another plane is rejected:
//! surface operations emit lines, roughing emits XY arcs, and only the XY case has a defined sample.

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, MoveRole, Plane, Program, Vec3 as IrVec};
use campanion_mesh::{Cutter, DropCutter, Mesh, Vec3 as MeshVec, check_gouges};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, OperationKind};
use crate::report::GougeNote;
use crate::rough;
use crate::surface;

/// Deepest gouges printed for one toolpath. The check still fails when there are more.
const MAX_REPORTS: usize = 32;

/// Segments materialised from one toolpath. Each one is at least one gouge sample.
const MAX_SEGMENTS: usize = 50_000_000;

const TAU: f64 = std::f64::consts::TAU;
const FULL_TURN_CHORD_MM: f64 = 1.0e-12;

/// `min(0.25, tolerance * 10)` millimetres. `tolerance` is the job chord tolerance.
pub(crate) fn sample_step_for(tolerance: f64) -> Result<f64, CliError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(CliError::invalid("gouge tolerance must be finite and non-negative"));
    }
    let step = (tolerance * 10.0).min(0.25);
    if step.is_finite() && step > 0.0 { Ok(step) } else { Err(CliError::invalid("gouge sample step must be finite and positive")) }
}

/// `ball:R`, `flat:R`, or `bull:R,C`. The numbers are radii in millimetres.
pub(crate) fn parse_tool(text: &str) -> Result<Cutter, CliError> {
    let Some((kind, rest)) = text.split_once(':') else {
        return Err(CliError::usage("tool must be ball:R, flat:R or bull:R,C"));
    };
    match kind {
        "flat" => Cutter::new_flat(parse_positive(rest, "flat radius")?).map_err(|err| CliError::usage(err.to_string())),
        "ball" => Cutter::new_ball(parse_positive(rest, "ball radius")?).map_err(|err| CliError::usage(err.to_string())),
        "bull" => {
            let Some((radius_text, corner_text)) = rest.split_once(',') else {
                return Err(CliError::usage("bull tool must be bull:R,C"));
            };
            if corner_text.contains(',') {
                return Err(CliError::usage("bull tool must be bull:R,C"));
            }
            let radius = parse_positive(radius_text, "bull radius")?;
            let corner = parse_positive(corner_text, "bull corner radius")?;
            Cutter::new_bull(radius, corner).map_err(|err| CliError::usage(err.to_string()))
        }
        _ => Err(CliError::usage("tool must be ball:R, flat:R or bull:R,C")),
    }
}

/// Check every cutting move of an STL job. Each operation uses its own flat, ball or bull cutter.
///
/// The reference is the drop-cutter tip on the part. `stock_to_leave` is not added. A surface
/// path already includes its axial stock in Z. A rough path sits above the surface plus its
/// stock, so a gouge is a tip below the part, not below the stock allowance.
pub(crate) fn check_job(job: &JobFile, mesh: &Mesh, program: &Program) -> Result<Vec<GougeNote>, CliError> {
    let tolerance = job.tolerance().chord();
    let sample_step = sample_step_for(tolerance)?;
    let paths = program.toolpaths();
    if paths.len() != job.operations().len() {
        return Err(CliError::run("gouge check: toolpath count does not match the job"));
    }
    let mut notes = Vec::new();
    for (index, op) in job.operations().iter().enumerate() {
        let Some(path) = paths.get(index) else {
            return Err(CliError::run("gouge check: missing toolpath"));
        };
        let Some(tool) = job.tool(op.tool_id()) else {
            return Err(CliError::run(format!("operation {}: unknown tool {}", op.name(), op.tool_id())));
        };
        notes.extend(check_one(op.name(), path, mesh, tool, sample_step, tolerance, op)?);
    }
    Ok(notes)
}

/// Check every toolpath in `program` with one cutter. Used by `verify --mesh`.
pub(crate) fn check_program(program: &Program, mesh: &Mesh, cutter: Cutter, tolerance: f64) -> Result<Vec<GougeNote>, CliError> {
    let sample_step = sample_step_for(tolerance)?;
    let dc = DropCutter::new(mesh, cutter).map_err(|err| CliError::run(format!("mesh: {err}")))?;
    let mut notes = Vec::new();
    for (index, path) in program.toolpaths().iter().enumerate() {
        let name = match path.name() {
            Some(name) if !name.is_empty() => name.to_string(),
            _ => format!("toolpath {index}"),
        };
        let segments = cutting_segments(path, sample_step)?;
        notes.extend(notes_for(&name, &dc, &segments, sample_step, tolerance)?);
    }
    Ok(notes)
}

fn check_one(
    name: &str,
    path: &ClPath,
    mesh: &Mesh,
    tool: &JobTool,
    sample_step: f64,
    tolerance: f64,
    op: &Operation,
) -> Result<Vec<GougeNote>, CliError> {
    let cutter = cutter_for(op, tool)?;
    let dc = DropCutter::new(mesh, cutter).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
    let segments = cutting_segments(path, sample_step)?;
    notes_for(name, &dc, &segments, sample_step, tolerance)
}

/// Flat, ball or bull nose taken from the job tool. Roughing uses the same shapes.
fn cutter_for(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    match op.kind() {
        OperationKind::Surface => surface::cutter(op, tool),
        OperationKind::Rough => rough::cutter(op, tool),
        _ => Err(CliError::run(format!("operation {}: gouge check needs a surface or rough operation", op.name()))),
    }
}

fn notes_for(name: &str, dc: &DropCutter, segments: &[(MeshVec, MeshVec)], sample_step: f64, tolerance: f64) -> Result<Vec<GougeNote>, CliError> {
    let (count, gouges) =
        check_gouges(dc, segments, sample_step, tolerance, MAX_REPORTS).map_err(|err| CliError::run(format!("operation {name}: {err}")))?;
    if count == 0 {
        return Ok(Vec::new());
    }
    Ok(gouges
        .iter()
        .map(|gouge| GougeNote { operation: name.to_string(), x: gouge.at.x, y: gouge.at.y, z: gouge.at.z, depth: gouge.depth })
        .collect())
}

fn parse_positive(text: &str, what: &str) -> Result<f64, CliError> {
    let Ok(value) = text.parse::<f64>() else {
        return Err(CliError::usage(format!("{what} must be a number")));
    };
    if value.is_finite() && value > 0.0 { Ok(value) } else { Err(CliError::usage(format!("{what} must be finite and positive"))) }
}

fn cutting(role: MoveRole) -> bool {
    matches!(role, MoveRole::Cut | MoveRole::LeadIn | MoveRole::LeadOut | MoveRole::Plunge)
}

fn mesh_vec(point: IrVec) -> MeshVec {
    MeshVec::new(point.x(), point.y(), point.z())
}

fn cutting_segments(path: &ClPath, sample_step: f64) -> Result<Vec<(MeshVec, MeshVec)>, CliError> {
    let mut cursor: Option<IrVec> = None;
    let mut segments = Vec::new();
    for block in path.blocks() {
        let ClBlock::Motion(motion) = block else {
            continue;
        };
        match motion {
            ClMotion::Rapid { to, .. } => cursor = Some(to.tip()),
            ClMotion::Linear { to, role, .. } => {
                let end = to.tip();
                if cutting(*role) {
                    let start = cursor.unwrap_or(end);
                    push_segment(&mut segments, mesh_vec(start), mesh_vec(end))?;
                }
                cursor = Some(end);
            }
            ClMotion::Arc { to, plane, center, sense, role, .. } => {
                let end = to.tip();
                if cutting(*role) {
                    let Some(start) = cursor else {
                        return Err(CliError::run("gouge check: arc needs a start position"));
                    };
                    append_arc(&mut segments, start, end, *center, *plane, *sense, None, sample_step)?;
                }
                cursor = Some(end);
            }
            ClMotion::Helix { to, plane, center, sense, sweep, role, .. } => {
                let end = to.tip();
                if cutting(*role) {
                    let Some(start) = cursor else {
                        return Err(CliError::run("gouge check: arc needs a start position"));
                    };
                    append_arc(&mut segments, start, end, *center, *plane, *sense, Some(*sweep), sample_step)?;
                }
                cursor = Some(end);
            }
        }
    }
    Ok(segments)
}

fn push_segment(segments: &mut Vec<(MeshVec, MeshVec)>, start: MeshVec, end: MeshVec) -> Result<(), CliError> {
    if segments.len() >= MAX_SEGMENTS {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    segments.push((start, end));
    Ok(())
}

fn plane_name(plane: Plane) -> &'static str {
    match plane {
        Plane::Xy => "XY",
        Plane::Zx => "ZX",
        Plane::Yz => "YZ",
    }
}

/// Sample an XY arc or helix. Z moves linearly from `start` to `end`.
fn append_arc(
    segments: &mut Vec<(MeshVec, MeshVec)>,
    start: IrVec,
    end: IrVec,
    center: IrVec,
    plane: Plane,
    sense: ArcSense,
    sweep: Option<f64>,
    sample_step: f64,
) -> Result<(), CliError> {
    if plane != Plane::Xy {
        return Err(CliError::run(format!(
            "gouge check cannot sample a {} arc; only XY arcs are sampled, and surface operations emit lines",
            plane_name(plane)
        )));
    }
    let (sx, sy, sz) = (start.x(), start.y(), start.z());
    let (ex, ey, ez) = (end.x(), end.y(), end.z());
    let (cx, cy) = (center.x(), center.y());
    for value in [sx, sy, sz, ex, ey, ez, cx, cy, center.z()] {
        if !value.is_finite() {
            return Err(CliError::invalid("arc coordinate is not finite"));
        }
    }
    let radius = (sx - cx).hypot(sy - cy);
    let end_radius = (ex - cx).hypot(ey - cy);
    if !radius.is_finite() || !end_radius.is_finite() || radius <= 1e-9 {
        return Err(CliError::run("gouge check: arc radius is not usable"));
    }
    if (radius - end_radius).abs() > 1.0e-3_f64.max(radius * 1.0e-4) {
        return Err(CliError::run("gouge check: arc endpoints are not on one circle"));
    }
    let ccw = sense == ArcSense::Ccw;
    let sweep = match sweep {
        Some(sweep) if sweep.is_finite() && sweep > 0.0 => sweep,
        Some(_) => return Err(CliError::run("gouge check: arc sweep is not usable")),
        None => xy_sweep(sx, sy, ex, ey, cx, cy, ccw)?,
    };
    let length = (radius * sweep).hypot(ez - sz);
    let count = spaced_points(length, sample_step)?;
    let a0 = (sy - cy).atan2(sx - cx);
    if !a0.is_finite() {
        return Err(CliError::run("gouge check: arc angle is not finite"));
    }
    let signed = if ccw { sweep } else { -sweep };
    let mut previous = None;
    for index in 0..count {
        let point = if index == 0 {
            MeshVec::new(sx, sy, sz)
        } else if index + 1 == count {
            MeshVec::new(ex, ey, ez)
        } else {
            let t = index as f64 / (count - 1) as f64;
            let angle = a0 + signed * t;
            let x = cx + radius * angle.cos();
            let y = cy + radius * angle.sin();
            let z = sz + (ez - sz) * t;
            if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                return Err(CliError::run("gouge check: arc sample is not finite"));
            }
            MeshVec::new(x, y, z)
        };
        if let Some(prev) = previous {
            push_segment(segments, prev, point)?;
        }
        previous = Some(point);
    }
    if count == 1
        && let Some(point) = previous
    {
        push_segment(segments, point, point)?;
    }
    Ok(())
}

fn xy_sweep(sx: f64, sy: f64, ex: f64, ey: f64, cx: f64, cy: f64, ccw: bool) -> Result<f64, CliError> {
    let a0 = (sy - cy).atan2(sx - cx);
    let a1 = (ey - cy).atan2(ex - cx);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(CliError::run("gouge check: arc angle is not finite"));
    }
    let chord = (ex - sx).hypot(ey - sy);
    let ccw_delta = (a1 - a0).rem_euclid(TAU);
    let sweep = if chord <= FULL_TURN_CHORD_MM {
        TAU
    } else if ccw {
        if ccw_delta <= 1e-15 { TAU } else { ccw_delta }
    } else {
        let cw = TAU - ccw_delta;
        if cw <= 1e-15 { TAU } else { cw }
    };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(CliError::run("gouge check: arc sweep is not usable")) }
}

fn spaced_points(length: f64, step: f64) -> Result<usize, CliError> {
    if !length.is_finite() || length < 0.0 {
        return Err(CliError::run("gouge check: arc length is not finite"));
    }
    if length == 0.0 {
        return Ok(1);
    }
    let mut intervals = (length / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(CliError::run("gouge check: arc spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_SEGMENTS as f64 {
        return Err(CliError::invalid(format!("too many gouge samples: limit is {MAX_SEGMENTS}")));
    }
    Ok(count as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{CompensationMode, Feeds, Pose, Tool, Vec3};
    use campanion_mesh::{Mesh, Triangle};

    fn plane() -> Mesh {
        let s = 40.0;
        let z = 5.0;
        Mesh::new(vec![
            Triangle { a: MeshVec::new(-s, -s, z), b: MeshVec::new(s, -s, z), c: MeshVec::new(s, s, z) },
            Triangle { a: MeshVec::new(-s, -s, z), b: MeshVec::new(s, s, z), c: MeshVec::new(-s, s, z) },
        ])
        .expect("plane")
    }

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn ok<T>(result: Result<T, CliError>) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{err}"),
        }
    }

    fn program(name: &str, blocks: Vec<ClBlock>) -> Program {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(Some(name.into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let tool = Tool::try_new(1, None, Some(2.0), None).expect("tool");
        Program::try_new(None, vec![tool], None, vec![path], Vec::new(), Vec::new()).expect("program")
    }

    #[test]
    fn tool_spec_parses_ball_flat_and_bull() {
        assert!(matches!(ok(parse_tool("ball:3")), Cutter::Ball { radius } if (radius - 3.0).abs() < 1e-12));
        assert!(matches!(ok(parse_tool("flat:3")), Cutter::Flat { radius } if (radius - 3.0).abs() < 1e-12));
        assert!(
            matches!(ok(parse_tool("bull:3,1")), Cutter::Bull { radius, corner_radius } if (radius - 3.0).abs() < 1e-12 && (corner_radius - 1.0).abs() < 1e-12)
        );
        assert!(parse_tool("ball").is_err());
        assert!(parse_tool("ball:0").is_err());
        assert!(parse_tool("bull:3").is_err());
        assert!(parse_tool("bull:3,4").is_err());
        assert!(parse_tool("taper:3").is_err());
    }

    #[test]
    fn linear_cut_below_the_plane_names_the_operation() {
        let mesh = plane();
        let surface = DropCutter::new(&mesh, Cutter::new_flat(1.0).expect("flat")).expect("index").tip_z(0.0, 0.0).expect("contact");
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, surface), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(0.2, 0.0, surface - 0.1), feed: 100.0, role: MoveRole::Cut }),
        ];
        let notes = ok(check_program(&program("dome", blocks), &mesh, Cutter::new_flat(1.0).expect("flat"), 0.01));
        let worst = notes.first().expect("gouge");
        assert_eq!(worst.operation, "dome");
        assert!((worst.depth - 0.1).abs() < 1e-9, "{}", worst.depth);
        assert!((worst.x - 0.2).abs() < 1e-9 && worst.y.abs() < 1e-9, "{worst:?}");
    }

    #[test]
    fn xy_arc_on_the_plane_is_clean_and_other_planes_are_rejected() {
        let mesh = plane();
        let surface = DropCutter::new(&mesh, Cutter::new_ball(1.0).expect("ball")).expect("index").tip_z(5.0, 0.0).expect("contact");
        let center = Vec3::new(0.0, 0.0, surface).expect("center");
        let arc = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface),
            plane: Plane::Xy,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface), feed: None }), arc];
        let notes = ok(check_program(&program("arc", blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01));
        assert!(notes.is_empty(), "{notes:?}");

        let low = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface - 0.5),
            plane: Plane::Xy,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let low_blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface - 0.5), feed: None }), low];
        let low_notes = ok(check_program(&program("arc", low_blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01));
        assert!(!low_notes.is_empty(), "lowered arc was not a gouge");
        assert!((low_notes[0].depth - 0.5).abs() < 1e-6, "{:?}", low_notes[0].depth);

        let zx = ClBlock::Motion(ClMotion::Arc {
            to: pose(-5.0, 0.0, surface),
            plane: Plane::Zx,
            center,
            sense: ArcSense::Ccw,
            feed: 100.0,
            role: MoveRole::Cut,
        });
        let zx_blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(5.0, 0.0, surface), feed: None }), zx];
        let err = check_program(&program("arc", zx_blocks), &mesh, Cutter::new_ball(1.0).expect("ball"), 0.01).expect_err("zx");
        let text = err.to_string();
        assert!(text.contains("XY") && text.contains("lines"), "{text}");
    }

    /// A rough cut between the part and the stock allowance is not a gouge. The reference is the
    /// drop-cutter tip of the job's flat, ball or bull tool, with no `stock_to_leave` added.
    #[test]
    fn rough_gouge_uses_the_job_tool_and_ignores_stock_to_leave() {
        let mesh = plane();
        let stock = 1.0;
        let cases = [
            ("flat", r#""flat""#, Cutter::new_flat(3.0).expect("flat")),
            ("ball", r#""ball""#, Cutter::new_ball(3.0).expect("ball")),
            ("bull", r#"{"bull":1}"#, Cutter::new_bull(3.0, 1.0).expect("bull")),
        ];
        for (name, shape, cutter) in cases {
            let job = load_rough_job(name, shape, stock);
            let contact = DropCutter::new(&mesh, cutter).expect("index").tip_z(0.0, 0.0).expect("contact");
            let above_part = contact + stock * 0.25;
            let clean = ok(check_job(&job, &mesh, &program("rough", cut_at(above_part))));
            assert!(clean.is_empty(), "{name} flagged a cut above the part and below the stock: {clean:?}");
            let into = contact - 0.4;
            let gouges = ok(check_job(&job, &mesh, &program("rough", cut_at(into))));
            let Some(worst) = gouges.first() else {
                panic!("{name} cut below the part was not a gouge");
            };
            assert_eq!(worst.operation, "rough");
            assert!((worst.depth - 0.4).abs() < 1e-6, "{name} depth {}", worst.depth);
        }
    }

    fn cut_at(z: f64) -> Vec<ClBlock> {
        vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, z), feed: None }),
            ClBlock::Motion(ClMotion::Linear { to: pose(1.0, 0.0, z), feed: 100.0, role: MoveRole::Cut }),
        ]
    }

    fn load_rough_job(name: &str, shape: &str, stock: f64) -> crate::jobfile::JobFile {
        let text = format!(
            r#"{{"format":"campanion-job/1","source":{{"path":"part.stl","hash":"cbf29ce484222325"}},"units":"mm","tools":[{{"id":1,"diameter":6,"shape":{shape}}}],"operations":[{{"name":"rough","type":"rough","tool_id":1,"step_down":1,"stepover":2,"stock_to_leave":{stock},"feeds":{{"cut":100}}}}]}}"#
        );
        let dir = std::env::temp_dir().join(format!("campanion-gouge-rough-{name}"));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("job.json");
        std::fs::write(&path, text).expect("write job");
        match crate::jobfile::load(&path) {
            Ok(job) => job,
            Err(err) => panic!("rough job {name}: {err}"),
        }
    }
}
