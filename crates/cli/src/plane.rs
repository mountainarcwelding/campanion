//! Indexed 3+2: plan in a tilted work plane, then write the path in the part frame.
//!
//! View +X is the plane `x_dir`, view +Z is the plane normal, and the view origin is the plane
//! origin. View +Y is their right-handed cross product. A 2D operation is planned in that view
//! with the drawing left as drawn. [`map_paths_to_part`] writes each pose back into the part
//! frame with the tool axis along the plane normal and no rotary angle.
//!
//! [`link_frames`] retracts to the job clearance along the plane normal when the next operation
//! uses a different frame. An operation that omits `plane` uses the part axes.

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, Event, MoveRole, Plane, Pose, Vec3};

use crate::error::CliError;
use crate::index::{self, Curve};
use crate::jobfile::WorkPlane;

/// How far two frame components may differ and still be the same frame.
const FRAME_EPS: f64 = 1e-9;

/// Part frame of one tilted work plane. All four vectors are in the part frame. `x`, `y` and `z` are unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ViewFrame {
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl ViewFrame {
    /// `z` is the plane normal, `x` is `x_dir`, and `y` is `z × x`. Each axis is unit length.
    pub(crate) fn from_plane(plane: WorkPlane) -> Result<Self, CliError> {
        let z = unit(plane.normal())?;
        let x = unit(plane.x_dir())?;
        let y = unit(cross(z, x)?)?;
        Ok(Self { origin: plane.origin(), x, y, z })
    }

    /// Part point as view coordinates: `((p - origin) · x, (p - origin) · y, (p - origin) · z)`.
    pub(crate) fn to_view_point(self, point: Vec3) -> Result<Vec3, CliError> {
        let dx = point.x() - self.origin.x();
        let dy = point.y() - self.origin.y();
        let dz = point.z() - self.origin.z();
        Vec3::new(dot_offset(dx, dy, dz, self.x), dot_offset(dx, dy, dz, self.y), dot_offset(dx, dy, dz, self.z)).map_err(CliError::from)
    }

    /// View point as a part point: `origin + v.x * x + v.y * y + v.z * z`.
    pub(crate) fn to_part_point(self, view: Vec3) -> Result<Vec3, CliError> {
        let offset = self.to_part_vector(view)?;
        Vec3::new(self.origin.x() + offset.x(), self.origin.y() + offset.y(), self.origin.z() + offset.z()).map_err(CliError::from)
    }

    /// View direction as a part direction: `v.x * x + v.y * y + v.z * z`.
    pub(crate) fn to_part_vector(self, view: Vec3) -> Result<Vec3, CliError> {
        Vec3::new(
            self.x.x() * view.x() + self.y.x() * view.y() + self.z.x() * view.z(),
            self.x.y() * view.x() + self.y.y() * view.y() + self.z.y() * view.z(),
            self.x.z() * view.x() + self.y.z() * view.y() + self.z.z() * view.z(),
        )
        .map_err(CliError::from)
    }
}

/// Map view-frame toolpaths into the part frame.
///
/// Straight moves keep their shape. An arc or helix whose plane normal lands on XY, ZX or YZ
/// stays a curve. Any other curve is replaced by lines within `chord_mm` of the view curve.
pub(crate) fn map_paths_to_part(paths: &[ClPath], frame: &ViewFrame, chord_mm: f64) -> Result<Vec<ClPath>, CliError> {
    let mut mapped = Vec::with_capacity(paths.len());
    for path in paths {
        mapped.push(map_path(path, frame, chord_mm)?);
    }
    Ok(mapped)
}

/// Retract to `safe_view_z` along the plane normal when the frame changes.
///
/// `None` is the part frame (origin at zero, axes along part +X, +Y and +Z). Frames differ when
/// the origin or an axis differs by more than 1e-9. The operation being left gains a rapid at
/// its own view X and Y and `safe_view_z`. The next operation gains a rapid at its view X and Y
/// and `safe_view_z`. Both rapids use that frame's Z as the tool axis. The first operation that
/// carries a frame gets only the approach rapid.
pub(crate) fn link_frames(groups: Vec<(Option<ViewFrame>, Vec<ClPath>)>, safe_view_z: f64) -> Result<Vec<ClPath>, CliError> {
    let mut out = Vec::new();
    let mut prev: Option<ViewFrame> = None;
    for (frame_opt, mut paths) in groups {
        let frame = frame_opt.unwrap_or_else(identity_frame);
        match prev {
            None => {
                if frame_opt.is_some() {
                    prepend_safe(&mut paths, frame, safe_view_z)?;
                }
            }
            Some(prev_frame) if frames_differ(prev_frame, frame) => {
                append_safe(&mut out, prev_frame, safe_view_z)?;
                prepend_safe(&mut paths, frame, safe_view_z)?;
            }
            Some(_) => {}
        }
        out.extend(paths);
        prev = Some(frame);
    }
    Ok(out)
}

fn map_path(path: &ClPath, frame: &ViewFrame, chord_mm: f64) -> Result<ClPath, CliError> {
    let mut blocks = Vec::with_capacity(path.blocks().len());
    let mut cursor: Option<Vec3> = None;
    for block in path.blocks() {
        blocks.extend(map_block(block, frame, chord_mm, &mut cursor)?);
    }
    let pierce = match path.pierce() {
        Some(point) => Some(frame.to_part_point(point)?),
        None => None,
    };
    ClPath::try_new(path.name().map(str::to_string), path.tool_id(), path.compensation(), path.feeds(), pierce, path.tabs().to_vec(), blocks)
        .map_err(CliError::from)
}

fn map_block(block: &ClBlock, frame: &ViewFrame, chord_mm: f64, cursor: &mut Option<Vec3>) -> Result<Vec<ClBlock>, CliError> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, feed }) => {
            let mapped = map_pose(frame, *to)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Rapid { to: mapped, feed: *feed })])
        }
        ClBlock::Motion(ClMotion::Linear { to, feed, role }) => {
            let mapped = map_pose(frame, *to)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Linear { to: mapped, feed: *feed, role: *role })])
        }
        ClBlock::Motion(ClMotion::Thread { to, lead }) => {
            let mapped = map_pose(frame, *to)?;
            *cursor = Some(to.tip());
            Ok(vec![ClBlock::Motion(ClMotion::Thread { to: mapped, lead: *lead })])
        }
        ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, feed, role }) => {
            map_curve(Curve::Arc, *to, *plane, *center, *sense, None, *feed, *role, frame, chord_mm, cursor)
        }
        ClBlock::Motion(ClMotion::Helix { to, plane, center, sense, sweep, feed, role }) => {
            map_curve(Curve::Helix, *to, *plane, *center, *sense, Some(*sweep), *feed, *role, frame, chord_mm, cursor)
        }
        ClBlock::Event(Event::Probe { toward, feed }) => {
            let toward = match toward {
                Some(pose) => Some(map_pose(frame, *pose)?),
                None => None,
            };
            Ok(vec![ClBlock::Event(Event::Probe { toward, feed: *feed })])
        }
        ClBlock::Event(other) => Ok(vec![ClBlock::Event(other.clone())]),
    }
}

fn map_pose(frame: &ViewFrame, pose: Pose) -> Result<Pose, CliError> {
    Pose::new(frame.to_part_point(pose.tip())?, frame.z).map_err(CliError::from)
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
    frame: &ViewFrame,
    chord_mm: f64,
    cursor: &mut Option<Vec3>,
) -> Result<Vec<ClBlock>, CliError> {
    let part_normal = frame.to_part_vector(index::plane_normal(plane))?;
    if let Some((new_plane, flip)) = index::plane_from_normal(part_normal) {
        let block = index::curve_block(
            curve,
            map_pose(frame, to)?,
            new_plane,
            frame.to_part_point(center)?,
            index::apply_flip(sense, flip),
            sweep,
            feed,
            role,
        )?;
        *cursor = Some(to.tip());
        return Ok(vec![ClBlock::Motion(block)]);
    }
    linearize_curve(to, plane, center, sense, sweep, feed, role, frame, chord_mm, cursor)
}

fn linearize_curve(
    to: Pose,
    plane: Plane,
    center: Vec3,
    sense: ArcSense,
    sweep: Option<f64>,
    feed: f64,
    role: MoveRole,
    frame: &ViewFrame,
    chord_mm: f64,
    cursor: &mut Option<Vec3>,
) -> Result<Vec<ClBlock>, CliError> {
    let Some(start) = *cursor else {
        return Err(CliError::run("index arc needs a start position"));
    };
    let samples = index::curve_samples(start, to.tip(), plane, center, sense, sweep, chord_mm)?;
    let mut blocks = Vec::with_capacity(samples.len());
    for (index, sample) in samples.iter().enumerate() {
        let pose = if index + 1 == samples.len() {
            map_pose(frame, to)?
        } else {
            Pose::new(frame.to_part_point(*sample)?, frame.z).map_err(CliError::from)?
        };
        blocks.push(ClBlock::Motion(ClMotion::Linear { to: pose, feed, role }));
    }
    *cursor = Some(to.tip());
    Ok(blocks)
}

fn prepend_safe(paths: &mut [ClPath], frame: ViewFrame, safe_view_z: f64) -> Result<(), CliError> {
    let pose = safe_pose(frame, first_motion_tip(paths)?, safe_view_z)?;
    let Some(first) = paths.first_mut() else {
        return Err(CliError::run("plane link needs a toolpath"));
    };
    *first = index::prepend_rapid(first, pose)?;
    Ok(())
}

fn append_safe(paths: &mut [ClPath], frame: ViewFrame, safe_view_z: f64) -> Result<(), CliError> {
    let pose = safe_pose(frame, last_motion_tip(paths)?, safe_view_z)?;
    let Some(last) = paths.last_mut() else {
        return Err(CliError::run("plane link needs a toolpath"));
    };
    *last = index::append_rapid(last, pose)?;
    Ok(())
}

fn safe_pose(frame: ViewFrame, tip: Vec3, safe_view_z: f64) -> Result<Pose, CliError> {
    let view = frame.to_view_point(tip)?;
    let at_safe = Vec3::new(view.x(), view.y(), safe_view_z)?;
    Pose::new(frame.to_part_point(at_safe)?, frame.z).map_err(CliError::from)
}

fn identity_frame() -> ViewFrame {
    ViewFrame { origin: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }
}

fn frames_differ(left: ViewFrame, right: ViewFrame) -> bool {
    separated(left.origin, right.origin) || separated(left.x, right.x) || separated(left.y, right.y) || separated(left.z, right.z)
}

fn separated(left: Vec3, right: Vec3) -> bool {
    (left.x() - right.x()).abs() > FRAME_EPS || (left.y() - right.y()).abs() > FRAME_EPS || (left.z() - right.z()).abs() > FRAME_EPS
}

fn first_motion_tip(paths: &[ClPath]) -> Result<Vec3, CliError> {
    for path in paths {
        for block in path.blocks() {
            if let Some(pose) = motion_pose(block) {
                return Ok(pose.tip());
            }
        }
    }
    Err(CliError::run("plane link needs a tool position"))
}

fn last_motion_tip(paths: &[ClPath]) -> Result<Vec3, CliError> {
    for path in paths.iter().rev() {
        for block in path.blocks().iter().rev() {
            if let Some(pose) = motion_pose(block) {
                return Ok(pose.tip());
            }
        }
    }
    Err(CliError::run("plane link needs a tool position"))
}

fn motion_pose(block: &ClBlock) -> Option<Pose> {
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

fn cross(a: Vec3, b: Vec3) -> Result<Vec3, CliError> {
    Vec3::new(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(), a.x() * b.y() - a.y() * b.x()).map_err(CliError::from)
}

fn unit(vector: Vec3) -> Result<Vec3, CliError> {
    let len = (vector.x() * vector.x() + vector.y() * vector.y() + vector.z() * vector.z()).sqrt();
    if !len.is_finite() || len <= 1e-12 {
        return Err(CliError::run("plane axes are not a frame"));
    }
    Vec3::new(vector.x() / len, vector.y() / len, vector.z() / len).map_err(CliError::from)
}

fn dot_offset(dx: f64, dy: f64, dz: f64, axis: Vec3) -> f64 {
    dx * axis.x() + dy * axis.y() + dz * axis.z()
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{CompensationMode, Feeds};

    fn must<T>(result: Result<T, CliError>, what: &str) -> T {
        match result {
            Ok(value) => value,
            Err(err) => panic!("{what}: {err}"),
        }
    }

    fn vec(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3::new(x, y, z).expect("vec")
    }

    fn dist(left: Vec3, right: Vec3) -> f64 {
        let dx = left.x() - right.x();
        let dy = left.y() - right.y();
        let dz = left.z() - right.z();
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn close(left: Vec3, right: Vec3) {
        let gap = dist(left, right);
        assert!(gap <= 1e-9, "gap {gap}");
    }

    fn pose_at(tip: Vec3) -> Pose {
        Pose::new(tip, Vec3::Z).expect("pose")
    }

    fn path_of(blocks: Vec<ClBlock>) -> ClPath {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        ClPath::try_new(Some("cut".into()), Some(1), CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path")
    }

    fn tilt30() -> ViewFrame {
        let normal = vec(0.0, -0.5, 0.8660254037844386);
        must(ViewFrame::from_plane(WorkPlane::new(vec(4.0, -1.0, 2.0), normal, vec(1.0, 0.0, 0.0))), "frame")
    }

    fn side_frame() -> ViewFrame {
        must(ViewFrame::from_plane(WorkPlane::new(vec(0.0, -20.0, -5.0), vec(0.0, -1.0, 0.0), vec(1.0, 0.0, 0.0))), "side")
    }

    fn quarter_arc() -> Vec<ClBlock> {
        vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose_at(vec(10.0, 0.0, 0.0)), feed: None }),
            ClBlock::Motion(ClMotion::Arc {
                to: pose_at(vec(0.0, 10.0, 0.0)),
                plane: Plane::Xy,
                center: vec(0.0, 0.0, 0.0),
                sense: ArcSense::Ccw,
                feed: 100.0,
                role: MoveRole::Cut,
            }),
        ]
    }

    #[test]
    fn view_frame_round_trips_points_and_vectors() {
        let frame = tilt30();
        let point = vec(8.0, 3.0, -6.0);
        let view = must(frame.to_view_point(point), "view");
        close(must(frame.to_part_point(view), "part"), point);
        let direction = vec(0.2, -0.5, 0.7);
        let mapped = must(frame.to_part_vector(direction), "vector");
        let back_x = mapped.x() * frame.x.x() + mapped.y() * frame.x.y() + mapped.z() * frame.x.z();
        let back_y = mapped.x() * frame.y.x() + mapped.y() * frame.y.y() + mapped.z() * frame.y.z();
        let back_z = mapped.x() * frame.z.x() + mapped.y() * frame.z.y() + mapped.z() * frame.z.z();
        assert!((back_x - direction.x()).abs() <= 1e-9, "{back_x}");
        assert!((back_y - direction.y()).abs() <= 1e-9, "{back_y}");
        assert!((back_z - direction.z()).abs() <= 1e-9, "{back_z}");
        let y_len = (frame.y.x() * frame.y.x() + frame.y.y() * frame.y.y() + frame.y.z() * frame.y.z()).sqrt();
        assert!((y_len - 1.0).abs() <= 1e-9, "{y_len}");
        let dot_xy = frame.x.x() * frame.y.x() + frame.x.y() * frame.y.y() + frame.x.z() * frame.y.z();
        let dot_zy = frame.z.x() * frame.y.x() + frame.z.y() * frame.y.y() + frame.z.z() * frame.y.z();
        assert!(dot_xy.abs() <= 1e-9 && dot_zy.abs() <= 1e-9, "{dot_xy} {dot_zy}");
    }

    #[test]
    fn tilted_xy_arc_is_linearized_and_keeps_its_end() {
        let frame = tilt30();
        let mapped = must(map_paths_to_part(&[path_of(quarter_arc())], &frame, 0.05), "map");
        let path = &mapped[0];
        let mut lines = 0;
        let mut end = None;
        for block in path.blocks() {
            match block {
                ClBlock::Motion(ClMotion::Arc { .. } | ClMotion::Helix { .. }) => panic!("tilted arc stayed a curve"),
                ClBlock::Motion(ClMotion::Linear { to, .. }) => {
                    lines += 1;
                    end = Some(to.tip());
                    close(to.axis(), frame.z);
                }
                _ => {}
            }
        }
        assert!(lines > 1, "arc was not sampled");
        let expected = must(frame.to_part_point(vec(0.0, 10.0, 0.0)), "end");
        close(end.expect("end"), expected);
    }

    #[test]
    fn side_xy_arc_stays_on_the_part_xz_plane() {
        let frame = side_frame();
        let mapped = must(map_paths_to_part(&[path_of(quarter_arc())], &frame, 0.05), "map");
        let mut arc = None;
        for block in mapped[0].blocks() {
            if let ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, .. }) = block {
                arc = Some((*to, *plane, *center, *sense));
            }
        }
        let (to, plane, center, sense) = arc.expect("arc");
        assert_eq!(plane, Plane::Zx);
        assert_eq!(sense, ArcSense::Cw);
        close(to.tip(), must(frame.to_part_point(vec(0.0, 10.0, 0.0)), "end"));
        close(center, must(frame.to_part_point(vec(0.0, 0.0, 0.0)), "center"));
        close(to.axis(), frame.z);
    }

    #[test]
    fn link_frames_inserts_the_two_rapids() {
        let side = side_frame();
        let safe = 8.0;
        let left = path_of(vec![ClBlock::Motion(ClMotion::Linear { to: pose_at(vec(1.0, 2.0, 3.0)), feed: 100.0, role: MoveRole::Cut })]);
        let hole = must(side.to_part_point(vec(4.0, 5.0, -2.0)), "hole");
        let right = path_of(vec![ClBlock::Motion(ClMotion::Linear { to: pose_at(hole), feed: 100.0, role: MoveRole::Cut })]);
        let same = path_of(vec![ClBlock::Motion(ClMotion::Linear { to: pose_at(hole), feed: 100.0, role: MoveRole::Cut })]);
        let linked = must(link_frames(vec![(None, vec![left]), (Some(side), vec![right]), (Some(side), vec![same])], safe), "link");
        assert_eq!(linked.len(), 3);
        let retract = motion_pose(linked[0].blocks().last().expect("retract")).expect("retract pose");
        assert!(matches!(linked[0].blocks().last(), Some(ClBlock::Motion(ClMotion::Rapid { .. }))));
        close(retract.tip(), vec(1.0, 2.0, safe));
        close(retract.axis(), Vec3::Z);
        let approach = motion_pose(linked[1].blocks().first().expect("approach")).expect("approach pose");
        assert!(matches!(linked[1].blocks().first(), Some(ClBlock::Motion(ClMotion::Rapid { .. }))));
        close(approach.tip(), must(side.to_part_point(vec(4.0, 5.0, safe)), "safe"));
        close(approach.axis(), side.z);
        assert_eq!(linked[1].blocks().len(), 2);
        assert_eq!(linked[2].blocks().len(), 1, "same frame must not add a rapid");

        let alone = path_of(vec![ClBlock::Motion(ClMotion::Linear { to: pose_at(hole), feed: 100.0, role: MoveRole::Cut })]);
        let first = must(link_frames(vec![(Some(side), vec![alone])], safe), "first");
        assert_eq!(first[0].blocks().len(), 2);
        assert!(matches!(first[0].blocks().first(), Some(ClBlock::Motion(ClMotion::Rapid { .. }))));
    }
}
