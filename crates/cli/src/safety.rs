//! Rapids must stay at or above the stock top / R plane while they are inside the part.
//!
//! Planar operations are checked in the planning frame, where the tool axis is +Z. Wrap and
//! rotary finish are already in the part frame, so they are checked by radius from the rotary
//! axis. A 3-axis program also turns the IR verifier's rapid-height findings into errors when
//! they sit below the job retract height.

use campanion_geom::{BoundingBox, Contour};
use campanion_ir::{ClBlock, ClMotion, ClPath, Vec3, VerifyReport};

use crate::error::CliError;

const PLANE_EPS: f64 = 1e-6;
const XY_EPS: f64 = 1e-9;
/// Stock top for 2.5D mill programs. Z is up.
const STOCK_TOP_Z: f64 = 0.0;

/// Inclusive XY region the tool can be over the part, grown by the cutter radius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Footprint {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) max_x: f64,
    pub(crate) max_y: f64,
}

impl Footprint {
    /// Bounding box of every contour, expanded by `grow` on each side.
    pub(crate) fn from_contours(contours: &[Contour], grow: f64) -> Result<Self, String> {
        let mut bounds: Option<BoundingBox> = None;
        for contour in contours {
            let next = contour.bounding_box().map_err(|err| err.to_string())?;
            bounds = Some(match bounds {
                None => next,
                Some(bounds) => bounds.union(next),
            });
        }
        let Some(bounds) = bounds else {
            return Err("no contours for the footprint".to_string());
        };
        Ok(Self::from_aabb(bounds.min().x(), bounds.min().y(), bounds.max().x(), bounds.max().y(), grow))
    }

    /// Axis-aligned box expanded by `grow` on each side.
    pub(crate) fn from_aabb(min_x: f64, min_y: f64, max_x: f64, max_y: f64, grow: f64) -> Self {
        Self { min_x: min_x - grow, min_y: min_y - grow, max_x: max_x + grow, max_y: max_y + grow }
    }

    /// Inclusive, with `1e-6` mm of slack on every edge.
    pub(crate) fn contains(self, x: f64, y: f64) -> bool {
        x >= self.min_x - PLANE_EPS && x <= self.max_x + PLANE_EPS && y >= self.min_y - PLANE_EPS && y <= self.max_y + PLANE_EPS
    }
}

/// Reject a rapid that ends, or travels, below `floor_z` inside `footprint`.
///
/// `paths` are in the operation's planning frame. The previous tip starts unset and carries
/// across paths. Events do not move it. When a rapid both ends and traverses below the plane,
/// the end error is returned.
pub(crate) fn check_planar(op_name: &str, paths: &[ClPath], footprint: &Footprint, floor_z: f64) -> Result<(), CliError> {
    let mut previous: Option<Vec3> = None;
    for path in paths {
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            let end = motion_tip(motion);
            if matches!(motion, ClMotion::Rapid { .. }) {
                if footprint.contains(end.x(), end.y()) && end.z() < floor_z - PLANE_EPS {
                    return Err(CliError::run(format!(
                        "operation {op_name}: rapid ends at Z {z:.3} below the safe plane {floor_z:.3} inside the part",
                        z = end.z()
                    )));
                }
                if let Some(start) = previous {
                    let xy = (end.x() - start.x()).abs() > XY_EPS || (end.y() - start.y()).abs() > XY_EPS;
                    let low = start.z().min(end.z());
                    let inside = footprint.contains(start.x(), start.y()) || footprint.contains(end.x(), end.y());
                    if xy && low < floor_z - PLANE_EPS && inside {
                        return Err(CliError::run(format!(
                            "operation {op_name}: rapid traverse at Z {low:.3} below the safe plane {floor_z:.3} inside the part"
                        )));
                    }
                }
            }
            previous = Some(end);
        }
    }
    Ok(())
}

/// Reject a rapid that ends closer to the rotary axis than every feed tip.
///
/// `paths` are in the part frame. `floor` is the largest radius of a non-rapid motion end. An
/// operation with no feed moves has nothing to compare and passes.
pub(crate) fn check_radial(op_name: &str, paths: &[ClPath], axis_point: Vec3, axis_dir: Vec3) -> Result<(), CliError> {
    let dir = unit_axis(op_name, axis_dir)?;
    let mut floor: Option<f64> = None;
    for path in paths {
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else {
                continue;
            };
            if matches!(motion, ClMotion::Rapid { .. }) {
                continue;
            }
            let radius = axis_radius(motion_tip(motion), axis_point, dir);
            floor = Some(floor.map_or(radius, |floor| floor.max(radius)));
        }
    }
    let Some(floor) = floor else {
        return Ok(());
    };
    for path in paths {
        for block in path.blocks() {
            let ClBlock::Motion(ClMotion::Rapid { to, .. }) = block else {
                continue;
            };
            let radius = axis_radius(to.tip(), axis_point, dir);
            if radius < floor - PLANE_EPS {
                return Err(CliError::run(format!("operation {op_name}: rapid ends at radius {radius:.3} below the cut surface {floor:.3}")));
            }
        }
    }
    Ok(())
}

/// Verifier rapid findings that a 3-axis program must reject.
///
/// Retract warnings (the rapid ends below `retract`) come first. Safe-height XY rapids follow,
/// and only when their lowest Z is below `retract`. A traverse that stays between the retract
/// and clearance heights is left out. The floor is also the stock top (Z 0), so a rapid through
/// the stock is an error even when `retract` is below the stock. Toolpath and block numbers are
/// 1-based.
pub(crate) fn blocking_rapids(report: &VerifyReport, retract: f64) -> Vec<String> {
    let plane = retract.max(STOCK_TOP_Z);
    let mut lines = Vec::new();
    for (toolpath, _channel, block, z, _limit) in report.rapid_retract_warnings() {
        if *z < plane - PLANE_EPS {
            let (toolpath, block) = place(*toolpath, *block);
            lines.push(format!("toolpath {toolpath} block {block}: rapid ends at Z {z:.3} below the safe plane {plane:.3}"));
        }
    }
    for (toolpath, _channel, block, z, _limit) in report.rapid_safe_errors() {
        if *z < plane - PLANE_EPS {
            let (toolpath, block) = place(*toolpath, *block);
            lines.push(format!("toolpath {toolpath} block {block}: rapid traverse at Z {z:.3} below the safe plane {plane:.3}"));
        }
    }
    lines
}

fn place(toolpath: Option<usize>, block: usize) -> (usize, usize) {
    (toolpath.map_or(0, |index| index.saturating_add(1)), block.saturating_add(1))
}

fn motion_tip(motion: &ClMotion) -> Vec3 {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => to.tip(),
    }
}

fn unit_axis(op_name: &str, dir: Vec3) -> Result<(f64, f64, f64), CliError> {
    let len = (dir.x() * dir.x() + dir.y() * dir.y() + dir.z() * dir.z()).sqrt();
    if !len.is_finite() || len <= 1e-15 {
        return Err(CliError::run(format!("operation {op_name}: rotary axis direction is zero")));
    }
    Ok((dir.x() / len, dir.y() / len, dir.z() / len))
}

fn axis_radius(point: Vec3, origin: Vec3, dir: (f64, f64, f64)) -> f64 {
    let vx = point.x() - origin.x();
    let vy = point.y() - origin.y();
    let vz = point.z() - origin.z();
    let cx = vy * dir.2 - vz * dir.1;
    let cy = vz * dir.0 - vx * dir.2;
    let cz = vx * dir.1 - vy * dir.0;
    (cx * cx + cy * cy + cz * cz).sqrt()
}

#[cfg(test)]
mod tests {
    use campanion_geom::{Contour, Line, Point, Segment, Tolerance};
    use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Pose, Program, Vec3, VerifyOptions, verify_with};

    use super::*;

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn rapid(x: f64, y: f64, z: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, z), feed: None })
    }

    fn linear(x: f64, y: f64, z: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Linear { to: pose(x, y, z), feed: 100.0, role: MoveRole::Cut })
    }

    fn path(blocks: Vec<ClBlock>) -> ClPath {
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        ClPath::try_new(None, None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path")
    }

    fn square() -> Contour {
        let tolerance = Tolerance::new(1e-6, 1e-6).expect("tolerance");
        let point = |x, y| Point::new(x, y).expect("point");
        let line = |a, b| Segment::Line(Line::new(a, b).expect("line"));
        Contour::try_new(
            vec![
                line(point(0.0, 0.0), point(10.0, 0.0)),
                line(point(10.0, 0.0), point(10.0, 10.0)),
                line(point(10.0, 10.0), point(0.0, 10.0)),
                line(point(0.0, 10.0), point(0.0, 0.0)),
            ],
            true,
            &tolerance,
        )
        .expect("contour")
    }

    fn message(result: Result<(), CliError>) -> String {
        match result {
            Ok(()) => String::new(),
            Err(err) => err.to_string(),
        }
    }

    #[test]
    fn footprint_grows_and_contains_the_edge() {
        let footprint = Footprint::from_contours(std::slice::from_ref(&square()), 2.0).expect("footprint");
        assert_eq!(footprint.min_x, -2.0);
        assert_eq!(footprint.max_y, 12.0);
        assert!(footprint.contains(12.0 + 1e-6, 5.0));
        assert!(!footprint.contains(12.0 + 2e-6, 5.0));
        let err = Footprint::from_contours(&[], 0.0).expect_err("empty");
        assert!(err.contains("no contours"));
    }

    #[test]
    fn planar_rejects_the_old_drill_rapid_and_accepts_the_r_plane() {
        let footprint = Footprint::from_aabb(10.0, 25.0, 20.0, 35.0, 0.0);
        let low = check_planar("drill", &[path(vec![rapid(15.0, 30.0, -3.0)])], &footprint, 2.0);
        assert_eq!(message(low), "operation drill: rapid ends at Z -3.000 below the safe plane 2.000 inside the part");
        assert!(check_planar("drill", &[path(vec![rapid(15.0, 30.0, 2.0)])], &footprint, 2.0).is_ok());
        assert!(check_planar("drill", &[path(vec![rapid(100.0, 100.0, -3.0)])], &footprint, 2.0).is_ok());
    }

    #[test]
    fn planar_rejects_a_traverse_and_prefers_the_end_error() {
        let footprint = Footprint::from_aabb(0.0, 0.0, 10.0, 10.0, 0.0);
        let traverse = check_planar("pocket", &[path(vec![linear(0.0, 0.0, 1.0), rapid(5.0, 0.0, 4.0)])], &footprint, 2.0);
        assert_eq!(message(traverse), "operation pocket: rapid traverse at Z 1.000 below the safe plane 2.000 inside the part");
        let both = check_planar("pocket", &[path(vec![linear(0.0, 0.0, 0.0), rapid(5.0, 0.0, -1.0)])], &footprint, 2.0);
        assert!(message(both).contains("rapid ends at Z -1.000"));
        assert!(check_planar("pocket", &[path(vec![linear(0.0, 0.0, 5.0), rapid(0.0, 0.0, -1.0)])], &footprint, 2.0).is_err());
    }

    #[test]
    fn planar_carries_the_tip_across_paths_and_ignores_events() {
        let footprint = Footprint::from_aabb(0.0, 0.0, 1.0, 1.0, 0.0);
        let paths = vec![path(vec![linear(0.0, 0.0, 0.0)]), path(vec![rapid(4.0, 0.0, 0.0)])];
        let err = check_planar("face", &paths, &footprint, 2.0);
        assert!(message(err).contains("rapid traverse at Z 0.000"));
    }

    #[test]
    fn radial_rejects_a_rapid_inside_the_cut_and_accepts_the_surface() {
        let origin = Vec3::ZERO;
        let low = check_radial("wrap", &[path(vec![linear(10.0, 0.0, 0.0), rapid(3.0, 0.0, 0.0)])], origin, Vec3::Z);
        assert_eq!(message(low), "operation wrap: rapid ends at radius 3.000 below the cut surface 10.000");
        assert!(check_radial("wrap", &[path(vec![linear(10.0, 0.0, 0.0), rapid(12.0, 0.0, 5.0)])], origin, Vec3::Z).is_ok());
        assert!(check_radial("wrap", &[path(vec![rapid(0.0, 0.0, 0.0)])], origin, Vec3::Z).is_ok());
        let along_x = check_radial("rotary", &[path(vec![linear(0.0, 0.0, 8.0), rapid(1.0, 0.0, 1.0)])], origin, Vec3::X);
        assert!(message(along_x).contains("rapid ends at radius 1.000"));
        assert!(check_radial("wrap", &[path(vec![linear(1.0, 0.0, 0.0)])], origin, Vec3::ZERO).is_err());
    }

    #[test]
    fn blocking_rapids_use_one_based_indexes_and_keep_retract_traverses() {
        let blocks = vec![rapid(0.0, 0.0, 10.0), rapid(4.0, 0.0, 1.0), rapid(8.0, 0.0, 6.0), rapid(12.0, 0.0, 5.0)];
        let program = Program::try_new(None, Vec::new(), None, vec![path(blocks)], Vec::new(), Vec::new(), None).expect("program");
        let report = verify_with(&program, &VerifyOptions { safe_z: Some(10.0), retract_z: Some(5.0) });
        assert_eq!(
            blocking_rapids(&report, 5.0),
            vec![
                "toolpath 1 block 2: rapid ends at Z 1.000 below the safe plane 5.000".to_string(),
                "toolpath 1 block 2: rapid traverse at Z 1.000 below the safe plane 5.000".to_string(),
                "toolpath 1 block 3: rapid traverse at Z 1.000 below the safe plane 5.000".to_string(),
            ]
        );

        let face = vec![rapid(0.0, 0.0, 10.0), rapid(10.0, 0.0, 5.0)];
        let face_program = Program::try_new(None, Vec::new(), None, vec![path(face)], Vec::new(), Vec::new(), None).expect("program");
        let face_report = verify_with(&face_program, &VerifyOptions { safe_z: Some(10.0), retract_z: Some(5.0) });
        assert_eq!(face_report.rapid_safe_errors().len(), 1);
        assert!(blocking_rapids(&face_report, 5.0).is_empty());
    }
}
