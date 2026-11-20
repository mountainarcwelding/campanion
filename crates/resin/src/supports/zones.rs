//! Painted support and no-support zones.
//!
//! Zones are stored per triangle of the part, so they survive moving and rotating the part (the
//! pose keeps triangle order). A painted support face that faces down gets supports at the zone
//! spacing even below the overhang angle. A no-support face never gets an automatic support;
//! supports added by hand are kept, because the user placed them on purpose.

use campanion_mesh::{Mesh, Vec3};
use serde::{Deserialize, Serialize};

use crate::error::{ResinError, Result};
use crate::supports::grid::VerticalHitGrid;
use crate::supports::place::{MAX_SUPPORT_POINTS, PointSource, SupportPoint, sample_triangle};

/// Schema tag for a zones file.
pub const ZONES_SCHEMA: &str = "campanion-resin-zones/1";

/// What a painted face asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZoneMark {
    None,
    /// Put supports here even when the face is below the overhang angle.
    Support,
    /// Never put an automatic support here.
    Block,
}

/// One mark per part triangle.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FaceZones {
    marks: Vec<u8>,
}

const MARK_NONE: u8 = 0;
const MARK_SUPPORT: u8 = 1;
const MARK_BLOCK: u8 = 2;

fn mark_byte(mark: ZoneMark) -> u8 {
    match mark {
        ZoneMark::None => MARK_NONE,
        ZoneMark::Support => MARK_SUPPORT,
        ZoneMark::Block => MARK_BLOCK,
    }
}

impl FaceZones {
    /// No marks for a mesh of `triangle_count` faces.
    pub fn new(triangle_count: usize) -> Self {
        Self { marks: vec![MARK_NONE; triangle_count] }
    }

    pub fn triangle_count(&self) -> usize {
        self.marks.len()
    }

    pub fn mark(&self, index: usize) -> ZoneMark {
        match self.marks.get(index).copied() {
            Some(MARK_SUPPORT) => ZoneMark::Support,
            Some(MARK_BLOCK) => ZoneMark::Block,
            _ => ZoneMark::None,
        }
    }

    /// `(support faces, no-support faces)`.
    pub fn counts(&self) -> (usize, usize) {
        let support = self.marks.iter().filter(|m| **m == MARK_SUPPORT).count();
        let block = self.marks.iter().filter(|m| **m == MARK_BLOCK).count();
        (support, block)
    }

    pub fn is_empty(&self) -> bool {
        self.marks.iter().all(|m| *m == MARK_NONE)
    }

    pub fn clear(&mut self) {
        self.marks.iter_mut().for_each(|m| *m = MARK_NONE);
    }

    /// Fail when the zones were painted on a mesh with a different face count.
    pub fn check_mesh(&self, mesh: &Mesh) -> Result<()> {
        if self.marks.len() == mesh.triangles().len() {
            Ok(())
        } else {
            Err(ResinError::invalid(format!(
                "painted zones cover {} faces but the part has {}; they were painted on a different part",
                self.marks.len(),
                mesh.triangles().len()
            )))
        }
    }

    /// Mark every face with a vertex or its centre inside the sphere `center`, `radius_mm`.
    ///
    /// When `view_dir` is given, only faces that face the viewer (normal · view_dir < 0) are
    /// painted, so a stroke does not paint through to the back of a thin wall. Returns how many
    /// faces changed.
    pub fn paint(&mut self, mesh: &Mesh, center: Vec3, radius_mm: f64, mark: ZoneMark, view_dir: Option<Vec3>) -> Result<usize> {
        self.check_mesh(mesh)?;
        if !radius_mm.is_finite() || radius_mm <= 0.0 {
            return Err(ResinError::invalid("brush radius must be greater than 0 mm"));
        }
        if !(center.x.is_finite() && center.y.is_finite() && center.z.is_finite()) {
            return Err(ResinError::invalid("brush centre is not finite"));
        }
        let r2 = radius_mm * radius_mm;
        let value = mark_byte(mark);
        let mut changed = 0usize;
        for (tri, slot) in mesh.triangles().iter().zip(self.marks.iter_mut()) {
            if *slot == value {
                continue;
            }
            let centroid = tri.a.add(tri.b).add(tri.c).scale(1.0 / 3.0);
            let inside = [tri.a, tri.b, tri.c, centroid].iter().any(|p| p.sub(center).length_sq() <= r2);
            if !inside {
                continue;
            }
            if let Some(dir) = view_dir
                && let Some(normal) = tri.normal()
                && normal.dot(dir) >= 0.0
            {
                continue;
            }
            *slot = value;
            changed += 1;
        }
        Ok(changed)
    }

    /// The face a support touches, when it can be found at `contact_z` above `(x, y)`.
    fn contact_face(&self, mesh: &Mesh, grid: &VerticalHitGrid, point: &SupportPoint) -> Option<usize> {
        let (hit, index) = grid.lowest_hit_above(mesh, point.x, point.y, point.contact_z - 1e-4)?;
        if (hit.z - point.contact_z).abs() <= 1e-3 { Some(index) } else { None }
    }

    /// Read a zones file for a mesh with `triangle_count` faces.
    pub fn parse(text: &str, triangle_count: usize) -> Result<Self> {
        let file: ZonesFile = serde_json::from_str(text).map_err(|err| ResinError::invalid(format!("zones: {err}")))?;
        if file.schema != ZONES_SCHEMA {
            return Err(ResinError::invalid(format!("zones: schema must be `{ZONES_SCHEMA}`, not `{}`", file.schema)));
        }
        if file.triangle_count != triangle_count {
            return Err(ResinError::invalid(format!("zones file is for a part with {} faces; this part has {triangle_count}", file.triangle_count)));
        }
        let mut zones = Self::new(triangle_count);
        for (ranges, value) in [(&file.support, MARK_SUPPORT), (&file.block, MARK_BLOCK)] {
            for &[start, end] in ranges {
                if start >= end || end > triangle_count {
                    return Err(ResinError::invalid(format!("zones: face range {start}..{end} is outside 0..{triangle_count}")));
                }
                for slot in zones.marks.get_mut(start..end).into_iter().flatten() {
                    if *slot != MARK_NONE && *slot != value {
                        return Err(ResinError::invalid(format!("zones: face range {start}..{end} is both support and no-support")));
                    }
                    *slot = value;
                }
            }
        }
        Ok(zones)
    }

    /// Write a zones file that [`FaceZones::parse`] reads back.
    pub fn write(&self) -> Result<String> {
        let ranges = |value: u8| -> Vec<[usize; 2]> {
            let mut out = Vec::new();
            let mut start: Option<usize> = None;
            for (index, mark) in self.marks.iter().enumerate() {
                match (start, *mark == value) {
                    (None, true) => start = Some(index),
                    (Some(s), false) => {
                        out.push([s, index]);
                        start = None;
                    }
                    _ => {}
                }
            }
            if let Some(s) = start {
                out.push([s, self.marks.len()]);
            }
            out
        };
        let file = ZonesFile {
            schema: ZONES_SCHEMA.to_string(),
            triangle_count: self.marks.len(),
            support: ranges(MARK_SUPPORT),
            block: ranges(MARK_BLOCK),
        };
        serde_json::to_string_pretty(&file).map_err(|err| ResinError::invalid(format!("zones: {err}")))
    }
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ZonesFile {
    schema: String,
    triangle_count: usize,
    #[serde(default)]
    support: Vec<[usize; 2]>,
    #[serde(default)]
    block: Vec<[usize; 2]>,
}

/// What [`apply_zones`] did, for the status line and warnings.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ZoneReport {
    /// Supports added on painted support faces.
    pub painted_added: usize,
    /// Automatic supports removed because they touched a no-support face.
    pub blocked_removed: usize,
    /// Painted support faces that cannot take a vertical support (facing up or sideways, or with part below).
    pub painted_unreachable: usize,
}

/// Add supports on painted faces and drop automatic supports that touch no-support faces.
///
/// `mesh` is the lifted part. `zone_spacing_mm` is the XY spacing inside painted zones.
/// Automatic supports closer than that to a painted support are dropped so the zone sets the density.
pub fn apply_zones(
    mesh: &Mesh,
    zones: &FaceZones,
    points: Vec<SupportPoint>,
    zone_spacing_mm: f64,
    z_min: f64,
) -> Result<(Vec<SupportPoint>, ZoneReport)> {
    zones.check_mesh(mesh)?;
    if !zone_spacing_mm.is_finite() || zone_spacing_mm <= 0.0 {
        return Err(ResinError::invalid("painted zone spacing must be greater than 0 mm"));
    }
    let mut report = ZoneReport::default();
    if zones.is_empty() {
        return Ok((points, report));
    }
    let grid = VerticalHitGrid::build(mesh)?;
    let mut painted: Vec<SupportPoint> = Vec::new();
    for (index, tri) in mesh.triangles().iter().enumerate() {
        if zones.mark(index) != ZoneMark::Support {
            continue;
        }
        let facing_down = tri.normal().is_some_and(|n| n.z < -1e-9);
        if !facing_down {
            report.painted_unreachable += 1;
            continue;
        }
        let mut samples = Vec::new();
        sample_triangle(*tri, zone_spacing_mm, &mut samples, index)?;
        let mut reachable = false;
        for (p, source) in samples {
            if p.z <= z_min || grid.is_self_supported(mesh, p, Some(source)) {
                continue;
            }
            reachable = true;
            painted.push(SupportPoint { x: p.x, y: p.y, contact_z: p.z, source: PointSource::Painted, preset_index: None });
        }
        if !reachable {
            report.painted_unreachable += 1;
        }
        if painted.len() > MAX_SUPPORT_POINTS * 8 {
            return Err(ResinError::invalid("painted zones need too many supports; increase the painted zone spacing"));
        }
    }
    let painted = crate::supports::place::thin_points(&painted, zone_spacing_mm)?;
    let spacing_sq = zone_spacing_mm * zone_spacing_mm;
    let mut kept: Vec<SupportPoint> = Vec::with_capacity(points.len() + painted.len());
    for point in points {
        if point.source != PointSource::Manual {
            if let Some(face) = zones.contact_face(mesh, &grid, &point)
                && zones.mark(face) == ZoneMark::Block
            {
                report.blocked_removed += 1;
                continue;
            }
            let near_painted = painted.iter().any(|p| {
                let dx = p.x - point.x;
                let dy = p.y - point.y;
                dx * dx + dy * dy < spacing_sq
            });
            if near_painted {
                continue;
            }
        }
        kept.push(point);
    }
    report.painted_added = painted.len();
    kept.extend(painted);
    if kept.len() > MAX_SUPPORT_POINTS {
        return Err(ResinError::invalid(format!("too many supports: {} > {MAX_SUPPORT_POINTS}; increase the spacing", kept.len())));
    }
    Ok((kept, report))
}

/// Drop automatic points that touch a no-support face, without adding painted ones.
pub fn remove_blocked(mesh: &Mesh, zones: &FaceZones, points: &mut Vec<SupportPoint>) -> Result<usize> {
    zones.check_mesh(mesh)?;
    let (_, block) = zones.counts();
    if block == 0 {
        return Ok(0);
    }
    let grid = VerticalHitGrid::build(mesh)?;
    let before = points.len();
    points.retain(|point| {
        point.source == PointSource::Manual || zones.contact_face(mesh, &grid, point).is_none_or(|face| zones.mark(face) != ZoneMark::Block)
    });
    Ok(before - points.len())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use campanion_mesh::{Aabb, Triangle};

    use super::*;

    fn slab() -> Mesh {
        // A 10 x 10 x 2 mm slab floating from z = 5 to z = 7.
        Mesh::new(Aabb::from_corners(Vec3::new(0.0, 0.0, 5.0), Vec3::new(10.0, 10.0, 7.0)).face_triangles().to_vec()).unwrap()
    }

    fn bottom_faces(mesh: &Mesh) -> Vec<usize> {
        mesh.triangles().iter().enumerate().filter(|(_, t)| t.normal().is_some_and(|n| n.z < -0.5)).map(|(i, _)| i).collect()
    }

    #[test]
    fn painting_marks_faces_inside_the_brush_and_respects_facing() {
        let mesh = slab();
        let mut zones = FaceZones::new(mesh.triangles().len());
        // From above, the bottom faces face away and are not painted.
        let changed = zones.paint(&mesh, Vec3::new(5.0, 5.0, 5.0), 20.0, ZoneMark::Support, Some(Vec3::new(0.0, 0.0, -1.0))).unwrap();
        assert!(changed > 0);
        for face in bottom_faces(&mesh) {
            assert_eq!(zones.mark(face), ZoneMark::None);
        }
        // From below, they are.
        zones.paint(&mesh, Vec3::new(5.0, 5.0, 5.0), 20.0, ZoneMark::Block, Some(Vec3::new(0.0, 0.0, 1.0))).unwrap();
        for face in bottom_faces(&mesh) {
            assert_eq!(zones.mark(face), ZoneMark::Block);
        }
        assert!(zones.paint(&mesh, Vec3::new(5.0, 5.0, 5.0), 0.0, ZoneMark::Block, None).is_err());
        let other = Mesh::new(vec![Triangle { a: Vec3::new(0.0, 0.0, 0.0), b: Vec3::new(1.0, 0.0, 0.0), c: Vec3::new(0.0, 1.0, 0.0) }]).unwrap();
        assert!(zones.paint(&other, Vec3::new(0.0, 0.0, 0.0), 1.0, ZoneMark::Block, None).is_err());
    }

    #[test]
    fn zones_file_round_trips() {
        let mesh = slab();
        let mut zones = FaceZones::new(mesh.triangles().len());
        zones.paint(&mesh, Vec3::new(0.0, 0.0, 5.0), 3.0, ZoneMark::Support, None).unwrap();
        zones.paint(&mesh, Vec3::new(10.0, 10.0, 7.0), 3.0, ZoneMark::Block, None).unwrap();
        let text = zones.write().unwrap();
        let back = FaceZones::parse(&text, mesh.triangles().len()).unwrap();
        assert_eq!(back, zones);
        assert!(FaceZones::parse(&text, mesh.triangles().len() + 1).is_err());
        assert!(FaceZones::parse(r#"{"schema":"campanion-resin-zones/1","triangle_count":12,"support":[[0,13]]}"#, 12).is_err());
    }

    #[test]
    fn block_zones_remove_auto_supports_and_keep_manual_ones() {
        let mesh = slab();
        let mut zones = FaceZones::new(mesh.triangles().len());
        zones.paint(&mesh, Vec3::new(5.0, 5.0, 5.0), 20.0, ZoneMark::Block, Some(Vec3::new(0.0, 0.0, 1.0))).unwrap();
        let points = vec![
            SupportPoint { x: 2.0, y: 2.0, contact_z: 5.0, source: PointSource::Overhang, preset_index: None },
            SupportPoint { x: 4.0, y: 4.0, contact_z: 5.0, source: PointSource::Island, preset_index: None },
            SupportPoint { x: 6.0, y: 6.0, contact_z: 5.0, source: PointSource::Manual, preset_index: None },
        ];
        let (kept, report) = apply_zones(&mesh, &zones, points.clone(), 2.0, 1.0).unwrap();
        assert_eq!(report.blocked_removed, 2);
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].source, PointSource::Manual);
        let mut again = points;
        assert_eq!(remove_blocked(&mesh, &zones, &mut again).unwrap(), 2);
    }

    #[test]
    fn support_zones_add_supports_at_the_zone_spacing() {
        let mesh = slab();
        let mut zones = FaceZones::new(mesh.triangles().len());
        zones.paint(&mesh, Vec3::new(5.0, 5.0, 5.0), 20.0, ZoneMark::Support, None).unwrap();
        let (kept, report) = apply_zones(&mesh, &zones, Vec::new(), 2.5, 1.0).unwrap();
        assert!(report.painted_added >= 9, "{report:?}");
        assert_eq!(kept.len(), report.painted_added);
        assert!(kept.iter().all(|p| p.source == PointSource::Painted && (p.contact_z - 5.0).abs() < 1e-9));
        // Top and side faces were painted too but cannot take a vertical support.
        assert!(report.painted_unreachable > 0);
        for (i, a) in kept.iter().enumerate() {
            for b in kept.iter().skip(i + 1) {
                assert!((a.x - b.x).hypot(a.y - b.y) > 2.5 - 1e-9);
            }
        }
    }
}
