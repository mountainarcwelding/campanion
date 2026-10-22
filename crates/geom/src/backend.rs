//! Offset and boolean operations.
//!
//! This is the only module that names `cavalier_contours`. Callers see [`Contour`] and
//! [`BooleanOp`]. Swap the engine by implementing [`RegionEngine`].

use cavalier_contours::polyline::{
    BooleanOp as CavOp, BooleanResultInfo, PlineOffsetOptions, PlineSource, PlineSourceMut, Polyline as CavPolyline, TouchingLoopBehavior,
};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};

use crate::GeomError;
use crate::Result;
use crate::cleanup::{cleanup, orient_contours};
use crate::contour::Contour;
use crate::point::Point;
use crate::polyline::{Polyline, Vertex};
use crate::region::{Region, contours_to_regions};
use crate::tolerance::Tolerance;
use crate::util::{self, finite};

/// Upper bound on [`offset_rings`]. A hostile count must fail instead of looping.
const MAX_OFFSET_RINGS: usize = 256;

/// Boolean operation on two closed contours. `Difference` is `a` minus `b`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanOp {
    Union,
    Intersection,
    Difference,
    ExclusiveOr,
}

/// Swappable offset and boolean engine.
///
/// Positive `distance` is to the left of travel. On a counter-clockwise contour that is inward;
/// on a clockwise hole it is outward (the hole grows, so the filled region shrinks). Joins are
/// round: a convex corner becomes a circular arc. Self-intersections in the offset are removed
/// and the pieces come back as separate contours.
pub trait RegionEngine {
    fn offset(&self, contour: &Contour, distance: f64, tolerance: &Tolerance) -> Result<Vec<Contour>>;

    fn boolean(&self, op: BooleanOp, a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<Vec<Contour>>;

    /// Offset every loop of a region together so an outer and its islands stay consistent.
    fn offset_region(&self, region: &Region, distance: f64, tolerance: &Tolerance) -> Result<Vec<Region>>;
}

/// [`RegionEngine`] backed by `cavalier_contours` 0.9.
///
/// Positive `distance` offsets to the left of travel (inward on a counter-clockwise contour).
#[derive(Clone, Copy, Debug, Default)]
pub struct CavalierEngine;

impl RegionEngine for CavalierEngine {
    fn offset(&self, contour: &Contour, distance: f64, tolerance: &Tolerance) -> Result<Vec<Contour>> {
        let distance = finite(distance, "offset distance")?;
        util::check_segment_count(contour.segments().len())?;
        let pline = to_cavalier(contour)?;
        let options = offset_options();
        let raw = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| pline.parallel_offset_opt(distance, &options)));
        let plines = raw.map_err(|_| GeomError::InvalidInput("offset backend failed".into()))?;
        finish(plines, tolerance)
    }

    fn boolean(&self, op: BooleanOp, a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<Vec<Contour>> {
        if !a.is_closed() || !b.is_closed() {
            return Err(GeomError::InvalidInput("boolean requires closed contours".into()));
        }
        util::check_segment_count(a.segments().len().saturating_add(b.segments().len()))?;
        let left = to_cavalier(a)?;
        let right = to_cavalier(b)?;
        let cav = match op {
            BooleanOp::Union => CavOp::Or,
            BooleanOp::Intersection => CavOp::And,
            BooleanOp::Difference => CavOp::Not,
            BooleanOp::ExclusiveOr => CavOp::Xor,
        };
        let raw = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| left.boolean(&right, cav)));
        let result = raw.map_err(|_| GeomError::InvalidInput("boolean backend failed".into()))?;
        if matches!(result.result_info, BooleanResultInfo::InvalidInput) {
            return Err(GeomError::InvalidInput("boolean backend rejected the contours".into()));
        }
        let mut plines = Vec::new();
        for item in result.pos_plines {
            plines.push(item.pline);
        }
        for item in result.neg_plines {
            plines.push(item.pline);
        }
        finish(plines, tolerance)
    }

    fn offset_region(&self, region: &Region, distance: f64, tolerance: &Tolerance) -> Result<Vec<Region>> {
        let distance = finite(distance, "offset distance")?;
        let mut count = region.outer().segments().len();
        for hole in region.holes() {
            count = count.saturating_add(hole.segments().len());
        }
        util::check_segment_count(count)?;
        let mut plines = Vec::new();
        plines.push(to_cavalier(region.outer())?);
        for hole in region.holes() {
            plines.push(to_cavalier(hole)?);
        }
        let options = ShapeOffsetOptions::<f64>::default();
        let raw = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let shape = Shape::from_plines(plines);
            shape.parallel_offset(distance, &options)
        }));
        let shape = raw.map_err(|_| GeomError::InvalidInput("offset backend failed".into()))?;
        let mut contours = Vec::new();
        for indexed in shape.ccw_plines.iter().chain(shape.cw_plines.iter()) {
            contours.extend(convert_preserving_winding(&indexed.polyline, tolerance)?);
        }
        contours_to_regions(&contours, tolerance)
    }
}

impl CavalierEngine {
    pub fn new() -> Self {
        Self
    }
}

/// Parallel offset. Positive distance is to the left of travel.
pub fn offset(contour: &Contour, distance: f64, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    CavalierEngine::new().offset(contour, distance, tolerance)
}

/// Boolean of two closed contours.
pub fn boolean(op: BooleanOp, a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    CavalierEngine::new().boolean(op, a, b, tolerance)
}

/// Offset every loop of a region. Positive distance shrinks a counter-clockwise outer and grows
/// clockwise islands. The region disappears when the offset eats it, and the result may split.
pub fn offset_region(region: &Region, distance: f64, tolerance: &Tolerance) -> Result<Vec<Region>> {
    CavalierEngine::new().offset_region(region, distance, tolerance)
}

/// Successive parallel offsets by `step`.
///
/// Ring *k* (0-based) is the previous ring offset by one more `step`. A ring that comes back
/// empty is not stored, and later rings are not attempted. `count` is an upper bound.
pub fn offset_rings(contour: &Contour, step: f64, count: usize, tolerance: &Tolerance) -> Result<Vec<Vec<Contour>>> {
    if count > MAX_OFFSET_RINGS {
        return Err(GeomError::InvalidInput(format!("too many offset rings: {count} > {MAX_OFFSET_RINGS}")));
    }
    let step = finite(step, "offset step")?;
    if step == 0.0 {
        return Err(GeomError::InvalidInput("offset step must be non-zero".into()));
    }
    let mut rings = Vec::new();
    let mut current = vec![contour.clone()];
    for _ in 0..count {
        let mut next = Vec::new();
        for piece in &current {
            next.extend(offset(piece, step, tolerance)?);
        }
        if next.is_empty() {
            break;
        }
        rings.push(next.clone());
        current = next;
    }
    Ok(rings)
}

fn offset_options() -> PlineOffsetOptions<'static, f64> {
    PlineOffsetOptions { handle_self_intersects: true, touching_loop_behavior: TouchingLoopBehavior::Separate, ..PlineOffsetOptions::default() }
}

/// `cleanup` orients a contour on its own, which would turn a hole into an outer. Put the sign
/// back so nesting can still tell islands from the boundary that contains them.
fn convert_preserving_winding(pline: &CavPolyline<f64>, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    let source_positive = pline.area() >= 0.0;
    let Some(contour) = from_cavalier(pline, tolerance)? else {
        return Ok(Vec::new());
    };
    let mut cleaned = cleanup(contour.segments(), tolerance)?;
    for contour in &mut cleaned {
        if !contour.is_closed() {
            continue;
        }
        let Ok(area) = contour.signed_area() else {
            continue;
        };
        if (area >= 0.0) != source_positive {
            *contour = contour.reversed();
        }
    }
    Ok(cleaned)
}

fn finish(plines: Vec<CavPolyline<f64>>, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    let mut contours = Vec::new();
    for pline in &plines {
        let Some(contour) = from_cavalier(pline, tolerance)? else {
            continue;
        };
        // Clean each result on its own so a hole is not joined to its outer.
        let cleaned = cleanup(contour.segments(), tolerance)?;
        contours.extend(cleaned);
    }
    orient_contours(&mut contours, tolerance)?;
    Ok(contours)
}

fn to_cavalier(contour: &Contour) -> Result<CavPolyline<f64>> {
    let poly = contour.to_polyline()?;
    let mut pline = if poly.is_closed() { CavPolyline::new_closed() } else { CavPolyline::new() };
    for vertex in poly.vertices() {
        pline.add(vertex.point().x(), vertex.point().y(), vertex.bulge());
    }
    Ok(pline)
}

fn from_cavalier(pline: &CavPolyline<f64>, tolerance: &Tolerance) -> Result<Option<Contour>> {
    let count = pline.vertex_count();
    if count < 2 {
        return Ok(None);
    }
    util::check_segment_count(count)?;
    let mut vertices = Vec::with_capacity(count);
    for index in 0..count {
        let Some(vertex) = pline.get(index) else {
            return Err(GeomError::InvalidInput("backend vertex missing".into()));
        };
        let point = Point::new(vertex.x, vertex.y)?;
        vertices.push(Vertex::new(point, vertex.bulge)?);
    }
    let poly = Polyline::new(vertices, pline.is_closed())?;
    match poly.to_contour(tolerance) {
        Ok(contour) => Ok(Some(contour)),
        Err(_) => {
            // The backend's own epsilon is coarser than 1e-6 mm.
            let loose = Tolerance::with_chord(1e-4, 1e-4, 1e-4)?;
            Ok(Some(poly.to_contour(&loose)?))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::line::Line;
    use crate::point::Point;
    use crate::polyline::{Polyline, Vertex};
    use crate::segment::Segment;
    use std::f64::consts::PI;

    fn circle(r: f64) -> Contour {
        Polyline::new(vec![Vertex::new(Point::new(r, 0.0).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(-r, 0.0).unwrap(), 1.0).unwrap()], true)
            .unwrap()
            .to_contour(&Tolerance::default())
            .unwrap()
    }

    fn rect(w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let line = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![line(p(0.0, 0.0), p(w, 0.0)), line(p(w, 0.0), p(w, h)), line(p(w, h), p(0.0, h)), line(p(0.0, h), p(0.0, 0.0))],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    #[test]
    fn circle_offset_left_and_right_match_analytic_radii() {
        let tol = Tolerance::default();
        let shape = circle(10.0);
        let inward = offset(&shape, 1.0, &tol).unwrap();
        assert_eq!(inward.len(), 1);
        assert!((inward[0].signed_area().unwrap() - 81.0 * PI).abs() < 1e-6, "{}", inward[0].signed_area().unwrap());
        let outward = offset(&shape, -1.0, &tol).unwrap();
        assert_eq!(outward.len(), 1);
        assert!((outward[0].signed_area().unwrap() - 121.0 * PI).abs() < 1e-6, "{}", outward[0].signed_area().unwrap());
    }

    #[test]
    fn rectangle_minus_circle_has_signed_areas() {
        let tol = Tolerance::default();
        let board = rect(4.0, 4.0);
        // Circle radius 1 centred at (2, 2).
        let hole = Polyline::new(
            vec![Vertex::new(Point::new(3.0, 2.0).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(1.0, 2.0).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap()
        .to_contour(&tol)
        .unwrap();
        let parts = boolean(BooleanOp::Difference, &board, &hole, &tol).unwrap();
        assert_eq!(parts.len(), 2, "got {}", parts.len());
        let sum: f64 = parts.iter().map(|c| c.signed_area().unwrap()).sum();
        assert!((sum - (16.0 - PI)).abs() < 1e-6, "sum {sum}");
        let mut areas: Vec<f64> = parts.iter().map(|c| c.signed_area().unwrap()).collect();
        areas.sort_by(|a, b| a.total_cmp(b));
        assert!(areas[0] < 0.0, "hole should be clockwise: {areas:?}");
        assert!(areas[1] > 0.0, "outer should be counter-clockwise: {areas:?}");
    }

    #[test]
    fn rejects_non_finite_offset_and_open_boolean() {
        let shape = rect(2.0, 2.0);
        assert!(offset(&shape, f64::NAN, &Tolerance::default()).is_err());
        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        assert!(boolean(BooleanOp::Union, &shape, &open, &Tolerance::default()).is_err());
    }

    /// Circles and axis-aligned edges meet 1e-6 mm. Round joins on a rectangle are checked at
    /// 1e-4 mm² because the backend's `offset_dist_eps` is 1e-4.
    const TIGHT: f64 = 1e-6;
    const LOOSE: f64 = 1e-4;

    fn area_of(contour: &Contour) -> f64 {
        contour.signed_area().expect("area")
    }

    fn arc_radii(contour: &Contour) -> Vec<f64> {
        contour.segments().iter().filter_map(|seg| if let Segment::Arc(arc) = seg { Some(arc.radius()) } else { None }).collect()
    }

    fn slot() -> Contour {
        // Stadium: straight run 20, radius 5. Area = 20 * 10 + π * 25.
        let line = |x0, y0, x1, y1| Segment::Line(Line::new(Point::new(x0, y0).unwrap(), Point::new(x1, y1).unwrap()).unwrap());
        let cap = |x0, y0, x1, y1| Segment::Arc(crate::Arc::from_bulge(Point::new(x0, y0).unwrap(), Point::new(x1, y1).unwrap(), 1.0).unwrap());
        Contour::try_new(
            vec![line(10.0, 5.0, -10.0, 5.0), cap(-10.0, 5.0, -10.0, -5.0), line(-10.0, -5.0, 10.0, -5.0), cap(10.0, -5.0, 10.0, 5.0)],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    #[test]
    fn circle_offset_grows_and_shrinks_the_radius() {
        let tol = Tolerance::default();
        let shape = circle(10.0);
        // Positive distance is to the left, inward on this counter-clockwise circle.
        let grown = offset(&shape, -2.5, &tol).unwrap();
        assert_eq!(grown.len(), 1);
        assert!((area_of(&grown[0]) - 12.5 * 12.5 * PI).abs() < TIGHT, "{}", area_of(&grown[0]));
        let radii = arc_radii(&grown[0]);
        assert!(!radii.is_empty());
        assert!(radii.iter().all(|r| (*r - 12.5).abs() < TIGHT), "{radii:?}");
        let shrunk = offset(&shape, 2.5, &tol).unwrap();
        assert_eq!(shrunk.len(), 1);
        assert!((area_of(&shrunk[0]) - 7.5 * 7.5 * PI).abs() < TIGHT, "{}", area_of(&shrunk[0]));
        let radii = arc_radii(&shrunk[0]);
        assert!(radii.iter().all(|r| (*r - 7.5).abs() < TIGHT), "{radii:?}");
    }

    #[test]
    fn rectangle_outward_offset_area_includes_round_corners() {
        let tol = Tolerance::default();
        // 10 by 6, outward by 1. Area = wh + 2d(w+h) + π d² = 60 + 32 + π.
        let shape = rect(10.0, 6.0);
        let grown = offset(&shape, -1.0, &tol).unwrap();
        assert_eq!(grown.len(), 1);
        let area = area_of(&grown[0]);
        assert!((area - (92.0 + PI)).abs() < LOOSE, "area {area}");
        let radii = arc_radii(&grown[0]);
        assert_eq!(radii.len(), 4, "round corners: {radii:?}");
        assert!(radii.iter().all(|r| (*r - 1.0).abs() < LOOSE), "{radii:?}");
    }

    #[test]
    fn rectangle_inward_offset_is_the_shrunk_rectangle() {
        let tol = Tolerance::default();
        // Inward by 1. Area (10−2)(6−2) = 32. Convex inward offsets meet at sharp corners.
        let shape = rect(10.0, 6.0);
        let shrunk = offset(&shape, 1.0, &tol).unwrap();
        assert_eq!(shrunk.len(), 1);
        let area = area_of(&shrunk[0]);
        assert!((area - 32.0).abs() < TIGHT, "area {area}");
        assert!(arc_radii(&shrunk[0]).is_empty(), "no corner arcs");
    }

    #[test]
    fn slot_offset_changes_the_cap_radius() {
        let tol = Tolerance::default();
        let shape = slot();
        // Outward by 1: run stays 20, radius 6. Area = 20*12 + π*36.
        let grown = offset(&shape, -1.0, &tol).unwrap();
        assert_eq!(grown.len(), 1);
        let area = area_of(&grown[0]);
        assert!((area - (240.0 + 36.0 * PI)).abs() < LOOSE, "grown {area}");
        let radii = arc_radii(&grown[0]);
        assert!(!radii.is_empty());
        assert!(radii.iter().all(|r| (*r - 6.0).abs() < LOOSE), "{radii:?}");
        // Inward by 1: radius 4. Area = 20*8 + π*16.
        let shrunk = offset(&shape, 1.0, &tol).unwrap();
        assert_eq!(shrunk.len(), 1);
        let area = area_of(&shrunk[0]);
        assert!((area - (160.0 + 16.0 * PI)).abs() < LOOSE, "shrunk {area}");
    }

    #[test]
    fn offset_larger_than_the_feature_vanishes() {
        let tol = Tolerance::default();
        assert!(offset(&circle(2.0), 2.0, &tol).unwrap().is_empty());
        assert!(offset(&circle(2.0), 3.0, &tol).unwrap().is_empty());
        // Half the short side is 3. A distance past that leaves no rectangle. Exactly 3 can
        // remain as a zero-area span, so the vanishing case is the one that is strictly larger.
        assert!(offset(&rect(10.0, 6.0), 4.0, &tol).unwrap().is_empty());
        assert!(offset(&slot(), 6.0, &tol).unwrap().is_empty());
    }

    #[test]
    fn open_line_offsets_to_the_left_and_a_corner_is_round() {
        let tol = Tolerance::default();
        let line =
            Contour::try_new(vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(10.0, 0.0).unwrap()).unwrap())], false, &tol)
                .unwrap();
        let shifted = offset(&line, 1.0, &tol).unwrap();
        assert_eq!(shifted.len(), 1);
        assert!(!shifted[0].is_closed());
        assert!((shifted[0].length().unwrap() - 10.0).abs() < LOOSE, "len {}", shifted[0].length().unwrap());
        for seg in shifted[0].segments() {
            assert!((seg.start().y() - 1.0).abs() < LOOSE, "{}", seg.start().y());
            assert!((seg.end().y() - 1.0).abs() < LOOSE, "{}", seg.end().y());
        }

        // Left turn. Negative distance is the exterior, so the corner is a quarter-circle.
        let elbow = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(10.0, 0.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(10.0, 0.0).unwrap(), Point::new(10.0, 10.0).unwrap()).unwrap()),
            ],
            false,
            &tol,
        )
        .unwrap();
        let rounded = offset(&elbow, -1.0, &tol).unwrap();
        assert_eq!(rounded.len(), 1);
        let length = rounded[0].length().unwrap();
        assert!((length - (20.0 + PI / 2.0)).abs() < LOOSE, "length {length}");
        let radii = arc_radii(&rounded[0]);
        assert_eq!(radii.len(), 1, "{radii:?}");
        assert!((radii[0] - 1.0).abs() < LOOSE);
    }

    #[test]
    fn inward_offset_of_a_barbell_splits_when_the_bridge_closes() {
        let tol = Tolerance::default();
        // Two 6 by 6 squares joined by a bridge of height 2 (y = 2..4, x = 6..10).
        // Inward by 1.1 is more than half the bridge, so the bridge disappears.
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |x0, y0, x1, y1| Segment::Line(Line::new(p(x0, y0), p(x1, y1)).unwrap());
        let barbell = Contour::try_new(
            vec![
                edge(0.0, 0.0, 6.0, 0.0),
                edge(6.0, 0.0, 6.0, 2.0),
                edge(6.0, 2.0, 10.0, 2.0),
                edge(10.0, 2.0, 10.0, 0.0),
                edge(10.0, 0.0, 16.0, 0.0),
                edge(16.0, 0.0, 16.0, 6.0),
                edge(16.0, 6.0, 10.0, 6.0),
                edge(10.0, 6.0, 10.0, 4.0),
                edge(10.0, 4.0, 6.0, 4.0),
                edge(6.0, 4.0, 6.0, 6.0),
                edge(6.0, 6.0, 0.0, 6.0),
                edge(0.0, 6.0, 0.0, 0.0),
            ],
            true,
            &tol,
        )
        .unwrap();
        let parts = offset(&barbell, 1.1, &tol).unwrap();
        assert_eq!(parts.len(), 2, "got {}", parts.len());
        for part in &parts {
            assert!(part.is_closed());
            assert!(area_of(part) > 1.0, "{}", area_of(part));
            let bridge = Point::new(8.0, 3.0).unwrap();
            assert_eq!(part.containment(bridge, &tol).unwrap(), crate::Containment::Outside);
        }
    }

    #[test]
    fn offset_rings_stop_when_the_circle_vanishes() {
        let tol = Tolerance::default();
        // Radius 5, inward step 1. Rings at radius 4, 3, 2, 1, then nothing.
        let rings = offset_rings(&circle(5.0), 1.0, 10, &tol).unwrap();
        assert_eq!(rings.len(), 4, "got {}", rings.len());
        for (index, ring) in rings.iter().enumerate() {
            assert_eq!(ring.len(), 1);
            let radius = 4.0 - index as f64;
            let area = area_of(&ring[0]);
            assert!((area - radius * radius * PI).abs() < TIGHT, "ring {index} area {area}");
        }
        let square = offset_rings(&rect(10.0, 10.0), 2.0, 10, &tol).unwrap();
        assert_eq!(square.len(), 2, "got {}", square.len());
        assert!((area_of(&square[0][0]) - 36.0).abs() < TIGHT, "{}", area_of(&square[0][0]));
        assert!((area_of(&square[1][0]) - 4.0).abs() < TIGHT, "{}", area_of(&square[1][0]));
        assert!(offset_rings(&rect(4.0, 4.0), 0.0, 2, &tol).is_err());
        assert!(offset_rings(&rect(4.0, 4.0), 1.0, MAX_OFFSET_RINGS + 1, &tol).is_err());
    }

    #[test]
    fn region_offset_moves_the_outer_and_the_island() {
        let tol = Tolerance::default();
        let outer = rect(20.0, 20.0);
        let island = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(7.0, 7.0).unwrap(), Point::new(13.0, 7.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(13.0, 7.0).unwrap(), Point::new(13.0, 13.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(13.0, 13.0).unwrap(), Point::new(7.0, 13.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(7.0, 13.0).unwrap(), Point::new(7.0, 7.0).unwrap()).unwrap()),
            ],
            true,
            &tol,
        )
        .unwrap();
        let region = Region::try_new(outer, vec![island], &tol).unwrap();
        assert!((region.area().unwrap() - (400.0 - 36.0)).abs() < TIGHT);
        // Inward 1: outer shrinks to a sharp 18×18. The island is a rectangle offset outward,
        // so it gains round corners: 36 + 2*1*(6+6) + π = 60 + π. Area = 324 − (60 + π).
        let shrunk = offset_region(&region, 1.0, &tol).unwrap();
        assert_eq!(shrunk.len(), 1, "got {}", shrunk.len());
        assert_eq!(shrunk[0].holes().len(), 1);
        let area = shrunk[0].area().unwrap();
        assert!((area - (264.0 - PI)).abs() < TIGHT, "inward {area}");
        // Outward 1: outer area 400 + 2*40 + π, island shrinks to 4×4.
        let grown = offset_region(&region, -1.0, &tol).unwrap();
        assert_eq!(grown.len(), 1, "got {}", grown.len());
        assert_eq!(grown[0].holes().len(), 1);
        let area = grown[0].area().unwrap();
        assert!((area - (464.0 + PI)).abs() < LOOSE, "outward {area}");
    }
}
