//! Property tests: area sign, cleanup idempotence, offset and boolean laws, hostile numbers.

use crate::{BooleanOp, Contour, Line, Point, Region, Segment, Tolerance, Vec2, boolean_regions, cleanup, offset, offset_region, offset_rings};
use proptest::prelude::*;

fn point(x: f64, y: f64) -> Option<Point> {
    Point::new(x, y).ok()
}

fn triangle(x0: f64, y0: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> Option<Contour> {
    let a = point(x0, y0)?;
    let b = point(x1, y1)?;
    let c = point(x2, y2)?;
    let edge = |p: Point, q: Point| Line::new(p, q).ok().map(Segment::Line);
    let segments = vec![edge(a, b)?, edge(b, c)?, edge(c, a)?];
    if segments.iter().any(|seg| seg.length() < 1e-2) {
        return None;
    }
    Contour::try_new(segments, true, &Tolerance::default()).ok()
}

fn rect_region(x: f64, y: f64, w: f64, h: f64) -> Option<Region> {
    if !(w.is_finite() && h.is_finite() && w > 0.5 && h > 0.5) {
        return None;
    }
    let a = point(x, y)?;
    let b = point(x + w, y)?;
    let c = point(x + w, y + h)?;
    let d = point(x, y + h)?;
    let edge = |p: Point, q: Point| Line::new(p, q).ok().map(Segment::Line);
    let segments = vec![edge(a, b)?, edge(b, c)?, edge(c, d)?, edge(d, a)?];
    let contour = Contour::try_new(segments, true, &Tolerance::default()).ok()?;
    Region::from_contour(contour, &Tolerance::default()).ok()
}

fn overlap_area(ax: f64, ay: f64, aw: f64, ah: f64, bx: f64, by: f64, bw: f64, bh: f64) -> f64 {
    let w = (ax + aw).min(bx + bw) - ax.max(bx);
    let h = (ay + ah).min(by + bh) - ay.max(by);
    w.max(0.0) * h.max(0.0)
}

fn region_areas(regions: &[Region]) -> f64 {
    regions.iter().map(|region| region.area().unwrap_or(0.0)).sum()
}

fn contours_match(a: &Contour, b: &Contour, tol: &Tolerance) -> bool {
    if a.is_closed() != b.is_closed() || a.segments().len() != b.segments().len() {
        return false;
    }
    a.segments().iter().zip(b.segments().iter()).all(|(left, right)| segments_match(*left, *right, tol))
}

fn segments_match(a: Segment, b: Segment, tol: &Tolerance) -> bool {
    match (a, b) {
        (Segment::Line(a), Segment::Line(b)) => a.start().almost_eq(b.start(), tol.linear()) && a.end().almost_eq(b.end(), tol.linear()),
        (Segment::Arc(a), Segment::Arc(b)) => {
            a.center().almost_eq(b.center(), tol.linear())
                && (a.radius() - b.radius()).abs() <= tol.linear()
                && (a.sweep() - b.sweep()).abs() <= tol.angular().max(1e-6)
                && crate::util::angle_diff(a.start_angle(), b.start_angle()).abs() <= tol.angular().max(1e-6)
        }
        _ => false,
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(80))]

    #[test]
    fn area_sign_flips_on_reverse(
        x0 in -100.0..100.0,
        y0 in -100.0..100.0,
        x1 in -100.0..100.0,
        y1 in -100.0..100.0,
        x2 in -100.0..100.0,
        y2 in -100.0..100.0,
    ) {
        let Some(shape) = triangle(x0, y0, x1, y1, x2, y2) else {
            return Ok(());
        };
        let area = shape.signed_area().expect("area");
        prop_assume!(area.abs() > 1e-2);
        let flipped = shape.reversed().signed_area().expect("reversed area");
        prop_assert!((area + flipped).abs() <= 1e-6 * (1.0 + area.abs()));
        let restored = shape.reversed().reversed().signed_area().expect("restored");
        prop_assert!((area - restored).abs() <= 1e-6 * (1.0 + area.abs()));

        let cleaned = cleanup(shape.segments(), &Tolerance::default()).expect("cleanup");
        prop_assert_eq!(cleaned.len(), 1);
        let cleaned_area = cleaned[0].signed_area().expect("cleaned area");
        prop_assert!(cleaned_area > 0.0);
        prop_assert!((cleaned_area.abs() - area.abs()).abs() <= 1e-4);
    }

    #[test]
    fn cleanup_is_idempotent(
        coords in proptest::collection::vec((-50.0..50.0, -50.0..50.0, -50.0..50.0, -50.0..50.0), 0..8)
    ) {
        let tol = Tolerance::default();
        let mut segs = Vec::new();
        for (x0, y0, x1, y1) in coords {
            let (Some(a), Some(b)) = (point(x0, y0), point(x1, y1)) else {
                continue;
            };
            if a.distance(b) < 1e-3 {
                continue;
            }
            if let Ok(line) = Line::new(a, b) {
                segs.push(Segment::Line(line));
            }
        }
        let once = match cleanup(&segs, &tol) {
            Ok(contours) => contours,
            Err(_) => return Ok(()),
        };
        let flat: Vec<Segment> = once.iter().flat_map(|c| c.segments().iter().copied()).collect();
        let twice = cleanup(&flat, &tol).expect("second cleanup");
        prop_assert_eq!(once.len(), twice.len());
        for (left, right) in once.iter().zip(twice.iter()) {
            prop_assert!(contours_match(left, right, &tol));
        }
    }

    #[test]
    fn hostile_numbers_do_not_panic(values in proptest::collection::vec(any::<f64>(), 0..24)) {
        let _ = Point::new(values.first().copied().unwrap_or(0.0), values.get(1).copied().unwrap_or(0.0));
        let _ = Vec2::new(values.get(2).copied().unwrap_or(0.0), values.get(3).copied().unwrap_or(0.0));
        let _ = Tolerance::new(values.get(4).copied().unwrap_or(0.0), values.get(5).copied().unwrap_or(0.0));
        let origin = Point::new(0.0, 0.0).expect("origin");
        let _ = crate::Arc::new(
            origin,
            values.get(6).copied().unwrap_or(1.0),
            values.get(7).copied().unwrap_or(0.0),
            values.get(8).copied().unwrap_or(1.0),
        );
        if let (Some(a), Some(b)) = (values.get(9).copied(), values.get(10).copied())
            && let (Ok(start), Ok(end)) = (Point::new(a, b), Point::new(values.get(11).copied().unwrap_or(0.0), values.get(12).copied().unwrap_or(0.0))) {
                let _ = crate::Arc::from_bulge(start, end, values.get(13).copied().unwrap_or(0.0));
            }
        let mut segs = Vec::new();
        let mut i = 0usize;
        while i + 3 < values.len() {
            let nums = [
                values.get(i).copied().unwrap_or(0.0),
                values.get(i + 1).copied().unwrap_or(0.0),
                values.get(i + 2).copied().unwrap_or(0.0),
                values.get(i + 3).copied().unwrap_or(0.0),
            ];
            i += 4;
            if let (Ok(a), Ok(b)) = (Point::new(nums[0], nums[1]), Point::new(nums[2], nums[3]))
                && let Ok(line) = Line::new(a, b) {
                    segs.push(Segment::Line(line));
                }
        }
        let tol = Tolerance::default();
        let _ = cleanup(&segs, &tol);
        let square = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(1.0, 0.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(1.0, 0.0).unwrap(), Point::new(1.0, 1.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(1.0, 1.0).unwrap(), Point::new(0.0, 1.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(0.0, 1.0).unwrap(), Point::new(0.0, 0.0).unwrap()).unwrap()),
            ],
            true,
            &tol,
        )
        .expect("square");
        let distance = values.first().copied().unwrap_or(0.0);
        let _ = offset(&square, distance, &tol);
        let _ = crate::boolean(BooleanOp::Union, &square, &square, &tol);
        let count = values.len() % 4;
        let _ = offset_rings(&square, distance, count, &tol);
        let _ = offset_rings(&square, f64::NAN, 1, &tol);
        let _ = offset_rings(&square, 0.0, 1, &tol);
        let _ = offset_rings(&square, 1.0, 10_000, &tol);
        if let Ok(region) = Region::from_contour(square.clone(), &tol) {
            let _ = offset_region(&region, distance, &tol);
            let _ = boolean_regions(BooleanOp::Union, &region, &region, &tol);
            let _ = boolean_regions(BooleanOp::Difference, &region, &region, &tol);
            let _ = boolean_regions(BooleanOp::Intersection, &region, &region, &tol);
            let _ = boolean_regions(BooleanOp::ExclusiveOr, &region, &region, &tol);
        }
        // A plate with an island, offset and combined with the same hostile distance.
        let plate = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(10.0, 0.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(10.0, 0.0).unwrap(), Point::new(10.0, 10.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(10.0, 10.0).unwrap(), Point::new(0.0, 10.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(0.0, 10.0).unwrap(), Point::new(0.0, 0.0).unwrap()).unwrap()),
            ],
            true,
            &tol,
        );
        let island = Contour::try_new(
            vec![
                Segment::Line(Line::new(Point::new(4.0, 4.0).unwrap(), Point::new(6.0, 4.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(6.0, 4.0).unwrap(), Point::new(6.0, 6.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(6.0, 6.0).unwrap(), Point::new(4.0, 6.0).unwrap()).unwrap()),
                Segment::Line(Line::new(Point::new(4.0, 6.0).unwrap(), Point::new(4.0, 4.0).unwrap()).unwrap()),
            ],
            true,
            &tol,
        );
        if let (Ok(plate), Ok(island)) = (plate, island)
            && let Ok(region) = Region::try_new(plate, vec![island], &tol)
        {
            let _ = offset_region(&region, distance, &tol);
            if let Ok(tool) = Region::from_contour(square, &tol) {
                let _ = boolean_regions(BooleanOp::Difference, &region, &tool, &tol);
                let _ = boolean_regions(BooleanOp::Union, &region, &tool, &tol);
            }
        }
    }

    #[test]
    fn offset_out_then_in_keeps_the_area(
        x in -30.0..30.0,
        y in -30.0..30.0,
        w in 2.0..20.0,
        h in 2.0..20.0,
        d in 0.05..0.8,
    ) {
        let Some(region) = rect_region(x, y, w, h) else {
            return Ok(());
        };
        let shape = region.outer().clone();
        let tol = Tolerance::default();
        let distance: f64 = d;
        // Outward then inward. Round joins of radius `distance` collapse, so the rectangle returns.
        // `offset_dist_eps` in the backend is 1e-4; the area window is looser than 1e-6 mm².
        let grown = match offset(&shape, -distance, &tol) {
            Ok(contours) => contours,
            Err(_) => return Ok(()),
        };
        prop_assume!(grown.len() == 1);
        let back = match offset(&grown[0], d, &tol) {
            Ok(contours) => contours,
            Err(_) => return Ok(()),
        };
        prop_assume!(back.len() == 1);
        let before = shape.signed_area().expect("area");
        let after = back[0].signed_area().expect("area");
        let eps = 1e-3 * (1.0 + before.abs());
        prop_assert!((before - after).abs() <= eps, "before {before} after {after} d {distance}");
    }

    #[test]
    fn rectangle_boolean_respects_area_laws(
        ax in -20.0..20.0,
        ay in -20.0..20.0,
        aw in 1.0..15.0,
        ah in 1.0..15.0,
        bx in -20.0..20.0,
        by in -20.0..20.0,
        bw in 1.0..15.0,
        bh in 1.0..15.0,
    ) {
        let Some(a) = rect_region(ax, ay, aw, ah) else {
            return Ok(());
        };
        let Some(b) = rect_region(bx, by, bw, bh) else {
            return Ok(());
        };
        let tol = Tolerance::default();
        let union = boolean_regions(BooleanOp::Union, &a, &b, &tol).expect("union");
        let inter = boolean_regions(BooleanOp::Intersection, &a, &b, &tol).expect("intersection");
        let diff = boolean_regions(BooleanOp::Difference, &a, &b, &tol).expect("difference");
        let aa = a.area().expect("a");
        let bb = b.area().expect("b");
        let ua = region_areas(&union);
        let ia = region_areas(&inter);
        let da = region_areas(&diff);
        let scale = aa.max(bb).max(1.0);
        // Axis-aligned edges usually hit 1e-6. The bound stays at 1e-4 mm² for stitched vertices.
        let eps = (1e-4 * scale).max(1e-4);
        prop_assert!(ua + eps >= aa.max(bb), "union {ua} max {}", aa.max(bb));
        prop_assert!((aa - (da + ia)).abs() <= eps, "A {aa} (A-B) {da} (A∩B) {ia}");
        let overlap = overlap_area(ax, ay, aw, ah, bx, by, bw, bh);
        prop_assert!((ia - overlap).abs() <= eps, "intersection {ia} expected {overlap}");
        prop_assert!((ua - (aa + bb - overlap)).abs() <= eps, "union {ua}");
        prop_assert!((da - (aa - overlap)).abs() <= eps, "difference {da}");
    }
}
