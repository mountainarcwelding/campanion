//! Join loose segments, drop junk, merge runs, and fix winding.

use std::collections::VecDeque;

use crate::GeomError;
use crate::Result;
use crate::arc::Arc;
use crate::contour::{Contour, Orientation};
use crate::line::Line;
use crate::point::{Point, Vec2};
use crate::segment::Segment;
use crate::tolerance::Tolerance;
use crate::util::{self, TAU, angle_diff};

/// Join loose segments into contours and clean them.
///
/// Drops segments shorter than the linear tolerance and exact duplicates (either direction).
/// Chains ends that lie within the linear tolerance, including a small gap that closes a loop.
/// Merges collinear lines and co-circular arcs. Closed contours are oriented so an outer
/// (even nesting depth) is counter-clockwise and a hole (odd depth) is clockwise.
///
/// More than [`crate::MAX_SEGMENTS`] inputs is an error. Non-finite geometry cannot be
/// constructed, so it never reaches this function.
pub fn cleanup(segments: &[Segment], tolerance: &Tolerance) -> Result<Vec<Contour>> {
    util::check_segment_count(segments.len())?;
    let mut items = Vec::new();
    for seg in segments {
        if !seg.length().is_finite() {
            return Err(GeomError::InvalidInput("segment length is not finite".into()));
        }
        if seg.length() <= tolerance.linear() {
            continue;
        }
        if items.iter().any(|kept: &Segment| segments_duplicate(*kept, *seg, tolerance)) {
            continue;
        }
        items.push(*seg);
    }

    let mut used = vec![false; items.len()];
    let mut contours = Vec::new();
    for i in 0..items.len() {
        if used.get(i).copied().unwrap_or(true) {
            continue;
        }
        set_used(&mut used, i)?;
        let Some(seed) = items.get(i).copied() else {
            return Err(GeomError::InvalidInput("internal: missing segment".into()));
        };
        let mut chain = VecDeque::new();
        chain.push_back(seed);
        extend(&mut chain, &items, &mut used, true, tolerance)?;
        extend(&mut chain, &items, &mut used, false, tolerance)?;
        let mut segs: Vec<Segment> = chain.into_iter().collect();
        let closed = match (segs.first(), segs.last()) {
            (Some(first), Some(last)) => first.start().almost_eq(last.end(), tolerance.linear()),
            _ => false,
        };
        segs = merge_runs(segs, closed, tolerance)?;
        let contour = Contour::try_new(segs, closed, tolerance)?;
        contours.push(contour);
    }
    orient_contours(&mut contours, tolerance)?;
    sort_contours(&mut contours)?;
    Ok(contours)
}

/// Outer counter-clockwise, holes clockwise, by nesting depth.
pub(crate) fn orient_contours(contours: &mut [Contour], tolerance: &Tolerance) -> Result<()> {
    let n = contours.len();
    let mut depth = vec![0usize; n];
    for i in 0..n {
        let Some(inner) = contours.get(i) else {
            continue;
        };
        if !inner.is_closed() {
            continue;
        }
        for j in 0..n {
            if i == j {
                continue;
            }
            let Some(outer) = contours.get(j) else {
                continue;
            };
            if !outer.is_closed() {
                continue;
            }
            if strictly_inside(inner, outer, tolerance)?
                && let Some(slot) = depth.get_mut(i)
            {
                *slot += 1;
            }
        }
    }
    for i in 0..n {
        let Some(contour) = contours.get(i) else {
            continue;
        };
        if !contour.is_closed() {
            continue;
        }
        let nest = depth.get(i).copied().unwrap_or(0);
        let want_ccw = nest % 2 == 0;
        let flipped = match contour.orientation(tolerance)? {
            Orientation::Degenerate => None,
            Orientation::CounterClockwise if !want_ccw => Some(contour.reversed()),
            Orientation::Clockwise if want_ccw => Some(contour.reversed()),
            _ => None,
        };
        if let Some(flipped) = flipped
            && let Some(slot) = contours.get_mut(i)
        {
            *slot = flipped;
        }
    }
    Ok(())
}

fn strictly_inside(inner: &Contour, outer: &Contour, tolerance: &Tolerance) -> Result<bool> {
    let mut any_strict = false;
    for seg in inner.segments() {
        match outer.containment(seg.start(), tolerance)? {
            crate::Containment::Outside => return Ok(false),
            crate::Containment::Inside => any_strict = true,
            crate::Containment::Boundary => {}
        }
    }
    Ok(any_strict)
}

fn extend(chain: &mut VecDeque<Segment>, items: &[Segment], used: &mut [bool], forward: bool, tolerance: &Tolerance) -> Result<()> {
    let limit = items.len().saturating_add(1);
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > limit {
            return Err(GeomError::InvalidInput("cleanup join did not finish".into()));
        }
        let at_seg = if forward { chain.back().copied() } else { chain.front().copied() };
        let Some(at_seg) = at_seg else {
            return Err(GeomError::InvalidInput("internal: empty chain".into()));
        };
        let at = if forward { at_seg.end() } else { at_seg.start() };
        let prefer = if forward { at_seg.end_tangent() } else { at_seg.start_tangent() };
        let Some((index, reverse)) = best_match(items, used, at, prefer, forward, tolerance) else {
            break;
        };
        set_used(used, index)?;
        let Some(raw) = items.get(index).copied() else {
            return Err(GeomError::InvalidInput("internal: missing segment".into()));
        };
        let seg = if reverse { raw.reversed() } else { raw };
        if forward {
            chain.push_back(seg);
        } else {
            chain.push_front(seg);
        }
    }
    Ok(())
}

struct Match {
    index: usize,
    reverse: bool,
    gap: f64,
    dot: f64,
}

fn best_match(items: &[Segment], used: &[bool], at: Point, prefer: Option<Vec2>, forward: bool, tolerance: &Tolerance) -> Option<(usize, bool)> {
    let mut best: Option<Match> = None;
    for (index, seg) in items.iter().copied().enumerate() {
        if used.get(index).copied().unwrap_or(true) {
            continue;
        }
        for reverse in [false, true] {
            let oriented = if reverse { seg.reversed() } else { seg };
            let join = if forward { oriented.start() } else { oriented.end() };
            let gap = join.distance(at);
            if !gap.is_finite() || gap > tolerance.linear() {
                continue;
            }
            let tangent = if forward { oriented.start_tangent() } else { oriented.end_tangent() };
            let dot = match (prefer, tangent) {
                (Some(want), Some(have)) => want.dot(have),
                _ => 0.0,
            };
            let candidate = Match { index, reverse, gap, dot };
            let take = match &best {
                None => true,
                Some(current) => better(&candidate, current),
            };
            if take {
                best = Some(candidate);
            }
        }
    }
    best.map(|m| (m.index, m.reverse))
}

fn better(a: &Match, b: &Match) -> bool {
    use std::cmp::Ordering;
    match crate::cmp_float_key(a.gap, b.gap) {
        Ordering::Less => return true,
        Ordering::Greater => return false,
        Ordering::Equal => {}
    }
    match crate::cmp_float_key(b.dot, a.dot) {
        Ordering::Less => return true,
        Ordering::Greater => return false,
        Ordering::Equal => {}
    }
    if a.index != b.index {
        return a.index < b.index;
    }
    !a.reverse && b.reverse
}

fn set_used(used: &mut [bool], index: usize) -> Result<()> {
    let slot = used.get_mut(index).ok_or_else(|| GeomError::InvalidInput("internal: used index".into()))?;
    *slot = true;
    Ok(())
}

fn segments_duplicate(a: Segment, b: Segment, tolerance: &Tolerance) -> bool {
    match (a, b) {
        (Segment::Line(a), Segment::Line(b)) => lines_duplicate(a, b, tolerance.linear()),
        (Segment::Arc(a), Segment::Arc(b)) => arcs_duplicate(a, b, tolerance),
        _ => false,
    }
}

fn lines_duplicate(a: Line, b: Line, tol: f64) -> bool {
    let same = a.start().almost_eq(b.start(), tol) && a.end().almost_eq(b.end(), tol);
    let rev = a.start().almost_eq(b.end(), tol) && a.end().almost_eq(b.start(), tol);
    same || rev
}

fn arcs_duplicate(a: Arc, b: Arc, tolerance: &Tolerance) -> bool {
    if a.center().distance(b.center()) > tolerance.linear() {
        return false;
    }
    if (a.radius() - b.radius()).abs() > tolerance.linear() {
        return false;
    }
    let ang = tolerance.angular().max(1e-9);
    let same = angle_diff(a.start_angle(), b.start_angle()).abs() <= ang && (a.sweep() - b.sweep()).abs() <= ang;
    let rev = angle_diff(a.end_angle(), b.start_angle()).abs() <= ang && (a.sweep() + b.sweep()).abs() <= ang;
    same || rev
}

fn merge_runs(segments: Vec<Segment>, closed: bool, tolerance: &Tolerance) -> Result<Vec<Segment>> {
    let mut segs = segments;
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > 64 {
            break;
        }
        let n = segs.len();
        if n < 2 {
            break;
        }
        let mut merged_any = false;
        let mut out: Vec<Segment> = Vec::with_capacity(n);
        let mut i = 0usize;
        while i < n {
            if i + 1 < n
                && let (Some(a), Some(b)) = (segs.get(i).copied(), segs.get(i + 1).copied())
                && let Some(merged) = try_merge(a, b, tolerance)
            {
                let prev_ok =
                    if i == 0 { true } else { segs.get(i - 1).is_some_and(|prev| prev.end().distance(merged.start()) <= tolerance.linear()) };
                let next_ok =
                    if i + 2 >= n { true } else { segs.get(i + 2).is_some_and(|next| merged.end().distance(next.start()) <= tolerance.linear()) };
                if prev_ok && next_ok {
                    out.push(merged);
                    i += 2;
                    merged_any = true;
                    continue;
                }
            }
            let Some(seg) = segs.get(i).copied() else {
                break;
            };
            out.push(seg);
            i += 1;
        }
        if closed && out.len() >= 2 {
            let last = out.len() - 1;
            if let (Some(a), Some(b)) = (out.get(last).copied(), out.first().copied())
                && let Some(merged) = try_merge(a, b, tolerance)
            {
                let prev_ok = if out.len() == 2 {
                    true
                } else {
                    out.get(last - 1).is_some_and(|prev| prev.end().distance(merged.start()) <= tolerance.linear())
                };
                let next_ok = if out.len() == 2 {
                    merged.end().distance(merged.start()) <= tolerance.linear()
                } else {
                    out.get(1).is_some_and(|next| merged.end().distance(next.start()) <= tolerance.linear())
                };
                if prev_ok && next_ok {
                    if let Some(slot) = out.get_mut(0) {
                        *slot = merged;
                    }
                    out.pop();
                    merged_any = true;
                }
            }
        }
        if !merged_any {
            break;
        }
        segs = out;
    }
    Ok(segs)
}

fn try_merge(a: Segment, b: Segment, tolerance: &Tolerance) -> Option<Segment> {
    match (a, b) {
        (Segment::Line(a), Segment::Line(b)) => merge_lines(a, b, tolerance).map(Segment::Line),
        (Segment::Arc(a), Segment::Arc(b)) => merge_arcs(a, b, tolerance).map(Segment::Arc),
        _ => None,
    }
}

fn merge_lines(a: Line, b: Line, tolerance: &Tolerance) -> Option<Line> {
    if a.end().distance(b.start()) > tolerance.linear() {
        return None;
    }
    let d1 = a.direction();
    let d2 = b.direction();
    if d1.dot(d2) <= 0.0 {
        return None;
    }
    let dir1 = d1.normalized()?;
    let dir2 = d2.normalized()?;
    let cross = dir1.cross(dir2);
    let dot = dir1.dot(dir2).clamp(-1.0, 1.0);
    let angle = cross.atan2(dot);
    if angle.abs() > tolerance.angular() {
        return None;
    }
    let merged = Line::new(a.start(), b.end()).ok()?;
    if merged.distance_to_point(a.end()) > tolerance.linear() {
        return None;
    }
    if merged.length() <= tolerance.linear() {
        return None;
    }
    Some(merged)
}

fn merge_arcs(a: Arc, b: Arc, tolerance: &Tolerance) -> Option<Arc> {
    if a.center().distance(b.center()) > tolerance.linear() {
        return None;
    }
    if (a.radius() - b.radius()).abs() > tolerance.linear() {
        return None;
    }
    if a.end_point().distance(b.start_point()) > tolerance.linear() {
        return None;
    }
    if a.sweep().signum() != b.sweep().signum() {
        return None;
    }
    if angle_diff(a.end_angle(), b.start_angle()).abs() > tolerance.angular().max(1e-8) {
        return None;
    }
    let sweep = a.sweep() + b.sweep();
    if sweep.abs() > TAU + 1e-6 {
        return None;
    }
    let merged = Arc::new(a.center(), a.radius(), a.start_angle(), sweep).ok()?;
    // Keep the exposed end within a nanometre of the original so a later join still fits.
    if merged.end_point().distance(b.end_point()) > 1e-8 {
        return None;
    }
    Some(merged)
}

fn sort_contours(contours: &mut [Contour]) -> Result<()> {
    let mut keyed: Vec<(u8, f64, f64, f64, f64, usize)> = Vec::with_capacity(contours.len());
    for (index, contour) in contours.iter().enumerate() {
        let rank = if contour.is_closed() { 0u8 } else { 1u8 };
        let (x, y) = match contour.segments().first() {
            Some(seg) => (seg.start().x(), seg.start().y()),
            None => (0.0, 0.0),
        };
        let area = if contour.is_closed() { contour.signed_area()? } else { 0.0 };
        let len = contour.length()?;
        keyed.push((rank, x, y, area, len, index));
    }
    keyed.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then(crate::cmp_float_key(a.1, b.1))
            .then(crate::cmp_float_key(a.2, b.2))
            .then(crate::cmp_float_key(a.3, b.3))
            .then(crate::cmp_float_key(a.4, b.4))
            .then(crate::cmp_xy_rounded((a.1, a.2), (b.1, b.2)))
            .then(a.5.cmp(&b.5))
    });
    let original = contours.to_vec();
    for (slot, key) in contours.iter_mut().zip(keyed.iter()) {
        if let Some(contour) = original.get(key.5) {
            *slot = contour.clone();
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arc::Arc;
    use crate::line::Line;
    use crate::point::Point;
    use std::f64::consts::PI;

    fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> Segment {
        Segment::Line(Line::new(Point::new(x0, y0).unwrap(), Point::new(x1, y1).unwrap()).unwrap())
    }

    fn square(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Segment> {
        vec![line(x0, y0, x1, y0), line(x1, y0, x1, y1), line(x1, y1, x0, y1), line(x0, y1, x0, y0)]
    }

    #[test]
    fn merges_collinear_lines_across_a_small_gap_and_is_idempotent() {
        let tol = Tolerance::default();
        let segs = vec![line(0.0, 0.0, 1.0, 0.0), line(1.0 + 5e-7, 0.0, 2.0, 0.0)];
        let once = cleanup(&segs, &tol).unwrap();
        assert_eq!(once.len(), 1);
        assert!(!once[0].is_closed());
        assert_eq!(once[0].segments().len(), 1);
        let seg = once[0].segments()[0];
        assert!(seg.start().almost_eq(Point::xy(0.0, 0.0), 1e-9));
        assert!(seg.end().almost_eq(Point::xy(2.0, 0.0), 1e-9));
        let flat: Vec<Segment> = once.iter().flat_map(|c| c.segments().iter().copied()).collect();
        let twice = cleanup(&flat, &tol).unwrap();
        assert_eq!(twice.len(), 1);
        assert_eq!(twice[0].segments().len(), 1);
        assert!(twice[0].segments()[0].start().almost_eq(seg.start(), 1e-9));
        assert!(twice[0].segments()[0].end().almost_eq(seg.end(), 1e-9));
    }

    #[test]
    fn drops_duplicates_and_tiny_segments() {
        let tol = Tolerance::default();
        let segs = vec![line(0.0, 0.0, 5.0, 0.0), line(5.0, 0.0, 0.0, 0.0), line(0.0, 0.0, 1e-9, 0.0)];
        let out = cleanup(&segs, &tol).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].segments().len(), 1);
        assert!((out[0].length().unwrap() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn orients_a_clockwise_square_and_a_hole() {
        let tol = Tolerance::default();
        let mut cw = square(0.0, 0.0, 10.0, 10.0);
        cw.reverse();
        for seg in &mut cw {
            *seg = seg.reversed();
        }
        let out = cleanup(&cw, &tol).unwrap();
        assert_eq!(out.len(), 1);
        assert!(out[0].signed_area().unwrap() > 0.0);
        assert!((out[0].signed_area().unwrap() - 100.0).abs() < 1e-8);

        let mut soup = square(0.0, 0.0, 30.0, 30.0);
        soup.extend(square(5.0, 5.0, 25.0, 25.0));
        soup.extend(square(10.0, 10.0, 20.0, 20.0));
        let nested = cleanup(&soup, &tol).unwrap();
        assert_eq!(nested.len(), 3);
        let mut areas: Vec<f64> = nested.iter().map(|c| c.signed_area().unwrap()).collect();
        areas.sort_by(|a, b| a.total_cmp(b));
        // Hole (middle square, area 400) is clockwise. Outer 900 and island 100 stay positive.
        assert!((areas[0] + 400.0).abs() < 1e-6, "{areas:?}");
        assert!((areas[1] - 100.0).abs() < 1e-6, "{areas:?}");
        assert!((areas[2] - 900.0).abs() < 1e-6, "{areas:?}");
    }

    #[test]
    fn merges_two_semicircles_into_a_full_turn() {
        let tol = Tolerance::default();
        let r = 3.0;
        let east = Point::new(r, 0.0).unwrap();
        let west = Point::new(-r, 0.0).unwrap();
        let a = Segment::Arc(Arc::from_bulge(east, west, 1.0).unwrap());
        let b = Segment::Arc(Arc::from_bulge(west, east, 1.0).unwrap());
        let once = cleanup(&[a, b], &tol).unwrap();
        assert_eq!(once.len(), 1);
        assert!(once[0].is_closed());
        assert_eq!(once[0].segments().len(), 1);
        let area = once[0].signed_area().unwrap();
        assert!((area - PI * r * r).abs() < 1e-8, "{area}");
        let flat: Vec<_> = once[0].segments().to_vec();
        let twice = cleanup(&flat, &tol).unwrap();
        assert_eq!(twice.len(), 1);
        assert_eq!(twice[0].segments().len(), 1);
        assert!((twice[0].signed_area().unwrap() - area).abs() < 1e-8);
    }

    #[test]
    fn clockwise_circle_is_flipped_counter_clockwise() {
        let r = 2.0;
        let east = Point::new(r, 0.0).unwrap();
        let west = Point::new(-r, 0.0).unwrap();
        let a = Segment::Arc(Arc::from_bulge(east, west, -1.0).unwrap());
        let b = Segment::Arc(Arc::from_bulge(west, east, -1.0).unwrap());
        let out = cleanup(&[a, b], &Tolerance::default()).unwrap();
        assert!(out[0].signed_area().unwrap() > 0.0);
    }

    #[test]
    fn empty_and_too_many_inputs() {
        assert!(cleanup(&[], &Tolerance::default()).unwrap().is_empty());
        let seg = line(0.0, 0.0, 1.0, 0.0);
        let many = vec![seg; crate::MAX_SEGMENTS + 1];
        assert!(cleanup(&many, &Tolerance::default()).is_err());
    }

    #[test]
    fn does_not_merge_a_right_angle() {
        let out = cleanup(&[line(0.0, 0.0, 1.0, 0.0), line(1.0, 0.0, 1.0, 1.0)], &Tolerance::default()).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].segments().len(), 2);
        assert!(!out[0].is_closed());
    }
}
