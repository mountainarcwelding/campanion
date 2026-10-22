//! Join open paths that share a layer into contours.
//!
//! Ends within `gap` millimetres are joined. A gap wider than the linear tolerance is filled
//! with a straight bridge. A path whose own ends lie within `gap` becomes closed. Anything that
//! does not close stays open.
//!
//! When several joins are equally close, the path already in the chain keeps its direction: the
//! joiner prefers to extend the chain's end, then prefers not to reverse the newcomer, then
//! prefers the earlier path.

use campanion_geom::{Contour, Line, Point, Segment, Tolerance};

use crate::{MAX_TOTAL_SEGMENTS, Result, invalid};

struct Path {
    segments: Vec<Segment>,
}

struct Choice {
    index: usize,
    at_end: bool,
    reverse: bool,
    gap: f64,
}

/// Chain open paths. Each input path is already connected within `tolerance`.
pub(crate) fn chain_paths(paths: Vec<Vec<Segment>>, gap: f64, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    if !gap.is_finite() || gap < 0.0 {
        return Err(invalid("gap tolerance must be finite and >= 0"));
    }
    let mut pending: Vec<Option<Path>> = Vec::with_capacity(paths.len());
    for segments in paths {
        if segments.is_empty() {
            return Err(invalid("internal: empty path"));
        }
        if segments.len() > MAX_TOTAL_SEGMENTS {
            return Err(invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")));
        }
        pending.push(Some(Path { segments }));
    }

    let mut contours = Vec::new();
    let mut cursor = 0usize;
    let mut guard = 0usize;
    let limit = pending.len().saturating_add(1);
    while cursor < pending.len() {
        guard = step(guard, limit)?;
        let Some(slot) = pending.get_mut(cursor) else {
            return Err(invalid("internal: path index"));
        };
        let Some(mut path) = slot.take() else {
            cursor = step(cursor, limit)?;
            continue;
        };
        extend(&mut path, &mut pending, gap, tolerance)?;
        contours.push(finish(path, gap, tolerance)?);
        cursor = step(cursor, limit)?;
    }
    Ok(contours)
}

fn extend(path: &mut Path, pending: &mut [Option<Path>], gap: f64, tolerance: &Tolerance) -> Result<()> {
    let mut guard = 0usize;
    let limit = pending.len().saturating_add(1);
    loop {
        guard = step(guard, limit)?;
        let Some(choice) = best_attach(path, pending, gap)? else {
            return Ok(());
        };
        let Some(slot) = pending.get_mut(choice.index) else {
            return Err(invalid("internal: missing path"));
        };
        let Some(other) = slot.take() else {
            return Err(invalid("internal: path already used"));
        };
        attach(path, other, &choice, tolerance)?;
    }
}

fn best_attach(path: &Path, pending: &[Option<Path>], gap: f64) -> Result<Option<Choice>> {
    let start = path_start(path)?;
    let end = path_end(path)?;
    let mut best: Option<Choice> = None;
    for (index, slot) in pending.iter().enumerate() {
        let Some(other) = slot else {
            continue;
        };
        let other_start = path_start(other)?;
        let other_end = path_end(other)?;
        let candidates = [
            Choice { index, at_end: true, reverse: false, gap: distance(end, other_start)? },
            Choice { index, at_end: true, reverse: true, gap: distance(end, other_end)? },
            Choice { index, at_end: false, reverse: false, gap: distance(start, other_end)? },
            Choice { index, at_end: false, reverse: true, gap: distance(start, other_start)? },
        ];
        for candidate in candidates {
            if candidate.gap > gap {
                continue;
            }
            let take = match &best {
                None => true,
                Some(current) => prefer(&candidate, current),
            };
            if take {
                best = Some(candidate);
            }
        }
    }
    Ok(best)
}

/// Smaller gap wins. Ties extend the chain end, avoid a reverse, then take the earlier path.
fn prefer(candidate: &Choice, current: &Choice) -> bool {
    let spread = (candidate.gap - current.gap).abs();
    if spread > 1e-9 && spread > candidate.gap.max(current.gap) * 1e-9 {
        return candidate.gap < current.gap;
    }
    if candidate.at_end != current.at_end {
        return candidate.at_end;
    }
    if candidate.reverse != current.reverse {
        return !candidate.reverse;
    }
    candidate.index < current.index
}

fn attach(path: &mut Path, other: Path, choice: &Choice, tolerance: &Tolerance) -> Result<()> {
    let segments = orient(other.segments, choice.reverse);
    let (Some(first), Some(last)) = (segments.first().copied(), segments.last().copied()) else {
        return Err(invalid("internal: empty path"));
    };
    if choice.at_end {
        if let Some(bridge) = bridge(path_end(path)?, first.start(), tolerance)? {
            path.segments.push(bridge);
        }
        path.segments.extend(segments);
    } else {
        let mut merged = segments;
        if let Some(bridge) = bridge(last.end(), path_start(path)?, tolerance)? {
            merged.push(bridge);
        }
        merged.append(&mut path.segments);
        path.segments = merged;
    }
    if path.segments.len() > MAX_TOTAL_SEGMENTS {
        return Err(invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")));
    }
    Ok(())
}

fn finish(mut path: Path, gap: f64, tolerance: &Tolerance) -> Result<Contour> {
    let start = path_start(&path)?;
    let end = path_end(&path)?;
    let closing = distance(end, start)?;
    let closed = closing <= gap;
    if closed && let Some(bridge) = bridge(end, start, tolerance)? {
        path.segments.push(bridge);
    }
    if path.segments.is_empty() || path.segments.len() > MAX_TOTAL_SEGMENTS {
        return Err(invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")));
    }
    Contour::try_new(path.segments, closed, tolerance).map_err(crate::ImportError::from_geom)
}

fn orient(mut segments: Vec<Segment>, reverse: bool) -> Vec<Segment> {
    if reverse {
        segments.reverse();
        for seg in &mut segments {
            *seg = seg.reversed();
        }
    }
    segments
}

/// A bridge is required only when the ends are farther apart than the linear tolerance.
fn bridge(from: Point, to: Point, tolerance: &Tolerance) -> Result<Option<Segment>> {
    let span = distance(from, to)?;
    if span <= tolerance.linear() {
        return Ok(None);
    }
    let line = Line::new(from, to).map_err(crate::ImportError::from_geom)?;
    Ok(Some(Segment::Line(line)))
}

fn path_start(path: &Path) -> Result<Point> {
    path.segments.first().map(|seg| seg.start()).ok_or_else(|| invalid("internal: empty path"))
}

fn path_end(path: &Path) -> Result<Point> {
    path.segments.last().map(|seg| seg.end()).ok_or_else(|| invalid("internal: empty path"))
}

fn distance(from: Point, to: Point) -> Result<f64> {
    let span = from.distance(to);
    if span.is_finite() { Ok(span) } else { Err(invalid("distance is not finite")) }
}

fn step(value: usize, limit: usize) -> Result<usize> {
    let next = value.checked_add(1).ok_or_else(|| invalid("chaining did not finish"))?;
    if next > limit { Err(invalid("chaining did not finish")) } else { Ok(next) }
}
