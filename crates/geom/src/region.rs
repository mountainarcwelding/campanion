//! A filled area: one counter-clockwise outer contour and clockwise holes.
//!
//! Holes are islands of empty space. A counter-clockwise contour sitting inside a hole is its
//! own region, not a hole of the outer. Boolean operations are built from [`crate::boolean`] so
//! the geometry engine stays behind [`crate::backend`].

use crate::GeomError;
use crate::Result;
use crate::backend::{BooleanOp, boolean};
use crate::contour::{Containment, Contour, Orientation};
use crate::tolerance::Tolerance;

/// How many times region clipping may nest before it gives up.
const MAX_BOOL_DEPTH: usize = 8;

/// Largest contour count accepted while folding a union.
const MAX_UNION_PIECES: usize = 64;

/// Filled area bounded by one outer contour and zero or more holes.
///
/// `try_new` stores the outer counter-clockwise and each hole clockwise. Area is the outer's
/// signed area plus the holes' signed areas, so a clockwise hole subtracts.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    outer: Contour,
    holes: Vec<Contour>,
}

impl Region {
    /// Outer must be closed. Holes must be closed and lie inside the outer, and must not contain
    /// each other. Winding is corrected to counter-clockwise outer and clockwise holes.
    pub fn try_new(outer: Contour, holes: Vec<Contour>, tolerance: &Tolerance) -> Result<Self> {
        if !outer.is_closed() {
            return Err(GeomError::InvalidInput("outer contour must be closed".into()));
        }
        let outer = force_ccw(outer, tolerance)?;
        let mut fixed = Vec::with_capacity(holes.len());
        for hole in holes {
            if !hole.is_closed() {
                return Err(GeomError::InvalidInput("hole contour must be closed".into()));
            }
            let hole = force_cw(hole, tolerance)?;
            if !strictly_inside(&hole, &outer, tolerance)? {
                return Err(GeomError::InvalidInput("hole is outside the outer contour".into()));
            }
            fixed.push(hole);
        }
        for i in 0..fixed.len() {
            for j in 0..fixed.len() {
                if i != j
                    && let (Some(inner), Some(outer_hole)) = (fixed.get(i), fixed.get(j))
                    && strictly_inside(inner, outer_hole, tolerance)?
                {
                    return Err(GeomError::InvalidInput("holes must not contain each other".into()));
                }
            }
        }
        Ok(Self { outer, holes: fixed })
    }

    /// A region with no holes. The contour is oriented counter-clockwise.
    pub fn from_contour(contour: Contour, tolerance: &Tolerance) -> Result<Self> {
        Self::try_new(contour, Vec::new(), tolerance)
    }

    pub fn outer(&self) -> &Contour {
        &self.outer
    }

    pub fn holes(&self) -> &[Contour] {
        &self.holes
    }

    /// Filled area in mm². Positive when the outer winds counter-clockwise.
    pub fn area(&self) -> Result<f64> {
        let mut area = self.outer.signed_area()?;
        for hole in &self.holes {
            area += hole.signed_area()?;
        }
        if area.is_finite() { Ok(area) } else { Err(GeomError::InvalidInput("area is not finite".into())) }
    }

    pub(crate) fn from_parts(outer: Contour, holes: Vec<Contour>) -> Self {
        Self { outer, holes }
    }
}

/// Union, difference, or intersection of two regions.
///
/// Results may be disjoint, and a difference may open a hole. `ExclusiveOr` is the symmetric
/// difference. An empty `Vec` means the filled area vanished.
pub fn boolean_regions(op: BooleanOp, a: &Region, b: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let regions = match op {
        BooleanOp::Union => union_regions(a, b, tolerance)?,
        BooleanOp::Intersection => intersect_regions(a, b, tolerance)?,
        BooleanOp::Difference => difference_regions(a, b, tolerance)?,
        BooleanOp::ExclusiveOr => {
            let left = difference_regions(a, b, tolerance)?;
            let right = difference_regions(b, a, tolerance)?;
            union_shapes(&left, &right, tolerance)?
        }
    };
    sort_regions(regions, tolerance)
}

/// Group closed contours into regions by nesting.
///
/// Even depth is an outer, odd depth a hole of that outer. A negative contour that nothing
/// contains is an orphan hole and is dropped.
pub(crate) fn contours_to_regions(contours: &[Contour], tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut closed = Vec::new();
    for contour in contours {
        if contour.is_closed() && !sliver(contour, tolerance) {
            closed.push(contour.clone());
        }
    }
    let n = closed.len();
    let mut parent: Vec<Option<usize>> = vec![None; n];
    for i in 0..n {
        let Some(inner) = closed.get(i) else {
            continue;
        };
        let mut best: Option<(usize, f64, (f64, f64))> = None;
        for j in 0..n {
            if i == j {
                continue;
            }
            let Some(outer) = closed.get(j) else {
                continue;
            };
            if !strictly_inside(inner, outer, tolerance)? {
                continue;
            }
            let area = outer.signed_area()?.abs();
            let bounds = outer.bounding_box()?;
            let xy = (bounds.min().x(), bounds.min().y());
            let take = match best {
                None => true,
                Some((best_index, best_area, best_xy)) => crate::min_beats(area, xy, j, best_area, best_xy, best_index),
            };
            if take {
                best = Some((j, area, xy));
            }
        }
        if let Some((j, _, _)) = best
            && let Some(slot) = parent.get_mut(i)
        {
            *slot = Some(j);
        }
    }

    let mut depth = vec![0usize; n];
    for i in 0..n {
        let mut d = 0usize;
        let mut cursor = parent.get(i).copied().flatten();
        let mut guard = 0usize;
        while let Some(p) = cursor {
            d += 1;
            guard += 1;
            if guard > n {
                return Err(GeomError::InvalidInput("contour nesting did not finish".into()));
            }
            cursor = parent.get(p).copied().flatten();
        }
        if let Some(slot) = depth.get_mut(i) {
            *slot = d;
        }
    }

    let mut regions = Vec::new();
    for i in 0..n {
        let nest = depth.get(i).copied().unwrap_or(0);
        if nest % 2 != 0 {
            continue;
        }
        let Some(raw) = closed.get(i) else {
            continue;
        };
        if nest == 0 && raw.signed_area()? < 0.0 {
            continue;
        }
        let outer = force_ccw(raw.clone(), tolerance)?;
        let mut holes = Vec::new();
        for j in 0..n {
            if parent.get(j).copied().flatten() == Some(i)
                && depth.get(j).copied() == Some(nest + 1)
                && let Some(hole_raw) = closed.get(j)
            {
                holes.push(force_cw(hole_raw.clone(), tolerance)?);
            }
        }
        regions.push(Region::from_parts(outer, holes));
    }
    sort_regions(regions, tolerance)
}

fn union_regions(a: &Region, b: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    union_one_region(std::slice::from_ref(a), b, tolerance, 0)
}

fn intersect_regions(a: &Region, b: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut parts = shape_intersect_disk(std::slice::from_ref(a), b.outer(), tolerance)?;
    for hole in b.holes() {
        parts = shape_minus_disk(&parts, hole, tolerance)?;
    }
    Ok(parts)
}

fn difference_regions(a: &Region, b: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    // (a − b.outer) plus the parts of a that sit in b's holes (those holes are not filled).
    let mut parts = shape_minus_disk(std::slice::from_ref(a), b.outer(), tolerance)?;
    for hole in b.holes() {
        let back = shape_intersect_disk(std::slice::from_ref(a), hole, tolerance)?;
        parts = union_shapes_depth(&parts, &back, tolerance, 0)?;
    }
    Ok(parts)
}

fn union_shapes(a: &[Region], b: &[Region], tolerance: &Tolerance) -> Result<Vec<Region>> {
    union_shapes_depth(a, b, tolerance, 0)
}

fn union_shapes_depth(a: &[Region], b: &[Region], tolerance: &Tolerance, depth: usize) -> Result<Vec<Region>> {
    let mut acc = a.to_vec();
    for region in b {
        acc = union_one_region(&acc, region, tolerance, depth)?;
    }
    Ok(acc)
}

fn union_one_region(shape: &[Region], region: &Region, tolerance: &Tolerance, depth: usize) -> Result<Vec<Region>> {
    if depth > MAX_BOOL_DEPTH {
        return Err(GeomError::InvalidInput("region boolean did not finish".into()));
    }
    let original = shape.to_vec();
    let mut acc = merge_disk(shape, region.outer(), tolerance)?;
    for hole in region.holes() {
        acc = shape_minus_disk(&acc, hole, tolerance)?;
        let back = shape_intersect_disk(&original, hole, tolerance)?;
        for piece in &back {
            acc = union_one_region(&acc, piece, tolerance, depth + 1)?;
        }
    }
    Ok(acc)
}

/// Union a simple disk into a shape. Regions far from the disk are left untouched.
fn merge_disk(shape: &[Region], disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let disk = force_ccw(disk.clone(), tolerance)?;
    let mut idle = Vec::new();
    let mut hit = Vec::new();
    for region in shape {
        if outers_separated(region.outer(), &disk, tolerance)? {
            idle.push(region.clone());
        } else {
            hit.push(region.clone());
        }
    }
    if hit.is_empty() {
        idle.push(Region::from_contour(disk, tolerance)?);
        return Ok(idle);
    }
    let Some((first, rest)) = hit.split_first() else {
        return Ok(idle);
    };
    let mut united = union_disk_with_one(first, &disk, tolerance)?;
    for region in rest {
        united = bridge_region(&united, region, tolerance)?;
    }
    idle.extend(united);
    Ok(idle)
}

fn union_disk_with_one(region: &Region, disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let outer = force_ccw(region.outer().clone(), tolerance)?;
    let disk = force_ccw(disk.clone(), tolerance)?;
    let merged = boolean(BooleanOp::Union, &outer, &disk, tolerance)?;
    let regs = contours_to_regions(&merged, tolerance)?;
    restore_holes(&regs, region.holes(), &disk, tolerance)
}

/// `disk` bridges several regions. Union their outers, then put each hole back where the new
/// disk does not cover it.
fn bridge_region(shape: &[Region], region: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let outer = force_ccw(region.outer().clone(), tolerance)?;
    let mut idle = Vec::new();
    let mut hit = Vec::new();
    for existing in shape {
        if outers_separated(existing.outer(), &outer, tolerance)? {
            idle.push(existing.clone());
        } else {
            hit.push(existing.clone());
        }
    }
    if hit.is_empty() {
        idle.push(region.clone());
        return Ok(idle);
    }
    let mut contours = vec![outer.clone()];
    for existing in &hit {
        contours = union_into(contours, existing.outer(), tolerance)?;
    }
    let mut regs = contours_to_regions(&contours, tolerance)?;
    // A hole stays empty only where none of the other filled outers cover it.
    for hole in region.holes() {
        let mut remain = vec![Region::from_parts(force_ccw(hole.clone(), tolerance)?, Vec::new())];
        for existing in &hit {
            remain = shape_minus_disk(&remain, existing.outer(), tolerance)?;
        }
        for piece in remain {
            regs = punch_disk(&regs, piece.outer(), tolerance)?;
            regs.extend(islands_of(&piece, tolerance)?);
        }
    }
    for existing in &hit {
        for hole in existing.holes() {
            let remain = disk_minus(hole, &outer, tolerance)?;
            for piece in remain {
                regs = punch_disk(&regs, piece.outer(), tolerance)?;
                regs.extend(islands_of(&piece, tolerance)?);
            }
        }
    }
    idle.extend(regs);
    Ok(idle)
}

fn union_into(set: Vec<Contour>, disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Contour>> {
    let mut acc = set;
    let mut pending = vec![force_ccw(disk.clone(), tolerance)?];
    let mut holes = Vec::new();
    let mut guard = 0usize;
    while let Some(seed) = pending.pop() {
        guard += 1;
        if guard > MAX_UNION_PIECES {
            return Err(GeomError::InvalidInput("region union did not finish".into()));
        }
        let mut merged = false;
        let mut index = 0usize;
        while index < acc.len() {
            let Some(piece) = acc.get(index) else {
                break;
            };
            if sliver(piece, tolerance) {
                acc.remove(index);
                continue;
            }
            let piece_ccw = force_ccw(piece.clone(), tolerance)?;
            if matches!(relate(&piece_ccw, &seed, tolerance)?, Relation::Disjoint) {
                index += 1;
                continue;
            }
            let piece = acc.remove(index);
            let united = boolean(BooleanOp::Union, &force_ccw(piece, tolerance)?, &seed, tolerance)?;
            split_signed(&united, tolerance, &mut pending, &mut holes)?;
            merged = true;
            break;
        }
        if !merged && !sliver(&seed, tolerance) {
            acc.push(seed);
        }
    }
    acc.extend(holes);
    Ok(acc)
}

fn split_signed(contours: &[Contour], tolerance: &Tolerance, positive: &mut Vec<Contour>, negative: &mut Vec<Contour>) -> Result<()> {
    for contour in contours {
        if !contour.is_closed() || sliver(contour, tolerance) {
            continue;
        }
        match contour.orientation(tolerance)? {
            Orientation::CounterClockwise => positive.push(contour.clone()),
            Orientation::Clockwise => negative.push(contour.clone()),
            Orientation::Degenerate => {}
        }
    }
    Ok(())
}

fn restore_holes(regs: &[Region], holes: &[Contour], disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut acc = regs.to_vec();
    for hole in holes {
        let remain = disk_minus(hole, disk, tolerance)?;
        for piece in remain {
            acc = punch_disk(&acc, piece.outer(), tolerance)?;
            acc.extend(islands_of(&piece, tolerance)?);
        }
    }
    Ok(acc)
}

fn islands_of(piece: &Region, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut islands = Vec::new();
    for hole in piece.holes() {
        if sliver(hole, tolerance) {
            continue;
        }
        islands.push(Region::from_contour(force_ccw(hole.clone(), tolerance)?, tolerance)?);
    }
    Ok(islands)
}

/// Filled area of `disk` minus `cut`. Both are treated as simple disks.
fn disk_minus(disk: &Contour, cut: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let disk = force_ccw(disk.clone(), tolerance)?;
    let cut = force_ccw(cut.clone(), tolerance)?;
    if sliver(&disk, tolerance) {
        return Ok(Vec::new());
    }
    match relate(&disk, &cut, tolerance)? {
        Relation::Disjoint => Ok(vec![Region::from_parts(disk, Vec::new())]),
        Relation::AInsideB | Relation::Equal => Ok(Vec::new()),
        Relation::BInsideA | Relation::Intersects => {
            let diff = boolean(BooleanOp::Difference, &disk, &cut, tolerance)?;
            contours_to_regions(&diff, tolerance)
        }
    }
}

fn shape_minus_disk(shape: &[Region], disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let disk = force_ccw(disk.clone(), tolerance)?;
    if sliver(&disk, tolerance) {
        return Ok(shape.to_vec());
    }
    let mut out = Vec::new();
    for region in shape {
        let outer = force_ccw(region.outer().clone(), tolerance)?;
        match relate(&outer, &disk, tolerance)? {
            Relation::Disjoint => out.push(region.clone()),
            Relation::AInsideB | Relation::Equal => {}
            Relation::BInsideA | Relation::Intersects => {
                let diff = boolean(BooleanOp::Difference, &outer, &disk, tolerance)?;
                let parts = contours_to_regions(&diff, tolerance)?;
                let parts = restore_holes(&parts, region.holes(), &disk, tolerance)?;
                out.extend(parts);
            }
        }
    }
    Ok(out)
}

fn shape_intersect_disk(shape: &[Region], disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let disk = force_ccw(disk.clone(), tolerance)?;
    if sliver(&disk, tolerance) {
        return Ok(Vec::new());
    }
    let mut out = Vec::new();
    for region in shape {
        let outer = force_ccw(region.outer().clone(), tolerance)?;
        match relate(&outer, &disk, tolerance)? {
            Relation::Disjoint => {}
            Relation::AInsideB | Relation::Equal => out.push(region.clone()),
            Relation::BInsideA => {
                let mut parts = vec![Region::from_parts(disk.clone(), Vec::new())];
                for hole in region.holes() {
                    parts = shape_minus_disk(&parts, hole, tolerance)?;
                }
                out.extend(parts);
            }
            Relation::Intersects => {
                let inter = boolean(BooleanOp::Intersection, &outer, &disk, tolerance)?;
                let mut parts = contours_to_regions(&inter, tolerance)?;
                for hole in region.holes() {
                    parts = shape_minus_disk(&parts, hole, tolerance)?;
                }
                out.extend(parts);
            }
        }
    }
    Ok(out)
}

fn punch_disk(shell: &[Region], disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let disk = force_ccw(disk.clone(), tolerance)?;
    if sliver(&disk, tolerance) {
        return Ok(shell.to_vec());
    }
    let mut out = Vec::new();
    for region in shell {
        let outer = force_ccw(region.outer().clone(), tolerance)?;
        match relate(&outer, &disk, tolerance)? {
            Relation::Disjoint => out.push(region.clone()),
            Relation::AInsideB | Relation::Equal => {}
            Relation::BInsideA => {
                if covered_by_hole(region, &disk, tolerance)? {
                    out.push(region.clone());
                } else {
                    out.extend(merge_hole(region, &disk, tolerance)?);
                }
            }
            Relation::Intersects => {
                let diff = boolean(BooleanOp::Difference, &outer, &disk, tolerance)?;
                let cut = contours_to_regions(&diff, tolerance)?;
                let cut = restore_holes(&cut, region.holes(), &disk, tolerance)?;
                out.extend(cut);
            }
        }
    }
    Ok(out)
}

fn covered_by_hole(region: &Region, disk: &Contour, tolerance: &Tolerance) -> Result<bool> {
    for hole in region.holes() {
        let hole = force_ccw(hole.clone(), tolerance)?;
        match relate(disk, &hole, tolerance)? {
            Relation::AInsideB | Relation::Equal => return Ok(true),
            Relation::Disjoint | Relation::BInsideA | Relation::Intersects => {}
        }
    }
    Ok(false)
}

fn merge_hole(region: &Region, disk: &Contour, tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut untouched = Vec::new();
    let mut cluster = vec![force_ccw(disk.clone(), tolerance)?];
    for hole in region.holes() {
        let hole_ccw = force_ccw(hole.clone(), tolerance)?;
        let mut overlaps = false;
        for piece in &cluster {
            if !matches!(relate(piece, &hole_ccw, tolerance)?, Relation::Disjoint) {
                overlaps = true;
                break;
            }
        }
        if overlaps {
            cluster.push(hole_ccw);
        } else {
            untouched.push(force_cw(hole.clone(), tolerance)?);
        }
    }
    let merged = union_disks(&cluster, tolerance)?;
    let mut holes = untouched;
    let mut islands = Vec::new();
    for piece in &merged {
        holes.push(force_cw(piece.outer().clone(), tolerance)?);
        islands.extend(islands_of(piece, tolerance)?);
    }
    let mut regions = vec![Region::from_parts(region.outer().clone(), holes)];
    regions.extend(islands);
    Ok(regions)
}

fn union_disks(disks: &[Contour], tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut acc: Vec<Region> = Vec::new();
    for disk in disks {
        if sliver(disk, tolerance) {
            continue;
        }
        acc = merge_disk(&acc, disk, tolerance)?;
    }
    Ok(acc)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Relation {
    Disjoint,
    /// `a` is covered by `b`.
    AInsideB,
    /// `b` is covered by `a`.
    BInsideA,
    Intersects,
    Equal,
}

fn relate(a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<Relation> {
    if bboxes_separated(a, b, tolerance)? {
        return Ok(Relation::Disjoint);
    }
    let inter = boolean(BooleanOp::Intersection, a, b, tolerance)?;
    let ia = filled_area(&inter, tolerance)?;
    let aa = a.signed_area()?.abs();
    let bb = b.signed_area()?.abs();
    let scale = aa.max(bb).max(1.0);
    // Inside tests stay loose enough for the backend's 1e-5 position epsilon. A partial overlap
    // is left as `Intersects` so the boolean, not this classification, decides the boundary.
    let inside_eps = (scale * 1e-4).max(1e-4);
    let disjoint_eps = (scale * 1e-6).max(1e-6);
    if ia <= disjoint_eps {
        return Ok(Relation::Disjoint);
    }
    let a_in = aa - ia <= inside_eps;
    let b_in = bb - ia <= inside_eps;
    if a_in && b_in {
        Ok(Relation::Equal)
    } else if a_in {
        Ok(Relation::AInsideB)
    } else if b_in {
        Ok(Relation::BInsideA)
    } else {
        Ok(Relation::Intersects)
    }
}

fn filled_area(contours: &[Contour], tolerance: &Tolerance) -> Result<f64> {
    let regions = contours_to_regions(contours, tolerance)?;
    let mut sum = 0.0;
    for region in &regions {
        sum += region.area()?;
    }
    if sum.is_finite() { Ok(sum) } else { Err(GeomError::InvalidInput("area is not finite".into())) }
}

fn outers_separated(a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<bool> {
    if bboxes_separated(a, b, tolerance)? {
        return Ok(true);
    }
    Ok(matches!(relate(a, b, tolerance)?, Relation::Disjoint))
}

fn bboxes_separated(a: &Contour, b: &Contour, tolerance: &Tolerance) -> Result<bool> {
    let ab = a.bounding_box()?;
    let bb = b.bounding_box()?;
    let gap = tolerance.linear().max(1e-6);
    Ok(ab.max().x() < bb.min().x() - gap
        || bb.max().x() < ab.min().x() - gap
        || ab.max().y() < bb.min().y() - gap
        || bb.max().y() < ab.min().y() - gap)
}

fn strictly_inside(inner: &Contour, outer: &Contour, tolerance: &Tolerance) -> Result<bool> {
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

fn sliver(contour: &Contour, tolerance: &Tolerance) -> bool {
    let Ok(area) = contour.signed_area() else {
        return true;
    };
    let Ok(bounds) = contour.bounding_box() else {
        return true;
    };
    let scale = bounds.width().max(bounds.height()).max(tolerance.linear());
    !area.is_finite() || area.abs() <= tolerance.linear() * scale
}

fn force_ccw(contour: Contour, tolerance: &Tolerance) -> Result<Contour> {
    match contour.orientation(tolerance)? {
        Orientation::CounterClockwise => Ok(contour),
        Orientation::Clockwise => Ok(contour.reversed()),
        Orientation::Degenerate => Err(GeomError::InvalidInput("degenerate contour".into())),
    }
}

fn force_cw(contour: Contour, tolerance: &Tolerance) -> Result<Contour> {
    match contour.orientation(tolerance)? {
        Orientation::Clockwise => Ok(contour),
        Orientation::CounterClockwise => Ok(contour.reversed()),
        Orientation::Degenerate => Err(GeomError::InvalidInput("degenerate contour".into())),
    }
}

fn sort_regions(mut regions: Vec<Region>, _tolerance: &Tolerance) -> Result<Vec<Region>> {
    let mut keyed = Vec::with_capacity(regions.len());
    for (index, region) in regions.iter().enumerate() {
        let area = region.area()?;
        let bounds = region.outer().bounding_box()?;
        keyed.push((area, bounds.min().x(), bounds.min().y(), index));
    }
    keyed.sort_by(|a, b| {
        crate::cmp_float_key(b.0, a.0)
            .then(crate::cmp_float_key(a.1, b.1))
            .then(crate::cmp_float_key(a.2, b.2))
            .then(crate::cmp_xy_rounded((a.1, a.2), (b.1, b.2)))
            .then(a.3.cmp(&b.3))
    });
    let original = regions.clone();
    for (slot, key) in regions.iter_mut().zip(keyed.iter()) {
        if let Some(region) = original.get(key.3) {
            *slot = region.clone();
        }
    }
    Ok(regions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::line::Line;
    use crate::point::Point;
    use crate::polyline::{Polyline, Vertex};
    use crate::segment::Segment;
    use std::f64::consts::PI;

    /// Hand calculations on circles and axis-aligned edges. The backend's own epsilon is coarser
    /// (`pos_equal_eps` 1e-5), so a few stitched results use [`LOOSE`] instead.
    const TIGHT: f64 = 1e-6;
    const LOOSE: f64 = 1e-4;

    fn point(x: f64, y: f64) -> Point {
        Point::new(x, y).expect("point")
    }

    fn rect(x: f64, y: f64, w: f64, h: f64) -> Contour {
        let line = |x0, y0, x1, y1| Segment::Line(Line::new(point(x0, y0), point(x1, y1)).expect("line"));
        Contour::try_new(
            vec![line(x, y, x + w, y), line(x + w, y, x + w, y + h), line(x + w, y + h, x, y + h), line(x, y + h, x, y)],
            true,
            &Tolerance::default(),
        )
        .expect("rect")
    }

    fn region_of(contour: Contour) -> Region {
        Region::from_contour(contour, &Tolerance::default()).expect("region")
    }

    fn circle_at(cx: f64, cy: f64, r: f64) -> Contour {
        Polyline::new(vec![Vertex::new(point(cx + r, cy), 1.0).expect("vertex"), Vertex::new(point(cx - r, cy), 1.0).expect("vertex")], true)
            .expect("polyline")
            .to_contour(&Tolerance::default())
            .expect("circle")
    }

    fn areas(regions: &[Region]) -> f64 {
        regions.iter().map(|r| r.area().expect("area")).sum()
    }

    #[test]
    fn overlapping_rectangles_union_intersection_difference() {
        let tol = Tolerance::default();
        // A is 6 by 4 at the origin (area 24). B is 6 by 4 at (2, 1) (area 24).
        // Overlap is x = 2..6 (4) by y = 1..4 (3), area 12.
        let a = region_of(rect(0.0, 0.0, 6.0, 4.0));
        let b = region_of(rect(2.0, 1.0, 6.0, 4.0));
        let union = boolean_regions(BooleanOp::Union, &a, &b, &tol).expect("union");
        let inter = boolean_regions(BooleanOp::Intersection, &a, &b, &tol).expect("intersection");
        let diff = boolean_regions(BooleanOp::Difference, &a, &b, &tol).expect("difference");
        assert!((areas(&union) - 36.0).abs() < TIGHT, "union {}", areas(&union));
        assert!((areas(&inter) - 12.0).abs() < TIGHT, "intersection {}", areas(&inter));
        assert!((areas(&diff) - 12.0).abs() < TIGHT, "difference {}", areas(&diff));
        assert_eq!(union.len(), 1);
        assert!(union[0].holes().is_empty());
        // A = (A − B) + (A ∩ B).
        assert!((24.0 - (areas(&diff) + areas(&inter))).abs() < TIGHT);
    }

    #[test]
    fn disjoint_rectangles_stay_split() {
        let tol = Tolerance::default();
        let a = region_of(rect(0.0, 0.0, 2.0, 2.0));
        let b = region_of(rect(5.0, 5.0, 3.0, 1.0));
        let union = boolean_regions(BooleanOp::Union, &a, &b, &tol).expect("union");
        assert_eq!(union.len(), 2, "got {}", union.len());
        assert!((areas(&union) - 7.0).abs() < TIGHT, "{}", areas(&union));
        let inter = boolean_regions(BooleanOp::Intersection, &a, &b, &tol).expect("intersection");
        assert!(inter.is_empty());
        let diff = boolean_regions(BooleanOp::Difference, &a, &b, &tol).expect("difference");
        assert_eq!(diff.len(), 1);
        assert!((areas(&diff) - 4.0).abs() < TIGHT);
    }

    #[test]
    fn rectangle_minus_circle_is_a_region_with_one_hole() {
        let tol = Tolerance::default();
        // 4 by 4 square minus a radius-1 circle at its centre. Area 16 − π.
        let board = region_of(rect(0.0, 0.0, 4.0, 4.0));
        let hole = region_of(circle_at(2.0, 2.0, 1.0));
        let parts = boolean_regions(BooleanOp::Difference, &board, &hole, &tol).expect("difference");
        assert_eq!(parts.len(), 1, "got {}", parts.len());
        assert_eq!(parts[0].holes().len(), 1);
        let area = parts[0].area().expect("area");
        assert!((area - (16.0 - PI)).abs() < TIGHT, "area {area}");
        assert!(parts[0].outer().signed_area().expect("outer") > 0.0);
        assert!(parts[0].holes()[0].signed_area().expect("hole") < 0.0);
    }

    #[test]
    fn difference_that_crosses_splits_into_two_regions() {
        let tol = Tolerance::default();
        // 10 by 4 plate. A band x = 3..7 cuts it in two. Each piece is 3 by 4, area 12.
        let plate = region_of(rect(0.0, 0.0, 10.0, 4.0));
        let band = region_of(rect(3.0, -1.0, 4.0, 6.0));
        let parts = boolean_regions(BooleanOp::Difference, &plate, &band, &tol).expect("difference");
        assert_eq!(parts.len(), 2, "got {}", parts.len());
        assert!((areas(&parts) - 24.0).abs() < TIGHT, "{}", areas(&parts));
        for part in &parts {
            assert!(part.holes().is_empty());
            assert!((part.area().expect("area") - 12.0).abs() < TIGHT);
        }
    }

    #[test]
    fn region_with_an_island_keeps_the_hole_and_union_fills_it() {
        let tol = Tolerance::default();
        let outer = rect(0.0, 0.0, 10.0, 10.0);
        let island = rect(4.0, 4.0, 2.0, 2.0);
        let region = Region::try_new(outer, vec![island.clone()], &tol).expect("region");
        assert_eq!(region.holes().len(), 1);
        assert!((region.area().expect("area") - 96.0).abs() < TIGHT, "{}", region.area().expect("area"));
        assert!(region.holes()[0].signed_area().expect("hole") < 0.0);

        let filled = boolean_regions(BooleanOp::Union, &region, &region_of(island), &tol).expect("union");
        assert!((areas(&filled) - 100.0).abs() < LOOSE, "filled {}", areas(&filled));

        // The constructed region matches rectangle-minus-rectangle.
        let via_boolean = boolean_regions(BooleanOp::Difference, &region_of(rect(0.0, 0.0, 10.0, 10.0)), &region_of(rect(4.0, 4.0, 2.0, 2.0)), &tol)
            .expect("difference");
        assert_eq!(via_boolean.len(), 1);
        assert_eq!(via_boolean[0].holes().len(), 1);
        assert!((via_boolean[0].area().expect("area") - 96.0).abs() < TIGHT);
    }

    #[test]
    fn identical_rectangles_are_idempotent() {
        let tol = Tolerance::default();
        let a = region_of(rect(0.0, 0.0, 5.0, 3.0));
        let union = boolean_regions(BooleanOp::Union, &a, &a, &tol).expect("union");
        let inter = boolean_regions(BooleanOp::Intersection, &a, &a, &tol).expect("intersection");
        let diff = boolean_regions(BooleanOp::Difference, &a, &a, &tol).expect("difference");
        assert!((areas(&union) - 15.0).abs() < TIGHT, "{}", areas(&union));
        assert!((areas(&inter) - 15.0).abs() < TIGHT, "{}", areas(&inter));
        assert!(diff.is_empty(), "difference area {}", areas(&diff));
    }

    #[test]
    fn try_new_rejects_a_hole_outside_the_outer() {
        let err = Region::try_new(rect(0.0, 0.0, 2.0, 2.0), vec![rect(5.0, 5.0, 1.0, 1.0)], &Tolerance::default()).unwrap_err();
        assert!(err.to_string().contains("outside"), "{err}");
    }

    fn min_corner(region: &Region) -> (f64, f64) {
        let bounds = region.outer().bounding_box().unwrap();
        (bounds.min().x(), bounds.min().y())
    }

    #[test]
    fn near_equal_region_areas_order_by_rounded_corner_not_ulp_area() {
        let tol = Tolerance::default();
        // Widening a 10 mm square by 1e-9 mm changes its area by 1e-8, inside 1e-9 * 100.
        let east = region_of(rect(10.0, 0.0, 10.0, 10.0));
        let west = region_of(rect(-10.0, 0.0, 10.0 + 1e-9, 10.0));
        let xs = |regions: Vec<Region>| sort_regions(regions, &tol).unwrap().into_iter().map(|region| min_corner(&region).0).collect::<Vec<_>>();
        assert_eq!(xs(vec![east.clone(), west.clone()]), vec![-10.0, 10.0]);
        assert_eq!(xs(vec![west.clone(), east.clone()]), vec![-10.0, 10.0]);

        for ulps in [-8_i32, -1, 1, 8] {
            let mut width = 10.0_f64;
            for _ in 0..ulps.unsigned_abs() {
                width = if ulps > 0 { width.next_up() } else { width.next_down() };
            }
            let nudged = region_of(rect(-10.0, 0.0, width, 10.0));
            assert_eq!(xs(vec![east.clone(), nudged.clone()]), vec![-10.0, 10.0], "ulps {ulps}");
            assert_eq!(xs(vec![nudged, east.clone()]), vec![-10.0, 10.0], "ulps {ulps}");
        }

        // A tenth of a millimetre is a real area gap, so the larger region stays first.
        let bigger = region_of(rect(10.0, 0.0, 10.1, 10.0));
        let clear = xs(vec![west.clone(), bigger]);
        assert!((clear[0] - 10.0).abs() < 1e-9, "{clear:?}");
        assert!((clear[1] + 10.0).abs() < 1e-9, "{clear:?}");

        // Mirror pockets: the slightly larger area still loses to the lower corner.
        let low_x = region_of(rect(-20.0, -8.0, 10.0, 10.0));
        let low_y = region_of(rect(-8.0, -20.0, 10.0 + 1e-9, 10.0));
        let corners = |regions: Vec<Region>| sort_regions(regions, &tol).unwrap().into_iter().map(|region| min_corner(&region)).collect::<Vec<_>>();
        for pair in [vec![low_x.clone(), low_y.clone()], vec![low_y, low_x]] {
            let ordered = corners(pair);
            assert!((ordered[0].0 + 20.0).abs() < 1e-9 && (ordered[0].1 + 8.0).abs() < 1e-9, "{ordered:?}");
            assert!((ordered[1].0 + 8.0).abs() < 1e-9 && (ordered[1].1 + 20.0).abs() < 1e-9, "{ordered:?}");
        }
    }
}
