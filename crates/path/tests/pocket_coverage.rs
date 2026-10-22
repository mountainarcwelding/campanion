//! Coverage, gouge clearance, and continuity of offset-ring pockets.

use std::f64::consts::{PI, TAU};

use campanion_geom::{Arc, Containment, Contour, Line, Point, Region, Segment, Tolerance};
use campanion_path::{Move, PocketOp, PocketStrategy, Tool};

type Result<T> = campanion_path::Result<T>;

const DIAMETER: f64 = 6.0;
const STEP: f64 = 3.0;
const CUT_Z: f64 = -1.0;
const GRID: f64 = 0.25;
const SAMPLES: usize = 32;
const COVER_LIMIT: f64 = 3.0 + 1e-6;
const GOUGE_LIMIT: f64 = 3.0 - 1e-6;
const JOIN_LIMIT: f64 = 1e-9;

fn tolerance() -> Result<Tolerance> {
    Ok(Tolerance::new(1e-6, 1e-6)?)
}

fn line(x0: f64, y0: f64, x1: f64, y1: f64) -> Result<Segment> {
    Ok(Segment::Line(Line::new(Point::new(x0, y0)?, Point::new(x1, y1)?)?))
}

fn arc(cx: f64, cy: f64, radius: f64, start: f64, sweep: f64) -> Result<Segment> {
    Ok(Segment::Arc(Arc::new(Point::new(cx, cy)?, radius, start, sweep)?))
}

fn rectangle(width: f64, height: f64, tol: &Tolerance) -> Result<Contour> {
    let (x0, y0, x1, y1) = (-width / 2.0, -height / 2.0, width / 2.0, height / 2.0);
    Ok(Contour::try_new(vec![line(x0, y0, x1, y0)?, line(x1, y0, x1, y1)?, line(x1, y1, x0, y1)?, line(x0, y1, x0, y0)?], true, tol)?)
}

/// Stadium on the origin: overall `length` by `width`, with semicircular ends.
fn slot(length: f64, width: f64, tol: &Tolerance) -> Result<Contour> {
    let radius = width / 2.0;
    let centre = (length - width) / 2.0;
    Ok(Contour::try_new(
        vec![
            arc(centre, 0.0, radius, -PI / 2.0, PI)?,
            line(centre, radius, -centre, radius)?,
            arc(-centre, 0.0, radius, PI / 2.0, PI)?,
            line(-centre, -radius, centre, -radius)?,
        ],
        true,
        tol,
    )?)
}

fn circle(radius: f64, sweep: f64, tol: &Tolerance) -> Result<Contour> {
    Ok(Contour::try_new(vec![arc(0.0, 0.0, radius, 0.0, sweep)?], true, tol)?)
}

struct Cuts {
    segs: Vec<Segment>,
    samples: Vec<Point>,
    gap: f64,
}

fn sample_t(i: usize) -> f64 {
    let steps = u32::try_from(SAMPLES - 1).map_or(1.0, f64::from);
    u32::try_from(i).map_or(0.0, f64::from) / steps
}

fn push_samples(cuts: &mut Cuts, seg: Segment, mut at: impl FnMut(f64) -> Result<Point>) -> Result<()> {
    for i in 0..SAMPLES {
        cuts.samples.push(at(sample_t(i))?);
    }
    cuts.segs.push(seg);
    Ok(())
}

fn push_line(cuts: &mut Cuts, x0: f64, y0: f64, x1: f64, y1: f64) -> Result<()> {
    let seg = line(x0, y0, x1, y1)?;
    push_samples(cuts, seg, |t| Ok(Point::new(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)?))
}

/// CCW-positive arc with sweep in (0, 2π]. A move that ends where it starts is a full circle.
fn push_arc(cuts: &mut Cuts, x0: f64, y0: f64, x1: f64, y1: f64, cx: f64, cy: f64, ccw: bool) -> Result<()> {
    let radius = (x0 - cx).hypot(y0 - cy);
    let a0 = (y0 - cy).atan2(x0 - cx);
    let a1 = (y1 - cy).atan2(x1 - cx);
    let closed = (x0 - x1).hypot(y0 - y1) <= 1e-9;
    let mut sweep = (a1 - a0).rem_euclid(TAU);
    if !ccw {
        sweep = (TAU - sweep).rem_euclid(TAU);
    }
    if closed || sweep == 0.0 {
        sweep = TAU;
    }
    let start = (if ccw || closed { a0 } else { a0 - sweep }).rem_euclid(TAU);
    let seg = arc(cx, cy, radius, start, sweep)?;
    push_samples(cuts, seg, |t| {
        let ang = start + sweep * t;
        Ok(Point::new(cx + radius * ang.cos(), cy + radius * ang.sin())?)
    })
}

fn note_join(cuts: &mut Cuts, previous: &mut Option<(f64, f64, f64)>, rapid_since: &mut bool, start: (f64, f64, f64), end: (f64, f64, f64)) {
    if !*rapid_since && let Some((x, y, z)) = *previous {
        cuts.gap = cuts.gap.max((start.0 - x).hypot(start.1 - y).hypot(start.2 - z));
    }
    *rapid_since = false;
    *previous = Some(end);
}

fn cutting(moves: &[Move]) -> Result<Cuts> {
    let mut pos: Option<(f64, f64, f64)> = None;
    let mut cuts = Cuts { segs: Vec::new(), samples: Vec::new(), gap: 0.0 };
    let mut previous = None;
    let mut rapid_since = false;
    for mv in moves {
        let next = match mv {
            Move::Event(_) | Move::Dwell { .. } => None,
            Move::Rapid { to } => {
                rapid_since = true;
                Some((to.x(), to.y(), to.z()))
            }
            Move::Linear { to, .. } => {
                if let Some(start) = pos.filter(|start| (start.2 - CUT_Z).abs() <= 1e-12 && (to.z() - CUT_Z).abs() <= 1e-12) {
                    let end = (to.x(), to.y(), to.z());
                    note_join(&mut cuts, &mut previous, &mut rapid_since, start, end);
                    push_line(&mut cuts, start.0, start.1, end.0, end.1)?;
                }
                Some((to.x(), to.y(), to.z()))
            }
            Move::Arc { to, centre, ccw, .. } => {
                if let Some(start) = pos.filter(|start| (start.2 - CUT_Z).abs() <= 1e-12 && (to.z() - CUT_Z).abs() <= 1e-12) {
                    let end = (to.x(), to.y(), to.z());
                    note_join(&mut cuts, &mut previous, &mut rapid_since, start, end);
                    push_arc(&mut cuts, start.0, start.1, end.0, end.1, centre.x(), centre.y(), *ccw)?;
                }
                Some((to.x(), to.y(), to.z()))
            }
        };
        if let Some(next) = next {
            pos = Some(next);
        }
    }
    Ok(cuts)
}

fn nearest<'a>(segs: impl IntoIterator<Item = &'a Segment>, point: &Point) -> f64 {
    segs.into_iter().fold(f64::INFINITY, |best, seg| best.min(seg.distance_to_point(*point)))
}

fn needs_cover(outer: &Contour, holes: &[Contour], point: &Point, tol: &Tolerance) -> Result<bool> {
    if !matches!(outer.containment(*point, tol)?, Containment::Inside) {
        return Ok(false);
    }
    for hole in holes {
        if matches!(hole.containment(*point, tol)?, Containment::Inside | Containment::Boundary) {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Sharp concave corners are interior, but a round tool never puts a centre in
/// reach of them. Centres are grid points inside the outer contour, outside every
/// hole, and at least `radius - 1e-9` from every boundary. A point is reachable
/// within `radius - 0.2` of a centre, plus half a grid step so lattice snap does
/// not exempt a whole boundary skin. Reachable points must lie within `radius` of
/// a cut, and unreachable interior points must stay under 2%.
fn coverage(outer: &Contour, holes: &[Contour], segs: &[Segment], tol: &Tolerance, bounds: [f64; 4]) -> Result<(f64, f64, f64, f64)> {
    let [xmin, ymin, xmax, ymax] = bounds;
    let (nx, ny) = (((xmax - xmin) / GRID).round() as isize + 1, ((ymax - ymin) / GRID).round() as isize + 1);
    let mut interior = vec![false; (nx * ny) as usize];
    let mut centres: Vec<(isize, isize)> = Vec::new();
    for ix in 0..nx {
        for iy in 0..ny {
            let point = Point::new(xmin + ix as f64 * GRID, ymin + iy as f64 * GRID)?;
            if !needs_cover(outer, holes, &point, tol)? {
                continue;
            }
            interior[(ix * ny + iy) as usize] = true;
            let mut clear = nearest(outer.segments(), &point);
            for hole in holes {
                clear = clear.min(nearest(hole.segments(), &point));
            }
            if clear >= DIAMETER / 2.0 - 1e-9 {
                centres.push((ix, iy));
            }
        }
    }
    let reach = DIAMETER / 2.0 - 0.2 + GRID / 2.0;
    let (reach_sq, cells) = (reach * reach, (reach / GRID) as isize + 1);
    let mut reachable = vec![false; interior.len()];
    for (cx, cy) in centres {
        for dx in -cells..=cells {
            let ix = cx + dx;
            if !(0..nx).contains(&ix) {
                continue;
            }
            for dy in -cells..=cells {
                let iy = cy + dy;
                if (0..ny).contains(&iy) && (dx * dx + dy * dy) as f64 * GRID * GRID <= reach_sq {
                    reachable[(ix * ny + iy) as usize] = true;
                }
            }
        }
    }
    let (mut worst, mut at, mut inside, mut missed) = (0.0_f64, (0.0, 0.0), 0.0_f64, 0.0_f64);
    for ix in 0..nx {
        for iy in 0..ny {
            let idx = (ix * ny + iy) as usize;
            if reachable[idx] {
                let point = Point::new(xmin + ix as f64 * GRID, ymin + iy as f64 * GRID)?;
                let distance = nearest(segs, &point);
                if distance > worst {
                    worst = distance;
                    at = (point.x(), point.y());
                }
            }
            if interior[idx] {
                inside += 1.0;
                if !reachable[idx] {
                    missed += 1.0;
                }
            }
        }
    }
    Ok((worst, at.0, at.1, missed / inside))
}

fn clearance(outer: &Contour, holes: &[Contour], samples: &[Point]) -> f64 {
    let mut worst = f64::INFINITY;
    for point in samples {
        let mut distance = nearest(outer.segments(), point);
        for hole in holes {
            distance = distance.min(nearest(hole.segments(), point));
        }
        worst = worst.min(distance);
    }
    worst
}

fn check(name: &str, outer: &Contour, holes: &[Contour], tol: &Tolerance, bounds: [f64; 4]) -> Result<()> {
    let region = if holes.is_empty() { Region::from_contour(outer.clone(), tol)? } else { Region::try_new(outer.clone(), holes.to_vec(), tol)? };
    let pocket = PocketOp::new(Tool::Diameter { diameter: DIAMETER }, PocketStrategy::OffsetRings, STEP)
        .z(CUT_Z)
        .retract_z(2.0)
        .tolerance(*tol)
        .generate(&region)?;
    let cuts = cutting(pocket.toolpath().moves())?;
    let mut sample_off = 0.0_f64;
    for (i, point) in cuts.samples.iter().enumerate() {
        sample_off = sample_off.max(cuts.segs[i / SAMPLES].distance_to_point(*point));
    }
    let (cover, cover_x, cover_y, unreachable) = coverage(outer, holes, &cuts.segs, tol, bounds)?;
    let clear = clearance(outer, holes, &cuts.samples);
    assert!(
        sample_off <= 1e-6 && cover <= COVER_LIMIT && unreachable < 0.02 && clear >= GOUGE_LIMIT && cuts.gap <= JOIN_LIMIT,
        "{name}: sample {sample_off}, coverage {cover} at ({cover_x}, {cover_y}) limit {COVER_LIMIT}, unreachable {unreachable:.4} limit 0.02, clearance {clear} limit {GOUGE_LIMIT}, gap {} limit {JOIN_LIMIT}",
        cuts.gap
    );
    Ok(())
}

#[test]
fn pocket_rect_30x20() -> Result<()> {
    let tol = tolerance()?;
    let outer = rectangle(30.0, 20.0, &tol)?;
    check("30x20 rectangle", &outer, &[], &tol, [-15.0, -10.0, 15.0, 10.0])
}

#[test]
fn pocket_rect_40x30_hole() -> Result<()> {
    let tol = tolerance()?;
    let outer = rectangle(40.0, 30.0, &tol)?;
    let hole = circle(5.0, TAU, &tol)?;
    let centre = hole.containment(Point::new(0.0, 0.0)?, &tol)?;
    let material = hole.containment(Point::new(12.0, 0.0)?, &tol)?;
    assert!(matches!(centre, Containment::Inside), "hole centre {centre:?} material {material:?}");
    assert!(matches!(material, Containment::Outside), "material {material:?}");
    check("40x30 rectangle with 10 mm hole", &outer, &[hole], &tol, [-20.0, -15.0, 20.0, 15.0])
}

#[test]
fn pocket_slot_30x18() -> Result<()> {
    let tol = tolerance()?;
    let outer = slot(30.0, 18.0, &tol)?;
    check("rounded slot 30x18", &outer, &[], &tol, [-15.0, -9.0, 15.0, 9.0])
}

#[test]
fn pocket_slot_40x13() -> Result<()> {
    let tol = tolerance()?;
    let outer = slot(40.0, 13.0, &tol)?;
    check("rounded slot 40x13", &outer, &[], &tol, [-20.0, -6.5, 20.0, 6.5])
}
