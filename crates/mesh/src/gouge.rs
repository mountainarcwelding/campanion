//! Gouge check for a straight tool-tip path.
//!
//! Each segment is sampled at least every `sample_step` millimetres, including both ends. A sample
//! whose Z is more than `tolerance` below [`crate::DropCutter::tip_z`] is a gouge. A miss (`None`)
//! does not touch the mesh. `depth` is the distance below that contact Z.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::{DropCutter, Result, Vec3, finite_vec, invalid};

/// Most tool-tip samples accepted in one check, across every segment.
const MAX_GOUGE_SAMPLES: usize = 50_000_000;

/// One tool-tip sample that cuts into the mesh.
///
/// `depth` is millimetres below the drop-cutter contact, not the amount past `tolerance`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gouge {
    pub at: Vec3,
    pub depth: f64,
}

/// Sample `segments` and return `(total gouges, up to max_reports deepest)`.
///
/// `sample_step` must be finite and positive. `tolerance` must be finite and non-negative. A tip
/// at `contact - tolerance` is allowed. Segment coordinates must be finite. More than
/// [`MAX_GOUGE_SAMPLES`] samples is an error. `max_reports` of zero counts gouges and stores none.
pub fn check_gouges(dc: &DropCutter, segments: &[(Vec3, Vec3)], sample_step: f64, tolerance: f64, max_reports: usize) -> Result<(usize, Vec<Gouge>)> {
    if !sample_step.is_finite() || sample_step <= 0.0 {
        return Err(invalid("sample_step must be finite and positive"));
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(invalid("tolerance must be finite and non-negative"));
    }
    let mut total_samples = 0usize;
    for &(start, end) in segments {
        let count = samples_on(start, end, sample_step)?;
        total_samples = match total_samples.checked_add(count) {
            Some(sum) if sum <= MAX_GOUGE_SAMPLES => sum,
            _ => return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}"))),
        };
    }

    let mut total = 0usize;
    let mut kept = BinaryHeap::new();
    for &(start, end) in segments {
        let count = samples_on(start, end, sample_step)?;
        for index in 0..count {
            let point = sample_at(start, end, index, count)?;
            let Some(contact) = dc.tip_z(point.x, point.y) else {
                continue;
            };
            if !contact.is_finite() {
                return Err(invalid("drop-cutter tip is not finite"));
            }
            let depth = contact - point.z;
            if !depth.is_finite() {
                return Err(invalid("gouge depth is not finite"));
            }
            if depth > tolerance {
                total += 1;
                keep(&mut kept, max_reports, point, depth, total as u64);
            }
        }
    }
    // `Ord` is inverted so the heap's ascending order is deepest first.
    let ranked = kept.into_sorted_vec();
    Ok((total, ranked.into_iter().map(|ranked| Gouge { at: ranked.at, depth: ranked.depth }).collect()))
}

/// `ceil(length / step) + 1` samples, or one sample when `length` is zero.
fn samples_on(start: Vec3, end: Vec3, step: f64) -> Result<usize> {
    if !finite_vec(start) || !finite_vec(end) {
        return Err(invalid("segment coordinate is not finite"));
    }
    let length = end.sub(start).length();
    if !length.is_finite() {
        return Err(invalid("segment length is not finite"));
    }
    if length == 0.0 {
        return Ok(1);
    }
    let mut intervals = (length / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
    }
    // `ceil` can leave `length / intervals` a fraction over `step`.
    let mut guard = 0u32;
    while length > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(invalid("gouge sample spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_GOUGE_SAMPLES as f64 {
        return Err(invalid(format!("too many gouge samples: limit is {MAX_GOUGE_SAMPLES}")));
    }
    Ok(count as usize)
}

fn sample_at(start: Vec3, end: Vec3, index: usize, count: usize) -> Result<Vec3> {
    if count <= 1 || index == 0 {
        return Ok(start);
    }
    if index + 1 >= count {
        return Ok(end);
    }
    let t = index as f64 / (count - 1) as f64;
    let point = Vec3::new(start.x + (end.x - start.x) * t, start.y + (end.y - start.y) * t, start.z + (end.z - start.z) * t);
    if finite_vec(point) { Ok(point) } else { Err(invalid("gouge sample is not finite")) }
}

/// Shallower gouges compare as greater so a full heap drops them first.
struct Ranked {
    depth: f64,
    seq: u64,
    at: Vec3,
}

impl PartialEq for Ranked {
    fn eq(&self, other: &Self) -> bool {
        self.depth.total_cmp(&other.depth) == Ordering::Equal && self.seq == other.seq
    }
}

impl Eq for Ranked {}

impl PartialOrd for Ranked {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Ranked {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.depth.total_cmp(&self.depth) {
            Ordering::Equal => self.seq.cmp(&other.seq),
            ord => ord,
        }
    }
}

fn keep(kept: &mut BinaryHeap<Ranked>, max_reports: usize, at: Vec3, depth: f64, seq: u64) {
    if max_reports == 0 {
        return;
    }
    let ranked = Ranked { depth, seq, at };
    if kept.len() < max_reports {
        kept.push(ranked);
        return;
    }
    let replace = kept.peek().is_some_and(|shallowest| depth.total_cmp(&shallowest.depth) == Ordering::Greater);
    if replace {
        kept.pop();
        kept.push(ranked);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cutter, Mesh, MeshError, RasterAngle, RasterParams, Triangle, parallel_finish};
    use proptest::prelude::*;

    fn plane_at(z: f64) -> Mesh {
        let s = 40.0;
        Mesh::new(vec![
            Triangle { a: Vec3::new(-s, -s, z), b: Vec3::new(s, -s, z), c: Vec3::new(s, s, z) },
            Triangle { a: Vec3::new(-s, -s, z), b: Vec3::new(s, s, z), c: Vec3::new(-s, s, z) },
        ])
        .expect("plane")
    }

    fn flat_index(mesh: &Mesh) -> DropCutter<'_> {
        DropCutter::new(mesh, Cutter::new_flat(1.0).expect("flat")).expect("index")
    }

    fn close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() <= 1e-9, "{actual} != {expected}");
    }

    #[test]
    fn path_on_the_surface_has_no_gouge() {
        let mesh = plane_at(5.0);
        let dc = flat_index(&mesh);
        let z = dc.tip_z(0.0, 0.0).expect("contact");
        close(z, 5.0);
        let path = [(Vec3::new(0.0, 0.0, z), Vec3::new(2.0, 0.5, z)), (Vec3::new(2.0, 0.5, z), Vec3::new(3.5, -0.5, z))];
        let (count, gouges) = check_gouges(&dc, &path, 0.25, 0.01, 8).expect("check");
        assert_eq!(count, 0);
        assert!(gouges.is_empty());
        let above = [(Vec3::new(0.0, 0.0, z + 1.0), Vec3::new(1.0, 0.0, z + 1.0))];
        assert_eq!(check_gouges(&dc, &above, 0.5, 0.0, 4).expect("above").0, 0);
        let miss = Vec3::new(1_000.0, 1_000.0, -100.0);
        assert_eq!(check_gouges(&dc, &[(miss, miss)], 1.0, 0.0, 4).expect("miss").0, 0);
        assert_eq!(check_gouges(&dc, &[], 1.0, 0.0, 4).expect("empty"), (0, Vec::new()));
    }

    #[test]
    fn lowering_one_point_reports_that_depth() {
        let mesh = plane_at(5.0);
        let dc = flat_index(&mesh);
        let z = dc.tip_z(0.0, 1.0).expect("contact");
        let lowered = Vec3::new(1.0, 1.0, z - 0.1);
        let segments = [(Vec3::new(0.0, 1.0, z), lowered), (lowered, Vec3::new(2.0, 1.0, z))];
        let (count, gouges) = check_gouges(&dc, &segments, 0.25, 0.01, 8).expect("check");
        assert!(count >= 1, "count {count}");
        let worst = gouges.first().expect("report");
        close(worst.depth, 0.1);
        close(worst.at.x, 1.0);
        close(worst.at.y, 1.0);
        close(worst.at.z, z - 0.1);
        assert!(gouges.iter().all(|gouge| gouge.depth <= worst.depth + 1e-12));

        let on_limit = Vec3::new(0.0, 0.0, z - 0.1);
        assert_eq!(check_gouges(&dc, &[(on_limit, on_limit)], 1.0, 0.1, 4).expect("equal").0, 0);
        let (just_over, reports) = check_gouges(&dc, &[(on_limit, on_limit)], 1.0, 0.1 - 1e-9, 4).expect("over");
        assert_eq!(just_over, 1);
        close(reports.first().expect("one").depth, 0.1);
        let (hidden, none) = check_gouges(&dc, &[(on_limit, on_limit)], 1.0, 0.0, 0).expect("count only");
        assert_eq!(hidden, 1);
        assert!(none.is_empty());
    }

    #[test]
    fn interior_samples_see_a_bump_and_the_deepest_are_kept() {
        let mut triangles = Vec::new();
        triangles.extend(quad(5.0, -10.0, -10.0, 10.0, 10.0));
        triangles.extend(quad(6.0, 0.0, -1.0, 1.0, 1.0));
        let mesh = Mesh::new(triangles).expect("mesh");
        let dc = DropCutter::new(&mesh, Cutter::new_flat(0.2).expect("flat")).expect("index");
        let z = dc.tip_z(-3.0, 0.0).expect("plane");
        close(z, 5.0);
        let over = dc.tip_z(0.5, 0.0).expect("bump");
        close(over, 6.0);
        let segment = [(Vec3::new(-3.0, 0.0, z), Vec3::new(3.0, 0.0, z))];
        let (count, gouges) = check_gouges(&dc, &segment, 0.5, 0.01, 2).expect("check");
        assert!(count > 2, "count {count}");
        assert_eq!(gouges.len(), 2);
        close(gouges[0].depth, 1.0);
        close(gouges[1].depth, 1.0);
        assert!((0.0..=1.0).contains(&gouges[0].at.x), "{}", gouges[0].at.x);
        assert!(gouges[0].depth >= gouges[1].depth);
    }

    #[test]
    fn rejects_bad_inputs_and_too_many_samples() {
        let mesh = plane_at(5.0);
        let dc = flat_index(&mesh);
        let origin = Vec3::new(0.0, 0.0, 5.0);
        let end = Vec3::new(1.0, 0.0, 5.0);
        for step in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let err = check_gouges(&dc, &[(origin, end)], step, 0.01, 1).expect_err("step");
            assert!(matches!(err, MeshError::InvalidInput(_)), "{err}");
        }
        for tolerance in [-0.01, f64::NAN, f64::NEG_INFINITY] {
            let err = check_gouges(&dc, &[(origin, end)], 1.0, tolerance, 1).expect_err("tolerance");
            assert!(matches!(err, MeshError::InvalidInput(_)), "{err}");
        }
        let bad = Vec3::new(f64::NAN, 0.0, 0.0);
        assert!(matches!(check_gouges(&dc, &[(bad, end)], 1.0, 0.0, 1), Err(MeshError::InvalidInput(_))));
        let long = Vec3::new(30_000_000.0, 0.0, 0.0);
        let err = check_gouges(&dc, &[(origin, long), (origin, long)], 1.0, 0.0, 1).expect_err("cap");
        assert!(err.to_string().contains("50000000"), "{err}");
        let huge = Vec3::new(6_000_000.0, 0.0, 0.0);
        let err = check_gouges(&dc, &[(origin, huge)], 0.1, 0.0, 1).expect_err("one segment");
        assert!(err.to_string().contains("50000000"), "{err}");
    }

    fn quad(z: f64, x0: f64, y0: f64, x1: f64, y1: f64) -> [Triangle; 2] {
        [
            Triangle { a: Vec3::new(x0, y0, z), b: Vec3::new(x1, y0, z), c: Vec3::new(x1, y1, z) },
            Triangle { a: Vec3::new(x0, y0, z), b: Vec3::new(x1, y1, z), c: Vec3::new(x0, y1, z) },
        ]
    }

    fn same_point(a: Vec3, b: Vec3) -> bool {
        a.x == b.x && a.y == b.y && a.z == b.z
    }

    fn segments_of(lines: &[Vec<Vec3>]) -> Vec<(Vec3, Vec3)> {
        let mut out = Vec::new();
        for line in lines {
            if line.len() < 2 {
                if let Some(only) = line.first() {
                    out.push((*only, *only));
                }
                continue;
            }
            for pair in line.windows(2) {
                if let (Some(start), Some(end)) = (pair.first(), pair.get(1)) {
                    out.push((*start, *end));
                }
            }
        }
        out
    }

    fn push_contact(contact: &mut Vec<Vec3>, dc: &DropCutter, point: Vec3) {
        if contact.iter().any(|kept| same_point(*kept, point)) {
            return;
        }
        if let Some(z) = dc.tip_z(point.x, point.y)
            && (point.z - z).abs() <= 1e-8
        {
            contact.push(point);
        }
    }

    fn mild_triangle(coords: (f64, f64, f64, f64, f64, f64, f64, f64, f64)) -> Option<Triangle> {
        let (ax, ay, az, bx, by, bz, cx, cy, cz) = coords;
        let triangle = Triangle { a: Vec3::new(ax, ay, az), b: Vec3::new(bx, by, bz), c: Vec3::new(cx, cy, cz) };
        let normal = triangle.normal()?;
        let up = if normal.z < 0.0 { normal.scale(-1.0) } else { normal };
        if up.z < 0.25 {
            return None;
        }
        for (p, q) in [(triangle.a, triangle.b), (triangle.b, triangle.c), (triangle.c, triangle.a)] {
            let horiz = (p.x - q.x).hypot(p.y - q.y);
            if horiz < 1e-3 || (p.z - q.z).abs() > 20.0 * horiz {
                return None;
            }
        }
        Some(triangle)
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(24))]

        #[test]
        fn parallel_finish_does_not_gouge_and_a_lowered_point_is_reported(
            raw in proptest::collection::vec(
                (-1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10, -1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10, -1.0f64..1.0, -1.0f64..1.0, 0.02f64..0.10),
                1..4usize,
            ),
            radius in 0.5f64..1.5,
            kind in 0..3u8,
            corner_frac in 0.25f64..0.75,
            stepover in 0.6f64..1.2,
            raster_step in 0.6f64..1.2,
            tolerance in 0.06f64..0.15,
            along_y in any::<bool>(),
            zigzag in any::<bool>(),
            which in 0usize..64,
            extra in 0.02f64..0.4,
        ) {
            let triangles: Vec<Triangle> = raw.iter().copied().filter_map(mild_triangle).collect();
            prop_assume!(!triangles.is_empty());
            let mesh = Mesh::new(triangles).expect("mesh");
            prop_assume!(!mesh.triangles().is_empty());
            let cutter = match kind {
                0 => Cutter::new_flat(radius).expect("flat"),
                1 => Cutter::new_ball(radius).expect("ball"),
                _ => Cutter::new_bull(radius, radius * corner_frac).expect("bull"),
            };
            let dc = DropCutter::new(&mesh, cutter).expect("index");
            let params = RasterParams {
                stepover,
                sample_step: raster_step,
                tolerance,
                angle: if along_y { RasterAngle::AlongY } else { RasterAngle::AlongX },
                floor_z: 0.0,
                zigzag,
            };
            let lines = parallel_finish(&dc, (-1.2, -1.2), (1.2, 1.2), &params).expect("finish");
            prop_assume!(lines.iter().any(|line| !line.is_empty()));
            let mut segments = segments_of(&lines);
            let step = 0.25_f64.min(raster_step);
            let (count, gouges) = check_gouges(&dc, &segments, step, tolerance, 8).expect("check");
            prop_assert!(count == 0, "gouges {:?}", gouges);

            let mut contact = Vec::new();
            for &(start, end) in &segments {
                push_contact(&mut contact, &dc, start);
                push_contact(&mut contact, &dc, end);
            }
            prop_assume!(!contact.is_empty());
            let point = contact[which % contact.len()];
            let drop = tolerance + extra;
            for (start, end) in &mut segments {
                if same_point(*start, point) {
                    start.z -= drop;
                }
                if same_point(*end, point) {
                    end.z -= drop;
                }
            }
            let (lowered_count, lowered) = check_gouges(&dc, &segments, step, tolerance, 100_000).expect("lowered");
            prop_assert!(lowered_count >= 1, "no gouge after lowering {:?} by {}", point, drop);
            let reported = lowered.iter().any(|gouge| {
                (gouge.at.x - point.x).abs() <= 1e-6
                    && (gouge.at.y - point.y).abs() <= 1e-6
                    && (gouge.at.z - (point.z - drop)).abs() <= 1e-5
                    && (gouge.depth - drop).abs() <= 1e-5
            });
            prop_assert!(reported, "point {:?} lowered by {} was not reported in {:?}", point, drop, lowered);
        }
    }
}
