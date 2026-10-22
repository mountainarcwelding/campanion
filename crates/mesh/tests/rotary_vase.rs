//! Revolved vase for `examples/rotary.stl`.
//!
//! The wall is a foot, a 15 mm belly, a neck and a lip, spun about X and closed with flat
//! end caps. The committed file is the binary STL these bytes produce. Regenerate it by
//! replacing `examples/rotary.stl` with [`write_stl_binary`] of [`vase`].

use std::collections::BTreeMap;
use std::path::PathBuf;

use campanion_mesh::{Mesh, Triangle, Vec3, parse_stl, write_stl_binary};

const LENGTH_MM: f64 = 60.0;
const HALF_LENGTH_MM: f64 = 30.0;
const SEGMENTS: usize = 36;
const STATIONS: usize = 31;
const MAX_STL_BYTES: usize = 200 * 1024;

fn example_stl() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/rotary.stl")
}

fn station_x(index: usize) -> f64 {
    -HALF_LENGTH_MM + LENGTH_MM * (index as f64) / ((STATIONS - 1) as f64)
}

/// Radius of a unit gaussian bump.
fn gauss(t: f64, center: f64, width: f64) -> f64 {
    let u = (t - center) / width;
    (-u * u).exp()
}

/// Wall radius at axial position `x` in `-30..=30`.
///
/// The belly peak is 15 mm (30 mm diameter). The ends stay near 9 mm and the neck is narrower.
fn wall_radius(x: f64) -> f64 {
    let t = (x + HALF_LENGTH_MM) / LENGTH_MM;
    let foot = 8.5;
    let belly = 6.5 * gauss(t, 0.42, 0.20);
    let neck = 3.2 * gauss(t, 0.78, 0.07);
    let lip = 2.0 * gauss(t, 0.96, 0.05);
    foot + belly - neck + lip
}

fn ring_point(x: f64, radius: f64, index: usize) -> Vec3 {
    let theta = index as f64 * std::f64::consts::TAU / SEGMENTS as f64;
    Vec3::new(x, radius * theta.sin(), radius * theta.cos())
}

/// `positive_x` is the outward side of a flat cap. The start cap faces −X.
fn push_cap(triangles: &mut Vec<Triangle>, center: Vec3, ring: &[Vec3], positive_x: bool) {
    for index in 0..ring.len() {
        let current = ring[index];
        let next = ring[(index + 1) % ring.len()];
        let (b, c) = if positive_x { (next, current) } else { (current, next) };
        triangles.push(Triangle { a: center, b, c });
    }
}

fn vase() -> Result<Mesh, String> {
    let mut rings = Vec::with_capacity(STATIONS);
    for station in 0..STATIONS {
        let x = station_x(station);
        let radius = wall_radius(x);
        let mut ring = Vec::with_capacity(SEGMENTS);
        for index in 0..SEGMENTS {
            ring.push(ring_point(x, radius, index));
        }
        rings.push(ring);
    }
    let mut triangles = Vec::new();
    for station in 0..STATIONS - 1 {
        for index in 0..SEGMENTS {
            let next = (index + 1) % SEGMENTS;
            let a = rings[station][index];
            let b = rings[station + 1][index];
            let c = rings[station][next];
            let d = rings[station + 1][next];
            triangles.push(Triangle { a, b, c });
            triangles.push(Triangle { a: b, b: d, c });
        }
    }
    push_cap(&mut triangles, Vec3::new(station_x(0), 0.0, 0.0), &rings[0], false);
    push_cap(&mut triangles, Vec3::new(station_x(STATIONS - 1), 0.0, 0.0), &rings[STATIONS - 1], true);
    Mesh::new(triangles).map_err(|err| err.to_string())
}

fn point_key(point: Vec3) -> (u64, u64, u64) {
    (point.x.to_bits(), point.y.to_bits(), point.z.to_bits())
}

fn edge_key(left: Vec3, right: Vec3) -> ((u64, u64, u64), (u64, u64, u64)) {
    let a = point_key(left);
    let b = point_key(right);
    if a <= b { (a, b) } else { (b, a) }
}

fn cross(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ux = b.x - a.x;
    let uy = b.y - a.y;
    let uz = b.z - a.z;
    let vx = c.x - a.x;
    let vy = c.y - a.y;
    let vz = c.z - a.z;
    Vec3::new(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
}

#[test]
fn rotary_example_is_a_closed_vase_and_matches_the_stl() -> Result<(), String> {
    let mesh = vase()?;
    assert_eq!(mesh.degenerate_dropped(), 0);
    let triangles = mesh.triangles();
    assert_eq!(triangles.len(), (STATIONS - 1) * SEGMENTS * 2 + SEGMENTS * 2);

    let bounds = mesh.bounds().expect("bounds");
    let length = bounds.max().x - bounds.min().x;
    assert!((bounds.min().x + HALF_LENGTH_MM).abs() < 1e-9, "{}", bounds.min().x);
    assert!((bounds.max().x - HALF_LENGTH_MM).abs() < 1e-9, "{}", bounds.max().x);
    assert!((length - LENGTH_MM).abs() < 1e-9, "{length}");

    let mut max_radius = 0.0_f64;
    let mut min_wall = f64::MAX;
    let mut saw_start_cap = false;
    let mut saw_end_cap = false;
    for triangle in triangles {
        for point in [triangle.a, triangle.b, triangle.c] {
            let radial = point.y.hypot(point.z);
            if point.y.abs() > 1e-9 || point.z.abs() > 1e-9 {
                max_radius = max_radius.max(radial);
            }
            if point.x.abs() < HALF_LENGTH_MM - 1.0 {
                min_wall = min_wall.min(radial);
            }
            if point.y.abs() < 1e-12 && point.z.abs() < 1e-12 && (point.x + HALF_LENGTH_MM).abs() < 1e-9 {
                saw_start_cap = true;
            }
            if point.y.abs() < 1e-12 && point.z.abs() < 1e-12 && (point.x - HALF_LENGTH_MM).abs() < 1e-9 {
                saw_end_cap = true;
            }
        }
    }
    assert!(saw_start_cap && saw_end_cap, "end caps are open");
    assert!((14.5..=15.2).contains(&max_radius), "diameter {}", max_radius * 2.0);
    assert!(min_wall > 5.0, "neck radius {min_wall}");

    let start_cap = triangles
        .iter()
        .find(|triangle| [triangle.a, triangle.b, triangle.c].iter().any(|point| point.y.abs() < 1e-12 && point.z.abs() < 1e-12 && point.x < 0.0));
    let start_cap = start_cap.expect("start cap");
    assert!(cross(start_cap.a, start_cap.b, start_cap.c).x < 0.0, "start cap faces inward");
    let end_cap = triangles
        .iter()
        .find(|triangle| [triangle.a, triangle.b, triangle.c].iter().any(|point| point.y.abs() < 1e-12 && point.z.abs() < 1e-12 && point.x > 0.0));
    let end_cap = end_cap.expect("end cap");
    assert!(cross(end_cap.a, end_cap.b, end_cap.c).x > 0.0, "end cap faces inward");

    let body = &triangles[0];
    let midpoint = Vec3::new((body.a.x + body.b.x + body.c.x) / 3.0, (body.a.y + body.b.y + body.c.y) / 3.0, (body.a.z + body.b.z + body.c.z) / 3.0);
    let normal = cross(body.a, body.b, body.c);
    let outward = normal.y * midpoint.y + normal.z * midpoint.z;
    assert!(outward > 0.0, "wall faces inward");

    let mut edges = BTreeMap::new();
    for triangle in triangles {
        for (left, right) in [(triangle.a, triangle.b), (triangle.b, triangle.c), (triangle.c, triangle.a)] {
            *edges.entry(edge_key(left, right)).or_insert(0) += 1;
        }
    }
    assert!(edges.values().all(|count| *count == 2), "mesh is not closed");

    let bytes = write_stl_binary(&mesh);
    assert!(bytes.len() < MAX_STL_BYTES, "{} bytes", bytes.len());
    assert_eq!(bytes.len(), 84 + 50 * triangles.len());
    let parsed = parse_stl(&bytes).expect("binary stl");
    assert_eq!(parsed.triangles().len(), triangles.len());
    assert_eq!(parsed.degenerate_dropped(), 0);

    let path = example_stl();
    let on_disk = std::fs::read(&path).unwrap_or_else(|err| panic!("read {}: {err}", path.display()));
    assert_eq!(on_disk, bytes, "examples/rotary.stl drifted from crates/mesh/tests/rotary_vase.rs");
    Ok(())
}
