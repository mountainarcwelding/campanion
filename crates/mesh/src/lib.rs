//! Triangle meshes for 3-axis work.
//!
//! Units are millimetres. The frame is right-handed with Z up. STL is read and written from
//! bytes ([`parse_stl`], [`write_stl_binary`]); this crate does not touch the filesystem, so it
//! builds for `wasm32`. [`check_gouges`] samples a tool-tip path against a [`DropCutter`].

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

mod dropcutter;
mod gouge;
mod raster;
mod scallop;
mod stl;
mod waterline;
mod zlevel;

pub use dropcutter::{Cutter, DropCutter, drop_cutter_triangle};
pub use gouge::{Gouge, check_gouges};
pub use raster::{RasterAngle, RasterParams, parallel_finish};
pub use scallop::scallop_stepover;
pub use stl::{parse_stl, write_stl_binary};
pub use waterline::{WaterlineLevel, WaterlineParams, waterline};
pub use zlevel::{ZLevel, ZLevelParams, zlevel_rough};

/// Crate version from the package manifest.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Most triangles accepted in one mesh.
///
/// Larger inputs are rejected so a hostile file cannot force an unbounded allocation.
pub const MAX_TRIANGLES: usize = 5_000_000;

/// Triangles with area below this, in square millimetres, are degenerate.
const MIN_TRIANGLE_AREA: f64 = 1e-12;

/// Result type for mesh operations.
pub type Result<T> = std::result::Result<T, MeshError>;

/// Failures while building or reading a mesh.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum MeshError {
    /// Coordinates, area, or a triangle count this crate rejects.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// Bytes are not a readable STL mesh. ASCII failures name the line number.
    #[error("parse error: {0}")]
    Parse(String),
}

pub(crate) fn invalid(message: impl Into<String>) -> MeshError {
    MeshError::InvalidInput(message.into())
}

pub(crate) fn parse_error(message: impl Into<String>) -> MeshError {
    MeshError::Parse(message.into())
}

pub(crate) fn parse_at(line: usize, message: impl std::fmt::Display) -> MeshError {
    parse_error(format!("line {line}: {message}"))
}

pub(crate) fn check_triangle_count(count: usize) -> Result<()> {
    if count > MAX_TRIANGLES { Err(invalid(format!("too many triangles: {count} > {MAX_TRIANGLES}"))) } else { Ok(()) }
}

fn finite_vec(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

/// A position or free vector in millimetres.
///
/// Fields are public so callers can build values directly. [`Mesh::new`] is what rejects
/// non-finite coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn add(self, other: Self) -> Self {
        Self { x: self.x + other.x, y: self.y + other.y, z: self.z + other.z }
    }

    pub const fn sub(self, other: Self) -> Self {
        Self { x: self.x - other.x, y: self.y - other.y, z: self.z - other.z }
    }

    pub const fn scale(self, factor: f64) -> Self {
        Self { x: self.x * factor, y: self.y * factor, z: self.z * factor }
    }

    pub const fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Right-handed cross product.
    pub const fn cross(self, other: Self) -> Self {
        Self { x: self.y * other.z - self.z * other.y, y: self.z * other.x - self.x * other.z, z: self.x * other.y - self.y * other.x }
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y).hypot(self.z)
    }

    /// Unit vector in the same direction, or `None` when the length is zero or not finite.
    pub fn normalized(self) -> Option<Self> {
        let len = self.length();
        if !len.is_finite() || len <= 0.0 { None } else { Some(self.scale(1.0 / len)) }
    }
}

/// One triangle. Vertex order follows the right-hand rule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Triangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
}

impl Triangle {
    /// Unit normal `(b - a) × (c - a)`.
    ///
    /// `None` when the area is not finite, or when it is below 1e-12 mm².
    pub fn normal(self) -> Option<Vec3> {
        let cross = self.b.sub(self.a).cross(self.c.sub(self.a));
        match triangle_area_from_cross(cross) {
            Some(area) if area >= MIN_TRIANGLE_AREA => cross.normalized(),
            _ => None,
        }
    }

    /// Axis-aligned bounds of the three vertices.
    pub fn bbox(self) -> Aabb {
        Aabb::from_point(self.a).include(self.b).include(self.c)
    }

    fn area(self) -> Option<f64> {
        triangle_area_from_cross(self.b.sub(self.a).cross(self.c.sub(self.a)))
    }
}

fn triangle_area_from_cross(cross: Vec3) -> Option<f64> {
    let area = 0.5 * cross.length();
    if area.is_finite() { Some(area) } else { None }
}

/// Inclusive axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn from_point(point: Vec3) -> Self {
        Self { min: point, max: point }
    }

    /// `None` when `points` is empty.
    pub fn from_points(points: impl IntoIterator<Item = Vec3>) -> Option<Self> {
        let mut points = points.into_iter();
        let first = points.next()?;
        Some(points.fold(Self::from_point(first), Self::include))
    }

    fn include(self, point: Vec3) -> Self {
        Self {
            min: Vec3::new(self.min.x.min(point.x), self.min.y.min(point.y), self.min.z.min(point.z)),
            max: Vec3::new(self.max.x.max(point.x), self.max.y.max(point.y), self.max.z.max(point.z)),
        }
    }

    pub fn union(self, other: Self) -> Self {
        self.include(other.min).include(other.max)
    }

    /// `x` and `y` lie inside the box, ignoring Z. Edges are inclusive.
    pub fn contains_xy(self, x: f64, y: f64) -> bool {
        x >= self.min.x && x <= self.max.x && y >= self.min.y && y <= self.max.y
    }

    pub fn min(self) -> Vec3 {
        self.min
    }

    pub fn max(self) -> Vec3 {
        self.max
    }
}

/// A triangle soup in millimetres.
///
/// Degenerate triangles are dropped at construction and counted by [`Mesh::degenerate_dropped`].
/// The kept triangles stay in input order.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    triangles: Vec<Triangle>,
    degenerate_dropped: usize,
}

impl Mesh {
    /// Rejects non-finite coordinates, non-finite areas, and more than [`MAX_TRIANGLES`] triangles.
    ///
    /// Finite triangles whose area is below 1e-12 are removed. An empty input, or an input that
    /// becomes empty after that, is a valid mesh; [`Mesh::bounds`] then fails.
    pub fn new(triangles: Vec<Triangle>) -> Result<Self> {
        check_triangle_count(triangles.len())?;
        let mut kept = Vec::new();
        let mut degenerate_dropped = 0usize;
        for triangle in triangles {
            if !finite_vec(triangle.a) || !finite_vec(triangle.b) || !finite_vec(triangle.c) {
                return Err(invalid("triangle coordinate is not finite"));
            }
            match triangle.area() {
                None => return Err(invalid("triangle area is not finite")),
                Some(area) if area < MIN_TRIANGLE_AREA => degenerate_dropped += 1,
                Some(_) => kept.push(triangle),
            }
        }
        Ok(Self { triangles: kept, degenerate_dropped })
    }

    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    pub fn degenerate_dropped(&self) -> usize {
        self.degenerate_dropped
    }

    /// Bounds of the kept triangles.
    pub fn bounds(&self) -> Result<Aabb> {
        let mut tris = self.triangles.iter();
        let Some(first) = tris.next() else {
            return Err(invalid("empty mesh"));
        };
        let mut bounds = first.bbox();
        for triangle in tris {
            bounds = bounds.union(triangle.bbox());
        }
        Ok(bounds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(a: Vec3, b: Vec3, c: Vec3) -> Triangle {
        Triangle { a, b, c }
    }

    #[test]
    fn vec3_arithmetic_matches_a_3_4_12_vector() {
        let v = Vec3::new(3.0, 0.0, 0.0);
        let w = Vec3::new(0.0, 4.0, 0.0);
        let up = Vec3::new(0.0, 0.0, 12.0);
        assert_eq!(v.add(w), Vec3::new(3.0, 4.0, 0.0));
        assert_eq!(v.sub(w), Vec3::new(3.0, -4.0, 0.0));
        assert_eq!(v.scale(2.0), Vec3::new(6.0, 0.0, 0.0));
        assert!(v.dot(w).abs() < 1e-12);
        assert_eq!(v.cross(w), Vec3::new(0.0, 0.0, 12.0));
        let long = v.add(w).add(up);
        assert!((long.length() - 13.0).abs() < 1e-12);
        let unit = long.normalized().expect("nonzero");
        assert!((unit.length() - 1.0).abs() < 1e-12);
        assert!((unit.x - 3.0 / 13.0).abs() < 1e-12);
        assert!(Vec3::new(0.0, 0.0, 0.0).normalized().is_none());
        assert!(Vec3::new(f64::INFINITY, 0.0, 0.0).normalized().is_none());
    }

    #[test]
    fn normal_is_the_unit_right_handed_normal() {
        let flat = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 3.0, 0.0));
        let normal = flat.normal().expect("area 3");
        assert!((normal.x).abs() < 1e-12);
        assert!((normal.y).abs() < 1e-12);
        assert!((normal.z - 1.0).abs() < 1e-12);
        let side = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        let side_n = side.normal().expect("side");
        assert!(side_n.x.abs() < 1e-12);
        assert!((side_n.y + 1.0).abs() < 1e-12);
        assert!(side_n.z.abs() < 1e-12);
        let degenerate = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 2.0));
        assert!(degenerate.normal().is_none());
    }

    #[test]
    fn area_threshold_is_one_e_minus_12() {
        let dropped = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1e-13, 0.0));
        let kept = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1e-9, 0.0));
        assert!(dropped.area().expect("flat area") < MIN_TRIANGLE_AREA);
        assert!(dropped.normal().is_none());
        assert!(kept.area().expect("kept area") >= MIN_TRIANGLE_AREA);
        assert!(kept.normal().is_some());
        let mesh = Mesh::new(vec![dropped, kept]).expect("finite");
        assert_eq!(mesh.triangles().len(), 1);
        assert_eq!(mesh.degenerate_dropped(), 1);
        assert_eq!(mesh.triangles()[0], kept);
    }

    #[test]
    fn aabb_from_points_union_and_contains_xy() {
        assert!(Aabb::from_points([]).is_none());
        let box_a = Aabb::from_points([Vec3::new(1.0, 2.0, 3.0), Vec3::new(-4.0, 5.0, -6.0)]).expect("points");
        assert_eq!(box_a.min(), Vec3::new(-4.0, 2.0, -6.0));
        assert_eq!(box_a.max(), Vec3::new(1.0, 5.0, 3.0));
        assert!(box_a.contains_xy(-4.0, 2.0));
        assert!(box_a.contains_xy(1.0, 5.0));
        assert!(box_a.contains_xy(0.0, 4.0));
        assert!(!box_a.contains_xy(-4.1, 2.0));
        assert!(!box_a.contains_xy(0.0, 5.1));
        let box_b = Aabb::from_points([Vec3::new(10.0, -1.0, 0.0)]).expect("one point");
        let union = box_a.union(box_b);
        assert_eq!(union.min(), Vec3::new(-4.0, -1.0, -6.0));
        assert_eq!(union.max(), Vec3::new(10.0, 5.0, 3.0));
        let triangle = tri(Vec3::new(1.0, 2.0, 3.0), Vec3::new(-4.0, 5.0, -6.0), Vec3::new(7.0, -8.0, 9.0));
        assert_eq!(triangle.bbox().min(), Vec3::new(-4.0, -8.0, -6.0));
        assert_eq!(triangle.bbox().max(), Vec3::new(7.0, 5.0, 9.0));
    }

    #[test]
    fn mesh_rejects_non_finite_coordinates_and_empty_bounds() {
        let nan = tri(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let inf = tri(Vec3::new(0.0, 0.0, f64::INFINITY), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert!(matches!(Mesh::new(vec![nan]), Err(MeshError::InvalidInput(_))));
        assert!(matches!(Mesh::new(vec![inf]), Err(MeshError::InvalidInput(_))));
        let empty = Mesh::new(vec![]).expect("empty is allowed");
        assert!(empty.triangles().is_empty());
        assert_eq!(empty.degenerate_dropped(), 0);
        assert!(matches!(empty.bounds(), Err(MeshError::InvalidInput(_))));
        let only_degenerate = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
        let dropped = Mesh::new(vec![only_degenerate]).expect("dropped");
        assert!(dropped.bounds().is_err());
        assert_eq!(dropped.degenerate_dropped(), 1);
        let huge = tri(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1e308, 0.0, 0.0), Vec3::new(0.0, 1e308, 0.0));
        assert!(matches!(Mesh::new(vec![huge]), Err(MeshError::InvalidInput(_))));
    }

    #[test]
    fn triangle_count_limit() {
        assert!(check_triangle_count(MAX_TRIANGLES).is_ok());
        let err = check_triangle_count(MAX_TRIANGLES + 1).expect_err("over the cap");
        assert!(err.to_string().contains("5000000"), "{err}");
    }

    #[test]
    fn error_display() {
        assert_eq!(MeshError::InvalidInput("nan".into()).to_string(), "invalid input: nan");
        assert_eq!(MeshError::Parse("line 3: nope".into()).to_string(), "parse error: line 3: nope");
    }
}
