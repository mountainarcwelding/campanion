//! Pose helpers for moving a resin part on the build plate.
//!
//! Empty [`MeshPose`] fields mean "leave this axis as imported." Position is the world location
//! of the axis-aligned bounding-box centre, in millimetres. Rotation is applied about that
//! centre, in degrees, in X-then-Y-then-Z order.

use campanion_mesh::{Mesh, Triangle, Vec3};

use crate::error::{ResinError, Result};

/// A typed or computed pose relative to the imported mesh.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MeshPose {
    /// World position of the bounding-box centre, millimetres. `None` keeps the imported centre.
    pub position_mm: [Option<f64>; 3],
    /// Rotation about the imported bounding-box centre, degrees. `None` is 0°.
    pub rotation_deg: [Option<f64>; 3],
}

impl MeshPose {
    /// No extra translation or rotation: the mesh stays as imported.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// True when every field is empty (the identity pose).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.position_mm.iter().all(Option::is_none) && self.rotation_deg.iter().all(Option::is_none)
    }
}

/// Round `value` to the nearest multiple of `step`.
pub fn snap_to_step(value: f64, step: f64) -> Result<f64> {
    if !value.is_finite() {
        return Err(ResinError::invalid("snap value is not finite"));
    }
    if !step.is_finite() || step <= 0.0 {
        return Err(ResinError::invalid("snap step must be greater than 0 mm"));
    }
    let units = value / step;
    let rounded = units.signum() * (units.abs() + 0.5).floor();
    let snapped = rounded * step;
    if !snapped.is_finite() {
        return Err(ResinError::invalid("snapped value is not finite"));
    }
    Ok(snapped)
}

/// Apply `pose` to `mesh`. An empty pose returns a clone of the original triangles.
pub fn apply_mesh_pose(mesh: &Mesh, pose: MeshPose) -> Result<Mesh> {
    if pose.position_mm.iter().flatten().any(|v| !v.is_finite()) || pose.rotation_deg.iter().flatten().any(|v| !v.is_finite()) {
        return Err(ResinError::invalid("pose values must be finite"));
    }
    let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let (min, max) = (bounds.min(), bounds.max());
    let center = Vec3::new((min.x + max.x) / 2.0, (min.y + max.y) / 2.0, (min.z + max.z) / 2.0);
    let rotation = [pose.rotation_deg[0].unwrap_or(0.0), pose.rotation_deg[1].unwrap_or(0.0), pose.rotation_deg[2].unwrap_or(0.0)];
    let needs_rotate = rotation.iter().any(|deg| *deg != 0.0);
    let needs_translate = pose.position_mm.iter().any(Option::is_some);
    if !needs_rotate && !needs_translate {
        return clone_mesh(mesh);
    }
    let triangles: Vec<Triangle> = mesh
        .triangles()
        .iter()
        .map(|tri| Triangle {
            a: transform_point(tri.a, center, rotation, pose.position_mm),
            b: transform_point(tri.b, center, rotation, pose.position_mm),
            c: transform_point(tri.c, center, rotation, pose.position_mm),
        })
        .collect();
    Mesh::new(triangles).map_err(|err| ResinError::invalid(format!("posed mesh: {err}")))
}

/// Move the bounding-box centre to X = 0, Y = 0, leaving Z as imported.
pub fn center_on_plate(mesh: &Mesh) -> Result<(Mesh, MeshPose)> {
    let pose = MeshPose { position_mm: [Some(0.0), Some(0.0), None], rotation_deg: [None, None, None] };
    let posed = apply_mesh_pose(mesh, pose)?;
    Ok((posed, pose))
}

/// Translate the mesh so its lowest point sits at Z = 0. XY is unchanged.
pub fn drop_to_plate(mesh: &Mesh) -> Result<(Mesh, MeshPose)> {
    let bounds = mesh.bounds().map_err(|err| ResinError::invalid(format!("part: {err}")))?;
    let (min, max) = (bounds.min(), bounds.max());
    let center_z = (min.z + max.z) / 2.0 - min.z;
    let pose = MeshPose { position_mm: [None, None, Some(center_z)], rotation_deg: [None, None, None] };
    let posed = apply_mesh_pose(mesh, pose)?;
    Ok((posed, pose))
}

fn transform_point(point: Vec3, center: Vec3, rotation_deg: [f64; 3], position_mm: [Option<f64>; 3]) -> Vec3 {
    let mut q = rotate_about(point, center, rotation_deg);
    let new_center = rotate_about(center, center, rotation_deg);
    let target = Vec3::new(position_mm[0].unwrap_or(new_center.x), position_mm[1].unwrap_or(new_center.y), position_mm[2].unwrap_or(new_center.z));
    q = q.add(target.sub(new_center));
    q
}

fn rotate_about(point: Vec3, center: Vec3, rotation_deg: [f64; 3]) -> Vec3 {
    let mut q = point.sub(center);
    if rotation_deg[0] != 0.0 {
        let (sin_t, cos_t) = rotation_deg[0].to_radians().sin_cos();
        q = Vec3::new(q.x, q.y * cos_t - q.z * sin_t, q.y * sin_t + q.z * cos_t);
    }
    if rotation_deg[1] != 0.0 {
        let (sin_t, cos_t) = rotation_deg[1].to_radians().sin_cos();
        q = Vec3::new(q.x * cos_t + q.z * sin_t, q.y, -q.x * sin_t + q.z * cos_t);
    }
    if rotation_deg[2] != 0.0 {
        let (sin_t, cos_t) = rotation_deg[2].to_radians().sin_cos();
        q = Vec3::new(q.x * cos_t - q.y * sin_t, q.x * sin_t + q.y * cos_t, q.z);
    }
    q.add(center)
}

fn clone_mesh(mesh: &Mesh) -> Result<Mesh> {
    Mesh::new(mesh.triangles().to_vec()).map_err(|err| ResinError::invalid(format!("part: {err}")))
}
