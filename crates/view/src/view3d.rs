//! Reusable 3D viewport: orthographic camera, shaded meshes, polylines, and picking.
//!
//! World coordinates are machine millimetres, Z-up, right-handed (X right, Y depth, Z height).
//! This matches resin and FDM beds and imported STL files. The 2D cutting preview in this crate
//! is unchanged.
//!
//! Mouse bindings follow SolidCraft's documented default
//! (<https://docs.solidcraft.io/guide/ui-tour.html>): right-drag orbits, middle-drag pans, and
//! the wheel zooms at the pointer. Standard views are Top, Front, Side, and Iso. The renderer is
//! a clean-room software z-buffer so it stays inside the existing egui stack, appears in
//! screenshots, and needs no extra crates.

use std::time::Instant;

use campanion_mesh::{Aabb, Mesh, Triangle, Vec3};

/// Default background, matching the GUI viewport fill `(18, 22, 30)`.
pub const BACKGROUND_RGBA: [u8; 4] = [18, 22, 30, 255];

/// Directional light in world space, used only for display shading.
const LIGHT: Vec3 = Vec3 { x: 0.35, y: -0.25, z: 0.9 };

/// Pitch of the true isometric view, `atan(1 / sqrt(2))`.
const ISO_PITCH: f64 = 0.615_479_708_670_387_3;

/// Soft floor so a fit on a flat shape still has a usable scale.
const MIN_RADIUS_MM: f64 = 1e-3;

/// A view direction the camera can snap to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandardView {
    /// Looking down −Z. X right, Y up on screen.
    Top,
    /// Looking toward +Y. X right, Z up on screen.
    Front,
    /// Looking toward −X. Y right, Z up on screen.
    Side,
    /// Isometric: yaw 45°, pitch `atan(1/√2)`.
    Iso,
}

impl StandardView {
    pub fn label(self) -> &'static str {
        match self {
            Self::Top => "Top",
            Self::Front => "Front",
            Self::Side => "Side",
            Self::Iso => "Iso",
        }
    }

    pub fn caption(self) -> &'static str {
        match self {
            Self::Top => "Top view — build plate plan",
            Self::Front => "Front view (X–Z)",
            Self::Side => "Side view (Y–Z)",
            Self::Iso => "Isometric view",
        }
    }

    fn yaw_pitch(self) -> (f64, f64) {
        match self {
            Self::Top => (0.0, std::f64::consts::FRAC_PI_2),
            Self::Front => (0.0, 0.0),
            Self::Side => (-std::f64::consts::FRAC_PI_2, 0.0),
            Self::Iso => (std::f64::consts::FRAC_PI_4, ISO_PITCH),
        }
    }
}

/// Why a mesh is in the scene. Colours are display-only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshRole {
    Part,
    Support,
    Raft,
    Island,
    Other,
}

/// One shaded triangle mesh in world millimetres.
#[derive(Clone, Debug)]
pub struct ShadedMesh {
    pub triangles: Vec<Triangle>,
    pub normals: Vec<Vec3>,
    pub color: [u8; 4],
    pub role: MeshRole,
    pub bounds: Option<Aabb>,
}

impl ShadedMesh {
    /// Builds per-triangle normals. Degenerate faces are kept but shaded flat-grey.
    pub fn from_mesh(mesh: &Mesh, color: [u8; 4], role: MeshRole) -> Self {
        let triangles = mesh.triangles().to_vec();
        let normals: Vec<Vec3> = triangles.iter().map(|tri| tri.normal().unwrap_or(Vec3::new(0.0, 0.0, 1.0))).collect();
        let bounds = mesh.bounds().ok();
        Self { triangles, normals, color, role, bounds }
    }
}

/// A 3D polyline, for toolpaths and later FDM extrusion paths.
#[derive(Clone, Debug)]
pub struct Polyline3 {
    pub points: Vec<Vec3>,
    pub color: [u8; 4],
}

/// Build volume in world millimetres. The caller supplies the box; nothing is invented.
///
/// When `max.z` equals `min.z`, only the bed rectangle is drawn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BuildVolume {
    pub min: Vec3,
    pub max: Vec3,
}

impl BuildVolume {
    /// A volume centred on the plate origin, Z from 0 up. `z` may be `None` for a bed rectangle only.
    pub fn centered_plate(x_mm: f64, y_mm: f64, z_mm: Option<f64>) -> Option<Self> {
        if !x_mm.is_finite() || !y_mm.is_finite() || x_mm <= 0.0 || y_mm <= 0.0 {
            return None;
        }
        if let Some(z) = z_mm
            && (!z.is_finite() || z < 0.0)
        {
            return None;
        }
        let z1 = z_mm.unwrap_or(0.0);
        Some(Self { min: Vec3::new(-x_mm * 0.5, -y_mm * 0.5, 0.0), max: Vec3::new(x_mm * 0.5, y_mm * 0.5, z1) })
    }

    pub fn has_height(self) -> bool {
        self.max.z - self.min.z > 1e-9
    }

    pub fn bounds(self) -> Aabb {
        Aabb::from_corners(self.min, self.max)
    }
}

/// Everything the renderer and picker can see.
#[derive(Clone, Debug, Default)]
pub struct Scene3 {
    pub meshes: Vec<ShadedMesh>,
    pub polylines: Vec<Polyline3>,
    pub volume: Option<BuildVolume>,
    /// Support tips or other markers, millimetres, drawn as depth-tested screen circles.
    pub markers: Vec<PointMarker>,
}

/// A pickable point in the view, typically a support tip.
#[derive(Clone, Copy, Debug)]
pub struct PointMarker {
    pub position: Vec3,
    pub color: [u8; 4],
    pub selected: bool,
    pub id: usize,
}

impl Scene3 {
    pub fn bounds(&self) -> Option<Aabb> {
        let mut bounds: Option<Aabb> = None;
        let mut include = |box_: Aabb| {
            bounds = Some(match bounds {
                Some(existing) => existing.union(box_),
                None => box_,
            });
        };
        for mesh in &self.meshes {
            if let Some(box_) = mesh.bounds {
                include(box_);
            }
        }
        for line in &self.polylines {
            if let Some(box_) = Aabb::from_points(line.points.iter().copied()) {
                include(box_);
            }
        }
        if let Some(volume) = self.volume {
            include(volume.bounds());
        }
        for marker in &self.markers {
            include(Aabb::from_corners(marker.position, marker.position));
        }
        bounds
    }
}

/// Orthographic camera, Z-up.
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    /// Point the view turns around, millimetres.
    pub target: Vec3,
    /// Yaw around world Z, radians. Zero looks toward +Y (front).
    pub yaw: f64,
    /// Pitch, radians. Zero is horizontal; `+π/2` is top (looking down −Z).
    pub pitch: f64,
    /// Half-height of the view, millimetres.
    pub radius: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self::standard(StandardView::Iso)
    }
}

impl Camera {
    pub fn standard(view: StandardView) -> Self {
        let (yaw, pitch) = view.yaw_pitch();
        Self { target: Vec3::new(0.0, 0.0, 0.0), yaw, pitch, radius: 50.0 }
    }

    pub fn set_standard(&mut self, view: StandardView) {
        let (yaw, pitch) = view.yaw_pitch();
        self.yaw = yaw;
        self.pitch = pitch;
    }

    pub fn matches_standard(self, view: StandardView) -> bool {
        let (yaw, pitch) = view.yaw_pitch();
        angle_close(self.yaw, yaw) && angle_close(self.pitch, pitch)
    }

    pub fn basis(self) -> Basis {
        let (yaw, pitch) = (self.yaw, self.pitch.clamp(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2));
        let (sy, cy) = (yaw.sin(), yaw.cos());
        let (sp, cp) = (pitch.sin(), pitch.cos());
        let forward = Vec3::new(sy * cp, cy * cp, -sp);
        let world_up = Vec3::new(0.0, 0.0, 1.0);
        let right = forward.cross(world_up);
        let right = right.normalized().unwrap_or_else(|| {
            // At the poles, yaw still defines "right".
            Vec3::new(cy, -sy, 0.0)
        });
        let up = right.cross(forward).normalized().unwrap_or(world_up);
        Basis { forward, right, up }
    }

    /// Scale in pixels per millimetre for a viewport of `height` pixels.
    pub fn pixels_per_mm(self, height_px: f64) -> f64 {
        if self.radius <= MIN_RADIUS_MM { 1.0 } else { (height_px * 0.5) / self.radius }
    }

    pub fn project(self, point: Vec3, width: f64, height: f64) -> ScreenPt {
        let basis = self.basis();
        let rel = point.sub(self.target);
        let scale = self.pixels_per_mm(height);
        let x = width * 0.5 + rel.dot(basis.right) * scale;
        let y = height * 0.5 - rel.dot(basis.up) * scale;
        let depth = rel.dot(basis.forward);
        ScreenPt { x, y, depth }
    }

    /// Ray through a pixel, for picking. Origin sits on the plane through `target`.
    pub fn ray_through(self, px: f64, py: f64, width: f64, height: f64) -> Option<Ray> {
        if !px.is_finite() || !py.is_finite() || width <= 1.0 || height <= 1.0 {
            return None;
        }
        let basis = self.basis();
        let scale = self.pixels_per_mm(height);
        if scale <= 0.0 {
            return None;
        }
        let x_mm = (px - width * 0.5) / scale;
        let y_mm = (height * 0.5 - py) / scale;
        let origin = self.target.add(basis.right.scale(x_mm)).add(basis.up.scale(y_mm)).sub(basis.forward.scale(self.radius * 4.0));
        Some(Ray { origin, direction: basis.forward })
    }

    /// World point on the target plane under a pixel.
    pub fn target_plane_point(self, px: f64, py: f64, width: f64, height: f64) -> Option<Vec3> {
        let ray = self.ray_through(px, py, width, height)?;
        let basis = self.basis();
        let denom = ray.direction.dot(basis.forward);
        if denom.abs() < 1e-12 {
            return None;
        }
        let t = self.target.sub(ray.origin).dot(basis.forward) / denom;
        Some(ray.origin.add(ray.direction.scale(t)))
    }

    pub fn orbit(&mut self, dx_px: f64, dy_px: f64) {
        const RAD_PER_PX: f64 = 0.007;
        self.yaw -= dx_px * RAD_PER_PX;
        self.pitch = (self.pitch + dy_px * RAD_PER_PX).clamp(-std::f64::consts::FRAC_PI_2 + 1e-3, std::f64::consts::FRAC_PI_2 - 1e-3);
    }

    pub fn pan(&mut self, dx_px: f64, dy_px: f64, height_px: f64) {
        let scale = self.pixels_per_mm(height_px);
        if scale <= 0.0 {
            return;
        }
        let basis = self.basis();
        self.target = self.target.sub(basis.right.scale(dx_px / scale)).add(basis.up.scale(dy_px / scale));
    }

    /// Zoom about the world point currently under `(px, py)`.
    pub fn zoom_at(&mut self, factor: f64, px: f64, py: f64, width: f64, height: f64) {
        let factor = factor.clamp(0.1, 10.0);
        let anchor = self.target_plane_point(px, py, width, height);
        self.radius = (self.radius / factor).clamp(MIN_RADIUS_MM, 1.0e6);
        if let Some(anchor) = anchor
            && let Some(after) = self.target_plane_point(px, py, width, height)
        {
            self.target = self.target.add(anchor.sub(after));
        }
    }

    /// Frame `bounds` in the current orientation, with a small margin.
    pub fn fit(&mut self, bounds: Aabb, width: f64, height: f64) {
        let basis = self.basis();
        let min = bounds.min();
        let max = bounds.max();
        let corners = [
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(max.x, max.y, min.z),
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(max.x, min.y, max.z),
            Vec3::new(min.x, max.y, max.z),
            Vec3::new(max.x, max.y, max.z),
        ];
        self.target = Vec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5);
        let mut half_w = MIN_RADIUS_MM;
        let mut half_h = MIN_RADIUS_MM;
        for corner in corners {
            let rel = corner.sub(self.target);
            half_w = half_w.max(rel.dot(basis.right).abs());
            half_h = half_h.max(rel.dot(basis.up).abs());
        }
        const MARGIN: f64 = 1.12;
        let aspect = if height > 1.0 { width / height } else { 1.0 };
        let radius_from_h = half_h * MARGIN;
        let radius_from_w = if aspect > 0.0 { (half_w * MARGIN) / aspect } else { half_w * MARGIN };
        self.radius = radius_from_h.max(radius_from_w).max(MIN_RADIUS_MM);
    }
}

/// Camera axes in world space.
#[derive(Clone, Copy, Debug)]
pub struct Basis {
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
}

/// Projected point, pixels, plus camera-forward depth in millimetres.
#[derive(Clone, Copy, Debug)]
pub struct ScreenPt {
    pub x: f64,
    pub y: f64,
    pub depth: f64,
}

/// A ray in world millimetres.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

impl Ray {
    pub fn at(self, t: f64) -> Vec3 {
        self.origin.add(self.direction.scale(t))
    }
}

/// Closest mesh (or marker) hit along a ray or under a pixel.
#[derive(Clone, Copy, Debug)]
pub struct PickHit {
    pub point: Vec3,
    pub distance: f64,
    pub role: MeshRole,
    pub mesh_index: Option<usize>,
    pub marker_id: Option<usize>,
}

/// Rasterized frame, unmultiplied RGBA, row-major, top-left origin.
#[derive(Clone, Debug)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
    /// Wall time of the last [`render`] call, milliseconds.
    pub frame_ms: f64,
}

/// Rasterize `scene` into an unmultiplied RGBA image.
pub fn render(scene: &Scene3, camera: &Camera, width: u32, height: u32) -> Frame {
    let started = Instant::now();
    let w = width.max(1);
    let h = height.max(1);
    let (wf, hf) = (f64::from(w), f64::from(h));
    let mut rgba = vec![0_u8; w as usize * h as usize * 4];
    for px in rgba.chunks_exact_mut(4) {
        px.copy_from_slice(&BACKGROUND_RGBA);
    }
    let mut zbuf = vec![f32::INFINITY; w as usize * h as usize];
    let basis = camera.basis();
    let light = LIGHT.normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    raster_meshes(scene, camera, wf, hf, w, h, &basis, light, &mut rgba, &mut zbuf);

    if let Some(volume) = scene.volume {
        stroke_volume(&mut rgba, &mut zbuf, w, h, camera, volume);
        fill_bed(&mut rgba, &mut zbuf, w, h, camera, volume);
    }

    for line in &scene.polylines {
        let pts = &line.points;
        if pts.len() < 2 {
            continue;
        }
        for pair in pts.windows(2) {
            let a = camera.project(pair[0], wf, hf);
            let b = camera.project(pair[1], wf, hf);
            stroke_line(&mut rgba, &mut zbuf, w, h, a, b, line.color);
        }
    }

    for marker in &scene.markers {
        let p = camera.project(marker.position, wf, hf);
        let radius = if marker.selected { 6.5 } else { 5.0 };
        fill_circle(&mut rgba, &mut zbuf, w, h, p, radius, marker.color, marker.selected);
    }

    Frame { width: w, height: h, rgba, frame_ms: started.elapsed().as_secs_f64() * 1_000.0 }
}

/// Closest mesh hit along `ray`.
pub fn pick_mesh(scene: &Scene3, ray: Ray) -> Option<PickHit> {
    let mut best: Option<PickHit> = None;
    for (mesh_index, mesh) in scene.meshes.iter().enumerate() {
        if mesh.role == MeshRole::Island {
            continue;
        }
        for tri in &mesh.triangles {
            if let Some((t, point)) = intersect_triangle(ray, *tri)
                && t >= 0.0
                && best.is_none_or(|hit| t < hit.distance)
            {
                best = Some(PickHit { point, distance: t, role: mesh.role, mesh_index: Some(mesh_index), marker_id: None });
            }
        }
    }
    best
}

/// Closest marker under a pixel, in screen pixels, within `tol_px`.
pub fn pick_marker(scene: &Scene3, camera: &Camera, px: f64, py: f64, width: f64, height: f64, tol_px: f64) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;
    for marker in &scene.markers {
        let p = camera.project(marker.position, width, height);
        let d = (p.x - px).hypot(p.y - py);
        let reach = if marker.selected { tol_px + 1.5 } else { tol_px };
        if d <= reach && best.is_none_or(|(_, best_d)| d < best_d) {
            best = Some((marker.id, d));
        }
    }
    best
}

/// Closest hit: markers win over the mesh when both are under the cursor.
pub fn pick(scene: &Scene3, camera: &Camera, px: f64, py: f64, width: f64, height: f64, marker_tol_px: f64) -> Option<PickHit> {
    if let Some((id, _)) = pick_marker(scene, camera, px, py, width, height, marker_tol_px)
        && let Some(marker) = scene.markers.iter().find(|m| m.id == id)
    {
        return Some(PickHit { point: marker.position, distance: 0.0, role: MeshRole::Support, mesh_index: None, marker_id: Some(id) });
    }
    let ray = camera.ray_through(px, py, width, height)?;
    pick_mesh(scene, ray)
}

/// A heightfield of about `target_triangles` triangles, for performance checks.
pub fn heightfield_mesh(target_triangles: usize, size_mm: f64) -> Result<Mesh, campanion_mesh::MeshError> {
    let target = target_triangles.max(2);
    let cells = ((target as f64 / 2.0).sqrt().ceil() as usize).max(1);
    let mut tris = Vec::with_capacity(cells * cells * 2);
    let step = size_mm / cells as f64;
    for iy in 0..cells {
        for ix in 0..cells {
            let x0 = ix as f64 * step;
            let y0 = iy as f64 * step;
            let x1 = x0 + step;
            let y1 = y0 + step;
            let z00 = height_at(x0, y0, size_mm);
            let z10 = height_at(x1, y0, size_mm);
            let z01 = height_at(x0, y1, size_mm);
            let z11 = height_at(x1, y1, size_mm);
            let a = Vec3::new(x0, y0, z00);
            let b = Vec3::new(x1, y0, z10);
            let c = Vec3::new(x1, y1, z11);
            let d = Vec3::new(x0, y1, z01);
            tris.push(Triangle { a, b, c });
            tris.push(Triangle { a, b: c, c: d });
        }
    }
    Mesh::new(tris)
}

fn height_at(x: f64, y: f64, size: f64) -> f64 {
    let nx = x / size * std::f64::consts::TAU;
    let ny = y / size * std::f64::consts::TAU;
    4.0 + 3.0 * nx.sin() * ny.cos() + 1.2 * (2.0 * nx).cos()
}

fn angle_close(a: f64, b: f64) -> bool {
    let mut d = (a - b) % std::f64::consts::TAU;
    if d > std::f64::consts::PI {
        d -= std::f64::consts::TAU;
    }
    if d < -std::f64::consts::PI {
        d += std::f64::consts::TAU;
    }
    d.abs() < 1e-6
}

fn shade_color(color: [u8; 4], shade: f64) -> [u8; 4] {
    let s = shade.clamp(0.0, 1.0);
    [(f64::from(color[0]) * s).round() as u8, (f64::from(color[1]) * s).round() as u8, (f64::from(color[2]) * s).round() as u8, color[3]]
}

fn raster_meshes(scene: &Scene3, camera: &Camera, wf: f64, hf: f64, w: u32, h: u32, basis: &Basis, light: Vec3, rgba: &mut [u8], zbuf: &mut [f32]) {
    let total: usize = scene.meshes.iter().map(|mesh| mesh.triangles.len()).sum();
    let threads = render_threads(total);
    if threads <= 1 {
        for mesh in &scene.meshes {
            raster_tri_range(mesh.color, &mesh.triangles, &mesh.normals, camera, wf, hf, w, h, basis, light, rgba, zbuf);
        }
        return;
    }
    // Split each mesh's triangles across workers so a single large mesh still parallelises.
    let mut work: Vec<Vec<(&[u8; 4], &[Triangle], &[Vec3])>> = (0..threads).map(|_| Vec::new()).collect();
    for mesh in &scene.meshes {
        let n = mesh.triangles.len();
        if n == 0 {
            continue;
        }
        let chunk = n.div_ceil(threads).max(1);
        for (t, slot) in work.iter_mut().enumerate() {
            let start = t * chunk;
            if start >= n {
                break;
            }
            let end = (start + chunk).min(n);
            slot.push((&mesh.color, &mesh.triangles[start..end], &mesh.normals[start..end]));
        }
    }
    let pixel_count = w as usize * h as usize;
    std::thread::scope(|scope| {
        let mut handles = Vec::new();
        for slot in &work {
            if slot.is_empty() {
                continue;
            }
            let handle = scope.spawn(move || {
                let mut local_rgba = vec![0_u8; pixel_count * 4];
                let mut local_z = vec![f32::INFINITY; pixel_count];
                for (color, tris, normals) in slot {
                    raster_tri_range(**color, tris, normals, camera, wf, hf, w, h, basis, light, &mut local_rgba, &mut local_z);
                }
                (local_rgba, local_z)
            });
            handles.push(handle);
        }
        for handle in handles {
            if let Ok((local_rgba, local_z)) = handle.join() {
                merge_buffers(rgba, zbuf, &local_rgba, &local_z);
            }
        }
    });
}

fn render_threads(triangle_count: usize) -> usize {
    if triangle_count < 8_000 {
        return 1;
    }
    std::thread::available_parallelism().map(std::num::NonZero::get).unwrap_or(1).clamp(1, 8)
}

fn raster_tri_range(
    color: [u8; 4],
    tris: &[Triangle],
    normals: &[Vec3],
    camera: &Camera,
    wf: f64,
    hf: f64,
    w: u32,
    h: u32,
    basis: &Basis,
    light: Vec3,
    rgba: &mut [u8],
    zbuf: &mut [f32],
) {
    for (tri, normal) in tris.iter().zip(normals.iter()) {
        if normal.dot(basis.forward) >= -1e-9 {
            continue;
        }
        let a = camera.project(tri.a, wf, hf);
        let b = camera.project(tri.b, wf, hf);
        let c = camera.project(tri.c, wf, hf);
        let shade = 0.28 + 0.72 * normal.dot(light).max(0.0);
        fill_triangle(rgba, zbuf, w, h, a, b, c, shade_color(color, shade));
    }
}

fn merge_buffers(rgba: &mut [u8], zbuf: &mut [f32], src_rgba: &[u8], src_z: &[f32]) {
    for i in 0..zbuf.len() {
        let z = src_z[i];
        if z < zbuf[i] {
            zbuf[i] = z;
            let o = i * 4;
            rgba[o] = src_rgba[o];
            rgba[o + 1] = src_rgba[o + 1];
            rgba[o + 2] = src_rgba[o + 2];
            rgba[o + 3] = src_rgba[o + 3];
        }
    }
}

fn fill_triangle(rgba: &mut [u8], zbuf: &mut [f32], w: u32, h: u32, a: ScreenPt, b: ScreenPt, c: ScreenPt, color: [u8; 4]) {
    let area = edge_f32(a.x as f32, a.y as f32, b.x as f32, b.y as f32, c.x as f32, c.y as f32);
    if area.abs() < 1e-8 {
        return;
    }
    let (a, b, c, area) = if area < 0.0 { (a, c, b, -area) } else { (a, b, c, area) };
    let min_x = a.x.min(b.x).min(c.x).floor().max(0.0) as i32;
    let max_x = a.x.max(b.x).max(c.x).ceil().min(f64::from(w) - 1.0) as i32;
    let min_y = a.y.min(b.y).min(c.y).floor().max(0.0) as i32;
    let max_y = a.y.max(b.y).max(c.y).ceil().min(f64::from(h) - 1.0) as i32;
    if min_x > max_x || min_y > max_y {
        return;
    }
    let ax = a.x as f32;
    let ay = a.y as f32;
    let bx = b.x as f32;
    let by = b.y as f32;
    let cx = c.x as f32;
    let cy = c.y as f32;
    let inv = 1.0 / area;
    // Incremental edge functions: d/dx of edge(b,c,p) is (by - cy).
    let dw0_dx = by - cy;
    let dw1_dx = cy - ay;
    let dw2_dx = ay - by;
    let dw0_dy = cx - bx;
    let dw1_dy = ax - cx;
    let dw2_dy = bx - ax;
    let za = a.depth as f32;
    let zb = b.depth as f32;
    let zc = c.depth as f32;
    let width = w as usize;
    let cr = color[0];
    let cg = color[1];
    let cb = color[2];
    let ca = color[3];
    let x0 = f32::from(min_x as u16) + 0.5;
    let y0 = f32::from(min_y as u16) + 0.5;
    let mut w0_row = edge_f32(bx, by, cx, cy, x0, y0);
    let mut w1_row = edge_f32(cx, cy, ax, ay, x0, y0);
    let mut w2_row = edge_f32(ax, ay, bx, by, x0, y0);
    for y in min_y..=max_y {
        let mut w0 = w0_row;
        let mut w1 = w1_row;
        let mut w2 = w2_row;
        let row = y as usize * width;
        for x in min_x..=max_x {
            if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
                let depth = (za * w0 + zb * w1 + zc * w2) * inv;
                let i = row + x as usize;
                if depth < zbuf[i] {
                    zbuf[i] = depth;
                    let o = i * 4;
                    rgba[o] = cr;
                    rgba[o + 1] = cg;
                    rgba[o + 2] = cb;
                    rgba[o + 3] = ca;
                }
            }
            w0 += dw0_dx;
            w1 += dw1_dx;
            w2 += dw2_dx;
        }
        w0_row += dw0_dy;
        w1_row += dw1_dy;
        w2_row += dw2_dy;
    }
}

fn edge_f32(ax: f32, ay: f32, bx: f32, by: f32, cx: f32, cy: f32) -> f32 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

fn fill_bed(rgba: &mut [u8], zbuf: &mut [f32], w: u32, h: u32, camera: &Camera, volume: BuildVolume) {
    let (wf, hf) = (f64::from(w), f64::from(h));
    let z0 = volume.min.z;
    let corners = [
        Vec3::new(volume.min.x, volume.min.y, z0),
        Vec3::new(volume.max.x, volume.min.y, z0),
        Vec3::new(volume.max.x, volume.max.y, z0),
        Vec3::new(volume.min.x, volume.max.y, z0),
    ];
    let pts = [
        camera.project(corners[0], wf, hf),
        camera.project(corners[1], wf, hf),
        camera.project(corners[2], wf, hf),
        camera.project(corners[3], wf, hf),
    ];
    let fill = [50, 80, 120, 70];
    fill_triangle(rgba, zbuf, w, h, pts[0], pts[1], pts[2], fill);
    fill_triangle(rgba, zbuf, w, h, pts[0], pts[2], pts[3], fill);
}

fn stroke_volume(rgba: &mut [u8], zbuf: &mut [f32], w: u32, h: u32, camera: &Camera, volume: BuildVolume) {
    let (wf, hf) = (f64::from(w), f64::from(h));
    let color = [110, 160, 210, 255];
    let (x0, y0, z0) = (volume.min.x, volume.min.y, volume.min.z);
    let (x1, y1, z1) = (volume.max.x, volume.max.y, volume.max.z);
    let p = |x, y, z| camera.project(Vec3::new(x, y, z), wf, hf);
    let edges = [(p(x0, y0, z0), p(x1, y0, z0)), (p(x1, y0, z0), p(x1, y1, z0)), (p(x1, y1, z0), p(x0, y1, z0)), (p(x0, y1, z0), p(x0, y0, z0))];
    for (a, b) in edges {
        stroke_line(rgba, zbuf, w, h, a, b, color);
    }
    if volume.has_height() {
        let up = [
            (p(x0, y0, z1), p(x1, y0, z1)),
            (p(x1, y0, z1), p(x1, y1, z1)),
            (p(x1, y1, z1), p(x0, y1, z1)),
            (p(x0, y1, z1), p(x0, y0, z1)),
            (p(x0, y0, z0), p(x0, y0, z1)),
            (p(x1, y0, z0), p(x1, y0, z1)),
            (p(x1, y1, z0), p(x1, y1, z1)),
            (p(x0, y1, z0), p(x0, y1, z1)),
        ];
        for (a, b) in up {
            stroke_line(rgba, zbuf, w, h, a, b, color);
        }
    }
}

fn stroke_line(rgba: &mut [u8], zbuf: &mut [f32], w: u32, h: u32, a: ScreenPt, b: ScreenPt, color: [u8; 4]) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let steps = dx.abs().max(dy.abs()).ceil().max(1.0) as i32;
    let width = w as usize;
    let height = h as usize;
    for i in 0..=steps {
        let t = f64::from(i) / f64::from(steps);
        let x = a.x + dx * t;
        let y = a.y + dy * t;
        let depth = (a.depth + (b.depth - a.depth) * t) as f32;
        let xi = x.round() as i32;
        let yi = y.round() as i32;
        if xi < 0 || yi < 0 {
            continue;
        }
        let (xu, yu) = (xi as usize, yi as usize);
        if xu >= width || yu >= height {
            continue;
        }
        let idx = yu * width + xu;
        if depth <= zbuf[idx] + 0.02 {
            zbuf[idx] = zbuf[idx].min(depth);
            let o = idx * 4;
            rgba[o] = color[0];
            rgba[o + 1] = color[1];
            rgba[o + 2] = color[2];
            rgba[o + 3] = color[3];
        }
    }
}

fn fill_circle(rgba: &mut [u8], zbuf: &mut [f32], w: u32, h: u32, p: ScreenPt, radius: f64, color: [u8; 4], selected: bool) {
    let width = w as usize;
    let height = h as usize;
    let r = radius.ceil() as i32 + 1;
    let cx = p.x.round() as i32;
    let cy = p.y.round() as i32;
    let depth = p.depth as f32;
    let outline = if selected { [255, 220, 80, 255] } else { [30, 30, 30, 255] };
    for dy in -r..=r {
        for dx in -r..=r {
            let x = cx + dx;
            let y = cy + dy;
            if x < 0 || y < 0 {
                continue;
            }
            let (xu, yu) = (x as usize, y as usize);
            if xu >= width || yu >= height {
                continue;
            }
            let dist = f64::from(dx).hypot(f64::from(dy));
            if dist > radius + 1.0 {
                continue;
            }
            let idx = yu * width + xu;
            if depth > zbuf[idx] + 0.15 {
                continue;
            }
            zbuf[idx] = zbuf[idx].min(depth);
            let paint = if dist >= radius - 1.0 { outline } else { color };
            let o = idx * 4;
            rgba[o] = paint[0];
            rgba[o + 1] = paint[1];
            rgba[o + 2] = paint[2];
            rgba[o + 3] = paint[3];
        }
    }
}

fn intersect_triangle(ray: Ray, tri: Triangle) -> Option<(f64, Vec3)> {
    // Möller–Trumbore.
    const EPS: f64 = 1e-10;
    let edge1 = tri.b.sub(tri.a);
    let edge2 = tri.c.sub(tri.a);
    let pvec = ray.direction.cross(edge2);
    let det = edge1.dot(pvec);
    if det.abs() < EPS {
        return None;
    }
    let inv = 1.0 / det;
    let tvec = ray.origin.sub(tri.a);
    let u = tvec.dot(pvec) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qvec = tvec.cross(edge1);
    let v = ray.direction.dot(qvec) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = edge2.dot(qvec) * inv;
    if t < 0.0 {
        return None;
    }
    Some((t, ray.at(t)))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::float_cmp)]
mod tests {
    use super::*;

    fn cube() -> Mesh {
        Mesh::new(Aabb::from_corners(Vec3::new(-5.0, -5.0, 0.0), Vec3::new(5.0, 5.0, 10.0)).face_triangles().to_vec()).expect("cube")
    }

    fn scene_with(mesh: &Mesh) -> Scene3 {
        let mut scene = Scene3::default();
        scene.meshes.push(ShadedMesh::from_mesh(mesh, [200, 200, 210, 255], MeshRole::Part));
        scene
    }

    #[test]
    fn standard_views_are_orthogonal() {
        let mut cam = Camera::standard(StandardView::Top);
        let basis = cam.basis();
        assert!((basis.forward.z + 1.0).abs() < 1e-9);
        assert!((basis.right.x - 1.0).abs() < 1e-9);
        assert!((basis.up.y - 1.0).abs() < 1e-9);

        cam.set_standard(StandardView::Front);
        let basis = cam.basis();
        assert!((basis.forward.y - 1.0).abs() < 1e-9);
        assert!((basis.up.z - 1.0).abs() < 1e-9);

        cam.set_standard(StandardView::Side);
        let basis = cam.basis();
        assert!((basis.forward.x + 1.0).abs() < 1e-9);
        assert!((basis.up.z - 1.0).abs() < 1e-9);
        assert!((basis.right.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn top_view_projects_xy() {
        let cam = Camera::standard(StandardView::Top);
        let p = cam.project(Vec3::new(10.0, 4.0, 7.0), 200.0, 200.0);
        let o = cam.project(Vec3::new(0.0, 0.0, 0.0), 200.0, 200.0);
        // X right, Y up on screen (smaller y is up).
        assert!(p.x > o.x);
        assert!(p.y < o.y);
    }

    #[test]
    fn zoom_at_cursor_keeps_the_world_point() {
        let mut cam = Camera::standard(StandardView::Iso);
        cam.target = Vec3::new(3.0, 2.0, 1.0);
        let before = cam.target_plane_point(140.0, 90.0, 200.0, 150.0).expect("anchor");
        cam.zoom_at(2.0, 140.0, 90.0, 200.0, 150.0);
        let after = cam.target_plane_point(140.0, 90.0, 200.0, 150.0).expect("anchor after");
        assert!((before.x - after.x).abs() < 1e-6);
        assert!((before.y - after.y).abs() < 1e-6);
        assert!((before.z - after.z).abs() < 1e-6);
    }

    #[test]
    fn fit_frames_the_bounds() {
        let mut cam = Camera::standard(StandardView::Iso);
        let bounds = Aabb::from_corners(Vec3::new(-20.0, -10.0, 0.0), Vec3::new(20.0, 10.0, 15.0));
        cam.fit(bounds, 800.0, 600.0);
        assert!((cam.target.x).abs() < 1e-9);
        assert!((cam.target.y).abs() < 1e-9);
        assert!((cam.target.z - 7.5).abs() < 1e-9);
        // Every corner stays inside the view with margin.
        let min = bounds.min();
        let max = bounds.max();
        for corner in [Vec3::new(min.x, min.y, min.z), Vec3::new(max.x, max.y, max.z), Vec3::new(min.x, max.y, max.z)] {
            let p = cam.project(corner, 800.0, 600.0);
            assert!(p.x >= 0.0 && p.x <= 800.0, "x {}", p.x);
            assert!(p.y >= 0.0 && p.y <= 600.0, "y {}", p.y);
        }
    }

    #[test]
    fn pick_hits_the_top_of_a_cube() {
        let mesh = cube();
        let scene = scene_with(&mesh);
        let mut cam = Camera::standard(StandardView::Top);
        cam.fit(mesh.bounds().expect("bounds"), 200.0, 200.0);
        let hit = pick(&scene, &cam, 100.0, 100.0, 200.0, 200.0, 8.0).expect("hit");
        assert_eq!(hit.role, MeshRole::Part);
        assert!((hit.point.z - 10.0).abs() < 1e-6, "z {}", hit.point.z);
        assert!(hit.point.x.abs() < 5.1);
        assert!(hit.point.y.abs() < 5.1);
    }

    #[test]
    fn marker_wins_over_the_mesh() {
        let mesh = cube();
        let mut scene = scene_with(&mesh);
        scene.markers.push(PointMarker { position: Vec3::new(0.0, 0.0, 10.0), color: [240, 140, 40, 255], selected: false, id: 7 });
        let mut cam = Camera::standard(StandardView::Top);
        cam.fit(mesh.bounds().expect("bounds"), 200.0, 200.0);
        let hit = pick(&scene, &cam, 100.0, 100.0, 200.0, 200.0, 8.0).expect("hit");
        assert_eq!(hit.marker_id, Some(7));
    }

    #[test]
    fn centered_plate_rejects_non_positive_sizes() {
        assert!(BuildVolume::centered_plate(0.0, 100.0, Some(50.0)).is_none());
        assert!(BuildVolume::centered_plate(100.0, -1.0, None).is_none());
        let vol = BuildVolume::centered_plate(200.0, 100.0, Some(80.0)).expect("vol");
        assert!((vol.min.x + 100.0).abs() < 1e-12);
        assert!((vol.max.y - 50.0).abs() < 1e-12);
        assert!(vol.has_height());
    }

    #[test]
    fn render_paints_more_than_the_background() {
        let mesh = cube();
        let scene = scene_with(&mesh);
        let mut cam = Camera::standard(StandardView::Iso);
        cam.fit(mesh.bounds().expect("bounds"), 120.0, 90.0);
        let frame = render(&scene, &cam, 120, 90);
        assert_eq!(frame.rgba.len(), 120 * 90 * 4);
        let lit = frame.rgba.chunks_exact(4).filter(|px| px != &BACKGROUND_RGBA).count();
        assert!(lit > 200, "expected a visible shaded cube, got {lit} lit pixels");
    }

    #[test]
    fn render_100k_triangles_reports_frame_time() {
        let mesh = heightfield_mesh(100_000, 120.0).expect("heightfield");
        let count = mesh.triangles().len();
        assert!((90_000..=110_000).contains(&count), "got {count} triangles");
        let scene = scene_with(&mesh);
        let mut cam = Camera::standard(StandardView::Iso);
        cam.fit(mesh.bounds().expect("bounds"), 800.0, 600.0);
        let warm = render(&scene, &cam, 800, 600);
        let mut times = Vec::new();
        for step in 0..8 {
            cam.orbit(18.0, 6.0 * if step % 2 == 0 { 1.0 } else { -1.0 });
            let frame = render(&scene, &cam, 800, 600);
            times.push(frame.frame_ms);
        }
        let mean = times.iter().sum::<f64>() / times.len() as f64;
        let max = times.iter().copied().fold(0.0_f64, f64::max);
        println!(
            "view3d 100k-tri orbit: {count} triangles, 800x600, warmup {:.2} ms, mean {:.2} ms, max {:.2} ms, samples {:?}",
            warm.frame_ms, mean, max, times
        );
        // The test records times for the report. It does not invent a hard FPS gate that would
        // flake on a slow CI runner; a clearly hung renderer (over 2 s a frame) still fails.
        assert!(max < 2_000.0, "orbit frame was {max:.1} ms");
    }
}
