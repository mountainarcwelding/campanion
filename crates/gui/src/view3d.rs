//! egui widget for the shared 3D viewport in `campanion-view`.
//!
//! Mouse bindings follow SolidCraft's documented default
//! (<https://docs.solidcraft.io/guide/ui-tour.html>): right-drag orbits, middle-drag pans, and
//! the wheel zooms at the pointer. Cutting workspaces keep the existing 2D viewport.

use campanion_mesh::Aabb;
use campanion_view::{Camera, Frame, PickHit, Scene3, StandardView, pick, pick_mesh, render};
use eframe::egui::{self, Color32, ColorImage, PointerButton, Pos2, Rect, RichText, Sense, TextureOptions};

use crate::theme;

const SCROLL_ZOOM: f64 = 0.0018;
const MARKER_TOL_PX: f64 = 10.0;
/// Cap the raster on the long side so a huge window still orbits smoothly.
const MAX_RASTER_EDGE: u32 = 1_600;

/// Interactive 3D camera for one viewport.
#[derive(Clone, Debug)]
pub struct View3d {
    pub camera: Camera,
    /// True after a user orbit, pan, zoom, or an explicit standard-view snap.
    pub user_adjusted: bool,
    /// Last physical size a fit was computed for.
    last_size: Option<[f32; 2]>,
    last_frame_ms: f64,
    /// After the next auto-fit, apply this orbit (pixels).
    pub pending_orbit: Option<(f64, f64)>,
    /// On the next frame, fit the full scene including the build volume.
    pub pending_fit_all: bool,
}

impl Default for View3d {
    fn default() -> Self {
        Self {
            camera: Camera::standard(StandardView::Iso),
            user_adjusted: false,
            last_size: None,
            last_frame_ms: 0.0,
            pending_orbit: None,
            pending_fit_all: false,
        }
    }
}

impl View3d {
    pub fn set_standard(&mut self, view: StandardView) {
        self.camera.set_standard(view);
        self.user_adjusted = true;
    }

    pub fn frame_ms(&self) -> f64 {
        self.last_frame_ms
    }
}

/// Result of drawing the 3D viewport for one frame.
pub struct View3dOutput {
    pub response: egui::Response,
    pub rect: Rect,
    pub hover_hit: Option<PickHit>,
    /// Mesh hit under the pointer, ignoring markers. Used while dragging a tip along the surface.
    pub hover_mesh_hit: Option<PickHit>,
    pub click_hit: Option<PickHit>,
    pub primary_dragging: bool,
    pub primary_drag_started: bool,
    pub primary_drag_stopped: bool,
}

/// Draw standard-view chips, Fit all, and the 3D viewport.
pub fn show_with_controls(ui: &mut egui::Ui, view: &mut View3d, scene: &Scene3, caption: &str) -> View3dOutput {
    ui.label(RichText::new(caption).color(theme::MUTED).size(11.0));
    ui.horizontal(|ui| {
        ui.label(RichText::new("View").color(theme::MUTED).size(11.0));
        for standard in [StandardView::Top, StandardView::Front, StandardView::Side, StandardView::Iso] {
            let selected = view.camera.matches_standard(standard);
            if ui.selectable_label(selected, standard.label()).on_hover_text(standard.caption()).clicked() {
                view.set_standard(standard);
            }
        }
        let fit_reason = fit_disabled_reason(scene);
        let enabled = fit_reason.is_none();
        let fit = ui.add_enabled(enabled, egui::Button::new("Fit all"));
        if let Some(reason) = fit_reason.as_deref() {
            fit.clone().on_hover_text(reason);
        } else {
            fit.clone().on_hover_text("Frame the part, supports, and the build volume when a bed size is on the machine profile.");
        }
        if enabled
            && fit.clicked()
            && let Some(bounds) = scene.bounds()
        {
            let size = ui.available_size();
            view.camera.fit(bounds, f64::from(size.x.max(1.0)), f64::from(size.y.max(1.0)));
            view.user_adjusted = true;
        }
    });
    ui.label(RichText::new("Right-drag orbits, middle-drag pans, wheel zooms at the pointer (SolidCraft default).").color(theme::MUTED).size(10.0));
    show(ui, view, scene)
}

/// Why Fit all is disabled. `None` when it can run.
pub fn fit_disabled_reason(scene: &Scene3) -> Option<String> {
    if scene.bounds().is_some() { None } else { Some("Fit all is disabled: import a part or enter a bed size on the machine profile.".to_string()) }
}

/// Bounds of meshes, markers and polylines, excluding the build volume, so a small part stays readable.
pub fn content_bounds(scene: &Scene3) -> Option<Aabb> {
    let mut bounds: Option<Aabb> = None;
    let mut include = |box_: Aabb| {
        bounds = Some(match bounds {
            Some(existing) => existing.union(box_),
            None => box_,
        });
    };
    for mesh in &scene.meshes {
        if let Some(box_) = mesh.bounds {
            include(box_);
        }
    }
    for line in &scene.polylines {
        if let Some(box_) = Aabb::from_points(line.points.iter().copied()) {
            include(box_);
        }
    }
    for marker in &scene.markers {
        include(Aabb::from_corners(marker.position, marker.position));
    }
    bounds
}

/// Draw the 3D viewport into the remaining space of `ui`.
pub fn show(ui: &mut egui::Ui, view: &mut View3d, scene: &Scene3) -> View3dOutput {
    let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
    let ppp = f64::from(ui.ctx().pixels_per_point()).max(1.0);
    let pixel_w = ((f64::from(rect.width()) * ppp).round() as u32).max(1);
    let pixel_h = ((f64::from(rect.height()) * ppp).round() as u32).max(1);
    let (raster_w, raster_h) = clamp_raster(pixel_w, pixel_h);

    let size = [raster_w as f32, raster_h as f32];
    if view.pending_fit_all {
        if let Some(bounds) = scene.bounds() {
            view.camera.fit(bounds, f64::from(raster_w), f64::from(raster_h));
            view.user_adjusted = true;
        }
        view.pending_fit_all = false;
        view.last_size = Some(size);
    } else if !view.user_adjusted {
        let size_changed = view.last_size.is_none_or(|prev| (prev[0] - size[0]).abs() > 1.0 || (prev[1] - size[1]).abs() > 1.0);
        if size_changed && let Some(bounds) = content_bounds(scene).or_else(|| scene.bounds()) {
            view.camera.fit(bounds, f64::from(raster_w), f64::from(raster_h));
            view.last_size = Some(size);
        }
    }
    if let Some((dx, dy)) = view.pending_orbit.take() {
        view.camera.orbit(dx, dy);
        view.user_adjusted = true;
    }

    if response.hovered() {
        let scroll = ui.input(|input| input.smooth_scroll_delta.y);
        if scroll.abs() > f32::EPSILON
            && let Some(pointer) = response.hover_pos()
        {
            let factor = (1.0 + f64::from(scroll) * SCROLL_ZOOM).clamp(0.1, 10.0);
            let (px, py) = pointer_to_raster(pointer, rect, raster_w, raster_h);
            view.camera.zoom_at(factor, px, py, f64::from(raster_w), f64::from(raster_h));
            view.user_adjusted = true;
        }
    }
    if response.dragged_by(PointerButton::Secondary) {
        let delta = response.drag_delta();
        view.camera.orbit(f64::from(delta.x), f64::from(delta.y));
        view.user_adjusted = true;
    } else if response.dragged_by(PointerButton::Middle) {
        let delta = response.drag_delta();
        view.camera.pan(f64::from(delta.x), f64::from(delta.y), f64::from(raster_h).max(1.0));
        view.user_adjusted = true;
    }

    let frame = render(scene, &view.camera, raster_w, raster_h);
    view.last_frame_ms = frame.frame_ms;
    blit_frame(ui, rect, &frame);

    let hover_pos = response.hover_pos();
    let hover_hit = hover_pos.and_then(|pos| {
        let (px, py) = pointer_to_raster(pos, rect, raster_w, raster_h);
        pick(scene, &view.camera, px, py, f64::from(raster_w), f64::from(raster_h), MARKER_TOL_PX)
    });
    let hover_mesh_hit = hover_pos.and_then(|pos| {
        let (px, py) = pointer_to_raster(pos, rect, raster_w, raster_h);
        let ray = view.camera.ray_through(px, py, f64::from(raster_w), f64::from(raster_h))?;
        pick_mesh(scene, ray)
    });
    let click_hit = if response.clicked_by(PointerButton::Primary) { hover_hit } else { None };
    let primary_dragging = response.dragged_by(PointerButton::Primary);
    let primary_drag_started = response.drag_started_by(PointerButton::Primary);
    let primary_drag_stopped = response.drag_stopped();

    View3dOutput { response, rect, hover_hit, hover_mesh_hit, click_hit, primary_dragging, primary_drag_started, primary_drag_stopped }
}

fn clamp_raster(width: u32, height: u32) -> (u32, u32) {
    let long = width.max(height);
    if long <= MAX_RASTER_EDGE {
        return (width.max(1), height.max(1));
    }
    let scale = f64::from(MAX_RASTER_EDGE) / f64::from(long);
    let w = ((f64::from(width) * scale).round() as u32).max(1);
    let h = ((f64::from(height) * scale).round() as u32).max(1);
    (w, h)
}

fn pointer_to_raster(pointer: Pos2, rect: Rect, raster_w: u32, raster_h: u32) -> (f64, f64) {
    let x = f64::from((pointer.x - rect.left()) / rect.width().max(1.0)) * f64::from(raster_w);
    let y = f64::from((pointer.y - rect.top()) / rect.height().max(1.0)) * f64::from(raster_h);
    (x, y)
}

fn blit_frame(ui: &mut egui::Ui, rect: Rect, frame: &Frame) {
    let image = ColorImage::from_rgba_unmultiplied([frame.width as usize, frame.height as usize], &frame.rgba);
    let texture = ui.ctx().load_texture("campanion-view3d", image, TextureOptions::LINEAR);
    ui.painter().image(texture.id(), rect, Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(1.0, 1.0)), Color32::WHITE);
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use campanion_mesh::{Aabb, Mesh, Vec3};
    use campanion_view::{MeshRole, ShadedMesh};

    use super::*;

    #[test]
    fn fit_is_disabled_on_an_empty_scene() {
        let scene = Scene3::default();
        let reason = fit_disabled_reason(&scene).expect("reason");
        assert!(reason.contains("import a part") || reason.contains("bed size"));
    }

    #[test]
    fn content_bounds_ignore_the_volume() {
        let mesh = Mesh::new(Aabb::from_corners(Vec3::new(-1.0, -1.0, 0.0), Vec3::new(1.0, 1.0, 2.0)).face_triangles().to_vec()).unwrap();
        let mut scene = Scene3::default();
        scene.meshes.push(ShadedMesh::from_mesh(&mesh, [255, 255, 255, 255], MeshRole::Part));
        scene.volume = campanion_view::BuildVolume::centered_plate(200.0, 100.0, Some(80.0));
        let content = content_bounds(&scene).expect("content");
        assert!(content.max().x < 5.0);
        let all = scene.bounds().expect("all");
        assert!(all.max().x > 50.0);
    }
}
