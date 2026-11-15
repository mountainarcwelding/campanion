//! Toolpath painting that reuses [`campanion_view`] scene data.

use campanion_view::{Playback, Projection, Scene, Stroke, StrokeKind, View, project, projected_bounds, tool_position, visible};
use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke as UiStroke};

use crate::theme;

const FIT_MARGIN_PX: f64 = 36.0;
const GRID_MM: f64 = 10.0;
const GRID_MIN_PX: f64 = 8.0;
const MAX_GRID_LINES: usize = 4_096;
const SCROLL_ZOOM: f64 = 0.0015;

/// Camera and playback for a loaded scene.
pub struct Viewport {
    /// World-to-screen transform.
    pub view: View,
    /// Playback along the toolpath, in millimetres.
    pub playback: Playback,
    /// Plan or isometric projection.
    pub projection: Projection,
    /// Whether a fit has been applied for the current viewport size.
    pub fitted: bool,
    /// Draw rapid moves as dashed lines.
    pub show_rapids: bool,
}

impl Viewport {
    /// Frame the whole path, paused.
    pub fn new(scene: &Scene) -> Self {
        Self {
            view: View { center: [0.0, 0.0], scale: 1.0 },
            playback: Playback::new(scene.total_length),
            projection: Projection::Top,
            fitted: false,
            show_rapids: true,
        }
    }

    /// Fit the path into `rect`.
    pub fn fit(&mut self, scene: &Scene, rect: Rect) {
        let bounds = projected_bounds(scene, self.projection).or(scene.bounds);
        if let Some(bounds) = bounds {
            self.view = View::fit(bounds, f64::from(rect.width()), f64::from(rect.height()), FIT_MARGIN_PX);
        } else {
            self.view = View { center: [0.0, 0.0], scale: 1.0 };
        }
        self.fitted = true;
    }
}

/// Draw the toolpath into `ui` and handle zoom, pan and fit.
pub fn show(ui: &mut egui::Ui, viewport: &mut Viewport, scene: Option<&Scene>) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::VIEWPORT);

    if let Some(scene) = scene {
        if !viewport.fitted || response.double_clicked() {
            viewport.fit(scene, rect);
        }
        if response.hovered() {
            let zoom = ui.input(|input| input.smooth_scroll_delta.y);
            if zoom.abs() > f32::EPSILON {
                let pointer = response.hover_pos().unwrap_or(rect.center());
                let screen = [f64::from(pointer.x - rect.left()), f64::from(pointer.y - rect.top())];
                let factor = (1.0 + f64::from(zoom) * SCROLL_ZOOM).max(0.1);
                viewport.view.zoom_at(factor, screen, f64::from(rect.width()), f64::from(rect.height()));
            }
        }
        if response.dragged() {
            let delta = response.drag_delta();
            viewport.view.pan(f64::from(delta.x), f64::from(delta.y));
        }
        if ui.input(|input| input.key_pressed(egui::Key::F)) {
            viewport.fit(scene, rect);
        }
        draw_grid(&painter, &viewport.view, rect, viewport.projection);
        let shown = visible(scene, viewport.playback.position);
        for stroke in &shown {
            draw_stroke(&painter, &viewport.view, rect, stroke, viewport.show_rapids, viewport.projection);
        }
        if let Some(tool) = tool_position(scene, viewport.playback.position) {
            let z = scene.z_range.map(|(lo, _)| lo).unwrap_or(0.0);
            let point = to_screen(&viewport.view, rect, project(tool, z, viewport.projection));
            let size = 6.0;
            painter.line_segment([Pos2::new(point.x - size, point.y), Pos2::new(point.x + size, point.y)], UiStroke::new(1.2, theme::TOOL));
            painter.line_segment([Pos2::new(point.x, point.y - size), Pos2::new(point.x, point.y + size)], UiStroke::new(1.2, theme::TOOL));
        }
        for label in &scene.labels {
            let pos = to_screen(&viewport.view, rect, project(label.at, 0.0, viewport.projection));
            painter.text(pos, egui::Align2::CENTER_BOTTOM, &label.text, egui::FontId::proportional(11.0), theme::TEXT);
        }
    } else {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "Open an IR file to preview the toolpath\nFile → Open job…",
            egui::FontId::proportional(14.0),
            theme::MUTED,
        );
    }

    response
}

fn draw_stroke(painter: &egui::Painter, view: &View, rect: Rect, stroke: &Stroke, show_rapids: bool, projection: Projection) {
    let points = screen_points(view, rect, stroke, projection);
    if points.len() < 2 {
        return;
    }
    match stroke.kind {
        StrokeKind::Rapid => {
            if show_rapids {
                painter.extend(egui::Shape::dashed_line(&points, UiStroke::new(1.0, theme::RAPID), 6.0, 4.0));
            }
        }
        StrokeKind::Cut => {
            let _ = painter.add(egui::Shape::line(points, UiStroke::new(1.6, theme::CUT)));
        }
        StrokeKind::Plunge => {
            let _ = painter.add(egui::Shape::line(points, UiStroke::new(1.4, theme::PLUNGE)));
        }
    }
}

fn screen_points(view: &View, rect: Rect, stroke: &Stroke, projection: Projection) -> Vec<Pos2> {
    let mut points = Vec::new();
    for (index, point) in stroke.points.iter().enumerate() {
        let z = stroke.zs.get(index).copied().filter(|value| value.is_finite()).unwrap_or(stroke.z_start);
        points.push(to_screen(view, rect, project(*point, z, projection)));
    }
    points
}

fn to_screen(view: &View, rect: Rect, point: [f64; 2]) -> Pos2 {
    let screen = view.to_screen(point, f64::from(rect.width()), f64::from(rect.height()));
    Pos2::new(rect.left() + screen[0] as f32, rect.top() + screen[1] as f32)
}

fn draw_grid(painter: &egui::Painter, view: &View, rect: Rect, projection: Projection) {
    if !view.scale.is_finite() || view.scale * GRID_MM < GRID_MIN_PX {
        return;
    }
    let Some((min, max)) = grid_world(view, rect, projection) else {
        return;
    };
    let stroke = UiStroke::new(1.0, theme::GRID);
    draw_grid_axis(painter, view, rect, min, max, true, stroke, projection);
    draw_grid_axis(painter, view, rect, min, max, false, stroke, projection);
    let x_axis = UiStroke::new(1.0, Color32::from_rgb(190, 80, 80));
    let y_axis = UiStroke::new(1.0, Color32::from_rgb(70, 170, 90));
    draw_segment(painter, view, rect, [min[0], 0.0], [max[0], 0.0], projection, x_axis);
    draw_segment(painter, view, rect, [0.0, min[1]], [0.0, max[1]], projection, y_axis);
}

fn draw_grid_axis(
    painter: &egui::Painter,
    view: &View,
    rect: Rect,
    min: [f64; 2],
    max: [f64; 2],
    vertical: bool,
    stroke: UiStroke,
    projection: Projection,
) {
    let (lo, hi) = if vertical { (min[0], max[0]) } else { (min[1], max[1]) };
    let Some(mut cursor) = grid_start(lo) else {
        return;
    };
    let mut count = 0;
    while cursor <= hi + 1e-9 && count < MAX_GRID_LINES {
        let (start, end) = if vertical { ([cursor, min[1]], [cursor, max[1]]) } else { ([min[0], cursor], [max[0], cursor]) };
        draw_segment(painter, view, rect, start, end, projection, stroke);
        let next = cursor + GRID_MM;
        if !(next > cursor) {
            break;
        }
        cursor = next;
        count += 1;
    }
}

fn draw_segment(painter: &egui::Painter, view: &View, rect: Rect, start: [f64; 2], end: [f64; 2], projection: Projection, stroke: UiStroke) {
    let a = to_screen(view, rect, project(start, 0.0, projection));
    let b = to_screen(view, rect, project(end, 0.0, projection));
    painter.line_segment([a, b], stroke);
}

fn grid_world(view: &View, rect: Rect, projection: Projection) -> Option<([f64; 2], [f64; 2])> {
    let w = f64::from(rect.width());
    let h = f64::from(rect.height());
    if w <= 0.0 || h <= 0.0 {
        return None;
    }
    let corners = [[0.0, 0.0], [w, 0.0], [0.0, h], [w, h]];
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    for corner in corners {
        let world = view.to_world(corner, w, h);
        // Inverse of a plan projection is the point itself. Iso uses the same screen corners.
        let _ = projection;
        min[0] = min[0].min(world[0]);
        min[1] = min[1].min(world[1]);
        max[0] = max[0].max(world[0]);
        max[1] = max[1].max(world[1]);
    }
    if min[0].is_finite() { Some((min, max)) } else { None }
}

fn grid_start(min: f64) -> Option<f64> {
    if !min.is_finite() {
        return None;
    }
    let start = (min / GRID_MM).floor() * GRID_MM;
    if start.is_finite() { Some(start) } else { None }
}
