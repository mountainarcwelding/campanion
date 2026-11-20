//! Toolpath painting that reuses [`campanion_view`] scene data.

use campanion_view::{Playback, Projection, Scene, Stroke, StrokeKind, View, project, projected_bounds, tool_position, visible};
use eframe::egui::{self, Align2, Color32, FontId, Pos2, Rect, Sense, Stroke as UiStroke, Vec2};

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
    /// True after the user pans or zooms, so a later layout change does not steal the view.
    pub user_adjusted: bool,
    /// Last rect size a fit was computed for.
    pub last_size: Option<[f32; 2]>,
    /// Draw rapid moves as dashed lines.
    pub show_rapids: bool,
}

/// Result of drawing the viewport for one frame.
pub struct ViewportOutput {
    /// Widget response.
    pub response: egui::Response,
    /// World-space delta, millimetres, from a left-drag that should move the part.
    pub part_delta: Option<[f64; 2]>,
}

/// Optional bed and sheet rectangles drawn in the viewport, millimetres, `[min_x, min_y, max_x, max_y]`.
#[derive(Debug, Clone, Default)]
pub struct ViewportGuides {
    pub bed: Option<[f64; 4]>,
    pub sheet: Option<[f64; 4]>,
}

impl Viewport {
    /// Frame the whole path, paused.
    pub fn new(scene: &Scene) -> Self {
        Self {
            view: View { center: [0.0, 0.0], scale: 1.0 },
            playback: Playback::new(scene.total_length),
            projection: Projection::Top,
            fitted: false,
            user_adjusted: false,
            last_size: None,
            show_rapids: true,
        }
    }

    /// Fit the path into `rect`.
    pub fn fit(&mut self, scene: &Scene, rect: Rect) {
        self.fit_with_guides(Some(scene), rect, None);
    }

    /// Fit the view to the machine bed when one is known, otherwise to the scene.
    pub fn fit_with_guides(&mut self, scene: Option<&Scene>, rect: Rect, guides: Option<&ViewportGuides>) {
        let bed = guides.and_then(|guides| guides.bed).map(|bounds| ([bounds[0], bounds[1]], [bounds[2], bounds[3]]));
        let scene_bounds = scene.and_then(|scene| scene.bounds.or(projected_bounds(scene, self.projection)));
        self.fit_world(bed.or(scene_bounds), rect);
    }

    fn fit_world(&mut self, bounds: Option<([f64; 2], [f64; 2])>, rect: Rect) {
        if rect.width() < 32.0 || rect.height() < 32.0 {
            return;
        }
        if let Some(bounds) = bounds {
            self.view = View::fit(bounds, f64::from(rect.width()), f64::from(rect.height()), FIT_MARGIN_PX);
        } else {
            self.view = View { center: [0.0, 0.0], scale: 1.0 };
        }
        self.fitted = true;
        self.last_size = Some([rect.width(), rect.height()]);
    }
}

/// Draw the toolpath into `ui` and handle zoom, pan and fit.
///
/// Left-drag moves the part when `allow_part_drag` is set. Shift+left-drag, Alt+left-drag, or
/// middle-drag pans the view.
pub fn show(ui: &mut egui::Ui, viewport: &mut Viewport, scene: Option<&Scene>, allow_part_drag: bool) -> ViewportOutput {
    show_with(ui, viewport, scene, allow_part_drag, None)
}

/// Draw the toolpath and optional bed/sheet guides.
pub fn show_with(
    ui: &mut egui::Ui,
    viewport: &mut Viewport,
    scene: Option<&Scene>,
    allow_part_drag: bool,
    guides: Option<&ViewportGuides>,
) -> ViewportOutput {
    let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::VIEWPORT);
    let mut part_delta = None;
    let has_guides = guides.is_some_and(|guides| guides.bed.is_some() || guides.sheet.is_some());

    if scene.is_some() || has_guides {
        let size = [rect.width(), rect.height()];
        let size_changed = viewport.last_size.is_none_or(|prev| (prev[0] - size[0]).abs() > 0.5 || (prev[1] - size[1]).abs() > 0.5);
        if response.double_clicked() {
            viewport.user_adjusted = false;
            viewport.fit_with_guides(scene, rect, guides);
        } else if !viewport.fitted || (size_changed && !viewport.user_adjusted) {
            viewport.fit_with_guides(scene, rect, guides);
        }
        if response.hovered() {
            let zoom = ui.input(|input| input.smooth_scroll_delta.y);
            if zoom.abs() > f32::EPSILON {
                let pointer = response.hover_pos().unwrap_or(rect.center());
                let screen = [f64::from(pointer.x - rect.left()), f64::from(pointer.y - rect.top())];
                let factor = (1.0 + f64::from(zoom) * SCROLL_ZOOM).max(0.1);
                viewport.view.zoom_at(factor, screen, f64::from(rect.width()), f64::from(rect.height()));
                viewport.user_adjusted = true;
            }
        }
        let pan_request = response.dragged_by(egui::PointerButton::Middle)
            || (response.dragged_by(egui::PointerButton::Primary) && ui.input(|input| input.modifiers.shift || input.modifiers.alt))
            || (response.dragged_by(egui::PointerButton::Primary) && !allow_part_drag);
        if pan_request {
            let delta = response.drag_delta();
            viewport.view.pan(f64::from(delta.x), f64::from(delta.y));
            viewport.user_adjusted = true;
        } else if allow_part_drag && response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            let width = f64::from(rect.width());
            let height = f64::from(rect.height());
            let before = viewport.view.to_world([0.0, 0.0], width, height);
            let after = viewport.view.to_world([f64::from(delta.x), f64::from(delta.y)], width, height);
            let dx = after[0] - before[0];
            let dy = after[1] - before[1];
            if dx.is_finite() && dy.is_finite() && (dx * dx + dy * dy) > 0.0 {
                part_delta = Some([dx, dy]);
            }
        }
        if ui.input(|input| input.key_pressed(egui::Key::F)) {
            viewport.user_adjusted = false;
            viewport.fit_with_guides(scene, rect, guides);
        }
        draw_grid(&painter, &viewport.view, rect, viewport.projection);
        if let Some(guides) = guides {
            if let Some(bed) = guides.bed {
                draw_guide(
                    &painter,
                    &viewport.view,
                    rect,
                    bed,
                    viewport.projection,
                    UiStroke::new(2.2, theme::BED),
                    theme::bed_fill(),
                    &format!("Bed {} × {} mm", format_mm(bed[2] - bed[0]), format_mm(bed[3] - bed[1])),
                );
            }
            if let Some(sheet) = guides.sheet {
                draw_guide(
                    &painter,
                    &viewport.view,
                    rect,
                    sheet,
                    viewport.projection,
                    UiStroke::new(2.0, theme::SHEET),
                    theme::sheet_fill(),
                    &format!("Sheet {} × {} mm", format_mm(sheet[2] - sheet[0]), format_mm(sheet[3] - sheet[1])),
                );
            }
        }
        if let Some(scene) = scene {
            let shown = visible(scene, viewport.playback.position);
            for stroke in &shown {
                let color = stroke_color(stroke, guides);
                draw_stroke_colored(&painter, &viewport.view, rect, stroke, viewport.show_rapids, viewport.projection, color);
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
                painter.text(pos, Align2::CENTER_BOTTOM, &label.text, FontId::proportional(11.0), theme::TEXT);
            }
        }
    } else {
        painter.text(
            rect.center(),
            egui::Align2::CENTER_CENTER,
            "Import a part or open a job to begin\nFile → Import part… or File → Open job…\nCAMpanion is primarily command-line driven; this GUI is still in development",
            egui::FontId::proportional(14.0),
            theme::MUTED,
        );
    }

    ViewportOutput { response, part_delta }
}

fn stroke_color(stroke: &Stroke, guides: Option<&ViewportGuides>) -> Color32 {
    if stroke.kind != StrokeKind::Cut {
        return if stroke.kind == StrokeKind::Plunge { theme::PLUNGE } else { theme::RAPID };
    }
    let Some(guides) = guides else { return theme::CUT };
    let Some(bounds) = stroke_bounds(stroke) else { return theme::CUT };
    if let Some(bed) = guides.bed
        && !crate::setup::inside_limits(bounds, [Some(bed[0]), Some(bed[1]), Some(bed[2]), Some(bed[3])])
    {
        return theme::OFF_BED;
    }
    if let Some(sheet) = guides.sheet
        && !crate::setup::inside_limits(bounds, [Some(sheet[0]), Some(sheet[1]), Some(sheet[2]), Some(sheet[3])])
    {
        return theme::OFF_SHEET;
    }
    theme::CUT
}

fn stroke_bounds(stroke: &Stroke) -> Option<[f64; 4]> {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for point in &stroke.points {
        min_x = min_x.min(point[0]);
        min_y = min_y.min(point[1]);
        max_x = max_x.max(point[0]);
        max_y = max_y.max(point[1]);
    }
    if min_x.is_finite() { Some([min_x, min_y, max_x, max_y]) } else { None }
}

fn draw_rect(painter: &egui::Painter, view: &View, rect: Rect, bounds: [f64; 4], projection: Projection, stroke: UiStroke) {
    let [x0, y0, x1, y1] = bounds;
    draw_segment(painter, view, rect, [x0, y0], [x1, y0], projection, stroke);
    draw_segment(painter, view, rect, [x1, y0], [x1, y1], projection, stroke);
    draw_segment(painter, view, rect, [x1, y1], [x0, y1], projection, stroke);
    draw_segment(painter, view, rect, [x0, y1], [x0, y0], projection, stroke);
}

fn draw_guide(
    painter: &egui::Painter,
    view: &View,
    rect: Rect,
    bounds: [f64; 4],
    projection: Projection,
    stroke: UiStroke,
    fill: Color32,
    label: &str,
) {
    let [x0, y0, x1, y1] = bounds;
    let a = to_screen(view, rect, project([x0, y0], 0.0, projection));
    let b = to_screen(view, rect, project([x1, y1], 0.0, projection));
    let area = Rect::from_two_pos(a, b);
    if area.width().abs() > 1.0 && area.height().abs() > 1.0 {
        painter.rect_filled(area, 0.0, fill);
    }
    draw_rect(painter, view, rect, bounds, projection, stroke);
    if !label.is_empty() {
        painter.text(area.left_top() + Vec2::new(6.0, 4.0), Align2::LEFT_TOP, label, FontId::proportional(12.0), stroke.color);
    }
}

fn format_mm(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    if (rounded - rounded.round()).abs() < 1e-9 { format!("{:.0}", rounded) } else { format!("{rounded}") }
}

fn draw_stroke_colored(painter: &egui::Painter, view: &View, rect: Rect, stroke: &Stroke, show_rapids: bool, projection: Projection, color: Color32) {
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
            let _ = painter.add(egui::Shape::line(points, UiStroke::new(1.6, color)));
        }
        StrokeKind::Plunge => {
            let _ = painter.add(egui::Shape::line(points, UiStroke::new(1.4, color)));
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

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_view::{Stroke, StrokeKind};

    fn cut_scene(points: Vec<[f64; 2]>) -> Scene {
        let mut min = [f64::INFINITY, f64::INFINITY];
        let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
        for point in &points {
            min[0] = min[0].min(point[0]);
            min[1] = min[1].min(point[1]);
            max[0] = max[0].max(point[0]);
            max[1] = max[1].max(point[1]);
        }
        let zs = vec![0.0; points.len()];
        Scene {
            strokes: vec![Stroke { kind: StrokeKind::Cut, toolpath: 0, points, zs, z_start: 0.0, z_end: 0.0, angles: Vec::new(), length: 1.0 }],
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: Some((min, max)),
            z_range: None,
            total_length: 1.0,
            names: vec!["part".into()],
            rotary: None,
            lathe: None,
            timeline: Vec::new(),
        }
    }

    #[test]
    fn fit_uses_the_bed_when_guides_are_present() {
        let scene = cut_scene(vec![[0.0, 0.0], [20.0, 10.0]]);
        let mut viewport = Viewport::new(&scene);
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(400.0, 300.0));
        let guides = ViewportGuides { bed: Some([0.0, 0.0, 600.0, 400.0]), sheet: Some([0.0, 0.0, 300.0, 200.0]) };
        viewport.fit_with_guides(Some(&scene), rect, Some(&guides));
        // A fit to the 20×10 part would use a much larger scale than a fit to the 600×400 bed.
        let bed_fit = View::fit(([0.0, 0.0], [600.0, 400.0]), 400.0, 300.0, FIT_MARGIN_PX);
        assert!((viewport.view.scale - bed_fit.scale).abs() < 1e-9);
        assert!((viewport.view.center[0] - 300.0).abs() < 1e-6);
        assert!((viewport.view.center[1] - 200.0).abs() < 1e-6);
    }
}
