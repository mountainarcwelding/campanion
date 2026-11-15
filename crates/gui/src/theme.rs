//! Dark, low-contrast colours used by the layout preview.
//!
//! The palette follows the approved mockups: a professional dark shell, a blue accent for the
//! active tab, and orange for user overrides.

use eframe::egui::{self, Color32, CornerRadius, Stroke, Visuals};

/// Window and panel fill.
pub const WINDOW: Color32 = Color32::from_rgb(14, 18, 24);
/// Slightly raised toolbar and cards.
pub const PANEL: Color32 = Color32::from_rgb(20, 26, 34);
/// Inputs and tree rows.
pub const WIDGET: Color32 = Color32::from_rgb(28, 35, 45);
/// Subtle hairline between panels.
pub const BORDER: Color32 = Color32::from_rgb(42, 51, 64);
/// Primary text.
pub const TEXT: Color32 = Color32::from_rgb(216, 222, 232);
/// Secondary labels.
pub const MUTED: Color32 = Color32::from_rgb(139, 149, 163);
/// Active tab and primary actions.
pub const ACCENT: Color32 = Color32::from_rgb(59, 130, 246);
/// Hovered accent.
pub const ACCENT_HOVER: Color32 = Color32::from_rgb(96, 165, 250);
/// Override marker and some timeline segments.
pub const OVERRIDE: Color32 = Color32::from_rgb(245, 158, 11);
/// The pinned STOP button.
pub const STOP: Color32 = Color32::from_rgb(220, 38, 38);
/// Start on the Run tab.
pub const START: Color32 = Color32::from_rgb(22, 163, 74);
/// Viewport fill, a little lighter than the chrome so the grid reads.
pub const VIEWPORT: Color32 = Color32::from_rgb(18, 22, 30);
/// Grid lines inside the viewport.
pub const GRID: Color32 = Color32::from_rgb(46, 54, 66);
/// Toolpath cuts, matching `campanion-view`.
pub const CUT: Color32 = Color32::from_rgb(90, 200, 250);
/// Rapids, matching `campanion-view`.
pub const RAPID: Color32 = Color32::from_rgb(150, 150, 150);
/// Vertical plunges.
pub const PLUNGE: Color32 = Color32::from_rgb(255, 150, 40);
/// Playhead and tool mark.
pub const TOOL: Color32 = Color32::from_rgb(255, 255, 255);
/// Light-theme window fill.
pub const LIGHT_WINDOW: Color32 = Color32::from_rgb(236, 239, 244);
/// Light-theme panel fill.
pub const LIGHT_PANEL: Color32 = Color32::from_rgb(247, 249, 252);
/// Light-theme text.
pub const LIGHT_TEXT: Color32 = Color32::from_rgb(28, 35, 45);

/// Apply the shell theme to an egui context.
pub fn apply(ctx: &egui::Context, dark: bool) {
    let mut visuals = if dark { Visuals::dark() } else { Visuals::light() };
    let window = if dark { WINDOW } else { LIGHT_WINDOW };
    let panel = if dark { PANEL } else { LIGHT_PANEL };
    let text = if dark { TEXT } else { LIGHT_TEXT };
    let widget = if dark { WIDGET } else { Color32::from_rgb(255, 255, 255) };
    let border = if dark { BORDER } else { Color32::from_rgb(196, 204, 214) };
    visuals.override_text_color = Some(text);
    visuals.window_fill = window;
    visuals.panel_fill = panel;
    visuals.extreme_bg_color = widget;
    visuals.faint_bg_color = if dark { Color32::from_rgb(24, 30, 40) } else { Color32::from_rgb(230, 234, 240) };
    visuals.widgets.noninteractive.bg_fill = panel;
    visuals.widgets.noninteractive.weak_bg_fill = panel;
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, text);
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, border);
    visuals.widgets.inactive.bg_fill = widget;
    visuals.widgets.inactive.weak_bg_fill = widget;
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, text);
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, border);
    visuals.widgets.hovered.bg_fill = if dark { Color32::from_rgb(38, 48, 62) } else { Color32::from_rgb(226, 234, 246) };
    visuals.widgets.hovered.weak_bg_fill = visuals.widgets.hovered.bg_fill;
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT_HOVER);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.0, text);
    visuals.widgets.active.bg_fill = ACCENT;
    visuals.widgets.active.weak_bg_fill = ACCENT;
    visuals.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    visuals.widgets.active.fg_stroke = Stroke::new(1.0, Color32::WHITE);
    visuals.selection.bg_fill = ACCENT.gamma_multiply(0.45);
    visuals.selection.stroke = Stroke::new(1.0, ACCENT);
    visuals.hyperlink_color = ACCENT_HOVER;
    visuals.window_stroke = Stroke::new(1.0, border);
    visuals.window_corner_radius = CornerRadius::same(6);
    visuals.menu_corner_radius = CornerRadius::same(6);
    visuals.widgets.inactive.corner_radius = CornerRadius::same(4);
    visuals.widgets.hovered.corner_radius = CornerRadius::same(4);
    visuals.widgets.active.corner_radius = CornerRadius::same(4);
    ctx.set_visuals(visuals);
    ctx.set_theme(if dark { egui::ThemePreference::Dark } else { egui::ThemePreference::Light });
}

/// Fill used for an overridden numeric field.
pub fn override_fill(dark: bool) -> Color32 {
    if dark { Color32::from_rgb(58, 42, 16) } else { Color32::from_rgb(255, 236, 200) }
}
