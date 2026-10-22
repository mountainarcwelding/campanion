//! Plan-view window for a CAMpanion IR file.
//!
//! `campanion-view <ir.json> [--arc-tol MM]` loads a program and draws its toolpath. Distances
//! are millimetres. The window fits the path; the wheel zooms, drag pans, and F or a double-click
//! fits again.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

use std::ffi::{OsStr, OsString};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;

use campanion_view::{Playback, Scene, Stroke, StrokeKind, View, find_system_font, load_scene, tool_position, visible};
use eframe::egui;

const DEFAULT_ARC_TOL_MM: f64 = 0.01;
const MAX_JSON_BYTES: u64 = 32 * 1024 * 1024;
const FIT_MARGIN_PX: f64 = 40.0;
const GRID_MM: f64 = 10.0;
const GRID_MIN_PX: f64 = 8.0;
const MAX_GRID_LINES: usize = 4_096;
const SCROLL_ZOOM: f64 = 0.0015;
const LENGTH_EPS_MM: f64 = 1e-9;

const BACKGROUND: egui::Color32 = egui::Color32::from_rgb(24, 26, 30);
const GRID_COLOR: egui::Color32 = egui::Color32::from_rgb(46, 50, 58);
const AXIS_X: egui::Color32 = egui::Color32::from_rgb(190, 80, 80);
const AXIS_Y: egui::Color32 = egui::Color32::from_rgb(70, 170, 90);
const CUT_COLOR: egui::Color32 = egui::Color32::from_rgb(90, 200, 250);
const RAPID_COLOR: egui::Color32 = egui::Color32::from_rgb(150, 150, 150);
const ORANGE: egui::Color32 = egui::Color32::from_rgb(255, 150, 40);
const TOOL_COLOR: egui::Color32 = egui::Color32::from_rgb(255, 255, 255);
const LABEL_COLOR: egui::Color32 = egui::Color32::from_rgb(236, 236, 236);

fn main() -> ExitCode {
    match try_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(AppError::Usage(message)) => {
            eprintln!("campanion-view: {message}");
            eprintln!("usage: campanion-view <ir.json> [--arc-tol MM]");
            ExitCode::from(2)
        }
        Err(AppError::Run(message)) => {
            eprintln!("campanion-view: {message}");
            ExitCode::from(1)
        }
    }
}

fn try_main() -> Result<(), AppError> {
    let args: Vec<OsString> = std::env::args_os().skip(1).collect();
    match parse_args(&args)? {
        Command::Help => write_stdout(&help_text()),
        Command::Version => write_stdout(&version_text()),
        Command::Preview { path, arc_tol } => {
            let json = read_ir(&path)?;
            let scene = load_scene(&json, arc_tol).map_err(|err| AppError::run(format!("{}: {err}", path.display())))?;
            open_preview(preview_title(&path), scene)
        }
    }
}

enum Command {
    Help,
    Version,
    Preview { path: PathBuf, arc_tol: f64 },
}

enum AppError {
    Usage(String),
    Run(String),
}

impl AppError {
    fn usage(message: impl Into<String>) -> Self {
        Self::Usage(message.into())
    }

    fn run(message: impl Into<String>) -> Self {
        Self::Run(message.into())
    }
}

fn help_text() -> String {
    format!(
        "\
campanion-view {version}

Usage:
  campanion-view <ir.json> [--arc-tol MM]
  campanion-view --help
  campanion-view --version

Open a plan-view preview of a CAMpanion IR file.
Distances are millimetres. Scroll zooms at the cursor, drag pans, and F or a
double-click fits the path in the window. Space toggles playback.

--arc-tol MM
        Maximum chord error when drawing arcs, in millimetres (default 0.01).
",
        version = env!("CARGO_PKG_VERSION"),
    )
}

fn version_text() -> String {
    format!("campanion-view {}\n", env!("CARGO_PKG_VERSION"))
}

fn parse_args(args: &[OsString]) -> Result<Command, AppError> {
    if args.iter().any(is_help) {
        return Ok(Command::Help);
    }
    if args.iter().any(is_version) {
        if args.len() == 1 {
            return Ok(Command::Version);
        }
        return Err(AppError::usage("--version does not take arguments"));
    }

    let mut path: Option<PathBuf> = None;
    let mut arc_tol = DEFAULT_ARC_TOL_MM;
    let mut index = 0;
    while let Some(arg) = args.get(index) {
        if arg.as_os_str() == "--arc-tol" {
            let Some(value) = args.get(index + 1) else {
                return Err(AppError::usage("--arc-tol needs a value in millimetres"));
            };
            arc_tol = parse_arc_tol(value)?;
            index += 2;
            continue;
        }
        if let Some(text) = arg.to_str()
            && let Some(value) = text.strip_prefix("--arc-tol=")
        {
            arc_tol = parse_arc_tol_text(value)?;
            index += 1;
            continue;
        }
        if arg.to_str().is_some_and(|text| text.starts_with('-')) {
            return Err(AppError::usage(format!("unknown argument {}", arg.to_string_lossy())));
        }
        if path.is_some() {
            return Err(AppError::usage("unexpected extra argument"));
        }
        if arg.is_empty() {
            return Err(AppError::usage("missing IR json file"));
        }
        path = Some(PathBuf::from(arg));
        index += 1;
    }
    let Some(path) = path else {
        return Err(AppError::usage("missing IR json file"));
    };
    Ok(Command::Preview { path, arc_tol })
}

fn is_help(arg: &OsString) -> bool {
    let arg = arg.as_os_str();
    arg == "--help" || arg == "-h"
}

fn is_version(arg: &OsString) -> bool {
    let arg = arg.as_os_str();
    arg == "--version" || arg == "-V"
}

fn parse_arc_tol(value: &OsStr) -> Result<f64, AppError> {
    let Some(text) = value.to_str() else {
        return Err(AppError::usage("--arc-tol value is not valid UTF-8"));
    };
    parse_arc_tol_text(text)
}

fn parse_arc_tol_text(text: &str) -> Result<f64, AppError> {
    let Ok(value) = text.parse::<f64>() else {
        return Err(AppError::usage(format!("--arc-tol needs a positive number of millimetres (got {text})")));
    };
    if !value.is_finite() || value <= 0.0 {
        return Err(AppError::usage(format!("--arc-tol needs a positive finite number of millimetres (got {text})")));
    }
    Ok(value)
}

fn read_ir(path: &Path) -> Result<String, AppError> {
    if let Ok(meta) = std::fs::metadata(path)
        && meta.is_file()
        && meta.len() > MAX_JSON_BYTES
    {
        return Err(too_large(path));
    }
    let file = std::fs::File::open(path).map_err(|err| AppError::run(format!("{}: {err}", path.display())))?;
    let mut file = file.take(MAX_JSON_BYTES + 1);
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).map_err(|err| AppError::run(format!("{}: {err}", path.display())))?;
    if bytes.len() as u64 > MAX_JSON_BYTES {
        return Err(too_large(path));
    }
    String::from_utf8(bytes).map_err(|_| AppError::run(format!("{} is not valid UTF-8", path.display())))
}

fn too_large(path: &Path) -> AppError {
    AppError::run(format!("{} is larger than {MAX_JSON_BYTES} bytes", path.display()))
}

fn write_stdout(text: &str) -> Result<(), AppError> {
    let mut out = io::stdout().lock();
    out.write_all(text.as_bytes()).map_err(|err| AppError::run(err.to_string()))?;
    if !text.ends_with('\n') {
        writeln!(out).map_err(|err| AppError::run(err.to_string()))?;
    }
    out.flush().map_err(|err| AppError::run(err.to_string()))?;
    Ok(())
}

fn preview_title(path: &Path) -> String {
    let name = path.file_name().map_or_else(|| path.to_string_lossy(), |name| name.to_string_lossy());
    format!("CAMpanion preview — {name}")
}

fn open_preview(title: String, scene: Scene) -> Result<(), AppError> {
    let native_options =
        eframe::NativeOptions { viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 800.0]).with_title(title), ..Default::default() };
    eframe::run_native(
        "campanion-view",
        native_options,
        Box::new(move |cc| {
            cc.egui_ctx.set_theme(egui::ThemePreference::Dark);
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let has_font = install_font(&cc.egui_ctx);
            Ok(Box::new(ViewerApp::new(scene, has_font)))
        }),
    )
    .map_err(|err| AppError::run(err.to_string()))
}

/// Load one system face. Built-in fonts are disabled, so order labels need this file.
fn install_font(ctx: &egui::Context) -> bool {
    let Some(path) = find_system_font() else {
        eprintln!("campanion-view: no font found; labels need a font (set CAMPANION_VIEW_FONT)");
        return false;
    };
    let bytes = match std::fs::read(&path) {
        Ok(bytes) if !bytes.is_empty() => bytes,
        Ok(_) => {
            eprintln!("campanion-view: font {} is empty; labels need a font (set CAMPANION_VIEW_FONT)", path.display());
            return false;
        }
        Err(err) => {
            eprintln!("campanion-view: could not read font {}: {err}; labels need a font (set CAMPANION_VIEW_FONT)", path.display());
            return false;
        }
    };
    let mut fonts = egui::FontDefinitions::empty();
    fonts.font_data.insert("sys".into(), Arc::new(egui::FontData::from_owned(bytes)));
    fonts.families.insert(egui::FontFamily::Proportional, vec!["sys".into()]);
    fonts.families.insert(egui::FontFamily::Monospace, vec!["sys".into()]);
    ctx.set_fonts(fonts);
    true
}

struct ViewerApp {
    scene: Scene,
    playback: Playback,
    view: View,
    fitted: bool,
    show_rapids: bool,
    show_order: bool,
    has_font: bool,
    toolpaths: usize,
    cut_length: f64,
    rapid_length: f64,
}

impl ViewerApp {
    fn new(scene: Scene, has_font: bool) -> Self {
        let (cut_length, rapid_length) = path_lengths(&scene);
        let toolpaths = scene.names.len();
        let playback = Playback::new(scene.total_length);
        Self {
            scene,
            playback,
            view: View { center: [0.0, 0.0], scale: 1.0 },
            fitted: false,
            show_rapids: true,
            show_order: true,
            has_font,
            toolpaths,
            cut_length,
            rapid_length,
        }
    }

    fn toggle_play(&mut self) {
        let total = finite_total(self.scene.total_length);
        if self.playback.playing {
            self.playback.playing = false;
            return;
        }
        if self.playback.position >= total {
            self.playback.restart();
            return;
        }
        self.playback.playing = true;
    }

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        let total = finite_total(self.scene.total_length);
        egui::Panel::top("bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                let play = if self.playback.playing { "Pause" } else { "Play" };
                if ui.button(play).clicked() {
                    self.toggle_play();
                }
                if ui.button("Restart").clicked() {
                    self.playback.restart();
                }
                ui.label("Speed");
                ui.spacing_mut().slider_width = 120.0;
                let speed_max = total.max(1.0);
                clamp_playback(&mut self.playback, total, speed_max);
                ui.add(egui::Slider::new(&mut self.playback.speed, 1.0..=speed_max).text("mm/s"));
                ui.label("Position");
                ui.spacing_mut().slider_width = 200.0;
                ui.add(egui::Slider::new(&mut self.playback.position, 0.0..=total).text("mm"));
                ui.checkbox(&mut self.show_rapids, "Rapids");
                ui.checkbox(&mut self.show_order, "Order");
                ui.label(self.status_text());
            });
        });
    }

    fn status_text(&self) -> String {
        let paths = if self.toolpaths == 1 { "1 toolpath".to_string() } else { format!("{} toolpaths", self.toolpaths) };
        format!("{paths}   cut {:.1} mm   rapid {:.1} mm", self.cut_length, self.rapid_length)
    }

    fn canvas(&mut self, ui: &mut egui::Ui, input: &FrameInput) {
        egui::CentralPanel::default().show(ui, |ui| {
            let (response, mut painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
            let rect = response.rect;
            self.apply_view(&response, rect, input);
            self.paint(&mut painter, rect);
        });
    }

    fn apply_view(&mut self, response: &egui::Response, rect: egui::Rect, input: &FrameInput) {
        let refit = !self.fitted || input.fit || response.double_clicked();
        if refit {
            self.fit_view(rect);
            return;
        }
        if response.dragged() {
            let delta = response.drag_delta();
            self.view.pan(f64::from(delta.x), f64::from(delta.y));
        }
        self.zoom_at_cursor(response, rect, input);
    }

    fn fit_view(&mut self, rect: egui::Rect) {
        let width = f64::from(rect.width());
        let height = f64::from(rect.height());
        if width <= 1.0 || height <= 1.0 {
            return;
        }
        let bounds = self.scene.bounds.unwrap_or(([0.0, 0.0], [0.0, 0.0]));
        self.view = View::fit(bounds, width, height, FIT_MARGIN_PX);
        self.fitted = true;
    }

    fn zoom_at_cursor(&mut self, response: &egui::Response, rect: egui::Rect, input: &FrameInput) {
        let Some(hover) = response.hover_pos() else {
            return;
        };
        let scroll = f64::from(input.scroll_y);
        let pinch = f64::from(input.zoom_delta);
        let pinch = if pinch.is_finite() && pinch > 0.0 { pinch } else { 1.0 };
        let factor = (scroll * SCROLL_ZOOM).exp() * pinch;
        if !factor.is_finite() || (factor - 1.0).abs() <= 1.0e-4 {
            return;
        }
        let width = f64::from(rect.width());
        let height = f64::from(rect.height());
        let screen = [f64::from(hover.x - rect.min.x), f64::from(hover.y - rect.min.y)];
        self.view.zoom_at(factor, screen, width, height);
    }

    fn paint(&self, painter: &mut egui::Painter, rect: egui::Rect) {
        painter.set_clip_rect(rect);
        painter.rect_filled(rect, 0.0, BACKGROUND);
        draw_grid(painter, &self.view, rect);
        draw_axes(painter, &self.view, rect);
        let shown = visible(&self.scene, self.playback.position);
        for stroke in &shown {
            draw_stroke(painter, &self.view, rect, stroke, self.show_rapids);
        }
        draw_pierces(painter, &self.scene, shown.len(), &self.view, rect);
        draw_tool(painter, &self.scene, self.playback.position, &self.view, rect);
        if self.has_font && self.show_order {
            draw_labels(painter, &self.scene, &self.view, rect);
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let input = read_input(ui);
        if input.space {
            self.toggle_play();
        }
        let total = finite_total(self.scene.total_length);
        self.playback.step(f64::from(input.dt), total);
        self.top_bar(ui);
        self.canvas(ui, &input);
        if self.playback.playing {
            ui.ctx().request_repaint();
        }
    }
}

struct FrameInput {
    scroll_y: f32,
    zoom_delta: f32,
    dt: f32,
    space: bool,
    fit: bool,
}

fn read_input(ui: &egui::Ui) -> FrameInput {
    ui.input_mut(|input| FrameInput {
        scroll_y: input.smooth_scroll_delta.y,
        zoom_delta: input.zoom_delta(),
        dt: input.stable_dt,
        space: input.consume_key(egui::Modifiers::NONE, egui::Key::Space),
        fit: input.key_pressed(egui::Key::F),
    })
}

fn path_lengths(scene: &Scene) -> (f64, f64) {
    let mut cut = 0.0;
    let mut rapid = 0.0;
    for stroke in &scene.strokes {
        let length = if stroke.length.is_finite() && stroke.length > 0.0 { stroke.length } else { 0.0 };
        match stroke.kind {
            StrokeKind::Cut => cut += length,
            StrokeKind::Rapid => rapid += length,
            StrokeKind::Plunge => {}
        }
    }
    (cut, rapid)
}

fn finite_total(total: f64) -> f64 {
    if total.is_finite() && total > 0.0 { total } else { 0.0 }
}

fn clamp_playback(playback: &mut Playback, total: f64, speed_max: f64) {
    if !playback.position.is_finite() || playback.position < 0.0 {
        playback.position = 0.0;
    } else if playback.position > total {
        playback.position = total;
    }
    if !playback.speed.is_finite() || playback.speed < 1.0 {
        playback.speed = 1.0;
    } else if playback.speed > speed_max {
        playback.speed = speed_max;
    }
}

fn draw_grid(painter: &egui::Painter, view: &View, rect: egui::Rect) {
    if !view.scale.is_finite() || view.scale * GRID_MM < GRID_MIN_PX {
        return;
    }
    let Some((min, max)) = world_span(view, rect) else {
        return;
    };
    let stroke = egui::Stroke::new(1.0, GRID_COLOR);
    draw_grid_axis(painter, view, rect, min, max, true, stroke);
    draw_grid_axis(painter, view, rect, min, max, false, stroke);
}

fn draw_grid_axis(painter: &egui::Painter, view: &View, rect: egui::Rect, min: [f64; 2], max: [f64; 2], vertical: bool, stroke: egui::Stroke) {
    let (lo, hi) = if vertical { (min[0], max[0]) } else { (min[1], max[1]) };
    let Some(mut cursor) = grid_start(lo) else {
        return;
    };
    let mut count = 0;
    while cursor <= hi + LENGTH_EPS_MM && count < MAX_GRID_LINES {
        let (start, end) = if vertical { ([cursor, min[1]], [cursor, max[1]]) } else { ([min[0], cursor], [max[0], cursor]) };
        draw_world_segment(painter, view, rect, start, end, stroke);
        let next = cursor + GRID_MM;
        if !(next > cursor) {
            break;
        }
        cursor = next;
        count += 1;
    }
}

fn grid_start(min: f64) -> Option<f64> {
    if !min.is_finite() {
        return None;
    }
    let start = (min / GRID_MM).floor() * GRID_MM;
    if start.is_finite() { Some(start) } else { None }
}

fn draw_axes(painter: &egui::Painter, view: &View, rect: egui::Rect) {
    let Some((min, max)) = world_span(view, rect) else {
        return;
    };
    draw_world_segment(painter, view, rect, [min[0], 0.0], [max[0], 0.0], egui::Stroke::new(1.0, AXIS_X));
    draw_world_segment(painter, view, rect, [0.0, min[1]], [0.0, max[1]], egui::Stroke::new(1.0, AXIS_Y));
}

fn draw_stroke(painter: &egui::Painter, view: &View, rect: egui::Rect, stroke: &Stroke, show_rapids: bool) {
    match stroke.kind {
        StrokeKind::Rapid => {
            if !show_rapids {
                return;
            }
            let points = screen_points(view, rect, &stroke.points);
            if points.len() >= 2 {
                painter.extend(egui::Shape::dashed_line(&points, egui::Stroke::new(1.0, RAPID_COLOR), 6.0, 4.0));
            }
        }
        StrokeKind::Cut => {
            let points = screen_points(view, rect, &stroke.points);
            if points.len() >= 2 {
                let _ = painter.add(egui::Shape::line(points, egui::Stroke::new(1.5, CUT_COLOR)));
            }
        }
        StrokeKind::Plunge => {
            if let Some(point) = stroke.points.first()
                && let Some(pos) = to_screen_pos(view, rect, *point)
            {
                painter.circle_filled(pos, 2.0, ORANGE);
            }
        }
    }
}

/// A pierce belongs to the stroke that follows it. Draw it once that stroke is in view.
/// An index past the last stroke waits until every stroke is shown.
fn draw_pierces(painter: &egui::Painter, scene: &Scene, shown: usize, view: &View, rect: egui::Rect) {
    for marker in &scene.pierces {
        if !pierce_visible(marker.stroke_index, scene.strokes.len(), shown) {
            continue;
        }
        if let Some(pos) = to_screen_pos(view, rect, marker.at) {
            painter.circle_filled(pos, 3.5, ORANGE);
        }
    }
}

fn pierce_visible(stroke_index: usize, stroke_count: usize, shown: usize) -> bool {
    if stroke_index < stroke_count { stroke_index < shown } else { shown == stroke_count }
}

fn draw_tool(painter: &egui::Painter, scene: &Scene, position: f64, view: &View, rect: egui::Rect) {
    let Some(at) = tool_position(scene, position) else {
        return;
    };
    let Some(pos) = to_screen_pos(view, rect, at) else {
        return;
    };
    painter.circle_stroke(pos, 6.0, egui::Stroke::new(1.5, TOOL_COLOR));
}

fn draw_labels(painter: &egui::Painter, scene: &Scene, view: &View, rect: egui::Rect) {
    for label in &scene.labels {
        let Some(pos) = to_screen_pos(view, rect, label.at) else {
            continue;
        };
        let _ =
            painter.text(pos + egui::vec2(6.0, -4.0), egui::Align2::LEFT_BOTTOM, label.text.as_str(), egui::FontId::proportional(14.0), LABEL_COLOR);
    }
}

fn draw_world_segment(painter: &egui::Painter, view: &View, rect: egui::Rect, start: [f64; 2], end: [f64; 2], stroke: egui::Stroke) {
    let (Some(a), Some(b)) = (to_screen_pos(view, rect, start), to_screen_pos(view, rect, end)) else {
        return;
    };
    painter.line_segment([a, b], stroke);
}

fn screen_points(view: &View, rect: egui::Rect, points: &[[f64; 2]]) -> Vec<egui::Pos2> {
    let mut screen = Vec::with_capacity(points.len());
    for point in points {
        if let Some(pos) = to_screen_pos(view, rect, *point) {
            screen.push(pos);
        }
    }
    screen
}

fn to_screen_pos(view: &View, rect: egui::Rect, world: [f64; 2]) -> Option<egui::Pos2> {
    let width = f64::from(rect.width());
    let height = f64::from(rect.height());
    let screen = view.to_screen(world, width, height);
    if !screen[0].is_finite() || !screen[1].is_finite() {
        return None;
    }
    let x = rect.min.x + screen[0] as f32;
    let y = rect.min.y + screen[1] as f32;
    if !x.is_finite() || !y.is_finite() {
        return None;
    }
    Some(egui::pos2(x, y))
}

fn world_span(view: &View, rect: egui::Rect) -> Option<([f64; 2], [f64; 2])> {
    let width = f64::from(rect.width());
    let height = f64::from(rect.height());
    if !(width.is_finite() && height.is_finite() && width > 0.0 && height > 0.0) {
        return None;
    }
    let left_top = view.to_world([0.0, 0.0], width, height);
    let right_bottom = view.to_world([width, height], width, height);
    if ![left_top[0], left_top[1], right_bottom[0], right_bottom[1]].into_iter().all(f64::is_finite) {
        return None;
    }
    Some(([left_top[0].min(right_bottom[0]), left_top[1].min(right_bottom[1])], [left_top[0].max(right_bottom[0]), left_top[1].max(right_bottom[1])]))
}
