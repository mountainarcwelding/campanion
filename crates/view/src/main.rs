//! Plan-view window for a CAMpanion IR file.
//!
//! `campanion-view <ir.json> [--arc-tol MM] [--iso] [--angle]` loads a program and draws its
//! toolpath. Distances are millimetres. The window fits the path; the wheel zooms, drag pans, and
//! F or a double-click fits again. `--iso` opens the isometric view. `--angle` colours cuts by
//! rotary angle instead of height; a program with no rotary axis ignores it. A toggles that
//! colouring. A rotary setup also draws its axis and lists index angles or the angle range.
//! A turning program is an XZ half-section (radius up, Z right): removed material, stock outline,
//! finished section, then the spindle line and the toolpath. `--iso` is ignored for that view.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

use std::ffi::{OsStr, OsString};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;

use campanion_view::{
    LathePreview, Playback, Projection, RotaryPreview, Scene, SpindleLane, Stroke, StrokeKind, TimelineStep, View, angle_color, angle_legend,
    find_system_font, height_color, load_scene, project, projected_bounds, timeline_at, timeline_text, tool_position, visible,
};
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
/// Rotary axis, distinct from the red X axis, the green Y axis, and the cyan cut colour.
const ROTARY_AXIS: egui::Color32 = egui::Color32::from_rgb(220, 70, 200);
/// Material still to come off the blank, warm and translucent so the section shows through.
fn removed_fill() -> egui::Color32 {
    egui::Color32::from_rgba_unmultiplied(214, 132, 64, 110)
}
/// Stock outline.
const STOCK_OUTLINE: egui::Color32 = egui::Color32::from_rgb(150, 150, 150);
/// Finished section, steel blue.
const PART_FILL: egui::Color32 = egui::Color32::from_rgb(70, 130, 180);
/// Outline of the finished section, lighter than the fill.
const PART_OUTLINE: egui::Color32 = egui::Color32::from_rgb(186, 214, 232);
/// Spindle centre line, drawn on top of the section.
const LATHE_AXIS: egui::Color32 = egui::Color32::from_rgb(236, 236, 236);
/// A spindle that is turning on the timeline strip.
const LANE_RUNNING: egui::Color32 = egui::Color32::from_rgb(46, 160, 90);
/// A spindle that is stopped on the timeline strip.
const LANE_STOPPED: egui::Color32 = egui::Color32::from_rgb(110, 114, 122);
/// Outline of a lane whose chuck is open.
const LANE_OPEN: egui::Color32 = egui::Color32::from_rgb(210, 64, 64);
/// Band drawn across both lanes while the spindles are synchronised.
fn sync_band() -> egui::Color32 {
    egui::Color32::from_rgba_unmultiplied(90, 170, 255, 90)
}
/// Work-coordinate mark for the sub housing on the half-section.
const SUB_TICK: egui::Color32 = egui::Color32::from_rgb(255, 196, 60);

fn main() -> ExitCode {
    match try_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(AppError::Usage(message)) => {
            eprintln!("campanion-view: {message}");
            eprintln!("usage: campanion-view <ir.json> [--arc-tol MM] [--iso] [--angle]");
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
        Command::Preview { path, arc_tol, iso, angle } => {
            let json = read_ir(&path)?;
            let scene = load_scene(&json, arc_tol).map_err(|err| AppError::run(format!("{}: {err}", path.display())))?;
            open_preview(preview_title(&path), scene, iso, angle)
        }
    }
}

enum Command {
    Help,
    Version,
    Preview { path: PathBuf, arc_tol: f64, iso: bool, angle: bool },
}

#[derive(Debug)]
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
  campanion-view <ir.json> [--arc-tol MM] [--iso] [--angle]
  campanion-view --help
  campanion-view --version

Open a plan-view preview of a CAMpanion IR file.
Distances are millimetres. Scroll zooms at the cursor, drag pans, and F or a
double-click fits the path in the window. Space toggles playback.
A rotary program draws its A or B axis and shows index angles or the angle range.
A turning program is an XZ half-section (radius up, Z right). --iso is ignored then.

--arc-tol MM
        Maximum chord error when drawing arcs, in millimetres (default 0.01).
--iso
        Open the preview in the isometric view.
--angle
        Colour cutting moves by rotary angle instead of height.
        Ignored when the program has no rotary axis. A toggles this in the window.
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
    let mut iso = false;
    let mut angle = false;
    let mut index = 0;
    while let Some(arg) = args.get(index) {
        if arg.as_os_str() == "--iso" {
            iso = true;
            index += 1;
            continue;
        }
        if arg.as_os_str() == "--angle" {
            angle = true;
            index += 1;
            continue;
        }
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
    Ok(Command::Preview { path, arc_tol, iso, angle })
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

fn open_preview(title: String, scene: Scene, iso: bool, angle: bool) -> Result<(), AppError> {
    let native_options =
        eframe::NativeOptions { viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 800.0]).with_title(title), ..Default::default() };
    eframe::run_native(
        "campanion-view",
        native_options,
        Box::new(move |cc| {
            cc.egui_ctx.set_theme(egui::ThemePreference::Dark);
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let has_font = install_font(&cc.egui_ctx);
            let projection = if iso { Projection::Iso } else { Projection::Top };
            Ok(Box::new(ViewerApp::new(scene, has_font, projection, angle)))
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
    projection: Projection,
    height_colours: bool,
    /// Colour cuts by rotary angle. Ignored when the scene has no rotary setup.
    angle_colours: bool,
    has_font: bool,
    toolpaths: usize,
    cut_length: f64,
    rapid_length: f64,
}

impl ViewerApp {
    fn new(scene: Scene, has_font: bool, projection: Projection, angle_colours: bool) -> Self {
        let (cut_length, rapid_length) = path_lengths(&scene);
        let toolpaths = scene.names.len();
        let playback = Playback::new(scene.total_length);
        // A lathe scene is the flat XZ half-section. Height colour and isometric are mill views.
        let lathe = scene.lathe.is_some();
        let projection = if lathe { Projection::Top } else { projection };
        let height_colours = !lathe && scene.z_range.is_some_and(|(low, high)| high - low > 1.0);
        // A program with no rotary axis ignores the flag, so the window stays on height colours.
        // A turning program has no rotary legend either.
        let angle_colours = angle_colours && scene.rotary.is_some() && !lathe;
        Self {
            scene,
            playback,
            view: View { center: [0.0, 0.0], scale: 1.0 },
            fitted: false,
            show_rapids: true,
            show_order: true,
            projection,
            height_colours,
            angle_colours,
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
                if self.scene.lathe.is_some() {
                    ui.label("XZ half-section (radius up, Z right)");
                } else {
                    ui.label("View");
                    let top = ui.selectable_value(&mut self.projection, Projection::Top, "Top");
                    let iso = ui.selectable_value(&mut self.projection, Projection::Iso, "Iso");
                    if top.clicked() || iso.clicked() {
                        self.fitted = false;
                    }
                }
                ui.checkbox(&mut self.show_rapids, "Rapids");
                ui.checkbox(&mut self.show_order, "Order");
                if self.scene.lathe.is_none() {
                    ui.checkbox(&mut self.height_colours, "Height colours");
                }
                if self.scene.rotary.is_some() && self.scene.lathe.is_none() {
                    ui.checkbox(&mut self.angle_colours, "Angle colours");
                }
                ui.label(self.status_text());
            });
            if !self.scene.timeline.is_empty() {
                draw_timeline(ui, &self.scene, self.playback.position, self.has_font);
            }
        });
    }

    fn status_text(&self) -> String {
        let paths = if self.toolpaths == 1 { "1 toolpath".to_string() } else { format!("{} toolpaths", self.toolpaths) };
        let mut text = format!("{paths}   cut {:.1} mm   rapid {:.1} mm", self.cut_length, self.rapid_length);
        if let Some(overlay) = self.scene.rotary.as_ref().and_then(RotaryPreview::overlay) {
            text.push_str("   ");
            text.push_str(&overlay);
        }
        text
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
        let bounds = projected_bounds(&self.scene, self.projection).unwrap_or(([0.0, 0.0], [0.0, 0.0]));
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
        let projection = self.projection;
        draw_grid(painter, &self.view, rect, projection);
        draw_axes(painter, &self.view, rect, projection);
        if let Some(lathe) = &self.scene.lathe {
            draw_lathe(painter, &self.view, rect, lathe, self.has_font);
            draw_sub_tick(painter, &self.view, rect, &self.scene, self.playback.position, self.has_font);
        } else if let Some(rotary) = &self.scene.rotary {
            draw_rotary_axis(painter, &self.view, rect, rotary, projection, self.has_font);
        }
        let angle_range = self.scene.rotary.as_ref().and_then(RotaryPreview::angle_range);
        let colour_by_angle = self.angle_colours && angle_range.is_some();
        let shown = visible(&self.scene, self.playback.position);
        for stroke in &shown {
            draw_stroke(
                painter,
                &self.view,
                rect,
                stroke,
                self.show_rapids,
                self.height_colours,
                self.scene.z_range,
                colour_by_angle,
                angle_range,
                projection,
            );
        }
        draw_pierces(painter, &self.scene, shown.len(), &self.view, rect, projection);
        draw_tool(painter, &self.scene, &shown, self.playback.position, &self.view, rect, projection);
        if self.has_font && self.show_order {
            draw_labels(painter, &self.scene, &self.view, rect, projection);
        }
        if self.scene.lathe.is_none()
            && colour_by_angle
            && let Some(range) = angle_range
        {
            draw_angle_legend(painter, rect, range, self.has_font);
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let input = read_input(ui);
        if input.space {
            self.toggle_play();
        }
        if input.angle && self.scene.rotary.is_some() {
            self.angle_colours = !self.angle_colours;
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
    angle: bool,
}

fn read_input(ui: &egui::Ui) -> FrameInput {
    ui.input_mut(|input| FrameInput {
        scroll_y: input.smooth_scroll_delta.y,
        zoom_delta: input.zoom_delta(),
        dt: input.stable_dt,
        space: input.consume_key(egui::Modifiers::NONE, egui::Key::Space),
        fit: input.key_pressed(egui::Key::F),
        angle: input.consume_key(egui::Modifiers::NONE, egui::Key::A),
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

fn draw_grid(painter: &egui::Painter, view: &View, rect: egui::Rect, projection: Projection) {
    if !view.scale.is_finite() || view.scale * GRID_MM < GRID_MIN_PX {
        return;
    }
    let Some((min, max)) = grid_world(view, rect, projection) else {
        return;
    };
    let stroke = egui::Stroke::new(1.0, GRID_COLOR);
    draw_grid_axis(painter, view, rect, min, max, true, stroke, projection);
    draw_grid_axis(painter, view, rect, min, max, false, stroke, projection);
}

fn draw_grid_axis(
    painter: &egui::Painter,
    view: &View,
    rect: egui::Rect,
    min: [f64; 2],
    max: [f64; 2],
    vertical: bool,
    stroke: egui::Stroke,
    projection: Projection,
) {
    let (lo, hi) = if vertical { (min[0], max[0]) } else { (min[1], max[1]) };
    let Some(mut cursor) = grid_start(lo) else {
        return;
    };
    let mut count = 0;
    while cursor <= hi + LENGTH_EPS_MM && count < MAX_GRID_LINES {
        let (start, end) = if vertical { ([cursor, min[1]], [cursor, max[1]]) } else { ([min[0], cursor], [max[0], cursor]) };
        draw_floor_segment(painter, view, rect, start, end, projection, stroke);
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

fn draw_axes(painter: &egui::Painter, view: &View, rect: egui::Rect, projection: Projection) {
    let Some((min, max)) = grid_world(view, rect, projection) else {
        return;
    };
    draw_floor_segment(painter, view, rect, [min[0], 0.0], [max[0], 0.0], projection, egui::Stroke::new(1.0, AXIS_X));
    draw_floor_segment(painter, view, rect, [0.0, min[1]], [0.0, max[1]], projection, egui::Stroke::new(1.0, AXIS_Y));
}

fn draw_stroke(
    painter: &egui::Painter,
    view: &View,
    rect: egui::Rect,
    stroke: &Stroke,
    show_rapids: bool,
    height_colours: bool,
    z_range: Option<(f64, f64)>,
    colour_by_angle: bool,
    angle_range: Option<(f64, f64)>,
    projection: Projection,
) {
    match stroke.kind {
        StrokeKind::Rapid => {
            if !show_rapids {
                return;
            }
            let points = screen_points(view, rect, stroke, projection);
            if points.len() >= 2 {
                painter.extend(egui::Shape::dashed_line(&points, egui::Stroke::new(1.0, RAPID_COLOR), 6.0, 4.0));
            }
        }
        StrokeKind::Cut => {
            if colour_by_angle && let Some(range) = angle_range {
                if stroke.angles.len() >= 2 && stroke.angles.len() == stroke.points.len() {
                    draw_cut_angles(painter, view, rect, stroke, range, projection);
                } else {
                    draw_flat_cut(painter, view, rect, stroke, projection);
                }
                return;
            }
            if height_colours && let Some(range) = z_range {
                draw_cut_heights(painter, view, rect, stroke, range, projection);
                return;
            }
            draw_flat_cut(painter, view, rect, stroke, projection);
        }
        StrokeKind::Plunge => draw_plunge(painter, view, rect, stroke, projection),
    }
}

fn draw_flat_cut(painter: &egui::Painter, view: &View, rect: egui::Rect, stroke: &Stroke, projection: Projection) {
    let points = screen_points(view, rect, stroke, projection);
    if points.len() >= 2 {
        let _ = painter.add(egui::Shape::line(points, egui::Stroke::new(1.5, CUT_COLOR)));
    }
}

/// Removed material, stock outline, finished section, then the spindle centre line.
///
/// Toolpaths are drawn by the caller after this, so the cuts sit on top of the blank.
fn draw_lathe(painter: &egui::Painter, view: &View, rect: egui::Rect, lathe: &LathePreview, has_font: bool) {
    for polygon in &lathe.removed {
        fill_lathe_polygon(painter, view, rect, polygon, removed_fill());
    }
    stroke_lathe_polygon(painter, view, rect, &lathe.stock, egui::Stroke::new(1.25, STOCK_OUTLINE));
    for polygon in lathe.part_fill() {
        fill_lathe_polygon(painter, view, rect, &polygon, PART_FILL);
    }
    stroke_lathe_polygon(painter, view, rect, &lathe.part, egui::Stroke::new(1.5, PART_OUTLINE));
    let axis = lathe_screen(view, rect, &[lathe.axis.0, lathe.axis.1]);
    if axis.len() == 2 {
        painter.extend(egui::Shape::dashed_line(&axis, egui::Stroke::new(1.0, LATHE_AXIS), 8.0, 5.0));
    }
    if !has_font {
        return;
    }
    let _ = painter.text(
        rect.left_top() + egui::vec2(12.0, 10.0),
        egui::Align2::LEFT_TOP,
        "XZ half-section (radius up, Z right)",
        egui::FontId::proportional(14.0),
        LABEL_COLOR,
    );
}

/// Two lanes under the playback slider. X is playback position over [`Scene::total_length`].
///
/// Lane 1 is `main` and lane 2 is `sub`. A running spindle is green and a stopped spindle is grey.
/// An open chuck is a red outline. Sync tints both lanes. The cursor sits on the playhead, and the
/// line under the lanes is [`timeline_text`] for the step in force.
fn draw_timeline(ui: &mut egui::Ui, scene: &Scene, position: f64, has_font: bool) {
    const STRIP_H: f32 = 64.0;
    const NAME_W: f32 = 48.0;
    const LABEL_H: f32 = 18.0;
    let width = ui.available_width().max(1.0);
    let (response, painter) = ui.allocate_painter(egui::vec2(width, STRIP_H), egui::Sense::hover());
    let rect = response.rect;
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(18, 20, 24));

    let plot = egui::Rect::from_min_max(egui::pos2(rect.min.x + NAME_W, rect.min.y + 4.0), egui::pos2(rect.max.x - 8.0, rect.max.y - LABEL_H));
    if plot.width() < 1.0 || plot.height() < 2.0 {
        return;
    }
    let lane_h = plot.height() * 0.5;
    let main_rect = egui::Rect::from_min_size(plot.min, egui::vec2(plot.width(), lane_h));
    let sub_rect = egui::Rect::from_min_size(egui::pos2(plot.min.x, plot.min.y + lane_h), egui::vec2(plot.width(), lane_h));
    let total = finite_total(scene.total_length);
    let span = if total > LENGTH_EPS_MM { total } else { 1.0 };
    draw_timeline_segments(&painter, plot, main_rect, sub_rect, &scene.timeline, span);

    let head = if total > LENGTH_EPS_MM { finite_nonneg_pos(position).min(total) } else { 0.0 };
    let cursor_x = timeline_x(plot, head, span);
    painter.line_segment([egui::pos2(cursor_x, plot.min.y), egui::pos2(cursor_x, plot.max.y)], egui::Stroke::new(1.5, TOOL_COLOR));

    if !has_font {
        return;
    }
    let _ = painter.text(
        egui::pos2(rect.min.x + 6.0, main_rect.center().y),
        egui::Align2::LEFT_CENTER,
        "main",
        egui::FontId::proportional(12.0),
        LABEL_COLOR,
    );
    let _ = painter.text(
        egui::pos2(rect.min.x + 6.0, sub_rect.center().y),
        egui::Align2::LEFT_CENTER,
        "sub",
        egui::FontId::proportional(12.0),
        LABEL_COLOR,
    );
    if let Some(step) = timeline_at(scene, position) {
        let _ = painter.text(
            egui::pos2(rect.min.x + 8.0, rect.max.y - 2.0),
            egui::Align2::LEFT_BOTTOM,
            timeline_text(step),
            egui::FontId::proportional(12.0),
            LABEL_COLOR,
        );
    }
}

fn draw_timeline_segments(painter: &egui::Painter, plot: egui::Rect, main_rect: egui::Rect, sub_rect: egui::Rect, steps: &[TimelineStep], span: f64) {
    for (index, step) in steps.iter().enumerate() {
        let start = step.position.max(0.0);
        let end = match steps.get(index + 1) {
            Some(next) => next.position.max(0.0),
            None => span,
        };
        if end <= start + LENGTH_EPS_MM {
            continue;
        }
        let x0 = timeline_x(plot, start, span);
        let x1 = timeline_x(plot, end, span);
        if x1 <= x0 {
            continue;
        }
        paint_lane(painter, main_rect, x0, x1, step.main);
        paint_lane(painter, sub_rect, x0, x1, step.sub);
        if step.sync {
            let band = egui::Rect::from_min_max(egui::pos2(x0, main_rect.min.y), egui::pos2(x1, sub_rect.max.y));
            painter.rect_filled(band, 0.0, sync_band());
        }
        if !step.main.clamped {
            stroke_rect(painter, lane_segment(main_rect, x0, x1), egui::Stroke::new(1.5, LANE_OPEN));
        }
        if !step.sub.clamped {
            stroke_rect(painter, lane_segment(sub_rect, x0, x1), egui::Stroke::new(1.5, LANE_OPEN));
        }
    }
}

fn paint_lane(painter: &egui::Painter, lane_rect: egui::Rect, x0: f32, x1: f32, lane: SpindleLane) {
    let fill = if lane.rpm.is_finite() && lane.rpm > 0.0 { LANE_RUNNING } else { LANE_STOPPED };
    painter.rect_filled(lane_segment(lane_rect, x0, x1), 0.0, fill);
}

fn lane_segment(lane_rect: egui::Rect, x0: f32, x1: f32) -> egui::Rect {
    let inset = 1.0;
    egui::Rect::from_min_max(egui::pos2(x0, lane_rect.min.y + inset), egui::pos2(x1, lane_rect.max.y - inset))
}

fn stroke_rect(painter: &egui::Painter, rect: egui::Rect, stroke: egui::Stroke) {
    let top_left = rect.min;
    let bottom_right = rect.max;
    let top_right = egui::pos2(bottom_right.x, top_left.y);
    let bottom_left = egui::pos2(top_left.x, bottom_right.y);
    painter.line_segment([top_left, top_right], stroke);
    painter.line_segment([top_right, bottom_right], stroke);
    painter.line_segment([bottom_right, bottom_left], stroke);
    painter.line_segment([bottom_left, top_left], stroke);
}

fn timeline_x(plot: egui::Rect, position: f64, span: f64) -> f32 {
    let span = if span.is_finite() && span > LENGTH_EPS_MM { span } else { 1.0 };
    let t = if position.is_finite() { (position / span).clamp(0.0, 1.0) } else { 0.0 };
    plot.min.x + plot.width() * t as f32
}

fn finite_nonneg_pos(position: f64) -> f64 {
    if position.is_finite() && position > 0.0 { position } else { 0.0 }
}

/// Short vertical mark at a work-coordinate sub-housing Z. Machine moves are not on this section.
fn draw_sub_tick(painter: &egui::Painter, view: &View, rect: egui::Rect, scene: &Scene, position: f64, has_font: bool) {
    let Some(step) = timeline_at(scene, position) else {
        return;
    };
    let Some(z) = step.sub_z else {
        return;
    };
    if step.sub_z_machine || !z.is_finite() {
        return;
    }
    let Some(origin) = to_screen_pos(view, rect, [z, 0.0], 0.0, Projection::Top) else {
        return;
    };
    let top = origin + egui::vec2(0.0, -16.0);
    painter.line_segment([origin, top], egui::Stroke::new(2.0, SUB_TICK));
    if !has_font {
        return;
    }
    let _ = painter.text(top + egui::vec2(4.0, 0.0), egui::Align2::LEFT_BOTTOM, "sub", egui::FontId::proportional(12.0), SUB_TICK);
}

fn fill_lathe_polygon(painter: &egui::Painter, view: &View, rect: egui::Rect, polygon: &[[f64; 2]], fill: egui::Color32) {
    let points = lathe_screen(view, rect, polygon);
    if points.len() >= 3 && points.len() == polygon.len() {
        let _ = painter.add(egui::Shape::convex_polygon(points, fill, egui::Stroke::new(0.0, egui::Color32::TRANSPARENT)));
    }
}

fn stroke_lathe_polygon(painter: &egui::Painter, view: &View, rect: egui::Rect, polygon: &[[f64; 2]], stroke: egui::Stroke) {
    if polygon.len() < 2 {
        return;
    }
    let mut points = lathe_screen(view, rect, polygon);
    if points.len() != polygon.len() || points.len() < 2 {
        return;
    }
    if let Some(first) = points.first().copied() {
        points.push(first);
    }
    let _ = painter.add(egui::Shape::line(points, stroke));
}

fn lathe_screen(view: &View, rect: egui::Rect, polygon: &[[f64; 2]]) -> Vec<egui::Pos2> {
    let mut screen = Vec::with_capacity(polygon.len());
    for point in polygon {
        let Some(pos) = to_screen_pos(view, rect, *point, 0.0, Projection::Top) else {
            continue;
        };
        screen.push(pos);
    }
    screen
}

/// The rotary axis through the setup centre, across the toolpath, labelled A or B.
fn draw_rotary_axis(painter: &egui::Painter, view: &View, rect: egui::Rect, rotary: &RotaryPreview, projection: Projection, has_font: bool) {
    let Some((start, end)) = rotary.line else {
        return;
    };
    draw_segment(painter, view, rect, [start[0], start[1]], start[2], [end[0], end[1]], end[2], projection, egui::Stroke::new(2.0, ROTARY_AXIS));
    if !has_font {
        return;
    }
    let Some(pos) = to_screen_pos(view, rect, [end[0], end[1]], end[2], projection) else {
        return;
    };
    let _ = painter.text(pos + egui::vec2(6.0, -4.0), egui::Align2::LEFT_BOTTOM, rotary.label, egui::FontId::proportional(14.0), ROTARY_AXIS);
}

/// Colour each cut segment by the mean rotary angle of its endpoints.
fn draw_cut_angles(painter: &egui::Painter, view: &View, rect: egui::Rect, stroke: &Stroke, range: (f64, f64), projection: Projection) {
    let segments = stroke.points.len().saturating_sub(1);
    for index in 0..segments {
        let (Some(start), Some(end)) = (stroke.points.get(index), stroke.points.get(index + 1)) else {
            break;
        };
        let (Some(a0), Some(a1)) = (stroke.angles.get(index).copied(), stroke.angles.get(index + 1).copied()) else {
            break;
        };
        let rgb = angle_color((a0 + a1) * 0.5, range);
        let color = egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
        draw_segment(
            painter,
            view,
            rect,
            *start,
            point_z(stroke, index),
            *end,
            point_z(stroke, index + 1),
            projection,
            egui::Stroke::new(1.5, color),
        );
    }
}

/// Swatches from the low angle (blue) to the high angle (red), with the end labels.
fn draw_angle_legend(painter: &egui::Painter, rect: egui::Rect, range: (f64, f64), has_font: bool) {
    let Some(legend) = angle_legend(range, 8) else {
        return;
    };
    let swatch = 12.0_f32;
    let gap = 2.0_f32;
    let count = legend.colours.len() as f32;
    if count < 1.0 {
        return;
    }
    let bar_w = count * swatch + (count - 1.0) * gap;
    let text_h = if has_font { 16.0 } else { 0.0 };
    let pad = 6.0;
    let width = bar_w + pad * 2.0;
    let height = swatch + text_h + pad * 2.0;
    let origin = egui::pos2(rect.left() + 12.0, rect.bottom() - 12.0 - height);
    let plate = egui::Rect::from_min_size(origin, egui::vec2(width, height));
    painter.rect_filled(plate, 4.0, egui::Color32::from_rgb(16, 18, 22));
    let bar_y = origin.y + pad + text_h;
    for (index, rgb) in legend.colours.iter().enumerate() {
        let x = origin.x + pad + index as f32 * (swatch + gap);
        let swatch_rect = egui::Rect::from_min_size(egui::pos2(x, bar_y), egui::vec2(swatch, swatch));
        painter.rect_filled(swatch_rect, 1.0, egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]));
    }
    if !has_font {
        return;
    }
    let low_pos = egui::pos2(origin.x + pad, bar_y - 2.0);
    let _ = painter.text(low_pos, egui::Align2::LEFT_BOTTOM, legend.low_label.as_str(), egui::FontId::proportional(12.0), LABEL_COLOR);
    let high_pos = egui::pos2(origin.x + pad + bar_w, bar_y - 2.0);
    let _ = painter.text(high_pos, egui::Align2::RIGHT_BOTTOM, legend.high_label.as_str(), egui::FontId::proportional(12.0), LABEL_COLOR);
}

/// Colour each cut segment by the mean Z of its endpoints.
fn draw_cut_heights(painter: &egui::Painter, view: &View, rect: egui::Rect, stroke: &Stroke, range: (f64, f64), projection: Projection) {
    let segments = stroke.points.len().saturating_sub(1);
    for index in 0..segments {
        let (Some(start), Some(end)) = (stroke.points.get(index), stroke.points.get(index + 1)) else {
            break;
        };
        let z0 = point_z(stroke, index);
        let z1 = point_z(stroke, index + 1);
        let rgb = height_color((z0 + z1) * 0.5, range);
        let color = egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
        draw_segment(painter, view, rect, *start, z0, *end, z1, projection, egui::Stroke::new(1.5, color));
    }
}

/// A vertical feed is a dot in plan view and a line once Z is projected.
fn draw_plunge(painter: &egui::Painter, view: &View, rect: egui::Rect, stroke: &Stroke, projection: Projection) {
    let segments = stroke.points.len().saturating_sub(1);
    for index in 0..segments {
        let (Some(start), Some(end)) = (stroke.points.get(index), stroke.points.get(index + 1)) else {
            break;
        };
        draw_segment(
            painter,
            view,
            rect,
            *start,
            point_z(stroke, index),
            *end,
            point_z(stroke, index + 1),
            projection,
            egui::Stroke::new(1.5, ORANGE),
        );
    }
    if let Some(point) = stroke.points.first()
        && let Some(pos) = to_screen_pos(view, rect, *point, point_z(stroke, 0), projection)
    {
        painter.circle_filled(pos, 2.0, ORANGE);
    }
}

/// A pierce belongs to the stroke that follows it. Draw it once that stroke is in view.
/// An index past the last stroke waits until every stroke is shown.
fn draw_pierces(painter: &egui::Painter, scene: &Scene, shown: usize, view: &View, rect: egui::Rect, projection: Projection) {
    for marker in &scene.pierces {
        if !pierce_visible(marker.stroke_index, scene.strokes.len(), shown) {
            continue;
        }
        if let Some(pos) = to_screen_pos(view, rect, marker.at, pierce_z(scene, marker.stroke_index), projection) {
            painter.circle_filled(pos, 3.5, ORANGE);
        }
    }
}

fn pierce_visible(stroke_index: usize, stroke_count: usize, shown: usize) -> bool {
    if stroke_index < stroke_count { stroke_index < shown } else { shown == stroke_count }
}

fn draw_tool(painter: &egui::Painter, scene: &Scene, shown: &[Stroke], position: f64, view: &View, rect: egui::Rect, projection: Projection) {
    let Some((at, z)) = tool_world(scene, shown, position) else {
        return;
    };
    let Some(pos) = to_screen_pos(view, rect, at, z, projection) else {
        return;
    };
    painter.circle_stroke(pos, 6.0, egui::Stroke::new(1.5, TOOL_COLOR));
}

/// Tool XY from the visible stroke, with Z from that stroke's last stored height.
///
/// At the start of the program nothing is visible yet, so the marker sits on the first point.
fn tool_world(scene: &Scene, shown: &[Stroke], position: f64) -> Option<([f64; 2], f64)> {
    if let Some(stroke) = shown.last()
        && let Some(point) = stroke.points.last().copied()
    {
        return Some((point, point_z(stroke, stroke.points.len().saturating_sub(1))));
    }
    let point = tool_position(scene, position)?;
    let z = match scene.strokes.first() {
        Some(stroke) => point_z(stroke, 0),
        None => 0.0,
    };
    Some((point, z))
}

fn draw_labels(painter: &egui::Painter, scene: &Scene, view: &View, rect: egui::Rect, projection: Projection) {
    for label in &scene.labels {
        let Some(pos) = to_screen_pos(view, rect, label.at, label_z(scene, label.at), projection) else {
            continue;
        };
        let _ =
            painter.text(pos + egui::vec2(6.0, -4.0), egui::Align2::LEFT_BOTTOM, label.text.as_str(), egui::FontId::proportional(14.0), LABEL_COLOR);
    }
}

fn draw_floor_segment(
    painter: &egui::Painter,
    view: &View,
    rect: egui::Rect,
    start: [f64; 2],
    end: [f64; 2],
    projection: Projection,
    stroke: egui::Stroke,
) {
    draw_segment(painter, view, rect, start, 0.0, end, 0.0, projection, stroke);
}

fn draw_segment(
    painter: &egui::Painter,
    view: &View,
    rect: egui::Rect,
    start: [f64; 2],
    z0: f64,
    end: [f64; 2],
    z1: f64,
    projection: Projection,
    stroke: egui::Stroke,
) {
    let (Some(a), Some(b)) = (to_screen_pos(view, rect, start, z0, projection), to_screen_pos(view, rect, end, z1, projection)) else {
        return;
    };
    painter.line_segment([a, b], stroke);
}

fn screen_points(view: &View, rect: egui::Rect, stroke: &Stroke, projection: Projection) -> Vec<egui::Pos2> {
    let mut screen = Vec::with_capacity(stroke.points.len());
    for (index, point) in stroke.points.iter().enumerate() {
        if let Some(pos) = to_screen_pos(view, rect, *point, point_z(stroke, index), projection) {
            screen.push(pos);
        }
    }
    screen
}

fn point_z(stroke: &Stroke, index: usize) -> f64 {
    if let Some(z) = stroke.zs.get(index).copied()
        && z.is_finite()
    {
        return z;
    }
    if index == 0 && stroke.z_start.is_finite() {
        return stroke.z_start;
    }
    if stroke.z_end.is_finite() {
        stroke.z_end
    } else if stroke.z_start.is_finite() {
        stroke.z_start
    } else {
        0.0
    }
}

fn pierce_z(scene: &Scene, stroke_index: usize) -> f64 {
    if let Some(stroke) = scene.strokes.get(stroke_index) {
        return point_z(stroke, 0);
    }
    if let Some(stroke) = scene.strokes.last() {
        return point_z(stroke, stroke.points.len().saturating_sub(1));
    }
    0.0
}

fn label_z(scene: &Scene, at: [f64; 2]) -> f64 {
    for stroke in &scene.strokes {
        if stroke.kind == StrokeKind::Rapid {
            continue;
        }
        for (index, point) in stroke.points.iter().enumerate() {
            if (point[0] - at[0]).abs() <= 1e-6 && (point[1] - at[1]).abs() <= 1e-6 {
                return point_z(stroke, index);
            }
        }
    }
    0.0
}

fn to_screen_pos(view: &View, rect: egui::Rect, world: [f64; 2], z: f64, projection: Projection) -> Option<egui::Pos2> {
    let width = f64::from(rect.width());
    let height = f64::from(rect.height());
    let screen = view.to_screen(project(world, z, projection), width, height);
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

/// World XY covered by the viewport on the Z = 0 plane. Top view uses the viewport directly.
fn grid_world(view: &View, rect: egui::Rect, projection: Projection) -> Option<([f64; 2], [f64; 2])> {
    let (min, max) = view_span(view, rect)?;
    match projection {
        Projection::Top => Some((min, max)),
        Projection::Iso => iso_floor_bounds(min, max),
    }
}

/// Inverse of the isometric map at Z = 0, so the grid stays on the machine floor.
fn unproject_iso_floor(projected: [f64; 2]) -> Option<[f64; 2]> {
    let cos30 = std::f64::consts::FRAC_PI_6.cos();
    let sin30 = std::f64::consts::FRAC_PI_6.sin();
    if !cos30.is_finite() || !sin30.is_finite() || cos30.abs() <= f64::EPSILON || sin30.abs() <= f64::EPSILON {
        return None;
    }
    let x_minus_y = projected[0] / cos30;
    let x_plus_y = projected[1] / sin30;
    let world = [(x_minus_y + x_plus_y) * 0.5, (x_plus_y - x_minus_y) * 0.5];
    if world[0].is_finite() && world[1].is_finite() { Some(world) } else { None }
}

fn iso_floor_bounds(min: [f64; 2], max: [f64; 2]) -> Option<([f64; 2], [f64; 2])> {
    let corners = [[min[0], min[1]], [max[0], min[1]], [min[0], max[1]], [max[0], max[1]]];
    let mut low = [f64::INFINITY, f64::INFINITY];
    let mut high = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    for corner in corners {
        let world = unproject_iso_floor(corner)?;
        low[0] = low[0].min(world[0]);
        low[1] = low[1].min(world[1]);
        high[0] = high[0].max(world[0]);
        high[1] = high[1].max(world[1]);
    }
    Some((low, high))
}

fn view_span(view: &View, rect: egui::Rect) -> Option<([f64; 2], [f64; 2])> {
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use std::ffi::OsString;

    use super::{Command, help_text, parse_args};

    #[test]
    fn help_documents_iso_and_keeps_arc_tol() {
        let help = help_text();
        assert!(help.contains("campanion-view <ir.json> [--arc-tol MM] [--iso] [--angle]"), "{help}");
        assert!(help.contains("--arc-tol MM"), "{help}");
        assert!(help.contains("Maximum chord error when drawing arcs"), "{help}");
        assert!(help.contains("--iso"), "{help}");
        assert!(help.contains("isometric view"), "{help}");
        assert!(help.contains("--angle"), "{help}");
        assert!(help.contains("rotary angle instead of height"), "{help}");
        assert!(help.contains("A toggles this in the window"), "{help}");
        assert!(help.contains("Scroll zooms at the cursor"), "{help}");
        assert!(help.contains("XZ half-section (radius up, Z right)"), "{help}");
        assert!(help.contains("--iso is ignored then"), "{help}");
    }

    #[test]
    fn angle_flag_follows_the_iso_flag() {
        let with_angle = parse_args(&[OsString::from("part.json"), OsString::from("--angle")]).expect("parse");
        let Command::Preview { angle, iso, path, .. } = with_angle else {
            panic!("expected a preview command");
        };
        assert!(angle);
        assert!(!iso);
        assert_eq!(path, std::path::PathBuf::from("part.json"));

        let plain = parse_args(&[OsString::from("--iso"), OsString::from("part.json")]).expect("parse");
        let Command::Preview { angle, iso, .. } = plain else {
            panic!("expected a preview command");
        };
        assert!(!angle);
        assert!(iso);
    }

    fn empty_scene(names: Vec<String>, rotary: Option<campanion_view::RotaryPreview>) -> campanion_view::Scene {
        campanion_view::Scene {
            strokes: Vec::new(),
            pierces: Vec::new(),
            labels: Vec::new(),
            bounds: None,
            z_range: None,
            total_length: 0.0,
            names,
            rotary,
            lathe: None,
            timeline: Vec::new(),
        }
    }

    #[test]
    fn indexed_overlay_is_listed_and_a_plain_program_ignores_angle_colours() {
        use campanion_view::{RotaryPreview, RotarySpan};

        let plain = super::ViewerApp::new(empty_scene(vec!["only".into()], None), false, super::Projection::Top, true);
        assert!(!plain.angle_colours, "a program with no rotary axis ignores --angle");
        assert_eq!(plain.status_text(), "1 toolpath   cut 0.0 mm   rapid 0.0 mm");

        let indexed = empty_scene(
            vec!["face0".into(), "face90".into()],
            Some(RotaryPreview {
                label: "A",
                center: [0.0, 0.0, 0.0],
                line: Some(([0.0, 0.0, 0.0], [20.0, 0.0, 0.0])),
                span: Some(RotarySpan::Indexed(vec![0.0, 90.0])),
            }),
        );
        let indexed = super::ViewerApp::new(indexed, false, super::Projection::Top, true);
        assert!(indexed.angle_colours);
        assert_eq!(indexed.status_text(), "2 toolpaths   cut 0.0 mm   rapid 0.0 mm   Index: 0°, 90°");

        let continuous = empty_scene(
            vec!["wrap".into()],
            Some(RotaryPreview {
                label: "B",
                center: [1.0, 2.0, 3.0],
                line: None,
                span: Some(RotarySpan::Continuous { min_deg: 0.0, max_deg: 360.0 }),
            }),
        );
        let continuous = super::ViewerApp::new(continuous, false, super::Projection::Top, false);
        assert!(!continuous.angle_colours);
        assert_eq!(continuous.status_text(), "1 toolpath   cut 0.0 mm   rapid 0.0 mm   Angle: 0°..360°");
    }

    #[test]
    fn lathe_preview_forces_the_flat_half_section() {
        use campanion_view::{LathePreview, RotaryPreview, RotarySpan};

        let mut scene = empty_scene(
            vec!["finish".into()],
            Some(RotaryPreview { label: "A", center: [0.0, 0.0, 0.0], line: None, span: Some(RotarySpan::Indexed(vec![0.0, 90.0])) }),
        );
        scene.z_range = Some((0.0, 30.0));
        scene.lathe = Some(LathePreview {
            stock: vec![[-10.0, 0.0], [0.0, 0.0], [0.0, 5.0], [-10.0, 5.0]],
            part: Vec::new(),
            removed: Vec::new(),
            axis: ([-10.0, 0.0], [0.0, 0.0]),
        });
        let app = super::ViewerApp::new(scene, false, super::Projection::Iso, true);
        assert_eq!(app.projection, super::Projection::Top, "a lathe scene has no isometric view");
        assert!(!app.height_colours, "a lathe scene has no height view");
        assert!(!app.angle_colours, "a lathe scene hides the rotary legend");
    }

    #[test]
    fn iso_flag_starts_in_the_isometric_view() {
        let with_iso = parse_args(&[OsString::from("--iso"), OsString::from("part.json")]).expect("parse");
        let Command::Preview { iso, path, .. } = with_iso else {
            panic!("expected a preview command");
        };
        assert!(iso);
        assert_eq!(path, std::path::PathBuf::from("part.json"));

        let plain = parse_args(&[OsString::from("part.json")]).expect("parse");
        let Command::Preview { iso, .. } = plain else {
            panic!("expected a preview command");
        };
        assert!(!iso);
    }

    #[test]
    fn iso_floor_unproject_inverts_the_projection() {
        for point in [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [12.0, -4.0], [-3.0, 8.0]] {
            let projected = campanion_view::project(point, 0.0, super::Projection::Iso);
            let world = super::unproject_iso_floor(projected).expect("inverse");
            let scale = 1.0 + point[0].abs().max(point[1].abs());
            assert!((world[0] - point[0]).abs() <= 1e-9 * scale, "{world:?} != {point:?}");
            assert!((world[1] - point[1]).abs() <= 1e-9 * scale, "{world:?} != {point:?}");
        }
    }
}
