//! The egui window: menus, toolbars, viewports and placeholder panels.

use std::path::{Path, PathBuf};
use std::sync::{Arc, mpsc};
use std::time::Duration;

use campanion_view::{Scene, StrokeKind, find_system_font};
use eframe::egui::{self, Align, Color32, CornerRadius, Layout, RichText, Stroke, Vec2};

use crate::theme;
use crate::viewport::{self, Viewport};
use crate::{
    GuiState, JOG_STEPS_MM, ModeTab, OpenedDocument, ParkedWorkspace, ROTATE_STEPS_DEG, RunPhase, SubProcess, UserField, Workspace, gcode_line_count,
    open_path,
};

const WINDOW_SIZE: [f32; 2] = [1600.0, 900.0];
const MAX_RECENT: usize = 8;

/// Arguments accepted by the window after the binary has parsed its flags.
pub struct Launch {
    /// Optional file to open on startup.
    pub open: Option<PathBuf>,
    /// When set, capture the main views into this directory and exit.
    pub screenshot_dir: Option<PathBuf>,
}

/// Native window entry used by the `campanion-gui` binary.
pub fn run(launch: Launch) -> Result<(), String> {
    let title = "CAMpanion";
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size(WINDOW_SIZE).with_min_inner_size([1100.0, 700.0]).with_title(title),
        ..Default::default()
    };
    eframe::run_native(
        "campanion-gui",
        native_options,
        Box::new(move |cc| {
            let has_font = install_font(&cc.egui_ctx);
            theme::apply(&cc.egui_ctx, true);
            let mut app = App::new(has_font);
            if let Some(path) = launch.open {
                app.open_file(&path);
            } else if launch.screenshot_dir.is_some() {
                app.open_sample_for_screenshot();
            }
            if let Some(dir) = launch.screenshot_dir {
                app.begin_screenshots(dir);
            }
            Ok(Box::new(app))
        }),
    )
    .map_err(|err| err.to_string())
}

fn install_font(ctx: &egui::Context) -> bool {
    let Some(path) = find_system_font() else {
        eprintln!("campanion-gui: no font found; set CAMPANION_VIEW_FONT to a .ttf file");
        return false;
    };
    let bytes = match std::fs::read(&path) {
        Ok(bytes) if !bytes.is_empty() => bytes,
        Ok(_) => {
            eprintln!("campanion-gui: font {} is empty", path.display());
            return false;
        }
        Err(err) => {
            eprintln!("campanion-gui: could not read font {}: {err}", path.display());
            return false;
        }
    };
    let mut fonts = egui::FontDefinitions::empty();
    fonts.font_data.insert("sys".into(), std::sync::Arc::new(egui::FontData::from_owned(bytes)));
    fonts.families.insert(egui::FontFamily::Proportional, vec!["sys".into()]);
    fonts.families.insert(egui::FontFamily::Monospace, vec!["sys".into()]);
    ctx.set_fonts(fonts);
    true
}

struct App {
    state: GuiState,
    scene: Option<Scene>,
    viewport: Option<Viewport>,
    _has_font: bool,
    open_dialog: bool,
    open_path_text: String,
    about: bool,
    preferences: bool,
    machine_profiles: bool,
    material_library: bool,
    screenshot: Option<ScreenshotJob>,
}

struct ScreenshotJob {
    dir: PathBuf,
    plan: Vec<(Workspace, ModeTab, String)>,
    index: usize,
    frames_waited: u32,
    awaiting: bool,
    receiver: mpsc::Receiver<Arc<egui::ColorImage>>,
    sender: mpsc::Sender<Arc<egui::ColorImage>>,
    written: Vec<PathBuf>,
    error: Option<String>,
}

impl App {
    fn new(has_font: bool) -> Self {
        Self {
            state: GuiState::new(),
            scene: None,
            viewport: None,
            _has_font: has_font,
            open_dialog: false,
            open_path_text: String::new(),
            about: false,
            preferences: false,
            machine_profiles: false,
            material_library: false,
            screenshot: None,
        }
    }

    fn open_file(&mut self, path: &Path) {
        match open_path(path) {
            Ok(OpenedDocument::Ir { path, scene, gcode }) => {
                self.state.gcode = gcode.unwrap_or_default();
                self.state.has_scene = true;
                self.state.pending_job = None;
                self.state.run_lines = gcode_line_count(&self.state.gcode);
                if self.state.export_name.is_empty()
                    && let Some(stem) = path.file_stem().and_then(|stem| stem.to_str())
                {
                    self.state.export_name = stem.to_string();
                }
                self.viewport = Some(Viewport::new(&scene));
                self.scene = Some(*scene);
                self.state.remember_open(path.clone());
                self.state.status = format!("Opened {}", path.display());
                if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
                    self.state.set_workspace(Workspace::ThermalJet);
                }
                if self.state.workspace.has_tab(ModeTab::Operations) {
                    let _ = self.state.set_tab(ModeTab::Operations);
                }
            }
            Ok(OpenedDocument::Job { path, format }) => {
                self.scene = None;
                self.viewport = None;
                self.state.has_scene = false;
                self.state.pending_job = Some(path.display().to_string());
                self.state.gcode.clear();
                self.state.remember_open(path.clone());
                self.state.status = format!(
                    "Opened {format} job. Planning is not wired in this preview; open an IR file from `campanion run --ir` to see a toolpath."
                );
            }
            Ok(OpenedDocument::Gcode { path, text }) => {
                self.state.gcode = text;
                self.state.run_lines = gcode_line_count(&self.state.gcode);
                self.state.pending_job = None;
                self.state.remember_open(path.clone());
                self.state.status = format!("Opened G-code {}", path.display());
                if self.state.workspace.has_tab(ModeTab::Export) {
                    let _ = self.state.set_tab(ModeTab::Export);
                }
            }
            Err(err) => {
                self.state.status = err.to_string();
            }
        }
        self.open_dialog = false;
    }

    fn open_sample_for_screenshot(&mut self) {
        for candidate in screenshot_ir_candidates() {
            if candidate.is_file() {
                self.open_file(&candidate);
                return;
            }
        }
    }

    fn begin_screenshots(&mut self, dir: PathBuf) {
        let (sender, receiver) = mpsc::channel();
        self.screenshot = Some(ScreenshotJob {
            dir,
            plan: vec![
                (Workspace::ThermalJet, ModeTab::Operations, "cutting-operations.png".to_string()),
                (Workspace::ThermalJet, ModeTab::Run, "cutting-run.png".to_string()),
                (Workspace::Resin, ModeTab::Orient, "resin-orient.png".to_string()),
                (Workspace::Resin, ModeTab::Supports, "resin-supports.png".to_string()),
                (Workspace::Resin, ModeTab::Slice, "resin-slice.png".to_string()),
                (Workspace::Resin, ModeTab::Export, "resin-export.png".to_string()),
            ],
            index: 0,
            frames_waited: 0,
            awaiting: false,
            receiver,
            sender,
            written: Vec::new(),
            error: None,
        });
        if let Some(job) = &self.screenshot {
            let (workspace, tab, _) = &job.plan[0];
            let workspace = *workspace;
            let tab = *tab;
            self.state.set_workspace(workspace);
            let _ = self.state.set_tab(tab);
        }
    }

    fn apply_document_new(&mut self) {
        self.scene = None;
        self.viewport = None;
        self.state.new_job();
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        let typing = ctx.egui_wants_keyboard_input();
        let mut quit = false;
        let mut new_job = false;
        let mut open = false;
        let mut export = false;
        let mut next = false;
        let mut prev = false;
        let mut arrange = false;
        let mut layer_up = false;
        let mut layer_down = false;
        ctx.input(|input| {
            let cmd = input.modifiers.command;
            if cmd && input.key_pressed(egui::Key::Q) {
                quit = true;
            }
            if cmd && input.key_pressed(egui::Key::N) {
                new_job = true;
            }
            if cmd && input.key_pressed(egui::Key::O) {
                open = true;
            }
            if cmd && input.key_pressed(egui::Key::E) {
                export = true;
            }
            if cmd && input.key_pressed(egui::Key::Tab) {
                if input.modifiers.shift {
                    prev = true;
                } else {
                    next = true;
                }
            }
            if !typing {
                if input.key_pressed(egui::Key::A) {
                    arrange = true;
                }
                if input.key_pressed(egui::Key::ArrowUp) || input.key_pressed(egui::Key::ArrowRight) {
                    layer_up = true;
                }
                if input.key_pressed(egui::Key::ArrowDown) || input.key_pressed(egui::Key::ArrowLeft) {
                    layer_down = true;
                }
            }
        });
        if quit {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if new_job {
            self.apply_document_new();
        }
        if open {
            self.open_dialog = true;
        }
        if export {
            let _ = self.state.set_tab(ModeTab::Export);
        }
        if next {
            self.state.next_tab();
        }
        if prev {
            self.state.prev_tab();
        }
        if arrange && self.state.workspace == Workspace::Resin && self.state.tab == ModeTab::Orient {
            self.state.status = "Arrange is a placeholder in this preview".to_string();
        }
        if self.state.tab == ModeTab::Slice {
            if layer_up {
                let _ = self.state.step_layer(1);
            }
            if layer_down {
                let _ = self.state.step_layer(-1);
            }
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        theme::apply(&ctx, self.state.dark_theme);
        self.handle_keys(&ctx);
        if let Some(scene) = &self.scene
            && let Some(viewport) = &mut self.viewport
        {
            viewport.playback.step(f64::from(ctx.input(|input| input.stable_dt)), scene.total_length);
        }
        if self.state.run == RunPhase::Running {
            let dt = ctx.input(|input| input.stable_dt);
            if dt.is_finite() && dt > 0.0 {
                let total = self.state.run_lines.max(1);
                let advance = ((dt * 12.0).ceil() as u32).max(1);
                self.state.run_line = self.state.run_line.saturating_add(advance).min(total);
                if self.state.run_lines > 0 && self.state.run_line >= self.state.run_lines {
                    self.state.finish_run();
                }
            }
            ctx.request_repaint_after(Duration::from_millis(50));
        }

        self.menu_bar(ui);
        self.toolbar(ui);
        self.status_bar(ui);
        if self.state.tab != ModeTab::Slice && self.state.tab != ModeTab::Export && self.state.show_timeline {
            self.timeline(ui);
        }
        self.body(ui);
        self.dialogs(&ctx);
        self.tick_screenshots(&ctx);
    }
}

impl App {
    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        egui::Panel::top("menu").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(24.0);
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New").clicked() {
                        self.apply_document_new();
                        ui.close();
                    }
                    if ui.button("Open job…").clicked() {
                        self.open_dialog = true;
                        ui.close();
                    }
                    ui.add_enabled(false, egui::Button::new("Save"));
                    ui.add_enabled(false, egui::Button::new("Save As…"));
                    ui.separator();
                    if ui.button("Import model / STEP…").clicked() {
                        self.state.status = "Import is not wired in this preview".to_string();
                        ui.close();
                    }
                    if ui.button("Export…").clicked() {
                        let _ = self.state.set_tab(ModeTab::Export);
                        ui.close();
                    }
                    ui.separator();
                    ui.menu_button("Recent", |ui| {
                        if self.state.recent.is_empty() {
                            ui.add_enabled(false, egui::Button::new("None"));
                        } else {
                            let recent: Vec<PathBuf> = self.state.recent.iter().take(MAX_RECENT).cloned().collect();
                            for path in recent {
                                if ui.button(path.display().to_string()).clicked() {
                                    self.open_file(&path);
                                    ui.close();
                                }
                            }
                        }
                    });
                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
                ui.menu_button("Edit", |ui| {
                    ui.add_enabled(false, egui::Button::new("Undo"));
                    ui.add_enabled(false, egui::Button::new("Redo"));
                    ui.separator();
                    if ui.button("Preferences…").clicked() {
                        self.preferences = true;
                        ui.close();
                    }
                });
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.state.show_tree, "Process tree");
                    ui.checkbox(&mut self.state.show_properties, "Properties");
                    ui.checkbox(&mut self.state.show_timeline, "Timeline");
                    ui.separator();
                    if ui.button("Reset camera").clicked() {
                        if let (Some(scene), Some(viewport)) = (&self.scene, &mut self.viewport) {
                            viewport.fitted = false;
                            let _ = scene;
                        }
                        self.state.status = "Camera reset".to_string();
                        ui.close();
                    }
                    ui.separator();
                    if ui.radio_value(&mut self.state.dark_theme, true, "Dark theme").clicked() {
                        ui.close();
                    }
                    if ui.radio_value(&mut self.state.dark_theme, false, "Light theme").clicked() {
                        ui.close();
                    }
                });
                ui.menu_button("Machine", |ui| {
                    if ui.button("Machine profiles…").clicked() {
                        self.machine_profiles = true;
                        ui.close();
                    }
                    if ui.button("Material library…").clicked() {
                        self.material_library = true;
                        ui.close();
                    }
                });
                ui.menu_button("Help", |ui| {
                    if ui.button("About CAMpanion").clicked() {
                        self.about = true;
                        ui.close();
                    }
                });
            });
        });
    }

    fn toolbar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("toolbar").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(44.0);
            ui.horizontal_centered(|ui| {
                ui.add_space(8.0);
                draw_wordmark(ui);
                ui.add_space(16.0);
                ui.separator();
                ui.add_space(8.0);
                let tabs: Vec<ModeTab> = self.state.workspace.tabs().to_vec();
                let current = self.state.tab;
                for tab in tabs {
                    let selected = tab == current;
                    let text = RichText::new(tab.label()).strong().size(14.0);
                    let button = if selected {
                        egui::Button::new(text.color(Color32::WHITE)).fill(theme::ACCENT)
                    } else {
                        egui::Button::new(text.color(theme::MUTED)).fill(Color32::TRANSPARENT)
                    };
                    if ui.add(button).clicked() {
                        let _ = self.state.set_tab(tab);
                    }
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.add_space(10.0);
                    let stop = egui::Button::new(RichText::new("STOP").strong().color(Color32::WHITE).size(15.0))
                        .fill(theme::STOP)
                        .min_size(Vec2::new(88.0, 30.0));
                    if ui.add(stop).clicked() {
                        self.state.stop_run();
                    }
                    ui.add_space(8.0);
                    self.workspace_selector(ui);
                    ui.label(RichText::new("Workspace").color(theme::MUTED).size(12.0));
                });
            });
        });
    }

    fn workspace_selector(&mut self, ui: &mut egui::Ui) {
        let current = self.state.workspace.label().to_string();
        egui::ComboBox::from_id_salt("workspace").selected_text(current).width(180.0).show_ui(ui, |ui| {
            for workspace in Workspace::all() {
                if ui.selectable_label(self.state.workspace == workspace, workspace.label()).clicked() {
                    self.state.set_workspace(workspace);
                }
            }
            ui.add_enabled(false, egui::Button::new(format!("{} (planned)", parked_label(ParkedWorkspace::Custom))));
        });
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::bottom("status").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(22.0);
            ui.horizontal_centered(|ui| {
                ui.add_space(8.0);
                ui.label(RichText::new(&self.state.status).color(theme::TEXT).size(12.0));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.add_space(8.0);
                    ui.label(RichText::new(format!("CAMpanion {}", env!("CARGO_PKG_VERSION"))).color(theme::MUTED).size(11.0));
                    ui.separator();
                    ui.label(RichText::new("Units: mm").color(theme::MUTED).size(11.0));
                });
            });
        });
    }

    fn timeline(&mut self, ui: &mut egui::Ui) {
        egui::Panel::bottom("timeline").frame(panel_frame()).show(ui, |ui| {
            ui.set_height(88.0);
            ui.add_space(4.0);
            ui.label(RichText::new("Operation Timeline").color(theme::MUTED).size(12.0));
            let scene = self.scene.as_ref();
            let total = scene.map(|scene| scene.total_length).unwrap_or(0.0);
            if let Some(viewport) = self.viewport.as_mut() {
                ui.horizontal(|ui| {
                    let playing = viewport.playback.playing;
                    let label = if playing { "Pause" } else { "Play" };
                    if ui.button(label).clicked() {
                        if playing {
                            viewport.playback.playing = false;
                        } else if total > 0.0 {
                            if viewport.playback.position >= total {
                                viewport.playback.restart();
                            } else {
                                viewport.playback.playing = true;
                            }
                        }
                    }
                    if total > 0.0 {
                        let mut pos = viewport.playback.position;
                        if ui.add(egui::Slider::new(&mut pos, 0.0..=total).show_value(false)).changed() {
                            viewport.playback.position = pos;
                            viewport.playback.playing = false;
                        }
                        ui.label(RichText::new(format!("{pos:.1} / {total:.1} mm")).color(theme::MUTED).size(11.0));
                    } else {
                        ui.label(RichText::new("No toolpath loaded").color(theme::MUTED).size(11.0));
                    }
                });
                draw_timeline_segments(ui, scene, viewport.playback.position, total);
            } else {
                ui.label(RichText::new("Load an IR file to see operation segments, rapids and the playhead.").color(theme::MUTED).size(12.0));
                let _ = ui.allocate_exact_size(Vec2::new(ui.available_width(), 28.0), egui::Sense::hover());
            }
        });
    }

    fn body(&mut self, ui: &mut egui::Ui) {
        let show_tree = self.state.show_tree && self.state.tab != ModeTab::Run && self.state.tab != ModeTab::Slice;
        let show_props = self.state.show_properties && self.state.tab != ModeTab::Slice;
        if show_tree {
            egui::Panel::left("tree").resizable(true).default_size(240.0).frame(panel_frame()).show(ui, |ui| {
                self.process_tree(ui);
            });
        }
        if show_props {
            egui::Panel::right("props").resizable(true).default_size(280.0).frame(panel_frame()).show(ui, |ui| match self.state.tab {
                ModeTab::Run => self.run_right(ui),
                ModeTab::Export => self.export_form(ui),
                ModeTab::Orient => self.orient_properties(ui),
                ModeTab::Supports => self.supports_properties(ui),
                _ => self.default_properties(ui),
            });
        }
        if self.state.tab == ModeTab::Run {
            egui::Panel::left("run-left").resizable(true).default_size(230.0).frame(panel_frame()).show(ui, |ui| {
                self.run_left(ui);
            });
        }
        egui::CentralPanel::default().frame(egui::Frame::new().fill(theme::WINDOW).inner_margin(8.0)).show(ui, |ui| match self.state.tab {
            ModeTab::Process => self.process_tab(ui),
            ModeTab::Slice if self.state.workspace == Workspace::Resin || self.state.workspace == Workspace::Fdm => self.slice_tab(ui),
            ModeTab::Export if self.state.workspace == Workspace::Resin => self.resin_export_center(ui),
            ModeTab::Export => self.cutting_export_center(ui),
            ModeTab::Supports => self.supports_center(ui),
            ModeTab::Orient if self.state.workspace == Workspace::Resin || self.state.workspace == Workspace::Fdm => self.orient_center(ui),
            ModeTab::Run => self.viewport_panel(ui, "Simulation / Run View"),
            _ => self.viewport_panel(ui, "Viewport"),
        });
    }

    fn process_tree(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Process Tree").strong().size(14.0));
        ui.add_space(6.0);
        if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
            tree_line(ui, "Machine", "Not selected");
            tree_line(ui, if self.state.workspace == Workspace::Resin { "Resin" } else { "Material" }, "Not selected");
            ui.separator();
            ui.collapsing("Models", |ui| {
                ui.label(RichText::new("No model imported").color(theme::MUTED).size(12.0));
            });
            if self.state.workspace == Workspace::Resin {
                ui.collapsing("Supports", |ui| {
                    ui.label(RichText::new("Support generation is not wired in this preview").color(theme::MUTED).size(12.0));
                    ui.label(RichText::new("Scaffold: off").color(theme::MUTED).size(12.0));
                    ui.label(RichText::new("No-support zones: 0").color(theme::MUTED).size(12.0));
                });
            }
            return;
        }
        tree_line(ui, "Machine", "Not selected");
        tree_line(ui, "Heads", "Not selected");
        tree_line(ui, "Material", "Not selected");
        ui.separator();
        ui.collapsing("Operations", |ui| {
            if let Some(scene) = &self.scene {
                for (index, name) in scene.names.iter().enumerate() {
                    let label = if name.is_empty() { format!("Toolpath {}", index + 1) } else { name.clone() };
                    let id = format!("toolpath-{index}");
                    let selected = self.state.selection.as_deref() == Some(id.as_str());
                    if ui.selectable_label(selected, label).clicked() {
                        self.state.selection = Some(id);
                    }
                }
                if scene.names.is_empty() {
                    ui.label(RichText::new("No operations").color(theme::MUTED).size(12.0));
                }
            } else if let Some(job) = &self.state.pending_job {
                ui.label(RichText::new(job).color(theme::MUTED).size(12.0));
                ui.label(RichText::new("Not planned yet").color(theme::MUTED).size(12.0));
            } else {
                ui.label(RichText::new("No operations").color(theme::MUTED).size(12.0));
            }
        });
    }

    fn default_properties(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Properties").strong().size(14.0));
        ui.add_space(6.0);
        if let Some(selection) = &self.state.selection {
            ui.label(format!("Selected: {selection}"));
            ui.label(RichText::new("Kerf, lead-in and feed fields stay empty until a library value is entered.").color(theme::MUTED).size(11.0));
        } else {
            ui.label(RichText::new("Select an item in the process tree.").color(theme::MUTED).size(12.0));
        }
        ui.add_space(8.0);
        ui.label(RichText::new("Overrides are marked in orange, with Reset and Save to library.").color(theme::MUTED).size(11.0));
    }

    fn process_tab(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Select a sub-process").strong().size(16.0));
        ui.add_space(8.0);
        ui.label(RichText::new("This is the first tab in every cutting, mill and turning workspace.").color(theme::MUTED));
        ui.add_space(12.0);
        let processes: Vec<SubProcess> = self.state.workspace.sub_processes().to_vec();
        for process in processes {
            let selected = self.state.process == Some(process);
            if ui.selectable_label(selected, process.label()).clicked() {
                let _ = self.state.set_process(process);
            }
        }
        for planned in self.state.workspace.planned_sub_processes() {
            ui.add_enabled(false, egui::Button::new(planned.label()));
        }
    }

    fn orient_center(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            self.viewport_panel(ui, "Build volume (placeholder)");
        });
    }

    fn orient_properties(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Transform").strong().size(14.0));
        ui.add_space(6.0);
        ui.label(RichText::new("Position X Y Z (mm)").color(theme::MUTED).size(12.0));
        field_row(ui, "X", &mut self.state.position_mm[0], self.state.dark_theme);
        field_row(ui, "Y", &mut self.state.position_mm[1], self.state.dark_theme);
        field_row(ui, "Z", &mut self.state.position_mm[2], self.state.dark_theme);
        ui.add_space(6.0);
        ui.label(RichText::new("Rotation X Y Z (deg)").color(theme::MUTED).size(12.0));
        field_row(ui, "X", &mut self.state.rotation_deg[0], self.state.dark_theme);
        field_row(ui, "Y", &mut self.state.rotation_deg[1], self.state.dark_theme);
        field_row(ui, "Z", &mut self.state.rotation_deg[2], self.state.dark_theme);
        ui.add_space(6.0);
        ui.label(RichText::new("Rotate step").color(theme::MUTED).size(12.0));
        ui.horizontal(|ui| {
            for step in ROTATE_STEPS_DEG {
                let selected = self.state.rotate_step_deg == step;
                let label = format!("{step}°");
                if ui.selectable_label(selected, label).clicked() {
                    self.state.rotate_step_deg = step;
                }
            }
        });
        field_row(ui, "Lift from plate (mm)", &mut self.state.lift_mm, self.state.dark_theme);
        ui.add_space(8.0);
        if ui.button("Lay flat on face").clicked() {
            self.state.status = "Lay flat is a placeholder in this preview".to_string();
        }
        if ui.button("Auto orient").clicked() {
            self.state.status = "Auto orient is a placeholder in this preview".to_string();
        }
        if ui.button("Reset").clicked() {
            self.state.reset_orient();
        }
        ui.separator();
        ui.label(RichText::new("Array").strong().size(13.0));
        field_row(ui, "Count X", &mut self.state.array_x, self.state.dark_theme);
        field_row(ui, "Count Y", &mut self.state.array_y, self.state.dark_theme);
        field_row(ui, "Spacing (mm)", &mut self.state.array_spacing_mm, self.state.dark_theme);
        ui.separator();
        ui.label(RichText::new("Arrange  (A)").strong().size(13.0));
        field_row(ui, "Minimum gap (mm)", &mut self.state.arrange_gap_mm, self.state.dark_theme);
        if ui.button("Arrange").clicked() {
            self.state.status = "Arrange is a placeholder in this preview".to_string();
        }
        ui.separator();
        ui.label(RichText::new("Info").strong().size(13.0));
        ui.label(RichText::new("Volume: —").color(theme::MUTED).size(12.0));
        ui.label(RichText::new("Bounding box: —").color(theme::MUTED).size(12.0));
        ui.label(RichText::new("Overhang area: —").color(theme::MUTED).size(12.0));
    }

    fn supports_center(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            for (label, message) in [
                ("Auto Generate", "Auto Generate is a placeholder in this preview"),
                ("Add Support", "Add Support is a placeholder in this preview"),
                ("Delete", "Delete is a placeholder in this preview"),
                ("Select Region", "Select Region is a placeholder in this preview"),
                ("No-Support Zone", "No-support zones are a placeholder in this preview"),
                ("Scaffold", "Scaffold generation is a placeholder in this preview"),
            ] {
                if ui.button(label).clicked() {
                    self.state.status = message.to_string();
                }
            }
        });
        ui.add_space(6.0);
        self.viewport_panel(ui, "Build plate (placeholder)");
    }

    fn supports_properties(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Support Settings").strong().size(14.0));
        ui.add_space(4.0);
        ui.label(RichText::new("Fields start empty. The program ships no default sizes.").color(theme::MUTED).size(11.0));
        field_row(ui, "Tip diameter (mm)", &mut self.state.support.tip_diameter_mm, self.state.dark_theme);
        field_row(ui, "Contact depth (mm)", &mut self.state.support.contact_depth_mm, self.state.dark_theme);
        field_row(ui, "Shaft diameter (mm)", &mut self.state.support.shaft_diameter_mm, self.state.dark_theme);
        field_row(ui, "Max angle (deg)", &mut self.state.support.max_angle_deg, self.state.dark_theme);
        field_row(ui, "Density (%)", &mut self.state.support.density_pct, self.state.dark_theme);
        field_row(ui, "Scaffold spacing (mm)", &mut self.state.support.scaffold_spacing_mm, self.state.dark_theme);
        field_row(ui, "Raft thickness (mm)", &mut self.state.support.raft_thickness_mm, self.state.dark_theme);
        ui.separator();
        ui.label(RichText::new(format!("Selected: {} supports", self.state.support.selected_count)).strong().size(13.0));
        ui.label(RichText::new("Select supports after generation is wired.").color(theme::MUTED).size(11.0));
    }

    fn slice_tab(&mut self, ui: &mut egui::Ui) {
        ui.columns(3, |columns| {
            if let Some(left) = columns.get_mut(0) {
                left.label(RichText::new("3D Layer View").strong().size(14.0));
                placeholder_pane(left, "No slice yet.\nSlicing is not wired in this preview.");
            }
            if let Some(mid) = columns.get_mut(1) {
                mid.set_width(88.0);
                mid.vertical_centered(|ui| {
                    if ui.button("▲").clicked() {
                        let _ = self.state.step_layer(1);
                    }
                    let count = self.state.layer_count;
                    let caption = if count == 0 { "Layer — / —".to_string() } else { format!("Layer {} / {count}", self.state.layer) };
                    ui.label(RichText::new(caption).size(12.0));
                    if count > 0 {
                        let mut layer = self.state.layer;
                        if ui.add(egui::Slider::new(&mut layer, 1..=count).vertical().show_value(false)).changed() {
                            self.state.set_layer(layer);
                        }
                    } else {
                        let mut unused = 0u32;
                        ui.add_enabled(false, egui::Slider::new(&mut unused, 0..=1).vertical().show_value(false));
                    }
                    ui.add(egui::TextEdit::singleline(&mut self.state.layer_text).desired_width(56.0));
                    if ui.input(|input| input.key_pressed(egui::Key::Enter)) {
                        self.state.apply_layer_text();
                    }
                    ui.label(RichText::new("Arrow keys: up / down one layer").color(theme::MUTED).size(10.0));
                    if ui.button("▼").clicked() {
                        let _ = self.state.step_layer(-1);
                    }
                });
            }
            if let Some(right) = columns.get_mut(2) {
                right.label(RichText::new("Display Layer").strong().size(14.0));
                placeholder_pane(right, "LCD layer image appears here after slicing.");
            }
        });
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("Layer height: —").color(theme::MUTED));
            ui.separator();
            ui.label(RichText::new("Z height: —").color(theme::MUTED));
            ui.separator();
            ui.label(RichText::new("Print time estimate: —").color(theme::MUTED));
        });
    }

    fn cutting_export_center(&mut self, ui: &mut egui::Ui) {
        ui.columns(2, |columns| {
            if let Some(left) = columns.get_mut(0) {
                left.label(RichText::new("Toolpath").strong().size(14.0));
                if let Some(viewport) = self.viewport.as_mut() {
                    viewport::show(left, viewport, self.scene.as_ref());
                } else {
                    placeholder_pane(left, "Open an IR file to preview the toolpath.");
                }
            }
            if let Some(right) = columns.get_mut(1) {
                right.label(RichText::new("G-code preview").strong().size(14.0));
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(right, |ui| {
                    if self.state.gcode.is_empty() {
                        ui.label(
                            RichText::new("No G-code loaded. Open a .ngc file, or an IR file next to one of the same name.").color(theme::MUTED),
                        );
                    } else {
                        ui.add(egui::TextEdit::multiline(&mut self.state.gcode).code_editor().desired_width(f32::INFINITY));
                    }
                });
            }
        });
    }

    fn resin_export_center(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Job Summary").strong().size(16.0));
        ui.add_space(8.0);
        placeholder_pane(ui, "Thumbnail appears here after a slice exists.");
        ui.add_space(8.0);
        for label in ["Layers", "Layer height (mm)", "Print time estimate", "Resin volume (ml)", "Bottom layers", "Exposure profile"] {
            ui.horizontal(|ui| {
                ui.label(RichText::new(label).color(theme::MUTED));
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.label("—");
                });
            });
        }
        ui.separator();
        ui.label(RichText::new("Pre-flight checks").strong().size(13.0));
        ui.label(RichText::new("Pending: this preview does not run checks yet.").color(theme::MUTED).size(12.0));
        ui.label("Parts inside the build volume — not checked");
        ui.label("Slice up to date — not sliced");
        ui.label("Supports generated — not generated");
    }

    fn export_form(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Export").strong().size(14.0));
        ui.add_space(6.0);
        ui.label("Format");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_format).desired_width(200.0));
        ui.label("File name");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_name).desired_width(200.0));
        ui.label("Save to");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_path).desired_width(200.0));
        ui.checkbox(&mut self.state.embed_thumbnail, "Embed thumbnail preview");
        ui.add_space(8.0);
        let export = egui::Button::new(RichText::new("Export File").color(Color32::WHITE)).fill(theme::ACCENT).min_size(Vec2::new(200.0, 28.0));
        if ui.add(export).clicked() {
            self.state.status = "Export is not wired in this preview".to_string();
        }
        ui.add_space(6.0);
        ui.add_enabled(false, egui::Button::new("Send to printer").min_size(Vec2::new(200.0, 28.0)));
        ui.label(RichText::new("coming later").color(theme::MUTED).size(11.0));
    }

    fn run_left(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Machine").strong().size(14.0));
        ui.add_space(4.0);
        ui.label(RichText::new("Connection — layout only, no port is opened").color(theme::MUTED).size(11.0));
        ui.label("Port");
        ui.add(egui::TextEdit::singleline(&mut self.state.port).desired_width(180.0).hint_text(""));
        ui.label("Baud");
        ui.add(egui::TextEdit::singleline(&mut self.state.baud).desired_width(180.0).hint_text(""));
        ui.label(RichText::new("Not connected").color(theme::MUTED));
        ui.add_enabled(false, egui::Button::new("Disconnect"));
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Position");
            let work = self.state.dro_work;
            if ui.selectable_label(work, "Work").clicked() {
                self.state.dro_work = true;
            }
            if ui.selectable_label(!work, "Machine").clicked() {
                self.state.dro_work = false;
            }
        });
        let dro = self.state.dro();
        for (axis, value) in ["X", "Y", "Z"].iter().zip(dro.iter()) {
            ui.label(format!("{axis}  {value:.3} mm"));
        }
        ui.separator();
        ui.label(RichText::new("Jog").strong().size(13.0));
        let enabled = self.state.can_jog();
        ui.add_enabled_ui(enabled, |ui| {
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                if ui.button("↑").clicked() {
                    let _ = self.state.jog(1, 1);
                }
            });
            ui.horizontal(|ui| {
                if ui.button("←").clicked() {
                    let _ = self.state.jog(0, -1);
                }
                if ui.button("→").clicked() {
                    let _ = self.state.jog(0, 1);
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                if ui.button("↓").clicked() {
                    let _ = self.state.jog(1, -1);
                }
                ui.add_space(12.0);
                if ui.button("Z+").clicked() {
                    let _ = self.state.jog(2, 1);
                }
                if ui.button("Z−").clicked() {
                    let _ = self.state.jog(2, -1);
                }
            });
        });
        ui.label("Step size");
        ui.horizontal(|ui| {
            for step in JOG_STEPS_MM {
                let selected = (self.state.jog_step_mm - step).abs() < 1e-12;
                if ui.selectable_label(selected, format!("{step}")).clicked() {
                    self.state.jog_step_mm = step;
                }
            }
            ui.label("mm");
        });
        ui.horizontal(|ui| {
            ui.add_enabled_ui(self.state.can_home(), |ui| {
                if ui.button("Home").clicked() {
                    let _ = self.state.home();
                }
            });
            ui.add_enabled_ui(self.state.can_set_origin(), |ui| {
                if ui.button("Set Origin").clicked() {
                    let _ = self.state.set_origin();
                }
            });
        });
        if !enabled {
            ui.label(RichText::new("Jog is locked while a preview run is active. Stop first.").color(theme::MUTED).size(11.0));
        }
    }

    fn run_right(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Job").strong().size(14.0));
        ui.add_space(4.0);
        ui.add_enabled_ui(self.state.can_frame(), |ui| {
            if ui.add(egui::Button::new("Frame").min_size(Vec2::new(180.0, 26.0))).clicked() {
                self.state.status = "Frame is a preview only; no machine connection".to_string();
            }
        });
        ui.add_enabled_ui(self.state.can_start(), |ui| {
            let start = egui::Button::new(RichText::new("Start").color(Color32::WHITE)).fill(theme::START).min_size(Vec2::new(180.0, 28.0));
            if ui.add(start).clicked() {
                let _ = self.state.start_run();
            }
        });
        ui.add_enabled_ui(self.state.can_pause() || self.state.run == RunPhase::Paused, |ui| {
            let label = if self.state.run == RunPhase::Paused { "Resume" } else { "Pause" };
            if ui.add(egui::Button::new(label).min_size(Vec2::new(180.0, 28.0))).clicked() {
                if self.state.run == RunPhase::Paused {
                    let _ = self.state.resume_run();
                } else {
                    let _ = self.state.pause_run();
                }
            }
        });
        let phase = match self.state.run {
            RunPhase::Idle => "Idle — no machine connection",
            RunPhase::Running => "Running (preview only)",
            RunPhase::Paused => "Paused (preview only)",
            RunPhase::Stopped => "Stopped",
            RunPhase::Finished => "Finished",
        };
        ui.label(RichText::new(phase).color(theme::MUTED).size(12.0));
        ui.label(format!("Line {} / {}", self.state.run_line, self.state.run_lines));
        ui.separator();
        ui.label(RichText::new("Console").strong().size(13.0));
        egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
            ui.label(RichText::new("Not connected. Machine I/O is not available in this layout preview.").color(theme::MUTED).size(11.0).monospace());
        });
    }

    fn viewport_panel(&mut self, ui: &mut egui::Ui, caption: &str) {
        ui.label(RichText::new(caption).color(theme::MUTED).size(11.0));
        if let Some(viewport) = self.viewport.as_mut() {
            viewport::show(ui, viewport, self.scene.as_ref());
        } else {
            placeholder_pane(ui, "Open an IR file to preview the toolpath.\nFile → Open job…");
        }
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        if self.open_dialog {
            let mut open = true;
            egui::Window::new("Open").open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
                ui.label("Path to an IR JSON, job file, or G-code file:");
                ui.add(egui::TextEdit::singleline(&mut self.open_path_text).desired_width(420.0));
                ui.horizontal(|ui| {
                    if ui.button("Open").clicked() && !self.open_path_text.trim().is_empty() {
                        let path = PathBuf::from(self.open_path_text.trim());
                        self.open_file(&path);
                    }
                    if ui.button("Cancel").clicked() {
                        self.open_dialog = false;
                    }
                });
            });
            if !open {
                self.open_dialog = false;
            }
        }
        show_placeholder_window(ctx, "About CAMpanion", &mut self.about, |ui| {
            ui.label(RichText::new(format!("CAMpanion {}", env!("CARGO_PKG_VERSION"))).strong().size(16.0));
            ui.label("Machine-neutral CAM companion.");
            ui.add_space(6.0);
            ui.label("This window is an early layout preview: menus, navigation and viewports.");
            ui.label("It does not talk to a machine.");
            ui.add_space(6.0);
            ui.label("Licence: MIT OR Apache-2.0");
        });
        show_placeholder_window(ctx, "Preferences", &mut self.preferences, |ui| {
            ui.label("Units are millimetres.");
            ui.label("The program never ships default power, speed or material values.");
            ui.label("Further preferences will land with the full GUI.");
        });
        show_placeholder_window(ctx, "Machine profiles", &mut self.machine_profiles, |ui| {
            ui.label("Machine profiles are not wired in this preview.");
            ui.label("A later step will let a machine carry several heads, each with its own library.");
        });
        show_placeholder_window(ctx, "Material library", &mut self.material_library, |ui| {
            ui.label("The material library is not wired in this preview.");
            ui.label("Every value will be entered by the user. The program will not ship defaults.");
        });
    }

    fn tick_screenshots(&mut self, ctx: &egui::Context) {
        let mut switch_to: Option<(Workspace, ModeTab)> = None;
        {
            let Some(job) = self.screenshot.as_mut() else {
                return;
            };
            if let Some(error) = &job.error {
                eprintln!("campanion-gui: screenshot failed: {error}");
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                return;
            }
            if let Ok(image) = job.receiver.try_recv() {
                job.awaiting = false;
                if let Some((_, _, name)) = job.plan.get(job.index) {
                    let path = job.dir.join(name);
                    match save_screenshot(&path, &image) {
                        Ok(()) => {
                            job.written.push(path);
                            job.index += 1;
                            job.frames_waited = 0;
                            switch_to = job.plan.get(job.index).map(|(workspace, tab, _)| (*workspace, *tab));
                        }
                        Err(err) => job.error = Some(err),
                    }
                }
            }
        }
        if let Some((workspace, tab)) = switch_to {
            self.state.set_workspace(workspace);
            let _ = self.state.set_tab(tab);
        }
        let Some(job) = self.screenshot.as_mut() else {
            return;
        };
        if job.index >= job.plan.len() {
            for path in &job.written {
                eprintln!("campanion-gui: wrote {}", path.display());
            }
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            return;
        }
        if job.awaiting {
            ctx.request_repaint();
            return;
        }
        job.frames_waited = job.frames_waited.saturating_add(1);
        if job.frames_waited < 8 {
            ctx.request_repaint();
            return;
        }
        let sender = job.sender.clone();
        job.awaiting = true;
        ctx.request_screenshot(move |image| {
            let _ = sender.send(image);
        });
        ctx.request_repaint();
    }
}

fn save_screenshot(path: &Path, image: &egui::ColorImage) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
    }
    let width = u32::try_from(image.width()).map_err(|_| "screenshot is too wide".to_string())?;
    let height = u32::try_from(image.height()).map_err(|_| "screenshot is too tall".to_string())?;
    let mut rgba = Vec::with_capacity(image.pixels.len().saturating_mul(4));
    for pixel in &image.pixels {
        rgba.extend_from_slice(&pixel.to_array());
    }
    let buffer = image::RgbaImage::from_raw(width, height, rgba).ok_or_else(|| "could not pack screenshot pixels".to_string())?;
    buffer.save(path).map_err(|err| format!("{}: {err}", path.display()))
}

fn screenshot_ir_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        out.push(cwd.join("crates/cli/tests/golden/sample.ir.json"));
        out.push(cwd.join("cli/tests/golden/sample.ir.json"));
    }
    out.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../cli/tests/golden/sample.ir.json"));
    out
}

fn draw_wordmark(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(22.0, 22.0), egui::Sense::hover());
    let painter = ui.painter();
    painter.rect_filled(rect, CornerRadius::same(4), Color32::from_rgb(234, 122, 36));
    painter.rect_stroke(rect.shrink(3.0), CornerRadius::same(2), Stroke::new(1.4, Color32::from_rgb(20, 24, 30)), egui::StrokeKind::Inside);
    ui.add_space(6.0);
    ui.label(RichText::new("CAMpanion").strong().size(16.0));
}

fn bar_frame() -> egui::Frame {
    egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::symmetric(6, 2)).stroke(Stroke::new(1.0, theme::BORDER))
}

fn panel_frame() -> egui::Frame {
    egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::same(10)).stroke(Stroke::new(1.0, theme::BORDER))
}

fn tree_line(ui: &mut egui::Ui, label: &str, value: &str) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(format!("{label}:")).color(theme::MUTED).size(12.0));
        ui.label(RichText::new(value).size(12.0));
    });
}

fn placeholder_pane(ui: &mut egui::Ui, message: &str) {
    let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, theme::VIEWPORT);
    painter.rect_stroke(rect, 4.0, Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, message, egui::FontId::proportional(13.0), theme::MUTED);
}

fn field_row(ui: &mut egui::Ui, label: &str, field: &mut UserField, dark: bool) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).size(12.0));
        let overridden = field.is_overridden();
        let mut edit = egui::TextEdit::singleline(&mut field.value).desired_width(90.0);
        if overridden {
            edit = edit.text_color(theme::OVERRIDE).background_color(theme::override_fill(dark));
        }
        ui.add(edit);
        if overridden {
            if ui.small_button("Reset").clicked() {
                field.reset();
            }
            if ui.small_button("Save to library").clicked() {
                field.save_to_library();
            }
        }
    });
}

fn show_placeholder_window(ctx: &egui::Context, title: &str, open: &mut bool, body: impl FnOnce(&mut egui::Ui)) {
    if !*open {
        return;
    }
    egui::Window::new(title).open(open).collapsible(false).resizable(false).show(ctx, body);
}

fn parked_label(parked: ParkedWorkspace) -> &'static str {
    match parked {
        ParkedWorkspace::Custom => "Custom",
    }
}

fn draw_timeline_segments(ui: &mut egui::Ui, scene: Option<&Scene>, position: f64, total: f64) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 22.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 3.0, theme::WIDGET);
    let Some(scene) = scene else {
        return;
    };
    if !(total > 0.0) || rect.width() <= 1.0 {
        return;
    }
    let mut cursor = 0.0;
    let mut index = 0usize;
    for stroke in &scene.strokes {
        let start = cursor;
        cursor += stroke.length.max(0.0);
        let x0 = rect.left() + (start / total) as f32 * rect.width();
        let x1 = rect.left() + (cursor / total) as f32 * rect.width();
        let color = match stroke.kind {
            StrokeKind::Cut => {
                if index.is_multiple_of(2) {
                    theme::ACCENT
                } else {
                    Color32::from_rgb(14, 116, 144)
                }
            }
            StrokeKind::Rapid => Color32::from_rgb(80, 90, 104),
            StrokeKind::Plunge => theme::OVERRIDE,
        };
        if x1 > x0 {
            painter.rect_filled(
                egui::Rect::from_min_max(egui::Pos2::new(x0, rect.top() + 3.0), egui::Pos2::new(x1, rect.bottom() - 3.0)),
                2.0,
                color,
            );
        }
        if stroke.kind == StrokeKind::Cut {
            index = index.saturating_add(1);
        }
    }
    let x = rect.left() + ((position / total).clamp(0.0, 1.0) as f32) * rect.width();
    painter.line_segment([egui::Pos2::new(x, rect.top()), egui::Pos2::new(x, rect.bottom())], Stroke::new(1.5, theme::TOOL));
}
