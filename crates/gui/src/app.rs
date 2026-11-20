//! The egui window: menus, toolbars, viewports and job session panels.
//!
//! CAMpanion is primarily command-line driven. This GUI is still in development. The Run tab can
//! stream to a Grbl-class controller; that path is not hardware-tested.

use std::path::{Path, PathBuf};
use std::sync::{Arc, mpsc};
use std::time::Duration;

use campanion_ir::Document;
use campanion_view::{Playback, Projection, Scene, StrokeKind, find_system_font, projected_bounds};
use eframe::egui::{self, Align, Color32, CornerRadius, FontId, Layout, RichText, Stroke, Vec2};

use crate::edit::{self, SessionClipboard, SessionSnapshot};
use crate::fields::{self, ProcessFields};
use crate::files::{self, DialogKind, DialogResult, current_dir_display, export_stem, resolve_user_path};
use crate::machines::{self, MachinesUi};
use crate::operations::{self, RegenState};
use crate::run_stream::{self, RunConnection, STREAM_HARDWARE_NOTICE};
use crate::session::{
    GUI_PREVIEW_NOTICE, GUI_PREVIEW_NOTICE_SHORT, JobSession, MessageKind, ToolDraft, ToolKind, TreeSelection, add_imported_part, all_part_ids,
    all_part_layers, export_destination, format_number, generate_toolpaths, import_part_file, layer_parts_label, list_posts, list_posts_for_process,
    open_job_file, operation_match_warning, parse_placement_mm, placed_scene_parts, post_session, scene_from_drawing, suggested_post_for_process,
    write_gcode, write_job,
};
use crate::setup::{self, AlignMode, ArraySpec};
use crate::simulate::{PlayIndex, blocks_export, estimate_time, play_index, session_checks};
use crate::theme;
use crate::viewport::{self, Viewport, ViewportGuides};
use crate::{
    DEFAULT_ARC_TOL_MM, GuiState, JOG_STEPS_MM, ModeTab, OpenedDocument, ROTATE_STEPS_DEG, RunPhase, SubProcess, UserField, Workspace,
    gcode_line_count, open_path,
};
use campanion_stream::mock::MOCK_PORT_NAME;
use campanion_stream::{Bounds as FrameBounds, JobKind, frame_program};

const WINDOW_SIZE: [f32; 2] = [1600.0, 900.0];
const MAX_RECENT: usize = 8;
const MACHINE_CARD_WIDTH: f32 = 210.0;
const RIBBON_TAB_ROW: f32 = 30.0;
const RIBBON_GROUP_ROW: f32 = 58.0;
const RIBBON_HEIGHT: f32 = RIBBON_TAB_ROW + RIBBON_GROUP_ROW + 10.0;

/// Arguments accepted by the window after the binary has parsed its flags.
pub struct Launch {
    /// Optional file to open on startup.
    pub open: Option<PathBuf>,
    /// When set, capture the main views into this directory and exit.
    pub screenshot_dir: Option<PathBuf>,
}

/// Native window entry used by the `campanion-gui` binary.
///
/// The Run tab may open a user-selected serial port. Streaming is not hardware-tested.
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
                if launch.screenshot_dir.is_some() {
                    app.prepare_opened_file_for_screenshot();
                }
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum FileDialogKind {
    Open,
    Import,
    SaveJob,
    ExportGcode,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LayoutTool {
    Array,
    Arrange,
    Rotate,
}

struct App {
    state: GuiState,
    session: JobSession,
    document: Option<Document>,
    scene: Option<Scene>,
    viewport: Option<Viewport>,
    _has_font: bool,
    file_dialog: Option<FileDialogKind>,
    path_text: String,
    about: bool,
    preferences: bool,
    machines: Option<MachinesUi>,
    screenshot: Option<ScreenshotJob>,
    profile_store: campanion_cli::ProfileStore,
    clipboard: Option<SessionClipboard>,
    undo: Vec<SessionSnapshot>,
    layout_tool: Option<LayoutTool>,
    job_section_open: bool,
    parts_tree_open: bool,
    g1_demo_built: bool,
    machine_menu_open: bool,
    regen: RegenState,
    material_save_name: String,
    reveal_sheet_fields: bool,
    reveal_material_fields: bool,
    operations_tree_open: bool,
    play_index: Option<PlayIndex>,
    preview_speed_mult: f32,
    run_conn: Option<RunConnection>,
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
    finished: bool,
}

impl App {
    fn new(has_font: bool) -> Self {
        Self {
            state: GuiState::new(),
            session: JobSession::new(),
            document: None,
            scene: None,
            viewport: None,
            _has_font: has_font,
            file_dialog: None,
            path_text: String::new(),
            about: false,
            preferences: false,
            machines: None,
            screenshot: None,
            clipboard: None,
            undo: Vec::new(),
            profile_store: campanion_cli::ProfileStore::from_env().unwrap_or_else(|_| campanion_cli::ProfileStore::from_dir(PathBuf::from("."))),
            layout_tool: None,
            job_section_open: false,
            parts_tree_open: true,
            g1_demo_built: false,
            machine_menu_open: false,
            regen: RegenState::default(),
            material_save_name: String::new(),
            reveal_sheet_fields: false,
            reveal_material_fields: false,
            operations_tree_open: true,
            play_index: None,
            preview_speed_mult: 1.0,
            run_conn: None,
        }
    }

    fn open_file(&mut self, path: &Path) {
        let ext = path.extension().and_then(|ext| ext.to_str()).unwrap_or("");
        if is_part_ext(ext) {
            self.import_part(path);
            return;
        }
        if path_looks_like_job(path) {
            self.open_job(path);
            return;
        }
        match open_path(path) {
            Ok(OpenedDocument::Ir { path, scene, gcode }) => {
                self.session = JobSession::new();
                self.document = None;
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
            Ok(OpenedDocument::Job { path, .. }) => {
                self.open_job(&path);
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
        self.file_dialog = None;
    }

    fn open_job(&mut self, path: &Path) {
        match open_job_file(path) {
            Ok((session, imported)) => {
                self.apply_session(
                    session,
                    imported.as_ref().and_then(|part| part.drawing.as_ref()).map(|drawing| scene_from_drawing(drawing, DEFAULT_ARC_TOL_MM)),
                );
                if let Some(stem) = export_stem(path)
                    && self.state.export_name.is_empty()
                {
                    self.state.export_name = stem.to_string();
                }
                self.state.pending_job = Some(path.display().to_string());
                self.state.remember_open(path.to_path_buf());
                self.state.status = format!("Opened job {}", path.display());
                if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
                    self.state.set_workspace(Workspace::ThermalJet);
                }
                if self.state.workspace.has_tab(ModeTab::Operations) {
                    let _ = self.state.set_tab(ModeTab::Operations);
                }
            }
            Err(err) => {
                self.state.status = err;
            }
        }
        self.file_dialog = None;
    }

    fn import_part(&mut self, path: &Path) {
        match import_part_file(path) {
            Ok(part) => {
                self.push_undo();
                add_imported_part(&mut self.session, &part);
                let display = path.display().to_string();
                self.document = None;
                self.state.gcode.clear();
                self.refresh_placed_scene();
                self.state.remember_open(path.to_path_buf());
                if self.state.export_name.is_empty()
                    && let Some(stem) = export_stem(path)
                {
                    self.state.export_name = stem.to_string();
                }
                self.state.status = format!("Imported {display}");
                if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
                    self.state.set_workspace(Workspace::ThermalJet);
                }
                if self.state.workspace.has_tab(ModeTab::Setup) {
                    let _ = self.state.set_tab(ModeTab::Setup);
                }
                self.merge_session_messages();
            }
            Err(err) => {
                self.state.status = err;
            }
        }
        self.file_dialog = None;
    }

    fn apply_session(&mut self, session: JobSession, geometry: Option<Scene>) {
        self.session = session;
        self.document = None;
        self.state.gcode.clear();
        self.state.run_lines = 0;
        self.viewport = None;
        self.scene = geometry;
        self.state.has_scene = self.scene.is_some();
        self.refresh_placed_scene();
        if self.scene.is_none() {
            self.viewport = None;
            self.state.has_scene = false;
        }
        self.merge_session_messages();
        if !self.session.machine_profile.is_empty() || !self.session.machine_path.is_empty() {
            self.apply_machine_selection();
        }
    }

    fn merge_session_messages(&mut self) {
        if let Some(message) = self.session.messages.first() {
            self.state.status = message.text.clone();
        }
    }

    fn push_undo(&mut self) {
        const LIMIT: usize = 32;
        self.undo.push(SessionSnapshot { session: self.session.clone() });
        if self.undo.len() > LIMIT {
            self.undo.remove(0);
        }
    }

    fn undo_last(&mut self) {
        let Some(snapshot) = self.undo.pop() else {
            self.state.status = "Nothing to undo".to_string();
            return;
        };
        self.session = snapshot.session;
        self.refresh_placed_scene();
        self.state.status = "Undo".to_string();
    }

    fn show_planned_scene(&mut self, mut scene: Scene) {
        scene.bounds = expand_bounds(scene.bounds, self.bed_bounds());
        if let Ok(Some(sheet)) = setup::sheet_bounds(&self.session) {
            scene.bounds = expand_bounds(scene.bounds, Some(sheet));
        }
        self.viewport = Some(Viewport::new(&scene));
        self.scene = Some(scene);
        self.state.has_scene = true;
    }

    fn refresh_placed_scene(&mut self) {
        match placed_scene_parts(&self.session, DEFAULT_ARC_TOL_MM) {
            Ok(Some(mut scene)) => {
                scene.bounds = expand_bounds(scene.bounds, self.bed_bounds());
                if let Ok(Some(sheet)) = setup::sheet_bounds(&self.session) {
                    scene.bounds = expand_bounds(scene.bounds, Some(sheet));
                }
                self.assign_scene(scene);
                self.note_off_bed_parts();
            }
            Ok(None) => self.show_bed_scene(),
            Err(err) => {
                self.state.status = err;
                self.show_bed_scene();
            }
        }
    }

    fn show_bed_scene(&mut self) {
        let Some(bed) = self.bed_bounds() else {
            return;
        };
        let mut scene = empty_scene_with_bounds(bed);
        if let Ok(Some(sheet)) = setup::sheet_bounds(&self.session) {
            scene.bounds = expand_bounds(scene.bounds, Some(sheet));
        }
        self.assign_scene(scene);
        self.note_off_bed_parts();
    }

    fn assign_scene(&mut self, scene: Scene) {
        if let Some(viewport) = self.viewport.as_mut() {
            viewport.fitted = false;
            viewport.user_adjusted = false;
            viewport.last_size = None;
            viewport.playback = Playback::new(scene.total_length);
        } else {
            self.viewport = Some(Viewport::new(&scene));
        }
        self.scene = Some(scene);
        self.state.has_scene = true;
    }

    fn bed_bounds(&self) -> Option<[f64; 4]> {
        match machines::session_profile(&self.profile_store, &self.session) {
            Ok(Some((profile, _))) => {
                if let Some(bed) = profile.bed {
                    return Some([0.0, 0.0, bed.x, bed.y]);
                }
                if let Some(area) = profile.work_area {
                    let x0 = area.x_min.unwrap_or(0.0);
                    let y0 = area.y_min.unwrap_or(0.0);
                    let x1 = area.x_max?;
                    let y1 = area.y_max?;
                    return Some([x0, y0, x1, y1]);
                }
                None
            }
            _ => None,
        }
    }

    fn viewport_guides(&self) -> ViewportGuides {
        ViewportGuides { bed: self.bed_bounds(), sheet: setup::sheet_bounds(&self.session).ok().flatten(), stale: self.regen.stale }
    }

    fn note_off_bed_parts(&mut self) {
        self.session.messages.retain(|message| !message.text.contains("is off the bed") && !message.text.contains("is off the sheet"));
        let bed = self.bed_bounds();
        let sheet = setup::sheet_bounds(&self.session).ok().flatten();
        for part in &self.session.parts {
            let Some(drawing) = part.drawing.as_ref() else { continue };
            for instance in &part.instances {
                let Ok(Some(bounds)) = setup::placed_bounds(drawing, instance) else { continue };
                if let Some(bed) = bed
                    && !setup::inside_limits(bounds, [Some(bed[0]), Some(bed[1]), Some(bed[2]), Some(bed[3])])
                {
                    self.session.messages.push(crate::session::warning(format!("part `{}` instance `{}` is off the bed", part.id, instance.id)));
                } else if let Some(sheet) = sheet
                    && !setup::inside_limits(bounds, [Some(sheet[0]), Some(sheet[1]), Some(sheet[2]), Some(sheet[3])])
                {
                    self.session.messages.push(crate::session::warning(format!("part `{}` instance `{}` is off the sheet", part.id, instance.id)));
                }
            }
        }
    }

    fn apply_part_delta(&mut self, delta: [f64; 2]) {
        match parse_placement_mm(&self.session) {
            Ok((x, y, rotate)) => {
                self.session.placement_x = format_number(x + delta[0]);
                self.session.placement_y = format_number(y + delta[1]);
                let _ = rotate;
                self.session.commit_placement();
                self.session.dirty = true;
                self.document = None;
                self.state.gcode.clear();
                self.refresh_placed_scene();
            }
            Err(err) => self.state.status = err,
        }
    }

    fn process_fields(&self) -> ProcessFields {
        fields::fields_for(self.state.process, self.state.workspace)
    }

    fn request_open(&mut self) {
        self.launch_dialog(DialogKind::Open, None);
    }

    fn request_import(&mut self) {
        self.launch_dialog(DialogKind::Import, None);
    }

    fn request_save_as(&mut self) {
        let name = self.session.job_path.as_ref().and_then(|path| path.file_name().and_then(|name| name.to_str())).unwrap_or("job.json").to_string();
        self.launch_dialog(DialogKind::SaveJob, Some(&name));
    }

    fn request_export(&mut self) {
        let _ = self.state.set_tab(ModeTab::Export);
        let suggested = if self.state.export_name.is_empty() { "output.ngc".to_string() } else { self.state.export_name.clone() };
        self.launch_dialog(DialogKind::ExportGcode, Some(&suggested));
    }

    fn launch_dialog(&mut self, kind: DialogKind, suggested: Option<&str>) {
        let start = self.dialog_start_dir();
        match files::pick_path(kind, start.as_deref(), suggested) {
            DialogResult::Picked(path) => self.apply_dialog_path(kind, &path),
            DialogResult::Cancelled => {}
            DialogResult::Unavailable => {
                self.file_dialog = Some(match kind {
                    DialogKind::Open => FileDialogKind::Open,
                    DialogKind::Import => FileDialogKind::Import,
                    DialogKind::SaveJob => FileDialogKind::SaveJob,
                    DialogKind::ExportGcode => FileDialogKind::ExportGcode,
                });
                self.path_text = suggested.unwrap_or("").to_string();
                self.state.status = "No system file chooser was available; type a path.".to_string();
            }
        }
    }

    fn dialog_start_dir(&self) -> Option<PathBuf> {
        if let Some(path) = self.session.job_path.as_ref().and_then(|path| path.parent()) {
            return Some(path.to_path_buf());
        }
        std::env::current_dir().ok()
    }

    fn apply_dialog_path(&mut self, kind: DialogKind, path: &Path) {
        match kind {
            DialogKind::Open => self.open_file(path),
            DialogKind::Import => self.import_part(path),
            DialogKind::SaveJob => self.save_job_to(path),
            DialogKind::ExportGcode => self.export_gcode_to(path),
        }
    }

    fn confirm_typed_path(&mut self, kind: FileDialogKind) {
        let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
        let home = std::env::var_os("HOME").map(PathBuf::from);
        match resolve_user_path(&self.path_text, &cwd, home.as_deref()) {
            Ok(path) => {
                let mapped = match kind {
                    FileDialogKind::Open => DialogKind::Open,
                    FileDialogKind::Import => DialogKind::Import,
                    FileDialogKind::SaveJob => DialogKind::SaveJob,
                    FileDialogKind::ExportGcode => DialogKind::ExportGcode,
                };
                self.file_dialog = None;
                self.apply_dialog_path(mapped, &path);
            }
            Err(err) => self.state.status = err,
        }
    }

    fn prepare_opened_file_for_screenshot(&mut self) {
        if self.session.operations.is_empty() && self.session.source_drawing.is_none() {
            return;
        }
        if self.session.post_id.is_empty() {
            if let Some(process) = self.state.process
                && let Some(id) = suggested_post_for_process(process.label())
            {
                self.session.post_id = id.to_string();
            } else if self.session.tools.iter().any(|tool| tool.kind == ToolKind::Kerf) {
                self.session.post_id = "grbl_laser".to_string();
                self.state.set_workspace(Workspace::ThermalJet);
                let _ = self.state.set_process(SubProcess::Laser);
            }
        }
        self.generate_and_post();
    }

    fn show_viewport(&mut self, ui: &mut egui::Ui, caption: &str) {
        ui.label(RichText::new(caption).color(theme::MUTED).size(11.0));
        self.show_viewport_area(ui);
    }

    fn show_viewport_area(&mut self, ui: &mut egui::Ui) {
        let allow_drag =
            (self.session.source_drawing.is_some() || self.session.parts.iter().any(|part| part.drawing.is_some())) && self.document.is_none();
        let guides = self.viewport_guides();
        if let Some(viewport) = self.viewport.as_mut() {
            let want_content =
                self.document.is_some() && matches!(self.state.tab, ModeTab::Operations | ModeTab::Simulate | ModeTab::Export | ModeTab::Run);
            if viewport.fit_content != want_content {
                viewport.fit_content = want_content;
                if !viewport.user_adjusted {
                    viewport.fitted = false;
                }
            }
            let output = viewport::show_with(ui, viewport, self.scene.as_ref(), allow_drag, Some(&guides));
            if let Some(delta) = output.part_delta {
                self.apply_part_delta(delta);
            }
        } else {
            placeholder_pane(ui, "Import a part or open a job to preview it.\nFile → Open… or File → Import part…");
        }
    }

    fn copy_selection(&mut self) {
        match edit::copy_selection(&self.session) {
            Ok(clip) => {
                self.clipboard = Some(clip);
                self.state.status = "Copied".to_string();
            }
            Err(err) => self.state.status = err,
        }
    }

    fn paste_selection(&mut self) {
        let Some(clip) = self.clipboard.clone() else {
            self.state.status = "The clipboard is empty".to_string();
            return;
        };
        self.push_undo();
        match edit::paste_clipboard(&mut self.session, &clip) {
            Ok(status) => self.state.status = status,
            Err(err) => {
                self.undo.pop();
                self.state.status = err;
            }
        }
    }

    fn duplicate_selection(&mut self) {
        self.push_undo();
        match edit::duplicate_selection(&mut self.session) {
            Ok(status) => self.state.status = status,
            Err(err) => {
                self.undo.pop();
                self.state.status = err;
            }
        }
    }

    fn delete_selection(&mut self) {
        self.push_undo();
        match edit::delete_selection(&mut self.session) {
            Ok(status) => {
                self.state.status = status;
                self.document = None;
                self.state.gcode.clear();
                if self.session.source_drawing.is_none() {
                    self.viewport = None;
                    self.scene = None;
                    self.state.has_scene = false;
                }
            }
            Err(err) => {
                self.undo.pop();
                self.state.status = err;
            }
        }
    }

    fn show_messages(&self, ui: &mut egui::Ui) {
        if self.session.messages.is_empty() {
            return;
        }
        ui.separator();
        ui.label(RichText::new("Messages").strong().size(13.0));
        for message in &self.session.messages {
            let (color, prefix) = match message.kind {
                MessageKind::Error => (theme::STOP, "Error"),
                MessageKind::Warning => (theme::OVERRIDE, "Warning"),
                MessageKind::Note => (theme::MUTED, "Note"),
            };
            ui.label(RichText::new(format!("{prefix}: {}", message.text)).color(color).size(12.0));
        }
    }

    fn tick_regen(&mut self, ctx: &egui::Context) {
        if self.screenshot.is_some() {
            return;
        }
        if let Some((result, _current)) = self.regen.tick(&self.session, &self.profile_store) {
            match result {
                Ok(planned) => {
                    self.session.messages = planned.messages;
                    self.document = Some(planned.document);
                    self.show_planned_scene(planned.scene);
                    let n = self.scene.as_ref().map(|scene| scene.names.len()).unwrap_or(0);
                    self.state.pending_job = self.session.job_path.as_ref().map(|path| path.display().to_string());
                    if self.session.messages.is_empty() {
                        self.state.status = format!("Generated {n} toolpath(s)");
                    } else {
                        self.merge_session_messages();
                    }
                    self.refresh_gcode_preview();
                }
                Err(err) if err.contains("cancelled") => {}
                Err(err) => {
                    self.state.status = err;
                }
            }
        }
        if self.regen.needs_repaint() {
            ctx.request_repaint_after(Duration::from_millis(50));
        }
    }

    fn generate_paths(&mut self) -> bool {
        match campanion_cli::with_store(&self.profile_store, || generate_toolpaths(&self.session)) {
            Ok(planned) => {
                self.regen.stale = false;
                self.session.messages = planned.messages;
                self.document = Some(planned.document);
                self.show_planned_scene(planned.scene);
                let n = self.scene.as_ref().map(|scene| scene.names.len()).unwrap_or(0);
                self.state.pending_job = self.session.job_path.as_ref().map(|path| path.display().to_string());
                if self.session.messages.is_empty() {
                    self.state.status = format!("Generated {n} toolpath(s)");
                } else {
                    self.merge_session_messages();
                }
                self.refresh_gcode_preview();
                true
            }
            Err(err) => {
                self.session.messages = vec![crate::session::SessionMessage { kind: MessageKind::Error, text: err.clone() }];
                self.state.gcode.clear();
                self.play_index = None;
                self.state.status = err;
                false
            }
        }
    }

    fn generate_and_post(&mut self) {
        if self.generate_paths() {
            self.preview_gcode();
        }
    }

    fn preview_gcode(&mut self) {
        if self.session.post_id.trim().is_empty()
            && let Some(process) = self.state.process
            && let Some(id) = suggested_post_for_process(process.label())
        {
            self.session.post_id = id.to_string();
        }
        let Some(document) = self.document.as_ref() else {
            self.state.status = "Generate toolpaths before previewing G-code".to_string();
            return;
        };
        match campanion_cli::with_store(&self.profile_store, || post_session(&self.session, document)) {
            Ok((text, messages)) => {
                self.state.gcode = text;
                self.state.run_lines = gcode_line_count(&self.state.gcode);
                self.session.messages.extend(messages);
                if let Some(message) = self.session.messages.last() {
                    if self.session.messages.iter().any(|item| item.kind != MessageKind::Note) {
                        self.state.status = message.text.clone();
                    } else {
                        self.state.status = "G-code preview updated".to_string();
                    }
                } else {
                    self.state.status = "G-code preview updated".to_string();
                }
                self.rebuild_play_index();
            }
            Err(err) => {
                self.play_index = None;
                self.state.status = err;
            }
        }
    }

    fn save_job_to(&mut self, path: &Path) {
        match write_job(path, &self.session) {
            Ok(()) => {
                self.session.job_path = Some(path.to_path_buf());
                self.session.dirty = false;
                self.state.remember_open(path.to_path_buf());
                if self.document.is_some() {
                    self.generate_and_post();
                }
                self.state.status = format!("Saved {}", path.display());
            }
            Err(err) => {
                self.state.status = err;
            }
        }
        self.file_dialog = None;
    }

    fn save_job(&mut self) {
        if let Some(path) = self.session.job_path.clone() {
            self.save_job_to(&path);
        } else {
            self.file_dialog = Some(FileDialogKind::SaveJob);
            self.path_text.clear();
        }
    }

    fn export_gcode_to(&mut self, path: &Path) {
        let checks = session_checks(&self.session, self.document.as_ref(), Some(&self.profile_store), self.current_process_id());
        if blocks_export(&checks) {
            if let Some(item) = checks.iter().find(|item| item.blocks_export()) {
                self.state.status = format!("Export blocked: {}", item.message);
            }
            self.file_dialog = None;
            return;
        }
        if self.state.gcode.is_empty() {
            self.preview_gcode();
        }
        if self.state.gcode.is_empty() {
            return;
        }
        match write_gcode(path, &self.state.gcode) {
            Ok(()) => {
                self.state.status = format!("Wrote {}", path.display());
            }
            Err(err) => {
                self.state.status = err;
            }
        }
        self.file_dialog = None;
    }

    fn export_gcode(&mut self) {
        match export_destination(&self.state.export_path, &self.state.export_name, &self.state.export_format) {
            Ok(path) => self.export_gcode_to(&path),
            Err(err) => {
                self.path_text = self.state.export_path.clone();
                self.state.status = err;
                self.file_dialog = Some(FileDialogKind::ExportGcode);
            }
        }
    }

    fn apply_document_new(&mut self) {
        self.session = JobSession::new();
        self.document = None;
        self.scene = None;
        self.viewport = None;
        self.play_index = None;
        self.state.new_job();
    }

    fn mark_dirty(&mut self) {
        self.session.dirty = true;
        self.schedule_regen();
    }

    fn schedule_regen(&mut self) {
        self.regen.note_edit(self.document.is_some());
    }

    #[allow(dead_code)]
    fn open_sample_for_screenshot(&mut self) {
        for candidate in screenshot_job_candidates() {
            if candidate.is_file() {
                self.open_job(&candidate);
                if self.session.post_id.is_empty() {
                    self.session.post_id = "grbl_laser".to_string();
                }
                self.state.set_workspace(Workspace::ThermalJet);
                let _ = self.state.set_process(SubProcess::Laser);
                self.generate_and_post();
                return;
            }
        }
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
                (Workspace::ThermalJet, ModeTab::Run, "g5-run-connected.png".to_string()),
                (Workspace::ThermalJet, ModeTab::Run, "g5-run-streaming.png".to_string()),
                (Workspace::ThermalJet, ModeTab::Run, "g5-run-paused.png".to_string()),
                (Workspace::ThermalJet, ModeTab::Run, "g5-run-alarm.png".to_string()),
            ],
            index: 0,
            frames_waited: 0,
            awaiting: false,
            receiver,
            sender,
            written: Vec::new(),
            error: None,
            finished: false,
        });
        if let Some(job) = &self.screenshot {
            let (workspace, tab, name) = &job.plan[0];
            let workspace = *workspace;
            let tab = *tab;
            let name = name.clone();
            self.state.set_workspace(workspace);
            let _ = self.state.set_tab(tab);
            self.prepare_screenshot_frame(Some(name.as_str()));
        }
    }

    fn prepare_screenshot_frame(&mut self, name: Option<&str>) {
        self.reveal_sheet_fields = name == Some("g1-setup-origin.png");
        self.reveal_material_fields = name == Some("g2-material-override.png");
        self.operations_tree_open = true;
        if name.is_some_and(|name| name.starts_with("g5-")) {
            self.prepare_g5_screenshot(name);
            return;
        }
        if name.is_some_and(|name| name.starts_with("g3-")) {
            self.prepare_g3_screenshot(name);
            return;
        }
        let g1 = name.is_some_and(|name| name.starts_with("g1-"));
        let Some(job) = self.screenshot.as_ref() else {
            return;
        };
        let store = campanion_cli::ProfileStore::from_dir(job.dir.join("config"));
        let example = machines::screenshot_example("Example shop laser (example values)");
        let resin_example = machines::screenshot_example_resin("Example resin printer (example values)");
        let resin = name.is_some_and(|name| name.starts_with("resin-"));
        let _ = store.save("example-shop-laser", &example);
        let _ = store.save("example-shop-resin", &resin_example);
        self.profile_store = store.clone();
        self.machine_menu_open = name == Some("job-machine-picker.png");
        if resin {
            self.session.machine_profile = "example-shop-resin".to_string();
            self.session.machine_head.clear();
            self.apply_machine_selection();
        } else {
            self.session.machine_profile = "example-shop-laser".to_string();
            self.session.machine_head = "laser".to_string();
            self.apply_machine_selection();
        }
        if name == Some("machines-manager.png") {
            let mut manager = MachinesUi::open(store);
            manager.select_profile("example-shop-laser");
            self.machines = Some(manager);
        } else {
            self.machines = None;
        }
        if g1 {
            self.prepare_g1_screenshot(name);
        }
        if name.is_some_and(|name| name.starts_with("g2-")) {
            self.prepare_g2_screenshot(name);
        }
    }

    fn prepare_g5_screenshot(&mut self, name: Option<&str>) {
        let Some(job) = self.screenshot.as_ref() else {
            return;
        };
        let store = campanion_cli::ProfileStore::from_dir(job.dir.join("config"));
        let mut profile = machines::first_user_laser_profile();
        // A size the screenshot user typed; not a shipped default.
        profile.rx_buffer = Some(128);
        let _ = store.save("my-laser", &profile);
        self.profile_store = store.clone();
        self.machines = None;
        self.machine_menu_open = false;
        self.state.set_workspace(Workspace::ThermalJet);
        let _ = self.state.set_process(SubProcess::Laser);
        self.session.machine_profile = "my-laser".to_string();
        self.session.machine_head = "laser".to_string();
        self.apply_machine_selection();
        if self.session.parts.is_empty()
            && let Some(path) = screenshot_part_path("laser_rect.svg")
        {
            match import_part_file(&path) {
                Ok(part) => add_imported_part(&mut self.session, &part),
                Err(err) => self.state.status = err,
            }
        }
        if let Some(instance) = self.session.parts.first_mut().and_then(|part| part.instances.first_mut()) {
            instance.x = "20".to_string();
            instance.y = "20".to_string();
        }
        if self.session.operations.is_empty() {
            if self.session.tools.is_empty() {
                self.session.tools = vec![ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: String::new() }];
            }
            self.add_session_operation("profile");
            if let Some(op) = self.session.operations.last_mut() {
                op.name = "outline".to_string();
                op.side = "outside".to_string();
            }
            self.session.selected = Some(TreeSelection::Operation(0));
            if let Some(entry) = profile.heads.first().and_then(|head| head.materials.first()) {
                let _ = machines::apply_material(&mut self.session, entry);
            }
        }
        self.generate_and_post();
        self.state.port = MOCK_PORT_NAME.to_string();
        self.state.baud = "115200".to_string();
        self.state.safe_mode = false;
        self.state.frame_feed = "1500".to_string();
        self.state.jog_feed = "300".to_string();
        self.state.dro_work = true;
        if self.run_conn.is_some() {
            self.stream_disconnect();
        }
        self.stream_connect();
        if let Some(conn) = self.run_conn.as_mut()
            && let Some(mock) = conn.mock_mut()
        {
            mock.set_position([25.0, 15.0, 0.0], [0.0, 0.0, 0.0]);
        }
        let _ = self.tick_stream_once(true);

        if name == Some("g5-run-connected.png") {
            self.state.status = "Connected to mock-grbl (in-process simulator)".to_string();
            return;
        }

        if let Some(conn) = self.run_conn.as_mut()
            && let Some(mock) = conn.mock_mut()
        {
            mock.defer_ok(true);
        }
        self.stream_start();
        if let Some(conn) = self.run_conn.as_mut()
            && let Some(mock) = conn.mock_mut()
        {
            mock.release_ok(4);
        }
        let _ = self.tick_stream_once(true);
        if let Some(line) = self.run_conn.as_ref().and_then(|conn| conn.last_acked_program_line()) {
            self.seek_gcode_line(line);
        }

        if name == Some("g5-run-streaming.png") {
            self.state.status = "Streaming; playhead follows acknowledged lines".to_string();
            return;
        }

        self.stream_pause();
        let _ = self.tick_stream_once(true);
        if name == Some("g5-run-paused.png") {
            self.state.status = "Feed hold — jog is locked; Resume and STOP stay available".to_string();
            return;
        }

        if let Some(conn) = self.run_conn.as_mut()
            && let Some(mock) = conn.mock_mut()
        {
            mock.set_in_motion(true);
        }
        self.stream_stop();
        let _ = self.tick_stream_once(true);
        self.state.status = "ALARM:3 after STOP mid-motion — $X requires confirm".to_string();
    }

    fn prepare_g3_screenshot(&mut self, name: Option<&str>) {
        let Some(job) = self.screenshot.as_ref() else {
            return;
        };
        let store = campanion_cli::ProfileStore::from_dir(job.dir.join("config"));
        self.profile_store = store.clone();
        self.machine_menu_open = false;
        self.layout_tool = None;
        self.job_section_open = false;
        self.parts_tree_open = true;
        self.operations_tree_open = true;
        self.state.set_workspace(Workspace::ThermalJet);
        let _ = self.state.set_process(SubProcess::Laser);

        if name == Some("g3-machines-empty.png") {
            self.machines = Some(MachinesUi::open(store));
            return;
        }

        let profile = machines::first_user_laser_profile();
        let _ = store.save("my-laser", &profile);
        self.profile_store = store.clone();

        if name == Some("g3-machines-new.png") {
            let mut manager = MachinesUi::open(store);
            manager.select_profile("my-laser");
            self.machines = Some(manager);
            return;
        }
        self.machines = None;

        self.session.machine_profile = "my-laser".to_string();
        self.session.machine_head = "laser".to_string();
        self.apply_machine_selection();

        if name == Some("g3-job-new.png") {
            self.machine_menu_open = true;
            self.refresh_placed_scene();
            return;
        }

        if self.session.parts.is_empty()
            && let Some(path) = screenshot_part_path("laser_rect.svg")
        {
            match import_part_file(&path) {
                Ok(part) => add_imported_part(&mut self.session, &part),
                Err(err) => self.state.status = err,
            }
        }
        if let Some(instance) = self.session.parts.first_mut().and_then(|part| part.instances.first_mut()) {
            instance.x = "20".to_string();
            instance.y = "20".to_string();
        }

        if name == Some("g3-setup-import.png") {
            self.refresh_placed_scene();
            return;
        }

        if self.session.operations.is_empty() {
            if self.session.tools.is_empty() {
                self.session.tools = vec![ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: String::new() }];
            }
            self.add_session_operation("profile");
            if let Some(op) = self.session.operations.last_mut() {
                op.name = "outline".to_string();
                op.side = "outside".to_string();
            }
            self.session.selected = Some(TreeSelection::Operation(0));
            if let Some(entry) = profile.heads.first().and_then(|head| head.materials.first()) {
                let _ = machines::apply_material(&mut self.session, entry);
            }
        }

        if name == Some("g3-export-blocked.png") {
            if let Some(instance) = self.session.parts.first_mut().and_then(|part| part.instances.first_mut()) {
                instance.x = "700".to_string();
                instance.y = "20".to_string();
            }
            self.session.placement_x = "700".to_string();
            self.session.placement_y = "20".to_string();
            self.generate_and_post();
            // Planning may refuse a toolpath outside the entered work area; show the part
            // against the bed so the off-bed pre-flight error is visible.
            self.refresh_placed_scene();
            return;
        }

        self.generate_and_post();
        if name == Some("g3-simulate.png") {
            self.seek_playhead_fraction(0.4);
        }
        if name == Some("g3-simulate-scrub.png") {
            self.seek_playhead_fraction(0.72);
        }
    }

    fn prepare_g2_screenshot(&mut self, name: Option<&str>) {
        self.ensure_g1_demo();
        // Restore on-bed placements so a previous off-bed frame does not linger.
        self.restore_g1_demo_placements();
        self.layout_tool = None;
        self.job_section_open = false;
        // Collapse the long Parts list so Operations and the cut-order list are on screen.
        self.parts_tree_open = false;
        self.operations_tree_open = true;
        self.seed_g2_operations();
        if name == Some("g2-cut-order.png") {
            self.session.cut_order_explicit = true;
        }
        if name == Some("g2-material-override.png")
            && let Some(op) = self.session.operations.first_mut()
        {
            op.material_name = "Example: 3 mm acrylic (example values)".to_string();
            op.override_fields = vec!["cut_feed".to_string(), "power".to_string()];
            op.cut_feed = "220".to_string();
            op.power = "0.6".to_string();
            self.session.selected = Some(TreeSelection::Operation(0));
        }
        if name == Some("g2-operations.png") {
            self.session.selected = Some(TreeSelection::Operation(0));
            if let Some(op) = self.session.operations.first_mut() {
                op.tabs_count = "4".to_string();
                op.tabs_width = "2".to_string();
                op.lead_in = "line".to_string();
                op.lead_in_size = "2".to_string();
            }
        }
        self.generate_paths();
        // Do not refresh_placed_scene after planning: that would replace toolpaths with part outlines.
        if name == Some("g2-stale-toolpaths.png") {
            self.regen.stale = true;
            self.state.status = "Toolpaths are stale until regenerate finishes".to_string();
        }
    }

    fn seed_g2_operations(&mut self) {
        if self.session.tools.is_empty() || self.session.tools.iter().all(|tool| tool.size.trim().is_empty()) {
            self.session.tools =
                vec![crate::session::ToolDraft { id: 1, name: String::new(), kind: crate::session::ToolKind::Kerf, size: "0.1".to_string() }];
        }
        self.session.operations.clear();
        self.add_session_operation("profile");
        if let Some(op) = self.session.operations.last_mut() {
            op.name = "outline".to_string();
            op.side = "outside".to_string();
            op.depth = "3".to_string();
            op.step_down = "3".to_string();
            op.cut_feed = "300".to_string();
            op.power = "0.8".to_string();
        }
        self.add_session_operation("engrave");
        if let Some(op) = self.session.operations.last_mut() {
            op.name = "engrave".to_string();
            op.depth = "0.26".to_string();
            op.step_down = "0.26".to_string();
            op.cut_feed = "400".to_string();
            op.power = "0.2".to_string();
        }
        self.session.order_rules_set = true;
        self.session.order_inner_first = true;
        self.session.order_cuts_last = true;
        self.session.cut_order_explicit = false;
    }

    fn prepare_g1_screenshot(&mut self, name: Option<&str>) {
        self.ensure_g1_demo();
        self.layout_tool = None;
        self.job_section_open = false;
        self.parts_tree_open = true;
        self.restore_g1_demo_placements();
        if name == Some("g1-setup-array.png") {
            self.state.array_x.value = "3".to_string();
            self.state.array_y.value = "2".to_string();
            self.state.array_spacing_mm.value = "110".to_string();
            self.state.array_spacing_y_mm.value = "70".to_string();
            self.layout_tool = Some(LayoutTool::Array);
        }
        if name == Some("g1-setup-arrange.png") {
            self.state.arrange_gap_mm.value = "8".to_string();
            let width = self.viewport_guides().sheet.or(self.viewport_guides().bed).map(|bounds| bounds[2] - bounds[0]);
            let _ = setup::arrange(&mut self.session, "8", width);
            self.layout_tool = Some(LayoutTool::Arrange);
        }
        if name == Some("g1-setup-origin.png") {
            self.session.z_zero = "bed".to_string();
            self.session.stock_thickness = "6".to_string();
            self.session.datum_anchor = "center".to_string();
            self.job_section_open = true;
        }
        if name == Some("g1-setup-off-bed.png") {
            self.move_g1_part_off_bed();
        }
        self.document = None;
        self.state.gcode.clear();
        self.refresh_placed_scene();
        self.parts_tree_open = true;
    }

    fn ensure_g1_demo(&mut self) {
        if self.g1_demo_built {
            return;
        }
        self.session.sheet_width = "500".to_string();
        self.session.sheet_height = "320".to_string();
        self.session.z_zero = "stock_top".to_string();
        self.session.datum_anchor = "min_min".to_string();
        self.session.start_from = "absolute".to_string();
        self.session.parts.clear();
        self.session.messages.clear();
        self.document = None;
        self.state.gcode.clear();
        for name in ["sample.svg", "curves.svg", "nest.dxf"] {
            if let Some(path) = screenshot_part_path(name) {
                match import_part_file(&path) {
                    Ok(part) => add_imported_part(&mut self.session, &part),
                    Err(err) => self.state.status = err,
                }
            }
        }
        self.apply_g1_demo_layout();
        self.g1_demo_built = true;
        self.parts_tree_open = true;
    }

    fn apply_g1_demo_layout(&mut self) {
        // Place three different imported parts, then array the first 3×2 and add a linked copy of the last.
        if let Some(instance) = self.session.parts.first_mut().and_then(|part| part.instances.first_mut()) {
            instance.x = "10".to_string();
            instance.y = "10".to_string();
        }
        if !self.session.parts.is_empty() {
            self.session.select_instance(0, 0);
            if let Ok(spec) = ArraySpec::parse("3", "2", "110", "70") {
                let _ = setup::array_copies(&mut self.session, spec);
            }
        }
        if let Some(instance) = self.session.parts.get_mut(1).and_then(|part| part.instances.first_mut()) {
            instance.x = "360".to_string();
            instance.y = "20".to_string();
        }
        if let Some(instance) = self.session.parts.get_mut(2).and_then(|part| part.instances.first_mut()) {
            instance.x = "360".to_string();
            instance.y = "220".to_string();
        }
        if self.session.parts.len() >= 3 {
            self.session.select_instance(2, 0);
            let _ = setup::linked_copy(&mut self.session);
        }
        if !self.session.parts.is_empty() {
            self.session.select_instance(0, 0);
        }
    }

    fn restore_g1_demo_placements(&mut self) {
        if !self.g1_demo_built {
            return;
        }
        // Rebuild the known demo placements so later frames are not left in an arrange or off-bed state.
        let first_id = self.session.parts.first().map(|part| part.id.clone());
        let second_id = self.session.parts.get(1).map(|part| part.id.clone());
        let third_id = self.session.parts.get(2).map(|part| part.id.clone());
        self.session
            .parts
            .retain(|part| Some(&part.id) == first_id.as_ref() || Some(&part.id) == second_id.as_ref() || Some(&part.id) == third_id.as_ref());
        for part in self.session.parts.iter_mut().take(3) {
            part.instances.truncate(1);
        }
        self.apply_g1_demo_layout();
        self.session.z_zero = "stock_top".to_string();
        self.session.stock_thickness.clear();
        self.session.datum_anchor = "min_min".to_string();
        self.session.sheet_width = "500".to_string();
        self.session.sheet_height = "320".to_string();
    }

    fn move_g1_part_off_bed(&mut self) {
        // Move the unique nest part so it crosses the 600 mm example bed edge and stays in view.
        let part_index = if self.session.parts.len() >= 3 { 2 } else { self.session.parts.len().saturating_sub(1) };
        if let Some(part) = self.session.parts.get_mut(part_index)
            && let Some(instance) = part.instances.first_mut()
        {
            instance.x = "590".to_string();
            instance.y = "40".to_string();
            self.session.select_instance(part_index, 0);
        }
    }

    fn apply_setup_result(&mut self, result: Result<String, String>) {
        match result {
            Ok(status) => {
                self.session.dirty = true;
                self.document = None;
                self.state.gcode.clear();
                self.refresh_placed_scene();
                self.state.status = status;
            }
            Err(err) => self.state.status = err,
        }
    }

    fn toggle_layout_tool(&mut self, tool: LayoutTool) {
        self.layout_tool = if self.layout_tool == Some(tool) { None } else { Some(tool) };
    }

    fn layout_tool_windows(&mut self, ctx: &egui::Context) {
        let Some(tool) = self.layout_tool else {
            return;
        };
        let mut open = true;
        let title = match tool {
            LayoutTool::Array => "Array",
            LayoutTool::Arrange => "Arrange",
            LayoutTool::Rotate => "Rotate",
        };
        egui::Window::new(title)
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::RIGHT_TOP, [-16.0, 120.0])
            .default_pos(egui::pos2(1180.0, 130.0))
            .show(ctx, |ui| match tool {
                LayoutTool::Array => {
                    ui.label(RichText::new("Linked copies of the selected instance. Fields start empty.").color(theme::MUTED).size(11.0));
                    field_row(ui, "Columns", &mut self.state.array_x, self.state.dark_theme);
                    field_row(ui, "Rows", &mut self.state.array_y, self.state.dark_theme);
                    field_row(ui, "Spacing X (mm)", &mut self.state.array_spacing_mm, self.state.dark_theme);
                    field_row(ui, "Spacing Y (mm)", &mut self.state.array_spacing_y_mm, self.state.dark_theme);
                    ui.add_space(6.0);
                    if ui.button("Apply array").clicked() {
                        self.run_array();
                    }
                }
                LayoutTool::Arrange => {
                    ui.label(
                        RichText::new("Packs selected instances, or every instance when fewer than two are selected. An empty gap is 0 mm.")
                            .color(theme::MUTED)
                            .size(11.0),
                    );
                    field_row(ui, "Gap (mm)", &mut self.state.arrange_gap_mm, self.state.dark_theme);
                    ui.add_space(6.0);
                    if ui.button("Apply arrange").clicked() {
                        self.run_arrange();
                    }
                }
                LayoutTool::Rotate => {
                    ui.label("Rotate by (deg, CCW)");
                    ui.add(egui::TextEdit::singleline(&mut self.session.rotate_by).desired_width(180.0).hint_text("enter an angle"));
                    ui.add_space(6.0);
                    if ui.button("Apply rotate").clicked() {
                        self.run_rotate();
                    }
                }
            });
        if !open {
            self.layout_tool = None;
        }
    }

    fn run_array(&mut self) {
        match ArraySpec::parse(
            &self.state.array_x.value,
            &self.state.array_y.value,
            &self.state.array_spacing_mm.value,
            if self.state.array_spacing_y_mm.value.trim().is_empty() {
                &self.state.array_spacing_mm.value
            } else {
                &self.state.array_spacing_y_mm.value
            },
        ) {
            Ok(spec) => {
                let result = setup::array_copies(&mut self.session, spec);
                self.apply_setup_result(result);
            }
            Err(err) => self.state.status = err,
        }
    }

    fn run_arrange(&mut self) {
        let guides = self.viewport_guides();
        let width = guides.sheet.or(guides.bed).map(|b| b[2] - b[0]);
        let gap = self.state.arrange_gap_mm.value.clone();
        let result = setup::arrange(&mut self.session, &gap, width);
        self.apply_setup_result(result);
    }

    fn run_align(&mut self, mode: AlignMode) {
        let guides = self.viewport_guides();
        let frame = guides.sheet.or(guides.bed);
        let result = setup::align(&mut self.session, mode, frame);
        self.apply_setup_result(result);
    }

    fn run_rotate(&mut self) {
        let degrees = self.session.rotate_by.clone();
        let result = setup::rotate_targets(&mut self.session, &degrees);
        self.apply_setup_result(result);
    }

    fn handle_keys(&mut self, ctx: &egui::Context) {
        let typing = ctx.egui_wants_keyboard_input();
        let mut quit = false;
        let mut new_job = false;
        let mut open = false;
        let mut save = false;
        let mut export = false;
        let mut next = false;
        let mut prev = false;
        let mut arrange = false;
        let mut layer_up = false;
        let mut layer_down = false;
        let mut copy = false;
        let mut paste = false;
        let mut duplicate = false;
        let mut delete = false;
        let mut undo = false;
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
            if cmd && input.key_pressed(egui::Key::S) {
                save = true;
            }
            if cmd && input.key_pressed(egui::Key::E) {
                export = true;
            }
            if cmd && input.key_pressed(egui::Key::C) && !typing {
                copy = true;
            }
            if cmd && input.key_pressed(egui::Key::V) && !typing {
                paste = true;
            }
            if cmd && input.key_pressed(egui::Key::D) && !typing {
                duplicate = true;
            }
            if cmd && input.key_pressed(egui::Key::Z) && !typing {
                undo = true;
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
                if input.key_pressed(egui::Key::Delete) || input.key_pressed(egui::Key::Backspace) {
                    delete = true;
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
            self.request_open();
        }
        if save {
            self.save_job();
        }
        if export {
            self.request_export();
        }
        if copy {
            self.copy_selection();
        }
        if paste {
            self.paste_selection();
        }
        if duplicate {
            self.duplicate_selection();
        }
        if delete {
            self.delete_selection();
        }
        if undo {
            self.undo_last();
        }
        if next {
            self.state.next_tab();
        }
        if prev {
            self.state.prev_tab();
        }
        if arrange {
            if self.state.tab == ModeTab::Setup {
                self.run_arrange();
            } else if self.state.workspace == Workspace::Resin && self.state.tab == ModeTab::Orient {
                self.state.status = "Arrange is a placeholder in this preview".to_string();
            }
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
        if self.state.run == RunPhase::Running && self.run_conn.is_none() {
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
        self.preview_banner(ui);
        self.status_bar(ui);
        if self.state.tab != ModeTab::Slice && self.state.tab != ModeTab::Export && self.state.show_timeline {
            self.timeline(ui);
        }
        self.body(ui);
        self.layout_tool_windows(&ctx);
        self.dialogs(&ctx);
        self.tick_regen(&ctx);
        self.tick_stream(&ctx);
        self.tick_screenshots(&ctx);
    }
}

impl App {
    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        egui::Panel::top("menu").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(28.0);
            egui::MenuBar::new().ui(ui, |ui| {
                ui.add_space(4.0);
                draw_wordmark(ui);
                ui.add_space(10.0);
                ui.menu_button("File", |ui| {
                    if ui.button("New").clicked() {
                        self.apply_document_new();
                        ui.close();
                    }
                    if ui.button("Open…").clicked() {
                        self.request_open();
                        ui.close();
                    }
                    if ui.button("Save").clicked() {
                        self.save_job();
                        ui.close();
                    }
                    if ui.button("Save As…").clicked() {
                        self.request_save_as();
                        ui.close();
                    }
                    ui.separator();
                    if ui.button("Import part…").clicked() {
                        self.request_import();
                        ui.close();
                    }
                    if ui.button("Export G-code…").clicked() {
                        self.request_export();
                        ui.close();
                    }
                    if ui.button("Enter path…").clicked() {
                        self.file_dialog = Some(FileDialogKind::Open);
                        self.path_text.clear();
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
                    if ui.add_enabled(!self.undo.is_empty(), egui::Button::new("Undo")).clicked() {
                        self.undo_last();
                        ui.close();
                    }
                    ui.add_enabled(false, egui::Button::new("Redo"));
                    ui.separator();
                    if ui.button("Copy").clicked() {
                        self.copy_selection();
                        ui.close();
                    }
                    if ui.button("Paste").clicked() {
                        self.paste_selection();
                        ui.close();
                    }
                    if ui.button("Duplicate").clicked() {
                        self.duplicate_selection();
                        ui.close();
                    }
                    if ui.button("Delete").clicked() {
                        self.delete_selection();
                        ui.close();
                    }
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
                        if let Some(viewport) = &mut self.viewport {
                            viewport.fitted = false;
                            viewport.user_adjusted = false;
                            viewport.last_size = None;
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
                    if ui.button("Machines…").clicked() {
                        self.open_machines_manager();
                        ui.close();
                    }
                    if ui.button("Material library…").clicked() {
                        self.open_machines_manager();
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
        // Machine selector on the left, CAM stage tabs beside it, grouped commands under the
        // tabs, pinned STOP on the right.
        egui::Panel::top("toolbar").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(RIBBON_HEIGHT);
            ui.horizontal(|ui| {
                ui.add_space(4.0);
                self.machine_selector(ui);
                ui.add_space(6.0);
                ui.separator();
                ui.add_space(6.0);
                ui.vertical(|ui| {
                    ui.set_height(RIBBON_HEIGHT - 4.0);
                    ui.horizontal(|ui| {
                        ui.set_height(RIBBON_TAB_ROW);
                        self.stage_tabs(ui);
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            ui.add_space(8.0);
                            self.stop_button(ui);
                        });
                    });
                    ui.add_space(2.0);
                    ui.separator();
                    ui.add_space(2.0);
                    ui.horizontal(|ui| {
                        ui.set_height(RIBBON_GROUP_ROW);
                        self.command_groups(ui);
                    });
                });
            });
        });
    }

    fn machine_selector(&mut self, ui: &mut egui::Ui) {
        let height = ui.available_height().max(RIBBON_HEIGHT - 8.0);
        let (rect, response) = ui.allocate_exact_size(Vec2::new(MACHINE_CARD_WIDTH, height), egui::Sense::click());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, CornerRadius::same(4), theme::WIDGET);
        let stroke = if response.hovered() { theme::ACCENT_HOVER } else { theme::BORDER };
        painter.rect_stroke(rect, CornerRadius::same(4), Stroke::new(1.0, stroke), egui::StrokeKind::Inside);
        let text_color = ui.visuals().text_color();
        let (name, typ) = self.machine_card_labels();
        let text_rect = rect.shrink2(Vec2::new(8.0, 8.0));
        let name_galley = painter.layout(name, FontId::proportional(12.0), text_color, text_rect.width());
        let type_galley = painter.layout(format!("{typ} ▾"), FontId::proportional(11.0), theme::MUTED, text_rect.width());
        let total_h = name_galley.size().y + 4.0 + type_galley.size().y;
        let name_pos = egui::pos2(text_rect.center().x - name_galley.size().x / 2.0, text_rect.center().y - total_h / 2.0);
        let type_pos = egui::pos2(text_rect.center().x - type_galley.size().x / 2.0, name_pos.y + name_galley.size().y + 4.0);
        painter.galley(name_pos, name_galley, text_color);
        painter.galley(type_pos, type_galley, theme::MUTED);
        if response.hovered() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
        }
        if response.clicked() {
            self.machine_menu_open = !self.machine_menu_open;
        }
        egui::Popup::from_toggle_button_response(&response).open(self.machine_menu_open).width(340.0).show(|ui| {
            ui.label(RichText::new("MACHINE").color(theme::MUTED).size(11.0).strong());
            ui.add_space(6.0);
            let listed = self.profile_store.list().ok().unwrap_or_default();
            if listed.is_empty() {
                ui.label(RichText::new("No machine profiles yet. Use Manage… to add one.").color(theme::MUTED).size(12.0));
            }
            for (id, _) in &listed {
                let (label, typ) = self.profile_card_labels(id);
                let selected = self.session.machine_profile == *id;
                let heading = if selected { format!("{label}  (current)") } else { label };
                if ui.selectable_label(selected, RichText::new(heading).strong().size(13.0)).clicked() {
                    self.select_machine_profile(id);
                    self.machine_menu_open = false;
                    ui.close();
                }
                ui.label(RichText::new(format!("{typ}  ·  {id}")).color(theme::MUTED).size(11.0));
                ui.add_space(6.0);
            }
            if ui.button("Manage…").clicked() {
                self.open_machines_manager();
                self.machine_menu_open = false;
                ui.close();
            }
        });
    }

    fn machine_card_labels(&self) -> (String, String) {
        if let Ok(Some(profile)) = machines::session_machine(&self.profile_store, &self.session)
            && profile.kind == campanion_cli::MachineKind::Resin
        {
            let name = if profile.name.is_empty() { self.session.machine_profile.clone() } else { profile.name };
            return (name, "Resin".to_string());
        }
        match machines::session_profile(&self.profile_store, &self.session) {
            Ok(Some((profile, head))) => {
                let typ = machines::process_from_id(&head.process)
                    .map(|(_, process)| process.label().to_string())
                    .filter(|label| !label.is_empty())
                    .unwrap_or_else(|| if head.process.is_empty() { "Machine".to_string() } else { head.process.clone() });
                let name = if profile.name.is_empty() { self.session.machine_profile.clone() } else { profile.name };
                (name, typ)
            }
            _ if !self.session.machine_profile.is_empty() => (self.session.machine_profile.clone(), "Select…".to_string()),
            _ => ("No machine".to_string(), "Select…".to_string()),
        }
    }

    fn profile_card_labels(&self, id: &str) -> (String, String) {
        match self.profile_store.load(id) {
            Ok(profile) => {
                if profile.kind == campanion_cli::MachineKind::Resin {
                    let name = if profile.name.is_empty() { id.to_string() } else { profile.name };
                    return (name, "Resin".to_string());
                }
                let typ = profile
                    .heads
                    .iter()
                    .find(|head| self.session.machine_head == head.id || self.session.machine_head.is_empty())
                    .and_then(|head| machines::process_from_id(&head.process).map(|(_, process)| process.label().to_string()))
                    .or_else(|| {
                        profile
                            .heads
                            .first()
                            .and_then(|head| machines::process_from_id(&head.process).map(|(_, process)| process.label().to_string()))
                    })
                    .unwrap_or_else(|| "Machine".to_string());
                let name = if profile.name.is_empty() { id.to_string() } else { profile.name };
                (name, typ)
            }
            Err(_) => (id.to_string(), "Machine".to_string()),
        }
    }

    fn select_machine_profile(&mut self, id: &str) {
        self.session.machine_profile = id.to_string();
        self.session.machine_path.clear();
        self.session.machine_head.clear();
        self.apply_machine_selection();
    }

    fn current_process_label(&self) -> Option<String> {
        machines::session_profile(&self.profile_store, &self.session)
            .ok()
            .flatten()
            .and_then(|(_, head)| machines::process_from_id(&head.process).map(|(_, process)| process.label().to_string()))
            .or_else(|| self.state.process.map(|process| process.label().to_string()))
    }

    fn stage_tabs(&mut self, ui: &mut egui::Ui) {
        let tabs: Vec<ModeTab> = self.state.workspace.tabs().to_vec();
        let current = self.state.tab;
        for tab in tabs {
            let selected = tab == current;
            let text = if selected {
                RichText::new(tab.label()).strong().size(14.0).color(Color32::WHITE)
            } else {
                RichText::new(tab.label()).size(14.0).color(theme::MUTED)
            };
            let button = if selected { egui::Button::new(text).fill(theme::ACCENT) } else { egui::Button::new(text).fill(Color32::TRANSPARENT) };
            if ui.add(button).clicked() {
                let _ = self.state.set_tab(tab);
            }
        }
    }

    fn stop_button(&mut self, ui: &mut egui::Ui) {
        let stop =
            egui::Button::new(RichText::new("STOP").strong().color(Color32::WHITE).size(15.0)).fill(theme::STOP).min_size(Vec2::new(88.0, 30.0));
        let response = ui.add_enabled(self.state.can_stop(), stop);
        if response.clicked() {
            self.stream_stop();
        }
        if !self.state.can_stop() {
            response.on_hover_text("STOP is live once a controller is connected.");
        }
    }

    fn command_groups(&mut self, ui: &mut egui::Ui) {
        match self.state.tab {
            ModeTab::Process => {
                ribbon_group(ui, "Machine", |ui| {
                    let text =
                        self.current_process_label().map(|label| format!("Process: {label}")).unwrap_or_else(|| "Select a machine".to_string());
                    ui.vertical(|ui| {
                        ui.add_space(6.0);
                        ui.label(RichText::new(text).size(12.0));
                        ui.label(RichText::new("Process comes from the selected machine head.").color(theme::MUTED).size(10.0));
                    });
                });
            }
            ModeTab::Setup => {
                ribbon_group(ui, "Source", |ui| {
                    if ribbon_command(ui, "Import part", false).clicked() {
                        self.request_import();
                    }
                    if ribbon_command(ui, "Open job", false).clicked() {
                        self.request_open();
                    }
                });
                ribbon_group_separator(ui);
                ribbon_group(ui, "Layout", |ui| {
                    if ribbon_command(ui, "Linked copy", false).clicked() {
                        let result = setup::linked_copy(&mut self.session);
                        self.apply_setup_result(result);
                    }
                    if ribbon_command(ui, "Array", self.layout_tool == Some(LayoutTool::Array)).clicked() {
                        self.toggle_layout_tool(LayoutTool::Array);
                    }
                    if ribbon_command(ui, "Arrange", self.layout_tool == Some(LayoutTool::Arrange)).clicked() {
                        self.toggle_layout_tool(LayoutTool::Arrange);
                    }
                    if ribbon_command(ui, "Align left", false).clicked() {
                        self.run_align(AlignMode::Left);
                    }
                    if ribbon_command(ui, "Align right", false).clicked() {
                        self.run_align(AlignMode::Right);
                    }
                    if ribbon_command(ui, "Mirror", false).clicked() {
                        let result = setup::mirror_targets(&mut self.session);
                        self.apply_setup_result(result);
                    }
                    if ribbon_command(ui, "Rotate", self.layout_tool == Some(LayoutTool::Rotate)).clicked() {
                        self.toggle_layout_tool(LayoutTool::Rotate);
                    }
                    if ribbon_command(ui, "Place at origin", false).clicked() {
                        let result = setup::place_at_origin(&mut self.session);
                        self.apply_setup_result(result);
                    }
                });
                ribbon_group_separator(ui);
                ribbon_group(ui, "Machine", |ui| {
                    if ribbon_command(ui, "Machines…", false).clicked() {
                        self.open_machines_manager();
                    }
                });
            }
            ModeTab::Operations => {
                ribbon_group(ui, "Tools", |ui| {
                    if ribbon_command(ui, "Add tool", false).clicked() {
                        self.add_session_tool();
                    }
                });
                ribbon_group_separator(ui);
                ribbon_group(ui, "Operations", |ui| {
                    self.add_operation_menu(ui);
                });
                ribbon_group_separator(ui);
                ribbon_group(ui, "Order", |ui| {
                    if ribbon_command(ui, "Reset order", false).clicked() {
                        operations::reset_cut_order(&mut self.session);
                        self.schedule_regen();
                    }
                });
            }
            ModeTab::Simulate => self.simulate_ribbon(ui),
            ModeTab::Orient => {
                ribbon_group(ui, "Orient", |ui| {
                    if ribbon_command(ui, "Lay flat on face", false).clicked() {
                        self.state.status = "Lay flat is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "Auto orient", false).clicked() {
                        self.state.status = "Auto orient is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "Reset", false).clicked() {
                        self.state.reset_orient();
                    }
                });
            }
            ModeTab::Supports => {
                ribbon_group(ui, "Generate", |ui| {
                    if ribbon_command(ui, "Auto Generate", false).clicked() {
                        self.state.status = "Auto Generate is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "Scaffold", false).clicked() {
                        self.state.status = "Scaffold generation is a placeholder in this preview".to_string();
                    }
                });
                ribbon_group_separator(ui);
                ribbon_group(ui, "Supports", |ui| {
                    if ribbon_command(ui, "Add Support", false).clicked() {
                        self.state.status = "Add Support is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "Delete", false).clicked() {
                        self.state.status = "Delete is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "Select Region", false).clicked() {
                        self.state.status = "Select Region is a placeholder in this preview".to_string();
                    }
                    if ribbon_command(ui, "No-Support Zone", false).clicked() {
                        self.state.status = "No-support zones are a placeholder in this preview".to_string();
                    }
                });
            }
            ModeTab::Slice => {
                ribbon_group(ui, "Layers", |ui| {
                    if ribbon_command(ui, "Layer up", false).clicked() {
                        let _ = self.state.step_layer(1);
                    }
                    if ribbon_command(ui, "Layer down", false).clicked() {
                        let _ = self.state.step_layer(-1);
                    }
                });
            }
            ModeTab::Export => {
                if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
                    ribbon_group(ui, "Export", |ui| {
                        if ribbon_command(ui, "Export File", false).clicked() {
                            self.export_gcode();
                        }
                    });
                } else {
                    ribbon_group(ui, "Preview", |ui| {
                        if ribbon_command(ui, "Preview G-code", false).clicked() {
                            self.preview_gcode();
                        }
                    });
                    ribbon_group_separator(ui);
                    ribbon_group(ui, "Export", |ui| {
                        if ribbon_command(ui, "Export File", false).clicked() {
                            self.export_gcode();
                        }
                    });
                }
            }
            ModeTab::Run => {
                ribbon_group(ui, "Run", |ui| {
                    ui.add_enabled_ui(self.state.can_start(), |ui| {
                        if ribbon_command(ui, "Start", false).clicked() {
                            self.stream_start();
                        }
                    });
                    ui.add_enabled_ui(self.state.can_pause() || self.state.run == RunPhase::Paused, |ui| {
                        let label = if self.state.run == RunPhase::Paused { "Resume" } else { "Pause" };
                        if ribbon_command(ui, label, false).clicked() {
                            if self.state.run == RunPhase::Paused {
                                self.stream_resume();
                            } else {
                                self.stream_pause();
                            }
                        }
                    });
                });
            }
        }
    }

    fn add_session_tool(&mut self) {
        let kind = default_tool_kind(self.state.process, self.state.workspace);
        let id = crate::session::next_tool_id(&self.session.tools);
        self.session.tools.push(ToolDraft { id, name: String::new(), kind, size: String::new() });
        self.session.selected = Some(TreeSelection::Tool(self.session.tools.len() - 1));
        self.mark_dirty();
    }

    fn current_process_id(&self) -> Option<&str> {
        match self.state.process {
            Some(SubProcess::Laser) => Some("laser"),
            Some(SubProcess::Plasma) => Some("plasma"),
            Some(SubProcess::Waterjet) => Some("waterjet"),
            Some(SubProcess::OxyFuel) => Some("oxyfuel"),
            Some(SubProcess::WireEdm) => Some("wire_edm"),
            Some(SubProcess::Milling) => Some("milling"),
            Some(SubProcess::Routing) => Some("routing"),
            Some(SubProcess::Turning) => Some("turning"),
            Some(SubProcess::Sla | SubProcess::Fdm) | None => None,
        }
    }

    fn refresh_gcode_preview(&mut self) {
        if self.session.post_id.trim().is_empty() || self.document.is_none() {
            self.play_index = None;
            return;
        }
        self.preview_gcode();
    }

    fn rebuild_play_index(&mut self) {
        let (Some(scene), post) = (self.scene.as_ref(), self.session.post_id.trim()) else {
            self.play_index = None;
            return;
        };
        if post.is_empty() || self.state.gcode.is_empty() {
            self.play_index = None;
            return;
        }
        match play_index(scene, &self.state.gcode, post) {
            Ok(index) => self.play_index = Some(index),
            Err(_) => self.play_index = None,
        }
        self.apply_preview_speed();
    }

    fn apply_preview_speed(&mut self) {
        let Some(viewport) = self.viewport.as_mut() else {
            return;
        };
        let total = self.scene.as_ref().map(|scene| scene.total_length).unwrap_or(0.0);
        let base = (total / 10.0).max(1.0);
        viewport.playback.speed = base * f64::from(self.preview_speed_mult);
    }

    fn seek_playhead_fraction(&mut self, fraction: f64) {
        let Some(total) = self.scene.as_ref().map(|scene| scene.total_length) else {
            return;
        };
        if let Some(viewport) = self.viewport.as_mut() {
            viewport.playback.playing = false;
            viewport.playback.position = (total * fraction).clamp(0.0, total);
        }
    }

    fn seek_gcode_line(&mut self, line: usize) {
        let Some(position) = self.play_index.as_ref().and_then(|index| index.position_for_line(line)) else {
            return;
        };
        if let Some(viewport) = self.viewport.as_mut() {
            viewport.playback.playing = false;
            viewport.playback.position = position;
        }
    }

    fn current_gcode_line(&self) -> Option<usize> {
        let position = self.viewport.as_ref()?.playback.position;
        self.play_index.as_ref()?.line_at(position)
    }

    fn simulate_ribbon(&mut self, ui: &mut egui::Ui) {
        ribbon_group(ui, "Playback", |ui| {
            let playing = self.viewport.as_ref().is_some_and(|viewport| viewport.playback.playing);
            let label = if playing { "Pause" } else { "Play" };
            if ribbon_command(ui, label, playing).clicked() {
                let total = self.scene.as_ref().map(|scene| scene.total_length).unwrap_or(0.0);
                if let Some(viewport) = self.viewport.as_mut() {
                    if playing {
                        viewport.playback.playing = false;
                    } else {
                        if viewport.playback.position >= total {
                            viewport.playback.restart();
                        } else {
                            viewport.playback.playing = true;
                        }
                    }
                }
            }
            if ribbon_command(ui, "Restart", false).clicked()
                && let Some(viewport) = self.viewport.as_mut()
            {
                viewport.playback.restart();
            }
        });
        ribbon_group_separator(ui);
        ribbon_group(ui, "Preview speed", |ui| {
            ui.horizontal(|ui| {
                for speed in [0.25_f32, 0.5, 1.0, 2.0, 4.0] {
                    let selected = (self.preview_speed_mult - speed).abs() < 1e-6;
                    let label = if (speed - 1.0).abs() < 1e-6 { "1x".to_string() } else { format!("{speed}x") };
                    if ui.selectable_label(selected, label).clicked() {
                        self.preview_speed_mult = speed;
                        self.apply_preview_speed();
                    }
                }
            });
            ui.label(RichText::new("Preview only, not a machine rate.").color(theme::MUTED).size(10.0));
        });
        ribbon_group_separator(ui);
        ribbon_group(ui, "G-code", |ui| {
            if ribbon_command(ui, "Preview G-code", false).clicked() {
                self.preview_gcode();
            }
        });
    }

    fn simulate_right(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Simulate").strong().size(14.0));
        ui.add_space(4.0);
        if self.regen.stale {
            ui.label(RichText::new("Toolpaths are stale until regenerate finishes.").color(theme::OVERRIDE).size(12.0));
            ui.add_space(4.0);
        }
        if let Some(document) = self.document.as_ref() {
            ui.label(RichText::new(estimate_time(document).label()).size(12.0));
        } else {
            ui.label(RichText::new("Generate toolpaths to estimate time from entered feeds.").color(theme::MUTED).size(12.0));
        }
        ui.label(RichText::new("Cuts are cyan; rapids are grey. Time uses only feeds you entered.").color(theme::MUTED).size(11.0));
        ui.add_space(8.0);
        self.show_preflight_checks(ui);
        ui.add_space(8.0);
        ui.label(RichText::new("G-code").strong().size(13.0));
        ui.label(RichText::new("The current line follows the playhead. Click a line to jump.").color(theme::MUTED).size(11.0));
        ui.add_space(4.0);
        self.show_gcode_listing(ui);
    }

    fn show_preflight_checks(&mut self, ui: &mut egui::Ui) {
        let checks = session_checks(&self.session, self.document.as_ref(), Some(&self.profile_store), self.current_process_id());
        ui.label(RichText::new("Pre-flight checks").strong().size(13.0));
        if checks.is_empty() {
            ui.label(RichText::new("No issues. Export is allowed.").color(theme::START).size(12.0));
            return;
        }
        for item in &checks {
            let color = if item.blocks_export() { theme::OFF_BED } else { theme::OFF_SHEET };
            let prefix = if item.blocks_export() { "Error" } else { "Warning" };
            ui.label(RichText::new(format!("{prefix}: {}", item.message)).color(color).size(12.0));
        }
    }

    fn show_gcode_listing(&mut self, ui: &mut egui::Ui) {
        if self.state.gcode.is_empty() {
            ui.label(RichText::new("Preview G-code to link the listing to the playhead.").color(theme::MUTED).size(12.0));
            return;
        }
        let lines: Vec<String> =
            if let Some(index) = &self.play_index { index.lines.clone() } else { self.state.gcode.lines().map(str::to_string).collect() };
        let current = self.current_gcode_line();
        let mut clicked = None;
        egui::ScrollArea::vertical().id_salt("simulate-gcode").auto_shrink([false, false]).max_height(420.0).show(ui, |ui| {
            for (index, line) in lines.iter().enumerate() {
                let is_current = current == Some(index);
                let shown = if line.is_empty() { " ".to_string() } else { line.clone() };
                let text = format!("{:>4}  {shown}", index + 1);
                let mut rich = RichText::new(text).monospace().size(11.0);
                let rapid = self.play_index.as_ref().and_then(|play| play.spans.iter().find(|span| span.line == index).map(|span| span.rapid));
                if is_current {
                    rich = rich.color(Color32::WHITE).background_color(theme::ACCENT);
                } else if rapid == Some(true) {
                    rich = rich.color(theme::RAPID);
                } else if rapid == Some(false) {
                    rich = rich.color(theme::CUT);
                }
                let response = ui.add(egui::Label::new(rich).sense(egui::Sense::click()));
                if is_current {
                    response.scroll_to_me(Some(Align::Center));
                }
                if response.clicked() {
                    clicked = Some(index);
                }
            }
        });
        if let Some(line) = clicked {
            self.seek_gcode_line(line);
        }
    }

    fn add_session_operation(&mut self, kind: &str) {
        if !operations::kind_allowed(self.state.process, kind) {
            self.state.status = format!("`{kind}` is not a valid operation for the selected head");
            return;
        }
        let tool_id = self.session.tools.first().map(|tool| tool.id).unwrap_or(1);
        let stem = if kind == "profile" { "profile" } else { kind };
        let name = crate::session::next_operation_name(&self.session.operations, stem);
        self.session.operations.push(operations::new_operation_on_session(&self.session, name, kind, tool_id));
        self.session.selected = Some(TreeSelection::Operation(self.session.operations.len() - 1));
        self.mark_dirty();
    }

    fn add_operation_menu(&mut self, ui: &mut egui::Ui) {
        let kinds = operations::kinds_for_process(self.state.process);
        let mut chosen = None;
        egui::ComboBox::from_id_salt("add-operation").selected_text("Add Operation").show_ui(ui, |ui| {
            for (kind, label) in kinds {
                if ui.selectable_label(false, *label).clicked() {
                    chosen = Some(*kind);
                }
            }
        });
        if let Some(kind) = chosen {
            self.add_session_operation(kind);
        }
    }

    fn preview_banner(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("preview-banner").frame(bar_frame()).show(ui, |ui| {
            ui.set_height(22.0);
            ui.horizontal_centered(|ui| {
                ui.add_space(8.0);
                ui.label(RichText::new(GUI_PREVIEW_NOTICE).color(theme::OVERRIDE).size(12.0));
                ui.separator();
                ui.label(RichText::new(STREAM_HARDWARE_NOTICE).color(theme::OVERRIDE).size(12.0));
            });
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
                    ui.separator();
                    ui.label(RichText::new(GUI_PREVIEW_NOTICE_SHORT).color(theme::OVERRIDE).size(11.0));
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
        let show_tree =
            self.state.show_tree && self.state.tab != ModeTab::Run && self.state.tab != ModeTab::Slice && self.state.tab != ModeTab::Simulate;
        let show_props = self.state.show_properties && self.state.tab != ModeTab::Slice;
        if show_tree {
            egui::Panel::left("tree").resizable(true).default_size(240.0).frame(panel_frame()).show(ui, |ui| {
                self.process_tree(ui);
            });
        }
        if show_props {
            let props_width = if self.state.tab == ModeTab::Simulate { 340.0 } else { 280.0 };
            egui::Panel::right("props").resizable(true).default_size(props_width).frame(panel_frame()).show(ui, |ui| {
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| match self.state.tab {
                    ModeTab::Run => self.run_right(ui),
                    ModeTab::Export => self.export_form(ui),
                    ModeTab::Simulate => self.simulate_right(ui),
                    ModeTab::Orient => self.orient_properties(ui),
                    ModeTab::Supports => self.supports_properties(ui),
                    _ => self.default_properties(ui),
                });
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
            ModeTab::Simulate => self.viewport_panel(ui, "Simulate"),
            ModeTab::Supports => self.supports_center(ui),
            ModeTab::Orient if self.state.workspace == Workspace::Resin || self.state.workspace == Workspace::Fdm => self.orient_center(ui),
            ModeTab::Run => self.viewport_panel(ui, "Simulation / Run View"),
            _ => self.viewport_panel(ui, "Viewport"),
        });
    }

    fn open_machines_manager(&mut self) {
        match MachinesUi::open_user() {
            Ok(ui) => self.machines = Some(ui),
            Err(err) => self.state.status = err,
        }
    }

    fn apply_machine_selection(&mut self) {
        if let Ok(Some(profile)) = machines::session_machine(&self.profile_store, &self.session)
            && profile.kind == campanion_cli::MachineKind::Resin
        {
            self.state.set_workspace(Workspace::Resin);
            let _ = self.state.set_process(SubProcess::Sla);
            self.session.post_id.clear();
            self.state.export_name.clear();
            self.session.dirty = true;
            self.state.status = format!("Machine: {} (Resin)", profile.name);
            self.refresh_placed_scene();
            return;
        }
        match machines::session_profile(&self.profile_store, &self.session) {
            Ok(Some((profile, head))) => {
                if let Some((workspace, process)) = machines::process_from_id(&head.process) {
                    self.state.set_workspace(workspace);
                    let _ = self.state.set_process(process);
                }
                if self.session.post_id.is_empty()
                    && let Some(post) = campanion_cli::effective_post(&profile, &head)
                {
                    self.session.post_id = post;
                }
                self.session.dirty = true;
                let process = machines::process_from_id(&head.process).map(|(_, process)| process.label()).unwrap_or(head.process.as_str());
                self.state.status = format!("Machine: {} ({process})", profile.name);
                self.refresh_placed_scene();
            }
            Ok(None) => {}
            Err(err) => self.state.status = err,
        }
    }

    fn machine_job_picker(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Machine").strong().size(13.0));
        let listed = self.profile_store.list().ok().unwrap_or_default();
        ui.horizontal(|ui| {
            let current = if self.session.machine_profile.is_empty() { "Not selected".to_string() } else { self.session.machine_profile.clone() };
            ui.label("Profile");
            egui::ComboBox::from_id_salt("job-machine-profile").selected_text(current).show_ui(ui, |ui| {
                if ui.selectable_label(self.session.machine_profile.is_empty(), "Not selected").clicked() {
                    self.session.machine_profile.clear();
                    self.session.machine_head.clear();
                    self.session.dirty = true;
                }
                for (id, _) in &listed {
                    if ui.selectable_label(self.session.machine_profile == *id, id).clicked() {
                        self.session.machine_profile = id.clone();
                        self.session.machine_path.clear();
                        self.session.machine_head.clear();
                        self.apply_machine_selection();
                    }
                }
            });
            if ui.button("Manage…").clicked() {
                self.open_machines_manager();
            }
        });
        if !self.session.machine_profile.is_empty() {
            match machines::session_profile(&self.profile_store, &self.session) {
                Ok(Some((profile, selected))) => {
                    let head_ids: Vec<String> = profile.heads.iter().map(|head| head.id.clone()).collect();
                    let selected_id = selected.id.clone();
                    let materials = profile.heads.iter().find(|head| head.id == selected_id).map(|head| head.materials.clone()).unwrap_or_default();
                    ui.horizontal(|ui| {
                        ui.label("Head");
                        let current_head = if self.session.machine_head.is_empty() { selected_id.clone() } else { self.session.machine_head.clone() };
                        egui::ComboBox::from_id_salt("job-machine-head").selected_text(current_head).show_ui(ui, |ui| {
                            for id in &head_ids {
                                let is_sel = self.session.machine_head == *id || (self.session.machine_head.is_empty() && *id == selected_id);
                                if ui.selectable_label(is_sel, id).clicked() {
                                    self.session.machine_head = id.clone();
                                    self.apply_machine_selection();
                                }
                            }
                        });
                    });
                    if !materials.is_empty() {
                        ui.label("Material");
                        for entry in materials {
                            if ui.button(&entry.name).clicked() {
                                match machines::apply_material(&mut self.session, &entry) {
                                    Ok(status) => self.state.status = status,
                                    Err(err) => self.state.status = err,
                                }
                            }
                        }
                    } else {
                        ui.label(RichText::new("No materials in this head. Add them in Machine → Machines…").color(theme::MUTED).size(11.0));
                    }
                }
                Ok(None) => {}
                Err(err) => {
                    ui.label(RichText::new(err).color(theme::MUTED).size(11.0));
                }
            }
        }
        ui.label(RichText::new("Material values stay empty until you enter them.").color(theme::MUTED).size(11.0));
    }

    fn tree_context_menu(&mut self, response: egui::Response) {
        response.context_menu(|ui| {
            if ui.button("Copy").clicked() {
                self.copy_selection();
                ui.close();
            }
            if ui.button("Paste").clicked() {
                self.paste_selection();
                ui.close();
            }
            if ui.button("Duplicate").clicked() {
                self.duplicate_selection();
                ui.close();
            }
            if ui.button("Delete").clicked() {
                self.delete_selection();
                ui.close();
            }
        });
    }

    fn process_tree(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Process Tree").strong().size(14.0));
        ui.add_space(6.0);
        self.machine_job_picker(ui);
        if matches!(self.state.workspace, Workspace::Resin | Workspace::Fdm) {
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
        ui.separator();
        let parts_open = Some(self.parts_tree_open);
        let parts_header = egui::CollapsingHeader::new("Parts").id_salt("parts-tree").default_open(true).open(parts_open).show(ui, |ui| {
            if self.session.parts.is_empty() {
                ui.label(RichText::new("Import a part on the Setup tab").color(theme::MUTED).size(12.0));
            }
            let part_count = self.session.parts.len();
            for part_index in 0..part_count {
                let id = self.session.parts[part_index].id.clone();
                let inst_count = self.session.parts[part_index].instances.len();
                let selected = self.session.selected == Some(TreeSelection::Part(part_index));
                if ui.selectable_label(selected, format!("{id} ({inst_count})")).clicked() {
                    self.session.selected = Some(TreeSelection::Part(part_index));
                    self.session.sync_placement_from_selection();
                }
                for inst in 0..inst_count {
                    let inst_id = self.session.parts[part_index].instances[inst].id.clone();
                    let x = self.session.parts[part_index].instances[inst].x.clone();
                    let y = self.session.parts[part_index].instances[inst].y.clone();
                    let selected = self.session.selected == Some(TreeSelection::Instance(part_index, inst));
                    if ui.selectable_label(selected, format!("    {inst_id}  ({x}, {y})")).clicked() {
                        self.session.select_instance(part_index, inst);
                    }
                }
            }
        });
        if parts_header.header_response.clicked() {
            self.parts_tree_open = !self.parts_tree_open;
        }
        if let Some(label) = self.session.source_path.as_ref().map(|path| path.display().to_string()) {
            let selected = self.session.selected == Some(TreeSelection::Source);
            let response = ui.selectable_label(selected, format!("Source: {label}"));
            if response.clicked() {
                self.session.selected = Some(TreeSelection::Source);
                self.session.sync_placement_from_selection();
            }
            self.tree_context_menu(response);
        } else if self.session.parts.is_empty() {
            tree_line(ui, "Source", "Not imported");
        }
        ui.collapsing("Layers", |ui| {
            let layers = all_part_layers(&self.session);
            if layers.is_empty() {
                ui.label(RichText::new("Import a part to list layers").color(theme::MUTED).size(12.0));
            } else {
                for layer in layers {
                    let id = TreeSelection::Layer(layer.name.clone());
                    let selected = self.session.selected.as_ref() == Some(&id);
                    let label = format!("{} ({})", layer_parts_label(&self.session, &layer.name), layer.contour_count);
                    if ui.selectable_label(selected, label).clicked() {
                        self.session.selected = Some(id);
                    }
                }
            }
        });
        ui.collapsing("Tools", |ui| {
            let count = self.session.tools.len();
            for index in 0..count {
                let tool = &self.session.tools[index];
                let label =
                    if tool.name.is_empty() { format!("T{} {}", tool.id, tool_kind_label(tool.kind)) } else { format!("T{} {}", tool.id, tool.name) };
                let selected = self.session.selected == Some(TreeSelection::Tool(index));
                let response = ui.selectable_label(selected, label);
                if response.clicked() {
                    self.session.selected = Some(TreeSelection::Tool(index));
                }
                self.tree_context_menu(response);
            }
            if ui.button("Add tool").clicked() {
                self.add_session_tool();
            }
        });
        let ops_open = Some(self.operations_tree_open);
        let ops_header = egui::CollapsingHeader::new("Operations").id_salt("ops-tree").default_open(true).open(ops_open).show(ui, |ui| {
            let count = self.session.operations.len();
            for index in 0..count {
                let op = &self.session.operations[index];
                let name = if op.name.is_empty() { format!("Operation {}", index + 1) } else { op.name.clone() };
                let unmatched = operation_match_warning(&self.session, op);
                let selected = self.session.selected == Some(TreeSelection::Operation(index));
                let label = if unmatched.is_some() { RichText::new(format!("{name}  !")).color(theme::OVERRIDE) } else { RichText::new(name) };
                let response = ui.selectable_label(selected, label);
                if response.clicked() {
                    self.session.selected = Some(TreeSelection::Operation(index));
                }
                self.tree_context_menu(response);
                if let Some(warning) = unmatched {
                    ui.label(RichText::new(warning).color(theme::OVERRIDE).size(11.0));
                }
            }
            if self.session.operations.is_empty() {
                ui.label(RichText::new("No operations yet").color(theme::MUTED).size(12.0));
            }
            self.add_operation_menu(ui);
            self.cut_order_list(ui);
        });
        if ops_header.header_response.clicked() {
            self.operations_tree_open = !self.operations_tree_open;
        }
    }

    fn cut_order_list(&mut self, ui: &mut egui::Ui) {
        if self.session.operations.len() < 2 {
            return;
        }
        ui.add_space(6.0);
        ui.label(RichText::new("Cut order").strong().size(13.0));
        ui.checkbox(&mut self.session.order_rules_set, "Use automatic rules");
        if self.session.order_rules_set {
            if ui.checkbox(&mut self.session.order_inner_first, "Inner first").changed() {
                self.mark_dirty();
            }
            if ui.checkbox(&mut self.session.order_cuts_last, "Cuts last").changed() {
                self.mark_dirty();
            }
        }
        if self.session.cut_order_explicit {
            ui.label(RichText::new("Drag or use the arrows to override automatic order.").color(theme::OVERRIDE).size(11.0));
        } else {
            ui.label(RichText::new("Drag or use the arrows to set an explicit order.").color(theme::MUTED).size(11.0));
        }
        let count = self.session.operations.len();
        let mut reorder = None;
        for index in 0..count {
            let id = ui.scope_id().with("cut-order").with(index);
            let name = if self.session.operations[index].name.is_empty() {
                format!("Operation {}", index + 1)
            } else {
                self.session.operations[index].name.clone()
            };
            let mut move_up = false;
            let mut move_down = false;
            let response = ui
                .dnd_drag_source(id, index, |ui| {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        if ui.add_enabled(index > 0, egui::Button::new("↑").small()).clicked() {
                            move_up = true;
                        }
                        if ui.add_enabled(index + 1 < count, egui::Button::new("↓").small()).clicked() {
                            move_down = true;
                        }
                        ui.label("☰");
                        ui.add(egui::Label::new(&name).truncate());
                    });
                })
                .response;
            if move_up {
                reorder = Some((index, index - 1));
            } else if move_down {
                reorder = Some((index, index + 1));
            } else if ui.input(|input| input.pointer.any_released())
                && let Some(from) = response.dnd_hover_payload::<usize>()
                && *from != index
            {
                reorder = Some((*from, index));
            }
        }
        if let Some((from, to)) = reorder {
            operations::reorder_operation(&mut self.session, from, to);
            self.schedule_regen();
        }
        if ui.button("Reset to rules").clicked() {
            operations::reset_cut_order(&mut self.session);
            self.schedule_regen();
        }
    }

    fn default_properties(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Properties").strong().size(14.0));
        ui.add_space(6.0);
        self.placement_properties(ui);
        match self.session.selected.clone() {
            Some(TreeSelection::Tool(index)) if index < self.session.tools.len() => {
                self.tool_properties(ui, index);
            }
            Some(TreeSelection::Operation(index)) if index < self.session.operations.len() => {
                self.operation_properties(ui, index);
            }
            Some(TreeSelection::Source) => {
                ui.label(RichText::new("Imported part").strong().size(13.0));
                ui.label(RichText::new("Delete (Edit → Delete, or the Delete key) removes the part from this job. Tools and operations are kept. Ctrl+Z undoes the removal.").color(theme::MUTED).size(12.0));
            }
            Some(TreeSelection::Layer(name)) => {
                ui.label(format!("Layer: {name}"));
                ui.label(RichText::new("Assign this layer to a profile in the operation properties.").color(theme::MUTED).size(12.0));
                ui.label(
                    RichText::new("Overridden material fields are shown in orange. Reset restores the last applied preset.")
                        .color(theme::MUTED)
                        .size(11.0),
                );
            }
            _ => {
                if self.state.tab != ModeTab::Setup {
                    ui.label(RichText::new("Select a tool, an operation, or the imported part in the process tree.").color(theme::MUTED).size(12.0));
                }
            }
        }
        self.show_messages(ui);
    }

    fn placement_properties(&mut self, ui: &mut egui::Ui) {
        if self.session.source_drawing.is_none() && self.session.parts.is_empty() {
            if self.state.tab == ModeTab::Setup {
                ui.label(RichText::new("Import a part on the Setup tab to edit its placement.").color(theme::MUTED).size(12.0));
                ui.separator();
                self.job_section(ui);
            }
            return;
        }
        let selected = match self.session.selected {
            Some(TreeSelection::Instance(part, inst)) => format!("Instance {}.{}", part + 1, inst + 1),
            Some(TreeSelection::Part(part)) => {
                self.session.parts.get(part).map(|part| format!("Part `{}`", part.id)).unwrap_or_else(|| "Part".to_string())
            }
            _ => self.session.parts.first().map(|part| format!("Part `{}`", part.id)).unwrap_or_else(|| "Selected part".to_string()),
        };
        ui.label(RichText::new("Placement").strong().size(13.0));
        ui.label(RichText::new(selected).color(theme::MUTED).size(11.0));
        let mut changed = false;
        changed |= self.placement_field(ui, "X (mm)", 0);
        changed |= self.placement_field(ui, "Y (mm)", 1);
        changed |= self.placement_field(ui, "Rotation (deg, CCW)", 2);
        ui.label("Scale");
        if ui.add(egui::TextEdit::singleline(&mut self.session.placement_scale).desired_width(180.0).hint_text("1")).changed() {
            changed = true;
        }
        if ui.checkbox(&mut self.session.placement_mirror, "Mirror").changed() {
            changed = true;
        }
        ui.label(
            RichText::new("Drag the part in the viewport to move it. Shift-drag, Alt-drag, or middle-drag pans the view.")
                .color(theme::MUTED)
                .size(11.0),
        );
        if changed {
            self.session.commit_placement();
            self.session.dirty = true;
            self.document = None;
            self.state.gcode.clear();
            self.refresh_placed_scene();
        }
        if self.state.tab == ModeTab::Setup {
            ui.separator();
            self.job_section(ui);
        }
    }

    fn job_section(&mut self, ui: &mut egui::Ui) {
        let response =
            egui::CollapsingHeader::new("Job").id_salt("job-section").default_open(false).open(Some(self.job_section_open)).show(ui, |ui| {
                ui.label(RichText::new("Origin").strong().size(13.0));
                ui.label("Z-zero");
                ui.horizontal(|ui| {
                    for (value, label) in [("", "unset"), ("stock_top", "stock top"), ("bed", "bed")] {
                        if ui.selectable_label(self.session.z_zero == value, label).clicked() {
                            self.session.z_zero = value.to_string();
                            self.session.dirty = true;
                        }
                    }
                });
                if self.session.z_zero == "bed" {
                    ui.label("Stock thickness (mm)");
                    if ui.add(egui::TextEdit::singleline(&mut self.session.stock_thickness).desired_width(180.0).hint_text("required")).changed() {
                        self.session.dirty = true;
                    }
                }
                ui.label("XY datum");
                ui.horizontal_wrapped(|ui| {
                    for (value, label) in [
                        ("", "unset"),
                        ("min_min", "min/min"),
                        ("max_min", "max/min"),
                        ("min_max", "min/max"),
                        ("max_max", "max/max"),
                        ("center", "centre"),
                    ] {
                        if ui.selectable_label(self.session.datum_anchor == value, label).clicked() {
                            self.session.datum_anchor = value.to_string();
                            self.session.dirty = true;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Offset X");
                    if ui.add(egui::TextEdit::singleline(&mut self.session.datum_offset_x).desired_width(70.0)).changed() {
                        self.session.dirty = true;
                    }
                    ui.label("Y");
                    if ui.add(egui::TextEdit::singleline(&mut self.session.datum_offset_y).desired_width(70.0)).changed() {
                        self.session.dirty = true;
                    }
                });
                ui.label("Start-from");
                ui.horizontal_wrapped(|ui| {
                    for (value, label) in
                        [("", "unset"), ("absolute", "absolute"), ("current_position", "current position"), ("user_origin", "user origin")]
                    {
                        if ui.selectable_label(self.session.start_from == value, label).clicked() {
                            self.session.start_from = value.to_string();
                            self.session.dirty = true;
                        }
                    }
                });
                if self.session.start_from == "current_position" || self.session.start_from == "user_origin" {
                    ui.label("Job origin");
                    ui.horizontal_wrapped(|ui| {
                        for (value, label) in [
                            ("", "unset"),
                            ("min_min", "SW"),
                            ("mid_min", "S"),
                            ("max_min", "SE"),
                            ("min_mid", "W"),
                            ("center", "centre"),
                            ("max_mid", "E"),
                            ("min_max", "NW"),
                            ("mid_max", "N"),
                            ("max_max", "NE"),
                        ] {
                            if ui.selectable_label(self.session.job_origin == value, label).clicked() {
                                self.session.job_origin = value.to_string();
                                self.session.dirty = true;
                            }
                        }
                    });
                } else {
                    ui.label(RichText::new("Job origin is used with current-position or user-origin starts.").color(theme::MUTED).size(11.0));
                }
                ui.separator();
                ui.label(RichText::new("Sheet").strong().size(13.0));
                ui.label(
                    RichText::new("The sheet is the stock rectangle. The bed comes from the machine profile and is not invented.")
                        .color(theme::MUTED)
                        .size(11.0),
                );
                ui.horizontal(|ui| {
                    ui.label("Width (mm)");
                    if ui.add(egui::TextEdit::singleline(&mut self.session.sheet_width).desired_width(70.0)).changed() {
                        self.session.dirty = true;
                        self.refresh_placed_scene();
                    }
                    ui.label("Height");
                    if ui.add(egui::TextEdit::singleline(&mut self.session.sheet_height).desired_width(70.0)).changed() {
                        self.session.dirty = true;
                        self.refresh_placed_scene();
                    }
                });
                if self.reveal_sheet_fields {
                    ui.scroll_to_cursor(Some(Align::BOTTOM));
                }
                ui.separator();
                ui.label(RichText::new("Heights").strong().size(13.0));
                ui.label(
                    RichText::new("Clearance and retract stay empty until typed. The planner uses its own defaults only when the job omits them.")
                        .color(theme::MUTED)
                        .size(11.0),
                );
                ui.label("Clearance (mm)");
                if ui.add(egui::TextEdit::singleline(&mut self.session.clearance).desired_width(70.0).hint_text("optional")).changed() {
                    self.mark_dirty();
                }
                ui.label("Retract (mm)");
                if ui.add(egui::TextEdit::singleline(&mut self.session.retract).desired_width(70.0).hint_text("optional")).changed() {
                    self.mark_dirty();
                }
                if !self.session.order_rules_set {
                    if ui.button("Set order rules").clicked() {
                        self.session.order_rules_set = true;
                        self.mark_dirty();
                    }
                } else {
                    if ui.checkbox(&mut self.session.order_inner_first, "Inner contours first").changed() {
                        self.mark_dirty();
                    }
                    if ui.checkbox(&mut self.session.order_cuts_last, "Cuts last").changed() {
                        self.mark_dirty();
                    }
                    ui.label(
                        RichText::new("Automatic rules apply when cut_order is empty. An explicit cut-order list still wins.")
                            .color(theme::MUTED)
                            .size(11.0),
                    );
                }
            });
        if response.header_response.clicked() {
            self.job_section_open = !self.job_section_open;
        }
    }

    fn placement_field(&mut self, ui: &mut egui::Ui, label: &str, field: usize) -> bool {
        ui.label(label);
        let value = match field {
            0 => &mut self.session.placement_x,
            1 => &mut self.session.placement_y,
            _ => &mut self.session.placement_rotate,
        };
        ui.add(egui::TextEdit::singleline(value).desired_width(180.0).hint_text("0")).changed()
    }

    fn tool_properties(&mut self, ui: &mut egui::Ui, index: usize) {
        ui.label(RichText::new("Tool").strong().size(13.0));
        let tool = &mut self.session.tools[index];
        ui.label("Name");
        if ui.add(egui::TextEdit::singleline(&mut tool.name).desired_width(180.0)).changed() {
            self.session.dirty = true;
        }
        ui.label("Kind");
        ui.horizontal(|ui| {
            let kinds = self.process_fields().tool_kinds;
            let tool = &mut self.session.tools[index];
            if !kinds.is_empty() && !kinds.contains(&tool.kind) {
                tool.kind = kinds[0];
                self.session.dirty = true;
            }
            for kind in kinds {
                let label = tool_kind_label(*kind);
                if ui.selectable_label(tool.kind == *kind, label).clicked() {
                    tool.kind = *kind;
                    self.session.dirty = true;
                }
            }
        });
        let size_label = match self.session.tools[index].kind {
            ToolKind::Kerf => "Kerf (mm)",
            ToolKind::Diameter => "Diameter (mm)",
            ToolKind::Wire => "Wire diameter (mm)",
        };
        ui.label(size_label);
        if ui.add(egui::TextEdit::singleline(&mut self.session.tools[index].size).desired_width(180.0).hint_text("enter a value")).changed() {
            self.session.dirty = true;
        }
    }

    fn operation_properties(&mut self, ui: &mut egui::Ui, index: usize) {
        ui.label(RichText::new("Operation").strong().size(13.0));
        ui.label("Name");
        if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].name).desired_width(180.0)).changed() {
            self.session.dirty = true;
        }
        ui.label(format!("Type: {}", self.session.operations[index].kind));
        if let Some(warning) = operation_match_warning(&self.session, &self.session.operations[index]) {
            ui.label(RichText::new(warning).color(theme::OVERRIDE).size(12.0));
        }
        if self.reveal_material_fields && !self.session.operations[index].override_fields.is_empty() {
            ui.label(
                RichText::new(format!("Orange overrides: {}", self.session.operations[index].override_fields.join(", ")))
                    .color(theme::OVERRIDE)
                    .strong()
                    .size(13.0),
            );
        }
        self.assign_editor(ui, index);
        let fields = self.process_fields();
        if self.session.operations[index].kind == "profile" {
            if fields.side {
                ui.label("Side");
                ui.horizontal(|ui| {
                    for side in ["inside", "outside", "on", "auto"] {
                        let selected = self.session.operations[index].side == side;
                        if ui.selectable_label(selected, side).clicked() {
                            self.session.operations[index].side = side.to_string();
                            self.session.dirty = true;
                        }
                    }
                });
            }
            ui.label("Layers");
            let available: Vec<String> = {
                let mut names: Vec<String> = all_part_layers(&self.session).into_iter().map(|layer| layer.name).collect();
                for layer in &self.session.operations[index].layers {
                    if !names.iter().any(|name| name == layer) {
                        names.push(layer.clone());
                    }
                }
                names
            };
            for layer in available {
                let mut on = self.session.operations[index].layers.iter().any(|item| item == &layer);
                let label = layer_parts_label(&self.session, &layer);
                if ui.checkbox(&mut on, label).changed() {
                    let layers = &mut self.session.operations[index].layers;
                    if on {
                        if !layers.iter().any(|item| item == &layer) {
                            layers.push(layer);
                        }
                    } else {
                        layers.retain(|item| item != &layer);
                    }
                    self.mark_dirty();
                }
            }
            if all_part_layers(&self.session).is_empty() {
                ui.label(
                    RichText::new("Import a part to pick layers. SVG and DXF layer names are listed from every part.").color(theme::MUTED).size(11.0),
                );
            }
        }
        ui.label("Tool id");
        let mut tool_text = self.session.operations[index].tool_id.to_string();
        if ui.add(egui::TextEdit::singleline(&mut tool_text).desired_width(80.0)).changed()
            && let Ok(id) = tool_text.parse::<u32>()
        {
            self.session.operations[index].tool_id = id;
            self.session.dirty = true;
        }
        let op_fields: Vec<(&str, usize)> = [
            fields.depth.then_some(("Depth (mm)", 0usize)),
            fields.step_down.then_some(("Step down (mm)", 1)),
            fields.cut_feed.then_some(("Cut feed (mm/min)", 2)),
            fields.plunge_feed.then_some(("Plunge feed (mm/min)", 3)),
            fields.power.then_some(("Power (0..1)", 4)),
        ]
        .into_iter()
        .flatten()
        .collect();
        for (label, field) in op_fields {
            let name = field_name(field);
            let color = self.override_color(index, name);
            ui.label(RichText::new(label).color(color));
            let value = match field {
                0 => &mut self.session.operations[index].depth,
                1 => &mut self.session.operations[index].step_down,
                2 => &mut self.session.operations[index].cut_feed,
                3 => &mut self.session.operations[index].plunge_feed,
                _ => &mut self.session.operations[index].power,
            };
            if ui.add(egui::TextEdit::singleline(value).desired_width(180.0).hint_text("enter a value").text_color(color)).changed() {
                operations::mark_override(&mut self.session.operations[index], name);
                self.mark_dirty();
            }
        }
        if fields.leads {
            ui.label(RichText::new("Leads (omit to use the job-file default)").color(theme::MUTED).size(11.0));
            ui.label("Lead in");
            self.lead_editor(ui, index, true);
            ui.label("Lead out");
            self.lead_editor(ui, index, false);
            ui.label(
                RichText::new("Overridden material fields are shown in orange. Reset restores the last applied preset.")
                    .color(theme::MUTED)
                    .size(11.0),
            );
        }
        if fields.direction {
            ui.label("Direction");
            ui.horizontal(|ui| {
                for item in ["", "auto", "climb", "conventional"] {
                    let label = if item.is_empty() { "default" } else { item };
                    let selected = self.session.operations[index].direction == item;
                    if ui.selectable_label(selected, label).clicked() {
                        self.session.operations[index].direction = item.to_string();
                        self.session.dirty = true;
                    }
                }
            });
        }
        if fields.compensation {
            ui.label("Compensation");
            ui.horizontal(|ui| {
                for item in ["", "computed", "controller_left", "controller_right"] {
                    let label = if item.is_empty() { "default" } else { item };
                    let selected = self.session.operations[index].compensation == item;
                    if ui.selectable_label(selected, label).clicked() {
                        self.session.operations[index].compensation = item.to_string();
                        self.session.dirty = true;
                    }
                }
            });
        }
        if fields.pierce_delay {
            ui.label("Pierce delay (s)");
            if ui
                .add(egui::TextEdit::singleline(&mut self.session.operations[index].pierce_delay).desired_width(180.0).hint_text("optional"))
                .changed()
            {
                self.session.dirty = true;
            }
        }
        if fields.post_delay {
            ui.label("Post delay (s)");
            if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].post_delay).desired_width(180.0).hint_text("optional")).changed()
            {
                self.mark_dirty();
            }
        }
        self.tabs_editor(ui, index);
        if self.state.process == Some(SubProcess::Plasma) {
            self.small_hole_editor(ui, index);
        }
        self.material_editor(ui, index);
    }

    fn override_color(&self, index: usize, field: &str) -> Color32 {
        if self.session.operations.get(index).is_some_and(|op| op.override_fields.iter().any(|name| name == field)) {
            theme::OVERRIDE
        } else {
            theme::TEXT
        }
    }

    fn assign_editor(&mut self, ui: &mut egui::Ui, index: usize) {
        ui.label("Assign");
        ui.horizontal(|ui| {
            for (id, label) in [("", "layers"), ("parts", "part"), ("layers", "layer"), ("contours", "contour")] {
                let selected = self.session.operations[index].assign == id;
                if ui.selectable_label(selected, label).clicked() {
                    if id == "parts" {
                        let parts = all_part_ids(&self.session);
                        let op = &mut self.session.operations[index];
                        op.assign = "parts".to_string();
                        op.assign_parts = parts;
                        op.layers.clear();
                    } else {
                        self.session.operations[index].assign = id.to_string();
                    }
                    self.mark_dirty();
                }
            }
        });
        if self.session.operations[index].assign == "parts" || self.session.operations[index].assign == "contours" {
            ui.label(if self.session.operations[index].assign == "parts" { "Part ids (comma-separated)" } else { "Part id" });
            if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].assign_parts).desired_width(180.0)).changed() {
                self.mark_dirty();
            }
            if self.session.operations[index].assign == "parts" && self.session.operations[index].assign_parts.trim().is_empty() {
                ui.label(
                    RichText::new("Enter at least one part id, or pick part assignment again to use every part.").color(theme::OVERRIDE).size(11.0),
                );
            }
        }
        if self.session.operations[index].assign == "contours" {
            ui.label("Contour ids (comma-separated)");
            if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].assign_contours).desired_width(180.0)).changed() {
                self.mark_dirty();
            }
        }
    }

    fn tabs_editor(&mut self, ui: &mut egui::Ui, index: usize) {
        if self.session.operations[index].kind != "profile" {
            return;
        }
        ui.add_space(6.0);
        ui.label(RichText::new("Tabs / bridges").strong().size(13.0));
        ui.label(
            RichText::new("Leave these empty until you want tabs. Width is required when a count or spacing is set.").color(theme::MUTED).size(11.0),
        );
        ui.label("Count");
        if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].tabs_count).desired_width(80.0).hint_text("optional")).changed() {
            self.mark_dirty();
        }
        ui.label("Spacing (mm)");
        if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].tabs_spacing).desired_width(80.0).hint_text("optional")).changed() {
            self.mark_dirty();
        }
        ui.label("Width (mm)");
        if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].tabs_width).desired_width(80.0).hint_text("optional")).changed() {
            self.mark_dirty();
        }
        ui.label("Height (mm)");
        if ui.add(egui::TextEdit::singleline(&mut self.session.operations[index].tabs_height).desired_width(80.0).hint_text("optional")).changed() {
            self.mark_dirty();
        }
    }

    fn small_hole_editor(&mut self, ui: &mut egui::Ui, index: usize) {
        ui.add_space(6.0);
        ui.label(RichText::new("Plasma small-hole rules").strong().size(13.0));
        ui.label(
            RichText::new("Enter both a max diameter and an action. Values are recorded as entered. Posts do not invent THC codes.")
                .color(theme::MUTED)
                .size(11.0),
        );
        ui.label("Max diameter (mm)");
        if ui
            .add(egui::TextEdit::singleline(&mut self.session.operations[index].small_hole_max).desired_width(80.0).hint_text("user-entered"))
            .changed()
        {
            self.mark_dirty();
        }
        ui.label("Action");
        ui.horizontal(|ui| {
            for action in ["", "pierce_only", "thc_off"] {
                let label = if action.is_empty() { "unset" } else { action };
                let selected = self.session.operations[index].small_hole_action == action;
                if ui.selectable_label(selected, label).clicked() {
                    self.session.operations[index].small_hole_action = action.to_string();
                    self.mark_dirty();
                }
            }
        });
    }

    fn material_editor(&mut self, ui: &mut egui::Ui, index: usize) {
        ui.add_space(6.0);
        ui.label(RichText::new("Material").strong().size(13.0));
        let materials =
            machines::session_profile(&self.profile_store, &self.session).ok().flatten().map(|(_, head)| head.materials).unwrap_or_default();
        if materials.is_empty() {
            ui.label(RichText::new("No materials on the selected head. Enter values, then Save to library.").color(theme::MUTED).size(11.0));
        } else {
            let current = self.session.operations[index].material_name.clone();
            let mut applied = None;
            egui::ComboBox::from_id_salt("material-preset")
                .selected_text(if current.is_empty() { "Select a preset".to_string() } else { current })
                .show_ui(ui, |ui| {
                    for entry in &materials {
                        if ui.selectable_label(self.session.operations[index].material_name == entry.name, &entry.name).clicked() {
                            applied = Some(entry.clone());
                        }
                    }
                });
            if let Some(entry) = applied {
                self.session.selected = Some(TreeSelection::Operation(index));
                match machines::apply_material(&mut self.session, &entry) {
                    Ok(msg) => {
                        self.state.status = msg;
                        self.schedule_regen();
                    }
                    Err(err) => self.state.status = err,
                }
            }
        }
        ui.horizontal(|ui| {
            if ui.button("Reset").clicked() {
                let name = self.session.operations[index].material_name.clone();
                operations::reset_overrides(&mut self.session.operations[index]);
                if let Some(entry) = materials.iter().find(|item| item.name == name).cloned() {
                    self.session.selected = Some(TreeSelection::Operation(index));
                    let _ = machines::apply_material(&mut self.session, &entry);
                }
                self.schedule_regen();
            }
        });
        ui.horizontal(|ui| {
            ui.label("Save as");
            ui.add(egui::TextEdit::singleline(&mut self.material_save_name).desired_width(140.0).hint_text("library name"));
            if ui.button("Save to library").clicked() {
                self.session.selected = Some(TreeSelection::Operation(index));
                match machines::material_from_selected_operation(&self.session, &self.material_save_name)
                    .and_then(|entry| machines::save_material(&self.profile_store, &self.session, entry))
                {
                    Ok(msg) => self.state.status = msg,
                    Err(err) => self.state.status = err,
                }
            }
        });
        if !self.session.operations[index].override_fields.is_empty() {
            ui.label(
                RichText::new(format!("Overrides: {}", self.session.operations[index].override_fields.join(", "))).color(theme::OVERRIDE).size(11.0),
            );
        }
    }

    fn lead_editor(&mut self, ui: &mut egui::Ui, index: usize, lead_in: bool) {
        ui.horizontal(|ui| {
            let kind = if lead_in { &mut self.session.operations[index].lead_in } else { &mut self.session.operations[index].lead_out };
            for item in ["", "none", "line", "arc", "perpendicular"] {
                let label = if item.is_empty() { "default" } else { item };
                if ui.selectable_label(kind.as_str() == item, label).clicked() {
                    *kind = item.to_string();
                    self.session.dirty = true;
                }
            }
        });
        let kind = if lead_in { self.session.operations[index].lead_in.as_str() } else { self.session.operations[index].lead_out.as_str() };
        if kind == "line" || kind == "arc" || kind == "perpendicular" {
            let label = if kind == "arc" { "Radius (mm)" } else { "Length (mm)" };
            ui.label(label);
            let size = if lead_in { &mut self.session.operations[index].lead_in_size } else { &mut self.session.operations[index].lead_out_size };
            if ui.add(egui::TextEdit::singleline(size).desired_width(180.0).hint_text("enter a value")).changed() {
                self.session.dirty = true;
            }
        }
    }

    fn process_tab(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("New Job").strong().size(16.0));
        ui.add_space(6.0);
        ui.label(
            RichText::new("Select a machine and units, then go to Setup to import parts. The process comes from the selected machine head.")
                .color(theme::MUTED)
                .size(12.0),
        );
        ui.add_space(10.0);
        if let Some(label) = self.current_process_label() {
            ui.label(RichText::new("Process").strong().size(13.0));
            ui.label(RichText::new(label).size(13.0));
            if let Some(process) = self.state.process
                && let Some(id) = suggested_post_for_process(process.label())
            {
                ui.label(RichText::new(format!("Suggested post: {id} (you can change this on the Export tab)")).color(theme::MUTED).size(12.0));
            }
            ui.add_space(10.0);
        } else {
            ui.label(RichText::new("Process").strong().size(13.0));
            ui.label(RichText::new("Select a machine to set the process from its head.").color(theme::MUTED).size(12.0));
            ui.add_space(10.0);
        }
        ui.label(RichText::new("Machine").strong().size(13.0));
        self.machine_job_picker(ui);
        ui.add_space(8.0);
        ui.label(RichText::new("Units").strong().size(13.0));
        ui.horizontal(|ui| {
            for units in ["mm", "inch"] {
                if ui.selectable_label(self.session.units == units, units).clicked() {
                    self.session.units = units.to_string();
                    self.session.dirty = true;
                }
            }
        });
        ui.label(RichText::new("Displayed values are millimetres. Inch is stored on save when selected.").color(theme::MUTED).size(11.0));
        ui.add_space(8.0);
        if let Some(path) = &self.session.job_path {
            ui.label(RichText::new(format!("Save location: {}", path.display())).color(theme::MUTED).size(12.0));
        } else {
            ui.label(RichText::new("Save location: unsaved (File → Save Job As…)").color(theme::MUTED).size(12.0));
        }
        ui.add_space(12.0);
        if ui.button("Continue to Setup").clicked() && self.state.workspace.has_tab(ModeTab::Setup) {
            let _ = self.state.set_tab(ModeTab::Setup);
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
                self.show_viewport_area(left);
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
        ui.label("Post");
        let current = if self.session.post_id.is_empty() { "Choose a post".to_string() } else { self.session.post_id.clone() };
        let process = self.current_process_id().unwrap_or("").to_string();
        let include = self.session.post_id.clone();
        let posts = if process.is_empty() { list_posts() } else { list_posts_for_process(&process, Some(include.as_str())) };
        egui::ComboBox::from_id_salt("post-id").selected_text(current).width(200.0).show_ui(ui, |ui| {
            for (id, name, _) in posts {
                if ui.selectable_label(self.session.post_id == id, format!("{id} ({name})")).clicked() {
                    self.session.post_id = id;
                    self.rebuild_play_index();
                }
            }
        });
        if !process.is_empty() {
            ui.label(RichText::new("Posts shown are those documented for this head.").color(theme::MUTED).size(11.0));
        }
        if ui.button("Preview G-code").clicked() {
            self.preview_gcode();
        }
        ui.add_space(6.0);
        ui.label("Format");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_format).desired_width(200.0));
        ui.label("File name");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_name).desired_width(200.0));
        ui.label("Save to");
        ui.add(egui::TextEdit::singleline(&mut self.state.export_path).desired_width(200.0));
        ui.checkbox(&mut self.state.embed_thumbnail, "Embed thumbnail preview");
        ui.label(RichText::new("Thumbnail embedding is not written for G-code.").color(theme::MUTED).size(11.0));
        ui.add_space(8.0);
        self.show_preflight_checks(ui);
        ui.add_space(8.0);
        let checks = session_checks(&self.session, self.document.as_ref(), Some(&self.profile_store), self.current_process_id());
        let blocked = blocks_export(&checks);
        let export = egui::Button::new(RichText::new("Export File").color(Color32::WHITE)).fill(theme::ACCENT).min_size(Vec2::new(200.0, 28.0));
        if ui.add_enabled(!blocked, export).clicked() {
            self.export_gcode();
        }
        if blocked {
            ui.label(RichText::new("Export is blocked until the errors above are fixed.").color(theme::OFF_BED).size(11.0));
        }
        ui.add_space(6.0);
        ui.add_enabled(false, egui::Button::new("Send to printer").min_size(Vec2::new(200.0, 28.0)));
        ui.label(RichText::new("coming later").color(theme::MUTED).size(11.0));
        self.show_messages(ui);
    }

    fn stream_connect(&mut self) {
        match run_stream::stream_mode_from_session(&self.session, &self.profile_store, self.state.safe_mode) {
            Ok(mode) => match RunConnection::connect(&self.state.port, &self.state.baud, mode) {
                Ok((mut conn, events)) => {
                    run_stream::apply_events(&mut self.state, &events, &mut conn);
                    self.state.connected = true;
                    self.state.status = "Connected".to_string();
                    self.run_conn = Some(conn);
                }
                Err(err) => self.state.status = err,
            },
            Err(err) => self.state.status = err,
        }
    }

    fn stream_disconnect(&mut self) {
        if let Some(mut conn) = self.run_conn.take() {
            match conn.close() {
                Ok(events) => run_stream::apply_events(&mut self.state, &events, &mut conn),
                Err(err) => self.state.status = err,
            }
        }
        self.state.connected = false;
        if !self.state.status.contains("not complete") {
            self.state.status = "Disconnected".to_string();
        }
    }

    fn preflight_blocks_motion(&self) -> Option<String> {
        let checks = session_checks(&self.session, self.document.as_ref(), Some(&self.profile_store), self.current_process_id());
        if blocks_export(&checks) { Some("pre-flight has errors; no motion is sent until they are fixed".to_string()) } else { None }
    }

    fn handle_stream_result(&mut self, result: Result<Vec<campanion_stream::StreamEvent>, String>) {
        match result {
            Ok(events) => {
                if let Some(conn) = self.run_conn.as_mut() {
                    run_stream::apply_events(&mut self.state, &events, conn);
                }
                if let Some(line) = self.run_conn.as_ref().and_then(|conn| conn.last_acked_program_line()) {
                    self.seek_gcode_line(line);
                }
            }
            Err(err) => {
                self.state.status = err;
                if self.state.status.contains("disconnected") {
                    self.state.connected = false;
                    self.run_conn = None;
                }
            }
        }
    }

    fn stream_start(&mut self) {
        if let Some(err) = self.preflight_blocks_motion() {
            self.state.status = err;
            return;
        }
        if self.state.gcode.trim().is_empty() {
            self.generate_and_post();
        }
        if self.state.gcode.trim().is_empty() {
            self.state.status = "Start needs posted G-code; plan and post the job first".to_string();
            return;
        }
        let lines: Vec<String> = self.state.gcode.lines().map(str::to_string).collect();
        self.state.run_lines = crate::gcode_line_count(&self.state.gcode);
        self.state.run_line = 0;
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.start(lines, JobKind::Program),
            None => Err("Start needs a connected controller".to_string()),
        };
        if result.is_ok() {
            let _ = self.state.start_run();
        }
        self.handle_stream_result(result);
    }

    fn stream_frame(&mut self) {
        if let Some(err) = self.preflight_blocks_motion() {
            self.state.status = err;
            return;
        }
        let feed = match run_stream::parse_jog_or_frame_feed(&self.state.frame_feed) {
            Ok(feed) => feed,
            Err(err) => {
                self.state.status = err;
                return;
            }
        };
        let Some(scene) = self.scene.as_ref() else {
            self.state.status = "Frame needs a planned toolpath".to_string();
            return;
        };
        let Some((min, max)) = projected_bounds(scene, Projection::Top).or(scene.bounds) else {
            self.state.status = "Frame needs a planned toolpath with bounds".to_string();
            return;
        };
        let bounds = match FrameBounds::new(min[0], min[1], max[0], max[1]) {
            Ok(bounds) => bounds,
            Err(err) => {
                self.state.status = err;
                return;
            }
        };
        let lines = match frame_program(bounds, feed) {
            Ok(lines) => lines,
            Err(err) => {
                self.state.status = err;
                return;
            }
        };
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.start(lines, JobKind::Frame),
            None => Err("Frame needs a connected controller".to_string()),
        };
        self.handle_stream_result(result);
    }

    fn stream_pause(&mut self) {
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.pause(),
            None => Err("Pause needs a connected controller".to_string()),
        };
        self.handle_stream_result(result);
    }

    fn stream_resume(&mut self) {
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.resume(),
            None => Err("Resume needs a connected controller".to_string()),
        };
        self.handle_stream_result(result);
    }

    fn stream_stop(&mut self) {
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.stop(),
            None => {
                self.state.stop_run();
                return;
            }
        };
        self.handle_stream_result(result);
    }

    fn stream_jog(&mut self, axis: usize, sign: i32) {
        let feed = match run_stream::parse_jog_or_frame_feed(&self.state.jog_feed) {
            Ok(feed) => feed,
            Err(err) => {
                self.state.status = err;
                return;
            }
        };
        let delta = self.state.jog_step_mm * f64::from(sign.signum());
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.jog(axis, delta, feed),
            None => Err("Jog needs a connected controller".to_string()),
        };
        self.handle_stream_result(result);
    }

    fn stream_unlock(&mut self) {
        let result = match self.run_conn.as_mut() {
            Some(conn) => conn.unlock_after_confirm(),
            None => Err("$X needs a connected controller".to_string()),
        };
        self.handle_stream_result(result);
    }

    fn tick_stream(&mut self, ctx: &egui::Context) {
        if self.run_conn.is_none() {
            return;
        }
        let _ = self.tick_stream_once(false);
        ctx.request_repaint_after(std::time::Duration::from_millis(50));
    }

    fn tick_stream_once(&mut self, force_status: bool) -> Result<(), String> {
        let events = match self.run_conn.as_mut() {
            Some(conn) => conn.tick(force_status)?,
            None => return Ok(()),
        };
        if let Some(conn) = self.run_conn.as_mut() {
            run_stream::apply_events(&mut self.state, &events, conn);
        }
        if let Some(line) = self.run_conn.as_ref().and_then(|conn| conn.last_acked_program_line()) {
            self.seek_gcode_line(line);
        }
        Ok(())
    }

    fn run_alarm_panel(&mut self, ui: &mut egui::Ui) {
        let Some(conn) = self.run_conn.as_ref() else {
            return;
        };
        if conn.phase() != campanion_stream::HostPhase::Alarm && !self.state.in_alarm {
            return;
        }
        ui.label(RichText::new("Alarm").strong().color(theme::OFF_BED).size(13.0));
        if let Some((code, text)) = conn.last_alarm() {
            ui.label(RichText::new(format!("ALARM:{code}")).color(theme::OFF_BED).size(12.0).monospace());
            ui.label(RichText::new(text).color(theme::OFF_BED).size(11.0));
        } else {
            ui.label(RichText::new("Controller is in alarm.").color(theme::OFF_BED).size(11.0));
        }
        if let Some((code, text)) = conn.last_error() {
            ui.label(RichText::new(format!("error:{code}")).color(theme::OFF_BED).size(12.0).monospace());
            ui.label(RichText::new(text).color(theme::OFF_BED).size(11.0));
        }
        ui.label(RichText::new("$X unlock requires confirm.").color(theme::OFF_BED).size(11.0));
        if ui.button("Confirm $X unlock").clicked()
            && let Some(conn) = self.run_conn.as_mut()
        {
            conn.confirm_unlock = true;
        }
        let confirmed = self.run_conn.as_ref().is_some_and(|conn| conn.confirm_unlock);
        ui.add_enabled_ui(confirmed, |ui| {
            if ui.button("Send $X").clicked() {
                self.stream_unlock();
            }
        });
        ui.add_space(6.0);
        ui.separator();
        ui.add_space(4.0);
    }

    fn run_left(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Machine").strong().size(14.0));
        ui.add_space(4.0);
        ui.label(RichText::new(STREAM_HARDWARE_NOTICE).color(theme::OVERRIDE).size(11.0));
        ui.label("Port");
        let ports = run_stream::available_ports(self.screenshot.is_some());
        let current_port = self.state.port.clone();
        egui::ComboBox::from_id_salt("run-port")
            .selected_text(if current_port.is_empty() { "Select a port".to_string() } else { current_port })
            .width(180.0)
            .show_ui(ui, |ui| {
                for port in &ports {
                    if ui.selectable_label(self.state.port == *port, port).clicked() {
                        self.state.port = port.clone();
                    }
                }
            });
        if ui.button("Refresh ports").clicked() {
            self.state.status = format!("{} port(s) found", run_stream::available_ports(self.screenshot.is_some()).len());
        }
        ui.label(
            RichText::new("On Linux, membership in the dialout group is typically required to open a USB serial device.")
                .color(theme::MUTED)
                .size(11.0),
        );
        ui.label("Baud");
        let baud_choices = ["115200 (Grbl documented)", "57600", "38400", "19200", "9600"];
        let baud_label = if self.state.baud.is_empty() { "Select or type a baud rate".to_string() } else { self.state.baud.clone() };
        egui::ComboBox::from_id_salt("run-baud").selected_text(baud_label).width(180.0).show_ui(ui, |ui| {
            for choice in baud_choices {
                if ui.selectable_label(self.state.baud == choice, choice).clicked() {
                    self.state.baud = choice.to_string();
                }
            }
        });
        ui.add(egui::TextEdit::singleline(&mut self.state.baud).desired_width(180.0).hint_text("baud rate"));
        ui.checkbox(&mut self.state.safe_mode, "Send-response safe mode (no RX buffer required)");
        ui.label(
            RichText::new(
                "Character-counting needs a serial RX buffer entered on the machine profile. Safe mode waits for ok or error after each line.",
            )
            .color(theme::MUTED)
            .size(11.0),
        );
        ui.horizontal(|ui| {
            if self.state.connected {
                if ui.button("Disconnect").clicked() {
                    self.stream_disconnect();
                }
                ui.label(RichText::new("Connected").color(theme::START));
            } else if ui.button("Connect").clicked() {
                self.stream_connect();
            }
        });
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
        ui.label("Jog feed (mm/min)");
        ui.add(egui::TextEdit::singleline(&mut self.state.jog_feed).desired_width(180.0).hint_text("required, not assumed"));
        let enabled = self.state.can_jog();
        ui.add_enabled_ui(enabled, |ui| {
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                if ui.button("↑").clicked() {
                    self.stream_jog(1, 1);
                }
            });
            ui.horizontal(|ui| {
                if ui.button("←").clicked() {
                    self.stream_jog(0, -1);
                }
                if ui.button("→").clicked() {
                    self.stream_jog(0, 1);
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                if ui.button("↓").clicked() {
                    self.stream_jog(1, -1);
                }
                ui.add_space(12.0);
                if ui.button("Z+").clicked() {
                    self.stream_jog(2, 1);
                }
                if ui.button("Z−").clicked() {
                    self.stream_jog(2, -1);
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
            ui.add_enabled_ui(false, |ui| {
                let _ = ui.button("Home");
                let _ = ui.button("Set Origin");
            });
        });
        ui.label(RichText::new("Home ($H) and Set Origin are not sent in this slice.").color(theme::MUTED).size(11.0));
        if !self.state.connected {
            ui.label(RichText::new("Connect a controller to jog, frame, start, or use STOP.").color(theme::MUTED).size(11.0));
        } else if !enabled {
            ui.label(RichText::new("Jog is locked while running or in hold. Only Pause and STOP stay available.").color(theme::MUTED).size(11.0));
        }
    }

    fn run_right(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Job").strong().size(14.0));
        ui.add_space(4.0);
        self.run_alarm_panel(ui);
        ui.label("Frame feed (mm/min)");
        ui.add(egui::TextEdit::singleline(&mut self.state.frame_feed).desired_width(180.0).hint_text("required, laser stays off"));
        ui.add_enabled_ui(self.state.can_frame(), |ui| {
            if ui.add(egui::Button::new("Frame").min_size(Vec2::new(180.0, 26.0))).clicked() {
                self.stream_frame();
            }
        });
        ui.label(RichText::new("Frame traces the job bounding box with M5/S0 at the feed you entered.").color(theme::MUTED).size(11.0));
        let can_start = self.state.can_start();
        ui.add_enabled_ui(can_start, |ui| {
            let fill = if can_start { theme::START } else { theme::MUTED };
            let start = egui::Button::new(RichText::new("Start").color(Color32::WHITE)).fill(fill).min_size(Vec2::new(180.0, 28.0));
            if ui.add(start).clicked() {
                self.stream_start();
            }
        });
        ui.add_enabled_ui(self.state.can_pause() || self.state.run == RunPhase::Paused, |ui| {
            let label = if self.state.run == RunPhase::Paused { "Resume" } else { "Pause" };
            if ui.add(egui::Button::new(label).min_size(Vec2::new(180.0, 28.0))).clicked() {
                if self.state.run == RunPhase::Paused {
                    self.stream_resume();
                } else {
                    self.stream_pause();
                }
            }
        });
        let phase = if !self.state.connected {
            "Not connected".to_string()
        } else {
            match self.state.run {
                RunPhase::Idle => "Idle".to_string(),
                RunPhase::Running => "Running".to_string(),
                RunPhase::Paused => "Hold".to_string(),
                RunPhase::Stopped => "Stopped — last line kept".to_string(),
                RunPhase::Finished => "Finished".to_string(),
            }
        };
        ui.label(RichText::new(phase).color(theme::MUTED).size(12.0));
        ui.label(format!("Line {} / {}", self.state.run_line, self.state.run_lines));
        self.show_preflight_checks(ui);
        ui.separator();
        ui.label(RichText::new("Console").strong().size(13.0));
        egui::ScrollArea::vertical().max_height(180.0).stick_to_bottom(true).show(ui, |ui| {
            if let Some(conn) = &self.run_conn {
                if conn.console.is_empty() {
                    ui.label(RichText::new("Connected. Waiting for messages.").color(theme::MUTED).size(11.0).monospace());
                } else {
                    for line in &conn.console {
                        ui.label(RichText::new(line).size(11.0).monospace());
                    }
                }
            } else {
                ui.label(RichText::new("Not connected.").color(theme::MUTED).size(11.0).monospace());
            }
        });
    }

    fn viewport_panel(&mut self, ui: &mut egui::Ui, caption: &str) {
        self.show_viewport(ui, caption);
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        if let Some(kind) = self.file_dialog {
            let mut open = true;
            let (title, prompt, action) = match kind {
                FileDialogKind::Open => ("Open", "Path to a job, part (DXF/SVG/STEP/STL), IR JSON, or G-code file:", "Open"),
                FileDialogKind::Import => ("Import part", "Path to a DXF, SVG, STEP or STL file:", "Import"),
                FileDialogKind::SaveJob => ("Save job", "Path for a campanion-job/1 file:", "Save"),
                FileDialogKind::ExportGcode => ("Export G-code", "Path for a .ngc or .gcode file:", "Export"),
            };
            egui::Window::new(title).open(&mut open).collapsible(false).resizable(false).show(ctx, |ui| {
                ui.label(prompt);
                let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
                ui.label(RichText::new(format!("Current directory: {}", current_dir_display(&cwd))).color(theme::MUTED).size(11.0));
                ui.label(RichText::new("Relative paths and ~ are accepted.").color(theme::MUTED).size(11.0));
                ui.add(egui::TextEdit::singleline(&mut self.path_text).desired_width(420.0).hint_text("path/to/file"));
                ui.horizontal(|ui| {
                    if ui.button(action).clicked() && !self.path_text.trim().is_empty() {
                        self.confirm_typed_path(kind);
                    }
                    if ui.button("Cancel").clicked() {
                        self.file_dialog = None;
                    }
                });
            });
            if !open {
                self.file_dialog = None;
            }
        }
        show_placeholder_window(ctx, "About CAMpanion", &mut self.about, |ui| {
            ui.label(RichText::new(format!("CAMpanion {}", env!("CARGO_PKG_VERSION"))).strong().size(16.0));
            ui.label("Machine-neutral CAM companion.");
            ui.add_space(6.0);
            ui.label(GUI_PREVIEW_NOTICE);
            ui.label(STREAM_HARDWARE_NOTICE);
            ui.label(
                "The Run tab can stream to a Grbl-class controller over USB serial when you connect. Home and Set Origin are not sent in this slice.",
            );
            ui.add_space(6.0);
            ui.label("Licence: MIT OR Apache-2.0");
        });
        show_placeholder_window(ctx, "Preferences", &mut self.preferences, |ui| {
            ui.label("Units are millimetres.");
            ui.label("The program never ships default power, speed or material values.");
            ui.label("Further preferences will land with the full GUI.");
        });
        if self.machines.is_some() {
            let mut open = true;
            egui::Window::new("Machines").open(&mut open).default_size([960.0, 680.0]).min_size([720.0, 480.0]).resizable(true).show(ctx, |ui| {
                if let Some(manager) = self.machines.as_mut() {
                    manager.show(ui);
                }
            });
            if !open {
                self.machines = None;
                if let Ok(store) = campanion_cli::ProfileStore::from_env() {
                    self.profile_store = store;
                }
            }
        }
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
        let shot_name = self.screenshot.as_ref().and_then(|job| job.plan.get(job.index).map(|(_, _, name)| name.clone()));
        if let Some((workspace, tab)) = switch_to {
            self.state.set_workspace(workspace);
            let _ = self.state.set_tab(tab);
            self.prepare_screenshot_frame(shot_name.as_deref());
        }
        let Some(job) = self.screenshot.as_mut() else {
            return;
        };
        if job.index >= job.plan.len() {
            if !job.finished {
                for path in &job.written {
                    eprintln!("campanion-gui: wrote {}", path.display());
                }
                job.finished = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
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

fn empty_scene_with_bounds(bounds: [f64; 4]) -> Scene {
    Scene {
        strokes: Vec::new(),
        pierces: Vec::new(),
        labels: Vec::new(),
        bounds: Some(([bounds[0], bounds[1]], [bounds[2], bounds[3]])),
        z_range: None,
        total_length: 0.0,
        names: Vec::new(),
        rotary: None,
        lathe: None,
        timeline: Vec::new(),
    }
}

fn screenshot_part_path(name: &str) -> Option<PathBuf> {
    let mut candidates = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        candidates.push(cwd.join("examples").join(name));
    }
    candidates.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples").join(name));
    candidates.into_iter().find(|path| path.is_file())
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

#[allow(dead_code)]
fn screenshot_job_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        out.push(cwd.join("examples/laser_rect.job.json"));
    }
    out.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/laser_rect.job.json"));
    out
}

fn is_part_ext(ext: &str) -> bool {
    ext.eq_ignore_ascii_case("dxf")
        || ext.eq_ignore_ascii_case("svg")
        || ext.eq_ignore_ascii_case("step")
        || ext.eq_ignore_ascii_case("stp")
        || ext.eq_ignore_ascii_case("stl")
}

fn path_looks_like_job(path: &Path) -> bool {
    path.file_name().and_then(|name| name.to_str()).is_some_and(|name| name.ends_with(".job.json"))
}

fn default_tool_kind(process: Option<SubProcess>, workspace: Workspace) -> ToolKind {
    fields::fields_for(process, workspace).tool_kinds.first().copied().unwrap_or(ToolKind::Diameter)
}

fn tool_kind_label(kind: ToolKind) -> &'static str {
    match kind {
        ToolKind::Kerf => "kerf",
        ToolKind::Diameter => "diameter",
        ToolKind::Wire => "wire",
    }
}

#[allow(dead_code)]
fn screenshot_ir_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        out.push(cwd.join("crates/cli/tests/golden/sample.ir.json"));
        out.push(cwd.join("cli/tests/golden/sample.ir.json"));
    }
    out.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../cli/tests/golden/sample.ir.json"));
    out
}

fn ribbon_command(ui: &mut egui::Ui, label: &str, selected: bool) -> egui::Response {
    ui.add(ribbon_command_widget(label, selected))
}

fn ribbon_command_widget(label: &str, selected: bool) -> egui::Button<'static> {
    let text =
        if selected { RichText::new(label.to_owned()).strong().size(12.0).color(Color32::WHITE) } else { RichText::new(label.to_owned()).size(12.0) };
    let button = if selected { egui::Button::new(text).fill(theme::ACCENT) } else { egui::Button::new(text) };
    button.min_size(Vec2::new(72.0, 32.0))
}

fn ribbon_group(ui: &mut egui::Ui, title: &str, add_contents: impl FnOnce(&mut egui::Ui)) {
    ui.vertical(|ui| {
        ui.horizontal(|ui| {
            add_contents(ui);
        });
        ui.add_space(2.0);
        ui.label(RichText::new(title).color(theme::MUTED).size(11.0));
    });
}

fn ribbon_group_separator(ui: &mut egui::Ui) {
    ui.add_space(8.0);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, RIBBON_GROUP_ROW - 10.0), egui::Sense::hover());
    ui.painter().rect_filled(rect, 0.0, theme::BORDER);
    ui.add_space(8.0);
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

fn expand_bounds(existing: Option<([f64; 2], [f64; 2])>, extra: Option<[f64; 4]>) -> Option<([f64; 2], [f64; 2])> {
    match (existing, extra) {
        (None, None) => None,
        (Some(bounds), None) => Some(bounds),
        (None, Some([x0, y0, x1, y1])) => Some(([x0, y0], [x1, y1])),
        (Some((min, max)), Some([x0, y0, x1, y1])) => Some(([min[0].min(x0), min[1].min(y0)], [max[0].max(x1), max[1].max(y1)])),
    }
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

fn field_name(field: usize) -> &'static str {
    match field {
        0 => "depth",
        1 => "step_down",
        2 => "cut_feed",
        3 => "plunge_feed",
        _ => "power",
    }
}
