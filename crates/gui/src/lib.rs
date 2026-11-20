//! Navigation and session state for the CAMpanion graphical workspace.
//!
//! CAMpanion is primarily command-line driven. This GUI is still in development: it can open a
//! part or job, edit operations, generate toolpaths, export G-code, and stream to a Grbl-class
//! controller over USB serial. Streaming is not hardware-tested and is not a finished CAM interface.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]

pub mod app;
pub mod edit;
pub mod fields;
pub mod files;
pub mod machines;
pub mod operations;
pub mod run_stream;
pub mod session;
pub mod setup;
pub mod simulate;
pub mod theme;
pub mod viewport;

use std::path::{Path, PathBuf};

use campanion_view::Scene;

pub use session::{
    GUI_PREVIEW_NOTICE, GUI_PREVIEW_NOTICE_SHORT, JobSession, MessageKind, OpDraft, SessionMessage, ToolDraft, ToolKind, TreeSelection,
};

/// Maximum size of a file the shell will open, in bytes.
pub const MAX_OPEN_BYTES: u64 = 32 * 1024 * 1024;

/// Default chord tolerance, in millimetres, when building a preview scene.
pub const DEFAULT_ARC_TOL_MM: f64 = 0.01;

/// Rotate-step chips on the Resin Orient tab, in degrees.
pub const ROTATE_STEPS_DEG: [u32; 4] = [1, 5, 15, 45];

/// Jog step chips on the Run tab, in millimetres.
pub const JOG_STEPS_MM: [f64; 3] = [0.1, 1.0, 10.0];

/// Process family chosen by the workspace selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Workspace {
    /// Milling and routing.
    MillRoute,
    /// Lathe work.
    Turning,
    /// Laser, plasma, waterjet and oxy-fuel.
    ThermalJet,
    /// Wire EDM, with hot-wire foam planned later.
    WireCutting,
    /// Vat photopolymerization / SLA.
    Resin,
    /// Fused deposition.
    Fdm,
}

/// A parked family that the selector may show disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParkedWorkspace {
    /// User-built machines. Hidden from the active set until later.
    Custom,
}

/// A prep-stage tab on the toolbar.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ModeTab {
    /// First tab on cutting and mill workspaces: pick the sub-process.
    Process,
    /// Machine, heads and material.
    Setup,
    /// Operations, contours and toolpaths.
    Operations,
    /// Playback, G-code listing, time estimate and pre-flight checks.
    Simulate,
    /// Model pose on the build plate or bed.
    Orient,
    /// Resin supports. Not offered for FDM in this preview.
    Supports,
    /// Layer slice review.
    Slice,
    /// Job summary and export settings.
    Export,
    /// Direct send to a connected Grbl-class controller. Not hardware-tested.
    Run,
}

/// A selectable sub-process inside a workspace.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SubProcess {
    Milling,
    Routing,
    Turning,
    Laser,
    Plasma,
    Waterjet,
    OxyFuel,
    WireEdm,
    Sla,
    Fdm,
}

/// A sub-process listed but not yet available.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PlannedSubProcess {
    /// Planned under Wire Cutting.
    HotWireFoam,
}

/// Simulated job-run phase used to preview lockouts. There is no controller behind this.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RunPhase {
    /// No job has been started in this session.
    Idle,
    /// A preview "run" is in progress.
    Running,
    /// Feed hold / pause. Jog stays locked.
    Paused,
    /// The user pressed Stop. Jog is allowed.
    Stopped,
    /// The preview run reached the end. Jog is allowed.
    Finished,
}

/// A value the user typed, with optional library comparison for override styling.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct UserField {
    /// The text in the field. Empty until the user types something.
    pub value: String,
    /// Library value, when one has been saved. Never pre-filled by the program.
    pub library: Option<String>,
}

impl UserField {
    /// A field with no text and no library value.
    pub fn empty() -> Self {
        Self::default()
    }

    /// True when the typed text differs from the library value, or when text is present and there
    /// is no library value yet.
    pub fn is_overridden(&self) -> bool {
        match &self.library {
            Some(library) => self.value != *library,
            None => !self.value.is_empty(),
        }
    }

    /// Restore the library text, or clear the field when no library value exists.
    pub fn reset(&mut self) {
        self.value = self.library.clone().unwrap_or_default();
    }

    /// Store the current text as the library value. Empty text clears the library entry.
    pub fn save_to_library(&mut self) {
        if self.value.is_empty() {
            self.library = None;
        } else {
            self.library = Some(self.value.clone());
        }
    }
}

/// Why a path could not be opened.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum OpenError {
    #[error("{0} is larger than {1} bytes")]
    TooLarge(String, u64),
    #[error("{0} is not valid UTF-8")]
    NotUtf8(String),
    #[error("{0}: {1}")]
    Io(String, String),
    #[error("{0}")]
    Parse(String),
}

/// A file the shell knows how to display.
#[derive(Debug, Clone)]
pub enum OpenedDocument {
    /// A `campanion-ir` program, ready to draw.
    Ir { path: PathBuf, scene: Box<Scene>, gcode: Option<String> },
    /// A job file. Planning is not wired in this preview.
    Job { path: PathBuf, format: String },
    /// NC text to show next to the toolpath viewport.
    Gcode { path: PathBuf, text: String },
}

/// Session state for workspaces, tabs, lockouts and placeholder fields.
#[derive(Debug, Clone)]
pub struct GuiState {
    /// Active process family.
    pub workspace: Workspace,
    /// Active prep-stage tab. Always one of [`Workspace::tabs`] for `workspace`.
    pub tab: ModeTab,
    /// Selected sub-process, when the user has picked one.
    pub process: Option<SubProcess>,
    /// Run phase driven by the streamer, or by the preview when no controller is connected.
    pub run: RunPhase,
    /// Left process tree.
    pub show_tree: bool,
    /// Right properties column.
    pub show_properties: bool,
    /// Bottom timeline.
    pub show_timeline: bool,
    /// When true, the shell uses the dark theme from the mockups.
    pub dark_theme: bool,
    /// Status-bar message.
    pub status: String,
    /// Path of the last successfully opened file.
    pub open_path: Option<PathBuf>,
    /// Session-only recent files, newest first.
    pub recent: Vec<PathBuf>,
    /// Loaded G-code text for the preview pane.
    pub gcode: String,
    /// Whether an IR scene is loaded. The scene itself lives in the window.
    pub has_scene: bool,
    /// Display name of a loaded job that has not been planned.
    pub pending_job: Option<String>,
    /// Export format extension, including the leading dot. Empty until the user or a workspace sets it.
    pub export_format: String,
    /// Export file name, without directory. Starts empty.
    pub export_name: String,
    /// Directory or full path the user typed for export. Starts empty.
    pub export_path: String,
    /// Embed a thumbnail in the exported file.
    pub embed_thumbnail: bool,
    /// Typed or selected serial port on the Run tab.
    pub port: String,
    /// Typed baud field on the Run tab.
    pub baud: String,
    /// True after a controller (or the in-process mock) has been opened.
    pub connected: bool,
    /// True while the controller is in alarm. Start, Frame and jog stay locked until `$X`.
    pub in_alarm: bool,
    /// When true, stream with send-response safe mode (no RX buffer required).
    pub safe_mode: bool,
    /// User-entered frame feed, millimetres per minute. Empty until typed.
    pub frame_feed: String,
    /// User-entered jog feed, millimetres per minute. Empty until typed.
    pub jog_feed: String,
    /// Work-coordinate readout when true, machine coordinates when false.
    pub dro_work: bool,
    /// Displayed work position, millimetres. Updated only by the preview jog pad.
    pub work_pos: [f64; 3],
    /// Displayed machine position, millimetres. Stays at the origin in this preview.
    pub machine_pos: [f64; 3],
    /// Selected jog increment, millimetres.
    pub jog_step_mm: f64,
    /// Preview line cursor used for progress text.
    pub run_line: u32,
    /// Preview total lines. Zero when no program text is loaded.
    pub run_lines: u32,
    /// Selected rotate step on the Resin Orient tab, degrees.
    pub rotate_step_deg: u32,
    /// Typed model pose, millimetres and degrees. Starts empty.
    pub position_mm: [UserField; 3],
    /// Typed model rotation, degrees. Starts empty.
    pub rotation_deg: [UserField; 3],
    /// Lift from the build plate, millimetres. Starts empty.
    pub lift_mm: UserField,
    /// Array count along X. Starts empty.
    pub array_x: UserField,
    /// Array count along Y. Starts empty.
    pub array_y: UserField,
    /// Array spacing along X, millimetres. Starts empty.
    pub array_spacing_mm: UserField,
    /// Array spacing along Y, millimetres. Starts empty.
    pub array_spacing_y_mm: UserField,
    /// Minimum gap for Arrange, millimetres. Starts empty.
    pub arrange_gap_mm: UserField,
    /// Support settings. Every field starts empty; the program ships no default sizes.
    pub support: SupportFields,
    /// Current review layer, 1-based when a slice exists. Zero means "no slice".
    pub layer: u32,
    /// Number of sliced layers. Zero until a slice exists.
    pub layer_count: u32,
    /// Layer field as typed by the user, kept in sync with [`Self::layer`] when valid.
    pub layer_text: String,
    /// Selected tree path, when any.
    pub selection: Option<String>,
}

/// Placeholder support parameters. All start empty.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SupportFields {
    pub tip_diameter_mm: UserField,
    pub contact_depth_mm: UserField,
    pub shaft_diameter_mm: UserField,
    pub max_angle_deg: UserField,
    pub density_pct: UserField,
    pub scaffold_spacing_mm: UserField,
    pub raft_thickness_mm: UserField,
    /// How many supports are selected in the preview tree. Always zero until generation exists.
    pub selected_count: u32,
}

impl Default for GuiState {
    fn default() -> Self {
        Self::new()
    }
}

impl GuiState {
    /// An empty session on Thermal/Jet Cutting, first tab, dark theme, no values filled in.
    pub fn new() -> Self {
        let workspace = Workspace::ThermalJet;
        let tab = first_tab(workspace);
        Self {
            workspace,
            tab,
            process: None,
            run: RunPhase::Idle,
            show_tree: true,
            show_properties: true,
            show_timeline: true,
            dark_theme: true,
            status: "Ready".to_string(),
            open_path: None,
            recent: Vec::new(),
            gcode: String::new(),
            has_scene: false,
            pending_job: None,
            export_format: default_export_format(workspace).to_string(),
            export_name: String::new(),
            export_path: String::new(),
            embed_thumbnail: true,
            port: String::new(),
            baud: String::new(),
            connected: false,
            in_alarm: false,
            safe_mode: false,
            frame_feed: String::new(),
            jog_feed: String::new(),
            dro_work: true,
            work_pos: [0.0, 0.0, 0.0],
            machine_pos: [0.0, 0.0, 0.0],
            jog_step_mm: 1.0,
            run_line: 0,
            run_lines: 0,
            rotate_step_deg: 15,
            position_mm: [UserField::empty(), UserField::empty(), UserField::empty()],
            rotation_deg: [UserField::empty(), UserField::empty(), UserField::empty()],
            lift_mm: UserField::empty(),
            array_x: UserField::empty(),
            array_y: UserField::empty(),
            array_spacing_mm: UserField::empty(),
            array_spacing_y_mm: UserField::empty(),
            arrange_gap_mm: UserField::empty(),
            support: SupportFields::default(),
            layer: 0,
            layer_count: 0,
            layer_text: String::new(),
            selection: None,
        }
    }

    /// Switch process family and land on that family's first tab.
    pub fn set_workspace(&mut self, workspace: Workspace) {
        if self.workspace == workspace {
            return;
        }
        self.workspace = workspace;
        self.tab = first_tab(workspace);
        self.process = None;
        self.export_format = default_export_format(workspace).to_string();
        if !self.has_scene {
            self.status = format!("Workspace: {}", workspace.label());
        }
    }

    /// Switch tab when it belongs to the current workspace.
    pub fn set_tab(&mut self, tab: ModeTab) -> bool {
        if !self.workspace.has_tab(tab) {
            return false;
        }
        self.tab = tab;
        true
    }

    /// Move to the next tab in the current workspace, wrapping.
    pub fn next_tab(&mut self) {
        let tabs = self.workspace.tabs();
        if let Some(index) = tabs.iter().position(|tab| *tab == self.tab)
            && let Some(tab) = tabs.get((index + 1) % tabs.len())
        {
            self.tab = *tab;
        }
    }

    /// Move to the previous tab in the current workspace, wrapping.
    pub fn prev_tab(&mut self) {
        let tabs = self.workspace.tabs();
        if let Some(index) = tabs.iter().position(|tab| *tab == self.tab) {
            let prev = if index == 0 { tabs.len() - 1 } else { index - 1 };
            if let Some(tab) = tabs.get(prev) {
                self.tab = *tab;
            }
        }
    }

    /// Select a sub-process that belongs to the current workspace.
    pub fn set_process(&mut self, process: SubProcess) -> bool {
        if !self.workspace.sub_processes().contains(&process) {
            return false;
        }
        self.process = Some(process);
        self.status = format!("Process: {}", process.label());
        true
    }

    /// Mark the session as running. The window only calls this after a controller accepts Start.
    pub fn start_run(&mut self) -> bool {
        if !self.can_start() {
            return false;
        }
        self.run = RunPhase::Running;
        self.run_line = 0;
        self.run_lines = gcode_line_count(&self.gcode);
        self.status = "Streaming started".to_string();
        true
    }

    /// Hold a preview run. Fails when the phase is not Running.
    pub fn pause_run(&mut self) -> bool {
        if !self.can_pause() {
            return false;
        }
        self.run = RunPhase::Paused;
        self.status = "Feed hold sent".to_string();
        true
    }

    /// Resume from Pause.
    pub fn resume_run(&mut self) -> bool {
        if self.run != RunPhase::Paused {
            return false;
        }
        self.run = RunPhase::Running;
        self.status = "Resume sent".to_string();
        true
    }

    /// Stop a preview run. Always succeeds, including from Idle, so the STOP button can be pressed
    /// on every tab.
    pub fn stop_run(&mut self) {
        if !self.connected {
            self.status = "STOP is live once a controller is connected".to_string();
            return;
        }
        if self.run == RunPhase::Idle {
            self.status = "STOP sent (soft-reset)".to_string();
            return;
        }
        if self.run != RunPhase::Stopped && self.run != RunPhase::Finished {
            self.run = RunPhase::Stopped;
        }
        self.status = "STOP sent (soft-reset); last line is kept".to_string();
    }

    /// Mark the preview run complete.
    pub fn finish_run(&mut self) {
        if self.run == RunPhase::Running {
            self.run = RunPhase::Finished;
            if self.run_lines > 0 {
                self.run_line = self.run_lines;
            }
            self.status = "Job finished".to_string();
        }
    }

    /// Start is live when connected, not in alarm, and idle, stopped, or finished.
    pub fn can_start(&self) -> bool {
        self.connected && !self.in_alarm && matches!(self.run, RunPhase::Idle | RunPhase::Stopped | RunPhase::Finished)
    }

    /// Pause is live only while running.
    pub fn can_pause(&self) -> bool {
        self.run == RunPhase::Running
    }

    /// Jog, Frame, Home and Set Origin stay locked while running, paused, or in alarm.
    pub fn can_jog(&self) -> bool {
        self.connected && !self.in_alarm && matches!(self.run, RunPhase::Idle | RunPhase::Stopped | RunPhase::Finished)
    }

    /// Same lock as jog.
    pub fn can_frame(&self) -> bool {
        self.can_jog()
    }

    /// `$H` is not sent in this slice.
    pub fn can_home(&self) -> bool {
        false
    }

    /// Work-origin commands are not sent in this slice.
    pub fn can_set_origin(&self) -> bool {
        false
    }

    /// The STOP button is live on every tab once a controller is connected.
    pub fn can_stop(&self) -> bool {
        self.connected
    }

    /// Displayed readout for the current DRO mode.
    pub fn dro(&self) -> [f64; 3] {
        if self.dro_work { self.work_pos } else { self.machine_pos }
    }

    /// Apply a preview jog increment along one axis (`axis` is 0, 1 or 2). Sign is the direction.
    pub fn jog(&mut self, axis: usize, sign: i32) -> bool {
        if !self.can_jog() || axis > 2 || sign == 0 || !self.jog_step_mm.is_finite() || self.jog_step_mm <= 0.0 {
            return false;
        }
        let delta = self.jog_step_mm * f64::from(sign.signum());
        if let Some(value) = self.work_pos.get_mut(axis) {
            *value += delta;
        }
        self.status = "Jog command queued".to_string();
        true
    }

    /// Zero the work coordinates. Machine coordinates are left alone.
    pub fn set_origin(&mut self) -> bool {
        if !self.can_set_origin() {
            return false;
        }
        self.work_pos = [0.0, 0.0, 0.0];
        self.status = "Work origin set in the preview".to_string();
        true
    }

    /// Reset displayed machine coordinates. There is no controller to home.
    pub fn home(&mut self) -> bool {
        if !self.can_home() {
            return false;
        }
        self.machine_pos = [0.0, 0.0, 0.0];
        self.status = "Home preview only; no machine connection".to_string();
        true
    }

    /// Step the slice review layer. `delta` is usually ±1. Ignored when there is no slice.
    pub fn step_layer(&mut self, delta: i32) -> bool {
        if self.layer_count == 0 {
            return false;
        }
        let current = i64::from(self.layer.max(1));
        let next = (current + i64::from(delta)).clamp(1, i64::from(self.layer_count));
        let next = u32::try_from(next).unwrap_or(1);
        if next == self.layer {
            return false;
        }
        self.set_layer(next);
        true
    }

    /// Set the review layer, clamped to `1..=layer_count`.
    pub fn set_layer(&mut self, layer: u32) {
        if self.layer_count == 0 {
            self.layer = 0;
            self.layer_text.clear();
            return;
        }
        self.layer = layer.clamp(1, self.layer_count);
        self.layer_text = self.layer.to_string();
    }

    /// Parse a typed layer number. Invalid text is ignored.
    pub fn apply_layer_text(&mut self) {
        let Some(parsed) = parse_u32_field(&self.layer_text) else {
            if self.layer > 0 {
                self.layer_text = self.layer.to_string();
            }
            return;
        };
        self.set_layer(parsed);
    }

    /// Clear loaded files and return pose fields to empty, keeping the workspace.
    pub fn new_job(&mut self) {
        let workspace = self.workspace;
        let dark_theme = self.dark_theme;
        let show_tree = self.show_tree;
        let show_properties = self.show_properties;
        let show_timeline = self.show_timeline;
        *self = Self::new();
        self.workspace = workspace;
        self.tab = first_tab(workspace);
        self.export_format = default_export_format(workspace).to_string();
        self.dark_theme = dark_theme;
        self.show_tree = show_tree;
        self.show_properties = show_properties;
        self.show_timeline = show_timeline;
        self.status = "New job".to_string();
    }

    /// Record a successful open in the recent list and status line.
    pub fn remember_open(&mut self, path: PathBuf) {
        self.open_path = Some(path.clone());
        self.recent.retain(|item| item != &path);
        self.recent.insert(0, path);
        self.recent.truncate(8);
    }

    /// Reset Orient pose fields to empty (or library values when present).
    pub fn reset_orient(&mut self) {
        for field in &mut self.position_mm {
            field.reset();
        }
        for field in &mut self.rotation_deg {
            field.reset();
        }
        self.lift_mm.reset();
        self.status = "Orient reset".to_string();
    }
}

impl Workspace {
    /// Families the selector can activate. Custom is omitted.
    pub const fn all() -> [Workspace; 6] {
        [Workspace::MillRoute, Workspace::Turning, Workspace::ThermalJet, Workspace::WireCutting, Workspace::Resin, Workspace::Fdm]
    }

    /// Label used in the selector and status line.
    pub const fn label(self) -> &'static str {
        match self {
            Workspace::MillRoute => "Mill/Route",
            Workspace::Turning => "Turning",
            Workspace::ThermalJet => "Thermal/Jet Cutting",
            Workspace::WireCutting => "Wire Cutting",
            Workspace::Resin => "Resin (SLA)",
            Workspace::Fdm => "FDM",
        }
    }

    /// Compact name for the workspace card on the left of the ribbon.
    pub const fn short_label(self) -> &'static str {
        match self {
            Workspace::MillRoute => "Mill/Route",
            Workspace::Turning => "Turning",
            Workspace::ThermalJet => "Thermal/Jet",
            Workspace::WireCutting => "Wire Cutting",
            Workspace::Resin => "Resin",
            Workspace::Fdm => "FDM",
        }
    }

    /// One-line summary shown in the workspace menu. Matches the family doc comments.
    pub const fn description(self) -> &'static str {
        match self {
            Workspace::MillRoute => "Milling and routing",
            Workspace::Turning => "Lathe work",
            Workspace::ThermalJet => "Laser, plasma, waterjet and oxy-fuel",
            Workspace::WireCutting => "Wire EDM",
            Workspace::Resin => "Vat photopolymerization / SLA",
            Workspace::Fdm => "Fused deposition",
        }
    }

    /// Toolbar tabs for this family. The first entry is the landing tab.
    pub fn tabs(self) -> &'static [ModeTab] {
        match self {
            Workspace::MillRoute | Workspace::Turning | Workspace::ThermalJet | Workspace::WireCutting => {
                &[ModeTab::Process, ModeTab::Setup, ModeTab::Operations, ModeTab::Simulate, ModeTab::Export, ModeTab::Run]
            }
            Workspace::Resin => &[ModeTab::Orient, ModeTab::Supports, ModeTab::Slice, ModeTab::Export, ModeTab::Run],
            Workspace::Fdm => &[ModeTab::Orient, ModeTab::Slice, ModeTab::Export, ModeTab::Run],
        }
    }

    /// Whether `tab` is in [`Self::tabs`].
    pub fn has_tab(self, tab: ModeTab) -> bool {
        self.tabs().contains(&tab)
    }

    /// Sub-processes offered on the Process tab, or implicit for Resin and FDM.
    pub fn sub_processes(self) -> &'static [SubProcess] {
        match self {
            Workspace::MillRoute => &[SubProcess::Milling, SubProcess::Routing],
            Workspace::Turning => &[SubProcess::Turning],
            Workspace::ThermalJet => &[SubProcess::Laser, SubProcess::Plasma, SubProcess::Waterjet, SubProcess::OxyFuel],
            Workspace::WireCutting => &[SubProcess::WireEdm],
            Workspace::Resin => &[SubProcess::Sla],
            Workspace::Fdm => &[SubProcess::Fdm],
        }
    }

    /// Planned sub-processes listed as disabled.
    pub fn planned_sub_processes(self) -> &'static [PlannedSubProcess] {
        match self {
            Workspace::WireCutting => &[PlannedSubProcess::HotWireFoam],
            _ => &[],
        }
    }
}

impl ModeTab {
    /// Toolbar label.
    pub const fn label(self) -> &'static str {
        match self {
            ModeTab::Process => "Job",
            ModeTab::Setup => "Setup",
            ModeTab::Operations => "Operations",
            ModeTab::Simulate => "Simulate",
            ModeTab::Orient => "Orient",
            ModeTab::Supports => "Supports",
            ModeTab::Slice => "Slice",
            ModeTab::Export => "Export",
            ModeTab::Run => "Run",
        }
    }
}

impl SubProcess {
    /// Short label for the Process tab.
    pub const fn label(self) -> &'static str {
        match self {
            SubProcess::Milling => "Milling",
            SubProcess::Routing => "Routing",
            SubProcess::Turning => "Turning",
            SubProcess::Laser => "Laser",
            SubProcess::Plasma => "Plasma",
            SubProcess::Waterjet => "Waterjet",
            SubProcess::OxyFuel => "Oxy-fuel",
            SubProcess::WireEdm => "Wire EDM",
            SubProcess::Sla => "SLA",
            SubProcess::Fdm => "FDM",
        }
    }
}

impl PlannedSubProcess {
    /// Short label, shown disabled.
    pub const fn label(self) -> &'static str {
        match self {
            PlannedSubProcess::HotWireFoam => "Hot wire foam (planned)",
        }
    }
}

/// Landing tab for a workspace (always the first entry of [`Workspace::tabs`]).
pub fn first_tab(workspace: Workspace) -> ModeTab {
    workspace.tabs().first().copied().unwrap_or(ModeTab::Export)
}

/// Suggested export extension for a workspace. This is a file type, not a machine setting.
pub fn default_export_format(workspace: Workspace) -> &'static str {
    match workspace {
        Workspace::Resin => ".pm7m",
        Workspace::Fdm => ".gcode",
        Workspace::MillRoute | Workspace::Turning | Workspace::ThermalJet | Workspace::WireCutting => ".ngc",
    }
}

/// Count non-empty lines in G-code text.
pub fn gcode_line_count(text: &str) -> u32 {
    let count = text.lines().filter(|line| !line.trim().is_empty()).count();
    u32::try_from(count).unwrap_or(u32::MAX)
}

/// Open a file from disk, capped at [`MAX_OPEN_BYTES`].
pub fn open_path(path: &Path) -> Result<OpenedDocument, OpenError> {
    let display = path.display().to_string();
    if let Ok(meta) = std::fs::metadata(path)
        && meta.is_file()
        && meta.len() > MAX_OPEN_BYTES
    {
        return Err(OpenError::TooLarge(display, MAX_OPEN_BYTES));
    }
    let bytes = std::fs::read(path).map_err(|err| OpenError::Io(display.clone(), err.to_string()))?;
    if bytes.len() as u64 > MAX_OPEN_BYTES {
        return Err(OpenError::TooLarge(display, MAX_OPEN_BYTES));
    }
    let text = String::from_utf8(bytes).map_err(|_| OpenError::NotUtf8(display))?;
    open_text(path, &text)
}

/// Parse already-read file text.
pub fn open_text(path: &Path, text: &str) -> Result<OpenedDocument, OpenError> {
    if let Some(tag) = json_format_tag(text) {
        if tag.starts_with("campanion-ir/") {
            let scene = campanion_view::load_scene(text, DEFAULT_ARC_TOL_MM).map_err(OpenError::Parse)?;
            let gcode = sibling_gcode(path);
            return Ok(OpenedDocument::Ir { path: path.to_path_buf(), scene: Box::new(scene), gcode });
        }
        if tag.starts_with("campanion-job/") || tag.starts_with("campanion-lathe/") {
            return Ok(OpenedDocument::Job { path: path.to_path_buf(), format: tag });
        }
        return Err(OpenError::Parse(format!("unrecognised format tag `{tag}`")));
    }
    if looks_like_gcode(path, text) {
        return Ok(OpenedDocument::Gcode { path: path.to_path_buf(), text: text.to_string() });
    }
    Err(OpenError::Parse("file is not CAMpanion IR, a job file, or G-code".to_string()))
}

fn json_format_tag(text: &str) -> Option<String> {
    let value: serde_json::Value = serde_json::from_str(text).ok()?;
    value.get("format")?.as_str().map(str::to_string)
}

fn looks_like_gcode(path: &Path, text: &str) -> bool {
    if let Some(ext) = path.extension().and_then(|ext| ext.to_str()) {
        let ext = ext.to_ascii_lowercase();
        if matches!(ext.as_str(), "ngc" | "nc" | "gcode" | "tap" | "cnc") {
            return true;
        }
    }
    let trimmed = text.trim_start();
    let first = trimmed.lines().find(|line| {
        let line = line.trim();
        !line.is_empty() && !line.starts_with('(') && !line.starts_with(';')
    });
    first.is_some_and(|line| {
        let line = line.trim_start();
        line.starts_with('%')
            || line.starts_with('/')
            || starts_with_word(line, 'G')
            || starts_with_word(line, 'M')
            || starts_with_word(line, 'N')
            || starts_with_word(line, 'O')
            || starts_with_word(line, 'T')
    })
}

fn starts_with_word(line: &str, letter: char) -> bool {
    let mut chars = line.chars();
    match chars.next() {
        Some(ch) if ch.eq_ignore_ascii_case(&letter) => chars.next().is_none_or(|next| next.is_ascii_digit() || next.is_ascii_whitespace()),
        _ => false,
    }
}

fn sibling_gcode(path: &Path) -> Option<String> {
    let stem = path.file_stem()?;
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    for ext in ["ngc", "nc", "gcode"] {
        let candidate = parent.join(stem).with_extension(ext);
        if candidate == *path || !candidate.is_file() {
            continue;
        }
        if let Ok(OpenedDocument::Gcode { text, .. }) = open_path(&candidate) {
            return Some(text);
        }
    }
    None
}

fn parse_u32_field(text: &str) -> Option<u32> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return None;
    }
    trimmed.parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_tab_is_process_on_cutting_workspaces() {
        for workspace in [Workspace::MillRoute, Workspace::Turning, Workspace::ThermalJet, Workspace::WireCutting] {
            assert_eq!(first_tab(workspace), ModeTab::Process);
            assert_eq!(workspace.tabs()[0], ModeTab::Process);
            assert_eq!(workspace.tabs(), &[ModeTab::Process, ModeTab::Setup, ModeTab::Operations, ModeTab::Simulate, ModeTab::Export, ModeTab::Run]);
            assert!(workspace.has_tab(ModeTab::Export));
            assert!(workspace.has_tab(ModeTab::Run));
            assert!(!workspace.has_tab(ModeTab::Orient));
        }
    }

    #[test]
    fn resin_and_fdm_tab_sets() {
        assert_eq!(Workspace::Resin.tabs(), &[ModeTab::Orient, ModeTab::Supports, ModeTab::Slice, ModeTab::Export, ModeTab::Run]);
        assert_eq!(Workspace::Fdm.tabs(), &[ModeTab::Orient, ModeTab::Slice, ModeTab::Export, ModeTab::Run]);
        assert!(!Workspace::Fdm.has_tab(ModeTab::Supports));
        assert_eq!(first_tab(Workspace::Resin), ModeTab::Orient);
        assert_eq!(first_tab(Workspace::Fdm), ModeTab::Orient);
    }

    #[test]
    fn custom_is_parked_and_not_selectable() {
        assert!(!Workspace::all().iter().any(|workspace| workspace.label().contains("Custom")));
        let _ = ParkedWorkspace::Custom;
    }

    #[test]
    fn workspace_short_labels_and_descriptions() {
        for workspace in Workspace::all() {
            assert!(!workspace.short_label().is_empty());
            assert!(!workspace.description().is_empty());
            assert!(workspace.short_label().chars().count() <= workspace.label().chars().count());
        }
        assert_eq!(Workspace::ThermalJet.short_label(), "Thermal/Jet");
        assert_eq!(Workspace::ThermalJet.label(), "Thermal/Jet Cutting");
        assert_eq!(Workspace::Resin.short_label(), "Resin");
        assert_eq!(Workspace::Resin.label(), "Resin (SLA)");
        assert_eq!(Workspace::ThermalJet.description(), "Laser, plasma, waterjet and oxy-fuel");
    }

    #[test]
    fn set_workspace_resets_tab_and_process() {
        let mut state = GuiState::new();
        assert_eq!(state.workspace, Workspace::ThermalJet);
        assert_eq!(state.tab, ModeTab::Process);
        assert!(state.set_process(SubProcess::Laser));
        state.set_tab(ModeTab::Export);
        state.set_workspace(Workspace::Resin);
        assert_eq!(state.workspace, Workspace::Resin);
        assert_eq!(state.tab, ModeTab::Orient);
        assert_eq!(state.process, None);
        assert_eq!(state.export_format, ".pm7m");
        assert!(!state.set_tab(ModeTab::Process));
        assert!(state.set_tab(ModeTab::Slice));
    }

    #[test]
    fn tab_navigation_wraps_inside_the_workspace() {
        let mut state = GuiState::new();
        state.set_workspace(Workspace::Fdm);
        assert_eq!(state.tab, ModeTab::Orient);
        state.next_tab();
        assert_eq!(state.tab, ModeTab::Slice);
        state.next_tab();
        assert_eq!(state.tab, ModeTab::Export);
        state.next_tab();
        assert_eq!(state.tab, ModeTab::Run);
        state.next_tab();
        assert_eq!(state.tab, ModeTab::Orient);
        state.prev_tab();
        assert_eq!(state.tab, ModeTab::Run);
    }

    #[test]
    fn lockout_rules_while_running_and_paused() {
        let mut state = GuiState::new();
        assert!(!state.can_start());
        assert!(!state.can_jog());
        assert!(!state.can_stop());
        state.connected = true;
        assert!(state.can_start());
        assert!(state.can_jog());
        assert!(state.can_frame());
        assert!(!state.can_home());
        assert!(!state.can_set_origin());
        assert!(!state.can_pause());
        assert!(state.can_stop());

        assert!(state.start_run());
        assert_eq!(state.run, RunPhase::Running);
        assert!(!state.can_start());
        assert!(!state.can_jog());
        assert!(!state.can_frame());
        assert!(!state.can_home());
        assert!(!state.can_set_origin());
        assert!(state.can_pause());
        assert!(state.can_stop());
        assert!(!state.jog(0, 1));

        assert!(state.pause_run());
        assert_eq!(state.run, RunPhase::Paused);
        assert!(!state.can_start());
        assert!(!state.can_jog());
        assert!(!state.can_pause());
        assert!(state.can_stop());
        assert!(!state.jog(1, -1));
        assert!(state.resume_run());
        assert_eq!(state.run, RunPhase::Running);
    }

    #[test]
    fn jog_unlocks_after_stop_or_finish() {
        let mut state = GuiState::new();
        state.connected = true;
        assert!(state.start_run());
        state.stop_run();
        assert_eq!(state.run, RunPhase::Stopped);
        assert!(state.can_jog());
        assert!(state.can_start());
        assert!(state.jog(0, 1));
        assert_eq!(state.work_pos[0], state.jog_step_mm);

        assert!(state.start_run());
        state.finish_run();
        assert_eq!(state.run, RunPhase::Finished);
        assert!(state.can_jog());
        assert!(state.can_start());
        assert!(!state.set_origin());
        assert!(!state.home());
    }

    #[test]
    fn start_frame_and_jog_stay_locked_in_alarm() {
        let mut state = GuiState::new();
        state.connected = true;
        state.run = RunPhase::Stopped;
        state.in_alarm = true;
        assert!(!state.can_start());
        assert!(!state.can_jog());
        assert!(!state.can_frame());
        assert!(state.can_stop());
        assert!(!state.jog(0, 1));
        state.in_alarm = false;
        assert!(state.can_start());
        assert!(state.can_jog());
    }

    #[test]
    fn stop_from_idle_stays_idle() {
        let mut state = GuiState::new();
        state.stop_run();
        assert_eq!(state.run, RunPhase::Idle);
        assert!(!state.can_jog());
        state.connected = true;
        state.stop_run();
        assert_eq!(state.run, RunPhase::Idle);
        assert!(state.can_jog());
    }

    #[test]
    fn user_fields_start_empty_and_mark_overrides() {
        let mut field = UserField::empty();
        assert!(!field.is_overridden());
        field.value = "1200".to_string();
        assert!(field.is_overridden());
        field.save_to_library();
        assert!(!field.is_overridden());
        field.value = "800".to_string();
        assert!(field.is_overridden());
        field.reset();
        assert_eq!(field.value, "1200");
        assert!(!field.is_overridden());
        let state = GuiState::new();
        assert!(state.support.tip_diameter_mm.value.is_empty());
        assert!(state.support.density_pct.value.is_empty());
        assert!(state.lift_mm.value.is_empty());
        assert!(state.port.is_empty());
        assert!(state.baud.is_empty());
        assert!(state.export_name.is_empty());
        for field in &state.position_mm {
            assert!(field.value.is_empty());
        }
    }

    #[test]
    fn layer_slider_steps_and_rejects_when_unsliced() {
        let mut state = GuiState::new();
        assert!(!state.step_layer(1));
        state.layer_count = 10;
        state.set_layer(4);
        assert_eq!(state.layer, 4);
        assert_eq!(state.layer_text, "4");
        assert!(state.step_layer(1));
        assert_eq!(state.layer, 5);
        assert!(state.step_layer(-10));
        assert_eq!(state.layer, 1);
        state.layer_text = "8".to_string();
        state.apply_layer_text();
        assert_eq!(state.layer, 8);
        state.layer_text = "99".to_string();
        state.apply_layer_text();
        assert_eq!(state.layer, 10);
    }

    #[test]
    fn rotate_and_jog_step_chips() {
        assert_eq!(ROTATE_STEPS_DEG, [1, 5, 15, 45]);
        assert_eq!(JOG_STEPS_MM, [0.1, 1.0, 10.0]);
        let state = GuiState::new();
        assert!(ROTATE_STEPS_DEG.contains(&state.rotate_step_deg));
        assert!(JOG_STEPS_MM.iter().any(|step| (*step - state.jog_step_mm).abs() < 1e-12));
    }

    #[test]
    fn open_ir_and_gcode_from_text() {
        let json = include_str!("../../../crates/cli/tests/golden/sample.ir.json");
        let opened = open_text(Path::new("sample.ir.json"), json).unwrap();
        match opened {
            OpenedDocument::Ir { scene, .. } => {
                assert!(!scene.strokes.is_empty());
            }
            other => panic!("expected IR, got {other:?}"),
        }

        let gcode = "G21\nG0 X0 Y0\n";
        let opened = open_text(Path::new("part.ngc"), gcode).unwrap();
        match opened {
            OpenedDocument::Gcode { text, .. } => assert!(text.contains("G21")),
            other => panic!("expected G-code, got {other:?}"),
        }

        let job = r#"{"format":"campanion-job/1"}"#;
        let opened = open_text(Path::new("part.job.json"), job).unwrap();
        match opened {
            OpenedDocument::Job { format, .. } => assert_eq!(format, "campanion-job/1"),
            other => panic!("expected job, got {other:?}"),
        }
    }

    #[test]
    fn gcode_line_count_skips_blanks() {
        assert_eq!(gcode_line_count("G21\n\nG0 X0\n"), 2);
        assert_eq!(gcode_line_count(""), 0);
    }

    #[test]
    fn process_must_belong_to_workspace() {
        let mut state = GuiState::new();
        assert!(state.set_process(SubProcess::Laser));
        assert!(!state.set_process(SubProcess::Milling));
        state.set_workspace(Workspace::WireCutting);
        assert!(state.set_process(SubProcess::WireEdm));
        assert_eq!(state.workspace.planned_sub_processes(), &[PlannedSubProcess::HotWireFoam]);
    }
}
