//! Machines manager: create, edit, duplicate, delete, import and export user profiles.
//!
//! Numeric fields stay empty until the user types them. This module does not ship limits or
//! material values.

use campanion_cli::{
    JobMachineRef, MachineFile, MachineHead, MaterialEntry, ProfileStore, WorkArea, effective_post, load_path, profile_to_json, selected_head,
};
use eframe::egui::{self, RichText};

use crate::files::{self, DialogKind, DialogResult};
use crate::session::{JobSession, format_number};
use crate::theme;
use crate::{SubProcess, Workspace};

/// Editing state for the Machines manager window.
pub struct MachinesUi {
    store: ProfileStore,
    list: Vec<(String, String)>,
    selected: Option<String>,
    draft: Option<ProfileDraft>,
    status: String,
}

#[derive(Clone)]
struct ProfileDraft {
    id: String,
    name: String,
    units: String,
    controller: String,
    post: String,
    x_min: String,
    x_max: String,
    y_min: String,
    y_max: String,
    z_min: String,
    z_max: String,
    heads: Vec<HeadDraft>,
    head_index: usize,
    material_index: Option<usize>,
}

#[derive(Clone)]
struct HeadDraft {
    id: String,
    name: String,
    process: String,
    post: String,
    materials: Vec<MaterialDraft>,
}

#[derive(Clone)]
struct MaterialDraft {
    name: String,
    kerf: String,
    diameter: String,
    wire: String,
    depth: String,
    step_down: String,
    cut_feed: String,
    plunge_feed: String,
    power: String,
    pierce_delay: String,
    post_delay: String,
}

impl MachinesUi {
    /// Open the manager on the default user config directory.
    pub fn open_user() -> Result<Self, String> {
        let store = ProfileStore::from_env().map_err(|err| err.to_string())?;
        Ok(Self::open(store))
    }

    /// Open the manager on `store`.
    pub fn open(store: ProfileStore) -> Self {
        let mut ui = Self { store, list: Vec::new(), selected: None, draft: None, status: String::new() };
        ui.reload();
        ui
    }

    /// Reload the list from disk, keeping the current draft when its id still exists.
    pub fn reload(&mut self) {
        match self.store.list() {
            Ok(entries) => {
                let mut list = Vec::new();
                for (id, path) in entries {
                    let name = load_path(&path).ok().map(|profile| profile.name).unwrap_or_else(|| id.clone());
                    list.push((id, name));
                }
                self.list = list;
            }
            Err(err) => self.status = err.to_string(),
        }
        if let Some(id) = &self.selected
            && !self.list.iter().any(|(item, _)| item == id)
        {
            self.selected = None;
            self.draft = None;
        }
    }

    pub fn select_profile(&mut self, id: &str) {
        self.select(id);
    }

    fn select(&mut self, id: &str) {
        match self.store.load(id) {
            Ok(profile) => {
                self.selected = Some(id.to_string());
                self.draft = Some(ProfileDraft::from_file(id, &profile));
                self.status.clear();
            }
            Err(err) => self.status = err.to_string(),
        }
    }

    fn create_new(&mut self) {
        let mut index = 1;
        let mut id = "machine1".to_string();
        while self.list.iter().any(|(item, _)| item == &id) {
            index += 1;
            id = format!("machine{index}");
        }
        let profile = MachineFile::blank("New machine", "laser");
        match self.store.save(&id, &profile) {
            Ok(_) => {
                self.reload();
                self.select(&id);
                self.status = "Created an empty machine profile. Enter its limits and materials yourself.".to_string();
            }
            Err(err) => self.status = err.to_string(),
        }
    }

    fn duplicate_selected(&mut self) {
        let Some(id) = self.selected.clone() else {
            self.status = "Select a machine to duplicate".to_string();
            return;
        };
        let Some(draft) = self.draft.as_ref() else {
            return;
        };
        match draft.to_file() {
            Ok(mut profile) => {
                profile.name = format!("{} copy", profile.name);
                let new_id = unique_id(&id, &self.list);
                match self.store.save(&new_id, &profile) {
                    Ok(_) => {
                        self.reload();
                        self.select(&new_id);
                        self.status = format!("Duplicated as `{new_id}`");
                    }
                    Err(err) => self.status = err.to_string(),
                }
            }
            Err(err) => self.status = err,
        }
    }

    fn delete_selected(&mut self) {
        let Some(id) = self.selected.clone() else {
            self.status = "Select a machine to delete".to_string();
            return;
        };
        match self.store.delete(&id) {
            Ok(()) => {
                self.selected = None;
                self.draft = None;
                self.reload();
                self.status = format!("Deleted `{id}`");
            }
            Err(err) => self.status = err.to_string(),
        }
    }

    fn save_draft(&mut self) {
        let Some(draft) = self.draft.as_ref() else {
            return;
        };
        match draft.to_file() {
            Ok(profile) => match self.store.save(&draft.id, &profile) {
                Ok(_) => {
                    if self.selected.as_deref() != Some(draft.id.as_str())
                        && let Some(old) = &self.selected
                        && old != &draft.id
                    {
                        let _ = self.store.delete(old);
                    }
                    self.selected = Some(draft.id.clone());
                    self.reload();
                    self.status = "Saved".to_string();
                }
                Err(err) => self.status = err.to_string(),
            },
            Err(err) => self.status = err,
        }
    }

    fn import_file(&mut self) {
        match files::pick_path(DialogKind::Open, None, Some("machine.json")) {
            DialogResult::Picked(path) => match load_path(&path) {
                Ok(profile) => {
                    let stem = path.file_stem().and_then(|stem| stem.to_str()).unwrap_or("imported");
                    let id = unique_id(stem, &self.list);
                    match self.store.save(&id, &profile) {
                        Ok(_) => {
                            self.reload();
                            self.select(&id);
                            self.status = format!("Imported `{id}`");
                        }
                        Err(err) => self.status = err.to_string(),
                    }
                }
                Err(err) => self.status = err.to_string(),
            },
            DialogResult::Cancelled => {}
            DialogResult::Unavailable => self.status = "No system file chooser was available".to_string(),
        }
    }

    fn export_selected(&mut self) {
        let Some(draft) = self.draft.as_ref() else {
            self.status = "Select a machine to export".to_string();
            return;
        };
        let name = format!("{}.json", draft.id);
        match files::pick_path(DialogKind::SaveJob, None, Some(&name)) {
            DialogResult::Picked(path) => match draft.to_file() {
                Ok(profile) => {
                    let text = match serde_json::to_string_pretty(&profile_to_json(&profile)) {
                        Ok(text) => text,
                        Err(err) => {
                            self.status = err.to_string();
                            return;
                        }
                    };
                    match std::fs::write(&path, format!("{text}\n")) {
                        Ok(()) => self.status = format!("Exported {}", path.display()),
                        Err(err) => self.status = err.to_string(),
                    }
                }
                Err(err) => self.status = err,
            },
            DialogResult::Cancelled => {}
            DialogResult::Unavailable => self.status = "No system file chooser was available".to_string(),
        }
    }

    /// Draw the manager into `window`.
    pub fn show(&mut self, ui: &mut egui::Ui) {
        ui.label(RichText::new("Machine profiles are stored in the user config directory. Every limit and material value is entered by you; CAMpanion does not ship defaults.").color(theme::MUTED).size(11.0));
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui.button("New").clicked() {
                self.create_new();
            }
            if ui.button("Duplicate").clicked() {
                self.duplicate_selected();
            }
            if ui.button("Delete").clicked() {
                self.delete_selected();
            }
            if ui.button("Import…").clicked() {
                self.import_file();
            }
            if ui.button("Export…").clicked() {
                self.export_selected();
            }
            if ui.button("Save").clicked() {
                self.save_draft();
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.set_min_width(180.0);
                ui.label(RichText::new("Machines").strong());
                if self.list.is_empty() {
                    ui.label(RichText::new("No profiles yet").color(theme::MUTED).size(12.0));
                }
                let list = self.list.clone();
                for (id, name) in list {
                    let selected = self.selected.as_deref() == Some(id.as_str());
                    let label = if name.is_empty() { id.clone() } else { format!("{name} ({id})") };
                    if ui.selectable_label(selected, label).clicked() {
                        self.select(&id);
                    }
                }
            });
            ui.separator();
            ui.vertical(|ui| {
                if self.draft.is_some() {
                    self.edit_form(ui);
                } else {
                    ui.label(RichText::new("Select a machine, or click New.").color(theme::MUTED));
                }
            });
        });
        if !self.status.is_empty() {
            ui.separator();
            ui.label(RichText::new(&self.status).size(12.0));
        }
    }

    fn edit_form(&mut self, ui: &mut egui::Ui) {
        let Some(draft) = self.draft.as_mut() else {
            return;
        };
        ui.label(RichText::new("Profile").strong());
        labeled_field(ui, "Id", &mut draft.id);
        labeled_field(ui, "Name", &mut draft.name);
        ui.horizontal(|ui| {
            ui.label("Units");
            for units in ["mm", "inch"] {
                if ui.selectable_label(draft.units == units, units).clicked() {
                    draft.units = units.to_string();
                }
            }
        });
        labeled_field(ui, "Controller", &mut draft.controller);
        labeled_field(ui, "Post", &mut draft.post);
        ui.label(RichText::new("Work area (leave blank if you have not measured a limit)").color(theme::MUTED).size(11.0));
        ui.horizontal(|ui| {
            area_field(ui, "X min", &mut draft.x_min);
            area_field(ui, "X max", &mut draft.x_max);
        });
        ui.horizontal(|ui| {
            area_field(ui, "Y min", &mut draft.y_min);
            area_field(ui, "Y max", &mut draft.y_max);
        });
        ui.horizontal(|ui| {
            area_field(ui, "Z min", &mut draft.z_min);
            area_field(ui, "Z max", &mut draft.z_max);
        });
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(RichText::new("Heads").strong());
            if ui.button("Add head").clicked() {
                let mut index = draft.heads.len() + 1;
                let mut id = format!("head{index}");
                while draft.heads.iter().any(|head| head.id == id) {
                    index += 1;
                    id = format!("head{index}");
                }
                draft.heads.push(HeadDraft::blank(id, "laser"));
                draft.head_index = draft.heads.len() - 1;
                draft.material_index = None;
            }
        });
        let head_labels: Vec<(usize, String)> = draft
            .heads
            .iter()
            .enumerate()
            .map(|(index, head)| {
                let label = if head.name.is_empty() { head.id.clone() } else { format!("{} ({})", head.name, head.id) };
                (index, label)
            })
            .collect();
        ui.horizontal(|ui| {
            for (index, label) in head_labels {
                if ui.selectable_label(draft.head_index == index, label).clicked() {
                    draft.head_index = index;
                    draft.material_index = None;
                }
            }
        });
        if draft.head_index >= draft.heads.len() {
            return;
        }
        {
            let head = &mut draft.heads[draft.head_index];
            labeled_field(ui, "Head id", &mut head.id);
            labeled_field(ui, "Head name", &mut head.name);
            ui.horizontal(|ui| {
                ui.label("Process");
                for process in campanion_cli::PROCESS_IDS {
                    if ui.selectable_label(head.process == *process, *process).clicked() {
                        head.process = (*process).to_string();
                    }
                }
            });
            labeled_field(ui, "Head post", &mut head.post);
            ui.horizontal(|ui| {
                ui.label(RichText::new("Materials").strong());
                if ui.button("Add material").clicked() {
                    head.materials.push(MaterialDraft::blank());
                    draft.material_index = Some(head.materials.len() - 1);
                }
            });
        }
        let material_names: Vec<String> = draft.heads[draft.head_index]
            .materials
            .iter()
            .map(|item| if item.name.is_empty() { "(unnamed)".to_string() } else { item.name.clone() })
            .collect();
        ui.horizontal(|ui| {
            for (index, name) in material_names.into_iter().enumerate() {
                if ui.selectable_label(draft.material_index == Some(index), name).clicked() {
                    draft.material_index = Some(index);
                }
            }
        });
        if let Some(index) = draft.material_index
            && index < draft.heads[draft.head_index].materials.len()
        {
            let material = &mut draft.heads[draft.head_index].materials[index];
            labeled_field(ui, "Material name", &mut material.name);
            labeled_field(ui, "Kerf (mm)", &mut material.kerf);
            labeled_field(ui, "Diameter (mm)", &mut material.diameter);
            labeled_field(ui, "Wire (mm)", &mut material.wire);
            labeled_field(ui, "Depth (mm)", &mut material.depth);
            labeled_field(ui, "Step down (mm)", &mut material.step_down);
            labeled_field(ui, "Cut feed", &mut material.cut_feed);
            labeled_field(ui, "Plunge feed", &mut material.plunge_feed);
            labeled_field(ui, "Power", &mut material.power);
            labeled_field(ui, "Pierce delay (s)", &mut material.pierce_delay);
            labeled_field(ui, "Post delay (s)", &mut material.post_delay);
            if ui.button("Remove material").clicked() {
                draft.heads[draft.head_index].materials.remove(index);
                draft.material_index = None;
            }
        }
        if ui.button("Remove head").clicked() && draft.heads.len() > 1 {
            draft.heads.remove(draft.head_index);
            draft.head_index = draft.head_index.min(draft.heads.len() - 1);
            draft.material_index = None;
        }
    }
}

fn labeled_field(ui: &mut egui::Ui, label: &str, value: &mut String) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::TextEdit::singleline(value).desired_width(180.0).hint_text(""));
    });
}

fn area_field(ui: &mut egui::Ui, label: &str, value: &mut String) {
    ui.label(label);
    ui.add(egui::TextEdit::singleline(value).desired_width(70.0).hint_text(""));
}

fn unique_id(base: &str, list: &[(String, String)]) -> String {
    if !list.iter().any(|(id, _)| id == base) && campanion_cli::valid_profile_id(base) {
        return base.to_string();
    }
    let stem = if campanion_cli::valid_profile_id(base) { base.to_string() } else { "machine".to_string() };
    for index in 2..10_000 {
        let candidate = format!("{stem}-{index}");
        if !list.iter().any(|(id, _)| id == &candidate) {
            return candidate;
        }
    }
    format!("{stem}-copy")
}

impl ProfileDraft {
    fn from_file(id: &str, profile: &MachineFile) -> Self {
        let area = profile.work_area;
        let heads: Vec<HeadDraft> = profile.heads.iter().map(HeadDraft::from_head).collect();
        Self {
            id: id.to_string(),
            name: profile.name.clone(),
            units: profile.units.clone(),
            controller: profile.controller.clone().unwrap_or_default(),
            post: profile.post.clone().unwrap_or_default(),
            x_min: fmt_opt(area.and_then(|area| area.x_min)),
            x_max: fmt_opt(area.and_then(|area| area.x_max)),
            y_min: fmt_opt(area.and_then(|area| area.y_min)),
            y_max: fmt_opt(area.and_then(|area| area.y_max)),
            z_min: fmt_opt(area.and_then(|area| area.z_min)),
            z_max: fmt_opt(area.and_then(|area| area.z_max)),
            heads,
            head_index: 0,
            material_index: None,
        }
    }

    fn to_file(&self) -> Result<MachineFile, String> {
        if !campanion_cli::valid_profile_id(&self.id) {
            return Err("profile id must start with a letter or digit and contain only letters, digits, '.', '_' or '-'".to_string());
        }
        if self.name.trim().is_empty() {
            return Err("enter a machine name".to_string());
        }
        if self.heads.is_empty() {
            return Err("a machine needs at least one head".to_string());
        }
        let work_area = parse_work_area(self)?;
        let heads: Result<Vec<_>, _> = self.heads.iter().map(HeadDraft::to_head).collect();
        Ok(MachineFile {
            name: self.name.trim().to_string(),
            units: if self.units == "inch" { "inch".to_string() } else { "mm".to_string() },
            work_area,
            controller: nonempty(&self.controller),
            post: nonempty(&self.post),
            heads: heads?,
        })
    }
}

fn parse_work_area(draft: &ProfileDraft) -> Result<Option<WorkArea>, String> {
    let area = WorkArea {
        x_min: parse_opt(&draft.x_min, "X min")?,
        x_max: parse_opt(&draft.x_max, "X max")?,
        y_min: parse_opt(&draft.y_min, "Y min")?,
        y_max: parse_opt(&draft.y_max, "Y max")?,
        z_min: parse_opt(&draft.z_min, "Z min")?,
        z_max: parse_opt(&draft.z_max, "Z max")?,
    };
    if area.x_min.is_none() && area.x_max.is_none() && area.y_min.is_none() && area.y_max.is_none() && area.z_min.is_none() && area.z_max.is_none() {
        return Ok(None);
    }
    Ok(Some(area))
}

impl HeadDraft {
    fn blank(id: String, process: &str) -> Self {
        Self { id, name: String::new(), process: process.to_string(), post: String::new(), materials: Vec::new() }
    }

    fn from_head(head: &MachineHead) -> Self {
        Self {
            id: head.id.clone(),
            name: head.name.clone(),
            process: head.process.clone(),
            post: head.post.clone().unwrap_or_default(),
            materials: head.materials.iter().map(MaterialDraft::from_entry).collect(),
        }
    }

    fn to_head(&self) -> Result<MachineHead, String> {
        if self.id.trim().is_empty() {
            return Err("head id is empty".to_string());
        }
        if !campanion_cli::PROCESS_IDS.contains(&self.process.as_str()) {
            return Err(format!("unknown process `{}`", self.process));
        }
        let materials: Result<Vec<_>, _> = self.materials.iter().map(MaterialDraft::to_entry).collect();
        Ok(MachineHead {
            id: self.id.trim().to_string(),
            name: self.name.trim().to_string(),
            process: self.process.clone(),
            post: nonempty(&self.post),
            materials: materials?,
        })
    }
}

impl MaterialDraft {
    fn blank() -> Self {
        Self {
            name: String::new(),
            kerf: String::new(),
            diameter: String::new(),
            wire: String::new(),
            depth: String::new(),
            step_down: String::new(),
            cut_feed: String::new(),
            plunge_feed: String::new(),
            power: String::new(),
            pierce_delay: String::new(),
            post_delay: String::new(),
        }
    }

    fn from_entry(entry: &MaterialEntry) -> Self {
        Self {
            name: entry.name.clone(),
            kerf: fmt_opt(entry.kerf),
            diameter: fmt_opt(entry.diameter),
            wire: fmt_opt(entry.wire),
            depth: fmt_opt(entry.depth),
            step_down: fmt_opt(entry.step_down),
            cut_feed: fmt_opt(entry.cut_feed),
            plunge_feed: fmt_opt(entry.plunge_feed),
            power: fmt_opt(entry.power),
            pierce_delay: fmt_opt(entry.pierce_delay),
            post_delay: fmt_opt(entry.post_delay),
        }
    }

    fn to_entry(&self) -> Result<MaterialEntry, String> {
        if self.name.trim().is_empty() {
            return Err("material name is empty".to_string());
        }
        Ok(MaterialEntry {
            name: self.name.trim().to_string(),
            kerf: parse_opt(&self.kerf, "kerf")?,
            diameter: parse_opt(&self.diameter, "diameter")?,
            wire: parse_opt(&self.wire, "wire")?,
            depth: parse_opt(&self.depth, "depth")?,
            step_down: parse_opt(&self.step_down, "step_down")?,
            cut_feed: parse_opt(&self.cut_feed, "cut_feed")?,
            plunge_feed: parse_opt(&self.plunge_feed, "plunge_feed")?,
            power: parse_opt(&self.power, "power")?,
            pierce_delay: parse_opt(&self.pierce_delay, "pierce_delay")?,
            post_delay: parse_opt(&self.post_delay, "post_delay")?,
        })
    }
}

fn fmt_opt(value: Option<f64>) -> String {
    value.map(format_number).unwrap_or_default()
}

fn parse_opt(text: &str, what: &str) -> Result<Option<f64>, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Ok(None);
    }
    let value: f64 = trimmed.parse().map_err(|_| format!("{what} is not a number"))?;
    if !value.is_finite() {
        return Err(format!("{what} is not finite"));
    }
    Ok(Some(value))
}

fn nonempty(text: &str) -> Option<String> {
    let trimmed = text.trim();
    if trimmed.is_empty() { None } else { Some(trimmed.to_string()) }
}

/// Map a profile process id to the GUI process and workspace.
#[must_use]
pub fn process_from_id(process: &str) -> Option<(Workspace, SubProcess)> {
    match process {
        "milling" => Some((Workspace::MillRoute, SubProcess::Milling)),
        "routing" => Some((Workspace::MillRoute, SubProcess::Routing)),
        "turning" => Some((Workspace::Turning, SubProcess::Turning)),
        "laser" => Some((Workspace::ThermalJet, SubProcess::Laser)),
        "plasma" => Some((Workspace::ThermalJet, SubProcess::Plasma)),
        "waterjet" => Some((Workspace::ThermalJet, SubProcess::Waterjet)),
        "oxyfuel" => Some((Workspace::ThermalJet, SubProcess::OxyFuel)),
        "wire_edm" => Some((Workspace::WireCutting, SubProcess::WireEdm)),
        "sla" => Some((Workspace::Resin, SubProcess::Sla)),
        "fdm" => Some((Workspace::Fdm, SubProcess::Fdm)),
        _ => None,
    }
}

/// Copy user-entered material fields into the selected operation and its tool.
pub fn apply_material(session: &mut JobSession, entry: &MaterialEntry) -> Result<String, String> {
    let Some(crate::session::TreeSelection::Operation(index)) = session.selected else {
        return Err("select an operation before applying a material".to_string());
    };
    let op = session.operations.get_mut(index).ok_or_else(|| "no operation is selected".to_string())?;
    if let Some(value) = entry.depth {
        op.depth = format_number(value);
    }
    if let Some(value) = entry.step_down {
        op.step_down = format_number(value);
    }
    if let Some(value) = entry.cut_feed {
        op.cut_feed = format_number(value);
    }
    if let Some(value) = entry.plunge_feed {
        op.plunge_feed = format_number(value);
    }
    if let Some(value) = entry.power {
        op.power = format_number(value);
    }
    if let Some(value) = entry.pierce_delay {
        op.pierce_delay = format_number(value);
    }
    if let Some(value) = entry.post_delay {
        op.post_delay = format_number(value);
    }
    let tool_id = op.tool_id;
    if let Some(tool) = session.tools.iter_mut().find(|tool| tool.id == tool_id) {
        if let Some(value) = entry.kerf {
            tool.kind = crate::session::ToolKind::Kerf;
            tool.size = format_number(value);
        } else if let Some(value) = entry.diameter {
            tool.kind = crate::session::ToolKind::Diameter;
            tool.size = format_number(value);
        } else if let Some(value) = entry.wire {
            tool.kind = crate::session::ToolKind::Wire;
            tool.size = format_number(value);
        }
    }
    session.dirty = true;
    Ok(format!("Applied material `{}`", entry.name))
}

/// Load the profile a job session currently names, if any.
pub fn session_profile(store: &ProfileStore, session: &JobSession) -> Result<Option<(MachineFile, MachineHead)>, String> {
    if session.machine_profile.is_empty() && session.machine_path.is_empty() {
        return Ok(None);
    }
    let reference = JobMachineRef {
        profile: if session.machine_profile.is_empty() { None } else { Some(session.machine_profile.clone()) },
        path: if session.machine_path.is_empty() { None } else { Some(std::path::PathBuf::from(&session.machine_path)) },
        head: if session.machine_head.is_empty() { None } else { Some(session.machine_head.clone()) },
    };
    let profile = campanion_cli::load_job_ref(&reference, store).map_err(|err| err.to_string())?;
    let head = selected_head(&profile, &reference).map_err(|err| err.to_string())?.clone();
    Ok(Some((profile, head)))
}

/// Post suggested by the selected head, when the user has entered one.
pub fn session_post(store: &ProfileStore, session: &JobSession) -> Option<String> {
    let (profile, head) = session_profile(store, session).ok().flatten()?;
    effective_post(&profile, &head)
}

/// A profile used only to populate a screenshot of the manager. It is not a shipped default.
pub fn screenshot_example(name: &str) -> MachineFile {
    MachineFile {
        name: name.to_string(),
        units: "mm".to_string(),
        work_area: Some(WorkArea { x_min: Some(0.0), x_max: Some(600.0), y_min: Some(0.0), y_max: Some(400.0), z_min: None, z_max: None }),
        controller: Some("grbl".to_string()),
        post: Some("grbl_laser".to_string()),
        heads: vec![MachineHead {
            id: "laser".to_string(),
            name: "CO2".to_string(),
            process: "laser".to_string(),
            post: Some("grbl_laser".to_string()),
            materials: vec![MaterialEntry {
                name: "Example: 3 mm acrylic (screenshot only)".to_string(),
                kerf: Some(0.15),
                diameter: None,
                wire: None,
                depth: Some(3.0),
                step_down: Some(3.0),
                cut_feed: Some(300.0),
                plunge_feed: None,
                power: Some(0.8),
                pierce_delay: None,
                post_delay: None,
            }],
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::{OpDraft, ToolDraft, ToolKind, TreeSelection};

    #[test]
    fn apply_material_fills_only_entered_fields() {
        let mut session = JobSession::new();
        session.tools.push(ToolDraft { id: 1, name: String::new(), kind: ToolKind::Kerf, size: String::new() });
        session.operations.push(OpDraft::new_profile("outline", 1));
        session.selected = Some(TreeSelection::Operation(0));
        let entry =
            MaterialEntry { name: "acrylic".to_string(), kerf: Some(0.2), cut_feed: Some(250.0), power: Some(0.5), ..MaterialEntry::default() };
        apply_material(&mut session, &entry).expect("apply");
        assert_eq!(session.operations[0].cut_feed, "250");
        assert_eq!(session.operations[0].power, "0.5");
        assert!(session.operations[0].depth.is_empty());
        assert!(session.operations[0].pierce_delay.is_empty());
        assert_eq!(session.tools[0].size, "0.2");
        assert_eq!(session.tools[0].kind, ToolKind::Kerf);
    }

    #[test]
    fn process_from_id_maps_laser_and_rejects_unknown() {
        assert_eq!(process_from_id("laser"), Some((Workspace::ThermalJet, SubProcess::Laser)));
        assert_eq!(process_from_id("spark"), None);
    }
}
