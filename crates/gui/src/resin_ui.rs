//! Resin Orient / Supports / Slice / Export helpers.
//!
//! Labels include units. Disabled controls always say why and what to enter. There are no shipped
//! numeric defaults. Palettes stay empty until the user saves a preset. The optimizer stays
//! disabled until a complete calibration profile is loaded.

use campanion_mesh::{Aabb, Mesh, Vec3};
use campanion_resin::{
    BracingParams, BuildReport, CalibrationProfile, CancelFlag, DetectedFeatures, FaceZones, GenerateOptions, GenerateProgress, InputPoint,
    IslandDetectParams, M7_MAX, MeshPose, MissingConstant, Placement, PointSource, PresetMix, ResinSettings, SliceIslands, SolidOptions,
    SupportParams, SupportPoint, SupportRole, SupportedMesh, TipGeometry, TipPreset, ZoneMark, apply_mesh_pose, build_pm7m, generate_supports_with,
    load_print_settings, parse_calibration, parse_presets, rebuild_supported_with, snap_to_step, write_presets,
};
use campanion_view::{BuildVolume, MeshRole, PointMarker, Scene3, ShadedMesh};
use campanion_view::{Scene, Stroke, StrokeKind};
use eframe::egui::{self, Color32, RichText, Stroke as UiStroke};

use crate::theme;
use crate::{SupportFields, UserField};

/// How the Supports viewport interprets a click.
/// Camera used in the Supports viewport.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SupportView {
    /// Looking down the Z axis (build-plate plan).
    #[default]
    Top,
    /// Looking along Y, X horizontal, Z up.
    Front,
    /// Looking along X, Y horizontal, Z up.
    Side,
    /// Low side angle so pillar heights and mid-air islands read in depth.
    ThreeQuarter,
}

impl SupportView {
    pub fn label(self) -> &'static str {
        match self {
            Self::Top => "Top",
            Self::Front => "Front",
            Self::Side => "Side",
            Self::ThreeQuarter => "Iso",
        }
    }

    pub fn caption(self) -> &'static str {
        match self {
            Self::Top => "Build plate, top view — supports and islands",
            Self::Front => "Front view (X–Z) — pillar heights",
            Self::Side => "Side view (Y–Z) — pillar heights",
            Self::ThreeQuarter => "Isometric view — orbit, pan and zoom",
        }
    }

    pub fn allows_edit(self) -> bool {
        true
    }

    pub fn standard(self) -> campanion_view::StandardView {
        match self {
            Self::Top => campanion_view::StandardView::Top,
            Self::Front => campanion_view::StandardView::Front,
            Self::Side => campanion_view::StandardView::Side,
            Self::ThreeQuarter => campanion_view::StandardView::Iso,
        }
    }

    pub fn from_standard(view: campanion_view::StandardView) -> Self {
        match view {
            campanion_view::StandardView::Top => Self::Top,
            campanion_view::StandardView::Front => Self::Front,
            campanion_view::StandardView::Side => Self::Side,
            campanion_view::StandardView::Iso => Self::ThreeQuarter,
        }
    }

    pub fn project(self, x: f64, y: f64, z: f64) -> [f64; 2] {
        match self {
            Self::Top => [x, y],
            Self::Front => [x, z],
            Self::Side => [y, z],
            Self::ThreeQuarter => project_three_quarter(x, y, z),
        }
    }
}

fn project_three_quarter(x: f64, y: f64, z: f64) -> [f64; 2] {
    // Same camera basis as render_supported_png: look from a low side angle, slightly upward.
    let look = normalize3(-1.35, 1.7, 0.22);
    let (lx, ly, lz) = look;
    let (rx, ry, rz) = normalize3(ly * 1.0 - lz * 0.0, lz * 0.0 - lx * 1.0, lx * 0.0 - ly * 0.0); // look × world_up (0,0,1)
    let (ux, uy, uz) = normalize3(ry * lz - rz * ly, rz * lx - rx * lz, rx * ly - ry * lx); // right × look
    [x * rx + y * ry + z * rz, x * ux + y * uy + z * uz]
}

fn normalize3(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let n = (x * x + y * y + z * z).sqrt();
    if n > 0.0 { (x / n, y / n, z / n) } else { (0.0, 0.0, 1.0) }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SupportEditTool {
    /// Click empty space to add, drag a point to move, Delete removes the selection.
    #[default]
    Direct,
    Add,
    Move,
    Delete,
    /// Left-drag paints faces that must get supports.
    PaintSupport,
    /// Left-drag paints faces that must not get automatic supports.
    PaintBlock,
    /// Left-drag clears paint.
    ErasePaint,
}

impl SupportEditTool {
    /// The mark a paint tool applies, if this is a paint tool.
    pub fn paint_mark(self) -> Option<ZoneMark> {
        match self {
            Self::PaintSupport => Some(ZoneMark::Support),
            Self::PaintBlock => Some(ZoneMark::Block),
            Self::ErasePaint => Some(ZoneMark::None),
            _ => None,
        }
    }
}

/// A user-saved tip geometry preset. Empty until the user saves one. Capacities live in calibration.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SavedTipPreset {
    pub name: String,
    pub tip_diameter_mm: String,
    pub tip_length_mm: String,
    pub pillar_diameter_mm: String,
    pub foot_diameter_mm: String,
    pub foot_height_mm: String,
    pub tip_penetration_mm: String,
}

/// Loaded calibration, or a documented incomplete parse.
#[derive(Clone, Debug)]
pub struct CalibrationState {
    pub path: std::path::PathBuf,
    pub profile: Option<CalibrationProfile>,
    pub missing: Vec<MissingConstant>,
    pub error: Option<String>,
}

impl CalibrationState {
    pub fn is_complete(&self) -> bool {
        self.profile.as_ref().is_some_and(CalibrationProfile::is_complete)
    }

    pub fn status_line(&self) -> String {
        if let Some(error) = &self.error {
            return format!("{}: {error}", self.path.display());
        }
        if self.is_complete() {
            let name = self.profile.as_ref().map(|p| p.name.as_str()).unwrap_or("profile");
            return format!("{} (`{name}`): all required constants are set", self.path.display());
        }
        let list = self.missing.iter().map(|item| format!("{} ({})", item.label(), item.reason)).collect::<Vec<_>>().join("; ");
        format!("{}: missing {list}", self.path.display())
    }
}

/// Session state for the resin workspace.
#[derive(Clone, Debug, Default)]
pub struct ResinSession {
    pub mesh: Option<Mesh>,
    pub mesh_path: Option<std::path::PathBuf>,
    pub points: Vec<SupportPoint>,
    pub features: Option<DetectedFeatures>,
    pub selected_point: Option<usize>,
    pub selected_island: Option<u32>,
    pub tool: SupportEditTool,
    pub dragging: Option<usize>,
    /// User-saved tip presets, loaded from and saved to [`Self::presets_path`].
    pub presets: Vec<TipPreset>,
    pub presets_path: Option<std::path::PathBuf>,
    pub presets_error: Option<String>,
    /// Which saved preset each kind of support uses. Empty uses the Tip and pillar fields.
    pub mix: PresetMix,
    pub preset_draft: SavedTipPreset,
    /// Painted support / no-support faces, one mark per part triangle.
    pub zones: Option<FaceZones>,
    /// Brush position and on-screen radius while a paint tool hovers the part.
    pub brush_hover: Option<(Vec3, f32)>,
    /// Islands in the sliced layers, from the exact images written.
    pub slice_islands: SliceIslands,
    pub pixel_area_mm2: f64,
    /// Cached preview of the last drawn layer: (index, width, height, rgba).
    pub layer_cache: Option<(usize, LayerPreview)>,
    /// Scroll the Supports panel to this section on the next frame (used for screenshots).
    pub scroll_to: Option<&'static str>,
    pub calibration: Option<CalibrationState>,
    pub supported: Option<SupportedMesh>,
    pub last_generate_note: Option<String>,
    pub warnings: Vec<String>,
    pub optimize_summary: Option<String>,
    pub sliced: bool,
    pub advanced_open: bool,
    pub view: SupportView,
    pub params_from: Option<std::path::PathBuf>,
    pub settings: Option<ResinSettings>,
    pub settings_error: Option<String>,
    pub pm7m: Option<Vec<u8>>,
    pub slice_report: Option<BuildReport>,
    pub layer_images: Vec<Vec<u8>>,
    pub slice_width: u32,
    pub slice_height: u32,
    pub points_dirty: bool,
    pub last_export_path: Option<std::path::PathBuf>,
    /// Cached shaded solids (part, raft, pillars, island boxes). Markers are rebuilt each frame.
    pub solid_meshes: Vec<ShadedMesh>,
    /// Pose applied to [`Self::mesh`] for the viewport, generate, and slice.
    pub pose: MeshPose,
    /// Cached result of applying [`Self::pose`] to [`Self::mesh`].
    posed_cache: Option<Mesh>,
    /// Parameters and solid options the current [`Self::supported`] was built with.
    pub built_with: Option<(SupportParams, SolidOptions)>,
}

/// Snapshot for resin undo (points and selection only).
#[derive(Clone, Debug)]
pub struct ResinSnapshot {
    pub points: Vec<SupportPoint>,
    pub selected_point: Option<usize>,
    pub zones: Option<FaceZones>,
}

impl ResinSession {
    pub fn snapshot(&self) -> ResinSnapshot {
        ResinSnapshot { points: self.points.clone(), selected_point: self.selected_point, zones: self.zones.clone() }
    }

    pub fn restore(&mut self, snapshot: ResinSnapshot) {
        let zones_changed = snapshot.zones != self.zones;
        self.points = snapshot.points;
        self.selected_point = snapshot.selected_point;
        self.zones = snapshot.zones;
        self.dragging = None;
        self.points_dirty = true;
        if zones_changed {
            self.invalidate_solids();
        }
        self.invalidate_slice();
    }

    /// `(support faces, no-support faces)` painted so far.
    pub fn zone_counts(&self) -> (usize, usize) {
        self.zones.as_ref().map(FaceZones::counts).unwrap_or((0, 0))
    }

    /// Paint at `center` with the current tool. Returns faces changed.
    pub fn paint_at(&mut self, center: Vec3, radius_mm: f64, mark: ZoneMark, view_dir: Option<Vec3>) -> Result<usize, String> {
        // Paint on the mesh shown in the viewport so the brush lands where the pointer is.
        let mesh = match self.supported.as_ref() {
            Some(supported) => supported.part.clone(),
            None => self.posed_part()?.clone(),
        };
        let count = mesh.triangles().len();
        let zones = self.zones.get_or_insert_with(|| FaceZones::new(count));
        if zones.triangle_count() != count {
            *zones = FaceZones::new(count);
        }
        let changed = zones.paint(&mesh, center, radius_mm, mark, view_dir).map_err(|err| err.to_string())?;
        if changed > 0 {
            self.refresh_part_meshes();
            self.invalidate_slice();
        }
        Ok(changed)
    }

    /// Clear every painted face.
    pub fn clear_paint(&mut self) {
        if let Some(zones) = self.zones.as_mut() {
            zones.clear();
        }
        self.refresh_part_meshes();
        self.invalidate_slice();
    }

    /// Geometry and names the current points' preset indices refer to.
    pub fn points_palette(&self) -> (Vec<TipGeometry>, Vec<String>) {
        if let Some(supported) = &self.supported
            && !supported.palette.is_empty()
        {
            return (supported.palette.clone(), supported.palette_names.clone());
        }
        (self.presets.iter().map(TipGeometry::from).collect(), self.presets.iter().map(|p| p.name.clone()).collect())
    }

    /// True when the points refer to the calibration palette (after Optimize), not to saved presets.
    pub fn palette_is_calibration(&self) -> bool {
        self.supported.as_ref().is_some_and(|supported| supported.optimize.is_some())
    }

    /// Load saved presets from `path`. A missing file means no presets yet.
    pub fn load_presets(&mut self, path: std::path::PathBuf) {
        self.presets_error = None;
        match std::fs::read_to_string(&path) {
            Ok(text) => match parse_presets(&text) {
                Ok(presets) => self.presets = presets,
                Err(err) => self.presets_error = Some(format!("{}: {err}", path.display())),
            },
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => self.presets.clear(),
            Err(err) => self.presets_error = Some(format!("{}: {err}", path.display())),
        }
        self.presets_path = Some(path);
    }

    /// Write presets to [`Self::presets_path`].
    pub fn save_presets(&mut self) -> Result<(), String> {
        let Some(path) = self.presets_path.clone() else {
            return Err("no preset file location is known".to_string());
        };
        let text = write_presets(&self.presets).map_err(|err| err.to_string())?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
        }
        std::fs::write(&path, text).map_err(|err| format!("{}: {err}", path.display()))
    }

    /// Add or replace a preset by name, then save.
    pub fn store_preset(&mut self, preset: TipPreset) -> Result<(), String> {
        preset.validate().map_err(|err| err.to_string())?;
        match self.presets.iter_mut().find(|p| p.name == preset.name) {
            Some(existing) => *existing = preset,
            None => {
                if self.presets.len() >= campanion_resin::supports::MAX_PRESETS {
                    return Err(format!("at most {} presets can be saved", campanion_resin::supports::MAX_PRESETS));
                }
                self.presets.push(preset);
            }
        }
        self.save_presets()
    }

    /// Remove a preset, fix up point indices and the mix, then save.
    pub fn remove_preset(&mut self, index: usize) -> Result<(), String> {
        if index >= self.presets.len() {
            return Err("that preset does not exist".to_string());
        }
        let removed = self.presets.remove(index);
        if !self.palette_is_calibration()
            && let Ok(index) = u16::try_from(index)
        {
            campanion_resin::forget_preset(&mut self.points, index);
            self.points_dirty = true;
            if let Some(supported) = self.supported.as_mut() {
                supported.palette = self.presets.iter().map(TipGeometry::from).collect();
                supported.palette_names = self.presets.iter().map(|p| p.name.clone()).collect();
            }
            self.invalidate_slice();
        }
        for role in SupportRole::ALL {
            if self.mix.get(role) == Some(removed.name.as_str()) {
                self.mix.set(role, None);
            }
        }
        self.save_presets()
    }

    pub fn has_part(&self) -> bool {
        self.mesh.is_some()
    }

    /// Drop the posed-mesh cache. Call when [`Self::pose`] or [`Self::mesh`] changes.
    pub fn invalidate_pose(&mut self) {
        self.posed_cache = None;
        self.supported = None;
        self.features = None;
        self.points.clear();
        self.selected_point = None;
        self.dragging = None;
        self.points_dirty = false;
        self.last_generate_note = None;
        self.optimize_summary = None;
        self.invalidate_solids();
        self.invalidate_slice();
    }

    /// The imported mesh after [`Self::pose`] is applied.
    pub fn posed_part(&mut self) -> Result<&Mesh, String> {
        if self.posed_cache.is_none() {
            let source = self.mesh.as_ref().ok_or_else(|| "open a part first".to_string())?;
            let posed = apply_mesh_pose(source, self.pose).map_err(|err| err.to_string())?;
            self.posed_cache = Some(posed);
        }
        self.posed_cache.as_ref().ok_or_else(|| "open a part first".to_string())
    }

    /// The mesh drawn in the viewport: the posed part when the cache is warm, otherwise the import.
    #[must_use]
    pub fn display_mesh(&self) -> Option<&Mesh> {
        self.posed_cache.as_ref().or(self.mesh.as_ref())
    }

    pub fn has_print_settings(&self) -> bool {
        self.settings.is_some()
    }

    pub fn invalidate_slice(&mut self) {
        self.slice_islands = SliceIslands::default();
        self.layer_cache = None;
        self.sliced = false;
        self.pm7m = None;
        self.slice_report = None;
        self.layer_images.clear();
        self.slice_width = 0;
        self.slice_height = 0;
    }
}

/// Parse a required millimetre or degree field. Empty is missing, not zero.
pub fn parse_required(field: &UserField, label: &str) -> Result<f64, String> {
    let text = field.value.trim();
    if text.is_empty() {
        return Err(label.to_string());
    }
    text.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{label} (not a number)"))
}

pub fn parse_optional(field: &UserField, label: &str) -> Result<Option<f64>, String> {
    let text = field.value.trim();
    if text.is_empty() {
        return Ok(None);
    }
    let value = text.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{label} (not a number)"))?;
    Ok(Some(value))
}

/// Read Orient position and rotation fields. Empty means "as imported" on that axis.
pub fn pose_from_fields(position_mm: &[UserField; 3], rotation_deg: &[UserField; 3]) -> Result<MeshPose, String> {
    Ok(MeshPose {
        position_mm: [
            parse_optional(&position_mm[0], "position X (mm)")?,
            parse_optional(&position_mm[1], "position Y (mm)")?,
            parse_optional(&position_mm[2], "position Z (mm)")?,
        ],
        rotation_deg: [
            parse_optional(&rotation_deg[0], "rotation X (deg)")?,
            parse_optional(&rotation_deg[1], "rotation Y (deg)")?,
            parse_optional(&rotation_deg[2], "rotation Z (deg)")?,
        ],
    })
}

/// Write a pose back into the Orient fields. `None` axes stay empty.
pub fn write_pose_fields(pose: MeshPose, position_mm: &mut [UserField; 3], rotation_deg: &mut [UserField; 3]) {
    for (field, value) in position_mm.iter_mut().zip(pose.position_mm) {
        field.value = value.map(format_pose_value).unwrap_or_default();
    }
    for (field, value) in rotation_deg.iter_mut().zip(pose.rotation_deg) {
        field.value = value.map(format_pose_value).unwrap_or_default();
    }
}

fn format_pose_value(value: f64) -> String {
    if (value - value.round()).abs() < 1e-9 { format!("{:.0}", value.round()) } else { format!("{value:.3}") }
}

/// Apply the typed pose to `session`. No-op when the pose is unchanged.
pub fn apply_pose_fields(session: &mut ResinSession, position_mm: &[UserField; 3], rotation_deg: &[UserField; 3]) -> Result<(), String> {
    let pose = pose_from_fields(position_mm, rotation_deg)?;
    if session.pose != pose {
        session.pose = pose;
        session.invalidate_pose();
    }
    Ok(())
}

/// Centre the part's bounding box on the plate origin (X = 0, Y = 0).
pub fn center_session_on_plate(
    session: &mut ResinSession,
    position_mm: &mut [UserField; 3],
    rotation_deg: &mut [UserField; 3],
) -> Result<(), String> {
    if !session.has_part() {
        return Err("Center on plate is disabled: open a part first.".to_string());
    }
    session.pose.position_mm[0] = Some(0.0);
    session.pose.position_mm[1] = Some(0.0);
    session.invalidate_pose();
    write_pose_fields(session.pose, position_mm, rotation_deg);
    Ok(())
}

/// Lower the part so its lowest point sits at Z = 0.
pub fn drop_session_to_plate(session: &mut ResinSession, position_mm: &mut [UserField; 3], rotation_deg: &mut [UserField; 3]) -> Result<(), String> {
    let source = session.mesh.as_ref().ok_or_else(|| "Drop to plate is disabled: open a part first.".to_string())?;
    let rotated = apply_mesh_pose(source, MeshPose { position_mm: [None, None, None], rotation_deg: session.pose.rotation_deg })
        .map_err(|err| err.to_string())?;
    let bounds = rotated.bounds().map_err(|err| err.to_string())?;
    let (min, max) = (bounds.min(), bounds.max());
    session.pose.position_mm[2] = Some((max.z - min.z) / 2.0);
    session.invalidate_pose();
    write_pose_fields(session.pose, position_mm, rotation_deg);
    Ok(())
}

fn current_center(mesh: &Mesh, pose: MeshPose) -> Result<campanion_mesh::Vec3, String> {
    let posed = apply_mesh_pose(mesh, pose).map_err(|err| err.to_string())?;
    let bounds = posed.bounds().map_err(|err| err.to_string())?;
    let (min, max) = (bounds.min(), bounds.max());
    Ok(campanion_mesh::Vec3::new((min.x + max.x) / 2.0, (min.y + max.y) / 2.0, (min.z + max.z) / 2.0))
}

/// Nudge the posed centre in X and Y and snap to `step_mm`.
/// Move the bounding-box centre to snapped world `(x, y)`, keeping Z.
pub fn place_session_xy(
    session: &mut ResinSession,
    x: f64,
    y: f64,
    step_mm: f64,
    position_mm: &mut [UserField; 3],
    rotation_deg: &mut [UserField; 3],
) -> Result<(), String> {
    let source = session.mesh.as_ref().ok_or_else(|| "open a part first".to_string())?;
    let cx = current_center(source, session.pose)?;
    let x = snap_to_step(x, step_mm).map_err(|err| err.to_string())?;
    let y = snap_to_step(y, step_mm).map_err(|err| err.to_string())?;
    session.pose.position_mm[0] = Some(x);
    session.pose.position_mm[1] = Some(y);
    if session.pose.position_mm[2].is_none() {
        session.pose.position_mm[2] = Some(cx.z);
    }
    session.invalidate_pose();
    write_pose_fields(session.pose, position_mm, rotation_deg);
    Ok(())
}

pub fn translate_session_xy(
    session: &mut ResinSession,
    dx: f64,
    dy: f64,
    step_mm: f64,
    position_mm: &mut [UserField; 3],
    rotation_deg: &mut [UserField; 3],
) -> Result<(), String> {
    let source = session.mesh.as_ref().ok_or_else(|| "open a part first".to_string())?;
    let cx = current_center(source, session.pose)?;
    let x = snap_to_step(cx.x + dx, step_mm).map_err(|err| err.to_string())?;
    let y = snap_to_step(cx.y + dy, step_mm).map_err(|err| err.to_string())?;
    let mut pose = session.pose;
    pose.position_mm[0] = Some(x);
    pose.position_mm[1] = Some(y);
    if pose.position_mm[2].is_none() {
        pose.position_mm[2] = Some(cx.z);
    }
    session.pose = pose;
    session.invalidate_pose();
    write_pose_fields(session.pose, position_mm, rotation_deg);
    Ok(())
}

/// Why Center / Drop are disabled.
pub fn pose_action_disabled_reason(session: &ResinSession, action: &str) -> Option<String> {
    if session.has_part() { None } else { Some(format!("{action} is disabled: open a part (File → Import part…, STL, millimetres).")) }
}

/// Fields Generate supports needs, in the order shown on the tab.
pub fn generate_required_fields<'a>(support: &'a SupportFields, lift_mm: &'a UserField) -> [(&'static str, &'a UserField); 13] {
    [
        ("layer height (mm)", &support.layer_height_mm),
        ("support spacing (mm)", &support.support_spacing_mm),
        ("overhang angle (deg)", &support.overhang_angle_deg),
        ("tip diameter (mm)", &support.tip_diameter_mm),
        ("tip length (mm)", &support.tip_length_mm),
        ("pillar diameter (mm)", &support.pillar_diameter_mm),
        ("foot diameter (mm)", &support.foot_diameter_mm),
        ("foot height (mm)", &support.foot_height_mm),
        ("part lift (mm)", lift_mm),
        ("raft thickness (mm)", &support.raft_thickness_mm),
        ("raft chamfer angle (deg)", &support.raft_chamfer_angle_deg),
        ("raft margin (mm)", &support.raft_margin_mm),
        ("raft corner relief (mm)", &support.raft_corner_relief_mm),
    ]
}

/// Why Generate supports is disabled, and what to enter. `None` when it can run.
pub fn generate_disabled_reason(session: &ResinSession, support: &SupportFields, lift_mm: &UserField) -> Option<String> {
    if !session.has_part() {
        return Some("Generate supports is disabled: open a part (File → Import part…, STL, millimetres).".to_string());
    }
    let mut missing: Vec<&str> =
        generate_required_fields(support, lift_mm).into_iter().filter(|(_, field)| field.value.trim().is_empty()).map(|(label, _)| label).collect();
    if session.zone_counts().0 > 0 && support.zone_spacing_mm.value.trim().is_empty() {
        missing.push("painted zone spacing (mm) — support zones are painted");
    }
    if support.bracing_on {
        for (label, field) in [
            ("brace diameter (mm)", &support.brace_diameter_mm),
            ("brace max span (mm)", &support.brace_span_mm),
            ("brace pitch (mm)", &support.brace_pitch_mm),
        ] {
            if field.value.trim().is_empty() {
                missing.push(label);
            }
        }
    }
    if missing.is_empty() { None } else { Some(format!("Generate supports is disabled: enter {}.", missing.join(", "))) }
}

/// Why painting is disabled. `None` when a paint tool can be used.
pub fn paint_disabled_reason(session: &ResinSession, support: &SupportFields) -> Option<String> {
    if !session.has_part() {
        return Some("Painting is disabled: open a part (File → Import part…, STL, millimetres).".to_string());
    }
    match parse_optional(&support.brush_radius_mm, "brush radius (mm)") {
        Ok(Some(radius)) if radius > 0.0 => None,
        Ok(Some(_)) => Some("Painting is disabled: brush radius must be greater than 0 mm.".to_string()),
        Ok(None) => Some("Painting is disabled: enter a brush radius (mm) under Painted zones.".to_string()),
        Err(err) => Some(format!("Painting is disabled: {err}.")),
    }
}

/// Penetration and bracing from the form. Empty penetration and bracing off mean "not used".
pub fn solid_options_from_fields(support: &SupportFields) -> Result<SolidOptions, String> {
    let tip_penetration_mm = parse_optional(&support.tip_penetration_mm, "tip penetration (mm)")?;
    let bracing = if support.bracing_on {
        Some(BracingParams {
            diameter_mm: parse_required(&support.brace_diameter_mm, "brace diameter (mm)")?,
            max_span_mm: parse_required(&support.brace_span_mm, "brace max span (mm)")?,
            pitch_mm: parse_required(&support.brace_pitch_mm, "brace pitch (mm)")?,
        })
    } else {
        None
    };
    let options = SolidOptions { tip_penetration_mm, bracing };
    options.validate().map_err(|err| err.to_string())?;
    Ok(options)
}

/// Why Optimize supports is disabled, and what to enter.
pub fn optimize_disabled_reason(session: &ResinSession, support: &SupportFields, lift_mm: &UserField) -> Option<String> {
    if let Some(reason) = generate_disabled_reason(session, support, lift_mm) {
        return Some(reason.replace("Generate supports", "Optimize supports"));
    }
    match &session.calibration {
        None => {
            Some("Optimize supports is disabled: load a complete calibration profile (JSON). Uniform generate still works without one.".to_string())
        }
        Some(state) if !state.is_complete() => {
            let list = if state.missing.is_empty() {
                state.error.clone().unwrap_or_else(|| "the profile is incomplete".to_string())
            } else {
                state.missing.iter().map(|item| item.label()).collect::<Vec<_>>().join(", ")
            };
            Some(format!("Optimize supports is disabled: the calibration profile is incomplete. Missing: {list}."))
        }
        Some(_) => None,
    }
}

pub fn slice_disabled_reason(session: &ResinSession, machine_selected: bool) -> Option<String> {
    if !session.has_part() {
        return Some("Slice is disabled: open a part first (File → Import part…).".to_string());
    }
    if !machine_selected {
        return Some("Slice is disabled: select a resin printer in the machine selector (Manage… to add one).".to_string());
    }
    if session.settings.is_none() {
        return Some(
            "Slice is disabled: choose a sample or params .pm7m (the same file `campanion resin --params-from` uses). No exposure, lift, or layer values are assumed."
                .to_string(),
        );
    }
    None
}

pub fn export_disabled_reason(session: &ResinSession, machine_selected: bool) -> Option<String> {
    if let Some(reason) = slice_disabled_reason(session, machine_selected) {
        return Some(reason.replace("Slice is disabled", "Export .pm7m is disabled"));
    }
    if !session.sliced || session.pm7m.is_none() {
        return Some("Export .pm7m is disabled: slice the part on the Slice tab first.".to_string());
    }
    None
}

/// Collect validated support parameters from the form. All required fields must already be present.
pub fn support_params_from_fields(support: &SupportFields, lift_mm: &UserField) -> Result<SupportParams, String> {
    let num = |field: &UserField, label: &str| parse_required(field, label);
    let params = SupportParams {
        raft_chamfer_angle_deg: num(&support.raft_chamfer_angle_deg, "raft chamfer angle (deg)")?,
        raft_thickness_mm: num(&support.raft_thickness_mm, "raft thickness (mm)")?,
        raft_margin_mm: num(&support.raft_margin_mm, "raft margin (mm)")?,
        raft_corner_relief_mm: num(&support.raft_corner_relief_mm, "raft corner relief (mm)")?,
        overhang_angle_deg: num(&support.overhang_angle_deg, "overhang angle (deg)")?,
        support_spacing_mm: num(&support.support_spacing_mm, "support spacing (mm)")?,
        tip_diameter_mm: num(&support.tip_diameter_mm, "tip diameter (mm)")?,
        tip_length_mm: num(&support.tip_length_mm, "tip length (mm)")?,
        pillar_diameter_mm: num(&support.pillar_diameter_mm, "pillar diameter (mm)")?,
        foot_diameter_mm: num(&support.foot_diameter_mm, "foot diameter (mm)")?,
        foot_height_mm: num(&support.foot_height_mm, "foot height (mm)")?,
        part_lift_mm: num(lift_mm, "part lift (mm)")?,
    };
    params.validate().map_err(|err| err.to_string())?;
    Ok(params)
}

pub fn detect_params_from_fields(support: &SupportFields) -> Result<Option<IslandDetectParams>, String> {
    let Some(layer_height_mm) = parse_optional(&support.layer_height_mm, "layer height (mm)")? else {
        return Ok(None);
    };
    let cell_mm = match parse_optional(&support.detect_cell_mm, "detection cell (mm)")? {
        Some(cell) => cell,
        None => parse_required(&support.support_spacing_mm, "support spacing (mm)")?,
    };
    Ok(Some(IslandDetectParams { layer_height_mm, cell_mm }))
}

pub fn load_calibration_text(path: std::path::PathBuf, text: &str) -> CalibrationState {
    match parse_calibration(text) {
        Ok(profile) => CalibrationState { path, profile: Some(profile), missing: Vec::new(), error: None },
        Err(err) => {
            let missing = campanion_resin::missing_constants_in(text).unwrap_or_default();
            CalibrationState { path, profile: None, missing, error: Some(err.to_string()) }
        }
    }
}

/// Distinct island fill colours, cycling by island id. These are display colours, not process values.
pub fn island_color(id: u32) -> Color32 {
    const COLOURS: [Color32; 6] = [
        Color32::from_rgb(80, 200, 120),
        Color32::from_rgb(80, 160, 230),
        Color32::from_rgb(230, 180, 70),
        Color32::from_rgb(210, 90, 160),
        Color32::from_rgb(120, 210, 210),
        Color32::from_rgb(200, 120, 80),
    ];
    COLOURS[(id.saturating_sub(1) as usize) % COLOURS.len()]
}

/// Scene of triangle edges projected for `view`.
pub fn mesh_view_scene(mesh: &Mesh, view: SupportView) -> Scene {
    mesh_scene_with(mesh, |p| view.project(p.x, p.y, p.z))
}

/// Top-view scene of triangle edges, for the plan viewport.
pub fn mesh_xy_scene(mesh: &Mesh) -> Scene {
    mesh_scene_with(mesh, |p| [p.x, p.y])
}

fn mesh_scene_with(mesh: &Mesh, project: impl Fn(Vec3) -> [f64; 2]) -> Scene {
    let mut strokes = Vec::new();
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut total = 0.0;
    for tri in mesh.triangles() {
        for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
            let pa = project(a);
            let pb = project(b);
            include_pt(&mut min, &mut max, pa);
            include_pt(&mut min, &mut max, pb);
            let length = (pb[0] - pa[0]).hypot(pb[1] - pa[1]);
            total += length;
            strokes.push(Stroke {
                kind: StrokeKind::Cut,
                toolpath: 0,
                points: vec![pa, pb],
                zs: vec![a.z, b.z],
                z_start: a.z,
                z_end: b.z,
                angles: Vec::new(),
                length,
            });
        }
    }
    let bounds = if min[0].is_finite() { Some((min, max)) } else { None };
    Scene {
        strokes,
        pierces: Vec::new(),
        labels: Vec::new(),
        bounds,
        z_range: None,
        total_length: total,
        names: Vec::new(),
        rotary: None,
        lathe: None,
        timeline: Vec::new(),
    }
}

fn include_pt(min: &mut [f64; 2], max: &mut [f64; 2], p: [f64; 2]) {
    min[0] = min[0].min(p[0]);
    min[1] = min[1].min(p[1]);
    max[0] = max[0].max(p[0]);
    max[1] = max[1].max(p[1]);
}

pub fn apply_generate(
    session: &mut ResinSession,
    support: &SupportFields,
    lift_mm: &UserField,
    optimize: bool,
    cancel: Option<&CancelFlag>,
    progress: Option<&GenerateProgress>,
) -> Result<SupportedMesh, String> {
    apply_generate_inner(session, support, lift_mm, optimize, cancel, progress)
}

fn apply_generate_inner(
    session: &mut ResinSession,
    support: &SupportFields,
    lift_mm: &UserField,
    optimize: bool,
    cancel: Option<&CancelFlag>,
    progress: Option<&GenerateProgress>,
) -> Result<SupportedMesh, String> {
    if let Some(reason) =
        if optimize { optimize_disabled_reason(session, support, lift_mm) } else { generate_disabled_reason(session, support, lift_mm) }
    {
        return Err(reason);
    }
    let mesh = session.posed_part()?.clone();
    let params = support_params_from_fields(support, lift_mm)?;
    let detect = detect_params_from_fields(support)?;
    let (_, names) = session.points_palette();
    let keep_names = !session.palette_is_calibration();
    let extra: Vec<InputPoint> = session
        .points
        .iter()
        .filter(|point| point.source == PointSource::Manual)
        .map(|point| InputPoint {
            x: point.x,
            y: point.y,
            z: Some(point.contact_z),
            preset: if keep_names { point.preset_index.and_then(|index| names.get(index as usize).cloned()) } else { None },
            source: PointSource::Manual,
        })
        .collect();
    let presets = session.presets.clone();
    let mix = session.mix.clone();
    let zones = session.zones.clone().filter(|zones| !zones.is_empty());
    let mut options = GenerateOptions::auto(&extra);
    options.presets = &presets;
    options.mix = if optimize || mix.is_empty() { None } else { Some(&mix) };
    options.zones = zones.as_ref();
    options.zone_spacing_mm = parse_optional(&support.zone_spacing_mm, "painted zone spacing (mm)")?;
    options.solids = solid_options_from_fields(support)?;
    let options_solids = options.solids;
    options.detect = detect;
    options.island_ignore_area_mm2 = parse_optional(&support.island_ignore_area_mm2, "island ignore area (mm²)")?;
    options.lowest_band_height_mm = parse_optional(&support.lowest_band_height_mm, "lowest-band height (mm)")?;
    options.lowest_band_spacing_mm = parse_optional(&support.lowest_band_spacing_mm, "lowest-band spacing (mm)")?;
    options.optimize = optimize;
    let profile_owned = session.calibration.as_ref().and_then(|state| state.profile.clone());
    options.calibration = profile_owned.as_ref();
    options.cancel = cancel;
    options.progress = progress;
    let result = generate_supports_with(&mesh, &params, options).map_err(|err| err.to_string())?;
    session.points = result.points.clone();
    session.features = result.features.clone();
    session.warnings = result.warnings.clone();
    session.selected_point = None;
    session.optimize_summary = result.optimize.as_ref().map(|forces| {
        format!(
            "Optimizer preset `{}`: demand {:.4} N, capacity {:.4} N, scar {:.4} mm², {} tips, sufficient={}",
            forces.preset_name, forces.demand_n, forces.capacity_n, forces.scar_area_mm2, forces.tip_count, forces.sufficient
        )
    });
    let islands = session.features.as_ref().map(|f| f.islands.len()).unwrap_or(0);
    let mut extras = Vec::new();
    if let Some(report) = &result.zone_report {
        extras.push(format!("{} on painted zones, {} removed by no-support zones", report.painted_added, report.blocked_removed));
    }
    if result.brace_count > 0 || result.braces_skipped > 0 {
        extras.push(format!("{} brace(s)", result.brace_count));
    }
    if result.optimize.is_none() && !result.palette_names.is_empty() {
        let mut per: Vec<String> = Vec::new();
        for (index, name) in result.palette_names.iter().enumerate() {
            let used = result.points.iter().filter(|p| p.preset_index.map(usize::from) == Some(index)).count();
            if used > 0 {
                per.push(format!("{used} {name}"));
            }
        }
        let plain = result.points.iter().filter(|p| p.preset_index.is_none()).count();
        if !per.is_empty() {
            if plain > 0 {
                per.push(format!("{plain} on Tip and pillar fields"));
            }
            extras.push(per.join(", "));
        }
    }
    let extras = if extras.is_empty() { String::new() } else { format!(" ({})", extras.join("; ")) };
    session.last_generate_note =
        Some(format!("Placed {} support(s){extras}, {} island(s) detected. NOT printer-tested.", session.points.len(), islands));
    session.supported = Some(result.clone());
    session.built_with = Some((params.clone(), options_solids));
    session.points_dirty = false;
    session.invalidate_solids();
    session.invalidate_slice();
    Ok(result)
}

/// Load print settings from a user-chosen sample `.pm7m`, the same file `campanion resin --params-from` uses.
pub fn load_params_file(session: &mut ResinSession, path: std::path::PathBuf) -> Result<ResinSettings, String> {
    match load_print_settings(&path) {
        Ok(settings) => {
            session.params_from = Some(path);
            session.settings = Some(settings.clone());
            session.settings_error = None;
            session.invalidate_slice();
            Ok(settings)
        }
        Err(err) => {
            session.params_from = Some(path);
            session.settings = None;
            session.settings_error = Some(err.to_string());
            session.invalidate_slice();
            Err(err.to_string())
        }
    }
}

/// Mesh the slicer should cut: the supported solid (raft and pillars included) when supports exist.
pub fn print_mesh(session: &mut ResinSession, support: &SupportFields, lift_mm: &UserField) -> Result<campanion_mesh::Mesh, String> {
    let source = session.posed_part()?.clone();
    if session.points.is_empty() && session.supported.is_none() {
        return Ok(source);
    }
    let params = support_params_from_fields(support, lift_mm)?;
    let solids = solid_options_from_fields(support)?;
    let same_settings = session.built_with.as_ref().is_some_and(|(p, o)| *p == params && *o == solids);
    if !session.points_dirty
        && same_settings
        && let Some(supported) = &session.supported
        && supported.points == session.points
    {
        return Ok(supported.mesh.clone());
    }
    let (palette, names) = session.points_palette();
    let previous = session.supported.take();
    let mut rebuilt = rebuild_supported_with(&source, &params, &session.points, &palette, &names, &solids).map_err(|err| err.to_string())?;
    if let Some(previous) = previous {
        // Keep what generate found; only the solids are rebuilt.
        rebuilt.features = previous.features;
        rebuilt.optimize = previous.optimize;
        rebuilt.zone_report = previous.zone_report;
    }
    let mesh = rebuilt.mesh.clone();
    session.supported = Some(rebuilt);
    session.built_with = Some((params, solids));
    session.points_dirty = false;
    session.invalidate_solids();
    Ok(mesh)
}

/// Run the same slicer `campanion resin` uses and keep the `.pm7m` bytes for export.
pub fn apply_slice(session: &mut ResinSession, support: &SupportFields, lift_mm: &UserField, machine_selected: bool) -> Result<BuildReport, String> {
    if let Some(reason) = slice_disabled_reason(session, machine_selected) {
        return Err(reason);
    }
    let settings = session.settings.clone().ok_or_else(|| "choose a sample or params .pm7m first".to_string())?;
    let mesh = print_mesh(session, support, lift_mm)?;
    let placement = if session.pose.is_empty() { Placement::Center } else { Placement::Keep };
    let (bytes, report) = build_pm7m(&mesh, &M7_MAX, &settings, placement).map_err(|err| err.to_string())?;
    let back = campanion_resin::read_pm7m(&bytes).map_err(|err| format!("read-back failed: {err}"))?;
    if back.layers.len() != report.layers || back.machine_name != M7_MAX.name {
        return Err("read-back does not match what was built".to_string());
    }
    session.slice_islands = report.islands.clone();
    session.pixel_area_mm2 = report.pixel_area_mm2;
    session.layer_images = back.layer_images;
    session.slice_width = back.width;
    session.slice_height = back.height;
    session.pm7m = Some(bytes);
    session.slice_report = Some(report.clone());
    session.sliced = true;
    Ok(report)
}

/// Write the already-sliced `.pm7m` bytes. Call [`apply_slice`] first.
pub fn write_export(session: &mut ResinSession, path: &std::path::Path, machine_selected: bool) -> Result<(), String> {
    if let Some(reason) = export_disabled_reason(session, machine_selected) {
        return Err(reason);
    }
    let bytes = session.pm7m.as_ref().ok_or_else(|| "slice the part on the Slice tab first".to_string())?;
    std::fs::write(path, bytes).map_err(|err| format!("write {}: {err}", path.display()))?;
    session.last_export_path = Some(path.to_path_buf());
    Ok(())
}

/// Downscale one LCD layer (gray 0–15) to an RGBA preview, width capped at `max_side`.
pub fn layer_preview_rgba(image: &[u8], width: u32, height: u32, max_side: u32) -> Result<(u32, u32, Vec<u8>), String> {
    layer_preview_with_islands(image, width, height, max_side, &[]).map(|preview| (preview.width, preview.height, preview.rgba))
}

/// A downscaled layer with island regions drawn in colour, and their boxes in preview pixels.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LayerPreview {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
    /// Per island: colour and (min x, min y, max x, max y) in preview pixels.
    pub island_boxes: Vec<(Color32, [f32; 4])>,
}

/// Like [`layer_preview_rgba`], with each island's pixels painted in its colour.
pub fn layer_preview_with_islands(
    image: &[u8],
    width: u32,
    height: u32,
    max_side: u32,
    islands: &[(&[campanion_resin::Span], Color32)],
) -> Result<LayerPreview, String> {
    let (out_w, out_h, mut rgba, (x0, y0, x1, y1), scale) = layer_preview_base(image, width, height, max_side)?;
    let mut boxes = Vec::new();
    for (spans, color) in islands {
        let mut min = [f32::INFINITY; 2];
        let mut max = [f32::NEG_INFINITY; 2];
        for span in spans.iter() {
            if span.row < y0 || span.row > y1 {
                continue;
            }
            let oy = (span.row - y0) / scale;
            let start = span.start.max(x0);
            let end = span.end.min(x1.saturating_add(1));
            if start >= end {
                continue;
            }
            let ox0 = (start - x0) / scale;
            let ox1 = (end - 1 - x0) / scale;
            for ox in ox0..=ox1 {
                let i = (oy as usize * out_w as usize + ox as usize) * 4;
                if let Some(dst) = rgba.get_mut(i..i + 4) {
                    dst.copy_from_slice(&[color.r(), color.g(), color.b(), 255]);
                }
            }
            min[0] = min[0].min(ox0 as f32);
            min[1] = min[1].min(oy as f32);
            max[0] = max[0].max(ox1 as f32 + 1.0);
            max[1] = max[1].max(oy as f32 + 1.0);
        }
        if min[0].is_finite() {
            boxes.push((*color, [min[0], min[1], max[0], max[1]]));
        }
    }
    Ok(LayerPreview { width: out_w, height: out_h, rgba, island_boxes: boxes })
}

type PreviewBase = (u32, u32, Vec<u8>, (u32, u32, u32, u32), u32);

fn layer_preview_base(image: &[u8], width: u32, height: u32, max_side: u32) -> Result<PreviewBase, String> {
    if width == 0 || height == 0 {
        return Err("layer has no pixels".to_string());
    }
    let pixels = campanion_resin::pw0::decode(image, width, height).map_err(|err| err.to_string())?;
    let (x0, y0, x1, y1) = lit_bounds(&pixels, width, height).unwrap_or((0, 0, width.saturating_sub(1), height.saturating_sub(1)));
    let crop_w = x1.saturating_sub(x0).saturating_add(1).max(1);
    let crop_h = y1.saturating_sub(y0).saturating_add(1).max(1);
    let max_side = max_side.max(1);
    let scale = crop_w.max(crop_h).div_ceil(max_side).max(1);
    let out_w = crop_w.div_ceil(scale);
    let out_h = crop_h.div_ceil(scale);
    let mut rgba = vec![0u8; (out_w as usize).saturating_mul(out_h as usize).saturating_mul(4)];
    let sw = scale as usize;
    for oy in 0..out_h as usize {
        for ox in 0..out_w as usize {
            let mut acc = 0u32;
            let mut n = 0u32;
            for dy in 0..sw {
                let iy = y0 as usize + oy * sw + dy;
                if iy >= height as usize || iy > y1 as usize {
                    break;
                }
                let row = iy * width as usize;
                for dx in 0..sw {
                    let ix = x0 as usize + ox * sw + dx;
                    if ix >= width as usize || ix > x1 as usize {
                        break;
                    }
                    if let Some(v) = pixels.get(row + ix) {
                        acc += u32::from(*v);
                        n += 1;
                    }
                }
            }
            let gray = acc.checked_div(n).map(|v| (v * 17).min(255) as u8).unwrap_or(0);
            let i = (oy * out_w as usize + ox) * 4;
            if let Some(dst) = rgba.get_mut(i..i + 4) {
                dst.copy_from_slice(&[gray, gray, gray, 255]);
            }
        }
    }
    Ok((out_w, out_h, rgba, (x0, y0, x1, y1), scale))
}

fn lit_bounds(pixels: &[u8], width: u32, height: u32) -> Option<(u32, u32, u32, u32)> {
    let mut min_x = width;
    let mut min_y = height;
    let mut max_x = 0u32;
    let mut max_y = 0u32;
    let mut any = false;
    for y in 0..height {
        let row = y as usize * width as usize;
        for x in 0..width {
            if pixels.get(row + x as usize).is_some_and(|v| *v > 0) {
                any = true;
                min_x = min_x.min(x);
                min_y = min_y.min(y);
                max_x = max_x.max(x);
                max_y = max_y.max(y);
            }
        }
    }
    if !any {
        return None;
    }
    let pad_x = (max_x.saturating_sub(min_x) / 8).max(8);
    let pad_y = (max_y.saturating_sub(min_y) / 8).max(8);
    let x0 = min_x.saturating_sub(pad_x);
    let y0 = min_y.saturating_sub(pad_y);
    let x1 = (max_x + pad_x).min(width.saturating_sub(1));
    let y1 = (max_y + pad_y).min(height.saturating_sub(1));
    Some((x0, y0, x1, y1))
}

pub fn primary_button(ui: &mut egui::Ui, label: &str, reason: Option<&str>) -> bool {
    let button = egui::Button::new(RichText::new(label).color(Color32::WHITE).size(14.0)).fill(theme::ACCENT).min_size(egui::Vec2::new(200.0, 30.0));
    let enabled = reason.is_none();
    let response = ui.add_enabled(enabled, button);
    if let Some(reason) = reason {
        response.clone().on_hover_text(reason);
        false
    } else {
        response.clicked()
    }
}

/// Full-width wrapped reason under a disabled control, so the text is not clipped.
pub fn show_disabled_reason(ui: &mut egui::Ui, reason: Option<&str>) {
    if let Some(reason) = reason {
        ui.add(egui::Label::new(RichText::new(reason).color(theme::OFF_BED).size(11.0)).wrap());
    }
}

pub fn field_with_tip(ui: &mut egui::Ui, label: &str, tip: &str, field: &mut UserField, dark: bool) {
    ui.horizontal(|ui| {
        let response = ui.label(RichText::new(label).size(12.0));
        response.on_hover_text(tip);
        let overridden = field.is_overridden();
        let mut edit = egui::TextEdit::singleline(&mut field.value).desired_width(90.0);
        if overridden {
            edit = edit.text_color(theme::OVERRIDE).background_color(theme::override_fill(dark));
        }
        ui.add(edit).on_hover_text(tip);
        if overridden {
            if ui.small_button("Reset").on_hover_text("Restore the saved library value, or clear the field.").clicked() {
                field.reset();
            }
            if ui
                .small_button("Save to library")
                .on_hover_text("Remember this value as your library preset. The program still ships no defaults.")
                .clicked()
            {
                field.save_to_library();
            }
        }
    });
}

/// Nearest support point within `tol_mm`, if any.
pub fn hit_point(points: &[SupportPoint], x: f64, y: f64, tol_mm: f64) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (index, point) in points.iter().enumerate() {
        let d = (point.x - x).hypot(point.y - y);
        if d <= tol_mm && best.is_none_or(|(_, best_d)| d < best_d) {
            best = Some((index, d));
        }
    }
    best.map(|(index, _)| index)
}

/// Display colours for shaded resin solids. These are not process values.
pub const COLOR_PART: [u8; 4] = [200, 206, 214, 255];
pub const COLOR_RAFT: [u8; 4] = [70, 90, 110, 255];
pub const COLOR_SUPPORT: [u8; 4] = [240, 140, 40, 255];
pub const COLOR_SUPPORT_SELECTED: [u8; 4] = [255, 220, 80, 255];
pub const COLOR_BRACE: [u8; 4] = [214, 120, 40, 255];
pub const COLOR_ZONE_SUPPORT: [u8; 4] = [70, 190, 110, 255];
pub const COLOR_ZONE_BLOCK: [u8; 4] = [220, 70, 70, 255];

impl ResinSession {
    /// Drop cached solids so the next [`Self::ensure_solid_meshes`] rebuilds them.
    pub fn invalidate_solids(&mut self) {
        self.solid_meshes.clear();
    }

    /// Rebuild shaded part/raft/pillar/island meshes when the cache is empty.
    pub fn ensure_solid_meshes(&mut self, island_layer_mm: Option<f64>) {
        if !self.solid_meshes.is_empty() {
            return;
        }
        let mut meshes = self.part_meshes();
        if let Some(supported) = &self.supported {
            meshes.push(ShadedMesh::from_mesh(&supported.raft, COLOR_RAFT, MeshRole::Raft));
            meshes.push(ShadedMesh::from_mesh(&supported.pillars, COLOR_SUPPORT, MeshRole::Support));
            if !supported.braces.triangles().is_empty() {
                meshes.push(ShadedMesh::from_mesh(&supported.braces, COLOR_BRACE, MeshRole::Support));
            }
        }
        if let Some(features) = &self.features {
            let slab = island_layer_mm.filter(|h| h.is_finite() && *h > 0.0).unwrap_or(0.05);
            for island in &features.islands {
                let color = island_color_rgba(island.id);
                let min = Vec3::new(island.min.0, island.min.1, island.z_mm);
                let max = Vec3::new(island.max.0, island.max.1, island.z_mm + slab);
                if let Ok(mesh) = Mesh::new(Aabb::from_corners(min, max).face_triangles().to_vec()) {
                    meshes.push(ShadedMesh::from_mesh(&mesh, color, MeshRole::Island));
                }
            }
        }
        self.solid_meshes = meshes;
    }

    /// The part split by paint: unpainted, support-painted and no-support faces, each in its own colour.
    fn part_meshes(&mut self) -> Vec<ShadedMesh> {
        let part = match self.supported.as_ref() {
            Some(supported) => supported.part.clone(),
            None => match self.posed_part() {
                Ok(mesh) => mesh.clone(),
                Err(_) => return Vec::new(),
            },
        };
        let zones = self.zones.as_ref().filter(|zones| zones.triangle_count() == part.triangles().len() && !zones.is_empty());
        let Some(zones) = zones else {
            return vec![ShadedMesh::from_mesh(&part, COLOR_PART, MeshRole::Part)];
        };
        let mut plain = Vec::new();
        let mut support = Vec::new();
        let mut block = Vec::new();
        for (index, tri) in part.triangles().iter().enumerate() {
            match zones.mark(index) {
                ZoneMark::None => plain.push(*tri),
                ZoneMark::Support => support.push(*tri),
                ZoneMark::Block => block.push(*tri),
            }
        }
        let mut out = Vec::new();
        for (tris, color) in [(plain, COLOR_PART), (support, COLOR_ZONE_SUPPORT), (block, COLOR_ZONE_BLOCK)] {
            if tris.is_empty() {
                continue;
            }
            if let Ok(mesh) = Mesh::new(tris) {
                out.push(ShadedMesh::from_mesh(&mesh, color, MeshRole::Part));
            }
        }
        out
    }

    /// Replace only the part meshes in the cache, so painting does not rebuild every pillar.
    pub fn refresh_part_meshes(&mut self) {
        if self.solid_meshes.is_empty() {
            return;
        }
        let parts = self.part_meshes();
        self.solid_meshes.retain(|mesh| mesh.role != MeshRole::Part);
        let mut rebuilt = parts;
        rebuilt.append(&mut self.solid_meshes);
        self.solid_meshes = rebuilt;
    }

    /// Scene for the shared 3D viewport. Solids come from the cache; markers follow live points.
    pub fn scene3(&self, volume: Option<BuildVolume>) -> Scene3 {
        let mut scene = Scene3 { meshes: self.solid_meshes.clone(), volume, ..Scene3::default() };
        for (index, point) in self.points.iter().enumerate() {
            let selected = self.selected_point == Some(index);
            scene.markers.push(PointMarker {
                position: Vec3::new(point.x, point.y, point.contact_z),
                color: if selected { COLOR_SUPPORT_SELECTED } else { point.preset_index.map(preset_color).unwrap_or(COLOR_SUPPORT) },
                selected,
                id: index,
            });
        }
        scene
    }
}

fn island_color_rgba(id: u32) -> [u8; 4] {
    let c = island_color(id);
    [c.r(), c.g(), c.b(), 160]
}

/// Draw island fills and support points in plan view. `to_screen` maps world millimetres to pixels.
pub fn paint_support_overlay(
    painter: &egui::Painter,
    features: Option<&DetectedFeatures>,
    points: &[SupportPoint],
    selected_point: Option<usize>,
    selected_island: Option<u32>,
    view: SupportView,
    raft_top_mm: f64,
    to_screen: impl Fn([f64; 2]) -> egui::Pos2,
) {
    if let Some(features) = features {
        for island in &features.islands {
            let mut color = island_color(island.id);
            if selected_island == Some(island.id) {
                color = Color32::from_rgb(255, 230, 80);
            }
            let z = island.z_mm;
            let corners = [
                view.project(island.min.0, island.min.1, z),
                view.project(island.max.0, island.min.1, z),
                view.project(island.max.0, island.max.1, z),
                view.project(island.min.0, island.max.1, z),
            ];
            let screen: Vec<egui::Pos2> = corners.iter().copied().map(&to_screen).collect();
            if screen.len() == 4 {
                let mut min = screen[0];
                let mut max = screen[0];
                for p in &screen {
                    min.x = min.x.min(p.x);
                    min.y = min.y.min(p.y);
                    max.x = max.x.max(p.x);
                    max.y = max.y.max(p.y);
                }
                let rect = egui::Rect::from_min_max(min, max).expand(2.0);
                painter.rect_filled(rect, 2.0, color.gamma_multiply(0.28));
                painter.rect_stroke(rect, 2.0, UiStroke::new(1.5, color), egui::StrokeKind::Inside);
            }
        }
    }
    if view != SupportView::Top {
        for point in points {
            let a = to_screen(view.project(point.x, point.y, raft_top_mm));
            let b = to_screen(view.project(point.x, point.y, point.contact_z));
            painter.line_segment([a, b], UiStroke::new(2.0, Color32::from_rgb(240, 140, 40)));
        }
    }
    for (index, point) in points.iter().enumerate() {
        let pos = to_screen(view.project(point.x, point.y, point.contact_z));
        let selected = selected_point == Some(index);
        let fill = if selected { Color32::from_rgb(255, 220, 80) } else { Color32::from_rgb(240, 140, 40) };
        let radius = if selected { 6.5 } else { 5.0 };
        painter.circle_filled(pos, radius, fill);
        painter.circle_stroke(pos, radius, UiStroke::new(1.2, Color32::from_rgb(30, 30, 30)));
    }
}

/// Nearest support in the current view, in projected millimetres.
pub fn hit_point_in_view(points: &[SupportPoint], x: f64, y: f64, tol_mm: f64, view: SupportView) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (index, point) in points.iter().enumerate() {
        let p = view.project(point.x, point.y, point.contact_z);
        let d = (p[0] - x).hypot(p[1] - y);
        if d <= tol_mm && best.is_none_or(|(_, best_d)| d < best_d) {
            best = Some((index, d));
        }
    }
    best.map(|(index, _)| index)
}

pub fn show_supports_properties(ui: &mut egui::Ui, session: &mut ResinSession, support: &mut SupportFields, lift: &mut UserField, dark: bool) {
    ui.label(RichText::new("Supports").strong().size(14.0));
    ui.label(RichText::new("Fields start empty. The program ships no default sizes.").color(theme::MUTED).size(11.0));
    ui.add_space(6.0);

    if let Some(state) = &session.calibration {
        ui.label(RichText::new("Calibration").strong().size(13.0));
        ui.label(RichText::new(state.status_line()).size(11.0));
        if !state.is_complete() {
            ui.label(RichText::new("Missing or invalid constants:").color(theme::OFF_BED).size(12.0));
            for item in &state.missing {
                ui.label(RichText::new(format!("• {} — {}", item.label(), item.reason)).color(theme::OFF_BED).size(11.0));
            }
            if state.missing.is_empty()
                && let Some(error) = &state.error
            {
                ui.label(RichText::new(error).color(theme::OFF_BED).size(11.0));
            }
        }
        ui.separator();
    }

    ui.label(RichText::new("Islands").strong().size(13.0));
    match session.features.as_ref() {
        None => {
            ui.label(RichText::new("Generate supports to detect first-appearance islands, including mid-air bodies.").color(theme::MUTED).size(11.0));
        }
        Some(features) if features.islands.is_empty() => {
            ui.label(RichText::new("No islands detected at this layer height and cell size.").color(theme::MUTED).size(11.0));
        }
        Some(features) => {
            ui.label(RichText::new(format!("{} island(s), {} cup(s)", features.islands.len(), features.cups.len())).size(12.0));
            egui::ScrollArea::vertical().max_height(140.0).show(ui, |ui| {
                for island in &features.islands {
                    let selected = session.selected_island == Some(island.id);
                    let label = format!("Island {}  z {:.2} mm  area {:.2} mm²", island.id, island.z_mm, island.area_mm2);
                    let mut job = egui::RichText::new(label).size(12.0);
                    if selected {
                        job = job.strong();
                    }
                    let response = ui.selectable_label(selected, job);
                    if response.clicked() {
                        session.selected_island = Some(island.id);
                    }
                    response.on_hover_text(format!(
                        "First appears at z = {:.3} mm, centroid ({:.3}, {:.3}) mm. Colour-matched in the viewport.",
                        island.z_mm, island.centroid.0, island.centroid.1
                    ));
                }
            });
        }
    }
    ui.separator();

    ui.label(RichText::new("Placement").strong().size(13.0));
    field_with_tip(
        ui,
        "Layer height (mm)",
        "Sampling height for island detection, millimetres. Same value the slicer uses. Required to detect mid-air islands. No default.",
        &mut support.layer_height_mm,
        dark,
    );
    field_with_tip(
        ui,
        "Detection cell (mm)",
        "Occupancy cell size for island detection. If empty, support spacing is used. No default.",
        &mut support.detect_cell_mm,
        dark,
    );
    field_with_tip(
        ui,
        "Support spacing (mm)",
        "Maximum XY spacing between auto-placed tips, millimetres. The lowest band may be tighter. No default.",
        &mut support.support_spacing_mm,
        dark,
    );
    field_with_tip(
        ui,
        "Overhang angle (deg)",
        "Minimum angle from vertical at which a downward face is an overhang. 0 supports every downward face; 90 only faces parallel to the plate. No default.",
        &mut support.overhang_angle_deg,
        dark,
    );
    ui.separator();

    ui.label(RichText::new("Tip and pillar").strong().size(13.0));
    field_with_tip(ui, "Tip diameter (mm)", "Diameter of the contact tip, millimetres. No default.", &mut support.tip_diameter_mm, dark);
    field_with_tip(
        ui,
        "Tip length (mm)",
        "Length of the tapered tip, millimetres. Part lift must be at least tip length plus foot height. No default.",
        &mut support.tip_length_mm,
        dark,
    );
    field_with_tip(ui, "Pillar diameter (mm)", "Diameter of the vertical shaft, millimetres. No default.", &mut support.pillar_diameter_mm, dark);
    field_with_tip(
        ui,
        "Foot diameter (mm)",
        "Diameter of the foot on the raft, millimetres. Must be at least the pillar diameter. No default.",
        &mut support.foot_diameter_mm,
        dark,
    );
    field_with_tip(ui, "Foot height (mm)", "Height of the foot, millimetres. May be 0. No default.", &mut support.foot_height_mm, dark);
    field_with_tip(
        ui,
        "Tip penetration (mm)",
        "Optional. Depth each tip continues straight into the part past the contact, at the tip diameter. Empty means tips stop at the surface. A preset can set its own. No default.",
        &mut support.tip_penetration_mm,
        dark,
    );
    field_with_tip(
        ui,
        "Part lift (mm)",
        "Gap from the raft top to the lowest point of the part, millimetres. Not the printer's peel lift. No default.",
        lift,
        dark,
    );

    ui.add_space(6.0);
    let raft_open = egui::CollapsingHeader::new("Raft (undercut, narrow on the bed)").id_salt("resin-raft").default_open(true).show(ui, |ui| {
        ui.label(
            RichText::new("The bed-side outline is inset by thickness / tan(chamfer angle). That undercut is not inverted.")
                .color(theme::MUTED)
                .size(11.0),
        );
        field_with_tip(ui, "Raft thickness (mm)", "Raft height, millimetres. No default.", &mut support.raft_thickness_mm, dark);
        field_with_tip(
            ui,
            "Raft chamfer angle (deg)",
            "Angle of the raft perimeter from the build plate (horizontal), exclusive of 0 and 90. No default.",
            &mut support.raft_chamfer_angle_deg,
            dark,
        );
        field_with_tip(
            ui,
            "Raft margin (mm)",
            "Outward offset of the raft's top outline around the part and support footprint, millimetres. No default.",
            &mut support.raft_margin_mm,
            dark,
        );
        field_with_tip(
            ui,
            "Raft corner relief (mm)",
            "Length each top corner is cut back along both edges. 0 keeps sharp corners. No default.",
            &mut support.raft_corner_relief_mm,
            dark,
        );
    });
    let _ = raft_open;

    let band_open = egui::CollapsingHeader::new("Lowest band and ignore area").id_salt("resin-band").default_open(true).show(ui, |ui| {
        ui.label(
            RichText::new("Optional. Height and spacing must be entered together. Leave both empty for uniform spacing.")
                .color(theme::MUTED)
                .size(11.0),
        );
        field_with_tip(
            ui,
            "Lowest-band height (mm)",
            "Height above the part's minimum Z that uses denser spacing. Required if lowest-band spacing is set. No default.",
            &mut support.lowest_band_height_mm,
            dark,
        );
        field_with_tip(
            ui,
            "Lowest-band spacing (mm)",
            "Support spacing inside the lowest band, millimetres. Required if lowest-band height is set. No default.",
            &mut support.lowest_band_spacing_mm,
            dark,
        );
        field_with_tip(
            ui,
            "Island ignore area (mm²)",
            "Islands at or below this area are left unsupported. Optional; nothing is ignored unless you enter a value.",
            &mut support.island_ignore_area_mm2,
            dark,
        );
    });
    let _ = band_open;

    scroll_here(ui, session, "zones");
    show_painted_zones(ui, session, support, dark);
    scroll_here(ui, session, "bracing");
    show_bracing(ui, support, dark);
    scroll_here(ui, session, "presets");
    show_palette_editor(ui, session, support);
    show_calibration_panel(ui, session, support, lift);
}

fn scroll_here(ui: &mut egui::Ui, session: &mut ResinSession, key: &str) {
    if session.scroll_to == Some(key) {
        ui.scroll_to_cursor(Some(egui::Align::Min));
        session.scroll_to = None;
    }
}

fn draft_to_preset(draft: &SavedTipPreset) -> Result<TipPreset, String> {
    let num = |text: &str, label: &str| -> Result<f64, String> {
        let text = text.trim();
        if text.is_empty() {
            return Err(format!("enter {label}"));
        }
        text.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{label}: `{text}` is not a number"))
    };
    let penetration = match draft.tip_penetration_mm.trim() {
        "" => None,
        text => Some(num(text, "tip penetration (mm)")?),
    };
    let preset = TipPreset {
        name: draft.name.trim().to_string(),
        tip_diameter_mm: num(&draft.tip_diameter_mm, "tip diameter (mm)")?,
        tip_length_mm: num(&draft.tip_length_mm, "tip length (mm)")?,
        pillar_diameter_mm: num(&draft.pillar_diameter_mm, "pillar diameter (mm)")?,
        foot_diameter_mm: num(&draft.foot_diameter_mm, "foot diameter (mm)")?,
        foot_height_mm: num(&draft.foot_height_mm, "foot height (mm)")?,
        tip_penetration_mm: penetration,
    };
    preset.validate().map_err(|err| err.to_string())?;
    Ok(preset)
}

/// Fill the preset draft from the Tip and pillar fields, so the current values can be saved by name.
pub fn draft_from_fields(support: &SupportFields) -> SavedTipPreset {
    SavedTipPreset {
        name: String::new(),
        tip_diameter_mm: support.tip_diameter_mm.value.clone(),
        tip_length_mm: support.tip_length_mm.value.clone(),
        pillar_diameter_mm: support.pillar_diameter_mm.value.clone(),
        foot_diameter_mm: support.foot_diameter_mm.value.clone(),
        foot_height_mm: support.foot_height_mm.value.clone(),
        tip_penetration_mm: support.tip_penetration_mm.value.clone(),
    }
}

/// Display colour for a support marker using preset `index`. These are display colours only.
pub fn preset_color(index: u16) -> [u8; 4] {
    const COLOURS: [[u8; 4]; 5] = [[90, 150, 240, 255], [190, 110, 230, 255], [60, 200, 200, 255], [230, 210, 80, 255], [240, 120, 160, 255]];
    COLOURS[index as usize % COLOURS.len()]
}

fn show_painted_zones(ui: &mut egui::Ui, session: &mut ResinSession, support: &mut SupportFields, dark: bool) {
    egui::CollapsingHeader::new("Painted zones").id_salt("resin-zones").default_open(true).show(ui, |ui| {
        ui.label(
            RichText::new(
                "Pick Paint support or Paint no-support in the ribbon, then left-drag on the part. Green faces get supports at the zone spacing even below the overhang angle. Red faces never get an automatic support; supports you add by hand stay.",
            )
            .color(theme::MUTED)
            .size(11.0),
        );
        field_with_tip(
            ui,
            "Brush radius (mm)",
            "Faces with a corner or centre inside this radius of the pointer are painted. Only faces turned toward you are painted. No default.",
            &mut support.brush_radius_mm,
            dark,
        );
        field_with_tip(
            ui,
            "Painted zone spacing (mm)",
            "XY spacing between supports inside green zones. Required once a support zone is painted. No default.",
            &mut support.zone_spacing_mm,
            dark,
        );
        let (support_faces, block_faces) = session.zone_counts();
        ui.label(RichText::new(format!("{support_faces} support face(s), {block_faces} no-support face(s) painted")).size(12.0));
        if let Some(report) = session.supported.as_ref().and_then(|s| s.zone_report.as_ref()) {
            ui.label(
                RichText::new(format!(
                    "Last generate: {} added on zones, {} removed by no-support zones, {} painted face(s) cannot take a vertical support.",
                    report.painted_added, report.blocked_removed, report.painted_unreachable
                ))
                .color(theme::MUTED)
                .size(11.0),
            );
        }
        let nothing = support_faces + block_faces == 0;
        let clear = ui.add_enabled(!nothing, egui::Button::new("Clear all paint"));
        if nothing {
            clear.on_hover_text("Clear all paint is disabled: nothing is painted.");
        } else if clear.on_hover_text("Remove every painted face. Undo (Ctrl+Z) brings it back.").clicked() {
            session.clear_paint();
            session.last_generate_note = Some("Cleared all paint. Generate again to update the supports.".to_string());
        }
    });
}

fn show_bracing(ui: &mut egui::Ui, support: &mut SupportFields, dark: bool) {
    egui::CollapsingHeader::new("Cross-bracing").id_salt("resin-bracing").default_open(true).show(ui, |ui| {
        ui.checkbox(&mut support.bracing_on, "Brace neighbouring pillars")
            .on_hover_text("Join pillars closer than the max span with zig-zag struts. Struts that would touch the part are left out.");
        ui.add_enabled_ui(support.bracing_on, |ui| {
            field_with_tip(
                ui,
                "Brace diameter (mm)",
                "Strut diameter, millimetres. Required with bracing on. No default.",
                &mut support.brace_diameter_mm,
                dark,
            );
            field_with_tip(
                ui,
                "Brace max span (mm)",
                "Longest XY distance between two pillars that get braced together. Required with bracing on. No default.",
                &mut support.brace_span_mm,
                dark,
            );
            field_with_tip(
                ui,
                "Brace pitch (mm)",
                "Height each strut climbs between the two pillars. Required with bracing on. No default.",
                &mut support.brace_pitch_mm,
                dark,
            );
        });
        if !support.bracing_on {
            ui.label(RichText::new("Off. Tick the box to enter brace values.").color(theme::MUTED).size(11.0));
        }
    });
}

fn show_palette_editor(ui: &mut egui::Ui, session: &mut ResinSession, support: &SupportFields) {
    egui::CollapsingHeader::new("Tip presets and mix").id_salt("resin-palette").default_open(true).show(ui, |ui| {
        ui.label(
            RichText::new(
                "Presets are tip geometry you save by name; they are kept between sessions. The mix picks a preset for each kind of support; anything left on Tip and pillar fields uses the values above.",
            )
            .color(theme::MUTED)
            .size(11.0),
        );
        if let Some(path) = &session.presets_path {
            ui.label(RichText::new(format!("Saved in {}", path.display())).color(theme::MUTED).size(10.0));
        }
        if let Some(err) = &session.presets_error {
            ui.label(RichText::new(err).color(theme::OFF_BED).size(11.0));
        }
        let mut remove: Option<usize> = None;
        if session.presets.is_empty() {
            ui.label(RichText::new("No tip presets saved yet.").color(theme::MUTED).size(12.0));
        } else {
            for (index, preset) in session.presets.iter().enumerate() {
                ui.horizontal(|ui| {
                    let c = preset_color(u16::try_from(index).unwrap_or(0));
                    ui.label(RichText::new("●").color(Color32::from_rgb(c[0], c[1], c[2])));
                    let depth = preset.tip_penetration_mm.map(|d| format!(", sinks {d} mm")).unwrap_or_default();
                    ui.label(
                        RichText::new(format!(
                            "{}: tip {}×{} mm, pillar {} mm, foot {}×{} mm{depth}",
                            preset.name, preset.tip_diameter_mm, preset.tip_length_mm, preset.pillar_diameter_mm, preset.foot_diameter_mm, preset.foot_height_mm
                        ))
                        .size(12.0),
                    );
                    if ui.small_button("Edit").on_hover_text("Copy this preset into the form below to change it.").clicked() {
                        session.preset_draft = SavedTipPreset {
                            name: preset.name.clone(),
                            tip_diameter_mm: preset.tip_diameter_mm.to_string(),
                            tip_length_mm: preset.tip_length_mm.to_string(),
                            pillar_diameter_mm: preset.pillar_diameter_mm.to_string(),
                            foot_diameter_mm: preset.foot_diameter_mm.to_string(),
                            foot_height_mm: preset.foot_height_mm.to_string(),
                            tip_penetration_mm: preset.tip_penetration_mm.map(|d| d.to_string()).unwrap_or_default(),
                        };
                    }
                    if ui.small_button("Delete").on_hover_text("Remove this preset. Supports that used it go back to the Tip and pillar fields.").clicked() {
                        remove = Some(index);
                    }
                });
            }
        }
        if let Some(index) = remove {
            match session.remove_preset(index) {
                Ok(()) => session.last_generate_note = Some("Deleted the preset.".to_string()),
                Err(err) => session.last_generate_note = Some(err),
            }
        }

        ui.add_space(4.0);
        ui.label(RichText::new("Mix").strong().size(12.0));
        let calibrated = session.palette_is_calibration();
        if calibrated {
            ui.label(RichText::new("The last run was Optimize, which picks tips from the calibration palette. The mix applies to Generate supports.").color(theme::MUTED).size(11.0));
        }
        let names: Vec<String> = session.presets.iter().map(|p| p.name.clone()).collect();
        egui::Grid::new("resin-mix-grid").num_columns(2).spacing([8.0, 4.0]).show(ui, |ui| {
            for role in SupportRole::ALL {
                ui.label(RichText::new(role.label()).size(12.0));
                let current = session.mix.get(role).map(str::to_string);
                let shown = current.clone().unwrap_or_else(|| "Tip and pillar fields".to_string());
                let combo = egui::ComboBox::from_id_salt(("resin-mix", role.key())).selected_text(shown).width(150.0);
                let response = ui.add_enabled_ui(!names.is_empty(), |ui| {
                    combo.show_ui(ui, |ui| {
                        if ui.selectable_label(current.is_none(), "Tip and pillar fields").clicked() {
                            session.mix.set(role, None);
                        }
                        for name in &names {
                            if ui.selectable_label(current.as_deref() == Some(name.as_str()), name).clicked() {
                                session.mix.set(role, Some(name.clone()));
                            }
                        }
                    });
                });
                if names.is_empty() {
                    response.response.on_hover_text("Save a preset first. Until then every support uses the Tip and pillar fields.");
                }
                ui.end_row();
            }
        });

        if let Some(index) = session.selected_point {
            ui.add_space(4.0);
            let (_, point_names) = session.points_palette();
            let current = session.points.get(index).and_then(|p| p.preset_index);
            let shown = current.and_then(|i| point_names.get(i as usize).cloned()).unwrap_or_else(|| "Tip and pillar fields".to_string());
            ui.horizontal(|ui| {
                ui.label(RichText::new(format!("Selected support {index}")).size(12.0));
                let mut chosen: Option<Option<u16>> = None;
                let enabled = !point_names.is_empty();
                let response = ui.add_enabled_ui(enabled, |ui| {
                    egui::ComboBox::from_id_salt("resin-point-preset").selected_text(shown).width(150.0).show_ui(ui, |ui| {
                        if ui.selectable_label(current.is_none(), "Tip and pillar fields").clicked() {
                            chosen = Some(None);
                        }
                        for (i, name) in point_names.iter().enumerate() {
                            let i = u16::try_from(i).unwrap_or(u16::MAX);
                            if ui.selectable_label(current == Some(i), name).clicked() {
                                chosen = Some(Some(i));
                            }
                        }
                    });
                });
                if !enabled {
                    response.response.on_hover_text("Save a preset first to give this support its own tip.");
                }
                if let Some(choice) = chosen
                    && let Some(point) = session.points.get_mut(index)
                {
                    point.preset_index = choice;
                    session.points_dirty = true;
                    session.invalidate_slice();
                }
            });
        }

        ui.add_space(4.0);
        ui.label(RichText::new("Save a preset").strong().size(12.0));
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.name).desired_width(120.0).hint_text("e.g. Heavy"));
            if ui.small_button("Copy Tip and pillar fields").on_hover_text("Fill the sizes below from the Tip and pillar fields.").clicked() {
                let name = session.preset_draft.name.clone();
                session.preset_draft = draft_from_fields(support);
                session.preset_draft.name = name;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Tip Ø (mm)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.tip_diameter_mm).desired_width(60.0));
            ui.label("Tip length (mm)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.tip_length_mm).desired_width(60.0));
        });
        ui.horizontal(|ui| {
            ui.label("Pillar Ø (mm)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.pillar_diameter_mm).desired_width(60.0));
            ui.label("Tip penetration (mm, optional)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.tip_penetration_mm).desired_width(60.0));
        });
        ui.horizontal(|ui| {
            ui.label("Foot Ø (mm)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.foot_diameter_mm).desired_width(60.0));
            ui.label("Foot height (mm)");
            ui.add(egui::TextEdit::singleline(&mut session.preset_draft.foot_height_mm).desired_width(60.0));
        });
        let draft = draft_to_preset(&session.preset_draft);
        let reason = draft.as_ref().err().map(|err| format!("Save preset is disabled: {err}."));
        let save = ui.add_enabled(reason.is_none(), egui::Button::new("Save preset"));
        match (&reason, draft) {
            (Some(reason), _) => {
                save.on_hover_text(reason.as_str());
                show_disabled_reason(ui, Some(reason.as_str()));
            }
            (None, Ok(preset)) => {
                if save.on_hover_text("Save this geometry under the name. A preset with the same name is replaced.").clicked() {
                    let name = preset.name.clone();
                    match session.store_preset(preset) {
                        Ok(()) => {
                            session.last_generate_note = Some(format!("Saved preset `{name}`."));
                            session.preset_draft = SavedTipPreset::default();
                        }
                        Err(err) => session.last_generate_note = Some(err),
                    }
                }
            }
            (None, Err(_)) => {}
        }
    });
}

fn show_calibration_panel(ui: &mut egui::Ui, session: &mut ResinSession, support: &SupportFields, lift: &UserField) {
    egui::CollapsingHeader::new("Calibration").id_salt("resin-calibration").default_open(true).show(ui, |ui| {
        ui.label(RichText::new("Reserved for a user calibration profile. The test-print geometry is not in this release; load measured constants from JSON. Uniform generate works without a profile.").color(theme::MUTED).size(11.0));
        if let Some(state) = &session.calibration {
            ui.label(RichText::new(state.status_line()).size(12.0));
            if state.is_complete() {
                ui.label(RichText::new("All required constants are set.").color(theme::MUTED).size(11.0));
            } else {
                ui.label(RichText::new("Missing or invalid constants:").color(theme::OFF_BED).size(12.0));
                for item in &state.missing {
                    ui.label(RichText::new(format!("• {} — {}", item.label(), item.reason)).color(theme::OFF_BED).size(11.0));
                }
                if state.missing.is_empty()
                    && let Some(error) = &state.error
                {
                    ui.label(RichText::new(error).color(theme::OFF_BED).size(11.0));
                }
            }
        } else {
            ui.label(RichText::new("No calibration profile loaded.").color(theme::MUTED).size(12.0));
            ui.label(RichText::new("Load a campanion-resin-calibration/1 JSON file. Required: schema, profile name, tip palette (geometry and tip capacity in N, contact area in mm²), peel force per area (N/mm²), suction force per area (N/mm²), safety factor, and resin density (g/cm³). Standard gravity (9.80665 m/s²) is a physical constant and is not entered.").color(theme::MUTED).size(11.0));
        }
        ui.add_space(4.0);
        let reason = optimize_disabled_reason(session, support, lift);
        let _ = reason;
        ui.label(RichText::new("Use the Optimize supports button in the toolbar. It stays disabled until a complete profile is loaded.").color(theme::MUTED).size(11.0));
    });
}

pub fn selected_count(session: &ResinSession) -> u32 {
    u32::try_from(session.selected_point.map(|_| 1).unwrap_or(0)).unwrap_or(0)
}

/// Background Generate / Optimize work, with progress and cancel.
#[derive(Default)]
pub struct ResinGenerateJob {
    cancel: Option<std::sync::Arc<CancelFlag>>,
    progress: Option<std::sync::Arc<GenerateProgress>>,
    receiver: Option<std::sync::mpsc::Receiver<Result<ResinSession, String>>>,
}

impl ResinGenerateJob {
    /// True while a worker is outstanding.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.receiver.is_some()
    }

    /// Start generation on a background thread. Refuses if a run is already in flight.
    pub fn start(&mut self, session: &ResinSession, support: SupportFields, lift_mm: UserField, optimize: bool) -> Result<(), String> {
        if self.is_running() {
            return Err("Generate supports is already running.".to_string());
        }
        let reason =
            if optimize { optimize_disabled_reason(session, &support, &lift_mm) } else { generate_disabled_reason(session, &support, &lift_mm) };
        if let Some(reason) = reason {
            return Err(reason);
        }
        let cancel = std::sync::Arc::new(CancelFlag::new());
        let progress = std::sync::Arc::new(GenerateProgress::new());
        let (sender, receiver) = std::sync::mpsc::channel();
        let mut worker_session = session.clone();
        let cancel_w = std::sync::Arc::clone(&cancel);
        let progress_w = std::sync::Arc::clone(&progress);
        std::thread::spawn(move || {
            let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                apply_generate(&mut worker_session, &support, &lift_mm, optimize, Some(cancel_w.as_ref()), Some(progress_w.as_ref()))
            }));
            let result = match outcome {
                Ok(Ok(_)) => Ok(worker_session),
                Ok(Err(err)) => Err(err),
                Err(payload) => Err(crate::crash::log_caught_panic(payload.as_ref()).0),
            };
            let _ = sender.send(result);
        });
        self.cancel = Some(cancel);
        self.progress = Some(progress);
        self.receiver = Some(receiver);
        Ok(())
    }

    /// Ask the worker to stop at the next layer or stage check.
    pub fn cancel(&self) {
        if let Some(flag) = &self.cancel {
            flag.cancel();
        }
    }

    /// Take a finished result, if any.
    pub fn tick(&mut self) -> Option<Result<ResinSession, String>> {
        let receiver = self.receiver.as_ref()?;
        match receiver.try_recv() {
            Ok(result) => {
                self.receiver = None;
                self.cancel = None;
                self.progress = None;
                Some(result)
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.receiver = None;
                self.cancel = None;
                self.progress = None;
                Some(Err("Generate supports stopped unexpectedly.".to_string()))
            }
        }
    }

    /// Status line while a run is in flight.
    #[must_use]
    pub fn progress_text(&self) -> Option<String> {
        let progress = self.progress.as_ref()?;
        let stage = progress.stage();
        let layer = progress.layer();
        let layers = progress.layers();
        if layers > 0 {
            Some(format!("{stage} — layer {layer} / {layers}"))
        } else if stage.is_empty() {
            Some("Generating supports…".to_string())
        } else {
            Some(format!("{stage}…"))
        }
    }

    /// 0..=1 while a run is in flight, based on the current stage and layer.
    #[must_use]
    pub fn progress_fraction(&self) -> Option<f32> {
        let progress = self.progress.as_ref()?;
        let stage = progress.stage();
        let layer = progress.layer();
        let layers = progress.layers();
        let frac = if stage.to_ascii_lowercase().contains("plac") {
            0.85
        } else if stage.to_ascii_lowercase().contains("build") {
            0.95
        } else if layers > 0 {
            (layer as f32 / layers as f32).clamp(0.0, 1.0) * 0.8
        } else {
            0.05
        };
        Some(frac.clamp(0.0, 1.0))
    }

    /// Drop an in-flight worker without applying its result. Used when a screenshot frame changes.
    pub fn abandon(&mut self) {
        self.cancel();
        self.receiver = None;
        self.cancel = None;
        self.progress = None;
    }
}

/// Copy generate outputs from a worker session onto the live session, keeping the user's view and tool.
pub fn commit_generate_session(live: &mut ResinSession, worker: ResinSession) {
    live.points = worker.points;
    live.features = worker.features;
    live.warnings = worker.warnings;
    live.selected_point = worker.selected_point;
    live.optimize_summary = worker.optimize_summary;
    live.last_generate_note = worker.last_generate_note;
    live.supported = worker.supported;
    live.points_dirty = worker.points_dirty;
    live.solid_meshes = worker.solid_meshes;
    live.sliced = worker.sliced;
    live.pm7m = worker.pm7m;
    live.slice_report = worker.slice_report;
    live.layer_images = worker.layer_images;
    live.slice_width = worker.slice_width;
    live.slice_height = worker.slice_height;
    live.built_with = worker.built_with;
    live.slice_islands = worker.slice_islands;
    live.layer_cache = None;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SupportFields;

    #[test]
    fn generate_and_optimize_reasons_name_what_to_enter() {
        let session = ResinSession::default();
        let support = SupportFields::default();
        let lift = UserField::empty();
        let reason = generate_disabled_reason(&session, &support, &lift).expect("disabled");
        assert!(reason.contains("open a part"), "{reason}");
        assert!(reason.contains("Generate supports is disabled"), "{reason}");

        let session = ResinSession { mesh: Some(Mesh::new(Vec::new()).expect("empty mesh")), ..ResinSession::default() };
        let reason = generate_disabled_reason(&session, &support, &lift).expect("fields");
        assert!(reason.contains("layer height (mm)"), "{reason}");
        assert!(reason.contains("support spacing (mm)"), "{reason}");
        assert!(reason.contains("tip diameter (mm)"), "{reason}");
        assert!(reason.contains("raft chamfer angle (deg)"), "{reason}");

        let reason = optimize_disabled_reason(&session, &support, &lift).expect("opt");
        assert!(reason.contains("Optimize supports is disabled"), "{reason}");

        let export = export_disabled_reason(&session, true).expect("export");
        assert!(export.contains("params") || export.contains("slice") || export.contains("part"), "{export}");
        let machine = slice_disabled_reason(&session, false).expect("machine");
        assert!(machine.contains("resin printer"), "{machine}");
    }

    #[test]
    fn palette_starts_empty() {
        let session = ResinSession::default();
        assert!(session.presets.is_empty());
        assert!(session.calibration.is_none());
    }

    #[test]
    fn incomplete_calibration_is_listed() {
        let state = load_calibration_text(std::path::PathBuf::from("partial.json"), r#"{"schema":"campanion-resin-calibration/1","name":"partial"}"#);
        assert!(!state.is_complete());
        assert!(state.missing.iter().any(|item| item.name == "tip_palette"), "{:?}", state.missing);
        let session = ResinSession { mesh: Some(Mesh::new(Vec::new()).expect("mesh")), calibration: Some(state), ..ResinSession::default() };
        let mut support = SupportFields::default();
        for field in [
            &mut support.layer_height_mm,
            &mut support.support_spacing_mm,
            &mut support.overhang_angle_deg,
            &mut support.tip_diameter_mm,
            &mut support.tip_length_mm,
            &mut support.pillar_diameter_mm,
            &mut support.foot_diameter_mm,
            &mut support.foot_height_mm,
            &mut support.raft_thickness_mm,
            &mut support.raft_chamfer_angle_deg,
            &mut support.raft_margin_mm,
            &mut support.raft_corner_relief_mm,
        ] {
            field.value = "1".into();
        }
        let mut lift = UserField::empty();
        lift.value = "5".into();
        let reason = optimize_disabled_reason(&session, &support, &lift).expect("incomplete");
        assert!(reason.contains("incomplete"), "{reason}");
        assert!(reason.contains("Tip palette") || reason.contains("Peel force per area"), "{reason}");
        assert!(!reason.contains("gravity_m_s2"), "{reason}");
    }

    #[test]
    fn constant_labels_use_units() {
        assert_eq!(campanion_resin::constant_label("peel_coefficient_n_per_mm2"), "Peel force per area (N/mm²)");
        assert_eq!(campanion_resin::constant_label("suction_coefficient_n_per_mm2"), "Suction force per area (N/mm²)");
        assert_eq!(campanion_resin::constant_label("resin_density_g_per_cm3"), "Resin density (g/cm³)");
        assert!((campanion_resin::STANDARD_GRAVITY_M_S2 - 9.80665).abs() < 1e-12);
    }

    fn off_center_box() -> Mesh {
        use campanion_mesh::{Aabb, Vec3};
        Mesh::new(Aabb::from_corners(Vec3::new(135.0, 70.0, 0.0), Vec3::new(163.0, 96.0, 61.0)).face_triangles().to_vec()).expect("box")
    }

    fn filled_support() -> (SupportFields, UserField) {
        let mut support = SupportFields::default();
        let set = |field: &mut UserField, value: &str| field.value = value.to_string();
        set(&mut support.layer_height_mm, "0.50");
        set(&mut support.detect_cell_mm, "0.50");
        set(&mut support.support_spacing_mm, "4.00");
        set(&mut support.overhang_angle_deg, "45");
        set(&mut support.tip_diameter_mm, "0.60");
        set(&mut support.tip_length_mm, "1.20");
        set(&mut support.pillar_diameter_mm, "1.20");
        set(&mut support.foot_diameter_mm, "3.00");
        set(&mut support.foot_height_mm, "0.80");
        set(&mut support.raft_thickness_mm, "1.50");
        set(&mut support.raft_chamfer_angle_deg, "30");
        set(&mut support.raft_margin_mm, "3.00");
        set(&mut support.raft_corner_relief_mm, "0");
        let mut lift = UserField::empty();
        lift.value = "5.00".to_string();
        (support, lift)
    }

    #[test]
    fn empty_pose_fields_leave_the_imported_part_in_place() {
        let mut session = ResinSession { mesh: Some(off_center_box()), ..ResinSession::default() };
        let position = [UserField::empty(), UserField::empty(), UserField::empty()];
        let rotation = [UserField::empty(), UserField::empty(), UserField::empty()];
        apply_pose_fields(&mut session, &position, &rotation).expect("pose");
        assert!(session.pose.is_empty());
        let posed = session.posed_part().expect("posed");
        let bounds = posed.bounds().expect("bounds");
        assert!((bounds.min().x - 135.0).abs() < 1e-9, "{}", bounds.min().x);
        assert!((bounds.min().y - 70.0).abs() < 1e-9, "{}", bounds.min().y);
    }

    #[test]
    fn typed_pose_and_center_on_plate_move_an_off_center_part() {
        let mut session = ResinSession { mesh: Some(off_center_box()), ..ResinSession::default() };
        let mut position = [UserField::empty(), UserField::empty(), UserField::empty()];
        let mut rotation = [UserField::empty(), UserField::empty(), UserField::empty()];
        position[0].value = "0".to_string();
        position[1].value = "0".to_string();
        apply_pose_fields(&mut session, &position, &rotation).expect("pose");
        let posed = session.posed_part().expect("posed");
        let bounds = posed.bounds().expect("bounds");
        assert!(((bounds.min().x + bounds.max().x) / 2.0).abs() < 1e-9);
        assert!(((bounds.min().y + bounds.max().y) / 2.0).abs() < 1e-9);

        let mut session = ResinSession { mesh: Some(off_center_box()), ..ResinSession::default() };
        center_session_on_plate(&mut session, &mut position, &mut rotation).expect("center");
        assert_eq!(position[0].value, "0");
        assert_eq!(position[1].value, "0");
        let posed = session.posed_part().expect("posed");
        let bounds = posed.bounds().expect("bounds");
        assert!(((bounds.min().x + bounds.max().x) / 2.0).abs() < 1e-9);
        assert!(((bounds.min().y + bounds.max().y) / 2.0).abs() < 1e-9);
        assert!((bounds.min().z - 0.0).abs() < 1e-9);
    }

    #[test]
    fn drop_to_plate_lifts_a_part_that_sits_below_z_zero() {
        use campanion_mesh::{Aabb, Vec3};
        let mesh = Mesh::new(Aabb::from_corners(Vec3::new(-10.0, -10.0, -42.0), Vec3::new(10.0, 10.0, 54.0)).face_triangles().to_vec()).expect("box");
        let mut session = ResinSession { mesh: Some(mesh), ..ResinSession::default() };
        let mut position = [UserField::empty(), UserField::empty(), UserField::empty()];
        let mut rotation = [UserField::empty(), UserField::empty(), UserField::empty()];
        drop_session_to_plate(&mut session, &mut position, &mut rotation).expect("drop");
        let posed = session.posed_part().expect("posed");
        let bounds = posed.bounds().expect("bounds");
        assert!((bounds.min().z).abs() < 1e-9, "min z {}", bounds.min().z);
        assert!((bounds.max().z - 96.0).abs() < 1e-9, "max z {}", bounds.max().z);
    }

    #[test]
    fn generate_uses_the_posed_mesh_and_honours_cancel() {
        let mut session = ResinSession { mesh: Some(off_center_box()), ..ResinSession::default() };
        let mut position = [UserField::empty(), UserField::empty(), UserField::empty()];
        let mut rotation = [UserField::empty(), UserField::empty(), UserField::empty()];
        center_session_on_plate(&mut session, &mut position, &mut rotation).expect("center");
        let (support, lift) = filled_support();
        let result = apply_generate(&mut session, &support, &lift, false, None, None).expect("generate");
        let bounds = result.part.bounds().expect("bounds");
        assert!(((bounds.min().x + bounds.max().x) / 2.0).abs() < 1e-6, "generated part was not centred");

        let cancel = campanion_resin::CancelFlag::new();
        cancel.cancel();
        let err = apply_generate(&mut session, &support, &lift, false, Some(&cancel), None).expect_err("cancelled");
        assert!(err == "cancelled" || err.contains("cancelled"), "{err}");
    }

    #[test]
    fn pose_buttons_explain_why_they_are_disabled() {
        let session = ResinSession::default();
        let reason = pose_action_disabled_reason(&session, "Center on plate").expect("disabled");
        assert!(reason.contains("Center on plate is disabled"), "{reason}");
        assert!(reason.contains("open a part"), "{reason}");
    }

    fn cube(size: f64) -> Mesh {
        use campanion_mesh::{Aabb, Vec3};
        Mesh::new(Aabb::from_corners(Vec3::new(0.0, 0.0, 0.0), Vec3::new(size, size, size)).face_triangles().to_vec()).expect("cube")
    }

    #[test]
    fn painting_marks_faces_and_undo_restores_them() {
        let mut session = ResinSession { mesh: Some(cube(10.0)), ..ResinSession::default() };
        let before = session.snapshot();
        let up = Vec3::new(0.0, 0.0, 1.0);
        let changed = session.paint_at(Vec3::new(5.0, 5.0, 0.0), 3.0, ZoneMark::Support, Some(up)).expect("paint");
        assert!(changed > 0);
        let (support, block) = session.zone_counts();
        assert!(support > 0 && block == 0, "{support} {block}");
        // Front-face only: the top of the cube faces away from a view looking up and stays clean.
        let changed = session.paint_at(Vec3::new(5.0, 5.0, 10.0), 3.0, ZoneMark::Block, Some(up)).expect("paint top");
        assert_eq!(changed, 0);
        session.restore(before);
        assert_eq!(session.zone_counts(), (0, 0));
        session.paint_at(Vec3::new(5.0, 5.0, 0.0), 3.0, ZoneMark::Block, None).expect("paint block");
        assert!(session.zone_counts().1 > 0);
        session.clear_paint();
        assert_eq!(session.zone_counts(), (0, 0));
    }

    #[test]
    fn painted_zones_and_bracing_name_their_missing_fields() {
        let (mut support, lift) = filled_support();
        let mut session = ResinSession { mesh: Some(cube(10.0)), ..ResinSession::default() };
        assert!(generate_disabled_reason(&session, &support, &lift).is_none());
        session.paint_at(Vec3::new(5.0, 5.0, 0.0), 3.0, ZoneMark::Support, None).expect("paint");
        let reason = generate_disabled_reason(&session, &support, &lift).expect("needs spacing");
        assert!(reason.contains("painted zone spacing"), "{reason}");
        support.zone_spacing_mm.value = "2".to_string();
        assert!(generate_disabled_reason(&session, &support, &lift).is_none());
        support.bracing_on = true;
        let reason = generate_disabled_reason(&session, &support, &lift).expect("needs brace fields");
        assert!(reason.contains("brace diameter") && reason.contains("brace max span") && reason.contains("brace pitch"), "{reason}");

        let reason = paint_disabled_reason(&session, &support).expect("needs brush");
        assert!(reason.contains("brush radius"), "{reason}");
        support.brush_radius_mm.value = "0".to_string();
        let reason = paint_disabled_reason(&session, &support).expect("zero brush");
        assert!(reason.contains("greater than 0"), "{reason}");
        support.brush_radius_mm.value = "2".to_string();
        assert!(paint_disabled_reason(&session, &support).is_none());
        assert!(paint_disabled_reason(&ResinSession::default(), &support).expect("no part").contains("open a part"));
    }

    #[test]
    fn solid_options_are_off_when_empty_and_checked_when_set() {
        let (mut support, _) = filled_support();
        assert_eq!(solid_options_from_fields(&support).expect("empty"), SolidOptions::default());
        support.tip_penetration_mm.value = "0.2".to_string();
        support.bracing_on = true;
        assert!(solid_options_from_fields(&support).is_err(), "bracing on needs its fields");
        support.brace_diameter_mm.value = "0.5".to_string();
        support.brace_span_mm.value = "6".to_string();
        support.brace_pitch_mm.value = "3".to_string();
        let options = solid_options_from_fields(&support).expect("set");
        assert_eq!(options.tip_penetration_mm, Some(0.2));
        assert_eq!(options.bracing.map(|b| b.max_span_mm), Some(6.0));
        support.tip_penetration_mm.value = "-1".to_string();
        assert!(solid_options_from_fields(&support).is_err());
    }

    #[test]
    fn removing_a_preset_fixes_points_and_mix() {
        let dir = std::env::temp_dir().join(format!("campanion-gui-presets-{}", std::process::id()));
        let path = dir.join("presets.json");
        let mut session = ResinSession::default();
        session.load_presets(path.clone());
        assert!(session.presets.is_empty());
        let preset = |name: &str, tip: f64| TipPreset {
            name: name.to_string(),
            tip_diameter_mm: tip,
            tip_length_mm: 1.0,
            pillar_diameter_mm: 1.0,
            foot_diameter_mm: 2.0,
            foot_height_mm: 0.5,
            tip_penetration_mm: None,
        };
        session.store_preset(preset("a", 0.4)).expect("a");
        session.store_preset(preset("b", 0.8)).expect("b");
        session.store_preset(preset("b", 0.9)).expect("replace b");
        assert_eq!(session.presets.len(), 2);
        session.mix.set(SupportRole::Island, Some("b".to_string()));
        session.mix.set(SupportRole::Overhang, Some("a".to_string()));
        let mut on_a = SupportPoint::new(1.0, 1.0, 5.0);
        on_a.preset_index = Some(0);
        let mut on_b = SupportPoint::new(2.0, 2.0, 5.0);
        on_b.preset_index = Some(1);
        session.points = vec![on_a, on_b];

        session.remove_preset(0).expect("remove a");
        assert_eq!(session.points.first().and_then(|p| p.preset_index), None);
        assert_eq!(session.points.get(1).and_then(|p| p.preset_index), Some(0));
        assert_eq!(session.mix.get(SupportRole::Island), Some("b"));
        assert_eq!(session.mix.get(SupportRole::Overhang), None);

        let mut reloaded = ResinSession::default();
        reloaded.load_presets(path);
        assert_eq!(reloaded.presets.len(), 1);
        assert_eq!(reloaded.presets.first().map(|p| p.tip_diameter_mm), Some(0.9));
        assert!(session.remove_preset(5).is_err());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn island_pixels_are_coloured_and_boxed() {
        let (w, h) = (8u32, 8u32);
        let mut pixels = vec![0u8; (w * h) as usize];
        for y in 0..8usize {
            for x in 0..8usize {
                if ((x < 3 && y < 3) || (x >= 5 && y >= 5))
                    && let Some(px) = pixels.get_mut(y * 8 + x)
                {
                    *px = 15;
                }
            }
        }
        let image = campanion_resin::pw0::encode(&pixels).expect("encode");
        let spans: Vec<campanion_resin::Span> = (5..8).map(|row| campanion_resin::Span { row, start: 5, end: 8 }).collect();
        let red = Color32::from_rgb(250, 10, 10);
        let preview = layer_preview_with_islands(&image, w, h, 64, &[(spans.as_slice(), red)]).expect("preview");
        assert_eq!((preview.width, preview.height), (8, 8));
        let at = |x: usize, y: usize| preview.rgba.get((y * 8 + x) * 4..(y * 8 + x) * 4 + 4).map(<[u8]>::to_vec);
        assert_eq!(at(6, 6), Some(vec![250, 10, 10, 255]));
        assert_ne!(at(1, 1), Some(vec![250, 10, 10, 255]));
        assert_eq!(preview.island_boxes.len(), 1);
        assert_eq!(preview.island_boxes.first().map(|b| b.1), Some([5.0, 5.0, 8.0, 8.0]));
        let plain = layer_preview_rgba(&image, w, h, 64).expect("plain");
        assert_ne!(plain.2.get((6 * 8 + 6) * 4..(6 * 8 + 6) * 4 + 4), Some(&[250u8, 10, 10, 255][..]));
    }
}
