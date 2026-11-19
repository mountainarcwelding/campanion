//! Multi-part planning, bed checks, and a cancellable planning handle.

use std::sync::atomic::{AtomicBool, Ordering};

use campanion_import::{Drawing, ImportOptions, import_dxf, import_svg, place_with};
use campanion_ir::{Document, MoveRole};

use crate::error::CliError;
use crate::job_parts::{JobPart, OpSelector, contour_ids, selector_includes_part};
use crate::jobfile::{FORMAT_TAG_V2, JobFile, Placement, join_job_path};
use crate::profiles::{self, JobType, MachineFile, MachineKind, WorkArea, job_type_for, load_job_ref, selected_head};
use crate::run::{self, PlannedProgram};
use crate::step;

/// Handle the GUI can use later to cancel a background regenerate.
#[derive(Debug, Default)]
pub struct PlanCancel {
    cancelled: AtomicBool,
}

impl PlanCancel {
    #[must_use]
    pub fn new() -> Self {
        Self { cancelled: AtomicBool::new(false) }
    }

    /// Ask planning to stop at the next instance or operation boundary.
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
    }

    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }
}

/// Plan `job`, running every instance in order when the job has more than one.
pub fn plan_job_inner(job: &JobFile, up_axis: step::UpAxis, cancel: Option<&PlanCancel>) -> Result<PlannedProgram, CliError> {
    if let Some(cancel) = cancel
        && cancel.is_cancelled()
    {
        return Err(cancelled());
    }
    let max_power = check_plan_machine(job)?;
    let shifted = apply_datum(job)?;
    let job = shifted.as_ref().unwrap_or(job);
    let planned = if job.is_multi_part() {
        plan_multi(job, up_axis, cancel, max_power)?
    } else if job.format() == FORMAT_TAG_V2 {
        plan_single_v2(job, up_axis, cancel, max_power)?
    } else {
        run::plan_loaded_job_with(job, up_axis, cancel)?
    };
    let mut planned = planned;
    if job.format() == FORMAT_TAG_V2 || job.is_multi_part() {
        planned.warnings.extend(planning_warnings(job, &planned.document)?);
    }
    Ok(planned)
}

fn cancelled() -> CliError {
    CliError::run("planning was cancelled")
}

fn plan_single_v2(job: &JobFile, up_axis: step::UpAxis, cancel: Option<&PlanCancel>, max_power: Option<f64>) -> Result<PlannedProgram, CliError> {
    let part = job.parts().first().ok_or_else(|| CliError::run("a job needs at least one part instance"))?;
    let instance = part.instances.first().ok_or_else(|| CliError::run("a job needs at least one part instance"))?;
    let instance_job = job.for_instance(part, instance, max_power).map_err(|err| CliError::run(err.to_string()))?;
    run::plan_loaded_job_with(&instance_job, up_axis, cancel)
}

fn plan_multi(job: &JobFile, up_axis: step::UpAxis, cancel: Option<&PlanCancel>, max_power: Option<f64>) -> Result<PlannedProgram, CliError> {
    let mut combined: Option<PlannedProgram> = None;
    for part in job.parts() {
        for instance in &part.instances {
            if let Some(cancel) = cancel
                && cancel.is_cancelled()
            {
                return Err(cancelled());
            }
            let instance_job = job.for_instance(part, instance, max_power).map_err(|err| CliError::run(err.to_string()))?;
            let planned = run::plan_loaded_job_with(&instance_job, up_axis, cancel)?;
            combined = Some(match combined {
                None => planned,
                Some(mut acc) => {
                    let document = acc.document.append_toolpaths(&planned.document).map_err(|err| CliError::run(err.to_string()))?;
                    acc.document = document;
                    acc.warnings.extend(planned.warnings);
                    acc.notes.extend(planned.notes);
                    acc.gouges.extend(planned.gouges);
                    acc
                }
            });
        }
    }
    combined.ok_or_else(|| CliError::run("a job needs at least one part instance"))
}

fn planning_warnings(job: &JobFile, document: &Document) -> Result<Vec<String>, CliError> {
    let mut warnings = Vec::new();
    if uses_color_selector(job) {
        warnings.push("colour selectors are stored but not matched yet; the importer does not record contour colour".to_string());
    }
    warnings.extend(unused_contour_warnings(job)?);
    warnings.extend(off_bed_warnings(job, false)?);
    warnings.extend(off_sheet_warnings(job)?);
    warnings.extend(lead_in_cross_warnings(job, document)?);
    Ok(warnings)
}

fn uses_color_selector(job: &JobFile) -> bool {
    job.operations()
        .iter()
        .any(|op| matches!(job.operation_extras(op.name()).and_then(|extra| extra.selector.as_ref()), Some(OpSelector::Color { .. })))
}

fn check_plan_machine(job: &JobFile) -> Result<Option<f64>, CliError> {
    let Some(reference) = job.machine() else {
        return Ok(None);
    };
    let store = profiles::ProfileStore::from_env()?;
    let profile = load_job_ref(reference, &store)?;
    if profile.kind == MachineKind::Resin {
        check_job_type(job, job_type_for(&profile, None))?;
        return Ok(None);
    }
    let head = selected_head(&profile, reference)?;
    check_job_type(job, job_type_for(&profile, Some(head)))?;
    Ok(match head.max_power {
        Some(power) if power.unit == crate::profiles::PowerUnit::Watts => Some(power.value),
        _ => None,
    })
}

fn check_job_type(job: &JobFile, expected: JobType) -> Result<(), CliError> {
    if let Some(stated) = job.job_type()
        && stated != expected
    {
        return Err(CliError::run(format!("job type `{}` does not match the selected head (`{}`)", stated.as_str(), expected.as_str())));
    }
    Ok(())
}

/// Refuse export when a part sits outside the user-entered bed or work area.
pub fn refuse_off_bed_for_export(job: &JobFile) -> Result<(), CliError> {
    let messages = off_bed_warnings(job, true)?;
    if let Some(message) = messages.into_iter().next() {
        return Err(CliError::run(message));
    }
    Ok(())
}

fn apply_datum(job: &JobFile) -> Result<Option<JobFile>, CliError> {
    let Some(datum) = job.setup_hooks().datum else {
        return Ok(None);
    };
    let rect = datum_rect(job)?;
    let shift = crate::job_setup::datum_shift(datum, rect);
    if shift[0] == 0.0 && shift[1] == 0.0 {
        return Ok(None);
    }
    let mut cloned = job.clone();
    cloned.shift_placements(shift[0], shift[1]);
    Ok(Some(cloned))
}

fn datum_rect(job: &JobFile) -> Result<[f64; 4], CliError> {
    if let Some(sheet) = job.sheet() {
        return Ok([0.0, 0.0, sheet.width(), sheet.height()]);
    }
    if let Some(stock) = job.stock() {
        return Ok([stock.min().x(), stock.min().y(), stock.max().x(), stock.max().y()]);
    }
    Err(CliError::run("datum needs a sheet or a stock box"))
}

/// Warning only: parts that sit outside a named sheet are not refused at export.
fn off_sheet_warnings(job: &JobFile) -> Result<Vec<String>, CliError> {
    let Some(sheet) = job.sheet() else {
        return Ok(Vec::new());
    };
    let limits = [Some(0.0), Some(0.0), Some(sheet.width()), Some(sheet.height())];
    let mut messages = Vec::new();
    for part in job.parts() {
        let drawing = load_part_drawing(job, part)?;
        for instance in &part.instances {
            let placed = place_instance(&drawing, instance.placement, job)?;
            if let Some(bounds) = drawing_bounds(&placed)
                && !inside_limits(bounds, limits)
            {
                messages.push(format!("warning: part `{}` instance `{}` is off the sheet", part.id, instance.id));
            }
        }
    }
    Ok(messages)
}

fn off_bed_warnings(job: &JobFile, refuse: bool) -> Result<Vec<String>, CliError> {
    let Some(limits) = bed_limits(job)? else {
        return Ok(Vec::new());
    };
    let mut messages = Vec::new();
    for part in job.parts() {
        let drawing = load_part_drawing(job, part)?;
        for instance in &part.instances {
            let placed = place_instance(&drawing, instance.placement, job)?;
            if let Some(bounds) = drawing_bounds(&placed)
                && !inside_limits(bounds, limits)
            {
                let verb = if refuse { "refusing to export" } else { "warning" };
                messages.push(format!("{verb}: part `{}` instance `{}` is off the bed", part.id, instance.id));
            }
        }
    }
    Ok(messages)
}

fn unused_contour_warnings(job: &JobFile) -> Result<Vec<String>, CliError> {
    let mut messages = Vec::new();
    for part in job.parts() {
        let drawing = load_part_drawing(job, part)?;
        let ids = contour_ids(&drawing);
        for (index, item) in drawing.contours().iter().enumerate() {
            let id = ids.get(index).map(String::as_str).unwrap_or("");
            if !contour_has_operation(job, part, item.layer(), id) {
                messages.push(format!("contour `{id}` on part `{}` has no operation", part.id));
            }
        }
    }
    Ok(messages)
}

fn contour_has_operation(job: &JobFile, part: &JobPart, layer: &str, contour_id: &str) -> bool {
    job.operations().iter().any(|op| {
        let extras = job.operation_extras(op.name());
        let selector = extras.and_then(|extra| extra.selector.as_ref());
        if !selector_includes_part(selector, &part.id, op.layers()) {
            return false;
        }
        match selector {
            Some(OpSelector::Contours { ids, .. }) => ids.iter().any(|id| id == contour_id),
            Some(OpSelector::Layers { layers, .. }) => layers.iter().any(|name| name == layer),
            Some(OpSelector::Color { .. }) => false,
            Some(OpSelector::Parts { .. }) | None => op.layers().is_empty() || op.layers().iter().any(|name| name == layer),
        }
    })
}

fn lead_in_cross_warnings(job: &JobFile, document: &Document) -> Result<Vec<String>, CliError> {
    if job.parts().iter().map(|part| part.instances.len()).sum::<usize>() < 2 {
        return Ok(Vec::new());
    }
    let mut bounds = Vec::new();
    for part in job.parts() {
        let drawing = load_part_drawing(job, part)?;
        for instance in &part.instances {
            let placed = place_instance(&drawing, instance.placement, job)?;
            if let Some(box_) = drawing_bounds(&placed) {
                bounds.push((part.id.clone(), instance.id.clone(), box_));
            }
        }
    }
    let mut messages = Vec::new();
    for path in document.program().toolpaths() {
        let mut previous: Option<[f64; 2]> = None;
        for block in path.blocks() {
            let campanion_ir::ClBlock::Motion(motion) = block else { continue };
            let role = motion_role(motion);
            let to = motion_xy(motion);
            if role == Some(MoveRole::LeadIn)
                && let (Some(from), Some(to)) = (previous, to)
                && lead_in_hits_other_part(from, to, &bounds)
            {
                let name = path.name().unwrap_or("toolpath");
                messages.push(format!("lead-in on `{name}` crosses another part"));
                break;
            }
            if let Some(to) = to {
                previous = Some(to);
            }
        }
    }
    Ok(messages)
}

fn lead_in_hits_other_part(from: [f64; 2], to: [f64; 2], bounds: &[(String, String, [f64; 4])]) -> bool {
    // A lead-in that only stays inside one instance box is fine. If its segment overlaps two
    // instance boxes, it is crossing into another part.
    let hits = bounds.iter().filter(|(_, _, box_)| segment_hits_box(from, to, *box_)).count();
    hits >= 2
}

fn segment_hits_box(from: [f64; 2], to: [f64; 2], box_: [f64; 4]) -> bool {
    let min_x = from[0].min(to[0]);
    let max_x = from[0].max(to[0]);
    let min_y = from[1].min(to[1]);
    let max_y = from[1].max(to[1]);
    max_x >= box_[0] && min_x <= box_[2] && max_y >= box_[1] && min_y <= box_[3]
}

fn motion_role(motion: &campanion_ir::ClMotion) -> Option<MoveRole> {
    match motion {
        campanion_ir::ClMotion::Linear { role, .. } | campanion_ir::ClMotion::Arc { role, .. } | campanion_ir::ClMotion::Helix { role, .. } => {
            Some(*role)
        }
        _ => None,
    }
}

fn motion_xy(motion: &campanion_ir::ClMotion) -> Option<[f64; 2]> {
    let pose = match motion {
        campanion_ir::ClMotion::Linear { to, .. } | campanion_ir::ClMotion::Arc { to, .. } | campanion_ir::ClMotion::Helix { to, .. } => to,
        _ => return None,
    };
    Some([pose.tip().x(), pose.tip().y()])
}

fn load_part_drawing(job: &JobFile, part: &JobPart) -> Result<Drawing, CliError> {
    let path = join_job_path(job, part.source.path());
    let text = crate::job::read_text(&path).map_err(|err| CliError::run(err.to_string()))?;
    let options = ImportOptions::new(job.tolerance());
    let is_svg = path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("svg"));
    if is_svg { import_svg(&text, &options) } else { import_dxf(&text, &options) }.map_err(|err| CliError::run(err.to_string()))
}

fn place_instance(drawing: &Drawing, placement: Placement, job: &JobFile) -> Result<Drawing, CliError> {
    if placement.is_identity() {
        return Ok(drawing.clone());
    }
    place_with(drawing, placement.x(), placement.y(), placement.rotate_deg(), placement.mirror(), placement.scale(), &job.tolerance())
        .map_err(|err| CliError::run(err.to_string()))
}

fn drawing_bounds(drawing: &Drawing) -> Option<[f64; 4]> {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for item in drawing.contours() {
        let Ok(bounds) = item.contour().bounding_box() else { continue };
        min_x = min_x.min(bounds.min().x());
        min_y = min_y.min(bounds.min().y());
        max_x = max_x.max(bounds.max().x());
        max_y = max_y.max(bounds.max().y());
    }
    if min_x.is_finite() { Some([min_x, min_y, max_x, max_y]) } else { None }
}

fn inside_limits(bounds: [f64; 4], limits: [Option<f64>; 4]) -> bool {
    let [min_x, min_y, max_x, max_y] = bounds;
    if let Some(limit) = limits[0]
        && min_x < limit
    {
        return false;
    }
    if let Some(limit) = limits[1]
        && min_y < limit
    {
        return false;
    }
    if let Some(limit) = limits[2]
        && max_x > limit
    {
        return false;
    }
    if let Some(limit) = limits[3]
        && max_y > limit
    {
        return false;
    }
    true
}

fn bed_limits(job: &JobFile) -> Result<Option<[Option<f64>; 4]>, CliError> {
    let Some(reference) = job.machine() else {
        return Ok(None);
    };
    let store = crate::profiles::ProfileStore::from_env()?;
    let profile = crate::profiles::load_job_ref(reference, &store)?;
    Ok(limits_from_profile(&profile))
}

fn limits_from_profile(profile: &MachineFile) -> Option<[Option<f64>; 4]> {
    if let Some(bed) = profile.bed {
        return Some([Some(0.0), Some(0.0), Some(bed.x), Some(bed.y)]);
    }
    let WorkArea { x_min, x_max, y_min, y_max, .. } = profile.work_area?;
    if profile.kind == MachineKind::Resin && profile.bed.is_none() && x_min.is_none() && x_max.is_none() {
        return None;
    }
    Some([x_min, y_min, x_max, y_max])
}
