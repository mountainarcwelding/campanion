//! Multi-part planning, bed checks, and a cancellable planning handle.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};

use campanion_geom::Containment;
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
    let ordered = apply_order_rules(job)?;
    let job = ordered.as_ref().unwrap_or(job);
    let planned = if job.is_multi_part() {
        plan_multi(job, up_axis, cancel, max_power)?
    } else if job.format() == FORMAT_TAG_V2 {
        plan_single_v2(job, up_axis, cancel, max_power)?
    } else {
        run::plan_loaded_job_with(job, up_axis, cancel)?
    };
    let mut planned = apply_program_origin(job, planned)?;
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

fn apply_order_rules(job: &JobFile) -> Result<Option<JobFile>, CliError> {
    if !job.cut_order().is_empty() {
        return Ok(None);
    }
    let Some(rules) = job.setup_hooks().order_rules else {
        return Ok(None);
    };
    if !rules.inner_first && !rules.cuts_last {
        return Ok(None);
    }
    let depths = if rules.inner_first { operation_inner_depths(job)? } else { HashMap::new() };
    let mut cloned = job.clone();
    let names: Vec<String> = cloned.operations().iter().map(|op| op.name().to_string()).collect();
    cloned.sort_operations_by_key(|op| {
        let cuts = if rules.cuts_last { crate::job_setup::cuts_last_rank(&op.kind()) } else { 0 };
        let inner = if rules.inner_first { -depths.get(op.name()).copied().unwrap_or(0) } else { 0 };
        let index = names.iter().position(|name| name == op.name()).unwrap_or(usize::MAX);
        (cuts, inner, index)
    });
    Ok(Some(cloned))
}

fn operation_inner_depths(job: &JobFile) -> Result<HashMap<String, i32>, CliError> {
    let mut depths = HashMap::new();
    for part in job.parts() {
        let drawing = match load_part_drawing(job, part) {
            Ok(drawing) => drawing,
            Err(_) => continue,
        };
        let nest = contour_nesting_depths(&drawing, job.tolerance());
        let ids = contour_ids(&drawing);
        for op in job.operations() {
            let mut max_depth = 0;
            for (index, item) in drawing.contours().iter().enumerate() {
                let id = ids.get(index).map(String::as_str).unwrap_or("");
                if !operation_includes_contour(job, part, op, item.layer(), id) {
                    continue;
                }
                max_depth = max_depth.max(nest.get(index).copied().unwrap_or(0));
            }
            let entry = depths.entry(op.name().to_string()).or_insert(0);
            *entry = (*entry).max(max_depth);
        }
    }
    Ok(depths)
}

fn operation_includes_contour(job: &JobFile, part: &JobPart, op: &crate::jobfile::Operation, layer: &str, contour_id: &str) -> bool {
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
}

fn contour_nesting_depths(drawing: &Drawing, tolerance: campanion_geom::Tolerance) -> Vec<i32> {
    let contours = drawing.contours();
    let mut depths = vec![0; contours.len()];
    for (inner_i, inner) in contours.iter().enumerate() {
        let Some(sample) = contour_sample(inner.contour()) else {
            continue;
        };
        for (outer_i, outer) in contours.iter().enumerate() {
            if inner_i == outer_i || !outer.contour().is_closed() {
                continue;
            }
            if let Ok(Containment::Inside) = outer.contour().containment(sample, &tolerance) {
                depths[inner_i] += 1;
            }
        }
    }
    depths
}

fn contour_sample(contour: &campanion_geom::Contour) -> Option<campanion_geom::Point> {
    let first = contour.segments().first()?;
    Some(first.start())
}

fn apply_program_origin(job: &JobFile, planned: PlannedProgram) -> Result<PlannedProgram, CliError> {
    let hooks = job.setup_hooks();
    let stock_height = job.stock().map(|stock| (stock.max().z() - stock.min().z()).abs());
    let dz = crate::job_setup::bed_z_shift(hooks, stock_height);
    let [dx, dy] = start_from_xy_shift(hooks, job, &planned);
    if dx == 0.0 && dy == 0.0 && dz == 0.0 {
        return Ok(planned);
    }
    let document = planned.document.translate(dx, dy, dz).map_err(|err| CliError::run(err.to_string()))?;
    Ok(PlannedProgram { document, ..planned })
}

fn start_from_xy_shift(hooks: &crate::job_setup::SetupHooks, job: &JobFile, planned: &PlannedProgram) -> [f64; 2] {
    match hooks.start_from {
        Some(crate::job_setup::StartFrom::UserOrigin | crate::job_setup::StartFrom::CurrentPosition) => {}
        Some(crate::job_setup::StartFrom::Absolute) | None => return [0.0, 0.0],
    }
    let Some(anchor) = hooks.job_origin else {
        return [0.0, 0.0];
    };
    let Some(rect) = job_origin_rect(hooks, job, planned) else {
        return [0.0, 0.0];
    };
    crate::job_setup::job_origin_shift(anchor, rect)
}

fn job_origin_rect(hooks: &crate::job_setup::SetupHooks, job: &JobFile, planned: &PlannedProgram) -> Option<[f64; 4]> {
    let base = if let Some(sheet) = job.sheet() {
        Some([0.0, 0.0, sheet.width(), sheet.height()])
    } else if let Some(stock) = job.stock() {
        Some([stock.min().x(), stock.min().y(), stock.max().x(), stock.max().y()])
    } else {
        planned.document.tip_bounds_xy()
    };
    let mut rect = base?;
    if let Some(datum) = hooks.datum
        && (job.sheet().is_some() || job.stock().is_some())
    {
        // `apply_datum` already shifted placements by this amount; move the stock/sheet
        // rectangle into the same program frame.
        let unshifted = if let Some(sheet) = job.sheet() {
            [0.0, 0.0, sheet.width(), sheet.height()]
        } else if let Some(stock) = job.stock() {
            [stock.min().x(), stock.min().y(), stock.max().x(), stock.max().y()]
        } else {
            rect
        };
        let shift = crate::job_setup::datum_shift(datum, unshifted);
        rect = [rect[0] + shift[0], rect[1] + shift[1], rect[2] + shift[0], rect[3] + shift[1]];
    }
    Some(rect)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::job_setup::{JobOriginAnchor, OrderRules, StartFrom, ZZero};
    use crate::jobfile;
    use std::path::PathBuf;

    fn examples_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples")
    }

    fn load_example(name: &str) -> JobFile {
        let path = examples_dir().join(name);
        let text = std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("{path:?}: {err}"));
        jobfile::parse(&text, &path).unwrap_or_else(|err| panic!("{err}"))
    }

    fn parse_with(text: &str, path: &std::path::Path) -> JobFile {
        jobfile::parse(text, path).unwrap_or_else(|err| panic!("{err}"))
    }

    fn plan(job: &JobFile) -> PlannedProgram {
        plan_job_inner(job, step::UpAxis::Z, None).unwrap_or_else(|err| panic!("{err}"))
    }

    fn first_cut_xyz(document: &Document) -> [f64; 3] {
        for path in document.program().toolpaths() {
            for block in path.blocks() {
                let campanion_ir::ClBlock::Motion(motion) = block else { continue };
                if motion_role(motion) != Some(MoveRole::Cut) {
                    continue;
                }
                let Some(xy) = motion_xy(motion) else { continue };
                let z = match motion {
                    campanion_ir::ClMotion::Linear { to, .. } | campanion_ir::ClMotion::Arc { to, .. } | campanion_ir::ClMotion::Helix { to, .. } => {
                        to.tip().z()
                    }
                    _ => continue,
                };
                return [xy[0], xy[1], z];
            }
        }
        panic!("no cut motion");
    }

    fn toolpath_names(document: &Document) -> Vec<String> {
        document.program().toolpaths().iter().filter_map(|path| path.name().map(str::to_string)).collect()
    }

    #[test]
    fn omitted_hooks_leave_laser_rect_z_at_stock_top() {
        let job = load_example("laser_rect.job.json");
        let [_, _, z] = first_cut_xyz(&plan(&job).document);
        // Stock-top convention: the cut sits at or below Z 0, never raised by a bed shift.
        assert!(z <= 0.0, "cut z {z}");
    }

    #[test]
    fn z_zero_bed_raises_posted_z_by_stock_thickness() {
        let base = load_example("laser_rect.job.json");
        let baseline = first_cut_xyz(&plan(&base).document);
        let path = examples_dir().join("laser_rect.job.json");
        let text = r#"{"format":"campanion-job/2","units":"mm","parts":[{"id":"p","source":{"path":"laser_rect.svg","hash":"7325d5be1c8f0420"}}],"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"outline","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}}],"z_zero":"bed","stock_thickness":6}"#.to_string();
        let job = parse_with(&text, &path);
        assert_eq!(job.setup_hooks().z_zero, Some(ZZero::Bed));
        let raised = first_cut_xyz(&plan(&job).document);
        assert!((raised[2] - (baseline[2] + 6.0)).abs() < 1e-9, "baseline z {} raised {}", baseline[2], raised[2]);
        assert!((raised[0] - baseline[0]).abs() < 1e-9);
        assert!((raised[1] - baseline[1]).abs() < 1e-9);
    }

    #[test]
    fn start_from_user_origin_shifts_xy_to_job_origin() {
        let path = examples_dir().join("laser_rect.job.json");
        let text = r#"{"format":"campanion-job/2","units":"mm","parts":[{"id":"p","source":{"path":"laser_rect.svg","hash":"7325d5be1c8f0420"}}],"sheet":{"width":200,"height":100},"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"outline","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}}],"start_from":"user_origin","job_origin":"max_max"}"#.to_string();
        let job = parse_with(&text, &path);
        assert_eq!(job.setup_hooks().start_from, Some(StartFrom::UserOrigin));
        assert_eq!(job.setup_hooks().job_origin, Some(JobOriginAnchor::MaxMax));
        let [x, y, _] = first_cut_xyz(&plan(&job).document);
        // The unshifted outline lives in the first quadrant; max_max of a 200×100 sheet is (200, 100).
        assert!(x < 0.0, "x {x} should be shifted left of the max-max origin");
        assert!(y < 0.0, "y {y} should be shifted below the max-max origin");
    }

    #[test]
    fn order_rules_cuts_last_puts_engrave_before_profile() {
        let path = examples_dir().join("laser_rect.job.json");
        let text = r#"{"format":"campanion-job/2","units":"mm","parts":[{"id":"p","source":{"path":"laser_rect.svg","hash":"7325d5be1c8f0420"}}],"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}},{"name":"mark","type":"engrave","layers":["0"],"tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}}],"order_rules":{"inner_first":false,"cuts_last":true}}"#.to_string();
        let job = parse_with(&text, &path);
        assert_eq!(job.setup_hooks().order_rules, Some(OrderRules { inner_first: false, cuts_last: true }));
        let names = toolpath_names(&plan(&job).document);
        let mark = names.iter().position(|name| name.contains("mark")).expect("mark");
        let cut = names.iter().position(|name| name.contains("cut")).expect("cut");
        assert!(mark < cut, "names {names:?}");
    }

    #[test]
    fn explicit_cut_order_wins_over_order_rules() {
        let path = examples_dir().join("laser_rect.job.json");
        let text = r#"{"format":"campanion-job/2","units":"mm","parts":[{"id":"p","source":{"path":"laser_rect.svg","hash":"7325d5be1c8f0420"}}],"tools":[{"id":1,"kerf":0.1}],"operations":[{"name":"cut","type":"profile","layers":["0"],"side":"outside","tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}},{"name":"mark","type":"engrave","layers":["0"],"tool_id":1,"depth":0.26,"step_down":0.26,"feeds":{"cut":300.0}}],"cut_order":["cut","mark"],"order_rules":{"inner_first":false,"cuts_last":true}}"#.to_string();
        let job = parse_with(&text, &path);
        let names = toolpath_names(&plan(&job).document);
        let mark = names.iter().position(|name| name.contains("mark")).expect("mark");
        let cut = names.iter().position(|name| name.contains("cut")).expect("cut");
        assert!(cut < mark, "names {names:?}");
    }
}
