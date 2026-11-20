//! Setup layout tools: linked copies, array, arrange, align, mirror and rotate.
//!
//! Numeric fields stay empty until the user types a value. Arrange uses a gap of 0 when the
//! gap field is left blank, and says so. No machine bed or sheet size is invented.

use campanion_geom::Tolerance;
use campanion_import::{Drawing, place_with};

use crate::session::{InstanceDraft, JobSession, PartDraft, TreeSelection, format_number, parse_optional_number, warning};

/// A world-space axis-aligned box: `[min_x, min_y, max_x, max_y]`.
pub type Bounds = [f64; 4];

/// How selected instances should be aligned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignMode {
    Left,
    Right,
    Bottom,
    Top,
    CenterX,
    CenterY,
}

/// Parsed array parameters. Counts must be whole numbers of at least 1.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArraySpec {
    pub columns: u32,
    pub rows: u32,
    pub dx: f64,
    pub dy: f64,
}

impl ArraySpec {
    /// Parse typed fields. Empty values are refused rather than defaulted, except that a
    /// missing spacing may be zero only when the caller passes `allow_zero_spacing`.
    pub fn parse(columns: &str, rows: &str, dx: &str, dy: &str) -> Result<Self, String> {
        let columns = parse_count(columns, "array columns")?;
        let rows = parse_count(rows, "array rows")?;
        if columns == 1 && rows == 1 {
            return Err("an array needs more than one cell".to_string());
        }
        Ok(Self { columns, rows, dx: parse_optional_number(dx, "array spacing X")?, dy: parse_optional_number(dy, "array spacing Y")? })
    }
}

fn parse_count(text: &str, what: &str) -> Result<u32, String> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err(format!("enter {what}"));
    }
    let value: f64 = trimmed.parse().map_err(|_| format!("{what} is not a number"))?;
    if !value.is_finite() || value < 1.0 || value.fract() != 0.0 || value > 1_000.0 {
        return Err(format!("{what} must be a whole number in 1..=1000"));
    }
    Ok(value as u32)
}

/// World bounds of a drawing after a placement is applied.
pub fn placed_bounds(drawing: &Drawing, instance: &InstanceDraft) -> Result<Option<Bounds>, String> {
    let placed = placed_drawing_of(drawing, instance)?;
    Ok(drawing_bounds(&placed))
}

/// Apply an instance placement to a drawing.
pub fn placed_drawing_of(drawing: &Drawing, instance: &InstanceDraft) -> Result<Drawing, String> {
    let placement = instance.to_placement()?;
    if placement.is_identity() {
        return Ok(drawing.clone());
    }
    place_with(drawing, placement.x(), placement.y(), placement.rotate_deg(), placement.mirror(), placement.scale(), &Tolerance::default())
        .map_err(|err| err.to_string())
}

/// Axis-aligned bounds of a drawing, or `None` when it has no finite geometry.
pub fn drawing_bounds(drawing: &Drawing) -> Option<Bounds> {
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

/// True when `bounds` sits entirely inside `limits` (each end optional).
#[must_use]
pub fn inside_limits(bounds: Bounds, limits: [Option<f64>; 4]) -> bool {
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

/// Next unused instance id on a part (`1`, `2`, …).
#[must_use]
pub fn next_instance_id(part: &PartDraft) -> String {
    let mut n = part.instances.len() + 1;
    loop {
        let id = n.to_string();
        if !part.instances.iter().any(|item| item.id == id) {
            return id;
        }
        n += 1;
    }
}

/// Next unused part id, based on a suggested stem.
#[must_use]
pub fn next_part_id(session: &JobSession, suggested: &str) -> String {
    let stem = sanitize_id(suggested);
    if !session.parts.iter().any(|part| part.id == stem) {
        return stem;
    }
    let mut n = 2;
    loop {
        let id = format!("{stem}-{n}");
        if !session.parts.iter().any(|part| part.id == id) {
            return id;
        }
        n += 1;
    }
}

fn sanitize_id(suggested: &str) -> String {
    let mut out = String::new();
    for ch in suggested.chars() {
        if ch.is_ascii_alphanumeric() || ch == '-' || ch == '_' {
            out.push(ch);
        } else if ch == ' ' && !out.ends_with('-') {
            out.push('-');
        }
    }
    let trimmed = out.trim_matches('-');
    if trimmed.is_empty() { "part".to_string() } else { trimmed.to_string() }
}

/// Indices of the current selection: every selected instance, or every instance when none is selected.
#[must_use]
pub fn target_instances(session: &JobSession) -> Vec<(usize, usize)> {
    match session.selected {
        Some(TreeSelection::Instance(part, inst)) => vec![(part, inst)],
        Some(TreeSelection::Part(part)) => {
            let n = session.parts.get(part).map(|item| item.instances.len()).unwrap_or(0);
            (0..n).map(|inst| (part, inst)).collect()
        }
        Some(TreeSelection::Source) => {
            session.parts.first().map(|part| (0..part.instances.len()).map(|inst| (0, inst)).collect()).unwrap_or_default()
        }
        _ => {
            let mut all = Vec::new();
            for (part_index, part) in session.parts.iter().enumerate() {
                for inst in 0..part.instances.len() {
                    all.push((part_index, inst));
                }
            }
            all
        }
    }
}

/// Add a linked copy of the selected (or first) instance, offset by the instance width so it
/// does not sit on top of the original. No extra gap is invented.
pub fn linked_copy(session: &mut JobSession) -> Result<String, String> {
    let (part_index, inst_index) = primary_target(session)?;
    session.commit_placement();
    let drawing = session.parts[part_index].drawing.clone();
    let source = session.parts[part_index].instances[inst_index].clone();
    let offset = match drawing.as_ref() {
        Some(drawing) => placed_bounds(drawing, &source)?.map(|b| (b[2] - b[0]).max(0.0)).unwrap_or(0.0),
        None => 0.0,
    };
    let mut copy = source;
    copy.id = next_instance_id(&session.parts[part_index]);
    let x = parse_optional_number(&copy.x, "placement X")?;
    copy.x = format_number(x + offset);
    let status = format!("Linked copy `{}` on part `{}`", copy.id, session.parts[part_index].id);
    session.parts[part_index].instances.push(copy);
    let inst = session.parts[part_index].instances.len() - 1;
    session.select_instance(part_index, inst);
    session.dirty = true;
    Ok(status)
}

fn primary_target(session: &JobSession) -> Result<(usize, usize), String> {
    match session.selected {
        Some(TreeSelection::Instance(part, inst)) if session.parts.get(part).and_then(|item| item.instances.get(inst)).is_some() => Ok((part, inst)),
        Some(TreeSelection::Part(part)) if session.parts.get(part).is_some_and(|item| !item.instances.is_empty()) => Ok((part, 0)),
        Some(TreeSelection::Source) if session.parts.first().is_some_and(|item| !item.instances.is_empty()) => Ok((0, 0)),
        _ => {
            if session.parts.first().is_some_and(|item| !item.instances.is_empty()) {
                Ok((0, 0))
            } else {
                Err("import a part before making a copy".to_string())
            }
        }
    }
}

/// Create a grid of linked copies of the selected instance. The original stays at its placement;
/// additional cells are offset by `dx`, `dy`.
pub fn array_copies(session: &mut JobSession, spec: ArraySpec) -> Result<String, String> {
    let (part_index, inst_index) = primary_target(session)?;
    session.commit_placement();
    let origin = session.parts[part_index].instances[inst_index].clone();
    let ox = parse_optional_number(&origin.x, "placement X")?;
    let oy = parse_optional_number(&origin.y, "placement Y")?;
    let mut created = 0u32;
    for row in 0..spec.rows {
        for col in 0..spec.columns {
            if row == 0 && col == 0 {
                continue;
            }
            let mut copy = origin.clone();
            copy.id = next_instance_id(&session.parts[part_index]);
            copy.x = format_number(ox + f64::from(col) * spec.dx);
            copy.y = format_number(oy + f64::from(row) * spec.dy);
            session.parts[part_index].instances.push(copy);
            created += 1;
        }
    }
    session.dirty = true;
    Ok(format!("Array added {created} linked copies on part `{}`", session.parts[part_index].id))
}

/// Pack target instances left-to-right, wrapping at the sheet or bed width when one is known.
/// An empty gap is treated as 0 and a note is recorded.
pub fn arrange(session: &mut JobSession, gap_text: &str, pack_width: Option<f64>) -> Result<String, String> {
    session.commit_placement();
    let mut targets = target_instances(session);
    if targets.len() < 2 {
        targets.clear();
        for (part_index, part) in session.parts.iter().enumerate() {
            for inst in 0..part.instances.len() {
                targets.push((part_index, inst));
            }
        }
    }
    if targets.is_empty() {
        return Err("import a part before arranging".to_string());
    }
    let gap = parse_optional_number(gap_text, "arrange gap")?;
    let mut gap_note = false;
    if gap_text.trim().is_empty() {
        gap_note = true;
        session.messages.push(warning("arrange gap was empty, so 0 mm was used".to_string()));
    }
    let mut items: Vec<(usize, usize, Bounds)> = Vec::new();
    for &(part, inst) in &targets {
        let Some(drawing) = session.parts[part].drawing.as_ref() else {
            return Err(format!("part `{}` has no 2D drawing to arrange", session.parts[part].id));
        };
        let instance = &session.parts[part].instances[inst];
        let Some(bounds) = placed_bounds(drawing, instance)? else {
            continue;
        };
        items.push((part, inst, bounds));
    }
    if items.is_empty() {
        return Err("no geometry to arrange".to_string());
    }
    items.sort_by(|a, b| {
        let ha = a.2[3] - a.2[1];
        let hb = b.2[3] - b.2[1];
        hb.partial_cmp(&ha).unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut cursor_x = 0.0;
    let mut cursor_y = 0.0;
    let mut row_h = 0.0;
    for (part, inst, bounds) in items {
        let width = bounds[2] - bounds[0];
        let height = bounds[3] - bounds[1];
        if let Some(limit) = pack_width
            && cursor_x > 0.0
            && cursor_x + width > limit
        {
            cursor_x = 0.0;
            cursor_y += row_h + gap;
            row_h = 0.0;
        }
        let dx = cursor_x - bounds[0];
        let dy = cursor_y - bounds[1];
        let instance = &mut session.parts[part].instances[inst];
        let x = parse_optional_number(&instance.x, "placement X")?;
        let y = parse_optional_number(&instance.y, "placement Y")?;
        instance.x = format_number(x + dx);
        instance.y = format_number(y + dy);
        cursor_x += width + gap;
        row_h = row_h.max(height);
    }
    session.sync_placement_from_selection();
    session.dirty = true;
    let message = if gap_note { "Arranged parts with a 0 mm gap (the gap field was empty)".to_string() } else { "Arranged parts".to_string() };
    Ok(message)
}

/// Align target instances. A single instance is aligned to the bed or sheet when `frame` is set.
pub fn align(session: &mut JobSession, mode: AlignMode, frame: Option<Bounds>) -> Result<String, String> {
    session.commit_placement();
    let targets = target_instances(session);
    if targets.is_empty() {
        return Err("import a part before aligning".to_string());
    }
    let mut boxes: Vec<(usize, usize, Bounds)> = Vec::new();
    for &(part, inst) in &targets {
        let Some(drawing) = session.parts[part].drawing.as_ref() else {
            continue;
        };
        if let Some(bounds) = placed_bounds(drawing, &session.parts[part].instances[inst])? {
            boxes.push((part, inst, bounds));
        }
    }
    if boxes.is_empty() {
        return Err("no geometry to align".to_string());
    }
    let guide = if boxes.len() == 1 {
        frame.ok_or_else(|| "select more than one part, or enter a bed or sheet to align to".to_string())?
    } else {
        union_bounds(boxes.iter().map(|item| item.2))
    };
    for (part, inst, bounds) in boxes {
        let (dx, dy) = align_delta(mode, bounds, guide);
        let instance = &mut session.parts[part].instances[inst];
        let x = parse_optional_number(&instance.x, "placement X")?;
        let y = parse_optional_number(&instance.y, "placement Y")?;
        instance.x = format_number(x + dx);
        instance.y = format_number(y + dy);
    }
    session.sync_placement_from_selection();
    session.dirty = true;
    Ok("Aligned parts".to_string())
}

fn union_bounds(items: impl Iterator<Item = Bounds>) -> Bounds {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for [a, b, c, d] in items {
        min_x = min_x.min(a);
        min_y = min_y.min(b);
        max_x = max_x.max(c);
        max_y = max_y.max(d);
    }
    [min_x, min_y, max_x, max_y]
}

fn align_delta(mode: AlignMode, bounds: Bounds, guide: Bounds) -> (f64, f64) {
    match mode {
        AlignMode::Left => (guide[0] - bounds[0], 0.0),
        AlignMode::Right => (guide[2] - bounds[2], 0.0),
        AlignMode::Bottom => (0.0, guide[1] - bounds[1]),
        AlignMode::Top => (0.0, guide[3] - bounds[3]),
        AlignMode::CenterX => (((guide[0] + guide[2]) - (bounds[0] + bounds[2])) / 2.0, 0.0),
        AlignMode::CenterY => (0.0, ((guide[1] + guide[3]) - (bounds[1] + bounds[3])) / 2.0),
    }
}

/// Toggle mirror on the targeted instances.
pub fn mirror_targets(session: &mut JobSession) -> Result<String, String> {
    session.commit_placement();
    let targets = target_instances(session);
    if targets.is_empty() {
        return Err("import a part before mirroring".to_string());
    }
    for (part, inst) in targets {
        let instance = &mut session.parts[part].instances[inst];
        instance.mirror = !instance.mirror;
    }
    session.sync_placement_from_selection();
    session.dirty = true;
    Ok("Mirrored the selected parts".to_string())
}

/// Add `degrees` (counter-clockwise) to the targeted instances' rotation.
pub fn rotate_targets(session: &mut JobSession, degrees_text: &str) -> Result<String, String> {
    let trimmed = degrees_text.trim();
    if trimmed.is_empty() {
        return Err("enter a rotation in degrees".to_string());
    }
    let degrees = parse_optional_number(trimmed, "rotation")?;
    session.commit_placement();
    let targets = target_instances(session);
    if targets.is_empty() {
        return Err("import a part before rotating".to_string());
    }
    for (part, inst) in targets {
        let instance = &mut session.parts[part].instances[inst];
        let current = parse_optional_number(&instance.rotate, "placement rotate_deg")?;
        instance.rotate = format_number(current + degrees);
    }
    session.sync_placement_from_selection();
    session.dirty = true;
    Ok(format!("Rotated the selected parts by {} deg", format_number(degrees)))
}

/// Move the targeted instance so its minimum corner sits at the origin (a 2D place-on-face).
pub fn place_at_origin(session: &mut JobSession) -> Result<String, String> {
    session.commit_placement();
    let (part, inst) = primary_target(session)?;
    let Some(kind) = session.parts[part].source_kind else {
        return place_min_corner(session, part, inst);
    };
    if matches!(kind, campanion_cli::SourceKind::Stl | campanion_cli::SourceKind::Step) && session.parts[part].drawing.is_none() {
        return Err("3D face placement is not in the job format yet; only a 2D footprint can be moved in X and Y".to_string());
    }
    place_min_corner(session, part, inst)
}

fn place_min_corner(session: &mut JobSession, part: usize, inst: usize) -> Result<String, String> {
    let Some(drawing) = session.parts[part].drawing.clone() else {
        return Err("the selected part has no 2D drawing".to_string());
    };
    let Some(bounds) = placed_bounds(&drawing, &session.parts[part].instances[inst])? else {
        return Err("the selected part has no geometry to place".to_string());
    };
    let instance = &mut session.parts[part].instances[inst];
    let x = parse_optional_number(&instance.x, "placement X")?;
    let y = parse_optional_number(&instance.y, "placement Y")?;
    instance.x = format_number(x - bounds[0]);
    instance.y = format_number(y - bounds[1]);
    session.select_instance(part, inst);
    session.dirty = true;
    Ok("Placed the part at the origin".to_string())
}

/// Sheet rectangle from typed width/height, starting at the origin, when both values are set.
pub fn sheet_bounds(session: &JobSession) -> Result<Option<Bounds>, String> {
    let w = session.sheet_width.trim();
    let h = session.sheet_height.trim();
    if w.is_empty() && h.is_empty() {
        return Ok(None);
    }
    if w.is_empty() || h.is_empty() {
        return Err("enter both sheet width and height, or leave both empty".to_string());
    }
    let width = parse_optional_number(w, "sheet width")?;
    let height = parse_optional_number(h, "sheet height")?;
    if width <= 0.0 || height <= 0.0 {
        return Err("sheet width and height must be greater than zero".to_string());
    }
    Ok(Some([0.0, 0.0, width, height]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_cli::SourceKind;
    use std::fs;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT: AtomicU64 = AtomicU64::new(0);

    const PART_SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" width="20mm" height="10mm" viewBox="0 0 20 10"><rect x="0" y="0" width="20" height="10" fill="none" stroke="black"/></svg>"#;

    fn temp() -> PathBuf {
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("campanion-gui-setup-{n}"));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("dir");
        dir
    }

    fn session_with_part() -> JobSession {
        let dir = temp();
        fs::write(dir.join("part.svg"), PART_SVG).expect("svg");
        let imported = crate::session::import_part_file(&dir.join("part.svg")).expect("import");
        crate::session::session_from_import(&imported)
    }

    #[test]
    fn linked_copy_offsets_by_width() {
        let mut session = session_with_part();
        let status = linked_copy(&mut session).expect("copy");
        assert!(status.contains("Linked copy"), "{status}");
        assert_eq!(session.parts[0].instances.len(), 2);
        let x = parse_optional_number(&session.parts[0].instances[1].x, "x").unwrap();
        assert!((x - 20.0).abs() < 1e-6, "offset {x}");
    }

    #[test]
    fn array_writes_a_grid() {
        let mut session = session_with_part();
        let spec = ArraySpec::parse("2", "2", "25", "15").expect("spec");
        array_copies(&mut session, spec).expect("array");
        assert_eq!(session.parts[0].instances.len(), 4);
        let xs: Vec<f64> = session.parts[0].instances.iter().map(|i| parse_optional_number(&i.x, "x").unwrap()).collect();
        let ys: Vec<f64> = session.parts[0].instances.iter().map(|i| parse_optional_number(&i.y, "y").unwrap()).collect();
        assert!(xs.contains(&0.0) && xs.contains(&25.0), "{xs:?}");
        assert!(ys.contains(&0.0) && ys.contains(&15.0), "{ys:?}");
    }

    #[test]
    fn arrange_empty_gap_is_zero_and_notes() {
        let mut session = session_with_part();
        linked_copy(&mut session).expect("copy");
        let status = arrange(&mut session, "", None).expect("arrange");
        assert!(status.contains("0 mm"), "{status}");
        assert!(session.messages.iter().any(|m| m.text.contains("0 mm")));
        let xs: Vec<f64> = session.parts[0].instances.iter().map(|item| parse_optional_number(&item.x, "x").unwrap()).collect();
        assert!(xs.iter().any(|x| (x - 0.0).abs() < 1e-6) && xs.iter().any(|x| (x - 20.0).abs() < 1e-6), "arranged xs {xs:?}");
    }

    #[test]
    fn array_refuses_empty_counts() {
        let err = ArraySpec::parse("", "2", "10", "10").expect_err("count");
        assert!(err.contains("columns"), "{err}");
    }

    #[test]
    fn empty_profile_bed_is_not_invented() {
        let session = JobSession::new();
        assert!(session.parts.is_empty());
        assert!(sheet_bounds(&session).expect("sheet").is_none());
    }

    fn example_path(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples").join(name)
    }

    #[test]
    fn import_three_parts_array_and_linked_copy_populate_the_list() {
        let mut session = JobSession::new();
        for name in ["sample.svg", "curves.svg", "nest.dxf"] {
            let imported = crate::session::import_part_file(&example_path(name)).unwrap_or_else(|err| panic!("{name}: {err}"));
            crate::session::add_imported_part(&mut session, &imported);
        }
        assert_eq!(session.parts.len(), 3);
        session.select_instance(0, 0);
        array_copies(&mut session, ArraySpec::parse("3", "2", "110", "70").expect("spec")).expect("array");
        session.select_instance(2, 0);
        linked_copy(&mut session).expect("copy");
        assert_eq!(session.parts[0].instances.len(), 6, "3x2 array of the first part");
        assert_eq!(session.parts[1].instances.len(), 1);
        assert_eq!(session.parts[2].instances.len(), 2, "linked copy of the third part");
        let scene = crate::session::placed_scene_parts(&session, 0.05).expect("scene").expect("geometry");
        let (min, max) = scene.bounds.expect("bounds");
        assert!(max[0] - min[0] > 200.0, "array should span X {min:?} {max:?}");
        assert!(max[1] - min[1] > 70.0, "array should span Y {min:?} {max:?}");
    }

    #[test]
    fn source_kind_is_used_for_place_on_face_message() {
        let mut session = JobSession::new();
        session.parts.push(PartDraft {
            id: "mesh".to_string(),
            source_path: Some(PathBuf::from("part.stl")),
            source_hash: Some("0".repeat(16)),
            source_kind: Some(SourceKind::Stl),
            layers: Vec::new(),
            drawing: None,
            instances: vec![InstanceDraft::identity("1")],
        });
        session.selected = Some(TreeSelection::Instance(0, 0));
        let err = place_at_origin(&mut session).expect_err("stl");
        assert!(err.contains("job format"), "{err}");
    }
}
