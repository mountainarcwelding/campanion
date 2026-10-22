//! Group-code reader for ASCII DXF.
//!
//! Pairs are a code line then a value line. Carriage returns are stripped. Geometry is scaled
//! from `$INSUNITS` into millimetres and handed to [`chain_paths`] when it is open.

use std::collections::{BTreeMap, HashMap};

use campanion_geom::{Arc, Contour, Point, Polyline, Segment, Tolerance, Vertex};

use crate::chain::chain_paths;
use crate::units::mm_per_unit;
use crate::{
    Drawing, ImportError, ImportOptions, ImportedContour, MAX_COORD_MM, MAX_DXF_BYTES, MAX_ENTITY_SEGMENTS, MAX_TOTAL_SEGMENTS, Result, invalid,
    invalid_at, unsupported,
};

const PI: f64 = std::f64::consts::PI;
const TAU: f64 = std::f64::consts::TAU;
const MAX_BODY: usize = MAX_ENTITY_SEGMENTS * 8 + 128;

/// Read one ASCII DXF into millimetre contours.
pub fn import_dxf(text: &str, options: &ImportOptions) -> Result<Drawing> {
    if text.len() > MAX_DXF_BYTES {
        return Err(invalid(format!("DXF exceeds {MAX_DXF_BYTES} bytes")));
    }
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let scale = read_insunits(text)?;
    read_entities(text, scale, options)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Top,
    Header,
    Entities,
    Skip,
}

struct Group {
    code: i32,
    value: String,
    line: usize,
}

struct Reader<'a> {
    rest: &'a str,
    number: usize,
    pushed: Option<Group>,
}

impl<'a> Reader<'a> {
    fn new(text: &'a str) -> Self {
        Self { rest: text, number: 0, pushed: None }
    }

    fn push(&mut self, group: Group) -> Result<()> {
        if self.pushed.is_some() {
            return Err(invalid("internal: group already buffered"));
        }
        self.pushed = Some(group);
        Ok(())
    }

    fn next_group(&mut self) -> Result<Option<Group>> {
        if let Some(group) = self.pushed.take() {
            return Ok(Some(group));
        }
        let Some((code_line, code_text)) = self.next_text()? else {
            return Ok(None);
        };
        if code_text.is_empty() {
            return Err(invalid_at(code_line, "empty group code"));
        }
        let code = code_text.parse::<i32>().map_err(|_| invalid_at(code_line, format!("group code '{code_text}' is not an integer")))?;
        let Some((value_line, value)) = self.next_text()? else {
            return Err(invalid_at(code_line, "group code has no value"));
        };
        Ok(Some(Group { code, value, line: value_line }))
    }

    fn next_text(&mut self) -> Result<Option<(usize, String)>> {
        let Some(raw) = next_physical_line(&mut self.rest) else {
            return Ok(None);
        };
        self.number = self.number.checked_add(1).ok_or_else(|| invalid("too many lines"))?;
        Ok(Some((self.number, raw.trim().to_string())))
    }
}

fn next_physical_line<'a>(rest: &mut &'a str) -> Option<&'a str> {
    if rest.is_empty() {
        return None;
    }
    let end = match rest.find(['\n', '\r']) {
        Some(index) => index,
        None => rest.len(),
    };
    let line = &rest[..end];
    let after = &rest[end..];
    if let Some(stripped) = after.strip_prefix("\r\n") {
        *rest = stripped;
    } else if let Some(stripped) = after.strip_prefix('\n') {
        *rest = stripped;
    } else if let Some(stripped) = after.strip_prefix('\r') {
        *rest = stripped;
    } else {
        *rest = "";
    }
    Some(line)
}

fn read_insunits(text: &str) -> Result<f64> {
    let mut reader = Reader::new(text);
    let mut mode = Mode::Top;
    let mut armed = false;
    let mut code = 0i32;
    while let Some(group) = reader.next_group()? {
        match mode {
            Mode::Top => match (group.code, group.value.as_str()) {
                (0, "SECTION") => mode = section_mode(&mut reader)?,
                (0, "EOF") => break,
                (0, "ENDSEC") => {}
                (0, other) => return Err(invalid_at(group.line, format!("unexpected record {other}"))),
                _ => {}
            },
            Mode::Header => match (group.code, group.value.as_str()) {
                (0, "ENDSEC") => {
                    mode = Mode::Top;
                    armed = false;
                }
                (0, "EOF") => break,
                (9, "$INSUNITS") => armed = true,
                (9, _) => armed = false,
                (70, _) if armed => {
                    code = parse_i32(&group.value, group.line)?;
                    armed = false;
                }
                _ => {}
            },
            Mode::Skip | Mode::Entities => {
                if group.code == 0 && group.value == "ENDSEC" {
                    mode = Mode::Top;
                } else if group.code == 0 && group.value == "EOF" {
                    break;
                }
            }
        }
    }
    mm_per_unit(code)
}

fn section_mode(reader: &mut Reader<'_>) -> Result<Mode> {
    let Some(name) = reader.next_group()? else {
        return Err(invalid("SECTION has no name"));
    };
    if name.code != 2 {
        return Err(invalid_at(name.line, "SECTION name must be group 2"));
    }
    Ok(match name.value.as_str() {
        "HEADER" => Mode::Header,
        "ENTITIES" => Mode::Entities,
        _ => Mode::Skip,
    })
}

struct Opens {
    order: Vec<String>,
    paths: HashMap<String, Vec<Vec<Segment>>>,
}

impl Opens {
    fn push(&mut self, layer: String, segments: Vec<Segment>) {
        match self.paths.entry(layer) {
            std::collections::hash_map::Entry::Vacant(slot) => {
                self.order.push(slot.key().clone());
                slot.insert(vec![segments]);
            }
            std::collections::hash_map::Entry::Occupied(mut slot) => slot.get_mut().push(segments),
        }
    }
}

fn read_entities(text: &str, scale: f64, options: &ImportOptions) -> Result<Drawing> {
    let tolerance = options.tolerance();
    let mut reader = Reader::new(text);
    let mut mode = Mode::Top;
    let mut saw_entities = false;
    let mut skipped = 0usize;
    let mut skipped_types = BTreeMap::new();
    let mut total = 0usize;
    let mut closed = Vec::new();
    let mut open = Opens { order: Vec::new(), paths: HashMap::new() };
    while let Some(group) = reader.next_group()? {
        match mode {
            Mode::Top => match (group.code, group.value.as_str()) {
                (0, "SECTION") => {
                    mode = section_mode(&mut reader)?;
                    saw_entities |= mode == Mode::Entities;
                }
                (0, "EOF") => break,
                (0, "ENDSEC") => {}
                (0, other) => return Err(invalid_at(group.line, format!("unexpected record {other}"))),
                _ => {}
            },
            Mode::Header | Mode::Skip => {
                if group.code == 0 && group.value == "ENDSEC" {
                    mode = Mode::Top;
                } else if group.code == 0 && group.value == "EOF" {
                    break;
                }
            }
            Mode::Entities => {
                if group.code != 0 {
                    return Err(invalid_at(group.line, "expected an entity type"));
                }
                match group.value.as_str() {
                    "ENDSEC" => mode = Mode::Top,
                    "EOF" => break,
                    "SECTION" => return Err(invalid_at(group.line, "SECTION inside ENTITIES")),
                    "POLYLINE" => {
                        skip_polyline(&mut reader)?;
                        skipped = note_skipped(skipped, &mut skipped_types, "POLYLINE")?;
                    }
                    "LINE" | "ARC" | "CIRCLE" | "LWPOLYLINE" => {
                        let kind = group.value.clone();
                        let body = take_body(&mut reader, true)?;
                        let built = build_entity(&kind, &body, scale, &tolerance, group.line)?;
                        store(&mut closed, &mut open, &mut total, built, &tolerance, group.line)?;
                    }
                    _ => {
                        take_body(&mut reader, false)?;
                        skipped = note_skipped(skipped, &mut skipped_types, &group.value)?;
                    }
                }
            }
        }
    }
    if !saw_entities {
        return Err(invalid("DXF has no ENTITIES section"));
    }
    for layer in std::mem::take(&mut open.order) {
        let Some(paths) = open.paths.remove(&layer) else {
            return Err(invalid("internal: missing layer"));
        };
        let mut before = 0usize;
        for path in &paths {
            before = before.checked_add(path.len()).ok_or_else(|| invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")))?;
        }
        let contours = chain_paths(paths, options.gap(), &tolerance)?;
        let mut after = 0usize;
        for contour in &contours {
            after = after.checked_add(contour.segments().len()).ok_or_else(|| invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")))?;
        }
        if after > before {
            account(&mut total, after - before)?;
        }
        for contour in contours {
            closed.push(ImportedContour::new(layer.clone(), contour));
        }
    }
    Ok(Drawing::new(closed, skipped, skipped_types.into_iter().collect()))
}

fn count_skipped(skipped: usize) -> Result<usize> {
    skipped.checked_add(1).ok_or_else(|| invalid("too many skipped entities"))
}

fn note_skipped(skipped: usize, types: &mut BTreeMap<String, usize>, kind: &str) -> Result<usize> {
    let skipped = count_skipped(skipped)?;
    if let Some(count) = types.get_mut(kind) {
        *count = count.checked_add(1).ok_or_else(|| invalid("too many skipped entities"))?;
    } else {
        types.insert(kind.to_string(), 1);
    }
    Ok(skipped)
}

fn account(total: &mut usize, count: usize) -> Result<()> {
    let next = total.checked_add(count).ok_or_else(|| invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")))?;
    if next > MAX_TOTAL_SEGMENTS {
        return Err(invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments")));
    }
    *total = next;
    Ok(())
}

fn store(closed: &mut Vec<ImportedContour>, open: &mut Opens, total: &mut usize, built: Built, tolerance: &Tolerance, line: usize) -> Result<()> {
    let count = built.segments.len();
    if count == 0 {
        return Err(invalid_at(line, "entity produced no geometry"));
    }
    if count > MAX_ENTITY_SEGMENTS {
        return Err(invalid_at(line, format!("entity exceeds {MAX_ENTITY_SEGMENTS} segments")));
    }
    for segment in &built.segments {
        in_range(*segment, line)?;
    }
    account(total, count)?;
    if built.closed {
        let contour = Contour::try_new(built.segments, true, tolerance).map_err(|err| geom(line, err))?;
        closed.push(ImportedContour::new(built.layer, contour));
    } else {
        open.push(built.layer, built.segments);
    }
    Ok(())
}

fn skip_polyline(reader: &mut Reader<'_>) -> Result<()> {
    take_body(reader, false)?;
    loop {
        let Some(group) = reader.next_group()? else {
            return Ok(());
        };
        if group.code != 0 {
            return Err(invalid_at(group.line, "expected an entity type"));
        }
        match group.value.as_str() {
            "VERTEX" => {
                take_body(reader, false)?;
            }
            "SEQEND" => {
                take_body(reader, false)?;
                return Ok(());
            }
            _ => {
                reader.push(group)?;
                return Ok(());
            }
        }
    }
}

fn take_body(reader: &mut Reader<'_>, keep: bool) -> Result<Vec<Group>> {
    let mut body = Vec::new();
    loop {
        let Some(group) = reader.next_group()? else {
            return Ok(body);
        };
        if group.code == 0 {
            reader.push(group)?;
            return Ok(body);
        }
        if keep {
            if body.len() >= MAX_BODY {
                return Err(invalid_at(group.line, format!("entity exceeds {MAX_ENTITY_SEGMENTS} segments")));
            }
            body.push(group);
        }
    }
}

struct Built {
    layer: String,
    segments: Vec<Segment>,
    closed: bool,
}

fn build_entity(kind: &str, body: &[Group], scale: f64, tolerance: &Tolerance, line: usize) -> Result<Built> {
    let layer = layer_of(body);
    let mirror = extrusion_mirror(body)?;
    let (segments, closed) = match kind {
        "LINE" => (vec![Segment::Line(line_of(body, scale, mirror, line)?)], false),
        "ARC" => arc_of(body, scale, mirror, line)?,
        "CIRCLE" => (circle_of(body, scale, mirror, line)?, true),
        "LWPOLYLINE" => return lwpolyline(body, scale, mirror, tolerance, line, layer),
        _ => return Err(invalid(format!("internal: {kind}"))),
    };
    Ok(Built { layer, segments, closed })
}

fn layer_of(body: &[Group]) -> String {
    let mut layer = String::from("0");
    for group in body {
        if group.code == 8 {
            layer.clone_from(&group.value);
        }
    }
    layer
}

fn extrusion_mirror(body: &[Group]) -> Result<bool> {
    let mut x = 0.0;
    let mut y = 0.0;
    let mut z = 1.0;
    let mut seen = false;
    let mut line = 0usize;
    for group in body {
        let slot = match group.code {
            210 => &mut x,
            220 => &mut y,
            230 => &mut z,
            _ => continue,
        };
        *slot = parse_f64(&group.value, group.line)?;
        seen = true;
        line = group.line;
    }
    if !seen {
        return Ok(false);
    }
    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
        return Err(invalid_at(line, "extrusion is not finite"));
    }
    let len = (x * x + y * y + z * z).sqrt();
    let lateral = (x * x + y * y).sqrt();
    if !len.is_finite() || len <= 1e-12 || lateral > len * 1e-8 || z.abs() <= len * 1e-8 {
        return Err(unsupported(format!("line {line}: extrusion not parallel to Z")));
    }
    Ok(z < 0.0)
}

fn line_of(body: &[Group], scale: f64, mirror: bool, line: usize) -> Result<campanion_geom::Line> {
    let start = mm_point(require(body, 10, line)?, require(body, 20, line)?, scale, mirror, line)?;
    let end = mm_point(require(body, 11, line)?, require(body, 21, line)?, scale, mirror, line)?;
    campanion_geom::Line::new(start, end).map_err(|err| geom(line, err))
}

fn arc_of(body: &[Group], scale: f64, mirror: bool, line: usize) -> Result<(Vec<Segment>, bool)> {
    let center = mm_point(require(body, 10, line)?, require(body, 20, line)?, scale, mirror, line)?;
    let radius = mm_len(require(body, 40, line)?, scale, line)?;
    let (mut start, mut sweep, closed) = sweep_radians(require(body, 50, line)?, require(body, 51, line)?, line)?;
    if mirror {
        start = PI - start;
        sweep = -sweep;
    }
    let arc = Arc::new(center, radius, start, sweep).map_err(|err| geom(line, err))?;
    Ok((vec![Segment::Arc(arc)], closed))
}

fn circle_of(body: &[Group], scale: f64, mirror: bool, line: usize) -> Result<Vec<Segment>> {
    let center = mm_point(require(body, 10, line)?, require(body, 20, line)?, scale, mirror, line)?;
    let radius = mm_len(require(body, 40, line)?, scale, line)?;
    let sweep = if mirror { -TAU } else { TAU };
    let arc = Arc::new(center, radius, 0.0, sweep).map_err(|err| geom(line, err))?;
    Ok(vec![Segment::Arc(arc)])
}

fn sweep_radians(start_deg: f64, end_deg: f64, line: usize) -> Result<(f64, f64, bool)> {
    if !start_deg.is_finite() || !end_deg.is_finite() {
        return Err(invalid_at(line, "arc angle is not finite"));
    }
    let mut sweep_deg = end_deg - start_deg;
    if sweep_deg < 0.0 {
        sweep_deg += 360.0;
    }
    if sweep_deg > 360.0 + 1e-6 {
        return Err(invalid_at(line, "arc sweep exceeds one full turn"));
    }
    if sweep_deg > 360.0 {
        sweep_deg = 360.0;
    }
    if sweep_deg <= 1e-12 {
        return Err(invalid_at(line, "arc sweep is zero"));
    }
    let closed = sweep_deg >= 360.0 - 1e-9;
    let sweep_deg = if closed { 360.0 } else { sweep_deg };
    Ok((start_deg.to_radians(), sweep_deg.to_radians(), closed))
}

struct Draft {
    x: Option<f64>,
    y: Option<f64>,
    bulge: f64,
    line: usize,
}

fn lwpolyline(body: &[Group], scale: f64, mirror: bool, tolerance: &Tolerance, line: usize, layer: String) -> Result<Built> {
    let mut closed = false;
    let mut current: Option<Draft> = None;
    let mut drafts = Vec::new();
    for group in body {
        match group.code {
            8 | 210 | 220 | 230 => {}
            70 => closed = parse_i32(&group.value, group.line)? & 1 != 0,
            90 => {
                let count =
                    usize::try_from(parse_i32(&group.value, group.line)?).map_err(|_| invalid_at(group.line, "polyline vertex count is negative"))?;
                if count > MAX_ENTITY_SEGMENTS + 1 {
                    return Err(invalid_at(group.line, format!("polyline exceeds {MAX_ENTITY_SEGMENTS} segments")));
                }
            }
            10 => {
                finish_draft(&mut drafts, current.take())?;
                current = Some(Draft { x: Some(parse_f64(&group.value, group.line)?), y: None, bulge: 0.0, line: group.line });
            }
            20 => {
                let Some(draft) = current.as_mut() else {
                    return Err(invalid_at(group.line, "polyline Y before X"));
                };
                draft.y = Some(parse_f64(&group.value, group.line)?);
            }
            42 => {
                let Some(draft) = current.as_mut() else {
                    return Err(invalid_at(group.line, "polyline bulge before a vertex"));
                };
                draft.bulge = parse_f64(&group.value, group.line)?;
            }
            _ => {}
        }
    }
    finish_draft(&mut drafts, current.take())?;
    let mut vertices = Vec::with_capacity(drafts.len());
    for draft in drafts {
        let (Some(x), Some(y)) = (draft.x, draft.y) else {
            return Err(invalid_at(draft.line, "polyline vertex is missing a coordinate"));
        };
        let bulge = if mirror { -draft.bulge } else { draft.bulge };
        vertices.push(Vertex::new(mm_point(x, y, scale, mirror, draft.line)?, bulge).map_err(|err| geom(draft.line, err))?);
    }
    if closed {
        drop_repeated_close(&mut vertices, tolerance);
    }
    if vertices.len() < 2 {
        return Err(invalid_at(line, "polyline needs at least two vertices"));
    }
    let count = if closed { vertices.len() } else { vertices.len() - 1 };
    if count > MAX_ENTITY_SEGMENTS {
        return Err(invalid_at(line, format!("polyline exceeds {MAX_ENTITY_SEGMENTS} segments")));
    }
    let contour = Polyline::new(vertices, closed).map_err(|err| geom(line, err))?.to_contour(tolerance).map_err(|err| geom(line, err))?;
    Ok(Built { layer, segments: contour.segments().to_vec(), closed: contour.is_closed() })
}

fn finish_draft(drafts: &mut Vec<Draft>, draft: Option<Draft>) -> Result<()> {
    let Some(draft) = draft else {
        return Ok(());
    };
    if drafts.len() > MAX_ENTITY_SEGMENTS {
        return Err(invalid_at(draft.line, format!("polyline exceeds {MAX_ENTITY_SEGMENTS} segments")));
    }
    drafts.push(draft);
    Ok(())
}

fn drop_repeated_close(vertices: &mut Vec<Vertex>, tolerance: &Tolerance) {
    if vertices.len() < 3 {
        return;
    }
    let (Some(first), Some(last)) = (vertices.first().copied(), vertices.last().copied()) else {
        return;
    };
    if last.bulge().abs() <= 1e-15 && last.point().almost_eq(first.point(), tolerance.linear()) {
        vertices.pop();
    }
}

fn require(body: &[Group], code: i32, line: usize) -> Result<f64> {
    for group in body.iter().rev() {
        if group.code == code {
            return parse_f64(&group.value, group.line);
        }
    }
    Err(invalid_at(line, format!("missing group {code}")))
}

fn mm_point(x: f64, y: f64, scale: f64, mirror: bool, line: usize) -> Result<Point> {
    let mut x = x * scale;
    let y = y * scale;
    if mirror {
        x = -x;
    }
    check_mm(x, line)?;
    check_mm(y, line)?;
    Point::new(x, y).map_err(|err| geom(line, err))
}

fn mm_len(length: f64, scale: f64, line: usize) -> Result<f64> {
    let length = length * scale;
    if !length.is_finite() || length <= 0.0 {
        return Err(invalid_at(line, "radius must be finite and positive"));
    }
    if length > MAX_COORD_MM {
        return Err(invalid_at(line, format!("radius is outside ±{MAX_COORD_MM} mm")));
    }
    Ok(length)
}

fn in_range(segment: Segment, line: usize) -> Result<()> {
    let bounds = segment.bounding_box();
    check_mm(bounds.min().x(), line)?;
    check_mm(bounds.min().y(), line)?;
    check_mm(bounds.max().x(), line)?;
    check_mm(bounds.max().y(), line)?;
    Ok(())
}

fn check_mm(value: f64, line: usize) -> Result<()> {
    if value.is_finite() && value.abs() <= MAX_COORD_MM {
        Ok(())
    } else {
        Err(invalid_at(line, format!("coordinate is outside ±{MAX_COORD_MM} mm")))
    }
}

fn parse_f64(value: &str, line: usize) -> Result<f64> {
    let number = value.parse::<f64>().map_err(|_| invalid_at(line, format!("'{value}' is not a number")))?;
    if number.is_finite() { Ok(number) } else { Err(invalid_at(line, "number is not finite")) }
}

fn parse_i32(value: &str, line: usize) -> Result<i32> {
    if let Ok(number) = value.parse::<i32>() {
        return Ok(number);
    }
    let number = parse_f64(value, line)?;
    if number.fract() == 0.0 && (i32::MIN as f64..=i32::MAX as f64).contains(&number) {
        Ok(number as i32)
    } else {
        Err(invalid_at(line, format!("'{value}' is not an integer")))
    }
}

fn geom(line: usize, err: campanion_geom::GeomError) -> ImportError {
    match err {
        campanion_geom::GeomError::NotSupported(message) => unsupported(format!("line {line}: {message}")),
        campanion_geom::GeomError::InvalidInput(message) => invalid_at(line, message),
    }
}

#[cfg(test)]
mod tests {
    use campanion_geom::{Segment, Tolerance};

    use crate::{ImportOptions, import_dxf};

    fn drawing(body: &str) -> crate::Drawing {
        let text = format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{body}0\nENDSEC\n0\nEOF\n");
        import_dxf(&text, &ImportOptions::new(Tolerance::default())).expect("import")
    }

    #[test]
    fn line_circle_arc_units_and_order() {
        let text = "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n0\n20\n0\n11\n1\n21\n0\n0\nCIRCLE\n8\nHoles\n10\n0\n20\n0\n40\n1\n0\nENDSEC\n0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n0\nEOF\n";
        let drawing = import_dxf(text, &ImportOptions::new(Tolerance::default())).unwrap();
        assert_eq!(drawing.contours().len(), 2);
        assert_eq!(drawing.contours()[0].layer(), "Holes");
        assert!(drawing.contours()[0].contour().is_closed());
        let area = drawing.contours()[0].contour().signed_area().unwrap();
        assert!((area - std::f64::consts::PI * 25.4 * 25.4).abs() < 1e-6, "{area}");
        assert_eq!(drawing.contours()[1].layer(), "0");
        assert!(!drawing.contours()[1].contour().is_closed());
        assert!((drawing.contours()[1].contour().length().unwrap() - 25.4).abs() < 1e-9);
    }

    #[test]
    fn mirrored_arc_reverses_sense() {
        let body = "0\nARC\n10\n0\n20\n0\n40\n1\n50\n0\n51\n90\n210\n0\n220\n0\n230\n-1\n";
        let drawing = drawing(body);
        let Segment::Arc(arc) = drawing.contours()[0].contour().segments()[0] else { panic!("arc") };
        assert!(arc.center().almost_eq(campanion_geom::Point::new(0.0, 0.0).unwrap(), 1e-9));
        assert!(arc.sweep() < 0.0);
        assert!(arc.start_point().almost_eq(campanion_geom::Point::new(-1.0, 0.0).unwrap(), 1e-9));
        assert!(arc.end_point().almost_eq(campanion_geom::Point::new(0.0, 1.0).unwrap(), 1e-9));
    }

    #[test]
    fn closed_bulge_polyline_is_a_circle_and_old_polyline_counts_once() {
        let body = "\
0\nLWPOLYLINE\n8\nPlate\n90\n2\n70\n1\n10\n0\n20\n0\n42\n1\n10\n2\n20\n0\n42\n1\n\
0\nPOLYLINE\n8\nPlate\n70\n1\n0\nVERTEX\n10\n0\n20\n0\n0\nVERTEX\n10\n1\n20\n0\n0\nSEQEND\n\
0\nSPLINE\n8\nPlate\n0\nELLIPSE\n8\nPlate\n";
        let drawing = drawing(body);
        assert_eq!(drawing.skipped_entities(), 3);
        assert_eq!(skipped_report(&drawing), "ELLIPSE x1, POLYLINE x1, SPLINE x1");
        assert_eq!(drawing.contours().len(), 1);
        assert_eq!(drawing.contours()[0].layer(), "Plate");
        let area = drawing.contours()[0].contour().signed_area().unwrap();
        assert!((area - std::f64::consts::PI).abs() < 1e-8, "{area}");
    }

    #[test]
    fn open_segments_chain_per_layer_and_bridge_a_gap() {
        let body = "\
0\nLINE\n8\nA\n10\n0\n20\n0\n11\n1\n21\n0\n\
0\nLINE\n8\nB\n10\n0\n20\n0\n11\n0\n21\n1\n\
0\nLINE\n8\nA\n10\n1.4\n20\n0\n11\n2.4\n21\n0\n";
        let options = ImportOptions::with_gap(Tolerance::default(), 0.5).unwrap();
        let text = format!("0\nSECTION\n2\nENTITIES\n{body}0\nENDSEC\n0\nEOF\n");
        let drawing = import_dxf(&text, &options).unwrap();
        assert_eq!(drawing.contours().len(), 2);
        let layer_a = drawing.contours().iter().find(|item| item.layer() == "A").unwrap();
        assert!(!layer_a.contour().is_closed());
        assert_eq!(layer_a.contour().segments().len(), 3);
        assert!((layer_a.contour().length().unwrap() - 2.4).abs() < 1e-9);
        assert!(drawing.contours().iter().any(|item| item.layer() == "B" && item.contour().segments().len() == 1));
    }

    #[test]
    fn rejects_bad_files() {
        let options = ImportOptions::new(Tolerance::default());
        assert!(import_dxf("0\nSECTION\n2\nHEADER\n0\nENDSEC\n0\nEOF\n", &options).is_err());
        assert!(import_dxf("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n21\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nENDSEC\n", &options).is_err());
        let tilted = drawing_err("0\nLINE\n10\n0\n20\n0\n11\n1\n21\n0\n210\n1\n220\n0\n230\n0\n");
        assert!(tilted.to_string().contains("extrusion"));
        assert!(drawing_err("0\nLINE\n10\n1000000000\n20\n0\n11\n0\n21\n0\n").to_string().contains("coordinate"));
    }

    #[test]
    fn sample_fixture_reports_text() {
        let text = include_str!("../tests/fixtures/sample.dxf");
        let drawing = import_dxf(text, &ImportOptions::new(Tolerance::default())).unwrap();
        assert_eq!(drawing.skipped_entities(), 1);
        assert_eq!(skipped_report(&drawing), "TEXT x1");
    }

    fn skipped_report(drawing: &crate::Drawing) -> String {
        drawing.skipped_types().iter().map(|(name, count)| format!("{name} x{count}")).collect::<Vec<_>>().join(", ")
    }

    fn drawing_err(body: &str) -> crate::ImportError {
        let text = format!("0\nSECTION\n2\nENTITIES\n{body}0\nENDSEC\n0\nEOF\n");
        import_dxf(&text, &ImportOptions::new(Tolerance::default())).unwrap_err()
    }
}
