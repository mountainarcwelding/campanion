//! SVG import into millimetre contours.
//!
//! The root `viewBox` and `width`/`height` become one affine, then Y is flipped into the same
//! plane as DXF import. Element `transform` attributes compose inside that map. Curves are fitted
//! with lines and arcs within tolerance. A circle whose full affine is a similarity stays one exact arc.

use std::collections::{BTreeMap, HashMap};

use campanion_geom::{Arc, Line, Point, Segment, Tolerance};
use kurbo::Affine;

use crate::chain::chain_paths;
use crate::{Drawing, ImportOptions, ImportedContour, MAX_COORD_MM, MAX_DXF_BYTES, MAX_TOTAL_SEGMENTS, Result, invalid};

const PX_MM: f64 = 25.4 / 96.0;
const TAU: f64 = std::f64::consts::TAU;
const INKSCAPE: &str = "http://www.inkscape.org/namespaces/inkscape";
/// `|det|` and column dot-product may drift by this fraction of `scale²`.
const SIMILARITY_EPS: f64 = 1e-9;

/// Read one SVG document into millimetre contours.
pub fn import_svg(text: &str, options: &ImportOptions) -> Result<Drawing> {
    if text.len() > MAX_DXF_BYTES {
        return Err(invalid(format!("SVG exceeds {MAX_DXF_BYTES} bytes")));
    }
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let doc = roxmltree::Document::parse(text).map_err(|err| invalid(format!("invalid XML: {err}")))?;
    let root = doc.root().children().find(|node| node.is_element()).ok_or_else(|| invalid("SVG document has no root element"))?;
    if root.tag_name().name() != "svg" {
        return Err(invalid(format!("root element is '{}', expected svg", root.tag_name().name())));
    }
    let mut import = Import::new(options, page_affine(root)?);
    import.walk(root, Affine::IDENTITY)?;
    import.finish()
}

struct Layers {
    order: Vec<String>,
    paths: HashMap<String, Vec<Vec<Segment>>>,
}

impl Layers {
    fn push(&mut self, layer: &str, segments: Vec<Segment>) {
        if let Some(paths) = self.paths.get_mut(layer) {
            paths.push(segments);
            return;
        }
        self.order.push(layer.to_string());
        self.paths.insert(layer.to_string(), vec![segments]);
    }
}

struct Import {
    tolerance: Tolerance,
    gap: f64,
    flatten_tol: f64,
    root_affine: Affine,
    layers: Layers,
    skipped: usize,
    skipped_types: BTreeMap<String, usize>,
    total: usize,
}

impl Import {
    fn new(options: &ImportOptions, root_affine: Affine) -> Self {
        let tolerance = options.tolerance();
        Self {
            tolerance,
            gap: options.gap(),
            flatten_tol: tolerance.linear(),
            root_affine,
            layers: Layers { order: Vec::new(), paths: HashMap::new() },
            skipped: 0,
            skipped_types: BTreeMap::new(),
            total: 0,
        }
    }

    fn walk(&mut self, node: roxmltree::Node, parent: Affine) -> Result<()> {
        if !node.is_element() {
            return Ok(());
        }
        match node.tag_name().name() {
            "svg" | "g" | "a" | "switch" => {
                let local = compose(parent, transform_of(node)?)?;
                for child in node.children() {
                    self.walk(child, local)?;
                }
                Ok(())
            }
            "path" | "rect" | "circle" | "ellipse" | "line" | "polyline" | "polygon" => {
                let local = compose(parent, transform_of(node)?)?;
                self.emit_shape(node, local)
            }
            tag => self.note_skipped(tag),
        }
    }

    fn emit_shape(&mut self, node: roxmltree::Node, local: Affine) -> Result<()> {
        let full = compose(self.root_affine, local)?;
        let layer = layer_name(node);
        match node.tag_name().name() {
            "path" => self.emit_path(node, &layer, full),
            "rect" => self.emit_rect(node, &layer, full),
            "circle" => self.emit_circle(node, &layer, full),
            "ellipse" => self.emit_ellipse(node, &layer, full),
            "line" => self.emit_line(node, &layer, full),
            "polyline" => self.emit_poly(node, &layer, full, false),
            "polygon" => self.emit_poly(node, &layer, full, true),
            _ => Ok(()),
        }
    }

    fn emit_path(&mut self, node: roxmltree::Node, layer: &str, full: Affine) -> Result<()> {
        let Some(d) = node.attribute("d") else {
            return Ok(());
        };
        if d.trim().is_empty() {
            return Ok(());
        }
        self.commit_bez(layer, path_to_bez(d)?, full)
    }

    fn emit_rect(&mut self, node: roxmltree::Node, layer: &str, full: Affine) -> Result<()> {
        let x = attr_user(node, "x")?;
        let y = attr_user(node, "y")?;
        let width = attr_user(node, "width")?;
        let height = attr_user(node, "height")?;
        if width <= 0.0 || height <= 0.0 {
            return Ok(());
        }
        let mut path = kurbo::BezPath::new();
        path.move_to((x, y));
        path.line_to((x + width, y));
        path.line_to((x + width, y + height));
        path.line_to((x, y + height));
        path.close_path();
        self.commit_bez(layer, path, full)
    }

    fn emit_circle(&mut self, node: roxmltree::Node, layer: &str, full: Affine) -> Result<()> {
        let cx = attr_user(node, "cx")?;
        let cy = attr_user(node, "cy")?;
        let radius = attr_user(node, "r")?;
        if radius <= 0.0 {
            return Ok(());
        }
        if let Some(scale) = similarity_scale(full) {
            let radius = radius * scale;
            if radius <= 0.0 || !radius.is_finite() {
                return Ok(());
            }
            let (x, y) = transform_xy(full, cx, cy);
            let center = Point::new(x, y).map_err(crate::ImportError::from_geom)?;
            let arc = Arc::new(center, radius, 0.0, TAU).map_err(crate::ImportError::from_geom)?;
            return self.commit_segments(layer, vec![Segment::Arc(arc)]);
        }
        self.commit_bez(layer, ellipse_bez(cx, cy, radius, radius), full)
    }

    fn emit_ellipse(&mut self, node: roxmltree::Node, layer: &str, full: Affine) -> Result<()> {
        let cx = attr_user(node, "cx")?;
        let cy = attr_user(node, "cy")?;
        let rx = attr_user(node, "rx")?;
        let ry = attr_user(node, "ry")?;
        if rx <= 0.0 || ry <= 0.0 {
            return Ok(());
        }
        self.commit_bez(layer, ellipse_bez(cx, cy, rx, ry), full)
    }

    fn emit_line(&mut self, node: roxmltree::Node, layer: &str, full: Affine) -> Result<()> {
        let x1 = attr_user(node, "x1")?;
        let y1 = attr_user(node, "y1")?;
        let x2 = attr_user(node, "x2")?;
        let y2 = attr_user(node, "y2")?;
        let mut path = kurbo::BezPath::new();
        path.move_to((x1, y1));
        path.line_to((x2, y2));
        self.commit_bez(layer, path, full)
    }

    fn emit_poly(&mut self, node: roxmltree::Node, layer: &str, full: Affine, close: bool) -> Result<()> {
        let Some(points) = node.attribute("points") else {
            return Ok(());
        };
        let nums = parse_points(points)?;
        if nums.len() < 2 {
            return Ok(());
        }
        let mut path = kurbo::BezPath::new();
        let mut pairs = nums.chunks_exact(2);
        let Some(first) = pairs.next() else {
            return Ok(());
        };
        let (Some(&x), Some(&y)) = (first.first(), first.get(1)) else {
            return Ok(());
        };
        path.move_to((x, y));
        for pair in pairs {
            let (Some(&x), Some(&y)) = (pair.first(), pair.get(1)) else {
                continue;
            };
            path.line_to((x, y));
        }
        if close {
            path.close_path();
        }
        self.commit_bez(layer, path, full)
    }

    fn commit_bez(&mut self, layer: &str, mut path: kurbo::BezPath, full: Affine) -> Result<()> {
        if path.is_empty() {
            return Ok(());
        }
        path.apply_affine(full);
        let subpaths = fit_path(&path, self.flatten_tol, self.budget())?;
        self.add_subpaths(layer, subpaths)
    }

    fn commit_segments(&mut self, layer: &str, segments: Vec<Segment>) -> Result<()> {
        if segments.is_empty() {
            return Ok(());
        }
        self.add_subpaths(layer, vec![segments])
    }

    fn add_subpaths(&mut self, layer: &str, subpaths: Vec<Vec<Segment>>) -> Result<()> {
        let mut count = 0usize;
        for path in &subpaths {
            for segment in path {
                in_range(*segment)?;
            }
            count = count.checked_add(path.len()).ok_or_else(too_many)?;
        }
        if count == 0 {
            return Ok(());
        }
        account(&mut self.total, count)?;
        for path in subpaths {
            if !path.is_empty() {
                self.layers.push(layer, path);
            }
        }
        Ok(())
    }

    fn budget(&self) -> usize {
        MAX_TOTAL_SEGMENTS.saturating_sub(self.total)
    }

    fn note_skipped(&mut self, tag: &str) -> Result<()> {
        self.skipped = self.skipped.checked_add(1).ok_or_else(|| invalid("too many skipped elements"))?;
        if let Some(count) = self.skipped_types.get_mut(tag) {
            *count = count.checked_add(1).ok_or_else(|| invalid("too many skipped elements"))?;
        } else {
            self.skipped_types.insert(tag.to_string(), 1);
        }
        Ok(())
    }

    fn finish(self) -> Result<Drawing> {
        let Import { tolerance, gap, layers, skipped, skipped_types, total, .. } = self;
        let Layers { order, mut paths } = layers;
        let mut contours = Vec::new();
        let mut total = total;
        for layer in order {
            let Some(layer_paths) = paths.remove(&layer) else {
                return Err(invalid("internal: missing layer"));
            };
            let before = segment_count(&layer_paths)?;
            let chained = chain_paths(layer_paths, gap, &tolerance)?;
            let mut after = 0usize;
            for contour in &chained {
                after = after.checked_add(contour.segments().len()).ok_or_else(too_many)?;
            }
            if after > before {
                account(&mut total, after - before)?;
            }
            for contour in chained {
                contours.push(ImportedContour::new(layer.clone(), contour));
            }
        }
        Ok(Drawing::new(contours, skipped, skipped_types.into_iter().collect()))
    }
}

fn too_many() -> crate::ImportError {
    invalid(format!("more than {MAX_TOTAL_SEGMENTS} segments"))
}

fn account(total: &mut usize, count: usize) -> Result<()> {
    let next = total.checked_add(count).ok_or_else(too_many)?;
    if next > MAX_TOTAL_SEGMENTS {
        return Err(too_many());
    }
    *total = next;
    Ok(())
}

fn segment_count(paths: &[Vec<Segment>]) -> Result<usize> {
    let mut count = 0usize;
    for path in paths {
        count = count.checked_add(path.len()).ok_or_else(too_many)?;
    }
    Ok(count)
}

fn in_range(segment: Segment) -> Result<()> {
    let bounds = segment.bounding_box();
    check_mm(bounds.min().x())?;
    check_mm(bounds.min().y())?;
    check_mm(bounds.max().x())?;
    check_mm(bounds.max().y())?;
    Ok(())
}

fn check_mm(value: f64) -> Result<()> {
    if value.is_finite() && value.abs() <= MAX_COORD_MM { Ok(()) } else { Err(invalid(format!("coordinate is outside ±{MAX_COORD_MM} mm"))) }
}

/// Page map: translate the viewBox origin, scale into millimetres, then flip Y.
///
/// `x' = sx * (x - origin_x)`, `y' = height_mm - sy * (y - origin_y)`.
fn page_affine(root: roxmltree::Node) -> Result<Affine> {
    let width = optional_mm(root, "width")?;
    let height = optional_mm(root, "height")?;
    let view_box = parse_view_box(root)?;
    let (sx, sy, origin_x, origin_y, height_mm) = match (width, height, view_box) {
        (Some(width_mm), Some(height_mm), Some(view_box)) => (width_mm / view_box.w, height_mm / view_box.h, view_box.x, view_box.y, height_mm),
        (_, _, Some(view_box)) => {
            let scale = PX_MM;
            (scale, scale, view_box.x, view_box.y, view_box.h * scale)
        }
        (_, height_mm, None) => (PX_MM, PX_MM, 0.0, 0.0, height_mm.unwrap_or(0.0)),
    };
    if ![sx, sy, origin_x, origin_y, height_mm].into_iter().all(f64::is_finite) {
        return Err(invalid("svg scale is not finite"));
    }
    let affine = Affine::new([sx, 0.0, 0.0, -sy, -sx * origin_x, height_mm + sy * origin_y]);
    finite_affine(affine)
}

fn parse_view_box(root: roxmltree::Node) -> Result<Option<svgtypes::ViewBox>> {
    let Some(text) = root.attribute("viewBox") else {
        return Ok(None);
    };
    let view_box = text.parse::<svgtypes::ViewBox>().map_err(|_| invalid(format!("invalid viewBox '{text}'")))?;
    if ![view_box.x, view_box.y, view_box.w, view_box.h].into_iter().all(f64::is_finite) {
        return Err(invalid("viewBox is not finite"));
    }
    if view_box.w <= 0.0 || view_box.h <= 0.0 {
        return Err(invalid("viewBox width and height must be positive"));
    }
    Ok(Some(view_box))
}

fn optional_mm(node: roxmltree::Node, name: &str) -> Result<Option<f64>> {
    match node.attribute(name) {
        Some(text) => Ok(Some(length_mm(text)?)),
        None => Ok(None),
    }
}

fn length_mm(text: &str) -> Result<f64> {
    let length = parse_length(text)?;
    let mm = length.number * mm_per_unit(length.unit, text)?;
    if mm.is_finite() { Ok(mm) } else { Err(invalid(format!("length '{text}' is not finite"))) }
}

fn mm_per_unit(unit: svgtypes::LengthUnit, text: &str) -> Result<f64> {
    Ok(match unit {
        svgtypes::LengthUnit::None | svgtypes::LengthUnit::Px => PX_MM,
        svgtypes::LengthUnit::In => 25.4,
        svgtypes::LengthUnit::Cm => 10.0,
        svgtypes::LengthUnit::Mm => 1.0,
        svgtypes::LengthUnit::Pt => 25.4 / 72.0,
        svgtypes::LengthUnit::Pc => 25.4 / 6.0,
        svgtypes::LengthUnit::Em | svgtypes::LengthUnit::Ex | svgtypes::LengthUnit::Percent => {
            return Err(invalid(format!("unsupported length unit in '{text}'")));
        }
    })
}

/// Shape coordinates are user units. `1px` is one user unit. Absolute units convert through CSS pixels.
fn user_length(text: &str) -> Result<f64> {
    let length = parse_length(text)?;
    let value = match length.unit {
        svgtypes::LengthUnit::None | svgtypes::LengthUnit::Px => length.number,
        svgtypes::LengthUnit::Em | svgtypes::LengthUnit::Ex | svgtypes::LengthUnit::Percent => {
            return Err(invalid(format!("unsupported length unit in '{text}'")));
        }
        unit => length.number * mm_per_unit(unit, text)? / PX_MM,
    };
    if value.is_finite() { Ok(value) } else { Err(invalid(format!("length '{text}' is not finite"))) }
}

fn parse_length(text: &str) -> Result<svgtypes::Length> {
    text.parse::<svgtypes::Length>().map_err(|_| invalid(format!("invalid length '{text}'")))
}

fn attr_user(node: roxmltree::Node, name: &str) -> Result<f64> {
    match node.attribute(name) {
        Some(text) => user_length(text),
        None => Ok(0.0),
    }
}

fn transform_of(node: roxmltree::Node) -> Result<Affine> {
    match node.attribute("transform") {
        Some(text) => parse_transform(text),
        None => Ok(Affine::IDENTITY),
    }
}

fn parse_transform(text: &str) -> Result<Affine> {
    let text = text.trim();
    if text.is_empty() {
        return Ok(Affine::IDENTITY);
    }
    let parsed = text.parse::<svgtypes::Transform>().map_err(|err| invalid(format!("invalid transform: {err}")))?;
    finite_affine(Affine::new([parsed.a, parsed.b, parsed.c, parsed.d, parsed.e, parsed.f]))
}

fn compose(parent: Affine, child: Affine) -> Result<Affine> {
    finite_affine(parent * child)
}

fn finite_affine(affine: Affine) -> Result<Affine> {
    if affine.as_coeffs().iter().all(|value| value.is_finite()) { Ok(affine) } else { Err(invalid("transform is not finite")) }
}

fn transform_xy(affine: Affine, x: f64, y: f64) -> (f64, f64) {
    let point = affine * kurbo::Point::new(x, y);
    (point.x, point.y)
}

/// Uniform scale, rotation, and reflection. `scale²` is the squared length of the first column.
fn similarity_scale(affine: Affine) -> Option<f64> {
    let [a, b, c, d, _, _] = affine.as_coeffs();
    let scale2 = a * a + b * b;
    if !scale2.is_finite() || scale2 <= 0.0 {
        return None;
    }
    let det = a * d - b * c;
    let skew = a * c + b * d;
    let limit = SIMILARITY_EPS * scale2;
    if (det.abs() - scale2).abs() > limit || skew.abs() > limit {
        return None;
    }
    let scale = scale2.sqrt();
    if scale.is_finite() && scale > 0.0 { Some(scale) } else { None }
}

fn layer_name(node: roxmltree::Node) -> String {
    let mut nearest_id: Option<String> = None;
    for ancestor in node.ancestors() {
        if !ancestor.is_element() || ancestor.tag_name().name() != "g" {
            continue;
        }
        if let Some(label) = ancestor.attribute((INKSCAPE, "label"))
            && !label.is_empty()
        {
            return label.to_string();
        }
        if nearest_id.is_none()
            && let Some(id) = ancestor.attribute("id")
            && !id.is_empty()
        {
            nearest_id = Some(id.to_string());
        }
    }
    nearest_id.unwrap_or_else(|| "0".to_string())
}

fn path_to_bez(d: &str) -> Result<kurbo::BezPath> {
    let mut path = kurbo::BezPath::new();
    for segment in svgtypes::SimplifyingPathParser::from(d) {
        let segment = segment.map_err(|err| invalid(format!("invalid path: {err}")))?;
        match segment {
            svgtypes::SimplePathSegment::MoveTo { x, y } => {
                let (x, y) = finite_xy(x, y)?;
                path.move_to((x, y));
            }
            svgtypes::SimplePathSegment::LineTo { x, y } => {
                let (x, y) = finite_xy(x, y)?;
                path.line_to((x, y));
            }
            svgtypes::SimplePathSegment::CurveTo { x1, y1, x2, y2, x, y } => {
                let (x1, y1) = finite_xy(x1, y1)?;
                let (x2, y2) = finite_xy(x2, y2)?;
                let (x, y) = finite_xy(x, y)?;
                path.curve_to((x1, y1), (x2, y2), (x, y));
            }
            svgtypes::SimplePathSegment::Quadratic { x1, y1, x, y } => {
                let (x1, y1) = finite_xy(x1, y1)?;
                let (x, y) = finite_xy(x, y)?;
                path.quad_to((x1, y1), (x, y));
            }
            svgtypes::SimplePathSegment::ClosePath => path.close_path(),
        }
    }
    Ok(path)
}

fn ellipse_bez(cx: f64, cy: f64, rx: f64, ry: f64) -> kurbo::BezPath {
    let kappa = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
    let kx = rx * kappa;
    let ky = ry * kappa;
    let mut path = kurbo::BezPath::new();
    path.move_to((cx + rx, cy));
    path.curve_to((cx + rx, cy + ky), (cx + kx, cy + ry), (cx, cy + ry));
    path.curve_to((cx - kx, cy + ry), (cx - rx, cy + ky), (cx - rx, cy));
    path.curve_to((cx - rx, cy - ky), (cx - kx, cy - ry), (cx, cy - ry));
    path.curve_to((cx + kx, cy - ry), (cx + rx, cy - ky), (cx + rx, cy));
    path.close_path();
    path
}

fn finite_xy(x: f64, y: f64) -> Result<(f64, f64)> {
    if x.is_finite() && y.is_finite() { Ok((x, y)) } else { Err(invalid("coordinate is not finite")) }
}

fn parse_points(text: &str) -> Result<Vec<f64>> {
    let mut nums = Vec::new();
    for token in text.split(|c: char| c.is_whitespace() || c == ',') {
        if token.is_empty() {
            continue;
        }
        let number = token.parse::<f64>().map_err(|_| invalid(format!("'{token}' is not a number")))?;
        if !number.is_finite() {
            return Err(invalid(format!("'{token}' is not finite")));
        }
        nums.push(number);
    }
    if nums.len() % 2 != 0 {
        return Err(invalid("points list has an odd number of coordinates"));
    }
    Ok(nums)
}

struct Pen {
    start: Option<Point>,
    last: Option<Point>,
    current: Vec<Segment>,
}

fn fit_path(path: &kurbo::BezPath, tolerance: f64, budget: usize) -> Result<Vec<Vec<Segment>>> {
    let mut subpaths = Vec::new();
    let mut pen = Pen { start: None, last: None, current: Vec::new() };
    let mut produced = 0usize;
    let opts = crate::arcfit::FitOptions::new(tolerance);
    for element in path.elements() {
        match *element {
            kurbo::PathEl::MoveTo(point) => move_pen(&mut pen, &mut subpaths, point)?,
            kurbo::PathEl::LineTo(point) => line_pen(&mut pen, point, &mut produced, budget)?,
            kurbo::PathEl::QuadTo(p1, p2) => fit_quad(&mut pen, p1, p2, &opts, &mut produced, budget)?,
            kurbo::PathEl::CurveTo(p1, p2, p3) => fit_cubic(&mut pen, p1, p2, p3, &opts, &mut produced, budget)?,
            kurbo::PathEl::ClosePath => close_pen(&mut pen, &mut subpaths, &mut produced, budget)?,
        }
    }
    if !pen.current.is_empty() {
        subpaths.push(pen.current);
    }
    Ok(subpaths)
}

fn fit_quad(pen: &mut Pen, p1: kurbo::Point, p2: kurbo::Point, opts: &crate::arcfit::FitOptions, produced: &mut usize, budget: usize) -> Result<()> {
    let Some(start) = pen.last else {
        return start_pen(pen, p2);
    };
    let segments = crate::arcfit::fit_quad((start.x(), start.y()), (p1.x, p1.y), (p2.x, p2.y), opts)?;
    push_fitted(pen, segments, produced, budget)?;
    pen.last = Some(geom_point(p2)?);
    Ok(())
}

fn fit_cubic(
    pen: &mut Pen,
    p1: kurbo::Point,
    p2: kurbo::Point,
    p3: kurbo::Point,
    opts: &crate::arcfit::FitOptions,
    produced: &mut usize,
    budget: usize,
) -> Result<()> {
    let Some(start) = pen.last else {
        return start_pen(pen, p3);
    };
    let segments = crate::arcfit::fit_cubic((start.x(), start.y()), (p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y), opts)?;
    push_fitted(pen, segments, produced, budget)?;
    pen.last = Some(geom_point(p3)?);
    Ok(())
}

/// No current point: adopt the curve end and emit nothing, as `line_pen` does.
fn start_pen(pen: &mut Pen, point: kurbo::Point) -> Result<()> {
    let point = geom_point(point)?;
    pen.start = Some(point);
    pen.last = Some(point);
    Ok(())
}

fn push_fitted(pen: &mut Pen, segments: Vec<Segment>, produced: &mut usize, budget: usize) -> Result<()> {
    for segment in segments {
        in_range(segment)?;
        *produced = produced.checked_add(1).ok_or_else(too_many)?;
        if *produced > budget {
            return Err(too_many());
        }
        pen.current.push(segment);
    }
    Ok(())
}

fn move_pen(pen: &mut Pen, subpaths: &mut Vec<Vec<Segment>>, point: kurbo::Point) -> Result<()> {
    if !pen.current.is_empty() {
        subpaths.push(std::mem::take(&mut pen.current));
    }
    let point = geom_point(point)?;
    pen.start = Some(point);
    pen.last = Some(point);
    Ok(())
}

fn close_pen(pen: &mut Pen, subpaths: &mut Vec<Vec<Segment>>, produced: &mut usize, budget: usize) -> Result<()> {
    if let (Some(start), Some(end)) = (pen.start, pen.last) {
        push_line(pen, end, start, produced, budget)?;
    }
    if !pen.current.is_empty() {
        subpaths.push(std::mem::take(&mut pen.current));
    }
    pen.last = pen.start;
    Ok(())
}

fn line_pen(pen: &mut Pen, point: kurbo::Point, produced: &mut usize, budget: usize) -> Result<()> {
    let point = geom_point(point)?;
    let Some(from) = pen.last else {
        pen.start = Some(point);
        pen.last = Some(point);
        return Ok(());
    };
    push_line(pen, from, point, produced, budget)
}

fn push_line(pen: &mut Pen, from: Point, to: Point, produced: &mut usize, budget: usize) -> Result<()> {
    pen.last = Some(to);
    if from.distance(to) == 0.0 {
        return Ok(());
    }
    let line = match Line::new(from, to) {
        Ok(line) => line,
        Err(err) => return Err(crate::ImportError::from_geom(err)),
    };
    if line.length() == 0.0 {
        return Ok(());
    }
    let segment = Segment::Line(line);
    in_range(segment)?;
    *produced = produced.checked_add(1).ok_or_else(too_many)?;
    if *produced > budget {
        return Err(too_many());
    }
    pen.current.push(segment);
    Ok(())
}

fn geom_point(point: kurbo::Point) -> Result<Point> {
    Point::new(point.x, point.y).map_err(crate::ImportError::from_geom)
}

#[cfg(test)]
mod tests {
    use campanion_geom::Tolerance;

    use crate::ImportOptions;

    #[test]
    fn rect_in_millimetre_view_box_flips_y() {
        let svg = r#"<svg width="100mm" height="50mm" viewBox="0 0 100 50"><rect x="10" y="10" width="20" height="10"/></svg>"#;
        let drawing = super::import_svg(svg, &ImportOptions::new(Tolerance::default())).expect("import");
        assert_eq!(drawing.contours().len(), 1);
        let bounds = drawing.contours()[0].contour().bounding_box().expect("bounds");
        let min = bounds.min();
        let max = bounds.max();
        assert!((min.x() - 10.0).abs() < 1e-6 && (max.x() - 30.0).abs() < 1e-6, "x {}..{}", min.x(), max.x());
        assert!((min.y() - 30.0).abs() < 1e-6 && (max.y() - 40.0).abs() < 1e-6, "y {}..{}", min.y(), max.y());
    }
}
