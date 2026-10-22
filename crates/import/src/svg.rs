//! SVG import into millimetre contours.
//!
//! The root `viewBox` and `width`/`height` become one affine, then Y is flipped into the same
//! plane as DXF import. Element `transform` attributes compose inside that map. Curves are fitted
//! with lines and arcs within tolerance. A circle, or a circular path arc under a similarity, stays one exact arc.

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
/// User-unit flatness for cubics that stand in for non-circular SVG arcs.
/// One tenth of a `1e-2` drawing tolerance.
const ARC_CUBIC_TOL: f64 = 1e-3;

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
        let items = parse_path(d)?;
        if items.is_empty() {
            return Ok(());
        }
        if let Some(scale) = similarity_scale(full) {
            self.commit_similarity(layer, &items, full, scale)
        } else {
            self.commit_bez(layer, path_from_items(&items), full)
        }
    }

    /// Similarity: circular arcs stay exact; other elements go through the fitter.
    fn commit_similarity(&mut self, layer: &str, items: &[PathItem], full: Affine, scale: f64) -> Result<()> {
        let subpaths = fit_items(items, full, scale, self.flatten_tol, self.budget())?;
        self.add_subpaths(layer, subpaths)
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

/// User-space path. `Arc` is circular after SVG F.6.6; ellipses are already cubics.
#[derive(Clone, Copy)]
enum PathItem {
    El(kurbo::PathEl),
    Arc { from: (f64, f64), to: (f64, f64), r: f64, large_arc: bool, sweep: bool },
}

fn parse_path(d: &str) -> Result<Vec<PathItem>> {
    let mut out = PathOut::default();
    for segment in svgtypes::PathParser::from(d) {
        let segment = segment.map_err(|err| invalid(format!("invalid path: {err}")))?;
        out.push_segment(segment)?;
    }
    Ok(out.items)
}

#[derive(Default)]
struct PathOut {
    items: Vec<PathItem>,
    cx: f64,
    cy: f64,
    sx: f64,
    sy: f64,
    ctrl_x: f64,
    ctrl_y: f64,
    has_point: bool,
    prev_cubic: bool,
    prev_quad: bool,
}

impl PathOut {
    fn push_segment(&mut self, segment: svgtypes::PathSegment) -> Result<()> {
        match segment {
            svgtypes::PathSegment::MoveTo { abs, x, y } => self.move_to(abs, x, y),
            svgtypes::PathSegment::LineTo { abs, x, y } => self.line_to(abs, x, y),
            svgtypes::PathSegment::HorizontalLineTo { abs, x } => self.horizontal(abs, x),
            svgtypes::PathSegment::VerticalLineTo { abs, y } => self.vertical(abs, y),
            svgtypes::PathSegment::CurveTo { abs, x1, y1, x2, y2, x, y } => self.curve_to(abs, (x1, y1), (x2, y2), (x, y)),
            svgtypes::PathSegment::SmoothCurveTo { abs, x2, y2, x, y } => self.smooth_curve(abs, x2, y2, x, y),
            svgtypes::PathSegment::Quadratic { abs, x1, y1, x, y } => self.quad_to(abs, (x1, y1), (x, y)),
            svgtypes::PathSegment::SmoothQuadratic { abs, x, y } => self.smooth_quad(abs, x, y),
            svgtypes::PathSegment::EllipticalArc { abs, rx, ry, x_axis_rotation, large_arc, sweep, x, y } => {
                self.elliptical_arc(abs, (rx, ry), x_axis_rotation, (large_arc, sweep), (x, y))
            }
            svgtypes::PathSegment::ClosePath { .. } => self.close_subpath(),
        }
    }

    fn move_to(&mut self, abs: bool, x: f64, y: f64) -> Result<()> {
        // A relative moveto at the start of the path is absolute (SVG 1.1).
        let (x, y) = if abs || !self.has_point { finite_xy(x, y)? } else { finite_xy(self.cx + x, self.cy + y)? };
        self.cx = x;
        self.cy = y;
        self.sx = x;
        self.sy = y;
        self.has_point = true;
        self.clear_smooth();
        self.items.push(PathItem::El(kurbo::PathEl::MoveTo(kurbo::Point::new(x, y))));
        Ok(())
    }

    fn line_to(&mut self, abs: bool, x: f64, y: f64) -> Result<()> {
        let (x, y) = self.resolve(abs, x, y)?;
        self.line_abs(x, y)
    }

    fn horizontal(&mut self, abs: bool, x: f64) -> Result<()> {
        if !self.has_point {
            return Err(invalid("invalid path: command before moveto"));
        }
        let x = if abs { x } else { self.cx + x };
        self.line_abs(x, self.cy)
    }

    fn vertical(&mut self, abs: bool, y: f64) -> Result<()> {
        if !self.has_point {
            return Err(invalid("invalid path: command before moveto"));
        }
        let y = if abs { y } else { self.cy + y };
        self.line_abs(self.cx, y)
    }

    fn line_abs(&mut self, x: f64, y: f64) -> Result<()> {
        let (x, y) = finite_xy(x, y)?;
        self.cx = x;
        self.cy = y;
        self.clear_smooth();
        self.items.push(PathItem::El(kurbo::PathEl::LineTo(kurbo::Point::new(x, y))));
        Ok(())
    }

    fn curve_to(&mut self, abs: bool, c1: (f64, f64), c2: (f64, f64), end: (f64, f64)) -> Result<()> {
        let (x1, y1) = self.resolve(abs, c1.0, c1.1)?;
        let (x2, y2) = self.resolve(abs, c2.0, c2.1)?;
        let (x, y) = self.resolve(abs, end.0, end.1)?;
        self.items.push(PathItem::El(kurbo::PathEl::CurveTo(kurbo::Point::new(x1, y1), kurbo::Point::new(x2, y2), kurbo::Point::new(x, y))));
        self.set_cubic(x2, y2);
        self.cx = x;
        self.cy = y;
        Ok(())
    }

    fn smooth_curve(&mut self, abs: bool, x2: f64, y2: f64, x: f64, y: f64) -> Result<()> {
        if !self.has_point {
            return Err(invalid("invalid path: command before moveto"));
        }
        let (x1, y1) = if self.prev_cubic { self.reflect_ctrl() } else { (self.cx, self.cy) };
        let (x1, y1) = finite_xy(x1, y1)?;
        let (x2, y2) = self.resolve(abs, x2, y2)?;
        let (x, y) = self.resolve(abs, x, y)?;
        self.items.push(PathItem::El(kurbo::PathEl::CurveTo(kurbo::Point::new(x1, y1), kurbo::Point::new(x2, y2), kurbo::Point::new(x, y))));
        self.set_cubic(x2, y2);
        self.cx = x;
        self.cy = y;
        Ok(())
    }

    fn quad_to(&mut self, abs: bool, ctrl: (f64, f64), end: (f64, f64)) -> Result<()> {
        let (x1, y1) = self.resolve(abs, ctrl.0, ctrl.1)?;
        let (x, y) = self.resolve(abs, end.0, end.1)?;
        self.items.push(PathItem::El(kurbo::PathEl::QuadTo(kurbo::Point::new(x1, y1), kurbo::Point::new(x, y))));
        self.set_quad(x1, y1);
        self.cx = x;
        self.cy = y;
        Ok(())
    }

    fn smooth_quad(&mut self, abs: bool, x: f64, y: f64) -> Result<()> {
        if !self.has_point {
            return Err(invalid("invalid path: command before moveto"));
        }
        let (x1, y1) = if self.prev_quad { self.reflect_ctrl() } else { (self.cx, self.cy) };
        let (x1, y1) = finite_xy(x1, y1)?;
        let (x, y) = self.resolve(abs, x, y)?;
        self.items.push(PathItem::El(kurbo::PathEl::QuadTo(kurbo::Point::new(x1, y1), kurbo::Point::new(x, y))));
        self.set_quad(x1, y1);
        self.cx = x;
        self.cy = y;
        Ok(())
    }

    fn close_subpath(&mut self) -> Result<()> {
        if !self.has_point {
            return Ok(());
        }
        self.items.push(PathItem::El(kurbo::PathEl::ClosePath));
        self.cx = self.sx;
        self.cy = self.sy;
        self.clear_smooth();
        Ok(())
    }

    fn elliptical_arc(&mut self, abs: bool, radii: (f64, f64), x_axis_rotation: f64, flags: (bool, bool), end: (f64, f64)) -> Result<()> {
        if !radii.0.is_finite() || !radii.1.is_finite() || !x_axis_rotation.is_finite() {
            return Err(invalid("coordinate is not finite"));
        }
        let (x, y) = self.resolve(abs, end.0, end.1)?;
        let from = (self.cx, self.cy);
        self.cx = x;
        self.cy = y;
        self.clear_smooth();
        self.add_arc(from, (x, y), radii.0, radii.1, x_axis_rotation, flags.0, flags.1)
    }

    /// `rx`/`ry` may be negative. Equal radii (after F.6.6) become [`PathItem::Arc`].
    fn add_arc(&mut self, from: (f64, f64), to: (f64, f64), rx: f64, ry: f64, rotation_deg: f64, large_arc: bool, sweep: bool) -> Result<()> {
        let rx = rx.abs();
        let ry = ry.abs();
        if from.0 == to.0 && from.1 == to.1 {
            return Ok(());
        }
        if rx == 0.0 || ry == 0.0 {
            self.items.push(PathItem::El(kurbo::PathEl::LineTo(kurbo::Point::new(to.0, to.1))));
            return Ok(());
        }
        let phi = rotation_deg.to_radians();
        let (crx, cry) = correct_radii(from, to, rx, ry, phi)?;
        if circular_radii(crx, cry) {
            self.items.push(PathItem::Arc { from, to, r: crx.max(cry), large_arc, sweep });
            return Ok(());
        }
        for element in arc_to_curves(from, to, rx, ry, phi, large_arc, sweep) {
            self.items.push(PathItem::El(element));
        }
        Ok(())
    }

    fn resolve(&self, abs: bool, x: f64, y: f64) -> Result<(f64, f64)> {
        if !self.has_point {
            return Err(invalid("invalid path: command before moveto"));
        }
        if abs { finite_xy(x, y) } else { finite_xy(self.cx + x, self.cy + y) }
    }

    fn clear_smooth(&mut self) {
        self.prev_cubic = false;
        self.prev_quad = false;
    }

    fn set_cubic(&mut self, x: f64, y: f64) {
        self.ctrl_x = x;
        self.ctrl_y = y;
        self.prev_cubic = true;
        self.prev_quad = false;
    }

    fn set_quad(&mut self, x: f64, y: f64) {
        self.ctrl_x = x;
        self.ctrl_y = y;
        self.prev_quad = true;
        self.prev_cubic = false;
    }

    /// Reflection of the previous control through the current point (SVG S/T rules).
    fn reflect_ctrl(&self) -> (f64, f64) {
        (2.0 * self.cx - self.ctrl_x, 2.0 * self.cy - self.ctrl_y)
    }
}

/// `true` when corrected radii agree to 1e-9 relative.
fn circular_radii(rx: f64, ry: f64) -> bool {
    let scale = rx.max(ry);
    scale > 0.0 && (rx - ry).abs() <= 1e-9 * scale
}

/// SVG F.6.6: scale radii up uniformly when the ellipse cannot reach both endpoints.
fn correct_radii(from: (f64, f64), to: (f64, f64), rx: f64, ry: f64, phi: f64) -> Result<(f64, f64)> {
    let (sin_phi, cos_phi) = phi.sin_cos();
    let dx = (from.0 - to.0) * 0.5;
    let dy = (from.1 - to.1) * 0.5;
    let x1p = cos_phi * dx + sin_phi * dy;
    let y1p = -sin_phi * dx + cos_phi * dy;
    let scale = (x1p / rx).hypot(y1p / ry);
    if !scale.is_finite() {
        return Err(invalid("coordinate is not finite"));
    }
    if scale <= 1.0 {
        return Ok((rx, ry));
    }
    let rx = rx * scale;
    let ry = ry * scale;
    if rx.is_finite() && ry.is_finite() { Ok((rx, ry)) } else { Err(invalid("coordinate is not finite")) }
}

fn arc_to_curves(from: (f64, f64), to: (f64, f64), rx: f64, ry: f64, x_rotation: f64, large_arc: bool, sweep: bool) -> Vec<kurbo::PathEl> {
    let svg_arc = kurbo::SvgArc {
        from: kurbo::Point::new(from.0, from.1),
        to: kurbo::Point::new(to.0, to.1),
        radii: kurbo::Vec2::new(rx, ry),
        x_rotation,
        large_arc,
        sweep,
    };
    let line = kurbo::PathEl::LineTo(kurbo::Point::new(to.0, to.1));
    let Some(arc) = kurbo::Arc::from_svg_arc(&svg_arc) else {
        return vec![line];
    };
    let curves: Vec<kurbo::PathEl> =
        arc.append_iter(ARC_CUBIC_TOL).filter(|element| matches!(element, kurbo::PathEl::CurveTo(_, _, _) | kurbo::PathEl::LineTo(_))).collect();
    if curves.is_empty() { vec![line] } else { curves }
}

fn path_from_items(items: &[PathItem]) -> kurbo::BezPath {
    let mut path = kurbo::BezPath::new();
    for item in items {
        match *item {
            PathItem::El(element) => push_el(&mut path, element),
            PathItem::Arc { from, to, r, large_arc, sweep } => {
                for element in arc_to_curves(from, to, r, r, 0.0, large_arc, sweep) {
                    push_el(&mut path, element);
                }
            }
        }
    }
    path
}

fn push_el(path: &mut kurbo::BezPath, element: kurbo::PathEl) {
    match element {
        kurbo::PathEl::MoveTo(point) => path.move_to(point),
        kurbo::PathEl::LineTo(point) => path.line_to(point),
        kurbo::PathEl::QuadTo(p1, p2) => path.quad_to(p1, p2),
        kurbo::PathEl::CurveTo(p1, p2, p3) => path.curve_to(p1, p2, p3),
        kurbo::PathEl::ClosePath => path.close_path(),
    }
}

/// Affine on one element. `BezPath::apply_affine` multiplies each element the same way.
fn transform_el(full: Affine, element: kurbo::PathEl) -> kurbo::PathEl {
    full * element
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
        consume_el(&mut pen, &mut subpaths, *element, &opts, &mut produced, budget)?;
    }
    if !pen.current.is_empty() {
        subpaths.push(pen.current);
    }
    Ok(subpaths)
}

fn consume_el(
    pen: &mut Pen,
    subpaths: &mut Vec<Vec<Segment>>,
    element: kurbo::PathEl,
    opts: &crate::arcfit::FitOptions,
    produced: &mut usize,
    budget: usize,
) -> Result<()> {
    match element {
        kurbo::PathEl::MoveTo(point) => move_pen(pen, subpaths, point),
        kurbo::PathEl::LineTo(point) => line_pen(pen, point, produced, budget),
        kurbo::PathEl::QuadTo(p1, p2) => fit_quad(pen, p1, p2, opts, produced, budget),
        kurbo::PathEl::CurveTo(p1, p2, p3) => fit_cubic(pen, p1, p2, p3, opts, produced, budget),
        kurbo::PathEl::ClosePath => close_pen(pen, subpaths, produced, budget),
    }
}

/// Map user-space items through a similarity and emit exact circular arcs.
fn fit_items(items: &[PathItem], full: Affine, scale: f64, tolerance: f64, budget: usize) -> Result<Vec<Vec<Segment>>> {
    let [a, b, c, d, _, _] = full.as_coeffs();
    let flip_sweep = a * d - b * c < 0.0;
    let mut subpaths = Vec::new();
    let mut pen = Pen { start: None, last: None, current: Vec::new() };
    let mut produced = 0usize;
    let opts = crate::arcfit::FitOptions::new(tolerance);
    for item in items {
        match *item {
            PathItem::El(element) => {
                let element = transform_el(full, element);
                consume_el(&mut pen, &mut subpaths, element, &opts, &mut produced, budget)?;
            }
            PathItem::Arc { from, to, r, large_arc, sweep } => {
                let sweep = if flip_sweep { !sweep } else { sweep };
                let arc =
                    OutArc { from: transform_xy(full, from.0, from.1), to: transform_xy(full, to.0, to.1), radius: r * scale, large_arc, sweep };
                push_similarity_arc(&mut pen, arc, &mut produced, budget)?;
            }
        }
    }
    if !pen.current.is_empty() {
        subpaths.push(pen.current);
    }
    Ok(subpaths)
}

struct OutArc {
    from: (f64, f64),
    to: (f64, f64),
    radius: f64,
    large_arc: bool,
    sweep: bool,
}

fn push_similarity_arc(pen: &mut Pen, arc: OutArc, produced: &mut usize, budget: usize) -> Result<()> {
    let to_pt = geom_point(kurbo::Point::new(arc.to.0, arc.to.1))?;
    let Some(current) = pen.last else {
        pen.start = Some(to_pt);
        pen.last = Some(to_pt);
        return Ok(());
    };
    match circular_geom(arc.from, arc.to, arc.radius, arc.large_arc, arc.sweep)? {
        Some(geom) => push_one(pen, Segment::Arc(geom), to_pt, produced, budget),
        None => push_line(pen, current, to_pt, produced, budget),
    }
}

fn push_one(pen: &mut Pen, segment: Segment, end: Point, produced: &mut usize, budget: usize) -> Result<()> {
    in_range(segment)?;
    *produced = produced.checked_add(1).ok_or_else(too_many)?;
    if *produced > budget {
        return Err(too_many());
    }
    pen.current.push(segment);
    pen.last = Some(end);
    Ok(())
}

/// Endpoint-to-center conversion for a circle (SVG F.6.5; rotation drops out when `rx == ry`).
///
/// `sweep` is the SVG flag in Y-up output space: `true` is CCW, a positive geom sweep.
fn circular_geom(from: (f64, f64), to: (f64, f64), radius: f64, large_arc: bool, sweep: bool) -> Result<Option<Arc>> {
    let (x1, y1) = from;
    let (x2, y2) = to;
    if ![x1, y1, x2, y2, radius].into_iter().all(f64::is_finite) {
        return Err(invalid("coordinate is not finite"));
    }
    let dx = (x1 - x2) * 0.5;
    let dy = (y1 - y2) * 0.5;
    let half = dx.hypot(dy);
    if half == 0.0 || radius == 0.0 {
        return Ok(None);
    }
    let mut r = radius.abs();
    if r < half {
        r = half;
    }
    // + when the flags differ, − when they match (SVG F.6.5).
    let sign = if large_arc == sweep { -1.0 } else { 1.0 };
    let mut radicand = (r / half) * (r / half) - 1.0;
    if radicand < 0.0 {
        radicand = 0.0;
    }
    let coef = sign * radicand.sqrt();
    let cx = (x1 + x2) * 0.5 + coef * dy;
    let cy = (y1 + y2) * 0.5 - coef * dx;
    if !cx.is_finite() || !cy.is_finite() {
        return Err(invalid("coordinate is not finite"));
    }
    let start_angle = (y1 - cy).atan2(x1 - cx);
    let end_angle = (y2 - cy).atan2(x2 - cx);
    let sweep_angle = signed_sweep(start_angle, end_angle, sweep);
    if !start_angle.is_finite() || !sweep_angle.is_finite() {
        return Err(invalid("coordinate is not finite"));
    }
    if sweep_angle == 0.0 {
        return Ok(None);
    }
    let center = Point::new(cx, cy).map_err(crate::ImportError::from_geom)?;
    let arc = Arc::new(center, r, start_angle, sweep_angle).map_err(crate::ImportError::from_geom)?;
    Ok(Some(arc))
}

/// Signed sweep in `(-TAU, TAU]`, CCW when `positive` (SVG sweep flag in Y-up space).
fn signed_sweep(start_angle: f64, end_angle: f64, positive: bool) -> f64 {
    let mut delta = end_angle - start_angle;
    if positive {
        if delta < 0.0 {
            delta += TAU;
        }
    } else if delta > 0.0 {
        delta -= TAU;
    }
    delta.clamp(-TAU, TAU)
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

    fn drawing(svg: &str) -> crate::Drawing {
        super::import_svg(svg, &ImportOptions::new(Tolerance::default())).expect("import")
    }

    fn near(a: campanion_geom::Point, b: campanion_geom::Point, tol: f64) -> bool {
        (a.x() - b.x()).abs() <= tol && (a.y() - b.y()).abs() <= tol
    }

    fn seg_ends(segment: &campanion_geom::Segment) -> (campanion_geom::Point, campanion_geom::Point) {
        (segment.start(), segment.end())
    }

    /// Slot of two lines and two semicircle caps. `positive_sweep` is the geom sweep sign.
    fn assert_circular_slot(svg: &str, positive_sweep: bool) {
        let drawing = drawing(svg);
        assert_eq!(drawing.contours().len(), 1, "contours");
        let contour = drawing.contours()[0].contour();
        let segments = contour.segments();
        assert_eq!(segments.len(), 4, "segment count");
        let bounds = contour.bounding_box().expect("bounds");
        assert!((bounds.min().x() - 35.0).abs() < 1e-6, "min x {}", bounds.min().x());
        assert!((bounds.max().x() - 65.0).abs() < 1e-6, "max x {}", bounds.max().x());
        assert!((bounds.min().y() - 25.0).abs() < 1e-6, "min y {}", bounds.min().y());
        assert!((bounds.max().y() - 35.0).abs() < 1e-6, "max y {}", bounds.max().y());

        let mut line_ends = Vec::new();
        let mut line_hits = Vec::new();
        let mut sweeps = Vec::new();
        let mut mids = Vec::new();
        for segment in segments {
            match segment {
                campanion_geom::Segment::Line(line) => {
                    assert!((line.length() - 20.0).abs() < 1e-6, "line {}", line.length());
                    line_ends.push((line.start(), line.end()));
                }
                campanion_geom::Segment::Arc(arc) => {
                    assert!((arc.radius() - 5.0).abs() <= 1e-9, "radius {}", arc.radius());
                    let sweep = arc.sweep();
                    assert!((sweep.abs() - std::f64::consts::PI).abs() < 1e-5, "sweep {sweep}");
                    if positive_sweep {
                        assert!(sweep > 0.0, "sweep {sweep}");
                    } else {
                        assert!(sweep < 0.0, "sweep {sweep}");
                    }
                    sweeps.push(sweep);
                    let mid = arc.point_at(0.5);
                    mids.push((mid.x(), mid.y()));
                    let start = arc.start_point();
                    let end = arc.end_point();
                    line_hits.push((start, end));
                }
            }
        }
        assert_eq!(line_ends.len(), 2);
        assert_eq!(sweeps.len(), 2);
        for (start, end) in line_hits {
            assert!(line_ends.iter().any(|(_, line_end)| near(*line_end, start, 1e-9)), "arc start does not meet a line");
            assert!(line_ends.iter().any(|(line_start, _)| near(*line_start, end, 1e-9)), "arc end does not meet a line");
        }
        let n = segments.len();
        for index in 0..n {
            let (_, end) = seg_ends(&segments[index]);
            let (start, _) = seg_ends(&segments[(index + 1) % n]);
            assert!(near(end, start, 1e-9), "contour open at {index}");
        }
        assert!(mids.iter().any(|&(x, y)| (x - 35.0).abs() < 1e-6 && (y - 30.0).abs() < 1e-6), "left cap {mids:?}");
        assert!(mids.iter().any(|&(x, y)| (x - 65.0).abs() < 1e-6 && (y - 30.0).abs() < 1e-6), "right cap {mids:?}");
    }

    #[test]
    fn circular_slot_imports_two_arcs() {
        let svg =
            r#"<svg width="100mm" height="60mm" viewBox="0 0 100 60"><path d="M 40 25 H 60 A 5 5 0 0 1 60 35 H 40 A 5 5 0 0 1 40 25 Z"/></svg>"#;
        assert_circular_slot(svg, false);
    }

    #[test]
    fn mirrored_circular_slot_flips_sweep() {
        let svg = r#"<svg width="100mm" height="60mm" viewBox="0 0 100 60"><path transform="scale(-1,1) translate(-100,0)" d="M 40 25 H 60 A 5 5 0 0 1 60 35 H 40 A 5 5 0 0 1 40 25 Z"/></svg>"#;
        assert_circular_slot(svg, true);
    }

    #[test]
    fn circular_slot_negative_radii_stay_exact() {
        let svg = r#"<svg width="100mm" height="60mm" viewBox="0 0 100 60"><path d="M 40 25 H 60 A -5 -5 30 0 1 60 35 H 40 A -5 -5 30 0 1 40 25 Z"/></svg>"#;
        assert_circular_slot(svg, false);
    }

    #[test]
    fn elliptical_arc_imports_closed() {
        let svg =
            r#"<svg width="100mm" height="60mm" viewBox="0 0 100 60"><path d="M 40 25 H 60 A 5 3 0 0 1 60 35 H 40 A 5 3 0 0 1 40 25 Z"/></svg>"#;
        let drawing = drawing(svg);
        assert_eq!(drawing.contours().len(), 1, "contours");
        let contour = drawing.contours()[0].contour();
        let segments = contour.segments();
        assert!(!segments.is_empty(), "no segments");
        assert!(segments.iter().any(|segment| matches!(segment, campanion_geom::Segment::Arc(_))), "expected arc-fitted segments");
        let n = segments.len();
        for index in 0..n {
            let (_, end) = seg_ends(&segments[index]);
            let (start, _) = seg_ends(&segments[(index + 1) % n]);
            assert!(near(end, start, 1e-4), "contour open at {index}");
        }
        let bounds = contour.bounding_box().expect("bounds");
        let width = bounds.max().x() - bounds.min().x();
        let height = bounds.max().y() - bounds.min().y();
        assert!(width > 32.0, "width {width}");
        assert!(height > 8.0, "height {height}");
    }
}
