//! SVG import: arcs, paths, transforms, units, layers, and errors.

#![allow(clippy::expect_used)]

use campanion_geom::{Segment, Tolerance};
use campanion_import::{ImportOptions, import_svg};

fn options() -> ImportOptions {
    ImportOptions::new(Tolerance::default())
}

fn drawing(svg: &str) -> campanion_import::Drawing {
    import_svg(svg, &options()).expect("import")
}

fn bbox_of(svg: &str) -> (f64, f64, f64, f64) {
    let drawing = drawing(svg);
    assert_eq!(drawing.contours().len(), 1, "one contour");
    let bounds = drawing.contours()[0].contour().bounding_box().expect("bounds");
    let min = bounds.min();
    let max = bounds.max();
    (min.x(), min.y(), max.x(), max.y())
}

fn assert_close(actual: f64, expected: f64, tol: f64, label: &str) {
    assert!((actual - expected).abs() < tol, "{label}: {actual} != {expected} (tol {tol})");
}

#[test]
fn circle_uniform_view_box_is_one_full_arc() {
    // 100mm over viewBox 50 is a uniform scale of 2. Circle centre (10, 5) r=3
    // maps to (20, 90) r=6 after that scale and the page Y flip.
    let svg = r#"<svg width="100mm" height="100mm" viewBox="0 0 50 50"><circle cx="10" cy="5" r="3"/></svg>"#;
    let drawing = drawing(svg);
    assert_eq!(drawing.contours().len(), 1);
    let segments = drawing.contours()[0].contour().segments();
    assert_eq!(segments.len(), 1, "a similarity keeps the circle as one arc");
    let Segment::Arc(arc) = &segments[0] else {
        panic!("expected a single arc");
    };
    assert_close(arc.radius(), 6.0, 1e-6, "radius");
    assert_close(arc.center().x(), 20.0, 1e-6, "centre x");
    assert_close(arc.center().y(), 90.0, 1e-6, "centre y");
    assert_close(arc.sweep().abs(), std::f64::consts::TAU, 1e-6, "full sweep");
}

#[test]
fn relative_path_is_closed_with_bbox() {
    // m 10 10 h 20 v 10 h -20 z is the rectangle x=10..30, y=10..20 in user space.
    // Page height is 40mm with a 1:1 viewBox, so Y flip puts it at y=20..30.
    let svg = r#"<svg width="40mm" height="40mm" viewBox="0 0 40 40"><path d="m 10 10 h 20 v 10 h -20 z"/></svg>"#;
    let drawing = drawing(svg);
    assert_eq!(drawing.contours().len(), 1);
    assert!(drawing.contours()[0].contour().is_closed(), "z closes the contour");
    let bounds = drawing.contours()[0].contour().bounding_box().expect("bounds");
    let min = bounds.min();
    let max = bounds.max();
    assert_close(min.x(), 10.0, 1e-6, "min x");
    assert_close(max.x(), 30.0, 1e-6, "max x");
    assert_close(min.y(), 20.0, 1e-6, "min y");
    assert_close(max.y(), 30.0, 1e-6, "max y");
}

#[test]
fn elliptical_arc_fits_to_closed_bbox() {
    // Two semicircles make a full ellipse centred at (50, 30), rx=30, ry=15.
    // User bbox x=20..80, y=15..45. Y flip in a 100mm page moves y to 55..85.
    // The arc fit stays on that box within tolerance.
    let svg = r#"<svg width="100mm" height="100mm" viewBox="0 0 100 100"><path d="M20 30 A30 15 0 0 0 80 30 A30 15 0 0 0 20 30 Z"/></svg>"#;
    let drawing = drawing(svg);
    assert_eq!(drawing.contours().len(), 1);
    let contour = drawing.contours()[0].contour();
    assert!(contour.is_closed(), "Z closes the contour");
    let segments = contour.segments();
    assert!(segments.len() > 1, "an elliptical arc is fitted");
    assert!(segments.iter().any(|segment| matches!(segment, Segment::Arc(_))), "fitted with arcs");
    assert!(segments.iter().all(|segment| matches!(segment, Segment::Line(_) | Segment::Arc(_))), "fitted with lines and arcs");
    let bounds = contour.bounding_box().expect("bounds");
    let min = bounds.min();
    let max = bounds.max();
    assert_close(min.x(), 20.0, 1e-4, "min x");
    assert_close(max.x(), 80.0, 1e-4, "max x");
    assert_close(min.y(), 55.0, 0.05, "min y");
    assert_close(max.y(), 85.0, 0.05, "max y");
}

#[test]
fn group_translate_then_scale_maps_rect() {
    // SVG applies the transform list right to left: scale(2) then translate(5, 0).
    // Local rect (10, 10, 10, 10) becomes user x=25..45, y=20..40.
    // Y flip in a 100mm page puts y at 60..80.
    let svg = r#"<svg width="100mm" height="100mm" viewBox="0 0 100 100"><g transform="translate(5,0) scale(2)"><rect x="10" y="10" width="10" height="10"/></g></svg>"#;
    let (x0, y0, x1, y1) = bbox_of(svg);
    assert_close(x0, 25.0, 1e-6, "min x");
    assert_close(x1, 45.0, 1e-6, "max x");
    assert_close(y0, 60.0, 1e-6, "min y");
    assert_close(y1, 80.0, 1e-6, "max y");
}

#[test]
fn inch_viewport_maps_view_box_to_millimetres() {
    // 1in = 25.4mm over viewBox 0..96, so a rect covering the viewBox is 0..25.4mm.
    let svg = r#"<svg width="1in" height="1in" viewBox="0 0 96 96"><rect x="0" y="0" width="96" height="96"/></svg>"#;
    let (x0, y0, x1, y1) = bbox_of(svg);
    assert_close(x0, 0.0, 1e-6, "min x");
    assert_close(y0, 0.0, 1e-6, "min y");
    assert_close(x1, 25.4, 1e-6, "max x");
    assert_close(y1, 25.4, 1e-6, "max y");
}

#[test]
fn inkscape_labels_are_layer_names() {
    let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape" width="100mm" height="50mm" viewBox="0 0 100 50"><g inkscape:label="A"><rect x="0" y="0" width="10" height="10"/></g><g inkscape:label="B"><rect x="20" y="0" width="10" height="10"/></g></svg>"#;
    let drawing = drawing(svg);
    assert_eq!(drawing.contours().len(), 2);
    let layers: Vec<_> = drawing.contours().iter().map(|contour| contour.layer()).collect();
    assert_eq!(layers, ["A", "B"]);
}

#[test]
fn text_is_counted_skipped() {
    let svg = r#"<svg width="100mm" height="50mm" viewBox="0 0 100 50"><text x="10" y="20">Hello</text></svg>"#;
    let drawing = drawing(svg);
    assert!(drawing.contours().is_empty());
    assert_eq!(drawing.skipped_types(), [("text".to_string(), 1usize)]);
}

#[test]
fn malformed_xml_returns_err() {
    let result = import_svg("<svg>", &options());
    assert!(result.is_err());
}
