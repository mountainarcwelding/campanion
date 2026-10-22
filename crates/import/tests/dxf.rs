//! ASCII DXF import through the public API.
//!
//! `fixtures/sample.dxf` is a hand-written millimetre drawing: a 100 by 60 plate on `OUTER`
//! (four separate lines), two radius-5 circles and a closed bulge slot on `INNER`, and one
//! `TEXT` entity the importer does not read.

use campanion_geom::{Point, Segment, Tolerance};
use campanion_import::{Drawing, ImportOptions, import_dxf};

const SAMPLE: &str = include_str!("fixtures/sample.dxf");

/// 100 mm by 60 mm rectangle.
const PLATE_AREA: f64 = 100.0 * 60.0;
/// Circle of radius 5 mm.
const HOLE_AREA: f64 = std::f64::consts::PI * 25.0;
/// Slot: 20 mm straight section, 10 mm wide, closed by two semicircle bulges.
const SLOT_AREA: f64 = 200.0 + HOLE_AREA;

type TestResult = Result<(), String>;

fn millimetres() -> ImportOptions {
    ImportOptions::new(Tolerance::default())
}

fn import(text: &str, options: &ImportOptions) -> Result<Drawing, String> {
    import_dxf(text, options).map_err(|err| err.to_string())
}

fn area(contour: &campanion_geom::Contour) -> Result<f64, String> {
    contour.signed_area().map_err(|err| err.to_string())
}

fn assert_area(contour: &campanion_geom::Contour, expected: f64) -> TestResult {
    let got = area(contour)?;
    assert!((got - expected).abs() < 1e-6, "area {got} expected {expected}");
    Ok(())
}

fn file(units: i32, entities: &str) -> String {
    format!("0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n{units}\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n{entities}0\nENDSEC\n0\nEOF\n")
}

#[test]
fn sample_plate_has_outer_and_three_inner_contours() -> TestResult {
    let drawing = import(SAMPLE, &millimetres())?;
    assert_eq!(drawing.skipped_entities(), 1);
    assert_eq!(drawing.contours().len(), 4);

    let outer: Vec<_> = drawing.contours().iter().filter(|item| item.layer() == "OUTER").collect();
    let inner: Vec<_> = drawing.contours().iter().filter(|item| item.layer() == "INNER").collect();
    assert_eq!(outer.len(), 1);
    assert_eq!(inner.len(), 3);

    let plate = outer.first().ok_or("missing OUTER")?.contour();
    assert!(plate.is_closed());
    assert_eq!(plate.segments().len(), 4);
    assert!(plate.segments().iter().all(|segment| matches!(segment, Segment::Line(_))));
    assert_area(plate, PLATE_AREA)?;
    let bounds = plate.bounding_box().map_err(|err| err.to_string())?;
    assert!((bounds.width() - 100.0).abs() < 1e-9);
    assert!((bounds.height() - 60.0).abs() < 1e-9);

    let mut circles = Vec::new();
    let mut slots = Vec::new();
    for item in inner {
        let contour = item.contour();
        assert!(contour.is_closed());
        match contour.segments().len() {
            1 => circles.push(contour),
            4 => slots.push(contour),
            other => return Err(format!("INNER contour has {other} segments")),
        }
    }
    assert_eq!(circles.len(), 2);
    assert_eq!(slots.len(), 1);

    for contour in circles {
        assert_area(contour, HOLE_AREA)?;
        let segment = contour.segments().first().copied().ok_or("circle has no edge")?;
        let Segment::Arc(arc) = segment else {
            return Err("circle was not stored as an arc".into());
        };
        assert!((arc.radius() - 5.0).abs() < 1e-9);
        assert!((arc.sweep() - std::f64::consts::TAU).abs() < 1e-9);
    }

    let slot = slots.first().ok_or("missing slot")?;
    assert_area(slot, SLOT_AREA)?;
    let arcs = slot.segments().iter().filter(|segment| matches!(segment, Segment::Arc(_))).count();
    let lines = slot.segments().iter().filter(|segment| matches!(segment, Segment::Line(_))).count();
    assert_eq!(arcs, 2);
    assert_eq!(lines, 2);
    Ok(())
}

#[test]
fn arc_keeps_radius_and_sweep() -> TestResult {
    let entities = "\
0
ARC
10
0
20
0
40
10
50
0
51
90
";
    let drawing = import(&file(4, entities), &millimetres())?;
    assert_eq!(drawing.skipped_entities(), 0);
    assert_eq!(drawing.contours().len(), 1);
    let imported = drawing.contours().first().ok_or("missing arc")?;
    assert_eq!(imported.layer(), "0");
    let contour = imported.contour();
    assert!(!contour.is_closed());
    let segment = contour.segments().first().copied().ok_or("arc has no edge")?;
    let Segment::Arc(arc) = segment else {
        return Err("ARC was not stored as an arc".into());
    };
    assert!((arc.radius() - 10.0).abs() < 1e-9);
    assert!((arc.sweep() - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    let start = Point::new(10.0, 0.0).map_err(|err| err.to_string())?;
    let end = Point::new(0.0, 10.0).map_err(|err| err.to_string())?;
    assert!(arc.start_point().almost_eq(start, 1e-9));
    assert!(arc.end_point().almost_eq(end, 1e-9));
    let length = contour.length().map_err(|err| err.to_string())?;
    assert!((length - 5.0 * std::f64::consts::PI).abs() < 1e-9);
    Ok(())
}

#[test]
fn bulge_polyline_closes_into_a_circle() -> TestResult {
    // Two vertices, bulge 1 each, closed: a circle of radius 1 centred at (1, 0).
    let entities = "\
0
LWPOLYLINE
8
BULGE
90
2
70
1
10
0
20
0
42
1
10
2
20
0
42
1
";
    let drawing = import(&file(4, entities), &millimetres())?;
    assert_eq!(drawing.contours().len(), 1);
    let imported = drawing.contours().first().ok_or("missing bulge contour")?;
    assert_eq!(imported.layer(), "BULGE");
    let contour = imported.contour();
    assert!(contour.is_closed());
    assert_eq!(contour.segments().len(), 2);
    assert!(contour.segments().iter().all(|segment| matches!(segment, Segment::Arc(_))));
    assert_area(contour, std::f64::consts::PI)?;
    Ok(())
}

#[test]
fn open_chain_bridges_a_gap_and_leaves_the_rest_open() -> TestResult {
    // The first two lines are 0.2 mm apart and join. The third stays a separate open path.
    let entities = "\
0
LINE
8
OPEN
10
0
20
0
11
10
21
0
0
LINE
8
OPEN
10
10.2
20
0
11
20
21
0
0
LINE
8
OPEN
10
100
20
100
11
100
21
105
";
    let options = ImportOptions::with_gap(Tolerance::default(), 0.5).map_err(|err| err.to_string())?;
    let drawing = import(&file(4, entities), &options)?;
    assert_eq!(drawing.contours().len(), 2);
    assert!(drawing.contours().iter().all(|item| item.layer() == "OPEN" && !item.contour().is_closed()));

    let mut by_edges: Vec<_> = drawing.contours().iter().collect();
    by_edges.sort_by_key(|item| item.contour().segments().len());
    let leftover = by_edges.first().ok_or("missing leftover")?.contour();
    let chained = by_edges.get(1).ok_or("missing chain")?.contour();
    assert_eq!(leftover.segments().len(), 1);
    assert_eq!(chained.segments().len(), 3);
    let leftover_length = leftover.length().map_err(|err| err.to_string())?;
    let chained_length = chained.length().map_err(|err| err.to_string())?;
    assert!((leftover_length - 5.0).abs() < 1e-9, "{leftover_length}");
    assert!((chained_length - 20.0).abs() < 1e-9, "{chained_length}");
    Ok(())
}

#[test]
fn inches_scale_coordinates_by_25_4() -> TestResult {
    let entities = "\
0
LINE
10
2
20
-1
11
2
21
1
";
    let drawing = import(&file(1, entities), &millimetres())?;
    assert_eq!(drawing.contours().len(), 1);
    let contour = drawing.contours().first().ok_or("missing line")?.contour();
    let length = contour.length().map_err(|err| err.to_string())?;
    assert!((length - 2.0 * 25.4).abs() < 1e-9, "{length}");
    let segment = contour.segments().first().copied().ok_or("line has no edge")?;
    let Segment::Line(line) = segment else {
        return Err("scaled entity was not a line".into());
    };
    let start = Point::new(2.0 * 25.4, -25.4).map_err(|err| err.to_string())?;
    let end = Point::new(2.0 * 25.4, 25.4).map_err(|err| err.to_string())?;
    assert!(line.start().almost_eq(start, 1e-9));
    assert!(line.end().almost_eq(end, 1e-9));
    Ok(())
}

#[test]
fn garbage_input_is_an_error() -> TestResult {
    let options = millimetres();
    let samples = [
        "",
        "this is not a dxf",
        "0\n",
        "   \n",
        "0\nSECTION\n2\nHEADER\n0\nENDSEC\n0\nEOF\n",
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\nnan\n20\n0\n11\n1\n21\n0\n0\nENDSEC\n0\nEOF\n",
        "999\norphan\n",
    ];
    for text in samples {
        assert!(import_dxf(text, &options).is_err(), "expected Err for {text:?}");
    }
    Ok(())
}
