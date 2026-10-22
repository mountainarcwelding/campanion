//! Chamfer (deburr) a closed edge with a V-bit.
//!
//! Z is up and the stock top is `stock_z`. The included angle is the cone's point angle; half of
//! that angle is `a`. The chamfer leg `w` is the horizontal width of the bevel into the material.
//! Tip clearance `e` is how far the tip sits below the bottom of the bevel, and `rt` is the tip
//! radius.
//!
//! The tool tip is at `stock_z - (w / tan(a) + e)`. The axis stands off the edge, away from the
//! material, by `rt + e * tan(a)`. Outside keeps that axis outside the contour (the part is
//! inside). Inside keeps it in the contour (a hole or pocket edge). One plunge pass cuts the
//! offset contour.

use campanion_geom::{Contour, Orientation, Tolerance};

use crate::Result;
use crate::check::{finite, invalid, non_negative, positive};
use crate::depth::{DepthEntry, DepthPlan, depth_toolpath};
use crate::profile::ProfileSide;
use crate::toolpath::Toolpath;

/// A finished chamfer pass.
#[derive(Clone, Debug, PartialEq)]
pub struct Chamfer {
    toolpath: Toolpath,
    tip_z: f64,
    offset: f64,
}

impl Chamfer {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Tool-tip height. The bevel is cut in one plunge to this Z.
    pub fn tip_z(&self) -> f64 {
        self.tip_z
    }

    /// Unsigned distance from the edge to the tool axis, away from the material.
    pub fn offset(&self) -> f64 {
        self.offset
    }
}

/// V-bit chamfer along one closed contour.
#[derive(Clone, Debug, PartialEq)]
pub struct ChamferOp {
    included_angle: f64,
    tip_diameter: f64,
    width: f64,
    side: ProfileSide,
    tip_clearance: f64,
    stock_z: f64,
    max_diameter: Option<f64>,
    tolerance: Tolerance,
}

impl ChamferOp {
    /// `included_angle_rad` is the point angle in radians. `tip_diameter` and `width` are millimetres.
    ///
    /// The side defaults to [`ProfileSide::Outside`], tip clearance and stock Z default to zero.
    pub fn new(included_angle_rad: f64, tip_diameter: f64, width: f64) -> Self {
        Self {
            included_angle: included_angle_rad,
            tip_diameter,
            width,
            side: ProfileSide::Outside,
            tip_clearance: 0.0,
            stock_z: 0.0,
            max_diameter: None,
            tolerance: Tolerance::default(),
        }
    }

    pub fn side(mut self, side: ProfileSide) -> Self {
        self.side = side;
        self
    }

    /// How far the tip goes below the bottom of the bevel, in millimetres. Defaults to zero.
    pub fn tip_clearance(mut self, clearance: f64) -> Self {
        self.tip_clearance = clearance;
        self
    }

    /// Stock top. Defaults to zero. Z is up.
    pub fn stock_z(mut self, stock_z: f64) -> Self {
        self.stock_z = stock_z;
        self
    }

    /// Largest cutting diameter of the V-bit, in millimetres.
    ///
    /// The cone radius at the stock surface is `rt + w + e * tan(a)`. A wider chamfer does not
    /// fit this tool.
    pub fn max_diameter(mut self, diameter: f64) -> Self {
        self.max_diameter = Some(diameter);
        self
    }

    pub fn tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Plunge once along the offset of `contour`.
    pub fn generate(&self, contour: &Contour) -> Result<Chamfer> {
        if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
            return Err(invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
        }
        let angle = finite(self.included_angle, "included angle")?;
        if angle <= 0.0 || angle >= std::f64::consts::PI {
            return Err(invalid("included angle must be in (0, pi)"));
        }
        let tip_diameter = non_negative(self.tip_diameter, "tip diameter")?;
        let width = positive(self.width, "chamfer width")?;
        let clearance = non_negative(self.tip_clearance, "tip clearance")?;
        let stock_z = finite(self.stock_z, "stock z")?;
        if matches!(self.side, ProfileSide::On) {
            return Err(invalid("chamfer side must be inside or outside"));
        }
        if !contour.is_closed() {
            return Err(invalid("chamfer needs a closed contour"));
        }

        let tan_half = half_tangent(angle)?;
        let tip_radius = tip_diameter * 0.5;
        let drop = width / tan_half + clearance;
        if !drop.is_finite() || drop <= 0.0 {
            return Err(invalid("chamfer depth is not finite"));
        }
        let tip_z = stock_z - drop;
        finite(tip_z, "tip z")?;
        // Axis stands off the edge by the tip radius plus the clearance's horizontal reach.
        let radial = tip_radius + clearance * tan_half;
        if !radial.is_finite() || radial < 0.0 {
            return Err(invalid("chamfer offset is not finite"));
        }
        // Cone radius where the flank meets the stock top: rt + (w/tan(a) + e)*tan(a).
        let cone_at_stock = tip_radius + width + clearance * tan_half;
        if !cone_at_stock.is_finite() {
            return Err(invalid("cone radius is not finite"));
        }
        if let Some(max_diameter) = self.max_diameter {
            let max_diameter = positive(max_diameter, "max diameter")?;
            if cone_at_stock > max_diameter * 0.5 + 1e-9 {
                return Err(invalid("chamfer too wide for the tool"));
            }
        }

        let path = offset_contour(contour, self.side, radial, &self.tolerance)?;
        let plan = DepthPlan::new(stock_z, drop, drop, DepthEntry::Plunge);
        let toolpath = depth_toolpath(&path, &plan, &self.tolerance)?;
        Ok(Chamfer { toolpath, tip_z, offset: radial })
    }
}

fn half_tangent(included_angle: f64) -> Result<f64> {
    let tan_half = (included_angle * 0.5).tan();
    if tan_half.is_finite() && tan_half > 0.0 { Ok(tan_half) } else { Err(invalid("included angle must be in (0, pi)")) }
}

fn offset_contour(contour: &Contour, side: ProfileSide, radial: f64, tolerance: &Tolerance) -> Result<Contour> {
    // A shift below the linear tolerance is noise. Cut the contour itself.
    if radial <= tolerance.linear() {
        return Ok(contour.clone());
    }
    let signed = signed_offset(contour, side, radial, tolerance)?;
    if signed.abs() <= tolerance.linear() {
        return Ok(contour.clone());
    }
    let parts = campanion_geom::offset(contour, signed, tolerance)?;
    if parts.is_empty() {
        return Err(invalid(match side {
            ProfileSide::Inside => "chamfer does not fit inside the contour",
            ProfileSide::Outside | ProfileSide::On => "offset produced no contour",
        }));
    }
    if parts.len() != 1 {
        return Err(invalid(format!("offset split the chamfer into {} contours", parts.len())));
    }
    let Some(path) = parts.into_iter().next() else {
        return Err(invalid("offset produced no contour"));
    };
    if !path.is_closed() {
        return Err(invalid("offset of a closed contour did not stay closed"));
    }
    Ok(path)
}

/// Signed offset. Positive geometry offset is to the left of travel.
///
/// Outside a counter-clockwise contour is to the right (negative). A clockwise contour flips
/// that sign. Inside is the opposite direction.
fn signed_offset(contour: &Contour, side: ProfileSide, magnitude: f64, tolerance: &Tolerance) -> Result<f64> {
    let into_interior = match contour.orientation(tolerance)? {
        Orientation::CounterClockwise => magnitude,
        Orientation::Clockwise => -magnitude,
        Orientation::Degenerate => return Err(invalid("degenerate contour")),
    };
    Ok(match side {
        ProfileSide::Inside => into_interior,
        ProfileSide::Outside => -into_interior,
        ProfileSide::On => 0.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolpath::{CutKind, Move};
    use crate::{DepthEntry, DepthPlan, PathError, depth_toolpath};
    use campanion_geom::{Line, Point, Polyline, Segment, Vertex};
    use proptest::prelude::*;
    use std::f64::consts::{FRAC_PI_2, PI, TAU};

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} != {b} (tol {tol})");
    }

    fn rect(w: f64, h: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![edge(p(0.0, 0.0), p(w, 0.0)), edge(p(w, 0.0), p(w, h)), edge(p(w, h), p(0.0, h)), edge(p(0.0, h), p(0.0, 0.0))],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    fn circle(radius: f64) -> Contour {
        Polyline::new(
            vec![Vertex::new(Point::new(radius, 0.0).unwrap(), 1.0).unwrap(), Vertex::new(Point::new(-radius, 0.0).unwrap(), 1.0).unwrap()],
            true,
        )
        .unwrap()
        .to_contour(&Tolerance::default())
        .unwrap()
    }

    fn wound(contour: Contour, ccw: bool) -> Contour {
        let is_ccw = matches!(contour.orientation(&Tolerance::default()).unwrap(), Orientation::CounterClockwise);
        if is_ccw == ccw { contour } else { contour.reversed() }
    }

    fn path_samples(path: &Toolpath) -> Vec<(f64, f64)> {
        let mut cursor: Option<(f64, f64)> = None;
        let mut out = Vec::new();
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } => {
                    let point = (to.x(), to.y());
                    if let (Some(from), Move::Linear { kind: CutKind::Cut, .. }) = (cursor, *mv) {
                        out.push(((from.0 + point.0) * 0.5, (from.1 + point.1) * 0.5));
                    }
                    out.push(point);
                    cursor = Some(point);
                }
                Move::Arc { to, centre, ccw, .. } => {
                    if let Some(from) = cursor {
                        sample_arc(&mut out, from, (to.x(), to.y()), (centre.x(), centre.y()), ccw);
                    }
                    let point = (to.x(), to.y());
                    out.push(point);
                    cursor = Some(point);
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        out
    }

    fn sample_arc(out: &mut Vec<(f64, f64)>, from: (f64, f64), to: (f64, f64), centre: (f64, f64), ccw: bool) {
        let radius = (from.0 - centre.0).hypot(from.1 - centre.1);
        if radius <= 1e-12 {
            return;
        }
        let a0 = (from.1 - centre.1).atan2(from.0 - centre.0);
        let a1 = (to.1 - centre.1).atan2(to.0 - centre.0);
        let mut sweep = a1 - a0;
        if ccw {
            while sweep <= 1e-15 {
                sweep += TAU;
            }
        } else {
            while sweep >= -1e-15 {
                sweep -= TAU;
            }
        }
        const N: usize = 8;
        for step in 1..N {
            let angle = a0 + sweep * (step as f64 / N as f64);
            out.push((centre.0 + radius * angle.cos(), centre.1 + radius * angle.sin()));
        }
    }

    fn distance_to_rect(x: f64, y: f64, w: f64, h: f64) -> f64 {
        let cx = x.clamp(0.0, w);
        let cy = y.clamp(0.0, h);
        (x - cx).hypot(y - cy)
    }

    fn assert_cut_z(path: &Toolpath, z: f64) {
        for mv in path.moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut | CutKind::Plunge } | Move::Arc { to, kind: CutKind::Cut, .. } => {
                    near(to.z(), z, 1e-9);
                }
                _ => {}
            }
        }
    }

    fn assert_outside_rect(path: &Toolpath, w: f64, h: f64, distance: f64) {
        let samples = path_samples(path);
        assert!(!samples.is_empty());
        for (x, y) in samples {
            near(distance_to_rect(x, y, w, h), distance, 1e-6);
        }
    }

    fn assert_circle_radius(path: &Toolpath, radius: f64) {
        let mut saw = false;
        let mut cursor: Option<(f64, f64)> = None;
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } => {
                    near(to.x().hypot(to.y()), radius, 1e-6);
                    cursor = Some((to.x(), to.y()));
                    saw = true;
                }
                Move::Arc { to, centre, .. } => {
                    near(centre.x().hypot(centre.y()), 0.0, 1e-6);
                    let end_radius = (to.x() - centre.x()).hypot(to.y() - centre.y());
                    near(end_radius, radius, 1e-6);
                    if let Some(from) = cursor {
                        let start_radius = (from.0 - centre.x()).hypot(from.1 - centre.y());
                        near(start_radius, radius, 1e-6);
                    }
                    near(to.x().hypot(to.y()), radius, 1e-6);
                    cursor = Some((to.x(), to.y()));
                    saw = true;
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        assert!(saw);
    }

    #[test]
    fn sharp_right_angle_bit_follows_the_rectangle() {
        let shape = rect(20.0, 10.0);
        let chamfer = ChamferOp::new(FRAC_PI_2, 0.0, 1.0).generate(&shape).unwrap();
        near(chamfer.tip_z(), -1.0, 1e-9);
        near(chamfer.offset(), 0.0, 1e-12);
        assert_cut_z(chamfer.toolpath(), chamfer.tip_z());

        let drop = -chamfer.tip_z();
        let direct = depth_toolpath(&shape, &DepthPlan::new(0.0, drop, drop, DepthEntry::Plunge), &Tolerance::default()).unwrap();
        assert_eq!(chamfer.toolpath().moves(), direct.moves());

        let cuts: Vec<_> = chamfer
            .toolpath()
            .moves()
            .iter()
            .filter_map(|mv| match mv {
                Move::Linear { to, kind: CutKind::Cut } => Some(*to),
                _ => None,
            })
            .collect();
        assert_eq!(cuts.len(), shape.segments().len());
        for (cut, seg) in cuts.iter().zip(shape.segments()) {
            near(cut.x(), seg.end().x(), 1e-9);
            near(cut.y(), seg.end().y(), 1e-9);
            near(cut.z(), -1.0, 1e-9);
        }
        let plunges = chamfer.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Linear { kind: CutKind::Plunge, .. })).count();
        assert_eq!(plunges, 1);

        let raised = ChamferOp::new(FRAC_PI_2, 0.0, 1.0).stock_z(4.0).generate(&shape).unwrap();
        near(raised.tip_z(), 3.0, 1e-9);
        near(raised.offset(), 0.0, 1e-12);
    }

    #[test]
    fn clearance_offsets_a_rectangle_outside_the_material() {
        let shape = rect(20.0, 10.0);
        let chamfer = ChamferOp::new(FRAC_PI_2, 0.0, 1.0).tip_clearance(0.5).generate(&shape).unwrap();
        near(chamfer.tip_z(), -1.5, 1e-9);
        near(chamfer.offset(), 0.5, 1e-9);
        assert_cut_z(chamfer.toolpath(), -1.5);
        assert_outside_rect(chamfer.toolpath(), 20.0, 10.0, 0.5);
        // Round offset of a rectangle adds one full corner circle of radius c.
        near(chamfer.toolpath().cutting_length().unwrap(), 2.0 * (20.0 + 10.0) + TAU * 0.5, 1e-6);
    }

    #[test]
    fn sixty_degree_bit_drops_by_the_half_angle() {
        let shape = rect(20.0, 10.0);
        let chamfer = ChamferOp::new(60.0_f64.to_radians(), 0.0, 1.0).generate(&shape).unwrap();
        let expected = -1.0 / 30.0_f64.to_radians().tan();
        near(chamfer.tip_z(), expected, 1e-12);
        near(chamfer.offset(), 0.0, 1e-12);
        assert_cut_z(chamfer.toolpath(), chamfer.tip_z());
    }

    #[test]
    fn inside_a_hole_the_axis_radius_is_r_minus_c() {
        let radius = 10.0;
        // 90° bit, tip diameter 1 (rt = 0.5), clearance 0.25. c = 0.5 + 0.25 = 0.75.
        let hole = wound(circle(radius), false);
        let chamfer = ChamferOp::new(FRAC_PI_2, 1.0, 1.0).side(ProfileSide::Inside).tip_clearance(0.25).generate(&hole).unwrap();
        near(chamfer.offset(), 0.75, 1e-9);
        near(chamfer.tip_z(), -1.25, 1e-9);
        assert_circle_radius(chamfer.toolpath(), radius - chamfer.offset());
        near(chamfer.toolpath().cutting_length().unwrap(), TAU * (radius - 0.75), 1e-5);
        assert_cut_z(chamfer.toolpath(), -1.25);
    }

    #[test]
    fn clockwise_input_matches_counterclockwise_geometry() {
        let ccw = rect(20.0, 10.0);
        let cw = ccw.reversed();
        let cut = |shape: &Contour| ChamferOp::new(FRAC_PI_2, 0.0, 1.0).tip_clearance(0.5).generate(shape).unwrap();
        let outside_ccw = cut(&ccw);
        let outside_cw = cut(&cw);
        near(outside_ccw.tip_z(), outside_cw.tip_z(), 1e-12);
        near(outside_ccw.offset(), outside_cw.offset(), 1e-12);
        near(outside_ccw.offset(), 0.5, 1e-9);
        assert_outside_rect(outside_ccw.toolpath(), 20.0, 10.0, 0.5);
        assert_outside_rect(outside_cw.toolpath(), 20.0, 10.0, 0.5);
        near(outside_ccw.toolpath().cutting_length().unwrap(), outside_cw.toolpath().cutting_length().unwrap(), 1e-6);

        let radius = 12.0;
        let ccw_hole = wound(circle(radius), true);
        let cw_hole = wound(circle(radius), false);
        let inner = |shape: &Contour| ChamferOp::new(FRAC_PI_2, 0.0, 1.0).side(ProfileSide::Inside).tip_clearance(0.4).generate(shape).unwrap();
        let inside_ccw = inner(&ccw_hole);
        let inside_cw = inner(&cw_hole);
        near(inside_ccw.offset(), inside_cw.offset(), 1e-12);
        near(inside_ccw.tip_z(), inside_cw.tip_z(), 1e-12);
        assert_circle_radius(inside_ccw.toolpath(), radius - inside_ccw.offset());
        assert_circle_radius(inside_cw.toolpath(), radius - inside_cw.offset());
        near(inside_ccw.toolpath().cutting_length().unwrap(), inside_cw.toolpath().cutting_length().unwrap(), 1e-5);
    }

    #[test]
    fn hostile_chamfer_input_is_an_error() {
        let shape = rect(20.0, 10.0);
        let rejected = [
            ChamferOp::new(0.0, 0.0, 1.0),
            ChamferOp::new(PI, 0.0, 1.0),
            ChamferOp::new(-0.2, 0.0, 1.0),
            ChamferOp::new(PI + 0.2, 0.0, 1.0),
            ChamferOp::new(f64::NAN, 0.0, 1.0),
            ChamferOp::new(f64::INFINITY, 0.0, 1.0),
            ChamferOp::new(FRAC_PI_2, f64::NAN, 1.0),
            ChamferOp::new(FRAC_PI_2, f64::INFINITY, 1.0),
            ChamferOp::new(FRAC_PI_2, -1.0, 1.0),
            ChamferOp::new(FRAC_PI_2, 0.0, 0.0),
            ChamferOp::new(FRAC_PI_2, 0.0, -0.5),
            ChamferOp::new(FRAC_PI_2, 0.0, f64::NAN),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).tip_clearance(-0.1),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).tip_clearance(f64::NAN),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).stock_z(f64::NAN),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).stock_z(f64::NEG_INFINITY),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).max_diameter(f64::NAN),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).max_diameter(0.0),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).max_diameter(-4.0),
            ChamferOp::new(FRAC_PI_2, 0.0, 1.0).side(ProfileSide::On),
        ];
        for op in rejected {
            let err = op.generate(&shape).expect_err("hostile chamfer input");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }

        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(5.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let err = ChamferOp::new(FRAC_PI_2, 0.0, 1.0).generate(&open).unwrap_err();
        assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        assert!(err.to_string().contains("closed"), "{err}");

        // Cone radius at the stock top is rt + w + e*tan(a) = 1. Diameter 1 is too small; 2 fits.
        let err = ChamferOp::new(FRAC_PI_2, 0.0, 1.0).max_diameter(1.0).generate(&shape).unwrap_err();
        assert!(err.to_string().contains("chamfer too wide for the tool"), "{err}");
        assert!(ChamferOp::new(FRAC_PI_2, 0.0, 1.0).max_diameter(2.0).generate(&shape).is_ok());
        // Tip radius counts: rt = 1, w = 1, cone radius 2. Diameter 4 fits; 3.9 does not.
        assert!(ChamferOp::new(FRAC_PI_2, 2.0, 1.0).max_diameter(4.0).generate(&shape).is_ok());
        let err = ChamferOp::new(FRAC_PI_2, 2.0, 1.0).max_diameter(3.9).generate(&shape).unwrap_err();
        assert!(err.to_string().contains("chamfer too wide for the tool"), "{err}");
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(80))]

        #[test]
        fn outside_circle_bevel_is_tangent_to_the_cone(
            angle_deg in 30.0..150.0f64,
            tip in 0.0..3.0f64,
            width in 0.1..5.0f64,
            clearance in 0.0..1.0f64,
            radius in 20.0..50.0f64,
        ) {
            let included = angle_deg.to_radians();
            let half = included * 0.5;
            let tan_half = half.tan();
            let stock_z = 0.0;
            // Tight linear tolerance so a sub-nanometre axis shift is still cut, and anything
            // skipped as noise is within the 1e-9 tangency bound.
            let tolerance = Tolerance::new(1e-12, 1e-9).unwrap();
            let shape = circle(radius);
            let chamfer = ChamferOp::new(included, tip, width)
                .tip_clearance(clearance)
                .stock_z(stock_z)
                .side(ProfileSide::Outside)
                .tolerance(tolerance)
                .generate(&shape)
                .unwrap();

            let path_radius = circle_path_radius(chamfer.toolpath());
            let rt = tip * 0.5;
            let at_stock = rt + (stock_z - chamfer.tip_z()) * tan_half;
            let at_bevel_bottom_z = stock_z - width / tan_half;
            let at_bottom = rt + (at_bevel_bottom_z - chamfer.tip_z()) * tan_half;
            let axis_gap = path_radius - radius;
            prop_assert!((at_stock - (axis_gap + width)).abs() <= 1e-9, "stock cone {} vs {}", at_stock, axis_gap + width);
            prop_assert!((at_bottom - axis_gap).abs() <= 1e-9, "bevel-bottom cone {at_bottom} vs axis gap {axis_gap}");
        }
    }

    fn circle_path_radius(path: &Toolpath) -> f64 {
        let mut radii = Vec::new();
        let mut cursor: Option<(f64, f64)> = None;
        for mv in path.moves() {
            match *mv {
                Move::Rapid { to } | Move::Linear { to, .. } => {
                    radii.push(to.x().hypot(to.y()));
                    cursor = Some((to.x(), to.y()));
                }
                Move::Arc { to, centre, .. } => {
                    if let Some(from) = cursor {
                        radii.push((from.0 - centre.x()).hypot(from.1 - centre.y()));
                    }
                    radii.push((to.x() - centre.x()).hypot(to.y() - centre.y()));
                    radii.push(to.x().hypot(to.y()));
                    cursor = Some((to.x(), to.y()));
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        assert!(!radii.is_empty());
        let first = radii[0];
        for radius in &radii {
            assert!((radius - first).abs() <= 1e-8, "{radius} != {first}");
        }
        first
    }
}
