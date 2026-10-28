//! 2D profile: side, climb or conventional, leads, overcut, tabs and the start point.
//!
//! # Travel direction
//!
//! Closed contours use the CCW-outer convention. An outer boundary is stored counter-clockwise,
//! so the part is inside it. A hole is stored clockwise, so the void is inside it. The profile
//! then travels in a fixed sense in the plane:
//!
//! - **Outside**, or **on** the line: climb is clockwise, conventional is counter-clockwise.
//! - **Inside**: climb is counter-clockwise, conventional is clockwise.
//!
//! Climb leaves the material on the right of travel for a right-hand cutter (the cutter spins
//! clockwise when viewed from +Z). Conventional leaves the material on the left. On an outside
//! pass of a CCW outer, climb therefore reverses the stored contour and conventional follows it.
//! On an inside pass of that same contour, climb follows it and conventional reverses it.
//!
//! An open contour has no winding. Inside is to the left of its stored direction and outside is
//! to the right. Climb travels with that side on the left; conventional travels with it on the right.
//!
//! # Leads
//!
//! A line lead lies on the path tangent. A perpendicular lead lies on the normal, on the waste
//! side. An arc lead is a quarter turn (`π/2`) that meets the path with the same tangent; its
//! centre is on the waste side, one radius out. Overcut, on a closed profile, keeps cutting
//! along the contour past the start. Tabs are gaps measured by arc length. They are not placed
//! on the lead-in span or on the overcut, both measured from the start along the path.
//!
//! A tab with a positive `height` is crossed at `z + height` as feed, following the compensated
//! contour exactly (lines, arcs and corners). Climbing onto the tab and dropping off it is a
//! vertical feed. The gap is never a rapid and never a chord that leaves the contour. When
//! `height` is omitted or 0, the tab is the full depth of the material: the tool feeds up to the
//! retract height, rapids across at that height, and feeds back down.
//!
//! The program rapids to the pierce XY at the clearance height, rapids down to the retract
//! height, and feeds (plunge) to `z`. After process-off it rapids back to the clearance height.
//! Clearance defaults to 5 mm and retract to 2 mm, the same defaults as `campanion run`. The
//! retract height is above the stock top unless the caller sets a lower plane on purpose. No
//! rapid that changes XY ends below the retract height.

use campanion_geom::{Contour, Orientation, Point, Tolerance, Vec2};

use crate::curve::{self, Location};
use crate::tool::{Compensation, ControllerSide, Tool};
use crate::toolpath::{CutKind, MAX_MOVES, Move, Point3, ProcessEvent, Toolpath};
use crate::{PathError, Result};

/// How many tabs one profile will place.
const MAX_TABS: usize = 4_096;

/// Stock top for 2.5D profiles. Z is up. A mill retract height has to sit above this.
const STOCK_TOP_Z: f64 = 0.0;

/// Default clearance height, millimetres. Same value `campanion run` uses when a job omits it.
pub const DEFAULT_CLEARANCE_MM: f64 = 5.0;

/// Default retract height, millimetres. Above the stock top. Same value `campanion run` uses.
pub const DEFAULT_RETRACT_MM: f64 = 2.0;

/// Which side of the contour the tool runs on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileSide {
    /// Tool in the interior of the contour.
    Inside,
    /// Tool in the exterior of the contour.
    Outside,
    /// Tool centre on the contour.
    On,
}

/// Climb or conventional. See the module docs for which way each one travels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutDirection {
    /// Material on the right of travel.
    Climb,
    /// Material on the left of travel.
    Conventional,
}

/// Lead-in or lead-out shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Lead {
    /// No lead move.
    None,
    /// Straight line along the path tangent. `length` is millimetres.
    Line { length: f64 },
    /// Quarter-circle, tangent to the path. `radius` is millimetres.
    Arc { radius: f64 },
    /// Straight line along the normal, on the waste side. `length` is millimetres.
    Perpendicular { length: f64 },
}

/// Where a closed profile starts cutting.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StartPoint {
    /// Nearest point of the compensated contour to `point`.
    User(Point),
    /// Midpoint of the longest straight edge on a closed contour.
    ///
    /// A closed contour with no straight edge starts at its sharpest corner.
    /// An open contour keeps the endpoint chosen by climb or conventional.
    Automatic,
}

/// How many tabs, or how far apart their centres are.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TabLayout {
    /// This many tabs, spaced evenly along the contour after the lead-in region.
    Count(usize),
    /// Centre-to-centre distance in millimetres. As many tabs are placed as fit.
    Spacing(f64),
}

/// Tabs (bridges) left uncut along the profile.
///
/// A positive [`Self::height`] raises the tool by that amount and feeds along the compensated
/// contour at the raised Z. `None` or 0 is a full-depth tab: feed up to the retract height, rapid
/// across at that height, and plunge back.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TabSpec {
    layout: TabLayout,
    width: f64,
    /// Lift while crossing the gap, millimetres. `None` or 0 is a full-depth tab.
    height: Option<f64>,
}

impl TabSpec {
    pub fn count(count: usize, width: f64) -> Self {
        Self { layout: TabLayout::Count(count), width, height: None }
    }

    pub fn spacing(spacing: f64, width: f64) -> Self {
        Self { layout: TabLayout::Spacing(spacing), width, height: None }
    }

    /// Lift while crossing the gap, millimetres. Positive values feed along the contour at `z + height`.
    /// Zero is stored and treated as a full-depth tab.
    pub fn height(mut self, height: f64) -> Self {
        self.height = Some(height);
        self
    }

    pub fn layout(&self) -> TabLayout {
        self.layout
    }

    pub fn width(&self) -> f64 {
        self.width
    }

    pub fn tab_height(&self) -> Option<f64> {
        self.height
    }
}

/// One tab gap, measured along the compensated contour from the cut start.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tab {
    start: f64,
    width: f64,
    height: Option<f64>,
}

impl Tab {
    /// Arc length from the profile start to the beginning of the gap.
    pub fn start(self) -> f64 {
        self.start
    }

    pub fn width(self) -> f64 {
        self.width
    }

    pub fn height(self) -> Option<f64> {
        self.height
    }
}

/// Parameters for one profile.
#[derive(Clone, Debug, PartialEq)]
pub struct ProfileOp {
    tool: Tool,
    compensation: Compensation,
    side: ProfileSide,
    direction: CutDirection,
    allowance: f64,
    lead_in: Lead,
    lead_out: Lead,
    overcut: f64,
    start: StartPoint,
    tabs: Option<TabSpec>,
    z: f64,
    /// Rapid plane, millimetres. Defaults to [`DEFAULT_CLEARANCE_MM`].
    clearance: f64,
    /// Height of the rapid down onto the part and of a full-depth tab crossing. Defaults to
    /// [`DEFAULT_RETRACT_MM`].
    retract: f64,
    tolerance: Tolerance,
}

impl ProfileOp {
    pub fn new(tool: Tool, compensation: Compensation, side: ProfileSide, direction: CutDirection) -> Self {
        Self {
            tool,
            compensation,
            side,
            direction,
            allowance: 0.0,
            lead_in: Lead::None,
            lead_out: Lead::None,
            overcut: 0.0,
            start: StartPoint::Automatic,
            tabs: None,
            z: 0.0,
            clearance: DEFAULT_CLEARANCE_MM,
            retract: DEFAULT_RETRACT_MM,
            tolerance: Tolerance::default(),
        }
    }

    /// Clearance and retract heights, millimetres.
    ///
    /// The program rapids to the pierce XY at `clearance`, rapids down to `retract` when that is
    /// lower, and feeds to the cut. A full-depth tab crosses at `retract`. `clearance` must be
    /// finite and at least `retract`. `retract` must be finite and at least 0. A mill profile
    /// should keep `retract` above the stock top; the profile document rejects a retract that is
    /// not. Defaults are [`DEFAULT_CLEARANCE_MM`] and [`DEFAULT_RETRACT_MM`] when this is not called.
    pub fn heights(mut self, clearance: f64, retract: f64) -> Self {
        self.clearance = clearance;
        self.retract = retract;
        self
    }

    /// Rapid plane, millimetres, before validation.
    pub fn clearance(&self) -> f64 {
        self.clearance
    }

    /// Retract height, millimetres, before validation.
    pub fn retract(&self) -> f64 {
        self.retract
    }

    pub fn allowance(mut self, allowance: f64) -> Self {
        self.allowance = allowance;
        self
    }

    pub fn lead_in(mut self, lead: Lead) -> Self {
        self.lead_in = lead;
        self
    }

    pub fn lead_out(mut self, lead: Lead) -> Self {
        self.lead_out = lead;
        self
    }

    pub fn overcut(mut self, overcut: f64) -> Self {
        self.overcut = overcut;
        self
    }

    pub fn start(mut self, start: StartPoint) -> Self {
        self.start = start;
        self
    }

    pub fn tabs(mut self, tabs: TabSpec) -> Self {
        self.tabs = Some(tabs);
        self
    }

    pub fn z(mut self, z: f64) -> Self {
        self.z = z;
        self
    }

    pub fn tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    pub fn tool(&self) -> Tool {
        self.tool
    }

    pub fn compensation(&self) -> Compensation {
        self.compensation
    }

    pub fn side(&self) -> ProfileSide {
        self.side
    }

    pub fn direction(&self) -> CutDirection {
        self.direction
    }

    /// Build the profile for `contour`.
    pub fn generate(&self, contour: &Contour) -> Result<Profile> {
        if contour.segments().len() > campanion_geom::MAX_SEGMENTS {
            return Err(invalid(format!("too many segments: {} > {}", contour.segments().len(), campanion_geom::MAX_SEGMENTS)));
        }
        let _heights = approach_heights(self.clearance, self.retract)?;
        let z = finite(self.z, "z")?;
        let overcut = non_negative(self.overcut, "overcut")?;
        if overcut > 0.0 && !contour.is_closed() {
            return Err(invalid("overcut requires a closed contour"));
        }
        if matches!(self.side, ProfileSide::On) && self.allowance > 0.0 {
            let allowance = non_negative(self.allowance, "finishing allowance")?;
            if allowance > 0.0 {
                return Err(invalid("finishing allowance needs an inside or outside side"));
            }
        }
        let magnitude = self.offset_magnitude()?;
        let path = self.compensated(contour, magnitude)?;
        let path = self.orient_travel(&path)?;
        let path = self.apply_start(&path)?;
        let length = path.length()?;
        if !length.is_finite() || length <= self.tolerance.linear() {
            return Err(invalid("profile length is too small"));
        }
        if overcut >= length {
            return Err(invalid("overcut must be shorter than the profile"));
        }
        let lead_in_span = lead_span(self.lead_in)?;
        let _lead_out_span = lead_span(self.lead_out)?;
        let skip = lead_in_span.max(overcut);
        let tabs = match self.tabs {
            Some(spec) => place_tabs(&path, spec, skip)?,
            None => Vec::new(),
        };
        let controller_side = match self.compensation {
            Compensation::Computed => None,
            Compensation::Controller(side) => Some(side),
        };
        let (toolpath, pierce) = emit(self, &path, &tabs, z, overcut)?;
        Ok(Profile { toolpath, pierce, tabs, controller_side })
    }

    fn offset_magnitude(&self) -> Result<f64> {
        let allowance = non_negative(self.allowance, "finishing allowance")?;
        let distance = match self.compensation {
            Compensation::Computed => {
                let sum = self.tool.compensation_distance()? + allowance;
                if sum.is_finite() {
                    sum
                } else {
                    return Err(invalid("compensation distance is not finite"));
                }
            }
            Compensation::Controller(_) => {
                // The controller applies the radius. Only stock to leave moves the programmed path.
                let _tool = self.tool.compensation_distance()?;
                allowance
            }
        };
        if matches!(self.side, ProfileSide::On) { Ok(0.0) } else { Ok(distance) }
    }

    fn compensated(&self, contour: &Contour, magnitude: f64) -> Result<Contour> {
        if magnitude <= self.tolerance.linear() {
            return Ok(contour.clone());
        }
        let signed = signed_offset(contour, self.side, magnitude, &self.tolerance)?;
        if signed.abs() <= self.tolerance.linear() {
            return Ok(contour.clone());
        }
        let parts = campanion_geom::offset(contour, signed, &self.tolerance)?;
        if parts.is_empty() {
            return Err(invalid(match self.side {
                ProfileSide::Inside => "tool does not fit inside the contour",
                ProfileSide::Outside | ProfileSide::On => "offset produced no contour",
            }));
        }
        if parts.len() != 1 {
            return Err(invalid(format!("offset split the profile into {} contours", parts.len())));
        }
        let Some(path) = parts.into_iter().next() else {
            return Err(invalid("offset produced no contour"));
        };
        if contour.is_closed() && !path.is_closed() {
            return Err(invalid("offset of a closed contour did not stay closed"));
        }
        Ok(path)
    }

    fn orient_travel(&self, path: &Contour) -> Result<Contour> {
        if path.is_closed() {
            let want_ccw = want_ccw(self.side, self.direction);
            let is_ccw = match path.orientation(&self.tolerance)? {
                Orientation::CounterClockwise => true,
                Orientation::Clockwise => false,
                Orientation::Degenerate => return Err(invalid("degenerate profile")),
            };
            if is_ccw == want_ccw { Ok(path.clone()) } else { Ok(path.reversed()) }
        } else if should_reverse_open(self.side, self.direction) {
            Ok(path.reversed())
        } else {
            Ok(path.clone())
        }
    }

    fn apply_start(&self, path: &Contour) -> Result<Contour> {
        // An open path has to run from an endpoint. Automatic keeps that endpoint;
        // the longest-edge rule is for a closed contour, which can be rotated.
        if !path.is_closed() && matches!(self.start, StartPoint::Automatic) {
            return Ok(path.clone());
        }
        let location = match self.start {
            StartPoint::User(point) => curve::closest_location(path, point, &self.tolerance)?,
            StartPoint::Automatic => automatic_start(path, &self.tolerance)?,
        };
        if path.is_closed() {
            return curve::rotate_closed(path, &location, &self.tolerance);
        }
        let Some(start) = path.segments().first().map(|seg| seg.start()) else {
            return Err(invalid("contour is empty"));
        };
        if location.point.almost_eq(start, self.tolerance.linear().max(1e-6)) {
            Ok(path.clone())
        } else {
            Err(invalid(format!("open profile cannot start at ({}, {}); the start is an endpoint", location.point.x(), location.point.y())))
        }
    }
}

/// A finished profile.
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    toolpath: Toolpath,
    pierce: Point3,
    tabs: Vec<Tab>,
    controller_side: Option<ControllerSide>,
}

impl Profile {
    pub fn toolpath(&self) -> &Toolpath {
        &self.toolpath
    }

    /// Where the process turns on: the start of the lead-in, or the cut start when there is no lead-in.
    pub fn pierce(&self) -> Point3 {
        self.pierce
    }

    pub fn tabs(&self) -> &[Tab] {
        &self.tabs
    }

    /// Set when compensation is left to the controller.
    pub fn controller_side(&self) -> Option<ControllerSide> {
        self.controller_side
    }
}

fn want_ccw(side: ProfileSide, direction: CutDirection) -> bool {
    let outside_like = matches!(side, ProfileSide::Outside | ProfileSide::On);
    match (outside_like, direction) {
        (true, CutDirection::Conventional) => true,
        (true, CutDirection::Climb) => false,
        (false, CutDirection::Climb) => true,
        (false, CutDirection::Conventional) => false,
    }
}

fn should_reverse_open(side: ProfileSide, direction: CutDirection) -> bool {
    match (side, direction) {
        (ProfileSide::Inside, CutDirection::Climb) => false,
        (ProfileSide::Inside, CutDirection::Conventional) => true,
        (ProfileSide::Outside | ProfileSide::On, CutDirection::Climb) => true,
        (ProfileSide::Outside | ProfileSide::On, CutDirection::Conventional) => false,
    }
}

fn signed_offset(contour: &Contour, side: ProfileSide, magnitude: f64, tolerance: &Tolerance) -> Result<f64> {
    if matches!(side, ProfileSide::On) || magnitude == 0.0 {
        return Ok(0.0);
    }
    if !contour.is_closed() {
        return Ok(match side {
            ProfileSide::Inside => magnitude,
            ProfileSide::Outside => -magnitude,
            ProfileSide::On => 0.0,
        });
    }
    // Positive geometry offset is to the left of the stored direction.
    // Interior is left of a counter-clockwise contour and right of a clockwise one.
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

fn automatic_start(contour: &Contour, tolerance: &Tolerance) -> Result<Location> {
    if let Some(line) = curve::longest_line_midpoint(contour, tolerance.linear())? {
        return Ok(line);
    }
    curve::sharpest_corner(contour)?.ok_or_else(|| invalid("contour has no start point"))
}

fn place_tabs(contour: &Contour, spec: TabSpec, skip: f64) -> Result<Vec<Tab>> {
    let width = positive(spec.width, "tab width")?;
    let height = match spec.height {
        None => None,
        Some(height) => Some(non_negative(height, "tab height")?),
    };
    let total = contour.length()?;
    if !skip.is_finite() || skip < 0.0 {
        return Err(invalid("lead-in span is not finite"));
    }
    if skip >= total {
        return Err(invalid("lead-in and overcut cover the whole contour; no room for tabs"));
    }
    let usable = total - skip;
    let centers = tab_centers(spec.layout, usable, width, skip)?;
    let mut tabs = Vec::with_capacity(centers.len());
    let mut prev_end = skip;
    for center in centers {
        let start = center - width * 0.5;
        let end = center + width * 0.5;
        if start < prev_end - 1e-6 || end > total + 1e-6 {
            return Err(invalid("tab overlaps the lead-in or another tab"));
        }
        tabs.push(Tab { start, width, height });
        prev_end = end;
    }
    Ok(tabs)
}

fn tab_centers(layout: TabLayout, usable: f64, width: f64, skip: f64) -> Result<Vec<f64>> {
    match layout {
        TabLayout::Count(0) => Err(invalid("tab count must be positive")),
        TabLayout::Count(count) => {
            if count > MAX_TABS {
                return Err(invalid(format!("too many tabs: {count} > {MAX_TABS}")));
            }
            let count_f = count as f64;
            if count_f * width >= usable {
                return Err(invalid("tabs are wider than the contour"));
            }
            Ok((0..count).map(|i| skip + (i as f64 + 0.5) * (usable / count_f)).collect())
        }
        TabLayout::Spacing(spacing) => {
            let spacing = positive(spacing, "tab spacing")?;
            if spacing <= width {
                return Err(invalid("tab spacing must be greater than the tab width"));
            }
            let count_f = (usable / spacing).floor();
            if !count_f.is_finite() || count_f < 1.0 {
                return Err(invalid("tab spacing is longer than the contour"));
            }
            let count = count_f as usize;
            if count > MAX_TABS {
                return Err(invalid(format!("too many tabs: {count} > {MAX_TABS}")));
            }
            Ok((0..count).map(|i| skip + (i as f64 + 0.5) * spacing).collect())
        }
    }
}

fn emit(op: &ProfileOp, path: &Contour, tabs: &[Tab], z: f64, overcut: f64) -> Result<(Toolpath, Point3)> {
    let start = path.segments().first().map(|seg| seg.start()).ok_or_else(|| invalid("contour is empty"))?;
    let travel = curve::start_tangent(path)?;
    let waste_left = matches!(op.direction, CutDirection::Climb);
    let mut moves = Vec::new();
    let pierce_xy = lead_in_point(start, travel, waste_left, op.lead_in)?;
    let pierce = point3(pierce_xy, z)?;
    let (clearance, retract) = approach_heights(op.clearance, op.retract)?;
    // Rapid at clearance, rapid down to the retract height (never below the cut), then a plunge
    // feed to depth. The retract back to clearance follows process-off, so a post delay can sit
    // between the stop and that rapid.
    let travel_z = clearance.max(z);
    let down_z = retract.max(z).min(travel_z);
    push_rapid(&mut moves, pierce_xy, travel_z)?;
    push_rapid(&mut moves, pierce_xy, down_z)?;
    push(&mut moves, Move::Event(ProcessEvent::On))?;
    push_vertical(&mut moves, pierce_xy, z, CutKind::Plunge)?;
    emit_lead(op.lead_in, start, travel, waste_left, false, z, CutKind::LeadIn, &mut moves)?;

    let length = path.length()?;
    let mut cursor = 0.0;
    for tab in tabs {
        if tab.start > cursor + 1e-9 {
            emit_span(path, cursor, tab.start, z, CutKind::Cut, &mut moves)?;
        }
        emit_gap(path, tab, z, retract, &mut moves)?;
        cursor = tab.start + tab.width;
    }
    if length > cursor + 1e-9 {
        emit_span(path, cursor, length, z, CutKind::Cut, &mut moves)?;
    }
    if overcut > 0.0 {
        emit_span(path, 0.0, overcut, z, CutKind::Cut, &mut moves)?;
    }

    let exit_s = if path.is_closed() { overcut } else { length };
    let exit = curve::point_at_length(path, exit_s)?;
    let exit_tangent = curve::tangent_at_length(path, exit_s)?;
    emit_lead(op.lead_out, exit, exit_tangent, waste_left, true, z, CutKind::LeadOut, &mut moves)?;
    let end_xy = lead_out_point(exit, exit_tangent, waste_left, op.lead_out)?;
    push(&mut moves, Move::Event(ProcessEvent::Off))?;
    push_rapid(&mut moves, end_xy, travel_z)?;
    Ok((Toolpath::new(moves)?, pierce))
}

fn emit_span(path: &Contour, s0: f64, s1: f64, z: f64, kind: CutKind, moves: &mut Vec<Move>) -> Result<()> {
    for seg in curve::subdivide(path, s0, s1)? {
        push_segment(seg, z, kind, moves)?;
    }
    Ok(())
}

fn push_segment(seg: campanion_geom::Segment, z: f64, kind: CutKind, moves: &mut Vec<Move>) -> Result<()> {
    match seg {
        campanion_geom::Segment::Line(line) => {
            push(moves, Move::Linear { to: point3(line.end(), z)?, kind })?;
        }
        campanion_geom::Segment::Arc(arc) => {
            push(moves, Move::Arc { to: point3(arc.end_point(), z)?, centre: point3(arc.center(), z)?, ccw: arc.sweep() > 0.0, kind })?;
        }
    }
    Ok(())
}

fn emit_gap(path: &Contour, tab: &Tab, z: f64, retract: f64, moves: &mut Vec<Move>) -> Result<()> {
    let from = curve::point_at_length(path, tab.start)?;
    let to = curve::point_at_length(path, tab.start + tab.width)?;
    let end_s = tab.start + tab.width;
    if let Some(height) = tab.height.filter(|height| *height > 0.0) {
        let raised = z + height;
        push_vertical(moves, from, raised, CutKind::Retract)?;
        emit_span(path, tab.start, end_s, raised, CutKind::Retract, moves)?;
        push_vertical(moves, to, z, CutKind::Plunge)?;
    } else {
        // Full-depth tab: feed up to the retract height (already there when the cut is higher),
        // rapid across, and plunge back. The rapid stays off the stock top.
        let cross = retract.max(z);
        push_vertical(moves, from, cross, CutKind::Retract)?;
        if !same_xy(from, to) {
            push(moves, Move::Rapid { to: point3(to, cross)? })?;
        }
        push_vertical(moves, to, z, CutKind::Plunge)?;
    }
    Ok(())
}

/// Finite clearance and retract. Retract is at least the stock top; clearance is at least retract.
fn approach_heights(clearance: f64, retract: f64) -> Result<(f64, f64)> {
    let clearance = finite(clearance, "clearance")?;
    let retract = finite(retract, "retract")?;
    if retract < STOCK_TOP_Z {
        return Err(invalid("retract must be >= 0"));
    }
    if retract > clearance {
        return Err(invalid("retract is above clearance"));
    }
    Ok((clearance, retract))
}

fn same_xy(a: Point, b: Point) -> bool {
    (a.x() - b.x()).abs() <= 1e-9 && (a.y() - b.y()).abs() <= 1e-9
}

fn push_vertical(moves: &mut Vec<Move>, xy: Point, z: f64, kind: CutKind) -> Result<()> {
    let to = point3(xy, z)?;
    if last_point(moves).is_some_and(|prev| prev.almost_eq(to, 1e-9)) {
        return Ok(());
    }
    push(moves, Move::Linear { to, kind })
}

fn push_rapid(moves: &mut Vec<Move>, xy: Point, z: f64) -> Result<()> {
    let to = point3(xy, z)?;
    if last_point(moves).is_some_and(|prev| prev.almost_eq(to, 1e-9)) {
        return Ok(());
    }
    push(moves, Move::Rapid { to })
}

fn last_point(moves: &[Move]) -> Option<Point3> {
    moves.iter().rev().find_map(|mv| match *mv {
        Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
        Move::Event(_) | Move::Dwell { .. } => None,
    })
}

fn emit_lead(
    lead: Lead,
    contact: Point,
    tangent: Vec2,
    waste_left: bool,
    departing: bool,
    z: f64,
    kind: CutKind,
    moves: &mut Vec<Move>,
) -> Result<()> {
    match lead {
        Lead::None => Ok(()),
        Lead::Line { .. } | Lead::Perpendicular { .. } => {
            let end = if departing { lead_out_point(contact, tangent, waste_left, lead)? } else { contact };
            // A lead-in was already positioned by the rapid. Only the feed onto or off the path is emitted.
            if departing {
                push(moves, Move::Linear { to: point3(end, z)?, kind })?;
            } else {
                push(moves, Move::Linear { to: point3(contact, z)?, kind })?;
            }
            Ok(())
        }
        Lead::Arc { radius } => {
            let arc = tangent_arc(contact, tangent, radius, waste_left, departing)?;
            push(moves, Move::Arc { to: point3(arc.end, z)?, centre: point3(arc.centre, z)?, ccw: arc.ccw, kind })?;
            Ok(())
        }
    }
}

fn lead_in_point(contact: Point, tangent: Vec2, waste_left: bool, lead: Lead) -> Result<Point> {
    match lead {
        Lead::None => Ok(contact),
        Lead::Line { length } => {
            let length = positive(length, "lead length")?;
            Ok(contact + tangent * -length)
        }
        Lead::Perpendicular { length } => {
            let length = positive(length, "lead length")?;
            let normal = waste_normal(tangent, waste_left)?;
            Ok(contact + normal * length)
        }
        Lead::Arc { radius } => {
            let radius = positive(radius, "lead radius")?;
            Ok(tangent_arc(contact, tangent, radius, waste_left, false)?.start)
        }
    }
}

fn lead_out_point(contact: Point, tangent: Vec2, waste_left: bool, lead: Lead) -> Result<Point> {
    match lead {
        Lead::None => Ok(contact),
        Lead::Line { length } => {
            let length = positive(length, "lead length")?;
            Ok(contact + tangent * length)
        }
        Lead::Perpendicular { length } => {
            let length = positive(length, "lead length")?;
            let normal = waste_normal(tangent, waste_left)?;
            Ok(contact + normal * length)
        }
        Lead::Arc { radius } => {
            let radius = positive(radius, "lead radius")?;
            Ok(tangent_arc(contact, tangent, radius, waste_left, true)?.end)
        }
    }
}

struct ArcLead {
    start: Point,
    end: Point,
    centre: Point,
    ccw: bool,
}

/// Quarter-turn arc that is tangent to `tangent` at `contact`.
///
/// `departing` is false for a lead-in (the arc ends on the path) and true for a lead-out.
fn tangent_arc(contact: Point, tangent: Vec2, radius: f64, waste_left: bool, departing: bool) -> Result<ArcLead> {
    let radius = positive(radius, "lead radius")?;
    let normal = waste_normal(tangent, waste_left)?;
    let centre = contact + normal * radius;
    let ccw = waste_left;
    let contact_angle = (contact - centre).angle();
    let sweep = if ccw { std::f64::consts::FRAC_PI_2 } else { -std::f64::consts::FRAC_PI_2 };
    if !departing {
        let start_angle = contact_angle - sweep;
        let start = centre + Vec2::new(radius * start_angle.cos(), radius * start_angle.sin())?;
        Ok(ArcLead { start, end: contact, centre, ccw })
    } else {
        let end_angle = contact_angle + sweep;
        let end = centre + Vec2::new(radius * end_angle.cos(), radius * end_angle.sin())?;
        Ok(ArcLead { start: contact, end, centre, ccw })
    }
}

fn waste_normal(tangent: Vec2, waste_left: bool) -> Result<Vec2> {
    let left = curve::left_normal(tangent)?;
    if waste_left { Ok(left) } else { Ok(Vec2::new(-left.x(), -left.y())?) }
}

/// Length along the contour that tabs must stay clear of, for this lead.
fn lead_span(lead: Lead) -> Result<f64> {
    match lead {
        Lead::None => Ok(0.0),
        Lead::Line { length } | Lead::Perpendicular { length } => positive(length, "lead length"),
        Lead::Arc { radius } => Ok(positive(radius, "lead radius")? * std::f64::consts::FRAC_PI_2),
    }
}

fn point3(point: Point, z: f64) -> Result<Point3> {
    Point3::new(point.x(), point.y(), z)
}

fn push(moves: &mut Vec<Move>, mv: Move) -> Result<()> {
    if moves.len() >= MAX_MOVES {
        return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
    }
    moves.push(mv);
    Ok(())
}

fn finite(value: f64, what: &str) -> Result<f64> {
    if value.is_finite() { Ok(value) } else { Err(invalid(format!("{what} must be finite"))) }
}

fn non_negative(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value >= 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be >= 0"))) }
}

fn positive(value: f64, what: &str) -> Result<f64> {
    let value = finite(value, what)?;
    if value > 0.0 { Ok(value) } else { Err(invalid(format!("{what} must be positive"))) }
}

fn invalid(message: impl Into<String>) -> PathError {
    PathError::InvalidInput(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolpath::CutKind;
    use campanion_geom::{Line, Segment};
    use std::f64::consts::PI;

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

    fn rect_cw(w: f64, h: f64) -> Contour {
        rect(w, h).reversed()
    }

    fn tool2() -> Tool {
        Tool::Diameter { diameter: 2.0 }
    }

    fn outside_conventional() -> ProfileOp {
        ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Outside, CutDirection::Conventional)
    }

    fn inside_climb() -> ProfileOp {
        ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Inside, CutDirection::Climb)
    }

    /// Backend round-join epsilon. Circles are tighter; rectangle corners follow the geometry crate.
    const LOOSE: f64 = 1e-4;

    fn near(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} != {b} (tol {tol})");
    }

    fn cut_bounds(profile: &Profile) -> (f64, f64, f64, f64) {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut consider = |x: f64, y: f64| {
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
        };
        consider(profile.pierce().x(), profile.pierce().y());
        for mv in profile.toolpath().moves() {
            match *mv {
                Move::Linear { to, kind: CutKind::Cut } | Move::Arc { to, kind: CutKind::Cut, .. } => consider(to.x(), to.y()),
                _ => {}
            }
        }
        (min_x, min_y, max_x, max_y)
    }

    #[test]
    fn outside_rectangle_length_is_perimeter_plus_a_full_corner_circle() {
        // 20 by 10, tool diameter 2 mm, radius 1. Round outside corners:
        // straights stay 2*(20+10), four quarter-circles sum to 2*π*1.
        let profile = outside_conventional().start(StartPoint::User(Point::new(10.0, -50.0).unwrap())).generate(&rect(20.0, 10.0)).unwrap();
        let expected = 2.0 * (20.0 + 10.0) + 2.0 * PI * 1.0;
        near(profile.toolpath().cutting_length().unwrap(), expected, LOOSE);
        let (min_x, min_y, max_x, max_y) = cut_bounds(&profile);
        near(min_x, -1.0, LOOSE);
        near(min_y, -1.0, LOOSE);
        near(max_x, 21.0, LOOSE);
        near(max_y, 11.0, LOOSE);
        near(profile.pierce().x(), 10.0, LOOSE);
        near(profile.pierce().y(), -1.0, LOOSE);
        assert!(profile.controller_side().is_none());
        let arcs = profile.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Arc { kind: CutKind::Cut, .. })).count();
        assert_eq!(arcs, 4, "one round corner per vertex");
    }

    #[test]
    fn inside_rectangle_is_18_by_8() {
        // Inward by the 1 mm radius. Sharp corners, so the path is the rectangle (1,1)-(19,9).
        let profile = inside_climb().start(StartPoint::User(Point::new(10.0, -5.0).unwrap())).generate(&rect(20.0, 10.0)).unwrap();
        near(profile.toolpath().cutting_length().unwrap(), 2.0 * (18.0 + 8.0), LOOSE);
        let (min_x, min_y, max_x, max_y) = cut_bounds(&profile);
        near(max_x - min_x, 18.0, LOOSE);
        near(max_y - min_y, 8.0, LOOSE);
        near(min_x, 1.0, LOOSE);
        near(min_y, 1.0, LOOSE);
        near(max_x, 19.0, LOOSE);
        near(max_y, 9.0, LOOSE);
        // Inside + climb follows the contour: counter-clockwise. From the bottom midpoint that is +X.
        let first = first_cut(profile.toolpath());
        match first {
            Move::Linear { to, .. } => assert!(to.x() > profile.pierce().x() + 1.0, "climb inside should leave toward +X, got {}", to.x()),
            other => panic!("expected a line, got {other:?}"),
        }
    }

    #[test]
    fn clockwise_hole_inside_is_the_same_18_by_8() {
        let profile = inside_climb().start(StartPoint::User(Point::new(10.0, -5.0).unwrap())).generate(&rect_cw(20.0, 10.0)).unwrap();
        near(profile.toolpath().cutting_length().unwrap(), 2.0 * (18.0 + 8.0), LOOSE);
        let (min_x, min_y, max_x, max_y) = cut_bounds(&profile);
        near(max_x - min_x, 18.0, LOOSE);
        near(max_y - min_y, 8.0, LOOSE);
    }

    #[test]
    fn kerf_uses_half_width() {
        // Kerf 1.5 mm. Half-kerf is 0.75, so the outside path sits 0.75 off the rectangle,
        // not 1.5. Length = 2*(20+10) + 2*π*0.75.
        let op = ProfileOp::new(Tool::Kerf { width: 1.5 }, Compensation::Computed, ProfileSide::Outside, CutDirection::Conventional)
            .start(StartPoint::User(Point::new(10.0, -20.0).unwrap()));
        let profile = op.generate(&rect(20.0, 10.0)).unwrap();
        let expected = 2.0 * (20.0 + 10.0) + 2.0 * PI * 0.75;
        near(profile.toolpath().cutting_length().unwrap(), expected, LOOSE);
        near(profile.pierce().x(), 10.0, LOOSE);
        near(profile.pierce().y(), -0.75, LOOSE);
    }

    #[test]
    fn finishing_allowance_adds_to_the_radius() {
        // Radius 1 + allowance 0.5 = 1.5. Length = 2*(20+10) + 2*π*1.5.
        let profile =
            outside_conventional().allowance(0.5).start(StartPoint::User(Point::new(10.0, -20.0).unwrap())).generate(&rect(20.0, 10.0)).unwrap();
        let expected = 2.0 * (20.0 + 10.0) + 2.0 * PI * 1.5;
        near(profile.toolpath().cutting_length().unwrap(), expected, 2e-4);
        near(profile.pierce().y(), -1.5, 2e-4);
    }

    #[test]
    fn arc_lead_in_is_tangent_and_a_quarter_circle() {
        // Conventional outside, start at the bottom midpoint (10, -1), tangent +X.
        // Waste is on the right. Radius 2. Centre (10, -3), arc starts at (8, -3), clockwise.
        let profile = outside_conventional()
            .start(StartPoint::User(Point::new(10.0, -40.0).unwrap()))
            .lead_in(Lead::Arc { radius: 2.0 })
            .generate(&rect(20.0, 10.0))
            .unwrap();
        near(profile.pierce().x(), 8.0, LOOSE);
        near(profile.pierce().y(), -3.0, LOOSE);
        let (lead, lead_end) = first_arc(&profile);
        match lead {
            Move::Arc { to, centre, ccw, kind } => {
                assert_eq!(*kind, CutKind::LeadIn);
                assert!(!ccw, "conventional waste is on the right, so the lead-in is clockwise");
                near(to.x(), 10.0, LOOSE);
                near(to.y(), -1.0, LOOSE);
                near(centre.x(), 10.0, LOOSE);
                near(centre.y(), -3.0, LOOSE);
                near(profile.pierce().distance(*to), 2.0 * std::f64::consts::SQRT_2, LOOSE);
                near(profile.pierce().distance(*centre), 2.0, LOOSE);
                near(to.distance(*centre), 2.0, LOOSE);
            }
            other => panic!("expected an arc lead-in, got {other:?}"),
        }
        let cut_tangent = first_cut_tangent(&profile);
        let lead_tangent = arc_end_tangent(lead_end);
        near(lead_tangent.dot(cut_tangent), 1.0, 1e-5);
        near(lead_tangent.cross(cut_tangent), 0.0, 1e-5);

        // Climb reverses travel. The lead-in is still tangent; the dot product stays 1.
        let climb = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Outside, CutDirection::Climb)
            .start(StartPoint::User(Point::new(10.0, -40.0).unwrap()))
            .lead_in(Lead::Arc { radius: 2.0 })
            .generate(&rect(20.0, 10.0))
            .unwrap();
        let (lead, _) = first_arc(&climb);
        let Move::Arc { to, centre, ccw, kind } = lead else {
            panic!("expected an arc, got {lead:?}");
        };
        assert_eq!(*kind, CutKind::LeadIn);
        assert!(*ccw, "climb keeps the waste on the left");
        let lead_tangent = arc_end_tangent(ArcEnd { to: *to, centre: *centre, ccw: *ccw });
        let cut_tangent = first_cut_tangent(&climb);
        near(lead_tangent.dot(cut_tangent), 1.0, 1e-5);
        near(lead_tangent.cross(cut_tangent), 0.0, 1e-5);
    }

    #[test]
    fn line_and_perpendicular_leads_match_the_hand_calc() {
        // Bottom midpoint (10, -1), travel +X. Line lead of 4 starts at (6, -1).
        let line = outside_conventional()
            .start(StartPoint::User(Point::new(10.0, -40.0).unwrap()))
            .lead_in(Lead::Line { length: 4.0 })
            .lead_out(Lead::Line { length: 3.0 })
            .generate(&rect(20.0, 10.0))
            .unwrap();
        near(line.pierce().x(), 6.0, LOOSE);
        near(line.pierce().y(), -1.0, LOOSE);
        // Overcut is 0, so the lead-out leaves the same point along +X: (13, -1).
        let lead_out = last_feed(&line);
        match lead_out {
            Move::Linear { to, kind } => {
                assert_eq!(*kind, CutKind::LeadOut);
                near(to.x(), 13.0, LOOSE);
                near(to.y(), -1.0, LOOSE);
            }
            other => panic!("expected a line lead-out, got {other:?}"),
        }

        // Perpendicular lead comes from the waste side, -Y, length 3: (10, -4).
        let perp = outside_conventional()
            .start(StartPoint::User(Point::new(10.0, -40.0).unwrap()))
            .lead_in(Lead::Perpendicular { length: 3.0 })
            .generate(&rect(20.0, 10.0))
            .unwrap();
        near(perp.pierce().x(), 10.0, LOOSE);
        near(perp.pierce().y(), -4.0, LOOSE);
    }

    #[test]
    fn tab_gaps_sum_to_count_times_width_and_shorten_the_cut() {
        let shape = rect(20.0, 10.0);
        let plain = inside_climb().generate(&shape).unwrap();
        let with = inside_climb().tabs(TabSpec::count(4, 2.0)).generate(&shape).unwrap();
        assert_eq!(with.tabs().len(), 4);
        let sum: f64 = with.tabs().iter().map(|tab| tab.width()).sum();
        near(sum, 8.0, 1e-12);
        near(plain.toolpath().cutting_length().unwrap() - with.toolpath().cutting_length().unwrap(), sum, 1e-6);
        let rapids = with.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Rapid { .. })).count();
        assert_eq!(rapids, 7, "clearance, retract, one rapid per full-depth tab, and the final retract");

        let lifted = inside_climb().tabs(TabSpec::count(4, 2.0).height(1.5)).generate(&shape).unwrap();
        let mut lifts = 0;
        let mut cursor: Option<Point3> = None;
        for mv in lifted.toolpath().moves() {
            if let Move::Linear { to, kind: CutKind::Retract } = mv
                && let Some(from) = cursor
                && (from.x() - to.x()).abs() <= 1e-12
                && (from.y() - to.y()).abs() <= 1e-12
            {
                lifts += 1;
                near(to.z(), 1.5, 1e-12);
            }
            if let Some(to) = move_end(*mv) {
                cursor = Some(to);
            }
        }
        assert_eq!(lifts, 4);
        near(plain.toolpath().cutting_length().unwrap() - lifted.toolpath().cutting_length().unwrap(), 8.0, 1e-6);
        let lifted_rapids = lifted.toolpath().moves().iter().filter(|mv| matches!(mv, Move::Rapid { .. })).count();
        assert_eq!(lifted_rapids, 3, "clearance, retract, and the final retract; a raised tab is feed");
    }

    #[test]
    fn tabs_skip_the_lead_in_and_spacing_fits_an_integer_count() {
        // Inside 20x10 with radius 1 is 52 mm. A line lead of 10 mm is the keep-out.
        let profile = inside_climb().lead_in(Lead::Line { length: 10.0 }).tabs(TabSpec::count(4, 1.0)).generate(&rect(20.0, 10.0)).unwrap();
        assert!(profile.tabs().iter().all(|tab| tab.start() >= 10.0 - 1e-9), "{:?}", profile.tabs());
        let sum: f64 = profile.tabs().iter().map(|tab| tab.width()).sum();
        near(sum, 4.0, 1e-12);

        // Usable length is 42. Spacing 10 yields floor(42/10) = 4 tabs, each of width 2. Sum 8.
        let spaced = inside_climb().lead_in(Lead::Line { length: 10.0 }).tabs(TabSpec::spacing(10.0, 2.0)).generate(&rect(20.0, 10.0)).unwrap();
        assert_eq!(spaced.tabs().len(), 4);
        near(spaced.tabs().iter().map(|tab| tab.width()).sum::<f64>(), 8.0, 1e-12);
        assert!(spaced.tabs().iter().all(|tab| tab.start() >= 10.0 - 1e-9));
    }

    #[test]
    fn overcut_adds_its_length_and_conventional_outside_is_counter_clockwise() {
        let shape = rect(20.0, 10.0);
        let plain = outside_conventional().start(StartPoint::User(Point::new(10.0, -40.0).unwrap())).generate(&shape).unwrap();
        let extra = outside_conventional().start(StartPoint::User(Point::new(10.0, -40.0).unwrap())).overcut(2.5).generate(&shape).unwrap();
        near(extra.toolpath().cutting_length().unwrap() - plain.toolpath().cutting_length().unwrap(), 2.5, 1e-6);
        // Conventional outside of a CCW outer travels counter-clockwise: from (10, -1) toward +X.
        match first_cut(plain.toolpath()) {
            Move::Linear { to, .. } => assert!(to.x() > plain.pierce().x() + 1.0, "{}", to.x()),
            other => panic!("expected a line, got {other:?}"),
        }
        let climb = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Outside, CutDirection::Climb)
            .start(StartPoint::User(Point::new(10.0, -40.0).unwrap()))
            .generate(&shape)
            .unwrap();
        match first_cut(climb.toolpath()) {
            Move::Linear { to, .. } => assert!(to.x() < climb.pierce().x() - 1.0, "climb outside is clockwise, got {}", to.x()),
            other => panic!("expected a line, got {other:?}"),
        }
    }

    #[test]
    fn controller_compensation_stays_on_the_contour_and_keeps_the_flag() {
        let op = ProfileOp::new(tool2(), Compensation::Controller(ControllerSide::Left), ProfileSide::Outside, CutDirection::Conventional)
            .start(StartPoint::User(Point::new(10.0, -5.0).unwrap()));
        let profile = op.generate(&rect(20.0, 10.0)).unwrap();
        near(profile.toolpath().cutting_length().unwrap(), 2.0 * (20.0 + 10.0), 1e-9);
        assert_eq!(profile.controller_side(), Some(ControllerSide::Left));
        near(profile.pierce().x(), 10.0, 1e-9);
        near(profile.pierce().y(), 0.0, 1e-9);
        assert!(profile.toolpath().moves().iter().all(|mv| !matches!(mv, Move::Arc { .. })));
    }

    #[test]
    fn automatic_start_is_the_long_side_midpoint_or_a_circle_vertex() {
        let profile = outside_conventional().generate(&rect(20.0, 10.0)).unwrap();
        let on_long_side = (profile.pierce().x() - 10.0).abs() < LOOSE
            && ((profile.pierce().y() + 1.0).abs() < LOOSE || (profile.pierce().y() - 11.0).abs() < LOOSE);
        assert!(on_long_side, "pierce {:?}", profile.pierce());

        // Two semicircles, no straight edge. Automatic start is a join (the sharpest corner; both turns match).
        let r = 5.0;
        let east = Point::new(r, 0.0).unwrap();
        let west = Point::new(-r, 0.0).unwrap();
        let circle = Contour::try_new(
            vec![
                Segment::Arc(campanion_geom::Arc::from_bulge(east, west, 1.0).unwrap()),
                Segment::Arc(campanion_geom::Arc::from_bulge(west, east, 1.0).unwrap()),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap();
        let on_line = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::On, CutDirection::Conventional).generate(&circle).unwrap();
        let dx = on_line.pierce().x() - r;
        let dy = on_line.pierce().y();
        assert!(dx.hypot(dy) < 1e-6 || (on_line.pierce().x() + r).hypot(on_line.pierce().y()) < 1e-6, "{:?}", on_line.pierce());

        // User point just off the circle. Nearest point is the radial projection.
        let user = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::On, CutDirection::Conventional)
            .start(StartPoint::User(Point::new(100.0, 1.0).unwrap()))
            .generate(&circle)
            .unwrap();
        near(user.pierce().x().hypot(user.pierce().y()), r, 1e-6);
        near(user.pierce().y().atan2(user.pierce().x()), 1.0_f64.atan2(100.0), 1e-6);
    }

    #[test]
    fn open_line_offsets_by_the_radius_and_climb_reverses_it() {
        let line = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(10.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let conventional = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Outside, CutDirection::Conventional).generate(&line).unwrap();
        near(conventional.toolpath().cutting_length().unwrap(), 10.0, LOOSE);
        near(conventional.pierce().y(), -1.0, LOOSE);
        near(conventional.pierce().x(), 0.0, LOOSE);
        let climb = ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::Outside, CutDirection::Climb).generate(&line).unwrap();
        near(climb.pierce().x(), 10.0, LOOSE);
        near(climb.pierce().y(), -1.0, LOOSE);
        near(climb.toolpath().cutting_length().unwrap(), 10.0, LOOSE);
    }

    #[test]
    fn process_markers_bracket_the_cut_and_z_is_carried() {
        let profile = inside_climb().z(-2.5).generate(&rect(20.0, 10.0)).unwrap();
        let moves = profile.toolpath().moves();
        let Some(Move::Rapid { to: at_clearance }) = moves.first() else {
            panic!("first motion should be a rapid at clearance, got {:?}", moves.first());
        };
        near(at_clearance.z(), DEFAULT_CLEARANCE_MM, 1e-12);
        let Some(Move::Rapid { to: at_retract }) = moves.get(1) else {
            panic!("second motion should be a rapid at the retract height, got {:?}", moves.get(1));
        };
        near(at_retract.z(), DEFAULT_RETRACT_MM, 1e-12);
        assert!((at_clearance.x() - at_retract.x()).abs() < 1e-12 && (at_clearance.y() - at_retract.y()).abs() < 1e-12);
        assert!(matches!(moves.get(2), Some(Move::Event(ProcessEvent::On))));
        assert!(matches!(moves.get(3), Some(Move::Linear { kind: CutKind::Plunge, to }) if (to.z() + 2.5).abs() < 1e-12));
        assert!(matches!(moves.last(), Some(Move::Rapid { to }) if (to.z() - DEFAULT_CLEARANCE_MM).abs() < 1e-12));
        let off = moves.iter().rposition(|mv| matches!(mv, Move::Event(ProcessEvent::Off))).expect("process off");
        assert_eq!(off + 1, moves.len() - 1, "the clearance rapid follows process-off");
        near(profile.pierce().z(), -2.5, 1e-12);
        for mv in moves {
            if let Move::Linear { to, kind: CutKind::Cut } | Move::Arc { to, kind: CutKind::Cut, .. } = mv {
                near(to.z(), -2.5, 1e-12);
            }
        }
        assert!(!xy_rapid_below(moves, DEFAULT_RETRACT_MM));
    }

    #[test]
    fn hostile_input_is_an_error() {
        let shape = rect(20.0, 10.0);
        let cases = [
            outside_conventional().allowance(f64::NAN),
            outside_conventional().allowance(-0.2),
            outside_conventional().allowance(f64::INFINITY),
            outside_conventional().z(f64::NAN),
            outside_conventional().overcut(-1.0),
            outside_conventional().overcut(f64::INFINITY),
            outside_conventional().overcut(1.0e9),
            outside_conventional().lead_in(Lead::Line { length: 0.0 }),
            outside_conventional().lead_in(Lead::Line { length: -2.0 }),
            outside_conventional().lead_in(Lead::Arc { radius: f64::NAN }),
            outside_conventional().lead_in(Lead::Perpendicular { length: f64::NEG_INFINITY }),
            outside_conventional().lead_out(Lead::Arc { radius: 0.0 }),
            ProfileOp::new(Tool::Diameter { diameter: f64::NAN }, Compensation::Computed, ProfileSide::Outside, CutDirection::Climb),
            ProfileOp::new(Tool::Diameter { diameter: 0.0 }, Compensation::Computed, ProfileSide::Outside, CutDirection::Climb),
            ProfileOp::new(Tool::Diameter { diameter: -4.0 }, Compensation::Computed, ProfileSide::Inside, CutDirection::Conventional),
            ProfileOp::new(Tool::Kerf { width: f64::NAN }, Compensation::Computed, ProfileSide::Outside, CutDirection::Conventional),
            ProfileOp::new(Tool::Kerf { width: -1.5 }, Compensation::Controller(ControllerSide::Right), ProfileSide::Outside, CutDirection::Climb),
            inside_climb().tabs(TabSpec::count(0, 1.0)),
            inside_climb().tabs(TabSpec::count(usize::MAX, 1.0)),
            inside_climb().tabs(TabSpec::count(4, 0.0)),
            inside_climb().tabs(TabSpec::count(4, -2.0)),
            inside_climb().tabs(TabSpec::count(4, f64::NAN)),
            inside_climb().tabs(TabSpec::spacing(f64::NAN, 1.0)),
            inside_climb().tabs(TabSpec::spacing(0.5, 1.0)),
            inside_climb().tabs(TabSpec::spacing(1.0e9, 1.0)),
            inside_climb().tabs(TabSpec::count(4, 1.0).height(-1.0)),
            inside_climb().tabs(TabSpec::count(100, 1.0)),
            inside_climb().heights(f64::NAN, DEFAULT_RETRACT_MM),
            inside_climb().heights(f64::INFINITY, DEFAULT_RETRACT_MM),
            inside_climb().heights(DEFAULT_CLEARANCE_MM, f64::NAN),
            inside_climb().heights(DEFAULT_CLEARANCE_MM, -0.1),
            inside_climb().heights(1.0, 2.0),
            ProfileOp::new(Tool::Diameter { diameter: 100.0 }, Compensation::Computed, ProfileSide::Inside, CutDirection::Climb),
            ProfileOp::new(tool2(), Compensation::Computed, ProfileSide::On, CutDirection::Conventional).allowance(0.4),
        ];
        for op in cases {
            let err = op.generate(&shape).expect_err("hostile input must fail");
            assert!(!err.to_string().is_empty(), "{err:?}");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
        }

        let open = Contour::try_new(
            vec![Segment::Line(Line::new(Point::new(0.0, 0.0).unwrap(), Point::new(5.0, 0.0).unwrap()).unwrap())],
            false,
            &Tolerance::default(),
        )
        .unwrap();
        let err = outside_conventional().overcut(1.0).generate(&open).unwrap_err();
        assert!(err.to_string().contains("overcut"), "{err}");
        let err = outside_conventional().start(StartPoint::User(Point::new(2.5, 0.0).unwrap())).generate(&open).unwrap_err();
        assert!(err.to_string().contains("open profile"), "{err}");

        let point = Point::new(1.0, 1.0).unwrap();
        let degenerate = Contour::try_new(vec![Segment::Line(Line::new(point, point).unwrap())], true, &Tolerance::default()).unwrap();
        assert!(outside_conventional().generate(&degenerate).is_err());

        let mut segments = Vec::new();
        for i in 0..campanion_geom::MAX_SEGMENTS + 1 {
            let x0 = i as f64;
            segments.push(Segment::Line(Line::new(Point::new(x0, 0.0).unwrap(), Point::new(x0 + 1.0, 0.0).unwrap()).unwrap()));
        }
        let long = Contour::try_new(segments, false, &Tolerance::default()).unwrap();
        let err = outside_conventional().generate(&long).unwrap_err();
        assert!(err.to_string().contains("too many segments"), "{err}");
    }

    #[test]
    fn notched_profile_tabs_follow_the_contour_and_start_at_clearance() {
        let part = notched_plate();
        let diameter = 6.0;
        let radius = diameter * 0.5;
        let z = -12.0;
        let height = 3.0;
        let raised = z + height;
        let profile = ProfileOp::new(Tool::Diameter { diameter }, Compensation::Computed, ProfileSide::Outside, CutDirection::Climb)
            .z(z)
            .tabs(TabSpec::count(6, 10.0).height(height))
            .generate(&part)
            .unwrap();
        let moves = profile.toolpath().moves();
        match moves.first() {
            Some(Move::Rapid { to }) => near(to.z(), DEFAULT_CLEARANCE_MM, 1e-12),
            other => panic!("first motion should be a rapid at clearance, got {other:?}"),
        }
        assert!(!xy_rapid_below(moves, DEFAULT_RETRACT_MM), "XY rapid below the retract height: {moves:?}");

        let mut tab_feeds = 0;
        let mut returns = 0;
        let mut cursor: Option<Point3> = None;
        for mv in moves {
            match *mv {
                Move::Linear { to, kind: CutKind::Retract } | Move::Arc { to, kind: CutKind::Retract, .. } if (to.z() - raised).abs() < 1e-9 => {
                    let moved_xy = cursor.is_some_and(|from| (from.x() - to.x()).abs() > 1e-9 || (from.y() - to.y()).abs() > 1e-9);
                    if moved_xy || matches!(mv, Move::Arc { .. }) {
                        tab_feeds += 1;
                    }
                }
                Move::Linear { to, kind: CutKind::Plunge } if (to.z() - z).abs() < 1e-9 => returns += 1,
                _ => {}
            }
            if let Some(to) = move_end(*mv) {
                cursor = Some(to);
            }
        }
        assert!(tab_feeds >= 6, "tab crossings as feed, got {tab_feeds}");
        assert!(returns >= 6, "plunge back to cut depth after each tab, got {returns}");

        for (x, y, _z) in sample_path(moves) {
            let at = Point::new(x, y).unwrap();
            let distance = outline_distance(&part, at);
            assert!(distance + 1e-6 >= radius, "gouge at ({x}, {y}): distance {distance} < radius {radius}");
        }
        let corner = Point::new(65.0, 0.0).unwrap();
        let min_corner = sample_path(moves).into_iter().map(|(x, y, _)| Point::new(x, y).unwrap().distance(corner)).fold(f64::INFINITY, f64::min);
        assert!(min_corner + 1e-6 >= radius, "notch corner gouge: closest approach {min_corner}");
    }

    #[test]
    fn full_depth_tab_crosses_at_the_retract_height() {
        for spec in [TabSpec::count(2, 3.0), TabSpec::count(2, 3.0).height(0.0)] {
            let profile = inside_climb().z(-8.0).tabs(spec).generate(&rect(40.0, 20.0)).unwrap();
            let moves = profile.toolpath().moves();
            assert!(!xy_rapid_below(moves, DEFAULT_RETRACT_MM), "{moves:?}");
            assert!(full_depth_crossing(moves, DEFAULT_RETRACT_MM, -8.0), "feed up, rapid across, feed down: {moves:?}");
            let mut at_depth = 0;
            for mv in moves {
                if let Move::Linear { to, kind: CutKind::Plunge } = mv
                    && (to.z() + 8.0).abs() < 1e-9
                {
                    at_depth += 1;
                }
            }
            assert!(at_depth >= 3, "entry plunge plus a plunge after each tab, got {at_depth}");
        }
    }

    #[test]
    fn custom_heights_are_the_approach_and_the_tab_plane() {
        let profile = inside_climb().z(-4.0).heights(9.0, 3.5).tabs(TabSpec::count(2, 3.0)).generate(&rect(20.0, 10.0)).unwrap();
        let moves = profile.toolpath().moves();
        match (moves.first(), moves.get(1)) {
            (Some(Move::Rapid { to: clearance }), Some(Move::Rapid { to: retract })) => {
                near(clearance.z(), 9.0, 1e-12);
                near(retract.z(), 3.5, 1e-12);
                assert!((clearance.x() - retract.x()).abs() < 1e-9 && (clearance.y() - retract.y()).abs() < 1e-9);
            }
            other => panic!("expected clearance then retract, got {other:?}"),
        }
        assert!(matches!(moves.get(3), Some(Move::Linear { kind: CutKind::Plunge, to }) if (to.z() + 4.0).abs() < 1e-9));
        assert!(matches!(moves.last(), Some(Move::Rapid { to }) if (to.z() - 9.0).abs() < 1e-9));
        assert!(full_depth_crossing(moves, 3.5, -4.0), "{moves:?}");
        assert!(!xy_rapid_below(moves, 3.5));

        let level = inside_climb().z(-4.0).heights(4.0, 4.0).generate(&rect(20.0, 10.0)).unwrap();
        let level_moves = level.toolpath().moves();
        assert!(matches!(level_moves.first(), Some(Move::Rapid { to }) if (to.z() - 4.0).abs() < 1e-12));
        assert!(matches!(level_moves.get(1), Some(Move::Event(ProcessEvent::On))));
        assert!(matches!(level_moves.get(2), Some(Move::Linear { kind: CutKind::Plunge, to }) if (to.z() + 4.0).abs() < 1e-12));
    }

    #[test]
    fn tab_across_an_arc_stays_on_the_arc() {
        let r = 20.0;
        let east = Point::new(r, 0.0).unwrap();
        let west = Point::new(-r, 0.0).unwrap();
        let circle = Contour::try_new(
            vec![
                Segment::Arc(campanion_geom::Arc::from_bulge(east, west, 1.0).unwrap()),
                Segment::Arc(campanion_geom::Arc::from_bulge(west, east, 1.0).unwrap()),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap();
        let z = -6.0;
        let height = 2.0;
        let raised = z + height;
        let profile = ProfileOp::new(Tool::Diameter { diameter: 4.0 }, Compensation::Computed, ProfileSide::On, CutDirection::Conventional)
            .z(z)
            .tabs(TabSpec::count(3, 8.0).height(height))
            .generate(&circle)
            .unwrap();
        let moves = profile.toolpath().moves();
        assert!(!xy_rapid_below(moves, DEFAULT_RETRACT_MM), "{moves:?}");
        let mut saw_arc_tab = false;
        let mut cursor: Option<Point3> = None;
        for mv in moves {
            if let Move::Arc { to, centre, kind: CutKind::Retract, .. } = *mv
                && (to.z() - raised).abs() < 1e-9
            {
                saw_arc_tab = true;
                if let Some(from) = cursor {
                    let r0 = (from.x() - centre.x()).hypot(from.y() - centre.y());
                    let r1 = (to.x() - centre.x()).hypot(to.y() - centre.y());
                    near(r0, r, 1e-6);
                    near(r1, r, 1e-6);
                }
            }
            if let Some(to) = move_end(*mv) {
                cursor = Some(to);
            }
        }
        assert!(saw_arc_tab, "tab across an arc must emit an arc feed, got {moves:?}");
        for (x, y, path_z) in sample_path(moves) {
            if (path_z - raised).abs() < 1e-6 || (path_z - z).abs() < 1e-6 {
                near(x.hypot(y), r, 1e-4);
            }
        }
    }

    #[test]
    fn self_intersecting_contour_does_not_panic() {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        let bowtie = Contour::try_new(
            vec![
                edge(p(0.0, 0.0), p(10.0, 10.0)),
                edge(p(10.0, 10.0), p(0.0, 10.0)),
                edge(p(0.0, 10.0), p(10.0, 0.0)),
                edge(p(10.0, 0.0), p(0.0, 0.0)),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap();
        match outside_conventional().generate(&bowtie) {
            Ok(profile) => {
                assert!(profile.toolpath().cutting_length().is_ok());
            }
            Err(err) => assert!(!err.to_string().is_empty()),
        }
    }

    fn first_cut(path: &Toolpath) -> &Move {
        path.moves().iter().find(|mv| matches!(mv, Move::Linear { kind: CutKind::Cut, .. } | Move::Arc { kind: CutKind::Cut, .. })).expect("cut")
    }

    struct ArcEnd {
        to: Point3,
        centre: Point3,
        ccw: bool,
    }

    fn first_arc(profile: &Profile) -> (&Move, ArcEnd) {
        for mv in profile.toolpath().moves() {
            if let Move::Arc { to, centre, ccw, kind: CutKind::LeadIn } = mv {
                return (mv, ArcEnd { to: *to, centre: *centre, ccw: *ccw });
            }
        }
        panic!("no lead-in arc");
    }

    fn first_cut_tangent(profile: &Profile) -> Vec2 {
        let mut cursor = None;
        for mv in profile.toolpath().moves() {
            match mv {
                Move::Rapid { to } => cursor = Some(*to),
                Move::Linear { to, kind } => {
                    if *kind == CutKind::Cut {
                        let from = cursor.expect("start");
                        return unit2(to.x() - from.x(), to.y() - from.y());
                    }
                    cursor = Some(*to);
                }
                Move::Arc { to, centre, ccw, kind } => {
                    if *kind == CutKind::Cut {
                        let from = cursor.expect("start");
                        return arc_start_tangent(from, *centre, *ccw);
                    }
                    cursor = Some(*to);
                }
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        panic!("no cut");
    }

    /// End tangent of a circular move. CCW tangent is the radius rotated 90° counter-clockwise.
    fn arc_end_tangent(arc: ArcEnd) -> Vec2 {
        let rx = arc.to.x() - arc.centre.x();
        let ry = arc.to.y() - arc.centre.y();
        if arc.ccw { unit2(-ry, rx) } else { unit2(ry, -rx) }
    }

    fn arc_start_tangent(start: Point3, centre: Point3, ccw: bool) -> Vec2 {
        let rx = start.x() - centre.x();
        let ry = start.y() - centre.y();
        if ccw { unit2(-ry, rx) } else { unit2(ry, -rx) }
    }

    fn unit2(x: f64, y: f64) -> Vec2 {
        let len = x.hypot(y);
        Vec2::new(x / len, y / len).unwrap()
    }

    fn last_feed(profile: &Profile) -> &Move {
        profile.toolpath().moves().iter().rev().find(|mv| matches!(mv, Move::Linear { .. } | Move::Arc { .. })).expect("feed")
    }

    fn move_end(mv: Move) -> Option<Point3> {
        match mv {
            Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
            Move::Event(_) | Move::Dwell { .. } => None,
        }
    }

    fn xy_rapid_below(moves: &[Move], floor: f64) -> bool {
        let mut cursor: Option<Point3> = None;
        for mv in moves {
            if let Move::Rapid { to } = *mv
                && let Some(from) = cursor
            {
                let xy = (from.x() - to.x()).abs() > 1e-9 || (from.y() - to.y()).abs() > 1e-9;
                if xy && from.z().min(to.z()) < floor - 1e-12 {
                    return true;
                }
            }
            if let Some(to) = move_end(*mv) {
                cursor = Some(to);
            }
        }
        false
    }

    /// Feed up to `retract`, rapid across at that height, feed back to `depth`.
    fn full_depth_crossing(moves: &[Move], retract: f64, depth: f64) -> bool {
        let mut cursor: Option<Point3> = None;
        let mut saw_up = false;
        let mut across: Option<Point3> = None;
        for mv in moves {
            match *mv {
                Move::Linear { to, kind: CutKind::Retract } if (to.z() - retract).abs() < 1e-9 => {
                    let vertical = cursor.is_some_and(|from| (from.x() - to.x()).abs() <= 1e-9 && (from.y() - to.y()).abs() <= 1e-9);
                    if vertical {
                        saw_up = true;
                        across = None;
                    }
                }
                Move::Rapid { to } if saw_up && (to.z() - retract).abs() < 1e-9 => {
                    let moved = cursor.is_some_and(|from| (from.x() - to.x()).abs() > 1e-9 || (from.y() - to.y()).abs() > 1e-9);
                    if moved {
                        across = Some(to);
                    }
                }
                Move::Linear { to, kind: CutKind::Plunge }
                    if across.is_some_and(|from| (from.x() - to.x()).abs() <= 1e-9 && (from.y() - to.y()).abs() <= 1e-9)
                        && (to.z() - depth).abs() < 1e-9 =>
                {
                    return true;
                }
                _ => {}
            }
            if let Some(to) = move_end(*mv) {
                cursor = Some(to);
            }
        }
        false
    }

    fn notched_plate() -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![
                edge(p(0.0, 0.0), p(35.0, 0.0)),
                edge(p(35.0, 0.0), p(35.0, 20.0)),
                edge(p(35.0, 20.0), p(65.0, 20.0)),
                edge(p(65.0, 20.0), p(65.0, 0.0)),
                edge(p(65.0, 0.0), p(100.0, 0.0)),
                edge(p(100.0, 0.0), p(100.0, 60.0)),
                edge(p(100.0, 60.0), p(0.0, 60.0)),
                edge(p(0.0, 60.0), p(0.0, 0.0)),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    fn outline_distance(contour: &Contour, point: Point) -> f64 {
        contour.segments().iter().map(|seg| seg.distance_to_point(point)).fold(f64::INFINITY, f64::min)
    }

    fn sample_path(moves: &[Move]) -> Vec<(f64, f64, f64)> {
        let mut out = Vec::new();
        let mut cursor: Option<Point3> = None;
        for mv in moves {
            match *mv {
                Move::Linear { to, .. } => {
                    if let Some(from) = cursor {
                        sample_line(&mut out, from, to);
                    }
                    cursor = Some(to);
                }
                Move::Arc { to, centre, ccw, .. } => {
                    if let Some(from) = cursor {
                        sample_arc(&mut out, from, to, centre, ccw);
                    }
                    cursor = Some(to);
                }
                Move::Rapid { to } => cursor = Some(to),
                Move::Event(_) | Move::Dwell { .. } => {}
            }
        }
        out
    }

    fn sample_line(out: &mut Vec<(f64, f64, f64)>, from: Point3, to: Point3) {
        let len = from.distance(to);
        let n = ((len / 0.5).ceil() as usize).clamp(1, 80);
        for i in 0..=n {
            let t = i as f64 / n as f64;
            out.push((from.x() + (to.x() - from.x()) * t, from.y() + (to.y() - from.y()) * t, from.z() + (to.z() - from.z()) * t));
        }
    }

    fn sample_arc(out: &mut Vec<(f64, f64, f64)>, from: Point3, to: Point3, centre: Point3, ccw: bool) {
        let a0 = (from.y() - centre.y()).atan2(from.x() - centre.x());
        let a1 = (to.y() - centre.y()).atan2(to.x() - centre.x());
        let mut sweep = if ccw { (a1 - a0).rem_euclid(std::f64::consts::TAU) } else { (a0 - a1).rem_euclid(std::f64::consts::TAU) };
        if from.almost_eq(to, 1e-8) || sweep < 1e-8 {
            sweep = std::f64::consts::TAU;
        }
        let n = ((sweep / (PI / 12.0)).ceil() as usize).clamp(8, 96);
        let r = (from.x() - centre.x()).hypot(from.y() - centre.y());
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let angle = if ccw { a0 + sweep * t } else { a0 - sweep * t };
            let z = from.z() + (to.z() - from.z()) * t;
            out.push((centre.x() + r * angle.cos(), centre.y() + r * angle.sin(), z));
        }
    }

    fn l_shape(w: f64, h: f64, notch: f64) -> Contour {
        let p = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![
                edge(p(0.0, 0.0), p(w, 0.0)),
                edge(p(w, 0.0), p(w, h)),
                edge(p(w, h), p(notch, h)),
                edge(p(notch, h), p(notch, notch)),
                edge(p(notch, notch), p(0.0, notch)),
                edge(p(0.0, notch), p(0.0, 0.0)),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    fn assert_profile_safe(part: &Contour, profile: &Profile, radius: f64) {
        let moves = profile.toolpath().moves();
        assert!(!xy_rapid_below(moves, DEFAULT_RETRACT_MM), "XY rapid below the retract height");
        match moves.first() {
            Some(Move::Rapid { to }) => assert!((to.z() - DEFAULT_CLEARANCE_MM).abs() < 1e-9, "first motion Z {}", to.z()),
            other => panic!("first motion should be a rapid, got {other:?}"),
        }
        for (x, y, _) in sample_path(moves) {
            let distance = outline_distance(part, Point::new(x, y).unwrap());
            assert!(distance + 1e-6 >= radius, "gouge at ({x}, {y}): distance {distance} < radius {radius}");
        }
    }

    proptest::proptest! {
        #![proptest_config(proptest::prelude::ProptestConfig::with_cases(32))]
        #[test]
        fn random_rectangles_and_l_shapes_do_not_gouge_or_rapid_through_stock(
            w in 40.0f64..120.0,
            h in 30.0f64..80.0,
            diameter in 4.0f64..8.0,
            z in -20.0f64..-2.0,
            count in 1usize..5,
            width in 2.0f64..6.0,
            height in 0.0f64..4.0,
            l_notch in proptest::bool::ANY,
            climb in proptest::bool::ANY,
        ) {
            proptest::prop_assume!(count as f64 * width < 0.35 * 2.0 * (w + h));
            let part = if l_notch {
                let notch = (w.min(h) * 0.4).clamp(8.0, 24.0);
                l_shape(w, h, notch)
            } else {
                rect(w, h)
            };
            let direction = if climb { CutDirection::Climb } else { CutDirection::Conventional };
            let mut spec = TabSpec::count(count, width);
            if height > 0.0 {
                spec = spec.height(height.min((-z) * 0.5).max(0.2));
            }
            let op = ProfileOp::new(Tool::Diameter { diameter }, Compensation::Computed, ProfileSide::Outside, direction).z(z).tabs(spec);
            let Ok(profile) = op.generate(&part) else {
                return Ok(());
            };
            assert_profile_safe(&part, &profile, diameter * 0.5);
        }
    }
}
