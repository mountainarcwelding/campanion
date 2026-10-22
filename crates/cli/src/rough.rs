//! Z-level roughing over an STL mesh.
//!
//! Levels run from just below the stock top down to the floor. Each closed ring is entered from
//! the previous level, or from the stock top on the first level. A plunge feeds straight down
//! at the ring start. A ramp follows the ring. A helix circles the ring's bounding-box centre
//! and is used only when that circle stays inside the ring and clear of the part; otherwise the
//! operation ramps at the helix angle and warns once. The tool rapids at clearance to the entry,
//! rapids down to the retract height, cuts, and feeds back up to the retract height. The process
//! turns on once before the first cut and off after the last retract, then the tool rapids to
//! clearance.

use std::io::{self, Write};

use campanion_geom::{Containment, Contour, Point, Tolerance};
use campanion_mesh::{Cutter, DropCutter, Mesh, ZLevel, ZLevelParams, zlevel_rough};
use campanion_path::{CutKind, DepthEntry, DepthPlan, Move, Point3, ProcessEvent, Toolpath, depth_toolpath};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, ToolShape};

/// Matches the move cap in `campanion-path`. A longer list is rejected before it is built.
const MAX_MOVES: usize = 65_536;

/// One roughing toolpath. `mesh` is the job's STL. Stock to leave is applied by the Z-level rings.
pub(crate) fn rough(job: &JobFile, op: &Operation, tool: &JobTool, mesh: &Mesh) -> Result<Toolpath, CliError> {
    let bounds = mesh.bounds().map_err(|err| op_err(op, err))?;
    let stepover = op.stepover().ok_or_else(|| op_err(op, "rough needs a stepover"))?;
    let step_down = op.step_down();
    if !(step_down > 0.0) {
        return Err(op_err(op, "rough needs a step_down"));
    }
    let grid = op.grid().ok_or_else(|| op_err(op, "rough needs a grid"))?;
    let floor = op.floor_z().unwrap_or(bounds.min().z);
    let top = op.top_z().unwrap_or(bounds.max().z);
    if !(top > floor) {
        return Err(op_err(op, "top_z must be above floor_z"));
    }
    let stock = op.stock_to_leave().floor();
    let margin = op.margin();
    let cutter = cutter(op, tool)?;
    let dc = DropCutter::new(mesh, cutter).map_err(|err| op_err(op, err))?;
    let params = ZLevelParams { step_down, stepover, grid, stock_to_leave: stock, top_z: top, bottom_z: floor };
    let area_min = (bounds.min().x - margin, bounds.min().y - margin);
    let area_max = (bounds.max().x + margin, bounds.max().y + margin);
    let levels = zlevel_rough(&dc, area_min, area_max, &params, &job.tolerance()).map_err(|err| op_err(op, err))?;
    link_levels(op, job.clearance(), job.retract(), top, floor, stock, job.tolerance(), &dc, &levels)
}

pub(crate) fn cutter(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "rough is not valid with a kerf tool"));
    };
    let radius = diameter * 0.5;
    let cutter = match tool.shape() {
        ToolShape::Flat => Cutter::new_flat(radius),
        ToolShape::Ball => Cutter::new_ball(radius),
        ToolShape::Bull { corner_radius } => Cutter::new_bull(radius, corner_radius),
    };
    cutter.map_err(|err| op_err(op, err))
}

/// Spacing of the helix fit samples, in millimetres along the circle.
const HELIX_SAMPLE_MM: f64 = 0.25;
/// A larger circle is treated as not fitting, so a hostile radius cannot scan without a bound.
const MAX_HELIX_SAMPLES: usize = 1_000_000;
/// Drop-cutter noise below this does not reject a helix that sits on the level.
const HELIX_FIT_MM: f64 = 1.0e-6;

struct Link<'a, 'm> {
    op: &'a Operation,
    dc: &'a DropCutter<'m>,
    tolerance: Tolerance,
    floor: f64,
    stock: f64,
    clearance: f64,
    retract: f64,
}

/// Levels top to bottom, rings in order. XY rapids between rings stay at clearance.
fn link_levels(
    op: &Operation,
    clearance: f64,
    retract: f64,
    top: f64,
    floor: f64,
    stock: f64,
    tolerance: Tolerance,
    dc: &DropCutter,
    levels: &[ZLevel],
) -> Result<Toolpath, CliError> {
    let link = Link { op, dc, tolerance, floor, stock, clearance, retract };
    let mut pen = Pen::new(op);
    let mut turned_on = false;
    let mut any = false;
    let mut warned = false;
    let mut prev_z = top;
    for level in levels {
        for ring in &level.rings {
            if ring.segments().is_empty() {
                continue;
            }
            let entry = entry_for_ring(&link, ring, level.z, op.entry(), &mut warned)?;
            cut_ring(&link, &mut pen, ring, prev_z, level.z, entry, &mut turned_on)?;
            any = true;
        }
        prev_z = level.z;
    }
    if !any {
        return Err(op_err(op, "rough produced no toolpath"));
    }
    pen.push(Move::Event(ProcessEvent::Off))?;
    if let Some(here) = pen.at {
        pen.rapid(here.x(), here.y(), clearance)?;
    }
    Toolpath::new(pen.moves).map_err(|err| op_err(op, err))
}

/// Helix that leaves the ring, or the stock allowance, becomes a ramp at the same angle.
fn entry_for_ring(link: &Link<'_, '_>, ring: &Contour, z: f64, entry: DepthEntry, warned: &mut bool) -> Result<DepthEntry, CliError> {
    let DepthEntry::Helix { radius, pitch } = entry else {
        return Ok(entry);
    };
    if helix_fits(link, ring, radius, z)? {
        return Ok(entry);
    }
    if !*warned {
        warn_helix(link.op)?;
        *warned = true;
    }
    let max_angle = (pitch / (radius * std::f64::consts::TAU)).atan();
    if !max_angle.is_finite() || max_angle <= 0.0 || max_angle >= std::f64::consts::FRAC_PI_2 {
        return Err(op_err(link.op, "helix angle is not usable as a ramp"));
    }
    Ok(DepthEntry::Ramp { max_angle })
}

fn warn_helix(op: &Operation) -> Result<(), CliError> {
    let mut out = io::stderr().lock();
    writeln!(out, "helix does not fit; ramping").map_err(|err| op_err(op, err))?;
    out.flush().map_err(|err| op_err(op, err))?;
    Ok(())
}

/// The helix is the circle `depth_toolpath` cuts: bounding-box centre, counter-clockwise.
fn helix_fits(link: &Link<'_, '_>, ring: &Contour, radius: f64, z: f64) -> Result<bool, CliError> {
    if !radius.is_finite() || radius <= 0.0 {
        return Ok(false);
    }
    let bounds = ring.bounding_box().map_err(|err| op_err(link.op, err))?;
    let cx = (bounds.min().x() + bounds.max().x()) * 0.5;
    let cy = (bounds.min().y() + bounds.max().y()) * 0.5;
    if !cx.is_finite() || !cy.is_finite() {
        return Ok(false);
    }
    let steps_f = (std::f64::consts::TAU * radius / HELIX_SAMPLE_MM).ceil().max(4.0);
    if !steps_f.is_finite() || steps_f > MAX_HELIX_SAMPLES as f64 {
        return Ok(false);
    }
    let steps = steps_f as usize;
    for index in 0..steps {
        let angle = std::f64::consts::TAU * (index as f64) / (steps as f64);
        let x = cx + radius * angle.cos();
        let y = cy + radius * angle.sin();
        if !x.is_finite() || !y.is_finite() {
            return Ok(false);
        }
        let tip = link.dc.tip_z_or(x, y, link.floor) + link.stock;
        if !tip.is_finite() || tip > z + HELIX_FIT_MM {
            return Ok(false);
        }
        let point = Point::new(x, y).map_err(|err| op_err(link.op, err))?;
        match ring.containment(point, &link.tolerance).map_err(|err| op_err(link.op, err))? {
            Containment::Outside => return Ok(false),
            Containment::Inside | Containment::Boundary => {}
        }
    }
    Ok(true)
}

/// One ring. The depth plan drops from `prev_z` to `z` in a single pass.
fn cut_ring(
    link: &Link<'_, '_>,
    pen: &mut Pen<'_>,
    ring: &Contour,
    prev_z: f64,
    z: f64,
    entry: DepthEntry,
    turned_on: &mut bool,
) -> Result<(), CliError> {
    let drop = prev_z - z;
    if !drop.is_finite() || drop <= 0.0 {
        return Err(op_err(link.op, "rough level is not below the previous level"));
    }
    let plan = DepthPlan::new(prev_z, drop, drop, entry);
    let path = depth_toolpath(ring, &plan, &link.tolerance).map_err(|err| op_err(link.op, err))?;
    let Some(start) = first_rapid(&path) else {
        return Err(op_err(link.op, "entry has no start"));
    };
    pen.approach(start.x(), start.y(), link.clearance, link.retract)?;
    // The ramp and helix are planned from `prev_z`. Feeding straight down from the retract
    // height would be a vertical move below the stock top, so the tool rapids down on the entry
    // point instead. That point is allowed at this level, which is below `prev_z`. A plunge
    // stays one feed from the retract height, which keeps an omitted entry unchanged.
    if !matches!(entry, DepthEntry::Plunge) {
        pen.rapid(start.x(), start.y(), prev_z)?;
    }
    if !*turned_on {
        pen.push(Move::Event(ProcessEvent::On))?;
        *turned_on = true;
    }
    for mv in path.moves() {
        match *mv {
            Move::Linear { .. } | Move::Arc { .. } => pen.push(*mv)?,
            Move::Rapid { .. } | Move::Event(_) | Move::Dwell { .. } => {}
        }
    }
    if let Some(here) = pen.at {
        pen.linear(here.x(), here.y(), link.retract, CutKind::Retract)?;
    }
    Ok(())
}

fn first_rapid(path: &Toolpath) -> Option<Point3> {
    path.moves().iter().find_map(|mv| match *mv {
        Move::Rapid { to } => Some(to),
        _ => None,
    })
}

struct Pen<'a> {
    op: &'a Operation,
    moves: Vec<Move>,
    at: Option<Point3>,
}

impl<'a> Pen<'a> {
    fn new(op: &'a Operation) -> Self {
        Self { op, moves: Vec::new(), at: None }
    }

    /// XY travel at clearance, then a vertical rapid down to the retract height.
    fn approach(&mut self, x: f64, y: f64, clearance: f64, retract: f64) -> Result<(), CliError> {
        if let Some(here) = self.at
            && here.z() < clearance
        {
            self.rapid(here.x(), here.y(), clearance)?;
        }
        self.rapid(x, y, clearance)?;
        self.rapid(x, y, retract)?;
        Ok(())
    }

    fn rapid(&mut self, x: f64, y: f64, z: f64) -> Result<(), CliError> {
        let to = point(self.op, x, y, z)?;
        if self.at.is_some_and(|here| same_point(here, to)) {
            return Ok(());
        }
        self.push(Move::Rapid { to })
    }

    fn linear(&mut self, x: f64, y: f64, z: f64, kind: CutKind) -> Result<(), CliError> {
        let to = point(self.op, x, y, z)?;
        if self.at.is_some_and(|here| same_point(here, to)) {
            return Ok(());
        }
        self.push(Move::Linear { to, kind })
    }

    fn push(&mut self, mv: Move) -> Result<(), CliError> {
        if self.moves.len() >= MAX_MOVES {
            return Err(op_err(self.op, format!("too many moves: {} > {MAX_MOVES}", self.moves.len() + 1)));
        }
        if let Some(to) = motion_to(mv) {
            self.at = Some(to);
        }
        self.moves.push(mv);
        Ok(())
    }
}

fn same_point(a: Point3, b: Point3) -> bool {
    a.x() == b.x() && a.y() == b.y() && a.z() == b.z()
}

fn motion_to(mv: Move) -> Option<Point3> {
    match mv {
        Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
        Move::Event(_) | Move::Dwell { .. } => None,
    }
}

fn point(op: &Operation, x: f64, y: f64, z: f64) -> Result<Point3, CliError> {
    Point3::new(x, y, z).map_err(|err| op_err(op, err))
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}
