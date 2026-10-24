//! Waterline finishing for steep walls of an STL mesh.
//!
//! Levels follow the same depth plan as Z-level roughing. Each polyline, closed or open, is
//! entered with a plunge: rapid at clearance to the first point, rapid down to the safe plane
//! (the retract height, or the stock top when that is higher), feed to that point's Z plus
//! axial stock, cut along the polyline, and feed back up to the retract height. The process
//! turns on once before the first plunge and off after the last retract, then the tool rapids
//! to clearance.

use campanion_mesh::{Cutter, DropCutter, Mesh, WaterlineLevel, WaterlineParams, waterline};
use campanion_path::{CutKind, Move, Point3, ProcessEvent, Toolpath};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, ToolShape};

/// Matches the move cap in `campanion-path`. A longer list is rejected before it is built.
const MAX_MOVES: usize = 65_536;

/// One waterline toolpath. `mesh` is the job's STL. Axial stock is added to every cut Z.
pub(crate) fn finish(job: &JobFile, op: &Operation, tool: &JobTool, mesh: &Mesh) -> Result<Toolpath, CliError> {
    let bounds = mesh.bounds().map_err(|err| op_err(op, err))?;
    let step_down = op.step_down();
    if !(step_down > 0.0) {
        return Err(op_err(op, "waterline needs a step_down"));
    }
    let grid = op.grid().ok_or_else(|| op_err(op, "waterline needs a grid"))?;
    let floor = op.floor_z().unwrap_or(bounds.min().z);
    let top = op.top_z().unwrap_or(bounds.max().z);
    if !(top > floor) {
        return Err(op_err(op, "top_z must be above floor_z"));
    }
    let stock = op.stock_to_leave().floor();
    let margin = op.margin();
    let cutter = cutter(op, tool)?;
    let dc = DropCutter::new(mesh, cutter).map_err(|err| op_err(op, err))?;
    let params =
        WaterlineParams { step_down, grid, tolerance: job.tolerance().chord(), top_z: top, bottom_z: floor, min_slope_deg: op.min_slope_deg() };
    let area_min = (bounds.min().x - margin, bounds.min().y - margin);
    let area_max = (bounds.max().x + margin, bounds.max().y + margin);
    let levels = waterline(&dc, area_min, area_max, &params).map_err(|err| op_err(op, err))?;
    let rapid_floor = job.retract().max(bounds.max().z);
    link_levels(op, job.clearance(), job.retract(), rapid_floor, stock, &levels)
}

pub(crate) fn cutter(op: &Operation, tool: &JobTool) -> Result<Cutter, CliError> {
    let Some(diameter) = tool.diameter() else {
        return Err(op_err(op, "waterline is not valid with a kerf tool"));
    };
    let radius = diameter * 0.5;
    let cutter = match tool.shape() {
        ToolShape::Flat => Cutter::new_flat(radius),
        ToolShape::Ball => Cutter::new_ball(radius),
        ToolShape::Bull { corner_radius } => Cutter::new_bull(radius, corner_radius),
    };
    cutter.map_err(|err| op_err(op, err))
}

/// Levels top to bottom, polylines in order. Rapids between polylines stay at clearance.
fn link_levels(op: &Operation, clearance: f64, retract: f64, rapid_floor: f64, stock: f64, levels: &[WaterlineLevel]) -> Result<Toolpath, CliError> {
    let travel = clearance.max(rapid_floor);
    let mut pen = Pen::new(op);
    let mut turned_on = false;
    let mut any = false;
    for level in levels {
        for points in &level.loops {
            let Some(start) = points.first() else {
                continue;
            };
            pen.approach(start.x, start.y, travel, rapid_floor)?;
            if !turned_on {
                pen.push(Move::Event(ProcessEvent::On))?;
                turned_on = true;
            }
            pen.linear(start.x, start.y, start.z + stock, CutKind::Plunge)?;
            for point in points.iter().skip(1) {
                pen.linear(point.x, point.y, point.z + stock, CutKind::Cut)?;
            }
            if let Some(here) = pen.at {
                pen.linear(here.x(), here.y(), retract, CutKind::Retract)?;
            }
            any = true;
        }
    }
    if !any {
        return Err(op_err(op, "waterline produced no toolpath"));
    }
    pen.push(Move::Event(ProcessEvent::Off))?;
    if let Some(here) = pen.at {
        pen.rapid(here.x(), here.y(), travel)?;
    }
    Toolpath::new(pen.moves).map_err(|err| op_err(op, err))
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
