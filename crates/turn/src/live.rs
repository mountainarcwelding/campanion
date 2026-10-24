//! Indexed live-tool drilling and slotting.
//!
//! The spindle is a positioning C axis. These functions do not interpolate C, and they do not
//! use a Y axis. C changes only on a rapid at the clearance point, while the brake is released.
//! Coordinates are the tool centre: Z along the spindle, positive away from the chuck, and X as
//! a radius. A front hole's tip stops at `z_start - depth`. An OD hole's tip stops at radius
//! `diameter / 2 - depth`. No drill-point allowance is added.
//!
//! Pecking feeds from the clearance point. Every peck except the last is followed by a rapid
//! back to that same clearance point (a full retract). The last peck is followed by a feed, at
//! the cut feed, back to clearance. A slot zigzags: plunge at `from`, feed to `to`, step down,
//! feed back, and after the last pass feed out to clearance. The caller turns the steps into a
//! cutter-location path and supplies the C-axis, brake and live-tool events.

use crate::ops::{self, MAX_MOVES};
use crate::{Result, TurnError};

/// Most holes one live drill accepts.
pub const MAX_LIVE_POSITIONS: usize = 64;

/// Most depth passes one live slot accepts.
const MAX_SLOT_PASSES: usize = 1_024;

/// Largest absolute C angle a turning pose accepts.
const MAX_C_DEG: f64 = 99_999.0;

/// A bottom radius this close to zero still counts as on the axis.
const RADIUS_EPS: f64 = 1e-9;

/// Which surface the live tool works.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LiveFace {
    /// Front face. The tool axis is the spindle axis.
    Front,
    /// Outside diameter. The tool axis is radial.
    Od,
}

impl LiveFace {
    /// True when the pose axis is the spindle axis.
    pub fn axial(self) -> bool {
        matches!(self, Self::Front)
    }
}

/// One indexed hole.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveStation {
    /// C angle in degrees.
    pub c_deg: f64,
    /// Front: tool-centre radius. OD: tool-centre Z.
    pub at: f64,
}

/// Parameters for [`live_drill`].
#[derive(Clone, Debug, PartialEq)]
pub struct LiveDrillParams {
    /// Front face or outside diameter.
    pub face: LiveFace,
    /// Holes, in cut order. One to [`MAX_LIVE_POSITIONS`].
    pub positions: Vec<LiveStation>,
    /// How far the tip goes past the surface, millimetres. Positive.
    pub depth: f64,
    /// Axial or radial peck. `None` is one feed to the bottom.
    pub peck: Option<f64>,
    /// Gap from the surface to the clearance point. `>= 0`.
    pub retract: f64,
    /// Z of the front face. Used for [`LiveFace::Front`].
    pub z_start: f64,
    /// Diameter of the surface being drilled. Used for [`LiveFace::Od`].
    pub diameter: f64,
}

/// Parameters for [`live_slot`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveSlotParams {
    /// Front face or outside diameter.
    pub face: LiveFace,
    /// C angle in degrees. The slot is indexed, not wrapped.
    pub c_deg: f64,
    /// Front: tool-centre radius at the start. OD: tool-centre Z at the start.
    pub from: f64,
    /// Front: tool-centre radius at the far end. OD: tool-centre Z at the far end.
    pub to: f64,
    /// How far the tip goes past the surface, millimetres. Positive.
    pub depth: f64,
    /// Maximum thickness of one pass. The last pass stops at [`Self::depth`].
    pub step_down: f64,
    /// Gap from the surface to the clearance point. `>= 0`.
    pub retract: f64,
    /// Z of the front face. Used for [`LiveFace::Front`].
    pub z_start: f64,
    /// Diameter of the surface being slotted. Used for [`LiveFace::Od`].
    pub diameter: f64,
}

/// One step of an indexed live cycle, in order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LiveStep {
    /// Rapid. C may change only while the brake is released, and only at clearance.
    Rapid { z: f64, x: f64, c_deg: f64 },
    /// Linear feed at the C of the rapid that indexed this hole or slot.
    Feed { z: f64, x: f64, c_deg: f64 },
    /// Clamp the spindle brake.
    Clamp,
    /// Release the spindle brake.
    Release,
}

/// Indexed live-tool motion. `axial` is true for a front tool (pose axis +Z).
#[derive(Clone, Debug, PartialEq)]
pub struct LivePath {
    /// True when the tool axis is the spindle axis. False when it is radial.
    pub axial: bool,
    /// Steps in cut order.
    pub steps: Vec<LiveStep>,
}

/// Drill each position: rapid to clearance, clamp, peck to the bottom, feed out, release.
pub fn live_drill(params: &LiveDrillParams) -> Result<LivePath> {
    check_drill(params)?;
    let mut path = Builder::new(params.face.axial());
    for station in &params.positions {
        let place = station.at;
        let c = station.c_deg;
        let (z_clear, x_clear) = clearance_at(params.face, params.z_start, params.diameter, params.retract, place);
        path.rapid(z_clear, x_clear, c)?;
        path.clamp()?;
        peck_to_bottom(&mut path, params, place, c)?;
        path.feed(z_clear, x_clear)?;
        path.release()?;
    }
    Ok(path.finish())
}

/// Slot along `from`..`to` in zigzag depth passes, then feed back to clearance.
pub fn live_slot(params: &LiveSlotParams) -> Result<LivePath> {
    check_slot(params)?;
    let levels = pass_depths(params.depth, params.step_down)?;
    let mut path = Builder::new(params.face.axial());
    let (z_clear, x_clear) = clearance_at(params.face, params.z_start, params.diameter, params.retract, params.from);
    path.rapid(z_clear, x_clear, params.c_deg)?;
    path.clamp()?;
    let mut along = params.from;
    let mut toward_to = true;
    for level in levels {
        let (z, x) = at_depth(params.face, params.z_start, params.diameter, along, level);
        path.feed(z, x)?;
        let next = if toward_to { params.to } else { params.from };
        let (z, x) = at_depth(params.face, params.z_start, params.diameter, next, level);
        path.feed(z, x)?;
        along = next;
        toward_to = !toward_to;
    }
    let (z_out, x_out) = clearance_at(params.face, params.z_start, params.diameter, params.retract, along);
    path.feed(z_out, x_out)?;
    path.release()?;
    Ok(path.finish())
}

fn check_drill(params: &LiveDrillParams) -> Result<()> {
    if params.positions.is_empty() {
        return Err(invalid("positions is empty"));
    }
    if params.positions.len() > MAX_LIVE_POSITIONS {
        return Err(invalid(format!("too many positions: {} > {MAX_LIVE_POSITIONS}", params.positions.len())));
    }
    positive(params.depth, "depth")?;
    non_negative(params.retract, "retract")?;
    if let Some(peck) = params.peck {
        positive(peck, "peck")?;
    }
    match params.face {
        LiveFace::Front => {
            finite(params.z_start, "z start")?;
            for station in &params.positions {
                check_c(station.c_deg)?;
                non_negative(station.at, "radius")?;
            }
        }
        LiveFace::Od => {
            check_surface(params.diameter, params.depth)?;
            for station in &params.positions {
                check_c(station.c_deg)?;
                finite(station.at, "z")?;
            }
        }
    }
    Ok(())
}

fn check_slot(params: &LiveSlotParams) -> Result<()> {
    check_c(params.c_deg)?;
    positive(params.depth, "depth")?;
    positive(params.step_down, "step down")?;
    non_negative(params.retract, "retract")?;
    match params.face {
        LiveFace::Front => {
            finite(params.z_start, "z start")?;
            non_negative(params.from, "radius")?;
            non_negative(params.to, "radius")?;
        }
        LiveFace::Od => {
            check_surface(params.diameter, params.depth)?;
            finite(params.from, "z")?;
            finite(params.to, "z")?;
        }
    }
    Ok(())
}

fn check_surface(diameter: f64, depth: f64) -> Result<()> {
    positive(diameter, "diameter")?;
    let bottom = diameter * 0.5 - depth;
    if !bottom.is_finite() || bottom < -RADIUS_EPS {
        return Err(invalid("depth exceeds the surface radius"));
    }
    Ok(())
}

fn check_c(c: f64) -> Result<()> {
    finite(c, "c")?;
    if c.abs() > MAX_C_DEG {
        return Err(invalid("c is outside -99999..=99999"));
    }
    Ok(())
}

/// Depth of each slot pass. `k * step` for `k = 1, 2, ...`, and the last value is `depth`.
fn pass_depths(depth: f64, step: f64) -> Result<Vec<f64>> {
    let mut levels = Vec::new();
    let mut k: u32 = 1;
    loop {
        if levels.len() >= MAX_SLOT_PASSES {
            return Err(invalid(format!("too many slot passes: limit is {MAX_SLOT_PASSES}")));
        }
        let raw = f64::from(k) * step;
        finite(raw, "slot pass")?;
        if raw + RADIUS_EPS >= depth {
            levels.push(depth);
            break;
        }
        levels.push(raw);
        k = k.checked_add(1).ok_or_else(|| invalid(format!("too many slot passes: limit is {MAX_SLOT_PASSES}")))?;
    }
    Ok(levels)
}

fn peck_to_bottom(path: &mut Builder, params: &LiveDrillParams, place: f64, _c: f64) -> Result<()> {
    let mut done = 0.0;
    let mut guard = 0usize;
    loop {
        guard += 1;
        if guard > MAX_MOVES {
            return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
        }
        let remaining = params.depth - done;
        if remaining <= 1e-15 {
            break;
        }
        let step = match params.peck {
            Some(peck) => peck.min(remaining),
            None => remaining,
        };
        if !(step > 0.0) {
            return Err(invalid("peck did not advance"));
        }
        done += step;
        let at_end = params.depth - done <= 1e-12;
        if at_end {
            done = params.depth;
        }
        let (z, x) = at_depth(params.face, params.z_start, params.diameter, place, done);
        path.feed(z, x)?;
        if at_end {
            break;
        }
        let (z_clear, x_clear) = clearance_at(params.face, params.z_start, params.diameter, params.retract, place);
        path.rapid(z_clear, x_clear, path.c())?;
    }
    Ok(())
}

fn surface_radius(diameter: f64) -> f64 {
    (diameter * 0.5).max(0.0)
}

fn clearance_at(face: LiveFace, z_start: f64, diameter: f64, retract: f64, place: f64) -> (f64, f64) {
    match face {
        LiveFace::Front => (z_start + retract, place),
        LiveFace::Od => (place, surface_radius(diameter) + retract),
    }
}

fn at_depth(face: LiveFace, z_start: f64, diameter: f64, place: f64, depth: f64) -> (f64, f64) {
    match face {
        LiveFace::Front => (z_start - depth, place),
        LiveFace::Od => {
            let radius = (surface_radius(diameter) - depth).max(0.0);
            (place, radius)
        }
    }
}

struct Builder {
    axial: bool,
    steps: Vec<LiveStep>,
    cursor: Option<(f64, f64, f64)>,
    clamped: bool,
}

impl Builder {
    fn new(axial: bool) -> Self {
        Self { axial, steps: Vec::new(), cursor: None, clamped: false }
    }

    fn c(&self) -> f64 {
        self.cursor.map(|(_, _, c)| c).unwrap_or(0.0)
    }

    fn finish(self) -> LivePath {
        LivePath { axial: self.axial, steps: self.steps }
    }

    fn rapid(&mut self, z: f64, x: f64, c: f64) -> Result<()> {
        if self.clamped && self.cursor.is_some_and(|(_, _, current)| current != c) {
            return Err(invalid("C changes while the brake is clamped"));
        }
        self.push(z, x, c, true)
    }

    fn feed(&mut self, z: f64, x: f64) -> Result<()> {
        if !self.clamped {
            return Err(invalid("feed while the brake is released"));
        }
        let c = self.cursor.map(|(_, _, c)| c).ok_or_else(|| invalid("feed before a rapid"))?;
        self.push(z, x, c, false)
    }

    fn clamp(&mut self) -> Result<()> {
        if self.clamped {
            return Err(invalid("brake is already clamped"));
        }
        self.steps.push(LiveStep::Clamp);
        self.clamped = true;
        Ok(())
    }

    fn release(&mut self) -> Result<()> {
        if !self.clamped {
            return Err(invalid("brake is already released"));
        }
        self.steps.push(LiveStep::Release);
        self.clamped = false;
        Ok(())
    }

    fn push(&mut self, z: f64, x: f64, c: f64, rapid: bool) -> Result<()> {
        if self.steps.len() >= MAX_MOVES {
            return Err(invalid(format!("too many moves: limit is {MAX_MOVES}")));
        }
        if self.cursor.is_some_and(|(cz, cx, cc)| cz == z && cx == x && cc == c) {
            return Ok(());
        }
        let step = if rapid { LiveStep::Rapid { z, x, c_deg: c } } else { LiveStep::Feed { z, x, c_deg: c } };
        self.steps.push(step);
        self.cursor = Some((z, x, c));
        Ok(())
    }
}

fn invalid(message: impl Into<String>) -> TurnError {
    ops::invalid(message)
}

fn finite(value: f64, what: &str) -> Result<()> {
    ops::finite(value, what)
}

fn positive(value: f64, what: &str) -> Result<()> {
    ops::positive(value, what)
}

fn non_negative(value: f64, what: &str) -> Result<()> {
    ops::non_negative(value, what)
}

#[cfg(test)]
mod tests {
    use super::{LiveDrillParams, LiveFace, LiveSlotParams, LiveStation, LiveStep, MAX_LIVE_POSITIONS, live_drill, live_slot};

    fn front_hole(c: f64, radius: f64) -> LiveStation {
        LiveStation { c_deg: c, at: radius }
    }

    fn od_hole(c: f64, z: f64) -> LiveStation {
        LiveStation { c_deg: c, at: z }
    }

    fn assert_indexed(steps: &[LiveStep]) {
        let mut c = None;
        let mut clamped = false;
        assert!(matches!(steps.first(), Some(LiveStep::Rapid { .. })), "path does not start on a rapid");
        assert!(matches!(steps.last(), Some(LiveStep::Release)), "path does not release the brake");
        for step in steps {
            match *step {
                LiveStep::Rapid { c_deg, .. } => {
                    if let Some(previous) = c
                        && previous != c_deg
                        && clamped
                    {
                        panic!("C changed from {previous} to {c_deg} while the brake was clamped");
                    }
                    c = Some(c_deg);
                }
                LiveStep::Feed { c_deg, .. } => {
                    assert!(clamped, "feed while the brake is released");
                    assert_eq!(c, Some(c_deg), "feed C {c_deg} is not the indexed angle {c:?}");
                }
                LiveStep::Clamp => {
                    assert!(!clamped, "clamped twice");
                    clamped = true;
                }
                LiveStep::Release => {
                    assert!(clamped, "released twice");
                    clamped = false;
                }
            }
        }
        assert!(!clamped, "brake left clamped");
    }

    fn message(err: &crate::TurnError) -> String {
        err.to_string()
    }

    #[test]
    fn front_drill_one_hole_feeds_to_the_tip_depth_and_back_out() {
        let path = live_drill(&LiveDrillParams {
            face: LiveFace::Front,
            positions: vec![front_hole(90.0, 10.0)],
            depth: 5.0,
            peck: None,
            retract: 2.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect("drill");
        assert!(path.axial);
        assert_eq!(
            path.steps,
            vec![
                LiveStep::Rapid { z: 2.0, x: 10.0, c_deg: 90.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -5.0, x: 10.0, c_deg: 90.0 },
                LiveStep::Feed { z: 2.0, x: 10.0, c_deg: 90.0 },
                LiveStep::Release,
            ]
        );
        assert_indexed(&path.steps);
    }

    #[test]
    fn front_drill_pecks_retract_to_clearance_and_the_next_hole_rapids_at_that_z() {
        let path = live_drill(&LiveDrillParams {
            face: LiveFace::Front,
            positions: vec![front_hole(0.0, 10.0), front_hole(90.0, 12.0)],
            depth: 10.0,
            peck: Some(4.0),
            retract: 2.0,
            z_start: 1.0,
            diameter: 0.0,
        })
        .expect("drill");
        assert_indexed(&path.steps);
        assert_eq!(
            &path.steps[..11],
            &[
                LiveStep::Rapid { z: 3.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -3.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Rapid { z: 3.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Feed { z: -7.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Rapid { z: 3.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Feed { z: -9.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Feed { z: 3.0, x: 10.0, c_deg: 0.0 },
                LiveStep::Release,
                LiveStep::Rapid { z: 3.0, x: 12.0, c_deg: 90.0 },
                LiveStep::Clamp,
            ]
        );
        let bottom = path.steps.iter().find_map(|step| match *step {
            LiveStep::Feed { z, x, c_deg } if c_deg == 90.0 && (z + 9.0).abs() < 1e-12 => Some((z, x)),
            _ => None,
        });
        assert_eq!(bottom, Some((-9.0, 12.0)));
    }

    #[test]
    fn od_drill_bottoms_at_the_remaining_radius() {
        let path = live_drill(&LiveDrillParams {
            face: LiveFace::Od,
            positions: vec![od_hole(180.0, -8.0), od_hole(270.0, -12.0)],
            depth: 4.0,
            peck: Some(2.5),
            retract: 1.0,
            z_start: 0.0,
            diameter: 30.0,
        })
        .expect("drill");
        assert!(!path.axial);
        assert_indexed(&path.steps);
        assert_eq!(
            &path.steps[..7],
            &[
                LiveStep::Rapid { z: -8.0, x: 16.0, c_deg: 180.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -8.0, x: 12.5, c_deg: 180.0 },
                LiveStep::Rapid { z: -8.0, x: 16.0, c_deg: 180.0 },
                LiveStep::Feed { z: -8.0, x: 11.0, c_deg: 180.0 },
                LiveStep::Feed { z: -8.0, x: 16.0, c_deg: 180.0 },
                LiveStep::Release,
            ]
        );
        assert!(path.steps.iter().any(|step| matches!(*step, LiveStep::Rapid { z: -12.0, x: 16.0, c_deg: 270.0 })));
    }

    #[test]
    fn front_slot_passes_stop_at_depth_and_zigzag() {
        let path = live_slot(&LiveSlotParams {
            face: LiveFace::Front,
            c_deg: 45.0,
            from: 5.0,
            to: 12.0,
            depth: 5.0,
            step_down: 2.0,
            retract: 1.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect("slot");
        assert!(path.axial);
        assert_indexed(&path.steps);
        assert_eq!(
            path.steps,
            vec![
                LiveStep::Rapid { z: 1.0, x: 5.0, c_deg: 45.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -2.0, x: 5.0, c_deg: 45.0 },
                LiveStep::Feed { z: -2.0, x: 12.0, c_deg: 45.0 },
                LiveStep::Feed { z: -4.0, x: 12.0, c_deg: 45.0 },
                LiveStep::Feed { z: -4.0, x: 5.0, c_deg: 45.0 },
                LiveStep::Feed { z: -5.0, x: 5.0, c_deg: 45.0 },
                LiveStep::Feed { z: -5.0, x: 12.0, c_deg: 45.0 },
                LiveStep::Feed { z: 1.0, x: 12.0, c_deg: 45.0 },
                LiveStep::Release,
            ]
        );
    }

    #[test]
    fn od_slot_passes_step_inward_and_a_single_step_is_one_pass() {
        let path = live_slot(&LiveSlotParams {
            face: LiveFace::Od,
            c_deg: 0.0,
            from: -20.0,
            to: -35.0,
            depth: 3.0,
            step_down: 1.5,
            retract: 2.0,
            z_start: 0.0,
            diameter: 40.0,
        })
        .expect("slot");
        assert!(!path.axial);
        assert_indexed(&path.steps);
        assert_eq!(
            path.steps,
            vec![
                LiveStep::Rapid { z: -20.0, x: 22.0, c_deg: 0.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -20.0, x: 18.5, c_deg: 0.0 },
                LiveStep::Feed { z: -35.0, x: 18.5, c_deg: 0.0 },
                LiveStep::Feed { z: -35.0, x: 17.0, c_deg: 0.0 },
                LiveStep::Feed { z: -20.0, x: 17.0, c_deg: 0.0 },
                LiveStep::Feed { z: -20.0, x: 22.0, c_deg: 0.0 },
                LiveStep::Release,
            ]
        );

        let one = live_slot(&LiveSlotParams {
            face: LiveFace::Front,
            c_deg: 10.0,
            from: 4.0,
            to: 8.0,
            depth: 2.0,
            step_down: 9.0,
            retract: 0.5,
            z_start: -1.0,
            diameter: 0.0,
        })
        .expect("one pass");
        assert_eq!(
            one.steps,
            vec![
                LiveStep::Rapid { z: -0.5, x: 4.0, c_deg: 10.0 },
                LiveStep::Clamp,
                LiveStep::Feed { z: -3.0, x: 4.0, c_deg: 10.0 },
                LiveStep::Feed { z: -3.0, x: 8.0, c_deg: 10.0 },
                LiveStep::Feed { z: -0.5, x: 8.0, c_deg: 10.0 },
                LiveStep::Release,
            ]
        );
    }

    #[test]
    fn od_depth_may_reach_the_axis_and_errors_past_it() {
        let on_axis = live_drill(&LiveDrillParams {
            face: LiveFace::Od,
            positions: vec![od_hole(0.0, -4.0)],
            depth: 15.0,
            peck: None,
            retract: 1.0,
            z_start: 0.0,
            diameter: 30.0,
        })
        .expect("on axis");
        assert!(on_axis.steps.iter().any(|step| matches!(*step, LiveStep::Feed { x: 0.0, .. })));

        let past = live_drill(&LiveDrillParams {
            face: LiveFace::Od,
            positions: vec![od_hole(0.0, -4.0)],
            depth: 15.1,
            peck: None,
            retract: 1.0,
            z_start: 0.0,
            diameter: 30.0,
        })
        .expect_err("past");
        assert!(message(&past).contains("depth exceeds the surface radius"), "{past}");

        let slot = live_slot(&LiveSlotParams {
            face: LiveFace::Od,
            c_deg: 0.0,
            from: 0.0,
            to: -5.0,
            depth: 11.0,
            step_down: 11.0,
            retract: 1.0,
            z_start: 0.0,
            diameter: 20.0,
        })
        .expect_err("slot past");
        assert!(message(&slot).contains("depth exceeds the surface radius"), "{slot}");
    }

    #[test]
    fn empty_positions_too_many_positions_and_a_bad_c_or_radius_fail() {
        let empty = live_drill(&LiveDrillParams {
            face: LiveFace::Front,
            positions: Vec::new(),
            depth: 1.0,
            peck: None,
            retract: 1.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect_err("empty");
        assert!(message(&empty).contains("positions is empty"), "{empty}");

        let mut positions = Vec::new();
        let mut angle = 0.0;
        for _ in 0..=MAX_LIVE_POSITIONS {
            positions.push(front_hole(angle, 1.0));
            angle += 1.0;
        }
        let many =
            live_drill(&LiveDrillParams { face: LiveFace::Front, positions, depth: 1.0, peck: None, retract: 1.0, z_start: 0.0, diameter: 0.0 })
                .expect_err("many");
        assert!(message(&many).contains("too many positions"), "{many}");

        let nan = live_drill(&LiveDrillParams {
            face: LiveFace::Front,
            positions: vec![front_hole(f64::NAN, 1.0)],
            depth: 1.0,
            peck: None,
            retract: 1.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect_err("nan");
        assert!(message(&nan).contains("c must be finite"), "{nan}");

        let radius = live_slot(&LiveSlotParams {
            face: LiveFace::Front,
            c_deg: 0.0,
            from: -1.0,
            to: 4.0,
            depth: 1.0,
            step_down: 1.0,
            retract: 1.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect_err("radius");
        assert!(message(&radius).contains("radius must be >= 0"), "{radius}");

        let huge = live_drill(&LiveDrillParams {
            face: LiveFace::Od,
            positions: vec![od_hole(100_000.0, -1.0)],
            depth: 1.0,
            peck: None,
            retract: 1.0,
            z_start: 0.0,
            diameter: 20.0,
        })
        .expect_err("c");
        assert!(message(&huge).contains("c is outside -99999..=99999"), "{huge}");

        let peck = live_drill(&LiveDrillParams {
            face: LiveFace::Front,
            positions: vec![front_hole(0.0, 1.0)],
            depth: 1.0,
            peck: Some(0.0),
            retract: 1.0,
            z_start: 0.0,
            diameter: 0.0,
        })
        .expect_err("peck");
        assert!(message(&peck).contains("peck must be positive"), "{peck}");
    }
}
