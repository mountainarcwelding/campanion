//! Check posted motion against the program that produced it.
//!
//! Every rapid and line must end at the programmed point. An arc or helix may be split or
//! replaced by chords. Split arcs share the centre. Chord vertices must lie on the original
//! curve, and the last vertex must be the programmed end.

use campanion_ir::{ArcSense, ClBlock, ClMotion, Plane, Pose, Program, Vec3};

use crate::error::{Result, invalid, unsupported};
use crate::geom::{self, Xyz};
use crate::read::ReadMove;

/// Compare motion ends in `program` with `moves`.
///
/// `tol` is millimetres. Non-motion blocks in `moves` are ignored. A channel program is
/// [`crate::PostError::NotSupported`].
pub fn compare(program: &Program, moves: &[ReadMove], tol: f64) -> Result<()> {
    if !(tol.is_finite() && tol >= 0.0) {
        return Err(invalid("tolerance must be finite and >= 0"));
    }
    if !program.channel_programs().is_empty() {
        return Err(unsupported("channel programs are not supported"));
    }
    let motions: Vec<&ReadMove> = moves.iter().filter(|move_| is_motion(move_)).collect();
    let mut cursor = 0usize;
    let mut pos: Option<Xyz> = None;
    let mut index = 0usize;
    for path in program.toolpaths() {
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else { continue };
            match motion {
                ClMotion::Rapid { to, .. } => {
                    let end = tip(*to);
                    take_line(&motions, &mut cursor, true, end, tol, index)?;
                    pos = Some(end);
                }
                ClMotion::Linear { to, .. } => {
                    let end = tip(*to);
                    take_line(&motions, &mut cursor, false, end, tol, index)?;
                    pos = Some(end);
                }
                ClMotion::Arc { to, plane, center, sense, .. } => {
                    let start = pos.ok_or_else(|| invalid(format!("motion {index}: arc needs a prior position")))?;
                    let end = tip(*to);
                    let center = xyz(*center);
                    let sweep = geom::arc_sweep(start, end, center, *plane, *sense)?;
                    consume_curve(&motions, &mut cursor, start, end, center, *plane, *sense, sweep, tol, index)?;
                    pos = Some(end);
                }
                ClMotion::Helix { to, plane, center, sense, sweep, .. } => {
                    let start = pos.ok_or_else(|| invalid(format!("motion {index}: helix needs a prior position")))?;
                    let end = tip(*to);
                    let center = xyz(*center);
                    consume_curve(&motions, &mut cursor, start, end, center, *plane, *sense, *sweep, tol, index)?;
                    pos = Some(end);
                }
            }
            index += 1;
        }
    }
    if cursor != motions.len() {
        return Err(invalid(format!("g-code has {} extra motion blocks", motions.len() - cursor)));
    }
    Ok(())
}

fn take_line(moves: &[&ReadMove], cursor: &mut usize, rapid: bool, end: Xyz, tol: f64, index: usize) -> Result<()> {
    let Some(mv) = moves.get(*cursor).copied() else {
        return Err(invalid(format!("motion {index}: missing g-code")));
    };
    *cursor += 1;
    let (got_rapid, got) = match mv {
        ReadMove::Rapid { end, .. } => (true, *end),
        ReadMove::Linear { end, .. } => (false, *end),
        ReadMove::Arc { .. } => return Err(invalid(format!("motion {index}: expected a line"))),
        _ => return Err(invalid(format!("motion {index}: expected a line"))),
    };
    if got_rapid != rapid {
        return Err(invalid(format!("motion {index}: feed and rapid were swapped")));
    }
    if geom::dist3(got, end) > tol {
        return Err(invalid(format!("motion {index}: end differs by {:.6} mm", geom::dist3(got, end))));
    }
    Ok(())
}

fn consume_curve(
    moves: &[&ReadMove],
    cursor: &mut usize,
    start: Xyz,
    end: Xyz,
    center: Xyz,
    plane: Plane,
    sense: ArcSense,
    sweep: f64,
    tol: f64,
    index: usize,
) -> Result<()> {
    if !(sweep.is_finite() && sweep > 0.0) {
        return Err(invalid(format!("motion {index}: sweep must be positive")));
    }
    let mut prev = start;
    let mut acc = 0.0;
    let mut guard = 0u32;
    loop {
        let Some(mv) = moves.get(*cursor).copied() else {
            return Err(invalid(format!("motion {index}: g-code ended before the arc")));
        };
        *cursor += 1;
        guard += 1;
        if guard > 100_001 {
            return Err(invalid(format!("motion {index}: too many blocks")));
        }
        match mv {
            ReadMove::Rapid { .. } => return Err(invalid(format!("motion {index}: arc was posted as a rapid"))),
            ReadMove::Arc { sense: got, center: got_center, radius, .. } => {
                if *got != sense {
                    return Err(invalid(format!("motion {index}: arc sense differs")));
                }
                if !centre_ok(*got_center, center, plane, radius.is_some(), tol) {
                    return Err(invalid(format!("motion {index}: arc centre differs")));
                }
            }
            ReadMove::Linear { .. } => {}
            _ => return Err(invalid(format!("motion {index}: expected a motion"))),
        }
        let point = end_of(mv);
        if !on_curve(point, start, end, center, plane, sense, sweep, tol) {
            return Err(invalid(format!("motion {index}: point is off the arc")));
        }
        acc += travel(prev, point, center, plane, sense, sweep - acc, tol);
        prev = point;
        if geom::dist3(point, end) <= tol && acc + 1e-3 >= sweep {
            if acc > sweep + 0.25 {
                return Err(invalid(format!("motion {index}: arc travelled too far")));
            }
            return Ok(());
        }
    }
}

fn centre_ok(got: Xyz, expect: Xyz, plane: Plane, radius_mode: bool, tol: f64) -> bool {
    if radius_mode { geom::in_plane_len(plane, got, expect) <= tol } else { geom::dist3(got, expect) <= tol }
}

fn on_curve(point: Xyz, start: Xyz, end: Xyz, center: Xyz, plane: Plane, sense: ArcSense, sweep: f64, tol: f64) -> bool {
    let radius = geom::in_plane_len(plane, start, center);
    if (geom::in_plane_len(plane, point, center) - radius).abs() > tol {
        return false;
    }
    let base = forward_from(start, point, center, plane, sense);
    let (_, _, p_ax) = geom::plane_uv(plane, point);
    let (_, _, s_ax) = geom::plane_uv(plane, start);
    let (_, _, e_ax) = geom::plane_uv(plane, end);
    let max_k = (sweep / std::f64::consts::TAU).ceil().clamp(0.0, 10_000.0) as i32;
    for k in 0..=max_k {
        let theta = if base <= 1e-6 { f64::from(k) * std::f64::consts::TAU } else { base + f64::from(k) * std::f64::consts::TAU };
        if theta > sweep + 0.05 {
            continue;
        }
        let t = if sweep <= 1e-12 { 0.0 } else { (theta / sweep).clamp(0.0, 1.0) };
        if (p_ax - (s_ax + (e_ax - s_ax) * t)).abs() <= tol {
            return true;
        }
    }
    false
}

fn travel(prev: Xyz, point: Xyz, center: Xyz, plane: Plane, sense: ArcSense, remaining: f64, tol: f64) -> f64 {
    if geom::in_plane_len(plane, prev, point) <= tol.max(1e-9) {
        if remaining >= std::f64::consts::TAU - 0.2 { std::f64::consts::TAU } else { 0.0 }
    } else {
        forward_from(prev, point, center, plane, sense)
    }
}

fn forward_from(start: Xyz, point: Xyz, center: Xyz, plane: Plane, sense: ArcSense) -> f64 {
    let delta = geom::forward_delta(geom::angle_of(plane, start, center), geom::angle_of(plane, point, center), matches!(sense, ArcSense::Ccw));
    if delta > std::f64::consts::TAU - 1e-4 { 0.0 } else { delta }
}

fn is_motion(move_: &ReadMove) -> bool {
    matches!(move_, ReadMove::Rapid { .. } | ReadMove::Linear { .. } | ReadMove::Arc { .. })
}

fn end_of(move_: &ReadMove) -> Xyz {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } => *end,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => [0.0, 0.0, 0.0],
    }
}

fn tip(pose: Pose) -> Xyz {
    let point = pose.tip();
    [point.x(), point.y(), point.z()]
}

fn xyz(point: Vec3) -> Xyz {
    [point.x(), point.y(), point.z()]
}
