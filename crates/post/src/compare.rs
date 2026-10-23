//! Check posted motion against the program that produced it.
//!
//! Every rapid and line must end at the programmed point. An arc or helix may be split or
//! replaced by chords. Split arcs share the centre. Chord vertices must lie on the original
//! curve, and the last vertex must be the programmed end.
//!
//! A lathe post's read-back is already in the part frame: diameter-mode X was divided by two,
//! Y is 0, and G2/G3 were mapped back to the IR sense. Arcs are then checked in the ZX plane
//! with the same centre and curve rules as a mill arc.

use campanion_ir::{ArcSense, ClBlock, ClMotion, Plane, Pose, Program, RotarySetup, Vec3};

use crate::def::{PostDef, RotaryFeedMode};
use crate::error::{Result, invalid, unsupported};
use crate::fmt::xy_text_eq;
use crate::geom::{self, Xyz};
use crate::read::ReadMove;
use crate::rotary::{self};

/// Compare motion ends in `program` with `moves`.
///
/// `tol` is millimetres. Non-motion blocks in `moves` are ignored. A channel program is
/// [`crate::PostError::NotSupported`].
///
/// `post` selects moves the writer left out. `z_mode` `drop` compares X and Y only. A rapid that
/// only changes Z is skipped when `process_feeds.rapid_z` is false. A feed that only changes Z
/// is skipped in that Z mode too, because the block would have no axis word.
///
/// Rotary moves are mapped back with [`campanion_ir::machine_to_part`]. The tip uses `tol`. The
/// angle uses `max(0.001 deg, format resolution)`. A feed is checked in the mode that was
/// written: inverse time against `1 / minutes` from the part-frame tip distance, and
/// degrees-per-minute or surface against the IR feed.
pub fn compare(program: &Program, moves: &[ReadMove], tol: f64, post: &PostDef) -> Result<()> {
    if !(tol.is_finite() && tol >= 0.0) {
        return Err(invalid("tolerance must be finite and >= 0"));
    }
    if !program.channel_programs().is_empty() {
        return Err(unsupported("channel programs are not supported"));
    }
    let drop_z = post.drop_z();
    let setup = program.rotary();
    let angle_tol = post.rotary.as_ref().map(|rotary| rotary::angle_tolerance(rotary.decimals)).unwrap_or(0.001);
    let motions: Vec<&ReadMove> = moves.iter().filter(|move_| is_motion(move_)).collect();
    let mut cursor = 0usize;
    let mut part: Option<Xyz> = None;
    let mut written: Option<Xyz> = None;
    let mut prev_angle: Option<f64> = None;
    let mut index = 0usize;
    for path in program.toolpaths() {
        for block in path.blocks() {
            let ClBlock::Motion(motion) = block else { continue };
            let end_pose = pose_of(motion);
            let end_angle = end_pose.rotary();
            let part_end = tip(end_pose);
            let written_end = written_tip(setup, end_pose)?;
            let rotating = rotary::angle_changes(prev_angle, end_angle);
            match motion {
                ClMotion::Rapid { .. } => {
                    if rotating || !skip_z_only(post, written, written_end, true)? {
                        take_line(
                            &motions,
                            &mut cursor,
                            true,
                            part_end,
                            tol,
                            index,
                            drop_z,
                            LineCheck { setup, angle: end_angle, angle_tol, feed: None, feed_decimals: post.decimals.feed },
                        )?;
                    }
                }
                ClMotion::Linear { feed, .. } => {
                    let length = part.map(|start| geom::dist3(start, part_end)).unwrap_or(0.0);
                    let feed_check = Some(FeedCheck { ir_feed: *feed, inverse: expect_inverse(post, rotating), length });
                    if rotating || !skip_z_only(post, written, written_end, false)? {
                        take_line(
                            &motions,
                            &mut cursor,
                            false,
                            part_end,
                            tol,
                            index,
                            drop_z,
                            LineCheck { setup, angle: end_angle, angle_tol, feed: feed_check, feed_decimals: post.decimals.feed },
                        )?;
                    }
                }
                ClMotion::Arc { plane, center, sense, feed, .. } => {
                    let start = part.ok_or_else(|| invalid(format!("motion {index}: arc needs a prior position")))?;
                    let center = xyz(*center);
                    let sweep = geom::arc_sweep(start, part_end, center, *plane, *sense, 0.0)?;
                    consume_curve(
                        &motions,
                        &mut cursor,
                        start,
                        part_end,
                        center,
                        *plane,
                        *sense,
                        sweep,
                        tol,
                        index,
                        drop_z,
                        CurveCheck { setup, start_angle: prev_angle, end_angle, ir_feed: *feed, angle_tol, post },
                    )?;
                }
                ClMotion::Helix { plane, center, sense, sweep, feed, .. } => {
                    let start = part.ok_or_else(|| invalid(format!("motion {index}: helix needs a prior position")))?;
                    let center = xyz(*center);
                    consume_curve(
                        &motions,
                        &mut cursor,
                        start,
                        part_end,
                        center,
                        *plane,
                        *sense,
                        *sweep,
                        tol,
                        index,
                        drop_z,
                        CurveCheck { setup, start_angle: prev_angle, end_angle, ir_feed: *feed, angle_tol, post },
                    )?;
                }
            }
            part = Some(part_end);
            written = Some(written_end);
            prev_angle = end_angle;
            index += 1;
        }
    }
    if cursor != motions.len() {
        return Err(invalid(format!("g-code has {} extra motion blocks", motions.len() - cursor)));
    }
    Ok(())
}

/// True when the writer emits no block: X and Y are unchanged, and either this is a dropped
/// Z rapid or Z words are omitted so a feed would also be empty.
fn skip_z_only(post: &PostDef, pos: Option<Xyz>, end: Xyz, rapid: bool) -> Result<bool> {
    let Some(pos) = pos else {
        return Ok(false);
    };
    if !xy_text_eq(pos, end, post.decimals.linear, post.trailing_zeros)? {
        return Ok(false);
    }
    let drop_rapid = rapid && !post.process_feeds.rapid_z;
    Ok(drop_rapid || post.drop_z())
}

fn project(point: Xyz, drop_z: bool) -> Xyz {
    if drop_z { [point[0], point[1], 0.0] } else { point }
}

struct FeedCheck {
    ir_feed: f64,
    inverse: bool,
    /// Straight-line distance between the part-frame tips. Used when `inverse` is set.
    length: f64,
}

struct LineCheck<'a> {
    setup: Option<&'a RotarySetup>,
    angle: Option<f64>,
    angle_tol: f64,
    feed: Option<FeedCheck>,
    feed_decimals: u8,
}

fn take_line(
    moves: &[&ReadMove],
    cursor: &mut usize,
    rapid: bool,
    end: Xyz,
    tol: f64,
    index: usize,
    drop_z: bool,
    check: LineCheck<'_>,
) -> Result<()> {
    let Some(mv) = moves.get(*cursor).copied() else {
        return Err(invalid(format!("motion {index}: missing g-code")));
    };
    *cursor += 1;
    let (got_rapid, got, got_angle, got_feed, got_inverse) = match mv {
        ReadMove::Rapid { end, feed, rotary, inverse_time } => (true, *end, *rotary, *feed, *inverse_time),
        ReadMove::Linear { end, feed, rotary, inverse_time } => (false, *end, *rotary, *feed, *inverse_time),
        ReadMove::Arc { .. } => return Err(invalid(format!("motion {index}: expected a line"))),
        _ => return Err(invalid(format!("motion {index}: expected a line"))),
    };
    if got_rapid != rapid {
        return Err(invalid(format!("motion {index}: feed and rapid were swapped")));
    }
    let got = map_tip(got, got_angle, check.angle, check.setup, index)?;
    if let Some(expect) = check.angle {
        check_angle(got_angle, expect, check.angle_tol, index)?;
    }
    let got = project(got, drop_z);
    let end = project(end, drop_z);
    if geom::dist3(got, end) > tol {
        return Err(invalid(format!("motion {index}: end differs by {:.6} mm", geom::dist3(got, end))));
    }
    if !rapid && let Some(feed) = check.feed {
        if got_inverse != feed.inverse {
            return Err(invalid(format!("motion {index}: feed mode differs")));
        }
        let expected = if feed.inverse { rotary::inverse_minutes_feed(feed.ir_feed, feed.length)? } else { feed.ir_feed };
        check_feed_number(got_feed, expected, check.feed_decimals, index)?;
    }
    Ok(())
}

struct CurveCheck<'a> {
    setup: Option<&'a RotarySetup>,
    start_angle: Option<f64>,
    end_angle: Option<f64>,
    ir_feed: f64,
    angle_tol: f64,
    post: &'a PostDef,
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
    drop_z: bool,
    check: CurveCheck<'_>,
) -> Result<()> {
    let rotary_curve = check.start_angle.is_some() || check.end_angle.is_some();
    let start = project(start, drop_z);
    let end = project(end, drop_z);
    let center = project(center, drop_z);
    if !(sweep.is_finite() && sweep > 0.0) {
        return Err(invalid(format!("motion {index}: sweep must be positive")));
    }
    let mut prev = start;
    let mut prev_part = start;
    let mut prev_angle = check.start_angle;
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
        let angle = move_angle(mv);
        let part_point = if rotary_curve {
            let angle = angle.ok_or_else(|| invalid(format!("motion {index}: missing rotary angle")))?;
            let setup = check.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?;
            rotary::part_tip_at(setup, end_of(mv), angle)?
        } else {
            end_of(mv)
        };
        let point = project(part_point, drop_z);
        match mv {
            ReadMove::Rapid { .. } => return Err(invalid(format!("motion {index}: arc was posted as a rapid"))),
            ReadMove::Arc { sense: got, center: got_center, radius, .. } => {
                let got_sense = if rotary_curve {
                    let angle = angle.ok_or_else(|| invalid(format!("motion {index}: missing rotary angle")))?;
                    let setup = check.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?;
                    rotary::part_sense(setup, angle, plane, *got)
                } else {
                    *got
                };
                if got_sense != sense {
                    return Err(invalid(format!("motion {index}: arc sense differs")));
                }
                let got_center = if rotary_curve {
                    let angle = angle.ok_or_else(|| invalid(format!("motion {index}: missing rotary angle")))?;
                    let setup = check.setup.ok_or_else(|| invalid("rotary angle needs a rotary setup"))?;
                    rotary::part_center(setup, *got_center, angle)?
                } else {
                    *got_center
                };
                let got_center = project(got_center, drop_z);
                let centre_tol = if radius.is_some() { radius_centre_limit(prev, point, center, plane, tol) } else { tol };
                if !centre_ok(got_center, center, plane, radius.is_some(), centre_tol) {
                    return Err(invalid(format!("motion {index}: arc centre differs")));
                }
            }
            ReadMove::Linear { .. } => {}
            _ => return Err(invalid(format!("motion {index}: expected a motion"))),
        }
        if !on_curve(point, start, end, center, plane, sense, sweep, tol) {
            return Err(invalid(format!("motion {index}: point is off the arc")));
        }
        check_curve_feed(mv, prev_part, part_point, prev_angle, angle, &check, tol, index)?;
        acc += travel(prev, point, center, plane, sense, sweep - acc, tol);
        prev = point;
        prev_part = part_point;
        prev_angle = angle;
        if geom::dist3(point, end) <= tol && acc + 1e-3 >= sweep {
            if acc > sweep + 0.25 {
                return Err(invalid(format!("motion {index}: arc travelled too far")));
            }
            if let Some(expect) = check.end_angle {
                check_angle(angle, expect, check.angle_tol, index)?;
            }
            return Ok(());
        }
    }
}

/// `R` recovers the centre from rounded endpoints. Short sweeps swing a distant centre by about
/// `radius/chord` quanta, and a near-semicircle swings it by about `sqrt(radius * quantum)`.
/// `tol` is that quantum for the built-in posts. The factor covers the amplification.
fn radius_centre_limit(prev: Xyz, point: Xyz, center: Xyz, plane: Plane, tol: f64) -> f64 {
    let radius = geom::in_plane_len(plane, prev, center);
    let chord = geom::in_plane_len(plane, prev, point).max(1e-6);
    if !(radius.is_finite() && radius > 1e-12 && tol.is_finite()) {
        return tol;
    }
    let swing = radius / chord * tol;
    let half = (chord * 0.5).min(radius);
    let height = (radius * radius - half * half).max(0.0).sqrt();
    let toward = if height > radius * 1e-3 { radius / height * tol } else { (radius * tol).max(0.0).sqrt() };
    let scaled = swing.max(toward).max(tol) * 4.0;
    if scaled.is_finite() { scaled } else { tol }
}

fn centre_ok(got: Xyz, expect: Xyz, plane: Plane, _radius_mode: bool, tol: f64) -> bool {
    // `K` on G17 (and the other out-of-plane offset) is not part of the arc. A helix keeps one
    // centre while Z moves, so later blocks reconstruct that centre in the plane of the block start.
    geom::in_plane_len(plane, got, expect) <= tol
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

fn pose_of(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. } | ClMotion::Arc { to, .. } | ClMotion::Helix { to, .. } => *to,
    }
}

fn written_tip(setup: Option<&RotarySetup>, pose: Pose) -> Result<Xyz> {
    let part = tip(pose);
    let Some(angle) = pose.rotary() else {
        return Ok(part);
    };
    let Some(setup) = setup else {
        return Ok(part);
    };
    rotary::machine_tip(setup, part, angle)
}

fn expect_inverse(post: &PostDef, rotating: bool) -> bool {
    rotating && post.rotary.as_ref().is_some_and(|rotary| rotary.mode() == RotaryFeedMode::InverseTime)
}

fn map_tip(got: Xyz, got_angle: Option<f64>, expect_angle: Option<f64>, setup: Option<&RotarySetup>, index: usize) -> Result<Xyz> {
    let Some(_) = expect_angle else {
        return Ok(got);
    };
    let Some(angle) = got_angle else {
        return Err(invalid(format!("motion {index}: missing rotary angle")));
    };
    let Some(setup) = setup else {
        return Err(invalid("rotary angle needs a rotary setup"));
    };
    rotary::part_tip_at(setup, got, angle)
}

fn check_angle(got: Option<f64>, expect: f64, tol: f64, index: usize) -> Result<()> {
    let Some(got) = got else {
        return Err(invalid(format!("motion {index}: missing rotary angle")));
    };
    let gap = (got - expect).abs();
    if gap > tol { Err(invalid(format!("motion {index}: rotary angle differs by {gap:.6} deg"))) } else { Ok(()) }
}

fn check_feed_number(got: Option<f64>, expected: f64, decimals: u8, index: usize) -> Result<()> {
    let Some(got) = got else {
        return Err(invalid(format!("motion {index}: missing feed")));
    };
    let quantum = 10_f64.powi(-i32::from(decimals));
    let limit = quantum * 0.51 + expected.abs() * 1e-12;
    if (got - expected).abs() > limit { Err(invalid(format!("motion {index}: feed {got} differs from {expected}"))) } else { Ok(()) }
}

fn move_angle(mv: &ReadMove) -> Option<f64> {
    match mv {
        ReadMove::Rapid { rotary, .. } | ReadMove::Linear { rotary, .. } | ReadMove::Arc { rotary, .. } => *rotary,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
    }
}

fn move_feed(mv: &ReadMove) -> Option<f64> {
    match mv {
        ReadMove::Rapid { feed, .. } | ReadMove::Linear { feed, .. } | ReadMove::Arc { feed, .. } => *feed,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
    }
}

fn move_inverse(mv: &ReadMove) -> bool {
    match mv {
        ReadMove::Rapid { inverse_time, .. } | ReadMove::Linear { inverse_time, .. } | ReadMove::Arc { inverse_time, .. } => *inverse_time,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => false,
    }
}

fn check_curve_feed(
    mv: &ReadMove,
    prev: Xyz,
    point: Xyz,
    prev_angle: Option<f64>,
    angle: Option<f64>,
    check: &CurveCheck<'_>,
    tol: f64,
    index: usize,
) -> Result<()> {
    let inverse = expect_inverse(check.post, rotary::angle_changes(prev_angle, angle));
    if move_inverse(mv) != inverse {
        return Err(invalid(format!("motion {index}: feed mode differs")));
    }
    if inverse {
        let Some(got) = move_feed(mv) else {
            return Err(invalid(format!("motion {index}: missing feed")));
        };
        if !(got.is_finite() && got > 0.0) {
            return Err(invalid(format!("motion {index}: feed must be positive")));
        }
        let implied = check.ir_feed / got;
        let length = geom::dist3(prev, point);
        let quantum = 10_f64.powi(-i32::from(check.post.decimals.linear));
        let limit = (4.0 * quantum).max(tol);
        if (implied - length).abs() > limit {
            return Err(invalid(format!("motion {index}: inverse-time feed implies {implied:.6} mm, segment is {length:.6} mm")));
        }
        Ok(())
    } else {
        check_feed_number(move_feed(mv), check.ir_feed, check.post.decimals.feed, index)
    }
}

fn xyz(point: Vec3) -> Xyz {
    [point.x(), point.y(), point.z()]
}
