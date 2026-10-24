//! Check posted motion against the program that produced it.
//!
//! Every rapid and line must end at the programmed point. An arc or helix may be split or
//! replaced by chords. Split arcs share the centre. Chord vertices must lie on the original
//! curve, and the last vertex must be the programmed end.
//!
//! A lathe post's read-back is already in the part frame: diameter-mode X was divided by two,
//! Y is 0 unless the post writes a Y axis, and G2/G3 were mapped back to the IR sense on ZX
//! arcs. Arcs are checked in the plane the IR stored.
//!
//! Face and cylinder interpolation that the post writes with its own block are read back in the
//! IR frame, so those poses compare directly. The X/C fallback is read back as a radius and a C
//! angle. [`crate::interp::to_polar`] converts the IR pose the same way the writer did.

use campanion_ir::{ArcSense, ClBlock, ClMotion, Event, Plane, Pose, Program, RotarySetup, Vec3};

use crate::def::{PostDef, RotaryFeedMode};
use crate::error::{Result, invalid, unsupported};
use crate::fmt::xy_text_eq;
use crate::geom::{self, Xyz};
use crate::interp::{self, PolarCurve, PolarFrame};
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
/// degrees-per-minute or surface against the IR feed. A lathe `c_feed` of `inverse_time` recovers
/// millimetres per minute as `F * length` and allows 1%.
pub fn compare(program: &Program, moves: &[ReadMove], tol: f64, post: &PostDef) -> Result<()> {
    if !(tol.is_finite() && tol >= 0.0) {
        return Err(invalid("tolerance must be finite and >= 0"));
    }
    if !program.channel_programs().is_empty() {
        return Err(unsupported("channel programs are not supported"));
    }
    let drop_z = post.drop_z();
    let setup = program.rotary();
    let angle_tol = if program.turning().is_some() && post.lathe.as_ref().is_some_and(|lathe| lathe.c_axis.is_some()) {
        rotary::angle_tolerance(post.linear_decimals())
    } else {
        post.rotary.as_ref().map(|rotary| rotary::angle_tolerance(rotary.decimals)).unwrap_or(0.001)
    };
    let motions: Vec<&ReadMove> = moves.iter().filter(|move_| is_motion(move_)).collect();
    let mut cursor = 0usize;
    let mut index = 0usize;
    let groups = match crate::balance::channel_sections(program, post)? {
        Some(sections) => sections.into_iter().map(|section| section.paths).collect::<Vec<_>>(),
        None => vec![(0..program.toolpaths().len()).collect()],
    };
    for group in groups {
        let mut part: Option<Xyz> = None;
        let mut written: Option<Xyz> = None;
        let mut prev_angle: Option<f64> = None;
        let mut prev_c = 0.0;
        let mut cart = InterpTrack::Off;
        for path_index in group {
            let Some(path) = program.toolpaths().get(path_index) else {
                return Err(invalid("toolpath index is out of range"));
            };
            for block in path.blocks() {
                if let ClBlock::Event(event) = block {
                    note_interp(event, post, &mut cart, &mut prev_c, &mut prev_angle);
                    continue;
                }
                let ClBlock::Motion(motion) = block else { continue };
                let end_pose = pose_of(motion);
                let end_angle = end_pose.rotary();
                let part_end = tip(end_pose);
                if let Some(frame) = cart.fallback() {
                    let (rapid, feed, curve) = fallback_curve(motion, part, index)?;
                    prev_c = consume_fallback(&motions, &mut cursor, frame, curve, part_end, prev_c, rapid, feed, tol, angle_tol, index, post)?;
                    part = Some(part_end);
                    written = Some(part_end);
                    prev_angle = end_angle;
                    index += 1;
                    continue;
                }
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
                        let c_inverse = lathe_c_inverse(post) && rotating;
                        let length = if c_inverse {
                            match part {
                                Some(start) => {
                                    let c0 = prev_angle.unwrap_or(0.0);
                                    let c1 = end_angle.unwrap_or(c0);
                                    rotary::lathe_indexed_length(start, part_end, c0, c1)
                                }
                                None => 0.0,
                            }
                        } else {
                            part.map(|start| geom::dist3(start, part_end)).unwrap_or(0.0)
                        };
                        let feed_check = Some(FeedCheck {
                            ir_feed: *feed,
                            inverse: expect_inverse(post, rotating) || c_inverse,
                            length,
                            within_percent: c_inverse,
                        });
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
                    ClMotion::Thread { lead, .. } => {
                        take_thread(&motions, &mut cursor, part_end, *lead, tol, index, drop_z, post, end_angle, angle_tol, setup)?;
                    }
                }
                part = Some(part_end);
                written = Some(written_end);
                prev_angle = end_angle;
                if program.turning().is_some()
                    && let Some(angle) = end_angle
                {
                    prev_c = angle;
                }
                index += 1;
            }
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
    if !xy_text_eq(pos, end, post.decimals.linear, post.trailing_zeros, post.decimal_point)? {
        return Ok(false);
    }
    let drop_rapid = rapid && !post.process_feeds.rapid_z;
    Ok(drop_rapid || post.drop_z())
}

/// Face or cylinder frame while `compare` walks the program.
#[derive(Clone, Copy)]
enum InterpTrack {
    Off,
    Face { native: bool },
    Cylinder { radius: f64, native: bool },
}

impl InterpTrack {
    fn fallback(self) -> Option<PolarFrame> {
        match self {
            Self::Face { native: false } => Some(PolarFrame::Face),
            Self::Cylinder { radius, native: false } => Some(PolarFrame::Cylinder { radius }),
            _ => None,
        }
    }
}

fn note_interp(event: &Event, post: &PostDef, cart: &mut InterpTrack, prev_c: &mut f64, prev_angle: &mut Option<f64>) {
    match event {
        Event::CAxis { engage: true, .. } => {
            *prev_c = 0.0;
            // Engage unwinds C, matching the writer. The next indexed length starts at 0.
            *prev_angle = Some(0.0);
        }
        Event::FaceInterpolation { on: true } => {
            let native = post.lathe.as_ref().is_some_and(|lathe| lathe.face_interp.is_some());
            *cart = InterpTrack::Face { native };
        }
        Event::FaceInterpolation { on: false } => {
            if matches!(cart, InterpTrack::Face { .. }) {
                *cart = InterpTrack::Off;
            }
        }
        Event::CylinderInterpolation { radius: Some(radius) } => {
            let native = post.lathe.as_ref().is_some_and(|lathe| lathe.cylinder_interp.is_some());
            *cart = InterpTrack::Cylinder { radius: *radius, native };
        }
        Event::CylinderInterpolation { radius: None } => {
            if matches!(cart, InterpTrack::Cylinder { .. }) {
                *cart = InterpTrack::Off;
            }
        }
        _ => {}
    }
}

fn fallback_curve(motion: &ClMotion, start: Option<Xyz>, index: usize) -> Result<(bool, Option<f64>, PolarCurve)> {
    let end = tip(pose_of(motion));
    match motion {
        ClMotion::Rapid { .. } => Ok((true, None, line_curve(start, end))),
        ClMotion::Linear { feed, .. } => Ok((false, Some(*feed), line_curve(start, end))),
        ClMotion::Arc { plane, center, sense, feed, .. } => {
            let start = start.ok_or_else(|| invalid(format!("motion {index}: arc needs a prior position")))?;
            let center = xyz(*center);
            let sweep = geom::arc_sweep(start, end, center, *plane, *sense, 0.0)?;
            Ok((false, Some(*feed), PolarCurve::Arc { start, end, center, plane: *plane, sense: *sense, sweep }))
        }
        ClMotion::Helix { plane, center, sense, sweep, feed, .. } => {
            let start = start.ok_or_else(|| invalid(format!("motion {index}: helix needs a prior position")))?;
            Ok((false, Some(*feed), PolarCurve::Arc { start, end, center: xyz(*center), plane: *plane, sense: *sense, sweep: *sweep }))
        }
        ClMotion::Thread { .. } => Err(invalid("thread move during interpolation")),
    }
}

fn line_curve(start: Option<Xyz>, end: Xyz) -> PolarCurve {
    match start {
        Some(start) => PolarCurve::Line { start, end },
        None => PolarCurve::Point { end },
    }
}

/// Walk the G1/G0 chords of one fallback move. Returns the continuous C at the end.
fn consume_fallback(
    moves: &[&ReadMove],
    cursor: &mut usize,
    frame: PolarFrame,
    curve: PolarCurve,
    end_tip: Xyz,
    prev_c: f64,
    rapid: bool,
    feed: Option<f64>,
    tol: f64,
    angle_tol: f64,
    index: usize,
    post: &PostDef,
) -> Result<f64> {
    let target = interp::chained_polar(frame, &curve, prev_c)?;
    let mut prev_cart = curve_start_point(&curve);
    let mut prev_read_c = if *cursor > 0 { move_angle(moves[*cursor - 1]).unwrap_or(prev_c) } else { prev_c };
    let mut guard = 0u32;
    loop {
        let Some(mv) = moves.get(*cursor).copied() else {
            return Err(invalid(format!("motion {index}: g-code ended before the interpolation move")));
        };
        *cursor += 1;
        guard += 1;
        if guard > 100_001 {
            return Err(invalid(format!("motion {index}: too many blocks")));
        }
        let (got_rapid, got, got_c, got_feed) = match mv {
            ReadMove::Rapid { end, rotary, feed, .. } => (true, *end, *rotary, *feed),
            ReadMove::Linear { end, rotary, feed, .. } => (false, *end, *rotary, *feed),
            _ => return Err(invalid(format!("motion {index}: expected a line"))),
        };
        if got_rapid != rapid {
            return Err(invalid(format!("motion {index}: feed and rapid were swapped")));
        }
        let c = got_c.ok_or_else(|| invalid(format!("motion {index}: missing rotary angle")))?;
        if got[1].abs() > tol {
            return Err(invalid(format!("motion {index}: fallback move has a Y word")));
        }
        let cart = interp::polar_to_cart(frame, got[0], c, got[2]);
        if !on_polar_curve(cart, curve, tol) {
            return Err(invalid(format!("motion {index}: point is off the interpolation path")));
        }
        if !rapid && let Some(ir_feed) = feed {
            let changed = (c - prev_read_c).abs() > 1e-6;
            let want_inverse = changed && lathe_c_inverse(post);
            if move_inverse(mv) != want_inverse {
                return Err(invalid(format!("motion {index}: feed mode differs")));
            }
            if want_inverse {
                check_recovered_feed(got_feed, geom::dist3(prev_cart, cart), ir_feed, index)?;
            } else {
                check_feed_number(got_feed, ir_feed, post.decimals.feed, index)?;
            }
        }
        prev_cart = cart;
        prev_read_c = c;
        let dist_end = geom::dist3(cart, end_tip);
        let next_closer =
            moves.get(*cursor).is_some_and(|next| fallback_tip(next, frame).is_some_and(|point| geom::dist3(point, end_tip) + 1e-9 < dist_end));
        if dist_end <= tol && !next_closer {
            if (got[0] - target.radius).abs() > tol || (got[2] - target.z).abs() > tol {
                return Err(invalid(format!("motion {index}: polar end differs")));
            }
            check_angle(Some(c), target.c_deg, angle_tol, index)?;
            return Ok(target.c_deg);
        }
    }
}

fn fallback_tip(mv: &ReadMove, frame: PolarFrame) -> Option<Xyz> {
    let (end, rotary) = match mv {
        ReadMove::Rapid { end, rotary, .. } | ReadMove::Linear { end, rotary, .. } => (*end, *rotary),
        _ => return None,
    };
    let c = rotary?;
    Some(interp::polar_to_cart(frame, end[0], c, end[2]))
}

fn on_polar_curve(point: Xyz, curve: PolarCurve, tol: f64) -> bool {
    match curve {
        PolarCurve::Point { end } => geom::dist3(point, end) <= tol,
        PolarCurve::Line { start, end } => interp::segment_distance(point, start, end) <= tol,
        PolarCurve::Arc { start, end, center, plane, sense, sweep } => on_curve(point, start, end, center, plane, sense, sweep, tol),
    }
}

fn project(point: Xyz, drop_z: bool) -> Xyz {
    if drop_z { [point[0], point[1], 0.0] } else { point }
}

struct FeedCheck {
    ir_feed: f64,
    inverse: bool,
    /// Straight-line distance between the part-frame tips. Used when `inverse` is set.
    length: f64,
    /// Lathe C inverse time: `F * length` is millimetres per minute, within 1%.
    within_percent: bool,
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
        ReadMove::Arc { .. } | ReadMove::Thread { .. } => return Err(invalid(format!("motion {index}: expected a line"))),
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
        if feed.within_percent {
            check_recovered_feed(got_feed, feed.length, feed.ir_feed, index)?;
        } else {
            let expected = if feed.inverse { rotary::inverse_minutes_feed(feed.ir_feed, feed.length)? } else { feed.ir_feed };
            check_feed_number(got_feed, expected, check.feed_decimals, index)?;
        }
    }
    Ok(())
}

fn lathe_c_inverse(post: &PostDef) -> bool {
    post.lathe.as_ref().is_some_and(|lathe| lathe.c_feed_inverse())
}

/// Recover the IR feed from an inverse-time `F` and allow 1%.
fn check_recovered_feed(got: Option<f64>, length: f64, ir_feed: f64, index: usize) -> Result<()> {
    let Some(got) = got else {
        return Err(invalid(format!("motion {index}: missing feed")));
    };
    if !(got.is_finite() && got > 0.0) {
        return Err(invalid(format!("motion {index}: feed must be positive")));
    }
    if !(length.is_finite() && length > 1e-9) {
        return Err(invalid(format!("motion {index}: inverse time feed needs a positive distance between part-frame tips")));
    }
    let recovered = got * length;
    let limit = ir_feed.abs() * 0.01;
    if (recovered - ir_feed).abs() > limit {
        return Err(invalid(format!("motion {index}: feed {recovered} mm/min differs from {ir_feed} by more than 1%")));
    }
    Ok(())
}

fn curve_start_point(curve: &PolarCurve) -> Xyz {
    match curve {
        PolarCurve::Point { end } => *end,
        PolarCurve::Line { start, .. } => *start,
        PolarCurve::Arc { start, .. } => *start,
    }
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
    // Lathe C is already in the part frame. Only a mill rotary setup remaps the curve.
    let rotary_curve = check.setup.is_some() && (check.start_angle.is_some() || check.end_angle.is_some());
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
        // A post with Z words omitted drops a feed whose XY does not change. A collapsed arc
        // then has no block. The following motion belongs to the next program move.
        if !rotary_curve {
            let prev_read = (*cursor > 0).then(|| moves.get(*cursor - 1)).flatten().map(|m| project(end_of(m), drop_z));
            let already = prev_read.is_some_and(|at| geom::dist3(at, end) <= tol && geom::dist3(at, start) <= tol);
            let next_on_curve =
                moves.get(*cursor).is_some_and(|mv| on_curve(project(end_of(mv), drop_z), start, end, center, plane, sense, sweep, tol));
            if already && !next_on_curve {
                if let Some(expect) = check.end_angle {
                    let angle = (*cursor > 0).then(|| moves.get(*cursor - 1)).flatten().and_then(|mv| move_angle(mv));
                    check_angle(angle, expect, check.angle_tol, index)?;
                }
                return Ok(());
            }
        }
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
                if !rotary_curve {
                    let read_start = (*cursor >= 2).then(|| moves.get(*cursor - 2)).flatten().map(|m| project(end_of(m), drop_z));
                    if let Some(rs) = read_start
                        && geom::in_plane_len(plane, rs, point) <= 1e-9
                        && sweep < std::f64::consts::TAU - 0.2
                    {
                        return Err(invalid(format!("motion {index}: arc block has equal endpoints and cuts a full circle")));
                    }
                }
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
        let radius = geom::in_plane_len(plane, start, center);
        let slack = if radius <= 1e-12 { 1e-3 } else { 1e-3_f64.max(tol / radius) };
        // Both programmed ends lie on this posted point when the chord rounded away.
        // A line there cannot cut a circle; an equal-endpoint arc was rejected above.
        let unresolved = matches!(mv, ReadMove::Linear { .. }) && geom::dist3(point, start) <= tol;
        if geom::dist3(point, end) <= tol && (acc + slack >= sweep || unresolved) {
            if acc > sweep + 0.25 {
                return Err(invalid(format!("motion {index}: arc travelled too far")));
            }
            // Completion already requires this block to be within tol of the programmed end, so a
            // same-sense arc block that also ends there is this arc's last piece. A separate tiny
            // program arc there is still accepted by the collapsed-arc branch at the loop top.
            // Rotary sense is remapped per block; the next block's angle is not this one.
            let tail_still_open = acc < sweep + slack
                && !rotary_curve
                && moves.get(*cursor).is_some_and(|next| match next {
                    ReadMove::Arc { sense: got, .. } if *got == sense => {
                        let next_end = project(end_of(next), drop_z);
                        on_curve(next_end, start, end, center, plane, sense, sweep, tol) && geom::dist3(next_end, end) <= tol
                    }
                    _ => false,
                });
            if tail_still_open {
                continue;
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
    // Rounding can leave the first posted point slightly behind the programmed start.
    if geom::dist3(point, start) <= tol {
        return true;
    }
    // Rounding can also leave the last posted point slightly past the programmed end. On a small
    // radius that is more angle than the window below allows.
    if geom::dist3(point, end) <= tol {
        return true;
    }
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
        if remaining >= std::f64::consts::TAU - 0.2 {
            std::f64::consts::TAU
        } else {
            // A step that rounds slightly backwards is not travel.
            let forward = forward_from(prev, point, center, plane, sense);
            if forward <= std::f64::consts::PI { forward } else { 0.0 }
        }
    } else {
        forward_from(prev, point, center, plane, sense)
    }
}

fn forward_from(start: Xyz, point: Xyz, center: Xyz, plane: Plane, sense: ArcSense) -> f64 {
    let delta = geom::forward_delta(geom::angle_of(plane, start, center), geom::angle_of(plane, point, center), matches!(sense, ArcSense::Ccw));
    if delta > std::f64::consts::TAU - 1e-4 { 0.0 } else { delta }
}

fn is_motion(move_: &ReadMove) -> bool {
    matches!(move_, ReadMove::Rapid { .. } | ReadMove::Linear { .. } | ReadMove::Arc { .. } | ReadMove::Thread { .. })
}

fn end_of(move_: &ReadMove) -> Xyz {
    match move_ {
        ReadMove::Rapid { end, .. } | ReadMove::Linear { end, .. } | ReadMove::Arc { end, .. } | ReadMove::Thread { end, .. } => *end,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => [0.0, 0.0, 0.0],
    }
}

fn take_thread(
    moves: &[&ReadMove],
    cursor: &mut usize,
    end: Xyz,
    lead: f64,
    tol: f64,
    index: usize,
    drop_z: bool,
    post: &PostDef,
    angle: Option<f64>,
    angle_tol: f64,
    setup: Option<&RotarySetup>,
) -> Result<()> {
    let Some(mv) = moves.get(*cursor).copied() else {
        return Err(invalid(format!("motion {index}: missing g-code")));
    };
    *cursor += 1;
    let ReadMove::Thread { end: got, lead: got_lead, rotary, .. } = mv else {
        return Err(invalid(format!("motion {index}: expected a thread move")));
    };
    let got = map_tip(*got, *rotary, angle, setup, index)?;
    if let Some(expect) = angle {
        check_angle(*rotary, expect, angle_tol, index)?;
    }
    let got = project(got, drop_z);
    let end = project(end, drop_z);
    if geom::dist3(got, end) > tol {
        return Err(invalid(format!("motion {index}: end differs by {:.6} mm", geom::dist3(got, end))));
    }
    let quantum = 10_f64.powi(-i32::from(post.linear_decimals()));
    let limit = tol.max(quantum * 0.51);
    if (got_lead - lead).abs() > limit {
        return Err(invalid(format!("motion {index}: lead {got_lead} differs from {lead}")));
    }
    Ok(())
}

fn tip(pose: Pose) -> Xyz {
    let point = pose.tip();
    [point.x(), point.y(), point.z()]
}

fn pose_of(motion: &ClMotion) -> Pose {
    match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
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
    // A turning C angle is already a part-frame coordinate. There is no rotary setup to invert.
    let Some(setup) = setup else {
        return Ok(got);
    };
    let Some(angle) = got_angle else {
        return Err(invalid(format!("motion {index}: missing rotary angle")));
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
        ReadMove::Rapid { rotary, .. } | ReadMove::Linear { rotary, .. } | ReadMove::Arc { rotary, .. } | ReadMove::Thread { rotary, .. } => *rotary,
        ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
    }
}

fn move_feed(mv: &ReadMove) -> Option<f64> {
    match mv {
        ReadMove::Rapid { feed, .. } | ReadMove::Linear { feed, .. } | ReadMove::Arc { feed, .. } => *feed,
        ReadMove::Thread { .. } | ReadMove::Dwell { .. } | ReadMove::Other { .. } => None,
    }
}

fn move_inverse(mv: &ReadMove) -> bool {
    match mv {
        ReadMove::Rapid { inverse_time, .. }
        | ReadMove::Linear { inverse_time, .. }
        | ReadMove::Arc { inverse_time, .. }
        | ReadMove::Thread { inverse_time, .. } => *inverse_time,
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

#[cfg(test)]
mod tests {
    use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole, Plane, Pose, Program, Vec3};

    use crate::{builtin, read_back};

    fn tip(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    #[test]
    fn equal_arc_endpoints_cut_a_full_circle() {
        let start = [1.23456, 2.0, 0.0];
        let radius = 0.5_f64;
        let sweep = 1e-5_f64;
        let center = [start[0] - radius, start[1], start[2]];
        let end = [center[0] + radius * sweep.cos(), center[1] - radius * sweep.sin(), start[2]];
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: tip(start[0], start[1], start[2]), feed: None }),
            ClBlock::Motion(ClMotion::Arc {
                to: tip(end[0], end[1], end[2]),
                plane: Plane::Xy,
                center: Vec3::new(center[0], center[1], center[2]).expect("center"),
                sense: ArcSense::Cw,
                feed: 100.0,
                role: MoveRole::Cut,
            }),
        ];
        let feeds = Feeds::try_new(100.0, None, None, None, None).expect("feeds");
        let path = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, Vec::new(), blocks).expect("path");
        let program = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None).expect("program");
        let post = builtin("linuxcnc").expect("linuxcnc");
        let text = "G21\nG90\nG17\nG0 X1.2346 Y2.0000 Z0.0000\nG2 X1.2346 Y2.0000 Z0.0000 I-0.5000 J0.0000 F100\n";
        let moves = read_back(text, &post).expect("read");
        let err = super::compare(&program, &moves, 1e-4, &post).expect_err("full circle");
        assert!(err.to_string().contains("cuts a full circle"), "{err}");
    }
}
