//! Per-operation process timing.
//!
//! [`apply_timing`] rewrites every process start and every process stop. That includes a stop
//! and the later start around a tab, and the start and stop that sit with a lead-in and a
//! lead-out. Heights are absolute Z. A delay of zero emits no dwell.

use crate::Result;
use crate::check::{self, finite, non_negative};
use crate::toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};

/// Delays and heights applied at each process start and each process stop.
///
/// Delays are seconds. Heights are absolute Z coordinates in millimetres; `None` does not add a
/// move. The pierce delay defaults to running with the process on. The post delay defaults to
/// running with the process off.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProcessTiming {
    pierce_delay_s: f64,
    pierce_process_on_during_delay: bool,
    post_delay_s: f64,
    post_process_on_during_delay: bool,
    pierce_height: Option<f64>,
    cut_height: Option<f64>,
}

impl ProcessTiming {
    /// Pierce delay keeps the process on. Post delay runs after the process turns off.
    pub fn new(pierce_delay_s: f64, post_delay_s: f64) -> Self {
        Self {
            pierce_delay_s,
            pierce_process_on_during_delay: true,
            post_delay_s,
            post_process_on_during_delay: false,
            pierce_height: None,
            cut_height: None,
        }
    }

    /// When true, the process is already on during [`Self::pierce_delay_s`].
    pub fn pierce_process_on_during_delay(mut self, on: bool) -> Self {
        self.pierce_process_on_during_delay = on;
        self
    }

    /// When true, the process stays on during [`Self::post_delay_s`] and turns off afterwards.
    pub fn post_process_on_during_delay(mut self, on: bool) -> Self {
        self.post_process_on_during_delay = on;
        self
    }

    /// Rapid to this Z before the pierce delay. `z` is absolute, in millimetres.
    pub fn pierce_height(mut self, z: f64) -> Self {
        self.pierce_height = Some(z);
        self
    }

    /// Feed straight down or up to this Z after the pierce, before the cut resumes.
    pub fn cut_height(mut self, z: f64) -> Self {
        self.cut_height = Some(z);
        self
    }

    /// Pierce delay in seconds, before validation.
    pub fn pierce_delay_s(self) -> f64 {
        self.pierce_delay_s
    }

    /// Post delay in seconds, before validation.
    pub fn post_delay_s(self) -> f64 {
        self.post_delay_s
    }

    /// Absolute pierce Z, in millimetres, when a rapid to that height was requested.
    pub fn pierce_height_z(self) -> Option<f64> {
        self.pierce_height
    }

    /// Absolute cut Z, in millimetres, when a plunge to that height was requested.
    pub fn cut_height_z(self) -> Option<f64> {
        self.cut_height
    }
}

impl Default for ProcessTiming {
    fn default() -> Self {
        Self::new(0.0, 0.0)
    }
}

/// Rewrite each process start and stop in `path`.
///
/// A start, in order, is: rapid to the pierce height when one is set; then either process on
/// followed by the pierce dwell, or the dwell followed by process on; then a plunge to the cut
/// height when one is set. The lead-in and the cut that already followed the start stay after
/// that sequence.
///
/// A stop, in order, is: either the post dwell then process off, or process off then the dwell.
/// A retract or rapid that already followed the stop stays after that sequence.
///
/// A delay of zero emits no [`Move::Dwell`]. Delays must be finite and >= 0. Heights must be finite.
pub fn apply_timing(path: &Toolpath, timing: &ProcessTiming) -> Result<Toolpath> {
    validate(timing)?;
    let mut out = Vec::with_capacity(path.moves().len());
    let mut cursor = None;
    for mv in path.moves() {
        match *mv {
            Move::Event(ProcessEvent::On) => emit_start(&mut out, &mut cursor, timing)?,
            Move::Event(ProcessEvent::Off) => emit_stop(&mut out, timing)?,
            other => {
                if let Some(to) = endpoint(other) {
                    cursor = Some(to);
                }
                check::push(&mut out, other)?;
            }
        }
    }
    Toolpath::new(out)
}

fn validate(timing: &ProcessTiming) -> Result<()> {
    non_negative(timing.pierce_delay_s, "pierce delay")?;
    non_negative(timing.post_delay_s, "post delay")?;
    if let Some(z) = timing.pierce_height {
        finite(z, "pierce height")?;
    }
    if let Some(z) = timing.cut_height {
        finite(z, "cut height")?;
    }
    Ok(())
}

fn emit_start(out: &mut Vec<Move>, cursor: &mut Option<Point3>, timing: &ProcessTiming) -> Result<()> {
    if let Some(z) = timing.pierce_height {
        vertical(out, cursor, z, true, "pierce height")?;
    }
    if timing.pierce_process_on_during_delay {
        check::push(out, Move::Event(ProcessEvent::On))?;
        emit_dwell(out, timing.pierce_delay_s)?;
    } else {
        emit_dwell(out, timing.pierce_delay_s)?;
        check::push(out, Move::Event(ProcessEvent::On))?;
    }
    if let Some(z) = timing.cut_height {
        vertical(out, cursor, z, false, "cut height")?;
    }
    Ok(())
}

fn emit_stop(out: &mut Vec<Move>, timing: &ProcessTiming) -> Result<()> {
    if timing.post_process_on_during_delay {
        emit_dwell(out, timing.post_delay_s)?;
        check::push(out, Move::Event(ProcessEvent::Off))?;
    } else {
        check::push(out, Move::Event(ProcessEvent::Off))?;
        emit_dwell(out, timing.post_delay_s)?;
    }
    Ok(())
}

fn emit_dwell(out: &mut Vec<Move>, seconds: f64) -> Result<()> {
    if seconds > 0.0 {
        check::push(out, Move::Dwell { seconds })?;
    }
    Ok(())
}

fn vertical(out: &mut Vec<Move>, cursor: &mut Option<Point3>, z: f64, rapid: bool, what: &str) -> Result<()> {
    let Some(at) = *cursor else {
        return Err(check::invalid(format!("{what} needs a position")));
    };
    let to = Point3::new(at.x(), at.y(), z)?;
    if rapid {
        check::push(out, Move::Rapid { to })?;
    } else {
        check::push(out, Move::Linear { to, kind: CutKind::Plunge })?;
    }
    *cursor = Some(to);
    Ok(())
}

fn endpoint(mv: Move) -> Option<Point3> {
    match mv {
        Move::Rapid { to } | Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
        Move::Event(_) | Move::Dwell { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{ProcessTiming, apply_timing};
    use crate::PathError;
    use crate::profile::{CutDirection, Lead, ProfileOp, ProfileSide, TabSpec};
    use crate::tool::{Compensation, ControllerSide, Tool};
    use crate::toolpath::{CutKind, Move, Point3, ProcessEvent, Toolpath};
    use campanion_geom::{Contour, Line, Point, Segment, Tolerance};

    const PIERCE: f64 = 0.25;
    const POST: f64 = 0.5;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z).unwrap()
    }

    fn sample() -> Toolpath {
        Toolpath::new(vec![
            Move::Rapid { to: p(0.0, 0.0, 5.0) },
            Move::Event(ProcessEvent::On),
            Move::Linear { to: p(4.0, 0.0, 0.0), kind: CutKind::LeadIn },
            Move::Linear { to: p(8.0, 0.0, 0.0), kind: CutKind::Cut },
            Move::Linear { to: p(10.0, 0.0, 0.0), kind: CutKind::LeadOut },
            Move::Event(ProcessEvent::Off),
            Move::Linear { to: p(10.0, 0.0, 5.0), kind: CutKind::Retract },
            Move::Rapid { to: p(20.0, 1.0, 5.0) },
            Move::Event(ProcessEvent::On),
            Move::Linear { to: p(24.0, 1.0, 0.0), kind: CutKind::Cut },
            Move::Event(ProcessEvent::Off),
            Move::Rapid { to: p(24.0, 1.0, 5.0) },
        ])
        .unwrap()
    }

    fn coord(to: Point3) -> String {
        format!("({:.4},{:.4},{:.4})", to.x(), to.y(), to.z())
    }

    fn kind_name(kind: CutKind) -> &'static str {
        match kind {
            CutKind::Cut => "cut",
            CutKind::LeadIn => "lead-in",
            CutKind::LeadOut => "lead-out",
            CutKind::Plunge => "plunge",
            CutKind::Retract => "retract",
        }
    }

    fn trace(path: &Toolpath) -> String {
        path.moves()
            .iter()
            .map(|mv| match *mv {
                Move::Rapid { to } => format!("rapid{}", coord(to)),
                Move::Linear { to, kind } => format!("{}{}", kind_name(kind), coord(to)),
                Move::Arc { to, kind, .. } => format!("arc-{}{}", kind_name(kind), coord(to)),
                Move::Event(ProcessEvent::On) => "on".to_string(),
                Move::Event(ProcessEvent::Off) => "off".to_string(),
                Move::Dwell { seconds } => format!("dwell({seconds:.4})"),
            })
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn four_toggle_combinations_keep_this_event_order() {
        // Defaults are pierce-on-during and post-off-during. The other three rows set the flags.
        // Two starts and two stops: the first stop is followed by a retract, the second by a rapid.
        let hand = [
            (
                true,
                false,
                concat!(
                    "rapid(0.0000,0.0000,5.0000) on dwell(0.2500) ",
                    "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                    "off dwell(0.5000) retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                    "on dwell(0.2500) cut(24.0000,1.0000,0.0000) off dwell(0.5000) rapid(24.0000,1.0000,5.0000)"
                ),
            ),
            (
                true,
                true,
                concat!(
                    "rapid(0.0000,0.0000,5.0000) on dwell(0.2500) ",
                    "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                    "dwell(0.5000) off retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                    "on dwell(0.2500) cut(24.0000,1.0000,0.0000) dwell(0.5000) off rapid(24.0000,1.0000,5.0000)"
                ),
            ),
            (
                false,
                false,
                concat!(
                    "rapid(0.0000,0.0000,5.0000) dwell(0.2500) on ",
                    "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                    "off dwell(0.5000) retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                    "dwell(0.2500) on cut(24.0000,1.0000,0.0000) off dwell(0.5000) rapid(24.0000,1.0000,5.0000)"
                ),
            ),
            (
                false,
                true,
                concat!(
                    "rapid(0.0000,0.0000,5.0000) dwell(0.2500) on ",
                    "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                    "dwell(0.5000) off retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                    "dwell(0.2500) on cut(24.0000,1.0000,0.0000) dwell(0.5000) off rapid(24.0000,1.0000,5.0000)"
                ),
            ),
        ];
        let length = sample().cutting_length().unwrap();
        for (pierce_on, post_on, want) in hand {
            let timing = if pierce_on && !post_on {
                ProcessTiming::new(PIERCE, POST)
            } else {
                ProcessTiming::new(PIERCE, POST).pierce_process_on_during_delay(pierce_on).post_process_on_during_delay(post_on)
            };
            let got = apply_timing(&sample(), &timing).unwrap();
            assert_eq!(trace(&got), want, "pierce_on_during={pierce_on} post_on_during={post_on}");
            assert!((got.cutting_length().unwrap() - length).abs() < 1e-9);
        }
    }

    #[test]
    fn zero_delay_emits_no_dwell() {
        let path = sample();
        for pierce_on in [true, false] {
            for post_on in [true, false] {
                let timing = ProcessTiming::new(0.0, 0.0).pierce_process_on_during_delay(pierce_on).post_process_on_during_delay(post_on);
                let timed = apply_timing(&path, &timing).unwrap();
                assert_eq!(timed.moves(), path.moves());
                assert!(timed.moves().iter().all(|mv| !matches!(mv, Move::Dwell { .. })));
            }
        }
        let negative_zero = apply_timing(&path, &ProcessTiming::new(-0.0, -0.0)).unwrap();
        assert_eq!(negative_zero.moves(), path.moves());

        let post_only = apply_timing(&path, &ProcessTiming::new(0.0, POST)).unwrap();
        assert_eq!(
            trace(&post_only),
            concat!(
                "rapid(0.0000,0.0000,5.0000) on ",
                "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                "off dwell(0.5000) retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                "on cut(24.0000,1.0000,0.0000) off dwell(0.5000) rapid(24.0000,1.0000,5.0000)"
            )
        );

        let with_heights = apply_timing(&path, &ProcessTiming::new(0.0, 0.0).pierce_height(3.0).cut_height(1.0)).unwrap();
        assert!(with_heights.moves().iter().all(|mv| !matches!(mv, Move::Dwell { .. })));
    }

    #[test]
    fn heights_appear_as_z() {
        let timing = ProcessTiming::new(0.2, 0.1).pierce_height(3.5).cut_height(-1.25);
        let got = apply_timing(&sample(), &timing).unwrap();
        assert_eq!(
            trace(&got),
            concat!(
                "rapid(0.0000,0.0000,5.0000) rapid(0.0000,0.0000,3.5000) on dwell(0.2000) plunge(0.0000,0.0000,-1.2500) ",
                "lead-in(4.0000,0.0000,0.0000) cut(8.0000,0.0000,0.0000) lead-out(10.0000,0.0000,0.0000) ",
                "off dwell(0.1000) retract(10.0000,0.0000,5.0000) rapid(20.0000,1.0000,5.0000) ",
                "rapid(20.0000,1.0000,3.5000) on dwell(0.2000) plunge(20.0000,1.0000,-1.2500) ",
                "cut(24.0000,1.0000,0.0000) off dwell(0.1000) rapid(24.0000,1.0000,5.0000)"
            )
        );

        // The second start is still at the arc end, not at the arc centre.
        let arc = Toolpath::new(vec![
            Move::Rapid { to: p(2.0, 0.0, 1.0) },
            Move::Event(ProcessEvent::On),
            Move::Arc { to: p(0.0, 2.0, 1.0), centre: p(0.0, 0.0, 1.0), ccw: true, kind: CutKind::Cut },
            Move::Event(ProcessEvent::Off),
            Move::Event(ProcessEvent::On),
            Move::Linear { to: p(0.0, 4.0, 1.0), kind: CutKind::Cut },
            Move::Event(ProcessEvent::Off),
        ])
        .unwrap();
        let timed = apply_timing(&arc, &ProcessTiming::new(0.0, 0.0).pierce_height(4.0).cut_height(1.5)).unwrap();
        assert_eq!(
            trace(&timed),
            concat!(
                "rapid(2.0000,0.0000,1.0000) rapid(2.0000,0.0000,4.0000) on plunge(2.0000,0.0000,1.5000) ",
                "arc-cut(0.0000,2.0000,1.0000) off ",
                "rapid(0.0000,2.0000,4.0000) on plunge(0.0000,2.0000,1.5000) ",
                "cut(0.0000,4.0000,1.0000) off"
            )
        );
        assert!(timed.moves().iter().all(|mv| !matches!(mv, Move::Dwell { .. })));
    }

    #[test]
    fn non_finite_and_negative_values_are_errors() {
        let path = Toolpath::new(vec![]).unwrap();
        let cases = [
            ProcessTiming::new(f64::NAN, 0.0),
            ProcessTiming::new(0.0, f64::NAN),
            ProcessTiming::new(f64::INFINITY, 0.0),
            ProcessTiming::new(0.0, f64::NEG_INFINITY),
            ProcessTiming::new(-0.01, 0.0),
            ProcessTiming::new(0.0, -1.0),
            ProcessTiming::new(0.0, 0.0).pierce_height(f64::NAN),
            ProcessTiming::new(0.0, 0.0).cut_height(f64::NAN),
            ProcessTiming::new(0.0, 0.0).pierce_height(f64::INFINITY),
            ProcessTiming::new(0.0, 0.0).cut_height(f64::NEG_INFINITY),
        ];
        for timing in cases {
            let err = apply_timing(&path, &timing).expect_err("hostile timing must fail");
            assert!(matches!(err, PathError::InvalidInput(_)), "{err:?}");
            let text = err.to_string();
            assert!(text.contains("finite") || text.contains(">="), "{text}");
        }

        let bare = Toolpath::new(vec![Move::Event(ProcessEvent::On)]).unwrap();
        let err = apply_timing(&bare, &ProcessTiming::new(0.0, 0.0).pierce_height(1.0)).unwrap_err();
        assert!(err.to_string().contains("position"), "{err}");
        let err = apply_timing(&bare, &ProcessTiming::new(0.0, 0.0).cut_height(1.0)).unwrap_err();
        assert!(err.to_string().contains("position"), "{err}");
    }

    fn rect(w: f64, h: f64) -> Contour {
        let point = |x, y| Point::new(x, y).unwrap();
        let edge = |a, b| Segment::Line(Line::new(a, b).unwrap());
        Contour::try_new(
            vec![
                edge(point(0.0, 0.0), point(w, 0.0)),
                edge(point(w, 0.0), point(w, h)),
                edge(point(w, h), point(0.0, h)),
                edge(point(0.0, h), point(0.0, 0.0)),
            ],
            true,
            &Tolerance::default(),
        )
        .unwrap()
    }

    #[test]
    fn profile_with_three_tabs_and_leads_times_every_start_and_stop() {
        let profile = ProfileOp::new(
            Tool::Diameter { diameter: 2.0 },
            Compensation::Controller(ControllerSide::Left),
            ProfileSide::On,
            CutDirection::Conventional,
        )
        .lead_in(Lead::Line { length: 5.0 })
        .lead_out(Lead::Line { length: 4.0 })
        .tabs(TabSpec::count(3, 2.0).height(1.5))
        .generate(&rect(40.0, 20.0))
        .unwrap();
        assert_eq!(profile.tabs().len(), 3);
        let source = profile.toolpath();
        let starts = source.moves().iter().filter(|mv| matches!(mv, Move::Event(ProcessEvent::On))).count();
        let stops = source.moves().iter().filter(|mv| matches!(mv, Move::Event(ProcessEvent::Off))).count();
        assert!(starts >= 1 && stops >= 1, "starts {starts}, stops {stops}");

        let pierce_z = 6.0;
        let cut_z = 0.8;
        let pierce_delay = 0.4;
        let post_delay = 0.2;
        let timed = apply_timing(source, &ProcessTiming::new(pierce_delay, post_delay).pierce_height(pierce_z).cut_height(cut_z)).unwrap();
        let moves = timed.moves();

        let timed_starts = moves.iter().filter(|mv| matches!(mv, Move::Event(ProcessEvent::On))).count();
        let timed_stops = moves.iter().filter(|mv| matches!(mv, Move::Event(ProcessEvent::Off))).count();
        assert_eq!(timed_starts, starts);
        assert_eq!(timed_stops, stops);

        let mut seen_starts = 0;
        for (i, mv) in moves.iter().enumerate() {
            if !matches!(mv, Move::Event(ProcessEvent::On)) {
                continue;
            }
            seen_starts += 1;
            let Move::Rapid { to: at } = moves[i - 1] else {
                panic!("start {i} should rapid to pierce height, got {:?}", moves[i - 1]);
            };
            assert_eq!(at.z(), pierce_z);
            let Move::Dwell { seconds } = moves[i + 1] else {
                panic!("start {i} should dwell with the process on, got {:?}", moves[i + 1]);
            };
            assert_eq!(seconds, pierce_delay);
            let Move::Linear { to, kind: CutKind::Plunge } = moves[i + 2] else {
                panic!("start {i} should drop to cut height, got {:?}", moves[i + 2]);
            };
            assert_eq!(to.z(), cut_z);
            assert_eq!((to.x(), to.y()), (at.x(), at.y()));
        }
        assert_eq!(seen_starts, starts);

        let mut seen_stops = 0;
        for (i, mv) in moves.iter().enumerate() {
            if !matches!(mv, Move::Event(ProcessEvent::Off)) {
                continue;
            }
            seen_stops += 1;
            let Move::Dwell { seconds } = moves[i + 1] else {
                panic!("stop {i} should dwell after process off, got {:?}", moves.get(i + 1));
            };
            assert_eq!(seconds, post_delay);
        }
        assert_eq!(seen_stops, stops);

        let on_at = moves.iter().position(|mv| matches!(mv, Move::Event(ProcessEvent::On))).expect("start");
        let Move::Rapid { to } = moves[on_at - 1] else {
            panic!("pierce rapid missing");
        };
        assert_eq!(to.x(), profile.pierce().x());
        assert_eq!(to.y(), profile.pierce().y());
        assert!(moves.iter().skip(on_at).any(|mv| matches!(mv, Move::Linear { kind: CutKind::LeadIn, .. })), "lead-in follows the start");

        let off_at = moves.iter().rposition(|mv| matches!(mv, Move::Event(ProcessEvent::Off))).expect("stop");
        assert!(matches!(moves[off_at - 1], Move::Linear { kind: CutKind::LeadOut, .. }));
        assert!(matches!(moves[off_at + 1], Move::Dwell { seconds } if seconds == post_delay));
        assert!(matches!(moves.last(), Some(Move::Rapid { to }) if (to.z() - crate::DEFAULT_CLEARANCE_MM).abs() < 1e-12));

        let lifts = moves.iter().filter(|mv| matches!(mv, Move::Linear { kind: CutKind::Retract, to } if to.z() == 1.5)).count();
        assert!(lifts >= 3, "at least one retract onto each tab, got {lifts}");
        let dwells = moves.iter().filter(|mv| matches!(mv, Move::Dwell { .. })).count();
        assert_eq!(dwells, starts + stops);
        assert!((timed.cutting_length().unwrap() - source.cutting_length().unwrap()).abs() < 1e-6);
    }
}
