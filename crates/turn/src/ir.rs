//! Cutter-location conversion for a programmed lathe path.
//!
//! Poses use a radius in X and the spindle coordinate in Z. Y is 0 on these XZ paths. The tool axis is
//! [`campanion_ir::Vec3::Z`] because a pose requires a unit axis; turning does not read it.

use campanion_ir::{ArcSense, ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Plane, Pose, Vec3};

use crate::{Result, TurnError, TurnMove, TurnPath};

/// Convert `path` into a cutter-location path.
///
/// The first block is a rapid to [`TurnPath::start`]. Each [`TurnMove::Rapid`] stays a rapid
/// with no feed. Feeds and arcs are [`MoveRole::Cut`]. A dwell is an [`campanion_ir::Event::Dwell`].
/// A thread move is [`ClMotion::Thread`] with the same lead. An arc lies in the ZX plane. Its centre
/// is `(cx, 0, cz)`. Counter-clockwise with +Z to the right and +X up is [`ArcSense::Ccw`].
/// Compensation is [`CompensationMode::Computed`]: nose compensation is already in `path`.
///
/// `rapid_clear`, when set, must be finite and `>= 0`. It does not move the poses or the rapid
/// feed. Facing, roughing and finishing already include clearance, and a rapid feed stays unset.
pub fn to_cl_path(path: &TurnPath, name: Option<String>, tool_id: u32, cut_feed: f64, rapid_clear: Option<f64>) -> Result<ClPath> {
    if let Some(clearance) = rapid_clear {
        if !clearance.is_finite() {
            return Err(TurnError::InvalidInput("rapid clearance must be finite".into()));
        }
        if clearance < 0.0 {
            return Err(TurnError::InvalidInput("rapid clearance must be >= 0".into()));
        }
    }
    let feeds = Feeds::try_new(cut_feed, None, None, None, None)?;
    let mut blocks = Vec::with_capacity(path.moves.len().saturating_add(1));
    blocks.push(ClBlock::Motion(ClMotion::Rapid { to: pose_at(path.start.0, path.start.1)?, feed: None }));
    for motion in &path.moves {
        blocks.push(cl_block(*motion, cut_feed)?);
    }
    Ok(ClPath::try_new(name, Some(tool_id), CompensationMode::Computed, feeds, None, Vec::new(), blocks)?)
}

fn pose_at(z: f64, x_radius: f64) -> Result<Pose> {
    Ok(Pose::new(Vec3::new(x_radius, 0.0, z)?, Vec3::Z)?)
}

fn cl_block(motion: TurnMove, cut_feed: f64) -> Result<ClBlock> {
    Ok(match motion {
        TurnMove::Dwell { seconds } => ClBlock::Event(Event::Dwell { seconds }),
        TurnMove::Thread { z, x, lead } => ClBlock::Motion(ClMotion::Thread { to: pose_at(z, x)?, lead }),
        other => ClBlock::Motion(cl_move(other, cut_feed)?),
    })
}

fn cl_move(motion: TurnMove, cut_feed: f64) -> Result<ClMotion> {
    Ok(match motion {
        TurnMove::Rapid { z, x } => ClMotion::Rapid { to: pose_at(z, x)?, feed: None },
        TurnMove::Feed { z, x } => ClMotion::Linear { to: pose_at(z, x)?, feed: cut_feed, role: MoveRole::Cut },
        TurnMove::Arc { z, x, cz, cx, ccw } => ClMotion::Arc {
            to: pose_at(z, x)?,
            plane: Plane::Zx,
            center: Vec3::new(cx, 0.0, cz)?,
            sense: if ccw { ArcSense::Ccw } else { ArcSense::Cw },
            feed: cut_feed,
            role: MoveRole::Cut,
        },
        TurnMove::Thread { .. } | TurnMove::Dwell { .. } => {
            return Err(TurnError::InvalidInput("not a feed or arc move".into()));
        }
    })
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use campanion_geom::{Arc, Line, Point, Segment};
    use campanion_ir::{ArcSense, ClBlock, ClMotion, CompensationMode, MoveRole, Plane, Vec3};

    use super::to_cl_path;
    use crate::{FaceParams, FinishParams, RoughParams, Side, TipPoint, TurnError, TurnMove, TurnPath, TurnProfile, TurnTool, face, finish, rough};

    fn sample() -> TurnPath {
        TurnPath {
            start: (1.0, 10.0),
            moves: vec![
                TurnMove::Rapid { z: 0.0, x: 12.0 },
                TurnMove::Feed { z: -5.0, x: 12.0 },
                TurnMove::Arc { z: -6.0, x: 11.0, cz: -5.0, cx: 11.0, ccw: true },
                TurnMove::Arc { z: -7.0, x: 10.0, cz: -7.0, cx: 11.0, ccw: false },
            ],
        }
    }

    fn tip_of(block: &ClBlock) -> (f64, f64, f64) {
        let ClBlock::Motion(motion) = block else { panic!("event in a converted path") };
        let pose = match motion {
            ClMotion::Rapid { to, .. }
            | ClMotion::Linear { to, .. }
            | ClMotion::Arc { to, .. }
            | ClMotion::Helix { to, .. }
            | ClMotion::Thread { to, .. } => to,
        };
        let tip = pose.tip();
        assert_eq!(pose.axis(), Vec3::Z);
        assert_eq!(tip.y(), 0.0);
        (tip.z(), tip.x(), tip.y())
    }

    #[test]
    fn conversion_keeps_endpoints_and_arc_senses() {
        let path = sample();
        let cl = to_cl_path(&path, Some("turn".into()), 7, 80.0, None).expect("convert");
        assert_eq!(cl.name(), Some("turn"));
        assert_eq!(cl.tool_id(), Some(7));
        assert_eq!(cl.compensation(), CompensationMode::Computed);
        assert_eq!(cl.feeds().cut(), 80.0);
        assert!(cl.feeds().lead_in().is_none());
        assert_eq!(cl.blocks().len(), path.moves.len() + 1);

        let Some(ClBlock::Motion(ClMotion::Rapid { to, feed: None })) = cl.blocks().first() else {
            panic!("first block is not a rapid without a feed");
        };
        assert_eq!(to.tip().z(), path.start.0);
        assert_eq!(to.tip().x(), path.start.1);

        let Some(ClBlock::Motion(ClMotion::Rapid { to, feed: None })) = cl.blocks().get(1) else {
            panic!("rapid move");
        };
        assert_eq!((to.tip().z(), to.tip().x()), (0.0, 12.0));

        let Some(ClBlock::Motion(ClMotion::Linear { to, feed, role })) = cl.blocks().get(2) else {
            panic!("feed move");
        };
        assert_eq!((to.tip().z(), to.tip().x()), (-5.0, 12.0));
        assert_eq!(*feed, 80.0);
        assert_eq!(*role, MoveRole::Cut);

        let Some(ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, feed, role })) = cl.blocks().get(3) else {
            panic!("ccw arc");
        };
        assert_eq!(*plane, Plane::Zx);
        assert_eq!(*sense, ArcSense::Ccw);
        assert_eq!(*role, MoveRole::Cut);
        assert_eq!(*feed, 80.0);
        assert_eq!((center.x(), center.y(), center.z()), (11.0, 0.0, -5.0));
        assert_eq!((to.tip().z(), to.tip().x()), (-6.0, 11.0));

        let Some(ClBlock::Motion(ClMotion::Arc { to, plane, center, sense, feed, role })) = cl.blocks().get(4) else {
            panic!("cw arc");
        };
        assert_eq!(*plane, Plane::Zx);
        assert_eq!(*sense, ArcSense::Cw);
        assert_eq!(*role, MoveRole::Cut);
        assert_eq!(*feed, 80.0);
        assert_eq!((center.x(), center.y(), center.z()), (11.0, 0.0, -7.0));
        assert_eq!((to.tip().z(), to.tip().x()), (-7.0, 10.0));

        let ends: Vec<_> = cl.blocks().iter().map(tip_of).collect();
        assert_eq!(ends, vec![(1.0, 10.0, 0.0), (0.0, 12.0, 0.0), (-5.0, 12.0, 0.0), (-6.0, 11.0, 0.0), (-7.0, 10.0, 0.0)]);
    }

    #[test]
    fn rapid_clearance_does_not_move_the_path() {
        let path = sample();
        let plain = to_cl_path(&path, None, 1, 100.0, None).expect("none");
        let cleared = to_cl_path(&path, None, 1, 100.0, Some(2.0)).expect("clear");
        let zero = to_cl_path(&path, None, 1, 100.0, Some(0.0)).expect("zero");
        assert_eq!(plain, cleared);
        assert_eq!(plain, zero);
    }

    #[test]
    fn rapid_clearance_rejects_non_finite_and_negative() {
        let path = sample();
        let nan = to_cl_path(&path, None, 1, 100.0, Some(f64::NAN)).expect_err("nan");
        assert!(matches!(nan, TurnError::InvalidInput(ref message) if message.contains("finite")), "{nan}");
        let negative = to_cl_path(&path, None, 1, 100.0, Some(-1.0)).expect_err("negative");
        assert!(matches!(negative, TurnError::InvalidInput(ref message) if message.contains(">=")), "{negative}");
    }

    #[test]
    fn ir_errors_use_the_ir_variant() {
        let path = sample();
        let feed = to_cl_path(&path, None, 1, 0.0, None).expect_err("feed");
        assert!(matches!(feed, TurnError::Ir(_)), "{feed}");
        let broken = TurnPath { start: (f64::NAN, 0.0), moves: Vec::new() };
        let coordinate = to_cl_path(&broken, None, 1, 100.0, None).expect_err("coordinate");
        assert!(matches!(coordinate, TurnError::Ir(_)), "{coordinate}");
    }

    #[test]
    fn finish_fillet_keeps_clockwise_sense() {
        let nose = 0.8;
        let centre_z = -15.0;
        let centre_x = 15.0;
        let fillet = Arc::new(Point::new(centre_z, centre_x).expect("centre"), 3.0, 0.0, -PI).expect("fillet");
        let start = fillet.start_point();
        let end = fillet.end_point();
        let profile = TurnProfile::try_new(
            vec![
                Segment::Line(Line::new(Point::new(0.0, centre_x).expect("a"), start).expect("lead")),
                Segment::Arc(fillet),
                Segment::Line(Line::new(end, Point::new(-30.0, centre_x).expect("b")).expect("trail")),
            ],
            1e-6,
        )
        .expect("profile");
        let tool = TurnTool { nose_radius: nose, tip: TipPoint::Centre };
        let params = FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 0.5, tol: 1e-6 };
        let turned = finish(&profile, &tool, &params).expect("finish");
        let cl = to_cl_path(&turned, Some("finish".into()), 1, 120.0, Some(0.5)).expect("convert");
        let mut matched = false;
        for block in cl.blocks() {
            if let ClBlock::Motion(ClMotion::Arc { center, sense, .. }) = block {
                let radius = (center.z() - centre_z).hypot(center.x() - centre_x);
                if radius < 1e-4 {
                    assert_eq!(*sense, ArcSense::Cw);
                    matched = true;
                }
            }
        }
        assert!(matched, "clockwise fillet was not converted");
    }

    #[test]
    fn example_shaft_imports_and_turns() {
        let dxf_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/shaft.dxf");
        let text = std::fs::read_to_string(&dxf_path).unwrap_or_else(|err| panic!("read {}: {err}", dxf_path.display()));
        let profile = TurnProfile::from_dxf(&text, Some("PROFILE"), 1e-6).expect("import");
        let (zmin, zmax) = profile.z_range();
        assert!((zmax - 0.0).abs() < 1e-6, "zmax {zmax}");
        assert!((zmin + 60.0).abs() < 1e-6, "zmin {zmin}");
        assert!((profile.max_radius() - 20.0).abs() < 1e-6, "max radius {}", profile.max_radius());
        assert!(profile.radius_at(-10.0).is_some_and(|radius| (radius - 15.0).abs() < 1e-6), "journal");
        assert!(profile.radius_at(-50.0).is_some_and(|radius| (radius - 20.0).abs() < 1e-6), "rear diameter");
        let tool = TurnTool { nose_radius: 0.8, tip: TipPoint::Centre };
        let faced = face(&tool, &FaceParams { stock_front_z: 1.0, final_z: 0.0, step_z: 0.5, from_radius: 25.0, to_radius: 0.0, clearance: 2.0 })
            .expect("face");
        let roughed = rough(
            &profile,
            &tool,
            &RoughParams {
                side: Side::Outside,
                stock_outer_radius: 25.0,
                stock_inner_radius: 0.0,
                depth_of_cut: 2.0,
                leave_x: 0.3,
                leave_z: 0.1,
                clearance: 2.0,
                retract: 2.0,
            },
        )
        .expect("rough");
        let finished =
            finish(&profile, &tool, &FinishParams { side: Side::Outside, leave_x: 0.0, leave_z: 0.0, clearance: 2.0, tol: 0.001 }).expect("finish");
        for (name, turned) in [("face", &faced), ("rough", &roughed), ("finish", &finished)] {
            let cl = to_cl_path(turned, Some(name.into()), 1, 100.0, Some(2.0)).unwrap_or_else(|err| panic!("{name}: {err}"));
            assert!(matches!(cl.blocks().first(), Some(ClBlock::Motion(ClMotion::Rapid { feed: None, .. }))), "{name}");
        }
    }

    #[test]
    fn thread_and_dwell_become_synchronised_motion_and_an_event() {
        let path = TurnPath { start: (1.0, 12.0), moves: vec![TurnMove::Thread { z: -8.0, x: 9.0, lead: 2.5 }, TurnMove::Dwell { seconds: 0.2 }] };
        let cl = to_cl_path(&path, Some("thread".into()), 3, 100.0, None).expect("convert");
        let Some(ClBlock::Motion(ClMotion::Thread { to, lead })) = cl.blocks().get(1) else {
            panic!("thread block missing: {:?}", cl.blocks());
        };
        assert_eq!(to.tip().z(), -8.0);
        assert_eq!(to.tip().x(), 9.0);
        assert_eq!(to.tip().y(), 0.0);
        assert_eq!(*lead, 2.5);
        assert!(matches!(cl.blocks().get(2), Some(ClBlock::Event(campanion_ir::Event::Dwell { seconds })) if *seconds == 0.2));
    }

    /// M20×2.5 outside thread, four constant-area passes, fixed at 800 rpm.
    ///
    /// `UPDATE_THREAD_IR=1` rewrites `crates/post/tests/data/turn_thread.ir.json`.
    #[test]
    fn m20_thread_ir_matches_the_post_fixture() {
        use campanion_ir::{Document, Program, SpindleSpeed, Tool, TurnBinding, TurnStock, TurningSetup};

        let depth = 0.6134 * 2.5;
        let params = crate::ThreadParams {
            side: Side::Outside,
            major_radius: 10.0,
            depth,
            pitch: 2.5,
            z_start: 0.0,
            z_end: -16.0,
            first_depth: depth / 2.0,
            min_depth: 0.05,
            spring_passes: 0,
            infeed: crate::Infeed::Flank { angle_deg: 29.5 },
            clearance: 1.0,
            included_angle_deg: 60.0,
        };
        let turned = crate::thread(&params).expect("thread");
        let threads = turned.moves.iter().filter(|motion| matches!(motion, TurnMove::Thread { lead, .. } if *lead == 2.5)).count();
        assert_eq!(threads, 4, "expected four passes, got {threads}");
        let motion = to_cl_path(&turned, Some("thread".into()), 1, 100.0, Some(1.0)).expect("cl");
        let stock = TurnStock::bar(20.0, 30.0).expect("stock");
        let binding =
            TurnBinding::try_new(0, "main", "turret1", 0).expect("binding").with_speed(Some(SpindleSpeed::rpm(800.0).expect("rpm"))).expect("speed");
        let turning = TurningSetup::single(Some(stock), 3_000.0).expect("setup").with_binding(binding).expect("bound");
        let tool = Tool::try_new(1, Some("thread".into()), None, None).expect("tool");
        let program = Program::try_new(Some("turn_thread".into()), vec![tool], None, vec![motion], Vec::new(), Vec::new(), None)
            .expect("program")
            .with_turning(Some(turning))
            .expect("turning");
        let doc = Document::new(program).expect("document");
        let mut pretty = doc.to_json_pretty().expect("json");
        pretty.push('\n');
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../post/tests/data/turn_thread.ir.json");
        if std::env::var("UPDATE_THREAD_IR").ok().as_deref() == Some("1") {
            std::fs::write(&path, &pretty).unwrap_or_else(|err| panic!("write {}: {err}", path.display()));
        }
        let committed = std::fs::read_to_string(&path).unwrap_or_else(|err| panic!("read {}: {err}", path.display()));
        assert_eq!(committed, pretty, "turn_thread.ir.json drifted; rerun with UPDATE_THREAD_IR=1");
        // serde_json does not bit-preserve every f64, so compare the loaded program by its thread moves.
        let loaded = Document::from_json(&committed).unwrap_or_else(|err| panic!("load {}: {err}", path.display()));
        let loaded_threads: Vec<f64> = loaded.program().toolpaths()[0]
            .blocks()
            .iter()
            .filter_map(|block| match block {
                ClBlock::Motion(ClMotion::Thread { lead, .. }) => Some(*lead),
                _ => None,
            })
            .collect();
        assert_eq!(loaded_threads, vec![2.5, 2.5, 2.5, 2.5]);
        let speed = loaded.program().turning().and_then(|turning| turning.binding_for(0)).and_then(|binding| binding.speed());
        assert!(matches!(speed, Some(SpindleSpeed::Rpm { rpm }) if rpm == 800.0));
    }
}
