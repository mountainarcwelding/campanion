//! Tool-aware replay for `verify --mesh` without a global cutter override.

use std::collections::BTreeMap;

use campanion_ir::{Event, Tool, ToolShape};

use super::*;

struct ToolCuts {
    cutter: Cutter,
    ranges: Vec<std::ops::Range<usize>>,
}

pub(crate) fn check_program_tools(program: &Program, mesh: &Mesh, tolerance: f64) -> Result<Vec<GougeNote>, CliError> {
    let sample_step = sample_step_for(tolerance)?;
    let tools: BTreeMap<u32, &Tool> = program.tools().iter().map(|tool| (tool.id(), tool)).collect();
    let setup = program.rotary();
    let mut active_tool = None;
    let mut notes = Vec::new();
    for (index, path) in program.toolpaths().iter().enumerate() {
        let name = path_label(path, index);
        // A path declares its initial tool. Explicit changes govern subsequent moves,
        // and a path without a declaration inherits the previously active tool.
        if let Some(id) = path.tool_id() {
            active_tool = Some(id);
        }
        let mut cursor = None;
        let mut groups: BTreeMap<u32, ToolCuts> = BTreeMap::new();
        // All tools share the original per-path sampling cap.
        let mut segments = Vec::new();
        for block in path.blocks() {
            if let ClBlock::Event(Event::ToolChange { tool_id }) = block {
                active_tool = Some(*tool_id);
            }
            let ClBlock::Motion(motion) = block else { continue };
            if !is_cut(motion) {
                cursor = Some(motion_pose(motion));
                continue;
            }
            let id = active_tool.ok_or_else(|| CliError::run(format!("toolpath `{name}`: cutting move has no tool; supply --tool")))?;
            let group = match groups.entry(id) {
                std::collections::btree_map::Entry::Occupied(entry) => entry.into_mut(),
                std::collections::btree_map::Entry::Vacant(entry) => {
                    let tool = tools.get(&id).ok_or_else(|| CliError::run(format!("toolpath `{name}`: unknown tool {id}")))?;
                    entry.insert(ToolCuts { cutter: cutter_for_ir(tool)?, ranges: Vec::new() })
                }
            };
            let before = segments.len();
            append_cutting_motion(motion, sample_step, setup, &mut cursor, &mut segments)?;
            if let Some(last) = group.ranges.last_mut()
                && last.end == before
            {
                last.end = segments.len();
            } else {
                group.ranges.push(before..segments.len());
            }
        }
        if groups.is_empty() {
            continue;
        }
        let view = indexed_mesh(mesh, setup, path_angle(path))?;
        for (id, group) in groups {
            let dc = DropCutter::new(view.as_ref(), group.cutter).map_err(|err| CliError::run(format!("tool {id}: {err}")))?;
            let mut tool_notes = Vec::new();
            for range in group.ranges {
                let cuts = segments.get(range).ok_or_else(|| CliError::run("gouge check: invalid segment range"))?;
                tool_notes.extend(notes_for(&format!("{name} (tool {id})"), &dc, cuts, sample_step, tolerance)?);
                tool_notes.sort_by(|left, right| right.depth.total_cmp(&left.depth));
                tool_notes.truncate(MAX_REPORTS);
            }
            notes.extend(tool_notes);
        }
    }
    Ok(notes)
}

fn is_cut(motion: &ClMotion) -> bool {
    match motion {
        ClMotion::Rapid { .. } => false,
        ClMotion::Thread { .. } => true,
        ClMotion::Linear { role, .. } | ClMotion::Arc { role, .. } | ClMotion::Helix { role, .. } => cutting(*role),
    }
}

fn cutter_for_ir(tool: &Tool) -> Result<Cutter, CliError> {
    let id = tool.id();
    let shape = tool.shape().ok_or_else(|| CliError::run(format!("tool {id} has no cutter shape; supply --tool for mesh verification")))?;
    let diameter = tool.diameter().ok_or_else(|| CliError::run(format!("tool {id} has no cutter diameter")))?;
    let radius = diameter * 0.5;
    let cutter = match shape {
        ToolShape::Flat {} => Cutter::new_flat(radius),
        ToolShape::Ball {} => Cutter::new_ball(radius),
        ToolShape::Bull { corner_radius } => Cutter::new_bull(radius, corner_radius),
        ToolShape::VBit { .. } => return Err(CliError::run(format!("not supported yet: mesh verification for V-bit tool {id}"))),
    };
    cutter.map_err(|err| CliError::run(format!("tool {id}: {err}")))
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;
    use campanion_ir::{CompensationMode, Feeds, RotaryAxisKind, RotaryDirection, Vec3, machine_to_part};
    use campanion_mesh::Triangle;
    use proptest::prelude::*;

    fn slope() -> Mesh {
        // z = x: the correct tip height differs for flat, ball, and bull cutters.
        let a = MeshVec::new(-20.0, -20.0, -20.0);
        let b = MeshVec::new(20.0, -20.0, 20.0);
        let c = MeshVec::new(20.0, 20.0, 20.0);
        let d = MeshVec::new(-20.0, 20.0, -20.0);
        Mesh::new(vec![Triangle { a, b, c }, Triangle { a, b: c, c: d }]).expect("slope")
    }

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn tool(id: u32, shape: Option<ToolShape>) -> Tool {
        let tool = Tool::try_new(id, None, Some(2.0), None).expect("tool");
        match shape {
            Some(shape) => tool.with_shape(shape).expect("shape"),
            None => tool,
        }
    }

    fn path(id: Option<u32>, blocks: Vec<ClBlock>) -> ClPath {
        ClPath::try_new(None, id, CompensationMode::Off, Feeds::try_new(100.0, None, None, None, None).expect("feeds"), None, Vec::new(), blocks)
            .expect("path")
    }

    fn cut(to: Pose) -> ClBlock {
        ClBlock::Motion(ClMotion::Linear { to, feed: 100.0, role: MoveRole::Cut, power: None })
    }

    fn program(tools: Vec<Tool>, paths: Vec<ClPath>) -> Program {
        Program::try_new(None, tools, None, paths, Vec::new(), Vec::new(), None).expect("program")
    }

    #[test]
    fn per_move_tools_follow_changes_and_preserve_segment_start() {
        let mesh = slope();
        let ball_height = 2.0_f64.sqrt() - 1.0;
        let blocks = vec![
            ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, -2.0, ball_height), feed: None }),
            cut(pose(0.0, -1.0, ball_height)),
            ClBlock::Event(Event::ToolChange { tool_id: 2 }),
            // Starts at ball height: a flat cutter gouges until this move reaches z=1.
            cut(pose(0.0, 1.0, 1.0)),
            ClBlock::Event(Event::ToolChange { tool_id: 1 }),
            cut(pose(0.0, 2.0, ball_height)),
            ClBlock::Event(Event::ToolChange { tool_id: 2 }),
            cut(pose(0.0, 3.0, ball_height)),
        ];
        let p = program(vec![tool(1, Some(ToolShape::Ball {})), tool(2, Some(ToolShape::Flat {}))], vec![path(Some(1), blocks)]);
        let notes = check_program_tools(&p, &mesh, 0.01).map_err(|err| err.to_string()).expect("check");
        assert!(!notes.is_empty());
        assert!(notes.iter().all(|note| note.operation.ends_with("(tool 2)")));
        assert!(notes.iter().any(|note| note.depth > 0.5));
        assert!(notes.iter().any(|note| note.y >= 2.0 && note.depth > 0.5));
        assert!(
            check_program(&p, &mesh, Cutter::new_ball(1.0).map_err(|err| err.to_string()).expect("ball"), 0.01)
                .map_err(|err| err.to_string())
                .expect("override")
                .is_empty()
        );
    }

    #[test]
    fn missing_shapes_vbits_and_unused_tools_are_explicit() {
        let mesh = slope();
        let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, 2.0), feed: None }), cut(pose(0.0, 1.0, 2.0))];
        for (shape, expected) in [
            (None, "tool 1 has no cutter shape"),
            (Some(ToolShape::VBit { angle: 1.0, tip_diameter: 0.0 }), "not supported yet: mesh verification for V-bit tool 1"),
        ] {
            let p = program(vec![tool(1, shape)], vec![path(Some(1), blocks.clone())]);
            assert!(check_program_tools(&p, &mesh, 0.01).expect_err("unsupported").to_string().contains(expected));
            assert!(
                check_program(&p, &mesh, Cutter::new_flat(1.0).map_err(|err| err.to_string()).expect("flat"), 0.01)
                    .map_err(|err| err.to_string())
                    .expect("override")
                    .is_empty()
            );
        }
        let p = program(
            vec![tool(1, Some(ToolShape::Flat {})), tool(2, Some(ToolShape::VBit { angle: 1.0, tip_diameter: 0.0 }))],
            vec![path(Some(1), blocks.clone())],
        );
        assert!(check_program_tools(&p, &mesh, 0.01).map_err(|err| err.to_string()).expect("unused V-bit").is_empty());
        let p = program(vec![], vec![path(None, blocks)]);
        assert!(check_program_tools(&p, &mesh, 0.01).expect_err("missing tool").to_string().contains("cutting move has no tool"));
    }

    #[test]
    fn path_declarations_and_inherited_tool_changes_select_the_right_cutter() {
        let blocks = || vec![ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, -1.0, 0.5), feed: None }), cut(pose(0.0, 1.0, 0.5))];
        let mut changed = vec![ClBlock::Event(Event::ToolChange { tool_id: 2 })];
        changed.extend(blocks());
        let paths = vec![path(Some(1), blocks()), path(None, changed), path(None, blocks()), path(Some(1), blocks())];
        let p = program(vec![tool(1, Some(ToolShape::Ball {})), tool(2, Some(ToolShape::Flat {}))], paths);
        let notes = check_program_tools(&p, &slope(), 0.01).map_err(|err| err.to_string()).expect("check");
        assert!(notes.iter().any(|note| note.operation == "toolpath 1 (tool 2)"));
        assert!(notes.iter().any(|note| note.operation == "toolpath 2 (tool 2)"));
        assert!(notes.iter().all(|note| note.operation.ends_with("(tool 2)")));
    }

    #[test]
    fn path_tool_selection_and_indexed_view_are_preserved() {
        let mesh = slope();
        let setup = RotarySetup::new(RotaryAxisKind::A, Vec3::new(0.0, 0.0, 0.0).expect("origin"), RotaryDirection::Positive, None).expect("setup");
        let mut paths = Vec::new();
        for (id, z) in [(1, 2.0), (2, 2.0)] {
            let to = |y| machine_to_part(&setup, pose(0.0, y, z), 180.0).with_rotary(180.0).expect("angle");
            paths.push(path(Some(id), vec![ClBlock::Motion(ClMotion::Rapid { to: to(0.0), feed: None }), cut(to(1.0))]));
        }
        let p = Program::try_new(
            None,
            vec![tool(1, Some(ToolShape::Ball {})), tool(2, Some(ToolShape::Bull { corner_radius: 0.5 }))],
            None,
            paths,
            Vec::new(),
            Vec::new(),
            Some(setup),
        )
        .expect("program");
        assert!(check_program_tools(&p, &mesh, 0.01).map_err(|err| err.to_string()).expect("indexed").is_empty());
    }

    proptest! {
        #[test]
        fn flat_and_ball_selection_changes_gouge_result(z in 0.45f64..0.95) {
            let mesh = slope();
            let blocks = vec![ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, -1.0, z), feed: None }), cut(pose(0.0, 1.0, z))];
            let ball = program(vec![tool(1, Some(ToolShape::Ball {}))], vec![path(Some(1), blocks.clone())]);
            let flat = program(vec![tool(1, Some(ToolShape::Flat {}))], vec![path(Some(1), blocks)]);
            prop_assert!(check_program_tools(&ball, &mesh, 0.01).map_err(|err| err.to_string()).expect("ball").is_empty());
            prop_assert!(!check_program_tools(&flat, &mesh, 0.01).map_err(|err| err.to_string()).expect("flat").is_empty());
        }
    }
}
