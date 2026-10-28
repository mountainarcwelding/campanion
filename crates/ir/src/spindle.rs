//! One spindle start per tool.
//!
//! Depth passes and later operations of the same tool share one process-on. A later process-on
//! is kept only when the spindle speed changes. One process-off is placed after that tool's last
//! move, so the retract above the stock is already done, and before the next tool change.
//! Torch, laser and other processes stay per cut. A turning program is not rewritten.

use crate::Result;
use crate::motion::ClBlock;
use crate::process::{Event, ProcessKind, ProcessParams};
use crate::program::Program;

/// How close two spindle speeds are before they count as the same rpm.
const SPEED_EPS: f64 = 1e-6;

/// How close a dwell must be to the delay stored on the process-on.
const DWELL_EPS: f64 = 1e-9;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mark {
    Keep,
    Drop,
}

struct Slot {
    path: usize,
    /// Tool id of the path that owns this block.
    path_tool: Option<u32>,
    first: bool,
    block: ClBlock,
    mark: Mark,
}

struct Cycle {
    start: usize,
    end: usize,
    /// Path that receives this cycle's process-off.
    path: usize,
    needs_off: bool,
    params: ProcessParams,
}

/// Rewrite `program` so a spindle stays on for the whole tool.
pub fn hold_spindle(program: Program) -> Result<Program> {
    if program.turning().is_some() || !has_spindle(&program) {
        return Ok(program);
    }
    let mut slots = slots_of(&program);
    let mut cycles = cycles_of(&slots);
    for cycle in &mut cycles {
        mark_cycle(&mut slots, cycle);
    }
    let mut blocks: Vec<Vec<ClBlock>> = program.toolpaths().iter().map(|_| Vec::new()).collect();
    let mut cursor = 0usize;
    for cycle in &cycles {
        while cursor < cycle.end {
            let Some(slot) = slots.get(cursor) else {
                break;
            };
            if slot.mark == Mark::Keep
                && let Some(path) = blocks.get_mut(slot.path)
            {
                path.push(slot.block.clone());
            }
            cursor = cursor.saturating_add(1);
        }
        if cycle.needs_off
            && let Some(path) = blocks.get_mut(cycle.path)
        {
            push_stop(path, &cycle.params);
        }
    }
    program.map_toolpaths(blocks)
}

fn has_spindle(program: &Program) -> bool {
    program.toolpaths().iter().any(|path| path.blocks().iter().any(is_spindle_block))
}

fn is_spindle_block(block: &ClBlock) -> bool {
    matches!(block, ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, .. } | Event::ProcessOff { process: ProcessKind::Spindle }))
}

fn slots_of(program: &Program) -> Vec<Slot> {
    let mut slots = Vec::new();
    for (path_index, path) in program.toolpaths().iter().enumerate() {
        for (block_index, block) in path.blocks().iter().enumerate() {
            slots.push(Slot { path: path_index, path_tool: path.tool_id(), first: block_index == 0, block: block.clone(), mark: Mark::Keep });
        }
    }
    slots
}

/// One cycle per tool. A tool change, or a path whose tool id differs, starts the next cycle.
fn cycles_of(slots: &[Slot]) -> Vec<Cycle> {
    let mut cycles = Vec::new();
    let mut start = 0usize;
    let mut tool = slots.first().and_then(|slot| slot.path_tool);
    for (index, slot) in slots.iter().enumerate() {
        let changed = tool_changed(slot, tool, index > start);
        if changed && index > start {
            cycles.push(cycle_between(slots, start, index));
            start = index;
        }
        if let Some(id) = tool_id_of(slot) {
            tool = Some(id);
        }
    }
    if start < slots.len() {
        cycles.push(cycle_between(slots, start, slots.len()));
    }
    cycles
}

fn tool_changed(slot: &Slot, tool: Option<u32>, after_start: bool) -> bool {
    if !after_start {
        return false;
    }
    if let ClBlock::Event(Event::ToolChange { .. }) = &slot.block {
        return true;
    }
    slot.first && slot.path_tool.is_some() && slot.path_tool != tool
}

fn tool_id_of(slot: &Slot) -> Option<u32> {
    match &slot.block {
        ClBlock::Event(Event::ToolChange { tool_id }) => Some(*tool_id),
        _ if slot.first => slot.path_tool,
        _ => None,
    }
}

fn cycle_between(slots: &[Slot], start: usize, end: usize) -> Cycle {
    let path = slots.get(end.saturating_sub(1)).map(|slot| slot.path).unwrap_or(0);
    Cycle { start, end, path, needs_off: false, params: ProcessParams::new() }
}

fn mark_cycle(slots: &mut [Slot], cycle: &mut Cycle) {
    let mut kept_speed: Option<Option<f64>> = None;
    let mut recent: Option<ProcessParams> = None;
    let mut index = cycle.start;
    while index < cycle.end {
        let kind = slots.get(index).and_then(|slot| spindle_kind(&slot.block));
        match kind {
            Some(SpindleKind::On(params)) => {
                cycle.needs_off = true;
                cycle.params = params.clone();
                let speed = params.speed_value();
                let keep = match kept_speed {
                    None => true,
                    Some(previous) => !speeds_match(previous, speed),
                };
                if keep {
                    kept_speed = Some(speed);
                } else {
                    if let Some(slot) = slots.get_mut(index) {
                        slot.mark = Mark::Drop;
                    }
                    drop_delay(slots, cycle, index, &params, true);
                }
                recent = Some(params);
            }
            Some(SpindleKind::Off) => {
                cycle.needs_off = true;
                if let Some(slot) = slots.get_mut(index) {
                    slot.mark = Mark::Drop;
                }
                if let Some(params) = recent.as_ref() {
                    drop_delay(slots, cycle, index, params, false);
                }
            }
            None => {}
        }
        index = index.saturating_add(1);
    }
}

enum SpindleKind {
    On(ProcessParams),
    Off,
}

fn spindle_kind(block: &ClBlock) -> Option<SpindleKind> {
    match block {
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params }) => Some(SpindleKind::On(params.clone())),
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }) => Some(SpindleKind::Off),
        _ => None,
    }
}

fn speeds_match(left: Option<f64>, right: Option<f64>) -> bool {
    match (left, right) {
        (None, None) => true,
        (Some(left), Some(right)) => (left - right).abs() <= SPEED_EPS,
        _ => false,
    }
}

/// Drop the pierce or post dwell that sits against a process event this cycle will not keep.
fn drop_delay(slots: &mut [Slot], cycle: &Cycle, event: usize, params: &ProcessParams, pierce: bool) {
    let (seconds, before) = if pierce {
        (params.pierce_delay_s(), !params.pierce_process_on_during_delay())
    } else {
        (params.post_delay_s(), params.post_process_on_during_delay())
    };
    if seconds <= 0.0 {
        return;
    }
    let Some(at) = (if before { event.checked_sub(1) } else { event.checked_add(1) }) else {
        return;
    };
    if at < cycle.start || at >= cycle.end {
        return;
    }
    let Some(slot) = slots.get_mut(at) else {
        return;
    };
    if slot.mark != Mark::Keep {
        return;
    }
    if let ClBlock::Event(Event::Dwell { seconds: dwell }) = &slot.block
        && (*dwell - seconds).abs() <= DWELL_EPS
    {
        slot.mark = Mark::Drop;
    }
}

fn push_stop(blocks: &mut Vec<ClBlock>, params: &ProcessParams) {
    let seconds = params.post_delay_s();
    let dwell = seconds > 0.0;
    if params.post_process_on_during_delay() && dwell {
        blocks.push(ClBlock::Event(Event::Dwell { seconds }));
    }
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }));
    if !params.post_process_on_during_delay() && dwell {
        blocks.push(ClBlock::Event(Event::Dwell { seconds }));
    }
}

#[cfg(test)]
mod tests {
    use super::hold_spindle;
    use crate::geom::{Pose, Vec3};
    use crate::motion::{ClBlock, ClMotion, ClPath, CompensationMode, Feeds, MoveRole};
    use crate::process::{Event, ProcessKind, ProcessParams, Tool};
    use crate::program::Program;
    use crate::turning::{TurnBinding, TurningSetup};

    fn pose(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(Vec3::new(x, y, z).expect("tip"), Vec3::Z).expect("pose")
    }

    fn feeds() -> Feeds {
        Feeds::try_new(1000.0, None, None, None, None).expect("feeds")
    }

    fn path(tool: Option<u32>, blocks: Vec<ClBlock>) -> ClPath {
        ClPath::try_new(None, tool, CompensationMode::Off, feeds(), None, Vec::new(), blocks).expect("path")
    }

    fn tool(id: u32) -> Tool {
        Tool::try_new(id, None, Some(6.0), None).expect("tool")
    }

    fn program(paths: Vec<ClPath>) -> Program {
        let tools = vec![tool(1), tool(2)];
        Program::try_new(None, tools, None, paths, Vec::new(), Vec::new(), None).expect("program")
    }

    fn on(speed: f64, post: f64) -> ClBlock {
        let params = ProcessParams::new().speed(Some(speed)).expect("speed").post_delay(post, false).expect("post");
        ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, params })
    }

    fn off() -> ClBlock {
        ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle })
    }

    fn rapid(z: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Rapid { to: pose(0.0, 0.0, z), feed: None })
    }

    fn cut(z: f64) -> ClBlock {
        ClBlock::Motion(ClMotion::Linear { to: pose(1.0, 0.0, z), feed: 1000.0, role: MoveRole::Cut })
    }

    fn kinds(path: &ClPath) -> Vec<&'static str> {
        path.blocks()
            .iter()
            .map(|block| match block {
                ClBlock::Event(Event::ProcessOn { process: ProcessKind::Spindle, .. }) => "on",
                ClBlock::Event(Event::ProcessOff { process: ProcessKind::Spindle }) => "off",
                ClBlock::Event(Event::ProcessOn { .. }) => "other-on",
                ClBlock::Event(Event::ProcessOff { .. }) => "other-off",
                ClBlock::Event(Event::Dwell { .. }) => "dwell",
                ClBlock::Event(Event::ToolChange { .. }) => "tool",
                ClBlock::Motion(ClMotion::Rapid { .. }) => "rapid",
                ClBlock::Motion(_) => "cut",
                ClBlock::Event(_) => "event",
            })
            .collect()
    }

    #[test]
    fn spindle_stays_on_across_passes_and_retracts_before_the_stop() {
        let first = path(
            Some(1),
            vec![
                ClBlock::Event(Event::ToolChange { tool_id: 1 }),
                rapid(5.0),
                on(12_000.0, 0.25),
                cut(-6.0),
                off(),
                ClBlock::Event(Event::Dwell { seconds: 0.25 }),
                rapid(5.0),
            ],
        );
        let second =
            path(Some(1), vec![rapid(5.0), on(12_000.0, 0.25), cut(-6.0), off(), ClBlock::Event(Event::Dwell { seconds: 0.25 }), rapid(5.0)]);
        let held = hold_spindle(program(vec![first, second])).expect("hold");
        let paths = held.toolpaths();
        assert_eq!(kinds(&paths[0]), vec!["tool", "rapid", "on", "cut", "rapid"]);
        assert_eq!(kinds(&paths[1]), vec!["rapid", "cut", "rapid", "off", "dwell"]);
    }

    #[test]
    fn speed_change_writes_another_on_without_an_off() {
        let first = path(Some(1), vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), on(10_000.0, 0.0), cut(-3.0), off(), rapid(5.0)]);
        let second = path(Some(1), vec![on(16_000.0, 0.0), cut(-6.0), off(), rapid(5.0)]);
        let held = hold_spindle(program(vec![first, second])).expect("hold");
        let paths = held.toolpaths();
        assert_eq!(kinds(&paths[0]), vec!["tool", "on", "cut", "rapid"]);
        assert_eq!(kinds(&paths[1]), vec!["on", "cut", "rapid", "off"]);
        let ClBlock::Event(Event::ProcessOn { params, .. }) = &paths[1].blocks()[0] else {
            panic!("speed change");
        };
        assert_eq!(params.speed_value(), Some(16_000.0));
    }

    #[test]
    fn torch_cycles_stay_per_cut() {
        let torch_on = ClBlock::Event(Event::ProcessOn { process: ProcessKind::Torch, params: ProcessParams::new() });
        let torch_off = ClBlock::Event(Event::ProcessOff { process: ProcessKind::Torch });
        let spindle = path(Some(1), vec![ClBlock::Event(Event::ToolChange { tool_id: 1 }), on(12_000.0, 0.0), cut(-1.0), off(), rapid(5.0)]);
        let torch = path(Some(2), vec![ClBlock::Event(Event::ToolChange { tool_id: 2 }), torch_on, cut(0.0), torch_off, rapid(5.0)]);
        let held = hold_spindle(program(vec![spindle, torch])).expect("hold");
        assert_eq!(kinds(&held.toolpaths()[0]), vec!["tool", "on", "cut", "rapid", "off"]);
        assert_eq!(kinds(&held.toolpaths()[1]), vec!["tool", "other-on", "cut", "other-off", "rapid"]);
    }

    #[test]
    fn turning_program_is_unchanged() {
        let blocks = vec![on(1000.0, 0.0), cut(-2.0), off()];
        let setup = TurningSetup::single(None, 3000.0)
            .expect("setup")
            .with_binding(TurnBinding::try_new(0, "main", "turret1", 0).expect("binding"))
            .expect("binding");
        let program = Program::try_new(None, vec![tool(1)], None, vec![path(Some(1), blocks)], Vec::new(), Vec::new(), None)
            .expect("program")
            .with_turning(Some(setup))
            .expect("turning");
        let held = hold_spindle(program.clone()).expect("hold");
        assert_eq!(held, program);
    }
}
