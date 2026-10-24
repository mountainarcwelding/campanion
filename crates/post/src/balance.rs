//! Per-channel time between sync events.
//!
//! [`channel_balance`] splits each turning channel at [`campanion_ir::Event::Sync`] and times the
//! pieces with [`campanion_ir::verify`]. The first motion of a piece has no start in that walk, so
//! a zero-feed rapid back to the previous tip is prepended when a tip exists. Rapids without a
//! feed add no time. A thread move adds length and no time.

use std::collections::BTreeMap;

use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, Pose, Program, Vec3, verify};

use crate::def::PostDef;
use crate::error::{Result, invalid, unsupported};

/// Time and idle for one stretch between syncs.
///
/// `segment` 0 is the motion before the first sync. `times_s[channel]` is that channel's cutting
/// and dwell time. `idle_s[channel]` is the slowest channel in the segment minus this channel.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelSegment {
    /// Index of this stretch. 0 is before the first sync.
    pub segment: usize,
    /// Seconds per channel. The index is the channel id.
    pub times_s: Vec<f64>,
    /// `max(times_s) - times_s[channel]`.
    pub idle_s: Vec<f64>,
}

/// One channel's toolpaths, in program order.
pub(crate) struct ChannelSection {
    pub(crate) channel: u16,
    pub(crate) paths: Vec<usize>,
}

/// Channel of toolpath `index`. An unbound path is channel 0.
pub(crate) fn channel_of(program: &Program, index: usize) -> u16 {
    let Ok(id) = u32::try_from(index) else {
        return 0;
    };
    program.turning().and_then(|turning| turning.binding_for(id)).map(|binding| binding.channel()).unwrap_or(0)
}

/// Channel sections when a binding uses a channel other than 0.
///
/// `None` keeps today's single-channel write: no section banners, and a sync stays a comment.
/// Every declared channel is returned, including one with no toolpaths. Used channels (at least
/// one toolpath) must list the same sync ids in the same order.
pub(crate) fn channel_sections(program: &Program, post: &PostDef) -> Result<Option<Vec<ChannelSection>>> {
    let Some(first) = (0..program.toolpaths().len()).map(|index| channel_of(program, index)).find(|channel| *channel != 0) else {
        return Ok(None);
    };
    let Some(spec) = post.lathe.as_ref().and_then(|lathe| lathe.channels.as_ref()) else {
        return Err(unsupported(format!("post `{}` is single-channel; the program uses channel {first}", post.id())));
    };
    let count = spec.count;
    for index in 0..program.toolpaths().len() {
        let channel = channel_of(program, index);
        if channel >= count {
            return Err(unsupported(format!("post `{}` has no channel {channel}", post.id())));
        }
    }
    let mut sections = Vec::with_capacity(usize::from(count));
    for channel in 0..count {
        sections.push(ChannelSection { channel, paths: Vec::new() });
    }
    for index in 0..program.toolpaths().len() {
        let channel = channel_of(program, index);
        let Some(section) = sections.get_mut(usize::from(channel)) else {
            continue;
        };
        section.paths.push(index);
    }
    check_sync_order(program, &sections)?;
    Ok(Some(sections))
}

fn check_sync_order(program: &Program, sections: &[ChannelSection]) -> Result<()> {
    let mut used = sections.iter().filter(|section| !section.paths.is_empty());
    let Some(first) = used.next() else {
        return Ok(());
    };
    let expected = sync_ids(program, &first.paths);
    for section in used {
        let found = sync_ids(program, &section.paths);
        if found != expected {
            return Err(invalid(format!(
                "channel sync mismatch: channel {} waits for {expected:?}, channel {} for {found:?}",
                first.channel, section.channel
            )));
        }
    }
    Ok(())
}

fn sync_ids(program: &Program, paths: &[usize]) -> Vec<u32> {
    let mut ids = Vec::new();
    for index in paths {
        let Some(path) = program.toolpaths().get(*index) else {
            continue;
        };
        for block in path.blocks() {
            if let ClBlock::Event(Event::Sync { id }) = block {
                ids.push(*id);
            }
        }
    }
    ids
}

/// Seconds of work and idle on each channel, split at sync events.
///
/// Channels that do not appear, and segments a channel does not reach, are 0. The time rule is
/// [`campanion_ir::VerifyReport::time_s`].
pub fn channel_balance(program: &Program) -> Vec<ChannelSegment> {
    if program.toolpaths().is_empty() {
        return Vec::new();
    }
    let mut runs: BTreeMap<u16, Vec<Vec<ClBlock>>> = BTreeMap::new();
    let mut max_channel = 0u16;
    for (index, path) in program.toolpaths().iter().enumerate() {
        let channel = channel_of(program, index);
        max_channel = max_channel.max(channel);
        let segments = runs.entry(channel).or_insert_with(|| vec![Vec::new()]);
        for block in path.blocks() {
            if matches!(block, ClBlock::Event(Event::Sync { .. })) {
                segments.push(Vec::new());
                continue;
            }
            if let Some(current) = segments.last_mut() {
                current.push(block.clone());
            }
        }
    }
    let mut segment_count = 0usize;
    for segments in runs.values() {
        segment_count = segment_count.max(segments.len());
    }
    let width = usize::from(max_channel).saturating_add(1);
    let mut cursors = vec![None; width];
    let mut out = Vec::with_capacity(segment_count);
    for segment in 0..segment_count {
        let mut times_s = vec![0.0; width];
        for channel in 0..width {
            let blocks = runs.get(&u16::try_from(channel).unwrap_or(0)).and_then(|segments| segments.get(segment)).map(Vec::as_slice).unwrap_or(&[]);
            let cursor = cursors.get(channel).copied().flatten();
            let (time, next) = segment_time(blocks, cursor);
            if let Some(slot) = times_s.get_mut(channel) {
                *slot = time;
            }
            if let Some(slot) = cursors.get_mut(channel) {
                *slot = next;
            }
        }
        let mut peak = 0.0;
        for time in &times_s {
            if *time > peak {
                peak = *time;
            }
        }
        let idle_s = times_s.iter().map(|time| peak - time).collect();
        out.push(ChannelSegment { segment, times_s, idle_s });
    }
    out
}

fn segment_time(blocks: &[ClBlock], cursor: Option<Vec3>) -> (f64, Option<Vec3>) {
    let next = blocks.iter().rev().find_map(block_tip).or(cursor);
    let mut body = Vec::with_capacity(blocks.len().saturating_add(1));
    if let Some(block) = anchor(cursor) {
        body.push(block);
    }
    body.extend(blocks.iter().cloned());
    (path_time(&body), next)
}

fn anchor(cursor: Option<Vec3>) -> Option<ClBlock> {
    let pose = Pose::new(cursor?, Vec3::Z).ok()?;
    Some(ClBlock::Motion(ClMotion::Rapid { to: pose, feed: None }))
}

fn block_tip(block: &ClBlock) -> Option<Vec3> {
    let ClBlock::Motion(motion) = block else {
        return None;
    };
    let pose = match motion {
        ClMotion::Rapid { to, .. }
        | ClMotion::Linear { to, .. }
        | ClMotion::Arc { to, .. }
        | ClMotion::Helix { to, .. }
        | ClMotion::Thread { to, .. } => *to,
    };
    Some(pose.tip())
}

fn path_time(blocks: &[ClBlock]) -> f64 {
    if blocks.is_empty() {
        return 0.0;
    }
    let Ok(feeds) = Feeds::try_new(1.0, None, None, None, None) else {
        return 0.0;
    };
    let Ok(path) = ClPath::try_new(None, None, CompensationMode::Off, feeds, None, Vec::new(), blocks.to_vec()) else {
        return 0.0;
    };
    let Ok(program) = Program::try_new(None, Vec::new(), None, vec![path], Vec::new(), Vec::new(), None) else {
        return 0.0;
    };
    verify(&program).time_s()
}
