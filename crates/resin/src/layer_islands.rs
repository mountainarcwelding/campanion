//! Islands in the sliced layers: lit regions with nothing lit under them in the layer below.
//!
//! This runs on the exact pixel spans written to the printer file, so it sees what will print,
//! supports and raft included. A region that starts in mid-air will not attach to anything and is
//! likely to fail. The first layer sits on the plate and is never an island. Pixels are joined
//! across rows when their columns overlap (4-connected) and within a row when they touch.

use crate::slice::Span;

/// Most island regions recorded for one print. Later ones are counted but not stored.
pub const MAX_RECORDED_ISLANDS: usize = 10_000;

/// One region that starts with nothing under it.
#[derive(Clone, Debug, PartialEq)]
pub struct SliceIsland {
    /// Zero-based layer index.
    pub layer: usize,
    pub pixels: u64,
    /// Inclusive pixel bounds: first column, first row, last column, last row.
    pub bounds: (u32, u32, u32, u32),
    pub spans: Vec<Span>,
}

/// Every island in a print, in layer order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SliceIslands {
    pub islands: Vec<SliceIsland>,
    /// Islands beyond [`MAX_RECORDED_ISLANDS`], counted only.
    pub unrecorded: usize,
}

impl SliceIslands {
    pub fn total(&self) -> usize {
        self.islands.len() + self.unrecorded
    }

    /// Islands that start on `layer`.
    pub fn on_layer(&self, layer: usize) -> impl Iterator<Item = &SliceIsland> {
        self.islands.iter().filter(move |island| island.layer == layer)
    }

    /// First island layer strictly after `layer`.
    pub fn next_layer_after(&self, layer: usize) -> Option<usize> {
        self.islands.iter().map(|island| island.layer).find(|l| *l > layer)
    }

    /// Last island layer strictly before `layer`.
    pub fn prev_layer_before(&self, layer: usize) -> Option<usize> {
        self.islands.iter().rev().map(|island| island.layer).find(|l| *l < layer)
    }
}

/// Collects islands one layer at a time, keeping only the previous layer's spans.
#[derive(Debug, Default)]
pub struct IslandTracker {
    prev: Vec<Span>,
    layer: usize,
    out: SliceIslands,
}

impl IslandTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed the next layer's spans, sorted by row then column.
    pub fn push(&mut self, spans: Vec<Span>) {
        if self.layer > 0 {
            for island in new_islands(&self.prev, &spans, self.layer) {
                if self.out.islands.len() < MAX_RECORDED_ISLANDS {
                    self.out.islands.push(island);
                } else {
                    self.out.unrecorded += 1;
                }
            }
        }
        self.prev = spans;
        self.layer += 1;
    }

    pub fn finish(self) -> SliceIslands {
        self.out
    }
}

/// Regions of `cur` with no pixel over a lit pixel of `prev`.
pub fn new_islands(prev: &[Span], cur: &[Span], layer: usize) -> Vec<SliceIsland> {
    if cur.is_empty() {
        return Vec::new();
    }
    let mut parent: Vec<usize> = (0..cur.len()).collect();
    // Join touching spans in a row, and overlapping spans in adjacent rows.
    let rows = row_ranges(cur);
    for (r, &(row, lo, hi)) in rows.iter().enumerate() {
        for k in lo + 1..hi {
            if let (Some(a), Some(b)) = (cur.get(k - 1), cur.get(k))
                && a.end >= b.start
            {
                union(&mut parent, k - 1, k);
            }
        }
        if let Some(&(next_row, nlo, nhi)) = rows.get(r + 1)
            && next_row == row + 1
        {
            let (mut i, mut j) = (lo, nlo);
            while i < hi && j < nhi {
                let (Some(a), Some(b)) = (cur.get(i), cur.get(j)) else {
                    break;
                };
                if a.start < b.end && b.start < a.end {
                    union(&mut parent, i, j);
                }
                if a.end <= b.end {
                    i += 1;
                } else {
                    j += 1;
                }
            }
        }
    }
    // A component is anchored when any of its spans overlaps a span of the layer below on the same row.
    let mut anchored = vec![false; cur.len()];
    let prev_rows = row_ranges(prev);
    let mut p = 0usize;
    for &(row, lo, hi) in &rows {
        while prev_rows.get(p).is_some_and(|r| r.0 < row) {
            p += 1;
        }
        let Some(&(prow, plo, phi)) = prev_rows.get(p) else {
            continue;
        };
        if prow != row {
            continue;
        }
        let (mut i, mut j) = (lo, plo);
        while i < hi && j < phi {
            let (Some(a), Some(b)) = (cur.get(i), prev.get(j)) else {
                break;
            };
            if a.start < b.end && b.start < a.end {
                let root = find(&mut parent, i);
                if let Some(slot) = anchored.get_mut(root) {
                    *slot = true;
                }
            }
            if a.end <= b.end {
                i += 1;
            } else {
                j += 1;
            }
        }
    }
    let mut groups: std::collections::BTreeMap<usize, SliceIsland> = std::collections::BTreeMap::new();
    for (index, span) in cur.iter().enumerate() {
        let root = find(&mut parent, index);
        if anchored.get(root).copied().unwrap_or(false) {
            continue;
        }
        let entry = groups.entry(root).or_insert_with(|| SliceIsland {
            layer,
            pixels: 0,
            bounds: (span.start, span.row, span.end.saturating_sub(1), span.row),
            spans: Vec::new(),
        });
        entry.pixels += u64::from(span.end - span.start);
        entry.bounds.0 = entry.bounds.0.min(span.start);
        entry.bounds.1 = entry.bounds.1.min(span.row);
        entry.bounds.2 = entry.bounds.2.max(span.end.saturating_sub(1));
        entry.bounds.3 = entry.bounds.3.max(span.row);
        entry.spans.push(*span);
    }
    groups.into_values().collect()
}

/// `(row, first index, one past last index)` for each row present.
fn row_ranges(spans: &[Span]) -> Vec<(u32, usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while let Some(first) = spans.get(i) {
        let row = first.row;
        let start = i;
        while spans.get(i).is_some_and(|s| s.row == row) {
            i += 1;
        }
        out.push((row, start, i));
    }
    out
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while let Some(&p) = parent.get(i) {
        if p == i {
            break;
        }
        let grand = parent.get(p).copied().unwrap_or(p);
        if let Some(slot) = parent.get_mut(i) {
            *slot = grand;
        }
        i = grand;
    }
    i
}

fn union(parent: &mut [usize], a: usize, b: usize) {
    let ra = find(parent, a);
    let rb = find(parent, b);
    if ra != rb
        && let Some(slot) = parent.get_mut(ra.max(rb))
    {
        *slot = ra.min(rb);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn s(row: u32, start: u32, end: u32) -> Span {
        Span { row, start, end }
    }

    #[test]
    fn a_region_over_nothing_is_an_island_and_a_connected_one_is_not() {
        let prev = vec![s(5, 10, 12)];
        // Region A rows 4..=6 touches the prev span on row 5. Region B is separate, far to the right.
        let cur = vec![s(4, 8, 14), s(5, 8, 11), s(6, 8, 14), s(5, 40, 44), s(6, 41, 43)];
        let mut sorted = cur.clone();
        sorted.sort_by_key(|sp| (sp.row, sp.start));
        let islands = new_islands(&prev, &sorted, 3);
        assert_eq!(islands.len(), 1);
        assert_eq!(islands[0].pixels, 6);
        assert_eq!(islands[0].bounds, (40, 5, 43, 6));
        assert_eq!(islands[0].layer, 3);
    }

    #[test]
    fn diagonal_neighbours_are_separate_regions() {
        let cur = vec![s(0, 0, 2), s(1, 2, 4)];
        assert_eq!(new_islands(&[], &cur, 1).len(), 2);
        let touching_row = vec![s(0, 0, 2), s(0, 2, 4)];
        assert_eq!(new_islands(&[], &touching_row, 1).len(), 1);
    }

    #[test]
    fn tracker_skips_the_first_layer_and_finds_later_islands() {
        let mut tracker = IslandTracker::new();
        tracker.push(vec![s(0, 0, 10)]);
        tracker.push(vec![s(0, 0, 10)]);
        tracker.push(vec![s(0, 0, 10), s(9, 0, 3)]);
        tracker.push(Vec::new());
        tracker.push(vec![s(0, 0, 10)]);
        let out = tracker.finish();
        assert_eq!(out.islands.iter().map(|i| i.layer).collect::<Vec<_>>(), vec![2, 4]);
        assert_eq!(out.next_layer_after(2), Some(4));
        assert_eq!(out.prev_layer_before(4), Some(2));
        assert_eq!(out.on_layer(2).count(), 1);
    }
}
