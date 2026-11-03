//! In-process stock in the preview.
//!
//! Loading, the snapshot choice, the height colours and the triangles or wireframe all live here.
//! The window only paints [`StockDraw`]. Distances are millimetres. A cell with more than one
//! solid interval is drawn in orange so an undercut is visible from the single top height.
//!
//! This display has not been proven on a machine.

use campanion_mesh::{StockFile, StockGrid, StockSnapshot};

use super::{Projection, height_color, project};

/// Orange used for a column that holds more than one solid interval.
const MULTI_RGB: [u8; 3] = [255, 150, 40];

/// Fill used when every solid cell has the same top, so a height ramp has nothing to show.
const FLAT_STOCK_RGB: [u8; 3] = [176, 140, 96];

/// Which picture [`tessellate_stock`] builds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StockViewKind {
    /// Shaded top surface, plus an orange edge on undercut cells.
    Plan,
    /// Wireframe of the top. Undercut cells are orange.
    Iso,
}

/// One filled triangle in part-frame millimetres.
#[derive(Debug, Clone, PartialEq)]
pub struct StockTri {
    /// Corners, Z up.
    pub pts: [[f64; 3]; 3],
    /// RGB.
    pub color: [u8; 3],
}

/// One wireframe edge in part-frame millimetres.
#[derive(Debug, Clone, PartialEq)]
pub struct StockLine {
    /// Start.
    pub a: [f64; 3],
    /// End.
    pub b: [f64; 3],
    /// RGB.
    pub color: [u8; 3],
}

/// Triangles and edges for one stock frame.
#[derive(Debug, Clone, PartialEq)]
pub struct StockDraw {
    /// Plan fill. Empty for [`StockViewKind::Iso`].
    pub triangles: Vec<StockTri>,
    /// Outlines. Plan keeps only the undercut cells. Iso keeps every solid cell.
    pub lines: Vec<StockLine>,
}

/// Final stock and, when the file has them, earlier snapshots ordered by tool travel.
#[derive(Debug, Clone, PartialEq)]
pub struct StockMovie {
    /// `true` when the scrub should pick a snapshot. A file with none always shows the final map.
    scrub: bool,
    frames: Vec<Frame>,
}

#[derive(Debug, Clone, PartialEq)]
struct Frame {
    position: f64,
    grid: StockGrid,
}

impl StockMovie {
    /// Read a `campanion-stock/1` document.
    pub fn load(json: &str) -> Result<Self, String> {
        let file = StockFile::from_json(json).map_err(|err| err.to_string())?;
        Ok(Self::from_file(file))
    }

    /// Frames in tool-travel order. The final grid is the state at [`StockFile::length`].
    #[must_use]
    pub fn from_file(file: StockFile) -> Self {
        if file.snapshots.is_empty() {
            return Self { scrub: false, frames: vec![Frame { position: file.length, grid: file.grid }] };
        }
        let mut frames: Vec<Frame> = file.snapshots.into_iter().map(frame_from_snapshot(&file.grid)).collect();
        frames.sort_by(|left, right| left.position.partial_cmp(&right.position).unwrap_or(std::cmp::Ordering::Equal));
        let same_end = frames.last().is_some_and(|frame| (frame.position - file.length).abs() <= 1e-6);
        if same_end {
            if let Some(last) = frames.last_mut() {
                last.grid.top = file.grid.top;
                last.grid.multi = file.grid.multi;
                last.position = file.length;
            }
        } else {
            frames.push(Frame { position: file.length, grid: file.grid });
        }
        Self { scrub: true, frames }
    }

    /// `true` when playback chooses a snapshot. Otherwise the movie is the final stock.
    #[must_use]
    pub fn scrubs(&self) -> bool {
        self.scrub
    }

    /// Index of the last frame at or before `position`.
    ///
    /// Without snapshots this is always `Some(0)`, the final stock. With snapshots, a position
    /// before every frame is `None` (a later frame is not shown early). The index never goes
    /// backwards as `position` increases: it is monotone.
    #[must_use]
    pub fn frame_index(&self, position: f64) -> Option<usize> {
        if self.frames.is_empty() {
            return None;
        }
        if !self.scrub {
            return Some(0);
        }
        if !position.is_finite() {
            return None;
        }
        let position = position.max(0.0);
        let mut chosen = None;
        for (index, frame) in self.frames.iter().enumerate() {
            if frame.position <= position + 1e-9 {
                chosen = Some(index);
            } else {
                break;
            }
        }
        chosen
    }

    /// Grid for [`StockMovie::frame_index`]. `None` when the index is outside the movie.
    #[must_use]
    pub fn grid(&self, index: usize) -> Option<&StockGrid> {
        self.frames.get(index).map(|frame| &frame.grid)
    }

    /// Tool travel of one frame.
    #[must_use]
    pub fn position(&self, index: usize) -> Option<f64> {
        self.frames.get(index).map(|frame| frame.position)
    }

    /// How many frames the scrub can land on.
    #[must_use]
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// `true` when the movie has no frames.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }
}

fn frame_from_snapshot(header: &StockGrid) -> impl Fn(StockSnapshot) -> Frame + '_ {
    move |snapshot| Frame {
        position: snapshot.position,
        grid: StockGrid { origin: header.origin, cell: header.cell, nx: header.nx, ny: header.ny, top: snapshot.top, multi: snapshot.multi },
    }
}

/// Triangles or a wireframe for `grid`.
///
/// Plan shades each cell by its top Z. A single height uses one stock colour. Iso draws the cell
/// square at that Z. A column in `multi` is orange in both pictures.
#[must_use]
pub fn tessellate_stock(grid: &StockGrid, kind: StockViewKind) -> StockDraw {
    let range = z_extent(grid);
    let multi = multi_set(grid);
    let mut triangles = Vec::new();
    let mut lines = Vec::new();
    if grid.nx == 0 || grid.ny == 0 || !grid.cell.is_finite() || grid.cell <= 0.0 {
        return StockDraw { triangles, lines };
    }
    for iy in 0..grid.ny {
        for ix in 0..grid.nx {
            let index = iy * grid.nx + ix;
            let Some(z) = grid.top.get(index).copied().flatten() else {
                continue;
            };
            if !z.is_finite() {
                continue;
            }
            let undercut = multi.contains(&index);
            let color = cell_color(z, range, undercut, kind);
            let corners = cell_corners(grid, ix, iy, z);
            match kind {
                StockViewKind::Plan => {
                    triangles.push(StockTri { pts: [corners[0], corners[1], corners[2]], color: cell_color(z, range, false, kind) });
                    triangles.push(StockTri { pts: [corners[0], corners[2], corners[3]], color: cell_color(z, range, false, kind) });
                    if undercut {
                        push_loop(&mut lines, &corners, MULTI_RGB);
                    }
                }
                StockViewKind::Iso => push_loop(&mut lines, &corners, color),
            }
        }
    }
    StockDraw { triangles, lines }
}

/// Projected bounds of the solid cells, for fitting the view.
#[must_use]
pub fn stock_bounds(grid: &StockGrid, projection: Projection) -> Option<([f64; 2], [f64; 2])> {
    let mut min = [f64::INFINITY, f64::INFINITY];
    let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut any = false;
    if grid.nx == 0 || grid.ny == 0 {
        return None;
    }
    for iy in 0..grid.ny {
        for ix in 0..grid.nx {
            let index = iy * grid.nx + ix;
            let Some(z) = grid.top.get(index).copied().flatten().filter(|z| z.is_finite()) else {
                continue;
            };
            for corner in cell_corners(grid, ix, iy, z) {
                let point = project([corner[0], corner[1]], corner[2], projection);
                if !point[0].is_finite() || !point[1].is_finite() {
                    continue;
                }
                any = true;
                min[0] = min[0].min(point[0]);
                min[1] = min[1].min(point[1]);
                max[0] = max[0].max(point[0]);
                max[1] = max[1].max(point[1]);
            }
        }
    }
    if any { Some((min, max)) } else { None }
}

fn cell_color(z: f64, range: Option<(f64, f64)>, undercut: bool, kind: StockViewKind) -> [u8; 3] {
    if undercut && kind == StockViewKind::Iso {
        return MULTI_RGB;
    }
    match range {
        Some((low, high)) if high > low => height_color(z, (low, high)),
        _ => FLAT_STOCK_RGB,
    }
}

fn z_extent(grid: &StockGrid) -> Option<(f64, f64)> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for z in grid.top.iter().copied().flatten() {
        if z.is_finite() {
            low = low.min(z);
            high = high.max(z);
        }
    }
    if low.is_finite() && high.is_finite() { Some((low, high)) } else { None }
}

fn multi_set(grid: &StockGrid) -> std::collections::BTreeSet<usize> {
    grid.multi.iter().copied().filter(|index| *index < grid.top.len()).collect()
}

fn cell_corners(grid: &StockGrid, ix: usize, iy: usize, z: f64) -> [[f64; 3]; 4] {
    let x0 = grid.origin[0] + ix as f64 * grid.cell;
    let y0 = grid.origin[1] + iy as f64 * grid.cell;
    let x1 = x0 + grid.cell;
    let y1 = y0 + grid.cell;
    [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]
}

fn push_loop(lines: &mut Vec<StockLine>, corners: &[[f64; 3]; 4], color: [u8; 3]) {
    for index in 0..4 {
        lines.push(StockLine { a: corners[index], b: corners[(index + 1) % 4], color });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_mesh::STOCK_FORMAT;
    use proptest::prelude::*;

    fn grid(top: Vec<Option<f64>>, multi: Vec<usize>) -> StockGrid {
        let cells = top.len();
        StockGrid { origin: [0.0, -1.0], cell: 2.0, nx: cells, ny: 1, top, multi }
    }

    #[test]
    fn plan_shades_by_height_and_iso_is_a_wireframe() {
        let map = grid(vec![Some(0.0), Some(4.0), None], vec![1]);
        let plan = tessellate_stock(&map, StockViewKind::Plan);
        assert_eq!(plan.triangles.len(), 4, "two triangles on each solid cell");
        assert_eq!(plan.triangles[0].color, height_color(0.0, (0.0, 4.0)));
        assert_eq!(plan.triangles[2].color, height_color(4.0, (0.0, 4.0)));
        assert_eq!(plan.lines.len(), 4, "only the undercut cell is outlined in plan");
        assert!(plan.lines.iter().all(|line| line.color == MULTI_RGB));
        let iso = tessellate_stock(&map, StockViewKind::Iso);
        assert!(iso.triangles.is_empty());
        assert_eq!(iso.lines.len(), 8, "two solid cells");
        assert!(iso.lines.iter().any(|line| line.color == MULTI_RGB));
        assert!(iso.lines.iter().any(|line| line.color == height_color(0.0, (0.0, 4.0))));
        let flat = tessellate_stock(&grid(vec![Some(3.0), Some(3.0)], Vec::new()), StockViewKind::Plan);
        assert!(flat.triangles.iter().all(|tri| tri.color == FLAT_STOCK_RGB));
        let bounds = stock_bounds(&map, Projection::Top).expect("bounds");
        assert!(bounds.0[0] <= 0.0 && bounds.1[0] >= 4.0);
    }

    #[test]
    fn the_file_round_trips_into_a_movie() {
        let map = grid(vec![Some(1.5), None], vec![]);
        let file = StockFile {
            grid: map.clone(),
            length: 8.0,
            snapshots: vec![StockSnapshot { position: 0.0, top: vec![Some(5.0), Some(5.0)], multi: vec![] }],
        };
        let text = file.to_json().expect("json");
        assert!(text.contains(STOCK_FORMAT));
        let movie = StockMovie::load(&text).expect("load");
        assert!(movie.scrubs());
        assert_eq!(movie.len(), 2);
        assert_eq!(movie.frame_index(0.0), Some(0));
        assert_eq!(movie.frame_index(3.0), Some(0));
        assert_eq!(movie.frame_index(8.0), Some(1));
        assert_eq!(movie.grid(0).expect("start").top, vec![Some(5.0), Some(5.0)]);
        assert_eq!(movie.grid(1).expect("end").top[0], Some(1.5));
    }

    fn picker_config() -> ProptestConfig {
        ProptestConfig { cases: 32, max_shrink_iters: 40, rng_seed: proptest::test_runner::RngSeed::Fixed(0x00C0_232B), ..ProptestConfig::default() }
    }

    proptest! {
        #![proptest_config(picker_config())]
        #[test]
        fn the_snapshot_index_is_monotone(
            marks in proptest::collection::vec(0.0..100.0_f64, 1..8),
            probes in proptest::collection::vec(-5.0..120.0_f64, 2..12),
        ) {
            let mut marks = marks;
            marks.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
            let top = vec![Some(1.0_f64)];
            let snapshots: Vec<StockSnapshot> = marks
                .iter()
                .copied()
                .map(|position| StockSnapshot { position, top: top.clone(), multi: Vec::new() })
                .collect();
            let file = StockFile {
                grid: StockGrid { origin: [0.0, 0.0], cell: 1.0, nx: 1, ny: 1, top: vec![Some(0.0)], multi: Vec::new() },
                length: 150.0,
                snapshots,
            };
            let movie = StockMovie::from_file(file);
            let mut probes = probes;
            probes.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
            let mut previous: i32 = -1;
            for probe in probes {
                let rank = movie.frame_index(probe).map(|index| index as i32).unwrap_or(-1);
                prop_assert!(rank >= previous, "probe {probe} rank {rank} after {previous}");
                previous = rank;
            }
            // A position before the first snapshot does not show a later frame.
            if let Some(first) = marks.first()
                && *first > 1.0
            {
                prop_assert_eq!(movie.frame_index(first - 1.0), None);
            }
            prop_assert_eq!(movie.frame_index(1_000.0), Some(movie.len() - 1));
        }
    }
}
