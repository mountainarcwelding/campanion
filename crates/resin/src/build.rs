//! Mesh in, `.pm7m` bytes out.

use campanion_mesh::Mesh;

use crate::error::Result;
use crate::pm7m::{MachineSpec, ResinSettings, WriteInput, make_previews, write_pm7m};
use crate::slice::{Placement, PreparedMesh, encode_spans};

/// What a build produced besides the file bytes.
#[derive(Debug, Clone, PartialEq)]
pub struct BuildReport {
    pub layers: usize,
    pub height_mm: f64,
    pub volume_ml: f64,
    pub xy_shift_mm: (f64, f64),
    pub empty_layers: usize,
    pub bytes: usize,
}

/// Slice `mesh` at `settings.layer_height_mm` and write a `.pm7m` for `machine`.
pub fn build_pm7m(mesh: &Mesh, machine: &MachineSpec, settings: &ResinSettings, placement: Placement) -> Result<(Vec<u8>, BuildReport)> {
    settings.validate()?;
    let grid = machine.grid();
    let prepared = PreparedMesh::new(mesh, grid, machine.bed_z_mm, settings.layer_height_mm, placement)?;
    let mut images = Vec::with_capacity(prepared.layer_count());
    let mut lit = 0u64;
    let mut empty = 0usize;
    for layer in 0..prepared.layer_count() {
        let spans = prepared.layer_spans(layer);
        if spans.is_empty() {
            empty += 1;
        }
        lit += PreparedMesh::lit_pixels(&spans);
        images.push(encode_spans(&spans, grid.width, grid.height)?);
    }
    let volume_mm3 = lit as f64 * grid.pitch_x_mm * grid.pitch_y_mm * settings.layer_height_mm;
    let volume_ml = volume_mm3 / 1000.0;
    let (p0, p1) = make_previews(mesh)?;
    let bytes = write_pm7m(&WriteInput { machine, settings, layer_images: &images, previews: (&p0, &p1), volume_ml })?;
    let report = BuildReport {
        layers: images.len(),
        height_mm: prepared.height_mm(),
        volume_ml,
        xy_shift_mm: prepared.xy_shift_mm(),
        empty_layers: empty,
        bytes: bytes.len(),
    };
    Ok((bytes, report))
}
