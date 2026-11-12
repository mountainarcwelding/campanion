//! Laser raster operations: load a PGM beside the job and turn it into IR with per-move power.

use std::path::Path;

use campanion_geom::Point;
use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, MoveRole, Pose, ProcessKind, Vec3};
use campanion_path::{GreyImage, RasterSpec, plan_raster};

use crate::error::CliError;
use crate::jobfile::{JobFile, JobTool, Operation, join_job_path};

/// Load the operation's PGM, plan zigzag scans, and emit one toolpath with per-move power.
pub(crate) fn cut(job: &JobFile, op: &Operation, tool: &JobTool) -> Result<ClPath, CliError> {
    let Some(geom) = op.raster() else {
        return Err(op_err(op, "laser raster is missing image data"));
    };
    let path = join_job_path(job, geom.image());
    let image = load_pgm(&path).map_err(|err| op_err(op, err))?;
    let width = geom.width();
    let height = geom.height();
    let sx = width / f64::from(image.width());
    let sy = height / f64::from(image.height());
    let step = geom.step().unwrap_or(sx.min(sy));
    if !(step.is_finite() && step > 0.0) {
        return Err(op_err(op, "raster step must be positive"));
    }
    let origin = Point::new(geom.origin().0, geom.origin().1).map_err(|err| op_err(op, err))?;
    let spec = RasterSpec {
        origin,
        width,
        height,
        angle: geom.angle(),
        step,
        overscan: geom.overscan(),
        scanning_offset: geom.scanning_offset(),
        map: geom.map().clone(),
    };
    let lines = plan_raster(&image, &spec).map_err(|err| op_err(op, err))?;
    let z = op.params().cut_height_z().unwrap_or(0.0);
    let retract = job.retract();
    let clearance = job.clearance();
    let feed = op.feeds().cut();
    let mut blocks = Vec::new();
    let mut start: Option<(f64, f64)> = None;
    for line in &lines {
        let Some(first) = line.segments.first() else {
            continue;
        };
        start = Some((first.from.x(), first.from.y()));
        break;
    }
    let Some((x0, y0)) = start else {
        return Err(op_err(op, "raster produced no cuts"));
    };
    blocks.push(rapid(x0, y0, clearance)?);
    blocks.push(rapid(x0, y0, retract)?);
    blocks.push(ClBlock::Event(Event::ProcessOn { process: ProcessKind::Laser, params: op.params().clone() }));
    let mut at_retract = true;
    for line in &lines {
        let Some(first) = line.segments.first() else {
            continue;
        };
        if at_retract {
            blocks.push(linear(first.from.x(), first.from.y(), z, feed, Some(0.0))?);
            at_retract = false;
        } else {
            blocks.push(rapid(first.from.x(), first.from.y(), retract)?);
            blocks.push(linear(first.from.x(), first.from.y(), z, feed, Some(0.0))?);
        }
        for seg in &line.segments {
            blocks.push(linear(seg.to.x(), seg.to.y(), z, feed, Some(seg.power))?);
        }
        let Some(last) = line.segments.last() else {
            continue;
        };
        blocks.push(linear(last.to.x(), last.to.y(), retract, feed, Some(0.0))?);
        at_retract = true;
    }
    blocks.push(ClBlock::Event(Event::ProcessOff { process: ProcessKind::Laser }));
    let end = match blocks.iter().rev().find_map(end_xy) {
        Some(end) => end,
        None => (x0, y0),
    };
    blocks.push(rapid(end.0, end.1, clearance)?);
    let _ = tool;
    ClPath::try_new(Some(op.name().to_string()), Some(op.tool_id()), CompensationMode::Off, op.feeds(), None, Vec::new(), blocks)
        .map_err(|err| op_err(op, err))
}

/// Axis-aligned bounds of the placed image, grown by overscan and the scanning offset.
pub(crate) fn bounds(op: &Operation) -> Result<(f64, f64, f64, f64), CliError> {
    let Some(geom) = op.raster() else {
        return Err(op_err(op, "laser raster is missing image data"));
    };
    let (sin_a, cos_a) = geom.angle().sin_cos();
    let pad = geom.overscan() + geom.scanning_offset().abs();
    let origin_x = geom.origin().0;
    let origin_y = geom.origin().1;
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for (u, v) in [(-pad, 0.0), (geom.width() + pad, 0.0), (geom.width() + pad, geom.height()), (-pad, geom.height())] {
        let x = origin_x + cos_a * u + (-sin_a) * v;
        let y = origin_y + sin_a * u + cos_a * v;
        min_x = min_x.min(x);
        min_y = min_y.min(y);
        max_x = max_x.max(x);
        max_y = max_y.max(y);
    }
    if !min_x.is_finite() {
        return Err(op_err(op, "raster bounds are not finite"));
    }
    Ok((min_x, min_y, max_x, max_y))
}

fn end_xy(block: &ClBlock) -> Option<(f64, f64)> {
    match block {
        ClBlock::Motion(ClMotion::Rapid { to, .. } | ClMotion::Linear { to, .. }) => Some((to.tip().x(), to.tip().y())),
        _ => None,
    }
}

fn rapid(x: f64, y: f64, z: f64) -> Result<ClBlock, CliError> {
    Ok(ClBlock::Motion(ClMotion::Rapid { to: pose(x, y, z)?, feed: None }))
}

fn linear(x: f64, y: f64, z: f64, feed: f64, power: Option<f64>) -> Result<ClBlock, CliError> {
    Ok(ClBlock::Motion(ClMotion::Linear { to: pose(x, y, z)?, feed, role: MoveRole::Cut, power }))
}

/// Binary or ASCII PGM (`P5` / `P2`). Lightness is `value / max`, with 0 black and 1 white.
pub(crate) fn load_pgm(path: &Path) -> Result<GreyImage, CliError> {
    let bytes = std::fs::read(path).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    parse_pgm(&bytes).map_err(|err| CliError::run(format!("{}: {err}", path.display())))
}

fn parse_pgm(bytes: &[u8]) -> Result<GreyImage, String> {
    let mut i = 0usize;
    let magic = token(bytes, &mut i)?;
    if magic != b"P5" && magic != b"P2" {
        return Err("raster image must be a PGM (P2 or P5)".into());
    }
    let width = parse_u32(token(bytes, &mut i)?, "width")?;
    let height = parse_u32(token(bytes, &mut i)?, "height")?;
    let max = parse_u32(token(bytes, &mut i)?, "max value")?;
    if max == 0 || max > 65_535 {
        return Err("PGM max value must be in 1..=65535".into());
    }
    skip_one_whitespace(bytes, &mut i)?;
    let count = usize::try_from(u64::from(width).saturating_mul(u64::from(height))).map_err(|_| "PGM is too large".to_string())?;
    let mut values = Vec::with_capacity(count);
    let two_byte = max > 255;
    if magic == b"P5" {
        let need = count.saturating_mul(if two_byte { 2 } else { 1 });
        let rest = bytes.get(i..).ok_or_else(|| "PGM is truncated".to_string())?;
        if rest.len() < need {
            return Err("PGM is truncated".into());
        }
        let mut offset = 0usize;
        for _ in 0..count {
            let sample = if two_byte {
                let hi = *rest.get(offset).ok_or_else(|| "PGM is truncated".to_string())?;
                let lo = *rest.get(offset + 1).ok_or_else(|| "PGM is truncated".to_string())?;
                offset += 2;
                u32::from(hi) * 256 + u32::from(lo)
            } else {
                let value = *rest.get(offset).ok_or_else(|| "PGM is truncated".to_string())?;
                offset += 1;
                u32::from(value)
            };
            if sample > max {
                return Err("PGM sample is greater than the max value".into());
            }
            values.push(f64::from(sample) / f64::from(max));
        }
    } else {
        for _ in 0..count {
            let sample = parse_u32(token(bytes, &mut i)?, "sample")?;
            if sample > max {
                return Err("PGM sample is greater than the max value".into());
            }
            values.push(f64::from(sample) / f64::from(max));
        }
    }
    GreyImage::new(width, height, values).map_err(|err| err.to_string())
}

fn token<'a>(bytes: &'a [u8], index: &mut usize) -> Result<&'a [u8], String> {
    loop {
        skip_spaces(bytes, index);
        if bytes.get(*index).copied() == Some(b'#') {
            while let Some(ch) = bytes.get(*index) {
                *index += 1;
                if *ch == b'\n' {
                    break;
                }
            }
            continue;
        }
        break;
    }
    let start = *index;
    while let Some(ch) = bytes.get(*index) {
        if ch.is_ascii_whitespace() {
            break;
        }
        *index += 1;
    }
    if start == *index {
        return Err("PGM header is truncated".into());
    }
    Ok(&bytes[start..*index])
}

fn skip_spaces(bytes: &[u8], index: &mut usize) {
    while let Some(ch) = bytes.get(*index) {
        if !ch.is_ascii_whitespace() {
            break;
        }
        *index += 1;
    }
}

fn skip_one_whitespace(bytes: &[u8], index: &mut usize) -> Result<(), String> {
    let Some(ch) = bytes.get(*index) else {
        return Err("PGM header is truncated".into());
    };
    if !ch.is_ascii_whitespace() {
        return Err("PGM header needs whitespace before the pixels".into());
    }
    *index += 1;
    Ok(())
}

fn parse_u32(token: &[u8], what: &str) -> Result<u32, String> {
    let text = std::str::from_utf8(token).map_err(|_| format!("PGM {what} is not a number"))?;
    text.parse::<u32>().map_err(|_| format!("PGM {what} is not a number"))
}

fn op_err(op: &Operation, err: impl std::fmt::Display) -> CliError {
    CliError::run(format!("operation {}: {err}", op.name()))
}

fn pose(x: f64, y: f64, z: f64) -> Result<Pose, CliError> {
    let tip = Vec3::new(x, y, z).map_err(|err| CliError::run(err.to_string()))?;
    Pose::new(tip, Vec3::Z).map_err(|err| CliError::run(err.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p2_and_p5_round_trip_lightness() {
        let p2 = b"P2\n# c\n2 1\n255\n0 255\n";
        let ascii = parse_pgm(p2).unwrap();
        assert_eq!(ascii.width(), 2);
        assert_eq!(ascii.height(), 1);
        assert!((ascii.lightness_at(0, 0).unwrap() - 0.0).abs() < 1e-12);
        assert!((ascii.lightness_at(1, 0).unwrap() - 1.0).abs() < 1e-12);
        let p5 = [b"P5\n2 1\n255\n".as_slice(), &[0u8, 255][..]].concat();
        let binary = parse_pgm(&p5).unwrap();
        assert_eq!(binary, ascii);
    }
}
