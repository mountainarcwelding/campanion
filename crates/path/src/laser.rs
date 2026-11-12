//! Laser raster planning: scan lines, grey-to-power, overscan, and scanning offset.
//!
//! The bitmap is already a light map. This module does not read files. A grey the map sends to
//! zero is power 0. Overscan is path past the image at power 0. Scanning offset is applied along
//! each pass's travel direction so opposite passes agree on the image.

use campanion_geom::{Point, Vec2};

use crate::Result;
use crate::check::{finite, invalid, non_negative, positive};

/// Largest number of pixels a raster image may contain.
pub const MAX_RASTER_PIXELS: u32 = 4_000_000;
/// Largest width or height, in pixels.
pub const MAX_RASTER_DIM: u32 = 8_192;
/// Largest number of feed segments one plan may emit.
pub const MAX_RASTER_SEGMENTS: usize = 262_144;

/// Lightness 0 (black) to 1 (white), row-major with row 0 at the top of the image.
#[derive(Clone, Debug, PartialEq)]
pub struct GreyImage {
    width: u32,
    height: u32,
    lightness: Vec<f64>,
}

impl GreyImage {
    /// `lightness` is row-major, top row first, one value per pixel in `[0, 1]`.
    pub fn new(width: u32, height: u32, lightness: Vec<f64>) -> Result<Self> {
        if width == 0 || height == 0 {
            return Err(invalid("raster image must have a positive size"));
        }
        if width > MAX_RASTER_DIM || height > MAX_RASTER_DIM {
            return Err(invalid(format!("raster image is larger than {MAX_RASTER_DIM} pixels on a side")));
        }
        let pixels = u64::from(width).saturating_mul(u64::from(height));
        if pixels > u64::from(MAX_RASTER_PIXELS) {
            return Err(invalid(format!("raster image has more than {MAX_RASTER_PIXELS} pixels")));
        }
        let expected = usize::try_from(pixels).map_err(|_| invalid("raster image is too large"))?;
        if lightness.len() != expected {
            return Err(invalid("raster image lightness does not match width and height"));
        }
        for value in &lightness {
            if !value.is_finite() || *value < 0.0 || *value > 1.0 {
                return Err(invalid("raster lightness must be in 0..=1"));
            }
        }
        Ok(Self { width, height, lightness })
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    /// Lightness at pixel `(x, y)`, with `y = 0` at the top row.
    pub fn lightness_at(&self, x: u32, y: u32) -> Result<f64> {
        if x >= self.width || y >= self.height {
            return Err(invalid("raster pixel is out of range"));
        }
        let width = usize::try_from(self.width).map_err(|_| invalid("raster image is too large"))?;
        let row = usize::try_from(y).map_err(|_| invalid("raster pixel is out of range"))?;
        let col = usize::try_from(x).map_err(|_| invalid("raster pixel is out of range"))?;
        self.lightness.get(row.saturating_mul(width).saturating_add(col)).copied().ok_or_else(|| invalid("raster pixel is out of range"))
    }
}

/// How image lightness becomes IR power. The result is in `[0, 1]`.
#[derive(Clone, Debug, PartialEq)]
pub enum GreyMap {
    /// `power = 1 - lightness`. Black burns at full power; white is 0.
    Linear,
    /// `power = 1 - lightness.powf(gamma)`. `gamma` must be positive.
    Gamma {
        /// Positive exponent applied to lightness before inversion.
        gamma: f64,
    },
    /// Piecewise-linear map from lightness 0..1 onto the listed powers, which must lie in `[0, 1]`.
    Table {
        /// At least two samples. The first is lightness 0, the last is lightness 1.
        values: Vec<f64>,
    },
}

impl GreyMap {
    fn check(&self) -> Result<()> {
        match self {
            Self::Linear => Ok(()),
            Self::Gamma { gamma } => {
                positive(*gamma, "grey map gamma")?;
                Ok(())
            }
            Self::Table { values } => {
                if values.len() < 2 {
                    return Err(invalid("grey map table needs at least two values"));
                }
                for value in values {
                    if !value.is_finite() || *value < 0.0 || *value > 1.0 {
                        return Err(invalid("grey map table values must be in 0..=1"));
                    }
                }
                Ok(())
            }
        }
    }

    /// Power in `[0, 1]` for a lightness in `[0, 1]`.
    pub fn power(&self, lightness: f64) -> Result<f64> {
        self.check()?;
        if !lightness.is_finite() || !(0.0..=1.0).contains(&lightness) {
            return Err(invalid("raster lightness must be in 0..=1"));
        }
        let power = match self {
            Self::Linear => 1.0 - lightness,
            Self::Gamma { gamma } => 1.0 - lightness.powf(*gamma),
            Self::Table { values } => {
                let last = values.len() - 1;
                let pos = lightness * last as f64;
                let i = pos.floor() as usize;
                if i >= last {
                    *values.last().ok_or_else(|| invalid("grey map table needs at least two values"))?
                } else {
                    let t = pos - i as f64;
                    let a = *values.get(i).ok_or_else(|| invalid("grey map table is short"))?;
                    let b = *values.get(i + 1).ok_or_else(|| invalid("grey map table is short"))?;
                    a * (1.0 - t) + b * t
                }
            }
        };
        if !power.is_finite() || !(0.0..=1.0).contains(&power) {
            return Err(invalid("grey map produced a power outside 0..=1"));
        }
        Ok(power)
    }
}

/// Placement and scan parameters. Lengths are millimetres.
#[derive(Clone, Debug, PartialEq)]
pub struct RasterSpec {
    /// Lower-left corner of the placed image, before rotation.
    pub origin: Point,
    /// Image width along the scan direction at angle 0.
    pub width: f64,
    /// Image height along the step direction at angle 0.
    pub height: f64,
    /// Scan angle in radians. 0 is +X, with +Y the step direction.
    pub angle: f64,
    /// Spacing between scan lines, along the step direction.
    pub step: f64,
    /// Extra travel past each end of the image, at power 0.
    pub overscan: f64,
    /// Shift applied along each pass's travel so opposite passes agree on the image.
    pub scanning_offset: f64,
    /// Lightness-to-power map.
    pub map: GreyMap,
}

impl RasterSpec {
    fn check(&self) -> Result<()> {
        positive(self.width, "raster width")?;
        positive(self.height, "raster height")?;
        finite(self.angle, "raster angle")?;
        positive(self.step, "raster step")?;
        non_negative(self.overscan, "raster overscan")?;
        finite(self.scanning_offset, "scanning offset")?;
        self.map.check()?;
        Ok(())
    }
}

/// One feed on a scan line, already shifted by the scanning offset.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RasterSegment {
    /// Start of the feed, millimetres.
    pub from: Point,
    /// End of the feed, millimetres.
    pub to: Point,
    /// IR power in `[0, 1]`.
    pub power: f64,
}

/// One zigzag pass, including overscan at power 0.
#[derive(Clone, Debug, PartialEq)]
pub struct RasterLine {
    /// Feeds in travel order.
    pub segments: Vec<RasterSegment>,
    /// Unit travel direction of this pass, before the scanning offset is applied.
    pub travel: Vec2,
}

/// Scan the image: zigzag lines, overscan at power 0, and scanning offset on the commanded points.
pub fn plan_raster(image: &GreyImage, spec: &RasterSpec) -> Result<Vec<RasterLine>> {
    spec.check()?;
    let sx = spec.width / f64::from(image.width);
    let sy = spec.height / f64::from(image.height);
    if !(sx.is_finite() && sy.is_finite() && sx > 0.0 && sy > 0.0) {
        return Err(invalid("raster pixel size must be positive"));
    }
    let (sin_a, cos_a) = spec.angle.sin_cos();
    let dir = Vec2::new(cos_a, sin_a)?;
    let normal = Vec2::new(-sin_a, cos_a)?;
    let mut lines = Vec::new();
    let mut segments = 0usize;
    let mut index = 0u32;
    loop {
        let v = (f64::from(index) + 0.5) * spec.step;
        if !v.is_finite() || v >= spec.height {
            break;
        }
        if index > MAX_RASTER_DIM.saturating_mul(4) {
            return Err(invalid("too many raster scan lines"));
        }
        let reverse = index % 2 == 1;
        let travel = if reverse { Vec2::new(-dir.x(), -dir.y())? } else { dir };
        if let Some(line) = plan_line(image, spec, sx, sy, dir, normal, travel, v, reverse)? {
            segments = segments.saturating_add(line.segments.len());
            if segments > MAX_RASTER_SEGMENTS {
                return Err(invalid(format!("raster produced more than {MAX_RASTER_SEGMENTS} segments")));
            }
            lines.push(line);
        }
        index = index.saturating_add(1);
        if index == 0 {
            return Err(invalid("too many raster scan lines"));
        }
    }
    if lines.is_empty() {
        return Err(invalid("raster produced no cuts"));
    }
    Ok(lines)
}

fn plan_line(
    image: &GreyImage,
    spec: &RasterSpec,
    sx: f64,
    sy: f64,
    dir: Vec2,
    normal: Vec2,
    travel: Vec2,
    v: f64,
    reverse: bool,
) -> Result<Option<RasterLine>> {
    let mut edges = vec![-spec.overscan, 0.0, spec.width, spec.width + spec.overscan];
    let mut u = 0.0;
    while u < spec.width {
        edges.push(u);
        u += sx;
        if edges.len() > usize::try_from(MAX_RASTER_DIM).unwrap_or(usize::MAX).saturating_add(8) {
            return Err(invalid("too many raster pixel edges"));
        }
    }
    edges.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    edges.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    if reverse {
        edges.reverse();
    }
    let mut raw = Vec::new();
    for pair in edges.windows(2) {
        let (u0, u1) = (pair[0], pair[1]);
        if (u1 - u0).abs() <= 1e-12 {
            continue;
        }
        let mid = (u0 + u1) * 0.5;
        let power = sample_power(image, spec, sx, sy, mid, v)?;
        raw.push((u0, u1, power));
    }
    let mut merged: Vec<(f64, f64, f64)> = Vec::new();
    for (u0, u1, power) in raw {
        if let Some(last) = merged.last_mut()
            && (last.2 - power).abs() <= 1e-12
        {
            last.1 = u1;
            continue;
        }
        merged.push((u0, u1, power));
    }
    let in_image = merged.iter().any(|(u0, u1, power)| *power > 0.0 && u_overlaps_image(*u0, *u1, spec.width));
    if !in_image {
        return Ok(None);
    }
    let offset = spec.scanning_offset;
    let mut segments = Vec::with_capacity(merged.len());
    for (u0, u1, power) in merged {
        let from = world_point(spec.origin, dir, normal, u0 - offset * travel_sign(reverse), v)?;
        let to = world_point(spec.origin, dir, normal, u1 - offset * travel_sign(reverse), v)?;
        if from.distance(to) <= 1e-12 {
            continue;
        }
        segments.push(RasterSegment { from, to, power });
    }
    if segments.is_empty() {
        return Ok(None);
    }
    Ok(Some(RasterLine { segments, travel }))
}

fn travel_sign(reverse: bool) -> f64 {
    if reverse { -1.0 } else { 1.0 }
}

fn u_overlaps_image(u0: f64, u1: f64, width: f64) -> bool {
    let lo = u0.min(u1);
    let hi = u0.max(u1);
    hi > 0.0 && lo < width
}

fn world_point(origin: Point, dir: Vec2, normal: Vec2, u: f64, v: f64) -> Result<Point> {
    Ok(Point::new(origin.x() + dir.x() * u + normal.x() * v, origin.y() + dir.y() * u + normal.y() * v)?)
}

fn sample_power(image: &GreyImage, spec: &RasterSpec, sx: f64, sy: f64, u: f64, v: f64) -> Result<f64> {
    if u < 0.0 || u >= spec.width || v < 0.0 || v >= spec.height {
        return Ok(0.0);
    }
    let ix = ((u / sx).floor() as i64).clamp(0, i64::from(image.width) - 1);
    let iy_world = ((v / sy).floor() as i64).clamp(0, i64::from(image.height) - 1);
    let iy_top = i64::from(image.height) - 1 - iy_world;
    let x = u32::try_from(ix).map_err(|_| invalid("raster pixel is out of range"))?;
    let y = u32::try_from(iy_top).map_err(|_| invalid("raster pixel is out of range"))?;
    spec.map.power(image.lightness_at(x, y)?)
}

/// Undo the scanning offset of a commanded point using that pass's travel direction.
pub fn unoffset(point: Point, travel: Vec2, scanning_offset: f64) -> Result<Point> {
    Ok(Point::new(point.x() + travel.x() * scanning_offset, point.y() + travel.y() * scanning_offset)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn image(width: u32, height: u32, value: f64) -> GreyImage {
        GreyImage::new(width, height, vec![value; (width * height) as usize]).unwrap()
    }

    fn checker(width: u32, height: u32) -> GreyImage {
        let mut values = Vec::with_capacity((width * height) as usize);
        for y in 0..height {
            for x in 0..width {
                values.push(if (x + y) % 2 == 0 { 0.0 } else { 1.0 });
            }
        }
        GreyImage::new(width, height, values).unwrap()
    }

    fn spec(width: f64, height: f64, step: f64, overscan: f64, offset: f64) -> RasterSpec {
        RasterSpec { origin: Point::new(0.0, 0.0).unwrap(), width, height, angle: 0.0, step, overscan, scanning_offset: offset, map: GreyMap::Linear }
    }

    #[test]
    fn oversized_images_are_refused() {
        let err = GreyImage::new(MAX_RASTER_DIM + 1, 1, vec![0.0; MAX_RASTER_DIM as usize + 1]).unwrap_err();
        assert!(err.to_string().contains("pixels on a side"), "{err}");
        let wide = MAX_RASTER_PIXELS + 1;
        // A 2001×2001 image is over the pixel cap and under the side cap.
        let w = 2_001u32;
        let h = 2_001u32;
        assert!(u64::from(w) * u64::from(h) > u64::from(MAX_RASTER_PIXELS));
        let err = GreyImage::new(w, h, vec![0.0; (w * h) as usize]).unwrap_err();
        assert!(err.to_string().contains("more than"), "{err}");
        let _ = wide;
    }

    #[test]
    fn linear_map_sends_white_to_zero_and_black_to_one() {
        assert!((GreyMap::Linear.power(1.0).unwrap() - 0.0).abs() < 1e-12);
        assert!((GreyMap::Linear.power(0.0).unwrap() - 1.0).abs() < 1e-12);
        assert!((GreyMap::Linear.power(0.25).unwrap() - 0.75).abs() < 1e-12);
        let gamma = GreyMap::Gamma { gamma: 2.0 };
        assert!((gamma.power(0.0).unwrap() - 1.0).abs() < 1e-12);
        assert!((gamma.power(1.0).unwrap() - 0.0).abs() < 1e-12);
        let table = GreyMap::Table { values: vec![1.0, 0.0] };
        assert!((table.power(0.0).unwrap() - 1.0).abs() < 1e-12);
        assert!((table.power(1.0).unwrap() - 0.0).abs() < 1e-12);
    }

    #[test]
    fn overscan_is_power_zero_and_a_white_pixel_is_not_a_cut() {
        let img = GreyImage::new(2, 1, vec![0.0, 1.0]).unwrap();
        let lines = plan_raster(&img, &spec(2.0, 1.0, 1.0, 0.5, 0.0)).unwrap();
        assert_eq!(lines.len(), 1);
        let powers: Vec<(f64, f64, f64)> = lines[0].segments.iter().map(|seg| (seg.from.x(), seg.to.x(), seg.power)).collect();
        assert_eq!(powers.len(), 3, "{powers:?}");
        assert!((powers[0].0 + 0.5).abs() < 1e-9 && (powers[0].1 - 0.0).abs() < 1e-9 && powers[0].2 == 0.0, "{powers:?}");
        assert!((powers[1].0 - 0.0).abs() < 1e-9 && (powers[1].1 - 1.0).abs() < 1e-9 && (powers[1].2 - 1.0).abs() < 1e-12, "{powers:?}");
        assert!((powers[2].0 - 1.0).abs() < 1e-9 && (powers[2].1 - 2.5).abs() < 1e-9 && powers[2].2 == 0.0, "{powers:?}");
    }

    #[test]
    fn blank_image_is_refused() {
        let err = plan_raster(&image(4, 2, 1.0), &spec(4.0, 2.0, 1.0, 1.0, 0.0)).unwrap_err();
        assert!(err.to_string().contains("no cuts"), "{err}");
    }

    #[test]
    fn zigzag_reverses_travel_and_offset_keeps_image_points() {
        let img = image(4, 2, 0.0);
        let offset = 0.3;
        let lines = plan_raster(&img, &spec(4.0, 2.0, 1.0, 0.0, offset)).unwrap();
        assert_eq!(lines.len(), 2);
        assert!((lines[0].travel.x() - 1.0).abs() < 1e-12);
        assert!((lines[1].travel.x() + 1.0).abs() < 1e-12);
        // A burn that covers image u in [0, 4] is commanded at u - offset * travel_sign.
        let first = &lines[0].segments[0];
        let last = &lines[1].segments[0];
        // Forward starts at -offset (image u=0). Reverse starts at width + offset (image u=width).
        assert!((first.from.x() + offset).abs() < 1e-9, "{} vs {}", first.from.x(), -offset);
        assert!((last.from.x() - (4.0 + offset)).abs() < 1e-9, "{} vs {}", last.from.x(), 4.0 + offset);
        let recovered = unoffset(first.from, lines[0].travel, offset).unwrap();
        assert!(recovered.almost_eq(Point::new(0.0, 0.5).unwrap(), 1e-9));
        let recovered = unoffset(last.from, lines[1].travel, offset).unwrap();
        assert!(recovered.almost_eq(Point::new(4.0, 1.5).unwrap(), 1e-9));
    }

    fn pixel_of(image: &GreyImage, spec: &RasterSpec, point: Point) -> Option<(u32, u32)> {
        let (sin_a, cos_a) = spec.angle.sin_cos();
        let dir = Vec2::new(cos_a, sin_a).unwrap();
        let normal = Vec2::new(-sin_a, cos_a).unwrap();
        let u = (point.x() - spec.origin.x()) * dir.x() + (point.y() - spec.origin.y()) * dir.y();
        let v = (point.x() - spec.origin.x()) * normal.x() + (point.y() - spec.origin.y()) * normal.y();
        if u < 0.0 || u >= spec.width || v < 0.0 || v >= spec.height {
            return None;
        }
        let sx = spec.width / f64::from(image.width);
        let sy = spec.height / f64::from(image.height);
        let ix = (u / sx).floor() as i64;
        let iy_world = (v / sy).floor() as i64;
        if ix < 0 || iy_world < 0 {
            return None;
        }
        let iy_top = i64::from(image.height) - 1 - iy_world;
        Some((ix as u32, iy_top as u32))
    }

    proptest! {
        #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]

        #[test]
        fn coverage_offset_and_zero_power(
            width in 1u32..=6,
            height in 1u32..=6,
            raw in proptest::collection::vec(0u8..=255, 1..=36),
            overscan in 0u8..=5,
            offset_ticks in 0u8..=10,
            gamma_pick in 0u8..=2,
        ) {
            let mut values = Vec::with_capacity((width * height) as usize);
            for i in 0..(width * height) as usize {
                values.push(f64::from(*raw.get(i).unwrap_or(&0)) / 255.0);
            }
            let image = GreyImage::new(width, height, values).unwrap();
            let map = match gamma_pick {
                1 => GreyMap::Gamma { gamma: 2.2 },
                2 => GreyMap::Table { values: vec![1.0, 0.5, 0.0] },
                _ => GreyMap::Linear,
            };
            let pixel = 1.0;
            let mut spec = spec(f64::from(width) * pixel, f64::from(height) * pixel, pixel, f64::from(overscan) * 0.2, f64::from(offset_ticks) * 0.05);
            spec.map = map.clone();
            let mut nonzero = 0u32;
            for y in 0..height {
                for x in 0..width {
                    if map.power(image.lightness_at(x, y).unwrap()).unwrap() > 1e-12 {
                        nonzero += 1;
                    }
                }
            }
            if nonzero == 0 {
                prop_assert!(plan_raster(&image, &spec).is_err());
                return Ok(());
            }
            let lines = plan_raster(&image, &spec).unwrap();
            let mut seen = vec![false; (width * height) as usize];
            for line in &lines {
                for seg in &line.segments {
                    prop_assert!((0.0..=1.0).contains(&seg.power));
                    let from = unoffset(seg.from, line.travel, spec.scanning_offset).unwrap();
                    let to = unoffset(seg.to, line.travel, spec.scanning_offset).unwrap();
                    let length = from.distance(to);
                    let steps = ((length / 0.1).ceil() as i32).max(1);
                    for i in 0..=steps {
                        let t = if length <= 1e-12 { 0.5 } else { (f64::from(i) + 0.5) / f64::from(steps + 1) };
                        // Stay inside the segment so a boundary does not leak into the next pixel.
                        let t = t.clamp(1e-6, 1.0 - 1e-6);
                        let point = Point::new(from.x() + (to.x() - from.x()) * t, from.y() + (to.y() - from.y()) * t).unwrap();
                        if let Some((x, y)) = pixel_of(&image, &spec, point) {
                            let expected = map.power(image.lightness_at(x, y).unwrap()).unwrap();
                            if seg.power > 1e-12 {
                                prop_assert!((seg.power - expected).abs() <= 1e-9, "pixel {x},{y} power {} vs {expected}", seg.power);
                                seen[(y * width + x) as usize] = true;
                            } else {
                                prop_assert!(expected <= 1e-12, "zero-power segment covered a live pixel {x},{y}");
                            }
                        } else {
                            prop_assert!(seg.power <= 1e-12, "overscan carried power {}", seg.power);
                        }
                    }
                }
            }
            for y in 0..height {
                for x in 0..width {
                    let expected = map.power(image.lightness_at(x, y).unwrap()).unwrap();
                    if expected > 1e-12 {
                        prop_assert!(seen[(y * width + x) as usize], "pixel {x},{y} was not covered");
                    }
                }
            }
        }
    }

    #[test]
    fn checker_has_alternating_power_and_covers_black_squares() {
        let img = checker(4, 2);
        let lines = plan_raster(&img, &spec(4.0, 2.0, 1.0, 0.0, 0.0)).unwrap();
        assert_eq!(lines.len(), 2);
        let mut live = 0u32;
        for line in &lines {
            for seg in &line.segments {
                if seg.power > 0.0 {
                    live += 1;
                    assert!((seg.power - 1.0).abs() < 1e-12);
                } else {
                    assert_eq!(seg.power, 0.0);
                }
            }
        }
        assert_eq!(live, 4);
    }
}
