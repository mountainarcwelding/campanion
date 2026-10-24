//! Rotary frame mapping shared by the writer and the read-back check.
//!
//! A pose with an angle is written in the machine frame: [`part_to_machine`] supplies `X` `Y` `Z`
//! and the IR angle is the A or B word. The inverse, [`machine_to_part`], is what read-back
//! compares to the program.
//!
//! Inverse-time feed uses the straight-line distance between the part-frame tips. See the post
//! crate README.

use campanion_ir::{ArcSense, Plane, Pose, RotarySetup, Vec3, machine_to_part, part_to_machine, rotate_point, rotate_vector};

use crate::error::{Result, invalid};
use crate::geom::{self, Xyz};

/// Two angles name the same index when they differ by less than this.
const ANGLE_EPS: f64 = 1e-9;

pub(crate) fn angle_changes(prev: Option<f64>, next: Option<f64>) -> bool {
    match (prev, next) {
        (Some(left), Some(right)) => (left - right).abs() > ANGLE_EPS,
        (None, Some(_)) => true,
        _ => false,
    }
}

pub(crate) fn angles_equal(left: f64, right: f64) -> bool {
    (left - right).abs() <= ANGLE_EPS
}

pub(crate) fn plane_normal(plane: Plane) -> Vec3 {
    match plane {
        Plane::Xy => Vec3::Z,
        Plane::Zx => Vec3::Y,
        Plane::Yz => Vec3::X,
    }
}

/// Principal plane of `normal`, and whether it points opposite that plane's positive normal.
///
/// `None` when the arc does not lie in XY, ZX, or YZ after the rotary mapping. RS274 has no
/// tilted-plane arc, so the writer linearises that case.
pub(crate) fn principal_plane(normal: Vec3) -> Option<(Plane, bool)> {
    let (x, y, z) = (normal.x(), normal.y(), normal.z());
    let (ax, ay, az) = (x.abs(), y.abs(), z.abs());
    const ALIGNED: f64 = 1.0 - 1e-6;
    if az >= ALIGNED && ax <= 1e-6 && ay <= 1e-6 {
        Some((Plane::Xy, z < 0.0))
    } else if ay >= ALIGNED && ax <= 1e-6 && az <= 1e-6 {
        Some((Plane::Zx, y < 0.0))
    } else if ax >= ALIGNED && ay <= 1e-6 && az <= 1e-6 {
        Some((Plane::Yz, x < 0.0))
    } else {
        None
    }
}

pub(crate) fn flip_sense(sense: ArcSense) -> ArcSense {
    match sense {
        ArcSense::Cw => ArcSense::Ccw,
        ArcSense::Ccw => ArcSense::Cw,
    }
}

/// Sense in the part frame. A rotation that reverses the plane normal flips G2 and G3.
pub(crate) fn part_sense(setup: &RotarySetup, angle: f64, plane: Plane, written: ArcSense) -> ArcSense {
    let rotated = rotate_vector(setup, plane_normal(plane), angle);
    match principal_plane(rotated) {
        Some((_, true)) => flip_sense(written),
        _ => written,
    }
}

pub(crate) fn xyz_of(point: Vec3) -> Xyz {
    [point.x(), point.y(), point.z()]
}

pub(crate) fn vec3_of(point: Xyz) -> Result<Vec3> {
    Vec3::new(point[0], point[1], point[2]).map_err(|err| invalid(err.to_string()))
}

pub(crate) fn pose_at(point: Xyz) -> Result<Pose> {
    Pose::new(vec3_of(point)?, Vec3::Z).map_err(|err| invalid(err.to_string()))
}

pub(crate) fn machine_tip(setup: &RotarySetup, part: Xyz, angle: f64) -> Result<Xyz> {
    let pose = pose_at(part)?.with_rotary(angle).map_err(|err| invalid(err.to_string()))?;
    Ok(xyz_of(part_to_machine(setup, pose, angle).tip()))
}

pub(crate) fn part_tip_at(setup: &RotarySetup, machine: Xyz, angle: f64) -> Result<Xyz> {
    let pose = pose_at(machine)?.with_rotary(angle).map_err(|err| invalid(err.to_string()))?;
    Ok(xyz_of(machine_to_part(setup, pose, angle).tip()))
}

pub(crate) fn part_center(setup: &RotarySetup, center: Xyz, angle: f64) -> Result<Xyz> {
    Ok(xyz_of(rotate_point(setup, vec3_of(center)?, -angle)))
}

/// Arc in the machine frame when the rotated plane is still XY, ZX, or YZ.
pub(crate) struct MappedArc {
    pub(crate) plane: Plane,
    pub(crate) sense: ArcSense,
    pub(crate) end: Xyz,
    pub(crate) center: Xyz,
}

pub(crate) fn map_arc(setup: &RotarySetup, angle: f64, plane: Plane, sense: ArcSense, part_end: Xyz, part_center: Xyz) -> Result<Option<MappedArc>> {
    let rotated = rotate_vector(setup, plane_normal(plane), angle);
    let Some((mach_plane, flipped)) = principal_plane(rotated) else {
        return Ok(None);
    };
    let sense = if flipped { flip_sense(sense) } else { sense };
    Ok(Some(MappedArc {
        plane: mach_plane,
        sense,
        end: machine_tip(setup, part_end, angle)?,
        center: xyz_of(rotate_point(setup, vec3_of(part_center)?, angle)),
    }))
}

pub(crate) struct CurveSample {
    pub(crate) part: Xyz,
    pub(crate) angle: f64,
}

/// Chord samples of a part-frame arc. The angle is linear in the sweep parameter.
pub(crate) fn sample_curve(
    start: Xyz,
    end: Xyz,
    center: Xyz,
    plane: Plane,
    sense: ArcSense,
    sweep: f64,
    start_angle: f64,
    end_angle: f64,
    tol: f64,
) -> Result<Vec<CurveSample>> {
    let radius = geom::in_plane_len(plane, start, center);
    let segments = geom::chord_segments(sweep, radius, tol)?;
    let mut out = Vec::with_capacity(segments as usize);
    for index in 1..=segments {
        let t = if index == segments { 1.0 } else { f64::from(index) / f64::from(segments) };
        let part = geom::point_at(start, end, center, plane, sense, sweep, t, true);
        let angle = start_angle + (end_angle - start_angle) * t;
        out.push(CurveSample { part, angle });
    }
    Ok(out)
}

/// Compare tolerance for an A/B word: at least one thousandth of a degree, and at least one
/// unit of the word's format resolution.
pub(crate) fn angle_tolerance(decimals: u8) -> f64 {
    10_f64.powi(-i32::from(decimals)).max(0.001)
}

/// Inverse-time `F` word.
///
/// `length` is the straight-line distance in millimetres between the part-frame tips, not the
/// machine-frame distance. The move time in minutes is `length / feed` with `feed` in
/// millimetres per minute, so `F = feed / length = 1 / minutes`.
pub(crate) fn inverse_minutes_feed(feed: f64, length: f64) -> Result<f64> {
    if !(length.is_finite() && length > 1e-9) {
        return Err(invalid("inverse time feed needs a positive distance between part-frame tips"));
    }
    let value = feed / length;
    if value.is_finite() { Ok(value) } else { Err(invalid("inverse time feed is not finite")) }
}

/// Straight-line distance between indexed lathe tips after C rotates them into the part frame.
///
/// `tip[0]` is the radius and `tip[1]` is the cross-slide offset. The length is the chord, not
/// the arc. Fallback face and cylinder moves already store part-frame tips, so they use
/// [`geom::dist3`] on those tips instead.
pub(crate) fn lathe_indexed_length(start: Xyz, end: Xyz, c0_deg: f64, c1_deg: f64) -> f64 {
    geom::dist3(spin_tip(start, c0_deg), spin_tip(end, c1_deg))
}

fn spin_tip(tip: Xyz, c_deg: f64) -> Xyz {
    let (sin, cos) = c_deg.to_radians().sin_cos();
    [tip[0] * cos - tip[1] * sin, tip[0] * sin + tip[1] * cos, tip[2]]
}
