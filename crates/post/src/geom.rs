//! Arc geometry shared by the writer and the read-back check.
//!
//! Plane angles increase counter-clockwise when viewed from the positive normal: +Z for XY,
//! +Y for ZX and +X for YZ. That is the RS274 sense of G3. `I` `J` `K` stay axis offsets.

use campanion_ir::{ArcSense, Plane};

use crate::error::{Result, invalid};

pub(crate) type Xyz = [f64; 3];

const TAU: f64 = std::f64::consts::TAU;

pub(crate) fn dist3(a: Xyz, b: Xyz) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// `(u, v, axial)` with `u` × `v` equal to the plane normal.
pub(crate) fn plane_uv(plane: Plane, point: Xyz) -> (f64, f64, f64) {
    match plane {
        Plane::Xy => (point[0], point[1], point[2]),
        Plane::Zx => (point[2], point[0], point[1]),
        Plane::Yz => (point[1], point[2], point[0]),
    }
}

pub(crate) fn compose(plane: Plane, u: f64, v: f64, axial: f64) -> Xyz {
    match plane {
        Plane::Xy => [u, v, axial],
        Plane::Zx => [v, axial, u],
        Plane::Yz => [axial, u, v],
    }
}

pub(crate) fn in_plane_len(plane: Plane, a: Xyz, b: Xyz) -> f64 {
    let (au, av, _) = plane_uv(plane, a);
    let (bu, bv, _) = plane_uv(plane, b);
    (au - bu).hypot(av - bv)
}

pub(crate) fn angle_of(plane: Plane, point: Xyz, center: Xyz) -> f64 {
    let (u, v, _) = plane_uv(plane, point);
    let (cu, cv, _) = plane_uv(plane, center);
    (v - cv).atan2(u - cu)
}

/// Sweep from `a0` to `a1` in `ccw` direction, in `[0, τ)`. A full turn is `0` here.
pub(crate) fn forward_delta(a0: f64, a1: f64, ccw: bool) -> f64 {
    let delta = (a1 - a0).rem_euclid(TAU);
    if ccw { delta } else { (TAU - delta).rem_euclid(TAU) }
}

pub(crate) fn same_radius(start: Xyz, end: Xyz, center: Xyz, plane: Plane) -> Result<f64> {
    let radius = in_plane_len(plane, start, center);
    let end_radius = in_plane_len(plane, end, center);
    if !(radius.is_finite() && radius > 1e-12) {
        return Err(invalid("arc radius must be positive"));
    }
    let slack = 1e-4_f64.max(radius * 1e-5);
    if (radius - end_radius).abs() > slack {
        return Err(invalid("arc end radius does not match the start radius"));
    }
    Ok(radius)
}

/// In-plane sweep of an arc, in `(0, τ]`. A coincident start and end is a full turn.
pub(crate) fn arc_sweep(start: Xyz, end: Xyz, center: Xyz, plane: Plane, sense: ArcSense) -> Result<f64> {
    same_radius(start, end, center, plane)?;
    let a0 = angle_of(plane, start, center);
    let a1 = angle_of(plane, end, center);
    if !a0.is_finite() || !a1.is_finite() {
        return Err(invalid("arc angle is not finite"));
    }
    let chord = in_plane_len(plane, start, end);
    let ccw = matches!(sense, ArcSense::Ccw);
    let delta = forward_delta(a0, a1, ccw);
    let sweep = if chord <= 1e-9 || delta <= 1e-12 { TAU } else { delta };
    if sweep.is_finite() && sweep > 0.0 { Ok(sweep) } else { Err(invalid("arc sweep must be positive")) }
}

/// Point at fraction `t` of the sweep. `t >= 1` with `snap_end` returns `end` exactly.
pub(crate) fn point_at(start: Xyz, end: Xyz, center: Xyz, plane: Plane, sense: ArcSense, sweep: f64, t: f64, snap_end: bool) -> Xyz {
    if snap_end && t >= 1.0 {
        return end;
    }
    let t = t.clamp(0.0, 1.0);
    let signed = if matches!(sense, ArcSense::Ccw) { sweep * t } else { -sweep * t };
    let (su, sv, s_ax) = plane_uv(plane, start);
    let (_, _, e_ax) = plane_uv(plane, end);
    let (cu, cv, _) = plane_uv(plane, center);
    let (ou, ov) = (su - cu, sv - cv);
    let (sin_a, cos_a) = signed.sin_cos();
    let u = cu + ou * cos_a - ov * sin_a;
    let v = cv + ou * sin_a + ov * cos_a;
    compose(plane, u, v, s_ax + (e_ax - s_ax) * t)
}

pub(crate) fn piece_count(sweep: f64, max_sweep: f64) -> Result<u32> {
    if !(sweep.is_finite() && sweep > 0.0) {
        return Err(invalid("sweep must be positive"));
    }
    if !(max_sweep.is_finite() && max_sweep > 0.0) {
        return Err(invalid("max sweep must be positive"));
    }
    let mut n = sweep / max_sweep;
    let nearest = n.round();
    if (n - nearest).abs() <= 1e-8 {
        n = nearest;
    }
    n = n.ceil();
    if !n.is_finite() || !(1.0..=100_000.0).contains(&n) {
        return Err(invalid("arc splits into too many blocks"));
    }
    Ok(n as u32)
}

pub(crate) fn max_step_angle(radius: f64, tol: f64) -> f64 {
    let ratio = if radius <= tol { 2.0 } else { (tol / radius).min(2.0) };
    let cos_half = (1.0 - ratio).clamp(-1.0, 1.0);
    let theta = 2.0 * cos_half.acos();
    if !theta.is_finite() || theta > std::f64::consts::PI { std::f64::consts::PI } else { theta.max(1e-9) }
}

pub(crate) fn chord_segments(sweep: f64, radius: f64, tol: f64) -> Result<u32> {
    if !(tol.is_finite() && tol > 0.0) {
        return Err(invalid("arc_tolerance must be positive to linearise an arc"));
    }
    let step = max_step_angle(radius, tol);
    if step <= 0.0 {
        return Err(invalid("arc segment angle is not positive"));
    }
    let mut n = (sweep / step).ceil();
    let nearest = n.round();
    if (n - nearest).abs() <= 1e-8 {
        n = nearest;
    }
    if !n.is_finite() || !(1.0..=100_000.0).contains(&n) {
        return Err(invalid("arc linearises to more than 100000 segments"));
    }
    Ok(n as u32)
}

/// Absolute centre from a signed radius word. Negative `radius` is the major arc.
pub(crate) fn center_from_radius(start: Xyz, end: Xyz, radius: f64, sense: ArcSense, plane: Plane) -> Result<Xyz> {
    if !(radius.is_finite() && radius.abs() > 0.0) {
        return Err(invalid("arc radius must be non-zero"));
    }
    let (u1, v1, axial) = plane_uv(plane, start);
    let (u2, v2, _) = plane_uv(plane, end);
    let (du, dv) = (u2 - u1, v2 - v1);
    let chord = du.hypot(dv);
    if chord <= 1e-9 {
        return Err(invalid("R word cannot describe a full circle"));
    }
    let abs_r = radius.abs();
    if chord > 2.0 * abs_r + 1e-6 {
        return Err(invalid("arc radius is smaller than the chord"));
    }
    let half = chord * 0.5;
    let h = (abs_r * abs_r - half * half).max(0.0).sqrt();
    let (lx, ly) = (-dv / chord, du / chord);
    let ccw = matches!(sense, ArcSense::Ccw);
    let sign = if ccw == (radius > 0.0) { 1.0 } else { -1.0 };
    Ok(compose(plane, (u1 + u2) * 0.5 + sign * h * lx, (v1 + v2) * 0.5 + sign * h * ly, axial))
}
