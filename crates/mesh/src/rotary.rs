//! Rotary (4-axis) drop-cutter lines.
//!
//! Each angle turns the mesh about the rotary axis by the right-hand rule, then builds one
//! [`DropCutter`] and samples the tool tip along the line above that axis. The cost is one
//! drop-cutter build per angle. Misses store `floor_radius`; a hit is the tip height above the
//! axis and is not raised to the floor.

use crate::{Cutter, DropCutter, Mesh, Result, Triangle, Vec3, invalid};

/// Most angles accepted in one [`rotary_lines`] call.
const MAX_ANGLES: usize = 100_000;

/// Most `(u, radius)` samples across every angle.
const MAX_SAMPLES: usize = 1_000_000;

/// Direction of the rotary axis line.
///
/// [`RotaryLineAxis::X`] is an A axis (right-hand about +X). [`RotaryLineAxis::Y`] is a B axis
/// (right-hand about +Y).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotaryLineAxis {
    /// Rotation about +X. Samples vary X and sit at the axis Y.
    X,
    /// Rotation about +Y. Samples vary Y and sit at the axis X.
    Y,
}

/// One constant-angle drop-cutter pass above the rotary axis.
///
/// `angle_deg` is the right-hand rotation, in degrees, applied to the mesh before sampling.
/// `samples` are `(u, radius)` with `u` along the axis. `radius` is `tip_z - axis_z` on a hit
/// and `floor_radius` on a miss.
#[derive(Clone, Debug, PartialEq)]
pub struct RotaryLine {
    pub angle_deg: f64,
    pub samples: Vec<(f64, f64)>,
}

/// Sample drop-cutter tip radii along the line above a rotary axis.
///
/// For each entry of `angles_deg` the mesh is rotated by that angle about `axis` through
/// `axis_point` (right-hand rule about +X or +Y). One [`DropCutter`] is built for that turned
/// mesh and queried along the line above the axis: machine Y is the axis Y for
/// [`RotaryLineAxis::X`], and machine X is the axis X for [`RotaryLineAxis::Y`]. `u` runs over
/// `u_range`, which is inclusive. Both ends are included and the spacing is at most `u_step`.
///
/// An empty `angles_deg` returns an empty list and builds nothing. The angles are stored as
/// given; this function does not wrap or sort them.
pub fn rotary_lines(
    mesh: &Mesh,
    cutter: Cutter,
    axis: RotaryLineAxis,
    axis_point: (f64, f64, f64),
    u_range: (f64, f64),
    angles_deg: &[f64],
    u_step: f64,
    floor_radius: f64,
) -> Result<Vec<RotaryLine>> {
    validate(axis_point, u_range, angles_deg, u_step, floor_radius)?;
    let sample_count = position_count(u_range.1 - u_range.0, u_step)?;
    if angles_deg.len().saturating_mul(sample_count) > MAX_SAMPLES {
        return Err(invalid(format!("too many rotary samples: limit is {MAX_SAMPLES}")));
    }
    let u_at = coordinates(u_range.0, u_range.1, sample_count)?;
    let origin = Vec3::new(axis_point.0, axis_point.1, axis_point.2);
    let mut lines = Vec::with_capacity(angles_deg.len());
    for &angle_deg in angles_deg {
        let turned = rotate_mesh(mesh, axis, origin, angle_deg)?;
        let dc = DropCutter::new(&turned, cutter)?;
        let mut samples = Vec::with_capacity(u_at.len());
        for &u in &u_at {
            let (x, y) = sample_xy(axis, axis_point, u);
            let radius = match dc.tip_z(x, y) {
                Some(z) => z - axis_point.2,
                None => floor_radius,
            };
            if !radius.is_finite() {
                return Err(invalid("rotary tip radius is not finite"));
            }
            samples.push((u, radius));
        }
        lines.push(RotaryLine { angle_deg, samples });
    }
    Ok(lines)
}

fn validate(axis_point: (f64, f64, f64), u_range: (f64, f64), angles_deg: &[f64], u_step: f64, floor_radius: f64) -> Result<()> {
    if !axis_point.0.is_finite() || !axis_point.1.is_finite() || !axis_point.2.is_finite() {
        return Err(invalid("rotary axis point is not finite"));
    }
    if !u_range.0.is_finite() || !u_range.1.is_finite() {
        return Err(invalid("rotary u range is not finite"));
    }
    if u_range.0 > u_range.1 {
        return Err(invalid("rotary u minimum is greater than maximum"));
    }
    if !u_step.is_finite() || u_step <= 0.0 {
        return Err(invalid("rotary u step must be finite and positive"));
    }
    if !floor_radius.is_finite() {
        return Err(invalid("floor radius must be finite"));
    }
    if angles_deg.len() > MAX_ANGLES {
        return Err(invalid(format!("too many rotary angles: {} > {MAX_ANGLES}", angles_deg.len())));
    }
    if angles_deg.iter().any(|angle| !angle.is_finite()) {
        return Err(invalid("rotary angle is not finite"));
    }
    Ok(())
}

/// `ceil(span / step) + 1` positions, or one position when `span` is zero.
fn position_count(span: f64, step: f64) -> Result<usize> {
    if span == 0.0 {
        return Ok(1);
    }
    let mut intervals = (span / step).ceil();
    if !intervals.is_finite() || intervals < 1.0 {
        return Err(invalid(format!("too many rotary samples: limit is {MAX_SAMPLES}")));
    }
    let mut guard = 0u32;
    while span > step * intervals {
        intervals += 1.0;
        guard += 1;
        if guard > 8 || !intervals.is_finite() {
            return Err(invalid("rotary u spacing is not usable"));
        }
    }
    let count = intervals + 1.0;
    if !count.is_finite() || count > MAX_SAMPLES as f64 {
        return Err(invalid(format!("too many rotary samples: limit is {MAX_SAMPLES}")));
    }
    Ok(count as usize)
}

fn coordinates(start: f64, end: f64, count: usize) -> Result<Vec<f64>> {
    if count == 0 {
        return Err(invalid("rotary sampling produced no positions"));
    }
    if count == 1 {
        return Ok(vec![start]);
    }
    let mut out = Vec::with_capacity(count);
    let gaps = (count - 1) as f64;
    let span = end - start;
    for index in 0..count - 1 {
        let value = start + span * (index as f64) / gaps;
        if !value.is_finite() {
            return Err(invalid("rotary u coordinate is not finite"));
        }
        out.push(value);
    }
    out.push(end);
    Ok(out)
}

fn sample_xy(axis: RotaryLineAxis, axis_point: (f64, f64, f64), u: f64) -> (f64, f64) {
    match axis {
        RotaryLineAxis::X => (u, axis_point.1),
        RotaryLineAxis::Y => (axis_point.0, u),
    }
}

fn rotate_mesh(mesh: &Mesh, axis: RotaryLineAxis, origin: Vec3, deg: f64) -> Result<Mesh> {
    let (cos, sin) = angle_cs(deg)?;
    let triangles = mesh
        .triangles()
        .iter()
        .map(|tri| Triangle {
            a: rotate_point(axis, origin, tri.a, cos, sin),
            b: rotate_point(axis, origin, tri.b, cos, sin),
            c: rotate_point(axis, origin, tri.c, cos, sin),
        })
        .collect();
    Mesh::new(triangles)
}

fn angle_cs(deg: f64) -> Result<(f64, f64)> {
    let theta = deg.to_radians();
    let cos = theta.cos();
    let sin = theta.sin();
    if cos.is_finite() && sin.is_finite() { Ok((cos, sin)) } else { Err(invalid("rotary angle is not usable")) }
}

/// Right-hand rotation about +X or +Y through `origin`.
fn rotate_point(axis: RotaryLineAxis, origin: Vec3, point: Vec3, cos: f64, sin: f64) -> Vec3 {
    let x = point.x - origin.x;
    let y = point.y - origin.y;
    let z = point.z - origin.z;
    let (x, y, z) = match axis {
        RotaryLineAxis::X => (x, y * cos - z * sin, y * sin + z * cos),
        RotaryLineAxis::Y => (x * cos + z * sin, y, -x * sin + z * cos),
    };
    Vec3::new(x + origin.x, y + origin.y, z + origin.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MeshError;
    use proptest::prelude::*;

    fn ball(radius: f64) -> Cutter {
        Cutter::new_ball(radius).expect("ball")
    }

    fn close(actual: f64, expected: f64, tol: f64) {
        assert!((actual - expected).abs() <= tol, "{actual} != {expected} within {tol}");
    }

    /// Closed strip of a surface of revolution. `theta = 0` puts a vertex on +Z.
    /// `radius_at(u, theta)` must be positive. Stations are increasing in `u`.
    fn revolved(axis: RotaryLineAxis, origin: (f64, f64, f64), stations: &[(f64, f64)], segments: usize, bump: impl Fn(f64, f64) -> f64) -> Mesh {
        assert!(segments >= 3);
        let mut rings = Vec::with_capacity(stations.len());
        for &(u, base) in stations {
            let mut ring = Vec::with_capacity(segments);
            for index in 0..segments {
                let theta = index as f64 * std::f64::consts::TAU / segments as f64;
                let radius = base + bump(u, theta);
                ring.push(ring_point(axis, origin, u, radius, theta));
            }
            rings.push(ring);
        }
        let mut triangles = Vec::new();
        for station in 0..stations.len() - 1 {
            for index in 0..segments {
                let next = (index + 1) % segments;
                let a = rings[station][index];
                let b = rings[station + 1][index];
                let c = rings[station][next];
                let d = rings[station + 1][next];
                triangles.push(Triangle { a, b, c });
                triangles.push(Triangle { a: b, b: d, c });
            }
        }
        Mesh::new(triangles).expect("revolved mesh")
    }

    fn ring_point(axis: RotaryLineAxis, origin: (f64, f64, f64), u: f64, radius: f64, theta: f64) -> Vec3 {
        let outward_y = radius * theta.sin();
        let outward_z = radius * theta.cos();
        match axis {
            RotaryLineAxis::X => Vec3::new(origin.0 + u, origin.1 + outward_y, origin.2 + outward_z),
            RotaryLineAxis::Y => Vec3::new(origin.0 + outward_y, origin.1 + u, origin.2 + outward_z),
        }
    }

    fn cylinder(axis: RotaryLineAxis, origin: (f64, f64, f64), radius: f64, u0: f64, u1: f64, segments: usize) -> Mesh {
        revolved(axis, origin, &[(u0, radius), (u1, radius)], segments, |_, _| 0.0)
    }

    fn full_turn(step: f64) -> Vec<f64> {
        let mut angles = Vec::new();
        let mut angle = 0.0;
        while angle < 360.0 - 1e-9 {
            angles.push(angle);
            angle += step;
        }
        angles.push(360.0);
        angles
    }

    #[test]
    fn cylinder_ball_tip_radius_is_the_cylinder_radius() {
        let radius = 15.0;
        let tool = 3.0;
        // 180 segments put a vertex every 2°. 0°, 90° and 180° land that vertex on +Z.
        let segments = 180;
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), radius, -20.0, 20.0, segments);
        let lines = rotary_lines(&mesh, ball(tool), RotaryLineAxis::X, (0.0, 0.0, 0.0), (-10.0, 10.0), &[0.0, 90.0, 180.0], 5.0, 0.0).expect("lines");
        assert_eq!(lines.len(), 3);
        for line in &lines {
            assert!(line.samples.len() >= 2);
            for &(u, tip) in &line.samples {
                assert!((-10.0..=10.0).contains(&u));
                close(tip, radius, 1e-6);
            }
        }
    }

    #[test]
    fn cylinder_off_vertex_sits_on_the_facet_chord() {
        let radius = 15.0;
        let segments = 180;
        let half = 180.0 / segments as f64;
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), radius, -20.0, 20.0, segments);
        let lines = rotary_lines(&mesh, ball(3.0), RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 0.0), &[half], 1.0, 0.0).expect("lines");
        let tip = lines[0].samples[0].1;
        close(tip, radius * half.to_radians().cos(), 1e-4);
    }

    #[test]
    fn flat_and_bull_on_an_aligned_cylinder_also_tip_at_the_radius() {
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), 10.0, -5.0, 5.0, 72);
        let flat = Cutter::new_flat(2.0).expect("flat");
        let bull = Cutter::new_bull(4.0, 1.0).expect("bull");
        for cutter in [flat, bull] {
            let lines = rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 0.0), &[0.0], 1.0, -1.0).expect("lines");
            close(lines[0].samples[0].1, 10.0, 1e-6);
        }
    }

    #[test]
    fn cone_tip_radius_is_linear_with_the_ball_offset() {
        let r0 = 10.0;
        let r1 = 20.0;
        let length = 40.0;
        let slope = (r1 - r0) / length;
        let tool = 2.0;
        let axial = (1.0_f64 + slope * slope).sqrt();
        let offset = tool * (axial - 1.0);
        let mesh = revolved(RotaryLineAxis::X, (0.0, 0.0, 0.0), &[(0.0, r0), (length, r1)], 72, |_, _| 0.0);
        let lines = rotary_lines(&mesh, ball(tool), RotaryLineAxis::X, (0.0, 0.0, 0.0), (5.0, 35.0), &[0.0], 5.0, 0.0).expect("lines");
        let samples = &lines[0].samples;
        assert!(samples.len() >= 3);
        for &(u, tip) in samples {
            let expected = r0 + slope * u + offset;
            close(tip, expected, 1e-4);
        }
        for window in samples.windows(2) {
            let (u0, z0) = window[0];
            let (u1, z1) = window[1];
            let rise = (z1 - z0) / (u1 - u0);
            close(rise, slope, 1e-4);
        }
    }

    #[test]
    fn offset_axis_and_b_axis_keep_the_cylinder_radius() {
        let origin = (4.0, 2.0, 3.0);
        // 72 segments put a vertex every 5°, so 90° still has a vertex on +Z.
        let mesh = cylinder(RotaryLineAxis::X, origin, 10.0, -8.0, 8.0, 72);
        let lines = rotary_lines(&mesh, ball(2.0), RotaryLineAxis::X, origin, (-2.0, 2.0), &[0.0, 90.0], 2.0, 0.0).expect("a");
        for line in &lines {
            for &(_, tip) in &line.samples {
                close(tip, 10.0, 1e-6);
            }
        }

        let mesh_b = cylinder(RotaryLineAxis::Y, origin, 8.0, -6.0, 6.0, 72);
        let lines_b = rotary_lines(&mesh_b, ball(1.5), RotaryLineAxis::Y, origin, (-2.0, 2.0), &[0.0, 180.0], 2.0, 0.0).expect("b");
        for line in &lines_b {
            for &(_, tip) in &line.samples {
                close(tip, 8.0, 1e-6);
            }
        }
    }

    #[test]
    fn a_miss_uses_the_floor_radius_and_an_empty_angle_list_builds_nothing() {
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), 5.0, -2.0, 2.0, 12);
        let lines = rotary_lines(&mesh, ball(1.0), RotaryLineAxis::X, (0.0, 0.0, 0.0), (100.0, 100.0), &[15.0], 1.0, 2.5).expect("miss");
        assert_eq!(lines[0].samples, vec![(100.0, 2.5)]);
        let empty = rotary_lines(&mesh, ball(1.0), RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0), &[], 0.5, 0.0).expect("empty");
        assert!(empty.is_empty());
        let bare = Mesh::new(vec![]).expect("empty mesh");
        let missed = rotary_lines(&bare, ball(1.0), RotaryLineAxis::Y, (1.0, 2.0, 3.0), (0.0, 0.0), &[0.0], 1.0, -4.0).expect("bare");
        assert_eq!(missed[0].samples, vec![(0.0, -4.0)]);
    }

    #[test]
    fn samples_include_both_ends_and_stay_within_the_step() {
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), 4.0, 0.0, 10.0, 8);
        let lines = rotary_lines(&mesh, ball(1.0), RotaryLineAxis::X, (0.0, 0.0, 0.0), (1.0, 9.0), &[0.0], 2.5, 0.0).expect("lines");
        let samples = &lines[0].samples;
        assert!((samples.first().expect("first").0 - 1.0).abs() < 1e-12);
        assert!((samples.last().expect("last").0 - 9.0).abs() < 1e-12);
        for window in samples.windows(2) {
            let gap = window[1].0 - window[0].0;
            assert!(gap > 0.0 && gap <= 2.5 + 1e-9, "{gap}");
        }
    }

    #[test]
    fn hostile_inputs_are_rejected() {
        let mesh = cylinder(RotaryLineAxis::X, (0.0, 0.0, 0.0), 4.0, 0.0, 2.0, 6);
        let cutter = ball(1.0);
        let cases = [
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, f64::NAN), (0.0, 1.0), &[0.0], 1.0, 0.0),
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (2.0, 1.0), &[0.0], 1.0, 0.0),
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0), &[0.0], 0.0, 0.0),
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0), &[f64::INFINITY], 1.0, 0.0),
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0), &[0.0], 1.0, f64::NAN),
            rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (f64::NAN, 1.0), &[0.0], 1.0, 0.0),
        ];
        for case in cases {
            assert!(matches!(case, Err(MeshError::InvalidInput(_))));
        }
        let too_many = vec![0.0; MAX_ANGLES + 1];
        let err = rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0), &too_many, 1.0, 0.0).expect_err("angles");
        assert!(err.to_string().contains("too many rotary angles"), "{err}");
        let err = rotary_lines(&mesh, cutter, RotaryLineAxis::X, (0.0, 0.0, 0.0), (0.0, 1.0e9), &[0.0], 0.001, 0.0).expect_err("samples");
        assert!(err.to_string().contains("too many rotary samples"), "{err}");
    }

    fn gaps_at_most(angles: &[f64], step: f64) {
        assert!((angles.first().copied().unwrap_or(0.0)).abs() < 1e-9);
        assert!((angles.last().copied().unwrap_or(0.0) - 360.0).abs() < 1e-6);
        for window in angles.windows(2) {
            let gap = window[1] - window[0];
            assert!(gap > 0.0 && gap <= step + 1e-6, "{gap} > {step}");
        }
    }

    fn independent_radius(mesh: &Mesh, cutter: Cutter, axis: RotaryLineAxis, origin: (f64, f64, f64), angle: f64, u: f64, floor: f64) -> f64 {
        let theta = angle.to_radians();
        let (cos, sin) = (theta.cos(), theta.sin());
        let origin_v = Vec3::new(origin.0, origin.1, origin.2);
        let triangles = mesh
            .triangles()
            .iter()
            .map(|tri| Triangle {
                a: rotate_point(axis, origin_v, tri.a, cos, sin),
                b: rotate_point(axis, origin_v, tri.b, cos, sin),
                c: rotate_point(axis, origin_v, tri.c, cos, sin),
            })
            .collect();
        let turned = Mesh::new(triangles).expect("turned");
        let dc = DropCutter::new(&turned, cutter).expect("dc");
        let (x, y) = sample_xy(axis, origin, u);
        match dc.tip_z(x, y) {
            Some(z) => z - origin.2,
            None => floor,
        }
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(32))]

        #[test]
        fn samples_stay_above_the_drop_cutter_and_angles_step_continuously(
            base in 4.0f64..14.0,
            amp in 0.0f64..1.5,
            freq in 1.0f64..3.0,
            stations in 3usize..6,
            segments in 8usize..14,
            step in 40.0f64..120.0,
            tool in 0.5f64..2.5,
            shift in -3.0f64..3.0,
            axis_bit in 0u8..2,
        ) {
            let axis = if axis_bit == 0 { RotaryLineAxis::X } else { RotaryLineAxis::Y };
            let origin = (shift, -shift * 0.5, shift * 0.25);
            let mut rings = Vec::with_capacity(stations);
            for index in 0..stations {
                let u = index as f64 * 6.0;
                rings.push((u, base));
            }
            let bump = |_: f64, theta: f64| amp * (freq * theta).sin();
            let mesh = revolved(axis, origin, &rings, segments, bump);
            let angles = full_turn(step);
            gaps_at_most(&angles, step);
            let u_range = (2.0, (stations as f64 - 1.0) * 6.0 - 2.0);
            let cutter = ball(tool);
            let floor = 0.0;
            let lines = rotary_lines(&mesh, cutter, axis, origin, u_range, &angles, 3.0, floor).expect("lines");
            prop_assert_eq!(lines.len(), angles.len());
            let returned: Vec<f64> = lines.iter().map(|line| line.angle_deg).collect();
            gaps_at_most(&returned, step);
            for line in &lines {
                for &(u, radius) in &line.samples {
                    let contact = independent_radius(&mesh, cutter, axis, origin, line.angle_deg, u, floor);
                    prop_assert!(radius + 1e-6 >= contact, "u {u} angle {} radius {radius} contact {contact}", line.angle_deg);
                    prop_assert!((radius - contact).abs() < 1e-4);
                }
            }
        }
    }
}
