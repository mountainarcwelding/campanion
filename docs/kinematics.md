# Kinematics

A kinematics file describes one or two rotary joints and the spindle direction at zero head angles. Angles are degrees. A rotation is right-handed about the joint axis. Lengths are millimetres.

The files in `crates/ir/kinematics/` are illustrative examples. They are not a description of any real machine.

Load a kinematics file with `Kinematics::from_json`. A program may store that model on `five_axis` with a `SolutionPolicy`: `nearest`, `first_positive` or `first_negative`. `SolutionPolicy::choice` maps the policy onto `Choice`. A program cannot store both a rotary setup and five-axis kinematics.

## Frames

- At all angles zero the part frame equals the machine frame.
- Rotaries are listed outer first: within the table joints, the one mounted on the machine base comes first; within the head joints, the one mounted on the Z slide comes first. Table and head joints may be interleaved in the list; only the order within each carrier matters.
- A table joint pivot is in machine coordinates at all angles zero. A second table joint's axis and pivot are given at zero angles and are carried by the first.
- A head joint pivot is relative to the gauge point (the point XYZ positions) at zero head angles. A second head joint is carried by the first.
- At zero head angles the tool tip is `gauge point - tool_length * spindle_axis`, and the tool axis is `spindle_axis`.
- Forward: `T_table = R_t1 * R_t2` (each `R` about its pivot and axis by its angle, using the zero-pose definitions). Head `H = R_h1 * R_h2` (about pivots relative to the gauge point). `tip_machine = xyz + H(-L * spindle_axis)` and `axis_machine = rot(H) * spindle_axis`. `tip_part = T_table^-1 * tip_machine` and `axis_part = rot(T_table)^-1 * axis_machine`.

`angles[i]` on a joint position belongs to `rotaries[i]`. With one rotary the other angle is 0.

## Inverse solutions

`Kinematics::inverse` returns every joint solution for a tool pose in the part frame. It solves the rotary angles that carry `spindle_axis` onto the requested tool axis, then the gauge position that puts the tip on the requested point. `tool_length` is the length `forward` uses, in millimetres, finite and non-negative.

`hint` is a pair of angles in `rotaries` order. A joint the tool axis does not determine takes its angle from `hint`. Each returned solution lies inside the joint travel limits. An unlimited joint is reported in `(-180, 180]`. Solutions are sorted by `(|angles[0]|, angles[0], angles[1])`.

A zero tool axis is rejected with `tool axis is zero`. No orientation solution is `tool axis (I, J, K) is not reachable`. An orientation that misses every travel limit is `tool axis (I, J, K) is outside the rotary limits`. `I`, `J` and `K` are the normalised target axis, six digits past the decimal.

## Solution choice

`Kinematics::choose` returns one solution from `inverse`.

| Choice | Hint | Selection |
| --- | --- | --- |
| `Nearest(prev)` | `prev` | Minimises the larger of the two absolute angle differences from `prev`. Equal distances keep inverse order. |
| `FirstPositive` | `[0, 0]` | First solution whose first angle is at least `-1e-9` degrees. Otherwise the first solution. |
| `FirstNegative` | `[0, 0]` | First solution whose first angle is at most `1e-9` degrees. Otherwise the first solution. |

The result is inside the joint travel limits, because `inverse` only returns solutions that are.

## Singularities

Some poses leave an angle free: every value of that joint produces the same tool axis. Inverse copies each free angle from `hint`.

That happens when the tilt is 0° or 180°. The spindle stays on a rotary axis, so the joint about that direction does not move the tool axis. On the examples the singular tilt is `A` for `table_table_ac`, `B` for `table_table_bc`, `A` for `head_head_ac`, and `B` for `head_table_bc` (table `C` is the free joint). It also happens when the two rotary axes are parallel: only the combined angle is fixed, and the inner joint keeps the hint. If the spindle is parallel to those axes as well, both angles keep the hint.

`Nearest(prev)` passes `prev` as the hint, so a free joint keeps its angle from `prev`. `FirstPositive` and `FirstNegative` hint `[0, 0]`, so a free joint is 0 unless selection picks another returned solution.

## Singular cone

`singular_cone_deg` is a toolpath setting. The default is 1. The value is in `(0, 10]`. It is omitted from JSON when it is the default.

When one rotary lies on the spindle axis, that rotary is the primary and its axis is the singular direction. On the examples the primary is `C`. The singular tilt is `A` or `B` at 0°, and the same cone sits around the opposite pole at 180°. A single rotary has no separate primary, so the cone does not change its solution. Parallel rotaries are left to the hint rule above.

While the requested tool axis lies inside that cone of either pole, `solve_path` keeps the primary at its previous angle. The secondary rotary takes the tilt that brings the tool axis as close as that locked primary allows. The tip matches the request. The achieved tool axis stays within `singular_cone_deg` of the requested axis. Outside the cone the solution is exact.

A step that enters or leaves the cone changes the primary by at most `90° + 2 × singular_cone_deg`. Inside the cone the primary does not move. The two exact branches just outside the cone are a half-turn apart on the primary; the nearer one, with the held tilt still inside the cone, stays inside that bound.

## Path solutions

`solve_path` takes a `FiveAxisSetup`, a list of tool poses, an optional starting joint position and `singular_cone_deg`. It returns one joint position per pose. The tool length is 0, so the gauge point is the tip. The setup's `SolutionPolicy` is not used: each pose takes the solution closest to the previous joints. `singular_cone_deg` is the toolpath setting of the same name.

The distance is the Euclidean length of the rotary-angle changes after each angle is unwound by whole turns onto the previous angle. An unlimited rotary may leave `(-180, 180]`. A joint does not jump by 360°.

The closest solution is the branch already in use. A solution that differs by more than 90° on a rotary is another branch. When the closest solution is outside a travel limit and another branch is inside, the error is `pose {index} flips the rotary branch`. When every solution is outside the limits, the error is `pose {index} leaves axis {letter}` for the first rotary of that closest solution that is outside. No orientation solution, and a zero tool axis, are `pose {index} is unreachable`.

`start` seeds the previous angles. Without it the previous angles are `[0, 0]`.

## Subdivision

`subdivide_path` takes the setup, the toolpath name, the poses, the joint solution, a chord tolerance in millimetres and an axis tolerance in degrees. Both tolerances are finite and positive. Poses and joints are the same length.

It inserts samples between neighbouring poses. Each inserted sample is linear in every joint, angles included, and the inserted pose is the forward pose of those joints. It repeats until, on every segment, the tip at the joint midpoint lies within the chord tolerance of the straight midpoint of the two poses, and the tool axis lies within the axis tolerance of the slerp of the forward axes of the two endpoint joints. The requested pose axis is not the one that is measured. Inside the singular cone that axis can differ from the solved joints by up to the cone angle, and the machine cuts the forward axis. A segment whose joints do not change is kept as it is, so a path of constant joints inserts nothing.

More than 10000 inserted poses is an error: `toolpath `{name}` inserts more than 10000 poses`.

## Continuous output

A post with `five_axis.continuous` and no `tcp` object writes a continuous toolpath without TCP. It calls `solve_path` from the current rotary angles, then `subdivide_path` at the post's `chord_tol` and `axis_tol_deg`. The IR tool table has no gauge length, so the tool length is 0 and the written XYZ is the gauge point, which is the tip. Each sample is one `G0` or `G1` with XYZ and the rotary words. Arcs and helices are linearised to `chord_tol` first. Feeds are inverse time (`G93`, `F = 1 / minutes` from the part-frame tip length, `G94` after the toolpath). `linuxcnc_5ax` and `haas_umc` both write G54 coordinates; their kinematics pivots are measured from G54. On `haas_umc` G255 is active for this output.

## TCP

`five_axis.continuous.tcp` writes the same continuous toolpath in TCP mode. `solve_path` still supplies the rotary angles, starting from `[0, 0]` so the branch matches the commanded zero. Branch continuity, the singular cone, the `90° + 2 × singular_cone_deg` bound, and reachability and travel-limit errors are unchanged. The written XYZ are not the inverse: they are the part-frame tool tip.

Subdivision uses the same `chord_tol` and `axis_tol_deg`, measured in the part frame. The assumption is that the control linearly interpolates the programmed tip and linearly interpolates the rotary joint words. It does not slerp the tool axis. The tool axis along the move is the forward kinematics of the interpolated angles at tool length 0. Because the tip interpolation is the chord, the tip chord error on a straight move is zero and `chord_tol` does not insert samples there. Arcs and helices are linearised to `chord_tol` in the part frame before that check. A sample is inserted when the forward axis at the joint midpoint differs from the slerp of the endpoint forward axes by more than `axis_tol_deg`. The inserted tip is the part-frame midpoint. More than 10000 inserted poses is the same error as `subdivide_path`. Inverse-time feed still uses the part-frame tip length. A rotary-only move uses the angle-change fallback.

`haas_umc_tcpc` cancels G254 with G255 when a frame is active, retracts, indexes the rotaries to zero, then commands `G234 H{tool}` (Haas mill G-code reference, G234 Tool Center Point Control, https://www.haascnc.com/service/codes-settings.type=gcode.machine=mill.value=G234.html). G234 cancels G43, so the H word is on that block. The rotaries are at 0 on the block before it. A rapid to the first tip (X and Y, then Z) follows. A rapid that changes a rotary is written as `G1` in `G93` at `rapid_feed` (5000 mm/min), because the tip is not maintained on a rapid rotary move. `G49` cancels G234, and `G43 H{tool}` restores the length offset. This output has not been proven on a machine.

`linuxcnc_5ax_tcp` leaves the on and off hooks empty. TCP is selected by the kinematics module, for example `xyzac-trt-kins`, `xyzbc-trt-kins`, or switchkins (https://linuxcnc.org/docs/html/motion/5-axis-kinematics.html, https://linuxcnc.org/docs/html/motion/switchkins.html). Switching codes are configuration-specific. The header carries `(TCP kinematics required)`. The machine must run TCP kinematics. This output has not been proven on a machine.

## JSON

Unknown fields are rejected. `spindle_axis` is optional and defaults to +Z `(0, 0, 1)`. Each axis is normalised on load. A zero axis is rejected.

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Non-empty name. Quoted in validation errors. |
| `rotaries` | yes | One or two joints. Outer joint first within each carrier. |
| `spindle_axis` | no | Tool axis at zero head angles. Default +Z. |
| `rotaries[].letter` | yes | `A`, `B` or `C`. A letter is used once. |
| `rotaries[].carrier` | yes | `table` or `head`. |
| `rotaries[].axis` | yes | Direction of the axis. Right-hand rotation. Not zero. |
| `rotaries[].pivot` | yes | A point on the axis line, in the frame described above. |
| `rotaries[].limits` | no | `{ "min_deg", "max_deg" }`, inclusive, `min_deg < max_deg`. Omitted means unlimited. |
| `model` | no | Link solids for the viewer. Omitted, the file is joints only. |

### Machine model

`model` is a sketch of the links as boxes and cylinders. It is not a CAD model of a machine. Coordinates are millimetres in each link's home frame, the frame at every joint zero. A vector is `{ "x", "y", "z" }`, the same shape as a pivot. Unknown keys are rejected. Without `model`, a kinematics file is unchanged.

```json
"model": {
  "links": [
    {
      "name": "disc",
      "carrier": "C",
      "solid": {
        "cylinder": {
          "base": { "x": 60.0, "y": 0.0, "z": 8.0 },
          "axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
          "radius": 36.0,
          "length": 18.0
        }
      }
    }
  ]
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `links` | yes | At most 64 solids. Names are unique and non-empty. An empty list is valid. |
| `links[].name` | yes | Quoted in validation and collision text. |
| `links[].carrier` | yes | `base`, `table`, `head`, or a rotary letter on this machine (`A`, `B` or `C`). |
| `links[].solid` | yes | `{ "box": { "min", "max" } }` or `{ "cylinder": { "base", "axis", "radius", "length" } }`. |

A box needs `max` strictly greater than `min` on every axis. A cylinder needs a positive radius, a positive length and a non-zero axis. The axis is normalised on load. `base` is one cap centre; `axis` points toward the other cap.

`base` does not move. A link whose carrier is rotary letter L moves with that joint and every rotary that carries it, in the same order `forward` uses. `table` is the full table chain and does not translate with XYZ: the linear axes move the head. `head` is the full head chain, then the gauge position. A head-side rotary letter translates with XYZ as well.

`campanion view --machine` (or `campanion-view --machine`) draws the solids as wireframe in the plan and isometric views. A program whose kinematics include `model` shows the sketch on open. M toggles it. The scrub uses the posted joint values from `solve_path` and `subdivide_path`, the same samples a continuous post writes. Colliding solids are drawn in red. The status line shows X Y Z, each rotary letter in degrees to three decimal places, and the first collision. The posted output has not been proven on a machine.

`examples/table_table_bc_model.kin.json` is `examples/table_table_bc.kin.json` plus a C-axis disc, a B-axis trunnion and a head box above the spindle. The numbers are an example, not a measured machine.

### Example

`crates/ir/kinematics/head_table_bc.json` is a mill-turn layout: C is a table joint about Z (a work spindle), and B is a head joint about Y whose pivot sits 150 mm above the gauge point. B travels from -110° to 110°. C is unlimited. The numbers are an example, not a measured machine.

```json
{
  "name": "head_table_bc",
  "spindle_axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
  "rotaries": [
    {
      "letter": "C",
      "carrier": "table",
      "axis": { "x": 0.0, "y": 0.0, "z": 1.0 },
      "pivot": { "x": 0.0, "y": 0.0, "z": 0.0 }
    },
    {
      "letter": "B",
      "carrier": "head",
      "axis": { "x": 0.0, "y": 1.0, "z": 0.0 },
      "pivot": { "x": 0.0, "y": 0.0, "z": 150.0 },
      "limits": { "min_deg": -110.0, "max_deg": 110.0 }
    }
  ]
}
```

At `B = 90°` and `C = 0°` the tool axis in the part frame is +X. That is a right-hand rotation of +Z about +Y.
