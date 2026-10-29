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

## Path solutions

`solve_path` takes a `FiveAxisSetup`, a list of tool poses and an optional starting joint position. It returns one joint position per pose. The tool length is 0, so the gauge point is the tip. The setup's `SolutionPolicy` is not used: each pose takes the solution closest to the previous joints.

The distance is the Euclidean length of the rotary-angle changes after each angle is unwound by whole turns onto the previous angle. An unlimited rotary may leave `(-180, 180]`. A joint does not jump by 360°.

The closest solution is the branch already in use. A solution that differs by more than 90° on a rotary is another branch. When the closest solution is outside a travel limit and another branch is inside, the error is `pose {index} flips the rotary branch`. When every solution is outside the limits, the error is `pose {index} leaves axis {letter}` for the first rotary of that closest solution that is outside. No orientation solution, and a zero tool axis, are `pose {index} is unreachable`.

`start` seeds the previous angles. Without it the previous angles are `[0, 0]`.

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
