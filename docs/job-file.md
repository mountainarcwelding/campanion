# Job file (`campanion-job/1`)

A job file names a source part, a tool table, and the layer operations to run on that part.
It does not contain geometry and it does not contain a toolpath. A later step imports the part
and builds paths from the operations recorded here.

This is not the profile document read by `campanion ir --job` and `campanion ngc --job`. That
document has no `format` tag. `campanion-job/1` rejects it.

Lathe work uses a different file, `campanion-lathe/1`, loaded by `campanion lathe`. See [Lathe jobs](#lathe-jobs).

The loader is `crates/cli/src/jobfile.rs`. `campanion run` loads it.

## Top level

JSON object. Unknown fields are rejected. Lengths in the file are in `units`. After `load`,
lengths are millimetres and feeds are millimetres per minute.

| Field | Required | Meaning |
| --- | --- | --- |
| `format` | yes | Exactly `campanion-job/1`. `campanion-job/2` is an unsupported major. Anything else is an unknown tag. |
| `source` | yes | Part file. See below. |
| `units` | yes | `mm` or `inch`. One inch is 25.4 mm. |
| `tolerance` | no | Linear, angular and chord tolerances. Omitted fields default to 1e-6 mm (linear), 1e-6 rad (angular) and 0.01 mm (chord). Defaults are not scaled when `units` is `inch`. |
| `tools` | yes | Array of tools. May be empty when `operations` is empty. |
| `operations` | yes | Array of operations, in cut order. May be empty. |
| `rotary` | no | Fourth-axis table for indexed 3+1, wrapping and rotary finishing. See below. |

`tolerance.linear` and `tolerance.chord` are lengths in `units`. `tolerance.angular` is radians either way.

`chord` is the largest allowed deviation when curves (SVG Beziers, and later splines and ellipses) are fitted with lines and arcs. Smaller values give more, shorter arcs; 0.01 mm suits most cutting.

## Source

```json
"source": { "path": "parts/plate.dxf", "hash": "cbf29ce484222325" }
```

The source format is chosen by extension: `.svg` is read as SVG, `.step`/`.stp` as STEP (see below), `.stl` as a triangle mesh (see below), anything else as DXF.
`path` is relative to the directory that contains the job file. Absolute paths, a path with no
file name (`.` or `..` alone), and control characters are rejected. `..` components are allowed.
`source_path` joins the job directory and folds `.` and `..` lexically. It does not look at the
filesystem, so a missing part still has a path.

`hash` is 16 hexadecimal digits. Uppercase is accepted and stored as lowercase.

### Hash

FNV-1a 64 of the **raw bytes** of the source part, not of the job file and not of a decoded or
canonical form. Whitespace in the part changes the hash.

The parameters are the FNV-1a 64-bit offset basis `14695981039346656037` (`0xcbf29ce484222325`)
and prime `1099511628211` (`0x100000001b3`). Start from the offset basis. For each byte, XOR it
into the hash, then multiply by the prime, wrapping at 64 bits. Write the result as 16 lowercase
hex digits.

| Bytes | Hash |
| --- | --- |
| empty | `cbf29ce484222325` |
| `a` | `af63dc4c8601ec8c` |
| `foobar` | `85944171f73967e8` |

This hash is for **change detection only**. It is not a cryptographic hash and it is not an
integrity or trust check. Collisions exist. A matching hash means the part bytes are very likely
the same as when the job was written, so existing toolpaths can be kept. A different hash means
the part changed and the toolpaths should be generated again.

`hash_bytes` computes the digest. `source_changed` is true when that digest is not the stored
hash. `with_updated_hash` returns a new in-memory job with the digest stored. It does not write
the job file. The caller copies `source.hash` back into the JSON if it should stick.

## Tools

A tool has `id` (integer, unique in the file) and exactly one of `diameter` or `kerf`. Both are
lengths in `units`, positive, and at most 1e6 mm after conversion. The same rule as the profile
job: a diameter is a rotating cutter, a kerf is a kerf-only process, and the path offset is half
of that value.

`shape` is optional and only meaningful on a diameter tool. `"flat"` is the default. `"ball"` is a
ball nose whose sphere radius is half the diameter. `{"bull": corner_radius}` is a bull nose;
`corner_radius` is a length in `units`, greater than 0 and strictly less than the tool radius.
A shape other than flat is rejected when a profile, pocket, drill, engrave, face, chamfer or wrap uses
that tool. Those operations keep the flat diameter offset and do not read `shape`. A surface, a
rough, a waterline or a rotary finish reads `shape`.

## Operations

Each operation selects layers of the source part. Layer names are case-sensitive, non-empty, and
unique within that operation. The same layer may appear on two operations.

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Short unique name. |
| `type` | yes | `profile`, `pocket`, `drill`, `engrave`, `face`, `chamfer`, `surface`, `rough`, `waterline`, `wrap` or `rotary_finish`. |
| `layers` | yes, except surface, rough, waterline and rotary finish | One or more layer names. A surface, a rough, a waterline or a rotary finish rejects `layers`. |
| `side` | profile and chamfer | Profile: `inside`, `outside` or `on`. Chamfer: `inside` or `outside`; `on` is rejected. Rejected on pocket, drill, engrave, face, surface, rough, waterline, wrap and rotary finish. |
| `tool_id` | yes | `id` of a tool in this file. |
| `depth` | yes, except chamfer, surface, rough, waterline and rotary finish | Positive distance downward, in `units`. Not an absolute Z. On a profile, pocket, drill, engrave or face this is below the stock surface. On a wrap it is below the cylinder surface. A chamfer rejects `depth`; the tip depth follows from the geometry. A surface, a rough, a waterline or a rotary finish rejects `depth`. |
| `step_down` | yes, except chamfer, surface and rotary finish | Positive maximum thickness of one pass, in `units`. The last pass of a 2D operation, including a wrap, stops at `depth` and is not deeper. A chamfer rejects `step_down`. A surface or a rotary finish rejects `step_down`. A rough or a waterline requires it: the spacing between Z levels. |
| `stepover` | pocket, face, surface, rough and rotary finish | Positive step between passes, in `units`. At most the tool diameter on a pocket, face, surface or rough. Rejected on a kerf tool. A face that omits it uses 0.75 times the tool diameter. A pocket that omits it leaves the choice to the path generator (the tool radius). A surface requires exactly one of `stepover` and `scallop_height`. A rough that omits it uses 0.4 times the tool diameter. A waterline rejects `stepover`. On a rotary finish it is the arc length between angle passes at the largest radius, and it is not capped at the tool diameter. When `angle_step_deg` is also set, the run uses `stepover`. |
| `scallop_height` | surface only | Positive length in `units`. Flat-floor cusp height used instead of `stepover`. See below. Rejected on every other kind. |
| `sample_step` | surface and rotary finish | Positive length in `units`. On a surface, the default is half of the stepover that is used (after a scallop height is resolved and capped). It is the maximum gap between the regular drop-cutter samples on a raster line, and between the samples on the link from one line to the next. On a rotary finish it is the spacing along the axis. Omitted there, the run uses `min(0.25, chord * 10)` millimetres. |
| `angle_deg` | face and surface | `0` (along +X) or `90` (along +Y). Default `0`. Stored as radians. Rejected on every other kind. |
| `margin` | face, surface, rough and waterline | Length in `units`, at least 0, at most 1e6 mm. Default 0 on a face or a surface, and the tool radius on a rough or a waterline. On a face, grows the faced rectangle past the geometry bounding box on every side. On a surface, a rough or a waterline, grows the mesh XY bounds the same way. The rough stock rectangle is that grown box. Rejected on every other kind. |
| `floor_z` | surface, rough and waterline | Absolute Z in `units`. Default is the mesh minimum Z. On a surface, the tip stays at this height where the cutter misses the mesh. On a rough or a waterline, this is the deepest level. |
| `top_z` | rough and waterline | Absolute Z in `units`. Default is the mesh maximum Z. The stock surface the depth plan starts from. The first level is `top_z - step_down`, and the last level is exactly `floor_z`. |
| `grid` | rough and waterline | Positive length in `units`. Spacing of the drop-cutter samples that decide where the tool centre may sit. On a rough, the default is the minimum of `stepover / 4` and 0.5 mm. On a waterline, the default is the minimum of `step_down` and 0.25 mm. Both caps are millimetre lengths, not scaled again when `units` is `inch`. |
| `min_slope_deg` | waterline only | Degrees from horizontal, from 0 to 90 inclusive. Not scaled by `units`. Default 0, which keeps every closed loop. A positive value keeps only the portions of each loop whose estimated slope is at least that angle. A portion that is not the whole loop is an open polyline. |
| `width` | chamfer | Positive length in `units`, at most 1e6 mm. Horizontal width of the bevel. Rejected on every other kind. |
| `v_angle_deg` | chamfer | Included angle of the V-bit, in degrees, greater than 0 and less than 180. Stored as radians. Not scaled by `units`. Rejected on every other kind. |
| `tip_diameter` | chamfer | Length in `units`, at least 0, default 0. Must be less than the tool diameter. Rejected on every other kind. |
| `tip_clearance` | chamfer | Length in `units`, at least 0, default 0. How far the tip sits below the bottom of the bevel. Rejected on every other kind. |
| `stock_to_leave` | profile, pocket, surface, rough and waterline | On a profile or pocket, an object with optional `wall` and `floor` lengths, in `units`. Each omitted field is 0. `floor` must be less than `depth`. On a surface, a rough or a waterline, a single length in `units`, at least 0. A surface or a waterline defaults to 0 and adds that length to every cutter-location Z. That is an axial offset approximation, not a true 3D rest-material offset along the surface normal. A rough defaults to 0.5 mm (a millimetre length, not scaled again when `units` is `inch`) and keeps the tool where the drop-cutter tip plus that stock is at or below the level. Rejected on a face. |
| `finish` | profile and pocket | Object with optional `step_down` (positive length in `units`; omitted, equal to `depth`) and optional `floor` (bool, default true, pocket only). Rejected on a face. |
| `entry` | no | How a pass is entered. Omitted, a plunge. `{"type":"plunge"}`, `{"type":"ramp","angle_deg"}`, or `{"type":"helix","radius","angle_deg"}`. `angle_deg` is from the XY plane, greater than 0 and at most 90, and is not scaled by `units`. A helix `radius` is a length in `units`. One turn drops `2π r tan(angle_deg)`. A rough uses the same object. A surface, a wrap or a rotary finish rejects `entry`. A wrap plunges. |
| `feeds` | yes | `cut` (required) and `plunge` (optional), in file-length units per minute. Omitted `plunge` means use `cut`. |
| `pierce_height` | no | Absolute Z to rapid to before the pierce delay. |
| `cut_height` | no | Absolute Z to feed to after the pierce, before the cut. |
| `timing` | no | Delays and the two process-on toggles. See below. Rejected on a wrap and on a rotary finish. |
| `index_deg` | no | Machine A or B angle in degrees. Not scaled by `units`. Requires a job-level `rotary`. Omitted on a rotary job, the operation runs at 0° and its poses carry rotary 0. Rejected on a wrap and on a rotary finish: the angle follows the drawing or the rotary passes. |
| `diameter` | wrap | Cylinder surface diameter, in `units`. Positive, at most 1e6 mm after conversion. This is not the tool diameter. Rejected on every other kind. |
| `u_offset` | wrap | Length in `units`, default 0, added to drawing X to make flat-frame `u`. May be negative. At most 1e6 mm in absolute value. Rejected on every other kind. |
| `v_offset` | wrap | Length in `units`, default 0, added to drawing Y to make flat-frame `v`. May be negative. At most 1e6 mm in absolute value. Rejected on every other kind. |
| `u_min` | rotary finish | Length in `units`. Low end of the pass along the rotary axis. Omitted, the mesh minimum on that axis. May be negative. |
| `u_max` | rotary finish | Length in `units`. High end of the pass along the rotary axis. Omitted, the mesh maximum on that axis. `u_min` greater than `u_max` is rejected. |
| `angle_step_deg` | rotary finish | Degrees between passes, greater than 0 and at most 360. Not scaled by `units`. See below. |
| `direction` | rotary finish | `zigzag` (the default) or `one_way`. |

`depth` and `step_down` must be a legal `campanion-path` depth plan: positive, at most 1e6 mm,
and at most 1024 passes. Five millimetres at a two millimetre step is three passes (2, 4, 5).
When `finish.step_down` is set, it must satisfy the same rule against `depth`.

Roughing leaves `wall` stock on the walls and `floor` stock above the floor. The finish pass
removes that remaining stock.

### Facing

`type` `face` clears a rectangle. The rectangle is the union of the bounding boxes of the
selected contours, expanded by `margin` on every side. Passes run at `angle_deg`. A kerf tool
is rejected on a face, with or without `stepover`. `side`, `stock_to_leave` and `finish` are
rejected too.

`examples/face.job.json` faces the outline layer of `examples/curves.svg` 0.5 mm deep in two
passes with a 20 mm tool and a 2 mm margin.

### Chamfer

`type` `chamfer` cuts a closed contour with a V-bit. `side` `outside` keeps the tool outside the
contour. `side` `inside` keeps it in a hole. `on` is rejected, and an open contour is an error
when the job runs. A kerf tool is rejected.

`width` is the horizontal width of the bevel. `v_angle_deg` is the included angle of the cone.
`tip_diameter` is the flat at the point, and `tip_clearance` is how far that point sits below
the bottom of the bevel. The tip depth, downward from the stock surface, is
`width / tan(a / 2) + tip_clearance`, where `a` is `v_angle_deg` in radians. After `load`,
`depth()` and `step_down()` both return that distance, so one pass reaches the tip. The file
must not set `depth` or `step_down`.

The tool `diameter` is the largest cutting diameter of the V-bit. `tip_diameter` must be smaller.

`examples/chamfer.job.json` chamfers `examples/curves.svg` with a 90° 6 mm V-bit: the `OUTER`
outline outside, 0.5 mm wide, and the `POCKET` hole inside, 0.3 mm wide. With no tip clearance
the tips are at −0.5 mm and −0.3 mm.

### Wrapping

`type` `wrap` lays the selected layer contours onto a cylinder about the job `rotary` axis. The source is a DXF or SVG. Geometry is the contours on `layers`, the same selection an engrave uses, and arcs are flattened at the job chord tolerance before they are wrapped. A kerf tool is allowed and the path stays on the drawing. A non-flat tool shape is rejected. An STL source is rejected.

`wrap` without `rotary` fails the load with `operation `<name>`: wrap needs a rotary setup`. `index_deg` is rejected. The rotary angle comes from the drawing.

`diameter` is the cylinder surface diameter, not the tool diameter.

`depth` is a positive distance below that surface. `step_down` is the maximum thickness of one pass, the same plan as a profile. A depth of 0.5 mm at a step of 0.25 mm is two passes.

Flat frame, in millimetres after `load`:

- `u` is drawing X plus `u_offset`: distance along the rotary axis from `rotary.center`. `u_offset` defaults to 0 and may be negative.
- `v` is drawing Y plus `v_offset`: arc length around the cylinder surface. `v_offset` defaults to 0 and may be negative.
- `w` is the radial offset from the surface. `0` is on the surface. A cut is into the material at `w = -level`.

The machine angle is `v / (π · diameter) · 360` degrees. It is not folded into one turn, so a long drawing keeps turning.

For each contour, and for each depth level from the first step down to `depth`:

1. Rapid at the safe radius to the start of the contour. The move is subdivided so each rotary step stays inside the chord tolerance.
2. Plunge to `w = -level`.
3. Follow the contour.
4. Rapid back to the safe radius.

Feeds are the tool's surface feeds. The distance is the flat-frame length `sqrt(Δu² + Δv² + Δw²)`, not the shorter chord between part-frame tips. `plunge` is the plunge feed. An omitted plunge uses `cut`.

The safe radius is the index safe height: the greater of `heights.clearance` and the swept radius plus that clearance. The swept radius is the larger of the cylinder radius (`diameter / 2`) and the stock swept radius of the other operations. A wrap's drawing is not part of that stock box, because its X and Y are `u` and `v`, not part coordinates. With the axis centre at the origin, a rapid at the safe radius has machine Z equal to that safe height.

A wrap followed or preceded by an indexed operation uses the same index change as any other angle change: retract above the swept radius in the angle being left, then rotate while holding that machine tip. The program still starts at 0°. See below.

`side`, `entry`, `stepover`, `timing`, mesh keys and chamfer keys are rejected.

`examples/wrap.job.json` wraps `examples/wrap.svg` onto a 40 mm cylinder about A through the origin, 0.5 mm deep in two steps of 0.25 mm. Drawing X is shifted by `u_offset` 6 mm. The strokes stay inside drawing Y 0..120 mm, and the first stroke starts at Y 0 so the wrap begins at angle 0.

### Surface

`type` `surface` is a 3-axis parallel finish of an STL mesh. The source path must end in `.stl`
(any case). An STL source allows `surface`, `rough`, `waterline` and `rotary_finish` only, and `surface` is rejected on a
DXF, SVG or STEP source. There are no layers. A kerf tool is rejected.

The raster covers the mesh XY bounds, grown by `margin` on every side. Passes run at `angle_deg`
and alternate direction. A surface requires exactly one of `stepover` and `scallop_height`.
Setting both, or neither, is an error. `stepover` is the maximum gap between passes. The lines
are spaced evenly from one side of the area to the other, so the pitch is at most that stepover.
`sample_step` is the gap between regular drop-cutter samples; the links between passes use the
same spacing. When `sample_step` is omitted it is half of the stepover that is used. The chord
tolerance of the job is the largest tip-Z error allowed when a raster chord replaces the true
drop-cutter height. `floor_z` is the tip height where the cutter misses the mesh.

`scallop_height` is the cusp left between passes on a flat floor. It is a length in `units`,
greater than 0. The job turns it into an XY stepover when it runs:

- ball of radius `R`: `2 * sqrt(2*R*h - h*h)`
- bull nose of radius `R` and corner radius `c`: `2*(R - c) + 2*sqrt(2*c*h - h*h)`

`h` must be less than the corner radius (`R` on a ball, `c` on a bull nose). A flat tool has no
corner, so the run fails with `scallop height needs a ball or bull-nose tool`. The computed
stepover is capped at the tool diameter. A value past that limit is reduced to the diameter, and
no warning is printed. An explicit `stepover` larger than the diameter is still an error.

Raster spacing is measured in XY. On a slope the real cusp is larger than `scallop_height` for
that same stepover. `scallop_height` is the flat-floor value.

`stock_to_leave` raises every cutter-location Z by that many millimetres after the drop-cutter
query. Clearance and retract stay at the job heights. Put those heights above the raised surface:
a rapid is still at clearance, and the retract feeds to the job retract height.

The toolpath rapids to clearance above the first point, turns the process on, plunges to the
first cutter location, cuts each polyline, cuts the drop-cutter samples that join one line to
the next, retracts to the retract height, turns the process off, and rapids back to clearance.
A chord that would pass more than the chord tolerance below the drop-cutter tip is not cut.
The tool retracts, rapids at clearance, and plunges onto the next point.

`examples/dome.job.json` finishes `examples/dome.stl` with a 6 mm ball nose, a 1 mm stepover and
a 0.01 mm chord tolerance. The mesh is a 40 mm square plate at Z = 0 with a dome of radius 15 mm
at the centre, rising to Z = 10 mm.

### Roughing

`type` `rough` is Z-level roughing of an STL mesh. The source path must end in `.stl` (any case).
An STL source allows `surface`, `rough`, `waterline` and `rotary_finish` only. `rough` is rejected on a DXF, SVG or STEP source.
There are no layers. A kerf tool is rejected. The tool `shape` is the flat, ball or bull nose a
surface uses.

The stock rectangle is the mesh XY bounds, grown by `margin` on every side. Omitted, `margin` is
the tool radius. `step_down` is required. Levels start one step below `top_z` and continue by
`step_down`, and the last level is exactly `floor_z`. `top_z` defaults to the mesh maximum Z.
`floor_z` defaults to the mesh minimum Z. `stepover` is the spacing between inset rings. Omitted,
it is 0.4 times the tool diameter, and it must be greater than 0 and at most the diameter.
`grid` is the spacing of the samples that find where the tool centre may sit. Omitted, it is the
minimum of `stepover / 4` and 0.5 mm.

`stock_to_leave` is one length. The rings stay where the drop-cutter tip plus that stock is at or
below the level. It is not a separate wall and floor pair. A level where that allowance leaves
no place for the tool has no ring and adds no moves. On the dome, 0.3 mm of stock keeps the flat
tool off the plate, so the floor level is empty and the deepest cut is the 2.5 mm level.

`entry` is optional. Omitted, or `"type": "plunge"`, the tool feeds straight down at the ring
start. `"type": "ramp"` follows the ring while it descends. `angle_deg` is the angle from the XY
plane. `"type": "helix"` is a counter-clockwise circle about the ring's bounding-box centre.
`radius` is the circle radius.

For each ring at level `z` the entry descends from the previous level's Z, or from `top_z` on the
first level, down to `z`. The tool rapids at clearance to the entry start and rapids down to the
retract height. A plunge then feeds to the level. A ramp or a helix rapids straight down to the
start height on that same point, then feeds the entry. The ring is cut, and the tool feeds back
to the retract height. XY rapids stay at clearance. The process turns on once before the first
entry and off after the last retract, then the tool rapids to clearance. Clearance and retract
stay at the job heights. Put those heights above the stock.

A ramp follows the ring, which is where the tool is allowed at that level. A helix is used only
when samples of its circle, every 0.25 mm, lie inside the ring and the drop-cutter tip plus
`stock_to_leave` is at or below the level. If a sample fails, the operation ramps at the helix
angle and writes one stderr line, `helix does not fit; ramping`.

Levels are cut from the top down, and the rings of a level are cut in order.

`examples/dome_rough.job.json` roughs `examples/dome.stl` with a 6 mm flat end mill, a 2.5 mm
step-down, a 2.4 mm stepover and 0.3 mm of stock to leave. Clearance is 15 mm and retract is 12 mm,
above the dome.

### Waterline

`type` `waterline` finishes the steep walls of an STL mesh. The source path must end in `.stl`
(any case). An STL source allows `surface`, `rough`, `waterline` and `rotary_finish` only. `waterline` is rejected
on a DXF, SVG or STEP source. There are no layers. A kerf tool is rejected. The tool `shape` is
the flat, ball or bull nose a surface uses.

The area is the mesh XY bounds, grown by `margin` on every side. Omitted, `margin` is the tool
radius. `step_down` is required. Levels start one step below `top_z` and continue by `step_down`,
and the last level is exactly `floor_z`. `top_z` defaults to the mesh maximum Z. `floor_z`
defaults to the mesh minimum Z. At each level the contact curve is the set of tool centres whose
drop-cutter tip equals that Z. `grid` is the spacing of the samples that find that curve.
Omitted, it is the minimum of `step_down` and 0.25 mm.

`min_slope_deg` drops the shallow portions of each curve. The slope is estimated from the
drop-cutter tip by central differences one grid step either side of the point:
`atan` of the size of that gradient, in degrees from horizontal. A point is kept when that
estimate is at least `min_slope_deg`. Zero, the default, keeps every loop closed, with the first
point repeated at the end. A positive angle splits a loop where the filter cuts it. A portion
that is not the whole loop is an open polyline and does not repeat its first point. A loop whose
every point passes stays closed.

`stock_to_leave` is one length. It is added to the Z of every plunge and cut. It is not a
separate wall and floor pair, and it is not added inside the contact query. Omitted, it is 0.
Clearance and retract stay at the job heights. Put those heights above the raised cuts.

Entry is a plunge. `entry` is rejected. Levels are cut from the top down, and the polylines of a
level are cut in order. For each polyline the tool rapids at clearance to the first point, rapids
down to the retract height, plunges to that point's Z (plus axial stock), cuts along the
polyline, and feeds back to the retract height. The process turns on once before the first plunge
and off after the last retract, then the tool rapids to clearance.

`examples/dome_waterline.job.json` finishes the steep part of `examples/dome.stl` with a 6 mm ball
nose, a 2.5 mm step-down, a 1 mm grid and a minimum slope of 30°. Clearance is 15 mm and retract is 12 mm.

### Rotary finishing

`type` `rotary_finish` is a continuous 4th-axis finish of an STL mesh. The source path must end in `.stl` (any case). It needs a job-level `rotary`. The tool stays on machine +Z and the part turns about that axis. There are no layers. A kerf tool is rejected. `index_deg` is rejected.

Passes run along the axis. The angle then advances. Angles start at 0° and the last pass is at 360°. `u_min` and `u_max` limit that axial window, in `units`. Omitted, the window is the mesh extent on the rotary axis.

`angle_step_deg` is the angle between passes, in degrees. It is not scaled by `units`. `stepover` is the same spacing as an arc length at the largest radius of the part. When both are set, the run uses `stepover`. When neither is set, the stepover is the flat-floor scallop for the job chord tolerance, capped at the tool diameter. That default needs a ball or bull nose.

`sample_step` is the gap between drop-cutter samples along the axis. Omitted, the run uses `min(0.25, chord * 10)` millimetres. `direction` is `zigzag` or `one_way`. Zigzag is the default. It alternates the axial direction and, at the shared end of two passes, rotates in contact when the tip stays above the part at both angles and at the probed angles between them. Otherwise the tool retracts to the safe radius, rotates, and plunges. One-way always retracts, and every pass starts at the low end.

The safe radius is at least the swept radius plus `heights.clearance`, and at least the highest cut plus that clearance. `campanion run` gouge-checks the path in the rotated frame: a constant-angle cut is sampled on the mesh turned by that angle, and a cut that changes angle is sampled between its ends.

`examples/rotary.job.json` finishes `examples/rotary.stl` with a 3 mm ball, zigzag, a 30° angle step and a 0.25 mm sample step, about axis A through the origin. The mesh is a vase revolved about X, 60 mm long and about 30 mm in diameter, with closed ends. `crates/mesh/tests/rotary_vase.rs` builds that STL.

Pocket strategy, leads, tabs and drill-cycle fields are not part of this version.

### Timing

Delays are seconds. They are not scaled with `units`. Heights are.

```json
"timing": {
  "pierce_delay": 0.25,
  "post_delay": 0.1,
  "pierce_process_on_during_delay": true,
  "post_process_on_during_delay": false
}
```

Defaults match `campanion-path` `ProcessTiming`, including when `timing` is omitted and when a
single field is omitted:

- `pierce_delay` defaults to 0. The process stays on during that delay (`pierce_process_on_during_delay` defaults to true).
- `post_delay` defaults to 0. The process is already off during that delay (`post_process_on_during_delay` defaults to false).

A delay of zero adds no dwell when a path is later timed. The toggles are the two
"process on during the delay" switches: one at pierce, one at the post (end) delay.

The loader stores delays, toggles and heights on the same process-parameter object the profile
job builds, so both files share one mapping.

## Indexed 3+1

`rotary` turns the part about one table axis while the tool stays on machine +Z. The IR stays in the part frame: every pose is mapped back with the inverse rotation and tagged with the index angle.

```json
"rotary": {
  "axis": "a",
  "center": [0, 0, 0],
  "direction": "positive",
  "limits": { "min_deg": -120, "max_deg": 120 }
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `axis` | yes | `"a"` rotates about part +X. `"b"` rotates about part +Y. |
| `center` | yes | `[x, y, z]`, a point on the axis line. A length in `units`, at most 1e6 mm after conversion. |
| `direction` | yes | `"positive"` uses the machine angle as a right-hand rotation about the axis. `"negative"` negates the angle first. |
| `limits` | no | `{min_deg, max_deg}` in degrees. Not scaled. `min_deg` is strictly less than `max_deg`. A stored angle must lie inside the inclusive range. |

`index_deg` is the angle θ for that operation. It is a finite number of degrees and is not scaled. `index_deg` without `rotary` fails the load with `operation `<name>`: index_deg needs a rotary setup`. An operation that omits `index_deg` on a rotary job runs at θ = 0, and its poses carry rotary 0. A job with no `rotary` stays 3-axis: poses have no rotary angle, and existing files produce the same toolpath bytes.

With index angle θ the machine turns the part by θ.

- A profile, pocket, drill, engrave, face or chamfer reads its DXF or SVG geometry as drawn in the indexed view (machine XY, Z up). That geometry is not rotated.
- A surface, rough or waterline, and the gouge check for those operations, runs on the mesh turned by θ about the rotary axis. The turn is the same rotation `rotate_point` applies. On a surface, a chord that would gouge that turned mesh retracts and plunges instead of cutting through the silhouette.
- Each planned pose is mapped back into the part frame and tagged with θ. The tool axis on that pose is machine +Z turned by the same inverse rotation.

When two consecutive operations have different index angles, and before the first operation when its index is not 0° (the program starts at 0°), the run inserts an index change:

1. A rapid retract to a safe height Zs in the current indexed view.
2. A rapid that keeps the tip at that machine position and changes the rotary angle. The pose is mapped with the new angle.
3. The next operation continues from there.

The retract is part of the operation being left, at the old angle. The rotary rapid is the first move of the operation being entered, at the new angle. When the program starts away from 0° there is no previous operation, so the retract is its own toolpath at 0° and the rotary rapid is prepended to the first operation.

Zs is the greater of the job safe height and the swept radius plus the rapid clearance. Both of those heights are `heights.clearance` (default 5 mm). The swept radius is the largest distance from the rotary axis line to the stock. A 2D job uses the corners of a box around the selected contours: XY from those bounds, Z from `-depth` up to the stock surface at 0. A wrap is not part of that box. Its cylinder radius is included in the swept radius on its own, because the drawing's X and Y are cylinder coordinates. An STL job has no stock box, so the radius is measured to the mesh vertices.

`examples/indexed.job.json` finishes `examples/dome.stl` with a 6 mm ball at index 0° and 90° about A through the origin.

A post with no `rotary` field rejects a program that contains any rotary angle, with `post `<id>` does not support a rotary axis`. A setup whose poses carry no angle still posts. LinuxCNC, Fanuc and Haas write axis A: machine `X` `Y` `Z` from `part_to_machine`, and inverse-time feed (`G93`, back to `G94` when the angle holds) when a feed changes the angle. Mach3 writes axis A and keeps the IR feed number (degrees per minute). Grbl, plasma and the Grbl laser post have no rotary axis. The length used for inverse time is the straight-line distance between the part-frame tips; see the post crate README.

## Limits

Hostile numbers and sizes fail the load instead of being passed on:

- JSON file at most 32 MiB, UTF-8.
- 256 tools, 1024 operations, 64 layers on one operation.
- Names at most 256 bytes. Source path at most 4096 bytes.
- Tool size, depth, step-down, chamfer width, tip diameter, tip clearance, bull corner radius, margin, sample step, grid, floor Z, top Z, wrap diameter and offsets, surface, rough and waterline stock, heights and tolerances at most 1e6 mm (angular tolerance at most 1e6 rad). An STL part read by `run` is at most 32 MiB, the same cap as a text source.
- Feeds at most 1e9 mm/min. Delays at most 1e6 s.
- At most 1024 depth passes.

## API

| Function | Behaviour |
| --- | --- |
| `load(path)` | `Result<JobFile, JobFileError>`. |
| `hash_bytes(bytes)` | 16-digit lowercase FNV-1a 64. |
| `source_changed(job, bytes)` | `true` when the digest differs from the stored hash. |
| `with_updated_hash(job, bytes)` | New job whose stored hash is the digest of `bytes`. |
| `source_path(job)` | Job directory joined with `source.path`. |

`JobFileError` is `Io`, `TooLarge`, `Json`, `Invalid`, `UnknownFormat` or `UnsupportedVersion`.

## Example

The hash below is FNV-1a 64 of zero bytes. Replace it with the hash of `parts/plate.dxf`.

```json
{
  "format": "campanion-job/1",
  "source": { "path": "parts/plate.dxf", "hash": "cbf29ce484222325" },
  "units": "mm",
  "tolerance": { "linear": 0.000001, "angular": 0.000001, "chord": 0.000001 },
  "tools": [
    { "id": 1, "diameter": 6.0 },
    { "id": 2, "kerf": 1.2 }
  ],
  "operations": [
    {
      "name": "outer",
      "type": "profile",
      "layers": ["OUTLINE"],
      "side": "outside",
      "tool_id": 1,
      "depth": 6.0,
      "step_down": 3.0,
      "feeds": { "cut": 1200.0, "plunge": 300.0 },
      "pierce_height": 2.0,
      "cut_height": 0.0,
      "timing": {
        "pierce_delay": 0.25,
        "post_delay": 0.1,
        "pierce_process_on_during_delay": true,
        "post_process_on_during_delay": false
      }
    }
  ]
}
```

## STEP sources

```json
"source": { "path": "plate.step", "hash": "...", "face": [50, 5, 5] }
```

A STEP part is cut from one planar face. `face` is a point on that face, in millimetres in the
part's model coordinates (never scaled by `units`). `campanion run` hands the part to SolidCraft's
command-line tool, which exports the face's outline and holes as DXF, and imports that DXF.

- The tool is found through the `CAMPANION_SOLIDCRAFT_CLI` environment variable (path to the
  `solidcraft-cli` executable) or, failing that, `solidcraft-cli` on `PATH`. Without it, a STEP job
  stops with an error that says so; DXF and SVG jobs don't need it.
- The face DXF has a single layer, so operations select contours by nesting instead: `OUTER` is
  the face outline (and any islands inside holes), `INNER` is the holes.
- The DXF is written to a private temporary directory that is removed afterwards.

`examples/step.job.json` cuts `examples/plate.step` (generated with `solidcraft-cli`).

## Lathe jobs

`campanion lathe JOB -o OUT [--ir OUT.json] [--post ID|FILE.json]` loads a `campanion-lathe/1`
file. It is not a `campanion-job/1` file: the fields below are the whole schema, and unknown
fields are rejected with a message that names the key. Every number must be finite. Lengths are
millimetres. Feeds are millimetres per minute. Constant surface speed is metres per minute.
Direct spindle speed is revolutions per minute.

`examples/shaft.lathe.json` faces, roughs and finishes the half-profile in `examples/shaft.dxf`.

```json
{
  "format": "campanion-lathe/1",
  "source": { "path": "shaft.dxf", "layer": "PROFILE" },
  "units": "mm",
  "stock": { "kind": "bar", "diameter": 50, "length": 60, "front_allowance": 1.0 },
  "spindle": { "max_rpm": 3000 },
  "tools": [
    { "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.8, "orientation_deg": 135, "hand": "right", "tip": "centre" }
  ],
  "operations": [
    { "name": "face", "type": "face", "tool_id": 1, "step_z": 0.5, "feed": 150, "speed": { "css": 200 } },
    { "name": "rough", "type": "rough", "tool_id": 1, "side": "outside", "depth_of_cut": 2.0, "leave_x": 0.3, "leave_z": 0.1, "feed": 250, "speed": { "css": 180 } },
    { "name": "finish", "type": "finish", "tool_id": 1, "side": "outside", "feed": 120, "speed": { "rpm": 1500 } }
  ],
  "tolerance": { "linear": 0.001, "chord": 0.01 },
  "clearance": 2.0
}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `format` | yes | Exactly `campanion-lathe/1`. `campanion-lathe/2` is an unsupported major. Anything else is an unknown tag. |
| `source` | yes | Half-profile. `path` is relative to the job file, the same rules as a mill job (no absolute path, 1 to 4096 bytes). |
| `units` | yes | Only `mm`. |
| `stock` | yes | Bar or tube, plus `front_allowance`. |
| `spindle` | yes | `max_rpm`, positive and at most 100000. That value is the constant-surface-speed cap. |
| `tools` | yes | Inserts, at most 256. May be empty when `operations` is empty. |
| `operations` | yes | Face, rough and finish, in order, at most 1024. |
| `tolerance` | no | `linear` (default 1e-6, must be positive) is the profile and finish tolerance. `chord` (default 0.01) is the post read-back tolerance. `angular` is accepted, must be `>= 0`, and is not used. |
| `clearance` | yes | Approach gap, millimetres, `>= 0`. Roughing retracts by the same distance. |

`source` chooses the reader by extension:

- `.dxf` — [`TurnProfile::from_dxf`](../crates/turn/src/profile.rs). Optional `layer`. `axis` and `step` are rejected.
- `.svg` — `from_svg` when `layer` is omitted. A layer uses the SVG importer and `TurnProfile::from_drawing`. The SVG fit uses `linear` as its chord; the job `chord` is the post read-back.
- `.stl` — `from_mesh`. Requires `axis` (`x`, `y` or `z`) and a positive `step`. A `layer` is rejected.

Drawing X is Z and drawing Y is the radius. Z = 0 is the finished front face and the part extends toward −Z. A closed drawing is opened by dropping the spindle-axis run.

`stock.kind` is `bar` (`diameter`, `length`, `front_allowance`) or `tube` (`outer_diameter`, `inner_diameter`, `length`, `front_allowance`). Diameters and `length` are positive. `front_allowance` is `>= 0`. The IR stock still occupies Z in `[-length, 0]`; the allowance is only the facing depth, from `+front_allowance` down to 0. A tube's inner diameter must be below its outer diameter.

Each tool is one insert on head `turret1` (12 stations, no live tooling). `id` is the tool-table id. `station` is 1 to 12, and ids and stations are unique. `shape` is `c80`, `d55`, `v35`, `w80`, `t60`, `s90`, `r`, `groove` or `thread`. A groove needs a positive `width`; other shapes may set it. `nose_radius` is positive. `orientation_deg` is the bisector from +Z toward +X, in −360 to 360. `hand` is `left`, `right` or `neutral`. `tip` is `centre` or `imaginary`. The tool table stores no milling diameter.

`tip: "imaginary"` programs the theoretical sharp corner. The IR insert offset is `(dx, dz)` in `(X, Z)`, from [`Insert::tip_offset`](../crates/ir/src/turning.rs): `(-r·sign(sin o), -r·sign(cos o))`, with `sign(0) = 0`. The turning path stores that pair as `dz`, `dx`. At 45° both components are `−nose_radius`. At 135° `dx` is `−nose_radius` and `dz` is `+nose_radius`. `tip: "centre"` programs the nose centre.

`speed` is either `{ "css": metres_per_minute }` or `{ "rpm": revolutions_per_minute }`, not both. CSS uses the spindle `max_rpm` as its cap. The speed is stored on the toolpath binding (spindle `main`, head `turret1`, channel 0), not on the process-on parameters.

Operations, in cut order. Names are unique and non-empty.

- `face` — `step_z` and `feed`. Passes run from the stock front (`+front_allowance`) to Z = 0. The radial feed starts at the stock outer radius plus `clearance` and ends at `−nose_radius` (one nose past the spindle axis). A centre tip is given a target radius of 0 because facing already shifts that end by `−nose_radius`. An imaginary tip is given `−nose_radius` directly.
- `rough` — `side` (`outside` or `inside`), `depth_of_cut`, `leave_x`, `leave_z`, `feed`. The stock outer and inner radii are the bar or tube radii (a bar's inner radius is 0). Retract equals `clearance`. Longitudinal roughing stops at the first boundary it cannot pass.
- `finish` — `side` and `feed`. `leave_x` and `leave_z` default to 0. The offset tolerance is `tolerance.linear`.

Each operation is one toolpath. The first time a tool id appears, the path starts with a tool change. Then the spindle turns on, the moves run (the first of those is a rapid to the path start), and the spindle turns off at the end of that path. Nose compensation is already in the coordinates (`compensation: computed`).

Without `--post`, `-o` is the RS274/NGC test sink. ZX arcs are `G18`. That sink is not a post for a machine. `--post linuxcnc_lathe` writes diameter mode (`G7`), the ZX plane (`G18`) and constant surface speed (`G96`); direct rpm is `G97`. X in the IR stays a radius. Posted text is read back at `tolerance.chord`. A mismatch exits 1 and writes nothing. Verify errors are printed and the status is 1; nothing is written in that case either. `--ir` writes the IR JSON after a successful `-o`, and the two paths must differ. The IR stores the finished section on `turning.section`, sampled from the loaded profile at `tolerance.chord`.
