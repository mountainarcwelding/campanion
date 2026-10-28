# Job file (`campanion-job/1`)

A job file names a source part, a tool table, and the layer operations to run on that part.
It does not contain geometry and it does not contain a toolpath. A later step imports the part
and builds paths from the operations recorded here.

This is not the profile document read by `campanion ir --job` and `campanion ngc --job`. That
document has no `format` tag. `campanion-job/1` rejects it. See [Profile document](#profile-document).

Lathe work uses a different file, `campanion-lathe/1`, loaded by `campanion lathe`. See [Lathe jobs](#lathe-jobs).

The loader is `crates/cli/src/jobfile.rs`. `campanion run` loads it.

## Profile document

The file read by `campanion ir --job` and `campanion ngc --job` is a single 2D profile, not a `campanion-job/1` job. Stock top is Z 0. Optional top-level `heights` is the same object a job file uses: `{"clearance": …, "retract": …}`. An omitted field keeps the job default, 5 mm clearance and 2 mm retract. Both values must be finite, `retract` must be above the stock top, and `clearance` must be at least `retract`.

The program rapids to the pierce XY at the clearance height, rapids down to the retract height, and feeds down to depth. After the process turns off it rapids back to the clearance height.

`profile.tabs` takes `width` and either `count` or `spacing`, with an optional `height`. `width` is the width of the bridge left on the part, measured along the part edge. On a straight edge the tool-centre gap is `width` plus the tool diameter (or the kerf). On an arc or a corner the gap is long enough that the uncut material along the part edge is at least `width`. For `side` `on`, the bridge is `width` centred on the line and the gap is `width` plus the tool diameter as well. `spacing` is the centre-to-centre distance along the compensated path and must be greater than `width` plus the tool diameter. A positive height is a bridge of that thickness: the tool feeds up, follows the compensated contour at `z + height` (lines, arcs and corners), and feeds back down. That crossing is never a rapid and never a chord that leaves the contour. When `height` is omitted or 0, the tab is the full depth of the material: feed up to the retract height, rapid across at that height, feed back down. An XY rapid below the retract height is an error. That height is above the stock top, so the rapid is strictly above Z 0.

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

`speed` is optional. It is a spindle speed in rpm: positive, finite, and at most 100000. It is not
scaled by `units`. Omitted, the tool has no speed. An operation may set `speed` too; see below.

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
| `speed` | no | Spindle speed in rpm: positive, finite, and at most 100000. Not scaled by `units`. When set, it overrides `tools[].speed` for this operation. Omitted, the tool's speed is used. See [Spindle speed](#spindle-speed) and [Tool changes](#tool-changes). |
| `depth` | yes, except chamfer, surface, rough, waterline and rotary finish | Positive distance downward, in `units`. Not an absolute Z. On a profile, pocket, drill, engrave or face this is below the stock surface. On a wrap it is below the cylinder surface. A chamfer rejects `depth`; the tip depth follows from the geometry. A surface, a rough, a waterline or a rotary finish rejects `depth`. |
| `step_down` | yes, except chamfer, surface and rotary finish | Positive maximum thickness of one pass, in `units`. The last pass of a 2D operation, including a wrap, stops at `depth` and is not deeper. A chamfer rejects `step_down`. A surface or a rotary finish rejects `step_down`. A rough or a waterline requires it: the spacing between Z levels. On a drill it is the peck depth: when it is less than `depth` the drill pecks, and otherwise the drill feeds to depth in one move. |
| `dwell` | drill only | Seconds to pause at the bottom of a drill, from 0 to 3600 inclusive. Not scaled by `units`. Rejected on every other kind. Omitted, the drill does not pause. |
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
| `entry` | no | How a pass is entered. Omitted, a plunge. `{"type":"plunge"}`, `{"type":"ramp","angle_deg"}`, or `{"type":"helix","radius","angle_deg"}`. `angle_deg` is from the XY plane, greater than 0 and at most 90, and is not scaled by `units`. A helix `radius` is a length in `units`. One turn drops `2π r tan(angle_deg)`. A rough uses the same object. A surface, a wrap or a rotary finish rejects `entry`. A wrap plunges. The tool rapids to the retract height, then feeds the entry. On a profile, engrave or pocket wall, a ramp or helix is sized from that height down to the first pass when the retract height is above the stock, and from the previous level on later passes, so every posted ramp or helix descent is at most `angle_deg`. A ramp follows the compensated contour, including around corners. A profile ramp that would cross a tab below the bridge top lifts across the bridge instead of descending through it. A pocket area pass uses this entry at each depth, sized from the retract height for the whole drop to that level. The ramp follows the first ring for one circuit and the operation fails, instead of plunging, when that circuit is shorter than the ramp. A helix is a circle about that ring's bounding-box centre; the operation fails when the circle or the feed onto the ring leaves the pocket. Later rings of the same depth are cut at that level. A feed joins two rings when the segment stays in the region the tool centre may enter; otherwise the tool retracts and enters the next ring with the same entry. Omitted, the area pass plunges, and a link that cannot stay inside retracts and plunges. The wall finish, when `finish` is set, uses this entry. |
| `tabs` | profile | Object with `width` and either `count` or `spacing`, plus an optional `height`. `width` is the bridge left on the part. Same fields and errors as a profile document. Lengths are in `units` (`count` is not scaled). `width` must be > 0. Exactly one of `count` and `spacing`. `height`, when set, must be >= 0 and less than `depth`; omitted or 0 is a full-depth tab. See [Profile tabs](#profile-tabs). Rejected on every other kind. |
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

### Tool changes

`campanion run` writes a tool-change event (`Event::ToolChange`) at the start of the first
toolpath and at the start of each later toolpath whose tool id differs from the previous
toolpath. Consecutive toolpaths that use the same tool get one change. Every mill operation
kind does this: profile, pocket, face, drill, chamfer, engrave, surface, rough, waterline,
wrap, rotary finish, and indexed or tilted work. A post that has a `tool_change` template
emits it (`T{tool} M6` on mach3, linuxcnc and haas; haas also writes `G43 H{tool}`). Lathe
jobs already write a tool change when the tool id changes; that path is unchanged.

### Spindle speed

`tools[].speed` and `operations[].speed` are rpm. The operation value wins when both are set.
`run` copies the resolved speed onto each process-on event so a post writes `M3 S{speed}`.
A spindle stays on through that tool's depth passes and later operations — one process-on after the tool change, the new speed without a stop when rpm changes, and one process-off after the retract above the stock at the next tool change or the end of the program. Mill posts default to `process_cycle` `hold` and write that single start and stop. Plasma and the Grbl laser post set `process_cycle` to `per_cut`: every process, including spindle, turns off before a rapid and on again before the next feed, so a torch or beam does not stay on through G0.

A job that omits both still parses. When `run --post` uses a mill post whose spindle-on
template substitutes `{speed}` (mach3, linuxcnc, haas, and the other mill posts) and a
spindle operation (a diameter tool) has no speed, `run` fails with
`operation <name>: no spindle speed; set tools[].speed or operations[].speed` and writes
nothing. A kerf (torch) operation does not need a speed. Without `--post`, the test-sink NGC
does not require one.

Roughing leaves `wall` stock on the walls and `floor` stock above the floor. The finish pass
removes that remaining stock.

### Profile tabs

`tabs` on a profile uses the same object as [`profile.tabs`](#profile-document): `width`, exactly one of `count` and `spacing`, and an optional `height`. Lengths scale with `units`. `count` does not. `width` is the width of the bridge left on the part, measured along the part edge. On a straight edge the tool-centre gap is `width` plus the tool diameter (or the kerf). On an arc or a corner the gap is long enough that the uncut material along the part edge is at least `width`. For `side` `on`, the bridge is `width` centred on the line and the gap is `width` plus the tool diameter as well. `spacing` must be greater than `width` plus the tool diameter.

The bridge top is `depth - height` below the stock surface. A pass whose Z is above that top cuts the whole contour. A pass below it feeds up to the bridge top, follows the compensated contour (lines, arcs and corners), and feeds back down. Tabs are placed on every compensated contour that reaches them, including a finish pass at the tool radius. `stock_to_leave` still holds the roughing passes off the wall and above the floor. When `height` is omitted or 0, every pass feeds up to the retract height, rapids across at that height, and feeds back down. An XY rapid below the retract height is an error.

Ramp and helix entries start off a tab, and they are sized from the retract height when that height is above the stock, so each descent stays within `angle_deg`. A ramp that would cross a tab below the bridge top lifts across the bridge instead of descending through it. If the contour start falls inside one, the cut begins at the end of that tab. On an open contour the tabs are spaced along its length and later passes reverse direction. A tilted `plane` lifts the tab along the tool axis, because the pass is planned in that view.

`examples/tabs.job.json` profiles the outline of `examples/sample.dxf` with four tabs.

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

A drill is expanded into moves: each hole is a rapid to clearance over the hole, a rapid to the job retract height (the R plane), a plunge feed to depth (pecking back to the R plane when `step_down` is less than `depth`), an optional `dwell`, and a rapid out to clearance. No canned cycle is written. `step_down` on a drill is the peck depth. Pocket strategy, leads and tabs are not part of this version.

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

## Kinematics and tilted planes (3+2)

`kinematics` is a path, relative to the job file's directory, of a kinematics JSON file. `solution` chooses the inverse solution a later post keeps. `campanion run` plans profile, pocket, drill, engrave, face and chamfer in the work plane and writes those toolpaths in the part frame.

```json
"kinematics": "table_table_bc.kin.json",
"solution": "nearest"
```

| Field | Required | Meaning |
| --- | --- | --- |
| `kinematics` | no | Path of the kinematics file, relative to the job file. Loaded with the IR parser. Cannot be combined with `rotary`. |
| `solution` | no | `nearest` (the default), `first_positive` or `first_negative`. Requires `kinematics`. |

An operation may set `plane`. View +X is `x_dir`, view +Z is `normal`, and the view origin is `origin`. View +Y is the right-handed cross product of those two axes. The drawing is read in that view, with no geometry change, and planned with the same 2D operations as a flat job. Each pose is then mapped into the part frame: the tip is the view point placed on the frame, and the tool axis is the plane normal. The pose carries no rotary angle. An arc whose plane normal maps onto XY, ZX or YZ stays an arc. Any other arc is replaced by lines within the job chord tolerance.

When the next operation's frame differs (the origin or an axis differs by more than 1e-9), the planner appends a rapid to the operation being left and prepends a rapid to the next one. Each rapid keeps the tool's view X and Y and moves to the job clearance measured along that frame's normal. The rapid's tool axis is that frame's normal. The first operation that sets `plane` gets only the approach rapid. An operation that omits `plane` uses the part axes: origin at zero, view +X along part +X, view +Z along part +Z.

```json
"plane": { "origin": [0, 0, 0], "normal": [0, 1, 0], "x_dir": [1, 0, 0] }
```

| Field | Required | Meaning |
| --- | --- | --- |
| `origin` | yes | `[x, y, z]` of the plane origin. A length in `units`. |
| `normal` | yes | `[i, j, k]` tool axis in the part frame. A direction, not scaled. Stored as a unit vector. |
| `x_dir` | no | `[i, j, k]` view +X. A direction, not scaled. Stored as a unit vector in the plane. Omitted, part +X is projected onto the plane. When the normal is within 1e-9 of ±X, part +Y is projected instead. |

`plane` needs `kinematics`. It cannot be combined with `index_deg`. It is not supported on `wrap`, `rotary_finish`, `surface`, `rough` or `waterline`. On load, every operation is checked against the kinematics: a plane uses its origin and normal, and an operation without a plane uses tool axis +Z. A job with kinematics and no plane is planned in the part frame, and the program stores the five-axis setup.

Load failures:

- `kinematics: ` followed by the read or parse error
- `solution needs a kinematics file`
- `kinematics and rotary cannot be combined`
- `operation `<name>`: plane needs a kinematics file`
- `operation `<name>`: plane normal is zero`
- `operation `<name>`: plane x_dir is parallel to the normal`
- `operation `<name>`: plane and index_deg cannot be combined`
- `operation `<name>`: plane is not supported on wrap`
- `operation `<name>`: plane is not supported on rotary_finish`
- `operation `<name>`: plane is not supported on surface`
- `operation `<name>`: plane is not supported on rough`
- `operation `<name>`: plane is not supported on waterline`
- `operation `<name>`: ` followed by the kinematics error when that pose is not reachable

`examples/tilted.job.json` profiles `examples/sample.dxf` flat, profiles it again on a 30° plane, and drills a side plane. A post with no `five_axis` block still rejects a tilted tool axis with `post `<id>` has no tilted-plane output`.

A post with `five_axis` writes indexed 3+2. Before the first move of a toolpath whose tool axis changed, and before the first toolpath of a five-axis program, it retracts, indexes the rotaries, and turns the frame on. `mode` `work` writes the part tip rotated about the origin by the table angles (the control applies the rotary centres). `mode` `machine` writes the gauge point from the kinematics inverse with tool length 0. The first move after an index is a rapid in X and Y, then a rapid in Z. At the end the frame is cancelled, the tool retracts, and the rotaries return to zero. `haas_umc` is work mode (G254/G255). `linuxcnc_5ax` is machine mode. The kinematics file has to match the machine; this output has not been proven on a machine.

## Rapid safety

`campanion run` rejects a mill toolpath when a rapid ends or travels below the safe plane inside the part. In the planning frame the tool axis is +Z. For a drawing, stock top is Z 0 and the safe plane is the job retract height (the R plane). For a mesh, the floor is the higher of the stock top (the highest Z of that view) and the retract height. The footprint is the selected contours, or the mesh bounds in XY, grown by the tool radius. A kerf tool is not grown.

`operation <name>: rapid ends at Z <z> below the safe plane <floor> inside the part`

`operation <name>: rapid traverse at Z <z> below the safe plane <floor> inside the part`

Wrap and rotary finish are already in the part frame. The cut surface is the largest radius of a feed tip from the rotary axis. A rapid that ends inside that radius fails with:

`operation <name>: rapid ends at radius <r> below the cut surface <floor>`

On a 3-axis program, with no rotary setup and no kinematics, the IR verifier's rapid findings are blocking as well. Nothing is written and the process exits 1. Toolpath and block numbers are 1-based. An XY rapid that stays between the retract height and the clearance height is not an error. Rotary and 3+2 programs use the planning-frame check above.

`toolpath <T> block <B>: rapid ends at Z <z> below the safe plane <r>`

`toolpath <T> block <B>: rapid traverse at Z <z> below the safe plane <r>`

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

`campanion run` accepts `--up-axis z|y|auto`. The default is `z`. `auto` is opt-in. DXF, SVG and
STL jobs ignore it. SolidCraft itself now asks which axis is up rather than guessing, so
CAMpanion does not guess by default.

- `auto` trusts the exporter named in the STEP header (`exporter` and `exporter_up`, as SolidCraft
  reports them). SolidWorks means Y up: the part is turned a quarter turn about +X so Z is up.
  That guess follows the header, not a measurement, so a SolidWorks file saved Z up is still
  reported as Y up. Pass `--up-axis z` to keep the file's axes. Any other `exporter_up`, a missing
  field, and an older SolidCraft that sends no exporter fields all leave the file as Z up. When
  `auto` does turn a file it prints one note on stderr naming the exporter.
- `z` keeps the file's axes and does not ask about the header.
- `y` turns the part the same way without asking. File Y becomes Z and file Z becomes -Y, so a
  file point `(x, y, z)` is `(x, -z, y)` afterwards.

`face` is in the resulting frame. After a Y-up turn, give the point in that turned frame:
file `(x, y, z)` is `(x, -z, y)`.

- The tool is found through the `CAMPANION_SOLIDCRAFT_CLI` environment variable (path to the
  `solidcraft-cli` executable) or, failing that, `solidcraft-cli` on `PATH`. Without it, a STEP job
  stops with an error that says so; DXF and SVG jobs don't need it.
- The face DXF has a single layer, so operations select contours by nesting instead: `OUTER` is
  the face outline (and any islands inside holes), `INNER` is the holes.
- The DXF is written to a private temporary directory that is removed afterwards.

`examples/step.job.json` cuts `examples/plate.step` (generated with `solidcraft-cli`).

## Lathe jobs

`campanion lathe JOB -o OUT [--ir OUT.json] [--post ID|FILE.json] [--up-axis z|y|auto]` loads a `campanion-lathe/1`
file. It is not a `campanion-job/1` file: the fields below are the whole schema, and unknown
fields are rejected with a message that names the key. Every number must be finite. Lengths are
millimetres. Feeds are millimetres per minute. Constant surface speed is metres per minute.
Direct spindle speed is revolutions per minute.

`examples/shaft.lathe.json` faces, centre-drills, roughs and finishes the half-profile in `examples/shaft.dxf` (a Ø30 journal at the front that steps up to Ø40 toward the chuck), then grooves the Ø40, single-point threads the Ø30 journal as M30×2.5, and parts off. `examples/angled.lathe.json` finishes that same shaft at B 0, then finishes it again at `b_deg` 30 so the second insert's effective orientation is 45°. `examples/shaft_fillet.lathe.json` is the same part with an R3 front fillet, read from `examples/shaft_fillet.step`; that job needs SolidCraft's `solidcraft-cli`. `examples/millturn.lathe.json` faces that shaft, then drills four holes on the front face and cuts an axial slot on the Ø40, with the spindle indexed as C. `examples/millturn_flat.lathe.json` mills one flat on a Ø40 field of that shaft with the Y axis. `examples/subspindle.lathe.json` faces and finishes `examples/subspindle.dxf` on the main spindle, hands the part to the sub, then faces and finishes the same half-profile on the sub. `examples/transfer.lathe.json` faces and finishes that bar, transfers it with a phased sync and a part-off, faces the sub, then ejects. `examples/millturn_face.lathe.json` mills a 16×10 mm front-face pocket from `examples/millturn_face.dxf` with face interpolation. `examples/tailstock.lathe.json` is the shaft example with a tailstock advance after the finish and a retract before the part-off. Rough and finish follow the profile face to the axis, so they run before the quill advances. `examples/catcher.lathe.json` is that shaft with a parts catcher on the part-off.

```json
{
  "format": "campanion-lathe/1",
  "source": { "path": "shaft.dxf", "layer": "PROFILE" },
  "units": "mm",
  "stock": { "kind": "bar", "diameter": 50, "length": 60, "front_allowance": 1.0 },
  "spindle": { "max_rpm": 3000 },
  "tools": [
    { "id": 1, "station": 1, "shape": "c80", "nose_radius": 0.8, "orientation_deg": 45, "hand": "right", "tip": "centre" }
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
| `spindle` | yes | `max_rpm`, positive and at most 100000. That value is the constant-surface-speed cap. Optional `c_axis` (boolean, default false). `true` builds spindle `main` with C axis id 0. |
| `sub` | no | Second spindle. `max_rpm` (positive, at most 100000), `source` (the same shape as the job `source`), and `back_allowance` (`>= 0`, default 0). Adds IR spindle `sub` with no C axis, work zero 0 and `reversed` false. |
| `tailstock` | no | Centre or quill at the finished face. `diameter` (positive) and `clearance` (`>= 0`, default 1). Lengths are millimetres. Not allowed together with `sub`. |
| `parts_catcher` | no | Largest part a `part` operation may catch. `max_diameter` and `max_length` are both required, finite and positive, in job units. |
| `head` | no | Y travel and an optional tilting B head for turret `turret1`. The tilting B head is a rotary milling head; it is not the linear B axis of a positioned Haas ST tailstock (Haas NGC lathe operator's manual, §4.66–4.68) or of the Haas DS secondary spindle. `y_axis` is a boolean, default false. When it is false, `y_min` and `y_max` are rejected. When it is true both limits are required, with `y_min < 0 < y_max`, and the head lists axis Y. There is no default travel: the job supplies the numbers (a Haas ST-Y class machine is about ±50.8 mm). A tool-centre Y outside `[y_min, y_max]` is `operation NAME: Y VALUE outside head travel`. Optional `b_axis` is `{ "min_deg", "max_deg", "pivot_length" }`. `min_deg` must be less than `max_deg` and both must be finite (`head.b_axis: min_deg must be less than max_deg`). `pivot_length` is the distance from the pivot to the tool tip along the tool axis, in millimetres (the job unit), default 0, and must be zero or positive (`head.b_axis: pivot_length must be zero or positive`). When `b_axis` is present the head lists axis B. |
| `tools` | yes | Inserts and live tools, at most 256. May be empty when `operations` is empty. |
| `operations` | yes | Face, rough, finish, thread, groove, part, drill, `transfer`, `eject`, `wait`, `tailstock`, `live_drill`, `live_slot`, `live_flat` and `live_mill`, in order, at most 1024. |
| `tolerance` | no | `linear` (default 1e-6, must be positive) is the profile and finish tolerance. `chord` (default 0.01) is the post read-back tolerance and the gouge tolerance. The finished section is sampled at `chord`. `angular` is accepted, must be `>= 0`, and is not used. |
| `clearance` | yes | Approach gap, millimetres, `>= 0`. Roughing retracts by the same distance. |

`source` chooses the reader by extension:

- `.dxf` — [`TurnProfile::from_dxf`](../crates/turn/src/profile.rs). Optional `layer`. `axis` and `step` are rejected.
- `.svg` — `from_svg` when `layer` is omitted. A layer uses the SVG importer and `TurnProfile::from_drawing`. The SVG fit uses `linear` as its chord; the job `chord` is the post read-back.
- `.stl` — `from_mesh`. Requires `axis` (`x`, `y` or `z`) and a positive `step`. A `layer` is rejected.
- `.step` and `.stp` — [`TurnProfile::from_section_dxf`](../crates/turn/src/section.rs). `solidcraft-cli` sections the solid through world Z after `--up-axis` (`z`, `y`, or `auto`; default `z`). `auto` turns Y up onto Z when the exporter header says Y. The part must be turned about Z, with the finished front face where you want lathe Z 0, because model Z is lathe Z. Optional `body` names the solid and is required when the file lists more than one. The importer keeps one half (the +X half when both agree, otherwise the half that still joins from the axis back to the axis), which is how a seam on the section plane is avoided. A through-bore is not supported yet.

Drawing X is Z and drawing Y is the radius for `.dxf` and `.svg`. Z = 0 is the finished front face and the part extends toward −Z. A closed drawing is opened by dropping the spindle-axis run.

### Sub-spindle

`sub` is optional. The sub profile is drawn in the sub frame with the same convention as the main profile: the back face is at Z 0 and the part extends toward −Z. Drawing X is still Z and drawing Y is still the radius. Sub stock uses the main stock's diameter and length. `back_allowance` is the facing depth on the sub, from `+back_allowance` down to 0, the same role `stock.front_allowance` has on the main. The IR stock record stays the main blank. `turning.section` stays the main profile, sampled at `tolerance.chord`. A face, rough or finish on the sub is gouge-checked against the sub profile.

```json
"sub": {
  "max_rpm": 2500,
  "source": { "path": "subspindle.dxf", "layer": "PROFILE" },
  "back_allowance": 0.5
}
```

```json
{ "name": "sub_face", "type": "face", "tool_id": 2, "spindle": "sub", "step_z": 0.5, "feed": 150, "speed": { "css": 160 } }
```

`examples/subspindle.dxf` is an open half-profile from `(0, 0)` to `(0, 10)` to `(-20, 10)`: a Ø20 cylinder, 20 mm long, front face on the axis. The job stock is a Ø30 bar of that length. Main `front_allowance` is 1 and the main face steps 0.5. Sub `back_allowance` is 0.5 and the sub face steps 0.25. Main `max_rpm` is 3000 and sub `max_rpm` is 2500, so a CSS op on each spindle clamps at a different speed.

### Part transfer

`transfer` hands the part from the main chuck to the sub. It needs the `sub` block. Without it the error is `operation NAME: transfer needs a "sub" block`. The operation is one toolpath bound to `main`. `approach_z` and `grip_z` are sub-housing positions in work coordinates, written as the post writes them. `park_z` is that housing in machine coordinates. `feed` is the grip feed and must be positive. `rpm` is the main spindle speed during the handoff, `>= 0`, default `0` (both spindles stay stopped after the sync). `phase` is optional degrees, finite, from 0 up to but not including 360. `dwell` is the pause at the approach and after the sub clamps, seconds, `>= 0`, default `0.5`.

`part_off`, when set, is a part operation without `name`, `type` or `spindle`. It uses the same fields as `part` (`tool_id`, `z`, optional `to_diameter`, optional `chamfer`, optional `peck`, `clearance`, `feed`, `speed`). It does not accept `catch`. The cut runs after the sub has clamped, and the spindle is not stopped at the end of that cut. Without `part_off` the main chuck unclamps before the retract and clamps again after the sub parks.

The blocks, in order:

1. Unclamp the sub.
2. Stop the main spindle and stop the sub spindle.
3. Engage spindle sync. `phase` is sent when it is set.
4. When `rpm` is above 0, start the main spindle at that rpm. Direction follows the main spindle's reversed flag. A job leaves that flag false, so the handoff runs forward.
5. Rapid the sub housing to `approach_z`, dwell, feed to `grip_z`, clamp the sub, dwell.
6. With `part_off`: that part-off toolpath (tool change, speed, cut, retract) and no spindle stop. Without it: unclamp the main.
7. Rapid back to `approach_z`, cancel sync, stop the main, rapid the housing to `park_z` in machine coordinates.
8. Without `part_off`: clamp the main.

`eject` is `{ "name", "type": "eject" }` and is bound to the sub. It stops the sub spindle and unclamps the sub chuck. On a Haas post the spring part ejector acts when the jaws open (Haas NGC lathe manual 96-8910 §8.1.35). It needs the `sub` block: `operation NAME: eject needs a "sub" block`.

An operation with `spindle` `sub`, and an `eject`, must follow a `transfer`. Otherwise the error is `operation NAME: sub-spindle op before any transfer`.

```json
{
  "name": "handoff",
  "type": "transfer",
  "rpm": 600,
  "phase": 90,
  "approach_z": 5,
  "grip_z": -2,
  "park_z": -100,
  "feed": 200,
  "dwell": 0.5,
  "part_off": { "tool_id": 2, "z": -18, "clearance": 2, "feed": 80, "speed": { "rpm": 800 } }
}
```

```json
{ "name": "drop", "type": "eject" }
```

`examples/transfer.lathe.json` is that handoff on `examples/subspindle.dxf`: main face and finish, transfer with `phase` and `part_off`, a sub face, then eject.

### Tailstock

`tailstock` is optional. `diameter` is the diameter of the centre or quill where it meets the finished face, and it must be positive. `clearance` is the gap around that cylinder, `>= 0`, default `1`. Lengths are millimetres. Unknown fields are rejected. A job that also has `sub` is rejected: `"tailstock" is not allowed with a "sub" block (the B axis is the secondary spindle)`.

```json
"tailstock": { "diameter": 12, "clearance": 1 }
```

```json
{ "name": "support", "type": "tailstock", "action": "advance" }
```

`action` is `advance` or `retract`. Anything else is `operation NAME: tailstock action must be advance or retract`. The operation is one event-only toolpath on channel 0, bound to `main`, with no tool. Without the block the error is `operation NAME: tailstock needs a "tailstock" block`. A second advance is `operation NAME: tailstock is already advanced`. A retract while it is not advanced is `operation NAME: tailstock is not advanced`.

While the tailstock is advanced, every move of every later operation is checked in the main part frame. X is a radius and Z is axial. Z 0 is the finished face and the tailstock sits on the +Z side. The zone is `z` greater than `-clearance` and a radius less than `diameter / 2 + clearance`. Each move is checked as the straight segment from the previous tip to its end, rapids included, so a diagonal rapid between two points outside the zone still fails when it cuts the zone corner. The first move of an operation starts at the last tip of the previous main operation. An arc is checked by its chord. A move that enters the zone is `operation NAME: move at X{diameter:.3} Z{z:.3} enters the tailstock zone`, reported at the point of that move with the smallest radius inside `z > -clearance`. The X in that message is the diameter (`2 * radius`), printed to three decimal places, and Z is printed the same way. A radius of exactly `diameter / 2 + clearance`, and a Z of exactly `-clearance`, pass. Face interpolation uses the distance from the spindle axis. Cylinder interpolation keeps the radius in X, because Y is the unrolled distance. A `transfer`, an `eject`, or any operation on spindle `sub` while advanced is `operation NAME: retract the tailstock first`. Ending the program with the tailstock still advanced is allowed. A face pass, a drill on the front face, and live work on the front face enter that cylinder, so they fail while the tailstock is advanced.

On a Haas ST the post writes only `M21` to advance and `M22` to retract. Hold force and travel are machine settings, not program words (Haas NGC lathe operator's manual, 2025, §8.1.11). `M21` uses Settings 341 and 342 (rapid position and advance distance) and `M22` uses Setting 105 (retract distance). The ST-10 quill uses none of them. Hold force is set at the hydraulic unit on the ST-20/30 and by Setting 241 on the ST-40 servo tailstock (§4.66, §4.68). On a hydraulic quill tailstock (a body positioned by hand and a hydraulic quill, as on the ST-10) the body must be locked before `M21` moves the quill (§4.65, §5.60). On a positioned tailstock, Haas warns not to program `M21` when the tailstock was positioned manually, because it backs away and re-advances, which may drop the part (§8.1.11). `M321`/`M322` are only for boxway lathes with a tow-along tailstock and are not written (§8.1.54). There is no programmed B move for the ST tailstock. On a DS lathe the B axis is the secondary spindle and there is no tailstock. Only `haas_lathe` has a tailstock block. Any other post rejects a tailstock event with `post ID has no tailstock`, where ID is that post's id.

`stock.kind` is `bar` (`diameter`, `length`, `front_allowance`) or `tube` (`outer_diameter`, `inner_diameter`, `length`, `front_allowance`). Diameters and `length` are positive. `front_allowance` is `>= 0`. The IR stock still occupies Z in `[-length, 0]`; the allowance is only the facing depth, from `+front_allowance` down to 0. A tube's inner diameter must be below its outer diameter.

Each tool sits on head `turret1` (12 stations). `id` is the tool-table id. `station` is 1 to 12, and ids and stations are unique. An insert tool has `shape` `c80`, `d55`, `v35`, `w80`, `t60`, `s90`, `r`, `groove` or `thread`. A groove needs a positive `width`; other shapes may set it. `nose_radius` is positive. `orientation_deg` is the bisector from the nose into the holder, measured from +Z toward +X, in −360 to 360. A right-hand outside tool turning toward the chuck has its holder up and toward +Z, so 45° is the usual outside orientation: an 80° insert at 45° leads at 95° and clears the end by 5°. 135° points the holder toward the chuck and gouges a shoulder. `hand` is `left`, `right` or `neutral`. `tip` is `centre` or `imaginary`. `holder`, when set, is `{ "width", "length" }`. Both are positive millimetres. The holder is a rectangle of that width, centred on the bisector, starting at the back of the insert and extending `length` into the shank. An insert's tool-table entry stores no milling diameter, and the insert is mounted on the station.

A live tool replaces the insert. Its keys are `id`, `station`, `live` (`axial` or `radial`), `kind` (`drill` or `endmill`), `diameter` (positive, at most 1000000 mm), optional `point_angle_deg` and optional `max_rpm` (positive, at most 100000). `point_angle_deg` is drill only, defaults to 118, and must be between 0 and 180. It is checked and then dropped: a live hole does not add a point allowance. Any insert key (`shape`, `nose_radius`, `orientation_deg`, `hand`, `tip`, `width`, `holder`) on a tool that has `live` is an error. A live tool is not mounted as an insert and is not gouge-checked. Its diameter is stored on the tool table. When any tool is live, the head's `live_tooling` flag is true.

`tip: "imaginary"` programs the theoretical sharp corner. The IR insert offset is `(dx, dz)` in `(X, Z)`, from [`Insert::tip_offset`](../crates/ir/src/turning.rs): `(-r·sign(sin o), -r·sign(cos o))`, with `sign(0) = 0`. The turning path stores that pair as `dz`, `dx`. At 45° both components are `−nose_radius`. At 135° `dx` is `−nose_radius` and `dz` is `+nose_radius`. `tip: "centre"` programs the nose centre.

`speed` is either `{ "css": metres_per_minute }` or `{ "rpm": revolutions_per_minute }`, not both. CSS uses that operation's spindle `max_rpm` as its cap: the main spindle, or `sub.max_rpm` when the operation names `sub`. The speed is stored on the toolpath binding (spindle `main` or `sub`, head `turret1` on channel 0 and head `lower` on channel 1), not on the process-on parameters. A live cycle's binding stores no speed. Its `rpm` is the live-tool speed, not a turning-spindle speed. A wait stores no speed.

Every cutting operation takes optional `spindle`: `main` (the default) or `sub`. `sub` needs the top-level `sub` block, or the error is `operation NAME: spindle sub needs a "sub" block`. Any other value is `operation NAME: unknown spindle \`X\``. `live_drill`, `live_slot`, `live_flat` and `live_mill` on `sub` are `operation NAME: live tooling on the sub-spindle is not supported`. A sub operation is generated from the sub profile and the sub stock (main diameters and length, front allowance `back_allowance`) and bound to spindle `sub`. Sub-spindle operations may use either channel.

Every cutting operation takes optional `channel`: `0` (the default) or `1`. Any other number is `operation NAME: channel must be 0 or 1`. Channel 0 binds head `turret1`. Channel 1 binds head `lower`, which is added to the setup only when some operation uses channel 1 or the job contains a wait. That head lists the same axes, station count and live-tooling flag as `turret1`. Inserts stay mounted on `turret1`.

### Two channels

`wait` is `{ "name", "type": "wait" }`. Any other key, including `channel`, is an unknown field. Each wait inserts `Event::Sync` with the next id (1, then 2, and so on, in job order) as an event-only toolpath on channel 0 and the same event on channel 1. Those paths sit at the wait's place in each channel: channel 0 sees the channel 0 cuts before the wait, then the sync, then the channel 0 cuts after it.

`examples/two_channel.lathe.json` faces and roughs `examples/shaft.dxf` on channel 0, drills on channel 1, waits, finishes on channel 0 and grooves on channel 1. `--post generic_lathe_2ch` writes one header, then `(CHANNEL 1)` with the channel 0 paths and `M101` for sync id 1, then `(CHANNEL 2)` with the channel 1 paths and `M101`, then one program end. That post is a template: two-path controls use machine-specific wait M-codes and program layout; check the control manual.

Posting a program that uses a channel other than 0 prints one stderr line per segment and a total. Segment 0 is the time before the first sync. `time_s` follows the verify time model (feed length over feed, times 60, plus dwells; a rapid with no feed is free). `idle_s` is the slowest channel in that segment minus this channel. `cycle_s` is the sum of those per-segment maxima.

```
segment 0 channel 0 time_s 12.500000 idle_s 0.000000 channel 1 time_s 4.000000 idle_s 8.500000
total channel 0 time_s 20.000000 channel 1 time_s 9.000000 cycle_s 24.500000
```

B is a head rotation in the XZ plane. At B = 0 the tool sits as defined in the tool table (`orientation_deg`). Positive B rotates the holder from +Z toward +X by B degrees, so the effective insert orientation is `orientation_deg + b_deg`, normalised into −360..=360. The tool axis unit vector from the tip toward the pivot is `axis_x = cos(B)`, `axis_z = sin(B)`: B = 0 is a radial tool axis (+X) and B = 90 is axial (+Z).

`face`, `rough`, `finish`, `groove` and `part` accept optional `b_deg` (finite). The value must lie in `[min_deg, max_deg]` or the job is `operation NAME: b_deg VALUE outside head B travel`. A `b_deg` key with no `head.b_axis` is `operation NAME: b_deg needs head.b_axis`. `thread`, `drill` and the live operations do not take `b_deg`; the key is `unknown operation field `b_deg``. Planning uses an insert whose `orientation_deg` is the effective orientation, so the gouge check, the holder clearance and nose compensation (the imaginary tip is rebuilt from that insert) all see it. The tool-table entry stays as written. The mounted `TurnInsert` copy carries the effective orientation when every use of that tool shares one angle (a missing `b_deg` counts as 0). When the angles differ, the mount keeps the table orientation.

An operation with `b_deg` emits `Event::HeadIndex` (`type` `head_index`) right after its tool change, or at the start of the path when the tool does not change, whenever the angle differs from the current one. Each channel starts at 0. The first operation after a B operation that omits `b_deg` emits `HeadIndex` with `b_deg` 0, including thread, drill, live, transfer, eject, tailstock and wait (a wait resets each channel on its own). The pose axis on a turning path in a job that has `b_axis` is `(cos B, 0, sin B)` for that operation's angle. `examples/angled.lathe.json` is the shaft above: an outside finish at B 0 and a second outside finish of the 45° chamfer at `b_deg` 30.

Operations, in cut order. Names are unique and non-empty.

- `face` — `step_z` and `feed`. Optional `b_deg` when the head has `b_axis`. Passes run from the stock front (`+front_allowance` on `main`, `+back_allowance` on `sub`) to Z = 0. The radial feed starts at the stock outer radius plus `clearance` and ends at `−nose_radius` (one nose past the spindle axis). A centre tip is given a target radius of 0 because facing already shifts that end by `−nose_radius`. An imaginary tip is given `−nose_radius` directly.
- `rough` — `side` (`outside` or `inside`), `depth_of_cut`, `leave_x`, `leave_z`, `feed`. Optional `b_deg` when the head has `b_axis`. The stock outer and inner radii are the bar or tube radii (a bar's inner radius is 0). Retract equals `clearance`. Longitudinal roughing stops at the first boundary it cannot pass.
- `finish` — `side` and `feed`. Optional `b_deg` when the head has `b_axis`. `leave_x` and `leave_z` default to 0. The offset tolerance is `tolerance.linear`.

Diameters in a thread, groove or part operation are job diameters. The cycle sees half of that number as a radius. A thread `depth` is already radial and is not halved. A drill `diameter` stays a diameter.

`thread` is a single-point thread. `rpm` is required and is a positive number, not a `speed` object. Constant surface speed is rejected: a `speed` key is an unknown field. The recorded cut feed on the path is 100 mm/min because a cutter-location path requires one; the thread moves themselves carry the pitch as a lead, not that feed. `included_angle_deg` is 60 and is not a job field.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `side` | yes | `side` is `outside` or `inside`. |
| `major_diameter` | yes | Outside major diameter, or the bore diameter for an inside thread. Stored as a radius. |
| `pitch` | yes | Lead, millimetres per revolution. |
| `z_start`, `z_end` | yes | The cut runs toward the chuck, so `z_start` is greater than `z_end`. |
| `depth` | no | Radial depth. Default `0.6134 * pitch` (metric 60°). |
| `first_depth` | no | Radial depth of the first constant-area pass. Default `min(depth, 0.35)`. |
| `min_depth` | no | Smallest radial increment. Default `0.05`. |
| `spring_passes` | no | Extra passes at the final depth. Integer `0..=255`. Default `1`. |
| `infeed` | no | `flank` (default, 29.5°) or `radial`. |
| `clearance` | no | Axial room in front of `z_start` and the radial pull-off. Default `1.0`. |
| `rpm` | yes | Fixed spindle speed. Threading has no `feed` and no `speed` object. |

`groove` plunges with the insert's left corner. `peck` omitted is one feed to the floor.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `side` | yes | `outside` cuts inward. `inside` cuts outward. |
| `z_left`, `z_right` | yes | Groove walls. `z_left` is the smaller Z. |
| `bottom_diameter` | yes | Floor diameter. Stored as a radius. |
| `surface_diameter` | no | Diameter the groove starts from. Default is the stock outer diameter. Stored as a radius. |
| `insert_width` | no | Width along Z. Default is the tool's insert width. A groove insert must have one. |
| `peck` | no | Radial peck. Omit for one feed. |
| `stepover_frac` | no | Axial step as a fraction of the insert width, in `(0, 1]`. Default `0.8`. |
| `clearance` | no | Radial room outside the surface. Default `1.0`. |
| `dwell_s` | no | Pause at the floor, seconds, `>= 0`. Default `0`. |
| `feed`, `speed` | yes | `speed` is `{css}` or `{rpm}`, the same object as face, rough and finish. |
| `b_deg` | no | Head angle in degrees. Requires `head.b_axis`. Same field on `part`. |

`part` plunges at `z` with the same left-corner convention. The bar radius is the stock outer radius. The insert width is the tool's width, not a job field, so the tool must be a groove insert (or any shape that carries `width`).

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `z` | yes | `z` is the left edge of the kerf. |
| `to_diameter` | no | Diameter to feed to. Default `-0.5`, which is a radius of `-0.25` (past centre). Zero reaches the axis. |
| `chamfer` | no | 45° corner break, millimetres. Omit for no chamfer. |
| `peck` | no | Radial peck. Omit for one feed. |
| `clearance` | yes | Extra radius beyond the stock for the approach and the retract. |
| `feed`, `speed` | yes | Same `speed` object as a groove. |
| `b_deg` | no | Head angle in degrees. Requires `head.b_axis`. |
| `catch` | no | Boolean, default false. Swing the parts catcher in around this part-off. Needs a `parts_catcher` block. Not accepted on a transfer `part_off`. |

`parts_catcher` is optional: `max_diameter` and `max_length` are both required, finite and positive, in job units, and unknown fields are rejected. A `part` may set `catch` (default false). `catch` true without the block is `operation NAME: catch needs a "parts_catcher" block`. The stock outer diameter must be at most `max_diameter`, or the error is `operation NAME: part diameter X exceeds parts_catcher.max_diameter D`. The part length is `|z|` plus the insert width (`z` is the part operation's left-corner Z) and must be at most `max_length`, or the error is `operation NAME: part length X exceeds parts_catcher.max_length L`. Both numbers are printed to three decimal places, and a length equal to `max_length` passes. Weight is not checked (3.6 kg on the Haas catcher). Catching while the tailstock is advanced is `operation NAME: retract the tailstock before catching the part`. The part path extends the catcher after the approach rapid and before the first feed toward X, and retracts it after the final retract. On a Haas NGC lathe, M36 rotates the parts catcher into position to catch a part and M37 rotates it out of the work envelope (Haas NGC lathe operator's manual, 2025, §8.1.16). M35 moves it to the part-off position (§8.1.15) and is not written. The Haas parts catcher page lists a maximum part of 3 in / 76 mm diameter, 4 in / 102 mm long and 8 lb / 3.6 kg (https://www.haascnc.com/reference-container/Technical-Details/Options/product-options/parts-catcher.html); those sizes are job limits, not post defaults. Only `haas_lathe` writes the catcher, M36 before the part-off plunge and M37 after the retract. Any other post rejects the event with `post ID has no parts catcher`.

`drill` stays on the spindle axis. `clearance` is the job clearance, not a field of this operation. `rpm` is a positive number, the same rule as a thread.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id` | yes | The cycle does not read the insert. The tool still has to exist. |
| `z_start` | no | Face the drill starts from. Default `0`. |
| `depth` | yes | How far the tip goes past `z_start` along −Z. |
| `diameter` | yes | Drill diameter, used for the through-hole point allowance. Not converted to a radius. |
| `peck` | no | Axial peck. Omit for one feed. |
| `retract` | no | Pull-back between pecks. Default `1.0`. |
| `point_angle_deg` | no | Included point angle. Default `118`. Must be between 0 and 180. |
| `through` | no | When true, the tip goes deeper by the point length. Default `false`. |
| `feed`, `rpm` | yes | `rpm` is revolutions per minute. |

`live_drill`, `live_slot`, `live_flat` and `live_mill` need `spindle.c_axis` true. A front cycle needs an axial live tool. An OD cycle, including `live_flat` and an OD `live_mill`, needs a radial live tool. A front `live_mill` stays axial even when `y_axis` is true. `live_drill` and `live_slot` stay on Y = 0 and do not interpolate C. C changes only on a rapid at the clearance point, while the spindle brake is released. The pose axis is +Z for an axial tool and +X for a radial tool. `Pose.rotary` is the C angle in degrees. `rpm` is positive and at most 100000, and must not exceed the tool's `max_rpm` when that is set. `feed` is millimetres per minute. Unknown keys are rejected the same way as on any other operation. `live_flat` also needs `head.y_axis` true. A job without it is `operation NAME: head has no Y axis`. The `live_flat` tool must be an endmill. `live_mill` accepts a drill or an endmill.

`live_drill` drills each position in order. At most 64 positions. `c` is finite and in −99999 to 99999. The tip stops at the programmed depth. No point allowance is added.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `face` | yes | `face` is `front` or `od`. |
| `positions` | yes | Front: `[{ "c", "r" }]`, `r` the tool-centre radius and `>= 0`. OD: `[{ "c", "z" }]`, `z` the tool-centre Z. |
| `depth` | yes | How far the tip goes past the surface. Front bottom Z is `z_start - depth`. OD bottom radius is `diameter / 2 - depth` and must be `>= 0`. |
| `z_start` | front | Z of the face. Not accepted on an OD drill. |
| `diameter` | od | Diameter of the surface being drilled. Not accepted on a front drill. |
| `peck` | no | Positive peck. Omit for one feed to the bottom. Between pecks the tool rapids back to the clearance point. After the last peck it feeds back out at the cut feed. |
| `retract` | no | Clearance gap, `>= 0`. Default is the job `clearance`. |
| `rpm`, `feed` | yes | Live-tool rpm and cutting feed. |

Front clearance is Z `z_start + retract` at the hole radius. Moves from one front hole to the next rapid at that Z. OD clearance is radius `diameter / 2 + retract` at the hole Z.

`live_slot` cuts one indexed slot. The brake stays clamped for the whole slot. The tool plunges at `from`, feeds to `to`, steps down, and feeds back. The last pass stops exactly at `depth`. It then feeds out to clearance. `step_down` defaults to `depth`, which is one pass. There is no `retract` key: the gap is the job `clearance`.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `face`, `c` | yes | `c` is the index angle in degrees. |
| `from`, `to` | yes | Front: tool-centre radii, `>= 0`. OD: tool-centre Z. Either order. |
| `depth` | yes | Same depth rule as `live_drill`. |
| `step_down` | no | Maximum thickness of one pass. Default `depth`. |
| `z_start` | front | Z of the face. Passes sit at `z_start - k * step_down`. |
| `diameter` | od | Surface diameter. Passes sit at radius `diameter / 2 - k * step_down`. |
| `rpm`, `feed` | yes | Same as `live_drill`. |

`live_flat` mills one flat with a radial endmill. The tool axis is +X. The face of the cutter makes the flat, at a distance `across` from the spindle centre, with `0 < across < diameter / 2`. `diameter` is the stock diameter at the flat, not the tool diameter. Passes run in Y, from `-width / 2 - tool radius` to `+width / 2 + tool radius`. `width` defaults to the full chord at the finished flat, `2 * sqrt((diameter / 2)² - across²)`, and that same width is used on the shallower passes. Z steps by `stepover` times the live-tool diameter, from `z_from` to `z_to` (either order; equal values are one station). Radial passes step from the stock radius down to `across` by `step_down`. Omit `step_down` for one pass at `across`. `stepover` is a fraction in `(0, 1]`, default `0.6`. The approach rapid is at radius `diameter / 2 + clearance`, Y = 0, Z = `z_from`, before the brake is clamped. The first feed moves Y to the side in the air, then X feeds in at that side, then Y crosses the flat. Y and Z both zigzag.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `c` | yes | Radial endmill. `c` is the index angle in degrees. |
| `diameter` | yes | Stock diameter at the flat. |
| `across` | yes | Distance from the spindle centre to the finished flat. Below half of `diameter`. |
| `z_from`, `z_to` | yes | Z of the first and last stations. |
| `width` | no | Width along Y. Default is the chord at `across`. Any positive number. |
| `step_down` | no | Radial thickness of one pass. Default is the whole depth, one pass at `across`. |
| `stepover` | no | Z step as a fraction of the live-tool diameter, in `(0, 1]`. Default `0.6`. |
| `rpm`, `feed` | yes | Same as `live_drill`. |

`live_mill` cuts a drawing with the same profile and pocket operations a mill job uses (`ProfileOp` and `PocketOp`). It does not invent a second offsetter. Lengths in the drawing are millimetres. A DXF is millimetres when `$INSUNITS` is millimetres or unitless. An SVG is converted to millimetres; a unitless SVG is 96 dpi. `drawing` is a `.dxf` or `.svg` path relative to the job file, with the same path rules as `source`. Optional `layer` keeps one layer, compared as a string. `strategy` is `profile` or `pocket`. A profile needs `side` (`outside`, `inside` or `on`). A pocket rejects `side` and takes `stepover`, a fraction of the live-tool diameter in `(0, 1]`, default `0.4`. `depth` is positive. `step_down` is the maximum thickness of one pass and defaults to `depth`. `rpm` and `feed` match `live_drill`.

The toolpath is built in the drawing plane. Its Z is 0 on the surface and negative into the part, and the job `clearance` is the positive retract height. Rapids from one feature to the next stay at that clearance.

`face` is `front` or `od`.

- `front` needs `z_start`, the Z of the face, and an axial live tool. Drawing X and Y are face X and Y about the spindle centre (the drawing origin is the spindle centre). A toolpath point `(u, v, d)` becomes the pose `(u, v, z_start + d)` inside face interpolation. Arcs stay arcs in the XY plane. The clearance rapid before interpolation is on Y = 0; the move onto the drawing's Y is a rapid at clearance after interpolation is on, so the turning program never carries a face Y outside that mode. A path that crosses the spindle centre is rejected by the Haas post (`face path crosses the spindle centre`). This operation does not repeat that check.
- `front` with `y_axis` true does not use face interpolation. It indexes `c` (degrees, default 0, in −99999 to 99999) and cuts the drawing's X and Y directly, which is how a machine with a Y axis mills across the spindle centre. The head must have `y_axis` true or the job is `operation NAME: y_axis needs head.y_axis`. Every tool-centre Y, including arc extremes, must lie in `[y_min, y_max]` or the job is `operation NAME: Y VALUE outside head travel`. `c` without `y_axis` is `c is only valid when y_axis is true`. `z_start` is still the face. The first non-zero Y is a second rapid after a centreline rapid that carries C, so C and Y are not written in one block.
- `od` needs `diameter`, the cylinder being cut, and a radial live tool. It rejects `z_start`, `y_axis` and `c`. Drawing X is the distance around the circumference, unrolled at that diameter, with 0 at C0. Drawing Y is the part Z. A toolpath point `(u, v, d)` becomes the tip `(x = diameter / 2 + d, y = u, z = v)` inside cylinder interpolation at radius `diameter / 2`. `d` is 0 on the surface and negative into the part. Arcs stay arcs in the YZ plane: YZ `(y, z)` is drawing `(u, v)`, so the arc sense is unchanged. The clearance rapid before interpolation is at unrolled distance 0; the move onto the drawing's X is a rapid at clearance after interpolation is on. The drawing's X span, arcs included, must not be greater than the circumference `π * diameter`. A longer span is `operation NAME: drawing wraps past one turn`. Equality is one full turn and is accepted.

A pocket, and a profile with `side` `inside`, need a closed contour. None is `operation NAME: drawing has no closed contour`. An outside or on-profile drawing with no contour is `operation NAME: drawing has no contour`. Several closed contours are nested the same way as a mill pocket: even depth is a pocket, the next depth is a hole.

| Field | Required | Meaning |
| --- | --- | --- |
| `tool_id`, `face`, `drawing`, `strategy`, `depth` | yes | `face` is `front` or `od`. `strategy` is `profile` or `pocket`. |
| `layer` | no | Keep this layer. Omit to keep every layer. |
| `side` | profile | `outside`, `inside` or `on`. Rejected on a pocket. |
| `stepover` | pocket | Fraction of the cutter diameter, in `(0, 1]`. Default `0.4`. Rejected on a profile. |
| `step_down` | no | Maximum pass thickness. Default `depth`. |
| `z_start` | front | Z of the face. Rejected on an OD mill. |
| `y_axis` | no | Front only. Default false. Direct X/Y at the indexed C, with no face interpolation. |
| `c` | no | Front `y_axis` only. Index angle in degrees. Default `0`. |
| `diameter` | od | Cylinder diameter. The mapping radius is half of this. |
| `rpm`, `feed` | yes | Live-tool rpm and cutting feed, millimetres per minute. |

`campanion lathe` checks each face, rough and finish path against that operation's finished profile, using the effective insert orientation when the operation sets `b_deg`. A main operation uses the main profile. A sub operation uses the sub profile. The profile is the polyline sampled at `tolerance.chord`. The insert flank length is 4 mm, and the tool's `holder` is included when the tool has one. Samples are `min(0.25, chord * 10)` millimetres apart. A hit deeper than `chord` is a gouge. Each gouge is one stderr line, `gouge operation NAME z Z x X depth DEPTH KIND`, with `KIND` one of `nose`, `edge` or `holder` and the numbers printed to six decimal places. A gouge exits 1 and writes nothing. Thread, groove, part, drill, transfer, eject, tailstock and the live cycles are not checked: those tools are meant to enter the profile, or they are not inserts.

Each turning operation is one toolpath. The first time a tool id appears on that operation's channel, the path starts with a tool change. A B change then writes `HeadIndex` before the spindle turns on (or at the start of the path when there is no tool change). Then the spindle turns on, the moves run (the first of those is a rapid to the path start), and the spindle turns off at the end of that path. A `transfer` is one toolpath on `main` and does not follow that on/off pair: the blocks are the handoff above, and a nested part-off keeps the spindle running. An `eject` is one toolpath on `sub` with a sub stop and an unclamp, and no tool change. A `tailstock` operation is one event-only toolpath on `main`, channel 0, with no tool: advance or retract, and nothing else. A wait is two toolpaths, one per channel, and each is only the sync event. Nose compensation is already in the coordinates (`compensation: computed`). A live cycle uses the same tool-change rule and the same compensation, and it does not turn the work spindle on. For `live_drill` and `live_slot` the blocks are C-axis engage on spindle `main`, live tool on forward at `rpm`, then for each hole or for the slot a rapid to the clearance point at the new C, brake clamp, the feed (pecking retracts all the way to the clearance point between pecks), a feed back out, a rapid to clearance when that point is not already the end of the feed, and brake release. Then the live tool stops and C disengages. `live_flat` clamps before the live tool starts: tool change, C-axis engage, the Y = 0 approach rapid, brake clamp, live tool on, the feeds, live tool off, brake release, C disengage. `live_mill` does not clamp the brake, because interpolation rejects a clamped brake. Its blocks are tool change, C-axis engage, the clearance rapid, live tool on forward, interpolation on, the cut, interpolation off, live tool off, C disengage. A front `y_axis` mill omits the interpolation events. Rapids between features stay inside interpolation, at the clearance level.

Without `--post`, `-o` is the RS274/NGC test sink. ZX arcs are `G18`. That sink is not a post for a machine, and it rejects synchronised thread moves, so a job that threads needs `--post`. `--post linuxcnc_lathe` writes diameter mode (`G7`), the ZX plane (`G18`) and constant surface speed (`G96`); direct rpm is `G97`. A thread is `G33` with the lead on `K` for LinuxCNC and grblHAL, and `G32` with the lead on `F` for Fanuc and Haas. X in the IR stays a radius. Posted text is read back at `tolerance.chord`. A mismatch exits 1 and writes nothing. Verify errors are printed and the status is 1; nothing is written in that case either. A face, rough or finish gouge is printed on its own, one line per gouge, and the status is 1. `--ir` writes the IR JSON after a successful `-o`, and the two paths must differ. The IR stores the main finished section on `turning.section`, sampled from the main profile at `tolerance.chord`. A sub profile is not stored on the setup.

### Haas output

`haas_lathe` writes an indexed live cycle as `M154` then `G28 H0.` to engage C, `M155` to disengage, `M14` to clamp the spindle brake and `M15` to release it, `P<rpm> M133` to start the live tool forward, and `M135` to stop it. Posted X is a diameter. Y is not doubled. The work spindle is not started with `M3` while C is engaged. A Y move with a radial tool writes `G19` on its own line before the move. An axial tool writes `G17`. `G18` is restored before `M155`. There is no Y activation code. A front `live_mill` posts `G112` around the cut and `G113` to leave it, with no `C` word between them; under `G112`, X is a radius. An OD `live_mill` posts `G107 Y0 C0 R<radius>` and a later `G107` cancels it. A front `live_mill` with `y_axis` posts X and Y at the indexed C and does not post `G112`. Y axis, G17/G19 live milling planes, no activation code: Haas NGC Lathe Operator's Manual 6.12 / https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html

A toolpath bound to spindle `sub` is preceded by `G55` then `G14` (secondary work offset, then the spindle swap) before its tool change. `G14` mirrors Z on the control, so the posted Z words stay in the part frame (`mirror_z` false). When the program ends on the sub, `G15` then `G54` are written before the `G53` home lines. G14/G15: Haas NGC lathe manual 96-8910 §7.1.8 (G14 secondary spindle swap; G14 mirrors Z automatically; cancelled by G15/M30/reset). G55: §7.1.72 example: G55 for secondary spindle work offset.

A part transfer on `haas_lathe` unclamps the sub with `M111` and clamps it with `M110`, and uses `M11` / `M10` for the main chuck. `M145` stops the sub spindle; after the sub is selected with `G14`, the eject stops it with `M5` instead (Haas alarms on `M145` under `G14`). `M143 P{rpm}` and `M144 P{rpm}` run it forward and reverse. `G199` engages synchronous spindle control and `G199 R{phase}` adds the phase. `G198` cancels it. The sub housing rapids with `G00 B{z}`, feeds with `G01 B{z} F{feed}`, and parks in machine coordinates with `G00 G53 B{z}`. No Haas lathe post has a tilting B head: that B word is the sub-spindle housing. M10/M11: §8.1.6. M110/M111 and the spring ejector: §8.1.35. M143/M144/M145: §8.1.44. G199/G198: §7.1.71-72. B axis: §6.34.

- https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html
- https://www.haascnc.com/service/codes-settings.type=gcode.machine=lathe.value=G107.html
- https://www.haascnc.com/service/codes-settings.type=mcode.machine=lathe.value=M133.html
- https://www.haascnc.com/service/service-content/guide-procedures/lathe---m-codes.html
- https://www.haascnc.com/service/service-content/guide-procedures/lathe---g-codes.html

### LinuxCNC output

`linuxcnc_lathe` needs a machine configured with spindle 1 as the live tool and a C joint. The post writes the live tool as `S<rpm> $1` and `M3 $1` or `M4 $1`, and stops it with `M5 $1`. C-axis engage, disengage and the spindle brake are left empty, because LinuxCNC does that switch in HAL or a user M-code; a custom post can fill them. There is no `G112` or `G107`. Face and cylinder cycles fall back to X and C, and a feed that changes C uses inverse time (`G93`, back to `G94` on the next move that does not change C).

- https://linuxcnc.org/docs/stable/html/gcode/m-code.html
- https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g93-g94-g95
- https://linuxcnc.org/docs/stable/html/gcode/m-code.html#mcode:m100-m199
