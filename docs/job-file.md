# Job file (`campanion-job/1`)

A job file names a source part, a tool table, and the layer operations to run on that part.
It does not contain geometry and it does not contain a toolpath. A later step imports the part
and builds paths from the operations recorded here.

This is not the profile document read by `campanion ir --job` and `campanion ngc --job`. That
document has no `format` tag. `campanion-job/1` rejects it.

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
A shape other than flat is rejected when a profile, pocket, drill, engrave, face or chamfer uses
that tool. Those operations keep the flat diameter offset and do not read `shape`.

## Operations

Each operation selects layers of the source part. Layer names are case-sensitive, non-empty, and
unique within that operation. The same layer may appear on two operations.

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Short unique name. |
| `type` | yes | `profile`, `pocket`, `drill`, `engrave`, `face`, `chamfer` or `surface`. |
| `layers` | yes, except surface | One or more layer names. A surface rejects `layers`. |
| `side` | profile and chamfer | Profile: `inside`, `outside` or `on`. Chamfer: `inside` or `outside`; `on` is rejected. Rejected on pocket, drill, engrave, face and surface. |
| `tool_id` | yes | `id` of a tool in this file. |
| `depth` | yes, except chamfer and surface | Positive distance downward from the stock surface, in `units`. Not an absolute Z. A chamfer rejects `depth`; the tip depth follows from the geometry. A surface rejects `depth`. |
| `step_down` | yes, except chamfer and surface | Positive maximum thickness of one pass, in `units`. The last pass stops at `depth` and is not deeper. A chamfer rejects `step_down`. A surface rejects `step_down`. |
| `stepover` | pocket, face and surface | Positive step between passes, in `units`. At most the tool diameter. Rejected on a kerf tool. A face that omits it uses 0.75 times the tool diameter. A pocket that omits it leaves the choice to the path generator (the tool radius). A surface requires it. |
| `sample_step` | surface only | Positive length in `units`. Default is half of `stepover`. Maximum gap between the regular drop-cutter samples on a raster line, and between the samples on the link from one line to the next. |
| `angle_deg` | face and surface | `0` (along +X) or `90` (along +Y). Default `0`. Stored as radians. Rejected on every other kind. |
| `margin` | face and surface | Length in `units`, at least 0, at most 1e6 mm. Default 0. On a face, grows the faced rectangle past the geometry bounding box on every side. On a surface, grows the mesh XY bounds the same way. Rejected on every other kind. |
| `floor_z` | surface only | Absolute tip Z in `units`. Default is the mesh minimum Z. Where the cutter misses the mesh, the tip stays at this height. |
| `width` | chamfer | Positive length in `units`, at most 1e6 mm. Horizontal width of the bevel. Rejected on every other kind. |
| `v_angle_deg` | chamfer | Included angle of the V-bit, in degrees, greater than 0 and less than 180. Stored as radians. Not scaled by `units`. Rejected on every other kind. |
| `tip_diameter` | chamfer | Length in `units`, at least 0, default 0. Must be less than the tool diameter. Rejected on every other kind. |
| `tip_clearance` | chamfer | Length in `units`, at least 0, default 0. How far the tip sits below the bottom of the bevel. Rejected on every other kind. |
| `stock_to_leave` | profile, pocket and surface | On a profile or pocket, an object with optional `wall` and `floor` lengths, in `units`. Each omitted field is 0. `floor` must be less than `depth`. On a surface, a single length in `units`, at least 0, default 0. That length raises every cutter-location Z. It is an axial offset approximation, not a true 3D rest-material offset along the surface normal. Rejected on a face. |
| `finish` | profile and pocket | Object with optional `step_down` (positive length in `units`; omitted, equal to `depth`) and optional `floor` (bool, default true, pocket only). Rejected on a face. |
| `feeds` | yes | `cut` (required) and `plunge` (optional), in file-length units per minute. Omitted `plunge` means use `cut`. |
| `pierce_height` | no | Absolute Z to rapid to before the pierce delay. |
| `cut_height` | no | Absolute Z to feed to after the pierce, before the cut. |
| `timing` | no | Delays and the two process-on toggles. See below. |

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

### Surface

`type` `surface` is a 3-axis parallel finish of an STL mesh. The source path must end in `.stl`
(any case). An STL source rejects every other operation type, and `surface` is rejected on a
DXF, SVG or STEP source. There are no layers. A kerf tool is rejected.

The raster covers the mesh XY bounds, grown by `margin` on every side. Passes run at `angle_deg`
and alternate direction. `stepover` is the gap between passes. `sample_step` is the gap between
regular drop-cutter samples; the links between passes use the same spacing. The chord tolerance
of the job is the largest tip-Z error allowed when a raster chord replaces the true drop-cutter
height. `floor_z` is the tip height where the cutter misses the mesh.

`stock_to_leave` raises every cutter-location Z by that many millimetres after the drop-cutter
query. Clearance and retract stay at the job heights. Put those heights above the raised surface:
a rapid is still at clearance, and the retract feeds to the job retract height.

The toolpath rapids to clearance above the first point, turns the process on, plunges to the
first cutter location, cuts each polyline, cuts the drop-cutter samples that join one line to
the next, retracts to the retract height, turns the process off, and rapids back to clearance.

`examples/dome.job.json` finishes `examples/dome.stl` with a 6 mm ball nose, a 1 mm stepover and
a 0.01 mm chord tolerance. The mesh is a 40 mm square plate at Z = 0 with a dome of radius 15 mm
at the centre, rising to Z = 10 mm.

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

## Limits

Hostile numbers and sizes fail the load instead of being passed on:

- JSON file at most 32 MiB, UTF-8.
- 256 tools, 1024 operations, 64 layers on one operation.
- Names at most 256 bytes. Source path at most 4096 bytes.
- Tool size, depth, step-down, chamfer width, tip diameter, tip clearance, bull corner radius, margin, sample step, floor Z, surface stock, heights and tolerances at most 1e6 mm (angular tolerance at most 1e6 rad). An STL part read by `run` is at most 32 MiB, the same cap as a text source.
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
