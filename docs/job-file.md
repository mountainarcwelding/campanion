# Job file (`campanion-job/1`)

A job file names a source part, a tool table, and the layer operations to run on that part.
It does not contain geometry and it does not contain a toolpath. A later step imports the part
and builds paths from the operations recorded here.

This is not the profile document read by `campanion ir --job` and `campanion ngc --job`. That
document has no `format` tag. `campanion-job/1` rejects it.

The loader is `crates/cli/src/jobfile.rs`. No command calls it yet.

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

The source format is chosen by extension: `.svg` is read as SVG, `.step`/`.stp` as STEP (see below), anything else as DXF.
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

## Operations

Each operation selects layers of the source part. Layer names are case-sensitive, non-empty, and
unique within that operation. The same layer may appear on two operations.

| Field | Required | Meaning |
| --- | --- | --- |
| `name` | yes | Short unique name. |
| `type` | yes | `profile`, `pocket`, `drill` or `engrave`. |
| `layers` | yes | One or more layer names. |
| `side` | profile only | `inside`, `outside` or `on`. Rejected on pocket, drill and engrave. Same mapping as the profile job. |
| `tool_id` | yes | `id` of a tool in this file. |
| `depth` | yes | Positive distance downward from the stock surface, in `units`. Not an absolute Z. |
| `step_down` | yes | Positive maximum thickness of one pass, in `units`. The last pass stops at `depth` and is not deeper. |
| `feeds` | yes | `cut` (required) and `plunge` (optional), in file-length units per minute. Omitted `plunge` means use `cut`. |
| `pierce_height` | no | Absolute Z to rapid to before the pierce delay. |
| `cut_height` | no | Absolute Z to feed to after the pierce, before the cut. |
| `timing` | no | Delays and the two process-on toggles. See below. |

`depth` and `step_down` must be a legal `campanion-path` depth plan: positive, at most 1e6 mm,
and at most 1024 passes. Five millimetres at a two millimetre step is three passes (2, 4, 5).

Pocket strategy, stepover, leads, tabs and drill-cycle fields are not part of this version.

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
- Tool size, depth, step-down, heights and tolerances at most 1e6 mm (angular tolerance at most 1e6 rad).
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
