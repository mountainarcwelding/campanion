# User machine profiles

A machine profile is a `campanion-machine/2` JSON file that describes a machine the user owns.
`campanion-machine/1` files are still loaded and are migrated in memory (and rewritten as v2
when saved).

Every numeric value is entered by the user. CAMpanion does not ship machine limits, posts, or
material feeds, kerfs, or power values.

CAMpanion is primarily command-line driven. The GUI Machines manager is the editor for these
files; `campanion run` honours a job that names a profile.

## Location

Profiles are stored in the user config directory, one file per machine:

- `CAMPANION_CONFIG_DIR/machines/<id>.json` when that environment variable is set
- otherwise `$XDG_CONFIG_HOME/campanion/machines/<id>.json`
- otherwise `$HOME/.config/campanion/machines/<id>.json`

`<id>` is the file stem. It must start with a letter or digit and may contain letters, digits,
`.`, `_`, or `-`. Import and export copy these files.

This format is not the lathe job `machine.model` block (`haas_st15y` and similar). That block
stays a cited Haas lathe check and is not a user profile.

## File format

`type` is `gantry` or `resin`.

A gantry has a user-entered bed size and one or more heads. Heads may be mixed: `laser`,
`plasma`, `spindle`, `extruder`, or another kind (the field is extensible). Each head has a
process, an optional post, a material/tool library, and, when entered, per-head XYZ offsets
and a safe-Z. None of those numbers are defaulted.

A resin machine has no heads. `screen` and `bed` are optional and only stored when the user
enters them. `bed.z` is the vat/build height when entered.

```json
{
  "format": "campanion-machine/2",
  "name": "Shop laser",
  "units": "mm",
  "type": "gantry",
  "bed": { "x": 600, "y": 400 },
  "work_area": { "x_min": 0, "x_max": 600, "y_min": 0, "y_max": 400, "z_min": 0, "z_max": 80 },
  "controller": "grbl",
  "post": "grbl_laser",
  "heads": [
    {
      "id": "laser",
      "name": "CO2",
      "kind": "laser",
      "process": "laser",
      "post": "grbl_laser",
      "offset": { "x": 0, "y": 0, "z": 0 },
      "safe_z": 15,
      "max_power": { "unit": "w", "value": 40 },
      "materials": [
        {
          "name": "3 mm acrylic",
          "kerf": 0.15,
          "cut_feed": 300,
          "power": { "unit": "percent", "value": 80 },
          "passes": 1,
          "depth": 3
        }
      ]
    }
  ]
}
```

A resin example:

```json
{
  "format": "campanion-machine/2",
  "name": "Mars Pro",
  "units": "mm",
  "type": "resin",
  "bed": { "x": 130, "y": 80, "z": 160 },
  "screen": { "x": 130, "y": 80 }
}
```

`format`, `name`, `units`, and `type` are required. `units` is `mm` or `inch`; lengths in the
file are in those units and are stored internally as millimetres. `work_area`, `controller`,
`post`, `bed`, and `screen` may be omitted. An omitted work area means no travel check is run
from that block. A work area must include at least one of `x_min`, `x_max`, `y_min`, `y_max`,
`z_min`, `z_max`. Only the limits that are present are checked. A value equal to a limit
passes; a value outside it is a code-generation error.

A gantry needs at least one head. A resin machine must not have a `heads` array (or must leave
it empty) and must not be used as the selected head on a cutting job.

Each gantry head has a unique `id` and a `process` of `milling`, `routing`, `turning`, `laser`,
`plasma`, `waterjet`, `oxyfuel`, `wire_edm`, `sla`, or `fdm`. `kind` defaults to a value
derived from `process` when omitted (`laser`, `plasma`, `spindle` for milling/routing,
`extruder` for FDM, otherwise the process id). `name`, `post`, `offset`, `safe_z`,
`max_power`, and `materials` may be omitted. A head post, when set, overrides the
machine-level post. `offset` is all three of `x`, `y`, and `z` together; there is no implied
zero for a missing axis. `safe_z` and `max_power` are stored only when entered.

A material entry must have a `name`. Optional fields are `kerf`, `diameter`, `wire`, `depth`,
`step_down`, `cut_feed`, `plunge_feed`, `power`, `passes`, `pierce_delay`, and `post_delay`.
Lengths use the profile units. Feeds and delays are not scaled. `passes` is a whole number in
1..=1000. `power` is an object `{ "unit": "percent"|"w", "value": ... }`, never a raw 0..=1
fraction. Percent is 0..=100. Watts need a user-entered `max_power` on the head before they
can be mapped to the planner's internal 0..=1 fraction. Omitted fields stay unset; they are
not filled with defaults.

Unknown keys are rejected.

### Migrating `campanion-machine/1`

On load, a v1 file becomes a gantry. If the work area names both X and Y extents, `bed` is
`x_max - x_min` by `y_max - y_min` (and `z` the same way when both Z ends are present). An
incomplete work area does not invent a bed. Head `kind` is inferred from `process`. A v1
material `power` in 0..=1 is stored as percent (`0.8` becomes `{ "unit": "percent", "value": 80 }`).
Offsets, `safe_z`, and `max_power` stay unset. Saving writes `campanion-machine/2`.

## Jobs

A `campanion-job/1` file may name a profile:

```json
"machine": { "profile": "shop-laser", "head": "laser" }
```

or an explicit file:

```json
"machine": { "path": "/path/to/shop-laser.json", "head": "laser" }
```

`profile` is the file stem in the machines directory. `head` may be omitted; the first head is
then used. `profile` and `path` cannot both be set. `machine.model` is not valid here; it belongs
to a lathe job.

When a job names a profile that has a work area, `campanion run` (and the GUI generate/export
path) checks the planned toolpath against those limits and refuses an overtravel. A missing
profile is a refusal. The selected head's process and post can be used by the GUI to filter
properties and to suggest a post. Applying a material copies only the fields that entry
contains into the selected operation (and the tool size when `kerf`, `diameter`, or `wire` is
set).
