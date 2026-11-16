# User machine profiles

A machine profile is a `campanion-machine/1` JSON file that describes a machine the user owns:
its name, units, optional work area, optional controller and post, and one or more heads. Each
head has a process, an optional post, and an optional material library.

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

```json
{
  "format": "campanion-machine/1",
  "name": "Shop laser",
  "units": "mm",
  "work_area": { "x_min": 0, "x_max": 600, "y_min": 0, "y_max": 400, "z_min": 0, "z_max": 80 },
  "controller": "grbl",
  "post": "grbl_laser",
  "heads": [
    {
      "id": "laser",
      "name": "CO2",
      "process": "laser",
      "post": "grbl_laser",
      "materials": [
        {
          "name": "3 mm acrylic",
          "kerf": 0.15,
          "cut_feed": 300,
          "power": 0.8,
          "depth": 3
        }
      ]
    }
  ]
}
```

`format`, `name`, `units`, and `heads` are required. `units` is `mm` or `inch`; lengths in the
file are in those units and are stored internally as millimetres. `work_area`, `controller`, and
`post` may be omitted. An omitted work area means no travel check is run. A work area must
include at least one of `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`. Only the limits
that are present are checked. A value equal to a limit passes; a value outside it is a
code-generation error.

`heads` must contain at least one head. Each head has a unique `id` and a `process` of
`milling`, `routing`, `turning`, `laser`, `plasma`, `waterjet`, `oxyfuel`, `wire_edm`, `sla`,
or `fdm`. `name`, `post`, and `materials` may be omitted. A head post, when set, overrides the
machine-level post.

A material entry must have a `name`. Optional fields are `kerf`, `diameter`, `wire`, `depth`,
`step_down`, `cut_feed`, `plunge_feed`, `power`, `pierce_delay`, and `post_delay`. Lengths use
the profile units. Feeds, power, and delays are not scaled. Omitted fields stay unset; they are
not filled with defaults.

Unknown keys are rejected.

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
