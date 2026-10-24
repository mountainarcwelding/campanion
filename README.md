# CAMpanion

An open, machine-neutral CAM (toolpath) engine in Rust, built as a companion to
[SolidCraft](https://github.com/tangency-labs/solidcraft) with the aim of merging into it later.

Status: early. Start with 2D path processing, then work up to multi-axis machines
(the toolpath format is designed for up to 13 axes from the start).

- Plan and milestones: [docs/plan.md](docs/plan.md)
- Rules for contributors and agents: [AGENTS.md](AGENTS.md)

CAMpanion (repo and crates: `campanion`). This project is not affiliated with SolidCraft or
Tangency Labs.

## Quick start

Build and run the sample job, which turns a DXF drawing into G-code
(`examples/sample_svg.job.json` does the same from an SVG, and `examples/step.job.json` from a
STEP part through SolidCraft's `solidcraft-cli`; `--up-axis` defaults to `z`. `auto` is opt-in
and trusts the exporter header and turns Y-up exports such as SolidWorks onto Z. SolidCraft
itself now asks which axis is up rather than guessing, so CAMpanion does not guess by default;
see docs/job-file.md):

```sh
cargo run -p campanion-cli -- run examples/sample.job.json -o out.ngc --ir out.json
```

- `examples/sample.job.json` describes the source drawing (`examples/sample.dxf`), tools,
  heights and operations (inside/outside profiles by layer).
- `-o out.ngc` writes reference G-code; `--ir out.json` writes the machine-neutral toolpath.
  Pass `--post` to write a controller dialect instead of the test sink. See [Posts](#posts).
- The toolpath is verified before anything is written; errors stop the run.
- `examples/sample_delays.job.json` shows pierce height and pierce/post delays.
- `examples/indexed.job.json` finishes `examples/dome.stl` at two rotary indexes. See [Indexed 3+1](docs/job-file.md#indexed-31) in the job-file docs.
- `examples/wrap.job.json` wraps `examples/wrap.svg` onto a 40 mm cylinder about A. Drawing X is the distance along the axis and drawing Y is the arc length around the surface. See [Wrapping](docs/job-file.md#wrapping).
- `examples/rotary.job.json` finishes `examples/rotary.stl` with a 3 mm ball, zigzag. The mesh is a vase revolved about X. See [Rotary finishing](docs/job-file.md#rotary-finishing).
- `examples/shaft.lathe.json` faces, roughs and finishes the stepped shaft in `examples/shaft.dxf`. See [Lathe jobs](docs/job-file.md#lathe-jobs).
- `examples/millturn.lathe.json` indexes C for a front live drill and an OD live slot on that shaft (no Y, no interpolation).

```sh
cargo run -p campanion-cli -- lathe examples/shaft.lathe.json -o out.ngc --ir out.json --post linuxcnc_lathe
```

`campanion lathe` loads a `campanion-lathe/1` file. Without `--post`, `-o` is the RS274/NGC test sink and ZX arcs are `G18`. `--post linuxcnc_lathe` writes diameter mode (`G7`), the ZX plane (`G18`) and constant surface speed (`G96`). Posted text is read back at the job chord tolerance. A mismatch exits 1 and writes nothing. Verify errors are printed and the status is 1. A face, rough or finish gouge is printed and the status is 1.

A `.step` or `.stp` lathe source is sectioned through the spindle by `solidcraft-cli` (`--up-axis`, default `z`); `examples/shaft_fillet.lathe.json` reads `examples/shaft_fillet.step` that way.

## Usage

`campanion verify FILE` checks an IR file and prints a report. The status is 1 when the report
has errors.

Check cutting moves against an STL mesh. The tool is a radius in millimetres (`ball:3`, `flat:3`,
or `bull:3,1`, where the number after the comma is the bull corner radius):

```sh
campanion verify out.json --mesh examples/dome.stl --tool ball:3
```

Tolerance is 0.01 mm, the job file's default chord tolerance. Samples are `min(0.25, tolerance*10)`
mm apart. A gouge is printed like any other verify error and names the operation, the position and
the depth. `campanion run` on an STL job does the same check with each operation's cutter and the
job's chord tolerance. XY arcs are sampled with Z interpolated from start to end. An arc in another
plane is an error. Surface operations emit lines. An indexed mesh operation is checked in that
indexed view: the mesh is turned by the same angle, and the path is mapped back to machine XY
before it is sampled. A rotary finish is checked on the mesh turned to each cutting angle.

## Posts

Without `--post`, `campanion run -o` writes the test-sink NGC. With `--post`, `-o` is G-code from
that post. The G-code is read back and compared to the toolpath at the job chord tolerance
(0.01 mm when the job omits `tolerance.chord`). A mismatch exits 1 and writes nothing.

`campanion posts` lists the built-in posts:

| id | name | Rotary | description |
| --- | --- | --- | --- |
| `linuxcnc` | LinuxCNC | A, inverse time | Millimetre RS274 dialect for LinuxCNC. Absolute moves, cutter compensation cancelled, IJK arcs and helices. A axis with inverse-time feed (G93, cancelled by G94) when the angle changes. |
| `grbl` | Grbl | - | Millimetre dialect for Grbl. Absolute moves, IJK arcs and helices, no line numbers, tool changes as comments. No rotary axis. |
| `mach3` | Mach3 | A, degrees per minute | Millimetre dialect for Mach3. Incremental IJK arcs and helices, dwell P in seconds, no line numbers. A axis with degrees-per-minute feed when the angle changes. |
| `fanuc` | Fanuc | A, inverse time | Millimetre Fanuc mill dialect. Program O0001, sequence numbers, R arcs, helices as lines, dwell P in milliseconds. A axis with inverse-time feed (G93, cancelled by G94) when the angle changes. |
| `haas` | Haas | A, inverse time | Millimetre Haas mill dialect. Program O00001, sequence numbers, IJK arcs and helices, dwell P in milliseconds. A axis with inverse-time feed (G93, cancelled by G94) when the angle changes. |
| `plasma` | LinuxCNC plasma | - | Millimetre plasma. Torch on M3 S1 / off M5, pierce dwell from the path, optional THC codes, helices as lines. No rotary axis. |
| `grbl_laser` | Grbl laser | - | Grbl laser mode. Dynamic power M4 S{power} (0..1 scaled to S0..S1000), Z words and Z-only rapids omitted. No rotary axis. |
| `linuxcnc_lathe` | LinuxCNC lathe | - | Millimetre LinuxCNC lathe. Diameter mode (G7), ZX plane (G18), constant surface speed (G96) and direct rpm (G97). No rotary axis. |
| `fanuc_lathe` | Fanuc lathe | - | Millimetre Fanuc lathe. Diameter mode by default, ZX plane (G18), feed per minute (G98), G50 spindle cap then G96. Home is G28 U0 W0. No rotary axis. |
| `haas_lathe` | Haas lathe | - | Millimetre Haas lathe. Program O00001, safety block G21 G18 G40 G80, feed per minute (G98), G53 home. No rotary axis. |
| `grbl_lathe` | grblHAL lathe | - | Millimetre grblHAL lathe. Diameter mode (G7), ZX plane (G18), feed per minute (G94). Tool change is T and M6; a manual tool change is expected. No rotary axis. |

```sh
campanion run examples/sample.job.json -o out.ngc --post linuxcnc
campanion run examples/sample.job.json -o out.ngc --ir out.json --post path/to/my-post.json
campanion post out.json --post grbl -o out.nc
campanion post out.json --post grbl
campanion posts
```

`--post FILE.json` loads that file (the value has to end in `.json`). Any other value is a built-in
id. An unknown id exits 1 and lists the ids above. `campanion post` prints the G-code when `-o` is
omitted, and compares the read-back at the default job chord tolerance, 0.01 mm.

A custom post is one JSON object. Unknown fields are rejected, and so are date or time
placeholders, so the same program and post always produce the same bytes. Start from
`crates/post/posts/linuxcnc.json` or `crates/post/posts/grbl.json`.

- `id`, `name`, `description` — short id, display name, and what the dialect is for.
- `units` — `mm`.
- `line_numbers` — `enabled`, `start`, `step`, and a one-letter `prefix`.
- `decimals` — digits for `linear`, `feed`, and `dwell` moves (0 through 8).
- `trailing_zeros` — keep trailing zeros when true.
- `decimal_point` — when true, dimensional words always include a decimal point (`52.`, `0.`). Fanuc and Haas read a missing point as least-input increments (`X52` is 0.052 mm). Feed, spindle, dwell, line numbers, and tool and program numbers are unchanged.
- `header`, `footer` — lines around the toolpaths. Placeholders: `{program_name}` and `{date-free!}` (expands to nothing).
- `modal_motion` — when true, a repeated `G0` or `G1` is left off the next line.
- `arcs` — `mode` is `ijk` (centre offsets `I` `J` `K`), `r` (signed radius), or `lines` (chords within the chord tolerance). `max_sweep_deg` splits a longer arc (greater than 0, at most 360). `helix` writes a helix as arcs when true, and as chords when false.
- `planes` — when true, the post can change plane. When false, an arc has to stay in the header plane.
- `dwell` — `word` (one letter) and `unit` (`s` or `ms`).
- `process` — map from a process name (`spindle`, `torch`, `laser`, `extruder`, `knife`, `other`) to `{ "on": [lines], "off": [lines] }`. Placeholders: `{speed}`, `{power}`, `{program_name}`, `{date-free!}`.
- `tool_change` — lines for a tool change. Placeholder: `{tool}`.
- `coolant` — `flood`, `mist`, and `off`, each a list of lines.
- `comments` — `style` is `paren`, `semicolon`, or `none`.
- `program_end` — at least one line, such as `M2` or `M30`.
- `max_line_length` — optional limit from 1 to 4096 characters.
- `axis_letters` — optional three distinct letters. The default is `XYZ`.
- `rotary` — optional. `axes` is `a`, `b`, or both. `decimals` is the degree word. `feed_mode` is `inverse_time`, `degrees_per_minute`, or `surface`. Inverse time also needs `inverse_time_on` and `inverse_time_off` (`G93` and `G94` on LinuxCNC, Fanuc and Haas). Mach3 uses `degrees_per_minute`. The feed length rule is in `crates/post/README.md`.

Check the posted program on the machine before cutting: dry run, then a single block. Read-back
only checks that the motion matches the toolpath.

## Preview

`campanion-view` shows a toolpath IR file (the `--ir` output of `campanion run`):

```sh
cargo run -p campanion-cli -- run examples/curves.job.json -o out.ngc --ir out.json
cargo run -p campanion-view -- out.json      # or: campanion view out.json
```

Cutting moves are solid, rapids dashed (toggle), pierces are dots, and numbers give the cut order.
Wheel zooms at the cursor, drag pans, F or double-click fits. Space plays or pauses; the sliders set
the playback speed and position. A rotary program draws its A or B axis across the toolpath and
lists the index angles (`Index: 0°, 90°`) or the continuous angle range. `--angle` colours cuts by
that angle instead of height (A toggles it); a program with no rotary axis ignores the option.
A turning program is an XZ half-section: radius upward, Z to the right. The view fills the removed
material, outlines the stock in grey, and fills the finished section in steel blue, then draws the
spindle centre line and the toolpath. It stays in that flat view, so the isometric control and the
rotary legend are hidden. There is no screenshot of the half-section; the window is the preview.
Labels need a system font (DejaVu, Liberation or Noto are found
automatically; otherwise set `CAMPANION_VIEW_FONT` to a .ttf file).

## Roadmap and contributing

See [docs/plan.md](docs/plan.md) for the roadmap and [CONTRIBUTING.md](CONTRIBUTING.md) for how to help.

## Licence

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option. Unless you explicitly state otherwise, any
contribution intentionally submitted for inclusion in this work, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
