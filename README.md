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
STEP part through SolidCraft's `solidcraft-cli`; see docs/job-file.md):

```sh
cargo run -p campanion-cli -- run examples/sample.job.json -o out.ngc --ir out.json
```

- `examples/sample.job.json` describes the source drawing (`examples/sample.dxf`), tools,
  heights and operations (inside/outside profiles by layer).
- `-o out.ngc` writes reference G-code; `--ir out.json` writes the machine-neutral toolpath.
- The toolpath is verified before anything is written; errors stop the run.
- `examples/sample_delays.job.json` shows pierce height and pierce/post delays.

## Preview

`campanion-view` shows a toolpath IR file (the `--ir` output of `campanion run`):

```sh
cargo run -p campanion-cli -- run examples/curves.job.json -o out.ngc --ir out.json
cargo run -p campanion-view -- out.json      # or: campanion view out.json
```

Cutting moves are solid, rapids dashed (toggle), pierces are dots, and numbers give the cut order.
Wheel zooms at the cursor, drag pans, F or double-click fits. Space plays or pauses; the sliders set
the playback speed and position. Labels need a system font (DejaVu, Liberation or Noto are found
automatically; otherwise set `CAMPANION_VIEW_FONT` to a .ttf file).

## Roadmap and contributing

See [docs/plan.md](docs/plan.md) for the roadmap and [CONTRIBUTING.md](CONTRIBUTING.md) for how to help.

## Licence

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option. Unless you explicitly state otherwise, any
contribution intentionally submitted for inclusion in this work, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
