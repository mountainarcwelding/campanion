# Resin output: Anycubic Photon Mono M7 Max

**Status: not printer-tested.** Nothing written by `campanion resin` or `campanion supports` has
been printed or physically tested. The format was derived from one file, so some fields are copied
rather than understood. CAMpanion is primarily command-line driven; the GUI is still in development. The resin
Orient tab can translate a part (typed X/Y/Z with snap, left-drag in the viewport,
Center on plate, Drop to plate). The Supports tab can detect and colour islands, edit
support points in the shared 3D viewport (orbit, pan, zoom; click a surface to add a
tip in any view), and load a calibration profile. Generate supports runs in the
background with a progress bar and cancel. Vertical self-support rays use an XY
hit grid so a many-triangle part finishes in a fraction of a second rather than minutes. Mouse bindings follow SolidCraft's documented default: right-drag
orbits, middle-drag pans, the wheel zooms at the pointer. The build volume is drawn
only from the selected machine profile. Optimize stays disabled until that profile is
complete. The Slice tab runs the same slicer as this command, and Export writes a
`.pm7m` (supports and raft included when they were generated). Print settings come from
a user-chosen sample `.pm7m`, the same file `--params-from` uses; missing settings are
refused. Image orientation is unverified until an asymmetric test part is printed.

## Commands

```
campanion supports MODEL.stl -o OUT.stl [--support-params FILE.json]
    [--raft-chamfer-angle DEG] [--raft-thickness MM] [--raft-margin MM]
    [--raft-corner-relief MM] [--overhang-angle DEG] [--support-spacing MM]
    [--tip-diameter MM] [--tip-length MM] [--pillar-diameter MM]
    [--foot-diameter MM] [--foot-height MM] [--part-lift MM]
    [--points FILE] [--points-only] [--preview FILE.png] [--points-out FILE]
    [--layer-height MM] [--detect-cell MM] [--island-ignore-area MM2]
    [--lowest-band-height MM] [--lowest-band-spacing MM]
    [--calibration FILE.json] [--optimize]
    [--zones FILE.json --zone-spacing MM] [--presets FILE.json] [--preset-mix ROLE=NAME,...]
    [--tip-penetration MM] [--brace-diameter MM --brace-span MM --brace-pitch MM]
campanion resin MODEL.stl -o OUT.pm7m [--params-from SAMPLE.pm7m]
    [--layer-height MM] [--bottom-layers N]
    [--bottom-exposure S] [--bottom-lift-height MM] [--bottom-lift-speed V]
    [--exposure S] [--lift-height MM] [--lift-speed V]
    [--resin-name NAME] [--keep-xy]
    [--supports ...support flags or --support-params FILE.json...]
    [--supports-out FILE.stl] [--preview FILE.png] [--points FILE] [--points-only]
    [--detect-cell MM] [--island-ignore-area MM2]
    [--lowest-band-height MM] [--lowest-band-spacing MM]
    [--calibration FILE.json] [--optimize] [--points-out FILE]
campanion resin-info FILE.pm7m
```

- Every exposure, lift and layer value is required, from a flag or from `--params-from`. A flag
  wins over the sample. `--params-from` reads layer height from layer 0, treats the leading layers
  that share layer 0's exposure and lift as bottom layers, and takes the normal values from the last
  layer. Lift speed is stored as given; the sample does not name its unit.
- The model is moved so its lowest point is on the plate and its XY bounds are centred on the plate
  (`--keep-xy` leaves XY alone, read as millimetres from the plate centre). A model that does not fit
  the 297.5 x 164 x 300 mm volume is refused, never clipped.
- Layer `i` is sampled at `(i + 0.5) * layer height`. Fill is non-zero winding, so a body overlapping
  its own support shells merges. A pixel is fully on (15) or off (0).
- `resin-info` decodes every layer image and checks the layer counts and markers.
- The finished file is read back before it is written.
- `campanion supports` (and `campanion resin --supports`) adds a chamfered raft and vertical
  pillar supports. Every support length and angle is required, from a flag or from
  `--support-params FILE.json`; a flag wins over the file. Units are millimetres. Nothing is
  assumed.
- The raft sits on z = 0. Its top outline (the part side) is the convex hull of the part and the
  support contacts, grown by `--raft-margin`. The bottom outline (the bed side) is inset by
  `thickness / tan(--raft-chamfer-angle)`. The angle is measured from the plate (horizontal), so
  the raft is narrower on the bed and forms an undercut a scraper can get under. An inset that
  would collapse the bottom face is refused. `--raft-corner-relief` cuts each top corner back
  along both edges; `0` keeps sharp corners.
- `--part-lift` is the gap from the top of the raft to the lowest point of the part. It is not
  the same as `--lift-height` on `campanion resin`, which is the printer's peel move.
- A downward-facing triangle gets a support when its angle from vertical is at or above
  `--overhang-angle`. `0` includes every downward face; `90` includes only faces parallel to the
  plate. Pillars go straight down: a foot of `--foot-diameter` and `--foot-height`, a shaft of
  `--pillar-diameter`, and a tip of `--tip-diameter` and `--tip-length`. Spacing is
  `--support-spacing`. `--points FILE` is lines of `x y`, `x y z`, or `x y z preset` (comments
  start with `#`); `--points-only` skips auto placement. `--points-out` writes the placed points
  back in that format.
- `--layer-height` on `campanion supports` enables first-appearance island detection, including
  mid-air islands. A region on a layer that does not overlap the previous layer is an island and
  gets at least one support (plus fill at `--support-spacing`, or at `--lowest-band-spacing` when
  the island is in the lowest band). `--detect-cell` is the occupancy cell size; if it is omitted,
  `--support-spacing` is used. `--island-ignore-area` is optional and is not assumed. On
  `campanion resin --supports`, island detection uses that command's `--layer-height`.
- `--lowest-band-height` and `--lowest-band-spacing` must be given together. They tighten spacing
  near the part's first contact. Omitting the pair leaves uniform spacing.
- `--optimize` runs the holding-force optimizer: feature classification (islands, lowest band,
  area jumps, overhangs, cups/suction, thin features) and the check
  `sum(tip capacity) >= safety factor × (peel + suction + weight)`. Every constant comes from
  `--calibration FILE.json`. There are no shipped defaults. A missing field is an error. Uniform
  and manual modes work without a calibration profile. Fitting measurements to constants is not
  implemented; enter fitted constants in the file. The geometry of a calibration test print is
  not defined in this release.
- The supported STL is a multi-body mesh: the part, the raft, and each pillar are separately
  watertight. The slicer fills with non-zero winding, so bodies that touch merge in the layer
  images. `--preview` writes a shaded 3/4 view from a low side angle (the raft undercut is meant to be visible).

### Painted zones, presets, tip penetration and cross-bracing

None of these has a default. Each is off until you give it, and a feature that is on needs every
value it uses.

- `--zones FILE.json` marks part triangles as support or no-support faces. The file is written by
  the GUI (Supports tab, Paint support / Paint no-support) or by hand:
  `{"schema":"campanion-resin-zones/1","triangle_count":N,"support":[[start,end],...],"block":[[start,end],...]}`.
  Ranges are half-open triangle indices into the input STL. A different triangle count is refused.
  Down-facing support faces are sampled at `--zone-spacing` even below `--overhang-angle`; faces
  that face up or sideways, or that have part below them, cannot take a vertical support and are
  counted in a warning. Automatic supports whose contact face is a no-support face are removed.
  Supports you list in `--points` are never removed. Painted zones work with `--points-only`.
- `--presets FILE.json` is a list of named tip presets you saved:
  `{"schema":"campanion-resin-presets/1","presets":[{"name":"fine","tip_diameter_mm":...,"tip_length_mm":...,"pillar_diameter_mm":...,"foot_diameter_mm":...,"foot_height_mm":...,"tip_penetration_mm":...}]}`
  (`tip_penetration_mm` is optional). Names may not contain spaces, commas or `=`. A `--points`
  line may name a preset in its fourth column. `--preset-mix` picks a preset for each kind of
  support: `overhang`, `island`, `lowest-band`, `painted`, `manual`. A role left out uses the tip
  and pillar flags. The mix cannot be combined with `--optimize`, which picks from the calibration
  tips instead. The GUI keeps its presets in `resin-tip-presets.json` in the user config folder.
- `--tip-penetration` sinks each tip that far into the part along the tip axis, so the contact is
  not tangent. A preset's own `tip_penetration_mm` wins for its supports.
- `--brace-diameter`, `--brace-span` and `--brace-pitch` go together. Pillars are joined into a
  minimum spanning tree of neighbours no farther apart than the span. Each link gets zig-zag
  struts of the given diameter at the given vertical pitch, between the top of the feet and the
  base of the tip cones. A strut that would come within one strut diameter of the part is left out
  and counted. Struts are closed 8-sided prisms; the slicer's non-zero fill merges them with the
  pillars.

### Islands in the slice

`campanion resin` reports regions of a layer that touch no lit pixel on the layer below (the first
layer is skipped). These would print in mid-air. The GUI Slice tab colours and outlines them on the
layer image and can step to the previous or next layer with one. The check is pixel overlap only;
it does not judge whether a thin overlap is strong enough.

`--support-params` is a JSON object with these keys (unknown keys are refused):

```
raft_chamfer_angle_deg, raft_thickness_mm, raft_margin_mm, raft_corner_relief_mm,
overhang_angle_deg, support_spacing_mm, tip_diameter_mm, tip_length_mm,
pillar_diameter_mm, foot_diameter_mm, foot_height_mm, part_lift_mm
```

## What was derived from the sample

Machine: `Anycubic Photon Mono M7 Max` (exact case), screen 6480 x 3600, bed 297.5 x 164 x 300 mm.

`.pm7m` is a zip with, in this order: `anycubic_photon_resins.pwsp`, `software_info.conf`,
`print_info.json`, `preview_images/preview_1.png` (336x252), `preview_images/preview_0.png` (224x168),
`scene.slice`, `layers_controller.conf`, `layer_images/layer_N.pw0Img`.

`.pw0Img`: big-endian 16-bit words, top 4 bits gray (0 to 15), low 12 bits run length (1 to 4095),
row-major, runs continue across rows, long runs split greedily at 4095. The runs sum to 6480 x 3600.

`scene.slice`: 13-byte magic `ANYCUBIC-PWSZ`; u32 3 at offset 80; u32 1 at 84; f32 1.0 at 96;
the layer count as u32 at 100; `<---` at 388; the count again at 392; then one 64-byte record per
layer (f32 top height at +0, u32 1 at +24, zeros elsewhere); `--->` at the end. The other
bytes are zero in the sample.

`layers_controller.conf`: tab-indented JSON with `count` and `paras`, each layer carrying
`exposure_time`, `layer_index`, `layer_minheight`, `layer_thickness`, `zup_height`, `zup_speed`.

## Checks against the sample

With the sample's path in `CAMPANION_PM7M_SAMPLE`, `cargo test -p campanion-resin --test sample`:

- all 592 layer images decode and re-encode to identical bytes;
- `scene.slice` is rebuilt byte for byte from the settings read out of the sample;
- the rebuilt layer list matches (min heights agree within 1e-9; see below), `machine_type` and the
  firmware block of the pwsp are identical, and the thumbnails have the sample's sizes.

The sample is not committed (it is a user file). Without the variable, those tests do nothing.

## Assumptions and gaps

- **Image orientation is unverified.** +X is image right and +Y is image up. Whether the printer shows
  the image mirrored or flipped needs a printed asymmetric test part.
- **Copied constants.** `machine_extern` in the pwsp (including the sample's `alias` of
  `Anycubic Photon Mono M7 Pro`), the resin's density, price, depth-penetration curve and
  `multi_state_paras` entries other than `bott_0` and `normal_0`, and the unknown `scene.slice` words
  above are the sample's values. Which of these the printer actually reads is not known.
- **Resin metadata.** The pwsp resin entry carries the user's exposures, lift, bottom layers and layer
  height; its retract speed (`zdown_speed`) is set equal to the lift speed, as in the sample.
- **`layer_minheight`** is `i * layer_height`. The sample's values differ from that in the last digit
  for some layers (a rounding difference in whatever wrote it); the printer should not care.
- **`software_info.conf`** says `CAMpanion`, not the slicer name in the sample.
- **`print_info.json`**: `print_time` is the total exposure time only, a lower bound that leaves out lift
  and settle time; `cost` is 0; `volume` is the solid volume in mL.
- Previews are a simple shaded top view.
- A calibration profile is a JSON object with schema `campanion-resin-calibration/1`. Required
  fields: `schema`, `name`, `tip_palette` (each entry needs `name`, `tip_diameter_mm`,
  `tip_length_mm`, `pillar_diameter_mm`, `foot_diameter_mm`, `foot_height_mm`, `tip_capacity_n`,
  `contact_area_mm2`), `peel_coefficient_n_per_mm2`, `suction_coefficient_n_per_mm2`,
  `safety_factor`, `resin_density_g_per_cm3`. Standard gravity (9.80665 m/s²) is a physical
  constant, not a calibration field; a `gravity_m_s2` key in a file is ignored. Optional: `island_ignore_area_mm2`,
  `thin_width_mm`, `area_jump_ratio`, `lowest_band_height_mm` with `lowest_band_spacing_mm`, and
  `measurements` (recorded observations; unused until fitting is implemented). No example numeric
  values are shipped.
- Not done: anti-aliasing, painted no-support zones, branching tree supports, scaffolding,
  fitting measurements to calibration constants, a calibration test-print geometry, STL units
  other than millimetres, other Anycubic models, other resin printers, wait-before-lift and
  wait-after-lift times, per-layer overrides.
- No UVtools read-back was done (it was not installed on the build machine); the read-back is
  CAMpanion's own reader.
