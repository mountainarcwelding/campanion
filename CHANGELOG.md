# Changelog

## [Unreleased]

### Added

- A built-in `mach4` three-axis mill/router post writes metric output with incremental IJK (`G91.1`) and `M9`/`M30` at the end. The opt-in `dwell.decimal_seconds` setting makes read-back treat an integer `G4 P` as milliseconds and a decimal `G4 P` as seconds, matching Mach4, and the writer always emits a decimal P so Mach4 reads seconds. Existing posts keep their dwell behavior and golden output. This output has not been proven on a machine.
- Pocket operations may set `start_depth`, a length downward from the original stock surface. Depth passes start below that Z and still end at `depth`. Omitted or 0 leaves existing output unchanged. Other operation kinds reject the field. This output has not been proven on a machine.
- Kerf-tool (laser or torch) operations in a `campanion-job/1` file must set `power`, an IR fraction in `0..=1`. `grbl_laser` scales that onto `S0`..`S1000`. A vector program without power is refused; it is never posted at full scale. A laser raster still takes power from the image. This output has not been proven on a machine.

## 0.12.0 — 2026-11-12

Plasma improvements: optional G41/G42 from IR controller compensation, job-file lead-ins and lead-outs with the small-hole rule, automatic contour side and climb or conventional direction, and a narrower-than-kerf refusal plus a warning when a rapid crosses a freed slug (#163–#166). This output has not been proven on a machine.

### Added

- Posts may set `cutter_comp` (`left`, `right`, `off`). A path whose IR compensation is `controller_left` or `controller_right` writes those words before the first feed and cancels at the end of the path. `plasma`, `linuxcnc`, `linuxcnc_router` and `mach3` write `G41`/`G42`/`G40`; `fanuc` and `haas` write `G41 D{tool}`/`G42 D{tool}`/`G40`. Posts that omit the block, including `grbl`, still refuse controller compensation. Computed programs are unchanged. Read-back and mill audit accept `G41`/`G42`, and audit requires cancellation before the program ends. This output has not been proven on a machine.
- `campanion-job/1` profile operations may set `lead_in` and `lead_out` (`none`, `line`, `perpendicular`, or `arc`). A kerf tool that omits them gets an arc of one kerf width. On a closed inside path, leads that would leave the waste are shrunk to the largest size not below a quarter of the kerf, or omitted. Leads are refused with a ramp or helix entry. Diameter tools still default to no leads, so existing mill programs are unchanged. This output has not been proven on a machine.
- Profile operations may set `direction` (`climb`, `conventional`, or `auto`; omitted is climb, and `auto` uses climb once the side is known) and `side: auto`, which treats a closed contour as inside when an odd number of other selected closed contours contain it. They may also set `compensation` (`computed`, `controller_left`, or `controller_right`). Controller compensation does not offset the path; posts with `cutter_comp` write G41/G42. Existing mill programs that omit these keys are unchanged. This output has not been proven on a machine.
- An inside kerf profile that cannot be offset is refused as a slot narrower than the kerf. A later rapid whose XY path crosses a fully cut inner profile with no tabs writes a warning to stderr. This output has not been proven on a machine.

## 0.11.0 — 2026-11-12

Laser raster: optional per-move power in the IR, posts with `power_scale` write each feed's `S` including `S0` on overscan, and `laser_raster` jobs engrave a PGM through the planner (#159–#161). This output has not been proven on a machine.

### Added

- Feed moves in the IR may carry an optional `power` (the same quantity as process-on power). Omitted, the field is absent and old IR loads unchanged. Laser raster uses one value per scan segment.
- Posts with `power_scale`, including `grbl_laser`, write each feed's `S` from that per-move power, including `S0` on a zero-power overscan. A program that uses per-move power refuses a feed that omits it, and a process-on without power writes `S0` instead of filling full scale. Vector programs without per-move power are unchanged, including the missing-power `S1000` comment. Read-back and `compare` check the `S` word against the IR. This output has not been proven on a machine.
- `laser_raster` operations engrave a PGM beside the job. Scan lines zigzag across the placed image at the stated step, with overscan at power 0 and an optional scanning offset applied along each pass so opposite passes agree on the image. Grey is mapped to power by a linear, gamma, or table map; a grey the map sends to zero is power 0. Images larger than 8192 pixels on a side, or more than 4,000,000 pixels, are refused. This output has not been proven on a machine.

## 0.10.1 — 2026-11-12

Post and IR follow-ups contributed by DarkBiscuits: G91.1 incremental arc centres in read-back and mill/five-axis audit, G53 machine retracts in mill audit, tool shapes preserved in IR for per-move mesh verification, and inch output for three-axis mill posts. This output has not been proven on a machine.

### Added

- Optional flat, ball, bull and V-bit cutter geometry in IR tools. Run jobs preserve their shapes and resolve chamfer geometry per tool ID, rejecting conflicting definitions before output (#146).
- `verify --mesh` uses each cutting move's tool geometry when `--tool` is omitted, including tool changes within a path. Missing shapes and unsupported V-bit verification fail explicitly; `--tool` remains a global override (#146).
- Three-axis mill and router posts may set `"units": "inch"`. The IR and physical tolerances stay in millimetres. The writer emits `G20` and converts linear words (`X` `Y` `Z`, `I` `J` `K`, `R`) and units-per-minute feeds; read-back and mill audit convert them back to millimetres. Template literals are not converted. Rotary, five-axis, and lathe inch configurations are refused. Built-in posts stay metric. This output has not been proven on a machine (#88).

### Fixed

- Post read-back, mill audit and five-axis audit accept `G91.1` as an explicit declaration of incremental IJK arc centres. `G90.1` is refused with an explanation that absolute arc centres are not supported. Custom post headers can now declare `G91.1` without failing their own read-back (#144).
- Mill modal audit accepts `G53` machine-coordinate retracts without adding a part-frame motion or changing its position, motion mode or plane, matching read-back (#145).

## 0.10.0 — 2026-11-09

Router gaps from the processes plan: cutter sense on diameter tools, onion-skin depth passes, a deterministic sheet packer with nesting checks, and a LinuxCNC router dialect with a cited probe cycle and vacuum/dust auxiliaries. This output has not been proven on a machine.

### Added

- A diameter tool may set `cutter_sense` (`upcut`, `downcut`, or `compression`) and, with compression, `upcut_length` (millimetres at the tip). Omitted, compensation stays the flat diameter offset. Sense is stored on the job and the IR for later checks; it does not change the 2D offset. Posted G-code is unchanged when the field is omitted. This output has not been proven on a machine.
- A profile, pocket, face, engrave, drill, wrap or live mill may set `onion_skin`, a positive length strictly less than `depth`. Earlier depth passes stop that far short of the floor. The last pass cuts the skin with the same entry as the other passes. Omitting the field leaves the depth plan unchanged. `campanion run` checks that no earlier pass enters the skin and that the last pass reaches the floor. This output has not been proven on a machine.
- `linuxcnc_router` is a millimetre LinuxCNC gantry post: the same RS274 motion as `linuxcnc`, with no rotary axis. A Probe event with a contact pose and a feed is `G38.2` (LinuxCNC straight probe; <https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38>). `vacuum` and `dust` are immediate digital outputs `M64`/`M65` (<https://linuxcnc.org/docs/stable/html/gcode/m-code.html#mcode:m62-m65>); pin numbers default to 0 and 1. Any other Other name is refused. Existing posts still fall back to `process.other`. This output has not been proven on a machine.
- Mill jobs may set `sheet` (width and height in the XY plane, origin at (0, 0)) and `nest` (gap, margin, clamp keep-outs and parts). When both are set, `campanion run` packs each part's axis-aligned bounds onto the sheet before building toolpaths. Parts are placed in input order. The first feasible pose wins: ascending Y, then ascending X, then rotation index (0°, then 90° when the step is 90°). The same parts always pack the same way. The packer is not a yield optimiser. It packs rectangles; contour packing can follow. A part lies inside the sheet inset by the margin, clears other parts by the gap, and clears each keep-out by the tool radius plus the gap. A part that does not fit, or that fails a check, exits 1 and nothing is written. Placements are printed on stderr. `--nest-out FILE.json` writes `campanion-nest/1` after the program is accepted. The IR stays ordinary toolpaths in the sheet frame. This output has not been proven on a machine.

## 0.9.3 — 2026-11-03

Code-generation hardening and in-process stock. A modal replay checker and property tests now cover every built-in post (mill, plasma, laser, Grbl, lathe and mill-turn, five-axis with and without TCP, and 3+1 rotary). The audit found five bugs, fixed below: three in posted output and two in the `compare` checker. Mill jobs may model the in-process stock as a Z-dexel and check rapids and the tool assembly against it, with removal rate and engagement per move. This output has not been proven on a machine.

### Fixed

- A continuous move inside the singular cone was refused on `linuxcnc_5ax` and `haas_umc` (`toolpath {i} move {j}: rotary-only move needs a feed`). Subdivision compared the joint midpoint with the requested tool axis, which that cone does not achieve, and split the move until a block was too small to feed. The midpoint is now compared with the forward axes of the solved joints. TCP posts already measured the forward axis.
- A positive feed that rounded to zero at the post's F precision was written as `F0` or `F0.0000`. The move is now rejected. Every built-in post that writes an `F` word was affected (mill, lathe, and five-axis).
- A full circle after an arc of the same sense was treated as the rest of that arc, so a correct program failed `compare`. The full circle is kept as its own move.
- A short arc that ends near the previous arc was treated as another block of that arc, so a correct program failed `compare`. It stays its own move unless it shares that arc's centre.
- A ZX or YZ arc on `grbl_laser` was written with the Z word left out, so the curve was not the programmed arc. That arc is now rejected. XY moves still omit Z.

### Added

- Mill jobs may set `stock` (a box in the part frame) and `stock_model` (`cell`, `enabled`). When the model is on, `campanion run` replays the toolpaths through a Z-dexel of the remaining stock. A rapid that enters the stock, or a shank, holder or head that meets it, fails the run and nothing is written. After an STL program the run prints the leftover and the model-gouge count. A sphere and a finite cylinder are exact on each dexel; a straight cut with a constant axis also subtracts the exact swept cylinder. A rotating axis, an arc and the ball nose are sampled. Steep walls are only as sharp as the cell. This output has not been proven on a machine.
- `audit_five` replays posted five-axis G-code and reports the work frame, TCP, tool length, inverse time, rotary words, and `G53` retracts. Property tests check random programs on `linuxcnc_5ax`, `linuxcnc_5ax_tcp`, `haas_umc`, `haas_umc_tcpc`, and the 3+1 rotary block on `linuxcnc`, `haas`, `fanuc`, and `mach3`. Posted G-code is unchanged.
- `audit` replays posted mill G-code and reports the modal state at each motion. Property tests check random programs on the mill posts (linuxcnc, grbl, mach3, fanuc, haas, plasma, grbl_laser). Posted G-code is unchanged.
- When the stock model is on, each feed move records the removed volume (mm³), the material-removal rate at the IR feed (mm³/min; rapids are skipped; a thread has no mm/min feed), the radial engagement (degrees of cutter circumference in contact, from the dexels at the flute's lowest active level) and the axial depth of cut. The run prints each operation's peak removal rate and peak engagement on stderr. `--stock-report FILE.csv` writes one row per feed move. `--stock-out FILE.json` writes the final height map (`campanion-stock/1`: origin, cell, counts, top Z, and columns with more than one interval). `--stock-snapshots N` also stores a map at the start and after every Nth motion. `campanion view --stock FILE.json` shades that top in plan and draws it as a wireframe in iso; S toggles it, and the scrub picks the latest snapshot at or before the playhead. A dexel can sit about one cell off a wall. A slot engages 180 degrees and a side cut engages `acos(1 - 2s/D)`, within about one cell. A straight slot's removal rate is within about one cell of width. This output has not been proven on a machine.
- `audit_lathe` replays posted lathe and mill-turn G-code and reports diameter or radius X, constant surface speed and its cap, feed per minute or per revolution or inverse time, the turret tool and offset, nose compensation, threading, C-axis engage, the spindle brake, the live tool, face and cylinder interpolation, Y-axis planes, sub-spindle select, part transfer, the tailstock, the steady rest, the parts catcher, bar feed, and two-channel wait codes. Property tests check random programs on the lathe posts (linuxcnc_lathe, fanuc_lathe, haas_lathe, grbl_lathe, generic_lathe_2ch). Posted G-code is unchanged.

## 0.9.2 — 2026-11-02

Collision checks and machine simulation (five-axis stages 5G–5H). This completes the staged continuous five-axis plan (5A–5H). The checks cover the tool assembly against the finished part, fixtures and work-holding; in-process stock is not modelled. This output has not been proven on a machine.

### Added

- Mill jobs may set a tool `assembly` (stickout, flute, shank, holder stack, optional head) and job-level `fixtures` and `collision`. After the gouge check, `campanion run` tests the shank, holder and head — and the flute on a rapid — against the finished part mesh and the fixtures. The first hit names the operation, toolpath, move, body, obstacle and part-frame point. In-process stock is not modelled. The check samples the IR poses; a non-TCP control interpolates joints, so the real path can differ between samples. This output has not been proven on a machine.
- Lathe jobs may set the same `assembly` on a live tool, measured along the live-tool axis, plus optional `chuck` and `sub_chuck` cylinders and the mill `collision` object. After the gouge check, `campanion lathe` tests the shank, holder and head — and the flute on a rapid — against the finished profile revolved about the spindle. No body may touch the chuck, and a holder may not touch a clamped steady rest. An insert holder is tested against the chuck and that steady rest, and the insert against the chuck, in the half-section. The first hit names the operation, toolpath, move, body, obstacle and point. The tailstock quill is not a solid in this check. In-process stock is not modelled. This output has not been proven on a machine.
- The viewer sketches a five-axis machine from an optional kinematics `model`: boxes and cylinders on `base`, `table`, `head` or a rotary letter. `--machine` draws them (a model shows on open; M toggles). The scrub uses the posted joint values. Colliding solids turn red, and the status line shows the joints and the first hit. The solids are not machine CAD. The posted output has not been proven on a machine. `campanion run` stores a tool `assembly` and program `fixtures` when the job has them, and omits both when it does not. An STL fixture is a path and a hash.

## 0.9.1 — 2026-11-02

TCP posts and tool-axis strategies (five-axis stages 5E–5F). This release fixes wrong output from 0.9.0: repost any `haas_umc` or `linuxcnc_5ax` program from 0.9.0 that has an indexed (3+2) toolpath after a continuous toolpath with the same tool axis.

### Fixed

- An indexed toolpath after a continuous five-axis toolpath on the same tool axis now reindexes and turns the work frame back on. In 0.9.0 the frame stayed off, so on `haas_umc` (no G254) and `linuxcnc_5ax` (no machine transform) the cut was written as raw part-frame XYZ while the rotaries remained at the continuous pose.

### Added

- A surface operation may set `tool_axis` (`lead_tilt`, `toward_point`, `away_from_point`, `toward_curve`) on a ball tool when the job has kinematics. The ball centre stays on the 3-axis drop-cutter centre, so the tilted tip is that centre minus the radius times the tool axis and the surface contact is unchanged. The cutting moves are one continuous toolpath. The axis is clamped to `max_tilt_deg` from +Z, and the raster is densified to the program axis step. A rapid that changes a rotary, and a rapid that follows a reorientation, starts at a safe retract: the tip is lifted along the tool axis until the posted machine Z is at least 0. Holder and shank collisions are not checked until the collision stage. This output has not been proven on a machine.
- `five_axis.continuous.tcp` writes a continuous toolpath in TCP mode. XYZ is the part-frame tool tip. Rotary angles still come from `solve_path` (branch continuity, singular cone, and the same reachability and limit errors). Subdivision uses the same chord and axis tolerances: the control is assumed to interpolate the tip linearly and the rotary words linearly, and the tool axis is the forward kinematics of those angles. Inverse time is unchanged.
- Post `haas_umc_tcpc` commands Haas G234 TCPC (`G234 H{tool}` after the rotaries are at zero, `G49` then `G43 H{tool}` after the path). A continuous toolpath with no tool is refused, because `G234 H0` would run TCPC without a length offset. A rapid that changes a rotary is `G1` in `G93` at 5000 mm/min. This output has not been proven on a machine.
- Post `linuxcnc_5ax_tcp` programs the tool tip with rotary words and empty TCP on/off hooks. The machine must run TCP kinematics. The header says `(TCP kinematics required)`. This output has not been proven on a machine.

## 0.9.0 — 2026-11-02

First continuous five-axis release (posts without TCP). Check any program that has a G2/G3 shorter than the output resolution: it now posts as a line.

### Fixed

- An arc shorter than the output resolution, whose rounded ends share an angle about the rounded IJK centre, is posted as a line. It was posted as a G2/G3 that a controller cuts as a full circle.

### Added

- Posts `linuxcnc_5ax` and `haas_umc` write continuous five-axis output without TCP. `five_axis.continuous` selects inverse-time feed (`G93`/`G94`), chord and axis tolerances, and linear `G0`/`G1` blocks with XYZ plus rotary words. The post runs `solve_path` and `subdivide_path`. Arcs and helices are linearised. On `haas_umc` G255 is active and XYZ are G54 coordinates with pivots measured from G54, the same rule as `linuxcnc_5ax`. This output has not been proven on a machine.
- A toolpath may set `singular_cone_deg` (default 1°, range `(0, 10]`, omitted from JSON at the default). Inside that cone of the singular direction, `solve_path` holds the primary rotary at its previous angle and lets the secondary rotary take the tilt. The tool axis stays within the cone of the requested axis. A step that enters or leaves the cone changes the primary by at most `90° + 2 × singular_cone_deg`.
- `subdivide_path` inserts joint-space samples until each segment's midpoint stays within a chord tolerance and a tool-axis tolerance. More than 10000 inserted poses is an error that names the toolpath.
- A toolpath may set `continuous` (default false, omitted from JSON). On a continuous toolpath the verifier rejects a feed whose tool axis changes by more than the program setting `max_axis_step_deg` (default 5°, range `(0, 90]`). The error names the toolpath index, the move index and the angle.
- `solve_path` turns a list of tool poses into joint positions. It keeps the rotary branch, unwinds continuous rotaries so a joint does not jump by 360°, and reports the first pose that leaves a travel limit or that no solution can reach.
- Posts without `five_axis.continuous` reject a continuous toolpath with `post `{id}` has no continuous five-axis output`.

### Changed

- The Haas ST-15Y, ST-20Y and DS-30Y profiles cap live-tool rpm at 6000 instead of 4000. `machine.live_tool_upgrade` is still accepted and no longer changes anything, including on the DS-30Y.

## 0.8.7 — 2026-10-29

### Added

- Lathe `machine` block with cited limit profiles for the Haas ST-15Y, ST-20Y and DS-30Y (travel extents, Y travel, spindle, sync and live-tool rpm).
- Lathe `orient` and `sub_brake` operations. `haas_lathe` writes `M19 R` / `M119 R` for spindle orient (two decimal places) and `M114`/`M115` for the secondary spindle brake. M119 is not written while G14 is active (Haas NGC lathe operator's manual, 2025, §8.1.10, §8.1.37, §8.1.38, §7.1.8).

## 0.8.6 — 2026-10-29

### Added

- Lathe `steady_rest` operation (clamp/unclamp) with a band check; `haas_lathe` writes M146/M147.
- Lathe `bar_feed` operation at program end; `haas_lathe` homes and writes G105 alone on its line, with optional J/K overrides.

## 0.8.5 — 2026-10-29

Second safety patch. Check tabbed jobs: tab `width` is now the bridge left on the part.

### Fixed

- A lathe STEP section keeps a radial end face when a seam pulls it off Z, so `examples/shaft_fillet.lathe.json` runs with solidcraft-cli. It failed with "the two halves of the section differ".
- A ramp or helix entry stays at or below `angle_deg` on every posted descent. The first pass is sized from the retract height, so a ramp that starts above the stock is not the stock step over a length meant for a shorter drop (#91).
- Pocket rings stay at most one stepover apart, so the band between an outer wall and an island is cut when it is narrower than two stepovers, and split and leftover regions always get a pass (#90).
- Pocket area passes honour `entry`: a ramp or helix from the retract height, or an error when that entry does not fit, and a feed between rings at the same depth when the link stays in the pocket (#89).
- A spindle stays on through a tool's depth passes and operations, and retracts above the stock before it stops. Plasma and laser posts keep a per-cut cycle (#93).

### Changed

- Tab `width` is the bridge left on the part, measured along the part edge. On a straight edge the tool-centre gap is that width plus the tool diameter. Profile-document tab gaps are wider by the tool diameter than in 0.8.4.
- Rough toolpaths no longer depend on the platform maths library: candidates within 1e-9 relative are tied and broken by position, then index. Some rough start points and goldens changed.

### Added

- Run jobs accept `tabs` on a profile operation. Bridges match a profile document: a pass below the tab top feeds along the compensated contour, and a full-depth tab rapids at the retract height (#95).

## 0.8.4 — 2026-10-28

Safety patch. Do not run tabbed profile-document output, or multi-tool `campanion run` output, from v0.8.3 or earlier; regenerate it with 0.8.4.

### Added

- Lathe `part` option `catch` with a `parts_catcher` size check; `haas_lathe` writes M36 before the part-off plunge and M37 after the retract.
- Mill jobs accept optional `tools[].speed` and `operations[].speed` (rpm). The operation value wins. `campanion run` copies the resolved speed onto process-on so posts write `M3 S{speed}`.

### Fixed

- Profile-document tabs are crossed by feed moves along the compensated contour at `z + height` (lines, arcs and corners), never as a rapid or a chord (fixes #95). A full-depth tab feeds up to the retract height, rapids there, and feeds back down. Profiles rapid to the pierce point at the clearance height and down to the retract height before feeding to depth; the profile document accepts optional `heights`. The verifier rejects an XY rapid below the retract height.
- `campanion run` writes a tool-change event at the start of the first toolpath and whenever the tool id changes, so mill posts emit `T{tool} M6` (fixes #94). Consecutive toolpaths that share a tool get one change. A spindle operation with no speed fails `run --post` when that post's spindle-on template uses `{speed}`.
- Full circles post on every mill post at any coordinates: a closed arc whose end lies within 1e-9 of a radius of its start is one full turn in the toolpath, the writer, and read-back (fixes #92). `examples/step.job.json` now posts on every mill post.

## 0.8.3 — 2026-10-24

### Added

- Lathe `tailstock` operation (advance/retract) with a tailstock zone check; `haas_lathe` writes M21/M22. Hold force and travel stay machine settings.

### Fixed

- `haas_lathe` writes G99 before G32 thread passes and G98 after them. Haas requires feed per revolution for G32; earlier versions wrote the thread passes under G98.

### Changed

- Docs and error messages call the tilting milling head a "tilting B head", so it is not confused with the linear B axis of positioned Haas ST tailstocks and of DS secondary spindles.

## 0.8.2 — 2026-10-24

### Fixed

- Posting `examples/dome_rough.job.json` (and other programs with very short fitted arcs) no longer fails read-back with "point is off the arc". The read-back comparator ignored the travel of arcs shorter than the tolerance and used angle limits tighter than endpoint rounding.
- A post no longer writes an arc whose end rounds onto its start (a controller cuts that as a full circle). A short one is written as a feed line within the arc tolerance when an axis still moves, and omitted when the rounded end is already the current position. Otherwise posting fails with "arc end rounds onto its start". The read-back comparator now rejects an arc block with equal endpoints unless the program arc is a full circle.
- A split arc no longer emits a feed that repeats the current position, and read-back finishes the arc only after its last block. A short final piece is not left over as the next move.

## 0.8.1 — 2026-10-24

Safety patch. Do not run mill drill output from v0.1.0–v0.8.0, or surface output from v0.3.0–v0.8.0 whose job clearance is below the top of the STL model; regenerate it with 0.8.1.

### Fixed

- Mill `drill` no longer rapids to the hole bottom. Each hole is now a rapid to clearance and to the R plane (job retract height), a plunge feed to depth (pecking when `step_down` is less than `depth`), an optional `dwell`, and a rapid out. Every release from v0.1.0 to v0.8.0 rapided to depth; do not run mill drill output from those versions.
- `campanion run` now rejects any mill toolpath with a rapid that ends below the R plane (job retract height) or below the cut surface inside the part, or that traverses below it; the IR verifier's rapid-height findings were computed but never enforced.
- `surface` rapids no longer traverse at the job clearance when that is below the top of the STL model; they now stay at or above the model top. Every release from v0.3.0 to v0.8.0 could rapid across the part when the clearance was below the model top (for example a multi-operation surface job with default heights); regenerate that output.
- `rough` ramp and helix entries no longer rapid down below the R plane to the previous level; they start on the safe plane and feed down along the entry. `waterline` rapids stay at or above the model top when the retract height is below it.

### Changed

- `examples/tilted.job.json` drills the side plane again, now that mill drill feeds to depth.

## 0.8.0 — 2026-10-24

Milestone M11 Part C, items 8–10: rotary kinematics as data, indexed 3+2 tilted work planes with Haas and LinuxCNC posts, and angled turning with a B head.

### Added

- Indexed 3+2 posts: `haas_umc` writes Haas G254/G255 dynamic work offsets, and `linuxcnc_5ax` writes machine coordinates from the kinematics inverse.
- Indexed 3+2: a job can name a kinematics file, a solution policy and a tilted work plane. Profile, pocket, drill, engrave, face and chamfer are planned in that plane and mapped into the part frame, and the IR program stores the five-axis setup.
- Angled turning and boring with a B head: a lathe job can set `head.b_axis` and `b_deg` on face, rough, finish, groove and part, and a lathe post can index B.
- Rotary kinematics as data: joint definitions, validation, forward kinematics from joint angles to a tool pose, inverse kinematics, and solution choice.

## 0.7.1 — 2026-10-24

Milestone M11 Part B: sub-spindle, part transfer and two-channel lathe programs, plus the LinuxCNC mill-turn output and `live_mill`.

### Added

- LinuxCNC mill-turn: `linuxcnc_lathe` runs the live tool as spindle 1 (`M3 $1`), and `lathe.c_feed` can be `inverse_time` (`G93`) for C moves. (#66)
- `live_mill`: front-face and OD-wrap milling from a drawing, through face or cylinder interpolation, or on the Y axis. (#68)
- Sub-spindle setups: a lathe post can select and deselect a second spindle (Haas G55/G14, then G15/G54), and a lathe job can name a sub block and bind operations to main or sub.
- Part transfer and eject on a lathe. Chuck clamp, spindle sync, a run of either spindle, and a move of the sub-spindle housing. Haas writes G199/G198, M10/M11, M110/M111, M143/M144/M145 and B-axis moves. A job `transfer` hands the part across, with an optional part-off, and `eject` opens the sub chuck.
- Two-channel lathe programs: a post can split one file into channel sections with sync waits, a job can put an operation on channel 0 or 1 and insert a wait, and posting a multichannel program prints a per-segment balance on stderr. `generic_lathe_2ch` is a template.
- Viewer: a two-lane spindle timeline under the playback slider for lathe programs with a sub spindle (chuck, rotation and sync per spindle, a cursor and a one-line state label), and a `sub` tick at the sub housing Z on the section view.
