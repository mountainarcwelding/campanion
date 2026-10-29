# Changelog

## [Unreleased]

### Added

- A toolpath may set `continuous` (default false, omitted from JSON). On a continuous toolpath the verifier rejects a feed whose tool axis changes by more than the program setting `max_axis_step_deg` (default 5°, range `(0, 90]`). The error names the toolpath index, the move index and the angle.
- `solve_path` turns a list of tool poses into joint positions. It keeps the rotary branch, unwinds continuous rotaries so a joint does not jump by 360°, and reports the first pose that leaves a travel limit or that no solution can reach.
- Posts reject a continuous toolpath with `post `{id}` has no continuous five-axis output`.

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
