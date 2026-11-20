# GUI implementation plan (v0.4)

This is the plan for making the existing graphical workspace functional, without changing
its theme or layout. It maps the feature survey's approved gap list onto the v0.4 stages
already sketched in the GUI and in `campanion-job/2` / `campanion-machine/2`.

CAMpanion remains primarily command-line driven. The GUI is still in development and has
not been machine-tested. Nothing in this plan invents machine limits, feeds, exposures,
pierce times, or change timings. Those values come from the user, a user library, or a
cited public specification.

Engine work stays in [docs/plan.md](plan.md) and [docs/processes-plan.md](processes-plan.md).
This file is only the GUI sequence and the data-model hooks it needs.

## Principles

1. **No redesign.** Keep the v0.18.2 chrome: SolidCraft-like dark theme, machine selector
   at the top-left of the ribbon (name, type, profile dropdown, Manage…), stage tabs beside it, actions grouped under the selected
   tab, a CLI-first banner, and STOP pinned on every tab. Add or fill controls inside that
   shell. Do not move the chrome to new corners or invent a new navigation model.
2. **CLI first, GUI second.** Each GUI milestone exposes behaviour the library and CLI
   already have, or lands the CLI/library work in the same release as the controls. Golden
   CLI tests remain the source of truth for toolpaths and G-code.
3. **User-entered numbers.** Empty fields stay empty until the user types a value or
   applies a library preset. The GUI must not pre-fill feeds, power, kerf, exposure, lift,
   or axis limits. Existing job-file defaults that already live in `docs/job-file.md`
   (for example clearance and retract on a loaded job) are shown, not invented in the
   editor.
4. **Head-limited operations.** Add Operation only offers types valid for the selected
   machine head and process. A resin profile cannot be the selected head on a cutting job.
5. **Clean room.** Public documentation and first principles only. Do not read or copy
   source from other CAM or slicer programs.
6. **Physical testing.** GUI work can ship as a preview. Issues stay open until the
   behaviour has been physically tested. A code-generation error is fixed, with a
   regression test, before the next feature.

## Current baseline (v0.22.1)

The backend and CLI already carry the v0.4 data model:

- `campanion-machine/2`: gantry (user-entered bed, mixed heads with process, post, offsets,
  safe-Z, and a material library) or resin (optional bed and screen). v1 migrates on load.
  Numeric values are user-entered only. Unknown keys are rejected.
- `campanion-job/2`: one machine and one head, a parts list with linked instances,
  selectors (part, layer, colour, contour), real-unit power, `cut_order`, per-contour
  overrides, optional `sheet` / `nest`, optional `heights`, tabs, leads, cancellable
  planning, bed and work-area checks, unused-contour and crossing-lead-in warnings. v1
  migrates on load. Multi-head mixed jobs (`machine.heads`) are reserved and refused.
- CLI posts for laser, plasma, router, waterjet, oxy-fuel, and 2-axis wire EDM, plus mill
  and lathe posts. `campanion resin` writes `.pm7m`. `campanion supports` builds an
  undercut raft (narrow on the bed, full width on the part side) and straight pillars.
  Single-level tool changes, rectangular sheet packing, indexed 3+1, and 3+2 kinematics
  already exist on the CLI. None of that output has been proven on a machine.

The GUI already compiles against that model, but the editor is still a preview:

- Cutting workspaces show Process (labelled Job), Setup, Operations, Simulate, Export, and Run.
- Resin shows Orient, Supports, Slice, Export, and Run. Most Orient and Supports ribbon
  actions are placeholders.
- FDM shows Orient, Slice, Export, and Run.
- A 2D cutting job can be opened, a part imported and placed, profile operations edited,
  toolpaths generated with an explicit Generate button, G-code previewed, and a file
  exported. The Machines manager edits profiles. Properties are filtered to the active
  process. Copy, paste, delete, and undo exist for tools, operations, and a single
  imported part.
- Opening a multi-part `campanion-job/2` can generate and preview every instance, but
  saving from the current Setup editor is refused so instances are not overwritten.
- The Run tab, Frame, jog, DRO, and STOP are layout and lockout previews only. They do
  not open a port or send motion.

## Layout mapping (no new chrome)

| v0.4 stage | Where it lives in the current shell |
| --- | --- |
| New Job (machine, head, material, units, save location) | First cutting tab, today labelled Process. Keep its place. Expand it to the New Job step and relabel the tab **Job**. The top-left card is the machine selector. The process comes from the selected machine head; the Job tab records machine, head, material, units, and save location. |
| Setup | Existing Setup tab and its Source / Machine ribbon groups, plus layout tools on the bed viewport and the parts list in the tree. |
| Operations | Existing Operations tab. Until auto-regenerate ships, a temporary regenerate action lives only here, labelled as temporary. There is no Generate button on Setup, in the tree, or on Export. |
| Simulate | New stage tab on cutting workspaces, between Operations and Export. Same tab row, same typography, no new window type. |
| Export | Existing Export tab, with pre-flight checks before write. |
| Run | Existing Run tab (Frame, Start, Pause, jog, DRO). STOP stays pinned on every tab. |
| Resin Orient / Supports / Slice / Export | Existing resin tabs. Print-file send uses Export (write) and, once streaming exists, Run (send). Do not add a sixth tab. |
| FDM | Existing FDM tabs, with a Supports tab added when FDM work starts, matching the resin pattern. |

Turning and wire-cutting workspaces keep the same cutting tab row. They do not get a
separate layout.

## Format policy

Optional objects may be added to `campanion-job/2` and `campanion-machine/2` without a
major bump when they are omitted by default, rejected if invalid, ignored by planner
paths that do not yet use them (existing G-code stays byte-identical), and round-tripped
on save. A major bump (`campanion-job/3`, `campanion-machine/3`) is only for a breaking
change. Unknown keys stay rejected, so a reserved field named in this plan must be
implemented in the loader and saver before any GUI writes it.

## Data-model hooks (land early)

These hooks exist so later milestones do not rewrite the formats. Ship them as a small
schema-and-round-trip PR at the start of G1 (or immediately before it). The planner may
ignore unused blocks until the matching milestone. No example file may invent machine
numbers; test fixtures use clearly labelled example values.

### Job (`campanion-job/2`)

- **`z_zero`**: `stock_top` or `bed`. Omitted means today's convention (stock top is Z 0).
  `bed` requires a user-entered stock thickness (or an equivalent stock box already in
  the file). The GUI must not invent thickness.
- **`datum`**: which corner or centre of the stock or sheet is X0,Y0, with an optional
  typed offset. Omitted keeps today's origin at `(0, 0)`.
- **`start_from`**: `absolute`, `current_position`, or `user_origin`, plus a nine-position
  `job_origin` anchor on the job bounds. These are placement rules for Run and Frame.
  Planning in absolute machine coordinates stays the default. Omitted means absolute,
  matching a workspace that already shares the machine origin.
- **`order_rules`**: boolean `inner_first` and `cuts_last` (engrave or marking before a
  through cut). These are the default order, visible and overridable. `cut_order` remains
  the explicit drag list and wins for named operations.
- **`sheet` and `nest`** already exist (rectangular packing, `campanion-nest/1` output).
  Setup must treat the machine bed and the job sheet as different rectangles. A part can
  be on the bed and still off the sheet. Do not invent a second packing format here.
- **`rotary`**, **`kinematics`**, and per-operation **`plane`** already describe 3+1 and
  3+2. Leave them in the file. The basic 3-axis GUI does not edit them, but it must
  round-trip them and refuse to silently drop a 5-axis job. Continuous 5-axis stays out
  of the 3-axis editor.

### Machine (`campanion-machine/2`)

- **`kinematics`**: optional path to a user-supplied kinematics file, same schema as
  [docs/kinematics.md](kinematics.md). Omitted, the machine is treated as 3-axis. Pivot
  lengths and rotary limits come only from that file. Built-in examples stay illustrative
  and must not be offered as default machine data.
- **`changer` on a gantry head**: optional nested tool list under that head. Each child
  tool has an id, a name, optional XYZ offsets, and an optional change-sequence id. The
  sequence text or macro is user-supplied later (G6). The job still names one head today.
  When a head has child tools, an operation may name a child tool id. A file that omits
  `changer` is unchanged.
- Resin machines still have no heads. An extruder head may later carry an INDX- or
  MMU-style `changer`. Do not invent dock positions or heat times.

### IR and posts

- Poses already carry a tool axis. Continuous 5-axis and TCP posts stay on the engine
  roadmap (plan.md M11). The GUI plan only requires that job and machine files can point
  at user kinematics without a format rewrite.
- Tool-change events already exist at a tool-id change. Nested changes need a second
  event level (head change, then child-tool change) when G6 starts. Reserve the IR
  shape in the hook PR if it can be omitted on 3-axis jobs so current posts stay
  byte-identical.

## Milestones

Ship in this order. Sizes are rough: **S** is a few days and a small PR, **M** is one
substantial release-sized chunk, **L** is several PRs or a large release. Each milestone
can split into reviewable PRs. Version numbers are not assigned here; they follow the
usual release process after the work is on `main` and the CLI-first note is updated.

### G1 — Setup (L)

Machine selector, New Job, drawn bed, parts, layout tools, origin, datum, Z-zero, and
sheet versus bed.

**GUI**

- Job tab (today's Process tab): sub-process chips, machine and head picker, material
  preset picker, units, save location. Machines… still opens the existing manager.
- Setup ribbon: Import (multi-file), linked copies, Array, Arrange, Align, Mirror,
  Rotate, and typed placement in the properties panel. Place on Face for 3D meshes
  (Orient already has a Lay flat placeholder; wire it for STL/STEP meshes).
- Viewport: machine bed drawn from the profile; job sheet drawn when present; off-bed
  parts in red; off-sheet parts warned in a second colour or an overlay and listed in
  messages. Drag and typed moves both write `placement`.
- Job / Setup properties: `z_zero`, stock thickness when Z-zero is the bed, XY datum,
  start-from and job-origin. Start-from is shown here so it matches Run and Frame later.

**Backend**

- Land the format hooks above and round-trip them.
- Setup must load and save multi-part `campanion-job/2` (linked instances, arrays).
  Saving a multi-part job from the GUI is no longer refused once the editor is safe.
- Arrange and array write instance placements. A basic pack can call the existing
  rectangular `nest` planner; true-shape work waits for G8.

**CLI parity**

- No new cutting operations. `campanion run` already honours bed and work-area limits
  and part placements. Document `z_zero` and `datum` in `docs/job-file.md` when they
  affect posted coordinates, with tests that old files (omitting the keys) stay
  byte-identical.

**Tests**

- Round-trip of hook fields; v1 and v2 jobs still load.
- Multi-part save does not collapse instances.
- Off-bed export still refuses; off-sheet is a warning or refusal as documented.
- GUI tests: layout actions write the expected placements; empty numeric fields stay
  empty; a profile without a bed does not invent one.

**Acceptance**

- A new job can pick a user machine and head, import several parts, make linked copies
  and an array, arrange them on the drawn bed, set datum and Z-zero, and save a
  `campanion-job/2` file the CLI can run unchanged.
- Absolute source paths still work on an unsaved job; Save stores a relative path when
  one can be formed.
- The chrome, banner, and STOP are unchanged.

### G2 — Operations (L)

Head-limited operations, automatic cut-order rules plus drag override, tabs, height
planes, lead-ins and start points, plasma small-hole rules, real-unit presets with
overrides, and auto-regenerate.

**GUI**

- Operations ribbon: Add Operation lists only types valid for the selected head
  (laser: cut / score / fill or the existing profile modes; plasma and jet: inside,
  outside, on the line; spindle: face, profile, pocket, drill first). No 5-axis
  operation types in this milestone.
- Tree: operations, then contours or layers under them. Drag the list to write
  `cut_order`. Visible checkboxes or toggles for `inner_first` and `cuts_last`.
- Viewport: lead-in / lead-out markers and start points that can be dragged; tabs as
  markers on through profiles; per-contour overrides drawn in the existing orange
  override style, with Reset and Save to library on the properties panel.
- Properties: real-unit presets (Assign copies values and leaves them editable; an
  optional Link follows later library edits and locks the fields). Output and Show
  toggles per operation. Height planes for spindle work: clearance, retract, feed,
  top of stock, bottom — shown, not defaulted in the editor. Tabs: width and count or
  spacing, optional height, all user-entered.
- Plasma: pierce fields already on the material library (delay, and so on) stay
  user-entered. Small-hole and THC-off-on-leads rules are optional, named operation or
  tool rules whose thresholds and actions are user-entered. No invented amperage or
  feed.
- Auto-regenerate: debounce on the order of 300 ms, background thread, cancel on a
  newer edit (`plan_job_cancellable` already exists). Last-good toolpaths stay visible
  and are marked stale. There is no Generate button.

**Backend**

- Implement `order_rules` in the planner so the default order is inner-then-outer and
  marking before a through cut, then apply `cut_order`. Tests must show an explicit
  `cut_order` still wins.
- Tabs, heights, leads, face, pocket, and drill already exist in the job file and
  `campanion run`. The GUI writes those fields; do not fork a second planner.
- Plasma rules are new job or tool data. Until a cited controller dialect defines THC
  codes, a rule may record the intent and only emit codes that a post already has;
  otherwise it is a documented refusal, not a guessed M-code.

**CLI parity**

- `campanion run` applies the same order rules so a saved GUI job and a CLI run match.
- Existing lead, tab, and height goldens stay byte-identical when the new keys are
  omitted.

**Tests**

- Order-rule unit tests (open vs closed, inner vs outer, marking vs cut).
- Auto-regen cancellation: a newer edit drops an in-flight plan.
- Override styling: a changed field is marked; Reset restores the library value.
- Planner refuses raw 0..1 power; watts without head `max_power` still refuse.

**Acceptance**

- Editing an operation updates the toolpaths after a short pause, without a Generate
  click. Dragging cut order and start points is reflected in Simulate (or the current
  preview until G3).
- A through profile can carry tabs. A spindle operation can show and write height
  planes. A plasma operation can keep pierce values from a preset and attach a
  user-entered small-hole rule.

### G3 — Simulate and Export (M)

**GUI**

- New Simulate tab on cutting workspaces, between Operations and Export.
- Playback, a scrubber, a linked G-code panel (the highlighted line follows the
  scrubber), time estimate when the post can provide one, and the existing message
  list for checks. Stale plans are labelled.
- For mills, optional stock display can wait if the in-process stock model is not
  ready; path playback is enough for this milestone.
- Export tab: pre-flight list (off-bed refusal, off-sheet, unused contours, crossing
  lead-ins, missing required fields). Export is refused when a check is an error.
  Preview G-code stays available. Removable-media eject can wait unless a path is
  already easy.

**Backend**

- Reuse `campanion-view` / IR preview and the existing post-and-read-back path.
  Simulate must not invent a second G-code writer.

**CLI parity**

- `campanion view`, `campanion post`, and verify reports stay the reference. Export from
  the GUI is the same post the CLI would run for that job.

**Tests**

- Scrubber index matches a known IR move and G-code line on a golden job.
- Pre-flight refuses off-bed export and allows a job that only has warnings, as
  documented.
- Screenshot set updated for the Simulate tab.

**Acceptance**

- A user can play and scrub a plan, see the matching G-code, fix an error, and export
  only after pre-flight passes. Frame is still inert until G5, but Simulate does not
  depend on a machine connection.

### G4 — Resin GUI (L)

Orient, island detection, paint-on supports, hollowing and drain holes, and the undercut
raft, on the existing resin tabs.

**GUI**

- Orient: import STL, Place on Face, rotate (existing step chips), auto-orient once a
  library or CLI routine exists, arrange on the drawn plate. Off-plate parts in red.
- Supports ribbon: Auto Generate and Add / Delete become real. Island detection with
  a way to step through islands and skip tiny ones. Paint-on supports and no-support
  zones (the No-Support Zone chip is already a placeholder). Click-to-place pillars;
  drag tip and base; copy parameters between supports. Presets (light / medium /
  heavy or user-named) are library entries with user-entered tip, shaft, and foot
  sizes — no built-in millimetre defaults.
- Raft: expose the CLI undercut raft (chamfer angle, thickness, margin, corner relief,
  part lift). The bed side stays the smaller face.
- Hollowing and drain holes, plus a cavity or suction-cup warning, on Supports or
  Slice. Values are user-entered.
- Slice: layer slider, first-layer and island review, then Export to `.pm7m` with
  `--params-from` or explicit exposure and lift fields. The GUI must not invent those
  numbers.

**Backend**

- Wire the GUI to `campanion-resin` and `campanion supports`. Island detection, paint
  masks, and hollowing are new library work if they are not already in the CLI; they
  ship with CLI flags in the same release.
- Image orientation remains unverified until an asymmetric test print is done
  (`docs/resin.md`). The GUI must surface that warning.

**CLI parity**

- Every exposure, lift, and support length still comes from flags, a params file, or a
  sample `.pm7m`. `campanion resin-info` remains the file checker.
- Painted points should be writable as the existing `--points` file so a GUI job is
  reproducible on the CLI.

**Tests**

- Raft undercut still refuses a collapsed bed face.
- Island detection fixtures; paint-on vs auto points both slice.
- Hollow with a drain hole produces a void and a through hole in layer images.
- No silent default for exposure or lift: omitted values refuse.

**Acceptance**

- A part can be oriented, supported with the undercut raft, inspected for islands,
  hollowed with drain holes, sliced, and exported to a `.pm7m` that `resin-info`
  accepts. Nothing has been printer-tested until that test is done.

### G5 — Run and LM2 streaming (L)

**GUI**

- Existing Run ribbon: Start, Pause / Resume, Frame (bounding-box and a rubber-band
  option for irregular jobs), jog, DRO, Set Origin / Home as already stubbed.
  Lockouts already exist (`can_jog`, `can_frame`, and so on); hook them to a real
  connection.
- Start-from and job-origin from G1 must agree with Frame and Start.
- STOP on every tab remains the emergency control and must cut motion, not only the
  preview state.
- Until a connection is configured, controls stay disabled with a clear reason, not a
  fake run.

**Backend**

- A streaming layer for the user-selected controller. Grbl-class and LM2 streaming are
  in scope only after the owner supplies the public protocol or dialect to implement
  (port, framing, jog, feed hold, and stop). The plan does not invent those codes.
- No motion is sent until pre-flight from G3 passes.

**CLI parity**

- File export stays the offline path. Streaming is GUI- and maybe a later CLI send
  command. Posted G-code must match the file the CLI would write.

**Tests**

- A recorded protocol fixture (from a public spec or a captured session the owner
  provides) is replayed: Frame, Start, Pause, Resume, STOP.
- Lockout tests: no jog while running; STOP always enabled.
- Disconnect and error paths refuse to claim success.

**Acceptance**

- With a connected, user-configured controller, Frame shows the job placement, Start
  streams the posted program, Pause holds, and STOP stops. This milestone is not
  done until it has been physically tested on the target machine. Until then it ships
  only as an untested preview and the banner stays up.

### G6 — Tool changing, including nested changers (M)

Single-level changes first, then a gantry head that owns child tools (INDX- or
MMU-style).

**GUI**

- Machine manager: on a gantry head, an optional child-tool list (ids, names, offsets).
  Change-sequence fields are empty until the user enters them or points at a cited
  macro.
- Operations: a tool picker that can name a child tool of the selected head.
- Simulate / Run: show the two-level change (head, then child) in the timeline and
  G-code panel.

**Backend**

- Today's `Event::ToolChange` covers a tool-id change and posts `T{tool} M6` (and
  similar) when the post has a template. Extend the IR so a head change and a child
  change can be emitted in order, omitted on jobs with no `changer`.
- Posts write only codes from a user-supplied sequence or a cited dialect. Do not
  invent INDX dock, heat, or prime timing.

**CLI parity**

- `campanion run` on a job that names child tools writes the same nested changes the
  GUI previews. Existing single-tool jobs stay byte-identical.

**Tests**

- Single-level goldens unchanged.
- A fixture with a head and two child tools emits head change, then child change, then
  the path, then the reverse on the way back, with user-supplied sequence text.
- A missing sequence is a refusal, not a guessed M-code.

**Acceptance**

- A spindle or extruder head can list child tools. A job can switch the gantry to that
  head and then to a child tool, and the post writes both levels in order when the
  user has provided the commands.

### G7 — FDM, including wave bridging (L)

Follows [docs/processes-plan.md](processes-plan.md) step 8 (planar FFF). Wave bridging
for 90° overhangs is in scope for this GUI milestone, but the paths are generated in
the library, not only in the editor.

**GUI**

- Add a Supports tab to the FDM workspace, in the same slot as resin Supports.
- Job / Setup: printer profile (machine), filament and print profile as user libraries,
  Place on Face, arrange, instances.
- Slice: layer slider, first-layer and floating-region checks, time and material
  estimates once the IR can provide them.
- Settings: layer height, line width, perimeters, infill spacing, retraction threshold,
  temperatures, and wave-overhang thresholds are job or library data. The long expert
  list, if any, is behind search. No invented slicer defaults in the GUI.
- Wave bridging: a visible option for near-horizontal overhangs. It turns on wave or
  arc paths that start at a supported edge and grow into the unsupported region of the
  same layer (the clean-room approach already described for planar FFF: public
  algorithm descriptions only, no third-party slicer source). Unsupported islands with
  no seed edge do not invent a starting wall; they need supports or a refusal.
- Paint-on supports and blockers when that library work exists. Tree or organic
  supports are later.

**Backend**

- Library and CLI path generation must exist in the same release (perimeters, skins,
  zigzag infill, retraction, strand checks, wave overhangs). A printer post waits
  until the owner names the public dialect (processes-plan decision 9).
- The GUI writes the job and previews the IR. It does not contain a second slicer.

**CLI parity**

- A saved FDM job runs headless and produces the same IR the Slice tab shows.

**Tests**

- Analytic overhang: waves stay in the unsupported region and start from a supported
  edge.
- No wave is generated for an island with no seed edge.
- Mill and laser goldens stay unchanged (optional deposited length omitted).
- Omitted temperatures or layer height refuse, rather than default.

**Acceptance**

- A user can place a part, enable wave bridging for a 90° overhang with user-entered
  thresholds, inspect layers, and export or (later) post to a cited dialect. Supports
  beyond paint-on and basic auto can follow in a patch.

### G8 — Nesting (M)

**GUI**

- Setup: a Nest or Pack action that fills the existing `sheet` and `nest` job blocks
  (gap, margin, tool radius, clamp keep-outs, rotation step). Show the sheet, keep-outs,
  and placements on the bed.
- Rectangular, axis-aligned packing first — that planner already exists. True-shape
  packing, remnants, and common-line cutting are a follow-up inside this milestone only
  if they stay reviewable; otherwise they are a later PR, still under G8, not a new
  layout.

**Backend**

- Reuse `campanion run --nest-out`. A true-shape packer, if added, should write the same
  `campanion-nest/1` placement list (or a documented extension) so the rest of the
  pipeline is unchanged.

**CLI parity**

- GUI nest and `campanion run` on the same job produce the same placements.

**Tests**

- Existing `examples/nest.job.json` placements stay exact.
- A part that does not fit still fails with the same error.
- Off-sheet after a nest is visible in Setup and blocked at Export as documented.

**Acceptance**

- Several parts can be packed onto a user-entered sheet with keep-outs, saved, and run
  from the CLI with the same result.

### G9 — 5-axis GUI (L, after a solid 3-axis GUI)

A must-have, but not until G1–G3 feel solid. Keep the data model ready (hooks above and
plan.md M11). This milestone is a GUI on top of kinematics and posts that already
exist or are in progress, not a new engine.

**GUI**

- Setup or a Machine manager page: attach a user kinematics file, show joint names and
  the limits from that file (never a built-in travel table).
- Operations: an optional work-plane / 3+2 control that writes `plane` (and later
  continuous tool-axis strategies as M11 lands). The default editor remains 3-axis.
- Simulate: playback of tool axis and, when M11 machine simulation exists, a kinematic
  model with collision highlighting. This is not full machine CAD.

**Backend**

- No second kinematics stack. Use `docs/kinematics.md`, job `kinematics` / `plane`, and
  the posts that already refuse uncited motion.

**CLI parity**

- A 3+2 or continuous job saved from the GUI is runnable with `campanion run --post`
  and matches the Simulate view.

**Tests**

- Round-trip of kinematics path and plane.
- A 3-axis job still writes identical G-code.
- Unreachable poses and limit violations surface in the GUI as planner errors.

**Acceptance**

- Indexed 3+2 on a user-supplied machine model can be set up, simulated, and exported
  from the GUI. Continuous 5-axis follows the M11 staging (5A–5H) and only appears in
  the GUI when the corresponding engine work is on `main`.

## Risks

- **Unverified resin orientation and untested streaming.** Shipping a preview is fine;
  calling it production-ready is not. The banner stays until physical tests exist.
- **Silent numeric defaults.** Job-file defaults already exist for some heights. The
  GUI must show them as loaded values, not as hidden editor pre-fills for new jobs.
- **Auto-regenerate cost.** Large multi-part jobs need cancellation and stale display
  or the UI will feel frozen or will show a mix of old and new paths.
- **Nested changers without real sequences.** The hook must store structure without
  inventing macros. A tempting “typical M6” default would be a bug.
- **Format drift.** If the GUI saver drops unknown-but-valid v2 fields (rotary,
  kinematics, nest), later 5-axis and nesting work will be rewritten. Round-trip tests
  are part of G1.
- **Scope leak into CAD.** Node editing, booleans, and art tools stay out. SolidCraft
  is the CAD side; CAMpanion imports and arranges.
- **FFF post dialect.** Wave bridging can preview from IR; a printer file waits for a
  named public specification.

## Open questions

These need a decision or real machine data before the matching milestone can finish.
None of them should be filled in with guessed numbers or dialects.

1. **Start-from on plasma and routers.** Is the LightBurn-style absolute / current
   position / user origin model required only for lasers, or for every gantry Run tab?
2. **Job tab label.** This plan relabels Process to Job in place. Confirm that name.
3. **Off-sheet policy.** Warning versus export refusal when a part is on the bed but
   outside the job sheet.
4. **Linked presets.** Is Link (library edits follow the job, fields locked) wanted in
   G2, or is Assign enough at first?
5. **LM2 and Grbl streaming.** Public protocol or dialect, port or transport, jog,
   feed hold, and stop behaviour must be supplied or cited before G5 writes a single
   motion command.
6. **Nested changer sequences.** INDX, MMU, or other child-tool dock, heat, prime, and
   purge commands and timings must come from the machine's documentation. The data
   model can land without them; the post cannot.
7. **Plasma THC and small-hole codes.** Which codes the target posts may write, from a
   cited manual. Until then, store the rule and refuse uncited output.
8. **Resin printers beyond the M7 Max `.pm7m` path.** Stay on that format until another
   machine and sample file are named.
9. **FDM post.** Name the public dialect before G7 exports machine files (IR preview
   can ship first).
10. **Wave-overhang parameters.** Which thresholds (angle, spacing, and similar) are
    required job fields, all user-entered.
11. **5-axis machine models.** Real pivot lengths, axis limits, and TCP versus non-TCP
    post choice come from user kinematics files, not from the illustrative examples in
    the repo.
12. **Order-rule defaults for old jobs.** When `order_rules` is omitted, keep today's
    file order (byte-identical) rather than silently applying inner-first?

## Related documents

- [docs/plan.md](plan.md) — engine milestones, including M11 5-axis.
- [docs/job-file.md](job-file.md) — `campanion-job/2`, heights, tabs, leads, nest, tool
  changes, kinematics.
- [docs/machine-profile.md](machine-profile.md) — `campanion-machine/2`.
- [docs/resin.md](resin.md) — `.pm7m` slicer and undercut supports.
- [docs/kinematics.md](kinematics.md) — rotary kinematics; example files are not
  measured machine data.
- [docs/processes-plan.md](processes-plan.md) — planar FFF (step 8) and later additive
  work.
