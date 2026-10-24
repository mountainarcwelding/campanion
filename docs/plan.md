# CAMpanion plan

## Goal
An open, machine-neutral CAM engine in Rust that turns geometry into good toolpaths, starting with
2D and working up to complex machines. It is a companion to SolidCraft now and is meant to merge
into it later.

Direction (2026-10-21):
- Rust, following SolidCraft's source rules (see `AGENTS.md`), so a merge is straightforward.
- Not tied to one machine or process yet. Mill, router, plasma, laser, waterjet, drag knife and
  additive are all "processes" on the same engine.
- Get path processing right first. Start with 2D, then 2.5D, 3-axis, 4/5-axis, turning and
  multi-channel machines.
- The toolpath format must support up to **13 axes** by the end. The format's hard cap is 16 axes per machine, matching common
  controller joint limits, so 13 fits with room to spare.

## Design principles
1. **Machine-neutral core.** Operations produce toolpaths in a neutral format (the IR). Machine
   definitions and posts come last and only consume the IR. No operation assumes a controller.
2. **Exact geometry where possible.** Keep lines and true arcs through offsetting and output, so
   G2/G3 come out as arcs, not chord soup. Splines are approximated within a stated tolerance.
3. **One tolerance model.** Every operation takes explicit tolerances (linear, angular, chord
   error) with sane defaults; results say what tolerance they met.
4. **Never crash, always explain.** Bad geometry returns an error that names the problem
   (open contour at x,y; self-intersection; tool too big for feature).
5. **Everything is callable headless.** Library API first, CLI second, UI last. That matches
   SolidCraft's "everything is a command" rule and makes integration easy.
6. **Verifiable.** Each milestone lands with analytic tests, property tests and a simple
   verifier (gouge/limit checks), not only golden files.

## Milestones

### M0: Scaffold (in progress)
Cargo workspace, crates `campanion-geom`, `campanion-path`, `campanion-ir` and `campanion-cli` (binary `campanion`),
SolidCraft's lint and format settings, CI workflow, `cargo-deny` licence check, local
`scripts/check.sh`.
Done when: a clean checkout passes fmt, clippy, tests, the wasm check and cargo-deny.

### M1: 2D geometry core
- Primitives: points, vectors, lines, arcs (centre/radius/sweep and bulge form), closed and open
  contours made of lines and arcs, bounding boxes, orientation, area, length, point-in-contour.
- Tolerance model and robust predicates (near-zero lengths, near-tangent joins, almost-closed
  contours).
- Cleanup: join loose segments into contours, remove duplicates and zero-length pieces, merge
  collinear lines and co-circular arcs, fix direction (outer counter-clockwise, holes clockwise).
- Offsetting with true arcs (inward and outward, round corners), including self-intersection
  removal and split results.
- Region booleans (union, difference, intersection) on line/arc contours.
- Library choice is an ADR in M1: candidates are our own implementation or permissive crates such as
  `cavalier_contours` (MIT OR Apache-2.0, line/arc offsets) and `i_overlay` (MIT OR Apache-2.0,
  booleans). They are wrapped behind our own types so they can be swapped.
Done when: offsets match analytic results (circles, rectangles, slots) to 1e-6 mm, property tests
(offset out then in returns the shape; area relations; no self-intersections) pass on random
inputs, and hostile input never panics.

### M2: 2D path processing
- Profile/contour: inside, outside or on the line; climb or conventional; tool radius or kerf
  compensation, done in the CAM (computed path) or left to the controller (G41/G42 flag in the IR).
- Pockets: offset rings and zigzag/raster, islands, corner clean-up, stepover control.
- Drilling points from circles and points (positions only; cycles are expanded or marked in IR).
- Engraving/on-line following, with optional V-carve later.
- Lead-in/lead-out (line, arc, perpendicular), overcut, pierce points, tabs/bridges.
- Start-point selection, cut ordering (inside features before outside, nearest-neighbour then
  improved), linking moves and rapid minimisation.
- Depth passes and ramp/helix entry for 2.5D-ready output.
- Per-operation process timing, as generic events and dwells: pierce delay (dwell at process
  start) and post/end delay (dwell at process stop, before the next rapid), each with a toggle
  for whether the process stays on during the dwell (defaults: pierce on, post off), plus
  optional pierce height and cut height on the process axis. Applied at every start and stop,
  including around tabs and after lead-ins/lead-outs.
Done when: a test plate (outer profile, holes, slots, a pocket with an island) produces correct,
ordered toolpaths for both a router-style tool and a kerf-only process, checked by the verifier.

### M3: Machine-neutral toolpath IR
Two levels, both serde JSON with a published JSON Schema and a format tag (`campanion-ir/N`):
- **Tool-space (CL) level:** what the operations produce. Tool tip position plus tool axis
  direction in the part frame, so 2D, 3-axis and 5-axis paths share one model.
- **Axis-space level:** what a machine runs, produced later by kinematics for a specific machine.
  Up to 16 axes per machine (target 13), each with an id, a letter or index (X Y Z A B C U V W,
  then indexed extras), linear or rotary, limits, and its place in a kinematic chain
  (parent link, direction, offset). Channels group axes for machines with several heads or
  spindles; sync points coordinate channels.
- Motion: rapid, linear, arc (with plane), helix, and later splines; feed rate per move.
- Process timing from M2 appears as process on/off events plus dwells in the order chosen by
  the toggles; posts never re-derive it.
- Process events are generic, not machine-specific: process on/off with a process type
  (spindle, torch, laser, extruder, knife, …) and parameters (speed, power, pierce delay),
  dwell, tool change, coolant/air/assist, probe, comments, and named custom events for later.
- A simple simulator/verifier: total length and time estimate, bounds, rapids into material,
  axis limit checks.
- A generic RS274/NGC writer used only as a **test sink** to inspect output. It is not a post for a
  real machine.
Done when: the IR round-trips through JSON unchanged, the schema validates sample files, and a
13-axis sample with two channels loads and checks.

### M4: Inputs and job files
- DXF and SVG import of 2D geometry (layers, blocks, arcs, polylines with bulges, splines).
- STEP parts through `solidcraft-cli` (sections and planar faces to 2D contours).
- A job file that links to the source part by path and content hash, so a changed part
  regenerates its toolpaths instead of going stale.
Status: DXF (LINE, ARC, CIRCLE, LWPOLYLINE) and SVG (paths, basic shapes, transforms, units,
layers) are in. Good first contributions: DXF POLYLINE (old-style, with VERTEX), ELLIPSE
(`crates/import/src/ellipse.rs` has the maths), SPLINE, and SVG curves fitted as arcs
instead of flattened to lines.
Done when: the test plate imports from DXF, SVG and a SolidCraft STEP export with identical results.

### M5: Viewer and preview
An egui viewer (same toolkit as SolidCraft) for 2D/3D toolpath preview: cutting moves, rapids,
lead-ins, pierces, order numbers and a simple playback. It only reads the IR.

### M6: 2.5D (done)
Multi-depth pockets and profiles with stock to leave (wall and floor) and a finishing pass,
pocket stepover, facing (zigzag with overrun), and chamfer/deburr with V-bits along closed
contours. Thread milling and adaptive clearing are still to come.

### M7: 3-axis surfacing (done)
Started after the 2.5D milestone (multi-depth pockets and profiles with stock to leave and a
finishing pass, facing, chamfer and deburr).
- **Input:** triangle meshes (binary and ASCII STL, and tessellated STEP through
  `solidcraft-cli`), with a stated chord tolerance. The mesh is checked for watertightness and
  degenerate triangles, and the result names the problem.
- **Tools:** flat, ball and bull-nose end mills described by a radial profile (radius, corner
  radius, flute length), so contact and gouge maths are shared by every operation.
- **Drop-cutter core:** for a tool at (x, y), the lowest Z where it touches the mesh, exact for
  vertices, edges and facets per tool shape. Clean-room from the published geometry. No
  OpenCAMLib code.
- **Roughing:** Z-level (waterline-based) roughing with step-down and stock to leave, reusing the
  2D offset and pocket code on each slice of the part.
- **Finishing:** parallel (raster) finishing along drop-cutter lines and waterline finishing for
  steep walls, with a slope split between them. Scallop height drives stepover for ball tools.
- **Safety:** every 3D path is checked against the mesh. No point is below the drop-cutter Z
  within tolerance (gouge-free), and rapids stay above the stock.
- **Output:** the same CL-level IR (tool tip plus axis), so posts and the viewer need no changes.
  The viewer gains a mesh and stock preview.
Done when: analytic tests pass (a ball on a plane, a sphere and a cylinder at known Z), and
property tests on random meshes show no gouge and stay within the scallop height.

### M8: Machine and manufacturer posts (done)
Get the basics solid on real controllers before adding axes. Posts consume only the
machine-neutral IR and cover everything built so far (2D, 2.5D, 3-axis):
- LinuxCNC, GRBL, Mach3 and Fanuc/Haas-style mill posts, plus one plasma/laser controller
  (process on/off, pierce delay, torch height and kerf from the IR's generic process events).
- A post is a data file (dialect, codes, number formats, header and footer templates, supported
  cycles, arc rules, limits) with a small documented hook interface for what data cannot
  express. A contributor adds a controller by writing one post file plus golden tests, without
  touching operations. Machine files (axes, limits, kinematics, process heads) pair with posts.
- Written from public controller manuals only.
Done when: every example job posts to each controller, read-back of each program matches the
IR within tolerance, and golden files cover each post.
4th axis and beyond start only after this milestone.

### M9: 4th axis: indexing, wrapping and rotary finishing (done)
One rotary axis (A about X, or B about Y). Simultaneous 5-axis, tool tilt and a machine
kinematics model are out of scope.
1. **Machine and setup data:** an optional rotary axis in the setup (axis, centre, direction,
   travel limits). Posts state which rotary axes they support and how A/B words are written.
2. **Indexed 3+1:** each operation may set an index angle. Existing 2.5D and 3D operations run
   in the rotated frame; the post writes the A move and safe retracts between indexes, with
   clearance checked against the stock's swept radius.
3. **Wrapped toolpaths:** a flat 2D/2.5D path wrapped onto a cylinder of given diameter as
   A-axis moves (lettering, flutes, cams), with feed handled as inverse time or degrees per
   minute, chosen per post.
4. **Posts:** A-axis output for the LinuxCNC, Mach3, Fanuc and Haas posts with golden files;
   Grbl only when its post opts in. Read-back compares rotary moves within angle and
   surface-length tolerance. The IR carries rotary motion as an axis value, not a new move
   kind; posts without that axis reject the program cleanly.
5. **Continuous rotary finishing:** drop-cutter passes around the part axis (rotary raster),
   gouge-checked in the rotated frame.
6. **Viewer:** shows the rotary axis and index angles.
Done when: property tests show indexing round-trips and unwrapping a wrap gives back the flat
path, rotary finishing passes the gouge check, and every 4-axis post has golden files.
Releases: v0.5.0, including continuous rotary finishing and the viewer.

### M10: Turning (done except item 2's STEP section, carried into v0.7.0)
Lathe work on the main spindle with one turret, X and Z only. The setup model is built so live
tooling, Y, B and a sub-spindle (M11) slot in without a rewrite.
- **Machine model:** a lathe machine has named spindles (`main`, later `sub`), each with its own
  C axis and work zero, and turrets/heads that carry X and Z and optionally Y and B. Every
  operation names its spindle and head. Multiple channels use the IR's existing channels and
  sync points (up to 13 axes). XZ toolpaths are stored as part-frame poses in radius; diameter
  mode is applied only in the post.
1. **Turning setup and IR:** X in diameter or radius (post setting), Z, constant surface speed
   (G96) with a spindle limit and fixed RPM (G97); bar and tube stock; insert shape, nose
   radius and orientation as data.
2. **Profile input:** a 2D half-profile from DXF/SVG; a revolved section from STEP (through
   `solidcraft-cli`) or STL.
3. **Facing and OD/ID turning:** facing, OD and ID roughing with stock to leave, finishing with
   computed nose-radius compensation (the path is the compensated tool-centre or tip path, not
   G41/G42 by default).
4. **Threading, grooving, parting, drilling:** single-point threading as explicit passes
   (infeed angle, spring passes; canned G76 as a post option), grooving, parting off, and
   centreline drilling.
5. **Checks:** insert-versus-profile gouge and holder collision checks, with property tests.
6. **Lathe posts as data files:** LinuxCNC lathe, Fanuc/Haas lathe and a Grbl-style lathe,
   with golden files and read-back.
7. **Viewer:** an XZ half-section showing stock, the part and the removed region.
Releases: v0.6.0 after 1, 2, 3 and 6; v0.6.1 after 4, 5 and 7.
Status: done in v0.6.1 except the revolved section from STEP in item 2 (DXF, SVG and STL
profiles are in). That item moves to M11 Part A, first in v0.7.0.

### M11: 5-axis and advanced turning (in progress)
Aimed at mill-turn machines in the class of the Haas ST and DS lines: live tooling with C and
Y, sub-spindle work and part transfer, synchronised channels, and B-axis heads. Simultaneous
5-axis milling (indexed 3+2 first, then continuous with tool-axis control and collision checks)
belongs here too.

Post order: **Haas first, then LinuxCNC.** Each part lands with a Haas mill-turn post (ST/DS
class: M154/M155 C-axis, M133/M134/M135 live tool, G112, G107, Y axis, sub-spindle sync), with
every code checked against Haas's public operator's manual and G/M-code reference pages and cited
in the post data or docs. The LinuxCNC equivalent follows in the same release where it fits, or
in the next.

Part A: live tooling on the main spindle (the IR already has C per spindle and optional Y per head)
0. **STEP section for turning (from M10.2):** `campanion lathe` reads a revolved half-profile
   from a STEP part through `solidcraft-cli`, reusing the STEP path and `--up-axis` of
   `campanion run`.
1. **Spindle modes in the IR and posts:** spindle-as-C (C-axis mode on/off, Haas M154/M155;
   LinuxCNC as a rotary axis), live-tool spindle speed and direction (Haas M133/M134/M135) as
   post data, C clamp/unclamp.
2. **Face and OD milling/drilling at indexed C:** face drill patterns, cross holes, flats and hex
   at a fixed C, polar coordinates. Reuses the 2.5D ops; the post maps the part frame to X
   (diameter), C and Z.
3. **Polar and cylindrical interpolation:** G112 (XC) and G107 (ZC) as post options, each with
   an explicit XC or ZC fallback through the M9 wrap path.
4. **Y-axis milling on the turret (ST-Y and DS class):** off-centre pockets and flats with real Y;
   head reach and travel limits come from the machine model.

Part B: sub-spindle and channels
5. **Sub-spindle setups:** a second work zero, the Z direction flipped, the sub's C axis;
   operations bound to `sub`.
6. **Part transfer as an operation:** spindle sync (M-code data), sub approach and clamp,
   part-off with the sub holding, pull and eject. Simulated as a two-spindle timeline in the
   viewer.
7. **Two-channel programs (upper/lower turret):** channel assignment per operation, sync points
   (wait codes as post data), and a balance/timing report. Single-channel posts reject
   multichannel programs.

Part C: B-axis heads and 5-axis milling
8. **Kinematics as data:** a B head on a mill-turn and table-table/head-table 5-axis mills:
   pivot lengths, axis limits, solution choice. Forward/inverse kinematics with property tests.
9. **Indexed 3+2:** tilted work planes (G68.2 or the LinuxCNC equivalent, or explicit rotated
   coordinates), any 2.5D/3-axis op on a tilted plane, with a reachability and limit check.
10. **Angled turning and boring with a B head** (insert orientation follows B).
11. **Continuous 5-axis:** tool-axis control (lead/tilt, toward point/curve, swarf on ruled
    surfaces), axis smoothing and singularity handling near B=0.
12. **TCP and non-TCP posts:** G43.4/TCPC and LinuxCNC TCP kinematics, or the post doing the
    inverse kinematics; inverse-time feed where the control needs it.
13. **Collision checks** for holder/head against stock, fixture and part for 5-axis and
    mill-turn, plus a viewer machine simulation (kinematic model, not a full machine CAD).

Also open from M10: canned `G76` threading as a post option.

Releases: v0.7.0 after 0–4 (Haas post; LinuxCNC as soon as it is ready), v0.7.1 after 5–7,
v0.8.0 after 8–10, v0.8.1 after 11–13.

### M12: Additive and non-planar slicing
See the additive entry under Later; it becomes M12 after M11.

### Later (each its own milestone when we get there)
- **2.5D extras:** thread milling, adaptive clearing.
- **3-axis extras:** scallop finishing, rest machining, stock simulation.
- **Multi-channel up to 13 axes:** multi-head machines beyond M11, sync between channels.
- **Additive and non-planar slicing (M12):** extrusion as one more process
  in the IR: an extrusion axis `E`, with flow, temperature and fan as process parameters. Planar
  slicing comes first as a baseline (perimeters, infill, supports and retraction). Then
  non-planar layers follow the part surface, reusing the mesh, drop-cutter and surface code.
  Nozzle and carriage collision checks run against the part as printed so far. Until then the
  IR must not assume a subtractive process: process kinds, axes and verify rules stay generic.

## SolidCraft integration path
1. **Now: companion over files and the CLI.** SolidCraft exports STEP; we read it (and DXF/SVG),
   and we can call `solidcraft-cli` for sections and faces. Our CLI and, later, an MCP server
   let SolidCraft's in-app agent or the user drive CAM jobs.
2. **When SolidCraft plugins exist (issue #1221, WebAssembly with a WIT interface):** the
   libraries already build for wasm, so `campanion-geom`/`campanion-path`/`campanion-ir` can sit behind a plugin
   interface. Their current plugin design has no custom UI or preview, so the viewer stays
   separate until it does.
3. **Upstream:** offer the crates (renamed to their conventions) as SolidCraft crates. Their
   tracking issue #2652 sets the boundary as "STEP + manufacturing settings in, path segments
   (plus an optional post) out; the design never stores toolpaths". Our IR maps onto that
   path-segment idea, and we track their open CAM PRs (#2612 document core with `PathSegment`,
   #2819 2D contour/pocket crate, #4158 2.5D core) so we align types rather than compete.

Watch items: SolidCraft's kernel decision (#3992), plugin system (#1221), and which CAM PR lands
first. If upstream lands a CAM core, we contribute to it and keep only what it lacks here.

## Testing approach
- Analytic tests (exact offsets of circles, rectangles, slots).
- Property tests for geometry invariants.
- Hostile-input tests: NaN, huge counts, degenerate contours, never a panic.
- Verifier checks on generated toolpaths (gouges against the 2D target, rapids through material).
- Reference values only from analytic math or open tools, never from commercial CAM.

## Decision log
- 2026-10-21: Project started. Rust, MIT OR Apache-2.0, SolidCraft source rules. Name:
  **CAMpanion** (repo `campanion`, crates `campanion-*`, binary `campanion`). We avoid
  `solidcraft-*` names: SolidCraft reserves them, and its own open PR #2819 already proposes a
  crate called `solidcraft-cam`.
- 2026-10-21: Machine-neutral first; 2D path processing first; IR designed for up to 13 axes
  (hard cap 16).
- 2026-10-22: 2.5D milestone done; 3-axis surfacing (M7) started on triangle meshes in a new
  `campanion-mesh` crate.
- 2026-10-22: Order after M7: controller posts (M8) for everything built so far, then 4th axis
  and beyond.
- 2026-10-22: M8 done (v0.4.0). M9 is the 4th axis: indexing, wrapping, rotary posts, then
  continuous rotary finishing and viewer support.
- 2026-10-22: M9 done (v0.5.1). Indexing, wrapping, rotary posts, continuous rotary finishing and the
  viewer are in. `examples/rotary.job.json` is the rotary finish. Order from here: M10 turning,
  then M11 5-axis and advanced turning (mill-turn, live tooling, sub-spindle, B-axis), then M12
  additive and non-planar slicing.
- 2026-10-23: M10 done (v0.6.1) except the STEP section, which moves to M11 v0.7.0. M11 broken
  down into Parts A–C. Mill-turn posts: Haas first, then LinuxCNC. 5-axis milling stays in M11.
