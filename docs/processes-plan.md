# Further processes

Proposal, awaiting approval (2026-11-02). This is a planning document only. It does not schedule
implementation. `docs/plan.md` still stands, including M12 after M11, until the maintainer
accepts the decisions at the end.

This is the work that follows mill, lathe, mill-turn and five-axis milling. Router jobs are
already in scope. They are listed first because that gantry path is unfinished.

## Shared rules

- The core stays machine-neutral. Operations write the IR. Posts only read it. Pierce, preheat,
  retraction and scan timing are events and dwells in the IR. A post does not invent them.
- Process kinds stay the set in `crates/ir/src/process.rs`: spindle, torch, laser, extruder,
  knife, and `Other` with a short name. The 13-axis sample already carries `Other("waterjet")`
  on a second channel. A post looks up `process.<name>` and, today, falls back to `process.other`.
- Where a later section needs a value that changes along a cut, a feed move may carry an optional
  power (0..1, the same sense as process power) and an optional deposited length in millimetres.
  Both are omitted when absent, so older files stay valid. They are plain numbers. The pose stays
  the tool tip plus the tool axis.
- New joints are machine axes. The letters stay X Y Z A B C U V W. Further joints use an index.
  The cap is 16 axes and the target is 13. A post chooses the letter it writes, including E for
  a filament joint.
- Named on/off auxiliaries (abrasive, preheat, cutting oxygen, vacuum, dust) are `Other`
  processes. A post already maps that name to on and off lines. `Event::Custom` is a comment
  today. A program stop or a tool-length probe needs a post template (decision 8).
- In-process stock is not modelled (decision of 2026-11-02 in `docs/plan.md`). Mill and lathe
  checks stay as they are. A deposited-material model is introduced once, for planar FFF, and
  runs only on a program that deposits material.
- Posts are data files, cited from public manuals, with golden files and read-back, as in M8.
- V-carving, a start depth, inch output, a Mach4 post, DXF POLYLINE import and a UI are outside
  this proposal, left for separate contributions. Drag knife stays `ProcessKind::Knife` on the
  on-line path. Thread milling, adaptive clearing, scallop finishing, rest machining and
  subtractive stock simulation stay on the Later list in `docs/plan.md`.

## CNC routers

A router job is mill code on a gantry. Profiles, pockets, facing, drilling, on-line engraving,
leads, depth passes and profile tabs already run. `grbl` and `mach3` already post them,
and so do `linuxcnc`, `fanuc` and `haas`.

1. **IR.** No new process kind, event or axis. The diameter tool gains an optional cutter sense:
   upcut, downcut or compression. A compression tool also names the upcut length at the tip.
   Omitted, the offset stays the flat diameter offset used today. Onion-skin is an operation
   thickness: earlier passes stop short of the sheet, and the last pass cuts the skin. Nesting
   and hold-down are setup data: a sheet rectangle, part spacing, and keep-out rectangles for
   clamps. The IR still stores ordinary toolpaths. Tool-length probing uses the existing `Probe`
   event (expected contact and feed). Tabs stay the profile tabs already in the job file: bridge
   width, count or spacing, and an optional height.
2. **Posts.** `grbl` and `mach3` remain the gantry posts that exist. Every current post writes a
   probe as a comment, so the first new dialect is a LinuxCNC router post beside `linuxcnc`: a
   cited tool-length probe cycle, plus vacuum and dust-collection on/off. Further router controls
   wait on a public manual. Output stays millimetre.
3. **Simulation.** The mill verifier already checks gouges, low rapids and tabs. Nesting checks
   that every part lies on the sheet, that parts clear each other and the keep-outs by the tool
   radius plus the stated gap, and that the same part list always packs the same way. Onion-skin
   checks that only the last pass reaches full depth. The probe move is a feed toward the named
   contact, with the spindle off. In-process stock stays unmodelled: the sheet top is Z 0, the
   same rapid rule as a drawing. This process deposits nothing, so the deposited-material model
   stays unused.
4. **Reuse.** Mill operations, profile tabs, lead-in and cut ordering, fixture-style keep-outs
   from the collision checks, the `Probe` event, and the `grbl` and `mach3` posts.
5. **Size and risks.** M. Cutter sense and onion-skin are S. Nesting is the bulk. The packer is
   deterministic and makes no claim of optimal yield, so a test can lock the placement. A probe
   cycle has to match its manual; the golden is taken from that manual. Compensation stays half the diameter.

## Waterjet

Straight cutting is a kerf profile with a torch. Jet lead (tilt along travel) and taper (tilt
across the wall) share the later tilt step with oxy-fuel bevel. The path records the requested
angle.

1. **IR.** No new process kind, and no new axis for a straight cut. The jet is `Torch` or
   `Other("waterjet")`. Kerf stays a kerf tool: the computed offset is half the width. Pierce
   type (stationary, wiggle, low pressure, and further names a dialect cites) is a process
   parameter. Pierce delay, the on-during-delay toggle, pierce height and cut height already
   exist and remain the timing. Abrasive on and off is a named auxiliary, so the order can be
   jet on, pierce, abrasive on, cut, abrasive off, jet off. Quality chooses a feed, which the move
   already stores. A speed-dependent kerf is stored as the offset already applied. The tilt step
   adds a tool-axis change and a small head kinematics file (A and B on the gantry head).
2. **Posts.** A LinuxCNC waterjet dialect in the shape of `crates/post/posts/plasma.json`:
   `process_cycle` per cut, pierce dwell taken from the path, jet on/off, abrasive on/off.
   Height-control words stay empty until a manual cites them, as `thc` does on `plasma`. Rotary
   words wait for the tilt step.
3. **Simulation.** The compensated path sits half a kerf off the contour. Leads stay on the
   scrap side the operation named. The jet is off on every rapid. Abrasive is off whenever the
   jet is off, and on for the cut when the operation asked for abrasive. The tilt step checks
   head-angle limits and checks that the jet axis matches the requested wall angle within
   tolerance. In-process stock is the plate at Z 0 under the existing rapid rule. Deposited
   material stays unused.
4. **Reuse.** Kerf offset, profiles, leads, tabs, pierce and post delays, pierce and cut height,
   the `plasma` post pattern, and later the tool axis and head kinematics from M11.
5. **Size and risks.** M for straight cutting. Tilt is a further M, shared with oxy-fuel bevel.
   A wrong abrasive order consumes a nozzle, so the golden locks the event sequence. The tilt
   risk is shipping a lead angle as if it were a measured taper. The path records the angle it
   was given. A measured table, when a machine has one, is cited machine data.

## Wire EDM

2-axis wire is a profile. 4-axis UV is its own later step: the upper and lower guides follow
their own contours, and the wire is the straight ruling between them.

1. **IR.** No new process kind. The process is `Other("wire_edm")`. Wire offset is a kerf: wire
   diameter plus the spark gap for that pass. Skim passes are further profiles, each with its own
   offset, feed and power. Start holes are points the job supplies, and the lead-in begins there.
   Tabs reuse profile tabs so a slug stays attached. A program stop beside the tab is a one-shot
   code (decision 8), because `Custom` is only a comment today. The 2-axis path carries one guide.
   The 4-axis step adds an upper-guide position on the move. U and V are already the secondary
   linears parallel to X and Y, and the post writes them. The pose stays the work-plane point
   plus the wire direction. Two independent contours need that second position, so a 2-axis file
   omits it.
2. **Posts.** A LinuxCNC 2-axis wire dialect: wire on/off, flush as coolant or assist, power from
   the process power. The 4-axis post adds U and V on that dialect once the moves carry the upper
   guide. A second control waits on a public manual.
3. **Simulation.** Each pass sits at its own half-offset, on the side the operation named. Skim
   offsets move toward the finish and stop on it. The path starts at a start hole and stays open
   across a tab. Rapids are wire-off. For 4-axis, the segment between the guides is a straight
   ruling, both guides stay inside travel, and the ruling meets both contours. The spark gap is
   the width the pass named. Discharge is not simulated. In-process stock stays unmodelled.
   Deposited material stays unused.
4. **Reuse.** Profile offset, leads, tabs, feeds, process power, a dwell when a job needs a
   pause at spark-on, and the U and V letters. The two guides are one move. Channels stay for a
   second head, such as a second oxy-fuel torch.
5. **Size and risks.** 2-axis is M. 4-axis is L. The 4-axis risk is posting the wire as a rotary
   tool axis: the machine expects U and V, and a rotary post would write angles. Each skim names
   its own gap, because the gap changes with power. The verifier checks that named offset.

## Sinker EDM

1. **IR.** No new kind, and no new axis for a vertical ram. The process is `Other("sinker")`.
   The electrode is a tool: a diameter for a simple pin, or a mesh for a shaped electrode, plus
   a spark gap stored as stock to leave. Orbiting is a repeating pattern in the plane (circle or
   square) with an amplitude, written out as moves while the ram feeds in Z. A rotary orbital C,
   when the machine has one, uses the existing rotary axis. Power and on-time are process
   parameters. Flush is coolant or assist.
2. **Posts.** One vertical-ram dialect from a public manual. The orbit is expanded in the IR, so
   the post writes the moves and the burn on/off. A canned orbit, off by default, can wait.
3. **Simulation.** The electrode body, grown by the gap, stays outside the finish surface. The
   orbit amplitude stays inside the cavity the operation named. The burn feed is a plunge. A
   rapid stays out of the cavity. The check uses the finished cavity, as the mill gouge check uses
   the mesh. Subtractive in-process stock stays unmodelled. Deposited material stays unused.
4. **Reuse.** Plunge and depth passes, stock to leave, 2D offset for a circular or square orbit,
   the rotary axis when the orbit is a C move, and the mesh gouge check when the electrode is a
   mesh.
5. **Size and risks.** M for a cylindrical electrode and an expanded orbit. A shaped mesh
   electrode is L and follows only after the cylinder path is tested. The risk is a canned orbit
   the simulator cannot replay. Expanding the orbit keeps the IR equal to the path that ran.

## Additive and non-planar slicing

This is the M12 entry. Planar slicing is the baseline. Non-planar layers follow it, and they
wait until the planar checks pass.

1. **IR.** No new process kind. `Extruder` exists, and there is no extruder-only struct. Flow,
   nozzle temperature, bed temperature and fan are process parameters. A change is a new process
   event. Those quantities are not axes. A depositing feed carries an optional non-negative
   length of material. Axis space integrates that length, times flow, into an extra linear joint.
   The post writes the joint as E. The pose stays tip plus axis, so a mill program has no E
   component. A retraction is a pair of moves of that joint while the nozzle is still: a negative
   increment, then a recovery of the same length, with no deposited length. The strand model
   ignores those two moves. Temperature waits are dwells already in the IR. Verify stays generic.
   A deposited length marks a depositing move, and the subtractive gouge rules do not run on an
   extruder program.
2. **Posts.** The first post is a millimetre FFF dialect with absolute E, temperature and fan
   words, cited from a public specification the maintainer names (decision 9). Read-back includes
   the E words. LinuxCNC with that joint is the check that E is a post letter for an indexed axis.
   The existing `grbl` post has no such joint, so it is a later port.
3. **Simulation.** A planar layer is perimeters, infill and supports at one Z. Extruded length
   matches path length times flow, within tolerance. A travel longer than the job's threshold
   has a retraction. A shorter travel has none. The nozzle stays clear of a strand already
   deposited, except where the path crosses on purpose. That check is the deposited-material
   model: one swept strand per depositing move, in order. It is the first material model in the
   verifier, and it runs only for a depositing program. Mill, lathe, torch and laser reports stay
   as they are. Subtractive in-process stock stays unmodelled. Non-planar layers follow a mesh.
   The nozzle and the carriage are checked against the strands so far and against the part, using
   the capsule and cylinder shapes from the holder check.
4. **Reuse.** Contour offset for perimeters, pocket zigzag for infill, the mesh, drop-cutter and
   surface code for non-planar layers, process on/off, the extra-axis machine model, and the
   collision solids from the five-axis work.
5. **Size and risks.** Planar is L. Non-planar is L. Islands, thin walls and supports are the
   bulk, and they are larger than the IR change. The deposited model has to leave a mill program
   alone. An existing mill IR is the regression. A missing temperature dwell is a code-generation
   error and is covered by a golden. Non-planar collision is only as complete as the strands
   recorded for the moves before it.

## Laser engraving and raster

Vector laser cutting is already in reach: `ProcessKind::Laser`, a kerf or an on-line engrave,
pierce and post timing, and `crates/post/posts/grbl_laser.json`. That post arms dynamic power
with `M4 S0`, writes `M4 S{power}` and `M5`, scales IR power 0..1 by `power_scale` 1000 onto
S0..S1000, drops Z, and uses `process_cycle` per cut. A missing power is posted as full scale.
A raster has to set every segment, so a missing value cannot become S1000 inside the image.

1. **IR.** No new process kind and no new axis. A feed move may carry the optional power from
   the shared rules, one value per scan segment, mapped from grey. Overscan is path past the
   image at power 0. Scanning offset is a distance the operation applies in the CAM, one way
   for each scan direction, so the IR already holds the shifted burns. The bitmap is an input
   beside the job. The IR stores the moves and the powers.
2. **Posts.** `grbl_laser` is the first post. Each segment's S word is the scaled power,
   including S0 on the overscan. A constant-power dialect is a later post, and only from a
   public manual.
3. **Simulation.** Power follows the grey map the operation names (a linear map, a gamma, or a
   table). Scan lines cover the image at the stated step and stay inside the image bounds plus
   the overscan. Opposite passes agree on the image once the scanning offset is applied. A grey
   the map sends to zero is power 0. There is no depth-of-cut model. In-process stock stays
   unmodelled. Deposited material stays unused. Focus height may stay on the pose. `grbl_laser`
   still drops the Z word.
4. **Reuse.** Zigzag from pockets and from surface raster, on-line engrave, kerf offset for a
   vector cut, and the `grbl_laser` dialect, including the per-cut cycle and pierce dwell.
5. **Size and risks.** M. Per-move power has to round-trip through JSON and read-back, or the S
   words drift. Scanning offset is a calibration the job states. The planner applies that number
   and does not invent one. A large bitmap makes a large program, so the operation refuses an
   image past a stated resolution.

## Oxy-fuel

1. **IR.** No new kind, and no new axis for one vertical torch. The cutting flame is `Torch`.
   Preheat is a named auxiliary, ordered around the existing pierce dwell: preheat on, dwell,
   cutting oxygen on, cut, cutting oxygen off, preheat off. Cutting oxygen is its own auxiliary,
   so it stays distinct from coolant and from laser assist. Pierce height and cut height already
   exist. Several torches are several channels, one torch each, with the existing sync points.
   Spacing between torches is machine data on those channels. Bevel is a tool-axis tilt and lands
   in the shared tilt step with waterjet lead and taper.
2. **Posts.** A LinuxCNC oxy-fuel dialect: preheat, cutting oxygen, off, and the pierce dwell
   from the path. Several torches use the channel-section shape of `generic_lathe_2ch` (a header
   per channel and a sync wait). The wait codes come from that dialect's manual. Bevel words wait
   for the tilt step.
3. **Simulation.** Cutting oxygen turns on only after the preheat dwell the job stated, and it
   is off on every rapid. Channels meet at the named sync ids. The tilt step checks the bevel
   angle against the head limits and against the requested wall. There is no heat model and no
   slag model. The plate uses the Z 0 rapid rule, as a waterjet plate does. Deposited material
   stays unused.
4. **Reuse.** Torch events, pierce and post delays, pierce and cut height, the `plasma` post
   shape, channels and sync, and later the tool axis and head kinematics.
5. **Size and risks.** S for one torch with preheat and pierce. M with several torches and the
   sync report. Bevel belongs to the shared tilt step. A preheat shorter than the job's minimum
   is a code-generation error, and the golden records the dwell.

## Robotic welding

1. **IR.** No new process kind. Arc on/off is `Other("weld")`. The weld post lists that name and
   accepts it. Torch angles are the tool axis. Lead and work angle use the lead/tilt frame from
   the five-axis strategies: the seam tangent is the travel direction, and the seam normal is the
   reference. Weave (amplitude, step, dwell at the side) is computed in the CAM. The IR holds the
   woven polyline. Wire feed is a process speed, length per minute, on arc-on. A welder holds a
   rate. The post writes that rate. It is a different quantity from the FFF filament joint.
   Seams are an operation input. The first source is an open contour the importers already read.
   A mesh edge is the next source. A STEP edge waits on the exporter. `solidcraft-cli` currently
   returns sections and planar faces.
   The robot solver is new. The five-axis model is one or two rotaries on a table or a head, and
   a six-joint arm is a serial chain beside that model. The five-axis solver stays as it is.
   External axes (a linear track, a positioner rotary) are ordinary machine axes. They share the
   arm's channel when the controller interpolates them together, and they use a synced channel
   when it does not. Six arm joints, a track and a two-axis positioner are nine axes, inside the
   target of 13 and the cap of 16.
2. **Posts.** One dialect whose public format the maintainer names (decision 9). Where the
   controller does not speak G-code, the post is a point list with process flags. It still
   consumes only the IR. Joint goldens are analytic and are kept apart from the post text.
3. **Simulation.** The torch axis follows the seam at the stated lead and work angles. Weave
   stays inside the amplitude. The arc is off on air moves. A pass that names one wire-feed rate
   keeps that rate on every arc-on. The serial solver names the first pose it cannot reach and
   checks joint limits. Property tests compare a known pose with forward(inverse(pose)), as the
   five-axis tests do. A later pass may keep the torch clear of a weld bead. That bead uses the
   deposited-strand model from planar FFF. Until a job asks for the bead check, verification is
   the seam frame and the joints. Subtractive in-process stock stays unmodelled.
4. **Reuse.** Tool axis and lead/tilt, process on/off, channels and sync, collision capsules,
   open-contour import, and the deposited-strand model once planar FFF has it. The serial solver
   is the new piece.
5. **Size and risks.** L. This is the furthest process from the machines already modelled. The
   main risk is folding the arm into the five-axis solver. That solver describes a table and a
   head. Weave stays in the path, so the IR remains the curve that can be checked. External-axis
   calibration is machine data. Seam extraction stops at imported contours, then mesh edges.

## Recommended order

Each step is one milestone, except steps 8 and 9, which are the two stages of M12. A
code-generation error on an earlier step is fixed before the next step starts (design principle 7
in `docs/plan.md`).

### 1. Router gaps

Mill operations, tabs, leads, `grbl`, `mach3` and the probe event are already shipped. This step
finishes that gantry path: cutter-sense data, a deterministic sheet packer, onion-skin passes and
a LinuxCNC router dialect. Containment, spacing, skin height and tab bridges are geometry tests
and need no machine. The risk is the probe cycle, which read-back against a cited manual covers.

### 2. Laser raster

`ProcessKind::Laser` and `grbl_laser` already post a vector cut, including M4 dynamic power, the
0..1 scale, dropped Z and a per-cut cycle. Raster adds a bitmap and a power that changes along a
zigzag, which pockets and surface finishing already order. A synthetic ramp checks the grey map,
overscan at power 0 and the scanning offset, and read-back checks the S words. The new IR surface
is optional per-move power. It round-trips in JSON and leaves the pose alone. No material model
is required.

### 3. Waterjet, straight cuts

A kerf tool, a profile, leads, pierce delay, pierce height and the `plasma` post already describe
a straight cut. Abrasive and pierce type are event order and parameters on top. Offset, lead
side, jet-off rapids and abrasive order are analytic tests, with no machine and no taper model.
The risk is the event order, which a golden locks. Tilt stays in its own step.

### 4. Oxy-fuel

Torch events, heights and the plasma-shaped post are the waterjet path with a longer, ordered
preheat. A single torch is a timeline test: dwell, then cutting oxygen, then off on rapids.
Several torches reuse channels and sync, which mill-turn already runs. The risk is a short
preheat. The job states the minimum dwell, and the golden records it. Bevel shares the tilt step.

### 5. Wire EDM, 2-axis

This is a profile with a kerf-sized offset, a lead from a supplied start hole, tabs, and skim
passes that are further offsets. That path code exists. Tests check the offsets, the open tab
and wire-off rapids, using the spark gap the pass names. The risk is a skim whose gap changes
with power and is left unnamed. This step stores no upper guide.

### 6. Tilting kerf heads

Waterjet lead and taper, and oxy-fuel bevel, are one implementation: a tool axis along the path
and a small head kinematics file. M11 already solves that shape and tests forward(inverse(pose)),
limits and the singular cone. The step follows the straight torch posts, so the gas sequence is
already golden. Tests check travel limits and the geometric wall. The path stores the requested
angle. A measured chart, when a machine has one, is cited machine data.

### 7. Sinker EDM

A vertical burn with an orbit is a plunge plus a 2D pattern, on top of depth passes and offset.
The electrode is a body, so the check is that solid against a known cavity. A cylinder and a
circular orbit are analytic and need no machine. The risk is a canned orbit the verifier cannot
replay, so the orbit is expanded into moves. A shaped mesh electrode is a later, larger follow-on.
The finished surface is the check. Burned-so-far stock stays unmodelled.

### 8. Planar FFF (M12 baseline)

`Extruder` exists. Slicing does not. Perimeters and infill reuse offset and zigzag, while
islands, supports, temperature and retraction are new. That is why this step is L and sits after
the smaller sheet and burn processes. It is testable without a printer: layer contours, extruded
length against path length, retraction on long travels, and nozzle clearance against strands
already laid. The deposited-strand model starts here, which is also why non-planar layers and a
weld bead come later. The risk is that model changing a mill report. It runs only on a depositing
program, with an existing mill IR as the regression. E is the post's letter for an indexed
linear joint. The pose stays tip plus axis.

### 9. Non-planar FFF (rest of M12)

This needs the planar slicer, the deposited strands, and the mesh, drop-cutter and surface code.
Those are available only after step 8 and the milling work already shipped. A plane and a
cylinder are the analytic surfaces: the layer follows them within tolerance, and the nozzle
clears the deposit. The risk is a collision check against a strand the model dropped. M12 is
these two stages, in this order, with no other process between them.

### 10. Wire EDM, 4-axis UV

2-axis offset, skim, start holes and tabs already have tests, and U and V already exist as axis
letters. The new piece is the upper-guide position on the move. A known taper is a straight
ruling between two contours, both guides stay inside travel, and read-back shows U and V. No
earlier step reads that second position, so this IR change waits until M12 has landed. The risk
is encoding the wire as a rotary axis. Two stored positions carry the two profiles.

### 11. Robotic welding

This is the furthest from what is built. Seams start as open contours, torch angles reuse
lead/tilt, arc on/off reuses process events, and wire feed is a speed parameter. Weave is a path
offset a test can bound. External axes fit the joint list and the 13-axis target. The serial
chain is new and stays beside the five-axis solver, because that solver describes a table and a
head. Tests without a robot are analytic poses, a named unreachable point, arc-off on air moves,
and weave inside its amplitude. A bead check reuses the FFF strands. The step is last so each
reused piece already has a golden.

## Decisions needed

The maintainer approves or rejects each item. Until that happens, this file is a proposal.

1. **Order.** Accept the eleven steps above. M12 planar FFF is step 8 and non-planar FFF is
   step 9. `docs/plan.md` currently starts M12 immediately after M11.
2. **Process kinds.** Add none. Plasma, waterjet and oxy-fuel use `Torch` plus named `Other`
   auxiliaries (abrasive, preheat, cutting oxygen, vacuum, dust). Wire, sinker and weld use
   `Other` with a short name. `Laser` and `Extruder` stay as they are.
3. **Per-move numbers.** A feed move may carry an optional power (0..1) and an optional
   non-negative deposited length. Both are omitted when absent. The pose does not change.
4. **Filament axis.** E is the letter a post writes for an indexed linear joint. E is not a new
   `AxisLetter`, and the pose does not store it. Welding wire feed is a process speed, not that
   joint.
5. **Deposited material.** The model starts with planar FFF, records one swept strand per
   depositing move, in order, and runs only on a depositing program. Mill, lathe and torch
   verification stay as they are.
6. **Wire guides.** The 2-axis step adds no field. The 4-axis step stores an upper-guide
   position and posts U and V. The two guides share one move.
7. **Kinematics.** The robot solver is a new serial chain. The five-axis solver remains one or
   two rotaries on a table or a head.
8. **Stops and probes.** Add a post template for a named program stop, and a post template for
   the existing `Probe` event, so a wire stop and a router tool-length probe emit cited codes.
   Where a post has no template, the current comment remains. New dialects list the auxiliary
   names they accept and refuse any other name. Existing posts keep their `other` fallback.
9. **First posts.** LinuxCNC router beside `linuxcnc`, `grbl_laser` for raster, LinuxCNC
   waterjet beside `plasma`, LinuxCNC oxy-fuel, LinuxCNC 2-axis wire, then one published
   millimetre FFF dialect, one published sinker dialect, and one published robot format. The
   maintainer names the FFF, sinker and robot documents before those steps start.
10. **Tilt data.** Waterjet lead, waterjet taper and oxy-fuel bevel record the requested tool
    axis. A measured chart is optional cited machine data. The core does not supply one.
