# Planar FFF slicing

This is the implementation plan for step 8 of `docs/processes-plan.md` (M12 baseline).
The ten decisions accepted there still apply. This document does not change them.

Path generation and slicing come first. There is no printer post in this work: verification
is through the IR, the simulator, and the deposited-strand model. A later step adds a
millimetre FFF dialect after the maintainer names the public specification (decision 9).

## Scope

In:

- Mesh (STL) to layers at Z, then layer contours with islands and holes.
- Perimeters by contour offset, top and bottom solid layers, sparse infill by zigzag.
- Travel with retraction when the travel is longer than a job-supplied threshold.
- Extruded length equals path length times flow, within tolerance.
- A deposited-strand model that checks nozzle clearance against strands already laid.
- Wave overhangs: arc-overhang style concentric waves that start at a supported edge and
  propagate outward over an unsupported region of the same layer.

Out of scope for this work, and left for later steps or separate contributions:

- Printer posts and E-word G-code.
- Supports, fuzzy skin, ironing, and non-planar layers (step 9).
- Invented printer numbers. Layer height, line width, flow, temperatures, fan, retraction
  length, travel threshold, perimeter count, solid-layer counts, infill spacing, nozzle
  diameter, and any overhang thresholds are job data. Tests use clearly labelled example
  values only.

STEP as a first-class solid source is deferred if `solidcraft-cli` has no cited tessellation
path to a triangle mesh. Today the CLI exports a planar face to DXF, or a lathe section.
Until a tessellated mesh is available, an FFF job takes an STL. A pre-tessellated mesh from
a STEP part is the same input.

## Milestones

Ship in reviewable PRs. Release as v0.17.0 when the core is coherent. Wave overhangs may
share that release if they stay reviewable; otherwise they follow in the next release.

1. **IR and mill-safe strands.** Optional non-negative deposited length on feed moves,
   omitted when absent so older files stay valid. Optional process parameters for flow,
   nozzle temperature, bed temperature and fan; a change is a new process event.
   Temperature waits are dwells already in the IR. The strand model records one swept
   strand per depositing move, in order, and runs only on a depositing program. An
   existing mill IR is the regression: mill, lathe and torch verify reports do not change.
   Subtractive mesh gouge does not run on an extruder program.
2. **Layer contours.** Intersect a triangle mesh with planes at Z, stitch segments into
   closed contours, and classify islands and holes. Analytic cases first (box, cylinder,
   prism), then randomized meshes.
3. **Path generation.** Perimeters, solid skins, zigzag infill, travels, and retractions
   from job data. Hook a job operation (mesh source, `Extruder` process) so `campanion run`
   writes an IR program. No post.
4. **Checks.** Extruded length against path length times flow. Retraction present on
   travels longer than the job threshold, and absent on shorter ones. Strand clearance
   as in milestone 1, including travel through an already laid bead.
5. **Wave overhangs.** Detect the unsupported region of a layer from the previous layer,
   generate wave paths there, and test with analytic overhangs. Same IR and strand rules.
6. **Docs and version.** Job-file documentation, changelog, and the v0.17.0 (or follow-on)
   version bump once the agreed slice of the work is on `main`.

## IR changes

No new process kind. `Extruder` already exists, and there is no extruder-only struct.

- A feed (`Linear`, `Arc`, `Helix`) may carry an optional deposited length in millimetres.
  Both this and per-move power are omitted when absent. The pose stays tool tip plus tool
  axis.
- Flow, nozzle temperature, bed temperature and fan are process parameters, not axes.
- E is not a new `AxisLetter`. It is the letter a later post will write for an indexed
  linear joint (`AxisName::Index`). Axis space integrates deposited length times flow into
  that joint. A mill program has no such component.
- A retraction is a pair of moves of that joint while the nozzle is still: a negative
  increment, then a recovery of the same length, with no deposited length. The strand
  model ignores those two moves.
- A deposited length marks a depositing move. The strand model is the first material
  model in the verifier and must not alter a program that does not deposit.

## Algorithms

**Slice at Z.** Intersect each triangle with the plane, stitch the resulting segments into
loops, and build `Region`s (outer contours plus holes) with the existing orientation and
containment tests. Coplanar faces (a cube sliced at its exact top or bottom) need a stated,
tested tie-break. Layer Z values come from job data: typically `z_min + n * layer_height`
for `n = 1, 2, …` while the plane still meets the solid. Do not invent a leftover-layer
rule that uses an implicit fraction of layer height; if a partial last layer is kept or
dropped, that rule is job data or a documented geometric predicate with tests.

**Perimeters.** Reuse `offset` / `offset_region`. Inset the island by `line_width / 2`, then
by further `line_width` steps for the job's perimeter count. Holes offset the other way
(into the material). Thin regions that collapse are dropped or reported, not silently
widened.

**Solid skins and sparse infill.** Reuse zigzag (export the existing pocket zigzag, or share
it as a path-crate entry). Solid top and bottom layers, and any solid skin inferred from
the layers above or below, fill the region inside the innermost perimeter at a job-supplied
solid spacing. Other layers use the job's sparse spacing. Infill stays inside that inner
offset; it does not invent a density-to-spacing conversion unless the job provides the
spacing.

**Travel and retraction.** A non-depositing travel longer than the job threshold is preceded
by the retract move and followed by the recovery move, nozzle still on both. A shorter
travel has neither. Depositing moves carry `path length × flow` as the deposited length.

**Strand model.** Each depositing move becomes one swept strand (a capsule or stadium whose
cross-section comes from job line width and layer height), recorded in order. During travel,
the nozzle shape is checked against strands already laid and must stay clear except where
the path meets or runs beside a bead on purpose (connected start, parallel beads at the
job line width, and the layer below the tip). Retract and recover are ignored. Holder and
carriage solids, when present, reuse the capsule and cylinder checks from the five-axis
holder work. The model is off for every non-depositing program.

**Wave overhangs (clean-room).** Implement from public algorithm *descriptions*, not from
slicer source. Do not copy GPL or other third-party code (including published Arc Overhang
scripts and their forks). Public descriptions, to be read and reimplemented, include:

- Steven McCulloch, [Arc Overhangs](https://hackaday.io/project/188704-arc-overhangs) and
  the algorithm summary in the project write-up: print 90° overhangs by wrapping filament
  in concentric arcs; an arc may start on an earlier arc; fill by growing arcs until the
  region is covered.
- [CNC Kitchen's account](https://www.cnckitchen.com/blog/arc-overhangs-a-new-way-of-printing-without-supports)
  of the same method: arcs grow from the still-supported edge into mid-air; adjacent beads
  overlap slightly so each is supported by the previous one; all motion stays planar.

Working description for this codebase:

1. Compare layer `n` to layer `n - 1`. The overhang is the current layer region minus the
   previous layer (minus any job-supplied support offset or threshold; there is no built-in
   default angle or distance).
2. The supported edge is the boundary between that overhang and the already-supported
   region of the same layer.
3. From that edge, generate concentric wave or arc fronts that advance into the overhang
   by about one line width per pass. Publicly, the centre of an arc lies on the current
   supporting front; the arc grows until it meets a boundary or a job-supplied maximum
   radius; the new front can branch; generation is breadth-first over the remaining
   overhang until the leftover gap is smaller than a job-supplied cutoff (for example a
   fraction of line width supplied in the job or test, never a hidden default).
4. Clip the waves to the layer outline. They replace sparse infill only inside the
   overhang; ordinary perimeters and supported infill stay as they are.
5. Unsupported islands with no supported edge in the layer cannot start a wave. They are
   left empty here (supports are out of scope) and the job or report should say so.

## Tests

- IR: deposited length and the new process parameters round-trip and are omitted when
  absent; older mill IR files load unchanged. `verify` on an existing mill IR (for example
  `crates/post/tests/data/sample.ir.json` and the CLI sample IR) is the regression for
  issue lists and totals.
- Slice: analytic box, cylinder and prisms at known Z (area, island count, hole count).
  Randomized meshes: every closed contour is simple, holes lie in their islands, and slice
  area is consistent with a sampled cross-section within tolerance.
- Paths: perimeter count and inset distance; solid layers at the job's top and bottom
  counts; zigzag stays inside the inner contour; extruded length vs path length times flow;
  retraction on and off around the job threshold, including equal-to-threshold.
- Strands: a travel that crosses a previously deposited same-layer bead fails; a travel
  that goes around it passes; parallel beads at the job line width and a connected
  continuation pass. A mill program is untouched.
- Waves: a rectangular ledge (supported half-plane plus an unsupported rectangle) grows
  fronts from the shared edge and covers the overhang without entering the supported
  interior. A smaller supported disk under a larger disk grows concentric arcs from the
  supported radius outward. An unsupported island with no seed edge produces no wave.
  Randomized overhang polygons: every wave segment lies in the overhang, and successive
  fronts stay within about one line width of the previous front.

## Risks

- The strand model changing a mill, lathe or torch report. Mitigation: the model is gated
  on a depositing program, and the mill IR regression runs in the first PR.
- Adding deposited length to every feed variant touches many `ClMotion` construction
  sites. Do that mechanically in the IR PR with a compile-and-unit-test pass there, not
  mixed into slicing.
- Mesh slices at exact coplanar top or bottom faces, and thin walls that collapse under
  offset, are easy to get silently wrong. They need explicit tests.
- Wave-front branching and leftover slivers can be fiddly. Analytic cases come first;
  randomized shapes exist to find holes, not to define the algorithm.
- STEP tessellation is not available on the current SolidCraft CLI path. Do not block the
  slicer on it.

## Deferred and decisions

Already deferred by this scope: printer posts, supports, fuzzy skin, ironing, non-planar
layers, and a built-in taper or overhang-angle chart (the core does not supply machine
data).

Items that need a decision before or during implementation:

- Whether v0.17.0 includes wave overhangs or only the slicing core, if the wave PR would
  make the release hard to review.
- Whether STEP-to-mesh waits on a cited `solidcraft-cli` tessellation command, or stays
  "supply an STL" for this step.
- The job-file operation name and required keys (no defaults that look like printer
  settings).
- The exact first-class names and units for the new process parameters (temperatures,
  fan, flow) and for the filament joint index in example machines.

No further process kind, and no change to the pose, should come up. If a post is requested
before the core checks pass, it stays a later step.
