# CAMpanion: rules for contributors and agents

CAMpanion is an open, machine-neutral CAM (toolpath) engine in Rust. It is meant to merge into
SolidCraft later, so it follows SolidCraft's source rules and conventions. When SolidCraft's own
`AGENTS.md` / `CONTRIBUTING.md` change in a way that matters here, update this file to match.

## Clean room and licences (non-negotiable)
- Licence: MIT OR Apache-2.0. Every dependency must be permissively licensed; `cargo deny check
  licenses` enforces the allow-list in `deny.toml`. **No GPL, LGPL or AGPL code, ever**, not even
  as an optional feature.
- **Never read, copy or link** GPL/LGPL/AGPL CAD/CAM sources: OpenCASCADE, FreeCAD (including its
  Path/CAM workbench and planegcs), OpenCAMLib, LinuxCNC source, SolveSpace, CadQuery/OCP,
  LibreCAD, QCAD, libdxfrw, PyCAM, and similar. Don't even read them. Running a GPL program as a
  separate tool to compute reference values is allowed only through its public interface, and
  nothing from it is linked, vendored or shipped.
- **No commercial CAD/CAM products** as a source: don't drive, script or extract data from them
  (Fusion, SolidWorks, CAMWorks, Mastercam, …). No test data, command lists, ids, posts or files
  made with them. Behaviour comes from public documentation, controller manuals, general
  machining practice, textbooks/papers, and user feedback.
- Do not copy code from other projects that link OCCT, PlaneGCS or OpenCAMLib.
- Code adapted from another MIT OR Apache-2.0 project is noted in `ATTRIBUTION.md` with source
  and licence.
- File formats (DXF, SVG, STEP, RS274/NGC) come from public specifications.

## Never crash
- No panics in non-test code: no `unwrap()`, `expect()`, `panic!`, `unreachable!`, `todo!`,
  `unimplemented!`; no `unsafe` (`unsafe_code = "forbid"`). Every crate root carries
  `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]`.
- Errors are `Result<T, E>` with the crate's `thiserror` error type. Unfinished features return
  "not supported yet: …".
- Input-derived numbers are hostile (job files, DXF/SVG/STEP, CLI arguments): use `get()` not
  `[i]`, check finiteness, cap sizes and loop counts.
- Geometry is checked with property tests (`proptest`) as well as analytic examples.

## Conventions (same as SolidCraft)
- Rust edition 2024, toolchain 1.95 (`rust-toolchain.toml`), `rustfmt.toml` max width 150.
- Units are millimetres and radians internally; Z is up; right-handed frames.
- Libraries below the CLI/UI must build for `wasm32-unknown-unknown` (SolidCraft has a web build).
- No UI toolkit below the UI crate; the UI (later) uses egui like SolidCraft.
- Quality gates before every commit: `scripts/check.sh` (fmt, clippy -D warnings, tests, wasm
  check, cargo-deny).

## How work is done
- Work in small, focused changes with tests; every change passes `scripts/check.sh` before review.
- One topic per branch and PR. Never force-push `main`. Never commit `plan/` or `target*/`.
- Don't use the SolidCraft name for our crates or binaries (`solidcraft-*` belongs to that
  project). Ours use the `campanion-` prefix (binary `campanion`) until a merge decides the final names.
