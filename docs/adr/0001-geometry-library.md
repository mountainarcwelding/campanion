# ADR 0001: 2D geometry library

- Status: Accepted
- Date: 2026-10-21

## Context

CAMpanion keeps lines and true circular arcs through offsetting and output (plan: exact geometry, G2/G3 not chord soup). The public model is ours: points, lines, arcs, contours, a tolerance, and `Result` errors. A dependency may implement offset or booleans only behind that model, so it can be swapped without rewriting callers.

Every dependency must be permissive (MIT, Apache-2.0, or another licence in `deny.toml`). `cargo deny check licenses` enforces that. This note records the licence string from each crate's packaged `Cargo.toml` (the file crates.io publishes; `license.workspace = true` is already resolved there). Copies live in the local registry after `cargo fetch`:

`~/.cargo/registry/src/index.crates.io-*/<crate>-<version>/Cargo.toml`

`i_overlay`, `geo`, and `geo-booleanop` were fetched the same way and are not workspace dependencies.

## Options

| | Own code | `cavalier_contours` 0.9.0 | `i_overlay` 9.1.0 | `geo` 0.33.1 | `geo-booleanop` 0.3.2 |
|---|---|---|---|---|---|
| Packaged `license` | MIT OR Apache-2.0 (this repo) | `MIT OR Apache-2.0` | `MIT OR Apache-2.0` | `MIT OR Apache-2.0` | `MIT` |
| Published | — | 2026-08-20 | 2026-10-14 | 2026-04-20 | 2020-06-27 |
| Maintenance | Ours to keep | Active. 0.9 focuses on offset robustness. One primary author. MSRV 1.88. Core crate `forbid(unsafe_code)`. | Active. 9.1.0 is current; long 4.x–9.x history. Powers `geo` booleans. | Active geospatial stack. Large dependency tree. | Last release June 2020. Depends on `geo-types` 0.6. Unmaintained. |
| wasm32 | Yes, pure Rust | Yes. Published crate is pure Rust and ships a wasm demo. `ahash` is built with `no-rng` because `getrandom` has no default wasm32 backend. Do not enable feature `unsafe_optimizations`. | Yes if default features stay empty. `rayon` is optional (`allow_multithreading`) and must stay off for wasm. Crate category includes `no-std`. | Poor fit. Default features include `multithreading` (rayon). `use-proj` links native PROJ, which wasm32 cannot build. A stripped build might compile and is still the wrong model. | Likely pure Rust, so it would probably build. Not worth carrying an unmaintained crate to find out. |
| Arcs | Full turns, centre/sweep and bulge | True line/arc polylines. Bulge is `tan(sweep/4)` and each span is limited to a half-turn (`bulge` in [-1, 1]). Offset and boolean keep arcs (round joins only). | Straight polygons. Round strokes are subdivided (the docs describe CORDIC arc subdivision). Booleans do not preserve circular arcs. | No circular-arc primitive. Booleans go through `i_overlay`. Buffer/offset emits polygons. | Martinez–Rueda on straight `geo` polygons. No arcs. |

Licence is not the discriminator. All four crates are on the permissive side of `deny.toml`. `geo-booleanop`'s old `robust` 0.1.2 dependency is `MIT/Apache-2.0` in its packaged manifest; we do not depend on it.

## Decision

1. **Kernel is ours.** Points, vectors, lines, arcs, contours, polylines, bounding boxes, tolerance, length, signed area, orientation, point-in-contour, and cleanup live in `campanion-geom` with no geometry crate underneath. That is the code that must match analytic areas (including arcs) and must turn hostile numbers into `GeomError` instead of panicking.
2. **Offset and booleans call `cavalier_contours` 0.9.0.** It is the only candidate that offsets and combines line/arc polylines without tessellating arcs, under a maintained MIT OR Apache-2.0 crate that already targets wasm32. The workspace dependency sets `default-features = false` and does not enable `unsafe_optimizations` or `serde`.
3. **Do not depend on `i_overlay`, `geo`, or `geo-booleanop`.** `i_overlay` is the fallback to revisit only if we explicitly accept chord tessellation; that would be a new ADR. `geo` is a geospatial stack whose boolean engine is `i_overlay` and whose default features fight wasm32. `geo-booleanop` is unmaintained, line-only, and stuck on `geo-types` 0.6.

## Seam

`cavalier_contours` types do not appear outside `crates/geom/src/backend.rs`.

- Callers use `Contour`, `Polyline`, `BooleanOp`, and `Tolerance`.
- `RegionEngine` is the swap point. `CavalierEngine` is the current implementation. `offset` and `boolean` construct that engine.
- Conversion splits any arc whose sweep is larger than half a turn so each bulge stays inside [-1, 1], which is the backend's documented limit. Our own `Arc` still stores a sweep up to one full turn.
- Positive offset distance is to the **left of travel**. On a counter-clockwise contour that is inward; on a clockwise hole it is outward.
- The backend documents panics for some spatial-index casts. The wrapper rejects non-finite distances and over-long inputs, and turns a backend panic into `GeomError::InvalidInput`.

Replacing the engine means a new `RegionEngine` impl and a new conversion in that one module. Cleanup, area, and containment do not need to change.

## Consequences

- `scripts/check.sh` builds `campanion-geom` for `wasm32-unknown-unknown` with this dependency, and `cargo deny check licenses` covers it and its crate graph.
- Boolean and offset quality is bounded by Cavalier Contours: two closed contours, round joins, half-turn bulges at the boundary, internal epsilons coarser than 1e-6 mm (`pos_equal_eps` 1e-5, `offset_dist_eps` 1e-4 in that crate's defaults). Analytic checks in this milestone cover a circle offset and a rectangle-minus-circle difference. Harder offset cases stay with the rest of M1.
- Own cleanup is what joins segments, drops duplicates and tiny spans, merges collinear lines and co-circular arcs, and sets outer contours counter-clockwise and holes clockwise.
