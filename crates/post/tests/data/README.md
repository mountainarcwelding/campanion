# Example IR used by the post tests

`sample.ir.json`, `curves.ir.json`, and `dome.ir.json` are cutter-location programs for `examples/sample.job.json`, `examples/curves.job.json`, and `examples/dome.job.json`. The golden G-code under `crates/post/tests/golden/` is posted from these files, so they contain no dates or timestamps.

`plasma.ir.json` is a torch program built in `tests/cutting.rs` (pierce dwell, post dwell, and power). It is not produced from a job file. Regenerate it with `UPDATE_PLASMA_IR=1 cargo test -p campanion-post --test cutting`.

Regenerate them from the repository root:

```
cargo run -p campanion-cli -- run examples/sample.job.json -o /tmp/sample.ngc --ir crates/post/tests/data/sample.ir.json
cargo run -p campanion-cli -- run examples/curves.job.json -o /tmp/curves.ngc --ir crates/post/tests/data/curves.ir.json
cargo run -p campanion-cli -- run examples/dome.job.json -o /tmp/dome.ngc --ir crates/post/tests/data/dome.ir.json
cargo run -p campanion-cli -- run examples/indexed.job.json -o /tmp/indexed.ngc --ir crates/post/tests/data/indexed.ir.json
cargo run -p campanion-cli -- run examples/wrap.job.json -o /tmp/wrap.ngc --ir crates/post/tests/data/wrap.ir.json
```

`indexed.ir.json` and `wrap.ir.json` carry an A axis. The golden test posts them only with dialects that list that axis. `sample`, `curves` and `dome` are posted with every built-in that is not a lathe post.

`turn.ir.json` is a turning program (bar, one insert, one CSS pass). The golden test posts it with every lathe post.

Then rewrite the goldens. The test posts each program with `arc_tolerance` of `0.01` mm:

```
UPDATE_GOLDEN=1 cargo test -p campanion-post --test golden
```
