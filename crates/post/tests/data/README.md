# Example IR used by the post tests

`sample.ir.json`, `curves.ir.json`, and `dome.ir.json` are cutter-location programs for `examples/sample.job.json`, `examples/curves.job.json`, and `examples/dome.job.json`. The golden G-code under `crates/post/tests/golden/` is posted from these files, so they contain no dates or timestamps.

Regenerate them from the repository root:

```
cargo run -p campanion-cli -- run examples/sample.job.json -o /tmp/sample.ngc --ir crates/post/tests/data/sample.ir.json
cargo run -p campanion-cli -- run examples/curves.job.json -o /tmp/curves.ngc --ir crates/post/tests/data/curves.ir.json
cargo run -p campanion-cli -- run examples/dome.job.json -o /tmp/dome.ngc --ir crates/post/tests/data/dome.ir.json
```

Then rewrite the goldens. The test posts each program with `arc_tolerance` of `0.01` mm:

```
UPDATE_GOLDEN=1 cargo test -p campanion-post --test golden
```
