# Contributing to CAMpanion

Contributions are welcome: code, tests, sample files, docs and bug reports.

## Build and test

You need a Rust toolchain (the version is pinned in `rust-toolchain.toml`). Then run:

```sh
scripts/check.sh
```

This runs formatting, clippy, the tests, the licence and dependency checks, and the wasm build. A PR must pass it before review.

## Rules

Read [AGENTS.md](AGENTS.md) before writing code. In short:

- **Clean room.** Write everything from public specifications and first principles. Do not copy, port or translate code from other CAM or CAD projects. In particular, never use GPL, LGPL or AGPL code, or code derived from it.
- **Permissive dependencies only.** New crates must be MIT, Apache-2.0, BSD, ISC, Zlib or similarly permissive, and pass `cargo deny`. Say why in the PR.
- **Never crash.** Library code returns errors and never uses `unwrap`, `expect` or `panic`.
- **Machine-neutral core.** Machine- and process-specific behaviour belongs in posts or configuration, not in the geometry, path or IR crates.

## Licence

CAMpanion is dual-licensed under MIT OR Apache-2.0. Unless you say otherwise, anything you submit is licensed the same way, with no extra terms or conditions.

## Picking work

[docs/plan.md](docs/plan.md) is the roadmap. Each milestone lists its pieces, and anything not yet done is open. Bugs and gaps in the current milestone are good first contributions. So are new import fixtures, golden tests and verifier rules. For larger work, open an issue first and describe the approach.

## Pull requests

- Keep PRs small and focused on one feature or fix.
- Add tests: unit tests next to the code, plus fixture or golden tests for anything that produces toolpaths or G-code.
- Explain what changed and how you tested it.
