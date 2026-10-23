#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# rust-toolchain.toml is honored by rustup. A distro cargo earlier on PATH would ignore it.
if command -v rustup >/dev/null 2>&1; then
    toolchain_bindir="$(dirname "$(rustup which cargo)")"
    export PATH="${toolchain_bindir}:${PATH}"
fi

cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo check -p campanion-geom -p campanion-mesh -p campanion-import -p campanion-path -p campanion-ir -p campanion-post -p campanion-turn --target wasm32-unknown-unknown
cargo deny check licenses
