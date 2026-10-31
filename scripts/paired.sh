#!/bin/sh
# Paired timing of Crowd::solve: a base commit against the working tree, on a host whose cores are shared.
#
#   scripts/paired.sh <base-ref> [rounds]        e.g. scripts/paired.sh v0.2.1 400
#
# Builds four copies of the crate under four package names (a1 and a2 from <base-ref>, b1 and b2 from the
# working tree), then runs scripts/paired/main.rs, which solves the same frozen crowds with each in a
# rotating order and prints the median per-round ratio next to the floor two copies of identical code
# give. Needs no engine: the copies depend on bevy_math (nostd-libm) alone. PAIRED_DIR names the scratch
# directory (default /tmp/bevy_orca_paired); PAIRED_SCENES=open,rally and PAIRED_SIZES=100,1000 narrow it.
set -eu
ref=${1:?usage: scripts/paired.sh <base-ref> [rounds]}
rounds=${2:-400}
root=$(git rev-parse --show-toplevel)
work=${PAIRED_DIR:-/tmp/bevy_orca_paired}
rm -rf "$work"
mkdir -p "$work/src"

manifest() { # name
  cat > "$work/$1/Cargo.toml" <<TOML
[package]
name = "$1"
version = "0.0.0"
edition = "2024"

[features]
serde = []
reflect = []

[dependencies]
bevy_math = { version = "0.20.0", default-features = false, features = ["nostd-libm"] }
TOML
}

for name in orca_a1 orca_a2; do
  mkdir -p "$work/$name"
  git -C "$root" archive "$ref" src | tar -x -C "$work/$name"
  manifest "$name"
done
for name in orca_b1 orca_b2; do
  mkdir -p "$work/$name"
  cp -R "$root/src" "$work/$name/src"
  manifest "$name"
done

cat > "$work/Cargo.toml" <<TOML
[package]
name = "paired"
version = "0.0.0"
edition = "2024"

[dependencies]
bevy_math = { version = "0.20.0", default-features = false, features = ["nostd-libm"] }
orca_a1 = { path = "orca_a1" }
orca_a2 = { path = "orca_a2" }
orca_b1 = { path = "orca_b1" }
orca_b2 = { path = "orca_b2" }

[workspace]
TOML
cp "$root/scripts/paired/main.rs" "$work/src/main.rs"
cp "$root/Cargo.lock" "$work/Cargo.lock"

cd "$work"
CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$work/target}" cargo build --release --offline --quiet
"${CARGO_TARGET_DIR:-$work/target}/release/paired" "$rounds"
