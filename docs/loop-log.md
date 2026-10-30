# bevy_orca loop log

One section per iteration, appended. The loop command is `.omp/commands/build-loop.md`; rules live in `CLAUDE.md`.

## Iteration 0 — 2026-10-30 — loop setup

By the owner's session, not a loop pass. Added `.omp/commands/build-loop.md` (adapted from the bevy_stigmergy loop), this log and `CHANGELOG.md`. No code changed.

- **Published:** `bevy_orca` 0.2.0 (`Cargo.toml` on `main`, tag `v0.2.0` at `e7160df`). The next release diffs from `v0.2.0`.
- **Open issues:** none (`gh issue list --repo Ladvien/bevy_orca --state open` empty on 2026-10-30; #1, #3, #4, #5 closed). The external demand is bevy_emerge's family issue F11 (#107, crowds and paths): walkers never pass a wall cell, a narrow gap congests more than a wide one, deterministic for a seed.
- **Instruments that do not exist yet:** `examples/bench.rs` (section 2 of the command), and a capture knob (`ORCA_FRAMES` / `ORCA_SHOT`) on `circle_swap`. The first pass that needs either adds it.
- **Host:** big (Linux, Xvfb `DISPLAY=:89` from the unit, RTX 3090). `circle_swap` is the only example that needs the GPU; `head_on` and `wall_corridor` print to a terminal.
- **Gate (CI order, no wasm):** `cargo fmt --check`; `cargo clippy --all-targets --all-features -- -D warnings`; `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features`; `cargo build && cargo build --all-features && cargo build --release`; `cargo test && cargo test --all-features`; `cargo build --examples --all-features`.

### Ranked backlog

No open issue, so the backlog is the crate's own, ranked by section 4 of the command:

1. **`examples/bench.rs`, the measuring instrument.** Four scenes (open field, circle swap, two-way corridor, bottleneck) at 100 / 1 000 / 5 000 / 10 000 agents; median and p99 step time, agents per ms, candidates scanned per agent, the 1k→10k scaling exponent, and a 500-step hash at both member orders and 1 / 2 / default threads. Every performance and determinism gate depends on it. Done when the `orca: ...` lines print and the hashes match.
2. **Counter-flow without deadlock.** `head_on` already documents that an exactly collinear pair stalls and `Crowd`'s `nudge` breaks it; nothing measures a two-way corridor of many agents. Measure arrivals within 3x the straight-line time and agents stalled under 5% of preferred speed for over 2 s; fix the worst (a preferred passing side, lane formation) with a research note. Done when 100% of a 2x50-agent corridor arrives and no agent stalls.
3. **Static obstacles as RVO2's obstacle lines.** Walls are caller-built half-planes today (`walls` closure), so every host hand-rolls segment→half-plane maths; #107 needs walkers that never cross a wall cell. Add segment/polygon obstacles with RVO2's obstacle ORCA lines (names kept, `NOTICE` credited). Done when a test shows zero wall penetration through a gap two agents wide and the bottleneck congests more than a wide opening.
4. **Spatial hash cost.** `Crowd` keeps `std::collections::HashMap<(i32, i32), Vec<usize>>` with SipHash and per-cell `Vec`s. Measure candidates per agent and the hash's share of the step at 10 000 agents; replace with a flat counting-sort grid if it is the bottleneck. Done when agents per ms at 10 000 rises with hashes unchanged and `tests/alloc.rs` still passes.
5. **A parallel solve whose output is thread-count independent.** A 10 000-agent step today is serial unless the caller rewrites the ring. Offer the parallelism as a value the caller passes (no `rayon` in `[dependencies]` without a recorded leaf-ratchet decision). Done when the bench's hashes are equal at 1, 2 and the default thread count and the step's wall clock at 8 threads is quoted.
6. **`circle_swap` capture knob** and a corridor/bottleneck picture, so a visible change can be captured headless under `DISPLAY=:89`.
