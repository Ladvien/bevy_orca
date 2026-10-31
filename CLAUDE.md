# bevy_orca — notes for agents

ORCA local collision avoidance: the 2-D linear program over discs, on `bevy_math`'s `Vec2`. A function you call, not a plugin you register.

## Source of truth

This repository, [`bevy-coven/bevy_orca`](https://github.com/bevy-coven/bevy_orca), is canonical since 2026-10-23: it moved to Bevy 0.20 while `foundation_vs_slop/crates/bevy_orca` stays frozen on 0.19. Do not mirror over it with `scripts/mirror_crates.sh`. MIT OR Apache-2.0.

## Build and test

A leaf: `bevy_math` is the whole default dependency list, so it builds and tests on its own. The optional `serde` and `reflect` features add only derive impls on value types, and both are covered by CI.

`tests/leaf.rs` is the ratchet that keeps it that way: it reads `[dependencies]` (deliberately not dev-dependencies) and scans `src/` for engine crate names. Widening either list is a design decision, so it costs a deliberate edit there.

```sh
cargo test -p bevy_orca      # in the workspace
cargo test                   # in a standalone checkout
cargo test --all-features    # the optional derives, and the same tests
```

## The non-negotiable: no ECS, no plugin, no schedule

`bevy_math` stays the whole dependency list (`serde` and `bevy_reflect` are optional, and only for derives on data that already exists). The caller decides when avoidance runs and what counts as a neighbour — those are gameplay decisions. The moment this crate takes `bevy_app`, or any type from a particular game, it stops being a library.

**`Crowd` is a caller-owned batch helper; the caller still decides who is a neighbour (`sees`) and what is solid (`walls`).** It owns the spatial hash, the scratch buffers and the deterministic tie-break — the ring every host would otherwise write around `new_velocity_in` — and nothing else. It never looks at a game type, and it allocates nothing once warm because its buffers are `Vec`s it clears rather than drops.

**Reciprocity is the algorithm, not a detail.** Each side takes *half* the avoidance; that is what stops two agents on a head-on course from freezing or oscillating the way summed-force separation does. A change that makes one agent yield fully has replaced ORCA with something else.

**Attribution is load-bearing.** The implementation keeps RVO2's function names deliberately — see `NOTICE`. Do not rename them to something more idiomatic: the correspondence to the paper and to the reference implementation is what makes this auditable.

## Rules

- **Bevy 0.20 is pinned.** Read the vendored source (`~/.cargo/registry/src/index.crates.io-*/bevy_math-0.20.0/src/`), not bevy.org — that documents `main` and has been wrong for this pin more than once.
- **No `unwrap()`**, no `expect` on caller data, no indexing that can panic. This runs in an inner loop over every agent every tick.
- **One path per feature.** No fallbacks, no legacy shims, no stub placeholders. If the LP cannot find a feasible velocity, return that honestly — do not substitute a degraded answer.

## In the monorepo

The game reaches this as `crate::orca` in its monorepo, `foundation_vs_slop/crates/bevy_orca/`, which is frozen on Bevy 0.19. This repository is where changes land; the monorepo copy is not updated. Do not push over this repository from the monorepo's `scripts/mirror_crates.sh`.
