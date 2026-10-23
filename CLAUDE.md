# bevy_orca — notes for agents

ORCA local collision avoidance: the 2-D linear program over discs, on `bevy_math`'s `Vec2`. A function you call, not a plugin you register.

## Source of truth

This repository, [`Ladvien/bevy_orca`](https://github.com/Ladvien/bevy_orca), is canonical since 2026-10-23: it moved to Bevy 0.20 while `foundation_vs_slop/crates/bevy_orca` stays frozen on 0.19. Do not mirror over it with `scripts/mirror_crates.sh`. MIT OR Apache-2.0.

## Build and test

A leaf: `bevy_math` is the whole dependency list, so it builds and tests on its own.

```sh
cargo test -p bevy_orca      # in the workspace
cargo test                   # in a standalone checkout
```

## The non-negotiable: no ECS, no plugin, no schedule

`bevy_math` stays the whole dependency list. The caller decides when avoidance runs and what counts as a neighbour — those are gameplay decisions. The moment this crate takes `bevy_app`, or any type from a particular game, it stops being a library.

**Reciprocity is the algorithm, not a detail.** Each side takes *half* the avoidance; that is what stops two agents on a head-on course from freezing or oscillating the way summed-force separation does. A change that makes one agent yield fully has replaced ORCA with something else.

**Attribution is load-bearing.** The implementation keeps RVO2's function names deliberately — see `NOTICE`. Do not rename them to something more idiomatic: the correspondence to the paper and to the reference implementation is what makes this auditable.

## Rules

- **Bevy 0.20 is pinned.** Read the vendored source (`~/.cargo/registry/src/index.crates.io-*/bevy_math-0.20.0/src/`), not bevy.org — that documents `main` and has been wrong for this pin more than once.
- **No `unwrap()`**, no `expect` on caller data, no indexing that can panic. This runs in an inner loop over every agent every tick.
- **One path per feature.** No fallbacks, no legacy shims, no stub placeholders. If the LP cannot find a feasible velocity, return that honestly — do not substitute a degraded answer.

## In the monorepo

The game reaches this as `crate::orca` in its monorepo, `foundation_vs_slop/crates/bevy_orca/`, which is frozen on Bevy 0.19. This repository is where changes land; the monorepo copy is not updated. Do not push over this repository from the monorepo's `scripts/mirror_crates.sh`.
