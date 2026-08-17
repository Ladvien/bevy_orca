# bevy_orca — notes for agents

ORCA local collision avoidance: the 2-D linear program over discs, on `bevy_math`'s `Vec2`. A function you call, not a plugin you register.

## Source of truth

The source of truth for this crate is [`Ladvien/foundation_vs_slop`](https://github.com/Ladvien/foundation_vs_slop) at `crates/bevy_orca/`. If you are reading this in a standalone `Ladvien/bevy_orca` checkout, that is a read-only `git subtree split` mirror — **changes made here cannot be pulled back**. Make them upstream.

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

- **Bevy 0.19 is pinned.** Read the vendored source (`~/.cargo/registry/src/index.crates.io-*/bevy_math-0.19.0/src/`), not bevy.org — that documents `main` and has been wrong for this pin more than once.
- Consult Bevy documentation often. It can be found at codex_fs/offline_reference_docs/bevy-0.19-book/
- **No `unwrap()`**, no `expect` on caller data, no indexing that can panic. This runs in an inner loop over every agent every tick.
- **One path per feature.** No fallbacks, no legacy shims, no stub placeholders. If the LP cannot find a feasible velocity, return that honestly — do not substitute a degraded answer.

## In the monorepo

The game reaches this as `crate::orca` (`src/lib.rs:119`). The root `CLAUDE.md` and `TESTING.md` carry the project-wide rules; neither is part of this mirror.
