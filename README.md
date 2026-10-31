# bevy_orca

> ⚠️ **Vibe Coded.** Written by AI agents working from a human's direction, covered by tests rather than a line-by-line human review. Read the code before you rely on it for something you care about.

Walkers that steer around each other without colliding: ORCA (optimal reciprocal collision avoidance) as a plain library on `bevy_math`'s `Vec2`, for crowds in games.

[![crates.io](https://img.shields.io/crates/v/bevy_orca.svg)](https://crates.io/crates/bevy_orca) [![docs.rs](https://img.shields.io/docsrs/bevy_orca)](https://docs.rs/bevy_orca) [![CI](https://github.com/bevy-coven/bevy_orca/actions/workflows/ci.yml/badge.svg)](https://github.com/bevy-coven/bevy_orca/actions/workflows/ci.yml) [![Bevy 0.20](https://img.shields.io/badge/bevy-0.20-purple.svg)](https://bevyengine.org) [![licence](https://img.shields.io/badge/licence-MIT%20OR%20Apache--2.0-blue.svg)](#who-and-licence)

![Twenty coloured discs starting on a circle and crossing to the opposite side; they shear into a rotating vortex through the contested centre without colliding, then re-form on the far side](https://raw.githubusercontent.com/bevy-coven/bevy_orca/main/docs/circle_swap.gif)

*`examples/circle_swap.rs`: twenty agents each walk to the point opposite, so all want the centre at once; the vortex is not scripted, it is what twenty small linear programs solved at the same time look like.*

**Status:** usable · Bevy 0.20 · MSRV 1.97.1

**Why:** summed-force separation makes two agents on a head-on course shove equally and freeze or oscillate; under ORCA each steps aside by half, and this crate keeps that guarantee (never faster than `max_speed`, a finite answer for any input) with no ECS, plugin or schedule of its own.

## Quick start

```sh
cargo add bevy_orca
```

```rust
use bevy_math::Vec2;
use bevy_orca::{Agent, new_velocity};

let me = Agent { pos: Vec2::new(-1.0, 0.0), vel: Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };
let them = Agent { pos: Vec2::new(1.0, 0.0), vel: -Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };

let v = new_velocity(
    &me,
    Vec2::X,    // preferred velocity: whatever your navigator wants
    &[them],    // neighbours
    &[],        // wall half-planes: (unit normal toward the wall, max approach speed)
    2.0,        // time horizon, in the host's time unit
    1.0 / 60.0, // dt, in the same unit
    1.5,        // max speed
);
assert!(v.is_finite() && v.length() <= 1.5 * (1.0 + 1e-6));
```

## Try it

```sh
cargo run --release --example circle_swap
```

The GIF above. `head_on`, `wall_corridor` and `street` print to the terminal, with no window or GPU.

![The circle swap mid-crossing: twenty coloured discs packed into a turning ring at the centre, none overlapping](https://raw.githubusercontent.com/bevy-coven/bevy_orca/main/docs/media/circle_swap.png)

*The same run mid-crossing: the vortex at the contested centre.*

## What you can do

- **Steer one agent**: `new_velocity` returns the collision-free velocity closest to the one you wanted, and has one documented answer for every unusable input: [one agent](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/one-agent.md).
- **Steer a whole crowd**: `Crowd` finds neighbours with a grid, breaks ties by your stable key and allocates nothing once warm: [a whole crowd](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/crowd.md).
- **Give right of way**: a member's `priority` sets its share of every dodge, so a stall holder or a cart holds its line: [right of way](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/crowd.md#right-of-way).
- **Split a step across threads**: `Crowd::begin` and `CrowdStep::solve_range` run on your own pool with bit-identical results: [across threads](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/crowd.md#across-threads).
- **Walk round buildings and walls**: `ObstaclesBuilder` turns boxes, polygons and tile maps into RVO2's obstacle lines, built once: [static obstacles](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/obstacles.md).
- **Replay a simulation exactly**: no RNG and fixed arithmetic order, so the same inputs give the same bits on any thread count: [determinism](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/units-and-determinism.md).
- **Save its settings**: the `serde` and `reflect` features derive on `Agent`, `CrowdMember` and `CrowdParams`.

## Part of the Coven

`bevy_orca` is one of the [Bevy Coven](https://github.com/bevy-coven) crates: [`bevy_society`](https://crates.io/crates/bevy_society) moves its villagers with it, and [`bevy_stigmergy`](https://crates.io/crates/bevy_stigmergy) holds the fields the same crowd reads. Grimoire, the Coven's editor, walks crowds across a map with it to show where people will jam: [use it in Grimoire](https://bevy-coven.github.io/crates/bevy_orca.html#use-it-in-grimoire).

## Bevy compatibility

| `bevy_orca` | Bevy |
|---|---|
| 0.2 | 0.20 |

## Learn more

- [docs.rs/bevy_orca](https://docs.rs/bevy_orca): the API.
- [The book page](https://bevy-coven.github.io/crates/bevy_orca.html).
- [`docs/guide/`](https://github.com/bevy-coven/bevy_orca/blob/main/docs/guide/README.md): one agent, a crowd, obstacles, determinism and the examples, with every measurement the old README carried.
- [`examples/`](https://github.com/bevy-coven/bevy_orca/tree/main/examples) and [`CHANGELOG.md`](https://github.com/bevy-coven/bevy_orca/blob/main/CHANGELOG.md).

## Who and licence

Maintained by [@Ladvien](https://github.com/Ladvien). This repository is canonical: issues and pull requests belong at [github.com/bevy-coven/bevy_orca/issues](https://github.com/bevy-coven/bevy_orca/issues), and contributions follow the org's [CONTRIBUTING.md](https://github.com/bevy-coven/.github/blob/main/CONTRIBUTING.md).

MIT OR Apache-2.0, at your option. Derived from RVO2 (Apache-2.0): see [`NOTICE`](https://github.com/bevy-coven/bevy_orca/blob/main/NOTICE).
