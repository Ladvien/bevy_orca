# Units, scope, features and determinism

Moved from the README on 2026-10-31, unchanged, when the README was brought to the [org documentation guide](https://github.com/bevy-coven/.github/blob/main/DOCUMENTATION.md).

`Vec2` is **the host's ground plane**, in the host's units: the crate never converts and never assumes a metric. `time_horizon` and `dt` must be in the same unit as your `vel` and `max_speed` — seconds and metres per second, or game-days and units per day, but not one of each. `neighbour_range`, `pos` and `radius` are in that same length unit. Left and right are the plane's own: a positive `CrowdParams::nudge` kicks clockwise in `(x, y)`, which is the walker's right when `+y` is up. A world whose ground plane is `(x, z)` may see that as its left, which is why the sign is yours to flip.

## Scope

Agent↔agent in `new_velocity_in`; static geometry as hard half-planes: either the `walls` you compute (a door that is shut this step: each a unit vector toward a solid plus the speed at which the agent may still close on it) or the ones `Obstacles` derives from polygons you built once. Neither is ever relaxed. The way round a building is not found here: it is your navigator's, and arrives as the preferred velocity. Final contact resolution is yours. So is where the work runs: the crate starts no thread, and `Crowd::begin` hands a step to whichever scheduler you have ([across threads](crowd.md#across-threads)).

## Features

- `serde` — `Serialize`/`Deserialize` on `Agent`, `CrowdMember` and `CrowdParams`. Also turns on `bevy_math/serialize`, which glam's impls need.
- `reflect` — `bevy_reflect::Reflect` on the same types, for a host that stores them in a Bevy world.

Both are off by default, and neither adds a type to the dependency graph a consumer did not ask for.

## Determinism

Pure arithmetic, no RNG, no interior mutability. Identical inputs give bit-identical outputs, which is why the parent project can hash its simulation state. The thread count is not an input: `Crowd::solve` and the same step split into any ranges over any number of threads (`CrowdStep::solve_range`) give the same bits, and `bench` hashes both at every thread count to prove it. Allocation happens only while a buffer grows: after the first call, `new_velocity_in`, `Crowd::solve` and a warm `CrowdStep::solve_range` allocate nothing, and `tests/alloc.rs` proves it with a counting allocator.

`bevy_math` is taken with `default-features = false, features = ["nostd-libm"]`, matching Bevy's own internal selection: the defaults would enable `glam/std` and swap glam's transcendentals from libm to std's, which is a behaviour change for anyone hashing transforms downstream.
