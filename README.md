# bevy_orca

Optimal Reciprocal Collision Avoidance (ORCA) as a plain library: the 2-D linear program, over discs,
on `bevy_math`'s `Vec2`. No ECS, no plugin, no schedule — you call a function and get a velocity back.

> **This repo is a read-only mirror.** It is split out of
> [`Ladvien/foundation_vs_slop`](https://github.com/Ladvien/foundation_vs_slop) with `git subtree
> split`, history intact. Issues and PRs belong upstream.

## What it does

Given an agent's *preferred* velocity — whatever your global navigator wants, a flow field or a path
follower — ORCA returns the collision-free velocity **closest** to it, assuming every neighbour is
reasoning the same way and each side takes half the avoidance.

That reciprocity is the whole point. Summed-force separation makes two agents on a head-on course shove
equally and net zero, so they freeze or oscillate; under ORCA each steps aside by half.

```rust
use bevy_math::Vec2;
use bevy_orca::{Agent, new_velocity};

let me = Agent { pos: Vec2::new(-1.0, 0.0), vel: Vec2::X, radius: 0.5, avoids: true };
let them = Agent { pos: Vec2::new(1.0, 0.0), vel: -Vec2::X, radius: 0.5, avoids: true };

let v = new_velocity(
    &me,
    Vec2::X,          // preferred velocity
    &[them],          // neighbours
    &[],              // wall half-planes: (unit normal toward the wall, max approach speed)
    2.0,              // time horizon, seconds
    1.0 / 60.0,       // dt
    1.5,              // max speed
);
```

`avoids` is whether that neighbour is *also* running avoidance. When it is, the pair splits the
avoidance 50/50; when it is not — an idle agent holding ground — the mover takes the **full**
avoidance, so a stationary agent is not assumed to step aside and then walked through.

## Scope

Agent↔agent only. Static geometry enters just as the optional `walls` half-planes, each a unit vector
toward a solid cell plus the speed at which the agent may still close on it this step: the agent can
slide along or coast up to contact, but never accelerate *through*. Final contact resolution is yours.

## Determinism

Pure arithmetic, no allocation beyond the line list, no RNG, no interior mutability. Identical inputs
give bit-identical outputs, which is why the parent project can hash its simulation state.

`bevy_math` is taken with `default-features = false, features = ["nostd-libm"]`, matching Bevy's own
internal selection: the defaults would enable `glam/std` and swap glam's transcendentals from libm to
std's, which is a behaviour change for anyone hashing transforms downstream.

## License

MIT OR Apache-2.0, at your option. Derived from RVO2 (Apache-2.0) — see [`NOTICE`](NOTICE).
