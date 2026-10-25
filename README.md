# bevy_orca

> ⚠️ **Vibe Coded** — written by an AI agent working from a human's direction. It is used in a shipping game and covered by tests, but it has had no line-by-line human audit. Read it before you trust it.

Optimal Reciprocal Collision Avoidance (ORCA) as a plain library: the 2-D linear program, over discs, on `bevy_math`'s `Vec2`. No ECS, no plugin, no schedule — you call a function and get a velocity back.

**This repository is canonical.** It is where issues and pull requests belong; the split-out copies elsewhere are the ones that are frozen. MIT OR Apache-2.0.

![Twenty coloured discs starting on a circle and crossing to the opposite side; they shear into a rotating vortex through the contested centre without colliding, then re-form on the far side](https://raw.githubusercontent.com/Ladvien/bevy_orca/main/docs/circle_swap.gif)

That is `examples/circle_swap.rs`: every agent walking to the point directly opposite its own, so all of them want the exact centre at the same moment. **The rotation is not scripted.** Nobody is told to turn, there is no path planner and no flow field — each agent solves a small linear program for the velocity closest to what it wanted that no neighbour can object to, and the vortex is what falls out when twenty of them do that simultaneously.

## What it does

Given an agent's *preferred* velocity — whatever your global navigator wants, a flow field or a path follower — ORCA returns the collision-free velocity **closest** to it, assuming every neighbour is reasoning the same way and each side takes half the avoidance.

That reciprocity is the whole point. Summed-force separation makes two agents on a head-on course shove equally and net zero, so they freeze or oscillate; under ORCA each steps aside by half.

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
    2.0,              // time horizon, in the host's time unit
    1.0 / 60.0,       // dt, in the same unit
    1.5,              // max speed
);
```

`avoids` is whether that neighbour is *also* running avoidance. When it is, the pair splits the avoidance 50/50; when it is not — an idle agent holding ground — the mover takes the **full** avoidance, so a stationary agent is not assumed to step aside and then walked through.

**The result is always finite.** A paused frame, a first frame or a time-scale of 0 hands in `dt = 0`, and a NaN written into a caller's `Transform` makes its agent vanish for good. A non-finite or non-positive `max_speed` gives `Vec2::ZERO`; a non-finite preference gives `Vec2::ZERO`; a `dt` or `time_horizon` that is not finite or not positive gives your preferred velocity, clamped — no time passes, so nothing can be constrained.

## A whole crowd

Solving one agent at a time leaves every host to write the same ring around it: a spatial hash to find neighbours without an O(n²) scan, a reused scratch buffer, a deterministic tie-break, the wall buffer. [`Crowd`] is that ring, once.

```rust
use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams};

let members: Vec<CrowdMember> = people
    .iter()
    .map(|p| CrowdMember {
        key: p.id,                      // your stable id: breaks ties, picks the nudge side
        pos: p.pos,
        vel: p.vel,
        radius: 0.4,
        max_speed: 1.6,
        pref_vel: Some(p.wish),         // None = another system moves this one this step
    })
    .collect();

let params = CrowdParams::new(4.0);     // neighbour range, in your units
let mut crowd = Crowd::default();

let solved = crowd.solve(
    &members,
    &params,
    dt,
    |i, j| members[i].team == members[j].team,     // who is a neighbour: your rule
    |i, buf| fill_walls(members[i].pos, buf),      // what is solid: your rule
);
for (member, v) in members.iter().zip(solved) {
    if let Some(v) = v {
        step(member, *v);
    }
}
```

The caller still owns the two gameplay decisions — **who is a neighbour** (`sees`) and **what is solid** (`walls`). `Crowd` only decides how they are found and how they are solved. `key` is what makes the result a property of the world rather than of your iteration order: candidates sort by `(distance, key)`, and a head-on meeting steps to a side hashed from each member's own key, so two agents do not pick the same side and stall.

`Crowd` allocates nothing after its buffers are warm. `CrowdParams::new(range)` gives a 2-second horizon, at most 10 neighbours and a 5% nudge; set `nudge` to `0.0` to keep perfectly symmetric input perfectly symmetric.

## Units and plane

`Vec2` is **the host's ground plane**, in the host's units: the crate never converts and never assumes a metric. `time_horizon` and `dt` must be in the same unit as your `vel` and `max_speed` — seconds and metres per second, or game-days and units per day, but not one of each. `neighbour_range`, `pos` and `radius` are in that same length unit.

## Scope

Agent↔agent only. Static geometry enters just as the optional `walls` half-planes, each a unit vector toward a solid cell plus the speed at which the agent may still close on it this step: the agent can slide along or coast up to contact, but never accelerate *through*. Final contact resolution is yours.

## Features

- `serde` — `Serialize`/`Deserialize` on `Agent`, `CrowdMember` and `CrowdParams`. Also turns on `bevy_math/serialize`, which glam's impls need.
- `reflect` — `bevy_reflect::Reflect` on the same types, for a host that stores them in a Bevy world.

Both are off by default, and neither adds a type to the dependency graph a consumer did not ask for.

## Determinism

Pure arithmetic, no RNG, no interior mutability. Identical inputs give bit-identical outputs, which is why the parent project can hash its simulation state. Allocation happens only while a buffer grows: after the first call, [`new_velocity_in`] and [`Crowd::solve`] allocate nothing, and `tests/alloc.rs` proves it with a counting allocator.

`bevy_math` is taken with `default-features = false, features = ["nostd-libm"]`, matching Bevy's own internal selection: the defaults would enable `glam/std` and swap glam's transcendentals from libm to std's, which is a behaviour change for anyone hashing transforms downstream.

## Examples

```sh
cargo run -p bevy_orca --example head_on        # terminal: reciprocity, each side takes half
cargo run -p bevy_orca --example wall_corridor  # terminal: walls hard, neighbours soft
cargo run -p bevy_orca --example circle_swap    # the gif above. Needs a GPU.
```

`head_on` and `wall_corridor` print to the terminal — no window, no GPU, nothing to install. `head_on` walks two agents into each other and prints the lateral offset each takes, then repeats it with the neighbour holding ground so you can see the mover absorb the whole avoidance. `wall_corridor` adds wall half-planes and reports the deepest penetration, which stays at zero.

`circle_swap` is the picture. Two things in it are worth knowing before you tune this in your own game, because both were found by watching it fail:

- **Perfect symmetry deadlocks.** Identical agents, evenly spaced, all aimed at one point give the linear program no reason to prefer left over right, and the ring simply stalls. RVO2's own circle demo perturbs the preferred velocity for this reason, and `Crowd`'s `nudge` does it deterministically from each member's key.
- **Too much lookahead stalls it too, and less obviously.** At a 3-second horizon the pack braked for a congestion that had not happened yet and thereby caused it: velocity decayed geometrically — 0.80, 0.57, 0.41, 0.29, 0.21 — while the crossing ground to a halt a ring-radius in. At 1.5 they commit, and the shear resolves it. If your crowd is mysteriously slow, this dial is the first suspect.

## Used by

The rest of the family, so a host learns one shape:

- [`bevy_stigmergy`](https://crates.io/crates/bevy_stigmergy) — influence fields the same crowd deposits into and reads back.
- [`bevy_society`](https://crates.io/crates/bevy_society) — persons with daily lives whose movement is solved here.

## License

MIT OR Apache-2.0, at your option. Derived from RVO2 (Apache-2.0) — see [`NOTICE`](NOTICE).
