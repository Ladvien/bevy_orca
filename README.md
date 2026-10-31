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

**The result is always finite, whatever you hand in.** A paused frame, a first frame or a time-scale of 0 hands in `dt = 0`, a physics blow-up hands in a NaN velocity, and a NaN written into a caller's `Transform` makes its agent vanish for good. So every input the solve cannot use has one documented answer:

- A `max_speed` that is not finite, not positive, so large its square overflows `f32` (above about `1.8e19`), or so small its square is not a normal number (below about `1.1e-19`) gives `Vec2::ZERO`; so does a non-finite preference.
- An agent whose own position or velocity is not finite, or whose radius is negative or not finite, gives `Vec2::ZERO`: it holds still for that step, and once you write the zero back its velocity is usable again. A radius of 0 is a point and is fine.
- A `dt` or `time_horizon` that is not finite, not positive, or so small its reciprocal overflows gives your preferred velocity, clamped — no time passes, so nothing can be constrained.
- A wall whose normal is not a unit vector (`Vec2::is_normalized`) or whose approach speed is not finite gives `Vec2::ZERO`. Walls are hard, so one that is not a half-plane holds the agent still rather than being dropped.
- A neighbour whose position or velocity is not finite, or whose radius is negative or not finite, is skipped — one corrupted agent does not move anyone else — and so is one so far away that its constraint overflows `f32`.

A finite preference is always usable, however long. One longer than `max_speed` is clamped to the speed disc along its own direction, and that includes one too long to square (above about `1.8e19`, which `offset / time_left` produces when `time_left` is tiny): the agent runs at full speed that way, it does not stop. `Crowd` follows the same rule for a member's `pref_vel`, and leaves its sideways nudge off a wish so large that the nudge cannot be added.

Finite, in-range input passes every one of these checks, so none of them changes a single bit of a normal solve.

**The result is never faster than `max_speed`**, to `f32` rounding (a relative `1e-6`), for any finite input: in a knot of forty agents, with a neighbour standing on the agent, with a navigator that hands in an own velocity of `1e10`. The speed disc is a constraint of the linear program, not a clamp after it. The reference implementation does not keep this promise: RVO2 (`snape/RVO2`) answers 14 805 of 1.9 million velocities over its `maxSpeed` on random packs of 2 to 40 agents, by up to 64×, and this crate did too (15 616, up to 77×) until every line was stored at its point nearest the origin (see `NOTICE`); packs of the same shape now give none over `1.000001×`, and `bench` prints `speed_max` for every scene so a regression shows.

## A whole crowd

Solving one agent at a time leaves every host to write the same ring around it: a spatial hash to find neighbours without an O(n²) scan, a reused scratch buffer, a deterministic tie-break, the wall buffer. [`Crowd`] is that ring, once.

```rust
use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams};

let members: Vec<CrowdMember> = people
    .iter()
    .map(|p| CrowdMember {
        key: p.id,                      // your stable id: breaks ties between equidistant neighbours
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

The caller still owns the two gameplay decisions — **who is a neighbour** (`sees`) and **what is solid** (`walls`). `Crowd` only decides how they are found and how they are solved. `key` is what makes the result a property of the world rather than of your iteration order: candidates sort by `(distance, key)`. The side a crowd passes on is not the key's but one number's: `CrowdParams::nudge` kicks every member toward the **same** side of its own wish, the right for a positive value (clockwise in your plane, which is where ORCA's own tie-break sends two agents walking straight at each other) and the left for a negative one, so two agents meeting head-on are kicked past each other and a crowd crossing a crowd turns one way instead of every way.

A member is `None` when it has no preference, its preference is not finite, or its position is not finite. Every other member gets `new_velocity_in`'s answer, so one whose velocity or radius is unusable, or whose `max_speed` is not finite, not positive, so large its square overflows or so small its square is not a normal number, is solved to `Vec2::ZERO` and holds still; and a member whose position, velocity or radius is unusable is nobody's neighbour and takes none of the `max_neighbours` places.

Reciprocity is assumed only of a neighbour this step moves. A member that is `None`, or that is solved to a stop because its `max_speed` leaves it no velocity (root a unit by setting it to `0.0`, and keep its wish if you like), is avoided in **full** by everyone near it; every other neighbour takes half. The solver's skip and the avoidance flag come from one test, so they cannot disagree.

`Crowd` allocates nothing after its buffers are warm, wherever the crowd walks and however tightly it packs: its neighbour grid is one entry per member, sorted by cell and rebuilt every solve, so its memory and its cost follow how many members there are and not how much ground they have covered; and of the candidates in range only the nearest `max_neighbours` are kept, found and ordered in place, so a member with hundreds of others in range pays for its ten neighbours rather than for ordering the rest. `CrowdParams::new(range)` gives a 2-second horizon, at most 10 neighbours and a 5% nudge to the right; set `nudge` to `0.0` to keep perfectly symmetric input perfectly symmetric, or negative to pass on the left.

## Units and plane

`Vec2` is **the host's ground plane**, in the host's units: the crate never converts and never assumes a metric. `time_horizon` and `dt` must be in the same unit as your `vel` and `max_speed` — seconds and metres per second, or game-days and units per day, but not one of each. `neighbour_range`, `pos` and `radius` are in that same length unit. Left and right are the plane's own: a positive `CrowdParams::nudge` kicks clockwise in `(x, y)`, which is the walker's right when `+y` is up. A world whose ground plane is `(x, z)` may see that as its left, which is why the sign is yours to flip.

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
cargo run -p bevy_orca --release --example bench  # terminal: what a crowd step costs, 100 to 10 000 agents
cargo run -p bevy_orca --example circle_swap    # the gif above. Needs a GPU.
```

`head_on` and `wall_corridor` print to the terminal — no window, no GPU, nothing to install. `head_on` walks two agents into each other and prints the lateral offset each takes, then repeats it with the neighbour holding ground so you can see the mover absorb the whole avoidance. `wall_corridor` adds wall half-planes and reports the deepest penetration, which stays at zero.

`bench` is the measuring instrument. It times one `Crowd::solve` step in five scenes — an open field of walkers, the circle swap, a two-way corridor with walls, a room draining through a gap two agents wide, and a rally of units hex-packed around one point with a 5 m neighbour range (about 200 candidates per member at 10 000 agents) — at 100, 1 000, 5 000 and 10 000 agents, and prints one line per case: median and p99 step time, agents per millisecond, neighbour candidates per agent, the deepest overlap and wall penetration, allocations inside the solve, and a hash of the final state that must be equal whether the members are given forwards, reversed, or solved by a caller's own parallel loop over `new_velocity_in`. `ORCA_SCENES=open,corridor` and `ORCA_SIZES=100,1000` narrow it.

`bench` also runs an `arrive` section: the circle swap (20 agents on a ring that jams, 20 on a wider one, 100 and 1 000), a two-way corridor and the bottleneck, each for three times the straight-line time, printing the fraction that arrived, how many stalled for over two seconds, and when half and nine tenths had arrived (`t50_s`, `t90_s`). `ORCA_SEED=n` shifts the members' keys for another crowd of the same scene, and `ORCA_SERIAL=1` times only the serial forward run.

`circle_swap` is the picture. Two things in it are worth knowing before you tune this in your own game, because both were found by watching it fail:

- **Perfect symmetry stalls, and so does a ring packed shoulder to shoulder.** Identical agents, evenly spaced, all aimed at one point give the linear program no reason to prefer left over right. `Crowd`'s `nudge` is the deterministic tie-break, and it is one side for the whole crowd: a crowd that all leans the same way crosses as one vortex. At the defaults (`CrowdParams::new(3.0)`, 0.3 m radius, 1 m of ring per agent) `bench` finds a 100-agent swap with nobody stalled and 90% through in 35 s, and a 1 000-agent swap with none stalled in 266 s; the reference RVO2, set up with the same radius, speed, horizon and neighbours, takes 63 s and 380 s. Neither frees a ring that is already as tight as it can be: 20 agents of radius 0.3 on a 3.2 m ring converge and stop where their discs touch (a ring of 1.92 m), because moving in closes the gap to both neighbours and the velocity closest to a wish that points in is zero. That is a fixed point of the solver, not a tie, and the cure is room (the same 20 agents on 1.9 m of ring each arrive in 15 s) or a navigator that gives a stuck agent a new wish. `bench` keeps the tight ring's row so the limit stays measured.
- **Too much lookahead stalls it too, and less obviously.** At a 3-second horizon the pack braked for a congestion that had not happened yet and thereby caused it: velocity decayed geometrically — 0.80, 0.57, 0.41, 0.29, 0.21 — while the crossing ground to a halt a ring-radius in. At 1.5 they commit, and the shear resolves it. If your crowd is mysteriously slow, this dial is the first suspect.

## Used by

The rest of the family, so a host learns one shape:

- [`bevy_stigmergy`](https://crates.io/crates/bevy_stigmergy) — influence fields the same crowd deposits into and reads back.
- [`bevy_society`](https://crates.io/crates/bevy_society) — persons with daily lives whose movement is solved here.

## License

MIT OR Apache-2.0, at your option. Derived from RVO2 (Apache-2.0) — see [`NOTICE`](NOTICE).
