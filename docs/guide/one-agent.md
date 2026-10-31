# One agent: what ORCA does, and what it promises

Moved from the README on 2026-10-31, unchanged, when the README was brought to the [org documentation guide](https://github.com/bevy-coven/.github/blob/main/DOCUMENTATION.md).

Given an agent's *preferred* velocity — whatever your global navigator wants, a flow field or a path follower — ORCA returns the collision-free velocity **closest** to it, assuming every neighbour is reasoning the same way and each side takes half the avoidance.

That reciprocity is the whole point. Summed-force separation makes two agents on a head-on course shove equally and net zero, so they freeze or oscillate; under ORCA each steps aside by half.

```rust
use bevy_math::Vec2;
use bevy_orca::{Agent, new_velocity};

let me = Agent { pos: Vec2::new(-1.0, 0.0), vel: Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };
let them = Agent { pos: Vec2::new(1.0, 0.0), vel: -Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };

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

`avoids` is whether that neighbour is *also* running avoidance. When it is, the pair splits the avoidance 50/50 (unless their `priority` differs: see [right of way](crowd.md#right-of-way)); when it is not — an idle agent holding ground — the mover takes the **full** avoidance, so a stationary agent is not assumed to step aside and then walked through.

**The result is always finite, whatever you hand in.** A paused frame, a first frame or a time-scale of 0 hands in `dt = 0`, a physics blow-up hands in a NaN velocity, and a NaN written into a caller's `Transform` makes its agent vanish for good. So every input the solve cannot use has one documented answer:

- A `max_speed` that is not finite, not positive, so large its square overflows `f32` (above about `1.8e19`), or so small its square is not a normal number (below about `1.1e-19`) gives `Vec2::ZERO`; so does a non-finite preference.
- An agent whose own position or velocity is not finite, or whose radius is negative or not finite, gives `Vec2::ZERO`: it holds still for that step, and once you write the zero back its velocity is usable again. A radius of 0 is a point and is fine.
- A `dt` or `time_horizon` that is not finite, not positive, or so small its reciprocal overflows gives your preferred velocity, clamped — no time passes, so nothing can be constrained.
- A wall whose normal is not a unit vector (`Vec2::is_normalized`) or whose approach speed is not finite gives `Vec2::ZERO`. Walls are hard, so one that is not a half-plane holds the agent still rather than being dropped.
- A neighbour whose position or velocity is not finite, or whose radius is negative or not finite, is skipped — one corrupted agent does not move anyone else — and so is one so far away that its constraint overflows `f32`.

A finite preference is always usable, however long. One longer than `max_speed` is clamped to the speed disc along its own direction, and that includes one too long to square (above about `1.8e19`, which `offset / time_left` produces when `time_left` is tiny): the agent runs at full speed that way, it does not stop. `Crowd` follows the same rule for a member's `pref_vel`, and leaves its sideways nudge off a wish so large that the nudge cannot be added.

Finite, in-range input passes every one of these checks, so none of them changes a single bit of a normal solve.

**The result is never faster than `max_speed`**, to `f32` rounding (a relative `1e-6`), for any finite input: in a knot of forty agents, with a neighbour standing on the agent, with a navigator that hands in an own velocity of `1e10`. The speed disc is a constraint of the linear program, not a clamp after it. The reference implementation does not keep this promise: RVO2 (`snape/RVO2`) answers 14 805 of 1.9 million velocities over its `maxSpeed` on random packs of 2 to 40 agents, by up to 64×, and this crate did too (15 616, up to 77×) until every line was stored at its point nearest the origin (see `NOTICE`); packs of the same shape now give none over `1.000001×`, and `bench` prints `speed_max` for every scene so a regression shows.
