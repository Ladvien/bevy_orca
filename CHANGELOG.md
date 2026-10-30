# Changelog

All notable changes to this crate are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Fixed

- `new_velocity_in` (and so `new_velocity` and `Crowd::solve`) returned NaN or infinite velocities, despite "the result is always finite", for input its guards did not look at: an agent's own non-finite velocity or radius, a neighbour's non-finite or negative-radius state, a non-finite or non-unit wall, a `max_speed` above about `1.8e19`, a subnormal `dt` or `time_horizon`, and finite values whose arithmetic overflows `f32` (agents near `±f32::MAX`). A hostile sweep found 6 505 non-finite answers in 200 000 single solves and 1 709 in 20 000 crowd steps; it now finds none. Each case has one documented answer: an agent with unusable own state, or an unusable wall, holds still (`Vec2::ZERO`); an unusable neighbour, or one whose constraint overflows, is skipped; a `max_speed` whose square overflows gives `Vec2::ZERO`; a `dt` or `time_horizon` whose reciprocal overflows means no time passes. These guards change no bit of finite, in-range input (the speed-bound fix below does).
- `new_velocity_in` (and so `new_velocity` and `Crowd::solve`) answered faster than `max_speed`, despite "result magnitude is clamped to `max_speed`": by up to 10× on ordinary dense input, 9.3× in `Crowd::solve` on dense random crowds and many orders of magnitude with a fast own velocity. Two nearly parallel constraints made `linear_program3` project a line through their intersection, far from the speed disc, and the disc test and the answer then cancelled huge numbers against each other. Every half-plane is now stored at its point nearest the origin and projected lines are built from the two lines' distances to the origin. Over 3.45 million probe solves (dense, coincident, near-parallel, fast own velocity, hostile, crowd) the worst answer is `1.0000002 × max_speed`; before, 2 544 were over `1.001×`. RVO2 has the same flaw (48× on the same crowds).
- The README and `examples/circle_swap.rs` said RVO2's circle demo perturbs the preferred velocity to break symmetry; RVO2's `Blocks` and `Roadmap` demos do, its `Circle` does not. The README now also says what a nudge cannot break: a ring already constrained on its first step.

### Changed

- `Crowd::solve` leaves a member whose position, velocity or radius is unusable out of everyone's neighbour list, so it takes none of the `max_neighbours` places; such a member with a finite position is solved to `Vec2::ZERO` rather than to a NaN.
- A wall normal must now be a unit vector to `Vec2::is_normalized`'s tolerance, as the docs always said; a zero or scaled normal used to be a weaker or absent constraint and now holds the agent still.
- Every solved velocity changes in its last bits, so a host that hashes its simulation state sees new hashes once; they remain equal across member order and thread count. Cost: +0.3% to +2.0% user cycles per `Crowd::solve` on fixed 10 000-member states.
- A wall normal inside `Vec2::is_normalized`'s tolerance is divided by its length before use, so the wall is exactly `v · b <= approach` rather than scaled by `|b|²`.

## [0.2.1] - 2026-10-30

### Added

- `examples/bench.rs`: what one `Crowd::solve` step costs and what it gets right, in four scenes (open field, circle swap, two-way corridor with walls, bottleneck) at 100 to 10 000 agents — median and p99 step, agents per millisecond, neighbour candidates per agent, the 1k→10k scaling exponent, the deepest overlap and wall penetration, allocations inside the solve, and a state hash that must match across member order and the caller's parallel solve at 2 and the default thread count. An `arrive` section reports arrivals and stalls in the circle swap and a two-way corridor.

### Fixed

- `Crowd::solve` allocated whenever a warm crowd walked into ground it had not covered before (2 619 allocations in 480 steps of a 1 000-agent circle swap), and its per-step cost grew with the ground covered, because the neighbour grid kept a list for every cell it had ever seen. The grid is now a sorted `(cell, member)` index rebuilt every solve: its size is the crowd's, and a warm `Crowd` allocates nothing wherever the crowd walks. Neighbour lists, and so every solved velocity, are bit-identical to before.
- `Crowd::solve` panicked in a debug build (`attempt to add with overflow`) when a member's finite position was far enough out to saturate its grid cell, such as `x = 1e30`.
- The crate docs and `NOTICE` cited DOI `10.1109/TRO.2011.2120810` for "Reciprocal n-Body Collision Avoidance"; that DOI is "The Hybrid Reciprocal Velocity Obstacle". The ORCA paper is DOI `10.1007/978-3-642-19457-3_1`.
