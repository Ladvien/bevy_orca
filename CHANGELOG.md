# Changelog

All notable changes to this crate are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Fixed

- `new_velocity_in` (and so `new_velocity` and `Crowd::solve`) returned NaN or infinite velocities, despite "the result is always finite", for input its guards did not look at: an agent's own non-finite velocity or radius, a neighbour's non-finite or negative-radius state, a non-finite or non-unit wall, a `max_speed` above about `1.8e19`, a subnormal `dt` or `time_horizon`, and finite values whose arithmetic overflows `f32` (agents near `±f32::MAX`). A hostile sweep found 6 505 non-finite answers in 200 000 single solves and 1 709 in 20 000 crowd steps; it now finds none. Each case has one documented answer: an agent with unusable own state, or an unusable wall, holds still (`Vec2::ZERO`); an unusable neighbour, or one whose constraint overflows, is skipped; a `max_speed` whose square overflows gives `Vec2::ZERO`; a `dt` or `time_horizon` whose reciprocal overflows means no time passes. Finite, in-range input is bit-identical to before.

### Changed

- `Crowd::solve` leaves a member whose position, velocity or radius is unusable out of everyone's neighbour list, so it takes none of the `max_neighbours` places; such a member with a finite position is solved to `Vec2::ZERO` rather than to a NaN.
- A wall normal must now be a unit vector to `Vec2::is_normalized`'s tolerance, as the docs always said; a zero or scaled normal used to be a weaker or absent constraint and now holds the agent still.

## [0.2.1] - 2026-10-30

### Added

- `examples/bench.rs`: what one `Crowd::solve` step costs and what it gets right, in four scenes (open field, circle swap, two-way corridor with walls, bottleneck) at 100 to 10 000 agents — median and p99 step, agents per millisecond, neighbour candidates per agent, the 1k→10k scaling exponent, the deepest overlap and wall penetration, allocations inside the solve, and a state hash that must match across member order and the caller's parallel solve at 2 and the default thread count. An `arrive` section reports arrivals and stalls in the circle swap and a two-way corridor.

### Fixed

- `Crowd::solve` allocated whenever a warm crowd walked into ground it had not covered before (2 619 allocations in 480 steps of a 1 000-agent circle swap), and its per-step cost grew with the ground covered, because the neighbour grid kept a list for every cell it had ever seen. The grid is now a sorted `(cell, member)` index rebuilt every solve: its size is the crowd's, and a warm `Crowd` allocates nothing wherever the crowd walks. Neighbour lists, and so every solved velocity, are bit-identical to before.
- `Crowd::solve` panicked in a debug build (`attempt to add with overflow`) when a member's finite position was far enough out to saturate its grid cell, such as `x = 1e30`.
- The crate docs and `NOTICE` cited DOI `10.1109/TRO.2011.2120810` for "Reciprocal n-Body Collision Avoidance"; that DOI is "The Hybrid Reciprocal Velocity Obstacle". The ORCA paper is DOI `10.1007/978-3-642-19457-3_1`.
