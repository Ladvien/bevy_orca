# Unusable input — what the solver answers when the caller's data is not a number

`new_velocity_in` documented "the result is always finite", and `Crowd::solve` inherits it. Iteration 1's hostile sweep found that promise broken for state the guards did not look at: an agent's own velocity or radius, a neighbour's state, a wall, and finite values so large that the arithmetic overflows. This note decides what each of those returns and why, before any code.

## Sources

- **Model:** `10.1109_robot.2008.4543489` (van den Berg, Lin & Manocha, RVO, ICRA 2008: an agent's velocity obstacle is defined from its position, velocity and radius); `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas, Game AI Pro 2 ch. 19: ORCA constraints and what dropping one costs).
- **Shipping game / engine:** Recast/Detour, `Detour/Source/DetourNavMeshQuery.cpp` (`recastnavigation/recastnavigation` on GitHub, `main`, read 2026-10-30; not in the corpus): every query that takes a position or a radius refuses a non-finite one with `DT_FAILURE | DT_INVALID_PARAM`. `DetourCrowd.cpp` itself validates nothing. `gameaipro2-ch19-guide-anticipatory-collision-avoidance` again for what a dropped constraint means in a shipped crowd.
- **Technical:** `book_real_time_collision_detection_ericson` (Ericson, ch. 11 "Numerical Robustness": INF and NaN in a geometric test); `PBR3_03_Shapes` and `PBR3_11_Volume_Scattering` (Pharr, Jakob & Humphreys, *PBRT* 3rd ed.: NaN comparisons and `0 · ∞`); glam 0.33's `Vec2::is_normalized` (vendored, `glam-0.33.12/src/f32/vec2.rs:618`: `abs(length_squared() - 1.0) <= 2e-4`).

## Craft principle

ORCA's constraint for a neighbour is built from three numbers per agent and nothing else:

> "Given n (planar translating) agents A1,...,An, each agent Ai has a current position p i (defined by its reference point), a current velocity v i, a goal location g i, and a preferred speed" — `10.1109_robot.2008.4543489`, the section on navigating many agents (p. 5)

So an agent whose position, velocity or radius is not a number has no velocity obstacle at all: there is nothing to put in the linear program. Dropping a constraint is not free — "When constraints are dropped in this manner, the resulting motion is no longer guaranteed to be collision-free for that agent, for that time step. However, in practice, this typically results in only minor, fleeting collisions." (`gameaipro2-ch19-guide-anticipatory-collision-avoidance`, the ORCA section, p. 12) — which is why only a *neighbour* is ever dropped, never a wall, and only when its constraint does not exist.

IEEE-754 makes the failure silent rather than loud. "Like infinity values, NaNs have well-specified semantics: for example, any logical comparison involving a NaN always evaluates to false" (`PBR3_03_Shapes`, the ray–bounds intersection test), and "multiplying Infinity by zero results in the floating-point 'not a number' (NaN) value, which propagates throughout all computations that use it" (`PBR3_11_Volume_Scattering`, `HomogeneousMedium::Tr`). A NaN in one ORCA line passes every `> 0.0` test in `linear_program2` as "satisfied", and a NaN discriminant passes `discriminant < 0.0` as "feasible" — which is exactly how a NaN reached the result.

## How shipping games do it

Detour, the crowd and navigation library most shipped engines took or copied, refuses invalid geometry at the door, with the same rule at every entry point:

> `if (!m_nav->isValidPolyRef(startRef) || !centerPos || !dtVisfinite(centerPos) || radius < 0 || !dtMathIsfinite(radius) || !filter || maxResult < 0) { return DT_FAILURE | DT_INVALID_PARAM; }` — `DetourNavMeshQuery.cpp`, `findPolysAroundCircle`

That is the rule adopted here for an agent's state: **a position or velocity that is not finite, or a radius that is negative or not finite, is invalid input**. Detour has a status code to refuse with; this crate's documented contract since 0.2.0 is a finite velocity ("A NaN velocity written into a caller's transform makes its agent vanish for good"), so the refusal is expressed as one — the same `Vec2::ZERO` an unusable `max_speed` already gets.

Ericson states the technical rule for the arithmetic once inputs are valid: when a division or overflow produces INF or NaN, "the code should correctly return nonintersection as the result. This can be accomplished by making sure the test for t being in the range [0, 1] is performed correctly" (`book_real_time_collision_detection_ericson`, ch. 11, the infinity-arithmetic discussion before §11.2.3). For `linear_program1` that is: a discriminant that is not a finite non-negative number means the line does not cut the speed disc.

## Model adopted

Every case gets one documented answer. Sane input is untouched: every guard below is a test that passes for finite, in-range values, so no bit of a sane solve changes.

| Input | Answer | Why |
|---|---|---|
| own `pos` or `vel` not finite, own `radius` negative or not finite | `Vec2::ZERO` | No velocity obstacle can be built; holding still is the refusal. A caller that writes the result back has a usable velocity next tick. |
| a neighbour's `pos` or `vel` not finite, its `radius` negative or not finite | that neighbour is skipped | It has no velocity obstacle (Detour's invalid parameter). Skipping it means one corrupted agent does not freeze everyone near it. |
| a neighbour whose ORCA line overflows f32 (finite inputs, e.g. agents at `±f32::MAX`) | that neighbour is skipped | The constraint is beyond what f32 can represent relative to this agent, like one out of range. |
| a wall whose normal is not a unit vector (`Vec2::is_normalized`), or whose `approach · normal` is not finite | `Vec2::ZERO` | Walls are hard: "geometry wins over politeness". A hard constraint that is not a half-plane cannot be dropped, so the agent holds still rather than risk walking through it. |
| `dt` or `time_horizon` whose reciprocal overflows (subnormal) | the wish, clamped | Indistinguishable from 0: no time passes. Same answer as `dt <= 0`. |
| `linear_program1`'s discriminant NaN or infinite | the line is infeasible | Ericson: an INF/NaN test answers "no intersection". |

`Crowd::solve` uses the same predicate: a member whose position, velocity or radius is unusable is no one's neighbour (it is left out of the grid, so it does not take one of the `max_neighbours` slots), and its own solve goes through `new_velocity_in`, so it gets `Some(Vec2::ZERO)` — except a non-finite position, which keeps its documented `None`.

## Parameters

| Parameter | Value | Source |
|---|---|---|
| unit-normal tolerance | `abs(|b|² − 1) <= 2e-4` | glam's `Vec2::is_normalized`, not a constant of this crate |
| radius 0 | usable (a point agent) | design default; Detour accepts `radius == 0` too |
| neighbour line overflow | `point` or `direction` not finite | f32's own limit; no magnitude constant |

## Validation targets

- A hostile sweep — 200 000 single solves with one hostile field each (own state, neighbour state, walls, `pref_vel`, `time_horizon`, `dt`, `max_speed`, coincident agents) plus all-fields chaos, and 20 000 crowd steps with hostile members, params and walls, including empty and single-member crowds — returns **0** non-finite velocities and **0** panics in a debug build (before: 6 505 and 1 709 non-finite, 0 panics).
- Every bench hash unchanged, at both member orders and every thread count: sane input is bit-identical.
- No bench case slower than 5% (median of three interleaved runs).
- The refusals are the documented ones: an unusable own state gives exactly `Vec2::ZERO`; an unusable neighbour gives exactly the bits of the same solve without it; an unusable wall gives exactly `Vec2::ZERO`.

## What great looks like

A game developer whose physics blows up one villager sees that villager stop for a tick — not vanish — and recover the moment its velocity is written back; the villagers beside it carry on exactly as they would without it; nothing else in the crowd moves by a single bit; and no NaN ever reaches a `Transform`, whatever the navigator, the physics or the config file hands in. The rule is short enough to state in one README paragraph and the same in `new_velocity_in` and `Crowd`.

## Deviations

- **From RVO2:** RVO2 validates nothing; its `linearProgram1` tests only `discriminant < 0.0f`. Here a NaN or infinite discriminant also means infeasible. For any finite discriminant the branch taken is RVO2's, bit for bit.
- **Wall normals must be unit vectors.** The README always said so; it is now enforced, with glam's tolerance. A caller passing a scaled or zero normal used to get a scaled or absent constraint and now gets `Vec2::ZERO`. A hand-typed `(0.707, 0.707)` is outside the tolerance (`|b|² = 0.9997`); `(0.7071, 0.7071)` is inside.
- **Design default:** an unusable own state holds the agent still rather than returning `None`. Recorded as an assumed decision (owner may override): options were (a) `Vec2::ZERO`, (b) changing `new_velocity_in` to return `Option<Vec2>` (breaking, every caller migrates), (c) the clamped wish (walks through neighbours). (a) is the conservative pick: non-breaking, the same answer `max_speed` already gets, and self-healing for a velocity.

## Pending index

- Recast/Detour source (`DetourNavMeshQuery.cpp`, `DetourCrowd.cpp`): read on GitHub, not a home-still document.
- ORCA, DOI `10.1007/978-3-642-19457-3_1`: still not downloadable (see `corpus.md`).
- Goldberg, "What every computer scientist should know about floating-point arithmetic" (ACM CS 1991): not in the corpus, not searched this pass; Ericson ch. 11 and PBRT §3.9 cover what this note needs.
