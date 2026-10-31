# A wish whose square overflows — what the speed disc does with a very fast `pref_vel` (#9)

`new_velocity_in` clamps the wish to the speed disc by comparing `|pref|²` with `max_speed²` and then normalising. For a finite `pref_vel` longer than `sqrt(f32::MAX)` ≈ 1.84e19 the square is `+inf`: the comparison is still true, the normalise divides by `inf`, and the agent is told to stand still. This note decides what the answer is and how to compute it, before any code.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (ISRR 2009; the paper RVO2 implements and `NOTICE` credits), read from the authors' copy `gamma.cs.unc.edu/ORCA/publications/ORCA.pdf` with the `read` tool (not a home-still document; see `corpus.md`), §5.1, §5.2 and §5.3.
- **Shipping game / engine:** RVO2 (`snape/RVO2`, `src/Agent.cc` `linearProgram2`, `src/Vector2.cc` `abs` and `normalize`) and Recast/Detour (`recastnavigation/recastnavigation`: `Detour/Include/DetourCommon.h` `dtVlen`, `dtVnormalize`; `DetourCrowd/Source/DetourCrowd.cpp` `dtCrowd::update`), all read on GitHub `main` on 2026-10-30; none is in the corpus.
- **Technical:** Anderson, "Algorithm 978: Safe Scaling in the Level 1 BLAS", ACM TOMS 44(1), 2017 (DOI `10.1145/3061665`; abstract from `paper_search`, no open-access copy) and Blue, "A Portable Fortran Program to Find the Euclidean Norm of a Vector", ACM TOMS 4(1), 1978, both read through Reference-LAPACK's `BLAS/SRC/dnrm2.f90` (Anderson is its author); glam 0.33.12's `Vec2::normalize_or` and `clamp_length_max` (vendored, `src/f32/vec2.rs:577` and `:967`).

## Craft principle

ORCA's answer for a wish is a nearest-point query, and the speed limit is one more constraint in it:

> "The fact that we include a circular constraint for the maximum speed does not significantly alter the algorithm, and does not affect the running time. The algorithm returns the velocity [in ORCA_A] that is closest to v^pref_A" — van den Berg et al., §5.1 (p. 9)

So for a free agent the answer is the point of the disc `D(0, v_max)` nearest to `v_pref`: `v_pref` itself inside the disc, and `v_pref / |v_pref| · v_max` outside it. That is a property of the *direction* of `v_pref` once it is outside the disc; nothing in the model changes when the wish is 1e3 times the speed limit or 1e30 times it. A wish of `(1e20, 0)` with `v_max = 1.5` has the one answer `(1.5, 0)` ("as fast as you can, that way"), and with half-planes present the nearest feasible point to a wish that far away is, to within the wish's own rounding, the extreme feasible velocity in that direction. An implementation that answers `(0, 0)` has computed the nearest point to a different wish.

## How shipping games do it

RVO2 has the same expression this crate inherited, and the same hole:

> `} else if (absSq(optVelocity) > radius * radius) {` / `/* Optimize closest point and outside circle. */` / `result = normalize(optVelocity) * radius;` — `Agent.cc`, `linearProgram2`, lines 154–156
> `float abs(const Vector2 &vector) { return std::sqrt(vector * vector); }` … `Vector2 normalize(const Vector2 &vector) { return vector / abs(vector); }` — `Vector2.cc`, lines 122 and 135

For `|v| > 1.84e19`, `vector * vector` is `inf`, `abs` is `inf`, and `vector / inf` is the zero vector. RVO2 therefore also returns `result = 0 * radius`: it stops. Nobody notices because RVO2's wish is a navigator's `goal - position` normalised to a preferred speed, bounded by construction.

Detour keeps its wish bounded by construction too. On the path-following branch it is a unit direction times a speed:

> `ag->desiredSpeed = ag->params.maxSpeed; dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);` — `DetourCrowd.cpp`, `dtCrowd::update`, lines 1207–1208

and only the branch that takes a *caller's* velocity (`requestMoveVelocity`) reads it unbounded, with the same unguarded length:

> `if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY) { dtVcopy(dvel, ag->targetPos); ag->desiredSpeed = dtVlen(ag->targetPos); }` — lines 1190–1193; `dtVlen` is `dtMathSqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])`, `DetourCommon.h` line 202, and `dtVnormalize` is `1.0f / dtMathSqrtf(dtSqr(v[0]) + …)`, lines 263–269

The lesson is the pattern, not a fix to copy: a shipped crowd lives with the hole while its wish is bounded by construction. This crate's `pref_vel` is a plain `Vec2` from a caller's navigator (`offset / time_left` with a small `time_left` is the issue's example), so the bound is not ours to assume.

## The technical rule

Overflow in `sqrt(x² + y²)` is a solved problem, and its solution does not change the answer for any input that did not overflow:

> "The square root of a sum of squares is well known to be prone to overflow and underflow. Ad hoc scaling of intermediate results, as has been done in numerical software such as the BLAS and LAPACK, mostly avoids the problem, but it can still occur at extreme values in the range of representable numbers." — Anderson, Algorithm 978, abstract

Blue's algorithm, as LAPACK's `dnrm2.f90` ships it, scales *only the values that need it*, by a power of the radix:

> "Compute the sum of squares in 3 accumulators: abig -- sums of squares scaled down to avoid overflow … The thresholds and multipliers are tbig -- values bigger than this are scaled down by sbig" and, once combined, `scl = one / sbig`, `DNRM2 = scl*sqrt( sumsq )` — `dnrm2.f90`, lines 132–138 and 169–198

Two properties carry over: a mid-range vector takes the unscaled path (so every sane result keeps its bits), and the scale is a power of two, so scaling is *exact* and adds no rounding to the result. A direction does not need the scale back: `v / |v|` is scale-free, so normalising `v · 2^-k` is normalising `v`.

## Model adopted

A finite wish outside the speed disc is clamped to the disc along its own direction, however far outside it is.

- **Where.** Two places take the square of the wish: `linear_program2` (the `direction_opt == false` call, the only one that receives `pref_vel`; the `direction_opt == true` calls pass a unit vector) and the degenerate-time return (`dt` or `time_horizon` not positive or so small its reciprocal overflows → "no time passes → the wish, clamped"). `Crowd::solve` forms `pref + pref.perp() * (nudge · side)` before calling the solver, which overflows to `±inf` when `|x| + nudge · |y|` of the wish exceeds `f32::MAX` (both components near `f32::MAX`) and hands `new_velocity_in` a non-finite wish it then refuses with `Vec2::ZERO`: the same stop for a finite wish, reached one step earlier. There the nudge is dropped when it is not representable and the wish is used as given. A wish that finite is a direction, and the nudge's only job (breaking a head-on tie by a few percent) is moot beside it.
- **How.** When `length_squared()` of the wish is not finite — the one condition that makes the existing expression wrong — scale the wish by `2^-96` first and then `normalize_or_zero()` as before. Every finite `f32` vector whose square overflows has a component of at least `2^63.5` (its square must exceed `f32::MAX ≈ 2^128`), so the scaled vector has one of at least `2^-32.5` (squares from `2^-65`, far above `f32::MIN_POSITIVE ≈ 2^-126`) and none above `2^32` (squares to `2^64`, sum `2^65`, far below `f32::MAX`): neither end can overflow or underflow the normalise. The scale is a power of two, so the product is exact; a component that scales below the normal range was smaller than the other by a factor under `2^-93`, which no `f32` sum can see. The finite-square path is the code it was, so every sane input is bit-identical.
- **What it costs.** One `is_finite` on a value the branch already computed, in a branch that only runs when the wish is already outside the disc, once per solve. Nothing per neighbour.
- **What is not changed.** The nearest-point semantics inside the LP (`linear_program1` receives the wish unclamped, as RVO2 does): there a huge wish only ever lands in `t < t_left` / `t > t_right` comparisons that clamp it (an `inf` from the dot product compares correctly). The one way to a NaN is `opt - point` overflowing to `inf` and meeting a zero direction component (`inf · 0`), which takes a line point near `1e31`. Measured, not assumed: 400 000 solves with an own velocity of 1e9 to 3e38 and a wish of 1e19 to `f32::MAX`, with neighbours and walls, give 0 non-finite velocities, before and after.

Rejected:

| Option | Why not |
|---|---|
| Divide by `abs().max_element()` first (the issue's suggestion) | Equal result, one division per component and a rounding step that a power-of-two scale does not have; same guard. |
| `hypot` | `f32::hypot` is the platform's libm in `std`: results differ across platforms, and this crate's whole feature discipline (`nostd-libm` only) exists to keep replay hashes platform-independent. |
| Clamp `pref_vel` up front to a safe magnitude (say `1e18`) | Changes which wish the LP is nearest to, for any input above the cap, in a way that is not the answer for `1e20`; adds a branch to every solve. |
| Refuse: answer `Vec2::ZERO` and document it (the issue's alternative) | Keeps the bug as a feature: an agent whose navigator says "go, as fast as you can" stops dead. The promise is "nearest to `pref_vel`", and a finite wish is usable. |
| Compare `|pref|` with `max_speed` in scaled space everywhere | Changes bits for sane input (the comparison and the clamp are different expressions). |

## Parameters

| Parameter | Value | Source |
|---|---|---|
| scale when the square overflows | `2^-96` = `1.2621774e-29` | design default, from the range argument above (any `2^-k` with `65 ≤ k ≤ 126` works); a power of two, as Blue's `sbig` is |
| guard | `!wish.length_squared().is_finite()` | the one condition under which `|v|² > r²` is wrong |

## Validation targets

- `a_huge_finite_wish_is_clamped_not_zeroed` (the issue's test, unchanged) fails on `main` and passes after, for `dt = 1/60` and `dt = 0`.
- A probe over wish magnitudes `1e10 … f32::MAX` (axis-aligned, diagonal, mixed, components at the magnitude), free / head-on / wall / both, `dt` normal and `0`: **0** results equal to `Vec2::ZERO`, **0** non-finite, **0** above `1.0001 · max_speed`; free agents within `1e-3 · max_speed` of `direction · max_speed` (before: 112 of 160 free cases `Vec2::ZERO`).
- `Crowd::solve` with the same wishes and `nudge ∈ {0, 0.05}`: every steered member's answer finite, inside the disc, and in the wish's quadrant (before: `Some(Vec2::ZERO)`).
- Sane input is bit-identical: every `bench` hash equal at both member orders and every thread count against the base; `head_on` and `wall_corridor` output byte-identical; `tests/alloc.rs` passes.
- No bench case more than 5% slower in a paired same-process comparison with an A/A control.
- The hostile sweeps of iteration 2 (`hostile_*` tests, 200 000 single solves, 20 000 crowd steps) still give 0 non-finite and 0 panics.

## What great looks like

A navigator that computes `pref_vel = offset / time_left` and lets `time_left` fall to a frame's epsilon sees its agent run at full speed toward the goal and arrive, never stop; the same through `Crowd`; the same at `dt = 0`. The answer's length is `max_speed` to 2e-7 relative (a few ulp, the rounding of a normalise) and its direction is the wish's, whether the wish is 2 m/s, 1e20 or `f32::MAX`. No sane solve moves by a bit, and no per-neighbour work is added.

## Deviations

- **From RVO2:** `normalize(optVelocity) * radius` with no overflow guard (it returns the zero vector for `|v| > 1.84e19`). Here the same expression is used for every finite square and a scaled one when the square overflows.
- **From Detour:** none to adopt; `dtVlen` / `dtVnormalize` have the same unguarded squares, and Detour's wishes are bounded by construction.
- **Design default (assumed decision, owner may override):** a finite wish above the overflow bound is clamped along its direction, not refused with `Vec2::ZERO`. Options: (a) clamp (the issue's first fix, and what "nearest to `pref_vel`" means), (b) `Vec2::ZERO` listed in the rustdoc (the issue's second option). Picked (a): it keeps the documented promise rather than shrinking it, and changes no answer a caller could have relied on being right.
- **Design default:** `Crowd::solve` drops its nudge when `pref + pref.perp() * nudge` is not finite, where it used to hand the solver a non-finite wish. Same reasoning, one step earlier in the same path. The test is on the sum, so it also covers a `nudge` that is NaN or infinite, which used to stop every member that had a neighbour (measured: `nudge = NaN` gave `Some(Vec2::ZERO)` to both members of a head-on pair, and now solves them as `nudge = 0` does). That is a change for a hostile parameter, not for any finite sane one.

## Results

Measured on macOS arm64 (14 cores, shared host), base `1836129` against this tree.

- **The issue:** `(1e20, 0)` at `max_speed = 1.5` gives `(1.5, 0)` at `dt = 1/60` and `dt = 0` (before `(0, 0)`). The three new tests fail on `main` (the crowd one at `nudge = 0`, then at `nudge = 0.05` once the nudge guard is removed) and pass after.
- **Probe** (160 wishes of 1e10 to `f32::MAX` × six situations): `Vec2::ZERO` 112 of 160 free → 0, non-finite 0 → 0, over-speed 0 → 0.
- **Differential sweep** (600 000 random solves: random own state, 0–8 neighbours, 0–3 walls, `dt` and `time_horizon` including 0, `max_speed` 0.3 to 1e5, wishes sane and 1.9e19 to `f32::MAX`): the 402 299 whose wish square is finite hash equal before and after; of the 197 701 whose square overflows, `Vec2::ZERO` 147 998 → 6 (all six have an own velocity of 1e9 or more), free agents off their direction 5 162 → 0, non-finite 0 → 0, over `1.001 · max_speed` 11 → 11 (the f32 cancellation in `linear_program1`, iteration 2's backlog, unchanged).
- **Precision:** 400 000 free huge wishes, worst `|len/max_speed − 1|` 1.86e-7 and worst direction error 1.87e-7.
- **Bench:** all 80 rows hash-equal across base, new, both member orders, threads 1, 2 and 14, three runs each; `overlap_max`, `wall_max`, `allocs` and the four `arrive` lines identical; `head_on` and `wall_corridor` byte-identical. The macro medians cannot resolve 5% on this host (the same binary read corridor 1 000 at 1 100 to 2 020 µs and bottleneck 1 000 at 3 986 to 5 025 µs across runs).
- **Cost, paired** (one process, base twice and new twice solving one frozen crowd in rotating order, thread CPU time, 1 500 reps, median of per-rep ratios, three rounds; the A/A pair is the floor): B/A open 1 000 −0.80 / −0.80 / −0.70%, open 1 000 with the changed branch always taken −0.92 / −0.69 / −0.79%, open 100 +1.36 / +0.30 / +0.99%, circle 1 000 +0.52 / +0.26 / +0.03%, corridor 1 000 −0.41 / −0.11 / −0.27%, corridor 100 +0.31 / −0.67 / −0.74%; A/A reached +3.45% (open 100). Results bit-equal in every round.

## Pending index

- `distill_search` and `distill_index` unavailable this pass (`gpu busy: 4029 MB free < 5000 MB required to load bge-m3`): nothing here is indexed, and no corpus document was searched.
- Anderson 2017 (DOI `10.1145/3061665`) and Blue 1978 (DOI `10.1145/355769.355771`): no open-access copy (`paper_download`: unpaywall, PMC, Semantic Scholar, Europe PMC, OpenAlex, CrossRef all none; CORE needs a key). Read through `dnrm2.f90`.
- van den Berg et al., the authors' PDF; RVO2 and Detour source; `dnrm2.f90`: read on the web, not home-still documents.
