# The speed bound — why the linear program's answer left the speed disc, and the representation that keeps it in

`new_velocity_in` documents "result magnitude is clamped to `max_speed`", and `Crowd::solve` inherits it. Iteration 2 measured the promise broken by f32 cancellation and logged it as the next bug. This pass's probe (`/tmp/o3/probe`, 200 000 single solves per family plus 2.45 million crowd solves, all deterministic) found it much worse than logged:

| Family (baseline `c552e41`) | over `1.000001×` | over `1.001×` | worst `|v| / max_speed` |
|---|---|---|---|
| sane: 0–12 neighbours within 3 m, 0–2 walls, own speed ≤ 1.2× | 5 971 | 436 | **10.2** |
| two coincident neighbours | 10 460 | 762 | **21.9** |
| a row of neighbours (near-parallel constraints) | 605 | 30 | 1.052 |
| own velocity 2 to 1e15 (finite) | 747 | 475 | 5.6e10 |
| one hostile field, finite answers | 2 831 | 221 | 7.0e12 |
| `Crowd::solve`, dense random packs of 2–40, 30 steps each | 18 811 | 620 | **9.30** |

An agent can leave a dense knot at ten times its top speed on ordinary input. The reference implementation does the same: RVO2 (`snape/RVO2`, `main`, built from source in `/tmp/o3/rvo2`) on the same crowd packs returns 640 velocities over `1.001×` its `maxSpeed`, the worst **47.96×**. This is RVO2's numerics, inherited.

## Sources

- **Model:** `10.1109_icra.2012.6225166` (Alonso-Mora et al., ORCA for non-holonomic robots, ICRA 2012: the ORCA velocity as an argmin over the permitted set); `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas — Guy is an ORCA author — Game AI Pro 2 ch. 19: each velocity obstacle becomes one line); RVO2's own contract (`src/Agent.cc`, `src/RVOSimulator.h`, read from source, not a home-still document). The ORCA paper itself is still not in the corpus (`corpus.md`).
- **Shipping game / engine:** `gameaipro3-ch19-rvo-orca-how-they-really-work` (Sunshine-Hill, Game AI Pro 3 ch. 19: ORCA's infeasible case is the common one); Recast/Detour `DetourCrowd/Source/DetourObstacleAvoidance.cpp` (`recastnavigation/recastnavigation`, `main`, read 2026-10-30; not in the corpus): the sampler never considers a candidate outside the speed disc.
- **Technical:** `book_real_time_collision_detection_ericson` (Ericson, ch. 11: catastrophic cancellation in a quadratic's discriminant; the point-normal form of a plane); `book_geometric_tools_computer_graphics_schneider` (Schneider & Eberly, §2 floating-point issues, p. 46: cancellation and division by near-zero in the quadratic formula); `10.1007_s10543-023-00975-x` (Bartels, Fisikopoulos & Weiser, "Fast floating-point filters for robust predicates", 2022: round-off in geometric predicates — new to `corpus.md` this pass).

## Craft principle

ORCA's answer is an optimum over a *bounded* set. In the argmin form, "the set of collision-free velocities for robot i is then given by v*ᵢ = argmin over v ∈ ORCAᵢᵗ of ‖v − vᵢᵖʳᵉᶠ‖" (`10.1109_icra.2012.6225166`, §IV, p. 4), and each neighbour contributes one line: "ORCA avoids this issue by approximating each RVO with a single line. This linear approximation is called an ORCA constraint" (`gameaipro2-ch19-guide-anticipatory-collision-avoidance`, p. 12). The bound is the disc: RVO2 "Solves a two-dimensional linear program subject to linear constraints defined by lines and a circular constraint", whose `radius` is "The radius of the circular constraint", and it passes `maxSpeed_` — "The maximum speed of this agent" — as that radius (`Agent.cc`, `linearProgram2`; `Agent.cc:612`). So the speed bound is not a clamp applied afterwards; it is the one constraint every candidate velocity in all three stages is built inside. An answer outside the disc is not a slightly-too-fast optimum — it is a point the LP never had in its feasible set.

Why the answer leaves the disc anyway is a floating-point fact, not a modelling one. `linear_program1` intersects a line with the disc by a quadratic's discriminant, `dot² + r² − |p|²`, and Ericson uses exactly that expression to show the danger: "consider computing the discriminator b² − 4ac of a quadratic equation for b = 3.456, a = 1.727, and c = 1.729. The exact answer is 4·10⁻⁶, but single-precision evaluation gives the result as 4.82387·10⁻⁶, of which only the first single digit is correct! When, as here, errors become significant due to cancellation, the cancellation is referred to as catastrophic." (`book_real_time_collision_detection_ericson`, ch. 11). Schneider & Eberly name the two culprits together: "cancellation of significant digits by subtraction of two numbers nearly equal in magnitude and division by numbers close to zero, both cases resulting in unacceptable numerical round-off errors" (`book_geometric_tools_computer_graphics_schneider`, p. 46).

Both happen here, in sequence. `linear_program3` builds each projected line through the *intersection* of two constraint lines, `point = pᵢ + (det(dⱼ, pᵢ − pⱼ) / det(dᵢ, dⱼ)) · dᵢ`; for two near-parallel lines the divisor is barely above `EPSILON = 1e-5`, so the point lands up to ~10⁵ m/s from the origin. `linear_program1` then subtracts `|p|² ≈ 10¹⁰` from `dot² ≈ 10¹⁰` to get a discriminant near `r² ≈ 1` — every digit of which is rounding — and returns `p + t·d` with `p` and `t·d` both ~10⁵ and their sum ~1: the answer is off by `10⁵ · 2⁻²⁴ ≈ 0.006` at best and by whole multiples of `r` at worst. An own velocity of 10¹⁰ m/s puts the agent lines themselves that far out.

The cure is the textbook one: "In many cases rewriting an expression can help avoid a cancellation error" (Ericson, ch. 11). A line need not be stored through an arbitrary point. Ericson's *point-normal form*, "n · (X − P) = 0" (`book_real_time_collision_detection_ericson`, the section on planes), says a line is fixed by its normal and one number, its signed distance `c` from the origin; for a unit direction `d`, `c = det(d, p)` for any point `p` on the line, and the line's point nearest the origin is `c · perp(d)`. Stored there, `|point| = |c|`, `dot ≈ 0`, the discriminant is `r² − c²` of small numbers, and the answer `point + t·d` with `|t| ≤ √(r² − c²)` has `|answer|² = c² + t² ≤ r²` up to a few ulps — whatever the line's history.

## How shipping games do it

Sunshine-Hill's account of ORCA in shipped games is that the over-constrained stage is not a corner case: "The problem is that ORCA's solver often does not succeed. … It is easy for a small number of nearby agents to completely rule out all velocities. … ORCA satisfies this by linearly relaxing all constraints." (`gameaipro3-ch19-rvo-orca-how-they-really-work`, §19.5). That relaxation is `linear_program3`, the stage whose projected lines carry the far points — so a crowd in a game takes the broken path every step, wherever it is dense.

Detour, whose crowd most engines took or copied, uses a sampler rather than an LP, and builds the bound into candidate generation so no answer can exceed it: `if (dtSqr(vcand[0])+dtSqr(vcand[2]) > dtSqr(vmax+0.001f)) continue;` (`DetourObstacleAvoidance.cpp`, `sampleVelocityAdaptive`; `sampleVelocityGrid` does the same with `vmax+cs/2`). A shipped engine treats "never faster than `vmax`, up to a fixed slack" as an invariant of the method, not a post-hoc clamp. The LP equivalent is to keep every line in a form where the disc test and the answer are computed from numbers of the disc's size.

## Model adopted

The linear program is RVO2's, stage for stage and name for name; only where each line is anchored changes.

1. **Every line is stored at its point nearest the origin**, with a unit direction. An agent line is built as RVO2 builds it (`point = vel + responsibility · u`) and then re-anchored: `c = det(direction, point)`, `point = c · perp(direction)`. A wall line is built at its foot already (`point = approach · b̂`, `direction = perp(b̂)`), with `b̂ = b / |b|` so the direction is unit to f32 rounding rather than to `is_normalized`'s `2e-4` tolerance — the half-plane is then exactly the documented `v · b̂ ≤ approach`.
2. **`linear_program3` builds each projected line from the two lines' offsets**, never from their intersection point. The projected line is the set where line *i* and line *j* are violated equally, `det(dⱼ − dᵢ, v) = cⱼ − cᵢ` (it passes through the intersection, and, for anti-parallel lines, through RVO2's midpoint); with `D = dⱼ − dᵢ`, its direction is `D / |D|` (RVO2's) and its point nearest the origin is `((cⱼ − cᵢ) / |D|) · perp(D / |D|)`. RVO2's same-direction skip (`|det(dᵢ, dⱼ)| ≤ EPSILON` and `dᵢ · dⱼ > 0`) is kept, so `|D|` is never below ~`EPSILON` where it is used. The anti-parallel case needs no separate midpoint: the same formula is RVO2's midpoint line when the two are exactly anti-parallel, and the true bisector when they are nearly so.
3. `linear_program1` and `linear_program2` are unchanged. With every point at its foot, RVO2's own discriminant `dot² + r² − |p|²` is computed from numbers no larger than the disc for every line that can cut it, and a line with `|c| > r` still fails it, as it must.

Same half-planes, same three stages, same tie-breaks: in exact arithmetic the answer is identical. In f32 every constrained solve rounds differently, so **every bench hash changes once**, logged as intended.

## Parameters

| Parameter | Value | Source |
|---|---|---|
| `EPSILON` (parallel test) | `1e-5`, unchanged | RVO2's `RVO_EPSILON` |
| speed-bound tolerance in tests | `1 + 1e-6` relative | f32 rounding of `c² + t²` (a few ulps of `2⁻²⁴`); design default. Detour allows an absolute `0.001`. |
| wall normal | normalized before use | design default; the refusal test (`is_normalized`) is unchanged |

## Validation targets

- The probe's six families: **0** answers over `1.000001 × max_speed` (before: 5 971 / 10 460 / 605 / 747 / 2 831 / 18 811), and so 0 over `1.001×` (before 436 / 762 / 30 / 475 / 221 / 620); 0 non-finite, 0 panics.
- Optimality unchanged: on 1 000 random small cases the answer is no farther from the wish than the best point of a dense sample of the feasible region (iteration 1: worst gap 0.000000).
- Every bench hash equal across `order=fwd`/`rev` and threads 1 / 2 / default (the hashes themselves change once, logged).
- Overlap and wall depth no worse than before in every bench case, `wall_corridor` 0.0000; every arrival that arrived before still arrives; `head_on` offsets within 1% of each other.
- Cost: no bench case's median step more than 5% slower (median of three runs); candidates per agent unchanged (the neighbour search is not touched).
- A permanent test that fails on the baseline: random dense constraint sets, including near-parallel rows and coincident neighbours, never answer outside the disc.

## What great looks like

A game developer sets `max_speed = 1.4` for a villager and never sees one leave a crowd at 13 m/s — in a doorway jam, in a knot of twelve, when a physics glitch hands in a 10¹⁰ m/s velocity, or after a teleport. No speed spike shows in a velocity graph and no agent pops a metre in a frame: the speed bound is an invariant of the solve, true to f32 rounding for every finite input, the way it is true by construction in Detour's sampler. Nothing else visibly changes: the circle swap, the corridor and the bottleneck look and score as before.

## Deviations

- **From RVO2: where a line is stored.** RVO2 keeps each line through the point it was constructed from and builds each projected line through the intersection of two lines; here every line is stored at its point nearest the origin and projected lines are built from the two lines' offsets. The half-planes and the algorithm are RVO2's; the function names are kept. RVO2's own answer leaves the disc by up to 47.96× on the same packs (`/tmp/o3/rvo2/speed.cc`).
- **From RVO2: the anti-parallel projection.** RVO2 places the projected line of two anti-parallel lines through their midpoint; here the same offset formula gives the exact equal-violation bisector, which is the midpoint line when the lines are exactly anti-parallel.
- **Wall normals are normalized** before the wall's line is built. A normal inside `is_normalized`'s tolerance but not unit used to scale the half-plane by `|b|²` (up to `1 ± 2e-4`); the wall is now exactly `v · b̂ ≤ approach`.

## Pending index

- ORCA, DOI `10.1007/978-3-642-19457-3_1`: still not downloadable (see `corpus.md`); the disc as a hard constraint is quoted from RVO2's source instead.
- Recast/Detour `DetourObstacleAvoidance.cpp`: read on GitHub, not a home-still document.
- RVO2 source (`snape/RVO2`): read and built from GitHub this pass, not a home-still document.
- Goldberg, "What every computer scientist should know about floating-point arithmetic" (ACM CS 1991): not searched; Ericson and Schneider & Eberly cover what this note needs.
