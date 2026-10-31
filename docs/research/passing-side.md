# The passing side — one side for the whole crowd, not one hashed per member

`Crowd::solve` kicks each member's wish sideways when it has any neighbour, "to break the exact symmetry of a head-on meeting, which the linear program has no reason to prefer either way". The side of the kick is `splitmix64(key) & 1`: **a coin per member**. The `README` says why: "a head-on meeting steps to a side hashed from each member's own key, so two agents do not pick the same side and stall". The bench's arrival section has said since iteration 1 that this does not work in a crowd: the circle swap at the defaults leaves 33 of 100 agents stalled (under 5% of preferred speed for over 2 s) and 606 of 1 000, and the deepest overlap is 0.14 of a radius. This note decides what the side should be, before any code.

## Sources

- **Model:** Snape, van den Berg, Guy & Manocha, "The Hybrid Reciprocal Velocity Obstacle", T-RO 2011 (`10.1109_tro.2011.2120810`: what makes a reciprocal scheme oscillation-free — both robots pass on the same side); Helbing & Molnár, "Social force model for pedestrian dynamics", PRE 1995 (`10.1103_physreve.51.4282`) and Moussaïd, Helbing & Theraulaz, PNAS 2011 (`10.1073_pnas.1016507108`): lane formation, the crowd-level order that one shared passing side is the local rule for.
- **Shipping game / engine:** Recast/Detour `DetourCrowd/Source/DetourObstacleAvoidance.cpp` (`recastnavigation/recastnavigation`, `main`, read 2026-10-31, lines 283–303; not a home-still document); RVO2 `src/Agent.cc` (`snape/RVO2`, `main`, lines 571–588); Sunshine-Hill, "RVO and ORCA: How They Really Work", Game AI Pro 3 ch. 19 (`gameaipro3-ch19-rvo-orca-how-they-really-work`: passing sides and the cornering deadlock).
- **Technical:** Revol & Théveny, "Numerical Reproducibility and Parallel Computations" (`10.48550_arXiv.1312.3300`), for why the side must be a function of the world and of nothing the schedule can change; RVO2's own 2-D linear program, which is this crate's `linear_program2`/`linear_program3`, and B-ORCA (`10.1109_icra.2012.6225166`, Remark 3) for the fixed point the ring in "What the nudge cannot fix" is.

## Craft principle

The reciprocity proof is conditional on the side the two agents pass on:

> "In theory, the reciprocal velocity obstacle formulation guarantees that if both robots A and B select a velocity outside the reciprocal velocity obstacle induced by the other, and both robots choose to pass each other on the same side, then the trajectories of both robots will be free of collisions and oscillations in the local time interval." — Snape et al., §IV

For two agents "the same side" is automatic: each takes the velocity closest to its own, which the paper shows lands them on one side (`v_A + u` is outside RVO A|B iff `v_B − u` is outside RVO B|A). For a *crowd* the sides are made by the first encounter, and Game AI Pro says what that costs when they are inconsistent:

> "The problem is that ORCA's solver often does not succeed. [...] It is easy for a small number of nearby agents to completely rule out all velocities. [...] C is passing A on the right and B on the left; A's obstacle prohibits it from going left, B's obstacle prohibits it from going right, and both prohibit it from going straight, or even from stopping or going backward." — Sunshine-Hill, ch. 19

Pedestrians avoid this by a convention: everybody passes on the same side, and the streams that result are lanes.

> "The segregation effect of lane formation is not a result of the initial pedestrian configuration but a consequence of the pedestrians' interactions. Nevertheless, it normally leads to a more effective pedestrian flow since time-consuming avoidance maneuvers occur less frequently." — Helbing & Molnár, the paragraph after Fig. 2

> "For bidirectional traffic in a street, assuming random initial positions of pedestrians, we find that flow directions separate spontaneously after a short time, as empirically observed [...]. This reflects the well-known lane formation phenomenon." — Moussaïd et al., "Collective patterns of motion"

A convention is a property of the *crowd*. A side drawn per member is the opposite of one: of two members that meet head-on, the nudge makes a lateral relative velocity only when their draws agree. With `nudged = wish + perp(wish) · nudge · s` and `perp` turning a quarter turn, two members walking at each other along a line, `A` at `+x` and `B` at `−x`, are kicked to `A: +s_A·y` and `B: −s_B·y`. When `s_A = s_B` they are kicked apart (relative lateral velocity `2·0.05·|wish|`); when `s_A ≠ s_B` they are kicked **the same way** and the relative velocity stays exactly collinear, which is the symmetry the kick was there to break. Half of all pairs, whatever the keys.

## How shipping games do it

Recast/Detour's local avoidance does not draw a side per agent either. Its sampler needs a "side" for a neighbour to penalise samples that cross the line between the two, and for a neighbour dead ahead it takes **one fixed side for every agent**:

> ```cpp
> const float a = dtTriArea2D(orig, cir->dp,dv);
> if (a < 0.01f)
> {
> 	cir->np[0] = -cir->dp[2];
> 	cir->np[2] = cir->dp[0];
> }
> else
> {
> 	cir->np[0] = cir->dp[2];
> 	cir->np[2] = -cir->dp[0];
> }
> ``` — `DetourObstacleAvoidance.cpp:293-303`, `dtObstacleAvoidanceQuery::prepare`

`a` is the signed area of the triangle (origin, direction to the neighbour, relative velocity): below `0.01` the relative velocity has no lateral component, which is the exact head-on case, and `np` is the left normal of `dp` for every agent, so every agent resolves the tie the same way. When there is a lateral component, `np` is the side the neighbour is already on. RVO2 does the same inside the linear program. `Agent::computeNewVelocity` projects a relative velocity on the left leg of the velocity obstacle when `det(relativePosition, w) > 0` and on the right leg otherwise:

> ```cpp
> if (det(relativePosition, w) > 0.0F) {
>   /* Project on left leg. */
> ...
> } else {
>   /* Project on right leg. */
> ``` — `Agent.cc:574-583`

so an exactly collinear meeting, `det = 0`, is the right leg for every agent: each steps to its own right, and the pair pass on opposite sides of the line. This crate's `linear_program`-side code is a port of that (`src/lib.rs`, "Project on one of the velocity-obstacle legs": `det(relative_position, w) > 0.0` is the left leg), and I checked the consequence by running it: with `nudge: 0`, an exactly head-on pair along `x` passes with the `+x` walker at `y = −0.301` and the `−x` walker at `y = +0.301` — **each to its own right** (clockwise from its heading).

RVO2's own `Circle` demo (`examples/Circle.cpp` at `v2.0.2`, `examples/Circle.cc` on `main`, read 2026-10-31) applies no perturbation: 250 agents on a 200 m ring of radius-1.5 discs, 5 m of ring per agent, and the `cos`/`sin` rounding of the ring is its only asymmetry. Adding a tiny random vector to every wish each step is a technique, not something that demo does; this crate cannot do it (no RNG, no per-step state), and `Results` shows that it would not help in the tight ring below anyway.

## Model adopted

**Every member's wish is kicked toward the same side of its own heading, the clockwise side: the side ORCA's own tie-break takes, and the side Detour's and RVO2's exact-tie rules choose.** With `CrowdParams::nudge = ν`:

```text
kicked = wish − perp(wish) · ν      // perp(v) = (−v.y, v.x): a quarter turn counter-clockwise
```

so a positive `ν` kicks toward the *right* of the wish when `+y` is up (clockwise in the plane the caller gave, whatever that plane is in the world), and a negative `ν` toward the left. The kick is still applied only when the member has at least one neighbour, still scaled by the wish (a standing member is not kicked), and still skipped when `kicked` is not finite. `key` no longer picks a side; it breaks ties between equidistant neighbours, as before. Nothing in `Crowd` hashes any more: `splitmix64` is deleted.

What this does and does not claim:

- It does not add a sign convention to the type: `nudge` stays an `f32` and `CrowdParams::new(range)` still gives `0.05`; the side is the sign.
- It agrees with ORCA's own exact-tie rule, so a head-on pair that the kick does not separate (a wish of zero length, say) still passes on the same side the kick would have chosen, and the two never fight.
- It makes the crowd's side a function of nothing but the world and the parameters: not the key, not the member order, not the thread count, not the tick.

## What the nudge cannot fix

A circle of `n` agents of radius `r` on a ring wide enough to start with, all wishing for the centre, converges and stops at the ring where their discs touch (radius `r / sin(π/n)`: `1.918` for `n = 20`, `r = 0.3`). The configuration is a fixed point of the linear program for every member: with both neighbours touching and at rest, the feasible set is the wedge `{v : v·û₊ ≤ 0, v·û₋ ≤ 0}` that excludes moving in, and the closest point of the wedge to a wish pointing in is its apex, `0`. Moving in closes the gap to both neighbours, whichever way the kick points; no per-member side, random perturbation or nudge size leaves it. B-ORCA's Remark 3 names the condition: "Deadlock situations may result when the robot's collision-free velocity closest to its preferred velocity tends to zero [...]. This can be resolved by choosing a new preferred velocity given by a global path planner." So the tight ring (`n ≤ ~40` at 1 m of ring per agent and `r = 0.3`) is outside what any local nudge can do and is the caller's navigator's to resolve; the README records it as a known limit with the number, and the bench keeps its 20-agent row so the limit stays measured.

## Parameters

- `nudge`: unchanged default `0.05` (a design default). Its **sign** is now the side: positive is clockwise (right when `+y` is up), negative counter-clockwise.
- No new field, constant or type.

## Validation targets

- The circle swap at the defaults (`CrowdParams::new(3.0)`, `r = 0.3`, 1.3 m/s, 1 m of ring per agent): arrived `1.000` and stalled `0` at 100 and at 1 000 agents, against `0.990`/33 and `1.000`/606 today; the deepest overlap at 1 000 down from `0.144 r`.
- The two-way corridor (100 agents, interleaved): arrived `1.000`, stalled `0`, overlap no worse than today's `0.021 r`; walls exactly `0`.
- Every bench scene's `overlap_max` no worse than today's, and no scene slower (the change removes a hash per member): at worst inside the paired A/A floor.
- A head-on pair along any axis passes with the two lateral offsets equal to within 1% for any keys; with `nudge < 0` the pass flips side.
- Determinism: every hash equal across member order and thread counts (the bench's caller-owned parallel ring is changed to the same rule); the hashes themselves change, which is the intended change and is logged.
- `tests/alloc.rs` unchanged and green; the leaf ratchet unchanged.

## What great looks like

A crowd that walks in circles in the circle swap's centre, all agents turning the same way, so the picture is the vortex of `docs/circle_swap.gif`: every disc sheared around one hub in one rotation, nobody frozen in the middle. A corridor with two streams that sort themselves into one lane per direction within a few metres, the way Helbing & Molnár's Fig. 2 shows and a pedestrian does by habit. A host that wants the other convention flips the sign of one number.

## Deviations

- **From RVO2:** RVO2 has no nudge; its `Circle` demo perturbs each agent's wish with a fresh random vector every step. This crate's nudge is deterministic and consistent instead; it is already this crate's extension (`CrowdParams::nudge`), and this changes which side it takes.
- **Assumed decision (owner may override):** how the side is chosen. Options: (a) keep the hashed per-member side; (b) one side for all members, selected by the sign of `nudge` (no new type); (c) a new `passing: Side` field on `CrowdParams`. Picked (b): it is the smallest change that fixes the measured failure, adds no field to a struct whose fields are public (so no semver break), and agrees with the side ORCA's own tie-break already takes. A host that needs a per-member or per-region side can still do it by passing its own wish; the kick is a convenience, not the only way to bias a crowd. The direction of the default (clockwise, right of the heading) is a design default taken from the tie-break above.
- **Design default:** `nudge = 0.05` is unchanged from the previous default; no value was fitted.

## Pending index

- Recast/Detour `DetourObstacleAvoidance.cpp` and RVO2 `Agent.cc` are read on GitHub; neither is a home-still document.
- van den Berg et al., "Reciprocal n-body Collision Avoidance" has no open-access copy for `paper_download` (see `corpus.md`); the claims about the reciprocity proof rest on Snape et al.'s restatement.
