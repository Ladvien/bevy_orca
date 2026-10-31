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

A convention is a property of the *crowd*, and a side drawn per member is the opposite of one. It costs in two places, both measured in `Results`:

1. **In a pair.** ORCA chooses which half-plane a neighbour casts from the relative position and the *current* velocities (`det(relative_position, w)`), never from the wish, so the wish's kick can only act once it has changed a velocity. For an exactly collinear pair the tie rule sends both to their own right whatever the kick says; the kick is a perturbation on top of that. With `nudged = wish + perp(wish) · nudge · s` and `A` at `−x` heading `+x`, `B` at `+x` heading `−x`, the kicks are `A: +s_A·y` and `B: −s_B·y`: when the two draws are equal the pair is kicked apart symmetrically, and when they differ both are kicked toward the same side of the line and the pair is no longer a half-turn of itself. Half the key pairs differ, and for those the two sidesteps are not equal and opposite (128 of 256 key pairs; the worse sidestep up to 22% longer). Reciprocity is the one thing a pair has to keep.
2. **In a crowd.** The kick is a constant lateral bias on every member that has a neighbour. A shared bias adds up: every agent turns the same way, and the circle swap becomes the vortex of `docs/circle_swap.gif`, a roundabout. A bias hashed per member mostly cancels and what is left is whichever way the keys' coin tosses happen to lean, so the crowd crosses in a loose, slow turn (60–84% of the crossing agents one way, and the way is the keys', not the caller's) and leaves a jam behind it: the stalls of the arrival section.

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
- It agrees with ORCA's own exact-tie rule (right leg), so a pair the wish's kick cannot separate passes on the side the kick would have chosen, and the two never fight.
- It makes the crowd's side a function of nothing but the world and the parameters: not the key, not the member order, not the thread count, not the tick.

## What the nudge cannot fix

A circle of `n` agents of radius `r` on a ring wide enough to start with, all wishing for the centre, converges and stops at the ring where their discs touch (radius `r / sin(π/n)`: `1.918` for `n = 20`, `r = 0.3`). The configuration is a fixed point of the linear program for every member: with both neighbours touching and at rest, the feasible set is the wedge `{v : v·û₊ ≤ 0, v·û₋ ≤ 0}` that excludes moving in, and the closest point of the wedge to a wish pointing in is its apex, `0`. Moving in closes the gap to both neighbours, whichever way the kick points; no per-member side, random perturbation or nudge size leaves it. B-ORCA's Remark 3 names the condition: "Deadlock situations may result when the robot's collision-free velocity closest to its preferred velocity tends to zero [...]. This can be resolved by choosing a new preferred velocity given by a global path planner." So the tight ring (`n ≤ ~40` at 1 m of ring per agent and `r = 0.3`) is outside what any local nudge can do and is the caller's navigator's to resolve; the README records it as a known limit with the number, and the bench keeps its 20-agent row so the limit stays measured.

## Parameters

- `nudge`: unchanged default `0.05` (a design default). Its **sign** is now the side: positive is clockwise (right when `+y` is up), negative counter-clockwise.
- No new field, constant or type.

## Validation targets

- The circle swap at the defaults (`CrowdParams::new(3.0)`, `r = 0.3`, 1.3 m/s, 1 m of ring per agent): arrived `1.000` and stalled `0` at 100 and at 1 000 agents, against `0.990`/33 and `1.000`/606 today; the deepest overlap at 1 000 down from `0.144 r`; the time for 90% to arrive (`t90_s`, new in the bench) no longer than today's.
- The two-way corridor (100 agents, interleaved) and the bottleneck (100 and 400 agents): arrived no lower than today, stalled no higher, walls exactly `0`.
- Every bench scene's `overlap_max` inside the spread the same scene shows under today's code when only the keys are shifted (a chaotic maximum over thousands of agents is not comparable to the digit), and no scene slower: the change removes a hash per member, so at worst inside the paired A/A floor.
- A head-on pair along `x`: the two sidesteps equal and opposite to within 1% for every one of 64 key pairs (today: 32 of 64 break it), each to its own right for a positive `nudge`.
- A crowd turns the way the sign says: with `nudge > 0` at least 90% of the moving agent-steps in the middle of a 75-agent circle swap turn counter-clockwise, with `nudge < 0` clockwise.
- Determinism: every hash equal across member order and thread counts (the bench's caller-owned parallel ring is changed to the same rule); the hashes themselves change, which is the intended change and is logged.
- `tests/alloc.rs` unchanged and green; the leaf ratchet unchanged.

## What great looks like

A crowd that turns the way the host chose, every agent the same way, so the picture is the vortex of `docs/circle_swap.gif`: every disc sheared around one hub in one rotation, nobody frozen in the middle, the whole ring through in about half the time. A host that wants the other convention flips the sign of one number. What it is not: clean lanes. A two-way corridor of 200 leans toward one lane per direction (0.53–0.81 of the agent-steps on their own right half in three crowds, against 0.47–0.52 with a side per member), but a 5% kick does not sort it the way Helbing & Molnár's Fig. 2 does, and nothing here claims it.

## Deviations

- **From RVO2:** RVO2 has no nudge, and its `Circle` demo applies none; this crate's nudge is already its own extension (`CrowdParams::nudge`), and this changes which side it takes and makes it one side for everyone, the way Detour fixes its dead-ahead side.
- **Assumed decision (owner may override):** how the side is chosen. Options: (a) keep the hashed per-member side; (b) one side for all members, selected by the sign of `nudge` (no new type); (c) a new `passing: Side` field on `CrowdParams`. Picked (b): it is the smallest change that fixes the measured failure, adds no field to a struct whose fields are public (so no semver break), and agrees with the side ORCA's own tie-break already takes. A host that needs a per-member or per-region side can still do it by passing its own wish; the kick is a convenience, not the only way to bias a crowd. The direction of the default (clockwise, right of the heading) is a design default taken from the tie-break above.
- **Design default:** `nudge = 0.05` is unchanged from the previous default; no value was fitted.

## Pending index

- Recast/Detour `DetourObstacleAvoidance.cpp` and RVO2 `Agent.cc` are read on GitHub; neither is a home-still document.
- van den Berg et al., "Reciprocal n-body Collision Avoidance" has no open-access copy for `paper_download` (see `corpus.md`); the claims about the reciprocity proof rest on Snape et al.'s restatement.

## Results (iteration 6)

Host: macOS arm64, 14 cores, shared (load 3–6 during the timing runs). "Before" is `e0fd584`, "after" is `90367b1` (the later commits are comments and docs); every bench row below is the same `examples/bench.rs` (the new one) run against the old and the new `Crowd`.

- **Tests.** All four new tests (`every_member_is_kicked_to_the_same_side_whatever_its_key`, `a_head_on_pair_sidesteps_equally_whatever_its_keys`, `a_circle_swap_arrives_and_nobody_stalls`, `the_side_of_the_kick_sets_which_way_a_crowd_turns`) fail on the old hashed side and pass after: key 2 is kicked the wrong way; keys (0, 2) sidestep −0.604 and +0.493; a 75-agent swap leaves 1 agent short with 47 stalls; 10 391 mid-ring agent-steps turn counter-clockwise against 53 136 clockwise for a positive `nudge`.
- **Exactness.** On 200 frozen random crowds (16 791 + 16 969 members) every member whose old hashed side was already the right-hand one solves bit-identically to before (0 of 16 791 differ), and of the 16 969 whose side was the left 9 028 differ (the ones with a neighbour). Nothing but the side of the kick moved.
- **Pair reciprocity** (`nudge` 0.05, two members 8 m apart walking at each other, 256 key pairs): old, 128 of 256 key pairs have `|peak_a + peak_b| > 1%` of the sidestep, worst 22.5%; new, 0 of 256, worst 0.0000. Each walker passes on its own right (`a` at −0.5964, `b` at +0.5964).
- **Arrival, the circle swap** (`CrowdParams::new(3.0)`, 0.3 m, 1.3 m/s, 1 m of ring per agent, three times the straight-line budget), old → new:

  | agents | arrived | stalled | t50 | t90 | deepest overlap |
  |---|---|---|---|---|---|
  | 20 | 0.000 → 0.000 | 20 → 20 | never | never | 0.0000 → 0.0000 |
  | 20, `circle_swap`'s wider ring | 1.000 → 1.000 | 0 → 0 | 14.1 → 15.2 s | 15.1 → 15.3 s | 0.0179 → 0.0000 |
  | 100 | 0.990 → 1.000 | 33 → 0 | 50.4 → 30.7 s | 60.5 → 34.6 s | 0.0980 → 0.0639 |
  | 1 000 | 1.000 → 1.000 | 606 → 0 | 325.7 → 259.0 s | 346.9 → 266.4 s | 0.1439 → 0.0173 |

  The reference implementation, built as shipped (`snape/RVO2` `main`, `Agent` defaults set to this bench's radius, speed, horizon 2 s, 10 neighbours, range 3 m, `dt` 1/30, the same antipodal goals and leave-on-arrival), gets t90 63.1 s at 100 agents and 379.6 s at 1 000, so the hashed side was about at its level and a shared side is 45% and 30% faster. It also has the tight ring: 0 of 20 arrive on the 1 m ring, with no perturbation and with a `1e-4` random one, stopping at a mean radius of 1.919 against the 1.918 where 20 discs of 0.3 touch; 0 of 30 (2.870 against 2.870) and 0 of 50 (4.778 against 4.778) do the same, 75 and 100 get through. It also stops 0 of 20 on the wider ring with perfect symmetry, which `Crowd` with a nudge does not.
- **Why 20 stays at 0.000.** Every member's discs touch both neighbours and its closest feasible velocity to a wish pointing in is zero: the fixed point of "What the nudge cannot fix", confirmed by the reference implementation above, so the row stays in the bench as a measured limit and the README says so.
- **Two-way corridor and bottleneck**, 8 crowds each (`ORCA_SEED` 0–7), old → new, mean [range]: corridor 100 arrived 0.999 → 1.000, stalled 0 → 0, deepest overlap 0.026 [0.019..0.033] → 0.037 [0.011..0.043]; bottleneck flow (agents through the gap within the budget, 100 / 200 / 400) 0.641 [0.55..0.76] → 0.617 [0.44..0.73], 0.456 [0.37..0.48] → 0.448 [0.39..0.51], 0.265 [0.20..0.31] → 0.286 [0.21..0.33]: no difference outside the spread between crowds. Walls: `wall_max` is exactly 0.0000 in all 20 measured timing rows, in every arrival row and in all 80 eight-crowd rows.
- **Deepest overlap, 8 crowds each at 100 and 1 000 agents**, old mean → new mean: open 0.038 → 0.037 and 0.065 → 0.059; circle 0.100 → 0.064 (100); corridor 0.026 → 0.037 (100, above the old range in 7 of 8 crowds) and 0.059 → 0.048 (1 000); bottleneck 0.126 → 0.112 and 0.189 → 0.167; rally 0.008 → 0.007 and 0.114 → 0.075. The 100-agent corridor is the one case that is worse: 0.011 of a radius, 3 mm at 0.3 m.
- **What was tried and did not help the corridor.** A kick gated to members with a neighbour that is closing on them or ahead of them (three variants) leaves the 100-agent corridor at 0.035–0.037; `nudge` 0 gives 0.032 (with 0.4 agents stalled on average), 0.02 gives 0.037, 0.1 gives 0.042 and 0.2 gives 0.044: a shared bias presses each stream a little toward its right wall, and the old per-member coin dithered it. Not fixed here; see the log's backlog.
- **The other sign.** Kicking left (the counter-clockwise side) measured the same within the spread: circle 100 arrived 1.000/0 stalled, circle 1 000 0 stalled, overlap 0.055 and 0.020, corridor 0.035; the clockwise side is kept because it agrees with the linear program's own tie rule.
- **Lanes.** Two-way corridor, 200 agents, 6 m wide, the middle third of the run and the middle half of the corridor's length: the fraction of agent-steps an agent spends on its own right half, old → new, for three crowds: 0.517, 0.474, 0.517 → 0.569, 0.528, 0.809. A lean, not lanes.
- **Determinism.** Every one of the 20 (scene, size) groups of the full bench has one hash across `order=fwd`, `order=rev`, 2 threads and 14 threads (80 rows). The hashes differ from the old ones in all 20 groups, which is the intended change.
- **Cost.** Four crate copies in one process (two old, two new) on a frozen crowd, thread CPU time, 1 500 repetitions, three rounds: new/old +0.34…+0.73% (open 1 000), +0.44…+0.62% (open 5 000), +0.06…+0.33% (1 000 converging at 1 agent/m²), −0.49…−0.66% (sparse, 1 000), against an A/A (old/old) floor of −0.38…+0.33%. Whole bench (three interleaved serial rounds, medians): mean −1.7%, median −1.0% over 20 cases; no scaling exponent above 1.15 (largest 1.081 before, 1.063 after: open 1.046 → 1.015, circle 1.037 → 1.061, corridor 1.081 → 1.039, bottleneck 0.874 → 0.876, rally 1.067 → 1.063); allocations 0–2 per case, except the 100-agent circle's 9 as before (circle 1 000 to 10 000: 2 → 0). Candidates per agent are within ±3% either way (bottleneck 31.7 → 33.1 at 100, rally 170.8 → 173.2 at 1 000, circle 23.8 → 21.7 at 100): the scan code is untouched, so that is where the crowd is, which the side changes. The three cases a three-round median put over +5% (bottleneck 100, corridor 1 000, open 1 000) read −7.3%, −5.3% and +4.4% over seven interleaved rounds, bimodal in both builds (the corridor 1 000 base binary read 797 µs once and about 2 000 µs six times).
- **Gate.** `cargo fmt --check`, `clippy --all-targets --all-features -D warnings`, `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features`, `cargo build` / `--all-features` / `--release`, `cargo test` and `cargo test --all-features` (31 + 5 + 2 each) and `cargo build --examples --all-features` all exit 0. `head_on` and `wall_corridor` output is byte-identical to before (A 0.448 / B 0.451, holding ground 0.895 / 0.000; closest 0.800, wall penetration 0.0000).
