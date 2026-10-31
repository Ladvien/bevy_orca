# A member the solve will not move — who takes half of the avoidance against it

`Crowd::solve` offers every neighbour to the solver with `avoids: pref_vel.is_some()`. `avoids` is the crate's one departure from RVO2's "every agent takes half": a neighbour that will not step aside is avoided in full, so the mover is not walked through it. Issue #8 (found by the owner's review) showed the predicate is wrong for two kinds of member that have `Some(wish)` but that this very solve does not move: a wish that is not finite (the solve skips it and returns `None`), and a `max_speed` that is not positive or whose square overflows (`new_velocity_in` returns `Vec2::ZERO`). Both hold still, and the mover still takes only half. This note decides what `avoids` means and where it is computed, before any code.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance", ISRR 2009 (the paper RVO2 implements and `NOTICE` credits; DOI `10.1007/978-3-642-19457-3_1`). Not in the corpus; read from the authors' copy, `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`, fetched 2026-10-30 (§3, §4.2, §5.4). `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas, Game AI Pro 2 ch. 19: what reciprocity means to a shipping crowd).
- **Shipping game / engine:** RVO2 itself, `src/Agent.cc` (`snape/RVO2` on GitHub, `main`, read 2026-10-30; line 607) and Recast/Detour `DetourCrowd/Source/DetourCrowd.cpp` (`recastnavigation/recastnavigation`, `main`, line 1273); neither is a home-still document. `gameaipro3-ch19-rvo-orca-how-they-really-work` (Sunshine-Hill, Game AI Pro 3 ch. 19: what the reciprocity proof assumes).
- **Technical:** IEEE-754 comparison semantics, for writing one predicate that is false for NaN: `PBR3_03_Shapes` (Pharr, Jakob & Humphreys, *PBRT* 3rd ed.); and this crate's own `new_velocity_in` guard (`src/lib.rs`), which the predicate must equal exactly.

## Craft principle

ORCA gives each side half of the avoidance **because the other side takes the other half**. The paper is explicit that this is an assumption about the neighbour, not a property of the neighbour:

> "However, each of the robots may assume that the other robots use the same strategy as itself to select a new velocity." — van den Berg et al., §3 "Problem Definition"

> "To 'share the responsibility' of avoiding collisions among the robots in a fair way, robot A adapts its velocity by (at least) ½u and assumes that B takes care of the other half." — §4.2

Where the assumption is false, the paper takes the full share. Its static obstacles do not move:

> "We basically follow the same approach as above, with a key difference being that obstacles do not move, so the robots should take full responsibility of avoiding collisions with them." — §5.4 "Static Obstacles"

Game AI Pro states the same coupling from the crowd's side: "ORCA resolves this issue through the use of reciprocity, allowing each agent to avoid only part of the collision with the knowledge that the neighboring agent will resolve the reminder [sic] of the collision (a simple solution is to split the work 50-50 between the two agents)." (`gameaipro2-ch19-guide-anticipatory-collision-avoidance`, ORCA section). The "knowledge" is what `avoids` stands for. A mover that takes half against a neighbour that does not move is relying on a knowledge it does not have, and the pair drifts into contact — the failure the crate's `an_unsteered_member_is_not_moved_and_is_avoided` test already guards for `None`.

So the rule is not "does this member have a `pref_vel`" but "will this member's velocity be changed by this solve". The model gives no other test.

## How shipping games do it

RVO2 has no flag at all. Every agent neighbour gets the half share:

> `line.point = velocity_ + 0.5F * u;` — `Agent.cc:607`, `Agent::computeNewVelocity`, in the loop over agent neighbours

and only static obstacle lines (built earlier in the same function) are full-responsibility. A game that roots a unit with RVO2 either models it as an obstacle or lets it step aside; the reference library has no third thing. Detour has no reciprocity either: it offers each neighbour to its sampler as a circle with a position, a velocity and a desired velocity,

> `m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);` — `DetourCrowd.cpp:1273`

and the only state test around it is whether the agent being solved is `DT_CROWDAGENT_STATE_WALKING`. So neither engine answers the question this issue asks; `avoids` is this crate's extension (ORCA's §5.4 applied to a neighbour that is an agent), and the extension has to be as exact as the thing it replaces. Game AI Pro 3 gives the precondition in one line: the guarantee "only holds for a world containing two agents (and no obstacles). With only two agents, each one is free to choose the perfect reciprocating velocity" (`gameaipro3-ch19-rvo-orca-how-they-really-work`, §19.3) — an agent that cannot choose a velocity is outside it.

## Model adopted

`avoids` for a neighbour is true exactly when **this `solve` will steer that member to a velocity of its own choosing**. `wish(m)` is the member loop's skip; `steers(m) = usable_speed(m.max_speed) && wish(m).is_some()` is the flag. Both are one definition, and `steers` runs once per member in the grid pass (a `Vec<bool>` the neighbour builder reads), not once per neighbour offered — the per-neighbour version measured +2 to +5% on sparse and open scenes in a paired same-process A/B, the per-member one +0.0 to +1.6% against an A/A floor of ±2%:

| Member | `solve`'s output for it | Offered to others as |
|---|---|---|
| `pref_vel: None` | `None` | `avoids: false` (full avoidance) — unchanged |
| `Some(wish)`, wish not finite | `None` (skipped) | `avoids: false` — **was `true`** |
| `Some(wish)`, `max_speed` not positive, NaN, or so large its square overflows | `Some(Vec2::ZERO)` (`new_velocity_in`'s guard) | `avoids: false` — **was `true`** |
| `Some(wish)`, finite wish, usable speed | `Some(solved)` | `avoids: true` (half) — unchanged |

The speed test is lifted out of `new_velocity_in` into one `pub(crate)` function, `usable_speed(max_speed) = max_speed > 0.0 && (max_speed * max_speed).is_finite()`, and `new_velocity_in` calls it, so "the solver returns ZERO" and "the neighbour is avoided in full" are the same condition by construction. It is equal to the old guard for every `f32` (`NaN > 0.0` is false, as PBRT states: "any logical comparison involving a NaN always evaluates to false"), so `new_velocity_in`'s results do not change.

A member whose own position, velocity or radius is unusable is not a neighbour at all (iteration 2: left out of the grid), so it needs no flag. A member whose `walls` closure returns an unusable wall also holds still (`Vec2::ZERO`), but only the closure can say so and it is called after the neighbours are built; that case is documented, not detected.

## Parameters

None. The change introduces no constant: the predicate is the two existing guards.

## Validation targets

- The issue's test, extended: a still member with a NaN wish, an infinite wish, `max_speed` of `0`, `-1`, NaN, `∞` and `1e20` each leaves the mover's velocity bit-equal to its answer against an unsteered (`None`) member; a steered member with a zero wish does not (the control).
- The test fails on `main` (the half-share answer differs) and passes after.
- Every sane solve is bit-identical: all bench hashes equal at both member orders and threads 1 / 2 / default, before and after.
- `usable_speed` equals the previous guard on every one of a sweep of `f32` bit patterns (including NaN, ±∞, ±0, subnormals, `±1.8e19`).
- No bench case more than 5% slower (median of three): the change adds one `is_finite` and one multiply per neighbour offered to a solve.
- `tests/alloc.rs` unchanged and green.

## What great looks like

A host roots a villager by setting `max_speed: 0.0` (a stun, a placed building, a unit loading onto a boat) and leaves its wish alone: everyone near it walks around it, taking the whole sidestep, and none of them clips through it the way they did when they assumed it would step aside. A navigator that produced a NaN for one villager for one tick freezes that villager and its neighbours give way to it for that tick. A host reading `README` finds the same rule in one sentence: *a neighbour that this step will not move is avoided in full, whether you said so with `None`, a wish that is not a number, or a speed of zero*.

## Deviations

- **From RVO2:** RVO2 has no per-neighbour responsibility (`0.5F * u` for every agent). `avoids` is already this crate's extension; this change only makes its input exact.
- **Assumed decision (owner may override):** what `Crowd::solve` returns for `Some(wish)` with a wish that is not finite. Options: (a) `None`, as the code and the rustdoc already do; (b) `Some(Vec2::ZERO)`, which is what `new_velocity_in` answers for the same wish and what `README` line 88 can be read to promise; (c) leave the two disagreeing. Picked (a): it changes no output, `None` already means "not solved" in the rustdoc, and (b) would turn a member the solve declines to steer into one it reports as solved to a stop. The README is corrected to say `None`.
- **Design default:** a member whose `walls` closure is the only thing wrong is still offered as `avoids: true` (see Model adopted).

## Pending index

- van den Berg et al., "Reciprocal n-body Collision Avoidance" — the authors' PDF is read and quoted above, but there is still no open-access copy for `paper_download` (see `corpus.md`). Not indexed.
- RVO2 `Agent.cc` and Recast/Detour `DetourCrowd.cpp`: read on GitHub, not home-still documents.
- `distill_search` and `abstract_search` were down this pass (`Embedding error: gpu busy: 4866 MB free < 5000 MB required to load bge-m3`); the sources above were read with `markdown_read` by stem and from the authors' PDF.

## Results (iteration 4)

- **Test.** `a_member_that_will_not_move_is_avoided_in_full` (seven non-moving members, one control) fails on `main` at the first case (`[0.7529, -0.2262]` against `[0.5307, -0.4747]`) and passes after.
- **Differential.** 60 000 random crowds of 2–9 members with one member made hostile (NaN wish, infinite wish, `max_speed` 0, −1, NaN, 1e20): every other member's velocity must equal its velocity in the same crowd with that member `pref_vel: None`. Before: 29 212 crowds differ (4 783–4 969 of 9 859–10 105 per kind). After: 0. The hostile member's own answer is `None` for a bad wish and `Some(Vec2::ZERO)` for a bad speed in all 60 000, before and after.
- **Predicate.** `usable_speed` against the old guard `max_speed <= 0.0 || !(max_speed * max_speed).is_finite()` on all 2³² `f32` bit patterns: 0 disagree; 1 602 224 127 are usable.
- **Bit-identical sane solves.** 80 bench rows (5 scenes × 4 sizes × fwd/rev/2 threads/14 threads): base hash = new hash in every row; 0 of 20 (scene, size) groups hold more than one hash across order, thread count and run; the four `arrive` lines match, hashes included.
- **Cost.** Four crate copies in one process (two of the base, two of the new), solving one frozen crowd in rotating order, thread CPU time, 1 500 repetitions, run twice: new/base paired median −0.57% … +1.62% over sparse 1k, open 1k, open 5k, dense 1k and rally 1k; the A/A pair of base copies disagrees by up to ±2.5%, so +1.6% is not distinguishable from layout. The full bench at 1 000 and 10 000 agents cannot resolve 5% on this host (the same base binary read 1 983 and 6 620 µs for open 1k in different rounds, load 4–14 on 14 cores); its numbers are in the log as noise.
