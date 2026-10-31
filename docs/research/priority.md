# Right of way — an unequal share of the avoidance between two members that both step aside

`new_velocity_in` gives a neighbour that avoids too exactly half of `u` (`responsibility = 0.5`), and a neighbour that does not, all of it (`1.0`). Those are the only two values the crate has. A game has three situations neither covers: a villager standing in a doorway or at a stall who *can* shuffle but should be walked around, not walked into; a cart, a captain or a quest-giver whose route the crowd should part for; and a unit that is boxed in by a wall and cannot give what half of a head-on dodge asks of it. This note decides what a per-member `priority` means, how two of them combine, and what it must not break, before any code.

## Sources

- **Model:** van den Berg, Lin & Manocha, "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation", ICRA 2008 (`10.1109_robot.2008.4543489`, §III-C "Generalized Reciprocal Velocity Obstacles", read with `distill_search`); van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (the authors' PDF, `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`, fetched 2026-10-31; §4.2). `gameaipro3-ch19-rvo-orca-how-they-really-work` (Sunshine-Hill, Game AI Pro 3 ch. 19, §19.7.1 "Modifying Weights", read in full with `markdown_read`).
- **Shipping game / engine:** the A* Pathfinding Project's `RVOController` (Aron Granberg; a Unity ORCA package that ships in many games), `https://arongranberg.com/astar/documentation/beta/rvocontroller.html`, read 2026-10-31: `Priority`, `Lock When Not Moving`, `Locked`, `flowFollowingStrength`. Unreal Engine's `UAvoidanceManager` and `AvoidanceWeight`, the Epic developer thread "RVO Avoidance" (`https://forums.unrealengine.com/t/rvo-avoidance/380536`, answered by the engineers who wrote it), read 2026-10-31. RVO2's own `Agent.cc` (`snape/RVO2`, `Agent::computeNewVelocity`, `line.point = velocity_ + 0.5F * u;`, line 607; read iteration 4): the one place a share is chosen, and it is a constant.
- **Technical:** IEEE 754 division and `f32::max` semantics for writing a share that is defined for every input: the Rust standard library documents `f32::max` as "Returns the maximum of the two numbers, ignoring NaN. If exactly one of the arguments is NaN (quiet or signaling), then the other argument is returned." (`https://doc.rust-lang.org/std/primitive.f32.html`, read 2026-10-31); `PBR3_03_Shapes` (Pharr, Jakob & Humphreys: a comparison with a NaN is false, `0 · ∞` is NaN). This crate's own `new_velocity_in` input contract (`docs/research/unusable-input.md`): every input gets one documented finite answer.

## Craft principle

The share is a free parameter of the model, with one constraint. The RVO paper states it as a generalization:

> "In the above we have implicitly assumed that each agent takes an equal share in the effort to avoid mutual collisions. However, there may be natural priorities among agents that motivate a different balance, and we can generalize the Reciprocal Velocity Obstacle concept accordingly. Let us denote the share of the effort agent A takes to avoid agent B by α<sub>B</sub><sup>A</sup>. By definition, α<sub>A</sub><sup>B</sup> = 1 − α<sub>B</sub><sup>A</sup>" — van den Berg, Lin & Manocha, ICRA 2008, §III-C

The ORCA paper picks ½ and says why, in one clause: "To 'share the responsibility' of avoiding collisions among the robots in a fair way, robot A adapts its velocity by (at least) ½u and assumes that B takes care of the other half." (§4.2). Its Definition 1 asks that the permitted velocities near each side's optimization velocity be equal in measure, which is the equal split and nothing else. The collision-free guarantee does not need that: A's half-plane passes through `v_A + α_A u`, B's through `v_B − α_B u`, and if `α_A + α_B = 1` the relative velocity of any pair of answers lies past `v_A − v_B + u`, the boundary point of the (convex) velocity obstacle whose outward normal is `n`, whatever the split. The sum is the whole condition, and Game AI Pro states what it costs to break it:

> "In the original RVO paper, agents can be 'weighted' with respect to each other—one agent can dodge by a larger or smaller percentage of the total required dodge than the other, as long as the two weights add up to 1 and the agents agree on what those weights are." … "If two agents are on a collision course, and the weights between them sum to anything less than 1.0, then their decisions each frame are guaranteed not to fully resolve the collision, and eventually they are guaranteed to veer off sharply and collide." — Sunshine-Hill, Game AI Pro 3, §19.7.1

"Agree on what those weights are" is a requirement on the interface: two members each choosing their own share cannot be trusted to add to 1, so the share must be computed from a quantity both sides read, the same for the pair, and complementary by construction.

## How shipping games do it

Both engines that expose the knob compute the pair's split from the two agents' numbers, never from one agent's alone. The A* Pathfinding Project, whose documentation gives the formula:

> "Priority: How strongly other agents will avoid this agent. Usually a value between 0 and 1. Agents with similar priorities will avoid each other with an equal strength. If an agent sees another agent with a higher priority than itself it will avoid that agent more strongly. In the extreme case (e.g this agent has a priority of 0 and the other agent has a priority of 1) it will treat the other agent as being a moving obstacle. Similarly if an agent sees another agent with a lower priority than itself it will avoid that agent less. In general the avoidance strength for this agent is: `if this.priority > 0 or other.priority > 0: avoidanceStrength = other.priority / (this.priority + other.priority); else: avoidanceStrength = 0.5`" — `RVOController`, A* Pathfinding Project

It also ships a separate `Lock When Not Moving`: "Automatically set locked to true when desired velocity is approximately zero. This prevents other units from pushing them away when they are supposed to e.g block a choke point", and says why priority is not that: `flowFollowingStrength` "works in a different way to Priority … This is tricky to achieve with priorities as priorities are all relative, so setting all agents to a low priority is the same thing as not changing priorities at all." Unreal's `AvoidanceWeight` is the same idea under another name; the engineer who wrote `UAvoidanceManager` describes it:

> "The idea behind weights is that that two objects trying to move around each other will act differently, where the lower-weight object will divert course more and the higher-weight object less … It accounts for its weight and the weight of the other object, and it diverts course based on the relative weights." … "if you have an object that isn't going to reciprocate, it should be at max weight." — Epic Developer Community Forums, "RVO Avoidance", posts 2 and 7

So the shipped convention is (a) relative, `other / (this + other)`; (b) equal priorities mean the plain half; (c) the largest weight is an object that does not reciprocate, which is what this crate already calls `avoids: false`; (d) a stationary unit is a separate switch (`locked`, here `pref_vel: None` or `max_speed: 0`), not a large priority. RVO2 has none of it: the share is the constant `0.5F`.

## Model adopted

Every member has a **priority**, `p ≥ 0`, default `1.0`. For a pair that both step aside, member `i` takes this share of the avoidance against member `j`:

```
share(p_i, p_j) = p_j / (p_i + p_j)        // i's line passes through  v_i + share · u
```

and `j` takes `share(p_j, p_i)`, so the two shares add to 1 by construction and both sides read the same two numbers: Game AI Pro's condition, met without the caller doing anything. Equal priorities give exactly ½, so an existing caller's output does not move by a bit. `avoids: false` stays what it is: a neighbour that this step will not move is avoided in full whatever its priority (a high priority is a claim on the way, not the ability to stand still), and walls and `Obstacles` stay at full responsibility.

Written for every `f32`, with no branch on a count and no NaN:

```
let (own, other) = (own.max(0.0), other.max(0.0));   // NaN and negative become 0
if own == other { 0.5 } else { 1.0 / (1.0 + own / other) }
```

| `own`, `other` | `own / other` | share | reading |
|---|---|---|---|
| equal (including `0`, `∞`, `1e30`) | — | `0.5` | the plain ORCA half; `0 / 0` and `∞ / ∞` never reach a division |
| `1`, `1` | — | `0.5` | the default, bit-identical to today |
| `3`, `1` | `3` | `0.25` | the higher takes a quarter; the other takes `0.75` |
| `0`, `1` | `0` | `1` | A*'s "treat the other as a moving obstacle": takes it all |
| `1`, `0` | `∞` | `0` | takes none: the other has nothing to claim and yields |
| `∞`, finite | `∞` | `0` | the Unreal "max weight": never gives way to anything finite |
| NaN or negative | — | as `0` | the lowest priority: it yields, never pushes harder |

Why the ratio is written `1 / (1 + own / other)` and not A*'s `other / (own + other)`: the sum overflows (`f32::MAX + f32::MAX`), and `∞ / (∞ + x)` is NaN; the ratio is `0`, finite or `∞` for every unequal pair (`0 / 0` and `∞ / ∞` are the equal cases, caught first), so the share is always in `[0, 1]`. The two shares of a pair add to 1 to within one rounding (about `1.2e-7`), which is the precision every line in this solver has.

An unusable priority is read as `0`, not refused: it is a member's own field and it cannot hide a neighbour (skipping one lets members walk through it), so the only safe reading is the one that makes the member give way. A NaN is a bug in the caller; it costs that member its right of way for as long as the bug lasts and nothing else.

Where it lives: `Agent` gains `priority: f32` (its own and each neighbour's), and `CrowdMember` the same field, which `Crowd` copies into the `Agent` it builds for the member and for every neighbour offered, so `Crowd::solve` and a hand-rolled ring around `new_velocity_in` read the same numbers. One function, `share(own, other)`, replaces the literal `0.5` at the one place `responsibility` is chosen.

**Touching discs split evenly.** The share is a claim on the way to avoid a collision that has not happened. Once two discs already overlap (the solver's second branch, `dist² ≤ (rᵢ + rⱼ)²`) there is no way left to claim: the line is the push that separates them over this step, and both members take `½` whatever their priorities (a neighbour with `avoids: false` still gets all of it, and the two shares still add to 1, which is all the guarantee asks). This is a measured change, not an argued one: the first implementation applied the share in contact too, and a high priority on one member of a touching pair left the other to do all of the separating, so the 100-agent circle swap's deepest overlap went from `0.040` of a radius at equal priority to `0.120` at a ratio of 2 and `0.209` at 8, and the city's from `0.047` to `0.131` at 8. With the even split in contact they are `0.081` and `0.071`, and `0.053` and `0.062` (Results, below). RVO2's overlap branch is `0.5F * u` like everything else, so for equal priorities this is the same line to the bit.

## Parameters

- `priority` default `1.0` in `Agent::default()` and in every example, test and README line that builds one. It is a ratio, so only comparisons mean anything: `2` against `1` is the same as `20` against `10`. A crowd that never sets it is plain ORCA.
- No constant enters the solver.

## Validation targets

- **Unchanged when equal.** Every bench `hash` row (all scenes, 100 to 10 000 agents, both member orders, 1, 2 and the default number of threads) and every `arrive` row equal before and after; `head_on`, `wall_corridor` and `street` byte-identical. `share(a, a)` is exactly `0.5` for every `f32` pattern that is not NaN, and for NaN and negative pairs too.
- **The share.** `share(a, b) + share(b, a)` within `2.4e-7` of 1 for a million random pairs (log-uniform over `1e-30..1e30`, plus `0`, `∞`, NaN, negative); `share` in `[0, 1]` for every `f32` pair including the specials; never rises when `own` rises; `share(0, 1) = 1` and `share(1, 0) = 0`.
- **It is what A* and Unreal describe.** A head-on pair at priorities 3 and 1 sidesteps in the ratio 1 : 3 (the higher takes a quarter), and the two peak sidesteps add to the equal-priority pair's, within 5%: the pair's total avoidance is unchanged, only who gives it.
- **Collision-free.** The ORCA guarantee under unequal shares, tested the way the paper states it: for 100 000 random two-agent configurations on a collision course with random priorities, both apply their answers and the closest approach over the horizon is at least the combined radius to within `1e-4` of it. The same sweep at equal priorities is the control. Without an infeasible program the guarantee is exact; the speed disc is made large enough not to bind.
- **It agrees with `avoids: false`.** A member whose neighbour has priority `0` and a member against a neighbour with `avoids: false` give bit-equal answers, on random scenes.
- **Touching discs split evenly.** For random overlapping pairs with random priorities (zero, infinite and NaN included) each member's answer is the one it gets at equal priorities, to the bit; the test fails with the rule removed. The circle swap's and the city's deepest overlap at ratios 2 to 8 is no more than twice the equal crowd's.
- **`Crowd` reads it.** A crowd of members solved by `Crowd::solve` equals the same members solved by hand through `new_velocity_in` with priorities in the `Agent`s, over random priorities (the crowd module's own hand-rolled ring test), and the hostile sweep draws NaN, negative, infinite and huge priorities: 0 non-finite answers, 0 panics.
- **Corridor.** In `bench` with the rightward stream at priority `P` (new knob `ORCA_PRIORITY`): at `P = 1` the `arrive` rows are byte-identical to today; for `P = 2, 4, 8` the high stream's delay (arrival time over the straight-line time, minus 1) falls and the other stream's rises, and wall depth is no worse than at `P = 1`. The draft also said overlap "no worse"; the first run, before the in-contact rule, read up to 5.2× at a crush and the final rule says at most twice (the corridor at 8 and 16 reads 1.12× and 1.17×).
- **Allocation and cost.** `tests/alloc.rs` unchanged and green; `scripts/paired.sh` new / base inside the A/A floor, because the only added work is one compare per neighbour offered.

## What great looks like

A villager standing at a market stall is given priority 8: walkers on the street take eight ninths of every dodge and pass around him with a few centimetres of his own drift, where today he takes half of each and is slowly shoved along the stall. A cart in a street of walkers has priority 4 and its path bends by a quarter of what an equal member's would while the walkers part, and the walkers' detour in total is no worse than the equal case, so the street does not jam. A captain with a wish that is walking to a gate has priority 2 in a column of recruits. The host sets one `f32` per member, or none. Two members that both think they own the way are still resolved by reciprocity, not frozen: equal priorities are the plain ORCA half.

What it will not do, and the README says so: a high priority does not make a member immovable (that is `pref_vel: None` or `max_speed: 0`, which is avoided in full); it does not lift the speed disc or the walls from a low-priority member boxed between two high-priority ones, whose program can still be infeasible (the answer is then the safest velocity, as it is for everyone); and it does not make a pair pass on a chosen side (that is `CrowdParams::nudge`).

## Deviations

- **From RVO2:** `Agent.cc:607` is `line.point = velocity_ + 0.5F * u;` for every agent neighbour. The share is now `share(own, other)`, which is `0.5` for equal priorities; `avoids: false` was already this crate's departure (iteration 4).
- **From ORCA's Definition 1:** the optimality criterion ("the amount of velocities close to the optimization velocity is equal for A and B") is stated for the equal split. An unequal split keeps the collision-free guarantee (the shares add to 1) and gives up that fairness criterion by choice, which is what a priority is.
- **Assumed decision (owner may override):** the name and the form. Options: (a) `priority`, a relative non-negative weight with `share = other / (own + other)`, the A* form; (b) `responsibility`, an absolute share per member in `[0, 1]`; (c) `weight`, Unreal's name, with the same relative form. Picked (a). (b) is the form Game AI Pro warns about: two members each setting a share have no reason to add to 1, and a pair that adds to less "is guaranteed … to veer off sharply and collide"; both engines that ship the knob compute it from the pair. (c) is the same maths under a name that reads as mass. `priority` says what a game developer means, and 0.5 / equal stays the default.
- **Assumed decision (owner may override):** `Agent` and `CrowdMember` gain a public field, a source-breaking change for a struct literal. Options: a field on both; a parallel slice or closure `priority(i)` passed to `solve` beside `sees`; a builder with a `Default`. Picked the field: a parallel slice drifts out of step with the member order (the one thing `Crowd` reorders internally), a closure is a call per neighbour offered, and the release these sit in (0.3.0, `CrowdParams::obstacle_horizon`) is already breaking. Cost: a consumer builds `CrowdMember { …, priority: 1.0 }`, one line.
- **Assumed decision (owner may override):** an unusable priority (NaN, negative) is read as `0`, not refused and not read as the default. Options: refuse like an unusable position (the member holds still and is left out); read it as `1`; read it as `0`. Picked `0`: refusing hides a perfectly good body from its neighbours, `1` hands a broken member a full claim, and `0` is the only reading under which the bug cannot make a member walk into anybody.
- **Design default:** `avoids: false` outranks any priority. A neighbour that will not move is avoided in full even at priority `∞`: a claim on the way does not stop a unit from being walked through.
- **Design default (measured):** discs that overlap split the push apart evenly whatever their priorities (see Model adopted). Without it a high priority multiplied the crush's overlap by up to 5.2×; with it the worst is 2.0×.

## Pending index

- The A* Pathfinding Project and Unreal Engine pages are web documents, not home-still documents (`paper_download` is for papers); quoted above from the pages as read on 2026-10-31.
- van den Berg et al., "Reciprocal n-body Collision Avoidance": still not a home-still document (see `corpus.md`); read from the authors' PDF.
- `distill_search` ranked the ICRA 2008 chunk first for the "unequal responsibility" query and the Game AI Pro 3 chunk for the weights query; no further indexing is needed for this item.

## Results (iteration 11)

Every number below was measured on this host (macOS arm64, 14 cores shared with other jobs) on the branch's tree; the commands are in the loop log.

- **Unchanged when equal.** `bench` before (`6cd83cd`) and after, three full or serial runs each, and a fourth full run on the final tree: every row of all 40 (scene, size) groups at both member orders and at 2 and 14 threads, and every `arrive` row, is identical in `hash`, `overlap_max`, `wall_max`, `speed_max`, `allocs`, `cand_mean` and `cand_max`, once the timing columns and the scaling exponents computed from them are removed (`diff` is empty on 179 lines). `head_on` (its first three passes), `wall_corridor` and `street` are byte-identical. `equal_priorities_are_plain_orca_at_any_value`: 2 000 random scenes, ten values each, equal to the bit.
- **The tests have teeth.** Making `share` return its square (the pair's shares then add to less than 1) fails `a_head_on_pair_splits_the_dodge_by_priority`, the standing-member test and `an_unequal_split_keeps_the_pair_collision_free`; handing `Crowd`'s neighbours priority `1.0` fails `crowd_reads_priority_for_the_member_and_for_its_neighbours` and the dense-crowd reference test; handing the member its own `1.0` fails the dense-crowd test; switching the in-contact rule off fails `overlapping_discs_push_apart_evenly_whatever_their_priorities`. Each was reverted.
- **The guarantee.** 30 000 random pairs (positions, velocities, wishes, radii, horizons, priorities log-uniform over `1e-3..1e3` with 10% zero and 10% infinite): the worst gap past touching is `−0.00000` for the unequal sweep and for the equal control (nobody ends up inside anybody); more than 3 000 of the control's cases made a member dodge, so it is not vacuous.
- **Cost for a caller who sets nothing** (`scripts/paired.sh 6cd83cd 300` on the final tree, four crate copies in one process, thread CPU time, rotating order; new / base, with the A/A floors beside): open 100 / 1 000 / 10 000 `+0.1%` / `−0.1%` / `−0.5%` (floors up to `3.7%`); circle `+0.4%` / `+0.6%` / `−0.3%` (`≤ 1.2%`); corridor `+0.5%` / `+0.3%` / `+0.2%` (`≤ 1.4%`); rally `−0.1%` / `+0.9%` / `+0.2%` (`≤ 0.9%`); the checksums equal the base's. An earlier run on the tree before the in-contact rule read `−1.0%` to `+1.2%` against floors up to `3.2%`. **A first paired run in this pass compared against a stale build:** `git archive` stamps the base's files with the commit's time, cargo treats a path dependency as fresh when its sources are older than its build, and the copies' package ids are the same in every run, so in the shared `CARGO_TARGET_DIR` the base `6cd83cd` was answered with iteration 10's artifact (`02ecd64`). It surfaced when a later run with a base *older than the previous run's* failed to compile. `scripts/paired.sh` now touches the base's files; the numbers above are from after the fix. The macro bench's medians cannot resolve this on the shared host (the same code reads `bottleneck` 100 at 248 and 101 µs and `bottleneck_obst` 100 at 144 and 232 µs); the scaling exponents 1k→10k, median of three, before → after, are open 0.925 → 0.959, circle 1.057 → 1.065, corridor 1.087 → 1.061, bottleneck 0.911 → 0.881, rally 1.081 → 1.068, bottleneck_obst 1.029 → 0.945, city 1.065 → 1.078, city_cells 1.086 → 1.057, city_boxes 1.114 → 1.079 and city_pushout 1.065 → 1.179 (one of the three after-runs read 1.197 there and `city_boxes` 1.479 while the sweeps below were running; the other two read 1.080 and 1.075 / 1.054 and 1.079).
- **What it does to a crowd** (`ORCA_PRIORITY=p`; the stream heading right at `p`, the rest at 1; means over seeds; delay is the arrival time beyond the straight-line time, mean over arrivals; with the in-contact rule):

  | scene (agents, crowds) | `p` | delay right | delay others | mean | stalled > 2 s | deepest overlap |
  |---|---|---|---|---|---|---|
  | city, 0.6 lanes (1 000, 8) | 1 / 2 / 4 / 8 | 2.61 / 2.33 / 2.45 / 2.15 s | 2.84 / 3.10 / 4.23 / 4.56 s | 2.73 / 2.71 / 3.34 / 3.36 s | 1.9 / 3.5 / 4.8 / 10.8 | 0.047 / 0.053 / 0.074 / 0.062 r |
  | city, 1.0 lanes, 0.2 m from the brick (1 000, 8) | 1 / 2 / 4 / 8 | 5.64 / 4.77 / 3.84 / 4.08 s | 5.56 / 6.43 / 7.91 / 9.30 s | 5.60 / 5.60 / 5.87 / 6.69 s | 19.3 / 22.5 / 28.3 / 27.9 | 0.078 / 0.064 / 0.083 / 0.073 r |
  | circle swap (100, 8) | 1 / 2 / 4 / 8 | 5.59 / 5.07 / 4.76 / 4.81 s | 6.47 / 5.98 / 6.22 / 5.59 s | 6.03 / 5.53 / 5.49 / 5.20 s | 0 / 0 / 0 / 0 | 0.040 / 0.081 / 0.064 / 0.071 r |
  | corridor (100, 64) | 1 / 2 / 4 / 8 / 16 | 0.67 / 0.64 / 0.61 / 0.52 / 0.69 s | 0.53 / 0.56 / 0.59 / 0.68 / 1.31 s | 0.60 / 0.60 / 0.60 / 0.60 / 1.00 s | 0.08 / 0.05 / 0.23 / 0.09 / 0.36 | 0.069 / 0.051 / 0.049 / 0.077 / 0.081 r |

  Every crowd arrives at every priority (`arrived` 1.000, `t90_s` within 1.4 s of the equal crowd's) and the deepest penetration past a wall is `0.0000` in the corridor and circle, `0.0001` in the tight city at every priority including 1 (box-distance rounding, also in the base). The deepest overlap is at most twice the equal crowd's in every row.
- **Why the in-contact rule.** The first implementation applied the share in contact too. The same crowds read: circle swap `0.040 / 0.120 / 0.104 / 0.209 r`, city (0.6 lanes) `0.047 / 0.064 / 0.069 / 0.131 r`, tight city `0.078 / 0.044 / 0.100 / 0.172 r`, corridor at 16 `0.108 r` with mean delay `1.11 s`: the high-priority member of a touching pair did none of the separating. Both sets of numbers are from the same seeds.
- **A stall-holder in a stream** (`a_stall_holder_with_priority_is_walked_around_by_a_stream`, 30 walkers past one standing member with a zero wish, three crowds, mean of each crowd's peak): pushed `0.406 / 0.317 / 0.221 / 0.137 / 0.042 / 0.000` m at priority `1 / 2 / 4 / 8 / 32 / ∞`, the walkers' extra path `0.129% … 0.133%`, overlap `0.0000 r`. A lone pair gives the cleaner version (`a_standing_member_with_priority_is_pushed_aside_by_the_share_it_is_owed`: `2/9` of the equal push at 8 against 1, to `1e-3`). The same 25 stall-holders as a 5 × 5 lattice 0.9 m apart (a throwaway, measured before the in-contact rule; no discs touch there) show the other end: with priority 32 the block stops being shoved and becomes a wall to 60 random walkers, 378 of 480 of whom had crossed in 50 s against 479 of 480 at priority 1. A high priority on a *group* is a blockade, as `max_speed: 0` is.
- **The premise of backlog item 1 does not hold.** Iteration 10 ranked priority first on the guess that giving the member that *can* step aside the larger share would free a head-on walker pinned against a wall. A static per-stream priority does not: with the lanes 0.2 m from the brick the stalled count is `19.3` at equal priority and `22.5 / 28.3 / 27.9` at 2 / 4 / 8, because both ends of a head-on lane pair are walled in, so whichever takes the larger share has nowhere to put it. A priority that follows who is walled in would be a different feature (a per-member measure the crate would derive from `walls`), and is left on the backlog as an open question rather than a result.
- **Hostile input.** The single-solve sweep (20 000 cases, now with a priority field drawn hostile in 1 in 19) and the crowd sweep (3 000 steps with priorities from `value(0.0, 4.0)` and the hostile table) give 0 non-finite answers and no panic.
- **Allocation.** `tests/alloc.rs` passes unchanged (7 of 7): the share is arithmetic on two `f32`s and adds no buffer.
- **Not re-run:** the brute-force optimality check of iteration 1 (the linear program is untouched; only where one half-plane sits changed, and the collision-free sweep checks every answer is inside its half-plane); the RVO2 differential.

### Critique (before the verdict)

- A high ratio is not free for the member that gives way: at 8 in the city the mean delay rises 23%, and the walkers held back to let a stream through stop for over two seconds six times as often (10.8 per 1 000 against 1.9); at 2 the mean is flat and the stalls double. The README says to use about 2 for a stream and a large value only for a member to be treated as a wall.
- The bench's streams are "heading right", not a role a game has; a game's priority comes from what a member is (stall-holder, cart), and the only numbers here for that are the single stall-holder and the lattice.
- Overlap in a crush still rises at unequal ratios, to at most twice the equal crowd's (the 100-agent circle swap at 2: `0.040 → 0.081 r`), though the even split in contact took it from 5.2×. The remainder is the avoidance landing on the member with less room before the discs touch.
- `priority` does not replace `pref_vel: None` for a unit that must not move, and the README does not let it be read that way.
- The change is source-breaking for a struct literal; the migration is one line, the release is 0.3.0 already.
- The in-contact rule is a design decision made from one measurement (overlap), not from a source; RVO2, A* and Unreal do not state one. It is recorded as a deviation and tested to the bit.
- No GIF or still was regenerated: no default behaviour moved and this host has no display (backlog 5).

Verdicts: **semantics (share, guarantee, in contact, `Crowd` threading, hostile input): GREAT**; **cost for a caller who sets nothing: GREAT**; **effect in a crowd: GREAT** at a ratio of 2 to 4 for a stream (the mean is the equal crowd's at 2, the stream's right of way is real) and for a stall-holder at any ratio (his push falls with it, to nothing at `∞`), with the price of a large ratio on a street (stalls, a slower mean) measured and written where a caller reads it.
