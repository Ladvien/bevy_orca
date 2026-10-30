# Crowd bench — measuring what a caller pays and what a caller sees

The instrument every later pass is graded with: `examples/bench.rs`. Four scenes at four sizes, timed the way a host pays for a `Crowd::solve` step, plus the correctness a host sees (deepest overlap, wall penetration, arrivals, stalls) and a determinism hash across member order and thread count.

## Sources

- **Model:** `10.1109_robot.2008.4543489` (RVO, ICRA 2008: the Circle experiment and its linear scaling claim); `10.1109_tro.2011.2120810` (HRVO, T-RO 2011: collisions and cost per step on a fixed circle as the count rises); `10.1103_physreve.51.4282` (social force: lanes in counter-flow, oscillation at a narrow door); `10.1073_pnas.1016507108` (Moussaïd et al.: stopped means under 0.05 m/s; bottleneck turbulence).
- **Shipping game:** `gameaipro3-ch19-rvo-orca-how-they-really-work` (Sunshine-Hill: ORCA's LP fails in small groups); `s2008-advances-full-course-notes-4` (AMD's Froblins crowd: deadlock at sinks).
- **Technical:** `eth-cgl-sim_anim-Sol14a` (Ihmsen et al., SPH STAR: "If the cell size is equal to the kernel support, 3^3 cells have to be queried in 3D, which is optimal", and "cells that are close in memory are necessarily not close in space which reduces the cache-hit rate for the neighborhood query" — the 2-D form is the 3×3 block `Crowd` scans with its cell equal to `neighbour_range`); `book_real_time_collision_detection_ericson` (uniform grids).

## Craft principle

An avoidance step is linear in agents only if the neighbour search is; the solver itself is constant work per agent. The RVO paper says so of its own Circle experiment:

> "Clearly, the amount of time needed to generate one frame (i.e., process one cycle in the simulation) scales linearly with the number of agents. Only the neighbor selecting routine has a quadratic nature, but this step is negligible in the total running time." — `10.1109_robot.2008.4543489`, p. 6

So the bench quotes the exponent of step time from 1 000 to 10 000 agents at constant density, and the neighbour candidates scanned per agent beside the neighbours used, so a search that grows faster than the crowd is visible apart from the solver.

The scenes are the ones the pedestrian literature uses to find failures, because each stresses one thing. Counter-flow is where lanes must form:

> "Above a critical pedestrian density one can observe the formation of lanes consisting of pedestrians with a uniform walking direction." — `10.1103_physreve.51.4282`, Fig. 2

and density on a fixed circle is where reciprocal methods start to collide: HRVO's Table III reports that "as the number of virtual agents exceeds 100, a small, but increasing, number of collisions per time step are observed" (`10.1109_tro.2011.2120810`, p. 9). A stall is a speed under 0.05 m/s, Moussaïd et al.'s definition of a stop (`10.1073_pnas.1016507108`, Fig. 4C).

## How shipping games do it

Games test exactly the cases where ORCA's guarantee runs out, because those are the ones players see:

> "The problem is that ORCA's solver often does not succeed. [...] It is easy for a small number of nearby agents to completely rule out all velocities." — `gameaipro3-ch19-rvo-orca-how-they-really-work`, the section before §19.7.3 "Time Horizons"

> "While the local avoidance model will prevent collisions in very densely packed situations, scenarios arise where agents can deadlock and will become stuck. This typically happens at sinks in agent navigations such as at a small goal." — `s2008-advances-full-course-notes-4`, p. 71 (AMD Froblins)

Hence a bottleneck (a sink: a room draining through a gap two agents wide) and a circle swap (every agent's goal across one contested centre) beside the open field and the corridor, and an overlap measured on every step rather than at the end.

## Model adopted

One example, `examples/bench.rs`, using the public API only (so the same file measures any earlier commit):

- **Scenes**, all with `sees = |_, _| true`:
  - `open` — random walkers on a jittered grid at 0.5 agents/m², each walking to goals drawn from its own key (not from a shared stream, so member order cannot change them).
  - `circle` — agents on a ring at 1 m of arc each, goal the antipode.
  - `corridor` — a 6 m wide corridor along x, walls at y = ±3 as half-planes, agents interleaved at 0.5/m², half walking +x and half −x to the far end.
  - `bottleneck` — a room at 1 agent/m² left of a wall at x = 0 with a 1.4 m gap; walls are the two wall segments as nearest-point half-planes.
- **Walls** as the caller builds them today: for a convex wall piece at distance `d` with unit direction `b` toward its nearest point, `(b, max(d − r, 0) / dt)`. Distance to a convex set is convex, so `d(p + v·dt) ≥ d(p) − (b·v)·dt ≥ r` — the constraint cannot be crossed in one step in exact arithmetic.
- **Timing:** `Crowd::solve` with the member slice in its given order, 20 warm-up steps then 480 timed steps; median and p99 per step; agents per ms = agents / median ms.
- **Candidates:** the number of `sees(i, j)` calls per solved agent per step — the in-range members `Crowd` sorts — mean and maximum.
- **Determinism:** after 500 steps, an FNV-1a hash of every agent's position and velocity bits in key order, for the given order (`order=fwd`), the reversed slice (`order=rev`), and the caller's parallel ring at 2 and the default number of threads: the grid, the `(distance, key)` sort, the key-hashed nudge and `new_velocity_in` on `std::thread::scope` workers with one `OrcaScratch` each. All four must be equal.
- **Correctness**, measured on the timed run: the deepest overlap between agents as a fraction of the radius, and the deepest wall penetration.
- **Arrivals:** a second section runs the circle swap at 20, 100 and 1 000 agents and a 2×50 two-way corridor to a budget of three times the straight-line time, and reports the fraction that arrived and the agents that stayed under 5% of their preferred speed for more than 2 s while away from their goal.

## Parameters

| Parameter | Value | Source |
|---|---|---|
| radius | 0.3 m | design default (a shoulder half-width) |
| max / preferred speed | 1.3 m/s | design default, near the free walking speed of the pedestrian literature |
| `dt` | 1/30 s | design default: a 30 Hz AI tick (RVO2's own demos use 0.25 s) |
| time horizon | 2 s | `CrowdParams::new` default |
| neighbour range | 3 m | design default: 10 radii |
| max neighbours | 10 | `CrowdParams::new` default, RVO2's `maxNeighbors` in its examples |
| nudge | 0.05 | `CrowdParams::new` default |
| sizes | 100, 1 000, 5 000, 10 000 | the loop's section 2 |
| stall | < 5% of preferred speed for > 2 s, away from goal | the loop's section 3; 5% of 1.3 m/s = 0.065 m/s ≈ Moussaïd's 0.05 m/s stop |

## Validation targets

- Deepest overlap between avoiding agents: 0 beyond float rounding (quoted as a fraction of the radius; anything over 1% is a finding).
- Deepest wall penetration: 0.
- Scaling exponent 1k → 10k at constant density: ≤ 1.15.
- Hashes: equal across `order=fwd`, `order=rev` and every thread count.
- Arrivals within 3× straight-line time: 100% in the circle swap at 20 and 100 and in the 2×50 corridor; none stalled.

## What great looks like

A line per case a reader can paste into a log and compare by eye: the scene, the count, the threads, the cost in microseconds and in agents per millisecond, the search's work beside the solver's, and one hash. Nothing a host would not see — no internal counter the crate does not expose — so the same file measures every commit before and after a change. A circle swap whose 1 000 agents all arrive with no disc ever inside another, a corridor where the two streams lane up and pass, and a bottleneck that drains at a steady rate without anyone stepping through the wall.

## Deviations

- The crate cited DOI `10.1109/TRO.2011.2120810` for "Reciprocal n-Body Collision Avoidance". That DOI is HRVO (Snape et al., T-RO 2011, `paper_get` confirms the title). ORCA's DOI is `10.1007/978-3-642-19457-3_1` (`paper_get`: "Reciprocal n-Body Collision Avoidance", van den Berg, Guy, Lin, Manocha, 2011). Fixed in `src/lib.rs` and `NOTICE`.
- Candidates are counted through `sees` because that is what a caller can observe; members inside the 3×3 cells but beyond range are scanned too and not counted. A pass that changes the hash must add a count of those if it needs one.

## Pending index

- ORCA, DOI `10.1007/978-3-642-19457-3_1`: not downloadable (no open-access copy; CORE needs a key). Author copy: `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`.
- Seyfried et al. bottleneck flow / fundamental diagram; Singh et al. SteerBench: not in the corpus, not searched for download this pass.
