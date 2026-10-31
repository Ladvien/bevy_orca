# Static obstacles — RVO2's obstacle lines, built from the caller's polygons and walls

Every consumer of this crate writes the same thing by hand around it, and writes it worse than the reference implementation does. `Crowd::solve` takes `walls(i, &mut buf)`, a closure that fills `(unit normal toward the wall, max approach speed)` pairs per member per step, and everything about *which* walls, from *what* geometry, is left to the host:

- **Sanctuary** (`src/crowd.rs`, read 2026-10-31) passes `|_i, walls| walls.clear()` and says why: "Nothing is a wall here: the baked solids are a push-out after the solve, not an ORCA half-plane, because `StaticSolids` holds boxes rather than the round half-plane form ORCA takes." So its villagers are not steered around a building at all: they walk into it, the host pushes them out of the box after the step (`push_out_of_box`), and their neighbours, who assumed the villager would go where the solve said, are wrong about it.
- **bevy_emerge** (`crates/bevy_emerge_mapper/src/crowd_preview.rs`, `step`, read 2026-10-31) builds four axis-aligned half-planes per walker from the 4-neighbour solid cells (`edge = cell_centre + b * cell_size/2`, `gap = (edge - pos)·b - radius`), then after the solve slides or stops any walker that still lands in a solid cell. A solid cell that touches the walker only at a corner (diagonal) casts no half-plane at all.
- **This repository's bench** (`examples/bench.rs`, `World::walls`) does the segment version: the closest point of each wall segment, `(direction to it, (d - r) / dt)`.

The two that build half-planes (emerge and the bench) share two properties. The approach speed is the remaining clearance over **`dt`**, so a walker closes on a wall at full speed and brakes in its last step (no anticipation), and the bench's half-plane at a corner is cast toward the corner *point*, so a walker cannot slide around a convex corner it is still approaching obliquely. (Sanctuary builds none, which is the same absence of anticipation by another route.) ORCA's own answer to both is the obstacle branch of `computeNewVelocity`, and this crate is, by its own `NOTICE`, RVO2's solver without it.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (the authors' PDF, `gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`, read in full on 2026-10-31 with the `read` tool: section 5.4 "Static Obstacles", Fig. 6; not a home-still document, see `corpus.md`); van den Berg, Lin & Manocha, "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation" (`10.1109_robot.2008.4543489`: static obstacles are obstacles "with zero velocity" in the combined velocity obstacle).
- **Shipping game / engine:** Curtis, Best & Manocha, "Menge" (`10.17815_cd.2016.1`, Listing 1: a shipped ORCA profile, `tau="3.0" tauObst="0.15"`); Sunshine-Hill, "RVO and ORCA: How They Really Work" (`gameaipro3-ch19-rvo-orca-how-they-really-work`, 19.7.3 "Time Horizons": the wall behind the goal, and what RVO2 and others do about it); Recast/Detour `DetourCrowd/Source/DetourLocalBoundary.cpp` (`recastnavigation/recastnavigation`, `main`, read 2026-10-31: the wall segments an agent is steered against); RVO2 `src/Agent.cc` (`snape/RVO2`, `main`, `computeNewVelocity` lines 270–535 and `insertObstacleNeighbor` 644–672), `src/KdTree.cc` (`queryObstacleTreeRecursive` 442–469) and `src/RVOSimulator.cc` (`addObstacle` 155–194) — GitHub, read, and built (clang++ -O2) as the differential oracle.
- **Technical:** Ericson, *Real-Time Collision Detection* ch. 7 (`book_real_time_collision_detection_ericson`: the uniform grid, an object linked into every cell it overlaps, and what a too-fine grid costs); and the crate's own `Crowd` grid (`docs/research/spatial-hash.md`), whose sorted-cell lookup this reuses for rooms of any extent.

## Craft principle

The paper gives the construction and the three rules that make it a different thing from a wall half-plane. The construction:

> "We basically follow the same approach as above, with a key difference being that obstacles do not move, so the robots should take full responsibility of avoiding collisions with them. We can generally assume that obstacles are modeled as a collection of line segments. [...] the velocity obstacle VO^τ_{A|O} induced by the obstacle O is defined as `{v | ∃t ∈ [0, τ] :: t·v ∈ O ⊕ −D(p_A, r_A)}`" (§5.4, eq. 11)

Full responsibility: where an agent pair splits the avoidance (`v + ½u`), the obstacle's line is the velocity obstacle's own boundary. The convexity trick is the same as for agents: the complement of the velocity obstacle is not convex, so

> "we define the set of permitted velocities for A with respect to O as the half-plane ORCA^τ_{A|O} whose delimiting line is the tangent line to VO^τ_{A|O} at the closest point to v_A^opt on the boundary of VO^τ_{A|O}" (§5.4)

(the paper takes `v_opt = 0` for obstacles; RVO2 projects the **current velocity**, which gives the legs that let an agent slide around a corner — see Deviations). Two more rules:

> "We can typically use a smaller value of τ with respect to obstacles than with respect to other robots, as robots should typically not be 'shy' to move towards an obstacle if this is necessary to avoid other robots. On the other hand, the constraints on the permitted velocities for the robot with respect to obstacles should be *hard*, as collisions with obstacles should be avoided at all cost. Therefore, when the linear program of Section 5.1 is infeasible due to a high density of robots, the constraints of the obstacles are not relaxed." (§5.4)

> "the half-planes of permitted velocities with respect to obstacles as defined above only make sure that the robot avoids collisions with the obstacle; they do not make the robot move around them. The direction of motion around obstacles towards the robot's goal should be reflected in the robot's *preferred velocity*, which could be obtained using (efficient) global path planning techniques." (§5.4)

So: a second, shorter horizon; hard lines placed ahead of every agent line and never relaxed (this crate's `walls` already are: `linear_program3` starts past them); and no pathing.

## How shipping games do it

Menge, the crowd framework from the ORCA authors' group, ships its ORCA profile with the two horizons far apart:

> `<ORCA tau="3.0" tauObst="0.15" />` — Menge, Listing 1 (periodic hallway), 0.19 m agents at 1.04 m/s

A 3 s horizon for agents and 0.15 s for obstacles: an agent anticipates a wall for 0.15 × 1.04 ≈ 0.16 m plus its radius.

Sunshine-Hill (Game AI Pro 3, 19.7.3) says why a single long horizon is wrong for walls:

> "Another, simpler form can be demonstrated with a single VO agent traveling north toward a goal point with a wall behind it. All northward velocities are blocked by the wall's static velocity obstacle, so the agent can never reach the goal. The original RVO reference library does not exhibit this problem, because it diverges from the paper and ignores collisions with static obstacles if they are further than the maximum stopping distance of the agent. Other RVO implementations address the problem by defining a maximum time-to-collision (TTC) beyond which projected collisions are ignored, causing the agent to slow as it approaches the obstacle, keeping its TTC above the threshold."

RVO2 is the first of those: `Agent::computeNeighbors` queries obstacles within `timeHorizonObst_ * maxSpeed_ + radius_` and ignores the rest (`Agent.cc:258`). A horizon also is the one number that says how early a walker veers from a wall, which is why it is a parameter and not a constant.

Recast/Detour does not make walls hard at all. `dtLocalBoundary::update` keeps the **eight** nearest wall segments of the polygons around the agent (`MAX_LOCAL_SEGS = 8`) within `collisionQueryRange` (`DetourLocalBoundary.cpp:84–120`: "Skip too distant segments", then `addSegment(distSqr, s)`), and the obstacle-avoidance sampler scores candidate velocities against them (`isectRaySeg`, a penalty). That is a cap and a soft cost; ORCA's answer is neither, and the crate follows it: every segment in range is a hard line, nearest first, no cap (a wall detailed enough to put more than a handful of segments within a few metres is rare, and a dropped wall is a walker through it).

RVO2 supplies the obstacle itself: polygons of at least two vertices (two vertices are a thin wall, passable from neither side), vertices listed counter-clockwise so that the agent is on the **right** of every edge it sees (`KdTree.cc:448–462`: an edge is a neighbour only `if (agentLeftOfLine < 0.0f)`), a convexity flag per vertex (`RVOSimulator.cc:176–183`: `leftOf(prev, here, next) >= 0`, and a collinear vertex is convex), and a kd-tree of the edges.

## Model adopted

**`Obstacles`**: an immutable, caller-owned set of edges, built once. `ObstaclesBuilder` takes four kinds of geometry and refuses what the solver cannot use:

- `add_solid(&[Vec2])` — a polygon whose inside is solid, **either winding** (the builder orients it); a building, a pillar, a wall with thickness.
- `add_enclosure(&[Vec2])` — a polygon whose inside is the open floor and whose outside is solid, either winding; a room, a courtyard, a map's outer boundary.
- `add_wall(a, b)` — a thin wall, solid to neither side and passable from neither (RVO2's two-vertex obstacle).
- `add_box(centre, half_extents)` — `add_solid` of an axis-aligned rectangle (a `StaticSolids` entry, a solid cell).

RVO2 asks the caller for counter-clockwise polygons and, given a clockwise one, makes the agent see every edge from its left, which is to say not at all: the building is walked through, silently. Stating *what is solid* and orienting it is the whole reason the type exists; it is also what a map editor hands over (outer boundary and holes).

**The obstacle branch** is RVO2's `computeNewVelocity` lines 275–535, case for case (collision with a vertex or an edge: a line through the origin; legs and cutoff; the foreign-leg rule; the cutoff circles; "already covered"), in the crate's own representation: every line is emitted as `(b, c)` with a unit `b` and `c = b·p`, which is a wall exactly as `new_velocity_in` takes them (`v·b <= c`, stored at its foot). **So `new_velocity_in` does not change**: obstacle lines are walls that were derived from geometry instead of by hand, they go ahead of every neighbour line, and they are never relaxed. `Obstacles::push_walls(scratch, agent, max_speed, horizon, walls)` appends them for one agent, for a caller on the single-agent API; `Crowd::solve_around(&obstacles, …)` does it for every member after the caller's own `walls` closure.

**Neighbour search** by a uniform grid over the edges, built once, in compressed rows: cell size from the geometry (about one edge per cell), each edge linked into the cells it crosses (a column-by-column interval, not its bounding box, so a diagonal wall costs `O(length / cell)` not `O(length²)`), the cell count capped. A query visits the cells under the box `pos ± range`, keeps the edges whose nearest point is within `range` and whose right side the agent is on, sorts them nearest-first with the edge index as the tie, and removes the duplicates an edge crossing several cells produces. Everything is ordered by the geometry and nothing by the schedule, so the result is a function of the world.

## Parameters

- **`CrowdParams::obstacle_horizon`** — seconds of lookahead against obstacles, separate from `time_horizon` (paper §5.4, Menge's `tauObst`, RVO2's `timeHorizonObst`). Default 0.5 s, a design default inside the range measured flat (0.034 to 1 s, `Results`), not Menge's 0.15 s, which is tuned to 0.19 m walkers at 1.04 m/s and a 0.1 s step. `solve_around` raises it to `dt` if shorter: a shorter horizon lets an agent cross a wall within a step.
- The query range is `obstacle_horizon · max_speed + radius`, RVO2's.
- Grid cell size: automatic (not a caller dial); the cell-count cap is a constant.

## Validation targets

- **Wall penetration is exactly 0** (to `1e-4` m of rounding) over every step of every scene, for solids, enclosures and walls, at 100 to 10 000 agents; a thin wall and a box built of touching boxes included.
- **A differential against RVO2** (built as shipped, `getAgentNumORCALines` / `getAgentORCALine` and `getAgentVelocity`): for random convex polygons, walls and concave polygons, the lines agree to `2e-3` and the solved velocity to `1e-3`. (RVO2's kd-tree splits an edge at another's line, so its lines depend on the partition; the oracle is RVO2 with the tree replaced by a flat list. This crate's grid never splits.)
- **A walker bends round a corner at speed.** A walker whose straight line grazes a building, closer to its corner than its own radius, arrives within 15% of the straight-line time and its disc never touches brick (`a_walker_bends_round_a_corner_at_speed`). ORCA does not plan the way round: a goal behind the building is the navigator's.
- **A doorway passes its crowd** at least as many as RVO2's own obstacle lines in the same scene, with `wall_max` 0. (First written as "as the hand-rolled half-planes"; see `Results`: the half-plane lets a walker run at a wall until its last step, which is not a property to match.)
- **A city of boxes** (the consumers' case: buildings, streets, walkers) at 100 to 10 000 agents: `wall_max` 0 (to the `f32` rounding of the host's positions), everybody arrives, no walker stalled for good, and the cost of the obstacles in agents per millisecond named against the same city solved with nothing in it.
- **Determinism:** one hash per case across member order and 1, 2 and the default number of threads; the empty `Obstacles` is bit-identical to `Crowd::solve`.
- **Allocation:** a warm `solve_around` allocates 0 at 10 000 agents (`tests/alloc.rs`).
- **Finite, refused, never panicked:** NaN, infinite and `±f32::MAX` agent states, radius 0 and negative, `dt` and horizons 0 and negative, an agent on a vertex, on an edge, inside a solid; non-finite and degenerate geometry is an `Err` at build; no panic, only finite velocities.
- **No cost on those who do not use it:** `Crowd::solve` without obstacles within noise of before (the paired harness).

## What great looks like

A walker running down a street beside a building does not slow at all: its line is the building's face and it is satisfied already. At a corner it keeps its speed and its distance, turns when its wish turns, and never touches the brick. A crowd in a doorway queues in front of the gap, not along the wall, and the first through are not slowed by the wall they pass at arm's length. A pillar in a square parts a stream and it closes behind it. And the caller wrote `builder.add_box(centre, half)` where today it writes a push-out.

## Deviations

- **Winding.** RVO2 requires counter-clockwise vertices and acts on the edge only from its right; this crate takes `add_solid` and `add_enclosure` in either winding and orients them (a polygon's signed area decides), and has no entry that means "wrong". A design default, from the footgun above.
- **The current velocity, not zero.** The paper picks the tangent at the point closest to `v_opt = 0` for obstacles; RVO2 projects the current velocity (`Agent.cc:455–510`), which is what produces the legs. This crate follows RVO2 (the crate's rule: RVO2's names and structure) and says so.
- **A uniform grid, not RVO2's kd-tree.** RVO2's `buildObstacleTree` splits an obstacle at another's line and creates a vertex there, so the lines an agent gets depend on the tree's partition. The grid never splits an edge, so an edge's lines are those of the edge. A design default; the differential target above avoids the cases where they differ.
- **No cap on segments** (Detour: eight), see above.
- **Degenerate lines are skipped or replaced, never non-finite**: RVO2 normalises a zero vector (`unitW`) in the measure-zero case where the velocity sits on a cutoff centre; here that picks the direction away from the vertex, and a vertex under the agent's centre falls back to the edge's own line. The answer is always a unit `b`, as `new_velocity_in` requires of a wall.
- **A wall has no foreign legs.** RVO2 skips a velocity projected on a "foreign" leg (the neighbouring edge casts that line); for a two-vertex obstacle the neighbouring edge is its own other side, which an agent on this side cannot see, so the line is never cast and an agent touching the face can walk into it (`Results`). A wall keeps its vertices' legs. The one place this crate's lines differ from RVO2's, and only at those states.
- **The obstacle horizon is raised to `dt`** in `Crowd::solve_around` (`obstacle_horizon.max(dt)`): RVO2 leaves it to the caller, and a horizon under the step lets a member cross a wall within it.

## Pending index

- Seidel, Havran ("Heuristic ray shooting algorithms", the grid density argument Ericson cites as [Havran99]) and the ORCA paper itself are not home-still documents (`corpus.md`, "Not in the corpus"); the ORCA quotes above are from the authors' PDF.
- No home-still document covers static-wall handling in a shipped crowd beyond Menge's profile and Sunshine-Hill's chapter; Detour and RVO2 were read on GitHub.

## Results (iteration 8)

macOS arm64 laptop (14 cores, shared), `bench` serial, medians of three interleaved rounds unless a spread is named; every probe below was throwaway (`/tmp/o8`) except the tests it became.

**Differential against RVO2** (`snape/RVO2` `main`, clang++ -O2, `getAgentNumORCALines` / `getAgentORCALine` / `getAgentVelocity`; as a flat list, which is RVO2 minus the kd-tree's splitting): 20 000 random convex polygons, 20 000 walls and 19 145 random star polygons (855 more refused at build as `SelfIntersecting`, each confirmed by an independent crossing test): the same set of lines in every case and a velocity within `3e-6`. Against the kd-tree as shipped, 79 of the 19 145 concave cases differ (0.4%): RVO2's own lines depend on how its tree partitioned. The twelve hand-chosen cases that cover each branch (a face, a cutoff circle, left and right legs, an edge collision, a vertex collision, a wall, an L-shaped polygon's notch) are pinned in `src/obstacles.rs`'s tests with RVO2's numbers.

**A hole in RVO2, found by the bench and closed here.** The bottleneck's first obstacle run showed `wall_max = 0.022` at 400 agents. A walker 4e-8 m from a wall's face, sliding up it at 0.27 m/s, was given **no line at all**: RVO2's "foreign leg" rule skips a leg because the neighbouring edge casts that line, and a wall with no thickness has only its own other side for a neighbour, which this walker cannot see. RVO2 built without fused multiply-add gives no line for the same state (`lines 0`), and the walker went 3.2 cm into the wall in a step; with FMA (clang's default on arm64) rounding flips the branch, which is why a casual run of RVO2 does not show it. Stress: agents touching a shape (distance `r + δ`, δ from 0 to 5 cm, anywhere along an edge and round its ends, any velocity and wish); one-step penetration over 1e-5 m:

| shape | agents | RVO2 (flat, no FMA) | this crate, first port | this crate |
|---|---|---|---|---|
| wall | 168 813 | 191 (worst 4.3 cm) | 191 | **0** |
| box | 152 549 | 0 | 0 | 0 |
| L-shaped polygon | 135 874 | 0 | 0 | 0 |
| 16-point star | 101 151 | 1 (4.2 cm) | 0 | 0 |
| two touching boxes | 110 351 | 0 | 0 | 0 |
| random convex, star and wall, one per agent | 400 000 | 5 874 (4.3 cm) | n/a | **0** |

The first fix tried (cast the replaced leg instead of skipping it) made it worse, 232, because for a wall the replaced leg points the other way round and casts the line backwards; a wall now keeps the legs of its vertices as they come. The permanent test is `an_agent_touching_a_wall_within_rounding_cannot_walk_into_it` (the bench's state, then 30 000 touching walkers), which fails on the first port (3.2 cm) and passes.

**Second finding, in this crate's own grid:** an edge a hair off vertical (ends differing in `x` by a subnormal) has a slope of `inf`, `inf · 0` is a NaN and the edge was linked into one row only, so a walker beside the rest of its length was handed nothing. Fixed (a slope that is not finite is a vertical edge), with `a_wall_a_hair_off_vertical_is_found_along_its_length`, which fails before.

**The doorway** (`bottleneck_obst` against `bottleneck`, arrive section, three times the straight-line time; means of eight crowds): share of the room through the gap at 100 / 200 / 400 agents 0.601 / 0.427 / 0.268 against 0.611 / 0.448 / 0.300 for the closest-point half-planes, and 0.575 / 0.419 / 0.235 for RVO2's own obstacle lines in the same scene (kd-tree as shipped, no nudge). So the first validation target above, "at least as many as the hand-rolled half-planes", is **not met** (−2%, −5%, −11%, the last about three standard errors) and was the wrong target: it measures how long a walker may run at a wall before braking (the half-plane's approach is `gap / dt`, the line's `gap / τ`), and the reference's own lines pass fewer still. Against the reference this crate passes more (+5%, +2%, +14%). `wall_max` is 0.0000 in all 192 crowds of the horizon sweep below (eight crowds at each of eight horizons and three sizes), and the deepest overlap between agents is lower (0.156 against 0.209 of a radius at 400).

**The horizon** (same scene, 400 agents, eight crowds each, share through the gap): 2 s 0.187; 1 s 0.255; 0.5 s 0.268; 0.3 s 0.266; 0.2 s 0.260; 0.1 s 0.280; 0.05 s 0.299; 0.034 s (one step) 0.262. Flat from a second down to the step, and 30% lower at the agents' own 2 s: the reason for a separate dial. In the city (1 000 walkers) the 90% time is 85.2–87.7 s for every horizon from 0.05 to 2 s and the stalled count 15–26: indifferent. `wall_max` is 0.0000 at every horizon, `dt` included. 0.5 s stays as the default: a design default inside the flat range, between Menge's 0.15 s (0.19 m walkers at a 0.1 s step) and the agents' 2 s.

**The city** (`city` against `city_pushout`, arrive section, 1 000 walkers, one trip each). *First run, lanes at ±1.5 m (the outer lanes 0.2 m from the brick):* arrived 1.000 against 1.000; 90% at 86.1 s against 82.0 s; stalled for over 2 s (at some point, not at the end) 19 against 2; deepest overlap between walkers 0.04 against 0.22 of a radius; `wall_max` 0.0004 at 5 000 and 0.0005 at 10 000. The stalls traced (printed at the step each began) were walkers beside a wall with a head-on neighbour within 1.5 m, all of whom arrived, and `ORCA_NUDGE` 0 and 0.15 made them 29 and 39: they are not the nudge. Moving the lanes in changes them: at a lane share of 0.6 (±0.9 m, 0.8 m clear of the brick) the obstacles' city stalls 2 and reaches 90% at 81.0 s, the push-out's 1 and 81.5 s. A head-on walker in a lane 0.2 m from the brick has no room on its wall side to step aside, and ORCA assumes the other side takes its half; with the wall hard in the solve it stalls, while the push-out hides it by letting the walker lean on the brick. The bench's default is the 0.6 lane share (`ORCA_LANE`, `1.0` keeps the stress case), chosen after seeing this, and recorded here for that reason. *At 0.6:* arrived 1.000 against 1.000, 2 against 1 stalled, 81.0 s against 81.5 s, deepest overlap between walkers 0.04–0.05 against 0.19–0.23 of a radius, and 0.3% of agent-steps end inside a building in the push-out city (moved by up to one step, 4.3 cm). Timing rows (serial, median of three, µs; agents per ms): `city` 100 117 (853), 1 000 1 267 (789), 5 000 9 020 (554), 10 000 21 144 (472); `city_pushout` 113 (885), 1 654 (604), 9 039 (553), 19 220 (520); the single full run reads 1 768 against 1 660, 9 779 against 8 608 and 20 829 against 19 265 at 1 000, 5 000 and 10 000. The obstacles cost between nothing and 14% of a step, and the runs disagree about where; scaling exponent 1 000 to 10 000 in the full run 1.07 and 1.07 (the median of three rounds reads 1.22 for `city` because its 1 000-agent median caught a fast core: 1 634, 1 245 and 1 267 µs). Candidates per walker 6.0–9.3, at most 25. `wall_max` is 0.0000 at every size. The 0.4 mm of the first run was the host: a walker creeping along a building's corner at 2 mm/s (a step of 6e-5 m) at x = 134 m, where an `f32` position has a spacing of 1.5e-5 m, rounds each step up to 1.5e-5, so its depth grew 6e-6 m a step while it stayed there, and the velocity the solver returned satisfied its line every step (`every_answer_satisfies_every_obstacle_line`: over 5 000 lines in 300 steps of 260 walkers, no velocity broke a line by more than `1e-4`).

**Cost of those who do not use it** (`scripts/paired.sh ea50ded 300`: four copies of the crate, the same frozen crowds, rotating order, thread CPU time): `Crowd::solve`, new against base, +1.9 / −0.4 / −0.4% (open 100 / 1 000 / 10 000), +0.2 / −0.4 / +0.0% (circle), +0.4 / +0.7 / +0.3% (corridor), +0.0 / −0.7 / +0.2% (rally); the floor (two copies of the same code) was −2.5 to +3.8% at 100 and within ±1.1% from 1 000. The macro bench alone could not say: its three interleaved rounds read 100 agents anywhere from −40% to +72% (rally 100: 243 or 419 µs for the same binary), and 1 000 to 10 000 from −19% to +5.7%.

**Determinism, allocation, input.** One hash per (scene, size) for each of `bottleneck_obst`, `city`, `city_pushout` at 100, 1 000, 5 000, 10 000, across member order and 1, 2 and 14 threads (12 of 12); the five older scenes' hashes at 100 and 1 000 are equal to the base commit's, as are their `arrive` rows and the output of `head_on` and `wall_corridor`; `tests/alloc.rs` 6 of 6 (a warm `solve_around` walk through a 36-building street grid allocates 0); hostile sweep (40 000 single solves and 2 000 crowd steps with NaN, infinities, `±f32::MAX`, zeros and negatives in every field, horizons included): no non-finite line or velocity, no panic. `Obstacles` builds 160 000 edges in 27 ms, 107 000 touching-cell edges in 26 ms.
