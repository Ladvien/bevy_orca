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

- **`CrowdParams::obstacle_horizon`** — seconds of lookahead against obstacles, separate from `time_horizon` (paper §5.4, Menge's `tauObst`, RVO2's `timeHorizonObst`). The default is a design default set by measurement in `Results`, not by Menge's 0.15 s, which is tuned to 0.19 m agents at 1.04 m/s and a `dt` of 0.1 s. It must be at least `dt`: a shorter horizon lets an agent cross a wall within a step.
- The query range is `obstacle_horizon · max_speed + radius`, RVO2's.
- Grid cell size: automatic (not a caller dial); the cell-count cap is a constant.

## Validation targets

- **Wall penetration is exactly 0** (to `1e-4` m of rounding) over every step of every scene, for solids, enclosures and walls, at 100 to 10 000 agents; a thin wall and a box built of touching boxes included.
- **A differential against RVO2** (built as shipped, `Agent::getNumORCALines` / `getORCALine` and `newVelocity`): for single convex polygons and single walls, where RVO2's kd-tree splits nothing, the lines agree to `1e-4` and the solved velocity to `1e-3` over 100 000 random agents. (For several polygons RVO2's tree splits an edge at another's line and its lines then depend on the partition; this crate's do not.)
- **Corners do not stop a walker.** An agent whose goal is behind a box's corner reaches it in no more than 1.3× the straight-line detour time; with a hand-rolled half-plane the same walker brakes at the corner.
- **A doorway passes its crowd.** The bench's bottleneck with its walls as `Obstacles` passes at least as many agents through the gap as the hand-rolled segments in the same budget, with `wall_max` 0.
- **A city of boxes** (the consumers' case: buildings, streets, walkers) at 100 to 10 000 agents: `wall_max` 0, no stalled walker, arrival no slower than with the push-out the consumers write today, and the cost of the obstacles in agents per millisecond named.
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

## Pending index

- Seidel, Havran ("Heuristic ray shooting algorithms", the grid density argument Ericson cites as [Havran99]) and the ORCA paper itself are not home-still documents (`corpus.md`, "Not in the corpus"); the ORCA quotes above are from the authors' PDF.
- No home-still document covers static-wall handling in a shipped crowd beyond Menge's profile and Sunshine-Hill's chapter; Detour and RVO2 were read on GitHub.
