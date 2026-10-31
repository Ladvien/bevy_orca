# Solid cells — a tile map's blocked cells as one outline per region

A great many games hold what is solid as a grid: a `Vec<bool>` or a tile enum, one entry per cell, the same grid their flow field or A* already reads. `ObstaclesBuilder` (iteration 8) takes polygons, so such a host has had exactly one way in, `add_box` per solid cell, and one way it is tempted by instead, the hand-rolled half-planes it wrote before this crate had obstacles. Both are measured below.

## What the consumers do today

- **bevy_emerge** (`crates/bevy_emerge_mapper/src/crowd_preview.rs`, `Preview::step`, read 2026-10-31, pinned to a `bevy_orca` revision before `Obstacles`) keeps `solid: Vec<bool>` (`solid_cells`, row-major, `CellFrame`) and builds four axis-aligned half-planes per walker from the 4-neighbour cells, then, because a cell that touches the walker only at a corner casts none, slides or stops any walker whose next position lands in a solid cell. Its issue (#107) asks for "Walls and walkable cells come from the map's pieces" and "Walkers never pass through a wall cell". The grid is its native form; it will move to `Obstacles` when its pin moves, and the way in is `add_box` per cell.
- **bevy_society** (`src/steering.rs`, `steer_movers`) hand-rolls a disc per `Obstacle { radius }` and loops every solid for every walker; a village of 24. Not a grid; not this item.
- **Sanctuary** (`src/crowd.rs`) passes `|_i, walls| walls.clear()` and pushes the baked boxes out after the solve. Boxes, so `add_box` already serves it.
- **This repository's bench** has no grid scene.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (the authors' PDF, read for `static-obstacles.md`): obstacles are "a collection of line segments", the constraints against them "*hard*". RVO2's own statement of what a polygon is (`snape/RVO2`, `src/RVOSimulator.h`, `addObstacle`, read 2026-10-31). Hechenberger et al., "Online Computation of Euclidean Shortest Paths in Two Dimensions" (`10.1609_icaps.v30i1.6654`): the grid-map convention for obstacles.
- **Shipping game / engine:** Recast `Recast/Source/RecastContour.cpp` (`walkContour`, `simplifyContour`; GitHub `recastnavigation/recastnavigation` `main`, read 2026-10-31): the contour of a region of grid cells is walked once, with one fixed turning rule, and simplified. Lengyel, "Transition Cells for Dynamic Multiresolution Marching Cubes" (`10.1080_2151237x.2011.563682`; his dissertation is `transvoxel_dissertation_lengyel2010`): the shipped terrain engine's rule for the ambiguous (checkerboard) face.
- **Technical:** Natarajan, "On generating topologically consistent isosurfaces from uniform samples" (`10.1007_bf01900699`), §3: in two dimensions "The topology for cases I and II are unique, while for case III, there are two possible topologies" — the checkerboard of two diagonal solid cells; Ericson, *Real-Time Collision Detection* ch. 7 (`book_real_time_collision_detection_ericson`) for why the edge count is what the neighbour grid pays for (an object "linked into each" cell it overlaps).

## Craft principle

RVO2 says what a polygon is, and says it by winding:

> "Adds a new obstacle to the simulation. @param vertices List of the vertices of the polygonal obstacle in counterclockwise order. [...] @note To add a "negative" obstacle, e.g., a bounding polygon around the environment, the vertices should be listed in clockwise order." — RVO2, `RVOSimulator.h`

So a region of solid cells is a set of **counter-clockwise outer loops and clockwise hole loops**, with the solid on the left of every edge: the boundary of the region, oriented. That is what an agent sees from its right, and nothing inside the solid or between two touching cells is ever seen. The paper's own premise is the same: "We can generally assume that obstacles are modeled as a collection of line segments" (§5.4), and the lines the segments cast are hard.

The grid case has one trap, and the literature names it. Two solid cells that touch only at a corner:

> "If two obstacles touch each other at a point (see D6 in Figure 5a), it is ambiguous if passing through this touching point is allowed or not." — Hechenberger, Stuckey, Harabor, Le Bodic & Cheema, ICAPS 2020, on tracing a grid's blocked cells into "a set of inner-obstacles and enclosure".

and its isosurface twin, Lengyel's "ambiguous faces arising when two diagonally opposing corners of a cell face have the same inside/outside classification and the other two corners have the inverse classification". The two readings give two different outlines (one polygon that touches itself at the corner, or two polygons that meet there), and `ObstaclesBuilder::build` refuses a polygon whose edges touch.

## How shipping games do it

Recast builds the outline of a region of cells by walking its boundary with one fixed rule and no choice at a pinch:

> `flags[i] &= ~(1 << dir); // Remove visited edges` / `dir = (dir+1) & 0x3; // Rotate CW` (a border edge: emit the vertex and turn) / `dir = (dir+3) & 0x3; // Rotate CCW` (an open neighbour: step into it and turn back) — Recast, `RecastContour.cpp`, `walkContour`

then drops the vertices it does not need (`simplifyContour`): the contour is one loop per boundary, each edge visited once, and a straight run of cells is one edge. Lengyel's engine settles the ambiguity the cheap way, and says why:

> "Because simplicity and speed are among our goals, and because it is often possible to easily change the input voxel data to eliminate problem areas in an output mesh, we dispense with the issues of ambiguous faces and topology preservation entirely and choose the simplest solution, which is enforcing a fixed-edge polarity on ambiguous faces." — Lengyel, `10.1080_2151237x.2011.563682`

A fixed rule, the same at every pinch. RVO2's reference demo (`examples/Blocks.cc`) lists four separate four-vertex obstacles; nothing in RVO2 turns a grid into outlines, so every host writes it.

## Model adopted

**`ObstaclesBuilder::add_cells(origin, cell_size, width, solid)`**: `solid` is row-major, `solid[row * width + column]`, column along `+x` and row along `+y` from `origin`, the lower-left corner of cell `(0, 0)`, each cell `cell_size` square; the grid's height is `solid.len() / width`. `true` is a solid cell. **Outside the grid is open floor**: a host that wants the map's edge to be a wall adds `add_enclosure` of the map's rectangle, or pads the grid, so one call does one thing.

**The outline** is the oriented boundary of the solid region, traced once:

1. A solid cell whose neighbour (or the map's edge) is open contributes the unit lattice edge that has the solid on its left: east along its south side, north along its east side, west along its north side, south along its west side. Each lattice vertex keeps a four-bit set of the edges that leave it.
2. Loops are walked from the first vertex in row-major order that has an unused edge, each edge leaving its vertex's set as it is used, and at every vertex the walk takes **the leftmost unused edge: left, then straight, then right**. At a pinch (two diagonal solid cells) the leftmost edge stays with the cell the walk is on, so **solid cells that meet only at a corner are separate polygons**; a disc has no way through a point, and a polygon that touched itself would be refused by `build`. The fixed rule is Recast's and Lengyel's. *Found while building: the rule alone is not enough.* When the two cells are joined elsewhere (a pocket of open floor pinched off by one corner contact: 3 cells wide, 5 high, with a hole at its fourth row is enough), the walk comes to the pinch twice in one loop, and the polygon touches itself, which `build` refuses (`SelfIntersecting`); the other pairing only moves the problem to the checkerboard. So a loop that visits a pinch vertex twice is cut there into the loops it is made of (the outside, and the pocket it pinches off); every loop emitted visits each vertex once.
3. A loop's collinear vertices are dropped, so a straight wall of any length is one edge and a rectangle is four.
4. A loop whose area is positive is a solid (`add_solid`), one whose area is negative is a hole in a solid (`add_enclosure`), both from the exact integer area. Vertices are `origin + lattice · cell_size`, computed from the integer coordinates, so every polygon that shares a lattice vertex has the same `f32` there.

The result is the same `Obstacles` any polygon makes: nothing downstream changes, `solve_around` and `push_walls` are untouched. The cost is `O(cells)` once, at build, in a transient byte per lattice vertex.

An error leaves the builder as it was. A new `ObstacleError::BadGrid` names a `width` of 0 for a non-empty grid, a `solid` whose length is not a multiple of `width`, and a `cell_size` that is not positive; a non-finite `origin` or `cell_size` is `NotFinite`; a coordinate beyond `1e18` is `OutOfRange`, as for any polygon.

## Parameters

- `origin`, `cell_size`, `width`, `solid` as above. Square cells (a rectangular cell is the host's own scale of the plane).
- No tuning: no simplification tolerance (the outline is exact, not an approximation), no minimum region size, no padding.

## Validation targets

- **Exact outline.** For random grids at every fill, the polygons' edges are exactly the boundary unit edges of the solid region, each covered once, each with the solid on its left, and no others; checked against an independent per-cell construction, then by point parity for random points in and out of the solid.
- **A rectangle is its four edges.** A block of cells builds the same lines as `add_box` of the block: bit-equal `push_walls` output for thousands of random agents.
- **`build` accepts every grid.** Checkerboards, spirals, rings with a pinched gap, all-solid, all-open, a single cell, a one-cell-wide wall at the map's edge, and 5 000 random grids at fills from 5% to 95%: never `SelfIntersecting`.
- **Walls hold.** Random cell maps with walkers started clear of every solid cell, a wish through the wall, over many steps: no disc enters a solid cell by more than `1e-4` m (the iteration-8 bound), including at pinch vertices, one-cell-wide walls and one-cell gaps; and in the bench's city, `wall_max` 0.
- **Cost.** The same city as boxes per cell (what a consumer does today) against the merged outlines at 100 to 10 000 agents, step time and edges; the merged outline within noise of one `add_box` per building (the paired harness for anything near 5%); the build time of a 1 000 × 1 000 grid stated.
- **Determinism, allocation, input.** One hash per (scene, size) across member order and threads for the new scenes; a warm `solve_around` on cell-built obstacles allocates 0; the hostile inputs above give `Err`, never a panic, and leave the builder as it was.

## What great looks like

A tile game passes the same `Vec<bool>` its flow field reads and gets back a few hundred edges where it had tens of thousands: a room is its four walls, a corridor wall is one segment however many cells long, a pillar is four edges, an L-shaped room has six. Walkers slide along a long wall exactly as they slide along one `add_box`, round a door frame without a catch at any cell seam, and a one-cell gap between two pillars that touch at a corner is shut. Rebuilding after the player places a building is a frame's work on a map a few hundred cells a side (about 1 ms); on a 1 000 × 1 000 map it is 18 ms, a hitch the host schedules rather than pays every frame.

## Measured before any code (main, `8e1ec6a`, `add_box` per solid cell)

A throwaway probe against the base commit (`/tmp/o9/probe`, removed with the scratch):

- **Edges.** 4 per solid cell however interior: a 200 × 200 map at 10% / 30% / 50% fill is 3 899 / 11 931 / 19 964 cells and 15 596 / 47 724 / 79 856 edges, built in 3.4 / 12.4 / 17.9 ms. A street of two 3-cell-thick, 400-cell-long walls is 9 600 edges where two long boxes are 8.
- **Per-agent cost.** `push_walls` for 10 000 agents scattered across that street, 1.5 m/s, 0.5 s horizon: **1 169 ns per agent against 192 ns** (6.1×), 0.59 against 0.40 lines per agent (the seams between touching boxes cast lines of their own).
- **Behaviour is not the defect.** A walker running along the tiled wall at 0, 5 and 30 cm clearance, and sliding along it with its wish into the wall at 10°, 30° and 60°, moves at the same speed as along one long box (1.5000, 1.4772, 1.2991, 0.7500 m/s): iteration 8's `add_box` handles seams correctly. The case for this item is what a grid costs to feed in and to query, not a wrong answer.

## Deviations

- **A pinch splits.** Diagonal solid cells are two polygons, not one that touches itself. Not RVO2 (it has no grid) and not the only reading (Hechenberger et al. cut a cell at each concave corner instead, changing the polygons); a design default, chosen because `build` refuses the other and a disc cannot pass a point.
- **Outside is open.** A design default: one call does one thing, and `add_enclosure` already says "the map's edge is a wall".
- **A slice, not a closure.** `solid: &[bool]` matches the consumers' data (`Vec<bool>`) and costs no call per cell; a host with another cell type fills a `Vec<bool>` (one byte per cell, once per edit). A design default.
- **Square cells.** A design default.

## Pending index

- Nothing the item rests on is missing from home-still: the ICAPS paper, Lengyel's paper and dissertation and the isosurface paper are indexed (`corpus.md`, "Solid cells to outlines"); the ORCA paper and the RVO2 and Recast sources were read from the authors' PDF and GitHub, as in `static-obstacles.md`.
- No home-still document treats contour tracing of tile maps for crowd avoidance; Recast's `RecastContour.cpp` is the one shipped implementation read.

## Results (iteration 9)

macOS arm64 laptop (14 cores, shared); every probe below was throwaway (`/tmp/o9`) except the tests it became.

**Exactness.** `the_outline_of_a_grid_is_exactly_its_boundary`: 4 000 random grids (1 to 13 wide, 1 to 11 high, 5% to 95% solid, with more than a thousand corner contacts among them): the polygons' unit edges are exactly the boundary unit edges of the solid region, each once and directed with the solid on its left, every loop closed, every vertex a real turn, no loop visits a vertex twice, and the loops' signed areas (outsides positive, pockets negative) add up to the solid's. `every_grid_builds`: 3 000 random grids with a random origin and cell size, never refused; the first run of it found the pinched pocket above (a loop touching itself) before the split existed.

**A block of cells is its box.** `a_block_of_cells_gives_the_lines_of_a_box`: five separate blocks in one grid (origin and cell size dyadic, so the corners are exact), 4 000 random agents: the same lines as `add_box` bit for bit, in sets. In `bench` the same fact at scale: `city_cells` and `city` print the same state hash at 100, 1 000, 5 000 and 10 000 agents (`6a76a7f2…`, `6420894f…`, `22b8a5f7…`, `f8d91cfc…`) and the same `arrive` line (`a49c961e…`, `aae24161…`; 1.000 arrived, 2 of 1 000 stalled, `t90_s` 81.0).

**Rebuild on edit, city-like maps** (buildings 8 × 8 on a 12-cell pitch and a wall ring, best of five, `add_cells` + `build`): 200 × 200 0.9 ms (1 162 edges), 360 × 360 1.0 ms (3 608), 1 000 × 1 000 17.9 ms (28 230), 2 000 × 2 000 73.7 ms (111 562). `O(cells)` every call; the first, cold call of the 360 × 360 map took 3.6 ms.

**Edges and the build.** The city as 1 m cells (buildings 8 × 8): 16 / 196 / 1 024 / 1 936 edges at 100 / 1 000 / 5 000 / 10 000 walkers through `add_cells` against 1 024 / 12 544 / 65 536 / 123 904 as a box per cell (64×), built in 0.05 / 0.14 / 0.46 / 2.0 ms against 0.16 / 3.3 / 12.6 / 26.7 ms (three runs of the 10 000 case: 2.0–2.5 against 26–30 ms). A 360 × 360 map of 900 buildings: 3 600 edges in 3.6 ms. Random fill is the worst case, nothing merges: 200 × 200 at 30% is 26 740 edges against 47 724 for boxes, 3.5 ms; 1 000 × 1 000 at 10% / 30% / 50% is 0.33 / 0.66 / 0.75 million edges in 0.10 / 0.29 / 0.38 s including the index; 4 000 × 4 000 at 2% (1.2 million edges) 0.44 s.

**Step cost of feeding it in cell by cell** (same process, one frozen city of 10 000 or 1 000 walkers, `solve_around` with the merged outlines, with one box per cell and with a second copy of the outlines, rotating order, 60 to 400 reps, median of per-rep ratios): boxes / outlines **1.026 and 1.108** at the default lane (1 000 / 10 000 walkers) and **1.216 and 1.236** with the outer lanes 0.2 m from the brick, the copy of the outlines 0.978 to 1.000. The macro `bench` reads the same pair at 10 000 walkers as +2.7% to +10.8% but at 1 000 anywhere from −24% to +92% (`city_boxes` 1 000 walkers: 1 330, 1 443 and 1 842 µs in three runs of the same binary, `city_cells` 752 to 1 778), the bimodal host again. The street probe above (every agent within range of brick) is the upper end, 6.1× on `push_walls` alone.

**A leak found on the way, in the obstacle lines, not in `add_cells`.** A crowd of forty on random caves (600 maps of 30 × 30 cells at 15% to 60% solid, 900 steps each: 21.6 million agent-steps) put a disc 4.7 cm into a cell in 28 agent-steps, and **identically with one `add_box` per cell**, so it predates this item. One walker, one box reproduces it: at `(0.3, 24.000284)` beside the corner `(0, 24)`, speed 0.03, wish `(-1.45, 0.4)`, `push_walls` gives no line and the step goes 4.8 cm into the corner. The mechanism is in `obstacle_lines`: an edge viewed obliquely from beyond its end (`s > 1`, `dist_sq_line <= radius_sq`) defines the velocity obstacle by its far vertex, the left leg lies along its own face and is replaced by that face's direction as "foreign", and a velocity projected on a foreign leg adds no line because "the neighbouring edge casts it" — and the neighbouring edge is this one (`vertices[v1.prev]` after `v1` became the far vertex), which casts nothing in this branch. The mirror image holds for the right leg at the left vertex (`s < 0`). RVO2 has the same rule. The fix is two assignments: a replaced leg is still skipped as foreign except in the branch where its neighbour is the edge itself, where it is cast. `an_agent_creeping_past_a_convex_corner_cannot_step_into_it` fails before (0.0479 into the corner for the state above; 310 of 40 000 swept corner states over `1e-4` m, worst 5.0 cm) and passes after (none, worst 6e-8). The same 21.6 million agent-steps after: none over 1 mm, the deepest 0.12 mm, the creep of an `f32` position pressed against a wall at 10 µm/s (a rounding accumulation of the kind iteration 8's log records, 0.4 mm at a creep of 2 mm/s). No bench scene's state hash moved (`bottleneck_obst`, `city`, `city_pushout` at 100 to 10 000; the older five at 100 and 1 000) and `head_on`, `wall_corridor` and `street` print byte for byte what they printed. Paired against the base commit, two copies of each, thread CPU time, the frozen city: new / base 0.994 to 1.023 against a floor of 0.987 to 1.007.

**Determinism, allocation, input.** `city_cells` and `city_boxes` at 100 / 1 000 / 5 000 / 10 000: one hash per size across member order and 1, 2 and 14 threads (8 of 8 groups). The solve is the one every obstacle set uses, so `tests/alloc.rs`' warm `solve_around` (6 of 6) is the proof there. `add_cells_refuses_what_is_not_a_grid`: a zero or mismatched `width`, a `cell_size` of 0, −1, −0, NaN or infinity, a NaN or infinite `origin`, coordinates past `1e18` and cells too small to tell their corners apart (`DegenerateEdge`) are each an `Err` and leave the builder with exactly what it had; an empty grid and an open grid are fine.

**Walls hold on random maps.** `a_crowd_on_a_random_map_never_enters_a_solid_cell` (16 caves at 30% and 50% solid, 40 walkers each, every one wishing to go through the rock) and `a_gap_between_cells_that_touch_at_a_corner_is_shut` (both diagonals): no disc enters a cell by more than `1e-4` m, and the walker aimed through the touching point does not get through. The bench's city: `wall_max` 0.0000 at every size.

**Flaws kept on record.** (1) Outside the grid is open: a host that forgets to pad the map or `add_enclosure` it has no wall at its edge, by design and documented. (2) The outline is exact, so a one-cell-thick diagonal "wall" of cells that touch only at corners is a chain of separate squares, each a polygon with four edges; a disc cannot pass between two of them, but a host that expects a diagonal line to be one long segment gets squares. (3) `add_cells` allocates a byte per lattice vertex while it runs (a 4 000 × 4 000 map: 16 MB for 0.44 s); it is a build-time call, and the warm solve is untouched. (4) The macro `bench` cannot resolve the step-cost claim by itself (above); the same-process A/B can. (5) Rebuilding the whole `Obstacles` for one edited cell is O(map); an incremental update is not offered.
