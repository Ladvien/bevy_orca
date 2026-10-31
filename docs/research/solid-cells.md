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
2. Loops are walked from the first vertex in row-major order that has an unused edge, each edge leaving its vertex's set as it is used, and at every vertex the walk takes **the leftmost unused edge: left, then straight, then right**. At a pinch (two diagonal solid cells) the leftmost edge stays with the cell the walk is on, so **solid cells that meet only at a corner are separate polygons**; a disc has no way through a point, and a polygon that touched itself would be refused by `build`. The fixed rule is Recast's and Lengyel's.
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

A tile game passes the same `Vec<bool>` its flow field reads and gets back a few hundred edges where it had tens of thousands: a room is its four walls, a corridor wall is one segment however many cells long, a pillar is four edges, an L-shaped room has six. Walkers slide along a long wall exactly as they slide along one `add_box`, round a door frame without a catch at any cell seam, and a one-cell gap between two pillars that touch at a corner is shut. Rebuilding after the player places a building is a frame's work, not a hitch.

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
