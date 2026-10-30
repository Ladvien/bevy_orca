# Crowd's neighbour search — a sorted cell index rebuilt every step

`Crowd::solve` finds each member's neighbours in a uniform grid whose cell is `neighbour_range`, scanning the 3×3 block around the member. Until iteration 1 the grid was a `HashMap<(i32, i32), Vec<usize>>` that kept every cell it had ever seen. Iteration 1's bench and tests found three faults in it:

- **It allocated every tick a crowd walked into new ground** — 2 619 allocations in 480 warm steps of the 1 000-agent circle swap, 203 in 20 ticks of a 60-member group crossing new cells — although the README promises nothing after warm-up.
- **Its per-step cost grew with the ground covered, not the crowd**: the rebuild cleared every cell ever visited.
- **It panicked in a debug build at a far but finite position**: a member at `x = 1e30` saturates its cell to `i32::MAX`, and `cx + 1` overflows (`attempt to add with overflow`, `src/crowd.rs:162`).

## Sources

- **Model:** `10.1109_robot.2008.4543489` (RVO, ICRA 2008: the neighbour region, and why only the neighbour search can break linear scaling).
- **Shipping game:** Recast/Detour's `DetourCrowd` (`DetourProximityGrid.cpp`, `DetourCrowd.cpp`, recastnavigation `main`), the crowd library behind many shipped navmesh crowds; not in the corpus.
- **Technical:** `eth-cgl-sim_anim-Sol14a` (Ihmsen et al., "SPH Fluids in Computer Graphics", EG STAR 2014: uniform grids, spatial hashing, compact hashing).

## Craft principle

Avoidance looks only at a neighbour region, and the cost of finding that region is the one part of the step that can grow faster than the crowd:

> "We do not need to take all other agents into account when selecting a new velocity [...]. Therefore, we define a neighbor region NR^i around the current position of agent A_i and only take into account the agents and obstacles inside this neighbor region" — `10.1109_robot.2008.4543489`, p. 6

> "Only the neighbor selecting routine has a quadratic nature, but this step is negligible in the total running time." — ibid.

So the search must cost O(n) or O(n log n) in members, with storage that is a function of the members alone.

## How shipping games do it

Detour's crowd sizes its proximity grid from the agent count once, at init, and clears and refills it every update — no storage per cell of the world:

> `if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))` — `DetourCrowd.cpp`, `dtCrowd::init`

> `m_bucketsSize = dtNextPow2(poolSize);` / `m_buckets = (unsigned short*)dtAlloc(sizeof(unsigned short)*m_bucketsSize, DT_ALLOC_PERM);` — `DetourProximityGrid.cpp`, `dtProximityGrid::init`

> `m_grid->clear();` followed by `m_grid->addItem(...)` for every agent — `DetourCrowd.cpp`, `dtCrowd::update`

RVO2 itself rebuilds a k-d tree of its agents every step (`KdTree::buildAgentTree`), which is an O(n log n) sort in all but name.

## Technical basis

> "If the cell size is equal to the kernel support, 3^3 cells have to be queried in 3D, which is optimal" — `eth-cgl-sim_anim-Sol14a`, §2.1

> "[THM*03] suggests to reserve memory for a certain number k of entries in all hash cells on initialization in order to avoid frequent memory allocations. However, [...] the hash table is generally sparsely filled. Thus, a significant amount of memory is unnecessarily pre-allocated. Furthermore, cells that are close in memory are necessarily not close in space which reduces the cache-hit rate" — ibid., §2.1.3

> "Thereby, the memory consumption scales with the number of particles and not with the simulation domain." — ibid., §2.1.4 (compact hashing)

Sorting the members by cell index every step, then finding a cell's members as one contiguous run of the sorted array, gives the same property with no hash table at all: the storage is one entry per member.

## Model adopted

- One `Vec<(u64, usize)>` of `(cell key, member index)`, cleared, refilled and sorted (`sort_unstable`; the pairs are unique, so the order is fully determined) at the start of every `solve`. It never shrinks, so after the widest crowd it allocates nothing — wherever that crowd walks.
- The key is row-major: the cell's row in the high 32 bits and its column in the low 32, each offset from `i32::MIN` so the order of keys is the order of `(row, column)`. The three cells of one row of the 3×3 block are then one contiguous key range, found with two binary searches (`partition_point`): six searches per member instead of nine hash lookups.
- Neighbouring rows and columns are computed in `i64` and clamped to the `i32` range, so a member whose cell saturated at the edge of the float range looks at its real neighbours and never overflows. Members beyond `±range · 2^31` share an edge cell; the distance test that follows rejects those that are not in range, as it already did.
- Candidates are still filtered by `dist <= range` and sorted by `(distance, key, index)`, so the neighbour lists — and every solved velocity — are bit-identical to the hash map's.

## Parameters

None new. The cell stays `neighbour_range` (the optimum the STAR cites for a query radius equal to the cell).

## Validation targets

- `tests/alloc.rs`: a warmed 60-member crowd that moves two cells every tick allocates 0 times in 20 ticks (203 before).
- The bench's `allocs` is 0 in every scene at every size (19 to 2 619 before).
- `a_member_at_the_edge_of_the_float_range_is_solved_without_panicking` passes in a debug build.
- The bench's hashes are unchanged at every scene, size, member order and thread count; `cand_mean` and `cand_max` are unchanged.
- No case's median step regresses more than 5%; the scaling exponent does not rise.

## What great looks like

A host can walk a crowd across a world of any size for an hour and the solver's memory and per-step cost are a function of how many members it has, not of where they have been; a stray far-off position in a debug build is just a member nobody is near.

## Deviations

- Design default: a sorted index rather than RVO2's k-d tree or Detour's fixed-size hashed buckets. Detour drops agents once its pool is full; a sorted `Vec` grows to the crowd instead, which keeps the "nobody is silently left out" rule this crate already has.

## Pending index

- Recast/Detour's source is quoted from GitHub; it is not a home-still document.
- Ihmsen et al., "A Parallel SPH Implementation on Multi-Core CPUs" (CGF 2011, the [IABT11] compact hashing paper): not in the corpus.
