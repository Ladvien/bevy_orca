# Crowd's candidate selection — the nearest `max_neighbours`, chosen in place

`Crowd::solve` gathers every member in range that `sees` accepts into `near`, a `Vec<(f32, u64, usize)>` of `(distance, key, index)`, sorts it with `sort_by`, and hands the first `max_neighbours` (10 by default) to the solver. Issue #7 found the sort on the hot path allocating, and the bench's new `rally` scene (units hex-packed at 2.65 agents/m² with a 5 m range: 89 to 258 candidates each) measures it:

- **`sort_by` is the stable driftsort, and it allocates a scratch of `max(len − len/2, min(len, 8 MB / 24 B))` elements whenever that does not fit its 4 KiB stack buffer** — 170 elements of 24 bytes. Every member with more than 170 candidates allocates once per `solve`: **260 619 allocations in 480 warm steps** of the 1 000-member rally (`cand_mean` 170.8, `cand_max` 258), against the README's "`Crowd` allocates nothing after its buffers are warm". The existing scenes never reach it (their densest member has 65 candidates).
- **It sorts every candidate when ten are used.** The 1 000-member rally's step takes 8.3 ms in `Crowd::solve` (median of three paired runs on a loaded 24-core host); with only the nearest ten chosen and ordered it takes 3.9 ms, and at 10 000 members 103 ms becomes 49 ms. The shorter lists of the bottleneck and the open field gain too (below).

## Sources

- **Model:** `10.1109_robot.2008.4543489` (van den Berg, Lin & Manocha, RVO, ICRA 2008: the neighbour region); `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas, Game AI Pro 2 ch. 19 §19.3.3 — Guy is an ORCA author: a fixed number of neighbours is what keeps the step linear); RVO2's `RVOSimulator.h` on `maxNeighbors` (`snape/RVO2` at `6b4eb00`, read from source).
- **Shipping game / engine:** Recast/Detour `DetourCrowd.cpp`, `addNeighbour` (`recastnavigation/recastnavigation`, `main`, the copy read in iteration 3's first attempt); RVO2's `Agent::insertAgentNeighbor` (`src/Agent.cc`).
- **Technical:** the Rust standard library at the toolchain this crate builds with (`rust-lang/rust` tag `1.98.1`): `library/core/src/slice/sort/stable/mod.rs` (driftsort's scratch), `library/core/src/slice/mod.rs` (`sort_unstable_by`, `select_nth_unstable_by`), `library/core/src/slice/sort/select.rs` (introselect), `library/alloc/src/slice.rs` (`sort_by`); `10.1145_3505286` (Axtmann, Witt, Ferizovic & Sanders, "Engineering In-place (Shared-memory) Sorting Algorithms", ACM TOPC 2022).

## Craft principle

Avoidance reads a bounded neighbourhood, and finding it is the part of the step that decides whether the step scales:

> "We do not need to take all other agents into account when selecting a new velocity [...]. Therefore, we define a neighbor region NR^i around the current position of agent A_i and only take into account the agents and obstacles inside this neighbor region" — `10.1109_robot.2008.4543489`, p. 6

> "The main bottleneck in performance is actually determining an agent's neighbors. [...] The proximity computations in the pruning step can be accelerated using a spatial data structure for nearest neighbor queries, such as a k-d tree or a uniform grid. By selecting a fixed maximum number of neighbors for each agent, the runtime will be nearly linear in the number of agents." — `gameaipro2-ch19-guide-anticipatory-collision-avoidance`, §19.3.3, p. 8

RVO2 states the same bound as a parameter: `maxNeighbors` is "The default maximum number of other agents a new agent takes into account in the navigation. The larger this number, the longer the running time of the simulation. If the number is too low, the simulation will not be safe." (`RVOSimulator.h`).

So the solver's input is the *k* nearest, not the neighbourhood sorted. Choosing them is a selection problem, and its cost should be linear in the candidates, with no memory beyond the buffer that already holds them.

## How shipping games do it

Neither reference implementation sorts a candidate list. Both keep a fixed-size array, nearest first, and insert into it as candidates are found. Detour's crowd:

> `// Insert neighbour based on the distance.` … `else if (dist >= neis[nneis-1].dist) { if (nneis >= maxNeis) return nneis; nei = &neis[nneis]; }` … `if (n > 0) memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour)*n);` — `DetourCrowd.cpp`, `addNeighbour`

RVO2, which also shrinks its search radius to the current *k*-th distance once the array is full:

> `if (agentNeighbors_.size() < maxNeighbors_) { agentNeighbors_.push_back(std::make_pair(distSq, agent)); }` … `while (i != 0U && distSq < agentNeighbors_[i - 1U].first) { agentNeighbors_[i] = agentNeighbors_[i - 1U]; --i; }` … `if (agentNeighbors_.size() == maxNeighbors_) { rangeSq = agentNeighbors_.back().first; }` — `src/Agent.cc`, `Agent::insertAgentNeighbor`

Detour's array is a fixed member of the agent; RVO2's `std::vector` is reserved once and never outgrows `maxNeighbors_`. Neither touches the allocator per query, and neither pays for ordering a candidate it will drop.

## Technical basis

The standard library documents which of its sorts allocate. The stable sort:

> "The auxiliary memory allocation behavior depends on the input length. Short slices are handled without allocation, medium sized slices allocate `self.len()` and beyond that it clamps at `self.len() / 2`." — `library/alloc/src/slice.rs`, `sort_by`

> "For small inputs 4KiB of stack storage suffices, which allows us to avoid calling the (de-)allocator." … `heap_buf = BufT::with_capacity(alloc_len);` — `library/core/src/slice/sort/stable/mod.rs`

The unstable sort and the selection do not:

> "This sort is unstable (i.e., may reorder equal elements), in-place (i.e., does not allocate), and *O*(*n* \* log(*n*)) worst-case." — `library/core/src/slice/mod.rs`, `sort_unstable_by`

> "Reorders the slice with a comparator function such that the element at `index` is at a sort-order position. All elements before `index` will be `<=` to this value, and all elements after will be `>=` to it [...]. This reordering is unstable (i.e. any element that compares equal to the nth element may end up at that position), in-place (i.e. does not allocate), and runs in *O*(*n*) time." — ibid., `select_nth_unstable_by`

> "The fallback algorithm used for introselect is Median of Medians using Tukey's Ninther for pivot selection. Using this as a fallback ensures O(n) worst case running time" — `library/core/src/slice/sort/select.rs`

And working in place is not a speed penalty: "part of the speed advantage is due to the additional feature of the algorithms to work in-place, i.e., they do not need a significant amount of space beyond the input array. Previously, the in-place feature often implied performance penalties." (`10.1145_3505286`, abstract).

Stability is only worth paying for when two elements can compare equal. Here they cannot: the order is `(distance by total_cmp, key, index)` and each index appears in `near` at most once (every member sits in one grid cell, and the 3×3 block visits each cell once). Under a strict total order every correct sort — stable or not — produces the same permutation, and a selection followed by a sort of the kept prefix produces the same first `max_neighbours`.

## Model adopted

1. Gather candidates exactly as before: every member of the 3×3 block within `neighbour_range`, offered to `sees` in the same order, so the caller's rule sees the same pairs.
2. If the list is longer than `SELECT_ABOVE` (16) **and** than twice `max_neighbours`, `select_nth_unstable_by(max_neighbours, nearest_first)` moves the nearest `max_neighbours` to the front in O(candidates) and `truncate(max_neighbours)` drops the rest.
3. `sort_unstable_by(nearest_first)` orders what is left, then `truncate(max_neighbours)` — for a list the selection skipped, that is the whole list sorted and cut, which is what the old stable sort and cut did.
4. `nearest_first` is one named comparator, `(distance by total_cmp, key, index)`, unchanged.

Every neighbour list, and so every solved velocity, is identical bit for bit; nothing allocates once `near` has grown to the widest candidate list the crowd has produced.

**Why not RVO2's and Detour's bounded insertion.** Insertion into a nearest-first array costs O(candidates × `max_neighbours`) in the worst case, which is quadratic for a caller that sets `max_neighbours` high to mean "everyone in range". Selection is O(candidates + `max_neighbours` log `max_neighbours`) for every setting. RVO2's shrinking radius would also stop offering far candidates to `sees`, which changes which pairs a caller's rule is asked about; that is a different change, measured on its own if it is ever made.

## Parameters

None for the caller: `max_neighbours` and `neighbour_range` keep their meaning and defaults. One internal constant, `SELECT_ABOVE = 16`, is measured and not a knob: `select_nth_unstable` insertion-sorts a slice of up to 16 in full, so selecting first only adds a second pass. Micro-benchmark (one candidate list per call, `(f32, u64, usize)` elements, ns per list, `k` = `max_neighbours`): for `k = 10`, 12 candidates sort 171 ns vs select+sort 182 ns, 16 candidates 231 vs 236, 18 candidates 262 vs 245, 24 candidates 417 vs 322, 64 candidates 1 276 vs 545; for `k = 30`, 32 candidates 612 vs 656 (selection loses), 48 candidates 893 vs 840, 64 candidates 1 286 vs 964 — hence also the "twice `max_neighbours`" condition. Stable `sort_by` and `sort_unstable_by` + cut agree to within 3% up to 20 candidates. The first version of this change selected whenever the list was longer than `max_neighbours`; the bench's corridor (mean 11 candidates, up to 34) then ran 3 to 9% slower than before in all three paired rounds, and the threshold removed that (±2% at 1 000 agents and above).

## Validation targets

- `tests/alloc.rs`: issue #7's dense crowd — 200 members 0.15 apart, `CrowdParams::new(4.0)`, 199 candidates each — allocates 0 times in 5 warm solves (before: 1 000, once per member per solve). **Met.**
- The bench's `rally` `allocs` falls from one per dense member per step (260 619 at 1 000 members, 4 050 400 at 10 000) to the 2 a growing buffer costs. **Met.**
- Every bench hash — every scene, size, member order and thread count — equal to the baseline's; `cand_mean` and `cand_max` unchanged (the same candidates are found and offered to `sees`). **Met: 0 mismatches over 80 rows × 3 rounds × base and new**, and `Crowd::solve` bit-equal to the base over 6 000 random tie-heavy crowds (446 656 solves, duplicate keys, lattices, random `sees`, `max_neighbours` from 0 to 1 000).
- Cost: no case's median step more than 5% slower; the rally's step falls. **Met** (see the loop log for the table). The 1k→10k scaling exponent was to stay put; it rises where the step got cheaper (open 0.864 → 0.936, bottleneck 0.879 → 0.948, rally 1.094 → 1.103, medians of three paired runs; circle and corridor unchanged) because the saving per member follows the candidates per member, which are fewer at 10 000 than at 1 000 in these scenes (open 16.9 against 22.2, bottleneck 38.2 against 51.4). Every absolute time falls and every exponent but the rally's stays below 1.0: a justified trade, logged, not a regression.

## What great looks like

A strategy game rallies ten thousand units on one flag with a 5 m neighbour range. Each unit pays for finding its ten nearest, not for ordering the two hundred around it, and the tick never calls the allocator — the frame-time graph is flat when the army bunches up, the same as when it is spread out. Nothing visible changes: every agent takes exactly the velocity it took before.

## Deviations

- **From RVO2 and Detour: selection after gathering instead of bounded insertion while gathering** (design default, reasons above). The kept set is the same — the nearest `max_neighbours` — except that this crate breaks distance ties by key and then index, as it already did, where RVO2 keeps the earlier-inserted agent.

## Pending index

- `10.1145_3505286`: downloaded this pass (`papers/10/10.1145_3505286.pdf`), not converted or indexed — scribe's big backend was unavailable and bmb busy; the quote above is the paper's abstract.
- Musser, "Introspective Sorting and Selection Algorithms" (1997): Wiley only (see `corpus.md`); the introselect quote is the standard library's own.
- Recast/Detour and RVO2 sources and the Rust standard library: read on GitHub, not home-still documents.
