# Crowd's neighbour search — stop at the *k*-th nearest

`Crowd::solve` offers `sees` every member within `neighbour_range`, pushes each that passes into a list, and then selects the nearest `max_neighbours` (10 by default) from it. Iteration 3 made the selection linear (`docs/research/candidate-selection.md`) and set one thing aside on purpose: "RVO2's shrinking radius would also stop offering far candidates to `sees`, which changes which pairs a caller's rule is asked about; that is a different change, measured on its own if it is ever made." This is that change.

The cost it removes is measured, in the bench's own scenes, on this iteration's base (`92f578d`, serial, macOS arm64):

| scene, 1 000 agents | `neighbour_range` | candidates offered to `sees` | median step | hash |
|---|---|---|---|---|
| rally | 5 m | 172.8 | 10 180 µs | `578d87…` |
| rally | 3 m | 67.8 | 5 568 µs | **`578d87…` (the same)** |
| rally | 1.5 m | 16.7 | 2 824 µs | differs at the block's edge only |
| bottleneck | 3 m | 51.8 | 2 225 µs | |
| open | 3 m | 21.9 | 2 772 µs | |

Going from 3 m to 5 m in the rally changes no neighbour list (the hash is the same) and nearly doubles the step: each candidate beyond the nearest ten costs about 47 ns ((10 180 − 2 824 µs) over the 156 extra candidates a member is offered at 5 m rather than 1.5 m, per member of 1 000) and contributes nothing to the answer. A host sets `neighbour_range` from the physics (a horizon times a relative speed: 2 s × 2.6 m/s ≈ 5 m is what the rally asks for), and is then charged for the density it gathers in, not the ten neighbours it uses. At 10 000 rally units the step is 123 ms (81 agents/ms, against 354 for the open field).

## Sources

- **Model:** `10.1109_robot.2008.4543489` (van den Berg, Lin & Manocha, RVO, ICRA 2008, p. 6: the neighbour region); `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas, Game AI Pro 2 ch. 19, p. 12: what ORCA drops first when it is over-constrained); the authors' ORCA PDF (§4.2, read with `read`: the guarantee is "collision-free for at least τ time", a statement about the neighbours the half-planes are built from).
- **Shipping game / engine:** RVO2 `src/Agent.cc` `Agent::insertAgentNeighbor` and `src/KdTree.cc` `KdTree::queryAgentTreeRecursive` (`snape/RVO2`, `main`); Recast/Detour `DetourCrowd/Source/DetourCrowd.cpp` `addNeighbour` (`recastnavigation/recastnavigation`, `main`). Both read on GitHub on 2026-10-31.
- **Technical:** `eth-cgl-sim_anim-Sol14a` (Ihmsen et al., "SPH Fluids in Computer Graphics", §2.1: the uniform grid whose cell is the query radius, 3^d cells); the Rust standard library at the toolchain this crate builds with (`rust-lang/rust` tag `1.98.1`, `library/alloc/src/collections/binary_heap/mod.rs`).

## Craft principle

The avoidance model reads a bounded neighbourhood, and which members fill it is the nearest ones:

> "We do not need to take all other agents into account when selecting a new velocity [...]. Therefore, we define a neighbor region NR^i around the current position of agent A_i and only take into account the agents and obstacles inside this neighbor region" — `10.1109_robot.2008.4543489`, p. 6

and when a member is over-constrained it is the far constraint that goes first:

> "In some cases, the ORCA constraints may overconstrain an agent's velocity leaving no valid velocity choice for this time step. In these cases, one option is to drop constraints from far away agents until a solution can be found." — `gameaipro2-ch19-guide-anticipatory-collision-avoidance`, p. 12

So once a member has its *k* nearest, a candidate no nearer than the *k*-th cannot change its answer, however many there are behind it. Everything the search does for such a candidate (fetching it, measuring it, asking the host's rule about it, pushing it, later selecting it away) is work the model has no use for.

## How shipping games do it

Both reference implementations stop at the *k*-th. Detour drops a candidate on one comparison once its array is full:

> `else if (dist >= neis[nneis-1].dist) { if (nneis >= maxNeis) return nneis; nei = &neis[nneis]; }` — `DetourCrowd.cpp`, `addNeighbour`

RVO2 does that and then shrinks the *search*: once the array is full the range becomes the *k*-th distance, and its kd-tree visits a subtree only if the subtree can still hold a nearer agent:

> `if (agentNeighbors_.size() == maxNeighbors_) { rangeSq = agentNeighbors_.back().first; }` — `Agent.cc`, `Agent::insertAgentNeighbor`

> `if (distSqLeft < distSqRight) { if (distSqLeft < rangeSq) { queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left); if (distSqRight < rangeSq) { queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right); } } }` — `KdTree.cc`, `KdTree::queryAgentTreeRecursive` (nearer child first, the other only if still in range)

Neither offers a far agent to a game-side predicate, because neither has one: the whole of RVO2's "who is a neighbour" is distance. This crate has `sees`, which is the one reason iteration 3 left the cutoff alone.

## Technical basis

The grid stays what iteration 1 made it: one `(cell key, member)` entry per member, sorted, the cell `neighbour_range` wide, a row's three cells one contiguous run. For a query of fixed radius that block is optimal:

> "If the cell size is equal to the kernel support, 3^3 cells have to be queried in 3D, which is optimal" — `eth-cgl-sim_anim-Sol14a`, §2.1

but a radius that shrinks to the *k*-th distance makes most of it unnecessary: in a crowd that packs ten members into a radius of 1.3 m, the member's own cell (5 m wide in the rally) holds them all, and an adjacent cell is worth visiting only if its near edge is closer than 1.3 m.

The *k* nearest so far is a max-heap on the order the list already has, `(distance by total_cmp, key, index)`. Iteration 3 rejected bounded insertion into a sorted array because it is O(candidates × `max_neighbours`) when a caller sets `max_neighbours` high; a heap is O(candidates × log `max_neighbours`) for every setting, and the standard library's `BinaryHeap` needs no new code and no allocation once its vector is warm: `From<Vec<T>>` "happens in-place, and has *O*(*n*) time complexity", `into_sorted_vec` consumes the heap and returns the same buffer in ascending order, and `peek_mut` re-sifts when the borrow drops (`binary_heap/mod.rs`).

## Model adopted

For each member, the grid is searched as before but offers its candidates to a bounded heap, with a cutoff that follows it:

1. **`bound`** starts at `neighbour_range` and becomes the heap's worst distance once the heap holds `max_neighbours`. It only ever falls.
2. **A candidate** (a grid entry that is not the member) is dropped, without being offered to `sees`, if its distance is beyond `bound`, or if the heap is full and it does not sort before the heap's worst under `(distance, key, index)`. Otherwise `sees(i, j)` decides, and a candidate it accepts enters the heap, replacing the worst when full.
3. **A cell** is skipped whole when the nearest point of its rectangle is beyond `bound`; a **row run** likewise before its two binary searches. The rectangle is widened by a margin of `1e-5 · (|x| + |y| + range)` so that the rounding of `floor(pos / range)` can never hide a member (a member's cell is computed in `f32`; the margin is far larger than that rounding and far smaller than a cell). A cell at the edge of the `i32` range, which holds every member beyond it, is never skipped. The member's own row is visited first, then the nearer of the other two; entries within a run are in cell order, so a skipped cell is skipped on its key changing.
4. The heap is read out nearest first (`into_sorted_vec`) and is exactly the list the old gather-select-sort produced.

Output: **every neighbour list, and so every solved velocity, is bit-identical** to the previous implementation's, provided `sees(i, j)` is a function of `(i, j)` — which `CrowdStep::solve_range` already requires (a split of a step gives the serial bits only "as long as your `sees` and `walls` are functions of the indices they are handed"). What changes is which pairs `sees` is asked about: a member whose nearest ten are already found is no longer asked about the 150 behind them, so `sees` is called for a subset of the members in range, in the same relative order the grid yields them.

## Parameters

None for the caller: `neighbour_range`, `max_neighbours` and their defaults keep their meaning. `neighbour_range` is still the grid's cell width and so still bounds what a member can see; it stops being a cost knob in a dense crowd (it remains one in a sparse one, where the heap never fills and the whole block is scanned). Internal, measured and not knobs: the cell margin `1e-5 · (|x| + |y| + range)`.

## Validation targets

- **Bit-identical output.** Every bench hash (every scene, size, member order and thread count) equal to the base's; `Crowd::solve` equal to the base over random crowds built to break the cutoff (lattices and coincident members so that distances tie, duplicate keys, members on cell edges and at saturated cells, random `sees` rules, `max_neighbours` from 0 to 1 000, ranges from tiny to larger than the crowd).
- **`sees` is asked about a subset.** For every member, the set of `j` passed to `sees` is a subset of those the base passes, and contains every `j` in the answer.
- **Cost.** The rally's step falls by at least half at 1 000, 5 000 and 10 000 agents; no case's median step is more than 5% slower (paired harness, thread CPU time, A/A floor), in particular the open field and the circle, where the heap rarely fills; the scaling exponent does not rise; allocations stay as they are (`tests/alloc.rs`, bench `allocs`).
- **Candidates.** `cand_mean` and `cand_max` (members offered to `sees`) fall in the dense scenes and do not rise in any.

## What great looks like

A strategy game rallies ten thousand units on one flag and sets the neighbour range from the physics, 5 m. Each unit pays for its ten nearest and the handful of cells they sit in, so the tick costs what the same army costs with a 1.5 m range, and the host's `sees` rule (a team test, a line-of-sight check, an ECS lookup) runs for tens of members per tick instead of hundreds. Nothing visible changes: every agent takes the velocity it took before.

## Deviations

- **From iteration 3's note: bounded selection while gathering instead of selection after gathering.** Its reason against (insertion's O(candidates × `max_neighbours`)) does not apply to a heap, and the call-pattern change it deferred is the point of this note. Recorded as an assumed decision below.
- **From RVO2 and Detour: a heap and a `(distance, key, index)` order, not a sorted array and an insertion-order tie-break** (as in iteration 3: the order is a property of the world, not of the input slice).
- **Assumed decision (owner may override): `sees` is not asked about a member that cannot be among the nearest.** Options: (a) keep offering every in-range member to `sees` and save only the push and the selection (identical call pattern; the 173 calls per member of the rally remain); (b) ask only about members that can still enter the *k* nearest (picked); (c) a new parameter choosing between them. Picked (b): `sees` is already documented as a function of the two indices, so under that contract the answers cannot differ; both reference implementations never evaluate a game-side rule on a far member; (c) is a second path for one feature. Cost: a host whose `sees` has side effects (a counter, a log) sees fewer calls; the README and `Crowd::solve` say so.

## Pending index

- Recast/Detour, RVO2 and the Rust standard library are quoted from GitHub, not home-still documents.
- The authors' ORCA PDF is read from `gamma.cs.unc.edu`, as in iterations 4 and 11; it is still not a home-still document (see `corpus.md`, "Not in the corpus").
