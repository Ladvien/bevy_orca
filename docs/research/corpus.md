# Corpus — the reading list per domain

Every stem below is a home-still document (`distill_search` / `hs distill search` finds it; `markdown_read` reads it). A source the corpus lacks is listed under **Not in the corpus** with what was tried, so the next pass does not search for it again.

Scanned last: iteration 11 (2026-10-31). `catalog_recent` (200 rows, 02:06–10:33 UTC; the rows past 09:07 are new since iteration 10) held nothing in these domains: more couples-therapy, forgiveness, parenting and domestic-violence papers (the 10:32–10:33 batch), a forest-ecology paper, the same plume paper (`10.5194_npg-21-269-2014`), the same two MCP/LLM-tooling papers, `The Computational Beauty of Nature` (an EmbedSkip on a Qdrant timeout again) and the SIGGRAPH 2024/2026 and "Advances in Real-Time Rendering" notes. No title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing or parallel stepping. `system_status`: distill up (CUDA bge-m3, idle), both scribe instances healthy (3 and 5 converting, 777 queued), 18 073 documents embedded. The priority item's sources were already in the corpus (see "Right of way" below); two shipping engines' documentation was read on the web.

The scan before it, iteration 10's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 10 (2026-10-31). `catalog_recent` (200 rows, 02:06–09:07 UTC; the rows past 08:44 are new since iteration 9) held nothing in these domains: couples-therapy, forgiveness and childhood-adversity/chronic-pain papers (the 09:07 batch), a forest-ecology paper and a plume paper (`10.5194_npg-21-269-2014`), the same two MCP/LLM-tooling papers, `The Computational Beauty of Nature` (EmbedSkip again), `transvoxel_*` and the SIGGRAPH 2024/2026 and "Advances in Real-Time Rendering" notes. No title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing or parallel stepping. `system_status`: distill up (CUDA bge-m3, idle), both scribe instances healthy (4 and 5 converting, 788 queued), 18 073 documents embedded. The parallel-solve item (`docs/research/parallel-solve.md`) found its model, technical and engine sources already indexed (below); `distill_search` ranks poorly for a parallel-computing question (psychology and rendering chunks came first), so the passages were pulled with narrower queries by stem. The LAN document links are not reachable from this host and the remote `cloud.lolzlab.com` link answered 504, so `distill_search` chunk text and the authors' ORCA PDF (`read`) were the readable routes.

The scan before it, iteration 9's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 9 (2026-10-31). `catalog_recent` (200 rows, 02:06-08:44 UTC; the rows past 06:10 are new since iteration 8) held nothing in these domains: psychology and pain-medicine papers, two candle-flame papers, a forest-ecology paper, two MCP/LLM-tooling papers (`10.48550_arxiv.2610.14334`, `10.48550_arxiv.2606.30317`), `The Computational Beauty of Nature` (an EmbedSkip on a Qdrant timeout), and the SIGGRAPH 2024/2026 and "Advances in Real-Time Rendering" notes again. No title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing, parallel stepping or grid outlines. `distill_search` is up (CUDA bge-m3, idle) and both scribe instances are healthy (2 and 5 converting, 803 queued). The solid-cells item (`docs/research/solid-cells.md`) found its technical sources by `distill_search` (listed under "Solid cells to outlines" below) and read Recast's `RecastContour.cpp` on GitHub.

The scan before it, iteration 8's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 8 (2026-10-31). `catalog_recent` (200 rows, 02:05-06:32 UTC) held nothing new in these domains: the same SIGGRAPH 2024/2026 papers and 2006-2025 "Advances in Real-Time Rendering" notes re-embedded, medical and psychology papers, two candle-flame physics papers, a forest-ecology paper, and `10.48550_arxiv.2512.04320` (VLCs) again. No title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing, wall or obstacle geometry. `distill_search` is up (CUDA bge-m3, idle), both scribe instances healthy (2 and 5 converting); the static-obstacles note (`docs/research/static-obstacles.md`) found its sources already indexed (`10.17815_cd.2016.1`, `gameaipro3-ch19-rvo-orca-how-they-really-work`, `10.1109_robot.2008.4543489`, `book_real_time_collision_detection_ericson`) and read the authors' ORCA PDF for section 5.4.

The scan before it, iteration 7's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 7 (2026-10-31). `catalog_recent` (200 rows, 02:04–05:55 UTC) held nothing new in these domains beyond what iteration 6 listed: the same 2006–2025 "Advances in Real-Time Rendering" notes and SIGGRAPH 2024/2026 papers, plus medical and psychology papers, a forest-ecology paper and one systems paper, `10.48550_arxiv.2512.04320` ("VLCs: Managing Parallelism with Virtualized Libraries": composing parallel libraries without oversubscription; of possible use to the parallel-solve item, not to anything built in iteration 7). No title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing or float robustness. `distill_search` is up (CUDA bge-m3, idle) and both scribe instances are healthy; the speed-bound note (`docs/research/speed-bound.md`) is researched from it, and every quotation in it was checked against the indexed text or the cited source file this pass.

The scan before it, iteration 6's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 6 (2026-10-31). `catalog_recent` (200 rows, 02:03–04:16 UTC) held nothing in these domains: the 2006–2025 "Advances in Real-Time Rendering" course notes and SIGGRAPH 2024/2026 papers re-embedded, medical and psychology papers, two software books (`slatkin_effective_python_3rd`, `The Computational Beauty of Nature -- Anna's Archive`, a general complexity book whose flocking chapter is boids, not collision avoidance); no title matches ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing or parallel stepping. `distill_search` is up (CUDA bge-m3); the passing-side note below (`docs/research/passing-side.md`) is researched from it and found its three kinds of source already indexed (`10.1109_tro.2011.2120810`, `10.1103_physreve.51.4282`, `10.1073_pnas.1016507108`, `gameaipro3-ch19-rvo-orca-how-they-really-work`, `10.1109_icra.2012.6225166`, `10.48550_arXiv.1312.3300`). Shipping code read on GitHub, not home-still documents: Recast/Detour `DetourCrowd/Source/DetourObstacleAvoidance.cpp` (`prepare`, lines 283-303: a dead-ahead neighbour gets one fixed side for every agent) and RVO2 `src/Agent.cc` (lines 571-588: an exact tie projects on the right leg).

The scan before it, iteration 5's, as written then (its "Scanned last" is superseded by this one):
Scanned last: iteration 5 (2026-10-30). `catalog_recent` (200 rows, 02:03–02:22 UTC on 2026-10-31) held nothing in these domains: SIGGRAPH 2024/2026 papers and the 2006–2025 "Advances in Real-Time Rendering" course notes, geometry-processing and graphics batches, medical papers and two EmbedSkips on a 503 from the embedder's memory backend; no title could be matched to ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing, parallel stepping or float robustness. `distill_search` was unavailable (`gpu busy: 4029 MB free < 5000 MB required to load bge-m3`) so the note for #9 was researched from `paper_search`, the authors' ORCA PDF and the sources named in `docs/research/huge-wish.md`. Before that: iteration 4 (2026-10-30). `catalog_recent` (200 rows, 22:57–23:13 UTC) held nothing identifiable in these domains: every stem is an opaque hash, an arXiv or PubMed number, or an Anna's Archive file name (medical, LLM and graphics batches); no title could be matched to ORCA, velocity obstacles, crowds, pedestrian dynamics, low-dimensional LP, spatial hashing or parallel stepping. `distill_search` was unavailable (`gpu busy`) while scanning. Earlier: iteration 3 scanned twice. `catalog_recent` (200 rows, 21:00–21:06 UTC, nearly all LLM and graphics arXiv embeds) held one document in these domains, `10.48550_arxiv.2210.04624` (below), and one on sorting, `10.48550_arxiv.2206.01784`; a second scan (200 rows, 21:18–21:27 UTC, LLM-agent-memory arXiv embeds only) held none in these domains (`10.48550_arxiv.math_9410209`, Edelsbrunner & Mücke's "Simulation of simplicity", is the only geometry row and is about degenerate inputs to exact predicates, not this crate's float LP). Iterations 1 and 2 found none.

## Velocity obstacles, RVO and ORCA (model)

- `10.1109_robot.2008.4543489` — van den Berg, Lin & Manocha, "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation", ICRA 2008. RVO; the Circle experiment; "scales linearly with the number of agents".
- `10.1109_tro.2011.2120810` — Snape, van den Berg, Guy & Manocha, "The Hybrid Reciprocal Velocity Obstacle", IEEE T-RO 27(4), 2011. HRVO; per-step cost and collisions vs agent count on a fixed circle (Table III). **Not** the ORCA paper, though this crate cited its DOI for it until iteration 1.
- `10.1109_robot.2008.4543489` also carries the static-obstacle case of the combined RVO ("the velocity obstacles generated by the (passively) moving obstacles"; static obstacles have zero velocity), read in iteration 8.
- `10.1109_icra.2011.5980408` — van den Berg, Snape, Guy & Manocha, "Reciprocal collision avoidance with acceleration-velocity obstacles", ICRA 2011. AVO; ORCA half-planes from an approximated boundary; OpenMP-parallel per-robot solve.
- `10.1109_icra.2012.6225166` — Alonso-Mora et al., ORCA for non-holonomic robots (tracking a holonomic trajectory, enlarged radius).
- `10.1007_978-3-642-32723-0_15` — reciprocal avoidance for differential-drive robots (ORCA reformulated as a low-dimensional LP, "reciprocal dances").

## Pedestrian dynamics (model)

- `10.1103_physreve.51.4282` — Helbing & Molnár, "Social force model for pedestrian dynamics", 1995. Lane formation in counter-flow, oscillation at a narrow door.
- `10.1073_pnas.1016507108` — Moussaïd, Helbing & Theraulaz, "How simple rules determine pedestrian behavior and crowd disasters", PNAS 2011. Velocity–density relation, bottleneck turbulence, stop-and-go (speeds < 0.05 m/s count as stopped).
- `10.1371_journal.pone.0010047` — Moussaïd et al., "The walking behaviour of pedestrian social groups and its impact on crowd dynamics", 2010.
- `10.1145_1778765.1778860` — Ondřej, Pettré, Olivier & Donikian, "A synthetic-vision based steering approach for crowd simulation", SIGGRAPH 2010.
- `10.1145_1141911.1142008` — Treuille, Cooper & Popović, "Continuum Crowds", SIGGRAPH 2006.

## Crowd frameworks and evaluation (model / shipping)

- `10.17815_cd.2016.1` — Curtis, Best & Manocha, "Menge: A Modular Framework for Simulating Crowd Movement", Collective Dynamics 2016. Scenario specs (periodic hallway), spatial-query choice per scene. Its Listing 1 is a shipped ORCA profile: `<ORCA tau="3.0" tauObst="0.15" />`, a 3 s horizon for agents and 0.15 s for obstacles (iteration 8, `docs/research/static-obstacles.md`).
- `10.48550_arxiv.2309.12841` — Kwiatkowski et al., reward design for crowd simulation via RL (scenario set).
- `10.48550_arxiv.2210.04624` — "WebCrowds: An Authoring Tool for Crowd Simulation", 2022. Found by the iteration 3 `catalog_recent` scan.

## Shipping games and engines

- `gameaipro3-ch19-rvo-orca-how-they-really-work` — Sunshine-Hill, "RVO and ORCA: How They Really Work", Game AI Pro 3, ch. 19. ORCA's infeasibility in small groups, the cornering problem, time horizons.
- `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (also `GameAIPro2_Chapter19_Guide_to_Anticipatory_Collision_Avoidance`) — Guy & Karamouzas, Game AI Pro 2, ch. 19.
- `gameaipro2-ch17-advanced-techniques-robust-efficient-crowds` (also `GameAIPro2_Chapter17_…`) — Game AI Pro 2, ch. 17: congestion maps, robust crowds.
- `GameAIPro2_Chapter21_Dynamic_Obstacle_Navigation_in_Fuse` — Game AI Pro 2, ch. 21.
- `gameaipro4-ch09-obstacle-avoidance-robots-multiple-sizes-forms` — Horizon Zero Dawn's obstacle avoidance for robots of several sizes, Game AI Pro 4, ch. 9.
- `gameaipro2-ch18-context-steering-behavior-driven-steering-macro` — context steering, Game AI Pro 2, ch. 18.
- `s2008-advances-full-course-notes-4` — SIGGRAPH 2008 Advances course (AMD Froblins): GPU crowd local avoidance, deadlock at sinks.
- Recast/Detour (`recastnavigation/recastnavigation` on GitHub, not a home-still document) — `DetourCrowd.cpp` (the crowd: proximity grid, neighbours sorted by distance) and `DetourNavMeshQuery.cpp` (every query refuses a non-finite position or a negative or non-finite radius with `DT_INVALID_PARAM`). Read on GitHub in iterations 1 and 2.

## Spatial hashing and neighbour search (technical)

- `eth-cgl-sim_anim-Sol14a` — Ihmsen et al., "SPH Fluids in Computer Graphics", EG STAR 2014. Uniform grid, cell = support radius, 3^d cells optimal; spatial vs compact hashing; cache locality.
- `eth-cgl-sim_anim-Sol14b` — "A Parallel Architecture for IISPH Fluids". Parallel neighbour search.
- `book_real_time_collision_detection_ericson` — Ericson, *Real-Time Collision Detection*. Uniform grids, hashed storage. Section 7.1 (iteration 8): an object overlapping many cells is linked into each of them ("fitting elephant-size objects into matchbox-size cells" is the too-fine case), which is the cost of a static wall set paid once at build.
- `10.1111_j.1467-8659.2005.00829.x` — Teschner et al., "Collision Detection for Deformable Objects", 2005. Spatial hashing.

## Sorting and selection (technical)

- `10.1145_3505286` — Axtmann, Witt, Ferizovic & Sanders, "Engineering In-place (Shared-memory) Sorting Algorithms", ACM TOPC 2022 (arXiv 2009.13569): in-place sorting as a speed advantage, not a penalty. Downloaded in iteration 3 (`papers/10/10.1145_3505286.pdf`); not yet converted or indexed.
- `10.48550_arxiv.2206.01784` — Adinets & Merrill, "Onesweep: A Faster Least Significant Digit Radix Sort for GPUs", 2022: the radix-sort alternative to a comparison sort of cell keys. Found by the iteration 3 `catalog_recent` scan.
- The Rust standard library's sorts and selection (`rust-lang/rust` tag `1.98.1`, `library/core/src/slice/mod.rs`, `library/core/src/slice/sort/`, `library/alloc/src/slice.rs`; GitHub, not home-still documents): which of `sort_by`, `sort_unstable_by` and `select_nth_unstable_by` allocate.

## Low-dimensional linear programming and robust geometry (technical)

- `book_geometric_tools_computer_graphics_schneider` — Schneider & Eberly, *Geometric Tools for Computer Graphics*.
- `10.1007_pl00009321` — Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates", 1997.
- `10.48550_arXiv.1312.3300` — "Numerical Reproducibility and Parallel Computations" (reproducibility across thread counts).
- `PBR3_03_Shapes`, `PBR3_11_Volume_Scattering` — Pharr, Jakob & Humphreys, *Physically Based Rendering* 3rd ed.: IEEE NaN and infinity semantics in geometric tests (NaN comparisons are false; `0 · ∞` is NaN and propagates).
- `book_real_time_collision_detection_ericson` ch. 11, "Numerical Robustness": INF/NaN in intersection tests must answer "no intersection".
- `book_geometric_tools_computer_graphics_schneider` p. 46 — cancellation and division by near-zero in the quadratic formula (the line–disc discriminant of `linear_program1`).
- `10.1007_s10543-023-00975-x` — Bartels, Fisikopoulos & Weiser, "Fast floating-point filters for robust predicates", *BIT Numerical Mathematics* (catalog date 2022-07-31): round-off in geometric predicates. In the corpus (its abstract was read in iteration 7); not quoted by any note.
- `book_real_time_collision_detection_ericson` ch. 11 also holds the worked cancellation example (a quadratic's discriminant in single precision) and "In many cases rewriting an expression can help avoid a cancellation error"; its plane equations give the point-normal form (`n · (X − P) = 0`, and "`|d|` equals the distance of the plane from the origin" for a unit `n`) that `speed-bound.md` stores a line in.

## Solid cells to outlines (technical)

Found in iteration 9 by `distill_search` for the solid-cells item; each is a home-still document unless marked GitHub.

- `10.1609_icaps.v30i1.6654` — Hechenberger, Stuckey, Harabor, Le Bodic & Cheema, "Online Computation of Euclidean Shortest Paths in Two Dimensions", ICAPS 2020. A grid map's blocked cells are "traced to construct a set of inner-obstacles and enclosure", and "If two obstacles touch each other at a point … it is ambiguous if passing through this touching point is allowed or not".
- `10.1080_2151237x.2011.563682` — Lengyel, "Transition Cells for Dynamic Multiresolution Marching Cubes", Journal of Graphics, GPU, and Game Tools 15(2), 2010 (Terathon, a shipped engine's voxel terrain): ambiguous faces, "the simplest solution, which is enforcing a fixed-edge polarity on ambiguous faces". `transvoxel_dissertation_lengyel2010` (his dissertation) has the same case in Fig. 3.2 and cites Nielson & Hamann 1991.
- `10.1007_bf01900699` — Natarajan, "On generating topologically consistent isosurfaces from uniform samples", The Visual Computer (HP Labs report HPL-91-76, 1991): §3 states the 2-D case, "for case III, there are two possible topologies" (two diagonal corners inside), which is the checkerboard of two diagonal solid cells.
- Recast `Recast/Source/RecastContour.cpp` (`recastnavigation/recastnavigation`, `main`; GitHub, read 2026-10-31): `walkContour`, the boundary walk over a cell grid with one fixed turning rule (`dir = (dir+1) & 0x3; // Rotate CW` at a border, `dir = (dir+3) & 0x3; // Rotate CCW` on a step), visited edges cleared (`flags[i] &= ~(1 << dir)`), then `simplifyContour` for the collinear vertices.

## Parallel stepping (technical)

Found in iteration 10 for the parallel-solve item; each is a home-still document unless marked otherwise.

- `10.1109_icra.2011.5980408` (listed above under ORCA) p. 6: "The implementation was parallelized using OpenMP to exploit the independence of the computations for each robot, and take advantage of the multiple cores available on our testbed system."
- `10.48550_arxiv.2512.04320` — Yan et al., "VLCs: Managing Parallelism with Virtualized Libraries", 2025: libraries that set threads globally "do not allow allocation of specific threads to a call" (the argument against a library that spawns its own).
- `10.48550_arXiv.1312.3300` — Revol & Théveny, "Numerical Reproducibility and Parallel Computations: Issues for Interval Algorithms", 2013: a reduction's order is what a thread count changes.
- `Game Engine Architecture, Third Edition, 3rd Edition -- Jason Gregory …` §8.6: data parallelism, one thread per subsystem, and the job system.
- Recast `DetourCrowd/Include/DetourCrowd.h` and `Source/DetourCrowd.cpp` (`recastnavigation/recastnavigation`, `main`; GitHub, read 2026-10-31): `dtCrowd::update` is one serial call whose velocity-planning loop reuses a single `m_obstacleQuery`.
- Bevy `bevy_tasks-0.20.0/src/task_pool.rs` (vendored; not a home-still document): `TaskPool::scope` and `Scope::spawn` (`Fut: Future + 'scope + Send`).

## Right of way: unequal shares of the avoidance (model, shipping)

Found in iteration 11 for the priority item; each is a home-still document unless marked otherwise.

- `10.1109_robot.2008.4543489` §III-C "Generalized Reciprocal Velocity Obstacles" (p. 4): "there may be natural priorities among agents that motivate a different balance, and we can generalize the Reciprocal Velocity Obstacle concept accordingly. Let us denote the share of the effort agent A takes to avoid agent B by α<sub>B</sub><sup>A</sup>. By definition, α<sub>A</sub><sup>B</sup> = 1 − α<sub>B</sub><sup>A</sup>".
- `gameaipro3-ch19-rvo-orca-how-they-really-work` §19.7.1 "Modifying Weights" (Sunshine-Hill, Game AI Pro 3): the two weights must "add up to 1 and the agents agree on what those weights are"; below 1.0 "their decisions each frame are guaranteed not to fully resolve the collision, and eventually they are guaranteed to veer off sharply and collide".
- van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" §4.2 (the authors' PDF, read again in iteration 11): the ½ is "to 'share the responsibility' of avoiding collisions among the robots in a fair way"; Definition 1's optimality is stated for the equal split.
- A* Pathfinding Project, `RVOController.priority` and "Local Avoidance" (Aron Granberg; `arongranberg.com/astar/documentation/stable/localavoidance.html`, `…/beta/rvocontroller.html`; web, read 2026-10-31): "If an agent has a higher priority, other agents will try to avoid it more, and it will also try to avoid other agents less", with `avoidanceStrength = other.priority / (this.priority + other.priority)` (0.5 when both are 0), plus a separate `locked` flag for a unit that cannot move.
- Unreal Engine `UCharacterMovementComponent::AvoidanceWeight` and `UAvoidanceManager` (Epic Developer Community Forums, "RVO Avoidance", posts 2 and 7, answered by the engineers who wrote it; web, read 2026-10-31): "The idea behind weights is that that two objects trying to move around each other will act differently, where the lower-weight object will divert course more and the higher-weight object less", and "if you have an object that isn't going to reciprocate, it should be at max weight"; default 0.5.

## Not in the corpus

- **van den Berg, Guy, Lin & Manocha, "Reciprocal n-Body Collision Avoidance"**, ISRR 2009, Springer STAR 70 (2011), DOI `10.1007/978-3-642-19457-3_1` — the paper RVO2 implements. `paper_download` (iteration 1): no open-access copy from unpaywall, PMC, Semantic Scholar, Europe PMC, OpenAlex or CrossRef; CORE needs a key. The authors' copy is `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf` and it downloads (1.6 MB); iteration 4 read and quoted it (§3, §4.2, §5.4: reciprocity, the half share, full responsibility for static obstacles) with the `read` tool, in `docs/research/members-that-do-not-move.md`. It is still not a home-still document.
- Seidel, "Small-dimensional linear programming and convex hulls made easy", DCG 1991 — the randomized incremental LP RVO2's three stages descend from.
- Seyfried et al., bottleneck flow and the fundamental diagram of pedestrian movement (2005–2009).
- Singh et al., "SteerBench: a benchmark suite for evaluating steering behaviors", 2009.
- Musser, "Introspective Sorting and Selection Algorithms", *Software: Practice and Experience* 27(8), 1997, DOI `10.1002/(sici)1097-024x(199708)27:8<983::aid-spe117>3.0.co;2-#` — introselect, the algorithm behind `select_nth_unstable`. `paper_search` (iteration 3) found only Wiley's paywalled PDF. Kiwiel, "On Floyd and Rivest's SELECT algorithm", TCS 2005 (`10.1016/j.tcs.2005.06.032`): ScienceDirect only, not tried.
- Anderson, "Algorithm 978: Safe Scaling in the Level 1 BLAS", ACM TOMS 44(1), 2017, DOI `10.1145/3061665`, and Blue, "A Portable Fortran Program to Find the Euclidean Norm of a Vector", ACM TOMS 4(1), 1978, DOI `10.1145/355769.355771` — the overflow-safe 2-norm. `paper_search` (iteration 5) finds Anderson's record and abstract; `paper_download` finds no open-access copy (unpaywall, PMC, Semantic Scholar, Europe PMC, OpenAlex, CrossRef: none; CORE needs a key). The algorithm is read from Reference-LAPACK `BLAS/SRC/dnrm2.f90` (GitHub; Anderson is its author) in `docs/research/huge-wish.md`. Neither is a home-still document.
