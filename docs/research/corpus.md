# Corpus — the reading list per domain

Every stem below is a home-still document (`distill_search` / `hs distill search` finds it; `markdown_read` reads it). A source the corpus lacks is listed under **Not in the corpus** with what was tried, so the next pass does not search for it again.

Scanned last: iteration 3 (2026-10-30), twice. `catalog_recent` (200 rows, 21:00–21:06 UTC, nearly all LLM and graphics arXiv embeds) held one document in these domains, `10.48550_arxiv.2210.04624` (below), and one on sorting, `10.48550_arxiv.2206.01784`; a second scan (200 rows, 21:18–21:27 UTC, LLM-agent-memory arXiv embeds only) held none in these domains (`10.48550_arxiv.math_9410209`, Edelsbrunner & Mücke's "Simulation of simplicity", is the only geometry row and is about degenerate inputs to exact predicates, not this crate's float LP). Iterations 1 and 2 found none.

## Velocity obstacles, RVO and ORCA (model)

- `10.1109_robot.2008.4543489` — van den Berg, Lin & Manocha, "Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation", ICRA 2008. RVO; the Circle experiment; "scales linearly with the number of agents".
- `10.1109_tro.2011.2120810` — Snape, van den Berg, Guy & Manocha, "The Hybrid Reciprocal Velocity Obstacle", IEEE T-RO 27(4), 2011. HRVO; per-step cost and collisions vs agent count on a fixed circle (Table III). **Not** the ORCA paper, though this crate cited its DOI for it until iteration 1.
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

- `10.17815_cd.2016.1` — Curtis, Best & Manocha, "Menge: A Modular Framework for Simulating Crowd Movement", Collective Dynamics 2016. Scenario specs (periodic hallway), spatial-query choice per scene.
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
- `book_real_time_collision_detection_ericson` — Ericson, *Real-Time Collision Detection*. Uniform grids, hashed storage.
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

## Not in the corpus

- **van den Berg, Guy, Lin & Manocha, "Reciprocal n-Body Collision Avoidance"**, ISRR 2009, Springer STAR 70 (2011), DOI `10.1007/978-3-642-19457-3_1` — the paper RVO2 implements. `paper_download` (iteration 1): no open-access copy from unpaywall, PMC, Semantic Scholar, Europe PMC, OpenAlex or CrossRef; CORE needs a key. The authors' copy is `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`.
- Seidel, "Small-dimensional linear programming and convex hulls made easy", DCG 1991 — the randomized incremental LP RVO2's three stages descend from.
- Seyfried et al., bottleneck flow and the fundamental diagram of pedestrian movement (2005–2009).
- Singh et al., "SteerBench: a benchmark suite for evaluating steering behaviors", 2009.
- Musser, "Introspective Sorting and Selection Algorithms", *Software: Practice and Experience* 27(8), 1997, DOI `10.1002/(sici)1097-024x(199708)27:8<983::aid-spe117>3.0.co;2-#` — introselect, the algorithm behind `select_nth_unstable`. `paper_search` (iteration 3) found only Wiley's paywalled PDF. Kiwiel, "On Floyd and Rivest's SELECT algorithm", TCS 2005 (`10.1016/j.tcs.2005.06.032`): ScienceDirect only, not tried.
