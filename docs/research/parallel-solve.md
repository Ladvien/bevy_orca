# Parallel solve — one step of a crowd, split over the caller's own threads

`Crowd::solve` is serial: 10 000 walkers cost 21–38 ms a step on this laptop (`docs/loop-log.md`, iteration 9), more than a 16 ms frame. The paper's own answer, and RVO2's, is to spread the per-agent programs over cores. A host that wants that today rewrites the ring `Crowd` is — a grid, a candidate sort, the tie-break, the kick, the `avoids` flag, the obstacle horizon — around `new_velocity_in`, and `examples/bench.rs` already shows what that costs: its caller's ring (`solve_parallel`, 80 lines) is a *different* neighbour rule from `Crowd`'s (a `HashMap` grid; `avoids` read from `pref_vel.is_some()` and not from `steers`; no `sees`; no `usable_state` filter; no finite-kick guard; `obstacle_horizon` not raised to `dt`). It only gives the serial hash because the bench's scenes never reach those cases.

## What the consumers do today

Read this pass (`gh repo clone … --depth 1`, removed afterwards):

- `bevy_society/src/steering.rs`: `mut crowd: Local<Crowd>`, `mut members: Local<Vec<CrowdMember>>`, one `crowd.solve(...)` inside a Bevy system. Serial.
- `sanctuary/src/crowd.rs`: the same ring in `solve_crowd`, reusing buffers between frames; `siege/mob_crowd.rs` places a mob on a ring and has no solver of its own.
- `bevy_emerge_mapper/src/ops_crowd.rs`: the editor's walkers preview. Serial.

None of them steps thousands yet and none hand-rolls a parallel ring. The shape they would take is the one they already use: a `Local` that holds the solver's buffers, now several of them. That is what the API below is for: a host with `Local<Vec<CrowdWorker>>` and `ComputeTaskPool::scope`, not one with a threading library inside this crate.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (authors' PDF, `https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`, §1 abstract and §6 "Performance Results", read with `read`); `10.1109_icra.2011.5980408` (van den Berg, Snape, Guy & Manocha, AVO, ICRA 2011, p. 6).
- **Shipping game:** Recast/Detour `DetourCrowd` (`DetourCrowd/Include/DetourCrowd.h`, `Source/DetourCrowd.cpp`, recastnavigation `main`, GitHub, read 2026-10-31); Gregory, *Game Engine Architecture* 3rd ed., §8.6 (`Game Engine Architecture, Third Edition, 3rd Edition -- Jason Gregory …`); Bevy's own task pool, `bevy_tasks-0.20.0/src/task_pool.rs` (vendored, `TaskPool::scope`, `Scope::spawn`).
- **Technical:** `10.48550_arxiv.2512.04320` (Yan et al., "VLCs: Managing Parallelism with Virtualized Libraries", 2025); `10.48550_arXiv.1312.3300` (Revol & Théveny, "Numerical Reproducibility and Parallel Computations", 2013).

## Craft principle

The model says the per-agent programs are independent, and says what that buys:

> "Because each agent makes independent decisions, we are able to efficiently parallelize the simulation by distributing the computations for agents across multiple processors. We used OpenMP multi-threading to parallelize key computations across eight Intel Xeon 2.66GHz (Clovertown) cores. [...] There is a fairly good scaling in all scenarios – with best observed results in nearly linear scaling for a large number of agents where the constant system overhead becomes far less significant in the overall computation time. [...] For 5,000 agents on eight cores, it takes 8 ms (125 frames per second) to solve the collision-avoidance linear program for every agent in the large circle simulation" — van den Berg et al., p. 14, §6 "Performance Results"

> "The implementation was parallelized using OpenMP to exploit the independence of the computations for each robot, and take advantage of the multiple cores available on our testbed system." — `10.1109_icra.2011.5980408`, p. 6

Independence is the whole property: robot A reads the others' *current* positions and velocities and writes only its own new velocity ("Each robot A independently performs a continuous cycle of sensing and acting", ibid.). A split of the members into ranges therefore changes no velocity, provided each range reads the one frozen world and owns its scratch. Nothing is reduced across members, so there is no sum whose order could move a bit; the technical source says why that matters:

> "This assumption on the order is not guaranteed by any BLAS library we know, and the discussion above tends to show that this assumption does not hold for multithreaded computations." — `10.48550_arXiv.1312.3300`, p. 9 (a reduction's order is what a thread count changes)

The solve has no reduction. Each member's answer is a function of the frozen members, the grid built from them, and the caller's `sees` and `walls` for that member; so the answer is the same for any partition into ranges, any order of running them and any number of threads. That is a property to *test*, not to hope for.

## How shipping games do it

Detour's crowd is one serial call, and its per-agent scratch is one object on the crowd, which is exactly what stops a host splitting the loop:

> `/// Updates the steering and positions of all agents.` / `void update(const float dt, dtCrowdAgentDebugInfo* debug);` — `DetourCrowd.h`, line 315

> `for (int i = 0; i < nagents; ++i) { ... m_obstacleQuery->reset(); // Add neighbours as obstacles. ...` — `DetourCrowd.cpp`, "Velocity planning" in `dtCrowd::update`; `dtObstacleAvoidanceQuery* m_obstacleQuery;` is a single member of `dtCrowd` (`DetourCrowd.h`, line 213)

A studio that wants Detour's avoidance on several threads has to fork it. The engines that do spread such work do it through their own scheduler, and say why one thread per subsystem (or per library) fails:

> "Data parallelism is best suited to situations in which a single computation needs to be performed repeatedly on a large number of data elements." — Gregory, §8.6 (introduction)

> "the number of engine subsystems probably won't match the number of cores on our gaming platform. As a result, we'll probably end up with more threads than we have cores, and some subsystem threads will need to share a core via time-slicing." — ibid., §8.6.2 "One Thread per Subsystem", and a "general-purpose job system" as the way to turn any workload into one that "can take advantage of hardware parallelism"

Bevy's pool is that job system for this crate's consumers, and its contract fixes the shape: a scope's tasks must be `Fut: Future<Output = T> + 'scope + Send` (`Scope::spawn`, `task_pool.rs`), so a task may *borrow* the step (`&CrowdStep`, which must therefore be `Sync`) and own a `&mut` scratch and `&mut` output slice (`Send`), and nothing needs to be `'static`.

## Model adopted

The technical source is the argument against putting threads inside the crate:

> "FFTW [19], SuperLU [34], OpenCV [27], and Sundials [24] allow setting the number of threads globally, but do not allow allocation of specific threads to a call." — `10.48550_arxiv.2512.04320`, §2

and "Complex application codes often combine multi-threaded libraries that do not always support fine-grained management of resources" (ibid.). A Bevy game already has a compute pool; a library that starts `std::thread::scope` workers every step puts a second set of threads on the same cores and cannot be told how many it may use (`CLAUDE.md`: "a choice about *where* work runs … is offered to the caller as a value it passes").

So the crate offers **the split, not the threads**:

1. `Crowd::begin(members, params, dt)` (and `begin_around(obstacles, …)`) does the serial part of a step — rebuild the grid, fill the `steers` flags — and returns a **`CrowdStep<'_>`**, a read-only view that borrows the crowd and the members and is `Sync`.
2. `CrowdStep::solve_range(&self, worker, first, out, sees, walls)` solves members `first .. first + out.len()` into `out`, with exactly the code `Crowd::solve` runs. `worker` is a **`CrowdWorker`**, the per-thread buffers (an `OrcaScratch`, the candidate and neighbour lists, the wall buffer): `Default`, `Send`, allocation-free once warm.
3. `Crowd::solve` / `solve_around` become `begin` plus one `solve_range` over the whole slice with the crowd's own worker. **One path**: the ring exists once, so the serial result and the split result cannot drift, and the bench's hand-written ring is deleted.

The caller chooses the ranges, how many workers, and where they run (`std::thread::scope`, Bevy's `ComputeTaskPool::scope`, rayon). The crate never spawns, never locks and holds no thread-local. A host balances load by cutting the slice finer than its thread count (an interleaved assignment of fixed-size chunks to workers is what the doc example and the bench do).

## Parameters

None added to `CrowdParams`. Public surface: `CrowdStep`, `CrowdWorker`, `Crowd::begin`, `Crowd::begin_around`, `CrowdStep::solve_range`. `first` and `out.len()` are the caller's; a range running past the members leaves the extra slots `None` (no panic, no error: a slot that is not a member is "not solved", the word `solve` already uses).

## Validation targets

- **Thread-count independence by construction**, tested: `solve_range` over any partition (chunk sizes 1, 7, 64, all; chunks run in reverse order) of a random crowd with obstacles, walls, a `sees` filter, unsteered members and hostile members gives the bits `solve` gives. Compile-time: `CrowdStep: Sync`, `CrowdWorker: Send`.
- **Serial cost unchanged:** `scripts/paired.sh` against `02ecd64`, `Crowd::solve` within the A/A floor at 100, 1 000, 10 000 agents in every scene; allocations 0 once warm (`tests/alloc.rs`, extended to `solve_range`).
- **Every bench hash unchanged** at both member orders, and equal at 1, 2, 4, 8 and 14 threads, with the bench's ring replaced by the crate's.
- **Throughput:** agents per ms at 10 000 agents against threads, quoted with the serial fraction (the grid build). The paper reports "nearly linear scaling for a large number of agents"; target: at least 5× the serial agents per ms at 10 000 agents on the open field and the city with the host's 10 performance cores (the laptop has 10 P and 4 E), and a 1 000-agent split no slower than serial (a split that loses at small n is the caller's to skip, and the docs say where it breaks even).
- **No new failure mode:** overlap, walls, speed bound, arrival and the hostile sweeps unchanged (the solve is the same code).

## What great looks like

A host with a Bevy system, `Local<Crowd>`, `Local<Vec<CrowdWorker>>`, a `ComputeTaskPool::scope` and twelve lines of glue gets a 10 000-walker step in a few milliseconds and the exact velocities its serial build gave, on any core count; nothing in the crate names a thread; the example in the rustdoc compiles (`std::thread::scope`) and a doctest runs it against `Crowd::solve`.

## Deviations

- RVO2 parallelises with OpenMP inside the library (`#pragma omp parallel for` over agents in `doStep`); this crate does not, for the reason above. Design default, not a paper's.
- `Crowd::solve`'s signature and bits are unchanged. `CrowdStep` borrows the crowd for its life, so a host cannot start a second step on the same `Crowd` while ranges of the first are still running: that is the borrow checker doing what the frozen world requires.

## Pending index

None. Every source above is a home-still document or was read directly (authors' ORCA PDF, Recast on GitHub, the vendored `bevy_tasks`). `Guy, Chhugani, Kim, Satish, Dubey, Lin & Manocha, "Highly parallel collision avoidance for multi-agent simulation", UNC tech report 2009` (ORCA's reference [10], the paper's own parallel implementation) is not in the corpus: not downloaded this pass.
