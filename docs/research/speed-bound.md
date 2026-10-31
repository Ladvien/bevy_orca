# The speed bound — why the linear program's answer leaves the speed disc, and the representation that keeps it in

`new_velocity_in` documents "result magnitude is clamped to `max_speed`", and `Crowd::solve` inherits it. Iteration 2 measured the promise broken by f32 cancellation and logged it as a bug; iterations 3, 4, 5 and 6 each ranked it and took an older or louder item first. A first draft of the fix and its note were parked by pass 1 (`origin/loop/iter-3`, `060ffa3`, `1e22bc5`, `3a50b29`) and never built, tested or gated against today's `main`, which has changed since (`direction_of`, `usable_speed`, the shared passing side). This pass re-measured the bug on `main` at `07c1048` before adopting anything.

`/tmp/o7/probe` (a throwaway crate over a copy of `src/`; deterministic, `SplitMix64`; four families of single solves plus dense `Crowd` packs) on `main`:

| Family | solves | over `1.000001×` | over `1.001×` | worst `|v| / max_speed` |
|---|---|---|---|---|
| sane: 0–12 neighbours within 3 m, 0–3 walls, own speed ≤ 1.2× | 100 000 | 2 893 | 206 | **6.29** |
| two neighbours standing on the agent | 100 000 | 5 122 | 378 | **13.14** |
| a row of neighbours (near-parallel constraints) | 100 000 | 309 | 10 | 1.239 |
| own velocity 10 to 1e15 (finite) | 100 000 | 307 | 258 | 1.06e8 |
| `Crowd::solve`, 3 000 dense random packs of 2–40 members, 30 steps each | 1 896 840 | 15 616 | 701 | **77.45** |

The smallest ordinary case that fails has **two** neighbours and no wall: an agent at `(−0.495, 3.443)` with velocity `(0.564, −0.273)`, radius 0.530 and `max_speed` 1.685, one neighbour 0.93 m away in +y that holds ground (`avoids: false`) and one 1.80 m away at `(+0.55, −1.72)` that steps aside, answers `(2.835, −1.333)`, 1.86 times its top speed. A villager that leaves a knot of forty at 77 times its top speed covers 77 × 1.5 m/s × 0.05 s ≈ 5.8 m in one 20 Hz step, which is what a game sees as a unit teleporting. Nothing in `Crowd`'s inputs is unusual; this is the dense, over-constrained step that Sunshine-Hill says shipped crowds often take.

The reference implementation does the same, at the same rate. `snape/RVO2` (`main`, built as shipped, `clang++ -O2`) on 3 000 random packs of the same shape (2–40 agents in a 2–6 m square, radius 0.25–0.40, `maxSpeed` 1–1.5, preferred speed 0.8–1.5, `timeHorizon` 2, 10 neighbours within 3 m, `dt` 0.05, 30 steps; `/tmp/o7/RVO2/speed.cc`) returns 1 896 840 velocities, **14 805** over `1.000001×` its `maxSpeed`, 675 over `1.001×`, the worst **63.98×**; this crate's 15 616, 701 and 77.45× on its own packs are the same bug at the same rate. `linearProgram3` builds each projected line through the intersection of two lines (below). This is RVO2's numerics, inherited.

## Sources

- **Model:** van den Berg, Guy, Lin & Manocha, "Reciprocal n-body Collision Avoidance" (the authors' PDF, `gamma.cs.unc.edu/ORCA/publications/ORCA.pdf`, read with `read` this pass; the paper itself is still not in the corpus, see `corpus.md`): §5.1 the linear program with "a circular constraint for the maximum speed", §5.3 equation (10) for the dense case, §7 on why a post-hoc clamp is not the model. `gameaipro2-ch19-guide-anticipatory-collision-avoidance` (Guy & Karamouzas; Guy is an ORCA author), p. 12: each velocity obstacle becomes one line.
- **Shipping game / engine:** `gameaipro3-ch19-rvo-orca-how-they-really-work` (Sunshine-Hill, Game AI Pro 3 ch. 19): the over-constrained case is the common one. Recast/Detour `DetourCrowd/Source/DetourObstacleAvoidance.cpp` (`recastnavigation/recastnavigation`, `main`, read and quoted this pass; GitHub, not in the corpus): the speed bound is part of candidate generation. RVO2 `src/Agent.cc` (`snape/RVO2`, `main`, read this pass; GitHub, not in the corpus): `linearProgram3`.
- **Technical:** `book_real_time_collision_detection_ericson` (Ericson, ch. 11 "Numerical Robustness": catastrophic cancellation in a quadratic's discriminant, and rewriting the expression; ch. 5, the point-normal form of a plane); `book_geometric_tools_computer_graphics_schneider` (Schneider & Eberly, p. 46: cancellation and division by near-zero in the quadratic formula); `PBR3_03_Shapes` (Pharr, Jakob & Humphreys, "Running error analysis": why subtraction of nearly equal terms carries the absolute error of the large one).

## Craft principle

ORCA's answer is an optimum over a *bounded* set, and the bound is part of the model, not an afterthought. The authors state it for the ordinary case: "The fact that we include a circular constraint for the maximum speed does not significantly alter the algorithm, and does not affect the running time. The algorithm returns the velocity in *ORCA* that is closest to **v**^pref" (van den Berg et al., §5.1). They state it again for the over-constrained case, where the program is relaxed rather than abandoned: "we select the 'safest possible' velocity for the robot, i.e. the velocity that minimally 'penetrates' the constraints induced by the other robots", `v_new = argmin_{v ∈ D(0, v_max)} max_B d_AB(v)` (§5.3, equation 10): the domain of that minimisation is the speed disc `D(0, v_max)`. Both stages of the model return a velocity inside the disc. An answer outside it is not a rough approximation of ORCA's; it is not in ORCA's feasible set.

The authors also say what is *not* the model. Of kinematic limits they write: "We can either do this as a post-processing step, in which the computed new velocity is 'clamped' to what the kinematic and dynamic constraints allow. This would not strictly guarantee avoiding collisions anymore, but it may work well in practice" (§7). A clamp after the solve gives up the half-planes it just satisfied; the disc has to be a constraint of the program, as it is in §5.1 and §5.3.

Why the answer leaves the disc anyway is a floating-point fact, not a modelling one. `linear_program1` intersects a line with the disc by a quadratic's discriminant, `dot² + r² − |p|²`, and Ericson uses exactly that kind of expression to show the danger: "consider computing the discriminator b² − 4ac of a quadratic equation for b = 3.456, a = 1.727, and c = 1.729. The exact answer is 4·10⁻⁶, but single-precision evaluation gives the result as 4.82387·10⁻⁶, of which only the first single digit is correct! When, as here, errors become significant due to cancellation, the cancellation is referred to as catastrophic" (`book_real_time_collision_detection_ericson`, ch. 11). Schneider & Eberly name the two culprits together: "cancellation of significant digits by subtraction of two numbers nearly equal in magnitude and division by numbers close to zero, both cases resulting in unacceptable numerical round-off errors" (p. 46). Pharr et al. say why the damage is proportional to the large term: "This substantial increase in relative error is called catastrophic cancellation. Equivalently, we can have a sense of the issue from the fact that the absolute error is in terms of the magnitude of |a|, though it's now in relation to a value much smaller than a" (`PBR3_03_Shapes`).

Both happen here, in sequence, and RVO2's source shows where. `linearProgram3` builds each projected line through the two lines' intersection, `line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction` (`Agent.cc`), with `determinant = det(lines[i].direction, lines[j].direction)`. For two near-parallel lines `determinant` is barely above `RVO_EPSILON = 1e-5`, so the point lands up to ~10⁵ m/s from the origin. `linearProgram1` then subtracts `|p|² ≈ 10¹⁰` from `dot² ≈ 10¹⁰` to get a discriminant near `r² ≈ 1`, every digit of which is rounding, and returns `p + t·d` with `p` and `t·d` both ~10⁵ and their sum ~1: off by `10⁵ · 2⁻²⁴ ≈ 0.006` at best and by whole multiples of `r` at worst. An own velocity of 10¹⁰ m/s puts the agent's own lines that far out. RVO2's source concedes the weakness in the same function: "This should in principle not happen. The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept."

The cure is Ericson's: "In many cases rewriting an expression can help avoid a cancellation error." A line does not have to be stored through an arbitrary point. In point-normal form, `n · X = d` with `d = n · P`, and "When **n** is unit, |d| equals the distance of the plane from the origin" (`book_real_time_collision_detection_ericson`, planes): a line is fixed by its unit normal and one number, its signed distance from the origin. For a unit direction `d̂` that number is `c = det(d̂, p)` for any point `p` on the line, and the line's point nearest the origin is `c · perp(d̂)`. Stored there, `|point| = |c|`, `dot ≈ 0`, the discriminant is `r² − c²` of numbers no larger than the disc, and the answer `point + t·d̂` with `|t| ≤ √(r² − c²)` has `|answer|² = c² + t² ≤ r²` up to a few ulps — whatever the line's history.

## How shipping games do it

Sunshine-Hill's account of ORCA in shipped games is that the over-constrained stage is not a corner case: "The problem is that ORCA's solver often does not succeed. … It is easy for a small number of nearby agents to completely rule out all velocities" (`gameaipro3-ch19-rvo-orca-how-they-really-work`). Guy & Karamouzas say the same of the method: "In some cases, the ORCA constraints may overconstrain an agent's velocity leaving no valid velocity choice for this time step" (`gameaipro2-ch19-guide-anticipatory-collision-avoidance`, p. 12). The stage that handles it, `linear_program3`, is the one that carries the far points, so a crowd in a game takes the broken path every step wherever it is dense: 15 616 of 1.9 million solves in the probe's packs.

Detour, whose crowd most engines took or copied, uses a sampler rather than an LP, and builds the bound into candidate generation so no answer can exceed it: `if (dtSqr(vcand[0])+dtSqr(vcand[2]) > dtSqr(vmax+0.001f)) continue;` in `sampleVelocityAdaptive` (and `dtSqr(vmax+cs/2)` in `sampleVelocityGrid`), `DetourObstacleAvoidance.cpp` lines 600 and 469. A shipped engine treats "never faster than `vmax`, up to a fixed slack" as an invariant of the method, not a post-hoc clamp. The LP's equivalent is to keep every line in a form where the disc test and the answer are computed from numbers of the disc's size.

## Model adopted

The linear program is RVO2's, stage for stage and name for name; only where each line is anchored changes.

1. **Every line is stored at its point nearest the origin**, with a unit direction. A neighbour's line is built as RVO2 builds it (`point = vel + responsibility · u`) and then re-anchored: `c = det(direction, point)`, `point = c · perp(direction)`. A wall's line is built at its foot already (`point = approach · b̂`, `direction = perp(b̂)`).
2. **`linear_program3` builds each projected line from the two lines' offsets**, never from their intersection point. The projected line is the set where line *i* and line *j* are violated equally — the authors' "moving the edges of the half-planes … perpendicularly outward with equal speed" (§5.3) — `det(dⱼ − dᵢ, v) = cⱼ − cᵢ`, where `c = det(d, p)` is the line's offset. It passes through the lines' intersection (both violations are zero there) and, for anti-parallel lines, through RVO2's midpoint. With `D = dⱼ − dᵢ`, its direction is `D / |D|` (RVO2's) and its point nearest the origin is `((cⱼ − cᵢ) / |D|) · perp(D / |D|)`. RVO2's same-direction skip (`|det(dᵢ, dⱼ)| ≤ EPSILON` with `dᵢ · dⱼ > 0`) is kept, so `|D|` is never near zero where it is used. No separate midpoint case is needed: the same formula is RVO2's midpoint line when the two are exactly anti-parallel.
3. **`linear_program1` and `linear_program2` are unchanged.** With every point at its foot, RVO2's own discriminant `dot² + r² − |p|²` is computed from numbers no larger than the disc for every line that can cut it, and a line with `|c| > r` still fails it, as it must.

Same half-planes, same three stages, same tie-breaks: in exact arithmetic the answer is identical. In f32 every constrained solve rounds differently, so **every bench hash changes once**, logged as intended; the hashes stay equal across member order and thread count.

What this does not promise: a neighbour whose own state is already ≥ 10⁸ m/s has lines whose offsets `det(d̂, vel + …)` are rounded at that magnitude, so *where* those lines lie is meaningless in f32; the speed bound still holds for the answer, and the answer is still a velocity the program found feasible against the lines it was given.

## Parameters

| Parameter | Value | Source |
|---|---|---|
| `EPSILON` (parallel test) | `1e-5`, unchanged | RVO2's `RVO_EPSILON` |
| speed-bound tolerance in tests | `1 + 1e-6` relative | f32 rounding of `c² + t²` (a few ulps of `2⁻²⁴`); design default. Detour allows an absolute `0.001`. |
| wall normal | divided by its length before use | design default, kept because it cost nothing measurable: new against new-without-it in the paired harness, −1.5% to +1.1% by scene and size (+1.0% only at the 100-agent corridor, two walls per agent, three rounds); the refusal test (`is_normalized`) is unchanged |

## Validation targets

- The probe's five families: **0** answers over `1.000001 × max_speed` (before: 2 893 / 5 122 / 309 / 307 / 15 616), so 0 over `1.001×` (before 206 / 378 / 10 / 258 / 701); 0 non-finite, 0 panics.
- Optimality unchanged: on 1 000 random small cases the answer is no farther from the wish than the best point of a dense sample of the feasible region, and the old and new solvers agree to f32 rounding wherever the old one stayed in the disc (iteration 1: worst gap 0.000000).
- Every bench hash equal across `order=fwd`/`rev` and threads 1 / 2 / default (the hashes themselves change once, logged); bench gains a `speed_max` column (the largest `|v| / max_speed` over every solve of the `threads=1 fwd` run) so the bound is measured on every scene, every time.
- Overlap and wall depth no worse than before in every bench case at eight seeds (a single `overlap_max` digit is chaotic), `wall_corridor` 0.0000; every arrival that arrived before still arrives; `head_on` offsets within 1% of each other.
- Cost: no bench case's step more than 5% slower in the paired harness (old/old/new/new in one process, thread CPU time, with its A/A floor); candidates per agent unchanged (the neighbour search is not touched).
- A permanent test that fails on `main`: the two-neighbour case above, and random dense constraint sets (near-parallel rows, coincident neighbours, fast own velocity) never answering outside the disc; and the same at `Crowd` level for dense packs.

## What great looks like

A game developer sets `max_speed = 1.4` for a villager and never sees one leave a crowd at 13 m/s: not in a doorway jam, not in a knot of twelve, not when a physics glitch hands in a 10¹⁰ m/s velocity, not after a teleport. No spike shows in a velocity graph and no agent pops a metre in a frame. The speed bound is an invariant of the solve, true to f32 rounding for every finite input, the way it is true by construction in Detour's sampler. Nothing else visibly changes: the circle swap, the corridor and the bottleneck look and score as before, and a dense pack of forty gets out of its own way at the speed it was allowed.

## Deviations

- **From RVO2: where a line is stored.** RVO2 keeps each line through the point it was constructed from and builds each projected line through the intersection of two lines; here every line is stored at its point nearest the origin and projected lines are built from the two lines' offsets. The half-planes and the algorithm are RVO2's; the function names are kept. RVO2's own answer leaves the disc by up to 63.98× on random packs (measured this pass, above). Credited in `NOTICE`.
- **From RVO2: the anti-parallel projection.** RVO2 places the projected line of two anti-parallel lines through their midpoint; here the same offset formula gives the exact equal-violation bisector, which is the midpoint line when the lines are exactly anti-parallel.
- **From RVO2: a neighbour line with no direction.** A neighbour whose relative velocity is within `EPSILON` of the cutoff circle's centre has a zero direction (`w_length ≤ EPSILON`, `unit_w = 0`) and no constraint of its own. RVO2's projection against it is an arbitrary line along `−d_i` through a midpoint; here it is line *i*'s own bisector (its violation is identically zero, so "equal violation" says line *i* must not be violated). Neither is derived from the model; the case is a degenerate input, and the speed bound holds for both.
- **Wall normals are normalized** before the wall's line is built. A normal inside `is_normalized`'s tolerance but not unit used to scale the half-plane by `|b|²` (up to `1 ± 2e-4`); the wall is now exactly `v · b̂ ≤ approach` and the speed bound holds for it too.
- **A `max_speed` whose square is not a normal number is refused** (below about `1.1e-19`, `Vec2::ZERO`), as one whose square overflows already was. Not in the pass-1 draft: this pass's hostile sweep found it (169 of 2 623 solves with `max_speed < 1e-15` over the bound, up to 117×: the disc test compared with a square that rounded to zero). The refusal is `usable_speed`'s, which `Crowd::steers` shares, so a member so limited is avoided in full like any member that will not move.

## Results

Measured at the end of iteration 7 (`/tmp/o7`, removed after the pass; the probes are throwaway crates over copies of `src/`).

| Check | Before (`07c1048`) | After |
|---|---|---|
| answers over `1.000001×` (single solves, 4 × 100 000: sane / two coincident neighbours / near-parallel row / fast own velocity) | 2 893 / 5 122 / 309 / 307 | **0 / 0 / 0 / 0** |
| `Crowd::solve`, 1 896 840 solves in 3 000 packs of 2–40 | 15 616 over `1.000001×`, 701 over `1.001×`, worst 77.45× | **0, 0, worst 1.000000238** (two ulps of f32) |
| hostile single solves (200 000: NaN, ±∞, `f32::MAX`, subnormal and 1e20 fields, one or many at a time) | 380 over (210 with `max_speed ≥ 1e-15`, worst 3.9e15×) | **0**, none non-finite, none panicked (before: none either) |
| 40 000 random cases against an f64 brute-force oracle (120 × 120 grid over the disc, 0–2 walls, 1–6 neighbours; 31 719 feasible, 8 277 over-constrained) | 0 infeasible answers, 0 farther than the best sample, 0 worse than the best minimax sample | **the same 0 / 0 / 0**; 18 of 40 000 answers differ from before by more than `1e-3` (5 where the old one stayed in the disc, worst 0.0039), every one of them also passing the oracle |
| `bench` `speed_max` (all scenes, 100 and 1 000 agents, eight crowds each, measured and arrival runs) | up to **1.438** (open 1 000), 1.062 (open 100), 1.0089 (corridor 1 000), 1.0064 (bottleneck 1 000) | **1.000000** in every run |
| `bench` hashes, 20 (scene, size) groups × order × 1 / 2 / 14 threads | one hash per group | one hash per group (17 of 20 differ from before: the three that do not are the free-flowing circle at 1 000, 5 000 and 10 000) |
| walls (`wall_max`), `wall_corridor` | 0.0000, closest 0.800 | 0.0000 in every row and run, `wall_corridor` and `head_on` byte-identical |
| deepest overlap, mean of eight crowds, before → after (paired difference ± standard error) | | bottleneck 100 0.1122 → 0.1125 (+0.0003 ± 0.0037), 1 000 0.1674 → 0.1753 (+0.0079 ± 0.0106), corridor 100 identical, 1 000 0.0479 → 0.0470, open 100 0.0373 → 0.0410 (+0.0037 ± 0.0025), 1 000 0.0585 → 0.0613 (+0.0027 ± 0.0061); circle 100 0.0639 → 0.0404; rally 100 identical, 1 000 0.0750 → 0.0789. None beyond 1.5 standard errors |
| arrivals (3× straight-line budget), eight crowds | circle 20 on the 1 m ring 0.000 (20 stalled, the fixed point), 20 wide 1.000, 100 1.000 (t90 34.6 s), 1 000 1.000 (266.4 s), corridor 100 1.000; bottleneck flow 0.617 / 0.448 / 0.286 (100 / 200 / 400) | the same arrivals and zero stalls where there were none; circle 100 t90 33.8 s, 1 000 264.7 s; bottleneck flow 0.611 / 0.448 / 0.300, stalled 11.2 / 44.6 / 92.8 against 14.2 / 41.9 / 82.8 |
| allocations, `tests/alloc.rs`, warm `Crowd::solve` at 10 000 | 0–2 per case, 5 of 5 | the same per case, 5 of 5 |
| scaling exponent 1k → 10k (macro bench, median of three) | 0.949 / 1.062 / 1.043 / 0.942 / 1.095 (open / circle / corridor / bottleneck / rally) | 0.980 / 1.061 / 1.033 / 0.926 / 1.084 |

**Cost, paired** (six builds of `Crowd::solve` in one process on one frozen mid-flow crowd — base twice, new twice, new without the wall normalisation twice — rotating order, thread CPU time, median of per-rep ratios; A/A within ±0.7% at 1 000 agents, up to ±2.7% at 100 and in the single 5 000 / 10 000 rounds). `new / base`:

| agents | open | circle | corridor | bottleneck | rally |
|---|---|---|---|---|---|
| 100 (3 rounds for circle and corridor, 1 for the rest) | +0.4% | +3.6% | +4.3% | +0.3% | +1.0% |
| 1 000 (3 rounds) | +0.2% | +2.8% | +1.2% | +0.8% | +1.3% |
| 5 000 (1 round) | −0.0% | +2.4% | +3.0% | +0.8% | +0.1% |
| 10 000 (1 round; bottleneck 4) | +0.4% | +1.4% | +1.2% | +2.6% | +2.6% |

Largest: corridor 100 at +4.3% (+4.07, +4.38, +4.44), under the 5% gate. The macro bench cannot resolve this on this host (rally 100 read 410 µs for the base and 175 µs for the new build in the median of three, new/base −58% in two rounds and +1.5% in the third; corridor 1 000 +92% in one round and +2.7% in the next: the laptop mixes fast and slow cores), so its table is kept only as a record: median of three, 20 cases, new against base mean −1.0%, median +1.8%. Candidates per agent are unchanged (the neighbour search is untouched).

## Pending index

- ORCA, DOI `10.1007/978-3-642-19457-3_1`: still not downloadable (see `corpus.md`); the disc as a hard constraint is quoted from the authors' PDF.
- Recast/Detour `DetourObstacleAvoidance.cpp` and RVO2 `Agent.cc`: read on GitHub, not home-still documents.
- Goldberg, "What every computer scientist should know about floating-point arithmetic" (ACM Computing Surveys 1991): not searched; Ericson, Schneider & Eberly and Pharr et al. cover what this note needs.
