---
description: One iteration of the bevy_orca improvement loop
argument-hint: "[optional focus, e.g. 'a deadlock in two-way corridors only']"
---

One run of this command is one iteration of a long-running loop on bevy_orca, a Bevy 0.20 leaf crate of Optimal Reciprocal Collision Avoidance: the 2-D linear program over discs on `bevy_math`'s `Vec2` that turns each agent's preferred velocity into the closest velocity no neighbour can object to, plus `Crowd`, the caller-owned batch helper (spatial hash, scratch buffers, deterministic tie-break) that solves a whole crowd per step. Games use it for local avoidance under whatever global navigator they already have: Sanctuary steps its villagers through it every tick, `bevy_society`'s persons move through it, and bevy_grimoire's crowds-and-paths family issue (#107) previews walkers with it in the editor. Do ONE improvement per iteration, the single most valuable one from a game developer's point of view, finish it completely, then stop.

This crate must be correct AND deterministic AND cheap on the caller's frame. Two discs that overlap, an agent that crosses a wall half-plane, a corridor where two streams freeze face to face, a velocity that changes with member order or thread count, and a step whose cost grows faster than the crowd are all failures, however good the circle swap looks. The judgement here is a crowd-simulation engineer's: no interpenetration and no NaN at any input, reciprocity kept (each side takes half), throughput quoted in agents per millisecond at thousands of agents, and an API that leaves the schedule, the neighbour rule and what is solid to the caller (`CLAUDE.md`, "The non-negotiable: no ECS, no plugin, no schedule").

You do all the work yourself, including looking at any images and judging them. Do not delegate
visual review.

Extra instructions for this run (a focus such as "a deadlock in two-way corridors only"): $ARGUMENTS
They override the choice of item in section 4, not the gates.

## Your work is saved even if you are killed — read this before you start

**The driver saves whatever you leave uncommitted, without touching your branch.** Every ten minutes
it records the whole tree to a snapshot ref (`refs/wiggum/pass-N-...`), and again when the pass ends.
Your `git status`, your index and your branch are unchanged — so this is not a substitute for your own
commit step, which is still the only thing that puts work on `main`.
Recover a killed pass's tree with `git -C <repo> log --all --oneline --grep='WIP(pass'` then
`git show <ref>`.

1. **Commit as you go, not once at the end.** Your commit step is last, behind implement, gate and
   benchmark, so a pass killed at the ceiling never reaches it. Commit the research note alone before
   writing code, as your sections already say; then commit the implementation, then the docs and the
   log. Each commit that lands is work a killed pass cannot lose.
2. **A snapshot ref is not a commit on your branch.** `git log` will not show it. Do not look for
   one; if an earlier pass was killed, its tree is in `refs/wiggum/` and nobody's gate ran on it.
3. **Parked work is a branch, and you look for it before you pick an item.** When a pass is stopped or
   killed with its tree uncommitted, the driver's next start commits that tree to a branch
   (`wiggum/parked-pass-N-<time>`, or on the pass's own `loop/iter-N`) in a commit titled
   `WIP(pass N, parked)` and puts the clone back on `main`: you start clean and nobody tells you. So run
   `git branch --list 'wiggum/parked-*'` and `git log --all --oneline -n 5 --grep='parked'` once, after
   you have triaged the filed issues and before you commit to an item. If the newest parked tree is for
   the item you would pick, or for an issue or bug that outranks your pick, **adopt it**: it is a first
   draft that was never built, tested or gated and was cut off mid-work, so read it, bring its files into
   your own `loop/iter-N` branch (`git checkout <parked-branch> -- <paths>` or `git cherry-pick`),
   finish it, and run the real gate on the result before it ships under your name. Never merge the
   parked branch itself and never delete it. If it is stale (its files conflict with `main`, or the item
   was fixed meanwhile), say so in one line in the log and ignore it. A parked branch that exists only on
   this clone holds a lot of data: the driver keeps anything over 10 MB of new data off origin.


## 1. Orient

1. `CLAUDE.md`: rules, the boundary (no ECS, no plugin, no schedule), the leaf ratchet, reciprocity,
   RVO2 attribution, determinism.
2. `docs/loop-log.md` (create it if missing): what earlier iterations did, found and deferred, and the
   newest iteration's ranked backlog. The next iteration number is the last one plus one.
3. `git log` since the last logged iteration.
4. The `docs/research/<system>.md` note of any system you may touch, and `docs/research/corpus.md` for
   the reading list per domain (create both the first time a pass researches).
5. `git status --short`. The driver (`wiggum.sh`) starts a pass only on `main` with no
   tracked changes. If you find tracked changes you did not make, stop and log them; do not commit or
   revert them.
6. **Re-scan the corpus for new sources.** Run `catalog_recent` (limit 200) and add any ORCA, velocity
   obstacle, RVO/RVO2, reciprocal collision avoidance, crowd-simulation, pedestrian-dynamics (social
   force, fundamental diagram, lane formation, bottleneck flow), low-dimensional linear programming,
   spatial-hashing or parallel-crowd-stepping document that `docs/research/corpus.md` does not list, as
   its own commit (`Research: corpus`).

Preflight home-still now, so research is not blocked later:

- `system_status` (MCP) or `hs status`. HomeStill's servers run on other machines; never restart them
  from here. If distill is down, skip `distill_search` and `distill_index`, search with the CLI
  (`hs distill search "<query>" --output json`) or `paper_search`, and list each unindexed note under
  `Pending index`. Record the live state in the log; do not copy an earlier iteration's outage note.
- Scribe: check `scribe_health`. `scribe_convert` may queue or fail; then rely on the corpus.
- **An outage is not a reason to idle.** `paper_search`, the CLI and the papers already quoted in this
  repository need no GPU; research from them and build.

## 2. Capture the current state

Build first (`cargo build --release --examples --all-features`). The supervisor sets `CARGO_TARGET_DIR`,
so the binaries are under `$CARGO_TARGET_DIR/release/examples/`, never `./target`. Keep everything under
`/tmp/oN/` (`N` the iteration), the baseline under `/tmp/oN/before/`. Keep a worktree of the starting
commit for later comparisons, **detached, at a commit, and off `/tmp`**: `git worktree add --detach ~/oN_base <sha>`, built
with `CARGO_TARGET_DIR=~/oN_base-target`.

Section 9 says how to name, build and remove that baseline. A baseline left behind costs the next pass
a build cache's worth of disk, and one with `main` checked out stops anything else checking `main` out.

**The numbers are the state.** This crate's cost lands on its caller's frame every tick, so measure it the
way a caller pays it:

- **Throughput:** one `Crowd::solve` step at 100, 1 000, 5 000 and 10 000 agents in four scenes: an open
  field of random walkers at a fixed density, the circle swap, a two-way corridor with walls, and a
  bottleneck (a wide room draining through a gap two agents wide). Median and p99 per step over at least
  200 steps after 20 warm-up steps, and **agents per millisecond** (agents / median step time).
- **Neighbour queries:** per step, the mean and maximum candidates scanned per agent (hash cells
  visited, members tested) against the neighbours actually used (`max_neighbours`), so a hash that scans
  too much shows up apart from a solver that is slow.
- **Scaling:** the exponent of step time from 1 000 to 10 000 agents at constant density
  (`log(t10k/t1k) / log(10)`); 1.0 is linear, and anything above 1.15 is a finding.
- **Determinism:** a fixed scenario's positions and velocities hashed bit-for-bit after 500 steps, with
  the member slice in its given order and reversed, and solved at 1, 2 and the default number of
  threads. The crate itself is serial today, so `threads>1` is the caller's parallel form: the same step
  solved with `new_velocity_in` over member chunks on `std::thread::scope` workers, one `OrcaScratch` per
  worker and the same neighbour lists, until the crate offers its own parallel solve, which then
  replaces it. The hashes must be equal.

The release example that does this is `examples/bench.rs`, printing one line per case as
`orca: <scene> agents=<n> threads=<t> median_us=<x> p99_us=<y> agents_per_ms=<a> cand_mean=<c> cand_max=<m> hash=<h>`.
Add it in the pass that first needs it (it does not exist yet), and extend it, never fork it. Record
every line in `/tmp/oN/before/bench.txt`. Run it as `$CARGO_TARGET_DIR/release/examples/bench`.

The loop's host shares its cores with other loops, so a single run is noise: run the bench three times
before and three times after, back to back on the same machine, and compare medians of medians.

The visible behaviour is the `circle_swap` example (a Bevy window; it needs the GPU) and the terminal
examples `head_on` and `wall_corridor`, whose printed offsets and penetration depths are part of the
state: record their output in `/tmp/oN/before/`. When a change can be seen in `circle_swap`, capture a
still or a short GIF under the loop's display (the unit sets `DISPLAY=:89`), adding an `ORCA_FRAMES` /
`ORCA_SHOT` capture knob to the example in the pass that first needs it (it has none yet).

## 3. Inspect closely (you judge the results yourself)

Read every number and every image yourself. Write the **crowd checklist** to `/tmp/oN/findings.md`,
each line with a measurement, not an impression:

- **no interpenetration:** over every step of every bench scene, the deepest overlap
  `max(r_i + r_j - |p_i - p_j|)` between agents that both avoid, quoted as a fraction of the radius; the
  target is 0 beyond float rounding (RVO2 reaches it at the same `dt` and horizon);
- **walls:** the deepest penetration past any wall half-plane in `wall_corridor`, the corridor and the
  bottleneck scenes; the target is exactly 0;
- **no deadlock:** the fraction of agents that reach their goal within a fixed budget (the straight-line
  time times 3) in the circle swap at 20, 100 and 1 000 agents and in the two-way corridor, and the
  number of agents whose speed stays under 5% of their preferred speed for more than 2 seconds while
  away from their goal;
- **reciprocity:** in `head_on`, the two lateral offsets equal to within 1%; with the neighbour holding
  ground (`avoids: false`), the mover's offset is the full avoidance;
- **optimality:** on 1 000 random small cases, the solver's velocity is no farther from the preferred
  velocity than the best of a dense brute-force sample of the feasible region (quote the worst gap);
- **finite, refused, never panicked:** 10 000 steps of hostile input (NaN and infinite positions,
  coincident agents, zero and negative radii, `dt = 0`, `max_speed = 0`, empty and single-member
  crowds) give only finite velocities or `None`, and no panic;
- **determinism:** the bench's hashes equal across member order and thread counts;
- **allocation:** `tests/alloc.rs` still passes, and a warm `Crowd::solve` at 10 000 agents allocates 0;
- **cost:** the bench's medians, p99s, agents per ms, candidates per agent and the scaling exponent;
- **the examples:** a still of the circle swap's centre when behaviour changed, read at full frame and
  zoomed.

## Owner input: recommend, record, proceed

**This loop runs fully automated (owner, 2026-10-29): never stop to wait for the owner.** When a pass needs the owner's input (a design choice the issue or item does not settle, such as a colour, a coverage, an API shape, a schema change or which of two behaviours; or a protected file that must change), it answers the question itself:

1. **Recommend.** Name the two or three real options and pick one, with one line of why. Prefer the conservative choice: reversible, conventional, consistent with the existing code and the issue's stated intent, and the smallest that meets the acceptance.
2. **Assume the owner chose it, and build it this pass.** A decision is never a reason to skip an item or end a pass.
3. **Record it where the owner reviews.** On the issue, a comment beginning `Assumed decision (owner may override):` with the options, the pick and why, and the label `assumed-decision` (`gh label create assumed-decision --color FBCA04 2>/dev/null; gh issue edit <N> --add-label assumed-decision`). With no issue, in the commit body and the log line, beginning `Assumed decision:`.
4. **The owner's reply wins.** An owner comment that contradicts an assumed decision overrides it: the next pass that reads it reworks the item to the owner's choice before anything else.
5. **Earlier blocks are lifted.** An issue or backlog item logged as blocked on, or waiting for, an owner decision is workable now: decide it this way.
6. **Protected checks change only additively.** An assumed decision may add to a protected test or check (a new row or entry, a new field's empty default in a literal, a count or version raised by exactly what the change adds), with the commit message `assumed owner exception: <what>`. It never removes, loosens, skips or re-pins an existing assertion, golden, eval answer or threshold; that is the one block left, so say so on the issue and go on.
7. **Never, whatever you assume.** These need the owner's word in a conversation, and no prompt can give it: yanking a release or changing crate ownership; force-pushing, rewriting history or deleting branches; deleting files or work this pass did not create; editing this command file; spending money or sharing credentials.

## Filed issues come before new features

**A filed issue is a consumer telling you what broke, which outranks anything this pass invented.**
Before you pick an item, list the repository's open issues and triage them:

```sh
gh issue list --repo bevy-coven/bevy_orca --state open --limit 100 \
  --json number,title,labels,createdAt --jq '.[] | "#\(.number) \(.title)"'
```

(If `gh` is absent, the same list is
`https://api.github.com/repos/Ladvien/bevy_orca/issues?state=open&per_page=100`.)

Then, in this order:

1. **A bug issue is the pass's item, ahead of any enhancement.** Anything reporting wrong behaviour —
   a crash, a panic, a visual artifact, a value that is implausible, a documented promise that is not
   kept — is a bug. Take the oldest such issue that is still reproducible, and say why it beats the
   others.
2. **An issue labelled `consumer` is the pass's item next, ahead of the series and your own candidates.**
   A project built on this crate needs it and is waiting. The `consumer:<name>` label says which project;
   `consumer:bevy_grimoire` is the owner's editor. Its own work resumes when the issue is closed as completed
   and it moves its git pin to a `main` commit containing the fix. Take the oldest one you can work this pass.
   - **Build the general form.** The requester is the example, never a name in this crate's API, a path or a
     default: this crate serves other projects too. Where an issue asks for something shaped to one project,
     build the general form and say so on the issue.
   - **Prove it here**, with a test in this repository that fails before and passes after against the issue's
     acceptance. Never build or run the requesting project to prove it.
   - **Pushing to `main` is the delivery.** Requesters pin a git revision, so a consumer fix never needs a
     publish or a version bump.
   - **One that needs a decision the issue does not settle is decided, not skipped:** recommend, record and
     build it (see "Owner input: recommend, record, proceed"). Only a change that would weaken a protected
     check cannot be worked: say exactly what is needed on the issue (`gh issue comment`), write one log
     line, and go on. A blocked consumer issue never stops the pass.
   - **Read `git log` first.** While this loop is stopped, the requester's loop may fix the issue here at the
     source, in its own commits with tests; check whether that has happened so the work is not done twice.
3. **Every other open enhancement issue is the pass's item next, ahead of your own candidates** (the
   owner's rule, 2026-10-29): a filed request was asked for; a candidate this pass invents was not. The
   order of work is: **bugs first** (a bug issue, then a bug already recorded in the backlog or the log),
   then `consumer` issues, then every other open enhancement issue, oldest first. Only when each of them
   is done, blocked or a release issue does "Pick the single most valuable item" choose a feature of your
   own. A blocked issue gets a `gh issue comment` saying exactly what it needs and is skipped; it never
   stops the pass. Name the issues you skipped, and why, in the log line, so the next pass sees the list
   was worked through.
4. **Close the loop.** An issue your pass fixes is referenced in the commit body (`Fixes #N`) **and**
   in the report, so the next pass can see it was addressed. Do not close an issue you did not fix,
   and do not close one whose fix this pass did not verify. For a `consumer` issue the close is the signal
   the requester waits for: close it only through a verified fix pushed to `main`, never as not planned
   without the owner's word.
5. **A release issue is done by the next release.** An issue asking to publish or bump a version is closed by the pass whose release (the "Release" block in the ship section) publishes it, with the crates.io URL.

**When the API fallback answers `403` or `429`** (rate limit — 60 requests/hour per IP unauthenticated,
shared by everything on this network), do not treat it as "no issues": write one line saying the list
could not be read and continue. Never spend a second call to retry it in the same pass, and never infer
from silence that nothing is filed.

**An empty issue list costs nothing:** if it is empty or `gh` is unreachable, write one line saying so
and continue — never spend a pass on the lookup.

## 4. Pick the single most valuable item

**You are here only when no bug and no open issue can be worked this pass** (see "Filed issues come
before new features"); otherwise that issue is the item. Rank candidates from your findings, the loop log's backlog and the extra instructions, always asking
what matters most to a game developer who will ship this:

1. A wrong result: two discs that overlap, an agent that crosses a wall, a NaN or a panic on caller
   data, a velocity that changes with member order or thread count, or a deadlock in a corridor,
   doorway or circle swap that RVO2 passes.
2. A cost the caller cannot afford: a step that scales worse than linearly in agents, a hash that scans
   far more candidates than it uses, an allocation per step, or no way to spread a 10 000-agent step over
   threads without the caller rewriting the ring.
3. API friction: walls the caller must turn into half-planes by hand from segments or polygons, a crowd
   whose setup takes more lines than it should, a query a game needs and cannot make (why an agent is
   slowed, which neighbour bound it), a value the caller must recompute because the crate hides it.
4. Missing features a game expects from local avoidance: static obstacle segments and polygons as
   RVO2's obstacle lines, per-agent avoidance responsibility (priority, not just `avoids`), lane
   formation or a preferred passing side for counter-flows, density-aware speed (the fundamental
   diagram), acceleration-limited or differential-drive agents.
5. The examples looking unconvincing next to what the solver can show: no corridor, doorway or
   bottleneck picture, only the circle swap.

Ownership is a gate: the schedule, the world mapping, who is a neighbour (`sees`) and what is solid
(`walls`) are the caller's (`CLAUDE.md`); global path planning, flow fields and navmeshes are the
caller's navigator's, influence fields and trails are the sibling field crate's that bevy_grimoire's
crowds-and-paths issue pairs with this one, and anything that needs `bevy_ecs` or `bevy_app` is a
wrapper the caller writes. Do not build those here; add a hook instead.

State in one sentence why your pick beats the runner-up. If it is too big, carve off the most valuable
self-contained slice. Record the ranked candidate list in the log, not only the winner.

**A pass never ends idle** (owner, 2026-10-29: "If they're not addressing bugs or enhancing features, they need to evolve their own code base to support Bevy Emerge better or add new features. Research these things in home still, implement them. They shouldn't stay idle."). An empty issue list or a stale backlog is this step's input, never a reason to ship nothing, and a candidate is not "speculation" once it is researched. When the ranked list above gives no workable item, take the first of these that applies:
1. **What the crate's consumers hand-roll.** Read how Sanctuary uses this crate (`gh repo clone Ladvien/sanctuary /tmp/oN/sanctuary -- --depth 1`, then `rg -l 'bevy_orca|orca::'`; removed afterwards) and bevy_grimoire's family issues (`gh issue list -R bevy-coven/bevy_grimoire --label family`, chiefly the crowds-and-paths one, #107). Find a formula, a wrapper or a query either writes by hand around this crate (wall half-planes from a grid or segments, a neighbour filter, a stuck-agent check, a congestion measure, a parallel step), and build its general form here, never named for either. Prove it with a test here.
2. **A feature a shipping game in this domain expects and this crate lacks**, found by research: home-still first (`distill_search`; for a paper the corpus lacks, `paper_search`, `paper_download`, `scribe_convert`, then `distill_index`), then shipped-game practice, then section 5 before any code.
3. **Depth on the weakest existing feature**: measure it against its research note's validation targets and improve the worst gap.

Record the ranked candidates and the pick in the log. A pass that ships nothing names the concrete blocker that stopped all three; "nothing new to do" is not a blocker. **"Not researched yet" is not a blocker either: it is this pass's work.** Take the top item, do section 5 for it now (home-still preflight included), commit the note, and build its first slice. "Building it would be speculation" means the research is missing, so do the research. A log-only iteration is a failed pass, not an outcome, and a pass never copies an earlier iteration's reasons for shipping nothing (measured 2026-10-29: iterations 194 and 195 each shipped a log line saying every item was "not researched", and neither ran the research).

## 5. Research thoroughly (before any code)

Every feature gets researched, new or improved, with no exceptions. An avoidance solver is a model of
how people and robots yield to each other put to work as game AI, so every iteration cites **three**
kinds of source:

1. a **model** source: velocity obstacles and ORCA (van den Berg, Guy, Lin & Manocha, "Reciprocal
   n-body collision avoidance", the paper RVO2 implements and `NOTICE` credits), RVO, pedestrian
   dynamics (social force, the fundamental diagram, lane formation and bottleneck flow);
2. a **shipping-game** source: crowd and local avoidance in shipped games and engines (Game AI Pro
   chapters, GDC talks, Recast/Detour's crowd, studio write-ups on RTS and city crowds);
3. a **technical** source: low-dimensional linear programming (Seidel's incremental LP), spatial hashing
   and uniform grids, cache-friendly agent layout, deterministic parallel stepping and work-stealing
   schedulers, or SIMD.

When the home-still MCP bridge answers `401 Refresh token expired`, search with the CLI,
`hs distill search "<query>" --output json`, which refreshes its own token.

If the corpus lacks one of the three, `paper_search` for it, and record the gap under the note's
`## Pending index`. Write the note with the headings this loop requires (`## Sources`,
`## Craft principle`, `## How shipping games do it`, `## Model adopted`, `## Parameters`,
`## Validation targets`, `## Deviations`, `## Pending index`) plus `## What great looks like`: the
visual qualities and reference images you are aiming for. `## Craft principle` (the model source) and
`## How shipping games do it` must each **quote** the source they rest on, not paraphrase it from
memory.

Any changed default goes under `Deviations` with a citation, or is marked a design default. Commit the
note alone before writing code: `git add docs/research/<system>.md && git commit -m "Research: <system>"`.

## 6. Implement within the crate's rules

- Keep every rule in `CLAUDE.md`. In particular: `tests/leaf.rs` is the ratchet (allowed: `bevy_math`,
  optional `serde` and `bevy_reflect`; no engine crate named in `src/`); widening it is a design decision
  recorded as one.
- **No ECS, no plugin, no schedule.** The crate registers nothing. `Crowd` owns the spatial hash, the
  scratch buffers and the deterministic tie-break and nothing else; the caller keeps `sees` and `walls`.
  A choice about *where* work runs (inline or parallel, how many threads) is offered to the caller as a
  value it passes, with a default that costs the caller's frame least.
- **Reciprocity is the algorithm.** Each side takes half the avoidance; a change that makes one agent
  yield fully has replaced ORCA with something else.
- **Attribution is load-bearing.** Keep RVO2's function names; do not rename them to something more
  idiomatic. New code derived from RVO2 is credited in `NOTICE`.
- **`bevy_math`'s feature set stays `nostd-libm` only.** Nothing may turn on `glam/std` for a consumer:
  that swaps glam's transcendentals and moves downstream replay hashes. `cargo build` (no
  dev-dependencies) is the feature set a consumer gets; it must build on its own.
- **Determinism:** no RNG, no interior mutability, no iteration over a `HashMap`'s order into a result;
  candidates sort by `(distance, key)`. A change that could alter a single bit of output proves it did
  not with the bench's hashes at both member orders and every thread count, or logs the new hashes as an
  intended change.
- **Allocation-free once warm.** `new_velocity_in` and `Crowd::solve` allocate nothing after the first
  call; `tests/alloc.rs` proves it, and a new hot path joins it.
- No `unwrap()`, no `expect` on caller data, no panicking index: this runs in an inner loop over every
  agent every tick. An infeasible program is reported honestly, never replaced by a degraded answer.
- **Bevy 0.20 is pinned.** Read the vendored source (`~/.cargo/registry/src/index.crates.io-*/bevy_math-0.20.0/src/`),
  not bevy.org.
- Plain functions with their own tests for any new maths, testable without an app.
- Re-export new public items from the crate root; every public item carries rustdoc.
- One path per feature, no fallbacks or shims; delete the code the change replaces.
- Tests: for a bug, a test that fails before and passes after. Permanent tests check what a caller sees
  (velocities, separations, wall depths, refusals, determinism); throwaway checks go in `/tmp`.

## 7. Verify and sign off: it must be RIGHT and FAST

You are grading your own work, so grade it as a hostile reviewer would — a crowd-simulation engineer who
will check the deepest overlap, the corridor arrivals and the thread-count hashes before reading
anything you are proud of.

- Rebuild, re-run the bench three times and the crowd checklist under `/tmp/oN/after/`, and compare
  every line with the "before".
  every line with the "before".
- Write the critique BEFORE the verdict (`/tmp/oN/critique.md`): every flaw you can find, against the
  research note's `## What great looks like` and `## Validation targets`. Look for what is wrong, not for
  confirmation that your change worked.
- Then give each feature exactly one verdict: GREAT, NOT YET or FAIL. GREAT means a game could ship on it
  today. "Fine", "acceptable" and "good" are NOT YET.
- On NOT YET or FAIL, write the specific changes that would move it to GREAT, apply them, rebuild,
  re-measure and re-judge. Do not lower the bar to pass.
- After 5 failed rounds on one feature, stop: log the blocker, the remaining flaws and your best
  hypothesis, and revert to the last GREAT state.
- **Performance gate**, equal in weight: median of the three bench medians before and after, same
  machine; no regression over 5% in median step time or agents per ms on any case, no rise in candidates
  scanned per agent and no rise in the scaling exponent, without a logged, justified trade. A
  correctness fix that costs time quotes its price per case.
- **Correctness gate**, equal in weight: the deepest overlap and wall penetration no worse than before,
  no agent that arrived before fails to arrive after, and the determinism hashes equal across member
  order and thread counts.
- Run the full gate (CI's steps, in its order; CI does not build wasm, so neither does this gate):
  1. `cargo fmt --check`
  2. `cargo clippy --all-targets --all-features -- -D warnings`
  3. `RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features`
  4. `cargo build && cargo build --all-features && cargo build --release`
  5. `cargo test && cargo test --all-features`
  6. `cargo build --examples --all-features`

## 8. Ship

Start only after section 7 passes. Then release (the "Release" block at the end of this section).

1. Document: rustdoc for every changed public item, the README for user-visible changes, the research
   note's `Deviations` and `Pending index`, and an entry under `## [Unreleased]` in `CHANGELOG.md`
   (`Added`, `Changed`, `Fixed`, `Removed`).
2. Regenerate `docs/circle_swap.gif` when the behaviour it shows has moved, from the example's capture knob.
3. Log: append `## Iteration N — <date> — <title>` to `docs/loop-log.md`:
   - what changed and why it ranked first (and the runner-up)
   - the sources used: the craft, shipping-game and technical stems, by stem
   - your critique and verdict per capture, with evidence filenames
   - the crowd checklist's numbers, before and after
   - the bench's medians and p99s, before and after
   - the commit hash, and any sibling-repo commits
   - the top 5 remaining items, ranked, for the next iteration
4. Branch and commit: `git checkout -b loop/iter-N` from `main`. Commit by path with messages that
   state the user-visible change: the research note (already committed in section 5), the
   implementation, the docs and changelog, then the log (`Loop log: iteration N, <title>`).
   `git status --short` must be empty afterwards.
5. Push and merge: `git push -u origin loop/iter-N`, then `git checkout main && git pull --ff-only &&
   git merge --no-ff loop/iter-N -m "Merge loop/iter-N: <title>"`. On a conflict, `git merge --abort`,
   log it, and stop.
6. Re-run the gate on the merged tree; a failure stops the pass before the push. Then
   `git push origin main`.

**Release, every pass that changed the package** (owner, 2026-10-29: loops publish, and never wait on CI to do it). After the push, from that pushed commit, on a clean tree with the gate green: if a file `cargo package --list` ships changed since the last published version (`git diff --quiet v<last> HEAD -- <those files>` fails; with no tag `v<last>`, diff from the commit that set that version in `Cargo.toml`, found with `git log -S'version = "<last>"' -- Cargo.toml`), release:
1. Decide the version with `cargo semver-checks check-release` (`cargo install cargo-semver-checks --locked` if missing). Pre-1.0: a breaking change bumps MINOR and resets PATCH, anything else bumps PATCH; never 1.0.0. Move `## [Unreleased]` in `CHANGELOG.md` to `## [<version>] - <date>`, bump `Cargo.toml` and `Cargo.lock`, and commit `release: v<version>`.
2. `cargo publish --dry-run`, then `cargo publish`, locally, with the credentials `cargo login` stored; never print them. Never wait for, watch or re-run a GitHub workflow to release.
3. `git tag -a v<version> -m v<version>`, then push the commit and the tag (merge, never force, on a refused push).
4. `cargo info <crate>@<version>` resolves; put the crates.io URL in the log entry.

**At most one release every 6 hours** (owner, 2026-10-30: bevy_lights published v0.2.1 through v0.2.20 in one day, and then crates.io answered 429 for three passes in a row, each of which did nothing but retry). Release only when the last tag is more than 6 hours old (`git log -1 --format=%ct <last tag>`); until then the changes ride in the next release. A publish refused with 429 is not retried in this pass or the next: log it once and build, and the next release carries it. A pass that only retries a release is an idle pass. Any other failed publish is logged with its error and retried at the next release. Never yank, never reuse a version; fix a bad release forward.

### README, demos and docs stay true

A README is read before anything else, so a false line there costs more than a false line anywhere else.
An audit on 2026-10-27 found one in each of the four loop repos: a `tour.gif` captioned "at noon" over a HUD
reading 10.00 h with a running clock; a `ties.gif` captioned for rivalry colours that appear in none of its 240
frames; a hero caption promising an effect its file never shows; and a demo video captioned as the full
recording of the GIF above it when it shows different objects. Before you push a change a user can see:

1. **Put it in the section it belongs to.** Do not append to the introduction or to the end, and do not
   restructure the README in a feature pass. It reads in this order — what it is in one line, the demo, what
   you get, install, quick start, then the detail sections — and an item added in the wrong place undoes it.
2. **A caption describes the file as it exists.** Open the GIF or still (frame count, size, any HUD burned into
   it) and write only what you can read off it; a figure quoted from program output comes from running that
   program now, not from the log. Recipe: the example's capture knob (section 2).
3. **Media you cannot regenerate here is logged, not left lying.** With no GPU or display on this machine, name
   the stale file and its path in the report and change its caption to what it still shows.
4. **The README's code compiles.** Edit a README snippet only together with the code it shows, and check it against the crate-root doctest it mirrors.

## 9. Clean up when you are done

Worktrees and build directories outside the clone are not free: a `target/` for this crate is tens of GB, a
loop host's `/tmp` may be RAM, and on 2026-10-27 earlier passes had left 37 GB of baseline builds and 5 GB of scratch behind.
**After you ship and before you write the pass's last log line, whether or not you shipped,** remove what *you*
made this pass, and list what earlier passes left.

1. **A baseline is a pair that can be found.** Make it with `git worktree add --detach <path> <sha>` (never a
   branch name: a worktree with a branch checked out stops anything else checking that branch out) and build it
   into `<path>-target`. Then `git worktree list`, plus the `*-target` directory beside each worktree, is the
   whole inventory.
2. **Remove yours.** `git worktree remove <path>` (plain, never `--force`) and `rm -rf <path>-target`, then your
   scratch under `/tmp` (captures, frames, venvs, gate logs, `rustdoctest*`). Remove only what you created this
   pass, and name each path in the log. If git refuses because the worktree has a change in it, name the file
   in the log and leave it.
3. **Leftovers from earlier passes are listed, never deleted.** A killed pass cannot clean up after itself, but
   an agent does not delete what it did not create, so list them once and let a person approve the removal:

   ```sh
   git worktree list --porcelain | awk '/^worktree /{sub(/^worktree /,""); print}' | tail -n +2   # all but the clone
   du -sh <wt> <wt>-target                       # the size of each one that is not yours
   git -C <wt> status --short                    # empty: nothing is lost by removing it
   find <wt> <wt>-target -mmin -120 -print -quit # empty: untouched for two hours, so no live build is using it
   ```

   Log each with its path, size, and whether it is clean, already on `origin/main` and idle (the likeliest to be
   stale). Say so, and still leave it.
4. **Never remove** the clone itself, its own `target/` (the build cache the next pass starts from), anything you
   did not create this pass, or anything a process is using.
5. Write one log line: what you removed and the space it freed (`df -h .` before and after), and the leftovers
   you listed.

## Rules

- One improvement per iteration, fully finished: researched, implemented, GREAT-signed-off,
  benchmarked, gated, committed, documented, logged.
- No feature ships without a written critique, a GREAT verdict and a clean performance check.
- If blocked, log the blocker and pick the next item. A failed gate or a visible defect is not a
  blocker: fix it. A failed push or merge is: log the exact error and stop. Never force-push.

README and docs follow https://github.com/bevy-coven/.github/blob/main/DOCUMENTATION.md; a feature is not done until its README line and, if visible, its GIF exist.
