# bevy_orca

> ⚠️ **Vibe Coded** — written by an AI agent working from a human's direction. It is used in a shipping game and covered by tests, but it has had no line-by-line human audit. Read it before you trust it.

Optimal Reciprocal Collision Avoidance (ORCA) as a plain library: the 2-D linear program, over discs, on `bevy_math`'s `Vec2`. No ECS, no plugin, no schedule — you call a function and get a velocity back.

**This repository is canonical.** It is where issues and pull requests belong; the split-out copies elsewhere are the ones that are frozen. MIT OR Apache-2.0.

![Twenty coloured discs starting on a circle and crossing to the opposite side; they shear into a rotating vortex through the contested centre without colliding, then re-form on the far side](https://raw.githubusercontent.com/Ladvien/bevy_orca/main/docs/circle_swap.gif)

That is `examples/circle_swap.rs`: every agent walking to the point directly opposite its own, so all of them want the exact centre at the same moment. **The rotation is not scripted.** Nobody is told to turn, there is no path planner and no flow field — each agent solves a small linear program for the velocity closest to what it wanted that no neighbour can object to, and the vortex is what falls out when twenty of them do that simultaneously.

## What it does

Given an agent's *preferred* velocity — whatever your global navigator wants, a flow field or a path follower — ORCA returns the collision-free velocity **closest** to it, assuming every neighbour is reasoning the same way and each side takes half the avoidance.

That reciprocity is the whole point. Summed-force separation makes two agents on a head-on course shove equally and net zero, so they freeze or oscillate; under ORCA each steps aside by half.

```rust
use bevy_math::Vec2;
use bevy_orca::{Agent, new_velocity};

let me = Agent { pos: Vec2::new(-1.0, 0.0), vel: Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };
let them = Agent { pos: Vec2::new(1.0, 0.0), vel: -Vec2::X, radius: 0.5, avoids: true, priority: 1.0 };

let v = new_velocity(
    &me,
    Vec2::X,          // preferred velocity
    &[them],          // neighbours
    &[],              // wall half-planes: (unit normal toward the wall, max approach speed)
    2.0,              // time horizon, in the host's time unit
    1.0 / 60.0,       // dt, in the same unit
    1.5,              // max speed
);
```

`avoids` is whether that neighbour is *also* running avoidance. When it is, the pair splits the avoidance 50/50 (unless their `priority` differs: see [right of way](#right-of-way)); when it is not — an idle agent holding ground — the mover takes the **full** avoidance, so a stationary agent is not assumed to step aside and then walked through.

**The result is always finite, whatever you hand in.** A paused frame, a first frame or a time-scale of 0 hands in `dt = 0`, a physics blow-up hands in a NaN velocity, and a NaN written into a caller's `Transform` makes its agent vanish for good. So every input the solve cannot use has one documented answer:

- A `max_speed` that is not finite, not positive, so large its square overflows `f32` (above about `1.8e19`), or so small its square is not a normal number (below about `1.1e-19`) gives `Vec2::ZERO`; so does a non-finite preference.
- An agent whose own position or velocity is not finite, or whose radius is negative or not finite, gives `Vec2::ZERO`: it holds still for that step, and once you write the zero back its velocity is usable again. A radius of 0 is a point and is fine.
- A `dt` or `time_horizon` that is not finite, not positive, or so small its reciprocal overflows gives your preferred velocity, clamped — no time passes, so nothing can be constrained.
- A wall whose normal is not a unit vector (`Vec2::is_normalized`) or whose approach speed is not finite gives `Vec2::ZERO`. Walls are hard, so one that is not a half-plane holds the agent still rather than being dropped.
- A neighbour whose position or velocity is not finite, or whose radius is negative or not finite, is skipped — one corrupted agent does not move anyone else — and so is one so far away that its constraint overflows `f32`.

A finite preference is always usable, however long. One longer than `max_speed` is clamped to the speed disc along its own direction, and that includes one too long to square (above about `1.8e19`, which `offset / time_left` produces when `time_left` is tiny): the agent runs at full speed that way, it does not stop. `Crowd` follows the same rule for a member's `pref_vel`, and leaves its sideways nudge off a wish so large that the nudge cannot be added.

Finite, in-range input passes every one of these checks, so none of them changes a single bit of a normal solve.

**The result is never faster than `max_speed`**, to `f32` rounding (a relative `1e-6`), for any finite input: in a knot of forty agents, with a neighbour standing on the agent, with a navigator that hands in an own velocity of `1e10`. The speed disc is a constraint of the linear program, not a clamp after it. The reference implementation does not keep this promise: RVO2 (`snape/RVO2`) answers 14 805 of 1.9 million velocities over its `maxSpeed` on random packs of 2 to 40 agents, by up to 64×, and this crate did too (15 616, up to 77×) until every line was stored at its point nearest the origin (see `NOTICE`); packs of the same shape now give none over `1.000001×`, and `bench` prints `speed_max` for every scene so a regression shows.

## A whole crowd

Solving one agent at a time leaves every host to write the same ring around it: a spatial hash to find neighbours without an O(n²) scan, a reused scratch buffer, a deterministic tie-break, the wall buffer. [`Crowd`] is that ring, once.

```rust
use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams};

let members: Vec<CrowdMember> = people
    .iter()
    .map(|p| CrowdMember {
        key: p.id,                      // your stable id: breaks ties between equidistant neighbours
        pos: p.pos,
        vel: p.vel,
        radius: 0.4,
        max_speed: 1.6,
        pref_vel: Some(p.wish),         // None = another system moves this one this step
        priority: 1.0,                  // right of way against another member that steps aside; 1.0 = equal
    })
    .collect();

let params = CrowdParams::new(4.0);     // neighbour range, in your units
let mut crowd = Crowd::default();

let solved = crowd.solve(
    &members,
    &params,
    dt,
    |i, j| members[i].team == members[j].team,     // who is a neighbour: your rule
    |i, buf| fill_walls(members[i].pos, buf),      // what is solid: your rule
);
for (member, v) in members.iter().zip(solved) {
    if let Some(v) = v {
        step(member, *v);
    }
}
```

The caller still owns the two gameplay decisions — **who is a neighbour** (`sees`) and **what is solid** (`walls`). `Crowd` only decides how they are found and how they are solved. `key` is what makes the result a property of the world rather than of your iteration order: candidates sort by `(distance, key)`. The side a crowd passes on is not the key's but one number's: `CrowdParams::nudge` kicks every member toward the **same** side of its own wish, the right for a positive value (clockwise in your plane, which is where ORCA's own tie-break sends two agents walking straight at each other) and the left for a negative one, so two agents meeting head-on are kicked past each other and a crowd crossing a crowd turns one way instead of every way.

A member is `None` when it has no preference, its preference is not finite, or its position is not finite. Every other member gets `new_velocity_in`'s answer, so one whose velocity or radius is unusable, or whose `max_speed` is not finite, not positive, so large its square overflows or so small its square is not a normal number, is solved to `Vec2::ZERO` and holds still; and a member whose position, velocity or radius is unusable is nobody's neighbour and takes none of the `max_neighbours` places.

Reciprocity is assumed only of a neighbour this step moves. A member that is `None`, or that is solved to a stop because its `max_speed` leaves it no velocity (root a unit by setting it to `0.0`, and keep its wish if you like), is avoided in **full** by everyone near it; every other neighbour takes half. The solver's skip and the avoidance flag come from one test, so they cannot disagree.

`Crowd` allocates nothing after its buffers are warm, wherever the crowd walks and however tightly it packs: its neighbour grid is one entry per member, sorted by cell and rebuilt every solve, so its memory and its cost follow how many members there are and not how much ground they have covered; and of the candidates in range only the nearest `max_neighbours` are kept, found and ordered in place, so a member with hundreds of others in range pays for its ten neighbours rather than for ordering the rest. `CrowdParams::new(range)` gives a 2-second horizon against other members, a 0.5-second one against [obstacles](#static-obstacles), at most 10 neighbours and a 5% nudge to the right; set `nudge` to `0.0` to keep perfectly symmetric input perfectly symmetric, or negative to pass on the left.

### Right of way

Reciprocity splits a dodge in half because the two members are alike. When they are not — a villager standing at a stall, a cart in a street of walkers, a captain at the head of a column — give each member a `priority`: `CrowdMember::priority` (`Agent::priority` for `new_velocity_in`), `1.0` by default, and a crowd that never sets it is plain ORCA. Of the dodge a collision course asks for, a member takes `other / (this + other)` and the other takes the rest: `3` against `1` takes a quarter and leaves the other three quarters, `0` takes all of it, and `∞` against anything finite takes none.

```rust
CrowdMember { priority: 8.0, ..stall_holder }   // walkers take 8/9 of every dodge around him
```

It is a ratio (`2` against `1` is `20` against `10`), and the two shares of a pair add to 1 because both are computed from the same two numbers: that sum is the whole of what ORCA's collision-free guarantee asks of the split (the RVO paper's generalisation, `α` and `1 − α`), and a pair whose shares came from two independent settings could add to less and "veer off sharply and collide" (Game AI Pro 3, §19.7.1). The A* Pathfinding Project's `RVOController.priority` is the same formula, and Unreal's `AvoidanceWeight` the same idea. `avoids: false` is still the full avoidance whatever the priority (a claim on the way is not the ability to stand still: for that, `pref_vel: None` or a `max_speed` of `0`), walls and `Obstacles` are always full, and a NaN or negative priority counts as `0`: the member gives way and no neighbour is skipped for it. `tests/priority.rs` holds it to the maths: the pair's total dodge is the equal pair's to `1e-4` whoever gives it, 30 000 random pairs with random priorities (zero and infinite included) are collision-free over their horizon exactly as at equal priorities, and equal priorities of any value are the plain solve bit for bit.

What it does to a crowd, measured in `bench` (`ORCA_PRIORITY=p` gives the members heading right priority `p` and prints each stream's own numbers; the city's 1 000 walkers at the default lanes, the mean of eight crowds, delay being the arrival time beyond the straight-line time):

| `p` | heading right | the others | mean | stalled > 2 s per 1 000 | deepest overlap |
|---|---|---|---|---|---|
| 1 | 2.61 s | 2.84 s | 2.73 s | 1.9 | 0.047 r |
| 2 | 2.40 s | 3.16 s | 2.78 s | 4.9 | 0.064 r |
| 4 | 2.39 s | 4.07 s | 3.23 s | 4.0 | 0.069 r |
| 8 | 2.31 s | 4.71 s | 3.51 s | 12.1 | 0.131 r |

A ratio of 2 gives the stream heading right two tenths of a second and costs the others three (the mean moves by 0.05 s); beyond about 4 the stream that gives way pays more than the other saves, because the other is already near free flow, and the street stalls and overlaps more, so use a modest ratio for a stream and a large one for a member the street should treat as nearly a wall. Where a crowd crushes it costs overlap too: the 100-agent circle swap's deepest overlap is 0.040 of a radius at equal priority, 0.120 at 2 and 0.209 at 8, with everyone still arriving. It does not free a walker boxed against a wall by a head-on one: with the lanes 0.2 m from the brick (`ORCA_LANE=1.0`) 19.3 of 1 000 stall at equal priority and 23 to 26 at 2 to 8. One standing villager (a wish of zero, so he still steps aside) with thirty walkers going past is pushed from his spot by up to 0.41 m at priority 1, 0.32 m at 2, 0.22 at 4, 0.14 at 8, 0.04 at 32 and not at all at `∞`, with the walkers' paths 0.13% longer than the straight line at every priority and no two discs touching (`a_stall_holder_with_priority_is_walked_around_by_a_stream`).

### Across threads

A step is one serial part and a lot of per-member work that reads a frozen world and writes only its own answer. [`Crowd::begin`] does the serial part (the neighbour grid: about 3% of a 10 000-walker step) and returns a [`CrowdStep`]; [`CrowdStep::solve_range`] does the rest for any run of members, into any slice of output, with a [`CrowdWorker`] (the per-thread buffers) of its own. The crate starts no thread, takes no lock and reads no thread count — it would put a second set of threads on the cores your engine already schedules — so you cut the ranges and run them where your game runs work, here on Bevy's task pool:

```rust
use bevy::tasks::ComputeTaskPool;

let step = crowd.begin(&members, &params, dt);               // the serial part: builds the grid
let chunk = 256;                                              // one worker per chunk: they are only buffers
workers.resize_with(members.len().div_ceil(chunk), CrowdWorker::default);
solved.resize(members.len(), None);                           // `Vec<Option<Vec2>>`, kept like the `Crowd`
let ranges: Vec<_> = solved.chunks_mut(chunk).zip(workers.iter_mut()).enumerate().collect();
ComputeTaskPool::get().scope(|scope| {
    for (c, (slice, worker)) in ranges {
        scope.spawn(async move {
            step.solve_range(worker, c * chunk, slice, |i, j| sees(i, j), |i, buf| fill_walls(i, buf));
        });
    }
});
```

`out[k]` is member `first + k`, exactly what `solve` returns for it (`None` for a member that is not solved, and for a slot past the last member). `CrowdStep` is `Copy`, `Send` and `Sync` and a `CrowdWorker` is `Send`, so a task of `bevy_tasks`' `scope` (`Future + 'scope + Send`) can hold both with nothing `'static` (the `sees` and `walls` closures you give it live in the task too, so they must be `Send`); `tests/task_pool.rs` runs this shape on the real pool. `begin_around` is the same with [`Obstacles`](#static-obstacles).

**The split changes no velocity.** No member's answer reads another's and nothing is summed across members, so any partition of the members into ranges, solved in any order on any number of threads, gives every slot the bits `solve` gives (as long as your `sees` and `walls` are functions of the indices they are handed): the thread count is not an input to the simulation, and a replay recorded on four cores plays back on fourteen. Measured on a 14-core laptop that was also running other jobs (a pure compute loop scales to only about 3 of its cores here), a 10 000-walker open field took 28 ms serially and 9.4 ms on a 4-, 8- or 14-thread Bevy pool, 1 000 walkers 2.4 ms and 0.76 ms, with the pool's total CPU time 0.98 to 1.14 times the serial step's: the split costs no more work than it spreads. On a pool a small crowd loses nothing: at 256 members a chunk, a hundred walkers are one task and take what `solve` takes (165 against 167 µs), 300 walkers are 1.4× faster, 1 000 sparse walkers (0.7 ms serially) 2.5× faster. Starting a thread per step instead, as `bench`'s `std::thread::scope` rows do, costs 100–170 µs and loses below about a thousand walkers.

## Static obstacles

`walls` is for what changes from step to step. Geometry that does not — buildings, a wall with a doorway, the edge of the map — is an [`Obstacles`], built once and handed to [`Crowd::solve_around`]:

```rust
use bevy_math::Vec2;
use bevy_orca::{CrowdParams, ObstaclesBuilder};

let mut builder = ObstaclesBuilder::new();
builder.add_box(Vec2::new(6.0, 6.0), Vec2::new(4.0, 4.0))?;    // a solid rectangle: a building, a pillar
builder.add_cells(Vec2::ZERO, 1.0, width, &solid)?;            // a tile map's solid cells (row-major bools): one outline per region
builder.add_solid(&pillar_outline)?;                           // any polygon whose inside is solid, either winding
builder.add_wall(Vec2::new(0.0, 0.7), Vec2::new(0.0, 25.0))?;  // a wall with no thickness
builder.add_enclosure(&map_edge)?;                             // a polygon whose inside is open floor
let obstacles = builder.build()?;   // refused here, once: a NaN, a zero-length edge, a polygon that crosses itself

let params = CrowdParams { obstacle_horizon: 0.5, ..CrowdParams::new(4.0) };
let solved = crowd.solve_around(&obstacles, &members, &params, dt, |i, j| sees(i, j), |_, _| {});
```

These are RVO2's **obstacle lines**. For every edge within `obstacle_horizon · max_speed + radius` of a member, the line tangent to that edge's velocity obstacle at the point nearest the member's current velocity: a face, a cutoff circle round a vertex, or a leg that lets a walker bend round a convex corner at speed instead of braking for it. They are hard (cast ahead of every neighbour and never relaxed, in a knot of forty as in an empty street), and the member takes **all** of the avoidance, because a building does not step aside. They are the `walls` `new_velocity_in` always took, derived from geometry, so a caller of the single-agent API calls `Obstacles::push_walls` and passes the result on.

- **It has its own horizon.** `CrowdParams::obstacle_horizon` (0.5 s by default; `solve_around` raises it to `dt` if it is shorter, because a horizon under the step lets a member cross a wall within it) is separate from `time_horizon` because a walker should not be shy of a wall it has to pass close to in order to avoid another walker: the ORCA paper asks for "a smaller value of τ with respect to obstacles than with respect to other robots", and Menge ships `tau=3.0` against `tauObst=0.15`. In `bench`'s bottleneck (400 agents, three times the straight-line time, eight crowds each) a 2 s obstacle horizon gets 19% of the room through the gap; from 1 s down to the step itself it is flat at 26–30%.
- **You say what is solid; it orients the polygon.** RVO2 needs counter-clockwise vertices and quietly walks agents through a clockwise polygon. Here `add_solid` takes either winding, `add_enclosure` is the same polygon with the open floor inside, and a polygon that crosses itself, has no area or has a zero-length edge is an `Err` at `build`, not a misbehaving wall later.
- **It does not route and it does not rescue.** An obstacle keeps a walker's disc off its edges; the direction round a building is the walker's wish. A walker whose centre is already *inside* a solid (spawned there, teleported there) sees none of its edges, as in RVO2: put it back outside yourself.
- **A tile map is its outline.** `add_cells(origin, cell_size, width, &solid)` takes the grid your flow field already reads (`solid[row * width + column]`, `true` for solid, outside the grid open floor) and traces the *boundary* of each region of solid cells once: a wall of any length is one edge, a block of cells is a box's four edges (a walker beside it is given bit for bit the lines `add_box` gives), a pocket of floor inside a solid is a hole, and two cells that meet only at a corner are two polygons that touch there, so the gap between them is shut. One `add_box` per cell works too (adjacent boxes are as good as one wall: a test holds a walker against eight touching boxes and against one long one) but it is four edges for every cell however deep in a building. A map's outline is still an enclosure, and a courtyard is an enclosure inside a solid.

It is RVO2's, measured against RVO2: on 60 000 random convex polygons, walls and concave polygons, built as shipped (as a flat list, because RVO2's kd-tree splits an obstacle at another's line, so what it casts depends on how the tree partitioned), this crate casts the same lines and returns a velocity within `3e-6` of RVO2's. It differs in two places on purpose, both where RVO2 skips a "foreign" leg because the neighbouring edge casts that line and the neighbouring edge does not. For a wall with no thickness the neighbouring edge is its own other side, which a walker on this side cannot see, so a walker a few ulps from the wall's face, moving along it, is handed no line at all and walks into the wall, by up to a whole step: of 400 000 random walkers touching a random polygon or wall, RVO2 built without fused multiply-add lets 5 874 in; this crate lets none. And viewed obliquely from beyond the end of a face, an edge defines the velocity obstacle by the corner and its "neighbour" is itself: a walker creeping along the extension of a face a few ulps from a convex corner, with its wish into the corner, is handed no line and steps into the corner by a whole step (28 agent-steps in 21.6 million of a crowd on random caves, 4.7 cm); this crate casts the line.

Measured in `bench` (macOS arm64, serial, medians of three interleaved rounds; flow shares are the means of eight crowds):

- **The doorway.** The bottleneck's wall as `Obstacles` passes 0.60, 0.43 and 0.27 of 100, 200 and 400 agents through its gap in three times the straight-line time, against 0.61, 0.45 and 0.30 for the same wall as closest-point half-planes, and 0.58, 0.42 and 0.23 for RVO2's own obstacle lines in the same scene (its kd-tree, no sideways nudge): an ORCA obstacle brakes for a wall a little earlier than a half-plane that lets a walker run at it until its last step, and this crate is level with the reference. The deepest disc into the wall is 0 in every crowd, and the deepest overlap between two agents is lower (0.16 of a radius at 400 against 0.21).
- **The city.** Walkers on the streets of a grid of 8 m buildings (a quarter of a walker per m² of street, in four lanes that keep 0.8 m clear of the brick), with the buildings as `Obstacles`: no disc reaches a building at any size from 100 to 10 000 walkers (the deepest is 0.0000 m), everybody arrives, 2 of 1 000 stall for over two seconds at some point, and 90% are there by 81.0 s. The same city the way a host does it without obstacles, with the buildings out of the solve and any walker whose disc ends a step in one pushed back out, reaches 90% by 81.5 s with 1 stalled, because its walkers lean on the brick: 0.3% of all agent-steps end inside a building, and the deepest overlap between two walkers is 0.19 to 0.23 of a radius against 0.04 to 0.05. The obstacles cost between nothing and 14% of a step, and the runs disagree about where (at 5 000 walkers 9.0 against 9.0 ms by the median of three rounds and 9.8 against 8.6 ms in one full run; at 10 000, 21.1 against 19.2 ms and 20.8 against 19.3 ms, which is 472 against 520 agents per ms); the exponent from 1 000 to 10 000 is 1.07 for both. Every hash is equal across member order and 1, 2 and 14 threads. Put the outer lanes 0.2 m from the brick (`ORCA_LANE=1.0`) and a head-on walker in one has no room to step aside on its wall side: 19 stall against 2, and 90% take 86.1 s against 82.0 s, the price of walls that are hard in the solve.
- **No cost for who does not use it.** `Crowd::solve` is untouched: a paired timing of the base commit against this one on the same frozen crowds (`scripts/paired.sh`, thread CPU time, rotating order, 300 rounds) reads −1.1% to +1.3% on every scene at 100, 1 000 and 10 000 agents, inside the ±1.2% that two copies of identical code differ by; `solve_around` on a frozen city of 1 000 and of 10 000 walkers, at both lane widths, 0.994 to 1.023 against a floor of 0.987 to 1.007. `ObstaclesBuilder` builds 160 000 edges in 27 ms.
- **A tile map.** The city again, as a grid of 1 m solid cells (an 8 × 8 square of them per building), through `add_cells`: the walkers are bit for bit the `city`'s (the same state hash at every size from 100 to 10 000, the same arrivals) on 16 to 1 936 edges where one `add_box` per cell is 1 024 to 123 904, and the builder takes 2.0 to 2.5 ms where the boxes take 26 to 30 ms at 10 000 walkers. Solving that crowd with the merged outlines against one box per cell, interleaved in one process on the same frozen crowd, the boxes cost +3% (1 000 walkers) and +11% (10 000) per step at the default lane, and +22% and +24% with the outer lanes 0.2 m from the brick, where every walker has a wall in range (two copies of the merged outlines differ by −2% to 0%). The build is `O(cells)` every call, so rebuild when the map changes: a city-like map of 360 × 360 cells takes 1 ms, 1 000 × 1 000 takes 18 ms and 2 000 × 2 000 takes 74 ms (best of five); at 30% random fill, where nothing merges, a 200 × 200 map takes 3.5 ms and a 1 000 × 1 000 one (660 000 edges) 0.2 s.

## Units and plane

`Vec2` is **the host's ground plane**, in the host's units: the crate never converts and never assumes a metric. `time_horizon` and `dt` must be in the same unit as your `vel` and `max_speed` — seconds and metres per second, or game-days and units per day, but not one of each. `neighbour_range`, `pos` and `radius` are in that same length unit. Left and right are the plane's own: a positive `CrowdParams::nudge` kicks clockwise in `(x, y)`, which is the walker's right when `+y` is up. A world whose ground plane is `(x, z)` may see that as its left, which is why the sign is yours to flip.

## Scope

Agent↔agent in `new_velocity_in`; static geometry as hard half-planes: either the `walls` you compute (a door that is shut this step: each a unit vector toward a solid plus the speed at which the agent may still close on it) or the ones [`Obstacles`] derives from polygons you built once. Neither is ever relaxed. The way round a building is not found here: it is your navigator's, and arrives as the preferred velocity. Final contact resolution is yours. So is where the work runs: the crate starts no thread, and [`Crowd::begin`] hands a step to whichever scheduler you have ([above](#across-threads)).

## Features

- `serde` — `Serialize`/`Deserialize` on `Agent`, `CrowdMember` and `CrowdParams`. Also turns on `bevy_math/serialize`, which glam's impls need.
- `reflect` — `bevy_reflect::Reflect` on the same types, for a host that stores them in a Bevy world.

Both are off by default, and neither adds a type to the dependency graph a consumer did not ask for.

## Determinism

Pure arithmetic, no RNG, no interior mutability. Identical inputs give bit-identical outputs, which is why the parent project can hash its simulation state. The thread count is not an input: [`Crowd::solve`] and the same step split into any ranges over any number of threads ([`CrowdStep::solve_range`]) give the same bits, and `bench` hashes both at every thread count to prove it. Allocation happens only while a buffer grows: after the first call, [`new_velocity_in`], [`Crowd::solve`] and a warm [`CrowdStep::solve_range`] allocate nothing, and `tests/alloc.rs` proves it with a counting allocator.

`bevy_math` is taken with `default-features = false, features = ["nostd-libm"]`, matching Bevy's own internal selection: the defaults would enable `glam/std` and swap glam's transcendentals from libm to std's, which is a behaviour change for anyone hashing transforms downstream.

## Examples

```sh
cargo run -p bevy_orca --example head_on        # terminal: reciprocity, each side takes half; or by right of way
cargo run -p bevy_orca --example wall_corridor  # terminal: walls hard, neighbours soft
cargo run -p bevy_orca --example street         # terminal: buildings as obstacles, walkers round the corners
cargo run -p bevy_orca --release --example bench  # terminal: what a crowd step costs, 100 to 10 000 agents
cargo run -p bevy_orca --example circle_swap    # the gif above. Needs a GPU.
```

`head_on`, `wall_corridor` and `street` print to the terminal — no window, no GPU, nothing to install. `head_on` walks two agents into each other and prints the lateral offset each takes, then repeats it with the neighbour holding ground so you can see the mover absorb the whole avoidance, and once more with one of them given priority 3 against 1: it sidesteps 0.10 m to the other's 0.82 m, where two equal agents sidestep 0.45 m each. `wall_corridor` adds wall half-planes and reports the deepest penetration, which stays at zero. `street` is the whole of the obstacle API in under 150 lines: four buildings, three streams of walkers crossing and turning the corners of the block, one `Crowd::solve_around` a step, the closest any disc came to a wall (never below zero) and how many arrived.

`bench` is the measuring instrument. It times one `Crowd::solve` step in ten scenes — an open field of walkers, the circle swap, a two-way corridor with walls, a room draining through a gap two agents wide, a rally of units hex-packed around one point with a 5 m neighbour range (about 200 candidates per member at 10 000 agents), that gap again with its wall as `Obstacles`, and a city of buildings with walkers on its streets, once with the buildings as `Obstacles`, once with them handled the way a host does without (nothing in the solve, and a walker whose disc ends a step in a building put back on its surface), and twice more as a grid of solid cells, through `add_cells` and as one `add_box` per cell, which print the obstacle `edges` and the `build_us` the builder took — at 100, 1 000, 5 000 and 10 000 agents, and prints one line per case: median and p99 step time, agents per millisecond, neighbour candidates per agent, the deepest overlap and wall penetration, allocations inside the solve, and a hash of the final state that must be equal whether the members are given forwards, reversed, or split over any number of `std::thread::scope` workers with `Crowd::begin` and `CrowdStep::solve_range`. `ORCA_SCENES=open,corridor`, `ORCA_SIZES=100,1000` and `ORCA_THREADS=2,4,8,14` narrow it.

`bench` also runs an `arrive` section: the circle swap (20 agents on a ring that jams, 20 on a wider one, 100 and 1 000), a two-way corridor, the bottleneck with its wall as half-planes and as `Obstacles`, and the city with and without its buildings in the solve, each for three times the straight-line time, printing the fraction that arrived, how many stalled for over two seconds, and when half and nine tenths had arrived (`t50_s`, `t90_s`). `ORCA_SEED=n` shifts the members' keys for another crowd of the same scene, `ORCA_SERIAL=1` times only the serial forward run, `ORCA_OBST_HORIZON=s` sets the obstacle horizon, and `ORCA_PRIORITY=p` gives the members heading right priority `p` and adds a line per case with each stream's own arrivals, stalls, delay and detour.

`circle_swap` is the picture. Two things in it are worth knowing before you tune this in your own game, because both were found by watching it fail:

- **Perfect symmetry stalls, and so does a ring packed shoulder to shoulder.** Identical agents, evenly spaced, all aimed at one point give the linear program no reason to prefer left over right. `Crowd`'s `nudge` is the deterministic tie-break, and it is one side for the whole crowd: a crowd that all leans the same way crosses as one vortex. At the defaults (`CrowdParams::new(3.0)`, 0.3 m radius, 1 m of ring per agent) `bench` finds a 100-agent swap with nobody stalled and 90% through in 35 s, and a 1 000-agent swap with none stalled in 266 s; the reference RVO2, set up with the same radius, speed, horizon and neighbours, takes 63 s and 380 s. Neither frees a ring that is already as tight as it can be: 20 agents of radius 0.3 on a 3.2 m ring converge and stop where their discs touch (a ring of 1.92 m), because moving in closes the gap to both neighbours and the velocity closest to a wish that points in is zero. That is a fixed point of the solver, not a tie, and the cure is room (the same 20 agents on 1.9 m of ring each arrive in 15 s) or a navigator that gives a stuck agent a new wish. `bench` keeps the tight ring's row so the limit stays measured.
- **Too much lookahead stalls it too, and less obviously.** At a 3-second horizon the pack braked for a congestion that had not happened yet and thereby caused it: velocity decayed geometrically — 0.80, 0.57, 0.41, 0.29, 0.21 — while the crossing ground to a halt a ring-radius in. At 1.5 they commit, and the shear resolves it. If your crowd is mysteriously slow, this dial is the first suspect.

## Used by

The rest of the family, so a host learns one shape:

- [`bevy_stigmergy`](https://crates.io/crates/bevy_stigmergy) — influence fields the same crowd deposits into and reads back.
- [`bevy_society`](https://crates.io/crates/bevy_society) — persons with daily lives whose movement is solved here.

## License

MIT OR Apache-2.0, at your option. Derived from RVO2 (Apache-2.0) — see [`NOTICE`](NOTICE).
