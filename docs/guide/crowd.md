# A whole crowd: Crowd, right of way, and threads

Moved from the README on 2026-10-31, unchanged, when the README was brought to the [org documentation guide](https://github.com/bevy-coven/.github/blob/main/DOCUMENTATION.md).

Solving one agent at a time leaves every host to write the same ring around it: a spatial hash to find neighbours without an O(n²) scan, a reused scratch buffer, a deterministic tie-break, the wall buffer. `Crowd` is that ring, once.

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

`Crowd` allocates nothing after its buffers are warm, wherever the crowd walks and however tightly it packs: its neighbour grid is one entry per member, sorted by cell and rebuilt every solve, so its memory and its cost follow how many members there are and not how much ground they have covered; and of the candidates in range only the nearest `max_neighbours` are kept, found and ordered in place, so a member with hundreds of others in range pays for its ten neighbours rather than for ordering the rest. `CrowdParams::new(range)` gives a 2-second horizon against other members, a 0.5-second one against [obstacles](obstacles.md), at most 10 neighbours and a 5% nudge to the right; set `nudge` to `0.0` to keep perfectly symmetric input perfectly symmetric, or negative to pass on the left.

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
| 2 | 2.33 s | 3.10 s | 2.71 s | 3.5 | 0.053 r |
| 4 | 2.45 s | 4.23 s | 3.34 s | 4.8 | 0.074 r |
| 8 | 2.15 s | 4.56 s | 3.36 s | 10.8 | 0.062 r |

A ratio of 2 gives the stream heading right 0.3 s and costs the others 0.3 s (the mean is the equal crowd's); beyond about 4 the stream that gives way pays more than the other saves, because the other is already near free flow, and the street stalls more (10.8 per 1 000 at 8 against 1.9), so use a modest ratio for a stream and a large one for a member the street should treat as nearly a wall. Discs that already overlap push apart evenly whatever their priorities: right of way is a claim on the way to avoid a collision, touching discs have none to give, and applying the share there left the other member of a touching pair to do all the separating (the 100-agent circle swap's deepest overlap was 0.209 of a radius at 8 against 0.040 at equal priority; it is 0.071 with the even split, 0.081 at 2). It does not free a walker boxed against a wall by a head-on one: with the lanes 0.2 m from the brick (`ORCA_LANE=1.0`) 19.3 of 1 000 stall at equal priority and 22 to 28 at 2 to 8. One standing villager (a wish of zero, so he still steps aside) with thirty walkers going past is pushed from his spot by up to 0.41 m at priority 1, 0.32 m at 2, 0.22 at 4, 0.14 at 8, 0.04 at 32 and not at all at `∞`, with the walkers' paths 0.13% longer than the straight line at every priority and no two discs touching (`a_stall_holder_with_priority_is_walked_around_by_a_stream`).

### Across threads

A step is one serial part and a lot of per-member work that reads a frozen world and writes only its own answer. `Crowd::begin` does the serial part (the neighbour grid: about 3% of a 10 000-walker step) and returns a `CrowdStep`; `CrowdStep::solve_range` does the rest for any run of members, into any slice of output, with a `CrowdWorker` (the per-thread buffers) of its own. The crate starts no thread, takes no lock and reads no thread count — it would put a second set of threads on the cores your engine already schedules — so you cut the ranges and run them where your game runs work, here on Bevy's task pool:

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

`out[k]` is member `first + k`, exactly what `solve` returns for it (`None` for a member that is not solved, and for a slot past the last member). `CrowdStep` is `Copy`, `Send` and `Sync` and a `CrowdWorker` is `Send`, so a task of `bevy_tasks`' `scope` (`Future + 'scope + Send`) can hold both with nothing `'static` (the `sees` and `walls` closures you give it live in the task too, so they must be `Send`); `tests/task_pool.rs` runs this shape on the real pool. `begin_around` is the same with `Obstacles`(obstacles.md).

**The split changes no velocity.** No member's answer reads another's and nothing is summed across members, so any partition of the members into ranges, solved in any order on any number of threads, gives every slot the bits `solve` gives (as long as your `sees` and `walls` are functions of the indices they are handed): the thread count is not an input to the simulation, and a replay recorded on four cores plays back on fourteen. Measured on a 14-core laptop that was also running other jobs (a pure compute loop scales to only about 3 of its cores here), a 10 000-walker open field took 28 ms serially and 9.4 ms on a 4-, 8- or 14-thread Bevy pool, 1 000 walkers 2.4 ms and 0.76 ms, with the pool's total CPU time 0.98 to 1.14 times the serial step's: the split costs no more work than it spreads. On a pool a small crowd loses nothing: at 256 members a chunk, a hundred walkers are one task and take what `solve` takes (165 against 167 µs), 300 walkers are 1.4× faster, 1 000 sparse walkers (0.7 ms serially) 2.5× faster. Starting a thread per step instead, as `bench`'s `std::thread::scope` rows do, costs 100–170 µs and loses below about a thousand walkers.
