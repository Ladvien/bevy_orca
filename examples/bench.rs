//! **The crowd bench: what one `Crowd::solve` step costs a host, and what it gets right.**
//!
//! Ten scenes, each at 100, 1 000, 5 000 and 10 000 agents:
//!
//! - `open` — random walkers at 0.5 agents/m², each walking to goals drawn from its own key;
//! - `circle` — the circle swap, every agent walking to its antipode, 1 m of ring per agent;
//! - `corridor` — a 6 m wide two-way corridor with wall half-planes, the two directions interleaved;
//! - `bottleneck` — a room at 1 agent/m² draining through a gap two agents wide in a wall;
//! - `rally` — units gathered at a rally point: hex-packed 6 cm apart (2.65 agents/m²), every one
//!   pressing toward the centre, with a 5 m neighbour range, so at 10 000 agents a member has about
//!   200 others in range to choose its 10 neighbours from;
//! - `bottleneck_obst` — the bottleneck with its two wall pieces as `Obstacles` (RVO2's obstacle
//!   lines) instead of the host's closest-point half-planes, solved with `Crowd::solve_around`;
//! - `city` — walkers on Manhattan routes along the streets of a grid of 8 m buildings on a 12 m
//!   pitch (a quarter of a walker per m² of street), the buildings an `Obstacles`;
//! - `city_pushout` — the same city the way a host does it without obstacles: nothing in the solve,
//!   and a walker whose disc ends a step inside a building is put back on its surface (Sanctuary's
//!   `push_out_of_box`). It also prints `pushed` (agent-steps that needed it) and `push_max` (the
//!   deepest, in metres).
//! - `city_cells` — the same city given as a grid of 1 m solid cells (a building is an 8 × 8 square of
//!   them) and traced into outlines by `ObstaclesBuilder::add_cells`: the lines, and so the walkers, are
//!   `city`'s. It prints `edges` (the obstacle edges) and `build_us` (what the builder took);
//! - `city_boxes` — that same grid the way a host does it without `add_cells`: one `add_box` per solid
//!   cell, four edges for every cell however deep in a building. The same walkers again, so the step
//!   time and the edges are the price of feeding the grid in cell by cell.
//!
//! Each case runs 20 warm-up steps and 480 timed ones, then prints one line:
//!
//! ```text
//! orca: <scene> agents=<n> threads=<t> order=<fwd|rev> median_us=<x> p99_us=<y> agents_per_ms=<a>
//!       cand_mean=<c> cand_max=<m> hash=<h> [overlap_max=<o> wall_max=<w> speed_max=<s> allocs=<k>
//!       edges=<e> build_us=<b>]
//! ```
//!
//! - `median_us` / `p99_us` — one step's solve, the part a host pays every tick. Building the member
//!   slice and moving the agents are the host's own work and are not timed.
//! - `agents_per_ms` — agents solved per step (on average) divided by the median step in ms.
//! - `cand_mean` / `cand_max` — members offered to `sees` per solved agent per step: the in-range
//!   candidates the search finds and sorts, against at most `max_neighbours` (10) used.
//! - `hash` — every agent's position and velocity bits after all 500 steps, in key order. The
//!   `order=rev` line solves the member slice reversed; the `threads>1` lines solve the same step with
//!   `Crowd::begin` and `CrowdStep::solve_range`: the members cut into chunks, dealt to that many
//!   `std::thread::scope` workers, one `CrowdWorker` each (the grid is built once, serially, first).
//!   All of a case's hashes must be equal.
//! - `overlap_max` — the deepest overlap between two agents on any step, as a fraction of the radius.
//!   `wall_max` — the deepest penetration past a wall, in metres. `speed_max` — the largest solved
//!   speed on any step as a fraction of its member's `max_speed`: `1.0000` is the bound, and anything
//!   above `1.000001` is the linear program's answer leaving the speed disc. `allocs` — allocations
//!   made inside the timed `Crowd::solve` calls: a warm `Crowd` allocates only while a buffer grows to
//!   a wider call than it has seen, so a scene that densifies shows a handful, never one per step.
//!   These four are measured on the `threads=1 order=fwd` run. `edges` — the obstacle edges the scene's
//!   `Obstacles` has, and `build_us` the microseconds `ObstaclesBuilder` took to make them (the scenes
//!   that have obstacles).
//!
//! After each scene, `scaling_1k_10k` is the exponent of the median step from 1 000 to 10 000 agents
//! at constant density: 1.0 is linear.
//!
//! A second section, `arrive`, runs the circle swap at 20, 100 and 1 000 agents (1 m of ring per agent)
//! and at 20 agents on the `circle_swap` example's wider ring, a 100-agent two-way corridor, the
//! bottleneck and `bottleneck_obst` at 100, 200 and 400 agents, and the four cities (`city`,
//! `city_pushout`, `city_cells`, `city_boxes`) at 100 and 1 000 (each walker leaves at its goal), each
//! for three times the longest straight-line time.
//! It prints the fraction that arrived (in the bottleneck, that got through the gap), how many stalled
//! (under 5% of their preferred speed for more than 2 s, away from their goal), and `t50_s` / `t90_s`,
//! when half and nine tenths had arrived. The 20 agents on the 1 m ring never arrive: that ring is the
//! tightest their discs fit, a fixed point of the solver no sideways kick leaves (see the README's
//! known limits).
//!
//! Knobs: `ORCA_SCENES=open,circle` restricts both sections to those scenes and `ORCA_SIZES=100,1000`
//! the timing one; `ORCA_ARRIVE=0` skips the arrival section, `ORCA_ARRIVE=only` runs nothing else;
//! `ORCA_SEED=n` shifts every member's key by `n` (another crowd of the same scene, for a spread);
//! `ORCA_SERIAL=1` times only the serial forward run, which is all a timing comparison needs, and
//! leaves the order and thread-count hashes to a full run; `ORCA_THREADS=2,4,8,14` replaces the thread
//! counts of the `threads>1` lines (default 2 and the machine's parallelism); `ORCA_OBST_HORIZON=s` sets
//! `CrowdParams::obstacle_horizon` for the scenes that have obstacles; `ORCA_LANE=f` is the share of
//! a city street's width its walkers' four lanes spread over (default 0.6, outer lanes 0.8 m from the
//! brick; 1.0 puts them 0.2 m from it, which squeezes head-on pairs against the wall); `ORCA_PRIORITY=p`
//! gives the members heading right (the goal, or a city walker's first leg, is to the right of the
//! start) `CrowdMember::priority` `p` and everyone else 1, and prints a second line per `arrive` case,
//! `arrive_priority`, with each stream's own arrived fraction, stalls and mean delay beyond the
//! straight-line time (`ORCA_PRIORITY=1` is the control); unset, every priority is 1 and nothing changes.
//!
//! Run: `cargo run --release --example bench`

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::time::Instant;

use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams, CrowdWorker, Obstacles, ObstaclesBuilder};

const RADIUS: f32 = 0.3;
const SPEED: f32 = 1.3;
const DT: f32 = 1.0 / 30.0;
const RANGE: f32 = 3.0;
const WARMUP: usize = 20;
const TIMED: usize = 480;
const SIZES: [usize; 4] = [100, 1_000, 5_000, 10_000];
/// Within this of a circle-swap goal, an agent has arrived and leaves the scene.
const ARRIVE: f32 = 0.5;
const CORRIDOR_WIDTH: f32 = 6.0;
const GAP: f32 = 1.4;
/// The rally's neighbour range: wide enough that a packed member has hundreds of candidates.
const RALLY_RANGE: f32 = 5.0;
/// Centre-to-centre spacing of the rally's hex lattice: discs 6 cm apart.
const RALLY_SPACING: f32 = 2.0 * RADIUS + 0.06;
/// The city scenes: buildings `BLOCK` metres square on a `PITCH` lattice, so the streets between them
/// are `PITCH - BLOCK` wide, with `STREET_DENSITY` walkers per m² of street.
const BLOCK: f32 = 8.0;
const PITCH: f32 = 12.0;
const STREET_DENSITY: f32 = 0.25;
/// How far ahead along its street a city walker's wish aims.
const CARROT: f32 = 3.0;

// ---------------------------------------------------------------------------------------------
// Allocation counter: per thread, so a worker thread's spawn does not count against the solve.

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

struct Counting;

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn allocs() -> usize {
    ALLOCS.try_with(Cell::get).unwrap_or(0)
}

// ---------------------------------------------------------------------------------------------
// Deterministic hashing: the bench's own randomness is a function of an agent's key, never of a
// shared stream, so member order cannot change a goal.

/// `ORCA_SEED`, added to every member's key: a different crowd of the same scene, for a spread. Zero
/// unless set, so the default run is the one the hashes in the log were taken from.
fn seed() -> u64 {
    std::env::var("ORCA_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0)
}

/// The share of the city street's usable width the four lanes spread over (`ORCA_LANE`, `0.6` unless
/// set): the outer lanes then keep 0.8 m clear of the brick. At `1.0` they keep 0.2 m, and a walker in
/// such a lane meeting a head-on one has no room on its wall side to step aside.
fn lane_share() -> f32 {
    std::env::var("ORCA_LANE").ok().and_then(|s| s.parse().ok()).unwrap_or(0.6)
}

/// `ORCA_PRIORITY`, the `CrowdMember::priority` of the stream heading right (the others keep 1.0), or
/// `None` when it is not set. Setting it, even to `1`, also prints each stream's own arrival numbers
/// (`arrive_priority`), so `ORCA_PRIORITY=1` is the control the other values are read against.
fn high_priority() -> Option<f32> {
    std::env::var("ORCA_PRIORITY").ok().and_then(|s| s.parse().ok())
}

/// The SplitMix64 output mix: the scenes' own stable pseudo-random numbers, drawn from a member's key.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// A uniform `[0, 1)` from a key and a salt.
fn unit(key: u64, salt: u64) -> f32 {
    (splitmix64(key ^ splitmix64(salt)) >> 40) as f32 / (1u64 << 24) as f32
}

/// FNV-1a over every agent's position and velocity bits, in key order.
fn state_hash(pos: &[Vec2], vel: &[Vec2]) -> u64 {
    let mut h: u64 = 0xCBF2_9CE4_8422_2325;
    for (p, v) in pos.iter().zip(vel) {
        for bits in [p.x.to_bits(), p.y.to_bits(), v.x.to_bits(), v.y.to_bits()] {
            for byte in bits.to_le_bytes() {
                h ^= u64::from(byte);
                h = h.wrapping_mul(0x0000_0100_0000_01B3);
            }
        }
    }
    h
}

// ---------------------------------------------------------------------------------------------
// Scenes.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Scene {
    Open,
    Circle,
    Corridor,
    Bottleneck,
    Rally,
    /// The bottleneck's wall as `Obstacles` instead of the host's closest-point half-planes.
    BottleneckObst,
    /// Walkers on Manhattan routes through a grid of buildings, which are `Obstacles`.
    City,
    /// The same city the way the consumers do it today: nothing in the solve, and a push-out of any
    /// walker whose disc ends a step inside a building.
    CityPushout,
    /// The same city, its buildings given as a grid of 1 m solid cells and traced into outlines by
    /// `ObstaclesBuilder::add_cells`: the same lines as `city`, from the grid a tile-map host holds.
    CityCells,
    /// The same grid the way a host does it without `add_cells`: one `add_box` per solid cell.
    CityBoxes,
}

impl Scene {
    const ALL: [Scene; 10] = [
        Scene::Open,
        Scene::Circle,
        Scene::Corridor,
        Scene::Bottleneck,
        Scene::Rally,
        Scene::BottleneckObst,
        Scene::City,
        Scene::CityPushout,
        Scene::CityCells,
        Scene::CityBoxes,
    ];

    fn name(self) -> &'static str {
        match self {
            Scene::Open => "open",
            Scene::Circle => "circle",
            Scene::Corridor => "corridor",
            Scene::Bottleneck => "bottleneck",
            Scene::Rally => "rally",
            Scene::BottleneckObst => "bottleneck_obst",
            Scene::City => "city",
            Scene::CityPushout => "city_pushout",
            Scene::CityCells => "city_cells",
            Scene::CityBoxes => "city_boxes",
        }
    }

    /// The neighbour range the scene is solved with.
    fn range(self) -> f32 {
        if self == Scene::Rally { RALLY_RANGE } else { RANGE }
    }

    fn is_city(self) -> bool {
        matches!(self, Scene::City | Scene::CityPushout | Scene::CityCells | Scene::CityBoxes)
    }
}

/// `CrowdParams` for a scene. `ORCA_OBST_HORIZON` overrides `obstacle_horizon`, to measure it.
fn params_for(scene: Scene) -> CrowdParams {
    let mut params = CrowdParams::new(scene.range());
    if let Some(h) = std::env::var("ORCA_OBST_HORIZON").ok().and_then(|s| s.parse().ok()) {
        params.obstacle_horizon = h;
    }
    params
}

/// A convex wall piece: a segment from `a` to `b`.
#[derive(Clone, Copy)]
struct Segment {
    a: Vec2,
    b: Vec2,
}

impl Segment {
    fn closest(&self, p: Vec2) -> Vec2 {
        let ab = self.b - self.a;
        let t = ((p - self.a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
        self.a + ab * t
    }
}

/// The signed distance from `p` to the building of half extents `half` centred at `centre`: negative
/// inside.
fn box_distance(p: Vec2, centre: Vec2, half: Vec2) -> f32 {
    let d = (p - centre).abs() - half;
    d.max(Vec2::ZERO).length() + d.x.max(d.y).min(0.0)
}

/// `p` moved out of the building, to where a disc of `RADIUS` just clears it: Sanctuary's
/// `push_out_of_box`, the consumers' stand-in for an obstacle in the solve.
fn push_out_of_box(p: Vec2, centre: Vec2, half: Vec2) -> Vec2 {
    let d = (p - centre).abs() - half;
    if d.x < 0.0 && d.y < 0.0 {
        // Inside: out along the nearer face.
        let sign = Vec2::new((p.x - centre.x).signum(), (p.y - centre.y).signum());
        return if d.x > d.y {
            Vec2::new(centre.x + sign.x * (half.x + RADIUS), p.y)
        } else {
            Vec2::new(p.x, centre.y + sign.y * (half.y + RADIUS))
        };
    }
    let outside = d.max(Vec2::ZERO);
    let distance = outside.length();
    if distance >= RADIUS {
        return p;
    }
    let nearest = p - Vec2::new((p.x - centre.x).signum() * outside.x, (p.y - centre.y).signum() * outside.y);
    nearest + (p - nearest) / distance * RADIUS
}

/// The world a host owns: positions, velocities, goals. `Crowd` sees only the member slice built
/// from it each step.
#[derive(Clone)]
struct World {
    scene: Scene,
    keys: Vec<u64>,
    pos: Vec<Vec2>,
    vel: Vec<Vec2>,
    goal: Vec<Vec2>,
    /// Goals reached so far (the open field's walkers draw the next goal from it).
    legs: Vec<u64>,
    /// An agent that arrived has left the scene: it is neither solved nor anyone's neighbour.
    gone: Vec<bool>,
    /// Steps each agent has been slower than 5% of its preferred speed, in a row.
    slow_run: Vec<u32>,
    stalled: Vec<bool>,
    /// The open field's side, or the corridor's length.
    extent: f32,
    segments: Vec<Segment>,
    /// The static geometry the solve is given (the bottleneck's wall, the city's buildings); empty
    /// for the scenes that do not use `Obstacles`.
    obstacles: Obstacles,
    /// Seconds the scene's `ObstaclesBuilder` took to build `obstacles`, in microseconds.
    build_us: f64,
    /// The city: buildings per side, the first waypoint of each walker's route (the second is its
    /// goal), and which one it is walking to.
    blocks: usize,
    route: Vec<Vec2>,
    /// Each city walker's offset from the middle of the street it walks along.
    lateral: Vec<f32>,
    stage: Vec<u8>,
    /// A city walker leaves at its goal (the arrival section) instead of drawing another.
    trips: bool,
    /// The push-out city: agent-steps that ended inside a building, and the deepest of them.
    pushed: u64,
    push_max: f32,
    /// Each agent's `CrowdMember::priority`: `ORCA_PRIORITY` for the stream heading right (its route's
    /// or goal's x beyond its start), 1.0 for the rest, so a crowd that sets nothing is plain ORCA.
    priority: Vec<f32>,
    /// Whether the agent is in the stream `ORCA_PRIORITY` applies to.
    high: Vec<bool>,
}

impl World {
    fn new(scene: Scene, n: usize) -> Self {
        Self::with_ring(scene, n, 1.0)
    }

    /// `scene`, with the circle swap's ring `metres_per_agent` of circumference per agent (the other
    /// scenes ignore it).
    fn with_ring(scene: Scene, n: usize, metres_per_agent: f32) -> Self {
        let mut pos = Vec::with_capacity(n);
        let mut goal = Vec::with_capacity(n);
        let keys: Vec<u64> = (0..n as u64).map(|k| k + seed()).collect();
        let mut extent = 0.0;
        let mut segments = Vec::new();
        let (mut blocks, mut route, mut lateral) = (0usize, Vec::new(), Vec::new());
        let mut obstacles = Obstacles::default();
        let mut build_us = 0.0f64;
        match scene {
            Scene::Open => {
                // A jittered grid at 0.5 agents/m², so nobody starts inside anybody.
                let spacing = (1.0 / 0.5f32).sqrt();
                let side = (n as f32).sqrt().ceil() as usize;
                extent = spacing * side as f32;
                for (i, &k) in keys.iter().enumerate() {
                    let (cx, cy) = ((i % side) as f32, (i / side) as f32);
                    let jitter =
                        Vec2::new(unit(k, 1) - 0.5, unit(k, 2) - 0.5) * (spacing - 2.0 * RADIUS) * 0.8;
                    pos.push(Vec2::new(cx + 0.5, cy + 0.5) * spacing + jitter);
                    goal.push(Vec2::new(unit(k, 100), unit(k, 101)) * extent);
                }
            }
            Scene::Circle => {
                let ring = n as f32 * metres_per_agent / std::f32::consts::TAU;
                for i in 0..n {
                    let angle = i as f32 * std::f32::consts::TAU / n as f32;
                    let p = Vec2::new(angle.cos(), angle.sin()) * ring;
                    pos.push(p);
                    goal.push(-p);
                }
            }
            Scene::Corridor => {
                // Four rows across the 6 m, columns along x at 0.5 agents/m²; each agent's direction is
                // drawn from its key, so the two streams are interleaved rather than pre-laned.
                let rows = 4;
                let length = n as f32 / (0.5 * CORRIDOR_WIDTH);
                extent = length;
                let cols = n.div_ceil(rows);
                let dx = length / cols as f32;
                let dy = (CORRIDOR_WIDTH - 2.0 * RADIUS - 0.6) / (rows - 1) as f32;
                for (i, &k) in keys.iter().enumerate() {
                    let (c, r) = (i / rows, i % rows);
                    let x = -length * 0.5 + (c as f32 + 0.5) * dx + (unit(k, 3) - 0.5) * 0.2;
                    let y = -CORRIDOR_WIDTH * 0.5 + RADIUS + 0.3 + r as f32 * dy + (unit(k, 4) - 0.5) * 0.2;
                    pos.push(Vec2::new(x, y));
                    let east = unit(k, 5) < 0.5;
                    let end = length * 0.5 + 1.0;
                    goal.push(Vec2::new(if east { end } else { -end }, y));
                }
            }
            Scene::Bottleneck | Scene::BottleneckObst => {
                // A square room left of the wall at x = 0, one agent per m², on a jittered grid.
                let side = (n as f32).sqrt().ceil() as usize;
                let room = side as f32;
                for (i, &k) in keys.iter().enumerate() {
                    let (cx, cy) = ((i % side) as f32, (i / side) as f32);
                    let jitter = Vec2::new(unit(k, 1) - 0.5, unit(k, 2) - 0.5) * (1.0 - 2.0 * RADIUS) * 0.8;
                    pos.push(Vec2::new(-1.0 - room + cx + 0.5, -room * 0.5 + cy + 0.5) + jitter);
                    goal.push(Vec2::new(4.0, 0.0));
                }
                let far = room + 10.0;
                segments.push(Segment { a: Vec2::new(0.0, GAP * 0.5), b: Vec2::new(0.0, far) });
                segments.push(Segment { a: Vec2::new(0.0, -GAP * 0.5), b: Vec2::new(0.0, -far) });
                if scene == Scene::BottleneckObst {
                    let mut builder = ObstaclesBuilder::new();
                    for s in &segments {
                        builder.add_wall(s.a, s.b).expect("a wall");
                    }
                    obstacles = builder.build().expect("two walls");
                }
            }
            Scene::Rally => {
                // A hex lattice in a square block centred on the rally point at the origin.
                let side = (n as f32).sqrt().ceil() as usize;
                let row_step = RALLY_SPACING * 3f32.sqrt() * 0.5;
                let rows = n.div_ceil(side);
                let centre = Vec2::new(side as f32 * RALLY_SPACING, rows as f32 * row_step) * 0.5;
                for i in 0..n {
                    let (c, r) = (i % side, i / side);
                    let shift = if r % 2 == 1 { RALLY_SPACING * 0.5 } else { 0.0 };
                    pos.push(Vec2::new(c as f32 * RALLY_SPACING + shift, r as f32 * row_step) - centre);
                    goal.push(Vec2::ZERO);
                }
            }
            Scene::City | Scene::CityPushout | Scene::CityCells | Scene::CityBoxes => {
                // Buildings on a lattice, streets between them. Walkers start on the horizontal streets
                // in four lanes, and walk along the street middles: first along their own street to
                // their goal's column, then along that column to the goal, a street crossing.
                let street_share = 1.0 - (BLOCK / PITCH) * (BLOCK / PITCH);
                let k = (((n as f32 / STREET_DENSITY / street_share).sqrt() / PITCH).round() as usize).max(2);
                blocks = k;
                extent = k as f32 * PITCH;
                let streets = k + 1;
                let per_street = n.div_ceil(streets);
                let lanes = 4usize;
                let columns = per_street.div_ceil(lanes);
                let dx = extent / columns as f32;
                let usable = (PITCH - BLOCK - 2.0 * RADIUS - 0.4) * lane_share();
                for (i, &key) in keys.iter().enumerate() {
                    let (street, idx) = (i % streets, i / streets);
                    let (lane, column) = (idx % lanes, idx / lanes);
                    let y = street as f32 * PITCH + (lane as f32 - 1.5) * usable / 3.0;
                    let x = (column as f32 + 0.5) * dx + (unit(key, 3) - 0.5) * 0.2;
                    pos.push(Vec2::new(x, y));
                    let g = Vec2::new(
                        (unit(key, 100) * streets as f32).floor().min(k as f32) * PITCH,
                        (unit(key, 101) * streets as f32).floor().min(k as f32) * PITCH,
                    );
                    goal.push(g);
                    route.push(Vec2::new(g.x, street as f32 * PITCH));
                    lateral.push((lane as f32 - 1.5) * usable / 3.0);
                }
                let started = Instant::now();
                match scene {
                    Scene::City => {
                        let mut builder = ObstaclesBuilder::new();
                        for i in 0..k {
                            for j in 0..k {
                                let centre = Vec2::new((i as f32 + 0.5) * PITCH, (j as f32 + 0.5) * PITCH);
                                builder.add_box(centre, Vec2::splat(BLOCK * 0.5)).expect("a building");
                            }
                        }
                        obstacles = builder.build().expect("buildings");
                    }
                    Scene::CityCells | Scene::CityBoxes => {
                        // The city as a tile map: 1 m cells, a building the `BLOCK`-wide square of cells
                        // in the middle of each `PITCH` lattice cell (the same corners as `city`'s boxes).
                        let side = k * PITCH as usize;
                        let (offset, size) = ((PITCH - BLOCK) as usize / 2, BLOCK as usize);
                        let solid: Vec<bool> = (0..side * side)
                            .map(|i| {
                                let (x, y) = (i % side % PITCH as usize, i / side % PITCH as usize);
                                (offset..offset + size).contains(&x) && (offset..offset + size).contains(&y)
                            })
                            .collect();
                        let mut builder = ObstaclesBuilder::new();
                        if scene == Scene::CityCells {
                            builder.add_cells(Vec2::ZERO, 1.0, side, &solid).expect("a grid");
                        } else {
                            for i in (0..solid.len()).filter(|&i| solid[i]) {
                                let centre = Vec2::new((i % side) as f32 + 0.5, (i / side) as f32 + 0.5);
                                builder.add_box(centre, Vec2::splat(0.5)).expect("a cell");
                            }
                        }
                        obstacles = builder.build().expect("cells");
                    }
                    _ => {}
                }
                build_us = started.elapsed().as_secs_f64() * 1e6;
            }
        }
        // The stream heading right, when there is one: a city walker's first leg is along its street.
        let high: Vec<bool> =
            (0..n).map(|a| (if scene.is_city() { route[a] } else { goal[a] }).x > pos[a].x).collect();
        let hi = high_priority().unwrap_or(1.0);
        World {
            scene,
            vel: vec![Vec2::ZERO; n],
            legs: vec![0; n],
            gone: vec![false; n],
            slow_run: vec![0; n],
            stalled: vec![false; n],
            keys,
            pos,
            goal,
            extent,
            segments,
            obstacles,
            build_us,
            blocks,
            stage: vec![0; route.len()],
            route,
            lateral,
            trips: false,
            pushed: 0,
            push_max: 0.0,
            priority: high.iter().map(|&h| if h { hi } else { 1.0 }).collect(),
            high,
        }
    }

    /// The buildings within reach of `p`: those of the lattice cell it is in and the eight around it.
    fn buildings_near(&self, p: Vec2) -> impl Iterator<Item = Vec2> + use<> {
        let (i, j) = ((p.x / PITCH).floor() as i64, (p.y / PITCH).floor() as i64);
        let k = self.blocks as i64;
        (i - 1..=i + 1)
            .flat_map(move |i| (j - 1..=j + 1).map(move |j| (i, j)))
            .filter(move |(i, j)| (0..k).contains(i) && (0..k).contains(j))
            .map(|(i, j)| Vec2::new((i as f32 + 0.5) * PITCH, (j as f32 + 0.5) * PITCH))
    }

    /// How far a walker has to go from `a`'s start to its goal along its route (the straight line for
    /// the scenes that have no route).
    fn trip(&self, a: usize) -> f32 {
        if self.scene.is_city() {
            self.pos[a].distance(self.route[a]) + self.route[a].distance(self.goal[a])
        } else {
            self.pos[a].distance(self.goal[a])
        }
    }

    /// Where agent `a` wants to go this step.
    fn pref(&self, a: usize) -> Vec2 {
        let p = self.pos[a];
        match self.scene {
            Scene::Open => (self.goal[a] - p).normalize_or_zero() * SPEED,
            Scene::Circle | Scene::Corridor | Scene::Rally => (self.goal[a] - p).clamp_length_max(SPEED),
            // Aim at the gap until through it, then straight on.
            Scene::Bottleneck | Scene::BottleneckObst => {
                if p.x < 0.0 {
                    (Vec2::new(0.5, 0.0) - p).normalize_or_zero() * SPEED
                } else {
                    Vec2::X * SPEED
                }
            }
            Scene::City | Scene::CityPushout | Scene::CityCells | Scene::CityBoxes => {
                // A wish that follows the street, as a navigator's would: a carrot a few metres ahead
                // along the street the walker is on, at the walker's own lateral offset in it. (Aimed
                // at a waypoint a street away, a walker pushed off its line aims into a building.)
                let (route, goal, off) = (self.route[a], self.goal[a], self.lateral[a]);
                let carrot = if self.stage[a] == 0 {
                    Vec2::new(p.x + (route.x - p.x).clamp(-CARROT, CARROT), route.y + off)
                } else {
                    Vec2::new(goal.x + off, p.y + (goal.y - p.y).clamp(-CARROT, CARROT))
                };
                (carrot - p).normalize_or_zero() * SPEED
            }
        }
    }

    /// Agent `a`'s wall half-planes, `(unit direction toward the wall, max approach speed)`: the
    /// approach is the remaining clearance over `dt`, which cannot be crossed in one step.
    fn walls(&self, p: Vec2, buf: &mut Vec<(Vec2, f32)>) {
        match self.scene {
            Scene::Open
            | Scene::Circle
            | Scene::Rally
            | Scene::BottleneckObst
            | Scene::City
            | Scene::CityPushout
            | Scene::CityCells
            | Scene::CityBoxes => {}
            Scene::Corridor => {
                let half = CORRIDOR_WIDTH * 0.5;
                buf.push((Vec2::Y, (half - p.y - RADIUS).max(0.0) / DT));
                buf.push((-Vec2::Y, (half + p.y - RADIUS).max(0.0) / DT));
            }
            Scene::Bottleneck => {
                for s in &self.segments {
                    let to = s.closest(p) - p;
                    let d = to.length();
                    if d < RANGE {
                        buf.push((to.normalize_or_zero(), (d - RADIUS).max(0.0) / DT));
                    }
                }
            }
        }
    }

    /// How far agent `a`'s disc reaches past a wall, in metres (0 when clear).
    fn wall_depth(&self, a: usize) -> f32 {
        let p = self.pos[a];
        match self.scene {
            Scene::Open | Scene::Circle | Scene::Rally => 0.0,
            Scene::Corridor => (p.y.abs() + RADIUS - CORRIDOR_WIDTH * 0.5).max(0.0),
            Scene::Bottleneck | Scene::BottleneckObst => {
                self.segments.iter().map(|s| (RADIUS - s.closest(p).distance(p)).max(0.0)).fold(0.0, f32::max)
            }
            Scene::City | Scene::CityPushout | Scene::CityCells | Scene::CityBoxes => self
                .buildings_near(p)
                .map(|c| (RADIUS - box_distance(p, c, Vec2::splat(BLOCK * 0.5))).max(0.0))
                .fold(0.0, f32::max),
        }
    }

    /// Move every solved agent, then let arrivals leave and walkers pick their next goal.
    fn advance(&mut self, slots: &[usize], solved: &[Option<Vec2>]) {
        for (&a, v) in slots.iter().zip(solved) {
            if let Some(v) = v {
                self.vel[a] = *v;
                self.pos[a] += *v * DT;
                if self.scene == Scene::CityPushout {
                    // The host's own contact resolution: a walker whose disc ends the step in a
                    // building is put back on its surface, and the solve is none the wiser.
                    let before = self.pos[a];
                    let after = self
                        .buildings_near(before)
                        .fold(before, |p, c| push_out_of_box(p, c, Vec2::splat(BLOCK * 0.5)));
                    if after != before {
                        self.pushed += 1;
                        self.push_max = self.push_max.max(before.distance(after));
                        self.pos[a] = after;
                    }
                }
            }
        }
        for &a in slots {
            let p = self.pos[a];
            match self.scene {
                Scene::Open => {
                    if p.distance(self.goal[a]) < 1.0 {
                        self.legs[a] += 1;
                        let leg = self.legs[a] * 2 + 100;
                        self.goal[a] =
                            Vec2::new(unit(self.keys[a], leg), unit(self.keys[a], leg + 1)) * self.extent;
                    }
                }
                Scene::Circle => self.gone[a] |= p.distance(self.goal[a]) < ARRIVE,
                Scene::Corridor => self.gone[a] |= p.x.abs() > self.extent * 0.5 + 0.5,
                Scene::Bottleneck | Scene::BottleneckObst => self.gone[a] |= p.x > 3.0,
                // Nobody leaves a rally: the press on its centre is the scene.
                Scene::Rally => {}
                Scene::City | Scene::CityPushout | Scene::CityCells | Scene::CityBoxes => {
                    let (route, goal) = (self.route[a], self.goal[a]);
                    // Reached the column of the goal, or the goal itself.
                    let reached = if self.stage[a] == 0 {
                        (p.x - route.x).abs() < 1.0
                    } else {
                        (p.y - goal.y).abs() < 1.0 && (p.x - goal.x).abs() < 2.0
                    };
                    if reached {
                        if self.stage[a] == 0 {
                            self.stage[a] = 1;
                        } else if self.trips {
                            self.gone[a] = true;
                        } else {
                            // The next trip starts at the crossing just reached: along its street to
                            // the new goal's column, then along the column.
                            self.legs[a] += 1;
                            let leg = self.legs[a] * 2 + 200;
                            let streets = (self.blocks + 1) as f32;
                            let goal = Vec2::new(
                                (unit(self.keys[a], leg) * streets).floor().min(self.blocks as f32) * PITCH,
                                (unit(self.keys[a], leg + 1) * streets).floor().min(self.blocks as f32)
                                    * PITCH,
                            );
                            self.route[a] = Vec2::new(goal.x, self.goal[a].y);
                            self.goal[a] = goal;
                            self.stage[a] = 0;
                        }
                    }
                }
            }
            if !self.gone[a] {
                if self.vel[a].length() < 0.05 * SPEED {
                    self.slow_run[a] += 1;
                    if self.slow_run[a] as f32 * DT > 2.0 {
                        self.stalled[a] = true;
                    }
                } else {
                    self.slow_run[a] = 0;
                }
            }
        }
    }

    /// The member slice for this step, in the given order, and which agent each slot is.
    fn members(&self, reversed: bool, members: &mut Vec<CrowdMember>, slots: &mut Vec<usize>) {
        members.clear();
        slots.clear();
        let n = self.pos.len();
        for i in 0..n {
            let a = if reversed { n - 1 - i } else { i };
            if self.gone[a] {
                continue;
            }
            slots.push(a);
            members.push(CrowdMember {
                key: self.keys[a],
                pos: self.pos[a],
                vel: self.vel[a],
                radius: RADIUS,
                max_speed: SPEED,
                pref_vel: Some(self.pref(a)),
                priority: self.priority[a],
            });
        }
    }

    /// The deepest overlap between two agents still in the scene, as a fraction of the radius.
    fn deepest_overlap(&self, grid: &mut Grid) -> f32 {
        grid.build(self.pos.iter().zip(&self.gone).map(|(p, g)| if *g { None } else { Some(*p) }), RANGE);
        let mut worst = 0.0f32;
        for (i, p) in self.pos.iter().enumerate() {
            if self.gone[i] {
                continue;
            }
            grid.around(*p, RANGE, |j| {
                if j > i {
                    let overlap = 2.0 * RADIUS - p.distance(self.pos[j]);
                    worst = worst.max(overlap / RADIUS);
                }
            });
        }
        worst
    }
}

// ---------------------------------------------------------------------------------------------
// The overlap check's own grid, independent of `Crowd`'s so the check does not trust what it checks.

/// A uniform grid of member indices, cell = range.
#[derive(Default)]
struct Grid {
    cells: HashMap<(i32, i32), Vec<usize>>,
}

fn cell_of(pos: Vec2, range: f32) -> Option<(i32, i32)> {
    if !pos.is_finite() || range.is_nan() || range <= 0.0 {
        return None;
    }
    Some(((pos.x / range).floor() as i32, (pos.y / range).floor() as i32))
}

impl Grid {
    fn build(&mut self, points: impl Iterator<Item = Option<Vec2>>, range: f32) {
        for list in self.cells.values_mut() {
            list.clear();
        }
        for (i, p) in points.enumerate() {
            if let Some(cell) = p.and_then(|p| cell_of(p, range)) {
                self.cells.entry(cell).or_default().push(i);
            }
        }
    }

    fn around(&self, p: Vec2, range: f32, mut visit: impl FnMut(usize)) {
        let Some((cx, cy)) = cell_of(p, range) else {
            return;
        };
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(list) = self.cells.get(&(cx + dx, cy + dy)) {
                    list.iter().copied().for_each(&mut visit);
                }
            }
        }
    }
}

/// The step on `workers.len()` threads: `Crowd::begin` builds the grid, then the members are cut into
/// chunks, four or so to a worker so a dense corner does not pin one thread, and dealt to the workers in
/// turn; each worker solves its chunks with `CrowdStep::solve_range` into its slice of `out`. `cand`
/// counts the members `sees` was asked about, per member, as the serial run does. A Bevy host hands the
/// same chunks to its task pool instead of `std::thread::scope`.
fn solve_parallel(
    crowd: &mut Crowd,
    world: &World,
    members: &[CrowdMember],
    params: &CrowdParams,
    workers: &mut [CrowdWorker],
    cand: &mut Vec<usize>,
    out: &mut Vec<Option<Vec2>>,
) {
    cand.clear();
    cand.resize(members.len(), 0);
    out.clear();
    out.resize(members.len(), None);
    let step = if world.obstacles.is_empty() {
        crowd.begin(members, params, DT)
    } else {
        crowd.begin_around(&world.obstacles, members, params, DT)
    };
    let lanes = workers.len().max(1);
    let chunk = members.len().div_ceil(lanes * 4).max(16);
    type Chunk<'a> = (usize, &'a mut [Option<Vec2>], &'a mut [usize]);
    let mut dealt: Vec<Vec<Chunk>> = (0..lanes).map(|_| Vec::new()).collect();
    for (c, (slice, counts)) in out.chunks_mut(chunk).zip(cand.chunks_mut(chunk)).enumerate() {
        if let Some(lane) = dealt.get_mut(c % lanes) {
            lane.push((c * chunk, slice, counts));
        }
    }
    std::thread::scope(|scope| {
        for (worker, lane) in workers.iter_mut().zip(dealt) {
            scope.spawn(move || {
                for (first, slice, counts) in lane {
                    step.solve_range(
                        worker,
                        first,
                        slice,
                        |i, _| {
                            if let Some(count) = counts.get_mut(i.wrapping_sub(first)) {
                                *count += 1;
                            }
                            true
                        },
                        |i, buf| {
                            if let Some(m) = members.get(i) {
                                world.walls(m.pos, buf);
                            }
                        },
                    );
                }
            });
        }
    });
}

// ---------------------------------------------------------------------------------------------
// Running a case.

/// The largest `|v| / max_speed` among the solved velocities of one step (`0` if none is solved or a
/// member's `max_speed` is not positive). In `f64`, so a rounding-level excess over `1` is visible.
fn speed_ratio(members: &[CrowdMember], solved: &[Option<Vec2>]) -> f64 {
    members
        .iter()
        .zip(solved)
        .filter_map(|(m, v)| {
            let v = (*v)?;
            (m.max_speed > 0.0).then(|| f64::from(v.x).hypot(f64::from(v.y)) / f64::from(m.max_speed))
        })
        .fold(0.0, f64::max)
}

struct Outcome {
    median_us: f64,
    p99_us: f64,
    agents_per_ms: f64,
    cand_mean: f64,
    cand_max: usize,
    hash: u64,
    overlap_max: f32,
    wall_max: f32,
    speed_max: f64,
    allocs: usize,
    /// The push-out city: agent-steps that ended inside a building, and the deepest of them.
    pushed: u64,
    push_max: f32,
    /// The scene's obstacle edges, and the microseconds `ObstaclesBuilder` took to make them.
    edges: usize,
    build_us: f64,
}

/// 20 warm-up and 480 timed steps of one scene. `threads == 0` is `Crowd::solve`; otherwise the
/// caller's parallel ring on that many workers. `measure` adds the overlap and wall checks.
fn run(scene: Scene, n: usize, threads: usize, reversed: bool, measure: bool) -> Outcome {
    let params = params_for(scene);
    let mut world = World::new(scene, n);
    let mut crowd = Crowd::default();
    let mut members = Vec::new();
    let mut slots = Vec::new();
    let mut cand = Vec::new();
    let mut check_grid = Grid::default();
    let mut workers: Vec<CrowdWorker> = (0..threads).map(|_| CrowdWorker::default()).collect();
    let mut out = Vec::new();

    let mut times = Vec::with_capacity(TIMED);
    let (mut solved_total, mut cand_total, mut cand_max) = (0usize, 0usize, 0usize);
    let (mut overlap_max, mut wall_max, mut alloc_total) = (0.0f32, 0.0f32, 0usize);
    let mut speed_max = 0.0f64;

    for step in 0..WARMUP + TIMED {
        world.members(reversed, &mut members, &mut slots);
        let timed = step >= WARMUP;
        if threads == 0 {
            cand.clear();
            cand.resize(members.len(), 0usize);
            let before = allocs();
            let start = Instant::now();
            let solved = if world.obstacles.is_empty() {
                crowd.solve(
                    &members,
                    &params,
                    DT,
                    |i, _| {
                        cand[i] += 1;
                        true
                    },
                    |i, buf| world.walls(members[i].pos, buf),
                )
            } else {
                crowd.solve_around(
                    &world.obstacles,
                    &members,
                    &params,
                    DT,
                    |i, _| {
                        cand[i] += 1;
                        true
                    },
                    |i, buf| world.walls(members[i].pos, buf),
                )
            };
            let elapsed = start.elapsed();
            let made = allocs() - before;
            out.clear();
            out.extend_from_slice(solved);
            if timed {
                times.push(elapsed.as_secs_f64() * 1e6);
                alloc_total += made;
                solved_total += out.iter().flatten().count();
                cand_total += cand.iter().sum::<usize>();
                cand_max = cand_max.max(cand.iter().copied().max().unwrap_or(0));
            }
        } else {
            let start = Instant::now();
            solve_parallel(&mut crowd, &world, &members, &params, &mut workers, &mut cand, &mut out);
            let elapsed = start.elapsed();
            if timed {
                times.push(elapsed.as_secs_f64() * 1e6);
                solved_total += out.iter().flatten().count();
                cand_total += cand.iter().sum::<usize>();
                cand_max = cand_max.max(cand.iter().copied().max().unwrap_or(0));
            }
        }
        world.advance(&slots, &out);
        if measure {
            speed_max = speed_max.max(speed_ratio(&members, &out));
            overlap_max = overlap_max.max(world.deepest_overlap(&mut check_grid));
            for a in 0..n {
                if !world.gone[a] {
                    wall_max = wall_max.max(world.wall_depth(a));
                }
            }
        }
    }

    times.sort_by(f64::total_cmp);
    let median_us = times[times.len() / 2];
    let p99_us = times[(times.len() * 99).div_ceil(100) - 1];
    let mean_solved = solved_total as f64 / TIMED as f64;
    Outcome {
        median_us,
        p99_us,
        agents_per_ms: mean_solved / (median_us / 1000.0),
        cand_mean: if solved_total == 0 { 0.0 } else { cand_total as f64 / solved_total as f64 },
        cand_max,
        hash: state_hash(&world.pos, &world.vel),
        overlap_max,
        wall_max,
        speed_max,
        allocs: alloc_total,
        pushed: world.pushed,
        push_max: world.push_max,
        edges: world.obstacles.len(),
        build_us: world.build_us,
    }
}

fn line(scene: Scene, n: usize, threads: usize, reversed: bool, o: &Outcome, measured: bool) {
    let mut s = format!(
        "orca: {} agents={n} threads={threads} order={} median_us={:.1} p99_us={:.1} agents_per_ms={:.0} \
         cand_mean={:.1} cand_max={} hash={:016x}",
        scene.name(),
        if reversed { "rev" } else { "fwd" },
        o.median_us,
        o.p99_us,
        o.agents_per_ms,
        o.cand_mean,
        o.cand_max,
        o.hash,
    );
    if measured {
        s += &format!(
            " overlap_max={:.4} wall_max={:.4} speed_max={:.6} allocs={}",
            o.overlap_max, o.wall_max, o.speed_max, o.allocs
        );
    }
    if measured && scene == Scene::CityPushout {
        s += &format!(" pushed={} push_max={:.4}", o.pushed, o.push_max);
    }
    if measured && o.edges > 0 {
        s += &format!(" edges={} build_us={:.0}", o.edges, o.build_us);
    }
    println!("{s}");
}

/// The arrival check: solve `scene` with `Crowd` for three times the longest straight-line time.
/// `metres_per_agent` is the circle swap's ring per agent. `t50_s` and `t90_s` are when half and nine
/// tenths of the agents had arrived, or `never`: a crowd that gets through slowly and one that gets
/// through fast both read `arrived=1.000` otherwise, and in the bottleneck `arrived` is the flow
/// through the gap within the budget.
fn arrive(scene: Scene, n: usize, metres_per_agent: f32) {
    let params = params_for(scene);
    let mut world = World::with_ring(scene, n, metres_per_agent);
    world.trips = true;
    let straight: Vec<f32> = (0..n).map(|a| world.trip(a)).collect();
    let longest = straight.iter().copied().fold(0.0, f32::max);
    let steps = (3.0 * longest / SPEED / DT).ceil() as usize;
    // The step each agent arrived on (0 = not yet), for the per-stream numbers `ORCA_PRIORITY` asks for.
    let mut arrived_on = vec![0usize; n];
    let mut crowd = Crowd::default();
    let (mut members, mut slots, mut out) = (Vec::new(), Vec::new(), Vec::new());
    let mut check_grid = Grid::default();
    let (mut overlap_max, mut wall_max) = (0.0f32, 0.0f32);
    let mut speed_max = 0.0f64;
    let (mut t50, mut t90) = (None, None);
    for step in 0..steps {
        world.members(false, &mut members, &mut slots);
        if members.is_empty() {
            break;
        }
        let solved = if world.obstacles.is_empty() {
            crowd.solve(&members, &params, DT, |_, _| true, |i, buf| world.walls(members[i].pos, buf))
        } else {
            crowd.solve_around(
                &world.obstacles,
                &members,
                &params,
                DT,
                |_, _| true,
                |i, buf| world.walls(members[i].pos, buf),
            )
        };
        out.clear();
        out.extend_from_slice(solved);
        speed_max = speed_max.max(speed_ratio(&members, &out));
        world.advance(&slots, &out);
        for (a, on) in arrived_on.iter_mut().enumerate() {
            if *on == 0 && world.gone[a] {
                *on = step + 1;
            }
        }
        overlap_max = overlap_max.max(world.deepest_overlap(&mut check_grid));
        for a in 0..n {
            if !world.gone[a] {
                wall_max = wall_max.max(world.wall_depth(a));
            }
        }
        let through = world.gone.iter().filter(|g| **g).count();
        if t50.is_none() && through * 2 >= n {
            t50 = Some((step + 1) as f32 * DT);
        }
        if t90.is_none() && through * 10 >= n * 9 {
            t90 = Some((step + 1) as f32 * DT);
        }
    }
    let arrived = world.gone.iter().filter(|g| **g).count();
    let stalled = world.stalled.iter().filter(|s| **s).count();
    let seconds = |t: Option<f32>| t.map_or_else(|| "never".to_string(), |t| format!("{t:.1}"));
    let (t50, t90) = (seconds(t50), seconds(t90));
    let ring = if scene == Scene::Circle && metres_per_agent != 1.0 {
        format!(" ring_m_per_agent={metres_per_agent}")
    } else {
        String::new()
    };
    let pushed = if scene == Scene::CityPushout {
        format!(" pushed={} push_max={:.4}", world.pushed, world.push_max)
    } else {
        String::new()
    };
    println!(
        "orca: arrive {} agents={n} steps={steps} arrived={:.3} stalled={stalled} overlap_max={:.4} wall_max={:.4} \
         speed_max={speed_max:.6} hash={:016x} t50_s={t50} t90_s={t90}{ring}{pushed}",
        scene.name(),
        arrived as f64 / n as f64,
        overlap_max,
        wall_max,
        state_hash(&world.pos, &world.vel),
    );
    if let Some(priority) = high_priority() {
        // Each stream on its own: how many of it arrived, stalled, and how long the arrivals took beyond
        // the straight-line time (`delay_s`, the mean over those that arrived).
        let stream = |wanted: bool| {
            let members: Vec<usize> = (0..n).filter(|&a| world.high[a] == wanted).collect();
            let done: Vec<usize> = members.iter().copied().filter(|&a| arrived_on[a] != 0).collect();
            let delay =
                done.iter().map(|&a| f64::from(arrived_on[a] as f32 * DT - straight[a] / SPEED)).sum::<f64>()
                    / done.len().max(1) as f64;
            let stalled = members.iter().filter(|&&a| world.stalled[a]).count();
            (members.len(), done.len() as f64 / members.len().max(1) as f64, stalled, delay)
        };
        let (high, high_arrived, high_stalled, high_delay) = stream(true);
        let (low, low_arrived, low_stalled, low_delay) = stream(false);
        println!(
            "orca: arrive_priority {} agents={n} priority={priority} high={high} low={low} \
             arrived_high={high_arrived:.3} arrived_low={low_arrived:.3} stalled_high={high_stalled} \
             stalled_low={low_stalled} delay_high_s={high_delay:.3} delay_low_s={low_delay:.3}",
            scene.name(),
        );
    }
}

fn main() {
    let scenes: Vec<Scene> = match std::env::var("ORCA_SCENES") {
        Ok(list) => {
            Scene::ALL.into_iter().filter(|s| list.split(',').any(|w| w.trim() == s.name())).collect()
        }
        Err(_) => Scene::ALL.to_vec(),
    };
    let sizes: Vec<usize> = match std::env::var("ORCA_SIZES") {
        Ok(list) => list.split(',').filter_map(|w| w.trim().parse().ok()).collect(),
        Err(_) => SIZES.to_vec(),
    };
    let arrive_mode = std::env::var("ORCA_ARRIVE").unwrap_or_default();
    let threads_default = std::thread::available_parallelism().map_or(1, |n| n.get());
    let serial_only = std::env::var("ORCA_SERIAL").is_ok();

    if arrive_mode != "only" {
        for &scene in &scenes {
            let mut medians = Vec::new();
            for &n in &sizes {
                let serial = run(scene, n, 0, false, true);
                line(scene, n, 1, false, &serial, true);
                medians.push((n, serial.median_us));
                if serial_only {
                    continue;
                }
                let rev = run(scene, n, 0, true, false);
                line(scene, n, 1, true, &rev, false);
                let thread_counts: Vec<usize> = match std::env::var("ORCA_THREADS") {
                    Ok(list) => {
                        list.split(',').filter_map(|w| w.trim().parse().ok()).filter(|&t| t > 0).collect()
                    }
                    Err(_) if threads_default > 2 => vec![2, threads_default],
                    Err(_) => vec![2],
                };
                for t in thread_counts {
                    let par = run(scene, n, t, false, false);
                    line(scene, n, t, false, &par, false);
                }
            }
            let at = |n: usize| medians.iter().find(|(m, _)| *m == n).map(|(_, t)| *t);
            if let (Some(t1), Some(t10)) = (at(1_000), at(10_000)) {
                println!("orca: {} scaling_1k_10k={:.3}", scene.name(), (t10 / t1).ln() / 10f64.ln());
            }
        }
    }

    if arrive_mode != "0" {
        let wanted = |scene: Scene| scenes.contains(&scene);
        if wanted(Scene::Circle) {
            for n in [20, 100, 1_000] {
                arrive(Scene::Circle, n, 1.0);
            }
            // The same 20 agents on a ring the `circle_swap` example's size (6 m of radius, 1.9 m of
            // ring each), which is not the tight ring of the first row: see the README's known limits.
            arrive(Scene::Circle, 20, 6.0 * std::f32::consts::TAU / 20.0);
        }
        if wanted(Scene::Corridor) {
            arrive(Scene::Corridor, 100, 1.0);
        }
        if wanted(Scene::Bottleneck) {
            for n in [100, 200, 400] {
                arrive(Scene::Bottleneck, n, 1.0);
            }
        }
        if wanted(Scene::BottleneckObst) {
            for n in [100, 200, 400] {
                arrive(Scene::BottleneckObst, n, 1.0);
            }
        }
        for scene in [Scene::City, Scene::CityPushout, Scene::CityCells, Scene::CityBoxes] {
            if wanted(scene) {
                for n in [100, 1_000] {
                    arrive(scene, n, 1.0);
                }
            }
        }
    }
}
