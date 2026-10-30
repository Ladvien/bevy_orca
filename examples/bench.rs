//! **The crowd bench: what one `Crowd::solve` step costs a host, and what it gets right.**
//!
//! Five scenes, each at 100, 1 000, 5 000 and 10 000 agents:
//!
//! - `open` — random walkers at 0.5 agents/m², each walking to goals drawn from its own key;
//! - `circle` — the circle swap, every agent walking to its antipode, 1 m of ring per agent;
//! - `corridor` — a 6 m wide two-way corridor with wall half-planes, the two directions interleaved;
//! - `bottleneck` — a room at 1 agent/m² draining through a gap two agents wide in a wall;
//! - `rally` — units gathered at a rally point: hex-packed 6 cm apart (2.65 agents/m²), every one
//!   pressing toward the centre, with a 5 m neighbour range, so each has about 200 members in range
//!   to choose its 10 neighbours from.
//!
//! Each case runs 20 warm-up steps and 480 timed ones, then prints one line:
//!
//! ```text
//! orca: <scene> agents=<n> threads=<t> order=<fwd|rev> median_us=<x> p99_us=<y> agents_per_ms=<a>
//!       cand_mean=<c> cand_max=<m> hash=<h> [overlap_max=<o> wall_max=<w> allocs=<k>]
//! ```
//!
//! - `median_us` / `p99_us` — one step's solve, the part a host pays every tick. Building the member
//!   slice and moving the agents are the host's own work and are not timed.
//! - `agents_per_ms` — agents solved per step (on average) divided by the median step in ms.
//! - `cand_mean` / `cand_max` — members offered to `sees` per solved agent per step: the in-range
//!   candidates the search finds and sorts, against at most `max_neighbours` (10) used.
//! - `hash` — every agent's position and velocity bits after all 500 steps, in key order. The
//!   `order=rev` line solves the member slice reversed; the `threads>1` lines solve the same step with
//!   `new_velocity_in` over member chunks on `std::thread::scope` workers, one `OrcaScratch` each,
//!   with the neighbour rule `Crowd` uses. All of a case's hashes must be equal.
//! - `overlap_max` — the deepest overlap between two agents on any step, as a fraction of the radius.
//!   `wall_max` — the deepest penetration past a wall, in metres. `allocs` — allocations made inside
//!   the timed `Crowd::solve` calls: a warm `Crowd` allocates only while a buffer grows to a wider
//!   call than it has seen, so a scene that densifies shows a handful, never one per step. These
//!   three are measured on the `threads=1 order=fwd` run.
//!
//! After each scene, `scaling_1k_10k` is the exponent of the median step from 1 000 to 10 000 agents
//! at constant density: 1.0 is linear.
//!
//! A second section, `arrive`, runs the circle swap at 20, 100 and 1 000 agents and a 2×50 two-way
//! corridor for three times the longest straight-line time, and prints the fraction that arrived and
//! how many stalled (under 5% of their preferred speed for more than 2 s, away from their goal).
//!
//! Knobs: `ORCA_SCENES=open,circle` and `ORCA_SIZES=100,1000` restrict the timing section;
//! `ORCA_ARRIVE=0` skips the arrival section, `ORCA_ARRIVE=only` runs nothing else.
//!
//! Run: `cargo run --release --example bench`

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::HashMap;
use std::time::Instant;

use bevy_math::Vec2;
use bevy_orca::{Agent, Crowd, CrowdMember, CrowdParams, OrcaScratch, new_velocity_in};

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

/// The SplitMix64 output mix — the same function `Crowd` hashes a key with to pick its nudge side.
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
}

impl Scene {
    const ALL: [Scene; 5] = [Scene::Open, Scene::Circle, Scene::Corridor, Scene::Bottleneck, Scene::Rally];

    fn name(self) -> &'static str {
        match self {
            Scene::Open => "open",
            Scene::Circle => "circle",
            Scene::Corridor => "corridor",
            Scene::Bottleneck => "bottleneck",
            Scene::Rally => "rally",
        }
    }

    /// The neighbour range the scene is solved with.
    fn range(self) -> f32 {
        if self == Scene::Rally { RALLY_RANGE } else { RANGE }
    }
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
}

impl World {
    fn new(scene: Scene, n: usize) -> Self {
        let mut pos = Vec::with_capacity(n);
        let mut goal = Vec::with_capacity(n);
        let keys: Vec<u64> = (0..n as u64).collect();
        let mut extent = 0.0;
        let mut segments = Vec::new();
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
                let ring = n as f32 / std::f32::consts::TAU;
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
            Scene::Bottleneck => {
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
        }
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
        }
    }

    /// Where agent `a` wants to go this step.
    fn pref(&self, a: usize) -> Vec2 {
        let p = self.pos[a];
        match self.scene {
            Scene::Open => (self.goal[a] - p).normalize_or_zero() * SPEED,
            Scene::Circle | Scene::Corridor | Scene::Rally => (self.goal[a] - p).clamp_length_max(SPEED),
            // Aim at the gap until through it, then straight on.
            Scene::Bottleneck => {
                if p.x < 0.0 {
                    (Vec2::new(0.5, 0.0) - p).normalize_or_zero() * SPEED
                } else {
                    Vec2::X * SPEED
                }
            }
        }
    }

    /// Agent `a`'s wall half-planes, `(unit direction toward the wall, max approach speed)`: the
    /// approach is the remaining clearance over `dt`, which cannot be crossed in one step.
    fn walls(&self, p: Vec2, buf: &mut Vec<(Vec2, f32)>) {
        match self.scene {
            Scene::Open | Scene::Circle | Scene::Rally => {}
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
            Scene::Bottleneck => {
                self.segments.iter().map(|s| (RADIUS - s.closest(p).distance(p)).max(0.0)).fold(0.0, f32::max)
            }
        }
    }

    /// Move every solved agent, then let arrivals leave and walkers pick their next goal.
    fn advance(&mut self, slots: &[usize], solved: &[Option<Vec2>]) {
        for (&a, v) in slots.iter().zip(solved) {
            if let Some(v) = v {
                self.vel[a] = *v;
                self.pos[a] += *v * DT;
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
                Scene::Bottleneck => self.gone[a] |= p.x > 3.0,
                // Nobody leaves a rally: the press on its centre is the scene.
                Scene::Rally => {}
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
// The caller's parallel ring, until the crate offers one: the same neighbour rule `Crowd` applies
// (members within range, sorted by distance then key, at most `max_neighbours`, nudged to the side
// hashed from the key), solved with `new_velocity_in` on scoped worker threads.

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

/// The order `Crowd` keeps candidates in: distance, then key, then index. The index makes it a strict
/// total order, so an unstable selection and sort give the one result a stable sort would.
fn nearest_first(a: &(f32, u64, usize), b: &(f32, u64, usize)) -> std::cmp::Ordering {
    a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2))
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

#[derive(Default)]
struct Worker {
    scratch: OrcaScratch,
    near: Vec<(f32, u64, usize)>,
    neighbours: Vec<Agent>,
    walls: Vec<(Vec2, f32)>,
    candidates: usize,
    candidates_max: usize,
}

fn solve_parallel(
    world: &World,
    members: &[CrowdMember],
    params: &CrowdParams,
    grid: &mut Grid,
    workers: &mut [Worker],
    out: &mut Vec<Option<Vec2>>,
) {
    let range = params.neighbour_range;
    grid.build(members.iter().map(|m| Some(m.pos)), range);
    out.clear();
    out.resize(members.len(), None);
    let chunk = members.len().div_ceil(workers.len()).max(1);
    let grid = &*grid;
    std::thread::scope(|scope| {
        for (c, (slice, w)) in out.chunks_mut(chunk).zip(workers.iter_mut()).enumerate() {
            scope.spawn(move || {
                for (k, slot) in slice.iter_mut().enumerate() {
                    let i = c * chunk + k;
                    let m = members[i];
                    let (Some(pref), true) = (m.pref_vel, m.pos.is_finite()) else {
                        continue;
                    };
                    if !pref.is_finite() {
                        continue;
                    }
                    w.near.clear();
                    grid.around(m.pos, range, |j| {
                        if j == i {
                            return;
                        }
                        let other = members[j];
                        let dist = other.pos.distance(m.pos);
                        if dist <= range {
                            w.near.push((dist, other.key, j));
                        }
                    });
                    w.candidates += w.near.len();
                    w.candidates_max = w.candidates_max.max(w.near.len());
                    let keep = params.max_neighbours;
                    if keep < w.near.len() {
                        w.near.select_nth_unstable_by(keep, nearest_first);
                        w.near.truncate(keep);
                    }
                    w.near.sort_unstable_by(nearest_first);
                    w.neighbours.clear();
                    for &(_, _, j) in &w.near {
                        let other = members[j];
                        w.neighbours.push(Agent {
                            pos: other.pos,
                            vel: other.vel,
                            radius: other.radius,
                            avoids: other.pref_vel.is_some(),
                        });
                    }
                    let wish = if w.near.is_empty() {
                        pref
                    } else {
                        let side = if splitmix64(m.key) & 1 == 0 { 1.0 } else { -1.0 };
                        pref + pref.perp() * (params.nudge * side)
                    };
                    w.walls.clear();
                    world.walls(m.pos, &mut w.walls);
                    let me = Agent { pos: m.pos, vel: m.vel, radius: m.radius, avoids: true };
                    *slot = Some(new_velocity_in(
                        &mut w.scratch,
                        &me,
                        wish,
                        &w.neighbours,
                        &w.walls,
                        params.time_horizon,
                        DT,
                        m.max_speed,
                    ));
                }
            });
        }
    });
}

// ---------------------------------------------------------------------------------------------
// Running a case.

struct Outcome {
    median_us: f64,
    p99_us: f64,
    agents_per_ms: f64,
    cand_mean: f64,
    cand_max: usize,
    hash: u64,
    overlap_max: f32,
    wall_max: f32,
    allocs: usize,
}

/// 20 warm-up and 480 timed steps of one scene. `threads == 0` is `Crowd::solve`; otherwise the
/// caller's parallel ring on that many workers. `measure` adds the overlap and wall checks.
fn run(scene: Scene, n: usize, threads: usize, reversed: bool, measure: bool) -> Outcome {
    let params = CrowdParams::new(scene.range());
    let mut world = World::new(scene, n);
    let mut crowd = Crowd::default();
    let mut members = Vec::new();
    let mut slots = Vec::new();
    let mut cand = Vec::new();
    let mut grid = Grid::default();
    let mut check_grid = Grid::default();
    let mut workers: Vec<Worker> = (0..threads).map(|_| Worker::default()).collect();
    let mut out = Vec::new();

    let mut times = Vec::with_capacity(TIMED);
    let (mut solved_total, mut cand_total, mut cand_max) = (0usize, 0usize, 0usize);
    let (mut overlap_max, mut wall_max, mut alloc_total) = (0.0f32, 0.0f32, 0usize);

    for step in 0..WARMUP + TIMED {
        world.members(reversed, &mut members, &mut slots);
        let timed = step >= WARMUP;
        if threads == 0 {
            cand.clear();
            cand.resize(members.len(), 0usize);
            let before = allocs();
            let start = Instant::now();
            let solved = crowd.solve(
                &members,
                &params,
                DT,
                |i, _| {
                    cand[i] += 1;
                    true
                },
                |i, buf| world.walls(members[i].pos, buf),
            );
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
            for w in workers.iter_mut() {
                w.candidates = 0;
                w.candidates_max = 0;
            }
            let start = Instant::now();
            solve_parallel(&world, &members, &params, &mut grid, &mut workers, &mut out);
            let elapsed = start.elapsed();
            if timed {
                times.push(elapsed.as_secs_f64() * 1e6);
                solved_total += out.iter().flatten().count();
                cand_total += workers.iter().map(|w| w.candidates).sum::<usize>();
                cand_max = cand_max.max(workers.iter().map(|w| w.candidates_max).max().unwrap_or(0));
            }
        }
        world.advance(&slots, &out);
        if measure {
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
        allocs: alloc_total,
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
        s += &format!(" overlap_max={:.4} wall_max={:.4} allocs={}", o.overlap_max, o.wall_max, o.allocs);
    }
    println!("{s}");
}

/// The arrival check: solve `scene` with `Crowd` for three times the longest straight-line time.
fn arrive(scene: Scene, n: usize) {
    let params = CrowdParams::new(RANGE);
    let mut world = World::new(scene, n);
    let longest = (0..n).map(|a| world.pos[a].distance(world.goal[a])).fold(0.0, f32::max);
    let steps = (3.0 * longest / SPEED / DT).ceil() as usize;
    let mut crowd = Crowd::default();
    let (mut members, mut slots, mut out) = (Vec::new(), Vec::new(), Vec::new());
    let mut check_grid = Grid::default();
    let (mut overlap_max, mut wall_max) = (0.0f32, 0.0f32);
    for _ in 0..steps {
        world.members(false, &mut members, &mut slots);
        if members.is_empty() {
            break;
        }
        let solved =
            crowd.solve(&members, &params, DT, |_, _| true, |i, buf| world.walls(members[i].pos, buf));
        out.clear();
        out.extend_from_slice(solved);
        world.advance(&slots, &out);
        overlap_max = overlap_max.max(world.deepest_overlap(&mut check_grid));
        for a in 0..n {
            if !world.gone[a] {
                wall_max = wall_max.max(world.wall_depth(a));
            }
        }
    }
    let arrived = world.gone.iter().filter(|g| **g).count();
    let stalled = world.stalled.iter().filter(|s| **s).count();
    println!(
        "orca: arrive {} agents={n} steps={steps} arrived={:.3} stalled={stalled} overlap_max={:.4} wall_max={:.4} \
         hash={:016x}",
        scene.name(),
        arrived as f64 / n as f64,
        overlap_max,
        wall_max,
        state_hash(&world.pos, &world.vel),
    );
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

    if arrive_mode != "only" {
        for &scene in &scenes {
            let mut medians = Vec::new();
            for &n in &sizes {
                let serial = run(scene, n, 0, false, true);
                line(scene, n, 1, false, &serial, true);
                medians.push((n, serial.median_us));
                let rev = run(scene, n, 0, true, false);
                line(scene, n, 1, true, &rev, false);
                let mut thread_counts = vec![2];
                if threads_default > 2 {
                    thread_counts.push(threads_default);
                }
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
        for n in [20, 100, 1_000] {
            arrive(Scene::Circle, n);
        }
        arrive(Scene::Corridor, 100);
    }
}
