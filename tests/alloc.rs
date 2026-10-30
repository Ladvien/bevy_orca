//! **Allocation-free solving, measured rather than asserted.**
//!
//! `new_velocity` is called for every agent every frame, and avoidable per-frame allocation is
//! forbidden in compiled code. The fix (issue #4) is a caller-owned [`OrcaScratch`]; this test is
//! what proves it: a counting global allocator, a warmed scratch, and 100 further solves that must
//! allocate nothing at all.
//!
//! The counter is **per thread** on purpose. `cargo test` runs the tests in a binary on parallel
//! threads, and a process-wide counter would then count another test's allocations and fail for a
//! reason that has nothing to do with this crate. The thread-local is `const`-initialised and holds a
//! plain `Cell`, so reading it from inside the allocator cannot itself allocate or register a
//! destructor.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use bevy_math::Vec2;
use bevy_orca::{Agent, OrcaScratch, new_velocity, new_velocity_in};

thread_local! {
    /// Every allocation request made on this thread since [`reset_counter`].
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

/// Allocation requests made on this thread since [`reset_counter`].
fn allocs() -> usize {
    ALLOCS.try_with(Cell::get).unwrap_or(0)
}

fn reset_counter() {
    let _ = ALLOCS.try_with(|c| c.set(0));
}

/// Counts allocation requests, then forwards to the system allocator. `try_with` rather than `with`
/// so a read during thread teardown cannot panic inside the allocator.
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

fn agent(pos: Vec2, vel: Vec2) -> Agent {
    Agent { pos, vel, radius: 0.5, avoids: true }
}

/// A deterministic `u32 -> [0,1)` hash (PCG-style output mix), copied by construction from the crate's
/// own unit tests so this sweep hits the same 40 configurations without an RNG dependency.
fn hash_f32(x: u32) -> f32 {
    let mut h = x.wrapping_mul(747_796_405).wrapping_add(2_891_336_453);
    h = ((h >> ((h >> 28).wrapping_add(4))) ^ h).wrapping_mul(277_803_737);
    h = (h >> 22) ^ h;
    (h as f32) / (u32::MAX as f32)
}

/// The six-neighbour ring the allocation test warms on: close enough to be real constraints, spread
/// far enough that the solver takes the interesting path rather than bailing out early.
fn six_neighbours() -> Vec<Agent> {
    (0..6)
        .map(|i| {
            let angle = (i as f32) * std::f32::consts::TAU / 6.0;
            agent(Vec2::new(angle.cos(), angle.sin()) * 1.4, Vec2::ZERO)
        })
        .collect()
}

#[test]
fn a_warmed_scratch_allocates_nothing() {
    let me = agent(Vec2::ZERO, Vec2::new(1.0, 0.0));
    let neighbours = six_neighbours();
    let walls = [(Vec2::new(0.0, 1.0), 0.3_f32)];
    let pref = Vec2::new(1.2, 0.1);
    let (horizon, dt, max_speed) = (2.0, 1.0 / 60.0, 1.5);

    let mut scratch = OrcaScratch::default();
    // Warm: the first call sizes both buffers, and every later call of this width must fit in them.
    let _ = new_velocity_in(&mut scratch, &me, pref, &neighbours, &walls, horizon, dt, max_speed);

    reset_counter();
    for _ in 0..100 {
        let v = new_velocity_in(&mut scratch, &me, pref, &neighbours, &walls, horizon, dt, max_speed);
        assert!(v.is_finite());
    }
    assert_eq!(allocs(), 0, "100 warmed solves with 6 neighbours made {} allocation(s)", allocs());

    // Fewer neighbours afterwards must not allocate either: the buffers only ever grow.
    reset_counter();
    for _ in 0..100 {
        let v = new_velocity_in(&mut scratch, &me, pref, &neighbours[..2], &walls, horizon, dt, max_speed);
        assert!(v.is_finite());
    }
    assert_eq!(allocs(), 0, "100 warmed solves with 2 neighbours made {} allocation(s)", allocs());
}

/// Reusing a buffer must be an allocation saving and nothing else: the two entry points have to agree
/// bit for bit, or a caller that switched to `new_velocity_in` would see different behaviour.
#[test]
fn reused_scratch_equals_the_allocating_path_bit_for_bit() {
    let max_speed = 2.0;
    let walls = [(Vec2::new(1.0, 0.0), 0.4_f32)];
    let mut scratch = OrcaScratch::default();
    for i in 0..40u32 {
        let hx = hash_f32(i.wrapping_mul(0x9E37_79B1)) * 4.0 - 2.0;
        let hy = hash_f32(i.wrapping_mul(0x85EB_CA6B).wrapping_add(1)) * 4.0 - 2.0;
        let me = agent(Vec2::ZERO, Vec2::new(1.0, 0.0));
        let them = agent(Vec2::new(hx, hy), Vec2::new(-0.5, 0.2));
        let pref = Vec2::new(1.5, 0.5);
        let plain = new_velocity(&me, pref, &[them], &walls, 2.0, 1.0 / 60.0, max_speed);
        let reused = new_velocity_in(&mut scratch, &me, pref, &[them], &walls, 2.0, 1.0 / 60.0, max_speed);
        assert_eq!(plain.x.to_bits(), reused.x.to_bits(), "config {i}: x differs");
        assert_eq!(plain.y.to_bits(), reused.y.to_bits(), "config {i}: y differs");
    }
}

/// The batch solver is the path a host with a crowd actually uses, so it carries the same promise: once
/// warm, a tick of sixty members with six neighbours each allocates nothing.
#[test]
fn a_warmed_crowd_allocates_nothing() {
    use bevy_orca::{Crowd, CrowdMember, CrowdParams};

    let params = CrowdParams::new(4.0);
    let dt = 1.0 / 60.0;
    let members: Vec<CrowdMember> = (0..60u32)
        .map(|i| {
            let angle = (i as f32) * std::f32::consts::TAU / 60.0;
            let radius = 1.0 + (i % 3) as f32 * 0.2;
            CrowdMember {
                key: u64::from(i),
                pos: Vec2::new(angle.cos(), angle.sin()) * radius,
                vel: Vec2::ZERO,
                radius: 0.28,
                max_speed: 1.5,
                pref_vel: Some(Vec2::new(angle.cos(), angle.sin())),
            }
        })
        .collect();

    let mut crowd = Crowd::default();
    let tick = |crowd: &mut Crowd| {
        let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());
        assert!(solved.iter().flatten().all(|v| v.is_finite()));
    };
    // Warm: the neighbour grid, the candidate list and the output slice all reach their working size.
    tick(&mut crowd);

    reset_counter();
    for _ in 0..20 {
        tick(&mut crowd);
    }
    assert_eq!(allocs(), 0, "20 warmed crowd solves allocated {} time(s)", allocs());
}

/// The same promise for a crowd that is going somewhere. Every tick here the whole group has moved two
/// cells on, so a neighbour search that keeps storage per cell visited would allocate on every one of
/// them and keep growing for as long as the crowd walks.
#[test]
fn a_warmed_crowd_that_walks_into_new_cells_allocates_nothing() {
    use bevy_orca::{Crowd, CrowdMember, CrowdParams};

    let params = CrowdParams::new(2.0);
    let dt = 1.0 / 60.0;
    let base: Vec<CrowdMember> = (0..60u32)
        .map(|i| CrowdMember {
            key: u64::from(i),
            pos: Vec2::new((i % 10) as f32, (i / 10) as f32) * 0.9,
            vel: Vec2::X,
            radius: 0.28,
            max_speed: 1.5,
            pref_vel: Some(Vec2::X),
        })
        .collect();
    let at_tick = |tick: u32, members: &mut Vec<CrowdMember>| {
        members.clear();
        let offset = Vec2::new(tick as f32 * 2.0 * params.neighbour_range, 0.0);
        members.extend(base.iter().map(|m| CrowdMember { pos: m.pos + offset, ..*m }));
    };

    let mut crowd = Crowd::default();
    let mut members = Vec::with_capacity(base.len());
    // Warm on the first tick's ground only.
    at_tick(0, &mut members);
    let _ = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());

    let mut made = 0;
    for tick in 1..=20 {
        at_tick(tick, &mut members);
        reset_counter();
        let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());
        made += allocs();
        assert!(solved.iter().flatten().all(|v| v.is_finite()));
    }
    assert_eq!(made, 0, "20 warmed solves of a crowd crossing new cells allocated {made} time(s)");
}

/// The same promise for a packed crowd (issue #7): 200 members 0.15 apart with a 4-unit range, so every
/// member has 199 candidates to choose its 10 neighbours from. The standard library's stable sort
/// takes a heap scratch above 170 such candidates, so a candidate list ordered with it allocated once
/// per member per tick — 1 000 times in these five ticks — exactly where units bunch up.
#[test]
fn a_warmed_dense_crowd_allocates_nothing() {
    use bevy_orca::{Crowd, CrowdMember, CrowdParams};

    let params = CrowdParams::new(4.0);
    let dt = 1.0 / 60.0;
    let members: Vec<CrowdMember> = (0..200u32)
        .map(|i| CrowdMember {
            key: u64::from(i),
            pos: Vec2::new((i % 20) as f32, (i / 20) as f32) * 0.15,
            vel: Vec2::ZERO,
            radius: 0.07,
            max_speed: 1.5,
            pref_vel: Some(Vec2::X),
        })
        .collect();
    let mut crowd = Crowd::default();
    let _ = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());

    reset_counter();
    for _ in 0..5 {
        let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());
        assert!(solved.iter().flatten().all(|v| v.is_finite()));
    }
    assert_eq!(allocs(), 0, "5 warmed solves of a dense crowd allocated {} time(s)", allocs());
}
