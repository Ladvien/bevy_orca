//! **The split step on Bevy's own task pool.**
//!
//! `CrowdStep::solve_range` is meant to be handed to a game's job system, and Bevy's is
//! `TaskPool::scope`, whose `Scope::spawn` takes a `Future + 'scope + Send`: a task may borrow the step
//! and own a `&mut` worker and output slice, and nothing has to be `'static`. This test is that shape
//! against the real pool (`bevy` is a dev-dependency only, as for the examples), so a field that cannot
//! cross a thread, or a lifetime the pool cannot hold, fails here and not in a host's build.

use bevy::tasks::TaskPool;
use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams, CrowdWorker};

/// A block of walkers all heading east, so every one of them has neighbours to avoid.
fn block(n: usize) -> Vec<CrowdMember> {
    (0..n)
        .map(|i| CrowdMember {
            key: i as u64,
            pos: Vec2::new((i % 50) as f32, (i / 50) as f32) * 0.7,
            vel: Vec2::new(1.0, 0.0),
            radius: 0.3,
            max_speed: 1.3,
            pref_vel: Some(Vec2::X),
            priority: 1.0,
        })
        .collect()
}

#[test]
fn a_task_pool_solves_the_step_in_ranges_and_gives_the_serial_bits() {
    let members = block(1_500);
    let params = CrowdParams::new(3.0);
    let dt = 1.0 / 30.0;

    let mut serial = Crowd::default();
    let want = serial.solve(&members, &params, dt, |_, _| true, |_, _| {}).to_vec();
    assert!(want.iter().flatten().all(|v| v.is_finite()) && want.iter().flatten().count() == members.len());

    let pool = TaskPool::new();
    let mut crowd = Crowd::default();
    let mut workers: Vec<CrowdWorker> = (0..4).map(|_| CrowdWorker::default()).collect();
    let mut got = vec![None; members.len()];

    let step = crowd.begin(&members, &params, dt);
    // One range per worker, cut before the scope so each task owns its slice and its worker.
    let chunk = members.len().div_ceil(workers.len());
    let ranges: Vec<_> = got.chunks_mut(chunk).zip(workers.iter_mut()).enumerate().collect();
    pool.scope(|scope| {
        for (c, (slice, worker)) in ranges {
            scope.spawn(async move {
                step.solve_range(worker, c * chunk, slice, |_, _| true, |_, _| {});
            });
        }
    });

    for (i, (g, w)) in got.iter().zip(&want).enumerate() {
        let bits = |v: &Option<Vec2>| v.map(|v| (v.x.to_bits(), v.y.to_bits()));
        assert_eq!(bits(g), bits(w), "member {i}");
    }
}
