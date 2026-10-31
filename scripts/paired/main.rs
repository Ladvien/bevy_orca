//! **A paired timing of `Crowd::solve` in two builds of this crate, for a host whose cores are shared.**
//!
//! `examples/bench.rs` times one build, once, and on a laptop with fast and slow cores or a loop host
//! under load the same binary reads 2x apart from one round to the next, so it cannot say whether a
//! change cost 3%. This harness can. `scripts/paired.sh` builds four copies of the crate under four
//! package names (`orca_a1` and `orca_a2` from a base commit, `orca_b1` and `orca_b2` from the working
//! tree), and this program solves the *same frozen crowd* with each of them, one after another and in a
//! rotating order, many times, reading the thread's own CPU time. Per round it forms the ratio
//! `(b1 + b2) / (a1 + a2)`; the median over all rounds is the answer. The two A copies are the same
//! code, and so are the two B copies: `a2 / a1` and `b2 / b1` are the floor of the measurement, what two
//! copies of identical code differ by from code layout alone, and a ratio inside that floor is not a
//! regression.
//!
//! The crowds are frozen states (positions, velocities, wishes) from the generators below, so a round
//! is a pure function of the code under test. Only the calls the base and the working tree share are
//! used (`Crowd::solve` with `CrowdParams::new`), so any two commits can be compared.

use std::time::Duration;

use bevy_math::Vec2;

/// One frozen member, in the form every copy of the crate takes.
#[derive(Clone, Copy)]
struct Frozen {
    key: u64,
    pos: Vec2,
    vel: Vec2,
    wish: Vec2,
}

/// A scene: its members, the neighbour range it is solved with, and the corridor's half width when it
/// has walls (`0` for none).
struct Scene {
    name: &'static str,
    members: Vec<Frozen>,
    range: f32,
    corridor_half: f32,
}

const RADIUS: f32 = 0.3;
const SPEED: f32 = 1.3;
const DT: f32 = 1.0 / 30.0;

fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn unit(key: u64, salt: u64) -> f32 {
    (splitmix64(key ^ splitmix64(salt)) >> 40) as f32 / (1u64 << 24) as f32
}

/// Walkers on a jittered grid at 0.5 per m², moving, each wishing toward a random goal.
fn open(n: usize) -> Scene {
    let side = (n as f32).sqrt().ceil() as usize;
    let spacing = 2.0f32.sqrt();
    let members = (0..n)
        .map(|i| {
            let key = i as u64;
            let jitter = Vec2::new(unit(key, 1) - 0.5, unit(key, 2) - 0.5) * (spacing - 2.0 * RADIUS) * 0.8;
            let pos = Vec2::new((i % side) as f32 + 0.5, (i / side) as f32 + 0.5) * spacing + jitter;
            let goal = Vec2::new(unit(key, 100), unit(key, 101)) * spacing * side as f32;
            let angle = unit(key, 3) * std::f32::consts::TAU;
            Frozen {
                key,
                pos,
                vel: Vec2::new(angle.cos(), angle.sin()) * SPEED * unit(key, 4),
                wish: (goal - pos).normalize_or_zero() * SPEED,
            }
        })
        .collect();
    Scene { name: "open", members, range: 3.0, corridor_half: 0.0 }
}

/// The circle swap's first step: a ring at 1 m of circumference per agent, every one wishing toward
/// its antipode, a third of them already moving that way.
fn circle(n: usize) -> Scene {
    let ring = n as f32 / std::f32::consts::TAU;
    let members = (0..n)
        .map(|i| {
            let angle = i as f32 * std::f32::consts::TAU / n as f32;
            let pos = Vec2::new(angle.cos(), angle.sin()) * ring;
            let wish = (-pos).clamp_length_max(SPEED);
            Frozen { key: i as u64, pos, vel: if i % 3 == 0 { wish } else { Vec2::ZERO }, wish }
        })
        .collect();
    Scene { name: "circle", members, range: 3.0, corridor_half: 0.0 }
}

/// A 6 m wide two-way corridor with wall half-planes, the two directions interleaved.
fn corridor(n: usize) -> Scene {
    let length = n as f32 / 3.0;
    let members = (0..n)
        .map(|i| {
            let key = i as u64;
            let (c, r) = (i / 4, i % 4);
            let x = -length * 0.5 + (c as f32 + 0.5) * length / n.div_ceil(4) as f32;
            let y = -3.0 + RADIUS + 0.3 + r as f32 * (6.0 - 2.0 * RADIUS - 0.6) / 3.0;
            let east = unit(key, 5) < 0.5;
            let dir = if east { Vec2::X } else { -Vec2::X };
            Frozen { key, pos: Vec2::new(x, y), vel: dir * SPEED * 0.8, wish: dir * SPEED }
        })
        .collect();
    Scene { name: "corridor", members, range: 3.0, corridor_half: 3.0 }
}

/// Units hex-packed 6 cm apart, every one pressing toward the centre, with a 5 m range.
fn rally(n: usize) -> Scene {
    let spacing = 2.0 * RADIUS + 0.06;
    let side = (n as f32).sqrt().ceil() as usize;
    let row = spacing * 3f32.sqrt() * 0.5;
    let rows = n.div_ceil(side);
    let centre = Vec2::new(side as f32 * spacing, rows as f32 * row) * 0.5;
    let members = (0..n)
        .map(|i| {
            let (c, r) = (i % side, i / side);
            let shift = if r % 2 == 1 { spacing * 0.5 } else { 0.0 };
            let pos = Vec2::new(c as f32 * spacing + shift, r as f32 * row) - centre;
            Frozen { key: i as u64, pos, vel: Vec2::ZERO, wish: (-pos).clamp_length_max(SPEED) }
        })
        .collect();
    Scene { name: "rally", members, range: 5.0, corridor_half: 0.0 }
}

/// This thread's CPU time, which a busy neighbour on another core cannot lengthen.
fn thread_cpu() -> Duration {
    #[repr(C)]
    struct Timespec {
        seconds: i64,
        nanoseconds: i64,
    }
    unsafe extern "C" {
        fn clock_gettime(clock: i32, out: *mut Timespec) -> i32;
    }
    const CLOCK_THREAD_CPUTIME_ID: i32 = if cfg!(target_os = "macos") { 16 } else { 3 };
    let mut t = Timespec { seconds: 0, nanoseconds: 0 };
    // SAFETY: `t` is a valid `timespec` for the call to fill in.
    let status = unsafe { clock_gettime(CLOCK_THREAD_CPUTIME_ID, &mut t) };
    if status != 0 {
        return Duration::ZERO;
    }
    Duration::new(t.seconds.max(0) as u64, t.nanoseconds.clamp(0, 999_999_999) as u32)
}

/// One module per copy of the crate: its own `Crowd`, its own member type, the scene converted once.
macro_rules! copy {
    ($module:ident, $krate:ident) => {
        mod $module {
            use super::{DT, Frozen, RADIUS, SPEED, Scene, thread_cpu};
            use bevy_math::Vec2;
            use $krate::{Crowd, CrowdMember, CrowdParams};

            pub struct Case {
                members: Vec<CrowdMember>,
                params: CrowdParams,
                crowd: Crowd,
                corridor_half: f32,
            }

            impl Case {
                pub fn new(scene: &Scene) -> Case {
                    let members = scene
                        .members
                        .iter()
                        .map(|f: &Frozen| CrowdMember {
                            key: f.key,
                            pos: f.pos,
                            vel: f.vel,
                            radius: RADIUS,
                            max_speed: SPEED,
                            pref_vel: Some(f.wish),
                        })
                        .collect();
                    let mut case = Case {
                        members,
                        params: CrowdParams::new(scene.range),
                        crowd: Crowd::default(),
                        corridor_half: scene.corridor_half,
                    };
                    // Warm the buffers: the first solve sizes them.
                    case.solve();
                    case
                }

                /// One `Crowd::solve`, in thread CPU seconds, with a checksum of its answer so that the
                /// work cannot be optimised away.
                pub fn solve(&mut self) -> (f64, f32) {
                    let half = self.corridor_half;
                    let members = &self.members;
                    let start = thread_cpu();
                    let solved = self.crowd.solve(
                        members,
                        &self.params,
                        DT,
                        |_, _| true,
                        |i, buf: &mut Vec<(Vec2, f32)>| {
                            if half > 0.0 {
                                let y = members[i].pos.y;
                                buf.push((Vec2::Y, (half - y - RADIUS).max(0.0) / DT));
                                buf.push((-Vec2::Y, (half + y - RADIUS).max(0.0) / DT));
                            }
                        },
                    );
                    let sum: f32 = solved.iter().flatten().map(|v| v.x + v.y).sum();
                    ((thread_cpu() - start).as_secs_f64(), sum)
                }
            }
        }
    };
}

copy!(a1, orca_a1);
copy!(a2, orca_a2);
copy!(b1, orca_b1);
copy!(b2, orca_b2);

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn main() {
    let reps: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(400);
    let only = std::env::var("PAIRED_SCENES").ok();
    let sizes: Vec<usize> = std::env::var("PAIRED_SIZES")
        .ok()
        .map(|s| s.split(',').filter_map(|w| w.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![100, 1_000, 10_000]);
    println!("paired: {reps} rounds per case; ratio = median of per-round (b1 + b2) / (a1 + a2); floors a2/a1 and b2/b1");
    for make in [open as fn(usize) -> Scene, circle, corridor, rally] {
        for &n in &sizes {
            let scene = make(n);
            if only.as_ref().is_some_and(|o| !o.split(',').any(|w| w == scene.name)) {
                continue;
            }
            let (mut a1c, mut a2c, mut b1c, mut b2c) =
                (a1::Case::new(&scene), a2::Case::new(&scene), b1::Case::new(&scene), b2::Case::new(&scene));
            let (mut ratio, mut floor_a, mut floor_b, mut times_a, mut times_b) =
                (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
            let mut sink = 0.0f32;
            for round in 0..reps {
                let mut t = [0.0f64; 4];
                // Rotating order: each copy takes every position in the sequence equally often.
                for k in 0..4 {
                    let (seconds, sum) = match (round + k) % 4 {
                        0 => a1c.solve(),
                        1 => b1c.solve(),
                        2 => a2c.solve(),
                        _ => b2c.solve(),
                    };
                    t[(round + k) % 4] = seconds;
                    sink += sum;
                }
                let (a1t, b1t, a2t, b2t) = (t[0], t[1], t[2], t[3]);
                ratio.push((b1t + b2t) / (a1t + a2t));
                floor_a.push(a2t / a1t);
                floor_b.push(b2t / b1t);
                times_a.push((a1t + a2t) / 2.0);
                times_b.push((b1t + b2t) / 2.0);
            }
            let pct = |r: f64| (r - 1.0) * 100.0;
            println!(
                "paired: {:9} agents={n:6} base_us={:9.1} new_us={:9.1}  new/base {:+5.1}%   floor a2/a1 {:+4.1}%  b2/b1 {:+4.1}%   (checksum {sink:.1})",
                scene.name,
                median(times_a) * 1e6,
                median(times_b) * 1e6,
                pct(median(ratio)),
                pct(median(floor_a)),
                pct(median(floor_b)),
            );
        }
    }
}
