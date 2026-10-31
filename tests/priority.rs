//! Right of way: [`Agent::priority`] and [`CrowdMember::priority`], what a caller sees.
//!
//! Of the dodge a collision course asks for, a member takes `other / (this + other)` and the other
//! takes the rest (`docs/research/priority.md`). These tests hold the model to its three promises:
//! equal priorities are plain ORCA to the bit, the pair's total dodge does not change with who gives
//! it (so the collision-free guarantee survives an unequal split), and `Crowd` reads the same numbers a
//! caller of `new_velocity_in` hands in.

use bevy_math::Vec2;
use bevy_orca::{Agent, Crowd, CrowdMember, CrowdParams, OrcaScratch, new_velocity, new_velocity_in};

const DT: f32 = 1.0 / 60.0;
const HORIZON: f32 = 2.0;

/// SplitMix64: this file's own stable pseudo-random numbers.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }

    fn vec(&mut self, lo: f32, hi: f32) -> Vec2 {
        Vec2::new(self.range(lo, hi), self.range(lo, hi))
    }

    /// A priority: usually log-uniform over `1e-3 .. 1e3`, sometimes zero or infinite.
    fn priority(&mut self) -> f32 {
        match self.next_u64() % 10 {
            0 => 0.0,
            1 => f32::INFINITY,
            _ => 10f32.powf(self.range(-3.0, 3.0)),
        }
    }
}

fn bits(v: Vec2) -> (u32, u32) {
    (v.x.to_bits(), v.y.to_bits())
}

/// How far each of two head-on members changes its velocity on the first step, when both ask to keep
/// walking: `(change of a, change of b)`. With nothing else in the way the answer is the wish moved to
/// the member's own half-plane, so the change is exactly that member's share of the dodge.
fn head_on(priority_a: f32, priority_b: f32) -> (Vec2, Vec2) {
    let a =
        Agent { pos: Vec2::new(-2.0, 0.1), vel: Vec2::X, radius: 0.5, avoids: true, priority: priority_a };
    let b =
        Agent { pos: Vec2::new(2.0, -0.1), vel: -Vec2::X, radius: 0.5, avoids: true, priority: priority_b };
    let va = new_velocity(&a, a.vel, &[b], &[], HORIZON, DT, 10.0);
    let vb = new_velocity(&b, b.vel, &[a], &[], HORIZON, DT, 10.0);
    (va - a.vel, vb - b.vel)
}

/// Two members that both step aside split the dodge in the ratio of their priorities, and the pair as
/// a whole dodges exactly as much as two equal members do.
#[test]
fn a_head_on_pair_splits_the_dodge_by_priority() {
    let (equal_a, equal_b) = head_on(1.0, 1.0);
    let whole = equal_a - equal_b;
    assert!(whole.length() > 0.1, "the control must be a real dodge: {whole}");
    assert!((equal_a.length() - equal_b.length()).abs() < 1e-4, "equal priorities take equal halves");

    for (a, b) in [(3.0, 1.0), (1.0, 3.0), (8.0, 1.0), (1.0, 8.0), (2.0, 5.0), (1e-3, 1e3), (40.0, 40.0)] {
        let (change_a, change_b) = head_on(a, b);
        assert!(
            ((change_a - change_b) - whole).length() < 1e-4 * whole.length(),
            "priorities {a}, {b}: the pair dodges {} where two equal members dodge {}",
            (change_a - change_b).length(),
            whole.length()
        );
        // `a` takes `b / (a + b)` of it and `b` takes `a / (a + b)`: a quarter and three quarters at 3 to 1.
        let want_a = whole.length() * b / (a + b);
        let want_b = whole.length() * a / (a + b);
        assert!(
            (change_a.length() - want_a).abs() < 1e-3 * whole.length(),
            "priorities {a}, {b}: a {change_a}"
        );
        assert!(
            (change_b.length() - want_b).abs() < 1e-3 * whole.length(),
            "priorities {a}, {b}: b {change_b}"
        );
        if a != b {
            assert_eq!(
                a > b,
                change_a.length() < change_b.length(),
                "the higher priority takes the smaller share"
            );
        }
    }
}

/// The ends of the scale: priority `0` takes all of the dodge and `∞` takes none of it, against anything
/// above `0` and below `∞`; a NaN or negative priority is a `0`.
#[test]
fn zero_takes_everything_and_infinity_nothing() {
    let (equal_a, equal_b) = head_on(1.0, 1.0);
    let whole = (equal_a - equal_b).length();
    for (a, b, a_takes_all) in [
        (0.0, 1.0, true),
        (f32::NAN, 1.0, true),
        (-4.0, 1.0, true),
        (f32::NEG_INFINITY, 2.0, true),
        (1.0, f32::INFINITY, true),
        (1.0, 0.0, false),
        (1.0, f32::NAN, false),
        (f32::INFINITY, 3.0, false),
    ] {
        let (change_a, change_b) = head_on(a, b);
        let (taker, giver) = if a_takes_all { (change_a, change_b) } else { (change_b, change_a) };
        assert!(
            (taker.length() - whole).abs() < 1e-4 * whole,
            "priorities {a}, {b}: {taker} is not the whole dodge"
        );
        assert!(giver.length() < 1e-5, "priorities {a}, {b}: {giver} should take none of it");
    }
}

/// A random scene around `me`: up to ten neighbours, some of which do not step aside, and sometimes a wall.
fn scene(rng: &mut Rng) -> (Agent, Vec2, Vec<Agent>, Vec<(Vec2, f32)>) {
    let me = Agent {
        pos: rng.vec(-1.0, 1.0),
        vel: rng.vec(-1.5, 1.5),
        radius: rng.range(0.2, 0.6),
        avoids: true,
        priority: 1.0,
    };
    let neighbours = (0..(rng.next_u64() % 11))
        .map(|_| Agent {
            pos: me.pos + rng.vec(-3.0, 3.0),
            vel: rng.vec(-1.5, 1.5),
            radius: rng.range(0.2, 0.6),
            avoids: rng.unit() < 0.7,
            priority: 1.0,
        })
        .collect();
    let angle = rng.range(0.0, std::f32::consts::TAU);
    let walls =
        if rng.unit() < 0.3 { vec![(Vec2::from_angle(angle), rng.range(0.0, 2.0))] } else { Vec::new() };
    (me, rng.vec(-1.5, 1.5), neighbours, walls)
}

fn solve(me: &Agent, pref: Vec2, neighbours: &[Agent], walls: &[(Vec2, f32)]) -> Vec2 {
    new_velocity_in(&mut OrcaScratch::default(), me, pref, neighbours, walls, HORIZON, DT, 1.8)
}

/// Whatever value every member holds, if all hold the same one the solve is the one a caller who never
/// heard of priorities gets, bit for bit: the existing callers' output does not move. A priority that
/// reads as `0` (NaN, negative) is equal to another that does.
#[test]
fn equal_priorities_are_plain_orca_at_any_value() {
    let mut rng = Rng(11);
    for case in 0..2_000 {
        let (me, pref, neighbours, walls) = scene(&mut rng);
        let plain = solve(&me, pref, &neighbours, &walls);
        for value in [0.0, 1e-30, 0.3, 3.7, 1e30, f32::MAX, f32::INFINITY, f32::NAN, -2.0, f32::NEG_INFINITY]
        {
            let me = Agent { priority: value, ..me };
            let neighbours: Vec<Agent> = neighbours.iter().map(|n| Agent { priority: value, ..*n }).collect();
            let got = solve(&me, pref, &neighbours, &walls);
            assert_eq!(bits(got), bits(plain), "case {case}, everyone at priority {value}: {got} vs {plain}");
        }
        // NaN here, negative there: both are `0`, so both are equal.
        let me = Agent { priority: f32::NAN, ..me };
        let neighbours: Vec<Agent> = neighbours.iter().map(|n| Agent { priority: -7.0, ..*n }).collect();
        let got = solve(&me, pref, &neighbours, &walls);
        assert_eq!(bits(got), bits(plain), "case {case}, NaN against negative: {got} vs {plain}");
    }
}

/// A NaN or negative priority — the agent's or a neighbour's — counts as `0`: the answer is the one for
/// `0`, finite, and no neighbour is skipped for it (a skipped one would be walked through).
#[test]
fn an_unusable_priority_counts_as_zero() {
    let mut rng = Rng(23);
    for case in 0..2_000 {
        let (me, pref, neighbours, walls) = scene(&mut rng);
        let others: Vec<f32> = neighbours.iter().map(|_| rng.range(0.1, 5.0)).collect();
        let with = |own: f32, others: &[f32]| {
            let me = Agent { priority: own, ..me };
            let neighbours: Vec<Agent> =
                neighbours.iter().zip(others).map(|(n, &p)| Agent { priority: p, ..*n }).collect();
            solve(&me, pref, &neighbours, &walls)
        };
        let zero = with(0.0, &others);
        for own in [f32::NAN, -1.0, -0.0, f32::NEG_INFINITY, -f32::MAX] {
            let got = with(own, &others);
            assert!(got.is_finite());
            assert_eq!(bits(got), bits(zero), "case {case}, own priority {own}");
        }
        let zeros = vec![0.0; others.len()];
        for unusable in [f32::NAN, -1.0, f32::NEG_INFINITY] {
            let got = with(1.0, &vec![unusable; others.len()]);
            assert!(got.is_finite());
            assert_eq!(
                bits(got),
                bits(with(1.0, &zeros)),
                "case {case}, every neighbour's priority {unusable}"
            );
        }
    }
}

/// A neighbour whose priority outranks everything (`∞`), or an agent whose own is `0`, takes the same
/// avoidance as one against a neighbour that holds ground (`avoids: false`): the full dodge.
#[test]
fn zero_against_one_is_the_full_avoidance_a_neighbour_holding_ground_gets() {
    let mut rng = Rng(37);
    for case in 0..2_000 {
        let (me, pref, neighbours, walls) = scene(&mut rng);
        // Every neighbour steps aside in the first form and holds ground in the second.
        let held: Vec<Agent> = neighbours.iter().map(|n| Agent { avoids: false, ..*n }).collect();
        let want = solve(&me, pref, &held, &walls);
        let giving_way = Agent { priority: 0.0, ..me };
        let steps_aside: Vec<Agent> =
            neighbours.iter().map(|n| Agent { avoids: true, priority: 1.0, ..*n }).collect();
        let got = solve(&giving_way, pref, &steps_aside, &walls);
        assert_eq!(bits(got), bits(want), "case {case}: priority 0 against 1");
        let outranked: Vec<Agent> =
            neighbours.iter().map(|n| Agent { avoids: true, priority: f32::INFINITY, ..*n }).collect();
        let got = solve(&me, pref, &outranked, &walls);
        assert_eq!(bits(got), bits(want), "case {case}: against priority infinity");
    }
}

/// The closest two discs come over `[0, horizon]` if they keep to these velocities.
fn closest_approach(pos_a: Vec2, vel_a: Vec2, pos_b: Vec2, vel_b: Vec2, horizon: f32) -> f64 {
    let p = (pos_b - pos_a).as_dvec2();
    let w = (vel_b - vel_a).as_dvec2();
    let t = if w.length_squared() > 0.0 {
        (-p.dot(w) / w.length_squared()).clamp(0.0, f64::from(horizon))
    } else {
        0.0
    };
    (p + w * t).length()
}

/// The ORCA guarantee, under any split of the dodge whose two shares add to 1: when both members take
/// a velocity inside their own half-plane, the pair is collision-free over the horizon. Tested the way
/// the paper states it, on two members with nothing else to satisfy and a speed limit that never binds,
/// for random positions, velocities, wishes and horizons and random priorities (zero and infinite
/// included); the same sweep with equal priorities is the control.
#[test]
fn an_unequal_split_keeps_the_pair_collision_free() {
    let mut rng = Rng(53);
    let (mut worst_unequal, mut worst_equal) = (f64::INFINITY, f64::INFINITY);
    let mut avoided = 0;
    for case in 0..30_000 {
        let radius_a = rng.range(0.2, 0.6);
        let radius_b = rng.range(0.2, 0.6);
        let reach = radius_a + radius_b;
        let pos_a = rng.vec(-3.0, 3.0);
        // Not touching: the guarantee is about discs that start apart.
        let pos_b =
            pos_a + Vec2::from_angle(rng.range(0.0, std::f32::consts::TAU)) * rng.range(reach + 0.05, 5.0);
        let (vel_a, vel_b) = (rng.vec(-1.5, 1.5), rng.vec(-1.5, 1.5));
        let (pref_a, pref_b) = (rng.vec(-1.5, 1.5), rng.vec(-1.5, 1.5));
        let horizon = rng.range(0.5, 3.0);
        let unequal = (rng.priority(), rng.priority());
        for (priority_a, priority_b, worst) in
            [(unequal.0, unequal.1, &mut worst_unequal), (1.0, 1.0, &mut worst_equal)]
        {
            let a = Agent { pos: pos_a, vel: vel_a, radius: radius_a, avoids: true, priority: priority_a };
            let b = Agent { pos: pos_b, vel: vel_b, radius: radius_b, avoids: true, priority: priority_b };
            let va = new_velocity(&a, pref_a, &[b], &[], horizon, DT, 100.0);
            let vb = new_velocity(&b, pref_b, &[a], &[], horizon, DT, 100.0);
            assert!(va.is_finite() && vb.is_finite(), "case {case}");
            let gap = closest_approach(pos_a, va, pos_b, vb, horizon) - f64::from(reach);
            *worst = worst.min(gap);
            assert!(
                gap > -2e-3,
                "case {case}: priorities {priority_a}, {priority_b} left the discs {gap} inside each other \
                 (a {a:?} -> {va}, b {b:?} -> {vb}, horizon {horizon})"
            );
            if priority_a == 1.0 && priority_b == 1.0 && (va - pref_a).length() > 1e-3 {
                avoided += 1;
            }
        }
    }
    assert!(avoided > 3_000, "the sweep must make members dodge, or it proves nothing: {avoided}");
    println!("worst gap past touching: unequal {worst_unequal:.5}, equal {worst_equal:.5}");
}

/// `Crowd` hands the solver the priorities a caller of `new_velocity_in` would: each member's own and
/// every neighbour's, so a pair solved by `Crowd::solve` equals the pair solved by hand, bit for bit.
#[test]
fn crowd_reads_priority_for_the_member_and_for_its_neighbours() {
    let params = CrowdParams { nudge: 0.0, ..CrowdParams::new(10.0) };
    let mut crowd = Crowd::default();
    for (priority_a, priority_b) in
        [(1.0, 1.0), (3.0, 1.0), (1.0, 3.0), (0.0, 1.0), (1.0, f32::INFINITY), (f32::NAN, 2.0)]
    {
        let member = |key: u64, pos: Vec2, vel: Vec2, priority: f32| CrowdMember {
            key,
            pos,
            vel,
            radius: 0.5,
            max_speed: 10.0,
            pref_vel: Some(vel),
            priority,
        };
        let members = [
            member(1, Vec2::new(-2.0, 0.1), Vec2::X, priority_a),
            // (-1, +0), not `-Vec2::X`: Crowd's zero nudge turns a wish's -0.0 into +0.0, which is a
            // different bit pattern for the same number.
            member(2, Vec2::new(2.0, -0.1), Vec2::new(-1.0, 0.0), priority_b),
        ];
        let solved = crowd.solve(&members, &params, DT, |_, _| true, |_, buf| buf.clear()).to_vec();
        let (want_a, want_b) = {
            let [a, b] = members.map(|m| Agent {
                pos: m.pos,
                vel: m.vel,
                radius: m.radius,
                avoids: true,
                priority: m.priority,
            });
            (
                new_velocity(&a, a.vel, &[b], &[], params.time_horizon, DT, 10.0),
                new_velocity(&b, b.vel, &[a], &[], params.time_horizon, DT, 10.0),
            )
        };
        assert_eq!(
            solved[0].map(bits),
            Some(bits(want_a)),
            "priorities {priority_a}, {priority_b}: member a"
        );
        assert_eq!(
            solved[1].map(bits),
            Some(bits(want_b)),
            "priorities {priority_a}, {priority_b}: member b"
        );
    }
}

/// A villager standing in the street (a wish of zero, so it is still a member that steps aside) is
/// shoved aside by a walker coming through it by the share the walker leaves it. At priority 8 against
/// the walker's 1 that is `1/9` where equal members split it `1/2`: `2/9` as far.
#[test]
fn a_standing_member_with_priority_is_pushed_aside_by_the_share_it_is_owed() {
    let params = CrowdParams { nudge: 0.0, ..CrowdParams::new(10.0) };
    let mut crowd = Crowd::default();
    let mut shoved = |standing_priority: f32| {
        let standing = CrowdMember {
            key: 1,
            pos: Vec2::new(0.0, 0.05),
            vel: Vec2::ZERO,
            radius: 0.3,
            max_speed: 1.5,
            pref_vel: Some(Vec2::ZERO),
            priority: standing_priority,
        };
        let walker = CrowdMember {
            key: 2,
            pos: Vec2::new(-2.0, 0.0),
            vel: Vec2::new(1.3, 0.0),
            radius: 0.3,
            max_speed: 1.5,
            pref_vel: Some(Vec2::new(1.3, 0.0)),
            priority: 1.0,
        };
        let solved =
            crowd.solve(&[standing, walker], &params, DT, |_, _| true, |_, buf| buf.clear()).to_vec();
        solved[0].map_or(f32::NAN, |v| v.length())
    };
    let equal = shoved(1.0);
    let held = shoved(8.0);
    assert!(equal > 0.05, "the control must be shoved: {equal}");
    assert!((held / equal - 2.0 / 9.0).abs() < 1e-3, "shoved {held} against {equal}: ratio {}", held / equal);
}

/// One stall-holder standing in a street (a wish of zero: he still steps aside) while thirty walkers go
/// past him one after another, each two metres behind the last and within 0.8 m of his side of the
/// street. Over three crowds, returns how far he was pushed from his spot at the worst (the mean of each
/// crowd's peak), the walkers' mean extra path as a percentage of the straight line, and the deepest two
/// discs ever overlapped, as a fraction of a radius.
fn stall_in_a_stream(priority: f32) -> (f32, f32, f32) {
    const WALKERS: usize = 30;
    const CROWDS: u64 = 3;
    let params = CrowdParams::new(3.0);
    let (mut pushed, mut extra, mut overlap) = (0.0f32, 0.0f32, 0.0f32);
    for seed in 1..=CROWDS {
        let mut rng = Rng(seed);
        let member = |key: u64, pos: Vec2, wish: Vec2, priority: f32| CrowdMember {
            key,
            pos,
            vel: wish,
            radius: 0.3,
            max_speed: 1.3,
            pref_vel: Some(wish),
            priority,
        };
        let mut members = vec![member(0, Vec2::ZERO, Vec2::ZERO, priority)];
        for k in 0..WALKERS {
            let start = Vec2::new(-8.0 - 2.0 * k as f32, rng.range(-0.8, 0.8));
            members.push(member(1 + k as u64, start, Vec2::new(1.3, 0.0), 1.0));
        }
        let starts: Vec<Vec2> = members.iter().map(|m| m.pos).collect();
        let mut walked = vec![0.0f32; members.len()];
        let mut crowd = Crowd::default();
        let mut peak = 0.0f32;
        for _ in 0..1_800 {
            let solved =
                crowd.solve(&members, &params, 1.0 / 30.0, |_, _| true, |_, buf| buf.clear()).to_vec();
            for (i, v) in solved.iter().enumerate() {
                let Some(v) = v else { continue };
                members[i].vel = *v;
                members[i].pos += *v / 30.0;
                walked[i] += v.length() / 30.0;
            }
            peak = peak.max(members[0].pos.length());
            // Each walker against the three behind it, and against the stall-holder.
            for i in 0..members.len() {
                for j in i + 1..members.len().min(i + 4) {
                    overlap = overlap.max((0.6 - members[i].pos.distance(members[j].pos)) / 0.3);
                }
                if i > 0 {
                    overlap = overlap.max((0.6 - members[i].pos.distance(members[0].pos)) / 0.3);
                }
            }
        }
        pushed += peak / CROWDS as f32;
        extra += (1..members.len())
            .map(|i| 100.0 * (walked[i] / (members[i].pos.x - starts[i].x).abs() - 1.0))
            .sum::<f32>()
            / (WALKERS as f32 * CROWDS as f32);
    }
    (pushed, extra, overlap)
}

/// A stall-holder given priority is walked around instead of shoved down the street: how far the stream
/// pushes him falls with it, to nothing at `∞`, while the walkers do not walk further and nobody ever
/// touches anyone (the walkers have room to go round him).
#[test]
fn a_stall_holder_with_priority_is_walked_around_by_a_stream() {
    let table: Vec<(f32, (f32, f32, f32))> =
        [1.0, 2.0, 4.0, 8.0, 32.0, f32::INFINITY].map(|p| (p, stall_in_a_stream(p))).to_vec();
    for (priority, (pushed, extra, overlap)) in &table {
        println!(
            "stall-holder priority {priority}: pushed {pushed:.3} m, walkers {extra:.3}% further, overlap {overlap:.4} r"
        );
    }
    let (_, (equal, equal_extra, _)) = table[0];
    assert!(equal > 0.3, "the control must be pushed about: {equal} m");
    for pair in table.windows(2) {
        assert!(
            pair[1].1.0 < pair[0].1.0,
            "priority {}: pushed {} m, after {} m",
            pair[1].0,
            pair[1].1.0,
            pair[0].1.0
        );
    }
    for (priority, (_, extra, overlap)) in &table {
        assert!(
            *extra < equal_extra * 1.1 + 0.01,
            "priority {priority}: the walkers walk {extra}% further than {equal_extra}%"
        );
        assert!(*overlap < 1e-3, "priority {priority}: overlap {overlap} r");
    }
    let (_, (eight, _, _)) = table[3];
    assert!(
        eight < 0.5 * equal,
        "at priority 8 he is pushed {eight} m against {equal} m: a ninth of each dodge against a half"
    );
    assert!(table[5].1.0 < 1e-3, "at infinity he takes none of any dodge: {} m", table[5].1.0);
}

/// Right of way is a claim on the way to avoid a collision that has not happened. Discs that already
/// overlap have no way to give, so both push apart evenly whatever their priorities: each member's
/// answer is the one it gets at equal priorities, to the bit, for random overlapping pairs with random
/// priorities (zero, infinite and NaN included). Without this a high priority left the other member of
/// a touching pair to do all the separating, and the 100-agent circle swap's deepest overlap went from
/// 0.040 of a radius to 0.209 at a ratio of 8.
#[test]
fn overlapping_discs_push_apart_evenly_whatever_their_priorities() {
    let mut rng = Rng(71);
    let mut moved = 0;
    for case in 0..5_000 {
        let radius_a = rng.range(0.2, 0.6);
        let radius_b = rng.range(0.2, 0.6);
        let pos_a = rng.vec(-2.0, 2.0);
        let pos_b = pos_a
            + Vec2::from_angle(rng.range(0.0, std::f32::consts::TAU))
                * rng.range(0.02, 0.98)
                * (radius_a + radius_b);
        let (vel_a, vel_b) = (rng.vec(-1.5, 1.5), rng.vec(-1.5, 1.5));
        let (pref_a, pref_b) = (rng.vec(-1.5, 1.5), rng.vec(-1.5, 1.5));
        let solve_pair = |priority_a: f32, priority_b: f32| {
            let a = Agent { pos: pos_a, vel: vel_a, radius: radius_a, avoids: true, priority: priority_a };
            let b = Agent { pos: pos_b, vel: vel_b, radius: radius_b, avoids: true, priority: priority_b };
            (
                new_velocity(&a, pref_a, &[b], &[], HORIZON, DT, 100.0),
                new_velocity(&b, pref_b, &[a], &[], HORIZON, DT, 100.0),
            )
        };
        let (equal_a, equal_b) = solve_pair(1.0, 1.0);
        let (priority_a, priority_b) =
            (rng.priority(), if case % 7 == 0 { f32::NAN } else { rng.priority() });
        let (unequal_a, unequal_b) = solve_pair(priority_a, priority_b);
        assert_eq!(
            bits(unequal_a),
            bits(equal_a),
            "case {case}: priorities {priority_a}, {priority_b}, member a"
        );
        assert_eq!(
            bits(unequal_b),
            bits(equal_b),
            "case {case}: priorities {priority_a}, {priority_b}, member b"
        );
        if (equal_a - pref_a).length() > 1e-3 {
            moved += 1;
        }
    }
    assert!(moved > 4_000, "the overlapping pairs must be pushed apart, or this proves nothing: {moved}");
}
