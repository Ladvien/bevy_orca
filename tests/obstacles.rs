//! **Static obstacles, as a caller sees them: where agents end up.**
//!
//! Every test here steps agents with `Crowd::solve_around` and measures the world with geometry of
//! its own (a distance to a box, to a wall), never with the crate's: an agent's disc must stay clear
//! of every obstacle on every step, whatever else is pushing it, and must still get where it is going
//! when there is a way.

use bevy_math::Vec2;
use bevy_orca::{
    Agent, Crowd, CrowdMember, CrowdParams, ObstacleError, Obstacles, ObstaclesBuilder, OrcaScratch,
    new_velocity_in,
};

const DT: f32 = 1.0 / 30.0;
const RADIUS: f32 = 0.3;
const SPEED: f32 = 1.5;
/// Rounding in `f32` positions a few metres from the origin: an agent's disc may not reach into an
/// obstacle by more than this.
const ROUNDING: f32 = 1.0e-4;

fn v(x: f32, y: f32) -> Vec2 {
    Vec2::new(x, y)
}

/// The SplitMix64 output mix: the tests' own stable pseudo-random numbers.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// The next 64 random bits from a counter.
fn draw(counter: &mut u64) -> u64 {
    *counter += 1;
    splitmix64(*counter)
}

fn unit(counter: &mut u64) -> f32 {
    *counter += 1;
    (splitmix64(*counter) >> 40) as f32 / (1u64 << 24) as f32
}

/// An obstacle as the test measures it: a box, or a wall with no thickness.
#[derive(Clone, Copy)]
enum Solid {
    Box { centre: Vec2, half: Vec2 },
    Wall { a: Vec2, b: Vec2 },
}

impl Solid {
    /// The distance from `p` to the solid: how far a disc centred there is from touching it, negative
    /// inside a box.
    fn distance(&self, p: Vec2) -> f32 {
        match *self {
            Solid::Box { centre, half } => {
                let d = (p - centre).abs() - half;
                d.max(Vec2::ZERO).length() + d.x.max(d.y).min(0.0)
            }
            Solid::Wall { a, b } => {
                let ab = b - a;
                let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
                p.distance(a + ab * t)
            }
        }
    }
}

fn obstacles_of(solids: &[Solid]) -> Obstacles {
    let mut builder = ObstaclesBuilder::new();
    for s in solids {
        match *s {
            Solid::Box { centre, half } => builder.add_box(centre, half),
            Solid::Wall { a, b } => builder.add_wall(a, b),
        }
        .expect("valid geometry");
    }
    builder.build().expect("valid geometry")
}

/// How far into a solid any of `agents`' discs reaches (0 when all are clear).
fn deepest(solids: &[Solid], agents: &[Vec2]) -> f32 {
    agents.iter().flat_map(|&p| solids.iter().map(move |s| RADIUS - s.distance(p))).fold(0.0, f32::max)
}

/// Within this of its goal an agent has arrived: its wish is the goal's offset, so it slows as it
/// closes, and an exact arrival is an exponential tail that says nothing about the walk.
const NEAR: f32 = 0.6;

/// One agent's journey among `solids` to `goal`, alone: the step it came within [`NEAR`] of the goal
/// (the number of steps if it never did), the deepest the disc reached into anything, and where it
/// ended.
fn walk(solids: &[Solid], start: Vec2, goal: Vec2, steps: usize) -> (usize, f32, Vec2) {
    let obstacles = obstacles_of(solids);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let (mut pos, mut vel) = (start, Vec2::ZERO);
    let (mut worst, mut arrived) = (0.0f32, steps);
    for step in 0..steps {
        let pref = (goal - pos).clamp_length_max(SPEED);
        let member = CrowdMember { key: 0, pos, vel, radius: RADIUS, max_speed: SPEED, pref_vel: Some(pref) };
        vel = crowd.solve_around(&obstacles, &[member], &params, DT, |_, _| true, |_, _| {})[0]
            .expect("solved");
        pos += vel * DT;
        worst = worst.max(deepest(solids, &[pos]));
        if arrived == steps && pos.distance(goal) < NEAR {
            arrived = step;
        }
    }
    (arrived, worst, pos)
}

/// A walker aimed straight at the middle of a building's face stops at it: it never enters, and it
/// does come up to it.
#[test]
fn an_agent_aimed_at_a_face_stops_at_it_and_never_enters() {
    let building = [Solid::Box { centre: v(0.0, 0.0), half: v(2.0, 1.0) }];
    let (_, worst, end) = walk(&building, v(-8.0, 0.0), v(0.0, 0.0), 400);
    assert!(worst <= ROUNDING, "the disc reached {worst} into the building");
    let gap = building[0].distance(end) - RADIUS;
    assert!((0.0..0.05).contains(&gap), "it ended {gap} m short of the face, not at it");
}

/// A walker whose straight line grazes a building, closer to its corner than its own radius, bends
/// round the corner without braking for it: it arrives within 15% of the straight-line time and its
/// disc never touches brick. (Obstacles keep an agent off a building; they do not plan the way
/// round, so the goals here are the ones a walker can reach by being pushed aside a few decimetres.)
#[test]
fn a_walker_bends_round_a_corner_at_speed() {
    let building = [Solid::Box { centre: v(2.0, 0.0), half: v(2.0, 1.0) }];
    for (start, goal) in [
        (v(-3.0, 1.15), v(7.0, 0.9)),
        (v(-3.0, -1.15), v(7.0, -0.9)),
        (v(7.0, 1.1), v(-3.0, 1.0)),
        (v(4.1, -4.0), v(3.9, 5.0)),
    ] {
        let straight = (start.distance(goal) - NEAR) / SPEED;
        let (arrived, worst, _) = walk(&building, start, goal, 600);
        assert!(worst <= ROUNDING, "{start} to {goal}: the disc reached {worst} into the building");
        let took = arrived as f32 * DT;
        assert!(took <= straight * 1.15 + 0.5, "{start} to {goal}: took {took} s, straight is {straight} s");
    }
}

/// A wall with no thickness holds from both sides: two walkers aimed through it from opposite sides
/// each stop at it.
#[test]
fn a_wall_with_no_thickness_holds_from_both_sides() {
    let wall = [Solid::Wall { a: v(0.0, -3.0), b: v(0.0, 3.0) }];
    for (start, goal) in
        [(v(-4.0, 0.5), v(4.0, 0.5)), (v(4.0, -0.5), v(-4.0, -0.5)), (v(0.4, 5.0), v(0.4, -5.0))]
    {
        let (_, worst, end) = walk(&wall, start, goal, 400);
        assert!(worst <= ROUNDING, "{start} to {goal}: the disc reached {worst} through the wall");
        assert!(end.distance(start) > 2.0, "{start} to {goal}: it never left its start: {end}");
    }
}

/// Eight touching boxes are a wall as good as one long box: a walker aimed at it from several angles
/// stops against both, and ends near where the single box lets it end.
#[test]
fn touching_boxes_hold_like_one_long_box() {
    let row: Vec<Solid> =
        (0..8).map(|i| Solid::Box { centre: v(i as f32 * 1.0 - 3.5, 0.0), half: v(0.5, 1.0) }).collect();
    let one = [Solid::Box { centre: v(0.0, 0.0), half: v(4.0, 1.0) }];
    for goal in [v(0.3, 4.0), v(-2.0, 5.0), v(3.0, 3.0), v(1.0, 1.2)] {
        let start = v(-1.0, -4.0).lerp(goal, 0.0);
        let aimed = v(goal.x, -goal.y);
        let (_, worst_row, end_row) = walk(&row, start, aimed, 500);
        let (_, worst_one, end_one) = walk(&one, start, aimed, 500);
        assert!(worst_row <= ROUNDING, "goal {goal}: the row let the disc {worst_row} in");
        assert!(worst_one <= ROUNDING, "goal {goal}: the box let the disc {worst_one} in");
        assert!(
            end_row.distance(end_one) < 0.25,
            "goal {goal}: the row ends at {end_row}, the box at {end_one}"
        );
    }
}

/// A crowd pressed against a wall by its own members never pushes through it: the lines are hard,
/// and a neighbour behind cannot buy room by shoving the one in front into brick.
#[test]
fn a_crowd_pressing_on_a_wall_never_pushes_through() {
    let wall = [Solid::Wall { a: v(-20.0, 0.0), b: v(20.0, 0.0) }];
    let obstacles = obstacles_of(&wall);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let mut counter = 0u64;
    let mut agents: Vec<(Vec2, Vec2)> = (0..90)
        .map(|i| (v((i % 15) as f32 * 0.65 - 4.5, 0.6 + (i / 15) as f32 * 0.65), Vec2::ZERO))
        .collect();
    let mut worst = 0.0f32;
    for _ in 0..300 {
        let members: Vec<CrowdMember> = agents
            .iter()
            .enumerate()
            .map(|(i, &(pos, vel))| CrowdMember {
                key: i as u64,
                pos,
                vel,
                radius: RADIUS,
                max_speed: SPEED,
                pref_vel: Some(v(unit(&mut counter) - 0.5, -SPEED)),
            })
            .collect();
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
        for (agent, s) in agents.iter_mut().zip(solved) {
            agent.1 = s.expect("solved");
            agent.0 += agent.1 * DT;
        }
        let positions: Vec<Vec2> = agents.iter().map(|a| a.0).collect();
        worst = worst.max(deepest(&wall, &positions));
    }
    assert!(worst <= ROUNDING, "a disc was pushed {worst} into the wall");
    // And they did press: the front row is at the wall.
    let front = agents.iter().map(|a| wall[0].distance(a.0) - RADIUS).fold(f32::INFINITY, f32::min);
    assert!(front < 0.05, "nobody reached the wall ({front} m short)");
}

/// A room built as an enclosure keeps its crowd inside however they are aimed, whichever way round
/// its vertices were listed.
#[test]
fn an_enclosure_keeps_everyone_in() {
    let room = [v(-5.0, -5.0), v(5.0, -5.0), v(5.0, 5.0), v(-5.0, 5.0)];
    let mut reversed = room;
    reversed.reverse();
    for ring in [room, reversed] {
        let mut builder = ObstaclesBuilder::new();
        builder.add_enclosure(&ring).expect("a room");
        let obstacles = builder.build().expect("a room");
        let params = CrowdParams::new(3.0);
        let mut crowd = Crowd::default();
        let mut counter = 7u64;
        let mut agents: Vec<(Vec2, Vec2, Vec2)> = (0..40)
            .map(|_| {
                let pos = v(unit(&mut counter) * 8.0 - 4.0, unit(&mut counter) * 8.0 - 4.0);
                let angle = unit(&mut counter) * core::f32::consts::TAU;
                (pos, Vec2::ZERO, v(angle.cos(), angle.sin()) * 30.0)
            })
            .collect();
        let mut worst = 0.0f32;
        for _ in 0..500 {
            let members: Vec<CrowdMember> = agents
                .iter()
                .enumerate()
                .map(|(i, &(pos, vel, goal))| CrowdMember {
                    key: i as u64,
                    pos,
                    vel,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some((goal - pos).clamp_length_max(SPEED)),
                })
                .collect();
            let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
            for (agent, s) in agents.iter_mut().zip(solved) {
                agent.1 = s.expect("solved");
                agent.0 += agent.1 * DT;
                worst = worst.max(agent.0.abs().max_element() + RADIUS - 5.0);
            }
        }
        assert!(worst <= ROUNDING, "a disc reached {worst} past the room's wall");
    }
}

/// The bench's bottleneck, small: a room draining through a gap two agents wide in a wall. Everyone
/// gets through, nobody touches the wall.
#[test]
fn a_gap_two_agents_wide_passes_its_crowd() {
    let (far, gap) = (30.0, 0.7);
    let wall =
        [Solid::Wall { a: v(0.0, gap), b: v(0.0, far) }, Solid::Wall { a: v(0.0, -gap), b: v(0.0, -far) }];
    let obstacles = obstacles_of(&wall);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let mut agents: Vec<(Vec2, Vec2)> =
        (0..48).map(|i| (v(-1.5 - (i % 6) as f32 * 0.9, -3.6 + (i / 6) as f32 * 0.9), Vec2::ZERO)).collect();
    let mut worst = 0.0f32;
    for _ in 0..1800 {
        let members: Vec<CrowdMember> = agents
            .iter()
            .enumerate()
            .filter(|(_, a)| a.0.x < 3.0)
            .map(|(i, &(pos, vel))| {
                let aim = if pos.x < 0.0 { v(0.5, 0.0) } else { v(8.0, 0.0) };
                CrowdMember {
                    key: i as u64,
                    pos,
                    vel,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some((aim - pos).normalize_or_zero() * SPEED),
                }
            })
            .collect();
        let keys: Vec<usize> = members.iter().map(|m| m.key as usize).collect();
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
        for (&k, s) in keys.iter().zip(solved) {
            agents[k].1 = s.expect("solved");
            let moved = agents[k].1 * DT;
            agents[k].0 += moved;
        }
        let positions: Vec<Vec2> = agents.iter().map(|a| a.0).collect();
        worst = worst.max(deepest(&wall, &positions));
    }
    assert!(worst <= ROUNDING, "a disc reached {worst} into the wall");
    let through = agents.iter().filter(|a| a.0.x >= 3.0).count();
    assert_eq!(through, 48, "only {through} of 48 got through the gap in a minute");
}

/// A small street grid: blocks of buildings, walkers on Manhattan routes along the street middles.
/// Nobody touches a building, and nobody stalls.
#[test]
fn a_city_of_boxes_keeps_its_walkers_out_and_moving() {
    let (pitch, block, streets) = (12.0, 8.0, 4);
    let mut solids = Vec::new();
    for i in 0..streets {
        for j in 0..streets {
            solids.push(Solid::Box {
                centre: v(i as f32 * pitch + pitch * 0.5, j as f32 * pitch + pitch * 0.5),
                half: Vec2::splat(block * 0.5),
            });
        }
    }
    let obstacles = obstacles_of(&solids);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let mut counter = 11u64;
    // Street middles run along x = k·pitch (vertical) and y = k·pitch (horizontal), k in 0..=streets.
    let lane = |counter: &mut u64| (unit(counter) * (streets + 1) as f32).floor().min(streets as f32) * pitch;
    let mut agents: Vec<(Vec2, Vec2, Vec<Vec2>, usize)> = Vec::new();
    for _ in 0..120 {
        let y = lane(&mut counter);
        let start = v(unit(&mut counter) * streets as f32 * pitch, y + (unit(&mut counter) - 0.5) * 2.0);
        let goal = v(lane(&mut counter), lane(&mut counter));
        agents.push((start, Vec2::ZERO, vec![v(goal.x, y), goal], 0));
    }
    let (mut worst, mut slow) = (0.0f32, vec![0usize; agents.len()]);
    for _ in 0..800 {
        let members: Vec<CrowdMember> = agents
            .iter_mut()
            .enumerate()
            .map(|(i, (pos, vel, route, at))| {
                if pos.distance(route[*at]) < 1.0 && *at + 1 < route.len() {
                    *at += 1;
                }
                let aim = route[*at];
                CrowdMember {
                    key: i as u64,
                    pos: *pos,
                    vel: *vel,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some(if pos.distance(aim) < 0.2 {
                        Vec2::ZERO
                    } else {
                        (aim - *pos).clamp_length_max(SPEED)
                    }),
                }
            })
            .collect();
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
        for (i, (agent, s)) in agents.iter_mut().zip(solved).enumerate() {
            agent.1 = s.expect("solved");
            agent.0 += agent.1 * DT;
            let goal = agent.2[agent.2.len() - 1];
            let moving = agent.1.length() >= 0.05 * SPEED || agent.0.distance(goal) < 1.0;
            slow[i] = if moving { 0 } else { slow[i] + 1 };
        }
        let positions: Vec<Vec2> = agents.iter().map(|a| a.0).collect();
        worst = worst.max(deepest(&solids, &positions));
    }
    assert!(worst <= ROUNDING, "a walker's disc reached {worst} into a building");
    let stalled = slow.iter().filter(|&&n| n as f32 * DT > 5.0).count();
    assert_eq!(stalled, 0, "{stalled} of 120 walkers stood still for over 5 s");
}

/// An empty set of obstacles is no obstacles: `solve_around` answers exactly what `solve` does.
#[test]
fn an_empty_set_changes_no_answer() {
    let empty = Obstacles::default();
    let params = CrowdParams::new(3.0);
    let (mut a, mut b) = (Crowd::default(), Crowd::default());
    let mut counter = 3u64;
    let members: Vec<CrowdMember> = (0..200)
        .map(|i| CrowdMember {
            key: i,
            pos: v(unit(&mut counter) * 20.0, unit(&mut counter) * 20.0),
            vel: v(unit(&mut counter) - 0.5, unit(&mut counter) - 0.5),
            radius: RADIUS,
            max_speed: SPEED,
            pref_vel: Some(v(unit(&mut counter) - 0.5, unit(&mut counter) - 0.5) * 3.0),
        })
        .collect();
    let walls = |i: usize, buf: &mut Vec<(Vec2, f32)>| buf.push((Vec2::Y, i as f32 * 0.01));
    let plain = a.solve(&members, &params, DT, |_, _| true, walls).to_vec();
    let around = b.solve_around(&empty, &members, &params, DT, |_, _| true, walls).to_vec();
    assert_eq!(plain, around);
}

/// The answer is a function of the world: the same crowd among the same obstacles gives each member
/// the same velocity, bit for bit, in whichever order the slice lists them, step after step.
#[test]
fn the_answer_does_not_depend_on_member_order() {
    let solids = [
        Solid::Box { centre: v(6.0, 6.0), half: v(2.0, 1.0) },
        Solid::Box { centre: v(14.0, 9.0), half: v(1.0, 3.0) },
        Solid::Wall { a: v(3.0, 14.0), b: v(15.0, 16.0) },
    ];
    let obstacles = obstacles_of(&solids);
    let params = CrowdParams::new(3.0);
    let mut counter = 5u64;
    let n = 160usize;
    let goals: Vec<Vec2> = (0..n).map(|_| v(unit(&mut counter) * 20.0, unit(&mut counter) * 20.0)).collect();
    let start: Vec<Vec2> = (0..n).map(|_| v(unit(&mut counter) * 20.0, unit(&mut counter) * 20.0)).collect();
    let (mut fwd, mut rev) = (Crowd::default(), Crowd::default());
    type State = Vec<(Vec2, Vec2)>;
    let (mut state_a, mut state_b): (State, State) =
        (start.iter().map(|&p| (p, Vec2::ZERO)).collect(), start.iter().map(|&p| (p, Vec2::ZERO)).collect());
    for step in 0..120 {
        let build = |state: &[(Vec2, Vec2)], order: &[usize]| -> Vec<CrowdMember> {
            order
                .iter()
                .map(|&i| CrowdMember {
                    key: i as u64,
                    pos: state[i].0,
                    vel: state[i].1,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some((goals[i] - state[i].0).clamp_length_max(SPEED)),
                })
                .collect()
        };
        let order_a: Vec<usize> = (0..n).collect();
        let order_b: Vec<usize> = (0..n).rev().collect();
        let (ma, mb) = (build(&state_a, &order_a), build(&state_b, &order_b));
        let sa = fwd.solve_around(&obstacles, &ma, &params, DT, |_, _| true, |_, _| {}).to_vec();
        let sb = rev.solve_around(&obstacles, &mb, &params, DT, |_, _| true, |_, _| {}).to_vec();
        for (slot, &i) in order_a.iter().enumerate() {
            state_a[i].1 = sa[slot].expect("solved");
            let moved = state_a[i].1 * DT;
            state_a[i].0 += moved;
        }
        for (slot, &i) in order_b.iter().enumerate() {
            state_b[i].1 = sb[slot].expect("solved");
            let moved = state_b[i].1 * DT;
            state_b[i].0 += moved;
        }
        let same = state_a
            .iter()
            .zip(&state_b)
            .all(|(a, b)| a.1.to_array().map(f32::to_bits) == b.1.to_array().map(f32::to_bits));
        assert!(same, "step {step}: a velocity changed with the member order");
    }
}

/// Obstacles and `walls` compose: a caller's own wall (a door that is shut this step) and a building
/// both hold.
#[test]
fn the_callers_own_walls_and_obstacles_both_hold() {
    let building = [Solid::Box { centre: v(0.0, 0.0), half: v(1.0, 1.0) }];
    let obstacles = obstacles_of(&building);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let (mut pos, mut vel) = (v(-4.0, 3.0), Vec2::ZERO);
    let (mut worst, mut ceiling) = (0.0f32, f32::NEG_INFINITY);
    for _ in 0..300 {
        // A shut door: nothing may approach y = 3.6 from below, the agent's start 0.6 below it.
        let member = CrowdMember {
            key: 0,
            pos,
            vel,
            radius: RADIUS,
            max_speed: SPEED,
            pref_vel: Some(v(1.0, 1.0).normalize() * SPEED),
        };
        let door = (Vec2::Y, ((3.6 - pos.y - RADIUS).max(0.0)) / DT);
        vel = crowd.solve_around(&obstacles, &[member], &params, DT, |_, _| true, |_, buf| buf.push(door))[0]
            .expect("solved");
        pos += vel * DT;
        worst = worst.max(deepest(&building, &[pos]));
        ceiling = ceiling.max(pos.y + RADIUS);
    }
    assert!(worst <= ROUNDING, "the disc reached {worst} into the building");
    assert!(ceiling <= 3.6 + ROUNDING, "the disc went through the caller's door: {ceiling}");
}

/// Hostile input never gives a non-finite velocity or line, and never panics: agents at, on, inside
/// and beyond the obstacles, with speeds and horizons that are zero, negative, NaN and enormous.
#[test]
fn hostile_agents_among_obstacles_give_only_finite_answers() {
    let mut builder = ObstaclesBuilder::new();
    builder.add_box(v(0.0, 0.0), v(2.0, 1.0)).expect("box");
    builder.add_wall(v(5.0, -3.0), v(5.0, 3.0)).expect("wall");
    builder.add_enclosure(&[v(-20.0, -20.0), v(20.0, -20.0), v(20.0, 20.0), v(-20.0, 20.0)]).expect("room");
    builder.add_solid(&[v(8.0, 8.0), v(12.0, 8.0), v(10.0, 12.0)]).expect("triangle");
    let obstacles = builder.build().expect("valid");
    let extremes = [
        0.0,
        -0.0,
        1.0e-30,
        1.0,
        100.0,
        1.0e20,
        f32::MAX,
        -f32::MAX,
        f32::INFINITY,
        f32::NAN,
        2.0,
        -2.0,
        5.0,
        12.0,
    ];
    let mut scratch = OrcaScratch::default();
    let mut crowd = Crowd::default();
    let mut counter = 0u64;
    let mut walls = Vec::new();
    for _ in 0..40_000 {
        let pick = |counter: &mut u64| {
            extremes[(splitmix64({
                *counter += 1;
                *counter
            }) % extremes.len() as u64) as usize]
        };
        let pos = v(pick(&mut counter), pick(&mut counter));
        let vel = v(pick(&mut counter), pick(&mut counter));
        let agent = Agent { pos, vel, radius: pick(&mut counter), avoids: true };
        let (speed, horizon) = (pick(&mut counter), pick(&mut counter));
        walls.clear();
        obstacles.push_walls(&mut scratch, &agent, speed, horizon, &mut walls);
        for &(normal, c) in &walls {
            assert!(
                normal.is_finite() && c.is_finite() && normal.is_normalized(),
                "{agent:?} {speed} {horizon}: line {normal:?} {c}"
            );
        }
        let pref = v(pick(&mut counter), pick(&mut counter));
        let answer = new_velocity_in(&mut scratch, &agent, pref, &[], &walls, horizon, DT, speed);
        assert!(answer.is_finite(), "{agent:?} {pref:?} {speed} {horizon}: {answer:?}");
    }
    // The same through the crowd, with the horizon a member parameter.
    for step in 0..2_000u64 {
        let pick = |counter: &mut u64| {
            extremes[(splitmix64({
                *counter += 1;
                *counter
            }) % extremes.len() as u64) as usize]
        };
        let members: Vec<CrowdMember> = (0..8)
            .map(|k| CrowdMember {
                key: k,
                pos: v(pick(&mut counter), pick(&mut counter)),
                vel: v(pick(&mut counter), pick(&mut counter)),
                radius: pick(&mut counter),
                max_speed: pick(&mut counter),
                pref_vel: Some(v(pick(&mut counter), pick(&mut counter))),
            })
            .collect();
        let params = CrowdParams { obstacle_horizon: pick(&mut counter), ..CrowdParams::new(3.0) };
        let dt = if step % 3 == 0 { pick(&mut counter) } else { DT };
        for answer in
            crowd.solve_around(&obstacles, &members, &params, dt, |_, _| true, |_, _| {}).iter().flatten()
        {
            assert!(answer.is_finite(), "step {step}: {answer:?}");
        }
    }
}

/// An agent on a vertex, on an edge and inside a solid is given a finite answer, and the one on an
/// edge's outer side cannot approach.
#[test]
fn an_agent_on_an_edge_or_a_vertex_cannot_approach() {
    let building = [Solid::Box { centre: v(0.0, 0.0), half: v(1.0, 1.0) }];
    let obstacles = obstacles_of(&building);
    let mut scratch = OrcaScratch::default();
    for pos in
        [v(1.0, 0.0), v(1.0, 1.0), v(1.0 + 1e-6, 0.3), v(1.2, 0.0), v(1.2, 1.2), v(0.0, 0.0), v(-1.0, -1.0)]
    {
        let agent = Agent { pos, vel: Vec2::ZERO, radius: RADIUS, avoids: true };
        let mut walls = Vec::new();
        obstacles.push_walls(&mut scratch, &agent, SPEED, 0.5, &mut walls);
        let answer = new_velocity_in(
            &mut scratch,
            &agent,
            v(-1.0, -1.0).normalize() * SPEED,
            &[],
            &walls,
            1.0,
            DT,
            SPEED,
        );
        assert!(answer.is_finite(), "{pos}: {answer:?}");
    }
    // Just outside the right face, wishing to go left into it: it may not.
    let agent = Agent { pos: v(1.2, 0.0), vel: Vec2::ZERO, radius: RADIUS, avoids: true };
    let mut walls = Vec::new();
    obstacles.push_walls(&mut scratch, &agent, SPEED, 0.5, &mut walls);
    let answer = new_velocity_in(&mut scratch, &agent, v(-SPEED, 0.0), &[], &walls, 1.0, DT, SPEED);
    assert!(answer.x >= -1e-6, "a disc touching the face walked into it: {answer:?}");
}

/// The builder reports what is wrong with the geometry, to the caller who built it.
#[test]
fn the_builder_says_what_is_wrong() {
    let mut b = ObstaclesBuilder::new();
    assert_eq!(b.add_wall(v(1.0, 1.0), v(1.0, 1.0)).err(), Some(ObstacleError::DegenerateEdge));
    assert!(
        b.add_solid(&[v(0.0, 0.0), v(f32::NAN, 0.0), v(1.0, 1.0)])
            .err()
            .is_some_and(|e| e == ObstacleError::NotFinite)
    );
    assert!(ObstacleError::SelfIntersecting.to_string().contains("cross"));
}

/// An agent whose disc touches a wall's face, to within a few ulps, and moves along it, is still held
/// off the wall. RVO2 hands such an agent no line at all (its "foreign leg" rule skips the wall's own
/// face), and it walks into the wall by the whole step: the first state below is one the bench found,
/// a walker in a crowd sliding up the wall of a doorway at 0.27 m/s, 4e-8 m from touching, which was
/// given an empty set of lines and a velocity of `(0.66, 0.06)` into brick. 168 813 touching agents in
/// RVO2 as shipped: 191 do the same.
#[test]
fn an_agent_touching_a_wall_within_rounding_cannot_walk_into_it() {
    let wall = [Solid::Wall { a: v(0.0, -0.7), b: v(0.0, -30.0) }];
    let obstacles = obstacles_of(&wall);
    let mut scratch = OrcaScratch::default();
    let mut walls = Vec::new();
    let mut step = |pos: Vec2, vel: Vec2, pref: Vec2| -> f32 {
        let agent = Agent { pos, vel, radius: RADIUS, avoids: true };
        walls.clear();
        obstacles.push_walls(&mut scratch, &agent, 1.3, 0.5, &mut walls);
        let answer = new_velocity_in(&mut scratch, &agent, pref, &[], &walls, 1.0, DT, 1.3);
        deepest(&wall, &[pos + answer * DT])
    };
    let found =
        step(v(-0.300_000_04, -0.720_935_34), v(5.736_947e-7, 0.270_439_12), v(0.965_720_83, 0.870_277_76));
    assert!(found <= ROUNDING, "the bench's walker went {found} into the wall in one step");

    // Then every way a walker can touch a wall: both faces, both ends, anywhere along it, a few ulps
    // to a few millimetres from touching, moving along it, away from it and into it.
    let mut counter = 99u64;
    let mut worst = 0.0f32;
    for _ in 0..30_000 {
        let delta = [0.0, 4e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3][(draw(&mut counter) % 7) as usize];
        let side = if draw(&mut counter).is_multiple_of(2) { -1.0 } else { 1.0 };
        let y = match draw(&mut counter) % 4 {
            0 => -0.7 - unit(&mut counter) * 0.05,
            1 => -30.0 + unit(&mut counter) * 0.05,
            _ => -0.7 - unit(&mut counter) * 29.3,
        };
        let pos = v(side * (RADIUS + delta), y);
        let along = if unit(&mut counter) < 0.5 { 1.0 } else { -1.0 };
        let vel = v(0.0, along * unit(&mut counter) * 1.3);
        let angle = unit(&mut counter) * core::f32::consts::TAU;
        let pref = v(angle.cos(), angle.sin()) * 1.3;
        worst = worst.max(step(pos, vel, pref));
    }
    assert!(worst <= ROUNDING, "a touching walker went {worst} into the wall in one step");
}

/// A walker a few ulps from a convex corner of a building, on the extension of one of its faces, creeping
/// along it with its wish into the corner, was handed no line at all and stepped into the corner by a
/// whole step. Viewed obliquely from beyond the end of a face, an edge defines the velocity obstacle by
/// the vertex, and the leg that lies along its own face was replaced by that face's direction and skipped
/// as "cast by the neighbour": the neighbour being itself. Found by a crowd on random caves (28
/// agent-steps in 21.6 million, 4.7 cm into a cell); the state is a walker at `(0.3, 24.000284)` beside the
/// corner `(0, 24)`, speed 0.03, wishing for `(-1.45, 0.4)`.
#[test]
fn an_agent_creeping_past_a_convex_corner_cannot_step_into_it() {
    let building = [Solid::Box { centre: v(-0.5, 23.5), half: Vec2::splat(0.5) }];
    let obstacles = obstacles_of(&building);
    let mut scratch = OrcaScratch::default();
    let mut walls = Vec::new();
    let mut step = |pos: Vec2, vel: Vec2, pref: Vec2| -> f32 {
        let agent = Agent { pos, vel, radius: RADIUS, avoids: true };
        walls.clear();
        obstacles.push_walls(&mut scratch, &agent, SPEED, 0.5, &mut walls);
        let answer = new_velocity_in(&mut scratch, &agent, pref, &[], &walls, 2.0, DT, SPEED);
        deepest(&building, &[pos + answer * DT])
    };
    let found = step(v(0.3, 24.000_284), v(5.98e-5, -0.031_08), v(-1.446_931_8, 0.395_459_62));
    assert!(found <= ROUNDING, "the walker from the caves went {found} into the corner in one step");

    // Then every corner of a box, on the extension of each of its two faces, a few ulps to a few
    // millimetres from touching and from the end of the face, creeping or still, wishing for the corner.
    let (centre, half) = (v(2.0, -1.0), v(1.5, 0.5));
    let building = [Solid::Box { centre, half }];
    let obstacles = obstacles_of(&building);
    let mut counter = 61u64;
    let mut worst = 0.0f32;
    let mut tried = 0usize;
    for _ in 0..40_000 {
        let corner = centre
            + v(
                if draw(&mut counter) % 2 == 0 { -half.x } else { half.x },
                if draw(&mut counter) % 2 == 0 { -half.y } else { half.y },
            );
        let (sx, sy) = ((corner.x - centre.x).signum(), (corner.y - centre.y).signum());
        // Outward normal of the face, and the direction along its extension away from the box.
        let (normal, along) =
            if draw(&mut counter) % 2 == 0 { (v(sx, 0.0), v(0.0, sy)) } else { (v(0.0, sy), v(sx, 0.0)) };
        let delta = [0.0, 4e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3][(draw(&mut counter) % 7) as usize];
        let overshoot = [0.0, 1e-6, 1e-5, 1e-4, 3e-4, 1e-3][(draw(&mut counter) % 6) as usize];
        let pos = corner + normal * (RADIUS + delta) + along * overshoot;
        let creep = [0.0, 1e-5, 0.03, 0.1][(draw(&mut counter) % 4) as usize];
        let angle = unit(&mut counter) * core::f32::consts::TAU;
        let vel =
            if draw(&mut counter) % 2 == 0 { along * -creep } else { v(angle.cos(), angle.sin()) * creep };
        let wobble = (unit(&mut counter) - 0.5) * 1.0;
        let aim = corner - pos;
        let pref = (aim.normalize() * 1.0 + aim.perp().normalize() * wobble).normalize() * SPEED;
        worst = worst.max(step(pos, vel, pref));
        tried += 1;
    }
    assert!(
        tried == 40_000 && worst <= ROUNDING,
        "a walker at a corner went {worst} into the building in one step"
    );
}

/// The lines are hard: whatever the crowd does to a member, the velocity it is given satisfies every
/// obstacle line cast on it, to rounding. (A host's own `f32` positions, summed step after step at a
/// coordinate of a hundred metres, can still lose a few `1e-5` m a step; this is the solver's half.)
#[test]
fn every_answer_satisfies_every_obstacle_line() {
    let mut solids = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            solids.push(Solid::Box {
                centre: v(i as f32 * 12.0 + 6.0, j as f32 * 12.0 + 6.0),
                half: v(4.0, 4.0),
            });
        }
    }
    solids.push(Solid::Wall { a: v(-2.0, -2.0), b: v(50.0, -2.0) });
    let obstacles = obstacles_of(&solids);
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let mut counter = 21u64;
    let mut agents: Vec<(Vec2, Vec2, Vec2)> = (0..260)
        .map(|_| {
            let street = (draw(&mut counter) % 5) as f32 * 12.0;
            let along = unit(&mut counter) * 48.0;
            let pos = if draw(&mut counter).is_multiple_of(2) { v(along, street) } else { v(street, along) };
            let goal = v(unit(&mut counter) * 48.0, unit(&mut counter) * 48.0);
            (pos + v(unit(&mut counter) - 0.5, unit(&mut counter) - 0.5), Vec2::ZERO, goal)
        })
        .collect();
    let (mut lines, mut worst) = (0usize, 0.0f32);
    let mut scratch = OrcaScratch::default();
    for _ in 0..300 {
        let members: Vec<CrowdMember> = agents
            .iter()
            .enumerate()
            .map(|(i, &(pos, vel, goal))| CrowdMember {
                key: i as u64,
                pos,
                vel,
                radius: RADIUS,
                max_speed: SPEED,
                pref_vel: Some((goal - pos).clamp_length_max(SPEED)),
            })
            .collect();
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {}).to_vec();
        for (member, answer) in members.iter().zip(&solved) {
            let answer = answer.expect("solved");
            let agent = Agent { pos: member.pos, vel: member.vel, radius: RADIUS, avoids: true };
            let mut walls = Vec::new();
            obstacles.push_walls(&mut scratch, &agent, SPEED, params.obstacle_horizon, &mut walls);
            for &(normal, c) in &walls {
                lines += 1;
                worst = worst.max(answer.dot(normal) - c);
            }
        }
        for (agent, answer) in agents.iter_mut().zip(&solved) {
            agent.1 = answer.expect("solved");
            let moved = agent.1 * DT;
            agent.0 += moved;
        }
    }
    assert!(lines > 5_000, "only {lines} obstacle lines were cast: the crowd never met a wall");
    assert!(worst <= 1.0e-4, "a velocity broke a hard obstacle line by {worst}");
}

/// A horizon shorter than the step is raised to it: with `dt = 1 s` and an obstacle horizon of 50 ms a
/// walker 0.7 m from a wall, running at it at 1.5 m/s, would cross it in the step (it looks 0.375 m
/// ahead and sees nothing). It stops at the face instead.
#[test]
fn an_obstacle_horizon_shorter_than_the_step_is_raised_to_it() {
    let wall = [Solid::Box { centre: v(2.0, 0.0), half: v(1.0, 5.0) }];
    let obstacles = obstacles_of(&wall);
    let params = CrowdParams { obstacle_horizon: 0.05, ..CrowdParams::new(3.0) };
    let mut crowd = Crowd::default();
    let member = CrowdMember {
        key: 0,
        pos: v(1.0 - RADIUS - 0.4, 0.0),
        vel: Vec2::ZERO,
        radius: RADIUS,
        max_speed: SPEED,
        pref_vel: Some(v(SPEED, 0.0)),
    };
    let dt = 1.0;
    let answer =
        crowd.solve_around(&obstacles, &[member], &params, dt, |_, _| true, |_, _| {})[0].expect("solved");
    let after = member.pos + answer * dt;
    assert!(
        deepest(&wall, &[after]) <= ROUNDING,
        "the walker ended the step {} inside the wall",
        deepest(&wall, &[after])
    );
    assert!(answer.x > 0.3, "and it still closed on the face: {answer:?}");
}

/// A building with a courtyard is a solid with an enclosure inside it: walkers in the courtyard cannot
/// leave it and walkers outside cannot enter the building, however they are aimed.
#[test]
fn a_courtyard_is_an_enclosure_inside_a_solid() {
    let outer = [v(-10.0, -10.0), v(10.0, -10.0), v(10.0, 10.0), v(-10.0, 10.0)];
    let inner = [v(-5.0, -5.0), v(5.0, -5.0), v(5.0, 5.0), v(-5.0, 5.0)];
    let mut builder = ObstaclesBuilder::new();
    builder.add_solid(&outer).expect("the building");
    builder.add_enclosure(&inner).expect("the courtyard");
    let obstacles = builder.build().expect("valid");
    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let mut counter = 5u64;
    // 24 in the courtyard wishing to go out (far away at a random bearing), 24 outside wishing to go to the middle.
    let mut agents: Vec<(Vec2, Vec2, Vec2, bool)> = Vec::new();
    for _ in 0..24 {
        let (angle, far) = (unit(&mut counter) * core::f32::consts::TAU, v(0.0, 0.0));
        let pos = v(unit(&mut counter) * 8.0 - 4.0, unit(&mut counter) * 8.0 - 4.0);
        agents.push((pos, Vec2::ZERO, far + v(angle.cos(), angle.sin()) * 60.0, true));
    }
    for _ in 0..24 {
        let angle = unit(&mut counter) * core::f32::consts::TAU;
        let pos = v(angle.cos(), angle.sin()) * (14.0 + unit(&mut counter) * 6.0);
        agents.push((pos, Vec2::ZERO, Vec2::ZERO, false));
    }
    let (mut worst_in, mut worst_out) = (0.0f32, 0.0f32);
    for _ in 0..600 {
        let members: Vec<CrowdMember> = agents
            .iter()
            .enumerate()
            .map(|(i, &(pos, vel, goal, _))| CrowdMember {
                key: i as u64,
                pos,
                vel,
                radius: RADIUS,
                max_speed: SPEED,
                pref_vel: Some((goal - pos).clamp_length_max(SPEED)),
            })
            .collect();
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
        for (agent, s) in agents.iter_mut().zip(solved) {
            agent.1 = s.expect("solved");
            let moved = agent.1 * DT;
            agent.0 += moved;
            if agent.3 {
                worst_in = worst_in.max(agent.0.abs().max_element() + RADIUS - 5.0);
            } else {
                // Outside the building: the distance to its square must stay at least the radius.
                let building = Solid::Box { centre: Vec2::ZERO, half: Vec2::splat(10.0) };
                worst_out = worst_out.max(RADIUS - building.distance(agent.0));
            }
        }
    }
    assert!(worst_in <= ROUNDING, "a courtyard walker's disc reached {worst_in} into the building");
    assert!(worst_out <= ROUNDING, "a walker outside got {worst_out} into the building");
}

fn cells_of(origin: Vec2, size: f32, width: usize, solid: &[bool]) -> Obstacles {
    let mut builder = ObstaclesBuilder::new();
    builder.add_cells(origin, size, width, solid).expect("a grid");
    builder.build().expect("every grid builds")
}

/// A random grid, `fill` of its cells solid.
fn random_cells(counter: &mut u64, width: usize, height: usize, fill: f32) -> Vec<bool> {
    (0..width * height).map(|_| unit(counter) < fill).collect()
}

/// A block of cells is its box: the lines a walker is given beside it are bit for bit the lines the box
/// gives, so a host that already placed `add_box` per building changes nothing by handing over its grid.
#[test]
fn a_block_of_cells_gives_the_lines_of_a_box() {
    let (origin, size, width, height) = (v(-8.0, 4.0), 2.0, 16usize, 12usize);
    // Blocks that do not touch: a gap of a cell or more between any two.
    let blocks = [(1usize, 1usize, 4usize, 3usize), (7, 1, 3, 3), (12, 2, 2, 7), (2, 6, 6, 4), (9, 7, 2, 4)];
    let mut solid = vec![false; width * height];
    let mut builder = ObstaclesBuilder::new();
    for &(c, r, w, h) in &blocks {
        for row in r..r + h {
            for column in c..c + w {
                solid[row * width + column] = true;
            }
        }
        let half = v(w as f32, h as f32) * size * 0.5;
        builder.add_box(origin + v(c as f32, r as f32) * size + half, half).expect("a box");
    }
    let boxes = builder.build().expect("boxes");
    let cells = cells_of(origin, size, width, &solid);
    assert_eq!(cells.len(), boxes.len());

    let mut counter = 3u64;
    let mut scratch = OrcaScratch::default();
    let (mut lines, mut asked) = (0usize, 0usize);
    for _ in 0..4000 {
        let pos = origin + v(unit(&mut counter) * width as f32, unit(&mut counter) * height as f32) * size;
        let vel = v(unit(&mut counter) - 0.5, unit(&mut counter) - 0.5) * 3.0;
        let agent = Agent { pos, vel, radius: RADIUS, avoids: true };
        let (mut a, mut b) = (Vec::new(), Vec::new());
        cells.push_walls(&mut scratch, &agent, SPEED, 0.5, &mut a);
        boxes.push_walls(&mut scratch, &agent, SPEED, 0.5, &mut b);
        let key = |l: &(Vec2, f32)| (l.0.x.to_bits(), l.0.y.to_bits(), l.1.to_bits());
        a.sort_by_key(key);
        b.sort_by_key(key);
        assert_eq!(a, b, "different lines for an agent at {pos}");
        lines += a.len();
        asked += 1;
    }
    assert!(lines > asked / 10, "the sweep barely met a wall: {lines} lines for {asked} agents");
}

/// The deepest any of `agents`' discs reaches into a solid cell of the grid.
fn deepest_in_cells(origin: Vec2, size: f32, width: usize, solid: &[bool], agents: &[Vec2]) -> f32 {
    let height = solid.len() / width;
    agents
        .iter()
        .map(|&p| {
            // Only the cells within a radius of the agent can be reached; the grid is small enough to
            // look at the 3 × 3 block around it and no more.
            let cell = ((p - origin) / size).floor();
            let mut deepest = 0.0f32;
            for dr in -1..=1i64 {
                for dc in -1..=1i64 {
                    let (c, r) = (cell.x as i64 + dc, cell.y as i64 + dr);
                    if c < 0
                        || r < 0
                        || c >= width as i64
                        || r >= height as i64
                        || !solid[r as usize * width + c as usize]
                    {
                        continue;
                    }
                    let b = Solid::Box {
                        centre: origin + v(c as f32 + 0.5, r as f32 + 0.5) * size,
                        half: Vec2::splat(size * 0.5),
                    };
                    deepest = deepest.max(RADIUS - b.distance(p));
                }
            }
            deepest
        })
        .fold(0.0, f32::max)
}

/// A crowd on a random cave never puts a disc into a solid cell: pinches where two cells meet at a
/// corner, one-cell walls, one-cell gaps and dead ends, at 30% and 50% solid, with every walker
/// wishing to go through the rock to a far goal.
#[test]
fn a_crowd_on_a_random_map_never_enters_a_solid_cell() {
    let params = CrowdParams::new(3.0);
    let (origin, size, width, height) = (v(-3.0, 5.0), 1.0, 30usize, 30usize);
    let mut worst = 0.0f32;
    let mut moved = 0.0f32;
    let mut counter = 17u64;
    for scene in 0..16usize {
        let fill = if scene < 8 { 0.3 } else { 0.5 };
        let solid = random_cells(&mut counter, width, height, fill);
        let obstacles = cells_of(origin, size, width, &solid);
        // Walkers start at the centres of distinct open cells (half a cell from any rock, so clear).
        let open: Vec<usize> = (0..solid.len()).filter(|&i| !solid[i]).collect();
        let mut agents: Vec<(Vec2, Vec2)> = Vec::new();
        let mut taken = std::collections::BTreeSet::new();
        while agents.len() < 40.min(open.len()) {
            let cell = open[(draw(&mut counter) % open.len() as u64) as usize];
            if taken.insert(cell) {
                agents.push((
                    origin + v((cell % width) as f32 + 0.5, (cell / width) as f32 + 0.5) * size,
                    Vec2::ZERO,
                ));
            }
        }
        let mut goals: Vec<Vec2> = agents.iter().map(|_| Vec2::ZERO).collect();
        let mut crowd = Crowd::default();
        for step in 0..600usize {
            if step % 150 == 0 {
                for g in &mut goals {
                    *g = origin
                        + v(unit(&mut counter) * width as f32, unit(&mut counter) * height as f32) * size;
                }
            }
            let members: Vec<CrowdMember> = agents
                .iter()
                .enumerate()
                .map(|(i, &(pos, vel))| CrowdMember {
                    key: i as u64,
                    pos,
                    vel,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some((goals[i] - pos).clamp_length_max(SPEED)),
                })
                .collect();
            let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
            for (agent, s) in agents.iter_mut().zip(solved) {
                agent.1 = s.expect("solved");
                agent.0 += agent.1 * DT;
                moved += agent.1.length() * DT;
            }
            let positions: Vec<Vec2> = agents.iter().map(|a| a.0).collect();
            worst = worst.max(deepest_in_cells(origin, size, width, &solid, &positions));
        }
    }
    assert!(moved > 2_000.0, "the crowds barely moved: {moved} m in all");
    assert!(worst <= ROUNDING, "a walker's disc reached {worst} into a solid cell");
}

/// Two solid cells that meet at a corner shut the gap between them: a walker whose straight line runs
/// through the touching point is stopped by the cells on either side, however long it pushes.
#[test]
fn a_gap_between_cells_that_touch_at_a_corner_is_shut() {
    let (origin, size, width) = (Vec2::ZERO, 1.0, 10usize);
    for diagonal in [false, true] {
        let mut solid = vec![false; 100];
        // Solid at (4, 4) and (5, 5), or at (5, 4) and (4, 5): the touching point is (5, 5).
        let cells = if diagonal { [(5, 4), (4, 5)] } else { [(4, 4), (5, 5)] };
        for (c, r) in cells {
            solid[r * width + c] = true;
        }
        let obstacles = cells_of(origin, size, width, &solid);
        assert_eq!(obstacles.len(), 8, "two separate squares");
        // From the middle of one open diagonal cell to the middle of the other, through the point.
        let (start, goal) = if diagonal { (v(4.5, 4.5), v(5.5, 5.5)) } else { (v(5.5, 4.5), v(4.5, 5.5)) };
        let params = CrowdParams::new(3.0);
        let mut crowd = Crowd::default();
        let (mut pos, mut vel, mut worst) = (start, Vec2::ZERO, 0.0f32);
        for _ in 0..600 {
            let member = CrowdMember {
                key: 0,
                pos,
                vel,
                radius: RADIUS,
                max_speed: SPEED,
                pref_vel: Some((goal - pos).clamp_length_max(SPEED)),
            };
            vel = crowd.solve_around(&obstacles, &[member], &params, DT, |_, _| true, |_, _| {})[0]
                .expect("solved");
            pos += vel * DT;
            worst = worst.max(deepest_in_cells(origin, size, width, &solid, &[pos]));
        }
        assert!(worst <= ROUNDING, "the disc reached {worst} into a cell (diagonal = {diagonal})");
        assert!(
            pos.distance(goal) > 0.4,
            "the walker got through the point to {pos} (diagonal = {diagonal})"
        );
    }
}
