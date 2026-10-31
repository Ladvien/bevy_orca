//! **Buildings are hard, and walkers go round them at speed.**
//!
//! Four buildings on a 12 m lattice with streets between them, and twenty-four walkers on three
//! routes: one stream east along the middle street, one north along the cross street, one round the
//! outside of the block. The streams cross at the middle junction and the third turns two corners.
//! The buildings are an [`Obstacles`] built once; each step is one `Crowd::solve_around`.
//!
//! Nothing here plans a route round a building: the way is each walker's wish (a waypoint per street),
//! as it would be from a navigator. What the obstacles do is keep every walker's disc off every wall,
//! on every step, however the others push, and let it slide round a corner without braking for it.
//!
//! Run: `cargo run -p bevy_orca --example street`

use bevy_math::Vec2;
use bevy_orca::{Crowd, CrowdMember, CrowdParams, Obstacles, ObstaclesBuilder};

const DT: f32 = 1.0 / 30.0;
const RADIUS: f32 = 0.3;
const SPEED: f32 = 1.4;
const PITCH: f32 = 12.0;
const HALF: f32 = 4.0;
const STEPS: usize = 1800;

struct Walker {
    pos: Vec2,
    vel: Vec2,
    /// The waypoints still to reach, in order.
    route: Vec<Vec2>,
    arrived: bool,
}

/// The distance from `p` to the nearest building's surface, negative inside one.
fn clearance(p: Vec2, centres: &[Vec2]) -> f32 {
    centres
        .iter()
        .map(|&c| {
            let d = (p - c).abs() - Vec2::splat(HALF);
            d.max(Vec2::ZERO).length() + d.x.max(d.y).min(0.0)
        })
        .fold(f32::INFINITY, f32::min)
}

fn main() {
    // The buildings: a 2 x 2 block, one solid box each. `add_box` is `add_solid` of a rectangle;
    // `add_solid` takes any polygon, in either winding, and `add_wall` a wall with no thickness.
    let centres: Vec<Vec2> = [(0.5, 0.5), (1.5, 0.5), (0.5, 1.5), (1.5, 1.5)]
        .iter()
        .map(|&(i, j)| Vec2::new(i * PITCH, j * PITCH))
        .collect();
    let mut builder = ObstaclesBuilder::new();
    for &c in &centres {
        builder.add_box(c, Vec2::splat(HALF)).expect("a building");
    }
    let obstacles: Obstacles = builder.build().expect("buildings that do not cross themselves");

    // Three streams of eight, each walker in its own lane of the street.
    let lane = |k: usize| (k % 4) as f32 * 0.9 - 1.35;
    let mut walkers: Vec<Walker> = Vec::new();
    for k in 0..8 {
        let back = -((k / 4) as f32) * 1.5;
        let (a, b, c) = (lane(k), lane(k + 1), lane(k + 2));
        walkers.push(Walker {
            pos: Vec2::new(back, PITCH + a),
            vel: Vec2::ZERO,
            route: vec![Vec2::new(2.0 * PITCH + 3.0, PITCH + a)],
            arrived: false,
        });
        walkers.push(Walker {
            pos: Vec2::new(PITCH + b, back),
            vel: Vec2::ZERO,
            route: vec![Vec2::new(PITCH + b, 2.0 * PITCH + 3.0)],
            arrived: false,
        });
        walkers.push(Walker {
            pos: Vec2::new(2.0 * PITCH - back, 2.0 * PITCH + c),
            vel: Vec2::ZERO,
            route: vec![Vec2::new(c, 2.0 * PITCH + c), Vec2::new(c, c)],
            arrived: false,
        });
    }

    let params = CrowdParams::new(3.0);
    let mut crowd = Crowd::default();
    let (mut deepest, mut closest) = (0.0f32, f32::INFINITY);

    println!("  {} walkers, radius {RADIUS} m, {} buildings, obstacle horizon {} s", walkers.len(), centres.len(), params.obstacle_horizon);
    println!("  step | arrived | mean speed | closest gap to a building");
    for step in 0..=STEPS {
        let members: Vec<CrowdMember> = walkers
            .iter()
            .enumerate()
            .map(|(i, w)| {
                let wish = match w.route.first() {
                    Some(&to) if !w.arrived => (to - w.pos).normalize_or_zero() * SPEED,
                    _ => Vec2::ZERO,
                };
                CrowdMember {
                    key: i as u64,
                    pos: w.pos,
                    vel: w.vel,
                    radius: RADIUS,
                    max_speed: SPEED,
                    pref_vel: Some(wish),
                }
            })
            .collect();
        // The one call: neighbours by the crowd's own rule, buildings by the obstacles, no walls of the
        // caller's own.
        let solved = crowd.solve_around(&obstacles, &members, &params, DT, |_, _| true, |_, _| {});
        for (w, v) in walkers.iter_mut().zip(solved) {
            w.vel = v.unwrap_or(Vec2::ZERO);
            w.pos += w.vel * DT;
            if let Some(&to) = w.route.first()
                && w.pos.distance(to) < 1.0
            {
                w.route.remove(0);
                w.arrived = w.route.is_empty();
            }
            let gap = clearance(w.pos, &centres) - RADIUS;
            closest = closest.min(gap);
            deepest = deepest.max(-gap);
        }
        if step % 150 == 0 {
            let arrived = walkers.iter().filter(|w| w.arrived).count();
            let moving: Vec<&Walker> = walkers.iter().filter(|w| !w.arrived).collect();
            let speed = moving.iter().fold(0.0, |sum, w| sum + w.vel.length()) / moving.len().max(1) as f32;
            println!("  {step:>4} | {arrived:>7} | {speed:>10.2} | {closest:>8.4}");
        }
    }

    println!("\n  deepest a disc reached into a building: {deepest:.4} m");
    println!("  arrived: {} of {}", walkers.iter().filter(|w| w.arrived).count(), walkers.len());
    println!(
        "\nThe closest gap is the smallest distance between a walker's disc and a wall on any step: it\n\
         is never negative, and the walkers that turn the corners of the block do not slow for them."
    );
}
