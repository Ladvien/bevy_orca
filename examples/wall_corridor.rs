//! **Walls are hard constraints, neighbours are soft ones.**
//!
//! An agent walks a narrow corridor while another comes the other way. Wall half-planes are passed
//! as `(unit normal pointing toward the wall, max approach speed)` and enter the linear program as
//! *obstacle* constraints — they are satisfied before any neighbour constraint, so the solver will
//! shove the agent into a slower velocity rather than let it clip geometry.
//!
//! Watch `|y|` stay inside the corridor half-width the whole way through, even on the steps where
//! avoiding the oncoming agent would rather push it out.
//!
//! Run: `cargo run -p bevy_orca --example wall_corridor`

use bevy_math::Vec2;
use bevy_orca::{new_velocity, Agent};

const DT: f32 = 1.0 / 30.0;
const MAX_SPEED: f32 = 1.5;
const TIME_HORIZON: f32 = 2.0;
const RADIUS: f32 = 0.4;
/// Corridor runs along x; walls sit at y = ±HALF_WIDTH.
const HALF_WIDTH: f32 = 1.0;
const STEPS: usize = 150;

/// The two corridor walls as ORCA half-planes, given where the agent currently is.
///
/// The normal points *toward* the wall, and the scalar is how fast the agent may still close on it —
/// zero once it is within its own radius of the surface.
fn corridor_walls(pos: Vec2) -> Vec<(Vec2, f32)> {
    let to_top = (HALF_WIDTH - pos.y - RADIUS).max(0.0);
    let to_bottom = (HALF_WIDTH + pos.y - RADIUS).max(0.0);
    vec![(Vec2::Y, to_top / TIME_HORIZON), (-Vec2::Y, to_bottom / TIME_HORIZON)]
}

fn main() {
    let mut a = Agent { pos: Vec2::new(-5.0, 0.3), vel: Vec2::ZERO, radius: RADIUS, avoids: true };
    let mut b = Agent { pos: Vec2::new(5.0, -0.3), vel: Vec2::ZERO, radius: RADIUS, avoids: true };

    let (goal_a, goal_b) = (Vec2::new(5.0, 0.0), Vec2::new(-5.0, 0.0));
    let mut worst_overshoot = 0.0f32;
    let mut min_gap = f32::INFINITY;

    println!("  corridor half-width {HALF_WIDTH:.2}, agent radius {RADIUS:.2}");
    println!("  step |   A.x    A.y  |   B.x    B.y  |  gap   | wall clearance");

    for step in 0..STEPS {
        let pref_a = (goal_a - a.pos).normalize_or_zero() * MAX_SPEED;
        let pref_b = (goal_b - b.pos).normalize_or_zero() * MAX_SPEED;

        let va = new_velocity(&a, pref_a, &[b], &corridor_walls(a.pos), TIME_HORIZON, DT, MAX_SPEED);
        let vb = new_velocity(&b, pref_b, &[a], &corridor_walls(b.pos), TIME_HORIZON, DT, MAX_SPEED);

        a.vel = va;
        b.vel = vb;
        a.pos += va * DT;
        b.pos += vb * DT;

        let gap = a.pos.distance(b.pos);
        min_gap = min_gap.min(gap);
        // How far past the wall surface each agent's disc reached; should stay at ~0.
        let clearance = HALF_WIDTH - a.pos.y.abs() - RADIUS;
        worst_overshoot = worst_overshoot.max(-clearance);

        if step % 15 == 0 {
            println!(
                "  {step:>4} | {:>6.2} {:>6.2} | {:>6.2} {:>6.2} | {gap:>6.2} | {clearance:>8.3}",
                a.pos.x, a.pos.y, b.pos.x, b.pos.y,
            );
        }
    }

    println!("\n  closest the two discs came: {:.3} (sum of radii is {:.2})", min_gap, RADIUS * 2.0);
    println!("  deepest wall penetration:   {worst_overshoot:.4}");
    println!(
        "\nThe walls held while the pair still resolved each other — obstacle constraints are\n\
         satisfied first, so geometry wins over politeness."
    );
}
