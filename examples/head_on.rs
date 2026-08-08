//! **Reciprocity, and the one case it cannot resolve for you.**
//!
//! Three passes over the same scene, each printing how far to the side either agent stepped.
//!
//! 1. **Exactly collinear.** Both agents ask to walk straight through each other along the same line.
//!    The linear program is perfectly symmetric, so there is no reason to prefer left over right: both
//!    slow to a crawl and neither passes. This is a real property worth knowing — ORCA resolves who
//!    yields, not which *side* to pass on, and an exactly-tied configuration has no preferred answer.
//!    Break it with a nudge, jittered goals, or a preferred-side bias; do not expect the solver to
//!    invent one.
//! 2. **Five centimetres of lateral offset.** With the tie broken, reciprocity does its job: each side
//!    takes half the avoidance and they slide past. This is the ordinary case — exactly collinear
//!    approaches are measure-zero once anything in your game moves.
//! 3. **The neighbour holds ground** (`avoids: false`). The mover takes the *full* avoidance instead
//!    of assuming an idle unit will step aside and then walking through it.
//!
//! Run: `cargo run -p bevy_orca --example head_on`

use bevy_math::Vec2;
use bevy_orca::{new_velocity, Agent};

const DT: f32 = 1.0 / 30.0;
const MAX_SPEED: f32 = 1.5;
const TIME_HORIZON: f32 = 2.0;
const RADIUS: f32 = 0.5;
const STEPS: usize = 400;

struct Outcome {
    peak_a: f32,
    peak_b: f32,
    passed: bool,
    final_gap: f32,
}

/// Walk the pair toward each other's start position. `offset` displaces them laterally to break the
/// symmetry; `b_avoids` is whether the neighbour is running avoidance too.
///
/// When `b_avoids` is false, B is a genuinely idle unit: it neither avoids **nor moves**. That is the
/// situation the flag exists to describe, and solving for B anyway would quietly make it a cooperating
/// agent that merely claimed not to be.
fn run(offset: f32, b_avoids: bool, label: &str) -> Outcome {
    let mut a = Agent { pos: Vec2::new(-4.0, offset), vel: Vec2::ZERO, radius: RADIUS, avoids: true };
    let mut b = Agent { pos: Vec2::new(4.0, -offset), vel: Vec2::ZERO, radius: RADIUS, avoids: b_avoids };

    let (goal_a, goal_b) = (Vec2::new(4.0, offset), Vec2::new(-4.0, -offset));
    let (mut peak_a, mut peak_b) = (0.0f32, 0.0f32);
    let mut passed = false;

    println!("\n── {label} ──");
    println!("  step |   A.x    A.y  |   B.x    B.y  |  gap   | A lateral  B lateral");

    for step in 0..STEPS {
        let pref_a = (goal_a - a.pos).normalize_or_zero() * MAX_SPEED;
        let pref_b = (goal_b - b.pos).normalize_or_zero() * MAX_SPEED;

        // Each agent sees the other as its only neighbour. No walls in this scene.
        let va = new_velocity(&a, pref_a, &[b], &[], TIME_HORIZON, DT, MAX_SPEED);
        a.vel = va;
        a.pos += va * DT;

        // An idle unit is not solved for at all — it is standing there.
        if b_avoids {
            let vb = new_velocity(&b, pref_b, &[a], &[], TIME_HORIZON, DT, MAX_SPEED);
            b.vel = vb;
            b.pos += vb * DT;
        }

        peak_a = peak_a.max((a.pos.y - offset).abs());
        peak_b = peak_b.max((b.pos.y + offset).abs());
        if !passed && a.pos.x > b.pos.x {
            passed = true;
        }

        if step % 50 == 0 {
            println!(
                "  {step:>4} | {:>6.2} {:>6.2} | {:>6.2} {:>6.2} | {:>6.2} | {:>9.3}  {:>9.3}",
                a.pos.x,
                a.pos.y,
                b.pos.x,
                b.pos.y,
                a.pos.distance(b.pos),
                (a.pos.y - offset).abs(),
                (b.pos.y + offset).abs(),
            );
        }
    }
    Outcome { peak_a, peak_b, passed, final_gap: a.pos.distance(b.pos) }
}

fn report(o: &Outcome) {
    println!(
        "  peak sidestep: A {:.3}, B {:.3}   passed: {}   final gap {:.2}",
        o.peak_a, o.peak_b, o.passed, o.final_gap
    );
}

fn main() {
    let tied = run(0.0, true, "1. exactly collinear — the degenerate tie");
    report(&tied);
    println!(
        "  Neither stepped aside, and they stalled at a standoff rather than colliding. The\n\
         constraint was satisfied; there was simply no asymmetry to resolve it in either direction."
    );

    let split = run(0.05, true, "2. 5 cm of offset — reciprocity resolves it");
    report(&split);
    if split.peak_b > 1.0e-4 {
        println!(
            "  ratio A/B = {:.2} — ≈1.00 means each took half, which is the whole point of the\n\
             reciprocal formulation. Summed-force separation would have them shove equally and net zero.",
            split.peak_a / split.peak_b
        );
    }

    let uneven = run(0.05, false, "3. B holds ground (avoids: false)");
    report(&uneven);
    if split.peak_a > 1.0e-4 {
        println!(
            "  A sidestepped {:.2}× further than when B was cooperating, and B stayed put.\n\
             An idle unit is never assumed to move out of the way.",
            uneven.peak_a / split.peak_a
        );
    }
    println!(
        "  A also did not get *past* B here, and that is not a failure: ORCA is local avoidance, not\n\
         path planning. It keeps you off the obstacle; routing around one is your navigator's job."
    );
}
