//! Hand-rolled **ORCA** (Optimal Reciprocal Collision Avoidance) — the local, unit-vs-unit
//! avoidance layer beneath a global navigator (a flow field, a path follower). Given each agent's
//! *preferred* velocity (from the flow field), ORCA returns the collision-free velocity **closest**
//! to it, assuming every neighbor does the same reasoning and each side takes half the avoidance
//! (reciprocity). That reciprocity is what removes the oscillation and the mutual-cancel freeze of
//! the old summed-force separation: two units on a head-on course each step aside by half instead of
//! shoving equally and netting zero.
//!
//! References:
//! - van den Berg, Lin & Manocha, "Reciprocal Velocity Obstacles for Real-Time Multi-Agent
//!   Navigation", ICRA 2008, DOI 10.1109/ROBOT.2008.4543489 (reciprocity removes oscillation).
//! - van den Berg, Guy, Lin & Manocha, "Reciprocal n-Body Collision Avoidance", 2011,
//!   DOI 10.1109/TRO.2011.2120810 (the ORCA half-plane construction + 2-D linear program below).
//!
//! The half-plane construction and the three-stage linear program follow the authors' reference
//! implementation, **RVO2** (van den Berg et al., Apache-2.0), closely enough that its names are kept
//! — [`linear_program1`]/[`linear_program2`]/[`linear_program3`], `t_left`/`t_right`, and
//! `RVO_EPSILON` as [`EPSILON`] — so the two can be read side by side. See `NOTICE`.
//!
//! # Scope
//!
//! Only agent↔agent avoidance lives here. Static geometry is the caller's concern in two senses: a
//! global navigator should not route through solid space in the first place, and final contact is
//! whatever the caller's collision resolver says it is. Walls therefore enter only as the optional
//! `walls` half-planes described on [`new_velocity`] — the LP's *obstacle-line* count from static
//! geometry is otherwise zero.

use bevy_math::Vec2;

/// Guard against division by near-zero / parallel-line degeneracies (RVO2's `RVO_EPSILON`).
const EPSILON: f32 = 1.0e-5;

/// A disc agent in the xz ground plane (y dropped): position, current velocity, radius.
///
/// `avoids` is whether this agent is *also* running avoidance this step. When a neighbor avoids
/// too (another commanded unit), the pair splits the avoidance 50/50 (reciprocity). When it does
/// not (an idle unit holding ground), the moving agent takes the **full** avoidance so the idle
/// unit isn't assumed to step aside and get walked through.
#[derive(Clone, Copy)]
pub struct Agent {
    pub pos: Vec2,
    pub vel: Vec2,
    pub radius: f32,
    pub avoids: bool,
}

/// A half-plane constraint `{ v : det(direction, v - point) >= 0 }` on the new velocity.
#[derive(Clone, Copy)]
struct Line {
    point: Vec2,
    direction: Vec2,
}

/// 2-D cross product (signed area); `> 0` means `b` is left of `a`.
#[inline]
fn det(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Compute the collision-avoiding velocity for `agent` nearest to `pref_vel`, given `neighbors`.
///
/// `time_horizon` is how many seconds ahead agent-agent collisions are anticipated (larger = more
/// cautious, earlier avoidance); `dt` is the simulation step (used for the in-contact case); result
/// magnitude is clamped to `max_speed`. Pure math — no panics, no allocation beyond the line list.
pub fn new_velocity(
    agent: &Agent,
    pref_vel: Vec2,
    neighbors: &[Agent],
    walls: &[(Vec2, f32)],
    time_horizon: f32,
    dt: f32,
    max_speed: f32,
) -> Vec2 {
    let inv_time_horizon = 1.0 / time_horizon;
    let inv_dt = 1.0 / dt;

    let mut lines: Vec<Line> = Vec::with_capacity(walls.len() + neighbors.len());

    // Wall half-planes come first as *hard* obstacle constraints (index 0..num_obstacle). Each is
    // `(b, approach)`: `b` is a unit vector from the agent toward a solid cell and `approach` is the
    // max speed at which the agent may still close on that wall this step (its remaining gap ÷ dt).
    // The constraint `v · b <= approach` lets the agent slide along or move away freely and coast up
    // to contact, but never accelerate *through* the wall — so ORCA won't dodge a neighbor by
    // steering into a wall and stalling. Actual contact is still the caller's collision resolver.
    for &(b, approach) in walls {
        lines.push(Line {
            point: approach * b,
            direction: Vec2::new(-b.y, b.x),
        });
    }
    let num_obstacle = lines.len();

    for other in neighbors {
        let relative_position = other.pos - agent.pos;
        let relative_velocity = agent.vel - other.vel;
        let dist_sq = relative_position.length_squared();
        let combined_radius = agent.radius + other.radius;
        let combined_radius_sq = combined_radius * combined_radius;

        let u: Vec2;
        let direction: Vec2;

        if dist_sq > combined_radius_sq {
            // No collision yet. `w` is the vector from the cutoff-circle center to the relative velocity.
            let w = relative_velocity - inv_time_horizon * relative_position;
            let w_length_sq = w.length_squared();
            let dot1 = w.dot(relative_position);

            if dot1 < 0.0 && dot1 * dot1 > combined_radius_sq * w_length_sq {
                // Project on the cutoff circle.
                let w_length = w_length_sq.sqrt();
                let unit_w = if w_length > EPSILON { w / w_length } else { Vec2::ZERO };
                direction = Vec2::new(unit_w.y, -unit_w.x);
                u = (combined_radius * inv_time_horizon - w_length) * unit_w;
            } else {
                // Project on one of the velocity-obstacle legs.
                let leg = (dist_sq - combined_radius_sq).max(0.0).sqrt();
                if det(relative_position, w) > 0.0 {
                    // Left leg.
                    direction = Vec2::new(
                        relative_position.x * leg - relative_position.y * combined_radius,
                        relative_position.x * combined_radius + relative_position.y * leg,
                    ) / dist_sq;
                } else {
                    // Right leg.
                    direction = -Vec2::new(
                        relative_position.x * leg + relative_position.y * combined_radius,
                        -relative_position.x * combined_radius + relative_position.y * leg,
                    ) / dist_sq;
                }
                let dot2 = relative_velocity.dot(direction);
                u = dot2 * direction - relative_velocity;
            }
        } else {
            // Already overlapping. Push apart along the cutoff circle of the current time step.
            let w = relative_velocity - inv_dt * relative_position;
            let w_length = w.length();
            let unit_w = if w_length > EPSILON { w / w_length } else { Vec2::ZERO };
            direction = Vec2::new(unit_w.y, -unit_w.x);
            u = (combined_radius * inv_dt - w_length) * unit_w;
        }

        // Reciprocity: split the change with neighbors that also avoid; take it all against ones
        // that don't (idle units holding ground).
        let responsibility = if other.avoids { 0.5 } else { 1.0 };
        lines.push(Line {
            point: agent.vel + responsibility * u,
            direction,
        });
    }

    let mut result = pref_vel;
    let fail = linear_program2(&lines, max_speed, pref_vel, false, &mut result);
    if fail < lines.len() {
        linear_program3(&lines, num_obstacle, fail, max_speed, &mut result);
    }
    result
}

/// Optimize `result` along a single line `lines[line_no]` subject to the speed disc and the earlier
/// half-planes. Returns `false` if the constraints are infeasible for this line.
fn linear_program1(
    lines: &[Line],
    line_no: usize,
    radius: f32,
    opt_velocity: Vec2,
    direction_opt: bool,
    result: &mut Vec2,
) -> bool {
    let line = lines[line_no];
    let dot_product = line.point.dot(line.direction);
    let discriminant = dot_product * dot_product + radius * radius - line.point.length_squared();

    if discriminant < 0.0 {
        // The max-speed disc doesn't intersect this line at all — infeasible.
        return false;
    }

    let sqrt_discriminant = discriminant.sqrt();
    let mut t_left = -dot_product - sqrt_discriminant;
    let mut t_right = -dot_product + sqrt_discriminant;

    for i in 0..line_no {
        let denominator = det(line.direction, lines[i].direction);
        let numerator = det(lines[i].direction, line.point - lines[i].point);

        if denominator.abs() <= EPSILON {
            // Lines are (almost) parallel.
            if numerator < 0.0 {
                return false;
            }
            continue;
        }

        let t = numerator / denominator;
        if denominator >= 0.0 {
            t_right = t_right.min(t);
        } else {
            t_left = t_left.max(t);
        }
        if t_left > t_right {
            return false;
        }
    }

    if direction_opt {
        // Optimize direction: take the extreme feasible point in the preferred heading.
        if opt_velocity.dot(line.direction) > 0.0 {
            *result = line.point + t_right * line.direction;
        } else {
            *result = line.point + t_left * line.direction;
        }
    } else {
        // Optimize closest to `opt_velocity`.
        let t = line.direction.dot(opt_velocity - line.point);
        if t < t_left {
            *result = line.point + t_left * line.direction;
        } else if t > t_right {
            *result = line.point + t_right * line.direction;
        } else {
            *result = line.point + t * line.direction;
        }
    }
    true
}

/// Find the velocity within the speed disc closest to `opt_velocity` satisfying all `lines`.
/// Returns `lines.len()` on success, or the index of the first infeasible line.
fn linear_program2(
    lines: &[Line],
    radius: f32,
    opt_velocity: Vec2,
    direction_opt: bool,
    result: &mut Vec2,
) -> usize {
    if direction_opt {
        *result = opt_velocity * radius;
    } else if opt_velocity.length_squared() > radius * radius {
        *result = opt_velocity.normalize_or_zero() * radius;
    } else {
        *result = opt_velocity;
    }

    for i in 0..lines.len() {
        if det(lines[i].direction, lines[i].point - *result) > 0.0 {
            // `result` violates constraint i; re-optimize on that line.
            let temp = *result;
            if !linear_program1(lines, i, radius, opt_velocity, direction_opt, result) {
                *result = temp;
                return i;
            }
        }
    }
    lines.len()
}

/// Dense/infeasible fallback: when no velocity satisfies every agent half-plane, minimize the maximum
/// agent-constraint violation (the "safest" velocity). This is ORCA's defined behavior for
/// over-constrained crowds, not a feature fallback. The first `num_obstacle` lines are hard wall
/// constraints kept in every projection and never relaxed (an agent yields into a neighbor before it
/// walks through a wall). `begin_line` is the first line that failed in `linear_program2`.
fn linear_program3(lines: &[Line], num_obstacle: usize, begin_line: usize, radius: f32, result: &mut Vec2) {
    let mut distance = 0.0_f32;

    for i in begin_line.max(num_obstacle)..lines.len() {
        if det(lines[i].direction, lines[i].point - *result) > distance {
            // `result` violates line i by more than any seen so far; project the earlier lines onto
            // it and re-solve for the velocity that pushes constraint i's violation to a minimum. The
            // hard wall lines (0..num_obstacle) are always carried into the projection set.
            let mut proj_lines: Vec<Line> = lines[0..num_obstacle].to_vec();
            for j in num_obstacle..i {
                let determinant = det(lines[i].direction, lines[j].direction);
                let point = if determinant.abs() <= EPSILON {
                    if lines[i].direction.dot(lines[j].direction) > 0.0 {
                        // Same direction — no new constraint.
                        continue;
                    }
                    0.5 * (lines[i].point + lines[j].point)
                } else {
                    lines[i].point
                        + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant)
                            * lines[i].direction
                };
                let direction = (lines[j].direction - lines[i].direction).normalize_or_zero();
                proj_lines.push(Line { point, direction });
            }

            let temp = *result;
            let opt = Vec2::new(-lines[i].direction.y, lines[i].direction.x);
            if linear_program2(&proj_lines, radius, opt, true, result) < proj_lines.len() {
                // Should not happen except through floating-point error; keep the safer prior result.
                *result = temp;
            }
            distance = det(lines[i].direction, lines[i].point - *result);
        }
    }
}

#[cfg(test)]
mod tests {
    // Pure ORCA math — no App, no ECS. Locks the reciprocal-velocity-obstacle contract (van den Berg
    // et al. 2011): the returned velocity is always inside the speed disc and, given an obstructing
    // neighbour, diverges from the preferred velocity (it actually avoids).
    use super::*;

    fn agent(pos: Vec2, vel: Vec2) -> Agent {
        Agent { pos, vel, radius: 0.5, avoids: true }
    }

    /// A deterministic `u32 -> [0,1)` hash (PCG-style output mix), inlined here so the sweep below
    /// needs neither an RNG crate nor an entropy source.
    ///
    /// It is a copy of the parent game's `util::hash_f32` **by construction, not by dependency**:
    /// this crate must not grow a dependency to seed a test, and the value only has to be
    /// *reproducible*, not *the same number* as any other caller's — nothing crosses the boundary.
    fn hash_f32(x: u32) -> f32 {
        let mut h = x.wrapping_mul(747_796_405).wrapping_add(2_891_336_453);
        h = ((h >> ((h >> 28).wrapping_add(4))) ^ h).wrapping_mul(277_803_737);
        h = (h >> 22) ^ h;
        (h as f32) / (u32::MAX as f32)
    }

    #[test]
    fn free_agent_keeps_its_preferred_velocity() {
        // No neighbours, no walls: ORCA has nothing to constrain, so it returns pref_vel unchanged.
        let a = agent(Vec2::ZERO, Vec2::ZERO);
        let pref = Vec2::new(1.0, 0.0);
        let v = new_velocity(&a, pref, &[], &[], 2.0, 1.0 / 60.0, 2.0);
        assert!((v - pref).length() < 1.0e-4, "expected {pref}, got {v}");
    }

    #[test]
    fn output_is_clamped_to_max_speed() {
        // A preferred velocity faster than max_speed is projected onto the speed disc.
        let a = agent(Vec2::ZERO, Vec2::ZERO);
        let max_speed = 1.5;
        let v = new_velocity(&a, Vec2::new(10.0, 0.0), &[], &[], 2.0, 1.0 / 60.0, max_speed);
        assert!(v.length() <= max_speed + 1.0e-4, "speed {} exceeds max {max_speed}", v.length());
        assert!((v.length() - max_speed).abs() < 1.0e-3, "should saturate the disc, got {}", v.length());
    }

    #[test]
    fn head_on_pair_deflects_instead_of_colliding() {
        // A and B close head-on. ORCA must return something other than "barrel straight ahead", and it
        // must stay within the speed disc. (Symmetry makes the deflection SIDE ambiguous, so we assert
        // divergence + the speed bound, not a specific y sign.)
        let a = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0));
        let pref = Vec2::new(1.0, 0.0);
        let max_speed = 1.5;
        let v = new_velocity(&a, pref, &[b], &[], 2.0, 1.0 / 60.0, max_speed);
        assert!(v.length() <= max_speed + 1.0e-4);
        assert!((v - pref).length() > 1.0e-3, "expected avoidance, but velocity stayed at pref {pref}");
    }

    #[test]
    fn speed_bound_holds_across_many_configurations() {
        // Property: for any neighbour placement the result never exceeds max_speed (the disc is a hard
        // constraint carried through every projection). Deterministic sweep — no RNG crate.
        let max_speed = 2.0;
        for i in 0..40u32 {
            // Deterministic pseudo-positions from a cheap hash, no entropy source.
            let hx = hash_f32(i.wrapping_mul(0x9E37_79B1)) * 4.0 - 2.0;
            let hy = hash_f32(i.wrapping_mul(0x85EB_CA6B).wrapping_add(1)) * 4.0 - 2.0;
            let a = agent(Vec2::ZERO, Vec2::new(1.0, 0.0));
            let b = agent(Vec2::new(hx, hy), Vec2::new(-0.5, 0.2));
            let v = new_velocity(&a, Vec2::new(1.5, 0.5), &[b], &[], 2.0, 1.0 / 60.0, max_speed);
            assert!(v.is_finite(), "non-finite velocity for config {i}: {v}");
            assert!(v.length() <= max_speed + 1.0e-4, "config {i}: speed {} > max {max_speed}", v.length());
        }
    }

    #[test]
    fn deterministic_for_identical_input() {
        let a = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, 0.3), Vec2::new(-1.0, 0.0));
        let call = || new_velocity(&a, Vec2::new(1.0, 0.0), &[b], &[], 2.0, 1.0 / 60.0, 1.5);
        assert_eq!(call(), call());
    }
}
