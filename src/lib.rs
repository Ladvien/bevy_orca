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
//! - van den Berg, Guy, Lin & Manocha, "Reciprocal n-Body Collision Avoidance", Robotics Research
//!   (ISRR 2009), Springer STAR 70, 2011, DOI 10.1007/978-3-642-19457-3_1 (the ORCA half-plane
//!   construction + 2-D linear program below).
//!
//! The half-plane construction and the three-stage linear program follow the authors' reference
//! implementation, **RVO2** (van den Berg et al., Apache-2.0), closely enough that its names are kept
//! — `linear_program1`/`linear_program2`/`linear_program3`, `t_left`/`t_right`, and `RVO_EPSILON` as
//! `EPSILON` — so the two can be read side by side. See `NOTICE`. One numerical change: every line is
//! stored at its point nearest the origin, and `linear_program3` builds its projected lines from the
//! two lines' distances to the origin rather than through their intersection. The half-planes are
//! RVO2's; the answer stays inside the speed disc to `f32` rounding, which RVO2's does not (it can
//! leave it by many times `max_speed` when two constraints are nearly parallel).
//!
//! # Scope
//!
//! Agent↔agent avoidance is [`new_velocity_in`]; static geometry is [`Obstacles`]. Walls enter the
//! linear program as hard `walls` half-planes, either ones the caller computes (a door that opens, a
//! corridor's two sides) or the ones [`Obstacles::push_walls`] derives from polygons and wall
//! segments the caller built once: RVO2's obstacle lines, with its legs round convex corners. The
//! two compose; both are satisfied before any neighbour, and neither is ever relaxed. A global
//! navigator should still not route through solid space, and final contact is whatever the
//! caller's collision resolver says it is.

pub mod crowd;
pub mod obstacles;

pub use crowd::{Crowd, CrowdMember, CrowdParams, CrowdStep, CrowdWorker};
pub use obstacles::{ObstacleError, Obstacles, ObstaclesBuilder};

use bevy_math::Vec2;

/// Guard against division by near-zero / parallel-line degeneracies (RVO2's `RVO_EPSILON`).
const EPSILON: f32 = 1.0e-5;

/// A disc agent in the xz ground plane (y dropped): position, current velocity, radius.
///
/// `avoids` is whether this agent is *also* running avoidance this step. When a neighbor avoids
/// too (another commanded unit), the pair splits the avoidance (reciprocity): evenly, unless their
/// [`priority`](Self::priority) differs. When it does not (an idle unit holding ground), the moving
/// agent takes the **full** avoidance so the idle unit isn't assumed to step aside and get walked
/// through.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "reflect", derive(bevy_reflect::Reflect))]
pub struct Agent {
    pub pos: Vec2,
    pub vel: Vec2,
    pub radius: f32,
    pub avoids: bool,
    /// Right of way against a neighbour that also avoids: a ratio, so only comparisons mean anything.
    /// Of the dodge a collision course asks for, this agent takes `other / (this + other)` and the
    /// neighbour takes the rest, so the two shares add to 1 (the condition the collision-free
    /// guarantee needs) and both sides compute them from the same two numbers. Equal priorities are
    /// ORCA's even half; `3` against `1` takes a quarter and leaves the neighbour three quarters;
    /// `0` against anything above it takes all of it, and `∞` against anything finite takes none.
    /// A NaN or negative priority counts as `0`: the member gives way and never pushes harder.
    /// A neighbour with `avoids: false` is avoided in full whatever its priority, and walls always
    /// are. Discs that already overlap split the push apart evenly whatever their priorities: right of
    /// way is a claim on the way to avoid a collision, and touching discs have none to give. `1.0` is
    /// the default and, for a crowd that never changes it, plain ORCA.
    pub priority: f32,
}

impl Default for Agent {
    fn default() -> Self {
        Self { pos: Vec2::ZERO, vel: Vec2::ZERO, radius: 0.0, avoids: false, priority: 1.0 }
    }
}

/// The share of an avoidance an agent of priority `own` takes against a neighbour of priority
/// `other`, both of which step aside: `other / (own + other)`, so the pair's two shares add to 1.
///
/// Written as `1 / (1 + own / other)` because the sum overflows for two huge priorities and
/// `∞ / (∞ + x)` is NaN, where the ratio is `0`, finite or `∞` for every unequal pair; the two
/// equal-priority cases whose ratio would be NaN (`0 / 0`, `∞ / ∞`) are caught first and are the
/// plain half, which also keeps a crowd that never sets a priority on exactly the `0.5` it had.
/// `f32::max` ignores a NaN, so a priority that is NaN or negative becomes `0`.
///
/// The A* Pathfinding Project ships `other.priority / (this.priority + other.priority)`, and RVO's
/// generalised obstacle (van den Berg, Lin & Manocha, ICRA 2008, §III-C) asks only that the two
/// shares add to 1.
#[inline]
pub(crate) fn share(own: f32, other: f32) -> f32 {
    let (own, other) = (own.max(0.0), other.max(0.0));
    if own == other { 0.5 } else { 1.0 / (1.0 + own / other) }
}

/// A half-plane constraint `{ v : det(direction, v - point) >= 0 }` on the new velocity.
///
/// `direction` is a unit vector and `point` is the line's point nearest the origin. Every stage of the
/// linear program relies on both: a line stored through a far point (two nearly parallel lines meet
/// far away) makes `linear_program1`'s disc test subtract two huge, nearly equal squares, and its
/// answer `point + t · direction` the sum of two huge, nearly opposite vectors — both catastrophic
/// cancellation, which is how RVO2's answer leaves the speed disc. Stored at its foot, every number
/// the test and the answer use is no larger than the disc for any line that can cut it.
///
/// The one exception is a neighbour's line whose direction came out as zero (its relative velocity
/// sits within `EPSILON` of the cutoff circle's centre): `det(ZERO, ·)` is zero, so no velocity ever
/// violates it, and its point is the origin.
#[derive(Debug, Clone, Copy)]
struct Line {
    point: Vec2,
    direction: Vec2,
}

impl Line {
    /// The line through `point` along the unit `direction`, stored at its point nearest the origin:
    /// `det(direction, point)` is its signed distance from the origin, the one number that fixes it.
    #[inline]
    fn anchored(point: Vec2, direction: Vec2) -> Line {
        Line { point: det(direction, point) * direction.perp(), direction }
    }

    /// The line's signed distance from the origin, `det(direction, point)`.
    #[inline]
    fn offset(&self) -> f32 {
        det(self.direction, self.point)
    }
}

/// Reusable buffers for [`new_velocity_in`], so a caller solving for a crowd allocates nothing after
/// the first call.
///
/// Own one per solver (or per thread) and pass it in; it is cleared at the start of every call and
/// never shrinks, so it grows to the widest call the caller ever makes and stays there.
#[derive(Debug, Default)]
pub struct OrcaScratch {
    lines: Vec<Line>,
    proj_lines: Vec<Line>,
    /// The obstacle edges near one agent, `(squared distance, edge)`, for [`Obstacles::push_walls`].
    near_edges: Vec<(f32, u32)>,
}

/// 2-D cross product (signed area); `> 0` means `b` is left of `a`.
#[inline]
fn det(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Whether an agent's state can enter a solve at all: a finite position and velocity, and a radius
/// that is finite and not negative (`0` is a point). Recast/Detour refuses the same values with
/// `DT_INVALID_PARAM`; here an agent that fails it holds still, and a neighbour that fails it is
/// skipped. [`Crowd`] uses the same test to keep such a member out of everyone's neighbour list.
#[inline]
pub(crate) fn usable_state(pos: Vec2, vel: Vec2, radius: f32) -> bool {
    pos.is_finite() && vel.is_finite() && radius.is_finite() && radius >= 0.0
}

/// Whether `max_speed` leaves an agent a velocity it may take at all: positive, and its square, which
/// every line test uses as the speed disc's `radius²`, a normal `f32`. A square that overflowed (a
/// `max_speed` above about `1.8e19`) is not a number to compare with, and one that underflowed (below
/// about `1.1e-19`, where the square is subnormal or zero) can no longer tell a wish from the bound,
/// so a wish `1e-38` long would pass unclamped through a speed limit of `1e-40`. A NaN is not
/// positive. [`new_velocity_in`] answers [`Vec2::ZERO`] when this is false, and [`Crowd`] offers such
/// a member to its neighbours as one that will not step aside; they are one test so the two cannot
/// disagree.
#[inline]
pub(crate) fn usable_speed(max_speed: f32) -> bool {
    let square = max_speed * max_speed;
    max_speed > 0.0 && square.is_finite() && square >= f32::MIN_POSITIVE
}

/// `2^-96`: the power of two [`direction_of`] scales a vector by when its square is not a number.
const OVERFLOW_SCALE: f32 = f32::from_bits((127 - 96) << 23);

/// The unit vector along a finite `v`, or [`Vec2::ZERO`] if `v` is zero.
///
/// `normalize_or_zero` squares the components, so for a finite `v` longer than `sqrt(f32::MAX)` (about
/// `1.8e19`) the square is `+inf`, the reciprocal length is `0`, and the answer is [`Vec2::ZERO`]: a wish
/// to go that way as fast as possible would stop its agent (RVO2's `normalize` has the same hole). Such
/// a vector is scaled by an exact power of two first, as Blue's algorithm scales the large values in
/// LAPACK's `dnrm2`: a vector whose square overflows has a component of at least `2^63.5`, so after
/// the scale its squares lie between `2^-65` and `2^65`, inside `f32`, and the direction is unchanged.
/// Every `v` whose square is finite takes the unscaled path, so its result is the bits it always was.
#[inline]
fn direction_of(v: Vec2) -> Vec2 {
    if v.length_squared().is_finite() {
        v.normalize_or_zero()
    } else {
        (v * OVERFLOW_SCALE).normalize_or_zero()
    }
}

/// Compute the collision-avoiding velocity for `agent` nearest to `pref_vel`, given `neighbors`.
///
/// `time_horizon` is how many seconds ahead agent-agent collisions are anticipated (larger = more
/// cautious, earlier avoidance); `dt` is the simulation step (used for the in-contact case). The
/// result is never faster than `max_speed`, to `f32` rounding (a relative `1e-6`), for any finite
/// input — however dense the neighbours, however nearly parallel their constraints, however fast the
/// agent's own velocity: the speed disc is a constraint of the linear program, not a clamp after it.
/// With nothing in the way a `pref_vel` longer than `max_speed` is cut to it along its own direction,
/// however long it is — `(1e20, 0)` at a `max_speed` of `1.5` gives `(1.5, 0)`, not a stop. Pure
/// math — no panics.
///
/// Allocates a fresh line buffer per call. To solve repeatedly without allocating, own an
/// [`OrcaScratch`] and call [`new_velocity_in`].
pub fn new_velocity(
    agent: &Agent,
    pref_vel: Vec2,
    neighbors: &[Agent],
    walls: &[(Vec2, f32)],
    time_horizon: f32,
    dt: f32,
    max_speed: f32,
) -> Vec2 {
    new_velocity_in(
        &mut OrcaScratch::default(),
        agent,
        pref_vel,
        neighbors,
        walls,
        time_horizon,
        dt,
        max_speed,
    )
}

/// [`new_velocity`], reusing the buffers in `scratch`.
///
/// The result is always finite, whatever the input, because a NaN written into a caller's
/// `Transform` makes its agent vanish for good. Every input the solve cannot use gets one documented
/// answer, in this order:
///
/// - `max_speed` not finite, `<= 0`, so large that its square overflows `f32` (above about `1.8e19`),
///   or so small that its square is not a normal number (below about `1.1e-19`) → [`Vec2::ZERO`]:
///   there is no velocity this agent may take.
/// - `pref_vel` not finite → [`Vec2::ZERO`]: the preference itself is unusable.
/// - the agent's own `pos` or `vel` not finite, or its `radius` negative or not finite →
///   [`Vec2::ZERO`]: it has no velocity obstacle to solve with, so it holds still. A caller that
///   writes the result back has a usable velocity again next step.
/// - `dt` or `time_horizon` not finite, `<= 0`, or so small that its reciprocal overflows →
///   `pref_vel` clamped to `max_speed`: no time passes, so no constraint applies this step.
/// - a wall whose normal is not a unit vector ([`Vec2::is_normalized`]) or whose `approach` is not
///   finite → [`Vec2::ZERO`]: a wall is a hard constraint, and one that is not a half-plane cannot be
///   dropped, so the agent holds still rather than risk walking through it.
/// - a neighbour whose `pos` or `vel` is not finite, or whose `radius` is negative or not finite, is
///   skipped: it has no velocity obstacle. So is one whose constraint overflows `f32` relative to this
///   agent (agents near `±f32::MAX`), as if it were out of range.
/// - a `priority` that is NaN or negative, the agent's own or a neighbour's, counts as `0` (see
///   [`Agent::priority`]): that agent gives way, and no neighbour is ever skipped for it. An agent
///   whose `priority` is `∞` against a finite one takes none of the avoidance; the answer is still
///   finite.
///
/// A `radius` of `0` is a point, and usable. So is any finite `pref_vel`, including one too long to
/// square (above about `1.8e19`): it is clamped to the speed disc along its own direction. Finite,
/// in-range input passes every one of these tests, so none of them changes a single bit of a sane
/// solve. Whatever the input, a velocity this function returns is inside the speed disc to `f32`
/// rounding: the disc is a constraint of the linear program, not a clamp after it (see
/// [`new_velocity`]).
// A solver's parameters are what the caller decides per step, and bundling them into a struct would
// allocate a place to copy them into on every call — the opposite of this function's point. The eight
// arguments mirror RVO2's `computeNewVelocity` and `new_velocity`'s own seven.
#[allow(clippy::too_many_arguments)]
pub fn new_velocity_in(
    scratch: &mut OrcaScratch,
    agent: &Agent,
    pref_vel: Vec2,
    neighbors: &[Agent],
    walls: &[(Vec2, f32)],
    time_horizon: f32,
    dt: f32,
    max_speed: f32,
) -> Vec2 {
    // A NaN velocity written into a caller's transform makes its agent vanish for good, so every
    // degenerate input lands on a documented finite answer rather than a division by zero.
    // The speed disc's radius is squared in every line test, so a `max_speed` whose square is not a
    // number is as unusable as an infinite one.
    if !usable_speed(max_speed) {
        return Vec2::ZERO;
    }
    if !pref_vel.is_finite() {
        return Vec2::ZERO;
    }
    if !usable_state(agent.pos, agent.vel, agent.radius) {
        return Vec2::ZERO;
    }
    let inv_time_horizon = 1.0 / time_horizon;
    let inv_dt = 1.0 / dt;
    if !dt.is_finite()
        || dt <= 0.0
        || !time_horizon.is_finite()
        || time_horizon <= 0.0
        || !inv_dt.is_finite()
        || !inv_time_horizon.is_finite()
    {
        // No time passes this step (a subnormal step is as good as none), so nothing can be
        // constrained: take the wish, clamped to the speed disc like `linear_program2` clamps it.
        return if pref_vel.length_squared().is_finite() {
            pref_vel.clamp_length_max(max_speed)
        } else {
            direction_of(pref_vel) * max_speed
        };
    }

    // Split the two buffers out of the scratch so the LP can hold `lines` while filling `proj_lines`.
    let OrcaScratch { lines, proj_lines, .. } = scratch;
    lines.clear();
    lines.reserve(walls.len() + neighbors.len());

    // Wall half-planes come first as *hard* obstacle constraints (index 0..num_obstacle). Each is
    // `(b, approach)`: `b` is a unit vector from the agent toward a solid cell and `approach` is the
    // max speed at which the agent may still close on that wall this step (its remaining gap ÷ dt).
    // The constraint `v · b <= approach` lets the agent slide along or move away freely and coast up
    // to contact, but never accelerate *through* the wall — so ORCA won't dodge a neighbor by
    // steering into a wall and stalling. Actual contact is still the caller's collision resolver.
    for &(b, approach) in walls {
        // A hard constraint that is not a half-plane cannot be dropped like a neighbour: hold still.
        if !b.is_normalized() {
            return Vec2::ZERO;
        }
        // `is_normalized` allows `|b|` within about 1e-4 of 1; dividing it out makes the half-plane
        // exactly `v · b <= approach` and its direction a unit vector, as every stage of the program
        // assumes. The line is then at its foot already: `approach · b` is its point nearest the origin.
        let b = b / b.length();
        let point = approach * b;
        if !point.is_finite() {
            return Vec2::ZERO;
        }
        lines.push(Line { point, direction: b.perp() });
    }
    let num_obstacle = lines.len();

    for other in neighbors {
        // A neighbour with no usable state has no velocity obstacle: there is nothing to constrain. A
        // negative radius is the one such state that still builds a finite line, so it is refused
        // here; a position, velocity or radius that is not finite makes the line not finite, and the
        // line is refused below.
        if other.radius < 0.0 {
            continue;
        }
        let relative_position = other.pos - agent.pos;
        let relative_velocity = agent.vel - other.vel;
        let dist_sq = relative_position.length_squared();
        let combined_radius = agent.radius + other.radius;
        let combined_radius_sq = combined_radius * combined_radius;

        let u: Vec2;
        let direction: Vec2;

        // Whether the discs are apart: the branch below, and whether a right of way applies to the line.
        let apart = dist_sq > combined_radius_sq;
        if apart {
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

        // Reciprocity: split the change with neighbors that also avoid, by right of way (evenly when
        // the two priorities are equal); take it all against ones that don't (idle units holding
        // ground), whatever their priority. Right of way is a claim on the way to avoid a collision
        // that has not happened; discs that already overlap have no way to give, so both push apart
        // evenly. The shares still add to 1, which is all the guarantee asks, and a crowd that
        // crushes (a high priority on one member of a touching pair left the other to do all the
        // separating) no longer overlaps more for it.
        let responsibility = if !other.avoids {
            1.0
        } else if apart {
            share(agent.priority, other.priority)
        } else {
            0.5
        };
        let point = agent.vel + responsibility * u;
        // Stored at its point nearest the origin, so a constraint built from a fast own velocity does
        // not carry that speed into the disc test. Finite input can still overflow `f32` (two agents
        // near `±f32::MAX`, a radius near it): a constraint the arithmetic cannot represent is as far
        // out of reach as one out of range.
        let line = Line::anchored(point, direction);
        if !line.point.is_finite() || !direction.is_finite() {
            continue;
        }
        lines.push(line);
    }

    let mut result = pref_vel;
    let fail = linear_program2(lines, max_speed, pref_vel, false, &mut result);
    if fail < lines.len() {
        linear_program3(lines, num_obstacle, fail, max_speed, proj_lines, &mut result);
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

    if !discriminant.is_finite() || discriminant < 0.0 {
        // The max-speed disc doesn't intersect this line at all — infeasible. A discriminant that
        // overflowed or is NaN answers the same: an INF/NaN test means "no intersection", never
        // "everywhere" (RVO2 tests only `< 0`; for every finite discriminant the branch is RVO2's).
        return false;
    }

    let sqrt_discriminant = discriminant.sqrt();
    let mut t_left = -dot_product - sqrt_discriminant;
    let mut t_right = -dot_product + sqrt_discriminant;

    for other_line in lines.iter().take(line_no) {
        let denominator = det(line.direction, other_line.direction);
        let numerator = det(other_line.direction, line.point - other_line.point);

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
        *result = direction_of(opt_velocity) * radius;
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
///
/// `proj_lines` is the caller's reusable projection buffer: it is cleared at the start of every
/// violating line and never shrinks, so a repeated solve allocates nothing.
fn linear_program3(
    lines: &[Line],
    num_obstacle: usize,
    begin_line: usize,
    radius: f32,
    proj_lines: &mut Vec<Line>,
    result: &mut Vec2,
) {
    let mut distance = 0.0_f32;

    for i in begin_line.max(num_obstacle)..lines.len() {
        if det(lines[i].direction, lines[i].point - *result) > distance {
            // `result` violates line i by more than any seen so far; project the earlier lines onto
            // it and re-solve for the velocity that pushes constraint i's violation to a minimum. The
            // hard wall lines (0..num_obstacle) are always carried into the projection set.
            proj_lines.clear();
            proj_lines.extend_from_slice(&lines[0..num_obstacle]);
            let offset_i = lines[i].offset();
            for j in num_obstacle..i {
                let determinant = det(lines[i].direction, lines[j].direction);
                if determinant.abs() <= EPSILON && lines[i].direction.dot(lines[j].direction) > 0.0 {
                    // Lines i and j are parallel and point the same way — no new constraint.
                    continue;
                }
                // The projected line is where lines i and j are violated equally, along RVO2's
                // direction `d_j - d_i`: `det(d_j - d_i, v) = offset_j - offset_i`. RVO2 anchors it at
                // the two lines' intersection (at the midpoint of their points when they are
                // anti-parallel), which for nearly parallel lines lies up to `1 / EPSILON` away and
                // turns `linear_program1` into catastrophic cancellation. Built from the two offsets
                // instead, it is the same line on the same side, anchored at its point nearest the
                // origin. `d_j - d_i` is never zero here: equal directions are skipped above.
                let between = lines[j].direction - lines[i].direction;
                // RVO2 normalizes `d_j - d_i` (glam's `normalize` multiplies by the reciprocal of the
                // length, as here); the offset is scaled by the same reciprocal.
                let inv_length = 1.0 / between.length();
                let direction = between * inv_length;
                let offset = (lines[j].offset() - offset_i) * inv_length;
                proj_lines.push(Line { point: offset * direction.perp(), direction });
            }

            let temp = *result;
            let opt = Vec2::new(-lines[i].direction.y, lines[i].direction.x);
            if linear_program2(proj_lines.as_slice(), radius, opt, true, result) < proj_lines.len() {
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
        Agent { pos, vel, radius: 0.5, avoids: true, priority: 1.0 }
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

    /// A finite wish whose square overflows f32 is still a direction: it is clamped to the speed disc,
    /// not turned into a stop.
    #[test]
    fn a_huge_finite_wish_is_clamped_not_zeroed() {
        let a = agent(Vec2::ZERO, Vec2::ZERO);
        for dt in [1.0 / 60.0, 0.0] {
            let v = new_velocity(&a, Vec2::new(1.0e20, 0.0), &[], &[], 2.0, dt, 1.5);
            assert!((v - Vec2::new(1.5, 0.0)).length() < 1.0e-4, "dt {dt}: got {v}");
        }
    }

    /// A wish far outside the speed disc is answered the same however far outside it is: the answer for
    /// a wish too long to square is the answer for the same wish short enough to, with nothing in the
    /// way, with a neighbour, with a wall, with both, and with no time passing.
    #[test]
    fn a_wish_too_long_to_square_gives_the_answer_of_one_that_is_not() {
        let a = agent(Vec2::ZERO, Vec2::ZERO);
        let ahead = [agent(Vec2::new(2.0, 0.0), Vec2::new(-1.0, 0.0))];
        let wall = [(Vec2::Y, 0.4_f32)];
        let mut scratch = OrcaScratch::default();
        let max_speed = 1.5;
        for with_neighbour in [false, true] {
            for with_wall in [false, true] {
                for dt in [1.0 / 60.0, 0.0] {
                    let neighbours: &[Agent] = if with_neighbour { &ahead } else { &[] };
                    let walls: &[(Vec2, f32)] = if with_wall { &wall } else { &[] };
                    let dirs =
                        [Vec2::X, Vec2::Y, -Vec2::X, -Vec2::Y, Vec2::new(0.6, 0.8), Vec2::new(-0.8, -0.6)];
                    for dir in dirs {
                        // 1e10 squares to 1e20: the same direction, short enough for the old arithmetic.
                        let mut solve = |magnitude: f32| {
                            new_velocity_in(
                                &mut scratch,
                                &a,
                                dir * magnitude,
                                neighbours,
                                walls,
                                2.0,
                                dt,
                                max_speed,
                            )
                        };
                        let want = solve(1.0e10);
                        assert!(want.length() > 0.1, "the reference for {dir} is a stop: {want}");
                        for magnitude in [2.0e19, 1.0e20, 1.0e30, 3.0e38] {
                            let got = solve(magnitude);
                            assert!(got.is_finite() && got.length() <= max_speed * 1.0001, "got {got}");
                            assert!(
                                (got - want).length() < 1.0e-3,
                                "wish {dir} x {magnitude}, dt {dt}, neighbour {with_neighbour}, wall {with_wall}: \
                                 got {got}, want {want}"
                            );
                        }
                    }
                }
            }
        }
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

    /// An ordinary two-neighbour step the probe found on `0.2.1`: no wall, no unusual value, one
    /// neighbour holding ground and one stepping aside, and the answer was `(2.835, -1.333)` at a
    /// `max_speed` of 1.685 — 1.86 times the limit. The first line fails, the second is projected
    /// onto it by `linear_program3`, and that projected line used to be anchored at the lines'
    /// intersection, far from the speed disc.
    #[test]
    fn two_ordinary_neighbours_cannot_push_an_agent_over_its_speed_limit() {
        let me = Agent {
            pos: Vec2::new(-0.49547195, 3.4425201),
            vel: Vec2::new(0.56427115, -0.2730432),
            radius: 0.5298626,
            avoids: true,
            priority: 1.0,
        };
        let holding = Agent {
            pos: Vec2::new(-0.5121927, 4.374553),
            vel: Vec2::new(0.7615271, 0.7948744),
            radius: 0.54053575,
            avoids: false,
            priority: 1.0,
        };
        let stepping_aside = Agent {
            pos: Vec2::new(0.058371365, 1.7260351),
            vel: Vec2::new(0.04875728, -1.309605),
            radius: 0.48520732,
            avoids: true,
            priority: 1.0,
        };
        let max_speed = 1.6848793_f32;
        let v = new_velocity(
            &me,
            Vec2::new(-0.48186377, -1.2985785),
            &[holding, stepping_aside],
            &[],
            1.0464567,
            0.058656223,
            max_speed,
        );
        let speed = f64::from(v.x).hypot(f64::from(v.y));
        assert!(
            speed <= f64::from(max_speed) * (1.0 + 1.0e-6),
            "speed {speed} is {}× max_speed {max_speed}",
            speed / f64::from(max_speed),
        );
    }

    /// The speed disc is the one constraint every stage of the linear program is built inside, so the
    /// answer is never faster than `max_speed`, to `f32` rounding: in a dense knot of up to twelve
    /// neighbours, with two neighbours standing on the agent, with a row of neighbours whose
    /// constraints are nearly parallel (`linear_program3` projects lines that meet far away), and with
    /// an own velocity far beyond `max_speed`. Before the lines were stored at their point nearest the
    /// origin (0.2.1), a fraction of these answers left the disc: by several times on ordinary input
    /// and by a factor of 10⁷ from a fast own velocity.
    #[test]
    fn speed_bound_holds_in_dense_and_degenerate_constraint_sets() {
        use std::f32::consts::TAU;
        let mut scratch = OrcaScratch::default();
        let mut seed = 0u32;
        let mut next = || {
            seed = seed.wrapping_add(1);
            hash_f32(seed.wrapping_mul(0x9E37_79B1))
        };
        let mut neighbours = Vec::new();
        let mut walls = Vec::new();
        for case in 0..8_000u32 {
            let family = case % 4;
            let max_speed = 0.5 + 1.5 * next();
            let own_speed =
                if family == 3 { 10.0_f32.powf(1.0 + 14.0 * next()) } else { 1.2 * max_speed * next() };
            let me = Agent {
                pos: Vec2::new(10.0 * next() - 5.0, 10.0 * next() - 5.0),
                vel: Vec2::from_angle(TAU * next()) * own_speed,
                radius: 0.2 + 0.4 * next(),
                avoids: true,
                priority: 1.0,
            };
            neighbours.clear();
            for k in 0..(13.0 * next()) as usize {
                let offset = match family {
                    1 if k < 2 => Vec2::ZERO,
                    2 => Vec2::new(6.0 * next() - 3.0, 0.69 + 0.02 * next()),
                    _ => Vec2::from_angle(TAU * next()) * 3.0 * next(),
                };
                neighbours.push(Agent {
                    pos: me.pos + offset,
                    vel: Vec2::from_angle(TAU * next()) * 2.0 * next(),
                    radius: 0.2 + 0.4 * next(),
                    avoids: next() < 0.7,
                    priority: 1.0,
                });
            }
            walls.clear();
            for _ in 0..(3.0 * next()) as usize {
                walls.push((Vec2::from_angle(TAU * next()), 5.0 * next() - 1.0));
            }
            let pref = Vec2::from_angle(TAU * next()) * 3.0 * next();
            let horizon = 0.5 + 2.5 * next();
            let dt = 1.0 / 60.0 + 0.08 * next();
            let v = new_velocity_in(&mut scratch, &me, pref, &neighbours, &walls, horizon, dt, max_speed);
            let speed = f64::from(v.x).hypot(f64::from(v.y));
            assert!(
                speed <= f64::from(max_speed) * (1.0 + 1.0e-6),
                "case {case} (family {family}): speed {speed} is {}× max_speed {max_speed}",
                speed / f64::from(max_speed),
            );
        }
    }

    #[test]
    fn deterministic_for_identical_input() {
        let a = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, 0.3), Vec2::new(-1.0, 0.0));
        let call = || new_velocity(&a, Vec2::new(1.0, 0.0), &[b], &[], 2.0, 1.0 / 60.0, 1.5);
        assert_eq!(call(), call());
    }

    /// Every degenerate `dt` a paused frame, a first frame or a zero time-scale can hand in must give
    /// a finite velocity inside the speed disc, because a NaN in a caller's `Transform` is permanent.
    #[test]
    fn degenerate_dt_still_returns_a_finite_velocity() {
        let a = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0));
        let pref = Vec2::new(1.0, 0.5);
        let max_speed = 1.5;
        for dt in [0.0_f32, -1.0, f32::NAN, f32::INFINITY, 1.0e-40] {
            let v = new_velocity(&a, pref, &[b], &[], 2.0, dt, max_speed);
            assert!(v.is_finite(), "dt {dt} gave a non-finite velocity {v}");
            assert!(v.length() <= max_speed + 1.0e-4, "dt {dt}: speed {} > max {max_speed}", v.length());
        }
    }

    /// The same for the time horizon: zero, negative and non-finite all mean "no anticipation".
    #[test]
    fn degenerate_time_horizon_still_returns_a_finite_velocity() {
        let a = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0));
        let pref = Vec2::new(1.0, 0.5);
        let max_speed = 1.5;
        for h in [0.0_f32, -1.0, f32::NAN, f32::INFINITY, 1.0e-40] {
            let v = new_velocity(&a, pref, &[b], &[], h, 1.0 / 60.0, max_speed);
            assert!(v.is_finite(), "time_horizon {h} gave a non-finite velocity {v}");
            assert!(v.length() <= max_speed + 1.0e-4, "h {h}: speed {} > max {max_speed}", v.length());
        }
    }

    /// A bad `max_speed` means there is no velocity to take; a bad preference means none was asked for.
    #[test]
    fn degenerate_scalars_return_zero() {
        let a = agent(Vec2::ZERO, Vec2::ZERO);
        // A speed limit too small for its square to be a normal number is as unusable as one too
        // large for its square to be finite: a wish of 1e-38 would pass a limit of 1e-40 unclamped.
        for max_speed in [0.0_f32, -1.0, f32::NAN, f32::INFINITY, 1.0e20, f32::MAX, 1.0e-30, 1.0e-40] {
            let v = new_velocity(&a, Vec2::new(1.0, 0.0), &[], &[], 2.0, 1.0 / 60.0, max_speed);
            assert_eq!(v, Vec2::ZERO, "max_speed {max_speed} should give the zero velocity");
        }
        for pref in [Vec2::new(f32::NAN, 0.0), Vec2::new(0.0, f32::INFINITY), Vec2::NAN] {
            let v = new_velocity(&a, pref, &[], &[], 2.0, 1.0 / 60.0, 2.0);
            assert_eq!(v, Vec2::ZERO, "pref {pref} should give the zero velocity");
        }
    }

    /// `new_velocity_in` with a scratch must be the same arithmetic as `new_velocity`, bit for bit —
    /// otherwise reusing a buffer would be a behaviour change rather than an allocation saving.
    #[test]
    fn scratch_matches_allocating_path_bit_for_bit() {
        let max_speed = 2.0;
        let mut scratch = OrcaScratch::default();
        let walls = [(Vec2::new(1.0, 0.0), 0.4_f32)];
        for i in 0..40u32 {
            let hx = hash_f32(i.wrapping_mul(0x9E37_79B1)) * 4.0 - 2.0;
            let hy = hash_f32(i.wrapping_mul(0x85EB_CA6B).wrapping_add(1)) * 4.0 - 2.0;
            let a = agent(Vec2::ZERO, Vec2::new(1.0, 0.0));
            let b = agent(Vec2::new(hx, hy), Vec2::new(-0.5, 0.2));
            let pref = Vec2::new(1.5, 0.5);
            let plain = new_velocity(&a, pref, &[b], &walls, 2.0, 1.0 / 60.0, max_speed);
            let reused = new_velocity_in(&mut scratch, &a, pref, &[b], &walls, 2.0, 1.0 / 60.0, max_speed);
            assert_eq!(plain.x.to_bits(), reused.x.to_bits(), "config {i}: x differs");
            assert_eq!(plain.y.to_bits(), reused.y.to_bits(), "config {i}: y differs");
        }
    }

    /// An agent whose own position, velocity or radius is not usable has no velocity obstacle to
    /// solve with: it holds still (and a caller that writes `ZERO` back has a usable velocity again).
    #[test]
    fn an_agent_with_unusable_state_holds_still() {
        let b = agent(Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0));
        let walls = [(Vec2::Y, 1.0_f32)];
        let fine = agent(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0));
        let broken = [
            Agent { pos: Vec2::new(f32::NAN, 0.0), ..fine },
            Agent { pos: Vec2::new(0.0, f32::INFINITY), ..fine },
            Agent { vel: Vec2::new(f32::NAN, 0.0), ..fine },
            Agent { vel: Vec2::new(f32::NEG_INFINITY, 0.0), ..fine },
            Agent { radius: f32::NAN, ..fine },
            Agent { radius: f32::INFINITY, ..fine },
            Agent { radius: -0.1, ..fine },
        ];
        for a in broken {
            for neighbours in [&[][..], &[b][..]] {
                let v = new_velocity(&a, Vec2::X, neighbours, &walls, 2.0, 1.0 / 60.0, 1.5);
                assert_eq!(v, Vec2::ZERO, "{a:?} with {} neighbours should hold still", neighbours.len());
            }
        }
        // A radius of zero is a point agent, not a broken one.
        let point = Agent { radius: 0.0, ..fine };
        let v = new_velocity(&point, Vec2::X, &[], &walls, 2.0, 1.0 / 60.0, 1.5);
        assert_eq!(v, Vec2::X, "a point agent with nothing near keeps its wish");
    }

    /// A neighbour with no usable state, or one so far away that its constraint overflows `f32`, is
    /// skipped: the answer is bit-for-bit the answer without it, so one corrupted agent does not move
    /// anyone else.
    #[test]
    fn an_unusable_neighbour_is_skipped() {
        let a = agent(Vec2::new(-1.0, 0.05), Vec2::new(1.0, 0.0));
        let b = agent(Vec2::new(1.0, -0.05), Vec2::new(-1.0, 0.0));
        let walls = [(Vec2::Y, 1.0_f32)];
        let pref = Vec2::new(1.0, 0.0);
        let without = new_velocity(&a, pref, &[b], &walls, 2.0, 1.0 / 60.0, 1.5);
        let broken = [
            Agent { pos: Vec2::new(f32::NAN, 0.0), ..b },
            Agent { pos: Vec2::new(0.0, f32::NEG_INFINITY), ..b },
            Agent { vel: Vec2::new(f32::INFINITY, 0.0), ..b },
            Agent { vel: Vec2::NAN, ..b },
            Agent { radius: f32::NAN, ..b },
            Agent { radius: f32::INFINITY, ..b },
            Agent { radius: -0.5, ..b },
            // Finite, but its constraint overflows: a radius near `f32::MAX`.
            Agent { pos: Vec2::new(0.0, 0.5), radius: f32::MAX, ..b },
        ];
        for bad in broken {
            let with = new_velocity(&a, pref, &[b, bad], &walls, 2.0, 1.0 / 60.0, 1.5);
            assert_eq!(
                with.x.to_bits(),
                without.x.to_bits(),
                "{bad:?} moved the answer: {with} vs {without}"
            );
            assert_eq!(
                with.y.to_bits(),
                without.y.to_bits(),
                "{bad:?} moved the answer: {with} vs {without}"
            );
        }

        // Two agents at opposite ends of the float range: their relative position overflows, so
        // neither constrains the other.
        let far = Agent { pos: Vec2::new(f32::MAX, 0.0), ..a };
        let other_end = Agent { pos: Vec2::new(-f32::MAX, 0.0), ..b };
        let v = new_velocity(&far, pref, &[other_end], &[], 2.0, 1.0 / 60.0, 1.5);
        assert_eq!(v, pref, "a neighbour beyond f32's reach should not constrain anything");
    }

    /// A wall is a hard constraint, so one that is not a half-plane — a normal that is not a unit
    /// vector, an approach that is not finite — cannot be dropped: the agent holds still.
    #[test]
    fn an_unusable_wall_holds_the_agent_still() {
        let a = agent(Vec2::ZERO, Vec2::new(1.0, 0.0));
        let unusable = [
            (Vec2::new(f32::NAN, 0.0), 1.0_f32),
            (Vec2::new(0.0, f32::INFINITY), 1.0),
            (Vec2::ZERO, 1.0),
            (Vec2::new(2.0, 0.0), 1.0),
            (Vec2::new(0.707, 0.707), 1.0),
            (Vec2::X, f32::NAN),
            (Vec2::X, f32::INFINITY),
            (Vec2::X, f32::NEG_INFINITY),
        ];
        for wall in unusable {
            let v = new_velocity(&a, Vec2::X, &[], &[(Vec2::Y, 2.0), wall], 2.0, 1.0 / 60.0, 1.5);
            assert_eq!(v, Vec2::ZERO, "wall {wall:?} should hold the agent still");
        }
        // A normal that is a unit vector to f32 rounding is a wall, and it is enforced.
        let diagonal = Vec2::new(1.0, 1.0).normalize();
        let v = new_velocity(&a, Vec2::X, &[], &[(diagonal, 0.1)], 2.0, 1.0 / 60.0, 1.5);
        assert!(v.dot(diagonal) <= 0.1 + 1.0e-5, "a valid wall must still bind: {v}");
        assert_ne!(v, Vec2::ZERO);
    }

    /// Values a hostile or broken host can hand in, for every `f32` input: the non-finite ones, the
    /// signed zeros, a subnormal, values whose square or reciprocal overflows, and negatives.
    pub(crate) const HOSTILE: [f32; 17] = [
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
        0.0,
        -0.0,
        1.0e-40,
        f32::MIN_POSITIVE,
        1.0e-30,
        1.0e10,
        1.0e20,
        -1.0e20,
        1.0e30,
        f32::MAX,
        -f32::MAX,
        -1.0,
        -0.5,
        3.0e38,
    ];

    /// The property the whole crate promises a host: whatever it hands in — every field of the agent,
    /// its neighbours and walls, and every scalar — the answer is a finite velocity and never a panic.
    /// Each case makes one field hostile, or (every nineteenth case, the 16th) any field with a 30% chance.
    #[test]
    fn hostile_input_never_gives_a_non_finite_velocity() {
        let mut scratch = OrcaScratch::default();
        let mut seed = 0u32;
        let mut next = || {
            seed = seed.wrapping_add(1);
            hash_f32(seed.wrapping_mul(0x9E37_79B1))
        };
        for case in 0..20_000u32 {
            let which = case % 19;
            let chaos = which == 16;
            let take = |field: u32, sane: f32, roll: f32, pick: f32| -> f32 {
                if which == field || (chaos && roll < 0.3) {
                    HOSTILE[((pick * HOSTILE.len() as f32) as usize).min(HOSTILE.len() - 1)]
                } else {
                    sane
                }
            };
            let mut field = |f: u32, lo: f32, hi: f32| {
                let (sane, roll, pick) = (lo + (hi - lo) * next(), next(), next());
                take(f, sane, roll, pick)
            };
            let me = Agent {
                pos: Vec2::new(field(0, -5.0, 5.0), field(1, -5.0, 5.0)),
                vel: Vec2::new(field(2, -2.0, 2.0), field(3, -2.0, 2.0)),
                radius: field(4, 0.2, 0.6),
                avoids: true,
                priority: field(18, 0.1, 4.0),
            };
            let mut neighbours = [Agent::default(); 4];
            for (k, n) in neighbours.iter_mut().enumerate() {
                let base = if me.pos.is_finite() { me.pos } else { Vec2::ZERO };
                *n = Agent {
                    pos: base + Vec2::new(field(5, -2.0, 2.0), field(6, -2.0, 2.0)),
                    vel: Vec2::new(field(7, -2.0, 2.0), field(7, -2.0, 2.0)),
                    radius: field(8, 0.2, 0.6),
                    avoids: k % 2 == 0,
                    priority: field(18, 0.1, 4.0),
                };
                // Coincident agents: the exact degenerate the overlapping branch divides by.
                if which == 9 && k < 2 {
                    n.pos = me.pos;
                }
            }
            let angle = field(17, 0.0, std::f32::consts::TAU);
            let normal = Vec2::new(angle.cos(), angle.sin());
            let walls = [(
                Vec2::new(field(10, normal.x, normal.x), field(10, normal.y, normal.y)),
                field(11, -1.0, 4.0),
            )];
            let pref = Vec2::new(field(12, -2.0, 2.0), field(12, -2.0, 2.0));
            let horizon = field(13, 0.5, 3.0);
            let dt = field(14, 1.0 / 60.0, 0.1);
            let max_speed = field(15, 0.5, 2.0);
            let count = (case % 5) as usize;
            let v = new_velocity_in(
                &mut scratch,
                &me,
                pref,
                &neighbours[..count],
                &walls[..(case % 2) as usize],
                horizon,
                dt,
                max_speed,
            );
            assert!(
                v.is_finite(),
                "case {case}: {v} from {me:?}, pref {pref}, neighbours {:?}, walls {:?}, horizon {horizon}, dt {dt}, max_speed {max_speed}",
                &neighbours[..count],
                &walls[..(case % 2) as usize],
            );
        }
    }
}

// The README's quick start is compiled as a doctest, so it cannot drift from the API.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;
