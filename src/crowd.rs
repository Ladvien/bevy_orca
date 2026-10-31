//! **A whole crowd, solved as one batch.**
//!
//! [`new_velocity_in`] solves one agent against a neighbour slice the caller assembled. Every host
//! that owns a crowd then writes the same four things around it — a spatial hash to find neighbours
//! (an O(n²) scan does not survive a few hundred agents), a reused [`OrcaScratch`], a tie-break that
//! is deterministic when two agents want the same space, and the wall buffer. This module is that
//! ring, once, so a host learns it once.
//!
//! The caller still decides the two things that are gameplay: **who is a neighbour**
//! (`solve`'s `sees`) and **what is solid** (its `walls`). This module never looks at a game type.

use bevy_math::Vec2;

use crate::{Agent, OrcaScratch, new_velocity_in, usable_speed, usable_state};

/// One member of a crowd, as the batch solver sees it.
///
/// `Vec2` is the host's ground plane, in the host's units: the crate never converts.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "reflect", derive(bevy_reflect::Reflect))]
pub struct CrowdMember {
    /// The caller's stable id for this member. It breaks a tie between two equidistant neighbours, so an
    /// identical world solves identically however the members are ordered in the input slice. It has no
    /// part in which side a member passes on: that is [`CrowdParams::nudge`]'s, the same for everyone.
    pub key: u64,
    /// Position on the ground plane.
    pub pos: Vec2,
    /// Current velocity.
    pub vel: Vec2,
    /// Body radius.
    pub radius: f32,
    /// Speed ceiling for the solved velocity. A ceiling that is not finite, not positive, so large
    /// that its square overflows `f32` (above about `1.8e19`), or so small that its square is not a
    /// normal number (below about `1.1e-19`) leaves the member no velocity to take: it is solved to
    /// [`Vec2::ZERO`], and its neighbours take the full avoidance against it.
    pub max_speed: f32,
    /// Where this member wants to go.
    ///
    /// `None` means another system moves this member this step: it is a neighbour to everyone else and
    /// is never solved for. Its velocity is still read, so the agent that is moving it is avoided, and
    /// in full: a member this step does not move is not assumed to step aside. A wish that is not
    /// finite is the same as `None`; a finite one is always a wish, however long — one too long to
    /// square (above `1.8e19`) is a direction at full speed, not a stop.
    pub pref_vel: Option<Vec2>,
}

/// How a crowd is solved. There is no [`Default`]: [`CrowdParams::neighbour_range`] is in the caller's
/// own units, so a default would be a number from nowhere.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "reflect", derive(bevy_reflect::Reflect))]
pub struct CrowdParams {
    /// Seconds of collision lookahead. Larger is more cautious and earlier; too large stalls a crowd
    /// by braking for a congestion that has not happened yet.
    pub time_horizon: f32,
    /// Only members this close are neighbours, in the host's units. It is also the width of the
    /// spatial-hash cell, so a much larger value costs more candidates per lookup.
    pub neighbour_range: f32,
    /// At most this many neighbours per member, nearest first.
    pub max_neighbours: usize,
    /// Sideways kick, as a fraction of the wish, applied when a member has any neighbour at all. It
    /// breaks the exact symmetry of a head-on meeting, which the linear program has no reason to
    /// prefer either way and will otherwise stall, and it is **one side for the whole crowd**: a
    /// positive value kicks every member toward the right of its own wish (clockwise in the plane the
    /// members are given in, the side ORCA's own tie-break passes on), a negative one toward the left.
    /// Two members that meet head-on are then kicked past each other, and a crowd in motion sorts
    /// itself into one stream per direction instead of leaving half its head-on pairs unbroken. A kick
    /// the wish cannot take — the kicked wish is not finite, as for a wish near `f32::MAX`, or `nudge`
    /// is NaN — is not applied.
    pub nudge: f32,
}

impl CrowdParams {
    /// Parameters for a crowd whose neighbours are within `neighbour_range` of each other: a 2-second
    /// horizon, at most 10 neighbours and a 5% nudge.
    pub fn new(neighbour_range: f32) -> Self {
        Self { time_horizon: 2.0, neighbour_range, max_neighbours: 10, nudge: 0.05 }
    }
}

/// A reusable crowd solver: the neighbour grid, the line buffers and the output slice.
///
/// Own one per crowd and call [`solve`](Self::solve) every tick. Nothing in it is per-member state, so
/// it is safe to solve several crowds with one `Crowd`, one after the other. Its buffers grow to the
/// largest crowd it has solved and never shrink, so once warm it allocates nothing, wherever the crowd
/// walks and however tightly it packs.
#[derive(Debug, Default)]
pub struct Crowd {
    scratch: OrcaScratch,
    /// The neighbour grid: `(cell key, member index)` for every member that can be a neighbour (finite
    /// position and velocity, finite non-negative radius), sorted,
    /// `neighbour_range` per cell. Rebuilt every solve, so its size is the crowd's and not the ground
    /// the crowd has covered. Lookup only: every candidate list is sorted before use.
    cells: Vec<(u64, usize)>,
    /// Whether [`steers`] holds for each member, filled in the same pass as the grid. A neighbour's
    /// `avoids` is read from here, so the test runs once per member and not once per neighbour offered.
    steered: Vec<bool>,
    /// Reused candidate buffer: `(distance, key, index)`, cut to the nearest `max_neighbours` and sorted
    /// nearest-first before use.
    near: Vec<(f32, u64, usize)>,
    /// Reused neighbour slice handed to the solver.
    neighbours: Vec<Agent>,
    /// Reused wall buffer, filled by the caller's `walls` closure.
    walls: Vec<(Vec2, f32)>,
    /// Reused output, indexed like the members slice.
    out: Vec<Option<Vec2>>,
}

/// One cell of the neighbour grid as `(column, row)`, or `None` when the position is not finite. A
/// position beyond `±range · 2^31` saturates to the edge cell; the distance test rejects the members
/// that share it without being in range.
fn cell_of(pos: Vec2, range: f32) -> Option<(i32, i32)> {
    if !pos.is_finite() || range.is_nan() || range <= 0.0 {
        return None;
    }
    Some(((pos.x / range).floor() as i32, (pos.y / range).floor() as i32))
}

/// A cell's place in the grid's order: row-major, each coordinate offset from `i32::MIN` so the order
/// of keys is the order of `(row, column)`. The cells of one row then sit together, and a run of
/// adjacent columns is one contiguous key range. Both arguments must lie in the `i32` range.
fn cell_key(column: i64, row: i64) -> u64 {
    let offset = |v: i64| (v - i64::from(i32::MIN)) as u64;
    (offset(row) << 32) | offset(column)
}

/// The order candidates are kept in: distance, then the caller's key, then the member's index. Each
/// index appears once in a candidate list, so no two candidates compare equal: the order is strict and
/// total, and an unstable selection or sort gives the one result a stable sort would.
fn nearest_first(a: &(f32, u64, usize), b: &(f32, u64, usize)) -> core::cmp::Ordering {
    a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2))
}

/// Candidate lists this short or shorter are sorted whole and cut, never selected from:
/// `select_nth_unstable` insertion-sorts a slice of up to 16 in full anyway, so selecting first only
/// adds a second pass (measured: about 10% dearer than one sort for 11 to 14 candidates).
const SELECT_ABOVE: usize = 16;

/// Cut `near` to its nearest `keep` candidates, in [`nearest_first`] order. A list longer than
/// [`SELECT_ABOVE`] and than twice `keep` is first narrowed to its nearest `keep` by an in-place
/// selection, O(candidates), so only those are sorted; selecting from a list barely longer than `keep`
/// discards too little to pay for itself. Neither step allocates, and as no two candidates compare
/// equal the result is the one a stable sort and a cut would give.
fn nearest(near: &mut Vec<(f32, u64, usize)>, keep: usize) {
    if keep == 0 {
        near.clear();
        return;
    }
    if near.len() > SELECT_ABOVE.max(keep.saturating_mul(2)) {
        near.select_nth_unstable_by(keep, nearest_first);
        near.truncate(keep);
    }
    near.sort_unstable_by(nearest_first);
    near.truncate(keep);
}

/// The wish [`Crowd::solve`] steers `m` by, or `None` when it leaves `m` unsolved: no
/// [`pref_vel`](CrowdMember::pref_vel), one that is not finite, or a position that is not finite.
#[inline]
fn wish(m: &CrowdMember) -> Option<Vec2> {
    m.pref_vel.filter(|pref| pref.is_finite() && m.pos.is_finite())
}

/// Whether [`Crowd::solve`] picks a velocity for `m` itself: it has a [`wish`] and a speed to move at.
/// This is what a neighbour's [`Agent::avoids`] has to mean. Reciprocity gives each side half of the
/// avoidance only because the other side takes the other half, so a member this step will not move
/// — no wish, a wish that is not finite, or a `max_speed` that leaves it no velocity, which
/// `new_velocity_in` answers with [`Vec2::ZERO`] — is avoided in full. The solve's skip and the
/// flag both come from here, so they cannot disagree.
#[inline]
fn steers(m: &CrowdMember) -> bool {
    usable_speed(m.max_speed) && wish(m).is_some()
}

impl Crowd {
    /// Solve every steered member of `members` for one step of `dt`.
    ///
    /// The returned slice is indexed like `members`. `None` means that member was not solved: it has no
    /// finite [`pref_vel`](CrowdMember::pref_vel), or its position is not finite. Every other steered
    /// member is solved by [`new_velocity_in`], so its answer to unusable input is that function's: a
    /// member whose velocity or radius is unusable (not finite, or a negative radius) is solved to
    /// [`Vec2::ZERO`] and holds still. A member whose position, velocity or radius is unusable is
    /// nobody's neighbour, and takes none of the `max_neighbours` places.
    ///
    /// Reciprocity is assumed only of a neighbour this step moves. One with no `pref_vel`, a
    /// `pref_vel` that is not finite, or a [`max_speed`](CrowdMember::max_speed) that leaves it no
    /// velocity will not step aside, so each member near it takes the *full* avoidance against it;
    /// every other neighbour takes half. (A member whose `walls` are unusable holds still too, but
    /// only its `walls` closure can say so and that runs after the neighbours are built, so it is still
    /// offered as stepping aside.)
    ///
    /// `sees(i, j)` decides whether member `i` treats member `j` as a neighbour at all — the host's
    /// gameplay rule (same storey, same team, can see through a door). `walls(i, &mut buf)` fills
    /// member `i`'s wall half-planes, exactly as [`new_velocity_in`] takes them; the buffer it is
    /// handed is cleared first and reused afterwards.
    pub fn solve(
        &mut self,
        members: &[CrowdMember],
        params: &CrowdParams,
        dt: f32,
        mut sees: impl FnMut(usize, usize) -> bool,
        mut walls: impl FnMut(usize, &mut Vec<(Vec2, f32)>),
    ) -> &[Option<Vec2>] {
        let range = params.neighbour_range;
        let search = range > 0.0 && range.is_finite();

        // Rebuild the grid: one entry per member that can be a neighbour, sorted by cell. The pairs are
        // unique, so the sorted order is fully determined, and the buffer is cleared rather than
        // dropped, so a warm `Crowd` allocates nothing here however far its members have walked. A
        // member with no usable state has no velocity obstacle, so it is left out here rather than
        // taking a neighbour's place only to be skipped by the solver.
        self.cells.clear();
        self.steered.clear();
        if search {
            for (i, m) in members.iter().enumerate() {
                self.steered.push(steers(m));
                if !usable_state(m.pos, m.vel, m.radius) {
                    continue;
                }
                if let Some((column, row)) = cell_of(m.pos, range) {
                    self.cells.push((cell_key(i64::from(column), i64::from(row)), i));
                }
            }
            self.cells.sort_unstable();
        }

        self.out.clear();
        self.out.resize(members.len(), None);

        for i in 0..members.len() {
            let m = members[i];
            let Some(pref) = wish(&m) else {
                continue;
            };

            // Neighbours: every member within `range` that this one sees, nearest first, at most
            // `max_neighbours` of them. Ordering by `(distance, key)` is what makes the result a
            // property of the world rather than of the input slice's order.
            self.near.clear();
            if search && let Some((cx, cy)) = cell_of(m.pos, range) {
                // The 3×3 block around this member's cell, one row at a time: a row's three cells are
                // one key range. Neighbouring rows and columns are taken in `i64` and kept inside the
                // `i32` range, so a member in a saturated edge cell looks only at real cells.
                let (lowest, highest) = (i64::from(i32::MIN), i64::from(i32::MAX));
                let (cx, cy) = (i64::from(cx), i64::from(cy));
                let (first_column, last_column) = ((cx - 1).max(lowest), (cx + 1).min(highest));
                for row in (cy - 1).max(lowest)..=(cy + 1).min(highest) {
                    let (from, to) = (cell_key(first_column, row), cell_key(last_column, row));
                    let start = self.cells.partition_point(|&(key, _)| key < from);
                    let end = self.cells.partition_point(|&(key, _)| key <= to);
                    let Some(run) = self.cells.get(start..end) else {
                        continue;
                    };
                    for &(_, j) in run {
                        if j == i {
                            continue;
                        }
                        let other = members[j];
                        let dist = other.pos.distance(m.pos);
                        if dist > range {
                            continue;
                        }
                        if sees(i, j) {
                            self.near.push((dist, other.key, j));
                        }
                    }
                }
            }
            // Only the nearest `max_neighbours` are used, so only they are ordered; a packed crowd
            // offers hundreds of candidates, and a stable sort would take a heap scratch for each
            // member above 170 of them.
            nearest(&mut self.near, params.max_neighbours);

            self.neighbours.clear();
            for &(_, _, j) in &self.near {
                let other = members[j];
                self.neighbours.push(Agent {
                    pos: other.pos,
                    vel: other.vel,
                    radius: other.radius,
                    // A member this step will not move is not assumed to step aside, so this one
                    // takes the full avoidance against it instead of walking through it.
                    avoids: self.steered.get(j).is_some_and(|&steered| steered),
                });
            }

            // The wish, kicked toward the right of its own direction when anyone is near: the same side
            // for every member, so two agents meeting head-on are kicked past each other and a stream
            // keeps to one side. A kick the wish cannot take — the sum is not finite, as for a wish near
            // `f32::MAX` — is not applied: such a wish is a direction already, and `new_velocity_in`
            // would refuse the non-finite sum.
            let wish = if self.near.is_empty() {
                pref
            } else {
                let kicked = pref - pref.perp() * params.nudge;
                if kicked.is_finite() { kicked } else { pref }
            };

            self.walls.clear();
            walls(i, &mut self.walls);

            let me = Agent { pos: m.pos, vel: m.vel, radius: m.radius, avoids: true };
            let solved = new_velocity_in(
                &mut self.scratch,
                &me,
                wish,
                &self.neighbours,
                &self.walls,
                params.time_horizon,
                dt,
                m.max_speed,
            );
            self.out[i] = Some(solved);
        }

        &self.out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn member(key: u64, pos: Vec2, vel: Vec2, pref: Option<Vec2>) -> CrowdMember {
        CrowdMember { key, pos, vel, radius: 0.5, max_speed: 1.5, pref_vel: pref }
    }

    /// The SplitMix64 output mix: the tests' own stable pseudo-random numbers.
    fn splitmix64(mut x: u64) -> u64 {
        x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = x;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Two members walking at each other must not pass through one another: reciprocity divides the
    /// avoidance, and this is what "divided" has to mean in metres.
    #[test]
    fn head_on_members_keep_their_distance() {
        let params = CrowdParams::new(4.0);
        let mut crowd = Crowd::default();
        let (mut a, mut b) = (
            member(1, Vec2::new(-2.0, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X)),
            member(2, Vec2::new(2.0, 0.0), Vec2::new(-1.0, 0.0), Some(-Vec2::X)),
        );
        let dt = 1.0 / 60.0;
        let mut min_gap = f32::INFINITY;
        for _ in 0..240 {
            let members = [a, b];
            let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());
            if let Some(v) = solved[0] {
                a.vel = v;
            }
            if let Some(v) = solved[1] {
                b.vel = v;
            }
            a.pos += a.vel * dt;
            b.pos += b.vel * dt;
            min_gap = min_gap.min(a.pos.distance(b.pos));
        }
        let sum_radii = a.radius + b.radius;
        assert!(
            min_gap >= sum_radii - 0.05,
            "members closed to {min_gap}, inside the {sum_radii} summed radii"
        );
    }

    /// The kick is one side for the whole crowd, whatever the keys. A lone member with a neighbour
    /// behind it, which constrains nothing, answers exactly `wish − perp(wish)·nudge`: toward its right
    /// for a positive `nudge`, its left for a negative one, and the same for every key. (The kick used
    /// to take a side from a hash of the key, so half of these keys were kicked the other way.)
    #[test]
    fn every_member_is_kicked_to_the_same_side_whatever_its_key() {
        let mut crowd = Crowd::default();
        let wish = Vec2::new(1.0, 0.0);
        for nudge in [0.05, -0.05, 0.2] {
            let params = CrowdParams { nudge, ..CrowdParams::new(4.0) };
            for key in 0..64 {
                let mover = member(key, Vec2::ZERO, Vec2::ZERO, Some(wish));
                let behind = member(key + 1000, Vec2::new(-3.0, 0.0), Vec2::ZERO, None);
                let members = [mover, behind];
                let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, b| b.clear());
                let v = solved[0].expect("the mover is solved");
                assert_eq!(v, Vec2::new(1.0, -nudge), "key {key}, nudge {nudge}");
            }
        }
    }

    /// Two members walking straight at each other sidestep by the same amount, whichever keys they
    /// carry: the pair is reciprocal, as a 180° turn of the whole scene is the same scene. With a side
    /// drawn per key, exactly the key pairs whose draws differed — half of them — were kicked toward
    /// the same side of the line, and one of the two sidestepped up to a fifth further than the other.
    /// A positive `nudge` agrees with where ORCA's own tie-break sends an exactly collinear pair, each
    /// to its own right.
    #[test]
    fn a_head_on_pair_sidesteps_equally_whatever_its_keys() {
        let dt = 1.0 / 60.0;
        let mut crowd = Crowd::default();
        for nudge in [0.05, -0.05] {
            let params = CrowdParams { nudge, ..CrowdParams::new(4.0) };
            for key_a in 0..8 {
                for key_b in 0..8 {
                    let mut a = member(key_a, Vec2::new(-4.0, 0.0), Vec2::ZERO, Some(Vec2::X));
                    let mut b = member(key_b, Vec2::new(4.0, 0.0), Vec2::ZERO, Some(-Vec2::X));
                    let (mut peak_a, mut peak_b) = (0.0f32, 0.0f32);
                    for _ in 0..480 {
                        let members = [a, b];
                        let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear());
                        a.vel = solved[0].expect("solved");
                        b.vel = solved[1].expect("solved");
                        a.pos += a.vel * dt;
                        b.pos += b.vel * dt;
                        if a.pos.y.abs() > peak_a.abs() {
                            peak_a = a.pos.y;
                        }
                        if b.pos.y.abs() > peak_b.abs() {
                            peak_b = b.pos.y;
                        }
                    }
                    let at =
                        format!("keys ({key_a}, {key_b}), nudge {nudge}: sidesteps {peak_a} and {peak_b}");
                    assert!(peak_a.abs() > 0.1, "{at}: the pair did not sidestep");
                    assert!((peak_a + peak_b).abs() < 0.01 * peak_a.abs(), "{at}: not equal and opposite");
                    assert!(a.pos.x > 3.5 && b.pos.x < -3.5, "{at}: the pair did not get past each other");
                    if nudge > 0.0 {
                        // `a` heads along +x, whose right is −y; `b` heads along −x, whose right is +y.
                        assert!(peak_a < 0.0 && peak_b > 0.0, "{at}: not each to its own right");
                    }
                }
            }
        }
    }

    /// The result of one circle swap: how many agents arrived, how many ever stopped for over two
    /// seconds on the way, and how many of the agent-steps spent between a tenth and half of the ring's
    /// radius from the centre, moving, were turning counter-clockwise rather than clockwise.
    struct Swap {
        arrived: usize,
        stalled: usize,
        turning_ccw: usize,
        turning_cw: usize,
    }

    /// `n` agents of radius 0.3 at 1 m of ring each, every one walking to the point opposite its own,
    /// at 1.3 m/s, stepped for three times the straight-line time with `Crowd`'s defaults but `nudge`.
    fn circle_swap(n: usize, nudge: f32) -> Swap {
        let (radius, speed, dt) = (0.3f32, 1.3f32, 1.0 / 30.0);
        let ring = n as f32 / core::f32::consts::TAU;
        let angle = |i: usize| i as f32 * core::f32::consts::TAU / n as f32;
        let mut pos: Vec<Vec2> = (0..n).map(|i| Vec2::new(angle(i).cos(), angle(i).sin()) * ring).collect();
        let goal: Vec<Vec2> = pos.iter().map(|p| -*p).collect();
        let mut vel = vec![Vec2::ZERO; n];
        let mut swap = Swap { arrived: 0, stalled: 0, turning_ccw: 0, turning_cw: 0 };
        let (mut gone, mut slow) = (vec![false; n], vec![0u32; n]);
        let params = CrowdParams { nudge, ..CrowdParams::new(3.0) };
        let mut crowd = Crowd::default();
        let (mut members, mut slots) = (Vec::new(), Vec::new());
        for _ in 0..(3.0 * 2.0 * ring / speed / dt) as usize {
            members.clear();
            slots.clear();
            for i in (0..n).filter(|&i| !gone[i]) {
                let pref = (goal[i] - pos[i]).clamp_length_max(speed);
                members.push(CrowdMember {
                    key: i as u64,
                    pos: pos[i],
                    vel: vel[i],
                    radius,
                    max_speed: speed,
                    pref_vel: Some(pref),
                });
                slots.push(i);
            }
            if members.is_empty() {
                break;
            }
            let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, b| b.clear());
            for (&i, v) in slots.iter().zip(solved) {
                vel[i] = v.expect("solved");
                pos[i] += vel[i] * dt;
            }
            for &i in &slots {
                let from_centre = pos[i].length();
                if pos[i].distance(goal[i]) < 0.5 {
                    gone[i] = true;
                } else if vel[i].length() < 0.05 * speed {
                    slow[i] += 1;
                    // Counted once, on the step the stop reaches two seconds.
                    if slow[i] == (2.0 / dt) as u32 + 1 {
                        swap.stalled += 1;
                    }
                } else {
                    slow[i] = 0;
                    if (0.1 * ring..0.5 * ring).contains(&from_centre) {
                        let turn = pos[i].perp_dot(vel[i]);
                        swap.turning_ccw += usize::from(turn > 0.0);
                        swap.turning_cw += usize::from(turn < 0.0);
                    }
                }
            }
        }
        swap.arrived = gone.iter().filter(|g| **g).count();
        swap
    }

    /// The circle swap at the defaults is every stream crossing every other. With a side drawn per key a
    /// third of a 75-agent ring stopped for over two seconds on the way; with one side for everyone
    /// they all arrive, and none of them stops.
    #[test]
    fn a_circle_swap_arrives_and_nobody_stalls() {
        let n = 75;
        let swap = circle_swap(n, 0.05);
        assert_eq!(
            (swap.arrived, swap.stalled),
            (n, 0),
            "{} of {n} arrived, {} stalled",
            swap.arrived,
            swap.stalled
        );
    }

    /// The one side is what turns the swap into a vortex: nearly every agent that is crossing the middle
    /// turns the same way, counter-clockwise for a positive `nudge` (each kicked to its right, as a
    /// right-hand-traffic roundabout turns) and clockwise for a negative one. A side drawn per key
    /// turned them every which way.
    #[test]
    fn the_side_of_the_kick_sets_which_way_a_crowd_turns() {
        for (nudge, ccw) in [(0.05, true), (-0.05, false)] {
            let swap = circle_swap(75, nudge);
            let (with, against) =
                if ccw { (swap.turning_ccw, swap.turning_cw) } else { (swap.turning_cw, swap.turning_ccw) };
            assert!(
                with as f32 > 0.9 * (with + against) as f32,
                "nudge {nudge}: {with} agent-steps turned with the kick and {against} against it"
            );
        }
    }

    /// An unsteered member is never solved for, and the steered one takes the *full* avoidance against
    /// it rather than half — otherwise the pair drifts through each other.
    #[test]
    fn an_unsteered_member_is_not_moved_and_is_avoided() {
        let params = CrowdParams::new(4.0);
        let mut crowd = Crowd::default();
        let still = member(7, Vec2::new(0.0, 0.0), Vec2::ZERO, None);
        let mover = member(8, Vec2::new(-1.5, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X));
        let members = [still, mover];
        let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
        assert_eq!(solved[0], None, "an unsteered member must not be solved for");
        let v = solved[1].expect("the steered member is solved");
        assert!(v.is_finite());
        assert!((v - Vec2::X).length() > 1.0e-3, "expected avoidance, got {v}");
    }

    /// A finite wish is a direction however fast it is. A member whose wish is too long to square — or
    /// too long to have the sideways nudge added to it, which a neighbour in range turns on — still
    /// walks that way at full speed and is not stopped, as `new_velocity_in` does for the same wish.
    #[test]
    fn a_huge_finite_wish_still_moves_the_member() {
        let mut crowd = Crowd::default();
        let max = f32::MAX;
        let wishes = [
            (Vec2::new(1.0e20, 0.0), Vec2::X),
            (Vec2::new(0.0, -1.0e30), -Vec2::Y),
            (Vec2::new(1.0e30, 1.0e30), Vec2::ONE.normalize()),
            (Vec2::new(max, max), Vec2::ONE.normalize()),
            (Vec2::new(-max, max), Vec2::new(-1.0, 1.0).normalize()),
        ];
        for nudge in [0.0, 0.05] {
            let params = CrowdParams { nudge, ..CrowdParams::new(4.0) };
            for (wish, direction) in wishes {
                // The neighbour stands behind the mover, so nothing constrains the way it wants to go.
                let mover = member(1, Vec2::ZERO, Vec2::ZERO, Some(wish));
                let behind = member(2, -direction * 1.2, Vec2::ZERO, None);
                let solved =
                    crowd.solve(&[mover, behind], &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
                let v = solved[0].expect("a finite wish is solved");
                // A 5% nudge turns the wish by at most 0.05 rad, which moves a 1.5 m/s answer by 0.075.
                let tolerance = if nudge == 0.0 { 1.0e-3 } else { 0.1 };
                assert!(
                    (v - direction * mover.max_speed).length() < tolerance,
                    "wish {wish}, nudge {nudge}: got {v}, want {} along {direction}",
                    mover.max_speed
                );
            }
        }
    }

    /// A member this solve will not move — a wish it cannot use, or no speed to move at — is avoided
    /// in full, exactly as an unsteered one is: `avoids` means "this member will step aside", and
    /// neither of these will. A steered member still takes half, so the control below differs.
    #[test]
    fn a_member_that_will_not_move_is_avoided_in_full() {
        let params = CrowdParams::new(4.0);
        let mover = member(8, Vec2::new(-1.5, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X));
        let unsteered = member(7, Vec2::ZERO, Vec2::ZERO, None);
        let mut crowd = Crowd::default();
        let mut solve_mover = |other: CrowdMember| {
            crowd.solve(&[other, mover], &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear())[1]
                .expect("the mover is solved")
        };
        let want = solve_mover(unsteered);

        let wish = |p: Vec2| CrowdMember { pref_vel: Some(p), ..unsteered };
        let wish_and_speed = |max_speed: f32| CrowdMember { max_speed, ..wish(Vec2::X) };
        for still in [
            wish(Vec2::new(f32::NAN, 0.0)),
            wish(Vec2::new(0.0, f32::INFINITY)),
            wish_and_speed(0.0),
            wish_and_speed(-1.0),
            wish_and_speed(f32::NAN),
            wish_and_speed(f32::INFINITY),
            // So large that its square overflows, or so small that it is not a normal number:
            // `new_velocity_in` answers `Vec2::ZERO` for them too.
            wish_and_speed(1.0e20),
            wish_and_speed(1.0e-30),
        ] {
            let got = solve_mover(still);
            assert_eq!(
                (got.x.to_bits(), got.y.to_bits()),
                (want.x.to_bits(), want.y.to_bits()),
                "{still:?} does not move but was assumed to take half: {got} vs {want}"
            );
        }

        // Control: a member that will step aside takes its half, so the mover takes less.
        let half = solve_mover(wish(Vec2::ZERO));
        assert_ne!(
            (half.x.to_bits(), half.y.to_bits()),
            (want.x.to_bits(), want.y.to_bits()),
            "a steered member must still be assumed to avoid"
        );
    }

    /// `sees` is the host's rule, and a `false` from it means the two genuinely ignore each other.
    #[test]
    fn a_hidden_member_is_ignored() {
        let params = CrowdParams::new(4.0);
        let mut crowd = Crowd::default();
        let a = member(1, Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X));
        let b = member(2, Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Some(-Vec2::X));
        let members = [a, b];
        let solved = crowd.solve(&members, &params, 1.0 / 60.0, |i, j| i == j, |_, buf| buf.clear());
        for (i, v) in solved.iter().enumerate() {
            let v = v.expect("both are steered");
            let pref = if i == 0 { Vec2::X } else { -Vec2::X };
            assert!(
                (v - pref).length() < 1.0e-4,
                "member {i} should ignore the other and keep {pref}, got {v}"
            );
        }
    }

    /// Shuffling the input slice must not change what any member does, because the caller's iteration
    /// order is an ECS query's business and the simulation has to be reproducible regardless.
    #[test]
    fn results_do_not_depend_on_member_order() {
        let params = CrowdParams::new(4.0);
        let members: Vec<CrowdMember> = (0..8)
            .map(|i| {
                let angle = (i as f32) * std::f32::consts::TAU / 8.0;
                member(
                    i as u64,
                    Vec2::new(angle.cos(), angle.sin()) * 1.2,
                    Vec2::ZERO,
                    Some(Vec2::new(angle.cos(), angle.sin())),
                )
            })
            .collect();

        let mut reference = Crowd::default();
        let base: Vec<(u64, Option<Vec2>)> = reference
            .solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear())
            .iter()
            .enumerate()
            .map(|(i, v)| (members[i].key, *v))
            .collect();

        // A fixed permutation (reverse, then rotate by three) — deterministic, like everything else.
        let order: Vec<usize> =
            (0..members.len()).map(|i| (members.len() - 1 - i + 3) % members.len()).collect();
        let shuffled: Vec<CrowdMember> = order.iter().map(|&i| members[i]).collect();

        let mut other = Crowd::default();
        let got = other.solve(&shuffled, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
        for (k, &member) in shuffled.iter().enumerate() {
            let expect = base.iter().find(|(key, _)| *key == member.key).expect("a key per member").1;
            let actual = got[k];
            match (expect, actual) {
                (Some(a), Some(b)) => {
                    assert_eq!(
                        a.x.to_bits(),
                        b.x.to_bits(),
                        "key {}: x differs under reordering",
                        member.key
                    );
                    assert_eq!(
                        a.y.to_bits(),
                        b.y.to_bits(),
                        "key {}: y differs under reordering",
                        member.key
                    );
                }
                (None, None) => {}
                _ => panic!("key {}: solved for in one order and not the other", member.key),
            }
        }
    }

    /// With far more members in range than `max_neighbours`, each member is solved against exactly its
    /// nearest `max_neighbours`, an equal distance going to the smaller key — including where the cut
    /// falls inside a ring of equidistant members, and for no neighbours, one, and more than are in
    /// range. The reference is a brute-force scan of every member, stably sorted.
    #[test]
    fn a_dense_crowd_is_solved_against_its_nearest_neighbours() {
        // A 15 × 15 lattice 0.5 apart: every member has rings of neighbours at bit-equal distances.
        // Keys are scrambled so key order is neither slice order nor lattice order, and velocities
        // differ so that which neighbours are kept changes the answer.
        let members: Vec<CrowdMember> = (0..225u64)
            .map(|i| CrowdMember {
                radius: 0.2,
                ..member(
                    splitmix64(i),
                    Vec2::new((i % 15) as f32, (i / 15) as f32) * 0.5,
                    Vec2::from_angle(i as f32 * 0.7) * 0.8,
                    Some(Vec2::new(1.0, 0.3)),
                )
            })
            .collect();
        let dt = 1.0 / 60.0;
        let mut crowd = Crowd::default();
        let mut scratch = OrcaScratch::default();
        let mut neighbours = Vec::new();
        for max_neighbours in [0, 1, 6, 10, 30, 1_000] {
            // No nudge, so the wish is the preference and the reference needs nothing but the scan.
            let params = CrowdParams { max_neighbours, nudge: 0.0, ..CrowdParams::new(3.0) };
            let solved = crowd.solve(&members, &params, dt, |_, _| true, |_, buf| buf.clear()).to_vec();
            for (i, m) in members.iter().enumerate() {
                let mut near: Vec<(f32, u64, usize)> = members
                    .iter()
                    .enumerate()
                    .filter(|&(j, o)| j != i && o.pos.distance(m.pos) <= params.neighbour_range)
                    .map(|(j, o)| (o.pos.distance(m.pos), o.key, j))
                    .collect();
                near.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                neighbours.clear();
                neighbours.extend(near.iter().take(max_neighbours).map(|&(_, _, j)| Agent {
                    pos: members[j].pos,
                    vel: members[j].vel,
                    radius: members[j].radius,
                    avoids: true,
                }));
                let me = Agent { pos: m.pos, vel: m.vel, radius: m.radius, avoids: true };
                let pref = m.pref_vel.unwrap_or_default();
                let want = new_velocity_in(
                    &mut scratch,
                    &me,
                    pref,
                    &neighbours,
                    &[],
                    params.time_horizon,
                    dt,
                    m.max_speed,
                );
                let got = solved[i].map(|v| (v.x.to_bits(), v.y.to_bits()));
                assert_eq!(
                    got,
                    Some((want.x.to_bits(), want.y.to_bits())),
                    "member {i} with max_neighbours {max_neighbours}: {:?} vs {want}",
                    solved[i]
                );
            }
        }
    }

    /// A degenerate range must not mean "everyone is a neighbour" — it means no neighbour at all.
    #[test]
    fn a_bad_range_leaves_members_alone() {
        let a = member(1, Vec2::new(-0.1, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X));
        let b = member(2, Vec2::new(0.1, 0.0), Vec2::new(-1.0, 0.0), Some(-Vec2::X));
        let members = [a, b];
        for range in [0.0_f32, -1.0, f32::NAN] {
            let params = CrowdParams::new(range);
            let mut crowd = Crowd::default();
            let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
            assert_eq!(solved[0], Some(Vec2::X), "range {range} should leave the wish alone");
            assert_eq!(solved[1], Some(-Vec2::X), "range {range} should leave the wish alone");
        }
    }

    /// A member whose position is unusable is reported unsolved and is not a neighbour to anyone.
    #[test]
    fn a_non_finite_position_is_reported_unsolved() {
        let params = CrowdParams::new(4.0);
        let mut crowd = Crowd::default();
        let broken = member(1, Vec2::new(f32::NAN, 0.0), Vec2::ZERO, Some(Vec2::X));
        let fine = member(2, Vec2::new(2.0, 0.0), Vec2::ZERO, Some(-Vec2::X));
        let members = [broken, fine];
        let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
        assert_eq!(solved[0], None);
        assert_eq!(solved[1], Some(-Vec2::X), "the broken member must not be a neighbour");
    }

    /// A finite position far enough out saturates its cell index at `i32::MAX` or `i32::MIN`; the
    /// cells beside it must still be looked up without overflowing, in a debug build as in release.
    #[test]
    fn a_member_at_the_edge_of_the_float_range_is_solved_without_panicking() {
        let params = CrowdParams::new(3.0);
        let mut crowd = Crowd::default();
        let members = [
            member(1, Vec2::new(1.0e30, -1.0e30), Vec2::ZERO, Some(Vec2::X)),
            member(2, Vec2::new(f32::MAX, f32::MAX), Vec2::ZERO, Some(-Vec2::X)),
            member(3, Vec2::new(-f32::MAX, 0.0), Vec2::ZERO, Some(Vec2::Y)),
        ];
        let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
        assert_eq!(solved, [Some(Vec2::X), Some(-Vec2::X), Some(Vec2::Y)], "nobody is near anybody");
    }

    /// A member whose velocity or radius is unusable holds still, and is nobody's neighbour: the rest
    /// of the crowd is solved bit for bit as if it were not there, and it takes no neighbour's place.
    #[test]
    fn a_member_with_unusable_state_holds_still_and_is_nobodys_neighbour() {
        let mut params = CrowdParams::new(4.0);
        params.max_neighbours = 1;
        let walker = member(1, Vec2::new(-1.0, 0.05), Vec2::new(1.0, 0.0), Some(Vec2::X));
        let other = member(2, Vec2::new(1.5, -0.05), Vec2::new(-1.0, 0.0), Some(-Vec2::X));
        let mut crowd = Crowd::default();
        let alone =
            crowd.solve(&[walker, other], &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear()).to_vec();
        // Between the two, so it would be each one's nearest neighbour if it counted.
        let between = member(3, Vec2::new(0.25, 0.0), Vec2::ZERO, Some(Vec2::Y));
        for broken in [
            CrowdMember { vel: Vec2::new(f32::NAN, 0.0), ..between },
            CrowdMember { vel: Vec2::new(0.0, f32::INFINITY), ..between },
            CrowdMember { radius: -0.4, ..between },
            CrowdMember { radius: f32::NAN, ..between },
        ] {
            let members = [walker, broken, other];
            let solved = crowd.solve(&members, &params, 1.0 / 60.0, |_, _| true, |_, buf| buf.clear());
            assert_eq!(solved[1], Some(Vec2::ZERO), "{broken:?} should hold still");
            for (got, want) in [(solved[0], alone[0]), (solved[2], alone[1])] {
                let (got, want) = (got.expect("solved"), want.expect("solved"));
                assert_eq!(
                    got.x.to_bits(),
                    want.x.to_bits(),
                    "{broken:?} moved a neighbour: {got} vs {want}"
                );
                assert_eq!(
                    got.y.to_bits(),
                    want.y.to_bits(),
                    "{broken:?} moved a neighbour: {got} vs {want}"
                );
            }
        }
    }

    /// Crowds of every size from empty to eight, with any member field, parameter, wall or `dt` made
    /// hostile at random: every solved velocity is finite, and nothing panics.
    #[test]
    fn hostile_crowds_never_give_a_non_finite_velocity() {
        use crate::tests::HOSTILE;
        let mut state = 0u64;
        let mut next = || {
            state = state.wrapping_add(1);
            (splitmix64(state) >> 40) as f32 / (1u64 << 24) as f32
        };
        let mut crowd = Crowd::default();
        for step in 0..3_000u64 {
            let share = if step % 4 == 0 { 0.0 } else { 0.25 };
            let mut value = |lo: f32, hi: f32| {
                let (sane, roll, pick) = (lo + (hi - lo) * next(), next(), next());
                if roll < share {
                    HOSTILE[((pick * HOSTILE.len() as f32) as usize).min(HOSTILE.len() - 1)]
                } else {
                    sane
                }
            };
            let n = (step % 9) as usize;
            let members: Vec<CrowdMember> = (0..n)
                .map(|i| CrowdMember {
                    key: i as u64,
                    pos: Vec2::new(value(-2.0, 2.0), value(-2.0, 2.0)),
                    vel: Vec2::new(value(-1.5, 1.5), value(-1.5, 1.5)),
                    radius: value(0.2, 0.5),
                    max_speed: value(0.5, 2.0),
                    pref_vel: Some(Vec2::new(value(-1.5, 1.5), value(-1.5, 1.5))),
                })
                .collect();
            let walls: Vec<(Vec2, f32)> = (0..n)
                .map(|_| {
                    let angle = value(0.0, std::f32::consts::TAU);
                    (Vec2::new(angle.cos(), angle.sin()), value(-0.5, 3.0))
                })
                .collect();
            let params = CrowdParams {
                time_horizon: value(0.5, 3.0),
                neighbour_range: value(1.0, 4.0),
                max_neighbours: (step % 7) as usize,
                nudge: value(0.0, 0.1),
            };
            let dt = value(1.0 / 60.0, 0.1);
            let solved = crowd.solve(
                &members,
                &params,
                dt,
                |i, j| (i + j) % 5 != 3,
                |i, buf| {
                    if let Some(&wall) = walls.get(i)
                        && i % 2 == 0
                    {
                        buf.push(wall);
                    }
                },
            );
            for (i, v) in solved.iter().enumerate() {
                if let Some(v) = v {
                    assert!(v.is_finite(), "step {step}: member {i} {:?} solved to {v}", members[i]);
                }
            }
        }
    }

    /// A packed knot is the over-constrained step where the linear program relaxes its constraints,
    /// and a member must still never leave it faster than its own `max_speed`: on `0.2.1`, 0.8% of
    /// the velocities `solve` returned for random packs of 2 to 40 members were over that limit, by
    /// up to 77 times. Packs here are 2 to 40 members in a 2 to 6 m square, each walking in a random
    /// direction at its own speed, stepped 30 times at 20 Hz.
    #[test]
    fn a_packed_crowd_never_leaves_faster_than_its_speed_limit() {
        let mut state = 7u64;
        let mut next = || {
            state = state.wrapping_add(1);
            (splitmix64(state) >> 40) as f32 / (1u64 << 24) as f32
        };
        let mut crowd = Crowd::default();
        let params = CrowdParams::new(3.0);
        for pack in 0..60 {
            let count = 2 + (39.0 * next()) as usize;
            let side = 2.0 + 4.0 * next();
            let mut members: Vec<CrowdMember> = (0..count)
                .map(|key| CrowdMember {
                    key: key as u64,
                    pos: Vec2::new(side * next(), side * next()),
                    vel: Vec2::ZERO,
                    radius: 0.25 + 0.15 * next(),
                    max_speed: 1.0 + 0.5 * next(),
                    pref_vel: Some(Vec2::from_angle(std::f32::consts::TAU * next()) * (0.8 + 0.7 * next())),
                })
                .collect();
            for step in 0..30 {
                let solved = crowd.solve(&members, &params, 0.05, |_, _| true, |_, _| {}).to_vec();
                for (i, (m, v)) in members.iter_mut().zip(solved).enumerate() {
                    let v = v.expect("every member has a wish");
                    let speed = f64::from(v.x).hypot(f64::from(v.y));
                    assert!(
                        speed <= f64::from(m.max_speed) * (1.0 + 1.0e-6),
                        "pack {pack} step {step} member {i}: speed {speed} is {}× max_speed {}",
                        speed / f64::from(m.max_speed),
                        m.max_speed,
                    );
                    m.vel = v;
                    m.pos += v * 0.05;
                }
            }
        }
    }
}
