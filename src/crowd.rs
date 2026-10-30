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

use crate::{Agent, OrcaScratch, new_velocity_in, usable_state};

/// One member of a crowd, as the batch solver sees it.
///
/// `Vec2` is the host's ground plane, in the host's units: the crate never converts.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "reflect", derive(bevy_reflect::Reflect))]
pub struct CrowdMember {
    /// The caller's stable id for this member. It breaks a tie between two equidistant neighbours and
    /// picks which side of an exact head-on meeting this member steps to, so an identical world solves
    /// identically however the members are ordered in the input slice.
    pub key: u64,
    /// Position on the ground plane.
    pub pos: Vec2,
    /// Current velocity.
    pub vel: Vec2,
    /// Body radius.
    pub radius: f32,
    /// Speed ceiling for the solved velocity.
    pub max_speed: f32,
    /// Where this member wants to go.
    ///
    /// `None` means another system moves this member this step: it is a neighbour to everyone else and
    /// is never solved for. Its velocity is still read, so the agent that is moving it is avoided.
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
    /// prefer either way and will otherwise stall.
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

/// The SplitMix64 output mix. A stable hash of the caller's key, so the side a head-on member steps to
/// is a property of that member and not of the tick.
fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
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
        if search {
            for (i, m) in members.iter().enumerate() {
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
            let (Some(pref), true) = (m.pref_vel, m.pos.is_finite()) else {
                continue;
            };
            if !pref.is_finite() {
                continue;
            }

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
                    // A member nobody steers is not assumed to step aside, so this one takes the full
                    // avoidance against it instead of walking through it.
                    avoids: other.pref_vel.is_some(),
                });
            }

            // The wish, nudged sideways when anyone is near. The side is the member's own, hashed from
            // its key, so two agents meeting head-on do not both pick the same side.
            let wish = if self.near.is_empty() {
                pref
            } else {
                let side = if splitmix64(m.key) & 1 == 0 { 1.0 } else { -1.0 };
                pref + pref.perp() * (params.nudge * side)
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
}
