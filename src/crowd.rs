//! **A whole crowd, solved as one batch.**
//!
//! [`new_velocity_in`] solves one agent against a neighbour slice the caller assembled. Every host
//! that owns a crowd then writes the same four things around it — a spatial hash to find neighbours
//! (an O(n²) scan does not survive a few hundred agents), a reused [`OrcaScratch`], a tie-break that
//! is deterministic when two agents want the same space, and the wall buffer. This module is that
//! ring, once, so a host learns it once.
//!
//! The caller still decides the things that are gameplay: **who is a neighbour** (`solve`'s `sees`),
//! **what is solid** (its `walls` closure, for a wall that changes every step, and the
//! [`Obstacles`] handed to [`Crowd::solve_around`], for geometry that does not) and where everyone
//! wants to go. This module never looks at a game type.

use bevy_math::Vec2;

use crate::{Agent, Obstacles, OrcaScratch, new_velocity_in, usable_speed, usable_state};

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
    /// Right of way against a member that also steps aside this step: a ratio, `1.0` unless the host
    /// says otherwise. Of the dodge a collision course asks for, this member takes
    /// `other / (this + other)` and the other takes the rest, so equal priorities split it evenly
    /// (ORCA's half), a member of priority `4` against one of `1` takes a fifth, and a stall-holder
    /// the street should walk around is given more than the walkers. It is not an immovable member:
    /// that is `pref_vel: None` or a `max_speed` of `0`, which everyone avoids in full whatever this
    /// says. NaN or negative counts as `0` (the member gives way), `∞` as the right of way over any
    /// finite priority. See [`Agent::priority`].
    pub priority: f32,
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
    /// Seconds of lookahead against [`Obstacles`], separate from `time_horizon`: only
    /// [`Crowd::solve_around`] reads it. An agent should not be "shy" of a wall it must pass close
    /// to in order to avoid another agent, so this is ordinarily shorter than `time_horizon` (the
    /// ORCA paper's section 5.4; Menge ships 0.15 s against 3 s). [`Crowd::solve_around`] raises it
    /// to the step `dt` if it is shorter, because a horizon under the step lets a member cross a
    /// wall within it. It is also how far a member looks for walls:
    /// `obstacle_horizon · max_speed + radius`.
    pub obstacle_horizon: f32,
}

impl CrowdParams {
    /// Parameters for a crowd whose neighbours are within `neighbour_range` of each other: a 2-second
    /// horizon against other members, 0.5 seconds against obstacles, at most 10 neighbours and a 5%
    /// nudge.
    pub fn new(neighbour_range: f32) -> Self {
        Self { time_horizon: 2.0, neighbour_range, max_neighbours: 10, nudge: 0.05, obstacle_horizon: 0.5 }
    }
}

/// A reusable crowd solver: the neighbour grid, the line buffers and the output slice.
///
/// Own one per crowd and call [`solve`](Self::solve) every tick. Nothing in it is per-member state, so
/// it is safe to solve several crowds with one `Crowd`, one after the other. Its buffers grow to the
/// largest crowd it has solved and never shrink, so once warm it allocates nothing, wherever the crowd
/// walks and however tightly it packs.
///
/// To spread one step over several threads, [`begin`](Self::begin) builds the grid and hands back a
/// [`CrowdStep`] whose ranges any number of [`CrowdWorker`]s can solve at once; `solve` is that, on one
/// range with the crowd's own worker.
#[derive(Debug, Default)]
pub struct Crowd {
    worker: CrowdWorker,
    /// The neighbour grid: `(cell key, member index)` for every member that can be a neighbour (finite
    /// position and velocity, finite non-negative radius), sorted,
    /// `neighbour_range` per cell. Rebuilt every solve, so its size is the crowd's and not the ground
    /// the crowd has covered. Lookup only: every candidate list is sorted before use.
    cells: Vec<(u64, usize)>,
    /// Whether [`steers`] holds for each member, filled in the same pass as the grid. A neighbour's
    /// `avoids` is read from here, so the test runs once per member and not once per neighbour offered.
    steered: Vec<bool>,
    /// Reused output, indexed like the members slice.
    out: Vec<Option<Vec2>>,
}

/// The buffers one thread solves members with: the line-program scratch, the candidate and neighbour
/// lists, and the wall buffer.
///
/// [`CrowdStep::solve_range`] takes one, so a host that splits a step over threads keeps one per
/// thread (in a Bevy system, a `Local<Vec<CrowdWorker>>`); [`Crowd`] owns the one [`Crowd::solve`]
/// uses. It holds no per-member state and nothing in it carries from one call to the next except
/// capacity: its buffers grow to the widest call they have served and never shrink, so once warm a
/// worker allocates nothing. Two ranges running at once need two workers.
#[derive(Debug, Default)]
pub struct CrowdWorker {
    scratch: OrcaScratch,
    /// Reused candidate buffer: `(distance, key, index)`, cut to the nearest `max_neighbours` and sorted
    /// nearest-first before use.
    near: Vec<(f32, u64, usize)>,
    /// Reused neighbour slice handed to the solver.
    neighbours: Vec<Agent>,
    /// Reused wall buffer, filled by the caller's `walls` closure.
    walls: Vec<(Vec2, f32)>,
}

/// One step of a crowd, ready to be solved in ranges: what [`Crowd::begin`] returns once the serial
/// part (the neighbour grid) is built.
///
/// It is a read-only view of the members, the grid and the step's parameters, so it is `Copy`, `Send`
/// and `Sync`: every task of a thread pool can hold one, and each calls
/// [`solve_range`](Self::solve_range) on its own slice of the output with its own [`CrowdWorker`]. The
/// crate starts no thread and takes no lock; where the ranges run, and how many, is the caller's.
///
/// **The split changes no velocity.** A member's answer is a function of the frozen members, the grid
/// built from them, and the caller's `sees` and `walls` for that member; no member's solve reads
/// another's answer, and nothing is summed across members. So any partition of the members into ranges,
/// solved in any order by any number of workers, gives slot for slot the bits [`Crowd::solve`] gives
/// (provided `sees` and `walls` are themselves functions of the indices they are given).
///
/// ```
/// use bevy_math::Vec2;
/// use bevy_orca::{Crowd, CrowdMember, CrowdParams, CrowdWorker};
///
/// // A thousand walkers in a block, all heading east.
/// let members: Vec<CrowdMember> = (0..1000)
///     .map(|i| CrowdMember {
///         key: i,
///         pos: Vec2::new((i % 40) as f32, (i / 40) as f32) * 0.7,
///         vel: Vec2::ZERO,
///         radius: 0.3,
///         max_speed: 1.3,
///         pref_vel: Some(Vec2::X),
///         priority: 1.0,
///     })
///     .collect();
/// let params = CrowdParams::new(3.0);
/// let dt = 1.0 / 30.0;
///
/// // The serial part, once; then four workers deal the ranges out between them.
/// let mut crowd = Crowd::default();
/// let mut workers: Vec<CrowdWorker> = (0..4).map(|_| CrowdWorker::default()).collect();
/// let mut solved = vec![None; members.len()];
/// let step = crowd.begin(&members, &params, dt);
/// let mut lanes: Vec<Vec<(usize, &mut [Option<Vec2>])>> = (0..4).map(|_| Vec::new()).collect();
/// for (c, slice) in solved.chunks_mut(128).enumerate() {
///     lanes[c % 4].push((c * 128, slice));
/// }
/// std::thread::scope(|scope| {
///     for (worker, lane) in workers.iter_mut().zip(lanes) {
///         scope.spawn(move || {
///             for (first, slice) in lane {
///                 step.solve_range(worker, first, slice, |_, _| true, |_, _| {});
///             }
///         });
///     }
/// });
///
/// // The same bits the serial solve gives.
/// let mut serial = Crowd::default();
/// assert_eq!(serial.solve(&members, &params, dt, |_, _| true, |_, _| {}), &solved[..]);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct CrowdStep<'a> {
    obstacles: Option<&'a Obstacles>,
    members: &'a [CrowdMember],
    params: CrowdParams,
    dt: f32,
    /// [`Crowd`]'s grid, built for `members`.
    cells: &'a [(u64, usize)],
    /// [`steers`] of each member, built with the grid.
    steered: &'a [bool],
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

/// Rebuild the grid: one entry per member that can be a neighbour, sorted by cell, and each member's
/// [`steers`] flag. The pairs are unique, so the sorted order is fully determined, and the buffers are
/// cleared rather than dropped, so a warm `Crowd` allocates nothing here however far its members have
/// walked. A member with no usable state has no velocity obstacle, so it is left out here rather than
/// taking a neighbour's place only to be skipped by the solver. A `range` that is not a usable cell
/// width leaves both empty: nobody has neighbours.
fn build_grid(cells: &mut Vec<(u64, usize)>, steered: &mut Vec<bool>, members: &[CrowdMember], range: f32) {
    cells.clear();
    steered.clear();
    if !searchable(range) {
        return;
    }
    for (i, m) in members.iter().enumerate() {
        steered.push(steers(m));
        if !usable_state(m.pos, m.vel, m.radius) {
            continue;
        }
        if let Some((column, row)) = cell_of(m.pos, range) {
            cells.push((cell_key(i64::from(column), i64::from(row)), i));
        }
    }
    cells.sort_unstable();
}

/// Whether `range` can be a grid cell's width: finite and positive.
#[inline]
fn searchable(range: f32) -> bool {
    range > 0.0 && range.is_finite()
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
    ///
    /// Static geometry that does not change from step to step belongs in an [`Obstacles`] and
    /// [`solve_around`](Self::solve_around), which does everything this does and derives the
    /// half-planes from the geometry. To spread the step over threads, [`begin`](Self::begin) and
    /// [`CrowdStep::solve_range`] do the same solve in ranges.
    pub fn solve(
        &mut self,
        members: &[CrowdMember],
        params: &CrowdParams,
        dt: f32,
        sees: impl FnMut(usize, usize) -> bool,
        walls: impl FnMut(usize, &mut Vec<(Vec2, f32)>),
    ) -> &[Option<Vec2>] {
        self.solve_with(None, members, params, dt, sees, walls)
    }

    /// [`solve`](Self::solve) among static [`Obstacles`]: every member also keeps clear of the
    /// polygons and walls in `obstacles`, by hard lines that go after the member's `walls` and ahead
    /// of every neighbour, are never relaxed, and are cast `params.obstacle_horizon` seconds ahead
    /// (raised to `dt` if shorter, since a horizon under the step lets a member cross a wall within
    /// it; see [`Obstacles::push_walls`]). An empty set gives exactly the answers `solve` gives.
    ///
    /// `walls` is still the caller's, for what changes every step (a door that opens); pass
    /// `|_, _| {}` when there is none. Obstacles make a member avoid, not navigate: the way round a
    /// building is its `pref_vel`.
    pub fn solve_around(
        &mut self,
        obstacles: &Obstacles,
        members: &[CrowdMember],
        params: &CrowdParams,
        dt: f32,
        sees: impl FnMut(usize, usize) -> bool,
        walls: impl FnMut(usize, &mut Vec<(Vec2, f32)>),
    ) -> &[Option<Vec2>] {
        self.solve_with(Some(obstacles), members, params, dt, sees, walls)
    }

    /// Build the neighbour grid for one step of `dt` and return it as a [`CrowdStep`], whose
    /// [`solve_range`](CrowdStep::solve_range) solves the members, in as many ranges, on as many
    /// threads, as the caller likes. This is the whole serial part of [`solve`](Self::solve); the rest
    /// is per-member work no member's answer depends on.
    ///
    /// The step borrows this `Crowd` and `members` for as long as it lives, so the members cannot
    /// change, nor a second step begin on this crowd, while ranges are being solved.
    pub fn begin<'a>(
        &'a mut self,
        members: &'a [CrowdMember],
        params: &CrowdParams,
        dt: f32,
    ) -> CrowdStep<'a> {
        self.begin_with(None, members, params, dt)
    }

    /// [`begin`](Self::begin) among static [`Obstacles`]: the step's ranges cast the same hard obstacle
    /// lines [`solve_around`](Self::solve_around) does.
    pub fn begin_around<'a>(
        &'a mut self,
        obstacles: &'a Obstacles,
        members: &'a [CrowdMember],
        params: &CrowdParams,
        dt: f32,
    ) -> CrowdStep<'a> {
        self.begin_with(Some(obstacles), members, params, dt)
    }

    fn begin_with<'a>(
        &'a mut self,
        obstacles: Option<&'a Obstacles>,
        members: &'a [CrowdMember],
        params: &CrowdParams,
        dt: f32,
    ) -> CrowdStep<'a> {
        build_grid(&mut self.cells, &mut self.steered, members, params.neighbour_range);
        CrowdStep { obstacles, members, params: *params, dt, cells: &self.cells, steered: &self.steered }
    }

    /// The one serial solve, with or without obstacles: the grid, then every member as one range.
    fn solve_with(
        &mut self,
        obstacles: Option<&Obstacles>,
        members: &[CrowdMember],
        params: &CrowdParams,
        dt: f32,
        sees: impl FnMut(usize, usize) -> bool,
        walls: impl FnMut(usize, &mut Vec<(Vec2, f32)>),
    ) -> &[Option<Vec2>] {
        build_grid(&mut self.cells, &mut self.steered, members, params.neighbour_range);
        self.out.clear();
        self.out.resize(members.len(), None);
        let step =
            CrowdStep { obstacles, members, params: *params, dt, cells: &self.cells, steered: &self.steered };
        step.solve_range(&mut self.worker, 0, &mut self.out, sees, walls);
        &self.out
    }
}

impl CrowdStep<'_> {
    /// Solve members `first .. first + out.len()` and write each answer to `out`: `out[k]` is member
    /// `first + k`, exactly what [`Crowd::solve`] returns for it (`None` when it is not solved, see
    /// there). A slot past the last member is `None`, and so is one whose index would overflow
    /// `usize`; every slot of `out` is overwritten. Calls may run at the same time on different
    /// threads, each with its own `worker` and its own slice of the output: nothing they read is
    /// written, and they share nothing they write.
    ///
    /// `sees(i, j)` and `walls(i, &mut buf)` are as in [`Crowd::solve`], and the indices they are
    /// given are indices into the `members` the step was begun with, not into the range. Each call takes
    /// its own closures, so a closure may capture a `&mut` counter of its own; a closure whose answer
    /// depends on which call asked, or when, makes the answer depend on the split, which is the
    /// caller's choice to make.
    pub fn solve_range(
        &self,
        worker: &mut CrowdWorker,
        first: usize,
        out: &mut [Option<Vec2>],
        mut sees: impl FnMut(usize, usize) -> bool,
        mut walls: impl FnMut(usize, &mut Vec<(Vec2, f32)>),
    ) {
        let (members, params, dt) = (self.members, &self.params, self.dt);
        let range = params.neighbour_range;
        let search = searchable(range);

        for (k, slot) in out.iter_mut().enumerate() {
            *slot = None;
            let Some(i) = first.checked_add(k) else {
                continue;
            };
            let Some(&m) = members.get(i) else {
                continue;
            };
            let Some(pref) = wish(&m) else {
                continue;
            };

            // Neighbours: every member within `range` that this one sees, nearest first, at most
            // `max_neighbours` of them. Ordering by `(distance, key)` is what makes the result a
            // property of the world rather than of the input slice's order.
            worker.near.clear();
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
                        let Some(other) = members.get(j) else {
                            continue;
                        };
                        let dist = other.pos.distance(m.pos);
                        if dist > range {
                            continue;
                        }
                        if sees(i, j) {
                            worker.near.push((dist, other.key, j));
                        }
                    }
                }
            }
            // Only the nearest `max_neighbours` are used, so only they are ordered; a packed crowd
            // offers hundreds of candidates, and a stable sort would take a heap scratch for each
            // member above 170 of them.
            nearest(&mut worker.near, params.max_neighbours);

            worker.neighbours.clear();
            for &(_, _, j) in &worker.near {
                let Some(other) = members.get(j) else {
                    continue;
                };
                worker.neighbours.push(Agent {
                    pos: other.pos,
                    vel: other.vel,
                    radius: other.radius,
                    // A member this step will not move is not assumed to step aside, so this one
                    // takes the full avoidance against it instead of walking through it.
                    avoids: self.steered.get(j).is_some_and(|&steered| steered),
                    priority: other.priority,
                });
            }

            // The wish, kicked toward the right of its own direction when anyone is near: the same side
            // for every member, so two agents meeting head-on are kicked past each other and a stream
            // keeps to one side. A kick the wish cannot take — the sum is not finite, as for a wish near
            // `f32::MAX` — is not applied: such a wish is a direction already, and `new_velocity_in`
            // would refuse the non-finite sum.
            let wish = if worker.near.is_empty() {
                pref
            } else {
                let kicked = pref - pref.perp() * params.nudge;
                if kicked.is_finite() { kicked } else { pref }
            };

            worker.walls.clear();
            walls(i, &mut worker.walls);

            let me = Agent { pos: m.pos, vel: m.vel, radius: m.radius, avoids: true, priority: m.priority };
            // The obstacle lines come after the caller's own walls and are never relaxed, like them.
            if let Some(obstacles) = self.obstacles {
                obstacles.push_walls(
                    &mut worker.scratch,
                    &me,
                    m.max_speed,
                    // Not shorter than the step: a horizon under `dt` lets a member cross a wall within it.
                    params.obstacle_horizon.max(dt),
                    &mut worker.walls,
                );
            }
            *slot = Some(new_velocity_in(
                &mut worker.scratch,
                &me,
                wish,
                &worker.neighbours,
                &worker.walls,
                params.time_horizon,
                dt,
                m.max_speed,
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn member(key: u64, pos: Vec2, vel: Vec2, pref: Option<Vec2>) -> CrowdMember {
        CrowdMember { key, pos, vel, radius: 0.5, max_speed: 1.5, pref_vel: pref, priority: 1.0 }
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
                    priority: 1.0,
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
        // Keys are scrambled so key order is neither slice order nor lattice order, velocities
        // differ so that which neighbours are kept changes the answer, and priorities differ (a few of
        // them zero, infinite and NaN) so that a priority the crowd drops or misreads changes it too.
        let priorities = [1.0, 0.5, 2.0, 8.0, 0.0, f32::INFINITY, f32::NAN, 1.0, -3.0];
        let members: Vec<CrowdMember> = (0..225u64)
            .map(|i| CrowdMember {
                radius: 0.2,
                priority: priorities[(splitmix64(i ^ 5) % priorities.len() as u64) as usize],
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
                    priority: members[j].priority,
                }));
                let me =
                    Agent { pos: m.pos, vel: m.vel, radius: m.radius, avoids: true, priority: m.priority };
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
        let mut halves = Crowd::default();
        let mut workers = [CrowdWorker::default(), CrowdWorker::default()];
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
                    priority: value(0.0, 4.0),
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
                obstacle_horizon: value(0.1, 1.0),
            };
            let dt = value(1.0 / 60.0, 0.1);
            let sees = |i: usize, j: usize| (i + j) % 5 != 3;
            let fill = |i: usize, buf: &mut Vec<(Vec2, f32)>| {
                if let Some(&wall) = walls.get(i)
                    && i.is_multiple_of(2)
                {
                    buf.push(wall);
                }
            };
            let solved = crowd.solve(&members, &params, dt, sees, fill).to_vec();
            for (i, v) in solved.iter().enumerate() {
                if let Some(v) = v {
                    assert!(v.is_finite(), "step {step}: member {i} {:?} solved to {v}", members[i]);
                }
            }
            // The same step in two ranges on two workers, hostile parameters included: the same bits.
            let mut split = vec![Some(Vec2::new(9.0, 9.0)); n];
            let (low, high) = split.split_at_mut(n / 2);
            let view = halves.begin(&members, &params, dt);
            let [first_worker, second_worker] = &mut workers;
            view.solve_range(second_worker, n / 2, high, sees, fill);
            view.solve_range(first_worker, 0, low, sees, fill);
            assert_eq!(bits(&split), bits(&solved), "step {step}: two ranges differ from one");
        }
    }

    /// `n` members in a 12 m square, so each has a dozen neighbours at a 3 m range and some stand by
    /// the boxes of [`split_obstacles`]: about one in nine has one hostile field (not finite, huge,
    /// tiny, negative or zero), and one in nine is not steered.
    fn split_crowd(seed: u64, n: usize) -> Vec<CrowdMember> {
        use crate::tests::HOSTILE;
        let mut state = seed.wrapping_mul(0x9E37_79B9);
        let mut next = || {
            state = state.wrapping_add(1);
            (splitmix64(state) >> 40) as f32 / (1u64 << 24) as f32
        };
        (0..n)
            .map(|i| {
                let mut m = CrowdMember {
                    // Keys are scrambled, so key order is neither slice order nor position order.
                    key: splitmix64(i as u64 ^ seed) % 1000,
                    pos: Vec2::new(next() * 12.0, next() * 12.0),
                    vel: Vec2::new(next() * 3.0 - 1.5, next() * 3.0 - 1.5),
                    radius: 0.2 + next() * 0.3,
                    max_speed: 0.5 + next() * 1.5,
                    pref_vel: Some(Vec2::new(next() * 3.0 - 1.5, next() * 3.0 - 1.5)),
                    priority: 1.0,
                };
                let hostile = HOSTILE[((next() * HOSTILE.len() as f32) as usize).min(HOSTILE.len() - 1)];
                match (next() * 9.0) as usize {
                    0 => m.pos.x = hostile,
                    1 => m.vel.y = hostile,
                    2 => m.radius = hostile,
                    3 => m.max_speed = hostile,
                    4 => m.pref_vel = Some(Vec2::new(hostile, 1.0)),
                    5 => m.pref_vel = None,
                    _ => {}
                }
                m
            })
            .collect()
    }

    /// Two boxes inside [`split_crowd`]'s square and a wall along its edge.
    fn split_obstacles() -> Obstacles {
        use crate::ObstaclesBuilder;
        let mut builder = ObstaclesBuilder::new();
        builder.add_box(Vec2::new(4.0, 4.0), Vec2::new(1.0, 1.5)).expect("a box");
        builder.add_box(Vec2::new(8.5, 7.0), Vec2::new(1.5, 0.5)).expect("a box");
        builder.add_wall(Vec2::new(0.0, 12.0), Vec2::new(12.0, 12.0)).expect("a wall");
        builder.build().expect("the obstacles build")
    }

    /// The bits of a solved slice, so `-0.0` against `0.0` and any NaN payload count as differences.
    fn bits(solved: &[Option<Vec2>]) -> Vec<Option<(u32, u32)>> {
        solved.iter().map(|v| v.map(|v| (v.x.to_bits(), v.y.to_bits()))).collect()
    }

    /// The split changes no velocity, which is what `begin` and `solve_range` are for. Over random
    /// crowds with hostile and unsteered members, a `sees` rule, caller walls and obstacles, every
    /// partition of the members into ranges (one member each, seven, sixty-four, all), solved in reverse
    /// order with the workers dealt in turn, over a slice that starts full of stale answers, gives bit for
    /// bit what `solve` and `solve_around` give — and so does the same thing on three real threads.
    #[test]
    fn any_split_of_a_step_gives_the_serial_answers() {
        let obstacles = split_obstacles();
        let mut params = CrowdParams::new(3.0);
        params.max_neighbours = 6;
        let dt = 1.0 / 30.0;
        let sees = |i: usize, j: usize| (i * 7 + j * 3) % 11 != 4;
        let walls = |i: usize, buf: &mut Vec<(Vec2, f32)>| {
            if i.is_multiple_of(3) {
                buf.push((Vec2::new(0.0, -1.0), 0.4));
            }
        };
        let stale = Some(Vec2::new(9.0, 9.0));
        for seed in 0..6u64 {
            for n in [0usize, 1, 2, 9, 120, 400] {
                let members = split_crowd(seed, n);
                for around in [false, true] {
                    let mut serial = Crowd::default();
                    let want = if around {
                        bits(serial.solve_around(&obstacles, &members, &params, dt, sees, walls))
                    } else {
                        bits(serial.solve(&members, &params, dt, sees, walls))
                    };
                    for chunk in [1usize, 7, 64, 1000] {
                        let mut crowd = Crowd::default();
                        let mut workers: Vec<CrowdWorker> = (0..3).map(|_| CrowdWorker::default()).collect();
                        let step = if around {
                            crowd.begin_around(&obstacles, &members, &params, dt)
                        } else {
                            crowd.begin(&members, &params, dt)
                        };

                        let mut got = vec![stale; n];
                        let starts: Vec<usize> = (0..n).step_by(chunk).rev().collect();
                        for (c, &first) in starts.iter().enumerate() {
                            let slice = &mut got[first..(first + chunk).min(n)];
                            step.solve_range(&mut workers[c % 3], first, slice, sees, walls);
                        }
                        assert_eq!(
                            bits(&got),
                            want,
                            "seed {seed}, {n} members, chunks of {chunk}, around {around}"
                        );

                        let mut threaded = vec![stale; n];
                        type Lane<'a> = Vec<(usize, &'a mut [Option<Vec2>])>;
                        let mut lanes: Vec<Lane> = (0..3).map(|_| Vec::new()).collect();
                        for (c, slice) in threaded.chunks_mut(chunk).enumerate() {
                            lanes[c % 3].push((c * chunk, slice));
                        }
                        std::thread::scope(|scope| {
                            for (worker, lane) in workers.iter_mut().zip(lanes) {
                                scope.spawn(move || {
                                    for (first, slice) in lane {
                                        step.solve_range(worker, first, slice, sees, walls);
                                    }
                                });
                            }
                        });
                        assert_eq!(
                            bits(&threaded),
                            want,
                            "seed {seed}, {n} members, threaded chunks of {chunk}, around {around}"
                        );
                    }
                }
            }
        }
    }

    /// A range is the caller's, and an unusable one is answered, never panicked on: a slot past the last
    /// member (or one whose index would overflow) is `None` and overwrites what was there, an empty slice
    /// is nothing to do, and a member that is not solved clears its stale slot too.
    #[test]
    fn a_range_past_the_members_leaves_the_extra_slots_unsolved() {
        let params = CrowdParams::new(4.0);
        let dt = 1.0 / 60.0;
        let members = [
            member(1, Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Some(Vec2::X)),
            member(2, Vec2::new(1.0, 0.0), Vec2::new(-1.0, 0.0), Some(-Vec2::X)),
            member(3, Vec2::new(0.0, 3.0), Vec2::ZERO, None),
        ];
        let mut crowd = Crowd::default();
        let want = crowd.solve(&members, &params, dt, |_, _| true, |_, _| {}).to_vec();
        assert!(want[0].is_some() && want[1].is_some() && want[2].is_none());

        let mut worker = CrowdWorker::default();
        let step = crowd.begin(&members, &params, dt);
        let stale = Some(Vec2::new(9.0, 9.0));
        let mut out = [stale; 6];
        step.solve_range(&mut worker, 1, &mut out, |_, _| true, |_, _| {});
        assert_eq!(&out[..2], &want[1..], "members 1 and 2, the unsteered one cleared");
        assert!(out[2..].iter().all(Option::is_none), "slots past the last member are unsolved: {out:?}");

        for first in [usize::MAX, usize::MAX - 1, 3] {
            let mut out = [stale; 3];
            step.solve_range(&mut worker, first, &mut out, |_, _| true, |_, _| {});
            assert!(out.iter().all(Option::is_none), "first {first}: {out:?}");
        }
        step.solve_range(&mut worker, 0, &mut [], |_, _| true, |_, _| {});
    }

    /// A host's task pool needs the step shared between tasks and a worker moved into one (Bevy's
    /// `Scope::spawn` takes `Future + 'scope + Send`): compile-time, so a field that is not thread-safe
    /// breaks the build here and not the host's.
    #[test]
    fn a_step_is_shared_and_a_worker_is_sent() {
        fn send_sync<T: Send + Sync + Copy>() {}
        fn send<T: Send>() {}
        send_sync::<CrowdStep<'static>>();
        send::<CrowdWorker>();
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
                    priority: 1.0,
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
