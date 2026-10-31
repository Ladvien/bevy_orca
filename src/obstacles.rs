//! Static obstacles: the caller's polygons and walls, turned into hard ORCA lines.
//!
//! [`new_velocity_in`](crate::new_velocity_in) takes the geometry that stops an agent as `walls`,
//! `(unit normal toward the wall, max approach speed)`, and leaves the geometry itself to the host:
//! which wall segments are near, which side of them the agent is on, what happens at a corner. This
//! module is that part. It is the obstacle branch of RVO2's `Agent::computeNewVelocity`
//! (van den Berg et al., Apache-2.0; see `NOTICE`), case for case: the velocity obstacle of a wall
//! segment is truncated at a horizon, and the line the agent must stay on the right side of is the
//! one tangent to it at the point nearest the agent's *current velocity*: a face of a wall, a
//! cutoff circle around a vertex, or a leg that lets the agent slide round a convex corner without
//! braking for it. An obstacle does not move, so the agent takes **all** of the avoidance (where a
//! pair of agents splits it), and the lines are hard: they are never relaxed, however dense the
//! crowd.
//!
//! The lines come out as `walls`, exactly as `new_velocity_in` takes them, so the solver is the same
//! solver: [`Obstacles::push_walls`] appends them for one agent, and
//! [`Crowd::solve_around`](crate::Crowd::solve_around) does it for every member.
//!
//! # Geometry
//!
//! An [`Obstacles`] set is built once by an [`ObstaclesBuilder`], which refuses what the solver
//! cannot use (a non-finite vertex, a zero-length edge, a polygon that crosses itself) and orients
//! what it is given. RVO2 asks for counter-clockwise polygons and sees an edge only from its right,
//! so a clockwise one is walked through without a word; here *what is solid* is stated by which
//! method the polygon is given to, and the winding is the builder's business:
//!
//! - [`add_solid`](ObstaclesBuilder::add_solid): the inside is solid (a building, a pillar);
//! - [`add_enclosure`](ObstaclesBuilder::add_enclosure): the inside is open floor and the outside is
//!   solid (a room, a courtyard, the edge of the map);
//! - [`add_wall`](ObstaclesBuilder::add_wall): a wall with no thickness, solid to neither side and
//!   passable from neither;
//! - [`add_box`](ObstaclesBuilder::add_box): an axis-aligned solid rectangle;
//! - [`add_cells`](ObstaclesBuilder::add_cells): a grid of solid cells, the way a tile map holds
//!   them, traced into the outline of each region of solid cells (a wall of any length is one edge,
//!   a block of cells is the four edges of [`add_box`](ObstaclesBuilder::add_box)).
//!
//! Obstacles do not make an agent go around them: the direction round a building is the agent's
//! preferred velocity, from whatever global navigator the host has (ORCA paper, section 5.4).
//!
//! # What is and is not seen
//!
//! As in RVO2, an agent sees an edge only from the open side, strictly. An agent whose centre is
//! already *inside* a solid has no lines from its edges and walks out or deeper as its wish says: a
//! spawn or a teleport into a building is the host's to repair, as final contact always is (see the
//! crate docs). An agent whose disc touches an edge from outside may not approach it at all, and
//! may slide along or leave.
//!
//! # Example
//!
//! ```
//! use bevy_math::Vec2;
//! use bevy_orca::{Agent, ObstaclesBuilder, OrcaScratch, new_velocity_in};
//!
//! // A building 8 m wide and 4 m deep, and an agent 1 m from its face, running straight at it.
//! let mut builder = ObstaclesBuilder::new();
//! builder.add_box(Vec2::ZERO, Vec2::new(4.0, 2.0)).expect("a box");
//! let obstacles = builder.build().expect("one box");
//!
//! let agent = Agent { pos: Vec2::new(0.0, -3.0), vel: Vec2::Y, radius: 0.3, avoids: true };
//! let mut scratch = OrcaScratch::default();
//! let mut walls = Vec::new();
//! // Half a second of look-ahead against obstacles, at a top speed of 1.5: the hard lines, in the
//! // form `new_velocity_in` takes its `walls`.
//! obstacles.push_walls(&mut scratch, &agent, 1.5, 0.5, &mut walls);
//! let v = new_velocity_in(&mut scratch, &agent, Vec2::Y * 1.5, &[], &walls, 2.0, 1.0 / 60.0, 1.5);
//! // It may still close on the face, but only as fast as the gap allows: (1 - 0.3) / 0.5 = 1.4.
//! assert!((v.y - 1.4).abs() < 1.0e-4 && v.x.abs() < 1.0e-4);
//! ```

use std::collections::BTreeMap;

use bevy_math::Vec2;

use crate::{Agent, EPSILON, OrcaScratch, det, usable_speed, usable_state};

/// Vertex coordinates beyond this magnitude are refused: the solver squares differences of
/// coordinates, and `2 · 1e18` squared still fits an `f32`.
const MAX_COORDINATE: f32 = 1.0e18;

/// More vertices than this are refused, so every index and link count fits a `u32`.
const MAX_VERTICES: usize = 1 << 26;

/// Why a polygon or wall was refused. The builder is left as it was.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleError {
    /// A polygon needs at least three vertices.
    TooFewVertices,
    /// A vertex has a NaN or infinite coordinate.
    NotFinite,
    /// A vertex coordinate is larger than `1e18` in magnitude, so a distance between vertices could
    /// overflow `f32` when squared.
    OutOfRange,
    /// Two consecutive vertices (the last and the first count as consecutive) are the same point, or
    /// so close that the edge between them has no direction an `f32` can hold.
    DegenerateEdge,
    /// The polygon has no inside: its vertices are collinear, or enclose no area an `f64` can see.
    /// For a wall with no thickness use [`ObstaclesBuilder::add_wall`].
    ZeroArea,
    /// Two edges of one polygon cross or touch. Reported by [`ObstaclesBuilder::build`], which
    /// checks every polygon once.
    SelfIntersecting,
    /// More than 2^26 vertices.
    TooMany,
    /// The grid given to [`ObstaclesBuilder::add_cells`] is not a grid: no `width` for cells, a
    /// number of cells that is not a multiple of `width`, or a `cell_size` that is not positive.
    BadGrid,
}

impl core::fmt::Display for ObstacleError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            ObstacleError::TooFewVertices => "a polygon needs at least three vertices",
            ObstacleError::NotFinite => "a vertex coordinate is not finite",
            ObstacleError::OutOfRange => "a vertex coordinate is beyond 1e18 in magnitude",
            ObstacleError::DegenerateEdge => "two consecutive vertices coincide",
            ObstacleError::ZeroArea => "the polygon encloses no area",
            ObstacleError::SelfIntersecting => "two edges of one polygon cross or touch",
            ObstacleError::TooMany => "more than 2^26 vertices",
            ObstacleError::BadGrid => "a cell grid needs a positive cell size and whole rows of cells",
        })
    }
}

impl core::error::Error for ObstacleError {}

/// One vertex of an obstacle, and with it the edge that leaves it. RVO2's `Obstacle`.
#[derive(Debug, Clone, Copy)]
struct Vertex {
    point: Vec2,
    /// The unit direction of the edge from this vertex to `next`.
    direction: Vec2,
    prev: u32,
    next: u32,
    /// The polygon this vertex belongs to, for the self-intersection check.
    ring: u32,
    /// Whether the obstacle turns left (or straight) at this vertex: a corner of the solid, where a
    /// cutoff circle is cast, rather than a corner of the open space, where the legs extend the
    /// edge. A wall's two ends are convex.
    convex: bool,
}

/// `det(a - c, b - a)`: positive when `c` is left of the directed line from `a` to `b`. RVO2's
/// `leftOf`.
#[inline]
fn left_of(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    det(a - c, b - a)
}

/// Collects polygons and walls into an [`Obstacles`] set. See the [module](self) for what each
/// method means.
#[derive(Debug, Clone, Default)]
pub struct ObstaclesBuilder {
    vertices: Vec<Vertex>,
    /// Where each polygon begins and how many vertices it has.
    rings: Vec<(u32, u32)>,
}

impl ObstaclesBuilder {
    /// An empty builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// A polygon whose inside is solid: a building, a pillar, a wall with thickness. The vertices
    /// may be listed in either winding.
    pub fn add_solid(&mut self, vertices: &[Vec2]) -> Result<&mut Self, ObstacleError> {
        let area = ring_area(vertices)?;
        self.push_ring(vertices, area < 0.0)?;
        Ok(self)
    }

    /// A polygon whose inside is open floor and whose outside is solid: a room, a courtyard, the
    /// boundary of the map. The vertices may be listed in either winding.
    pub fn add_enclosure(&mut self, vertices: &[Vec2]) -> Result<&mut Self, ObstacleError> {
        let area = ring_area(vertices)?;
        self.push_ring(vertices, area > 0.0)?;
        Ok(self)
    }

    /// A wall with no thickness from `a` to `b`: an agent may not cross it from either side.
    pub fn add_wall(&mut self, a: Vec2, b: Vec2) -> Result<&mut Self, ObstacleError> {
        self.push_ring(&[a, b], false)?;
        Ok(self)
    }

    /// An axis-aligned solid rectangle of the given `centre` and `half_extents`, a solid cell or a
    /// baked box. A half extent that is zero, negative or not finite is [`ObstacleError`].
    pub fn add_box(&mut self, centre: Vec2, half_extents: Vec2) -> Result<&mut Self, ObstacleError> {
        let h = half_extents;
        if !(h.x > 0.0 && h.y > 0.0) {
            // A negative or NaN extent is not a box; a zero one is a wall, and has its own method.
            return Err(if h.is_finite() { ObstacleError::ZeroArea } else { ObstacleError::NotFinite });
        }
        let corners = [
            centre + Vec2::new(-h.x, -h.y),
            centre + Vec2::new(h.x, -h.y),
            centre + Vec2::new(h.x, h.y),
            centre + Vec2::new(-h.x, h.y),
        ];
        self.add_solid(&corners)
    }

    /// A grid of solid cells, as a tile map or a flow-field's cost grid holds them, traced into the
    /// outline of each region of solid cells.
    ///
    /// `solid[row * width + column]` says whether the square cell `column` cells along `+x` and
    /// `row` along `+y` of `origin` is solid, `origin` being the lower-left corner of cell
    /// `(0, 0)` and `cell_size` the side of a cell, in the host's units; the grid is `width` cells
    /// wide and `solid.len() / width` high. **Outside the grid is open floor**: to make the edge of
    /// the map a wall, pad the grid with solid cells or add the map's rectangle with
    /// [`add_enclosure`](Self::add_enclosure).
    ///
    /// What is added is the boundary of the solid region, once: a polygon per region of cells that
    /// share an edge, and a hole (an enclosure) per pocket of open floor inside one, with every
    /// straight run of cells a single edge. A block of cells is the four edges
    /// [`add_box`](Self::add_box) gives it (and a walker beside it is handed the same lines): a
    /// 360 × 360 map of 900 buildings, each an 8 × 8 square of cells, is 3 600 edges built in about 4 ms,
    /// where one `add_box` per cell is 230 400, four for every cell however deep in a building.
    /// Two solid cells that meet only at a corner are two polygons that touch there, never one polygon
    /// that crosses itself, even when the region is joined elsewhere and the corner pinches off a pocket
    /// of floor: no disc passes through a point, so the gap between them is shut.
    ///
    /// No cell is solid means nothing is added, and an empty grid (no `width` and no cells) is fine.
    /// A `width` of 0 for a grid that has cells, a number of cells that is not a multiple of
    /// `width`, or a `cell_size` that is not positive is [`ObstacleError::BadGrid`]; a non-finite
    /// `origin` or `cell_size` is [`ObstacleError::NotFinite`]; a corner beyond `1e18` is
    /// [`ObstacleError::OutOfRange`]. An error leaves the builder as it was.
    ///
    /// ```
    /// use bevy_math::Vec2;
    /// use bevy_orca::ObstaclesBuilder;
    ///
    /// // A 5 × 4 map, 2 m cells: a room's four walls (a ring of solid cells around a 3 × 2 floor).
    /// let (w, h) = (5, 4);
    /// let solid: Vec<bool> =
    ///     (0..w * h).map(|i| (i % w == 0) || (i % w == w - 1) || (i / w == 0) || (i / w == h - 1)).collect();
    /// let mut builder = ObstaclesBuilder::new();
    /// builder.add_cells(Vec2::ZERO, 2.0, w, &solid).expect("a grid");
    /// let obstacles = builder.build().expect("a room");
    /// // 14 solid cells, but the outside of the ring is one polygon (4 edges) and the floor inside it
    /// // is one hole (4 edges).
    /// assert_eq!(obstacles.len(), 8);
    /// ```
    pub fn add_cells(
        &mut self,
        origin: Vec2,
        cell_size: f32,
        width: usize,
        solid: &[bool],
    ) -> Result<&mut Self, ObstacleError> {
        if !(origin.is_finite() && cell_size.is_finite()) {
            return Err(ObstacleError::NotFinite);
        }
        if cell_size <= 0.0
            || (width == 0 && !solid.is_empty())
            || (width != 0 && !solid.len().is_multiple_of(width))
        {
            return Err(ObstacleError::BadGrid);
        }
        let (vertices, rings) = (self.vertices.len(), self.rings.len());
        let (ox, oy, size) = (f64::from(origin.x), f64::from(origin.y), f64::from(cell_size));
        let mut points: Vec<Vec2> = Vec::new();
        let traced = trace_cells(width, solid, |corners, area| {
            points.clear();
            points.extend(
                corners
                    .iter()
                    .map(|&(x, y)| Vec2::new((ox + x as f64 * size) as f32, (oy + y as f64 * size) as f32)),
            );
            // A positive area is the outside of a region of solid cells, a negative one a pocket of
            // open floor inside it: the same two things `add_solid` and `add_enclosure` say.
            if area > 0 { self.add_solid(&points) } else { self.add_enclosure(&points) }.map(|_| ())
        });
        if let Err(error) = traced {
            self.vertices.truncate(vertices);
            self.rings.truncate(rings);
            return Err(error);
        }
        Ok(self)
    }

    /// Appends the polygon `ring`, reversed if `reverse`, after checking it. Two vertices are a wall.
    fn push_ring(&mut self, ring: &[Vec2], reverse: bool) -> Result<(), ObstacleError> {
        check_ring(ring)?;
        let n = ring.len();
        let base = self.vertices.len();
        if base + n > MAX_VERTICES {
            return Err(ObstacleError::TooMany);
        }
        let at = |i: usize| ring[if reverse { n - 1 - i } else { i }];
        let polygon = u32::try_from(self.rings.len()).map_err(|_| ObstacleError::TooMany)?;
        for i in 0..n {
            let (prev, here, next) = (at((i + n - 1) % n), at(i), at((i + 1) % n));
            let edge = next - here;
            self.vertices.push(Vertex {
                point: here,
                direction: edge / edge.length(),
                prev: (base + (i + n - 1) % n) as u32,
                next: (base + (i + 1) % n) as u32,
                ring: polygon,
                // RVO2: a wall's two vertices are both convex; otherwise a vertex is convex when
                // the obstacle does not turn right there, and a straight one counts.
                convex: n == 2 || left_of(prev, here, next) >= 0.0,
            });
        }
        self.rings.push((base as u32, n as u32));
        Ok(())
    }

    /// Indexes the geometry and checks that no polygon crosses itself.
    pub fn build(self) -> Result<Obstacles, ObstacleError> {
        let grid = Grid::build(&self.vertices);
        grid.check_rings(&self.vertices, &self.rings)?;
        Ok(Obstacles { vertices: self.vertices, grid })
    }
}

/// What a polygon's vertices must satisfy before its area means anything: three or more of them, all
/// finite and in range, no two consecutive ones the same.
fn check_ring(ring: &[Vec2]) -> Result<(), ObstacleError> {
    if ring.len() < 2 {
        return Err(ObstacleError::TooFewVertices);
    }
    for v in ring {
        if !v.is_finite() {
            return Err(ObstacleError::NotFinite);
        }
        if v.abs().max_element() > MAX_COORDINATE {
            return Err(ObstacleError::OutOfRange);
        }
    }
    let n = ring.len();
    // Two vertices are one wall, whose second edge is the first reversed: n edges for n > 2, and the
    // pair for n == 2 (checked once, but the same edge both ways).
    for i in 0..n {
        let edge = ring[(i + 1) % n] - ring[i];
        let length_squared = edge.length_squared();
        if !(length_squared >= f32::MIN_POSITIVE && length_squared.is_finite())
            || !(edge / edge.length()).is_finite()
        {
            return Err(ObstacleError::DegenerateEdge);
        }
    }
    Ok(())
}

/// Twice the signed area of a polygon, positive for counter-clockwise, summed in `f64` from its first
/// vertex so that a polygon far from the origin loses nothing to cancellation. Errors if the
/// polygon is not valid or has no area.
fn ring_area(ring: &[Vec2]) -> Result<f64, ObstacleError> {
    if ring.len() < 3 {
        return Err(ObstacleError::TooFewVertices);
    }
    check_ring(ring)?;
    let origin = ring[0];
    let mut area = 0.0f64;
    for pair in ring.windows(2) {
        let (a, b) = (pair[0] - origin, pair[1] - origin);
        area += f64::from(a.x) * f64::from(b.y) - f64::from(a.y) * f64::from(b.x);
    }
    if area == 0.0 || !area.is_finite() {
        return Err(ObstacleError::ZeroArea);
    }
    Ok(area)
}

/// The four directions of a lattice edge, counter-clockwise from east, as `(dx, dy)`.
const STEPS: [(i64, i64); 4] = [(1, 0), (0, 1), (-1, 0), (0, -1)];

/// The outline of the solid cells of a `width`-wide grid, as loops of lattice corners, handed to
/// `emit` one at a time with the loop's doubled signed area (positive for a counter-clockwise loop,
/// the outside of a region; negative for a clockwise one, a pocket of open floor inside it).
///
/// Every boundary between a solid cell and open floor (the grid's edge is open) is a unit lattice edge
/// directed so that the solid is on its left, and every lattice vertex keeps the set of those that
/// leave it. Loops are walked from the first vertex in row-major order that still has an edge,
/// each edge used once, and at every vertex the walk takes the leftmost unused edge (left, then
/// straight, then right), the one fixed rule Recast's `walkContour` also walks a region of cells by.
/// Corners are the only vertices kept, so a straight wall is one edge.
///
/// A vertex with two edges leaving it is a *pinch*: two solid cells meet there only at a corner
/// (a checkerboard), and the walk takes the pairing that keeps each cell's own boundary. When the
/// two cells are one region joined elsewhere the walk then comes to the pinch twice in one loop,
/// which would be a polygon that touches itself, so such a loop is cut there into the loops it is
/// made of: an outside and the pocket of open floor it pinches off, or the two halves of a figure
/// eight. Every loop emitted visits each vertex once.
///
/// `solid` is row-major, `width` cells to a row, and its length a multiple of `width`.
fn trace_cells(
    width: usize,
    solid: &[bool],
    mut emit: impl FnMut(&[(i64, i64)], i128) -> Result<(), ObstacleError>,
) -> Result<(), ObstacleError> {
    if width == 0 || !solid.iter().any(|&cell| cell) {
        return Ok(());
    }
    let height = solid.len() / width;
    let stride = width + 1;
    // One byte per lattice vertex: bit `d` is set when a boundary edge leaves it in direction `d`.
    let mut leaving = vec![0u8; stride * (height + 1)];
    let open = |column: usize, row: usize, dx: i64, dy: i64| {
        let (c, r) = (column as i64 + dx, row as i64 + dy);
        let inside = c >= 0 && r >= 0 && (c as usize) < width && (r as usize) < height;
        !(inside && solid.get(r as usize * width + c as usize).copied().unwrap_or(false))
    };
    for (row, cells) in solid.chunks_exact(width).enumerate() {
        for (column, &cell) in cells.iter().enumerate() {
            if !cell {
                continue;
            }
            // South side: east along it; east side: north; north side: west; west side: south.
            for (neighbour, corner, direction) in [
                ((0, -1), (column, row), 0u8),
                ((1, 0), (column + 1, row), 1),
                ((0, 1), (column + 1, row + 1), 2),
                ((-1, 0), (column, row + 1), 3),
            ] {
                if open(column, row, neighbour.0, neighbour.1)
                    && let Some(bits) = leaving.get_mut(corner.1 * stride + corner.0)
                {
                    *bits |= 1 << direction;
                }
            }
        }
    }
    // The pinches, in order: only these vertices can be visited twice by one walk.
    let pinches: Vec<usize> =
        leaving.iter().enumerate().filter(|(_, bits)| bits.count_ones() > 1).map(|(i, _)| i).collect();

    let mut walk: Vec<(i64, i64)> = Vec::new();
    let mut corners: Vec<(i64, i64)> = Vec::new();
    for start in 0..leaving.len() {
        while let Some(&bits) = leaving.get(start)
            && bits != 0
        {
            let first = bits.trailing_zeros() as usize;
            let (mut x, mut y) = ((start % stride) as i64, (start / stride) as i64);
            let mut direction = first;
            walk.clear();
            loop {
                walk.push((x, y));
                let here = y as usize * stride + x as usize;
                if (here, direction) != (start, first)
                    && let Some(bits) = leaving.get_mut(here)
                {
                    *bits &= !(1 << direction);
                }
                let (dx, dy) = STEPS.get(direction).copied().unwrap_or((0, 0));
                (x, y) = (x + dx, y + dy);
                let there = y as usize * stride + x as usize;
                let available = leaving.get(there).copied().unwrap_or(0);
                let Some(next) = [(direction + 1) % 4, direction, (direction + 3) % 4]
                    .into_iter()
                    .find(|&d| available & (1 << d) != 0)
                else {
                    break;
                };
                if there == start && next == first {
                    break;
                }
                direction = next;
            }
            if let Some(bits) = leaving.get_mut(start) {
                *bits &= !(1 << first);
            }

            // Keep the corners: the vertices where the loop turns.
            let n = walk.len();
            let before = walk.iter().cycle().skip(n - 1);
            let after = walk.iter().cycle().skip(1);
            corners.clear();
            corners.extend(
                before.zip(walk.iter()).zip(after).filter_map(|((p, c), a)| {
                    ((c.0 - p.0, c.1 - p.1) != (a.0 - c.0, a.1 - c.1)).then_some(*c)
                }),
            );
            emit_loops(&corners, stride, &pinches, &mut emit)?;
        }
    }
    Ok(())
}

/// Twice the signed area of a loop of lattice corners: positive when it runs counter-clockwise.
fn doubled_area(corners: &[(i64, i64)]) -> i128 {
    let next = corners.iter().cycle().skip(1);
    corners
        .iter()
        .zip(next)
        .map(|(a, b)| i128::from(a.0) * i128::from(b.1) - i128::from(b.0) * i128::from(a.1))
        .sum()
}

/// Hands `emit` the loop `corners`, or, when it passes a pinch vertex twice (see [`trace_cells`]), the
/// loops it is made of, each cut out at the vertex it came back to. `pinches` are the lattice
/// vertices (`y * stride + x`) that can repeat, sorted.
fn emit_loops(
    corners: &[(i64, i64)],
    stride: usize,
    pinches: &[usize],
    emit: &mut impl FnMut(&[(i64, i64)], i128) -> Result<(), ObstacleError>,
) -> Result<(), ObstacleError> {
    let is_pinch = |c: &(i64, i64)| pinches.binary_search(&(c.1 as usize * stride + c.0 as usize)).is_ok();
    if !corners.iter().any(is_pinch) {
        return emit(corners, doubled_area(corners));
    }
    // The loop so far, and where each pinch on it is. Coming back to one closes a loop of its own.
    let mut path: Vec<(i64, i64)> = Vec::new();
    let mut on_path: BTreeMap<(i64, i64), usize> = BTreeMap::new();
    for &corner in corners {
        if !is_pinch(&corner) {
            path.push(corner);
        } else if let Some(&first) = on_path.get(&corner) {
            let mut closed = vec![corner];
            closed.extend(path.split_off(first + 1));
            for gone in closed.iter().skip(1) {
                on_path.remove(gone);
            }
            emit(&closed, doubled_area(&closed))?;
        } else {
            on_path.insert(corner, path.len());
            path.push(corner);
        }
    }
    emit(&path, doubled_area(&path))
}

/// A set of static obstacles, indexed for a query per agent. Build one with [`ObstaclesBuilder`]; it
/// does not change afterwards, so any number of crowds and threads may read it.
///
/// An empty set (`Obstacles::default()`) has no lines to give, and costs nothing to ask.
#[derive(Debug, Clone, Default)]
pub struct Obstacles {
    vertices: Vec<Vertex>,
    grid: Grid,
}

impl Obstacles {
    /// The number of edges (a wall counts two, one each way).
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Whether the set has no edges.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Appends to `walls` the hard lines `agent` must satisfy this step, in the form
    /// [`new_velocity_in`](crate::new_velocity_in) takes them: `(unit normal, c)` meaning
    /// `v · normal <= c`. `walls` is not cleared, so a caller's own walls may be in it already.
    ///
    /// `time_horizon` is how many seconds ahead a collision with an obstacle is anticipated, and it
    /// should be at least the step `dt`, or an agent can cross a wall within one step. It is
    /// ordinarily shorter than the agent horizon: an agent should not be "shy" of a wall it must
    /// pass close to in order to avoid another agent (ORCA paper, section 5.4). Only edges within
    /// `time_horizon · max_speed + radius` of the agent are considered.
    ///
    /// Gives nothing for an agent with no usable state or speed, or a horizon that is not a positive
    /// finite number whose reciprocal is finite, which is the answer
    /// [`new_velocity_in`](crate::new_velocity_in) gives such input without them. Every line pushed
    /// is finite and has a unit normal.
    pub fn push_walls(
        &self,
        scratch: &mut OrcaScratch,
        agent: &Agent,
        max_speed: f32,
        time_horizon: f32,
        walls: &mut Vec<(Vec2, f32)>,
    ) {
        if self.vertices.is_empty()
            || !usable_speed(max_speed)
            || !usable_state(agent.pos, agent.vel, agent.radius)
        {
            return;
        }
        let inv_time_horizon = 1.0 / time_horizon;
        if !(time_horizon > 0.0 && time_horizon.is_finite() && inv_time_horizon.is_finite()) {
            return;
        }
        let (pos, velocity, radius) = (agent.pos, agent.vel, agent.radius);
        let range = time_horizon * max_speed + radius;
        let range_squared = range * range;
        if range.is_nan() || range_squared.is_nan() {
            return;
        }

        // The edges in range that the agent is on the right side of, nearest first, the edge's own
        // index breaking a tie. An edge crossing several grid cells is found once per cell.
        let near = &mut scratch.near_edges;
        near.clear();
        self.grid.for_each_edge(pos, range, |e| {
            let Some(&v1) = self.vertices.get(e as usize) else {
                return;
            };
            let Some(&v2) = self.vertices.get(v1.next as usize) else {
                return;
            };
            let edge = v2.point - v1.point;
            if left_of(v1.point, v2.point, pos) >= 0.0 {
                return;
            }
            let along = ((pos - v1.point).dot(edge) / edge.length_squared()).clamp(0.0, 1.0);
            let distance_squared = (pos - (v1.point + edge * along)).length_squared();
            if distance_squared < range_squared {
                near.push((distance_squared, e));
            }
        });
        near.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        near.dedup();

        let first = walls.len();
        for &(_, e) in near.iter() {
            let Some(&v1) = self.vertices.get(e as usize) else {
                continue;
            };
            let Some(&v2) = self.vertices.get(v1.next as usize) else {
                continue;
            };
            obstacle_lines(&self.vertices, e, v1, v2, pos, velocity, radius, inv_time_horizon, first, walls);
        }
    }
}

/// Appends the line (or lines: at most one) that the edge `v1 → v2` casts on an agent, to `walls`.
/// `first` is where this agent's obstacle lines begin in `walls`: "already covered" is judged
/// against those only, never against a caller's own walls ahead of them.
///
/// RVO2's `computeNewVelocity`, lines 275–535, with its variable names; `i1` is the index of `v1` in
/// `vertices`, which is how RVO2's `obstacle1 == obstacle2` is spelled once either may have been
/// replaced by the other.
#[allow(clippy::too_many_arguments)]
fn obstacle_lines(
    vertices: &[Vertex],
    i1: u32,
    v1: Vertex,
    v2: Vertex,
    position: Vec2,
    velocity: Vec2,
    radius: f32,
    inv_time_horizon_obst: f32,
    first: usize,
    walls: &mut Vec<(Vec2, f32)>,
) {
    let relative_position1 = v1.point - position;
    let relative_position2 = v2.point - position;

    // Is the velocity obstacle of this edge already inside one cast by an earlier, nearer one?
    let covered = walls.get(first..).unwrap_or(&[]).iter().any(|&(normal, c)| {
        let (direction, point) = (normal.perp(), normal * c);
        det(inv_time_horizon_obst * relative_position1 - point, direction) - inv_time_horizon_obst * radius
            >= -EPSILON
            && det(inv_time_horizon_obst * relative_position2 - point, direction)
                - inv_time_horizon_obst * radius
                >= -EPSILON
    });
    if covered {
        return;
    }

    let dist_sq1 = relative_position1.length_squared();
    let dist_sq2 = relative_position2.length_squared();
    let radius_sq = radius * radius;

    let obstacle_vector = v2.point - v1.point;
    let s = (-relative_position1).dot(obstacle_vector) / obstacle_vector.length_squared();
    let dist_sq_line = (-relative_position1 - s * obstacle_vector).length_squared();

    if s < 0.0 && dist_sq1 <= radius_sq {
        // Collision with the left vertex. Ignore it if non-convex.
        if v1.convex {
            push_line(walls, Vec2::ZERO, toward_perpendicular(relative_position1, -v1.direction));
        }
        return;
    }
    if s > 1.0 && dist_sq2 <= radius_sq {
        // Collision with the right vertex. Ignore it if non-convex, or if the neighbouring obstacle
        // takes care of it.
        if v2.convex && det(relative_position2, v2.direction) >= 0.0 {
            push_line(walls, Vec2::ZERO, toward_perpendicular(relative_position2, -v2.direction));
        }
        return;
    }
    if (0.0..=1.0).contains(&s) && dist_sq_line <= radius_sq {
        // Collision with the edge itself.
        push_line(walls, Vec2::ZERO, -v1.direction);
        return;
    }

    // No collision. Work out the legs of the velocity obstacle. Obliquely viewed, both legs can come
    // from a single vertex, and a non-convex vertex's leg extends the cutoff line.
    let (mut i1, mut i2) = (i1, v1.next);
    let (mut v1, mut v2) = (v1, v2);
    let (mut left_leg_direction, mut right_leg_direction);
    // Which vertex defines the velocity obstacle when the edge is viewed obliquely: the legs a "foreign"
    // leg is replaced by, below, can then be this very edge's own line.
    let (mut viewed_at_left_vertex, mut viewed_at_right_vertex) = (false, false);

    if s < 0.0 && dist_sq_line <= radius_sq {
        // Obliquely viewed, so that the left vertex defines the velocity obstacle.
        if !v1.convex {
            return;
        }
        i2 = i1;
        viewed_at_left_vertex = true;
        v2 = v1;
        let leg1 = (dist_sq1 - radius_sq).max(0.0).sqrt();
        left_leg_direction = Vec2::new(
            relative_position1.x * leg1 - relative_position1.y * radius,
            relative_position1.x * radius + relative_position1.y * leg1,
        ) / dist_sq1;
        right_leg_direction = Vec2::new(
            relative_position1.x * leg1 + relative_position1.y * radius,
            -relative_position1.x * radius + relative_position1.y * leg1,
        ) / dist_sq1;
    } else if s > 1.0 && dist_sq_line <= radius_sq {
        // Obliquely viewed, so that the right vertex defines the velocity obstacle.
        if !v2.convex {
            return;
        }
        i1 = i2;
        viewed_at_right_vertex = true;
        v1 = v2;
        let leg2 = (dist_sq2 - radius_sq).max(0.0).sqrt();
        left_leg_direction = Vec2::new(
            relative_position2.x * leg2 - relative_position2.y * radius,
            relative_position2.x * radius + relative_position2.y * leg2,
        ) / dist_sq2;
        right_leg_direction = Vec2::new(
            relative_position2.x * leg2 + relative_position2.y * radius,
            -relative_position2.x * radius + relative_position2.y * leg2,
        ) / dist_sq2;
    } else {
        // The usual situation.
        if v1.convex {
            let leg1 = (dist_sq1 - radius_sq).max(0.0).sqrt();
            left_leg_direction = Vec2::new(
                relative_position1.x * leg1 - relative_position1.y * radius,
                relative_position1.x * radius + relative_position1.y * leg1,
            ) / dist_sq1;
        } else {
            // The left vertex is non-convex: the left leg extends the cutoff line.
            left_leg_direction = -v1.direction;
        }
        if v2.convex {
            let leg2 = (dist_sq2 - radius_sq).max(0.0).sqrt();
            right_leg_direction = Vec2::new(
                relative_position2.x * leg2 + relative_position2.y * radius,
                -relative_position2.x * radius + relative_position2.y * leg2,
            ) / dist_sq2;
        } else {
            // The right vertex is non-convex: the right leg extends the cutoff line.
            right_leg_direction = v1.direction;
        }
    }

    // A leg can never point into the neighbouring edge at a convex vertex: take that edge's cutoff
    // line instead. A velocity projected on such a "foreign" leg adds no constraint, because the
    // neighbouring edge casts that line itself.
    //
    // **Not for a wall** (the one place this differs from RVO2). A wall has no inside for a leg to
    // point into, and its neighbouring edge is its own other side, which an agent on this side
    // cannot see and which therefore casts nothing. RVO2 applies the rule anyway, and an agent a few
    // ulps from a wall's face, moving along it, is then handed no line at all and walks into the
    // wall: 191 of 168 813 such agents in RVO2 built as shipped (no FMA), by up to a full step,
    // 4.3 cm. A wall keeps the legs of its vertices as they come.
    //
    // **Not when the "neighbour" is this edge itself** (the second place). Viewed obliquely, this edge
    // is not casting its own face line: it defines the velocity obstacle by one vertex, and the leg
    // that lies along its own face (the left leg at its right vertex, the right leg at its left one)
    // is replaced by the face's direction and then skipped as the neighbour's, which nobody casts. An
    // agent a few ulps from a convex corner on the extension of a face, creeping along it with its
    // wish into the corner, is then handed no line and steps into the corner by a whole step: 28
    // agent-steps in 21.6 million of a crowd on random caves, 4.7 cm. The replaced leg is cast.
    let Some(&left_neighbour) = vertices.get(v1.prev as usize) else {
        return;
    };
    let is_wall = v1.prev == v1.next;
    let (mut is_left_leg_foreign, mut is_right_leg_foreign) = (false, false);
    if !is_wall && v1.convex && det(left_leg_direction, -left_neighbour.direction) >= 0.0 {
        // The left leg points into the obstacle.
        left_leg_direction = -left_neighbour.direction;
        is_left_leg_foreign = !viewed_at_right_vertex;
    }
    if !is_wall && v2.convex && det(right_leg_direction, v2.direction) <= 0.0 {
        // The right leg points into the obstacle.
        right_leg_direction = v2.direction;
        is_right_leg_foreign = !viewed_at_left_vertex;
    }

    // The cutoff centres.
    let left_cutoff = inv_time_horizon_obst * (v1.point - position);
    let right_cutoff = inv_time_horizon_obst * (v2.point - position);
    let cutoff_vector = right_cutoff - left_cutoff;

    // Project the current velocity on the velocity obstacle: first, is it projected on a cutoff
    // circle?
    let t = if i1 == i2 {
        0.5
    } else {
        (velocity - left_cutoff).dot(cutoff_vector) / cutoff_vector.length_squared()
    };
    let t_left = (velocity - left_cutoff).dot(left_leg_direction);
    let t_right = (velocity - right_cutoff).dot(right_leg_direction);

    if (t < 0.0 && t_left < 0.0) || (i1 == i2 && t_left < 0.0 && t_right < 0.0) {
        // Project on the left cutoff circle.
        let unit_w = away_from(velocity - left_cutoff, -relative_position1);
        push_line(
            walls,
            left_cutoff + radius * inv_time_horizon_obst * unit_w,
            Vec2::new(unit_w.y, -unit_w.x),
        );
        return;
    }
    if t > 1.0 && t_right < 0.0 {
        // Project on the right cutoff circle.
        let unit_w = away_from(velocity - right_cutoff, -relative_position2);
        push_line(
            walls,
            right_cutoff + radius * inv_time_horizon_obst * unit_w,
            Vec2::new(unit_w.y, -unit_w.x),
        );
        return;
    }

    // Project on the left leg, the right leg, or the cutoff line, whichever is closest.
    let dist_sq_cutoff = if t < 0.0 || t > 1.0 || i1 == i2 {
        f32::INFINITY
    } else {
        (velocity - (left_cutoff + t * cutoff_vector)).length_squared()
    };
    let dist_sq_left = if t_left < 0.0 {
        f32::INFINITY
    } else {
        (velocity - (left_cutoff + t_left * left_leg_direction)).length_squared()
    };
    let dist_sq_right = if t_right < 0.0 {
        f32::INFINITY
    } else {
        (velocity - (right_cutoff + t_right * right_leg_direction)).length_squared()
    };

    if dist_sq_cutoff <= dist_sq_left && dist_sq_cutoff <= dist_sq_right {
        // Project on the cutoff line.
        let direction = -v1.direction;
        let point = left_cutoff + radius * inv_time_horizon_obst * Vec2::new(-direction.y, direction.x);
        push_line(walls, point, direction);
        return;
    }
    if dist_sq_left <= dist_sq_right {
        // Project on the left leg.
        if is_left_leg_foreign {
            return;
        }
        let direction = left_leg_direction;
        let point = left_cutoff + radius * inv_time_horizon_obst * Vec2::new(-direction.y, direction.x);
        push_line(walls, point, direction);
        return;
    }
    // Project on the right leg.
    if is_right_leg_foreign {
        return;
    }
    let direction = -right_leg_direction;
    let point = right_cutoff + radius * inv_time_horizon_obst * Vec2::new(-direction.y, direction.x);
    push_line(walls, point, direction);
}

/// The unit vector along `v`, or, when `v` has no direction (RVO2 divides by its length and gets a
/// NaN; here it is the one case where a choice is made), along `fallback`: the direction from the
/// vertex to the agent, which holds the agent off it.
#[inline]
fn away_from(v: Vec2, fallback: Vec2) -> Vec2 {
    let length_squared = v.length_squared();
    if length_squared >= f32::MIN_POSITIVE && length_squared.is_finite() {
        v / length_squared.sqrt()
    } else {
        fallback.normalize_or_zero()
    }
}

/// The unit vector along `perp(v)` (RVO2's `normalize(Vector2(-v.y, v.x))`), or `fallback` when `v`
/// is zero: the agent's centre is exactly on the vertex, and the edge's own line is the safe answer.
#[inline]
fn toward_perpendicular(v: Vec2, fallback: Vec2) -> Vec2 {
    let length_squared = v.length_squared();
    if length_squared >= f32::MIN_POSITIVE && length_squared.is_finite() {
        Vec2::new(-v.y, v.x) / length_squared.sqrt()
    } else {
        fallback
    }
}

/// Appends the half-plane left of the line through `point` along `direction`, as `(normal, c)` with
/// `v · normal <= c`: RVO2's `Line` is `det(direction, v - point) >= 0`, and `normal` is
/// `(direction.y, -direction.x)`. A line that is not finite, or whose direction is not a direction,
/// is not pushed.
#[inline]
fn push_line(walls: &mut Vec<(Vec2, f32)>, point: Vec2, direction: Vec2) {
    let length_squared = direction.length_squared();
    if !(length_squared >= f32::MIN_POSITIVE && length_squared.is_finite()) {
        return;
    }
    let direction =
        if (length_squared - 1.0).abs() > 1.0e-6 { direction / length_squared.sqrt() } else { direction };
    let normal = Vec2::new(direction.y, -direction.x);
    let c = normal.dot(point);
    if normal.is_finite() && c.is_finite() {
        walls.push((normal, c));
    }
}

/// The shape of a uniform grid: where it starts, how finely it divides the plane and how many
/// cells it has. Copied out of a [`Grid`] so that building one can fill its own counts.
#[derive(Debug, Clone, Copy, Default)]
struct Lattice {
    origin: Vec2,
    inv_cell: f32,
    columns: usize,
    rows: usize,
}

/// A cell's fraction of padding on every side of an edge's cells, so that rounding in the walk can
/// never leave an edge out of a cell it touches.
const CELL_PAD: f32 = 1.0e-3;

impl Lattice {
    /// Calls `f` with the index of every cell the edge `a → b` crosses, padded: for each column the
    /// edge passes through, the rows between where it enters and where it leaves. `columns` and
    /// `rows` are at least 1.
    fn cells_of(&self, a: Vec2, b: Vec2, mut f: impl FnMut(usize)) {
        let (ga, gb) = ((a - self.origin) * self.inv_cell, (b - self.origin) * self.inv_cell);
        let (x0, x1) = (ga.x.min(gb.x), ga.x.max(gb.x));
        // Infinite (or, for an edge that is a hair off vertical, too steep to hold) when vertical: then
        // its rows are simply those of its two ends.
        let slope = (gb.y - ga.y) / (gb.x - ga.x);
        let vertical = !slope.is_finite();
        let first = ((x0 - CELL_PAD).floor().max(0.0) as usize).min(self.columns.saturating_sub(1));
        let last = ((x1 + CELL_PAD).floor().max(0.0) as usize).min(self.columns.saturating_sub(1));
        for column in first..=last {
            // Where the edge is, in y, over the part of it that lies in this column.
            let (lo_x, hi_x) = ((column as f32).max(x0), ((column + 1) as f32).min(x1));
            let (mut y0, mut y1) = if vertical {
                (ga.y.min(gb.y), ga.y.max(gb.y))
            } else {
                let at = |x: f32| ga.y + slope * (x - ga.x);
                let (p, q) = (at(lo_x.min(hi_x)), at(hi_x.max(lo_x)));
                if p.is_nan() || q.is_nan() { (ga.y.min(gb.y), ga.y.max(gb.y)) } else { (p.min(q), p.max(q)) }
            };
            y0 -= CELL_PAD;
            y1 += CELL_PAD;
            let row_first = (y0.floor().max(0.0) as usize).min(self.rows.saturating_sub(1));
            let row_last = (y1.floor().max(0.0) as usize).min(self.rows.saturating_sub(1));
            for row in row_first..=row_last {
                f(row * self.columns + column);
            }
        }
    }
}

/// A uniform grid over the edges, in compressed rows: each edge is linked into every cell it
/// crosses, so a query reads the edges of the cells under a box and nothing else. Built once, so a
/// long wall that crosses many cells costs its length in links at build and nothing per step.
#[derive(Debug, Clone, Default)]
struct Grid {
    lattice: Lattice,
    /// `edges[starts[c]..starts[c + 1]]` are the edges linked into cell `c`, row-major.
    starts: Vec<u32>,
    edges: Vec<u32>,
}

impl Grid {
    fn build(vertices: &[Vertex]) -> Grid {
        if vertices.is_empty() {
            return Grid::default();
        }
        let (mut lo, mut hi) = (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY));
        for v in vertices {
            lo = lo.min(v.point);
            hi = hi.max(v.point);
        }
        let extent = hi - lo;
        let n = vertices.len();
        let edge_of = |v: &Vertex| (v.point, vertices.get(v.next as usize).map_or(v.point, |w| w.point));
        // About one cell per edge to begin with; a cell size that links more than 32 cells per edge
        // on average (long walls, many of them) is doubled until it does not.
        let mut cell = extent.x.max(extent.y) / (n as f32).sqrt();
        loop {
            let inv_cell = 1.0 / cell;
            let columns = ((extent.x * inv_cell).floor() as usize).saturating_add(1);
            let rows = ((extent.y * inv_cell).floor() as usize).saturating_add(1);
            let lattice = Lattice { origin: lo, inv_cell, columns, rows };
            let cells = columns.saturating_mul(rows);
            if cells <= 4 * n + 64 {
                let mut starts = vec![0u32; cells + 1];
                for v in vertices {
                    let (a, b) = edge_of(v);
                    lattice.cells_of(a, b, |c| starts[c + 1] += 1);
                }
                for c in 0..cells {
                    starts[c + 1] += starts[c];
                }
                let links = starts[cells] as usize;
                if links <= 32 * n + 256 {
                    let mut fill = starts.clone();
                    let mut edges = vec![0u32; links];
                    for (e, v) in vertices.iter().enumerate() {
                        let (a, b) = edge_of(v);
                        lattice.cells_of(a, b, |c| {
                            edges[fill[c] as usize] = e as u32;
                            fill[c] += 1;
                        });
                    }
                    return Grid { lattice, starts, edges };
                }
            }
            cell *= 2.0;
        }
    }

    /// Calls `f` with every edge linked into a cell under the box `pos ± range`, a few of them more
    /// than once.
    fn for_each_edge(&self, pos: Vec2, range: f32, mut f: impl FnMut(u32)) {
        let Lattice { origin, inv_cell, columns, rows } = self.lattice;
        if columns == 0 || rows == 0 {
            return;
        }
        let lo = (pos - Vec2::splat(range) - origin) * inv_cell;
        let hi = (pos + Vec2::splat(range) - origin) * inv_cell;
        // A box wholly outside the grid, or with a NaN edge, reaches no cell.
        if !(hi.x >= 0.0 && hi.y >= 0.0 && lo.x < columns as f32 && lo.y < rows as f32) {
            return;
        }
        let first_column = lo.x.floor().max(0.0) as usize;
        let last_column = (hi.x.floor().max(0.0) as usize).min(columns - 1);
        let first_row = lo.y.floor().max(0.0) as usize;
        let last_row = (hi.y.floor().max(0.0) as usize).min(rows - 1);
        for row in first_row..=last_row {
            // A row's cells in the box are one run of cells, and so one run of links.
            let (from, to) = (row * columns + first_column, row * columns + last_column);
            let (Some(&start), Some(&end)) = (self.starts.get(from), self.starts.get(to + 1)) else {
                continue;
            };
            if let Some(run) = self.edges.get(start as usize..end as usize) {
                run.iter().for_each(|&e| f(e));
            }
        }
    }

    /// Whether any two edges of one polygon cross or touch, other than neighbours at their shared
    /// vertex. Two edges that touch share a cell, so only the edges in each cell are compared.
    fn check_rings(&self, vertices: &[Vertex], rings: &[(u32, u32)]) -> Result<(), ObstacleError> {
        // A wall (two vertices) is its own edge both ways, and a triangle's edges all neighbour.
        let checked = |ring: u32| rings.get(ring as usize).is_some_and(|&(_, n)| n > 3);
        for cell in 0..self.lattice.columns.saturating_mul(self.lattice.rows) {
            let (Some(&start), Some(&end)) = (self.starts.get(cell), self.starts.get(cell + 1)) else {
                continue;
            };
            let Some(run) = self.edges.get(start as usize..end as usize) else {
                continue;
            };
            for (k, &i) in run.iter().enumerate() {
                let Some(&vi) = vertices.get(i as usize) else {
                    continue;
                };
                if !checked(vi.ring) {
                    continue;
                }
                for &j in &run[k + 1..] {
                    let Some(&vj) = vertices.get(j as usize) else {
                        continue;
                    };
                    if vi.ring != vj.ring || i == j || vi.next == j || vj.next == i {
                        continue;
                    }
                    let (Some(&wi), Some(&wj)) =
                        (vertices.get(vi.next as usize), vertices.get(vj.next as usize))
                    else {
                        continue;
                    };
                    if segments_meet(vi.point, wi.point, vj.point, wj.point) {
                        return Err(ObstacleError::SelfIntersecting);
                    }
                }
            }
        }
        Ok(())
    }
}

/// Whether the closed segments `a–b` and `c–d` share a point, decided in `f64` from the sign of each
/// end's side of the other segment's line.
fn segments_meet(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let side = |p: Vec2, q: Vec2, r: Vec2| -> f64 {
        (f64::from(q.x) - f64::from(p.x)) * (f64::from(r.y) - f64::from(p.y))
            - (f64::from(q.y) - f64::from(p.y)) * (f64::from(r.x) - f64::from(p.x))
    };
    let within = |p: Vec2, q: Vec2, r: Vec2| {
        r.x >= p.x.min(q.x) && r.x <= p.x.max(q.x) && r.y >= p.y.min(q.y) && r.y <= p.y.max(q.y)
    };
    let (d1, d2, d3, d4) = (side(c, d, a), side(c, d, b), side(a, b, c), side(a, b, d));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && within(c, d, a))
        || (d2 == 0.0 && within(c, d, b))
        || (d3 == 0.0 && within(a, b, c))
        || (d4 == 0.0 && within(a, b, d))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::new_velocity_in;

    /// The SplitMix64 output mix: the tests' own stable pseudo-random numbers.
    fn splitmix64(mut x: u64) -> u64 {
        x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = x;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A uniform value in `[lo, hi)` from a counter.
    fn value(counter: &mut u64, lo: f32, hi: f32) -> f32 {
        *counter += 1;
        lo + (splitmix64(*counter) >> 40) as f32 / (1u64 << 24) as f32 * (hi - lo)
    }

    fn v(x: f32, y: f32) -> Vec2 {
        Vec2::new(x, y)
    }

    /// What one agent of radius 0.3 and speed limit 1.5 is given by `obstacles`, and the velocity the
    /// solver then picks for `pref` with no neighbour, as the tests below compare with RVO2's.
    fn solve(
        obstacles: &Obstacles,
        pos: Vec2,
        vel: Vec2,
        pref: Vec2,
        horizon: f32,
    ) -> (Vec<(Vec2, f32)>, Vec2) {
        let agent = Agent { pos, vel, radius: 0.3, avoids: true };
        let mut scratch = OrcaScratch::default();
        let mut walls = Vec::new();
        obstacles.push_walls(&mut scratch, &agent, 1.5, horizon, &mut walls);
        let answer = new_velocity_in(&mut scratch, &agent, pref, &[], &walls, 1.0, 1.0 / 30.0, 1.5);
        (walls, answer)
    }

    fn built(
        f: impl FnOnce(&mut ObstaclesBuilder) -> Result<&mut ObstaclesBuilder, ObstacleError>,
    ) -> Obstacles {
        let mut builder = ObstaclesBuilder::new();
        f(&mut builder).expect("the geometry is valid");
        builder.build().expect("the geometry is valid")
    }

    /// The numbers `snape/RVO2` (built as shipped, `Agent.cc` at `main`) gives for one agent among
    /// one obstacle, with no other agent: the lines it casts (`point`, `direction` turned into this
    /// crate's `(normal, c)` form) and `newVelocity`. One row per branch of the obstacle code: a
    /// face, the top of a box seen from beside, a collision with an edge and with a vertex, a cutoff
    /// circle, legs, a wall with no thickness, and an L-shaped polygon's notch (two lines) and
    /// face. The agent has radius 0.3 and `maxSpeed` 1.5. Lines are compared as sets, to `1e-4`.
    #[test]
    fn the_lines_and_velocity_are_rvo2s_for_one_obstacle() {
        type Row = (&'static [(f32, f32)], [f32; 6], f32, &'static [((f32, f32), f32)], (f32, f32));
        let boxed: &[(f32, f32)] = &[(2.0, -1.0), (4.0, -1.0), (4.0, 1.0), (2.0, 1.0)];
        let ell: &[(f32, f32)] = &[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (3.0, 4.0), (3.0, 1.0), (0.0, 1.0)];
        let wall: &[(f32, f32)] = &[(3.0, -1.0), (3.0, 1.0)];
        // polygon, [pos.x, pos.y, vel.x, vel.y, pref.x, pref.y], horizon, expected lines, expected velocity
        let rows: &[Row] = &[
            (boxed, [0.0, 0.0, 1.0, 0.0, 1.5, 0.0], 2.0, &[((1.0, 0.0), 0.85)], (0.85, 0.0)),
            (boxed, [1.8, 1.5, 1.0, 0.0, 1.5, -0.5], 2.0, &[((0.0, -1.0), 0.1)], (1.496_663, -0.1)),
            (boxed, [1.8, 0.2, 1.0, 0.0, 1.5, 0.0], 2.0, &[((1.0, 0.0), 0.0)], (0.0, 0.0)),
            (boxed, [1.9, 1.2, 0.0, 0.0, 1.5, -1.5], 2.0, &[((0.447_214, -0.894_427), 0.0)], (0.6, 0.3)),
            (
                boxed,
                [3.0, -3.0, -0.5, 0.5, -0.5, 1.5],
                3.0,
                &[((core::f32::consts::FRAC_1_SQRT_2, core::f32::consts::FRAC_1_SQRT_2), 0.1357)],
                (-0.904_044, 1.095_956),
            ),
            (
                boxed,
                [-0.06, 1.01, -0.95, 1.41, -0.91, 1.40],
                2.0,
                &[((0.813_595, -0.581_433), 0.690_91)],
                (-0.817_482, 1.257_665),
            ),
            (
                boxed,
                [-2.58, -0.15, 0.61, -1.33, 1.43, -1.43],
                3.0,
                &[((0.658_844, 0.752_28), 0.692_69)],
                (1.060_66, -1.060_66),
            ),
            (
                boxed,
                [-0.10, 1.51, 0.14, 1.39, 0.15, -0.60],
                2.0,
                &[((0.484_061, -0.875_034), 0.5814)],
                (0.142_143, -0.585_797),
            ),
            (wall, [0.0, 0.0, 1.0, 0.0, 1.5, 0.3], 2.0, &[((1.0, 0.0), 1.35)], (1.35, 0.3)),
            (ell, [5.0, 0.5, -1.0, 0.0, -1.5, 0.0], 2.0, &[((-1.0, 0.0), 0.35)], (-0.35, 0.0)),
            (
                ell,
                [2.0, 2.0, -0.5, -0.5, -1.5, -1.5],
                2.0,
                &[((0.0, -1.0), 0.35), ((1.0, 0.0), 0.35)],
                (-1.458_595, -0.35),
            ),
        ];
        for (i, (polygon, [px, py, vx, vy, rx, ry], horizon, lines, velocity)) in rows.iter().enumerate() {
            let points: Vec<Vec2> = polygon.iter().map(|&(x, y)| v(x, y)).collect();
            let obstacles = built(|b| {
                if points.len() == 2 { b.add_wall(points[0], points[1]) } else { b.add_solid(&points) }
            });
            let (walls, answer) = solve(&obstacles, v(*px, *py), v(*vx, *vy), v(*rx, *ry), *horizon);
            assert_eq!(walls.len(), lines.len(), "row {i}: lines {walls:?}");
            for &((nx, ny), c) in lines.iter() {
                assert!(
                    walls.iter().any(|&(n, d)| (n - v(nx, ny)).length() < 1e-4 && (d - c).abs() < 1e-4),
                    "row {i}: RVO2's line {:?} is not among {walls:?}",
                    ((nx, ny), c)
                );
            }
            assert!(
                (answer - v(velocity.0, velocity.1)).length() < 1e-4,
                "row {i}: velocity {answer:?}, RVO2's {velocity:?}"
            );
        }
    }

    /// Either winding of a solid is the same obstacle, and an enclosure of the same polygon is its
    /// opposite: the outside agent sees a solid and not a room.
    #[test]
    fn winding_is_the_builders_business_and_solid_and_enclosure_are_opposites() {
        let square = [v(0.0, 0.0), v(4.0, 0.0), v(4.0, 4.0), v(0.0, 4.0)];
        let mut reversed = square;
        reversed.reverse();
        let (ccw, cw) = (built(|b| b.add_solid(&square)), built(|b| b.add_solid(&reversed)));
        let (room_ccw, room_cw) =
            (built(|b| b.add_enclosure(&square)), built(|b| b.add_enclosure(&reversed)));
        let outside = (v(-1.0, 2.0), v(1.0, 0.0), v(1.5, 0.0));
        let inside = (v(2.0, 2.0), v(1.0, 0.0), v(1.5, 0.0));
        for (name, o) in [("ccw", &ccw), ("cw", &cw)] {
            assert_eq!(
                solve(o, outside.0, outside.1, outside.2, 2.0).0.len(),
                1,
                "{name}: the outside agent sees the wall"
            );
            assert!(
                solve(o, inside.0, inside.1, inside.2, 2.0).0.is_empty(),
                "{name}: the inside agent is inside a solid, and sees nothing"
            );
        }
        for (name, o) in [("ccw", &room_ccw), ("cw", &room_cw)] {
            // Centred in a 4 m room an agent with 1.5 m of look-ahead is out of reach of every wall; 0.5 m
            // from the left one it sees it.
            assert!(
                solve(o, inside.0, inside.1, inside.2, 1.0).0.is_empty(),
                "{name}: the room's middle is out of reach"
            );
            let near_wall = solve(o, v(0.8, 2.0), v(-1.0, 0.0), v(-1.5, 0.0), 2.0).0;
            assert!(
                near_wall.iter().any(|&(n, _)| n == v(-1.0, 0.0)),
                "{name}: the inside agent sees the room's left wall: {near_wall:?}"
            );
        }
        // The same obstacle either way round: the same lines, bit for bit.
        assert_eq!(
            solve(&ccw, outside.0, outside.1, outside.2, 2.0),
            solve(&cw, outside.0, outside.1, outside.2, 2.0)
        );
        assert_eq!(
            solve(&room_ccw, inside.0, inside.1, inside.2, 2.0),
            solve(&room_cw, inside.0, inside.1, inside.2, 2.0)
        );
    }

    /// Everything the builder refuses is refused with the reason, and leaves the builder as it was.
    #[test]
    fn the_builder_refuses_what_the_solver_cannot_use() {
        let mut b = ObstaclesBuilder::new();
        let tri = [v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)];
        assert_eq!(b.add_solid(&tri[..2]).err(), Some(ObstacleError::TooFewVertices));
        assert_eq!(b.add_enclosure(&[]).err(), Some(ObstacleError::TooFewVertices));
        assert_eq!(b.add_solid(&[tri[0], tri[1], v(f32::NAN, 1.0)]).err(), Some(ObstacleError::NotFinite));
        assert_eq!(b.add_wall(v(0.0, 0.0), v(f32::INFINITY, 0.0)).err(), Some(ObstacleError::NotFinite));
        assert_eq!(b.add_solid(&[tri[0], tri[1], v(1.0e19, 1.0)]).err(), Some(ObstacleError::OutOfRange));
        assert_eq!(b.add_solid(&[tri[0], tri[0], tri[1], tri[2]]).err(), Some(ObstacleError::DegenerateEdge));
        assert_eq!(b.add_wall(tri[0], tri[0]).err(), Some(ObstacleError::DegenerateEdge));
        assert_eq!(
            b.add_solid(&[v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)]).err(),
            Some(ObstacleError::ZeroArea)
        );
        assert_eq!(b.add_box(v(0.0, 0.0), v(1.0, 0.0)).err(), Some(ObstacleError::ZeroArea));
        assert_eq!(b.add_box(v(0.0, 0.0), v(-1.0, 1.0)).err(), Some(ObstacleError::ZeroArea));
        assert_eq!(b.add_box(v(0.0, 0.0), v(f32::NAN, 1.0)).err(), Some(ObstacleError::NotFinite));
        assert_eq!(b.add_box(v(f32::NAN, 0.0), v(1.0, 1.0)).err(), Some(ObstacleError::NotFinite));
        // A bow-tie is a polygon until it is built: the crossing is found among every edge.
        let bow = [v(0.0, 0.0), v(3.0, 3.0), v(2.0, 0.0), v(0.0, 2.0)];
        assert!(b.add_solid(&bow).is_ok());
        assert_eq!(b.clone().build().err(), Some(ObstacleError::SelfIntersecting));
        // Nothing above was kept except the bow-tie.
        assert_eq!(b.vertices.len(), 4);
        // An edge touching another edge's end is a crossing too.
        let pinched = [v(0.0, 0.0), v(4.0, 0.0), v(4.0, 4.0), v(2.0, 0.0), v(0.0, 4.0)];
        let mut c = ObstaclesBuilder::new();
        assert!(c.add_solid(&pinched).is_ok());
        assert_eq!(c.build().err(), Some(ObstacleError::SelfIntersecting));
        // A polygon whose edges only meet at their own corners, however concave, is fine.
        let mut d = ObstaclesBuilder::new();
        d.add_solid(&[v(0.0, 0.0), v(4.0, 0.0), v(4.0, 4.0), v(3.0, 4.0), v(3.0, 1.0), v(0.0, 1.0)])
            .expect("valid");
        assert!(d.build().is_ok());
    }

    /// An agent far from every edge, or with nothing to see, is given nothing, and so is one whose
    /// state the solver refuses; the empty set costs nothing to ask.
    #[test]
    fn nothing_is_pushed_for_what_the_solver_will_not_solve() {
        let obstacles = built(|b| b.add_box(v(0.0, 0.0), v(1.0, 1.0)));
        let mut scratch = OrcaScratch::default();
        let mut walls = Vec::new();
        let near = Agent { pos: v(2.0, 0.0), vel: Vec2::ZERO, radius: 0.3, avoids: true };
        obstacles.push_walls(&mut scratch, &near, 1.5, 1.0, &mut walls);
        assert_eq!(walls.len(), 1, "an agent 1 m from the face, 1.5 m of look-ahead");
        walls.clear();
        let far = Agent { pos: v(50.0, 0.0), ..near };
        obstacles.push_walls(&mut scratch, &far, 1.5, 1.0, &mut walls);
        assert!(walls.is_empty());
        let bad = [
            (Agent { pos: v(f32::NAN, 0.0), ..near }, 1.5, 1.0),
            (Agent { vel: v(f32::INFINITY, 0.0), ..near }, 1.5, 1.0),
            (Agent { radius: -1.0, ..near }, 1.5, 1.0),
            (near, 0.0, 1.0),
            (near, f32::NAN, 1.0),
            (near, 1.5, 0.0),
            (near, 1.5, -1.0),
            (near, 1.5, f32::NAN),
            (near, 1.5, f32::INFINITY),
            (near, 1.5, 1.0e-39),
        ];
        for (agent, speed, horizon) in bad {
            obstacles.push_walls(&mut scratch, &agent, speed, horizon, &mut walls);
            assert!(walls.is_empty(), "{agent:?} {speed} {horizon}");
        }
        Obstacles::default().push_walls(&mut scratch, &near, 1.5, 1.0, &mut walls);
        assert!(walls.is_empty() && Obstacles::default().is_empty());
        // Lines are pushed after what the caller already has, which stays.
        walls.push((Vec2::Y, 3.0));
        obstacles.push_walls(&mut scratch, &near, 1.5, 1.0, &mut walls);
        assert_eq!(walls.len(), 2);
        assert_eq!(walls[0], (Vec2::Y, 3.0));
    }

    /// `push_walls` with the edges taken from the grid and with every edge tried give the same lines,
    /// bit for bit: no edge near an agent is missed by a cell, a long diagonal wall included, wherever
    /// the geometry is and however far the agent looks.
    #[test]
    fn the_grid_misses_no_edge() {
        let mut counter = 0u64;
        let mut produced = 0usize;
        for scene in 0..40u64 {
            let offset = if scene % 4 == 0 { v(5000.0, -3000.0) } else { Vec2::ZERO };
            let mut builder = ObstaclesBuilder::new();
            for _ in 0..(1 + scene % 12) {
                let centre = offset + v(value(&mut counter, -30.0, 30.0), value(&mut counter, -30.0, 30.0));
                match splitmix64(counter + scene) % 3 {
                    0 => {
                        let half = v(value(&mut counter, 0.2, 4.0), value(&mut counter, 0.2, 4.0));
                        builder.add_box(centre, half).expect("a box");
                    }
                    1 => {
                        // Long walls, some across the whole scene, some near diagonal and near axis-aligned.
                        let reach = v(value(&mut counter, -60.0, 60.0), value(&mut counter, -60.0, 60.0));
                        let reach = if scene % 3 == 0 { v(reach.x, reach.y * 0.001) } else { reach };
                        builder.add_wall(centre, centre + reach).expect("a wall");
                    }
                    _ => {
                        // A star with a clear gap in its angles, so it is a simple polygon.
                        let n = 5 + (splitmix64(counter) % 5) as usize;
                        let ring: Vec<Vec2> = (0..n)
                            .map(|k| {
                                let a = (k as f32 + 0.25 * value(&mut counter, -1.0, 1.0))
                                    * core::f32::consts::TAU
                                    / n as f32;
                                centre + v(a.cos(), a.sin()) * value(&mut counter, 0.5, 5.0)
                            })
                            .collect();
                        builder.add_solid(&ring).expect("a star");
                    }
                }
            }
            let obstacles = builder.build().expect("simple polygons");
            let mut scratch = OrcaScratch::default();
            for _ in 0..200 {
                let pos = offset + v(value(&mut counter, -50.0, 50.0), value(&mut counter, -50.0, 50.0));
                let vel = v(value(&mut counter, -2.0, 2.0), value(&mut counter, -2.0, 2.0));
                let agent = Agent { pos, vel, radius: value(&mut counter, 0.05, 0.6), avoids: true };
                let (speed, horizon) = (value(&mut counter, 0.5, 3.0), value(&mut counter, 0.2, 6.0));
                let mut from_grid = Vec::new();
                obstacles.push_walls(&mut scratch, &agent, speed, horizon, &mut from_grid);

                // The same, trying every edge.
                let range = horizon * speed + agent.radius;
                let mut all: Vec<(f32, u32)> = Vec::new();
                for (e, v1) in obstacles.vertices.iter().enumerate() {
                    let v2 = obstacles.vertices[v1.next as usize];
                    let edge = v2.point - v1.point;
                    if left_of(v1.point, v2.point, pos) >= 0.0 {
                        continue;
                    }
                    let along = ((pos - v1.point).dot(edge) / edge.length_squared()).clamp(0.0, 1.0);
                    let d = (pos - (v1.point + edge * along)).length_squared();
                    if d < range * range {
                        all.push((d, e as u32));
                    }
                }
                all.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                let mut by_hand = Vec::new();
                for &(_, e) in &all {
                    let v1 = obstacles.vertices[e as usize];
                    let v2 = obstacles.vertices[v1.next as usize];
                    obstacle_lines(
                        &obstacles.vertices,
                        e,
                        v1,
                        v2,
                        pos,
                        vel,
                        agent.radius,
                        1.0 / horizon,
                        0,
                        &mut by_hand,
                    );
                }
                assert_eq!(
                    from_grid, by_hand,
                    "scene {scene}: agent {agent:?}, speed {speed}, horizon {horizon}"
                );
                produced += from_grid.len();
            }
        }
        // The comparison is not vacuous: agents did see edges.
        assert!(produced > 1000, "only {produced} lines in 8 000 queries");
    }

    /// A wall that is a hair off vertical (its ends differ in `x` by a subnormal) has a slope too steep
    /// for an `f32`, `inf · 0` is a NaN, and the grid then linked it into one row only: a walker beside
    /// the rest of its length was handed no line at all. It is linked along its whole length.
    #[test]
    fn a_wall_a_hair_off_vertical_is_found_along_its_length() {
        let hair = 1.0e-39; // a subnormal
        let obstacles = built(|b| {
            b.add_wall(v(0.0, 0.0), v(hair, 100.0))?
                .add_wall(v(30.0, 0.0), v(30.0, 100.0))?
                .add_wall(v(60.0, 10.0), v(90.0, 10.0))
        });
        for y in [0.5, 3.0, 25.0, 50.0, 75.0, 97.0, 99.5] {
            let (walls, _) = solve(&obstacles, v(0.5, y), Vec2::ZERO, v(-1.5, 0.0), 2.0);
            assert!(!walls.is_empty(), "no line for a walker 0.5 m beside the wall at y = {y}");
        }
    }

    /// A random grid: `width × height` cells, each solid with probability `fill`.
    fn random_cells(counter: &mut u64, width: usize, height: usize, fill: f32) -> Vec<bool> {
        (0..width * height).map(|_| value(counter, 0.0, 1.0) < fill).collect()
    }

    /// The boundary of the solid region, the slow obvious way: for every solid cell and each of its
    /// four sides with open floor (or the edge of the grid) beyond it, the unit lattice edge along that
    /// side with the solid on its left. Sorted.
    fn boundary_edges(width: usize, solid: &[bool]) -> Vec<((i64, i64), (i64, i64))> {
        let height = (solid.len() / width.max(1)) as i64;
        let at = |c: i64, r: i64| {
            c >= 0 && r >= 0 && c < width as i64 && r < height && solid[(r * width as i64 + c) as usize]
        };
        let mut edges = Vec::new();
        for r in 0..height {
            for c in 0..width as i64 {
                if !at(c, r) {
                    continue;
                }
                if !at(c, r - 1) {
                    edges.push(((c, r), (c + 1, r)));
                }
                if !at(c + 1, r) {
                    edges.push(((c + 1, r), (c + 1, r + 1)));
                }
                if !at(c, r + 1) {
                    edges.push(((c + 1, r + 1), (c, r + 1)));
                }
                if !at(c - 1, r) {
                    edges.push(((c, r + 1), (c, r)));
                }
            }
        }
        edges.sort_unstable();
        edges
    }

    /// Every loop `trace_cells` emits, as `(corners, doubled signed area)`.
    fn traced(width: usize, solid: &[bool]) -> Vec<(Vec<(i64, i64)>, i128)> {
        let mut loops = Vec::new();
        trace_cells(width, solid, |corners, area| {
            loops.push((corners.to_vec(), area));
            Ok(())
        })
        .expect("tracing cannot fail");
        loops
    }

    /// The outline of any grid is exactly the boundary of its solid region: every boundary edge once,
    /// directed with the solid on its left, and no other; loops are closed, every vertex of one is a
    /// real corner and none repeats, and the loops' signed areas (outsides positive, pockets
    /// negative) add up to the solid's.
    #[test]
    fn the_outline_of_a_grid_is_exactly_its_boundary() {
        let mut counter = 0u64;
        let mut checkerboards = 0usize;
        for scene in 0..4000usize {
            let (width, height) = (1 + scene % 13, 1 + (scene / 13) % 11);
            let fill = [0.05, 0.2, 0.35, 0.5, 0.5, 0.65, 0.8, 0.95][scene % 8];
            let solid = random_cells(&mut counter, width, height, fill);
            let loops = traced(width, &solid);

            let mut edges = Vec::new();
            let mut area = 0i128;
            for (corners, doubled) in &loops {
                let n = corners.len();
                assert!(
                    n >= 4 && n % 2 == 0,
                    "a rectilinear loop has an even number of corners, at least four"
                );
                let mut seen = corners.clone();
                seen.sort_unstable();
                seen.dedup();
                assert_eq!(
                    seen.len(),
                    n,
                    "a loop revisited a vertex: {corners:?} in {solid:?} ({width} wide)"
                );
                for i in 0..n {
                    let (a, b, c) = (corners[i], corners[(i + 1) % n], corners[(i + 2) % n]);
                    let (d1, d2) = ((b.0 - a.0, b.1 - a.1), (c.0 - b.0, c.1 - b.1));
                    assert!(d1.0 == 0 || d1.1 == 0, "a diagonal edge");
                    assert!(d1.0 * d2.1 - d1.1 * d2.0 != 0, "a vertex that is not a turn");
                    let steps = (d1.0.abs() + d1.1.abs()).max(1);
                    let unit = (d1.0.signum(), d1.1.signum());
                    for k in 0..steps {
                        edges.push((
                            (a.0 + unit.0 * k, a.1 + unit.1 * k),
                            (a.0 + unit.0 * (k + 1), a.1 + unit.1 * (k + 1)),
                        ));
                    }
                }
                area += doubled;
            }
            edges.sort_unstable();
            assert_eq!(edges, boundary_edges(width, &solid), "{solid:?}, {width} wide");
            let cells = solid.iter().filter(|&&s| s).count() as i128;
            assert_eq!(area, 2 * cells, "the loops' areas are not the solid's: {solid:?}, {width} wide");
            checkerboards += (0..solid.len())
                .filter(|&i| {
                    let (c, r) = (i % width, i / width);
                    c + 1 < width
                        && r + 1 < height
                        && solid[i]
                        && solid[i + width + 1]
                        && !solid[i + 1]
                        && !solid[i + width]
                })
                .count();
        }
        assert!(checkerboards > 1000, "the sweep must contain corner contacts: {checkerboards}");
    }

    /// Cells that meet only at a corner are separate loops that touch there, in either diagonal, and
    /// four around a hole are four loops, not one loop that touches itself.
    #[test]
    fn solid_cells_that_meet_only_at_a_corner_are_separate_loops() {
        for solid in [[true, false, false, true], [false, true, true, false]] {
            let loops = traced(2, &solid);
            assert_eq!(loops.len(), 2, "{solid:?}");
            assert!(loops.iter().all(|(corners, area)| corners.len() == 4 && *area == 2));
        }
        let diamond = [false, true, false, true, false, true, false, true, false];
        let loops = traced(3, &diamond);
        assert_eq!(loops.len(), 4);
        assert!(loops.iter().all(|(corners, area)| corners.len() == 4 && *area == 2));
    }

    /// Every grid builds: the polygons that come out of the tracer are never refused as crossing
    /// themselves, whatever corners the cells meet at, and the set has one edge per corner.
    #[test]
    fn every_grid_builds() {
        let mut counter = 7u64;
        for scene in 0..3000usize {
            let (width, height) = (1 + scene % 17, 1 + (scene / 17) % 15);
            let fill = [0.1, 0.3, 0.5, 0.5, 0.7, 0.9][scene % 6];
            let solid = random_cells(&mut counter, width, height, fill);
            let origin = v(value(&mut counter, -50.0, 50.0), value(&mut counter, -50.0, 50.0));
            let size = value(&mut counter, 0.25, 3.0);
            let corners: usize = traced(width, &solid).iter().map(|(c, _)| c.len()).sum();
            let mut builder = ObstaclesBuilder::new();
            builder.add_cells(origin, size, width, &solid).expect("a grid");
            let obstacles =
                builder.build().unwrap_or_else(|e| panic!("{e}: {solid:?}, {width} wide, size {size}"));
            assert_eq!(obstacles.len(), corners);
        }
    }

    /// A straight wall is one edge however many cells make it, a block of cells is a box's four, and an
    /// L, a ring and a room each have the corners they have.
    #[test]
    fn an_outline_has_a_corner_per_turn_and_no_more() {
        let edges =
            |width: usize, solid: &[bool]| built(|b| b.add_cells(Vec2::ZERO, 1.0, width, solid)).len();
        let wall: Vec<bool> = (0..3000).map(|i| i / 1000 == 1).collect();
        assert_eq!(edges(1000, &wall), 4, "a wall 1000 cells long");
        let block: Vec<bool> =
            (0..9 * 6).map(|i| (2..7).contains(&(i % 9)) && (1..4).contains(&(i / 9))).collect();
        assert_eq!(edges(9, &block), 4, "a 5 x 3 block");
        let l = [true, true, true, true, false, false, true, false, false];
        assert_eq!(edges(3, &l), 6, "an L of cells");
        let ring = [true, true, true, true, false, true, true, true, true];
        assert_eq!(edges(3, &ring), 8, "a ring of cells: its outside and its pocket");
        assert_eq!(edges(3, &[false; 9]), 0);
        assert_eq!(edges(3, &[true; 9]), 4, "a block of solid cells filling the whole grid");
        assert_eq!(edges(1, &[true]), 4, "a single cell");
    }

    /// What is not a grid is refused with the reason, and the builder is left as it was.
    #[test]
    fn add_cells_refuses_what_is_not_a_grid() {
        let mut b = ObstaclesBuilder::new();
        b.add_box(v(0.0, 0.0), v(1.0, 1.0)).expect("a box");
        let solid = [true, false, true, true, true, false];
        let nan = f32::NAN;
        assert_eq!(b.add_cells(Vec2::ZERO, 1.0, 0, &solid).err(), Some(ObstacleError::BadGrid));
        assert_eq!(b.add_cells(Vec2::ZERO, 1.0, 4, &solid).err(), Some(ObstacleError::BadGrid));
        for size in [0.0, -1.0, -0.0] {
            assert_eq!(
                b.add_cells(Vec2::ZERO, size, 3, &solid).err(),
                Some(ObstacleError::BadGrid),
                "{size}"
            );
        }
        for (origin, size) in
            [(v(nan, 0.0), 1.0), (v(0.0, f32::INFINITY), 1.0), (Vec2::ZERO, nan), (Vec2::ZERO, f32::INFINITY)]
        {
            assert_eq!(b.add_cells(origin, size, 3, &solid).err(), Some(ObstacleError::NotFinite));
        }
        // A corner past 1e18 is refused half way through the first loop's polygons, after others
        // may have been added: the earlier ones go too.
        assert_eq!(b.add_cells(v(1.0e18, 0.0), 1.0e18, 3, &solid).err(), Some(ObstacleError::OutOfRange));
        // Cells so small beside their origin that two corners are the same `f32`.
        assert_eq!(b.add_cells(v(1.0e9, 0.0), 1.0e-3, 3, &solid).err(), Some(ObstacleError::DegenerateEdge));
        assert_eq!(b.clone().build().expect("the box").len(), 4, "a refused grid left something behind");
        // Nothing to add is fine: an empty grid, a grid with nothing solid.
        b.add_cells(Vec2::ZERO, 1.0, 0, &[]).expect("an empty grid");
        b.add_cells(Vec2::ZERO, 1.0, 3, &[false; 6]).expect("an open grid");
        assert_eq!(b.build().expect("the box").len(), 4);
    }
}
