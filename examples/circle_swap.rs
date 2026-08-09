//! **The circle swap — the standard way to see whether an avoidance solver actually works.**
//!
//! Every agent starts on a circle and is told to walk to the point directly opposite. That means all
//! of them want to pass through the exact centre at the same moment, which is the worst case: the one
//! configuration where "just steer around the nearest obstacle" deadlocks or piles up.
//!
//! Nothing here plans a path. Each agent knows only where it wants to go and where its neighbours are
//! right now, and each solves a small 2-D linear program for the velocity closest to what it wanted
//! that no neighbour can object to. The rotation you see is not scripted and not a rule anyone wrote —
//! it is what falls out when everyone makes that choice simultaneously.
//!
//! **Reciprocity is the load-bearing word.** Both agents in a pair take half the avoidance, so neither
//! needs to know what the other will do and they cannot oscillate by both dodging the same way. Set
//! `avoids: false` on one and the other takes the full burden instead — which is how you model
//! something that is not going to step aside for you.
//!
//! Watch the middle. The pack does not collide and does not stop; it shears into a rotation, unwinds
//! on the far side, and re-forms. Then the goals flip and it does it again.
//!
//! This is the only example here that needs a GPU; `head_on` and `wall_corridor` print to a terminal.
//!
//! Run: `cargo run -p bevy_orca --example circle_swap`

use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy_orca::{Agent, new_velocity};

/// How many agents on the ring. Enough that the centre is genuinely contested.
const COUNT: usize = 20;
/// Ring radius, world units.
const RING: f32 = 6.0;
/// Agent radius, world units.
const AGENT_R: f32 = 0.26;
/// Screen pixels per world unit.
const SCALE: f32 = 52.0;

const MAX_SPEED: f32 = 2.4;
/// Seconds of lookahead, and the single most consequential dial here.
///
/// Larger means earlier, more cautious avoidance — and at 3.0 this demo visibly **stalled**: velocity
/// decayed geometrically (0.80, 0.57, 0.41, 0.29, 0.21 …) while the pack ground to a halt about a
/// ring-radius in. Everyone braked for a congestion that had not happened yet, which then guaranteed
/// it. 1.5 commits them, and the shear that resolves the crossing is what you are meant to see.
const TIME_HORIZON: f32 = 1.5;
/// Fixed step, so the demo looks the same on any monitor.
const DT: f32 = 1.0 / 60.0;
/// Within this of its goal, an agent is done.
const ARRIVE: f32 = 0.45;

#[derive(Component)]
struct Walker {
    pos: Vec2,
    vel: Vec2,
    goal: Vec2,
}

fn world_of(p: Vec2) -> Vec3 {
    Vec3::new(p.x * SCALE, p.y * SCALE, 1.0)
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "bevy_orca — 28 agents, one contested centre".into(),
                // 0.19 takes physical pixels as `u32` — there is no `(f32, f32)` conversion.
                resolution: (860u32, 860u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(Update, step)
        .run();
}

/// A soft-edged disc, built once and tinted per agent. Cheaper than a mesh per agent and it keeps the
/// example to one rendering concept.
fn disc_image(px: u32) -> Image {
    let mut data = vec![0u8; (px * px * 4) as usize];
    let c = px as f32 * 0.5;
    for y in 0..px {
        for x in 0..px {
            let d = Vec2::new(x as f32 + 0.5 - c, y as f32 + 0.5 - c).length();
            // One pixel of feather at the rim so the discs do not look aliased when scaled down.
            let a = ((c - d).clamp(0.0, 1.0) * 255.0) as u8;
            let i = ((y * px + x) * 4) as usize;
            data[i] = 255;
            data[i + 1] = 255;
            data[i + 2] = 255;
            data[i + 3] = a;
        }
    }
    Image::new(
        Extent3d { width: px, height: px, depth_or_array_layers: 1 },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        bevy::asset::RenderAssetUsages::default(),
    )
}

fn setup(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    commands.spawn(Camera2d);
    spawn_legend(&mut commands);

    let disc = images.add(disc_image(64));
    for i in 0..COUNT {
        let a = i as f32 / COUNT as f32 * std::f32::consts::TAU;
        let pos = Vec2::new(a.cos(), a.sin()) * RING;
        // Colour by starting angle, so you can read where each agent came from once they mix.
        let hue = i as f32 / COUNT as f32 * 360.0;
        commands.spawn((
            Walker { pos, vel: Vec2::ZERO, goal: -pos },
            Sprite {
                image: disc.clone(),
                color: Color::hsl(hue, 0.62, 0.62),
                custom_size: Some(Vec2::splat(AGENT_R * 2.0 * SCALE)),
                ..default()
            },
            Transform::from_translation(world_of(pos)),
        ));
    }
}

/// One ORCA step for every agent, then integrate.
///
/// Two passes on purpose: every agent must solve against the SAME snapshot of the world. Solve and
/// move in one pass and the agents late in the iteration see their neighbours already moved, which
/// quietly breaks the reciprocity the whole method rests on.
fn step(mut walkers: Query<(&mut Walker, &mut Transform)>) {
    // Goals travel in the snapshot too, so the solve loop never has to index back into the query.
    let snapshot: Vec<(Vec2, Vec2, Vec2)> =
        walkers.iter().map(|(w, _)| (w.pos, w.vel, w.goal)).collect();

    let mut solved: Vec<Vec2> = Vec::with_capacity(snapshot.len());
    for (i, (pos, vel, goal)) in snapshot.iter().enumerate() {
        let me = Agent { pos: *pos, vel: *vel, radius: AGENT_R, avoids: true };
        let neighbors: Vec<Agent> = snapshot
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .map(|(_, (p, v, _))| Agent { pos: *p, vel: *v, radius: AGENT_R, avoids: true })
            .collect();

        // Where it would go with no one in the way. Ease off near the goal so arrivals settle instead
        // of jittering on the spot.
        let to_goal = *goal - *pos;
        let dist = to_goal.length();
        // **Break the symmetry, or they deadlock — and that is a property of the method, not a bug
        // in this demo.** With every agent identical, equally spaced, and aimed at the exact same
        // point, the linear program has no reason to prefer left over right; the constraints stay
        // perfectly balanced and the ring simply stalls. RVO2's own circle demo perturbs the preferred
        // velocity for precisely this reason. Deterministic here rather than random, so the picture is
        // the same on every run.
        let nudge = Vec2::new((i as f32 * 12.9898).sin(), (i as f32 * 78.233).sin()) * 0.16;
        let pref = if dist < 1.0e-4 {
            Vec2::ZERO
        } else {
            to_goal / dist * MAX_SPEED.min(dist * 2.0) + nudge
        };

        solved.push(new_velocity(&me, pref, &neighbors, &[], TIME_HORIZON, DT, MAX_SPEED));
    }

    let mut all_arrived = true;
    for ((mut w, mut tf), v) in walkers.iter_mut().zip(solved) {
        w.vel = v;
        let p = w.pos + v * DT;
        w.pos = p;
        tf.translation = world_of(p);
        if w.pos.distance(w.goal) > ARRIVE {
            all_arrived = false;
        }
    }

    // Send everyone back across, so the demo loops instead of ending in a settled ring.
    if all_arrived {
        for (mut w, _) in &mut walkers {
            w.goal = -w.goal;
        }
    }
}

/// A legend, because "these dots avoid each other" is not the interesting claim — *how* is.
///
/// ASCII only, deliberately: Bevy's embedded default font carries 95 codepoints, so an arrow or a
/// middle dot renders as nothing at all and the label silently loses a word.
fn spawn_legend(commands: &mut Commands) {
    let lines = [
        "bevy_orca - reciprocal collision avoidance (ORCA)",
        "every agent is walking to the point opposite its own",
        "no paths, no scripting: each solves a 2-D linear program",
        "for the velocity closest to what it wanted that no",
        "neighbour can object to",
        "the rotation is emergent - nobody told them to turn",
    ];
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(12.0),
                top: Val::Px(12.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(3.0),
                padding: UiRect::all(Val::Px(10.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
        ))
        .with_children(|p| {
            for (i, line) in lines.into_iter().enumerate() {
                p.spawn((
                    Text::new(line),
                    // 0.19 makes `font_size` a `FontSize` enum rather than a bare f32.
                    TextFont { font_size: FontSize::Px(if i == 0 { 14.0 } else { 12.0 }), ..default() },
                    TextColor(if i == 0 {
                        Color::srgb(1.0, 0.93, 0.78)
                    } else {
                        Color::srgb(0.82, 0.84, 0.90)
                    }),
                ));
            }
        });
}
