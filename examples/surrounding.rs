// In this game, the player navigates to wherever you click

use std::time::Duration;
use bevy::{prelude::*, sprite::Anchor};
use bevy::sprite::MaterialMesh2dBundle;
use seldom_map_nav::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            MapNavPlugin::<Transform>::default()
        ))
        // This plugin is required for pathfinding and navigation
        // The type parameter is the position component that you use
        .init_resource::<CursorPos>()
        .add_systems(Startup, init)
        .add_systems(Update, (
            (update_cursor_pos, move_player).chain(),
            move_chasers
        ))
        .run();
}

const MAP_SIZE: UVec2 = UVec2::new(24, 24);
const TILE_SIZE: Vec2 = Vec2::new(32., 32.);
// This is the radius of a square around the player that should not intersect with the terrain
const PLAYER_CLEARANCE: f32 = 8.;

fn init(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle {
        // Centering the camera
        transform: Transform::from_translation((MAP_SIZE.as_vec2() * TILE_SIZE / 2.).extend(999.9)),
        ..default()
    });

    // Randomly generate the tilemap
    let tilemap = [(); (MAP_SIZE.x * MAP_SIZE.y) as usize].map(|_| Navability::Navable);
    let navability = |pos: UVec2| tilemap[(pos.y * MAP_SIZE.x + pos.x) as usize];

    // Spawn images for the tiles
    let tile_image = asset_server.load("tile.png");
    let mut player_pos = default();
    let colliders = [
        (10, 10),
        (10, 11),
        (10, 12),
        (10, 13),
        (10, 14),
        (11, 10),
        (11, 11),
        (11, 12),
        (11, 13),
        (11, 14),
    ];
    let uvec_pos = |x: u32, y: u32| UVec2::new(x, y).as_vec2() * TILE_SIZE;
    for x in 0..MAP_SIZE.x {
        for y in 0..MAP_SIZE.y {
            let pos = uvec_pos(x, y);
            if x == MAP_SIZE.x -1 && y == MAP_SIZE.y - 1 {
                player_pos = pos;
            }

            commands.spawn(SpriteBundle {
                sprite: Sprite {
                    anchor: Anchor::BottomLeft,
                    ..default()
                },
                transform: Transform::from_translation(pos.extend(0.)),
                texture: tile_image.clone(),
                ..default()
            });
        }
    }

    // Here's the important bit:

    // Spawn the tilemap with a `Navmeshes` component
    let navmeshes = commands
        .spawn(Navmeshes::generate(MAP_SIZE, TILE_SIZE, navability, [PLAYER_CLEARANCE]).unwrap()).id();

    // Spawn the player component. A position component is necessary. We will add `NavBundle`
    // later.
    let player = commands.spawn((
        SpriteBundle {
            transform: Transform::from_translation((uvec_pos(14, 14) + TILE_SIZE / 2.).extend(1.)),
            texture: asset_server.load("player.png"),
            ..default()
        },
        Player,
        Collider
    )).id();
    let circle_mesh = meshes.add(shape::Circle::new(50.).into());
    let material = materials.add(ColorMaterial::from(Color::PURPLE));

    //colliders components
    for collider in colliders {
        commands.spawn((
            MaterialMesh2dBundle {
                transform: Transform{
                    translation: (uvec_pos(collider.0, collider.1) + TILE_SIZE / 2.).extend(1.),
                    scale: Vec3::splat(0.25),
                    ..default()
                },
                mesh: circle_mesh.clone().into(),
                material: material.clone(),
                ..default()
            },
            Collider,
        ));
    }
}

// Navigate the player to wherever you click
fn move_player(
    mut commands: Commands,
    players: Query<Entity, With<Player>>,
    navmesheses: Query<Entity, With<Navmeshes>>,
    cursor_pos: Res<CursorPos>,
    mouse: Res<Input<MouseButton>>,
) {
    if mouse.just_pressed(MouseButton::Left) {
        if let Some(cursor_pos) = **cursor_pos {
            // Clicked somewhere on the screen!
            // Add `NavBundle` to start navigating to that position
            // If you want to write your own movement, but still want paths generated,
            // only insert `Pathfind`.
            commands.entity(players.single()).insert(NavBundle {
                pathfind: Pathfind::new(
                    navmesheses.single(),
                    PLAYER_CLEARANCE,
                    None,
                    PathTarget::Static(cursor_pos),
                    NavQuery::Accuracy,
                    NavPathMode::Accuracy,
                ),
                nav: Nav::new(200.),
            });
        }
    }
}

fn move_chasers (
    mut commands: Commands,
    chasers: Query<(Entity, &Transform, Option<&Pathfind>), (With<Collider>, Without<Player>)>,
    players: Query<(Entity, &Transform), With<Player>>,
    navmesheses: Query<Entity, With<Navmeshes>>,
) {
    for (chaser, transform, pathfind) in chasers.iter() {
        let (player, player_transform) = players.single();
        if transform.translation.truncate()
            .distance(player_transform.translation.truncate()) <= 60. {
            if pathfind.is_some() {
                commands.entity(chaser).remove::<Pathfind>();
            }
        } else {
            if pathfind.is_none() {
                commands.entity(chaser).insert(
                    NavBundle {
                        pathfind: Pathfind::new(
                            navmesheses.single(),
                            PLAYER_CLEARANCE,
                            Some(Duration::from_secs_f32(1.0)),
                            PathTarget::Dynamic(player),
                            NavQuery::Accuracy,
                            NavPathMode::Accuracy,
                        ),
                        nav: Nav::new(120.),
                    }
                );
            }
        }
    }
}

#[derive(Component)]
struct Player;

#[derive(Default, Deref, DerefMut, Resource)]
struct CursorPos(Option<Vec2>);

fn update_cursor_pos(
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut position: ResMut<CursorPos>,
) {
    let (camera, transform) = cameras.single();
    **position = windows
        .single()
        .cursor_position()
        .and_then(|cursor_pos| camera.viewport_to_world_2d(transform, cursor_pos));
}
