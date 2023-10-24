use std::{collections::VecDeque, error::Error, time::Duration};
use bevy_spatial::{AutomaticUpdate, SpatialAccess, kdtree::KDTree2, SpatialStructure};

use mint::Vector3;
use navmesh::{NavPathMode, NavQuery};

use crate::{prelude::*, set::MapNavSet};


/// Component required for applying steering forces
#[derive(Component, Default)]
pub struct Collider;

type KDTree = KDTree2<Collider>;

const MAX_SEE_AHEAD: f32 = 4.2;
const MAX_AVOID_FORCE: f32 = 0.88;
const COLLISION_RADIUS: f32 = 48.;

fn find_closest_obstacle(
    self_id: Entity,
    self_pos: Vec2,
    ahead: Vec2,
    ahead2: Vec2,
    tree: &Res<KDTree>
) -> Option<Vec2> {
    let neighbours = tree.k_nearest_neighbour(ahead, 2);

    if let Some((obstacle_pos, entity)) = neighbours.get(1) {
        if let Some(entity) = entity{
            if entity != &self_id && (
                obstacle_pos.distance(ahead) <= COLLISION_RADIUS
                // || obstacle_pos.distance(ahead2) <= COLLISION_RADIUS
            ) {
                return Some(*obstacle_pos)
            }
        }
    }
    None
}

fn collision_avoidance(
    self_id: Entity,
    desired_pos: Vec2,
    current_pos: Vec2,
    tree: &Res<KDTree>
) -> Vec2 {
    let desired_velocity = (desired_pos - current_pos).normalize();

    let ahead_mult = desired_velocity * MAX_SEE_AHEAD;
    let ahead = current_pos + ahead_mult;
    let ahead2 = current_pos + ahead_mult * 0.5;

    if let Some(obstacle) = find_closest_obstacle(self_id, current_pos, ahead, ahead2, tree) {
        (ahead - obstacle).normalize()
    } else { Vec2::ZERO }
}

pub(crate) fn nav_plugin<P: Position2<Position = Vec2>>(app: &mut App) {
    app
        .add_plugins(
            AutomaticUpdate::<Collider>::new()
            .with_frequency(Duration::from_secs_f32(0.1))
            .with_spatial_ds(SpatialStructure::KDTree2)
            // .with_transform(TransformMode::GlobalTransform)
        )
        .add_systems(
        Update,
        (apply_deferred, generate_paths::<P>, nav::<P>)
            .chain()
            .in_set(MapNavSet),
    );
}

/// A target to navigate to
#[derive(Clone, Copy, Debug, PartialEq, Reflect)]
pub enum PathTarget {
    /// A position
    Static(Vec2),
    /// An entity that has a position
    Dynamic(Entity),
}

/// Add this component to your entity to have it generate paths. Works as a state
/// in `seldom_state`.
#[derive(Clone, Component, Debug)]
pub struct Pathfind {
    /// Tilemap with the [`Navmeshes`] component
    pub map: Entity,
    /// Clearance radius
    pub radius: f32,
    /// How often to regenerate the path, if ever
    pub repath_frequency: Option<Duration>,
    /// Next time to repath
    pub next_repath: Duration,
    /// Target to navigate to
    pub target: PathTarget,
    /// Generated path
    pub path: VecDeque<Vec2>,
    /// Quality of querying a point on the navmesh
    pub query: NavQuery,
    /// Quality of finding a path
    pub path_mode: NavPathMode,
}

impl Pathfind {
    /// Create a `Pathfind`
    pub fn new(
        map: Entity,
        radius: f32,
        repath_frequency: Option<Duration>,
        target: PathTarget,
        query: NavQuery,
        path_mode: NavPathMode,
    ) -> Self {
        Self {
            map,
            radius,
            repath_frequency,
            next_repath: Duration::ZERO,
            target,
            path: default(),
            query,
            path_mode,
        }
    }
}

/// Add this component and [`Pathfind`] to your entity to have it navigate
#[derive(Clone, Component, Copy, Debug, Reflect)]
pub struct Nav {
    /// Speed by which to navigate
    pub speed: f32,
    /// Whether the entity has navigated to the destination
    pub done: bool,
}

impl Nav {
    /// Create a `Nav`
    pub fn new(speed: f32) -> Self {
        Self { speed, done: false }
    }
}

/// Components required for navigation
#[derive(Bundle, Clone, Debug)]
pub struct NavBundle {
    /// Pathfinding
    pub pathfind: Pathfind,
    /// Navigation
    pub nav: Nav,
}

pub(crate) fn generate_paths<P: Position2<Position = Vec2>>(
    #[cfg(feature = "state")] mut commands: Commands,
    positions: Query<&P>,
    mut pathfinds: Query<(Entity, &P, &mut Pathfind)>,
    mut navs: Query<&mut Nav>,
    meshes: Query<&Navmeshes>,
    time: Res<Time>,
) {
    #[allow(unused_variables)]
    for (entity, position, mut pathfind) in &mut pathfinds {
        let repath = pathfind
            .repath_frequency
            .map(|repath_frequency| {
                let repath = pathfind.next_repath <= time.elapsed();
                if repath {
                    pathfind.next_repath = time.elapsed() + repath_frequency;
                }
                repath
            })
            .unwrap_or_else(|| {
                let path = pathfind.next_repath == Duration::ZERO;
                if path {
                    pathfind.next_repath = Duration::MAX;
                }
                path
            });

        if !repath {
            continue;
        }

        let path = || -> Result<VecDeque<Vec2>, Box<dyn Error>> {
            Ok(meshes
                .get(pathfind.map)?
                .mesh(pathfind.radius)
                .ok_or_else(|| {
                    format!(
                        "missing navmesh with clearance of at least {}",
                        pathfind.radius
                    )
                })?
                .find_path(
                    Vector3::from(position.get().extend(0.)).into(),
                    Vector3::from(
                        match pathfind.target {
                            PathTarget::Static(target) => target,
                            PathTarget::Dynamic(target) => positions.get(target)?.get(),
                        }
                        .extend(0.),
                    )
                    .into(),
                    pathfind.query,
                    pathfind.path_mode,
                )
                .ok_or("no valid path was found")?
                .into_iter()
                .map(|pos| Vec3::from(Vector3::from(pos)).truncate())
                .collect())
        };

        let path = path();  // executing 1-time closure

        #[cfg(feature = "log")]
        if let Err(error) = &path {
            warn!("failed to generate path: {error}");
        }
        #[cfg(feature = "state")]
        let failure = path.is_err();
        pathfind.path = path.unwrap_or_default();

        let Ok(mut nav) = navs.get_mut(entity) else { continue };

        nav.done = pathfind.path.is_empty();

        #[cfg(feature = "state")]
        if failure {
            commands.entity(entity).insert(Done::Failure);
        }
    }
}

fn nav<P: Position2<Position = Vec2>>(
    #[cfg(feature = "state")] mut commands: Commands,
    mut navs: Query<(
        Entity,
        &mut P,
        &mut Pathfind,
        &mut Nav,
    )>,
    time: Res<Time>,
    tree: Res<KDTree>
) {
    #[allow(unused_variables)]
    for (entity, mut position, mut pathfind, mut nav) in &mut navs {
        if pathfind.path.is_empty() {
            #[cfg(feature = "state")]
            commands.entity(entity).insert(Done::Success);
            continue;
        }

        let old_pos = position.get();
        let mut pos = old_pos;

        let mut travel_dist = nav.speed * time.delta_seconds();
        let mut dest;
        let mut dest_dist;

        // consume as much path points as distance allows
        // consumes 0 points if target is too far
        while travel_dist >= {
            dest = *pathfind.path.front().unwrap();
            dest_dist = (dest - pos).length();
            dest_dist
        } {
            pos = dest;
            travel_dist -= dest_dist;

            pathfind.path.pop_front();
            if pathfind.path.is_empty() {
                break;
            }
        }

        if pathfind.path.is_empty() {
            nav.done = true;
            #[cfg(feature = "state")]
            commands.entity(entity).insert(Done::Success);
        } else {
            let delta = (dest - pos).normalize() * travel_dist;
            pos += delta;
        }

        pos += collision_avoidance(
            entity,
            pos,
            old_pos,
            &tree
        ) * travel_dist * MAX_AVOID_FORCE;

        // next frame position
        position.set(pos);
    }
}
