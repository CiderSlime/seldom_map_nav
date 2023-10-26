use bevy_spatial::{SpatialAccess, kdtree::KDTree2};

use crate::{prelude::*};

const AVOID_FORCE_RATE: f32 = 1.01;
const MAX_SEE_AHEAD: f32 = 14.2;
const COLLISION_RADIUS: f32 = 56.;

const SEPARATION_FORCE_RATE: f32 = 1.25;
const SEPARATION_RADIUS: f32 = 38.;

const MAX_FORCE: f32 = 1.4;

/// Component required for applying steering forces
#[derive(Component, Default)]
pub struct Collider;

/// Shortname for search tree type
pub type KDTree = KDTree2<Collider>;

/// applies forces to result velocity
pub fn apply_forces(
    self_id: Entity,
    next_pos: Vec2,
    old_pos: Vec2,
    tree: &Res<KDTree>
) -> Vec2 {
    let base_velocity = (next_pos - old_pos).normalize();

    let mut steering = Vec2::ZERO;

    steering += collision_avoidance_force(
        self_id,
        base_velocity,
        old_pos,
        &tree
    );

    steering += separation_force(
        self_id,
        old_pos,
        &tree
    );

    if steering != Vec2::ZERO {
        steering = steering.normalize() * MAX_FORCE
    }

    (base_velocity + steering).normalize()
    // base_velocity
}

fn find_closest_obstacle(
    self_id: Entity,
    ahead: Vec2,
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

/// when obstacle detected on a course of movement
fn collision_avoidance_force(
    self_id: Entity,
    velocity: Vec2,
    current_pos: Vec2,
    tree: &Res<KDTree>
) -> Vec2 {
    let ahead = current_pos + velocity * MAX_SEE_AHEAD;

    if let Some(obstacle) = find_closest_obstacle(self_id, ahead, tree) {
        (ahead - obstacle).normalize() * AVOID_FORCE_RATE
    } else { Vec2::ZERO }
}

/// when another actors is too close
fn separation_force(
    self_id: Entity,
    current_pos: Vec2,
    tree: &Res<KDTree>
) -> Vec2 {
    let mut force = Vec2::ZERO;
    let mut neighbour_count = 0_f32;
    for (neighbour_pos, entity) in tree.within_distance(current_pos, SEPARATION_RADIUS) {
        if let Some(entity) = entity {
            if entity != self_id {
                force += neighbour_pos - current_pos;
                neighbour_count += 1.;
            }
        }
    }
    if neighbour_count > 0. {
        force /= -neighbour_count;
        force.normalize() * SEPARATION_FORCE_RATE
    } else { force }
}