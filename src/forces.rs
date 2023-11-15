use bevy::reflect::Array;
use bevy_spatial::{SpatialAccess, kdtree::KDTree2};

use crate::{prelude::*};

const AVOID_FORCE_RATE: f32 = 0.8;
const MAX_QUEUE_AHEAD: f32 = 54.2;
const AVOID_RADIUS: f32 = 36.;

const MAX_QUEUE_RADIUS: f32 = 52.;

const SEPARATION_FORCE_RATE: f32 = 1.25;
const SEPARATION_RADIUS: f32 = 38.;

const MAX_FORCE: f32 = 3.6;
const SEEK_FORCE_RATE: f32 = 1.0;

/// Component required for applying steering forces
#[derive(Component, Default)]
pub struct Collider;

/// Shortname for search tree type
pub type KDTree = KDTree2<Collider>;

/// applies forces to result velocity
pub fn apply_forces(
    self_id: Entity,
    target_pos: Vec2,
    current_pos: Vec2,
    mut current_velocity: Vec2,
    tree: &Res<KDTree>
) -> Vec2 {
    let desired_velocity = (target_pos - current_pos).normalize_or_zero();

    // seek force
    let mut steering = (desired_velocity - current_velocity).normalize_or_zero() * SEEK_FORCE_RATE;

    // steering += collision_avoidance_force(
    //     self_id,;
    //     desired_velocity,
    //     current_pos,
    //     &tree
    // );

    steering += separation_force(
        self_id,
        current_pos,
        &tree
    );

    if steering != Vec2::ZERO {
        steering = steering.normalize_or_zero() * MAX_FORCE
    }

    if let Some(obstacle) = get_neighbour_ahead(
        self_id,
        current_pos,
        current_velocity,
        tree
    ) {
        let mut brake = -current_velocity + steering * -0.8;
        brake += separation_force(
            self_id,
            current_pos,
            &tree
        );

        if current_pos.distance(obstacle) <= MAX_QUEUE_RADIUS {
            current_velocity *= 0.3;
        }

        steering += brake;
    }

    (current_velocity + steering).normalize_or_zero()
}

fn get_neighbour_ahead(
    self_id: Entity,
    current_pos: Vec2,
    current_velocity: Vec2,
    tree: &Res<KDTree>
) -> Option<Vec2> {
    let ahead = current_pos + current_velocity * MAX_QUEUE_AHEAD;
    // let ahead2 = ahead * 0.5;

    for p in [
        // current_pos,
        // ahead2,
        ahead
    ] {
        let neighbours = tree.k_nearest_neighbour(p, 2);
        for (obstacle_pos, entity) in neighbours.iter() {
            if let Some(entity) = entity {
                if entity != &self_id && (
                    obstacle_pos.distance(p) <= AVOID_RADIUS
                ) {
                    return Some(*obstacle_pos)
                }
            }
        }
    }

    None
}

/// when obstacle detected on a course of movement
// fn collision_avoidance_force(
//     self_id: Entity,
//     velocity: Vec2,
//     current_pos: Vec2,
//     tree: &Res<KDTree>
// ) -> Vec2 {
//     let ahead = current_pos + velocity * MAX_SEE_AHEAD;
//
//     if let Some(obstacle) = find_closest_obstacle(self_id, ahead, tree) {
//         let delta = ahead - obstacle;
//         delta.normalize() * AVOID_FORCE_RATE
//     } else { Vec2::ZERO }
// }

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
                let delta = neighbour_pos - current_pos;
                let rate = delta.length() / SEPARATION_RADIUS;
                force += delta / rate;
                neighbour_count += 1.;
            }
        }
    }
    if neighbour_count > 0. {
        force /= -neighbour_count;
        force
        // force.normalize() * SEPARATION_FORCE_RATE
    } else { force }
}