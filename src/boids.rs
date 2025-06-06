use std::collections::HashMap;

use image::Rgb;
use nalgebra::Vector2;
use rand::prelude::*;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::Parameters;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Boid {
    id: usize,
    pub pos: Vector2<f32>,
    vel: Vector2<f32>,
    current_speed: f32,
    #[serde(with = "rgb_serde")]
    pub colour: Rgb<u8>,
}

impl Boid {
    pub fn new(
        id: usize,
        pos: Vector2<f32>,
        vel: Vector2<f32>,
        current_speed: f32,
        colour: Rgb<u8>,
    ) -> Self {
        Boid {
            id,
            pos,
            vel,
            current_speed,
            colour,
        }
    }
}

pub fn populate_grid(boids: &[Boid], cell_size: f32) -> HashMap<(u32, u32), Vec<usize>> {
    let mut grid: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (index, boid) in boids.iter().enumerate() {
        let cell_x: u32 = (boid.pos.x / cell_size).floor() as u32;
        let cell_y: u32 = (boid.pos.y / cell_size).floor() as u32;
        grid.entry((cell_x, cell_y)).or_default().push(index);
    }
    grid
}

pub fn update_boids(boids: &mut Vec<Boid>, height: u32, width: u32, parameters: Parameters) {
    let protected_range_squared = parameters.protected_range * parameters.protected_range;
    let visible_range_squared = parameters.visible_range * parameters.visible_range;
    let grid = populate_grid(boids, parameters.cell_size);
    // For rust, we'll need to gather all the changes, then apply
    let new_boid_states: Vec<(Vector2<f32>, Vector2<f32>, f32)> = boids
        .par_iter()
        .enumerate()
        .map(|(boid_idx, boid)| {
            let mut rng = rand::rng();
            let mut pos_avg = Vector2::zeros();
            let mut vel_avg = Vector2::zeros();
            let mut close_offset = Vector2::zeros();

            let mut neighboring_boids: usize = 0;

            let boid_cell_x: i32 = (boid.pos.x / parameters.cell_size).floor() as i32;
            let boid_cell_y: i32 = (boid.pos.y / parameters.cell_size).floor() as i32;
            for x_offset in -1..=1 {
                for y_offset in -1..=1 {
                    let new_x = boid_cell_x + x_offset;
                    let new_y = boid_cell_y + y_offset;
                    if new_x >= 0 && new_y >= 0 {
                        let key = (new_x as u32, new_y as u32);
                        if let Some(near_boids) = grid.get(&key) {
                            for otherboid_idx in near_boids {
                                if *otherboid_idx == boid_idx {
                                    continue;
                                }
                                let otherboid = &boids[*otherboid_idx];

                                let offset = boid.pos - otherboid.pos;
                                // Only consider those within our visible box
                                if offset.x.abs() < parameters.visible_range
                                    && offset.y.abs() < parameters.visible_range
                                {
                                    let dist_sq = offset.norm_squared();
                                    if dist_sq < protected_range_squared {
                                        close_offset += offset;
                                    } else if dist_sq < visible_range_squared {
                                        pos_avg += otherboid.pos;
                                        vel_avg += otherboid.vel;
                                        neighboring_boids += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            let mut next_vel = boid.vel;
            if neighboring_boids > 0 {
                let n = neighboring_boids as f32;
                pos_avg /= n;
                vel_avg /= n;
                next_vel += (pos_avg - boid.pos) * parameters.centering_factor
                    + (vel_avg - boid.vel) * parameters.matching_factor;
            }
            next_vel += close_offset * parameters.avoid_factor;

            // Turn if approaching the edge of the screen
            if boid.pos.y > (height - parameters.margin) as f32 {
                next_vel.y -= parameters.turn_factor;
            }
            if boid.pos.x > (width - parameters.margin) as f32 {
                next_vel.x -= parameters.turn_factor;
            }
            if boid.pos.x < parameters.margin as f32 {
                next_vel.x += parameters.turn_factor;
            }
            if boid.pos.y < parameters.margin as f32 {
                next_vel.y += parameters.turn_factor;
            }

            // Make sure we're within speed limits
            let mut speed = next_vel.norm();
            if speed > 0.0 {
                if speed < parameters.min_speed {
                    next_vel = next_vel.normalize() * parameters.min_speed;
                    speed = parameters.min_speed;
                } else if speed > parameters.max_speed {
                    next_vel = next_vel.normalize() * parameters.max_speed;
                    speed = parameters.max_speed;
                }
            } else if parameters.min_speed > 0.0 {
                // Give it a nudge if stopped
                next_vel = Vector2::new(
                    rng.random_range(-parameters.min_speed..parameters.min_speed),
                    rng.random_range(-parameters.min_speed..parameters.min_speed),
                );
                speed = parameters.min_speed;
            }

            let mut next_pos = boid.pos + next_vel;

            // Finally, clamp them so they're in the screen
            if next_pos.x < 0.0 {
                next_pos.x = 0.0;
            } else if next_pos.x >= width as f32 {
                next_pos.x = (width - 1) as f32;
            }
            if next_pos.y < 0.0 {
                next_pos.y = 0.0;
            } else if next_pos.y >= height as f32 {
                next_pos.y = (height - 1) as f32;
            }

            (next_pos, next_vel, speed)
        })
        .collect();

    // apply the changes
    for (i, boid) in boids.iter_mut().enumerate() {
        let (new_pos, new_vel, new_speed) = new_boid_states[i];
        boid.pos = new_pos;
        boid.vel = new_vel;
        boid.current_speed = new_speed;
    }
}

// Module to handle Rgb<u8> serialization/deserialization
mod rgb_serde {
    use image::Rgb;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    // Serialize Rgb<u8> as [u8; 3]
    pub fn serialize<S>(rgb: &Rgb<u8>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Rgb<T> internally stores its data as a [T; N] array in field 0.
        // For Rgb<u8>, it's [u8; 3].
        rgb.0.serialize(serializer)
    }

    // Deserialize Rgb<u8> from [u8; 3]
    pub fn deserialize<'de, D>(deserializer: D) -> Result<Rgb<u8>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let arr: [u8; 3] = Deserialize::deserialize(deserializer)?;
        Ok(Rgb(arr))
    }
}
