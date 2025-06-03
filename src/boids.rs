use std::collections::HashMap;

use image::Rgb;
use rand::prelude::*;
use rayon::prelude::*;

#[derive(Debug, Clone, PartialEq)]
pub struct Boid {
    id: usize,
    pub x: f32,
    pub y: f32,
    xv: f32,
    yv: f32,
    current_speed: f32,
    pub colour: Rgb<u8>,
}

impl Boid {
    pub fn new(
        id: usize,
        x: f32,
        y: f32,
        xv: f32,
        yv: f32,
        current_speed: f32,
        colour: Rgb<u8>,
    ) -> Self {
        Boid {
            id,
            x,
            y,
            xv,
            yv,
            current_speed,
            colour,
        }
    }
}

pub fn populate_grid(boids: &Vec<Boid>, cell_size: f32) -> HashMap<(u32, u32), Vec<usize>> {
    let mut grid: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (index, boid) in boids.iter().enumerate() {
        let cell_x: u32 = (boid.x / cell_size).floor() as u32;
        let cell_y: u32 = (boid.y / cell_size).floor() as u32;
        grid.entry((cell_x, cell_y)).or_default().push(index);
    }
    grid
}

pub fn update_boids(
    boids: &mut Vec<Boid>,
    height: u32,
    width: u32,
    cell_size: f32,
    visible_range: f32,
    protected_range: f32,
    centering_factor: f32,
    matching_factor: f32,
    avoid_factor: f32,
    turn_factor: f32,
    margin: u32,
    min_speed: f32,
    max_speed: f32,
) {
    let protected_range_squared = protected_range * protected_range;
    let visible_range_squared = visible_range * visible_range;
    let grid = populate_grid(boids, cell_size);
    // For rust, we'll need to gather all the changes, then apply
    let new_boid_states: Vec<(f32, f32, f32, f32, f32)> = boids
        .par_iter()
        .enumerate()
        .map(|(boid_idx, boid)| {
            let mut rng = rand::rng();

            let mut xpos_avg: f32 = 0.0;
            let mut ypos_avg: f32 = 0.0;
            let mut xvel_avg: f32 = 0.0;
            let mut yvel_avg: f32 = 0.0;
            let mut neighboring_boids: usize = 0;
            let mut close_dx: f32 = 0.0;
            let mut close_dy: f32 = 0.0;

            let boid_cell_x: i32 = (boid.x / cell_size).floor() as i32;
            let boid_cell_y: i32 = (boid.y / cell_size).floor() as i32;
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

                                let dx = boid.x - otherboid.x;
                                let dy = boid.y - otherboid.y;

                                if dx.abs() < visible_range && dy.abs() < visible_range {
                                    let squared_distance = dx * dx + dy * dy;
                                    if squared_distance < protected_range_squared {
                                        close_dx += dx;
                                        close_dy += dy;
                                    } else if squared_distance < visible_range_squared {
                                        xpos_avg += otherboid.x;
                                        ypos_avg += otherboid.y;
                                        xvel_avg += otherboid.xv;
                                        yvel_avg += otherboid.yv;
                                        neighboring_boids += 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            let mut next_xv = boid.xv;
            let mut next_yv = boid.yv;
            if neighboring_boids > 0 {
                xpos_avg /= neighboring_boids as f32;
                ypos_avg /= neighboring_boids as f32;
                xvel_avg /= neighboring_boids as f32;
                yvel_avg /= neighboring_boids as f32;

                next_xv +=
                    (xpos_avg - boid.x) * centering_factor + (xvel_avg - boid.xv) * matching_factor;

                next_yv +=
                    (ypos_avg - boid.y) * centering_factor + (yvel_avg - boid.yv) * matching_factor;
            }
            next_xv += close_dx * avoid_factor;
            next_yv += close_dy * avoid_factor;

            // Turn if approaching the edge of the screen
            if boid.y > (height - margin) as f32 {
                next_yv -= turn_factor;
            }
            if boid.x > (width - margin) as f32 {
                next_xv -= turn_factor;
            }
            if boid.x < margin as f32 {
                next_xv += turn_factor;
            }
            if boid.y < margin as f32 {
                next_yv += turn_factor;
            }

            // Make sure we're within speed limits
            let mut speed = (boid.xv * boid.xv + boid.yv * boid.yv).sqrt();
            if speed > 0.0 {
                // Avoid division by 0
                if speed < min_speed {
                    let factor = min_speed / speed;
                    next_xv *= factor;
                    next_yv *= factor;
                    speed = min_speed;
                } else if speed > max_speed {
                    let factor = max_speed / speed;
                    next_xv *= factor;
                    next_yv *= factor;
                    speed = max_speed;
                };
            } else if min_speed > 0.0 {
                // If speed is 0 give it a nudge
                next_xv = rng.random_range(-min_speed..min_speed);
                next_yv = rng.random_range(-min_speed..min_speed);
                speed = min_speed;
            };

            let mut next_x = boid.x + next_xv;
            let mut next_y = boid.y + next_yv;
            // Finally, clamp them so they're in the screen
            if next_x as u32 >= width {
                next_x = (width - 1) as f32;
            }
            if next_y as u32 >= height {
                next_y = (height - 1) as f32;
            }
            (next_x, next_y, next_xv, next_yv, speed)
        })
        .collect();

    // apply the changes
    for (i, boid) in boids.iter_mut().enumerate() {
        let (new_x, new_y, new_xv, new_yv, new_speed) = new_boid_states[i];
        boid.x = new_x;
        boid.y = new_y;
        boid.xv = new_xv;
        boid.yv = new_yv;
        boid.current_speed = new_speed;
    }
}
