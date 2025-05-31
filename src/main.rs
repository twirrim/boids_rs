use std::collections::HashMap;

use image::{Rgb, RgbImage};
use indicatif::{ProgressBar, ProgressStyle};
use rand::prelude::*;
use rayon::prelude::*;

const MAX_SPEED: f32 = 3.0;
const MIN_SPEED: f32 = 0.5;
const MARGIN: u32 = 10;
const WIDTH: u32 = 1920;
const HEIGHT: u32 = 1080;
const BOIDS: usize = 100000;
const VISIBLE_RANGE: f32 = 20.0;
const VISIBLE_RANGE_SQUARED: f32 = VISIBLE_RANGE * VISIBLE_RANGE;
const PROTECTED_RANGE: f32 = 2.0;
const PROTECTED_RANGE_SQUARED: f32 = PROTECTED_RANGE * PROTECTED_RANGE;
const AVOID_FACTOR: f32 = 0.05;
const MATCHING_FACTOR: f32 = 0.05;
const CENTERING_FACTOR: f32 = 0.0005;
const TURN_FACTOR: f32 = 0.2;
const CELL_SIZE: f32 = VISIBLE_RANGE * 1.1;
const FRAMES: usize = 5000;

const WHITE: Rgb<u8> = Rgb([255, 255, 255]);

#[derive(Debug, Clone, PartialEq)]
struct Boid {
    id: usize,
    x: f32,
    y: f32,
    xv: f32,
    yv: f32,
    current_speed: f32,
}

fn populate_grid(boids: &Vec<Boid>) -> HashMap<(u32, u32), Vec<usize>> {
    let mut grid: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (index, boid) in boids.iter().enumerate() {
        let cell_x: u32 = (boid.x / CELL_SIZE).floor() as u32;
        let cell_y: u32 = (boid.y / CELL_SIZE).floor() as u32;
        grid.entry((cell_x, cell_y)).or_insert(vec![]).push(index);
    }
    grid
}

fn update_boids(boids: &mut Vec<Boid>, grid: HashMap<(u32, u32), Vec<usize>>) {
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

            let boid_cell_x: i32 = (boid.x / CELL_SIZE).floor() as i32;
            let boid_cell_y: i32 = (boid.y / CELL_SIZE).floor() as i32;
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

                                if dx.abs() < VISIBLE_RANGE && dy.abs() < VISIBLE_RANGE {
                                    let squared_distance = dx * dx + dy * dy;
                                    if squared_distance < PROTECTED_RANGE_SQUARED {
                                        close_dx += dx;
                                        close_dy += dy;
                                    } else if squared_distance < VISIBLE_RANGE_SQUARED {
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
                xpos_avg = xpos_avg / neighboring_boids as f32;
                ypos_avg = ypos_avg / neighboring_boids as f32;
                xvel_avg = xvel_avg / neighboring_boids as f32;
                yvel_avg = yvel_avg / neighboring_boids as f32;

                next_xv +=
                    (xpos_avg - boid.x) * CENTERING_FACTOR + (xvel_avg - boid.xv) * MATCHING_FACTOR;

                next_yv +=
                    (ypos_avg - boid.y) * CENTERING_FACTOR + (yvel_avg - boid.yv) * MATCHING_FACTOR;
            }
            next_xv += close_dx * AVOID_FACTOR;
            next_yv += close_dy * AVOID_FACTOR;

            // Turn if approaching the edge of the screen
            if boid.y > (HEIGHT - MARGIN) as f32 {
                next_yv -= TURN_FACTOR;
            }
            if boid.x > (WIDTH - MARGIN) as f32 {
                next_xv -= TURN_FACTOR;
            }
            if boid.x < MARGIN as f32 {
                next_xv += TURN_FACTOR;
            }
            if boid.y < MARGIN as f32 {
                next_yv += TURN_FACTOR;
            }

            // Make sure we're within speed limits
            let mut speed = (boid.xv * boid.xv + boid.yv * boid.yv).sqrt();
            if speed > 0.0 {
                // Avoid division by 0
                if speed < MIN_SPEED {
                    let factor = MIN_SPEED / speed;
                    next_xv *= factor;
                    next_yv *= factor;
                    speed = MIN_SPEED;
                } else if speed > MAX_SPEED {
                    let factor = MAX_SPEED / speed;
                    next_xv *= factor;
                    next_yv *= factor;
                    speed = MAX_SPEED;
                };
            } else if MIN_SPEED > 0.0 {
                // If speed is 0 give it a nudge
                next_xv = rng.random_range(-MIN_SPEED..MIN_SPEED);
                next_yv = rng.random_range(-MIN_SPEED..MIN_SPEED);
                speed = MIN_SPEED;
            };

            let mut next_x = boid.x + next_xv;
            let mut next_y = boid.y + next_yv;
            // Finally, clamp them so they're in the screen
            if next_x as u32 >= WIDTH {
                next_x = (WIDTH - 1) as f32;
            }
            if next_y as u32 >= HEIGHT {
                next_y = (HEIGHT - 1) as f32;
            }
            return (next_x, next_y, next_xv, next_yv, speed);
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

fn main() {
    let mut rng = rand::rng();
    let mut boids: Vec<Boid> = (0..BOIDS)
        .into_iter()
        .map(|id| Boid {
            id,
            x: rng.random_range(0..WIDTH) as f32,
            y: rng.random_range(0..HEIGHT) as f32,
            xv: rng.random_range(-MAX_SPEED / 2.0..MAX_SPEED / 2.0), // Initial velocity
            yv: rng.random_range(-MAX_SPEED / 2.0..MAX_SPEED / 2.0),
            current_speed: 0.0,
        })
        .collect();
    let mut running = true;
    let mut frame = 0;
    let pbar = ProgressBar::new(FRAMES as u64);
    pbar.set_style(
        ProgressStyle::with_template(
            "[{elapsed_precise}/{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}",
        )
        .unwrap(),
    );
    while running {
        let mut img = RgbImage::new(WIDTH, HEIGHT);
        let grid = populate_grid(&boids);
        update_boids(&mut boids, grid);
        for boid in &boids {
            img.put_pixel(boid.x as u32, boid.y as u32, WHITE);
        }
        img.save(format!("./frames/frames_{:0>8}.png", frame))
            .unwrap();

        frame += 1;
        pbar.inc(1);
        if frame > FRAMES {
            running = false;
        }
    }
}
