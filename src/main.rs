use std::path::Path;

use argh::FromArgs;
use colors_transform::{Color, Hsl};
use image::{Rgb, RgbImage};
use indicatif::{ProgressBar, ProgressStyle};
use rand::prelude::*;

use boids::boids::{Boid, update_boids};

#[derive(Debug, FromArgs)]
#[argh(help_triggers("-h", "--help", "help"), description = "Boids simulator")]
struct Flags {
    #[argh(
        option,
        description = "width of image, defaults 1920",
        default = "1920"
    )]
    width: u32,
    #[argh(
        option,
        description = "height of image, defaults 1080",
        default = "1080"
    )]
    height: u32,
    #[argh(
        option,
        description = "directory for images",
        from_str_fn(valid_directory)
    )]
    dir: String,
    #[argh(option, description = "frames to simulate", default = "1000")]
    frames: usize,
    #[argh(option, description = "boids to simulate", default = "10000")]
    boids: usize,
}

fn valid_directory(dir: &str) -> Result<String, String> {
    if Path::new(dir).is_dir() {
        return Ok(String::from(dir));
    }
    Err(String::from("Target directory not valid"))
}

const MAX_SPEED: f32 = 3.0;
const MIN_SPEED: f32 = 0.5;
const MARGIN: u32 = 10;
const VISIBLE_RANGE: f32 = 20.0;
const PROTECTED_RANGE: f32 = 2.0;
const AVOID_FACTOR: f32 = 0.10;
const MATCHING_FACTOR: f32 = 0.05;
const CENTERING_FACTOR: f32 = 0.0005;
const TURN_FACTOR: f32 = 0.2;
const CELL_SIZE: f32 = VISIBLE_RANGE * 1.1;
const DRAW_RADIUS: i32 = 2; // Radius of the circle that represents a boid
const DRAW_RADIUS_SQUARED: i32 = DRAW_RADIUS * DRAW_RADIUS; // Pre-calculate

fn get_colour_by_width(x: f32, width: u32) -> Rgb<u8> {
    let width = width as f32;
    let h_per = 360.0 / width;
    let hsl = Hsl::from(h_per * x, 100.0, 50.0);
    let rgb = hsl.to_rgb();
    Rgb([
        rgb.get_red() as u8,
        rgb.get_green() as u8,
        rgb.get_blue() as u8,
    ])
}

fn main() {
    let args: Flags = argh::from_env();
    let mut rng = rand::rng();
    let mut boids: Vec<Boid> = (0..args.boids)
        .into_iter()
        .map(|id| {
            let x = rng.random_range(0..args.width) as f32;
            Boid::new(
                id,
                x,
                rng.random_range(0..args.height) as f32,
                rng.random_range(-MAX_SPEED / 2.0..MAX_SPEED / 2.0),
                rng.random_range(-MAX_SPEED / 2.0..MAX_SPEED / 2.0),
                0.0,
                get_colour_by_width(x, args.width),
            )
        })
        .collect();
    let mut running = true;
    let mut frame = 0;
    let pbar = ProgressBar::new(args.frames as u64);
    pbar.set_style(
        ProgressStyle::with_template(
            "[{elapsed_precise}/{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}",
        )
        .unwrap(),
    );
    while running {
        let mut img = RgbImage::new(args.width, args.height);
        update_boids(
            &mut boids,
            args.height,
            args.width,
            CELL_SIZE,
            VISIBLE_RANGE,
            PROTECTED_RANGE,
            CENTERING_FACTOR,
            MATCHING_FACTOR,
            AVOID_FACTOR,
            TURN_FACTOR,
            MARGIN,
            MIN_SPEED,
            MAX_SPEED,
        );
        for boid in &boids {
            // Rather than a single pixel, going to create a circle
            let boid_x_int = boid.x.round() as i32;
            let boid_y_int = boid.y.round() as i32;
            for dy_offset in -DRAW_RADIUS..=DRAW_RADIUS {
                for dx_offset in -DRAW_RADIUS..=DRAW_RADIUS {
                    if (dx_offset * dx_offset + dy_offset * dy_offset) <= DRAW_RADIUS_SQUARED {
                        let px = boid_x_int + dx_offset;
                        let py = boid_y_int + dy_offset;
                        if px >= 0 && px < args.width as i32 && py >= 0 && py < args.height as i32 {
                            img.put_pixel(px as u32, py as u32, boid.colour);
                        }
                    }
                }
            }
            img.put_pixel(boid.x as u32, boid.y as u32, boid.colour);
        }
        img.save(format!("{}/frames_{:0>8}.png", args.dir, frame))
            .unwrap();

        frame += 1;
        pbar.inc(1);
        if frame > args.frames {
            running = false;
        }
    }
}
