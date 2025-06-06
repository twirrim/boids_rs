use std::fs;
use std::path::Path;

use argh::FromArgs;
use colors_transform::{Color, Hsl};
use image::{Rgb, RgbImage};
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector2;
use rand::prelude::*;

use boids::boids::{update_boids, Boid};
use boids::Parameters;

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
    #[argh(option, description = "file to save starting boids to")]
    save_file: Option<String>,
    #[argh(
        option,
        description = "file to load starting boids from",
        from_str_fn(valid_file)
    )]
    load_file: Option<String>,
}

fn valid_file(file: &str) -> Result<String, String> {
    if Path::new(file).is_file() {
        return Ok(String::from(file));
    }
    Err(String::from("Source file for boids valid"))
}

fn valid_directory(dir: &str) -> Result<String, String> {
    if Path::new(dir).is_dir() {
        return Ok(String::from(dir));
    }
    Err(String::from("Target directory not valid"))
}

fn get_colour_by_width(x: f32, width: u32) -> Rgb<u8> {
    let width = width as f32;
    let h_per = 360.0 / width;
    let hsl = Hsl::from(h_per * x, 100.0, 50.0);
    let rgb = hsl.to_rgb();
    Rgb([
        rgb.get_red().round() as u8,
        rgb.get_green().round() as u8,
        rgb.get_blue().round() as u8,
    ])
}

fn main() {
    let args: Flags = argh::from_env();

    let parameters: Parameters = Parameters {
        max_speed: 3.0,
        min_speed: 0.5,
        margin: 10,
        visible_range: 20.0,
        protected_range: 2.0,
        avoid_factor: 0.10,
        matching_factor: 0.05,
        centering_factor: 0.0005,
        turn_factor: 0.2,
        cell_size: 22.0,
        draw_radius: 2,
    };
    let mut rng = rand::rng();
    let mut boids: Vec<Boid>;
    if let Some(source) = args.load_file {
        println!("Loading starting state from {source}");
        let data = fs::read_to_string(source).expect("Unable to read source file");
        boids = serde_json::from_str(&data).unwrap();
    } else {
        boids = (0..args.boids)
            .map(|id| {
                let x = rng.random_range(0..args.width) as f32;
                Boid::new(
                    id,
                    Vector2::new(x, rng.random_range(0..args.height) as f32),
                    Vector2::new(
                        rng.random_range(-parameters.max_speed / 2.0..parameters.max_speed / 2.0),
                        rng.random_range(-parameters.max_speed / 2.0..parameters.max_speed / 2.0),
                    ),
                    0.0,
                    get_colour_by_width(x, args.width),
                )
            })
            .collect();
    }
    if let Some(target) = args.save_file {
        println!("Saving starting state to {target}");
        let data = serde_json::to_string(&boids).unwrap();
        fs::write(target, data).expect("Unable to write file");
    }
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
        update_boids(&mut boids, args.height, args.width, parameters);
        for boid in &boids {
            // Rather than a single pixel, going to create a circle
            let boid_x_int = boid.pos.x.round() as i32;
            let boid_y_int = boid.pos.y.round() as i32;
            for dy_offset in -parameters.draw_radius..=parameters.draw_radius {
                for dx_offset in -parameters.draw_radius..=parameters.draw_radius {
                    if (dx_offset * dx_offset + dy_offset * dy_offset)
                        <= (parameters.draw_radius * parameters.draw_radius)
                    {
                        let px = boid_x_int + dx_offset;
                        let py = boid_y_int + dy_offset;
                        if px >= 0 && px < args.width as i32 && py >= 0 && py < args.height as i32 {
                            img.put_pixel(px as u32, py as u32, boid.colour);
                        }
                    }
                }
            }
            img.put_pixel(boid.pos.x as u32, boid.pos.y as u32, boid.colour);
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
