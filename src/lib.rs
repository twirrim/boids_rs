pub mod boids;

#[derive(Debug, Clone, Copy)]
pub struct Parameters {
    pub max_speed: f32,
    pub min_speed: f32,
    pub margin: u32,
    pub visible_range: f32,
    pub protected_range: f32,
    pub avoid_factor: f32,
    pub matching_factor: f32,
    pub centering_factor: f32,
    pub turn_factor: f32,
    pub cell_size: f32,
    pub draw_radius: i32,
}
