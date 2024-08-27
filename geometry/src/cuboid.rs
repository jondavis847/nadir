use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Cuboid {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub color: [f32;4],
}

impl Cuboid {
    pub fn new(x: f32, y: f32, z: f32, color: [f32;4]) -> Self {
        Self { x, y, z , color}
    }
}
