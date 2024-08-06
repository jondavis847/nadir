use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Cuboid {
    pub length: f32,
    pub width: f32,
    pub height: f32,
}
