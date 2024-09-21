use color::Color;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Material {
    Basic {
        color: Color,
    },
    Phong {
        color: Color, 
        specular_power: f32,
    },
}