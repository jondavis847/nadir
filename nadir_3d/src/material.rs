use color::Color;
use serde::{Deserialize, Serialize};
use thiserror::Error;
#[derive(Error, Debug)]
pub enum MaterialErrors {}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Material {
    Basic { color: Color },
    Phong { color: Color, specular_power: f32 },
}

impl Default for Material {
    fn default() -> Self {
        Material::Phong {
            color: Color::GREEN,
            specular_power: 32.0,
        }
    }
}
