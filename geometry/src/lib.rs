use serde::{Serialize, Deserialize};
pub mod cuboid;
use cuboid::Cuboid;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Geometry {
    Cuboid(Cuboid),
}