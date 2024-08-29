use serde::{Serialize, Deserialize};
pub mod cuboid;
use cuboid::Cuboid;
pub mod ellipsoid;
use ellipsoid::Ellipsoid;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Geometry {
    Cuboid(Cuboid),
    Ellipsoid(Ellipsoid),
}