pub mod cuboid;
use cuboid::Cuboid;

#[derive(Debug, Clone, Copy)]
pub enum Geometry {
    Cuboid(Cuboid),
}