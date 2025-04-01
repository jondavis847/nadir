pub mod geometry;
pub mod material;
pub mod mesh;
pub mod texture;
pub mod vertex;

use geometry::GeometryErrors;
use material::MaterialErrors;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum Nadir3dErrors {
    #[error("{0}")]
    Geometry(#[from] GeometryErrors),
    #[error("{0}")]
    Material(#[from] MaterialErrors),
}
