use std::fmt::Debug;

use glam::{Mat3, Mat4, DQuat, DVec3};
use serde::{Deserialize, Serialize};

pub mod cuboid;
//pub mod cylinder;
pub mod ellipsoid;

use cuboid::Cuboid;
use ellipsoid::{Ellipsoid16,Ellipsoid32,Ellipsoid64};

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub enum Geometry {
    Ellipsoid16(Ellipsoid16),
    Ellipsoid32(Ellipsoid32),
    Ellipsoid64(Ellipsoid64),
    Cuboid(Cuboid),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GeometryState {
    pub position: DVec3,
    pub rotation: DQuat,
}

#[derive(Debug)]
pub struct GeometryTransform {
    pub transformation_matrix: Mat4,
    pub normal_matrix: Mat3,
}

impl GeometryTransform {
    pub fn new(transformation_matrix: Mat4, normal_matrix: Mat3) -> Self {
        Self {
            transformation_matrix,
            normal_matrix,
        }
    }
}

pub trait GeometryTrait: Debug + Sync + Send {
    fn get_mesh_transform(&self, state: &GeometryState) -> GeometryTransform;
}

impl GeometryTrait for Geometry {
    fn get_mesh_transform(&self, state: &GeometryState) -> GeometryTransform {
        match self {
            Geometry::Cuboid(geometry) => geometry.get_mesh_transform(state),
            Geometry::Ellipsoid16(geometry) => geometry.get_mesh_transform(state),
            Geometry::Ellipsoid32(geometry) => geometry.get_mesh_transform(state),
            Geometry::Ellipsoid64(geometry) => geometry.get_mesh_transform(state),
        }
    }    
}
