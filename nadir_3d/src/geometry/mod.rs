use std::fmt::Debug;

use glam::{DQuat, DVec3, Mat3, Mat4};
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod cuboid;
//pub mod cylinder;
pub mod ellipsoid;

use cuboid::{Cuboid, CuboidErrors};
use ellipsoid::{Ellipsoid16, Ellipsoid32, Ellipsoid64, EllipsoidErrors};

use crate::vertex::Vertex;

#[derive(Debug, Error)]
pub enum GeometryErrors {
    #[error("{0}")]
    CuboidErrors(#[from] CuboidErrors),
    #[error("{0}")]
    EllipsoidErrors(#[from] EllipsoidErrors),
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub enum Geometry {
    Ellipsoid16(Ellipsoid16),
    Ellipsoid32(Ellipsoid32),
    Ellipsoid64(Ellipsoid64),
    Cuboid(Cuboid),
}

impl Default for Geometry {
    fn default() -> Self {
        Geometry::Cuboid(Cuboid { x: 1.0, y: 1.0, z: 1.0 })
    }
}

impl From<Cuboid> for Geometry {
    fn from(value: Cuboid) -> Self {
        Geometry::Cuboid(value)
    }
}

impl From<Ellipsoid16> for Geometry {
    fn from(value: Ellipsoid16) -> Self {
        Geometry::Ellipsoid16(value)
    }
}

impl From<Ellipsoid32> for Geometry {
    fn from(value: Ellipsoid32) -> Self {
        Geometry::Ellipsoid32(value)
    }
}

impl From<Ellipsoid64> for Geometry {
    fn from(value: Ellipsoid64) -> Self {
        Geometry::Ellipsoid64(value)
    }
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
        Self { transformation_matrix, normal_matrix }
    }
}

pub trait GeometryTrait: Debug + Sync + Send {
    fn get_vertices(&self) -> Vec<Vertex>;
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform;
}

impl GeometryTrait for Geometry {
    fn get_transform(&self, state: &GeometryState) -> GeometryTransform {
        match self {
            Geometry::Cuboid(geometry) => geometry.get_transform(state),
            Geometry::Ellipsoid16(geometry) => geometry.get_transform(state),
            Geometry::Ellipsoid32(geometry) => geometry.get_transform(state),
            Geometry::Ellipsoid64(geometry) => geometry.get_transform(state),
        }
    }

    fn get_vertices(&self) -> Vec<Vertex> {
        match self {
            Geometry::Cuboid(geometry) => geometry.get_vertices(),
            Geometry::Ellipsoid16(geometry) => geometry.get_vertices(),
            Geometry::Ellipsoid32(geometry) => geometry.get_vertices(),
            Geometry::Ellipsoid64(geometry) => geometry.get_vertices(),
        }
    }
}
