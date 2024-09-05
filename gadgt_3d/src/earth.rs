use super::mesh::Mesh;
use crate::{
    geometry::{
        ellipsoid::Ellipsoid64,
        Geometry, GeometryState,
    },
    material::Material,
};
use color::Color;
use glam::{Quat, Vec3};

#[derive(Debug)]
pub struct Earth(pub Mesh);

impl Default for Earth {
    fn default() -> Self {
        let mesh = Mesh {
            name: "earth".to_string(),
            geometry: Geometry::Ellipsoid64(Ellipsoid64::new(6378137.0, 6378137.0, 6356752.3)),
            material: Material::Phong {
                color: Color::WHITE,
                specular_power: 80.0,
            },
            state: GeometryState {
                position: Vec3::ZERO,
                rotation: Quat::from_rotation_x(23.439281 * std::f32::consts::PI / 180.0),
            },
            texture: None,
        };
        Self(mesh)
    }
}
