use super::mesh::Mesh;
use crate::{
    geometry::{
        ellipsoid::{Ellipsoid32, Ellipsoid64},
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

#[derive(Debug)]
pub struct Atmosphere(pub Mesh);

impl From<&Earth> for Atmosphere {
    fn from(earth: &Earth) -> Self {
        let atmosphere_thickness = 1.05e5; // 100 km?
        let earth_ellipsoid = match earth.0.geometry {
            Geometry::Ellipsoid16(ellipsoid) => ellipsoid.0,
            Geometry::Ellipsoid32(ellipsoid) => ellipsoid.0,
            Geometry::Ellipsoid64(ellipsoid) => ellipsoid.0,
            _ => panic!("earth should be an ellipsoid"),
        };

        let mesh = Mesh {
            name: "atmosphere".to_string(),
            geometry: Geometry::Ellipsoid32(Ellipsoid32::new(
                earth_ellipsoid.radius_x + atmosphere_thickness,
                earth_ellipsoid.radius_y + atmosphere_thickness,
                earth_ellipsoid.radius_z + atmosphere_thickness,
            )),
            material: Material::Phong {
                color: Color::new(100.0 / 255.0, 130.0 / 255.0, 250.0 / 255.0, 0.05),
                specular_power: 0.0,
            },
            state: GeometryState {
                position: earth.0.state.position,
                rotation: earth.0.state.rotation,
            },
            texture: None,
        };
        Self(mesh)
    }
}
