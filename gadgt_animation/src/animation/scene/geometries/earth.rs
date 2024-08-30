use super::ellipsoid::Ellipsoid;
use glam::{Quat, Vec3};

/// A single instance of an ellipsoid.
#[derive(Debug, Clone)]
pub struct Earth(pub Ellipsoid);

impl Default for Earth {
    fn default() -> Self {
        let ellipsoid = Ellipsoid {
            name: "earth".to_string(),
            rotation: Quat::from_rotation_x(23.439281 * std::f32::consts::PI / 180.0),
            position: Vec3::ZERO,
            radius_x: 6378137.0,
            radius_y: 6378137.0,
            radius_z: 6356752.3,
            latitude_bands: 32,
            longitude_bands: 32,
            color: [1.0, 1.0, 1.0, 1.0],
        };
        Self(ellipsoid)
    }
}