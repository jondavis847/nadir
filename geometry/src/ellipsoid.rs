use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Ellipsoid {
    pub radius_x: f32, // Radius along the x-axis
    pub radius_y: f32, // Radius along the y-axis
    pub radius_z: f32, // Radius along the z-axis
    pub latitude_bands: u32,
    pub longitude_bands: u32,
    pub color: [f32; 4],
}

impl Ellipsoid {
    pub fn new(
        radius_x: f32,
        radius_y: f32,
        radius_z: f32,
        latitude_bands: u32,
        longitude_bands: u32,
        color: [f32; 4],
    ) -> Self {
        Self {
            radius_x,
            radius_y,
            radius_z,
            latitude_bands,
            longitude_bands,
            color,
        }
    }
}
