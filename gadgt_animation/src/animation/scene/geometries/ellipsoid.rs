use super::super::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4, Quat, Vec3, Vec4};
use iced::widget::shader::wgpu;

/// A single instance of an ellipsoid.
#[derive(Debug, Clone)]
pub struct Ellipsoid {
    pub name: String,
    pub rotation: Quat,
    pub position: Vec3,
    pub radius_x: f32, // Radius along the x-axis
    pub radius_y: f32, // Radius along the y-axis
    pub radius_z: f32, // Radius along the z-axis
    pub latitude_bands: u32,
    pub longitude_bands: u32,
    pub color: [f32; 4],
}

impl Default for Ellipsoid {
    fn default() -> Self {
        Self {
            name: String::new(),
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
            radius_x: 1.0,
            radius_y: 1.0,
            radius_z: 1.0,
            latitude_bands: 16,
            longitude_bands: 16,
            color: [1.0, 1.0, 1.0, 1.0],
        }
    }
}

impl Ellipsoid {
    pub fn new(
        name: String,
        radius_x: f32,
        radius_y: f32,
        radius_z: f32,
        rotation: Quat,
        position: Vec3,
        latitude_bands: u32,
        longitude_bands: u32,
        color: [f32; 4],
    ) -> Self {
        Self {
            name,
            rotation,
            position,
            radius_x,
            radius_y,
            radius_z,
            latitude_bands,
            longitude_bands,
            color,
        }
    }

    pub fn update(&mut self, rotation: Quat, position: Vec3) {
        self.rotation = rotation;
        self.position = position;
    }
}

#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct EllipsoidRaw {
    transformation: Mat4, // 16 * 4 = 64 bytes
    color: Vec4,          // 4 * 4 = 16 bytes
    normal: Mat3,         // 9 * 4 = 36 bytes
    _padding1: [f32; 3],  // 12 bytes of padding to align the struct size
    latitude_bands: u32,  // 4
    longitude_bands: u32, // 4
    _padding2: [u32; 2],  // 8 bytes of padding to align the struct size
}

impl EllipsoidRaw {
    const ATTRIBS: [wgpu::VertexAttribute; 8] = wgpu::vertex_attr_array![
        4 => Float32x4, // transformation row 1
        5 => Float32x4, // transformation row 2
        6 => Float32x4, // transformation row 3
        7 => Float32x4, // transformation row 4
        8 => Float32x4, // color (RGBA)
        9 => Float32x3, // normal row 1 (padded Mat3)
        10 => Float32x3, // normal row 2 (padded Mat3)
        11 => Float32x3, // normal row 3 (padded Mat3)
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }

    pub fn from_ellipsoid(ellipsoid: &Ellipsoid) -> Self {
        let transformation = Mat4::from_scale_rotation_translation(
            vec3(ellipsoid.radius_x, ellipsoid.radius_y, ellipsoid.radius_z),
            ellipsoid.rotation,
            ellipsoid.position,
        );

        let normal = transformation.inverse().transpose();
        let normal = Mat3::from_mat4(normal);

        Self {
            transformation,
            normal,
            _padding1: [0.0; 3],
            color: Vec4::from_array(ellipsoid.color),
            latitude_bands: ellipsoid.latitude_bands,
            longitude_bands: ellipsoid.longitude_bands,
            _padding2: [0; 2],
        }
    }

    pub fn vertices(&self) -> Vec<Vertex> {
        let mut grid_points = Vec::with_capacity(self.latitude_bands as usize + 1);
        let mut vertices = Vec::new();

        // using wikipedia spherical coordinate frame.
        // z up.
        // theta is angle from z, 0 to pi. (latitude)
        // phi is angle from phi, 0 to 2pi (longitude)

        // Generate grid points
        for lat in 0..=self.latitude_bands {
            let mut row = Vec::with_capacity(self.longitude_bands as usize + 1);

            let theta = lat as f32 * std::f32::consts::PI / self.latitude_bands as f32;
            let sin_theta = theta.sin();
            let cos_theta = theta.cos();

            for lon in 0..=self.longitude_bands {
                let phi = lon as f32 * 2.0 * std::f32::consts::PI / self.longitude_bands as f32;
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();

                let x = cos_phi * sin_theta;
                let y = sin_phi * sin_theta;
                let z = cos_theta;

                let normal = vec3(x, y, z).normalize();
                // Tangent vector at the point on the sphere
                let tangent =
                    vec3(-phi.sin() * theta.sin(), phi.sin() * theta.cos(), 0.0).normalize();
                let uv = vec2(
                    lon as f32 / self.longitude_bands as f32,
                    lat as f32 / self.latitude_bands as f32,
                );

                row.push(Vertex {
                    pos: vec3(x, y, z),
                    normal,
                    tangent,
                    uv,
                });
            }

            grid_points.push(row);
        }
        let north_pole = grid_points[0][0];
        let south_pole = grid_points[self.latitude_bands as usize][0];
        // Create triangles from grid points
        for lat in 0..self.latitude_bands as usize {
            for lon in 0..self.longitude_bands as usize {
                let next_lon = lon + 1;
                let next_lat = lat + 1;

                // Vertices for the current quad
                let top_left = grid_points[lat][lon];
                let bottom_left = grid_points[next_lat][lon];
                let bottom_right = grid_points[next_lat][next_lon];
                let top_right = grid_points[lat][next_lon];

                // Handle poles
                if lat == 0 {
                    // North pole triangle
                    vertices.push(north_pole);
                    vertices.push(bottom_left);
                    vertices.push(bottom_right);
                } else if lat == self.latitude_bands as usize - 1 {
                    // South pole triangle
                    vertices.push(top_left);
                    vertices.push(south_pole);
                    vertices.push(top_right);
                } else {
                    // Two triangles for non-pole regions

                    //Triangle 1
                    vertices.push(top_left);
                    vertices.push(bottom_left);
                    vertices.push(bottom_right);

                    //Triangle 2
                    vertices.push(top_left);
                    vertices.push(bottom_right);
                    vertices.push(top_right);
                }
            }
        }

        vertices
    }
}
