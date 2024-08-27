use super::super::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4, Quat, Vec3};
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
    transformation: Mat4,  // 16 * 4 = 64 bytes
    normal: [[f32; 4]; 3], // 12 * 4 = 48 bytes
    color: [f32; 4],       // 16 bytes
    latitude_bands: u32,
    longitude_bands: u32,
    _padding: [f32; 2], // 12 bytes to ensure the struct size is a multiple of 16 bytes
}

impl EllipsoidRaw {
    const ATTRIBS: [wgpu::VertexAttribute; 8] = wgpu::vertex_attr_array![
        4 => Float32x4, // transformation row 1
        5 => Float32x4, // transformation row 2
        6 => Float32x4, // transformation row 3
        7 => Float32x4, // transformation row 4
        8 => Float32x4, // normal row 1 (padded Mat3)
        9 => Float32x4, // normal row 2 (padded Mat3)
        10 => Float32x4, // normal row 3 (padded Mat3)
        11 => Float32x4, // color (RGBA)
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }

    pub fn from_ellipsoid(ellipsoid: &Ellipsoid) -> Self {
        let mat3 = Mat3::from_quat(ellipsoid.rotation);
        let normal = [
            [mat3.x_axis.x, mat3.x_axis.y, mat3.x_axis.z, 0.0],
            [mat3.y_axis.x, mat3.y_axis.y, mat3.y_axis.z, 0.0],
            [mat3.z_axis.x, mat3.z_axis.y, mat3.z_axis.z, 0.0],
        ];

        Self {
            transformation: Mat4::from_scale_rotation_translation(
                vec3(ellipsoid.radius_x, ellipsoid.radius_y, ellipsoid.radius_z),
                ellipsoid.rotation,
                ellipsoid.position,
            ),
            normal,
            color: ellipsoid.color,
            latitude_bands: ellipsoid.latitude_bands,
            longitude_bands: ellipsoid.longitude_bands,
            _padding: [0.0; 2],
        }
    }

    pub fn vertices(&self) -> Vec<Vertex> {
        let mut grid_points = Vec::with_capacity(self.latitude_bands as usize + 1);
        let mut vertices = Vec::new();
    
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
                let tangent = vec3(-sin_phi, 0.0, cos_phi).normalize();
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
    
        // Create triangles from grid points
        for lat in 0..self.latitude_bands as usize {
            for lon in 0..self.longitude_bands as usize {
                let next_lon = lon + 1;
                let next_lat = lat + 1;
    
                // Vertices for the current quad
                let bottom_left = grid_points[lat][lon];
                let bottom_right = grid_points[lat][next_lon];
                let top_left = grid_points[next_lat][lon];
                let top_right = grid_points[next_lat][next_lon];
    
                // Avoid wrapping to poles
                if lat < self.latitude_bands as usize - 1 {
                    // Triangle 1
                    vertices.push(bottom_left);
                    vertices.push(bottom_right);
                    vertices.push(top_right);
    
                    // Triangle 2
                    vertices.push(bottom_left);
                    vertices.push(top_right);
                    vertices.push(top_left);
                }
            }
        }
    
        vertices
    }
}
