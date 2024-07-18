use super::super::vertex::Vertex;
use iced::widget::shader::wgpu;
use glam::{vec2, vec3, Quat, Vec3, Mat3, Mat4};

/// A single instance of a cuboid.
#[derive(Debug, Clone)]
pub struct Cuboid {
    pub rotation: Quat,
    pub position: Vec3,
    pub length: f32, // x
    pub width: f32,  // y
    pub height: f32, // z
}

impl Default for Cuboid {
    fn default() -> Self {
        Self {
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
            length: 1.0,
            width: 1.0,
            height: 1.0,
        }
    }
}

impl Cuboid {
    pub fn new(length: f32, width: f32, height: f32, rotation: Quat, position: Vec3) -> Self {
        Self {
            rotation,
            position,
            length,
            width,
            height,
        }
    }

    pub fn update(&mut self, rotation: Quat, position: Vec3) {
        self.rotation = rotation;
        self.position = position;
    }
}

#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct CuboidRaw {
    transformation: Mat4, // 16 * 4 = 64 bytes
    normal: [[f32; 4]; 3], // 3 rows of 4 floats (padded Mat3)
    length: f32,          // 4 bytes
    width: f32,           // 4 bytes
    height: f32,          // 4 bytes
    _padding: [f32; 1],   // 4 bytes to ensure the struct size is a multiple of 16 bytes
}

impl CuboidRaw {
    const ATTRIBS: [wgpu::VertexAttribute; 7] = wgpu::vertex_attr_array![
        4 => Float32x4, // transformation row 1
        5 => Float32x4, // transformation row 2
        6 => Float32x4, // transformation row 3
        7 => Float32x4, // transformation row 4
        8 => Float32x4, // normal row 1 (padded Mat3)
        9 => Float32x4, // normal row 2 (padded Mat3)
        10 => Float32x4 // normal row 3 (padded Mat3)
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }

    pub fn from_cuboid(cuboid: &Cuboid) -> Self {
        let mat3 = Mat3::from_quat(cuboid.rotation);
        let normal = [
            [mat3.x_axis.x, mat3.x_axis.y, mat3.x_axis.z, 0.0],
            [mat3.y_axis.x, mat3.y_axis.y, mat3.y_axis.z, 0.0],
            [mat3.z_axis.x, mat3.z_axis.y, mat3.z_axis.z, 0.0],
        ];

        Self {
            transformation: Mat4::from_scale_rotation_translation(
                vec3(cuboid.length, cuboid.width, cuboid.height),
                cuboid.rotation,
                cuboid.position,
            ),
            normal,
            length: cuboid.length,
            width: cuboid.width,
            height: cuboid.height,
            _padding: [0.0; 1],
        }
    }

    pub fn vertices(&self) -> [Vertex; 36] {
        // Generate vertices for a cuboid, ensuring correct scaling
        let mut vertices = [Vertex::default(); 36];
        let mut idx = 0;

        // Define vertices for each face
        let faces = [
            // Define each face with 6 vertices
            (vec3(0.0, 0.0, -1.0), [
                vec3(-0.5, -0.5, -0.5),
                vec3( 0.5, -0.5, -0.5),
                vec3( 0.5,  0.5, -0.5),
                vec3( 0.5,  0.5, -0.5),
                vec3(-0.5,  0.5, -0.5),
                vec3(-0.5, -0.5, -0.5),
            ]),
            (vec3(0.0, 0.0, 1.0), [
                vec3(-0.5, -0.5, 0.5),
                vec3( 0.5, -0.5, 0.5),
                vec3( 0.5,  0.5, 0.5),
                vec3( 0.5,  0.5, 0.5),
                vec3(-0.5,  0.5, 0.5),
                vec3(-0.5, -0.5, 0.5),
            ]),
            (vec3(-1.0, 0.0, 0.0), [
                vec3(-0.5,  0.5,  0.5),
                vec3(-0.5,  0.5, -0.5),
                vec3(-0.5, -0.5, -0.5),
                vec3(-0.5, -0.5, -0.5),
                vec3(-0.5, -0.5,  0.5),
                vec3(-0.5,  0.5,  0.5),
            ]),
            (vec3(1.0, 0.0, 0.0), [
                vec3( 0.5,  0.5,  0.5),
                vec3( 0.5,  0.5, -0.5),
                vec3( 0.5, -0.5, -0.5),
                vec3( 0.5, -0.5, -0.5),
                vec3( 0.5, -0.5,  0.5),
                vec3( 0.5,  0.5,  0.5),
            ]),
            (vec3(0.0, -1.0, 0.0), [
                vec3(-0.5, -0.5, -0.5),
                vec3( 0.5, -0.5, -0.5),
                vec3( 0.5, -0.5,  0.5),
                vec3( 0.5, -0.5,  0.5),
                vec3(-0.5, -0.5,  0.5),
                vec3(-0.5, -0.5, -0.5),
            ]),
            (vec3(0.0, 1.0, 0.0), [
                vec3(-0.5,  0.5, -0.5),
                vec3( 0.5,  0.5, -0.5),
                vec3( 0.5,  0.5,  0.5),
                vec3( 0.5,  0.5,  0.5),
                vec3(-0.5,  0.5,  0.5),
                vec3(-0.5,  0.5, -0.5),
            ]),
        ];

        for (normal, positions) in faces.iter() {
            for pos in positions {
                vertices[idx] = Vertex {
                    pos: vec3(pos.x * self.length, pos.y * self.width, pos.z * self.height),
                    normal: *normal,
                    tangent: vec3(1.0, 0.0, 0.0), // Simplified, adjust if necessary
                    uv: vec2(pos.x + 0.5, pos.y + 0.5), // Simplified UV calculation
                };
                idx += 1;
            }
        }

        vertices
    }
}
