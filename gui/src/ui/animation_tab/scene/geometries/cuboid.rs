use super::super::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4, Quat, Vec3};
use iced::widget::shader::wgpu;

/// A single instance of a cuboid.
#[derive(Debug, Clone)]
pub struct Cuboid {
    pub name: String,
    pub rotation: Quat,
    pub position: Vec3,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Cuboid {
    fn default() -> Self {
        Self {
            name: String::new(),
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
            x: 1.0,
            y: 1.0,
            z: 1.0,
        }
    }
}

impl Cuboid {
    pub fn new(name: String, x: f32, y: f32, z: f32, rotation: Quat, position: Vec3) -> Self {
        Self {
            name,
            rotation,
            position,
            x,
            y,
            z,
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
    transformation: Mat4,  // 16 * 4 = 64 bytes
    normal: [[f32; 4]; 3], // 3 rows of 4 floats (padded Mat3)
    x: f32,                // 4 bytes
    y: f32,                // 4 bytes
    z: f32,                // 4 bytes
    _padding: [f32; 1],    // 4 bytes to ensure the struct size is a multiple of 16 bytes
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
                vec3(cuboid.x, cuboid.y, cuboid.z),
                cuboid.rotation,
                cuboid.position,
            ),
            normal,
            x: cuboid.x,
            y: cuboid.y,
            z: cuboid.z,
            _padding: [0.0; 1],
        }
    }

    pub fn vertices(&self) -> [Vertex; 36] {
        // Define vertices for each face
        let vertices = [
            //face 1
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 2
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 3
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            //face 4
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            //face 5
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, -0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 6
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, 0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5 * self.x, 0.5 * self.y, -0.5 * self.z),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
        ];

        vertices
    }
}
