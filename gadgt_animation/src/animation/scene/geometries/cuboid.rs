use super::super::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4, Quat, Vec3};
use iced::widget::shader::wgpu;

/// A single instance of a cuboid.
#[derive(Debug, Clone)]
pub struct Cuboid {
    pub name: String,
    pub rotation: Quat,
    pub position: Vec3,
    pub length: f32, // x
    pub width: f32,  // y
    pub height: f32, // z
    pub color: [f32; 4],
}

impl Default for Cuboid {
    fn default() -> Self {
        Self {
            name: String::new(),
            rotation: Quat::IDENTITY,
            position: Vec3::ZERO,
            length: 1.0,
            width: 1.0,
            height: 1.0,
            color: [1.0, 1.0, 1.0, 1.0],
        }
    }
}

impl Cuboid {
    pub fn new(
        name: String,
        length: f32,
        width: f32,
        height: f32,
        rotation: Quat,
        position: Vec3,
        color: [f32; 4],
    ) -> Self {
        Self {
            name,
            rotation,
            position,
            length,
            width,
            height,
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
pub struct CuboidRaw {
    transformation: Mat4,  // 16 * 4 = 64 bytes
    normal: Mat3,          // 9 * 4 = 36 bytes
    color: [f32; 4],       // 16 bytes    
    _padding: [f32; 3],
}

impl CuboidRaw {
    const ATTRIBS: [wgpu::VertexAttribute; 8] = wgpu::vertex_attr_array![
        4 => Float32x4, // transformation row 1
        5 => Float32x4, // transformation row 2
        6 => Float32x4, // transformation row 3
        7 => Float32x4, // transformation row 4
        8 => Float32x3, // normal row 1 (padded Mat3)
        9 => Float32x3, // normal row 2 (padded Mat3)
        10 => Float32x3, // normal row 3 (padded Mat3)
        11 => Float32x4, // color (RGBA)
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }

    pub fn from_cuboid(cuboid: &Cuboid) -> Self {
        let transformation = Mat4::from_scale_rotation_translation(
            vec3(cuboid.length, cuboid.width, cuboid.height),
            cuboid.rotation,
            cuboid.position,
        );
        
        let normal = Mat3::from_mat4(transformation).inverse().transpose();

        Self {
            transformation,
            normal,
            color: cuboid.color,
            _padding: [0.0,0.0,0.0],
        }
    }

    pub fn vertices(&self) -> [Vertex; 36] {
        // Define vertices for each face
        let vertices = [
            //face 1
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(0.0, 0.0, -1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 2
            Vertex {
                pos: vec3(-0.5, -0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, 0.5),
                normal: vec3(0.0, 0.0, 1.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 3
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            //face 4
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(0.0, 0.0, -1.0),
                uv: vec2(0.0, 1.0),
            },
            //face 5
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, 0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, 0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, 0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(0.0, -1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //face 6
            Vertex {
                pos: vec3(-0.5, 0.5, -0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, -0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, -0.5),
                normal: vec3(0.0, 1.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
        ];

        vertices
    }
}
