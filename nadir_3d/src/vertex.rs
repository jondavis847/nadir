use glam::{Vec2, Vec3};
use iced::widget::shader::wgpu::{self, VertexBufferLayout};

#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable, Default)]
#[repr(C)]
pub struct Vertex {
    pub pos: Vec3,
    pub normal: Vec3,
    pub tangent: Vec3,
    pub uv: Vec2,
}

impl Vertex {
    const ATTRIBS: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
        //position
        0 => Float32x3,
        //normal
        1 => Float32x3,
        //tangent
        2 => Float32x3,
        //uv
        3 => Float32x2,
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

// a hashing key that allows for determining if a vertex already exists in the vertex buffer
#[derive(Hash, PartialEq, Eq)]
pub struct VertexKey {
    pos: [i32; 3],     // Quantized position
    normal: [i32; 3],  // Quantized normal
    uv: [i32; 2],      // Quantized UV coordinates
    tangent: [i32; 3], // Quantized tangent
}

impl VertexKey {
    pub fn from_vertex(vertex: &Vertex, precision: f32) -> Self {
        let quantize = |value: f32| (value * precision).round() as i32;

        Self {
            pos: [quantize(vertex.pos[0]), quantize(vertex.pos[1]), quantize(vertex.pos[2])],
            normal: [
                quantize(vertex.normal[0]),
                quantize(vertex.normal[1]),
                quantize(vertex.normal[2]),
            ],
            uv: [quantize(vertex.uv[0]), quantize(vertex.uv[1])],
            tangent: [
                quantize(vertex.tangent[0]),
                quantize(vertex.tangent[1]),
                quantize(vertex.tangent[2]),
            ],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SimpleVertex {
    pub pos: Vec3,
    pub _padding: f32, // Align to 16 bytes for GPU efficiency
}

impl SimpleVertex {
    const ATTRIBS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![
        //position
        0 => Float32x3,
    ];

    pub fn desc<'a>() -> VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}

// a hashing key that allows for determining if a vertex already exists in the vertex buffer
#[derive(Hash, PartialEq, Eq)]
pub struct SimpleVertexKey {
    pos: [i32; 3], // Quantized position
}

impl SimpleVertexKey {
    pub fn from_vertex(vertex: &SimpleVertex, precision: f32) -> Self {
        let quantize = |value: f32| (value * precision).round() as i32;

        Self {
            pos: [quantize(vertex.pos[0]), quantize(vertex.pos[1]), quantize(vertex.pos[2])],
        }
    }
}

pub fn simple_vertices(vertices: &Vec<Vertex>) -> Vec<SimpleVertex> {
    vertices
        .iter()
        .map(|v| SimpleVertex { pos: v.pos, _padding: 0.0 })
        .collect()
}
