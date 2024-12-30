use super::geometry::{Geometry, GeometryState, GeometryTrait};
use super::material::Material;
use super::texture::Texture;
use glam::{DQuat, DVec3, Mat3, Mat4};
use iced::widget::shader::wgpu;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Mesh {
    pub name: String,
    pub geometry: Geometry,
    pub material: Material,
    pub state: GeometryState,
    pub texture: Option<Texture>,
}

impl Mesh {
    pub fn set_position_from_target(&mut self, target: DVec3) {
        self.state.position -= target;
    }

    pub fn update(&mut self, position: DVec3, rotation: DQuat) {
        self.state.position = position;
        self.state.rotation = rotation;
    }
}

#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct MeshGpu {
    transformation: Mat4, // 16 * 4 = 64 bytes
    normal: Mat3,         // 9 * 4 = 36 bytes
    color: [f32; 4],      // 16 bytes
    material: u32,        // 4
    specular_power: f32,  // 4
    _padding: f32,
}

impl MeshGpu {
    const ATTRIBS: [wgpu::VertexAttribute; 10] = wgpu::vertex_attr_array![
        4 => Float32x4,     // transformation row 1
        5 => Float32x4,     // transformation row 2
        6 => Float32x4,     // transformation row 3
        7 => Float32x4,     // transformation row 4
        8 => Float32x3,     // normal row 1
        9 => Float32x3,     // normal row 2
        10 => Float32x3,    // normal row 3
        11 => Float32x4,    // color (RGBA)
        12 => Uint32,          // material
        13 => Float32,          // specular power
    ];

    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBS,
        }
    }
}

impl From<&Mesh> for MeshGpu {
    fn from(mesh: &Mesh) -> Self {
        let transforms = mesh.geometry.get_mesh_transform(&mesh.state);
        let (color, material, specular_power) = match &mesh.material {
            Material::Basic { color } => (color, 0, 0.0),
            Material::Phong {
                color,
                specular_power,
            } => (color, 1, *specular_power),
        };
        MeshGpu {
            transformation: transforms.transformation_matrix,
            normal: transforms.normal_matrix,
            color: color.into(),
            material,
            specular_power,
            _padding: 0.0,
        }
    }
}

#[derive(Debug)]
pub struct MeshPrimitive {
    pub mesh_gpu: MeshGpu,
    pub geometry: Geometry,
}

impl From<&Mesh> for MeshPrimitive {
    fn from(mesh: &Mesh) -> Self {
        let transforms = mesh.geometry.get_mesh_transform(&mesh.state);
        let (color, material, specular_power) = match &mesh.material {
            Material::Basic { color } => (color, 0, 0.0),
            Material::Phong {
                color,
                specular_power,
            } => (color, 1, *specular_power),
        };
        let mesh_gpu = MeshGpu {
            transformation: transforms.transformation_matrix,
            normal: transforms.normal_matrix,
            color: color.into(),
            material,
            specular_power,
            _padding: 0.0,
        };

        MeshPrimitive {
            geometry: mesh.geometry,
            mesh_gpu,
        }
    }
}
