use super::{GeometryState, GeometryTrait, GeometryTransform};
use crate::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4, Quat};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum CuboidErrors {
    #[error("cuboid dimension must be greater than 0")]
    Dimension,
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct Cuboid {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Cuboid {
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self, CuboidErrors> {
        if x <= 0.0 || y <= 0.0 || z <= 0.0 {
            return Err(CuboidErrors::Dimension);
        }
        Ok(Self { x, y, z })
    }

    pub fn vertices() -> Vec<Vertex> {
        vec![
            //bottom face
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
            // top face
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
            //back face
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, -0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, -0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(-0.5, 0.5, 0.5),
                normal: vec3(-1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //front face
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 1.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, -0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(1.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, -0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 0.0),
            },
            Vertex {
                pos: vec3(0.5, 0.5, 0.5),
                normal: vec3(1.0, 0.0, 0.0),
                tangent: vec3(1.0, 0.0, 0.0),
                uv: vec2(0.0, 1.0),
            },
            //left face
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
            // right face
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
        ]
    }
}

impl GeometryTrait for Cuboid {
    fn get_mesh_transform(&self, state: &GeometryState) -> GeometryTransform {
        let transformation = Mat4::from_scale_rotation_translation(
            vec3(self.x as f32, self.y as f32, self.z as f32),
            Quat::from_xyzw(
                state.rotation.x as f32,
                state.rotation.y as f32,
                state.rotation.z as f32,
                state.rotation.w as f32,
            ),
            vec3(
                state.position.x as f32,
                state.position.y as f32,
                state.position.z as f32,
            ),
        );

        let normal = Mat3::from_mat4(transformation).inverse().transpose();
        GeometryTransform::new(transformation, normal)
    }
}
