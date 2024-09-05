use super::{GeometryState, GeometryTrait, GeometryTransform};
use crate::vertex::Vertex;
use glam::{vec2, vec3, Mat3, Mat4};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct Cuboid {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Cuboid {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn vertices() -> Vec<Vertex> {
        vec![
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
        ]
    }
}

impl GeometryTrait for Cuboid {
    fn get_mesh_transform(&self, state: &GeometryState) -> GeometryTransform {
        let transformation = Mat4::from_scale_rotation_translation(
            vec3(self.x, self.y, self.z),
            state.rotation,
            state.position,
        );

        let normal = Mat3::from_mat4(transformation).inverse().transpose();
        GeometryTransform::new(transformation, normal)
    }
}
