use super::super::camera::Camera;

use iced::{Color, Rectangle};

#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct Uniforms {
    camera_proj: glam::Mat4,
    camera_pos: glam::Vec4,
    light_color: glam::Vec4,        
    light_pos: glam:: Vec3,
    _padding: f32,
}

impl Uniforms {
    pub fn new(camera: &Camera, bounds: Rectangle, light_color: Color, light_pos: [f32;3]) -> Self {
        let camera_proj = camera.build_view_proj_matrix(bounds);

        Self {
            camera_proj,
            camera_pos: camera.position(),
            light_color: glam::Vec4::from(light_color.into_linear()),
            light_pos: glam::Vec3::from(light_pos),
            _padding: 0.0,
        }
    }
}