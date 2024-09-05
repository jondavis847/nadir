use super::super::Scene;
use iced::Rectangle;

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
    pub fn new(scene: &Scene, bounds: Rectangle) -> Self {
        let camera_proj = scene.camera.build_view_proj_matrix(bounds);

        Self {
            camera_proj,
            camera_pos: scene.camera.position(),
            light_color: glam::Vec4::from(scene.light_color.into_linear()),
            light_pos: glam::Vec3::from(scene.light_pos),
            _padding: 0.0,
        }
    }
}