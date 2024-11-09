use super::super::Scene;
use iced::Rectangle;

#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct Uniforms {
    camera_proj: glam::Mat4, // 16 * 4 = 64 bytes
    camera_pos: glam::Vec4,  // 4 * 4 = 16 bytes
    light_color: glam::Vec4, // 4 * 4 = 16 bytes
    light_pos: glam::Vec3,   // 3 * 4 = 12 bytes
    _padding: f32,
}

impl Uniforms {
    pub fn new(scene: &Scene, bounds: Rectangle) -> Self {
        let camera_proj = scene.camera.build_view_proj_matrix(bounds);

        let light_color = scene.light_color.into_linear();

        Self {
            camera_proj,
            camera_pos: scene.camera.position(),
            light_color: glam::Vec4::new(
                light_color[0] as f32,
                light_color[1] as f32,
                light_color[2] as f32,
                light_color[3] as f32,
            ),
            light_pos: glam::Vec3::new(
                scene.light_pos[0] as f32,
                scene.light_pos[1] as f32,
                scene.light_pos[2] as f32,
            ),
            _padding: 0.0,
        }
    }
}
