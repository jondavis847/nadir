use glam::{mat4, vec3, vec4, Vec2, Vec3};
use iced::Rectangle;
use std::f32::consts::PI;

#[derive(Copy, Clone, Debug)]
pub struct Camera {
    eye: Vec3,
    target: Vec3,
    up: Vec3,
    fov_y: f32,
    near: f32,
    far: f32,
    sensitivity: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            eye: vec3(5.0, 0.0, 0.0),
            target: Vec3::ZERO,
            up: Vec3::Y,
            fov_y: 45.0,
            near: 0.1,
            far: 100.0,
            sensitivity: 0.002,
        }
    }
}

pub const OPENGL_TO_WGPU_MATRIX: glam::Mat4 = mat4(
    vec4(1.0, 0.0, 0.0, 0.0),
    vec4(0.0, 1.0, 0.0, 0.0),
    vec4(0.0, 0.0, 0.5, 0.0),
    vec4(0.0, 0.0, 0.5, 1.0),
);

impl Camera {
    pub fn build_view_proj_matrix(&self, bounds: Rectangle) -> glam::Mat4 {
        //TODO looks distorted without padding; base on surface texture size instead?
        let aspect_ratio = bounds.width / (bounds.height + 150.0);

        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(self.fov_y, aspect_ratio, self.near, self.far);

        OPENGL_TO_WGPU_MATRIX * proj * view
    }

    pub fn position(&self) -> glam::Vec4 {
        glam::Vec4::from((self.eye, 0.0))
    }

    pub fn update_position_from_mouse_delta(&mut self, mouse_delta: iced::Vector) {
        let mouse_delta = Vec2::new(mouse_delta.y, mouse_delta.x); // not sure why i have to swap these
        // Calculate the vector from the target to the camera
        let target_to_camera = self.eye - self.target;

        // Calculate spherical coordinates (radius, theta, phi)
        let radius = target_to_camera.length();
        let theta = (target_to_camera.z / radius).acos();
        let phi = target_to_camera.y.atan2(target_to_camera.x);

        // Calculate angular changes from the mouse delta
        let delta_theta = -mouse_delta.y * self.sensitivity;
        let delta_phi = mouse_delta.x * self.sensitivity;
        
        // Update theta and phi
        let new_theta = (theta + delta_theta).clamp(0.0001, PI - 0.0001); // Avoid gimbal lock
        let new_phi = phi + delta_phi;

        // Convert spherical coordinates back to Cartesian coordinates
        let new_camera_pos = Vec3::new(
            radius * new_theta.sin() * new_phi.cos(),
            radius * new_theta.sin() * new_phi.sin(),
            radius * new_theta.cos(),
        );

        self.eye = new_camera_pos;
    }
}
