use glam::{mat4, vec3, vec4, Quat, Vec2, Vec3};
use iced::{mouse::ScrollDelta, Rectangle, Vector};
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
            up: Vec3::Z,
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
        let aspect_ratio = bounds.width / bounds.height;

        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(self.fov_y, aspect_ratio, self.near, self.far);

        OPENGL_TO_WGPU_MATRIX * proj * view
    }

    pub fn position(&self) -> glam::Vec4 {
        glam::Vec4::from((self.eye, 0.0))
    }

    pub fn update_position_from_mouse_delta(&mut self, mouse_delta: Vector) {
        let mouse_delta = Vec2::new(mouse_delta.y, mouse_delta.x); // Swap y and x to match the mouse movement direction

        // Calculate the vector from the target to the camera
        let target_to_camera = self.eye - self.target;

        // Calculate the current forward, up, and right vectors
        let forward = target_to_camera.normalize();
        let up = Vec3::Y;
        let right = forward.cross(up).normalize();

        // Convert mouse delta to yaw and pitch
        let yaw = Quat::from_axis_angle(up, -mouse_delta.x * self.sensitivity);
        let pitch = Quat::from_axis_angle(right, -mouse_delta.y * self.sensitivity);

        // Combine yaw and pitch into a single rotation quaternion
        let rotation = yaw * pitch;

        // Apply the rotation to the forward vector
        let new_forward = rotation * forward;

        // Calculate the new camera position
        let new_camera_pos = self.target + new_forward * target_to_camera.length();

        // Update the camera position
        self.eye = new_camera_pos;
    }

    pub fn update_position_from_scroll_delta(&mut self, scroll_delta: ScrollDelta) {
        let delta = match scroll_delta {
            ScrollDelta::Lines { x, y } => {
                if x.abs() > y.abs() {
                    x
                } else {
                    y
                }
                //TODO: Play with scale of the mag
            }
            ScrollDelta::Pixels { x, y } => {
                if x.abs() > y.abs() {
                    x
                } else {
                    y
                }
                //TODO: Play with scale of the mag
            }
        };

        // Calculate the vector from the target to the camera
        let target_to_camera = self.eye - self.target;

        // Calculate the new camera position
        self.eye = target_to_camera * (1.0 + 0.1 * delta);
    }
}
