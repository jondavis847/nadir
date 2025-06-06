use glam::{Mat3, Quat, Vec3, mat4, vec3, vec4};
use iced::{Rectangle, Vector, mouse::ScrollDelta};

#[derive(Copy, Clone, Debug)]
pub struct Camera {
    eye: Vec3,
    target: Vec3,
    up: Vec3,
    rotation: Quat,
    pub fov_y: f32,
    near: f32,
    far: f32,
    sensitivity: f32,
}

impl Default for Camera {
    fn default() -> Self {
        let mut camera = Self {
            eye: vec3(5.0, 0.0, 0.0),
            target: vec3(0.0, 0.0, 0.0),
            up: Vec3::Z,
            rotation: Quat::IDENTITY,
            fov_y: 45.0,
            near: 0.1,
            far: 100.0,
            sensitivity: 0.003,
        };

        //update rotation to not be identity
        //camera.update_rotation();
        camera
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
        let aspect_ratio = bounds.width / bounds.height;
        let view = glam::Mat4::look_at_rh(self.eye, self.target, self.up);
        let proj = glam::Mat4::perspective_rh(
            self.fov_y * std::f32::consts::PI / 180.0,
            aspect_ratio,
            self.near,
            self.far,
        );

        OPENGL_TO_WGPU_MATRIX * proj * view
    }

    pub fn position(&self) -> glam::Vec4 {
        glam::Vec4::from((self.eye, 0.0))
    }

    pub fn set_far(&mut self, far: f32) {
        self.far = far;
    }

    pub fn set_fov(&mut self, fov: f32) {
        self.fov_y = fov;
    }

    /*
    pub fn set_near(&mut self, near: f32) {
        self.near = near;
    }
    */
    pub fn set_position(&mut self, pos: Vec3) {
        self.eye = pos;
    }

    pub fn set_target(&mut self, tar: Vec3) {
        self.target = tar;
    }

    pub fn update_position_from_mouse_delta(&mut self, mouse_delta: Vector) {
        // Calculate the vector from the camera to the target
        let camera_to_target = self.target - self.eye;

        // Calculate the current forward, up, and right vectors
        let forward = camera_to_target.normalize();
        let right = forward.cross(self.up).normalize();

        // Convert mouse delta to yaw and pitch
        let yaw = Quat::from_axis_angle(self.up, -mouse_delta.x * self.sensitivity);
        let pitch = Quat::from_axis_angle(right, mouse_delta.y * self.sensitivity);

        // Combine yaw and pitch into a single rotation quaternion and update camera rotation
        let incremental_rotation = yaw * pitch;
        self.rotation = incremental_rotation * self.rotation;

        // Apply the updated rotation to the forward vector
        let new_forward = incremental_rotation * camera_to_target;

        // Calculate the new camera position
        let new_camera_pos = self.target - new_forward;

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

        let zoom_factor = 0.1;
        let target_to_camera = self.eye - self.target;
        let zoom_amount = if delta >= 0.0 {
            1.0 + zoom_factor
        } else {
            1.0 - zoom_factor
        };
        //let limit = target_to_camera.normalize() * 0.1;
        let zoom_delta = target_to_camera * zoom_amount;
        self.eye = self.target + zoom_delta;
    }

    fn update_rotation(&mut self) {
        let forward = (self.target - self.eye).normalize();
        let right = forward.cross(self.up).normalize();
        //let rotation_matrix = Mat3::from_cols(right, self.up, forward);
        let rotation_matrix = Mat3::from_cols(-forward, right, self.up);
        self.rotation = Quat::from_mat3(&rotation_matrix);
    }
}
