use crate::{
    joint::{Joint, JointRef},
    result::{MultibodyResultTrait, ResultEntry},
};

use aerospace::{
    celestial_system::CelestialSystem,
    gravity::{Gravity, GravityTrait},
};
use mass_properties::MassProperties;
use nadir_3d::mesh::Mesh;
use nalgebra::{Vector3, Vector6};
use rotations::{quaternion::Quaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Force, SpatialTransform};
use std::{
    cell::RefCell,
    collections::HashMap,
    mem::take,
    rc::{Rc, Weak},
};
use thiserror::Error;
use transforms::Transform;

#[derive(Clone, Debug, Error)]
pub enum BodyErrors {
    #[error("name cannot be empty for body")]
    EmptyName,
    #[error("attempted to connect inner joint to body '{0}', but it already has an inner joint")]
    InnerJointExists(String),
    #[error("joint '{0}' already connected to {1} as an outer joint")]
    OuterJointExists(String, String),
}

pub trait BodyTrait {
    fn get_name(&self) -> String;
    fn connect_outer_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors>;
}

pub type BodyRef = Rc<RefCell<Body>>;
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Body {
    #[serde(skip)]
    pub inner_joint: Option<Weak<RefCell<Joint>>>,
    pub mass_properties: MassProperties,
    pub mesh: Option<Mesh>,
    pub name: String,
    #[serde(skip)]
    pub outer_joints: Vec<Weak<RefCell<Joint>>>, // id of joint in system.joints, joint contains the transform information
    #[serde(skip)]
    pub state: BodyState,
    #[serde(skip)]
    result: BodyResult,
}

impl Body {
    pub fn connect_inner_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists(self.name.to_string())),
            None => self.inner_joint = Some(Rc::downgrade(&joint.clone())),
        }
        Ok(())
    }

    pub fn new(name: &str, mass_properties: MassProperties) -> Result<Self, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Self {
            mesh: None,
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            result: BodyResult::default(),
            state: BodyState::default(),
        })
    }

    pub fn with_mesh(mut self, mesh: Mesh) -> Self {
        self.mesh = Some(mesh);
        self
    }

    pub fn calculate_gravity(&mut self, body_from_base: &SpatialTransform, gravity: &Gravity) {
        let g_vec = gravity.calculate(self.state.position_base);

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        self.state.gravity_force_body = body_from_base.0.rotation.transform(g_vec);
    }

    //TODO: combine this with calculate_gravity?
    pub fn calculate_gravity_celestial(
        &mut self,
        body_from_base: &SpatialTransform,
        celestial: &CelestialSystem,
    ) {
        let g_vec = celestial.calculate_gravity(self.state.position_base);

        // convert g_vec to a force by multiplying by mass
        // note that we just calculate gravity as translation of the cm
        // any torque applied via gravity and it's joints is handled by
        // the conversion to a joint force through spatial algebra
        let g_vec = g_vec * self.mass_properties.mass;

        self.state.gravity_force_base = g_vec;
        // transform to the body frame
        self.state.gravity_force_body = body_from_base.0.rotation.transform(g_vec);
    }

    pub fn calculate_external_force(&mut self) {
        // convert gravity to spatial force
        let gravity_force_body = Force::from(Vector6::new(
            0.0,
            0.0,
            0.0,
            self.state.gravity_force_body[0],
            self.state.gravity_force_body[1],
            self.state.gravity_force_body[2],
        ));

        // sum the forces
        self.state.external_spatial_force_body = gravity_force_body
            + self.state.actuator_force_body
            + self.state.environments_force_body;

        // populate values for reporting
        self.state.external_force_body = *self.state.external_spatial_force_body.translation();
        self.state.external_torque_body = *self.state.external_spatial_force_body.rotation();
    }

    pub fn update_acceleration(&mut self) {
        let inner_joint_weak = self.inner_joint.as_ref().unwrap();
        let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
        let inner_joint = inner_joint_rc.borrow();

        let transforms = &inner_joint.cache.transforms;
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
        let joint_a = inner_joint.cache.a;
        let body_a = body_from_joint * joint_a;
        // accel in body to accel in base is just a rotation, translation due to rotation should be accounted for in calc of body_a
        let body_a_in_base = base_from_body * body_a;
        self.state.acceleration_body = *body_a.translation();
        self.state.acceleration_base = *body_a_in_base.translation();
        self.state.angular_accel_body = *body_a.rotation();
    }

    pub fn update_state(&mut self) {
        let inner_joint_weak = self.inner_joint.as_ref().unwrap();
        let inner_joint_rc = inner_joint_weak.upgrade().unwrap();
        let inner_joint = inner_joint_rc.borrow();

        let transforms = &inner_joint.cache.transforms;
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;
        let joint_v = inner_joint.cache.v;
        let body_v = body_from_joint * joint_v;
        let body_v_in_base_translation = base_from_body.0.rotation.transform(*body_v.translation());
        self.state.velocity_body = *body_v.translation();
        self.state.velocity_base = body_v_in_base_translation;
        self.state.angular_rate_body = *body_v.rotation();
        let body_from_base = base_from_body.0.inv();
        self.state.position_base = body_from_base.translation.vec();
        self.state.attitude_base = Quaternion::from(&body_from_base.rotation);
    }
}

impl MultibodyResultTrait for Body {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert(
            "acceleration[x]{base}".to_string(),
            take(&mut self.result.acceleration_x_base),
        );
        result.insert(
            "acceleration[y]{base}".to_string(),
            take(&mut self.result.acceleration_y_base),
        );
        result.insert(
            "acceleration[z]{base}".to_string(),
            take(&mut self.result.acceleration_z_base),
        );
        result.insert(
            "acceleration[x]{body}".to_string(),
            take(&mut self.result.acceleration_x_body),
        );
        result.insert(
            "acceleration[y]{body}".to_string(),
            take(&mut self.result.acceleration_y_body),
        );
        result.insert(
            "acceleration[z]{body}".to_string(),
            take(&mut self.result.acceleration_z_body),
        );
        result.insert(
            "angular_accel[x]{body}".to_string(),
            take(&mut self.result.angular_accel_x_body),
        );
        result.insert(
            "angular_accel[y]{body}".to_string(),
            take(&mut self.result.angular_accel_y_body),
        );
        result.insert(
            "angular_accel[z]{body}".to_string(),
            take(&mut self.result.angular_accel_z_body),
        );
        result.insert(
            "angular_rate[x]{body}".to_string(),
            take(&mut self.result.angular_rate_x_body),
        );
        result.insert(
            "angular_rate[y]{body}".to_string(),
            take(&mut self.result.angular_rate_y_body),
        );
        result.insert(
            "angular_rate[z]{body}".to_string(),
            take(&mut self.result.angular_rate_z_body),
        );
        result.insert(
            "attitude[x]{base}".to_string(),
            take(&mut self.result.attitude_x_base),
        );
        result.insert(
            "attitude[y]{base}".to_string(),
            take(&mut self.result.attitude_y_base),
        );
        result.insert(
            "attitude[z]{base}".to_string(),
            take(&mut self.result.attitude_z_base),
        );
        result.insert(
            "attitude[w]{base}".to_string(),
            take(&mut self.result.attitude_w_base),
        );
        result.insert(
            "external_force[x]{body}".to_string(),
            take(&mut self.result.external_force_x_body),
        );
        result.insert(
            "external_force[y]{body}".to_string(),
            take(&mut self.result.external_force_y_body),
        );
        result.insert(
            "external_force[z]{body}".to_string(),
            take(&mut self.result.external_force_z_body),
        );
        result.insert(
            "external_torque[x]{body}".to_string(),
            take(&mut self.result.external_torque_x_body),
        );
        result.insert(
            "external_torque[y]{body}".to_string(),
            take(&mut self.result.external_torque_y_body),
        );
        result.insert(
            "external_torque[z]{body}".to_string(),
            take(&mut self.result.external_torque_z_body),
        );
        result.insert(
            "position[x]{base}".to_string(),
            take(&mut self.result.position_x_base),
        );
        result.insert(
            "position[y]{base}".to_string(),
            take(&mut self.result.position_y_base),
        );
        result.insert(
            "position[z]{base}".to_string(),
            take(&mut self.result.position_z_base),
        );
        result.insert(
            "velocity[x]{base}".to_string(),
            take(&mut self.result.velocity_x_base),
        );
        result.insert(
            "velocity[y]{base}".to_string(),
            take(&mut self.result.velocity_y_base),
        );
        result.insert(
            "velocity[z]{base}".to_string(),
            take(&mut self.result.velocity_z_base),
        );
        result.insert(
            "velocity[x]{body}".to_string(),
            take(&mut self.result.velocity_x_body),
        );
        result.insert(
            "velocity[y]{body}".to_string(),
            take(&mut self.result.velocity_y_body),
        );
        result.insert(
            "velocity[z]{body}".to_string(),
            take(&mut self.result.velocity_z_body),
        );

        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.acceleration_x_base = Vec::with_capacity(capacity);
        self.result.acceleration_y_base = Vec::with_capacity(capacity);
        self.result.acceleration_z_base = Vec::with_capacity(capacity);
        self.result.acceleration_x_body = Vec::with_capacity(capacity);
        self.result.acceleration_y_body = Vec::with_capacity(capacity);
        self.result.acceleration_z_body = Vec::with_capacity(capacity);
        self.result.angular_accel_x_body = Vec::with_capacity(capacity);
        self.result.angular_accel_y_body = Vec::with_capacity(capacity);
        self.result.angular_accel_z_body = Vec::with_capacity(capacity);
        self.result.angular_rate_x_body = Vec::with_capacity(capacity);
        self.result.angular_rate_y_body = Vec::with_capacity(capacity);
        self.result.angular_rate_z_body = Vec::with_capacity(capacity);
        self.result.attitude_w_base = Vec::with_capacity(capacity);
        self.result.attitude_x_base = Vec::with_capacity(capacity);
        self.result.attitude_y_base = Vec::with_capacity(capacity);
        self.result.attitude_z_base = Vec::with_capacity(capacity);
        self.result.external_force_x_body = Vec::with_capacity(capacity);
        self.result.external_force_y_body = Vec::with_capacity(capacity);
        self.result.external_force_z_body = Vec::with_capacity(capacity);
        self.result.external_torque_x_body = Vec::with_capacity(capacity);
        self.result.external_torque_y_body = Vec::with_capacity(capacity);
        self.result.external_torque_z_body = Vec::with_capacity(capacity);
        self.result.position_x_base = Vec::with_capacity(capacity);
        self.result.position_y_base = Vec::with_capacity(capacity);
        self.result.position_z_base = Vec::with_capacity(capacity);
        self.result.velocity_x_base = Vec::with_capacity(capacity);
        self.result.velocity_y_base = Vec::with_capacity(capacity);
        self.result.velocity_z_base = Vec::with_capacity(capacity);
        self.result.velocity_x_body = Vec::with_capacity(capacity);
        self.result.velocity_y_body = Vec::with_capacity(capacity);
        self.result.velocity_z_body = Vec::with_capacity(capacity);
    }
    fn update_result(&mut self) {
        self.result
            .acceleration_x_base
            .push(self.state.acceleration_base[0]);
        self.result
            .acceleration_y_base
            .push(self.state.acceleration_base[1]);
        self.result
            .acceleration_z_base
            .push(self.state.acceleration_base[2]);
        self.result
            .acceleration_x_body
            .push(self.state.acceleration_body[0]);
        self.result
            .acceleration_y_body
            .push(self.state.acceleration_body[1]);
        self.result
            .acceleration_z_body
            .push(self.state.acceleration_body[2]);
        self.result
            .angular_accel_x_body
            .push(self.state.angular_accel_body[0]);
        self.result
            .angular_accel_y_body
            .push(self.state.angular_accel_body[1]);
        self.result
            .angular_accel_z_body
            .push(self.state.angular_accel_body[2]);
        self.result
            .angular_rate_x_body
            .push(self.state.angular_rate_body[0]);
        self.result
            .angular_rate_y_body
            .push(self.state.angular_rate_body[1]);
        self.result
            .angular_rate_z_body
            .push(self.state.angular_rate_body[2]);
        self.result.attitude_w_base.push(self.state.attitude_base.s);
        self.result.attitude_x_base.push(self.state.attitude_base.x);
        self.result.attitude_y_base.push(self.state.attitude_base.y);
        self.result.attitude_z_base.push(self.state.attitude_base.z);
        self.result
            .external_force_x_body
            .push(self.state.external_force_body[0]);
        self.result
            .external_force_y_body
            .push(self.state.external_force_body[1]);
        self.result
            .external_force_z_body
            .push(self.state.external_force_body[2]);
        self.result
            .external_torque_x_body
            .push(self.state.external_torque_body[0]);
        self.result
            .external_torque_y_body
            .push(self.state.external_torque_body[1]);
        self.result
            .external_torque_z_body
            .push(self.state.external_torque_body[2]);
        self.result
            .position_x_base
            .push(self.state.position_base[0]);
        self.result
            .position_y_base
            .push(self.state.position_base[1]);
        self.result
            .position_z_base
            .push(self.state.position_base[2]);
        self.result
            .velocity_x_base
            .push(self.state.velocity_base[0]);
        self.result
            .velocity_y_base
            .push(self.state.velocity_base[1]);
        self.result
            .velocity_z_base
            .push(self.state.velocity_base[2]);
        self.result
            .velocity_x_body
            .push(self.state.velocity_body[0]);
        self.result
            .velocity_y_body
            .push(self.state.velocity_body[1]);
        self.result
            .velocity_z_body
            .push(self.state.velocity_body[2]);
    }
}

impl BodyTrait for Body {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn connect_outer_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors> {
        // Check if the joint already exists in outer_joints
        let name = joint.borrow().name.clone();
        for jointref in &self.outer_joints {
            if let Some(joint) = jointref.upgrade() {
                if joint.borrow().name == name {
                    return Err(BodyErrors::OuterJointExists(name, self.name.clone()));
                }
            }
        }

        // Push the new joint connection
        self.outer_joints.push(Rc::downgrade(&joint.clone()));
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct BodyState {
    pub position_base: Vector3<f64>,
    pub velocity_base: Vector3<f64>,
    pub velocity_body: Vector3<f64>,
    pub acceleration_base: Vector3<f64>,
    pub acceleration_body: Vector3<f64>,
    pub attitude_base: Quaternion,
    pub angular_rate_body: Vector3<f64>,
    pub angular_accel_body: Vector3<f64>,
    pub actuator_force_body: Force,
    pub environments_force_body: Force,
    pub gravity_force_base: Vector3<f64>,
    pub gravity_force_body: Vector3<f64>,
    pub external_spatial_force_body: Force, //used for calculations
    pub external_force_body: Vector3<f64>,  //use for reporting
    pub external_torque_body: Vector3<f64>, //use for reporting
    pub angular_momentum_body: Vector3<f64>,
    pub angular_momentum_base: Vector3<f64>,
    pub linear_momentum_body: Vector3<f64>,
    pub linear_momentum_base: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
struct BodyResult {
    acceleration_x_base: Vec<f64>,
    acceleration_y_base: Vec<f64>,
    acceleration_z_base: Vec<f64>,
    acceleration_x_body: Vec<f64>,
    acceleration_y_body: Vec<f64>,
    acceleration_z_body: Vec<f64>,
    angular_accel_x_body: Vec<f64>,
    angular_accel_y_body: Vec<f64>,
    angular_accel_z_body: Vec<f64>,
    angular_rate_x_body: Vec<f64>,
    angular_rate_y_body: Vec<f64>,
    angular_rate_z_body: Vec<f64>,
    attitude_w_base: Vec<f64>,
    attitude_x_base: Vec<f64>,
    attitude_y_base: Vec<f64>,
    attitude_z_base: Vec<f64>,
    external_force_x_body: Vec<f64>,
    external_force_y_body: Vec<f64>,
    external_force_z_body: Vec<f64>,
    external_torque_x_body: Vec<f64>,
    external_torque_y_body: Vec<f64>,
    external_torque_z_body: Vec<f64>,
    position_x_base: Vec<f64>,
    position_y_base: Vec<f64>,
    position_z_base: Vec<f64>,
    velocity_x_base: Vec<f64>,
    velocity_y_base: Vec<f64>,
    velocity_z_base: Vec<f64>,
    velocity_x_body: Vec<f64>,
    velocity_y_body: Vec<f64>,
    velocity_z_body: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyConnection {
    pub body: BodyRef,
    pub transform: Transform,
}
impl BodyConnection {
    pub fn new(body: BodyRef, transform: Transform) -> Self {
        Self { body, transform }
    }
}
