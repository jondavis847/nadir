use crate::{
    result::{MultibodyResultTrait, ResultEntry},
    sensor::Sensor,
};

use super::{aerospace::MultibodyGravity, MultibodyTrait};
use aerospace::gravity::GravityTrait;
use gadgt_3d::mesh::Mesh;
use mass_properties::{MassProperties, MassPropertiesErrors};
use nalgebra::{Vector3, Vector6};
use rotations::{quaternion::Quaternion, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::{Force, SpatialTransform};
use std::collections::HashMap;
use transforms::Transform;
use uuid::Uuid;

pub mod body_enum;
use super::joint::JointTrait;

#[derive(Clone, Copy, Debug)]
pub enum BodyErrors {
    EmptyName,
    InnerJointExists,
    MassPropertiesErrors(MassPropertiesErrors),
    NoBaseInnerConnection,
    OuterJointExists,
}

pub trait BodyTrait: MultibodyTrait {
    fn connect_gravity(&mut self, gravity: &MultibodyGravity);
    fn connect_outer_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors>;
    fn delete_outer_joint(&mut self, joint_id: &Uuid);
    fn get_outer_joints(&self) -> &Vec<Uuid>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    pub id: Uuid,
    pub inner_joint: Option<Uuid>,
    pub mass_properties: MassProperties,
    pub name: String,
    pub outer_joints: Vec<Uuid>, // id of joint in system.joints, joint contains the transform information
    pub mesh: Option<Mesh>,
    pub gravity: Vec<Uuid>, // a vec in case say you want moon and earth or something
    pub sensors: Vec<Uuid>, // id of sensor in system.sensors, sensor contains the transform information
}

impl Body {
    pub fn connect_inner_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(*joint.get_id()),
        }
        Ok(())
    }

    pub fn connect_sensor(&mut self, sensor: Uuid) -> Result<(), BodyErrors> {
        // only add if it's not already there
        if !self.sensors.contains(&sensor) {
            self.sensors.push(sensor);
        }
        Ok(())
    }

    pub fn delete_inner_joint(&mut self) {
        if self.inner_joint.is_some() {
            self.inner_joint = None;
        }
    }

    pub fn delete_sensor(&mut self, sensor: Uuid) -> Result<(), BodyErrors> {
        self.sensors.retain(|&id| id != sensor);
        Ok(())
    }

    pub fn get_inner_joint_id(&self) -> &Option<Uuid> {
        &self.inner_joint
    }

    pub fn get_mass_properties(&self) -> &MassProperties {
        &self.mass_properties
    }

    pub fn new(name: &str, mass_properties: MassProperties) -> Result<Self, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Self {
            //actuators: Vec::new(),
            mesh: None,
            gravity: Vec::new(),
            id: Uuid::new_v4(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            sensors: Vec::new(),
        })
    }

    pub fn with_mesh(mut self, mesh: Mesh) -> Self {
        self.mesh = Some(mesh);
        self
    }
}

impl BodyTrait for Body {
    fn connect_outer_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
        let joint_id = joint.get_id();
        // Check if the joint already exists in outer_joints
        if self.outer_joints.iter().any(|id| id == joint_id) {
            return Err(BodyErrors::OuterJointExists);
        }
        // Push the new joint connection
        self.outer_joints.push(*joint_id);
        Ok(())
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        self.outer_joints.retain(|id| id != joint_id);
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        &self.outer_joints
    }

    fn connect_gravity(&mut self, gravity: &MultibodyGravity) {
        self.gravity.push(*gravity.get_id())
    }
}

impl MultibodyTrait for Body {
    fn get_id(&self) -> &Uuid {
        &self.id
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}

/// It is not expected that you would create this directly
/// Instead, create a Body and call BodySim::from(Body), or
/// Body.into() if appropriate

#[derive(Clone, Default, Debug, Serialize, Deserialize)]
pub struct BodySim {
    pub state: BodyState,
    pub mesh: Option<Mesh>,
    pub gravity: Vec<Uuid>,
    pub mass_properties: MassProperties,
    pub sensors: Vec<Uuid>,
}

impl From<Body> for BodySim {
    fn from(body: Body) -> Self {
        let state = BodyState::default();
        Self {
            state,
            mesh: body.mesh,
            gravity: body.gravity,
            mass_properties: body.mass_properties,
            sensors: body.sensors,
        }
    }
}

impl BodySim {
    pub fn get_external_force_body(&self) -> &Force {
        &self.state.external_spatial_force_body
    }

    pub fn calculate_gravity(
        &mut self,
        body_from_base: &SpatialTransform,
        gravities: &HashMap<Uuid, MultibodyGravity>,
    ) {
        // loop over all gravities attached to body to calculate and sum the accelerations
        let mut g_vec = Vector3::zeros();
        self.gravity.iter().for_each(|gravity_id| {
            let gravity = gravities.get(gravity_id).unwrap();
            g_vec += gravity.gravity.calculate(self.state.position_base);
        });

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

    pub fn initialize_result(&self) -> BodyResult {
        BodyResult::default()
    }

    pub fn update_sensors(&self, sensors: &mut HashMap<Uuid, Sensor>) {
        self.sensors.iter().for_each(|id| {
            if let Some(sensor) = sensors.get_mut(id) {
                sensor.update(self);
            }
        })
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BodyResult {
    pub position_base: Vec<Vector3<f64>>,
    pub velocity_base: Vec<Vector3<f64>>,
    pub velocity_body: Vec<Vector3<f64>>,
    pub acceleration_base: Vec<Vector3<f64>>,
    pub acceleration_body: Vec<Vector3<f64>>,
    pub attitude_base: Vec<Quaternion>,
    pub angular_rate_body: Vec<Vector3<f64>>,
    pub angular_accel_body: Vec<Vector3<f64>>,
    pub external_force_body: Vec<Vector3<f64>>,
    pub external_torque_body: Vec<Vector3<f64>>,
    pub angular_momentum_body: Vec<Vector3<f64>>,
    pub angular_momentum_base: Vec<Vector3<f64>>,
    pub linear_momentum_body: Vec<Vector3<f64>>,
    pub linear_momentum_base: Vec<Vector3<f64>>,
}

impl BodyResult {
    pub fn update(&mut self, body: &BodySim) {
        self.position_base.push(body.state.position_base);
        self.velocity_base.push(body.state.velocity_base);
        self.acceleration_base.push(body.state.acceleration_base);
        self.acceleration_body.push(body.state.acceleration_body);
        self.angular_accel_body.push(body.state.angular_accel_body);
        self.angular_rate_body.push(body.state.angular_rate_body);
        self.attitude_base.push(body.state.attitude_base);
        self.external_force_body
            .push(body.state.external_force_body);
        self.external_torque_body
            .push(body.state.external_torque_body);
    }
}

impl MultibodyResultTrait for BodyResult {
    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Body(self.clone())
    }
    fn get_state_names(&self) -> Vec<&'static str> {
        vec![
                    "accel_base_x",
                    "accel_base_y",
                    "accel_base_z",
                    "accel_body_x",
                    "acceleration_body_y",
                    "acceleration_body_z",
                    "angular_accel_body_x",
                    "angular_accel_body_y",
                    "angular_accel_body_z",
                    "angular_rate_body_x",
                    "angular_rate_body_y",
                    "angular_rate_body_z",
                    "attitude_base_s",
                    "attitude_base_x",
                    "attitude_base_y",
                    "attitude_base_z",
                    "external_force_body_x",
                    "external_force_body_y",
                    "external_force_body_z",
                    "external_torque_body_x",
                    "external_torque_body_y",
                    "external_torque_body_z",
                    "position_base_x",
                    "position_base_y",
                    "position_base_z",
                    "velocity_base_x",
                    "velocity_base_y",
                    "velocity_base_z",
                ]
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodyConnection {
    pub body_id: Uuid,
    pub transform: Transform,
}
impl BodyConnection {
    pub fn new(body_id: Uuid, transform: Transform) -> Self {
        Self { body_id, transform }
    }
}
