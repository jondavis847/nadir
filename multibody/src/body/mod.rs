use crate::{
    result::{MultibodyResultTrait, ResultEntry},
    sensor::Sensor, MultibodyErrors,
};

use super::MultibodyTrait;
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
use std::collections::HashMap;
use transforms::Transform;
use uuid::Uuid;

use super::joint::JointTrait;

#[derive(Clone, Copy, Debug)]
pub enum BodyErrors {
    EmptyName,
    InnerJointExists,
    NoBaseInnerConnection,
    OuterJointExists,
}

impl std::fmt::Display for BodyErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BodyErrors::EmptyName => write!(f, "Name cannot be empty."),
            BodyErrors::InnerJointExists => write!(f, "The body already has an inner joint. Can only have one inner joint. Must disconnect first."),
            BodyErrors::NoBaseInnerConnection => write!(f, "The base cannot have an inner connection."),
            BodyErrors::OuterJointExists => write!(f, "The joint is already connected as an outer joint."),
        }
    }
}

impl std::error::Error for BodyErrors {}

pub trait BodyTrait: MultibodyTrait {
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
    pub mass_properties: MassProperties,
    pub sensors: Vec<Uuid>,
}

impl From<Body> for BodySim {
    fn from(body: Body) -> Self {
        let state = BodyState::default();
        Self {
            state,
            mesh: body.mesh,
            mass_properties: body.mass_properties,
            sensors: body.sensors,
        }
    }
}

impl BodySim {
    pub fn get_external_force_body(&self) -> &Force {
        &self.state.external_spatial_force_body
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

    pub fn initialize_result(&self) -> BodyResult {
        BodyResult::new()
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
pub struct BodyResult(HashMap<String, Vec<f64>>);

impl BodyResult {
    pub fn new() -> Self {
        let mut result = HashMap::new();
        Self::STATES.iter().for_each(|state| {
            result.insert(state.to_string(), Vec::new());
        });
        Self(result)
    }
}

impl MultibodyResultTrait for BodyResult {
    type Component = BodySim;
    const STATES: &'static [&'static str] = &[
        "acceleration[x]{base}",
        "acceleration[y]{base}",
        "acceleration[z]{base}",
        "acceleration[x]{body}",
        "acceleration[y]{body}",
        "acceleration[z]{body}",
        "angular_accel[x]{body}",
        "angular_accel[y]{body}",
        "angular_accel[z]{body}",
        "angular_rate[x]{body}",
        "angular_rate[y]{body}",
        "angular_rate[z]{body}",
        "attitude[w]{base}",
        "attitude[x]{base}",
        "attitude[y]{base}",
        "attitude[z]{base}",
        "external_force[x]{body}",
        "external_force[y]{body}",
        "external_force[z]{body}",
        "external_torque[x]{body}",
        "external_torque[y]{body}",
        "external_torque[z]{body}",
        "position[x]{base}",
        "position[y]{base}",
        "position[z]{base}",
        "velocity[x]{base}",
        "velocity[y]{base}",
        "velocity[z]{base}",
        "velocity[x]{body}",
        "velocity[y]{body}",
        "velocity[z]{body}",
    ];

    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Body(self.clone())
    }

    fn get_state_names(&self) -> &'static [&'static str] {
        Self::STATES
    }

    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        self.0.get(state).ok_or(MultibodyErrors::ComponentStateNotFound(state.to_string()))
    }

    fn update(&mut self, body: &BodySim) {
        self.0
            .get_mut("acceleration[x]{base}")
            .unwrap()
            .push(body.state.acceleration_base[0]);
        self.0
            .get_mut("acceleration[y]{base}")
            .unwrap()
            .push(body.state.acceleration_base[1]);
        self.0
            .get_mut("acceleration[z]{base}")
            .unwrap()
            .push(body.state.acceleration_base[2]);
        self.0
            .get_mut("acceleration[x]{body}")
            .unwrap()
            .push(body.state.acceleration_body[0]);
        self.0
            .get_mut("acceleration[y]{body}")
            .unwrap()
            .push(body.state.acceleration_body[1]);
        self.0
            .get_mut("acceleration[z]{body}")
            .unwrap()
            .push(body.state.acceleration_body[2]);
        self.0
            .get_mut("angular_accel[x]{body}")
            .unwrap()
            .push(body.state.angular_accel_body[0]);
        self.0
            .get_mut("angular_accel[y]{body}")
            .unwrap()
            .push(body.state.angular_accel_body[1]);
        self.0
            .get_mut("angular_accel[z]{body}")
            .unwrap()
            .push(body.state.angular_accel_body[2]);
        self.0
            .get_mut("angular_rate[x]{body}")
            .unwrap()
            .push(body.state.angular_rate_body[0]);
        self.0
            .get_mut("angular_rate[y]{body}")
            .unwrap()
            .push(body.state.angular_rate_body[1]);
        self.0
            .get_mut("angular_rate[z]{body}")
            .unwrap()
            .push(body.state.angular_rate_body[2]);
        self.0
            .get_mut("attitude[x]{base}")
            .unwrap()
            .push(body.state.attitude_base.x);
        self.0
            .get_mut("attitude[y]{base}")
            .unwrap()
            .push(body.state.attitude_base.y);
        self.0
            .get_mut("attitude[z]{base}")
            .unwrap()
            .push(body.state.attitude_base.z);
        self.0
            .get_mut("attitude[w]{base}")
            .unwrap()
            .push(body.state.attitude_base.s);
        self.0
            .get_mut("external_force[x]{body}")
            .unwrap()
            .push(body.state.external_force_body[0]);
        self.0
            .get_mut("external_force[y]{body}")
            .unwrap()
            .push(body.state.external_force_body[1]);
        self.0
            .get_mut("external_force[z]{body}")
            .unwrap()
            .push(body.state.external_force_body[2]);
        self.0
            .get_mut("external_torque[x]{body}")
            .unwrap()
            .push(body.state.external_torque_body[0]);
        self.0
            .get_mut("external_torque[y]{body}")
            .unwrap()
            .push(body.state.external_torque_body[1]);
        self.0
            .get_mut("external_torque[z]{body}")
            .unwrap()
            .push(body.state.external_torque_body[2]);
        self.0
            .get_mut("position[x]{base}")
            .unwrap()
            .push(body.state.position_base[0]);
        self.0
            .get_mut("position[y]{base}")
            .unwrap()
            .push(body.state.position_base[1]);
        self.0
            .get_mut("position[z]{base}")
            .unwrap()
            .push(body.state.position_base[2]);
        self.0
            .get_mut("velocity[x]{base}")
            .unwrap()
            .push(body.state.velocity_base[0]);
        self.0
            .get_mut("velocity[y]{base}")
            .unwrap()
            .push(body.state.velocity_base[1]);
        self.0
            .get_mut("velocity[z]{base}")
            .unwrap()
            .push(body.state.velocity_base[2]);
        self.0
            .get_mut("velocity[x]{body}")
            .unwrap()
            .push(body.state.velocity_body[0]);
        self.0
            .get_mut("velocity[y]{body}")
            .unwrap()
            .push(body.state.velocity_body[1]);
        self.0
            .get_mut("velocity[z]{body}")
            .unwrap()
            .push(body.state.velocity_body[2]);
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
