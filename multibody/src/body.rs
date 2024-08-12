use super::{aerospace::MultibodyGravity, MultibodyTrait};
use aerospace::gravity::{Gravity, GravityTrait};
use geometry::Geometry;
use mass_properties::{MassProperties, MassPropertiesErrors};
use nalgebra::{Vector3, Vector6};
use rotations::quaternion::Quaternion;
use spatial_algebra::{Acceleration, Force, SpatialTransform};
use std::collections::HashMap;
use uuid::Uuid;
use serde::{Serialize, Deserialize};

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

#[derive(Debug, Clone,Serialize, Deserialize)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    pub id: Uuid,
    pub inner_joint: Option<Uuid>,
    pub mass_properties: MassProperties,
    pub name: String,
    pub outer_joints: Vec<Uuid>,
    pub geometry: Option<Geometry>,
    pub gravity: Vec<Uuid>, // a vec in case say you want moon and earth or something
                            //sensors: Vec<BodySensorConnection>,
}

impl Body {
    pub fn connect_inner_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(*joint.get_id()),
        }
        Ok(())
    }
    pub fn delete_inner_joint(&mut self) {
        if self.inner_joint.is_some() {
            self.inner_joint = None;
        }
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
            geometry: None,
            gravity: Vec::new(),
            id: Uuid::new_v4(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
        })
    }

    pub fn with_geometry(mut self, geometry: Geometry) -> Self {
        self.geometry = Some(geometry);
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

#[derive(Clone, Default, Debug)]
pub struct BodySim {
    pub state: BodyState,
    pub geometry: Option<Geometry>,
    pub gravity: Vec<Uuid>,
    pub mass_properties: MassProperties,
}

impl From<Body> for BodySim {
    fn from(body: Body) -> Self {
        let state = BodyState::default();
        Self {
            state,
            geometry: body.geometry,
            gravity: body.gravity,
            mass_properties: body.mass_properties,
        }
    }
}

impl BodySim {
    pub fn get_external_force_body(&self) -> &Force {
        &self.state.external_spatial_force_body
    }

    pub fn calculate_gravity_acceleration_base(
        &mut self,
        gravities: &HashMap<Uuid, MultibodyGravity>,
    ) -> Vector3<f64> {
        let mut g_vec = Vector3::zeros();
        self.gravity.iter().for_each(|gravity_id| {
            let gravity = gravities.get(gravity_id).unwrap();
            g_vec += gravity.gravity.calculate(self.state.position_base);
        });
        g_vec
    }
    pub fn calculate_external_force(&mut self) {
        //reset
        self.state.external_spatial_force_body = Vector6::zeros().into();
        //just add body forces, gravity is treated differently
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyState {
    pub position_base: Vector3<f64>,
    pub velocity_base: Vector3<f64>,
    pub acceleration_base: Vector3<f64>,
    pub acceleration_body: Vector3<f64>,
    pub attitude_base: Quaternion,
    pub angular_rate_body: Vector3<f64>,
    pub angular_accel_body: Vector3<f64>,
    pub external_spatial_force_body: Force, //used for calculations
    pub external_spatial_force_joint: Force, //used for calculations
    pub external_force_body: Vector3<f64>,  //use for reporting
    pub external_torque_body: Vector3<f64>, //use for reporting
    pub angular_momentum_body: Vector3<f64>,
    pub angular_momentum_base: Vector3<f64>,    
    pub linear_momentum_body: Vector3<f64>,
    pub linear_momentum_base: Vector3<f64>,
}

#[derive(Debug, Clone, Default)]
pub struct BodyResult {
    pub position_base: Vec<Vector3<f64>>,
    pub velocity_base: Vec<Vector3<f64>>,
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
