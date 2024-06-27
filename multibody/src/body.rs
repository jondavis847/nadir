use super::MultibodyTrait;
use linear_algebra::vector3::Vector3;
use mass_properties::{MassProperties, MassPropertiesErrors};
use rotations::quaternion::Quaternion;
use spatial_algebra::Force;
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
    fn connect_outer_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors>;
    fn delete_outer_joint(&mut self, joint_id: &Uuid);
    fn get_outer_joints(&self) -> &Vec<Uuid>;
}

#[derive(Debug, Clone)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    id: Uuid,
    inner_joint: Option<Uuid>,
    mass_properties: MassProperties,
    name: String,
    outer_joints: Vec<Uuid>,
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
            id: Uuid::new_v4(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
        })
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

#[derive(Clone, Copy, Default, Debug)]
pub struct BodySim {
    pub state: BodyState,
}

impl From<Body> for BodySim {
    fn from(body: Body) -> Self {
        let state = BodyState::default();
        Self { state }
    }
}

impl BodySim {
    fn set_state(&mut self, state: &BodyState) {
        self.state = *state;
    }

    fn get_state(&self) -> &BodyState {
        &self.state
    }

    pub fn get_external_force(&self) -> &Force {
        &self.state.external_force
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyState {
    pub position_base: Vector3,
    pub velocity_base: Vector3,
    pub acceleration_base: Vector3,
    pub acceleration_body: Vector3,
    pub attitude_base: Quaternion,
    pub angular_rate_body: Vector3,
    pub angular_accel_body: Vector3,
    pub external_force: Force,         //used for calculations
    pub external_force_body: Vector3,  //use for reporting
    pub external_torque_body: Vector3, //use for reporting
}

#[derive(Debug, Clone, Default)]
pub struct BodyResult {
    pub position_base: Vec<Vector3>,
    pub velocity_base: Vec<Vector3>,
    pub acceleration_base: Vec<Vector3>,
    pub acceleration_body: Vec<Vector3>,
    pub attitude_base: Vec<Quaternion>,
    pub angular_rate_body: Vec<Vector3>,
    pub angular_accel_body: Vec<Vector3>,
    pub external_force_body: Vec<Vector3>,
    pub external_torque_body: Vec<Vector3>,
}
