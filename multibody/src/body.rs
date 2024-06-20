use super::{base::Base, joint::JointRef, MultibodyTrait};
use mass_properties::{MassProperties, MassPropertiesErrors};
use spatial_algebra::Force;
use uuid::Uuid;

pub mod body_enum;
pub mod body_ref;
pub mod body_state;
pub mod connection_joint;

use body_enum::BodyEnum;
use body_ref::BodyRef;
use body_state::BodyState;
use connection_joint::BodyJointConnection;

#[derive(Clone, Copy, Debug)]
pub enum BodyErrors {
    EmptyName,
    InnerJointExists,
    MassPropertiesErrors(MassPropertiesErrors),
    NoBaseInnerConnection,
    OuterJointExists,
}

pub trait BodyTrait {
    fn connect_inner_joint(&mut self, joint_id: Uuid) -> Result<(), BodyErrors>;

    fn connect_outer_joint(&mut self, joint_id: Uuid) -> Result<(), BodyErrors>;

    fn delete_inner_joint(&mut self);
    fn delete_outer_joint(&mut self, joint_id: Uuid);
    fn get_external_force(&self) -> Force;
    fn get_inner_joint(&self) -> Option<BodyJointConnection>;
    fn get_outer_joints(&self) -> Vec<BodyJointConnection>;
    fn get_mass_properties(&self) -> MassProperties;
}

#[derive(Debug, Clone)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    inner_joint: Option<BodyJointConnection>,
    mass_properties: MassProperties,
    name: String,
    outer_joints: Vec<BodyJointConnection>,
    //sensors: Vec<BodySensorConnection>,
    state: BodyState,
}

impl Body {
    pub fn new(name: &str, mass_properties: MassProperties) -> Result<BodyRef, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Rc::new(RefCell::new(BodyEnum::Body(Self {
            //actuators: Vec::new(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            state: BodyState::default(),
        }))))
    }

    pub fn get_mass_properties(&self) -> MassProperties {
        self.mass_properties
    }
}

impl BodyTrait for Body {
    fn connect_inner_joint(&mut self, jointref: JointRef) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(BodyJointConnection::new(jointref)),
        }
        Ok(())
    }

    fn connect_outer_joint(&mut self, jointref: JointRef) -> Result<(), BodyErrors> {
        // Borrow the joint and get its name
        let joint_name = jointref.borrow().get_name().to_string();

        // Check if the joint already exists in outer_joints
        if self
            .outer_joints
            .iter()
            .any(|connection| connection.joint.borrow().get_name() == joint_name)
        {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints.push(BodyJointConnection::new(jointref));
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        if self.inner_joint.is_some() {
            self.inner_joint = None;
        }
    }

    fn delete_outer_joint(&mut self, jointref: JointRef) {
        self.outer_joints
            .retain(|connection| !Rc::ptr_eq(&connection.joint, &jointref));
    }

    fn get_external_force(&self) -> Force {
        self.state.external_force
    }

    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        self.inner_joint.clone()
    }

    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        self.outer_joints.clone()
    }

    fn get_mass_properties(&self) -> MassProperties {
        self.mass_properties
    }
}

impl MultibodyTrait for Body {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
