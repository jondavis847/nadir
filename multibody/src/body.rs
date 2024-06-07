use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;
use transforms::Transform;

use super::{
    base::Base,
    joint::JointRef,
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyTrait,
};

pub trait BodyTrait {
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors>;

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors>;

    fn delete_inner_joint(&mut self);
    fn delete_outer_joint(&mut self, jointref: JointRef);
    fn get_inner_joint(&self) -> Option<BodyJointConnection>;
    fn get_outer_joints(&self) -> Vec<BodyJointConnection>;
}

pub type BodyRef = Rc<RefCell<BodyEnum>>;

impl BodyTrait for BodyRef {
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_inner_joint(jointref, transform)
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_outer_joint(jointref, transform)
    }

    fn delete_inner_joint(&mut self) {
        self.borrow_mut().delete_inner_joint()
    }
    fn delete_outer_joint(&mut self, jointref: JointRef) {
        self.borrow_mut().delete_outer_joint(jointref)
    }

    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        self.borrow().get_inner_joint()
    }

    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        self.borrow().get_outer_joints()
    }
}

#[derive(Clone, Debug)]
pub enum BodyEnum {
    Base(Base),
    Body(Body),
}

impl BodyTrait for BodyEnum {
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        match self {
            BodyEnum::Base(base) => base.connect_inner_joint(jointref, transform),
            BodyEnum::Body(body) => body.connect_inner_joint(jointref, transform),
        }
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        match self {
            BodyEnum::Base(base) => base.connect_outer_joint(jointref, transform),
            BodyEnum::Body(body) => body.connect_outer_joint(jointref, transform),
        }
    }

    fn delete_inner_joint(&mut self) {
        match self {
            BodyEnum::Base(base) => base.delete_inner_joint(),
            BodyEnum::Body(body) => body.delete_inner_joint(),
        }
    }
    fn delete_outer_joint(&mut self, jointref: JointRef) {
        match self {
            BodyEnum::Base(base) => base.delete_outer_joint(jointref),
            BodyEnum::Body(body) => body.delete_outer_joint(jointref),
        }
    }

    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        match self {
            BodyEnum::Base(base) => base.get_inner_joint(),
            BodyEnum::Body(body) => body.get_inner_joint(),
        }
    }

    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        match self {
            BodyEnum::Base(base) => base.get_outer_joints(),
            BodyEnum::Body(body) => body.get_outer_joints(),
        }
    }
}

impl MultibodyTrait for BodyEnum {
    fn get_name(&self) -> &str {
        match self {
            BodyEnum::Base(base) => base.get_name(),
            BodyEnum::Body(body) => body.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            BodyEnum::Base(base) => base.set_name(name),
            BodyEnum::Body(body) => body.set_name(name),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum BodyErrors {
    EmptyName,
    InnerJointExists,
    MassPropertiesErrors(MassPropertiesErrors),
    NoBaseInnerConnection,
    OuterJointExists,
}

#[derive(Clone)]
pub struct BodyJointConnection {
    pub component: JointRef,
    pub transform: Transform,
}

impl BodyJointConnection {
    pub fn new(component: JointRef, transform: Transform) -> Self {
        Self {
            component,
            transform,
        }
    }
}

impl fmt::Debug for BodyJointConnection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let joint = self.component.borrow();

        f.debug_struct("BodyJointConnection")
            .field("joint_name", &joint.get_name())
            .finish()
    }
}

#[derive(Debug, Clone)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    inner_joint: Option<BodyJointConnection>,
    mass_properties: MassProperties,
    name: String,
    outer_joints: Vec<BodyJointConnection>,
    transforms: Vec<BodyTransforms>,
    //sensors: Vec<BodySensorConnection>,
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
            transforms: Vec::new(),
            //sensors: Vec::new(),
        }))))
    }

    /// Returns the x-coordinate of the center of mass.
    pub fn get_cmx(&self) -> f64 {
        self.mass_properties.get_cmx()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> f64 {
        self.mass_properties.get_cmy()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> f64 {
        self.mass_properties.get_cmz()
    }

    /// Returns the moment of inertia around the x-axis.
    pub fn get_ixx(&self) -> f64 {
        self.mass_properties.get_ixx()
    }

    /// Returns the product of inertia for the xy-plane.
    pub fn get_ixy(&self) -> f64 {
        self.mass_properties.get_ixy()
    }

    /// Returns the product of inertia for the xz-plane.
    pub fn get_ixz(&self) -> f64 {
        self.mass_properties.get_ixz()
    }

    /// Returns the moment of inertia around the y-axis.
    pub fn get_iyy(&self) -> f64 {
        self.mass_properties.get_iyy()
    }

    /// Returns the product of inertia for the yz-plane.
    pub fn get_iyz(&self) -> f64 {
        self.mass_properties.get_iyz()
    }

    /// Returns the moment of inertia around the z-axis.
    pub fn get_izz(&self) -> f64 {
        self.mass_properties.get_izz()
    }

    /// Returns the mass of the object.
    pub fn get_mass(&self) -> f64 {
        self.mass_properties.get_mass()
    }

    fn set_cmx(&mut self, cmx: f64) {
        self.mass_properties.set_cmx(cmx);
    }

    fn set_cmy(&mut self, cmy: f64) {
        self.mass_properties.set_cmy(cmy);
    }

    fn set_cmz(&mut self, cmz: f64) {
        self.mass_properties.set_cmz(cmz);
    }

    fn set_ixx(&mut self, ixx: f64) -> Result<(), BodyErrors> {
        match self.mass_properties.set_ixx(ixx) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_ixy(&mut self, ixy: f64) {
        self.mass_properties.set_ixy(ixy);
    }

    fn set_ixz(&mut self, ixz: f64) {
        self.mass_properties.set_ixz(ixz);
    }

    fn set_iyy(&mut self, iyy: f64) -> Result<(), BodyErrors> {
        match self.mass_properties.set_iyy(iyy) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_iyz(&mut self, iyz: f64) -> Result<(), BodyErrors> {
        match self.mass_properties.set_ixx(iyz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_izz(&mut self, izz: f64) -> Result<(), BodyErrors> {
        match self.mass_properties.set_izz(izz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_mass(&mut self, mass: f64) -> Result<(), BodyErrors> {
        match self.mass_properties.set_mass(mass) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }
}

impl BodyTrait for Body {
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(BodyJointConnection::new(jointref, transform)),
        }
        Ok(())
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef,
        transform: Transform,
    ) -> Result<(), BodyErrors> {
        // Borrow the joint and get its name
        let joint_name = jointref.borrow().get_name().to_string();

        // Check if the joint already exists in outer_joints
        if self
            .outer_joints
            .iter()
            .any(|connection| connection.component.borrow().get_name() == joint_name)
        {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints
            .push(BodyJointConnection::new(jointref, transform));
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        if self.inner_joint.is_some() {
            self.inner_joint = None;
        }
    }

    fn delete_outer_joint(&mut self, jointref: JointRef) {
        self.outer_joints
            .retain(|connection| !Rc::ptr_eq(&connection.component, &jointref));
    }

    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        self.inner_joint.clone()
    }

    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        self.outer_joints.clone()
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

/// base: the "body/base frame" of the base
/// body: the "body frame" of the current body
/// parent: the "body frame" of the parent body of the current body
/// ijof: the "inner joint outer frame" of the current body
#[derive(Clone, Copy, Debug)]
struct BodyTransforms {
    ijof_to_parent: Transform,
    ijof_to_base: Transform,
    parent_to_ijof: Transform,
    base_to_ijof: Transform,
    body_to_parent: Transform,
    body_to_base: Transform,
    parent_to_body: Transform,
    base_to_body: Transform,
}
