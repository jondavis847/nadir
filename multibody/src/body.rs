use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;
use std::fmt;
use transforms::Transform;

use super::{
    base::Base,
    joint::JointRef,
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyTrait,
};

pub type BodyRef<T> = Rc<RefCell<Bodies<T>>>;

impl<T> BodyTrait<T> for BodyRef<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_inner_joint(jointref, transform)
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_outer_joint(jointref, transform)
    }

    fn delete_inner_joint(&mut self) {
        self.borrow_mut().delete_inner_joint()
    }
    fn delete_outer_joint(&mut self, jointref: JointRef<T>) {
        self.borrow_mut().delete_outer_joint(jointref)
    }
}
pub trait BodyTrait<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors>;

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors>;

    fn delete_inner_joint(&mut self);
    fn delete_outer_joint(&mut self, jointref: JointRef<T>);
}

#[derive(Clone, Debug)]
pub enum Bodies<T>
where
    T: SimValue,
{
    Base(Base<T>),
    Body(Body<T>),
}

impl<T> BodyTrait<T> for Bodies<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        match self {
            Bodies::Base(base) => base.connect_inner_joint(jointref, transform),
            Bodies::Body(body) => body.connect_inner_joint(jointref, transform),
        }
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        match self {
            Bodies::Base(base) => base.connect_outer_joint(jointref, transform),
            Bodies::Body(body) => body.connect_outer_joint(jointref, transform),
        }
    }

    fn delete_inner_joint(&mut self) {
        match self {
            Bodies::Base(base) => base.delete_inner_joint(),
            Bodies::Body(body) => body.delete_inner_joint(),
        }
    }
    fn delete_outer_joint(&mut self, jointref: JointRef<T>) {
        match self {
            Bodies::Base(base) => base.delete_outer_joint(jointref),
            Bodies::Body(body) => body.delete_outer_joint(jointref),
        }
    }
}

impl<T> MultibodyTrait for Bodies<T>
where
    T: SimValue,
{
    fn get_name(&self) -> &str {
        match self {
            Bodies::Base(base) => base.get_name(),
            Bodies::Body(body) => body.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            Bodies::Base(base) => base.set_name(name),
            Bodies::Body(body) => body.set_name(name),
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
pub struct BodyJointConnection<T>
where
    T: SimValue,
{
    pub component: JointRef<T>,
    pub transform: Transform<T>,
}

impl<T> BodyJointConnection<T>
where
    T: SimValue,
{
    pub fn new(component: JointRef<T>, transform: Transform<T>) -> Self {
        Self {
            component,
            transform,
        }
    }
}

impl<T> fmt::Debug for BodyJointConnection<T>
where
    T: SimValue,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let joint = self.component.borrow();        

        f.debug_struct("BodyJointConnection")
            .field("joint_name", &joint.get_name())            
            .finish()
    }
}

#[derive(Debug, Clone)]
pub struct Body<T>
where
    T: SimValue,
{
    //actuators: Vec<BodyActuatorConnection<T>>,
    inner_joint: Option<BodyJointConnection<T>>,
    mass_properties: MassProperties<T>,
    name: String,
    outer_joints: Vec<BodyJointConnection<T>>,
    //sensors: Vec<BodySensorConnection<T>>,
}

impl<T> Body<T>
where
    T: SimValue,
{
    pub fn new(name: &str, mass_properties: MassProperties<T>) -> Result<BodyRef<T>, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Rc::new(RefCell::new(Bodies::Body(Self {
            //actuators: Vec::new(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            //sensors: Vec::new(),
        }))))
    }

    /// Returns the x-coordinate of the center of mass.
    pub fn get_cmx(&self) -> T {
        self.mass_properties.get_cmx()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmy(&self) -> T {
        self.mass_properties.get_cmy()
    }

    /// Returns the y-coordinate of the center of mass.
    pub fn get_cmz(&self) -> T {
        self.mass_properties.get_cmz()
    }

    /// Returns the moment of inertia around the x-axis.
    pub fn get_ixx(&self) -> T {
        self.mass_properties.get_ixx()
    }

    /// Returns the product of inertia for the xy-plane.
    pub fn get_ixy(&self) -> T {
        self.mass_properties.get_ixy()
    }

    /// Returns the product of inertia for the xz-plane.
    pub fn get_ixz(&self) -> T {
        self.mass_properties.get_ixz()
    }

    /// Returns the moment of inertia around the y-axis.
    pub fn get_iyy(&self) -> T {
        self.mass_properties.get_iyy()
    }

    /// Returns the product of inertia for the yz-plane.
    pub fn get_iyz(&self) -> T {
        self.mass_properties.get_iyz()
    }

    /// Returns the moment of inertia around the z-axis.
    pub fn get_izz(&self) -> T {
        self.mass_properties.get_izz()
    }

    /// Returns the mass of the object.
    pub fn get_mass(&self) -> T {
        self.mass_properties.get_mass()
    }

    fn set_cmx(&mut self, cmx: T) {
        self.mass_properties.set_cmx(cmx);
    }

    fn set_cmy(&mut self, cmy: T) {
        self.mass_properties.set_cmy(cmy);
    }

    fn set_cmz(&mut self, cmz: T) {
        self.mass_properties.set_cmz(cmz);
    }

    fn set_ixx(&mut self, ixx: T) -> Result<(), BodyErrors> {
        match self.mass_properties.set_ixx(ixx) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_ixy(&mut self, ixy: T) {
        self.mass_properties.set_ixy(ixy);
    }

    fn set_ixz(&mut self, ixz: T) {
        self.mass_properties.set_ixz(ixz);
    }

    fn set_iyy(&mut self, iyy: T) -> Result<(), BodyErrors> {
        match self.mass_properties.set_iyy(iyy) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_iyz(&mut self, iyz: T) -> Result<(), BodyErrors> {
        match self.mass_properties.set_ixx(iyz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_izz(&mut self, izz: T) -> Result<(), BodyErrors> {
        match self.mass_properties.set_izz(izz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }

    fn set_mass(&mut self, mass: T) -> Result<(), BodyErrors> {
        match self.mass_properties.set_mass(mass) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(()),
        }
    }
}

impl<T> BodyTrait<T> for Body<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(BodyJointConnection::new(jointref, transform)),
        }
        Ok(())
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
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

    fn delete_outer_joint(&mut self, jointref: JointRef<T>) {
        self.outer_joints
            .retain(|connection| !Rc::ptr_eq(&connection.component, &jointref));
    }
}
impl<T> MultibodyTrait for Body<T>
where
    T: SimValue,
{
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
