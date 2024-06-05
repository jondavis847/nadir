use sim_value::SimValue;
use transforms::Transform;
use uuid::Uuid;

use super::{
    base::Base,
    connection::ConnectionErrors,
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyMeta, MultibodyTrait,
};

pub trait BodyTrait<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors>;

    fn connect_outer_joint(
        &mut self,
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors>;

    fn delete_inner_joint(&mut self);
    fn delete_outer_joint(&mut self, id: Uuid);
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
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        match self {
            Bodies::Base(base) => base.connect_inner_joint(id, transform),
            Bodies::Body(body) => body.connect_inner_joint(id, transform),
        }
    }

    fn connect_outer_joint(
        &mut self,
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        match self {
            Bodies::Base(base) => base.connect_outer_joint(id, transform),
            Bodies::Body(body) => body.connect_outer_joint(id, transform),
        }
    }

    fn delete_inner_joint(&mut self) {
        match self {
            Bodies::Base(base) => base.delete_inner_joint(),
            Bodies::Body(body) => body.delete_inner_joint(),
        }
    }
    fn delete_outer_joint(&mut self, id: Uuid) {
        match self {
            Bodies::Base(base) => base.delete_outer_joint(id),
            Bodies::Body(body) => body.delete_outer_joint(id),
        }
    }
}

impl<T> MultibodyTrait for Bodies<T>
where
    T: SimValue,
{
    fn get_id(&self) -> Uuid {
        match self {
            Bodies::Base(base) => base.get_id(),
            Bodies::Body(body) => body.get_id(),
        }
    }

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
    MassPropertiesErrors(MassPropertiesErrors),
}

#[derive(Debug, Clone)]
pub struct Body<T>
where
    T: SimValue,
{
    mass_properties: MassProperties<T>,
    meta: MultibodyMeta,
    inner_joint: Option<BodyConnection<T>>,
    outer_joints: Vec<BodyConnection<T>>,
}

impl<T> Body<T>
where
    T: SimValue,
{
    pub fn new(name: &str, mass_properties: MassProperties<T>) -> Self {
        Self {
            mass_properties: mass_properties,
            meta: MultibodyMeta::new(name),
            inner_joint: None,
            outer_joints: Vec::new(),
        }
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
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        match self.inner_joint {
            Some(_) => return Err(ConnectionErrors::BodyInnerAlreadyExists),
            None => {
                let connection = BodyConnection::new(id, transform);
                self.inner_joint = Some(connection);
                Ok(())
            }
        }
    }

    fn connect_outer_joint(
        &mut self,
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        if !self
            .outer_joints
            .iter()
            .any(|connection| connection.get_joint_id() == id)
        {
            let connection = BodyConnection::new(id, transform);
            self.outer_joints.push(connection);
        }
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        self.inner_joint = None;
    }

    fn delete_outer_joint(&mut self, id: Uuid) {
        self.outer_joints
            .retain(|connection| connection.get_joint_id() != id)
    }
}

impl<T> MultibodyTrait for Body<T>
where
    T: SimValue,
{
    fn get_id(&self) -> Uuid {
        self.meta.id
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyConnection<T>
where
    T: SimValue,
{
    joint_id: Uuid,
    transform: Transform<T>,
}

impl<T> BodyConnection<T>
where
    T: SimValue,
{
    pub fn new(joint_id: Uuid, transform: Transform<T>) -> Self {
        Self {
            joint_id,
            transform,
        }
    }

    pub fn get_joint_id(&self) -> Uuid {
        self.joint_id
    }
}
