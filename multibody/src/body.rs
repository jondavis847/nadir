use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;

use super::{
    base::Base,
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyTrait,
};

pub type BodyRef<T> = Rc<RefCell<Bodies<T>>>;

#[derive(Clone, Debug)]
pub enum Bodies<T>
where
    T: SimValue,
{
    Base(Base),
    Body(Body<T>),
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
    MassPropertiesErrors(MassPropertiesErrors),    
}

#[derive(Debug, Clone)]
pub struct Body<T>
where
    T: SimValue,
{
    mass_properties: MassProperties<T>,
    name: String,
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
            mass_properties: mass_properties,
            name: name.to_string(),
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
