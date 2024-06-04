use uuid::Uuid;
use sim_value::SimValue;

use super::{
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyMeta, MultibodyTrait,
};

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
}

impl<T> Body<T>
where
    T: SimValue,
{
    pub fn new(mass_properties: MassProperties<T>, meta: MultibodyMeta) -> Self {
        Self {
            mass_properties,
            meta,
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

impl<T> MultibodyTrait for Body<T>
where
    T: SimValue,
{
    fn connect_inner(&mut self, id: Uuid) {
        self.meta.id_inner = Some(id);
    }

    fn connect_outer(&mut self, id: Uuid) {
        self.meta.id_outer.push(id);
    }
    fn delete_inner(&mut self) {
        self.meta.id_inner = None;
    }
    fn delete_outer(&mut self, id: Uuid) {
        self.meta.id_outer.retain(|&outer_id| outer_id != id);
    }

    fn get_id(&self) -> Uuid {
        self.meta.id
    }

    fn get_inner_id(&self) -> Option<Uuid> {
        self.meta.id_inner
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn get_outer_id(&self) -> &Vec<Uuid> {
        &self.meta.id_outer
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
