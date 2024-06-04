use crate::ui::dummies::{DummyBody, DummyComponent, DummyErrors, DummyTrait};

use super::{
    mass_properties::{MassProperties, MassPropertiesErrors},
    MultibodyMeta, MultibodyTrait,
};
use uuid::Uuid;

#[derive(Debug, Clone, Copy)]
pub enum BodyField {
    Name,
    Mass,
    Cmx,
    Cmy,
    Cmz,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
}

#[derive(Debug, Clone)]
pub struct Body {
    mass_properties: MassProperties,
    meta: MultibodyMeta,    
}

#[derive(Debug, Clone, Copy)]
pub enum BodyErrors {
    DummyErrors(DummyErrors),
    MassPropertiesErrors(MassPropertiesErrors),
}

impl Body {
    pub fn from_dummy(
        component_id: Uuid,        
        dummy: &DummyBody,
        node_id: Uuid,        
    ) -> Result<Self, BodyErrors> {        

        let name = dummy.get_name();

        if name.is_empty() {
            return Err(BodyErrors::DummyErrors(DummyErrors::NameIsEmpty))
        }        

        let meta = MultibodyMeta::new(component_id, dummy.get_id(), name, node_id);
        let mass_properties = match MassProperties::new(
            dummy.mass.parse().unwrap_or(1.0),
            dummy.cmx.parse().unwrap_or(0.0),
            dummy.cmy.parse().unwrap_or(0.0),
            dummy.cmz.parse().unwrap_or(0.0),
            dummy.ixx.parse().unwrap_or(1.0),
            dummy.iyy.parse().unwrap_or(1.0),
            dummy.izz.parse().unwrap_or(1.0),
            dummy.ixy.parse().unwrap_or(0.0),
            dummy.ixz.parse().unwrap_or(0.0),
            dummy.iyz.parse().unwrap_or(0.0),
        ) {
            Ok(mass_properties) => mass_properties,
            Err(error) => return Err(BodyErrors::MassPropertiesErrors(error)),
        };

        Ok(Self {
            meta,
            mass_properties,
        })
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

    fn set_ixx(&mut self, ixx: f64) -> Result<(),BodyErrors> {
        match self.mass_properties.set_ixx(ixx) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(())
        }
    }

    fn set_ixy(&mut self, ixy: f64) {
        self.mass_properties.set_ixy(ixy);
    }

    fn set_ixz(&mut self, ixz: f64) {
        self.mass_properties.set_ixz(ixz);
    }

    fn set_iyy(&mut self, iyy: f64) -> Result<(),BodyErrors> {
        match self.mass_properties.set_iyy(iyy) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(())
        }
    }

    fn set_iyz(&mut self, iyz: f64) -> Result<(),BodyErrors> {
        match self.mass_properties.set_ixx(iyz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(())
        }
    }

    fn set_izz(&mut self, izz: f64) -> Result<(),BodyErrors> {
        match self.mass_properties.set_izz(izz) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(())
        }
    }

    fn set_mass(&mut self, mass: f64) -> Result<(),BodyErrors> {
        match self.mass_properties.set_mass(mass) {
            Err(error) => Err(BodyErrors::MassPropertiesErrors(error)),
            Ok(_) => Ok(())
        }
    }


}

impl MultibodyTrait for Body {
    fn connect_from(&mut self, id: Uuid) {
        self.meta.from_id = Some(id);
    }

    fn connect_to(&mut self, id: Uuid) {
        self.meta.to_id.push(id);
    }
    fn delete_from(&mut self) {
        self.meta.from_id = None;
    }
    fn delete_to(&mut self, id: Uuid) {
        self.meta.to_id.retain(|&to_id| to_id != id);
    }

    fn get_component_id(&self) -> Uuid {
        self.meta.component_id
    }

    fn get_dummy_id(&self) -> Uuid {
        self.meta.dummy_id
    }

    fn get_from_id(&self) -> Option<Uuid> {
        self.meta.from_id
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn get_node_id(&self) -> Uuid {
        self.meta.node_id
    }

    fn get_to_id(&self) -> &Vec<Uuid> {
        &self.meta.to_id
    }

    //TODO: handle the errors instead of unwrap
    fn inherit_from(&mut self, dummy: &DummyComponent) {
        match dummy {
            DummyComponent::Body(dummy_body) => {
                self.set_name(dummy.get_name()); 
                self.set_mass(dummy_body.mass.parse().unwrap_or(1.0)).unwrap();
                self.set_cmx(dummy_body.cmx.parse().unwrap_or(0.0));
                self.set_cmy(dummy_body.cmy.parse().unwrap_or(0.0));
                self.set_cmz(dummy_body.cmz.parse().unwrap_or(0.0));
                self.set_ixx(dummy_body.ixx.parse().unwrap_or(1.0)).unwrap();
                self.set_iyy(dummy_body.iyy.parse().unwrap_or(1.0)).unwrap();
                self.set_izz(dummy_body.izz.parse().unwrap_or(1.0)).unwrap();
                self.set_ixy(dummy_body.ixy.parse().unwrap_or(0.0));
                self.set_ixz(dummy_body.ixz.parse().unwrap_or(0.0));
                self.set_iyz(dummy_body.iyz.parse().unwrap_or(0.0)).unwrap();
            }
            _ => {} //error! must be a body
        }
    }

    fn set_component_id(&mut self, id: Uuid) {
        self.meta.component_id = id;
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }

    fn set_node_id(&mut self, id: Uuid) {
        self.meta.node_id = id;
    }

    fn set_system_id(&mut self, id: usize) {
        self.meta.system_id = Some(id);
    }
}
